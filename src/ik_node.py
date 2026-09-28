"""IK node: /ik/solve, the /ik_action/* goal action (send_goal, cancel_goal,
feedback, result), and the /ik_trial/* timed trial harness (start, skip,
stop, status). See spec/PROJECT2_PENDULARM.md, "Inverse kinematics".

All three layers talk to the rest of the runtime only through the Registry,
the same way an external client would:
- /ik/solve fetches the arm's *current* link lengths on every call through
  /arm_sim/set_params' empty-request query, then calls the owner's
  kinematics.inverse_kinematics(). kinematics.Unreachable becomes
  result:false with str(e) as the status. Any other exception from the math
  module (e.g. NotImplementedError while it's still a stub) is also turned
  into a rejection, never a crash or a hang.
- The action (_IkAction) solves through /ik/solve, commands the solution by
  publishing /joint_trajectory (the existing setpoint path; whether the PID
  actually drives the arm there is up to /pid_controller/enable -- this node
  never enables it), and tracks convergence from /joint_states. Each
  /joint_states message is one control tick: its header.stamp is the
  simulation clock (so dwell/elapsed don't depend on real-time factor, and
  stop advancing while paused) and its position array is the measured pose.
- The trial harness (_IkTrial) is a pure /ik_action client: it calls
  /ik_action/send_goal and /ik_action/cancel_goal and subscribes to
  /ik_action/feedback and /ik_action/result. It never reads /joint_states or
  publishes /joint_trajectory. It uses /arm_sim/set_params' query for link
  lengths when sampling targets.

Implementation-defined choices (spec: "What's explicitly up to you"):
- default epsilon 0.05 m, default success_hold 0.5 s (sim time), default
  trial duration 60 s.
- distance_remaining / final_distance / epsilon are end-effector *position*
  distances (m); a 3-link goal's phi is honored by the IK solution but not
  part of the epsilon check.
- goal ids are "goal-<n>", counting from 1 per runtime.
- the trial's own elapsed/duration clock is wall time (time.monotonic),
  since the harness may not read /joint_states' sim clock itself.
- trial targets: uniform random joint angles in [-pi, pi) mapped through
  kinematics.forward_kinematics (always reachable by construction; a 3-link
  target also carries the sampled phi, so the wrist rule holds too). Up to
  SAMPLE_ATTEMPTS draws per target.
- /ik_trial/start's duration/epsilon/success_hold follow set_params' partial
  override convention: an omitted field keeps its current (initially
  default) value; a supplied value persists for later starts. Any invalid
  field rejects the whole start without touching the running trial.
"""
from __future__ import annotations

import asyncio
import math
import random
import time
from typing import Any, Callable

import kinematics
from gateway import log
from registry import Registry

DEFAULT_EPSILON = 0.05
DEFAULT_SUCCESS_HOLD = 0.5
DEFAULT_TRIAL_DURATION = 60.0
TRIAL_STATUS_PERIOD_S = 0.1
SAMPLE_ATTEMPTS = 50


def _is_finite_number(value: Any) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _current_lengths(registry: Registry) -> list[float]:
    """Link lengths, fetched fresh through /arm_sim/set_params' `{}` query."""
    _, values, _ = registry.call_service_sync("/arm_sim/set_params", {})
    return [float(v) for v in values["lengths"]]


def _stamp_seconds(msg: dict) -> float:
    stamp = msg["header"]["stamp"]
    return float(stamp["sec"]) + float(stamp["nanosec"]) * 1e-9


class _Subscriber:
    """In-process Connection stand-in forwarding a topic's inner `msg` to a
    callback. Exceptions are logged (once) and swallowed so a bad message
    can never propagate back into the publisher (e.g. arm_sim's
    publish_loop).
    """

    def __init__(self, callback: Callable[[Any], None], name: str) -> None:
        self._callback = callback
        self._name = name
        self._logged = False

    def send(self, message: dict) -> None:
        try:
            self._callback(message.get("msg"))
        except Exception as exc:
            if not self._logged:
                log(f"{self._name}: message handling failed ({exc!r}); further failures not logged")
                self._logged = True


def make_solve_handler(registry: Registry):
    """/ik/solve: `{"x", "y", "phi"?}` -> `{"positions": [n values]}`."""

    def solve(args: Any) -> tuple[bool, dict, str]:
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return False, {}, "request must be an object"
        x, y = args.get("x"), args.get("y")
        if not _is_finite_number(x):
            return False, {}, "missing or non-numeric 'x'"
        if not _is_finite_number(y):
            return False, {}, "missing or non-numeric 'y'"
        try:
            lengths = _current_lengths(registry)
        except Exception as exc:
            return False, {}, f"could not read current link lengths: {exc}"
        n = len(lengths)

        # phi only means something for a 3-link arm; a 2-link arm ignores it
        # entirely (even a malformed one). null counts as omitted.
        phi = args.get("phi")
        if n == 2 or phi is None:
            phi = None
        elif not _is_finite_number(phi):
            return False, {}, "'phi' must be a number"
        else:
            phi = float(phi)

        try:
            q = kinematics.inverse_kinematics(float(x), float(y), lengths, phi)
        except kinematics.Unreachable as exc:
            return False, {}, str(exc) or "target unreachable"
        except NotImplementedError:
            return False, {}, "inverse kinematics not implemented yet"
        except Exception as exc:
            return False, {}, f"inverse kinematics failed: {exc!r}"

        try:
            positions = [float(v) for v in q]
        except (TypeError, ValueError):
            positions = []
        if len(positions) != n or not all(math.isfinite(v) for v in positions):
            return False, {}, "inverse kinematics returned an invalid solution"
        return True, {"positions": positions}, ""

    return solve


class _IkAction:
    """/ik_action/*: at most one active goal. Ticked by /joint_states."""

    def __init__(self, registry: Registry) -> None:
        self._registry = registry
        self._next_id = 1
        self.goal: dict | None = None
        # Most recent measured pose (for a preempted goal's final_distance).
        self._last_q: list[float] | None = None

    def _distance(self, q: list[float], target: dict) -> float:
        x, y, _ = kinematics.forward_kinematics(q, _current_lengths(self._registry))
        return math.hypot(float(x) - target["x"], float(y) - target["y"])

    def _final_distance(self, goal: dict) -> float | None:
        if self._last_q is None:
            return goal["distance"]
        try:
            return self._distance(self._last_q, goal["target"])
        except Exception:
            return goal["distance"]

    def _conclude(self, outcome: str, final_distance: float | None) -> None:
        """Publish the active goal's single /ik_action/result. The goal is
        detached *before* publishing, so a subscriber reacting to the result
        (e.g. the trial submitting its next target) sees no active goal."""
        goal, self.goal = self.goal, None
        self._registry.publish("/ik_action/result", {
            "goal_id": goal["goal_id"], "outcome": outcome, "target": dict(goal["target"]),
            "final_distance": final_distance,
        })

    def send_goal(self, args: Any) -> tuple[bool, dict, str]:
        """Validate and solve first; only a fully accepted goal preempts the
        active one (an unreachable/malformed request leaves it untouched)."""
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return False, {}, "request must be an object"
        epsilon = args.get("epsilon", DEFAULT_EPSILON)
        if not _is_finite_number(epsilon) or epsilon <= 0:
            return False, {}, "'epsilon' must be a positive number"
        success_hold = args.get("success_hold", DEFAULT_SUCCESS_HOLD)
        if not _is_finite_number(success_hold) or success_hold < 0:
            return False, {}, "'success_hold' must be a number >= 0"

        solve_args = {k: args[k] for k in ("x", "y", "phi") if k in args}
        ok, values, status = self._registry.call_service_sync("/ik/solve", solve_args)
        if not ok:
            return False, {}, status or "target unreachable"
        positions = list(values["positions"])

        target = {"x": float(args["x"]), "y": float(args["y"])}
        if len(positions) == 3 and args.get("phi") is not None:
            target["phi"] = float(args["phi"])

        if self.goal is not None:
            self._conclude("preempted", self._final_distance(self.goal))

        goal_id = f"goal-{self._next_id}"
        self._next_id += 1
        self.goal = {
            "goal_id": goal_id, "target": target, "positions": positions,
            "epsilon": float(epsilon), "success_hold": float(success_hold),
            "elapsed": 0.0, "last_t": None, "inside": False, "dwell": 0.0,
            "distance": None,
        }
        self._registry.publish("/joint_trajectory", {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": [f"joint{i}" for i in range(1, len(positions) + 1)],
            "points": [{"positions": list(positions), "velocities": [0.0] * len(positions),
                        "accelerations": [], "time_from_start": {"sec": 0, "nanosec": 0}}],
        })
        return True, {"goal_id": goal_id}, ""

    def cancel_goal(self, args: Any) -> tuple[bool, dict, str]:
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return False, {}, "request must be an object"
        if self.goal is None:
            return False, {}, "no active goal"
        if "goal_id" in args and args["goal_id"] != self.goal["goal_id"]:
            return False, {}, f"goal_id {args['goal_id']!r} is not the active goal"
        goal_id = self.goal["goal_id"]
        self._conclude("preempted", self._final_distance(self.goal))
        return True, {"goal_id": goal_id}, ""

    def on_joint_states(self, msg: Any) -> None:
        """One control tick: dwell/elapsed advance by the sim-clock delta
        since the previous tick (clamped at 0, so a pause freezes them and an
        /arm_sim/reset clock jump backwards doesn't go negative)."""
        q = [float(v) for v in msg["position"]]
        t = _stamp_seconds(msg)
        self._last_q = q
        goal = self.goal
        if goal is None:
            return

        distance = self._distance(q, goal["target"])
        dt = 0.0 if goal["last_t"] is None else max(0.0, t - goal["last_t"])
        goal["last_t"] = t
        goal["elapsed"] += dt
        goal["distance"] = distance

        if distance <= goal["epsilon"]:
            # First tick inside the ball starts the dwell at 0 (so
            # success_hold == 0 is met right away); any tick outside resets it.
            goal["dwell"] = goal["dwell"] + dt if goal["inside"] else 0.0
            goal["inside"] = True
        else:
            goal["inside"] = False
            goal["dwell"] = 0.0

        self._registry.publish("/ik_action/feedback", {
            "goal_id": goal["goal_id"], "target": dict(goal["target"]),
            "positions": list(goal["positions"]), "distance_remaining": distance,
            "elapsed": goal["elapsed"],
        })
        # Tiny tolerance: dwell is a sum of float stamp deltas.
        if goal["inside"] and goal["dwell"] >= goal["success_hold"] - 1e-9:
            self._conclude("reached", distance)


class _IkTrial:
    """/ik_trial/*: a timed trial run purely through /ik_action/*."""

    def __init__(self, registry: Registry, clock: Callable[[], float] = time.monotonic,
                 rng: random.Random | None = None) -> None:
        self._registry = registry
        self._clock = clock
        self._rng = rng or random.Random()
        self.duration = DEFAULT_TRIAL_DURATION
        self.epsilon = DEFAULT_EPSILON
        self.success_hold = DEFAULT_SUCCESS_HOLD
        self.exists = False  # status is only published once a trial was started
        self.running = False
        self._start_time = 0.0
        self._elapsed = 0.0
        self.targets_reached = 0
        self.goal_id: str | None = None
        self.needs_target = False  # running, but the last submission failed
        self._clear_goal_view()

    def _clear_goal_view(self) -> None:
        self.target: dict | None = None
        self.error: float | None = None
        self.desired_positions: list[float] | None = None
        self.action_status = "idle"

    def elapsed(self) -> float:
        if self.running:
            self._elapsed = min(self._clock() - self._start_time, self.duration)
        return self._elapsed

    def status_msg(self) -> dict:
        return {
            "running": self.running, "elapsed": self.elapsed(), "duration": self.duration,
            "targets_reached": self.targets_reached,
            "target": dict(self.target) if self.target is not None else None,
            "error": self.error,
            "desired_positions": list(self.desired_positions) if self.desired_positions is not None else None,
            "action_status": self.action_status,
        }

    def _abandon_goal(self) -> None:
        """Cancel our in-flight goal (if any). goal_id is cleared first so
        the resulting "preempted" result is recognized as our own doing."""
        goal_id, self.goal_id = self.goal_id, None
        if goal_id is not None:
            self._registry.call_service_sync("/ik_action/cancel_goal", {"goal_id": goal_id})

    def _submit(self) -> tuple[bool, str]:
        """Sample a reachable target and send it as a goal."""
        self.goal_id = None
        self._clear_goal_view()
        status = "no reachable target could be sampled"
        for _ in range(SAMPLE_ATTEMPTS):
            try:
                lengths = _current_lengths(self._registry)
                q = [self._rng.uniform(-math.pi, math.pi) for _ in lengths]
                x, y, phi = kinematics.forward_kinematics(q, lengths)
                target = {"x": float(x), "y": float(y)}
                if len(lengths) == 3:
                    target["phi"] = float(phi)
            except Exception as exc:
                status = f"could not sample a target: {exc!r}"
                continue
            if not all(math.isfinite(v) for v in target.values()):
                continue
            ok, values, reason = self._registry.call_service_sync("/ik_action/send_goal", dict(
                target, epsilon=self.epsilon, success_hold=self.success_hold))
            if ok:
                self.goal_id = values["goal_id"]
                self.target = target
                self.action_status = "active"
                self.needs_target = False
                return True, ""
            status = f"sampled target rejected: {reason}"
        self.needs_target = True
        return False, status

    def start(self, args: Any) -> tuple[bool, dict, str]:
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return False, self.status_msg(), "request must be an object"
        updates = {}
        for name, positive in (("duration", True), ("epsilon", True), ("success_hold", False)):
            if name not in args:
                continue
            value = args[name]
            if not _is_finite_number(value) or value < 0 or (positive and value == 0):
                bound = "> 0" if positive else ">= 0"
                return False, self.status_msg(), f"'{name}' must be a number {bound}"
            updates[name] = float(value)
        for name, value in updates.items():
            setattr(self, name, value)

        self._abandon_goal()
        self.exists = True
        self.running = True
        self.targets_reached = 0
        self._start_time = self._clock()
        self._elapsed = 0.0
        ok, status = self._submit()
        if not ok:
            self.running = False
            self.needs_target = False
            return False, self.status_msg(), status
        return True, self.status_msg(), ""

    def skip(self, args: Any) -> tuple[bool, dict, str]:
        if not self.running:
            return False, self.status_msg(), "no trial is running"
        self._abandon_goal()
        ok, status = self._submit()
        return ok, self.status_msg(), status

    def _end(self) -> None:
        self.elapsed()  # freeze the clock at its final value
        self._abandon_goal()
        self.running = False
        self.needs_target = False
        self._clear_goal_view()

    def stop(self, args: Any) -> tuple[bool, dict, str]:
        if not self.running:
            return False, self.status_msg(), "no trial is running"
        self._end()
        return True, self.status_msg(), ""

    def on_feedback(self, msg: Any) -> None:
        if self.goal_id is None or msg.get("goal_id") != self.goal_id:
            return
        self.error = msg.get("distance_remaining")
        self.desired_positions = list(msg.get("positions") or []) or None
        self.action_status = "active"

    def on_result(self, msg: Any) -> None:
        if self.goal_id is None or msg.get("goal_id") != self.goal_id:
            return  # not ours, or one we abandoned ourselves
        self.error = msg.get("final_distance")
        self.goal_id = None
        if msg.get("outcome") == "reached":
            self.action_status = "reached"
            if self.running:
                self.targets_reached += 1
                self._submit()
        else:
            # Preempted by some other /ik_action client: record it and leave
            # the trial waiting (skip/stop/duration) rather than fight over
            # the single goal slot.
            self.action_status = "preempted"

    def tick(self) -> None:
        """Periodic work: end the trial when its duration elapses, retry a
        failed submission, and publish /ik_trial/status once a trial exists."""
        if self.running and self.elapsed() >= self.duration:
            self._end()
        elif self.running and self.needs_target:
            self._submit()
        if self.exists:
            self._registry.publish("/ik_trial/status", self.status_msg())


async def trial_status_loop(trial: _IkTrial) -> None:
    logged_failure = False
    while True:
        await asyncio.sleep(TRIAL_STATUS_PERIOD_S)
        try:
            trial.tick()
        except Exception as exc:
            if not logged_failure:
                log(f"trial_status_loop: tick failed ({exc!r}); further failures not logged")
                logged_failure = True


def register(registry: Registry) -> tuple[_IkAction, _IkTrial]:
    """Register /ik/*, /ik_action/*, /ik_trial/*. arm_sim_node.register()
    must already have run (/ik/solve queries /arm_sim/set_params)."""
    registry.register_handler("/ik/solve", make_solve_handler(registry))

    action = _IkAction(registry)
    registry.register_handler("/ik_action/send_goal", action.send_goal)
    registry.register_handler("/ik_action/cancel_goal", action.cancel_goal)
    registry.subscribe(_Subscriber(action.on_joint_states, "ik_action"), "/joint_states")

    trial = _IkTrial(registry)
    registry.register_handler("/ik_trial/start", trial.start)
    registry.register_handler("/ik_trial/skip", trial.skip)
    registry.register_handler("/ik_trial/stop", trial.stop)
    registry.subscribe(_Subscriber(trial.on_feedback, "ik_trial"), "/ik_action/feedback")
    registry.subscribe(_Subscriber(trial.on_result, "ik_trial"), "/ik_action/result")
    return action, trial
