"""arm_sim node: the /arm_sim/integration_step checkpoint service, the
/arm_sim/set_params, /arm_sim/set_integrator, /arm_sim/pause, and
/arm_sim/reset services, the /joint_trajectory setpoint subscriber, the
/pid_controller/enable and /pid_controller/set_gains services, and the live
n-link arm's physics loop (physics_loop) and /joint_states publisher
(publish_loop).

/joint_trajectory stores the held setpoint (state.setpoint_pos /
state.setpoint_vel), which physics_loop feeds to the owner's
pid.PIDController.update() once per tick while PID is enabled (disabled by
default; tau is the zero vector while disabled and the integral does not
advance). /arm_sim/reset snaps the plant, zeroes sim_time, and calls
state.reset_pid_controller(), which resets the stored setpoint to the reset
pose and clears the controller's integral (gains are kept).

Assumptions documented here since the spec doesn't give literal defaults for
these (only gravity=9.81 is spec-stated):
- masses/lengths default to 1.0 per link
- integrator defaults to "euler" with timestep=0.01
- set_integrator's two fields are validated/applied independently (matching
  the family-wide "every field optional and independently applied"
  convention stated up front in the spec), even though the spec's own
  set_integrator paragraph phrases rejection singularly ("reject it") rather
  than "independently" the way set_params does explicitly
- reset pose is q = qdot = [0.0]*links (see agent-notes/PLAN.md, "Open
  questions and assumptions")
- default PID gains kp=200, ki=60, kd=10 per joint and integral_limit=10.0
  (spec: "no single correct set of gains"). Checked offline to converge
  (< 0.05 rad within ~7.5 s) on 2- and 3-link unit-mass/unit-length arms
  under all four integrators at the default timestep 0.01. A larger kd
  (e.g. 40) makes the light distal-link mode too stiff for explicit
  integration at dt=0.01 and diverges.

physics_loop/publish_loop wiring (agent-notes/PLAN.md, "Proposed
architecture"): physics_loop paces itself by the *current*
state.integrator_timestep each iteration and, while not paused, steps
(state.q, state.qdot) forward one tick via integrators.METHODS[...], with
tau computed once per tick by the PID controller while enabled (zero while
disabled) and held constant across the integrator's substages. publish_loop is entirely independent of physics_loop: it publishes
/joint_states at a fixed reference 60 Hz regardless of `paused` or the
current integrator_timestep. Both loops read every relevant _ArmSimState
field fresh on each iteration (never cached), so a mid-run
/arm_sim/set_params or /arm_sim/set_integrator call takes effect on the very
next tick.
"""
from __future__ import annotations

import asyncio
import math
from typing import Any

import arm_dynamics
import expr
import integrators
import pid
from gateway import log
from registry import Registry

PUBLISH_PERIOD_S = 1 / 60

DEFAULT_KP = 200.0
DEFAULT_KI = 60.0
DEFAULT_KD = 10.0
INTEGRAL_LIMIT = 10.0


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _integration_step(args: Any) -> tuple[bool, dict, str]:
    """/arm_sim/integration_step: standalone 1-DOF numerical integration,
    fully decoupled from any live arm state. See spec/PROJECT2_PENDULARM.md,
    "Project checkpoint -- Numerical Integration Step Service".
    """
    args = args or {}

    function_str = args.get("function")
    if not isinstance(function_str, str):
        return False, {}, "missing or invalid 'function'"
    try:
        f = expr.parse_function(function_str)
    except expr.ExpressionError as exc:
        return False, {}, f"invalid function: {exc}"

    x0 = args.get("x0")
    if not _is_number(x0):
        return False, {}, "missing or invalid 'x0'"
    xdot0 = args.get("xdot0", 0.0)
    if not _is_number(xdot0):
        return False, {}, "invalid 'xdot0'"

    dt = args.get("dt")
    if not _is_number(dt) or dt <= 0:
        return False, {}, "'dt' must be a positive number"

    steps = args.get("steps")
    if not isinstance(steps, int) or isinstance(steps, bool) or steps <= 0:
        return False, {}, "'steps' must be a positive integer"

    integrator_name = args.get("integrator")
    method = integrators.METHODS.get(integrator_name) if isinstance(integrator_name, str) else None
    if method is None:
        return False, {}, f"unknown integrator {integrator_name!r}"

    def accel_fn(t: float, q: list[float], qdot: list[float]) -> list[float]:
        return [f(t)]

    q, qdot, t = [float(x0)], [float(xdot0)], 0.0
    times, positions, velocities = [t], [q[0]], [qdot[0]]
    for _ in range(steps):
        q, qdot = method(accel_fn, t, q, qdot, dt)
        t += dt
        times.append(t)
        positions.append(q[0])
        velocities.append(qdot[0])

    return True, {"times": times, "positions": positions, "velocities": velocities}, ""


class _ArmSimState:
    """Holds the parameter/mode state for /arm_sim/set_params,
    /arm_sim/set_integrator, /arm_sim/pause, and /arm_sim/reset, plus the
    live arm state and the /joint_trajectory setpoint. One instance per
    runtime.
    """

    def __init__(self, links: int) -> None:
        self.links = links
        self.gravity = 9.81
        self.masses = [1.0] * links
        self.lengths = [1.0] * links
        self.integrator_method = "euler"
        self.integrator_timestep = 0.01
        self.paused = False
        # Live arm state: reset pose is q = qdot = [0]*links (see module
        # docstring); sim_time is the simulation clock physics_loop advances
        # and /joint_states' header.stamp reports.
        self.q = [0.0] * links
        self.qdot = [0.0] * links
        self.sim_time = 0.0
        # Held PID setpoint, set by /joint_trajectory (last point only) and
        # by /arm_sim/reset. Starts at the reset pose so an enabled
        # controller with no trajectory yet received just holds home.
        self.setpoint_pos = [0.0] * links
        self.setpoint_vel = [0.0] * links
        # PID: starts disabled (spec). `tau` is the effort physics_loop last
        # applied (zero while disabled), which /joint_states reports.
        self.pid_enabled = False
        self.controller = pid.PIDController(
            links, [DEFAULT_KP] * links, [DEFAULT_KI] * links, [DEFAULT_KD] * links,
            INTEGRAL_LIMIT)
        self.tau = [0.0] * links

    def set_params(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        errors: list[str] = []

        if "gravity" in args:
            gravity = args["gravity"]
            if _is_number(gravity) and gravity >= 0:
                self.gravity = float(gravity)
            else:
                errors.append("'gravity' must be a number >= 0")

        if "masses" in args:
            masses = args["masses"]
            if (isinstance(masses, list) and len(masses) == self.links
                    and all(_is_number(v) and v > 0 for v in masses)):
                self.masses = [float(v) for v in masses]
            else:
                errors.append(f"'masses' must be a list of {self.links} positive numbers")

        if "lengths" in args:
            lengths = args["lengths"]
            if (isinstance(lengths, list) and len(lengths) == self.links
                    and all(_is_number(v) and v > 0 for v in lengths)):
                self.lengths = [float(v) for v in lengths]
            else:
                errors.append(f"'lengths' must be a list of {self.links} positive numbers")

        values = {"gravity": self.gravity, "masses": list(self.masses), "lengths": list(self.lengths)}
        return (not errors), values, "; ".join(errors)

    def set_integrator(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        errors: list[str] = []

        if "method" in args:
            method = args["method"]
            if isinstance(method, str) and method in integrators.METHODS:
                self.integrator_method = method
            else:
                errors.append(f"unknown integrator method {method!r}")

        if "timestep" in args:
            timestep = args["timestep"]
            if _is_number(timestep) and timestep > 0:
                self.integrator_timestep = float(timestep)
            else:
                errors.append("'timestep' must be a positive number")

        values = {"method": self.integrator_method, "timestep": self.integrator_timestep}
        return (not errors), values, "; ".join(errors)

    def pause(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        if "data" in args:
            data = args["data"]
            if not isinstance(data, bool):
                return False, {"data": self.paused}, "'data' must be a boolean"
            self.paused = data
        return True, {"data": self.paused}, ""

    def reset_pid_controller(self) -> None:
        """Reset the controller "to match" a freshly-reset plant (spec,
        /arm_sim/reset): the held setpoint goes to the reset pose so the arm
        isn't pulled back toward an old target, and the accumulated integral
        is cleared (gains and the enabled flag are kept).
        """
        self.setpoint_pos = [0.0] * self.links
        self.setpoint_vel = [0.0] * self.links
        self.controller.reset()
        self.tau = [0.0] * self.links

    def pid_enable(self, args: Any) -> tuple[bool, dict, str]:
        """/pid_controller/enable: `{"data": <bool>}`; `{}` queries. Enabling
        from disabled clears the integral (whatever accumulated "while
        nothing was listening"); enabling while already enabled is a no-op.
        Echoes the current enabled flag, like /arm_sim/pause.
        """
        args = args or {}
        if not isinstance(args, dict):
            return False, {"data": self.pid_enabled}, "request must be an object"
        if "data" in args:
            data = args["data"]
            if not isinstance(data, bool):
                return False, {"data": self.pid_enabled}, "'data' must be a boolean"
            if data and not self.pid_enabled:
                self.controller.reset()
            self.pid_enabled = data
            if not data:
                self.tau = [0.0] * self.links
        return True, {"data": self.pid_enabled}, ""

    def _gains_values(self) -> dict:
        c = self.controller
        return {"kp": [float(v) for v in c.kp], "ki": [float(v) for v in c.ki],
                "kd": [float(v) for v in c.kd]}

    def set_gains(self, args: Any) -> tuple[bool, dict, str]:
        """/pid_controller/set_gains: optional `kp`/`ki`/`kd` arrays, each
        validated (exactly `links` finite numbers, each >= 0) and applied
        independently; `{}` queries. Always echoes the current gains. Never
        touches the integral (spec).
        """
        args = args or {}
        if not isinstance(args, dict):
            return False, self._gains_values(), "request must be an object"
        errors: list[str] = []
        for name in ("kp", "ki", "kd"):
            if name not in args:
                continue
            value = args[name]
            if (isinstance(value, list) and len(value) == self.links
                    and all(_is_number(v) and math.isfinite(v) and v >= 0 for v in value)):
                setattr(self.controller, name, [float(v) for v in value])
            else:
                errors.append(f"'{name}' must be a list of {self.links} non-negative numbers")
        return (not errors), self._gains_values(), "; ".join(errors)

    def reset(self, args: Any) -> tuple[bool, dict, str]:
        """/arm_sim/reset: `{}` request. Snaps (q, qdot) to the reset pose,
        zeroes sim_time, resets the PID controller, and echoes the resulting
        pose. `paused` is left as-is (the spec doesn't tie reset to it).

        No awaits here, so on the single asyncio loop this can never
        interleave with a half-finished physics_loop tick.
        """
        if args is not None and not isinstance(args, dict):
            return (False, {"position": list(self.q), "velocity": list(self.qdot)},
                    "request must be an object ({})")
        self.q = [0.0] * self.links
        self.qdot = [0.0] * self.links
        self.sim_time = 0.0
        self.reset_pid_controller()
        return True, {"position": list(self.q), "velocity": list(self.qdot)}, ""

    def _setpoint_array(self, value: Any) -> list[float]:
        """A positions/velocities array from a trajectory point: used as-is
        if it's exactly `links` finite numbers, otherwise all-zero (spec:
        "defaulting to all-zero if omitted or the wrong length"; non-numeric
        or non-finite entries are treated the same way).
        """
        if (isinstance(value, list) and len(value) == self.links
                and all(_is_number(v) and math.isfinite(v) for v in value)):
            return [float(v) for v in value]
        return [0.0] * self.links

    def set_trajectory(self, msg: Any) -> bool:
        """Apply a /joint_trajectory message: only the *last* entry of
        `points` becomes the held setpoint, the instant it arrives (step
        servo, not a trajectory follower). A message with no usable last
        point (not an object, `points` missing/empty/not a list, last entry
        not an object) is ignored and the previous setpoint is kept --
        topics have no response channel to report a rejection on.
        `joint_names` is not used for reordering; arrays are taken in joint
        order. Returns whether the message was applied (for tests).
        """
        if not isinstance(msg, dict):
            return False
        points = msg.get("points")
        if not isinstance(points, list) or not points or not isinstance(points[-1], dict):
            return False
        last = points[-1]
        self.setpoint_pos = self._setpoint_array(last.get("positions"))
        self.setpoint_vel = self._setpoint_array(last.get("velocities"))
        return True


class _TrajectorySubscriber:
    """In-process Connection stand-in subscribed to /joint_trajectory:
    Registry.publish() hands it the full `{"op": "publish", ...}` envelope,
    and it forwards the inner `msg` to state.set_trajectory().
    """

    def __init__(self, state: _ArmSimState) -> None:
        self._state = state

    def send(self, message: dict) -> None:
        self._state.set_trajectory(message.get("msg"))


def _joint_names(links: int) -> list[str]:
    """1-indexed joint names ("joint1", "joint2", ...), matching the spec's
    own /joint_trajectory example verbatim. Shared by publish_loop now and
    reusable by a future /joint_trajectory subscriber without redefinition.
    """
    return [f"joint{i}" for i in range(1, links + 1)]


async def physics_loop(state: _ArmSimState) -> None:
    """Infinite loop advancing the live arm's (q, qdot) one integration step
    per iteration, paced by the *current* state.integrator_timestep.

    While state.paused, the integration step (and state.sim_time) is simply
    skipped -- the loop keeps sleeping/looping so it resumes promptly once
    unpaused, rather than exiting or blocking.

    tau: while state.pid_enabled, computed once per tick by
    state.controller.update() from the measured (q, qdot) at the start of the
    tick and this tick's dt, then held constant across the integrator's
    substages (a zero-order-hold control input); while disabled, the zero
    vector, and update() is not called so the integral does not advance.
    state.tau records the effort actually applied, for /joint_states.

    Any exception from a tick (e.g. a bug in forward_dynamics or the
    controller) is caught, logged once
    (then throttled to avoid flooding stderr on every subsequent tick), and
    the loop keeps running so the rest of the runtime (services,
    publish_loop) stays fully responsive regardless.
    """
    logged_failure = False
    while True:
        dt = state.integrator_timestep
        await asyncio.sleep(dt)

        if state.paused:
            continue

        try:
            gravity, masses, lengths = state.gravity, state.masses, state.lengths
            links = state.links
            method = integrators.METHODS[state.integrator_method]

            if state.pid_enabled:
                tau = [float(v) for v in state.controller.update(
                    state.setpoint_pos, state.setpoint_vel, state.q, state.qdot, dt)]
            else:
                tau = [0.0] * links

            def _accel_fn(t: float, q: list[float], qdot: list[float]) -> list[float]:
                return arm_dynamics.forward_dynamics(q, qdot, tau, gravity, masses, lengths)

            q_next, qdot_next = method(_accel_fn, state.sim_time, state.q, state.qdot, dt)
            state.q, state.qdot = q_next, qdot_next
            state.tau = tau
            state.sim_time += dt
        except Exception as exc:
            if not logged_failure:
                log(f"physics_loop: tick failed, arm frozen until this is resolved ({exc!r}); "
                    "further failures on this tick will not be individually logged")
                logged_failure = True


async def publish_loop(registry: Registry, state: _ArmSimState) -> None:
    """Infinite loop publishing /joint_states at a fixed reference 60 Hz,
    entirely independent of `state.paused` and `state.integrator_timestep`
    -- a paused sim still publishes its frozen snapshot, and the publish
    rate never tracks whatever the physics timestep currently is.
    """
    names = _joint_names(state.links)
    while True:
        await asyncio.sleep(PUBLISH_PERIOD_S)

        stamp_sec = int(state.sim_time)
        stamp_nanosec = int(round((state.sim_time - stamp_sec) * 1e9))
        msg = {
            "header": {"stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec}, "frame_id": ""},
            "name": names,
            "position": list(state.q),
            "velocity": list(state.qdot),
            "effort": list(state.tau),
        }
        registry.publish("/joint_states", msg)


def register(registry: Registry, links: int = 2) -> _ArmSimState:
    registry.register_handler("/arm_sim/integration_step", _integration_step)

    state = _ArmSimState(links)
    registry.register_handler("/arm_sim/set_params", state.set_params)
    registry.register_handler("/arm_sim/set_integrator", state.set_integrator)
    registry.register_handler("/arm_sim/pause", state.pause)
    registry.register_handler("/arm_sim/reset", state.reset)
    registry.register_handler("/pid_controller/enable", state.pid_enable)
    registry.register_handler("/pid_controller/set_gains", state.set_gains)
    registry.subscribe(_TrajectorySubscriber(state), "/joint_trajectory")
    return state
