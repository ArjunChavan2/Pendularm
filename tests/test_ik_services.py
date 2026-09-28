"""Tests for src/ik_node.py: /ik/solve, /ik_action/* (send_goal, cancel_goal,
feedback, result), and /ik_trial/* (start, skip, stop, status).

src/kinematics.py is the project owner's hand-written file. These tests
never depend on its state: every test patches kinematics.forward_kinematics
/ kinematics.inverse_kinematics with the TEST-LOCAL reference implementation
below (_ref_fk / _ref_ik), so they exercise only the service layer and stay
valid once the real implementation lands.

Action ticks are driven deterministically by publishing hand-built
/joint_states messages on the registry (explicit sim-time stamps and poses)
instead of running physics_loop/publish_loop; the trial harness gets an
injected fake clock and a seeded RNG.
"""
from __future__ import annotations

import asyncio
import math
import os
import random
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

import arm_sim_node  # noqa: E402
import ik_node  # noqa: E402
import kinematics  # noqa: E402
from gateway import Gateway  # noqa: E402
from registry import Registry  # noqa: E402

from client_helper import Client  # noqa: E402


# -- test-local reference kinematics (NOT the graded src/kinematics.py) -------

def _ref_fk(q, lengths):
    x = y = phi = 0.0
    for qi, li in zip(q, lengths):
        phi += qi
        x += li * math.cos(phi)
        y += li * math.sin(phi)
    return x, y, phi


def _ref_ik2(x, y, l1, l2):
    r = math.hypot(x, y)
    tol = 1e-9 * (l1 + l2)
    if r > l1 + l2 + tol:
        raise kinematics.Unreachable("target farther than full extension")
    if r < abs(l1 - l2) - tol:
        raise kinematics.Unreachable("target closer than |l1-l2|")
    c2 = max(-1.0, min(1.0, (r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2)))
    q2 = math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    return [q1, q2]


def _ref_ik(x, y, lengths, phi=None):
    if len(lengths) == 2:
        return _ref_ik2(x, y, *lengths)
    l1, l2, l3 = lengths
    if math.hypot(x, y) > l1 + l2 + l3 + 1e-9:
        raise kinematics.Unreachable("target farther than full extension")
    if phi is None:
        phi = math.atan2(y, x)
    try:
        q1, q2 = _ref_ik2(x - l3 * math.cos(phi), y - l3 * math.sin(phi), l1, l2)
    except kinematics.Unreachable:
        raise kinematics.Unreachable("wrist point outside links 1-2's reach") from None
    return [q1, q2, phi - q1 - q2]


class _Recorder:
    def __init__(self) -> None:
        self.messages: list[tuple[str, dict]] = []

    def send(self, message: dict) -> None:
        self.messages.append((message["topic"], message["msg"]))

    def on(self, topic: str) -> list[dict]:
        return [m for t, m in self.messages if t == topic]


def _joint_states(t: float, q: list[float]) -> dict:
    sec = int(t)
    return {"header": {"stamp": {"sec": sec, "nanosec": int(round((t - sec) * 1e9))}, "frame_id": ""},
            "name": [f"joint{i}" for i in range(1, len(q) + 1)],
            "position": list(q), "velocity": [0.0] * len(q), "effort": [0.0] * len(q)}


class _PatchedKinematics(unittest.TestCase):
    """Patches kinematics with the reference implementation and builds an
    in-process runtime (arm_sim_node + ik_node) with a recorder on every
    IK-related topic."""

    links = 2

    def setUp(self) -> None:
        for name, impl in (("forward_kinematics", _ref_fk), ("inverse_kinematics", _ref_ik)):
            patcher = mock.patch.object(kinematics, name, side_effect=impl)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.registry = Registry()
        self.state = arm_sim_node.register(self.registry, self.links)
        self.action, self.trial = ik_node.register(self.registry)
        self.now = 100.0
        self.trial._clock = lambda: self.now
        self.trial._rng = random.Random(1234)
        self.rec = _Recorder()
        for topic in ("/ik_action/feedback", "/ik_action/result", "/ik_trial/status", "/joint_trajectory"):
            self.registry.subscribe(self.rec, topic)

    def call(self, service: str, args):
        return self.registry.call_service_sync(service, args)

    def tick(self, t: float, q: list[float]) -> None:
        self.registry.publish("/joint_states", _joint_states(t, q))


class TestIkSolve(_PatchedKinematics):
    def test_round_trip_two_link(self):
        for x, y in ((1.2, 0.5), (0.0, 1.9), (-0.7, -0.4), (2.0, 0.0)):
            ok, values, status = self.call("/ik/solve", {"x": x, "y": y})
            self.assertTrue(ok, status)
            self.assertEqual(len(values["positions"]), 2)
            fx, fy, _ = _ref_fk(values["positions"], [1.0, 1.0])
            self.assertAlmostEqual(fx, x, places=6)
            self.assertAlmostEqual(fy, y, places=6)

    def test_rejects_missing_or_non_numeric_xy(self):
        for bad in ({}, None, {"x": 1.0}, {"y": 1.0}, {"x": "1", "y": 0.0}, {"x": 1.0, "y": None},
                    {"x": True, "y": 0.0}, {"x": float("nan"), "y": 0.0}, [1, 2], "x"):
            ok, values, status = self.call("/ik/solve", bad)
            self.assertFalse(ok, bad)
            self.assertTrue(status, bad)

    def test_rejects_too_far_and_too_close(self):
        ok, _, status = self.call("/ik/solve", {"x": 2.5, "y": 0.0})
        self.assertFalse(ok)
        self.assertIn("farther", status)
        self.call("/arm_sim/set_params", {"lengths": [1.0, 0.5]})
        ok, _, status = self.call("/ik/solve", {"x": 0.1, "y": 0.2})
        self.assertFalse(ok)
        self.assertIn("closer", status)

    def test_phi_ignored_for_two_link(self):
        ok, values, status = self.call("/ik/solve", {"x": 1.0, "y": 1.0, "phi": "junk"})
        self.assertTrue(ok, status)
        self.assertIsNone(kinematics.inverse_kinematics.call_args.args[3])

    def test_fresh_lengths_on_every_call(self):
        self.assertFalse(self.call("/ik/solve", {"x": 2.9, "y": 0.0})[0])
        self.call("/arm_sim/set_params", {"lengths": [2.0, 1.0]})
        ok, values, status = self.call("/ik/solve", {"x": 2.9, "y": 0.0})
        self.assertTrue(ok, status)
        self.assertAlmostEqual(_ref_fk(values["positions"], [2.0, 1.0])[0], 2.9, places=6)
        self.call("/arm_sim/set_params", {"lengths": [1.0, 1.0]})
        self.assertFalse(self.call("/ik/solve", {"x": 2.9, "y": 0.0})[0])

    def test_math_module_failures_reject_gracefully(self):
        for exc in (NotImplementedError("stub"), RuntimeError("boom"), ZeroDivisionError()):
            with mock.patch.object(kinematics, "inverse_kinematics", side_effect=exc):
                ok, values, status = self.call("/ik/solve", {"x": 1.0, "y": 0.0})
                self.assertFalse(ok)
                self.assertTrue(status)
                self.assertEqual(values, {})
        for garbage in ([float("nan"), 0.0], [0.0], None, [0.0, "a"]):
            with mock.patch.object(kinematics, "inverse_kinematics", return_value=garbage):
                ok, _, status = self.call("/ik/solve", {"x": 1.0, "y": 0.0})
                self.assertFalse(ok, garbage)
                self.assertTrue(status)


class TestIkSolveThreeLink(_PatchedKinematics):
    links = 3

    def test_round_trip_with_phi(self):
        for x, y, phi in ((1.5, 1.0, 0.3), (-1.0, 0.5, 2.0), (3.0, 0.0, 0.0)):
            ok, values, status = self.call("/ik/solve", {"x": x, "y": y, "phi": phi})
            self.assertTrue(ok, status)
            fx, fy, fphi = _ref_fk(values["positions"], [1.0, 1.0, 1.0])
            self.assertAlmostEqual(fx, x, places=6)
            self.assertAlmostEqual(fy, y, places=6)
            self.assertAlmostEqual(math.remainder(fphi - phi, 2 * math.pi), 0.0, places=6)

    def test_wrist_rule_rejection(self):
        # (2.5, 0) is reachable on its own, but phi = pi puts the wrist at
        # (3.5, 0), beyond links 1-2's reach of 2.
        self.assertTrue(self.call("/ik/solve", {"x": 2.5, "y": 0.0, "phi": 0.0})[0])
        ok, _, status = self.call("/ik/solve", {"x": 2.5, "y": 0.0, "phi": math.pi})
        self.assertFalse(ok)
        self.assertIn("wrist", status)

    def test_non_numeric_phi_rejected(self):
        ok, _, status = self.call("/ik/solve", {"x": 1.0, "y": 0.0, "phi": "up"})
        self.assertFalse(ok)
        self.assertIn("phi", status)
        self.assertTrue(self.call("/ik/solve", {"x": 1.0, "y": 0.0, "phi": None})[0])


class TestIkAction(_PatchedKinematics):
    def send(self, **args):
        ok, values, status = self.call("/ik_action/send_goal", args)
        self.assertTrue(ok, status)
        return values["goal_id"]

    def test_send_goal_commands_setpoint(self):
        goal_id = self.send(x=1.0, y=1.0)
        self.assertTrue(goal_id)
        traj = self.rec.on("/joint_trajectory")
        self.assertEqual(len(traj), 1)
        self.assertEqual(self.state.setpoint_pos, traj[0]["points"][-1]["positions"])
        fx, fy, _ = _ref_fk(self.state.setpoint_pos, [1.0, 1.0])
        self.assertAlmostEqual(fx, 1.0)
        self.assertAlmostEqual(fy, 1.0)
        self.assertFalse(self.state.pid_enabled)  # never auto-enabled

    def test_feedback_shape(self):
        goal_id = self.send(x=1.0, y=1.0)
        self.tick(0.0, [0.0, 0.0])
        self.tick(0.25, [0.0, 0.0])
        fb = self.rec.on("/ik_action/feedback")
        self.assertEqual(len(fb), 2)
        msg = fb[-1]
        self.assertEqual(set(msg), {"goal_id", "target", "positions", "distance_remaining", "elapsed"})
        self.assertEqual(msg["goal_id"], goal_id)
        self.assertEqual(msg["target"], {"x": 1.0, "y": 1.0})
        self.assertEqual(msg["positions"], self.state.setpoint_pos)
        self.assertAlmostEqual(msg["distance_remaining"], math.sqrt(2))  # FK([0,0]) = (2, 0)
        self.assertAlmostEqual(msg["elapsed"], 0.25)

    def test_no_feedback_without_goal(self):
        self.tick(0.0, [0.0, 0.0])
        self.assertEqual(self.rec.on("/ik_action/feedback"), [])

    def test_unreachable_rejected_and_active_goal_undisturbed(self):
        goal_id = self.send(x=1.0, y=1.0)
        for bad in ({"x": 5.0, "y": 0.0}, {"y": 1.0}, {"x": 1.0, "y": 1.0, "epsilon": 0},
                    {"x": 1.0, "y": 1.0, "epsilon": -1}, {"x": 1.0, "y": 1.0, "success_hold": -0.1},
                    {"x": 1.0, "y": 1.0, "success_hold": "1"}, "nope"):
            ok, _, status = self.call("/ik_action/send_goal", bad)
            self.assertFalse(ok, bad)
            self.assertTrue(status, bad)
        self.assertEqual(self.action.goal["goal_id"], goal_id)
        self.assertEqual(self.rec.on("/ik_action/result"), [])
        self.assertEqual(len(self.rec.on("/joint_trajectory")), 1)

    def test_preemption_publishes_preempted_exactly_once(self):
        first = self.send(x=1.0, y=1.0)
        self.tick(0.0, [0.0, 0.0])
        second = self.send(x=0.5, y=1.2)
        self.assertNotEqual(first, second)
        results = self.rec.on("/ik_action/result")
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["goal_id"], first)
        self.assertEqual(results[0]["outcome"], "preempted")
        self.assertEqual(results[0]["target"], {"x": 1.0, "y": 1.0})
        self.assertAlmostEqual(results[0]["final_distance"], math.sqrt(2))
        # Later ticks only concern the new goal.
        self.tick(0.1, [0.0, 0.0])
        self.assertEqual(len(self.rec.on("/ik_action/result")), 1)
        self.assertEqual(self.rec.on("/ik_action/feedback")[-1]["goal_id"], second)

    def test_cancel_rejections_without_side_effects(self):
        ok, _, status = self.call("/ik_action/cancel_goal", {})
        self.assertFalse(ok)
        self.assertTrue(status)
        goal_id = self.send(x=1.0, y=1.0)
        for bad in ({"goal_id": "goal-999"}, {"goal_id": None}, "x"):
            ok, _, status = self.call("/ik_action/cancel_goal", bad)
            self.assertFalse(ok, bad)
            self.assertTrue(status)
        self.assertEqual(self.action.goal["goal_id"], goal_id)
        self.assertEqual(self.rec.on("/ik_action/result"), [])

    def test_cancel_active_goal(self):
        for args in ({}, None, "id"):
            goal_id = self.send(x=1.0, y=1.0)
            req = {"goal_id": goal_id} if args == "id" else args
            ok, values, status = self.call("/ik_action/cancel_goal", req)
            self.assertTrue(ok, status)
            self.assertEqual(values, {"goal_id": goal_id})
            result = self.rec.on("/ik_action/result")[-1]
            self.assertEqual((result["goal_id"], result["outcome"]), (goal_id, "preempted"))
            self.assertIsNone(self.action.goal)
            self.assertFalse(self.call("/ik_action/cancel_goal", {})[0])
        self.assertEqual(len(self.rec.on("/ik_action/result")), 3)

    def test_dwell_resets_on_excursion(self):
        goal_id = self.send(x=1.0, y=1.0, epsilon=0.01, success_hold=0.5)
        at = list(self.state.setpoint_pos)
        away = [at[0] + 0.5, at[1]]
        for t, q in ((0.0, at), (0.3, at), (0.45, away), (0.5, at), (0.9, at)):
            self.tick(t, q)
            self.assertEqual(self.rec.on("/ik_action/result"), [], t)
        self.tick(1.0, at)  # 0.5 s continuously inside since t=0.5
        results = self.rec.on("/ik_action/result")
        self.assertEqual(len(results), 1)
        self.assertEqual((results[0]["goal_id"], results[0]["outcome"]), (goal_id, "reached"))
        self.assertLess(results[0]["final_distance"], 0.01)
        self.tick(1.1, at)
        self.assertEqual(len(self.rec.on("/ik_action/result")), 1)
        self.assertIsNone(self.action.goal)

    def test_success_hold_zero_is_immediate(self):
        self.send(x=1.0, y=1.0, success_hold=0)
        self.tick(3.0, list(self.state.setpoint_pos))
        results = self.rec.on("/ik_action/result")
        self.assertEqual([r["outcome"] for r in results], ["reached"])

    def test_dwell_uses_sim_clock_and_survives_backward_jump(self):
        self.send(x=1.0, y=1.0, success_hold=0.5)
        at = list(self.state.setpoint_pos)
        # Paused: stamp frozen -> no dwell progress however many ticks.
        for _ in range(10):
            self.tick(2.0, at)
        self.assertEqual(self.rec.on("/ik_action/result"), [])
        self.tick(0.0, at)  # /arm_sim/reset jumps the clock back
        self.assertGreaterEqual(self.rec.on("/ik_action/feedback")[-1]["elapsed"], 0.0)
        self.tick(0.5, at)
        self.assertEqual([r["outcome"] for r in self.rec.on("/ik_action/result")], ["reached"])

    def test_bad_joint_states_do_not_break_publisher(self):
        self.send(x=1.0, y=1.0)
        self.registry.publish("/joint_states", {"garbage": True})  # must not raise
        with mock.patch.object(kinematics, "forward_kinematics", side_effect=NotImplementedError):
            self.tick(0.0, [0.0, 0.0])  # must not raise
        self.assertIsNotNone(self.action.goal)


class TestIkActionThreeLink(_PatchedKinematics):
    links = 3

    def test_target_carries_phi_and_wrist_rule_applies(self):
        ok, values, status = self.call("/ik_action/send_goal", {"x": 1.5, "y": 1.0, "phi": 0.3})
        self.assertTrue(ok, status)
        self.tick(0.0, [0.0, 0.0, 0.0])
        self.assertEqual(self.rec.on("/ik_action/feedback")[-1]["target"], {"x": 1.5, "y": 1.0, "phi": 0.3})
        ok, _, _ = self.call("/ik_action/send_goal", {"x": 2.5, "y": 0.0, "phi": math.pi})
        self.assertFalse(ok)
        self.assertEqual(self.action.goal["goal_id"], values["goal_id"])


class TestIkTrial(_PatchedKinematics):
    def status(self) -> dict:
        self.trial.tick()
        return self.rec.on("/ik_trial/status")[-1]

    def test_no_status_and_rejections_before_any_trial(self):
        self.trial.tick()
        self.assertEqual(self.rec.on("/ik_trial/status"), [])
        for service in ("/ik_trial/skip", "/ik_trial/stop"):
            ok, _, status = self.call(service, {})
            self.assertFalse(ok)
            self.assertTrue(status)
        self.assertIsNone(self.action.goal)

    def test_start_submits_reachable_goal(self):
        ok, values, status = self.call("/ik_trial/start", {})
        self.assertTrue(ok, status)
        self.assertIsNotNone(self.action.goal)
        self.assertEqual(self.action.goal["goal_id"], self.trial.goal_id)
        self.assertEqual(self.action.goal["epsilon"], ik_node.DEFAULT_EPSILON)
        self.assertEqual(self.action.goal["success_hold"], ik_node.DEFAULT_SUCCESS_HOLD)
        st = self.status()
        self.assertEqual(set(st), {"running", "elapsed", "duration", "targets_reached", "target",
                                   "error", "desired_positions", "action_status"})
        self.assertTrue(st["running"])
        self.assertEqual(st["duration"], ik_node.DEFAULT_TRIAL_DURATION)
        self.assertEqual(st["targets_reached"], 0)
        self.assertEqual(st["action_status"], "active")
        self.assertEqual(set(st["target"]), {"x", "y"})
        # error / desired_positions come from the action's feedback.
        self.tick(0.0, [0.0, 0.0])
        st = self.status()
        self.assertEqual(st["desired_positions"], self.action.goal["positions"])
        self.assertIsInstance(st["error"], float)

    def test_start_overrides_apply_and_invalid_rejects(self):
        ok, _, status = self.call("/ik_trial/start", {"duration": 5, "epsilon": 0.2, "success_hold": 0})
        self.assertTrue(ok, status)
        self.assertEqual(self.action.goal["epsilon"], 0.2)
        self.assertEqual(self.action.goal["success_hold"], 0.0)
        goal_id = self.trial.goal_id
        for bad in ({"duration": 0}, {"epsilon": 0}, {"success_hold": -1}, {"duration": "10"}):
            ok, _, status = self.call("/ik_trial/start", bad)
            self.assertFalse(ok, bad)
            self.assertTrue(status)
        self.assertEqual(self.trial.goal_id, goal_id)  # running trial untouched
        self.assertTrue(self.trial.running)

    def test_reached_counts_and_submits_next(self):
        self.call("/ik_trial/start", {"success_hold": 0})
        first = self.trial.goal_id
        self.tick(0.0, list(self.action.goal["positions"]))
        self.assertEqual(self.trial.targets_reached, 1)
        self.assertNotEqual(self.trial.goal_id, first)
        self.assertEqual(self.action.goal["goal_id"], self.trial.goal_id)
        self.assertEqual(self.status()["action_status"], "active")

    def test_skip_abandons_without_counting(self):
        self.call("/ik_trial/start", {})
        first = self.trial.goal_id
        ok, _, status = self.call("/ik_trial/skip", {})
        self.assertTrue(ok, status)
        self.assertNotEqual(self.trial.goal_id, first)
        self.assertEqual(self.action.goal["goal_id"], self.trial.goal_id)
        results = self.rec.on("/ik_action/result")
        self.assertEqual([(r["goal_id"], r["outcome"]) for r in results], [(first, "preempted")])
        self.assertEqual(self.trial.targets_reached, 0)

    def test_stop_clears_goal_view(self):
        self.call("/ik_trial/start", {})
        self.tick(0.0, [0.0, 0.0])
        self.now += 2.0
        ok, _, status = self.call("/ik_trial/stop", {})
        self.assertTrue(ok, status)
        self.assertIsNone(self.action.goal)
        st = self.status()
        self.assertFalse(st["running"])
        self.assertEqual((st["target"], st["error"], st["desired_positions"], st["action_status"]),
                         (None, None, None, "idle"))
        self.assertAlmostEqual(st["elapsed"], 2.0)
        self.now += 5.0
        self.assertAlmostEqual(self.status()["elapsed"], 2.0)  # frozen after stop
        self.assertFalse(self.call("/ik_trial/stop", {})[0])
        self.assertFalse(self.call("/ik_trial/skip", {})[0])

    def test_duration_elapse_ends_trial(self):
        self.call("/ik_trial/start", {"duration": 3})
        self.now += 1.0
        self.assertTrue(self.status()["running"])
        self.now += 2.5
        st = self.status()
        self.assertFalse(st["running"])
        self.assertEqual(st["elapsed"], 3.0)
        self.assertEqual((st["target"], st["error"], st["desired_positions"], st["action_status"]),
                         (None, None, None, "idle"))
        self.assertIsNone(self.action.goal)
        self.assertEqual(self.rec.on("/ik_action/result")[-1]["outcome"], "preempted")

    def test_restart_cancels_leftover_and_resets(self):
        self.call("/ik_trial/start", {"success_hold": 0})
        self.tick(0.0, list(self.action.goal["positions"]))
        leftover = self.trial.goal_id
        self.now += 4.0
        ok, _, status = self.call("/ik_trial/start", {})
        self.assertTrue(ok, status)
        st = self.status()
        self.assertEqual((st["targets_reached"], st["elapsed"]), (0, 0.0))
        self.assertIn((leftover, "preempted"),
                      [(r["goal_id"], r["outcome"]) for r in self.rec.on("/ik_action/result")])
        self.assertEqual(self.action.goal["goal_id"], self.trial.goal_id)

    def test_start_fails_when_no_target_can_be_sampled(self):
        with mock.patch.object(kinematics, "forward_kinematics", side_effect=NotImplementedError):
            ok, _, status = self.call("/ik_trial/start", {})
        self.assertFalse(ok)
        self.assertTrue(status)
        self.assertFalse(self.trial.running)
        self.assertIsNone(self.action.goal)

    def test_trial_is_a_pure_action_client(self):
        for sub in self.registry._subscribers.get("/joint_states", set()):
            self.assertIsNot(getattr(getattr(sub, "_callback", None), "__self__", None), self.trial)
        # Trial goals reach /joint_trajectory only via the action's own publish.
        self.call("/ik_trial/start", {})
        self.call("/ik_trial/skip", {})
        self.assertEqual(len(self.rec.on("/joint_trajectory")), 2)


class TestIkTrialThreeLink(_PatchedKinematics):
    links = 3

    def test_sampled_targets_carry_phi(self):
        for _ in range(5):
            ok, _, status = self.call("/ik_trial/start", {})
            self.assertTrue(ok, status)
            self.assertEqual(set(self.trial.target), {"x", "y", "phi"})


class _ServerThread:
    """Same harness as tests/test_pid_services.py, including the
    gateway.stop()-before-loop.stop() teardown order; also registers
    ik_node."""

    def __init__(self, links: int = 2) -> None:
        self.loop = asyncio.new_event_loop()
        self.registry = Registry()
        self.state = arm_sim_node.register(self.registry, links)
        self.action, self.trial = ik_node.register(self.registry)
        self.gateway = Gateway(self.registry)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.gateway.start())
        self.loop.run_forever()

    def start(self) -> None:
        self._thread.start()
        time.sleep(0.2)

    def stop(self) -> None:
        fut = asyncio.run_coroutine_threadsafe(self.gateway.stop(), self.loop)
        try:
            fut.result(timeout=2)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(timeout=2)


class TestIkOverWire(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.patchers = [mock.patch.object(kinematics, "forward_kinematics", side_effect=_ref_fk),
                        mock.patch.object(kinematics, "inverse_kinematics", side_effect=_ref_ik)]
        for p in cls.patchers:
            p.start()
        cls.server = _ServerThread(links=3)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()
        for p in cls.patchers:
            p.stop()

    def test_solve_and_action_round_trip(self):
        with Client() as c:
            resp = c.call_service("/ik/solve", {"x": 1.5, "y": 1.0, "phi": 0.3})
            self.assertTrue(resp["result"], resp)
            self.assertEqual(len(resp["values"]["positions"]), 3)
            resp = c.call_service("/ik/solve", {"x": 9.0, "y": 0.0})
            self.assertFalse(resp["result"])
            self.assertTrue(resp["status"])

            c.subscribe("/ik_action/result")
            resp = c.call_service("/ik_action/send_goal", {"x": 1.5, "y": 1.0})
            self.assertTrue(resp["result"], resp)
            goal_id = resp["values"]["goal_id"]
            self.assertIsInstance(goal_id, str)
            # The result is published synchronously inside the cancel call,
            # so it arrives *before* the service_response; read both raw.
            c.send({"op": "call_service", "service": "/ik_action/cancel_goal", "id": "cancel-1",
                    "args": {"goal_id": goal_id}})
            got = {}
            while len(got) < 2:
                m = c.recv_line()
                if m.get("op") == "publish" and m.get("topic") == "/ik_action/result":
                    got["result"] = m["msg"]
                elif m.get("op") == "service_response" and m.get("id") == "cancel-1":
                    got["resp"] = m
            self.assertTrue(got["resp"]["result"], got["resp"])
            self.assertEqual(got["result"]["goal_id"], goal_id)
            self.assertEqual(got["result"]["outcome"], "preempted")
            self.assertEqual(set(got["result"]), {"goal_id", "outcome", "target", "final_distance"})
            resp = c.call_service("/ik_action/cancel_goal", {})
            self.assertFalse(resp["result"])

            resp = c.call_service("/ik_trial/stop", {})
            self.assertFalse(resp["result"])
            self.assertTrue(resp["status"])


if __name__ == "__main__":
    unittest.main()
