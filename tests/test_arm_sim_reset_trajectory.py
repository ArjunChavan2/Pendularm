"""Tests for /arm_sim/reset and the /joint_trajectory setpoint subscriber in
arm_sim_node.py: unit-level against _ArmSimState directly, plus wire-level
through a real in-process gateway (publish over TCP -> internal subscriber;
call_service -> reset handler).

These check the stored setpoint_pos/setpoint_vel and the plant/clock reset;
the controller-side effects (integral cleared on reset, effort, convergence)
are covered in tests/test_pid_services.py.
"""
import asyncio
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

import arm_sim_node  # noqa: E402
from arm_sim_node import _ArmSimState  # noqa: E402
from gateway import Gateway  # noqa: E402
from registry import Registry  # noqa: E402

from client_helper import Client  # noqa: E402


def _traj(points, joint_names=("joint1", "joint2")):
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
        "joint_names": list(joint_names),
        "points": points,
    }


def _point(positions=None, velocities=None):
    p = {"accelerations": [], "time_from_start": {"sec": 0, "nanosec": 0}}
    if positions is not None:
        p["positions"] = positions
    if velocities is not None:
        p["velocities"] = velocities
    return p


class TestSetTrajectoryUnit(unittest.TestCase):
    def test_initial_setpoint_is_reset_pose(self):
        state = _ArmSimState(3)
        self.assertEqual(state.setpoint_pos, [0.0, 0.0, 0.0])
        self.assertEqual(state.setpoint_vel, [0.0, 0.0, 0.0])

    def test_single_point_sets_setpoint(self):
        state = _ArmSimState(2)
        self.assertTrue(state.set_trajectory(_traj([_point([0.3, -0.5], [0.1, 0.2])])))
        self.assertEqual(state.setpoint_pos, [0.3, -0.5])
        self.assertEqual(state.setpoint_vel, [0.1, 0.2])

    def test_only_last_point_is_used(self):
        state = _ArmSimState(2)
        state.set_trajectory(_traj([_point([9.0, 9.0], [9.0, 9.0]), _point([1.0, 2.0], [0.0, -1.0])]))
        self.assertEqual(state.setpoint_pos, [1.0, 2.0])
        self.assertEqual(state.setpoint_vel, [0.0, -1.0])

    def test_ints_are_stored_as_floats(self):
        state = _ArmSimState(2)
        state.set_trajectory(_traj([_point([1, 0], [0, 0])]))
        self.assertEqual(state.setpoint_pos, [1.0, 0.0])
        self.assertTrue(all(isinstance(v, float) for v in state.setpoint_pos + state.setpoint_vel))

    def test_omitted_velocities_default_to_zero(self):
        state = _ArmSimState(2)
        state.setpoint_vel = [5.0, 5.0]
        state.set_trajectory(_traj([_point([0.3, -0.5])]))
        self.assertEqual(state.setpoint_pos, [0.3, -0.5])
        self.assertEqual(state.setpoint_vel, [0.0, 0.0])

    def test_wrong_length_arrays_default_to_zero_independently(self):
        state = _ArmSimState(2)
        state.set_trajectory(_traj([_point([0.3], [0.5, 0.5])]))
        self.assertEqual(state.setpoint_pos, [0.0, 0.0])
        self.assertEqual(state.setpoint_vel, [0.5, 0.5])

    def test_empty_velocities_default_to_zero(self):
        state = _ArmSimState(3)
        state.set_trajectory(_traj([_point([0.1, 0.2, 0.3], [])], ("joint1", "joint2", "joint3")))
        self.assertEqual(state.setpoint_pos, [0.1, 0.2, 0.3])
        self.assertEqual(state.setpoint_vel, [0.0, 0.0, 0.0])

    def test_non_numeric_or_non_finite_entries_default_to_zero(self):
        state = _ArmSimState(2)
        state.set_trajectory(_traj([_point(["a", 0.1], [True, 0.0])]))
        self.assertEqual(state.setpoint_pos, [0.0, 0.0])
        self.assertEqual(state.setpoint_vel, [0.0, 0.0])
        state.set_trajectory(_traj([_point([float("nan"), 0.1], [float("inf"), 0.0])]))
        self.assertEqual(state.setpoint_pos, [0.0, 0.0])
        self.assertEqual(state.setpoint_vel, [0.0, 0.0])

    def test_unusable_messages_are_ignored_and_keep_previous_setpoint(self):
        state = _ArmSimState(2)
        state.set_trajectory(_traj([_point([0.3, -0.5], [0.1, 0.2])]))
        for bad in (None, [], "x", 3, {}, {"points": []}, {"points": "x"}, {"points": [1]},
                    {"points": [_point([1.0, 1.0]), None]}):
            self.assertFalse(state.set_trajectory(bad), bad)
            self.assertEqual(state.setpoint_pos, [0.3, -0.5], bad)
            self.assertEqual(state.setpoint_vel, [0.1, 0.2], bad)

    def test_trajectory_does_not_touch_plant_state(self):
        state = _ArmSimState(2)
        state.q, state.qdot, state.sim_time = [0.7, 0.8], [0.1, 0.2], 3.0
        state.set_trajectory(_traj([_point([0.3, -0.5])]))
        self.assertEqual((state.q, state.qdot, state.sim_time), ([0.7, 0.8], [0.1, 0.2], 3.0))


class TestResetUnit(unittest.TestCase):
    def _moved_state(self, links=2) -> _ArmSimState:
        state = _ArmSimState(links)
        state.q = [0.5 * (i + 1) for i in range(links)]
        state.qdot = [-1.0] * links
        state.sim_time = 12.34
        state.setpoint_pos = [1.0] * links
        state.setpoint_vel = [0.25] * links
        return state

    def test_reset_snaps_pose_clock_and_setpoint(self):
        for links in (2, 3):
            state = self._moved_state(links)
            result, values, status = state.reset({})
            self.assertTrue(result)
            self.assertEqual(status, "")
            self.assertEqual(values, {"position": [0.0] * links, "velocity": [0.0] * links})
            self.assertEqual(state.q, [0.0] * links)
            self.assertEqual(state.qdot, [0.0] * links)
            self.assertEqual(state.sim_time, 0.0)
            self.assertEqual(state.setpoint_pos, [0.0] * links)
            self.assertEqual(state.setpoint_vel, [0.0] * links)

    def test_none_args_accepted(self):
        result, _, _ = self._moved_state().reset(None)
        self.assertTrue(result)

    def test_reset_calls_pid_hook(self):
        state = self._moved_state()
        calls = []
        state.reset_pid_controller = lambda: calls.append(1)
        state.reset({})
        self.assertEqual(calls, [1])

    def test_reset_leaves_params_and_pause_alone(self):
        state = self._moved_state()
        state.set_params({"gravity": 3.0, "masses": [2.0, 2.0]})
        state.set_integrator({"method": "rk4", "timestep": 0.02})
        state.pause({"data": True})
        state.reset({})
        self.assertEqual((state.gravity, state.masses), (3.0, [2.0, 2.0]))
        self.assertEqual((state.integrator_method, state.integrator_timestep), ("rk4", 0.02))
        self.assertTrue(state.paused)

    def test_malformed_request_rejected_without_resetting(self):
        for bad in ([], "reset", 1, True):
            state = self._moved_state()
            result, values, status = state.reset(bad)
            self.assertFalse(result, bad)
            self.assertTrue(status, bad)
            self.assertEqual(values, {"position": [0.5, 1.0], "velocity": [-1.0, -1.0]})
            self.assertEqual(state.sim_time, 12.34)
            self.assertEqual(state.setpoint_pos, [1.0, 1.0])

    def test_echoed_lists_are_copies(self):
        state = self._moved_state()
        _, values, _ = state.reset({})
        values["position"][0] = 99.0
        self.assertEqual(state.q, [0.0, 0.0])


class _ServerThread:
    """Same harness as tests/test_arm_sim_state_services.py (including the
    gateway.stop()-before-loop.stop() teardown order), but keeps the
    _ArmSimState register() returns so tests can inspect it."""

    def __init__(self, links: int = 2) -> None:
        self.loop = asyncio.new_event_loop()
        self.registry = Registry()
        self.state = arm_sim_node.register(self.registry, links)
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

    def run_in_loop(self, fn):
        """Run fn() on the server's event loop thread and return its result
        (state mutations/reads stay on the loop, as in production)."""
        async def _wrap():
            return fn()
        return asyncio.run_coroutine_threadsafe(_wrap(), self.loop).result(timeout=2)


def _wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class TestResetAndTrajectoryOverWire(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=2)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def setUp(self) -> None:
        self.server.run_in_loop(lambda: self.server.state.reset({}))

    def _setpoint(self):
        s = self.server.state
        return self.server.run_in_loop(lambda: (list(s.setpoint_pos), list(s.setpoint_vel)))

    def test_published_trajectory_sets_setpoint(self):
        with Client() as c:
            c.advertise("/joint_trajectory")
            c.publish("/joint_trajectory", _traj([_point([0.3, -0.5], [0.0, 0.0])]))
            self.assertTrue(_wait_until(lambda: self._setpoint() == ([0.3, -0.5], [0.0, 0.0])),
                            self._setpoint())

    def test_multi_point_trajectory_holds_last_point(self):
        with Client() as c:
            c.advertise("/joint_trajectory")
            c.publish("/joint_trajectory",
                      _traj([_point([1.0, 1.0], [0.0, 0.0]), _point([-0.2, 0.4], [0.1, 0.0])]))
            self.assertTrue(_wait_until(lambda: self._setpoint() == ([-0.2, 0.4], [0.1, 0.0])),
                            self._setpoint())

    def test_malformed_trajectory_ignored_and_gateway_stays_responsive(self):
        with Client() as c:
            c.advertise("/joint_trajectory")
            c.publish("/joint_trajectory", _traj([_point([0.3, -0.5])]))
            self.assertTrue(_wait_until(lambda: self._setpoint()[0] == [0.3, -0.5]))
            c.publish("/joint_trajectory", {"points": []})
            c.publish("/joint_trajectory", "garbage")
            # A service round-trip on the same connection orders after the
            # publishes, so they've been processed by the time it returns.
            resp = c.call_service("/arm_sim/set_params", {})
            self.assertTrue(resp["result"])
        self.assertEqual(self._setpoint(), ([0.3, -0.5], [0.0, 0.0]))

    def test_reset_over_wire_echoes_pose_and_resets_state(self):
        s = self.server.state

        def _move():
            s.q, s.qdot, s.sim_time = [0.4, -0.3], [1.0, 2.0], 5.0
        self.server.run_in_loop(_move)
        with Client() as c:
            c.advertise("/joint_trajectory")
            c.publish("/joint_trajectory", _traj([_point([0.3, -0.5], [0.2, 0.2])]))
            self.assertTrue(_wait_until(lambda: self._setpoint()[0] == [0.3, -0.5]))
            resp = c.call_service("/arm_sim/reset", {})
        self.assertTrue(resp["result"], resp)
        self.assertEqual(resp["values"], {"position": [0.0, 0.0], "velocity": [0.0, 0.0]})
        self.assertEqual(self.server.run_in_loop(lambda: (s.q, s.qdot, s.sim_time)),
                         ([0.0, 0.0], [0.0, 0.0], 0.0))
        self.assertEqual(self._setpoint(), ([0.0, 0.0], [0.0, 0.0]))

    def test_reset_malformed_request_rejected_over_wire(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/reset", [1, 2])
        self.assertFalse(resp["result"])
        self.assertIsInstance(resp["status"], str)
        self.assertTrue(resp["status"])
        self.assertEqual(set(resp["values"]), {"position", "velocity"})


if __name__ == "__main__":
    unittest.main()
