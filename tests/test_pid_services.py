"""Tests for the PID wiring in arm_sim_node.py: /pid_controller/enable,
/pid_controller/set_gains, /arm_sim/reset's controller reset, the effort
physics_loop applies (and /joint_states reports), and closed-loop
convergence on 2- and 3-link arms using the real arm_dynamics / integrators
/ pid modules.

The physics_loop tests use the same counting asyncio.sleep stand-in as
tests/test_arm_sim_physics_loop.py, so simulated seconds run in milliseconds
of wall time.
"""
from __future__ import annotations

import asyncio
import os
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

import arm_dynamics  # noqa: E402
import arm_sim_node  # noqa: E402
from arm_sim_node import _ArmSimState, physics_loop, publish_loop  # noqa: E402
from gateway import Gateway  # noqa: E402
from registry import Registry  # noqa: E402

from client_helper import Client  # noqa: E402


def _counting_sleep(n_ticks: int, on_tick=None):
    count = 0

    async def fake_sleep(dt: float) -> None:
        nonlocal count
        count += 1
        if count > n_ticks:
            raise asyncio.CancelledError()
        if on_tick is not None:
            on_tick(count)

    return fake_sleep


async def _run_physics(state: _ArmSimState, n_ticks: int, on_tick=None) -> None:
    with mock.patch("arm_sim_node.asyncio.sleep", side_effect=_counting_sleep(n_ticks, on_tick)):
        task = asyncio.create_task(physics_loop(state))
        try:
            await task
        except asyncio.CancelledError:
            pass


class FakeRegistry:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    def publish(self, topic: str, msg) -> None:
        self.published.append((topic, msg))


class TestEnableUnit(unittest.TestCase):
    def test_disabled_by_default(self):
        state = _ArmSimState(2)
        self.assertFalse(state.pid_enabled)
        self.assertEqual(state.pid_enable({}), (True, {"data": False}, ""))
        self.assertEqual(state.pid_enable(None), (True, {"data": False}, ""))

    def test_enable_disable_echo(self):
        state = _ArmSimState(2)
        self.assertEqual(state.pid_enable({"data": True}), (True, {"data": True}, ""))
        self.assertTrue(state.pid_enabled)
        self.assertEqual(state.pid_enable({"data": False}), (True, {"data": False}, ""))
        self.assertFalse(state.pid_enabled)

    def test_non_bool_data_rejected_without_change(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        for bad in ({"data": 1}, {"data": 0}, {"data": "false"}, {"data": None}, [1], "x"):
            result, values, status = state.pid_enable(bad)
            self.assertFalse(result, bad)
            self.assertTrue(status, bad)
            self.assertEqual(values, {"data": True}, bad)
            self.assertTrue(state.pid_enabled, bad)

    def test_reenable_after_disable_clears_integral(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        state.controller.integral = [0.5, -0.5]
        state.pid_enable({"data": False})
        self.assertEqual(list(state.controller.integral), [0.5, -0.5])
        state.pid_enable({"data": True})
        self.assertEqual(list(state.controller.integral), [0.0, 0.0])

    def test_enable_while_already_enabled_keeps_integral(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        state.controller.integral = [0.5, -0.5]
        state.pid_enable({"data": True})
        self.assertEqual(list(state.controller.integral), [0.5, -0.5])

    def test_disable_zeroes_reported_effort(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        state.tau = [3.0, 4.0]
        state.pid_enable({"data": False})
        self.assertEqual(state.tau, [0.0, 0.0])


class TestSetGainsUnit(unittest.TestCase):
    def test_empty_request_queries_defaults(self):
        for links in (2, 3):
            state = _ArmSimState(links)
            result, values, status = state.set_gains({})
            self.assertTrue(result)
            self.assertEqual(status, "")
            self.assertEqual(values, {"kp": [arm_sim_node.DEFAULT_KP] * links,
                                      "ki": [arm_sim_node.DEFAULT_KI] * links,
                                      "kd": [arm_sim_node.DEFAULT_KD] * links})
            self.assertEqual(state.set_gains(None)[1], values)

    def test_valid_update_applies_and_echoes(self):
        state = _ArmSimState(2)
        result, values, status = state.set_gains({"kp": [10, 20.5], "ki": [0, 0], "kd": [1.0, 2.0]})
        self.assertTrue(result, status)
        self.assertEqual(values, {"kp": [10.0, 20.5], "ki": [0.0, 0.0], "kd": [1.0, 2.0]})
        self.assertEqual(list(state.controller.kp), [10.0, 20.5])

    def test_each_field_rejected_independently(self):
        bad_values = ([1.0], [1.0, 2.0, 3.0], [-1.0, 2.0], [1.0, "a"], [True, 1.0],
                      [float("nan"), 1.0], [float("inf"), 1.0], "x", None, 5)
        for field in ("kp", "ki", "kd"):
            for bad in bad_values:
                state = _ArmSimState(2)
                before = state.set_gains({})[1]
                others = [f for f in ("kp", "ki", "kd") if f != field]
                req = {field: bad, others[0]: [7.0, 8.0]}
                result, values, status = state.set_gains(req)
                self.assertFalse(result, (field, bad))
                self.assertIn(field, status)
                self.assertEqual(values[field], before[field], (field, bad))
                self.assertEqual(values[others[0]], [7.0, 8.0], (field, bad))
                self.assertEqual(values[others[1]], before[others[1]], (field, bad))

    def test_malformed_request_rejected(self):
        state = _ArmSimState(2)
        result, values, status = state.set_gains([1, 2])
        self.assertFalse(result)
        self.assertTrue(status)
        self.assertEqual(set(values), {"kp", "ki", "kd"})

    def test_set_gains_preserves_integral(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        state.controller.integral = [0.25, -0.75]
        state.set_gains({"kp": [1.0, 1.0], "ki": [2.0, 2.0], "kd": [3.0, 3.0]})
        self.assertEqual(list(state.controller.integral), [0.25, -0.75])
        self.assertTrue(state.pid_enabled)

    def test_echo_is_a_copy(self):
        state = _ArmSimState(2)
        _, values, _ = state.set_gains({})
        values["kp"][0] = -99.0
        self.assertEqual(state.set_gains({})[1]["kp"][0], arm_sim_node.DEFAULT_KP)


class TestResetClearsController(unittest.TestCase):
    def test_reset_clears_integral_keeps_gains_and_enabled(self):
        state = _ArmSimState(3)
        state.pid_enable({"data": True})
        state.set_gains({"kp": [1.0, 2.0, 3.0]})
        state.controller.integral = [0.1, 0.2, 0.3]
        state.tau = [1.0, 1.0, 1.0]
        result, _, _ = state.reset({})
        self.assertTrue(result)
        self.assertEqual(list(state.controller.integral), [0.0, 0.0, 0.0])
        self.assertEqual(state.tau, [0.0, 0.0, 0.0])
        self.assertEqual(list(state.controller.kp), [1.0, 2.0, 3.0])
        self.assertTrue(state.pid_enabled)


class TestPhysicsLoopEffort(unittest.IsolatedAsyncioTestCase):
    async def test_disabled_applies_zero_effort_and_never_updates(self):
        state = _ArmSimState(2)
        state.setpoint_pos = [1.0, 1.0]
        seen_tau = []

        def spy(q, qdot, tau, gravity, masses, lengths):
            seen_tau.append(list(tau))
            return [0.0, 0.0]

        with mock.patch.object(state.controller, "update", side_effect=AssertionError("called")), \
                mock.patch("arm_dynamics.forward_dynamics", side_effect=spy):
            await _run_physics(state, 5)

        self.assertEqual(len(seen_tau), 5)
        self.assertTrue(all(t == [0.0, 0.0] for t in seen_tau))
        self.assertEqual(state.tau, [0.0, 0.0])
        self.assertEqual(list(state.controller.integral), [0.0, 0.0])

    async def test_enabled_under_gravity_gives_nonzero_effort(self):
        state = _ArmSimState(2)
        state.pid_enable({"data": True})
        await _run_physics(state, 20)
        self.assertTrue(any(abs(v) > 1e-3 for v in state.tau), state.tau)

    async def test_update_called_once_per_tick_with_start_state_and_dt(self):
        state = _ArmSimState(2)
        state.integrator_method = "rk4"
        state.integrator_timestep = 0.02
        state.pid_enable({"data": True})
        state.setpoint_pos = [0.3, -0.5]
        state.setpoint_vel = [0.1, 0.0]
        state.q, state.qdot = [0.1, 0.2], [0.3, 0.4]
        calls = []
        seen_tau = []

        def fake_update(sp, sv, q, qdot, dt):
            calls.append((list(sp), list(sv), list(q), list(qdot), dt))
            return [1.5, -2.5]

        def spy(q, qdot, tau, gravity, masses, lengths):
            seen_tau.append(list(tau))
            return [0.0, 0.0]

        with mock.patch.object(state.controller, "update", side_effect=fake_update), \
                mock.patch("arm_dynamics.forward_dynamics", side_effect=spy):
            await _run_physics(state, 1)

        self.assertEqual(calls, [([0.3, -0.5], [0.1, 0.0], [0.1, 0.2], [0.3, 0.4], 0.02)])
        self.assertEqual(len(seen_tau), 4)  # rk4's four stages, tau held constant
        self.assertTrue(all(t == [1.5, -2.5] for t in seen_tau))
        self.assertEqual(state.tau, [1.5, -2.5])

    async def test_disable_stops_integral_accumulation(self):
        state = _ArmSimState(2)
        state.setpoint_pos = [0.5, 0.5]
        state.pid_enable({"data": True})
        await _run_physics(state, 10)
        frozen = list(state.controller.integral)
        self.assertTrue(any(v != 0 for v in frozen))
        state.pid_enable({"data": False})
        await _run_physics(state, 10)
        self.assertEqual(list(state.controller.integral), frozen)
        self.assertEqual(state.tau, [0.0, 0.0])
        state.pid_enable({"data": True})
        self.assertEqual(list(state.controller.integral), [0.0, 0.0])

    async def test_paused_does_not_advance_integral(self):
        state = _ArmSimState(2)
        state.setpoint_pos = [0.5, 0.5]
        state.pid_enable({"data": True})
        await _run_physics(state, 5)
        frozen = list(state.controller.integral)
        state.pause({"data": True})
        await _run_physics(state, 5)
        self.assertEqual(list(state.controller.integral), frozen)

    async def test_publish_loop_reports_applied_effort(self):
        state = _ArmSimState(2)
        state.tau = [1.25, -0.5]
        registry = FakeRegistry()
        with mock.patch("arm_sim_node.asyncio.sleep", side_effect=_counting_sleep(1)):
            try:
                await publish_loop(registry, state)
            except asyncio.CancelledError:
                pass
        self.assertEqual(registry.published[0][1]["effort"], [1.25, -0.5])


class TestConvergence(unittest.IsolatedAsyncioTestCase):
    """Closed loop with the default gains and real dynamics: the arm must
    settle at the commanded setpoint, with steady-state effort equal to the
    gravity load G(q) there (spec, "Dynamics")."""

    async def _converge(self, links, setpoint, method, ticks):
        state = _ArmSimState(links)
        state.integrator_method = method
        state.set_trajectory({"points": [{"positions": setpoint}]})
        ok, values, _ = state.pid_enable({"data": True})
        self.assertTrue(ok and values["data"])
        await _run_physics(state, ticks)
        for actual, target in zip(state.q, setpoint):
            self.assertAlmostEqual(actual, target, delta=0.05, msg=(state.q, setpoint))
        for v in state.qdot:
            self.assertLess(abs(v), 0.05, state.qdot)
        g = arm_dynamics.G(state.q, state.gravity, state.masses, state.lengths)
        for tau_i, g_i in zip(state.tau, g):
            self.assertAlmostEqual(tau_i, g_i, delta=0.5, msg=(state.tau, g))
        self.assertTrue(any(abs(v) > 1.0 for v in state.tau), state.tau)

    async def test_two_link_converges_euler(self):
        await self._converge(2, [0.3, -0.5], "euler", 1000)

    async def test_two_link_converges_rk4(self):
        await self._converge(2, [1.2, 0.8], "rk4", 1000)

    async def test_three_link_converges_euler(self):
        await self._converge(3, [0.3, -0.5, 0.4], "euler", 3000)

    async def test_three_link_converges_rk4(self):
        await self._converge(3, [1.0, -1.0, 1.0], "rk4", 3000)


class _ServerThread:
    """Same harness as tests/test_arm_sim_reset_trajectory.py, including the
    gateway.stop()-before-loop.stop() teardown order."""

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


class TestPidServicesOverWire(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=3)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_enable_round_trip(self):
        with Client() as c:
            resp = c.call_service("/pid_controller/enable", {})
            self.assertEqual((resp["result"], resp["values"]), (True, {"data": False}))
            resp = c.call_service("/pid_controller/enable", {"data": True})
            self.assertEqual((resp["result"], resp["values"]), (True, {"data": True}))
            resp = c.call_service("/pid_controller/enable", {"data": "yes"})
            self.assertFalse(resp["result"])
            self.assertTrue(resp["status"])
            self.assertEqual(resp["values"], {"data": True})
            resp = c.call_service("/pid_controller/enable", {"data": False})
            self.assertEqual((resp["result"], resp["values"]), (True, {"data": False}))

    def test_set_gains_round_trip(self):
        with Client() as c:
            resp = c.call_service("/pid_controller/set_gains", {})
            self.assertTrue(resp["result"])
            self.assertEqual(set(resp["values"]), {"kp", "ki", "kd"})
            resp = c.call_service("/pid_controller/set_gains",
                                  {"kp": [5.0, 5.0, 5.0], "kd": [1.0, -1.0, 1.0]})
            self.assertFalse(resp["result"])
            self.assertIn("kd", resp["status"])
            self.assertEqual(resp["values"]["kp"], [5.0, 5.0, 5.0])
            self.assertEqual(resp["values"]["kd"], [arm_sim_node.DEFAULT_KD] * 3)
            c.call_service("/pid_controller/set_gains", {"kp": [arm_sim_node.DEFAULT_KP] * 3})


if __name__ == "__main__":
    unittest.main()
