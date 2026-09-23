"""Unit-level wiring tests for arm_sim_node.physics_loop and .publish_loop.

These deliberately do NOT depend on arm_dynamics.forward_dynamics' real
implementation (it's an intentional stub that always raises
NotImplementedError until its owner hand-implements it -- see
src/arm_dynamics.py and agent-notes/PLAN.md's standing rule). Instead they
monkeypatch arm_dynamics.forward_dynamics with a simple, known stand-in so
the *wiring* -- does physics_loop call integrators.METHODS[...] with the
right arguments and read state fresh each tick; does it skip stepping while
paused; does publish_loop build a correctly-shaped, correctly-rated message
independent of paused/integrator_timestep -- can be verified without waiting
on the owner's real dynamics. Mirrors how tests/test_integrators.py already
scopes integrators.py's own algorithmic correctness out of this kind of
wiring test.

To make the infinite while-True loops in physics_loop/publish_loop
deterministic and fast under a test runner (rather than sleeping real
wall-clock seconds), asyncio.sleep is monkeypatched with a counting stand-in
that raises asyncio.CancelledError once a target tick count is reached --
the same natural mechanism asyncio uses for real task cancellation, so the
loop under test exits exactly as it would in production, just without
waiting on the real clock.
"""
from __future__ import annotations

import asyncio
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import arm_dynamics  # noqa: E402
import arm_sim_node  # noqa: E402
import integrators  # noqa: E402
from arm_sim_node import _ArmSimState, physics_loop, publish_loop  # noqa: E402


def _counting_sleep(n_ticks: int, on_tick=None):
    """Returns an async stand-in for asyncio.sleep that lets the caller's
    loop run exactly `n_ticks` iterations (recording each requested `dt`),
    then ends the loop by raising CancelledError from inside the awaited
    sleep -- exactly how a real task cancellation would terminate it.
    """
    calls: list[float] = []
    count = 0

    async def fake_sleep(dt: float) -> None:
        nonlocal count
        count += 1
        if count > n_ticks:
            raise asyncio.CancelledError()
        calls.append(dt)
        if on_tick is not None:
            on_tick(count)

    return fake_sleep, calls


class FakeRegistry:
    """Minimal publish()-only stand-in for registry.Registry, so publish_loop
    tests don't need a real Registry/gateway."""

    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    def publish(self, topic: str, msg) -> None:
        self.published.append((topic, msg))


class TestPhysicsLoopWiring(unittest.IsolatedAsyncioTestCase):
    async def _run_loop_ticks(self, state: _ArmSimState, n_ticks: int, on_tick=None) -> list[float]:
        fake_sleep, calls = _counting_sleep(n_ticks, on_tick=on_tick)
        with mock.patch("arm_sim_node.asyncio.sleep", side_effect=fake_sleep):
            task = asyncio.create_task(physics_loop(state))
            try:
                await task
            except asyncio.CancelledError:
                pass
        return calls

    async def test_advances_q_qdot_matching_hand_computed_euler(self):
        # links=1, euler, dt=0.1, constant acceleration 2.0 -- so this
        # exactly matches integrators.euler's own hand-verified formula:
        # q1 = q0 + qdot0*dt; qdot1 = qdot0 + a*dt.
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 0.1

        with mock.patch("arm_dynamics.forward_dynamics", return_value=[2.0]):
            await self._run_loop_ticks(state, n_ticks=1)

        self.assertAlmostEqual(state.q[0], 0.0 + 0.0 * 0.1)
        self.assertAlmostEqual(state.qdot[0], 0.0 + 2.0 * 0.1)
        self.assertAlmostEqual(state.sim_time, 0.1)

        # A second tick continues from the updated state, not a cached copy.
        with mock.patch("arm_dynamics.forward_dynamics", return_value=[2.0]):
            await self._run_loop_ticks(state, n_ticks=1)

        self.assertAlmostEqual(state.q[0], 0.0 + 0.2 * 0.1)  # q0 + qdot0*dt
        self.assertAlmostEqual(state.qdot[0], 0.2 + 2.0 * 0.1)
        self.assertAlmostEqual(state.sim_time, 0.2)

    async def test_calls_integrators_methods_with_current_method_and_params(self):
        state = _ArmSimState(links=1)
        state.integrator_method = "rk4"
        state.integrator_timestep = 0.05

        recorded = {}
        real_rk4 = integrators.METHODS["rk4"]

        def spy_rk4(accel_fn, t, q, qdot, dt):
            recorded["t"] = t
            recorded["q"] = list(q)
            recorded["qdot"] = list(qdot)
            recorded["dt"] = dt
            return real_rk4(accel_fn, t, q, qdot, dt)

        with mock.patch.dict(integrators.METHODS, {"rk4": spy_rk4}), \
                mock.patch("arm_dynamics.forward_dynamics", return_value=[0.0]):
            await self._run_loop_ticks(state, n_ticks=1)

        self.assertEqual(recorded["t"], 0.0)
        self.assertEqual(recorded["q"], [0.0])
        self.assertEqual(recorded["qdot"], [0.0])
        self.assertEqual(recorded["dt"], 0.05)

    async def test_reads_gravity_fresh_each_tick_not_cached(self):
        # forward_dynamics stub echoes the gravity value it was called with,
        # so a mid-run state.gravity change is directly observable in the
        # resulting qdot only if physics_loop re-reads state.gravity every
        # tick rather than capturing it once outside the loop.
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 1.0
        state.gravity = 1.0

        def stub_forward_dynamics(q, qdot, tau, gravity, masses, lengths):
            return [gravity]

        def bump_gravity(tick_count: int) -> None:
            if tick_count == 2:
                # Fires during tick 2's sleep, before tick 2's own body runs
                # (but after tick 1's body already ran and used the old
                # value) -- simulating a concurrent /arm_sim/set_params call
                # arriving mid-run.
                state.gravity = 5.0

        with mock.patch("arm_dynamics.forward_dynamics", side_effect=stub_forward_dynamics):
            await self._run_loop_ticks(state, n_ticks=2, on_tick=bump_gravity)

        # tick 1: qdot 0 -> 0 + 1.0*1.0 = 1.0 (using gravity=1.0)
        # tick 2: qdot 1.0 -> 1.0 + 5.0*1.0 = 6.0 (using the NEW gravity=5.0)
        self.assertAlmostEqual(state.qdot[0], 6.0)

    async def test_reads_integrator_timestep_fresh_each_tick_not_cached(self):
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 0.1

        def bump_timestep(tick_count: int) -> None:
            if tick_count == 2:
                state.integrator_timestep = 0.5

        with mock.patch("arm_dynamics.forward_dynamics", return_value=[0.0]):
            dt_calls = await self._run_loop_ticks(state, n_ticks=3, on_tick=bump_timestep)

        self.assertEqual(dt_calls, [0.1, 0.1, 0.5])

    async def test_reads_integrator_method_fresh_each_tick_not_cached(self):
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 1.0

        called_with_methods: list[str] = []
        real_euler = integrators.METHODS["euler"]
        real_rk4 = integrators.METHODS["rk4"]

        def spy_euler(accel_fn, t, q, qdot, dt):
            called_with_methods.append("euler")
            return real_euler(accel_fn, t, q, qdot, dt)

        def spy_rk4(accel_fn, t, q, qdot, dt):
            called_with_methods.append("rk4")
            return real_rk4(accel_fn, t, q, qdot, dt)

        def switch_method(tick_count: int) -> None:
            if tick_count == 2:
                state.integrator_method = "rk4"

        with mock.patch.dict(integrators.METHODS, {"euler": spy_euler, "rk4": spy_rk4}), \
                mock.patch("arm_dynamics.forward_dynamics", return_value=[0.0]):
            await self._run_loop_ticks(state, n_ticks=2, on_tick=switch_method)

        self.assertEqual(called_with_methods, ["euler", "rk4"])

    async def test_skips_stepping_while_paused(self):
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 0.1
        state.paused = True

        call_count = 0

        def counting_stub(q, qdot, tau, gravity, masses, lengths):
            nonlocal call_count
            call_count += 1
            return [10.0]  # large, so a mistaken step would be obvious

        with mock.patch("arm_dynamics.forward_dynamics", side_effect=counting_stub):
            await self._run_loop_ticks(state, n_ticks=5)

        self.assertEqual(call_count, 0, "forward_dynamics must not be called while paused")
        self.assertEqual(state.q, [0.0])
        self.assertEqual(state.qdot, [0.0])
        self.assertEqual(state.sim_time, 0.0)

    async def test_resumes_from_where_it_left_off_after_unpause(self):
        state = _ArmSimState(links=1)
        state.integrator_method = "euler"
        state.integrator_timestep = 0.1
        state.paused = True

        def unpause_after_first_tick(tick_count: int) -> None:
            if tick_count == 2:
                state.paused = False

        with mock.patch("arm_dynamics.forward_dynamics", return_value=[1.0]):
            await self._run_loop_ticks(state, n_ticks=3, on_tick=unpause_after_first_tick)

        # Tick 1 paused (no-op); unpaused before tick 2's body runs, so
        # ticks 2-3 each take one euler step of dt=0.1 with constant
        # acceleration 1.0 -- two steps' worth of motion, resuming from
        # exactly where it left off (not reset).
        self.assertAlmostEqual(state.sim_time, 0.2)
        self.assertAlmostEqual(state.qdot[0], 0.2)

    async def test_forward_dynamics_exception_is_caught_and_loop_keeps_running(self):
        # The real, currently-unimplemented arm_dynamics.forward_dynamics
        # always raises NotImplementedError -- physics_loop must catch this
        # (and any other exception) and keep looping rather than letting the
        # task die silently, per the plan's "Edge cases" section.
        state = _ArmSimState(links=2)
        state.integrator_method = "euler"
        state.integrator_timestep = 0.1

        # Use the REAL, unmodified arm_dynamics.forward_dynamics stub here
        # (no monkeypatch) -- this is the actual failure mode in the shipped
        # repo today.
        calls = await self._run_loop_ticks(state, n_ticks=5)

        self.assertEqual(len(calls), 5, "loop must keep sleeping/ticking despite the exception")
        # A tick that raises must not partially apply -- state stays frozen
        # at the reset pose.
        self.assertEqual(state.q, [0.0, 0.0])
        self.assertEqual(state.qdot, [0.0, 0.0])
        self.assertEqual(state.sim_time, 0.0)

    async def test_set_params_and_set_integrator_handlers_take_effect_without_restart(self):
        # Exercises the actual public handler methods (not direct attribute
        # assignment) mid-run, matching how a real /arm_sim/set_params or
        # /arm_sim/set_integrator service call would mutate this same state
        # object while a single continuous physics_loop task keeps running
        # -- no restart, no re-creating the loop.
        state = _ArmSimState(links=1)  # default gravity=9.81, timestep=0.01

        seen_gravities: list[float] = []

        def recording_stub(q, qdot, tau, gravity, masses, lengths):
            seen_gravities.append(gravity)
            return [0.0]

        def apply_mid_run_changes(tick_count: int) -> None:
            if tick_count == 2:
                ok, values, status = state.set_params({"gravity": 3.0})
                self.assertTrue(ok, status)
                ok, values, status = state.set_integrator({"timestep": 2.0})
                self.assertTrue(ok, status)

        with mock.patch("arm_dynamics.forward_dynamics", side_effect=recording_stub):
            dt_calls = await self._run_loop_ticks(state, n_ticks=3, on_tick=apply_mid_run_changes)

        # tick 1: default gravity (9.81), default pacing timestep (0.01).
        # tick 2: set_params({"gravity": 3.0}) fires before this tick's body
        #   reads gravity, and set_integrator({"timestep": 2.0}) fires before
        #   the *next* sleep call re-reads the pacing interval.
        # tick 3: both changes still in effect (no restart needed).
        self.assertEqual(seen_gravities, [9.81, 3.0, 3.0])
        self.assertEqual(dt_calls, [0.01, 0.01, 2.0])


class TestPublishLoopWiring(unittest.IsolatedAsyncioTestCase):
    async def _run_loop_ticks(self, registry: FakeRegistry, state: _ArmSimState, n_ticks: int, on_tick=None) -> list[float]:
        fake_sleep, calls = _counting_sleep(n_ticks, on_tick=on_tick)
        with mock.patch("arm_sim_node.asyncio.sleep", side_effect=fake_sleep):
            task = asyncio.create_task(publish_loop(registry, state))
            try:
                await task
            except asyncio.CancelledError:
                pass
        return calls

    async def test_message_shape_and_content(self):
        state = _ArmSimState(links=2)
        state.q = [0.3, -0.5]
        state.qdot = [1.0, 2.0]
        state.sim_time = 1.5
        registry = FakeRegistry()

        await self._run_loop_ticks(registry, state, n_ticks=1)

        self.assertEqual(len(registry.published), 1)
        topic, msg = registry.published[0]
        self.assertEqual(topic, "/joint_states")
        self.assertEqual(msg["name"], ["joint1", "joint2"])
        self.assertEqual(msg["position"], [0.3, -0.5])
        self.assertEqual(msg["velocity"], [1.0, 2.0])
        self.assertEqual(msg["effort"], [0.0, 0.0])
        self.assertEqual(msg["header"]["stamp"]["sec"], 1)
        self.assertAlmostEqual(msg["header"]["stamp"]["nanosec"], 5 * 1e8, delta=1)
        self.assertEqual(msg["header"]["frame_id"], "")

    async def test_three_link_arm_has_three_joint_names(self):
        state = _ArmSimState(links=3)
        registry = FakeRegistry()

        await self._run_loop_ticks(registry, state, n_ticks=1)

        _, msg = registry.published[0]
        self.assertEqual(msg["name"], ["joint1", "joint2", "joint3"])
        self.assertEqual(len(msg["position"]), 3)
        self.assertEqual(len(msg["velocity"]), 3)
        self.assertEqual(len(msg["effort"]), 3)

    async def test_publishes_at_fixed_rate_independent_of_paused(self):
        state = _ArmSimState(links=1)
        state.paused = True
        registry = FakeRegistry()

        dt_calls = await self._run_loop_ticks(registry, state, n_ticks=5)

        self.assertEqual(len(registry.published), 5, "publish_loop must keep publishing while paused")
        for dt in dt_calls:
            self.assertAlmostEqual(dt, arm_sim_node.PUBLISH_PERIOD_S)

    async def test_publishes_at_fixed_rate_independent_of_integrator_timestep(self):
        for timestep in (0.0001, 1.0, 100.0):
            with self.subTest(timestep=timestep):
                state = _ArmSimState(links=1)
                state.integrator_timestep = timestep
                registry = FakeRegistry()

                dt_calls = await self._run_loop_ticks(registry, state, n_ticks=3)

                for dt in dt_calls:
                    self.assertAlmostEqual(dt, arm_sim_node.PUBLISH_PERIOD_S)

    async def test_publishes_frozen_snapshot_while_paused(self):
        state = _ArmSimState(links=1)
        state.paused = True
        state.q = [0.7]
        state.qdot = [0.0]
        registry = FakeRegistry()

        await self._run_loop_ticks(registry, state, n_ticks=3)

        for _, msg in registry.published:
            self.assertEqual(msg["position"], [0.7])
            self.assertEqual(msg["velocity"], [0.0])


class TestRegisterReturnsState(unittest.TestCase):
    def test_register_returns_arm_sim_state_with_live_fields(self):
        import registry as registry_mod

        reg = registry_mod.Registry()
        state = arm_sim_node.register(reg, links=3)

        self.assertIsInstance(state, _ArmSimState)
        self.assertEqual(state.links, 3)
        self.assertEqual(state.q, [0.0, 0.0, 0.0])
        self.assertEqual(state.qdot, [0.0, 0.0, 0.0])
        self.assertEqual(state.sim_time, 0.0)
        # Existing service handlers are still registered against this same state.
        self.assertTrue(reg.has_in_process_handler("/arm_sim/set_params"))
        self.assertTrue(reg.has_in_process_handler("/arm_sim/set_integrator"))
        self.assertTrue(reg.has_in_process_handler("/arm_sim/pause"))
        self.assertTrue(reg.has_in_process_handler("/arm_sim/integration_step"))


if __name__ == "__main__":
    unittest.main()
