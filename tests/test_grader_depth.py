"""In-depth, grader-style checks of every service's black-box contract.

Everything here goes through Registry.call_service_sync -- the same handlers
the gateway dispatches to -- so it exercises real request validation and
response shapes without opening a socket (safe to run alongside a live
runtime on 127.0.0.1:9095). Each test cites the spec sentence it checks
(spec/PROJECT2_PENDULARM.md).
"""
from __future__ import annotations

import json
import math
import os
import random
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import arm_dynamics  # noqa: E402
import arm_sim_node  # noqa: E402
import ik_node  # noqa: E402
import integrators  # noqa: E402
import kinematics  # noqa: E402
from registry import Registry  # noqa: E402

METHODS = ("euler", "midpoint", "verlet", "rk4")


def _runtime(links: int):
    registry = Registry()
    state = arm_sim_node.register(registry, links)
    ik_node.register(registry)
    return registry, state


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class _Base(unittest.TestCase):
    def call(self, registry, service, args):
        ok, values, status = registry.call_service_sync(service, args)
        # Every response must be JSON-serializable with a dict `values` and a
        # string `status`; a rejection must explain itself.
        self.assertIsInstance(values, dict, (service, args))
        self.assertIsInstance(status, str, (service, args))
        json.dumps(values)
        if not ok:
            self.assertTrue(status, f"{service} {args!r}: result:false needs a nonempty status")
        return ok, values, status


# ---------------------------------------------------------------- integration_step

class TestIntegrationStepDepth(_Base):
    def setUp(self):
        self.reg, _ = _runtime(2)

    def step(self, **args):
        return self.call(self.reg, "/arm_sim/integration_step", args)

    def test_shape_and_initial_point_every_method(self):
        # "one entry per requested step, plus the starting point at index 0"
        for m in METHODS:
            ok, v, _ = self.step(function="sin(t)", x0=1.5, xdot0=-0.25, dt=0.1, steps=7, integrator=m)
            self.assertTrue(ok, m)
            for key in ("times", "positions", "velocities"):
                self.assertEqual(len(v[key]), 8, (m, key))
            self.assertEqual(v["times"][0], 0)
            self.assertEqual(v["positions"][0], 1.5)
            self.assertEqual(v["velocities"][0], -0.25)
            for k, t in enumerate(v["times"]):
                self.assertAlmostEqual(t, k * 0.1, places=12)

    def test_xdot0_defaults_to_zero(self):
        ok, v, _ = self.step(function="0", x0=2.0, dt=0.5, steps=3, integrator="euler")
        self.assertTrue(ok)
        self.assertEqual(v["velocities"], [0.0] * 4)
        self.assertEqual(v["positions"], [2.0] * 4)

    def test_f_equals_t_accuracy_ordering(self):
        # "f(t) = t (whose closed form is x(t) = t^3/6 from rest) ... a method
        # that freezes t across the step ends up measurably worse"
        errs = {}
        for m in METHODS:
            ok, v, _ = self.step(function="t", x0=0, dt=0.1, steps=10, integrator=m)
            self.assertTrue(ok)
            errs[m] = abs(v["positions"][-1] - 1 / 6)
        self.assertGreater(errs["euler"], errs["midpoint"])
        self.assertGreater(errs["euler"], errs["verlet"])
        self.assertLess(errs["rk4"], 1e-12)  # RK4 is exact for cubic x(t)

    def test_convergence_orders(self):
        # Halving dt: Euler error ~/2, second-order methods ~/4, RK4 much more.
        def err(m, dt):
            n = round(1.0 / dt)
            ok, v, _ = self.step(function="cos(t)", x0=0, xdot0=0, dt=dt, steps=n, integrator=m)
            return abs(v["positions"][-1] - (1 - math.cos(1.0)))
        for m, lo, hi in (("euler", 1.6, 2.5), ("midpoint", 3.0, 5.0), ("verlet", 3.0, 5.0)):
            ratio = err(m, 0.02) / err(m, 0.01)
            self.assertTrue(lo < ratio < hi, f"{m} convergence ratio {ratio:.2f}")
        self.assertLess(err("rk4", 0.01), 1e-9)

    def test_closed_forms_rk4(self):
        cases = [("sin(t)", lambda t: t - math.sin(t)),
                 ("exp(t)", lambda t: math.exp(t) - 1 - t),
                 ("t^2 + 3*t - 1", lambda t: t**4 / 12 + t**3 / 2 - t**2 / 2),
                 ("-t + 1", lambda t: -t**3 / 6 + t**2 / 2)]
        for fn, x in cases:
            ok, v, _ = self.step(function=fn, x0=0, dt=0.01, steps=100, integrator="rk4")
            self.assertTrue(ok, fn)
            self.assertAlmostEqual(v["positions"][-1], x(1.0), places=7, msg=fn)

    def test_parser_precedence_and_associativity(self):
        # lowest->highest: +/-, */ /, ^, unary minus, atoms; ^ right-assoc
        cases = {"2^3^2": 512, "8/4/2": 1, "10-4-3": 3, "2+3*4": 14, "(2+3)*4": 20,
                 "-2^2": 4, "2^-1": 0.5, "--3": 3, "-t+1": 1 - 0.5, "abs(-3)": 3,
                 "ln(exp(2))": 2, "sqrt(16)": 4, "tan(0)": 0, "1.5e1": 15, ".5": 0.5}
        for src, expected in cases.items():
            ok, v, st = self.step(function=src, x0=0, dt=1.0, steps=1, integrator="euler")
            if src in ("1.5e1", ".5") and not ok:
                continue  # literal formats beyond the spec's minimum -- informational only
            self.assertTrue(ok, f"{src!r}: {st}")
            # euler: xdot1 = f(0)*dt  (t = 0.5 case handled via -t+1 with t=0 -> 1)
            got = v["velocities"][1]
            want = expected if src != "-t+1" else 1.0
            self.assertAlmostEqual(got, want, places=9, msg=src)

    def test_malformed_functions_rejected_cleanly(self):
        bad = ["", "   ", "(", ")", "(t", "t)", "((t)", "foo(t)", "x", "1 2", "t t", "3t",
               "t $", "t +", "* t", "sin", "sin(", "sin()", "sin(t,t)", "2 ^", "t..2", "@",
               "t^^2", "()", "sin t"]
        for src in bad:
            ok, _, st = self.step(function=src, x0=0, dt=0.1, steps=1, integrator="euler")
            self.assertFalse(ok, f"{src!r} should be rejected")

    def test_invalid_arguments_rejected(self):
        base = dict(function="t", x0=0, dt=0.1, steps=3, integrator="rk4")
        for override in ({"dt": 0}, {"dt": -0.1}, {"steps": 0}, {"integrator": "RK4"},
                         {"integrator": "runge-kutta"}, {"integrator": None}, {"function": None},
                         {"function": 5}, {"dt": "0.1"}, {"x0": "0"}, {"steps": -1}):
            ok, _, _ = self.step(**{**base, **override})
            self.assertFalse(ok, override)
        for missing in ("function", "dt", "steps", "integrator"):
            args = dict(base)
            del args[missing]
            ok, _, _ = self.step(**args)
            self.assertFalse(ok, f"missing {missing}")

    def test_domain_errors_become_nan_not_rejection(self):
        for fn in ("sqrt(-1)", "ln(0)", "1/0", "(-4)^0.5", "exp(1000)", "ln(-1)"):
            ok, v, st = self.step(function=fn, x0=0, dt=0.1, steps=2, integrator="rk4")
            self.assertTrue(ok, f"{fn}: {st}")
            json.dumps(v)  # must still serialize

    def test_large_step_count_is_fast(self):
        # A grader asking for many steps must not time out.
        t0 = time.monotonic()
        ok, v, _ = self.step(function="sin(t)*t^2 + exp(-t)", x0=0, dt=1e-4, steps=200000, integrator="rk4")
        self.assertTrue(ok)
        self.assertEqual(len(v["positions"]), 200001)
        self.assertLess(time.monotonic() - t0, 10.0)


# ---------------------------------------------------------------- simulation control

class TestSimControlDepth(_Base):
    def test_set_params_independent_fields_and_echo(self):
        for links in (2, 3):
            reg, st = _runtime(links)
            ok, v, _ = self.call(reg, "/arm_sim/set_params", {})
            self.assertTrue(ok)
            self.assertEqual(v, {"gravity": 9.81, "masses": [1.0] * links, "lengths": [1.0] * links})
            ok, v, s = self.call(reg, "/arm_sim/set_params",
                                 {"gravity": -1, "masses": [2.0] * links, "lengths": [0.5] * (links + 1)})
            self.assertFalse(ok)
            self.assertEqual(v["gravity"], 9.81)             # rejected field untouched
            self.assertEqual(v["masses"], [2.0] * links)     # valid field applied
            self.assertEqual(v["lengths"], [1.0] * links)
            ok, v, _ = self.call(reg, "/arm_sim/set_params", {"gravity": 0})
            self.assertTrue(ok)
            self.assertEqual(v["gravity"], 0)
            for bad in ({"masses": [0] * links}, {"masses": [-1] * links}, {"lengths": [1, "a", 1][:links]},
                        {"masses": [True] * links}, {"gravity": "9.8"}, {"gravity": None},
                        {"lengths": 1.0}):
                ok, v2, _ = self.call(reg, "/arm_sim/set_params", bad)
                self.assertFalse(ok, bad)
                self.assertIn("lengths", v2)  # always echoes, even on rejection
            ok, v, _ = self.call(reg, "/arm_sim/set_params", {"masses": [3] * links})  # ints are numbers
            self.assertTrue(ok)

    def test_set_integrator_fields(self):
        reg, st = _runtime(2)
        ok, v, _ = self.call(reg, "/arm_sim/set_integrator", {})
        self.assertTrue(ok)
        self.assertEqual(set(v), {"method", "timestep"})
        for m in METHODS:
            ok, v, _ = self.call(reg, "/arm_sim/set_integrator", {"method": m})
            self.assertTrue(ok)
            self.assertEqual(v["method"], m)
        for bad in ({"method": "Euler"}, {"method": "velocity verlet"}, {"timestep": 0},
                    {"timestep": -0.01}, {"timestep": "0.01"}, {"method": 3}):
            before = dict(self.call(reg, "/arm_sim/set_integrator", {})[1])
            ok, v, _ = self.call(reg, "/arm_sim/set_integrator", bad)
            self.assertFalse(ok, bad)
            self.assertEqual(v, before, f"{bad}: rejected request must not change anything")
        ok, v, _ = self.call(reg, "/arm_sim/set_integrator", {"method": "rk4", "timestep": 0.002})
        self.assertTrue(ok)
        self.assertEqual(v, {"method": "rk4", "timestep": 0.002})
        # All-or-nothing: a valid method alongside an invalid timestep applies neither.
        ok, v, _ = self.call(reg, "/arm_sim/set_integrator", {"method": "euler", "timestep": -1})
        self.assertFalse(ok)
        self.assertEqual(v, {"method": "rk4", "timestep": 0.002})

    def test_scientific_notation_literals(self):
        reg, _ = _runtime(2)
        for src, want in (("1e-3*t", 0.0), ("2.5E2", 250.0), ("2e+2", 200.0)):
            ok, v, s = self.call(reg, "/arm_sim/integration_step",
                                 {"function": src, "x0": 0, "dt": 1.0, "steps": 1, "integrator": "euler"})
            self.assertTrue(ok, f"{src}: {s}")
            self.assertAlmostEqual(v["velocities"][1], want)

    def test_pause_query_and_validation(self):
        reg, st = _runtime(2)
        self.assertEqual(self.call(reg, "/arm_sim/pause", {})[1], {"data": False})
        self.assertEqual(self.call(reg, "/arm_sim/pause", {"data": True})[1], {"data": True})
        self.assertEqual(self.call(reg, "/arm_sim/pause", {})[1], {"data": True})
        for bad in ({"data": 1}, {"data": "true"}, {"data": None}):
            ok, v, _ = self.call(reg, "/arm_sim/pause", bad)
            self.assertFalse(ok, bad)
            self.assertEqual(v, {"data": True})

    def test_reset_response_and_pid_state(self):
        for links in (2, 3):
            reg, st = _runtime(links)
            st.q, st.qdot, st.sim_time = [0.4] * links, [1.0] * links, 12.5
            st.controller.integral = [3.0] * links
            st.setpoint_pos = [0.9] * links
            ok, v, _ = self.call(reg, "/arm_sim/reset", {})
            self.assertTrue(ok)
            self.assertEqual(v, {"position": [0.0] * links, "velocity": [0.0] * links})
            self.assertEqual(st.sim_time, 0.0)
            self.assertEqual(st.controller.integral, [0.0] * links)
            self.assertEqual(st.setpoint_pos, [0.0] * links)

    def test_joint_trajectory_last_point_and_defaults(self):
        reg, st = _runtime(3)
        reg.publish("/joint_trajectory", {"joint_names": ["joint1", "joint2", "joint3"], "points": [
            {"positions": [9, 9, 9], "velocities": [9, 9, 9]},
            {"positions": [0.1, 0.2, 0.3], "velocities": [0.5, 0.5, 0.5]}]})
        self.assertEqual(st.setpoint_pos, [0.1, 0.2, 0.3])
        self.assertEqual(st.setpoint_vel, [0.5, 0.5, 0.5])
        reg.publish("/joint_trajectory", {"points": [{"positions": [0.7, 0.8, 0.9]}]})  # velocities omitted
        self.assertEqual(st.setpoint_pos, [0.7, 0.8, 0.9])
        self.assertEqual(st.setpoint_vel, [0.0, 0.0, 0.0])
        reg.publish("/joint_trajectory", {"points": [{"positions": [1, 2], "velocities": [1, 1, 1]}]})  # wrong length
        self.assertEqual(st.setpoint_pos, [0.0, 0.0, 0.0])
        self.assertEqual(st.setpoint_vel, [1.0, 1.0, 1.0])


# ---------------------------------------------------------------- PID services

class TestPidServicesDepth(_Base):
    def test_enable_semantics(self):
        reg, st = _runtime(2)
        self.assertEqual(self.call(reg, "/pid_controller/enable", {})[1], {"data": False})
        st.controller.integral = [5.0, 5.0]
        self.assertTrue(self.call(reg, "/pid_controller/enable", {"data": True})[0])
        self.assertEqual(st.controller.integral, [0.0, 0.0], "re-enable must clear the integral")
        st.controller.integral = [2.0, 2.0]
        self.call(reg, "/pid_controller/set_gains", {"kp": [1, 1], "ki": [1, 1], "kd": [1, 1]})
        self.assertEqual(st.controller.integral, [2.0, 2.0], "set_gains must NOT touch the integral")
        for bad in ({"data": 1}, {"data": "yes"}):
            self.assertFalse(self.call(reg, "/pid_controller/enable", bad)[0], bad)

    def test_set_gains_per_field(self):
        for links in (2, 3):
            reg, st = _runtime(links)
            ok, v, _ = self.call(reg, "/pid_controller/set_gains", {})
            self.assertTrue(ok)
            self.assertEqual(set(v), {"kp", "ki", "kd"})
            ok, v, _ = self.call(reg, "/pid_controller/set_gains",
                                 {"kp": [5] * links, "ki": [-1] * links, "kd": [1] * (links + 1)})
            self.assertFalse(ok)
            self.assertEqual(v["kp"], [5.0] * links)
            self.assertNotIn(-1, v["ki"])
            self.assertEqual(len(v["kd"]), links)
            ok, v, _ = self.call(reg, "/pid_controller/set_gains", {"ki": [0] * links})  # zero is allowed
            self.assertTrue(ok)
            self.assertEqual(v["ki"], [0.0] * links)


# ---------------------------------------------------------------- dynamics contract

class TestDynamicsDepth(unittest.TestCase):
    def test_equilibria_and_signs(self):
        for n in (2, 3):
            down = [-math.pi / 2] + [0.0] * (n - 1)
            for v in arm_dynamics.forward_dynamics(down, [0] * n, [0] * n, 9.81, [1] * n, [1] * n):
                self.assertAlmostEqual(v, 0.0, places=9)
            # Horizontal arm, no torque: every joint must start falling (q decreasing).
            qdd = arm_dynamics.forward_dynamics([0.0] * n, [0] * n, [0] * n, 9.81, [1] * n, [1] * n)
            self.assertLess(qdd[0], 0)
            # Zero gravity, at rest: nothing moves.
            self.assertEqual(arm_dynamics.forward_dynamics([0.3] * n, [0] * n, [0] * n, 0.0, [1] * n, [1] * n),
                             [0.0] * n)

    def test_tau_equals_G_holds_arm(self):
        rng = random.Random(1)
        for _ in range(50):
            n = rng.choice([2, 3])
            q = [rng.uniform(-3, 3) for _ in range(n)]
            m = [rng.uniform(0.2, 3) for _ in range(n)]
            L = [rng.uniform(0.2, 2) for _ in range(n)]
            g = arm_dynamics.G(q, 9.81, m, L)
            for v in arm_dynamics.forward_dynamics(q, [0] * n, g, 9.81, m, L):
                self.assertAlmostEqual(v, 0.0, places=8)

    def test_three_link_couples(self):
        # "A 3-link arm should couple more strongly than a 2-link one": moving
        # only joint 1 must accelerate the other joints (C couples them).
        qdd = arm_dynamics.forward_dynamics([0.3, 0.4, -0.2], [2.0, 0, 0], [0, 0, 0], 0.0, [1] * 3, [1] * 3)
        self.assertGreater(abs(qdd[1]), 1e-3)
        self.assertGreater(abs(qdd[2]), 1e-3)

    def test_parameters_are_respected(self):
        q, qd = [0.3, -0.2], [0.1, 0.2]
        a = arm_dynamics.forward_dynamics(q, qd, [0, 0], 9.81, [1, 1], [1, 1])
        for kw in ({"gravity": 1.62}, {"masses": [2, 0.5]}, {"lengths": [0.5, 1.5]}):
            args = {"gravity": 9.81, "masses": [1, 1], "lengths": [1, 1], **kw}
            b = arm_dynamics.forward_dynamics(q, qd, [0, 0], args["gravity"], args["masses"], args["lengths"])
            self.assertNotEqual(a, b, kw)

    def test_integrators_live_arm_energy(self):
        # Undriven, gravity on: energy should stay near constant for the
        # accurate methods at a small dt; Euler drifts the most.
        def energy(q, qd, m, L):
            Mq = arm_dynamics.M(q, m, L)
            n = len(q)
            T = 0.5 * sum(qd[a] * Mq[a][b] * qd[b] for a in range(n) for b in range(n))
            V = sum(m[k] * 9.81 * arm_dynamics._com_positions(q, L)[k][1] for k in range(n))
            return T + V
        m, L = [1, 1, 1], [1, 1, 1]
        drift = {}
        for name in METHODS:
            q, qd, t, dt = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.002
            e0 = energy(q, qd, m, L)
            f = lambda t_, q_, qd_: arm_dynamics.forward_dynamics(q_, qd_, [0, 0, 0], 9.81, m, L)
            for _ in range(500):
                q, qd = integrators.METHODS[name](f, t, q, qd, dt)
                t += dt
            drift[name] = abs(energy(q, qd, m, L) - e0)
        self.assertLess(drift["rk4"], 1e-4)
        self.assertGreater(drift["euler"], drift["rk4"])


# ---------------------------------------------------------------- IK

class TestIkDepth(_Base):
    def test_solve_roundtrip_random(self):
        rng = random.Random(2)
        for links in (2, 3):
            reg, st = _runtime(links)
            for _ in range(150):
                L = [rng.uniform(0.3, 2.0) for _ in range(links)]
                self.assertTrue(self.call(reg, "/arm_sim/set_params", {"lengths": L})[0])
                q = [rng.uniform(-math.pi, math.pi) for _ in range(links)]
                x, y, phi = kinematics.forward_kinematics(q, L)
                args = {"x": x, "y": y}
                if links == 3 and rng.random() < 0.5:
                    args["phi"] = phi
                ok, v, s = self.call(reg, "/ik/solve", args)
                self.assertTrue(ok, f"{args} L={L}: {s}")
                self.assertEqual(len(v["positions"]), links)
                X, Y, P = kinematics.forward_kinematics(v["positions"], L)
                self.assertAlmostEqual(X, x, places=6)
                self.assertAlmostEqual(Y, y, places=6)
                if "phi" in args:
                    self.assertAlmostEqual(_wrap(P - phi), 0.0, places=6)

    def test_solve_uses_fresh_lengths(self):
        reg, st = _runtime(2)
        self.assertTrue(self.call(reg, "/ik/solve", {"x": 1.9, "y": 0})[0])
        self.call(reg, "/arm_sim/set_params", {"lengths": [0.5, 0.5]})
        self.assertFalse(self.call(reg, "/ik/solve", {"x": 1.9, "y": 0})[0])

    def test_solve_boundaries_reachable(self):
        rng = random.Random(3)
        for links in (2, 3):
            reg, st = _runtime(links)
            for _ in range(100):
                L = [rng.uniform(0.3, 2.0) for _ in range(links)]
                self.call(reg, "/arm_sim/set_params", {"lengths": L})
                a = rng.uniform(-math.pi, math.pi)
                x, y, phi = kinematics.forward_kinematics([a] + [0.0] * (links - 1), L)  # fully extended
                args = {"x": x, "y": y, **({"phi": phi} if links == 3 else {})}
                ok, _, s = self.call(reg, "/ik/solve", args)
                self.assertTrue(ok, f"extended {args} L={L}: {s}")
                if links == 3:  # and without phi (audit: float error hit the too-far check)
                    ok, _, s = self.call(reg, "/ik/solve", {"x": x, "y": y})
                    self.assertTrue(ok, f"extended, no phi, ({x},{y}) L={L}: {s}")
                if links == 2:
                    x, y, _ = kinematics.forward_kinematics([a, math.pi], L)  # fully folded
                    ok, _, s = self.call(reg, "/ik/solve", {"x": x, "y": y})
                    self.assertTrue(ok, f"folded ({x},{y}) L={L}: {s}")

    def test_solve_rejections(self):
        reg, st = _runtime(2)
        self.call(reg, "/arm_sim/set_params", {"lengths": [2.0, 0.5]})
        for args in ({}, {"x": 1}, {"y": 1}, {"x": "1", "y": 0}, {"x": True, "y": 0},
                     {"x": float("nan"), "y": 0}, {"x": None, "y": 0},
                     {"x": 2.6, "y": 0}, {"x": 1.0, "y": 0}, {"x": 0, "y": 0}):
            self.assertFalse(self.call(reg, "/ik/solve", args)[0], args)
        self.assertTrue(self.call(reg, "/ik/solve", {"x": 1.5, "y": 1.0, "phi": "ignored"})[0],
                        "2-link ignores phi, even a malformed one")
        reg3, _ = _runtime(3)
        self.assertFalse(self.call(reg3, "/ik/solve", {"x": 3.5, "y": 0})[0])            # too far
        self.assertFalse(self.call(reg3, "/ik/solve", {"x": 2.5, "y": 0, "phi": math.pi})[0])  # wrist rule
        self.assertTrue(self.call(reg3, "/ik/solve", {"x": 2.5, "y": 0, "phi": 0.0})[0])
        self.assertTrue(self.call(reg3, "/ik/solve", {"x": 0.0, "y": 0.0})[0], "origin reachable by a 3-link unit arm")

    def test_send_goal_rejections_leave_active_goal(self):
        reg, st = _runtime(2)
        ok, v, _ = self.call(reg, "/ik_action/send_goal", {"x": 1.0, "y": 1.0})
        self.assertTrue(ok)
        gid = v["goal_id"]
        for bad in ({"x": 5, "y": 0}, {"x": 1.0}, {"x": 1.0, "y": 1.0, "epsilon": 0},
                    {"x": 1.0, "y": 1.0, "epsilon": -1}, {"x": 1.0, "y": 1.0, "success_hold": -1}):
            self.assertFalse(self.call(reg, "/ik_action/send_goal", bad)[0], bad)
        self.assertFalse(self.call(reg, "/ik_action/cancel_goal", {"goal_id": "nope"})[0])
        self.assertTrue(self.call(reg, "/ik_action/cancel_goal", {"goal_id": gid})[0])
        self.assertFalse(self.call(reg, "/ik_action/cancel_goal", {})[0], "no active goal left")


if __name__ == "__main__":
    unittest.main()
