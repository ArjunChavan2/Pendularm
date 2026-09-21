"""Contract tests for src/integrators.py -- expected to fail until you implement it.

Test strategy, per spec/PROJECT2_PENDULARM.md:
  1. Constant acceleration has a known closed form (x = x0 + v0*t + 0.5*a*t^2)
     that midpoint, verlet, and rk4 should reproduce essentially exactly (they
     integrate a constant-coefficient quadratic exactly); euler should not
     (it's only first-order accurate).
  2. f(t) = t from rest has closed form x(t) = t**3/6, v(t) = t**2/2 -- the
     spec's own example for catching a method that "freezes t" across a step.
     rk4 should be accurate to machine precision on this case (it integrates
     a cubic exactly); midpoint/verlet should be much closer than euler.
  3. Halving dt should roughly quarter midpoint's error (2nd-order
     convergence) -- a method that freezes t degrades toward 1st-order
     (halving dt only halves error), so this also indirectly catches that bug.
  4. A directly targeted check, using a "spy" accel_fn that records every
     (t, q, qdot) it's called with: each method must evaluate at the correct
     fractional times in the correct order, not just get a numerically
     plausible-looking answer.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import integrators  # noqa: E402


class _Spy:
    """Records every (t, q, qdot) call; returns a fixed constant acceleration."""

    def __init__(self, value=1.0):
        self.calls = []
        self._value = value

    def __call__(self, t, q, qdot):
        self.calls.append((t, list(q), list(qdot)))
        return [self._value] * len(q)


class TestConstantAcceleration(unittest.TestCase):
    """midpoint/verlet/rk4 should be exact (to float precision) for constant a;
    euler should not be (it's missing the 0.5*a*dt**2 position term)."""

    A = 3.0
    Q0 = [0.0]
    V0 = [1.0]
    DT = 0.1
    STEPS = 10

    def _accel(self, t, q, qdot):
        return [self.A]

    def _run(self, method):
        q, v, t = list(self.Q0), list(self.V0), 0.0
        for _ in range(self.STEPS):
            q, v = method(self._accel, t, q, v, self.DT)
            t += self.DT
        return q, v

    def _exact(self):
        T = self.STEPS * self.DT
        q_exact = self.Q0[0] + self.V0[0] * T + 0.5 * self.A * T * T
        v_exact = self.V0[0] + self.A * T
        return q_exact, v_exact

    def test_midpoint_exact(self):
        q, v = self._run(integrators.midpoint)
        q_exact, v_exact = self._exact()
        self.assertAlmostEqual(q[0], q_exact, places=9)
        self.assertAlmostEqual(v[0], v_exact, places=9)

    def test_verlet_exact(self):
        q, v = self._run(integrators.verlet)
        q_exact, v_exact = self._exact()
        self.assertAlmostEqual(q[0], q_exact, places=9)
        self.assertAlmostEqual(v[0], v_exact, places=9)

    def test_rk4_exact(self):
        q, v = self._run(integrators.rk4)
        q_exact, v_exact = self._exact()
        self.assertAlmostEqual(q[0], q_exact, places=9)
        self.assertAlmostEqual(v[0], v_exact, places=9)

    def test_euler_not_exact_but_velocity_is(self):
        # Euler's velocity update (v += a*dt) happens to be exact for constant
        # a; its position update (q += v*dt, missing the 0.5*a*dt**2 term) is
        # not -- that gap is the whole reason to include euler in this project.
        q, v = self._run(integrators.euler)
        q_exact, v_exact = self._exact()
        self.assertAlmostEqual(v[0], v_exact, places=9)
        self.assertGreater(abs(q[0] - q_exact), 0.05)


class TestTimeVaryingForcingAccuracy(unittest.TestCase):
    """f(t) = t from rest: closed form x(t) = t**3/6, v(t) = t**2/2. The
    spec's own example for a method that freezes t across a step."""

    def _accel(self, t, q, qdot):
        return [t]

    def _run(self, method, steps, dt):
        q, v, t = [0.0], [0.0], 0.0
        for _ in range(steps):
            q, v = method(self._accel, t, q, v, dt)
            t += dt
        return q[0], v[0]

    def test_rk4_matches_closed_form_to_machine_precision(self):
        q, v = self._run(integrators.rk4, steps=10, dt=0.1)
        self.assertAlmostEqual(q, 1.0 ** 3 / 6, places=9)
        self.assertAlmostEqual(v, 1.0 ** 2 / 2, places=9)

    def test_accuracy_ordering_rk4_beats_midpoint_beats_euler(self):
        q_exact, v_exact = 1.0 ** 3 / 6, 1.0 ** 2 / 2
        q_euler, _ = self._run(integrators.euler, steps=10, dt=0.1)
        q_mid, _ = self._run(integrators.midpoint, steps=10, dt=0.1)
        q_verlet, _ = self._run(integrators.verlet, steps=10, dt=0.1)
        q_rk4, _ = self._run(integrators.rk4, steps=10, dt=0.1)

        err_euler = abs(q_euler - q_exact)
        err_mid = abs(q_mid - q_exact)
        err_verlet = abs(q_verlet - q_exact)
        err_rk4 = abs(q_rk4 - q_exact)

        self.assertGreater(err_euler, 0.03)  # euler should be visibly off
        self.assertLess(err_mid, err_euler / 10)  # midpoint much closer
        self.assertLess(err_verlet, err_euler / 10)
        self.assertLess(err_rk4, 1e-6)  # rk4 essentially exact here

    def test_midpoint_convergence_order_halving_dt_quarters_error(self):
        # A method that (correctly) evaluates at t+dt/2 is 2nd-order: halving
        # dt should cut the error by ~4x. A method that freezes t degrades to
        # 1st-order (only ~2x), so this also indirectly catches that bug.
        q_exact = 1.0 ** 3 / 6
        q_coarse, _ = self._run(integrators.midpoint, steps=10, dt=0.1)
        q_fine, _ = self._run(integrators.midpoint, steps=20, dt=0.05)
        err_coarse = abs(q_coarse - q_exact)
        err_fine = abs(q_fine - q_exact)
        self.assertGreater(err_coarse, 0.0)  # sanity: not a degenerate zero-error case
        self.assertLess(err_fine, err_coarse / 3)  # expect ~4x; generous margin


class TestFractionalTimeEvaluation(unittest.TestCase):
    """Directly checks *which* (t, ...) each method calls accel_fn with,
    rather than inferring it indirectly from accuracy -- catches a frozen-t
    bug even if it happens to still look numerically plausible."""

    T0 = 2.0
    DT = 0.4

    def test_euler_evaluates_once_at_t(self):
        spy = _Spy()
        integrators.euler(spy, self.T0, [0.0], [0.0], self.DT)
        self.assertEqual(len(spy.calls), 1)
        self.assertAlmostEqual(spy.calls[0][0], self.T0)

    def test_midpoint_evaluates_at_t_then_t_plus_half_dt(self):
        spy = _Spy()
        integrators.midpoint(spy, self.T0, [0.0], [0.0], self.DT)
        self.assertEqual(len(spy.calls), 2)
        self.assertAlmostEqual(spy.calls[0][0], self.T0)
        self.assertAlmostEqual(spy.calls[1][0], self.T0 + self.DT / 2)

    def test_verlet_evaluates_at_t_then_t_plus_dt(self):
        spy = _Spy()
        integrators.verlet(spy, self.T0, [0.0], [0.0], self.DT)
        self.assertEqual(len(spy.calls), 2)
        self.assertAlmostEqual(spy.calls[0][0], self.T0)
        self.assertAlmostEqual(spy.calls[1][0], self.T0 + self.DT)

    def test_rk4_evaluates_at_correct_fractional_times_in_order(self):
        spy = _Spy()
        integrators.rk4(spy, self.T0, [0.0], [0.0], self.DT)
        self.assertEqual(len(spy.calls), 4)
        expected_times = [self.T0, self.T0 + self.DT / 2, self.T0 + self.DT / 2, self.T0 + self.DT]
        for (actual_t, _, _), expected_t in zip(spy.calls, expected_times):
            self.assertAlmostEqual(actual_t, expected_t)


class TestVelocityDependentForce(unittest.TestCase):
    """Constant-acceleration and pure-time-varying accel_fns (used elsewhere in this
    file) ignore their qdot argument entirely, so they can't catch a method that
    passes the *wrong* velocity into a later accel_fn call -- e.g. verlet's second
    evaluation must use its provisionally-estimated velocity, not the original one.
    A simple velocity-dependent force (qddot = -qdot, i.e. damping) makes the
    exact qdot passed in observable."""

    def test_verlet_second_evaluation_uses_provisional_velocity(self):
        calls = []

        def accel_fn(t, q, qdot):
            calls.append((t, list(q), list(qdot)))
            return [-v for v in qdot]

        integrators.verlet(accel_fn, 0.0, [0.0], [2.0], 0.1)

        self.assertEqual(len(calls), 2)
        # first call: original state
        self.assertAlmostEqual(calls[0][2][0], 2.0)
        # second call must NOT reuse the original qdot (2.0) -- it must use the
        # provisional velocity estimate (qdot + qddot0*dt = 2.0 + (-2.0)*0.1 = 1.8)
        self.assertNotAlmostEqual(calls[1][2][0], 2.0)
        self.assertAlmostEqual(calls[1][2][0], 1.8)


class TestVectorState(unittest.TestCase):
    """Multi-DOF (n=2) sanity check: two decoupled constant accelerations,
    verified componentwise against rk4's exactness for constant acceleration."""

    def test_two_dof_constant_acceleration(self):
        accels = [2.0, -5.0]

        def accel_fn(t, q, qdot):
            return list(accels)

        q, v, t, dt, steps = [0.0, 0.0], [1.0, -1.0], 0.0, 0.1, 10
        for _ in range(steps):
            q, v = integrators.rk4(accel_fn, t, q, v, dt)
            t += dt

        T = steps * dt
        for i in range(2):
            q_exact = 0.0 + [1.0, -1.0][i] * T + 0.5 * accels[i] * T * T
            v_exact = [1.0, -1.0][i] + accels[i] * T
            self.assertAlmostEqual(q[i], q_exact, places=9)
            self.assertAlmostEqual(v[i], v_exact, places=9)


class TestMethodRegistry(unittest.TestCase):
    def test_all_four_methods_registered(self):
        self.assertEqual(set(integrators.METHODS.keys()), {"euler", "midpoint", "verlet", "rk4"})
        self.assertIs(integrators.METHODS["euler"], integrators.euler)
        self.assertIs(integrators.METHODS["midpoint"], integrators.midpoint)
        self.assertIs(integrators.METHODS["verlet"], integrators.verlet)
        self.assertIs(integrators.METHODS["rk4"], integrators.rk4)


if __name__ == "__main__":
    unittest.main()
