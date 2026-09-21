"""Four numerical integrators for advancing a second-order system one timestep.

STUB: implement this file yourself. No external physics/ODE-solver libraries
(no `scipy.integrate`, no numpy-based ODE helpers, etc.) -- write the stepping
logic by hand.

This module backs two things, both of which must call into these exact same
four functions (per spec/PROJECT2_PENDULARM.md, "the arm's simulation node(s)
must actually call into them to advance the live arm's (q, qdot) every
physics tick"):
  - the standalone `/arm_sim/integration_step` checkpoint service (1-DOF,
    state represented as a length-1 list)
  - the live n-link arm's physics tick inside arm_sim_node.py (n-DOF, state
    is one entry per joint)

State representation: `q` and `qdot` are always `list[float]`, one entry per
degree of freedom (length 1 for the 1-DOF checkpoint case, length n for the
live arm). This keeps one code path for both use sites instead of a separate
scalar special case.

The system being integrated is `qddot = f(t, q, qdot)` -- second-order, and
generally depends on the *current* q and qdot (not just t), which is why
`accel_fn` takes all three. (The `/arm_sim/integration_step` checkpoint's
`function` field only varies with `t`, but that's just a degenerate case of
this same general signature -- `accel_fn` there simply ignores its `q`/`qdot`
arguments.)

Critical correctness requirement, called out explicitly in the spec: each
method's sub-stages must evaluate `accel_fn` at the *correct fractional
time* (`t`, `t + dt/2`, `t + dt`, whichever apply to that method) using a
state estimate appropriate to that fractional time -- not `t` frozen across
the whole step. Freezing `t` degrades midpoint/verlet/rk4 toward Euler's
accuracy on a genuinely time-varying `accel_fn` (e.g. `f(t) = t`, whose
closed form from rest is `x(t) = t**3/6`) without being obviously wrong on
constant or state-only-dependent forcing -- exactly the kind of bug that's
easy to introduce and hard to notice.

Each function returns `(q_next, qdot_next)` after advancing by `dt` from
`(q, qdot)` at time `t`. `accel_fn(t, q, qdot) -> list[float]` returns an
acceleration vector the same length as `q`.
"""
from __future__ import annotations

from typing import Callable

State = list[float]
AccelFn = Callable[[float, State, State], State]


def euler(accel_fn: AccelFn, t: float, q: State, qdot: State, dt: float) -> tuple[State, State]:
    """Forward Euler: one acceleration evaluation, at the current state.

    First-order accurate. Should visibly lose accuracy faster than the other
    three methods on the same input -- that's expected, not a bug to smooth
    over (see spec/PROJECT2_PENDULARM.md, "Numerical integrators").
    """
    qNew = [q_i + qdot_i * dt for q_i, qdot_i in zip(q, qdot)]
    f = accel_fn(t, q, qdot)
    qdotNew = [qdot_i + f_i * dt for qdot_i, f_i in zip(qdot, f)]
    return (qNew, qdotNew)


def midpoint(accel_fn: AccelFn, t: float, q: State, qdot: State, dt: float) -> tuple[State, State]:
    """Midpoint method: evaluate acceleration at the current state, take a
    trial half-step to estimate the state at t + dt/2, evaluate acceleration
    there, then use that midpoint value to take the real full step.

    Two acceleration evaluations per step.
    """
    step = (dt/2)
    q0 = [q_i + qdot_i * step for q_i, qdot_i in zip(q, qdot)]
    f0 = accel_fn(t, q, qdot)
    qdot0 = [qdot_i + f_i * step for qdot_i, f_i in zip(qdot, f0)]
    
    
    q1 = [q1_i + qdot1_i * dt for q1_i, qdot1_i in zip(q, qdot0)]
    f = accel_fn(t + step, q0, qdot0)
    qdot1 = [q1_i + f_i * dt for q1_i, f_i in zip(qdot, f)]
    return (q1, qdot1)


def verlet(accel_fn: AccelFn, t: float, q: State, qdot: State, dt: float) -> tuple[State, State]:
    """Velocity Verlet (predictor-corrector variant, for velocity-dependent forces):
    predict the new position using the current velocity and acceleration,
    provisionally estimate the new velocity, evaluate acceleration again at
    that predicted state, then finalize the velocity by averaging the two
    acceleration evaluations.

    Two acceleration evaluations per step. (This is velocity Verlet rather
    than the textbook position-only Verlet specifically because this
    project's forces depend on velocity as well as position -- plain Verlet
    has nowhere to plug in a velocity-dependent force.)
    """
    qddot0 = accel_fn(t, q, qdot)
    q1 = [q0_i + qdot0_i * dt + 0.5 * qddot0_i * dt**2 for q0_i, qdot0_i, qddot0_i in zip(q, qdot, qddot0)]
    qdot1_prov = [qdot0_i + qddot0_i * dt for qdot0_i, qddot0_i in zip(qdot, qddot0)]
    qddot1 = accel_fn(t + dt, q1, qdot1_prov)
    qdot1 = [qdot0_i + 0.5*(qddot0_i + qddot1_i) * dt for qdot0_i, qddot0_i, qddot1_i in zip(qdot, qddot0, qddot1)]
    return (q1, qdot1)
    


def rk4(accel_fn: AccelFn, t: float, q: State, qdot: State, dt: float) -> tuple[State, State]:
    """Classical 4th-order Runge-Kutta: four acceleration evaluations per
    step (at the start, twice near the midpoint from two different trial
    states, and at the end), blended with 1:2:2:1 weights.

    The most accurate of the four per step, at the highest cost per step.
    """
    #k1 = f(t_i, x_i)
    qddot0 = accel_fn(t, q, qdot)
    k1 = (qdot, qddot0)
    #k2 = f(t_i + dt/2, x_i + k1 * dt/2)
    q1 = [q0_i + qdot0_i  * dt/2 for q0_i, qdot0_i in zip(q, k1[0])]
    qdot1 = [qdot0_i + qddot0_i * dt/2 for qdot0_i, qddot0_i in zip(qdot, k1[1])]
    qddot1 = accel_fn(t + dt/2, q1, qdot1)
    k2 = (qdot1, qddot1)
    #k3 = f(t_i + dt/2, x_i + k2 * dt/2)
    q2 = [q0_i + qdot0_i  * dt/2 for q0_i, qdot0_i in zip(q, k2[0])]
    qdot2 = [qdot0_i + qddot0_i * dt/2 for qdot0_i, qddot0_i in zip(qdot, k2[1])]
    qddot2 = accel_fn(t + dt/2, q2, qdot2)
    k3 = (qdot2, qddot2)
    #k4 = f(t_i + dt, x_i + k3 * dt)
    q3 = [q0_i + qdot0_i  * dt for q0_i, qdot0_i in zip(q, k3[0])]
    qdot3 = [qdot0_i + qddot0_i * dt for qdot0_i, qddot0_i in zip(qdot, k3[1])]
    qddot3 = accel_fn(t + dt, q3, qdot3)
    k4 = (qdot3, qddot3)
    #x_i+1 = (dt/6)(k1 + 2k2 + 2k3 + k4)
    q_final = [q_i + (dt/6) * (k1qdot_i + 2 * k2qdot_i + 2 * k3qdot_i + k4qdot_i) for q_i, k1qdot_i, k2qdot_i, k3qdot_i, k4qdot_i in zip(q, k1[0], k2[0], k3[0], k4[0])]
    qdot_final = [qdot_i + (dt/6) * (k1qddot_i + 2 * k2qddot_i + 2 * k3qddot_i + k4qddot_i) for qdot_i, k1qddot_i, k2qddot_i, k3qddot_i, k4qddot_i in zip(qdot, k1[1], k2[1], k3[1], k4[1])]
    return (q_final, qdot_final)

METHODS: dict[str, Callable[[AccelFn, float, State, State, float], tuple[State, State]]] = {
    "euler": euler,
    "midpoint": midpoint,
    "verlet": verlet,
    "rk4": rk4,
}
"""Name -> function lookup, for /arm_sim/set_integrator and /arm_sim/integration_step
to select a method by the string names the spec uses. Pure wiring, not part
of the graded algorithm -- provided as-is."""
