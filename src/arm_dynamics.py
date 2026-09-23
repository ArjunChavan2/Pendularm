"""Lagrangian forward dynamics for a serial, planar, n-link ("RR...R") arm.

STUB: implement this file yourself. This module holds the manipulator
equation's mass matrix M(q), Coriolis/centrifugal term C(q, qdot), gravity
load G(q), and the forward-dynamics solve

    qddot = M(q)^-1 (tau - C(q, qdot) qdot - G(q))

derived from Lagrangian mechanics for a *general* n-link arm (n == 2 or 3),
not two separately hand-derived 2-link/3-link special cases -- see
spec/PROJECT2_PENDULARM.md, "Dynamics." No external physics/ODE-solver or
linear-algebra libraries (no `scipy`, no numpy-based solve, etc.) -- derive
and implement this by hand, same standing rule as src/integrators.py,
src/pid.py, and src/kinematics.py.

This module backs `arm_sim_node.py`'s live physics loop (`physics_loop`),
which calls `forward_dynamics` every tick, feeding its result into
`integrators.METHODS[...]` to advance the live arm's (q, qdot). Only
`forward_dynamics`'s signature/contract is specified here (agreed during
planning, see agent-notes/PLAN.md, "Interfaces and data flow") -- the
internal M/C/G derivation and the linear-algebra method used to solve/invert
M(q) are entirely up to you.
"""
from __future__ import annotations


def forward_dynamics(
    q: list[float],
    qdot: list[float],
    tau: list[float],
    gravity: float,
    masses: list[float],
    lengths: list[float],
) -> list[float]:
    """Solve the manipulator equation for joint acceleration:

        qddot = M(q)^-1 (tau - C(q, qdot) qdot - G(q))

    for a serial, planar, n-link ("RR...R") arm, derived from Lagrangian
    mechanics for general n (not separately hand-derived per link count).

    q_i is joint i's own relative rotation; link i's absolute world
    orientation is phi_i = q_1 + ... + q_i. Link i is a uniform rigid rod of
    length lengths[i] and mass masses[i], hinged to link i-1 (link 0 hinged
    to a fixed base at a static frame's origin). Gravity acts along world -y
    with magnitude `gravity`.

    Args:
      q, qdot, tau: length-n lists (joint angles, joint angular velocities,
        applied joint efforts), n == 2 or 3.
      gravity: scalar g >= 0.
      masses, lengths: length-n lists, each entry > 0.

    Returns:
      qddot: length-n list, the resulting joint angular accelerations.

    Invariants the implementation must satisfy (not enforced by this
    signature, but part of the contract callers may rely on):
      - M(q) is symmetric and positive-definite for any physically valid
        input (masses/lengths > 0), so a solution always exists.
      - Called with tau = [0]*n and any q where G(q) != 0, the returned
        qddot must be nonzero (this is the spec's own stated correctness
        check for the "arm swings freely under gravity" milestone).
      - Purely a function of (q, qdot, tau, gravity, masses, lengths) -- no
        explicit time dependence, no internal state/memory between calls.

    Not part of the contract, left to you: whether M(q)/C(q, qdot)/G(q) are
    computed as separate internal helper functions or inline; any internal
    caching; the exact linear-algebra method used to solve/invert M(q)
    (Gauss-Jordan with partial pivoting works fine, but so does anything
    else) -- a caller outside this module must never depend on any of that.
    """
    raise NotImplementedError("arm_dynamics.forward_dynamics: implement by hand (see module docstring)")
