"""Forward and closed-form inverse kinematics for the planar 2-/3-link arm.

STUB: implement this file yourself. Same joint-angle convention as
arm_dynamics.py: q_i is joint i's relative rotation, phi_i = q_1 + ... + q_i
is link i's absolute heading, and the end effector sits at

    x = sum(l_i * cos(phi_i)),   y = sum(l_i * sin(phi_i)),   orientation = phi_n

(spec/PROJECT2_PENDULARM.md, "Inverse kinematics"). IK is closed-form: Law of
Cosines for a 2-link position problem, plus kinematic decoupling (subtract
link 3's own contribution to get the wrist point) for a 3-link arm's
orientation. Service-level concerns -- parsing /ik/solve requests, fetching
current link lengths, result:false responses -- live in the service layer;
this module only does the math and signals "unreachable" by raising
Unreachable with a human-readable reason (the service layer turns that into
the response's `status`).
"""
from __future__ import annotations
from math import sin, cos


class Unreachable(ValueError):
    """Raised when a requested IK target can't be reached; str(e) is the reason."""


def forward_kinematics(q: list[float], lengths: list[float]) -> tuple[float, float, float]:
    """End-effector pose (x, y, phi) for joint angles q.

    Example: q = [0, 0], lengths = [1, 1] -> (2.0, 0.0, 0.0).
    """
    phi, x, y = 0, 0, 0
    n = len(q)
    for k in range(n):
      phi += q[k]
      x += lengths[k] * cos(phi)
      y += lengths[k] * sin(phi)
    return (x, y, phi)


def inverse_kinematics(x: float, y: float, lengths: list[float],
                       phi: float | None = None) -> list[float]:
    """Joint angles q (length n = len(lengths), 2 or 3) placing the end effector at (x, y).

    n == 2: phi is ignored (position alone uses both DOF). Law of Cosines;
      either elbow configuration is acceptable.
    n == 3: phi is the desired end-effector orientation (the last link's
      absolute heading). Subtract link 3's own contribution to get the wrist
      point, solve the 2-link problem for it, then choose q_3 so the headings
      sum to phi. If phi is None, any orientation that reaches (x, y) is
      acceptable.

    Raises Unreachable (never returns garbage/NaN) when:
      - the target is farther than sum(lengths);
      - n == 2 and the target is closer than |l1 - l2|;
      - n == 3 and the requested phi puts the wrist point outside links
        1-2's reach.
    The exact boundary (fully extended, or for n == 2 fully folded) counts as
    reachable: clamp tiny floating-point overshoot rather than rejecting it.

    Check (the grader's own property): forward_kinematics(result, lengths)
    reproduces (x, y) -- and phi, when n == 3 and phi was given.
    """
    raise NotImplementedError("kinematics.inverse_kinematics: implement by hand (see module docstring)")
