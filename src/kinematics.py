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
from math import sin, cos, acos, atan2, sqrt, pi


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


def _two_link_ik(x: float, y: float, l1: float, l2: float) -> list[float]:
    """Joint angles [q1, q2] placing a 2-link arm's tip at (x, y).

    Law of Cosines: cos q2 = (r^2 - l1^2 - l2^2) / (2 l1 l2), with
    r^2 = x^2 + y^2; then q1 = atan2(y, x) - atan2(l2 sin q2, l1 + l2 cos q2).
    Either elbow configuration is acceptable.

    Raises Unreachable if (x, y) is farther than l1 + l2 or closer than
    |l1 - l2| -- beyond a tiny floating-point tolerance; the exact boundary
    (fully extended / fully folded) counts as reachable, so clamp cos q2
    into [-1, 1] before acos.

    Example: (1, 1) with l1 = l2 = 1 -> [0, pi/2] (elbow-down) or
    [pi/2, -pi/2] (elbow-up).

    Also used by inverse_kinematics for a 3-link arm's wrist point.
    """
    rsq = x**2 + y**2
    k = (rsq - l1**2 - l2**2) / (2 * l1 * l2)
    if k < -(1):
      if k > -(1 + 1e-9):
        k = -1
      else:
        raise Unreachable("Unreachable destination: too close")
    if k > (1):
      if k < (1 + 1e-9):
        k = 1
      else:
        raise Unreachable("Unreachable destination: too far")
    q2 = acos(k)
    a = atan2(y, x)
    b = atan2(l2 * sin(q2), l1 + l2 * cos(q2))
    q1 = a - b
    return [q1, q2]


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
    n = len(lengths)
    if n == 2:
      return _two_link_ik(x, y, lengths[0], lengths[1])
    
    if phi is not None:
      wx, wy = x - lengths[2] * cos(phi), y - lengths[2] * sin(phi)
      q = _two_link_ik(wx, wy, lengths[0], lengths[1])
      q.append(phi - sum(q))
      return q
    else:
      dest = sqrt(x**2 + y**2)
      if dest > sum(lengths) * (1 + 1e-9):
        raise Unreachable("Destination is too far away")

      phi = atan2(y, x)
      for i in range(0, 360, 5):
        try:
          return inverse_kinematics(x, y, lengths, phi + ((i * pi) / 180))
        except Unreachable:
          continue
      raise Unreachable("Phi couldnt be found")