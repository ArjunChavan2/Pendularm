"""Lagrangian forward dynamics for a serial, planar, n-link ("RR...R") arm.

STUB: implement this file yourself. This module holds the manipulator
equation's mass matrix M(q), Coriolis/centrifugal matrix C(q, qdot), gravity
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
`forward_dynamics`'s signature/contract (and, now, `M`/`C`/`G` as their own
named functions) is specified here (agreed during planning, see
agent-notes/PLAN.md, "Interfaces and data flow") -- the internal derivation
and the linear-algebra method used to solve/invert M(q) are entirely up to
you.

Shared convention for every function below: q_i is joint i's own relative
rotation; link i's absolute world orientation is phi_i = q_1 + ... + q_i.
Link i is a uniform rigid rod of length lengths[i] and mass masses[i],
hinged to link i-1 (link 0 hinged to a fixed base at a static frame's
origin). Gravity acts along world -y with magnitude `gravity`. n = len(q)
(== len(qdot) == len(masses) == len(lengths)), and is 2 or 3.
"""
from __future__ import annotations
from math import sin, cos


def _com_positions(q: list[float], lengths: list[float]) -> list[tuple[float, float]]:
    """World (x, y) position of each link's center of mass.

    Link k's center of mass is a full arrow l_i * (cos phi_i, sin phi_i) for
    every link i before k, plus a half arrow 0.5 * l_k * (cos phi_k, sin phi_k)
    for link k itself (same phi_i convention as the module docstring).

    Returns:
      A length-n list; element k is link k's center-of-mass (x, y).

    Example: q = [0, 0], lengths = [1, 1] -> [(0.5, 0.0), (1.5, 0.0)].

    Internal helper (not part of forward_dynamics's contract): used by G(q)
    for each link's height, and handy for numerically checking M's Jacobians
    by finite differences.
    """
    l = len(lengths)
    if l == 0:
      return []
    phi = q[0]
    ret = [(0.5 * lengths[0] * cos(phi), 0.5 * lengths[0] * sin(phi))]
    last = ret[0]
    
    for i in range(1, l):
      phi += q[i]
      com_i = [ret[-1][0] + last[0], ret[-1][1] + last[1]]
      last = [0.5 * lengths[i] * cos(phi), 0.5 * lengths[i] * sin(phi)]
      com_i[0] += last[0]
      com_i[1] += last[1]
      ret.append((com_i[0], com_i[1]))
      
    return ret



def _com_jacobian(q: list[float], lengths: list[float], k: int) -> list[list[float]]:
    """Jacobian of link k's center of mass with respect to q (k is 0-indexed).

    Returns:
      A 2 x n matrix (list of 2 rows, each a length-n list): row 0 is
      [d x_k / d q_j for j in range(n)], row 1 is [d y_k / d q_j ...], where
      (x_k, y_k) is _com_positions(q, lengths)[k]. So link k's center-of-mass
      velocity is J @ qdot. Column j is all zeros for j > k (a joint past
      link k doesn't move it).

    Example: q = [0, 0], lengths = [1, 1]:
      k = 0 -> [[0.0, 0.0], [0.5, 0.0]]
      k = 1 -> [[0.0, 0.0], [1.5, 0.5]]

    Check: each column j should match the finite difference
    (_com_positions(q with q[j] += eps)[k] - _com_positions(q)[k]) / eps
    for a small eps (e.g. 1e-6), at any q.
    """
    n = len(q)
    if n == 0:
      return []
    phi = sum(q[0:k + 1])
    J = [[0 for i in range(n)] for j in range(2)]
    J[0][k], J[1][k] = -0.5 * lengths[k] * sin(phi), 0.5 * lengths[k] * cos(phi)
    phi -= q[k]
    for i in range(k - 1, -1, -1):
      J[0][i] = J[0][i + 1] + -lengths[i] * sin(phi)
      J[1][i] = J[1][i + 1] + lengths[i] * cos(phi)
      phi -= q[i]
    return J


def _com_jacobian_dq(q: list[float], lengths: list[float], k: int, c: int) -> list[list[float]]:
    """Derivative of _com_jacobian(q, lengths, k) with respect to q[c].

    Returns:
      A 2 x n matrix, same layout as _com_jacobian: entry [r][j] is
      d(J_k[r][j]) / d q_c. Per entry, the sum runs over links i from
      max(j, c) to k (empty -> 0), each term getting the sin/cos swap.

    Example: q = [0, 0], lengths = [1, 1]:
      k = 0, c = 0 -> [[-0.5, 0.0], [0.0, 0.0]]
      k = 1, c = 0 -> [[-1.5, -0.5], [0.0, 0.0]]
      k = 1, c = 1 -> [[-0.5, -0.5], [0.0, 0.0]]
      k = 0, c = 1 -> all zeros (joint 1 is past link 0)

    Check: should match (_com_jacobian(q with q[c] += eps, lengths, k)
    - _com_jacobian(q, lengths, k)) / eps for a small eps, at any q.
    """
    n = len(q)
    J = [[0 for i in range(n)] for j in range(2)]
    if n == 0 or c > k:
      return J
    phi = sum(q[0:k + 1])
    J[0][k], J[1][k] = -0.5 * lengths[k] * cos(phi), -0.5 * lengths[k] * sin(phi)
    phi -= q[k]
    for i in range(k - 1, -1, -1):
      if i >= c:
        J[0][i] += -lengths[i] * cos(phi)
        J[1][i] += -lengths[i] * sin(phi)
      J[0][i] += J[0][i + 1]
      J[1][i] += J[1][i + 1]
      phi -= q[i]
    return J



def M(q: list[float], masses: list[float], lengths: list[float]) -> list[list[float]]:
    """The n x n mass/inertia matrix M(q).

    Returns:
      An n x n matrix (list of n rows, each a length-n list) such that
      M(q) qddot is the generalized-force contribution from acceleration
      alone (i.e. the coefficient matrix on qddot in the manipulator
      equation).

    Invariant: for any physically valid input (masses/lengths all > 0),
    M(q) must be symmetric and positive-definite, so it's always invertible.
    """
    n = len(q)
    M = [[0 for i in range(n)] for j in range(n)]
    
    for k in range(0, n):
      J = _com_jacobian(q, lengths, k)
      w = [1 for i in range(k + 1)]
      w += [0 for i in range(n - k - 1)]
      I = masses[k] * lengths[k]**2 / 12
      for a in range(0, n):
        for b in range(0, n):
          M[a][b] += masses[k]*(J[0][a] * J[0][b] + J[1][a] * J[1][b]) + I * w[a]*w[b]
        
    return M


def _dM_dq(q: list[float], masses: list[float], lengths: list[float], c: int) -> list[list[float]]:
    """Derivative of M(q, masses, lengths) with respect to q[c].

    Returns:
      An n x n matrix; entry [a][b] is d M[a][b] / d q_c. Only M's
      translation part (m_k * J_k^T J_k) depends on q -- the rotation part
      (I_k * w_k^T w_k) is constant -- so this is the product rule applied
      to that part, with J' = _com_jacobian_dq(q, lengths, k, c).

    Example: q = [0, pi/2], masses = [1, 1], lengths = [1, 1]:
      c = 0 -> all zeros (turning the base never changes M)
      c = 1 -> [[-1.0, -0.5], [-0.5, 0.0]]

    Check: should match (M(q with q[c] += eps) - M(q)) / eps for a small
    eps, at any q; and should always be symmetric, like M.
    """
    n = len(q)
    dM = [[0 for i in range(n)] for j in range(n)]
    for k in range(n):
      J = _com_jacobian(q, lengths, k)
      dJ = _com_jacobian_dq(q, lengths, k, c)
      for a in range(n):
        for b in range(n):
          dM[a][b] += masses[k] * (J[0][a] * dJ[0][b] + dJ[0][a] * J[0][b] + 
                                   J[1][a] * dJ[1][b] + dJ[1][a] * J[1][b])
      
    return dM


def C(q: list[float], qdot: list[float], masses: list[float], lengths: list[float]) -> list[list[float]]:
    """The n x n Coriolis/centrifugal matrix C(q, qdot).

    Derived from the Christoffel symbols of M(q) (see spec/PROJECT2_PENDULARM.md,
    "Dynamics": "C(q, qdot) qdot ... coming from the Christoffel symbols of
    M(q) -- this is what lets one joint's motion push on the others").

    Returns:
      An n x n matrix such that C(q, qdot) @ qdot is the Coriolis/centrifugal
      generalized-force vector (what `forward_dynamics` actually needs).

    Note: a 3-link arm should couple more strongly than a 2-link one -- if
    C(q, qdot) is accidentally zero (or the coupling degenerates to n
    independent single-joint terms), that's a sign this isn't really wired
    in per the spec's own stated check.
    """
    n = len(q)
    C = [[0 for i in range(n)] for j in range(n)]
    dMs = [[0] for j in range(n)]
    for c in range(n):
      dMs[c] = _dM_dq(q, masses, lengths, c)

    for i in range(n):
      for j in range(n):
        for k in range(n):
          C[i][j] += 0.5*(dMs[k][i][j] + dMs[j][i][k] - dMs[i][j][k]) * qdot[k]
    
    return C
      


def G(q: list[float], gravity: float, masses: list[float], lengths: list[float]) -> list[float]:
    """The length-n gravity-load vector G(q) -- the gradient of the arm's
    potential energy with respect to q.

    Returns:
      A length-n list, the generalized-force contribution from gravity
      alone (positive `gravity` pulls along world -y).

    Note: at a converged, steady-state pose (qddot = qdot = 0), the manipulator
    equation collapses to tau = G(q) -- useful for sanity-checking this
    function's sign conventions independently later, once a PID controller
    exists to hold the arm at a commanded setpoint.
    """
    n = len(q)
    G = [0 for i in range(n)]
    for k in range(0, n):
      y = _com_jacobian(q, lengths, k)[1]
      for j in range(n):
        G[j] += (gravity * masses[k] * y[j])
    
    return G
  
  
def rref(M, b):
    A = [M[i][:] + [b[i]] for i in range(len(M))]
    n = len(A)

    for col in range(n):
        # Find a pivot
        pivot = col
        for row in range(col, n):
            if abs(A[row][col]) > abs(A[pivot][col]):
                pivot = row

        # Swap pivot row into position
        A[col], A[pivot] = A[pivot], A[col]

        # Make pivot = 1
        pivot_value = A[col][col]
        for j in range(n + 1):
            A[col][j] /= pivot_value

        # Make every other entry in this column = 0
        for row in range(n):
            if row == col:
                continue

            factor = A[row][col]

            for j in range(n + 1):
                A[row][j] -= factor * A[col][j]

    # Last column is qddot
    return [A[i][n] for i in range(n)]

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

    Expected to call M(...), C(...), G(...) above, assemble the right-hand
    side tau - C(q,qdot)@qdot - G(q), and solve the resulting n x n linear
    system for qddot (the exact linear-algebra method -- Gauss-Jordan with
    partial pivoting works fine, but so does anything else -- is entirely
    your choice; a caller outside this module must never depend on it).

    Args:
      q, qdot, tau: length-n lists (joint angles, joint angular velocities,
        applied joint efforts), n == 2 or 3.
      gravity: scalar g >= 0.
      masses, lengths: length-n lists, each entry > 0.

    Returns:
      qddot: length-n list, the resulting joint angular accelerations.

    Invariants the implementation must satisfy (not enforced by this
    signature, but part of the contract callers may rely on):
      - Called with tau = [0]*n and any q where G(q) != 0, the returned
        qddot must be nonzero (this is the spec's own stated correctness
        check for the "arm swings freely under gravity" milestone).
      - Purely a function of (q, qdot, tau, gravity, masses, lengths) -- no
        explicit time dependence, no internal state/memory between calls.
    """
    n = len(q)
    b = [0 for i in range(n)]
    C_mat = C(q, qdot, masses, lengths)
    C_qdot = []
    for i in range(n):
      tot = 0
      for j in range(n):
        tot += C_mat[i][j] * qdot[j]
      C_qdot.append(tot)
    G_vec = G(q, gravity, masses, lengths)
    for i in range(n):
      b[i] = tau[i] - C_qdot[i] - G_vec[i]
    
    
    M_mat = M(q, masses, lengths)
    return rref(M_mat, b)
