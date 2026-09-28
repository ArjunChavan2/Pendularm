"""Per-joint PID control law for the live arm (decentralized: each joint independent).

STUB: implement this file yourself. Holds the controller's state (gains,
accumulated integral) and the control law from spec/PROJECT2_PENDULARM.md,
"Control law":

    position_error = setpoint_pos - actual_pos
    velocity_error = setpoint_vel - actual_vel
    integral       = clamp(integral + position_error * dt, -integral_limit, +integral_limit)
    effort         = kp * position_error + ki * integral + kd * velocity_error

applied to every joint independently each control update. Service-level
concerns -- enabling/disabling, validating /pid_controller/set_gains requests
(wrong length, negative values), echoing gains -- live in the service layer,
not here; this module assumes it's handed already-valid values.
"""
from __future__ import annotations


class PIDController:
    """PID state and control law for an n-joint arm.

    Attributes callers may read/write directly (all length-n lists of float):
      kp, ki, kd: current gains, one per joint.
      integral:   accumulated position error * dt, one per joint.
    integral_limit: the anti-windup clamp bound (positive float; any
      reasonable fixed value -- the spec says only that some bound exists).
    """

    def __init__(self, n: int, kp: list[float], ki: list[float], kd: list[float],
                 integral_limit: float) -> None:
        self.n = n
        self.kp = list(kp)
        self.ki = list(ki)
        self.kd = list(kd)
        self.integral_limit = integral_limit
        
        self.integral = [0 for i in range(n)]

    def reset(self) -> None:
        """Clear accumulated controller state (the integral) without touching gains.

        Called on /arm_sim/reset and when the controller is re-enabled after
        being disabled -- NOT when gains change (spec: set_gains must leave
        the integral alone).
        """
        self.integral = [0 for i in range(self.n)]

    def update(self, setpoint_pos: list[float], setpoint_vel: list[float],
               q: list[float], qdot: list[float], dt: float) -> list[float]:
        """Advance the integral by one control step and return the effort.

        Args:
          setpoint_pos, setpoint_vel: commanded joint positions/velocities.
          q, qdot: measured joint positions/velocities.
          dt: time since the previous control update (seconds, > 0).

        Returns:
          A length-n list: the joint effort (tau) to apply, per the control law.

        Side effect: updates self.integral (clamped to +/- integral_limit).
        """
        tau = [0 for i in range(self.n)]
        for i in range(self.n):
          ep = setpoint_pos[i] - q[i]
          ev = setpoint_vel[i] - qdot[i]
          new_val = self.integral[i] + ep * dt
          if new_val < -self.integral_limit:
            new_val = -self.integral_limit
          elif new_val > self.integral_limit:
            new_val = self.integral_limit
          self.integral[i] = new_val
          tau[i] = self.kp[i] * ep + self.ki[i] * self.integral[i] + self.kd[i] * ev
          
        return tau
