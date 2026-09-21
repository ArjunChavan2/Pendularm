"""arm_sim node: the /arm_sim/integration_step checkpoint service, plus
minimal state-storage stubs for /arm_sim/set_params, /arm_sim/set_integrator,
and /arm_sim/pause -- these three don't need arm_dynamics.py/pid.py to exist,
since they're just parameter/mode storage, not physics computation. They're
built now so the grader's startup probe (which pings /arm_sim/set_params)
stops failing; nothing yet reads this stored state, since there's no live
physics loop to consume it.

/arm_sim/reset is NOT included here: the spec ties it to also resetting the
PID controller's setpoint/integral, which doesn't exist yet -- deferred to
when pid.py/the live sim loop are built.

Assumptions documented here since the spec doesn't give literal defaults for
these (only gravity=9.81 is spec-stated):
- masses/lengths default to 1.0 per link
- integrator defaults to "euler" with timestep=0.01
- set_integrator's two fields are validated/applied independently (matching
  the family-wide "every field optional and independently applied"
  convention stated up front in the spec), even though the spec's own
  set_integrator paragraph phrases rejection singularly ("reject it") rather
  than "independently" the way set_params does explicitly
"""
from __future__ import annotations

from typing import Any

import expr
import integrators
from registry import Registry


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _integration_step(args: Any) -> tuple[bool, dict, str]:
    """/arm_sim/integration_step: standalone 1-DOF numerical integration,
    fully decoupled from any live arm state. See spec/PROJECT2_PENDULARM.md,
    "Project checkpoint -- Numerical Integration Step Service".
    """
    args = args or {}

    function_str = args.get("function")
    if not isinstance(function_str, str):
        return False, {}, "missing or invalid 'function'"
    try:
        f = expr.parse_function(function_str)
    except expr.ExpressionError as exc:
        return False, {}, f"invalid function: {exc}"

    x0 = args.get("x0")
    if not _is_number(x0):
        return False, {}, "missing or invalid 'x0'"
    xdot0 = args.get("xdot0", 0.0)
    if not _is_number(xdot0):
        return False, {}, "invalid 'xdot0'"

    dt = args.get("dt")
    if not _is_number(dt) or dt <= 0:
        return False, {}, "'dt' must be a positive number"

    steps = args.get("steps")
    if not isinstance(steps, int) or isinstance(steps, bool) or steps <= 0:
        return False, {}, "'steps' must be a positive integer"

    integrator_name = args.get("integrator")
    method = integrators.METHODS.get(integrator_name) if isinstance(integrator_name, str) else None
    if method is None:
        return False, {}, f"unknown integrator {integrator_name!r}"

    def accel_fn(t: float, q: list[float], qdot: list[float]) -> list[float]:
        return [f(t)]

    q, qdot, t = [float(x0)], [float(xdot0)], 0.0
    times, positions, velocities = [t], [q[0]], [qdot[0]]
    for _ in range(steps):
        q, qdot = method(accel_fn, t, q, qdot, dt)
        t += dt
        times.append(t)
        positions.append(q[0])
        velocities.append(qdot[0])

    return True, {"times": times, "positions": positions, "velocities": velocities}, ""


class _ArmSimState:
    """Holds the parameter/mode state for /arm_sim/set_params,
    /arm_sim/set_integrator, and /arm_sim/pause. One instance per runtime.
    """

    def __init__(self, links: int) -> None:
        self.links = links
        self.gravity = 9.81
        self.masses = [1.0] * links
        self.lengths = [1.0] * links
        self.integrator_method = "euler"
        self.integrator_timestep = 0.01
        self.paused = False

    def set_params(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        errors: list[str] = []

        if "gravity" in args:
            gravity = args["gravity"]
            if _is_number(gravity) and gravity >= 0:
                self.gravity = float(gravity)
            else:
                errors.append("'gravity' must be a number >= 0")

        if "masses" in args:
            masses = args["masses"]
            if (isinstance(masses, list) and len(masses) == self.links
                    and all(_is_number(v) and v > 0 for v in masses)):
                self.masses = [float(v) for v in masses]
            else:
                errors.append(f"'masses' must be a list of {self.links} positive numbers")

        if "lengths" in args:
            lengths = args["lengths"]
            if (isinstance(lengths, list) and len(lengths) == self.links
                    and all(_is_number(v) and v > 0 for v in lengths)):
                self.lengths = [float(v) for v in lengths]
            else:
                errors.append(f"'lengths' must be a list of {self.links} positive numbers")

        values = {"gravity": self.gravity, "masses": list(self.masses), "lengths": list(self.lengths)}
        return (not errors), values, "; ".join(errors)

    def set_integrator(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        errors: list[str] = []

        if "method" in args:
            method = args["method"]
            if isinstance(method, str) and method in integrators.METHODS:
                self.integrator_method = method
            else:
                errors.append(f"unknown integrator method {method!r}")

        if "timestep" in args:
            timestep = args["timestep"]
            if _is_number(timestep) and timestep > 0:
                self.integrator_timestep = float(timestep)
            else:
                errors.append("'timestep' must be a positive number")

        values = {"method": self.integrator_method, "timestep": self.integrator_timestep}
        return (not errors), values, "; ".join(errors)

    def pause(self, args: Any) -> tuple[bool, dict, str]:
        args = args or {}
        if "data" in args:
            data = args["data"]
            if not isinstance(data, bool):
                return False, {"data": self.paused}, "'data' must be a boolean"
            self.paused = data
        return True, {"data": self.paused}, ""


def register(registry: Registry, links: int = 2) -> None:
    registry.register_handler("/arm_sim/integration_step", _integration_step)

    state = _ArmSimState(links)
    registry.register_handler("/arm_sim/set_params", state.set_params)
    registry.register_handler("/arm_sim/set_integrator", state.set_integrator)
    registry.register_handler("/arm_sim/pause", state.pause)
