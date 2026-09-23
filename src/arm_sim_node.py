"""arm_sim node: the /arm_sim/integration_step checkpoint service, the
/arm_sim/set_params, /arm_sim/set_integrator, and /arm_sim/pause
parameter/mode-storage services, and the live n-link arm's physics loop
(physics_loop) and /joint_states publisher (publish_loop).

/arm_sim/reset is NOT included here: the spec ties it to also resetting the
PID controller's setpoint/integral, which doesn't exist yet -- deferred to
when pid.py is built.

Assumptions documented here since the spec doesn't give literal defaults for
these (only gravity=9.81 is spec-stated):
- masses/lengths default to 1.0 per link
- integrator defaults to "euler" with timestep=0.01
- set_integrator's two fields are validated/applied independently (matching
  the family-wide "every field optional and independently applied"
  convention stated up front in the spec), even though the spec's own
  set_integrator paragraph phrases rejection singularly ("reject it") rather
  than "independently" the way set_params does explicitly
- reset pose is q = qdot = [0.0]*links (see agent-notes/PLAN.md, "Open
  questions and assumptions")

physics_loop/publish_loop wiring (agent-notes/PLAN.md, "Proposed
architecture"): physics_loop paces itself by the *current*
state.integrator_timestep each iteration and, while not paused, steps
(state.q, state.qdot) forward one tick via integrators.METHODS[...], with
tau always the zero vector (PID is not wired in yet -- always disabled this
pass). publish_loop is entirely independent of physics_loop: it publishes
/joint_states at a fixed reference 60 Hz regardless of `paused` or the
current integrator_timestep. Both loops read every relevant _ArmSimState
field fresh on each iteration (never cached), so a mid-run
/arm_sim/set_params or /arm_sim/set_integrator call takes effect on the very
next tick.
"""
from __future__ import annotations

import asyncio
from typing import Any

import arm_dynamics
import expr
import integrators
from gateway import log
from registry import Registry

PUBLISH_PERIOD_S = 1 / 60


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
        # Live arm state: reset pose is q = qdot = [0]*links (see module
        # docstring); sim_time is the simulation clock physics_loop advances
        # and /joint_states' header.stamp reports.
        self.q = [0.0] * links
        self.qdot = [0.0] * links
        self.sim_time = 0.0

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


def _joint_names(links: int) -> list[str]:
    """1-indexed joint names ("joint1", "joint2", ...), matching the spec's
    own /joint_trajectory example verbatim. Shared by publish_loop now and
    reusable by a future /joint_trajectory subscriber without redefinition.
    """
    return [f"joint{i}" for i in range(1, links + 1)]


async def physics_loop(state: _ArmSimState) -> None:
    """Infinite loop advancing the live arm's (q, qdot) one integration step
    per iteration, paced by the *current* state.integrator_timestep.

    While state.paused, the integration step (and state.sim_time) is simply
    skipped -- the loop keeps sleeping/looping so it resumes promptly once
    unpaused, rather than exiting or blocking.

    tau is always the zero vector this pass (PID is not wired in yet).
    arm_dynamics.forward_dynamics is expected to raise NotImplementedError
    until its owner fills it in by hand -- any exception from a tick (that,
    or a future bug in a completed implementation) is caught, logged once
    (then throttled to avoid flooding stderr on every subsequent tick), and
    the loop keeps running so the rest of the runtime (services,
    publish_loop) stays fully responsive regardless.
    """
    logged_failure = False
    while True:
        dt = state.integrator_timestep
        await asyncio.sleep(dt)

        if state.paused:
            continue

        try:
            gravity, masses, lengths = state.gravity, state.masses, state.lengths
            links = state.links
            method = integrators.METHODS[state.integrator_method]

            def _accel_fn(t: float, q: list[float], qdot: list[float]) -> list[float]:
                tau = [0.0] * links
                return arm_dynamics.forward_dynamics(q, qdot, tau, gravity, masses, lengths)

            q_next, qdot_next = method(_accel_fn, state.sim_time, state.q, state.qdot, dt)
            state.q, state.qdot = q_next, qdot_next
            state.sim_time += dt
        except Exception as exc:
            if not logged_failure:
                log(f"physics_loop: tick failed, arm frozen until this is resolved ({exc!r}); "
                    "further failures on this tick will not be individually logged")
                logged_failure = True


async def publish_loop(registry: Registry, state: _ArmSimState) -> None:
    """Infinite loop publishing /joint_states at a fixed reference 60 Hz,
    entirely independent of `state.paused` and `state.integrator_timestep`
    -- a paused sim still publishes its frozen snapshot, and the publish
    rate never tracks whatever the physics timestep currently is.
    """
    names = _joint_names(state.links)
    while True:
        await asyncio.sleep(PUBLISH_PERIOD_S)

        stamp_sec = int(state.sim_time)
        stamp_nanosec = int(round((state.sim_time - stamp_sec) * 1e9))
        msg = {
            "header": {"stamp": {"sec": stamp_sec, "nanosec": stamp_nanosec}, "frame_id": ""},
            "name": names,
            "position": list(state.q),
            "velocity": list(state.qdot),
            "effort": [0.0] * state.links,
        }
        registry.publish("/joint_states", msg)


def register(registry: Registry, links: int = 2) -> _ArmSimState:
    registry.register_handler("/arm_sim/integration_step", _integration_step)

    state = _ArmSimState(links)
    registry.register_handler("/arm_sim/set_params", state.set_params)
    registry.register_handler("/arm_sim/set_integrator", state.set_integrator)
    registry.register_handler("/arm_sim/pause", state.pause)
    return state
