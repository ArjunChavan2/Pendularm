# Plan — Project 2 checkpoint: `/arm_sim/integration_step` + `/arm_sim/*` state services

**Note:** written retroactively, after implementation, at the user's request — this documents the
decisions actually made, not a plan produced before the code existed. Future work on this repo
should run the real Plan → Implement cycle from the start (see `prompts/PLAN.md`). This file
covers two rounds of work: the original checkpoint (below), and a second round (see "Round 2")
that fixed `agent-notes/AUDIT.md`'s findings and added `/arm_sim/set_params`,
`/arm_sim/set_integrator`, `/arm_sim/pause`. `agent-notes/TEST_RESULTS.md` is the current,
up-to-date verification record covering both rounds.

## Goal

Make `/arm_sim/integration_step` (spec/PROJECT2_PENDULARM.md, "Project checkpoint — Numerical
Integration Step Service") callable end-to-end over the real TCP/JSON gateway on
`127.0.0.1:9095`, wired to the user's hand-written `src/integrators.py`. This is the Sep 25
checkpoint and its own 25%-weighted grading category ("Integrators").

## Relevant requirements

- Standalone service, fully decoupled from any live arm state (never reads/writes `arm_sim`'s own
  `(q, qdot)`, no effect on `/joint_states`).
- Request: `{"function": "<expr in t>", "x0": <f64>, "xdot0": <f64, optional, default 0>, "dt":
  <f64>, "steps": <u64>, "integrator": "euler"|"midpoint"|"verlet"|"rk4"}`.
- Response on success: `{"times": [...], "positions": [...], "velocities": [...]}`, one entry per
  step plus the starting point at index 0 (`times[0]==0`, `positions[0]==x0`,
  `velocities[0]==xdot0`).
- `function` is a small math expression in `t`: `+ - * /` (left-assoc), `^` (right-assoc, higher
  precedence than `*`/`/`), unary minus (binds *tighter* than `^` per spec's stated precedence
  order), parens, numeric literals, `sin cos tan exp sqrt ln abs`.
- Reject cleanly (`result:false`, non-empty `status`), never crash/hang: unparseable `function`,
  unknown `integrator` name, `dt <= 0`, `steps == 0`.
- Domain errors at evaluation time (e.g. `sqrt` of a negative number) should become NaN/inf, "the
  same as ordinary floating-point arithmetic" — not raise.
- Runtime must stay fully responsive to later calls after a bad request.
- No `make map` target this project; `make build`/`run`/`clean` only (`test` is a project
  convenience, not graded).

## Repository observations

- Starter kit (`~/project2-student-kit`) provides only a placeholder `src/main.py` and a stub
  Makefile (`build`/`run`/`clean`) — no gateway, no services.
- Project 1 (`~/A-Star-Path-Planning`) already has a working, generic, domain-agnostic
  `registry.py`/`gateway.py` implementing the exact same rosbridge protocol
  (`spec/ROSBRIDGE_PROTOCOL.md` is identical between the two projects) — the spec explicitly says
  to reuse this transport unchanged, so no reason to reimplement it.
- `src/integrators.py` (hand-written by the user, per the established division of labor from
  Project 1's `heap.py`/`astar.py`) already existed and was fully implemented/tested
  (`tests/test_integrators.py`, 14/14 passing) before this checkpoint work began: `euler`,
  `midpoint`, `verlet`, `rk4`, each `(accel_fn, t, q, qdot, dt) -> (q_next, qdot_next)` over
  `list[float]` state (uniform 1-DOF/n-DOF representation), plus a `METHODS` name→function dict.

## Proposed architecture

- Port `registry.py`/`gateway.py` from Project 1 verbatim (confirmed no map/heap-specific logic in
  either file — both are already fully generic).
- New `src/expr.py`: a small hand-rolled recursive-descent parser turning a `function` string into
  a `Callable[[float], float]`. Own module, no dependency on the rest of the runtime, since it's a
  self-contained parsing/evaluation concern.
- New `src/arm_sim_node.py`: registers `/arm_sim/integration_step` against the `Registry`, using
  `expr.parse_function` + `integrators.METHODS`. Named for the eventual home of the rest of
  `/arm_sim/*` (not yet built) rather than a separate file, since the spec's own node diagram
  groups all `arm_sim` responsibilities under one logical node.
- Replace the placeholder `src/main.py`: wires `Registry` + `Gateway` + `arm_sim_node`, reads
  `ARM_SIM_LINKS` (defaulting to `"2"`, validated but not yet consumed by anything since the live
  arm doesn't exist yet), signal-based clean shutdown (mirroring Project 1's `main.py`).

## Interfaces and data flow

```
external client --call_service--> gateway.py --> registry.py (in-process handler)
    --> arm_sim_node._integration_step(args)
        --> expr.parse_function(args["function"]) -> f(t)
        --> wraps as accel_fn(t, q, qdot) = [f(t)]  (1-DOF, ignores q/qdot)
        --> loop: q, qdot = integrators.METHODS[name](accel_fn, t, q, qdot, dt)
        --> collects times/positions/velocities arrays
    <-- (result, values, status)
<-- service_response
```

## Implementation steps (as executed)

1. Port `registry.py`, `gateway.py` unchanged; confirm `py_compile`.
2. Write `expr.py` (tokenizer + recursive-descent parser + domain-safe named functions).
3. Write `arm_sim_node.py` (arg validation, `expr`/`integrators` wiring).
4. Write `main.py` (replace placeholder).
5. Port `tests/client_helper.py` from Project 1.
6. Write `tests/test_expr.py` (parser correctness + malformed-input + domain-error-to-NaN/inf).
7. Write `tests/test_arm_sim_integration_step.py` (wire-level, real gateway on a background
   thread, same pattern as Project 1's `test_gateway_protocol.py`).
8. Verify: `make build`, `make test`, then a genuine `make run` + live TCP calls + clean `SIGTERM`
   shutdown.

## Edge cases and failure modes

- Empty/malformed `function` strings (unbalanced parens, unknown identifiers, trailing garbage,
  adjacent tokens with no operator, unrecognized characters) — all must reject via
  `expr.ExpressionError`, not raise uncaught.
- `dt <= 0`, `steps == 0` (or negative), missing/non-numeric `x0`, unknown `integrator` name — all
  rejected before any integration runs.
- `xdot0` omitted → defaults to `0.0`.
- Domain errors at evaluation time (`sqrt(-1)`, `ln(-1)`, `ln(0)`, division by zero) must become
  NaN/inf, not raise — Python's own `math.sqrt`/`math.log` raise `ValueError` and float division
  by zero raises `ZeroDivisionError`, neither of which matches IEEE-754; `expr.py` has to guard
  these explicitly.
- A bad request must not affect the runtime's ability to answer a subsequent good request.

## Open questions and assumptions

- Whether `steps` being negative (not just `== 0`) should also reject — assumed yes (`steps <= 0`)
  as a reasonable superset of the spec's explicit `steps == 0` case; not something the spec states
  directly.
- Scientific notation (`1e5`) in numeric literals is not supported by the tokenizer — the spec's
  examples (`t`, `sin(t)`, `t^2 + 3*t - 1`, `-t + 1`) don't require it, and it wasn't flagged as
  required.
- `ARM_SIM_LINKS` is read and validated in `main.py` but not yet consumed by anything (no live arm
  exists yet) — deferred to whenever `arm_dynamics.py`/the live sim loop are built.

## Verification strategy

- Unit tests for `expr.py` covering valid expressions (precedence, right-associativity of `^`,
  unary-minus-binds-tighter-than-`^`, named functions, whitespace), malformed input (each failure
  mode above), and domain-error-becomes-NaN/inf behavior.
- Wire-level integration tests (`test_arm_sim_integration_step.py`) against a real running
  gateway: correctness against a closed-form solution, all four integrators selectable, `xdot0`
  defaulting, each rejection case, and responsiveness after a bad request.
- A genuine `make build && make run`, live TCP calls from a separate client process, and a clean
  `SIGTERM` shutdown — not just unit tests in-process.

---

## Round 2 — domain-safety fixes + `/arm_sim/set_params`, `/arm_sim/set_integrator`, `/arm_sim/pause`

**Note:** also written retroactively, after this round's implementation.

### Goal

Two triggers, addressed together:
1. `agent-notes/AUDIT.md` (independent Audit phase against Round 1) found that `expr.py`'s
   domain-safety guarantee was only partially implemented — the `^` operator and `sin`/`cos`/`tan`
   had no guards, and one case (`(-4)^0.5`) caused a genuine hang.
2. The user reported a real autograder failure: the grading harness's startup probe calls
   `/arm_sim/set_params` with an empty `{}` query and expects an echo of current parameters within
   a startup deadline; with no provider registered for that service, it got `result:false,
   status:"no provider"` and the whole run failed before any category-specific grading happened.

### Relevant requirements

- `expr.py`: same "domain errors at evaluation time become NaN/inf, never raise" requirement as
  Round 1 — Round 1 only partially satisfied it.
- `/arm_sim/set_params` — `{"gravity": <>=0>, "masses": [n values, each >0], "lengths": [n values,
  each >0]}` (`n` = link count from `ARM_SIM_LINKS`; gravity defaults 9.81). Every field optional;
  response always echoes current post-update parameters, even when the request changed nothing or
  was rejected outright; each field validated/rejected independently, without disturbing other
  valid fields in the same request.
- `/arm_sim/set_integrator` — `{"method": "euler"|"midpoint"|"verlet"|"rk4", "timestep": <positive
  seconds>}`. An unrecognized method or non-positive timestep must be rejected, not silently
  defaulted.
- `/arm_sim/pause` — `{"data": <bool>}`. Freezes/resumes the simulation clock (no live physics loop
  exists yet to actually freeze, so this is currently just state storage).
- `/arm_sim/reset` is explicitly NOT in scope for this round — the spec ties it to also resetting
  PID controller state, which doesn't exist yet.

### Proposed architecture

- Add `_safe_pow` (catches `ZeroDivisionError`/`OverflowError`/complex results from `**`) and
  `_safe_trig` (guards `sin`/`cos`/`tan` against non-finite input) to `expr.py`, wired into
  `_combine`'s `"^"` branch and `_FUNCTIONS`.
- Add an `_ArmSimState` class to `arm_sim_node.py` holding `gravity`/`masses`/`lengths`/
  `integrator_method`/`integrator_timestep`/`paused`, with one method per new service implementing
  the independent-per-field validation pattern. `register(registry, links)` now takes the link
  count (previously just `register(registry)`), threaded from `main.py`'s already-parsed
  `ARM_SIM_LINKS`.

### Open questions and assumptions

- No literal defaults are spec-given for `masses`/`lengths`/integrator method/timestep (only
  `gravity=9.81` is spec-stated) — assumed `masses`/`lengths` default to `1.0` per link,
  `integrator` defaults to `"euler"` with `timestep=0.01`.
- `/arm_sim/set_integrator`'s two fields are validated/applied independently (matching the
  family-wide "every field optional and independently applied" convention stated up front in the
  spec), even though the spec's own `set_integrator` paragraph phrases rejection singularly
  ("reject it") rather than "independently" the way `set_params` does explicitly — documented as
  an assumption, not a certainty.

### Verification strategy

- Regression tests in `tests/test_expr.py` for each of the Audit's specific reproduction cases.
- New `tests/test_arm_sim_state_services.py` covering the empty-query convention,
  independent-field validation, and both 2-link and 3-link configurations.
- A dedicated, independent Test phase (see `agent-notes/TEST_RESULTS.md`) re-verifying all of the
  above directly against the current code rather than trusting this plan or the Audit findings.
