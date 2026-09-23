# Implementation — Project 2 checkpoint: `/arm_sim/integration_step` + `/arm_sim/*` state services

**Note:** written retroactively, after implementation, at the user's request — see
`agent-notes/PLAN.md` for the same caveat. Covers two rounds; see "Round 2" below for the
domain-safety fixes and the three new state-storage services. `agent-notes/TEST_RESULTS.md` is the
current, up-to-date verification record covering both rounds — this file and `AUDIT.md` are
historical/point-in-time.

## What changed

- **New** `src/registry.py`, `src/gateway.py` — ported byte-for-byte from
  `~/A-Star-Path-Planning/src/{registry,gateway}.py`. No changes needed; both were already
  domain-agnostic.
- **New** `src/expr.py` — tokenizer (regex-based) + hand-written recursive-descent parser,
  producing `Callable[[float], float]` closures directly during parsing (no separate AST step).
  Precedence implemented exactly per spec's stated order (lowest→highest: `+/-`, `*//`, `^`,
  unary minus, atoms) — notably unary minus binds *tighter* than `^`, so `-2^2` parses as
  `(-2)^2 == 4`, not `-(2^2)`, per the spec's explicit ordering (this differs from Python's own
  `**` precedence, where unary minus binds *looser* than `**` — a deliberate deviation from
  "normal" language precedence to match the spec, verified with an explicit test).
- **New** `src/arm_sim_node.py` — `register(registry)` registers `/arm_sim/integration_step`.
  Validates all five fields independently before running anything; wraps `expr`'s `f(t)` as a
  1-DOF `accel_fn(t, q, qdot) = [f(t)]` (ignoring `q`/`qdot`, matching the checkpoint's stated
  "purely time-varying forcing" semantics); loops `steps` times calling
  `integrators.METHODS[name]`.
- **Replaced** `src/main.py` (was the starter placeholder) — wires `Registry`, `Gateway`,
  `arm_sim_node`; reads/validates `ARM_SIM_LINKS`; SIGINT/SIGTERM → clean `gateway.stop()`.
- **New** `tests/client_helper.py` — ported unchanged from Project 1.
- **New** `tests/test_expr.py` (28 tests), `tests/test_arm_sim_integration_step.py` (8 tests,
  wire-level against a real gateway on a background thread/event loop, same pattern as Project 1's
  `test_gateway_protocol.py`).

## Important implementation decisions

- **Domain-safe math functions in `expr.py`.** Plain `math.sqrt`/`math.log` raise `ValueError` on
  out-of-domain input, and Python's `/` raises `ZeroDivisionError` for float division by zero —
  neither matches the spec's "let it become NaN/inf, the same as ordinary floating-point
  arithmetic." Implemented `_safe_sqrt`, `_safe_ln`, `_safe_exp` (catches `OverflowError`), and
  `_safe_div` (mirrors IEEE-754 sign rules for `x/0`) so evaluation never raises for a
  domain-error input, only for a genuinely unparseable expression.
- **Closures instead of a separate AST + eval step.** Each parse function returns a `t -> float`
  callable directly (using the "late-binding via default-argument-style immediately-invoked
  lambda" pattern to avoid Python closure-capture-by-reference bugs in the loop-building code).
  Simpler than building and later walking a tree, at the cost of being slightly harder to
  introspect/debug than an explicit AST — acceptable given this module's small scope.
- **`arm_sim_node.py` as the eventual home for all of `/arm_sim/*`**, not just this one service —
  named for where the live-arm services will live once `arm_dynamics.py`/`pid.py` exist, rather
  than splitting the checkpoint into its own separately-named file.
- **Validation order in `_integration_step`**: `function` parsed first (so a malformed expression
  fails fast with a specific message), then `x0`/`xdot0`/`dt`/`steps`/`integrator` each checked
  independently — each returns its own specific `status` string rather than a generic failure.

## Deviations from the plan

None — the plan (written after the fact, see its own caveat) matches what was actually built.

## Known limitations or unresolved questions

- Scientific notation (`1e5`) is not supported in numeric literals — not required by any spec
  example, but worth confirming isn't silently expected elsewhere in grading.
- `ARM_SIM_LINKS` is read/validated but unused until the live arm exists.
- The live n-link arm (`arm_dynamics.py`, `pid.py`, `/arm_sim/set_*`, `/joint_trajectory`,
  `/joint_states`) and IK (`kinematics.py`, `/ik/*`, `/ik_action/*`, `/ik_trial/*`) are not built —
  out of scope for this checkpoint.

## Checks performed

- `make build` (compiles all of `src/*.py`) — clean.
- `make test` — 50/50 passing (14 `test_integrators.py` + 28 `test_expr.py` + 8
  `test_arm_sim_integration_step.py`).
- Live smoke test: real `make run` (backgrounded), raw TCP calls from a separate client process
  (`sin(t)` via `rk4`, a polynomial via `midpoint`, a deliberately malformed expression correctly
  rejected with a specific `status`), then `SIGTERM` → confirmed clean shutdown log line, no
  traceback, port released (`lsof` check).
- Found and fixed one real bug during testing: `expr.py`'s division operator initially let
  Python's `ZeroDivisionError` propagate for `x/0` instead of producing `inf`/`nan` per spec; a
  dedicated test (`test_division_by_zero_does_not_raise`) caught it before it shipped.
- Found and cleared one environment issue (not a code bug): a stray `python3 src/main.py` process
  from an earlier manual test session was still holding port 9095, causing the first
  `test_arm_sim_integration_step.py` run to fail with `OSError: address already in use` on every
  test in that module (all failing with a misleading "no provider" status, since the test's own
  server never actually started). Killed the stray process; re-ran clean.

---

## Round 2 — domain-safety fixes + `/arm_sim/set_params`, `/arm_sim/set_integrator`, `/arm_sim/pause`

### What changed

- **`src/expr.py`**: added `_safe_pow(base, exponent)` (catches `ZeroDivisionError` for
  `0**negative` → `inf`; catches `OverflowError` for a too-large result → signed `inf`; checks
  `isinstance(result, complex)` for a negative base / non-integer exponent → `nan`) and
  `_safe_trig(fn)` (a wrapper checking `math.isfinite` before calling `sin`/`cos`/`tan`, returning
  `nan` for non-finite input instead of letting `ValueError` propagate). Wired `_safe_pow` into
  `_combine`'s `"^"` branch and `_safe_trig` into `_FUNCTIONS`' `sin`/`cos`/`tan` entries.
- **`src/arm_sim_node.py`**: added `_ArmSimState` (holds `links`, `gravity`, `masses`, `lengths`,
  `integrator_method`, `integrator_timestep`, `paused`) with three methods —
  `set_params`/`set_integrator`/`pause` — each validating/applying its request fields
  independently and always echoing current (post-update) values. `register(registry, links=2)`
  now takes a `links` parameter and registers these three alongside the existing
  `/arm_sim/integration_step`.
- **`src/main.py`**: now converts the validated `ARM_SIM_LINKS` string to an `int` and passes it
  to `arm_sim_node.register(registry, links)`.
- **New** `tests/test_arm_sim_state_services.py` (18 tests): the empty-`{}`-query convention,
  independent-field validation (an invalid field doesn't block other valid fields in the same
  request), rejection cases, and both 2-link and 3-link configurations.
- **New regression tests** in `tests/test_expr.py` for each of `AUDIT.md`'s specific reproduction
  cases (`(-4)^0.5`, `0^-1`, `t^1000` at `t=500`, `sin(1/t)` at `t=0`).
- A subsequent, independent Test phase (see `agent-notes/TEST_RESULTS.md`) added
  `tests/test_live_process_e2e.py` (7 tests) re-verifying all of the above over a genuine `make
  run` OS subprocess with a from-scratch TCP client, independent of the existing in-process test
  harness. Combined suite: 75 tests, all passing.

### Important implementation decisions

- **`_safe_pow`'s overflow sign handling**: for a negative base overflowing to infinite magnitude,
  the sign of the result depends on whether the exponent is an odd integer (mirroring how a finite
  negative-base odd-integer power would be negative). This is a simplification — it doesn't
  attempt to handle every theoretically possible edge case (e.g. overflow combined with a
  complex-producing fractional exponent), which is an accepted, documented gap given the spec's
  own "no need to special-case" latitude on domain errors.
- **Reused `_is_number` and independent-per-field validation** from `_integration_step` for the
  new state services, keeping the validation style consistent across all of `arm_sim_node.py`
  rather than introducing a second pattern.

### Deviations from the plan

None — matches `agent-notes/PLAN.md`'s "Round 2" section.

### Known limitations or unresolved questions

- `/arm_sim/reset` is still not built (needs PID controller state per spec).
- The live n-link arm (`arm_dynamics.py`, `pid.py`, `/joint_trajectory`, `/joint_states`) and IK
  are still not built — `ARM_SIM_LINKS`/the new state services' stored values aren't consumed by
  anything yet.
- Whether the grader's startup probe checks *only* `/arm_sim/set_params`, or other `/arm_sim/*`
  endpoints too (e.g. `/arm_sim/reset`), wasn't confirmed against the actual grader beyond the one
  error message the user shared — a re-submission is the only way to know for certain.

### Checks performed

- `make build`, `make test` — 75/75 passing (see `agent-notes/TEST_RESULTS.md` for the full
  independent-Test-phase breakdown).
- Live smoke tests over a real `make run` subprocess: each of Audit's exact reproduction strings
  confirmed to now return `result:true` with NaN/inf values instead of hanging or being rejected;
  `/arm_sim/set_params` empty-query confirmed to echo `{"gravity": 9.81, "masses": [1.0, 1.0],
  "lengths": [1.0, 1.0]}` (2-link) instead of `"no provider"`.
- `submission.tar.gz` rebuilt and re-verified (fresh extraction, `make build`/`make test`/`make
  run`) after each change in this round before being sent to the user.
