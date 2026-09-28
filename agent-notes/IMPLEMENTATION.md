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

---

## Round 3 — `arm_dynamics.py` stub + live physics/publish loops (`agent-notes/PLAN.md`, this pass)

Executes `agent-notes/PLAN.md`'s "Implementation steps" (all 9) in full: creates
`src/arm_dynamics.py` as a documented stub, wires a live `physics_loop`/`publish_loop` pair into
`src/arm_sim_node.py`, and starts/stops them as background asyncio tasks from `src/main.py`. Does
**not** implement `arm_dynamics.py`'s Lagrangian derivation, `M`/`C`/`G`, or the linear-algebra
solve — per the standing rule, that file is hand-implemented by the project owner; this pass only
fixes its public contract (signature + full docstring) and raises `NotImplementedError` in the
body, mirroring `src/integrators.py`'s original "STUB: implement this file yourself" framing.

### What changed

- **New** `src/arm_dynamics.py`: stub-only. Module docstring states ownership/scope; the single
  public function `forward_dynamics(q, qdot, tau, gravity, masses, lengths) -> qddot` carries the
  full contract/invariants from the plan's "Interfaces and data flow" section (verbatim) and
  `raise NotImplementedError(...)` in the body. No `M`/`C`/`G` helpers, no solve step — nothing
  beyond the signature and docstring was written.
- **`src/arm_sim_node.py`**:
  - `import arm_dynamics` alongside the existing `import integrators`; also now imports `asyncio`
    and `gateway.log` (for `physics_loop`'s throttled failure logging).
  - `_ArmSimState.__init__` gains `self.q = [0.0]*links`, `self.qdot = [0.0]*links`,
    `self.sim_time = 0.0` (reset pose per the plan's "Open questions" section).
  - New `_joint_names(links) -> list[str]`: `["joint1", "joint2"]` / `[..., "joint3"]`,
    1-indexed, matching the spec's `/joint_trajectory` example.
  - New `async def physics_loop(state)`: infinite loop, paces itself by re-reading
    `state.integrator_timestep` before each `asyncio.sleep(...)` call; skips the integration step
    (but keeps looping) while `state.paused`; otherwise builds a zero `tau`, closes over the
    *current* `state.gravity`/`masses`/`lengths`/`links` in a per-tick `_accel_fn` matching
    `integrators.AccelFn`'s `(t, q, qdot) -> qddot` shape, calls the *current*
    `integrators.METHODS[state.integrator_method]`, writes the result back to `state.q`/`qdot`,
    and advances `state.sim_time` by the `dt` actually used. The dynamics-call + integration step
    is wrapped in `try/except Exception`, logging once via `gateway.log` on the first failure (and
    silently retrying every subsequent tick without further log spam) — this is what keeps the
    still-unimplemented `arm_dynamics.forward_dynamics`'s `NotImplementedError` from crashing the
    task or flooding stderr.
  - New `async def publish_loop(registry, state)`: infinite loop, `await asyncio.sleep(1/60)`
    (module constant `PUBLISH_PERIOD_S`), builds `/joint_states` (`header.stamp` from
    `state.sim_time`, `name` from `_joint_names`, `position`/`velocity` as fresh copies of
    `state.q`/`qdot`, `effort` all-zero) and calls `registry.publish(...)` directly — entirely
    independent of `state.paused` and `state.integrator_timestep`.
  - `register(registry, links=2)` now constructs `state = _ArmSimState(links)` before registering
    the four existing handlers (unchanged bodies, same `state` instance) and `return`s it, instead
    of returning `None`.
- **`src/main.py`**: `state = arm_sim_node.register(registry, links)` (captures the return value);
  after `await gateway.start()`, `physics_task`/`publish_task` are created via
  `asyncio.create_task(...)`; after `await stop_event.wait()` and `await gateway.stop()`, both
  tasks are cancelled and awaited via `asyncio.gather(..., return_exceptions=True)` so shutdown
  stays clean.
- **New** `tests/test_arm_sim_physics_loop.py` (15 tests): unit-level wiring tests for
  `physics_loop`/`publish_loop`, entirely independent of `arm_dynamics.forward_dynamics`'s real
  (unimplemented) body — monkeypatches it with constant/recording stand-ins. Covers: calling
  `integrators.METHODS[...]` with the right `(accel_fn, t, q, qdot, dt)` arguments; matching
  hand-computed euler output; fresh (not cached) reads of `gravity`/`integrator_method`/
  `integrator_timestep` each tick, including via the real `set_params`/`set_integrator` handler
  methods mid-run; skip-while-paused and resume-from-where-it-left-off semantics; that the *real*,
  unmodified `arm_dynamics.forward_dynamics` (still raising `NotImplementedError`) is caught and
  the loop keeps ticking without crashing or applying a partial step; `/joint_states` message
  shape/content/naming (2- and 3-link) and its fixed-rate, `paused`-independent,
  `integrator_timestep`-independent publish cadence; `register()`'s return value.
- **Extended** `tests/test_live_process_e2e.py`: added `subscribe`/`recv_publish` methods to
  `_RawClient`, and one new test,
  `test_joint_states_publishes_and_runtime_stays_responsive_with_unimplemented_arm_dynamics` —
  subscribes to `/joint_states` on the real 3-link `make run` subprocess, confirms messages arrive
  with the correct shape and (since the real, unimplemented `arm_dynamics.py` raises every tick)
  values frozen at the reset pose `[0.0, 0.0, 0.0]`, then confirms `/arm_sim/integration_step` and
  `/arm_sim/set_params` still respond normally afterward. The existing
  `test_zz_clean_sigterm_shutdown_of_underlying_python_process` (unmodified) now doubles as the
  clean-shutdown regression check for the two new background tasks — it already passed against
  this round's changes without modification, confirming `main.py`'s new cancel-and-await logic
  doesn't reintroduce a shutdown hang or traceback.

### Important implementation decisions

- **Deterministic async tests via a counting `asyncio.sleep` stand-in.** Rather than waiting on
  real wall-clock time (flaky and slow for an infinite `while True` loop), each `physics_loop`/
  `publish_loop` unit test monkeypatches `asyncio.sleep` with a stand-in that records each
  requested `dt`, lets a chosen exact number of iterations complete, and then ends the loop by
  raising `asyncio.CancelledError` from inside the awaited sleep — exactly how real task
  cancellation would terminate it, so the loop under test exits through its normal code path, not
  a special test-only escape hatch. An `on_tick(count)` callback fires once per completed
  iteration (after that iteration's `dt` is recorded, before the loop body resumes), letting tests
  mutate `state` mid-run to prove fresh (non-cached) reads — including via a subtlety worth noting
  for future edits to this test file: since `dt` is captured *before* `asyncio.sleep` is called
  each iteration but `gravity`/`integrator_method`/`paused` are read *after* it returns, an
  `on_tick(k)` mutation is visible to iteration `k`'s own body for the latter fields, but only
  takes effect starting at iteration `k+1` for pacing (`dt`) — this matches the plan's own stated
  edge case ("the iteration currently mid-sleep still completes with the previously-read
  interval") and is not a test bug; the tests' `on_tick` trigger points were chosen accordingly.
- **Exception throttling: log once, then retry silently.** Per the plan's explicitly-open choice
  ("log once... or log at a throttled rate — either acceptable"), `physics_loop` logs the first
  `forward_dynamics`/integration failure via `gateway.log` and sets a flag so subsequent identical
  failures (guaranteed every tick while `arm_dynamics.py` is unimplemented) don't spam stderr,
  while still retrying every tick afterward (so a future fix to `arm_dynamics.py` starts working
  immediately, with no restart needed).
- **`register()`'s new return value is additive, not a signature break for existing callers**:
  every existing call site (`main.py`, all four existing test modules) either already captured no
  return value (fine, since `None` becoming `_ArmSimState` doesn't break an ignored return) or is
  this round's own new code.

### Deviations from the plan

None. All 9 implementation steps were followed as specified; the `arm_dynamics.py` contract,
`_accel_fn` closure shape, message shapes, and edge-case handling all match the plan's "Interfaces
and data flow" / "Edge cases and failure modes" sections without modification.

### Known limitations or unresolved questions

- `src/arm_dynamics.py` is still an intentional stub (`NotImplementedError`) — the live arm cannot
  actually move under gravity yet; this is expected and explicitly out of scope for an agent to
  fix (see the standing rule). Until the project owner hand-implements it, `/joint_states` will
  keep publishing the frozen reset pose, exactly as this round's own tests lock in.
- `/pid_controller/*`, `/arm_sim/reset`, `/joint_trajectory`, `kinematics.py`, and all `/ik/*`
  nodes remain unbuilt — unchanged from before this pass, and explicitly out of scope per
  `agent-notes/PLAN.md`'s "Scope of this pass."
- `tau` is hard-coded to the zero vector in `physics_loop` (PID not wired in yet, per plan) — the
  single-line substitution point for a future `pid.py` is the `_accel_fn` closure inside
  `physics_loop`.
- The plan's "Physics loop pacing" open question (coupling physics wall-clock rate 1:1 to
  `integrator_timestep` rather than a fixed-quantum accumulator) was implemented as specified;
  revisiting it, if a future pass finds it behaves poorly at extreme `dt`, is flagged there as a
  plausible future extension, not something this pass changed.

### Checks performed

- `make build` — clean compile of all `src/*.py`, including the new `arm_dynamics.py` stub.
- `make test` — 91/91 passing (75 pre-existing + 15 new `tests/test_arm_sim_physics_loop.py` + 1
  new `tests/test_live_process_e2e.py` test), including the real-subprocess E2E module
  (`PENDULARM_SKIP_E2E` unset).
- Live smoke test against a real `make build && make run` (`ARM_SIM_LINKS=2`): a raw Python TCP
  client subscribed to `/joint_states` and received continuous messages (~60 Hz) shaped
  `{"header": {...}, "name": ["joint1","joint2"], "position": [0.0,0.0], "velocity": [0.0,0.0],
  "effort": [0.0,0.0]}`; confirmed `/arm_sim/integration_step` and `/arm_sim/set_params` responded
  normally throughout; confirmed the process's stderr log contained exactly one graceful
  `physics_loop: tick failed, arm frozen until this is resolved (NotImplementedError(...))` line
  (not a traceback, not repeated every tick); sent `SIGTERM` and confirmed no traceback, prompt
  exit, and port 9095 released (`lsof -iTCP:9095 -sTCP:LISTEN` empty afterward).
- `submission.tar.gz` rebuilt (`tar czf ... Makefile README.md .gitignore src tests`, entries
  listed by name, no bare `.` — verified via `tar tzf | sort`) and re-verified from a fresh
  extraction under `/tmp`: `make build` and `make test` both passed (91/91) against the extracted
  copy, independent of the working tree.
- No stray `python3 src/main.py` processes or port-9095 listeners left behind after any manual
  smoke test (checked via `lsof -iTCP:9095 -sTCP:LISTEN` and `ps aux | grep src/main.py` before
  finishing).

## Pass: `/joint_trajectory` setpoint subscriber + `/arm_sim/reset` (2026-09-28)

### What changed

- `src/arm_sim_node.py`:
  - `_ArmSimState` gained `setpoint_pos` / `setpoint_vel` (length-`links` float lists, initialized
    to the reset pose), shaped to pass straight into `pid.PIDController.update(setpoint_pos,
    setpoint_vel, q, qdot, dt)`.
  - `set_trajectory(msg)`: uses only the *last* entry of `points`; `positions` -> `setpoint_pos`,
    `velocities` -> `setpoint_vel`, each independently defaulting to all-zero if omitted, wrong
    length, or containing non-numeric/non-finite entries. Applied the instant it arrives.
  - `_TrajectorySubscriber`: in-process `Connection` stand-in (`send(message)`) subscribed via
    `registry.subscribe(..., "/joint_trajectory")` in `register()`; forwards the envelope's `msg`.
    External clients must `advertise` before `publish`, per the existing gateway gating.
  - `reset(args)` registered as `/arm_sim/reset`: `q = qdot = [0]*links`, `sim_time = 0.0`, calls
    `reset_pid_controller()`, returns `{"position": [...], "velocity": [...]}`. Leaves
    `paused`, params, and integrator settings untouched.
  - **PID-reset hook**: `_ArmSimState.reset_pid_controller()`. Currently resets the stored
    setpoint to the reset pose (spec: controller setpoint "reset to match"); the integral reset is
    a no-op. When `pid.py` is integrated, call the controller's `PIDController.reset()` there
    (clears integral, keeps gains). Not imported/instantiated yet, since its methods still raise.
- `tests/test_arm_sim_reset_trajectory.py` (new, 21 tests): unit tests on `_ArmSimState` plus
  wire tests through an in-process gateway (same `_ServerThread` harness, `gateway.stop()` before
  the loop stops).
- `tests/test_live_process_e2e.py`: +1 test (`test_reset_and_joint_trajectory_on_real_runtime`) on
  the real 3-link `make run` subprocess. It pauses, publishes a trajectory, resets, checks the
  echo, checks that `/joint_states` shows a zero pose and zero stamp, checks that a malformed
  reset is rejected, then unpauses.
- Not touched: `arm_dynamics.py`, `integrators.py`, `pid.py`, `kinematics.py`, `main.py` (no
  wiring change needed there, since `register()` does it). `submission.tar.gz` was not rebuilt.

### Spec ambiguities and how they were resolved

- **Malformed `/joint_trajectory`**: topics have no response channel, so `result:false` can't
  apply. A message with no usable last point (not an object; `points` missing, empty, or not a
  list; last entry not an object) is silently ignored and the previous setpoint is kept.
  Bad arrays inside a usable point fall back to all-zero, per the spec's "omitted or wrong
  length" rule. Non-numeric and NaN/inf entries are grouped with that rule.
- **`joint_names`** is not used to reorder arrays. They are taken in joint order, since the spec
  only defines the length rule.
- **Malformed `/arm_sim/reset`**: the spec's request is `{}` with no fields. `None` or any object
  is accepted, and unknown keys are ignored. A non-object `args` (list, string, number, bool)
  returns `result:false` with a nonempty status, echoes the *current* pose, and leaves the
  state unchanged.
- **Reset while paused** stays paused, because the spec doesn't connect reset and pause.
- **Setpoint after reset** is the reset pose with zero velocity, so an enabled controller holds
  home.

### Checks performed

- `lsof -iTCP:9095 -sTCP:LISTEN` empty before/after; no stray `src/main.py`.
- `make build && make test`: 113/113 passing (91 existing + 22 new), including the E2E subprocess
  module.

### Remaining work (next integration pass)

- `physics_loop` still applies `tau = [0.0] * links`. Once the owner's `pid.PIDController` passes
  its checks, wire it at the `_accel_fn` substitution point: while enabled, use `tau =
  controller.update(state.setpoint_pos, state.setpoint_vel, q, qdot, dt)`, and keep zero effort
  while disabled.
- `/pid_controller/enable` (`{"data": bool}`; disabled by default; re-enabling calls
  `PIDController.reset()`) and `/pid_controller/set_gains` (optional `kp`/`ki`/`kd`, per-field
  rejection of wrong length or negative values, echo current gains, never reset the integral)
  are still unbuilt. They are service-layer infrastructure, not the owner's graded file.
- In `reset_pid_controller()`, replace the commented no-op with `controller.reset()`.
- `/joint_states`' `effort` field should report the applied `tau` once PID is wired in.

## Session status (2026-09-28)

- `src/arm_dynamics.py` has been **hand-implemented by the project owner and verified** (committed
  `1edf945`). It contains `_com_positions`, `_com_jacobian`, `_com_jacobian_dq`, `M`, `_dM_dq`,
  `C` (Christoffel symbols), `G`, and `forward_dynamics`, plus a Gaussian-elimination solver. The
  checks were finite differences against positions, M and potential energy, the textbook 2-link
  M and Coriolis terms, antisymmetry of Mdot - 2C, and kinetic-energy conservation (drift 2e-11
  over 3 s on a free 3-link arm). With tau = 0 the live arm now swings under gravity. Two tests
  that assumed the stub were updated in the same commit.
- `src/pid.py` was added as a **stub contract only** (`PIDController.__init__`, `reset()`,
  `update()`, all raising `NotImplementedError`). The owner implements the bodies.
- Still unbuilt: `src/kinematics.py` and the `/ik/*`, `/ik_action/*`, `/ik_trial/*` services
  (the IK category, 15%).
- Final deadline: **11:59 PM, Sep 28**.

## Pass: PID wiring + `/pid_controller/*` services (2026-09-28)

### What changed

- `src/arm_sim_node.py` (`pid.py` itself untouched):
  - `_ArmSimState` now owns `controller = pid.PIDController(links, kp, ki, kd, INTEGRAL_LIMIT)`
    (fresh lists per state), `pid_enabled = False` (spec default), and `tau` (the effort last
    applied, zero-initialized).
  - `physics_loop`: while enabled, `tau = controller.update(setpoint_pos, setpoint_vel, q, qdot,
    dt)` is called **once per tick** from the measured state at the start of the tick and that
    tick's `dt`, then held constant across the integrator's substages (zero-order hold). While
    disabled, `tau = [0]*links` and `update()` is not called, so the integral does not advance.
    `state.tau` is set only after a successful step. Pause skips the whole tick, so the integral
    freezes too (spec).
  - `publish_loop`: `/joint_states.effort` = `list(state.tau)` (it used to be hard-coded zeros).
  - `pid_enable(args)` -> `/pid_controller/enable`: `{"data": bool}`; `{}`/`None` queries; echoes
    `{"data": enabled}` like `/arm_sim/pause`. A non-bool `data` or a non-object request gives
    `result:false` with a nonempty status, the current echo, and no state change. Going from
    disabled to enabled calls `controller.reset()`. Enabling while already enabled is a no-op.
    Disabling zeroes `state.tau` right away.
  - `set_gains(args)` -> `/pid_controller/set_gains`: `kp`/`ki`/`kd` are each optional and
    validated independently. Each must be a list of exactly `links` finite, non-bool numbers,
    each `>= 0`. A valid field is assigned as a fresh float list (`setattr(controller, name,
    [...])`). The response always echoes the current gains, `{}` queries, and the integral is
    never touched.
  - `reset_pid_controller()` now also calls `controller.reset()` and zeroes `state.tau`. Gains and
    the enabled flag are kept.
  - `register()` registers both `/pid_controller/*` handlers.
- `src/main.py`: docstring only.

### Decision: default gains differ from the ones suggested in the brief

The brief said kp=100, ki=20, kd=40, but with the default timestep of 0.01 **these diverge to NaN
within about 0.15 s** in 23 of 24 offline cases (2- and 3-link, all 4 integrators). The distal
link's effective inertia is small (min eig of M(0) is about 0.066 for 2 links), so
`kd*dt/m_eff` is far outside explicit-integration stability. A gain sweep chose **kp=200, ki=60,
kd=10, integral_limit=10**. At dt=0.01 all 24 cases converge; 15 s worst error is 0.003 rad.
Time to stay within 0.05 rad is about 2 s for 2 links and up to about 7.3 s for 3 links with
euler. At dt=0.005 the gains are also stable. At dt=0.02, 3-link euler/midpoint-style cases
diverge, which the spec describes as expected. The integral limit leaves ample headroom, since
the largest 3-link G is about 44 N·m, versus ki*limit=600.

### Spec decisions

- Enable/set_gains follow the family-wide "every field optional, `{}` queries" convention.
- Enable's response shape is `{"data": bool}`, mirroring `/arm_sim/pause`, because the spec
  doesn't give one.
- NaN/inf gains are rejected along with negatives and wrong lengths.

### Tests

- New `tests/test_pid_services.py`, 25 tests, about 1 s:
  - enable/disable semantics: default disabled, echo, non-bool rejection, re-enable clears the
    integral, enabling while enabled keeps it, and disable zeroes the effort.
  - set_gains: defaults echo, apply and echo, per-field rejection for each of kp/ki/kd across 10
    bad values while other fields are still applied, integral preserved, and the echo is a copy.
  - reset clears the integral and keeps gains and enabled.
  - physics_loop: disabled gives zero tau and never calls update; enabled under gravity gives
    nonzero effort; update runs once per tick with start-of-tick state and dt, and tau is held
    across rk4's 4 stages; disable freezes the integral; pause freezes it; publish_loop reports
    `state.tau`.
  - convergence with real dynamics: 2-link euler/rk4 and 3-link euler/rk4. Position must be
    within 0.05 rad, `|qdot| < 0.05`, and steady-state tau must be within 0.5 of `arm_dynamics.G(q)`.
  - wire round-trips of both services through an in-process gateway (3-link).
- Changed: only `tests/test_arm_sim_reset_trajectory.py`'s module docstring, which said "PID
  doesn't exist yet". No existing assertions were obsolete, because the default-disabled PID
  keeps effort zero.

### Checks performed

- `lsof -iTCP:9095 -sTCP:LISTEN` empty before and after; no stray `src/main.py`.
- `make build && make test`: 138/138 (113 + 25), about 3 s.
- Live smoke test (`ARM_SIM_LINKS=2 python3 src/main.py`) with a raw TCP client:
  - `enable {}` returned `{"data": false}`; `set_gains {}` echoed the defaults; enabling
    returned true.
  - After publishing `/joint_trajectory` [0.3, -0.5] and waiting about 5 s, `/joint_states`
    showed position [0.287, -0.513] with effort [18.9, 4.8].
  - The process was killed and the port released.
- `submission.tar.gz` was NOT rebuilt, and nothing was committed.

## Pass: IK service layer (`/ik/*`, `/ik_action/*`, `/ik_trial/*`) (2026-09-28)

### What changed

- **New `src/ik_node.py`** holds all three layers. Everything is on the one asyncio loop and talks
  only through the Registry. `register(registry)` must run after `arm_sim_node.register`.
  - `/ik/solve`:
    - Rejects a non-object request and a missing or non-finite `x`/`y` (bool, NaN and inf count
      as non-finite).
    - Fetches link lengths **fresh on every call** through `/arm_sim/set_params` with `{}`, then
      calls `kinematics.inverse_kinematics(x, y, lengths, phi)`. `phi` is passed as `None` for a
      2-link arm, which ignores it even if it's malformed. For 3 links, a `phi` of `null` counts
      as omitted, and a non-numeric one is rejected.
    - `Unreachable` becomes `result:false` with `str(e)`. `NotImplementedError` becomes "inverse
      kinematics not implemented yet". Any other exception, and any non-list, wrong-length or
      non-finite solution, is also rejected. Rejections return `values: {}`.
    - No reachability logic is duplicated here. Too far, too close, the wrist rule and the
      boundary tolerance are all delegated to `kinematics.py`.
  - `_IkAction` handles `/ik_action/send_goal`, `/ik_action/cancel_goal`, and publishes
    `/ik_action/feedback` and `/ik_action/result`:
    - `send_goal` validates `epsilon` (> 0, finite) and `success_hold` (>= 0, finite), then solves
      via `/ik/solve`. Only after the goal is accepted does it preempt the old goal (publishing
      its `"preempted"` result once), start the new one (`goal_id` "goal-<n>"), and publish the
      solution on `/joint_trajectory`. A rejected request has no side effects.
    - `cancel_goal` takes `{}`/`null`, or a `goal_id` that must match the active goal.
      Otherwise it rejects with no side effects. Success returns `{"goal_id": ...}`.
    - It subscribes to **`/joint_states`, and each message is one control tick**. The sim clock
      is `header.stamp` and the measured pose is `position`. Distance is
      `hypot(FK(q) - target)` with lengths fetched fresh.
    - Elapsed and dwell advance by `max(0, stamp delta)`, so a pause freezes them and a reset's
      backwards clock jump can't make them negative. Dwell resets to 0 on any tick outside
      epsilon, and first entry starts it at 0, so `success_hold=0` is met immediately. The
      comparison has a 1e-9 tolerance for float stamp sums.
    - When a goal concludes, it is detached before its result is published. That lets a result
      subscriber (the trial) submit the next goal re-entrantly.
    - Subscriber callbacks are wrapped by `_Subscriber`, which logs once and swallows errors. A
      malformed `/joint_states` or a broken FK therefore can't kill `arm_sim`'s `publish_loop`.
  - `_IkTrial` handles `/ik_trial/start`, `/ik_trial/skip`, `/ik_trial/stop`, and publishes
    `/ik_trial/status`. It is a pure `/ik_action` client: it calls `send_goal`/`cancel_goal`,
    subscribes to feedback and result, and never touches `/joint_states` or
    `/joint_trajectory`.
    - Targets come from uniform random `q` in [-pi, pi) mapped through
      `kinematics.forward_kinematics`. A 3-link target includes the sampled `phi`. There are up
      to 50 attempts per target.
    - When a goal is reached, it is counted and the next one is submitted straight away.
    - `start` cancels the leftover goal, resets the count and clock, and submits a target. If no
      target could be sampled or accepted, it returns `result:false` and the trial doesn't run.
    - `skip` and `stop` reject when no trial is running.
    - `stop` and duration expiry cancel the in-flight goal and clear `target`, `error` and
      `desired_positions` to null, and `action_status` to `"idle"`. `elapsed` is then frozen.
    - `error` and `desired_positions` come from the action's feedback (`distance_remaining`,
      `positions`). They are null until the first feedback tick after a submit.
    - `trial_status_loop` calls `tick()` at 10 Hz. `tick()` checks the duration, retries a failed
      replacement submission, and publishes status once any trial has been started.
- `src/main.py` registers `ik_node` and runs `trial_status_loop` as a third task. It is cancelled
  on shutdown like the others.
- **New `tests/test_ik_services.py`**, 33 tests, about 0.2 s. It patches `kinematics.*` with a
  **test-local reference FK/IK** defined in the test file. Ticks are driven by publishing
  hand-built `/joint_states`, and the trial gets a fake clock and a seeded RNG. The in-process
  wire test uses the `_ServerThread` harness, calling `gateway.stop()` before stopping the loop.
- `tests/test_live_process_e2e.py`: +1 test on the real 3-link subprocess. It covers `/ik/solve`
  and `send_goal` rejections, plus `cancel_goal`, `trial skip` and `trial stop` rejection with
  nothing active. It **skips its round-trip tail while `inverse_kinematics` raises
  NotImplementedError**. No goals or trials are started there, so the shared runtime isn't
  moved.
- Not touched: `kinematics.py`, `pid.py`, `arm_dynamics.py`, `integrators.py`, `arm_sim_node.py`.

### Defaults and spec decisions

- Defaults are `epsilon=0.05` m, `success_hold=0.5` s of sim time, and trial `duration=60` s.
- `epsilon`, `distance_remaining` and `final_distance` measure end-effector **position** only. A
  3-link `phi` shapes the IK solution but isn't part of the epsilon check.
- `final_distance` for a preempted goal is computed from the last measured pose, or null if no
  `/joint_states` has arrived yet.
- `target` includes `phi` only for a 3-link goal that supplied it.
- The trial's `elapsed`/`duration` clock is **wall time** (`time.monotonic`), because the harness
  may not read `/joint_states`' sim clock. Pausing the sim doesn't pause the trial.
- `/ik_trial/start` follows set_params' partial-override convention: omitted fields keep their
  current values, and supplied ones persist for later starts. Unlike set_params, **any invalid
  field rejects the whole start** (duration and epsilon must be > 0, success_hold >= 0) and leaves
  a running trial untouched.
- If another client preempts the trial's goal, the trial records `action_status: "preempted"`
  and waits for skip, stop or duration expiry rather than fighting over the single goal slot.
- start, skip and stop return the status snapshot as `values`.

### Depends on the owner's `kinematics.py`

- `/ik/solve`, `send_goal` success, action ticking (distance needs FK) and trial sampling (needs
  FK and IK) all return graceful rejections until the owner's functions work. Nothing crashes or
  hangs.
- In a live smoke run, the owner's in-progress FK (`phi, x, y = 0`) made `/ik_trial/start`
  reject with "could not sample a target: TypeError(...)", which is the intended behavior.
- **PID is not auto-enabled.** For a goal to actually be reached, a client must call
  `/pid_controller/enable {"data": true}`. Otherwise the arm just swings and goals never
  conclude "reached".
- Once IK lands, run `make test`. The e2e test's skipped tail should then pass. Also do a live
  check: enable PID, `send_goal`, and watch `/ik_action/result` report "reached".

### Checks performed

- `lsof -iTCP:9095 -sTCP:LISTEN` empty before and after; no stray `src/main.py`.
- `make build && make test`: 172 run, all OK, 1 skipped (the e2e round-trip tail, because IK is
  still a stub). That is 138 existing + 33 + 1.
- Live smoke test (`ARM_SIM_LINKS=3 python3 src/main.py`): the IK services rejected cleanly with
  the stub/WIP kinematics, `/ik_trial/stop` echoed a cleared status, and `/joint_states` and
  set_params were unaffected. SIGTERM gave a clean shutdown with no traceback.
- `submission.tar.gz` was NOT rebuilt, and nothing was committed.
