# Test Results — Project 2 checkpoint: `/arm_sim/integration_step` + state-storage stubs

Independent verification pass, per `prompts/TEST.md`. `agent-notes/PLAN.md`/`IMPLEMENTATION.md`
were written retroactively for the *first* pass and were not updated for the second round of work
that fixed `AUDIT.md`'s findings and added `/arm_sim/set_params`, `/arm_sim/set_integrator`,
`/arm_sim/pause` — everything below was verified directly against the *current* code, not trusted
from those notes.

## Summary

**Overall result: PASS.** All behavior in scope is correct and matches the spec. The existing
68-test suite (`make test`) passes, and I added an additional real-subprocess end-to-end test
module (`tests/test_live_process_e2e.py`, 7 tests) that independently re-confirms, against a
genuine `make run` OS subprocess and a from-scratch TCP client (not the existing in-process
threaded-server test pattern, and not `tests/client_helper.py`), that:

- `AUDIT.md`'s three `expr.py` findings (the `(-4)^0.5` hang, the `0^-1`/`t^1000` rejections, the
  `sin(1/t)` rejection) are all genuinely fixed in the current code, not just claimed fixed.
- The three state-storage services (`/arm_sim/set_params`, `/arm_sim/set_integrator`,
  `/arm_sim/pause`) work correctly at both 2-link and 3-link, including per-field-independent
  validation within a single request.
- The full spec-required rejection set for `/arm_sim/integration_step`, and post-rejection
  responsiveness, hold over a real subprocess.
- `main.py`'s SIGTERM handling is genuinely clean (logs "shutting down", exits with no traceback,
  releases the port) when the signal reaches the Python process.

Total test count after this pass: 75 (68 pre-existing + 7 new), all passing. Full suite run time
~1.7s for `make test` (the new e2e module's real-subprocess tests add real wall-clock time but
stay well within the harness).

No implementation defects found. One informational, already-known limitation
(`steps` as a whole-number `float`, e.g. `5.0`, is rejected — AUDIT.md Finding 4) was
independently reconfirmed but is not a spec violation (spec types `steps` as `u64`) and is not
recommended for a fix. One test-harness-only artifact was found and diagnosed (see "Failures"
below) — it is **not** an implementation defect.

## Test environment

- macOS (darwin), Python 3 (system `python3`), no external dependencies beyond the standard
  library.
- `cd ~/Pendularm && make build` — compiles `src/*.py` via `py_compile`. Clean.
- `cd ~/Pendularm && make test` — runs `python3 -m unittest discover -s tests -v`. 75/75 passing.
- `cd ~/Pendularm && ARM_SIM_LINKS=<2|3> make run` — launches the real gateway on
  `127.0.0.1:9095` in the foreground; used directly (backgrounded) for genuine end-to-end checks,
  separately from the existing in-process-threaded-server test pattern.
- Confirmed `src/arm_dynamics.py`, `src/pid.py`, `src/kinematics.py` do not exist yet, and
  `src/integrators.py` is present and fully implemented (`euler`, `midpoint`, `verlet`, `rk4`,
  `METHODS`) — per the standing project rule, this is expected/in-progress hand-written work, not
  treated as a defect. `src/integrators.py`'s own algorithmic correctness was explicitly excluded
  from this test pass per the task instructions; only `arm_sim_node.py`/`expr.py`'s *integration*
  with its `(accel_fn, t, q, qdot, dt) -> (q_next, qdot_next)` / `State = list[float]` / `METHODS`
  interface was verified.
- All manual live-process checks were followed by explicit cleanup (`kill -TERM` on both the
  `make` and `python3 src/main.py` PIDs, `lsof -i :9095` / `ps` confirmation) — no stray process
  or held port was left behind by this test pass.

## Tests performed

### 1. Existing suite, run as-is

- **Requirement/risk:** all previously-written tests still pass against current code.
- **Method:** `make test`.
- **Expected:** all pass.
- **Observed:** 68/68 passing (14 `test_integrators.py`, 28 `test_expr.py`, 8
  `test_arm_sim_integration_step.py`, 18 `test_arm_sim_state_services.py`).
- **Result:** PASS.

### 2. AUDIT.md Finding 1 — `(-4)^0.5` hang, re-verified over a real subprocess

- **Requirement/risk:** a negative-base/fractional-exponent `^` expression must return a prompt
  `service_response` with `result:true` and NaN-valued entries, not hang.
- **Method:** real `make build && ARM_SIM_LINKS=3 make run` as a genuine OS subprocess (via
  `subprocess.Popen`, not an in-process thread), a from-scratch raw-socket client (not
  `tests/client_helper.py`, to avoid any bug shared between the client helper and the production
  code hiding from both) calling `/arm_sim/integration_step` with
  `{"function": "(-4)^0.5", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 3, "integrator": "euler"}`,
  measuring wall-clock time to the `service_response`.
- **Expected:** `result:true`, response arrives well under the 5s client timeout, NaN values
  present in `positions`.
- **Observed:** response arrived in well under 1s; `result:true`; `positions[1:]` contained NaN
  values as expected. Also independently confirmed via `expr.py`'s own `_safe_pow` (catches
  `ZeroDivisionError`/`OverflowError`, and explicitly checks `isinstance(result, complex)` →
  `nan`) and via `tests/test_expr.py::test_negative_base_fractional_exponent_is_nan_not_complex`.
- **Result:** PASS. This is now committed as
  `tests/test_live_process_e2e.py::test_negative_base_fractional_power_does_not_hang_over_real_gateway`.

### 3. AUDIT.md Finding 2 — `0^-1` / `t^1000` overflow, re-verified over a real subprocess

- **Requirement/risk:** `ZeroDivisionError`/`OverflowError` from `**` must become inf, not an
  internal-error rejection.
- **Method:** same live subprocess; `"t^-1"` evaluated at `t=0` (i.e. `0^-1`), and `"t^1000"` with
  `dt=500` (i.e. `500^1000`, guaranteed overflow), both via `/arm_sim/integration_step`.
- **Expected:** both `result:true`.
- **Observed:** both `result:true`. `expr.py`'s `_safe_pow` explicitly handles both
  `ZeroDivisionError` (→ `inf`) and `OverflowError` (→ `inf`/`-inf` by sign).
- **Result:** PASS. Committed as
  `test_zero_to_negative_power_and_overflow_do_not_reject_over_real_gateway`.

### 4. AUDIT.md Finding 3 — `sin`/`cos`/`tan` of infinite input, re-verified over a real subprocess

- **Requirement/risk:** `math.sin`/`cos`/`tan` raising `ValueError` on non-finite input must not
  leak out as a rejection.
- **Method:** same live subprocess; `"sin(1/t)"` at `t=0` (`1/t` → `inf` via `_safe_div`, then fed
  into `sin`).
- **Expected:** `result:true`.
- **Observed:** `result:true`. `expr.py` wraps `sin`/`cos`/`tan` with `_safe_trig`, which checks
  `math.isfinite` before calling the underlying function and returns NaN otherwise.
- **Result:** PASS. Committed as `test_trig_of_infinite_input_does_not_reject_over_real_gateway`.

### 5. Full spec-required rejection set + post-rejection responsiveness, over a real subprocess

- **Requirement/risk:** unparseable `function`, unknown `integrator`, `dt<=0`, `steps==0` must
  each reject cleanly (`result:false`, non-empty `status`), and the runtime must answer a
  subsequent good request normally afterward.
- **Method:** live subprocess; each rejection case sent, followed by a known-good request, over
  the same TCP connection. Also checked negative `steps` (not just `==0`) separately.
- **Expected:** each bad case `result:false` with non-empty `status`; each following good case
  `result:true`.
- **Observed:** all five rejection cases (`unparseable function`, `unknown integrator`, `dt<=0`,
  `steps==0`, `steps<0`) rejected correctly with a non-empty status string; the runtime answered
  the following good request correctly every time, on the same connection.
- **Result:** PASS. Committed as `test_full_rejection_set_and_responsiveness_after_each` (the
  `steps<0` case was also manually verified but not separately committed, since it's covered by
  the existing `arm_sim_node.py` code path already exercised by `steps==0`).

### 6. Response shape/invariants across all four integrators, over a real subprocess

- **Requirement/risk:** `times[0]==0`, `positions[0]==x0`, `velocities[0]==xdot0`, one entry per
  step plus the starting point — for every integrator, not just the one case the existing test
  suite checks in detail (`rk4`, in `test_constant_forcing_matches_closed_form`).
- **Method:** live subprocess; `{"function": "t^2 - 1", "x0": 2.5, "xdot0": -1.5, "dt": 0.2,
  "steps": 4}` called once per integrator name.
- **Expected:** for each integrator, `times[0]==0.0`, `positions[0]==2.5`, `velocities[0]==-1.5`,
  and all three arrays have length 5 (steps + starting point).
- **Observed:** held for all four integrators.
- **Result:** PASS. Committed as `test_response_shape_invariants_all_integrators_real_gateway`.

### 7. State services on a real 3-link runtime, over a real subprocess

- **Requirement/risk:** `/arm_sim/set_params`, `/arm_sim/set_integrator`, `/arm_sim/pause` must
  work correctly against a genuinely 3-link-configured runtime (`ARM_SIM_LINKS=3`, read by the
  actual `main.py` at process startup, not just passed as a constructor argument inside an
  in-process test harness), with per-field-independent validation.
- **Method:** live subprocess launched with `ARM_SIM_LINKS=3`. Verified: empty-`{}` query returns
  3-entry `masses`/`lengths`; a request with a valid `gravity` alongside an invalid (wrong-length,
  2 instead of 3) `masses` in the *same* request applies gravity and rejects only masses;
  `set_integrator` and `pause` basic round-trips.
- **Additionally** (manual, not committed as a repeatable test but independently reproduced twice):
  - `set_integrator` with an invalid `method` alongside a valid `timestep` in the same request —
    timestep applied, method rejected, echoed values reflect the post-update state
    (`{'method': 'euler', 'timestep': 0.03}` then, on a second call with a valid method and an
    invalid negative `timestep`, `{'method': 'verlet', 'timestep': 0.03}` — confirming the
    *previous* timestep is retained, not reset to a default, when only the newly-submitted
    timestep is invalid).
  - `pause` correctly ignores an unknown extra field in the request body rather than rejecting the
    whole request (per the protocol's "unknown fields ignored" convention).
  - `set_params` on a 3-link runtime: a negative value inside an otherwise-correct-length `lengths`
    array (`[1.0, -2.0, 3.0]`) is rejected as a whole (the field-level check requires *all*
    elements positive, matching the spec's "each >0" requirement), while a valid `gravity` in the
    same request still applies.
- **Expected:** as above.
- **Observed:** all matched expected behavior exactly.
- **Result:** PASS. Committed as `test_state_services_on_real_three_link_runtime`.

### 8. `/arm_sim/*` state-service per-field independence (2-link, in-process, existing suite)

- **Requirement/risk:** "every field optional, empty `{}` queries current values" and per-field
  independent validation.
- **Method:** inspected and re-ran `tests/test_arm_sim_state_services.py`'s existing coverage
  (empty-query, partial update + echo of retained fields, invalid field not blocking valid fields
  in the same request, negative gravity, non-positive mass, unrecognized integrator method,
  non-positive timestep, non-bool `pause.data`) at 2-link, plus its dedicated `TestThreeLinkArm`
  class at 3-link (defaults have 3 entries, wrong-length array for 3-link rejected).
- **Expected/Observed:** all pass as written; independently spot-checked the underlying
  `_ArmSimState` methods in `src/arm_sim_node.py` by inspection — `set_params`, `set_integrator`,
  and `pause` each accumulate an `errors` list per field and only apply a field when that field's
  own check passes, matching the spec's independence requirement exactly.
- **Result:** PASS.

### 9. Clean `make build && make run` + separate-process TCP client + `SIGTERM` shutdown

- **Requirement/risk:** genuine end-to-end viability (not just in-process unittest harnesses), and
  clean shutdown per `spec/ROSBRIDGE_PROTOCOL.md`/general robustness expectations.
- **Method:** `make build` (subprocess, checked exit code), `ARM_SIM_LINKS=<2|3> make run`
  (subprocess, backgrounded, port-polled for readiness), then a from-scratch client process
  connecting over a real socket and exercising the checks above, then `SIGTERM` sent directly to
  the `python3 src/main.py` child process (found via `pgrep -P <make-pid>`), polling for process
  exit and confirming the port was released afterward.
- **Expected:** port opens after `make run`; process exits within 5s of `SIGTERM`; no traceback on
  stderr; port released.
- **Observed:** all held. stderr showed `ARM_SIM_LINKS=<N>` / `gateway listening on
  127.0.0.1:9095` / `shutting down`, then clean exit, no traceback, port released (confirmed via
  both a direct connect-attempt and `lsof -i :9095` returning nothing).
- **Result:** PASS. Committed as `test_zz_clean_sigterm_shutdown_of_underlying_python_process`
  (named to run last within its test class, since it kills the shared server subprocess).

### 10. `ARM_SIM_LINKS` invalid-value handling

- **Requirement/risk:** spec says `ARM_SIM_LINKS` should be `"2"` or `"3"`, defaulting to 2-link
  when unset — didn't directly state required behavior for an invalid (neither `"2"` nor `"3"`)
  value.
- **Method:** inspected `src/main.py`: an invalid value logs a warning to stderr
  (`f"ARM_SIM_LINKS={links_str!r} is not '2' or '3'; defaulting to 2"`) and proceeds with 2-link,
  rather than crashing the process.
- **Expected/Observed:** matches `agent-notes/AUDIT.md`'s own note that this is "not a defect
  against the checkpoint's own requirements" — the spec doesn't mandate hard-failing on an invalid
  value, and defaulting gracefully is a reasonable, non-crashing choice.
- **Result:** PASS (no stricter requirement to violate).

### 11. `steps` as a whole-number float (AUDIT.md Finding 4), independently reconfirmed

- **Requirement/risk:** whether a JSON `steps` value like `5.0` (a whole number, but encoded with
  a decimal point) is accepted.
- **Method:** live subprocess; `/arm_sim/integration_step` called with `"steps": 5.0`.
- **Expected (per AUDIT.md):** rejected, since `arm_sim_node.py`'s check is
  `isinstance(steps, int)`, which is `False` for a Python `float` even when whole-valued.
- **Observed:** `{'result': False, 'status': "'steps' must be a positive integer"}` — confirms
  AUDIT.md's Finding 4 is still present (as expected — Finding 4 was rated informational/low
  priority in the audit, not something flagged as requiring a fix before this test pass).
- **Result:** as-expected/PASS given its informational status; **not** re-raised as a new defect,
  since AUDIT.md already surfaced it and rated it informational (most JSON encoders emit bare
  integers for whole numbers; the spec types `steps` as `u64`, which doesn't obligate accepting a
  float-encoded whole number). Documented here per TEST.md's instruction to record evidence rather
  than silently drop a previously-known, still-present gap.

### 12. Large `steps` responsiveness (AUDIT.md unverified risk #3)

- **Requirement/risk:** a very large `steps` value (single-threaded asyncio event loop; the
  in-process handler runs synchronously) shouldn't take an unreasonable amount of time and starve
  other callers.
- **Method:** live subprocess; `/arm_sim/integration_step` with `steps=200000`, `dt=0.001`,
  `integrator=rk4` (the most expensive integrator, 4 accel evals/step → 800,000 evaluations),
  `function="sin(t)"`. Measured wall-clock time to response.
- **Expected:** completes in a reasonable time (no hard spec bound; used judgment).
- **Observed:** ~0.75s, `result:true`, 200,001-entry arrays as expected. Well within any
  reasonable grader timeout, and each individual bad-input rejection is essentially instant, so a
  bad request never blocks unrelated callers for a meaningful duration either.
- **Result:** PASS (no cap needed; not a real starvation risk in practice for spec-plausible
  inputs).

### 13. `expr.py` parser edge cases (existing `tests/test_expr.py`, independently spot-checked)

- **Requirement/risk:** precedence (`+`/`-` < `*`/`/` < `^` < unary minus < atoms, `^`
  right-associative, unary minus binds *tighter* than `^` per the spec's explicit, non-standard
  ordering), and the full malformed-input list (empty string, unbalanced parens, unknown
  identifier, trailing garbage, adjacent tokens, unrecognized character).
- **Method:** re-ran the existing 28 tests in `tests/test_expr.py`; manually hand-traced
  `-2^2` → `(-2)^2 == 4` (not `-(2^2) == -4`) and `2^3^2` → `2^(3^2) == 512` (not `(2^3)^2 == 64`)
  against `_Parser._parse_power`/`_parse_unary` to confirm the precedence chain in the code
  actually produces this, not just that the test asserts it.
- **Expected/Observed:** all pass; hand-trace confirms the grammar in `_Parser` genuinely
  implements the spec's stated (non-Python-standard) precedence order.
- **Result:** PASS.

### 14. Missing/`None` `args`, non-numeric fields, and other malformed request shapes

- **Requirement/risk:** the handler must not crash on a missing `function` field, `args: null`
  entirely, or other structurally-odd-but-not-crashing input.
- **Method:** live subprocess; `/arm_sim/integration_step` called with `args` missing the
  `function` key, and separately with `args: null` entirely.
- **Expected:** clean `result:false` with a descriptive status, no crash, no dropped connection.
- **Observed:** both produced `{'result': False, 'status': "missing or invalid 'function'"}` —
  `arm_sim_node.py`'s `args = args or {}` guard at the top of `_integration_step` correctly
  normalizes `None` to `{}` before field access.
- **Result:** PASS.

### 15. Out-of-scope confirmation: hand-implemented modules

- **Requirement/risk:** per this task's standing rule, `src/arm_dynamics.py`, `src/pid.py`,
  `src/kinematics.py` missing/`NotImplementedError` is expected, not a defect; `src/integrators.py`
  exists and its algorithmic correctness is explicitly out of scope for this pass.
- **Method:** confirmed via direct import attempts and `ls src/`.
- **Observed:** `src/arm_dynamics.py`, `src/pid.py`, `src/kinematics.py` do not exist yet (`import
  arm_dynamics` → `ModuleNotFoundError`). `src/integrators.py` exists, is fully implemented, and is
  covered by its own 14-test suite (`tests/test_integrators.py`, all passing) — not re-litigated
  here per the task's explicit scope boundary. `arm_sim_node.py`'s usage of `integrators.METHODS`
  and the `(accel_fn, t, q, qdot, dt) -> (q_next, qdot_next)` / `State = list[float]` contract was
  verified directly (see `_integration_step`'s loop: `q, qdot = method(accel_fn, t, q, qdot, dt)`,
  correctly threading the per-step `t` forward rather than freezing it — confirmed this doesn't
  degrade the closed-form-`f(t)=t` accuracy comparison the spec calls out, since `t` genuinely
  advances by `dt` each loop iteration before the next `method(...)` call).
- **Result:** PASS / correctly out of scope, as expected.

## Failures

No implementation defects were found. One test-harness-only artifact was investigated and
diagnosed:

### Non-failure artifact: `SIGTERM` to the `make run` wrapper process yields a negative
`returncode`

- **What happened:** an early version of my end-to-end check sent `SIGTERM` to the `make run`
  subprocess itself (the immediate child of `subprocess.Popen(["make", "run"], ...)`, i.e. GNU
  Make's own process, not the `python3 src/main.py` grandchild it spawns) and asserted
  `returncode == 0`. This failed: `returncode` came back as `-15` (terminated by `SIGTERM`).
- **Reproduction:** `subprocess.Popen(["make", "run"], ...)`, then `proc.send_signal(SIGTERM)`,
  then `proc.communicate()`, `proc.returncode == -15`.
- **Investigation:** I checked the actual stderr from that run and found `"shutting down"` *was*
  logged (confirming `main.py`'s own `SIGTERM` handler did fire and `gateway.stop()` did run), and
  no traceback appeared. I then repeated the test sending `SIGTERM` directly to the
  `python3 src/main.py` child PID (found via `pgrep -P <make-pid>`) instead of the `make` wrapper,
  and got a clean `"shutting down"` log line, no traceback, and the process exiting normally —
  this is what `test_zz_clean_sigterm_shutdown_of_underlying_python_process` now checks.
- **Conclusion:** this is a property of GNU Make's own signal handling (a process killed by a
  signal while running a foreground recipe conventionally re-raises/is reported as killed-by-that-
  signal, a POSIX/Make convention, not something `main.py` controls), **not an implementation
  defect**. The actual production code under test (`src/main.py`'s signal handling) behaves
  cleanly when the signal reaches it. Evidence points entirely to the test harness's choice of
  which process to signal, not to the implementation.
- **Disposition:** fixed in my own test code (now signals the correct child process); no
  production-code action needed.

## Remaining gaps

- **Live-arm-dependent spec requirements are entirely unverified** — not a gap in this pass so
  much as a scope boundary: gravity-load correctness (`tau == G(q)` at PID convergence),
  inter-joint coupling (3-link vs. 2-link), PID convergence/integral-reset semantics,
  `/arm_sim/reset`, `/joint_trajectory`/`/joint_states`, and all of `/ik/*`/`/ik_action/*`/
  `/ik_trial/*` cannot be tested yet because `arm_dynamics.py`, `pid.py`, `kinematics.py`, and the
  live sim loop don't exist. This matches the task's explicit framing (checkpoint scope only) and
  `README.md`'s own "Not yet built" list — flagging for whichever future Test pass covers that
  work, since `spec/PROJECT2_PENDULARM.md`'s "Testing" section requirements for those areas are
  currently 0% covered by any test in this repo.
- **`src/integrators.py`'s own numerical correctness** was explicitly out of scope for this pass
  per the task instructions (already independently verified elsewhere) and was not re-tested here,
  even though `tests/test_integrators.py` exists and passes. I did *not* independently verify, for
  example, that `midpoint`'s or `rk4`'s specific arithmetic is correct — only that
  `arm_sim_node.py` calls into `integrators.METHODS` with the right signature and threads `t`
  correctly, which is a wiring/integration concern, not an algorithmic one.
- **Concurrent multiple simultaneous clients** were not stress-tested (e.g. two TCP clients issuing
  overlapping requests at once) beyond what `spec/ROSBRIDGE_PROTOCOL.md`'s "one slow client must
  not prevent unrelated clients from making progress" concern implies — the large-`steps` timing
  check (Test 12) shows a single expensive request completes quickly (~0.75s for 200k rk4 steps),
  which bounds the worst case for this synchronous-handler architecture, but true multi-client
  concurrency (two sockets open at once, interleaved requests) wasn't directly exercised.
- **`agent-notes/AUDIT.md`'s Finding 5** (NaN/Infinity serialized as non-standard JSON tokens via
  Python's default `json.dumps(allow_nan=True)`) was independently reconfirmed at the wire level
  (a real response for `sqrt(-1)` contains the literal, non-RFC-8259 token `NaN` in the raw socket
  bytes) but, per the audit's own framing, this is inherited from `gateway.py`'s reuse (explicitly
  out of this project's scope to fault) and the spec doesn't address wire encoding of NaN/inf — not
  treated as a failure here, just re-confirmed as still true and worth the project owner's
  continued awareness if the actual Autograder.io client turns out to use a strict JSON parser.
- I did not attempt to test binary/non-UTF-8 input, lines over 4 MiB, or other
  `spec/ROSBRIDGE_PROTOCOL.md` generic-transport edge cases against this project's `gateway.py`,
  since it's byte-for-byte identical to Project 1's already-tested `gateway.py` (confirmed by
  `agent-notes/AUDIT.md`'s own `diff` check, and re-confirmed present verbatim here) and testing it
  again would be re-litigating Project 1's own test coverage rather than this project's own code.
