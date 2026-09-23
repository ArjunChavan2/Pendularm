# CLAUDE.md — known critical errors and fixes

Draft written from memory for the user to review/correct. Purpose: a running log of real bugs hit
in this repo (and the course pattern generally) and how they were actually fixed, so future
sessions don't rediscover them from scratch.

**Note on scope**: `src/integrators.py` (and later `arm_dynamics.py`, `pid.py`, `kinematics.py`)
are the user's own hand-written graded exercise — see `prompts/*.md`'s standing rule. Entries
below about those files stay at the conceptual/"what class of bug" level, not exact working code,
so this doc doesn't become a de facto answer key.

## Submission packaging

**`tar czf out.tar.gz --exclude=... .` fails the grader with `UnsafeArchive: unsafe archive path:
'.'`.** GNU tar includes an explicit `.` directory member when you tar the current directory this
way; the grader's `setup_submission.py` rejects that member outright. **Fix**: list top-level
entries by name instead — `tar czf out.tar.gz [--exclude=...] Makefile src tests ...`, never a bare
`.`. Full detail and a verification snippet: `spec/submission_layout.md`. Hit independently on both
Project 1 and Project 2; also documented in cross-project memory so future course repos get it
right from the start.

**Always rebuild and re-verify `submission.tar.gz` from a fresh extraction** (`tar xzf ... -C
/tmp/scratch && cd /tmp/scratch && make build && make test`) before sending it — don't trust the
working tree alone. Caught real staleness this way more than once (archive built before a fix
landed).

## Test/dev environment

**A stray `python3 src/main.py` left running from a manual `make run` will make every subsequent
test-suite run against port 9095 fail in a *misleading* way.** The symptom is not an obvious "port
in use" error — it's every service call in the affected test module failing with `result:false,
status:"no provider"`, because the test's own server never actually started (its `bind()` failed
silently inside the background thread, and the thread's exception doesn't surface as a normal test
failure). **Fix**: `lsof -iTCP:9095 -sTCP:LISTEN` to find it, `kill <pid>`, re-run. Consider this
first whenever a whole test module fails uniformly with "no provider" rather than a specific
assertion mismatch.

**Test server-thread harnesses (`_ServerThread`-style, used in every `test_*_integration*.py` /
`test_*_state_services.py` / `test_live_process_e2e.py`) must `await gateway.stop()` (which closes
the listening socket) *before* stopping the event loop/thread**, not after or never. Skipping this
leaves the port bound after teardown, causing intermittent `OSError: address already in use` when
a second server-backed test module starts right after it in the same `make test` run, depending on
module ordering. Already fixed once in Project 1, correctly replicated here — if a new test module
copies this harness pattern, copy this part too.

## `expr.py` — domain-error handling (infra code, not the graded exercise; documented in full)

The spec requires `/arm_sim/integration_step`'s `function` field to make evaluation-time domain
errors "become NaN/inf, the same as ordinary floating-point arithmetic" and never raise or hang.
**Python's own math/operators do not do this by default** — every one of these needed an explicit
guard:

- `math.sqrt(negative)`, `math.log(negative or zero)` raise `ValueError` → wrapped as `_safe_sqrt`/
  `_safe_ln` (return `nan`/`-inf` as appropriate).
- Float division by zero raises `ZeroDivisionError` in Python, unlike IEEE-754 → wrapped as
  `_safe_div` (sign-aware `inf`/`-inf`/`nan` for `0/0`).
- `math.exp(large)` raises `OverflowError` → wrapped as `_safe_exp` → `inf`.
- **The one that actually caused a live hang, not just a wrong rejection**: `(negative) ** (non-integer)`
  — e.g. `(-4)**0.5` — doesn't raise at all; Python's `**` silently returns a `complex` number.
  That complex value flowed all the way into the service response, and `json.dumps` raised
  `TypeError` trying to serialize it — but that failure happened *outside* the gateway's
  handler-call `try/except` (which only wraps the handler invocation, not the subsequent
  `Connection.send`), so no `service_response` was ever sent and the calling client hung until its
  own timeout. The gateway process itself stayed up and answered other requests fine — only that
  one request hung. **Fix**: `_safe_pow` checks `isinstance(result, complex)` → `nan`, plus
  catches `ZeroDivisionError` (`0**negative`) and `OverflowError` (too-large result), all before
  the result can reach a JSON encoder.
- `math.sin/cos/tan(inf)` raise `ValueError` → wrapped as `_safe_trig` (checks `math.isfinite`
  first) → `nan`.

**General lesson for any future expression-evaluator or numeric service in this course**: assume
Python's raw math functions/operators will raise on the exact inputs a spec says should gracefully
degrade to NaN/inf, and audit every one explicitly — don't assume "it's just arithmetic" means
it's safe.

**Related, lower-severity**: Python's `json.dumps` (default `allow_nan=True`, unchanged in
`gateway.py`) serializes `NaN`/`Infinity`/`-Infinity` as bare, non-standard JSON tokens (not valid
per strict RFC 8259). Flagged for awareness, not currently treated as a bug to fix — the grader's
own JSON parser needs to tolerate this or the whole "let it become NaN/inf" spec requirement
wouldn't be checkable at all.

## Grader startup-probe behavior

**The autograder's startup probe calls `/arm_sim/set_params` with an empty `{}` request and
expects a prompt echo of current parameters — if no provider is registered, the *entire grading
run* can fail at the startup stage** (`"did not echo current parameters before startup deadline"`)
before any category-specific grading even happens, even for e.g. just the Integrators checkpoint
category. **Fix**: `/arm_sim/set_params`, `/arm_sim/set_integrator`, and `/arm_sim/pause` are pure
parameter/mode *storage* — no dynamics or PID needed — so they can (and should) be built early,
well before `arm_dynamics.py`/`pid.py` exist, purely to satisfy this probe. `/arm_sim/reset` is the
one exception left unbuilt, since the spec explicitly ties it to resetting PID controller state.

Unconfirmed: whether the probe checks other `/arm_sim/*` endpoints too (e.g. `/arm_sim/reset`)
beyond the one error message actually seen — worth re-submitting to check rather than assuming
this is now fully resolved.

## Integrators (`src/integrators.py`) — conceptual gotchas only, not exact fixes

- **A second-order system's RK4 is not the same shape as scalar-ODE-textbook RK4.** The familiar
  `k1 = f(t,x)`, `k2 = f(t+dt/2, x+k1*dt/2)`, ... formula is for a first-order system. Extending it
  to `(q, qdot)` means treating that pair as one combined state whose own rate of change is
  `(qdot, qddot)` — so *every* stage needs **two** rate values (a q-rate and a v-rate), built from
  a genuinely updated trial `(q, qdot)` pair each time, not just a different time argument reusing
  the original untouched state.
- **Changing the time argument passed to the acceleration function without also updating the state
  argument to match** is the single most common mistake across every method here (hit in early
  drafts of `midpoint`, `verlet`, and `rk4` alike) — the state and time must represent the *same*
  point along the trajectory on every evaluation.
- **Velocity Verlet's second acceleration evaluation must use the provisional velocity estimate,
  not the original velocity** — easy to compute the provisional value and then simply forget to
  use it in the very next line.
- **Plain Python `list`s don't support elementwise arithmetic.** `list + list` concatenates;
  `list * float` raises `TypeError`. Elementwise vector math needs `zip()` + a list comprehension
  (no numpy — this project is stdlib-only per the offline-build constraint, same reasoning as the
  submission-packaging note above).
