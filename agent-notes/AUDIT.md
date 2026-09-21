# Audit — Project 2 checkpoint: `/arm_sim/integration_step`

Scope: `src/registry.py`, `src/gateway.py`, `src/expr.py`, `src/arm_sim_node.py`, `src/main.py`,
`tests/client_helper.py`, `tests/test_expr.py`, `tests/test_arm_sim_integration_step.py`, and how
these correctly (or incorrectly) integrate with `src/integrators.py`'s already-verified interface.
`PLAN.md`/`IMPLEMENTATION.md` were written retroactively and are treated here only as claims to
verify, not as ground truth.

## Summary

The transport layer (`registry.py`, `gateway.py`) is confirmed byte-for-byte identical to Project
1's originals (`diff` against `~/A-Star-Path-Planning/src/{registry,gateway}.py` is empty) — the
"ported unchanged" claim holds, as does the "test harness closes its listening socket on teardown"
fix (`tests/test_arm_sim_integration_step.py`'s `_ServerThread.stop()` matches Project 1's
`test_gateway_protocol.py` pattern exactly, including the comment explaining why). `main.py`'s
signal handling also matches Project 1's proven pattern exactly. The response shape, the
`times[0]==0`/`positions[0]==x0`/`velocities[0]==xdot0` invariant, decoupling from any live arm
state, and all four of the spec's required clean-rejection cases (`dt<=0`, `steps==0`, unknown
integrator, unparseable function) are implemented correctly and match `integrators.py`'s actual
`(accel_fn, t, q, qdot, dt) -> (q_next, qdot_next)` / `State = list[float]` / `METHODS` contract.
The parser's precedence, including the spec's unusual "unary minus binds tighter than `^`" rule
(verified directly: `-t^2` at `t=2` evaluates to `4.0`, i.e. `(-t)^2`, not `-(t^2)`), is correctly
implemented and matches hand-traced behavior for every case I checked.

However, **`expr.py`'s "domain errors become NaN/inf, never raise" guarantee — the spec's central
requirement for the `function` field — is only partially implemented.** It was applied to `/`,
`sqrt`, `ln`, and `exp`, but not to `^` (raw Python `**`) or to `sin`/`cos`/`tan` (raw `math.sin`
etc.). This is a real, reproducible defect, not a theoretical one: I confirmed by direct
invocation (both at the `expr.py` level and end-to-end over a live gateway instance) that certain
legal, spec-compliant `function` strings cause either an incorrectly-rejected request or, in one
case, a genuine hang of that specific request until the caller's own timeout — the exact failure
mode the spec explicitly forbids ("stay fully responsive... not something that should take the
service down"). None of this is exercised by the existing 50-test suite (all 50 pass, confirmed by
re-running `make test`), so "the tests pass" does not currently establish this guarantee.

## Findings

### Finding 1 — `^` with a negative base and non-integer exponent silently produces a Python `complex` value, which hangs that specific request

- **Severity:** critical
- **Location:** `src/expr.py`, `_combine` (the `"^"` branch, line 191: `return lambda t: left(t) ** right(t)`)
- **Problem:** Python's `**` operator returns a `complex` number for `(negative float) ** (non-integer float)`, e.g. `(-4.0) ** 0.5 == (1.22e-16+2j)`. Unlike `/`, `sqrt`, `ln`, `exp`, the `^` operator has no domain-safety wrapper, so this complex value is neither rejected nor converted to NaN — it silently flows through the rest of the evaluation and into the response's `positions`/`velocities` arrays. When `arm_sim_node.py`'s handler returns successfully, `gateway.py` (`Connection.send`, called from `_handle_call_service`) calls `json.dumps` on the response; `json.dumps` raises `TypeError: Object of type complex is not JSON serializable` for a complex value. That `TypeError` occurs *outside* the try/except that wraps only the handler invocation (`gateway.py` lines ~166-174), so it propagates up into `_dispatch`'s outer try/except (lines ~117-154), which sends a generic `{"op":"status","level":"error",...}` message instead of a `service_response`. The caller's `call_service` (per `tests/client_helper.py` and the wire protocol) is waiting specifically for `op=="service_response"` with the matching `id` — it never arrives.
- **Reproduced live:** a real gateway instance (background thread, real socket) called with `{"function": "(-4)^0.5", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 3, "integrator": "euler"}` never received a `service_response`; the client call timed out after its full 3s timeout (`TimeoutError`). A follow-up call on the same connection succeeded immediately afterward, so the *service* stays up, but *this specific request* genuinely hangs from the caller's point of view — directly the behavior `spec/PROJECT2_PENDULARM.md`'s checkpoint section says must not happen ("Reject the request... rather than crashing or hanging... the runtime should stay fully responsive to later calls").
- **Why it matters:** This is a plausible grader input — `"function"` strings with a negative constant or a negative-valued sub-expression raised to a non-integer power are well within what "handle standard infix arithmetic... `^` for exponentiation" implies a grader might try, and the spec calls out `sqrt` of a negative as the paradigm domain-error case specifically because it's the same kind of thing `^` can produce.
- **Recommended action:** Wrap `^` similarly to `_safe_div`: catch/detect a complex result (e.g. `isinstance(result, complex)`) and coerce to `nan`, or precompute `math.isfinite`/sign checks before calling `**` to force real-domain-only exponentiation. This is a fix confined to `expr.py`; no change to `gateway.py` is required (its generic exception handling is otherwise correct and is out of this project's scope to fault), though it's worth noting the same class of caller-side hang would recur for *any* future handler that manages to slip a non-JSON-serializable value into a successful response.

### Finding 2 — `^` also raises uncaught `ZeroDivisionError` / `OverflowError` for other legal inputs, rejecting requests instead of returning NaN/inf

- **Severity:** major
- **Location:** `src/expr.py`, `_combine` (the `"^"` branch)
- **Problem:** Two more unguarded domain errors on the same code path:
  - `0.0 ** -1` (and any `0 ^ (negative)`) raises `ZeroDivisionError: 0.0 cannot be raised to a negative power` — reachable via, e.g., `"t^-1"` evaluated at `t=0`, a perfectly ordinary case for a `function` field that gets evaluated starting at `t=0`.
  - A large-enough result (e.g. `t^1000` at `t=500`) raises `OverflowError: (34, 'Result too large')` instead of the IEEE-754 `inf` that `10.0**400` etc. would normally produce for other operators.
  Both were confirmed directly against `expr.py`. Because these exceptions occur mid-loop inside `arm_sim_node._integration_step` (not in the final JSON-encode step), they *are* caught by `gateway.py`'s handler-call try/except, so the request does not hang — but it comes back as `result:false, status:"internal error: ..."`, i.e. a rejected request, not the `result:true` with NaN/inf-valued arrays the spec calls for ("A domain error at evaluation time... is fine to just let become NaN/inf... there's no need to special-case that").
- **Why it matters:** Same requirement as Finding 1, different manifestation. A grader comparing `/arm_sim/integration_step` against a closed-form reference for a function that happens to pass through a `^` domain edge case partway through the requested `steps` would get a hard rejection of the *entire* multi-step response (losing whatever steps up to that point would have been valid) rather than NaN/inf entries from that step onward, and the terse `status` string ("internal error: ...") doesn't distinguish this from a genuine bug.
- **Recommended action:** Same fix location as Finding 1 — a `_safe_pow` helper analogous to `_safe_div`/`_safe_sqrt` that catches `ZeroDivisionError`/`OverflowError` and complex results, returning `nan`/`inf` as appropriate, used in `_combine`'s `"^"` branch.

### Finding 3 — `sin`/`cos`/`tan` are unguarded `math.sin`/`math.cos`/`math.tan`; infinite input raises instead of producing NaN

- **Severity:** minor
- **Location:** `src/expr.py`, `_FUNCTIONS` dict (lines 54-62) — `"sin": math.sin`, `"cos": math.cos`, `"tan": math.tan` are registered directly, unlike `sqrt`/`ln`/`exp` which go through `_safe_*` wrappers.
- **Problem:** `math.sin(float('inf'))` (and `cos`, `tan`) raises `ValueError: expected a finite input, got inf`, not IEEE-754 NaN. This is reachable through ordinary composition, e.g. `"sin(1/t)"` evaluated at `t=0`: `1/t` correctly becomes `inf` via `_safe_div`, but passing that `inf` into `sin(...)` then raises. Confirmed directly. As in Finding 2, this exception is caught by `gateway.py`'s handler-call try/except (no hang), but the request is rejected with an internal-error status instead of returning NaN for that data point, per spec's domain-error-becomes-NaN/inf rule.
- **Why it matters:** Lower likelihood of being hit by a grader than Finding 1/2 (requires a composed expression producing `inf`/`nan` as an argument to a trig function, rather than a single obviously-adversarial input), but it's the same category of gap and the fix is essentially free once Finding 2's helper pattern exists.
- **Recommended action:** Wrap `sin`/`cos`/`tan` (e.g. a small `_safe_trig(fn)` factory) to catch `ValueError` from non-finite input and return `nan`.

### Finding 4 — `steps` validation requires a Python `int`, which may reject a spec-legal encoding

- **Severity:** informational
- **Location:** `src/arm_sim_node.py`, `_integration_step`, line 46: `if not isinstance(steps, int) or isinstance(steps, bool) or steps <= 0:`
- **Problem:** The spec types `steps` as `u64`. If a client's JSON encoder emits an integer-valued `steps` as a JSON number with a decimal point or exponent (e.g. `5.0`), Python's `json.loads` will hand `arm_sim_node.py` a `float`, which this check rejects outright as `"'steps' must be a positive integer"` even though the value is a legitimate whole number. Most JSON encoders (Python's own `json`, JS's `JSON.stringify`) emit bare integers for whole numbers, so this is unlikely to bite in practice, but it's a narrower acceptance than the spec's numeric type alone guarantees.
- **Why it matters:** Only matters if the grading client's JSON serialization emits a decimal-point-bearing float for what is conceptually a whole-number step count; can't be confirmed without knowing the grader's implementation.
- **Recommended action:** Consider accepting a whole-number `float` (`isinstance(steps, (int, float)) and not isinstance(steps, bool) and float(steps).is_integer() and steps > 0`), converting to `int` before use. Low priority given the likely encodings in practice.

### Finding 5 — NaN/Infinity values are serialized as non-standard JSON tokens

- **Severity:** informational
- **Location:** `src/gateway.py`, `Connection.send` (`json.dumps(message, ...)`, default `allow_nan=True`) — interacting with `expr.py`/`arm_sim_node.py` intentionally producing `float('nan')`/`float('inf')` values per spec.
- **Problem:** Python's `json.dumps` with default settings emits the literal (non-RFC-8259) tokens `NaN`, `Infinity`, `-Infinity` for those float values, rather than encoding them as strings or omitting them. Python's own `json.loads` accepts these back (so this project's own test client round-trips fine, as seen in `test_division_by_zero_does_not_raise`), but a strict external JSON parser (many languages' standard libraries) would reject a response containing these tokens as invalid JSON.
- **Why it matters:** This is inherent to reusing `gateway.py` unchanged (out of this project's scope to fault) combined with the spec's explicit instruction to let domain errors become NaN/inf — the spec doesn't address wire encoding of these values, so this may be expected/acceptable, but it's a real interoperability risk if the grader's client uses a strict JSON parser.
- **Recommended action:** None from this audit — flagging for the test agent / owner's awareness. No production-code opinion offered given it's a `gateway.py` behavior inherited by design.

## Unverified risks (for the test agent)

1. **Finding 1 reproduction over the wire** — confirmed via a manual live-gateway script during this audit (not part of the committed test suite); the test agent should add this as a first-class regression test (`"(-4)^0.5"` or similar) and confirm the fix, once applied, returns a normal `service_response` with NaN-valued entries rather than hanging.
2. **Finding 2 and 3 reproduction over the wire** — same; confirmed at the `expr.py`/in-process level and reasoned through the `gateway.py` code path, but not independently exercised end-to-end over the real socket in this audit (only Finding 1 was). Worth a direct wire-level check for `"t^-1"` at `x0` such that `t=0` is reached, `"t^1000"` with large `steps`, and `"sin(1/t)"` starting at `t=0`.
3. **Behavior under a very large `steps` value** (e.g. 10^8+) — no cap exists; not a correctness bug per the spec (which doesn't require one), but worth a timing/responsiveness check to confirm a single pathological request can't starve other callers for an unacceptable duration, especially since the gateway is single-threaded/asyncio (one slow in-process handler call blocks the event loop for everyone until it returns — `spec/ROSBRIDGE_PROTOCOL.md`'s "one slow client must not prevent unrelated clients from making progress" concern applies to this literally, since `_integration_step` runs synchronously in the event loop).
4. **`ARM_SIM_LINKS` future behavior** — currently read/validated in `main.py` but unused (correctly out of scope per the checkpoint), silently defaulting to `"2"` on an invalid value rather than failing the process. Not a defect against the checkpoint's own requirements, but worth revisiting once the live arm actually consumes it — confirm the intended failure behavior for an invalid value against whatever the eventual spec section for launch requires (spec text for this wasn't in the checkpoint-scoped section reviewed here).
5. **Precision/rounding at exact reachability boundaries and closed-form accuracy comparisons across all four integrators** for the `/arm_sim/integration_step` service specifically (as opposed to `integrators.py`'s own already-tested unit accuracy) — inspection confirms `arm_sim_node.py` wires `integrators.METHODS` correctly and doesn't freeze `t` (the per-step `t` is threaded through the loop correctly, and the actual fractional-time evaluation is `integrators.py`'s own concern, already independently verified) — but an end-to-end numerical cross-check (e.g. `f(t)=t` against the closed-form `t^3/6`) for all four integrators through the full service path, not just `test_constant_forcing_matches_closed_form`'s single case, would still be worth the test agent's time.

If no other problems turn up in that follow-on testing, the highest-risk behaviors to prioritize are Finding 1 (a genuine hang) and Finding 2/3 (spec-mandated NaN/inf behavior not actually met) — everything else in this checkpoint's scope (transport reuse, response shape, rejection cases, precedence, decoupling, test-harness socket cleanup) checked out correctly under inspection and direct testing.
