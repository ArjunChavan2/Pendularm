# Plan — `arm_dynamics.py` contract + live gravity-driven sim loop

**Scope of this pass** (per `prompts/PLAN.md` and this session's task): plan only (1) the
contract/interface `src/arm_dynamics.py` must expose, and (2) wiring that contract into a minimal
live simulation loop inside `src/arm_sim_node.py` that makes the arm swing freely under gravity
(PID disabled, `tau = 0`) and continuously publishes `/joint_states`. Explicitly **not** in scope:
`pid.py`, `/joint_trajectory`, `/arm_sim/reset`, `kinematics.py`, any `/ik/*` node. See git history
and `agent-notes/IMPLEMENTATION.md`/`TEST_RESULTS.md` for the already-completed checkpoint
(`/arm_sim/integration_step` + `/arm_sim/set_params`/`set_integrator`/`pause` state storage) —
not repeated here.

**Standing rule carried into this plan**: `src/arm_dynamics.py`, `src/integrators.py`,
`src/pid.py`, `src/kinematics.py` are hand-implemented by the project owner. This plan defines
`arm_dynamics.py`'s public contract (signature, arguments, return value, invariants) only — never
its internal derivation/linear-algebra. `src/integrators.py` is already complete and hand-verified
(`euler`, `midpoint`, `verlet`, `rk4`, `METHODS` dict, `State = list[float]`,
`accel_fn(t, q, qdot) -> qddot`) — treated as a fixed, working dependency here, not something this
pass touches.

## Goal

Make the arm visibly move under gravity as soon as the runtime starts (PID disabled, `tau = 0`),
continuously publishing `/joint_states` at a reference 60 Hz, using `src/integrators.py`'s
`METHODS` and the *current* `_ArmSimState` (gravity/masses/lengths/integrator method & timestep/
paused) every physics tick — so this is the same correctness milestone the spec calls out
verbatim: "Starting from the reset pose with PID disabled, the arm should visibly move ... unless
that particular pose happens to be a gravity equilibrium."

## Relevant requirements

(All from `spec/PROJECT2_PENDULARM.md`; see "Dynamics," "Numerical integrators," and the
`/joint_states` part of "Topics and services" for full text — summarized here, not restated in
full.)

- Manipulator equation `M(q) qddot + C(q,qdot) qdot + G(q) = tau`; forward dynamics solves
  `qddot = M(q)^-1 (tau - C(q,qdot)qdot - G(q))`.
- Serial planar `n`-link (`n` = 2 or 3) arm, uniform rigid rods, link `i` hinged to link `i-1`
  (link 0 = fixed base at a static frame's origin), `q_i` is joint `i`'s own relative rotation,
  `phi_i = q_1 + ... + q_i` is link `i`'s absolute orientation. Gravity along world `-y`,
  magnitude `g`.
- Must be derived from Lagrangian mechanics for a *general* `n`-link arm, not two hand-derived
  2-link/3-link special cases — explicitly graded ("testing both link counts against the same
  code is exactly how that kind of mistake tends to surface").
- `M(q)` symmetric, positive-definite for any physically valid arm (positive lengths/masses).
- The linear-algebra method used to solve/invert `M(q)` is explicitly the implementer's choice —
  not part of the required contract, not planned here.
- `gravity`/`masses`/`lengths` for the live arm are exactly `_ArmSimState`'s current values
  (settable via `/arm_sim/set_params`); the live loop must read *current* state each tick, not a
  cached copy from startup, so a mid-run `set_params` call takes effect.
- `/joint_states` (publish): `{"header": {...}, "name": [...], "position": [...],
  "velocity": [...], "effort": [...]}`, continuous, reference 60 Hz, decoupled from the physics
  timestep. `name` has exactly one entry per joint; the three data arrays are parallel. Timestamp
  may be simulation time or wall time.
- The live loop must use `integrators.METHODS` and the *current* `_ArmSimState.integrator_method`/
  `integrator_timestep` (settable via `/arm_sim/set_integrator`) to step `(q, qdot)` every tick —
  same requirement the checkpoint service already satisfies for its own standalone use, now
  applied to the live arm.
- `tau` is the PID controller's output, or zero while disabled. This pass has no `pid.py` yet, so
  `tau` is always the zero vector.
- A "reset pose" is needed as the initial `(q, qdot)`; the spec leaves this open beyond gravity
  being along `-y` and the base being fixed — `q = qdot = [0]*n` is spec-compatible.
- Explicitly out of scope for this pass, noted in the task: a too-large timestep going unstable
  under an *enabled* PID loop is expected behavior, but irrelevant here since `tau = 0` (no active
  instability driver) — not something to engineer around in this pass.
- The live loop must run concurrently with the gateway's own client-handling on the same asyncio
  event loop — `main.py` currently just starts the gateway and awaits a stop event.

## Repository observations

- `src/integrators.py` (owner-written, complete, tested): `State = list[float]`,
  `AccelFn = Callable[[float, State, State], State]` — `accel_fn(t, q, qdot) -> qddot`. Each of
  `euler`/`midpoint`/`verlet`/`rk4` has signature `(accel_fn, t, q, qdot, dt) -> (q_next,
  qdot_next)`. `METHODS: dict[str, ...]` maps `"euler"|"midpoint"|"verlet"|"rk4"` to these. This is
  exactly the abstraction the live loop must drive — no changes needed here.
- `src/arm_sim_node.py` (current state, `register(registry, links=2)`):
  - `_integration_step`: the checkpoint service, fully decoupled from any live state — untouched
    by this pass.
  - `_ArmSimState.__init__(self, links)`: currently holds `gravity` (9.81 default),
    `masses`/`lengths` (`[1.0]*links` default), `integrator_method` (`"euler"` default),
    `integrator_timestep` (`0.01` default), `paused` (`False`). **Does not yet hold `q`/`qdot`/sim
    clock** — this pass needs to add them.
  - `set_params`/`set_integrator`/`pause` each validate fields independently and mutate
    `_ArmSimState` in place; nothing currently reads `paused`, or any dynamics-relevant field, at
    runtime — this pass is what makes that state actually *consumed*.
  - `register()` currently returns `None` and only registers service handlers; main.py has no
    handle to the constructed `_ArmSimState`.
- `src/main.py`: `run()` builds `Registry`, calls `arm_sim_node.register(registry, links)`, starts
  `Gateway`, installs `SIGINT`/`SIGTERM` handlers that set `stop_event`, then
  `await stop_event.wait()` followed by `await gateway.stop()`. No other concurrent task exists
  yet — this is the extension point for the physics/publish loops.
- `src/registry.py`: `Registry.publish(topic, msg)` unconditionally sends to all current
  subscribers of `topic` — the `is_advertised` gating only applies to the external wire `publish`
  op inside `gateway.py`'s dispatch (see `Registry.is_advertised`'s own docstring: "Only the
  external wire `publish` op is gated by this... internal nodes that call `publish()` directly...
  bypass the wire dispatch layer entirely and are unaffected."). So the live loop can call
  `registry.publish("/joint_states", msg)` directly, in-process, with no `advertise` call needed —
  consistent with how this codebase's other internal nodes are expected to publish. Confirmed by
  reading `registry.py` directly, not just inferring from the docstring.
  Also confirmed: `Registry` is documented "intended to run entirely on a single asyncio event
  loop (no threads), so no locking is used" — the physics loop, publish loop, and the gateway's
  own service-handler calls will all run as coroutines on the *same* event loop (no threads), so
  no locking is needed around `_ArmSimState`'s field reads/writes as long as nothing `await`s in
  the middle of a read-modify-write sequence on it.
- `src/gateway.py`: confirmed generic/domain-agnostic, ported unchanged from Project 1 — no
  changes needed or planned.
- CLAUDE.md's Python-list-arithmetic gotcha (list `+`/`*` don't do elementwise math; need
  `zip()` + comprehensions) applies to the wiring code in `arm_sim_node.py` too (e.g. anywhere it
  builds/copies `q`/`qdot` lists), not just to the owner-written modules.

## Proposed architecture

- **`src/arm_dynamics.py`** (new file, owner-implemented; this pass only fixes its public
  contract): exposes one function, `forward_dynamics`, computing `qddot` from the manipulator
  equation for a general `n`-link planar arm. See "Interfaces and data flow" below for the exact
  signature. Per the standing rule, an Implement phase working from this plan should create this
  file as a **stub only** — the agreed signature, a docstring capturing the contract/invariants
  below, and a body that raises `NotImplementedError` — mirroring how `src/integrators.py` itself
  was originally scaffolded ("STUB: implement this file yourself") before the owner filled it in
  by hand. An agent must not write the Lagrangian derivation or the `M(q)` solve/invert step.
- **`src/arm_sim_node.py`** (extended, agent-built):
  - `_ArmSimState` gains three new fields: `q: list[float]`, `qdot: list[float]` (both
    initialized to the reset pose, `[0.0]*links`), and `sim_time: float` (initialized to `0.0`).
    No new validation logic needed for these — they're not settable via any request field in this
    pass (that's `/arm_sim/reset`'s job, out of scope here).
  - A new module-level coroutine `physics_loop(state: _ArmSimState) -> None`: an infinite loop
    that paces itself by the *current* `state.integrator_timestep` each iteration, and — when not
    paused — builds a `tau = [0.0] * state.links` zero-effort vector, wraps
    `arm_dynamics.forward_dynamics` in a small closure matching `integrators.AccelFn`'s
    `(t, q, qdot) -> qddot` shape (see below), calls the *current* `integrators.METHODS[
    state.integrator_method]` with it, and writes the result back into `state.q`/`state.qdot`,
    advancing `state.sim_time` by the timestep actually used. All of `gravity`/`masses`/`lengths`/
    `integrator_method`/`integrator_timestep`/`paused` are read fresh from `state` every
    iteration — never cached — satisfying the "mid-run `set_params`/`set_integrator` takes effect"
    requirement.
  - A new module-level coroutine `publish_loop(registry: Registry, state: _ArmSimState) -> None`:
    an infinite loop that sleeps a fixed `1/60` s each iteration and calls
    `registry.publish("/joint_states", msg)` with the *current* `state.q`/`state.qdot` (and an
    all-zero `effort` array, since `tau` is always zero this pass) — independent of `paused` (a
    paused sim must keep publishing its frozen snapshot, per spec) and independent of whatever
    `integrator_timestep` currently is (decoupled rate, per spec).
  - `register(registry, links=2)` changes to *construct and return* the `_ArmSimState` instance
    (instead of returning `None`), so `main.py` can hand it to both loops. Existing service
    registrations (`/arm_sim/integration_step`, `/arm_sim/set_params`, `/arm_sim/set_integrator`,
    `/arm_sim/pause`) are unchanged.
- **`src/main.py`** (extended, agent-built): after `state = arm_sim_node.register(registry,
  links)` and `await gateway.start()`, create two background tasks —
  `asyncio.create_task(arm_sim_node.physics_loop(state))` and
  `asyncio.create_task(arm_sim_node.publish_loop(registry, state))` — running concurrently with
  the gateway's own per-connection coroutines on the same event loop (this is exactly what
  `asyncio.create_task` is for: it schedules a coroutine to run interleaved with everything else
  already on the loop, without blocking `gateway`'s `_handle_client` loops for other connections).
  On shutdown (after `stop_event.wait()` returns, alongside `gateway.stop()`), both tasks must be
  explicitly cancelled and awaited (catching `asyncio.CancelledError`) so the process still exits
  cleanly under `SIGTERM`/`SIGINT` — the existing clean-shutdown behavior CLAUDE.md/
  `TEST_RESULTS.md` already verified must keep holding.
- No changes to `registry.py`, `gateway.py`, `integrators.py`, `expr.py`.

## Interfaces and data flow

### `arm_dynamics.py` contract (the only part of this file this pass specifies)

```python
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
    for a serial, planar, n-link ("RR...R") arm, derived from Lagrangian
    mechanics for general n (not separately hand-derived per link count).

    q_i is joint i's own relative rotation; link i's absolute world
    orientation is phi_i = q_1 + ... + q_i. Link i is a uniform rigid rod of
    length lengths[i] and mass masses[i], hinged to link i-1 (link 0 hinged
    to a fixed base at a static frame's origin). Gravity acts along world -y
    with magnitude `gravity`.

    Args:
      q, qdot, tau: length-n lists (joint angles, joint angular velocities,
        applied joint efforts), n == 2 or 3.
      gravity: scalar g >= 0.
      masses, lengths: length-n lists, each entry > 0.

    Returns:
      qddot: length-n list, the resulting joint angular accelerations.

    Invariants the implementation must satisfy (not enforced by this
    signature, but part of the contract callers may rely on):
      - M(q) is symmetric and positive-definite for any physically valid
        input (masses/lengths > 0), so a solution always exists.
      - Called with tau = [0]*n and any q where G(q) != 0, the returned
        qddot must be nonzero (this is the spec's own stated correctness
        check for the "arm swings freely under gravity" milestone).
      - Purely a function of (q, qdot, tau, gravity, masses, lengths) — no
        explicit time dependence, no internal state/memory between calls.
    """
```

- `n = len(q)` is inferred from the input lists; the caller (`arm_sim_node.py`) is responsible for
  always passing lists of the correct, consistent length (`state.links`).
- **Not part of the contract, left to the owner**: the exact linear-algebra method used to
  solve/invert `M(q)` (Gauss-Jordan with partial pivoting or anything else); whether `M`/`C`/`G`
  are computed as separate internal helper functions or inline; any internal caching. A caller
  outside `arm_dynamics.py` must never depend on any of that.
- **Recommended, not required**: the owner may find it useful to additionally expose a standalone
  `gravity_load(q, gravity, masses, lengths) -> list[float]` (i.e. `G(q)` alone) for the manual
  sanity-check the spec itself describes ("compare the steady-state effort you're seeing against
  `G(q)` computed independently") and for a future PID-convergence test. This pass's wiring does
  not need it and does not call it — noted here only so a later pass doesn't have to guess whether
  it was considered.

### Live loop adapting `arm_dynamics` to `integrators`

`integrators.AccelFn` is `(t, q, qdot) -> qddot`, with no `tau`/`gravity`/`masses`/`lengths`
parameters — those are closed over per tick from the *current* `_ArmSimState`, and `t` is ignored
(the manipulator equation has no explicit time dependence, unlike the checkpoint's
`f(t)`-forcing case):

```python
def _accel_fn(t, q, qdot):
    tau = [0.0] * state.links  # PID disabled this pass; always zero effort
    return arm_dynamics.forward_dynamics(
        q, qdot, tau, state.gravity, state.masses, state.lengths,
    )
q_next, qdot_next = integrators.METHODS[state.integrator_method](
    _accel_fn, state.sim_time, state.q, state.qdot, state.integrator_timestep,
)
```

Rebuilt (or at least re-reading `state.*`) every tick — not built once at startup — so a
mid-run `/arm_sim/set_params` or `/arm_sim/set_integrator` call is reflected on the *next* tick.

### Data flow, end to end

```
main.py: state = arm_sim_node.register(registry, links)
main.py: asyncio.create_task(arm_sim_node.physics_loop(state))
main.py: asyncio.create_task(arm_sim_node.publish_loop(registry, state))

physics_loop (every state.integrator_timestep, while not state.paused):
    reads state.{gravity,masses,lengths,integrator_method,integrator_timestep}
    tau = zero vector
    (state.q, state.qdot) = METHODS[method](accel_fn, state.sim_time, state.q, state.qdot, dt)
        accel_fn -> arm_dynamics.forward_dynamics(q, qdot, tau, gravity, masses, lengths)
    state.sim_time += dt

publish_loop (every 1/60 s, regardless of paused):
    registry.publish("/joint_states", {
        header: {stamp: <from state.sim_time>, frame_id: ""},
        name: ["joint1", "joint2", ...],       # one per link
        position: list(state.q),
        velocity: list(state.qdot),
        effort: [0.0] * state.links,           # tau always zero this pass
    })
    -> Registry fans this out in-process to any connection currently
       subscribed to /joint_states (external TCP subscribers via gateway.py,
       transparently — no gating, per Registry.publish's own semantics).

/arm_sim/set_params, /arm_sim/set_integrator, /arm_sim/pause (unchanged,
already built): mutate the same `state` object physics_loop reads every tick.
```

## Implementation steps

1. Create `src/arm_dynamics.py` as a **stub**: module docstring explaining scope/ownership
   (mirroring `integrators.py`'s existing "STUB: implement this file yourself" framing), the
   `forward_dynamics` signature and full docstring from "Interfaces and data flow" above, body
   `raise NotImplementedError(...)`. Do not implement `M`/`C`/`G` or any solve step.
2. In `src/arm_sim_node.py`, add `import arm_dynamics` alongside the existing `import
   integrators`.
3. Extend `_ArmSimState.__init__` to also set `self.q = [0.0] * links`, `self.qdot = [0.0] *
   links`, `self.sim_time = 0.0`.
4. Add a small private helper, e.g. `_joint_names(links: int) -> list[str]`, returning
   `["joint1", "joint2"]`/`["joint1", "joint2", "joint3"]` — used by `publish_loop` (and reusable
   by a later `/joint_trajectory` pass without redefinition).
5. Add `async def physics_loop(state: _ArmSimState) -> None` per "Proposed architecture" above:
   loop forever; each iteration, `await asyncio.sleep(state.integrator_timestep)` (read *before*
   sleeping, since that's the interval being paced), then if `not state.paused`, perform one
   integration step as described and advance `state.sim_time`. Wrap the per-tick dynamics-call +
   integration step in `try/except Exception` that logs (via `gateway.log`) and continues the loop
   rather than letting one bad/unimplemented tick kill the task silently (see "Edge cases" below).
6. Add `async def publish_loop(registry: Registry, state: _ArmSimState) -> None`: loop forever,
   `await asyncio.sleep(1/60)`, build and `registry.publish("/joint_states", ...)` the message
   described above, converting `state.sim_time` to `{"sec": ..., "nanosec": ...}` for
   `header.stamp`.
7. Change `register(registry, links=2)` to build `state = _ArmSimState(links)` before registering
   the four existing handlers (which must still close over/use this same `state` instance, not a
   new one), and `return state` at the end.
8. In `src/main.py`, after `state = arm_sim_node.register(registry, links)` and
   `await gateway.start()`, add:
   ```python
   physics_task = asyncio.create_task(arm_sim_node.physics_loop(state))
   publish_task = asyncio.create_task(arm_sim_node.publish_loop(registry, state))
   ```
   After `await stop_event.wait()` and alongside `await gateway.stop()`, cancel and await both
   tasks (e.g. `for t in (physics_task, publish_task): t.cancel()` then
   `await asyncio.gather(physics_task, publish_task, return_exceptions=True)`), so shutdown stays
   clean under `SIGTERM`/`SIGINT` (matching the already-verified clean-shutdown behavior in
   `TEST_RESULTS.md`).
9. Do not touch `registry.py`, `gateway.py`, `expr.py`, `integrators.py`, or the existing
   `_integration_step`/`set_params`/`set_integrator`/`pause` handler bodies.

## Edge cases and failure modes

- **`arm_dynamics.forward_dynamics` raising `NotImplementedError`** (true until the owner fills it
  in): `physics_loop`'s per-tick `try/except` must catch this (and any other exception from a
  not-yet-correct implementation) and `log()` it rather than letting the task die silently —
  otherwise the whole live-arm feature appears to "work" (gateway stays up, all services still
  respond) while `/joint_states` quietly freezes at the reset pose forever, which is a confusing
  failure mode to debug. Logging on every tick would flood stderr, though — reasonable options: log
  once (a "physics loop failing, see below" style message the first time only) and keep retrying
  each subsequent tick without further per-tick log spam, or log at a throttled rate. Left to the
  Implement phase to choose a specific throttling approach; either is acceptable as long as (a) the
  loop keeps running afterward and (b) the gateway/other services stay fully responsive throughout.
- **Singular or ill-conditioned `M(q)`**: shouldn't occur for a physically valid arm per the
  spec's own invariant (`M(q)` PD whenever masses/lengths are positive), and `set_params` already
  rejects non-positive masses/lengths independently — so this should be structurally unreachable
  given the existing validation. The `try/except` in `physics_loop` (previous bullet) is
  sufficient defense-in-depth; no additional guard needed in the wiring itself.
- **Paused simulation**: `physics_loop` must skip the integration step and *not* advance
  `state.sim_time` while `state.paused` is true, but must keep sleeping/looping (so it resumes
  promptly once unpaused) — not exit or block. `publish_loop` is unaffected by `paused` and keeps
  publishing the frozen `state.q`/`state.qdot` snapshot every tick, per spec
  ("`/joint_states` keeps publishing a frozen snapshot" while paused).
- **Mid-run `/arm_sim/set_params`/`/arm_sim/set_integrator` changes**: must take effect on the
  *next* physics tick, not require a restart — guaranteed by reading `state.*` fresh inside the
  loop body each iteration (never captured in a local/closure variable outside the loop).
- **Changing `integrator_timestep` mid-run**: `physics_loop` paces itself by re-reading
  `state.integrator_timestep` before each `asyncio.sleep(...)` call, so a change takes effect
  starting from the *next* sleep — the iteration currently mid-sleep still completes with the
  previously-read interval. This is a reasonable, simple behavior; flagged as an assumption below
  since the spec doesn't specify sub-tick timestep-change semantics.
- **Task lifecycle / clean shutdown**: both loops are infinite `while True` coroutines; if left
  unmanaged, `asyncio.run()`'s teardown can emit "Task was destroyed but it is pending" warnings,
  or in the worst case interfere with a clean, fast process exit under `SIGTERM` — which
  `TEST_RESULTS.md` explicitly already verified works cleanly for the gateway alone. Both new
  tasks must be explicitly cancelled and awaited (catching `asyncio.CancelledError`) during
  shutdown, per implementation step 8, to preserve that property.
- **No locking needed** for `_ArmSimState` field access across `physics_loop`, `publish_loop`, and
  the service handlers: all run as coroutines on one asyncio event loop (confirmed via
  `registry.py`'s own "single event loop, no threads, no locking" design note), and no coroutine
  here `await`s in the middle of reading-then-writing a given field, so there's no interleaving
  hazard to guard against.
- **CPU/self-throttling at very small `integrator_timestep`**: an aggressively small timestep
  (e.g. via `/arm_sim/set_integrator`) makes `physics_loop` sleep-and-tick very frequently,
  increasing CPU usage — acceptable for this course project's scope; not engineered around here
  (matches the spec's own framing that a too-small/too-large `dt` having visible consequences is
  the point of the assignment, not a bug to hide).
- **A too-large `dt` under `tau = 0`**: the spec's explicit "expected to go unstable" callout is
  scoped to an *enabled* PID loop; with PID disabled and `tau` always zero this pass, there's no
  active instability driver, though pure numerical drift from a large `dt` under gravity alone
  could still in principle be visible — not something this pass needs to guard against or test for
  (no acceptance criterion here depends on numerical stability at extreme `dt`).
- **`links` consistency**: `state.q`/`state.qdot`/`tau` must always be exactly `state.links`
  long, matching `masses`/`lengths`/`gravity`'s already-established length invariant from
  `set_params`. Since `q`/`qdot` are only ever initialized (to `[0.0]*links`) and then
  overwritten by `integrators.METHODS[...]`'s return value (which preserves list length given a
  correct `accel_fn`), no additional length-checking is needed in the wiring itself.

## Open questions and assumptions

- **Reset pose = all-zero angles** (`q = qdot = [0.0]*links`). The spec explicitly leaves this
  open ("anything about the root joint beyond it being a fixed base... is entirely yours"); `[0]`
  is the simplest spec-compatible choice and matches this plan's stated invariant that
  `forward_dynamics([0]*n, [0]*n, [0]*n, gravity, masses, lengths)` should be nonzero for the
  default arm (visible motion from the moment the runtime starts, satisfying the spec's own
  stated milestone) unless that particular pose is a coincidental equilibrium.
- **`/joint_states.header.stamp` should carry simulation time (`state.sim_time`), not wall time.**
  The spec allows either, but a later, out-of-scope-for-this-pass requirement
  (`/ik_action/result`'s `success_hold` dwell timer) explicitly says to "read simulation time from
  the same clock `/joint_states`'s `header.stamp` already carries" — choosing simulation time now
  avoids a breaking change to `/joint_states`'s meaning later. Flagged as a deliberate
  forward-compatibility choice, not something this pass's own acceptance criteria require by
  themselves.
- **Physics loop pacing**: this plan chooses the simplest "sleep for the current timestep, then
  take exactly one integration step" loop shape over a fixed-quantum wall-clock accumulator (which
  would decouple wall-clock physics rate from `integrator_timestep`, e.g. always ticking every 1ms
  and running zero-or-more `dt`-sized integration steps per wall-clock quantum based on elapsed
  real time). The simpler shape means physics wall-clock rate and `integrator_timestep` are
  coupled 1:1; the spec's only explicit decoupling requirement is between `/joint_states`'
  publish rate and the physics timestep, which this design satisfies (`publish_loop` is entirely
  independent). If a future pass finds the coupled pacing behaves poorly (e.g. under a very large
  `dt`, physics visibly "jumps" once every several wall-clock seconds), revisiting this to an
  accumulator loop is a plausible extension, not a spec violation of the current design.
- **Joint naming**: `["joint1", "joint2"]` / `["joint1", "joint2", "joint3"]`, 1-indexed, matching
  the spec's own `/joint_trajectory` example (`["joint1", "joint2"]`) verbatim. Not spec-mandated
  as the exact literal string, but the closest thing to an explicit convention given.
  `/joint_trajectory` (out of scope this pass) will need to match whatever naming `/joint_states`
  uses, so recording the choice here for consistency later.
- **`tau` is always `[0.0]*links`, sourced nowhere else, this pass** — explicit scope boundary;
  the closure in "Interfaces and data flow" is written so that swapping in a real `tau` (from a
  future `pid.py`, when enabled) later is a one-line change at that single point.
- **Whether `arm_dynamics.py` should expose `M`/`C`/`G` separately, not just `forward_dynamics`**:
  left as a recommendation, not a requirement (see "Interfaces and data flow" above) — this pass's
  wiring only calls `forward_dynamics`, so nothing here depends on the answer. Flagged so a later
  pass planning PID convergence tests (which want an independent `G(q)` to sanity-check
  steady-state effort against) doesn't have to rediscover this was considered and deliberately
  left open.
- **Exception-throttling strategy for a persistently-failing `physics_loop` tick** (see "Edge
  cases"): left as an Implement-phase choice between "log once, then run silently" and "log at a
  throttled rate" — either satisfies this plan's actual requirement (the loop and the rest of the
  runtime must keep running), so the exact throttling mechanism isn't specified further here.

## Verification strategy

- **Primary spec correctness check** (once `arm_dynamics.py` is hand-implemented by the owner):
  start the runtime, do not enable PID, observe `/joint_states` over a few seconds of wall time —
  `position`/`velocity` must change from the reset-pose values (unless the default parameters
  happen to put the arm at a gravity equilibrium, which the all-zero default reset pose is not
  expected to be for a generic multi-link arm under gravity). This is exactly the spec's own
  stated milestone, verbatim.
- **`/joint_states` shape/rate**: subscribe a test client to `/joint_states`; confirm `name` has
  exactly `links` entries and matches across `position`/`velocity`/`effort` (also each length
  `links`); confirm messages arrive at roughly the reference 60 Hz cadence, independent of
  whatever `integrator_timestep` is currently set to (e.g. set an unusually large or small
  timestep via `/arm_sim/set_integrator` and confirm the publish cadence doesn't change).
- **Mid-run `set_params` takes effect without restart**: start the runtime, let it run briefly,
  call `/arm_sim/set_params` with a different `gravity`/`masses`/`lengths`, and confirm subsequent
  ticks' motion is consistent with the *new* parameters (e.g. setting `gravity: 0` should make
  the arm's velocity stop changing shortly after the call, rather than continuing to accelerate
  under the old gravity value).
- **Mid-run `set_integrator` takes effect without restart**: similarly, call
  `/arm_sim/set_integrator` with a different `method`/`timestep` mid-run and confirm subsequent
  behavior reflects the change (harder to verify by eye than `set_params`; a more precise version
  of this check can monkeypatch/stub `arm_dynamics.forward_dynamics` with a known, simple
  acceleration function and assert on the exact resulting `(q, qdot)` sequence against
  hand-computed expected values for the newly-selected method).
- **`pause` semantics**: call `/arm_sim/pause` with `data: true`; confirm `/joint_states` keeps
  publishing (doesn't stop) but with an unchanging `position`/`velocity` snapshot; call it again
  with `data: false` and confirm motion resumes from where it left off (not reset).
- **Runtime stays responsive even if `arm_dynamics.py` is unimplemented or buggy**: with a stub
  `forward_dynamics` that raises, confirm the gateway still answers all existing services normally
  (`/arm_sim/integration_step`, `/arm_sim/set_params`, etc.) and that `/joint_states` still
  publishes (frozen at the reset pose, since no tick ever successfully advances it) — i.e. the
  live-loop addition must not regress the already-passing checkpoint behavior or crash the
  process.
- **Clean shutdown regression check**: re-run (or extend) the existing real-subprocess `SIGTERM`
  test (`tests/test_live_process_e2e.py`'s pattern) with the two new background tasks running,
  confirming the process still exits promptly with no traceback and the port is released — this
  is the specific risk implementation step 8 exists to prevent.
- **Unit-level wiring tests independent of the owner's real dynamics**: since
  `arm_dynamics.forward_dynamics`'s internals are out of scope and hand-written separately, wiring
  correctness (does `physics_loop` call `integrators.METHODS[...]` with the right arguments; does
  it read `state.*` fresh each tick rather than a cached copy; does it skip stepping while paused;
  does `publish_loop` build a correctly-shaped message) should be tested against a
  monkeypatched/stub `forward_dynamics` (e.g. a fixed constant, or a simple closed-form function)
  so these tests don't depend on the owner's dynamics being complete or correct yet — mirroring
  how `TEST_RESULTS.md` already scoped `integrators.py`'s own algorithmic correctness out of the
  checkpoint's wiring tests.
