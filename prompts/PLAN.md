# Planning Agent

Your job is to understand the task and produce a concrete implementation plan.

**Do not implement the solution in this session.**

## Repository conventions

- Reusable agent instructions live in `prompts/`.
- Persistent outputs and handoff notes live in `agent-notes/`.
- Do not modify files in `prompts/`.
- Write this phase's output to `agent-notes/PLAN.md`.

## Project context

This repository implements Project 2 (autorob.org): "Pendularm" — a planar n-link (2 or 3)
rotational robot arm physics simulation and PID servo controller, reusing the same rosbridge
TCP/JSON gateway protocol as Project 1 (a sibling repo, `~/A-Star-Path-Planning`), unchanged, on
`127.0.0.1:9095`.

- Specification: `spec/PROJECT2_PENDULARM.md`, `spec/ROSBRIDGE_PROTOCOL.md`.
- Target architecture — status markers below are a snapshot, not guaranteed current; confirm
  against `agent-notes/PLAN.md`/`IMPLEMENTATION.md` and the repository itself before planning
  against it:
  - `src/registry.py`, `src/gateway.py` — **built**, ported unchanged from Project 1
    (`~/A-Star-Path-Planning/src/{registry,gateway}.py}`), since the spec explicitly says to reuse
    this transport "unchanged." Same generic topic/service registry + asyncio TCP/JSON gateway;
    domain-agnostic, no Pendularm-specific logic belongs here.
  - `src/arm_dynamics.py` — **not yet built**. Will hold the Lagrangian mass matrix `M(q)`,
    Coriolis/centrifugal `C(q,qdot)`, gravity load `G(q)`, and forward dynamics
    (`qddot = M^-1(tau - C*qdot - G)`) for a general n-link arm.
  - `src/integrators.py` — **built and verified** (hand-written by the project owner). The four
    selectable numerical integrators (`euler`, `midpoint`, `verlet`, `rk4`), each advancing a
    second-order system one timestep, evaluating acceleration at the correct fractional time per
    sub-stage, plus a `METHODS` name→function dict.
  - `src/pid.py` — **not yet built**. Will hold the per-joint PID control law (position/velocity
    error, clamped integral, anti-windup).
  - `src/kinematics.py` — **not yet built**. Will hold forward kinematics (`x,y,phi` from `q`) and
    closed-form inverse kinematics (Law of Cosines + orientation decoupling) for 2- and 3-link arms.
  - `src/arm_sim_node.py` — **partially built**. Currently registers the standalone
    `/arm_sim/integration_step` checkpoint service (done, tested), plus minimal state-storage-only
    stubs for `/arm_sim/set_params`, `/arm_sim/set_integrator`, `/arm_sim/pause` (done, tested —
    these needed no dynamics/PID, just parameter/mode storage). Still missing: wiring
    `arm_dynamics` + `integrators` + `pid` into an actual live sim loop, `/arm_sim/reset` (needs
    PID controller state to reset), `/joint_trajectory` subscription, `/joint_states` publication.
  - `src/expr.py` — **built and verified**. The math-expression parser/evaluator
    `/arm_sim/integration_step`'s `function` field needs (arithmetic, `^`, unary minus, parens,
    `sin`/`cos`/`tan`/`exp`/`sqrt`/`ln`/`abs`), including domain-safe handling so evaluation-time
    domain errors (e.g. negative base to a fractional power, `0^negative`, overflow, trig of
    infinity) become NaN/inf rather than raising or hanging.
  - `src/ik_node.py`, `src/ik_action_node.py`, `src/ik_trial_node.py` — **not yet built**. Will
    hold `/ik/*`, `/ik_action/*`, `/ik_trial/*` respectively, built on `kinematics.py`.
  - `src/main.py` — **built**. Wires `Registry` + `Gateway` + `arm_sim_node`; `make run`'s entry
    point; reads/validates `ARM_SIM_LINKS` (`"2"`/`"3"`, default `2`) — read but not yet consumed
    by anything beyond `arm_sim_node`'s link-count-aware state storage, since the live arm doesn't
    exist yet.
  - `Makefile` — **built**. `build`/`run`/`clean` (required by the grader; no `make map` this
    project) plus `test` (project convenience, not graded).
  - `tests/` — **built for what exists**: `unittest`-based, a reusable raw-socket TCP/JSON test
    client (`client_helper.py`, ported from Project 1), integration tests against both an
    in-process threaded gateway and a genuine `make run` OS subprocess. 75 tests passing as of the
    last Test phase — see `agent-notes/TEST_RESULTS.md`.
- **`src/arm_dynamics.py`, `src/integrators.py`, `src/pid.py`, and `src/kinematics.py` are
  hand-implemented by the project owner as the graded exercise, not by an agent** — the same
  division of labor as Project 1's `heap.py`/`astar.py`. Everything else (transport, node wiring,
  the `/arm_sim/integration_step` expression parser, tests) is fair game for this workflow to
  build. Treat those four owner-owned modules' public functions as a fixed contract (signatures
  to be agreed during planning) rather than something to design the internals of.

## Before you begin

1. Read the project specification (`spec/PROJECT2_PENDULARM.md`, `spec/ROSBRIDGE_PROTOCOL.md`) and
   any other relevant documentation.
2. Inspect the repository structure and relevant source files.
3. Understand the existing implementation before proposing changes.

## Goals

1. Identify the actual problem to solve.
2. Extract the required behavior, interfaces, constraints, invariants, and acceptance criteria.
3. Identify dependencies between components.
4. Identify important edge cases and failure modes.
5. Avoid the **X/Y problem**:
   - distinguish the user's actual goal from a proposed implementation
   - do not assume a suggested approach is required unless the specification requires it
6. Identify uncertainties, missing information, and risky assumptions.
7. Propose the simplest reasonable architecture that satisfies the requirements.
8. Break the work into small, ordered implementation steps.
9. Identify how the major requirements can later be independently verified.

## Output

Write the final plan to:

`agent-notes/PLAN.md`

The plan should contain:

- **Goal**
- **Relevant requirements**
- **Repository observations**
- **Proposed architecture**
- **Interfaces and data flow**
- **Implementation steps**
- **Edge cases and failure modes**
- **Open questions and assumptions**
- **Verification strategy**

## Rules

- Do not modify implementation code.
- Do not begin implementing while planning.
- Prefer evidence from the specification and repository over assumptions.
- Do not invent requirements.
- Keep scope limited to what the specification requires.
- Make the plan specific enough that a fresh implementation agent can execute it without relying on this conversation history.
- Do not plan to implement or rewrite the algorithm bodies of `src/arm_dynamics.py`,
  `src/integrators.py`, `src/pid.py`, or `src/kinematics.py` — see "Project context" above.
