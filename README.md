# Project 2 — Pendularm

Python implementation of a planar n-link (2 or 3) rotational robot arm physics simulation and PID
servo controller, reusing Project 1's rosbridge TCP/JSON gateway protocol unchanged on
`127.0.0.1:9095`. Full spec: `spec/PROJECT2_PENDULARM.md`, `spec/ROSBRIDGE_PROTOCOL.md`.

## Status

Just scaffolding so far: the unmodified starter skeleton (`Makefile`, `src/main.py` placeholder)
plus the `prompts/`/`agent-notes/` workflow described below. No dynamics, integrators, PID, IK, or
gateway code has been written yet.

## Layout

- `prompts/` — reusable Plan/Implement/Audit/Test agent instructions for working on this repo in
  short, focused sessions. Not modified per-task. Each file's "Project context" section describes
  the target architecture (see below) — the intended layout to build toward, not yet as-built.
- `agent-notes/` — persistent handoff artifacts for the current task: `PLAN.md`,
  `IMPLEMENTATION.md`, `AUDIT.md`, `TEST_RESULTS.md`, written by their respective phase.

## Working on this repo (Plan → Implement → Audit → Test)

Each phase runs as its own short, focused session, reading `prompts/<PHASE>.md` for instructions
and `agent-notes/` for prior-phase context, rather than depending on conversation history:

1. **Plan** (`prompts/PLAN.md`) — explores the problem, writes `agent-notes/PLAN.md`. Does not implement.
2. **Implement** (`prompts/IMPLEMENT.md`) — reads `agent-notes/PLAN.md`, builds the change, writes `agent-notes/IMPLEMENTATION.md`.
3. **Audit** (`prompts/AUDIT.md`) — independently checks the implementation against the plan and spec, writes `agent-notes/AUDIT.md`. Does not fix anything.
4. **Test** (`prompts/TEST.md`) — independently verifies behavior against the spec, writes `agent-notes/TEST_RESULTS.md`. Does not modify production code.

## Target architecture (not yet built)

- `src/registry.py`, `src/gateway.py` — ported from Project 1 (`~/A-Star-Path-Planning`), reused
  "unchanged" per spec. Generic topic/service registry + asyncio TCP/JSON gateway.
- `src/arm_dynamics.py` — Lagrangian `M(q)`, `C(q,qdot)`, `G(q)`, forward dynamics.
- `src/integrators.py` — `euler`, `midpoint`, `verlet`, `rk4`.
- `src/pid.py` — per-joint PID control law.
- `src/kinematics.py` — forward/inverse kinematics (2- and 3-link, closed-form).
- `src/arm_sim_node.py` — live sim loop, `/arm_sim/*` services, `/joint_trajectory`,
  `/joint_states`, `/arm_sim/integration_step` (the checkpoint service).
- `src/expr.py` — math-expression parser/evaluator for `/arm_sim/integration_step`'s `function`.
- `src/ik_node.py`, `src/ik_action_node.py`, `src/ik_trial_node.py` — `/ik/*`, `/ik_action/*`,
  `/ik_trial/*`.
- `src/main.py` — wiring, `make run` entry point, reads `ARM_SIM_LINKS` (`"2"`/`"3"`, default `2`).

**Division of labor** (matching Project 1's `heap.py`/`astar.py` split): `src/arm_dynamics.py`,
`src/integrators.py`, `src/pid.py`, and `src/kinematics.py` are meant to be hand-implemented as
the graded exercise, not written by an agent. Everything else — transport, node wiring, the
expression parser, tests — is fair game for the Plan/Implement/Audit/Test workflow.

## Running (once built)

```
make build   # offline, noninteractive
make run     # foreground, listens on 127.0.0.1:9095, reads ARM_SIM_LINKS (default 2)
make clean   # removes generated artifacts
```

There is no `make map` target for this project (no occupancy-grid concept).
