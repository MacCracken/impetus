# 004 — XPBD solver for 2D backend

## Status: Accepted

## Context

The existing sequential impulse (SI) solver works well for general cases but has known stability issues with stiff constraint stacks and ragdolls. XPBD (Extended Position-Based Dynamics) solves these cases better through position-level constraint solving with compliance parameters.

The `SolverKind` enum was scaffolded in ADR-002. This ADR covers the actual solver implementation.

## Decision

Implement XPBD as an alternative solver path in the 2D backend (`step_xpbd`), selected via `WorldConfig::solver = SolverKind::Xpbd`.

### Algorithm

1. **Store** previous positions (`prev_position`, `prev_rotation`) for velocity derivation
2. **Integrate** velocities (gravity + forces + damping) — same as SI
3. **Predict** positions: `x += v * dt`
4. **Broadphase + narrowphase** at predicted positions
5. **XPBD constraint loop** (N iterations):
   - Contact constraints: push bodies apart along normal with Lagrange multiplier accumulation
   - Joint constraints: enforce positional relationships (Fixed, Revolute, Distance, Spring, Rope)
   - Compliance `α̃ = 1/(ω²·dt²)` controls constraint stiffness
6. **Derive** velocities: `v = (x - x_prev) / dt`
7. **Velocity-level** friction and restitution
8. Islands, sleep, events

### Key formulas

- Constraint update: `Δλ = (C - α̃·λ) / (w_a + w_b + α̃)`
- Position correction: `Δx = Δλ · w · n`
- Spring compliance: `α̃ = 1/(k·dt²)` per-joint

### Joint support

XPBD mode supports: Fixed, Revolute, Distance, Spring, Rope joints at position level.
Prismatic, Wheel, and Mouse joints fall through (not position-level constrained in XPBD mode).

## Consequences

- Users can choose XPBD for better stacking stability at the cost of slightly different physics feel
- SI solver remains the default and is unaffected
- XPBD reuses broadphase, narrowphase, manifold, island, and event infrastructure from SI
- `prev_position`/`prev_rotation` fields added to `RigidBody2d` (8 bytes per body)
- 3D backend does not yet have XPBD — selecting it for 3D falls through to SI
