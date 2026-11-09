# Architecture Decision Records

Decisions about impetus — what we chose, the context, and the consequences we accept. Use these when a future reader would reasonably ask *"why did we do it this way?"*

## Conventions

- **Filename**: `NNNN-kebab-case-title.md`, zero-padded to four digits. Never renumber.
- **One decision per ADR.** If a decision supersedes a prior one, add a new ADR and set the old one's status to `Superseded by NNNN`.
- **Status lifecycle**: `Proposed` → `Accepted` → (optionally) `Superseded` or `Deprecated`.
- Use [`template.md`](template.md) as the starting point.

## ADR vs. architecture note vs. guide

| Kind | Lives in | Answers |
|---|---|---|
| ADR | `docs/adr/` | *Why did we choose X over Y?* |
| Architecture note | `docs/architecture/` | *What non-obvious constraint is true about the code?* |
| Guide | `docs/guides/` | *How do I do X?* |

## Index

| ADR | Title | Status |
|---|---|---|
| [0001](0001-oneshot-manifold-clipping.md) | One-shot manifold generation via Sutherland-Hodgman clipping | Accepted (point accumulation superseded by 0006) |
| [0002](0002-solver-kind-enum.md) | SolverKind enum for configurable solver algorithm | Accepted |
| [0003](0003-gyroscopic-torque.md) | Gyroscopic torque in 3D velocity integration | Accepted |
| [0004](0004-xpbd-solver.md) | XPBD solver for 2D backend | Accepted (not carried into 2.0, see 0008) |
| [0005](0005-velocity-level-joints.md) | Velocity-level joint constraints | Accepted |
| [0006](0006-manifold-rebuild-per-frame.md) | Rebuild contact manifolds every frame | Accepted |
| [0007](0007-cyrius-port-hisab-first.md) | Port to Cyrius on hisab, with Rust 1.4.0 frozen as the oracle | Accepted |
| [0008](0008-port-api-divergences.md) | API changes in the Cyrius 2.0 port | Accepted |

0001–0006 were written for the Rust crate (three-digit numbers until the port).
