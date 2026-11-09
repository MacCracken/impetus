# 0008 — API changes in the Cyrius 2.0 port

**Status**: Accepted
**Date**: 2026-11-09

## Context

The roadmap's "2.0 — Breaking Cleanup" lists changes that need a public-API or snapshot-format break,
so Rust 1.x could not make them. There will be no Rust 2.0. The Cyrius port is a new major version
anyway, and kimiya set the precedent (its ADR 0002): breaking API changes land in the Cyrius API only,
and each keeps a defined comparison against the oracle, so no function leaves parity entirely.

Some roadmap items are API shape changes. Others are new physics with no oracle to compare against:
3D islands, 3D revolute/prismatic/motors, consistent 3D joint frames, XPBD parity. Only the first kind
belongs in the port (ADR 0007: port first, then add).

## Decision

impetus 2.0.0 (Cyrius) differs from Rust 1.4.0 in exactly these ways. The harness maps each oracle
row as shown, and reports each mapped row by this ADR's number.

| # | Rust 1.4.0 | Cyrius 2.0.0 | Golden comparison |
|---|---|---|---|
| 1 | `add_collider` / `add_joint` store an orphan for a missing body | return `Result`: `Err(IM_ERR_BODY_NOT_FOUND)` | oracle orphan ↔ `Err`; otherwise class A on handles |
| 2 | deprecated `restore` beside `try_restore` | one fallible `restore` | the snapshot is Cyrius-only (port plan M6) |
| 3 | incomplete bitcode snapshot (no force fields, manifolds, sleep state or arena metadata) | complete, versioned binary format | round trip and step-equivalence: a restored world steps to the same bits as the original |
| 4 | 3D `BodyState` keeps only rotation about Z | full quaternion and angular-velocity vector | the oracle's hidden `oracle_dump()` supplies the full state |
| 5 | 3D Revolute / Prismatic / motors accepted, with no effect | `add_joint` returns `Err(IM_ERR_UNSUPPORTED)` in the 3D profile | oracle accept-and-ignore ↔ `Err` |
| 6 | `SolverKind::Xpbd` (2D-only, experimental) | refused by `config_validate` with `Err(IM_ERR_UNSUPPORTED)` | oracle Xpbd rows ↔ `Err` |
| 7 | results bit-identical only on the same platform | impetus-owned arithmetic bit-identical across targets | measured on x86_64 ELF, x86_64 PE and aarch64 |

Not in 2.0 (new physics, started after 2.0 with reference tests only): 3D islands and island sleep,
3D revolute/prismatic/motors, consistent 3D joint frames, XPBD parity.

## Consequences

- **Positive**
  - The 2.0 breaking cleanup happens once, at the language change, rather than in a separate Rust
    major release that nobody would consume.
  - Silent failure modes become explicit errors: orphan colliders and joints, ignored 3D joints,
    and an experimental solver selected by accident.
  - Snapshots restore bit-identically, which the two `#[ignore]`d Rust regression tests could not
    show.
- **Negative**
  - A consumer moving from the Rust crate must handle the new `Err` returns (#1, #5, #6) and
    re-save snapshots (#2, #3).
  - Rows #1, #5 and #6 are compared through a mapping, not by value, so the harness needs a mapping
    table (as kimiya's).
- **Neutral**
  - XPBD's absence is demand-gated; it can return as a Cyrius 2.x feature with reference tests.

## Alternatives considered

- **Keep the 1.4 API exactly.** Rejected: it would port known silent failures into a new major
  version, and the snapshot format has to change anyway (bitcode has no Cyrius implementation).
- **Also land the new 3D physics in 2.0.** Rejected: with no oracle, a translation error and a new
  feature look the same. They follow 2.0.
- **A Rust 2.0 first.** Rejected, as kimiya did: no consumer would use it, and it would delay the
  port for a release with a short life.
