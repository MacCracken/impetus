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

## Amendment — #4 implemented (2026-11-10)

How #4 lands, now that it is built (after M5, before M6):
- **One name, two shapes.** `ImBodyState` is each profile's own type, as the world is. The 2D profile keeps Rust's
  (`src/b2_types.cyr`: `rotation: f64`, `angular_velocity: f64`). The 3D profile's (`src/b3_types.cyr`) has
  `rotation: ImQuat`, a unit quaternion (x, y, z, w) as glam's `DQuat`, and `angular_velocity: ImVec3`. The other
  fields are shared. Rust's two features already give one `BodyState` per build, so a consumer that compiles against
  one bundle sees one type.
- **Get and set.** `get_body_state` copies the body's quaternion and angular velocity. `set_body_state` stores them as
  given, then wakes the body and refreshes its world inverse inertia, as Rust does. It does not normalize the
  quaternion: normalizing would change a unit quaternion's last bit. Rust 1.4.0's call with angle θ and spin ω is
  the 2.0 call with `ImQuat_from_rotation_z(θ)` and (0, 0, ω), bit for bit.
- **`BodyDesc` is unchanged.** It still builds a body turned about z, as in Rust. A body gets any other orientation
  from `set_body_state` or from the dynamics.
- **JSON.** The 3D document is what serde_json writes for the 2.0 struct: `"rotation":[x,y,z,w]` (glam's `DQuat`
  order) and `"angular_velocity":[x,y,z]` (`src/b3_json.cyr`). Rust 1.4.0's 3D document, with a number in each, is
  refused with `IM_ERR_DESERIALIZE`. The 2D document is unchanged. M6's `BodySnapshot` follows the same shapes.
- **Golden comparison**, as the table says. `b3_state.tsv`'s gets carry the body's full state from the oracle's dump.
  Its sets carry the state Rust stores for its angle and spin, which the generator checks against the dump after
  each call. Both compare by value, with no mapping to count. Rust's 3D tests that read the angle use the
  quaternion's whole angle, which is stricter than the angle about z.
