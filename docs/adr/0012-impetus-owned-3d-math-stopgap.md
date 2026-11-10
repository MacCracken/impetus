# 0012 — Impetus-owned 3D math and GJK/EPA until hisab's allocation can be scoped

**Status**: Accepted
**Date**: 2026-11-09

## Context

ADR 0007 makes the port hisab-first. Its M0 amendment keeps hisab off the step path and the queries until the
allocation of its calls can be scoped, because every hisab call that returns a value leaks for the life of the
process (`port-parity.md` §4.1, reproduced 2026-11-09):

| Call | Global heap per call, never returned |
|---|---|
| `hvec3_add`, `hvec3_cross`, `hquat_rotate_vec3` | 24 B |
| `m3_mul_vec3` | 48 B |
| `gjk_epa_3d`, two overlapping spheres | 105,920 B |

The scoping is S1, a stdlib allocation scope (cyrius issue `2026-11-09-no-scoped-allocator-for-alloc.md`, open),
or H9, allocator-taking hisab calls, which had not been filed. The amendment left M4 to decide between waiting for
S1 or H9 and porting impetus's own GJK/EPA as a stopgap.

M4's 3D step path needs three things:
- **Vectors, quaternions and 3×3 matrices.** Rust 1.4.0 uses glam 0.29.3's `DVec3`, `DQuat` and `DMat3`, which
  Rust hisab 1.4.0 re-exports. That covers rotation, world inverse inertia R·I⁻¹·Rᵀ, the implicit gyroscopic step,
  and the parallel-axis shift.
- **Impetus's own GJK/EPA**, for hulls and general shape pairs.
- **Closed forms** for spheres, capsules, boxes and segments.

The maintainer chose the stopgap (2026-11-09).

## Decision

- **3D value math is impetus-owned for now.** `src/math3.cyr` transcribes glam 0.29.3's f64 formulas as value
  structs: `ImVec3` operations, `ImQuat` and `ImMat3`. The operation order is glam's, so impetus's 3D arithmetic
  matches Rust bit for bit.
  - The formulas are read from the registry source of the exact glam version the oracle links, with the
    `std`/`serde` features only, which are scalar f64 code.
  - The transcendentals go through crmath (ADR 0011): `from_rotation_z`'s `sin_cos`, and every `sqrt` and `recip`
    is IEEE.
- **GJK/EPA is impetus's own**: a port of `backend_3d/narrowphase.rs`'s `gjk_epa`, exactly as Rust wrote it.
- **Parity classes.** These paths are compared as the 2D backend is: classes A–C, gated at 0 ulp against the
  corrected oracle, not class D. ADR 0007's class-D list does not apply to them while this ADR stands.
- **Storage.** No allocation on the step path: the math is by value, and GJK/EPA's simplex and polytope live in
  world-owned scratch reused every step (port plan §7.5).
- **The switch.** When S1 lands, or H9 in hisab, each impetus-owned path is replaced by its hisab call in its own
  change. Its rows move to class D, with the bound set by `port-parity.md`'s rule before the first comparison, and
  the reference tests carry the physics. This ADR is then superseded.
- **Upstream.** H9 is filed in hisab with probe 1's numbers (2026-11-09), alongside S1.

## Consequences

- **Positive**
  - M4 and M5 proceed now, with bit parity against Rust: the same 0-ulp gating, corrected oracle and
    transcription check as the 2D backend.
  - The 3D step stays flat on the heap.
  - The transcription documents exactly which glam operations impetus needs. That is the shopping list for H1
    (allocation-free vector math) and H9.
- **Negative**
  - Code that hisab also has is owned twice until the switch, against ADR 0007's direction: about 300 lines of
    vector, quaternion and matrix math, and the GJK/EPA.
  - The switch to hisab will move some rows from exact to class D, because hisab's quaternion rotation, matrix
    inverse and GJK are different algorithms (ADR 0007).
- **Neutral**
  - Rust's own inertia formulas (`types.rs`) are transcribed rather than taken from hisab's `inertia_box` and
    `inertia_sphere`, since the oracle computes its own. hisab is still allowed off the step path, but using it
    there would make the mass rows class D for no gain.

## Alternatives considered

- **Wait for S1 or H9.** Rejected by the maintainer: M4's narrowphase and M5 would wait indefinitely on another
  repository. Only the off-path pieces could proceed.
- **Call hisab on the step path anyway.** Rejected: about 10 MB a step leaked at 100 contacts, which breaks the
  flat-heap rule (port plan §7.5).
- **Transcribe hisab's algorithms instead of glam's.** Rejected: it would duplicate hisab just the same, and give
  class-D rows with no oracle to agree with.
