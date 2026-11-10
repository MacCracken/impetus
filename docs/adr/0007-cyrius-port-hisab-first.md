# 0007 — Port to Cyrius on hisab, with Rust 1.4.0 frozen as the oracle

## Status: Accepted (2026-11-09)

## Context

AGNOS libraries are moving from Rust to Cyrius. tanmatra finished its port (2.0.0, 2026-11-08) and
kimiya is partway through. Both freeze a repaired Rust release as a parity oracle and compare the
port against it value by value. impetus 1.4.0 has just been through an audit-and-repair sweep, and
every fix is pinned by a regression test, so it can serve as that oracle.

impetus differs from both siblings in two ways:
- **It is a stateful simulation.** A 1-ulp difference in a stacked pile grows every frame, so
  long trajectories can't be compared bit for bit the way pure functions can.
- **Its Cyrius dependency, hisab 3.4.1, already provides much of what impetus wrote for itself.**
  This includes vector/quaternion/matrix types, ray tests, GJK/EPA/MPR, a BVH, island detection
  and inertia tensors. It computes some of them differently from glam and from impetus:
  - `hquat_rotate_vec3` rotates by q/|q|;
  - `m3_inverse_checked` decides singularity exactly;
  - hisab's GJK is a different, more robust algorithm.

The maintainer's direction (2026-11-09) is to use as much of hisab as possible. Where hisab or the
language falls short, the port should file the gap rather than work around it privately. The ports
are how the ecosystem finds where the compiler, ganita and hisab need work. Speed is measured, not
gated: Cyrius is young and hasn't had speed work yet.

## Decision

- **impetus 2.0.0 is a Cyrius library.** Rust 1.4.0 is frozen in `rust-old/` as the oracle and is
  never edited. A wrong oracle value is fixed in Rust 1.4.x and the corpus regenerated.
- **hisab-first.** The port calls hisab for everything hisab provides. It keeps impetus-owned
  code only where hisab has no counterpart:
  - the sequential-impulse solver and joints (hisab's audit D039 assigns them to impetus);
  - manifold generation (SAT and face clipping);
  - the 2D narrowphase;
  - the broadphase, unless a probe shows hisab's BVH is enough.

  Each missing piece is filed upstream with the probe or benchmark that shows it.
- **Parity in layers.** Function rows use classes A–D as tanmatra. Single-step rows load the
  oracle's full state at frame k, step once, and compare frame k + 1. Scenario reference tests
  check physical outcomes. Bit-level trajectories are kept only for scenes that are not chaotic.
- **hisab-backed paths are compared within class D**, and by the reference tests, not by bits. The
  harness lists each such path by name. That list is the record of this ADR's accepted divergences.
- **Cross-target determinism.** Impetus-owned arithmetic uses only the operations that are
  bit-identical across targets: + − × ÷, sqrt, the polyfilled sin/cos/exp/atan, and ganita's pure
  functions. Results then match across x86_64 and aarch64, which Rust 1.4 doesn't promise.

The scope, milestones, probes and upstream gap list are in
[`docs/development/impetus-port-plan.md`](../development/impetus-port-plan.md).

## Consequences

- **Positive**
  - impetus becomes a real hisab consumer, the first with a game-style hot loop. Its benchmarks
    show hisab, ganita and the compiler where that workload spends its time.
  - Less code is owned twice: ray tests, GJK/EPA, inertia and islands live in hisab.
  - The roadmap's 2.0 breaking cleanup (complete snapshots, full 3D state, fallible
    `add_collider` / `add_joint`) lands in the Cyrius API, as kimiya did, without a Rust 2.0.
- **Negative**
  - The 3D backend can't be bit-identical to Rust, because its quaternion, matrix and GJK paths
    are hisab's. Agreement there rests on class-D bounds and physical reference tests.
  - Step parity needs internal state that Rust's public API doesn't expose: full 3D orientation,
    manifolds, warm-start impulses and sleep timers. A hidden `oracle` feature was added after
    the 1.4.0 release as unreleased port tooling, so the oracle is tag 1.4.0 plus that read-only
    change.
  - Until hisab has allocation-free vector operations (request H1), the solver's inner loops use
    plain struct values or scalars rather than hisab handles.
  - Two toolchains are needed until the port completes: cargo for the oracle and its generator,
    cyrius for the library.
- **Neutral**
  - Cargo's exclusive `2d` / `3d` features become two bundle profiles, `[lib]` and `[lib.3d]`,
    with the same public names.
  - Snapshots get a new, complete format with no oracle, checked by round trip and
    step-equivalence.

## Alternatives considered

- **Bit parity everywhere, transcribing glam and impetus's own GJK.** Rejected: it duplicates
  hisab, and the maintainer chose hisab-first. It would also hide the gaps the port is meant to find.
- **Port and fix together, with no frozen oracle.** Rejected for the reason tanmatra gives:
  without a fixed reference, a translation error and a deliberate change look the same.
- **Wait until kiran or soorat need impetus in Cyrius.** Rejected: the port is part of how the
  ecosystem gets ready for them, and its benchmarks and issues are inputs to that work.

## Amendment — M0 step 9 (2026-11-09): hisab stays off the step path until allocation can be scoped

**Context.** Probe 1 (`docs/development/port-parity.md` §4.1) measured what a hisab call costs a
running world:
- The global `alloc` is a bump allocator with no free and no redirection hook. hisab calls it
  directly, and a library can't `alloc_reset` without wiping its consumer's heap.
- Every hisab function that returns a value therefore leaks for the life of the process: 24 bytes
  per vector operation, 105,920 bytes per `gjk_epa_3d` on two spheres, 10,936 per `detect_islands`
  over 100 bodies, 20,792 per `bvh_build` of 100 boxes.
- A 3D scene with 100 touching pairs would leak about 10 MB per step.

**Decision.** hisab-first stands as the direction. Until hisab's allocations can be scoped, the port
calls hisab **only off the step path**: at setup (`inertia_box` / `inertia_sphere` at `add_collider`)
and nowhere a step or a per-frame query runs. Scoping means either a stdlib allocation scope that
redirects `alloc` to a caller's arena (request S1, port plan §8), or allocator-taking hisab variants
(H9).
- The step path and the queries are impetus-owned.
- The 2D backend (M1–M3) needs nothing from hisab on its step path, so it is unaffected.
- The 3D narrowphase (M4) decides at its start between two options:
  - wait for S1 or H9;
  - port impetus's own GJK/EPA as a stopgap, under its own ADR, and switch to hisab when the
    allocation is scoped.

**Consequences.**
- The gap list (port plan §8) leads with S1 and H9, measured.
- Every hisab use on the step path that this ADR names (3D quaternion and matrix ops, GJK/EPA, BVH,
  `detect_islands`) moves behind that request. Where the port plan lists them as "use", read "use
  once S1/H9 lands".

**M4 decision (2026-11-09).** The maintainer chose the stopgap: [ADR 0012](0012-impetus-owned-3d-math-stopgap.md)
transcribes glam's 3D math by value and ports impetus's own GJK/EPA, gated at classes A–C. H9 is filed in hisab
(`hisab/docs/development/issues/2026-11-09-allocator-scoped-calls.md`) alongside S1.

## Amendment — M7 part 3 (2026-11-10): hisab is a test-scope dependency until the library calls it

**Context.** At 2.0 no library module calls hisab. The step path, the queries and the setup are all
impetus-owned: the 2D backend never needed hisab, the 3D math and GJK/EPA are ADR 0012's stopgap, and
the inertia at `add_collider` is the oracle's own formula, transcribed. The one caller is
`tests/impetus_3d.bcyr`, whose two `(allocates)` rows measure hisab's forms beside ADR 0012's. Yet
`[deps.hisab]` was a production dependency, and Cyrius walks a dependency's production deps into its
consumer's resolve and prepends every module it vendors to every compile. A scratch consumer that only
creates and frees a 2D world (M7 part 3) showed what that cost:

| `[deps.hisab]` in impetus | vendored into the consumer | binary | `cyrius build` |
|---|---|---:|---:|
| production | impetus, hisab, sakshi | 3,681,032 B | 1.56 s |
| `scope = "test"` | impetus | 1,702,480 B | 0.55 s |

**Decision.** `[deps.hisab]` carries `scope = "test"`: impetus's own tests and benches still get it, and
no consumer does. hisab-first stands as the direction. The dependency returns to production scope with
the first library call, which waits on S1 or H9 as the first amendment says.

**Consequences.**
- A consumer's resolve, binary and compile time carry only impetus and the stdlib leaves in
  `dist/impetus.deps`.
- `cyrius deps --locked`, `cyrius.lock` and the bundles are unchanged: impetus's own resolve still
  includes hisab for its test pass.
- When a library module first calls hisab, the scope goes and the CHANGELOG says so, since every
  consumer's resolve grows by hisab and sakshi.
