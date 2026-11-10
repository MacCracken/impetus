# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [2.0.0] - 2026-11-10

**impetus is a Cyrius library.** 2.0.0 ports the Rust crate 1.4.0 to Cyrius 6.7.18, on two bundles:
`dist/impetus.cyr` (2D) and `dist/impetus-3d.cyr` (3D). The Rust crate is frozen in `rust-old/` as the parity
oracle. Every module is ported except the bridges to dravya, sharira and ushma, which wait for a Cyrius consumer
(port plan §9). The plan of record is `docs/development/impetus-port-plan.md`; code moving from the Rust crate reads
[the migration guide](docs/guides/migration-2.0.md).

### Breaking
**impetus 2.0 is a Cyrius library.** The Rust crate stays at 1.4.0, frozen in `rust-old/`. Code that used it moves
with the [migration guide](docs/guides/migration-2.0.md), whose every Cyrius example compiles and runs
(`scripts/doc_examples.py`).
- **The language.** The API keeps Rust's names in Cyrius form:
  - structs are `Im<Name>` with methods of the Rust names, constructors `Im<Name>_<constructor>`, and enum members
    and constants carry `IM_`;
  - a fallible call returns a value-form `Result` whose `Err` is an `IM_ERR_*` code. `ImpetusError`'s message
    strings are gone (`im_error_message(code)` gives fixed text), and `IM_ERR_UNSUPPORTED` is new;
  - an `Option` return becomes an out-pointer and a 1/0 return, a `Vec` return an `ImWordList` the caller owns, and
    a slice a pointer and a count;
  - what a caller creates, it frees: `world.free()`, and `free()` on snapshots, word lists and array-holding shapes.
- **Cargo's features become two bundles:** `dist/impetus.cyr` (2D, Rust's default `2d`) and `dist/impetus-3d.cyr`
  (`3d`, `full`). Snapshots and JSON are in both, so `serialize` has no counterpart; `oracle` stays in `rust-old/`.
- **Seven API changes** ([ADR 0008](docs/adr/0008-port-api-divergences.md)):
  1. `add_collider` and `add_joint` return `Err(IM_ERR_BODY_NOT_FOUND)` for a missing body, where 1.4.0 stored an
     orphan. Find the call sites by search: a discarded `Result` still compiles.
  2. One fallible `restore`, Rust's `try_restore`; the deprecated `restore` is gone.
  3. Snapshots hold the whole world ([ADR 0013](docs/adr/0013-complete-snapshots.md)), in a new checksummed binary
     form and in JSON, so a restored world steps to the original's bits. 1.4.0's bitcode bytes and snapshot JSON
     can't be read: rebuild those worlds once from scene data, then save 2.0 snapshots.
  4. The 3D `ImBodyState` has `rotation: ImQuat` and `angular_velocity: ImVec3`; 1.4.0's angle θ and spin ω are
     `ImQuat_from_rotation_z(θ)` and (0, 0, ω), bit for bit. Its JSON changes to match, and 1.4.0's 3D document is
     refused. The 2D `ImBodyState` is unchanged.
  5. The 3D profile refuses Revolute and Prismatic joints and motors with `Err(IM_ERR_UNSUPPORTED)`, in `add_joint`
     and `restore`. 1.4.0's 3D world accepted them and ignored them.
  6. `IM_SOLVER_XPBD` is refused by `ImPhysicsWorld_try_new` and `restore` with `Err(IM_ERR_UNSUPPORTED)`.
     `ImPhysicsWorld_new` doesn't validate, as Rust's `new`, and steps such a world as sequential impulses.
  7. Results are bit-identical across x86_64 Linux, x86_64 Windows and aarch64; 1.4.0's were only on the same
     platform.
- **Not carried over:** XPBD (#6); the dravya, sharira and ushma bridges, deferred until a Cyrius consumer needs
  them (port plan §9).

### Verified
On x86_64 Linux (ELF), x86_64 Windows (PE, under wine) and aarch64 (under qemu), with Cyrius 6.7.18 and the oracle
built by rustc 1.99.1 against glibc 2.44:
- **Golden corpus:** 2D **98,874 ok, 0 failed, 6 pending, 166 accepted**; 3D **29,404 ok, 0 failed, 11 accepted**
  (`docs/development/port-parity.md`). The digests of impetus's own outputs are equal on all three targets
  (ADR 0008 #7).
  - Every single-step, contact and world-step row is at 0 ulp. The accepted rows are inputs the 2.0 API refuses
    (ADR 0008) and the rows where glibc misrounds and the port is correctly rounded (ADR 0011, against MPFR).
  - The 6 pending rows wait on bayan: integers above `i64::MAX` (S3) and duplicate keys (S4).
- **The Rust tests:** all 432 are accounted for in `docs/development/rust-test-ledger.md`, gated by
  `scripts/rust_test_ledger.py --check`. 388 are ported to a test of the same name, 17 are not applicable (8 XPBD,
  9 for the oracle hooks), and 27 are deferred with the bridges. Rust's two `#[ignore]`d restore tests run and pass.
- **Surface parity:** all 662 public items of the 30 ported Rust modules have their Cyrius names
  (`scripts/surface_parity.py`).
- **Benchmarks:** every one of Rust's 41 benches has a twin on each profile it applies to
  (`docs/benchmarks-rust-v-cyrius.md`, `scripts/bench_record.py --check`).
- **Snapshots** (no oracle): exact round trips, step-equivalence on both profiles, refusals that leave the world
  unchanged, and the fuzz targets on both profiles.
- **The docs:** every Cyrius code block in the README and the guides compiles and runs against the bundles
  (`scripts/doc_examples.py`).

### Known limitations
- JSON can't read back an integer above `i64::MAX`, so a handle whose slot was reused more than 2^31 times
  doesn't round-trip through JSON (bayan S3). The binary snapshot holds it.
- A JSON document with a duplicate key is refused, where serde ignores repeats of unknown fields (bayan S4).
- The JSON writers exit the process when the stdlib's `str_builder` can't grow, the one abort left in the
  library (port plan §8).
- Speed: the median benchmark is 10× Rust's in 2D and 13× in 3D; a whole step is 2.6–11× and 5.7–17×. These are
  first numbers. The upstream issues behind them are filed (port plan §8), among them S5, the stdlib's byte-loop
  `memcpy`.

The entries below record the port in the order it was done. The `oracle` feature and the `2d` change are
port tooling in `rust-old/`, not a Rust release: they let the golden generator drive the oracle (M0 step 0), and
the simulation is unchanged.

### Added
- **oracle** — a hidden, non-default `oracle` feature (`impetus::oracle`, not
  public API):
  - `PhysicsWorld::oracle_dump()` returns the full world state that the public API
    hides: arena slots and free lists, every body field (full 3D quaternion,
    angular velocity vector and inertia tensors), colliders, joints with
    reference angles and rotations, 2D manifolds with warm-start impulses, and the
    contact tracker;
  - pair-level entry points: `contacts_2d`, `shapes_touch_2d`, `contact_3d`;
  - broadphase candidate pairs from the AABB tree and the spatial hash.

  `tests/oracle.rs` checks that the dump is read-only (dumping every frame
  leaves the simulation bit-identical), that it matches the public state, and
  that it predicts the next handle.

### Changed
- `2d` feature: no longer depends on hisab. The 2D backend never used it; only
  the 3D backend does. The default build no longer compiles hisab and glam.
- **Cyrius port, M0 steps 1–3** ([ADR 0007](docs/adr/0007-cyrius-port-hisab-first.md)):
  - `cyrius port` scaffold. The Rust crate moved into `rust-old/`: sources, tests,
    benches, fuzz targets, examples, Cargo files, `rust-toolchain.toml`,
    cargo-vet config, CI workflows, scripts, the architecture overview and the
    testing guide. `rust-old/README.md` is restored for the crate's README doctest.
  - `cyrius.cyml` pins Cyrius 6.7.18 and hisab 3.4.1; `lib/` and `cyrius.lock` are vendored.
  - Cyrius CI with `test --bounds`, `deps --locked` and `audit`.
  - CLAUDE.md and `docs/development/state.md` for the Cyrius process.
  - ADRs renumbered to four digits; [ADR 0008](docs/adr/0008-port-api-divergences.md)
    records the 2.0 API changes.
- **Cyrius port, M0 steps 4–7**:
  - `scripts/oracle/`, the golden-value generator (kimiya's format), built twice because the
    oracle's backends are exclusive. It writes `tests/golden/constants.tsv` (3 rows) and
    `math.tsv` (18,919 rows: every std f64 method impetus calls, against glibc 2.44), and
    `src/constants.cyr`, with a proof that every public Rust constant is listed.
  - `validate.py` recomputes the 7,197 class-A/B math rows and the constants independently.
  - `tests/parity_2d.tcyr` replays the corpus: 3,154 ok, 0 failed, 11,722 measured (class C,
    bounds come with the step-8 baselines), 4,046 pending (M1's Rust-semantics math helpers).
- **Cyrius port, M0 steps 8–9** (`docs/development/port-parity.md`):
  - Baselines on x86_64 ELF, x86_64 PE and aarch64. A per-function digest shows bit-identical
    output on all three for sin, cos, hypot, pow and every exact row. exp and atan2 use the
    polyfill forms, the only primitives whose native bits differ by target. Class-C bound
    K = 2 ulp, now gated: **14,875 ok, 0 failed, 0 measured, 4,046 pending, 1 accepted**.
  - [ADR 0009](docs/adr/0009-accept-ganita-pow-minus-one-divergence.md) accepts ganita's
    `pow(-1, DBL_MAX)`.
  - Six probes (`scripts/probes/`). Every hisab call that returns a value leaks from the
    global bump heap (24 B per vector op, 105,920 B per `gjk_epa_3d`), so hisab stays off
    the step path until allocation can be scoped (ADR 0007 amendment). Freelist and a
    per-step arena stay flat.
  - Hot loops compute on `f64` locals; struct values work but cost 2.5×.
  - Both solvers' shock-propagation sorts rely on stability, so the port sorts on
    (level, index).
- **Cyrius port, M1a** (port plan §6; `docs/development/port-parity.md` §5):
  - `src/mathx.cyr`, Rust std semantics on the Cyrius builtins: NaN-ignoring min/max/clamp,
    the predicates, to_radians, the literal powi, exp and atan2 on the polyfills, hypot and
    sin_cos. It closes the 4,045 pending math rows.
  - `src/error.cyr`, `vec.cyr`, `config.cyr`, `material.cyr`, `units.cyr` and `force.cyr`.
    `validate` refuses XPBD, accepted by ADR 0008 #6 (92 mapped rows). `Quantity`'s Display
    number waits for a stdlib shortest-printer (S2).
  - Golden config, material, units and force rows; `validate.py` recomputes 5,230 of them.
  - Result: **24,431 ok, 0 failed, 281 pending, 93 accepted**, every row 0 ulp, on x86_64
    ELF, x86_64 PE and aarch64.
  - 33 Rust tests ported, `[lib]` + `dist/impetus.cyr`, and `scripts/surface_parity.py`
    (100/100).
  - 6 benches against the Rust numbers (4.8–31×, `docs/benchmarks-rust-v-cyrius.md`).
- S1 filed upstream: `cyrius/docs/development/issues/2026-11-09-no-scoped-allocator-for-alloc.md`.
- S3 and S4 filed in bayan's queue, each with a self-proving repro:
  `2026-11-09-json-integer-above-i64-max-refuses-the-document.md` (u64 integers) and
  `2026-11-09-json-duplicate-key-has-no-opt-in-for-ignored-members.md` (an opt-in, not a defect).
- S5 filed upstream as an addition to ranga's open issue
  `cyrius/docs/development/issues/2026-11-02-memcpy-memset-byte-loops-dominate-frame-copies.md`:
  the byte-loop `memcpy` costs 4–7× on struct-sized copies too, with a new repro
  (`repros/2026-11-09-memcpy-small-struct-copies.bcyr`).
- Correctly rounded exp, sin, cos and hypot filed for ganita, in its own queue:
  `ganita/docs/development/issues/2026-11-09-correctly-rounded-exp-sin-cos-hypot.md`. It carries a
  self-proving repro: 32 arguments, every one wrong on ganita 1.2.15 / cyrius 6.7.18, all right through
  impetus's crmath. It proposes a correctly rounded tier seeded from `src/crmath.cyr`. Against MPFR,
  `ganita_f64_hypot` misrounds 33.65 % of seeded arguments, and the stdlib polyfills misround 9.73 % (exp),
  3.13 % (sin) and 2.96 % (cos).
- **Cyrius port, M1b part 1** ([ADR 0010](docs/adr/0010-serde-compatible-json.md);
  `docs/development/port-parity.md` §6): serde-compatible JSON, replacing the plan's
  "serde not carried over".
  - `src/fmtf.cyr` prints the shortest round-trip digits (Burger & Dybvig on a bignum) in
    Rust's Display form and serde_json's. All 37,576 golden rows are exact; the two Rust
    printers differ only on exact ties (core rounds up, zmij to even).
  - `src/json.cyr` reads and writes serde_json's documents for WorldConfig, PhysicsMaterial,
    Quantity, Force, Impulse and Torque, on bayan's value tree with serde's derive semantics.
    All 545 golden rows are exact, including the edge documents.
  - The 11 M1a serde tests are ported, and `Quantity`'s Display is complete.
  - Finding: serde_json's default float parse is 1 ulp off on some long decimals. The oracle
    uses `float_roundtrip`, which matches the port's correctly rounded parse.
- **Cyrius port, M1b part 2** (`docs/development/port-parity.md` §7):
  - `src/types.cyr`: body, collider, joint, event and query descriptors. `ColliderShape`,
    `JointType` and `CollisionEvent` are a kind enum (append-only values) plus one struct with
    every variant's fields; shape arrays live on freelist storage the shape owns.
  - `src/arena.cyr`: the generational arena on freelist, handles `generation << 32 | index`,
    with LIFO reuse, `insert_at` and the lazy free-list rebuild (lowest index first), `retain`,
    and generations wrapping at 2^32, as Rust's.
  - `src/contact_tracker.cyr`: sorted pair lists, compared as unsigned u64 like Rust's handles,
    with events once per frame and a contact that starts and stops within a frame reporting both.
  - `src/spring.cyr`: `Spring`, `Spring2d` and `Spring3d`, the exact step with Rust's operation
    order and the cached transition.
  - `src/json.cyr`: codecs for every M1b type: externally tagged payload enums, u64 handles
    written unsigned. Every reader also takes serde's sequence form (a struct as an array) and
    one-key unit variants (`{"Average":null}`), part-1 types included (ADR 0010 findings).
  - Golden rows: json 545 → 937, spring 1,000, types 1. **63,419 ok, 0 failed, 800 measured,
    7 pending, 93 accepted** on x86_64 ELF, x86_64 PE and aarch64. `validate.py` recomputes all
    1,000 spring rows on Python's libm.
  - The spring step rows stay measured: their sensitivity, measured per row, puts the rule's
    class-C bound far past 64, so the rule asks for exact exp, sin and cos (decision open, port
    plan §8). 1,531 of 1,600 step outputs are already bit-identical.
  - Findings: bayan refuses integers above i64::MAX (S3: u64 handles at generation ≥ 2^31) and
    every duplicate key (S4), where serde_json reads both (7 rows pending); the writers inherit
    `str_builder`'s abort on OOM.
  - All 63 Rust tests of the M1b modules ported; surface parity 258/258.

- **Cyrius port, M1b part 3** ([ADR 0011](docs/adr/0011-correctly-rounded-exp-sin-cos.md);
  `docs/development/port-parity.md` §8): exp, sin and cos are correctly rounded.
  - `src/crmath.cyr`: a double-double fast path with a proven rounding test, then a 364-bit fixed-point
    slow path (Payne–Hanek for sin and cos). The same bits on every target.
  - Its tables (`src/crmath_data.cyr`) are generated by `validate.py --write` from π and ln 2 computed
    exactly, and checked against MPFR to 2,048 bits.
  - Equal to MPFR on 180,062 seeded arguments (x86_64 and aarch64), 90,000 in the fast paths' ranges,
    and 1,200 where glibc misrounds.
  - `_im_exp`, `_im_sin_cos`, `_im_sin` and `_im_cos` use them; the polyfills become shadow lines.
  - The corrected oracle, from kimiya (its ADR 0006): where glibc 2.44 misrounds a call (0.1–0.2 % of
    exp, sin and cos arguments), `tests/golden/<module>.cr.tsv` holds the oracle's operations on
    correctly rounded results, and the harness compares those rows against it. There are 10 such rows
    (math 9, spring 1), all equal to MPFR.
  - Class C is exact (bound 0) for exp, sin, cos, sin_cos and `Spring::step`: all 800 step rows are
    gated and 0 ulp. The math grid gains 246 threshold rows.
  - Result: **64,455 ok, 0 failed, 0 measured, 7 pending, 103 accepted** on x86_64 ELF, x86_64 PE and
    aarch64.
  - For a Rust 1.x user: results differ in the last bit wherever glibc misrounds an exp, sin or cos.
  - `scripts/oracle/mpfr_ref.py` (MPFR through ctypes, from kimiya) and `crtables.py`; the harness core
    takes kimiya's corrected-oracle replay, with its transitional rule off.
  - A table-driven phase 1 (2^(j/64); sin and cos of i/64) before the double-double one decides about
    99.7 % of calls. Results are unchanged (same digests, same probes); its measured worst error is
    2^-66 against a 2^-62 bound. The tables are generated and checked against MPFR.

- **Cyrius port, M2 part 1: the broadphase** (`docs/development/port-parity.md` §9):
  - `src/aabb_tree.cyr`, the dynamic AABB tree: SAH insertion, Box2D's AVL balance, 0.1 fattening.
  - `src/spatial_hash.cyr`, the spatial hash: cells as sorted (cell, key) records rather than a map, and
    an oversize list past 4,096 cells or for non-finite boxes.
  - Both are 2D and 3D, keep their storage across rebuilds, and order keys as unsigned words.
  - Golden `broadphase.tsv`: 76 rows of pair sets and cell sizes, all exact on three targets.
    `validate.py` recomputes every row by brute force.
  - The 18 Rust tests are ported, and surface parity is 275/275. The harness takes rows of up to
    8,192 tokens.
- **Cyrius port, M2 part 2: the 2D backend's types** (`docs/development/port-parity.md` §10):
  - `src/b2_types.cyr`:
    - rigid bodies: inverse masses, the solver's view, velocity and position integration;
    - colliders: world AABBs, and mass properties from `shape_props` and `polygon_mass_props` (either
      winding, with the `MIN_MASS` / `MIN_INERTIA` floors);
    - the per-collider accumulation `add_collider` does, shifted by the parallel-axis theorem;
    - the joint, contact and manifold records, and the island union-find (`union` is `unite`, a
      reserved word).
  - `ColliderShape::clone`: a collider owns its copy of the shape's arrays, as Rust's does.
  - Private constants are generated: `scripts/oracle/src/t_private.rs` writes
    `src/constants_private.cyr` from the Rust literals (11 values). It proves each module's `const`
    items are emitted or excused, and `validate.py` checks each value. The broadphase now reads
    its margin and cell cap from it.
  - Golden `b2_mass.tsv`: 75 bodies from world dumps (every shape kind and its edge sizes, offsets,
    densities, explicit and NaN masses, body types, several colliders). All 0 ulp on three targets,
    and `validate.py` recomputes every body. **64,606 ok, 0 failed, 0 measured, 7 pending,
    103 accepted.**
  - The 12 Rust type tests are ported, with Cyrius-only groups (65 assertions). Surface parity is
    374/374, now counting `pub(super)` items.
- **Cyrius port, M2 part 3: the 2D narrowphase** (`docs/development/port-parity.md` §11):
  - `src/b2_narrowphase.cyr`:
    - the round tests: circle, box, capsule, segment and hull against a circle, including a centre
      inside the hull;
    - Box2D's reference/incident-face clipping for boxes, hulls (either winding) and segments, with
      the SAT fallback;
    - capsule end-cap contacts against boxes, hulls and segments;
    - `generate_contacts_multi` and `shapes_touch`.

    Contacts are five-word records in a caller's two-contact buffer.
  - Golden `b2_narrow.tsv`: 17,854 rows. They cover every ordered pair of a 12-shape catalog at
    0/π/4/π/2 and seeded rotations, poses touching and ±10⁻⁹ apart, the Rust tests' poses, NaN and
    infinite poses, 24- and 40-vertex hulls, and 2,400 seeded pairs.
    - Contacts are class C, at 0 ulp against the corrected oracle; the touch test is class A.
    - `validate.py` reproduces every row through a Python transcription. It also writes
      `b2_narrow.cr.tsv`: the 3 rows where glibc misrounds a `sin`, which the port matches.
    - **82,457 ok, 0 failed, 0 measured, 7 pending, 106 accepted** on three targets.
  - The 32 Rust narrowphase tests are ported, with Cyrius-only groups (96 assertions). Surface
    parity is 386/386.
  - Found on the way: multi-return values are untyped words, so an operator applied to one works on
    the integer bits, with no f64 diagnostic (port plan §8). Two golden rows caught it, and the fix
    binds such values to `f64` locals.
- **Cyrius port, M2 part 4: correctly rounded hypot, and the 2D queries**
  ([ADR 0011, hypot amendment](docs/adr/0011-correctly-rounded-exp-sin-cos.md);
  `docs/development/port-parity.md` §12). M2 is done.
  - `_im_cr_hypot` in `src/crmath.cyr`:
    - a double-double phase 1 after exact scaling, then an exact fixed-point slow path that rounds
      the true midpoints (Pythagorean triples with a 54-bit odd hypotenuse) to even;
    - matches MPFR on 404,234 probe pairs, with identical bits on x86_64 and aarch64;
    - `_im_hypot` uses it, and the 1,715 `f64::hypot` rows are gated at 0 ulp (12 corrected where
      glibc misrounds). ganita's `hypot` stays as a shadow line.
  - `src/b2_raycast.cyr`:
    - the ray tests for circle, oriented box, capsule, segment and convex hull;
    - `RayInterval`;
    - the per-collider parts of `raycast_filtered`, `overlap_sphere` and `overlap_aabb`. Their
      world loops arrive with ImPhysicsState2d in M3.

    No hisab: queries stay off it with the step until allocation can be scoped (ADR 0007
    amendment).
  - Golden `b2_ray.tsv`: 14,754 rows through the public world API.
    - Rays are class C at 0 ulp against the corrected oracle (39 rows corrected, 38 of them
      through `hypot`). Overlaps are class A and also pin `world_aabb`.
    - `validate.py` reproduces every row on glibc's sin, cos and hypot.
    - **97,160 ok, 0 failed, 0 measured, 7 pending, 157 accepted** on three targets.
  - The 18 Rust raycast tests are ported (45 assertions), plus 14 `hypot` assertions in
    `crmath.tcyr`. Surface parity is 394/394, with the 4 world query loops listed as pending M3.
  - The oracle's 2D emitters share their shape builders (`scripts/oracle/src/shapes.rs`), and their
    output is unchanged.
- **Cyrius port, M3 part 1: the 2D backend's state** (`docs/development/port-parity.md` §13):
  - `src/b2_state.cyr`, ImPhysicsState2d:
    - bodies, colliders and joints in generational arenas, so handles are Rust's;
    - each body's collider list beside its slot;
    - the manifold store in key order, the contact tracker and the island union-find;
    - every `state.rs` method: adds, removals with cascades and waking, `recompute_mass`, forces,
      impulses, torques, the snapshot inserts, and the body-state getters and setters;
    - the M2 query methods: `raycast`, `raycast_filtered`, `overlap_sphere`, `overlap_aabb`.

    A collider or joint for a missing body is `Err(IM_ERR_BODY_NOT_FOUND)` (ADR 0008 #1).
  - Golden `b2_state.tsv`: 411 scripts through the world API, each ending in the full backend dump
    (`scripts/oracle/src/dump2d.rs`). All 0 ulp on three targets; the 3 orphan scripts are mapped.
    `validate.py` replays every script on a model of Rust's arenas and the state.
    **97,568 ok, 0 failed, 0 measured, 7 pending, 160 accepted.**
  - The ray and overlap rows and the world-level raycast tests now run on ImPhysicsState2d. 5 more
    Rust tests are ported (28 assertions). Surface parity is 417/417; it now reads a receiver on the
    line after `pub fn name(`.
- **Cyrius port, M3 part 2: joints, integration, islands and sleep** (`docs/development/port-parity.md`
  §14):
  - `src/b2_joints.cyr`: the velocity-level joint solver for all eight joint types. Rigid points,
    angles and axes carry Baumgarte bias; springs, wheels, mice and joint damping are soft; limits
    are one-sided; motors are clamped to `max_force · dt`; joints break above their force limit. The
    rows live in scratch the state reuses each step.
  - `src/b2_solver.cyr`: the stages a step runs whether or not anything touches. These are
    integration, islands and sleep, and `finish_step`.
  - The first single-step rows, `b2_step.tsv`: 214 rows, each a backend dump at frame k in and the
    dump at k + 1 out. The scenes cover every joint type, sub-steps, breaking, a kinematic driver,
    sleep onset and 40 seeded scenes; their colliders pair nothing. The harness loads each dump into
    an ImPhysicsState2d.
    - All 0 ulp against the corrected oracle on three targets, with 4 rows corrected where glibc
      misrounds.
    - `validate.py` transcribes the step and reproduces every row.
    - **97,778 ok, 0 failed, 0 measured, 7 pending, 164 accepted.**
  - The mouse-joint test is ported, with Cyrius-only joint, island and sleep tests (21 assertions).
    Surface parity is 421/421. `JOINT_BAUMGARTE` is generated.
- **Cyrius port, M3 part 3: the contact pipeline and `step`** (`docs/development/port-parity.md` §15):
  - `src/b2_contacts.cyr`: the contact stages of the sequential-impulse step.
    - The broadphase, under either kind, sweeps each AABB by its body's velocity. It skips pairs on
      the same body, pairs of static bodies, sensor pairs and pairs whose layers miss each other's
      masks.
    - One-shot manifolds are rebuilt each sub-step and warm started from last sub-step's matching
      points, unless the normal flipped.
    - Sleeping bodies wake on contact.
    - The contact solver orders manifolds by shock propagation. It block-solves two-point manifolds
      when well conditioned, applies Coulomb friction with static friction below 0.01 m/s, resists
      rolling, and bounces above 1 m/s.
    - Penetration is corrected by split impulses, through the soft constraint or Baumgarte.
    - Storage is scratch reused every step; a test confirms the heap stays flat.
  - `step` (`src/b2_solver.cyr`): one sub-step, then the touching pairs recorded for events. XPBD
    stays refused (ADR 0008 #6), so `step` runs the sequential-impulse solver for any solver kind.
  - The harness's `b2::step` rows now run the library's `step`. New `b2_contact.tsv`: 418 single
    steps with contacts, under both broadphases. Its scenes cover every material rule, shape family
    and position-solver setting, plus sensors, layers, sleep and wake, fast bodies, joints over
    ground, and 40 seeded scenes.
    - All 0 ulp against the corrected oracle on three targets. Two rows are corrected, each where
      glibc's `cos` misrounds.
    - `validate.py` now transcribes the whole step, contacts and events included, and reproduces all
      632 step rows on glibc.
    - **98,194 ok, 0 failed, 0 measured, 7 pending, 166 accepted.**
  - 24 more Rust tests are ported in `tests/b2_contacts.tcyr`: sensors, kinematics, sleep and wake,
    layers, manifold persistence, warm starting, the stepping joint tests, static friction and
    sub-stepping. Cyrius-only groups check that the broadphase kinds agree, the block solver, restitution,
    rolling resistance, the event order and the flat heap (62 assertions). The three xpbd_* tests are
    not ported.
  - Surface parity is 418/418 across 21 modules. It now skips `#[cfg(test)]` helpers.
    `RESTITUTION_VELOCITY_THRESHOLD` and `MAX_CONDITION_NUMBER` are generated.
  - The parity harness names a failing row's first differing output.
- **Cyrius port, M3 part 4: correctly rounded pow, the world and its particles**
  (`docs/development/port-parity.md` §16):
  - **pow** (ADR 0011 amended; ADR 0009 superseded). `src/crmath.cyr` gains `_im_cr_pow`, correctly
    rounded on every input with C99 Annex F's specials.
    - Two double-double phases: log x by the atanh series, then exp.
    - A slow path that computes dyadic results exactly, its ties to even, and otherwise uses the
      364-bit fixed point.
    - It matches MPFR on 1,290,769 pairs, and ganita's pow misrounds 9.62 % of ordinary arguments.
    - `f64::powf` rows are gated at 0 ulp. The `pow(-1, DBL_MAX)` row is an ordinary match.
  - `src/particle.cyr`: particles, emitters, sub-emitters and force fields, with Rust's builders.
  - `src/world.cyr`: `ImPhysicsWorld`, the 2D world. Its step runs the frame counter, the backend's
    sub-steps and events, then the particles: spawns with the R3 spread and the 10,000 cap, gravity,
    radial and directional fields, quadratic drag, damping, collisions, sub-emitters and removal of
    the dead. `add_collider` and `add_joint` return a Result (ADR 0008 #1); `ImPhysicsWorld_try_new`
    validates and refuses XPBD (#6).
  - The new `world_step.tsv` holds 198 world steps from dumps that include the particles, emitters
    and fields. All 0 ulp on three targets, and `validate.py` transcribes the world step.
    **98,393 ok, 0 failed, 0 measured, 7 pending, 165 accepted** before particle JSON.
  - 65 more Rust tests: `world.rs` (22), `particle.rs` (8), `tests/integration.rs` (9) and the 2D
    tests of `tests/regressions.rs` (26), in `tests/world.tcyr` and `tests/regressions.tcyr`.
  - Surface parity is 507/507 across 23 modules, with `snapshot`, `restore` and `try_restore`
    pending M6.
  - The world's constants are generated, the `R3_ALPHA` array as three globals.
  - **Particle JSON** (ADR 0010): `src/json.cyr` writes and reads `ForceField`, `SubEmitter`,
    `Particle` and `ParticleEmitter` as serde_json does. That includes the emitter's private
    `accumulator`, which serde carries, and `on_death_emit`'s default. `json.tsv` grows from 937
    to 1,092 rows (sequence forms, bare variant names, missing and extra fields, a u32 out of range),
    all on the first run. The two particle serde tests are ported.
    **98,548 ok, 0 failed, 0 measured, 7 pending, 165 accepted.**
- **Cyrius port, M4 part 1: 3D math and the 3D backend's types** ([ADR 0012](docs/adr/0012-impetus-owned-3d-math-stopgap.md);
  `docs/development/port-parity.md` §17):
  - **ADR 0012**: the 3D step path runs on impetus-owned math until hisab's allocations can be scoped
    (S1 in cyrius, or H9 in hisab). H9 is filed in hisab with a repro that exits 1 while 7 of 7 step-path
    calls leak (24 B for a vector operation to 105,920 B for `gjk_epa_3d`).
  - `src/math3.cyr`: glam 0.29.3's `DVec3`, `DQuat` and `DMat3` operations, transcribed by value with
    glam's operation order (`ImVec3` operations, `ImQuat`, `ImMat3`; `+` and `−` overloaded).
  - `src/b3_types.cyr`: `ImRigidBody3d` (`from_desc`, the world inverse inertia R·I⁻¹·Rᵀ,
    `integrate_velocities` with the implicit, energy-projected gyroscopic step, `integrate_positions`),
    `ImCollider3d` (world AABBs, mass, inertia and centroid for every shape), the parallel-axis shift,
    the mass recompute in three steps, `ImJoint3d`, `ImAabb3d` and `ImContact3d`. The 3D private
    constants are generated (`_IM_B3_*`).
  - `_im_signum` (Rust's `f64::signum`, for glam's `any_orthonormal_pair`); `math.tsv` grows by 326
    signum rows to 19,491.
  - **The 3D profile**: `[lib.3d]` in `cyrius.cyml` (bundle `dist/impetus-3d.cyr`), `tests/lib_3d.cyr`,
    and `tests/parity_3d.tcyr` for the `b3_*` modules. The descriptor token helpers moved from
    `parity_2d.tcyr` to the shared `tests/parity_types.cyr`.
  - The oracle's `b3` build writes its first modules: `b3_math.tsv` (7,087 rows of glam operations,
    through hisab 1.4.0's re-export, now a dependency of the `b3` build) and `b3_body.tsv` (234 bodies
    from world dumps). All 0 ulp on x86_64 ELF, x86_64 PE and aarch64, with identical digests, and
    recomputed by `validate.py`. **7,321 ok, 0 failed** in 3D; the 2D corpus is **98,874 ok**.
  - 23 assertions in `tests/b3_types.tcyr`: the Rust type tests (quaternions, masses, the capsule
    tensor, the parallel-axis theorem, gyroscopic precession), a free spin integrated for 600 steps
    without gaining energy, and a flat heap.
  - Surface parity is 583/583 across 24 modules. It now also checks that each profile's test mirror
    matches `cyrius.cyml`, and scans for the compiler defect below.
  - **Compiler defect, worked around and filed**: cyrius 6.7.18 stores the wrong value in
    `s.f = g(t.e)` when `g` returns a struct and `t.e` is a struct-typed field. It stores `t.e`
    instead of the result. The defect left 202 of the 234 bodies with their inertia as their
    inverse inertia. `src/b3_types.cyr` binds such calls to a local first. The filing is
    `cyrius/docs/development/issues/2026-11-09-field-store-of-struct-call-copies-a-field-argument.md`,
    with a repro that exits 1.
- **Cyrius port, M4 part 2: the 3D narrowphase** (`docs/development/port-parity.md` §18):
  - `src/b3_narrowphase.cyr` ports `generate_contact_3d` and every pair it handles:
    - the closed forms for ball, box, capsule and segment pairs;
    - the AABB fast path, and the 15-axis OBB separating-axis test with reference-face clipping and edge–edge points;
    - impetus's own GJK + EPA for hulls (ADR 0012).

    The GJK and EPA working sets live in an `ImGjkScratch3d` the caller owns and reuses. Rust's `Vec` order
    semantics (`retain`, `swap_remove`, the first minimum by `total_cmp`) are kept exactly.
  - The golden `b3_narrow.tsv` has 16,340 pairs through the oracle's `contact_3d` hook:
    - every ordered pair of a 15-shape catalog at 4 rotation pairs and 15 placements;
    - the unhandled shapes against all of them;
    - targeted cases (exact touching, centres inside boxes, edge–edge boxes, lying capsules, degenerate hulls, large
      and non-finite poses);
    - 2,000 seeded pairs.

    Class A, because the narrowphase has no transcendental. **All 16,340 match bit for bit** (6,828 contacts, signs
    of zero included) on x86_64 ELF, x86_64 PE and aarch64, on the first run. `validate.py` transcribes the whole
    narrowphase in Python and reproduces every row.
  - glam's `DVec3::clamp` (max then min, unlike `f64::clamp`) joins `src/math3.cyr`, with 229 `b3_math` rows.
  - The narrowphase's own constants (`REL_TOL`, `ABS_TOL`, the GJK and EPA iteration caps) are generated.
    `ImRecordList` moved to `src/aabb_tree.cyr`, so both profiles have it.
  - 37 Rust narrowphase tests are ported in `tests/b3_narrowphase.tcyr`, plus Cyrius-only groups for the unhandled
    pairs and the scratch reuse (a flat heap after warm-up): 65 assertions.
  - Surface parity is 599/599 across 25 modules. **3D: 23,890 ok, 0 failed.**
- **Cyrius port, M4 part 3: the 3D raycast; M4 done** (`docs/development/port-parity.md` §19):
  - `src/b3_raycast.cyr`:
    - the ray tests against a ball, a box in its own frame, and a capsule (cylinder and end-ball candidates, kept on
      the surface);
    - `raycast_impl`'s nearest-hit loop over a body arena and a collider arena in slot order, with the layer filter
      (`raycast_filtered`) or none (`raycast`, which also sees layer-0 colliders).

    The 3D state (M5) calls it with its own arenas. Rust 1.4.0's 3D world has no overlap queries (`overlap_sphere` and
    `overlap_aabb` return nothing in 3D), so there are none to port.
  - The golden `b3_ray.tsv` has 4,138 rays through the public world API:
    - every shape kind at four poses with 52 rays each (axes, obliques, tangents, from inside, along a slab and along
      the axis, max distances 3.9, 4, 100 and ∞);
    - degenerate rays (zero, tiny, NaN and infinite directions, non-finite origins, max distance 0 or negative);
    - nearest-hit, tie and layer-mask scenes;
    - 2,000 seeded scenes.

    Class C (a body's z rotation is a `sin_cos`): 4,136 at 0 ulp, and the 2 that glibc's `sin` misrounds match the
    corrected oracle (`b3_ray.cr.tsv`), on all three targets. `validate.py` transcribes the raycast.
  - The 7 Rust raycast tests are ported in `tests/b3_raycast.tcyr` (23 assertions, with a Cyrius-only layer and
    nearest-hit group and a flat heap). The four state-level tests run on a test-local pair of arenas until M5.
  - Surface parity is 604/604 across 26 modules, with `PhysicsState3d::raycast` and `raycast_filtered` pending M5.
  - **M4 is done.** 3D: 4 modules, **28,026 ok, 0 failed, 2 accepted (ADR 0011)**. 2D unchanged at **98,874 ok**.
- **Cyrius port, M5 part 1: correctly rounded atan2 and the 3D state** (`docs/development/port-parity.md` §20):
  - **atan2** ([ADR 0011](docs/adr/0011-correctly-rounded-exp-sin-cos.md) amended). The 3D state reads a body's rotation
    back as 2·atan2(q.z, q.w), and the port's atan2, ganita's on the atan polyfill, misrounds 25.70 % of ordinary
    arguments. `src/crmath.cyr` gains `_im_cr_atan2`, correctly rounded on every input with C99 Annex F's specials:
    - ratios below 2^-60 are the IEEE quotient, except at exact midpoints of the subnormal grid, which round toward
      zero;
    - phase 1 in double-double, from a generated atan(i/64) table and a series through t^11, decides 99.9 % of
      arguments;
    - the slow path is vectoring CORDIC in the 364-bit fixed point, each constant from its series, so there is no
      static table.

    It matches MPFR on 1,355,242 pairs; glibc 2.44 misrounds 0.56 % and 0.14 % of the two sets. The 1,200
    `f64::atan2` rows are gated at 0 ulp (2 before). glibc misrounds one of them, so `math.cr.tsv` has 22 rows. The
    old atan2 stays as an ungated shadow. `tests/crmath.tcyr` grows to 78 assertions, and atan2 joins ganita's
    correctly rounded tier filing (8 more repro rows).
  - `src/b3_state.cyr`: `ImPhysicsState3d` holds the body, collider and joint arenas, each body's collider list, the
    contact tracker and the GJK scratch. It has:
    - Rust's add, remove and `_at` methods, and the mass recompute over every collider;
    - forces, impulses and torques, impulses through the world inverse inertia;
    - `get_body_state`, `set_body_state` (a rotation about z) and `set_body_type`;
    - `raycast` and `raycast_filtered` over `_im_b3_raycast`.

    `add_collider` and `add_joint` refuse a body that isn't live (ADR 0008 #1).
  - The golden `b3_state.tsv` holds 411 scripts on `PhysicsState3d`, each ending in the 3D backend dump
    (`scripts/oracle/src/dump3d.rs`). The token writers both dumps use moved to `dumpcore.rs`, and the 2D corpus is
    unchanged. Class C: 403 at 0 ulp; the 5 where glibc misrounds an atan2, cos or sin match the corrected oracle
    (`b3_state.cr.tsv`); the 3 orphan scripts are accepted under ADR 0008 #1. `validate.py` replays every script on a
    model of the 3D state.
  - 25 assertions in `tests/b3_state.tcyr`: the state-level Rust tests that need no step (body counts, teleports, the
    missing body, both type changes, colliders accumulating mass, the parallel-axis theorem with a removal, the
    world-space angular impulse), with Cyrius-only refusal, joint, removal and flat-heap groups. The four state-level
    raycast tests now run on `ImPhysicsState3d`.
  - Surface parity is 627/627 across 27 modules; only the M6 snapshot functions are pending.
  - **3D: 28,429 ok, 0 failed, 10 accepted (ADR 11: 7)**; 2D **98,873 ok, 0 failed, 7 pending, 166 accepted (ADR 11:
    71)**, one atan2 row having moved to the corrected oracle. Identical digests on x86_64 ELF, x86_64 PE and aarch64.
- **Cyrius port, M5 part 2: the 3D joints and the step when nothing touches** (`docs/development/port-parity.md` §21):
  - `src/b3_joints.cyr` ports all of `backend_3d/joints.rs`:
    - the Fixed joint's coupled 6×6 block through the Schur complement, or the point block alone;
    - the Distance, Rope and Wheel rows with their split-impulse position passes;
    - the explicit forces once per sub-step (Spring, the Wheel's suspension, Mouse, the per-joint damping; Revolute
      and Prismatic are no-ops in Rust 1.4.0's 3D backend apart from that damping);
    - `break_joints`.

    The constraints live in a scratch the state reuses from step to step.
  - `src/b3_solver.cyr` has the stages of the step that run when nothing touches: integration (the first golden test
    of M4's gyroscopic step), the per-body sleep check, `finish_step` and `error_reduction`. The contact stages and
    `step` come with part 3.
  - The golden `b3_step.tsv` holds 333 single steps with every collider at mask 0. It covers spins, torques, the speed
    cap, every 3D joint path, breaking, kinematics, sleep, the solver settings and 50 seeded scenes. The harness
    composes the contact-free sub-step from the library's stages (`tests/step3.cyr`) and checks with the broadphase's
    own filters that no pair could touch. **All 333 at 0 ulp** on the first run. `validate.py` transcribes the sub-step
    and reproduces every row; there is no correction file.
  - 31 assertions in `tests/b3_joints.tcyr`: the 11 stepping Rust tests whose colliders can't meet, plus Cyrius-only
    groups for the Prismatic no-op, breaking by explicit force, the point block alone, `error_reduction` and a flat
    heap. `impulse_changes_velocity_3d` joins `tests/b3_state.tcyr`.
  - The config and dump loaders moved from `parity_2d.tcyr` to the shared `tests/parity_types.cyr`.
  - Surface parity is 632/632 across 28 modules.
  - **3D: 28,762 ok, 0 failed, 10 accepted (ADR 11: 7)**; 2D unchanged at **98,873 ok**. The same on x86_64 ELF,
    x86_64 PE and aarch64.
- **Cyrius port, M5 part 3: the 3D contacts and `step`** (`docs/development/port-parity.md` §22):
  - `src/b3_solver.cyr` completes `backend_3d/solver.rs`:
    - the broadphase (swept boxes, the spatial hash or the AABB tree);
    - the narrowphase stage on the state's GJK scratch;
    - wake on contact and the shock-propagation order;
    - the contact solver: restitution, two-direction friction with the static cone, rolling resistance, and the
      split-impulse position pass;
    - `step` itself.

    The solver's two private constants are generated. The step's working sets are scratch the state reuses.
  - The golden `b3_contact.tsv` holds 450 single steps with contacts. It covers every shape family under both
    broadphases, stacks, restitution and the combine rules, friction, rolling, sensors and layers, kinematics, sleep and
    wake, the solver settings, fast bodies, joints with contacts and 40 seeded scenes. 449 match at 0 ulp. In the other,
    glibc misrounds one friction `hypot`, and the port matches the corrected oracle (`b3_contact.cr.tsv`). The 333
    `b3_step` rows now run through `step` too, still all 0 ulp. `validate.py` transcribes the contact stages and
    reproduces every row.
  - 23 assertions in `tests/b3_contacts.tcyr`: the 10 Rust stepping tests with contacts, plus Cyrius-only groups for
    the broadphases agreeing bit for bit, the event sequence, sensors and layers, sleep and wake, and a flat heap.
    `tests/b3_joints.tcyr` runs on `step`, and `tests/step3.cyr` is gone.
  - **ADR 0008 #5, which part 1 missed**: the 3D `add_joint` and `add_joint_at` now refuse Revolute and Prismatic joints
    and motors with `Err(IM_ERR_UNSUPPORTED)`. `b3_state.tsv` (414 scripts) maps three
    `b3::state_script_unsupported` scripts, and its seeded scripts no longer draw those joints.
  - Surface parity is 637/637 across 29 modules.
  - **3D: 29,214 ok, 0 failed, 11 accepted (ADR 11: 5, ADR 0008: 6)**; 2D unchanged at **98,873 ok**.
    The same on x86_64 ELF, x86_64 PE and aarch64.
- **Cyrius port, M5 part 4: the 3D world, closing M5** (`docs/development/port-parity.md` §23):
  - `src/world.cyr` is now shared by both profiles, as port plan §4 step 3 intended. Each profile's own world file
    comes before it with the storage over its backend, the constructor, the sub-step, the particle collisions and the
    overlap queries: `src/b2_world.cyr` (M3's 2D code, moved unchanged) and the new `src/b3_world.cyr`. The 3D world
    is Rust's 3D `PhysicsWorld`:
    - every sub-step runs sequential impulses, whatever the config's solver kind (`ImPhysicsWorld_try_new` still
      refuses XPBD, ADR 0008 #6);
    - particles test every non-sensor collider with no bounding-box pre-check, against balls, boxes, capsules and
      segments, as Rust's `particle_vs_collider_3d` does (no contact with hulls, meshes or heightfields);
    - `overlap_sphere` and `overlap_aabb` return no colliders, as Rust's 3D world does.

    Each collider's centre is computed once a step rather than for every particle, the same value from the same
    operations.
  - The golden `b3_world.tsv` holds 190 world steps in 3D. It covers emitters, fields off the z = 0 plane, every 3D
    shape (turned, offset, sensor, hull), particles streamed along z into faces, tumbling bodies spun about skew axes,
    a kinematic paddle, sub-emitters, joints, contacts beside particles, sub-steps and 30 seeded scenes. **All 190 at
    0 ulp** on the first run, and none needs a correction. `validate.py` transcribes the 3D particle collisions and
    reproduces every row.
  - The tests Rust runs on both backends now run on both profiles from one source: `tests/world_cases.cyr` (the
    world and particle tests) and `tests/regression_cases.cyr` (the 9 integration and 12 shared regression tests).
    `tests/world.tcyr` and `tests/regressions.tcyr` run them on 2D, with the 14 2D-only regressions. The new
    `tests/b3_world.tcyr` (74 assertions) and `tests/b3_regressions.tcyr` (29) run them on 3D, with Cyrius-only groups
    for the 3D particle contacts, sensors, hulls and offset colliders, the empty overlaps, the unread solver kind, the
    world against the backend's own step and a flat heap, and with Rust's 3D-only `substeps_are_honoured_in_3d`.
  - The oracle's world-state tokens moved to the shared `dumpcore.rs`, and the parity harness's to
    `tests/parity_types.cyr`.
  - Surface parity is 637/637 across 29 modules, with the 3 snapshot functions pending M6.
  - **3D: 29,404 ok, 0 failed, 11 accepted (ADR 11: 5, ADR 0008: 6)**; 2D unchanged at **98,873 ok**.
- **Cyrius port, ADR 0008 #4: the full 3D body state** (`docs/development/port-parity.md` §24; the ADR's amendment):
  - The 3D profile's `ImBodyState` carries the full orientation and angular velocity: `rotation` is an `ImQuat`
    (x, y, z, w) and `angular_velocity` an `ImVec3`. The 2D profile's is unchanged. Each profile defines its own
    `ImBodyState` (`src/b2_types.cyr`, `src/b3_types.cyr`) and its JSON (`src/b2_json.cyr`, `src/b3_json.cyr`).
  - `get_body_state` copies the body's quaternion and angular velocity; Rust 1.4.0 returns 2·atan2(q.z, q.w) and the
    spin about z. `set_body_state` stores them as given (a unit quaternion is expected), wakes the body and refreshes
    its world inverse inertia.
  - The 3D JSON document writes `"rotation":[x,y,z,w]` and `"angular_velocity":[x,y,z]`. Rust 1.4.0's 3D document is
    refused.
  - **Migrating a 3D consumer:** where Rust's state had angle θ and spin ω, pass
    `ImQuat_from_rotation_z(θ)` and `ImVec3_new(0.0, 0.0, ω)`. If only the angle about z is wanted, it is
    2·atan2(q.z, q.w), as Rust computed it. `BodyDesc` is unchanged.
  - `b3_state.tsv`'s gets and sets now compare the full state, from the oracle's dump, all 0 ulp.
    `tests/b3_state.tcyr` has 34 assertions (+6: arbitrary orientations, the inertia refresh, the 3D JSON).

- **Cyrius port, M6 part 1: 2D snapshots, the struct and `restore`** ([ADR 0013](docs/adr/0013-complete-snapshots.md);
  `docs/development/port-parity.md` §25):
  - `ImWorldSnapshot` holds a 2D world's whole state: the config and id counters, particles, emitters and force fields,
    each arena's slots and free list, every body, collider and joint record (the backend's own types), each body's
    collider list, the contact manifolds with their warm-start impulses, the contact tracker and the last events.
    Rust 1.4.0's `WorldSnapshot` has no force fields, manifolds, sleep timers, mass properties, accumulators, arena
    metadata or tracker.
  - `world.snapshot(out)` fills one. `world.restore(snap)` checks it, builds a fresh world and swaps it in. On any
    error the world is unchanged: the config's own error, or `IM_ERR_DESERIALIZE`. Rust's `restore` and
    `try_restore` are one fallible `restore` (ADR 0008 #2). A restored world steps to the same bits as the original,
    with the same events and next handles.
  - The checks extend Rust's `validate_snapshot`: the index cap, slot words and free lists, one record per occupied
    slot at its generation, colliders on live bodies and on their lists once, joints' bodies, manifold keys and point
    counts, sorted tracker lists, enums and flags.
  - `IM_MAX_SNAPSHOT_INDEX` (2^20) is generated; its golden row, pending since M1, now replays (2D: 98,874 ok, 6
    pending).
  - `tests/snapshot.tcyr` has 31 assertions:
    - Rust's `restore_rejects_huge_handle_index`, and `restore_replaces_force_fields` (`#[ignore]`d in Rust), at
      the struct;
    - exact round trips in five scenes, and step-equivalence in eight runs over contacts, sleep, every joint type,
      breaking, particles, removals, sub-steps and the AABB tree;
    - 13 refusals, each leaving the world unchanged, and a flat heap.

    Dropping the warm starts, the free-list order or the tracker on restore each fails 5–6 of them.
  - Surface parity is 662 across 30 modules: Rust's `serialize` module joins, with its snapshot records mapped onto
    ADR 0013's and `serialize_world` / `deserialize_world` pending part 2.

- **Cyrius port, M6 part 2: the binary form** ([ADR 0013](docs/adr/0013-complete-snapshots.md);
  `docs/development/port-parity.md` §26):
  - `snap.to_bytes(out)` writes version 1: little-endian 64-bit words, the same on every target. A five-word header
    (magic "IMPSNAP\0", version, profile, payload length, a 64-bit FNV-1a of the payload) leads the world half and
    the backend half. An absent optional is a 0 flag and zero words, and a payload enum writes only its variant's
    fields, so the bytes depend on the state alone.
  - `ImWorldSnapshot_from_bytes` refuses a bad length, magic, version, profile, payload length or checksum before
    reading a field. It checks every count against the words left before allocating, and refuses short reads,
    unknown kinds, flags out of range and leftover words. `serialize_world` and `deserialize_world` keep Rust's
    names. Their bytes are the port's own, since Rust's bitcode can't be read here.
  - `tests/snapshot.tcyr` grows from 31 to 49 assertions:
    - the five 2D tests of `serialize.rs` that use bytes, exact where Rust allows 1e-10;
    - `restore_preserves_handle_allocation` and `restore_replaces_force_fields` through bytes, both `#[ignore]`d
      in Rust;
    - byte round trips in five scenes, the header's refusals, every truncation, a one-bit change in any word;
    - 1,500 seeded corruptions behind a recomputed checksum (Rust's `fuzz_serialize` contract: refused with the
      world unchanged, or restored and stepping);
    - a pinned checksum, which the PE and aarch64 runs confirm.
  - Surface parity is 662 across 30 modules with nothing pending.

- **Cyrius port, M6 part 3: the JSON form** ([ADR 0013](docs/adr/0013-complete-snapshots.md);
  `docs/development/port-parity.md` §27):
  - `snap.to_json(sb)` and `ImWorldSnapshot_from_json` write and read one object. Format, version and profile lead,
    then the struct's fields. Records are objects keyed by their field names, and the shared types nest through
    their serde-compatible codecs (ADR 0010). It has no checksum, being meant for reading and editing; `restore`
    checks it as any snapshot.
  - Finite values come back bit for bit. JSON has no non-finite number, so `to_json` refuses one with
    `IM_ERR_SERIALIZE`; the binary form holds it. The float printer now counts the values it writes as `null`.
  - The particle, emitter and event readers in `src/json.cyr` were split so they can nest (`_im_jr_particle`,
    `_im_jr_emitter`, `_im_jr_event`); the json corpus is unchanged.
  - `tests/snapshot.tcyr` grows from 49 to 58 assertions:
    - Rust's `snapshot_serde_json`;
    - JSON round trips in five scenes (the same state, the same text again);
    - step-equivalence through JSON, and an edited document restoring with its edit;
    - six reader refusals and one by `restore`;
    - a NaN refused by `to_json`.

- **Cyrius port, M6 part 4: 3D snapshots** ([ADR 0013](docs/adr/0013-complete-snapshots.md);
  `docs/development/port-parity.md` §28):
  - `src/b3_snapshot.cyr` gives the 3D profile all three forms: the struct over the 3D backend's own records (the full
    orientation, angular velocity and inertia tensors, ADR 0008 #4; no manifolds), the binary form (62 words a body)
    and JSON (the quaternion `[x,y,z,w]`, each matrix its columns).
  - `restore` refuses a Revolute or Prismatic joint or a motor with `IM_ERR_UNSUPPORTED`, as the 3D `add_joint` does
    (ADR 0008 #5).
  - Rust runs its snapshot tests on both backends from one source, and so does the port: `tests/snapshot_cases.cyr`
    holds Rust's six `serialize.rs` tests, its three restore tests and the Cyrius-only byte and JSON tests (30
    assertions). `tests/snapshot.tcyr` (2D, 58 assertions) and the new `tests/b3_snapshot.tcyr` (3D, 59) add their
    own:
    - in 3D, a tumbling body's full state through both forms;
    - step-equivalence in eight 3D runs, including the six supported joint types on tumbling bodies;
    - 13 refusals of edited 3D records;
    - a pin and a JSON edit.

- **Cyrius port, M6 part 5: the fuzz targets, the capture profiled, M6 closed** (`docs/development/port-parity.md` §29):
  - Rust's two fuzz targets run as seeded loops on both profiles: `tests/fuzz_cases.cyr`, driven by
    `tests/impetus.fcyr` (2D) and the new `tests/impetus_3d.fcyr` (3D) under `cyrius fuzz`. An LCG with edge-weighted
    words stands in for libFuzzer, and an optional argument multiplies the counts (`args` joins `[test]`'s stdlib).
    The checks:
    - `fuzz_contacts`: two balls from arbitrary bytes, decoded as Rust decodes them, step to finite positions and
      velocities;
    - `fuzz_serialize`: mutated and random snapshot bytes. A refused input leaves the world unchanged, and a restored
      world steps and serializes again. Mutated config documents that `try_new` accepts step;
    - a Cyrius-only target: mutated snapshot JSON. A restored world steps and serializes again.

    The default run takes about 2.5 s a profile and a scale-20 soak passed. Four faults put in the library each fail the
    default run, under `--bounds` and `--poison` as well.
  - The capture now skips the shape clone for a ball, box, capsule or segment, since the record copy has already
    copied that shape whole (`_im_snap_shape_plain`). A hull, mesh or heightfield still clones its arrays.
  - The shared `array_shapes_owned` group adds what no snapshot test held: shapes that own arrays, copied at new
    addresses into the snapshot and again into the restored world, each copy outliving the other. `snapshot.tcyr`
    has 62 assertions and `b3_snapshot.tcyr` 63 (the shared cases 34). Skipping every clone fails the group.

- **Cyrius port, M7 part 1: the Rust-test ledger** (`docs/development/port-parity.md` §30):
  - `scripts/rust_test_ledger.py` (after tanmatra's) writes `docs/development/rust-test-ledger.md`. Its `--check`
    fails on a stale ledger or a Rust test that is neither ported nor recorded with a reason. A test counts as
    ported when one of the Cyrius files mapped to its Rust file has a `test_group` of the same name.
  - The ledger covers 432 tests: rust-old's 430 `#[test]`s and the README's two examples, which are doc tests because
    `lib.rs` takes the README as the crate's docs. 388 are ported. 17 are n/a: 8 XPBD tests (ADR 0008 #6) and the 9
    tests of the oracle hooks. 27 are deferred: the bridges (port plan §3.2).
  - Groups that held several Rust tests are split, one per test: 30 3D narrowphase tests in `tests/b3_narrowphase.tcyr`
    and the 4 combine rules in `tests/material.tcyr`. The assertions are unchanged.
  - Newly ported:
    - `backend_2d/tests.rs`'s two static-friction serde tests (`tests/material.tcyr`) and its three sub-step tests
      (`tests/config.tcyr`). An old document reads into a target that starts with other values, so the reader must
      reset them;
    - `lib.rs`'s `create_default_world`, and the README's Quick Start, in the shared world cases (both profiles);
    - the README's spring example (`tests/spring.tcyr`).

- **Cyrius port, M7 part 2: every Rust benchmark has its twin** (`docs/benchmarks-rust-v-cyrius.md`):
  - All 41 of Rust's benches now have a 2D twin, and 26 a 3D one. The other 15: 14 profile-free rows that are the
    same code on both profiles, and `joints/revolute_chain_20`, which the 3D profile refuses (ADR 0008 #5). The plan
    counted 38, without Rust's three `serialize`-gated snapshot benches.
  - New 2D twins in `tests/impetus.bcyr`: `queries/raycast_100_bodies`, `queries/overlap_sphere_100_bodies`,
    `mutation/get_body_state`, `set_body_state`, `set_body_type`, `substep/step_100_bodies_substep4`,
    `stacking/box_stack_10`, `joints/revolute_chain_20` and `broadphase/aabb_tree_1000_row`. The new 3D twins are the
    overlap, `set_body_type`, the box stack and the tree row. The rows both profiles share are in the new
    `tests/bench_cases.cyr`.
  - M2's Cyrius-only tree row is renamed `broadphase/aabb_tree_1000_row (rebuild only)`, since Rust's name now belongs
    to the whole-step twin.
  - `scripts/bench_record.py` holds the mapping. Its `--check` fails when a Rust bench lacks a twin. Given Rust's and
    Cyrius's bench logs, it prints the record's table.

- **Cyrius port, M7 part 3: the 2.0 docs and a final review:**
  - **README** rewritten for the Cyrius library, on tanmatra's model: how it is checked against Rust, the features,
    the dependency declaration, Rust's two README examples as Cyrius programs, the API conventions, the two
    profiles, the known limitations, building, and the oracle checks.
  - **`docs/guides/migration-2.0.md`**: moving from the Rust crate. The Cargo features as bundles, a table of Rust
    idioms in Cyrius, and each ADR 0008 change with code that handles it.
  - **`scripts/doc_examples.py`**: the Cyrius form of Rust's doc tests. It compiles and runs every `cyrius` block in
    the README and the guides against the shipped bundles, and fails on a warning, a failed build or a nonzero exit:
    9 blocks, 9 ran. A planted syntax error and two wrong expected error codes each failed it.
  - **hisab is a test-scope dependency** (`cyrius.cyml`; [ADR 0007](docs/adr/0007-cyrius-port-hisab-first.md)'s
    second amendment). No library module calls hisab (ADR 0012), but as a production dependency it reached every
    consumer's resolve. A scratch consumer that only creates and frees a world vendored hisab and sakshi and
    built a 3,681,032-byte binary in 1.56 s; with test scope it vendors only impetus and builds 1,702,480 bytes
    in 0.55 s. impetus's own resolve, `cyrius.lock` and the bundles are unchanged, and the 3D bench's two hisab
    rows still compile.
  - The final review also rewrote `docs/guides/getting-started.md` (using the library, the layout, changing a
    module), `CONTRIBUTING.md` (the shipped library, the parity scripts), `SECURITY.md` (the bundles in scope; the
    parsed inputs and their checks as the attack surface), and the roadmap's "2.0 — Breaking Cleanup": the API
    items are done, and the new-physics items (3D islands, consistent joint frames) and shape validation are open.
    The README's links to a root `docs/architecture/overview.md` and `docs/guides/testing.md`, which never existed
    in the port, now point at the Rust copies in `rust-old/docs/`.

- **Cyrius port, M7 part 4: the 2.0.0 release:**
  - `VERSION` is 2.0.0, which `cyrius.cyml` reads. Both bundles are regenerated, and only their version line
    changed.
  - This CHANGELOG's `[Unreleased]` became `[2.0.0]`, with the Breaking, Verified and Known limitations sections
    above. The release workflow's extraction of it is 69,758 characters, within GitHub's limit.
  - CI also runs the Rust-test ledger check, the bench-twin check, `cyrius fuzz` and `scripts/doc_examples.py`.
    Its stale note about a separate 3D job is gone: `cyrius test` already runs both profiles.
  - The release gates, all green:
    - `cyrius test` passes on x86_64 ELF, x86_64 PE and aarch64, and under `--bounds`: 33 files each, no warnings;
    - the corpus on all three targets: 2D 98,874 ok, 6 pending, 166 accepted; 3D 29,404 ok, 11 accepted. The parity
      digests are equal across targets;
    - `cyrius fuzz` (also under `--bounds --poison`), `cyrius bench` (129 rows, no warnings), `cyrius audit`,
      `cyrius vet`, `deps --locked` and `distlib --check` all pass, and the build succeeds for all three targets;
    - surface parity (662), the ledger (432), the bench twins (41) and the doc examples (9) pass;
    - `validate.py` and the generator's `--check` for both profiles confirm the corpus is current, and the Rust
      oracle's own tests still pass (2D 344, 3D 306).

### Performance
- **Correctly rounded exp, sin and cos** (ADR 0011): phase 1 takes exp from 2.6 µs to 97 ns and
  sin_cos from 3.3 µs to 187 ns (the polyfills: 48–64 and 75–83 ns).
- **Broadphase, first numbers** (Cyrius only; `docs/benchmarks-rust-v-cyrius.md`): rebuilding and
  querying 1,000 boxes takes 4.7 ms (tree) and 1.95 ms (hash). Rust's whole world step on that layout
  takes 0.51 ms, so this is a finding to profile before M3.
- **2D backend types, first numbers** (Cyrius only; Rust has no bench for these alone):
  - a ball's mass, 545 ns; Rust's whole `add_body_with_collider` takes 253 ns;
  - a 16-vertex hull's mass properties, 578 ns;
  - a rotated box's world AABB, 261 ns.

  349 ns of the ball's 545 is its shape copy, mostly one 184-byte `memcpy`. The stdlib copies a
  byte at a time, and 8-byte chunks are 4–7× faster (S5, port plan §8).
- **2D narrowphase, first numbers** (Cyrius only; Rust times it inside its step benches, M3):
  ball–box 132 ns, rotated box–box 1.28 µs, a lying capsule on a box 506 ns, two 8-vertex hulls
  3.18 µs. Taking one `sin_cos` per polygon and per contact, where Rust takes four and two of the
  same angle, gave 169 → 132 ns, 1.98 → 1.28 µs, 560 → 506 ns and 3.29 → 3.18 µs. The bits are
  identical, because `sin_cos` is pure.
- **Correctly rounded hypot and the 2D queries, first numbers** (Cyrius only; Rust has no bench for
  these alone): `hypot` 168 ns, against ganita's 33 ns. A ray costs 389 ns on a ball, 433 ns on a
  rotated box, 667 ns on a capsule and 2.11 µs on an 8-vertex hull; a sphere overlap on a box costs
  872 ns. The correctly rounded `sin_cos` and `hypot` dominate.
- **2D state, first numbers against Rust 1.4.0** (`docs/benchmarks-rust-v-cyrius.md`): `add_body`
  81.7 ns → 1.20 µs, `add_remove_body` 60.9 → 618 ns, `add_body_with_collider` 213 ns → 3.23 µs,
  `add_joint` 23.4 ns → 1.22 µs, `apply_force` 3.3 → 45 ns, `apply_impulse` 1.7 → 41 ns. Arena
  records (224 B per body, 320 B per collider) move by the stdlib's byte-loop `memcpy` (S5).
- **2D joints, first numbers** (Cyrius only; Rust times its step whole): a 20-link revolute chain's
  joints take 22.5 µs a sub-step to prepare and solve (4 iterations). Its whole frame through `step`
  takes 88.2 µs, broadphase included (54.7 µs for the contact-free stages alone).
- **2D step, first numbers against Rust** (Rust's `world/step_*` scenes through the backend's `step`;
  `docs/benchmarks-rust-v-cyrius.md`):
  - empty: 438 ns against 115 ns (3.8×);
  - 10 balls: 29.0 µs against 4.31 µs (6.7×);
  - 100 balls: 374 µs against 101 µs (3.7×);
  - 1,000 balls: 4.07 ms against 853 µs (4.8×).

  A resting stack of five boxes steps in 51 µs.
- **Correctly rounded pow**: 755 ns a call (`math/pow_cr`), against ganita's 200 ns. The first design,
  a double-double phase alone, took 5.4 µs.
- **The world and particles against Rust** (`docs/benchmarks-rust-v-cyrius.md`):
  - `world/step_*` through the world: 3.5–6.5×.
  - `particles/step_100_particles` 9.9 µs against 1.95 µs, and `step_100_particles_with_floor` 1.89 µs
    against 444 ns.
  - `step_1000_particles` (93 µs against 5.5 µs), `spawn_particle` (694 ns against 20 ns) and
    `emitter_step` (18.5 µs against 493 ns) are 17–37×. Each 136-byte particle record moves by the
    stdlib's byte-loop `memcpy` (S5) on push and on every removal.
- **3D math and types, first numbers** (Cyrius only; Rust times the 3D backend only through its world,
  M5). Writing the composite operations with f64 locals, in glam's order (identical bits), took:
  - a quaternion rotation from 128 to 41 ns;
  - a 3×3 product from 297 to 86 ns;
  - a 3×3 inverse from 213 to 50 ns;
  - `integrate_velocities` from 1.95 to 1.10 µs.

  hisab's allocating `hquat_rotate_vec3` and `m3_mul` take 51 and 89 ns. A box's mass recompute takes
  1.07 µs and a rotated box's world AABB 1.23 µs. A by-value struct is copied on entry, so chains of
  small helper calls dominate (`port-parity.md` §4.2).
- **The 3D narrowphase, first numbers against Rust** (both timed per pair through `generate_contact_3d`; Rust
  through the oracle hook in a scratch timing, since Rust benches its narrowphase only inside a step):
  - the closed forms: 0.24–2.5 µs against 41–450 ns (6–20×);
  - the OBB SAT: 8.3 µs against 580 ns (face) and 6.3–8.0 µs against 465 ns (edge pair), down from 14.6 and
    12.6 µs once its overlap test, clip and hull support read f64 words (identical bits);
  - GJK + EPA on hulls: 30–55 µs a pair (one long batch, noisy) against 4.8–5.5 µs.
- **The 3D raycast against Rust**: `queries/raycast_100_bodies` on the 3D build takes 21.1 µs a ray against Rust's
  2.28 µs (criterion, `--all-features`), 9.3×. Alone, a ray test costs 141 ns (box), 220 ns (ball) and 630 ns
  (capsule).
- **Correctly rounded atan2 and the 3D state against Rust** (`docs/benchmarks-rust-v-cyrius.md`; Rust's benches run
  on its 3D build):
  - `math/atan2_cr` 634 ns, against ganita's 62 ns; `b3/get_body_state` 338 ns;
  - `bodies/add_body` 113 ns → 2.42 µs, `add_remove_body` 66.5 ns → 1.20 µs, `add_body_with_collider` 309 ns →
    5.56 µs, `add_joint` 32.4 ns → 1.56 µs;
  - `forces/apply_force` 7.48 → 57 ns, `apply_impulse` 2.34 → 40 ns.

  A 3D body record is 496 B against 2D's 224 B, and the arenas copy records with the byte-loop `memcpy` (S5).
- **The 3D joints and the contact-free step, first numbers** (Cyrius only; Rust times its 3D step whole, broadphase
  included, so the comparison comes with part 3's `step`): a tumbling box's gyroscopic velocity step 1.21 µs; one
  sub-step's joint work on a 20-link chain 59.9 µs (3 µs a joint); that chain's whole contact-free frame 97.4 µs.
- **The 3D step against Rust, and a faster broadphase for both profiles** (`docs/benchmarks-rust-v-cyrius.md`;
  measured with nothing else running, Rust on its 3D build):
  - `world/step_*` in 3D: empty 68.7 ns → 389 ns, 10 bodies 4.97 → 54.6 µs, 100 bodies 116 → 649 µs, 1,000 bodies
    1.07 → 5.84 ms; `substep/step_100_bodies_substep4` 554 µs → 6.27 ms.
  - The spatial hash's record sort now moves words instead of byte-copying 24-byte records (S5), and its pair sort
    compares inline. The orders are unchanged. `broadphase/spatial_hash_1000_row` 1.98 ms → 588 µs; the 3D
    `step_1000_bodies` 9.07 → 5.84 ms; the 2D `b2/step_1000_bodies` 3.76 → 2.33 ms and `world/step_1000_bodies`
    3.89 → 2.48 ms.
- **The 3D world against Rust** (`docs/benchmarks-rust-v-cyrius.md`; measured with nothing else running, Rust on its
  3D build):
  - `world/step_*` now through the world: empty 70.5 → 424 ns, 10 bodies 4.96 → 56.3 µs, 100 bodies 115 → 663 µs,
    1,000 bodies 1.08 → 6.00 ms; `substep/step_100_bodies_substep4` 550 µs → 6.17 ms. The world adds about 35 ns a
    frame over the backend.
  - `particles/*` on the 3D world: `step_100_particles` 1.90 → 9.90 µs, `step_1000_particles` 6.39 → 94.0 µs,
    `spawn_particle` 19.9 → 714 ns, `emitter_step` 505 ns → 18.5 µs, as in 2D.
  - `step_100_particles_with_floor` 338 ns → 2.85 µs (8.4×, 2D's is 4.3×). Its particles soon expire, so it times a
    frame with one static box, and 1.84 µs of that frame is the 3D broadphase's rebuild for the one box. Recorded as a
    finding.
- **The full 3D body state against Rust** (ADR 0008 #4; quiet machine, Rust on its 3D build): `mutation/get_body_state`
  11.5 → 40 ns, down from part 1's 338 ns (a copy, no `atan2`); `mutation/set_body_state` 26.9 → 379 ns, 304 ns of it
  the R·I⁻¹·Rᵀ refresh.
- **Snapshots, first numbers** (Cyrius only, the struct; Rust's `snapshot/*` rows go through bytes, part 2): on Rust's
  scenes a capture of an empty world takes 1.39 µs, of 100 balls 208 µs, and a restore of those 100 into a fresh world
  234 µs. Not profiled yet.
- **Snapshots against Rust** (quiet machine; Rust's bitcode on its 2D build): `snapshot/serialize_empty` 1.35 →
  2.24 µs, `serialize_100_bodies` 14.5 → 282 µs, `roundtrip_100_bodies` 33.6 → 357 µs. The port's snapshot holds the
  whole state (49 words for a ball and its body), and the struct capture is 208 µs of the 282; not profiled yet.
- **The snapshot's JSON form, first numbers** (Cyrius only): 100 balls written in 2.31 ms and read back and restored
  in 3.70 ms, about 0.8 µs per shortest-form float (29 a ball); the binary form is 8–10× faster.
- **3D snapshots against Rust** (quiet machine, Rust's bitcode on its 3D build): `snapshot/serialize_empty` 1.23 →
  2.13 µs, `serialize_100_bodies` 13.7 → 396 µs, `roundtrip_100_bodies` 37.2 → 478 µs. A 3D body record is 496 B.
- **Snapshots, profiled** (M6 part 5; quiet machine): a probe split the 2D capture of Rust's 100 balls (205 µs). The
  body and collider records through the byte-loop `memcpy` took 87 µs (12 µs as 8-byte copies, S5). The shape clones
  took 63 µs, the arena slots 7 µs and the collider lists 4 µs. Skipping the clone for shapes without arrays cuts the
  struct capture from 210 to 145 µs (−31%) and the restore from 235 to 166 µs (−29%). Against Rust:
  - 2D: `serialize_100_bodies` 283 → 221 µs (19× → 15×), `roundtrip_100_bodies` 359 → 286 µs (11× → 8.5×);
  - 3D: `serialize_100_bodies` 390 → 330 µs (29× → 24×), `roundtrip_100_bodies` 481 → 416 µs (13× → 11×).

  The JSON rows are unchanged within noise.
- **The full benchmark record** (M7 part 2; quiet machine, Rust's 2D and 3D builds): Cyrius's median ratio to Rust is
  10× in 2D (geometric mean 9.4×, 41 rows) and 13× in 3D (12×, 26 rows). Rust's whole-step rows are closest: 2.6–11× in
  2D and 5.7–17× in 3D. The widest gaps are its smallest bodies: `add_joint` 24 ns → 1.14 µs, and `spawn_particle`
  20 ns → 711 ns. The AABB-tree step (`broadphase/aabb_tree_1000_row`, 517 µs → 5.69 ms) is mostly the tree's
  rebuild, 4.73 ms on its own (M2's open finding).
- **Cyrius port, M1b** (first numbers, `docs/benchmarks-rust-v-cyrius.md`; Rust 1.4.0 → Cyrius):
  `spring_1d_step` 4.98 → 28.5 ns (5.7×), `spring_2d_step` 6.04 → 59.5 ns (9.9×),
  `spring_settle_frames` 1.47 → 1.80 µs (1.2×), then 1.86 µs (1.26×) on the correctly rounded exp; `quantity_display` 81 → 830 ns (10×); serde
  `serialize_body_desc` 252 ns → 3.5 µs (14×), `deserialize_body_desc` 276 ns → 7.2 µs (26×),
  `serialize_config` 422 ns → 8.9 µs (21×), `roundtrip_collider_desc` 892 ns → 16.1 µs (18×).

### Removed
- Committed fuzz build output (`fuzz/target/`, 208 files). It is now ignored under
  `rust-old/`.

## [1.4.0] - 2026-11-09

Audit and repair sweep. Simulation results change: most items below fix
physically wrong behaviour, so replays recorded with 1.3 will diverge.

### Changed
- **toolchain** — Rust 1.89 → 1.99.1 (`rust-toolchain.toml`); MSRV 1.89 → 1.99; edition 2024
- **deps** — hisab 1.3 → 1.4; serde 1.0.229, serde_json 1.0.154, thiserror 2.0.21 (lockfile); CI actions checkout v7, codecov-action v7, action-gh-release v3
- **ci** — every backend combination is linted and tested (`--all-features` compiles only the 3D backend, so the 2D backend and its tests were never run in CI); coverage merges 2D and 3D runs; README code blocks are doctested
- **world** — `sub_steps` now applies to the 3D backend too; forces applied before `step()` act over the whole step (were cleared after the first sub-step); collision events are diffed once per step (Started was lost with sub-steps), and a contact that starts and ends within one step reports Started then Stopped
- **joints (2D)** — velocity-level constraint solver with angular terms, soft springs, clamped motors and limits, interleaved with contacts ([ADR 0005](docs/adr/0005-velocity-level-joints.md)); Fixed and Prismatic hold their creation-time relative rotation; prismatic/wheel axes are in body A's frame; `break_force` is the linear constraint force in newtons; generic joint `damping` acts on relative angular velocity (revolute, wheel), along the axis (prismatic) or along the anchors (distance, rope)
- **joints (3D)** — Fixed, Distance, Rope and Wheel are velocity-level, mass-weighted constraints; spring/wheel/mouse/damping forces apply once per sub-step (were multiplied by the iteration count); Fixed also locks rotation; Wheel constrains both axes perpendicular to the suspension (z was ignored); `break_force` compares the reaction force
- **friction (3D)** — two tangent directions with their own effective mass, clamped to μ·N; static friction holds when the impulse needed to stop slipping fits within μ_s·N
- **manifolds (2D)** — rebuilt each frame with impulses carried over from matching points, instead of accumulating stale points ([ADR 0006](docs/adr/0006-manifold-rebuild-per-frame.md))
- **sleep (2D)** — islands sleep atomically and no longer merge through static or kinematic bodies; sleeping bodies are immovable in the solver and are woken when a contact begins or a neighbour is removed
- **spring** — `Spring::step` uses the exact closed-form solution (was semi-implicit Euler, which diverged for `k·dt` large enough, e.g. a 0.5 s frame hitch); one long step equals many short ones; the step's transition matrix is cached while `dt`, stiffness and damping are unchanged; non-positive/NaN `dt` is ignored; negative stiffness in the presets is treated as its magnitude (was a debug-only panic)
- **particles** — drag is quadratic as documented (was linear); emitter spread uses an R3 low-discrepancy sequence (x and y offsets were ~perfectly anti-correlated); radial fields are softened (ε = 0.01) so they stay finite near the centre
- **mass properties** — collider offsets contribute via the parallel-axis theorem (the body origin is the centre of mass); capsule inertia includes the cap offsets; 2D convex hulls and tri-meshes use their real area and inertia, 3D hulls/meshes their bounding volume (were fixed at 1 m²/m³)
- **bridge** — `youngs_to_bulk_modulus` returns `f64::INFINITY` at ν ≥ 0.5 (incompressible) instead of 0 or a negative modulus
- **broadphase** — dynamic AABB tree rebalances (AVL rotations) and traverses iteratively; the spatial hash keeps huge/non-finite AABBs out of the grid
- **docs** — README (quick start compiles, bitcode not bincode, feature list), architecture overview, testing guide and roadmap brought up to date

### Added
- **config** — `WorldConfig::validate()` with `MAX_SUB_STEPS` / `MAX_ITERATIONS`
- **world** — `PhysicsWorld::try_new(config)`; `PhysicsWorld::try_restore(&snapshot)` validates a snapshot (config, unique handles, handle index ≤ `PhysicsWorld::MAX_SNAPSHOT_INDEX`, colliders/joints on existing bodies) before replacing state; `serialize::deserialize_world` uses it
- **narrowphase (2D)** — Capsule vs ConvexHull, Capsule vs Segment, Segment vs ConvexHull contacts; two-point manifolds for capsules lying on boxes and for boxes resting on segments
- **raycast** — 2D rays hit segments and convex hulls; 3D rays hit capsules
- **tests** — `tests/regressions.rs` (30 tests, one per fixed defect, run on whichever backend is built) plus ~60 new unit tests; unit tests went from 271 → 298 (2D build) and 229 → 274 (3D build)
- **benches** — `stacking/box_stack_10`, `joints/revolute_chain_20`, `broadphase/aabb_tree_1000_row`
- **fuzz** — `fuzz_serialize` now fuzzes snapshot decoding and validated config parsing

### Deprecated
- **world** — `PhysicsWorld::restore` (silently ignores invalid snapshots); use `try_restore`

### Fixed
- **narrowphase** — Ball/Capsule/Segment vs Box normals were inverted in both argument orders (2D and 3D), so balls fell through box floors and box contacts never bounced; these pairs also ignored box rotation (2D)
- **narrowphase (2D)** — polygon clipping used inward normals, so ConvexHull pairs generated no contacts and rotated boxes got a single point; axis-aligned box contacts sat at the centre of A's face; a circle centred inside a hull got an inward normal (or no contact); hulls wound clockwise misbehaved
- **narrowphase (3D)** — GJK gave up on collinear initial simplices; EPA returned None at its iteration cap; OBB SAT skipped the 9 edge-edge axes (phantom contacts); box contact points caused spurious torque; segment closest points were wrong when clamped or parallel; capsule-box missed end-cap contacts; hull-sphere treated the hull as a polyline
- **solver** — restitution was recomputed from the live velocity each iteration (an elastic ball kept approaching at −10 m/s); the block solver skipped the normal impulse when ill-conditioned and dropped coupling terms in its clamping cases; warm-start scaling wasn't stored back; Baumgarte correction was missing `/dt` (≈60× too weak) and every contact point re-applied the full correction; rolling friction scaled with iterations × points; friction clamp could panic on a negative/NaN coefficient
- **solver (3D)** — body-space inverse inertia was used as world-space (wrong angular response for rotated bodies); friction used the normal effective mass and ignored static friction
- **bodies** — `set_body_type(Dynamic)` on a body created static/kinematic left it with zero mass (frozen); sleeping bodies kept drifting on residual velocity
- **joints** — motors panicked (`clamp`) on negative or NaN `max_force`; joint damping could reverse velocities
- **queries** — 2D/3D raycasts and 2D overlap queries ignored box rotation; rays starting inside a box reported the entry normal; a zero ray direction cast along +Y in 3D
- **particles** — an infinite emitter rate hung `step()` (spawns are capped per step); fast particles tunnelled into boxes once their centre was inside; 3D particle collisions ignored body rotation; 2D particle collisions used a stale position for later colliders
- **snapshots** — a hostile snapshot could request a ~4 billion-slot allocation, double-count mass with duplicate handles, or insert orphan colliders; restore was O(n²) in the number of entities
- **broadphase** — the AABB tree degenerated into a list for sorted inserts (O(n²) pairs, recursion depth n); a NaN or infinite AABB made the spatial hash loop over ~4 billion cells
- **removal (2D)** — removing a body, collider or joint now wakes what was resting on it
- **library code** — remaining `unwrap()` / `unreachable!()` / `debug_assert!` in library paths replaced

### Known limitations (planned for 2.0)
- Snapshots don't capture force fields, contact manifolds, sleep state, arena free-list order, or 3D orientation beyond Z — see the roadmap's 2.0 section
- 3D Revolute, Prismatic and motors are accepted but have no effect; 3D has no islands
- XPBD (2D only) lacks prismatic/wheel/mouse joints, motors, limits and breaking

### Performance
Measured with `./scripts/bench-history.sh` on the same machine, baseline = this
release's toolchain/dependency update before any repairs (2026-11-09T08:48Z),
final = 2026-11-09T09:49Z. (The 1.99 toolchain itself was checked against 1.89
on this machine and benchmarks the same.)

- world/step_empty: 140.3 ns → 114.9 ns (−18%)
- queries/raycast_100_bodies: 1.06 µs → 0.96 µs (−9%), with rotated boxes, segments and hulls now hittable
- particles/step_1000_particles: 6.46 µs → 5.49 µs (−15%)
- springs/spring_settle_frames: 1.77 µs → 1.47 µs (−17%); spring_1d_step 4.95 ns → 4.98 ns (exact step at Euler cost via the cached transition)
- world/step_100_bodies 99.6 µs → 100.6 µs, step_1000_bodies 844 µs → 853 µs (≈ +1%, noise level)
- **Slower, by design:** substep/step_100_bodies_substep4 455 µs → 628 µs (+38%) — the balls in this bench used to fall through the box floor (inverted normals) and now come to rest on it, so the step solves a full contact pile; mutation/set_body_type 4.3 ns → 14.1 ns (recomputes mass instead of leaving a body frozen); springs/spring_2d_step 5.0 ns → 6.0 ns; queries/overlap_sphere_100_bodies 1.25 µs → 1.34 µs (+8%, precise rotated tests)
- New: stacking/box_stack_10 10.5 µs, joints/revolute_chain_20 14.3 µs, broadphase/aabb_tree_1000_row 506 µs (the unbalanced tree was O(n²) for sorted inserts)

## [1.3.0]

### Added
- **bridge** — sharira: muscle force to joint torque, cylinder inertia for bone segments, gait GRF to force vector, body CoM gravity force, joint stiffness/damping to restoring torque

## [1.2.0]

### Added
- **bridge** — cross-crate primitive-value bridges for dravya (impact stress, strain rate, bulk modulus, damage volume), sharira (joint torque to force, angular to tangential velocity, segment inertia, limb force), ushma (friction heat rate, collision temperature rise, thermal strain)

### Updated
- hisab 1.1.0 -> 1.3.0, zerocopy 0.8.47 -> 0.8.48

## [Unreleased-pre-1.2]

### Added

- **narrowphase** — one-shot manifold generation via Sutherland-Hodgman edge clipping for OBB-OBB and ConvexHull-ConvexHull; produces full contact manifold in a single pass instead of accumulating over frames
- **narrowphase** — contact point reduction via area maximization; reduces large manifolds to 4 optimal points for solver efficiency
- **3D solver** — gyroscopic torque (`ω × (I·ω)`) precession term in 3D velocity integration; required for realistic tops, wheels, projectiles, and fast-spinning bodies
- **config** — `SolverKind` enum (`SequentialImpulse`, `Xpbd`) with `#[non_exhaustive]`; `WorldConfig::solver` field (default: `SequentialImpulse`)
- **solver** — XPBD (Extended Position-Based Dynamics) solver for 2D backend; position-level constraint solving with compliance (α̃ = 1/(ω²·dt²)), Lagrange multiplier accumulation, velocity-level friction and restitution, position-level joint constraints (Fixed, Revolute, Distance, Spring, Rope)
- **manifold** — `MAX_MANIFOLD_POINTS` increased from 2 to 4 in 2D for edge-clipping contact quality

### Fixed

- **solver** — division-by-zero guards added to single-point normal impulse, block solver re-solve cases, and friction effective mass computation
- **solver** — replaced `expect()` panics with defensive `if let` checks in manifold impulse accumulation and friction solver
- **narrowphase** — convex hull circle contact now computes actual edge outward normal when circle center coincides with hull edge, instead of using arbitrary `[0, 1]` fallback
- **aabb_tree** — replaced `expect()` panics with graceful fallbacks in `update()` and `query_pairs()`
- **narrowphase** — manifold point replacement uses `unwrap_or` for NaN-safe comparison
- **narrowphase** — `reduce_contacts()` area maximization now uses `first_unselected()` fallback to prevent duplicate index selection
- **narrowphase** — epsilon guard added to Sutherland-Hodgman clip plane intersection to prevent division by near-zero
- **solver** — XPBD compliance calculation guarded against zero timestep (`dt > EPSILON`)
- **state** — replaced remaining `expect()` calls in `add_body`/`add_collider` with defensive `if let`

### Changed

- **spring** — `critically_damped()`, `over_damped()`, `under_damped()` factory functions use `stiffness.abs().sqrt()` with `debug_assert!` on negative stiffness to prevent NaN
- **particle** — `Particle::with_radius()` clamps to non-negative with `debug_assert!` validation
- **world** — added `#[must_use]` to `raycast()`, `raycast_filtered()`, `overlap_sphere()`, `overlap_aabb()` query methods

## [1.0.0] — 2026-03-25

v1.0 release — solver quality, broadphase, collision, and inertia overhaul.

### Added

- Split impulse position correction via pseudo-velocities — prevents energy injection from positional correction leaking into real velocities
- Block solver for 2-point manifolds — coupled 2x2 normal impulse solve with four-case clamping (Box2D-style), improves box stacking stability
- Shock propagation — BFS bottom-up contact ordering so bodies closer to static geometry are solved first, stabilizes deep stacks
- Soft constraints via ERP/CFM spring-damper formulation — configurable `WorldConfig::constraint_frequency` (Hz, default 30) and `WorldConfig::constraint_damping_ratio` (default 1.0 critically damped); set frequency to 0 for legacy Baumgarte
- GJK + EPA narrowphase — native f64 implementation for general convex-convex 3D collision; handles ConvexHull vs Box, ConvexHull vs ConvexHull, ConvexHull vs Capsule
- Speculative contacts — broadphase AABBs expanded along velocity to detect contacts before penetration, preventing tunneling for fast-moving objects
- Dynamic AABB tree broadphase — balanced BVH with surface-area heuristic insertion and fattened AABBs; better for heterogeneous object sizes
- `BroadphaseKind` enum (`SpatialHash`, `AabbTree`) with `#[non_exhaustive]`
- `WorldConfig::broadphase` field for broadphase algorithm selection (default: `SpatialHash`)

### Changed

- **Breaking:** 3D inertia tensor upgraded from diagonal `DVec3` to full `DMat3` — enables accurate angular response for asymmetric meshes, compound shapes, and off-axis colliders; angular impulse application uses matrix-vector multiply throughout
- **Breaking:** `PhysicsState2d::step()` and `PhysicsState3d::step()` signatures extended with `constraint_frequency`, `constraint_damping_ratio`, and `broadphase_kind` parameters
- Position solver uses split impulse pseudo-velocities instead of direct position mutation
- 2D solver friction and rolling friction extracted into shared `solve_contact_friction()` method
- `hisab` dependency upgraded from 0.22.4 to 1.1
- `criterion` dev-dependency upgraded from 0.5 to 0.8
- Benchmarks use `std::hint::black_box` instead of deprecated `criterion::black_box`

## [0.23.3] — 2026-03-23

Initial public release.

### Added

- 2D and 3D rigid body simulation backends (feature-gated: `2d` default, `3d` optional)
- Sequential impulse constraint solver with Coulomb friction and angular response
- Warm starting with accumulated impulse clamping
- Persistent multi-point contact manifolds (up to 2 per pair in 2D)
- Simulation islands via union-find with atomic sleep/wake per island
- Static vs dynamic friction with configurable `static_friction` on materials
- Sub-stepping via `WorldConfig::sub_steps`
- Restitution velocity threshold to prevent micro-bouncing of resting objects
- Configurable Baumgarte positional correction (`position_slop`, `position_correction`)
- CCD via `WorldConfig::max_velocity` clamp
- Spatial hash broadphase with collision layer/mask filtering
- 2D narrowphase: circle, AABB, OBB (rotation-aware SAT), capsule, ConvexHull, segment
- 3D narrowphase: sphere, OBB, capsule-sphere, capsule-box, capsule-capsule, segment-sphere, segment-box, convex hull-sphere
- Raycasting with hit point/normal for circle, AABB, and capsule shapes
- `raycast_filtered()` with collision layer mask
- Overlap queries: `overlap_sphere()`, `overlap_aabb()`
- Collision events: `Started`, `Stopped`, `Ongoing` with contact data
- Sensor colliders (events only, no physical response)
- Body types: Static, Dynamic, Kinematic
- `set_body_state()`, `set_body_type()` for runtime mutation
- `remove_body()`, `remove_collider()`, `remove_joint()` by handle
- Mass/inertia accumulation across multiple colliders per body
- Sleep/deactivation with velocity threshold and island-based atomic wake
- Generational arena storage for O(1) body/collider/joint access
- 8 joint types: Fixed, Revolute, Prismatic, Spring, Distance, Wheel, Rope, Mouse
- Joint motors (revolute/prismatic) with `target_velocity` and `max_force`
- Joint damping and breaking via `break_force` threshold
- Joint limits on Revolute and Prismatic
- Material presets: `ice()`, `rubber()`, `wood()`, `steel()`, `bouncy()`
- Rolling friction and static friction fields on `PhysicsMaterial`
- Friction/restitution combine rules: `Min`, `Average`, `Multiply`, `Max`
- Physics particles with gravity, drag, damping, lifetime, collider interaction
- Particle emitters with rate-based spawning and golden-ratio spread
- Radial force fields (attraction/repulsion with configurable falloff)
- Directional force fields (wind zones with AABB regions)
- Sub-emitters (spawn children on particle death)
- Standalone `Spring`, `Spring2d`, `Spring3d` types (no world required)
- Spring presets: `critically_damped()`, `over_damped()`, `under_damped()`
- Spring settle detection, snap, fling, retarget
- `WorldSnapshot` with `snapshot()`/`restore()` on `PhysicsWorld`
- `serialize_world()`/`deserialize_world()` via bitcode (feature-gated: `serialize`)
- Step-exact determinism verified via serialize-restore-continue tests
- `Quantity` type with 14 `PhysicsUnit` variants and display formatting
- `#[non_exhaustive]` on all public enums
- `#[must_use]` on pure functions and accessors
- `Send + Sync` compile-time assertions on all public types
- BTreeMap/BTreeSet for deterministic iteration order
- 35 benchmarks across 10 groups with three-point history tracking
- Fuzz testing targets for contact generation and serialization
- CI: check, security audit, cargo-deny, test (Linux/macOS/Windows), MSRV 1.89, coverage, doc, semver
- Supply-chain hardening: cargo-vet config, cargo-deny strict mode

[Unreleased]: https://github.com/MacCracken/impetus/compare/1.4.0...HEAD
[1.4.0]: https://github.com/MacCracken/impetus/compare/1.3.0...1.4.0
[1.3.0]: https://github.com/MacCracken/impetus/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/MacCracken/impetus/compare/1.1.0...1.2.0
[1.0.0]: https://github.com/MacCracken/impetus/compare/0.23.3...1.0.0
[0.23.3]: https://github.com/MacCracken/impetus/releases/tag/0.23.3
