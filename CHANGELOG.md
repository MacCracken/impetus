# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

The Cyrius port (2.0.0) has begun; the plan of record is
`docs/development/impetus-port-plan.md`. Rust 1.4.0 is frozen in `rust-old/` as
the parity oracle, together with the port-tooling changes below. Those are not
scheduled for a Rust release; they exist so the port's golden-value generator
can drive the oracle (M0 step 0), and simulation behaviour is unchanged.

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
