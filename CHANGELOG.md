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
