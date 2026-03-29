# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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

[Unreleased]: https://github.com/MacCracken/impetus/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/MacCracken/impetus/compare/v0.23.3...v1.0.0
[0.23.3]: https://github.com/MacCracken/impetus/releases/tag/v0.23.3
