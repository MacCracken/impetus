# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/).

## [0.23.3] — 2026-03-23

Initial public release.

### Added

#### Physics Engine
- 2D and 3D rigid body simulation backends (feature-gated: `2d`, `3d`)
- Sequential impulse constraint solver with Coulomb friction and angular response
- Warm starting with accumulated impulse clamping
- Persistent multi-point contact manifolds (up to 2 per pair in 2D)
- Simulation islands via union-find (atomic sleep/wake per island)
- Static vs dynamic friction with configurable `static_friction` on materials
- Sub-stepping (`sub_steps` on `WorldConfig`)
- Restitution velocity threshold (prevents micro-bouncing)
- Configurable Baumgarte positional correction (`position_slop`, `position_correction`)
- CCD via `max_velocity` clamp on `WorldConfig`

#### Collision Detection
- Spatial hash broadphase with collision layer/mask filtering
- Narrowphase: circle, AABB, OBB (rotation-aware SAT), capsule, ConvexHull, segment
- 3D narrowphase: sphere, OBB, capsule-sphere, capsule-box, capsule-capsule, segment-sphere, segment-box, convex hull-sphere
- Raycasting with hit point/normal (circle, AABB, capsule)
- `raycast_filtered()` with collision layer mask
- Overlap queries: `overlap_sphere()`, `overlap_aabb()`
- Collision events: `Started`, `Stopped`, `Ongoing`
- Sensor colliders (events only, no physical response)

#### Bodies & Colliders
- Body types: Static, Dynamic, Kinematic
- `set_body_state()`, `set_body_type()` for runtime mutation
- `remove_collider()`, `remove_joint()` by handle
- Mass/inertia accumulation across multiple colliders per body
- Sleep/deactivation with velocity threshold and island-based atomic wake
- Generational arena storage for O(1) access

#### Joints
- 8 joint types: Fixed, Revolute, Prismatic, Spring, Distance, Wheel, Rope, Mouse
- Joint motors (revolute/prismatic) with `target_velocity` and `max_force`
- Joint damping and breaking (`break_force` threshold)
- Joint limits on Revolute and Prismatic

#### Materials
- Presets: `ice()`, `rubber()`, `wood()`, `steel()`, `bouncy()`
- Rolling friction
- Static friction (`static_friction` field)
- Friction/restitution combine rules: `Min`, `Average`, `Multiply`, `Max`

#### Particles
- Physics particles with gravity, drag, damping, lifetime, collider interaction
- Particle emitters with rate-based spawning and golden-ratio spread
- Radial force fields (attraction/repulsion with configurable falloff)
- Directional force fields (wind zones with AABB regions)
- Sub-emitters (spawn children on particle death)

#### Spring Animation
- Standalone `Spring`, `Spring2d`, `Spring3d` types (no world needed)
- Presets: `critically_damped()`, `over_damped()`, `under_damped()`
- Settle detection, snap, fling, retarget

#### Serialization
- `WorldSnapshot` with `snapshot()`/`restore()` on `PhysicsWorld`
- `serialize_world()`/`deserialize_world()` via bitcode
- Step-exact determinism verified (serialize at step N, restore, continue)

#### Units
- `Quantity` type with 14 `PhysicsUnit` variants
- Display formatting (N, m, kg, rad, Pa, etc.)

#### API Quality
- `#[non_exhaustive]` on all public enums
- `#[must_use]` on pure functions and accessors
- `Send + Sync` compile-time assertions on all public types
- BTreeMap/BTreeSet for deterministic iteration order
- Zero `.unwrap()` in library code

#### Infrastructure
- Native physics on hisab math — zero external physics engine deps
- hisab 0.22.4 (DVec3, DQuat via glam f64 re-exports)
- CI: check, security audit, cargo-deny, test (Linux/macOS/Windows), MSRV 1.89, coverage, doc, semver
- 35 benchmarks across 10 groups with three-point history tracking
- 227 tests (2D path), 190 tests (3D path)
- Fuzz testing targets (contact generation, serialization)
- Supply-chain: cargo-vet config, cargo-deny strict mode
- Documentation: architecture overview, roadmap, testing guide
- SECURITY.md, CONTRIBUTING.md
- Backend modules split into 8 sub-modules each for maintainability
