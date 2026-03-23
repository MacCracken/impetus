# Changelog

## 0.22.3

### Phase 1 — Scaffold

- Core types: `BodyDesc`, `BodyHandle`, `BodyType`, `BodyState`, `ColliderDesc`, `ColliderHandle`, `ColliderShape`
- Physics world with deterministic stepping, body/collider/joint management
- Material presets (ice, rubber, wood, steel, bouncy)
- Force, impulse, and torque types with constructors and magnitude helpers
- Collision events and contact data
- Spatial query types (raycast, point query, overlap sphere/AABB)
- Unit-aware quantities with PhysicsUnit enum (14 units, display formatting)
- Error types with PartialEq for testability
- Feature flags: `2d` (default), `3d`, `serialize`, `full`
- Send + Sync compile-time assertions on all public types
- `#[non_exhaustive]` on all public enums, `#[must_use]` on pure functions

### Phase 2 — Native 2D Backend

- Spatial hash broadphase with collision layer/mask filtering
- Narrowphase: circle, AABB, OBB (rotation-aware SAT), capsule, ConvexHull, segment contacts
- Sequential impulse constraint solver with Coulomb friction and angular response
- Configurable Baumgarte positional correction (position_slop, position_correction)
- Sleep/deactivation system with velocity threshold and contact waking
- Raycasting with hit point and normal (circle, AABB, capsule shapes)
- Collision events: Started/Stopped/Ongoing with contact pair tracking
- Kinematic body support, sensor colliders
- Physics particles: gravity, drag, damping, lifetime, collider AABB pre-filtering
- Particle emitters with golden-ratio spread
- Mass/inertia accumulation across multiple colliders
- CCD via max_velocity clamp

### Phase 3 — 3D Backend + Serialization

- Native 3D backend with DVec3/DQuat (via hisab 0.22.4)
- 3D spatial hash broadphase with collision layer filtering
- 3D narrowphase: sphere-sphere, sphere-OBB, OBB-OBB, capsule-sphere, capsule-box, capsule-capsule, segment-sphere, segment-box, convex hull-sphere
- 3D contact solver with friction, angular response, 3-axis inertia tensor
- 3D joint solver: Fixed, Distance, Spring with damping
- Sleep/deactivation system (mirrors 2D)
- Proper capsule inertia (hemisphere + parallel axis theorem)
- Quaternion rotation integration
- Bincode serialization: WorldSnapshot with snapshot/restore
- All public types use `[f64; 3]` for unified 2D/3D API

### Phase 4 — Spring Animation

- Standalone `Spring` module: damped harmonic oscillator (no world needed)
- 1D, 2D, 3D spring types
- Presets: critically_damped, over_damped, under_damped
- Settle detection, snap, fling (add_velocity), retarget

### Phase 5 — Kiran ECS Bridge

- Physics bridge in kiran: RigidBody, Collider, PhysicsPosition, Velocity components
- PhysicsEngine resource with entity-body handle mapping
- `physics_step()` system function with position sync and collision event publishing

### Phase 6 — Production Hardening

- CI: 8-job matrix (check, security, deny, test×3OS, MSRV, coverage, doc, semver)
- Supply-chain: cargo-vet config, cargo-deny strict mode
- Documentation: architecture overview, development roadmap, testing guide
- SECURITY.md, CONTRIBUTING.md
- Fuzz testing targets (contact generation, serialization)
- Named constants replacing magic numbers throughout

### Phase 7 — Engineering Backlog

- Arena storage replacing HashMap for O(1) body/collider/joint access
- BTreeMap/BTreeSet for deterministic iteration order
- `set_body_state()`, `set_body_type()` for runtime mutation
- `remove_collider()`, `remove_joint()` by handle
- `raycast_filtered()` with collision layer mask
- `CollisionEvent::Ongoing` for persistent contacts
- Configurable Baumgarte constants, NaN guards, u64 wrapping
- Shared spatial hash module (deduplicated 2D/3D)

### Phase 8 — Joints & Simulation Quality

- 8 joint types: Fixed, Revolute, Prismatic, Spring, Distance, Wheel, Rope, Mouse
- Joint motors (revolute/prismatic), damping, breaking
- Rolling friction, static vs dynamic friction
- Friction/restitution combine rules (Min, Average, Multiply, Max)
- Restitution velocity threshold (no micro-bouncing)

### Phase 9 — Particles & Force Fields

- Physics particles with gravity, drag, damping, lifetime, collider interaction
- Particle emitters with rate-based spawning
- Radial force fields (attraction/repulsion with falloff)
- Directional force fields (wind zones with AABB regions)
- Sub-emitters (spawn children on particle death)

### Phase 10 — Essential Solver Quality

- Warm starting with accumulated impulse clamping
- Persistent contact manifolds with frame-to-frame matching
- Multi-point contact manifolds (up to 2 per pair, incremental building with revalidation)
- Simulation islands (union-find, atomic sleep/wake per island)
- Static vs dynamic friction on PhysicsMaterial
- Sub-stepping (configurable sub_steps on WorldConfig)

### Refactoring

- Backend modules split: `backend_2d/` and `backend_3d/` with 8 sub-modules each
- Step-exact determinism verified (serialize/restore roundtrip test)
- Cargo doc clean with `-D warnings`

### Infrastructure

- Native physics on hisab math — zero external physics deps
- hisab 0.22.4 from crates.io (DVec3, DQuat re-exports)
- Generational arena for O(1) body/collider/joint storage
- Three-point benchmark tracking (baseline/previous/latest)
- 35 benchmarks across 10 groups
- 227 tests (2D), 190 tests (3D), all feature paths clean
