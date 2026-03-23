# Changelog

## 0.22.3

### Phase 1 — Scaffold

- Core types: `BodyDesc`, `BodyHandle`, `BodyType`, `BodyState`, `ColliderDesc`, `ColliderHandle`, `ColliderShape`
- Physics world with deterministic stepping, body/collider/joint management
- Material presets (ice, rubber, wood, steel, bouncy)
- Force, impulse, and torque types with constructors and magnitude helpers
- Joint types (fixed, revolute, prismatic, spring, distance) with motor support
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
- All 5 joint types with limits, motors (revolute/prismatic), and damping
- Sleep/deactivation system with velocity threshold and contact waking
- Raycasting with hit point and normal (circle, AABB, capsule shapes)
- Collision events: Started/Stopped with contact pair tracking and cleanup on removal
- Kinematic body support, sensor colliders
- Physics particles: gravity, drag, damping, lifetime, collider AABB pre-filtering
- Particle emitters with golden-ratio spread
- Mass/inertia accumulation across multiple colliders
- CCD via max_velocity clamp

### Phase 3 — 3D Backend + Serialization

- Native 3D backend with DVec3/DQuat (via hisab 0.22.4)
- 3D spatial hash broadphase with collision layer filtering
- 3D narrowphase: sphere-sphere, sphere-AABB, AABB-AABB, capsule-sphere
- 3D contact solver with friction, angular response, 3-axis inertia tensor
- 3D joint solver: Fixed, Distance, Spring with damping
- Sleep/deactivation system (mirrors 2D)
- Proper capsule inertia (hemisphere + parallel axis theorem)
- Quaternion rotation integration
- Bincode serialization: WorldSnapshot with snapshot/restore
- All public types use `[f64; 3]` for unified 2D/3D API
- TriMesh/ConvexHull/Segment/Heightfield AABB computation

### Phase 4 — Spring Animation

- Standalone `Spring` module: damped harmonic oscillator (no world needed)
- 1D, 2D, 3D spring types
- Presets: critically_damped, over_damped, under_damped
- Settle detection, snap, fling (add_velocity), retarget

### Phase 5 — Kiran ECS Bridge

- `kiran-physics` bridge: RigidBody, Collider, PhysicsPosition, Velocity components
- PhysicsEngine resource with entity-body handle mapping
- `physics_step()` system function with position sync and collision event publishing
- Collision layer/mask builder methods

### Phase 6 — Production Hardening

- CI: 8-job matrix (check, security, deny, test×3OS, MSRV, coverage, doc, semver)
- Supply-chain: cargo-vet config, cargo-deny strict mode
- Documentation: architecture overview, development roadmap, testing guide
- SECURITY.md, CONTRIBUTING.md
- Fuzz testing targets (contact generation, serialization)
- Named constants replacing magic numbers throughout
- NaN guards in joint normalization
- u64 ID wrapping for deterministic overflow
- Shared spatial hash module (deduplicated 2D/3D)

### Infrastructure

- Native physics on hisab math — zero external physics deps
- hisab 0.22.4 from crates.io (DVec3, DQuat re-exports)
- Three-point benchmark tracking (baseline/previous/latest)
- 26 benchmarks across 7 groups
- 178+ tests (2D), 137+ tests (3D), all feature paths clean
