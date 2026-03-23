# Changelog

## 0.1.0

### Phase 1 — Scaffold

- Core types: `BodyDesc`, `BodyHandle`, `BodyType`, `BodyState`, `ColliderDesc`, `ColliderHandle`, `ColliderShape`
- Physics world with deterministic stepping, body/collider/joint management
- Material presets (ice, rubber, wood, steel, bouncy)
- Force, impulse, and torque types with constructors and magnitude helpers
- Joint types (fixed, revolute, prismatic, spring, distance)
- Collision events and contact data
- Spatial query types (raycast, point query)
- Unit-aware quantities with PhysicsUnit enum (14 units, display formatting)
- Error types with PartialEq for testability
- Feature flags: `2d` (default), `3d`, `serialize`, `full`
- Send + Sync compile-time assertions on all public types
- CI/CD: GitHub Actions (check, test, security audit, supply chain, MSRV, coverage, release)
- Criterion benchmarks with bench-history.sh tracking
- Integration tests, example

### Phase 2 — Native 2D Backend

- Spatial hash broadphase (O(n) from O(n²), 25x speedup at 1000 bodies)
- Narrowphase contact generation: circle-circle, circle-AABB, AABB-AABB, capsule-circle, capsule-AABB, capsule-capsule
- Sequential impulse constraint solver with friction (Coulomb) and angular response
- Baumgarte positional correction (separate velocity/position iterations from WorldConfig)
- All 5 joint types: Fixed, Revolute (with limits), Prismatic (with limits), Spring (damped), Distance
- Raycasting: circle and AABB shapes
- Collision events: Started/Stopped with contact pair tracking
- Kinematic body support (user-driven velocity, no gravity)
- Sensor colliders (events only, no physical response)
- Physics particles: gravity, drag, damping, lifetime, collider interaction (Ball, Box, Capsule)
- Particle emitters: rate-based spawning with golden-ratio spread
- Mass/inertia accumulation across multiple colliders per body
- Collision pair cleanup on body removal

### Phase 3 — 3D Backend + Serialization

- Native 3D physics backend with DVec3/DQuat (via hisab)
- 3D spatial hash broadphase with (i32, i32, i32) cells
- 3D narrowphase: sphere-sphere, sphere-AABB, AABB-AABB, capsule-sphere
- 3D contact solver with friction, angular response, and 3-axis inertia tensor
- 3D joint solver: Fixed, Distance, Spring
- Quaternion rotation integration
- Bincode serialization: WorldSnapshot with snapshot/restore for bodies, colliders, joints, particles, emitters
- All public types migrated from `[f64; 2]` to `[f64; 3]` for unified 2D/3D API
- 3D particle collision support

### Phase 4 — Spring Animation

- Standalone `Spring` module: damped harmonic oscillator (no world needed)
- 1D, 2D, 3D spring types
- Presets: critically_damped, over_damped, under_damped
- settle detection, snap, fling (add_velocity), retarget

### Infrastructure

- Removed rapier dependency — fully native physics on hisab math
- 3D backend uses hisab's DVec3/DQuat instead of hand-rolled helpers
- Three-point benchmark tracking (baseline/previous/latest)
- 26 benchmarks across 7 groups
- 140+ tests across all feature paths
- hisab patched locally for f64 re-exports (DVec3, DQuat, DMat3, DMat4)
