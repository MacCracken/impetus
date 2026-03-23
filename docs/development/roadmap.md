# Roadmap

## Completed (0.22.3)

| Phase | Description |
|-------|-------------|
| 1 | Scaffold — types, CI/CD, benchmarks, tests |
| 2 | Native 2D backend — spatial hash, narrowphase, solver, particles |
| 3 | 3D backend (DVec3/DQuat), bincode serialization, `[f64;3]` API |
| 4 | Standalone spring API for Aethersafha |
| 5 | Kiran ECS bridge |
| 6 | Production hardening |
| 7 | Engineering backlog — all items resolved |

## Road to v1.0

### Scope

Impetus is a **rigid body + particle physics engine**. The following are explicitly out of scope and belong in separate AGNOS crates:

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates

### v1.0 Requirements

#### 3D Feature Parity
- [ ] 3D Revolute joint solver with limits and motor
- [ ] 3D Prismatic joint solver with limits and motor
- [ ] 3D OBB contacts (rotation-aware box-vs-box)
- [ ] 3D capsule-capsule, capsule-box narrowphase
- [ ] 3D ConvexHull narrowphase (GJK/EPA or SAT)
- [ ] 3D segment narrowphase

#### API Completeness
- [ ] `set_body_state()` — teleport bodies, set velocity externally
- [ ] `set_body_type()` — change Static/Dynamic/Kinematic at runtime
- [ ] Raycast with collision layer filter
- [ ] Compound colliders — multiple shapes per body with combined inertia
- [ ] Trigger events: Enter/Exit/Stay (not just Started/Stopped)
- [ ] Remove individual collider (currently only remove_body removes all)
- [ ] Remove joint by handle

#### Determinism
- [ ] Documented determinism guarantee (same platform, same inputs = same output)
- [ ] HashMap iteration order stability (use IndexMap or sorted iteration)
- [ ] Step-exact serialization roundtrip test (serialize at step N, restore, step 100 more, compare)

#### Documentation
- [ ] API documentation on all public types and methods (cargo doc clean)
- [ ] Usage examples: platformer, top-down, 3D scene
- [ ] Performance guide (body count limits, spatial hash tuning, sleep thresholds)

#### Testing
- [ ] Property-based tests (arbitrary body configurations, verify energy conservation)
- [ ] Stress tests (10k bodies, stability over 10k steps)
- [ ] Cross-feature-path integration tests (2D serialize roundtrip, 3D serialize roundtrip)

### Nice to Have (post v1.0)

- Continuous collision detection (swept tests, not just velocity clamp)
- GPU-accelerated broadphase
- Network-synchronized deterministic replay
- Performance profiling API (step timing breakdown)
- Body groups (disable collision between groups of bodies)
