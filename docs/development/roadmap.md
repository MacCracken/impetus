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
| 8 | Arena storage, BTreeMap determinism, API completeness |
| 9 | Warm starting, persistent manifolds, joints, simulation quality, particles, refactoring |

## Current Status

Feature-complete for 0.22.3. 217+ tests (2D), 190+ tests (3D), 34 benchmarks. Backends refactored into sub-modules. Kiran builds clean.

## Road to v1.0

### Essential — Solver Quality

- [ ] **Multi-point contact manifolds** — 2 points per pair (2D), 4 per pair (3D). Sutherland-Hodgman clipping.
- [ ] **Simulation islands** — union-find grouping of connected bodies. Atomic sleep/wake, solver partitioning.
- [ ] **Static vs dynamic friction** — two coefficients per material.
- [ ] **Sub-stepping** — multiple solver sub-steps per frame (Box2D v3 "Soft Step" approach).

### Important — Simulation Quality

- [ ] **Soft constraints** — spring-damper formulation replacing Baumgarte.
- [ ] **Shock propagation** — bottom-up solve order in contact graph.
- [ ] **Block solver** — solve 2-point manifold normal+friction as coupled system.
- [ ] **Split impulse** — separate position correction from velocity.
- [ ] **GJK + EPA** — general convex-convex collision via support functions.
- [ ] **Speculative contacts** — expand broadphase AABB by velocity for true CCD.
- [ ] **Dynamic AABB tree** — tree-based broadphase for heterogeneous object sizes.

### Nice to Have (post v1.0)

- [ ] XPBD solver
- [ ] Velocity Verlet integration
- [ ] Anisotropic friction
- [ ] GJK simplex caching
- [ ] Gear/pulley joints
- [ ] Vortex/turbulence force fields
- [ ] Particle-particle interaction
- [ ] Multi-level spatial hash grid
- [ ] Gyroscopic torque (3D)
- [ ] Conservative advancement CCD
- [ ] One-shot manifold generation
- [ ] Contact point reduction
- [ ] Full 3x3 inertia tensor

## Out of Scope (separate AGNOS crates)

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates
