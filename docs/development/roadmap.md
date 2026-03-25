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
| 10 | Essential solver quality — multi-point manifolds, simulation islands, static/dynamic friction, sub-stepping |

## Current Status

v1.0 complete. 207 tests, 35 benchmarks. All simulation quality items shipped. Kiran builds clean.

## Completed in v1.0

### Simulation Quality

- [x] **Split impulse** — pseudo-velocity position correction, no energy injection
- [x] **Block solver** — coupled 2x2 normal solve for 2-point manifolds (Box2D-style)
- [x] **Shock propagation** — BFS bottom-up contact ordering for stacking stability
- [x] **Soft constraints** — ERP/CFM spring-damper with configurable frequency/damping
- [x] **GJK + EPA** — native f64 convex-convex 3D collision
- [x] **Speculative contacts** — velocity-expanded broadphase AABBs for CCD
- [x] **Dynamic AABB tree** — BVH broadphase with SAH insertion and fattened AABBs

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
