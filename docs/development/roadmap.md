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

## Current Status

Feature-complete for 0.22.3. 192+ tests, 34 benchmarks, cargo doc clean. Warm starting + persistent manifolds implemented. Kiran builds clean.

## Road to v1.0

### Essential — Solver Quality

These are the core improvements that separate a toy physics engine from a production one.

- [x] **Warm starting** — cache impulses from previous frame, re-apply at start of solver. Accumulated impulse clamping.
- [x] **Persistent contact manifolds** — match contacts across frames by body-local proximity. Impulses preserved on matched points.
- [ ] **Multi-point contact manifolds** — 2 points per pair (2D), 4 per pair (3D). Sutherland-Hodgman clipping. Single-point contacts cause rocking instability in box stacks.
- [ ] **Simulation islands** — union-find grouping of connected bodies. Atomic sleep/wake, solver partitioning, parallelization opportunity.
- [ ] **Restitution velocity threshold** — zero out restitution below ~1 m/s relative velocity. Prevents micro-bouncing of resting objects.
- [ ] **Static vs dynamic friction** — two coefficients per material (`friction_static`, `friction_dynamic`). Objects shouldn't slide on slopes when at rest.
- [ ] **Sub-stepping** — multiple solver sub-steps per frame (Box2D v3 "Soft Step" approach). Better convergence than more iterations at the same dt.

### Important — Simulation Quality

- [ ] **Soft constraints** — spring-damper formulation replacing Baumgarte. Energy-correct, configurable frequency + damping ratio.
- [ ] **Shock propagation** — bottom-up solve order in contact graph. Dramatically stabilizes tall stacks (10+ boxes).
- [ ] **Block solver** — solve 2-point manifold normal+friction as coupled system.
- [ ] **Split impulse** — separate position correction from velocity. Prevents Baumgarte energy injection.
- [ ] **GJK + EPA** — general convex-convex collision via support functions. Enables arbitrary convex shapes.
- [ ] **Rolling friction** — torque opposing rolling motion. Prevents infinite rolling of spheres.
- [ ] **Friction/restitution combine rules** — configurable: `min`, `max`, `average`, `multiply`, `sqrt(a*b)`.
- [ ] **Speculative contacts** — expand broadphase AABB by velocity for true CCD.
- [ ] **Dynamic AABB tree** — tree-based broadphase for heterogeneous object sizes.

### Important — Joints & Interaction

- [ ] **Wheel joint** — prismatic (suspension) + revolute (spin) for vehicles.
- [ ] **Rope/max-distance joint** — inequality constraint, prevents exceeding max length only.
- [ ] **Mouse/target joint** — spring-damper drag toward world-space target. Essential for interactive demos.
- [ ] **Joint breaking** — remove joint when force/torque exceeds threshold.

### Important — Particles

- [ ] **Radial force fields** — attraction/repulsion: `F = k * dir / |r|^n`
- [ ] **Directional force fields** — constant force within a region (wind zones)
- [ ] **Sub-emitters** — particles that spawn particles (firework bursts)

### Nice to Have (post v1.0)

- [ ] XPBD solver (for cloth/soft-body use cases)
- [ ] Velocity Verlet integration
- [ ] Anisotropic friction
- [ ] GJK simplex caching (warm-start GJK from previous frame)
- [ ] Gear/pulley joints
- [ ] Vortex/turbulence force fields (Perlin noise-based)
- [ ] Particle-particle interaction
- [ ] Multi-level spatial hash grid
- [ ] Gyroscopic torque (3D)
- [ ] Conservative advancement CCD
- [ ] One-shot manifold generation
- [ ] Contact point reduction (maximize contact area)
- [ ] Full 3x3 inertia tensor (off-diagonal terms for asymmetric 3D shapes)

## Out of Scope (separate AGNOS crates)

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates
