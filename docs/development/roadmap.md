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

## Post-v1.0 Roadmap

### Phase 11 — Solver & Collision Precision

The sim should be indistinguishable from reality for rigid bodies.

- [ ] **XPBD solver** — position-based dynamics with compliant constraints; more stable than sequential impulse for stiff stacks and rag-dolls
- [ ] **One-shot manifold generation** — compute full contact manifold in a single pass (clipping-based) instead of accumulating over frames
- [ ] **Contact point reduction** — reduce large manifolds to 4 optimal points via area maximisation; less work per iteration, same quality
- [ ] **Conservative advancement CCD** — exact time-of-impact for fast convex pairs, replacing speculative-only approach for bullet-through-paper scenarios
- [x] **Full 3x3 inertia tensor** — `DMat3` replaces diagonal `DVec3`; accurate for asymmetric meshes, compound shapes, off-axis colliders
- [ ] **Gyroscopic torque (3D)** — `ω × (I·ω)` precession term for spinning rigid bodies; required for tops, wheels, projectiles

### Phase 12 — Performance & Scalability

Make 10k+ body scenes real-time.

- [ ] **GJK simplex caching** — persist simplex between frames per contact pair; skip GJK warm-up, ~2x narrowphase speedup for stable contacts
- [ ] **Multi-level spatial hash** — hierarchical grid with 2–3 cell sizes; eliminates large-object-in-small-cell problem without switching to AABB tree
- [ ] **Velocity Verlet integration** — symplectic integrator with better energy conservation than semi-implicit Euler; configurable per-world
- [ ] **Parallel island solving** — solve independent simulation islands on separate threads via rayon; linear speedup for disjoint object groups
- [ ] **Contact cache compaction** — arena-allocate manifold points contiguously for cache-friendly iteration; reduce pointer chasing in solver hot loop
- [ ] **Incremental broadphase** — persist AABB tree across frames, update only moved bodies; O(log n) per move instead of O(n) rebuild

### Phase 13 — Joints & Constraints

Every mechanical connection a game or sim needs.

- [ ] **Gear joint** — couples rotation of two revolute joints at a ratio; gearboxes, clock mechanisms
- [ ] **Pulley joint** — two anchor points with a rope length constraint; elevators, counterweights
- [ ] **Cone-twist joint** — 3D ball-socket with angular limits; ragdoll shoulders, hips
- [ ] **Weld joint** — zero-DOF rigid connection; breakable structures, compound objects
- [ ] **Slider-crank** — combined prismatic + revolute for engine pistons, linkages
- [ ] **Joint soft limits** — spring-damper resistance at angular/linear limits instead of hard stops

### Phase 14 — Forces & Interaction

Rich environmental effects.

- [ ] **Anisotropic friction** — direction-dependent friction coefficients; ice rinks, brushed surfaces, conveyor belts
- [ ] **Vortex force fields** — rotational force with configurable falloff; tornadoes, whirlpools, fan blasts
- [ ] **Turbulence fields** — Perlin-noise-driven force perturbation; wind gusts, atmospheric effects
- [ ] **Particle-particle interaction** — N-body forces between particles; flocking, magnetic dust, attraction/repulsion swarms
- [ ] **Buoyancy volumes** — AABB regions that apply upward force proportional to submerged volume; water, lava
- [ ] **Conveyor surfaces** — surface velocity on static bodies; conveyor belts, moving floors, escalators

### Phase 15 — Developer Experience

Make impetus the obvious choice.

- [ ] **Debug stats API** — expose per-frame counters: broadphase pairs, narrowphase tests, solver iterations, active/sleeping bodies, island count
- [ ] **Debug wireframe data** — emit AABB, contact point, contact normal, joint anchor data for external renderers; no rendering dep
- [ ] **Profiling spans** — `tracing` spans on broadphase, narrowphase, solver, integration, sleep; plug into any subscriber
- [ ] **Deterministic replay** — record input forces/impulses per frame, replay with bit-exact results; network sync, testing
- [ ] **Migration guide** — rapier-to-impetus, nphysics-to-impetus API mapping docs
- [ ] **Cookbook examples** — platformer, top-down, billiards, ragdoll, stacking, breakable, vehicle, chain

### Phase 16 — Shape & Query Expansion

Handle every geometry a game throws at you.

- [ ] **Compound shapes** — multiple colliders treated as a single rigid shape for mass/inertia; faster than multi-collider bodies
- [ ] **Heightfield collision (2D/3D)** — efficient terrain narrowphase via grid cell lookup; currently AABB-only
- [ ] **Shape casting** — sweep a shape along a direction, return first hit; character controllers, projectile prediction
- [ ] **Convex decomposition integration** — use hisab's `convex_decompose()` to auto-generate convex hull sets from concave meshes
- [ ] **Point-in-shape queries** — fast inside/outside test for all shape types; pick detection, spawn validation

## Out of Scope (separate AGNOS crates)

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates
