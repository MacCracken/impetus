# Roadmap

## History

| Version | Description |
|---------|-------------|
| 0.1–0.10 | Scaffold, 2D backend, 3D backend, serialization, spring API, kiran bridge |
| 0.11–0.20 | Production hardening, arena storage, determinism, joints, manifolds, islands |
| 0.21–0.23 | Multi-point manifolds, static/dynamic friction, sub-stepping, particles |
| 1.0 | Split impulse, block solver, shock propagation, soft constraints, GJK+EPA, speculative contacts, dynamic AABB tree, full 3x3 inertia tensor |
| 1.1 | XPBD solver (2D), one-shot manifold generation (Sutherland-Hodgman clipping), contact point reduction (area maximization), gyroscopic torque (3D), SolverKind config, audit fixes (div-by-zero guards, panic removal) |

## 1.2.0 — Collision Completeness

Handle every geometry a game throws at you.

- [ ] **Conservative advancement CCD** — exact time-of-impact for fast convex pairs; complement speculative contacts for bullet-through-paper scenarios
- [ ] **Compound shapes** — multiple colliders fused into a single rigid shape for mass/inertia; faster than multi-collider bodies for complex objects
- [ ] **Shape casting** — sweep a shape along a direction, return first hit with normal and TOI; character controllers, projectile prediction
- [ ] **Heightfield collision (2D/3D)** — efficient terrain narrowphase via grid cell lookup instead of brute-force triangle iteration
- [ ] **Convex decomposition integration** — use hisab's `convex_decompose()` to auto-generate convex hull sets from concave meshes at load time
- [ ] **Point-in-shape queries** — fast inside/outside test for all shape types; pick detection, spawn validation, trigger checks

## 1.3.0 — Performance & Scalability

Make 10k+ body scenes real-time.

- [ ] **GJK simplex caching** — persist simplex between frames per contact pair; skip GJK warm-up, ~2x narrowphase speedup for stable contacts
- [ ] **Incremental broadphase** — persist AABB tree across frames, update only moved bodies; O(log n) per move instead of O(n) rebuild
- [ ] **Contact cache compaction** — arena-allocate manifold points contiguously for cache-friendly iteration; reduce pointer chasing in solver hot loop
- [ ] **Parallel island solving** — solve independent simulation islands on separate threads via rayon (feature-gated); linear speedup for disjoint object groups
- [ ] **Multi-level spatial hash** — hierarchical grid with 2–3 cell sizes; eliminates large-object-in-small-cell problem without switching to AABB tree
- [ ] **Velocity Verlet integration** — symplectic integrator with better energy conservation than semi-implicit Euler; configurable per-world

## 1.4.0 — Joints & Constraints

Every mechanical connection a game or sim needs.

- [ ] **Cone-twist joint** — 3D ball-socket with angular limits; ragdoll shoulders, hips
- [ ] **Weld joint** — zero-DOF rigid connection; breakable structures, compound objects
- [ ] **Gear joint** — couples rotation of two revolute joints at a ratio; gearboxes, clock mechanisms
- [ ] **Pulley joint** — two anchor points with a rope length constraint; elevators, counterweights
- [ ] **Slider-crank** — combined prismatic + revolute for engine pistons, linkages
- [ ] **Joint soft limits** — spring-damper resistance at angular/linear limits instead of hard stops

## 1.5.0 — Forces & Environment

Rich environmental effects without leaving the physics engine.

- [ ] **Buoyancy volumes** — AABB regions that apply upward force proportional to submerged volume; water, lava, mud
- [ ] **Conveyor surfaces** — surface velocity on static bodies; conveyor belts, moving floors, escalators
- [ ] **Anisotropic friction** — direction-dependent friction coefficients; ice rinks, brushed surfaces, rails
- [ ] **Vortex force fields** — rotational force with configurable falloff; tornadoes, whirlpools, fan blasts
- [ ] **Turbulence fields** — Perlin-noise-driven force perturbation; wind gusts, atmospheric effects
- [ ] **Particle-particle interaction** — N-body forces between particles; flocking, magnetic dust, attraction/repulsion swarms

## 1.6.0 — Developer Experience

Make impetus the obvious choice for anyone writing a game or sim in Rust.

- [ ] **Debug stats API** — expose per-frame counters: broadphase pairs, narrowphase tests, solver iterations, active/sleeping bodies, island count
- [ ] **Debug wireframe data** — emit AABB, contact point, contact normal, joint anchor geometry for external renderers; zero rendering deps
- [ ] **Profiling spans** — `tracing` spans on broadphase, narrowphase, solver, integration, sleep; plug into any subscriber
- [ ] **Deterministic replay** — record input forces/impulses per frame, replay with bit-exact results; network sync, testing, bug repro
- [ ] **Migration guide** — rapier-to-impetus, nphysics-to-impetus API mapping docs
- [ ] **Cookbook examples** — platformer, top-down, billiards, ragdoll, stacking, breakable, vehicle, chain

## Cross-Crate Bridges

- [ ] `bridge.rs` module — primitive-value conversions for cross-crate physics (no deps on sibling crates)
- [ ] **dravya bridge**: impact force (N), contact area (m²) → stress at collision point; deformation velocity → strain rate
- [ ] **sharira bridge**: joint torques (Nm), angular velocities (rad/s) → locomotion forces; body mass (kg) → inertia tensor
- [ ] **ushma bridge**: friction coefficient, sliding velocity → heat generation rate (W); collision energy → temperature delta

---

## Out of Scope (separate AGNOS crates)

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates
