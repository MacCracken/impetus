# Roadmap

## History

| Version | Description |
|---------|-------------|
| 0.1–0.10 | Scaffold, 2D backend, 3D backend, serialization, spring API, kiran bridge |
| 0.11–0.20 | Production hardening, arena storage, determinism, joints, manifolds, islands |
| 0.21–0.23 | Multi-point manifolds, static/dynamic friction, sub-stepping, particles |
| 1.0 | Split impulse, block solver, shock propagation, soft constraints, GJK+EPA, speculative contacts, dynamic AABB tree, full 3x3 inertia tensor |
| 1.1 | XPBD solver (2D), one-shot manifold generation (Sutherland-Hodgman clipping), contact point reduction (area maximization), gyroscopic torque (3D), SolverKind config, audit fixes (div-by-zero guards, panic removal) |
| 1.2 | Cross-crate bridges: dravya, sharira, ushma |
| 1.3 | sharira bridge additions (muscle torque, cylinder inertia, gait GRF, CoM gravity) |
| 1.4 | Rust 1.99 / hisab 1.4; audit and repair sweep — contact normal orientation, polygon clipping, solver (restitution, block solver, warm start, split impulse), velocity-level joints, sub-step forces/events, 3D world-space inertia, GJK/EPA/SAT fixes, mass properties, sleep/islands, balanced AABB tree, validated config and snapshot restore, exact springs |

**Next: the Cyrius port (2.0.0).** Rust 1.4.0 is the final Rust feature release and the parity
oracle; see [`impetus-port-plan.md`](impetus-port-plan.md) and
[ADR 0007](../adr/0007-cyrius-port-hisab-first.md). The 2.0 breaking items below land in the Cyrius
API; the themed sections continue in Cyrius after 2.0.

Planned sections below are grouped by theme; version numbers are assigned when work is scheduled.

## Collision Completeness

Handle every geometry a game throws at you.

- [ ] **Conservative advancement CCD** — exact time-of-impact for fast convex pairs; complement speculative contacts for bullet-through-paper scenarios
- [ ] **Compound shapes** — multiple colliders fused into a single rigid shape for mass/inertia; faster than multi-collider bodies for complex objects
- [ ] **Shape casting** — sweep a shape along a direction, return first hit with normal and TOI; character controllers, projectile prediction
- [ ] **Heightfield collision (2D/3D)** — efficient terrain narrowphase via grid cell lookup (Heightfield and TriMesh colliders currently generate no contacts)
- [ ] **ConvexHull raycast (3D)** — needs hull faces; 3D hulls are point clouds today
- [ ] **Convex decomposition integration** — use hisab's `convex_decompose()` to auto-generate convex hull sets from concave meshes at load time
- [ ] **Point-in-shape queries** — fast inside/outside test for all shape types; pick detection, spawn validation, trigger checks

## Performance & Scalability

Make 10k+ body scenes real-time.

- [ ] **GJK simplex caching** — persist simplex between frames per contact pair; skip GJK warm-up, ~2x narrowphase speedup for stable contacts
- [ ] **Incremental broadphase** — persist AABB tree across frames, update only moved bodies; O(log n) per move instead of O(n) rebuild
- [ ] **Contact cache compaction** — arena-allocate manifold points contiguously for cache-friendly iteration; reduce pointer chasing in solver hot loop
- [ ] **Parallel island solving** — solve independent simulation islands on separate threads via rayon (feature-gated); linear speedup for disjoint object groups
- [ ] **Multi-level spatial hash** — hierarchical grid with 2–3 cell sizes; eliminates large-object-in-small-cell problem without switching to AABB tree
- [ ] **Velocity Verlet integration** — symplectic integrator with better energy conservation than semi-implicit Euler; configurable per-world

## Joints & Constraints

Every mechanical connection a game or sim needs.

- [ ] **3D revolute, prismatic and motors** — accepted today but have no effect in the 3D backend
- [ ] **XPBD parity** — prismatic, wheel and mouse joints, motors, limits and `break_force` under `SolverKind::Xpbd`; per-point (not per-manifold) contact projection; restitution from pre-solve velocity
- [ ] **Cone-twist joint** — 3D ball-socket with angular limits; ragdoll shoulders, hips
- [ ] **Weld joint** — zero-DOF rigid connection; breakable structures, compound objects
- [ ] **Gear joint** — couples rotation of two revolute joints at a ratio; gearboxes, clock mechanisms
- [ ] **Pulley joint** — two anchor points with a rope length constraint; elevators, counterweights
- [ ] **Slider-crank** — combined prismatic + revolute for engine pistons, linkages
- [ ] **Joint soft limits** — spring-damper resistance at angular/linear limits instead of hard stops

## Forces & Environment

Rich environmental effects without leaving the physics engine.

- [ ] **Buoyancy volumes** — AABB regions that apply upward force proportional to submerged volume; water, lava, mud
- [ ] **Conveyor surfaces** — surface velocity on static bodies; conveyor belts, moving floors, escalators
- [ ] **Anisotropic friction** — direction-dependent friction coefficients; ice rinks, brushed surfaces, rails
- [ ] **Vortex force fields** — rotational force with configurable falloff; tornadoes, whirlpools, fan blasts
- [ ] **Turbulence fields** — Perlin-noise-driven force perturbation; wind gusts, atmospheric effects
- [ ] **Particle-particle interaction** — N-body forces between particles; flocking, magnetic dust, attraction/repulsion swarms

## Developer Experience

Make impetus the obvious choice for anyone writing a game or sim in Rust.

- [ ] **Debug stats API** — expose per-frame counters: broadphase pairs, narrowphase tests, solver iterations, active/sleeping bodies, island count
- [ ] **Debug wireframe data** — emit AABB, contact point, contact normal, joint anchor geometry for external renderers; zero rendering deps
- [ ] **Profiling spans** — `tracing` spans on broadphase, narrowphase, solver, integration, sleep; plug into any subscriber (no `tracing` instrumentation exists yet)
- [ ] **Deterministic replay** — record input forces/impulses per frame, replay with bit-exact results; network sync, testing, bug repro
- [ ] **Migration guide** — rapier-to-impetus, nphysics-to-impetus API mapping docs
- [ ] **Cookbook examples** — platformer, top-down, billiards, ragdoll, stacking, breakable, vehicle, chain

## Cross-Crate Bridges

- [x] `bridge.rs` module — primitive-value conversions for cross-crate physics (no deps on sibling crates) — 1.2
- [x] **dravya bridge**: impact force (N), contact area (m²) → stress at collision point; deformation velocity → strain rate — 1.2
- [x] **sharira bridge**: joint torques (Nm), angular velocities (rad/s) → locomotion forces; body mass (kg) → inertia tensor — 1.2/1.3
- [x] **ushma bridge**: friction coefficient, sliding velocity → heat generation rate (W); collision energy → temperature delta — 1.2

## 2.0 — Breaking Cleanup (deferred from the 1.4 audit)

Each item needs a public-API or snapshot-format change, so it waits for a major release.

- [ ] **Complete snapshots** — force fields, contact manifolds and warm-start impulses, sleep state, and arena slot generations + free-list order, so a restored world steps bit-identically and allocates the same handles (regression tests `restore_replaces_force_fields`, `restore_preserves_handle_allocation` are `#[ignore]`d until then). Bitcode is not self-describing, so this changes the format.
- [ ] **3D orientation in `BodyState` / `BodySnapshot`** — full quaternion and angular velocity; today 3D get/set/snapshot keep only rotation about Z
- [ ] **Fallible `add_collider` / `add_joint`** — return `Result` instead of storing orphans for missing bodies; validate shapes (negative extents) and materials
- [ ] **Remove deprecated `PhysicsWorld::restore`** in favour of `try_restore`
- [ ] **Consistent joint frames** — prismatic/wheel axis in body A's frame in 3D as in 2D
- [ ] **3D islands** — island-atomic sleep, sleeping bodies as infinite mass in the solver, waking neighbours on removal (2D does all three)
- [ ] **Cross-platform determinism** — `sin_cos`, `atan2` and `powf` come from the platform libm, so results are only bit-identical on the same platform

---

## Out of Scope (separate AGNOS crates)

- Fluid simulation (SPH, Navier-Stokes) → fluidity crate
- Soft body / deformable meshes → separate crate
- Cloth / rope (mass-spring networks) → separate crate
- Electromagnetism, optics, quantum → separate science crates
