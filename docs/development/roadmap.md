# Roadmap

Planned work for impetus. Done work is recorded in the [CHANGELOG](../../CHANGELOG.md), and the current state in
[`state.md`](state.md). Version numbers are assigned when work is scheduled.

Two rules shape every item:
- **Ported code keeps parity.** A change to behavior that `rust-old/` defines diverges from the oracle, so it needs
  an ADR first.
- **New physics has no oracle.** It lands with reference tests (analytic cases, conservation, published results)
  rather than golden rows.

## Correctness and API

- [ ] **No aborts in the JSON writers.** Move `src/fmtf.cyr` and `src/json.cyr` onto `str_builder`'s `_a` calls,
  which return -1 when a grow fails, and return `Err(IM_ERR_SERIALIZE)`. Today the writers exit the process on out of
  memory, the one abort left in the library (port plan §8).
- [ ] **Validate shapes and materials at `add_collider`:** negative extents, non-finite values, empty hulls. It
  diverges from Rust 1.4.0, which accepts them, so it needs an ADR.
- [ ] **Typed accessors for record lists.** `collision_events`, `particles` and `force_fields` return a pointer and
  a count, so a consumer needs its own cast function to read a record (the migration guide shows it). An accessor
  per list, like `world.collision_event(i)`, would remove that.
- [ ] **JSON limits, when bayan lands S3 and S4** (see [Upstream](#upstream)): read integers above `i64::MAX` (handles
  whose slot was reused more than 2^31 times) and ignore repeated unknown keys as serde does. This closes the corpus's
  6 pending rows.

## Bridges

The Rust crate's `bridge.rs` (17 functions; its 27 tests are deferred in the [ledger](rust-test-ledger.md)), ported
as `src/bridge.cyr` one partner at a time, each when a Cyrius consumer needs it. Parity comes from the Rust bridge
tests in `rust-old/`.

- [ ] **dravya:** impact force and contact area → stress at the collision point; deformation velocity → strain
  rate.
- [ ] **sharira:** joint torques and angular velocities → locomotion forces; body mass → inertia tensor; muscle
  torque, cylinder inertia, gait ground-reaction force, centre-of-mass gravity.
- [ ] **ushma:** friction coefficient and sliding velocity → heat generation rate; collision energy → temperature
  change.

## 3D completeness

What the 3D profile still lacks against 2D.

- [ ] **3D islands and island sleep.** Island-atomic sleep, sleeping bodies as infinite mass in the solver, waking
  neighbours on removal, as 2D does.
- [ ] **3D revolute, prismatic and motors.** The 3D `add_joint` refuses them with `IM_ERR_UNSUPPORTED` (ADR 0008
  #5); implementing them lifts the refusal, in `add_joint` and in `restore`.
- [ ] **Consistent joint frames:** the prismatic and wheel axis in body A's frame in 3D, as in 2D.
- [ ] **3D overlap queries.** `overlap_sphere` and `overlap_aabb` return no colliders in 3D.
- [ ] **ConvexHull raycast (3D).** It needs hull faces; 3D hulls are point clouds (a 3D convex hull, hisab H6).

## Collision completeness

Handle every geometry a game throws at you.

- [ ] **Conservative advancement CCD:** exact time of impact for fast convex pairs, alongside speculative contacts
  for bullet-through-paper cases (on hisab's `time_of_impact` / `swept_aabb` once hisab is on the step path).
- [ ] **Compound shapes:** several colliders fused into one rigid shape for mass and inertia; faster than
  multi-collider bodies for complex objects.
- [ ] **Shape casting:** sweep a shape along a direction and return the first hit with its normal and time of
  impact; character controllers, projectile prediction.
- [ ] **Heightfield and trimesh contacts (2D and 3D).** Both are accepted as colliders and generate no contacts.
  Heightfields want a grid-cell narrowphase.
- [ ] **Convex decomposition:** hisab's `convex_decompose()` to build convex hull sets from concave meshes at load
  time.
- [ ] **Point-in-shape queries:** an inside/outside test for every shape type; picking, spawn validation, triggers.

## Joints and constraints

Every mechanical connection a game or sim needs.

- [ ] **XPBD**, if a consumer asks. 2.0 refuses `IM_SOLVER_XPBD` (ADR 0008 #6). A port needs what Rust's XPBD
  lacked: prismatic, wheel and mouse joints, motors, limits and `break_force`; per-point contact projection;
  restitution from the pre-solve velocity.
- [ ] **Cone-twist joint:** a 3D ball-socket with angular limits; ragdoll shoulders and hips.
- [ ] **Weld joint:** a zero-DOF rigid connection; breakable structures, compound objects.
- [ ] **Gear joint:** couples two revolute joints at a ratio; gearboxes, clock mechanisms.
- [ ] **Pulley joint:** two anchors with a shared rope length; elevators, counterweights.
- [ ] **Slider-crank:** prismatic and revolute combined; engine pistons, linkages.
- [ ] **Joint soft limits:** spring-damper resistance at angular and linear limits instead of hard stops.

## Forces and environment

Rich environmental effects without leaving the physics engine.

- [ ] **Buoyancy volumes:** AABB regions with an upward force proportional to the submerged volume; water, lava,
  mud.
- [ ] **Conveyor surfaces:** surface velocity on static bodies; belts, moving floors, escalators.
- [ ] **Anisotropic friction:** direction-dependent friction; ice rinks, brushed surfaces, rails.
- [ ] **Vortex force fields:** rotational force with configurable falloff; tornadoes, whirlpools, fans.
- [ ] **Turbulence fields:** noise-driven force perturbation; wind gusts.
- [ ] **Particle-particle interaction:** N-body forces between particles; flocking, magnetic dust, swarms.

## Performance and scalability

Make 10k+ body scenes real-time, and close the gap to Rust 1.4.0: the median benchmark is 10× Rust's in 2D and 13×
in 3D, and a whole step 2.6–11× and 5.7–17× ([the record](../benchmarks-rust-v-cyrius.md)).

- [ ] **Speed work with the ecosystem.** Once kiran and soorat are in Cyrius, the benchmark record and the upstream
  issues (port plan §8: S5's byte-loop `memcpy`, f64 values in integer registers, the 2.5× cost of struct-value
  kernels) are the input to compiler, ganita and hisab tuning.
- [ ] **Incremental broadphase:** keep the AABB tree across frames and update only moved bodies. The rebuild is
  4.73 ms of the 1,000-body tree step's 5.69 ms.
- [ ] **hisab on the step path,** once S1 or H9 lands: hisab's 3D math, GJK/EPA and `detect_islands` in place of
  impetus's own ([ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md)), with `[deps.hisab]` back in production
  scope ([ADR 0007](../adr/0007-cyrius-port-hisab-first.md)).
- [ ] **GJK simplex caching:** keep each contact pair's simplex between frames to skip the GJK warm-up.
- [ ] **Contact cache compaction:** manifold points stored contiguously, for cache-friendly solver loops.
- [ ] **Parallel island solving:** independent islands on worker threads, with results merged in island order so
  stepping stays deterministic.
- [ ] **Multi-level spatial hash:** two or three cell sizes, so a large object in a small-cell grid needs no AABB
  tree.
- [ ] **Velocity Verlet integration:** a symplectic integrator with better energy conservation than semi-implicit
  Euler, chosen per world.

## Developer experience

Make impetus the obvious choice for anyone writing a game or sim in Cyrius.

- [ ] **Profiling spans:** sakshi spans on broadphase, narrowphase, solver, integration and sleep, behind a level.
  No module logs yet.
- [ ] **Debug stats API:** per-frame counters for broadphase pairs, narrowphase tests, solver iterations, active and
  sleeping bodies, and islands.
- [ ] **Debug wireframe data:** AABBs, contact points and normals, and joint anchors for an external renderer, with
  no rendering dependency.
- [ ] **Deterministic replay:** record each frame's forces and impulses, then replay them to the same bits; network
  sync, testing, bug reports. A replay can start from a snapshot, which restores exactly.
- [ ] **Migration guides** from other engines (rapier, nphysics): API mapping docs.
- [ ] **Cookbook examples:** platformer, top-down, billiards, ragdoll, stacking, breakable, vehicle, chain, in
  `docs/guides/`, where `scripts/doc_examples.py` runs their code.

## Upstream

Open issues impetus filed, and what each unblocks here. The measurements behind them are in the port plan, §8.

| Issue | Where | Unblocks |
|---|---|---|
| S1: a scoped allocator for `alloc` | cyrius, `2026-11-09-no-scoped-allocator-for-alloc.md` | hisab on the step path |
| H9: allocator-scoped calls | hisab, `2026-11-09-allocator-scoped-calls.md` | the same, as hisab's alternative to S1 |
| S3: JSON integers above `i64::MAX` | bayan, `2026-11-09-json-integer-above-i64-max-refuses-the-document.md` | 3 pending rows; JSON for large handles |
| S4: an opt-in duplicate-key policy | bayan, `2026-11-09-json-duplicate-key-has-no-opt-in-for-ignored-members.md` | 3 pending rows; serde's handling of repeated keys |
| S5: byte-loop `memcpy` | cyrius, `2026-11-02-memcpy-memset-byte-loops-dominate-frame-copies.md` | faster arena, shape and snapshot copies (87 of a 2D capture's 145 µs) |
| A field store from a struct call copies its argument | cyrius, `2026-11-09-field-store-of-struct-call-copies-a-field-argument.md` | dropping the workaround and `surface_parity.py`'s guard for it |
| Correctly rounded transcendentals | ganita, `2026-11-09-correctly-rounded-exp-sin-cos-hypot.md` | retiring `src/crmath.cyr` for ganita's, at the same bits |

- [ ] **File the gaps still unfiled** (port plan §8): a shortest round-trip f64 printer in the stdlib, seeded from
  `src/fmtf.cyr` (S2); derive features for serde-shaped JSON, which would let `src/json.cyr` move to `#derive`; the
  stdlib math and collection asks, a stable sort among them; hisab H1–H8.

## Retiring `rust-old/`

- [ ] Once the bridges are ported or dropped: archive the golden corpus and its generators, close the Rust-test
  ledger, and remove `rust-old/`, as tanmatra did.

## Out of scope (separate AGNOS libraries)

- Fluid simulation (SPH, Navier-Stokes) → fluidity
- Soft bodies and deformable meshes → a separate library
- Cloth and rope as mass-spring networks → a separate library
- Electromagnetism, optics, quantum → the science libraries
