# Benchmarks — Rust 1.4.0 vs the Cyrius port

Each row times one benchmark body: the same work in Rust (`rust-old/`, release build, criterion) and
in Cyrius (`tests/impetus.bcyr`, `cyrius bench`). Port milestones append rows as their modules land.
Numbers are host-specific, so compare within a column set, never across hosts.

Speed is measured, not gated (port plan §0): these are first numbers on a young language with no
speed work yet, recorded so the compiler, ganita and hisab can see where impetus's workload spends its
time. A slow row is a finding to file, with its numbers, not a reason to change the port.

## Method

- **Rust, criterion:** the latest `rust-old/bench-history.csv` run (2026-11-09T09:49Z, `af59838`),
  median of criterion's estimate.
- **Cyrius:** `cyrius bench tests/impetus.bcyr`, average of the batch. `lib/bench.cyr` measures its
  clock-read floor and subtracts it from every sample (about 1.3 µs per read on this host). 1,000,000
  iterations per body.
- **Host:** x86_64 Linux. Toolchains: rustc 1.99.1 (release); Cyrius 6.7.18 (ganita 1.2.15).

## M1a Foundation: forces, materials, units (2026-11-09)

| Bench | Body | Rust | Cyrius | Cyrius / Rust |
|---|---|---|---|---|
| `forces/force_magnitude` | \|(3, 4, 0)\| | 2.10 ns | 13 ns | 6.2× |
| `forces/gravity_force` | the weight of 10 kg at 9.81 m/s² (builds a force) | 1.27 ns | 39 ns | 31× |
| `materials/preset_steel` | build the steel preset | 1.32 ns | 21 ns | 16× |
| `materials/preset_rubber` | build the rubber preset | 1.30 ns | 21 ns | 16× |
| `materials/degrees_to_radians` | 180° to radians | 1.06 ns | 6 ns | 5.7× |
| `materials/quantity_new` | 42 N | 1.04 ns | 5 ns | 4.8× |
| `materials/quantity_display` | format 9.81 N | 81.3 ns | — | waits for S2 (port plan §8) |

**Reading the gap.** Rust inlines each body into a few instructions. Cyrius compiles every call as a
call, and the rows that build struct values pay most:
- `gravity_force` makes two struct-returning calls (`ImVec3_new` inside `ImForce_new`), each through
  the hidden return pointer, then copies the nested vectors into the force;
- the material presets build an 8-field struct through `_im_material` and return it the same way.

That is the cost probe 2 measured (`port-parity.md` §4.2: struct values 2.5× the same arithmetic on
`f64` locals), filed as a compiler finding in the port plan §8. The pure-arithmetic rows (magnitude,
to_radians) are 5–6×.

## M1b Foundation: springs, Display, serde (2026-11-09)

Two runs each; the Cyrius column is their mean. The serde and Display bodies build a fresh document per
iteration, as Rust's `to_string` and `format!` allocate a fresh `String`, and run 10,000 iterations (the
builder grows the global heap).

| Bench | Body | Rust | Cyrius | Cyrius / Rust |
|---|---|---|---|---|
| `springs/spring_1d_step` | one cached step at 60 fps | 4.98 ns | 28.5 ns | 5.7× |
| `springs/spring_2d_step` | both axes, cached | 6.04 ns | 59.5 ns | 9.9× |
| `springs/spring_settle_frames` | a critically damped spring from rest until settled | 1.47 µs | 1.80 µs → 1.86 µs (ADR 0011) | 1.2× → 1.26× |
| `materials/quantity_display` | format 9.81 N | 81.3 ns | 830 ns | 10× |
| `serde/serialize_body_desc` | a dynamic body to JSON | 252 ns | 3.48 µs | 14× |
| `serde/deserialize_body_desc` | the default body from JSON | 276 ns | 7.19 µs | 26× |
| `serde/serialize_config` | the default world config to JSON | 422 ns | 8.94 µs | 21× |
| `serde/roundtrip_collider_desc` | a rubber ball to JSON and back | 892 ns | 16.1 µs | 18× |

**Reading the gap.**
- **The spring step** is six f64 loads, four multiplies, four adds and two `is_finite` checks. Rust keeps
  them in registers; Cyrius moves every operand through memory and xmm0/xmm1 (the compiler finding in port
  plan §8). The 2D step is two 1D steps through nested struct fields.
- **`spring_settle_frames` is the near-tie (1.2×)**: one transition (exp, sqrt), then 52 cached steps and
  53 `is_settled` checks, a fresh spring per iteration. Per frame that is 28 ns in Rust and 35 ns in
  Cyrius. Rust's frame here costs over five times its tight `spring_1d_step` iteration; why is not
  measured yet.
- **The float text is the cost in Display and serde.** `src/fmtf.cyr` finds the shortest digits with
  Burger & Dybvig's algorithm on a 40-limb bignum (ADR 0010 chose it for size and provability, not speed),
  about 0.8 µs per number, against Rust's table-driven Ryu/zmij at tens of nanoseconds. A body descriptor
  has 10 floats and a config 9, which is most of their time. A stdlib Ryu or Schubfach (S2, port plan §8)
  would close most of it.
- **Decoding** builds bayan's value tree on the decode arena, then looks each field up by name; serde_json
  streams into the struct. The tree costs allocation and a second pass.

## M1b part 3: correctly rounded exp, sin and cos (2026-11-09, ADR 0011)

The spring's transition now calls the correctly rounded primitives (`src/crmath.cyr`). Cyrius-only rows: Rust's
std calls glibc, about 5 ns. The polyfills they replaced are given for scale (scratch bench, 200,000 iterations).

| Body | First version (phases 2 and 3) | With phase 1 | The polyfill it replaced |
|---|---|---|---|
| `math/exp_cr`: exp(−0.3) | 2.61 µs | 97 ns | 48–64 ns (`_f64_exp_polyfill`) |
| `math/sin_cos_cr`: sin and cos of 2.5 | 3.29 µs | 187 ns | 75–83 ns (`f64_sin` + `f64_cos`) |
| `springs/spring_settle_frames` (Rust 1.47 µs) | 4.44 µs | 1.86 µs (1.26×) | 1.80 µs |
| exp, the slow path (forced) | 26.6 µs | 24.6 µs | — |
| sin of ~2^996, the slow path (Payne–Hanek) | 18.1 µs | 16.6 µs | — |

**Reading it.**
- **The first version** went straight to phase 2. That is about 80 double-double operations (27 Taylor terms for exp,
  16 per series for sin and cos), each a multi-return call through memory.
- **Phase 1** replaces them in about 99.7 % of calls:
  - a table lookup (2^(j/64); sin and cos of i/64);
  - a short polynomial in plain double;
  - one or two inline TwoProducts.

  That puts the correctly rounded functions at about 2× the polyfills, which are not correctly rounded.
- **Steady spring state is unaffected.** The spring caches its transition, so `spring_1d_step` and `spring_2d_step`
  don't change.
- **A fresh spring** pays one transition: 1.86 µs to settle, against 1.80 µs on the polyfills.

## M2 part 1: the broadphase (2026-11-09)

Cyrius-only rows. Rust's only broadphase bench, `broadphase/aabb_tree_1000_row`, times a whole world step
(505.76 µs), so its comparison waits for M3. The layout is the same: 1,000 unit boxes 1.5 apart, rebuilt and
queried with the storage kept, as a step does. 100 iterations, two runs.

| Bench | Body | Cyrius |
|---|---|---|
| `broadphase/aabb_tree_1000_row` | clear, 1,000 inserts, query the pairs | 4.70 ms |
| `broadphase/spatial_hash_1000_row` | the automatic cell size, reset, 1,000 inserts, query the pairs | 1.95 ms |

**Reading it.** The tree's broadphase alone takes about 9× Rust's whole step, so this is a finding to profile before
M3.
- Each insert walks about 10 levels. At every level it merges three boxes, which is six Rust-semantics min/max calls
  per box, and every node field is a load or store through a method call.
- The query does the same per leaf.
- The hash is 2.4× faster than the tree on this layout, which is what the structures predict for uniform boxes.


## M2 part 2: the 2D backend's types (2026-11-09)

Cyrius-only rows, 100,000 iterations each, x86_64 ELF. Rust has no bench for these alone. Its nearest row,
`bodies/add_body_with_collider`, also covers the world's arena and map inserts, so the like-for-like comparison waits
for M3; it was measured today for context: **253 ns** (criterion, 250.2–257.0).

| Bench | Body | Cyrius |
|---|---|---|
| `b2/add_collider_mass_ball` | a body and a ball collider from their descriptors, the mass added | 545 ns |
| `b2/hull16_mass_properties` | mass, inertia and centroid of a 16-vertex hull (14 triangles) | 578 ns |
| `b2/world_aabb_box_rotated` | a box's world AABB at a rotation (one correctly rounded sin_cos) | 261 ns |

**Reading it.** The mass half alone takes 2.2× Rust's whole insert. A scratch split shows where the time goes:

| Step | Cyrius |
|---|---|
| `ImRigidBody2d_from_desc` | 58 ns |
| `ImCollider2d_from_desc` | 405 ns |
| of which `ImColliderShape::clone`, a ball (no arrays) | 349 ns |
| `_im_b2_add_collider_mass` | 82 ns |
| of which `mass_properties` | 46 ns |

The shape copy's 349 ns is mostly one `memcpy` of the 184-byte `ColliderShape` record, and the stdlib's
`memcpy` copies a byte at a time. 8-byte chunks with a byte tail, in plain Cyrius, are 4–7× faster (the repro filed
upstream, two runs):

| Bytes | What it is | `memcpy` (byte loop) | 8-byte chunks + byte tail |
|---|---|---|---|
| 24 | a spatial hash record | 42–44 ns | 11 ns |
| 184 | a `ColliderShape` | 287–296 ns | 43–45 ns |
| 224 | an `ImRigidBody2d` | 341–361 ns | 51–54 ns |
| 4,096 | a page | 6.08–6.45 µs | 0.86–0.91 µs |

That is upstream finding **S5** (port plan §8). It was filed as an addition to ranga's open issue, which measured the
same ratio on 8 MB frames. The port also calls `memcpy`:
- for every arena insert and get (M3's body records are 224 B);
- for the spatial hash's 24-byte record moves, once per record per merge pass. How much of the M2a hash row that
  costs is to measure in the profiling before M3;
- in every JSON reader.

The port isn't changed for speed. The world AABB's 261 ns is mostly its `sin_cos` (209 ns in `math/sin_cos_cr`).

## M2 part 3: the 2D narrowphase (2026-11-09)

Cyrius-only rows, 100,000 calls of `generate_contacts_multi` on a touching pair, x86_64 ELF. Rust times its
narrowphase only inside the step benches, so the comparison waits for M3.

| Bench | Pair | First port | One `sin_cos` per polygon and contact |
|---|---|---|---|
| `b2/contacts_ball_box` | a ball on a box (one round test) | 169 ns | 132 ns |
| `b2/contacts_box_box_rotated` | two boxes, one at 0.3 rad (clipping, two points) | 1.98 µs | 1.28 µs |
| `b2/contacts_capsule_box_lying` | a capsule lying on a box (two end caps) | 560 ns | 506 ns |
| `b2/contacts_hull8_hull8` | two 8-vertex hulls | 3.29 µs | 3.18 µs |

**Reading it.**
- **The correctly rounded `sin_cos` dominates the small pairs.** It costs about 200 ns (`math/sin_cos_cr`), against
  roughly 15 ns for glibc's.
- **The box paths called it more than they needed to.** Rust's box `polygon_vertices` calls `local_to_world` once
  per vertex, and `box_to_world` rotates and translates separately: four and two calls of one angle. The port now
  takes each once. The bits are the same because `sin_cos` is pure, and all 16,133 contact rows still match. That
  is the 0.7 µs off the rotated box pair.
- **The hull pair is arithmetic.** It runs about 2 × 8 × 8 dot products for the separations and 16 edge-normal
  square roots per call, on the compiler's xmm0/xmm1 code (port plan §8).

## M2 part 4: correctly rounded hypot, and the 2D queries (2026-11-09)

Cyrius-only rows, x86_64 ELF. Rust has no bench for `hypot` or the queries alone.

| Bench | Body | Cyrius |
|---|---|---|
| `math/hypot_cr` | the correctly rounded `hypot` (ADR 0011 amendment), on (0.6, 0.8 + a step) | 168 ns |
| `math/hypot_ganita` | ganita's `hypot`, which it replaced (a shadow line now) | 33 ns |
| `b2/raycast_ball` | a ray on a ball collider of a rotated body | 389 ns |
| `b2/raycast_box_rotated` | a ray on a box at 0.4 rad (slabs in the box's frame) | 433 ns |
| `b2/raycast_capsule` | a ray on a capsule (two discs and the shaft, unioned) | 667 ns |
| `b2/raycast_hull8` | a ray on an 8-vertex hull (Cyrus–Beck) | 2.11 µs |
| `b2/overlap_sphere_box` | a sphere probe on a box (world AABB, then the contact test) | 872 ns |

**Reading it.**
- **Exact `hypot` costs 5× ganita's.** It sits between exp (104 ns) and sin_cos (201 ns), and every call is
  correctly rounded. ganita's is 1 to 2 ulp off on 24 % of the golden rows.
- **The correctly rounded primitives dominate the queries.**
  - The ball ray pays one `sin_cos` (the collider's world position) and one `hypot` (the normal).
  - The hull ray pays one `hypot` per edge, as Rust's does.
  - The sphere overlap pays four `sin_cos` of one angle: `world_aabb`, `world_pos`, and `circle_obb`'s two
    transforms. Rust takes four as well. Sharing them, as the narrowphase's box paths now do, is open for M3's
    profiling.

## M3 part 1: the 2D state (2026-11-09)

The Rust benches' own loops, on ImPhysicsState2d (PhysicsWorld's 2D calls are the backend's). Rust 1.4.0 is from
`rust-old/bench-history.csv` (criterion); Cyrius is 100,000 iterations on x86_64 ELF. As in Rust, `add_body` and
`add_joint` grow one world.

| Bench | Rust 1.4.0 | Cyrius | Ratio |
|---|---|---|---|
| `bodies/add_body` | 81.7 ns | 1.20 µs | 15× |
| `bodies/add_remove_body` | 60.9 ns | 618 ns | 10× |
| `bodies/add_body_with_collider` | 213 ns | 3.23 µs | 15× |
| `bodies/add_joint` | 23.4 ns | 1.22 µs | 52× |
| `forces/apply_force` | 3.31 ns | 45 ns | 14× |
| `forces/apply_impulse` | 1.69 ns | 41 ns | 24× |

**Reading it.**
- **The arenas copy their records with `memcpy`, which moves a byte at a time (S5).** A body is 224 B and a
  collider 320 B. A grown arena copies all its records again, and a struct returned by value is copied once more.
  `add_body_with_collider` also pays the shape clone and the mass properties (M2b).
- **`add_joint` is the worst ratio.** Rust's is a small move. The port builds the joint record by value, then
  copies it into the arena: two byte-loop copies of the `ImJointType` payload.
- **The force calls build their sums through `ImVec2_new`** and read three words through a typed pointer.

These are first numbers: correctness first, then profiling, which the M2a broadphase finding already asks for
before M3's step. A word-at-a-time `memcpy` upstream (S5) would cut most of the gap.

## M3 part 2: joints (2026-11-09)

Cyrius-only rows, 2,000 iterations, x86_64 ELF. Rust times joints only inside its world step, which M3c brings.

| Bench | Body | Cyrius |
|---|---|---|
| `b2/joints_chain20_prepare_solve` | a 20-link revolute chain: prepare its joints, 4 velocity iterations | 22.5 µs |
| `b2/joints_chain20_frame` | the same chain's whole frame without contacts (integration, joints, islands, sleep) | 54.7 µs |

**Reading it.**
- **Correctly rounded primitives again.** Each revolute joint pays two rotations (`sin_cos`, about 200 ns each) to
  prepare, and a `hypot` (168 ns) on every point-block solve. That is about 1.1 µs a joint a sub-step.
- **The rest of the frame is integration and the islands.** The island pass takes five freelist buffers a step and
  frees them, which keeps the heap flat.

## M3 part 3: the 2D step with contacts (2026-11-09)

Rust's `world/step_*` rows (criterion, the latest `rust-old/bench-history.csv` run) against the same scenes through the
backend's `step` and `finish_step`. `PhysicsWorld` itself waits for M3 part 4, but its step adds only a frame counter
and an empty particle pass. The balls sit in rows 1 apart (touching) and 2 apart vertically, and fall together; both
sides time a world that has already stepped (criterion's warm-up; 20–100 frames here). x86_64 ELF.

| Bench | Body | Rust | Cyrius | Cyrius / Rust |
|---|---|---|---|---|
| `b2/step_empty` | an empty world's frame (Rust: `world/step_empty`) | 115 ns | 438 ns | 3.8× |
| `b2/step_10_bodies` | 10 balls in rows of 5 (`world/step_10_bodies`) | 4.31 µs | 29.0 µs | 6.7× |
| `b2/step_100_bodies` | 100 balls in rows of 10 (`world/step_100_bodies`) | 101 µs | 374 µs | 3.7× |
| `b2/step_1000_bodies` | 1,000 balls in rows of 32 (`world/step_1000_bodies`) | 853 µs | 4.07 ms | 4.8× |
| `b2/step_stack5_resting` | a settled stack of five boxes, kept awake: two-point manifolds, block solver, warm start | — | 51.5 µs | — |
| `b2/joints_chain20_frame` | the 20-link chain's frame, now through `step` (broadphase included; 54.7 µs before) | — | 88.2 µs | — |

**Reading it.**
- **The whole step is closer to Rust than its parts.** These ratios (3.7–6.7×) are well below the state benches'
  10–52×. Rust's step allocates every frame (BTreeMaps for the AABBs, levels, islands and manifolds, a Vec of
  contacts), while the port's scratch is reused, so its heap stays flat.
- **Where the time goes.** The correctly rounded `sin_cos` (about 200 ns) runs in every AABB, narrowphase pair and
  manifold, and `hypot` (168 ns) in the circle contacts. The 10-ball scene pays the most for both, relative to its
  size.
- **Not acted on.** Speed is measured, not gated. These are first numbers.

## M3 part 4: the world, its particles, and a correctly rounded pow (2026-11-09)

Rust's `world/*` and `particles/*` rows (criterion, the latest `rust-old/bench-history.csv` run) against the same
scenes through `ImPhysicsWorld`. x86_64 ELF. The two `step_N_particles` rows time one step of a fresh world per sample,
as Rust's `iter_custom` does. The others step one world on and on, as criterion does: the floor scene's particles have
expired before most of its 100,000 iterations, and the emitter scene holds about 50 live particles.

| Bench | Body | Rust | Cyrius | Cyrius / Rust |
|---|---|---|---|---|
| `world/step_empty` | an empty world's step | 115 ns | 403 ns | 3.5× |
| `world/step_10_bodies` | 10 balls in rows of 5 | 4.31 µs | 28.1 µs | 6.5× |
| `world/step_100_bodies` | 100 balls in rows of 10 | 101 µs | 355 µs | 3.5× |
| `world/step_1000_bodies` | 1,000 balls in rows of 32 | 853 µs | 3.81 ms | 4.5× |
| `particles/step_100_particles` | a fresh world's first step, 100 particles | 1.95 µs | 9.92 µs | 5.1× |
| `particles/step_1000_particles` | the same with 1,000 | 5.49 µs | 93.0 µs | 17× |
| `particles/step_100_particles_with_floor` | a floor and 100 falling particles, stepped on | 444 ns | 1.89 µs | 4.3× |
| `particles/spawn_particle` | spawn one particle | 20.1 ns | 694 ns | 35× |
| `particles/emitter_step` | an emitter at 100/s, lifetime 0.5 | 493 ns | 18.5 µs | 37× |
| `math/pow_cr` | the correctly rounded pow, at the force field's form (x ≈ 4, y = 0.75) | — | 755 ns | — |
| `math/pow_ganita` | ganita's pow, the same arguments | — | 200 ns | — |

**Reading it.**
- **The world adds nothing measurable to the backend.** Its rows are within 5 % of the `b2/step_*` rows: the frame
  counter, an empty particle pass and the event copy are small.
- **The slow particle rows are S5 again.** A particle is a 136-byte record:
  - `spawn_particle` copies it three times, by the stdlib's byte-loop `memcpy` (the value parameter, the local, the
    push);
  - `emitter_step` removes one expired particle from the front about every frame, and Rust's `retain` order means
    every later record moves.

  The 1,000-particle step is one integration per particle. Most of it is the record's field traffic.
- **pow costs 3.8× ganita's.** Phase 1 decides over 99 % of arguments with a double-double log, whose series runs 4
  terms in double-double and 11 more in double, and exp's table phase. The first design ran the full double-double
  series and exp's phase 2 on every call: 5.4 µs.
- **Not acted on.** Speed is measured, not gated: S5 is filed with the cyrius memcpy issue, and pow is a first number.

## M4 part 1: glam's 3D math and the 3D backend's types (2026-11-09)

Cyrius-only rows (`tests/impetus_3d.bcyr`, the 3D profile), x86_64 ELF. Rust's `simulation.rs` times the 3D backend
only through its world, which ports in M5. The hisab rows are its allocating forms of the same operation, for ADR
0012's trade. Each hisab call leaks 24 B (rotation) or 72 B (product) of global heap.

| Bench | Body | First form | Now |
|---|---|---|---|
| `b3/math_quat_mul_vec3` | glam's `DQuat * DVec3` | 128 ns | **41 ns** |
| `b3/hisab_hquat_rotate_vec3` | hisab's rotation (divides by \|q\|² unless it is 1) | 51 ns | 51 ns |
| `b3/math_mat3_mul_mat3` | glam's `DMat3 * DMat3` | 297 ns | **86 ns** |
| `b3/hisab_m3_mul` | hisab's product | 90 ns | 89 ns |
| `b3/math_mat3_inverse` | glam's adjugate inverse | 213 ns | **50 ns** |
| `b3/gyroscopic_step` | one implicit Newton step with the energy projection | 1.28 µs | 742 ns |
| `b3/integrate_velocities` | gravity, force, the body-frame gyroscopic step and torque, damping, the speed cap | 1.95 µs | 1.10 µs |
| `b3/integrate_positions` | position and quaternion integration, renormalised; R·I⁻¹·Rᵀ | 952 ns | 523 ns |
| `b3/body_mass_box_offset` | the mass recompute for one offset box | 1.68 µs | 1.07 µs |
| `b3/world_aabb_box_rotated` | a rotated box's world AABB (eight rotated corners) | 2.04 µs | 1.23 µs |

**Reading it.** A struct passed by value is copied on entry, and its result comes back through the hidden pointer. The
first `ImQuat_mul_vec3` chained nine helpers (dots, scales, a cross, adds), each copying 24 to 48 bytes in and 24
out. Written with f64 locals in glam's operation order it does the same arithmetic in one frame, with identical bits:
the `b3_math` digest and all 7,321 rows are unchanged. The same rewrite went into the 3×3 product, the inverse and the
determinant. The value forms are now faster than hisab's and allocate nothing.

The body rows still chain helpers (`gyroscopic_step` builds and multiplies four matrices). M5's solver loops are where
to take the same step next, measured there.

## M4 part 2: the 3D narrowphase (2026-11-09)

One pair per row through `generate_contact_3d`, a reused GJK scratch, x86_64 ELF. Rust benches its narrowphase only
inside its step (M5), so the Rust column is a scratch timing for context: the same pairs through the oracle's
`contact_3d` hook, release build, best of five batches of 200,000 (`rust-old/` untouched). Cyrius is
`tests/impetus_3d.bcyr`, one batch of 100,000 (10,000 for the EPA cap row).

| Bench | Path | Rust | Cyrius, first | Cyrius, now |
|---|---|---|---|---|
| `b3/contact_ball_ball` | closed form | 41 ns | 240 ns | 237–247 ns |
| `b3/contact_ball_box_rotated` | ball against OBB | 75 ns | 747 ns | 753–776 ns |
| `b3/contact_box_box_aabb` | the AABB fast path | 49 ns | 607 ns | 602–639 ns |
| `b3/contact_box_box_obb_face` | SAT, a face contact (clipping) | 579 ns | 14.6 µs | **8.3–8.5 µs** |
| `b3/contact_box_box_obb_edge` | SAT, an edge pair | 465 ns | 12.6 µs | **6.3–8.0 µs** |
| `b3/contact_capsule_box` | five ball-against-box candidates | 451 ns | 8.2 µs | 6.2–8.2 µs |
| `b3/contact_capsule_capsule` | segment closest points | 123 ns | 2.45 µs | 2.45–2.50 µs |
| `b3/contact_hull_ball_gjk` | GJK + EPA, hull against ball | 5.50 µs | 44 µs | 54–56 µs |
| `b3/contact_hull_box_gjk` | GJK + EPA, hull against box | 4.82 µs | 49 µs | 30–49 µs |
| `b3/gjk_epa_ball_ball` | two balls through GJK, EPA to its 64-iteration cap | — | 663 µs | 684–738 µs |

The Rust hull pair `hull_hull` (two cubes 0.8 apart) took 3.41 µs. Two balls never reach GJK through
`generate_contact_3d`, so the EPA-cap row has no Rust twin.

**Reading it.** The closed forms are 6–20× Rust. Each step is a struct-valued helper call that copies its arguments,
where Rust's are inlined into registers.
- The SAT was the outlier at 25–27×: 15 overlap tests, each building six vectors to take six dot products. Reading
  the axes and half extents as f64 words in the same operation order took it to 14–17×. The polygon clip and the
  hull support got the same rewrite, and all 16,340 rows keep their bits.
- GJK + EPA is 6–11×; most of it is the EPA loop's face scan, retain and horizon.
- The GJK rows move by up to 1.6× between runs: each is one long batch (3–5 s). The ranges above are three runs.

Not pursued further here. Speed is measured, not gated, and M5's step is where the pair costs add up and get profiled.

## M4 part 3: the 3D raycast (2026-11-09)

x86_64 ELF, `tests/impetus_3d.bcyr`. Rust's `queries/raycast_100_bodies` was run on the 3D build for this row
(criterion, `cargo bench --all-features -- queries/raycast_100_bodies`, median 2,277 ns, interval 2,274–2,279). The
bench history holds only the default 2D build.

| Bench | Body | Rust | Cyrius |
|---|---|---|---|
| `queries/raycast_100_bodies` (3D) | 100 unit balls on a 10 × 10 grid 3 apart, one ray along +x | 2.28 µs | 21.1 µs (9.3×) |
| `b3/ray_sphere` | a hit, normal included | — | 220 ns |
| `b3/ray_aabb_3d` | a slab hit | — | 141 ns |
| `b3/ray_capsule` | six candidates, a side hit | — | 630 ns |

**Reading it.** Per collider, the loop rotates the offset into the world (a quaternion rotation), adds the body
position and runs the ray test. Ninety of the hundred balls are rejected at the discriminant. As in the narrowphase,
the cost is struct-valued helper calls copying their arguments. Not pursued here: M5's step is where the queries
meet the rest of the pipeline.
