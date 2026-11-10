# impetus — Current State

> Refreshed every release and at every port milestone. `CLAUDE.md` is
> preferences/process/procedures (durable); this file is **state** (volatile).

## Version

**1.4.0** (`VERSION`): the last Rust release, tag `1.4.0`. It is frozen as the oracle in `rust-old/`,
together with the unreleased port-tooling commit `55cc0b2` (the hidden `oracle` feature; `2d` without
hisab). The Cyrius port is in progress and ships as **2.0.0**; `VERSION` moves to 2.0.0 at M7.
`cyrius.cyml` reads the version through `version = "${file:VERSION}"`.

## Port progress

Plan of record: [`impetus-port-plan.md`](impetus-port-plan.md).

| Milestone | Status |
|---|---|
| M0 Initialize | ✅ **done 2026-11-09.** Steps 0–3: oracle hooks; `cyrius port` scaffold and repairs; ADRs renumbered, ADR 0008; `cyrius.cyml` with hisab 3.4.1. Steps 4–7: math probed; `scripts/oracle/` writes `tests/golden/` (constants 3 + math 18,919), deterministic, glibc-pinned, validated; `src/constants.cyr`; `tests/parity_2d.tcyr`. Step 8 ([`port-parity.md`](port-parity.md) §2): **14,875 ok, 0 failed, 0 measured, 4,046 pending, 1 accepted (ADR 0009)** with identical output bits on x86_64 ELF, x86_64 PE and aarch64; exp and atan2 on the polyfills; K = 2. Step 9 (§4): hisab leaks on the step path (ADR 0007 amendment: hisab off it until S1/H9); freelist and the step arena flat; hot loops on `f64` locals; stride arrays and struct values work; the stable-sort trap; per-profile test includes |
| M1a Foundation | ✅ **done 2026-11-09.** `src/mathx.cyr` (the `_im_` helpers: Rust-semantics min/max/clamp, the predicates, to_radians, the literal powi, exp and atan2 on the polyfills, hypot, sin_cos) closes the 4,045 pending math rows; `src/error.cyr`, `vec.cyr`, `config.cyr` (`validate`, with ADR 0008 #6), `material.cyr`, `units.cyr` (Display's number waits for S2), `force.cyr`; golden config 293 + material 2,046 + units 2,236 + force 1,308; **24,431 ok, 0 failed, 281 pending, 93 accepted**, every row 0 ulp, on x86_64 ELF, x86_64 PE and aarch64; 33 Rust tests ported; `[lib]` + `dist/impetus.cyr`; surface parity 100/100; 6 benches |
| M1b Foundation | ✅ **done 2026-11-09** (ADRs 0010, 0011). Part 1: `src/fmtf.cyr` (shortest digits) and `src/json.cyr` for the M1a types. Part 2: `src/types.cyr`, `arena.cyr`, `contact_tracker.cyr`, `spring.cyr`, with serde-compatible codecs for every type; S3/S4 found (7 pending rows). Part 3: correctly rounded exp, sin and cos (`src/crmath.cyr`), gated at 0 ulp against the corrected oracle (`tests/golden/*.cr.tsv`), so every spring step row is gated; **64,455 ok, 0 failed, 0 measured, 7 pending, 103 accepted** on all three targets; all 63 Rust tests of the M1b modules ported; surface parity 258/258 |
| M2 2D collision | ✅ **done 2026-11-09**. Part 1: the broadphase, `src/aabb_tree.cyr` and `src/spatial_hash.cyr` (2D and 3D, storage reused across rebuilds); golden `broadphase.tsv` 76 rows, all exact on all three targets and recomputed by brute force; the 18 Rust tests ported. Part 2: `src/b2_types.cyr` (bodies, colliders, world AABBs, mass properties, islands); private constants generated (`src/constants_private.cyr`); golden `b2_mass.tsv` 75 rows from world dumps, all 0 ulp on all three targets and recomputed by `validate.py`; the 12 Rust type tests ported. Part 3: `src/b2_narrowphase.cyr` (round tests, reference/incident-face clipping, capsule end caps); golden `b2_narrow.tsv` 17,854 rows, all 0 ulp against the corrected oracle (3 rows corrected) on all three targets, recomputed by a Python transcription; the 32 Rust narrowphase tests ported. Part 4: correctly rounded `hypot` (ADR 0011 amendment; `f64::hypot` gated at 0 ulp, 12 rows corrected) and `src/b2_raycast.cyr` (rays, overlap queries); golden `b2_ray.tsv` 14,754 rows, all 0 ulp or exact on all three targets (39 corrected), recomputed by the transcription; the 18 Rust raycast tests ported; surface parity 394/394, the 4 `PhysicsState2d` query loops pending M3. **97,160 ok, 0 failed, 0 measured, 7 pending, 157 accepted** |
| M3 2D dynamics | ✅ **done 2026-11-09**. Part 1: `src/b2_state.cyr`, ImPhysicsState2d (arenas, collider lists, the manifold store, tracker, islands; every `state.rs` method, and the M2 query methods); golden `b2_state.tsv` 411 scripts ending in the full backend dump, all 0 ulp on all three targets and replayed by `validate.py`'s model; the orphan rows mapped (ADR 0008 #1); surface parity 417/417. Part 2: `src/b2_joints.cyr` (all eight joint types, limits, motors, damping, breaking) and the contact-free stages of `src/b2_solver.cyr` (integration, islands, sleep, `finish_step`); the first single-step rows, `b2_step.tsv` 214 (dump in, dump out), all 0 ulp against the corrected oracle on all three targets and transcribed by `validate.py`; surface parity 421/421. Part 3: `src/b2_contacts.cyr`, the contact pipeline (both broadphases, the narrowphase and the manifold rebuild, wake on contact, warm start, the contact and position solvers) and `step`; golden `b2_contact.tsv` 418 single steps with contacts, all 0 ulp against the corrected oracle on all three targets, the whole SI step transcribed by `validate.py`; 24 more Rust tests ported; surface parity 418/418 (it now skips `#[cfg(test)]` helpers). Part 4: a correctly rounded pow in `src/crmath.cyr` (ADR 0011 amended, ADR 0009 superseded; 1,290,769 MPFR pairs; `f64::powf` rows gated at 0 ulp), `src/particle.cyr` and `src/world.cyr` (ImPhysicsWorld, the 2D step with particles, emitters, fields and particle collisions); golden `world_step.tsv` 198 rows, all 0 ulp on all three targets and transcribed by `validate.py`; 65 more Rust tests (world, particle, integration, regressions); surface parity 507/507 with 3 snapshot fns pending M6; then particle JSON (`ForceField`, `SubEmitter`, `Particle`, `ParticleEmitter` in `src/json.cyr`; `json.tsv` 937 → 1,092, all 155 new rows on the first run) |
| M4 3D collision | ✅ **done 2026-11-09** ([ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md): impetus-owned 3D math and GJK/EPA until S1/H9; H9 filed in hisab). Part 1: `src/math3.cyr` (glam 0.29.3's `DVec3`/`DQuat`/`DMat3` by value, glam's operation order) and `src/b3_types.cyr`; `[lib.3d]` and `dist/impetus-3d.cyr`; `tests/parity_3d.tcyr`; golden `b3_math.tsv` + `b3_body.tsv`; a cyrius field-store defect worked around and filed. Part 2: `src/b3_narrowphase.cyr` (closed forms, the OBB SAT with clipping, impetus's own GJK + EPA on a caller-owned scratch); `b3_narrow.tsv` 16,340 pairs, class A, all bit-identical. Part 3: `src/b3_raycast.cyr` (the ray tests and `raycast_impl`'s loop over two arenas; Rust's 3D world has no overlap queries); `b3_ray.tsv` 4,138 rays, class C, 2 corrected. **3D: 28,026 ok, 0 failed, 2 accepted** on all three targets, every row recomputed by `validate.py`; the 3D Rust tests of the three modules ported (111 assertions); surface parity 604 (26 modules) |
| M5 3D dynamics | ✅ **done 2026-11-10.** Part 1: a correctly rounded atan2 (ADR 0011 amended; 1,355,242 MPFR pairs; the 1,200 `f64::atan2` rows at 0 ulp, 1 corrected) and `src/b3_state.cyr` (`ImPhysicsState3d`: arenas, collider lists, the mass recompute, forces, body state, raycasts; ADR 0008 #1 refusals); `b3_state.tsv` 411 scripts, class C, 5 corrected, 3 orphans accepted, replayed by `validate.py`. **3D: 28,429 ok, 0 failed, 10 accepted**, 2D **98,873 ok**, identical digests on all three targets; 25 assertions in `b3_state.tcyr`; surface parity 627 (27 modules). Part 2: `src/b3_joints.cyr` (all of `backend_3d/joints.rs`) and `src/b3_solver.cyr`'s contact-free stages; `b3_step.tsv` 333 single steps at mask 0, all 0 ulp, the harness composing the contact-free sub-step with a broadphase guard (`tests/step3.cyr`); 31 assertions in `b3_joints.tcyr`; **3D: 28,762 ok, 0 failed**; surface parity 632 (28 modules). Part 3: the contact stages and `step` in `src/b3_solver.cyr`; `b3_contact.tsv` 450 single steps with contacts, 1 corrected (a glibc `hypot`), the b3_step rows through `step`; ADR 0008 #5 (part 1 missed it): the 3D `add_joint` refuses Revolute, Prismatic and motors, 3 unsupported scripts mapped; 23 assertions in `b3_contacts.tcyr`; **3D: 29,214 ok, 0 failed**; surface parity 637 (29 modules). Part 4: the 3D world, `src/world.cyr` shared by both profiles over each profile's own file (`src/b2_world.cyr`, M3's code moved; `src/b3_world.cyr`, Rust's 3D particle collisions and empty overlaps); `b3_world.tsv` 190 world steps, all 0 ulp, none corrected, transcribed by `validate.py`; the world, integration and shared regression tests run on both profiles from `tests/world_cases.cyr` and `tests/regression_cases.cyr`, with 19 3D-only assertions in `b3_world.tcyr` (74) and Rust's 3D-only regression in `b3_regressions.tcyr` (29); **3D: 29,404 ok, 0 failed**; surface parity 637 (29 modules). Then ADR 0008 #4: the 3D `ImBodyState` carries the full quaternion and angular-velocity vector (each profile its own `ImBodyState` and its JSON, `src/b2_json.cyr`, `src/b3_json.cyr`); `b3_state.tsv`'s gets and sets compare the full state from the oracle's dump, all 0 ulp; `b3_state.tcyr` 34 assertions. Next: M6 |
| M6 Snapshots | ◐ **part 1 done 2026-11-10** ([ADR 0013](../adr/0013-complete-snapshots.md)). `ImWorldSnapshot` (2D) holds a world's whole state, the backend's own records; `snapshot` and a validated `restore` that leaves the world unchanged on error (`src/snapshot.cyr`, `src/b2_snapshot.cyr`, `src/world_snapshot.cyr`); `IM_MAX_SNAPSHOT_INDEX` generated (2D: 98,874 ok, 6 pending); `tests/snapshot.tcyr` 31 assertions (round trips, step-equivalence in eight runs, 13 refusals, Rust's two struct-level restore tests); surface parity 662 (30 modules). Part 2: the binary form (version 1: little-endian words, a checksummed header, per-variant fields), `to_bytes`, `ImWorldSnapshot_from_bytes`, `serialize_world`, `deserialize_world`; `tests/snapshot.tcyr` 49 assertions (Rust's five byte-level snapshot tests and its two `#[ignore]`d restore tests through bytes, truncations, one-bit changes, 1,500 seeded corruptions, a cross-target pin); surface parity 662 with nothing pending. Part 3: the JSON form (`to_json`, `ImWorldSnapshot_from_json`; finite values exact, a non-finite one refused), the particle, emitter and event readers split for nesting; `tests/snapshot.tcyr` 58 assertions. Part 4: 3D snapshots, all three forms (`src/b3_snapshot.cyr`; Revolute, Prismatic and motors refused with IM_ERR_UNSUPPORTED, ADR 0008 #5); the shared tests in `tests/snapshot_cases.cyr` (Rust's nine snapshot tests and the byte and JSON tests, 30 assertions), run by `snapshot.tcyr` (58) and the new `b3_snapshot.tcyr` (59). Next: part 5, the fuzz port, profiling and the close |
| M7 Release 2.0.0 | ☐ |

## Toolchain

- **Cyrius pin**: `6.7.18` (`cyrius.cyml` `[package].cyrius`)
- **ganita**: 1.2.15, from the 6.7.18 stdlib bundle (`lib/ganita.cyr`)
- **hisab**: 3.4.1 (`[deps.hisab]`, commit-pinned in `cyrius.lock`)
- **MPFR**: 4.2.2 (system `libmpfr.so.6`), for the corrected oracle and the table check (development only)
- **Rust oracle**: impetus 1.4.0 + `oracle` hooks at `rust-old/`, green on rustc 1.99.1
  (2D: 298 unit + 9 integration + 7 oracle + 28 regression (2 ignored) + 2 doc; 3D: 274 + 9 + 7 + 14
  (2 ignored) + 2). Its transcendental results come from the host **glibc 2.44** (Rust std); its 3D
  vector math from glam 0.29.3 (what hisab 1.4.0 links).

## Source

- **Rust reference:** `rust-old/` (21,825 lines incl. tests), frozen; do not edit. `rust-old/docs/`,
  `rust-old/.github/` and `rust-old/scripts/` keep the 1.4.0 architecture overview, testing guide, CI
  and cargo scripts; `rust-old/bench-history.csv` is the Rust benchmark baseline.
- **Cyrius port:** `[lib]` (bundle `dist/impetus.cyr`, `cyrius distlib`) = `src/error.cyr`, `src/constants.cyr`
  (generated: `IM_MAX_SUB_STEPS`, `IM_MAX_ITERATIONS`), `src/constants_private.cyr` (generated: the ported
  modules' private constants, `_IM_*` / `_IM_B2_*` / `_IM_B3_*`), `src/crmath_data.cyr` (generated by `validate.py --write`),
  `src/crmath.cyr` (correctly rounded exp, sin, cos, hypot, pow, atan2), `src/mathx.cyr`, `src/fmtf.cyr`, `src/vec.cyr`,
  `src/config.cyr`, `src/material.cyr`, `src/units.cyr`, `src/force.cyr`, `src/types.cyr`, `src/arena.cyr`,
  `src/contact_tracker.cyr`, `src/spring.cyr`, `src/aabb_tree.cyr`, `src/spatial_hash.cyr`, `src/b2_types.cyr`,
  `src/b2_narrowphase.cyr`, `src/b2_raycast.cyr`, `src/b2_state.cyr`, `src/b2_joints.cyr`, `src/b2_contacts.cyr`, `src/b2_solver.cyr`, `src/particle.cyr`,
  `src/b2_world.cyr` (the 2D world's storage and backend steps), `src/world.cyr` (the world, shared), `src/json.cyr`, `src/b2_json.cyr`
  (the 2D `BodyState` JSON), `src/snapshot.cyr` (the shared snapshot pieces), `src/b2_snapshot.cyr` (the 2D
  `ImWorldSnapshot`), `src/world_snapshot.cyr` (`snapshot`, `restore`); plus
  the `src/main.cyr` stub. `dist/impetus.deps`: str, bayan, freelist. `[lib.3d]` (bundle `dist/impetus-3d.cyr`) = the
  shared modules through `src/spatial_hash.cyr`, then `src/math3.cyr` (glam's 3D math, ADR 0012), `src/b3_types.cyr`,
  `src/b3_narrowphase.cyr`, `src/b3_raycast.cyr`, `src/b3_state.cyr`, `src/b3_joints.cyr`, `src/b3_solver.cyr` (the
  step), `src/particle.cyr`, `src/b3_world.cyr` (the 3D world's storage and backend steps), `src/world.cyr`, `src/json.cyr`, `src/b3_json.cyr`
  (the 3D `BodyState` JSON, ADR 0008 #4), `src/snapshot.cyr`, `src/b3_snapshot.cyr` (the 3D
  `ImWorldSnapshot`), `src/world_snapshot.cyr`.
- **Generator:** `scripts/oracle/` (the `impetus-oracle` crate, by path on `rust-old/` with `oracle` + `serialize`; builds `b2` and `b3`; writes `src/constants.cyr` and `src/constants_private.cyr`), `validate.py` (also writes and checks `src/crmath_data.cyr` and the corrected oracle, `tests/golden/*.cr.tsv`), `mpfr_ref.py` (MPFR through ctypes) and `crtables.py`.

## Tests

- `tests/parity_2d.tcyr` (+ `tests/parity_core.cyr`, `tests/ulp.cyr`, `tests/lib_2d.cyr`): the golden-corpus
  harness for the shared and 2D modules — 18 modules (b2_contact, b2_mass, b2_narrow, b2_ray, b2_state, b2_step,
  broadphase, config, constants, fmtf, force, json, material, math, spring, types, units, world_step), **98,874 ok, 0
  failed, 0 measured, 6 pending (S3 3, S4 3), 166 accepted (ADR 0008 #6 92, #1 3, ADR 0011 71)** on x86_64 ELF, x86_64 PE and aarch64, with identical
  digests on all three. The core is kimiya's, with its corrected oracle (`tests/golden/<module>.cr.tsv`). The
  descriptor token helpers are `tests/parity_types.cyr`, shared with the 3D harness.
- `tests/parity_3d.tcyr` (+ `tests/lib_3d.cyr`): the `b3_*` modules — b3_body, b3_contact, b3_math, b3_narrow, b3_ray,
  b3_state, b3_step, b3_world — **29,404 ok, 0 failed, 11 accepted (ADR 0011 5, ADR 0008 #1 3, #5 3)**, on x86_64 ELF, x86_64 PE and aarch64, one
  `b3_math` digest on all three.
- Module tests: `error.tcyr`, `config.tcyr`, `material.tcyr`, `units.tcyr`, `force.tcyr` (the 44 M1a Rust tests);
  `types.tcyr` (body, collider, joint, event and query: 34), `arena.tcyr` (10), `contact_tracker.tcyr` (3),
  `spring.tcyr` (16), `aabb_tree.tcyr` (10), `spatial_hash.tcyr` (8), `b2_types.tcyr` (12; 65 assertions),
  `b2_narrowphase.tcyr` (32; 96 assertions), `b2_raycast.tcyr` (18; 45 assertions, the world-level ones on
  ImPhysicsState2d), `b2_state.tcyr` (5; 28 assertions), `b2_joints.tcyr` (1; 21 assertions), `b2_contacts.tcyr` (24; 62 assertions), `world.tcyr` (30; 55 assertions), `regressions.tcyr` (the 35 2D tests of
  `rust-old/tests/integration.rs` and `regressions.rs`; 44 assertions), `snapshot.tcyr` (2D snapshots, ADR 0013: Rust's
  two struct-level restore tests, round trips, step-equivalence, refusals, a flat heap; part 2: Rust's byte-level
  snapshot tests, truncations, one-bit changes, seeded corruptions, a cross-target pin; part 3: `snapshot_serde_json`, JSON
  round trips, step-equivalence through JSON, edits, refusals; 58 assertions; the shared cases in `tests/snapshot_cases.cyr`);
  `b3_snapshot.tcyr` (the shared cases on 3D, a tumbling body's full state through both forms, 3D step-equivalence,
  refusals including ADR 0008 #5, a pin, a JSON edit; 59 assertions), each with Cyrius-only groups (`world.tcyr` and
  `regressions.tcyr` run the cases both profiles share from `tests/world_cases.cyr` and `tests/regression_cases.cyr`); `crmath.tcyr` (78
  assertions: specials, fast = slow, the fast paths' coverage, the hardest reduction, pow's exact and tie cases,
  atan2's tiny path and subnormal ties);
  `b3_types.tcyr` (the type-level tests of `backend_3d/tests.rs`, a 600-step free spin, a flat heap; 23 assertions);
  `b3_narrowphase.tcyr` (the 37 narrowphase tests, the unhandled pairs, the GJK scratch reuse; 65 assertions);
  `b3_raycast.tcyr` (the 7 raycast tests, the state-level ones on ImPhysicsState3d, layers, a flat heap; 23
  assertions); `b3_state.tcyr` (the state-level tests of `backend_3d/tests.rs` that need no step, refusals, a joint's
  reference rotation, removals, the two field addresses, the full state and its JSON (ADR 0008 #4), a flat heap; 34
  assertions); `b3_joints.tcyr` (the 11 stepping
  tests whose colliders can't meet, the ADR 0008 #5 refusals, breaking by force, the point block alone,
  `error_reduction`, a flat heap; 31 assertions); `b3_contacts.tcyr` (the 10 stepping tests with contacts, the broadphases agreeing, the
  event sequence, sensors and layers, sleep and wake, a flat heap; 23 assertions); both step through `step`, with the
  tests' `frame` helper in `tests/frame3.cyr`; `b3_world.tcyr` (the shared world cases on the 3D world, the 3D particle
  contacts, sensors, hulls and offset colliders, the empty overlaps, the unread solver kind, the world against the
  backend's step, a flat heap; 74 assertions); `b3_regressions.tcyr` (the 21 shared integration and regression cases
  and `substeps_are_honoured_in_3d`; 29 assertions).
- `tests/impetus.bcyr`: 66 benches; `tests/impetus_3d.bcyr`: 50 (3D profile); recorded in
  `docs/benchmarks-rust-v-cyrius.md`.
- `scripts/surface_parity.py`: 662 items ok across 30 modules (M1's 13, `aabb_tree`, `spatial_hash`,
  `backend_2d/types` with its `pub(super)` items and generated private constants, `backend_2d/narrowphase`,
  `backend_2d/raycast`, `backend_2d/state`, `backend_2d/joints`, `backend_2d/solver`, `particle`, `world`,
  `backend_3d/types`, `backend_3d/narrowphase`, `backend_3d/raycast`, `backend_3d/state`, `backend_3d/joints`, `backend_3d/solver`,
  `serialize`, its snapshot records mapped onto ADR 0013's); nothing pending. It also fails when a profile's test
  mirror drifts from `cyrius.cyml`, or on a field store the cyrius 6.7.18 defect miscompiles. A fn whose own
  attributes include `#[cfg(test)]` is a test helper and not counted.
- `scripts/probes/`: `p1_memory.cyr`, `p2_vectors.bcyr` (M0 step 9, `port-parity.md` §4).
- `tests/golden/`: `b3_body.tsv` (234), `b3_state.tsv` (414), `b3_step.tsv` (333), `b3_contact.tsv` (450), `b3_world.tsv` (190), `b3_math.tsv` (7,316), `b3_narrow.tsv` (16,340), `b3_ray.tsv` (4,138), `b2_mass.tsv` (75), `b2_narrow.tsv` (17,854), `b2_ray.tsv` (14,754), `b2_contact.tsv` (418), `b2_state.tsv` (411), `b2_step.tsv` (214), `world_step.tsv` (198), `broadphase.tsv` (76), `config.tsv` (293), `constants.tsv` (3), `fmtf.tsv` (37,576), `force.tsv` (1,308), `json.tsv`
  (1,092), `material.tsv` (2,046), `math.tsv` (19,491), `spring.tsv` (1,000), `types.tsv` (1), `units.tsv`
  (2,236), `modules.txt`; the corrected oracle `math.cr.tsv` (22), `spring.cr.tsv` (1),
  `b2_contact.cr.tsv` (2), `b2_narrow.cr.tsv` (3), `b2_ray.cr.tsv` (39), `b2_step.cr.tsv` (4), `b3_ray.cr.tsv` (2), `b3_state.cr.tsv` (2) and `b3_contact.cr.tsv` (1).
- `tests/impetus.tcyr`, `.bcyr`, `.fcyr`: scaffold stubs.

## Dependencies

Declared in `cyrius.cyml`:
- stdlib: syscalls, string, alloc, str, fmt, io, vec, math, ganita, tagged, result, freelist,
  fnptr, bayan (JSON); sakshi arrives through hisab's own dependency
- hisab 3.4.1 (git tag, `dist/hisab.cyr`)
- test stdlib (`[test] stdlib`): assert, bench, overflow (tests/ulp.cyr's saturating subtract)

## Consumers

None in Cyrius yet. kiran and joshua have no local repo; aethersafha (Cyrius) declares no impetus
dependency; the bridge partners dravya, sharira and ushma have no local repo.

## Next

M5 part 4, 3D dynamics: the 3D world (`world.rs` under the `3d` feature: `PhysicsWorld` over `ImPhysicsState3d`, its
step with the particles in 3D) and the remaining 3D world rows, closing M5.
Waiting on hisab: H9 (allocator-scoped calls), filed 2026-11-09; on cyrius: S1 (the scoped allocator) and the
field-store defect (`2026-11-09-field-store-of-struct-call-copies-a-field-argument.md`), both filed 2026-11-09, and S5
(the byte-loop `memcpy`). Waiting on bayan from M1b: S3 and S4; their 6 golden rows stay pending. On ganita: a
correctly rounded exp, sin, cos, hypot, pow and atan2 tier seeded from `src/crmath.cyr`; impetus keeps crmath until it lands.
