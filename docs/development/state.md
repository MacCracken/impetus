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
| M3 2D dynamics | ☐ |
| M4 3D collision | ☐ |
| M5 3D dynamics | ☐ |
| M6 Snapshots | ☐ |
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
  modules' private constants, `_IM_*` / `_IM_B2_*`), `src/crmath_data.cyr` (generated by `validate.py --write`),
  `src/crmath.cyr` (correctly rounded exp, sin, cos, hypot), `src/mathx.cyr`, `src/fmtf.cyr`, `src/vec.cyr`,
  `src/config.cyr`, `src/material.cyr`, `src/units.cyr`, `src/force.cyr`, `src/types.cyr`, `src/arena.cyr`,
  `src/contact_tracker.cyr`, `src/spring.cyr`, `src/aabb_tree.cyr`, `src/spatial_hash.cyr`, `src/b2_types.cyr`,
  `src/b2_narrowphase.cyr`, `src/b2_raycast.cyr`, `src/json.cyr`; plus
  the `src/main.cyr` stub. `dist/impetus.deps`: str, bayan. `[lib.3d]` arrives in M4.
- **Generator:** `scripts/oracle/` (the `impetus-oracle` crate, by path on `rust-old/` with `oracle` + `serialize`; builds `b2` and `b3`; writes `src/constants.cyr` and `src/constants_private.cyr`), `validate.py` (also writes and checks `src/crmath_data.cyr` and the corrected oracle, `tests/golden/*.cr.tsv`), `mpfr_ref.py` (MPFR through ctypes) and `crtables.py`.

## Tests

- `tests/parity_2d.tcyr` (+ `tests/parity_core.cyr`, `tests/ulp.cyr`, `tests/lib_2d.cyr`): the golden-corpus
  harness for the shared and 2D modules — 14 modules (b2_mass, b2_narrow, b2_ray, broadphase, config, constants,
  fmtf, force, json, material, math, spring, types, units), **97,160 ok, 0 failed, 0 measured, 7 pending (S3 3, S4 3,
  the M6 constant), 157 accepted (ADR 0008 #6 92, ADR 0009 1, ADR 0011 64)** on x86_64 ELF, x86_64 PE and aarch64, with identical
  digests on all three. The core is kimiya's, with its corrected oracle (`tests/golden/<module>.cr.tsv`).
- Module tests: `error.tcyr`, `config.tcyr`, `material.tcyr`, `units.tcyr`, `force.tcyr` (the 44 M1a Rust tests);
  `types.tcyr` (body, collider, joint, event and query: 34), `arena.tcyr` (10), `contact_tracker.tcyr` (3),
  `spring.tcyr` (16), `aabb_tree.tcyr` (10), `spatial_hash.tcyr` (8), `b2_types.tcyr` (12; 65 assertions),
  `b2_narrowphase.tcyr` (32; 96 assertions), `b2_raycast.tcyr` (18; 45 assertions, the world-level ones on a
  test-local world until M3), each with Cyrius-only groups; `crmath.tcyr` (23 assertions: specials, fast = slow, the fast
  path's coverage, the hardest reduction).
- `tests/impetus.bcyr`: 32 benches, recorded in `docs/benchmarks-rust-v-cyrius.md`.
- `scripts/surface_parity.py`: 394 items ok across 18 modules (M1's 13, `aabb_tree`, `spatial_hash`,
  `backend_2d/types` with its `pub(super)` items and generated private constants, `backend_2d/narrowphase`,
  `backend_2d/raycast`), 4 of them pending M3 (the `PhysicsState2d` query loops).
- `scripts/probes/`: `p1_memory.cyr`, `p2_vectors.bcyr` (M0 step 9, `port-parity.md` §4).
- `tests/golden/`: `b2_mass.tsv` (75), `b2_narrow.tsv` (17,854), `b2_ray.tsv` (14,754), `broadphase.tsv` (76), `config.tsv` (293), `constants.tsv` (3), `fmtf.tsv` (37,576), `force.tsv` (1,308), `json.tsv`
  (937), `material.tsv` (2,046), `math.tsv` (19,165), `spring.tsv` (1,000), `types.tsv` (1), `units.tsv`
  (2,236), `modules.txt`; the corrected oracle `math.cr.tsv` (21), `spring.cr.tsv` (1),
  `b2_narrow.cr.tsv` (3) and `b2_ray.cr.tsv` (39).
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

M3, 2D dynamics: `backend_2d/state.rs`, `solver.rs` and `joints.rs`, then `particle.rs` and `world.rs`. It brings
ImPhysicsState2d, whose query methods loop `src/b2_raycast.cyr`'s per-collider queries, and the step corpus
(class S and R rows). Waiting on bayan from M1b: S3 and S4, filed 2026-11-09 (port plan §8); their 6 golden rows stay pending.
Also waiting on cyrius: S1 (the scoped allocator), and S5 (the byte-loop `memcpy`, 4–7× on struct-sized copies),
filed 2026-11-09 as an addition to ranga's open issue (port plan §8). On ganita: a correctly rounded exp, sin, cos
and hypot tier seeded from `src/crmath.cyr`, filed 2026-11-09; impetus keeps crmath until it lands.
