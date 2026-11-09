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
| M1b Foundation | ◐ **parts 1 and 2 done 2026-11-09** (ADR 0010). Part 1: `src/fmtf.cyr` (shortest digits; S2 solved locally) and `src/json.cyr` for the M1a types. Part 2: `src/types.cyr`, `arena.cyr`, `contact_tracker.cyr`, `spring.cyr`, with serde-compatible codecs for every type (both struct forms, one-key unit variants); golden json 937 + spring 1,000 + types 1; **63,419 ok, 0 failed, 800 measured, 7 pending, 93 accepted** on all three targets; all 63 Rust tests of the M1b modules ported; surface parity 258/258. Open: the spring's class-C gate (its sensitivity asks for exact exp/sin/cos, `port-parity.md` §7) and the bayan gaps S3/S4 (7 pending rows) |
| M2 2D collision | ☐ |
| M3 2D dynamics | ☐ |
| M4 3D collision | ☐ |
| M5 3D dynamics | ☐ |
| M6 Snapshots | ☐ |
| M7 Release 2.0.0 | ☐ |

## Toolchain

- **Cyrius pin**: `6.7.18` (`cyrius.cyml` `[package].cyrius`)
- **ganita**: 1.2.15, from the 6.7.18 stdlib bundle (`lib/ganita.cyr`)
- **hisab**: 3.4.1 (`[deps.hisab]`, commit-pinned in `cyrius.lock`)
- **Rust oracle**: impetus 1.4.0 + `oracle` hooks at `rust-old/`, green on rustc 1.99.1
  (2D: 298 unit + 9 integration + 7 oracle + 28 regression (2 ignored) + 2 doc; 3D: 274 + 9 + 7 + 14
  (2 ignored) + 2). Its transcendental results come from the host **glibc 2.44** (Rust std); its 3D
  vector math from glam 0.29.3 (what hisab 1.4.0 links).

## Source

- **Rust reference:** `rust-old/` (21,825 lines incl. tests), frozen; do not edit. `rust-old/docs/`,
  `rust-old/.github/` and `rust-old/scripts/` keep the 1.4.0 architecture overview, testing guide, CI
  and cargo scripts; `rust-old/bench-history.csv` is the Rust benchmark baseline.
- **Cyrius port:** `[lib]` (bundle `dist/impetus.cyr`, `cyrius distlib`) = `src/error.cyr`, `src/constants.cyr`
  (generated: `IM_MAX_SUB_STEPS`, `IM_MAX_ITERATIONS`), `src/mathx.cyr`, `src/fmtf.cyr`, `src/vec.cyr`,
  `src/config.cyr`, `src/material.cyr`, `src/units.cyr`, `src/force.cyr`, `src/types.cyr`, `src/arena.cyr`,
  `src/contact_tracker.cyr`, `src/spring.cyr`, `src/json.cyr`; plus the `src/main.cyr` stub. `dist/impetus.deps`: str, bayan. `[lib.3d]` arrives in M4.
- **Generator:** `scripts/oracle/` (the `impetus-oracle` crate, by path on `rust-old/` with `oracle` + `serialize`; builds `b2` and `b3`), `validate.py`.

## Tests

- `tests/parity_2d.tcyr` (+ `tests/parity_core.cyr`, `tests/ulp.cyr`, `tests/lib_2d.cyr`): the golden-corpus
  harness for the shared and 2D modules — 10 modules (config, constants, fmtf, force, json, material, math,
  spring, types, units), **63,419 ok, 0 failed, 800 measured (`Spring::step`), 7 pending (S3 3, S4 3, the M6
  constant), 93 accepted (ADR 0008 #6 92, ADR 0009 1)** on x86_64 ELF, x86_64 PE and aarch64, with identical
  math digests on all three.
- Module tests: `error.tcyr`, `config.tcyr`, `material.tcyr`, `units.tcyr`, `force.tcyr` (the 44 M1a Rust tests);
  `types.tcyr` (body, collider, joint, event and query: 34), `arena.tcyr` (10), `contact_tracker.tcyr` (3),
  `spring.tcyr` (16), each with Cyrius-only groups.
- `tests/impetus.bcyr`: 14 benches, recorded in `docs/benchmarks-rust-v-cyrius.md`.
- `scripts/surface_parity.py`: 258 items ok across the 13 M1 modules.
- `scripts/probes/`: `p1_memory.cyr`, `p2_vectors.bcyr` (M0 step 9, `port-parity.md` §4).
- `tests/golden/`: `config.tsv` (293), `constants.tsv` (3), `fmtf.tsv` (37,576), `force.tsv` (1,308), `json.tsv`
  (937), `material.tsv` (2,046), `math.tsv` (18,919), `spring.tsv` (1,000), `types.tsv` (1), `units.tsv`
  (2,236), `modules.txt`.
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

Two decisions close M1b: how the spring's step rows get gated (the rule asks for correctly rounded exp, sin
and cos on its path, `port-parity.md` §7), and whether to file S3 and S4 against bayan (port plan §8). Then M2,
2D collision: the AABB tree, the spatial hash, and 2D narrowphase and raycasts.
