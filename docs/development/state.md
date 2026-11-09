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
| M1b Foundation | ☐ next: body, collider, joint, event, query descriptors; arena; contact tracker; spring |
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
- **Cyrius port:** the `src/main.cyr` stub and `src/constants.cyr` (generated: `IM_MAX_SUB_STEPS`, `IM_MAX_ITERATIONS`). `[lib]` and `[lib.3d]` arrive in M1 and M4.
- **Generator:** `scripts/oracle/` (the `impetus-oracle` crate, by path on `rust-old/` with `oracle` + `serialize`; builds `b2` and `b3`), `validate.py`.

## Tests

- `tests/parity_2d.tcyr` (+ `tests/parity_core.cyr`, `tests/ulp.cyr`, `tests/lib_2d.cyr`): the golden-corpus
  harness for the shared and 2D modules — 6 modules (config, constants, force, material, math, units),
  **24,431 ok, 0 failed, 0 measured, 281 pending (S2's 280 Display rows, the M6 constant), 93 accepted (ADR
  0008 #6 92, ADR 0009 1)** on x86_64 ELF, x86_64 PE and aarch64, with identical math digests on all three.
- `tests/error.tcyr`, `config.tcyr`, `material.tcyr`, `units.tcyr`, `force.tcyr`: 33 ported Rust tests and
  the Cyrius-only groups, 97 assertions. 11 serde tests are not carried over.
- `tests/impetus.bcyr`: 6 benches, recorded in `docs/benchmarks-rust-v-cyrius.md`.
- `scripts/surface_parity.py`: 100 items ok across the five M1a modules.
- `scripts/probes/`: `p1_memory.cyr`, `p2_vectors.bcyr` (M0 step 9, `port-parity.md` §4).
- `tests/golden/`: `config.tsv` (293), `constants.tsv` (3), `force.tsv` (1,308), `material.tsv` (2,046),
  `math.tsv` (18,919), `units.tsv` (2,236), `modules.txt`; 3.5 MB.
- `tests/impetus.tcyr`, `.bcyr`, `.fcyr`: scaffold stubs.

## Dependencies

Declared in `cyrius.cyml`:
- stdlib: syscalls, string, alloc, str, fmt, io, vec, math, ganita, tagged, result, freelist,
  fnptr, sakshi
- hisab 3.4.1 (git tag, `dist/hisab.cyr`)
- test stdlib (`[test] stdlib`): assert, bench, overflow (tests/ulp.cyr's saturating subtract)

## Consumers

None in Cyrius yet. kiran and joshua have no local repo; aethersafha (Cyrius) declares no impetus
dependency; the bridge partners dravya, sharira and ushma have no local repo.

## Next

M1b (port plan §6): the body, collider, joint, event and query descriptors (payload enums as a kind plus a struct), the generational arena on a typed stride buffer, the contact tracker on sorted arrays, and the exact spring.
