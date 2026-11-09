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
| M1 Foundation | ☐ |
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

- `tests/parity_2d.tcyr` (+ `tests/parity_core.cyr`, `tests/ulp.cyr`): the golden-corpus harness for the shared
  and 2D modules — 2 modules (constants, math), **14,875 ok, 0 failed, 0 measured, 4,046 pending (M1's `_im_`
  math helpers), 1 accepted (ADR 0009)** on x86_64 ELF, x86_64 PE and aarch64, with identical output digests
  on all three. Class C K = 2 ulp against glibc (max 1; hypot 2 on 3 rows).
- `scripts/probes/`: `p1_memory.cyr`, `p2_vectors.bcyr` (M0 step 9, `port-parity.md` §4).
- `tests/golden/`: `constants.tsv` (3), `math.tsv` (18,919), `modules.txt`.
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

M1 Foundation (port plan §6): error, config, material, units, types, arena, contact_tracker, spring, and `src/mathx.cyr` (the `_im_` helpers that close the 4,045 pending math rows, `_im_atan2`).
