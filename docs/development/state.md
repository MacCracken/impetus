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
| M0 Initialize | ◐ **in progress.** Step 0 ✅ (oracle hooks). Steps 1–3 ✅: `cyrius port` scaffold; CLAUDE.md merged; roadmap restored; Rust CI, scripts, architecture overview and testing guide moved under `rust-old/`; `rust-old/README.md` restored for the doctest include; ADRs renumbered, ADR 0008; `cyrius.cyml` with hisab 3.4.1. Steps 4–7 ✅ 2026-11-09: math counterparts probed; `scripts/oracle/` writes `tests/golden/` (constants 3 + math 18,919 rows, 2.2 MB), deterministic, glibc-pinned, `--check`ed for both builds, independently validated (`validate.py`); `src/constants.cyr` generated; `tests/parity_2d.tcyr` replays it: **3,154 ok, 0 failed, 11,722 measured, 4,046 pending** on x86_64 ELF. Next: step 8 (baselines on x86_64 PE and aarch64, class-C bounds), step 9 (probes) |
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
  and 2D modules — 2 modules (constants, math), 3,154 ok, 0 failed, 11,722 measured (class C, no bound until
  step 8), 4,046 pending (M1's `_im_` math helpers), on x86_64 ELF. Against glibc: sin, cos, atan2, exp, powf
  ≤ 1 ulp, hypot ≤ 2; the one outlier is ganita's `pow(-1, DBL_MAX)`.
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

M0 steps 8–9 (port plan §4): cross-target baselines and class-C bounds, then the probes.
