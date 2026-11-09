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
| M0 Initialize | ◐ **in progress.** Step 0 ✅ (oracle hooks, 2026-11-09). Steps 1–3 ✅ 2026-11-09: `cyrius port` scaffold; CLAUDE.md merged; roadmap restored; Rust CI, scripts, architecture overview and testing guide moved under `rust-old/`; `rust-old/README.md` restored for the doctest include; ADRs renumbered to four digits, ADR 0008 written; `cyrius.cyml` with hisab 3.4.1. Next: step 4 (math check), 5–7 (generator, tables, harness), 8 (baselines), 9 (probes) |
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
  vector math from glam 0.34.3.

## Source

- **Rust reference:** `rust-old/` (21,825 lines incl. tests), frozen; do not edit. `rust-old/docs/`,
  `rust-old/.github/` and `rust-old/scripts/` keep the 1.4.0 architecture overview, testing guide, CI
  and cargo scripts; `rust-old/bench-history.csv` is the Rust benchmark baseline.
- **Cyrius port:** scaffold only — the `src/main.cyr` stub. `[lib]` and `[lib.3d]` arrive in M1.

## Tests

- `tests/impetus.tcyr`, `.bcyr`, `.fcyr`: scaffold stubs. The parity harness arrives in M0 step 7.

## Dependencies

Declared in `cyrius.cyml`:
- stdlib: syscalls, string, alloc, str, fmt, io, vec, math, ganita, tagged, result, freelist,
  fnptr, sakshi
- hisab 3.4.1 (git tag, `dist/hisab.cyr`)
- test stdlib (`[test] stdlib`): assert, bench

## Consumers

None in Cyrius yet. kiran and joshua have no local repo; aethersafha (Cyrius) declares no impetus
dependency; the bridge partners dravya, sharira and ushma have no local repo.

## Next

M0 step 4 onward (port plan §4).
