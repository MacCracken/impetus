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
