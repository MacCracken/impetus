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
| `springs/spring_settle_frames` | a critically damped spring from rest until settled | 1.47 µs | 1.80 µs | 1.2× |
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

