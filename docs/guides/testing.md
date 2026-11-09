# Testing Guide

## Running Tests

The `2d` and `3d` features select different backends, and `3d` wins when both
are enabled — so `--all-features` does **not** compile the 2D backend. Run
both:

```bash
# 2D backend (default) + serialization
cargo test --features serialize

# 3D backend + serialization
cargo test --all-features

# 3D backend only
cargo test --no-default-features --features 3d

# No backend (stub path)
cargo test --no-default-features

# Everything CI runs
make test

# Single test
cargo test --features serialize test_name
```

## Test Organization

| Location | What it covers |
|----------|----------------|
| `src/backend_2d/tests.rs`, `narrowphase.rs`, `raycast.rs`, `types.rs` | 2D narrowphase (normal orientation, clipping, rotated boxes, hulls either winding), raycasts, mass properties, joints, islands/sleep, XPBD |
| `src/backend_3d/tests.rs` | 3D contacts (GJK/EPA, 15-axis OBB SAT, capsules), world-space inertia, gyroscopic energy, friction, joints, raycasts |
| `src/aabb_tree.rs`, `src/spatial_hash.rs`, `src/contact_tracker.rs` | Broadphase balance and brute-force parity, oversized/non-finite AABBs, sub-step event bookkeeping |
| `src/world.rs`, `src/serialize.rs` | World integration: gravity, impulses, events, raycast, particles, snapshot round trips |
| `src/*.rs` (other) | Serde round trips, config validation, springs (exactness, large-dt stability), bridges, units |
| `tests/regressions.rs` | One test per defect fixed in the 1.4 audit, run against whichever backend is compiled |
| `tests/integration.rs` | Cross-module integration |
| README code blocks | Compiled as doctests (`src/lib.rs` `ReadmeDoctests`) |

Counts at the 1.4 audit: 298 unit tests on the 2D build and 274 on the 3D
build, plus 30 regression tests (two `#[ignore]`d until the 2.0 snapshot
format) and 9 integration tests. `cargo test -- --list` prints the current set.

## Testing Patterns

- **Regression first**: reproduce a defect as a failing test, then fix it
- **Physics validation**: create scenario → step N frames → assert physical behaviour (resting height, bounce speed, swing, settle)
- **Exactness**: compare against closed forms (spring), brute force (broadphase pairs) or reference formulas (mass properties)
- **Orientation**: every contact pairing is checked in both argument orders
- **Edge cases**: coincident bodies, zero-length segments, NaN/∞ inputs, zero gravity
- **Feature-gated tests**: `#[cfg(all(feature = "2d", not(feature = "3d")))]` for 2D-only, `#[cfg(feature = "3d")]` for 3D-only

## Benchmarks

```bash
# Run with history tracking (default features = 2D backend)
make bench

# Raw criterion run
cargo bench
```

38 benchmarks across 14 groups: world stepping, body management, forces,
materials, serde, particles, serialization, queries, mutation, springs,
sub-stepping, stacking, joints and broadphase.

History tracked in `bench-history.csv` with three-point comparison in `benchmarks.md`.

## Fuzzing

```bash
cargo +nightly fuzz run fuzz_contacts
cargo +nightly fuzz run fuzz_serialize   # snapshot decoding + config parsing
```

## Coverage

```bash
make coverage   # merges the 2D and 3D runs
# Report: coverage/html/index.html
```

Target: 80% project, 75% patch (enforced via codecov.yml).
