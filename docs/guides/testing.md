# Testing Guide

## Running Tests

```bash
# All tests (default 2D backend)
cargo test

# All features (3D backend + serialize)
cargo test --all-features

# 3D backend only
cargo test --features 3d

# No backend (stub path)
cargo test --no-default-features

# Single test
cargo test test_name
```

## Test Organization

| Location | Type | Count |
|----------|------|-------|
| `src/backend_2d/` | Unit tests — narrowphase, ray, AABB, mass, spatial hash, sensors, kinematic, one-shot manifold, contact reduction | ~50 |
| `src/backend_3d/` | Unit tests — 3D contacts, quaternion, mass, spatial hash, joints, gyroscopic torque | ~30 |
| `src/world.rs` | Integration — gravity, impulse, collision events, raycast, particles | ~15 |
| `src/*.rs` (other) | Unit tests — serde roundtrips, equality, constructors, display, SolverKind | ~55 |
| `src/serialize.rs` | Serialization — empty/full roundtrip, position preservation, simulation continuity | ~5 |
| `src/spring.rs` | Spring — convergence, overshoot, settling, snap, fling, serde | ~13 |
| `tests/integration.rs` | Cross-module integration | ~9 |

## Testing Patterns

- **Serde roundtrips**: serialize → deserialize → assert equal (on all public types)
- **Physics validation**: create scenario → step N frames → assert physical behavior
- **Edge cases**: coincident bodies, zero-length segments, zero gravity
- **Feature-gated tests**: `#[cfg(feature = "2d")]` for backend-specific tests

## Benchmarks

```bash
# Run with history tracking
make bench

# Raw criterion run
cargo bench
```

26 benchmarks across 7 groups: world stepping, body management, forces, materials, serde, particles, serialization.

History tracked in `bench-history.csv` with three-point comparison in `benchmarks.md`.

## Coverage

```bash
make coverage
# Report: coverage/html/index.html
```

Target: 80% project, 75% patch (enforced via codecov.yml).
