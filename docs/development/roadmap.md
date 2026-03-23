# Roadmap

## Completed

| Phase | Description |
|-------|-------------|
| 1 | Scaffold — types, CI/CD, benchmarks, tests |
| 2 | Native 2D backend — spatial hash, narrowphase, solver, particles |
| 3 | 3D backend (DVec3/DQuat), bincode serialization, `[f64;3]` API |
| 4 | Standalone spring API for Aethersafha |
| 5 | Kiran ECS bridge (kiran-physics crate) |
| 6 | Production hardening — `#[non_exhaustive]`, `#[must_use]`, docs, CI, supply-chain |

## Recently Completed (from backlog)

- [x] Fix TriMesh AABB in 3D backend (was zero-volume)
- [x] Sleep/deactivation system (2D backend)
- [x] Collision layers/masks (broadphase filtering)
- [x] Overlap queries (sphere + AABB)
- [x] `cargo semver-checks` in CI
- [x] `supply-chain/` with cargo-vet config
- [x] Cross-platform CI (ubuntu + macos + windows)
- [x] `docs/` directory (architecture, roadmap, testing guide)

## Engineering Backlog

### High — Missing Features

- [ ] Particle spatial hashing (currently O(P*C) per frame)
- [ ] Constraint motors (revolute/prismatic driven motion)
- [ ] OBB (oriented bounding box) rotation-aware contacts
- [ ] Sleep system for 3D backend (done in 2D only)

### High — Production Readiness

- [ ] Fuzz testing targets (contact generation, serialization)

### Medium — Performance

- [ ] Cache AABBs per body (avoid recomputing sin_cos every frame)
- [ ] Reduce HashMap lookups in contact solver inner loop
- [ ] Configurable Baumgarte constants (currently hardcoded slop=0.01, percent=0.2)

### Medium — Refactoring

- [ ] Extract common spatial hash into shared module (300+ LOC duplicated 2D/3D)
- [ ] Split `solve_contacts` into smaller functions (<50 lines each)
- [ ] Replace magic numbers with named constants
- [ ] NaN guard in spring joint normalization (check dist_sq not dist)

### Low

- [ ] Continuous collision detection (tunneling prevention)
- [ ] Joint damping (separate from body damping)
- [ ] u64 ID overflow handling (checked increment or wrapping strategy)
- [ ] Capsule inertia tensor accuracy (3D uses cylinder approximation)
- [ ] ConvexHull / Segment narrowphase contact generation
