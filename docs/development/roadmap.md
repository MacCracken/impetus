# Roadmap

## Completed

| Phase | Description |
|-------|-------------|
| 1 | Scaffold — types, CI/CD, benchmarks, tests |
| 2 | Native 2D backend — spatial hash, narrowphase, solver, particles |
| 3 | 3D backend (DVec3/DQuat), bincode serialization, `[f64;3]` API |
| 4 | Standalone spring API for Aethersafha |
| 5 | Kiran ECS bridge |
| 6 | Production hardening — `#[non_exhaustive]`, `#[must_use]`, docs, CI, supply-chain |
| 7 | Engineering backlog — see below |

## Completed from Backlog

- [x] Fix TriMesh AABB in 3D backend
- [x] Sleep/deactivation system (2D + 3D)
- [x] Collision layers/masks (broadphase filtering)
- [x] Overlap queries (sphere + AABB)
- [x] Constraint motors (revolute + prismatic)
- [x] OBB rotation-aware contacts (2D SAT)
- [x] Fuzz testing targets (contacts + serialization)
- [x] Configurable Baumgarte constants (position_slop, position_correction)
- [x] Named constants replacing magic numbers
- [x] NaN guard in spring/distance joint normalization
- [x] Joint damping
- [x] ConvexHull-vs-Ball narrowphase
- [x] u64 ID wrapping overflow handling
- [x] Particle AABB pre-filtering
- [x] `cargo semver-checks` in CI
- [x] `supply-chain/` with cargo-vet config
- [x] Cross-platform CI (ubuntu + macos + windows)
- [x] `docs/` directory (architecture, roadmap, testing guide)

## Remaining — Low Priority

- [ ] Continuous collision detection (tunneling prevention for fast small objects)
- [ ] Capsule inertia tensor accuracy (3D uses cylinder approximation)
- [ ] Extract common spatial hash into shared module (300+ LOC duplicated 2D/3D)
- [ ] Split `solve_contacts` into smaller functions (<50 lines each)
- [ ] ConvexHull-vs-ConvexHull narrowphase (SAT with N axes)
- [ ] Segment narrowphase contact generation
- [ ] Cache AABBs per body (avoid recomputing sin_cos every frame)
