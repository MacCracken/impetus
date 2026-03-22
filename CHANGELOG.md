# Changelog

## 0.1.0

### Phase 1 — Scaffold

- Core types: `BodyDesc`, `BodyHandle`, `BodyType`, `BodyState`, `ColliderDesc`, `ColliderHandle`, `ColliderShape`
- Physics world with deterministic stepping, body/collider/joint management
- Material presets (ice, rubber, wood, steel, bouncy)
- Force, impulse, and torque types with constructors and magnitude helpers
- Joint types (fixed, revolute, prismatic, spring, distance)
- Collision events and contact data
- Spatial query types (raycast, point query)
- Unit-aware quantities with PhysicsUnit enum (14 units, display formatting)
- Error types with PartialEq for testability
- Feature flags: `2d` (default), `3d`, `serialize`, `full`
- Send + Sync compile-time assertions on all public types
- Full re-exports: Torque, PointQuery, ContactData, ContactPoint, BodyState, Quantity, PhysicsUnit

### Infrastructure

- CI/CD: GitHub Actions (check, test, security audit, supply chain, MSRV, coverage, release)
- Criterion benchmarks (22 benchmarks across 5 groups) with bench-history.sh tracking
- Integration tests (9 cross-module tests)
- Example: basic_physics
- Makefile with coverage target
- codecov.yml (80% project / 75% patch targets)

### Audit / Refactor

- Removed unused dependencies (uuid, glam, toml, tracing)
- Moved serde_json to dev-dependencies
- Added PartialEq to all public types
- Added missing constructors (Torque::new, Impulse::at_point, Impulse::magnitude)
- Removed redundant default_damping() function
- Fixed remove_body to use crate::Result
