# Changelog

## 0.1.0

Initial scaffold.

### Added

- Core types: `BodyDesc`, `BodyHandle`, `BodyType`, `ColliderDesc`, `ColliderHandle`, `ColliderShape`
- Physics world with deterministic stepping, body/collider/joint management
- Material presets (ice, rubber, wood, steel, bouncy)
- Force, impulse, and torque types
- Joint types (fixed, revolute, prismatic, spring, distance)
- Collision events and contact data
- Spatial queries (raycast, point query)
- Unit-aware quantities bridging abaco-core's UnitCategory
- Serialization module (feature-gated)
- Feature flags: `2d` (default), `3d`, `serialize`, `full`
- Criterion benchmarks
