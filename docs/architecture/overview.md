# Architecture Overview

## Design Principles

- **Pure physics library** — no ECS, no rendering, no scene loading
- **Zero external physics deps** — broadphase, narrowphase, solver built in-house on hisab
- **Native f64 precision** — no lossy f32 conversions
- **Feature-gated backends** — `2d` (default) or `3d`, not both simultaneously
- **Standalone spring API** — no world needed for animation use cases

## Module Map

| Module | Purpose |
|--------|---------|
| `body` | Rigid body types (Static, Dynamic, Kinematic), descriptors, state |
| `collider` | Collision shapes (Ball, Box, Capsule, ConvexHull, Segment, Heightfield, TriMesh) |
| `config` | World configuration (timestep, gravity, solver iterations) |
| `error` | `ImpetusError` enum |
| `event` | Collision events (Started/Stopped), contact data |
| `force` | Force, Impulse, Torque types |
| `joint` | Constraint types (Fixed, Revolute, Prismatic, Spring, Distance) |
| `material` | Physics materials with presets (ice, rubber, wood, steel, bouncy) |
| `particle` | Physics particles, emitters, lifetime/drag/collision |
| `query` | Spatial queries (RayHit, PointQuery) |
| `serialize` | Bincode snapshot/restore (feature-gated) |
| `spring` | Standalone damped harmonic oscillator (1D/2D/3D) |
| `units` | Unit-aware quantities (Quantity + PhysicsUnit) |
| `world` | PhysicsWorld — the simulation container |
| `backend_2d` | Native 2D physics (internal, feature-gated) |
| `backend_3d` | Native 3D physics with DVec3/DQuat (internal, feature-gated) |

## Simulation Pipeline (per step)

1. Integrate velocities (gravity + accumulated forces)
2. Broadphase — spatial hash finds AABB overlaps
3. Narrowphase — shape-vs-shape contact generation
4. Velocity constraint solving (normal + friction impulses, angular response)
5. Joint constraint solving (positional + velocity)
6. Positional correction (Baumgarte stabilization)
7. Integrate positions
8. Clear forces
9. Generate collision events (Started/Stopped diffing)
10. Step particles (gravity, drag, damping, collider interaction, lifetime)

## Consumers

- **Kiran** — game engine, via `kiran-physics` bridge crate
- **Aethersafha** — desktop compositor, via standalone `Spring` / `Spring2d` types
- **Simulation workloads** — headless, via `PhysicsWorld` directly

## Dependencies

- **hisab** — math (DVec3, DQuat for 3D backend; glam re-exports)
- **serde** — serialization on all public types
- **thiserror** — error enum derivation
- **bincode** — world snapshot serialization (feature-gated)
