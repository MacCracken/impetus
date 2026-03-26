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
| `config` | World configuration (timestep, gravity, solver iterations, broadphase, solver kind) |
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

1. Integrate velocities (gravity + forces, gyroscopic torque in 3D)
2. Broadphase — spatial hash or dynamic AABB tree finds AABB overlaps (speculative expansion by velocity)
3. Narrowphase — one-shot manifold generation (Sutherland-Hodgman clipping for box/convex, single-contact for others)
4. Update manifold cache (point matching, contact reduction to 4 points max)
5. Wake sleeping bodies on contact with moving bodies
6. Warm start — apply cached impulses from previous frame
7. Velocity constraint solving — shock propagation ordering, block solver (2-point), per-point solver, friction + rolling friction
8. Joint constraint solving (positional + velocity, motors, breaking)
9. Positional correction (split impulse pseudo-velocities, soft constraints via ERP/CFM or Baumgarte fallback)
10. Integrate positions
11. Simulation islands + sleep check (union-find grouping, island-level atomic sleep)
12. Clear forces
13. Generate collision events (Started/Stopped/Ongoing diffing)
14. Step particles (gravity, drag, damping, collider interaction, lifetime, sub-emitters)

## Consumers

- **Kiran** — game engine, via `kiran-physics` bridge crate
- **Aethersafha** — desktop compositor, via standalone `Spring` / `Spring2d` types
- **Simulation workloads** — headless, via `PhysicsWorld` directly

## Dependencies

- **hisab** — math (DVec3, DQuat for 3D backend; glam re-exports)
- **serde** — serialization on all public types
- **thiserror** — error enum derivation
- **bincode** — world snapshot serialization (feature-gated)
