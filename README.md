# Impetus

**Impetus** (Latin: *impetus* — driving force, the medieval theory of why
objects keep moving after being thrown) is a physics engine for the AGNOS
ecosystem. It provides 2D/3D rigid-body simulation with deterministic stepping,
serializable state, and unit-aware quantities.

Built on [hisab](https://github.com/MacCracken/hisab) — the AGNOS math
library — for geometry, transforms, and numerical methods. No external physics
engine dependencies; the entire simulation stack is owned in-house.

## Architecture

Impetus is a pure physics library — no ECS, no rendering, no scene loading.
It sits at the foundation layer, consumed by:

- **Kiran** — game engine (ECS integration via kiran-core, physics queries, collision callbacks)
- **Aethersafha** — desktop compositor (window spring animations, snapping physics)
- **Simulation workloads** — headless agent training environments via daimon

## Features

- **2D and 3D** — feature-gated backends built on hisab geometry
- **Native f64** — full double precision, no lossy f32 conversions
- **Deterministic** — fixed timestep, reproducible simulation for replay and network sync
- **Serializable** — bincode world snapshots for save/load and network state transfer
- **Unit-aware** — `Quantity` type with `PhysicsUnit` enum (Newtons, meters, kg, etc.)
- **Material presets** — ice, rubber, wood, steel, bouncy with physically plausible defaults
- **Joint system** — fixed, revolute, prismatic, spring, distance constraints
- **Spatial queries** — raycast, point queries
- **Collision events** — started/stopped events with detailed contact data
- **Zero external physics deps** — broadphase, narrowphase, solver all built on hisab

## Quick Start

```rust
use impetus::{PhysicsWorld, WorldConfig, BodyDesc, BodyType,
              ColliderDesc, ColliderShape, PhysicsMaterial, Force};

let mut world = PhysicsWorld::new(WorldConfig::default());

// Add a dynamic ball
let ball = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: [0.0, 10.0],
    ..Default::default()
});
world.add_collider(ball, ColliderDesc {
    shape: ColliderShape::Ball { radius: 0.5 },
    offset: [0.0, 0.0],
    material: PhysicsMaterial::rubber(),
    is_sensor: false,
    mass: None,
});

// Add a static floor
let floor = world.add_body(BodyDesc {
    body_type: BodyType::Static,
    position: [0.0, 0.0],
    ..Default::default()
});
world.add_collider(floor, ColliderDesc {
    shape: ColliderShape::Box { half_extents: [50.0, 0.5] },
    offset: [0.0, 0.0],
    material: PhysicsMaterial::wood(),
    is_sensor: false,
    mass: None,
});

// Apply a sideways force and step
world.apply_force(ball, Force::new(5.0, 0.0));
world.step();
```

## Feature Flags

| Flag | Default | Description |
|------|---------|-------------|
| `2d` | yes | 2D physics backend (hisab geometry) |
| `3d` | no | 3D physics backend |
| `serialize` | no | bincode world snapshots |
| `full` | no | all features |

## Unit-Aware Quantities

```rust
use impetus::Quantity;

let force = Quantity::newtons(9.81);
let angle = Quantity::degrees(45.0);
println!("{}", force);                  // "9.81 N"
println!("{:.4} rad", angle.to_radians()); // "0.7854 rad"
```

## Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | done | Scaffold — types, world stubs, CI/CD, benchmarks, tests |
| 2 | next | Native 2D physics backend — hisab-based broadphase, narrowphase, solver |
| 3 | | 3D backend, bincode serialization |
| 4 | | Kiran ECS bridge, Aethersafha spring physics API |

### Phase 2 scope

Native 2D physics backend in `backend_2d` module, built on hisab:
- **Broadphase** — spatial hash or sweep-and-prune using hisab AABB
- **Narrowphase** — shape-vs-shape contact generation (circle, box, capsule, segment)
- **Constraint solver** — sequential impulse solver for contacts and joints
- **Island manager** — connected-body grouping for sleep/wake optimization
- Bodies fall under gravity, collide, respond to forces/impulses/torques
- All 5 joint types functional (Fixed, Revolute, Prismatic, Spring, Distance)
- Raycasting via hisab's ray-intersection tests
- Collision events generated from actual contacts
- `get_body_state()` for reading simulation results

### Phase 3 scope

- 3D backend extending the same architecture with hisab's 3D primitives
- bincode world serialization for save/load and network sync

### Phase 4 scope

- Kiran integration: physics system that bridges kiran-core's ECS with impetus worlds
- Aethersafha: spring physics API for window animations and snapping
- Body state readback into ECS components

## Reference Code

| Crate | Relevance |
|-------|-----------|
| [hisab](https://github.com/MacCracken/hisab) | Math foundation (geometry, transforms, numerical methods) |
| [kiran](https://github.com/MacCracken/kiran) | Game engine (ECS consumer) |
| [abaco](https://github.com/MacCracken/abaco) | Unit categories (Length, Mass, Force, etc.) |
| [aethersafha](https://github.com/MacCracken/agnosticos) | Desktop compositor (spring animation consumer) |

## License

GPL-3.0 — see [LICENSE](LICENSE).
