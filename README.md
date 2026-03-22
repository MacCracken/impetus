# Impetus

**Impetus** (Latin: *impetus* — driving force, the medieval theory of why
objects keep moving after being thrown) is a physics engine for the AGNOS
ecosystem. It wraps [rapier](https://rapier.rs/) with AGNOS-specific
integration: deterministic stepping, serializable state, TOML scene loading,
and unit-aware quantities via abaco-core compatibility.

## Architecture

Impetus sits at the foundation layer, consumed by:

- **Joshua** — game engine (ECS integration, physics queries, collision callbacks)
- **Aethersafha** — desktop compositor (window spring animations, snapping physics)
- **Simulation workloads** — headless agent training environments via daimon

## Features

- **2D and 3D** — feature-gated rapier2d/rapier3d backends
- **Deterministic** — fixed timestep, reproducible simulation for replay and network sync
- **Serializable** — bincode world snapshots for save/load and network state transfer
- **Unit-aware** — `Quantity` type with `PhysicsUnit` enum (Newtons, meters, kg, etc.) bridging abaco-core's `UnitCategory`
- **Material presets** — ice, rubber, wood, steel, bouncy with physically plausible defaults
- **Joint system** — fixed, revolute, prismatic, spring, distance constraints
- **Spatial queries** — raycast, point queries, shape casts
- **Collision events** — started/stopped events with detailed contact data

## Quick Start

```rust
use impetus::{PhysicsWorld, WorldConfig, body::{BodyDesc, BodyType},
              collider::{ColliderDesc, ColliderShape}, material::PhysicsMaterial,
              force::Force};

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
| `2d` | yes | rapier2d backend |
| `3d` | no | rapier3d backend |
| `serialize` | no | bincode world snapshots |
| `full` | no | all features |

## Unit-Aware Quantities

Impetus provides `Quantity` and `PhysicsUnit` for type-safe physics values.
These map to abaco-core's `UnitCategory` variants (Length, Mass, Time, Force,
Energy, Pressure, Angle, Frequency, Power):

```rust
use impetus::units::Quantity;

let force = Quantity::newtons(9.81);
let angle = Quantity::degrees(45.0);
println!("{}", force);           // "9.81 N"
println!("{} rad", angle.to_radians()); // "0.785... rad"
```

## Roadmap

| Phase | Description |
|-------|-------------|
| 1 | Scaffold: types, world, stub rapier integration |
| 2 | Full rapier2d integration: bodies, colliders, joints, queries |
| 3 | rapier3d support, bincode serialization, TOML scene loading |
| 4 | Joshua ECS integration, aethersafha spring physics, MCP tools |

## Reference Code

| Crate | Relevance |
|-------|-----------|
| [rapier](https://rapier.rs/) | Physics backend (2D + 3D) |
| [bevy_rapier](https://github.com/dimforge/bevy_rapier) | ECS integration patterns |
| [abaco-core](https://github.com/MacCracken/abaco) | Unit categories (Length, Mass, Force, etc.) |
| [aethersafta](https://github.com/MacCracken/agnosticos) | Desktop compositor (spring animation consumer) |
| [dhvani](https://github.com/MacCracken/agnosticos) | Audio (wave simulation reference) |
| [ranga](https://github.com/MacCracken/agnosticos) | Graphics (rendering pipeline integration) |

## License

GPL-3.0 — see [LICENSE](LICENSE).
