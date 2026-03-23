# Impetus

**Impetus** (Latin: *impetus* — driving force, the medieval theory of why
objects keep moving after being thrown) is a physics engine for the AGNOS
ecosystem. It provides 2D/3D rigid-body simulation with deterministic stepping,
serializable state, physics particles, and standalone spring animation.

Built on [hisab](https://github.com/MacCracken/hisab) — the AGNOS math
library — for vector/quaternion math (DVec3, DQuat), geometry, and numerical
methods. No external physics engine dependencies; the entire simulation stack
is owned in-house.

## Architecture

Impetus is a pure physics library — no ECS, no rendering, no scene loading.
It sits at the foundation layer, consumed by:

- **Kiran** — game engine (ECS integration via kiran-core, physics queries, collision callbacks)
- **Aethersafha** — desktop compositor (window spring animations, snapping physics)
- **Simulation workloads** — headless agent training environments via daimon

## Features

- **2D and 3D** — feature-gated backends with spatial hash broadphase
- **Native f64** — full double precision via hisab's DVec3/DQuat
- **Deterministic** — fixed timestep, reproducible simulation for replay and network sync
- **Serializable** — bincode world snapshots for save/load and network state transfer
- **Physics particles** — debris, projectiles with gravity, drag, collider interaction, emitters
- **Spring animation** — standalone damped harmonic oscillator (1D/2D/3D) for UI/compositor use
- **Unit-aware** — `Quantity` type with `PhysicsUnit` enum (Newtons, meters, kg, etc.)
- **Material presets** — ice, rubber, wood, steel, bouncy with physically plausible defaults
- **Joint system** — fixed, revolute, prismatic, spring, distance constraints
- **Spatial queries** — raycast with hit point and normal
- **Collision events** — started/stopped events with contact data
- **Zero external physics deps** — broadphase, narrowphase, solver all built on hisab

## Quick Start

```rust
use impetus::{PhysicsWorld, WorldConfig, BodyDesc, BodyType,
              ColliderDesc, ColliderShape, PhysicsMaterial, Force};

let mut world = PhysicsWorld::new(WorldConfig::default());

// Add a dynamic ball
let ball = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: [0.0, 10.0, 0.0],
    ..Default::default()
});
world.add_collider(ball, ColliderDesc {
    shape: ColliderShape::Ball { radius: 0.5 },
    offset: [0.0, 0.0, 0.0],
    material: PhysicsMaterial::rubber(),
    is_sensor: false,
    mass: None,
});

// Add a static floor
let floor = world.add_body(BodyDesc {
    body_type: BodyType::Static,
    position: [0.0, 0.0, 0.0],
    ..Default::default()
});
world.add_collider(floor, ColliderDesc {
    shape: ColliderShape::Box { half_extents: [50.0, 0.5, 50.0] },
    offset: [0.0, 0.0, 0.0],
    material: PhysicsMaterial::wood(),
    is_sensor: false,
    mass: None,
});

// Apply a sideways force and step
world.apply_force(ball, Force::new(5.0, 0.0, 0.0));
world.step();
```

## Spring Animation

Standalone springs for UI — no world needed:

```rust
use impetus::Spring2d;

let mut pos = Spring2d::critically_damped([0.0, 0.0], [500.0, 300.0], 400.0);
// Each frame:
pos.step(1.0 / 60.0);
let [x, y] = pos.position();
```

## Feature Flags

| Flag | Default | Description |
|------|---------|-------------|
| `2d` | yes | 2D physics backend |
| `3d` | no | 3D physics backend (uses hisab DVec3/DQuat) |
| `serialize` | no | bincode world snapshots |
| `full` | no | all features |

## Roadmap

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | done | Scaffold — types, CI/CD, benchmarks, tests |
| 2 | done | Native 2D backend — spatial hash, narrowphase, solver, particles |
| 3 | done | 3D backend (DVec3/DQuat), bincode serialization, `[f64;3]` API |
| 4 | done | Standalone spring API for Aethersafha |
| 5 | next | Kiran ECS bridge |

## Reference Code

| Crate | Relevance |
|-------|-----------|
| [hisab](https://github.com/MacCracken/hisab) | Math foundation (DVec3, DQuat, geometry, transforms) |
| [kiran](https://github.com/MacCracken/kiran) | Game engine (ECS consumer) |
| [aethersafha](https://github.com/MacCracken/agnosticos) | Desktop compositor (spring animation consumer) |

## License

GPL-3.0 — see [LICENSE](LICENSE).
