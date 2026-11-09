# Impetus

> **Porting to Cyrius.** impetus 2.0 is being ported to Cyrius
> ([plan](docs/development/impetus-port-plan.md), [state](docs/development/state.md)).
> The Rust crate 1.4.0 described below is frozen in `rust-old/` as the parity oracle;
> the code examples are its API.

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

- **Kiran** — game engine (ECS integration via kiran-physics bridge)
- **Aethersafha** — desktop compositor (window spring animations, snapping physics)
- **Simulation workloads** — headless agent training environments via daimon

See [docs/architecture/overview.md](docs/architecture/overview.md) for the full module map and simulation pipeline.

## Features

- **2D and 3D** — feature-gated backends with spatial hash or dynamic AABB tree broadphase
- **Native f64** — full double precision via hisab's DVec3/DQuat
- **Deterministic** — fixed timestep and ordered (BTree) iteration: the same inputs on the same platform give bit-identical results
- **Serializable** — bitcode world snapshots for save/load and state transfer, validated on restore
- **Physics particles** — debris, projectiles with gravity, drag, collider interaction, emitters
- **Spring animation** — standalone damped harmonic oscillator (1D/2D/3D) for UI/compositor use, solved exactly so any frame time is stable
- **Unit-aware** — `Quantity` type with `PhysicsUnit` enum (Newtons, meters, kg, etc.)
- **Material presets** — ice, rubber, wood, steel, bouncy with physically plausible defaults
- **Joint system** — fixed, revolute (limits, motor), prismatic (limits, motor), spring, distance, rope, wheel and mouse joints, with breaking (revolute, prismatic and motors are 2D-only for now)
- **Spatial queries** — raycast with hit point and normal, sphere/AABB overlap
- **Collision events** — started/ongoing/stopped events per collider pair, reported once per step
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
    collision_layer: u32::MAX,
    collision_mask: u32::MAX,
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
    collision_layer: u32::MAX,
    collision_mask: u32::MAX,
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
| `3d` | no | 3D physics backend (uses hisab DVec3/DQuat); takes precedence over `2d` when both are enabled |
| `serialize` | no | bitcode world snapshots |
| `full` | no | all features (selects the 3D backend) |
| `oracle` | no | internal: state dump for the Cyrius port's generator; not public API |

Requires Rust 1.99 or newer (edition 2024).

## Documentation

- [Architecture overview](docs/architecture/overview.md)
- [Development roadmap](docs/development/roadmap.md)
- [Testing guide](docs/guides/testing.md)

## License

GPL-3.0 — see [LICENSE](LICENSE).
