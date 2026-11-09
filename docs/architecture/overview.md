# Architecture Overview

## Design Principles

- **Pure physics library** — no ECS, no rendering, no scene loading
- **Zero external physics deps** — broadphase, narrowphase, solver built in-house on hisab
- **Native f64 precision** — no lossy f32 conversions
- **Feature-gated backends** — `2d` (default) or `3d`; when both are enabled `3d` wins
- **Deterministic** — BTree collections and arena order everywhere; same inputs on the same platform give bit-identical results
- **Standalone spring API** — no world needed for animation use cases

## Module Map

| Module | Purpose |
|--------|---------|
| `body` | Rigid body types (Static, Dynamic, Kinematic), descriptors, state |
| `bridge` | Primitive-value conversions for sibling science crates (dravya, sharira, ushma) |
| `collider` | Collision shapes (Ball, Box, Capsule, ConvexHull, Segment, Heightfield, TriMesh) |
| `config` | World configuration and `WorldConfig::validate` |
| `error` | `ImpetusError` enum |
| `event` | Collision events (Started / Ongoing / Stopped) |
| `force` | Force, Impulse, Torque types |
| `joint` | Constraint types (Fixed, Revolute, Prismatic, Spring, Distance, Wheel, Rope, Mouse) |
| `material` | Physics materials with presets (ice, rubber, wood, steel, bouncy) |
| `particle` | Physics particles, emitters, force fields, sub-emitters |
| `query` | Spatial query results (RayHit) |
| `serialize` | Bitcode snapshot/restore (feature-gated) |
| `spring` | Standalone damped harmonic oscillator (1D/2D/3D), exact closed-form step |
| `units` | Unit-tagged quantities (Quantity + PhysicsUnit) |
| `world` | `PhysicsWorld` — the simulation container, particles, snapshot validation |
| `backend_2d` | Native 2D physics (internal, feature-gated) |
| `backend_3d` | Native 3D physics with DVec3/DQuat (internal, feature-gated) |
| `aabb_tree`, `spatial_hash` | Broadphase structures (internal) |
| `arena` | Generational arena for bodies, colliders, joints (internal) |
| `contact_tracker` | Frame-level Started/Ongoing/Stopped bookkeeping across sub-steps (internal) |

Contact normals point **from collider A to collider B** everywhere: the solver
applies `−j·n` to A and `+j·n` to B. Every narrowphase pairing is tested in
both argument orders.

## Simulation Pipeline

`PhysicsWorld::step()` runs `sub_steps` backend sub-steps of `timestep / sub_steps`,
then finishes the frame once:

Per sub-step (2D sequential-impulse solver; 3D is the same shape unless noted):

1. Integrate velocities — gravity and accumulated forces (forces act for the whole frame); 3D adds the implicit, energy-preserving gyroscopic term in the body frame
2. Broadphase — spatial hash (oversized/non-finite AABBs handled separately) or balanced dynamic AABB tree, with speculative expansion by velocity
3. Narrowphase — one-shot manifolds: reference/incident-face clipping for polygons (Box, ConvexHull, Segment) in 2D; GJK/EPA and 15-axis OBB SAT with face clipping in 3D; up to two end-cap contacts for capsules
4. Manifold cache — 2D rebuilds manifolds from this frame's contacts and carries impulses over from matching points; contacts that *begin* wake sleeping bodies
5. Warm start — 0.95× last frame's impulses (stored back into the accumulators)
6. Velocity solve — `velocity_iterations` passes, each solving joints then contacts: shock-propagation order, Box2D block solver for 2-point manifolds (sequential fallback when ill-conditioned), restitution from the pre-solve approach speed, Coulomb friction (static below a slip threshold), accumulated rolling resistance; joint breaking afterwards
7. Position correction — split-impulse pseudo-velocities with accumulated, clamped per-point impulses (soft ERP from `constraint_frequency`, or Baumgarte `position_correction / dt`)
8. Integrate positions (sleeping bodies don't move)
9. Islands and sleep (2D) — union-find over dynamic bodies (static ground doesn't merge islands); an island sleeps atomically once all members have rested; 3D sleeps per body
10. Record touching pairs for the frame's events

Once per frame:

11. Clear forces; emit collision events (a contact that starts and ends within one frame reports Started then Stopped)
12. Step particles — emitters (R3 low-discrepancy spread, capped spawns), gravity, softened radial and directional fields, quadratic drag, damping, collision with rotated colliders, lifetime, sub-emitters

The XPBD solver (`SolverKind::Xpbd`) is 2D-only and replaces steps 5–7 with
position-level contact and joint projection; see its docs for what it doesn't
support.

## Joints

2D joints are velocity-level constraints solved with accumulated impulses
(Box2D-style Jacobians with angular terms): rigid parts use a Baumgarte bias,
springs and joint damping are implicit soft constraints, limits are one-sided,
motors clamp their accumulated impulse to `max_force · dt`, and `break_force`
compares the linear constraint force in newtons. Fixed and Prismatic joints
hold the relative rotation they had when created; prismatic and wheel axes are
in body A's frame.

3D implements Fixed, Distance, Rope, Spring, Wheel and Mouse joints the same
way; Revolute, Prismatic and motors are accepted but have no effect yet, and
the 3D wheel axis is in world space.

## Consumers

- **Kiran** — game engine, via `kiran-physics` bridge crate
- **Aethersafha** — desktop compositor, via standalone `Spring` / `Spring2d` types
- **Simulation workloads** — headless, via `PhysicsWorld` directly

## Dependencies

- **hisab** — math (DVec3, DQuat, DMat3 for the 3D backend)
- **serde** — serialization on all public types
- **thiserror** — error enum derivation
- **bitcode** — world snapshot serialization (feature-gated)
