# 003 — Gyroscopic torque in 3D velocity integration

## Status: Accepted

## Context

The 3D rigid body integrator applied external torques but neglected the gyroscopic term `ω × (I·ω)`. For bodies with symmetric inertia tensors or slow rotation, this is negligible. But for fast-spinning asymmetric bodies (tops, wheels, projectiles), the missing precession term produces incorrect behavior — objects that should precess instead spin stably, and angular momentum is not conserved correctly.

## Decision

Add the gyroscopic torque correction to `RigidBody3d::integrate_velocities`:

```rust
let gyro_torque = ω.cross(I * ω);
ω += I_inv * (τ_external - gyro_torque) * dt;
```

This is the implicit Euler form — subtracting `ω × (I·ω)` from the torque before applying `I_inv`. For symmetric bodies (`I = kI`), the cross product is zero and there is no effect.

## Consequences

- Tops, gyroscopes, and projectiles now exhibit realistic precession and nutation
- Negligible performance cost (one cross product and one matrix-vector multiply per dynamic body per step)
- Bodies with uniform inertia are unaffected
- Existing tests continue to pass because the term is zero for symmetric shapes
