# 005 — Velocity-level joint constraints

## Status: Accepted

## Context

Joints were solved by nudging positions directly: Fixed/Revolute/Distance/Rope moved each body half the error, with no mass weighting, no angular terms and no velocity correction, while Spring/Wheel/Mouse/damping applied an explicit force inside the solver iteration loop. The result:

- velocity along a constraint grew without bound under gravity (a rope stretched to 2.9 m of its 2 m length; hanging bodies never slept);
- an anchor offset from a body's centre never produced torque (a pinned rod didn't swing; Fixed was the same as Revolute);
- spring stiffness and motor torque were multiplied by `velocity_iterations`;
- `break_force` compared metres of error for some joints and newtons for others.

## Decision

Solve joints the way contacts are solved: as velocity-level constraints with accumulated, clamped impulses (Box2D-style Jacobians including `r × n` angular terms), prepared once per sub-step and interleaved with the contact solver in each velocity iteration.

- Rigid parts (point, distance, angle, axis) use a Baumgarte bias `0.2·C/dt`.
- Springs, wheel suspension, mouse joints and joint damping are implicit soft constraints (`γ = 1/(dt(c + dt·k))`), so they are stable for stiff springs and converge to the same answer for any iteration count.
- Limits are one-sided (speculative) rows; motors are velocity rows whose accumulated impulse is clamped to `|max_force|·dt` (NaN → 0, never a `clamp` panic).
- Fixed and Prismatic hold the relative rotation captured when the joint is created; prismatic/wheel axes are in body A's frame (2D).
- `break_force` compares the linear constraint force (accumulated impulse / dt) in newtons.

## Consequences

- Joint behaviour changes for every existing joint (it now matches the documented physics). Spring joints follow implicit Euler (`Δv = −k·x·dt / (1 + k·dt²/m)`) rather than explicit Euler.
- Joint impulses are not warm-started across frames yet; long chains converge more slowly than they could.
- The XPBD path keeps its own joint projection; prismatic, wheel and mouse joints, motors, limits and breaking are not supported there (documented on `SolverKind::Xpbd`).
- 3D implements the same approach for Fixed, Distance, Rope, Spring, Wheel and Mouse; Revolute, Prismatic and motors remain to be done.
