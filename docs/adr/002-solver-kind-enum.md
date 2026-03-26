# 002 — SolverKind enum for configurable solver algorithm

## Status: Accepted

## Context

The existing sequential impulse (SI) solver works well for general cases but has known stability issues with stiff constraint stacks and ragdolls. XPBD (Extended Position-Based Dynamics) solves these cases better but has different trade-offs (compliance parameters instead of velocity iterations, different warm-starting).

Rather than replacing the SI solver, we want both available and selectable per-world.

## Decision

Add a `SolverKind` enum to `config.rs`:

```rust
#[non_exhaustive]
pub enum SolverKind {
    SequentialImpulse,
    Xpbd,
}
```

With a `WorldConfig::solver` field (default: `SequentialImpulse`).

The enum is `#[non_exhaustive]` to allow future solver variants (e.g., TGS, PGS) without breaking changes. The XPBD variant is scaffolded now but not yet implemented — selecting it currently falls through to SI.

## Consequences

- Config is forward-compatible: saved worlds with `solver: "Xpbd"` will deserialize correctly
- XPBD implementation can land incrementally without config changes
- Consumers can opt into XPBD per-world when it's ready
- The `#[non_exhaustive]` means match arms need a wildcard, which is the right default for downstream code
