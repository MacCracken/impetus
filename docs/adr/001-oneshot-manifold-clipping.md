# 001 — One-shot manifold generation via Sutherland-Hodgman clipping

## Status: Accepted

## Context

The original 2D contact system accumulated manifold points incrementally over frames. This meant:
- First frame of collision only had 1 contact point, causing jitter
- Frame-to-frame point matching had to tolerate noisy positions
- MAX_MANIFOLD_POINTS was limited to 2

Box-box and convex-convex collisions benefit from knowing all contact points on the first frame.

## Decision

Implement Sutherland-Hodgman edge clipping for OBB-OBB and ConvexHull-ConvexHull pairs:

1. SAT finds the minimum penetration axis and reference face
2. The incident edge (most anti-parallel to the normal) is clipped against the reference face's side planes
3. Clipped points behind the reference face become contact points
4. Area-maximization reduction selects the best 4 points when more are produced

Single-contact shapes (ball, capsule, segment) still use the existing `generate_contact` path. The new `generate_contacts_multi` dispatches appropriately.

## Consequences

- Better first-frame contact quality for box stacks and convex shapes
- MAX_MANIFOLD_POINTS increased from 2 to 4
- Block solver (2-point) still handles the common case; manifolds with 3-4 points use the per-point solver path
- Slightly more narrowphase compute per box-box pair (clipping vs single support point), but fewer solver iterations needed for convergence
