# 006 — Rebuild contact manifolds every frame

## Status: Accepted (supersedes the incremental accumulation described in 001)

## Context

The 2D manifold cache accumulated points across frames: a new contact point that didn't match an existing one was appended (up to four), and old points were kept while their separation stayed within a tolerance. Their depth was never refreshed and tangential drift was never checked, so a sliding or walking body dragged up to four stale "ghost" points behind it with outdated depths and lever arms — they kept supporting it after it left a ledge, and the position solver acted on stale penetration.

Accumulation existed because the narrowphase used to produce one point per frame. With reference/incident-face clipping (polygons) and end-cap contacts (capsules), the narrowphase now produces the full manifold in one pass.

## Decision

Rebuild each manifold from the current frame's contacts. Carry accumulated normal/tangent/rolling impulses over (for warm starting) from the closest unused point of last frame's manifold, matched in either body's local frame, and only when the normal hasn't flipped.

## Consequences

- No stale points; depths are always current; manifolds never exceed what the narrowphase reports (≤ 2 points in 2D).
- Warm starting is lost for a point that moves more than `MANIFOLD_MATCH_THRESHOLD` relative to both bodies in one frame (fast sliding); the solver converges from zero for that point.
- Contacts that *begin* (a key absent from last frame's cache) are what wake sleeping bodies.
