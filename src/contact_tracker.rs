//! Frame-level contact bookkeeping for collision events.
//!
//! Backends report the set of touching collider pairs after every sub-step.
//! [`ContactTracker::finish`] then diffs the frame against the previous one,
//! so events are emitted once per `PhysicsWorld::step()` regardless of how many
//! sub-steps ran, and contacts that begin and end inside one frame still
//! produce a `Started` followed by a `Stopped`.

use crate::collider::ColliderHandle;
use crate::event::CollisionEvent;

/// Ordered collider pair (smaller handle first).
pub(crate) type PairKey = (ColliderHandle, ColliderHandle);

/// Order a collider pair so the smaller handle comes first.
#[inline]
#[must_use]
#[cfg_attr(not(feature = "3d"), allow(dead_code))]
pub(crate) fn pair_key(a: ColliderHandle, b: ColliderHandle) -> PairKey {
    if a.0 <= b.0 { (a, b) } else { (b, a) }
}

#[derive(Debug, Default)]
pub(crate) struct ContactTracker {
    /// Pairs touching at the end of the previous frame (sorted, unique).
    prev: Vec<PairKey>,
    /// Pairs touching at the end of the most recent sub-step (sorted, unique).
    current: Vec<PairKey>,
    /// Pairs touching at the end of an earlier sub-step this frame (sorted,
    /// unique); empty when the frame has a single sub-step.
    seen: Vec<PairKey>,
    /// Whether `current` holds a sub-step recorded this frame.
    recorded: bool,
}

/// Union of two sorted, de-duplicated slices.
fn merge_union(a: &[PairKey], b: &[PairKey]) -> Vec<PairKey> {
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        match a[i].cmp(&b[j]) {
            std::cmp::Ordering::Less => {
                out.push(a[i]);
                i += 1;
            }
            std::cmp::Ordering::Greater => {
                out.push(b[j]);
                j += 1;
            }
            std::cmp::Ordering::Equal => {
                out.push(a[i]);
                i += 1;
                j += 1;
            }
        }
    }
    out.extend_from_slice(&a[i..]);
    out.extend_from_slice(&b[j..]);
    out
}

impl ContactTracker {
    /// Bookkeeping state for the oracle dump (see `crate::oracle`).
    #[cfg(feature = "oracle")]
    pub(crate) fn oracle_dump(&self) -> crate::oracle::TrackerDump {
        let words = |v: &[PairKey]| v.iter().map(|(a, b)| (a.0, b.0)).collect();
        crate::oracle::TrackerDump {
            prev: words(&self.prev),
            current: words(&self.current),
            seen: words(&self.seen),
            recorded: self.recorded,
        }
    }

    /// Record the pairs touching at the end of a sub-step (any order;
    /// duplicates are fine).
    pub(crate) fn record_substep(&mut self, pairs: impl IntoIterator<Item = PairKey>) {
        if self.recorded && !self.current.is_empty() {
            self.seen = merge_union(&self.seen, &self.current);
        }
        self.current.clear();
        self.current.extend(pairs);
        // Manifold keys arrive sorted already; this is then a linear check.
        if !self.current.is_sorted() {
            self.current.sort_unstable();
        }
        self.current.dedup();
        self.recorded = true;
    }

    /// Close the frame and return its events in deterministic order:
    /// pairs touching now (`Started`/`Ongoing`), then pairs that stopped
    /// (`Stopped`, preceded by `Started` for contacts that began this frame).
    pub(crate) fn finish(&mut self) -> Vec<CollisionEvent> {
        let mut events = Vec::with_capacity(self.current.len() + self.prev.len());
        for &(a, b) in &self.current {
            events.push(if self.prev.binary_search(&(a, b)).is_ok() {
                CollisionEvent::Ongoing {
                    collider_a: a,
                    collider_b: b,
                }
            } else {
                CollisionEvent::Started {
                    collider_a: a,
                    collider_b: b,
                }
            });
        }
        let merged;
        let candidates: &[PairKey] = if self.seen.is_empty() {
            &self.prev
        } else {
            merged = merge_union(&self.prev, &self.seen);
            &merged
        };
        for &(a, b) in candidates {
            if self.current.binary_search(&(a, b)).is_ok() {
                continue;
            }
            if self.prev.binary_search(&(a, b)).is_err() {
                // Touched only within this frame.
                events.push(CollisionEvent::Started {
                    collider_a: a,
                    collider_b: b,
                });
            }
            events.push(CollisionEvent::Stopped {
                collider_a: a,
                collider_b: b,
            });
        }
        self.prev = std::mem::take(&mut self.current);
        self.seen.clear();
        self.recorded = false;
        events
    }

    /// Whether any pair was touching at the end of the previous frame.
    #[cfg(all(test, feature = "3d"))]
    pub(crate) fn has_previous_contacts(&self) -> bool {
        !self.prev.is_empty()
    }

    /// Forget every pair involving one of `colliders`, so no `Stopped` event
    /// is reported for them (used by backends when colliders are removed).
    #[cfg_attr(not(feature = "3d"), allow(dead_code))]
    pub(crate) fn forget_colliders(&mut self, colliders: &[ColliderHandle]) {
        let keep = |k: &PairKey| !colliders.contains(&k.0) && !colliders.contains(&k.1);
        self.prev.retain(keep);
        self.current.retain(keep);
        self.seen.retain(keep);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn k(a: u64, b: u64) -> PairKey {
        pair_key(ColliderHandle(a), ColliderHandle(b))
    }

    #[test]
    fn started_ongoing_stopped() {
        let mut t = ContactTracker::default();
        t.record_substep([k(1, 0)]);
        assert!(matches!(t.finish()[..], [CollisionEvent::Started { .. }]));
        t.record_substep([k(0, 1)]);
        assert!(matches!(t.finish()[..], [CollisionEvent::Ongoing { .. }]));
        t.record_substep([]);
        assert!(matches!(t.finish()[..], [CollisionEvent::Stopped { .. }]));
        t.record_substep([]);
        assert!(t.finish().is_empty());
    }

    #[test]
    fn transient_contact_within_frame_reports_both() {
        let mut t = ContactTracker::default();
        t.record_substep([k(0, 1)]);
        t.record_substep([]);
        let ev = t.finish();
        assert!(matches!(
            ev[..],
            [
                CollisionEvent::Started { .. },
                CollisionEvent::Stopped { .. }
            ]
        ));
    }

    #[test]
    fn substeps_collapse_to_one_event_per_pair() {
        let mut t = ContactTracker::default();
        for _ in 0..4 {
            t.record_substep([k(0, 1)]);
        }
        assert_eq!(t.finish().len(), 1);
    }
}
