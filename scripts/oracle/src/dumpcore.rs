//! The dump tokens both backends share: an arena's slots, a pair list and the contact tracker, in `oracle_dump()`'s
//! order (`dump2d`, `dump3d`), and the world's own state (the `world_step` and `b3_world` rows).

use impetus::oracle::{ArenaDump, TrackerDump};

use crate::flat::{self, handle};
use crate::tsv::{Tok, b, i};

pub fn arena(t: &mut Vec<Tok>, a: &ArenaDump) {
    t.push(Tok::V(a.slots.len()));
    for s in &a.slots {
        t.extend([b(s.occupied), i(s.generation)]);
        t.push(s.next_free.map_or(Tok::None, i));
    }
    t.push(a.free_head.map_or(Tok::None, i));
    t.push(b(a.free_list_stale));
}

pub fn pairs(t: &mut Vec<Tok>, p: &[(u64, u64)]) {
    t.push(Tok::V(p.len()));
    for &(a, bb) in p {
        t.extend([handle(a), handle(bb)]);
    }
}

pub fn tracker(t: &mut Vec<Tok>, k: &TrackerDump) {
    pairs(t, &k.prev);
    pairs(t, &k.current);
    pairs(t, &k.seen);
    t.push(b(k.recorded));
}

/// The world's own state before its backend: the particle and emitter counters, then the particles,
/// emitters and force fields in storage order.
pub fn world(d: &impetus::oracle::WorldDump) -> Vec<Tok> {
    let mut t = vec![
        i(i64::try_from(d.next_particle_id).unwrap_or(i64::MAX)),
        i(i64::try_from(d.next_emitter_id).unwrap_or(i64::MAX)),
    ];
    t.push(Tok::V(d.particles.len()));
    for p in &d.particles {
        t.extend(flat::particle(p));
    }
    t.push(Tok::V(d.emitters.len()));
    for e in &d.emitters {
        t.extend(flat::emitter(e));
    }
    t.push(Tok::V(d.force_fields.len()));
    for x in &d.force_fields {
        t.extend(flat::force_field(x));
    }
    t
}
