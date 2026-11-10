//! The dump tokens both backends share: an arena's slots, a pair list and the contact tracker, in `oracle_dump()`'s
//! order (`dump2d`, `dump3d`).

use impetus::oracle::{ArenaDump, TrackerDump};

use crate::flat::handle;
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
