//! The 2D backend's full state as tokens, in `oracle_dump()`'s order: each arena's slots then the values
//! in slot order, `body_colliders` in handle order, the manifolds in key order, the contact tracker. The
//! state rows (`m_b2_state`) end with it, and the step rows (M3) will compare it frame by frame.

use impetus::oracle::{ArenaDump, Backend2dDump, TrackerDump};

use crate::flat::{self, handle};
use crate::tsv::{Tok, b, e, f, i};

fn arena(t: &mut Vec<Tok>, a: &ArenaDump) {
    t.push(Tok::V(a.slots.len()));
    for s in &a.slots {
        t.extend([b(s.occupied), i(s.generation)]);
        t.push(s.next_free.map_or(Tok::None, i));
    }
    t.push(a.free_head.map_or(Tok::None, i));
    t.push(b(a.free_list_stale));
}

fn pairs(t: &mut Vec<Tok>, p: &[(u64, u64)]) {
    t.push(Tok::V(p.len()));
    for &(a, bb) in p {
        t.extend([handle(a), handle(bb)]);
    }
}

fn tracker(t: &mut Vec<Tok>, k: &TrackerDump) {
    pairs(t, &k.prev);
    pairs(t, &k.current);
    pairs(t, &k.seen);
    t.push(b(k.recorded));
}

fn v2(t: &mut Vec<Tok>, v: [f64; 2]) {
    t.extend([f(v[0]), f(v[1])]);
}

/// The tokens of the backend dump.
pub fn backend(d: &Backend2dDump) -> Vec<Tok> {
    let mut t = Vec::new();
    arena(&mut t, &d.body_arena);
    t.push(Tok::V(d.bodies.len()));
    for x in &d.bodies {
        t.extend([handle(x.handle), e("BodyType", x.body_type)]);
        v2(&mut t, x.position);
        t.push(f(x.rotation));
        v2(&mut t, x.linear_velocity);
        t.extend([
            f(x.angular_velocity),
            f(x.linear_damping),
            f(x.angular_damping),
            b(x.fixed_rotation),
            f(x.gravity_scale),
        ]);
        v2(&mut t, x.force_accumulator);
        t.extend([
            f(x.torque_accumulator),
            f(x.mass),
            f(x.inv_mass),
            f(x.inertia),
            f(x.inv_inertia),
        ]);
        v2(&mut t, x.pseudo_velocity);
        t.push(f(x.pseudo_angular_velocity));
        v2(&mut t, x.prev_position);
        t.extend([
            f(x.prev_rotation),
            b(x.is_sleeping),
            f(x.sleep_timer),
            i(x.island_id),
        ]);
    }
    arena(&mut t, &d.collider_arena);
    t.push(Tok::V(d.colliders.len()));
    for c in &d.colliders {
        t.extend([handle(c.handle), handle(c.body)]);
        t.extend(flat::shape(&c.shape));
        t.extend([f(c.offset[0]), f(c.offset[1]), f(c.offset[2])]);
        t.extend(flat::material(&c.material));
        t.push(b(c.is_sensor));
        t.push(c.mass.map_or(Tok::None, f));
        t.extend([i(c.collision_layer), i(c.collision_mask)]);
    }
    arena(&mut t, &d.joint_arena);
    t.push(Tok::V(d.joints.len()));
    for j in &d.joints {
        t.extend([handle(j.handle), handle(j.body_a), handle(j.body_b)]);
        t.extend(flat::joint_type(&j.joint_type));
        v2(&mut t, j.local_anchor_a);
        v2(&mut t, j.local_anchor_b);
        match &j.motor {
            Some(m) => t.extend([f(m.target_velocity), f(m.max_force)]),
            None => t.push(Tok::None),
        }
        t.push(f(j.damping));
        t.push(j.break_force.map_or(Tok::None, f));
        t.push(f(j.reference_angle));
    }
    t.push(Tok::V(d.body_colliders.len()));
    for (body, cs) in &d.body_colliders {
        t.push(handle(*body));
        t.push(Tok::V(cs.len()));
        t.extend(cs.iter().map(|c| handle(*c)));
    }
    t.push(Tok::V(d.manifolds.len()));
    for m in &d.manifolds {
        t.extend([
            handle(m.collider_a),
            handle(m.collider_b),
            handle(m.body_a),
            handle(m.body_b),
        ]);
        v2(&mut t, m.normal);
        t.push(Tok::V(m.points.len()));
        for p in &m.points {
            v2(&mut t, p.local_a);
            v2(&mut t, p.local_b);
            t.extend([f(p.normal_impulse), f(p.tangent_impulse), f(p.depth)]);
        }
        t.push(f(m.rolling_impulse));
    }
    tracker(&mut t, &d.tracker);
    t
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
