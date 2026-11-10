//! The 3D backend's full state as tokens, in `oracle_dump()`'s order: each arena's slots then the values in slot
//! order, `body_colliders` in handle order, the contact tracker (the 3D backend keeps no manifolds). The state rows
//! (`m_b3_state`) end with it, and the step rows (M5) compare it frame by frame.

use impetus::oracle::Backend3dDump;

use crate::dumpcore::{arena, tracker};
use crate::flat::{self, handle};
use crate::tsv::{Tok, b, e, f, i};

fn fs(t: &mut Vec<Tok>, v: &[f64]) {
    t.extend(v.iter().map(|&x| f(x)));
}

fn mat(t: &mut Vec<Tok>, m: &[[f64; 3]; 3]) {
    for col in m {
        fs(t, col);
    }
}

/// The tokens of the backend dump.
pub fn backend(d: &Backend3dDump) -> Vec<Tok> {
    let mut t = Vec::new();
    arena(&mut t, &d.body_arena);
    t.push(Tok::V(d.bodies.len()));
    for x in &d.bodies {
        t.extend([handle(x.handle), e("BodyType", x.body_type)]);
        fs(&mut t, &x.position);
        fs(&mut t, &x.rotation);
        fs(&mut t, &x.linear_velocity);
        fs(&mut t, &x.angular_velocity);
        t.extend([f(x.linear_damping), f(x.angular_damping), b(x.fixed_rotation), f(x.gravity_scale)]);
        fs(&mut t, &x.force_accumulator);
        fs(&mut t, &x.torque_accumulator);
        t.extend([f(x.mass), f(x.inv_mass)]);
        mat(&mut t, &x.inertia);
        mat(&mut t, &x.inv_inertia);
        mat(&mut t, &x.inv_inertia_world);
        fs(&mut t, &x.pseudo_velocity);
        fs(&mut t, &x.pseudo_angular_velocity);
        t.extend([b(x.is_sleeping), f(x.sleep_timer)]);
    }
    arena(&mut t, &d.collider_arena);
    t.push(Tok::V(d.colliders.len()));
    for c in &d.colliders {
        t.extend([handle(c.handle), handle(c.body)]);
        t.extend(flat::shape(&c.shape));
        fs(&mut t, &c.offset);
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
        fs(&mut t, &j.local_anchor_a);
        fs(&mut t, &j.local_anchor_b);
        fs(&mut t, &j.reference_rotation);
        match &j.motor {
            Some(m) => t.extend([f(m.target_velocity), f(m.max_force)]),
            None => t.push(Tok::None),
        }
        t.push(f(j.damping));
        t.push(j.break_force.map_or(Tok::None, f));
    }
    t.push(Tok::V(d.body_colliders.len()));
    for (body, cs) in &d.body_colliders {
        t.push(handle(*body));
        t.push(Tok::V(cs.len()));
        t.extend(cs.iter().map(|c| handle(*c)));
    }
    tracker(&mut t, &d.tracker);
    t
}
