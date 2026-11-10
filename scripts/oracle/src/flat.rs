//! Flattening of the oracle's public structs into golden tokens: fields in declaration order, an
//! `Option` as `n:` or its value's tokens, an enum as `e:<Enum>.<Variant>`. The Cyrius dispatchers
//! read and write the same order.

use impetus::WorldConfig;
use impetus::body::{BodyDesc, BodyState};
use impetus::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use impetus::event::{CollisionEvent, ContactData, ContactPoint};
use impetus::force::{Force, Impulse};
use impetus::joint::{JointDesc, JointType};
use impetus::material::PhysicsMaterial;
use impetus::query::{PointQuery, RayHit};
use impetus::spring::{Spring, Spring2d, Spring3d};
use impetus::units::Quantity;

use crate::tsv::{Tok, b, e, f, i};

/// The 15 tokens of a `WorldConfig`.
pub fn config(c: &WorldConfig) -> Vec<Tok> {
    vec![
        f(c.timestep),
        f(c.gravity[0]),
        f(c.gravity[1]),
        f(c.gravity[2]),
        i(c.velocity_iterations),
        i(c.position_iterations),
        b(c.deterministic),
        i(i64::try_from(c.step).unwrap_or(i64::MAX)),
        f(c.position_slop),
        f(c.position_correction),
        f(c.max_velocity),
        i(c.sub_steps),
        f(c.constraint_frequency),
        f(c.constraint_damping_ratio),
        e("BroadphaseKind", c.broadphase),
        e("SolverKind", c.solver),
    ]
}

/// The tokens of a `PhysicsMaterial` (7 fields; `static_friction` is `n:` or one float).
pub fn material(m: &PhysicsMaterial) -> Vec<Tok> {
    let mut t = vec![f(m.friction), f(m.restitution), f(m.density)];
    t.push(m.static_friction.map_or(Tok::None, f));
    t.push(f(m.rolling_friction));
    t.push(e("CombineRule", m.friction_combine));
    t.push(e("CombineRule", m.restitution_combine));
    t
}

/// The 2 tokens of a `Quantity`.
pub fn quantity(q: &Quantity) -> Vec<Tok> {
    vec![f(q.value), e("PhysicsUnit", q.unit)]
}

fn vector_and_point(v: [f64; 3], p: Option<[f64; 3]>) -> Vec<Tok> {
    let mut t = vec![f(v[0]), f(v[1]), f(v[2])];
    match p {
        Some(p) => t.extend([f(p[0]), f(p[1]), f(p[2])]),
        None => t.push(Tok::None),
    }
    t
}

/// The tokens of a `Force`: the vector, then `n:` or the point.
pub fn force(x: &Force) -> Vec<Tok> {
    vector_and_point(x.vector, x.point)
}

/// The tokens of an `Impulse`: the vector, then `n:` or the point.
pub fn impulse(x: &Impulse) -> Vec<Tok> {
    vector_and_point(x.vector, x.point)
}

// ── M1b: bodies, colliders, joints, events, queries, springs ──
// A handle is `i:` of its bit pattern (two's complement above i64::MAX, as the port's word).
// A payload enum is `e:<Enum>.<Variant>` then the variant's fields; a `Vec` is `v:<len>` then its
// elements.

pub fn handle(x: u64) -> Tok {
    Tok::I(x.cast_signed())
}

fn opt_f(x: Option<f64>) -> Tok {
    x.map_or(Tok::None, f)
}

fn v3(t: &mut Vec<Tok>, x: [f64; 3]) {
    t.extend([f(x[0]), f(x[1]), f(x[2])]);
}

fn v2(t: &mut Vec<Tok>, x: [f64; 2]) {
    t.extend([f(x[0]), f(x[1])]);
}

fn variant(name: &'static str, v: &'static str) -> Tok {
    Tok::E(name, v.to_string())
}

/// The tokens of a `BodyDesc` (12, `gravity_scale` `n:` or one float).
pub fn body_desc(d: &BodyDesc) -> Vec<Tok> {
    let mut t = vec![e("BodyType", d.body_type)];
    v3(&mut t, d.position);
    t.push(f(d.rotation));
    v3(&mut t, d.linear_velocity);
    t.extend([
        f(d.angular_velocity),
        f(d.linear_damping),
        f(d.angular_damping),
        b(d.fixed_rotation),
        opt_f(d.gravity_scale),
    ]);
    t
}

/// The 11 tokens of a `BodyState`.
pub fn body_state(s: &BodyState) -> Vec<Tok> {
    let mut t = vec![handle(s.handle.0), e("BodyType", s.body_type)];
    v3(&mut t, s.position);
    t.push(f(s.rotation));
    v3(&mut t, s.linear_velocity);
    t.extend([f(s.angular_velocity), b(s.is_sleeping)]);
    t
}

/// The Rust variant name of a shape.
pub fn shape_name(s: &ColliderShape) -> &'static str {
    match s {
        ColliderShape::Box { .. } => "Box",
        ColliderShape::Ball { .. } => "Ball",
        ColliderShape::Capsule { .. } => "Capsule",
        ColliderShape::ConvexHull { .. } => "ConvexHull",
        ColliderShape::TriMesh { .. } => "TriMesh",
        ColliderShape::Heightfield { .. } => "Heightfield",
        ColliderShape::Segment { .. } => "Segment",
        _ => "Unknown",
    }
}

/// The tokens of a `ColliderShape`: the variant, then its fields.
pub fn shape(s: &ColliderShape) -> Vec<Tok> {
    let mut t = vec![variant("ColliderShape", shape_name(s))];
    match s {
        ColliderShape::Box { half_extents } => v3(&mut t, *half_extents),
        ColliderShape::Ball { radius } => t.push(f(*radius)),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => t.extend([f(*half_height), f(*radius)]),
        ColliderShape::ConvexHull { points } => {
            t.push(Tok::V(points.len()));
            for p in points {
                v3(&mut t, *p);
            }
        }
        ColliderShape::TriMesh { vertices, indices } => {
            t.push(Tok::V(vertices.len()));
            for p in vertices {
                v3(&mut t, *p);
            }
            t.push(Tok::V(indices.len()));
            for ix in indices {
                t.extend([i(ix[0]), i(ix[1]), i(ix[2])]);
            }
        }
        ColliderShape::Heightfield { heights, scale } => {
            t.push(Tok::V(heights.len()));
            t.extend(heights.iter().map(|&h| f(h)));
            v3(&mut t, *scale);
        }
        ColliderShape::Segment { a, b: bb } => {
            v3(&mut t, *a);
            v3(&mut t, *bb);
        }
        _ => {}
    }
    t
}

/// The tokens of a `ColliderDesc`: the shape's, the offset, the material's, then the rest.
pub fn collider_desc(d: &ColliderDesc) -> Vec<Tok> {
    let mut t = shape(&d.shape);
    v3(&mut t, d.offset);
    t.extend(material(&d.material));
    t.extend([
        b(d.is_sensor),
        opt_f(d.mass),
        i(d.collision_layer),
        i(d.collision_mask),
    ]);
    t
}

/// The Rust variant name of a joint type.
pub fn joint_name(j: &JointType) -> &'static str {
    match j {
        JointType::Fixed => "Fixed",
        JointType::Revolute { .. } => "Revolute",
        JointType::Prismatic { .. } => "Prismatic",
        JointType::Spring { .. } => "Spring",
        JointType::Distance { .. } => "Distance",
        JointType::Wheel { .. } => "Wheel",
        JointType::Rope { .. } => "Rope",
        JointType::Mouse { .. } => "Mouse",
        _ => "Unknown",
    }
}

fn limits(t: &mut Vec<Tok>, l: Option<[f64; 2]>) {
    match l {
        Some(l) => v2(t, l),
        None => t.push(Tok::None),
    }
}

/// The tokens of a `JointType`: the variant, then its fields (`limits` `n:` or two floats).
pub fn joint_type(j: &JointType) -> Vec<Tok> {
    let mut t = vec![variant("JointType", joint_name(j))];
    match j {
        JointType::Revolute { anchor, limits: l } => {
            v2(&mut t, *anchor);
            limits(&mut t, *l);
        }
        JointType::Prismatic { axis, limits: l } => {
            v2(&mut t, *axis);
            limits(&mut t, *l);
        }
        JointType::Spring {
            rest_length,
            stiffness,
            damping,
        } => t.extend([f(*rest_length), f(*stiffness), f(*damping)]),
        JointType::Distance { length } => t.push(f(*length)),
        JointType::Wheel {
            axis,
            stiffness,
            damping,
        } => {
            v2(&mut t, *axis);
            t.extend([f(*stiffness), f(*damping)]);
        }
        JointType::Rope { max_length } => t.push(f(*max_length)),
        JointType::Mouse {
            target,
            stiffness,
            damping,
            max_force,
        } => {
            v3(&mut t, *target);
            t.extend([f(*stiffness), f(*damping), f(*max_force)]);
        }
        _ => {}
    }
    t
}

/// The tokens of a `JointDesc` (`motor` `n:` or two floats).
pub fn joint_desc(d: &JointDesc) -> Vec<Tok> {
    let mut t = vec![handle(d.body_a.0), handle(d.body_b.0)];
    t.extend(joint_type(&d.joint_type));
    v2(&mut t, d.local_anchor_a);
    v2(&mut t, d.local_anchor_b);
    match &d.motor {
        Some(m) => t.extend([f(m.target_velocity), f(m.max_force)]),
        None => t.push(Tok::None),
    }
    t.extend([f(d.damping), opt_f(d.break_force)]);
    t
}

/// The 3 tokens of a `CollisionEvent`.
pub fn event(x: &CollisionEvent) -> Vec<Tok> {
    let (name, a, bb) = match x {
        CollisionEvent::Started {
            collider_a,
            collider_b,
        } => ("Started", collider_a, collider_b),
        CollisionEvent::Stopped {
            collider_a,
            collider_b,
        } => ("Stopped", collider_a, collider_b),
        CollisionEvent::Ongoing {
            collider_a,
            collider_b,
        } => ("Ongoing", collider_a, collider_b),
        _ => ("Unknown", &ColliderHandle(0), &ColliderHandle(0)),
    };
    vec![variant("CollisionEvent", name), handle(a.0), handle(bb.0)]
}

/// The 7 tokens of a `ContactPoint`.
pub fn contact_point(p: &ContactPoint) -> Vec<Tok> {
    let mut t = Vec::new();
    v3(&mut t, p.local_a);
    v3(&mut t, p.local_b);
    t.push(f(p.impulse));
    t
}

/// The tokens of a `ContactData`: the pair, normal, depth, then `v:<n>` points.
pub fn contact_data(c: &ContactData) -> Vec<Tok> {
    let mut t = vec![handle(c.collider_a.0), handle(c.collider_b.0)];
    v3(&mut t, c.normal);
    t.push(f(c.depth));
    t.push(Tok::V(c.points.len()));
    for p in &c.points {
        t.extend(contact_point(p));
    }
    t
}

/// The 8 tokens of a `RayHit`.
pub fn ray_hit(h: &RayHit) -> Vec<Tok> {
    let mut t = vec![handle(h.collider.0)];
    v3(&mut t, h.point);
    v3(&mut t, h.normal);
    t.push(f(h.distance));
    t
}

/// The 3 tokens of a `PointQuery`.
pub fn point_query(q: &PointQuery) -> Vec<Tok> {
    vec![handle(q.collider.0), f(q.distance), b(q.is_inside)]
}

/// The 6 tokens of a `Spring`, its serialized fields in order. `settle_threshold` has no getter, so
/// it comes from serde (the cache is `#[serde(skip)]`), which writes a non-finite one as `null`:
/// every spring the corpus flattens has a finite threshold.
pub fn spring(s: &Spring) -> Vec<Tok> {
    let v = serde_json::to_value(s).unwrap_or_default();
    let threshold = v
        .get("settle_threshold")
        .and_then(serde_json::Value::as_f64)
        .expect("a finite settle threshold");
    vec![
        f(s.position()),
        f(s.velocity()),
        f(s.target()),
        f(s.stiffness()),
        f(s.damping()),
        f(threshold),
    ]
}

/// The 12 tokens of a `Spring2d`: x's, then y's.
pub fn spring2d(s: &Spring2d) -> Vec<Tok> {
    let mut t = spring(&s.x);
    t.extend(spring(&s.y));
    t
}

/// The 18 tokens of a `Spring3d`.
pub fn spring3d(s: &Spring3d) -> Vec<Tok> {
    let mut t = spring(&s.x);
    t.extend(spring(&s.y));
    t.extend(spring(&s.z));
    t
}

/// The tokens of a particle (handle, position, velocity, the six scalars, then `n:` or the
/// sub-emitter's count and five scalars).
pub fn particle(p: &impetus::Particle) -> Vec<Tok> {
    let mut t = vec![handle(p.handle.0)];
    v3(&mut t, p.position);
    v3(&mut t, p.velocity);
    t.extend([
        f(p.lifetime),
        f(p.radius),
        f(p.drag),
        f(p.restitution),
        f(p.gravity_scale),
        f(p.damping),
    ]);
    match &p.on_death_emit {
        Some(s) => t.extend([
            i(s.count),
            f(s.speed),
            f(s.lifetime),
            f(s.radius),
            f(s.gravity_scale),
            f(s.restitution),
        ]),
        None => t.push(Tok::None),
    }
    t
}

/// The tokens of an emitter, its private spawn accumulator read through serde (which carries it).
pub fn emitter(e: &impetus::ParticleEmitter) -> Vec<Tok> {
    let acc = serde_json::to_value(e)
        .ok()
        .and_then(|v| v.get("accumulator").and_then(serde_json::Value::as_f64))
        .unwrap_or(f64::NAN);
    let mut t = vec![handle(e.handle.0)];
    v3(&mut t, e.position);
    v3(&mut t, e.velocity);
    v3(&mut t, e.velocity_spread);
    t.extend([
        f(e.rate),
        f(e.particle_lifetime),
        f(e.particle_radius),
        f(e.particle_restitution),
        f(e.particle_gravity_scale),
        f(e.particle_damping),
        b(e.active),
        f(acc),
    ]);
    t
}

/// The tokens of a force field: its variant, then its fields.
pub fn force_field(x: &impetus::ForceField) -> Vec<Tok> {
    let mut t = Vec::new();
    match x {
        impetus::ForceField::Radial {
            center,
            strength,
            falloff,
            radius,
        } => {
            t.push(Tok::E("ForceField", "Radial".to_string()));
            v3(&mut t, *center);
            t.extend([f(*strength), f(*falloff), f(*radius)]);
        }
        impetus::ForceField::Directional { force, min, max } => {
            t.push(Tok::E("ForceField", "Directional".to_string()));
            v3(&mut t, *force);
            v3(&mut t, *min);
            v3(&mut t, *max);
        }
        _ => t.push(Tok::E("ForceField", "Unknown".to_string())),
    }
    t
}
