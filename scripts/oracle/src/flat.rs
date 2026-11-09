//! Flattening of the oracle's public structs into golden tokens: fields in declaration order, an
//! `Option` as `n:` or its value's tokens, an enum as `e:<Enum>.<Variant>`. The Cyrius dispatchers
//! read and write the same order.

use impetus::WorldConfig;
use impetus::force::{Force, Impulse};
use impetus::material::PhysicsMaterial;
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
