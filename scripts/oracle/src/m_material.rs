//! `material.tsv` — combine rules, the presets and `effective_static_friction` (M1).

use impetus::{CombineRule, PhysicsMaterial};

use crate::flat;
use crate::grids::{Rng, specials};
use crate::tsv::{Class, Module, e, f};

const RULES: [CombineRule; 4] = [
    CombineRule::Min,
    CombineRule::Average,
    CombineRule::Multiply,
    CombineRule::Max,
];

pub fn emit() -> Module {
    let mut m = Module::new("material");

    // combine: every rule over the special grid, friction/restitution-range pairs and wide pairs.
    let mut pairs: Vec<(f64, f64)> = Vec::new();
    let sp = specials();
    for &a in &sp {
        for &b in &sp {
            pairs.push((a, b));
        }
    }
    let mut rng = Rng::new(0xc0b1);
    for _ in 0..200 {
        pairs.push((rng.uniform(0.0, 1.5), rng.uniform(0.0, 1.5)));
    }
    for _ in 0..100 {
        pairs.push((
            rng.signed_log_uniform(-300.0, 300.0),
            rng.signed_log_uniform(-300.0, 300.0),
        ));
    }
    for rule in RULES {
        // min and max are exact; the mean and the product are one or two roundings.
        let class = match rule {
            CombineRule::Min | CombineRule::Max => Class::A,
            _ => Class::B,
        };
        for &(a, b) in &pairs {
            let r = rule.combine(std::hint::black_box(a), std::hint::black_box(b));
            m.row(
                "CombineRule::combine",
                class,
                &[e("CombineRule", rule), f(a), f(b)],
                &[f(r)],
            );
        }
    }

    let presets: [(&str, PhysicsMaterial); 6] = [
        ("PhysicsMaterial::default", PhysicsMaterial::default()),
        ("PhysicsMaterial::ice", PhysicsMaterial::ice()),
        ("PhysicsMaterial::rubber", PhysicsMaterial::rubber()),
        ("PhysicsMaterial::wood", PhysicsMaterial::wood()),
        ("PhysicsMaterial::steel", PhysicsMaterial::steel()),
        ("PhysicsMaterial::bouncy", PhysicsMaterial::bouncy()),
    ];
    for (name, mat) in &presets {
        m.row(name, Class::A, &[], &flat::material(mat));
    }

    // effective_static_friction: with and without the override, friction over specials and a sweep.
    let mut fr = specials();
    fr.extend((0..100).map(|_| rng.uniform(0.0, 2.0)));
    for &x in &fr {
        for over in [None, Some(0.0), Some(0.75), Some(x)] {
            let mat = PhysicsMaterial {
                friction: x,
                static_friction: over,
                ..PhysicsMaterial::default()
            };
            m.row(
                "PhysicsMaterial::effective_static_friction",
                Class::B,
                &flat::material(&mat),
                &[f(mat.effective_static_friction())],
            );
        }
    }
    m
}
