//! `force.tsv` — `Force`, `Impulse` and `Torque` (M1).

use impetus::{Force, Impulse, Torque};

use crate::flat;
use crate::grids::{Rng, specials};
use crate::tsv::{Class, Module, f};

pub fn emit() -> Module {
    let mut m = Module::new("force");
    let mut rng = Rng::new(0xf04c);
    let sp = specials();
    let mut vecs: Vec<[f64; 3]> = Vec::new();
    for &x in &sp {
        vecs.push([x, 0.0, 0.0]);
        vecs.push([0.0, x, 0.0]);
        vecs.push([1.0, 2.0, x]);
    }
    vecs.extend([
        [3.0, 4.0, 0.0],
        [1.0, 2.0, 2.0],
        [2.0, 3.0, 6.0],
        [1e200, 1e200, 0.0],
    ]);
    for _ in 0..200 {
        vecs.push([
            rng.uniform(-1e3, 1e3),
            rng.uniform(-1e3, 1e3),
            rng.uniform(-1e3, 1e3),
        ]);
    }
    for _ in 0..100 {
        vecs.push([
            rng.signed_log_uniform(-150.0, 150.0),
            rng.signed_log_uniform(-150.0, 150.0),
            rng.signed_log_uniform(-150.0, 150.0),
        ]);
    }

    for v in &vecs[..20] {
        let x = Force::new(v[0], v[1], v[2]);
        m.row(
            "Force::new",
            Class::A,
            &[f(v[0]), f(v[1]), f(v[2])],
            &flat::force(&x),
        );
        let p = Force::at_point(v[0], v[1], v[2], v[2], v[0], v[1]);
        m.row(
            "Force::at_point",
            Class::A,
            &[f(v[0]), f(v[1]), f(v[2]), f(v[2]), f(v[0]), f(v[1])],
            &flat::force(&p),
        );
        let i = Impulse::new(v[0], v[1], v[2]);
        m.row(
            "Impulse::new",
            Class::A,
            &[f(v[0]), f(v[1]), f(v[2])],
            &flat::impulse(&i),
        );
        let ip = Impulse::at_point(v[0], v[1], v[2], v[1], v[2], v[0]);
        m.row(
            "Impulse::at_point",
            Class::A,
            &[f(v[0]), f(v[1]), f(v[2]), f(v[1]), f(v[2]), f(v[0])],
            &flat::impulse(&ip),
        );
    }
    for v in &vecs {
        let x = Force::new(v[0], v[1], v[2]);
        m.row(
            "Force::magnitude",
            Class::B,
            &flat::force(&x),
            &[f(x.magnitude())],
        );
        let i = Impulse::at_point(v[0], v[1], v[2], 1.0, 0.0, 0.0);
        m.row(
            "Impulse::magnitude",
            Class::B,
            &flat::impulse(&i),
            &[f(i.magnitude())],
        );
    }
    let mut masses = sp.clone();
    masses.extend((0..100).map(|_| rng.log_uniform(-3.0, 4.0)));
    for &mass in &masses {
        for g in [9.81, 1.62, 0.0, -9.81, 24.79] {
            let x = Force::gravity(std::hint::black_box(mass), std::hint::black_box(g));
            m.row(
                "Force::gravity",
                Class::B,
                &[f(mass), f(g)],
                &flat::force(&x),
            );
        }
    }
    for &v in &sp {
        m.row("Torque::new", Class::A, &[f(v)], &[f(Torque::new(v).value)]);
    }
    m
}
