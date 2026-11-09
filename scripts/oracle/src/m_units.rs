//! `units.tsv` — `Quantity`'s constructors, `to_radians` and `Display` (M1).
//!
//! `Quantity::fmt` rows record Rust's `Display` text (`s:` tokens). The port has no shortest
//! round-trip f64 printer yet (port plan §8, S2), so they are pending in the harness until it does.

use impetus::{PhysicsUnit, Quantity};

use crate::flat;
use crate::grids::{Rng, specials};
use crate::tsv::{Class, Module, e, f, s};

const UNITS: [PhysicsUnit; 14] = [
    PhysicsUnit::Meters,
    PhysicsUnit::Kilograms,
    PhysicsUnit::Seconds,
    PhysicsUnit::MetersPerSecond,
    PhysicsUnit::MetersPerSecondSquared,
    PhysicsUnit::Newtons,
    PhysicsUnit::NewtonMeters,
    PhysicsUnit::Joules,
    PhysicsUnit::Radians,
    PhysicsUnit::Degrees,
    PhysicsUnit::RadiansPerSecond,
    PhysicsUnit::KgPerCubicMeter,
    PhysicsUnit::KgPerSquareMeter,
    PhysicsUnit::Pascals,
];

/// A named `Quantity` constructor.
type Ctor = (&'static str, fn(f64) -> Quantity);

pub fn emit() -> Module {
    let mut m = Module::new("units");
    let mut vals = specials();
    vals.extend([
        9.81, 5.0, 90.0, 0.1, 180.0, -45.0, 360.0, 1e-7, 1e21, 123.456,
    ]);
    let mut rng = Rng::new(0x0417);
    vals.extend((0..100).map(|_| rng.uniform(-720.0, 720.0)));

    for unit in UNITS {
        for &v in &vals[..12] {
            m.row(
                "Quantity::new",
                Class::A,
                &[f(v), e("PhysicsUnit", unit)],
                &flat::quantity(&Quantity::new(v, unit)),
            );
        }
    }
    let named: [Ctor; 9] = [
        ("Quantity::meters", Quantity::meters),
        ("Quantity::kilograms", Quantity::kilograms),
        ("Quantity::seconds", Quantity::seconds),
        ("Quantity::newtons", Quantity::newtons),
        ("Quantity::joules", Quantity::joules),
        ("Quantity::radians", Quantity::radians),
        ("Quantity::degrees", Quantity::degrees),
        ("Quantity::meters_per_second", Quantity::meters_per_second),
        ("Quantity::pascals", Quantity::pascals),
    ];
    for (name, ctor) in named {
        for &v in &vals[..12] {
            m.row(name, Class::A, &[f(v)], &flat::quantity(&ctor(v)));
        }
    }
    // to_radians: every unit (only degrees convert, as (value·π)/180), over the whole grid.
    for unit in UNITS {
        for &v in &vals {
            let q = Quantity::new(std::hint::black_box(v), unit);
            m.row(
                "Quantity::to_radians",
                Class::B,
                &flat::quantity(&q),
                &[f(q.to_radians())],
            );
        }
    }
    for unit in UNITS {
        for &v in &vals[..20] {
            let q = Quantity::new(v, unit);
            m.row(
                "Quantity::fmt",
                Class::A,
                &flat::quantity(&q),
                &[s(q.to_string())],
            );
        }
    }
    m
}
