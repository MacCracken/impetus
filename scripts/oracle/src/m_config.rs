//! `config.tsv` — `WorldConfig::default` and `WorldConfig::validate` (M1).
//!
//! `validate` rows take the 16 config tokens (`flat::config`) and give `-` for `Ok(())` or
//! `E:InvalidConfig`. The grid is the default config with one field moved at a time to each
//! boundary and special value (port plan §6, M1), then pairs of bad fields (the first one in Rust's
//! order decides), then every row again with the XPBD solver: Rust accepts it, the port refuses it
//! (ADR 0008 #6, a mapped row).

use impetus::{BroadphaseKind, SolverKind, WorldConfig};

use crate::flat;
use crate::grids::specials;
use crate::tsv::{Class, Module, Tok, err};

fn validate_row(m: &mut Module, c: &WorldConfig) {
    let out: Vec<Tok> = match c.validate() {
        Ok(()) => vec![],
        Err(x) => vec![err(&x)],
    };
    m.row("WorldConfig::validate", Class::A, &flat::config(c), &out);
}

#[allow(clippy::too_many_lines)] // one block per field, in Rust's validation order
pub fn emit() -> Module {
    let mut m = Module::new("config");
    m.row(
        "WorldConfig::default",
        Class::A,
        &[],
        &flat::config(&WorldConfig::default()),
    );

    let tiny = f64::from_bits(1);
    let reals = specials();
    let mut cases: Vec<WorldConfig> = vec![WorldConfig::default()];
    let base = WorldConfig::default;

    for &x in &reals {
        cases.push(WorldConfig {
            timestep: x,
            ..base()
        });
    }
    for x in [1e-300, 1.0 / 60.0, 1.0 / 240.0, 1.0, 1e9] {
        cases.push(WorldConfig {
            timestep: x,
            ..base()
        });
    }
    for k in 0..3 {
        for &x in &reals {
            let mut g = [0.0, -9.81, 0.0];
            g[k] = x;
            cases.push(WorldConfig {
                gravity: g,
                ..base()
            });
        }
    }
    let counts = [
        0,
        1,
        2,
        4,
        4095,
        4096,
        4097,
        9_999,
        10_000,
        10_001,
        u32::MAX,
    ];
    for &n in &counts {
        cases.push(WorldConfig {
            sub_steps: n,
            ..base()
        });
        cases.push(WorldConfig {
            velocity_iterations: n,
            ..base()
        });
        cases.push(WorldConfig {
            position_iterations: n,
            ..base()
        });
    }
    for &x in reals.iter().chain(&[-tiny, 0.5]) {
        cases.push(WorldConfig {
            position_slop: x,
            ..base()
        });
    }
    for &x in reals
        .iter()
        .chain(&[-tiny, 0.5, 1.0 + f64::EPSILON, 1.0 - f64::EPSILON / 2.0])
    {
        cases.push(WorldConfig {
            position_correction: x,
            ..base()
        });
    }
    for &x in reals.iter().chain(&[-tiny, 1e-300, 1e300]) {
        cases.push(WorldConfig {
            max_velocity: x,
            ..base()
        });
    }
    for &x in reals.iter().chain(&[-tiny, 120.0]) {
        cases.push(WorldConfig {
            constraint_frequency: x,
            ..base()
        });
        cases.push(WorldConfig {
            constraint_damping_ratio: x,
            ..base()
        });
    }
    cases.push(WorldConfig {
        broadphase: BroadphaseKind::AabbTree,
        ..base()
    });
    cases.push(WorldConfig {
        deterministic: false,
        step: u64::from(u32::MAX),
        ..base()
    });
    // Two bad fields: the first in Rust's order decides the verdict (the code is the same).
    cases.push(WorldConfig {
        timestep: -1.0,
        max_velocity: 0.0,
        ..base()
    });
    cases.push(WorldConfig {
        position_correction: 2.0,
        sub_steps: 5000,
        ..base()
    });

    // Every case again with the XPBD solver (ADR 0008 #6).
    let xpbd: Vec<WorldConfig> = cases
        .iter()
        .map(|c| WorldConfig {
            solver: SolverKind::Xpbd,
            ..c.clone()
        })
        .collect();
    for c in cases.iter().chain(&xpbd) {
        validate_row(&mut m, c);
    }
    m
}
