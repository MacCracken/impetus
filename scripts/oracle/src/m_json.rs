//! `json.tsv` — what serde_json 1.0.154 writes and reads for impetus's types (ADR 0010).
//!
//! Per type, `serde_json::to_string::<T>` rows (the value's flattened fields in, the document out)
//! and `serde_json::from_str::<T>` rows (a document in, the fields or `E:Deserialize` out). The
//! documents read are every one written, plus hand-written edge documents: required and defaulted
//! fields missing, `null` for an f64, a fraction, a negative and 2^32 for a u32, wrong array lengths,
//! unknown variant names, unknown fields, duplicate keys, integers for floats, exponent forms,
//! whitespace and non-objects.

use impetus::{
    BroadphaseKind, CombineRule, Force, Impulse, PhysicsMaterial, PhysicsUnit, Quantity,
    SolverKind, Torque, WorldConfig,
};
use serde::Serialize;
use serde::de::DeserializeOwned;

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, f, s};

fn deserialize_err() -> Tok {
    Tok::Err("Deserialize".to_string())
}

fn write_rows<T: Serialize>(
    m: &mut Module,
    ty: &str,
    values: &[T],
    flat: fn(&T) -> Vec<Tok>,
) -> Vec<String> {
    let mut docs = Vec::new();
    for v in values {
        let json = serde_json::to_string(v).unwrap_or_default();
        m.row(
            &format!("serde_json::to_string::<{ty}>"),
            Class::A,
            &flat(v),
            &[s(json.clone())],
        );
        docs.push(json);
    }
    docs
}

fn read_rows<T: DeserializeOwned>(
    m: &mut Module,
    ty: &str,
    docs: &[String],
    flat: fn(&T) -> Vec<Tok>,
) {
    for d in docs {
        let out = match serde_json::from_str::<T>(d) {
            Ok(v) => flat(&v),
            Err(_) => vec![deserialize_err()],
        };
        m.row(
            &format!("serde_json::from_str::<{ty}>"),
            Class::A,
            &[s(d.clone())],
            &out,
        );
    }
}

fn torque(t: &Torque) -> Vec<Tok> {
    vec![f(t.value)]
}

fn edge(base: &str, cases: &[&str]) -> Vec<String> {
    let mut v: Vec<String> = cases.iter().map(|c| (*c).to_string()).collect();
    v.push(base.to_string());
    v
}

#[allow(clippy::too_many_lines)] // one block per type
pub fn emit() -> Module {
    let mut m = Module::new("json");
    let mut rng = Rng::new(0x15_0a);

    // ── WorldConfig ──
    let mut configs = vec![
        WorldConfig::default(),
        WorldConfig {
            timestep: 1.0 / 120.0,
            gravity: [0.0, -10.0, 0.0],
            velocity_iterations: 8,
            position_iterations: 3,
            deterministic: false,
            step: 0,
            position_slop: 0.02,
            position_correction: 0.3,
            max_velocity: 100.0,
            sub_steps: 1,
            constraint_frequency: 30.0,
            constraint_damping_ratio: 1.0,
            broadphase: BroadphaseKind::SpatialHash,
            solver: SolverKind::SequentialImpulse,
        },
        WorldConfig {
            solver: SolverKind::Xpbd,
            broadphase: BroadphaseKind::AabbTree,
            ..WorldConfig::default()
        },
        WorldConfig {
            timestep: f64::NAN,
            max_velocity: f64::INFINITY,
            step: u64::from(u32::MAX) * 7,
            sub_steps: u32::MAX,
            ..WorldConfig::default()
        },
    ];
    for _ in 0..60 {
        configs.push(WorldConfig {
            timestep: rng.log_uniform(-6.0, 0.0),
            gravity: [
                rng.signed_log_uniform(-3.0, 3.0),
                rng.signed_log_uniform(-3.0, 3.0),
                rng.signed_log_uniform(-3.0, 3.0),
            ],
            velocity_iterations: u32::try_from(rng.int(0, 100)).unwrap_or(0),
            position_slop: rng.log_uniform(-20.0, 20.0),
            constraint_frequency: rng.uniform(0.0, 240.0),
            step: u64::try_from(rng.int(0, i64::MAX / 2)).unwrap_or(0),
            ..WorldConfig::default()
        });
    }
    let mut docs = write_rows(&mut m, "WorldConfig", &configs, flat::config);
    let def = serde_json::to_string(&WorldConfig::default()).unwrap_or_default();
    docs.extend(edge(
        &def,
        &[
            r#"{"timestep":0.016,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":null,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4.0,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":-1,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4294967296,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4294967295,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0,1],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":1,"step":0}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"solver":"Xpbd","broadphase":"AabbTree"}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"solver":"xpbd"}"#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"extra":{"a":[1,2,{"b":null}]}}"#,
            r#"{"timestep":1,"timestep":2,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#,
            r#"{"timestep":2.5e-3,"gravity":[0E0,-9.81e0,1e-400],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"sub_steps":8}"#,
            r#"  { "timestep" : 0.5 , "gravity" : [ 0 , 0 , 0 ] , "velocity_iterations" : 0 , "position_iterations" : 0 , "deterministic" : false , "step" : 9007199254740993 }  "#,
            r#"{"timestep":1,"gravity":[0,-9.81,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"max_velocity":"100"}"#,
            "[]",
            "null",
            "",
            "{",
        ],
    ));
    read_rows::<WorldConfig>(&mut m, "WorldConfig", &docs, flat::config);

    // ── PhysicsMaterial ──
    let mut mats = vec![
        PhysicsMaterial::default(),
        PhysicsMaterial::ice(),
        PhysicsMaterial::rubber(),
        PhysicsMaterial::wood(),
        PhysicsMaterial::steel(),
        PhysicsMaterial::bouncy(),
        PhysicsMaterial {
            static_friction: Some(0.75),
            rolling_friction: 0.01,
            friction_combine: CombineRule::Max,
            restitution_combine: CombineRule::Multiply,
            ..PhysicsMaterial::default()
        },
        PhysicsMaterial {
            friction: f64::NAN,
            ..PhysicsMaterial::default()
        },
    ];
    for _ in 0..40 {
        mats.push(PhysicsMaterial {
            friction: rng.uniform(0.0, 2.0),
            restitution: rng.uniform(0.0, 1.0),
            density: rng.log_uniform(-3.0, 4.0),
            static_friction: if rng.unit() < 0.5 {
                Some(rng.uniform(0.0, 2.0))
            } else {
                None
            },
            ..PhysicsMaterial::default()
        });
    }
    let mut docs = write_rows(&mut m, "PhysicsMaterial", &mats, flat::material);
    docs.extend(edge(
        r#"{"friction":0.5,"restitution":0.0,"density":1.0}"#,
        &[
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"static_friction":null}"#,
            r#"{"friction":0.5,"restitution":0,"density":1,"static_friction":2,"rolling_friction":0}"#,
            r#"{"friction":0.5,"restitution":0.0}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"friction_combine":"Median"}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"friction_combine":"Max","restitution_combine":"Average"}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"rolling_friction":null}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"static_friction":"0.3"}"#,
        ],
    ));
    read_rows::<PhysicsMaterial>(&mut m, "PhysicsMaterial", &docs, flat::material);

    // ── Quantity ──
    let units = [
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
    let mut qs = Vec::new();
    for u in units {
        for v in [42.0, 9.81, -0.0, 1e21, 1.5e-7, f64::NAN] {
            qs.push(Quantity::new(v, u));
        }
    }
    let docs_q = write_rows(&mut m, "Quantity", &qs, flat::quantity);
    let mut docs = docs_q;
    docs.extend(edge(
        r#"{"value":42,"unit":"Newtons"}"#,
        &[
            r#"{"value":42.0}"#,
            r#"{"unit":"Newtons"}"#,
            r#"{"value":42.0,"unit":"newtons"}"#,
            r#"{"value":"42","unit":"Newtons"}"#,
            r#"{"value":null,"unit":"Newtons"}"#,
        ],
    ));
    read_rows::<Quantity>(&mut m, "Quantity", &docs, flat::quantity);

    // ── Force, Impulse, Torque ──
    let mut forces = vec![
        Force::new(3.0, 4.0, 0.0),
        Force::at_point(3.0, 4.0, 0.0, 1.0, 2.0, 0.0),
        Force::gravity(10.0, 9.81),
        Force::new(f64::NAN, 0.0, f64::NEG_INFINITY),
    ];
    let mut impulses = vec![
        Impulse::new(5.0, 10.0, 0.0),
        Impulse::at_point(1.0, 2.0, 0.0, 3.0, 4.0, 0.0),
    ];
    for _ in 0..20 {
        let v = [
            rng.signed_log_uniform(-10.0, 10.0),
            rng.signed_log_uniform(-10.0, 10.0),
            rng.signed_log_uniform(-10.0, 10.0),
        ];
        forces.push(Force::at_point(v[0], v[1], v[2], v[1], v[2], v[0]));
        impulses.push(Impulse::new(v[2], v[0], v[1]));
    }
    let vp_edges = [
        r#"{"vector":[1,2,3]}"#,
        r#"{"vector":[1,2,3],"point":null}"#,
        r#"{"vector":[1,2,3],"point":[1,2]}"#,
        r#"{"point":[1,2,3]}"#,
        r#"{"vector":[1,2,3],"point":[0.5,0.5,0]}"#,
    ];
    let mut docs = write_rows(&mut m, "Force", &forces, flat::force);
    docs.extend(vp_edges.iter().map(|e| (*e).to_string()));
    read_rows::<Force>(&mut m, "Force", &docs, flat::force);
    let mut docs = write_rows(&mut m, "Impulse", &impulses, flat::impulse);
    docs.extend(vp_edges.iter().map(|e| (*e).to_string()));
    read_rows::<Impulse>(&mut m, "Impulse", &docs, flat::impulse);
    let torques: Vec<Torque> = [5.0, -3.5, 0.0, -0.0, 1e300, f64::NAN]
        .iter()
        .map(|&v| Torque::new(v))
        .collect();
    let mut docs = write_rows(&mut m, "Torque", &torques, torque);
    docs.extend([
        "{\"value\":1}".to_string(),
        "{}".to_string(),
        "{\"value\":true}".to_string(),
    ]);
    read_rows::<Torque>(&mut m, "Torque", &docs, torque);
    m
}
