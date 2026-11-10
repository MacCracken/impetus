//! `json.tsv` — what serde_json 1.0.154 writes and reads for impetus's types (ADR 0010).
//!
//! Per type, `serde_json::to_string::<T>` rows (the value's flattened fields in, the document out)
//! and `serde_json::from_str::<T>` rows (a document in, the fields or `E:Deserialize` out). The
//! documents read are every one written, plus hand-written edge documents: required and defaulted
//! fields missing, `null` for an f64, a fraction, a negative and 2^32 for a u32, wrong array lengths,
//! unknown variant names, unknown fields, duplicate keys, integers for floats, exponent forms,
//! whitespace and non-objects.

use impetus::{
    BodyDesc, BodyHandle, BodyState, BodyType, BroadphaseKind, ColliderDesc, ColliderHandle,
    ColliderShape, CollisionEvent, CombineRule, ContactData, ContactPoint, Force, ForceField,
    Impulse, JointDesc, JointMotor, JointType, Particle, ParticleEmitter, PhysicsMaterial,
    PhysicsUnit, PointQuery, Quantity, RayHit, SolverKind, Spring, Spring2d, Spring3d, SubEmitter,
    Torque, WorldConfig,
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
            // serde's sequence form: by position, trailing defaulted fields may be left off.
            "[0.016,[0,-9.81,0],4,1,true,0]",
            r#"[0.016,[0,-9.81,0],4,1,true,0,0.01,0.2,500,4,60,0.7,"AabbTree","SequentialImpulse"]"#,
            r#"[0.016,[0,-9.81,0],4,1,true,0,0.01,0.2,500,4,60,0.7,"AabbTree","SequentialImpulse",1]"#,
            "[0.016,[0,-9.81,0],4,1,true]",
            // unit variants as one-key objects.
            r#"{"timestep":1,"gravity":[0,0,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"solver":{"Xpbd":null}}"#,
            r#"{"timestep":1,"gravity":[0,0,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"solver":{"Xpbd":[]}}"#,
            // S3: a u64 above i64::MAX (serde_json reads it; bayan refuses the document).
            r#"{"timestep":1,"gravity":[0,0,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":9223372036854775808}"#,
            // S4: a duplicate key serde ignores.
            r#"{"timestep":1,"gravity":[0,0,0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0,"x":1,"x":2}"#,
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
            "[0.5,0,1]",
            "[0.5,0,1,null]",
            r#"[0.5,0,1,0.3,0,"Max","Min"]"#,
            r#"[0.5,0,1,0.3,0,"Max","Min",0]"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"friction_combine":{"Max":null}}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"friction_combine":{"Max":null,"Min":null}}"#,
            r#"{"friction":0.5,"restitution":0.0,"density":1.0,"friction_combine":{}}"#,
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
            r#"[42,"Newtons"]"#,
            "[42]",
            r#"{"value":42,"unit":{"Newtons":null}}"#,
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
        "[[1,2,3]]",
        "[[1,2,3],null]",
        "[[1,2,3],[4,5,6]]",
        "[[1,2,3],null,null]",
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
        "[5]".to_string(),
        "[5,6]".to_string(),
        "[]".to_string(),
        // S3: an integer beyond u64 reads as an f64 in serde_json; bayan refuses it.
        "{\"value\":100000000000000000000}".to_string(),
        // S4: duplicate keys serde ignores (an unknown field; inside an ignored value).
        "{\"value\":1,\"x\":1,\"x\":2}".to_string(),
        "{\"value\":1,\"x\":{\"a\":1,\"a\":2}}".to_string(),
    ]);
    read_rows::<Torque>(&mut m, "Torque", &docs, torque);
    emit_m1b(&mut m, &mut rng);
    emit_particles(&mut m, &mut rng);
    m
}

fn rand3(rng: &mut Rng, lo: f64, hi: f64) -> [f64; 3] {
    [
        rng.signed_log_uniform(lo, hi),
        rng.signed_log_uniform(lo, hi),
        rng.signed_log_uniform(lo, hi),
    ]
}

fn rand2(rng: &mut Rng, lo: f64, hi: f64) -> [f64; 2] {
    [
        rng.signed_log_uniform(lo, hi),
        rng.signed_log_uniform(lo, hi),
    ]
}

/// A handle below 2^63 (above it, bayan can't read the document back: S3).
fn rand_handle(rng: &mut Rng) -> u64 {
    rng.bits() >> 1
}

#[allow(clippy::too_many_lines)] // one block per type
fn emit_m1b(m: &mut Module, rng: &mut Rng) {
    // ── BodyDesc ──
    let mut bodies = vec![
        BodyDesc::default(),
        BodyDesc {
            body_type: BodyType::Static,
            position: [5.0, 3.0, 0.0],
            rotation: 1.57,
            ..BodyDesc::default()
        },
        BodyDesc {
            body_type: BodyType::Kinematic,
            position: [1.0, 2.0, 0.0],
            linear_velocity: [3.0, 4.0, 0.0],
            angular_velocity: 0.5,
            fixed_rotation: true,
            gravity_scale: Some(0.5),
            ..BodyDesc::default()
        },
        BodyDesc {
            gravity_scale: Some(f64::NAN),
            rotation: f64::INFINITY,
            ..BodyDesc::default()
        },
    ];
    for _ in 0..20 {
        bodies.push(BodyDesc {
            body_type: [BodyType::Static, BodyType::Dynamic, BodyType::Kinematic]
                [usize::try_from(rng.int(0, 2)).unwrap_or(0)],
            position: rand3(rng, -3.0, 3.0),
            rotation: rng.uniform(-7.0, 7.0),
            linear_velocity: rand3(rng, -3.0, 3.0),
            angular_velocity: rng.signed_log_uniform(-3.0, 3.0),
            linear_damping: rng.uniform(0.0, 1.0),
            angular_damping: rng.uniform(0.0, 1.0),
            fixed_rotation: rng.unit() < 0.5,
            gravity_scale: if rng.unit() < 0.5 {
                Some(rng.uniform(-2.0, 2.0))
            } else {
                None
            },
        });
    }
    let mut docs = write_rows(m, "BodyDesc", &bodies, flat::body_desc);
    docs.extend(
        [
            r#"{"body_type":"Static","position":[0,0,0],"rotation":0}"#,
            r#"{"body_type":"Static","position":[0,0,0]}"#,
            r#"{"body_type":{"Kinematic":null},"position":[0,0,0],"rotation":0,"gravity_scale":null}"#,
            r#"{"body_type":"static","position":[0,0,0],"rotation":0}"#,
            r#"{"body_type":"Static","position":[0,0,0],"rotation":0,"fixed_rotation":1}"#,
            r#"{"body_type":"Static","position":[0,0,0],"rotation":0,"linear_velocity":[1,2]}"#,
            r#"["Dynamic",[1,2,3],0.5]"#,
            r#"["Dynamic",[1,2,3],0.5,[0,0,0],0,0,0,false,2]"#,
            r#"["Dynamic",[1,2,3],0.5,[0,0,0],0,0,0,false,2,0]"#,
            r#"["Dynamic",[1,2,3]]"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<BodyDesc>(m, "BodyDesc", &docs, flat::body_desc);

    // ── BodyState ──
    let mut states = vec![
        BodyState {
            handle: BodyHandle(42),
            body_type: BodyType::Dynamic,
            position: [1.0, 2.0, 0.0],
            rotation: 0.5,
            linear_velocity: [3.0, 4.0, 0.0],
            angular_velocity: 1.0,
            is_sleeping: false,
        },
        BodyState {
            handle: BodyHandle(0),
            body_type: BodyType::Static,
            position: [0.0; 3],
            rotation: 0.0,
            linear_velocity: [0.0; 3],
            angular_velocity: 0.0,
            is_sleeping: true,
        },
    ];
    for _ in 0..10 {
        states.push(BodyState {
            handle: BodyHandle(rand_handle(rng)),
            body_type: BodyType::Dynamic,
            position: rand3(rng, -3.0, 3.0),
            rotation: rng.uniform(-3.2, 3.2),
            linear_velocity: rand3(rng, -3.0, 3.0),
            angular_velocity: rng.signed_log_uniform(-3.0, 3.0),
            is_sleeping: rng.unit() < 0.5,
        });
    }
    // Written as serde_json writes it; the read back is S3's (pending).
    states.push(BodyState {
        handle: BodyHandle(u64::MAX),
        ..states[1].clone()
    });
    let mut docs = write_rows(m, "BodyState", &states, flat::body_state);
    docs.extend(
        [
            r#"{"handle":1.0,"body_type":"Static","position":[0,0,0],"rotation":0,"linear_velocity":[0,0,0],"angular_velocity":0,"is_sleeping":true}"#,
            r#"{"handle":-1,"body_type":"Static","position":[0,0,0],"rotation":0,"linear_velocity":[0,0,0],"angular_velocity":0,"is_sleeping":true}"#,
            r#"{"handle":7,"body_type":"Static","position":[0,0,0],"rotation":0,"linear_velocity":[0,0,0],"angular_velocity":0}"#,
            r#"[7,"Static",[0,0,0],0,[0,0,0],0,true]"#,
            r#"[7,"Static",[0,0,0],0,[0,0,0],0]"#,
            r#"{"handle":9223372036854775807,"body_type":"Static","position":[0,0,0],"rotation":0,"linear_velocity":[0,0,0],"angular_velocity":0,"is_sleeping":true}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<BodyState>(m, "BodyState", &docs, flat::body_state);

    // ── ColliderShape ──
    let mut shapes = vec![
        ColliderShape::Box {
            half_extents: [1.0, 2.0, 3.0],
        },
        ColliderShape::Ball { radius: 0.5 },
        ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.25,
        },
        ColliderShape::ConvexHull {
            points: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        },
        ColliderShape::ConvexHull { points: vec![] },
        ColliderShape::TriMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![[0, 1, 2], [2, 1, u32::MAX]],
        },
        ColliderShape::Heightfield {
            heights: vec![0.0, 1.5, -2.25, 1e-7],
            scale: [1.0, 2.0, 1.0],
        },
        ColliderShape::Segment {
            a: [0.0, 0.0, 0.0],
            b: [1.0, -1.0, 0.5],
        },
        ColliderShape::Ball { radius: f64::NAN },
    ];
    for _ in 0..10 {
        let n = usize::try_from(rng.int(1, 8)).unwrap_or(1);
        shapes.push(ColliderShape::ConvexHull {
            points: (0..n).map(|_| rand3(rng, -3.0, 3.0)).collect(),
        });
        shapes.push(ColliderShape::Box {
            half_extents: rand3(rng, -2.0, 2.0),
        });
    }
    let mut docs = write_rows(m, "ColliderShape", &shapes, flat::shape);
    docs.extend(
        [
            r#"{"Ball":[0.5]}"#,
            r#"{"Ball":{"radius":0.5},"Box":{"half_extents":[1,1,1]}}"#,
            r#""Ball""#,
            r#"{"Sphere":{"radius":1}}"#,
            r#"{"TriMesh":{"vertices":[],"indices":[[0,1,4294967296]]}}"#,
            r#"{"TriMesh":{"vertices":[[0,0]],"indices":[]}}"#,
            r#"{"TriMesh":{"vertices":[[0,0,0]],"indices":[[0,1,2.0]]}}"#,
            r#"{"Heightfield":{"heights":[1,2,null],"scale":[1,1,1]}}"#,
            r#"{"Heightfield":[[1,2],[1,1,1]]}"#,
            r#"{"Ball":{}}"#,
            r#"{"Ball":null}"#,
            "{}",
            "[]",
            r#"{"Capsule":[1]}"#,
            r#"{"Capsule":[1,2,3]}"#,
            r#"{"Capsule":{"radius":2,"half_height":1}}"#,
            r#"{"Segment":{"a":[0,0,0],"b":[1,1,1],"c":3}}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ColliderShape>(m, "ColliderShape", &docs, flat::shape);

    // ── ColliderDesc ──
    let mut cdescs = vec![
        ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0; 3],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: u32::MAX,
            collision_mask: u32::MAX,
        },
        ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [1.0, 0.5, 0.0],
            },
            offset: [0.1, 0.2, 0.0],
            material: PhysicsMaterial::rubber(),
            is_sensor: true,
            mass: Some(2.5),
            collision_layer: 0b0101,
            collision_mask: 0,
        },
    ];
    for _ in 0..8 {
        cdescs.push(ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: rng.uniform(0.1, 3.0),
                radius: rng.uniform(0.1, 1.0),
            },
            offset: rand3(rng, -2.0, 2.0),
            material: PhysicsMaterial::ice(),
            is_sensor: rng.unit() < 0.5,
            mass: if rng.unit() < 0.5 {
                Some(rng.log_uniform(-2.0, 3.0))
            } else {
                None
            },
            collision_layer: u32::try_from(rng.int(0, i64::from(u32::MAX))).unwrap_or(0),
            collision_mask: u32::try_from(rng.int(0, i64::from(u32::MAX))).unwrap_or(0),
        });
    }
    let mut docs = write_rows(m, "ColliderDesc", &cdescs, flat::collider_desc);
    docs.extend(
        [
            r#"{"shape":{"Ball":{"radius":0.5}}}"#,
            r#"[{"Ball":{"radius":0.5}}]"#,
            r#"[{"Ball":{"radius":0.5}},[0,0,0],{"friction":1,"restitution":0,"density":1},false,null,1,2]"#,
            r#"{"shape":{"Ball":{"radius":0.5}},"collision_layer":4294967296}"#,
            r#"{"shape":{"Ball":{"radius":0.5}},"mass":null,"material":{"friction":1,"restitution":0,"density":1}}"#,
            r#"{"shape":{"Ball":{"radius":0.5}},"material":{"friction":1,"restitution":0}}"#,
            r#"{"offset":[0,0,0]}"#,
            r#"{"shape":{"ConvexHull":{"points":[[1,2,3]]}},"is_sensor":"yes"}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ColliderDesc>(m, "ColliderDesc", &docs, flat::collider_desc);

    // ── JointType ──
    let mut jtypes = vec![
        JointType::Fixed,
        JointType::Revolute {
            anchor: [0.5, 0.0],
            limits: None,
        },
        JointType::Revolute {
            anchor: [0.0, 1.0],
            limits: Some([-1.57, 1.57]),
        },
        JointType::Prismatic {
            axis: [1.0, 0.0],
            limits: Some([-2.0, 2.0]),
        },
        JointType::Prismatic {
            axis: [0.0, 1.0],
            limits: None,
        },
        JointType::Spring {
            rest_length: 1.0,
            stiffness: 100.0,
            damping: 5.0,
        },
        JointType::Distance { length: 2.5 },
        JointType::Wheel {
            axis: [0.0, 1.0],
            stiffness: 50.0,
            damping: 2.0,
        },
        JointType::Rope { max_length: 3.0 },
        JointType::Mouse {
            target: [1.0, 2.0, 0.0],
            stiffness: 1000.0,
            damping: 10.0,
            max_force: 1e4,
        },
    ];
    for _ in 0..6 {
        jtypes.push(JointType::Revolute {
            anchor: rand2(rng, -2.0, 2.0),
            limits: Some(rand2(rng, -1.0, 1.0)),
        });
        jtypes.push(JointType::Spring {
            rest_length: rng.log_uniform(-2.0, 2.0),
            stiffness: rng.log_uniform(0.0, 4.0),
            damping: rng.log_uniform(-1.0, 2.0),
        });
    }
    let mut docs = write_rows(m, "JointType", &jtypes, flat::joint_type);
    docs.extend(
        [
            r#"{"Fixed":null}"#,
            r#"{"Fixed":[]}"#,
            r#""Revolute""#,
            r#"{"Revolute":{"anchor":[0,0]}}"#,
            r#"{"Revolute":[[0,0]]}"#,
            r#"{"Revolute":[[0,0],null]}"#,
            r#"{"Revolute":{"anchor":[0,0],"limits":[1]}}"#,
            r#"{"Distance":{"length":1},"Rope":{"max_length":1}}"#,
            "{}",
            r#"{"Distance":{"length":1,"length":2}}"#,
            r#"{"Mouse":{"target":[1,2],"stiffness":1,"damping":1,"max_force":1}}"#,
            r#"{"Mouse":[[1,2,3],1,1,1]}"#,
            r#"{"Spring":{"rest_length":1,"stiffness":2}}"#,
            r#"{"Wheel":{"axis":[0,1],"stiffness":1,"damping":1,"extra":[]}}"#,
            r#""fixed""#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<JointType>(m, "JointType", &docs, flat::joint_type);

    // ── JointDesc ──
    let jdescs = vec![
        JointDesc {
            body_a: BodyHandle(0),
            body_b: BodyHandle(1),
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        },
        JointDesc {
            body_a: BodyHandle(0),
            body_b: BodyHandle(1),
            joint_type: JointType::Spring {
                rest_length: 1.0,
                stiffness: 100.0,
                damping: 5.0,
            },
            local_anchor_a: [0.5, 0.0],
            local_anchor_b: [-0.5, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        },
        JointDesc {
            body_a: BodyHandle(3 << 32 | 7),
            body_b: BodyHandle(9),
            joint_type: JointType::Revolute {
                anchor: [0.0, 0.5],
                limits: Some([-0.5, 0.5]),
            },
            local_anchor_a: [0.0, 0.5],
            local_anchor_b: [0.0, -0.5],
            motor: Some(JointMotor {
                target_velocity: 2.0,
                max_force: 50.0,
            }),
            damping: 0.1,
            break_force: Some(1000.0),
        },
    ];
    let mut docs = write_rows(m, "JointDesc", &jdescs, flat::joint_desc);
    docs.extend(
        [
            r#"{"body_a":0,"body_b":1,"joint_type":"Fixed","local_anchor_a":[0,0],"local_anchor_b":[0,0]}"#,
            r#"{"body_a":0,"body_b":1,"joint_type":"Fixed","local_anchor_a":[0,0],"local_anchor_b":[0,0],"motor":null}"#,
            r#"[0,1,"Fixed",[0,0],[0,0]]"#,
            r#"[0,1,"Fixed",[0,0],[0,0],[1,2],0.5,null]"#,
            r#"{"body_a":0,"body_b":1,"joint_type":"Fixed","local_anchor_a":[0,0],"local_anchor_b":[0,0],"motor":{"target_velocity":1}}"#,
            r#"{"body_a":-1,"body_b":1,"joint_type":"Fixed","local_anchor_a":[0,0],"local_anchor_b":[0,0]}"#,
            r#"{"body_a":0,"body_b":1,"joint_type":"Fixed","local_anchor_a":[0,0,0],"local_anchor_b":[0,0]}"#,
            r#"{"body_a":0,"body_b":1,"local_anchor_a":[0,0],"local_anchor_b":[0,0]}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<JointDesc>(m, "JointDesc", &docs, flat::joint_desc);

    // ── CollisionEvent ──
    let mut events = vec![
        CollisionEvent::Started {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
        },
        CollisionEvent::Stopped {
            collider_a: ColliderHandle(3),
            collider_b: ColliderHandle(7),
        },
        CollisionEvent::Ongoing {
            collider_a: ColliderHandle(2),
            collider_b: ColliderHandle(5),
        },
    ];
    for _ in 0..6 {
        events.push(CollisionEvent::Ongoing {
            collider_a: ColliderHandle(rand_handle(rng)),
            collider_b: ColliderHandle(rand_handle(rng)),
        });
    }
    let mut docs = write_rows(m, "CollisionEvent", &events, flat::event);
    docs.extend(
        [
            r#"{"Started":[0,1]}"#,
            r#"{"Started":{"collider_a":0}}"#,
            r#""Started""#,
            r#"{"Begun":{"collider_a":0,"collider_b":1}}"#,
            r#"{"Stopped":{"collider_a":0,"collider_b":1.5}}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<CollisionEvent>(m, "CollisionEvent", &docs, flat::event);

    // ── ContactPoint, ContactData ──
    let pts: Vec<ContactPoint> = (0..6)
        .map(|_| ContactPoint {
            local_a: rand3(rng, -2.0, 1.0),
            local_b: rand3(rng, -2.0, 1.0),
            impulse: rng.log_uniform(-3.0, 3.0),
        })
        .collect();
    let mut docs = write_rows(m, "ContactPoint", &pts, flat::contact_point);
    docs.extend(
        [
            r#"{"local_a":[0,0,0],"local_b":[1,1,1]}"#,
            "[[0,0,0],[1,1,1],2]",
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ContactPoint>(m, "ContactPoint", &docs, flat::contact_point);
    let datas = vec![
        ContactData {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
            normal: [0.0, 1.0, 0.0],
            depth: 0.05,
            points: vec![ContactPoint {
                local_a: [0.0, -0.5, 0.0],
                local_b: [0.0, 0.5, 0.0],
                impulse: 9.81,
            }],
        },
        ContactData {
            collider_a: ColliderHandle(4),
            collider_b: ColliderHandle(1 << 32 | 2),
            normal: [0.6, 0.8, 0.0],
            depth: 0.0,
            points: vec![],
        },
        ContactData {
            collider_a: ColliderHandle(1),
            collider_b: ColliderHandle(2),
            normal: [0.0, 0.0, 1.0],
            depth: 1e-9,
            points: pts.clone(),
        },
    ];
    let mut docs = write_rows(m, "ContactData", &datas, flat::contact_data);
    docs.extend(
        [
            r#"{"collider_a":0,"collider_b":1,"normal":[0,1,0],"depth":0}"#,
            r#"{"collider_a":0,"collider_b":1,"normal":[0,1,0],"depth":0,"points":[[[0,0,0],[1,1,1],2]]}"#,
            r#"{"collider_a":0,"collider_b":1,"normal":[0,1,0],"depth":0,"points":[{"local_a":[0,0,0]}]}"#,
            r#"{"collider_a":0,"collider_b":1,"normal":[0,1,0],"depth":0,"points":{}}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ContactData>(m, "ContactData", &docs, flat::contact_data);

    // ── RayHit, PointQuery ──
    let hits = vec![
        RayHit {
            collider: ColliderHandle(5),
            point: [1.0, 2.0, 0.0],
            normal: [0.0, 1.0, 0.0],
            distance: 3.5,
        },
        RayHit {
            collider: ColliderHandle(rand_handle(rng)),
            point: rand3(rng, -3.0, 3.0),
            normal: [0.6, -0.8, 0.0],
            distance: rng.log_uniform(-3.0, 3.0),
        },
    ];
    let mut docs = write_rows(m, "RayHit", &hits, flat::ray_hit);
    docs.push(r#"{"collider":5,"point":[1,2,0],"normal":[0,1,0]}"#.to_string());
    docs.push("[5,[1,2,0],[0,1,0],3.5]".to_string());
    read_rows::<RayHit>(m, "RayHit", &docs, flat::ray_hit);
    let queries = vec![
        PointQuery {
            collider: ColliderHandle(2),
            distance: 0.0,
            is_inside: true,
        },
        PointQuery {
            collider: ColliderHandle(3),
            distance: 1.5,
            is_inside: false,
        },
        PointQuery {
            collider: ColliderHandle(1),
            distance: -0.25,
            is_inside: true,
        },
    ];
    let mut docs = write_rows(m, "PointQuery", &queries, flat::point_query);
    docs.push(r#"{"collider":2,"distance":0,"is_inside":"true"}"#.to_string());
    docs.push("[2,0.5,false]".to_string());
    read_rows::<PointQuery>(m, "PointQuery", &docs, flat::point_query);

    // ── Spring, Spring2d, Spring3d ──
    let mut springs = vec![
        Spring::new(10.0, 90.0, 200.0, 12.0),
        Spring::critically_damped(0.0, 100.0, 300.0),
        Spring::new(f64::NAN, 1.0, 0.0, 0.0),
    ];
    let mut moving = Spring::under_damped(0.0, 1.0, 400.0);
    moving.step(0.05);
    springs.push(moving);
    for _ in 0..6 {
        let mut s = Spring::new(
            rng.uniform(-100.0, 100.0),
            rng.uniform(-100.0, 100.0),
            rng.log_uniform(0.0, 3.0),
            rng.log_uniform(-1.0, 2.0),
        );
        s.set_velocity(rng.uniform(-50.0, 50.0));
        s.set_settle_threshold(rng.log_uniform(-4.0, 0.0));
        springs.push(s);
    }
    let mut docs = write_rows(m, "Spring", &springs, flat::spring);
    docs.extend(
        [
            r#"{"position":0,"velocity":0,"target":1,"stiffness":1,"damping":1}"#,
            r#"{"position":0,"velocity":0,"target":1,"stiffness":1,"damping":1,"settle_threshold":0.01,"cache":1}"#,
            "[0,0,1,1,1,0.01]",
            "[0,0,1,1,1]",
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<Spring>(m, "Spring", &docs, flat::spring);
    let s2 = vec![
        Spring2d::new([0.0, 1.0], [2.0, 3.0], 4.0, 5.0),
        Spring2d::critically_damped([0.0, 0.0], [50.0, 75.0], 300.0),
    ];
    let mut docs = write_rows(m, "Spring2d", &s2, flat::spring2d);
    docs.push("[[0,0,1,1,1,0.01],[0,0,1,1,1,0.01]]".to_string());
    docs.push(r#"{"x":[0,0,1,1,1,0.01]}"#.to_string());
    read_rows::<Spring2d>(m, "Spring2d", &docs, flat::spring2d);
    let s3 = vec![Spring3d::new(
        [0.0, 0.0, 0.0],
        [10.0, 20.0, 30.0],
        300.0,
        20.0,
    )];
    let mut docs = write_rows(m, "Spring3d", &s3, flat::spring3d);
    docs.push("[[0,0,1,1,1,0.01],[0,0,1,1,1,0.01],[0,0,1,1,1,0.01]]".to_string());
    read_rows::<Spring3d>(m, "Spring3d", &docs, flat::spring3d);
}

fn sub_emitter(x: &SubEmitter) -> Vec<Tok> {
    vec![
        Tok::I(i64::from(x.count)),
        f(x.speed),
        f(x.lifetime),
        f(x.radius),
        f(x.gravity_scale),
        f(x.restitution),
    ]
}

fn rand_sub(rng: &mut Rng) -> SubEmitter {
    SubEmitter {
        count: u32::try_from(rng.int(0, 40)).unwrap_or(0),
        speed: rng.uniform(0.0, 10.0),
        lifetime: rng.log_uniform(-2.0, 1.0),
        radius: rng.uniform(0.0, 0.2),
        gravity_scale: rng.uniform(-1.0, 2.0),
        restitution: rng.uniform(0.0, 1.0),
    }
}

/// An emitter read from a document, the only way to set its crate-private spawn accumulator.
fn emitter_with_accumulator(acc: &str) -> ParticleEmitter {
    let doc = format!(
        r#"{{"handle":7,"position":[1,2,0],"velocity":[0,5,0],"velocity_spread":[1,0.5,0],"rate":60,"particle_lifetime":2,"particle_radius":0.05,"particle_restitution":0.3,"particle_gravity_scale":1,"particle_damping":0,"active":true,"accumulator":{acc}}}"#
    );
    serde_json::from_str(&doc).unwrap_or_else(|_| ParticleEmitter::new([0.0; 3], [0.0; 3], 1.0))
}

/// `particle.rs`'s types (M3d): ForceField, SubEmitter, Particle, ParticleEmitter.
#[allow(clippy::too_many_lines)] // one block per type
fn emit_particles(m: &mut Module, rng: &mut Rng) {
    // ── ForceField ──
    let mut fields = vec![
        ForceField::Radial {
            center: [0.0, 0.0, 0.0],
            strength: 100.0,
            falloff: 2.0,
            radius: 0.0,
        },
        ForceField::Radial {
            center: [1.5, -2.0, 0.25],
            strength: -6.5,
            falloff: 1.5,
            radius: 4.0,
        },
        ForceField::Directional {
            force: [3.0, 1.0, 0.0],
            min: [-1.0, -1.0, -1.0],
            max: [1.0, 1.0, 1.0],
        },
        ForceField::Radial {
            center: [0.0, 0.0, 0.0],
            strength: f64::NAN,
            falloff: f64::INFINITY,
            radius: 0.0,
        },
    ];
    for _ in 0..8 {
        fields.push(ForceField::Radial {
            center: rand3(rng, -3.0, 3.0),
            strength: rng.signed_log_uniform(-2.0, 3.0),
            falloff: rng.uniform(0.0, 3.0),
            radius: rng.uniform(0.0, 5.0),
        });
        fields.push(ForceField::Directional {
            force: rand3(rng, -3.0, 3.0),
            min: rand3(rng, -3.0, 0.0),
            max: rand3(rng, 0.0, 3.0),
        });
    }
    let mut docs = write_rows(m, "ForceField", &fields, flat::force_field);
    docs.extend(
        [
            r#""Radial""#,
            r#"{"Radial":null}"#,
            r#"{"Radial":[[0,0,0],1,2,0]}"#,
            r#"{"Directional":[[1,0,0],[-1,-1,-1],[1,1,1]]}"#,
            r#"{"Radial":{"center":[0,0,0],"strength":1,"falloff":2}}"#,
            r#"{"Radial":{"center":[0,0],"strength":1,"falloff":2,"radius":0}}"#,
            r#"{"Radial":{"center":[0,0,0],"strength":1,"falloff":2,"radius":0,"extra":true}}"#,
            r#"{"Vortex":{"center":[0,0,0]}}"#,
            r#"{"Radial":{"center":[0,0,0],"strength":1,"falloff":2,"radius":0},"Directional":{"force":[0,0,0],"min":[0,0,0],"max":[0,0,0]}}"#,
            r#"{"Directional":{"force":[0,0,0],"min":[0,0,0],"max":null}}"#,
            r#""radial""#,
            "[]",
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ForceField>(m, "ForceField", &docs, flat::force_field);

    // ── SubEmitter ──
    let mut subs = vec![
        SubEmitter {
            count: 4,
            speed: 3.0,
            lifetime: 2.0,
            radius: 0.02,
            gravity_scale: 0.0,
            restitution: 0.5,
        },
        SubEmitter {
            count: u32::MAX,
            speed: -1.0,
            lifetime: 0.0,
            radius: f64::NAN,
            gravity_scale: 1.0,
            restitution: 1.0,
        },
    ];
    for _ in 0..8 {
        subs.push(rand_sub(rng));
    }
    let mut docs = write_rows(m, "SubEmitter", &subs, sub_emitter);
    docs.extend(
        [
            r#"{"count":4,"speed":3,"lifetime":2,"radius":0.02,"gravity_scale":0}"#,
            r#"{"count":-1,"speed":3,"lifetime":2,"radius":0.02,"gravity_scale":0,"restitution":0.5}"#,
            r#"{"count":4.0,"speed":3,"lifetime":2,"radius":0.02,"gravity_scale":0,"restitution":0.5}"#,
            r#"{"count":4294967296,"speed":3,"lifetime":2,"radius":0.02,"gravity_scale":0,"restitution":0.5}"#,
            r#"[6,1.5,0.5,0.04,0.5,0.6]"#,
            r#"[6,1.5,0.5,0.04,0.5]"#,
            r#"[6,1.5,0.5,0.04,0.5,0.6,7]"#,
            r#"{"count":4,"speed":"3","lifetime":2,"radius":0.02,"gravity_scale":0,"restitution":0.5}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<SubEmitter>(m, "SubEmitter", &docs, sub_emitter);

    // ── Particle ──
    let mut particles = vec![
        Particle::new([1.0, 2.0, 0.0], [3.0, 4.0, 0.0], 5.0),
        Particle::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 3.0)
            .with_radius(0.1)
            .with_drag(0.5)
            .with_restitution(0.8)
            .with_gravity_scale(0.5)
            .with_damping(0.1),
        Particle::new([5.0, 5.0, 0.0], [0.0, 0.0, 0.0], 0.025).with_sub_emitter(SubEmitter {
            count: 4,
            speed: 3.0,
            lifetime: 2.0,
            radius: 0.02,
            gravity_scale: 0.0,
            restitution: 0.5,
        }),
        Particle::new([0.0, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0], f64::NAN),
    ];
    for _ in 0..10 {
        let mut p = Particle::new(
            rand3(rng, -3.0, 3.0),
            rand3(rng, -3.0, 3.0),
            rng.log_uniform(-2.0, 2.0),
        )
        .with_radius(rng.uniform(0.0, 0.2))
        .with_drag(rng.uniform(0.0, 2.0))
        .with_restitution(rng.uniform(0.0, 1.0))
        .with_gravity_scale(rng.uniform(-1.0, 2.0))
        .with_damping(rng.uniform(0.0, 1.0));
        p.handle = impetus::ParticleHandle(rand_handle(rng));
        if rng.unit() < 0.4 {
            p = p.with_sub_emitter(rand_sub(rng));
        }
        particles.push(p);
    }
    let mut docs = write_rows(m, "Particle", &particles, flat::particle);
    docs.extend(
        [
            r#"{"handle":3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0}"#,
            r#"{"handle":3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0,"on_death_emit":null}"#,
            r#"{"handle":3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1}"#,
            r#"{"handle":-3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0}"#,
            r#"{"handle":3,"position":[1,2],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0}"#,
            r#"{"handle":3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":null,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0}"#,
            r#"[3,[1,2,0],[0,0,0],1,0.05,0,0.3,1,0]"#,
            r#"[3,[1,2,0],[0,0,0],1,0.05,0,0.3,1,0,null]"#,
            r#"[3,[1,2,0],[0,0,0],1,0.05,0,0.3,1,0,[2,1,0.5,0.01,0,0.4]]"#,
            r#"[3,[1,2,0],[0,0,0],1,0.05,0,0.3,1,0,null,5]"#,
            r#"[3,[1,2,0],[0,0,0],1,0.05,0,0.3,1]"#,
            r#"{"handle":3,"position":[1,2,0],"velocity":[0,0,0],"lifetime":1,"radius":0.05,"drag":0,"restitution":0.3,"gravity_scale":1,"damping":0,"on_death_emit":{"count":2}}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<Particle>(m, "Particle", &docs, flat::particle);

    // ── ParticleEmitter ──
    let mut emitters = vec![
        ParticleEmitter::new([0.0, 0.0, 0.0], [0.0, 10.0, 0.0], 100.0),
        ParticleEmitter::new([1.0, 2.0, 0.0], [3.0, 4.0, 0.0], 10.0)
            .with_spread([1.0, 2.0, 0.0])
            .with_lifetime(3.0)
            .with_radius(0.2)
            .with_gravity_scale(0.0)
            .with_damping(0.5),
        emitter_with_accumulator("0.3700000000000001"),
        emitter_with_accumulator("1e-300"),
        ParticleEmitter::new([0.0; 3], [0.0; 3], f64::INFINITY),
    ];
    let mut off = ParticleEmitter::new([0.0, 1.0, 0.0], [0.0, 1.0, 0.0], 30.0);
    off.active = false;
    emitters.push(off);
    for _ in 0..8 {
        let mut e = ParticleEmitter::new(
            rand3(rng, -3.0, 3.0),
            rand3(rng, -3.0, 3.0),
            rng.log_uniform(-1.0, 3.0),
        )
        .with_spread(rand3(rng, -1.0, 1.0))
        .with_lifetime(rng.log_uniform(-1.0, 1.0))
        .with_radius(rng.uniform(0.0, 0.2))
        .with_gravity_scale(rng.uniform(-1.0, 2.0))
        .with_damping(rng.uniform(0.0, 1.0));
        e.handle = impetus::EmitterHandle(rand_handle(rng));
        e.active = rng.unit() < 0.8;
        emitters.push(e);
    }
    let mut docs = write_rows(m, "ParticleEmitter", &emitters, flat::emitter);
    docs.extend(
        [
            r#"{"handle":0,"position":[0,0,0],"velocity":[0,1,0],"velocity_spread":[0,0,0],"rate":5,"particle_lifetime":2,"particle_radius":0.05,"particle_restitution":0.3,"particle_gravity_scale":1,"particle_damping":0,"active":true}"#,
            r#"{"handle":0,"position":[0,0,0],"velocity":[0,1,0],"velocity_spread":[0,0,0],"rate":5,"particle_lifetime":2,"particle_radius":0.05,"particle_restitution":0.3,"particle_gravity_scale":1,"particle_damping":0,"active":1,"accumulator":0}"#,
            r#"{"handle":0,"position":[0,0,0],"velocity":[0,1,0],"velocity_spread":[0,0,0],"rate":null,"particle_lifetime":2,"particle_radius":0.05,"particle_restitution":0.3,"particle_gravity_scale":1,"particle_damping":0,"active":true,"accumulator":0}"#,
            r#"[0,[0,0,0],[0,1,0],[0,0,0],5,2,0.05,0.3,1,0,false,0.25]"#,
            r#"[0,[0,0,0],[0,1,0],[0,0,0],5,2,0.05,0.3,1,0,false]"#,
            r#"[0,[0,0,0],[0,1,0],[0,0,0],5,2,0.05,0.3,1,0,false,0.25,1]"#,
            r#"{"handle":0,"position":[0,0,0],"velocity":[0,1,0],"velocity_spread":[0,0],"rate":5,"particle_lifetime":2,"particle_radius":0.05,"particle_restitution":0.3,"particle_gravity_scale":1,"particle_damping":0,"active":true,"accumulator":0}"#,
        ]
        .iter()
        .map(|d| (*d).to_string()),
    );
    read_rows::<ParticleEmitter>(m, "ParticleEmitter", &docs, flat::emitter);
}
