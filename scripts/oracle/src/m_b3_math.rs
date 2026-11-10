//! `b3_math.tsv` — glam 0.29.3's f64 3D math (M4, ADR 0012): the `DVec3`, `DQuat` and `DMat3` operations the 3D
//! backend calls, which the port transcribes by value in `src/math3.cyr`.
//!
//! Tokens are flat f64s: a vector is 3 (x, y, z), a quaternion 4 (x, y, z, w), a matrix 9 by columns (x_axis,
//! y_axis, z_axis). A bool is `i:0` / `i:1` and `Option::None` is `n:`. Class A where each output is one IEEE
//! operation (± × ÷, abs, min, max, a sign flip), B for compositions of + − × ÷ √, C for the rotations built on
//! `sin_cos` (gated against the corrected oracle, ADR 0011). Every input passes through `black_box`.

use std::hint::black_box;

use hisab::{DMat3, DQuat, DVec3, DVec4};

use crate::grids::{Rng, specials};
use crate::tsv::{Class, Module, Tok, b, f};

fn v3(v: DVec3) -> Vec<Tok> {
    vec![f(v.x), f(v.y), f(v.z)]
}

fn q4(q: DQuat) -> Vec<Tok> {
    vec![f(q.x), f(q.y), f(q.z), f(q.w)]
}

fn m9(m: DMat3) -> Vec<Tok> {
    m.to_cols_array().iter().map(|&x| f(x)).collect()
}

fn cat(parts: &[Vec<Tok>]) -> Vec<Tok> {
    parts.concat()
}

/// Edge vectors (zeros of each sign, the axes, a subnormal, values whose squares overflow or underflow, ±∞, NaN),
/// then seeded ones: uniform in [−10, 10] and signed log-uniform over 10^±20.
fn vectors() -> Vec<DVec3> {
    let mut v = vec![
        DVec3::ZERO,
        DVec3::new(-0.0, -0.0, -0.0),
        DVec3::X,
        DVec3::Y,
        DVec3::Z,
        DVec3::NEG_X,
        DVec3::NEG_Y,
        DVec3::NEG_Z,
        DVec3::ONE,
        DVec3::new(1e-300, 0.0, -0.0),
        DVec3::new(f64::from_bits(1), 0.0, 0.0),
        DVec3::new(f64::MIN_POSITIVE, -0.0, 3.0),
        DVec3::new(1e200, -1e200, 0.0),
        DVec3::new(f64::MAX, 1.0, 1.0),
        DVec3::new(f64::INFINITY, 0.0, 0.0),
        DVec3::new(1.0, f64::NEG_INFINITY, 2.0),
        DVec3::new(f64::NAN, 1.0, 2.0),
        DVec3::new(3.0, 3.0, -1.0),
        DVec3::new(-2.0, 2.0, 0.5),
    ];
    let mut rng = Rng::new(0xb3a7_0001);
    for _ in 0..150 {
        v.push(DVec3::new(rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)));
    }
    for _ in 0..60 {
        v.push(DVec3::new(
            rng.signed_log_uniform(-20.0, 20.0),
            rng.signed_log_uniform(-20.0, 20.0),
            rng.signed_log_uniform(-20.0, 20.0),
        ));
    }
    v
}

/// Scalars for `v * s`, `v / s` and `M * s`: the specials, small integers and halves, then seeded values.
fn scalars() -> Vec<f64> {
    let mut s = specials();
    s.extend([0.5, 2.0, -3.0, 1e-10, 1e10, 1.0 / 60.0]);
    let mut rng = Rng::new(0xb3a7_0002);
    s.extend((0..40).map(|_| rng.signed_log_uniform(-10.0, 10.0)));
    s
}

/// Unit vectors for `any_orthonormal_pair`: the axes (z of each sign and ±0), then normalized seeded vectors.
fn units() -> Vec<DVec3> {
    let mut u = vec![
        DVec3::X,
        DVec3::Y,
        DVec3::Z,
        DVec3::NEG_X,
        DVec3::NEG_Y,
        DVec3::NEG_Z,
        DVec3::new(1.0, 0.0, -0.0),
        DVec3::new(0.0, 1.0, -0.0),
        DVec3::new(f64::NAN, 0.0, 1.0),
    ];
    let mut rng = Rng::new(0xb3a7_0003);
    for _ in 0..80 {
        let v = DVec3::new(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
        u.push(v.normalize());
    }
    u
}

/// Rotation angles: zeros, the quarter turns and their neighbours, a tiny and a huge angle, ±∞ and NaN, then seeded
/// angles in [−10, 10].
fn angles() -> Vec<f64> {
    let pi = std::f64::consts::PI;
    let mut a = vec![
        0.0,
        -0.0,
        pi / 4.0,
        pi / 2.0,
        pi,
        -pi,
        2.0 * pi,
        -pi / 3.0,
        1e-8,
        -1e-300,
        1e8,
        1e300,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ];
    let mut rng = Rng::new(0xb3a7_0004);
    a.extend((0..100).map(|_| rng.uniform(-10.0, 10.0)));
    a
}

/// Quaternions: the identity and its negation, the three axis rotations, seeded unit quaternions, then non-unit
/// ones for `normalize` (seeded, zero, NaN, components whose squares overflow).
fn quats() -> Vec<DQuat> {
    let mut q = vec![
        DQuat::IDENTITY,
        DQuat::from_xyzw(0.0, 0.0, 0.0, -1.0),
        DQuat::from_rotation_x(0.3),
        DQuat::from_rotation_y(-1.2),
        DQuat::from_rotation_z(2.5),
    ];
    let mut rng = Rng::new(0xb3a7_0005);
    for _ in 0..80 {
        let v = DVec4::new(
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
        );
        q.push(DQuat::from_vec4(v.normalize()));
    }
    for _ in 0..20 {
        q.push(DQuat::from_xyzw(
            rng.uniform(-5.0, 5.0),
            rng.uniform(-5.0, 5.0),
            rng.uniform(-5.0, 5.0),
            rng.uniform(-5.0, 5.0),
        ));
    }
    q.extend([
        DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        DQuat::from_xyzw(f64::NAN, 0.0, 0.0, 1.0),
        DQuat::from_xyzw(1e200, 1e200, 0.0, 1.0),
        DQuat::from_xyzw(1e-200, 0.0, 0.0, 1e-200),
    ]);
    q
}

/// Matrices: seeded dense ones, rotations, diagonal inertia-like ones, then singular and non-finite ones.
fn mats() -> Vec<DMat3> {
    let mut rng = Rng::new(0xb3a7_0006);
    let mut m = vec![DMat3::IDENTITY, DMat3::ZERO];
    for _ in 0..60 {
        let c: Vec<f64> = (0..9).map(|_| rng.uniform(-5.0, 5.0)).collect();
        m.push(DMat3::from_cols_slice(&c));
    }
    for _ in 0..20 {
        let v = DVec4::new(
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
        );
        m.push(DMat3::from_quat(DQuat::from_vec4(v.normalize())));
    }
    for _ in 0..20 {
        m.push(DMat3::from_diagonal(DVec3::new(
            rng.log_uniform(-6.0, 3.0),
            rng.log_uniform(-6.0, 3.0),
            rng.log_uniform(-6.0, 3.0),
        )));
    }
    let a = DVec3::new(1.0, 2.0, 3.0);
    let b = DVec3::new(-2.0, 0.5, 4.0);
    m.extend([
        // Rank 1 and rank 2: determinant 0, so the inverse is ±∞ or NaN.
        DMat3::from_cols(a, a * 2.0, a * -0.5),
        DMat3::from_cols(a, b, a + b),
        DMat3::from_diagonal(DVec3::new(1.0, 0.0, 2.0)),
        DMat3::from_diagonal(DVec3::new(1e-200, 1e-200, 1e-200)),
        DMat3::from_cols(DVec3::new(f64::NAN, 0.0, 0.0), DVec3::Y, DVec3::Z),
        DMat3::from_cols(DVec3::new(f64::INFINITY, 0.0, 0.0), DVec3::Y, DVec3::Z),
    ]);
    m
}

/// Pairs (xs[k], xs[(7k + 3) mod n]) for every k.
fn pairs<T: Copy>(xs: &[T]) -> Vec<(T, T)> {
    let n = xs.len();
    (0..n).map(|k| (xs[k], xs[(7 * k + 3) % n])).collect()
}

fn emit_vectors(m: &mut Module) {
    let vs = vectors();
    let ps = pairs(&vs);
    let ss = scalars();
    for &(a, c) in &ps {
        let (a, c) = (black_box(a), black_box(c));
        let ins = cat(&[v3(a), v3(c)]);
        m.row("DVec3::add", Class::A, &ins, &v3(a + c));
        m.row("DVec3::sub", Class::A, &ins, &v3(a - c));
        m.row("DVec3::mul", Class::A, &ins, &v3(a * c));
        m.row("DVec3::min", Class::A, &ins, &v3(a.min(c)));
        m.row("DVec3::max", Class::A, &ins, &v3(a.max(c)));
        m.row("DVec3::dot", Class::B, &ins, &[f(a.dot(c))]);
        m.row("DVec3::cross", Class::B, &ins, &v3(a.cross(c)));
        m.row("DVec3::normalize_or", Class::B, &ins, &v3(a.normalize_or(c)));
    }
    for &a in &vs {
        let a = black_box(a);
        let ins = v3(a);
        m.row("DVec3::neg", Class::A, &ins, &v3(-a));
        m.row("DVec3::abs", Class::A, &ins, &v3(a.abs()));
        m.row("DVec3::is_finite", Class::A, &ins, &[b(a.is_finite())]);
        m.row("DVec3::length_squared", Class::B, &ins, &[f(a.length_squared())]);
        m.row("DVec3::length", Class::B, &ins, &[f(a.length())]);
        m.row("DVec3::length_recip", Class::B, &ins, &[f(a.length_recip())]);
        m.row("DVec3::normalize", Class::B, &ins, &v3(a.normalize()));
        let tn = a.try_normalize().map_or_else(|| vec![Tok::None], v3);
        m.row("DVec3::try_normalize", Class::B, &ins, &tn);
        m.row("DVec3::normalize_or_zero", Class::B, &ins, &v3(a.normalize_or_zero()));
        m.row("DVec3::any_orthogonal_vector", Class::A, &ins, &v3(a.any_orthogonal_vector()));
    }
    for (k, &a) in vs.iter().enumerate() {
        let s = black_box(ss[k % ss.len()]);
        let a = black_box(a);
        let ins = cat(&[v3(a), vec![f(s)]]);
        m.row("DVec3::mul_f64", Class::A, &ins, &v3(a * s));
        m.row("DVec3::div_f64", Class::A, &ins, &v3(a / s));
    }
    for &u in &units() {
        let u = black_box(u);
        let (p, q) = u.any_orthonormal_pair();
        m.row("DVec3::any_orthonormal_pair", Class::B, &v3(u), &cat(&[v3(p), v3(q)]));
    }
}

fn emit_quats(m: &mut Module) {
    for &a in &angles() {
        let a = black_box(a);
        m.row("DQuat::from_rotation_x", Class::C, &[f(a)], &q4(DQuat::from_rotation_x(a)));
        m.row("DQuat::from_rotation_y", Class::C, &[f(a)], &q4(DQuat::from_rotation_y(a)));
        m.row("DQuat::from_rotation_z", Class::C, &[f(a)], &q4(DQuat::from_rotation_z(a)));
    }
    let qs = quats();
    let vs = vectors();
    for (k, &q) in qs.iter().enumerate() {
        let q = black_box(q);
        let ins = q4(q);
        m.row("DQuat::conjugate", Class::A, &ins, &q4(q.conjugate()));
        m.row("DQuat::inverse", Class::A, &ins, &q4(q.inverse()));
        m.row("DQuat::is_finite", Class::A, &ins, &[b(q.is_finite())]);
        m.row("DQuat::length_squared", Class::B, &ins, &[f(q.length_squared())]);
        m.row("DQuat::normalize", Class::B, &ins, &q4(q.normalize()));
        let r = black_box(qs[(7 * k + 3) % qs.len()]);
        m.row("DQuat::mul_quat", Class::B, &cat(&[q4(q), q4(r)]), &q4(q * r));
        let v = black_box(vs[k % vs.len()]);
        m.row("DQuat::mul_vec3", Class::B, &cat(&[q4(q), v3(v)]), &v3(q * v));
    }
}

fn emit_mats(m: &mut Module) {
    for &q in &quats() {
        let q = black_box(q);
        m.row("DMat3::from_quat", Class::B, &q4(q), &m9(DMat3::from_quat(q)));
    }
    let ms = mats();
    let vs = vectors();
    let ss = scalars();
    for (k, &a) in ms.iter().enumerate() {
        let a = black_box(a);
        let ins = m9(a);
        m.row("DMat3::transpose", Class::A, &ins, &m9(a.transpose()));
        m.row("DMat3::neg", Class::A, &ins, &m9(-a));
        m.row("DMat3::determinant", Class::B, &ins, &[f(a.determinant())]);
        m.row("DMat3::inverse", Class::B, &ins, &m9(a.inverse()));
        let c = black_box(ms[(7 * k + 3) % ms.len()]);
        let ins2 = cat(&[m9(a), m9(c)]);
        m.row("DMat3::add_mat3", Class::A, &ins2, &m9(a + c));
        m.row("DMat3::sub_mat3", Class::A, &ins2, &m9(a - c));
        m.row("DMat3::mul_mat3", Class::B, &ins2, &m9(a * c));
        let v = black_box(vs[k % vs.len()]);
        m.row("DMat3::mul_vec3", Class::B, &cat(&[m9(a), v3(v)]), &v3(a * v));
        let s = black_box(ss[k % ss.len()]);
        m.row("DMat3::mul_scalar", Class::A, &cat(&[m9(a), vec![f(s)]]), &m9(a * s));
    }
    for &d in &vectors() {
        let d = black_box(d);
        m.row("DMat3::from_diagonal", Class::A, &v3(d), &m9(DMat3::from_diagonal(d)));
    }
}

pub fn emit() -> Module {
    let mut m = Module::new("b3_math");
    emit_vectors(&mut m);
    emit_quats(&mut m);
    emit_mats(&mut m);
    m
}
