//! `broadphase.tsv` — the dynamic AABB tree and the spatial hash (M2a), through the oracle's
//! pair hooks (`aabb_tree_pairs_{2d,3d}`, `spatial_hash_pairs_{2d,3d}`,
//! `spatial_hash_auto_cell_size`). The structures are shared by both backends, so the 2D build
//! emits the 3D rows too.
//!
//! A row's inputs are the boxes in insertion order (`v:N`, then min and max per box; the hash
//! rows start with the cell size); its output is the candidate pair set the structure reports,
//! sorted (`v:P`, then the two input indices of each pair). Pair sets are class A: the tree
//! reports exactly the pairs whose fattened boxes overlap, the hash those that share a cell (or
//! involve an oversized entry), whatever their internal layout.

use impetus::oracle;

use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, f, i};

type Box2 = ([f64; 2], [f64; 2]);
type Box3 = ([f64; 3], [f64; 3]);

fn pairs_out(p: &[(u32, u32)]) -> Vec<Tok> {
    let mut t = vec![Tok::V(p.len())];
    for &(a, b) in p {
        t.extend([i(a), i(b)]);
    }
    t
}

fn boxes2_in(bx: &[Box2]) -> Vec<Tok> {
    let mut t = vec![Tok::V(bx.len())];
    for (mn, mx) in bx {
        t.extend([f(mn[0]), f(mn[1]), f(mx[0]), f(mx[1])]);
    }
    t
}

fn boxes3_in(bx: &[Box3]) -> Vec<Tok> {
    let mut t = vec![Tok::V(bx.len())];
    for (mn, mx) in bx {
        t.extend([f(mn[0]), f(mn[1]), f(mn[2]), f(mx[0]), f(mx[1]), f(mx[2])]);
    }
    t
}

fn tree2(m: &mut Module, bx: &[Box2]) {
    let p = oracle::aabb_tree_pairs_2d(bx);
    m.row(
        "aabb_tree_pairs_2d",
        Class::A,
        &boxes2_in(bx),
        &pairs_out(&p),
    );
}

fn tree3(m: &mut Module, bx: &[Box3]) {
    let p = oracle::aabb_tree_pairs_3d(bx);
    m.row(
        "aabb_tree_pairs_3d",
        Class::A,
        &boxes3_in(bx),
        &pairs_out(&p),
    );
}

fn hash2(m: &mut Module, cell: f64, bx: &[Box2]) {
    let p = oracle::spatial_hash_pairs_2d(cell, bx);
    let mut ins = vec![f(cell)];
    ins.extend(boxes2_in(bx));
    m.row("spatial_hash_pairs_2d", Class::A, &ins, &pairs_out(&p));
}

fn hash3(m: &mut Module, cell: f64, bx: &[Box3]) {
    let p = oracle::spatial_hash_pairs_3d(cell, bx);
    let mut ins = vec![f(cell)];
    ins.extend(boxes3_in(bx));
    m.row("spatial_hash_pairs_3d", Class::A, &ins, &pairs_out(&p));
}

fn sq(x: f64, y: f64, w: f64) -> Box2 {
    ([x, y], [x + w, y + w])
}

fn cube(x: f64, y: f64, z: f64, w: f64) -> Box3 {
    ([x, y, z], [x + w, y + w, z + w])
}

/// n random squares in [0, span)², sides in [lo, hi).
fn random2(rng: &mut Rng, n: usize, span: f64, lo: f64, hi: f64) -> Vec<Box2> {
    (0..n)
        .map(|_| {
            let x = rng.uniform(0.0, span);
            let y = rng.uniform(0.0, span);
            let w = rng.uniform(lo, hi);
            sq(x, y, w)
        })
        .collect()
}

/// n random boxes in [−span, span)³, sides in [lo, hi) per axis.
fn random3(rng: &mut Rng, n: usize, span: f64, lo: f64, hi: f64) -> Vec<Box3> {
    (0..n)
        .map(|_| {
            let mn = [
                rng.uniform(-span, span),
                rng.uniform(-span, span),
                rng.uniform(-span, span),
            ];
            let mx = [
                mn[0] + rng.uniform(lo, hi),
                mn[1] + rng.uniform(lo, hi),
                mn[2] + rng.uniform(lo, hi),
            ];
            (mn, mx)
        })
        .collect()
}

/// A row of n unit squares, `pitch` apart (the Rust test's sorted inserts, and the bench's row).
fn row2(n: u32, pitch: f64) -> Vec<Box2> {
    (0..n).map(|k| sq(f64::from(k) * pitch, 0.0, 1.0)).collect()
}

/// Boxes that break the arithmetic: NaN, infinite, inverted, empty, huge and far.
fn odd2() -> Vec<Box2> {
    vec![
        ([f64::NAN, 0.0], [f64::NAN, 1.0]),
        ([0.0, 0.0], [1.0, 1.0]),
        ([f64::NEG_INFINITY, 0.0], [f64::INFINITY, 1.0]),
        ([2.0, 2.0], [1.0, 1.0]),
        ([0.5, 0.5], [0.5, 0.5]),
        ([-1e300, -1e300], [1e300, 1e300]),
        ([1e12, 1e12], [1e12 + 1.0, 1e12 + 1.0]),
        ([-3.0, -3.0], [-2.5, -2.5]),
        ([0.25, f64::NAN], [0.75, 0.9]),
        ([0.0, 0.0], [f64::INFINITY, f64::INFINITY]),
    ]
}

#[allow(clippy::too_many_lines)] // one block per structure
pub fn emit() -> Module {
    let mut m = Module::new("broadphase");
    let mut rng = Rng::new(0xb0ad);

    // ── the AABB tree, 2D ──
    tree2(&mut m, &[]);
    tree2(&mut m, &[sq(0.0, 0.0, 1.0)]);
    tree2(
        &mut m,
        &[sq(0.0, 0.0, 1.0), sq(0.5, 0.5, 1.0), sq(10.0, 10.0, 1.0)],
    );
    tree2(&mut m, &[sq(0.0, 0.0, 1.0), sq(5.0, 5.0, 1.0)]);
    tree2(&mut m, &vec![sq(0.0, 0.0, 1.0); 10]);
    // The fattening margin (0.1 a side): boxes 0.2 apart touch after fattening, a hair either way.
    for gap in [0.2, 0.2_f64.next_up(), 0.2_f64.next_down(), 0.19, 0.21] {
        tree2(&mut m, &[sq(0.0, 0.0, 1.0), sq(1.0 + gap, 0.0, 1.0)]);
        tree2(&mut m, &[sq(0.0, 0.0, 1.0), sq(0.0, 1.0 + gap, 1.0)]);
    }
    tree2(&mut m, &odd2());
    for n in [5, 20, 60, 200] {
        let bx = random2(&mut rng, n, 50.0, 0.5, 3.5);
        tree2(&mut m, &bx);
    }
    tree2(&mut m, &random2(&mut rng, 100, 10.0, 0.01, 0.3));
    tree2(&mut m, &random2(&mut rng, 100, 2000.0, 1.0, 100.0));
    tree2(&mut m, &row2(1000, 1.0));
    tree2(&mut m, &row2(1000, 1.5));
    tree2(&mut m, &random2(&mut rng, 1000, 100.0, 0.5, 2.0));

    // ── the AABB tree, 3D ──
    tree3(&mut m, &[]);
    tree3(
        &mut m,
        &[cube(0.0, 0.0, 0.0, 1.0), cube(0.5, 0.5, 0.5, 1.0)],
    );
    tree3(
        &mut m,
        &[cube(0.0, 0.0, 0.0, 1.0), cube(0.0, 0.0, 1.2, 1.0)],
    );
    tree3(
        &mut m,
        &[
            ([f64::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]),
            cube(0.0, 0.0, 0.0, 1.0),
            ([0.0, 0.0, f64::NEG_INFINITY], [1.0, 1.0, f64::INFINITY]),
            cube(0.5, 0.5, 5.0, 0.2),
        ],
    );
    for n in [10, 80, 300] {
        let bx = random3(&mut rng, n, 20.0, 0.5, 4.0);
        tree3(&mut m, &bx);
    }
    tree3(
        &mut m,
        &(0..500_u32)
            .map(|k| cube(f64::from(k) * 1.5, 0.0, 0.0, 1.0))
            .collect::<Vec<_>>(),
    );

    // ── the spatial hash, 2D ──
    let layout = vec![
        ([0.0, 0.0], [0.5, 0.5]),
        ([0.2, 0.2], [0.8, 0.8]),
        ([10.0, 10.0], [10.5, 10.5]),
        ([0.9, 0.9], [1.1, 1.1]),
        ([-0.5, -0.5], [-0.1, -0.1]),
        ([1.0, 0.0], [1.0, 0.5]),
    ];
    for cell in [
        1.0,
        0.5,
        2.5,
        3.0,
        0.0,
        -2.0,
        1e-11,
        1e-10,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        hash2(&mut m, cell, &layout);
    }
    hash2(&mut m, 1.0, &[]);
    hash2(&mut m, 1.0, &[([0.0, 0.0], [0.5, 0.5])]);
    hash2(
        &mut m,
        1.0,
        &[([0.0, 0.0], [0.5, 0.5]), ([10.0, 10.0], [10.5, 10.5])],
    );
    hash2(
        &mut m,
        0.1,
        &[
            ([f64::NAN, 0.0], [f64::NAN, 1.0]),
            ([-1e12, -1e12], [1e12, 1e12]),
            ([f64::NEG_INFINITY, 0.0], [f64::INFINITY, 1.0]),
            ([0.0, 0.0], [0.05, 0.05]),
        ],
    );
    // The oversize cut, 4,096 cells: 64×64 rasterises, 64×65 and 65×64 don't.
    hash2(
        &mut m,
        1.0,
        &[
            ([0.0, 0.0], [63.5, 63.5]),
            ([0.0, 0.0], [63.5, 64.5]),
            ([0.0, 0.0], [64.5, 63.5]),
            ([30.0, 30.0], [30.5, 30.5]),
            ([200.0, 200.0], [200.5, 200.5]),
        ],
    );
    // Exact cell boundaries, both signs, and boxes past the i32 cell range.
    hash2(
        &mut m,
        1.0,
        &[
            ([1.0, 1.0], [2.0, 2.0]),
            ([2.0, 2.0], [3.0, 3.0]),
            ([-1.0, -1.0], [0.0, 0.0]),
            ([0.0, 0.0], [0.0, 0.0]),
            ([-0.0, -0.0], [-0.0, -0.0]),
            ([2_147_483_647.0, 0.0], [2_147_483_647.5, 0.5]),
            ([2_147_483_648.0, 0.0], [2_147_483_648.5, 0.5]),
            ([-2_147_483_648.0, 0.0], [-2_147_483_647.5, 0.5]),
            ([-2_147_483_649.0, 0.0], [-2_147_483_648.5, 0.5]),
        ],
    );
    hash2(&mut m, 1.0, &odd2());
    for (n, cell) in [(10, 1.0), (60, 2.0), (200, 4.0), (200, 0.7)] {
        let bx = random2(&mut rng, n, 50.0, 0.5, 3.5);
        hash2(&mut m, cell, &bx);
    }
    let big = random2(&mut rng, 1000, 100.0, 0.5, 2.0);
    let dims: Vec<f64> = big
        .iter()
        .map(|(mn, mx)| (mx[0] - mn[0]).max(mx[1] - mn[1]))
        .collect();
    hash2(&mut m, oracle::spatial_hash_auto_cell_size(&dims), &big);
    hash2(&mut m, 2.0, &row2(1000, 1.5));

    // ── the spatial hash, 3D ──
    hash3(
        &mut m,
        2.0,
        &[cube(0.0, 0.0, 0.0, 1.0), cube(0.5, 0.5, 0.5, 1.0)],
    );
    hash3(
        &mut m,
        1.0,
        &[cube(0.0, 0.0, 0.0, 0.5), cube(10.0, 10.0, 10.0, 0.5)],
    );
    // 16×16×16 = 4,096 cells rasterises; 16×16×17 doesn't.
    hash3(
        &mut m,
        1.0,
        &[
            ([0.0, 0.0, 0.0], [15.5, 15.5, 15.5]),
            ([0.0, 0.0, 0.0], [15.5, 15.5, 16.5]),
            cube(7.0, 7.0, 7.0, 0.5),
            cube(100.0, 100.0, 100.0, 0.5),
        ],
    );
    hash3(
        &mut m,
        0.5,
        &[
            ([f64::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]),
            cube(0.0, 0.0, 0.0, 1.0),
            ([0.0, 0.0, -1e15], [1.0, 1.0, 1e15]),
            cube(-1.0, -1.0, -1.0, 0.25),
        ],
    );
    for (n, cell) in [(20, 2.0), (150, 3.0)] {
        let bx = random3(&mut rng, n, 20.0, 0.5, 4.0);
        hash3(&mut m, cell, &bx);
    }

    // ── the automatic cell size ──
    let dim_sets: Vec<Vec<f64>> = vec![
        vec![],
        vec![1.0, 2.0],
        vec![1.0, f64::NAN, f64::NEG_INFINITY, 3.0],
        vec![-1.0, 2.0],
        vec![0.0],
        vec![0.001],
        vec![f64::MAX, f64::MAX],
        vec![f64::INFINITY],
        vec![f64::NAN],
        dims.clone(),
    ];
    let mut all_sets = dim_sets;
    for n in [3, 17, 100] {
        all_sets.push((0..n).map(|_| rng.log_uniform(-3.0, 3.0)).collect());
    }
    for d in &all_sets {
        let mut ins = vec![Tok::V(d.len())];
        ins.extend(d.iter().map(|&x| f(x)));
        let r = oracle::spatial_hash_auto_cell_size(d);
        m.row("spatial_hash_auto_cell_size", Class::B, &ins, &[f(r)]);
    }
    m
}
