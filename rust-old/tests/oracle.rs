//! Tests for the hidden `oracle` hooks that the Cyrius port's golden-value
//! generator reads (see `docs/development/impetus-port-plan.md`, M0 step 0).
//!
//! The hooks must be read-only, and must show the state the public API hides.

#![cfg(all(feature = "oracle", any(feature = "2d", feature = "3d")))]

use impetus::oracle::*;
use impetus::*;

fn collider(shape: ColliderShape) -> ColliderDesc {
    ColliderDesc {
        shape,
        offset: [0.0; 3],
        material: PhysicsMaterial::default(),
        is_sensor: false,
        mass: None,
        collision_layer: u32::MAX,
        collision_mask: u32::MAX,
    }
}

fn zero_g() -> WorldConfig {
    WorldConfig {
        gravity: [0.0; 3],
        ..WorldConfig::default()
    }
}

/// A static ground box with a unit box resting on it. Returns the world, the
/// box body and both collider handles (ground first).
fn resting_box() -> (PhysicsWorld, BodyHandle, ColliderHandle, ColliderHandle) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let gc = world.add_collider(
        ground,
        collider(ColliderShape::Box {
            half_extents: [5.0, 0.5, 5.0],
        }),
    );
    let body = world.add_body(BodyDesc {
        position: [0.0, 1.0, 0.0],
        ..BodyDesc::default()
    });
    let bc = world.add_collider(
        body,
        collider(ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.5],
        }),
    );
    (world, body, gc, bc)
}

/// Dumping every frame must not change the simulation by a single bit.
#[test]
fn dump_is_read_only() {
    let (mut dumped, ..) = resting_box();
    let (mut plain, ..) = resting_box();
    for _ in 0..60 {
        dumped.step();
        let _ = dumped.oracle_dump();
        plain.step();
    }
    assert_eq!(dumped.oracle_dump(), plain.oracle_dump());
}

/// The dump agrees with what the public API does show.
#[test]
fn dump_matches_public_state() {
    let (mut world, body, ..) = resting_box();
    for _ in 0..10 {
        world.step();
    }
    let dump = world.oracle_dump();
    assert_eq!(dump.config, *world.config());
    assert_eq!(dump.backend.bodies.len(), 2);
    assert_eq!(dump.backend.colliders.len(), 2);
    let b = dump
        .backend
        .bodies
        .iter()
        .find(|b| b.handle == body.0)
        .expect("box body in dump");
    let state = world.get_body_state(body).unwrap();
    assert_eq!(b.position[0].to_bits(), state.position[0].to_bits());
    assert_eq!(b.position[1].to_bits(), state.position[1].to_bits());
    assert_eq!(b.is_sleeping, state.is_sleeping);
}

/// The arena dump predicts the next handle `add_body` gives out.
#[test]
fn arena_dump_predicts_next_handle() {
    let mut world = PhysicsWorld::new(zero_g());
    let a = world.add_body(BodyDesc::default());
    let _b = world.add_body(BodyDesc::default());
    world.remove_body(a).unwrap();

    let arena = world.oracle_dump().backend.body_arena;
    assert_eq!(arena.slots.len(), 2);
    assert!(!arena.slots[0].occupied);
    assert!(arena.slots[1].occupied);
    let head = arena.free_head.expect("a free slot");
    let predicted = (u64::from(arena.slots[head as usize].generation) << 32) | u64::from(head);

    let c = world.add_body(BodyDesc::default());
    assert_eq!(c.0, predicted);
    assert_ne!(c, a, "a reused slot gets a new generation");
}

/// Once the box presses on the ground, the tracker carries the pair into the
/// next frame. (It starts exactly touching, which the 3D narrowphase doesn't
/// count, so let gravity close the gap first.)
#[test]
fn tracker_dump_holds_touching_pair() {
    let (mut world, _, gc, bc) = resting_box();
    for _ in 0..5 {
        world.step();
    }
    let tracker = world.oracle_dump().backend.tracker;
    let key = if gc.0 <= bc.0 {
        (gc.0, bc.0)
    } else {
        (bc.0, gc.0)
    };
    assert!(tracker.prev.contains(&key), "{tracker:?}");
}

// ---------------------------------------------------------------------------
// Broadphase pair sets
// ---------------------------------------------------------------------------

fn xorshift(state: &mut u64) -> f64 {
    *state ^= *state << 13;
    *state ^= *state >> 7;
    *state ^= *state << 17;
    (*state >> 11) as f64 / (1u64 << 53) as f64
}

fn random_boxes_3d(n: usize, seed: u64) -> Vec<([f64; 3], [f64; 3])> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            let c = [
                xorshift(&mut s) * 20.0,
                xorshift(&mut s) * 20.0,
                xorshift(&mut s) * 20.0,
            ];
            let h = 0.1 + xorshift(&mut s) * 1.5;
            (
                [c[0] - h, c[1] - h, c[2] - h],
                [c[0] + h, c[1] + h, c[2] + h],
            )
        })
        .collect()
}

fn brute_pairs<const D: usize>(boxes: &[([f64; D], [f64; D])]) -> Vec<(u32, u32)> {
    let mut out = Vec::new();
    for i in 0..boxes.len() {
        for j in i + 1..boxes.len() {
            let (a, b) = (&boxes[i], &boxes[j]);
            if (0..D).all(|k| a.0[k] <= b.1[k] && a.1[k] >= b.0[k]) {
                out.push((i as u32, j as u32));
            }
        }
    }
    out
}

fn covers(candidates: &[(u32, u32)], exact: &[(u32, u32)]) -> bool {
    exact.iter().all(|p| candidates.binary_search(p).is_ok())
}

/// Both structures report candidate pairs: the tree from its fattened leaves,
/// the hash from shared cells. Each covers every overlapping pair, and the
/// answer is deterministic.
#[test]
fn broadphase_pairs_against_brute_force() {
    let boxes3 = random_boxes_3d(300, 0x9E37_79B9_7F4A_7C15);
    let expected3 = brute_pairs(&boxes3);
    assert!(!expected3.is_empty());
    let tree3 = aabb_tree_pairs_3d(&boxes3);
    assert!(covers(&tree3, &expected3));
    assert_eq!(tree3, aabb_tree_pairs_3d(&boxes3));
    let hash3 = spatial_hash_pairs_3d(2.0, &boxes3);
    assert!(covers(&hash3, &expected3));

    let boxes2: Vec<([f64; 2], [f64; 2])> = boxes3
        .iter()
        .map(|(lo, hi)| ([lo[0], lo[1]], [hi[0], hi[1]]))
        .collect();
    let expected2 = brute_pairs(&boxes2);
    assert!(covers(&aabb_tree_pairs_2d(&boxes2), &expected2));
    assert!(covers(&spatial_hash_pairs_2d(2.0, &boxes2), &expected2));

    assert!(spatial_hash_auto_cell_size(&[1.0, 2.0, 3.0]) > 0.0);
}

// ---------------------------------------------------------------------------
// 2D only
// ---------------------------------------------------------------------------

/// A resting box keeps a two-point manifold whose warm-start impulses carry
/// its weight; none of this is visible through the public API.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn manifold_dump_carries_warm_start() {
    let (mut world, ..) = resting_box();
    for _ in 0..30 {
        world.step();
    }
    let manifolds = world.oracle_dump().backend.manifolds;
    assert_eq!(manifolds.len(), 1);
    let m = &manifolds[0];
    assert_eq!(m.points.len(), 2);
    assert!(m.points.iter().all(|p| p.normal_impulse > 0.0), "{m:?}");
    // Ground (collider A, added first) → box: the normal points up.
    assert!((m.normal[1] - 1.0).abs() < 1e-12, "{m:?}");
}

/// Box on box: two clipped points, normal A→B in either argument order.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn contacts_2d_box_on_box() {
    let ground = ColliderShape::Box {
        half_extents: [5.0, 0.5, 5.0],
    };
    let unit = ColliderShape::Box {
        half_extents: [0.5, 0.5, 0.5],
    };
    let up = contacts_2d(&ground, [0.0, 0.0], 0.0, &unit, [0.0, 0.95], 0.0);
    assert_eq!(up.len(), 2);
    for (n, depth, _) in &up {
        assert!((n[1] - 1.0).abs() < 1e-12 && n[0].abs() < 1e-12);
        assert!((depth - 0.05).abs() < 1e-9);
    }
    let down = contacts_2d(&unit, [0.0, 0.95], 0.0, &ground, [0.0, 0.0], 0.0);
    assert_eq!(down.len(), 2);
    assert!(down.iter().all(|(n, ..)| (n[1] + 1.0).abs() < 1e-12));

    assert!(shapes_touch_2d(
        &ground,
        [0.0, 0.0],
        0.0,
        &unit,
        [0.0, 0.95],
        0.0
    ));
    assert!(!shapes_touch_2d(
        &ground,
        [0.0, 0.0],
        0.0,
        &unit,
        [0.0, 3.0],
        0.0
    ));
}

// ---------------------------------------------------------------------------
// 3D only
// ---------------------------------------------------------------------------

/// A box that lands off-centre (along z) on a ball tips about X: an
/// orientation the Z-only `BodyState` can't show.
#[cfg(feature = "3d")]
#[test]
fn dump_exposes_full_3d_orientation() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let post = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.4],
        ..BodyDesc::default()
    });
    world.add_collider(post, collider(ColliderShape::Ball { radius: 0.5 }));
    let body = world.add_body(BodyDesc {
        position: [0.0, 1.2, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(
        body,
        collider(ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.5],
        }),
    );
    for _ in 0..60 {
        world.step();
    }
    let dump = world.oracle_dump();
    let b = dump
        .backend
        .bodies
        .iter()
        .find(|b| b.handle == body.0)
        .expect("box body in dump");
    let [x, y, z, w] = b.rotation;
    assert!(x.abs() > 1e-6, "{b:?}");
    assert!(((x * x + y * y + z * z + w * w).sqrt() - 1.0).abs() < 1e-9);
    assert!(b.angular_velocity[0].abs() > 1e-6);
    assert!(b.inv_inertia_world[0][0] > 0.0);
}

/// Two spheres overlapping along X: normal A→B, depth 0.5.
#[cfg(feature = "3d")]
#[test]
fn contact_3d_sphere_pair() {
    let ball = ColliderShape::Ball { radius: 1.0 };
    let id = [0.0, 0.0, 0.0, 1.0];
    let (n, depth, _) =
        contact_3d(&ball, [0.0; 3], id, &ball, [1.5, 0.0, 0.0], id).expect("overlap");
    assert!((n[0] - 1.0).abs() < 1e-12);
    assert!((depth - 0.5).abs() < 1e-12);
    assert!(contact_3d(&ball, [0.0; 3], id, &ball, [3.0, 0.0, 0.0], id).is_none());
}
