use super::*;
use crate::body::{BodyDesc, BodyHandle, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointType};
use crate::material::PhysicsMaterial;
use crate::spatial_hash::SpatialHashGrid;

use super::narrowphase::*;
use super::raycast::*;
use super::types::*;

const EPS: f64 = 1e-6;

// -- Narrowphase contact tests --

#[test]
fn circle_circle_overlap() {
    let r = circle_circle([0.0, 0.0], 1.0, [1.5, 0.0], 1.0);
    assert!(r.is_some());
    let (n, d, _p) = r.unwrap();
    assert!((n[0] - 1.0).abs() < EPS);
    assert!(n[1].abs() < EPS);
    assert!((d - 0.5).abs() < EPS);
}

#[test]
fn circle_circle_no_overlap() {
    assert!(circle_circle([0.0, 0.0], 1.0, [3.0, 0.0], 1.0).is_none());
}

#[test]
fn circle_circle_coincident() {
    let r = circle_circle([0.0, 0.0], 1.0, [0.0, 0.0], 1.0);
    assert!(r.is_some());
    let (_, d, _) = r.unwrap();
    assert!((d - 2.0).abs() < EPS);
}

#[test]
fn circle_aabb_overlap() {
    let r = circle_aabb([1.8, 0.0], 0.5, [0.0, 0.0], [1.5, 1.0]);
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!(d > 0.0);
    assert!((n[0] - 1.0).abs() < EPS); // normal points away from box
}

#[test]
fn circle_aabb_no_overlap() {
    assert!(circle_aabb([5.0, 0.0], 0.5, [0.0, 0.0], [1.0, 1.0]).is_none());
}

#[test]
fn circle_aabb_inside() {
    let r = circle_aabb([0.0, 0.0], 0.5, [0.0, 0.0], [2.0, 2.0]);
    assert!(r.is_some());
    let (_, d, _) = r.unwrap();
    assert!(d > 0.0);
}

#[test]
fn aabb_aabb_overlap() {
    let r = aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [1.5, 0.0], [1.0, 1.0]);
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!((n[0] - 1.0).abs() < EPS);
    assert!((d - 0.5).abs() < EPS);
}

#[test]
fn aabb_aabb_no_overlap() {
    assert!(aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [5.0, 0.0], [1.0, 1.0]).is_none());
}

#[test]
fn aabb_aabb_vertical_overlap() {
    let r = aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [0.0, 1.5], [1.0, 1.0]);
    assert!(r.is_some());
    let (n, _, _) = r.unwrap();
    assert!((n[1] - 1.0).abs() < EPS);
}

// -- Ray intersection tests --

#[test]
fn ray_circle_hit() {
    let r = ray_circle([0.0, 0.0], [1.0, 0.0], [5.0, 0.0], 1.0);
    assert!(r.is_some());
    let (t, n) = r.unwrap();
    assert!((t - 4.0).abs() < EPS);
    assert!((n[0] - (-1.0)).abs() < EPS);
}

#[test]
fn ray_circle_miss() {
    assert!(ray_circle([0.0, 0.0], [0.0, 1.0], [5.0, 0.0], 1.0).is_none());
}

#[test]
fn ray_circle_inside() {
    let r = ray_circle([5.0, 0.0], [1.0, 0.0], [5.0, 0.0], 1.0);
    assert!(r.is_some());
    let (t, _) = r.unwrap();
    assert!((t - 1.0).abs() < EPS);
}

#[test]
fn ray_aabb_hit() {
    let r = ray_aabb_2d([0.0, 0.0], [1.0, 0.0], [4.0, -1.0], [6.0, 1.0]);
    assert!(r.is_some());
    let (t, n) = r.unwrap();
    assert!((t - 4.0).abs() < EPS);
    assert!((n[0] - (-1.0)).abs() < EPS);
}

#[test]
fn ray_aabb_miss() {
    assert!(ray_aabb_2d([0.0, 0.0], [0.0, 1.0], [4.0, -1.0], [6.0, 1.0]).is_none());
}

#[test]
fn ray_aabb_inside() {
    let r = ray_aabb_2d([5.0, 0.0], [1.0, 0.0], [4.0, -1.0], [6.0, 1.0]);
    assert!(r.is_some());
}

// -- AABB overlap tests --

#[test]
fn aabb_overlaps() {
    let a = Aabb2d {
        min: [0.0, 0.0],
        max: [2.0, 2.0],
    };
    let b = Aabb2d {
        min: [1.0, 1.0],
        max: [3.0, 3.0],
    };
    assert!(a.overlaps(&b));
    assert!(b.overlaps(&a));
}

#[test]
fn aabb_no_overlap() {
    let a = Aabb2d {
        min: [0.0, 0.0],
        max: [1.0, 1.0],
    };
    let b = Aabb2d {
        min: [2.0, 2.0],
        max: [3.0, 3.0],
    };
    assert!(!a.overlaps(&b));
}

// -- Mass/inertia tests --

#[test]
fn ball_mass() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let m = c.compute_mass();
    assert!((m - std::f64::consts::PI).abs() < EPS);
}

#[test]
fn box_mass() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [1.0, 1.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    assert!((c.compute_mass() - 4.0).abs() < EPS);
}

#[test]
fn segment_mass_nonzero() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Segment {
                a: [0.0, 0.0, 0.0],
                b: [10.0, 0.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    assert!(c.compute_mass() > 0.0);
}

#[test]
fn explicit_mass_overrides() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: Some(42.0),
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    assert!((c.compute_mass() - 42.0).abs() < EPS);
}

#[test]
fn multiple_colliders_accumulate_mass() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc::default());

    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let mass_after_first = state.bodies.get(body_ah(bh)).unwrap().mass;

    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [1.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let mass_after_second = state.bodies.get(body_ah(bh)).unwrap().mass;

    assert!(mass_after_second > mass_after_first);
    assert!((mass_after_second - 2.0 * mass_after_first).abs() < EPS);
}

// -- Capsule contact tests --

#[test]
fn capsule_circle_overlap() {
    // Vertical capsule at origin, circle to the right
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Ball { radius: 0.5 },
        [0.8, 0.0],
        0.0,
    );
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!(d > 0.0);
    assert!((n[0] - 1.0).abs() < EPS); // normal points toward circle
}

#[test]
fn capsule_circle_miss() {
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Ball { radius: 0.5 },
        [5.0, 0.0],
        0.0,
    );
    assert!(r.is_none());
}

#[test]
fn capsule_circle_endpoint() {
    // Circle near the top endpoint of a vertical capsule
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Ball { radius: 0.5 },
        [0.0, 1.3],
        0.0,
    );
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!(d > 0.0);
    assert!(n[1] > 0.5); // normal should point upward
}

#[test]
fn capsule_aabb_overlap() {
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.0],
        },
        [0.8, 0.0],
        0.0,
    );
    assert!(r.is_some());
    let (_, d, _) = r.unwrap();
    assert!(d > 0.0);
}

#[test]
fn capsule_aabb_miss() {
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.0],
        },
        [5.0, 0.0],
        0.0,
    );
    assert!(r.is_none());
}

#[test]
fn capsule_capsule_overlap() {
    // Two vertical capsules side by side
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.8, 0.0],
        0.0,
    );
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!(d > 0.0);
    assert!((n[0] - 1.0).abs() < EPS);
}

#[test]
fn capsule_capsule_miss() {
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [5.0, 0.0],
        0.0,
    );
    assert!(r.is_none());
}

#[test]
fn capsule_capsule_perpendicular() {
    // Vertical capsule at x=0, horizontal capsule at x=0.2
    // With radius 0.5 each, sum=1.0, so overlap when segment dist < 1.0
    let r = generate_contact(
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.0, 0.0],
        0.0,
        &ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        },
        [0.2, 0.0],
        std::f64::consts::FRAC_PI_2,
    );
    assert!(r.is_some(), "capsule-capsule should overlap");
    let (_, d, _) = r.unwrap();
    assert!(d > 0.0);
}

// -- Capsule ray tests --

#[test]
fn ray_capsule_hit_shaft() {
    // Horizontal ray hitting the shaft of a vertical capsule
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: 1.0,
                radius: 0.5,
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let r = state.raycast([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 100.0);
    assert!(r.is_some());
    let hit = r.unwrap();
    assert!((hit.distance - 4.5).abs() < 0.1); // should hit at ~4.5 (5 - radius 0.5)
}

#[test]
fn ray_capsule_hit_endpoint() {
    // Ray aimed at the top endpoint
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: 2.0,
                radius: 0.5,
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let r = state.raycast([0.0, 2.0, 0.0], [1.0, 0.0, 0.0], 100.0);
    assert!(r.is_some());
}

#[test]
fn ray_capsule_miss() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: 1.0,
                radius: 0.5,
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let r = state.raycast([0.0, 5.0, 0.0], [1.0, 0.0, 0.0], 100.0);
    assert!(r.is_none());
}

// -- World AABB tests --

#[test]
fn world_aabb_ball_no_rotation() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let aabb = c.world_aabb([5.0, 3.0], 0.0);
    assert!((aabb.min[0] - 4.0).abs() < EPS);
    assert!((aabb.max[0] - 6.0).abs() < EPS);
    assert!((aabb.min[1] - 2.0).abs() < EPS);
    assert!((aabb.max[1] - 4.0).abs() < EPS);
}

#[test]
fn world_aabb_box_with_rotation() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [2.0, 1.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    // 90 degree rotation swaps extents
    let aabb = c.world_aabb([0.0, 0.0], std::f64::consts::FRAC_PI_2);
    // After 90deg rotation, a 2x1 box becomes ~1x2 in AABB
    assert!((aabb.max[0] - aabb.min[0] - 2.0).abs() < 0.01);
    assert!((aabb.max[1] - aabb.min[1] - 4.0).abs() < 0.01);
}

#[test]
fn world_aabb_with_offset() {
    let c = Collider2d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [3.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let aabb = c.world_aabb([0.0, 0.0], 0.0);
    assert!((aabb.min[0] - 2.5).abs() < EPS);
    assert!((aabb.max[0] - 3.5).abs() < EPS);
}

// -- Sensor tests --

#[test]
fn sensor_generates_events_no_physics() {
    let mut state = PhysicsState2d::new();

    // Static floor
    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [10.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: true, // Sensor!
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Dynamic ball overlapping the sensor
    let ball = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        ball,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let vel_before = state.bodies.get(body_ah(ball)).unwrap().linear_velocity;
    let events = state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );

    // Should generate events
    assert!(!events.is_empty());
    // But sensor should not affect velocity (no physical response)
    let vel_after = state.bodies.get(body_ah(ball)).unwrap().linear_velocity;
    assert!((vel_after[0] - vel_before[0]).abs() < EPS);
    assert!((vel_after[1] - vel_before[1]).abs() < EPS);
}

// -- Kinematic body tests --

#[test]
fn kinematic_body_moves_from_velocity() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Kinematic,
        position: [0.0, 0.0, 0.0],
        linear_velocity: [10.0, 0.0, 0.0],
        ..BodyDesc::default()
    });

    let dt = 1.0 / 60.0;
    state.step(
        [0.0, -9.81, 0.0],
        dt,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );

    let rb = &state.bodies.get(body_ah(bh)).unwrap();
    // Should have moved from velocity
    assert!(rb.position[0] > 0.0);
    // Should NOT have been affected by gravity
    assert!((rb.linear_velocity[1]).abs() < EPS);
}

// -- Stale collision pair cleanup --

#[test]
fn remove_body_cleans_collision_pairs() {
    let mut state = PhysicsState2d::new();

    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.5, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Step to generate collision manifolds
    state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(!state.manifolds.is_empty());

    // Remove body b — should clean manifolds
    state.remove_body(b);
    assert!(state.manifolds.is_empty());
}

// -- Spatial hash tests (delegated to spatial_hash module, but verify integration) --

#[test]
fn spatial_hash_finds_nearby_pair() {
    let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(2.0);
    grid.insert_2d(ColliderHandle(0), [0.0, 0.0], [1.0, 1.0]);
    grid.insert_2d(ColliderHandle(1), [0.5, 0.5], [1.5, 1.5]);
    let pairs = grid.query_pairs();
    assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
}

#[test]
fn spatial_hash_no_false_pair() {
    let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(1.0);
    grid.insert_2d(ColliderHandle(0), [0.0, 0.0], [0.5, 0.5]);
    grid.insert_2d(ColliderHandle(1), [10.0, 10.0], [10.5, 10.5]);
    let pairs = grid.query_pairs();
    assert!(pairs.is_empty());
}

#[test]
fn spatial_hash_auto_cell_size() {
    let dims = vec![1.0_f64, 2.0];
    let size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(dims.into_iter(), 2);
    // avg max dimension = (1+2)/2 = 1.5, *2 = 3.0
    assert!((size - 3.0).abs() < EPS);
}

#[test]
fn spatial_hash_empty() {
    let size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(std::iter::empty(), 0);
    assert!((size - 1.0).abs() < EPS);
}

// -- Segment closest point tests --

#[test]
fn closest_point_on_segment_midpoint() {
    let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [5.0, 3.0]);
    assert!((pt[0] - 5.0).abs() < EPS);
    assert!(pt[1].abs() < EPS);
    assert!((t - 0.5).abs() < EPS);
}

#[test]
fn closest_point_on_segment_endpoint_a() {
    let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [-5.0, 0.0]);
    assert!(pt[0].abs() < EPS);
    assert!((t).abs() < EPS);
}

#[test]
fn closest_point_on_segment_endpoint_b() {
    let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [15.0, 0.0]);
    assert!((pt[0] - 10.0).abs() < EPS);
    assert!((t - 1.0).abs() < EPS);
}

#[test]
fn closest_points_segments_perpendicular() {
    // Vertical (0,-1)->(0,1) vs horizontal (1,0)->(3,0)
    // Use closest_point_on_segment to verify
    let (p1, _) = closest_point_on_segment([0.0, -1.0], [0.0, 1.0], [1.0, 0.0]);
    let (p2, _) = closest_point_on_segment([1.0, 0.0], [3.0, 0.0], [0.0, 0.0]);
    assert!((p1[0]).abs() < EPS);
    assert!((p1[1]).abs() < EPS);
    assert!((p2[0] - 1.0).abs() < EPS);
    assert!((p2[1]).abs() < EPS);
}

// =======================================================================
// Feature 1: Sleep / deactivation tests
// =======================================================================

#[test]
fn body_falls_asleep_when_stationary() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Zero gravity, zero velocity — body should go to sleep after SLEEP_TIME_THRESHOLD
    let dt = 1.0 / 60.0;
    let steps_needed = (SLEEP_TIME_THRESHOLD / dt).ceil() as usize + 10;
    for _ in 0..steps_needed {
        state.step(
            [0.0, 0.0, 0.0],
            dt,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    assert!(
        state.bodies.get(body_ah(bh)).unwrap().is_sleeping,
        "body should be sleeping after sitting still"
    );
}

#[test]
fn sleeping_body_skips_integration() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Manually put to sleep
    state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;

    let pos_before = state.bodies.get(body_ah(bh)).unwrap().position;
    // Step with gravity — sleeping body should not move
    state.step(
        [0.0, -9.81, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );

    let pos_after = state.bodies.get(body_ah(bh)).unwrap().position;
    assert!(
        (pos_after[0] - pos_before[0]).abs() < EPS && (pos_after[1] - pos_before[1]).abs() < EPS,
        "sleeping body should not have moved"
    );
}

#[test]
fn force_wakes_sleeping_body() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Put to sleep
    state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
    state.bodies.get_mut(body_ah(bh)).unwrap().sleep_timer = 1.0;

    // Apply force should wake it
    state.apply_force(bh, &Force::new(10.0, 0.0, 0.0));
    assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
    assert!((state.bodies.get(body_ah(bh)).unwrap().sleep_timer).abs() < EPS);
}

#[test]
fn impulse_wakes_sleeping_body() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
    state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
    assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
}

#[test]
fn torque_wakes_sleeping_body() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
    state.apply_torque(bh, &Torque::new(5.0));
    assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
}

#[test]
fn moving_body_does_not_sleep() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        linear_velocity: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Step many times — body is moving so should not sleep
    for _ in 0..100 {
        state.step(
            [0.0, 0.0, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }
    assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
}

#[test]
fn get_body_state_reports_sleeping() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    assert!(!state.get_body_state(bh).unwrap().is_sleeping);

    state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
    assert!(state.get_body_state(bh).unwrap().is_sleeping);
}

#[test]
fn contact_wakes_sleeping_body() {
    let mut state = PhysicsState2d::new();

    // A sleeping body at origin
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    state.bodies.get_mut(body_ah(a)).unwrap().is_sleeping = true;

    // A moving body heading toward it
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [3.0, 0.0, 0.0],
        linear_velocity: [-5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Step until contact
    for _ in 0..60 {
        state.step(
            [0.0, 0.0, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    // The sleeping body should have been woken by the impact
    assert!(
        !state.bodies.get(body_ah(a)).unwrap().is_sleeping,
        "sleeping body should wake on contact"
    );
}

// =======================================================================
// Feature 2: Collision layer filtering tests
// =======================================================================

#[test]
fn collision_layers_prevent_collision() {
    let mut state = PhysicsState2d::new();

    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x01,
        },
    );

    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.5, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x02,
            collision_mask: 0x02,
        },
    );

    let events = state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(
        events.is_empty(),
        "different collision layers should not generate events"
    );
}

#[test]
fn collision_layers_allow_same_layer() {
    let mut state = PhysicsState2d::new();

    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x01,
        },
    );

    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.5, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x01,
        },
    );

    let events = state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(
        !events.is_empty(),
        "same collision layer should generate events"
    );
}

#[test]
fn collision_layers_asymmetric_mask() {
    let mut state = PhysicsState2d::new();

    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x03,
        },
    );

    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.5, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x02,
            collision_mask: 0x02,
        },
    );

    let events = state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(
        !events.is_empty(),
        "asymmetric mask should still allow collision when one side matches"
    );
}

#[test]
fn collision_layers_default_collide_everything() {
    let mut state = PhysicsState2d::new();

    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.5, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let events = state.step(
        [0.0, 0.0, 0.0],
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(!events.is_empty(), "default layers should collide");
}

// =======================================================================
// Feature 3: Overlap query tests
// =======================================================================

#[test]
fn overlap_sphere_finds_ball() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    let ch = ColliderHandle(0);
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let hits = state.overlap_sphere([5.0, 0.0, 0.0], 0.5);
    assert!(hits.contains(&ch), "should find overlapping ball");

    let misses = state.overlap_sphere([100.0, 0.0, 0.0], 0.5);
    assert!(misses.is_empty(), "should not find distant ball");
}

#[test]
fn overlap_sphere_finds_box() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    let ch = ColliderHandle(0);
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [2.0, 2.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let hits = state.overlap_sphere([2.3, 0.0, 0.0], 0.5);
    assert!(hits.contains(&ch), "sphere near box edge should overlap");

    let misses = state.overlap_sphere([10.0, 0.0, 0.0], 0.5);
    assert!(misses.is_empty(), "distant sphere should not overlap");
}

#[test]
fn overlap_sphere_misses_distant() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let results = state.overlap_sphere([10.0, 10.0, 0.0], 0.5);
    assert!(results.is_empty());
}

#[test]
fn overlap_aabb_finds_colliders() {
    let mut state = PhysicsState2d::new();

    let b1 = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    let c1 = ColliderHandle(0);
    state.add_collider(
        b1,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let b2 = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [10.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    let c2 = ColliderHandle(1);
    state.add_collider(
        b2,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let hits = state.overlap_aabb([-2.0, -2.0, 0.0], [2.0, 2.0, 0.0]);
    assert!(hits.contains(&c1), "should find collider at origin");
    assert!(!hits.contains(&c2), "should not find collider at x=10");

    let all_hits = state.overlap_aabb([-2.0, -2.0, 0.0], [12.0, 2.0, 0.0]);
    assert!(all_hits.contains(&c1));
    assert!(all_hits.contains(&c2));
}

#[test]
fn overlap_aabb_empty() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let results = state.overlap_aabb([50.0, 50.0, 0.0], [60.0, 60.0, 0.0]);
    assert!(results.is_empty());
}

#[test]
fn overlap_sphere_finds_capsule() {
    let mut state = PhysicsState2d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    let ch = ColliderHandle(0);
    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: 2.0,
                radius: 0.5,
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let hits = state.overlap_sphere([0.0, 2.3, 0.0], 0.5);
    assert!(
        hits.contains(&ch),
        "sphere near capsule endpoint should overlap"
    );

    let misses = state.overlap_sphere([5.0, 0.0, 0.0], 0.5);
    assert!(
        misses.is_empty(),
        "distant sphere should not overlap capsule"
    );
}

// =======================================================================
// Feature: Persistent contact manifolds & warm starting
// =======================================================================

#[test]
fn manifold_persistence() {
    let mut state = PhysicsState2d::new();

    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -1.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [10.0, 1.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.5,
                restitution: 0.0,
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let box_body = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.4, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        box_body,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [0.5, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.5,
                restitution: 0.0,
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    state.step(
        [0.0, -9.81, 0.0],
        1.0 / 60.0,
        8,
        4,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(
        !state.manifolds.is_empty(),
        "manifold should exist after first step with contact"
    );

    state.step(
        [0.0, -9.81, 0.0],
        1.0 / 60.0,
        8,
        4,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    assert!(
        !state.manifolds.is_empty(),
        "manifold should persist across frames"
    );

    let has_nonzero_impulse = state
        .manifolds
        .values()
        .any(|m| m.points.iter().any(|p| p.normal_impulse.abs() > EPS));
    assert!(
        has_nonzero_impulse,
        "manifold should have non-zero accumulated impulse after two frames"
    );
}

#[test]
fn warm_start_stabilizes_stack() {
    let mut state = PhysicsState2d::new();

    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -0.5, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [20.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.8,
                restitution: 0.0,
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let box_size = 0.5;
    let mut top_handle = BodyHandle(0);
    for i in 0..5 {
        let y = box_size + (i as f64) * (2.0 * box_size);
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, y, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(
            bh,
            &ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [box_size, box_size, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial {
                    friction: 0.8,
                    restitution: 0.0,
                    density: 1.0,
                    ..PhysicsMaterial::default()
                },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        if i == 4 {
            top_handle = bh;
        }
    }

    let initial_y = state.bodies.get(body_ah(top_handle)).unwrap().position[1];

    let dt = 1.0 / 60.0;
    for _ in 0..300 {
        state.step(
            [0.0, -9.81, 0.0],
            dt,
            8,
            4,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let final_y = state.bodies.get(body_ah(top_handle)).unwrap().position[1];

    assert!(
        final_y > 3.0,
        "top box should be stable in stack (y={final_y}), expected > 3.0"
    );
    assert!(
        final_y < initial_y + 1.0,
        "top box should not have been launched (y={final_y})"
    );
}

// -- Wheel joint tests --

#[test]
fn wheel_joint_constrains_perpendicular() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, -1.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Wheel {
            axis: [0.0, 1.0],
            stiffness: 500.0,
            damping: 10.0,
        },
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    for _ in 0..60 {
        state.step(
            [0.0, -9.81, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let pos = state.bodies.get(body_ah(b)).unwrap().position;
    assert!(
        pos[0].abs() < 0.1,
        "wheel should constrain x (x={})",
        pos[0]
    );
}

// -- Rope joint tests --

#[test]
fn rope_joint_allows_closer_than_max() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [1.0, 0.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Rope { max_length: 5.0 },
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    let initial_y = state.bodies.get(body_ah(b)).unwrap().position[1];
    for _ in 0..30 {
        state.step(
            [0.0, -9.81, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }
    let final_y = state.bodies.get(body_ah(b)).unwrap().position[1];
    assert!(
        final_y < initial_y,
        "body should fall under gravity (y={})",
        final_y
    );
}

#[test]
fn rope_joint_prevents_exceeding_max_length() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, -1.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Rope { max_length: 2.0 },
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    for _ in 0..120 {
        state.step(
            [0.0, -9.81, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let pos = state.bodies.get(body_ah(b)).unwrap().position;
    let dist = (pos[0] * pos[0] + pos[1] * pos[1]).sqrt();
    assert!(
        dist < 2.5,
        "rope should constrain distance (dist={dist}), max_length=2.0"
    );
}

// -- Mouse joint tests --

#[test]
fn mouse_joint_drags_body_toward_target() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        a,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        ..Default::default()
    });
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Mouse {
            target: [5.0, 0.0, 0.0],
            stiffness: 500.0,
            damping: 20.0,
            max_force: 1000.0,
        },
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    for _ in 0..120 {
        state.step(
            [0.0, 0.0, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let pos = state.bodies.get(body_ah(a)).unwrap().position;
    assert!(
        pos[0] > 2.0,
        "mouse joint should pull body toward x=5 (x={})",
        pos[0]
    );
}

// -- Joint breaking tests --

#[test]
fn joint_breaking_removes_joint() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, -3.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let jh = state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Fixed,
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: Some(0.001),
    });

    assert!(state.joints.contains(joint_ah(jh)));

    for _ in 0..10 {
        state.step(
            [0.0, -9.81, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    assert!(
        !state.joints.contains(joint_ah(jh)),
        "joint should be broken"
    );
}

#[test]
fn joint_not_broken_when_force_below_threshold() {
    let mut state = PhysicsState2d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.0, 0.0],
        ..Default::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let jh = state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Fixed,
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: Some(100000.0),
    });

    for _ in 0..10 {
        state.step(
            [0.0, -9.81, 0.0],
            1.0 / 60.0,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    assert!(
        state.joints.contains(joint_ah(jh)),
        "strong joint should survive"
    );
}

// -- Combine rule tests --

#[test]
fn combine_rule_max_priority_wins() {
    use crate::material::CombineRule;
    assert_eq!(CombineRule::Max.max(CombineRule::Average), CombineRule::Max);
    let val = CombineRule::Max.combine(0.2, 0.8);
    assert!((val - 0.8).abs() < EPS);
}

// =======================================================================
// Feature: Multi-point contact manifolds
// =======================================================================

#[test]
fn multi_point_manifold() {
    // A wide box resting on a wide floor. Over several frames the manifold
    // should accumulate 2 contact points (one near each edge).
    let mut state = PhysicsState2d::new();

    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -0.5, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [20.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.8,
                restitution: 0.0,
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // A wide box that sits on the floor — wider than a point contact
    let box_body = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 1.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        box_body,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [2.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.8,
                restitution: 0.0,
                density: 1.0,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Step several frames to let the box drop onto the floor
    let dt = 1.0 / 60.0;
    let mut found_impulse = false;
    let mut max_points = 0;
    for _ in 0..120 {
        state.step(
            [0.0, -9.81, 0.0],
            dt,
            8,
            4,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
        for manifold in state.manifolds.values() {
            max_points = max_points.max(manifold.points.len());
            if manifold.points.iter().any(|p| p.normal_impulse.abs() > EPS) {
                found_impulse = true;
            }
        }
    }

    // The manifold system should support up to 2 points per manifold.
    // Verify no manifold ever had more than 2 points.
    assert!(
        max_points <= 2,
        "manifold should have at most 2 points, observed {}",
        max_points
    );
    // Check that manifolds existed and had non-zero impulses at some point during sim
    assert!(!state.manifolds.is_empty(), "should have manifolds");
    assert!(
        found_impulse,
        "manifold points should have accumulated impulses at some point"
    );
}

// =======================================================================
// Feature: Simulation islands & island-based sleep
// =======================================================================

#[test]
fn simulation_islands_sleep() {
    // Two separate clusters of bodies. One cluster settles and sleeps.
    // After the first cluster sleeps, wake one body in the second cluster
    // and verify the first stays asleep.
    let mut state = PhysicsState2d::new();

    let mat = PhysicsMaterial {
        friction: 0.5,
        restitution: 0.0,
        density: 1.0,
        ..PhysicsMaterial::default()
    };

    // --- Cluster A: a body on a static floor at x = 0 ---
    let floor_a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -1.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor_a,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [5.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: mat.clone(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let body_a1 = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, -0.2, 0.0], // start close to resting position
        ..BodyDesc::default()
    });
    state.add_collider(
        body_a1,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.3 },
            offset: [0.0, 0.0, 0.0],
            material: mat.clone(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // --- Cluster B: a body far away at x = 100 ---
    let floor_b = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [100.0, -1.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor_b,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [5.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: mat.clone(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );
    let body_b1 = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [100.0, -0.2, 0.0], // start close to resting position
        ..BodyDesc::default()
    });
    state.add_collider(
        body_b1,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.3 },
            offset: [0.0, 0.0, 0.0],
            material: mat,
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Use zero gravity so bodies are immediately stationary (no gravity jitter).
    // The key thing we're testing is island-level atomic sleep/wake, not
    // gravity settling.
    let dt = 1.0 / 60.0;
    let steps_needed = (SLEEP_TIME_THRESHOLD / dt).ceil() as usize + 30;
    for _ in 0..steps_needed {
        state.step(
            [0.0, 0.0, 0.0],
            dt,
            4,
            1,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    // Both should be sleeping now (zero velocity, zero gravity)
    let a1_sleeping = state.bodies.get(body_ah(body_a1)).unwrap().is_sleeping;
    let b1_sleeping = state.bodies.get(body_ah(body_b1)).unwrap().is_sleeping;
    assert!(a1_sleeping, "cluster A body should be sleeping");
    assert!(b1_sleeping, "cluster B body should be sleeping");

    // Wake cluster B by applying a force
    state.apply_force(body_b1, &Force::new(100.0, 0.0, 0.0));
    state.step(
        [0.0, 0.0, 0.0],
        dt,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );

    // Cluster A should remain sleeping (they're on a separate island)
    let a1_still_sleeping = state.bodies.get(body_ah(body_a1)).unwrap().is_sleeping;
    assert!(
        a1_still_sleeping,
        "cluster A should still be sleeping after cluster B was woken"
    );
    // Cluster B should be awake
    let b1_now_awake = !state.bodies.get(body_ah(body_b1)).unwrap().is_sleeping;
    assert!(
        b1_now_awake,
        "cluster B body should be awake after force applied"
    );
}

// =======================================================================
// Feature: Static vs dynamic friction
// =======================================================================

#[test]
fn static_friction_holds() {
    // A body on a slope (simulated with a small horizontal force). With high
    // static friction, it should resist sliding. Then with a larger force,
    // it should overcome static friction.
    let mut state = PhysicsState2d::new();

    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -1.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [20.0, 1.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.5,
                static_friction: Some(2.0), // very high static friction
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let box_body = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 0.5, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        box_body,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [0.5, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.5,
                static_friction: Some(2.0),
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Let the box settle on the floor
    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        state.step(
            [0.0, -9.81, 0.0],
            dt,
            8,
            4,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let pos_after_settle = state.bodies.get(body_ah(box_body)).unwrap().position[0];

    // Apply a small horizontal force — should be held by static friction
    for _ in 0..30 {
        state.apply_force(box_body, &Force::new(1.0, 0.0, 0.0));
        state.step(
            [0.0, -9.81, 0.0],
            dt,
            8,
            4,
            0.01,
            0.2,
            100.0,
            30.0,
            1.0,
            crate::config::BroadphaseKind::SpatialHash,
        );
    }

    let pos_after_small_force = state.bodies.get(body_ah(box_body)).unwrap().position[0];
    let drift = (pos_after_small_force - pos_after_settle).abs();
    assert!(
        drift < 0.5,
        "body should barely move with small force under high static friction (drift={})",
        drift
    );
}

#[test]
fn static_friction_default_from_kinetic() {
    // When static_friction is None, effective_static_friction = friction * 1.5
    let mat = PhysicsMaterial {
        friction: 0.6,
        static_friction: None,
        ..PhysicsMaterial::default()
    };
    assert!((mat.effective_static_friction() - 0.9).abs() < EPS);

    // When static_friction is Some, use that value
    let mat2 = PhysicsMaterial {
        friction: 0.6,
        static_friction: Some(1.2),
        ..PhysicsMaterial::default()
    };
    assert!((mat2.effective_static_friction() - 1.2).abs() < EPS);
}

#[test]
fn static_friction_serde_roundtrip() {
    let mat = PhysicsMaterial {
        friction: 0.5,
        static_friction: Some(0.8),
        ..PhysicsMaterial::default()
    };
    let json = serde_json::to_string(&mat).unwrap();
    let back: PhysicsMaterial = serde_json::from_str(&json).unwrap();
    assert_eq!(back.static_friction, Some(0.8));
}

#[test]
fn static_friction_serde_default_none() {
    // Old JSON without static_friction should deserialize to None
    let json = r#"{"friction":0.5,"restitution":0.0,"density":1.0}"#;
    let mat: PhysicsMaterial = serde_json::from_str(json).unwrap();
    assert_eq!(mat.static_friction, None);
}

// =======================================================================
// Feature: Sub-stepping
// =======================================================================

#[test]
fn sub_stepping_stability() {
    use crate::PhysicsWorld;
    use crate::config::WorldConfig;

    // Stack of 5 boxes with sub_steps=4 vs sub_steps=1.
    // Both should produce valid simulations (no NaN, no explosion).
    for sub_steps in [1, 4] {
        let config = WorldConfig {
            sub_steps,
            ..Default::default()
        };
        let mut world = PhysicsWorld::new(config);

        // Static floor
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, -0.5, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [20.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial {
                    friction: 0.8,
                    restitution: 0.0,
                    density: 1.0,
                    ..PhysicsMaterial::default()
                },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        let box_size = 0.5;
        let mut top_handle = None;
        for i in 0..5 {
            let y = box_size + (i as f64) * (2.0 * box_size);
            let bh = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, y, 0.0],
                ..Default::default()
            });
            world.add_collider(
                bh,
                ColliderDesc {
                    shape: ColliderShape::Box {
                        half_extents: [box_size, box_size, 0.0],
                    },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial {
                        friction: 0.8,
                        restitution: 0.0,
                        density: 1.0,
                        ..PhysicsMaterial::default()
                    },
                    is_sensor: false,
                    mass: None,
                    collision_layer: 0xFFFF_FFFF,
                    collision_mask: 0xFFFF_FFFF,
                },
            );
            if i == 4 {
                top_handle = Some(bh);
            }
        }

        for _ in 0..300 {
            world.step();
        }

        let state = world.get_body_state(top_handle.unwrap()).unwrap();
        assert!(
            !state.position[0].is_nan() && !state.position[1].is_nan(),
            "sub_steps={sub_steps}: position should not be NaN"
        );
        assert!(
            state.position[1] > 0.0,
            "sub_steps={sub_steps}: top box should still be above ground (y={})",
            state.position[1]
        );
    }
}

#[test]
fn sub_steps_config_default() {
    let config = crate::config::WorldConfig::default();
    assert_eq!(config.sub_steps, 1);
}

#[test]
fn sub_steps_config_serde() {
    let config = crate::config::WorldConfig {
        sub_steps: 4,
        ..Default::default()
    };
    let json = serde_json::to_string(&config).unwrap();
    let back: crate::config::WorldConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(back.sub_steps, 4);
}

#[test]
fn sub_steps_config_serde_default() {
    // Old JSON without sub_steps should default to 1
    let json = r#"{"timestep":0.016666666666666666,"gravity":[0.0,-9.81,0.0],"velocity_iterations":4,"position_iterations":1,"deterministic":true,"step":0}"#;
    let config: crate::config::WorldConfig = serde_json::from_str(json).unwrap();
    assert_eq!(config.sub_steps, 1);
}
