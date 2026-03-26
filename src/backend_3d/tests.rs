use super::*;
use crate::body::{BodyDesc, BodyHandle, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::force::Impulse;
use crate::joint::{JointDesc, JointType};
use crate::material::PhysicsMaterial;
use crate::spatial_hash::SpatialHashGrid;
use hisab::{DQuat, DVec3};

use super::narrowphase::*;
use super::raycast::*;
use super::types::*;

const EPS: f64 = 1e-6;

#[test]
fn sphere_sphere_overlap() {
    let r = sphere_sphere(DVec3::ZERO, 1.0, DVec3::new(1.5, 0.0, 0.0), 1.0);
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!((n.x - 1.0).abs() < EPS);
    assert!((d - 0.5).abs() < EPS);
}

#[test]
fn sphere_sphere_no_overlap() {
    assert!(sphere_sphere(DVec3::ZERO, 1.0, DVec3::new(5.0, 0.0, 0.0), 1.0).is_none());
}

#[test]
fn sphere_obb_overlap() {
    let r = sphere_obb(
        DVec3::new(1.8, 0.0, 0.0),
        0.5,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(1.5, 1.0, 1.0),
    );
    assert!(r.is_some());
}

#[test]
fn sphere_obb_miss() {
    assert!(
        sphere_obb(
            DVec3::new(5.0, 0.0, 0.0),
            0.5,
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(1.0, 1.0, 1.0),
        )
        .is_none()
    );
}

#[test]
fn sphere_obb_rotated() {
    // Box rotated 45° around Z — a sphere that would miss an AABB should hit the rotated box
    let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
    // Box half_extents [2, 0.5, 1], rotated 45° around Z.
    // The corner extends to sqrt(2^2 + 0.5^2) ≈ 2.06 along the diagonal.
    let r = sphere_obb(
        DVec3::new(1.5, 1.5, 0.0),
        0.5,
        DVec3::ZERO,
        rot,
        DVec3::new(2.0, 0.5, 1.0),
    );
    assert!(r.is_some());
}

#[test]
fn aabb_3d_overlap() {
    let r = aabb_aabb_3d(
        DVec3::ZERO,
        DVec3::ONE,
        DVec3::new(1.5, 0.0, 0.0),
        DVec3::ONE,
    );
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!((n.x - 1.0).abs() < EPS);
    assert!((d - 0.5).abs() < EPS);
}

#[test]
fn aabb_3d_no_overlap() {
    assert!(
        aabb_aabb_3d(
            DVec3::ZERO,
            DVec3::ONE,
            DVec3::new(5.0, 0.0, 0.0),
            DVec3::ONE,
        )
        .is_none()
    );
}

#[test]
fn ray_sphere_hit() {
    let r = ray_sphere(DVec3::ZERO, DVec3::X, DVec3::new(5.0, 0.0, 0.0), 1.0);
    assert!(r.is_some());
    let (t, _) = r.unwrap();
    assert!((t - 4.0).abs() < EPS);
}

#[test]
fn ray_sphere_miss() {
    assert!(ray_sphere(DVec3::ZERO, DVec3::Y, DVec3::new(5.0, 0.0, 0.0), 1.0).is_none());
}

#[test]
fn ray_aabb_3d_hit() {
    let r = ray_aabb_3d(
        DVec3::ZERO,
        DVec3::X,
        DVec3::new(4.0, -1.0, -1.0),
        DVec3::new(6.0, 1.0, 1.0),
    );
    assert!(r.is_some());
    let (t, _) = r.unwrap();
    assert!((t - 4.0).abs() < EPS);
}

#[test]
fn quaternion_identity_rotation() {
    let q = DQuat::IDENTITY;
    let v = DVec3::X;
    let result = q * v;
    assert!((result.x - 1.0).abs() < EPS);
    assert!(result.y.abs() < EPS);
    assert!(result.z.abs() < EPS);
}

#[test]
fn quaternion_z_rotation() {
    let q = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let v = DVec3::X;
    let result = q * v;
    assert!(result.x.abs() < EPS);
    assert!((result.y - 1.0).abs() < EPS);
    assert!(result.z.abs() < EPS);
}

#[test]
fn gravity_moves_body_3d() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
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

    assert!(
        state.bodies.get(body_ah(bh)).unwrap().position.y < 10.0,
        "body should fall"
    );
}

#[test]
fn sphere_collision_3d() {
    let mut state = PhysicsState3d::new();

    let floor = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        floor,
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [50.0, 0.5, 50.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let ball = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 2.0, 0.0],
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

    let mut found_event = false;
    for _ in 0..120 {
        let events = state.step(
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
        if !events.is_empty() {
            found_event = true;
            break;
        }
    }
    assert!(found_event, "should generate collision events");
}

#[test]
fn raycast_3d() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [5.0, 0.0, 0.0],
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

    let hit = state.raycast([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 100.0);
    assert!(hit.is_some());
    let hit = hit.unwrap();
    assert!((hit.distance - 4.0).abs() < 0.1);
}

#[test]
fn body_count_3d() {
    let mut state = PhysicsState3d::new();
    assert_eq!(state.body_count(), 0);
    state.add_body(&BodyDesc::default());
    assert_eq!(state.body_count(), 1);
    state.remove_body(BodyHandle(0));
    assert_eq!(state.body_count(), 0);
}

#[test]
fn sphere_mass_3d() {
    let c = Collider3d::from_desc(
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
    let expected = (4.0 / 3.0) * std::f64::consts::PI;
    assert!((m - expected).abs() < EPS);
}

#[test]
fn box_mass_3d() {
    let c = Collider3d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [1.0, 1.0, 1.0],
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
    assert!((c.compute_mass() - 8.0).abs() < EPS);
}

#[test]
fn multiple_colliders_accumulate_mass_3d() {
    let mut state = PhysicsState3d::new();
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
    let mass_first = state.bodies.get(body_ah(bh)).unwrap().mass;

    state.add_collider(
        bh,
        &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [2.0, 0.0, 0.0],
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
    assert!((state.bodies.get(body_ah(bh)).unwrap().mass - 2.0 * mass_first).abs() < EPS);
}

#[test]
fn capsule_sphere_3d_overlap() {
    let r = capsule_sphere_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        1.0,
        0.5,
        DVec3::new(0.8, 0.0, 0.0),
        0.5,
    );
    assert!(r.is_some());
}

#[test]
fn capsule_sphere_3d_miss() {
    assert!(
        capsule_sphere_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            1.0,
            0.5,
            DVec3::new(5.0, 0.0, 0.0),
            0.5
        )
        .is_none()
    );
}

#[test]
fn impulse_changes_velocity_3d() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
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

    state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
    assert!(state.bodies.get(body_ah(bh)).unwrap().linear_velocity.x > 0.0);
}

#[test]
fn remove_cleans_collision_pairs_3d() {
    let mut state = PhysicsState3d::new();

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
    assert!(!state.prev_collision_pairs.is_empty());

    state.remove_body(b);
    assert!(state.prev_collision_pairs.is_empty());
}

#[test]
fn fixed_joint_3d() {
    let mut state = PhysicsState3d::new();
    let a = BodyHandle(0);
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 5.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 3.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
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
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Fixed,
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
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
    // Joint should prevent body from falling far
    assert!(state.bodies.get(body_ah(b)).unwrap().position.y > 2.0);
}

#[test]
fn spatial_hash_3d_finds_pair() {
    let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(2.0);
    grid.insert_3d(ColliderHandle(0), [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    grid.insert_3d(ColliderHandle(1), [0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
    let pairs = grid.query_pairs();
    assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
}

#[test]
fn spatial_hash_3d_no_false_pair() {
    let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(1.0);
    grid.insert_3d(ColliderHandle(0), [0.0, 0.0, 0.0], [0.5, 0.5, 0.5]);
    grid.insert_3d(ColliderHandle(1), [10.0, 10.0, 10.0], [10.5, 10.5, 10.5]);
    assert!(grid.query_pairs().is_empty());
}

// -----------------------------------------------------------------------
// set_body_state / set_body_type tests
// -----------------------------------------------------------------------

#[test]
fn set_body_state_teleports() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc::default());
    state.add_collider(
        bh,
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

    let new_state = crate::body::BodyState {
        handle: bh,
        body_type: BodyType::Dynamic,
        position: [10.0, 20.0, 30.0],
        rotation: 1.0,
        linear_velocity: [1.0, 2.0, 3.0],
        angular_velocity: 0.5,
        is_sleeping: false,
    };
    state.set_body_state(bh, &new_state).unwrap();

    let rb = &state.bodies.get(body_ah(bh)).unwrap();
    assert!((rb.position.x - 10.0).abs() < EPS);
    assert!((rb.position.y - 20.0).abs() < EPS);
    assert!((rb.position.z - 30.0).abs() < EPS);
    assert!(!rb.is_sleeping);
}

#[test]
fn set_body_state_not_found() {
    let mut state = PhysicsState3d::new();
    let result = state.set_body_state(
        BodyHandle(999),
        &crate::body::BodyState {
            handle: BodyHandle(999),
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            rotation: 0.0,
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: 0.0,
            is_sleeping: false,
        },
    );
    assert!(result.is_err());
}

#[test]
fn set_body_type_dynamic_to_static() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        linear_velocity: [5.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
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

    state.set_body_type(bh, BodyType::Static).unwrap();
    let rb = &state.bodies.get(body_ah(bh)).unwrap();
    assert_eq!(rb.body_type, BodyType::Static);
    assert_eq!(rb.inv_mass, 0.0);
    assert_eq!(rb.linear_velocity, DVec3::ZERO);
}

#[test]
fn set_body_type_static_to_dynamic() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
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

    let mass_before = state.bodies.get(body_ah(bh)).unwrap().mass;
    assert!(mass_before > 0.0);

    // Switch to static and back
    state.set_body_type(bh, BodyType::Static).unwrap();
    state.set_body_type(bh, BodyType::Dynamic).unwrap();
    let rb = &state.bodies.get(body_ah(bh)).unwrap();
    assert!(rb.inv_mass > 0.0);
    assert_eq!(rb.body_type, BodyType::Dynamic);
}

// -----------------------------------------------------------------------
// OBB-OBB tests
// -----------------------------------------------------------------------

#[test]
fn obb_obb_aligned_overlap() {
    // Two axis-aligned boxes overlapping
    let r = obb_obb_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ONE,
        DVec3::new(1.5, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ONE,
    );
    assert!(r.is_some());
    let (n, d, _) = r.unwrap();
    assert!((n.x - 1.0).abs() < EPS);
    assert!((d - 0.5).abs() < EPS);
}

#[test]
fn obb_obb_no_overlap() {
    assert!(
        obb_obb_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::ONE,
            DVec3::new(5.0, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ONE,
        )
        .is_none()
    );
}

#[test]
fn obb_obb_rotated_overlap() {
    // One box rotated 45° around Z
    let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
    let r = obb_obb_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ONE,
        DVec3::new(1.5, 0.0, 0.0),
        rot,
        DVec3::ONE,
    );
    assert!(r.is_some());
}

#[test]
fn obb_obb_rotated_separated() {
    // One box rotated, far enough apart to not overlap
    let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
    assert!(
        obb_obb_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(0.5, 0.5, 0.5),
            DVec3::new(3.0, 0.0, 0.0),
            rot,
            DVec3::new(0.5, 0.5, 0.5),
        )
        .is_none()
    );
}

// -----------------------------------------------------------------------
// Capsule-Capsule tests
// -----------------------------------------------------------------------

#[test]
fn capsule_capsule_overlap() {
    let r = capsule_capsule_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        1.0,
        0.5,
        DVec3::new(0.8, 0.0, 0.0),
        DQuat::IDENTITY,
        1.0,
        0.5,
    );
    assert!(r.is_some());
}

#[test]
fn capsule_capsule_miss() {
    assert!(
        capsule_capsule_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            1.0,
            0.5,
            DVec3::new(5.0, 0.0, 0.0),
            DQuat::IDENTITY,
            1.0,
            0.5,
        )
        .is_none()
    );
}

#[test]
fn capsule_capsule_crossed() {
    // Two capsules crossing at right angles
    let rot_x = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let r = capsule_capsule_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        2.0,
        0.3,
        DVec3::new(0.0, 0.0, 0.0),
        rot_x,
        2.0,
        0.3,
    );
    assert!(r.is_some());
}

// -----------------------------------------------------------------------
// Capsule-Box tests
// -----------------------------------------------------------------------

#[test]
fn capsule_box_overlap() {
    let r = capsule_box_3d(
        DVec3::new(0.0, 0.0, 0.0),
        DQuat::IDENTITY,
        1.0,
        0.5,
        DVec3::new(1.0, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ONE,
    );
    assert!(r.is_some());
}

#[test]
fn capsule_box_miss() {
    assert!(
        capsule_box_3d(
            DVec3::new(0.0, 0.0, 0.0),
            DQuat::IDENTITY,
            1.0,
            0.5,
            DVec3::new(5.0, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ONE,
        )
        .is_none()
    );
}

// -----------------------------------------------------------------------
// Segment tests
// -----------------------------------------------------------------------

#[test]
fn segment_sphere_overlap() {
    let r = segment_sphere_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(-2.0, 0.0, 0.0),
        DVec3::new(2.0, 0.0, 0.0),
        DVec3::new(0.0, 0.3, 0.0),
        0.5,
    );
    assert!(r.is_some());
}

#[test]
fn segment_sphere_miss() {
    assert!(
        segment_sphere_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(-2.0, 0.0, 0.0),
            DVec3::new(2.0, 0.0, 0.0),
            DVec3::new(0.0, 5.0, 0.0),
            0.5,
        )
        .is_none()
    );
}

#[test]
fn segment_box_overlap() {
    // Segment passing through the center of a box
    let r = segment_box_3d(
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(-0.5, 0.0, 0.0),
        DVec3::new(0.5, 0.0, 0.0),
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ONE,
    );
    assert!(r.is_some());
}

#[test]
fn segment_box_miss() {
    assert!(
        segment_box_3d(
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(-0.5, 0.0, 0.0),
            DVec3::new(0.5, 0.0, 0.0),
            DVec3::new(5.0, 5.0, 5.0),
            DQuat::IDENTITY,
            DVec3::ONE,
        )
        .is_none()
    );
}

// -----------------------------------------------------------------------
// ConvexHull vs Sphere tests
// -----------------------------------------------------------------------

#[test]
fn convex_hull_sphere_overlap() {
    // Triangle hull near sphere
    let points = vec![[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]];
    let r = convex_hull_sphere_3d(
        &points,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(0.0, 1.2, 0.0),
        0.5,
    );
    assert!(r.is_some());
}

#[test]
fn convex_hull_sphere_miss() {
    let points = vec![[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]];
    assert!(
        convex_hull_sphere_3d(
            &points,
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(0.0, 5.0, 0.0),
            0.5,
        )
        .is_none()
    );
}

// -----------------------------------------------------------------------
// Closest points between segments
// -----------------------------------------------------------------------

#[test]
fn closest_points_parallel_segments() {
    let (p1, p2) = closest_points_segments_3d(
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, 1.0, 0.0),
        DVec3::new(1.0, 1.0, 0.0),
    );
    // Closest points should be on the same x coordinate
    assert!((p1.y).abs() < EPS);
    assert!((p2.y - 1.0).abs() < EPS);
}

#[test]
fn closest_points_crossing_segments() {
    let (p1, p2) = closest_points_segments_3d(
        DVec3::new(-1.0, 0.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        DVec3::new(0.0, -1.0, 1.0),
        DVec3::new(0.0, 1.0, 1.0),
    );
    // p1 should be at origin (0,0,0) and p2 at (0,0,1)
    assert!((p1 - DVec3::new(0.0, 0.0, 0.0)).length() < EPS);
    assert!((p2 - DVec3::new(0.0, 0.0, 1.0)).length() < EPS);
}

#[test]
fn gyroscopic_torque_precession() {
    // A spinning body with asymmetric inertia should exhibit precession
    // due to the gyroscopic torque term ω × (I·ω).
    use hisab::DMat3;

    let mut rb = RigidBody3d::from_desc(
        BodyHandle(100),
        &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        },
    );
    // Asymmetric inertia: Ix=1, Iy=2, Iz=3
    rb.mass = 1.0;
    rb.inv_mass = 1.0;
    rb.inertia = DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0));
    rb.inv_inertia = DMat3::from_diagonal(DVec3::new(1.0, 0.5, 1.0 / 3.0));
    // Spin around X at 10 rad/s — gyroscopic torque should induce cross-axis rotation
    rb.angular_velocity = DVec3::new(10.0, 0.0, 0.0);

    // Test with a spin that has components on multiple axes for gyroscopic coupling:
    rb.angular_velocity = DVec3::new(10.0, 5.0, 0.0);
    let before_z = rb.angular_velocity.z;
    rb.integrate_velocities(DVec3::ZERO, 1.0 / 60.0, 100.0);

    // With asymmetric inertia and multi-axis spin, gyroscopic torque
    // should produce a non-zero z-component change
    // ω × (I·ω) = (10,5,0) × (10,10,0) = (0,0,100-50) = (0,0,50)
    // Effect: Δω_z = inv_Iz * (-50) * dt = (1/3) * (-50) * (1/60) ≈ -0.278
    assert!(
        (rb.angular_velocity.z - before_z).abs() > 0.01,
        "gyroscopic torque should cause cross-axis angular velocity change, got dz={}",
        rb.angular_velocity.z - before_z
    );
}

#[test]
fn gyroscopic_torque_symmetric_no_precession() {
    // A symmetric body (uniform sphere) spinning around one axis should have
    // zero gyroscopic torque: ω × (I·ω) = 0 when I is scalar multiple of identity.
    use hisab::DMat3;

    let mut rb = RigidBody3d::from_desc(
        BodyHandle(101),
        &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        },
    );
    rb.mass = 1.0;
    rb.inv_mass = 1.0;
    rb.inertia = DMat3::from_diagonal(DVec3::splat(2.0));
    rb.inv_inertia = DMat3::from_diagonal(DVec3::splat(0.5));
    rb.angular_velocity = DVec3::new(10.0, 0.0, 0.0);

    let before = rb.angular_velocity;
    rb.integrate_velocities(DVec3::ZERO, 1.0 / 60.0, 100.0);

    // Symmetric body: gyroscopic torque = ω × (I·ω) = 10x × 20x = 0
    // Only damping should apply
    let diff = (rb.angular_velocity - before).length();
    // Should be very small (only damping, which is 0 by default)
    assert!(
        diff < 0.01,
        "symmetric body should have negligible gyroscopic effect, got diff={}",
        diff
    );
}
