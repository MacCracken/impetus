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
        state.step_frame(
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
        let events = state.step_frame(
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

    state.step_frame(
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
    assert!(state.contact_tracker.has_previous_contacts());

    state.remove_body(b);
    assert!(!state.contact_tracker.has_previous_contacts());
}

#[test]
fn fixed_joint_3d() {
    let mut state = PhysicsState3d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 5.0, 0.0],
        ..BodyDesc::default()
    });
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 3.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(b, &desc(ColliderShape::Ball { radius: 0.5 }));
    state.add_joint(&JointDesc {
        body_a: a,
        body_b: b,
        joint_type: JointType::Fixed,
        local_anchor_a: [0.0, -2.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    for _ in 0..600 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    // Joint holds the body in place under gravity.
    let rb = state.bodies.get(body_ah(b)).unwrap();
    assert!(
        (rb.position - DVec3::new(0.0, 3.0, 0.0)).length() < 0.01,
        "p = {}",
        rb.position
    );
    assert!(
        rb.linear_velocity.length() < 0.1,
        "v = {}",
        rb.linear_velocity
    );
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

// -----------------------------------------------------------------------
// Audit regressions
// -----------------------------------------------------------------------

fn desc(shape: ColliderShape) -> ColliderDesc {
    ColliderDesc {
        shape,
        offset: [0.0, 0.0, 0.0],
        material: PhysicsMaterial::default(),
        is_sensor: false,
        mass: None,
        collision_layer: 0xFFFF_FFFF,
        collision_mask: 0xFFFF_FFFF,
    }
}

fn frame(state: &mut PhysicsState3d, gravity: [f64; 3]) -> Vec<crate::event::CollisionEvent> {
    state.step_frame(
        gravity,
        1.0 / 60.0,
        4,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    )
}

fn cube_points(h: f64) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for x in [-h, h] {
        for y in [-h, h] {
            for z in [-h, h] {
                points.push([x, y, z]);
            }
        }
    }
    points
}

fn cube_hull(h: f64) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: cube_points(h),
    }
}

/// Assert the contact normal points from A to B in both argument orders.
fn assert_normal_a_to_b(
    sa: &ColliderShape,
    pa: DVec3,
    sb: &ColliderShape,
    pb: DVec3,
    expected: DVec3,
) {
    let q = DQuat::IDENTITY;
    let (n_ab, d_ab, _) = generate_contact_3d(sa, pa, q, sb, pb, q)
        .unwrap_or_else(|| panic!("no contact {sa:?} vs {sb:?}"));
    let (n_ba, d_ba, _) = generate_contact_3d(sb, pb, q, sa, pa, q)
        .unwrap_or_else(|| panic!("no contact {sb:?} vs {sa:?}"));
    assert!(
        (n_ab - expected).length() < 1e-6,
        "{sa:?} vs {sb:?}: n = {n_ab}, expected {expected}"
    );
    assert!(
        (n_ba + expected).length() < 1e-6,
        "{sb:?} vs {sa:?}: n = {n_ba}, expected {}",
        -expected
    );
    assert!((d_ab - d_ba).abs() < 1e-6, "depth {d_ab} vs {d_ba}");
}

#[test]
fn contact_normals_point_from_a_to_b() {
    let cube = ColliderShape::Box {
        half_extents: [1.0, 0.5, 1.0],
    };
    let ball = ColliderShape::Ball { radius: 0.6 };
    let capsule = ColliderShape::Capsule {
        half_height: 0.5,
        radius: 0.6,
    };
    let seg = ColliderShape::Segment {
        a: [-0.5, 0.0, 0.0],
        b: [0.5, 0.0, 0.0],
    };
    let up = DVec3::Y;
    // Box below, other shape above: normal Box -> other is +Y.
    assert_normal_a_to_b(&cube, DVec3::ZERO, &ball, DVec3::new(0.0, 1.0, 0.0), up);
    assert_normal_a_to_b(&cube, DVec3::ZERO, &capsule, DVec3::new(0.0, 1.5, 0.0), up);
    assert_normal_a_to_b(&cube, DVec3::ZERO, &seg, DVec3::new(0.0, 0.4, 0.0), up);
    assert_normal_a_to_b(&ball, DVec3::ZERO, &ball, DVec3::new(0.0, 1.0, 0.0), up);
    assert_normal_a_to_b(&capsule, DVec3::ZERO, &ball, DVec3::new(0.0, 1.5, 0.0), up);
    assert_normal_a_to_b(&seg, DVec3::ZERO, &ball, DVec3::new(0.0, 0.5, 0.0), up);
    assert_normal_a_to_b(
        &cube_hull(0.5),
        DVec3::ZERO,
        &ball,
        DVec3::new(0.0, 0.9, 0.0),
        up,
    );
    assert_normal_a_to_b(
        &cube_hull(0.5),
        DVec3::ZERO,
        &cube,
        DVec3::new(0.0, 0.9, 0.0),
        up,
    );
}

#[test]
fn closest_points_parallel_offset_segments() {
    let (p, q) = closest_points_segments_3d(
        DVec3::ZERO,
        DVec3::new(10.0, 0.0, 0.0),
        DVec3::new(5.0, 1.0, 0.0),
        DVec3::new(15.0, 1.0, 0.0),
    );
    assert!(((p - q).length() - 1.0).abs() < 1e-9, "p = {p}, q = {q}");
}

#[test]
fn closest_points_segments_match_brute_force() {
    // Deterministic LCG so the test is reproducible.
    let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
    let mut rnd = move || {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((seed >> 11) as f64 / (1u64 << 53) as f64) * 4.0 - 2.0
    };
    let seg_dist = |a: DVec3, b: DVec3, c: DVec3, d: DVec3| {
        // Golden-section on s of the exact point-to-segment distance.
        let f = |s: f64| {
            let p = a + (b - a) * s;
            (p - closest_point_on_segment_3d(c, d, p)).length()
        };
        let mut best = f(0.0).min(f(1.0));
        for i in 0..=2000 {
            best = best.min(f(i as f64 / 2000.0));
        }
        best
    };
    for _ in 0..300 {
        let a = DVec3::new(rnd(), rnd(), rnd());
        let b = DVec3::new(rnd(), rnd(), rnd());
        let c = DVec3::new(rnd(), rnd(), rnd());
        let d = DVec3::new(rnd(), rnd(), rnd());
        let (p, q) = closest_points_segments_3d(a, b, c, d);
        let got = (p - q).length();
        let want = seg_dist(a, b, c, d);
        assert!(
            got <= want + 1e-3,
            "{a} {b} {c} {d}: got {got}, want {want}"
        );
    }
}

#[test]
fn capsule_endpoint_touches_box() {
    let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
    let r = capsule_box_3d(
        DVec3::new(7.0, 2.0, 0.0),
        rot,
        std::f64::consts::SQRT_2,
        0.6,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(10.0, 0.5, 10.0),
    );
    let (n, d, _) = r.expect("tilted capsule endpoint should touch the box");
    assert!((n - DVec3::Y).length() < 1e-6, "n = {n}");
    assert!((d - 0.1).abs() < 1e-6, "d = {d}");
}

#[test]
fn gjk_handles_collinear_initial_simplex() {
    // Axis-aligned hull cubes offset along X: the first two Minkowski support
    // points are collinear with the origin.
    let hull = cube_hull(0.5);
    let r = gjk_epa_shapes(
        &hull,
        DVec3::ZERO,
        DQuat::IDENTITY,
        &hull,
        DVec3::new(0.8, 0.0, 0.0),
        DQuat::IDENTITY,
    );
    let (n, d, _) = r.expect("overlapping cubes must collide");
    assert!((n - DVec3::X).length() < 1e-6, "n = {n}");
    assert!((d - 0.2).abs() < 1e-6, "d = {d}");
}

#[test]
fn epa_on_curved_shapes_returns_best_face() {
    // Sphere-sphere through GJK/EPA converges slowly on the curved surface;
    // hitting the iteration cap must still produce a contact.
    let ball = ColliderShape::Ball { radius: 1.0 };
    let pb = DVec3::new(1.2, 0.5, 0.3);
    let (n, d, _) = gjk_epa_shapes(
        &ball,
        DVec3::ZERO,
        DQuat::IDENTITY,
        &ball,
        pb,
        DQuat::IDENTITY,
    )
    .expect("overlapping spheres must collide");
    let want = 2.0 - pb.length();
    assert!((d - want).abs() < 0.02, "d = {d}, want {want}");
    assert!(n.dot(pb.normalize()) > 0.99, "n = {n}");
}

#[test]
fn convex_hull_sphere_uses_hull_volume() {
    let points = cube_points(0.5);
    // Sphere resting 0.1 into the top face.
    let (n, d, _) = convex_hull_sphere_3d(
        &points,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(0.0, 0.7, 0.0),
        0.3,
    )
    .expect("sphere overlaps the top face");
    assert!((n - DVec3::Y).length() < 1e-6, "n = {n}");
    assert!((d - 0.1).abs() < 1e-6, "d = {d}");

    // Sphere centre inside the hull: outward normal, depth = face gap + r.
    let (n, d, _) = convex_hull_sphere_3d(
        &points,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(0.05, 0.4, 0.1),
        0.3,
    )
    .expect("sphere centre inside hull");
    assert!((n - DVec3::Y).length() < 1e-6, "n = {n}");
    assert!((d - 0.4).abs() < 1e-6, "d = {d}");
}

#[test]
fn aabb_contact_point_is_overlap_centre() {
    let ground = DVec3::new(10.0, 0.5, 10.0);
    let small = DVec3::splat(0.5);
    let pos = DVec3::new(3.0, 0.95, 2.0);
    let (n, d, p) = aabb_aabb_3d(DVec3::ZERO, ground, pos, small).expect("overlap");
    assert!((n - DVec3::Y).length() < EPS);
    assert!((d - 0.05).abs() < EPS);
    assert!((p - DVec3::new(3.0, 0.475, 2.0)).length() < EPS, "p = {p}");
    let (_, _, p2) = aabb_aabb_3d(pos, small, DVec3::ZERO, ground).expect("overlap");
    assert!((p2 - p).length() < EPS, "p2 = {p2}");
}

#[test]
fn obb_contact_point_is_under_the_box() {
    let ground = DVec3::new(10.0, 0.5, 10.0);
    let small = DVec3::splat(0.5);
    let pos = DVec3::new(3.0, 0.95, 2.0);
    let rot = DQuat::from_rotation_y(0.3);
    for (pa, ra, ha, pb, rb, hb, sign) in [
        (DVec3::ZERO, DQuat::IDENTITY, ground, pos, rot, small, 1.0),
        (pos, rot, small, DVec3::ZERO, DQuat::IDENTITY, ground, -1.0),
    ] {
        let (n, d, p) = obb_obb_3d(pa, ra, ha, pb, rb, hb).expect("overlap");
        assert!((n - DVec3::Y * sign).length() < 1e-9, "n = {n}");
        assert!((d - 0.05).abs() < 1e-9, "d = {d}");
        assert!(
            (p.x - 3.0).abs() < 1e-9 && (p.z - 2.0).abs() < 1e-9,
            "p = {p}"
        );
        assert!(p.y > 0.44 && p.y < 0.51, "p = {p}");
    }
}

#[test]
fn obb_sat_uses_edge_axes() {
    let r = obb_obb_3d(
        DVec3::ZERO,
        DQuat::from_rotation_y(std::f64::consts::FRAC_PI_4),
        DVec3::splat(0.5),
        DVec3::new(1.5, 0.0, 0.0),
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4),
        DVec3::splat(0.5),
    );
    assert!(r.is_none(), "edge-edge separated boxes reported {r:?}");
}

#[test]
fn obb_edge_edge_contact() {
    // Same pair, moved into edge-edge overlap of 0.1 along X.
    let sep = std::f64::consts::SQRT_2 - 0.1;
    let (n, d, p) = obb_obb_3d(
        DVec3::ZERO,
        DQuat::from_rotation_y(std::f64::consts::FRAC_PI_4),
        DVec3::splat(0.5),
        DVec3::new(sep, 0.0, 0.0),
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4),
        DVec3::splat(0.5),
    )
    .expect("edge-edge overlap");
    assert!((n - DVec3::X).length() < 1e-6, "n = {n}");
    assert!((d - 0.1).abs() < 1e-6, "d = {d}");
    assert!(
        (p - DVec3::new(sep * 0.5, 0.0, 0.0)).length() < 1e-6,
        "p = {p}"
    );
}

#[test]
fn static_body_switched_to_dynamic_gains_mass() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(bh, &desc(ColliderShape::Ball { radius: 0.5 }));
    assert_eq!(state.bodies.get(body_ah(bh)).unwrap().inv_mass, 0.0);

    state.set_body_type(bh, BodyType::Dynamic).unwrap();
    let rb = state.bodies.get(body_ah(bh)).unwrap();
    let expected = (4.0 / 3.0) * std::f64::consts::PI * 0.125;
    assert!((rb.mass - expected).abs() < EPS, "mass = {}", rb.mass);
    assert!((rb.inv_mass - 1.0 / expected).abs() < EPS);
    assert!(rb.inv_inertia.x_axis.x > 0.0);

    for _ in 0..30 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    assert!(state.bodies.get(body_ah(bh)).unwrap().position.y < 9.5);

    state.set_body_type(bh, BodyType::Kinematic).unwrap();
    let rb = state.bodies.get(body_ah(bh)).unwrap();
    assert_eq!(rb.mass, 0.0);
    assert_eq!(rb.inv_mass, 0.0);
    assert_eq!(rb.inv_inertia, hisab::DMat3::ZERO);
}

#[test]
fn offset_colliders_use_parallel_axis_theorem() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc::default());
    for x in [-2.0, 2.0] {
        state.add_collider(
            bh,
            &ColliderDesc {
                offset: [x, 0.0, 0.0],
                mass: Some(1.0),
                ..desc(ColliderShape::Ball { radius: 1.0 })
            },
        );
    }
    let rb = state.bodies.get(body_ah(bh)).unwrap();
    let i_own = 0.4; // 2/5 m r² per ball
    assert!(
        (rb.inertia.x_axis.x - 2.0 * i_own).abs() < EPS,
        "{}",
        rb.inertia
    );
    assert!((rb.inertia.y_axis.y - 2.0 * (i_own + 4.0)).abs() < EPS);
    assert!((rb.inertia.z_axis.z - 2.0 * (i_own + 4.0)).abs() < EPS);

    // Off-diagonal terms for a diagonal offset.
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc::default());
    let ch = state.add_collider(
        bh,
        &ColliderDesc {
            offset: [1.0, 1.0, 0.0],
            mass: Some(1.0),
            ..desc(ColliderShape::Ball { radius: 1.0 })
        },
    );
    let rb = state.bodies.get(body_ah(bh)).unwrap();
    assert!((rb.inertia.x_axis.y + 1.0).abs() < EPS, "{}", rb.inertia);
    assert!((rb.inertia.z_axis.z - (0.4 + 2.0)).abs() < EPS);

    // Removing the collider recomputes with the same rules.
    state.remove_collider(ch).unwrap();
    let rb = state.bodies.get(body_ah(bh)).unwrap();
    assert_eq!(rb.mass, 0.0);
    assert_eq!(rb.inv_mass, 0.0);
}

#[test]
fn capsule_inertia_matches_standard_tensor() {
    let (r, hh): (f64, f64) = (0.5, 1.0);
    let c = Collider3d::from_desc(
        ColliderHandle(0),
        BodyHandle(0),
        &desc(ColliderShape::Capsule {
            half_height: hh,
            radius: r,
        }),
    );
    let m = c.compute_mass();
    let i = c.compute_inertia(m);
    let h = 2.0 * hh;
    let pi = std::f64::consts::PI;
    let (v_cyl, v_hemis) = (pi * r * r * h, 4.0 / 3.0 * pi * r.powi(3));
    let (m_cyl, m_hemis) = (
        m * v_cyl / (v_cyl + v_hemis),
        m * v_hemis / (v_cyl + v_hemis),
    );
    let i_axis = m_cyl * r * r / 2.0 + m_hemis * 2.0 * r * r / 5.0;
    let i_perp = m_cyl * (h * h / 12.0 + r * r / 4.0)
        + m_hemis * (2.0 * r * r / 5.0 + h * h / 4.0 + 3.0 * h * r / 8.0);
    assert!(
        (i.y_axis.y - i_axis).abs() < 1e-9,
        "{} vs {i_axis}",
        i.y_axis.y
    );
    assert!(
        (i.x_axis.x - i_perp).abs() < 1e-9,
        "{} vs {i_perp}",
        i.x_axis.x
    );
    assert!((i.z_axis.z - i_perp).abs() < 1e-9);
}

#[test]
fn hull_mass_scales_with_size() {
    let mass = |h| {
        Collider3d::from_desc(ColliderHandle(0), BodyHandle(0), &desc(cube_hull(h))).compute_mass()
    };
    assert!((mass(0.5) - 1.0).abs() < EPS);
    assert!((mass(1.0) - 8.0).abs() < EPS);
}

#[test]
fn angular_impulse_uses_world_space_inertia() {
    // Long box (along body X) rotated 90° about Z: its long axis is world Y,
    // so a world-Y angular impulse sees the *small* body-X inertia.
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        rotation: std::f64::consts::FRAC_PI_2,
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &desc(ColliderShape::Box {
            half_extents: [2.0, 0.5, 0.5],
        }),
    );
    let i_long = {
        let rb = state.bodies.get(body_ah(bh)).unwrap();
        rb.inertia.x_axis.x
    };
    // (1,0,0) × (0,0,1) = (0,-1,0): unit angular impulse about -Y.
    state.apply_impulse(
        bh,
        &Impulse {
            vector: [0.0, 0.0, 1.0],
            point: Some([1.0, 0.0, 0.0]),
        },
    );
    let w = state.bodies.get(body_ah(bh)).unwrap().angular_velocity;
    assert!(
        (w.y + 1.0 / i_long).abs() < 1e-9,
        "w = {w}, want y = {}",
        -1.0 / i_long
    );
    assert!(w.x.abs() < 1e-9 && w.z.abs() < 1e-9, "w = {w}");
}

fn spin_energy_and_momentum(rb: &RigidBody3d) -> (f64, f64) {
    let r = hisab::DMat3::from_quat(rb.rotation);
    let i_world = r * rb.inertia * r.transpose();
    let l = i_world * rb.angular_velocity;
    (0.5 * rb.angular_velocity.dot(l), l.length())
}

#[test]
fn free_spinning_body_does_not_gain_energy() {
    let mut state = PhysicsState3d::new();
    let bh = state.add_body(&BodyDesc {
        rotation: 0.7,
        ..BodyDesc::default()
    });
    state.add_collider(
        bh,
        &desc(ColliderShape::Box {
            half_extents: [1.0, 0.5, 0.25],
        }),
    );
    // Mostly about the (unstable) intermediate axis, rotated into world frame.
    {
        let rb = state.bodies.get_mut(body_ah(bh)).unwrap();
        rb.rotation = DQuat::from_rotation_x(0.3)
            * DQuat::from_rotation_y(0.7)
            * DQuat::from_rotation_z(-0.4);
        rb.angular_velocity = rb.rotation * DVec3::new(0.2, 8.0, 0.3);
    }
    let (e0, l0) = spin_energy_and_momentum(state.bodies.get(body_ah(bh)).unwrap());
    for _ in 0..600 {
        frame(&mut state, [0.0, 0.0, 0.0]);
        let (e, _) = spin_energy_and_momentum(state.bodies.get(body_ah(bh)).unwrap());
        assert!(e <= e0 * (1.0 + 1e-9), "energy grew: {e0} -> {e}");
    }
    let (e1, l1) = spin_energy_and_momentum(state.bodies.get(body_ah(bh)).unwrap());
    assert!(e1 > 0.5 * e0, "energy collapsed: {e0} -> {e1}");
    assert!((l1 - l0).abs() < 0.1 * l0, "|L| drifted: {l0} -> {l1}");
}

fn ground(state: &mut PhysicsState3d, material: PhysicsMaterial) -> BodyHandle {
    let g = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    state.add_collider(
        g,
        &ColliderDesc {
            material,
            ..desc(ColliderShape::Box {
                half_extents: [50.0, 0.5, 50.0],
            })
        },
    );
    g
}

#[allow(clippy::too_many_arguments)]
fn step_with(
    state: &mut PhysicsState3d,
    gravity: [f64; 3],
    velocity_iterations: u32,
    position_iterations: u32,
    slop: f64,
    correction: f64,
    frequency: f64,
) {
    state.step_frame(
        gravity,
        1.0 / 60.0,
        velocity_iterations,
        position_iterations,
        slop,
        correction,
        100.0,
        frequency,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
}

#[test]
fn baumgarte_position_correction_is_per_step_fraction() {
    // depth 0.1, slop 0, correction 0.2 => one step separates by 0.02,
    // independent of position iterations and of the number of contacts.
    for (iters, two_colliders) in [(1, false), (4, false), (4, true)] {
        let mut state = PhysicsState3d::new();
        ground(&mut state, PhysicsMaterial::default());
        let b = state.add_body(&BodyDesc {
            position: [0.0, 0.9, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &desc(ColliderShape::Ball { radius: 0.5 }));
        if two_colliders {
            state.add_collider(b, &desc(ColliderShape::Ball { radius: 0.5 }));
        }
        step_with(&mut state, [0.0; 3], 4, iters, 0.0, 0.2, 0.0);
        let rb = state.bodies.get(body_ah(b)).unwrap();
        let moved = rb.position.y - 0.9;
        assert!(
            (moved - 0.02).abs() < 1e-6,
            "iters={iters} two={two_colliders}: moved {moved}"
        );
        // Split impulse: no real velocity is left behind.
        assert!(rb.linear_velocity.length() < 1e-9);
    }
}

#[test]
fn restitution_independent_of_iteration_count() {
    for iters in 1..=6 {
        for e in [0.5, 1.0] {
            let bouncy = PhysicsMaterial {
                restitution: e,
                ..PhysicsMaterial::default()
            };
            let mut state = PhysicsState3d::new();
            ground(&mut state, bouncy.clone());
            let b = state.add_body(&BodyDesc {
                position: [0.0, 0.95, 0.0],
                linear_velocity: [0.0, -10.0, 0.0],
                ..BodyDesc::default()
            });
            state.add_collider(
                b,
                &ColliderDesc {
                    material: bouncy,
                    ..desc(ColliderShape::Ball { radius: 0.5 })
                },
            );
            step_with(&mut state, [0.0; 3], iters, 1, 0.01, 0.2, 30.0);
            let vy = state.bodies.get(body_ah(b)).unwrap().linear_velocity.y;
            assert!(
                (vy - 10.0 * e).abs() < 1e-6,
                "iters={iters} e={e}: vy = {vy}"
            );
        }
    }
}

fn sliding_box(state: &mut PhysicsState3d, vx: f64) -> BodyHandle {
    let b = state.add_body(&BodyDesc {
        position: [0.0, 1.0, 0.0],
        linear_velocity: [vx, 0.0, 0.0],
        fixed_rotation: true,
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &desc(ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.5],
        }),
    );
    b
}

#[test]
fn kinetic_friction_decelerates_at_mu_g() {
    let mut state = PhysicsState3d::new();
    ground(&mut state, PhysicsMaterial::default());
    let b = sliding_box(&mut state, 5.0);
    for _ in 0..30 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let v = state.bodies.get(body_ah(b)).unwrap().linear_velocity;
    let want = 5.0 - 0.5 * 9.81 * 0.5;
    assert!((v.x - want).abs() < 0.1, "vx = {}, want {want}", v.x);
    assert!(v.z.abs() < 1e-9, "v = {v}");
}

#[test]
fn static_friction_holds_below_threshold() {
    // mu = 0.5, static = 0.75: a 0.6·m·g push must not start a slide.
    let mut state = PhysicsState3d::new();
    ground(&mut state, PhysicsMaterial::default());
    let b = sliding_box(&mut state, 0.0);
    state.bodies.get_mut(body_ah(b)).unwrap().position.y = 0.99;
    let m = state.bodies.get(body_ah(b)).unwrap().mass;
    for _ in 0..60 {
        state.apply_force(b, &crate::force::Force::new(0.6 * m * 9.81, 0.0, 0.0));
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let x = state.bodies.get(body_ah(b)).unwrap().position.x;
    assert!(x.abs() < 1e-3, "x = {x}");
}

#[test]
fn bad_friction_coefficients_do_not_panic() {
    for mu in [-1.0, f64::NAN, f64::INFINITY] {
        let mat = PhysicsMaterial {
            friction: mu,
            static_friction: Some(mu),
            restitution: mu,
            rolling_friction: mu,
            ..PhysicsMaterial::default()
        };
        for g in [-9.81, 0.0] {
            let mut state = PhysicsState3d::new();
            ground(&mut state, mat.clone());
            let b = sliding_box(&mut state, 3.0);
            // Resting contact with zero approach speed (zero normal impulse).
            state.bodies.get_mut(body_ah(b)).unwrap().position.y = 0.99;
            for _ in 0..10 {
                frame(&mut state, [0.0, g, 0.0]);
            }
            let rb = state.bodies.get(body_ah(b)).unwrap();
            assert!(
                rb.position.is_finite() && rb.linear_velocity.is_finite(),
                "mu={mu} g={g}"
            );
        }
    }
}

#[test]
fn box_resting_off_centre_on_ground_stays_upright() {
    let mut state = PhysicsState3d::new();
    ground(&mut state, PhysicsMaterial::default());
    let b = state.add_body(&BodyDesc {
        position: [3.0, 1.0, 2.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &desc(ColliderShape::Box {
            half_extents: [0.5, 0.5, 0.5],
        }),
    );
    // Rotate about Y so the OBB path is used.
    state.bodies.get_mut(body_ah(b)).unwrap().rotation = DQuat::from_rotation_y(0.3);
    for _ in 0..180 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    let up = rb.rotation * DVec3::Y;
    assert!(up.y > 0.9999, "box tipped: up = {up}");
    assert!(
        rb.angular_velocity.length() < 1e-3,
        "w = {}",
        rb.angular_velocity
    );
    assert!((rb.position.y - 1.0).abs() < 0.03, "y = {}", rb.position.y);
    assert!((rb.position.x - 3.0).abs() < 1e-3 && (rb.position.z - 2.0).abs() < 1e-3);
}

fn joint_desc(a: BodyHandle, b: BodyHandle, joint_type: JointType) -> JointDesc {
    JointDesc {
        body_a: a,
        body_b: b,
        joint_type,
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    }
}

/// Static anchor at (0, 10, 0) and a dynamic ball at `pos`.
fn anchored_ball(pos: [f64; 3]) -> (PhysicsState3d, BodyHandle, BodyHandle) {
    let mut state = PhysicsState3d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    let b = state.add_body(&BodyDesc {
        position: pos,
        ..BodyDesc::default()
    });
    state.add_collider(b, &desc(ColliderShape::Ball { radius: 0.25 }));
    (state, a, b)
}

#[test]
fn distance_joint_holds_hanging_ball() {
    let (mut state, a, b) = anchored_ball([0.0, 8.0, 0.0]);
    state.add_joint(&joint_desc(a, b, JointType::Distance { length: 2.0 }));
    for _ in 0..600 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    let dist = (rb.position - DVec3::new(0.0, 10.0, 0.0)).length();
    assert!(
        rb.linear_velocity.length() < 1.0,
        "v = {}",
        rb.linear_velocity
    );
    assert!((dist - 2.0).abs() < 0.01, "dist = {dist}");
}

#[test]
fn distance_joint_pendulum_keeps_length_and_energy() {
    let (mut state, a, b) = anchored_ball([2.0, 10.0, 0.0]);
    state.add_joint(&joint_desc(a, b, JointType::Distance { length: 2.0 }));
    let v_max = (2.0_f64 * 9.81 * 2.0).sqrt();
    for _ in 0..600 {
        frame(&mut state, [0.0, -9.81, 0.0]);
        let rb = state.bodies.get(body_ah(b)).unwrap();
        let dist = (rb.position - DVec3::new(0.0, 10.0, 0.0)).length();
        assert!((dist - 2.0).abs() < 0.01, "dist = {dist}");
        assert!(
            rb.linear_velocity.length() < v_max * 1.01,
            "v = {}",
            rb.linear_velocity
        );
    }
}

#[test]
fn rope_joint_holds_max_length() {
    let (mut state, a, b) = anchored_ball([1.0, 8.5, 0.0]);
    state.add_joint(&joint_desc(a, b, JointType::Rope { max_length: 2.0 }));
    for _ in 0..600 {
        frame(&mut state, [0.0, -9.81, 0.0]);
        let rb = state.bodies.get(body_ah(b)).unwrap();
        let dist = (rb.position - DVec3::new(0.0, 10.0, 0.0)).length();
        assert!(dist < 2.02, "dist = {dist}");
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    assert!(rb.linear_velocity.length() < 7.0);
    // Slack rope: no force while closer than max_length.
    let (mut state, a, b) = anchored_ball([0.0, 9.0, 0.0]);
    state.add_joint(&joint_desc(a, b, JointType::Rope { max_length: 2.0 }));
    frame(&mut state, [0.0, -9.81, 0.0]);
    let vy = state.bodies.get(body_ah(b)).unwrap().linear_velocity.y;
    assert!((vy + 9.81 / 60.0).abs() < 1e-9, "vy = {vy}");
}

#[test]
fn fixed_joint_locks_relative_rotation() {
    // Cantilever: B's centre is 2 m from the anchor at A's centre, so gravity
    // produces a torque about the anchor that only an angular lock resists.
    let mut state = PhysicsState3d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let b = state.add_body(&BodyDesc {
        position: [2.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            collision_layer: 0,
            collision_mask: 0,
            ..desc(ColliderShape::Box {
                half_extents: [0.5, 0.1, 0.1],
            })
        },
    );
    state.add_joint(&JointDesc {
        local_anchor_b: [-2.0, 0.0],
        ..joint_desc(a, b, JointType::Fixed)
    });
    for _ in 0..300 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    assert!(
        (rb.position - DVec3::new(2.0, 0.0, 0.0)).length() < 0.05,
        "p = {}",
        rb.position
    );
    assert!((rb.rotation * DVec3::X).x > 0.999, "q = {}", rb.rotation);
}

#[test]
fn wheel_joint_constrains_both_perpendicular_axes() {
    let (mut state, a, b) = anchored_ball([0.3, 9.0, 0.2]);
    state.add_joint(&joint_desc(
        a,
        b,
        JointType::Wheel {
            axis: [0.0, 1.0],
            stiffness: 50.0,
            damping: 2.0,
        },
    ));
    for _ in 0..600 {
        frame(&mut state, [0.0, -9.81, 0.0]);
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    assert!(rb.position.x.abs() < 0.01, "p = {}", rb.position);
    assert!(rb.position.z.abs() < 0.01, "p = {}", rb.position);
    assert!(
        rb.linear_velocity.length() < 0.1,
        "v = {}",
        rb.linear_velocity
    );
}

/// Velocity of a 1 kg ball after one 0.01 s step with the given joint.
fn one_step_velocity(joint_type: JointType, damping: f64, iters: u32) -> DVec3 {
    let mut state = PhysicsState3d::new();
    let a = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let b = state.add_body(&BodyDesc {
        position: [1.0, 0.0, 0.0],
        linear_velocity: [0.0, 2.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(
        b,
        &ColliderDesc {
            mass: Some(1.0),
            ..desc(ColliderShape::Ball { radius: 0.1 })
        },
    );
    let jt = joint_type;
    let body_a = if matches!(jt, JointType::Mouse { .. }) {
        b
    } else {
        a
    };
    state.add_joint(&JointDesc {
        damping,
        ..joint_desc(body_a, b, jt)
    });
    state.step_frame(
        [0.0; 3],
        0.01,
        iters,
        1,
        0.01,
        0.2,
        100.0,
        30.0,
        1.0,
        crate::config::BroadphaseKind::SpatialHash,
    );
    state.bodies.get(body_ah(b)).unwrap().linear_velocity
}

#[test]
fn explicit_joint_forces_independent_of_iterations() {
    let cases = [
        (
            JointType::Spring {
                rest_length: 0.0,
                stiffness: 10.0,
                damping: 0.0,
            },
            0.0,
            DVec3::new(-0.1, 2.0, 0.0),
        ),
        (
            JointType::Mouse {
                target: [2.0, 0.0, 0.0],
                stiffness: 10.0,
                damping: 0.0,
                max_force: 100.0,
            },
            0.0,
            DVec3::new(0.1, 2.0, 0.0),
        ),
        (
            // Joint damping on a (no-op in 3D) revolute: Δv = -c·v·dt.
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
            5.0,
            DVec3::new(0.0, 2.0 - 5.0 * 2.0 * 0.01, 0.0),
        ),
    ];
    for (jt, damping, want) in cases {
        let v1 = one_step_velocity(jt.clone(), damping, 1);
        let v8 = one_step_velocity(jt.clone(), damping, 8);
        assert!(
            (v1 - want).length() < 1e-9,
            "{jt:?}: v1 = {v1}, want {want}"
        );
        assert!((v8 - v1).length() < 1e-12, "{jt:?}: v1 = {v1}, v8 = {v8}");
    }
}

#[test]
fn joint_breaks_on_reaction_force() {
    // The 0.065 kg ball weighs ~0.64 N.
    for (bf, breaks) in [(0.1, true), (1.0, false)] {
        let (mut state, a, b) = anchored_ball([0.0, 8.0, 0.0]);
        state.add_joint(&JointDesc {
            break_force: Some(bf),
            ..joint_desc(a, b, JointType::Distance { length: 2.0 })
        });
        for _ in 0..5 {
            frame(&mut state, [0.0, -9.81, 0.0]);
        }
        assert_eq!(state.joints.len() == 0, breaks, "break_force = {bf}");
    }
}

fn static_shape(state: &mut PhysicsState3d, pos: [f64; 3], rot_z: f64, shape: ColliderShape) {
    let b = state.add_body(&BodyDesc {
        body_type: BodyType::Static,
        position: pos,
        rotation: rot_z,
        ..BodyDesc::default()
    });
    state.add_collider(b, &desc(shape));
}

#[test]
fn raycast_respects_box_rotation() {
    let mut state = PhysicsState3d::new();
    static_shape(
        &mut state,
        [0.0; 3],
        std::f64::consts::FRAC_PI_2,
        ColliderShape::Box {
            half_extents: [2.0, 0.1, 0.5],
        },
    );
    // Rotated 90° about Z, the box spans x ∈ [-0.1, 0.1], y ∈ [-2, 2].
    for hit in [
        state.raycast([5.0, 1.5, 0.0], [-1.0, 0.0, 0.0], 100.0),
        state.raycast_filtered([5.0, 1.5, 0.0], [-1.0, 0.0, 0.0], 100.0, 0xFFFF_FFFF),
    ] {
        let hit = hit.expect("ray should hit the rotated box");
        assert!((hit.distance - 4.9).abs() < 1e-9, "t = {}", hit.distance);
        let n = DVec3::from_array(hit.normal);
        assert!((n - DVec3::X).length() < 1e-9, "n = {n}");
    }
    let miss = state.raycast([1.5, 5.0, 0.0], [0.0, -1.0, 0.0], 100.0);
    assert!(miss.is_none(), "hit: {miss:?}");
}

#[test]
fn raycast_zero_direction_returns_none() {
    let mut state = PhysicsState3d::new();
    static_shape(
        &mut state,
        [0.0; 3],
        0.0,
        ColliderShape::Ball { radius: 1.0 },
    );
    assert!(state.raycast([0.0, -5.0, 0.0], [0.0; 3], 100.0).is_none());
    assert!(
        state
            .raycast_filtered([0.0, -5.0, 0.0], [0.0; 3], 100.0, 0xFFFF_FFFF)
            .is_none()
    );
    assert!(
        state
            .raycast([0.0, -5.0, 0.0], [f64::NAN, 1.0, 0.0], 100.0)
            .is_none()
    );
}

#[test]
fn raycast_hits_capsule() {
    let capsule = ColliderShape::Capsule {
        half_height: 1.0,
        radius: 0.5,
    };
    let mut state = PhysicsState3d::new();
    static_shape(&mut state, [5.0, 0.0, 0.0], 0.0, capsule.clone());
    // Side of the cylinder.
    let hit = state
        .raycast([0.0; 3], [1.0, 0.0, 0.0], 100.0)
        .expect("side hit");
    assert!((hit.distance - 4.5).abs() < 1e-9, "t = {}", hit.distance);
    assert!((DVec3::from_array(hit.normal) + DVec3::X).length() < 1e-9);
    // Along the axis into the bottom cap.
    let hit = state
        .raycast([5.0, -5.0, 0.0], [0.0, 1.0, 0.0], 100.0)
        .expect("cap hit");
    assert!((hit.distance - 3.5).abs() < 1e-9, "t = {}", hit.distance);
    assert!((DVec3::from_array(hit.normal) + DVec3::Y).length() < 1e-9);
    // Grazing miss.
    assert!(
        state
            .raycast([0.0, 0.0, 0.6], [1.0, 0.0, 0.0], 100.0)
            .is_none()
    );

    // Rotated 90° about Z: the axis lies along X, so the cap faces the ray.
    let mut state = PhysicsState3d::new();
    static_shape(
        &mut state,
        [5.0, 0.0, 0.0],
        std::f64::consts::FRAC_PI_2,
        capsule,
    );
    let hit = state
        .raycast([0.0; 3], [1.0, 0.0, 0.0], 100.0)
        .expect("cap hit");
    assert!((hit.distance - 3.5).abs() < 1e-9, "t = {}", hit.distance);
}

#[test]
fn sleeping_body_does_not_drift() {
    let mut state = PhysicsState3d::new();
    let b = state.add_body(&BodyDesc {
        linear_velocity: [0.005, 0.0, 0.0],
        ..BodyDesc::default()
    });
    state.add_collider(b, &desc(ColliderShape::Ball { radius: 0.5 }));
    for _ in 0..120 {
        frame(&mut state, [0.0; 3]);
    }
    let rb = state.bodies.get(body_ah(b)).unwrap();
    assert!(rb.is_sleeping, "body should be asleep");
    assert_eq!(rb.linear_velocity, DVec3::ZERO);
    let p1 = rb.position;
    for _ in 0..120 {
        frame(&mut state, [0.0; 3]);
    }
    assert_eq!(state.bodies.get(body_ah(b)).unwrap().position, p1);
}
