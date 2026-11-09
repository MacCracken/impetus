//! Regression tests for defects found in the v1.4 audit.
//!
//! Each test pins one previously-wrong behaviour. Tests that only make sense
//! for one backend are gated on it; `3d` takes precedence when both are on.

use impetus::*;

#[cfg(any(feature = "2d", feature = "3d"))]
fn ball(radius: f64) -> ColliderDesc {
    ColliderDesc {
        shape: ColliderShape::Ball { radius },
        offset: [0.0; 3],
        material: PhysicsMaterial::default(),
        is_sensor: false,
        mass: None,
        collision_layer: 0xFFFF_FFFF,
        collision_mask: 0xFFFF_FFFF,
    }
}

#[cfg(any(feature = "2d", feature = "3d"))]
fn boxed(half: [f64; 3]) -> ColliderDesc {
    ColliderDesc {
        shape: ColliderShape::Box { half_extents: half },
        ..ball(1.0)
    }
}

#[cfg(any(feature = "2d", feature = "3d"))]
fn zero_g() -> WorldConfig {
    WorldConfig {
        gravity: [0.0; 3],
        ..WorldConfig::default()
    }
}

// ---------------------------------------------------------------------------
// Sub-stepping
// ---------------------------------------------------------------------------

/// A force applied before `step()` must act over the whole step, not only
/// the first sub-step.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn substep_force_acts_for_full_step() {
    let mut world = PhysicsWorld::new(WorldConfig {
        sub_steps: 4,
        ..zero_g()
    });
    let b = world.add_body(BodyDesc::default());
    world.add_collider(
        b,
        ColliderDesc {
            mass: Some(1.0),
            ..ball(0.5)
        },
    );
    world.apply_force(b, Force::new(10.0, 0.0, 0.0));
    world.step();
    let vx = world.get_body_state(b).unwrap().linear_velocity[0];
    assert!((vx - 10.0 / 60.0).abs() < 1e-9, "vx = {vx}");
}

/// A contact that begins inside a sub-stepped frame must report `Started`.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn substep_reports_started_event() {
    let mut world = PhysicsWorld::new(WorldConfig {
        sub_steps: 4,
        ..WorldConfig::default()
    });
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(ground, boxed([10.0, 0.5, 10.0]));
    let b = world.add_body(BodyDesc {
        position: [0.0, 0.9, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.5));
    world.step();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|e| matches!(e, CollisionEvent::Started { .. })),
        "events: {:?}",
        world.collision_events()
    );
}

/// In 3D, `sub_steps` must change the integration (it used to be ignored).
#[cfg(feature = "3d")]
#[test]
fn substeps_are_honoured_in_3d() {
    let fall = |sub_steps| {
        let mut world = PhysicsWorld::new(WorldConfig {
            sub_steps,
            ..WorldConfig::default()
        });
        let b = world.add_body(BodyDesc::default());
        world.add_collider(b, ball(0.5));
        world.step();
        world.get_body_state(b).unwrap().position[1]
    };
    assert_ne!(fall(1), fall(4));
}

// ---------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------

/// A body created static and switched to dynamic must gain mass and fall.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn static_to_dynamic_gains_mass() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let b = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.5));
    world.set_body_type(b, BodyType::Dynamic).unwrap();
    for _ in 0..60 {
        world.step();
    }
    let y = world.get_body_state(b).unwrap().position[1];
    assert!(y < 9.0, "y = {y}");
}

// ---------------------------------------------------------------------------
// Particles
// ---------------------------------------------------------------------------

/// Particle drag is documented as quadratic: Δv = c·v²·dt.
#[test]
fn particle_drag_is_quadratic() {
    let mut world = PhysicsWorld::new(WorldConfig {
        gravity: [0.0; 3],
        ..WorldConfig::default()
    });
    world.spawn_particle(Particle::new([0.0; 3], [10.0, 0.0, 0.0], 10.0).with_drag(1.0));
    world.step();
    let vx = world.particles()[0].velocity[0];
    let expected = 10.0 - 100.0 / 60.0;
    assert!((vx - expected).abs() < 1e-9, "vx = {vx}");
}

/// An emitter with an infinite rate must not hang `step()`.
#[test]
fn infinite_emitter_rate_terminates() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    world.add_emitter(ParticleEmitter::new([0.0; 3], [0.0; 3], f64::INFINITY));
    world.step();
    assert!(world.particle_count() <= 100_000);
}

/// Emitter spread on x and y must not be (anti-)correlated.
#[test]
fn emitter_spread_axes_independent() {
    let mut world = PhysicsWorld::new(WorldConfig {
        gravity: [0.0; 3],
        ..WorldConfig::default()
    });
    world.add_emitter(
        ParticleEmitter::new([0.0; 3], [0.0; 3], 600.0)
            .with_spread([1.0, 1.0, 0.0])
            .with_lifetime(100.0)
            .with_radius(0.0),
    );
    for _ in 0..50 {
        world.step();
    }
    let ps = world.particles();
    let n = ps.len() as f64;
    assert!(n > 100.0);
    let mx = ps.iter().map(|p| p.velocity[0]).sum::<f64>() / n;
    let my = ps.iter().map(|p| p.velocity[1]).sum::<f64>() / n;
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for p in ps {
        let dx = p.velocity[0] - mx;
        let dy = p.velocity[1] - my;
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    let corr = sxy / (sxx * syy).sqrt();
    assert!(corr.abs() < 0.3, "corr = {corr}");
}

/// A fast particle whose centre ends up inside a box must be pushed out,
/// not tunnel through.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn particle_does_not_tunnel_into_box() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let floor = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(floor, boxed([50.0, 0.5, 50.0]));
    world.spawn_particle(Particle::new([0.0, 2.5, 0.0], [0.0; 3], 100.0).with_radius(0.05));
    for _ in 0..240 {
        world.step();
    }
    let y = world.particles()[0].position[1];
    assert!(y > 0.0, "y = {y}");
}

// ---------------------------------------------------------------------------
// Joints (2D)
// ---------------------------------------------------------------------------

#[cfg(all(feature = "2d", not(feature = "3d")))]
fn joint(a: BodyHandle, b: BodyHandle, joint_type: JointType) -> JointDesc {
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

/// A rope must hold its max length under gravity over many steps.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn rope_holds_length_over_time() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let anchor = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    let b = world.add_body(BodyDesc {
        position: [0.0, 8.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.25));
    world.add_joint(joint(anchor, b, JointType::Rope { max_length: 2.0 }));
    for _ in 0..300 {
        world.step();
    }
    let s = world.get_body_state(b).unwrap();
    let dist = (10.0 - s.position[1]).hypot(s.position[0]);
    let speed = s.linear_velocity[0].hypot(s.linear_velocity[1]);
    assert!(dist < 2.05, "dist = {dist}");
    assert!(speed < 1.0, "speed = {speed}");
}

/// A spring joint's response must not scale with solver iterations.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn spring_joint_independent_of_iterations() {
    let run = |iters| {
        let mut world = PhysicsWorld::new(WorldConfig {
            velocity_iterations: iters,
            timestep: 0.01,
            ..zero_g()
        });
        let a = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            ..BodyDesc::default()
        });
        let b = world.add_body(BodyDesc {
            position: [1.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            b,
            ColliderDesc {
                mass: Some(1.0),
                ..ball(0.1)
            },
        );
        world.add_joint(joint(
            a,
            b,
            JointType::Spring {
                rest_length: 0.0,
                stiffness: 10.0,
                damping: 0.0,
            },
        ));
        world.step();
        world.get_body_state(b).unwrap().linear_velocity[0]
    };
    let v1 = run(1);
    let v8 = run(8);
    // Implicit (soft) spring: Δv = −k·x·dt / (1 + k·dt²/m) ≈ −0.0999.
    let expected = -0.1 / (1.0 + 10.0 * 0.01 * 0.01);
    assert!((v1 - expected).abs() < 1e-9, "v1 = {v1}");
    assert!((v8 - v1).abs() < 1e-9, "v1 = {v1}, v8 = {v8}");
}

/// A motor with a negative or NaN max force must not panic.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn motor_with_bad_max_force_does_not_panic() {
    for max_force in [-1.0, f64::NAN] {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let a = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            ..BodyDesc::default()
        });
        let b = world.add_body(BodyDesc {
            position: [1.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(b, ball(0.25));
        world.add_joint(JointDesc {
            motor: Some(JointMotor {
                target_velocity: 1.0,
                max_force,
            }),
            ..joint(
                a,
                b,
                JointType::Revolute {
                    anchor: [0.0, 0.0],
                    limits: None,
                },
            )
        });
        world.step();
    }
}

// ---------------------------------------------------------------------------
// Queries (2D)
// ---------------------------------------------------------------------------

/// Raycasts must respect box rotation.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn raycast_hits_rotated_box() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let b = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        rotation: std::f64::consts::FRAC_PI_2,
        ..BodyDesc::default()
    });
    world.add_collider(b, boxed([2.0, 0.1, 0.0]));
    // Rotated 90°, the box spans x ∈ [-0.1, 0.1], y ∈ [-2, 2].
    let hit = world.raycast([5.0, 1.5, 0.0], [-1.0, 0.0, 0.0], 100.0);
    let hit = hit.expect("ray should hit rotated box");
    assert!((hit.distance - 4.9).abs() < 1e-9, "t = {}", hit.distance);
    // And a point the unrotated box would contain must miss.
    let miss = world.raycast([1.5, 5.0, 0.0], [0.0, -1.0, 0.0], 100.0);
    assert!(miss.is_none(), "hit: {miss:?}");
}

/// Overlap queries must respect box rotation.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn overlap_sphere_respects_rotation() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let b = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        rotation: std::f64::consts::FRAC_PI_2,
        ..BodyDesc::default()
    });
    world.add_collider(b, boxed([2.0, 0.1, 0.0]));
    assert!(world.overlap_sphere([1.5, 0.0, 0.0], 0.05).is_empty());
    assert_eq!(world.overlap_sphere([0.0, 1.5, 0.0], 0.05).len(), 1);
}

// ---------------------------------------------------------------------------
// Sleep (2D)
// ---------------------------------------------------------------------------

/// A sleeping body must not keep drifting.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn sleeping_body_does_not_drift() {
    let mut world = PhysicsWorld::new(zero_g());
    let b = world.add_body(BodyDesc {
        linear_velocity: [0.005, 0.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.5));
    for _ in 0..120 {
        world.step();
    }
    let s1 = world.get_body_state(b).unwrap();
    assert!(s1.is_sleeping, "body should be asleep");
    for _ in 0..120 {
        world.step();
    }
    let s2 = world.get_body_state(b).unwrap();
    assert_eq!(s1.position, s2.position);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// After restore, newly allocated handles must match the original world.
#[cfg(all(feature = "serialize", any(feature = "2d", feature = "3d")))]
#[test]
#[ignore = "needs arena free-list metadata in snapshots (format change planned for 2.0)"]
fn restore_preserves_handle_allocation() {
    let mut w1 = PhysicsWorld::new(WorldConfig::default());
    let hs: Vec<_> = (0..4)
        .map(|i| {
            w1.add_body(BodyDesc {
                position: [i as f64 * 3.0, 0.0, 0.0],
                ..BodyDesc::default()
            })
        })
        .collect();
    w1.remove_body(hs[0]).unwrap();
    w1.remove_body(hs[2]).unwrap();
    let bytes = serialize::serialize_world(&w1).unwrap();
    let mut w2 = PhysicsWorld::new(WorldConfig::default());
    serialize::deserialize_world(&mut w2, &bytes).unwrap();
    assert_eq!(
        w1.add_body(BodyDesc::default()),
        w2.add_body(BodyDesc::default())
    );
    assert_eq!(
        w1.add_body(BodyDesc::default()),
        w2.add_body(BodyDesc::default())
    );
}

/// A snapshot with an absurd handle index must be rejected, not allocate
/// billions of slots; duplicate handles and orphan colliders are rejected too.
#[cfg(all(feature = "serialize", any(feature = "2d", feature = "3d")))]
#[test]
fn restore_rejects_huge_handle_index() {
    let mut w1 = PhysicsWorld::new(WorldConfig::default());
    w1.add_body(BodyDesc::default());
    let mut snap = w1.snapshot();
    snap.bodies[0].handle = BodyHandle(0xFFFF_FFF0);
    let mut w2 = PhysicsWorld::new(WorldConfig::default());
    assert!(w2.try_restore(&snap).is_err());
    assert_eq!(w2.body_count(), 0, "world must be unchanged on error");

    let mut w1 = PhysicsWorld::new(WorldConfig::default());
    let b = w1.add_body(BodyDesc::default());
    w1.add_collider(b, ball(0.5));
    let mut dup = w1.snapshot();
    dup.colliders.push(dup.colliders[0].clone());
    assert!(w2.try_restore(&dup).is_err());
    let mut orphan = w1.snapshot();
    orphan.colliders[0].body = BodyHandle(99);
    assert!(w2.try_restore(&orphan).is_err());
    assert!(w2.try_restore(&w1.snapshot()).is_ok());
    assert_eq!(w2.body_count(), 1);
}

/// Restore must replace force fields too.
#[cfg(all(feature = "serialize", any(feature = "2d", feature = "3d")))]
#[test]
#[ignore = "needs force fields in snapshots (format change planned for 2.0)"]
fn restore_replaces_force_fields() {
    let mut w1 = PhysicsWorld::new(WorldConfig::default());
    w1.add_force_field(ForceField::Directional {
        force: [1.0, 0.0, 0.0],
        min: [-1.0; 3],
        max: [1.0; 3],
    });
    let bytes = serialize::serialize_world(&w1).unwrap();
    let mut w2 = PhysicsWorld::new(WorldConfig::default());
    w2.add_force_field(ForceField::Radial {
        center: [0.0; 3],
        strength: 1.0,
        falloff: 2.0,
        radius: 1.0,
    });
    serialize::deserialize_world(&mut w2, &bytes).unwrap();
    assert_eq!(w2.force_fields(), w1.force_fields());
}

// ---------------------------------------------------------------------------
// Contacts (both backends)
// ---------------------------------------------------------------------------

/// A ball dropped onto a box floor must come to rest on top of it, at either
/// handle order (floor created first or second).
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn ball_rests_on_box_floor_either_order() {
    for floor_first in [true, false] {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let make_floor = |w: &mut PhysicsWorld| {
            let f = w.add_body(BodyDesc {
                body_type: BodyType::Static,
                ..BodyDesc::default()
            });
            w.add_collider(f, boxed([10.0, 0.5, 10.0]));
        };
        if floor_first {
            make_floor(&mut world);
        }
        let b = world.add_body(BodyDesc {
            position: [3.0, 3.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(b, ball(0.5));
        if !floor_first {
            make_floor(&mut world);
        }
        for _ in 0..300 {
            world.step();
        }
        let s = world.get_body_state(b).unwrap();
        assert!(
            (s.position[1] - 1.0).abs() < 0.05,
            "floor_first={floor_first}: y = {}",
            s.position[1]
        );
        assert!(s.linear_velocity[1].abs() < 0.1);
    }
}

/// A perfectly elastic ball hitting a box floor must bounce back up.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn elastic_ball_bounces_off_box() {
    let mut world = PhysicsWorld::new(zero_g());
    let f = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(
        f,
        ColliderDesc {
            material: PhysicsMaterial {
                restitution: 1.0,
                ..PhysicsMaterial::default()
            },
            ..boxed([10.0, 0.5, 10.0])
        },
    );
    let b = world.add_body(BodyDesc {
        position: [0.0, 1.05, 0.0],
        linear_velocity: [0.0, -10.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(
        b,
        ColliderDesc {
            material: PhysicsMaterial {
                restitution: 1.0,
                ..PhysicsMaterial::default()
            },
            ..ball(0.5)
        },
    );
    for _ in 0..3 {
        world.step();
    }
    let vy = world.get_body_state(b).unwrap().linear_velocity[1];
    assert!(vy > 9.0, "vy = {vy}");
}

/// An elastic ball hitting a static ball must bounce with the same speed.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn elastic_ball_bounces_off_ball() {
    let mut world = PhysicsWorld::new(zero_g());
    let bouncy = |r| ColliderDesc {
        material: PhysicsMaterial {
            restitution: 1.0,
            ..PhysicsMaterial::default()
        },
        ..ball(r)
    };
    let f = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(f, bouncy(0.5));
    let b = world.add_body(BodyDesc {
        position: [0.0, 1.05, 0.0],
        linear_velocity: [0.0, -10.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, bouncy(0.5));
    for _ in 0..3 {
        world.step();
    }
    let vy = world.get_body_state(b).unwrap().linear_velocity[1];
    assert!(vy > 9.0, "vy = {vy}");
}

// ---------------------------------------------------------------------------
// Joints and stacking (2D behaviour)
// ---------------------------------------------------------------------------

/// A box pinned at one end by a revolute joint must swing under gravity
/// (joint impulses need angular terms).
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn pinned_box_swings() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let pin = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let rod = world.add_body(BodyDesc {
        position: [1.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(rod, boxed([1.0, 0.05, 0.0]));
    world.add_joint(JointDesc {
        local_anchor_b: [-1.0, 0.0],
        ..joint(
            pin,
            rod,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
        )
    });
    for _ in 0..30 {
        world.step();
    }
    let s = world.get_body_state(rod).unwrap();
    assert!(s.rotation < -0.3, "rotation = {}", s.rotation);
    // The pinned end stays at the pivot.
    let end = [
        s.position[0] - s.rotation.cos(),
        s.position[1] - s.rotation.sin(),
    ];
    assert!(end[0].hypot(end[1]) < 0.02, "pivot drift = {end:?}");
}

/// A fixed joint must hold relative rotation against an applied torque.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn fixed_joint_locks_rotation() {
    let mut world = PhysicsWorld::new(zero_g());
    let a = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let b = world.add_body(BodyDesc {
        position: [1.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, boxed([0.5, 0.5, 0.0]));
    world.add_joint(JointDesc {
        local_anchor_a: [1.0, 0.0],
        ..joint(a, b, JointType::Fixed)
    });
    for _ in 0..30 {
        world.apply_torque(b, Torque::new(10.0));
        world.step();
    }
    let s = world.get_body_state(b).unwrap();
    assert!(s.rotation.abs() < 0.01, "rotation = {}", s.rotation);
    assert!((s.position[0] - 1.0).abs() < 0.01, "x = {}", s.position[0]);
}

/// A ball hanging on a distance joint settles at the joint length.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn distance_joint_hangs_at_length() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let a = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 10.0, 0.0],
        ..BodyDesc::default()
    });
    let b = world.add_body(BodyDesc {
        position: [0.0, 8.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.25));
    world.add_joint(joint(a, b, JointType::Distance { length: 2.0 }));
    for _ in 0..600 {
        world.step();
    }
    let s = world.get_body_state(b).unwrap();
    assert!(
        (10.0 - s.position[1] - 2.0).abs() < 0.01,
        "y = {}",
        s.position[1]
    );
    assert!(
        s.linear_velocity[1].abs() < 0.1,
        "v = {:?}",
        s.linear_velocity
    );
}

/// A revolute motor drives the relative angular velocity to its target.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn revolute_motor_reaches_target_speed() {
    let mut world = PhysicsWorld::new(zero_g());
    let a = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    let b = world.add_body(BodyDesc::default());
    world.add_collider(b, boxed([0.5, 0.5, 0.0]));
    world.add_joint(JointDesc {
        motor: Some(JointMotor {
            target_velocity: 2.0,
            max_force: 1000.0,
        }),
        ..joint(
            a,
            b,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
        )
    });
    for _ in 0..10 {
        world.step();
    }
    let w = world.get_body_state(b).unwrap().angular_velocity;
    assert!((w - 2.0).abs() < 1e-6, "w = {w}");
}

/// A mouse joint drags its body to the target.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn mouse_joint_reaches_target() {
    let mut world = PhysicsWorld::new(zero_g());
    let b = world.add_body(BodyDesc::default());
    world.add_collider(b, ball(0.25));
    world.add_joint(joint(
        b,
        b,
        JointType::Mouse {
            target: [3.0, 1.0, 0.0],
            stiffness: 200.0,
            damping: 30.0,
            max_force: 1000.0,
        },
    ));
    for _ in 0..240 {
        world.step();
    }
    let p = world.get_body_state(b).unwrap().position;
    assert!(
        (p[0] - 3.0).abs() < 0.01 && (p[1] - 1.0).abs() < 0.01,
        "p = {p:?}"
    );
}

/// A joint breaks once its constraint force exceeds `break_force` (newtons).
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn joint_breaks_under_load() {
    let make = |break_force| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let a = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 10.0, 0.0],
            ..BodyDesc::default()
        });
        let b = world.add_body(BodyDesc {
            position: [0.0, 9.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            b,
            ColliderDesc {
                mass: Some(1.0),
                ..ball(0.25)
            },
        );
        world.add_joint(JointDesc {
            break_force: Some(break_force),
            ..joint(a, b, JointType::Distance { length: 1.0 })
        });
        for _ in 0..30 {
            world.step();
        }
        world.get_body_state(b).unwrap().position[1]
    };
    // Load is ~9.81 N: a 5 N joint breaks and the ball falls; 50 N holds.
    assert!(make(5.0) < 8.0);
    assert!((make(50.0) - 9.0).abs() < 0.02);
}

/// A stack of boxes stays upright and comes to rest.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn box_stack_is_stable() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(ground, boxed([10.0, 0.5, 0.0]));
    let boxes: Vec<_> = (0..6)
        .map(|i| {
            let b = world.add_body(BodyDesc {
                position: [0.0, 1.0 + i as f64 * 1.0, 0.0],
                ..BodyDesc::default()
            });
            world.add_collider(b, boxed([0.5, 0.5, 0.0]));
            b
        })
        .collect();
    for _ in 0..600 {
        world.step();
    }
    for (i, b) in boxes.iter().enumerate() {
        let s = world.get_body_state(*b).unwrap();
        assert!(s.position[0].abs() < 0.05, "box {i} x = {}", s.position[0]);
        assert!(s.rotation.abs() < 0.05, "box {i} rot = {}", s.rotation);
        let expected_y = 1.0 + i as f64;
        assert!(
            (s.position[1] - expected_y).abs() < 0.1,
            "box {i} y = {}",
            s.position[1]
        );
    }
}

/// A box resting off-centre on a wide ground box must not spin.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn off_centre_box_rests_without_spin() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(ground, boxed([10.0, 0.5, 10.0]));
    let b = world.add_body(BodyDesc {
        position: [8.0, 1.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, boxed([0.5, 0.5, 0.5]));
    for _ in 0..300 {
        world.step();
    }
    let s = world.get_body_state(b).unwrap();
    assert!((s.position[0] - 8.0).abs() < 0.05, "x = {}", s.position[0]);
    assert!((s.position[1] - 1.0).abs() < 0.05, "y = {}", s.position[1]);
    assert!(s.rotation.abs() < 0.05, "rot = {}", s.rotation);
}

/// With `constraint_frequency = 0` (Baumgarte), penetration is still
/// resolved at `position_correction` per step.
#[cfg(any(feature = "2d", feature = "3d"))]
#[test]
fn baumgarte_mode_resolves_penetration() {
    let mut world = PhysicsWorld::new(WorldConfig {
        constraint_frequency: 0.0,
        ..zero_g()
    });
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(ground, boxed([10.0, 0.5, 10.0]));
    let b = world.add_body(BodyDesc {
        position: [0.0, 0.9, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(b, ball(0.5));
    for _ in 0..60 {
        world.step();
    }
    let y = world.get_body_state(b).unwrap().position[1];
    assert!(y > 0.98, "y = {y}");
}

/// Removing the support under a sleeping body wakes it so it falls.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn removing_support_wakes_sleepers() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let ground = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..BodyDesc::default()
    });
    world.add_collider(ground, boxed([10.0, 0.5, 0.0]));
    let plank = world.add_body(BodyDesc {
        position: [0.0, 1.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(plank, boxed([2.0, 0.5, 0.0]));
    let top = world.add_body(BodyDesc {
        position: [0.0, 2.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(top, boxed([0.5, 0.5, 0.0]));
    for _ in 0..240 {
        world.step();
    }
    assert!(
        world.get_body_state(top).unwrap().is_sleeping,
        "stack should sleep"
    );
    let y0 = world.get_body_state(top).unwrap().position[1];
    world.remove_body(plank).unwrap();
    for _ in 0..60 {
        world.step();
    }
    let y1 = world.get_body_state(top).unwrap().position[1];
    assert!(y1 < y0 - 0.5, "y0 = {y0}, y1 = {y1}");
}

/// Kinematic bodies follow their velocity under the XPBD solver too.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[test]
fn xpbd_moves_kinematic_bodies() {
    let mut world = PhysicsWorld::new(WorldConfig {
        solver: SolverKind::Xpbd,
        ..WorldConfig::default()
    });
    let k = world.add_body(BodyDesc {
        body_type: BodyType::Kinematic,
        linear_velocity: [1.0, 0.0, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(k, boxed([0.5, 0.5, 0.0]));
    for _ in 0..60 {
        world.step();
    }
    let x = world.get_body_state(k).unwrap().position[0];
    assert!((x - 1.0).abs() < 1e-9, "x = {x}");
}
