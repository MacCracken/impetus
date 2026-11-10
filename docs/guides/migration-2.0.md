# Moving from impetus 1.4 (Rust) to 2.0 (Cyrius)

impetus 2.0 is the Cyrius port of the Rust crate 1.4.0. The Rust crate stays at 1.4.0, frozen in `rust-old/` as
the parity oracle. This guide is for code that used the Rust crate (kiran, joshua and aethersafha did): what to
declare, how each Rust idiom reads in Cyrius, and the seven deliberate changes
([ADR 0008](../adr/0008-port-api-divergences.md)), each with the code that handles it.

The physics doesn't change. For the same inputs, a 2.0 world steps to the bits Rust 1.4.0 computes, with the one
exception in [What stays the same](#what-stays-the-same).

Every `cyrius` block below compiles and runs against the shipped bundles: `python3 -I scripts/doc_examples.py .`
wraps each fragment in a `main`, includes the bundle the fragment names (the 2D one unless its first line says
3D), and fails on a compile warning or a nonzero exit.

## 1. Declare the dependency

```toml
# your cyrius.cyml
[deps.impetus]
git = "https://github.com/MacCracken/impetus.git"
tag = "2.0.0"
modules = ["dist/impetus.cyr"]        # the 2D profile; "dist/impetus-3d.cyr" for 3D
```

`cyrius deps` vendors the bundle and the stdlib modules it needs, which are listed in `dist/impetus.deps`.
Cargo's features become a choice of bundle:

| Cargo feature (1.4.0) | 2.0 |
|---|---|
| `2d` (the default) | `dist/impetus.cyr` |
| `3d`, `full` | `dist/impetus-3d.cyr` |
| `serialize` | always on: both bundles have snapshots, their binary form and JSON |
| `oracle` | not shipped; it exists only in `rust-old/` to drive the golden generator |

A program uses one bundle. Both define the same names (`ImPhysicsWorld`, `ImBodyState`, …), as Cargo's two
features gave one backend per build. In 1.4.0, enabling both `2d` and `3d` picked 3D; in 2.0 you name the bundle.

## 2. Rust idioms in Cyrius

The names are Rust's. Structs are `Im<Name>` with methods (`world.add_body(desc)`), constructors are
`Im<Name>_<constructor>` (`ImSpring2d_critically_damped`), and enum members and constants carry `IM_`.

| Rust 1.4.0 | Cyrius 2.0 |
|---|---|
| `PhysicsWorld::new(WorldConfig::default())` | `ImPhysicsWorld_new(ImWorldConfig_default())` |
| `Drop` | explicit: `world.free()`, and `free()` on each snapshot, `ImWordList` and array-holding shape you make |
| `BodyDesc { position: [0.0, 10.0, 0.0], ..Default::default() }` | `var d: ImBodyDesc = ImBodyDesc_default();` then `d.position = ImVec3_new(0.0, 10.0, 0.0);` |
| `[f64; 3]`, `[f64; 2]` | `ImVec3`, `ImVec2` values |
| `BodyHandle(u64)`, `ColliderHandle(u64)`, … | a word holding the same 64 bits |
| `BodyType::Dynamic` | `IM_BODY_DYNAMIC` |
| `ColliderShape::Ball { radius: 0.5 }` | `ImColliderShape_ball(0.5)` (`_box`, `_capsule`, `_segment`, `_convex_hull`, …) |
| `JointType::Rope { max_length: 2.0 }` | `ImJointType_rope(2.0)` |
| `PhysicsMaterial::rubber()` | `ImPhysicsMaterial_rubber()` |
| `Result<T, ImpetusError>` | a value-form `Result`: `var t, v = f(..);` then `is_ok(t)`; on `Err`, `v` is an `IM_ERR_*` code |
| `ImpetusError::BodyNotFound(String)` | `IM_ERR_BODY_NOT_FOUND`: the code alone. `im_error_message(code)` gives fixed text |
| `Option<RayHit>` from `raycast` | `world.raycast(origin, dir, max, &hit)`: 1 with `hit` filled, or 0 |
| `Vec<ColliderHandle>` from `overlap_sphere` | `world.overlap_sphere(centre, r, &list)` fills an `ImWordList` you own (1, or 0 with no memory) |
| `&[CollisionEvent]` | `var ev, n = world.collision_events();`: `n` `ImCollisionEvent` records at `ev` |
| `serde_json::to_string(&desc)` | `desc.to_json(sb)` onto a `str_builder` |
| `serde_json::from_str::<BodyDesc>(s)` | `ImBodyDesc_from_json(buf, len, &out)` |

A world, its queries and its events. Cyrius has no cast, so a record list returned as a pointer and a count
(`collision_events`, `particles`, `force_fields`) is read through a one-line function whose return type is the
typed pointer:

```cyrius
include "lib/impetus.cyr"

# The k-th of the records at ev.
fn event_at(ev, k): *ImCollisionEvent { return ev + k * sizeof(ImCollisionEvent); }

fn main(): i64 {
    alloc_init();
    var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
    var fd: ImBodyDesc = ImBodyDesc_default();
    fd.body_type = IM_BODY_STATIC;
    var floor = world.add_body(fd);
    var tf, cf = world.add_collider(floor, ImColliderDesc_new(ImColliderShape_box(10.0, 0.5, 10.0)));
    var bd: ImBodyDesc = ImBodyDesc_default();
    bd.position = ImVec3_new(0.0, 0.9, 0.0);
    var ball = world.add_body(bd);
    var tb, cb = world.add_collider(ball, ImColliderDesc_new(ImColliderShape_ball(0.5)));
    if (is_ok(tf) + is_ok(tb) != 2) { return 1; }
    world.step();

    # Option<RayHit>: 1 and the hit, or 0.
    var hit: ImRayHit;
    if (world.raycast(ImVec3_new(0.0, 5.0, 0.0), ImVec3_new(0.0, -1.0, 0.0), 10.0, &hit) == 0) { return 1; }
    var dist: f64 = hit.distance;                       # and hit.collider, hit.point, hit.normal
    if (hit.collider != cb) { return 1; }

    # Vec<ColliderHandle>: a word list you own.
    var near: ImWordList = ImWordList_new();
    if (world.overlap_sphere(ImVec3_new(0.0, 0.0, 0.0), 2.0, &near) == 0) { return 1; }
    var touched = 0;
    for (var i = 0; i < near.len; i = i + 1) {
        if (near.at(i) == cb) { touched = 1; }
    }
    near.free();

    # &[CollisionEvent]: a pointer and a count.
    var ev, n = world.collision_events();
    var started = 0;
    for (var k = 0; k < n; k = k + 1) {
        var e: *ImCollisionEvent = event_at(ev, k);
        if (e.kind == IM_EVENT_STARTED) { started = started + 1; }
    }
    world.free();
    if (touched == 0 || started != 1) { return 1; }
    return 0;
}

var r = main();
sys_exit_group(r);
```

## 3. The seven changes

### #1 `add_collider` and `add_joint` refuse a missing body

Rust 1.4.0 stored the collider or joint anyway, as an orphan, and returned its handle. 2.0 stores nothing and
returns `Err(IM_ERR_BODY_NOT_FOUND)`. Code that ignored the return still compiles, so look for the call sites:
a discarded `Result` is not a build error.

```cyrius
var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
var body = world.add_body(ImBodyDesc_default());
var t, collider = world.add_collider(body, ImColliderDesc_new(ImColliderShape_ball(0.5)));
if (is_ok(t) == 0) { return 1; }

var tr, er = world.remove_body(body);
var t2, e2 = world.add_collider(body, ImColliderDesc_new(ImColliderShape_ball(0.5)));
if (is_ok(t2) == 1 || e2 != IM_ERR_BODY_NOT_FOUND) { return 1; }     # Rust stored an orphan here
var joint: ImJointDesc = ImJointDesc_new(body, body, ImJointType_fixed(), ImVec2_new(0.0, 0.0), ImVec2_new(0.0, 0.0));
var t3, e3 = world.add_joint(joint);
if (is_ok(t3) == 1 || e3 != IM_ERR_BODY_NOT_FOUND) { return 1; }
world.free();
```

### #2 and #3 One fallible `restore`, and snapshots that hold the whole world

Rust 1.4.0 had `restore` (deprecated) beside `try_restore`, and a bitcode `WorldSnapshot` that left out the force
fields, the contact manifolds and their warm-start impulses, sleep timers, the arena slots and the contact tracker.
So a restored world stepped differently and handed out different handles.

2.0 has one `restore`, which returns a `Result` and leaves the world unchanged on any error. Its snapshot holds the
world's whole state ([ADR 0013](../adr/0013-complete-snapshots.md)). A restored world is the original: it steps to
the same bits and hands out the same handles, on every target. The bytes are a checksummed, versioned format of
little-endian 64-bit words, the same on every target.

```cyrius
var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
var ball = world.add_body(ImBodyDesc_default());
var tc, c = world.add_collider(ball, ImColliderDesc_new(ImColliderShape_ball(0.5)));

# Rust: let bytes = serialize_world(&world)?;
var bytes: ImWordList = ImWordList_new();
var ts, es = serialize_world(&world, &bytes);
if (is_ok(ts) == 0) { return 1; }

# Rust: deserialize_world(&mut world, &bytes)?;  The byte count is the word count × 8.
world.step();
var tl, el = deserialize_world(&world, bytes.data, bytes.len * 8);
if (is_ok(tl) == 0) { return 1; }     # el: IM_ERR_DESERIALIZE, IM_ERR_INVALID_CONFIG or IM_ERR_UNSUPPORTED
if (world.current_step() != 0) { return 1; }
bytes.free();
world.free();
```

The snapshot is also a struct you can read and edit, and it has a JSON form:

```cyrius
var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
var snap: ImWorldSnapshot = ImWorldSnapshot_new();
var t1, e1 = world.snapshot(&snap);                # Rust: let snap = world.snapshot();
var t2, e2 = world.restore(&snap);                 # Rust: world.try_restore(&snap)?;

var sb = str_builder_new();
var t3, e3 = snap.to_json(sb);
var text = str_builder_build(sb);
var back: ImWorldSnapshot = ImWorldSnapshot_new();
var t4, e4 = ImWorldSnapshot_from_json(str_data(text), str_len(text), &back);
if (is_ok(t1) + is_ok(t2) + is_ok(t3) + is_ok(t4) != 4) { return 1; }
back.free();
snap.free();
world.free();
```

**Saved 1.4.0 snapshots can't be read.** Neither bitcode bytes nor 1.4.0's JSON of `WorldSnapshot` has a Cyrius
reader, and neither held enough to restore exactly. Rebuild those worlds once from your own scene data, then save
2.0 snapshots.

### #4 The 3D `BodyState` holds the full orientation

In Rust 1.4.0 the 3D `BodyState` kept only the rotation about z (`rotation: f64`, `angular_velocity: f64`). In the
2.0 3D profile they are `rotation: ImQuat`, a unit quaternion (x, y, z, w), and `angular_velocity: ImVec3`. The 2D
`ImBodyState` is unchanged. Rust's call with angle θ and spin ω is this call, bit for bit:

```cyrius
# 3D profile (dist/impetus-3d.cyr)
var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
var body = world.add_body(ImBodyDesc_default());
var tc, c = world.add_collider(body, ImColliderDesc_new(ImColliderShape_box(0.5, 0.5, 0.5)));
var st: ImBodyState;
var tg, eg = world.get_body_state(body, &st);
st.rotation = ImQuat_from_rotation_z(0.3);           # Rust 1.4.0: state.rotation = 0.3
st.angular_velocity = ImVec3_new(0.0, 0.0, 2.0);     # Rust 1.4.0: state.angular_velocity = 2.0
var ts, es = world.set_body_state(body, st);
if (is_ok(tc) + is_ok(tg) + is_ok(ts) != 3) { return 1; }
world.free();
```

`set_body_state` stores the quaternion as given, without normalizing it. Its JSON follows the struct:
`"rotation":[x,y,z,w]` and `"angular_velocity":[x,y,z]`. A 1.4.0 3D document, with a number in each, is refused
with `IM_ERR_DESERIALIZE`.

### #5 The 3D profile refuses Revolute, Prismatic and motors

Rust 1.4.0's 3D world accepted them and did nothing with them. 2.0's 3D `add_joint` returns
`Err(IM_ERR_UNSUPPORTED)`, and `restore` refuses a snapshot that holds one. Since they never had an effect, drop
them from 3D scenes. The 2D profile has all three, as before.

```cyrius
# 3D profile (dist/impetus-3d.cyr)
var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());
var a = world.add_body(ImBodyDesc_default());
var b = world.add_body(ImBodyDesc_default());
var hinge: ImJointType = ImJointType_revolute(ImVec2_new(0.0, 0.0), 0, 0.0, 0.0);
var t, e = world.add_joint(ImJointDesc_new(a, b, hinge, ImVec2_new(0.5, 0.0), ImVec2_new(-0.5, 0.0)));
if (is_ok(t) == 1 || e != IM_ERR_UNSUPPORTED) { return 1; }
world.free();
```

### #6 The XPBD solver is refused

`SolverKind::Xpbd` was 2D-only and experimental. 2.0's `ImPhysicsWorld_try_new` and `restore` refuse it with
`Err(IM_ERR_UNSUPPORTED)`. As in Rust, `ImPhysicsWorld_new` doesn't validate its config, and a world made that way
with `IM_SOLVER_XPBD` steps as sequential impulses. So build from a config you didn't write yourself with
`try_new`.

```cyrius
var cfg: ImWorldConfig = ImWorldConfig_default();
cfg.solver = IM_SOLVER_XPBD;
var world: ImPhysicsWorld;
var t, e = ImPhysicsWorld_try_new(cfg, &world);
if (is_ok(t) == 1 || e != IM_ERR_UNSUPPORTED) { return 1; }
```

### #7 Bit-identical across targets

This one needs no code change. Rust 1.4.0 promised the same bits for the same inputs only on the same platform.
2.0 gives the same bits on x86_64 Linux, x86_64 Windows and aarch64, because every transcendental the step calls is
impetus's own and correctly rounded ([ADR 0011](../adr/0011-correctly-rounded-exp-sin-cos.md)). A snapshot saved
on one target restores on the others and steps on to the same bits.

## What stays the same

- **The physics.** The golden corpus replays Rust 1.4.0's own results: single steps from full world dumps, multi-
  frame world runs, every narrowphase pair, ray and query, and the joint, particle and spring paths, on both
  profiles. The exception is where glibc misrounds one of `exp`, `sin`, `cos`, `hypot`, `pow` or `atan2`
  (Rust's std calls glibc). There 2.0 computes the correctly rounded value, so a world can drift from Rust's from
  that step on. The rows where that happens are listed as corrected in
  [port-parity.md](../development/port-parity.md); they are rare.
- **Defaults and constants** are Rust's, generated from the oracle.
- **JSON** documents are what serde_json writes and reads for the same values
  ([ADR 0010](../adr/0010-serde-compatible-json.md)), apart from the 3D `BodyState` (#4) and the limits below.
- **What Rust 1.4.0 left undone stays undone:** heightfield and trimesh colliders generate no contacts, the 3D world
  has no islands or island sleep, and the 3D `overlap_sphere` and `overlap_aabb` return no colliders.

## Not carried over

- XPBD (#6), the deprecated `restore` without a `Result` (#2), and bitcode and 1.4.0's snapshot JSON (#3).
- The `oracle` feature (port tooling, never public API).
- The bridges to dravya, sharira and ushma (`bridge.rs`). They come back, with their Rust tests, once a Cyrius
  consumer needs them (port plan §9).

## Known limitations

- **JSON integers above `i64::MAX`.** bayan refuses them (filed upstream as S3), so a handle whose slot has been
  reused more than 2^31 times is written to JSON but can't be read back. The binary form holds it.
- **Duplicate JSON keys.** bayan refuses any document with a repeated key (S4). serde ignores repeats of unknown
  fields, so a few documents serde reads are refused here.
- **Out of memory while writing JSON.** The stdlib's `str_builder` exits the process when it can't grow. That's the
  one place 2.0 can abort, and only in the JSON writers.
- **Speed.** The median benchmark is about 10× Rust's in 2D and 13× in 3D; a whole step is 2.6–11× and 5.7–17×
  ([the record](../benchmarks-rust-v-cyrius.md)). These are the port's first numbers. The upstream issues that
  explain them are filed.
