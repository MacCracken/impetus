"""The benchmark record: every Rust benchmark beside its Cyrius twin, on both profiles.

Port plan M7 (part 2). Usage, from the repo root:
    python3 -I scripts/bench_record.py . --check
        exit 1 unless every bench in rust-old/benches/simulation.rs has a twin (or a recorded reason) on each
        profile, and every twin's name is a bench in tests/impetus.bcyr or tests/impetus_3d.bcyr
    python3 -I scripts/bench_record.py . RUST2D.log RUST3D.log CYR2D.log CYR3D.log
        print the markdown table from four logs: criterion's output for Rust's 2D build (`cargo bench --features
        serialize --bench simulation`) and its 3D build (`--all-features`), and `cyrius bench` on each bench file

A profile-free row (forces, materials, serde, springs) is the same code on both profiles, so it has one twin.
"""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
args = sys.argv[2:]

# Rust bench -> (2D twin, 3D twin). A 3D twin of None is a profile-free row; a string starting "n/a:" is a reason.
TWINS = {
    "world/step_empty": ("world/step_empty", "b3/step_empty"),
    "world/step_10_bodies": ("world/step_10_bodies", "b3/step_10_bodies"),
    "world/step_100_bodies": ("world/step_100_bodies", "b3/step_100_bodies"),
    "world/step_1000_bodies": ("world/step_1000_bodies", "b3/step_1000_bodies"),
    "bodies/add_body": ("bodies/add_body", "bodies/add_body (3D)"),
    "bodies/add_remove_body": ("bodies/add_remove_body", "bodies/add_remove_body (3D)"),
    "bodies/add_body_with_collider": ("bodies/add_body_with_collider", "bodies/add_body_with_collider (3D)"),
    "bodies/add_joint": ("bodies/add_joint", "bodies/add_joint (3D)"),
    "forces/apply_force": ("forces/apply_force", "forces/apply_force (3D)"),
    "forces/apply_impulse": ("forces/apply_impulse", "forces/apply_impulse (3D)"),
    "forces/force_magnitude": ("forces/force_magnitude", None),
    "forces/gravity_force": ("forces/gravity_force", None),
    "materials/preset_steel": ("materials/preset_steel", None),
    "materials/preset_rubber": ("materials/preset_rubber", None),
    "materials/quantity_display": ("materials/quantity_display", None),
    "materials/degrees_to_radians": ("materials/degrees_to_radians", None),
    "materials/quantity_new": ("materials/quantity_new", None),
    "serde/serialize_body_desc": ("serde/serialize_body_desc", None),
    "serde/deserialize_body_desc": ("serde/deserialize_body_desc", None),
    "serde/serialize_config": ("serde/serialize_config", None),
    "serde/roundtrip_collider_desc": ("serde/roundtrip_collider_desc", None),
    "particles/step_100_particles": ("particles/step_100_particles", "b3/particles/step_100_particles"),
    "particles/step_1000_particles": ("particles/step_1000_particles", "b3/particles/step_1000_particles"),
    "particles/step_100_particles_with_floor": ("particles/step_100_particles_with_floor",
                                                "b3/particles/step_100_particles_with_floor"),
    "particles/spawn_particle": ("particles/spawn_particle", "b3/particles/spawn_particle"),
    "particles/emitter_step": ("particles/emitter_step", "b3/particles/emitter_step"),
    "snapshot/serialize_empty": ("snapshot/serialize_empty", "b3/snapshot/serialize_empty"),
    "snapshot/serialize_100_bodies": ("snapshot/serialize_100_bodies", "b3/snapshot/serialize_100_bodies"),
    "snapshot/roundtrip_100_bodies": ("snapshot/roundtrip_100_bodies", "b3/snapshot/roundtrip_100_bodies"),
    "queries/raycast_100_bodies": ("queries/raycast_100_bodies", "queries/raycast_100_bodies (3D)"),
    "queries/overlap_sphere_100_bodies": ("queries/overlap_sphere_100_bodies",
                                          "b3/queries/overlap_sphere_100_bodies"),
    "mutation/get_body_state": ("mutation/get_body_state", "b3/mutation/get_body_state"),
    "mutation/set_body_state": ("mutation/set_body_state", "b3/mutation/set_body_state"),
    "mutation/set_body_type": ("mutation/set_body_type", "b3/mutation/set_body_type"),
    "springs/spring_1d_step": ("springs/spring_1d_step", None),
    "springs/spring_2d_step": ("springs/spring_2d_step", None),
    "springs/spring_settle_frames": ("springs/spring_settle_frames", None),
    "substep/step_100_bodies_substep4": ("substep/step_100_bodies_substep4", "b3/step_100_bodies_substep4"),
    "stacking/box_stack_10": ("stacking/box_stack_10", "b3/stacking/box_stack_10"),
    "joints/revolute_chain_20": ("joints/revolute_chain_20",
                                 "n/a: the 3D profile refuses Revolute (ADR 0008 #5); Rust's 3D build steps the "
                                 "links unjoined"),
    "broadphase/aabb_tree_1000_row": ("broadphase/aabb_tree_1000_row", "b3/broadphase/aabb_tree_1000_row"),
}


def rust_benches():
    """The group/name of each bench_function in rust-old/benches/simulation.rs, in file order."""
    out = []
    group = None
    for line in (root / "rust-old/benches/simulation.rs").read_text().split("\n"):
        m = re.search(r'benchmark_group\("(\w+)"\)', line)
        if m:
            group = m.group(1)
        m = re.search(r'bench_function\("(\w+)"', line)
        if m:
            out.append(f"{group}/{m.group(1)}")
    return out


def cyrius_names(rel):
    return set(re.findall(r'"([a-z0-9_]+/[^"]+)"', (root / rel).read_text()))


def check():
    errors = []
    benches = rust_benches()
    have2 = cyrius_names("tests/impetus.bcyr") | cyrius_names("tests/bench_cases.cyr")
    have3 = cyrius_names("tests/impetus_3d.bcyr")
    for b in benches:
        if b not in TWINS:
            errors.append(f"{b} has no TWINS entry")
            continue
        t2, t3 = TWINS[b]
        if t2 not in have2:
            errors.append(f"{b}: 2D twin {t2!r} is not a bench name in tests/impetus.bcyr")
        if t3 is not None and not t3.startswith("n/a:") and t3 not in have3:
            errors.append(f"{b}: 3D twin {t3!r} is not a bench name in tests/impetus_3d.bcyr")
    for b in TWINS:
        if b not in benches:
            errors.append(f"TWINS names {b}, which is not a Rust bench")
    for e in errors:
        print("BENCH", e)
    n3 = sum(1 for t2, t3 in TWINS.values() if t3 is not None and not t3.startswith("n/a:"))
    print(f"bench record: {len(benches)} Rust benches, {len(benches)} 2D twins, {n3} 3D twins, "
          f"{sum(1 for _, t3 in TWINS.values() if t3 is None)} profile-free; {len(errors)} problem(s)")
    return 1 if errors else 0


UNITS = {"ps": 0.001, "ns": 1.0, "us": 1000.0, "µs": 1000.0, "ms": 1e6, "s": 1e9}


def criterion(path):
    """Rust bench -> its median in ns, from criterion's text output."""
    out = {}
    name = None
    for line in Path(path).read_text().split("\n"):
        m = re.match(r"^([a-z0-9_]+/[a-z0-9_]+)\s*(time:.*)?$", line.strip())
        if m:
            name = m.group(1)
            line = m.group(2) or ""
        m = re.search(r"time:\s*\[\S+ \S+ (\S+) (\S+) \S+ \S+\]", line)
        if m and name:
            out[name] = float(m.group(1)) * UNITS[m.group(2)]
            name = None
    return out


def cyrius(path):
    """Cyrius bench -> its average in ns, from `cyrius bench` output."""
    out = {}
    for line in Path(path).read_text().split("\n"):
        m = re.match(r"^\s+(.+?): ([\d.]+)(ns|us|ms|s) avg", line)
        if m:
            out[m.group(1)] = float(m.group(2)) * UNITS[m.group(3)]
    return out


def show(ns):
    if ns is None:
        return "—"
    for unit, scale in (("ms", 1e6), ("µs", 1000.0)):
        if ns >= scale:
            v = ns / scale
            return f"{v:.3g} {unit}" if v < 1000 else f"{v:.0f} {unit}"
    return f"{ns:.3g} ns"


def ratio(r, c):
    if r is None or c is None:
        return "—"
    x = c / r
    return f"{x:.2g}×" if x < 10 else f"{x:.0f}×"


def table(r2p, r3p, c2p, c3p):
    r2, r3, c2, c3 = criterion(r2p), criterion(r3p), cyrius(c2p), cyrius(c3p)
    lines = ["| Rust bench | Rust 2D | Cyrius 2D | Ratio | Rust 3D | Cyrius 3D | Ratio |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    missing = []
    for b in rust_benches():
        t2, t3 = TWINS[b]
        a, x = r2.get(b), c2.get(t2)
        if a is None or x is None:
            missing.append(f"{b} (2D)")
        if t3 is None:
            cells = ["—", "as 2D", "—"]
        elif t3.startswith("n/a:"):
            cells = [show(r3.get(b)), "n/a (ADR 0008 #5)", "—"]
        else:
            y, z = r3.get(b), c3.get(t3)
            if y is None or z is None:
                missing.append(f"{b} (3D)")
            cells = [show(y), show(z), ratio(y, z)]
        lines.append(f"| `{b}` | {show(a)} | {show(x)} | {ratio(a, x)} | " + " | ".join(cells) + " |")
    for m in missing:
        print("MISSING", m, file=sys.stderr)
    print("\n".join(lines))
    return 1 if missing else 0


if args and args[0] == "--check":
    sys.exit(check())
if len(args) == 4:
    sys.exit(table(*args))
print(__doc__)
sys.exit(2)
