"""Surface parity: every public item of the ported Rust modules has a Cyrius counterpart.

Port plan §6 (a per-milestone gate) and M7 (the release gate). Adapted from kimiya's and
tanmatra's scripts/surface_parity.py. Usage, from the repo root:
    python3 -I scripts/surface_parity.py .

For each ported module it lists rust-old/src/<module>.rs's public items before `#[cfg(test)]` (and
skips a fn whose own attributes include `#[cfg(test)]`, a test helper) and checks that src/ defines the Cyrius name the naming rules give (port plan §7.4):
- free `pub fn f`                         -> `fn f(`
- `impl Struct { pub fn f(...) }`, no self -> `fn Im<Struct>_f(`
- `impl Struct { pub fn m(&self ...) }`    -> `fn m(` inside `impl Im<Struct> {`
- `impl Enum { pub fn m(self ...) }`       -> `fn <snake(Enum)>_m(`
- `impl Default for X`                     -> `fn Im<X>_default(`
- `pub const X` (also inside an impl)      -> `const IM_X =` (src/constants.cyr), or a PENDING_CONSTS entry
- `pub(super)` / `pub(crate) const X`      -> `<PRIVATE_PREFIX[module]>X`, a const or f64 global
                                              (src/constants_private.cyr, generated)
- `pub enum E` and each variant V          -> `enum <ENUM_NAMES[E]> {` and `<ENUM_PREFIX[E]><SNAKE(V)> =`
- `pub struct S` and each `pub` field f    -> `struct Im<S> {` with a field f, or FIELD_RENAMES' fields
`pub`, `pub(crate)` and `pub(super)` items all count: the backend's types are crate-internal in
Rust but are the port's surface all the same. A function the naming rules would name otherwise is in FN_RENAMES; one whose Cyrius owner arrives
with a later milestone is in PENDING_FNS (counted and listed); anything else needs an EXCEPTIONS entry
with its reason. Every file under rust-old/src must be in
MODULES, PENDING_MODULES (a later milestone), DEFERRED_MODULES or NOT_APPLICABLE_MODULES. Exits 1
on a missing item, an EXCEPTIONS entry that names no Rust item, or a file unaccounted for.
"""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()

MODULES = [
    "error", "config", "material", "units", "force",
    "body", "collider", "joint", "event", "query", "arena", "contact_tracker", "spring",
    "aabb_tree", "spatial_hash", "backend_2d/types", "backend_2d/narrowphase",
    "backend_2d/raycast", "backend_2d/state", "backend_2d/joints", "backend_2d/solver",
    "particle", "world", "backend_3d/types", "backend_3d/narrowphase", "backend_3d/raycast",
    "backend_3d/state", "backend_3d/joints", "backend_3d/solver", "serialize",
]
PENDING_MODULES = {
    "backend_2d/tests": "M3",
    "backend_3d/tests": "M5",
}
DEFERRED_MODULES = {"bridge": "until a Cyrius consumer exists (port plan §3.2)"}
NOT_APPLICABLE_MODULES = {
    "lib": "crate root: module list, re-exports, Send + Sync assertions",
    "oracle": "the hidden oracle hooks, Rust-only port tooling",
    "backend_2d/mod": "backend module root (handle conversions; private)",
    "backend_3d/mod": "backend module root (handle conversions; private)",
    "backend_2d/oracle": "oracle hooks", "backend_3d/oracle": "oracle hooks",
}

# A payload enum is a kind enum plus one struct carrying every variant's fields (port plan §7.3):
# the enum check finds the kind enum, the struct is `Im<Name>`.
ENUM_NAMES = {
    "ImpetusError": "ImError", "ColliderShape": "ImShapeKind", "JointType": "ImJointKind",
    "CollisionEvent": "ImEventKind", "ForceField": "ImForceFieldKind",
}
ENUM_PREFIX = {
    "ImpetusError": "IM_ERR_", "BroadphaseKind": "IM_BROADPHASE_", "SolverKind": "IM_SOLVER_",
    "CombineRule": "IM_COMBINE_", "PhysicsUnit": "IM_UNIT_", "BodyType": "IM_BODY_",
    "ColliderShape": "IM_SHAPE_", "JointType": "IM_JOINT_", "CollisionEvent": "IM_EVENT_",
    "ForceField": "IM_FORCE_FIELD_",
}
# Rust function -> the Cyrius function that provides it, where the naming rules give another name.
FN_RENAMES = {
    # ADR 0008 #2: one fallible restore, Rust's try_restore under restore's name.
    "PhysicsWorld::try_restore": "ImPhysicsWorld::restore",
    # A handle is a bare word (src/arena.cyr), so ArenaHandle's methods are private word helpers.
    "ArenaHandle::new": "_im_arena_handle",
    "ArenaHandle::index": "_im_handle_index",
    "ArenaHandle::generation": "_im_handle_generation",
    # `get` returns the value's address, for reading and writing alike.
    "Arena::get_mut": "ImArena::get",
    # Iteration is i over 0..slot_count(), skipping a slot_handle(i) of -1, then get().
    "Arena::values": "ImArena::slot_handle",
    "Arena::values_mut": "ImArena::slot_handle",
    "Arena::iter": "ImArena::slot_handle",
    "Arena::iter_mut": "ImArena::slot_handle",
    # pub(crate) in Rust: a private helper here.
    "pair_key": "_im_pair_key",
    # `union` is a reserved word in Cyrius.
    "IslandManager::union": "ImIslandManager::unite",
    # The 2D narrowphase is backend-internal (`pub(super)`): private `_im_b2_` helpers here. (Rust's
    # test-only generate_contact, ray_circle and ray_aabb_2d are ported too, for the ported tests.)
    **{f: f"_im_b2_{f}" for f in (
        "world_pos", "world_to_local", "local_to_world", "ordered_manifold_key",
        "shapes_touch", "circle_circle", "circle_aabb", "capsule_endpoints", "closest_point_on_segment",
        "generate_contacts_multi",
    )},
    # The 3D backend's free helpers are backend-internal (`pub(super)`): private `_im_b3_` helpers here.
    **{f: f"_im_b3_{f}" for f in (
        "skew", "gyroscopic_step", "parallel_axis",
        "is_identity_quat", "closest_point_on_segment_3d", "capsule_endpoints_3d", "closest_points_segments_3d",
        "generate_contact_3d", "gjk_epa_shapes", "sphere_sphere", "sphere_obb", "aabb_aabb_3d", "capsule_sphere_3d",
        "capsule_capsule_3d", "capsule_box_3d", "obb_obb_3d", "segment_sphere_3d", "segment_box_3d",
        "convex_hull_sphere_3d", "ray_sphere", "ray_aabb_3d", "ray_capsule", "solver_mass", "error_reduction",
    )},
}
# Rust field -> the Cyrius fields that carry it (Option<T> is a presence flag plus the value).
FIELD_RENAMES = {
    ("PhysicsMaterial", "static_friction"): ["has_static_friction", "static_friction"],
    ("Force", "point"): ["has_point", "point"],
    ("Impulse", "point"): ["has_point", "point"],
    ("Collider2d", "mass"): ["has_mass", "mass"],
    ("Joint2d", "motor"): ["has_motor", "motor"],
    ("Joint2d", "break_force"): ["has_break_force", "break_force"],
    ("Collider3d", "mass"): ["has_mass", "mass"],
    ("Joint3d", "motor"): ["has_motor", "motor"],
    ("Joint3d", "break_force"): ["has_break_force", "break_force"],
    # A Vec is its records' address and their count.
    ("ContactManifold", "points"): ["points", "point_count"],
    # ADR 0013: a 2.0 snapshot holds the backend's own records. A BodySnapshot is the whole body record (`body`)
    # with its collider list; a JointSnapshot is the handle and the whole joint record (`joint`).
    ("BodySnapshot", "handle"): ["body"],
    ("BodySnapshot", "desc"): ["body"],
    ("BodySnapshot", "position"): ["body"],
    ("BodySnapshot", "rotation"): ["body"],
    ("BodySnapshot", "linear_velocity"): ["body"],
    ("BodySnapshot", "angular_velocity"): ["body"],
    ("JointSnapshot", "desc"): ["joint"],
}
# The Cyrius prefix of a module's crate-internal constants (scripts/oracle/src/t_private.rs).
PRIVATE_PREFIX = {"backend_2d/types": "_IM_B2_", "backend_3d/types": "_IM_B3_", "backend_3d/narrowphase": "_IM_B3_",
                  "backend_3d/solver": "_IM_B3_"}
PENDING_CONSTS = {}
# Rust functions whose Cyrius owner arrives with a later milestone: counted, listed, never silently dropped.
PENDING_FNS = {
    "serialize_world": "M6 part 2, the binary form (ADR 0013)",
    "deserialize_world": "M6 part 2, the binary form (ADR 0013)",
}
EXCEPTIONS = {
    "units::impl Display for Quantity": "`impl ImQuantity { fn fmt(self, sb) }`: src/fmtf.cyr's "
                                         "shortest digits in Rust's Display layout, then the symbol",
    # Handles are plain words: index in the low 32 bits, generation in the high 32.
    "body::BodyHandle": "a handle is a bare word (src/types.cyr)",
    "collider::ColliderHandle": "a handle is a bare word (src/types.cyr)",
    "joint::JointHandle": "a handle is a bare word (src/types.cyr)",
    "arena::ArenaHandle": "a handle is a bare word; _im_arena_handle builds one",
    "particle::ParticleHandle": "a particle id is a bare word the world assigns (src/particle.cyr)",
    "particle::EmitterHandle": "an emitter id is a bare word the world assigns (src/particle.cyr)",
    "world::PhysicsWorld::oracle_dump": "the oracle hook (Rust-only port tooling); the world_step rows "
                                        "read ImPhysicsWorld's fields",
    "contact_tracker::type PairKey": "a pair key is two handle words; ImPairList holds them",
    "arena::Arena::oracle_dump": "the oracle hook (Rust-only port tooling); M3's dump rows read the "
                                 "port's slots through ImArena's fields",
    "contact_tracker::ContactTracker::oracle_dump": "the oracle hook (Rust-only port tooling); M3's "
                                                    "dump rows read ImContactTracker's lists",
    "spring::impl PartialEq for StepCache": "StepCache is private: ImSpring's cache_* fields, which "
                                             "ImSpring::eq ignores as Rust's equality does",
    "spring::impl Debug for StepCache": "StepCache is private; there is no Debug in the port",
    "backend_2d/types::type ManifoldKey": "a manifold key is two collider handle words, ordered as "
                                          "a pair key (src/contact_tracker.cyr)",
    "backend_2d/narrowphase::type Contact2": "a contact is a five-word record (normal, depth, point; "
                                             "_IM_B2_CONTACT bytes) in a caller's buffer",
    # ADR 0013: the backend's collider record is the 2.0 collider snapshot, every field of it.
    "serialize::ColliderSnapshot": "the backend's collider record (ImCollider2d, ImCollider3d) is the collider "
                                   "snapshot (ADR 0013)",
    "serialize::ColliderSnapshot::handle": "ImCollider2d / ImCollider3d `handle` (ADR 0013)",
    "serialize::ColliderSnapshot::body": "ImCollider2d / ImCollider3d `body` (ADR 0013)",
    "serialize::ColliderSnapshot::desc": "the collider record's shape, offset, material, is_sensor, mass, "
                                         "collision_layer and collision_mask (ADR 0013)",
}


def snake(camel):
    out = []
    for k, ch in enumerate(camel):
        if ch.isupper() and k > 0:
            prev = camel[k - 1]
            nxt = camel[k + 1] if k + 1 < len(camel) else ""
            if prev.islower() or prev.isdigit() or (prev.isupper() and nxt.islower()):
                out.append("_")
        out.append(ch.upper())
    return "".join(out)


def library_part(text):
    cut = text.find("#[cfg(test)]\nmod ")
    return text if cut < 0 else text[:cut]


def cyrius_index():
    """Every fn, impl method (as Type_m), const, enum member, enum and struct field in src/."""
    fns, consts, members, enums, structs = set(), set(), set(), set(), {}
    globals_ = set()
    for path in sorted((root / "src").glob("*.cyr")):
        impl = None
        depth = 0
        for line in path.read_text().splitlines():
            s = line.strip()
            m = re.match(r"impl (\w+) \{", s)
            if m and depth == 0:
                impl, depth = m.group(1), 0
            m = re.match(r"fn (\w+)\(", s)
            if m:
                fns.add(f"{impl}::{m.group(1)}" if impl else m.group(1))
            m = re.match(r"const (\w+) =", s)
            if m:
                consts.add(m.group(1))
            m = re.match(r"var (\w+): f64 =", line)
            if m:
                globals_.add(m.group(1))
            m = re.match(r"(IM_\w+) = -?\d+;", s)
            if m:
                members.add(m.group(1))
            m = re.match(r"enum (\w+) \{", s)
            if m:
                enums.add(m.group(1))
            m = re.match(r"struct (\w+) \{(.*)\}", s)
            if m:
                structs[m.group(1)] = {f.split(":")[0].strip() for f in m.group(2).split(";") if f.strip()}
            elif (m := re.match(r"struct (\w+) \{", s)):
                structs[m.group(1)] = set()
                current = m.group(1)
                continue
            if impl is not None:
                depth += line.count("{") - line.count("}")
                if depth <= 0 and s == "}":
                    impl = None
    # Multi-line struct bodies.
    for path in sorted((root / "src").glob("*.cyr")):
        text = path.read_text()
        for m in re.finditer(r"struct (\w+) \{\n(.*?)\n\}", text, re.S):
            structs[m.group(1)] = {l.strip().split(":")[0].rstrip(";").strip()
                                   for l in m.group(2).splitlines() if l.strip() and not l.strip().startswith("#")}
    return fns, consts | globals_, members, enums, structs


VIS = r"pub(?:\((?:crate|super)\))?"


def rust_items(module):
    """(kind, owner, name, extra) for each public item of rust-old/src/<module>.rs."""
    text = library_part((root / "rust-old/src" / f"{module}.rs").read_text())
    items = []
    owner, owner_kind, trait = None, None, None
    enum_name = None
    struct_name = None
    lines = text.splitlines()
    for li, line in enumerate(lines):
        s = line.strip()
        if (m := re.match(r"impl (\w+(?:::\w+)*) for (\w+)", s)):
            owner, trait = m.group(2), m.group(1).split("::")[-1]
            items.append(("trait", owner, trait, None))
            continue
        if (m := re.match(r"impl(?:<[^>]*>)? (\w+)(?:<[^>]*>)? \{", s)):
            owner, trait = m.group(1), None
            continue
        if (m := re.match(VIS + r" enum (\w+)", s)):
            enum_name, struct_name = m.group(1), None
            items.append(("enum", None, enum_name, None))
            continue
        if (m := re.match(VIS + r" struct (\w+)", s)):
            struct_name, enum_name = m.group(1), None
            items.append(("struct", None, struct_name, None))
            continue
        if s.startswith("}") and not line.startswith(" "):
            owner, enum_name, struct_name = None, None, None
            continue
        if enum_name and (m := re.match(r"([A-Z]\w*)(?:\(|,|\s*\{|$)", s)) and not s.startswith("#"):
            items.append(("variant", enum_name, m.group(1), None))
        if struct_name and (m := re.match(r"pub (\w+):", s)):
            items.append(("field", struct_name, m.group(1), None))
        if (m := re.match(VIS + r" fn (\w+)(?:<[^>]*>)?\((&?(?:mut )?self)?", s)):
            # A test-only helper (`#[cfg(test)]` among the attributes above it) is not library surface.
            back = li - 1
            test_only = False
            while back >= 0 and lines[back].strip().startswith("#["):
                test_only = test_only or lines[back].strip() == "#[cfg(test)]"
                back -= 1
            if test_only:
                continue
            has_self = bool(m.group(2))
            # A signature that breaks after `(` names its receiver on the next line.
            if not has_self and s.endswith("(") and li + 1 < len(lines):
                has_self = bool(re.match(r"&?(?:mut )?self\b", lines[li + 1].strip()))
            items.append(("fn", owner, m.group(1), has_self))
        if (m := re.match(r"pub const (\w+):", s)):
            items.append(("const", owner, m.group(1), None))
        if (m := re.match(r"pub\((?:crate|super)\) const (\w+):", s)):
            items.append(("pconst", owner, m.group(1), None))
        if (m := re.match(VIS + r" type (\w+)", s)):
            items.append(("type", None, m.group(1), None))
    return items


def profile_drift():
    """Each profile's test mirror (tests/lib_2d.cyr, tests/lib_3d.cyr) includes exactly its cyrius.cyml module
    list ([lib], [lib.3d]) in the same order (port-parity.md §4.4)."""
    manifest = (root / "cyrius.cyml").read_text()
    out = []
    for table, mirror in (("lib", "tests/lib_2d.cyr"), ("lib.3d", "tests/lib_3d.cyr")):
        m = re.search(r"^\[" + re.escape(table) + r"\]\nmodules = \[(.*?)\]", manifest, re.S | re.M)
        if m is None:
            out.append(f"cyrius.cyml has no [{table}] modules list")
            continue
        want = re.findall(r'"([^"]+)"', m.group(1))
        got = re.findall(r'^include "([^"]+)"', (root / mirror).read_text(), re.M)
        if got != want:
            out.append(f"{mirror} includes {len(got)} modules that differ from cyrius.cyml [{table}]'s {len(want)}")
    return out


def field_store_scan():
    """No `s.f = g(.., t.e, ..)` where g returns a struct and t.e is struct-typed: cyrius 6.7.18 stores t.e instead of
    g's result (cyrius docs/development/issues/2026-11-09-field-store-of-struct-call-copies-a-field-argument.md). Bind
    the call to a local first. Types are resolved from struct fields, fn parameters, `var x: T` and `self` in an impl."""
    files = sorted(list((root / "src").glob("*.cyr")) + list((root / "tests").glob("*.*cyr")))
    texts = {f: f.read_text() for f in files}
    fields = {}
    for text in texts.values():
        for m in re.finditer(r"^struct (\w+)\s*\{(.*?)\}", text, re.M | re.S):
            fields[m.group(1)] = dict(re.findall(r"(\w+)\s*:\s*\*?(\w+)\s*;", m.group(2)))
    sret = set()
    for text in texts.values():
        for m in re.finditer(r"^\s*fn (\w+)\([^)]*\)\s*:\s*(\w+)\s*\{", text, re.M):
            if m.group(2) in fields:
                sret.add(m.group(1))
    store = re.compile(r"([a-z_]\w*(?:\.\w+)+)\s*=\s*(\w+)\(([^;]*)\);")
    ref = re.compile(r"\b([a-z_]\w*)((?:\.[a-z_]\w*)+)\b(?!\s*\()")
    out = []
    for f, text in texts.items():
        types, impl = {}, None
        for i, line in enumerate(text.split("\n"), 1):
            m = re.match(r"^impl (\w+)", line)
            if m:
                impl = m.group(1)
            elif line.startswith("}"):
                impl = None
            m = re.match(r"^\s*fn \w+\(([^)]*)\)", line)
            if m:
                types = dict(re.findall(r"(\w+)\s*:\s*\*?(\w+)", m.group(1)))
                if impl:
                    types["self"] = impl
            for name, ty in re.findall(r"\bvar (\w+)\s*:\s*\*?(\w+)", line):
                types[name] = ty
            for sm in store.finditer(line):
                if sm.group(2) not in sret:
                    continue
                for base, chain in ref.findall(sm.group(3)):
                    ty = types.get(base)
                    for name in chain.split(".")[1:]:
                        ty = fields.get(ty, {}).get(name)
                    if ty in fields:
                        out.append(f"{f.relative_to(root)}:{i}: a struct call with a struct-field argument stored into "
                                   f"a field (cyrius 6.7.18 misstores it; bind to a local): {line.strip()[:90]}")
                        break
    return out


def main():
    fns, consts, members, enums, structs = cyrius_index()
    enum_owners = set()
    missing, ok = [], 0
    used_exceptions = set()
    used_renames = set()
    used_pending = set()
    pending = []
    for module in MODULES:
        items = rust_items(module)
        enum_owners |= {name for kind, _, name, _ in items if kind == "enum"}
        for kind, owner, name, extra in items:
            key = f"{module}::{kind} {name}" if kind in ("type",) else None
            label = f"{module}::{owner + '::' if owner else ''}{name} ({kind})"
            if kind == "type":
                ex = f"{module}::type {name}"
                if ex in EXCEPTIONS:
                    used_exceptions.add(ex)
                    ok += 1
                else:
                    missing.append(label)
                continue
            if kind == "trait":
                ex = f"{module}::impl {name} for {owner}"
                if ex in EXCEPTIONS:
                    used_exceptions.add(ex)
                    ok += 1
                elif name == "Default" and f"Im{owner}_default" in fns:
                    ok += 1
                else:
                    missing.append(label)
                continue
            ex = f"{module}::{owner + '::' if owner else ''}{name}"
            if ex in EXCEPTIONS:
                used_exceptions.add(ex)
                ok += 1
                continue
            if kind == "enum":
                want = ENUM_NAMES.get(name, f"Im{name}")
                (ok := ok + 1) if want in enums else missing.append(f"{label} -> enum {want}")
                continue
            if kind == "variant":
                want = ENUM_PREFIX.get(owner, f"IM_{snake(owner)}_") + snake(name)
                (ok := ok + 1) if want in members else missing.append(f"{label} -> {want}")
                continue
            if kind == "struct":
                (ok := ok + 1) if f"Im{name}" in structs else missing.append(f"{label} -> struct Im{name}")
                continue
            if kind == "field":
                want = FIELD_RENAMES.get((owner, name), [name])
                have = structs.get(f"Im{owner}", set())
                (ok := ok + 1) if all(w in have for w in want) else missing.append(f"{label} -> fields {want}")
                continue
            if kind == "pconst":
                want = PRIVATE_PREFIX.get(module, "_IM_") + name
                (ok := ok + 1) if want in consts else missing.append(f"{label} -> {want}")
                continue
            if kind == "const":
                if name in PENDING_CONSTS:
                    ok += 1
                    continue
                (ok := ok + 1) if f"IM_{name}" in consts else missing.append(f"{label} -> const IM_{name}")
                continue
            if kind == "fn":
                if f"{owner + '::' if owner else ''}{name}" in PENDING_FNS:
                    used_pending.add(f"{owner + '::' if owner else ''}{name}")
                    pending.append(label)
                    ok += 1
                    continue
                rename = FN_RENAMES.get(f"{owner + '::' if owner else ''}{name}")
                if rename is not None:
                    used_renames.add(f"{owner + '::' if owner else ''}{name}")
                    (ok := ok + 1) if rename in fns else missing.append(f"{label} -> {rename}")
                    continue
                if owner is None:
                    want = [name]
                elif owner in enum_owners:
                    want = [f"{snake(owner).lower()}_{name}"]
                elif extra:
                    want = [f"Im{owner}::{name}"]
                else:
                    want = [f"Im{owner}_{name}"]
                (ok := ok + 1) if any(w in fns for w in want) else missing.append(f"{label} -> {want[0]}")
    for ex in EXCEPTIONS:
        if ex not in used_exceptions:
            missing.append(f"EXCEPTIONS entry names no Rust item: {ex}")
    for rn in FN_RENAMES:
        if rn not in used_renames:
            missing.append(f"FN_RENAMES entry names no Rust function: {rn}")
    for pn in PENDING_FNS:
        if pn not in used_pending:
            missing.append(f"PENDING_FNS entry names no Rust function: {pn}")
    missing.extend(profile_drift())
    missing.extend(field_store_scan())
    # Every rust-old/src file is accounted for.
    accounted = set(MODULES) | set(PENDING_MODULES) | set(DEFERRED_MODULES) | set(NOT_APPLICABLE_MODULES)
    for path in sorted((root / "rust-old/src").rglob("*.rs")):
        rel = str(path.relative_to(root / "rust-old/src"))[:-3]
        if rel not in accounted:
            missing.append(f"rust-old/src/{rel}.rs is in no module list")
    print(f"surface parity ({', '.join(MODULES)}): {ok} items ok ({len(pending)} pending a later "
          f"milestone), {len(missing)} missing")
    for p in pending:
        print("  PENDING", p)
    for m in missing:
        print("  MISSING", m)
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
