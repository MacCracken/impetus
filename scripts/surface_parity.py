"""Surface parity: every public item of the ported Rust modules has a Cyrius counterpart.

Port plan §6 (a per-milestone gate) and M7 (the release gate). Adapted from kimiya's and
tanmatra's scripts/surface_parity.py. Usage, from the repo root:
    python3 -I scripts/surface_parity.py .

For each ported module it lists rust-old/src/<module>.rs's public items before `#[cfg(test)]` and
checks that src/ defines the Cyrius name the naming rules give (port plan §7.4):
- free `pub fn f`                         -> `fn f(`
- `impl Struct { pub fn f(...) }`, no self -> `fn Im<Struct>_f(`
- `impl Struct { pub fn m(&self ...) }`    -> `fn m(` inside `impl Im<Struct> {`
- `impl Enum { pub fn m(self ...) }`       -> `fn <snake(Enum)>_m(`
- `impl Default for X`                     -> `fn Im<X>_default(`
- `pub const X` (also inside an impl)      -> `const IM_X =` (src/constants.cyr), or a PENDING_CONSTS entry
- `pub enum E` and each variant V          -> `enum <ENUM_NAMES[E]> {` and `<ENUM_PREFIX[E]><SNAKE(V)> =`
- `pub struct S` and each `pub` field f    -> `struct Im<S> {` with a field f, or FIELD_RENAMES' fields
Anything else needs an EXCEPTIONS entry with its reason. Every file under rust-old/src must be in
MODULES, PENDING_MODULES (a later milestone), DEFERRED_MODULES or NOT_APPLICABLE_MODULES. Exits 1
on a missing item, an EXCEPTIONS entry that names no Rust item, or a file unaccounted for.
"""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()

MODULES = ["error", "config", "material", "units", "force"]
PENDING_MODULES = {
    "body": "M1b", "collider": "M1b", "joint": "M1b", "event": "M1b", "query": "M1b",
    "arena": "M1b", "contact_tracker": "M1b", "spring": "M1b",
    "aabb_tree": "M2", "spatial_hash": "M2", "backend_2d/types": "M2",
    "backend_2d/narrowphase": "M2", "backend_2d/raycast": "M2",
    "backend_2d/solver": "M3", "backend_2d/joints": "M3", "backend_2d/state": "M3",
    "backend_2d/tests": "M3", "particle": "M3", "world": "M3",
    "backend_3d/types": "M4", "backend_3d/narrowphase": "M4", "backend_3d/raycast": "M4",
    "backend_3d/solver": "M5", "backend_3d/joints": "M5", "backend_3d/state": "M5",
    "backend_3d/tests": "M5", "serialize": "M6",
}
DEFERRED_MODULES = {"bridge": "until a Cyrius consumer exists (port plan §3.2)"}
NOT_APPLICABLE_MODULES = {
    "lib": "crate root: module list, re-exports, Send + Sync assertions",
    "oracle": "the hidden oracle hooks, Rust-only port tooling",
    "backend_2d/mod": "backend module root (handle conversions; private)",
    "backend_3d/mod": "backend module root (handle conversions; private)",
    "backend_2d/oracle": "oracle hooks", "backend_3d/oracle": "oracle hooks",
}

ENUM_NAMES = {"ImpetusError": "ImError"}
ENUM_PREFIX = {
    "ImpetusError": "IM_ERR_", "BroadphaseKind": "IM_BROADPHASE_", "SolverKind": "IM_SOLVER_",
    "CombineRule": "IM_COMBINE_", "PhysicsUnit": "IM_UNIT_",
}
# Rust field -> the Cyrius fields that carry it (Option<T> is a presence flag plus the value).
FIELD_RENAMES = {
    ("PhysicsMaterial", "static_friction"): ["has_static_friction", "static_friction"],
    ("Force", "point"): ["has_point", "point"],
    ("Impulse", "point"): ["has_point", "point"],
}
PENDING_CONSTS = {"MAX_SNAPSHOT_INDEX": "M6, with the snapshot module"}
EXCEPTIONS = {
    "units::impl Display for Quantity": "the unit symbols are physics_unit_symbol; the shortest "
                                         "round-trip number waits for the stdlib printer (S2)",
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
    cut = text.find("#[cfg(test)]\nmod tests")
    return text if cut < 0 else text[:cut]


def cyrius_index():
    """Every fn, impl method (as Type_m), const, enum member, enum and struct field in src/."""
    fns, consts, members, enums, structs = set(), set(), set(), set(), {}
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
    return fns, consts, members, enums, structs


def rust_items(module):
    """(kind, owner, name, extra) for each public item of rust-old/src/<module>.rs."""
    text = library_part((root / "rust-old/src" / f"{module}.rs").read_text())
    items = []
    owner, owner_kind, trait = None, None, None
    enum_name = None
    struct_name = None
    for line in text.splitlines():
        s = line.strip()
        if (m := re.match(r"impl (\w+(?:::\w+)*) for (\w+)", s)):
            owner, trait = m.group(2), m.group(1).split("::")[-1]
            items.append(("trait", owner, trait, None))
            continue
        if (m := re.match(r"impl (\w+) \{", s)):
            owner, trait = m.group(1), None
            continue
        if (m := re.match(r"pub enum (\w+)", s)):
            enum_name, struct_name = m.group(1), None
            items.append(("enum", None, enum_name, None))
            continue
        if (m := re.match(r"pub struct (\w+)", s)):
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
        if (m := re.match(r"pub fn (\w+)\((&?(?:mut )?self)?", s)):
            items.append(("fn", owner, m.group(1), bool(m.group(2))))
        if (m := re.match(r"pub const (\w+):", s)):
            items.append(("const", owner, m.group(1), None))
        if (m := re.match(r"pub type (\w+)", s)):
            items.append(("type", None, m.group(1), None))
    return items


def main():
    fns, consts, members, enums, structs = cyrius_index()
    enum_owners = set()
    missing, ok = [], 0
    used_exceptions = set()
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
            if kind == "const":
                if name in PENDING_CONSTS:
                    ok += 1
                    continue
                (ok := ok + 1) if f"IM_{name}" in consts else missing.append(f"{label} -> const IM_{name}")
                continue
            if kind == "fn":
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
    # Every rust-old/src file is accounted for.
    accounted = set(MODULES) | set(PENDING_MODULES) | set(DEFERRED_MODULES) | set(NOT_APPLICABLE_MODULES)
    for path in sorted((root / "rust-old/src").rglob("*.rs")):
        rel = str(path.relative_to(root / "rust-old/src"))[:-3]
        if rel not in accounted:
            missing.append(f"rust-old/src/{rel}.rs is in no module list")
    print(f"surface parity ({', '.join(MODULES)}): {ok} items ok, {len(missing)} missing")
    for m in missing:
        print("  MISSING", m)
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
