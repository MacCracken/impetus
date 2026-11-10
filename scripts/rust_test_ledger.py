"""The Rust-test ledger: every test of the frozen Rust oracle is ported, or recorded with a reason.

Port plan M7 (the mela pattern, after tanmatra's scripts/rust_test_ledger.py). Usage, from the repo root:
    python3 -I scripts/rust_test_ledger.py .           # write docs/development/rust-test-ledger.md
    python3 -I scripts/rust_test_ledger.py . --check   # exit 1 if the ledger is stale or a test is unaccounted

It lists each `#[test]` in rust-old/src/**.rs and rust-old/tests/*.rs, and each doc test (a runnable fence in a doc
comment, or in rust-old/README.md, which lib.rs takes as the crate's docs), and looks for its port: a
`test_group("<name>")` in one of the Cyrius test files mapped to that Rust file. A Rust test with no port must have an
entry in REASONS (or its file one in WHOLE_FILE); a REASONS entry for a test that is ported, or that no longer exists,
is an error too, as is a Rust file with tests and no FILES entry.
"""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
check = "--check" in sys.argv[2:]
out_path = root / "docs/development/rust-test-ledger.md"

B2 = ["tests/b2_types.tcyr", "tests/b2_narrowphase.tcyr", "tests/b2_raycast.tcyr", "tests/b2_state.tcyr",
      "tests/b2_joints.tcyr", "tests/b2_contacts.tcyr"]
B3 = ["tests/b3_types.tcyr", "tests/b3_narrowphase.tcyr", "tests/b3_raycast.tcyr", "tests/b3_state.tcyr",
      "tests/b3_joints.tcyr", "tests/b3_contacts.tcyr"]

# Rust file -> the Cyrius test files its tests are ported into ([] when none is).
FILES = {
    "src/error.rs": ["tests/error.tcyr"],
    "src/config.rs": ["tests/config.tcyr"],
    "src/material.rs": ["tests/material.tcyr"],
    "src/units.rs": ["tests/units.tcyr"],
    "src/force.rs": ["tests/force.tcyr"],
    "src/body.rs": ["tests/types.tcyr"],
    "src/collider.rs": ["tests/types.tcyr"],
    "src/joint.rs": ["tests/types.tcyr"],
    "src/event.rs": ["tests/types.tcyr"],
    "src/query.rs": ["tests/types.tcyr"],
    "src/arena.rs": ["tests/arena.tcyr"],
    "src/contact_tracker.rs": ["tests/contact_tracker.tcyr"],
    "src/spring.rs": ["tests/spring.tcyr"],
    "src/aabb_tree.rs": ["tests/aabb_tree.tcyr"],
    "src/spatial_hash.rs": ["tests/spatial_hash.tcyr"],
    "src/backend_2d/types.rs": ["tests/b2_types.tcyr"],
    "src/backend_2d/narrowphase.rs": ["tests/b2_narrowphase.tcyr"],
    "src/backend_2d/raycast.rs": ["tests/b2_raycast.tcyr"],
    # The 2D backend's own test module, with its serde and sub-step tests in the config and material files.
    "src/backend_2d/tests.rs": B2 + ["tests/config.tcyr", "tests/material.tcyr"],
    "src/backend_3d/tests.rs": B3,
    "src/particle.rs": ["tests/world_cases.cyr"],
    "src/world.rs": ["tests/world_cases.cyr"],
    "src/serialize.rs": ["tests/snapshot_cases.cyr"],
    "src/lib.rs": ["tests/world_cases.cyr"],
    "src/bridge.rs": [],
    "tests/integration.rs": ["tests/regression_cases.cyr"],
    "tests/regressions.rs": ["tests/regression_cases.cyr", "tests/regressions.tcyr", "tests/b3_regressions.tcyr",
                             "tests/b3_joints.tcyr", "tests/snapshot_cases.cyr"],
    "tests/oracle.rs": [],
}

# The doc tests: (where, which fence) -> (Cyrius file, test group).
DOC_TESTS = {
    ("README.md", "Quick Start"): ("tests/world_cases.cyr", "doc_readme_quick_start"),
    ("README.md", "Spring Animation"): ("tests/spring.tcyr", "doc_readme_spring_animation"),
}

XPBD = ("n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan "
        "§3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks")
BRIDGE = ("deferred: the dravya, sharira and ushma bridges wait on a Cyrius consumer (port plan §3.2, §9 item 1); "
          "they port as a small parity-tested file from these tests")
ORACLE = ("n/a: the tests of the `oracle` hooks added to the frozen crate for this port (port plan §4 step 0), "
          "not of impetus 1.4.0's API. `scripts/oracle/` drives the hooks, and the golden corpus checks what they "
          "dump")

# (Rust file, test) -> why it has no port.
REASONS = {
    ("src/backend_2d/tests.rs", "xpbd_ball_on_floor_rests"): XPBD,
    ("src/backend_2d/tests.rs", "xpbd_zero_gravity_no_drift"): XPBD,
    ("src/backend_2d/tests.rs", "xpbd_distance_joint_maintains_length"): XPBD,
    ("src/world.rs", "xpbd_gravity_moves_body"): XPBD,
    ("src/world.rs", "xpbd_collision_prevents_fallthrough"): XPBD,
    ("src/world.rs", "xpbd_collision_generates_events"): XPBD,
    ("src/world.rs", "xpbd_joint_distance"): XPBD,
    ("tests/regressions.rs", "xpbd_moves_kinematic_bodies"): XPBD,
}
# Rust file -> the reason for each of its tests (the whole file has no port).
WHOLE_FILE = {"src/bridge.rs": BRIDGE, "tests/oracle.rs": ORACLE}


def rust_tests(rel):
    """The (name, ignored) of each #[test] in rust-old/<rel>, in file order."""
    lines = (root / "rust-old" / rel).read_text().split("\n")
    found = []
    for i, line in enumerate(lines):
        if line.strip().startswith("#[test]"):
            ignored = False
            for nxt in lines[i + 1:i + 8]:
                if nxt.strip().startswith("#[ignore"):
                    ignored = True
                m = re.match(r"\s*(?:pub )?fn (\w+)", nxt)
                if m:
                    found.append((m.group(1), ignored))
                    break
    return found


def groups(rel):
    return set(re.findall(r'test_group\("([^"]+)"', (root / rel).read_text()))


def profiles(rel):
    """The profiles a Cyrius test file runs on: its own include, or those of the harnesses that include it."""
    text = (root / rel).read_text()
    own = {p for p, lib in (("2D", "tests/lib_2d.cyr"), ("3D", "tests/lib_3d.cyr")) if f'include "{lib}"' in text}
    if own:
        return own
    found = set()
    for h in root.glob("tests/*.tcyr"):
        if f'include "{rel}"' in h.read_text():
            found |= profiles(str(h.relative_to(root)))
    return found


NOT_RUN = {"ignore", "text", "toml", "sh", "bash", "json"}


def runnable_fences(text, prefix):
    """The info strings of the runnable code fences in text (lines starting with prefix, then ```)."""
    out = []
    inside = False
    for line in text.split("\n"):
        body = line.strip()
        if prefix:
            if not body.startswith(prefix):
                continue
            body = body[len(prefix):].strip()
        if body.startswith("```"):
            if inside:
                inside = False
                continue
            inside = True
            info = body[3:].strip()
            tags = {t.strip() for t in info.split(",") if t.strip()}
            # rustdoc runs an untagged fence or a `rust` one, unless it is marked ignore or another language.
            if (not tags or "rust" in tags) and not tags & NOT_RUN:
                out.append(info)
    return out


errors = []
rust_files = sorted(str(p.relative_to(root / "rust-old")) for d in ("src", "tests")
                    for p in (root / "rust-old" / d).rglob("*.rs"))
for rel in rust_files:
    if rust_tests(rel) and rel not in FILES:
        errors.append(f"{rel} has tests and no FILES entry")

rows = []      # (rust file, cyrius files, profiles, tests, counts)
notes = []     # (rust file, test, reason)
ignored = []   # (rust file, test, where it is ported)
for rel, cys in FILES.items():
    tests = rust_tests(rel)
    have = set()
    for cy in cys:
        have |= groups(cy)
    prof = set()
    for cy in cys:
        prof |= profiles(cy)
    counts = {"ported": 0, "n/a": 0, "deferred": 0}
    for name, ign in tests:
        if name in have:
            counts["ported"] += 1
            if (rel, name) in REASONS:
                errors.append(f"{rel}: {name} is ported but has a REASONS entry")
            if ign:
                where = [cy for cy in cys if name in groups(cy)]
                ignored.append((rel, name, where))
            continue
        reason = REASONS.get((rel, name)) or WHOLE_FILE.get(rel)
        if reason is None:
            errors.append(f"{rel}: {name} is neither ported nor recorded")
            continue
        counts[reason.split(":")[0]] += 1
        if rel not in WHOLE_FILE:
            notes.append((rel, name, reason))
        if ign:
            ignored.append((rel, name, None))
    for (r, t) in REASONS:
        if r == rel and t not in [n for n, _ in tests]:
            errors.append(f"{rel}: REASONS names {t}, which is not a test")
    rows.append((rel, cys, prof, len(tests), counts))

# Doc tests: every runnable fence is in DOC_TESTS, and each is ported.
fences = [("README.md", f) for f in runnable_fences((root / "rust-old/README.md").read_text(), "")]
for p in (root / "rust-old/src").rglob("*.rs"):
    text = p.read_text()
    for prefix in ("//!", "///"):
        for f in runnable_fences(text, prefix):
            fences.append((str(p.relative_to(root / "rust-old")), f))
readme_doc = '#[doc = include_str!("../README.md")]' in (root / "rust-old/src/lib.rs").read_text()
if not readme_doc:
    fences = [f for f in fences if f[0] != "README.md"]
if len(fences) != len(DOC_TESTS):
    errors.append(f"{len(fences)} runnable doc fences ({fences}), {len(DOC_TESTS)} DOC_TESTS entries")
doc_rows = []
for (where, what), (cy, name) in DOC_TESTS.items():
    if name not in groups(cy):
        errors.append(f"doc test {where} ({what}): {cy} has no test_group(\"{name}\")")
    doc_rows.append((where, what, cy, name))

total = sum(r[3] for r in rows) + len(DOC_TESTS)
sums = {k: sum(r[4][k] for r in rows) for k in ("ported", "n/a", "deferred")}
sums["ported"] += len(DOC_TESTS)


def cell(cys):
    return "<br>".join(f"`{c}`" for c in cys) if cys else "—"


md = []
md.append("# Rust-test ledger")
md.append("")
md.append("> **Generated** by `python3 -I scripts/rust_test_ledger.py .`; `--check` gates it (port plan M7, the mela")
md.append("> pattern). Every test of the frozen Rust oracle (`rust-old/`, impetus 1.4.0 with the `oracle` hooks) is")
md.append("> ported to a Cyrius test group of the same name, or recorded here with its reason. The golden corpus")
md.append("> (`tests/golden/`) checks every ported function on its grid besides; this ledger is about the oracle's own")
md.append("> assertions. Profiles are those whose harnesses run the Cyrius files: the shared case files run on both.")
md.append("")
md.append(f"**{total} Rust tests:** {sums['ported']} ported, {sums['n/a']} not applicable, {sums['deferred']} "
          "deferred.")
md.append("")
md.append("| Rust file | Cyrius tests | Profiles | Rust tests | Ported | N/A | Deferred |")
md.append("|---|---|---|---:|---:|---:|---:|")
for rel, cys, prof, n, c in rows:
    md.append(f"| `{rel}` | {cell(cys)} | {', '.join(sorted(prof)) or '—'} | {n} | {c['ported']} | {c['n/a']} | "
              f"{c['deferred']} |")
doc_files = sorted({cy for _, _, cy, _ in doc_rows})
doc_prof = set()
for cy in doc_files:
    doc_prof |= profiles(cy)
md.append(f"| doc tests (`README.md`) | {cell(doc_files)} | {', '.join(sorted(doc_prof))} | {len(DOC_TESTS)} | "
          f"{len(DOC_TESTS)} | 0 | 0 |")
md.append(f"| **Total** | | | **{total}** | **{sums['ported']}** | **{sums['n/a']}** | **{sums['deferred']}** |")
md.append("")
md.append("## Doc tests")
md.append("")
md.append("`src/lib.rs` takes `README.md` as the crate's docs, so its runnable examples are doc tests. The `ignore`")
md.append("and `text` fences in `spring.rs` and `bridge.rs` are not run.")
md.append("")
for where, what, cy, name in doc_rows:
    md.append(f"- `rust-old/{where}`, the {what} example: `{cy}`, `{name}`.")
md.append("")
md.append("## Rust's `#[ignore]`d tests")
md.append("")
md.append("Rust 1.4.0's incomplete snapshot made these fail, so they were ignored. The port's complete snapshot")
md.append("(ADR 0013) runs them.")
md.append("")
for rel, name, where in ignored:
    md.append(f"- `{rel}` `{name}`: " + (f"ported, live, in {cell(where)}." if where else "not ported."))
md.append("")
md.append("## Tests without a port")
md.append("")
md.append("| Rust file | Test | Status |")
md.append("|---|---|---|")
for rel, t, reason in notes:
    md.append(f"| `{rel}` | `{t}` | {reason} |")
for rel, reason in WHOLE_FILE.items():
    md.append(f"| `{rel}` | all {len(rust_tests(rel))} | {reason} |")
md.append("")
text = "\n".join(md)

for e in errors:
    print("LEDGER", e)
if check:
    old = out_path.read_text() if out_path.exists() else ""
    if old != text:
        print(f"STALE    {out_path.relative_to(root)} (regenerate: python3 -I scripts/rust_test_ledger.py .)")
        errors.append("stale")
else:
    out_path.write_text(text)
print(f"rust-test ledger: {total} tests — {sums['ported']} ported, {sums['n/a']} n/a, {sums['deferred']} deferred; "
      f"{len(errors)} problem(s)")
sys.exit(1 if errors else 0)
