"""Compiles and runs every `cyrius` code block in the README and the guides, against the shipped bundles.

The Cyrius form of Rust's doc tests (M7 part 3). Usage, from the repo root, after `cyrius distlib --all`:
    python3 -I scripts/doc_examples.py .

A block that defines `fn main` is a whole consumer program: its `include "lib/impetus.cyr"` (or
`lib/impetus-3d.cyr`) is pointed at `dist/`, where a consumer's `cyrius deps` would have vendored it. Any other block
is a fragment: it goes into `fn main(): i64 { alloc_init(); <block> return 0; }` after the 2D bundle's include, or the
3D one's when its first line names the 3D profile. Each program is built with `cyrius build` into build/doc_examples/
and run. A compile warning, a failed build or a nonzero exit fails the check: exit 1, naming the block.
"""
import re
import subprocess
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(".").resolve()
DOCS = ["README.md"] + sorted(str(p.relative_to(root)) for p in (root / "docs/guides").glob("*.md"))
BUNDLE = {"2d": "dist/impetus.cyr", "3d": "dist/impetus-3d.cyr"}
TRAILER = "\nvar r = main();\nsys_exit_group(r);\n"


def blocks(text):
    """(line number, code) of each ```cyrius block."""
    out = []
    lines = text.split("\n")
    i = 0
    while i < len(lines):
        if lines[i].strip() == "```cyrius":
            start = i + 1
            j = start
            while j < len(lines) and lines[j].strip() != "```":
                j += 1
            out.append((start + 1, "\n".join(lines[start:j])))
            i = j
        i += 1
    return out


def program(code):
    """The block as a whole program, or None with a reason."""
    if re.search(r"^fn main\(", code, re.M):
        src = code.replace('include "lib/impetus-3d.cyr"', f'include "{BUNDLE["3d"]}"')
        src = src.replace('include "lib/impetus.cyr"', f'include "{BUNDLE["2d"]}"')
        if "dist/impetus" not in src:
            return None, 'a whole program must include "lib/impetus.cyr" or "lib/impetus-3d.cyr"'
        if "sys_exit_group(" not in src:
            src += TRAILER
        return src, None
    profile = "3d" if re.match(r"\s*#.*\b3D profile\b", code) else "2d"
    body = "\n".join("    " + ln if ln else "" for ln in code.split("\n"))
    src = (f'include "{BUNDLE[profile]}"\n\nfn main(): i64 {{\n    alloc_init();\n{body}\n    return 0;\n}}\n'
           + TRAILER)
    return src, None


def main():
    out_dir = root / "build/doc_examples"
    out_dir.mkdir(parents=True, exist_ok=True)
    total, failed = 0, []
    for doc in DOCS:
        path = root / doc
        if not path.exists():
            continue
        for line, code in blocks(path.read_text()):
            total += 1
            label = f"{doc}:{line}"
            src, why = program(code)
            if src is None:
                failed.append(f"{label}: {why}")
                continue
            stem = re.sub(r"[^A-Za-z0-9]+", "_", f"{doc}_{line}")
            cyr, exe = out_dir / f"{stem}.cyr", out_dir / stem
            cyr.write_text(src)
            b = subprocess.run(["cyrius", "build", str(cyr.relative_to(root)), str(exe.relative_to(root))],
                               cwd=root, capture_output=True, text=True)
            log = b.stdout + b.stderr
            warn = [ln.strip() for ln in log.split("\n") if "warning" in ln.lower()]
            if b.returncode != 0 or warn:
                errs = [ln.strip() for ln in log.split("\n") if "error" in ln.lower()]
                detail = warn if b.returncode == 0 else (errs or [ln for ln in log.split("\n") if ln.strip()][-3:])
                what = "warned" if b.returncode == 0 else "failed"
                failed.append(f"{label}: build {what}: " + " | ".join(detail[:3]))
                continue
            r = subprocess.run([str(exe)], cwd=root, capture_output=True, text=True, timeout=120)
            if r.returncode != 0:
                failed.append(f"{label}: exit {r.returncode}")
    for f in failed:
        print("DOC", f)
    print(f"doc examples: {total} blocks in {len(DOCS)} files, {total - len(failed)} ran, {len(failed)} failed")
    return 1 if failed else 0


sys.exit(main())
