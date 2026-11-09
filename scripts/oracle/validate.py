#!/usr/bin/env python3
"""Independent check of the impetus golden corpus (port plan §4 step 5).

Python, standard library only; run with `python3 -I scripts/oracle/validate.py .`. It never
calls the Rust oracle, so it catches a generator that records the wrong thing:

- every `tests/golden/*.tsv` parses: the five-line header, the row count, five tab fields per
  row, a class in A–D, well-formed tokens; `modules.txt` lists exactly the files present;
- `constants.tsv` matches the literals in `rust-old/src` (the `pub const` items);
- every class-A and class-B row of `math.tsv` is recomputed in Python, in Rust's operation
  order: sqrt, abs, floor, ceil, min, max, clamp, is_nan, is_finite, to_radians, powi2, powi3;
- the class-C rows are checked for internal consistency: `sin_cos(x)` equals the separate
  `sin(x)` and `cos(x)` rows, and hypot of a Pythagorean triple is exact.

Exit status 1 on any failure. Adapted in spirit from kimiya's and tanmatra's validate.py; the
class-C references (60-digit transcendental values) arrive with the step 8 baselines.
"""

import math
import re
import struct
import sys
from pathlib import Path

FAILS = []


def fail(msg):
    FAILS.append(msg)


def from_bits(b):
    return struct.unpack("<d", struct.pack("<Q", b))[0]


def bits(x):
    return struct.unpack("<Q", struct.pack("<d", x))[0]


def same(a, b):
    """Class A: identical bits, or both NaN."""
    if math.isnan(a) and math.isnan(b):
        return True
    return bits(a) == bits(b)


TOKEN = re.compile(r"^(f:[0-9a-f]{16}|i:-?[0-9]+|e:\S+|s:\S*|n:|E:\S+|v:[0-9]+)$")


def parse_tokens(field, where):
    if field == "-":
        return []
    toks = field.split(" ")
    for t in toks:
        if not TOKEN.match(t):
            fail(f"{where}: malformed token {t!r}")
    return toks


def tok_f(t):
    return from_bits(int(t[2:], 16))


def tok_i(t):
    return int(t[2:])


def read_module(path):
    """The rows of one golden file, as (fn, class, ins, outs, note) tuples."""
    lines = path.read_text().split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    head = lines[:5]
    name = path.stem
    if len(head) < 5 or not head[0].startswith(f"# impetus golden corpus, format 1: {name}"):
        fail(f"{path.name}: bad header line 1")
    if len(head) < 5 or "libm: " not in head[1]:
        fail(f"{path.name}: the oracle line names no libm platform")
    m = re.match(r"^# rows: ([0-9]+)$", head[4]) if len(head) == 5 else None
    rows = []
    for k, line in enumerate(lines[5:], start=6):
        if line.startswith("#"):
            continue
        parts = line.split("\t")
        where = f"{path.name}:{k}"
        if len(parts) != 5:
            fail(f"{where}: {len(parts)} fields, want 5")
            continue
        fn, cls, ins, outs, note = parts
        if cls not in ("A", "B", "C", "D"):
            fail(f"{where}: class {cls!r}")
        rows.append((fn, cls, parse_tokens(ins, where), parse_tokens(outs, where), note))
    if not m or int(m.group(1)) != len(rows):
        fail(f"{path.name}: the header row count doesn't match {len(rows)} rows")
    return rows


# ── constants ────────────────────────────────────────────────────────────────────────


def rust_consts(src_dir):
    """Every `pub const NAME: <int type> = <expr>;` outside test code, evaluated."""
    out = {}
    for path in sorted(src_dir.rglob("*.rs")):
        if path.name == "tests.rs":
            continue
        text = path.read_text()
        cut = text.find("#[cfg(test)]\nmod tests")
        if cut >= 0:
            text = text[:cut]
        for m in re.finditer(r"pub const ([A-Z_0-9]+): [ui](?:8|16|32|64|size) = ([^;]+);", text):
            expr = m.group(2).replace("_", "")
            if not re.fullmatch(r"[0-9 <>()+*-]+", expr):
                fail(f"constants: can't evaluate {m.group(1)} = {expr}")
                continue
            out[m.group(1)] = eval(expr, {"__builtins__": {}})  # digits and operators only
    return out


def check_constants(root, rows):
    literals = rust_consts(root / "rust-old/src")
    seen = set()
    for fn, cls, ins, outs, _ in rows:
        name = ins[0][2:].rsplit("::", 1)[-1]
        seen.add(name)
        if fn != "const" or cls != "A":
            fail(f"constants: {name}: want a class-A const row")
        if name not in literals:
            fail(f"constants: {name} is not a public const in rust-old/src")
        elif tok_i(outs[0]) != literals[name]:
            fail(f"constants: {name} = {tok_i(outs[0])}, rust-old/src says {literals[name]}")
    for name in sorted(set(literals) - seen):
        fail(f"constants: rust-old/src's {name} has no row")


# ── math, class A and B ──────────────────────────────────────────────────────────────


def rust_floor(x):
    if math.isnan(x) or math.isinf(x) or x == math.floor(x):
        return x
    return float(math.floor(x))


def rust_ceil(x):
    if math.isnan(x) or math.isinf(x) or x == math.ceil(x):
        return x
    r = float(math.ceil(x))
    return -0.0 if r == 0.0 and x < 0 else r


def rust_sqrt(x):
    if math.isnan(x) or x < 0:
        return math.nan
    return math.sqrt(x)


# Rust's f64::min/max ignore a NaN operand. For equal operands (±0 included) rustc 1.99.1
# on x86_64 returns the SECOND one (`minsd`/`maxsd`), as kimiya's corpus found.
def rust_min(x, y):
    if math.isnan(x):
        return y
    if math.isnan(y):
        return x
    if x < y:
        return x
    if y < x:
        return y
    return y


def rust_max(x, y):
    if math.isnan(x):
        return y
    if math.isnan(y):
        return x
    if x > y:
        return x
    if y > x:
        return y
    return y


def rust_clamp(x, lo, hi):
    if x < lo:
        x = lo
    if x > hi:
        x = hi
    return x


RADS_PER_DEG = math.pi / 180.0  # Rust's `consts::PI / 180.0`, folded to the same double


def mul(a, b):
    """IEEE multiply; Python raises nothing for floats except via math functions."""
    return a * b


UNARY_A = {
    "f64::sqrt": rust_sqrt,
    "f64::abs": abs,
    "f64::floor": rust_floor,
    "f64::ceil": rust_ceil,
}
UNARY_B = {
    "f64::to_radians": lambda x: mul(x, RADS_PER_DEG),
    "f64::powi2": lambda x: mul(x, x),
    # LLVM expands a literal powi(3) to (x·x)·x; x·(x·x) multiplies the same two operands,
    # so the order can't matter. (It does from powi(4) on, as tanmatra found.)
    "f64::powi3": lambda x: mul(mul(x, x), x),
}
BINARY_A = {"f64::min": rust_min, "f64::max": rust_max}


def check_math(rows):
    sin, cos = {}, {}
    counted = 0
    for fn, cls, ins, outs, note in rows:
        xs = [tok_f(t) if t.startswith("f:") else tok_i(t) for t in ins]
        if fn in UNARY_A or fn in UNARY_B:
            got = (UNARY_A.get(fn) or UNARY_B[fn])(xs[0])
            if not same(got, tok_f(outs[0])):
                fail(f"math: {note}: Python says {got!r}")
            counted += 1
        elif fn in BINARY_A:
            got = BINARY_A[fn](xs[0], xs[1])
            if not same(got, tok_f(outs[0])):
                fail(f"math: {note}: Python says {got!r}")
            counted += 1
        elif fn == "f64::clamp":
            got = rust_clamp(*xs)
            if not same(got, tok_f(outs[0])):
                fail(f"math: {note}: Python says {got!r}")
            counted += 1
        elif fn == "f64::is_nan":
            if tok_i(outs[0]) != int(math.isnan(xs[0])):
                fail(f"math: {note}")
            counted += 1
        elif fn == "f64::is_finite":
            if tok_i(outs[0]) != int(math.isfinite(xs[0])):
                fail(f"math: {note}")
            counted += 1
        elif fn == "f64::sin":
            sin[ins[0]] = outs[0]
        elif fn == "f64::cos":
            cos[ins[0]] = outs[0]
        elif fn == "f64::hypot":
            x, y = xs
            if abs(x) < 100 and abs(y) < 100 and x == int(x) and y == int(y):
                r = math.isqrt(int(x * x + y * y))
                if r * r == int(x * x + y * y) and not same(float(r), tok_f(outs[0])):
                    fail(f"math: {note}: a Pythagorean triple is not exact")
        elif cls in ("A", "B"):
            fail(f"math: no Python reference for class-{cls} {fn}")
    # std's sin_cos is (sin, cos): its rows must equal the separate calls'.
    pairs = 0
    for fn, cls, ins, outs, note in rows:
        if fn == "f64::sin_cos":
            if ins[0] in sin and outs[0] != sin[ins[0]]:
                fail(f"math: {note}: sin differs from the f64::sin row")
            if ins[0] in cos and outs[1] != cos[ins[0]]:
                fail(f"math: {note}: cos differs from the f64::cos row")
            pairs += 1
    return counted, pairs


# ── M1a: config, material, units, force ─────────────────────────────────────────────


def tok_any(t):
    if t.startswith("f:"):
        return tok_f(t)
    if t.startswith("i:"):
        return tok_i(t)
    if t == "n:":
        return None
    return t


def rust_validate(c):
    """WorldConfig::validate's verdict for the 16 config tokens, in Rust's order."""
    ts, gx, gy, gz, vi, pi_, _det, _step, slop, corr, mv, sub, fq, dr = c[:14]
    fin = math.isfinite
    if not (fin(ts) and ts > 0.0):
        return "E:InvalidConfig"
    if not all(fin(g) for g in (gx, gy, gz)):
        return "E:InvalidConfig"
    if sub > 4096 or vi > 10_000 or pi_ > 10_000:
        return "E:InvalidConfig"
    if not (fin(slop) and slop >= 0.0):
        return "E:InvalidConfig"
    if not (0.0 <= corr <= 1.0):
        return "E:InvalidConfig"
    if math.isnan(mv) or mv <= 0.0:
        return "E:InvalidConfig"
    if not (fin(fq) and fq >= 0.0) or not (fin(dr) and dr >= 0.0):
        return "E:InvalidConfig"
    return None


def combine(rule, a, b):
    return {
        "e:CombineRule.Average": lambda: (a + b) * 0.5,
        "e:CombineRule.Min": lambda: rust_min(a, b),
        "e:CombineRule.Max": lambda: rust_max(a, b),
        "e:CombineRule.Multiply": lambda: a * b,
    }[rule]()


def check_m1a(name, rows):
    """Recomputes every row of the M1a modules that Python can. Returns the count."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        v = [tok_any(t) for t in ins]
        want = outs
        if fn == "WorldConfig::validate":
            verdict = rust_validate(v)
            if (verdict is None and want) or (verdict is not None and want != [verdict]):
                fail(f"{name}: {note}: Python says {verdict or 'Ok'}")
            counted += 1
        elif fn == "CombineRule::combine":
            got = combine(ins[0], v[1], v[2])
            if not same(got, tok_f(want[0])):
                fail(f"{name}: {note}: Python says {got!r}")
            counted += 1
        elif fn == "PhysicsMaterial::effective_static_friction":
            got = v[3] if v[3] is not None else v[0] * 1.5
            if not same(got, tok_f(want[0])):
                fail(f"{name}: {note}: Python says {got!r}")
            counted += 1
        elif fn == "Quantity::to_radians":
            got = v[0] * math.pi / 180.0 if ins[1] == "e:PhysicsUnit.Degrees" else v[0]
            if not same(got, tok_f(want[0])):
                fail(f"{name}: {note}: Python says {got!r}")
            counted += 1
        elif fn in ("Force::magnitude", "Impulse::magnitude"):
            x, y, z = v[0], v[1], v[2]
            got = rust_sqrt(x * x + y * y + z * z)
            if not same(got, tok_f(want[0])):
                fail(f"{name}: {note}: Python says {got!r}")
            counted += 1
        elif fn == "Force::gravity":
            m, g = v
            got = [0.0, (-m) * g, 0.0]
            if not all(same(a, tok_f(b)) for a, b in zip(got, want[:3])) or want[3] != "n:":
                fail(f"{name}: {note}: Python says {got!r}")
            counted += 1
    return counted


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    golden = root / "tests/golden"
    names = sorted(p.stem for p in golden.glob("*.tsv"))
    index = [
        l for l in (golden / "modules.txt").read_text().splitlines() if l and not l.startswith("#")
    ]
    if index != names:
        fail(f"modules.txt lists {index}, the directory holds {names}")
    total = 0
    for name in names:
        rows = read_module(golden / f"{name}.tsv")
        total += len(rows)
        if name == "constants":
            check_constants(root, rows)
            print(f"constants: {len(rows)} rows checked against rust-old/src")
        elif name in ("config", "material", "units", "force"):
            print(f"{name}: {check_m1a(name, rows)} rows recomputed")
        elif name == "math":
            counted, pairs = check_math(rows)
            print(
                f"math: {counted} class-A/B rows recomputed; "
                f"{pairs} sin_cos rows agree with sin and cos"
            )
    print(f"{len(names)} modules, {total} rows parsed")
    for f in FAILS[:40]:
        print("FAIL", f)
    if FAILS:
        print(f"{len(FAILS)} failures")
        return 1
    print("ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
