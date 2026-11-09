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

Exit status 1 on any failure. Adapted in spirit from kimiya's and tanmatra's validate.py.

The correctly rounded math (ADR 0011) adds two generated outputs, which `--write` writes and a plain
run checks are current:
- `src/crmath_data.cyr`, the tables of src/crmath.cyr (scripts/oracle/crtables.py), whose π and ln 2
  are checked against MPFR;
- the corrected oracle, `tests/golden/<module>.cr.tsv` (from M1b part 3).
MPFR (libmpfr.so.6, 4.x) is the one system library this needs, through ctypes
(scripts/oracle/mpfr_ref.py).
"""

import importlib.util
import math
import re
import struct
import sys
from pathlib import Path

WRITE = "--write" in sys.argv[2:]
sys.dont_write_bytecode = True   # the modules loaded below leave no __pycache__ in the repo


def _load(name):
    """A module beside this script, by path: `-I` keeps the script's directory off sys.path."""
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parent / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


MPFR = _load("mpfr_ref")
CRTABLES = _load("crtables")

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


# ── fmtf and json (ADR 0010) ────────────────────────────────────────────────────────

import json as pyjson
from urllib.parse import unquote


def tok_s(t):
    return unquote(t[2:])


def digits_of(text):
    mant = re.split("[eE]", text)[0].replace("-", "").replace(".", "")
    return mant.lstrip("0").rstrip("0")


def check_fmtf(rows):
    """Every printed string reads back to its value; serde_json's digits are Python's shortest repr
    (both round half to even); Rust's Display digits are the same except on exact ties."""
    ties = 0
    for fn, cls, ins, outs, note in rows:
        x = tok_f(ins[0])
        text = tok_s(outs[0])
        if math.isnan(x) or math.isinf(x):
            continue
        if fn == "serde_json::f64":
            if not same(float(text), x):
                fail(f"fmtf: {note}: does not read back")
            if x != 0 and digits_of(text) != digits_of(repr(abs(x))):
                fail(f"fmtf: {note}: digits differ from Python's shortest repr {repr(x)}")
        elif fn == "f64::fmt_display":
            if not same(float(text), x):
                fail(f"fmtf: {note}: does not read back")
            if x != 0 and digits_of(text) != digits_of(repr(abs(x))):
                ties += 1
    return ties


def json_fields(ty, doc):
    """Python's reading of a document, flattened like scripts/oracle/src/flat.rs, or None."""
    try:
        o = pyjson.loads(doc, parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
    except ValueError:
        return None
    # serde reads a struct from an object or an array, and a unit variant from a bare string, so any
    # JSON value can be a document.
    return o


def check_json(rows):
    """Every document the oracle decodes has fields equal to Python's correctly rounded parse; every
    document the oracle writes reads back (in Python) to the floats it was written from."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn.startswith("serde_json::from_str::"):
            if outs == ["E:Deserialize"]:
                continue
            o = json_fields(fn, tok_s(ins[0]))
            if o is None:
                fail(f"json: {note}: the oracle decoded a document Python can't read")
                continue
            # Every float the oracle decoded must equal Python's float() of the matching number.
            nums = [v for v in walk_numbers(o)]
            got = [tok_f(t) for t in outs if t.startswith("f:")]
            for v in nums:
                if isinstance(v, float) and not any(same(v, g) for g in got):
                    fail(f"json: {note}: Python reads {v!r}, which the oracle's fields don't hold")
            counted += 1
        elif fn.startswith("serde_json::to_string::"):
            doc = tok_s(outs[0])
            o = json_fields(fn, doc)
            if o is None:
                fail(f"json: {note}: the oracle wrote a document Python can't read")
                continue
            written = [tok_f(t) for t in ins if t.startswith("f:")]
            back = list(walk_numbers(o))
            for w in written:
                if math.isfinite(w) and not any(same(w, b) for b in back):
                    fail(f"json: {note}: {w!r} doesn't read back from the document")
            counted += 1
    return counted


def walk_numbers(o):
    if isinstance(o, dict):
        for v in o.values():
            yield from walk_numbers(v)
    elif isinstance(o, list):
        for v in o:
            yield from walk_numbers(v)
    elif isinstance(o, float):
        # Only JSON floats: integer fields are `i:` tokens, and an integer written for an f64
        # converts exactly below 2^53.
        yield o


def _exp(x):
    try:
        return math.exp(x)
    except OverflowError:
        return math.inf


def _sin_cos(x):
    try:
        return math.sin(x), math.cos(x)
    except ValueError:
        return math.nan, math.nan


def _spring_step(p, v, t, k, c, dt, n):
    """`Spring::step` n times in Rust's expression order. Python's math calls the same libm (glibc)
    exp, sin and cos as Rust's f64 methods, so the bits must be the oracle's."""
    if math.isnan(dt) or dt <= 0.0:
        return p, v
    if math.isinf(dt):
        if c > 0.0 and k >= 0.0:
            return t, 0.0
        return p, v
    alpha = 0.5 * c
    disc = alpha * alpha - k
    scale = alpha * alpha + abs(k)
    if abs(disc) <= 1e-9 * scale or scale == 0.0:
        e = _exp(-alpha * dt)
        m = [e * (1.0 + alpha * dt), e * dt, -e * alpha * alpha * dt, e * (1.0 - alpha * dt)]
    elif disc < 0.0:
        wd = math.sqrt(-disc)
        e = _exp(-alpha * dt)
        sn, cs = _sin_cos(wd * dt)
        m = [e * (cs + alpha / wd * sn), e * sn / wd, -e * k / wd * sn, e * (cs - alpha / wd * sn)]
    else:
        beta = math.sqrt(disc)
        r1, r2 = -alpha + beta, -alpha - beta
        d = r1 - r2
        e1, e2 = _exp(r1 * dt), _exp(r2 * dt)
        m = [(r1 * e2 - r2 * e1) / d, (e1 - e2) / d, r1 * r2 * (e2 - e1) / d, (r1 * e1 - r2 * e2) / d]
    for _ in range(n):
        y0 = p - t
        v0 = v
        y = m[0] * y0 + m[1] * v0
        vv = m[2] * y0 + m[3] * v0
        if math.isfinite(y) and math.isfinite(vv):
            p = t + y
            v = vv
    return p, v


def check_spring(rows):
    """Recomputes every spring row: the constructors, `is_settled`, and `step` through a Python
    transcription on the same libm."""
    presets = {
        "Spring::critically_damped": 2.0,
        "Spring::over_damped": 3.0,
        "Spring::under_damped": 0.5,
    }
    counted = 0
    for fn, cls, ins, outs, note in rows:
        x = [tok_f(t) if t.startswith("f:") else int(t[2:]) for t in ins]
        if fn == "Spring::new":
            want = [x[0], 0.0, x[1], x[2], x[3], 0.01]
        elif fn in presets:
            k = abs(x[2])
            want = [x[0], 0.0, x[1], k, presets[fn] * math.sqrt(k), 0.01]
        elif fn == "Spring::is_settled":
            p, v, t, thr = x
            if outs != [f"i:{int(abs(p - t) < thr and abs(v) < thr)}"]:
                fail(f"spring: {note}: Python's is_settled differs")
            counted += 1
            continue
        elif fn == "Spring::step":
            want = list(_spring_step(*x))
        else:
            fail(f"spring: unknown function {fn}")
            continue
        got = [tok_f(t) for t in outs]
        if len(got) != len(want) or not all(same(a, b) for a, b in zip(got, want)):
            fail(f"spring: {note}: Python computes {want}")
        counted += 1
    return counted


# ── the corrected oracle (ADR 0011) ──────────────────────────────────────────────────
# A class-C row's reference is the oracle's own operations with every exp, sin and cos result
# correctly rounded (MPFR). Where glibc rounded every call of a row correctly, that is the oracle's
# row and nothing is written; where it didn't, tests/golden/<module>.cr.tsv carries the corrected
# outputs, which the harness compares against (accepted under ADR 11).


def _cr_spring_step(p, v, t, k, c, dt, n, prim):
    """_spring_step with exp, sin and cos from `prim` (a name -> function map)."""
    if math.isnan(dt) or dt <= 0.0:
        return p, v
    if math.isinf(dt):
        if c > 0.0 and k >= 0.0:
            return t, 0.0
        return p, v
    alpha = 0.5 * c
    disc = alpha * alpha - k
    scale = alpha * alpha + abs(k)
    if abs(disc) <= 1e-9 * scale or scale == 0.0:
        e = prim["exp"](-alpha * dt)
        m = [e * (1.0 + alpha * dt), e * dt, -e * alpha * alpha * dt, e * (1.0 - alpha * dt)]
    elif disc < 0.0:
        wd = math.sqrt(-disc)
        e = prim["exp"](-alpha * dt)
        sn, cs = prim["sin"](wd * dt), prim["cos"](wd * dt)
        m = [e * (cs + alpha / wd * sn), e * sn / wd, -e * k / wd * sn, e * (cs - alpha / wd * sn)]
    else:
        beta = math.sqrt(disc)
        r1, r2 = -alpha + beta, -alpha - beta
        d = r1 - r2
        e1, e2 = prim["exp"](r1 * dt), prim["exp"](r2 * dt)
        m = [(r1 * e2 - r2 * e1) / d, (e1 - e2) / d, r1 * r2 * (e2 - e1) / d, (r1 * e1 - r2 * e2) / d]
    for _ in range(n):
        y0 = p - t
        v0 = v
        y = m[0] * y0 + m[1] * v0
        vv = m[2] * y0 + m[3] * v0
        if math.isfinite(y) and math.isfinite(vv):
            p = t + y
            v = vv
    return p, v


def _glibc(name):
    return {"exp": _exp, "sin": lambda x: _sin_cos(x)[0], "cos": lambda x: _sin_cos(x)[1]}[name]


def _logged(name, log):
    """MPFR's correctly rounded `name`, recording each call glibc rounds otherwise into `log`."""
    def f(x):
        want = MPFR.cr(name, x)
        got = _glibc(name)(x)
        if not same(got, want):
            log.append(f"{name}(f:{bits(x):016x}) glibc f:{bits(got):016x} correctly rounded f:{bits(want):016x}")
        return want
    return f


def corrections(module, rows):
    """(fn, inputs text, corrected outputs text, note) for every row of `module` whose outputs change
    when every exp, sin and cos result is correctly rounded."""
    out = []
    for fn, cls, ins, outs, note in rows:
        log = []
        prim = {nm: _logged(nm, log) for nm in ("exp", "sin", "cos")}
        if module == "math" and fn in ("f64::exp", "f64::sin", "f64::cos", "f64::sin_cos"):
            x = tok_f(ins[0])
            if fn == "f64::sin_cos":
                want = [prim["sin"](x), prim["cos"](x)]
            else:
                want = [prim[fn[5:]](x)]
        elif module == "spring" and fn == "Spring::step":
            x = [tok_f(t) if t.startswith("f:") else int(t[2:]) for t in ins]
            want = list(_cr_spring_step(*x, prim))
        else:
            continue
        got = [tok_f(t) for t in outs]
        if all(same(a, b) for a, b in zip(got, want)) and len(got) == len(want):
            continue
        new = " ".join(f"f:{bits(w):016x}" for w in want)
        out.append((fn, " ".join(ins), new, "oracle " + " ".join(outs) + "; " + "; ".join(log)))
    return out


def cr_text(module, golden_header, rows):
    """The text of tests/golden/<module>.cr.tsv, or None when no row needs a correction."""
    fixes = corrections(module, rows)
    if not fixes:
        return None
    oracle = next(l for l in golden_header if l.startswith("# oracle:"))
    oracle = oracle.replace(" Generated by scripts/oracle; do not edit.", "")
    head = [
        f"# impetus corrected oracle (ADR 0011), format 1: {module}",
        f"{oracle} Correctly rounded: MPFR {MPFR.VERSION}. Generated by scripts/oracle/validate.py; do not edit.",
        "# regenerate: python3 -I scripts/oracle/validate.py . --write",
        "# columns: fn<TAB>inputs<TAB>outputs<TAB>note (fn, inputs: the golden row's, byte for byte; outputs: "
        "the oracle's operations with every exp, sin and cos result correctly rounded; note: the oracle's "
        "outputs, then each call glibc misrounded)",
        f"# rows: {len(fixes)}",
    ]
    return "\n".join(head + ["\t".join(f) for f in fixes]) + "\n"


def check_corrections(root, module, path_tsv, rows):
    """tests/golden/<module>.cr.tsv is current (written with --write)."""
    header = [l for l in path_tsv.read_text().splitlines()[:5]]
    text = cr_text(module, header, rows)
    path = path_tsv.with_name(f"{module}.cr.tsv")
    rel = f"tests/golden/{module}.cr.tsv"
    if WRITE:
        if text is None:
            if path.exists():
                path.unlink()
                print(f"removed {rel}")
        else:
            path.write_text(text)
            print(f"wrote {rel} ({text.count(chr(10)) - 5} rows)")
        return
    if text is None:
        if path.exists():
            fail(f"{rel}: no row needs a correction, but the file exists (regenerate with --write)")
        return
    if not path.exists() or path.read_text() != text:
        fail(f"{rel}: differs from the corrected oracle (regenerate with --write)")
        return
    print(f"{module}.cr.tsv: current, {text.count(chr(10)) - 5} corrected rows")


def check_tables(root):
    """src/crmath_data.cyr is crtables.py's rendering, and its π and ln 2 are MPFR's."""
    n = CRTABLES.PREC
    for name, fn in (("pi", CRTABLES.pi_bits), ("log2", CRTABLES.ln2_bits)):
        if fn(n) != MPFR.const_bits(name, n):
            fail(f"crtables: {name} to {n} bits differs from MPFR's")
    t = CRTABLES.tables()
    for j, (hi, lo) in enumerate(t["exp2"]):
        if hi != MPFR.bits(MPFR.cr("exp2", j / 64)):
            fail(f"crtables: 2^({j}/64) hi is not MPFR's correctly rounded value")
    for i, (sh, sl, ch, cl) in enumerate(t["sincos"]):
        if sh != MPFR.bits(MPFR.cr("sin", i / 64)) or ch != MPFR.bits(MPFR.cr("cos", i / 64)):
            fail(f"crtables: sin or cos of {i}/64 hi is not MPFR's correctly rounded value")
    path = root / "src/crmath_data.cyr"
    text = CRTABLES.render()
    if WRITE:
        path.write_text(text)
        print(f"wrote src/crmath_data.cyr ({len(text.splitlines())} lines)")
    elif not path.exists() or path.read_text() != text:
        fail("src/crmath_data.cyr is not crtables.py's output (regenerate with --write)")
    else:
        print(f"crmath_data: current; π and ln 2 to {n} bits, and every table's hi, equal MPFR {MPFR.VERSION}'s")


# The private constants t_private.rs extracts: file, Cyrius prefix, consts it doesn't carry.
PRIVATE_SOURCES = (
    ("aabb_tree.rs", "_IM_", {"NULL_NODE"}),
    ("spatial_hash.rs", "_IM_", set()),
    ("backend_2d/types.rs", "_IM_B2_", {"FALLBACK"}),
)
PRIVATE_CONST = re.compile(r"^\s*(?:pub(?:\((?:super|crate)\))?\s+)?const\s+(\w+)\s*:\s*([\w(),\[\]; ]+?)\s*=\s*(.+?);")


def check_private(root):
    """src/constants_private.cyr holds every library const of the ported modules, each equal to
    Python's own parse of the Rust literal; the b2 floors this script uses are the same values."""
    before = len(FAILS)
    cyr = {}
    for line in (root / "src/constants_private.cyr").read_text().splitlines():
        m = re.match(r"^(?:var (\w+): f64|const (\w+)) = ([-0-9.]+);$", line)
        if m:
            cyr[m.group(1) or m.group(2)] = m.group(3)
    seen = set()
    for file, prefix, excused in PRIVATE_SOURCES:
        src = (root / "rust-old/src" / file).read_text()
        for marker in ("#[cfg(test)]\nmod tests", "#[cfg(test)]\nmod mass_tests"):
            src = src.split(marker, 1)[0]
        for line in src.splitlines():
            m = PRIVATE_CONST.match(line)
            if not m or m.group(1) in excused:
                continue
            name, ty, lit = prefix + m.group(1), m.group(2), m.group(3).replace("_", "")
            seen.add(name)
            if name not in cyr:
                fail(f"constants_private: {name} (rust-old/src/{file}) is missing")
            elif ty == "f64":
                if float(cyr[name]) != float(lit):
                    fail(f"constants_private: {name} = {cyr[name]}, Rust's literal reads {float(lit)!r}")
            elif int(cyr[name]) != int(lit):
                fail(f"constants_private: {name} = {cyr[name]}, Rust's is {lit}")
    for name in sorted(set(cyr) - seen):
        fail(f"constants_private: {name} has no const in the listed Rust sources")
    for name, mine in (("_IM_B2_MIN_MASS", B2_MIN_MASS), ("_IM_B2_MIN_INERTIA", B2_MIN_INERTIA),
                       ("_IM_B2_EPSILON_SQ", B2_EPSILON_SQ)):
        if name in cyr and float(cyr[name]) != mine:
            fail(f"validate.py's {name} differs from the generated one")
    if len(FAILS) == before:
        print(f"constants_private: {len(seen)} constants equal Python's parse of the Rust literals")


def _pairs_tokens(pairs):
    out = [f"v:{len(pairs)}"]
    for a, b in pairs:
        out += [f"i:{a}", f"i:{b}"]
    return out


def _boxes(tokens, dims):
    """The boxes of a broadphase row: v:N then 2·dims floats each, as (min, max) lists."""
    n = int(tokens[0][2:])
    vals = [tok_f(t) for t in tokens[1:1 + 2 * dims * n]]
    return [(vals[k * 2 * dims:k * 2 * dims + dims], vals[k * 2 * dims + dims:(k + 1) * 2 * dims]) for k in range(n)]


def _tree_pairs(boxes):
    """Brute force: every pair whose boxes, fattened by 0.1 a side, overlap (NaN overlaps nothing)."""
    fat = [([x - 0.1 for x in mn] + [-0.1] * (3 - len(mn)), [x + 0.1 for x in mx] + [0.1] * (3 - len(mx)))
           for mn, mx in boxes]
    out = []
    for i in range(len(fat)):
        for j in range(i + 1, len(fat)):
            a, b = fat[i], fat[j]
            if all(a[0][k] <= b[1][k] and a[1][k] >= b[0][k] for k in range(3)):
                out.append((i, j))
    return out


def _axis(lo_v, hi_v, inv):
    a, b = lo_v * inv, hi_v * inv
    if not (math.isfinite(a) and math.isfinite(b)):
        return None
    lo, hi = float(math.floor(a)), float(math.floor(b))
    if not (lo <= hi) or not (-2147483648.0 <= lo <= 2147483647.0) or not (-2147483648.0 <= hi <= 2147483647.0):
        return None
    return int(lo), int(hi)


def _hash_pairs(cell, boxes):
    """Python's rasterisation: cells shared, plus every pair with an oversized entry."""
    safe = abs(cell) if math.isfinite(cell) and abs(cell) >= 1e-10 else 1.0
    inv = 1.0 / safe
    cells, over = {}, []
    for k, (mn, mx) in enumerate(boxes):
        mn3, mx3 = list(mn) + [0.0] * (3 - len(mn)), list(mx) + [0.0] * (3 - len(mx))
        rs = [_axis(mn3[d], mx3[d], inv) for d in range(3)]
        count = 1.0
        if all(r is not None for r in rs):
            for lo, hi in rs:
                count = count * (float(hi) - float(lo) + 1.0)
        if any(r is None for r in rs) or not (count <= 4096.0):
            over.append(k)
            continue
        for cx in range(rs[0][0], rs[0][1] + 1):
            for cy in range(rs[1][0], rs[1][1] + 1):
                for cz in range(rs[2][0], rs[2][1] + 1):
                    cells.setdefault((cx, cy, cz), []).append(k)
    pairs = set()
    for ks in cells.values():
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                if ks[i] != ks[j]:
                    pairs.add((min(ks[i], ks[j]), max(ks[i], ks[j])))
    for o in over:
        for k in range(len(boxes)):
            if k != o:
                pairs.add((min(o, k), max(o, k)))
    return sorted(pairs)


def check_broadphase(rows):
    """Recomputes every broadphase row: the tree's pairs by brute force on the fattened boxes, the
    hash's by an independent rasterisation, the automatic cell size."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn.startswith("aabb_tree_pairs_"):
            want = _pairs_tokens(_tree_pairs(_boxes(ins, 3 if fn.endswith("3d") else 2)))
        elif fn.startswith("spatial_hash_pairs_"):
            want = _pairs_tokens(_hash_pairs(tok_f(ins[0]), _boxes(ins[1:], 3 if fn.endswith("3d") else 2)))
        elif fn == "spatial_hash_auto_cell_size":
            dims = [tok_f(t) for t in ins[1:]]
            ok = [d for d in dims if math.isfinite(d) and d >= 0.0]
            if not ok:
                r = 1.0
            else:
                total = 0.0
                for d in ok:
                    total += d
                q = total / float(len(ok)) * 2.0
                r = 0.1 if (math.isnan(q) or q < 0.1) else q    # Rust's f64::max(q, 0.1)
            if not same(tok_f(outs[0]), r):
                fail(f"broadphase: {note}: Python computes {r!r}")
            counted += 1
            continue
        else:
            fail(f"broadphase: unknown function {fn}")
            continue
        if outs != want:
            fail(f"broadphase: {fn}: the oracle's pair set differs from the independent recomputation")
        counted += 1
    return counted


# The 2D backend's mass floors and polygon area cut (rust-old/src/backend_2d/types.rs).
B2_MIN_MASS = 1e-6
B2_MIN_INERTIA = 1e-10
B2_EPSILON_SQ = 1e-20
B2_FALLBACK = (1.0, (0.0, 0.0), 1.0)


def _polygon_mass(pts):
    """`polygon_mass_props` in Rust's expression order: (area, centroid, inertia per unit mass), or None."""
    if len(pts) < 3:
        return None
    o = pts[0]
    area = cx = cy = i_o = 0.0
    for w in range(1, len(pts) - 1):
        e1 = (pts[w][0] - o[0], pts[w][1] - o[1])
        e2 = (pts[w + 1][0] - o[0], pts[w + 1][1] - o[1])
        d = e1[0] * e2[1] - e1[1] * e2[0]
        tri = 0.5 * d
        area += tri
        cx += tri * (e1[0] + e2[0]) / 3.0
        cy += tri * (e1[1] + e2[1]) / 3.0
        intx2 = e1[0] * e1[0] + e2[0] * e1[0] + e2[0] * e2[0]
        inty2 = e1[1] * e1[1] + e2[1] * e1[1] + e2[1] * e2[1]
        i_o += (0.25 / 3.0 * d) * (intx2 + inty2)
    if abs(area) < B2_EPSILON_SQ or not math.isfinite(area):
        return None
    c = (cx / area, cy / area)
    i_c = i_o / area - (c[0] * c[0] + c[1] * c[1])
    return (abs(area), (o[0] + c[0], o[1] + c[1]), rust_max(i_c, 0.0))


def _b2_shape(ins, k):
    """The shape at token k (flat::shape's layout): (shape_props, next token)."""
    kind = ins[k].split(".", 1)[1]
    k += 1
    if kind == "Ball":
        r = tok_f(ins[k])
        return (math.pi * r * r, (0.0, 0.0), 0.5 * r * r), k + 1
    if kind == "Box":
        w = 2.0 * tok_f(ins[k])
        h = 2.0 * tok_f(ins[k + 1])
        return (abs(w * h), (0.0, 0.0), (w * w + h * h) / 12.0), k + 3
    if kind == "Capsule":
        h, r = abs(tok_f(ins[k])), abs(tok_f(ins[k + 1]))
        rect_area = 4.0 * h * r
        circle_area = math.pi * r * r
        area = rect_area + circle_area
        if not area > 0.0:
            return B2_FALLBACK, k + 2
        lc = 4.0 * r / (3.0 * math.pi)
        rect_i = rect_area * (4.0 * r * r + 4.0 * h * h) / 12.0
        circle_i = circle_area * (0.5 * r * r + h * h + 2.0 * h * lc)
        return (area, (0.0, 0.0), (rect_i + circle_i) / area), k + 2
    if kind == "Segment":
        a = (tok_f(ins[k]), tok_f(ins[k + 1]))
        b = (tok_f(ins[k + 3]), tok_f(ins[k + 4]))
        dx, dy = b[0] - a[0], b[1] - a[1]
        ln = rust_sqrt(dx * dx + dy * dy)
        return (ln * 0.01, ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5), ln * ln / 12.0), k + 6
    if kind == "ConvexHull":
        n = int(ins[k][2:])
        pts = [(tok_f(ins[k + 1 + 3 * j]), tok_f(ins[k + 2 + 3 * j])) for j in range(n)]
        return (_polygon_mass(pts) or B2_FALLBACK), k + 1 + 3 * n
    if kind == "TriMesh":
        nv = int(ins[k][2:])
        verts = [(tok_f(ins[k + 1 + 3 * j]), tok_f(ins[k + 2 + 3 * j])) for j in range(nv)]
        k += 1 + 3 * nv
        ni = int(ins[k][2:])
        tris = [[tok_i(ins[k + 1 + 3 * j + q]) for q in range(3)] for j in range(ni)]
        k += 1 + 3 * ni
        area = 0.0
        c = [0.0, 0.0]
        second = 0.0
        for tri in tris:
            if any(not 0 <= i < nv for i in tri):
                continue
            p = _polygon_mass([verts[i] for i in tri])
            if p is not None:
                ai, ci, ii = p
                area += ai
                c[0] += ai * ci[0]
                c[1] += ai * ci[1]
                second += ai * (ii + ci[0] * ci[0] + ci[1] * ci[1])
        if not area > 0.0:
            return B2_FALLBACK, k
        c = (c[0] / area, c[1] / area)
        return (area, c, rust_max(second / area - c[0] * c[0] - c[1] * c[1], 0.0)), k
    if kind == "Heightfield":
        nh = int(ins[k][2:])
        return B2_FALLBACK, k + 1 + nh + 3
    fail(f"b2_mass: unknown shape {kind}")
    return B2_FALLBACK, len(ins)


def check_b2_mass(rows):
    """Recomputes every body's mass, inertia and inverses: shape_props, mass_properties, state.rs's
    accumulation (parallel-axis shift to the body origin) and refresh_inverse_mass."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn != "b2::body_mass":
            fail(f"b2_mass: unknown function {fn}")
            continue
        dynamic = ins[0] == "e:BodyType.Dynamic"
        fixed = tok_i(ins[1]) == 1
        n = int(ins[2][2:])
        k = 3
        mass = inertia = 0.0
        for _ in range(n):
            (area, cen, ipm), k = _b2_shape(ins, k)
            ox, oy, density = tok_f(ins[k]), tok_f(ins[k + 1]), tok_f(ins[k + 2])
            explicit = None if ins[k + 3] == "n:" else tok_f(ins[k + 3])
            k += 4
            m = rust_max(explicit if explicit is not None else area * density, B2_MIN_MASS)
            ic = rust_max(m * ipm, B2_MIN_INERTIA)
            dx, dy = ox + cen[0], oy + cen[1]
            mass += m
            inertia += ic + m * (dx * dx + dy * dy)
        inv_m = inv_i = 0.0
        if dynamic and mass > 0.0 and math.isfinite(mass):
            inv_m = 1.0 / mass
            if not (fixed or math.isnan(inertia) or inertia <= 0.0 or not math.isfinite(inertia)):
                inv_i = 1.0 / inertia
        if k != len(ins):
            fail(f"b2_mass: {note}: {len(ins) - k} input tokens left over")
        want = (mass, inertia, inv_m, inv_i)
        if not all(same(tok_f(o), w) for o, w in zip(outs, want)) or len(outs) != 4:
            fail(f"b2_mass: {note}: Python computes {want!r}")
        counted += 1
    return counted


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    check_tables(root)
    check_private(root)
    golden = root / "tests/golden"
    names = sorted(p.stem for p in golden.glob("*.tsv") if not p.name.endswith(".cr.tsv"))
    index = [
        l for l in (golden / "modules.txt").read_text().splitlines() if l and not l.startswith("#")
    ]
    if index != names:
        fail(f"modules.txt lists {index}, the directory holds {names}")
    total = 0
    for name in names:
        rows = read_module(golden / f"{name}.tsv")
        total += len(rows)
        if name in ("math", "spring"):
            check_corrections(root, name, golden / f"{name}.tsv", rows)
        if name == "constants":
            check_constants(root, rows)
            print(f"constants: {len(rows)} rows checked against rust-old/src")
        elif name == "fmtf":
            ties = check_fmtf(rows)
            print(f"fmtf: {len(rows)} rows read back; serde_json = Python's shortest digits; "
                  f"{ties} Display rows differ only by Rust's round-half-up on an exact tie")
        elif name == "json":
            print(f"json: {check_json(rows)} documents checked against Python's json")
        elif name == "broadphase":
            print(f"broadphase: {check_broadphase(rows)} rows recomputed (brute force, rasterisation)")
        elif name == "b2_mass":
            print(f"b2_mass: {check_b2_mass(rows)} bodies recomputed (shape_props, parallel-axis sum)")
        elif name == "spring":
            print(f"spring: {check_spring(rows)} rows recomputed (step on Python's libm)")
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
