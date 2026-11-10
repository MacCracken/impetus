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


def _logged_hypot(log):
    """MPFR's correctly rounded hypot, recording each call glibc rounds otherwise into `log`."""
    def f(x, y):
        want = MPFR.cr2("hypot", x, y)
        got = _LIBM.hypot(x, y)
        if not same(got, want):
            log.append(f"hypot(f:{bits(x):016x}, f:{bits(y):016x}) glibc f:{bits(got):016x} "
                       f"correctly rounded f:{bits(want):016x}")
        return want
    return f


def corrections(module, rows):
    """(fn, inputs text, corrected outputs text, note) for every row of `module` whose outputs change
    when every exp, sin and cos result is correctly rounded."""
    out = []
    for fn, cls, ins, outs, note in rows:
        log = []
        prim = {nm: _logged(nm, log) for nm in ("exp", "sin", "cos")}
        if module == "math" and fn == "f64::hypot":
            x, y = tok_f(ins[0]), tok_f(ins[1])
            want = [MPFR.cr2("hypot", x, y)]
            glibc = tok_f(outs[0])
            if not same(glibc, want[0]):
                log.append(f"hypot(f:{bits(x):016x}, f:{bits(y):016x}) glibc f:{bits(glibc):016x} "
                           f"correctly rounded f:{bits(want[0]):016x}")
        elif module == "math" and fn in ("f64::exp", "f64::sin", "f64::cos", "f64::sin_cos"):
            x = tok_f(ins[0])
            if fn == "f64::sin_cos":
                want = [prim["sin"](x), prim["cos"](x)]
            else:
                want = [prim[fn[5:]](x)]
        elif module == "spring" and fn == "Spring::step":
            x = [tok_f(t) if t.startswith("f:") else int(t[2:]) for t in ins]
            want = list(_cr_spring_step(*x, prim))
        elif module in ("b2_step", "b2_contact"):
            toks = _Step2d(prim["sin"], prim["cos"], _logged_hypot(log)).step(ins, module)
            if not _same_tokens(outs, toks):
                seen = list(dict.fromkeys(log))
                out.append((fn, " ".join(ins), " ".join(toks), "oracle (dump); " + "; ".join(seen)))
            continue
        elif module == "b2_ray":
            toks = _ray_tokens(fn, _Ray(prim["sin"], prim["cos"], _logged_hypot(log)), ins)
            if not _same_tokens(outs, toks):
                seen = list(dict.fromkeys(log))
                out.append((fn, " ".join(ins), " ".join(toks), "oracle " + " ".join(outs) + "; " + "; ".join(seen)))
            continue
        elif module == "b2_narrow":
            toks = _np_tokens(fn, _Narrow(prim["sin"], prim["cos"]), _np_args(ins))
            if not _same_tokens(outs, toks):
                seen = list(dict.fromkeys(log))
                out.append((fn, " ".join(ins), " ".join(toks), "oracle " + " ".join(outs) + "; " + "; ".join(seen)))
            continue
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
        "the oracle's operations with every exp, sin, cos and hypot result correctly rounded; note: the oracle's "
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


# ── the 2D narrowphase (rust-old/src/backend_2d/narrowphase.rs) ─────────────────────────────
# A transcription in Rust's expression order, with every `sin_cos` call through the instance's sin
# and cos: glibc's (Python's math, the same libm as Rust's f64 methods) reproduces the oracle bit for
# bit; MPFR's gives the corrected oracle. Folds are explicit loops: Python 3.12's sum() of floats is
# compensated, Rust's is not.

B2_EPSILON = 1e-10


def _np_shape(ins, k):
    """The shape at token k (flat::shape's layout) as a tuple, and the token after it."""
    kind = ins[k].split(".", 1)[1]
    k += 1
    if kind == "Ball":
        return ("Ball", tok_f(ins[k])), k + 1
    if kind == "Box":
        return ("Box", tok_f(ins[k]), tok_f(ins[k + 1])), k + 3
    if kind == "Capsule":
        return ("Capsule", tok_f(ins[k]), tok_f(ins[k + 1])), k + 2
    if kind == "Segment":
        a = (tok_f(ins[k]), tok_f(ins[k + 1]))
        b = (tok_f(ins[k + 3]), tok_f(ins[k + 4]))
        return ("Segment", a, b), k + 6
    if kind == "ConvexHull":
        n = int(ins[k][2:])
        pts = [(tok_f(ins[k + 1 + 3 * j]), tok_f(ins[k + 2 + 3 * j])) for j in range(n)]
        return ("ConvexHull", pts), k + 1 + 3 * n
    if kind == "TriMesh":
        nv = int(ins[k][2:])
        verts = [(tok_f(ins[k + 1 + 3 * j]), tok_f(ins[k + 2 + 3 * j])) for j in range(nv)]
        k += 1 + 3 * nv
        ni = int(ins[k][2:])
        return ("TriMesh", verts), k + 1 + 3 * ni
    if kind == "Heightfield":
        nh = int(ins[k][2:])
        heights = [tok_f(ins[k + 1 + j]) for j in range(nh)]
        scale = (tok_f(ins[k + 1 + nh]), tok_f(ins[k + 2 + nh]))
        return ("Heightfield", heights, scale), k + 1 + nh + 3
    fail(f"b2_narrow: unknown shape {kind}")
    return ("?",), len(ins)


def _np_div(a, b):
    """IEEE division (Python raises on a zero divisor)."""
    if b == 0.0:
        if a == 0.0 or math.isnan(a):
            return math.nan
        return math.copysign(math.inf, a) * math.copysign(1.0, b)
    return a / b


class _Narrow:
    """narrowphase.rs with `sin_cos` from the given sin and cos."""

    def __init__(self, sin, cos):
        self.sin, self.cos = sin, cos

    def sc(self, x):
        return self.sin(x), self.cos(x)

    def world_pos(self, bp, br, off):
        s, c = self.sc(br)
        return (bp[0] + c * off[0] - s * off[1], bp[1] + s * off[0] + c * off[1])

    def world_to_local(self, wp, bp, br):
        dx = wp[0] - bp[0]
        dy = wp[1] - bp[1]
        s, c = self.sc(br)
        return (c * dx + s * dy, -s * dx + c * dy)

    def local_to_world(self, lp, bp, br):
        s, c = self.sc(br)
        return (bp[0] + c * lp[0] - s * lp[1], bp[1] + s * lp[0] + c * lp[1])

    def rotate(self, v, rot):
        s, c = self.sc(rot)
        return (c * v[0] - s * v[1], s * v[0] + c * v[1])

    @staticmethod
    def dot2(a, b):
        return a[0] * b[0] + a[1] * b[1]

    @staticmethod
    def flip(c):
        n, d, p = c
        return ((-n[0], -n[1]), d, p)

    @staticmethod
    def deepest(cs):
        best = None
        for c in cs:
            if c is None:
                continue
            if best is not None and best[1] >= c[1]:
                continue
            best = c
        return best

    @staticmethod
    def signed_area_2x(pts):
        n = len(pts)
        acc = -0.0
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            acc += a[0] * b[1] - b[0] * a[1]
        return acc

    @staticmethod
    def edge_normal(a, b, winding):
        e = (b[0] - a[0], b[1] - a[1])
        ln = rust_sqrt(e[0] * e[0] + e[1] * e[1])
        if ln < B2_EPSILON:
            return None
        return (_np_div(winding * e[1], ln), _np_div(-winding * e[0], ln))

    def circle_circle(self, pa, ra, pb, rb):
        dx = pb[0] - pa[0]
        dy = pb[1] - pa[1]
        dist_sq = dx * dx + dy * dy
        sum_r = ra + rb
        if dist_sq >= sum_r * sum_r:
            return None
        dist = rust_sqrt(dist_sq)
        if dist < B2_EPSILON:
            normal, depth = (0.0, 1.0), sum_r
        else:
            normal, depth = (_np_div(dx, dist), _np_div(dy, dist)), sum_r - dist
        return (normal, depth, (pa[0] + normal[0] * ra, pa[1] + normal[1] * ra))

    def circle_aabb(self, cp, radius, bp, he):
        dx = cp[0] - bp[0]
        dy = cp[1] - bp[1]
        cx = rust_clamp(dx, -he[0], he[0])
        cy = rust_clamp(dy, -he[1], he[1])
        ddx = dx - cx
        ddy = dy - cy
        dist_sq = ddx * ddx + ddy * ddy
        if dist_sq >= radius * radius:
            return None
        dist = rust_sqrt(dist_sq)
        if dist < B2_EPSILON:
            f0 = he[0] - abs(dx)
            f1 = he[1] - abs(dy)
            if f0 < f1:
                normal, depth = (1.0 if dx >= 0.0 else -1.0, 0.0), f0 + radius
            else:
                normal, depth = (0.0, 1.0 if dy >= 0.0 else -1.0), f1 + radius
        else:
            normal, depth = (_np_div(ddx, dist), _np_div(ddy, dist)), radius - dist
        return (normal, depth, (bp[0] + cx, bp[1] + cy))

    def capsule_endpoints(self, pos, rot, hh):
        s, c = self.sc(rot)
        dx = -s * hh
        dy = c * hh
        return (pos[0] - dx, pos[1] - dy), (pos[0] + dx, pos[1] + dy)

    @staticmethod
    def closest_on_segment(a, b, p):
        ab = (b[0] - a[0], b[1] - a[1])
        len_sq = ab[0] * ab[0] + ab[1] * ab[1]
        if len_sq < B2_EPSILON_SQ:
            return a
        t = _np_div((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1], len_sq)
        t = rust_clamp(t, 0.0, 1.0)
        return (a[0] + ab[0] * t, a[1] + ab[1] * t)

    def closest_segments(self, a, b, c, d):
        def dist_sq(p, q):
            x, y = p[0] - q[0], p[1] - q[1]
            return x * x + y * y

        ab = (b[0] - a[0], b[1] - a[1])
        cd = (d[0] - c[0], d[1] - c[1])
        d1 = ab[0] * ab[0] + ab[1] * ab[1]
        d2 = cd[0] * cd[0] + cd[1] * cd[1]
        pa = self.closest_on_segment(c, d, a)
        pb = self.closest_on_segment(c, d, b)
        pc = self.closest_on_segment(a, b, c)
        pd = self.closest_on_segment(a, b, d)
        best1, best2, best_d = a, pa, dist_sq(a, pa)
        for p1, p2 in ((b, pb), (pc, c), (pd, d)):
            dd = dist_sq(p1, p2)
            if dd < best_d:
                best1, best2, best_d = p1, p2, dd
        if d1 > B2_EPSILON_SQ and d2 > B2_EPSILON_SQ:
            r = (a[0] - c[0], a[1] - c[1])
            d3 = ab[0] * cd[0] + ab[1] * cd[1]
            d4 = ab[0] * r[0] + ab[1] * r[1]
            d5 = cd[0] * r[0] + cd[1] * r[1]
            denom = d1 * d2 - d3 * d3
            if abs(denom) > B2_EPSILON_SQ:
                s = rust_clamp(_np_div(d3 * d5 - d4 * d2, denom), 0.0, 1.0)
                t = rust_clamp(_np_div(d3 * s + d5, d2), 0.0, 1.0)
                s = rust_clamp(_np_div(t * d3 - d4, d1), 0.0, 1.0)
                t = rust_clamp(_np_div(d3 * s + d5, d2), 0.0, 1.0)
                p1 = (a[0] + ab[0] * s, a[1] + ab[1] * s)
                p2 = (c[0] + cd[0] * t, c[1] + cd[1] * t)
                if dist_sq(p1, p2) < best_d:
                    best1, best2 = p1, p2
        return best1, best2

    def capsule_circle(self, cpos, crot, hh, cr, bpos, br):
        e0, e1 = self.capsule_endpoints(cpos, crot, hh)
        return self.circle_circle(self.closest_on_segment(e0, e1, bpos), cr, bpos, br)

    def capsule_capsule(self, pa, ra, hha, cra, pb, rb, hhb, crb):
        a1, a2 = self.capsule_endpoints(pa, ra, hha)
        b1, b2 = self.capsule_endpoints(pb, rb, hhb)
        cpa, cpb = self.closest_segments(a1, a2, b1, b2)
        return self.circle_circle(cpa, cra, cpb, crb)

    def transform_hull(self, pts, pos, rot):
        s, c = self.sc(rot)
        return [(pos[0] + c * p[0] - s * p[1], pos[1] + s * p[0] + c * p[1]) for p in pts]

    @staticmethod
    def project(pts, axis):
        mn, mx = math.inf, -math.inf
        for p in pts:
            proj = p[0] * axis[0] + p[1] * axis[1]
            mn = rust_min(mn, proj)
            mx = rust_max(mx, proj)
        return mn, mx

    @staticmethod
    def support(pts, d):
        best = pts[0]
        best_dot = best[0] * d[0] + best[1] * d[1]
        for p in pts[1:]:
            dd = p[0] * d[0] + p[1] * d[1]
            if dd > best_dot:
                best_dot, best = dd, p
        return best

    def box_to_world(self, c, bp, br):
        n, d, p = c
        return (self.rotate(n, br), d, self.local_to_world(p, bp, br))

    def circle_obb(self, cp, r, bp, br, he):
        local = self.world_to_local(cp, bp, br)
        c = self.circle_aabb(local, r, (0.0, 0.0), he)
        return None if c is None else self.box_to_world(c, bp, br)

    def end_contacts(self, e0, e1, test):
        out = [c for c in (test(e0), test(e1)) if c is not None]
        if len(out) == 2 and self.dot2(out[0][0], out[1][0]) < 0.999:
            out = [out[0] if out[0][1] >= out[1][1] else out[1]]
        if not out:
            mid = ((e0[0] + e1[0]) * 0.5, (e0[1] + e1[1]) * 0.5)
            nearest = self.closest_on_segment(e0, e1, (0.0, 0.0))
            best = self.deepest([test(mid), test(nearest)])
            if best is not None:
                out.append(best)
        return out

    def capsule_obb(self, cpos, crot, hh, cr, bp, br, he):
        lp = self.world_to_local(cpos, bp, br)
        e0, e1 = self.capsule_endpoints(lp, crot - br, hh)
        cs = self.end_contacts(e0, e1, lambda p: self.circle_aabb(p, cr, (0.0, 0.0), he))
        return [self.box_to_world(c, bp, br) for c in cs]

    def capsule_hull(self, cpos, crot, hh, cr, pts, hp, hr):
        lp = self.world_to_local(cpos, hp, hr)
        e0, e1 = self.capsule_endpoints(lp, crot - hr, hh)
        cs = self.end_contacts(e0, e1, lambda p: self.hull_circle(pts, (0.0, 0.0), 0.0, p, cr))
        return [self.box_to_world(c, hp, hr) for c in cs]

    def capsule_segment(self, cpos, crot, hh, cr, sp, sr, sa, sb):
        e0, e1 = self.capsule_endpoints(cpos, crot, hh)
        out = [c for c in (self.segment_circle(sp, sr, sa, sb, e, cr) for e in (e0, e1)) if c is not None]
        if len(out) == 2 and self.dot2(out[0][0], out[1][0]) < 0.999:
            out = [out[0] if out[0][1] >= out[1][1] else out[1]]
        if not out:
            wa = self.world_pos(sp, sr, sa)
            wb = self.world_pos(sp, sr, sb)
            on_seg, on_cap = self.closest_segments(wa, wb, e0, e1)
            c = self.circle_circle(on_seg, 0.0, on_cap, cr)
            if c is not None:
                out.append(c)
        return out

    def hull_circle(self, pts, hp, hr, cp, radius):
        n = len(pts)
        if n < 2:
            return None
        winding = -1.0 if self.signed_area_2x(pts) < 0.0 else 1.0
        c = self.world_to_local(cp, hp, hr)
        best_d2, best_cl = math.inf, pts[0]
        max_sep, sep_n = -math.inf, (0.0, 1.0)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            cl = self.closest_on_segment(a, b, c)
            x, y = c[0] - cl[0], c[1] - cl[1]
            d2 = x * x + y * y
            if d2 < best_d2:
                best_d2, best_cl = d2, cl
            nrm = self.edge_normal(a, b, winding)
            if nrm is not None:
                sep = self.dot2(nrm, (c[0] - a[0], c[1] - a[1]))
                if sep > max_sep:
                    max_sep, sep_n = sep, nrm
        if n >= 3 and max_sep <= 0.0:
            normal = sep_n
            depth = radius - max_sep
            point = (c[0] - sep_n[0] * max_sep, c[1] - sep_n[1] * max_sep)
        else:
            if best_d2 >= radius * radius:
                return None
            dist = rust_sqrt(best_d2)
            if dist < B2_EPSILON:
                normal, depth, point = sep_n, radius, best_cl
            else:
                normal = (_np_div(c[0] - best_cl[0], dist), _np_div(c[1] - best_cl[1], dist))
                depth, point = radius - dist, best_cl
        return (self.rotate(normal, hr), depth, self.local_to_world(point, hp, hr))

    def segment_circle(self, sp, sr, sa, sb, cp, cr):
        s, c = self.sc(sr)
        wa = (sp[0] + c * sa[0] - s * sa[1], sp[1] + s * sa[0] + c * sa[1])
        wb = (sp[0] + c * sb[0] - s * sb[1], sp[1] + s * sb[0] + c * sb[1])
        cl = self.closest_on_segment(wa, wb, cp)
        dx = cp[0] - cl[0]
        dy = cp[1] - cl[1]
        dist_sq = dx * dx + dy * dy
        if dist_sq >= cr * cr:
            return None
        dist = rust_sqrt(dist_sq)
        if dist < B2_EPSILON:
            return ((0.0, 1.0), cr, cl)
        return ((_np_div(dx, dist), _np_div(dy, dist)), cr - dist, cl)

    def polygon_vertices(self, s, pos, rot):
        if s[0] == "Box":
            hx, hy = s[1], s[2]
            return [self.local_to_world(p, pos, rot) for p in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        if s[0] == "ConvexHull" and len(s[1]) >= 2:
            v = self.transform_hull(s[1], pos, rot)
            if self.signed_area_2x(v) < 0.0:
                v.reverse()
            return v
        if s[0] == "Segment":
            return [self.world_pos(pos, rot, s[1]), self.world_pos(pos, rot, s[2])]
        return None

    def max_separation(self, p1, p2):
        n = len(p1)
        best = None
        for i in range(n):
            v1 = p1[i]
            nrm = self.edge_normal(v1, p1[(i + 1) % n], 1.0)
            if nrm is None:
                continue
            sep = math.inf
            for v in p2:
                sep = rust_min(sep, self.dot2(nrm, (v[0] - v1[0], v[1] - v1[1])))
            if best is None or sep > best[1]:
                best = (i, sep)
        return best

    def clip(self, v, normal, offset):
        d0 = self.dot2(normal, v[0]) - offset
        d1 = self.dot2(normal, v[1]) - offset
        out = []
        if d0 <= 0.0:
            out.append(v[0])
        if d1 <= 0.0:
            out.append(v[1])
        if d0 * d1 < 0.0 and len(out) < 2:
            t = _np_div(d0, d0 - d1)
            out.append((v[0][0] + t * (v[1][0] - v[0][0]), v[0][1] + t * (v[1][1] - v[0][1])))
        return out if len(out) == 2 else None

    def collide_polygons(self, pa, pb):
        ra = self.max_separation(pa, pb)
        if ra is None or ra[1] > 0.0:
            return None
        rb = self.max_separation(pb, pa)
        if rb is None or rb[1] > 0.0:
            return None
        if rb[1] > ra[1] + 5e-4:
            ref, inc, edge, flip = pb, pa, rb[0], True
        else:
            ref, inc, edge, flip = pa, pb, ra[0], False
        v11, v12 = ref[edge], ref[(edge + 1) % len(ref)]
        rn = self.edge_normal(v11, v12, 1.0)
        if rn is None:
            return None
        tangent = (-rn[1], rn[0])
        ie, min_dot = 0, math.inf
        for i in range(len(inc)):
            nrm = self.edge_normal(inc[i], inc[(i + 1) % len(inc)], 1.0)
            if nrm is not None:
                d = self.dot2(nrm, rn)
                if d < min_dot:
                    min_dot, ie = d, i
        incident = [inc[ie], inc[(ie + 1) % len(inc)]]
        c = self.clip(incident, (-tangent[0], -tangent[1]), -self.dot2(tangent, v11))
        if c is not None:
            c = self.clip(c, tangent, self.dot2(tangent, v12))
        if c is None:
            return []
        front = self.dot2(rn, v11)
        normal = (-rn[0], -rn[1]) if flip else rn
        out = []
        for v in c:
            sep = self.dot2(rn, v) - front
            if sep <= 0.0:
                out.append((normal, -sep, (v[0] - rn[0] * sep * 0.5, v[1] - rn[1] * sep * 0.5)))
        return out

    def sat(self, wa, wb, ca, cb):
        cd = (cb[0] - ca[0], cb[1] - ca[1])
        min_ov, best = math.inf, (0.0, 0.0)
        for h in (wa, wb):
            n = len(h)
            for i in range(n):
                axis = self.edge_normal(h[i], h[(i + 1) % n], 1.0)
                if axis is None:
                    continue
                mna, mxa = self.project(wa, axis)
                mnb, mxb = self.project(wb, axis)
                ov = rust_min(mxa, mxb) - rust_max(mna, mnb)
                if ov <= 0.0:
                    return None
                if ov < min_ov:
                    min_ov = ov
                    best = axis if self.dot2(cd, axis) >= 0.0 else (-axis[0], -axis[1])
        if not math.isfinite(min_ov):
            return None
        pa = self.support(wa, best)
        pb = self.support(wb, (-best[0], -best[1]))
        return (best, min_ov, ((pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5))

    def polygon_contacts(self, a, pa, ra, b, pb, rb):
        va = self.polygon_vertices(a, pa, ra)
        vb = self.polygon_vertices(b, pb, rb)
        if va is None or vb is None:
            return []
        cs = self.collide_polygons(va, vb)
        if cs is None:
            return []
        if cs:
            return cs
        c = self.sat(va, vb, pa, pb)
        return [] if c is None else [c]

    def round_contact(self, a, pa, ra, b, pb, rb):
        ka, kb = a[0], b[0]
        if ka == "Ball" and kb == "Ball":
            return self.circle_circle(pa, a[1], pb, b[1])
        if ka == "Ball" and kb == "Box":
            c = self.circle_obb(pa, a[1], pb, rb, (b[1], b[2]))
            return None if c is None else self.flip(c)
        if ka == "Box" and kb == "Ball":
            return self.circle_obb(pb, b[1], pa, ra, (a[1], a[2]))
        if ka == "Capsule" and kb == "Ball":
            return self.capsule_circle(pa, ra, a[1], a[2], pb, b[1])
        if ka == "Ball" and kb == "Capsule":
            c = self.capsule_circle(pb, rb, b[1], b[2], pa, a[1])
            return None if c is None else self.flip(c)
        if ka == "Capsule" and kb == "Capsule":
            return self.capsule_capsule(pa, ra, a[1], a[2], pb, rb, b[1], b[2])
        if ka == "ConvexHull" and kb == "Ball":
            return self.hull_circle(a[1], pa, ra, pb, b[1])
        if ka == "Ball" and kb == "ConvexHull":
            c = self.hull_circle(b[1], pb, rb, pa, a[1])
            return None if c is None else self.flip(c)
        if ka == "Segment" and kb == "Ball":
            return self.segment_circle(pa, ra, a[1], a[2], pb, b[1])
        if ka == "Ball" and kb == "Segment":
            c = self.segment_circle(pb, rb, b[1], b[2], pa, a[1])
            return None if c is None else self.flip(c)
        return None

    def contacts(self, a, pa, ra, b, pb, rb):
        poly = ("Box", "ConvexHull", "Segment")
        ka, kb = a[0], b[0]
        if ka in poly and kb in poly:
            return self.polygon_contacts(a, pa, ra, b, pb, rb)
        if ka == "Capsule" and kb == "Box":
            return [self.flip(c) for c in self.capsule_obb(pa, ra, a[1], a[2], pb, rb, (b[1], b[2]))]
        if ka == "Box" and kb == "Capsule":
            return self.capsule_obb(pb, rb, b[1], b[2], pa, ra, (a[1], a[2]))
        if ka == "Capsule" and kb == "ConvexHull":
            return [self.flip(c) for c in self.capsule_hull(pa, ra, a[1], a[2], b[1], pb, rb)]
        if ka == "ConvexHull" and kb == "Capsule":
            return self.capsule_hull(pb, rb, b[1], b[2], a[1], pa, ra)
        if ka == "Capsule" and kb == "Segment":
            return [self.flip(c) for c in self.capsule_segment(pa, ra, a[1], a[2], pb, rb, b[1], b[2])]
        if ka == "Segment" and kb == "Capsule":
            return self.capsule_segment(pb, rb, b[1], b[2], pa, ra, a[1], a[2])
        c = self.round_contact(a, pa, ra, b, pb, rb)
        return [] if c is None else [c]

    def touch(self, a, pa, ra, b, pb, rb):
        if a[0] == "Ball" or b[0] == "Ball":
            return self.round_contact(a, pa, ra, b, pb, rb) is not None
        return len(self.contacts(a, pa, ra, b, pb, rb)) > 0


def _np_args(ins):
    """(shape A, pose A, shape B, pose B) of a b2_narrow row, checking every token is used."""
    a, k = _np_shape(ins, 0)
    pa, ra = (tok_f(ins[k]), tok_f(ins[k + 1])), tok_f(ins[k + 2])
    b, k = _np_shape(ins, k + 3)
    pb, rb = (tok_f(ins[k]), tok_f(ins[k + 1])), tok_f(ins[k + 2])
    if k + 3 != len(ins):
        fail(f"b2_narrow: {len(ins) - k - 3} input tokens left over")
    return a, pa, ra, b, pb, rb


def _np_tokens(fn, narrow, args):
    """The output tokens of a b2_narrow row computed by `narrow`."""
    if fn == "b2::shapes_touch":
        return [f"i:{int(narrow.touch(*args))}"]
    cs = narrow.contacts(*args)
    out = [f"v:{len(cs)}"]
    for n, d, p in cs:
        out += [f"f:{bits(x):016x}" for x in (n[0], n[1], d, p[0], p[1])]
    return out


def _same_tokens(got, want):
    """Token lists equal, floats compared by `same` (any NaN equals any NaN)."""
    if len(got) != len(want):
        return False
    for g, w in zip(got, want):
        if g.startswith("f:") and w.startswith("f:"):
            if not same(tok_f(g), tok_f(w)):
                return False
        elif g != w:
            return False
    return True


GLIBC_NARROW = _Narrow(lambda x: _sin_cos(x)[0], lambda x: _sin_cos(x)[1])


def check_b2_narrow(rows):
    """Recomputes every narrowphase row on glibc's sin and cos (the oracle's own libm)."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn not in ("b2::contacts", "b2::shapes_touch"):
            fail(f"b2_narrow: unknown function {fn}")
            continue
        want = _np_tokens(fn, GLIBC_NARROW, _np_args(ins))
        if not _same_tokens(outs, want):
            fail(f"b2_narrow: {note[:160]}: Python computes {' '.join(want)[:200]}")
        counted += 1
    return counted


# ── the 2D raycast and overlap queries (rust-old/src/backend_2d/raycast.rs) ─────────────────
# The narrowphase transcription plus the ray tests, `world_aabb` and the overlap loop, with `hypot` a
# parameter too: glibc's own (through ctypes; CPython's math.hypot is a different, correctly rounded
# implementation) for the check, MPFR's for the corrected oracle.

import ctypes as _ct

_LIBM = _ct.CDLL("libm.so.6")
_LIBM.hypot.restype = _ct.c_double
_LIBM.hypot.argtypes = [_ct.c_double, _ct.c_double]


def _glibc_hypot(x, y):
    return _LIBM.hypot(x, y)


class _Ray(_Narrow):
    """raycast.rs (and types.rs's world_aabb) on the given sin, cos and hypot."""

    def __init__(self, sin, cos, hyp):
        super().__init__(sin, cos)
        self.hyp = hyp

    def world_aabb(self, s, off, pos, rot):
        sn, cs = self.sc(rot)
        wx = pos[0] + cs * off[0] - sn * off[1]
        wy = pos[1] + sn * off[0] + cs * off[1]
        k = s[0]
        if k == "Ball":
            r = s[1]
            return (wx - r, wy - r, wx + r, wy + r)
        if k == "Box":
            ex = abs(cs * s[1]) + abs(sn * s[2])
            ey = abs(sn * s[1]) + abs(cs * s[2])
            return (wx - ex, wy - ey, wx + ex, wy + ey)
        if k == "Capsule":
            ex = abs(sn * s[1]) + s[2]
            ey = abs(cs * s[1]) + s[2]
            return (wx - ex, wy - ey, wx + ex, wy + ey)
        if k == "Segment":
            a, b = s[1], s[2]
            ax = cs * a[0] - sn * a[1] + wx
            ay = sn * a[0] + cs * a[1] + wy
            bx = cs * b[0] - sn * b[1] + wx
            by = sn * b[0] + cs * b[1] + wy
            return (rust_min(ax, bx), rust_min(ay, by), rust_max(ax, bx), rust_max(ay, by))
        if k == "Heightfield":
            heights, scale = s[1], s[2]
            w = scale[0] * float(max(len(heights), 1) - 1)
            hmin, hmax = math.inf, -math.inf
            for h in heights:
                hmin = rust_min(hmin, h)
                hmax = rust_max(hmax, h)
            return (wx, wy + hmin * scale[1], wx + w, wy + hmax * scale[1])
        pts = s[1]  # ConvexHull points, TriMesh vertices
        lx, ly, hx, hy = math.inf, math.inf, -math.inf, -math.inf
        for p in pts:
            px = cs * p[0] - sn * p[1] + wx
            py = sn * p[0] + cs * p[1] + wy
            lx, ly = rust_min(lx, px), rust_min(ly, py)
            hx, hy = rust_max(hx, px), rust_max(hy, py)
        return (lx, ly, hx, hy)

    def ray_circle_first(self, o, d, c, r):
        oc = (o[0] - c[0], o[1] - c[1])
        half_b = oc[0] * d[0] + oc[1] * d[1]
        cc = oc[0] * oc[0] + oc[1] * oc[1] - r * r
        disc = half_b * half_b - cc
        if math.isnan(disc) or disc < 0.0:
            return None
        sq = rust_sqrt(disc)
        if -half_b - sq >= 0.0:
            t = -half_b - sq
        elif -half_b + sq >= 0.0:
            t = -half_b + sq
        else:
            return None
        p = (oc[0] + d[0] * t, oc[1] + d[1] * t)
        ln = self.hyp(p[0], p[1])
        n = (_np_div(p[0], ln), _np_div(p[1], ln)) if ln > B2_EPSILON else (-d[0], -d[1])
        return (t, n)

    def ray_circle_interval(self, o, d, c, r):
        oc = (o[0] - c[0], o[1] - c[1])
        half_b = oc[0] * d[0] + oc[1] * d[1]
        cc = oc[0] * oc[0] + oc[1] * oc[1] - r * r
        disc = half_b * half_b - cc
        if math.isnan(disc) or disc < 0.0:
            return None
        sq = rust_sqrt(disc)

        def normal_at(t):
            p = (o[0] + d[0] * t - c[0], o[1] + d[1] * t - c[1])
            ln = self.hyp(p[0], p[1])
            return (_np_div(p[0], ln), _np_div(p[1], ln)) if ln > B2_EPSILON else (-d[0], -d[1])

        t_in = -half_b - sq
        t_out = -half_b + sq
        return (t_in, normal_at(t_in), t_out, normal_at(t_out))

    @staticmethod
    def ray_aabb_interval(o, d, mn, mx):
        t_in, t_out = -math.inf, math.inf
        n_in, n_out = (-d[0], -d[1]), d
        for i in (0, 1):
            if abs(d[i]) < B2_EPSILON:
                if o[i] < mn[i] or o[i] > mx[i]:
                    return None
                continue
            inv = 1.0 / d[i]
            t1 = (mn[i] - o[i]) * inv
            t2 = (mx[i] - o[i]) * inv
            n1 = [0.0, 0.0]
            n1[i] = -1.0
            n2 = [0.0, 0.0]
            n2[i] = 1.0
            if t1 > t2:
                t1, t2, n1, n2 = t2, t1, n2, n1
            if t1 > t_in:
                t_in, n_in = t1, tuple(n1)
            if t2 < t_out:
                t_out, n_out = t2, tuple(n2)
            if t_in > t_out:
                return None
        return (t_in, n_in, t_out, n_out)

    def ray_polygon_interval(self, o, d, verts):
        t_in, t_out = -math.inf, math.inf
        n_in, n_out = (-d[0], -d[1]), d
        n = len(verts)
        for i in range(n):
            a, b = verts[i], verts[(i + 1) % n]
            e = (b[0] - a[0], b[1] - a[1])
            ln = self.hyp(e[0], e[1])
            if ln < B2_EPSILON:
                continue
            nrm = (_np_div(e[1], ln), _np_div(-e[0], ln))
            dist = (o[0] - a[0]) * nrm[0] + (o[1] - a[1]) * nrm[1]
            rate = d[0] * nrm[0] + d[1] * nrm[1]
            if abs(rate) < B2_EPSILON:
                if dist > 0.0:
                    return None
                continue
            t = _np_div(-dist, rate)
            if rate < 0.0:
                if t > t_in:
                    t_in, n_in = t, nrm
            elif t < t_out:
                t_out, n_out = t, nrm
            if t_in > t_out:
                return None
        return (t_in, n_in, t_out, n_out)

    def ray_segment(self, o, d, a, b):
        e = (b[0] - a[0], b[1] - a[1])
        denom = d[0] * e[1] - d[1] * e[0]
        if abs(denom) < B2_EPSILON:
            return None
        ao = (a[0] - o[0], a[1] - o[1])
        t = _np_div(ao[0] * e[1] - ao[1] * e[0], denom)
        u = _np_div(ao[0] * d[1] - ao[1] * d[0], denom)
        if not (0.0 <= u <= 1.0) or t < 0.0:
            return None
        ln = self.hyp(e[0], e[1])
        n = (_np_div(e[1], ln), _np_div(-e[0], ln))
        if n[0] * d[0] + n[1] * d[1] > 0.0:
            n = (-n[0], -n[1])
        return (t, n)

    @staticmethod
    def union(a, b):
        t_in, n_in = (b[0], b[1]) if b[0] < a[0] else (a[0], a[1])
        t_out, n_out = (b[2], b[3]) if b[2] > a[2] else (a[2], a[3])
        return (t_in, n_in, t_out, n_out)

    def ray_capsule_interval(self, o, d, cp, crot, hh, r):
        ea, eb = self.capsule_endpoints(cp, crot, hh)
        parts = [self.ray_circle_interval(o, d, ea, r), self.ray_circle_interval(o, d, eb, r)]
        axis = (eb[0] - ea[0], eb[1] - ea[1])
        axis_len = self.hyp(axis[0], axis[1])
        if axis_len > B2_EPSILON:
            ax = (_np_div(axis[0], axis_len), _np_div(axis[1], axis_len))
            perp = (-ax[1], ax[0])
            rel = (o[0] - cp[0], o[1] - cp[1])
            lo = (rel[0] * ax[0] + rel[1] * ax[1], rel[0] * perp[0] + rel[1] * perp[1])
            ld = (d[0] * ax[0] + d[1] * ax[1], d[0] * perp[0] + d[1] * perp[1])

            def back(n):
                return (n[0] * ax[0] + n[1] * perp[0], n[0] * ax[1] + n[1] * perp[1])

            iv = self.ray_aabb_interval(lo, ld, (-hh, -r), (hh, r))
            parts.append(None if iv is None else (iv[0], back(iv[1]), iv[2], back(iv[3])))
        acc = None
        for p in parts:
            if p is not None:
                acc = p if acc is None else self.union(acc, p)
        return acc

    def ray_shape(self, s, pos, rot, o, d):
        k = s[0]
        if k == "Ball":
            return self.ray_circle_interval(o, d, pos, s[1])
        if k == "Box":
            lo = self.world_to_local(o, pos, rot)
            sn, cs = self.sc(rot)
            ld = (cs * d[0] + sn * d[1], -sn * d[0] + cs * d[1])
            he = (abs(s[1]), abs(s[2]))
            iv = self.ray_aabb_interval(lo, ld, (-he[0], -he[1]), he)
            if iv is None:
                return None

            def back(n):
                return (cs * n[0] - sn * n[1], sn * n[0] + cs * n[1])

            return (iv[0], back(iv[1]), iv[2], back(iv[3]))
        if k == "Capsule":
            return self.ray_capsule_interval(o, d, pos, rot, s[1], s[2])
        if k == "Segment":
            wa = self.world_pos(pos, rot, s[1])
            wb = self.world_pos(pos, rot, s[2])
            h = self.ray_segment(o, d, wa, wb)
            return None if h is None else (h[0], h[1], h[0], h[1])
        if k == "ConvexHull" and len(s[1]) >= 3:
            verts = [self.world_pos(pos, rot, p) for p in s[1]]
            if self.signed_area_2x(verts) < 0.0:
                verts.reverse()
            return self.ray_polygon_interval(o, d, verts)
        return None

    @staticmethod
    def first(iv):
        if iv[0] >= 0.0:
            return (iv[0], iv[1])
        if iv[2] >= 0.0:
            return (iv[2], iv[3])
        return None

    def raycast(self, bodies, o, d, max_dist, mask):
        dl = self.hyp(d[0], d[1])
        if math.isnan(dl) or dl < B2_EPSILON or not math.isfinite(dl):
            return None
        dv = (d[0] / dl, d[1] / dl)
        best = None
        for idx, (pos, rot, s, off, layer) in enumerate(bodies):
            if layer & mask == 0:
                continue
            wp = self.world_pos(pos, rot, off)
            if s[0] == "Ball":
                hit = self.ray_circle_first(o, dv, wp, s[1])
            else:
                iv = self.ray_shape(s, wp, rot, o, dv)
                hit = None if iv is None else self.first(iv)
            if hit is not None and hit[0] <= max_dist and (best is None or hit[0] < best[0]):
                best = (hit[0], idx, hit[1])
        if best is None:
            return None
        t, idx, n = best
        return (idx, o[0] + dv[0] * t, o[1] + dv[1] * t, n[0], n[1], t)

    def overlap(self, bodies, probe, ppos, box):
        out = []
        for idx, (pos, rot, s, off, layer) in enumerate(bodies):
            wa = self.world_aabb(s, off, pos, rot)
            if not (box[0] <= wa[2] and box[2] >= wa[0] and box[1] <= wa[3] and box[3] >= wa[1]):
                continue
            if s[0] in ("Heightfield", "TriMesh"):
                out.append(idx)
                continue
            wp = self.world_pos(pos, rot, off)
            if self.touch(probe, ppos, 0.0, s, wp, rot):
                out.append(idx)
        return out


def _ray_args(ins):
    """(bodies, query arguments) of a b2_ray row: each body (pos, rot, shape, offset, layer)."""
    n = int(ins[0][2:])
    k = 1
    bodies = []
    for _ in range(n):
        pos, rot = (tok_f(ins[k]), tok_f(ins[k + 1])), tok_f(ins[k + 2])
        s, k = _np_shape(ins, k + 3)
        off, layer = (tok_f(ins[k]), tok_f(ins[k + 1])), tok_i(ins[k + 2])
        bodies.append((pos, rot, s, off, layer & 0xFFFFFFFF))
        k += 3
    return bodies, ins[k:]


def _ray_tokens(fn, ray, ins):
    """The output tokens of a b2_ray row computed by `ray`."""
    bodies, q = _ray_args(ins)
    if fn == "b2::raycast":
        o = (tok_f(q[0]), tok_f(q[1]))
        d = (tok_f(q[2]), tok_f(q[3]))
        hit = ray.raycast(bodies, o, d, tok_f(q[4]), tok_i(q[5]) & 0xFFFFFFFF)
        if hit is None:
            return ["n:"]
        return [f"i:{hit[0]}"] + [f"f:{bits(x):016x}" for x in hit[1:]]
    if fn == "b2::overlap_sphere":
        c, r = (tok_f(q[0]), tok_f(q[1])), tok_f(q[2])
        found = ray.overlap(bodies, ("Ball", r), c, (c[0] - r, c[1] - r, c[0] + r, c[1] + r))
    elif fn == "b2::overlap_aabb":
        lo, hi = (tok_f(q[0]), tok_f(q[1])), (tok_f(q[2]), tok_f(q[3]))
        probe = ("Box", abs(hi[0] - lo[0]) * 0.5, abs(hi[1] - lo[1]) * 0.5)
        c = ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)
        found = ray.overlap(bodies, probe, c, (lo[0], lo[1], hi[0], hi[1]))
    else:
        fail(f"b2_ray: unknown function {fn}")
        return []
    return [f"v:{len(found)}"] + [f"i:{h}" for h in found]


GLIBC_RAY = _Ray(lambda x: _sin_cos(x)[0], lambda x: _sin_cos(x)[1], _glibc_hypot)


def check_b2_ray(rows):
    """Recomputes every raycast and overlap row on glibc's sin, cos and hypot (the oracle's libm)."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        want = _ray_tokens(fn, GLIBC_RAY, ins)
        if not _same_tokens(outs, want):
            fail(f"b2_ray: {note[:160]}: Python computes {' '.join(want)[:200]}")
        counted += 1
    return counted


# ── the 2D backend's state (rust-old/src/backend_2d/state.rs, arena.rs) ──────────────────────
# A model that replays each script: generational arenas with Rust's free list, the bodies' fields, the
# colliders' mass sums (the b2_mass helpers), joints, body_colliders, waking. It rebuilds every op's
# result and the dump in dump2d.rs's order, token for token.


class _Arena:
    """rust-old/src/arena.rs: slots of [occupied, generation, next_free, value]."""

    def __init__(self):
        self.slots, self.free_head, self.stale = [], None, False

    @staticmethod
    def handle(i, g):
        return (g << 32) | i

    def live(self, h):
        i, g = h & 0xFFFFFFFF, (h >> 32) & 0xFFFFFFFF
        return i if i < len(self.slots) and self.slots[i][0] and self.slots[i][1] == g else None

    def get(self, h):
        i = self.live(h)
        return None if i is None else self.slots[i][3]

    def insert(self, v):
        if self.stale:
            self.free_head = None
            for i in range(len(self.slots) - 1, -1, -1):
                if not self.slots[i][0]:
                    self.slots[i][2] = self.free_head
                    self.free_head = i
            self.stale = False
        if self.free_head is not None:
            i = self.free_head
            s = self.slots[i]
            self.free_head = s[2]
            s[0], s[2], s[3] = True, None, v
            return self.handle(i, s[1])
        self.slots.append([True, 0, None, v])
        return self.handle(len(self.slots) - 1, 0)

    def _free(self, i):
        s = self.slots[i]
        v = s[3]
        s[0], s[1], s[2], s[3] = False, (s[1] + 1) & 0xFFFFFFFF, self.free_head, None
        self.free_head = i
        return v

    def remove(self, h):
        i = self.live(h)
        return None if i is None else self._free(i)

    def retain(self, keep):
        for i, s in enumerate(self.slots):
            if s[0] and not keep(s[3]):
                self._free(i)

    def values(self):
        return [(self.handle(i, s[1]), s[3]) for i, s in enumerate(self.slots) if s[0]]

    def tokens(self):
        t = [f"v:{len(self.slots)}"]
        for s in self.slots:
            t += [f"i:{int(s[0])}", f"i:{s[1]}", "n:" if s[2] is None else f"i:{s[2]}"]
        t += ["n:" if self.free_head is None else f"i:{self.free_head}", f"i:{int(self.stale)}"]
        return t


def _st_word(t):
    """A handle token's u64 word."""
    return int(t[2:]) & 0xFFFFFFFFFFFFFFFF


def _st_h(w):
    """The `i:` token of a u64 handle word (two's complement above i64::MAX)."""
    return f"i:{w - (1 << 64) if w >= 1 << 63 else w}"


def _st_f(x):
    return f"f:{bits(x):016x}"


def _st_shape_end(ins, k):
    """The token after the shape at k."""
    return _b2_shape(ins, k)[1]


def _st_joint_type_end(ins, k):
    kind = ins[k].split(".", 1)[1]
    k += 1
    if kind in ("Revolute", "Prismatic"):
        return k + 2 + (1 if ins[k + 2] == "n:" else 2)
    return k + {"Fixed": 0, "Spring": 3, "Distance": 1, "Wheel": 4, "Rope": 1, "Mouse": 6}[kind]


def _st_refresh(b):
    if b["type"] == "e:BodyType.Dynamic" and b["mass"] > 0.0 and math.isfinite(b["mass"]):
        b["inv_mass"] = 1.0 / b["mass"]
        i = b["inertia"]
        b["inv_inertia"] = 0.0 if (b["fixed"] or math.isnan(i) or i <= 0.0 or not math.isfinite(i)) else 1.0 / i
    else:
        b["inv_mass"] = b["inv_inertia"] = 0.0


def _st_mass_terms(c):
    """(mass, inertia about the body origin) of a collider: mass_properties and the parallel-axis shift."""
    (area, cen, ipm) = c["props"]
    m = rust_max(c["explicit"] if c["explicit"] is not None else area * c["density"], B2_MIN_MASS)
    ic = rust_max(m * ipm, B2_MIN_INERTIA)
    dx, dy = c["off"][0] + cen[0], c["off"][1] + cen[1]
    return m, ic + m * (dx * dx + dy * dy)


def _st_wake(b):
    b["sleeping"], b["sleep_timer"] = False, 0.0


def _b2_state_row(ins):
    """The outputs of a state script: each op's result, then the backend dump."""
    bodies, colls, joints = _Arena(), _Arena(), _Arena()
    lists = {}
    out = []

    def wake_neighbours(h):
        wake = [h]
        for _, j in joints.values():
            if j["a"] == h:
                wake.append(j["b"])
            elif j["b"] == h:
                wake.append(j["a"])
        for w in wake:
            b = bodies.get(w)
            if b is not None:
                _st_wake(b)

    def recompute(h):
        mass = inertia = 0.0
        for ch in lists.get(h, []):
            c = colls.get(ch)
            if c is not None:
                m, i = _st_mass_terms(c)
                mass += m
                inertia += i
        b = bodies.get(h)
        if b is not None:
            b["mass"], b["inertia"] = mass, inertia
            _st_refresh(b)

    n = int(ins[0][2:])
    k = 1
    for _ in range(n):
        op = ins[k].split(".", 1)[1]
        k += 1
        if op == "AddBody":
            d = ins[k:k + 13]
            k += 13
            px, py, rot = tok_f(d[1]), tok_f(d[2]), tok_f(d[4])
            b = {"type": d[0], "pos": (px, py), "rot": rot, "vel": (tok_f(d[5]), tok_f(d[6])),
                 "w": tok_f(d[8]), "ld": tok_f(d[9]), "ad": tok_f(d[10]), "fixed": tok_i(d[11]) == 1,
                 "gs": 1.0 if d[12] == "n:" else tok_f(d[12]), "force": (0.0, 0.0), "torque": 0.0,
                 "mass": 0.0, "inv_mass": 0.0, "inertia": 0.0, "inv_inertia": 0.0,
                 "sleeping": False, "sleep_timer": 0.0, "prev": (px, py), "prev_rot": rot}
            h = bodies.insert(b)
            b["handle"] = h
            lists[h] = []
            out.append(_st_h(h))
        elif op == "AddCollider":
            body = _st_word(ins[k])
            s0 = k + 1
            props, k2 = _b2_shape(ins, s0)
            shape = ins[s0:k2]
            mat = ins[k2 + 3:k2 + 10]
            c = {"body": body, "shape": shape, "off": (tok_f(ins[k2]), tok_f(ins[k2 + 1])), "mat": mat,
                 "density": tok_f(mat[2]), "props": props, "sensor": ins[k2 + 10],
                 "mass_tok": ins[k2 + 11], "explicit": None if ins[k2 + 11] == "n:" else tok_f(ins[k2 + 11]),
                 "layer": ins[k2 + 12], "mask": ins[k2 + 13]}
            k = k2 + 14
            b = bodies.get(body)
            if b is not None:
                m, i = _st_mass_terms(c)
                b["mass"] += m
                b["inertia"] += i
                _st_refresh(b)
            h = colls.insert(c)
            c["handle"] = h
            lists.setdefault(body, []).append(h)
            out.append(_st_h(h))
        elif op == "AddJoint":
            a, bb = _st_word(ins[k]), _st_word(ins[k + 1])
            ke = _st_joint_type_end(ins, k + 2)
            jt = ins[k + 2:ke]
            anchors = ins[ke:ke + 4]
            k = ke + 4
            motor = ins[k:k + 1] if ins[k] == "n:" else ins[k:k + 2]
            k += len(motor)
            damping, brk = ins[k], ins[k + 1]
            k += 2
            ra = bodies.get(a)["rot"] if bodies.get(a) is not None else 0.0
            rb = bodies.get(bb)["rot"] if bodies.get(bb) is not None else 0.0
            j = {"a": a, "b": bb, "jt": jt, "anchors": anchors, "motor": motor, "damping": damping,
                 "brk": brk, "ref": rb - ra}
            out.append(_st_h(joints.insert(j)))
        elif op in ("ApplyForce", "ApplyImpulse"):
            h = _st_word(ins[k])
            v = (tok_f(ins[k + 1]), tok_f(ins[k + 2]))
            pt = None if ins[k + 4] == "n:" else (tok_f(ins[k + 4]), tok_f(ins[k + 5]))
            k += 5 if pt is None else 7
            b = bodies.get(h)
            if b is None:
                continue
            if op == "ApplyForce":
                _st_wake(b)
                b["force"] = (b["force"][0] + v[0], b["force"][1] + v[1])
                if pt is not None:
                    b["torque"] += pt[0] * v[1] - pt[1] * v[0]
            elif b["type"] == "e:BodyType.Dynamic" and b["inv_mass"] > 0.0:
                _st_wake(b)
                b["vel"] = (b["vel"][0] + v[0] * b["inv_mass"], b["vel"][1] + v[1] * b["inv_mass"])
                if pt is not None:
                    b["w"] += (pt[0] * v[1] - pt[1] * v[0]) * b["inv_inertia"]
        elif op == "ApplyTorque":
            b = bodies.get(_st_word(ins[k]))
            if b is not None:
                _st_wake(b)
                b["torque"] += tok_f(ins[k + 1])
            k += 2
        elif op == "RemoveBody":
            h = _st_word(ins[k])
            k += 1
            wake_neighbours(h)
            bodies.remove(h)
            if h in lists:
                for ch in lists.pop(h):
                    colls.remove(ch)
            joints.retain(lambda j, h=h: j["a"] != h and j["b"] != h)
            out.append("i:0")
        elif op == "RemoveCollider":
            h = _st_word(ins[k])
            k += 1
            c = colls.remove(h)
            if c is None:
                out.append("E:ColliderNotFound")
                continue
            wake_neighbours(c["body"])
            if c["body"] in lists:
                lists[c["body"]] = [x for x in lists[c["body"]] if x != h]
            recompute(c["body"])
            out.append("i:0")
        elif op == "RemoveJoint":
            h = _st_word(ins[k])
            k += 1
            j = joints.remove(h)
            if j is None:
                out.append("E:JointNotFound")
                continue
            for x in (j["a"], j["b"]):
                b = bodies.get(x)
                if b is not None:
                    _st_wake(b)
            out.append("i:0")
        elif op == "GetBodyState":
            b = bodies.get(_st_word(ins[k]))
            k += 1
            if b is None:
                out.append("E:BodyNotFound")
                continue
            out += [_st_h(b["handle"]), b["type"], _st_f(b["pos"][0]), _st_f(b["pos"][1]), _st_f(0.0),
                    _st_f(b["rot"]), _st_f(b["vel"][0]), _st_f(b["vel"][1]), _st_f(0.0), _st_f(b["w"]),
                    f"i:{int(b['sleeping'])}"]
        elif op == "SetBodyState":
            b = bodies.get(_st_word(ins[k]))
            s = ins[k + 1:k + 12]
            k += 12
            if b is None:
                out.append("E:BodyNotFound")
                continue
            b["pos"], b["rot"] = (tok_f(s[2]), tok_f(s[3])), tok_f(s[5])
            b["vel"], b["w"] = (tok_f(s[6]), tok_f(s[7])), tok_f(s[9])
            _st_wake(b)
            out.append("i:0")
        elif op == "SetBodyType":
            h = _st_word(ins[k])
            t = ins[k + 1]
            k += 2
            b = bodies.get(h)
            if b is None:
                out.append("E:BodyNotFound")
                continue
            b["type"] = t
            _st_wake(b)
            if t in ("e:BodyType.Static", "e:BodyType.Kinematic"):
                b["vel"], b["w"] = (0.0, 0.0), 0.0
            recompute(h)
            out.append("i:0")
        elif op == "BodyCount":
            out.append(f"i:{len(bodies.values())}")
        else:
            fail(f"b2_state: unknown op {op}")
            return out
    if k != len(ins):
        fail(f"b2_state: {len(ins) - k} input tokens left over")
    # The dump, in dump2d.rs's order.
    out += bodies.tokens() + [f"v:{len(bodies.values())}"]
    for h, b in bodies.values():
        out += [_st_h(h), b["type"]] + [_st_f(x) for x in (*b["pos"], b["rot"], *b["vel"], b["w"], b["ld"], b["ad"])]
        out += [f"i:{int(b['fixed'])}"] + [_st_f(x) for x in (b["gs"], *b["force"], b["torque"], b["mass"],
                                                               b["inv_mass"], b["inertia"], b["inv_inertia"],
                                                               0.0, 0.0, 0.0, *b["prev"], b["prev_rot"])]
        out += [f"i:{int(b['sleeping'])}", _st_f(b["sleep_timer"]), "i:0"]
    out += colls.tokens() + [f"v:{len(colls.values())}"]
    for h, c in colls.values():
        out += [_st_h(h), _st_h(c["body"])] + c["shape"] + [_st_f(c["off"][0]), _st_f(c["off"][1]), _st_f(0.0)]
        out += c["mat"] + [c["sensor"], c["mass_tok"], c["layer"], c["mask"]]
    out += joints.tokens() + [f"v:{len(joints.values())}"]
    for h, j in joints.values():
        out += [_st_h(h), _st_h(j["a"]), _st_h(j["b"])] + j["jt"] + j["anchors"] + j["motor"]
        out += [j["damping"], j["brk"], _st_f(j["ref"])]
    out.append(f"v:{len(lists)}")
    for h in sorted(lists):
        out += [_st_h(h), f"v:{len(lists[h])}"] + [_st_h(x) for x in lists[h]]
    out += ["v:0", "v:0", "v:0", "v:0", "i:0"]
    return out


def check_b2_state(rows):
    """Replays every state script on the model and compares its results and dump."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn not in ("b2::state_script", "b2::state_script_orphan"):
            fail(f"b2_state: unknown function {fn}")
            continue
        want = _b2_state_row(ins)
        if not _same_tokens(outs, want):
            first = next((i for i, (a, b) in enumerate(zip(outs, want)) if a != b), min(len(outs), len(want)))
            fail(f"b2_state: {note[:120]}: the model differs at output {first}: "
                 f"{' '.join(outs[first:first + 4])} vs {' '.join(want[first:first + 4])}")
        counted += 1
    return counted


# ── single steps (backend_2d/solver.rs's sequential-impulse step, backend_2d/joints.rs; M3b–c) ──────
# A b2_step or b2_contact row's input is a backend dump; the transcription parses it, steps it as
# PhysicsWorld::step does (sub-steps, each recording the touching pairs, then finish_step), and writes the
# dump and the frame's events. Every M3b scene (b2_step) has collision mask 0 on every collider, so its
# contact stages are empty (the check asserts it); the M3c scenes (b2_contact) touch. sin, cos and hypot
# are parameters: glibc's for the check, MPFR's for the corrected oracle.
#
# The broadphase is brute force over every collider pair: both of Rust's broadphases report a superset
# of the pairs whose boxes overlap (the tree's fattened boxes contain the tight ones; overlapping boxes
# share a hash cell, cells being monotone in the coordinate, and a non-finite or oversized box pairs with
# everything), and the overlap test then keeps exactly those, so which one ran doesn't change the set.

B2_SLEEP_VELOCITY = 0.01
B2_SLEEP_TIME = 0.5
B2_JOINT_BAUMGARTE = 0.2
B2_MANIFOLD_MATCH = 0.02
B2_MAX_MANIFOLD_POINTS = 4
B2_WARM_START = 0.95
B2_RESTITUTION_VELOCITY = 1.0
B2_MAX_CONDITION = 1000.0
B2_STATIC = "e:BodyType.Static"
B2_DYNAMIC = "e:BodyType.Dynamic"
B2_RULE_ORDER = {"e:CombineRule.Min": 0, "e:CombineRule.Average": 1, "e:CombineRule.Multiply": 2,
                 "e:CombineRule.Max": 3}


def _dump_arena(t, k):
    n = int(t[k][2:])
    slots = [[tok_i(t[k + 1 + 3 * i]) == 1, tok_i(t[k + 2 + 3 * i]),
              None if t[k + 3 + 3 * i] == "n:" else tok_i(t[k + 3 + 3 * i]), None] for i in range(n)]
    k += 1 + 3 * n
    a = _Arena()
    a.slots = slots
    a.free_head = None if t[k] == "n:" else tok_i(t[k])
    a.stale = tok_i(t[k + 1]) == 1
    return a, k + 2


def _dump_parse(t, k):
    """A backend dump at token k: (state dict, token after it)."""
    st = {}
    st["body_arena"], k = _dump_arena(t, k)
    nb = int(t[k][2:])
    k += 1
    for _ in range(nb):
        x = t[k:k + 29]
        b = {"handle": _st_word(x[0]), "type": x[1], "pos": [tok_f(x[2]), tok_f(x[3])], "rot": tok_f(x[4]),
             "vel": [tok_f(x[5]), tok_f(x[6])], "w": tok_f(x[7]), "ld": tok_f(x[8]), "ad": tok_f(x[9]),
             "fixed": tok_i(x[10]) == 1, "gs": tok_f(x[11]), "force": [tok_f(x[12]), tok_f(x[13])],
             "torque": tok_f(x[14]), "mass": tok_f(x[15]), "inv_mass": tok_f(x[16]), "inertia": tok_f(x[17]),
             "inv_inertia": tok_f(x[18]), "pv": [tok_f(x[19]), tok_f(x[20])], "pw": tok_f(x[21]),
             "prev": [tok_f(x[22]), tok_f(x[23])], "prev_rot": tok_f(x[24]), "sleeping": tok_i(x[25]) == 1,
             "sleep_timer": tok_f(x[26]), "island": tok_i(x[27])}
        st["body_arena"].slots[b["handle"] & 0xFFFFFFFF][3] = b
        k += 28
    st["coll_start"] = k
    st["coll_arena"], k = _dump_arena(t, k)
    nc = int(t[k][2:])
    k += 1
    masks = []
    for _ in range(nc):
        h = _st_word(t[k])
        shape, k2 = _np_shape(t, k + 2)
        if k2 != _st_shape_end(t, k + 2):
            fail("b2_step: the shape parsers disagree")
        mt = t[k2 + 3:k2 + 10]
        c = {"handle": h, "body": _st_word(t[k + 1]), "shape": shape, "off": (tok_f(t[k2]), tok_f(t[k2 + 1])),
             "mat": {"fric": tok_f(mt[0]), "rest": tok_f(mt[1]),
                     "sfric": None if mt[3] == "n:" else tok_f(mt[3]), "roll": tok_f(mt[4]), "fc": mt[5],
                     "rc": mt[6]},
             "sensor": tok_i(t[k2 + 10]) == 1, "layer": tok_i(t[k2 + 12]), "mask": tok_i(t[k2 + 13])}
        masks.append(c["mask"])
        st["coll_arena"].slots[h & 0xFFFFFFFF][3] = c
        k = k2 + 14
    st["masks"] = masks
    st["coll_tokens"] = t[st["coll_start"]:k]
    st["joint_arena"], k = _dump_arena(t, k)
    nj = int(t[k][2:])
    k += 1
    for _ in range(nj):
        h = _st_word(t[k])
        ke = _st_joint_type_end(t, k + 3)
        j = {"handle": h, "a": _st_word(t[k + 1]), "b": _st_word(t[k + 2]), "jt": t[k + 3:ke],
             "la": (tok_f(t[ke]), tok_f(t[ke + 1])), "lb": (tok_f(t[ke + 2]), tok_f(t[ke + 3]))}
        k = ke + 4
        if t[k] == "n:":
            j["motor"], k = None, k + 1
        else:
            j["motor"], k = (tok_f(t[k]), tok_f(t[k + 1])), k + 2
        j["damping"] = tok_f(t[k])
        j["brk"] = None if t[k + 1] == "n:" else tok_f(t[k + 1])
        j["ref"] = tok_f(t[k + 2])
        k += 3
        st["joint_arena"].slots[h & 0xFFFFFFFF][3] = j
    s0 = k
    nbc = int(t[k][2:])
    k += 1
    for _ in range(nbc):
        k += 2 + int(t[k + 1][2:])
    st["body_colliders"] = t[s0:k]
    nm = int(t[k][2:])
    k += 1
    ms = []
    for _ in range(nm):
        m = {"ca": _st_word(t[k]), "cb": _st_word(t[k + 1]), "ba": _st_word(t[k + 2]), "bb": _st_word(t[k + 3]),
             "n": (tok_f(t[k + 4]), tok_f(t[k + 5])), "pts": []}
        np_ = int(t[k + 6][2:])
        k += 7
        for _ in range(np_):
            x = [tok_f(v) for v in t[k:k + 7]]
            m["pts"].append([(x[0], x[1]), (x[2], x[3]), x[4], x[5], x[6]])
            k += 7
        m["roll"] = tok_f(t[k])
        k += 1
        ms.append(m)
    st["manifolds"] = ms
    tr = {}
    for part in ("prev", "current", "seen"):
        n = int(t[k][2:])
        tr[part] = [(_st_word(t[k + 1 + 2 * i]), _st_word(t[k + 2 + 2 * i])) for i in range(n)]
        k += 1 + 2 * n
    tr["recorded"] = tok_i(t[k]) == 1
    st["tracker"] = tr
    return st, k + 1


def _dump_emit(st):
    out = st["body_arena"].tokens() + [f"v:{len(st['body_arena'].values())}"]
    for _, b in st["body_arena"].values():
        out += [_st_h(b["handle"]), b["type"]] + [_st_f(x) for x in (*b["pos"], b["rot"], *b["vel"], b["w"], b["ld"], b["ad"])]
        out += [f"i:{int(b['fixed'])}"] + [_st_f(x) for x in (b["gs"], *b["force"], b["torque"], b["mass"],
                                                               b["inv_mass"], b["inertia"], b["inv_inertia"],
                                                               *b["pv"], b["pw"], *b["prev"], b["prev_rot"])]
        out += [f"i:{int(b['sleeping'])}", _st_f(b["sleep_timer"]), f"i:{b['island']}"]
    out += st["coll_tokens"]
    out += st["joint_arena"].tokens() + [f"v:{len(st['joint_arena'].values())}"]
    for h, j in st["joint_arena"].values():
        out += [_st_h(h), _st_h(j["a"]), _st_h(j["b"])] + j["jt"]
        out += [_st_f(x) for x in (*j["la"], *j["lb"])]
        out += ["n:"] if j["motor"] is None else [_st_f(j["motor"][0]), _st_f(j["motor"][1])]
        out += [_st_f(j["damping"]), "n:" if j["brk"] is None else _st_f(j["brk"]), _st_f(j["ref"])]
    out += st["body_colliders"] + [f"v:{len(st['manifolds'])}"]
    for m in st["manifolds"]:
        out += [_st_h(m["ca"]), _st_h(m["cb"]), _st_h(m["ba"]), _st_h(m["bb"]), _st_f(m["n"][0]), _st_f(m["n"][1])]
        out += [f"v:{len(m['pts'])}"]
        for la, lb, ni, ti, d in m["pts"]:
            out += [_st_f(x) for x in (la[0], la[1], lb[0], lb[1], ni, ti, d)]
        out += [_st_f(m["roll"])]
    tr = st["tracker"]
    for part in ("prev", "current", "seen"):
        out += [f"v:{len(tr[part])}"]
        for a, b in tr[part]:
            out += [_st_h(a), _st_h(b)]
    out += [f"i:{int(tr['recorded'])}"]
    return out


def _sim(b):
    """`solver_inv_mass`: (inv_mass, inv_inertia) of an awake dynamic body, else zeros."""
    if b["type"] == B2_DYNAMIC and not b["sleeping"]:
        return b["inv_mass"], b["inv_inertia"]
    return 0.0, 0.0


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _at(v, r):
    """`SolverVel::at`: the velocity of the point at offset r of a body moving at v = [vx, vy, w]."""
    return (v[0] - v[2] * r[1], v[1] + v[2] * r[0])


def _rel(va, vb, ra, rb, d):
    """dot2(sub2(vb.at(rb), va.at(ra)), d)."""
    pb, pa = _at(vb, rb), _at(va, ra)
    return (pb[0] - pa[0]) * d[0] + (pb[1] - pa[1]) * d[1]


def _apply(v, sign, imp, r, im, ii):
    """`SolverVel::apply`."""
    v[0] += sign * imp[0] * im
    v[1] += sign * imp[1] * im
    v[2] += sign * _cross(r, imp) * ii


def _combine2(ra, rb, a, b):
    """`combine_property`: the higher-priority rule of the two, applied to a and b."""
    rule = rb if B2_RULE_ORDER[rb] > B2_RULE_ORDER[ra] else ra
    return combine(rule, a, b)


def _union(a, b):
    return sorted(set(a) | set(b))


class _Step2d:
    """The sequential-impulse step (contacts, joints, integration, islands and sleep) on the given sin,
    cos and hypot."""

    def __init__(self, sin, cos, hyp):
        self.sin, self.cos, self.hyp = sin, cos, hyp
        self.geo = _Ray(sin, cos, hyp)

    # ── the contact pipeline (solver.rs) ──

    def broadphase(self, st, dt):
        """The kept pairs, sorted: brute force over every collider pair (see the section comment)."""
        bodies, colls = st["body_arena"], st["coll_arena"]
        boxes = {}
        for h, c in colls.values():
            b = bodies.get(c["body"])
            if b is None:
                continue
            x0, y0, x1, y1 = self.geo.world_aabb(c["shape"], c["off"], b["pos"], b["rot"])
            dx, dy = b["vel"][0] * dt, b["vel"][1] * dt
            if dx > 0.0:
                x1 += dx
            else:
                x0 += dx
            if dy > 0.0:
                y1 += dy
            else:
                y0 += dy
            boxes[h] = (x0, y0, x1, y1)
        hs = sorted(boxes)
        pairs = []
        for i, ha in enumerate(hs):
            for hb in hs[i + 1:]:
                ca, cb = colls.get(ha), colls.get(hb)
                if ca["body"] == cb["body"]:
                    continue
                ba, bb = bodies.get(ca["body"]), bodies.get(cb["body"])
                if ba is not None and bb is not None and ba["type"] == B2_STATIC and bb["type"] == B2_STATIC:
                    continue
                if ca["sensor"] and cb["sensor"]:
                    continue
                if (ca["layer"] & cb["mask"]) == 0 and (cb["layer"] & ca["mask"]) == 0:
                    continue
                a, b = boxes[ha], boxes[hb]
                if a[0] <= b[2] and a[2] >= b[0] and a[1] <= b[3] and a[3] >= b[1]:
                    pairs.append((ha, hb))
        return pairs

    def update_manifolds(self, st, pairs):
        """`narrowphase` then `update_manifolds`."""
        bodies, colls = st["body_arena"], st["coll_arena"]
        fresh = {}

        def pose(h):
            b = bodies.get(h)
            return ((0.0, 0.0), 0.0) if b is None else (b["pos"], b["rot"])

        for ha, hb in pairs:
            ca, cb = colls.get(ha), colls.get(hb)
            ba, bb = bodies.get(ca["body"]), bodies.get(cb["body"])
            if ba is None or bb is None:
                continue
            pa = self.geo.world_pos(ba["pos"], ba["rot"], ca["off"])
            pb = self.geo.world_pos(bb["pos"], bb["rot"], cb["off"])
            for n, d, pt in self.geo.contacts(ca["shape"], pa, ba["rot"], cb["shape"], pb, bb["rot"]):
                key = (ha, hb) if ha <= hb else (hb, ha)
                if key[0] != ha:
                    body_a, body_b, normal = cb["body"], ca["body"], (-n[0], -n[1])
                else:
                    body_a, body_b, normal = ca["body"], cb["body"], n
                (qa, qra), (qb, qrb) = pose(body_a), pose(body_b)
                point = [self.geo.world_to_local(pt, qa, qra), self.geo.world_to_local(pt, qb, qrb), 0.0, 0.0, d]
                if key not in fresh:
                    fresh[key] = {"ca": key[0], "cb": key[1], "ba": body_a, "bb": body_b, "n": normal, "pts": [],
                                  "roll": 0.0}
                m = fresh[key]
                if len(m["pts"]) < B2_MAX_MANIFOLD_POINTS:
                    m["pts"].append(point)
        old = {(m["ca"], m["cb"]): m for m in st["manifolds"]}
        for key in sorted(fresh):
            m, o = fresh[key], old.get(key)
            if o is None:
                continue
            if o["n"][0] * m["n"][0] + o["n"][1] * m["n"][1] < 0.9:
                continue
            m["roll"] = o["roll"]
            single = len(m["pts"]) == 1 and len(o["pts"]) == 1
            used = [False] * B2_MAX_MANIFOLD_POINTS
            for p in m["pts"]:
                best, best_d = None, B2_MANIFOLD_MATCH * B2_MANIFOLD_MATCH
                for i, op in enumerate(o["pts"]):
                    if i >= len(used) or used[i]:
                        continue
                    ex, ey = op[0][0] - p[0][0], op[0][1] - p[0][1]
                    fx, fy = op[1][0] - p[1][0], op[1][1] - p[1][1]
                    dd = rust_min(ex * ex + ey * ey, fx * fx + fy * fy)
                    if dd < best_d or single:
                        best_d, best = dd, i
                if best is not None:
                    used[best] = True
                    p[2], p[3] = o["pts"][best][2], o["pts"][best][3]
        st["manifolds"] = [fresh[k] for k in sorted(fresh)]

    @staticmethod
    def wake_on_contact(st):
        bodies = st["body_arena"]

        def moving(b):
            t = B2_SLEEP_VELOCITY
            return (b is not None and not b["sleeping"] and b["type"] == B2_DYNAMIC
                    and (abs(b["vel"][0]) > t or abs(b["vel"][1]) > t or abs(b["w"]) > t))

        for m in st["manifolds"]:
            ba, bb = bodies.get(m["ba"]), bodies.get(m["bb"])
            a_sl = ba is not None and ba["sleeping"]
            b_sl = bb is not None and bb["sleeping"]
            a_mv, b_mv = moving(ba), moving(bb)
            if a_sl and b_mv:
                _st_wake(ba)
            if b_sl and a_mv:
                _st_wake(bb)

    @staticmethod
    def sensor(st, m, missing):
        ca, cb = st["coll_arena"].get(m["ca"]), st["coll_arena"].get(m["cb"])
        if ca is None or cb is None:
            return missing
        return ca["sensor"] or cb["sensor"]

    def arms(self, ba, bb, p):
        cp = self.geo.local_to_world(p[0], ba["pos"], ba["rot"])
        return (cp[0] - ba["pos"][0], cp[1] - ba["pos"][1]), (cp[0] - bb["pos"][0], cp[1] - bb["pos"][1])

    def warm_start(self, st):
        bodies = st["body_arena"]
        pending = []
        for m in st["manifolds"]:
            m["roll"] *= B2_WARM_START
            for p in m["pts"]:
                p[2] *= B2_WARM_START
                p[3] *= B2_WARM_START
            if self.sensor(st, m, True):
                continue
            ba, bb = bodies.get(m["ba"]), bodies.get(m["bb"])
            if ba is None or bb is None:
                continue
            n = m["n"]
            t = (-n[1], n[0])
            for p in m["pts"]:
                if abs(p[2]) < B2_EPSILON and abs(p[3]) < B2_EPSILON:
                    continue
                ra, rb = self.arms(ba, bb, p)
                imp = (p[2] * n[0] + p[3] * t[0], p[2] * n[1] + p[3] * t[1])
                pending.append((m["ba"], m["bb"], imp, _cross(ra, imp), _cross(rb, imp)))
            if m["roll"] != 0.0:
                pending.append((m["ba"], m["bb"], (0.0, 0.0), m["roll"], m["roll"]))
        for a, b, imp, aa, ab in pending:
            ba = bodies.get(a)
            if ba is not None:
                im, ii = _sim(ba)
                ba["vel"] = [ba["vel"][0] - imp[0] * im, ba["vel"][1] - imp[1] * im]
                ba["w"] -= aa * ii
            bb = bodies.get(b)
            if bb is not None:
                im, ii = _sim(bb)
                bb["vel"] = [bb["vel"][0] + imp[0] * im, bb["vel"][1] + imp[1] * im]
                bb["w"] += ab * ii

    def prepare_contacts(self, st):
        bodies, colls, ms = st["body_arena"], st["coll_arena"], st["manifolds"]
        level = {}
        for _, b in bodies.values():
            if b["type"] == B2_STATIC or b["type"] != B2_DYNAMIC:
                level[b["handle"]] = 0
        changed = True
        while changed:
            changed = False
            for m in ms:
                la, lb = level.get(m["ba"]), level.get(m["bb"])
                if la is not None and lb is None:
                    level[m["bb"]], changed = la + 1, True
                elif la is None and lb is not None:
                    level[m["ba"]], changed = lb + 1, True
                elif la is not None and lb is not None:
                    if la + 1 < lb:
                        level[m["bb"]], changed = la + 1, True
                    elif lb + 1 < la:
                        level[m["ba"]], changed = lb + 1, True
        none = 0xFFFFFFFF
        order = sorted(range(len(ms)), key=lambda i: min(level.get(ms[i]["ba"], none), level.get(ms[i]["bb"], none)))
        prep = []
        for i in order:
            m = ms[i]
            ca, cb = colls.get(m["ca"]), colls.get(m["cb"])
            if ca is not None and cb is not None:
                a, b = ca["mat"], cb["mat"]
                rest = _combine2(a["rc"], b["rc"], a["rest"], b["rest"])
                fric = _combine2(a["fc"], b["fc"], a["fric"], b["fric"])
                sa = a["sfric"] if a["sfric"] is not None else a["fric"] * 1.5
                sb = b["sfric"] if b["sfric"] is not None else b["fric"] * 1.5
                sfric = _combine2(a["fc"], b["fc"], sa, sb)
                roll = (a["roll"] + b["roll"]) * 0.5
                sensor = ca["sensor"] or cb["sensor"]
            else:
                rest = fric = sfric = roll = 0.0
                sensor = False
            bias = [0.0] * B2_MAX_MANIFOLD_POINTS
            if not (rest <= 0.0 or sensor):
                ba, bb = bodies.get(m["ba"]), bodies.get(m["bb"])
                if ba is not None and bb is not None:
                    va, vb = [ba["vel"][0], ba["vel"][1], ba["w"]], [bb["vel"][0], bb["vel"][1], bb["w"]]
                    for pi, p in enumerate(m["pts"][:B2_MAX_MANIFOLD_POINTS]):
                        ra, rb = self.arms(ba, bb, p)
                        vn = _rel(va, vb, ra, rb, m["n"])
                        if vn < -B2_RESTITUTION_VELOCITY:
                            bias[pi] = -rest * vn
            prep.append((i, fric, sfric, roll, sensor, bias))
        return prep

    def solve_contacts_once(self, st, prep):
        bodies, ms = st["body_arena"], st["manifolds"]
        for i, fric, sfric, roll, sensor, bias in prep:
            if sensor:
                continue
            m = ms[i]
            ba, bb = bodies.get(m["ba"]), bodies.get(m["bb"])
            if ba is None or bb is None:
                continue
            ima, iia = _sim(ba)
            imb, iib = _sim(bb)
            if ima == 0.0 and imb == 0.0:
                continue
            va, vb = [ba["vel"][0], ba["vel"][1], ba["w"]], [bb["vel"][0], bb["vel"][1], bb["w"]]
            n = m["n"]
            t = (-n[1], n[0])
            np_ = min(len(m["pts"]), B2_MAX_MANIFOLD_POINTS)
            ra, rb, acc_n, acc_t = [], [], [], []
            for p in m["pts"][:np_]:
                a, b = self.arms(ba, bb, p)
                ra.append(a)
                rb.append(b)
                acc_n.append(p[2])
                acc_t.append(p[3])
            acc_roll = m["roll"]
            a_dyn = ima > 0.0 or iia > 0.0
            b_dyn = imb > 0.0 or iib > 0.0

            def eff(r_a, r_b, d):
                ca, cb = _cross(r_a, d), _cross(r_b, d)
                return ima + imb + ca * ca * iia + cb * cb * iib

            solved = False
            if np_ == 2:
                rn = ((_cross(ra[0], n), _cross(rb[0], n)), (_cross(ra[1], n), _cross(rb[1], n)))
                k11, k22 = eff(ra[0], rb[0], n), eff(ra[1], rb[1], n)
                k12 = ima + imb + rn[0][0] * rn[1][0] * iia + rn[0][1] * rn[1][1] * iib
                det = k11 * k22 - k12 * k12
                if k11 > B2_EPSILON and k22 > B2_EPSILON and k11 * k11 < B2_MAX_CONDITION * det:
                    solved = True
                    a0, a1 = acc_n
                    vn0, vn1 = _rel(va, vb, ra[0], rb[0], n), _rel(va, vb, ra[1], rb[1], n)
                    b0 = vn0 - bias[0] - (k11 * a0 + k12 * a1)
                    b1 = vn1 - bias[1] - (k12 * a0 + k22 * a1)
                    x0 = -(k22 * b0 - k12 * b1) / det
                    x1 = -(-k12 * b0 + k11 * b1) / det
                    if x0 >= 0.0 and x1 >= 0.0:
                        x = [x0, x1]
                    else:
                        x0 = -b0 / k11
                        if x0 >= 0.0 and k12 * x0 + b1 >= 0.0:
                            x = [x0, 0.0]
                        else:
                            x1 = -b1 / k22
                            if x1 >= 0.0 and k12 * x1 + b0 >= 0.0:
                                x = [0.0, x1]
                            elif b0 >= 0.0 and b1 >= 0.0:
                                x = [0.0, 0.0]
                            else:
                                x = None
                    if x is not None:
                        for pi in range(2):
                            d = x[pi] - acc_n[pi]
                            imp = (n[0] * d, n[1] * d)
                            _apply(va, -1.0, imp, ra[pi], ima, iia)
                            _apply(vb, 1.0, imp, rb[pi], imb, iib)
                            acc_n[pi] = x[pi]
            if not solved:
                for pi in range(np_):
                    k = eff(ra[pi], rb[pi], n)
                    if k < B2_EPSILON:
                        continue
                    vn = _rel(va, vb, ra[pi], rb[pi], n)
                    new = rust_max(acc_n[pi] - (vn - bias[pi]) / k, 0.0)
                    d = new - acc_n[pi]
                    acc_n[pi] = new
                    imp = (n[0] * d, n[1] * d)
                    _apply(va, -1.0, imp, ra[pi], ima, iia)
                    _apply(vb, 1.0, imp, rb[pi], imb, iib)
            if fric > 0.0 or sfric > 0.0:
                for pi in range(np_):
                    k = eff(ra[pi], rb[pi], t)
                    if k < B2_EPSILON:
                        continue
                    vt = _rel(va, vb, ra[pi], rb[pi], t)
                    mu = sfric if abs(vt) < 0.01 else fric
                    max_f = rust_max(mu * acc_n[pi], 0.0)
                    new = rust_clamp(acc_t[pi] - vt / k, -max_f, max_f)
                    if not math.isfinite(new):
                        new = 0.0
                    d = new - acc_t[pi]
                    acc_t[pi] = new
                    imp = (t[0] * d, t[1] * d)
                    _apply(va, -1.0, imp, ra[pi], ima, iia)
                    _apply(vb, 1.0, imp, rb[pi], imb, iib)
            if roll > 0.0 and iia + iib > B2_EPSILON:
                total = -0.0
                for x in acc_n[:np_]:
                    total += x
                max_r = rust_max(roll * total, 0.0)
                new = rust_clamp(acc_roll - (vb[2] - va[2]) / (iia + iib), -max_r, max_r)
                if not math.isfinite(new):
                    new = 0.0
                d = new - acc_roll
                acc_roll = new
                va[2] -= d * iia
                vb[2] += d * iib
            for pi in range(np_):
                m["pts"][pi][2], m["pts"][pi][3] = acc_n[pi], acc_t[pi]
            m["roll"] = acc_roll
            if a_dyn:
                ba["vel"], ba["w"] = [va[0], va[1]], va[2]
            if b_dyn:
                bb["vel"], bb["w"] = [vb[0], vb[1]], vb[2]

    def solve_positions(self, st, cfg, dt):
        bodies = st["body_arena"]
        for _, b in bodies.values():
            b["pv"], b["pw"] = [0.0, 0.0], 0.0
        if dt <= B2_EPSILON:
            return
        if cfg["freq"] > 0.0:
            omega = 2.0 * math.pi * cfg["freq"]
            d = 2.0 * cfg["ratio"] * omega
            k = omega * omega
            fraction = dt * k / (d + dt * k)
        else:
            fraction = cfg["corr"]
        if math.isnan(fraction) or fraction <= 0.0:
            return
        fraction = rust_min(fraction, 1.0)
        points = []
        for m in st["manifolds"]:
            if self.sensor(st, m, True):
                continue
            ba, bb = bodies.get(m["ba"]), bodies.get(m["bb"])
            if ba is None or bb is None:
                continue
            for p in m["pts"]:
                pen = p[4] - cfg["slop"]
                if pen <= 0.0:
                    continue
                ra, rb = self.arms(ba, bb, p)
                points.append([m["ba"], m["bb"], ra, rb, m["n"], fraction * pen / dt, 0.0])
        for _ in range(max(cfg["piters"], 1)):
            for pt in points:
                ba, bb = bodies.get(pt[0]), bodies.get(pt[1])
                if ba is None or bb is None:
                    continue
                ima, iia = _sim(ba)
                imb, iib = _sim(bb)
                va, vb = [ba["pv"][0], ba["pv"][1], ba["pw"]], [bb["pv"][0], bb["pv"][1], bb["pw"]]
                n = pt[4]
                ca, cb = _cross(pt[2], n), _cross(pt[3], n)
                k = ima + imb + ca * ca * iia + cb * cb * iib
                if k < B2_EPSILON:
                    continue
                vn = _rel(va, vb, pt[2], pt[3], n)
                new = rust_max(pt[6] + (pt[5] - vn) / k, 0.0)
                d = new - pt[6]
                pt[6] = new
                imp = (n[0] * d, n[1] * d)
                _apply(va, -1.0, imp, pt[2], ima, iia)
                _apply(vb, 1.0, imp, pt[3], imb, iib)
                if ima > 0.0:
                    ba["pv"], ba["pw"] = [va[0], va[1]], va[2]
                if imb > 0.0:
                    bb["pv"], bb["pw"] = [vb[0], vb[1]], vb[2]

    @staticmethod
    def record(st):
        """`ContactTracker::record_substep` with the manifold keys."""
        tr = st["tracker"]
        if tr["recorded"] and tr["current"]:
            tr["seen"] = _union(tr["seen"], tr["current"])
        tr["current"] = sorted(set((m["ca"], m["cb"]) for m in st["manifolds"]))
        tr["recorded"] = True

    @staticmethod
    def finish(st):
        """`ContactTracker::finish`: the frame's events as (kind, a, b)."""
        tr = st["tracker"]
        prev, cur = set(tr["prev"]), set(tr["current"])
        ev = [("Ongoing" if p in prev else "Started", p) for p in tr["current"]]
        for p in (tr["prev"] if not tr["seen"] else _union(tr["prev"], tr["seen"])):
            if p in cur:
                continue
            if p not in prev:
                ev.append(("Started", p))
            ev.append(("Stopped", p))
        tr["prev"], tr["current"], tr["seen"], tr["recorded"] = tr["current"], [], [], False
        return ev

    def rotate(self, v, rot):
        s, c = self.sin(rot), self.cos(rot)
        return (c * v[0] - s * v[1], s * v[0] + c * v[1])

    def normalize(self, v):
        ln = self.hyp(v[0], v[1])
        return (v[0] / ln, v[1] / ln) if (ln > B2_EPSILON and math.isfinite(ln)) else None

    @staticmethod
    def soft(k, c, dt):
        k = rust_max(k, 0.0) if math.isfinite(k) else 0.0
        c = rust_max(c, 0.0) if math.isfinite(c) else 0.0
        g = dt * (c + dt * k)
        if not g > B2_EPSILON:
            return None
        gamma = 1.0 / g
        return (gamma, dt * k * gamma)

    @staticmethod
    def cap(mf, dt):
        c = abs(mf * dt)
        return 0.0 if math.isnan(c) else c

    @staticmethod
    def row(lin, aa, ab, a, b, gamma):
        k = (a[1] + b[1]) * (lin[0] * lin[0] + lin[1] * lin[1]) + a[2] * aa * aa + b[2] * ab * ab + gamma
        return [lin[0], lin[1], aa, ab, 1.0 / k if k > B2_EPSILON else 0.0, 0.0, gamma, -math.inf, math.inf, 0.0]

    @staticmethod
    def rigid(r, err, dt):
        r[5] = B2_JOINT_BAUMGARTE * err / dt
        return r

    @staticmethod
    def one_sided(r, c, dt):
        r[5] = c / dt if c > 0.0 else B2_JOINT_BAUMGARTE * c / dt
        r[7] = 0.0
        return r

    @staticmethod
    def point(ra, rb, a, b, gamma):
        ma, ia, mb, ib = a[1], a[2], b[1], b[2]
        k11 = ma + mb + ia * ra[1] * ra[1] + ib * rb[1] * rb[1] + gamma
        k12 = -ia * ra[0] * ra[1] - ib * rb[0] * rb[1]
        k22 = ma + mb + ia * ra[0] * ra[0] + ib * rb[0] * rb[0] + gamma
        det = k11 * k22 - k12 * k12
        if math.isnan(det) or abs(det) <= B2_EPSILON or not math.isfinite(det):
            return None
        inv = 1.0 / det
        return {"ra": ra, "rb": rb, "ik": [k22 * inv, -k12 * inv, -k12 * inv, k11 * inv], "bias": [0.0, 0.0],
                "gamma": gamma, "max": math.inf, "acc": [0.0, 0.0]}

    @staticmethod
    def bodyref(h, b):
        if b is None:
            return (None, 0.0, 0.0)
        if b["type"] == "e:BodyType.Dynamic" and not b["sleeping"]:
            return (h, b["inv_mass"], b["inv_inertia"])
        return (h, 0.0, 0.0)

    def prepare_joint(self, st, h, j, dt):
        ba = st["body_arena"].get(j["a"])
        bb = st["body_arena"].get(j["b"])
        a = self.bodyref(j["a"], ba)
        b = self.bodyref(j["b"], bb)
        kind = j["jt"][0].split(".", 1)[1]
        f = [tok_f(x) for x in j["jt"][1:] if x.startswith("f:")]
        if kind == "Mouse":
            if ba is None:
                return None
            anchor = self.rotate(j["la"], ba["rot"])
            ground = (None, 0.0, 0.0)
            gb = self.soft(f[3], f[4], dt) or (0.0, 0.0)
            p = self.point((0.0, 0.0), anchor, ground, a, gb[0])
            if p is None:
                return None
            c = (ba["pos"][0] + anchor[0] - f[0], ba["pos"][1] + anchor[1] - f[1])
            p["bias"] = [c[0] * gb[1], c[1] * gb[1]]
            p["max"] = self.cap(f[5], dt)
            return {"handle": h, "a": ground, "b": a, "point": p, "rows": [], "break_row": None, "brk": j["brk"]}
        pa, rot_a = (ba["pos"], ba["rot"]) if ba is not None else ((0.0, 0.0), 0.0)
        pb, rot_b = (bb["pos"], bb["rot"]) if bb is not None else ((0.0, 0.0), 0.0)
        if a[1] == 0.0 and b[1] == 0.0 and a[2] == 0.0 and b[2] == 0.0:
            return None
        ra = self.rotate(j["la"], rot_a) if ba is not None else (0.0, 0.0)
        rb = self.rotate(j["lb"], rot_b) if bb is not None else (0.0, 0.0)
        anchor_a = (pa[0] + ra[0], pa[1] + ra[1]) if ba is not None else j["la"]
        anchor_b = (pb[0] + rb[0], pb[1] + rb[1]) if bb is not None else j["lb"]
        d = (anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1])
        angle = rot_b - rot_a
        point, rows, break_row = None, [], None

        def cross(u, v):
            return u[0] * v[1] - u[1] * v[0]

        def dot(u, v):
            return u[0] * v[0] + u[1] * v[1]

        def rigid_point(gamma):
            p = self.point(ra, rb, a, b, gamma)
            if p is not None:
                p["bias"] = [B2_JOINT_BAUMGARTE * d[0] / dt, B2_JOINT_BAUMGARTE * d[1] / dt]
            return p

        def angular():
            return self.row((0.0, 0.0), 1.0, 1.0, a, b, 0.0)

        def anchor_row(u, gamma):
            return self.row(u, cross(ra, u), cross(rb, u), a, b, gamma)

        def axis_row(u):
            arm = (d[0] + ra[0], d[1] + ra[1])
            return self.row(u, cross(arm, u), cross(rb, u), a, b, 0.0)

        jt = j["jt"]
        if kind == "Fixed":
            point = rigid_point(0.0)
            rows.append(self.rigid(angular(), angle - j["ref"], dt))
        elif kind == "Revolute":
            point = rigid_point(0.0)
            if jt[3] != "n:":
                lo, hi = tok_f(jt[3]), tok_f(jt[4])
                rows.append(self.one_sided(angular(), angle - lo, dt))
                rows.append(self.one_sided(self.row((0.0, 0.0), -1.0, -1.0, a, b, 0.0), hi - angle, dt))
            if j["motor"] is not None:
                r = angular()
                r[5] = -j["motor"][0]
                c = self.cap(j["motor"][1], dt)
                r[7], r[8] = -c, c
                rows.append(r)
        elif kind in ("Prismatic", "Wheel"):
            ax = self.normalize(self.rotate((tok_f(jt[1]), tok_f(jt[2])), rot_a))
            if ax is None:
                return None
            perp = (-ax[1], ax[0])
            break_row = len(rows)
            rows.append(self.rigid(axis_row(perp), dot(d, perp), dt))
            if kind == "Prismatic":
                rows.append(self.rigid(angular(), angle - j["ref"], dt))
                tr = dot(d, ax)
                if jt[3] != "n:":
                    lo, hi = tok_f(jt[3]), tok_f(jt[4])
                    rows.append(self.one_sided(axis_row(ax), tr - lo, dt))
                    up = axis_row(ax)
                    up[0], up[1], up[2], up[3] = -ax[0], -ax[1], -up[2], -up[3]
                    rows.append(self.one_sided(up, hi - tr, dt))
                if j["motor"] is not None:
                    r = axis_row(ax)
                    r[5] = -j["motor"][0]
                    c = self.cap(j["motor"][1], dt)
                    r[7], r[8] = -c, c
                    rows.append(r)
            else:
                gb = self.soft(tok_f(jt[3]), tok_f(jt[4]), dt)
                if gb is not None:
                    r = axis_row(ax)
                    r = self.row((r[0], r[1]), r[2], r[3], a, b, gb[0])
                    r[5] = dot(d, ax) * gb[1]
                    rows.append(r)
        elif kind == "Spring":
            u = self.normalize(d)
            if u is None:
                return None
            gb = self.soft(tok_f(jt[2]), tok_f(jt[3]), dt)
            if gb is None:
                return None
            r = anchor_row(u, gb[0])
            r[5] = (self.hyp(d[0], d[1]) - tok_f(jt[1])) * gb[1]
            break_row = len(rows)
            rows.append(r)
        elif kind == "Distance":
            u = self.normalize(d)
            if u is None:
                return None
            break_row = len(rows)
            rows.append(self.rigid(anchor_row(u, 0.0), self.hyp(d[0], d[1]) - tok_f(jt[1]), dt))
        elif kind == "Rope":
            u = self.normalize(d)
            if u is None:
                return None
            r = self.row((-u[0], -u[1]), -cross(ra, u), -cross(rb, u), a, b, 0.0)
            break_row = len(rows)
            rows.append(self.one_sided(r, tok_f(jt[1]) - self.hyp(d[0], d[1]), dt))
        if j["damping"] > 0.0 and math.isfinite(j["damping"]):
            gamma = 1.0 / (dt * j["damping"])
            if kind in ("Revolute", "Wheel"):
                rows.append(self.row((0.0, 0.0), 1.0, 1.0, a, b, gamma))
            elif kind == "Prismatic":
                ax2 = self.normalize(self.rotate((tok_f(jt[1]), tok_f(jt[2])), rot_a))
                if ax2 is not None:
                    r = axis_row(ax2)
                    rows.append(self.row((r[0], r[1]), r[2], r[3], a, b, gamma))
            elif kind in ("Distance", "Rope"):
                u2 = self.normalize(d)
                if u2 is not None:
                    rows.append(anchor_row(u2, gamma))
        return {"handle": h, "a": a, "b": b, "point": point, "rows": rows, "break_row": break_row, "brk": j["brk"]}

    @staticmethod
    def solve_row(r, va, vb, a, b):
        if r[4] == 0.0:
            return
        cdot = r[0] * (vb[0] - va[0]) + r[1] * (vb[1] - va[1]) + r[3] * vb[2] - r[2] * va[2]
        lam = -r[4] * (cdot + r[5] + r[6] * r[9])
        new = rust_clamp(r[9] + lam, r[7], r[8])
        if not math.isfinite(new):
            new = 0.0
        d = new - r[9]
        r[9] = new
        va[0] -= r[0] * d * a[1]
        va[1] -= r[1] * d * a[1]
        va[2] -= r[2] * d * a[2]
        vb[0] += r[0] * d * b[1]
        vb[1] += r[1] * d * b[1]
        vb[2] += r[3] * d * b[2]

    def solve_point(self, p, va, vb, a, b):
        ra, rb = p["ra"], p["rb"]
        pa = (va[0] - va[2] * ra[1], va[1] + va[2] * ra[0])
        pb = (vb[0] - vb[2] * rb[1], vb[1] + vb[2] * rb[0])
        cd = (pb[0] - pa[0], pb[1] - pa[1])
        rhs = (cd[0] + p["bias"][0] + p["gamma"] * p["acc"][0], cd[1] + p["bias"][1] + p["gamma"] * p["acc"][1])
        ik = p["ik"]
        lam = (-(ik[0] * rhs[0] + ik[1] * rhs[1]), -(ik[2] * rhs[0] + ik[3] * rhs[1]))
        new = [p["acc"][0] + lam[0], p["acc"][1] + lam[1]]
        mag = self.hyp(new[0], new[1])
        if mag > p["max"]:
            new = [new[0] * p["max"] / mag, new[1] * p["max"] / mag]
        if not (math.isfinite(new[0]) and math.isfinite(new[1])):
            new = [0.0, 0.0]
        d = (new[0] - p["acc"][0], new[1] - p["acc"][1])
        p["acc"] = new
        va[0] -= d[0] * a[1]
        va[1] -= d[1] * a[1]
        va[2] -= (ra[0] * d[1] - ra[1] * d[0]) * a[2]
        vb[0] += d[0] * b[1]
        vb[1] += d[1] * b[1]
        vb[2] += (rb[0] * d[1] - rb[1] * d[0]) * b[2]

    def substep(self, st, cfg, dt):
        bodies = st["body_arena"]
        g = (cfg["g"][0], cfg["g"][1])
        for _, b in bodies.values():
            if b["type"] != "e:BodyType.Dynamic" or b["inv_mass"] == 0.0 or b["sleeping"]:
                continue
            vx, vy = b["vel"]
            vx += g[0] * b["gs"] * dt
            vy += g[1] * b["gs"] * dt
            vx += b["force"][0] * b["inv_mass"] * dt
            vy += b["force"][1] * b["inv_mass"] * dt
            if not b["fixed"]:
                b["w"] += b["torque"] * b["inv_inertia"] * dt
            vx *= 1.0 / (1.0 + dt * b["ld"])
            vy *= 1.0 / (1.0 + dt * b["ld"])
            b["w"] *= 1.0 / (1.0 + dt * b["ad"])
            sq = vx * vx + vy * vy
            if sq > cfg["maxv"] * cfg["maxv"]:
                scale = cfg["maxv"] / rust_sqrt(sq)
                vx *= scale
                vy *= scale
            b["vel"] = [vx, vy]
        self.update_manifolds(st, self.broadphase(st, dt))
        self.wake_on_contact(st)
        self.warm_start(st)
        cprep = self.prepare_contacts(st)
        prepared = []
        if dt > B2_EPSILON:
            for h, j in st["joint_arena"].values():
                pj = self.prepare_joint(st, h, j, dt)
                if pj is not None:
                    prepared.append(pj)
        for _ in range(cfg["iters"]):
            for pj in prepared:
                def read(ref):
                    b = bodies.get(ref[0]) if ref[0] is not None else None
                    return [0.0, 0.0, 0.0] if b is None else [b["vel"][0], b["vel"][1], b["w"]]
                va, vb = read(pj["a"]), read(pj["b"])
                for r in pj["rows"]:
                    self.solve_row(r, va, vb, pj["a"], pj["b"])
                if pj["point"] is not None:
                    self.solve_point(pj["point"], va, vb, pj["a"], pj["b"])
                for side, vel in ((pj["a"], va), (pj["b"], vb)):
                    if (side[1] > 0.0 or side[2] > 0.0) and side[0] is not None:
                        b = bodies.get(side[0])
                        if b is not None:
                            b["vel"], b["w"] = [vel[0], vel[1]], vel[2]
            self.solve_contacts_once(st, cprep)
        for pj in prepared:
            if pj["brk"] is None:
                continue
            if pj["point"] is not None:
                imp = self.hyp(pj["point"]["acc"][0], pj["point"]["acc"][1])
            elif pj["break_row"] is not None:
                imp = abs(pj["rows"][pj["break_row"]][9]) if pj["break_row"] < len(pj["rows"]) else 0.0
            else:
                imp = 0.0
            force = imp / dt if dt > B2_EPSILON else 0.0
            if force > pj["brk"]:
                st["joint_arena"].remove(pj["handle"])
        self.solve_positions(st, cfg, dt)
        for _, b in bodies.values():
            if b["type"] == B2_STATIC:
                continue
            if b["type"] == B2_DYNAMIC and (b["inv_mass"] == 0.0 or b["sleeping"]):
                b["pv"], b["pw"] = [0.0, 0.0], 0.0
                continue
            b["pos"] = [b["pos"][0] + b["vel"][0] * dt, b["pos"][1] + b["vel"][1] * dt]
            b["pos"] = [b["pos"][0] + b["pv"][0] * dt, b["pos"][1] + b["pv"][1] * dt]
            if not b["fixed"]:
                b["rot"] += b["w"] * dt
                b["rot"] += b["pw"] * dt
            b["pv"], b["pw"] = [0.0, 0.0], 0.0
        self.islands(st, dt)

    @staticmethod
    def islands(st, dt):
        bodies = st["body_arena"]
        hs = [b["handle"] for _, b in bodies.values()]
        n = len(hs)
        if n == 0:
            return
        idx = {h: i for i, h in enumerate(hs)}
        parent, rank = list(range(n)), [0] * n

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def dyn(h):
            b = bodies.get(h)
            return b is not None and b["type"] == "e:BodyType.Dynamic"

        ends = [(m["ba"], m["bb"]) for m in st["manifolds"]] + [(j["a"], j["b"]) for _, j in st["joint_arena"].values()]
        links = [(idx[a], idx[b]) for a, b in ends if dyn(a) and dyn(b) and a in idx and b in idx]
        for ia, ib in links:
            ra, rb = find(ia), find(ib)
            if ra == rb:
                continue
            if rank[ra] < rank[rb]:
                parent[ra] = rb
            elif rank[ra] > rank[rb]:
                parent[rb] = ra
            else:
                parent[rb] = ra
                rank[ra] += 1
        islands = {}
        for i in range(n):
            islands.setdefault(find(i), []).append(i)
        for root in sorted(islands):
            members = islands[root]
            all_slow, has_dyn = True, False
            for i in members:
                b = bodies.get(hs[i])
                if b["type"] != "e:BodyType.Dynamic" or b["inv_mass"] == 0.0:
                    continue
                has_dyn = True
                lin = rust_sqrt(b["vel"][0] * b["vel"][0] + b["vel"][1] * b["vel"][1])
                if lin >= B2_SLEEP_VELOCITY or abs(b["w"]) >= B2_SLEEP_VELOCITY:
                    all_slow = False
                    break
            if not has_dyn:
                continue
            min_t = math.inf
            for i in members:
                isl = find(i)
                b = bodies.get(hs[i])
                if b["type"] != "e:BodyType.Dynamic" or b["inv_mass"] == 0.0:
                    continue
                b["island"] = isl
                if all_slow:
                    b["sleep_timer"] += dt
                    min_t = rust_min(min_t, b["sleep_timer"])
                else:
                    b["sleep_timer"], b["sleeping"] = 0.0, False
            if all_slow and min_t >= B2_SLEEP_TIME:
                for i in members:
                    b = bodies.get(hs[i])
                    if b["type"] == "e:BodyType.Dynamic" and not b["sleeping"]:
                        b["sleeping"], b["vel"], b["w"] = True, [0.0, 0.0], 0.0

    def step(self, ins, module="b2_step"):
        """The output tokens of a b2_step or b2_contact row."""
        cfg = {"dt": tok_f(ins[0]), "g": (tok_f(ins[1]), tok_f(ins[2])), "iters": tok_i(ins[4]),
               "piters": tok_i(ins[5]), "slop": tok_f(ins[8]), "corr": tok_f(ins[9]), "maxv": tok_f(ins[10]),
               "sub": tok_i(ins[11]), "freq": tok_f(ins[12]), "ratio": tok_f(ins[13])}
        if ins[15] != "e:SolverKind.SequentialImpulse":
            fail(f"{module}: a step row runs {ins[15]} (the port refuses XPBD, ADR 0008 #6)")
        st, k = _dump_parse(ins, 16)
        if k != len(ins):
            fail(f"{module}: {len(ins) - k} input tokens left over")
        if module == "b2_step":
            if any(m != 0 for m in st["masks"]):
                fail("b2_step: an M3b scene has a collider that can pair (mask != 0)")
            if st["manifolds"] or st["tracker"]["prev"]:
                fail("b2_step: an M3b row's dump has manifolds or contacts")
        sub = max(cfg["sub"], 1)
        dt = cfg["dt"] / float(sub)
        for _ in range(sub):
            self.substep(st, cfg, dt)
            self.record(st)
        for _, b in st["body_arena"].values():
            b["force"], b["torque"] = [0.0, 0.0], 0.0
        ev = self.finish(st)
        out = _dump_emit(st) + [f"v:{len(ev)}"]
        for kind, (a, b) in ev:
            out += [f"e:CollisionEvent.{kind}", _st_h(a), _st_h(b)]
        return out


GLIBC_STEP = _Step2d(lambda x: _sin_cos(x)[0], lambda x: _sin_cos(x)[1], _glibc_hypot)


def check_b2_step(rows, module="b2_step"):
    """Recomputes every single-step row on glibc's sin, cos and hypot."""
    counted = 0
    for fn, cls, ins, outs, note in rows:
        if fn != "b2::step":
            fail(f"{module}: unknown function {fn}")
            continue
        want = GLIBC_STEP.step(ins, module)
        if not _same_tokens(outs, want):
            first = next((i for i, (a, b) in enumerate(zip(outs, want)) if not _same_tokens([a], [b])),
                         min(len(outs), len(want)))
            fail(f"{module}: row {counted}: the transcription differs at output {first} of {len(outs)}: "
                 f"{' '.join(outs[first:first + 3])} vs {' '.join(want[first:first + 3])}")
        counted += 1
    return counted


# The private constants t_private.rs extracts: file, Cyrius prefix, consts it doesn't carry.
PRIVATE_SOURCES = (
    ("aabb_tree.rs", "_IM_", {"NULL_NODE"}),
    ("spatial_hash.rs", "_IM_", set()),
    ("backend_2d/joints.rs", "_IM_B2_", set()),
    ("backend_2d/solver.rs", "_IM_B2_", set()),
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
                       ("_IM_B2_EPSILON_SQ", B2_EPSILON_SQ), ("_IM_B2_EPSILON", B2_EPSILON),
                       ("_IM_B2_SLEEP_VELOCITY_THRESHOLD", B2_SLEEP_VELOCITY),
                       ("_IM_B2_SLEEP_TIME_THRESHOLD", B2_SLEEP_TIME), ("_IM_B2_JOINT_BAUMGARTE", B2_JOINT_BAUMGARTE),
                       ("_IM_B2_MANIFOLD_MATCH_THRESHOLD", B2_MANIFOLD_MATCH),
                       ("_IM_B2_MAX_MANIFOLD_POINTS", B2_MAX_MANIFOLD_POINTS),
                       ("_IM_B2_WARM_START_FACTOR", B2_WARM_START),
                       ("_IM_B2_RESTITUTION_VELOCITY_THRESHOLD", B2_RESTITUTION_VELOCITY),
                       ("_IM_B2_MAX_CONDITION_NUMBER", B2_MAX_CONDITION)):
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
        if name in ("math", "spring", "b2_narrow", "b2_ray", "b2_step", "b2_contact"):
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
        elif name == "b2_step":
            print(f"b2_step: {check_b2_step(rows)} single steps recomputed (joints, integration, sleep)")
        elif name == "b2_contact":
            print(f"b2_contact: {check_b2_step(rows, name)} single steps recomputed (contacts, joints, sleep)")
        elif name == "b2_state":
            print(f"b2_state: {check_b2_state(rows)} scripts replayed on a model of the arenas and the state")
        elif name == "b2_ray":
            print(f"b2_ray: {check_b2_ray(rows)} rows recomputed (a transcription on glibc's sin, cos, hypot)")
        elif name == "b2_narrow":
            print(f"b2_narrow: {check_b2_narrow(rows)} rows recomputed (a transcription on Python's libm)")
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
