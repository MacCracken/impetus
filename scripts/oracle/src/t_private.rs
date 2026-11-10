//! `src/constants_private.cyr` — the private constants of the ported modules, extracted from
//! the Rust source (port plan §4 step 6, as tanmatra did).
//!
//! A private constant isn't reachable through the oracle's API, so its value comes from its
//! literal in `rust-old/src`, parsed with Rust's own `f64::from_str` (correctly rounded). It is
//! proven by the golden rows that depend on it (the broadphase's fattening and oversize cuts,
//! the mass floors, …). `complete` proves that every `const` item in each listed module is
//! emitted or excused, so a constant added to a ported module can't be missed.
//!
//! Names: `_IM_<NAME>` for the shared modules, `_IM_B2_<NAME>` for the 2D backend's and `_IM_B3_<NAME>`
//! for the 3D backend's (it repeats some names). f64 values are written positionally (the Cyrius lexer has no exponent)
//! as typed f64 globals; integers as `const`.

use crate::cyr::{doc_comment, header};
use std::fmt::Write as _;
use std::path::Path;

/// A ported module whose private constants are generated: its file under `rust-old/src`, the
/// Cyrius name prefix, and the consts not carried (name, reason).
struct Source {
    file: &'static str,
    prefix: &'static str,
    excused: &'static [(&'static str, &'static str)],
}

/// The ported modules, in port order.
fn sources() -> Vec<Source> {
    vec![
        Source {
            file: "aabb_tree.rs",
            prefix: "_IM_",
            excused: &[(
                "NULL_NODE",
                "the port's null node index is -1 (src/aabb_tree.cyr)",
            )],
        },
        Source {
            file: "spatial_hash.rs",
            prefix: "_IM_",
            excused: &[],
        },
        Source {
            file: "backend_2d/joints.rs",
            prefix: "_IM_B2_",
            excused: &[],
        },
        Source {
            file: "backend_2d/solver.rs",
            prefix: "_IM_B2_",
            excused: &[],
        },
        Source {
            file: "backend_2d/types.rs",
            prefix: "_IM_B2_",
            excused: &[(
                "FALLBACK",
                "the (1, [0, 0], 1) tuple local to `shape_props`, written in place in \
                 src/b2_types.cyr",
            )],
        },
        Source {
            file: "backend_3d/narrowphase.rs",
            prefix: "_IM_B3_",
            excused: &[],
        },
        Source {
            file: "backend_3d/types.rs",
            prefix: "_IM_B3_",
            excused: &[],
        },
        Source {
            file: "world.rs",
            prefix: "_IM_",
            excused: &[(
                "MAX_SNAPSHOT_INDEX",
                "the snapshot code's public cap on restored handle indices, carried with M6",
            )],
        },
    ]
}

/// One `const NAME: TYPE = LITERAL;` item.
struct Item {
    name: String,
    ty: String,
    literal: String,
    /// The item's line prefix, for `doc_comment`.
    head: String,
}

/// The non-test part of a Rust source file.
fn library_part(src: &str) -> &str {
    let mut end = src.len();
    for marker in ["#[cfg(test)]\nmod tests", "#[cfg(test)]\nmod mass_tests"] {
        if let Some(at) = src.find(marker) {
            end = end.min(at);
        }
    }
    &src[..end]
}

/// Every `const` item (any visibility) in `src`'s library part. An array literal may span lines up
/// to its `];`; its elements are joined into one `[a, b, …]` literal.
fn items(src: &str) -> Result<Vec<Item>, String> {
    let mut out = Vec::new();
    let lines: Vec<&str> = library_part(src).lines().collect();
    let mut li = 0;
    while li < lines.len() {
        let line = lines[li];
        li += 1;
        let t = line.trim_start();
        let rest = [
            "pub(super) const ",
            "pub(crate) const ",
            "pub const ",
            "const ",
        ]
        .iter()
        .find_map(|p| t.strip_prefix(p).map(|r| (*p, r)));
        let Some((vis, rest)) = rest else { continue };
        let (name, after) = rest
            .split_once(':')
            .ok_or_else(|| format!("unparsed const line: {line}"))?;
        let (ty, lit) = after
            .split_once('=')
            .ok_or_else(|| format!("unparsed const line: {line}"))?;
        let mut literal = lit.trim().trim_end_matches(';').trim().to_string();
        if literal == "[" {
            let mut parts = Vec::new();
            while li < lines.len() && !lines[li].trim_start().starts_with("];") {
                parts.push(lines[li].trim().trim_end_matches(',').to_string());
                li += 1;
            }
            li += 1;
            literal = format!("[{}]", parts.join(", "));
        }
        out.push(Item {
            name: name.trim().to_string(),
            ty: ty.trim().to_string(),
            literal,
            head: format!("{vis}{}", name.trim()),
        });
    }
    Ok(out)
}

/// The Cyrius literal for a Rust f64 literal: the value Rust parses, written positionally with a
/// decimal point (Rust's `Display` never uses an exponent).
fn f64_literal(lit: &str) -> Result<String, String> {
    let clean: String = lit.chars().filter(|c| *c != '_').collect();
    let v: f64 = clean
        .parse()
        .map_err(|e| format!("f64 literal `{lit}`: {e}"))?;
    let mut s = format!("{v}");
    if !s.contains('.') {
        s.push_str(".0");
    }
    Ok(s)
}

/// `src/constants_private.cyr`.
pub fn emit(root: &Path, provenance: &str) -> Result<String, String> {
    let mut out = header(
        "private constants of the ported modules",
        provenance,
        &[
            "Extracted from rust-old/src by t_private.rs: each literal parsed as Rust parses it and\n\
             written positionally. Proven by the golden rows that depend on them.",
        ],
    );
    for s in sources() {
        let path = root.join("rust-old/src").join(s.file);
        let src = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let found = items(&src)?;
        for (name, _) in s.excused {
            if !found.iter().any(|i| i.name == *name) {
                return Err(format!(
                    "t_private: {} excuses {name}, which isn't in it",
                    s.file
                ));
            }
        }
        let _ = writeln!(out, "# ── rust-old/src/{} ──", s.file);
        out.push('\n');
        for it in &found {
            if let Some((_, why)) = s.excused.iter().find(|(n, _)| *n == it.name) {
                let _ = writeln!(out, "# {} is not carried: {why}.", it.name);
                out.push('\n');
                continue;
            }
            out.push_str(&doc_comment(&src, &it.head)?);
            let _ = writeln!(out, "# Rust: `{}: {} = {}`", it.name, it.ty, it.literal);
            let cyr = format!("{}{}", s.prefix, it.name);
            match it.ty.as_str() {
                "f64" => {
                    let _ = writeln!(out, "var {cyr}: f64 = {};", f64_literal(&it.literal)?);
                }
                ty if ty.starts_with("[f64; ") => {
                    // `[f64; N]`: one generated global per element, `_0` … `_{N-1}`.
                    let inner = it.literal.trim_start_matches('[').trim_end_matches(']');
                    let want: usize = ty["[f64; ".len()..ty.len() - 1]
                        .trim()
                        .parse()
                        .map_err(|e| format!("{}: array type `{ty}`: {e}", s.file))?;
                    let elems: Vec<&str> = inner.split(',').map(str::trim).collect();
                    if elems.len() != want {
                        return Err(format!(
                            "{}: {} has {} elements, its type says {want}",
                            s.file,
                            it.name,
                            elems.len()
                        ));
                    }
                    for (k, e) in elems.iter().enumerate() {
                        let _ = writeln!(out, "var {cyr}_{k}: f64 = {};", f64_literal(e)?);
                    }
                }
                "usize" | "u32" | "i32" | "u64" | "i64" => {
                    let v: i64 = it.literal.replace('_', "").parse().map_err(|e| {
                        format!("{}: integer literal `{}`: {e}", s.file, it.literal)
                    })?;
                    let _ = writeln!(out, "const {cyr} = {v};");
                }
                other => {
                    return Err(format!("t_private: {} has a const of type {other}", s.file));
                }
            }
            out.push('\n');
        }
    }
    while out.ends_with("\n\n") {
        out.pop();
    }
    Ok(out)
}
