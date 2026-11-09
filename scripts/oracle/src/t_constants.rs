//! `src/constants.cyr` — impetus's public constants, as `IM_` consts.
//!
//! impetus 1.4.0 has three public scalar consts, all `u32` limits. `ALL` lists every one, and
//! `complete` proves the list against `rust-old/src`; `tests/golden/constants.tsv` pins every
//! value (`m_constants.rs`).
//! - **Config limits** (`Home::Constants`) are generated here: `WorldConfig::validate` checks
//!   them from M1.
//! - **The snapshot index cap** (`Home::Module("M6")`) is generated with the snapshot module;
//!   its golden row is *pending* in the harness until then.
//!
//! Private constants (solver thresholds, the broadphase margin, the emitter spawn cap, …) are
//! not public API. They are extracted from the Rust source and proven against the oracle's
//! output at the milestone whose module uses them, as tanmatra did (port plan §4 step 6).
//!
//! Values come from the oracle's API; the comments are the Rust doc comments.

use crate::cyr::{doc_comment, header};
use std::fmt::Write as _;
use std::path::Path;

/// Where a constant is generated.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Home {
    /// `src/constants.cyr` (this generator).
    Constants,
    /// With its module, at the named (later) milestone.
    Module(&'static str),
}

/// One public const of the oracle.
pub struct Constant {
    /// `config::WorldConfig::MAX_SUB_STEPS`, the golden row's input.
    pub path: &'static str,
    /// The Cyrius name.
    pub cyr: &'static str,
    /// The Rust file under `rust-old/src`, and the item line its doc comment sits on.
    file: &'static str,
    item: &'static str,
    pub value: i64,
    pub home: Home,
}

/// Every public const of impetus 1.4.0, in source order.
pub fn all() -> Vec<Constant> {
    vec![
        Constant {
            path: "config::WorldConfig::MAX_SUB_STEPS",
            cyr: "IM_MAX_SUB_STEPS",
            file: "config.rs",
            item: "pub const MAX_SUB_STEPS",
            value: i64::from(impetus::WorldConfig::MAX_SUB_STEPS),
            home: Home::Constants,
        },
        Constant {
            path: "config::WorldConfig::MAX_ITERATIONS",
            cyr: "IM_MAX_ITERATIONS",
            file: "config.rs",
            item: "pub const MAX_ITERATIONS",
            value: i64::from(impetus::WorldConfig::MAX_ITERATIONS),
            home: Home::Constants,
        },
        Constant {
            path: "world::PhysicsWorld::MAX_SNAPSHOT_INDEX",
            cyr: "IM_MAX_SNAPSHOT_INDEX",
            file: "world.rs",
            item: "pub const MAX_SNAPSHOT_INDEX",
            value: i64::from(impetus::PhysicsWorld::MAX_SNAPSHOT_INDEX),
            home: Home::Module("M6"),
        },
    ]
}

/// The non-test part of a Rust source file: everything before its `#[cfg(test)] mod tests`.
fn library_part(src: &str) -> &str {
    src.find("#[cfg(test)]\nmod tests")
        .map_or(src, |at| &src[..at])
}

/// Every `pub const NAME` item in `rust-old/src` outside test code must be in `all()`, and
/// every listed one must exist: a new public constant can't be missed.
fn complete(root: &Path) -> Result<(), String> {
    let listed: Vec<String> = all()
        .iter()
        .map(|c| c.item.trim_start_matches("pub const ").to_string())
        .collect();
    let mut found = Vec::new();
    let mut stack = vec![root.join("rust-old/src")];
    while let Some(dir) = stack.pop() {
        let entries = std::fs::read_dir(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            let name = path.file_name().map(|n| n.to_string_lossy().into_owned());
            if path.extension().is_none_or(|x| x != "rs") || name.as_deref() == Some("tests.rs") {
                continue;
            }
            let src =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            for line in library_part(&src).lines() {
                if let Some(rest) = line.trim_start().strip_prefix("pub const ")
                    && let Some((ident, _)) = rest.split_once(':')
                {
                    found.push(ident.trim().to_string());
                }
            }
        }
    }
    found.sort();
    let mut want = listed;
    want.sort();
    if found == want {
        Ok(())
    } else {
        Err(format!(
            "t_constants::all() lists {want:?}, but rust-old/src has public consts {found:?}"
        ))
    }
}

/// `src/constants.cyr`.
pub fn emit(root: &Path, provenance: &str) -> Result<String, String> {
    complete(root)?;
    let mut out = header(
        "public constants",
        provenance,
        &[
            "Every public constant of the Rust crate; t_constants.rs proves the list complete\n\
             against rust-old/src, and tests/golden/constants.tsv pins each value.",
            "The snapshot index cap (PhysicsWorld::MAX_SNAPSHOT_INDEX) is generated with the\n\
             snapshot module (M6). Private constants are extracted and proven at the milestone\n\
             whose module uses them.",
        ],
    );
    for c in all().iter().filter(|c| c.home == Home::Constants) {
        let file = root.join("rust-old/src").join(c.file);
        let src = std::fs::read_to_string(&file).map_err(|e| format!("{}: {e}", file.display()))?;
        out.push_str(&doc_comment(&src, c.item)?);
        let _ = writeln!(out, "# Rust: impetus::{}", c.path);
        let _ = writeln!(out, "const {} = {};", c.cyr, c.value);
        out.push('\n');
    }
    // One trailing newline, not a blank line, at the end of the file.
    while out.ends_with("\n\n") {
        out.pop();
    }
    Ok(out)
}
