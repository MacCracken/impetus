//! `oracle` — writes everything the Cyrius port derives from the frozen Rust impetus 1.4.0 in
//! `rust-old/` (with its hidden `oracle` hooks):
//! - the golden corpus, `tests/golden/<module>.tsv` (port plan §4 step 5);
//! - the generated Cyrius sources, `src/constants.cyr` and later data modules (§4 step 6).
//!
//! ```sh
//! cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml             # 2D + shared
//! cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check  # verify
//! cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml \
//!     --no-default-features --features b3 [-- --check]                               # 3D
//! ```
//!
//! The oracle's `2d` and `3d` backends are exclusive, so the generator is built twice. Each
//! build **owns** its files: the 3D build every `b3_*.tsv`, the 2D build everything else
//! (the shared modules, `b2_*.tsv` and the generated `.cyr` files). `tests/golden/modules.txt`
//! lists the union of both builds' modules, so either build keeps it current.
//!
//! `--check` regenerates the build's outputs in memory and compares them byte for byte with the
//! committed files. It exits 1 if any output differs or is missing, if an owned `.tsv` has no
//! emitter, or if an owned file was recorded on another glibc platform (`glibc.rs`): its
//! transcendental rows are glibc's bits, so they can only be checked where they were made. A
//! `<module>.cr.tsv` beside a module's file is the corrected oracle (ADR 0011), which
//! `validate.py --write` writes and checks: here it is stale only when its module has no emitter,
//! and its platform is checked like the corpus's.
//!
//! Adapted from kimiya's and tanmatra's `scripts/oracle/` (same format, same rules). Modules are
//! added at the start of their milestone, with the golden grid from the port plan §6:
//! `constants` and `math` (M0); `config`, `material`, `units` and `force` (M1a); `fmtf` and `json` (M1b, ADR 0010); …

mod cyr;
mod flat;
mod glibc;
mod grids;
#[cfg(not(feature = "b3"))]
mod m_b2_mass;
mod m_broadphase;
mod m_config;
mod m_constants;
mod m_fmtf;
mod m_force;
mod m_json;
mod m_material;
mod m_math;
mod m_spring;
mod m_types;
mod m_units;
mod t_constants;
mod t_private;
mod tsv;

#[cfg(all(feature = "b2", feature = "b3"))]
compile_error!("build the oracle with exactly one backend: `b2` (default) or `b3`");

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

/// The backend this build drives.
const BUILD: &str = if cfg!(feature = "b3") { "3d" } else { "2d" };

/// Whether this build owns `tests/golden/<name>`.
fn owns(name: &str) -> bool {
    name.starts_with("b3_") == cfg!(feature = "b3")
}

/// Every golden emitter of this build, in file-name order.
fn golden_modules() -> Vec<tsv::Module> {
    if cfg!(feature = "b3") {
        vec![]
    } else {
        vec![
            // The 2D backend's bodies: the 3D dump's inertia is a tensor.
            #[cfg(not(feature = "b3"))]
            m_b2_mass::emit(),
            m_broadphase::emit(),
            m_config::emit(),
            m_constants::emit(),
            m_fmtf::emit(),
            m_force::emit(),
            m_json::emit(),
            m_material::emit(),
            m_math::emit(),
            m_spring::emit(),
            m_types::emit(),
            m_units::emit(),
        ]
    }
}

/// One generated file: its path from the repo root, its contents, and a summary.
struct Output {
    rel: String,
    text: String,
    summary: String,
}

fn read(path: &Path) -> Result<String, String> {
    fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

/// Every generated Cyrius source this build owns, in file-name order.
fn cyrius_outputs(root: &Path, prov: &str) -> Result<Vec<Output>, String> {
    let files = if cfg!(feature = "b3") {
        vec![]
    } else {
        vec![
            ("src/constants.cyr", t_constants::emit(root, prov)?),
            ("src/constants_private.cyr", t_private::emit(root, prov)?),
        ]
    };
    Ok(files
        .into_iter()
        .map(|(rel, text)| {
            let lines = text.lines().count();
            Output {
                rel: rel.to_string(),
                text,
                summary: format!("{lines} lines"),
            }
        })
        .collect())
}

/// `version = "x"` from the first line that starts with it (the `[package]` one).
fn toml_version(text: &str) -> Option<&str> {
    let line = text.lines().find(|l| l.starts_with("version = \""))?;
    line.strip_prefix("version = \"")?.strip_suffix('"')
}

/// The locked version of `crate_name` in a Cargo.lock (its first entry).
fn lock_version<'a>(lock: &'a str, crate_name: &str) -> Option<&'a str> {
    let needle = format!("name = \"{crate_name}\"\n");
    let at = lock.find(&needle)? + needle.len();
    let line = lock[at..].lines().next()?;
    line.strip_prefix("version = \"")?.strip_suffix('"')
}

/// `impetus 1.4.0 (rust-old/), hisab 1.4.0, glam 0.29.3`: the oracle and its math.
fn oracle_provenance(root: &Path) -> Result<String, String> {
    let cargo = read(&root.join("rust-old/Cargo.toml"))?;
    let lock = read(&root.join("scripts/oracle/Cargo.lock"))?;
    let im = toml_version(&cargo).ok_or("rust-old/Cargo.toml: no version")?;
    let hs = lock_version(&lock, "hisab").ok_or("scripts/oracle/Cargo.lock: no hisab")?;
    // hisab 1.4.0 re-exports glam's D-types. The lock holds more than one glam, so read the
    // one listed in hisab's own dependencies (`"glam 0.29.3"`).
    let hisab_entry = &lock[lock
        .find("name = \"hisab\"\n")
        .ok_or("scripts/oracle/Cargo.lock: no hisab")?..];
    let deps = hisab_entry
        .split("\n\n")
        .next()
        .ok_or("scripts/oracle/Cargo.lock: empty hisab entry")?;
    let glam = deps
        .lines()
        .find_map(|l| l.trim().strip_prefix("\"glam "))
        .and_then(|rest| rest.strip_suffix("\","))
        .ok_or("scripts/oracle/Cargo.lock: hisab lists no versioned glam")?;
    Ok(format!("impetus {im} (rust-old/), hisab {hs}, glam {glam}"))
}

/// Write `text` to `root/rel`, or in check mode compare it. Returns false on a mismatch.
fn emit(root: &Path, check: bool, out: &Output) -> Result<bool, String> {
    let path: PathBuf = root.join(&out.rel);
    if !check {
        fs::write(&path, &out.text).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("wrote    {} ({})", out.rel, out.summary);
        return Ok(true);
    }
    match fs::read_to_string(&path) {
        Ok(old) if old == out.text => {
            println!("ok       {} ({})", out.rel, out.summary);
            Ok(true)
        }
        Ok(_) => {
            println!("DIFFERS  {}", out.rel);
            Ok(false)
        }
        Err(_) => {
            println!("MISSING  {}", out.rel);
            Ok(false)
        }
    }
}

/// The libm platform in a TSV's `# oracle:` header line: the text from `libm: ` to the next
/// `. `, e.g. `glibc 2.44 x86_64 fma`.
fn recorded_platform(text: &str) -> Option<String> {
    let line = text
        .lines()
        .take(5)
        .find_map(|l| l.strip_prefix("# oracle: "))?;
    let (_, rest) = line.split_once("libm: ")?;
    Some(rest.split_once(". ").map_or(rest, |(p, _)| p).to_string())
}

/// The `.tsv` names in `dir`, sorted.
fn tsv_names(dir: &Path) -> Vec<String> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut names: Vec<String> = entries
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".tsv"))
        .collect();
    names.sort();
    names
}

/// In check mode: every owned `.tsv` must have been recorded on the running platform. Its
/// `# oracle:` header line names the glibc, so a corpus made elsewhere is refused with a reason
/// rather than as an anonymous DIFFERS.
fn check_platform(dir: &Path, platform: &str) -> bool {
    let mut ok = true;
    for name in tsv_names(dir).into_iter().filter(|n| owns(n)) {
        let Ok(text) = fs::read_to_string(dir.join(&name)) else {
            continue;
        };
        match recorded_platform(&text) {
            Some(p) if p == platform => {}
            Some(p) => {
                println!(
                    "PLATFORM tests/golden/{name}: recorded on {p}, running {platform}. Its \
                     transcendental rows are glibc's bits: check it on the recorded platform, \
                     or regenerate deliberately"
                );
                ok = false;
            }
            None => {
                println!("PLATFORM tests/golden/{name}: no recorded libm platform in its header");
                ok = false;
            }
        }
    }
    ok
}

fn run() -> Result<bool, String> {
    let check = std::env::args().skip(1).any(|a| a == "--check");
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let root = root
        .canonicalize()
        .map_err(|e| format!("{}: {e}", root.display()))?;
    let oracle = oracle_provenance(&root)?;
    let platform = glibc::platform();
    // Every TSV row may hold glibc's bits; the generated .cyr files hold only the oracle's
    // constants, so they name the oracle alone and don't change with the platform.
    let tsv_prov = format!("{oracle}, {BUILD} backend; libm: {platform}");
    let dir = root.join("tests/golden");
    if !check {
        fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }

    let mut ok = true;
    if check {
        ok &= check_platform(&dir, &platform);
    }

    let mut outputs = Vec::new();
    let mut emitted = BTreeSet::new();
    for module in golden_modules() {
        let name = format!("{}.tsv", module.name);
        debug_assert!(owns(&name), "{name} is not this build's");
        emitted.insert(name.clone());
        outputs.push(Output {
            rel: format!("tests/golden/{name}"),
            text: module.render(&tsv_prov),
            summary: format!("{} rows", module.len()),
        });
    }
    // The harnesses read this list, so a module added here can't go untested: without a
    // dispatcher it is reported as pending, row by row. It is the union of both builds'
    // modules: this build's emitters, plus the other build's files already present (never a
    // `<module>.cr.tsv`, which is its module's corrected oracle, not a module).
    let mut all = emitted.clone();
    all.extend(
        tsv_names(&dir)
            .into_iter()
            .filter(|n| !owns(n) && !n.ends_with(".cr.tsv")),
    );
    let mut index = String::from(
        "# impetus golden corpus: one module per line; tests/parity_2d.tcyr replays the shared and\n\
         # b2_ modules, tests/parity_3d.tcyr the b3_ modules\n",
    );
    for name in &all {
        index.push_str(name.trim_end_matches(".tsv"));
        index.push('\n');
    }
    outputs.push(Output {
        rel: "tests/golden/modules.txt".to_string(),
        summary: format!("{} modules", all.len()),
        text: index,
    });
    outputs.extend(cyrius_outputs(&root, &oracle)?);

    for out in &outputs {
        ok &= emit(&root, check, out)?;
    }

    // An owned .tsv with no emitter is stale: the harness would test rows nobody regenerates. A
    // `<module>.cr.tsv` (the corrected oracle, ADR 0011, written by validate.py) belongs to its
    // module's emitter.
    for name in tsv_names(&dir) {
        let module_file = name
            .strip_suffix(".cr.tsv")
            .map_or_else(|| name.clone(), |m| format!("{m}.tsv"));
        if owns(&name) && !emitted.contains(&module_file) {
            println!("STALE    tests/golden/{name} (no emitter)");
            ok = false;
        }
    }
    Ok(ok)
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("oracle: {e}");
            ExitCode::from(2)
        }
    }
}
