//! `constants.tsv` — every public const of impetus 1.4.0, class A.
//!
//! One row per constant, `const A s:<path> i:<value>`. The list is shared with the
//! `src/constants.cyr` generator (`t_constants::all`), so the two can't drift apart. Rows for a
//! constant generated at a later milestone are pending until then.

use crate::t_constants;
use crate::tsv::{Class, Module, i, s};

pub fn emit() -> Module {
    let mut m = Module::new("constants");
    for c in t_constants::all() {
        m.row("const", Class::A, &[s(c.path)], &[i(c.value)]);
    }
    m
}
