//! `types.tsv` — the descriptor types' Rust-side constructors (M1b): `BodyDesc::default`.

use impetus::BodyDesc;

use crate::flat;
use crate::tsv::{Class, Module};

pub fn emit() -> Module {
    let mut m = Module::new("types");
    m.row(
        "BodyDesc::default",
        Class::A,
        &[],
        &flat::body_desc(&BodyDesc::default()),
    );
    m
}
