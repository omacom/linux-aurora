// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Compile and exercise the production allocation lifetime counters on the host.

extern crate self as kernel;

pub mod prelude {
    #[derive(Debug, PartialEq, Eq)]
    pub enum Error {
        Invalid,
        Overflow,
    }
    pub const EINVAL: Error = Error::Invalid;
    pub const EOVERFLOW: Error = Error::Overflow;
    pub type Result<T = ()> = std::result::Result<T, Error>;
}
pub mod fmt {
    pub use std::fmt::{Formatter, Result};
}
pub mod debugfs {
    pub trait Writer {
        fn write(&self, f: &mut crate::fmt::Formatter<'_>) -> crate::fmt::Result;
    }
}

#[allow(dead_code)]
#[path = "../../drivers/gpu/drm/asahi/agx_memory_stats.rs"]
mod memory;
