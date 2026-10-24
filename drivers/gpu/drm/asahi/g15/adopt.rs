// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Without the G15 backend (`CONFIG_DRM_ASAHI_G15=n`), the parts of it the T8122 runtime uses to
//! adopt the firmware's UAT: the handoff window ([`uat`]), its mappings ([`regs`]) and the
//! hardware constants they name ([`hw`]). Nothing here probes a device.

// Only the adoption is used; the rest of these files serves the G15 backend.
#![allow(dead_code)]

// hw.rs's `t6031` submodule is cfg-gated on G15; were it built here, a #[path]-loaded file
// would look for it at g15/t6031.rs, not g15/hw/t6031.rs.
#[path = "hw.rs"]
pub(crate) mod hw;
#[path = "regs.rs"]
pub(crate) mod regs;
#[path = "uat.rs"]
pub(crate) mod uat;

use kernel::prelude::*;

extern "C" {
    fn asahi_g15_memory_mode(start: u64, size: u64) -> u32;
}

/// WB for complete reserved RAM, WC for aligned non-RAM carveouts; reject everything else.
pub(crate) fn memory_mode(start: u64, size: u64) -> Result<u32> {
    // SAFETY: Only queries resources and page metadata; it does not create a mapping.
    match unsafe { asahi_g15_memory_mode(start, size) } {
        0 => Err(EINVAL),
        mode => Ok(mode),
    }
}
