// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! M5 firmware-specific work encodings, using the shared G17 queue primitives.

pub(crate) mod initdata;
pub(crate) mod power;

/// M5 adds a statistics record at tag 32, beyond the legacy 32-bit mask.
pub(crate) const STATISTICS_TYPES: u64 = 0x1_5fff_0000;

/// M5 stores an entry count in the trailer, with its upper half clear.
/// The preceding register storage and self pointer use the shared layout.
pub(crate) fn set_register_array(
    array: &mut super::kick::RegisterArray,
    address: u64,
    writes: &[super::kick::RegisterWrite],
) -> kernel::prelude::Result {
    array.set(address, writes)?;
    array.size = writes.len() as u32;
    Ok(())
}

pub(crate) mod pool;
pub(crate) mod pool_budget;
pub(crate) mod queue;
pub(crate) mod compute;

pub(crate) mod render_demand;

pub(crate) mod render;
pub(crate) mod render_tail;

pub(crate) mod render_registers;

pub(crate) mod render_assembly;

pub(crate) mod target;
pub(crate) mod topology;
