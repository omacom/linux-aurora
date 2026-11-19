// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Live M5 Region C inputs. These offsets do not follow the Neo layout.

pub(crate) const EFFORT: usize = 0xe8;
pub(crate) const EFFORT_OVERRIDE: usize = 0xec;
pub(crate) const POWER_ASSERT_TALLY: usize = 0x1c99;

/// Control records which assert power among the opcodes this driver emits.
pub(crate) const fn asserts_gpu_power(opcode: u32) -> bool {
    matches!(opcode, 0x08 | 0x0a | 0x29)
}
