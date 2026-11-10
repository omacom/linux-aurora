// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Measured J516M/T6034 30-core profile. Share the six-channel G15C layout
//! with J514M, but retain independent machine, firmware and fused-core admission.
//! Native rendering is verified; inherited J516C power/reference values remain
//! unqualified for sustained power and thermal behavior on this board.

use super::super::Config;
use super::FirmwareBuild;

pub(crate) const CONFIG: Config = Config {
    machine: kernel::c_str!("apple,j516m"),
    firmware: &[FirmwareBuild {
        name: "J516M RTKit 2419.140.12 (power qualification pending)",
        machine: Some(kernel::c_str!("apple,j516m")),
        text_size: 0x64000,
        data_size: 0x140000,
        marker: 0x5cf28,
        text_sha256: [
            0x4c, 0xa2, 0x4b, 0x65, 0x8f, 0x8e, 0xbe, 0x7c, 0x27, 0x37, 0x5f, 0xdf, 0x38, 0xe4,
            0x1e, 0x5d, 0xaf, 0x86, 0x32, 0x51, 0x50, 0x77, 0xb5, 0x37, 0xca, 0x19, 0xe6, 0xd0,
            0xdc, 0x77, 0x04, 0xb7,
        ],
    }],
    ..super::t6034::CONFIG
};

/// The 14-core-CPU, 30-core-GPU J516M: its G15C revision and counts, and exactly 30 enabled
/// cores. Which cores are fused off differs between Macs of the same bin, so the mask itself is
/// not pinned (the per-cluster plausibility check in `start()` still applies).
pub(crate) const ENABLED_CORES: u32 = 30;

pub(crate) fn identity_matches(version: u32, counts: u32, cores: u64) -> bool {
    version == 0x07041100 && counts == 0x0021040a && cores.count_ones() == ENABLED_CORES
}
