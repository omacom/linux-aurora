// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! J514M/T6034 (M3 Max, G15C) measurement profile.
//!
//! The powered identity read on this J514M found four ten-slot clusters with
//! enabled masks [0x1fd, 0x1fd, 0x1fc, 0x1ed], or 30 enabled cores. Its ADT has
//! four voltage tables and six memory channels. The four-cluster G15C object
//! and command layouts below are inherited from T6031; they are not a native
//! macOS capture of this machine. The J516C power/reference words remain
//! unqualified for this board and are identified at admission.
//!
//! No loaded firmware digest has been measured by the kernel yet. An empty
//! build list permits the normal admission path to report that identity and
//! refuses firmware startup, including the generic prefix escape hatch.

use super::super::Config;
use super::{HwId, IoMap, Layout};

/// Six memory-cache elements, rather than the T6031 reference's eight.
const MCC_BYTES: u64 = 6 * 0xd_8000;
const IOMAP_GAP: u64 = 0x4000;

/// Keep the shared G15C physical apertures and pack firmware VAs after the
/// smaller memory-cache aperture. No seventh or eighth element is mapped.
const fn iomaps() -> [IoMap; 17] {
    let mut out = super::t6031::IOMAPS;
    let mut va = out[0].va;
    let mut k = 0;
    while k < out.len() {
        if out[k].slot == 11 {
            out[k].map_bytes = MCC_BYTES;
        }
        out[k].va = va;
        va += out[k].map_bytes + IOMAP_GAP;
        k += 1;
    }
    out
}

pub(crate) const IOMAPS: [IoMap; 17] = iomaps();

const fn record_slots() -> [(u32, u32, bool); 17] {
    let mut out = [(0, 0, false); 17];
    let mut k = 0;
    while k < out.len() {
        out[k] = super::t6031::LAYOUT.record_slots[k];
        if IOMAPS[k].slot == 11 {
            out[k].0 = MCC_BYTES as u32;
        }
        k += 1;
    }
    out
}

const RECORD_SLOTS: [(u32, u32, bool); 17] = record_slots();

pub(crate) const LAYOUT: Layout = Layout {
    iomaps: &IOMAPS,
    record_slots: &RECORD_SLOTS,
    ..super::t6031::LAYOUT
};

pub(crate) const CONFIG: Config = Config {
    name: "T6034 G15C",
    chip_id: 0x6034,
    gpu_variant: crate::hw::GpuVariant::C,
    // ID_VERSION's silicon revision byte: (0x07041100 >> 8) & 0xff.
    gpu_revision: 0x11,
    uat_ias: 42,
    uat_oas: 42,
    num_clusters: 4,
    num_cores_per_cluster: 10,
    hw_ids: &[HwId {
        version: 0x07041100,
        counts: 0x0021040a,
    }],
    firmware: &[],
    machine: kernel::c_str!("apple,j514m"),
    platform: super::Platform::DeviceTree,
    layout: &LAYOUT,
    board: super::Board::DeviceTree(&super::super::board::record::J516C),
    // Binning changes enabled masks, not the four physical clusters or the
    // ten slots per cluster used by these G15C command words.
    compute: super::t6031::CONFIG.compute,
};
