// SPDX-License-Identifier: GPL-2.0-only OR MIT

use crate::g17::fw::initdata::RegisterWindow;

const MMIO_APERTURE: u64 = 0xffff_fc21_8000_0000;

pub(super) const fn window(
    slot: usize,
    phys: u64,
    offset: u64,
    size: u32,
    unk_18: u64,
    unk_20: u32,
) -> RegisterWindow {
    RegisterWindow {
        slot,
        phys,
        va: MMIO_APERTURE + offset,
        size,
        unk_18,
        unk_20,
        instances: 1,
        phys_stride: 0,
    }
}

pub(super) const fn instances(window: RegisterWindow, instances: u32, phys_stride: u64) -> RegisterWindow {
    RegisterWindow {
        instances,
        phys_stride,
        ..window
    }
}
