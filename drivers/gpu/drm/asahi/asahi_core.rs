// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod agx_memory_stats;
mod alloc;
mod buffer;
mod channel;
mod cleanup;
mod debug;
mod driver;
mod event;
mod file;
mod float;
mod fw;
mod gem;
mod g17;
mod gpu;
mod hw;
mod initdata;
mod mem;
mod m3;
mod microseq;
mod mmu;
mod object;
mod pgtable;
mod queue;
mod regs;
mod slotalloc;
mod stats;
mod sysfs_exports;
mod util;
#[cfg(CONFIG_DRM_ASAHI_MAPLE_TREE)]
mod vm;
mod workqueue;

use kernel::prelude::*;

#[pin_data]
struct DriverModule {
    // Field drop order unregisters/joins driver producers before queue teardown.
    #[pin]
    _driver: kernel::driver::Registration<kernel::platform::Adapter<driver::AsahiPlatformDriver>>,
    _cleanup: cleanup::QueueOwner,
}

impl kernel::InPlaceModule for DriverModule {
    fn init(module: &'static kernel::ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            // Initialize the queue first, before registration can invoke probe.
            _cleanup: cleanup::QueueOwner::new()?,
            _driver <- kernel::driver::Registration::new(
                <Self as kernel::ModuleMetadata>::NAME,
                module,
            ),
        })
    }
}

module! {
    type: DriverModule,
    name: "asahi",
    description: "AGX GPU driver for Apple silicon SoCs",
    license: "Dual MIT/GPL",
    params: {
        debug_flags: u64 {
            default: 0,
            // permissions: 0o644,
            description: "Debug flags",
        },
        compute_preempt: i32 {
            default: -1,
            description: "Compute preemption (-1: M2 Max or Debug0; 0: off; nonzero: on)",
        },
        fault_control: u32 {
            default: 0xb,
            // permissions: 0,
            description: "Fault control (0x0: hard faults, 0xb: macOS default)",
        },
        initial_tvb_size: usize {
            default: 0x8,
            // permissions: 0o644,
            description: "Initial TVB size in blocks",
        },
        robust_isolation: u32 {
            default: 0,
            // permissions: 0o644,
            description: "Fully isolate GPU contexts (limits performance)",
        },
        stats_export: u8 {
            default: 1,
            // permissions: 0,
            description: "Export AGX firmware stats to sysfs (1 = on, 0 = off)",
        },
    },
}
