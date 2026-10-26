// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

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
#[cfg(CONFIG_DRM_ASAHI_G15)]
mod g15;
// Without the G15 backend, only the UAT adoption the T8122 runtime shares with it.
#[cfg(all(CONFIG_DRM_ASAHI_UAT_ADOPTION, not(CONFIG_DRM_ASAHI_G15)))]
#[path = "g15/adopt.rs"]
mod g15;
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

#[cfg(CONFIG_DRM_ASAHI_G15)]
type G15Registration =
    kernel::driver::Registration<kernel::platform::Adapter<g15::Driver<'static>>>;

#[pin_data]
struct DriverModule {
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    #[pin]
    _g15: G15Registration,
    // Field drop order unregisters/joins driver producers before queue teardown.
    #[pin]
    _driver: kernel::driver::Registration<kernel::platform::Adapter<driver::AsahiPlatformDriver>>,
    _cleanup: cleanup::QueueOwner,
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    _g15_ready: (),
}

impl kernel::InPlaceModule for DriverModule {
    fn init(module: &'static kernel::ThisModule) -> impl PinInit<Self, Error> {
        try_pin_init!(Self {
            // Initialize the queue first, before registration can invoke probe.
            _cleanup: cleanup::QueueOwner::new()?,
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            _g15 <- kernel::driver::Registration::new(c"asahi-g15", module),
            _driver <- kernel::driver::Registration::new(
                <Self as kernel::ModuleMetadata>::NAME,
                module,
            ),
            // All fallible fields are initialized. This final unit initializer cannot fail.
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            _g15_ready: g15::activate(),
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
        g15_deadline_ms: u32 {
            default: 5000,
            description: "G15: host deadline of a published prefix without progress, ms (min 100)",
        },
        g15_compute_deadline_ms: u32 {
            default: 1000,
            description: "G15: no-progress budget of a started compute before the class-2 kill, ms",
        },
        g15_repoll_us: u32 {
            default: 15,
            description: "G15: repoll interval while a ready prefix waits for record consumption, us (5..100)",
        },
        g15_resnap_us: u32 {
            default: 100,
            description: "G15: completion repoll interval, us (25..1000)",
        },
        g15_max_inflight: u32 {
            default: 0,
            description: "G15: published prefixes in flight per VM at most (1..8; 0: the tuned value)",
        },
        g15_idle_standby_us: u32 {
            default: 0,
            description: "G15: firmware idle standby time override, us (Globals +0x7a8, 100..100000; 0: the device tree's, else the tuned value)",
        },
        g15_idle_min_conf: u32 {
            default: 500,
            description: "G15: firmware idle minimum confidence, permille (Globals +0x7c4)",
        },
        g15_recovery: u32 {
            default: 1,
            description: "G15: answer firmware halts with the halt/resume handshake (0: fail-stop)",
        },
        g15_stream_gate: u32 {
            default: 1,
            description: "G15: admit only known CDM/VDM stream records (0: bring-up only)",
        },
        g15_pb_max_blocks: u32 {
            default: 3072,
            description: "G15: PB growth maximum given to the firmware, 128 KiB blocks (64..4095)",
        },
        g15_pb_blocks: u32 {
            default: 64,
            description: "G15: PB blocks posted at (re)initialization, 128 KiB each (64..4095)",
        },
        g15_pb_reclaim: u32 {
            default: 2,
            description: "G15: PB reclaim when idle: 0 never, 1 re-initialize, 2 also free backing",
        },
        g15_pb_idle_ms: u32 {
            default: 1000,
            description: "G15: idle time before a PB reclaim, ms (at least 100)",
        },
        g15_pb_reclaim_blocks: u32 {
            default: 1024,
            description: "G15: PB blocks posted above which an idle engine reclaims the PB",
        },
        g15_early_tiling: u32 {
            default: 0,
            description: "G15: advertise FRAGMENT_BARRIERS, mapped to early tiling (slower on 2419)",
        },
        g15_accept_text_prefix: u64 {
            default: 0,
            description: "G15: admit an unpinned GPU firmware build whose TEXT SHA-256 starts with these 8 bytes (bring-up only)",
        },
        stats_export: u8 {
            default: 1,
            // permissions: 0,
            description: "Export AGX firmware stats to sysfs (1 = on, 0 = off)",
        },
    },
}
