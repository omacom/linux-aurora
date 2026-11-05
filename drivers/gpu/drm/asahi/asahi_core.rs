// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod alloc;
mod binding;
mod buffer;
mod channel;
mod cleanup;
mod debug;
mod driver;
mod event;
mod file;
mod float;
mod fw;
mod fwid;
#[cfg(CONFIG_DRM_ASAHI_G15)]
mod g15;
// Without the G15 backend, only the UAT adoption the T8122 runtime shares with it.
#[cfg(all(CONFIG_DRM_ASAHI_UAT_ADOPTION, not(CONFIG_DRM_ASAHI_G15)))]
#[path = "g15/adopt.rs"]
mod g15;
mod g17;
mod gem;

#[cfg(CONFIG_DRM_ASAHI_G16)]
mod g16;
#[cfg(CONFIG_DRM_ASAHI_G18)]
mod g18;
#[cfg(CONFIG_DRM_ASAHI_M5)]
mod m5;
mod gpu;
mod hw;
mod initdata;
mod m3;
mod legacy_rx;
mod mem;
mod memacct;
mod microseq;
mod mmu;
mod object;
mod pgtable;
mod queue;
mod queue_share;
mod regs;
mod retain;
mod slotalloc;
mod stats;
mod stats_busy;
mod stats_raw;
mod sysfs_exports;
mod timestamp;
mod usage;
mod util;
#[cfg(CONFIG_DRM_ASAHI_MAPLE_TREE)]
mod vm;
mod workqueue;

use kernel::prelude::*;

#[cfg(CONFIG_DRM_ASAHI_G15)]
type G15Registration =
    kernel::driver::Registration<kernel::platform::Adapter<g15::Driver<'static>>>;

#[cfg(CONFIG_DRM_ASAHI_G16)]
type G16Registration = kernel::driver::Registration<kernel::platform::Adapter<g16::Driver>>;
#[cfg(not(CONFIG_DRM_ASAHI_G16))]
type G16Registration = ();

fn register_g16(module: &'static kernel::ThisModule) -> Result<Option<Pin<KBox<G16Registration>>>> {
    #[cfg(CONFIG_DRM_ASAHI_G16)]
    {
        Ok(Some(KBox::pin_init(
            kernel::driver::Registration::new(g16::DRIVER_NAME, module),
            GFP_KERNEL,
        )?))
    }
    #[cfg(not(CONFIG_DRM_ASAHI_G16))]
    {
        let _ = module;
        Ok(None)
    }
}

#[pin_data]
struct DriverModule {
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    #[pin]
    _g15: G15Registration,
    // Field drop order unregisters/joins driver producers before queue teardown.
    _g16: Option<Pin<KBox<G16Registration>>>,
    #[cfg(CONFIG_DRM_ASAHI_G18)]
    #[pin]
    _g18: kernel::driver::Registration<kernel::platform::Adapter<g18::probe::Driver>>,
    #[pin]
    _driver: kernel::driver::Registration<kernel::platform::Adapter<driver::AsahiPlatformDriver>>,
    _legacy_rx: legacy_rx::QueueOwner,
    _cleanup: cleanup::QueueOwner,
    #[cfg(CONFIG_DRM_ASAHI_G15)]
    _g15_ready: (),
}

impl kernel::InPlaceModule for DriverModule {
    fn init(module: &'static kernel::ThisModule) -> impl PinInit<Self, Error> {
        // SAFETY: Module init runs once, before either adapter is registered below.
        unsafe { binding::init() };
        // SAFETY: Module init runs once, before either adapter is registered below.
        unsafe { queue_share::init() };
        // SAFETY: Module init runs once, before either adapter is registered below.
        unsafe { g17::exec_share::init() };
        try_pin_init!(Self {
            // Initialize the queue first, before registration can invoke probe.
            _cleanup: cleanup::QueueOwner::new()?,
            _legacy_rx: legacy_rx::QueueOwner::new()?,
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            _g15 <- kernel::driver::Registration::new(c"aurora-g15", module),
            _driver <- kernel::driver::Registration::new(
                <Self as kernel::ModuleMetadata>::NAME,
                module,
            ),
            _g16: register_g16(module)?,
            #[cfg(CONFIG_DRM_ASAHI_G18)]
            _g18 <- kernel::driver::Registration::new(g18::probe::DRIVER_NAME, module),
            // All fallible fields are initialized. This final unit initializer cannot fail.
            #[cfg(CONFIG_DRM_ASAHI_G15)]
            _g15_ready: g15::activate(),
        })
    }
}

module! {
    type: DriverModule,
    name: "aurora",
    description: "AGX GPU driver for Apple silicon SoCs",
    license: "Dual MIT/GPL",
    alias: ["asahi"],
    params: {
        m5_mesh_storage_chunks: u32 {
            default: 0x200,
            description: "M5 mesh rendering buffer bound to API task (object-stage) render kicks only, in 64 KiB chunks; 0 disables the binding and the task feature",
        },
        m5_thermal_governor: u32 {
            default: 0,
            description: "M5 thermal flags: bit 0 enables the host GPU power budget controller, bit 2 lets the controller lower the CPU frequency ceiling while the GPU is budget-bound, bit 3 runs the fans at their maximum while it governs; default 0 preserves the existing policy",
        },
        m5_thermal_target_mc: u32 {
            default: 90000,
            description: "M5 GPU power budget governor target for the SoC hotspot temperature computed by the SMC, in milli-degrees C (70000..104000)",
        },
        m5_depth_msaa_compression: u32 {
            default: 1,
            description: "Enable single-layer D32 compressed 2x/4x ZLS admission",
        },
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
        legacy_inline_submit: u32 {
            default: 1,
            description: "G13/G14: publish ready in-order jobs from SUBMIT (0: off)",
        },
        legacy_rx_fastpath: u32 {
            default: 1,
            description: "G13/G14: drain GPU events on a dedicated receive worker (0: off)",
        },
        legacy_rx_util_min: u32 {
            default: 512,
            description: "G13/G14: GPU event-drain utilization minimum (0..1024)",
        },
        compute_timeout_max_ms: u32 {
            default: 30000,
            description: "G17: longest compute watchdog allowance a queue may set, ms (floor 2000)",
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
        g15_early_tiling: u32 {
            default: 0,
            description: "G15: advertise FRAGMENT_BARRIERS, mapped to early tiling (slower on 2419)",
        },
        g15_accept_text_prefix: u64 {
            default: 0,
            description: "G15: admit an unpinned GPU firmware build whose TEXT SHA-256 starts with these 8 bytes (bring-up only)",
        },
        g15_j514c_bringup: u32 {
            default: 0,
            description: "G15: opt into J514C/T6031 bring-up with the J516C reference power/thermal words (1 only; unqualified on J514C)",
        },
        stats_export: u8 {
            default: 1,
            // permissions: 0,
            description: "Export AGX firmware stats to sysfs (1 = on, 0 = off)",
        },
        mem_limit_pct: u32 {
            default: 67,
            description: "Device-wide GPU memory limit in percent of RAM (0 or >= 100: none)",
        },
        client_mem_limit_pct: u32 {
            default: 50,
            description: "Per-client GPU memory limit in percent of RAM (0: the device limit)",
        },
        oom_attribution: u8 {
            default: 1,
            description: "Count held GPU memory in its creator's RSS for the OOM killer (1 = on)",
        },
        m5_fw_trace: u32 {
            default: 0,
            description: "M5: firmware trace classes (power bit 0, submission bit 1, recovery classifier bit 9)",
        },
        g16_deadline_ms: u32 {
            default: 5000,
            description: "G16: administrative completion-stamp deadline, ms (0 disables; long running work can exceed it)",
        },
        g16_compute_deadline_ms: u32 {
            default: 5000,
            description: "G16: compute completion-stamp deadline, ms (0 permits unbounded compute; firmware recovery stays active)",
        },
        g16_repoll_us: u32 {
            default: 15,
            description: "G16: repoll interval while publication waits for record consumption, us",
        },
        g16_resnap_us: u32 {
            default: 100,
            description: "G16: completion repoll interval, us",
        },
        g16_recovery: u32 {
            default: 1,
            description: "G16: answer qualified firmware recovery handshakes",
        },
        g16_stream_gate: u32 {
            default: 1,
            description: "G16: validate CDM/VDM stream records",
        },
    },
}
