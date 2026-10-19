// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Driver for the Apple AGX GPUs found in Apple Silicon SoCs.

mod alloc;
mod buffer;
mod channel;
mod cleanup;
#[cfg(CONFIG_DEV_COREDUMP)]
mod crashdump;
mod debug;
mod driver;
mod g16_attachments;
mod g16_board;
mod g16_compute;
mod g16_config;
mod g16_device;
mod g16_dispatch;
mod g16_drm;
#[cfg(CONFIG_DEV_COREDUMP)]
mod g16_fault;
mod g16_firmware;
mod g16_initdata;
mod g16_j613_power;
mod g16_job;
mod g16_memory;
mod g16_owner_cache;
mod g16_parameter;
mod g16_power;
mod g16_queue;
mod g16_render;
mod g16_render_command;
mod g16_render_job;
mod g16_render_state;
mod g16_resources;
mod g16_rtkit;
mod g16_runtime;
mod g16_submit;
mod g16_uapi;
mod g16_profile;

mod drm_gpu;
mod event;
mod file;
mod float;
mod fw;
mod m3_resources;
mod m3_firmware;
mod m3_device;
mod m3_rtkit;
mod m3_init_storage;
mod m3_compute;
mod m3_render;
mod m3_queue_layout;
mod m3_sync_layout;
mod m3_state_layout;
mod m3_scene_layout;
mod m3_pool_layout;
mod m3_pool;
mod m3_parameter_layout;
mod m3_compute_layout;
mod m3_compute_sequence;
mod m3_render_sequence;
mod m3_tiler_command;
mod m3_fragment_command;
mod m3_pass_layout;
mod m3_shared_layout;
mod m3_pass;
mod m3_memory;
mod m3_compute_storage;
mod m3_submit;
mod m3_config;
mod m3_board;
mod m3_soc;
mod m3_init_layout;
mod m3_runtime;
mod m3_completion;
mod m3_timeline;
mod m3_coverage;
mod m3_drm;
mod m3_params;
mod m3_adt_config;
mod m3_thermal;
mod m3_thermal_policy;
mod m3_active_power;
mod m3_consistent_perf;
mod m3_globals_debug;
mod m3_client;
mod t8122_admission;
mod t8122_hwdata;
mod t8122_knobs;
mod t8122_start;
#[cfg(CONFIG_DEV_COREDUMP)]
mod agx_fault;
mod agx_host_progress;
mod agx_timing_stats;
mod agx_memory_stats;
mod agx_memory;
mod agx_compute;
mod agx_attachments;
mod agx_render;
mod agx_render_state;
mod agx_resources;
mod agx_status;
mod agx_uapi;
mod gem;
mod g17;
mod g15;
mod gpu;
mod hw;
mod identity;
mod initdata;
mod mem;
mod microseq;
mod mmu;
mod object;
mod pgtable;
mod pgtable_memory;
mod handoff_lock;
mod queue;
mod regs;
mod slotalloc;
mod uat;
mod util;
#[cfg(CONFIG_DRM_ASAHI_MAPLE_TREE)]
mod vm;
mod workqueue;

use kernel::prelude::*;

#[pin_data]
struct DriverModule {
    // Field drop order unregisters/joins driver producers before queue teardown.
    #[pin]
    _driver: kernel::driver::Registration<kernel::platform::Adapter<driver::AsahiDriver>>,
    // Deferred G17P cleanup work; allocated at module load on every platform.
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
        g16_packet_commands: u32 {
            default: 8,
            description: "25G83 commands per scheduler packet (1..32, at most 16 per engine)",
        },
        g16_main_overlay: u32 {
            default: 1,
            description: "25G83 fixed controller configuration (1 enabled, 0 diagnostic)",
        },
        g16_qualify: u32 {
            default: 0,
            description: "25G83 bounded boot compute self-test (0 disabled, 1 enabled)",
        },
        g16_trace_mask: u32 {
            default: 0,
            description: "25G83 firmware diagnostic trace bits (0 disabled)",
        },
        g16_j615_setup_records: u32 {
            default: 0,
            description: "J615 25G83 (experimental): 1 accepts the 26.6.2 GPU image when only values iBoot writes into it differ from the J613's",
        },
        g16_pstate: u32 {
            default: 2,
            description: "25G83 performance ceiling: default 2; explicit override bounded by calibrated OPP states",
        },
        debug_flags: u64 {
            default: 0,
            // permissions: 0o644,
            description: "Debug flags",
        },
        m3_render_batch_size: u32 {
            default: 1,
            description: "M3 bounded ordered render batch size (1..16)",
        },
        m3_compute_batch_size: u32 {
            default: 1,
            description: "M3 bounded ordered compute batch size (1..16)",
        },
        m3_early_tiling: u32 {
            default: 0,
            description: "M3 opt-in fragment-only stage dependencies",
        },
        m3_expose: i32 {
            default: -1,
            description: "M3 render node registration: -1 = auto (on apple,j514s and apple,j516s, where the M3 runtime is validated), 0 = never, 1 = always",
        },
        compute_preempt: i32 {
            default: -1,
            description: "Let the firmware preempt compute work: -1 = auto (on for M2 Max or Debug0), 0 = never, 1 = always",
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
        g15_idle_standby_us: u32 {
            default: 2500,
            description: "G15: firmware idle standby time, us (Globals +0x7a8, 100..100000)",
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
        g15_early_tiling: u32 {
            default: 0,
            description: "G15: advertise FRAGMENT_BARRIERS, mapped to early tiling (slower on 2419)",
        },
        g15_accept_text_prefix: u64 {
            default: 0,
            description: "G15: admit an unpinned GPU firmware build whose TEXT SHA-256 starts with these 8 bytes (bring-up only)",
        },

        // NOTE: every `permissions:` line in this block is commented out, so
        // the macro gives each parameter permissions 0. That means NONE of
        // these appear under /sys/module/asahi/parameters -- they can only be
        // set on the insmod command line. Looking for one in sysfs and not
        // finding it does not mean the build lacks it; check
        // `strings asahi.ko | grep parmtype=` instead.

    },
}
