// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Compile-only M3 backend. No firmware or memory owner is constructed here.

// All imported modules are type-checked; live startup stays gated.
#![allow(dead_code)]

use kernel::{device::Core, platform, prelude::*};

#[derive(Clone, Copy)]
pub(crate) enum Chip {
    T8122,
    T6030,
    T6031,
    T6032,
}

/// Refuse M3 before DMA setup, MMIO mapping, firmware access or DRM allocation.
pub(crate) fn refuse_probe(pdev: &platform::Device<Core<'_>>, chip: Chip) -> Error {
    // [ryan: adapted to 7.2.10 — minimal, pending DJ review: M3 needs an
    // independent event/runtime owner and current GPUVM backing lifetimes.]
    match chip {
        // This gate reads only DT properties. A complete hardware config and
        // exact image gate are still missing, so startup remains disabled.
        Chip::T8122 => match t8122_admission::read_identity(pdev) {
            Ok(identity) => dev_info!(
                pdev.as_ref(),
                "G15G: T8122 revision 2.0 identity admitted ({} active / {} slots); image and hardware config incomplete, startup refused\n",
                identity.active_cores,
                identity.core_slots
            ),
            Err(error) => dev_info!(
                pdev.as_ref(),
                "G15G: T8122 identity or ABI rejected ({:?}); startup refused\n",
                error
            ),
        },
        _ => dev_err!(
            pdev.as_ref(),
            "M3 unavailable: event/runtime and GPUVM adaptation pending DJ review\n"
        ),
    }
    ENODEV
}

mod m3_compute_layout;

mod m3_compute_sequence;

mod m3_compute_storage;

mod m3_coverage;

mod m3_init_layout;

mod m3_init_storage;

mod m3_parameter_layout;

mod m3_pass_layout;

mod m3_pool_layout;

mod m3_queue_layout;

mod m3_render_sequence;

mod m3_scene_layout;

mod m3_shared_layout;

mod m3_state_layout;

mod m3_sync_layout;

mod m3_thermal_policy;

mod agx_host_progress;


mod agx_timing_stats;

mod m3_timeline;

mod m3_batch_policy;

mod m3_command_layout;

mod m3_active_power;

mod m3_consistent_perf;

mod m3_globals_debug;

mod agx_render;

mod agx_render_state;

mod m3_tiler_command;

mod m3_fragment_command;

mod t8122_admission;

mod m3_soc;

mod t8122_hwdata;
