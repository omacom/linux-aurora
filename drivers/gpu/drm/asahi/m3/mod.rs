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

/// Whether the M3 runtime may start on this T8122 (M3, G15G), whose GPU the boot loader handed
/// over: its identity gate passes (only DT properties are read) and the backend selection allows
/// the board. The runtime then admits the boot loader's description, and refuses the SoC unless
/// `asahi.t8122_start=1` arms the start ([`t8122_start`]).
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
pub(crate) fn t8122_admitted(pdev: &platform::Device<Core<'_>>) -> Result<&'static m3_soc::Soc> {
    let identity = t8122_admission::read_identity(pdev).map_err(|error| {
        dev_info!(
            pdev.as_ref(),
            "G15G: T8122 identity or ABI rejected ({:?}); startup refused\n",
            error
        );
        ENODEV
    })?;
    dev_info!(
        pdev.as_ref(),
        "G15G: T8122 revision 2.0 identity admitted ({} active / {} slots)\n",
        identity.active_cores,
        identity.core_slots
    );
    if !m3_params::t8122_backend(pdev) {
        return Err(ENODEV);
    }
    Ok(&m3_soc::T8122)
}

// The T8122 runtime, ported from the 7.1.12 release-20 kernel (c46d7dcdb871).
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_attachments;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_compute;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_memory;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_resources;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_status;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod agx_uapi;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod g15_raw;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod hw;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod identity;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod initdata;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_adt_config;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_board;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_client;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_completion;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_compute;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_config;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_device;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
pub(crate) mod m3_drm;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_firmware;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_memory;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_params;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_pass;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_pool;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_render;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_resources;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_rtkit;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_runtime;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_submit;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod m3_thermal;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod t8122_knobs;
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
mod t8122_start;

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

mod agx_memory_stats;

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

// The runtime's SoC tables; they name the runtime's own modules.
#[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
pub(crate) mod m3_soc;

mod t8122_hwdata;

mod m3_init_26;
mod m3_profile;
mod m3_mmio_26;
