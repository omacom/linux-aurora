// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Module parameters and the GET_PARAMS identity.
//!
//! With the shared `file.rs` additions (VM_STATUS and VM_STATUS_MIRROR from `supports_vm_status`,
//! INDEPENDENT_QUEUES from `queue_limits`, SOFT_FAULTS suppressed by `soft_faults`), the feature
//! word is exactly 0x3e (0xbe with `g15_early_tiling`). Never advertise SOFT_FAULTS,
//! FEW_PRIMITIVES or RENDER_RAY_QUERY on G15S.

use kernel::{prelude::*, uapi};

use super::{hw, Gpu, GPU_GENERATION};
use crate::module_parameters;

/// Operations per VM_BIND batch at most: batches are atomic and may be large (sparse residency
/// updates); this bounds the kernel's staging allocation.
pub(crate) const MAX_BINDS: u32 = 1 << 20;

/// Host deadline of a published prefix without progress, ms (at least 100).
pub(crate) fn deadline_ms() -> u32 {
    (*module_parameters::g15_deadline_ms.value()).max(100)
}

/// Compute no-progress budget before the kill, ms.
pub(crate) fn compute_deadline_ms() -> u32 {
    (*module_parameters::g15_compute_deadline_ms.value()).max(100)
}

/// Repoll while a ready prefix waits for record consumption, us.
pub(crate) fn repoll_us() -> u32 {
    (*module_parameters::g15_repoll_us.value()).clamp(5, 100)
}

/// Completion repoll interval, us.
pub(crate) fn resnap_us() -> u32 {
    (*module_parameters::g15_resnap_us.value()).clamp(25, 1000)
}

/// Firmware idle standby time, us.
pub(crate) fn idle_standby_us() -> u32 {
    (*module_parameters::g15_idle_standby_us.value()).clamp(100, 100000)
}

/// Firmware idle minimum confidence, permille.
pub(crate) fn idle_min_conf() -> u32 {
    (*module_parameters::g15_idle_min_conf.value()).clamp(1, 1000)
}

/// Whether firmware halts are answered with the handshake (else fail-stop).
pub(crate) fn recovery() -> bool {
    *module_parameters::g15_recovery.value() != 0
}

/// Whether the CDM/VDM stream gates are on.
pub(crate) fn stream_gate() -> bool {
    *module_parameters::g15_stream_gate.value() != 0
}

/// The 8-byte TEXT digest prefix of an unpinned firmware build to admit (0: none).
pub(crate) fn accept_text_prefix() -> u64 {
    *module_parameters::g15_accept_text_prefix.value()
}

/// Whether FRAGMENT_BARRIERS is advertised and mapped to early tiling.
pub(crate) fn early_tiling() -> bool {
    *module_parameters::g15_early_tiling.value() != 0
}

/// Fills the generation part of GET_PARAMS group 0.
pub(crate) fn fill(gpu: &Gpu, params: &mut uapi::drm_asahi_params_global) -> Result {
    params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_PER_COMMAND_INPUT_SYNCS as u64
        | uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_EXACT_PRIOR_BARRIERS as u64;
    if early_tiling() {
        params.features |= uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_BARRIERS as u64;
    }
    params.gpu_generation = GPU_GENERATION;
    params.gpu_variant = gpu.cfg.gpu_variant as u32;
    params.gpu_revision = gpu.cfg.gpu_revision;
    params.chip_id = gpu.cfg.chip_id;
    params.num_dies = 1;
    params.num_clusters_total = gpu.cfg.num_clusters;
    params.num_cores_per_cluster = gpu.cfg.num_cores_per_cluster;
    params.max_frequency_khz = gpu.max_frequency_khz;
    params.usc_generation = 3;
    params.gpu_hal_generation = uapi::drm_asahi_gpu_hal_generation_DRM_ASAHI_GPU_HAL_LEGACY;
    params.max_attachments = hw::sizes::MAX_ATTACHMENTS as u32;
    for (i, mask) in gpu
        .id
        .core_masks
        .iter()
        .enumerate()
        .take(gpu.cfg.num_clusters as usize)
    {
        *(params.core_masks.get_mut(i).ok_or(EIO)?) = (*mask).into();
    }
    Ok(())
}

/// QUEUE_LIMITS: a policy bound on logical queues (they own no firmware state) and the
/// number of prefixes in flight.
pub(crate) fn queue_limits() -> uapi::drm_asahi_queue_limits {
    uapi::drm_asahi_queue_limits {
        max_queues: hw::sizes::MAX_QUEUES,
        max_in_flight_per_queue: hw::sizes::FLIGHTS as u32,
        ..Default::default()
    }
}
