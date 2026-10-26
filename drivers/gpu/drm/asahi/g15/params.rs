// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Module parameters and the GET_PARAMS identity.
//!
//! With the shared `file.rs` additions (VM_STATUS and VM_STATUS_MIRROR from `supports_vm_status`,
//! INDEPENDENT_QUEUES from `queue_limits`, SOFT_FAULTS suppressed by `soft_faults`), the feature
//! word is exactly 0x3e (0xbe with `g15_early_tiling`). Never advertise SOFT_FAULTS,
//! FEW_PRIMITIVES or RENDER_RAY_QUERY on G15S.

use kernel::prelude::*;

use super::wire as uapi;

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
/// Published prefixes in flight per VM at most (0: the tuned value), bounded by the flight slots.
pub(crate) fn max_inflight() -> u32 {
    match *module_parameters::g15_max_inflight.value() {
        0 => hw::tuning::MAX_INFLIGHT,
        n => n,
    }
    .clamp(1, hw::sizes::FLIGHTS as u32)
}

/// The explicit firmware idle standby time, us (0: not set).
pub(crate) fn idle_standby_override_us() -> u32 {
    *module_parameters::g15_idle_standby_us.value()
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

/// The parameter for the PB growth maximum the firmware is given, in blocks, clamped to
/// 64..=4095. [`super::pools::Pools::soft_max`] raises it to at least the initial posting; the
/// blocks above it, up to the 4095 posted at most, answer the synchronous requests of partial
/// renders. It is read-only after boot and does not change the heap's backing.
pub(crate) fn pb_max_blocks() -> u32 {
    (*module_parameters::g15_pb_max_blocks.value()).clamp(64, super::hw::tuning::PB_MAX_BLOCKS)
}

/// The initial PB posting, in blocks (64..=4095; the ledger clamps it again). Read-only after
/// boot.
pub(crate) fn pb_blocks() -> u32 {
    (*module_parameters::g15_pb_blocks.value()).clamp(64, super::hw::tuning::PB_MAX_BLOCKS)
}

/// PB reclaim at idle: 0 never, 1 re-initialize, 2 re-initialize and free the backing.
pub(crate) fn pb_reclaim() -> u32 {
    (*module_parameters::g15_pb_reclaim.value()).min(2)
}

/// Idle time before a PB reclaim, ns (at least 100 ms).
pub(crate) fn pb_idle_ns() -> u64 {
    (*module_parameters::g15_pb_idle_ms.value()).max(100) as u64 * 1_000_000
}

/// Posted blocks above which an idle engine reclaims the PB.
pub(crate) fn pb_reclaim_blocks() -> u32 {
    *module_parameters::g15_pb_reclaim_blocks.value()
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

/// Shared status/queue features in the native G15 ABI's assignments.
pub(crate) fn feature_flags() -> u64 {
    uapi::SHARED_FEATURE_FLAGS
}
