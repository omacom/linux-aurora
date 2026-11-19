// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Private G15 wire for Mesa 751 / public SF `asahi_m3_drm.h`.
//!
//! G15 modules that decode commands or advertise features do
//! `use crate::g15::wire as uapi`. Normal and G17 code keep `kernel::uapi`.
//! This module is not installed UAPI and is not a bindgen input.
//! `include/uapi/drm/asahi_drm.h` stays the canonical header: render 448 bytes
//! with `prior_vdm` at 320, compute 200 bytes with `scs_layout` at 128.
//!
//! Native feature numbers, matching that Mesa header. Canonical bits 10..15
//! are a different assignment; Mesa rejects any of them (`0xfc00`).
//!
//! | bit | native name | canonical name at the same bit |
//! | 0 | SOFT_FAULTS | SOFT_FAULTS |
//! | 1 | VM_STATUS | VM_STATUS |
//! | 2 | INDEPENDENT_QUEUES | INDEPENDENT_QUEUES |
//! | 3 | PER_COMMAND_INPUT_SYNCS | SCHEDULED_QUEUES |
//! | 4 | VM_STATUS_MIRROR | RENDER_TIMESTAMP_COPIES |
//! | 5 | EXACT_PRIOR_BARRIERS | COMPUTE_TIMESTAMP_COPIES |
//! | 6 | FEW_PRIMITIVES | FRAGMENT_DEPENDENCY |
//! | 7 | FRAGMENT_BARRIERS | FRAGMENT_DEPENDENCY_COMPUTE |
//! | 8 | RENDER_RAY_QUERY | COMPUTE_WIDE_VISIBILITY |

#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::mem::{offset_of, size_of};

use kernel::uapi;

use crate::util::AnyBitPattern;

/// Canonical bits 10..15. A G15 feature word with any of these set is the
/// maintained assignment, which Mesa 751 refuses for chip 0x6030 and 0x6031.
pub(crate) const LEGACY_FEATURE_MASK: u64 = 0xfc00;

// Bindgen names, so `use crate::g15::wire as uapi` still matches params.rs.
// Scoped to these nine items; the rest of the module stays under the lint.
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_SOFT_FAULTS: u64 = 1 << 0;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS: u64 = 1 << 1;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_INDEPENDENT_QUEUES: u64 = 1 << 2;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_PER_COMMAND_INPUT_SYNCS: u64 = 1 << 3;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS_MIRROR: u64 = 1 << 4;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_EXACT_PRIOR_BARRIERS: u64 = 1 << 5;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_FEW_PRIMITIVES: u64 = 1 << 6;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_BARRIERS: u64 = 1 << 7;
#[allow(non_upper_case_globals)]
pub(crate) const drm_asahi_feature_DRM_ASAHI_FEATURE_RENDER_RAY_QUERY: u64 = 1 << 8;

/// What `Gpu::feature_flags` returns for G15. This replaces the shared default
/// (canonical VM_STATUS | VM_STATUS_MIRROR | INDEPENDENT_QUEUES | optional
/// SOFT_FAULTS). Mirror is native bit 4. Soft faults, few-primitives and ray
/// query are not advertised. Command bits 3, 5 and 7 stay in `params::fill`
/// via this module's names, so this constant does not include them.
pub(crate) const SHARED_FEATURE_FLAGS: u64 = drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS
    | drm_asahi_feature_DRM_ASAHI_FEATURE_INDEPENDENT_QUEUES
    | drm_asahi_feature_DRM_ASAHI_FEATURE_VM_STATUS_MIRROR;

/// Command-feature bits `params::fill` ORs. Bit 7 only while early tiling is on.
pub(crate) const fn command_feature_flags(early_tiling: bool) -> u64 {
    let mut flags = drm_asahi_feature_DRM_ASAHI_FEATURE_PER_COMMAND_INPUT_SYNCS
        | drm_asahi_feature_DRM_ASAHI_FEATURE_EXACT_PRIOR_BARRIERS;
    if early_tiling {
        flags |= drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_BARRIERS;
    }
    flags
}

/// Full G15 GET_PARAMS word: `0x3e`, or `0xbe` with early tiling.
pub(crate) const fn feature_word(early_tiling: bool) -> u64 {
    SHARED_FEATURE_FLAGS | command_feature_flags(early_tiling)
}

/// Ioctl types and constants whose layout and values match the canonical header.
/// Render flags used by G15 are the same numbers on both headers (1<<0, 1<<1,
/// 1<<2, 1<<4, 1<<18, 1<<20). Feature constants are not reexported.
pub(crate) use uapi::{
    drm_asahi_attachment, drm_asahi_bg_eot, drm_asahi_cmd_header, drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE,
    drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER, drm_asahi_cmd_type_DRM_ASAHI_SET_COMPUTE_ATTACHMENTS,
    drm_asahi_cmd_type_DRM_ASAHI_SET_FRAGMENT_ATTACHMENTS,
    drm_asahi_cmd_type_DRM_ASAHI_SET_VERTEX_ATTACHMENTS, drm_asahi_gpu_hal_generation_DRM_ASAHI_GPU_HAL_LEGACY,
    drm_asahi_helper_program, drm_asahi_params_global, drm_asahi_queue_limits,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_DBIAS_IS_INT,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_FRAGMENT_BARRIERS,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_RSRC_SPEC_HI,
    drm_asahi_render_flags_DRM_ASAHI_RENDER_VERTEX_SCRATCH, drm_asahi_timestamp,
    drm_asahi_timestamps, drm_asahi_zls_buffer, DRM_ASAHI_BARRIER_NONE,
};

/// Native render command. 384 bytes. The first 256 match the canonical prefix
/// through `bg_partial_rsrc_spec_hi` (offset 252). `prior_vdm` is at 256 and
/// `prior_cdm` at 264. `ray_query` is at 272 and is not advertised: a nonzero
/// byte from there on is rejected by `command::check_stream`. A 256-byte packet
/// zero-fills the priors; a 272-byte packet zero-fills `ray_query`.
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub(crate) struct drm_asahi_cmd_render {
    pub flags: u32,
    pub isp_zls_pixels: u32,
    pub vdm_ctrl_stream_base: u64,
    pub vertex_helper: drm_asahi_helper_program,
    pub fragment_helper: drm_asahi_helper_program,
    pub isp_scissor_base: u64,
    pub isp_dbias_base: u64,
    pub isp_oclqry_base: u64,
    pub depth: drm_asahi_zls_buffer,
    pub stencil: drm_asahi_zls_buffer,
    pub zls_ctrl: u64,
    pub ppp_multisamplectl: u64,
    pub sampler_heap: u64,
    pub ppp_ctrl: u32,
    pub width_px: u16,
    pub height_px: u16,
    pub layers: u16,
    pub sampler_count: u16,
    pub utile_width_px: u8,
    pub utile_height_px: u8,
    pub samples: u8,
    pub sample_size_B: u8,
    pub isp_merge_upper_x: u32,
    pub isp_merge_upper_y: u32,
    pub bg: drm_asahi_bg_eot,
    pub eot: drm_asahi_bg_eot,
    pub partial_bg: drm_asahi_bg_eot,
    pub partial_eot: drm_asahi_bg_eot,
    pub isp_bgobjdepth: u32,
    pub isp_bgobjvals: u32,
    pub ts_vtx: drm_asahi_timestamps,
    pub ts_frag: drm_asahi_timestamps,
    pub bg_eot_rsrc_spec_hi: u32,
    pub bg_eot_partial_rsrc_spec_hi: u32,
    pub bg_rsrc_spec_hi: u32,
    pub bg_partial_rsrc_spec_hi: u32,
    pub prior_vdm: u64,
    pub prior_cdm: u64,
    pub ray_query: [uapi::drm_asahi_render_ray_query_stage; 2],
}

/// Native compute command. 136 bytes. The 64-byte prefix ends at `ts`.
/// `scs_layout` is at 64, `prior_vdm` at 72, `prior_cdm` at 80. G15 rejects a
/// nonzero `scs_layout` and the intersection tail; those fields still sit at
/// the native offsets so a 136-byte packet is not read as canonical scratch.
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub(crate) struct drm_asahi_cmd_compute {
    pub flags: u32,
    pub sampler_count: u32,
    pub cdm_ctrl_stream_base: u64,
    pub cdm_ctrl_stream_end: u64,
    pub sampler_heap: u64,
    pub helper: drm_asahi_helper_program,
    pub ts: drm_asahi_timestamps,
    pub scs_layout: u64,
    pub prior_vdm: u64,
    pub prior_cdm: u64,
    pub cdm_auxiliary_allocation: u32,
    pub reserved: u32,
    pub intersection_pool: u64,
    pub intersection_pool_bytes: u64,
    pub intersection_first: u32,
    pub intersection_count: u32,
    pub intersection_default_program: u64,
    pub intersection_default_control: u64,
}

// SAFETY: Every field is an integer or a `repr(C)` UAPI struct of integers.
// `command::payload` zero-fills a short record and rejects a nonzero tail.
unsafe impl AnyBitPattern for drm_asahi_cmd_render {}
// SAFETY: Same as `drm_asahi_cmd_render`.
unsafe impl AnyBitPattern for drm_asahi_cmd_compute {}

const _: () = {
    assert!(size_of::<uapi::drm_asahi_cmd_render>() == 480);
    assert!(offset_of!(uapi::drm_asahi_cmd_render, prior_vdm) == 320);
    assert!(size_of::<uapi::drm_asahi_cmd_compute>() == 200);
    assert!(offset_of!(uapi::drm_asahi_cmd_compute, scs_layout) == 128);

    assert!(size_of::<drm_asahi_cmd_render>() == 384);
    assert!(offset_of!(drm_asahi_cmd_render, bg_partial_rsrc_spec_hi) == 252);
    assert!(offset_of!(drm_asahi_cmd_render, prior_vdm) == 256);
    assert!(offset_of!(drm_asahi_cmd_render, prior_cdm) == 264);
    assert!(offset_of!(drm_asahi_cmd_render, ray_query) == 272);

    assert!(size_of::<drm_asahi_cmd_compute>() == 136);
    assert!(offset_of!(drm_asahi_cmd_compute, scs_layout) == 64);
    assert!(offset_of!(drm_asahi_cmd_compute, prior_vdm) == 72);
    assert!(offset_of!(drm_asahi_cmd_compute, prior_cdm) == 80);
    assert!(offset_of!(drm_asahi_cmd_compute, cdm_auxiliary_allocation) == 88);
    assert!(offset_of!(drm_asahi_cmd_compute, intersection_default_control) == 128);

    assert!(SHARED_FEATURE_FLAGS == 0x16);
    assert!(command_feature_flags(false) == 0x28);
    assert!(feature_word(false) == 0x3e);
    assert!(feature_word(true) == 0xbe);
    assert!(feature_word(true) & LEGACY_FEATURE_MASK == 0);
    assert!(drm_asahi_feature_DRM_ASAHI_FEATURE_RENDER_RAY_QUERY == 1 << 8);
    assert!(drm_asahi_feature_DRM_ASAHI_FEATURE_FEW_PRIMITIVES == 1 << 6);
    assert!(drm_asahi_feature_DRM_ASAHI_FEATURE_SOFT_FAULTS == 1);
};
