// SPDX-License-Identifier: GPL-2.0-only OR MIT

#![cfg_attr(not(test), allow(dead_code))]

//! Pure modern-Asahi UAPI parsing and G17P command translation.
//!
//! The byte layouts match `drm_asahi_cmd_header`, `drm_asahi_cmd_render`,
//! `drm_asahi_cmd_compute`, and `drm_asahi_attachment`. Translation validates
//! the queue's 4 GiB USC window and every userspace range before constructing
//! G17P render parameters or compute-entry operands. This module allocates no
//! object, publishes no queue state, and performs no firmware or MMIO action.

#[cfg(not(test))]
use crate::{g18::fw::compute_params as g17_compute, g18::fw::render_params as g17_render, g18::fw::submission as g17_submission};
#[cfg(test)]
#[path = "../fw/compute_params.rs"]
mod g17_compute;
#[cfg(test)]
#[path = "../fw/render_params.rs"]
mod g17_render;
#[cfg(test)]
#[allow(dead_code)]
#[path = "../fw/submission.rs"]
mod g17_submission;

use g17_render::{
    G17pRenderParameters, RenderBuildError, RenderDescriptorMetadata, RenderDescriptorObjects,
};
use g17_submission::{
    G17PClBarrierDependency, G17PClKickEntryOperands, G17PClMcacheAperture, G17PClRceBinding,
};

pub(crate) const UAPI_COMMAND_HEADER_SIZE: usize = 8;
/// `sizeof(struct drm_asahi_cmd_render)`. Shorter payloads are zero-extended.
pub(crate) const UAPI_RENDER_SIZE: usize = 448;
/// `drm_asahi_cmd_render::ts_frag_end_copies`, which G18 does not implement:
/// the words must be zero.
const RENDER_TIMESTAMP_COPIES: core::ops::Range<usize> = 0x100..0x140;
/// `drm_asahi_cmd_render::prior_vdm` and `prior_cdm`.
const RENDER_PRIOR_VDM: usize = 0x140;
const RENDER_PRIOR_CDM: usize = 0x148;
/// `drm_asahi_cmd_render::ray_query`. G18 has no ray queries: the words
/// must be zero.
const RENDER_RAY_QUERY: core::ops::Range<usize> = 0x150..UAPI_RENDER_SIZE;
/// `sizeof(struct drm_asahi_cmd_render_g18)`: the complete render payload
/// followed by the 80-byte `drm_asahi_g18_render_resources`.
pub(crate) const UAPI_G18_RENDER_SIZE: usize = UAPI_RENDER_SIZE + 80;
/// `sizeof(struct drm_asahi_cmd_compute)`. Shorter payloads are zero-extended.
pub(crate) const UAPI_COMPUTE_SIZE: usize = 200;
/// `drm_asahi_cmd_compute::ts_end_copies`, which G18 does not implement.
const COMPUTE_TIMESTAMP_COPIES: core::ops::Range<usize> = 0x40..0x80;
/// `drm_asahi_cmd_compute::scs_layout`.
const COMPUTE_SCS_LAYOUT: usize = 0x80;
/// `drm_asahi_cmd_compute::prior_vdm` and `prior_cdm`.
const COMPUTE_PRIOR_VDM: usize = 0x88;
const COMPUTE_PRIOR_CDM: usize = 0x90;
/// `cdm_auxiliary_allocation` through `intersection_default_control`, which
/// G18 does not implement: the words must be zero.
const COMPUTE_UNSUPPORTED: core::ops::Range<usize> = 0x98..UAPI_COMPUTE_SIZE;
/// `sizeof(struct drm_asahi_cmd_compute_g18)`: the complete compute payload
/// followed by the 64-byte `drm_asahi_g18_compute_resources`.
pub(crate) const UAPI_G18_COMPUTE_SIZE: usize = UAPI_COMPUTE_SIZE + 64;
/// `DRM_ASAHI_COMPUTE_G18_RESOURCES`.
pub(crate) const COMPUTE_G18_RESOURCES: u32 = 1 << 19;
/// `DRM_ASAHI_RENDER_G18_RESOURCES`. Only the G18 adapter may consume and
/// strip it; the shared translator's RENDER_FLAGS keeps refusing it.
pub(crate) const RENDER_G18_RESOURCES: u32 = 1 << 25;
pub(crate) const UAPI_ATTACHMENT_SIZE: usize = 24;
pub(crate) const UAPI_MAX_ATTACHMENTS: usize = 16;
pub(crate) const UAPI_MAX_HARDWARE_COMMANDS: u32 = 64;
// Match the image/framebuffer limit advertised by Borealis. Storage is
// sized from each submission; this is a format limit, not a demo window cap.
pub(crate) const M5_MAX_RENDER_DIMENSION: u32 = 16384;
pub(crate) const UAPI_BARRIER_NONE: u16 = 0xffff;
pub(crate) const USC_WINDOW_SIZE: u64 = 1 << 32;
#[cfg(not(test))]
pub(crate) const COMPUTE_G17P_ADD3_PROOF: u32 =
    kernel::uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_G17P_ADD3_PROOF as u32;
#[cfg(test)]
pub(crate) const COMPUTE_G17P_ADD3_PROOF: u32 = 1 << 0;
const G17P_ADD3_BUFFER_BYTES: u64 = 64 * 4;

const COMMAND_RENDER: u16 = 0;
const COMMAND_COMPUTE: u16 = 1;
const SET_VERTEX_ATTACHMENTS: u16 = 2;
const SET_FRAGMENT_ATTACHMENTS: u16 = 3;
const SET_COMPUTE_ATTACHMENTS: u16 = 4;

const RENDER_VERTEX_SCRATCH: u32 = 1 << 0;
const RENDER_PROCESS_EMPTY_TILES: u32 = 1 << 1;
const RENDER_NO_VERTEX_CLUSTERING: u32 = 1 << 2;
const RENDER_RSRC_SPEC_HI: u32 = 1 << 4;
const RENDER_DBIAS_IS_INT: u32 = 1 << 18;
const RENDER_FLAGS: u32 = RENDER_VERTEX_SCRATCH
    | RENDER_PROCESS_EMPTY_TILES
    | RENDER_NO_VERTEX_CLUSTERING
    | RENDER_RSRC_SPEC_HI
    | RENDER_DBIAS_IS_INT
    // Private, opt-in matched-workload diagnostic. Ordinary builds reject it.
    | if cfg!(g17p_aux_fb_small_probe) { 1 << 31 } else { 0 }
    | if cfg!(g17p_mcache_aperture_probe) { 1 << 30 } else { 0 };

const SAMPLER_SIZE: u64 = 8;
const MAX_SAMPLERS: u32 = 1024;
const PAGE_SIZE: u64 = 0x4000;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum UapiTranslateError {
    HeaderTruncated,
    PayloadTruncated,
    PayloadTail,
    CommandType,
    AttachmentPayloadSize,
    AttachmentCount,
    AttachmentFields,
    AttachmentBarrier,
    HardwareCommandCount,
    BarrierFuture,
    QueueUscAlignment,
    QueueUscRange,
    AddressRange,
    AddressAlignment,
    FlagsUnknownBits,
    FlagsUnsupported,
    Dimensions,
    Layers,
    UtileDimensions,
    Samples,
    TilebufferSize,
    SamplerCount,
    SamplerPair,
    HelperProgram,
    Add3ProofAttachments,
    ProgramOffset,
    ZlsFields,
    StencilClearBits,
    RenderBuild(RenderBuildError),
    ComputeBuild(g17_compute::ComputeBuildError),
}

impl From<RenderBuildError> for UapiTranslateError {
    fn from(value: RenderBuildError) -> Self {
        Self::RenderBuild(value)
    }
}

impl From<g17_compute::ComputeBuildError> for UapiTranslateError {
    fn from(value: g17_compute::ComputeBuildError) -> Self {
        Self::ComputeBuild(value)
    }
}

impl UapiTranslateError {
    /// Stable short name for one rejection, so a failing submit reports which
    /// check refused it instead of a bare EINVAL. Kept as `&'static str` (no
    /// `Debug` formatting) to stay cheap on the ioctl path.
    pub(crate) fn name(&self) -> &'static str {
        match self {
            Self::HeaderTruncated => "header-truncated",
            Self::PayloadTruncated => "payload-truncated",
            Self::PayloadTail => "payload-tail",
            Self::CommandType => "command-type",
            Self::AttachmentPayloadSize => "attachment-payload-size",
            Self::AttachmentCount => "attachment-count",
            Self::AttachmentFields => "attachment-fields",
            Self::AttachmentBarrier => "attachment-barrier",
            Self::HardwareCommandCount => "hardware-command-count",
            Self::BarrierFuture => "barrier-future",
            Self::QueueUscAlignment => "queue-usc-alignment",
            Self::QueueUscRange => "queue-usc-range",
            Self::AddressRange => "address-range",
            Self::AddressAlignment => "address-alignment",
            Self::FlagsUnknownBits => "flags-unknown-bits",
            Self::FlagsUnsupported => "flags-unsupported",
            Self::Dimensions => "dimensions",
            Self::Layers => "layers",
            Self::UtileDimensions => "utile-dimensions",
            Self::Samples => "samples",
            Self::TilebufferSize => "tilebuffer-size",
            Self::SamplerCount => "sampler-count",
            Self::SamplerPair => "sampler-pair",
            Self::HelperProgram => "helper-program",
            Self::Add3ProofAttachments => "add3-proof-attachments",
            Self::ProgramOffset => "program-offset",
            Self::ZlsFields => "zls-fields",
            Self::StencilClearBits => "stencil-clear-bits",
            Self::RenderBuild(_) => "render-build",
            Self::ComputeBuild(_) => "compute-build",
        }
    }
}

fn get_u16(raw: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(raw[offset..offset + 2].try_into().unwrap())
}

fn get_u32(raw: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(raw[offset..offset + 4].try_into().unwrap())
}

fn get_u64(raw: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(raw[offset..offset + 8].try_into().unwrap())
}

fn padded_payload<const N: usize>(raw: &[u8]) -> Result<[u8; N], UapiTranslateError> {
    if raw.len() > N && raw[N..].iter().any(|byte| *byte != 0) {
        return Err(UapiTranslateError::PayloadTail);
    }
    let mut out = [0u8; N];
    let count = core::cmp::min(raw.len(), N);
    out[..count].copy_from_slice(&raw[..count]);
    Ok(out)
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiCommandHeader {
    pub(crate) command_type: u16,
    pub(crate) size: u16,
    pub(crate) render_barrier: u16,
    pub(crate) compute_barrier: u16,
}

impl UapiCommandHeader {
    fn parse(raw: &[u8]) -> Result<Self, UapiTranslateError> {
        if raw.len() < UAPI_COMMAND_HEADER_SIZE {
            return Err(UapiTranslateError::HeaderTruncated);
        }
        Ok(Self {
            command_type: get_u16(raw, 0),
            size: get_u16(raw, 2),
            render_barrier: get_u16(raw, 4),
            compute_barrier: get_u16(raw, 6),
        })
    }
}

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub(crate) struct UapiAttachment {
    pub(crate) address: u64,
    pub(crate) size: u64,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiAttachmentList {
    pub(crate) entries: [UapiAttachment; UAPI_MAX_ATTACHMENTS],
    pub(crate) count: u8,
}

impl UapiAttachmentList {
    pub(crate) const EMPTY: Self = Self {
        entries: [UapiAttachment {
            address: 0,
            size: 0,
        }; UAPI_MAX_ATTACHMENTS],
        count: 0,
    };

    pub(crate) fn as_slice(&self) -> &[UapiAttachment] {
        &self.entries[..self.count as usize]
    }

    fn parse(raw: &[u8]) -> Result<Self, UapiTranslateError> {
        if raw.len() % UAPI_ATTACHMENT_SIZE != 0 {
            return Err(UapiTranslateError::AttachmentPayloadSize);
        }
        let count = raw.len() / UAPI_ATTACHMENT_SIZE;
        if count > UAPI_MAX_ATTACHMENTS {
            return Err(UapiTranslateError::AttachmentCount);
        }
        let mut out = Self::EMPTY;
        for index in 0..count {
            let offset = index * UAPI_ATTACHMENT_SIZE;
            if get_u32(raw, offset + 0x10) != 0 || get_u32(raw, offset + 0x14) != 0 {
                return Err(UapiTranslateError::AttachmentFields);
            }
            out.entries[index] = UapiAttachment {
                address: get_u64(raw, offset),
                size: get_u64(raw, offset + 8),
            };
        }
        out.count = count as u8;
        Ok(out)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiHelperProgram {
    pub(crate) binary: u32,
    pub(crate) config: u32,
    pub(crate) data: u64,
}

fn helper(raw: &[u8], offset: usize) -> UapiHelperProgram {
    UapiHelperProgram {
        binary: get_u32(raw, offset),
        config: get_u32(raw, offset + 4),
        data: get_u64(raw, offset + 8),
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiProgram {
    pub(crate) usc: u32,
    pub(crate) resource_spec: u64,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiTimestamp {
    pub(crate) handle: u32,
    pub(crate) offset: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiTimestamps {
    pub(crate) start: UapiTimestamp,
    pub(crate) end: UapiTimestamp,
}

fn timestamps(raw: &[u8], offset: usize) -> UapiTimestamps {
    UapiTimestamps {
        start: UapiTimestamp {
            handle: get_u32(raw, offset),
            offset: get_u32(raw, offset + 4),
        },
        end: UapiTimestamp {
            handle: get_u32(raw, offset + 8),
            offset: get_u32(raw, offset + 12),
        },
    }
}

fn program(raw: &[u8], offset: usize, high_offset: usize) -> UapiProgram {
    UapiProgram {
        usc: get_u32(raw, offset),
        resource_spec: get_u32(raw, offset + 4) as u64
            | (get_u32(raw, high_offset) as u64) << 32,
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiZlsBuffer {
    pub(crate) base: u64,
    pub(crate) compression_base: u64,
    pub(crate) stride: u32,
    pub(crate) compression_stride: u32,
}

fn zls(raw: &[u8], offset: usize) -> UapiZlsBuffer {
    UapiZlsBuffer {
        base: get_u64(raw, offset),
        compression_base: get_u64(raw, offset + 8),
        stride: get_u32(raw, offset + 0x10),
        compression_stride: get_u32(raw, offset + 0x14),
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiRenderCommand {
    pub(crate) flags: u32,
    pub(crate) depth_dimensions: u32,
    pub(crate) vdm_base: u64,
    pub(crate) vertex_helper: UapiHelperProgram,
    pub(crate) fragment_helper: UapiHelperProgram,
    pub(crate) scissor_base: u64,
    pub(crate) depth_bias_base: u64,
    pub(crate) occlusion_query_base: u64,
    pub(crate) depth: UapiZlsBuffer,
    pub(crate) stencil: UapiZlsBuffer,
    pub(crate) zls_control: u64,
    pub(crate) multisample_control: u64,
    pub(crate) sampler_heap: u64,
    pub(crate) ppp_control: u32,
    pub(crate) width: u16,
    pub(crate) height: u16,
    pub(crate) layers: u16,
    pub(crate) sampler_count: u16,
    pub(crate) utile_width: u8,
    pub(crate) utile_height: u8,
    pub(crate) samples: u8,
    pub(crate) sample_size: u8,
    pub(crate) merge_upper_x: u32,
    pub(crate) merge_upper_y: u32,
    pub(crate) background: UapiProgram,
    pub(crate) end_of_tile: UapiProgram,
    pub(crate) partial_background: UapiProgram,
    pub(crate) partial_end_of_tile: UapiProgram,
    pub(crate) depth_clear: u32,
    pub(crate) stencil_clear: u32,
    pub(crate) vertex_timestamps: UapiTimestamps,
    pub(crate) fragment_timestamps: UapiTimestamps,
    pub(crate) prior_vdm: u64,
    pub(crate) prior_cdm: u64,
    /// `drm_asahi_g18_render_resources` words; zero unless flagged.
    pub(crate) g18_resources: [u32; 20],
}

impl UapiRenderCommand {
    fn parse(raw: &[u8]) -> Result<Self, UapiTranslateError> {
        // Shorter payloads are zero-extended. A flagged G18 suffix must be
        // complete; padding could hide a truncated field. Words G18 does not
        // implement (timestamp copies, ray queries) must be zero, as must
        // anything after the structure or, when flagged, its suffix.
        let has_resources = raw.len() >= 4 && get_u32(raw, 0) & RENDER_G18_RESOURCES != 0;
        if has_resources && raw.len() < UAPI_G18_RENDER_SIZE {
            return Err(UapiTranslateError::PayloadTruncated);
        }
        let raw = padded_payload::<UAPI_G18_RENDER_SIZE>(raw)?;
        let unused_tail = if has_resources { UAPI_G18_RENDER_SIZE } else { UAPI_RENDER_SIZE };
        if raw[RENDER_TIMESTAMP_COPIES].iter().chain(&raw[RENDER_RAY_QUERY])
            .chain(&raw[unused_tail..]).any(|byte| *byte != 0)
        {
            return Err(UapiTranslateError::PayloadTail);
        }
        let mut g18_resources = [0; 20];
        if has_resources {
            for (i, word) in g18_resources.iter_mut().enumerate() {
                *word = get_u32(&raw, UAPI_RENDER_SIZE + i * 4);
            }
        }
        Ok(Self {
            flags: get_u32(&raw, 0x00),
            depth_dimensions: get_u32(&raw, 0x04),
            vdm_base: get_u64(&raw, 0x08),
            vertex_helper: helper(&raw, 0x10),
            fragment_helper: helper(&raw, 0x20),
            scissor_base: get_u64(&raw, 0x30),
            depth_bias_base: get_u64(&raw, 0x38),
            occlusion_query_base: get_u64(&raw, 0x40),
            depth: zls(&raw, 0x48),
            stencil: zls(&raw, 0x60),
            zls_control: get_u64(&raw, 0x78),
            multisample_control: get_u64(&raw, 0x80),
            sampler_heap: get_u64(&raw, 0x88),
            ppp_control: get_u32(&raw, 0x90),
            width: get_u16(&raw, 0x94),
            height: get_u16(&raw, 0x96),
            layers: get_u16(&raw, 0x98),
            sampler_count: get_u16(&raw, 0x9a),
            utile_width: raw[0x9c],
            utile_height: raw[0x9d],
            samples: raw[0x9e],
            sample_size: raw[0x9f],
            merge_upper_x: get_u32(&raw, 0xa0),
            merge_upper_y: get_u32(&raw, 0xa4),
            background: program(&raw, 0xa8, 0xf8),
            end_of_tile: program(&raw, 0xb0, 0xf0),
            partial_background: program(&raw, 0xb8, 0xfc),
            partial_end_of_tile: program(&raw, 0xc0, 0xf4),
            depth_clear: get_u32(&raw, 0xc8),
            stencil_clear: get_u32(&raw, 0xcc),
            vertex_timestamps: timestamps(&raw, 0xd0),
            fragment_timestamps: timestamps(&raw, 0xe0),
            prior_vdm: get_u64(&raw, RENDER_PRIOR_VDM),
            prior_cdm: get_u64(&raw, RENDER_PRIOR_CDM),
            g18_resources,
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UapiComputeCommand {
    pub(crate) flags: u32,
    pub(crate) sampler_count: u32,
    pub(crate) control_stream_base: u64,
    pub(crate) control_stream_end: u64,
    pub(crate) sampler_heap: u64,
    pub(crate) helper: UapiHelperProgram,
    pub(crate) timestamps: UapiTimestamps,
    pub(crate) scs_layout: u64,
    pub(crate) g18_resources: [u32; 16],
    pub(crate) prior_vdm: u64,
    pub(crate) prior_cdm: u64,
}

impl UapiComputeCommand {
    fn parse(raw: &[u8]) -> Result<Self, UapiTranslateError> {
        // Shorter compute payloads are zero-extended. A flagged metadata
        // payload must be complete; padding it could hide truncation of a
        // valid zero-valued field or reserved word. Words G18 does not
        // implement (timestamp copies, the auxiliary allocation, intersection
        // tables) must be zero, as must anything after the structure or,
        // when flagged, its suffix.
        let has_resources = raw.len() >= 4 && get_u32(raw, 0) & COMPUTE_G18_RESOURCES != 0;
        if has_resources && raw.len() < UAPI_G18_COMPUTE_SIZE {
            return Err(UapiTranslateError::PayloadTruncated);
        }
        let raw = padded_payload::<UAPI_G18_COMPUTE_SIZE>(raw)?;
        let unused_tail = if has_resources { UAPI_G18_COMPUTE_SIZE } else { UAPI_COMPUTE_SIZE };
        if raw[COMPUTE_TIMESTAMP_COPIES].iter().chain(&raw[COMPUTE_UNSUPPORTED])
            .chain(&raw[unused_tail..]).any(|byte| *byte != 0)
        {
            return Err(UapiTranslateError::PayloadTail);
        }
        let mut g18_resources = [0; 16];
        if has_resources {
            for (i, word) in g18_resources.iter_mut().enumerate() {
                *word = get_u32(&raw, UAPI_COMPUTE_SIZE + i * 4);
            }
        }
        Ok(Self {
            flags: get_u32(&raw, 0),
            sampler_count: get_u32(&raw, 4),
            control_stream_base: get_u64(&raw, 8),
            control_stream_end: get_u64(&raw, 0x10),
            sampler_heap: get_u64(&raw, 0x18),
            helper: helper(&raw, 0x20),
            timestamps: timestamps(&raw, 0x30),
            scs_layout: get_u64(&raw, COMPUTE_SCS_LAYOUT),
            g18_resources,
            prior_vdm: get_u64(&raw, COMPUTE_PRIOR_VDM),
            prior_cdm: get_u64(&raw, COMPUTE_PRIOR_CDM),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum ParsedHardwareCommand {
    Render {
        index: u32,
        header: UapiCommandHeader,
        payload: UapiRenderCommand,
        vertex_attachments: UapiAttachmentList,
        fragment_attachments: UapiAttachmentList,
    },
    Compute {
        index: u32,
        header: UapiCommandHeader,
        payload: UapiComputeCommand,
        attachments: UapiAttachmentList,
    },
}

pub(crate) struct UapiCommandParser<'a> {
    raw: &'a [u8],
    offset: usize,
    command_index: u32,
    hardware_count: u32,
    render_count: u16,
    compute_count: u16,
    vertex_attachments: UapiAttachmentList,
    fragment_attachments: UapiAttachmentList,
    compute_attachments: UapiAttachmentList,
}

impl<'a> UapiCommandParser<'a> {
    pub(crate) const fn new(raw: &'a [u8]) -> Self {
        Self {
            raw,
            offset: 0,
            command_index: 0,
            hardware_count: 0,
            render_count: 0,
            compute_count: 0,
            vertex_attachments: UapiAttachmentList::EMPTY,
            fragment_attachments: UapiAttachmentList::EMPTY,
            compute_attachments: UapiAttachmentList::EMPTY,
        }
    }

    pub(crate) fn finish(&self) -> Result<u32, UapiTranslateError> {
        if self.offset != self.raw.len() {
            return Err(UapiTranslateError::PayloadTruncated);
        }
        if self.hardware_count == 0 || self.hardware_count > UAPI_MAX_HARDWARE_COMMANDS {
            return Err(UapiTranslateError::HardwareCommandCount);
        }
        Ok(self.hardware_count)
    }

    pub(crate) fn next_hardware(
        &mut self,
    ) -> Result<Option<ParsedHardwareCommand>, UapiTranslateError> {
        loop {
            if self.offset == self.raw.len() {
                return Ok(None);
            }
            if self.raw.len() - self.offset < UAPI_COMMAND_HEADER_SIZE {
                return Err(UapiTranslateError::HeaderTruncated);
            }
            let header = UapiCommandHeader::parse(&self.raw[self.offset..])?;
            self.offset += UAPI_COMMAND_HEADER_SIZE;
            let end = self
                .offset
                .checked_add(header.size as usize)
                .ok_or(UapiTranslateError::PayloadTruncated)?;
            if end > self.raw.len() {
                return Err(UapiTranslateError::PayloadTruncated);
            }
            let payload = &self.raw[self.offset..end];
            self.offset = end;
            self.command_index += 1;

            match header.command_type {
                COMMAND_RENDER | COMMAND_COMPUTE => {
                    if header.render_barrier != UAPI_BARRIER_NONE
                        && header.render_barrier > self.render_count
                    {
                        return Err(UapiTranslateError::BarrierFuture);
                    }
                    if header.compute_barrier != UAPI_BARRIER_NONE
                        && header.compute_barrier > self.compute_count
                    {
                        return Err(UapiTranslateError::BarrierFuture);
                    }
                    self.hardware_count += 1;
                    if self.hardware_count > UAPI_MAX_HARDWARE_COMMANDS {
                        return Err(UapiTranslateError::HardwareCommandCount);
                    }
                    if header.command_type == COMMAND_RENDER {
                        self.render_count += 1;
                        return Ok(Some(ParsedHardwareCommand::Render {
                            index: self.command_index,
                            header,
                            payload: UapiRenderCommand::parse(payload)?,
                            vertex_attachments: self.vertex_attachments,
                            fragment_attachments: self.fragment_attachments,
                        }));
                    }
                    self.compute_count += 1;
                    return Ok(Some(ParsedHardwareCommand::Compute {
                        index: self.command_index,
                        header,
                        payload: UapiComputeCommand::parse(payload)?,
                        attachments: self.compute_attachments,
                    }));
                }
                SET_VERTEX_ATTACHMENTS | SET_FRAGMENT_ATTACHMENTS | SET_COMPUTE_ATTACHMENTS => {
                    if header.render_barrier != UAPI_BARRIER_NONE
                        || header.compute_barrier != UAPI_BARRIER_NONE
                    {
                        return Err(UapiTranslateError::AttachmentBarrier);
                    }
                    let attachments = UapiAttachmentList::parse(payload)?;
                    match header.command_type {
                        SET_VERTEX_ATTACHMENTS => self.vertex_attachments = attachments,
                        SET_FRAGMENT_ATTACHMENTS => self.fragment_attachments = attachments,
                        SET_COMPUTE_ATTACHMENTS => self.compute_attachments = attachments,
                        _ => unreachable!(),
                    }
                }
                _ => return Err(UapiTranslateError::CommandType),
            }
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum GpuAccess {
    Read,
    Write,
    ReadWrite,
}

pub(crate) trait GpuAddressSpace {
    fn covers(&self, address: u64, size: u64, access: GpuAccess) -> bool;
}

fn require_range<A: GpuAddressSpace>(
    address_space: &A,
    address: u64,
    size: u64,
    access: GpuAccess,
) -> Result<(), UapiTranslateError> {
    let end = address
        .checked_add(size)
        .ok_or(UapiTranslateError::AddressRange)?;
    if size == 0 || end <= address || !address_space.covers(address, size, access) {
        return Err(UapiTranslateError::AddressRange);
    }
    Ok(())
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct QueueUscWindow {
    pub(crate) base: u64,
    pub(crate) user_start: u64,
    pub(crate) user_end: u64,
}

impl QueueUscWindow {
    pub(crate) fn validate(self) -> Result<(), UapiTranslateError> {
        if self.base & (USC_WINDOW_SIZE - 1) != 0 {
            return Err(UapiTranslateError::QueueUscAlignment);
        }
        let end = self
            .base
            .checked_add(USC_WINDOW_SIZE)
            .ok_or(UapiTranslateError::QueueUscRange)?;
        if self.base < self.user_start || end > self.user_end || end <= self.base {
            return Err(UapiTranslateError::QueueUscRange);
        }
        Ok(())
    }

    fn full_program<A: GpuAddressSpace>(
        self,
        address_space: &A,
        tagged_offset: u32,
    ) -> Result<u64, UapiTranslateError> {
        self.validate()?;
        if tagged_offset == 0 {
            return Err(UapiTranslateError::ProgramOffset);
        }
        let allocation = self
            .base
            .checked_add((tagged_offset & !7) as u64)
            .ok_or(UapiTranslateError::ProgramOffset)?;
        require_range(address_space, allocation, 4, GpuAccess::Read)?;
        // Borealis carries resource metadata in the low three UAPI bits,
        // but G17P's native 3D command strips those bits before programming
        // every BG/EOT/partial USC address. Preserving bit 2 launched four
        // bytes into the instruction stream and contradicted all complete
        // Metal register-list captures.
        Ok(allocation)
    }
}

fn validate_sampler<A: GpuAddressSpace>(
    address_space: &A,
    heap: u64,
    count: u32,
) -> Result<(), UapiTranslateError> {
    if count > MAX_SAMPLERS {
        return Err(UapiTranslateError::SamplerCount);
    }
    if (heap == 0) != (count == 0) {
        return Err(UapiTranslateError::SamplerPair);
    }
    if count != 0 {
        if heap & (SAMPLER_SIZE - 1) != 0 {
            return Err(UapiTranslateError::AddressAlignment);
        }
        require_range(
            address_space,
            heap,
            count as u64 * SAMPLER_SIZE,
            GpuAccess::Read,
        )?;
    }
    Ok(())
}

fn validate_attachments<A: GpuAddressSpace>(
    address_space: &A,
    attachments: &UapiAttachmentList,
) -> Result<(), UapiTranslateError> {
    for attachment in attachments.as_slice() {
        require_range(
            address_space,
            attachment.address,
            attachment.size,
            GpuAccess::Write,
        )?;
    }
    Ok(())
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RenderInternalState {
    pub(crate) parameters: G17pRenderParameters,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct TranslatedRenderCommand {
    /// Exact logical render prefix assigned by the DRM ioctl dispatcher.
    pub(crate) render_order: g17_submission::G17PRenderOrder,
    /// Original UAPI render index, retained through admission.
    pub(crate) render_barrier: u16,
    /// Original UAPI compute index, retained through admission.
    pub(crate) compute_barrier: u16,
    /// Exact compute prefix resolved at ioctl admission. Later independent
    /// compute commands must not become parents of this render.
    pub(crate) compute_wait_through: Option<u64>,
    pub(crate) flags: u32,
    pub(crate) vdm_base: u64,
    pub(crate) parameters: G17pRenderParameters,
    pub(crate) vertex_attachments: UapiAttachmentList,
    pub(crate) fragment_attachments: UapiAttachmentList,
    pub(crate) vertex_timestamps: UapiTimestamps,
    pub(crate) fragment_timestamps: UapiTimestamps,
}

fn validate_zls<A: GpuAddressSpace>(
    address_space: &A,
    value: UapiZlsBuffer,
    layers: u16,
    g18_subpage: bool,
) -> Result<(), UapiTranslateError> {
    if value.base == 0
        && (value.compression_base != 0 || value.stride != 0 || value.compression_stride != 0)
    {
        return Err(UapiTranslateError::ZlsFields);
    }
    if value.compression_base == 0 && value.compression_stride != 0 {
        return Err(UapiTranslateError::ZlsFields);
    }
    if !g18_subpage && layers > 1 && value.base != 0 && value.stride == 0 {
        return Err(UapiTranslateError::ZlsFields);
    }
    if value.stride != 0 && value.stride & 0x3fff != 1
        && !(g18_subpage && value.stride & 0x3fff == 0) {
        return Err(UapiTranslateError::ZlsFields);
    }
    if value.compression_stride & 0x3fff != 0 {
        return Err(UapiTranslateError::ZlsFields);
    }
    if value.base != 0 {
        let layer_stride = if g18_subpage && layers > 1 {
            let unit = if value.stride & 1 != 0 { PAGE_SIZE } else { 0x80 };
            ((value.stride as u64 >> 14) + 1) * unit
        } else if value.stride == 0 {
            0
        } else {
            ((value.stride as u64 >> 14) + 1) * PAGE_SIZE
        };
        let size = layer_stride
            .checked_mul(layers.saturating_sub(1) as u64)
            .and_then(|value| value.checked_add(1))
            .ok_or(UapiTranslateError::AddressRange)?;
        require_range(address_space, value.base, size, GpuAccess::ReadWrite)?;
    }
    if value.compression_base != 0 {
        let layer_stride = ((value.compression_stride as u64 >> 14) + 1) * 0x80;
        let size = layer_stride
            .checked_mul(layers.saturating_sub(1) as u64)
            .and_then(|value| value.checked_add(1))
            .ok_or(UapiTranslateError::AddressRange)?;
        require_range(
            address_space,
            value.compression_base,
            size,
            GpuAccess::ReadWrite,
        )?;
    }
    Ok(())
}

/// Bounded M5 D32 clear/store envelope, alongside the original color-only
/// profile. Native 25G76 supplies ff00ff dimensions and 80044 with compression;
/// bits2/6 select compression, while bit19 stores the depth plane. Geometry,
/// stencil absence, and application programs are checked by the M5 caller.
pub(crate) fn validate_m5_reference_depth<A: GpuAddressSpace>(
    command: &UapiRenderCommand,
    address_space: &A,
) -> Result<(), UapiTranslateError> {
    let depth = command.depth;
    if depth.base == 0 {
        // No depth plane. The surface may still carry an inert stencil one,
        // which contributes nothing but its compression pair and the shared
        // pixel dimensions; a render that transfers it is refused with the
        // rest of the stencil profile.
        let stencil_only = command.stencil.base != 0
            && command.zls_control
                == if command.stencil.compression_base != 0 { 0x110 } else { 0 }
            && command.depth_dimensions
                == ((u32::from(command.width) - 1)
                    | ((u32::from(command.height) - 1) << 16));
        return if depth.compression_base == 0 && depth.stride == 0
            && depth.compression_stride == 0
            && (stencil_only
                || (command.zls_control == 0 && command.depth_dimensions == 0)) {
            Ok(())
        } else {
            Err(UapiTranslateError::ZlsFields)
        };
    }
    let compressed = depth.compression_base != 0;
    // The control word's load and store bits are the only part a render
    // chooses: bit15 loads the depth plane and bit19 stores it, and a render
    // that neither tests nor writes depth sets neither, which is how a
    // conformance surface with an unused depth buffer arrives. Everything
    // else -- the compression pair in bits 2 and 6, the GPU tiling, the 32F
    // format -- is the witnessed profile and stays exact. depth_flags reaches
    // register 0x15319 unchanged either way, so the admitted envelope is the
    // set of load/store combinations over that fixed remainder.
    let compression_bits = if compressed { 0x44 } else { 0 };
    // Bits 25-26 select the depth format: 0 is 32-bit float, 2 is 16-bit
    // unorm. The narrow plane was captured on 25G76 alongside the wide one
    // and moves nothing else in the register program except the clear value
    // and one ISP flag, both of which the caller derives. Its compression
    // metadata has no witness, so a compressed 16-bit plane stays out.
    let z16 = command.zls_control & (3 << 25) == (2 << 25);
    if z16 && compressed {
        return Err(UapiTranslateError::ZlsFields);
    }
    let format_bits = if z16 { 2 << 25 } else { 0 };
    // A stencil plane may be attached without being transferred: a surface
    // with a depth-stencil buffer carries one for every render, whether or
    // not the render uses it. Its compression pair in bits 4 and 8 describes
    // the attached layout and is the only stencil state admitted here --
    // loading or storing the plane (bits 14 and 18) needs its own witnesses,
    // as does the stencil unit in the graphics record.
    let stencil_compression_bits =
        if command.stencil.compression_base != 0 { 0x110 } else { 0 };
    let control_transfers = command.zls_control & 0x88000;
    let expected_control =
        compression_bits | stencil_compression_bits | format_bits | control_transfers;
    let width = u32::from(command.width);
    let height = u32::from(command.height);
    if !(1..=M5_MAX_RENDER_DIMENSION).contains(&width)
        || !(1..=M5_MAX_RENDER_DIMENSION).contains(&height)
        || command.layers != 1 || (compressed && (width < 16 || height < 16)) {
        return Err(UapiTranslateError::ZlsFields);
    }
    // AIL tiles are 16 KiB whatever the format: 64x64 at four bytes a pixel,
    // 128x64 at two. While both axes fit one tile the level is a whole
    // number of them; if either axis is smaller, the level uses a
    // power-of-two miptail aligned to a 128-byte cacheline. Single-layer M5
    // surfaces are not page-padded.
    let (tile_width, bytes_per_pixel) = if z16 { (128u32, 2u64) } else { (64u32, 4u64) };
    let data_bytes = if width >= tile_width && height >= 64 {
        u64::from(width.div_ceil(tile_width)) * u64::from(height.div_ceil(64)) * 0x4000
    } else {
        (u64::from(width.next_power_of_two())
            * u64::from(height.next_power_of_two()) * bytes_per_pixel)
            .div_ceil(128) * 128
    };
    let metadata_bytes = (u64::from(width.next_power_of_two().div_ceil(16))
        * u64::from(height.next_power_of_two().div_ceil(16)) * 8).div_ceil(128) * 128;
    let dimensions = (width - 1) | ((height - 1) << 16);
    if command.depth_dimensions != dimensions
        || command.zls_control != expected_control
        // The clear value is whatever the application asked for; it reaches
        // register 0x15301 verbatim and is only consumed when the render
        // actually clears the plane.
        || (command.depth_clear & 0x7fffffff) > 0x7f800000
        // Image suballocations and small miptails use cacheline alignment;
        // the backing VM pages remain independently validated below.
        || depth.base & 0x7f != 0 || depth.compression_base & 0x7f != 0
        // Both strides are array strides, and the M5 caller zeroes them
        // because native single-layer ZLS does not encode one. Every render
        // admitted here is single-layer -- layers != 1 is refused above -- so
        // the value that arrives is whatever the userspace packer computed
        // and the hardware never sees it: for a surface smaller than a page
        // Mesa's (pages - 1) << 14 | 1 underflows to 0xffffc001. Comparing it
        // would refuse a valid render over a field that is discarded. The
        // backing store is still validated in full below, from the
        // dimensions rather than from either stride.
        || (!compressed && depth.compression_stride != 0) {
        return Err(UapiTranslateError::ZlsFields);
    }
    // Validate the entire resized surface and metadata before the
    // M5 caller removes the unused single-layer array stride fields.
    require_range(address_space, depth.base, data_bytes, GpuAccess::ReadWrite)?;
    if compressed {
        require_range(address_space, depth.compression_base, metadata_bytes,
            GpuAccess::ReadWrite)?;
    }
    Ok(())
}

pub(crate) fn translate_render_command<A: GpuAddressSpace>(
    command: UapiRenderCommand,
    vertex_attachments: UapiAttachmentList,
    fragment_attachments: UapiAttachmentList,
    queue: QueueUscWindow,
    address_space: &A,
    internal: RenderInternalState,
) -> Result<TranslatedRenderCommand, UapiTranslateError> {
    translate_render_command_with_capacity(command, vertex_attachments,
        fragment_attachments, queue, address_space, internal, 32_768, false)
}

/// G18 has a separately admitted native 96-KiB resident multisample profile.
/// Its caller validates sample count, utile geometry, stride and layer count.
pub(crate) fn translate_g18_render_command<A: GpuAddressSpace>(
    command: UapiRenderCommand,
    vertex_attachments: UapiAttachmentList,
    fragment_attachments: UapiAttachmentList,
    queue: QueueUscWindow,
    address_space: &A,
    internal: RenderInternalState,
) -> Result<TranslatedRenderCommand, UapiTranslateError> {
    // Native1044's 1x RGBA32 MRT footprint reaches128 bytes per sample at 32x32
    // (128KiB); native1121 adds 40/56/72. Layered, geometry and MSAA stay separate.
    let wide_single = command.samples == 1 && command.layers == 1
        && (command.utile_width, command.utile_height) == (32, 32)
        && (((48..=128).contains(&command.sample_size) && command.sample_size & 15 == 0)
            || matches!(command.sample_size, 40 | 56 | 72));
    let capacity = if wide_single { 131_072 }
        else if matches!(command.samples, 2 | 4) { 98_304 }
        else { 32_768 };
    translate_render_command_with_capacity(command, vertex_attachments,
        fragment_attachments, queue, address_space, internal, capacity, true)
}

fn translate_render_command_with_capacity<A: GpuAddressSpace>(
    command: UapiRenderCommand,
    vertex_attachments: UapiAttachmentList,
    fragment_attachments: UapiAttachmentList,
    queue: QueueUscWindow,
    address_space: &A,
    internal: RenderInternalState,
    tile_capacity: u64,
    g18_subpage: bool,
) -> Result<TranslatedRenderCommand, UapiTranslateError> {
    if command.flags & !RENDER_FLAGS != 0 {
        return Err(UapiTranslateError::FlagsUnknownBits);
    }
    // Genuinely unimplemented on G17P rather than merely unvalidated:
    // `G17pRenderParameters` carries no helper-program fields at all, so
    // VERTEX_SCRATCH has nowhere to put the scratch binding, and there is no
    // cluster-count control for NO_VERTEX_CLUSTERING. A clean EINVAL beats
    // rendering with the request silently dropped.
    if command.flags & (RENDER_VERTEX_SCRATCH | RENDER_NO_VERTEX_CLUSTERING) != 0 {
        return Err(UapiTranslateError::FlagsUnsupported);
    }
    // RSRC_SPEC_HI is *declarative*, and honouring it must not mean rejecting.
    // The UAPI defines it as "the appended resource-specifier high dwords are
    // present", and states its purpose: "This flag lets old kernels report an
    // unsupported render payload instead of silently truncating 64-bit G17
    // values." It exists for kernels that do NOT understand the dwords.
    //
    // This kernel does understand them -- `program()` ORs the high dword in
    // unconditionally, with no reference to the flag -- so the flag cannot
    // change what we do with a payload that sets it. The old check therefore
    // refused payloads it would otherwise have handled identically, protecting
    // nothing; it is the same bring-up leftover as the stencil-clear mask.
    //
    // Honour the documented meaning instead: when the flag is clear the high
    // dwords are "not present", so mask them off rather than refuse the
    // submission. A userspace built against the 240-byte upstream struct (whose
    // payload `padded_payload` zero-fills) and one that clears the flag now
    // behave identically, which is what "not present" has to mean.
    let resource_spec_mask = if command.flags & RENDER_RSRC_SPEC_HI != 0 {
        u64::MAX
    } else {
        u32::MAX as u64
    };
    if !(1..=16_384).contains(&command.width) || !(1..=16_384).contains(&command.height) {
        return Err(UapiTranslateError::Dimensions);
    }
    if !(1..=2048).contains(&command.layers) {
        return Err(UapiTranslateError::Layers);
    }
    if !matches!(
        (command.utile_width, command.utile_height),
        (32, 32) | (32, 16) | (16, 16)
    ) {
        return Err(UapiTranslateError::UtileDimensions);
    }
    let samples_log2 = match command.samples {
        1 => 0,
        2 => 1,
        4 => 2,
        _ => return Err(UapiTranslateError::Samples),
    };
    let utile_bytes = command.sample_size as u64
        * command.utile_width as u64
        * command.utile_height as u64
        * command.samples as u64;
    if utile_bytes > tile_capacity {
        return Err(UapiTranslateError::TilebufferSize);
    }
    let blocks_per_utile = utile_bytes.div_ceil(2048);

    if command.vdm_base == 0 || command.vdm_base & 3 != 0 {
        return Err(UapiTranslateError::AddressAlignment);
    }
    require_range(address_space, command.vdm_base, 4, GpuAccess::Read)?;

    if command.scissor_base == 0 || command.scissor_base & 7 != 0 {
        return Err(UapiTranslateError::AddressAlignment);
    }
    require_range(address_space, command.scissor_base, 8, GpuAccess::Read)?;
    if command.depth_bias_base != 0 {
        if command.depth_bias_base & 7 != 0 {
            return Err(UapiTranslateError::AddressAlignment);
        }
        require_range(address_space, command.depth_bias_base, 8, GpuAccess::Read)?;
    }
    if command.occlusion_query_base != 0 {
        if command.occlusion_query_base & 7 != 0 {
            return Err(UapiTranslateError::AddressAlignment);
        }
        require_range(
            address_space,
            command.occlusion_query_base,
            8,
            GpuAccess::Write,
        )?;
    }
    validate_zls(address_space, command.depth, command.layers, g18_subpage)?;
    validate_zls(address_space, command.stencil, command.layers, g18_subpage)?;
    validate_sampler(
        address_space,
        command.sampler_heap,
        command.sampler_count as u32,
    )?;
    if command.vertex_helper
        != (UapiHelperProgram {
            binary: 0,
            config: 0,
            data: 0,
        })
        || command.fragment_helper
            != (UapiHelperProgram {
                binary: 0,
                config: 0,
                data: 0,
            })
    {
        return Err(UapiTranslateError::HelperProgram);
    }
    // `isp_bgobjvals` is the *whole* ISP_BGOBJVALS register value, not just
    // the stencil clear: the UAPI documents "the bottom 8-bits contain the
    // stencil buffer clear value", and every Mesa backend unconditionally
    // seeds the upper bits with 0x300 before OR-ing the clear in
    // (hk_cmd_draw.c `render->cr.isp_bgobjvals = 0x300;`,
    // agx_pipe.c `c->isp_bgobjvals = 0x300;`, d12_queue.c `0x300 | stencil`).
    // The legacy queue path passes the field straight through to the firmware
    // (queue/render.rs `let load_bgobjvals = cmdbuf.isp_bgobjvals as u64`), so
    // rejecting anything above 0xff refused every real render pass while the
    // in-tree render selftest -- which leaves the field zero -- was accepted.
    // Accept the register value; `build_*_descriptor` re-asserts the 0x300
    // bits with `stencil_clear_value | 0x300`, which is idempotent for the
    // value userspace sends and still supplies 0x300 when it sends zero.

    let background = queue.full_program(address_space, command.background.usc)?;
    let end_of_tile = queue.full_program(address_space, command.end_of_tile.usc)?;
    let partial_background = queue.full_program(address_space, command.partial_background.usc)?;
    let partial_end_of_tile = queue.full_program(address_space, command.partial_end_of_tile.usc)?;
    validate_attachments(address_space, &vertex_attachments)?;
    validate_attachments(address_space, &fragment_attachments)?;

    let utile_config = ((command.utile_width as u64 / 16) << 12)
        | ((command.utile_height as u64 / 16) << 14)
        | samples_log2;
    let mut tile_config = 0x280;
    if command.layers > 1 {
        tile_config |= 1;
    }
    if command.flags & RENDER_PROCESS_EMPTY_TILES != 0 {
        tile_config |= 0x1_0000;
    }

    let mut parameters = internal.parameters;
    parameters.width = command.width as u32;
    parameters.height = command.height as u32;
    parameters.encoder = command.vdm_base;
    parameters.layers = command.layers;
    parameters.utile_width = command.utile_width;
    parameters.utile_height = command.utile_height;
    parameters.utile_config = utile_config;
    parameters.multisample_control = command.multisample_control;
    parameters.ppp_control = command.ppp_control as u64;
    parameters.tib_blocks = blocks_per_utile;
    parameters.tile_config = tile_config;
    parameters.depth_dimensions = command.depth_dimensions as u64;
    parameters.depth_buffer = command.depth.base;
    parameters.depth_aux_buffer = command.depth.compression_base;
    parameters.depth_stride = command.depth.stride as u64;
    parameters.depth_aux_stride = command.depth.compression_stride as u64;
    parameters.stencil_buffer = command.stencil.base;
    parameters.stencil_aux_buffer = command.stencil.compression_base;
    parameters.stencil_stride = command.stencil.stride as u64;
    parameters.stencil_aux_stride = command.stencil.compression_stride as u64;
    parameters.depth_flags = command.zls_control;
    // The firmware's internal partial-render phases preserve attached Z/S
    // surfaces even when the render-pass boundary is store-only. Mesa already
    // computes this distinction for its partial BG/EOT programs, but the
    // current 256-byte render UAPI carries only the boundary ZLS word. Derive
    // the exact uncompressed enable bits here until the append-only partial
    // ZLS UAPI field is plumbed; tiling/compression/format bits remain those
    // packed into the original word.
    parameters.partial_depth_flags = command.zls_control
        | if command.depth.base != 0 { (1 << 15) | (1 << 19) } else { 0 }
        | if command.stencil.base != 0 { (1 << 14) | (1 << 18) } else { 0 };
    parameters.occlusion_query_base = command.occlusion_query_base;
    parameters.scissor_array = command.scissor_base;
    parameters.depth_bias_array = command.depth_bias_base;
    parameters.sampler_array = command.sampler_heap;
    parameters.sampler_count = command.sampler_count as u32;
    parameters.process_empty_tiles = command.flags & RENDER_PROCESS_EMPTY_TILES != 0;
    parameters.merge_upper_x_bits = command.merge_upper_x;
    parameters.merge_upper_y_bits = command.merge_upper_y;
    parameters.depth_clear_value_bits = command.depth_clear;
    parameters.stencil_clear_value = command.stencil_clear;
    parameters.load_pipeline_bind = g17p_bg_resource_spec(
        command.background.resource_spec,
        resource_spec_mask,
        internal.parameters.bg_resource_prefix,
    );
    parameters.load_pipeline = background;
    parameters.store_pipeline_bind = command.end_of_tile.resource_spec & resource_spec_mask;
    parameters.store_pipeline = end_of_tile;
    // These high 17 bits are BG shader pass control, not an address class.
    // Native derives main and partial masks from their respective shader
    // outputs; they need not match. The default diagnostic prefix is zero,
    // preserving both full RT0 output masks and no-output masks from userspace.
    parameters.partial_load_pipeline_bind = g17p_bg_resource_spec(
        command.partial_background.resource_spec,
        resource_spec_mask,
        internal.parameters.bg_resource_prefix,
    );
    parameters.partial_load_pipeline = partial_background;
    parameters.partial_store_pipeline_bind =
        command.partial_end_of_tile.resource_spec & resource_spec_mask;
    parameters.partial_store_pipeline = partial_end_of_tile;

    Ok(TranslatedRenderCommand {
        render_order: g17_submission::G17PRenderOrder::default(),
        render_barrier: UAPI_BARRIER_NONE,
        compute_barrier: UAPI_BARRIER_NONE,
        compute_wait_through: None,
        flags: command.flags,
        vdm_base: command.vdm_base,
        parameters,
        vertex_attachments,
        fragment_attachments,
        vertex_timestamps: command.vertex_timestamps,
        fragment_timestamps: command.fragment_timestamps,
    })
}

fn g17p_bg_resource_spec(resource_spec: u64, present_mask: u64, diagnostic_prefix: u64) -> u64 {
    diagnostic_prefix | (resource_spec & present_mask)
}

pub(crate) fn build_render_descriptors(
    command: &TranslatedRenderCommand,
    objects: RenderDescriptorObjects,
    tiling_metadata: RenderDescriptorMetadata,
    fragment_metadata: RenderDescriptorMetadata,
    tiling: &mut [u8],
    fragment: &mut [u8],
) -> Result<(), UapiTranslateError> {
    g17_render::build_ta_descriptor(&command.parameters, objects, tiling_metadata, tiling)?;
    g17_render::build_fragment_descriptor(
        &command.parameters,
        objects,
        fragment_metadata,
        fragment,
    )?;
    Ok(())
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct TranslatedComputeCommand {
    /// Monotonic logical index, independent of the hardware timestamp.
    pub(crate) compute_sequence: u64,
    /// Explicit same-engine prefix; only host-published timestamps need host waits.
    pub(crate) compute_wait_through: Option<u64>,
    pub(crate) render_wait_through: Option<u64>,
    pub(crate) render_wait_last: Option<u64>,
    pub(crate) usc_exec_base: u64,
    pub(crate) control_stream_base: u64,
    pub(crate) control_stream_end: u64,
    pub(crate) control_stream_terminator: u64,
    pub(crate) sampler_heap: u64,
    pub(crate) sampler_count: u32,
    pub(crate) attachments: UapiAttachmentList,
    pub(crate) g17p_add3_buffers: Option<[u64; 3]>,
    pub(crate) timestamps: UapiTimestamps,
    pub(crate) scs_layout: u64,
    /// G18 only: the command carried `DRM_ASAHI_COMPUTE_USES_TEXTURE_STATE`.
    /// [`translate_compute_command`] refuses that bit like any other unknown
    /// one and leaves this false; the G18 adapter admits the bit, strips it
    /// before translation and sets this afterwards.
    pub(crate) g18_texture_state: bool,
    /// Validated native descriptor inputs; resource/launch integration is
    /// separate, so this field does not by itself authorize scratch use.
    pub(crate) g18_resource_descriptor: Option<[u32; 23]>,
}

pub(crate) fn translate_compute_command<A: GpuAddressSpace>(
    command: UapiComputeCommand,
    attachments: UapiAttachmentList,
    queue: QueueUscWindow,
    address_space: &A,
) -> Result<TranslatedComputeCommand, UapiTranslateError> {
    queue.validate()?;
    if command.flags & !COMPUTE_G17P_ADD3_PROOF != 0 {
        return Err(UapiTranslateError::FlagsUnknownBits);
    }
    if command.g18_resources != [0; 16] {
        return Err(UapiTranslateError::FlagsUnsupported);
    }
    if command.control_stream_base & 3 != 0
        || command.control_stream_end & 3 != 0
        || command.control_stream_end <= command.control_stream_base
    {
        return Err(UapiTranslateError::AddressAlignment);
    }
    require_range(
        address_space,
        command.control_stream_base,
        command.control_stream_end - command.control_stream_base,
        GpuAccess::Read,
    )?;
    validate_sampler(address_space, command.sampler_heap, command.sampler_count)?;
    if command.helper
        != (UapiHelperProgram {
            binary: 0,
            config: 0,
            data: 0,
        })
    {
        return Err(UapiTranslateError::HelperProgram);
    }
    let g17p_add3_buffers = if command.flags & COMPUTE_G17P_ADD3_PROOF != 0 {
        let entries = attachments.as_slice();
        if entries.len() != 3
            || entries
                .iter()
                .any(|entry| entry.size < G17P_ADD3_BUFFER_BYTES)
        {
            return Err(UapiTranslateError::Add3ProofAttachments);
        }
        require_range(
            address_space,
            entries[0].address,
            G17P_ADD3_BUFFER_BYTES,
            GpuAccess::Read,
        )?;
        require_range(
            address_space,
            entries[1].address,
            G17P_ADD3_BUFFER_BYTES,
            GpuAccess::Read,
        )?;
        require_range(
            address_space,
            entries[2].address,
            G17P_ADD3_BUFFER_BYTES,
            GpuAccess::Write,
        )?;
        Some([entries[0].address, entries[1].address, entries[2].address])
    } else {
        validate_attachments(address_space, &attachments)?;
        None
    };
    Ok(TranslatedComputeCommand {
        compute_sequence: 0,
        compute_wait_through: None,
        render_wait_through: None,
        render_wait_last: None,
        usc_exec_base: queue.base,
        control_stream_base: command.control_stream_base,
        control_stream_end: command.control_stream_end,
        control_stream_terminator: command.control_stream_end - 4,
        sampler_heap: command.sampler_heap,
        sampler_count: command.sampler_count,
        attachments,
        g17p_add3_buffers,
        timestamps: command.timestamps,
        scs_layout: command.scs_layout,
        g18_texture_state: false,
        g18_resource_descriptor: None,
    })
}

/// Queue-owned inputs that are independent of one userspace compute command.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ComputeInternalState {
    pub(crate) preempt_base: u64,
    pub(crate) dispatch_identity: u64,
    pub(crate) context_id: u32,
    pub(crate) work_ordinal: u32,
    pub(crate) robustness: u64,
    pub(crate) operand_state_base: u64,
    pub(crate) usc_flist_id: u32,
}

/// GPU addresses resolved from the UAPI timestamp handles while the object
/// table is locked.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ComputeTimestampAddresses {
    pub(crate) start: u64,
    pub(crate) end: u64,
}

/// A compute command owns exactly one start/end pair. Both words must be
/// visible before the queue entry and its userspace fence can retire.
pub(crate) const fn completed_compute_timestamps(start: u64, end: u64) -> Option<[u64; 2]> {
    if start == 0 || end == 0 {
        None
    } else {
        Some([start, end])
    }
}

/// Derive the descriptor/RCE checkpoint quartet from the exact command's
/// register program without publishing either object.
pub(crate) fn compute_resume_checkpoints(
    command: &TranslatedComputeCommand,
    internal: ComputeInternalState,
) -> Result<[u16; 4], UapiTranslateError> {
    let registers = g17_compute::build_compute_registers_for_layout(
        g17_compute::ComputeRegisterParameters {
            preempt_base: internal.preempt_base,
            cdm_base: command.control_stream_base,
            usc_exec_base: command.usc_exec_base,
            helper_binary: 0,
            helper_data: 0,
            helper_config: 0,
            dispatch_identity: internal.dispatch_identity,
            context_id: internal.context_id,
            work_ordinal: internal.work_ordinal,
            robustness: internal.robustness,
            operand_state_base: internal.operand_state_base,
            usc_flist_id: internal.usc_flist_id,
        },
        command.scs_layout,
    )?;
    Ok(g17_compute::compute_resume_checkpoints(registers.as_slice())?)
}

/// Build the complete selector-3 descriptor for a translated command.
///
/// This is the boundary that consumes the command's CDM range, sampler state,
/// USC window, and timestamp destinations. The translated attachment list
/// remains in `command` for the queue's reservation and lifetime tracking.
pub(crate) fn build_compute_descriptor(
    command: &TranslatedComputeCommand,
    internal: ComputeInternalState,
    objects: g17_compute::ComputeDescriptorObjects,
    metadata: g17_compute::ComputeDescriptorMetadata,
    timestamps: ComputeTimestampAddresses,
    raw: &mut [u8],
) -> Result<[u16; 4], UapiTranslateError> {
    let registers = g17_compute::build_compute_registers_for_layout(
        g17_compute::ComputeRegisterParameters {
            preempt_base: internal.preempt_base,
            cdm_base: command.control_stream_base,
            usc_exec_base: command.usc_exec_base,
            helper_binary: 0,
            helper_data: 0,
            helper_config: 0,
            dispatch_identity: internal.dispatch_identity,
            context_id: internal.context_id,
            work_ordinal: internal.work_ordinal,
            robustness: internal.robustness,
            operand_state_base: internal.operand_state_base,
            usc_flist_id: internal.usc_flist_id,
        },
        command.scs_layout,
    )?;
    let resume_checkpoints = g17_compute::build_compute_descriptor(
        &registers,
        objects,
        metadata,
        g17_compute::ComputeDescriptorCommand {
            cdm_terminator: command.control_stream_terminator,
            sampler_array: command.sampler_heap,
            sampler_count: command.sampler_count,
            user_timestamp_start: timestamps.start,
            user_timestamp_end: timestamps.end,
        },
        raw,
    )?;
    Ok(resume_checkpoints)
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct ComputeEntryContext<'a> {
    pub(crate) descriptor_flag_4c: bool,
    pub(crate) converted_command_timestamp: u64,
    pub(crate) barriers: &'a [G17PClBarrierDependency],
    pub(crate) mcache: Option<G17PClMcacheAperture>,
    pub(crate) payload: [u64; 2],
    pub(crate) event_mask: [u64; 4],
    pub(crate) rce_kind: u8,
    pub(crate) rce_bindings: [G17PClRceBinding; 4],
    pub(crate) auxiliary: u64,
}

impl TranslatedComputeCommand {
    /// Native `processComputeSetup` writes descriptor +0x5c8 from whether the
    /// derived SCS page count is nonzero. `scs_pages_for_layout` has already
    /// validated the same layout before an entry can be published; its native
    /// zero-page gate is exactly both payload fields being zero.
    pub(crate) const fn scs_enabled(&self) -> bool {
        self.scs_layout >> 32 != 0
    }

    pub(crate) const fn entry_operands<'a>(
        &self,
        context: ComputeEntryContext<'a>,
    ) -> G17PClKickEntryOperands<'a> {
        G17PClKickEntryOperands {
            descriptor_flag_4c: context.descriptor_flag_4c,
            // `generateCLRegisterListG16Common` uses the same +0x5c8 bit to
            // prepend 0x17d9, and encodeCLKickEntry copies it to header bit61.
            // Derive both from the command so they cannot diverge.
            descriptor_flag_5c8: self.scs_enabled(),
            converted_command_timestamp: context.converted_command_timestamp,
            barriers: context.barriers,
            mcache: context.mcache,
            payload: context.payload,
            event_mask: context.event_mask,
            rce_kind: context.rce_kind,
            rce_bindings: context.rce_bindings,
            auxiliary: context.auxiliary,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn nibble(value: u8) -> u8 {
        match value {
            b'0'..=b'9' => value - b'0',
            b'a'..=b'f' => value - b'a' + 10,
            _ => 0xff,
        }
    }

    const fn hex<const N: usize, const M: usize>(source: &[u8; M]) -> [u8; N] {
        let mut out = [0u8; N];
        let mut index = 0;
        while index < N {
            out[index] = (nibble(source[index * 2]) << 4) | nibble(source[index * 2 + 1]);
            index += 1;
        }
        out
    }

    const RENDER_FIXTURE: [u8; 312] = hex::<312, 624>(b"02001800ffffffff00000a00000000000001000000000000000000000000000003001800ffffffff00000b0000000000000200000000000000000000000000000000f000ffffffff020000000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000088000000000000000000000000000000020200004000300001000000202001040000000000000000040001004000000004000200080000000400010000000000040002000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
    const COMPUTE_FIXTURE: [u8; 104] = hex::<104, 208>(b"04001800ffffffff00000300000000000001000000000000000000000000000001004000ffffffff00000000000000000000010000000000400001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");

    fn fnv1a64(raw: &[u8]) -> u64 {
        let mut value = 0xcbf2_9ce4_8422_2325u64;
        for byte in raw {
            value ^= *byte as u64;
            value = value.wrapping_mul(0x0000_0100_0000_01b3);
        }
        value
    }

    struct TestAddressSpace;

    impl GpuAddressSpace for TestAddressSpace {
        fn covers(&self, address: u64, size: u64, _access: GpuAccess) -> bool {
            let end = match address.checked_add(size) {
                Some(value) => value,
                None => return false,
            };
            (0x1_0000..=0x20_0000).contains(&address) && end <= 0x20_0000
                || (0x11_0000_0000..0x12_0000_0000).contains(&address) && end <= 0x12_0000_0000
        }
    }

    fn window() -> QueueUscWindow {
        QueueUscWindow {
            base: 0x11_0000_0000,
            user_start: 0x4000,
            user_end: (1 << 39) - 0x8000,
        }
    }

    fn objects() -> RenderDescriptorObjects {
        RenderDescriptorObjects {
            record_a: 0xffff_fc20_0150_0000,
            shared: 0xffff_fc20_0160_0000,
            record_b: 0xffff_fc20_0158_0000,
            zero: 0xffff_fc20_0168_0000,
        }
    }

    #[test]
    fn m5_d32_envelope_rejects_wrong_layout_and_incomplete_backing() {
        let mut parser = UapiCommandParser::new(&RENDER_FIXTURE);
        let ParsedHardwareCommand::Render { mut payload, .. } =
            parser.next_hardware().unwrap().unwrap() else { panic!("not render") };
        payload.width = 256;
        payload.height = 256;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        payload.depth_dimensions = 0x00ff00ff;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload.depth.base = 0x10000;
        payload.depth.stride = 0x3c001;
        payload.zls_control = 0x80000;
        payload.depth_clear = 0x3f800000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        payload.depth.compression_base = 0x60000;
        payload.zls_control = 0x80044;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        let valid = payload;
        // The real gears command formerly used G17P's 15-bit dimensions.
        payload.depth_dimensions = 0x007f80ff;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload = valid;
        payload.depth.compression_base = 0;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload = valid;
        payload.depth.base = 0x1fc000; // First byte mapped, remainder absent.
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::AddressRange));
        payload = valid;
        payload.depth.compression_base = 0x200000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::AddressRange));
        payload = valid;
        // Every combination of the load and store bits is a render the
        // application may ask for, including neither: a bound depth buffer
        // that this render does not test is not written back.
        for transfers in [0u64, 0x8000, 0x80000, 0x88000] {
            payload.zls_control = 0x44 | transfers;
            assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
                Ok(()));
        }
        // Nothing outside those two bits may move: stencil transfers, the
        // twiddled tilings, the 16-bit format and the resolve bits all still
        // need their own witnesses.
        for stray in [0x2u64, 0x4000, 0x40000, 1 << 25, 1 << 56] {
            payload.zls_control = 0x80044 | stray;
            assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
                Err(UapiTranslateError::ZlsFields));
        }
        payload = valid;
        payload.layers = 2;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload = valid;
        // The clear value is the application's float and reaches register
        // 0x15301 unchanged; only a NaN payload is refused.
        for clear in [0u32, 0x3f000000, 0xbf800000, 0x7f800000] {
            payload.depth_clear = clear;
            assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
                Ok(()));
        }
        payload.depth_clear = 0x7fc00000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        // An attached stencil plane contributes only its compression pair.
        payload = valid;
        payload.stencil.base = 0x10000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        payload.stencil.compression_base = 0x60000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload.zls_control = 0x80154;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        // Loading or storing it is outside the profile.
        for transfer in [0x4000u64, 0x40000] {
            payload.zls_control = 0x80154 | transfer;
            assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
                Err(UapiTranslateError::ZlsFields));
        }
    }

    #[test]
    fn m5_resized_depth_validates_tile_padding_and_complete_backing() {
        let mut parser = UapiCommandParser::new(&RENDER_FIXTURE);
        let ParsedHardwareCommand::Render { mut payload, .. } =
            parser.next_hardware().unwrap().unwrap() else { panic!("not render") };
        payload.width = 319;
        payload.height = 191;
        payload.depth_dimensions = 318 | (190 << 16);
        payload.depth.base = 0x10000;
        payload.depth.stride = 0x38001; // 5x3 pages, not width*height*4.
        payload.depth.compression_base = 0x60000;
        payload.depth.compression_stride = 0x7c000; // 512x256 / (16x16) *8.
        payload.zls_control = 0x80044;
        payload.depth_clear = 0x3f800000;
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        let valid = payload;
        // The array strides are discarded for a single-layer render, so any
        // value in them is admitted -- including the underflow a surface
        // smaller than one page produces. The backing store below is what
        // actually bounds the render.
        for stride in [0u32, 0x3c001, 0xffffc001] {
            payload.depth.stride = stride;
            assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
                Ok(()));
        }
        payload = valid;
        payload.depth.compression_stride = 0x3c000; // Old metadata stride.
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace), Ok(()));
        payload = valid;
        payload.depth.compression_base = 0; // Metadata stride without metadata.
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::ZlsFields));
        payload = valid;
        payload.depth.base = 0x1c8000; // Only 14 pages remain, but 15 are needed.
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::AddressRange));
        payload = valid;
        payload.width = 1024;
        payload.height = 1024;
        payload.depth_dimensions = 1023 | (1023 << 16);
        assert_eq!(validate_m5_reference_depth(&payload, &TestAddressSpace),
            Err(UapiTranslateError::AddressRange)); // Four MiB exceeds this test VM.
    }

    #[test]
    fn render_fixture_translates_to_full_g17p_programs_and_descriptors() {
        assert_eq!(fnv1a64(&RENDER_FIXTURE), 0xf056_ea1d_ef06_32c5);
        let mut parser = UapiCommandParser::new(&RENDER_FIXTURE);
        let command = parser.next_hardware().unwrap().unwrap();
        let (payload, vertex, fragment) = match command {
            ParsedHardwareCommand::Render {
                index,
                payload,
                vertex_attachments,
                fragment_attachments,
                ..
            } => {
                assert_eq!(index, 3);
                (payload, vertex_attachments, fragment_attachments)
            }
            _ => panic!("wrong command kind"),
        };
        assert!(parser.next_hardware().unwrap().is_none());
        assert_eq!(parser.finish(), Ok(1));

        let mut internal = G17pRenderParameters::source_rsrc8();
        internal.context_base = 0;
        // This frozen descriptor fixture predates userspace high-dword masks;
        // retain its explicit legacy diagnostic mask, not a default injection.
        internal.bg_resource_prefix = 0x0007_8000_0000_0000;
        let translated = translate_render_command(
            payload,
            vertex,
            fragment,
            window(),
            &TestAddressSpace,
            RenderInternalState {
                parameters: internal,
            },
        )
        .unwrap();
        assert_eq!(translated.parameters.load_pipeline, 0x11_0001_0000);
        assert_eq!(translated.parameters.store_pipeline, 0x11_0002_0000);
        assert_eq!(
            translated.parameters.load_pipeline_bind,
            0x0007_8000_0000_0040
        );
        assert_eq!(translated.parameters.store_pipeline_bind, 0x8);
        assert_eq!(translated.parameters.partial_load_pipeline, 0x11_0001_0000);
        assert_eq!(translated.parameters.partial_store_pipeline, 0x11_0002_0000);
        assert_eq!(translated.parameters.utile_config, 0xa000);
        assert_eq!(translated.parameters.tib_blocks, 2);
        assert_eq!(translated.parameters.tile_config, 0x1_0280);
        assert_eq!(translated.parameters.encoder, 0x1_0000);
        assert_eq!(
            translated.vertex_attachments.as_slice(),
            &[UapiAttachment {
                address: 0xa_0000,
                size: 0x100,
            }]
        );
        assert_eq!(
            translated.fragment_attachments.as_slice(),
            &[UapiAttachment {
                address: 0xb_0000,
                size: 0x200,
            }]
        );

        let mut tiling = [0u8; g17_render::TA_DESCRIPTOR_SIZE];
        let mut fragment_raw = [0u8; g17_render::FRAGMENT_DESCRIPTOR_SIZE];
        build_render_descriptors(
            &translated,
            objects(),
            RenderDescriptorMetadata::source(g17_render::RenderDescriptorKind::Tiling),
            RenderDescriptorMetadata::source(g17_render::RenderDescriptorKind::Fragment),
            &mut tiling,
            &mut fragment_raw,
        )
        .unwrap();
        assert_eq!(fnv1a64(&tiling), 0x8581_5830_2d5b_e227);
        assert_eq!(fnv1a64(&fragment_raw), 0x6688_e9f8_d061_d776);
    }

    #[test]
    fn background_pass_masks_preserve_userspace_and_explicit_diagnostics() {
        let default_prefix = G17pRenderParameters::source_rsrc8().bg_resource_prefix;
        assert_eq!(default_prefix, 0);
        assert_eq!(g17p_bg_resource_spec(0xffff_8000_0000_0040, u64::MAX, default_prefix),
                   0xffff_8000_0000_0040);
        assert_eq!(g17p_bg_resource_spec(0x40, u64::MAX, default_prefix), 0x40);
        assert_eq!(g17p_bg_resource_spec(0xffff_8000_0000_0040, u32::MAX as u64, default_prefix),
                   0x40);
        assert_eq!(g17p_bg_resource_spec(0x40, u64::MAX, 0xffff_8000_0000_0000),
                   0xffff_8000_0000_0040);
    }

    #[test]
    fn partial_zls_preserves_depth_and_stencil_across_internal_splits() {
        let mut parser = UapiCommandParser::new(&RENDER_FIXTURE);
        let (mut payload, vertex, fragment) = match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Render {
                payload,
                vertex_attachments,
                fragment_attachments,
                ..
            } => (payload, vertex_attachments, fragment_attachments),
            _ => panic!("wrong command kind"),
        };
        payload.depth.base = 0x1_8000;
        payload.stencil.base = 0x1_c000;
        payload.zls_control = 0x000c_0000;

        let translated = translate_render_command(
            payload,
            vertex,
            fragment,
            window(),
            &TestAddressSpace,
            RenderInternalState {
                parameters: G17pRenderParameters::source_rsrc8(),
            },
        )
        .unwrap();
        assert_eq!(translated.parameters.depth_flags, 0x000c_0000);
        assert_eq!(translated.parameters.partial_depth_flags, 0x000c_c000);
    }

    #[test]
    fn render_resource_spec_high_dwords_reach_g17_register_programs() {
        const RENDER_HEADER: usize = 64;
        const PAYLOAD: usize = RENDER_HEADER + UAPI_COMMAND_HEADER_SIZE;
        let mut extended = [0u8; RENDER_FIXTURE.len() + 16];
        extended[..RENDER_FIXTURE.len()].copy_from_slice(&RENDER_FIXTURE);
        extended[RENDER_HEADER + 2..RENDER_HEADER + 4]
            .copy_from_slice(&(UAPI_RENDER_SIZE as u16).to_le_bytes());
        extended[PAYLOAD..PAYLOAD + 4]
            .copy_from_slice(&(RENDER_PROCESS_EMPTY_TILES | RENDER_RSRC_SPEC_HI).to_le_bytes());

        for (offset, value) in [(0xac, 0x42), (0xb4, 0x40), (0xbc, 0x43), (0xc4, 0x41)] {
            extended[PAYLOAD + offset..PAYLOAD + offset + 4]
                .copy_from_slice(&(value as u32).to_le_bytes());
        }
        for (offset, value) in [
            (0xf0, 0x0007_8000),
            (0xf4, 0x0007_8001),
            (0xf8, 0xffff_8002),
            (0xfc, 0),
        ] {
            extended[PAYLOAD + offset..PAYLOAD + offset + 4]
                .copy_from_slice(&(value as u32).to_le_bytes());
        }

        let mut parser = UapiCommandParser::new(&extended);
        let (payload, vertex, fragment) = match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Render {
                payload,
                vertex_attachments,
                fragment_attachments,
                ..
            } => (payload, vertex_attachments, fragment_attachments),
            _ => panic!("wrong command kind"),
        };
        assert_eq!(parser.finish(), Ok(1));
        let translated = translate_render_command(
            payload,
            vertex,
            fragment,
            window(),
            &TestAddressSpace,
            RenderInternalState {
                parameters: G17pRenderParameters::source_rsrc8(),
            },
        )
        .unwrap();
        assert_eq!(
            translated.parameters.store_pipeline_bind,
            0x0007_8000_0000_0040
        );
        assert_eq!(
            translated.parameters.partial_store_pipeline_bind,
            0x0007_8001_0000_0041
        );
        assert_eq!(
            translated.parameters.load_pipeline_bind,
            0xffff_8002_0000_0042
        );
        assert_eq!(
            translated.parameters.partial_load_pipeline_bind,
            0x43
        );
        let registers = g17_render::build_fragment_registers(&translated.parameters).unwrap();
        assert_eq!(
            registers
                .iter()
                .find(|register| register.number == 0x15369)
                .unwrap()
                .value,
            0xffff_8002_0000_0042
        );

        // Clearing RSRC_SPEC_HI means "the high dwords are not present", so
        // they are masked off -- not grounds to refuse the submission.
        let mut no_flag = payload;
        no_flag.flags &= !RENDER_RSRC_SPEC_HI;
        let masked = translate_render_command(
            no_flag,
            vertex,
            fragment,
            window(),
            &TestAddressSpace,
            RenderInternalState {
                parameters: G17pRenderParameters::source_rsrc8(),
            },
        )
        .unwrap();
        assert_eq!(masked.parameters.load_pipeline_bind, 0x42);
        assert_eq!(
            masked.parameters.partial_load_pipeline_bind,
            0x43
        );

        // An explicitly selected diagnostic OR-mask still applies; it is no
        // longer silently imposed on ordinary no-output background programs.
        let mut diagnostic = G17pRenderParameters::source_rsrc8();
        diagnostic.bg_resource_prefix = 0xffff_8000_0000_0000;
        let overridden = translate_render_command(
            no_flag, vertex, fragment, window(), &TestAddressSpace,
            RenderInternalState { parameters: diagnostic },
        )
        .unwrap();
        assert_eq!(overridden.parameters.load_pipeline_bind, 0xffff_8000_0000_0042);
        assert_eq!(
            overridden.parameters.partial_load_pipeline_bind,
            0xffff_8000_0000_0043
        );
    }

    #[test]
    fn compute_fixture_translates_to_existing_cl_operands() {
        assert_eq!(fnv1a64(&COMPUTE_FIXTURE), 0xeb42_2636_cc6a_c274);
        let mut parser = UapiCommandParser::new(&COMPUTE_FIXTURE);
        let (payload, attachments) = match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Compute {
                index,
                payload,
                attachments,
                ..
            } => {
                assert_eq!(index, 2);
                (payload, attachments)
            }
            _ => panic!("wrong command kind"),
        };
        assert!(parser.next_hardware().unwrap().is_none());
        assert_eq!(parser.finish(), Ok(1));
        let translated =
            translate_compute_command(payload, attachments, window(), &TestAddressSpace).unwrap();
        assert_eq!(translated.control_stream_base, 0x1_0000);
        assert_eq!(translated.control_stream_end, 0x1_0040);
        assert_eq!(translated.control_stream_terminator, 0x1_003c);
        assert_eq!(translated.usc_exec_base, 0x11_0000_0000);
        assert_eq!(translated.g17p_add3_buffers, None);
        assert_eq!(
            translated.attachments.as_slice(),
            &[UapiAttachment {
                address: 0x3_0000,
                size: 0x100,
            }]
        );

        let empty = G17PClRceBinding { address: 0, tag: 0 };
        let operands = translated.entry_operands(ComputeEntryContext {
            descriptor_flag_4c: false,
            converted_command_timestamp: 1,
            barriers: &[],
            mcache: None,
            payload: [0xffff_fc20_c001_a000, 0xffff_fc20_c000_0480],
            event_mask: [1, 2, 3, 4],
            rce_kind: 0,
            rce_bindings: [empty; 4],
            auxiliary: 0x003f_ffff_ffff_ffff,
        });
        assert_eq!(
            operands.payload,
            [0xffff_fc20_c001_a000, 0xffff_fc20_c000_0480]
        );
        assert_eq!(operands.event_mask, [1, 2, 3, 4]);
        assert_eq!(operands.auxiliary, 0x003f_ffff_ffff_ffff);
        assert!(!operands.descriptor_flag_5c8);

        let mut scs_translated = translated;
        scs_translated.scs_layout = 0x48u64 << 48;
        let scs_operands = scs_translated.entry_operands(ComputeEntryContext {
            descriptor_flag_4c: false,
            converted_command_timestamp: 1,
            barriers: &[],
            mcache: None,
            payload: [0xffff_fc20_c001_a000, 0xffff_fc20_c000_0480],
            event_mask: [1, 2, 3, 4],
            rce_kind: 0,
            rce_bindings: [empty; 4],
            auxiliary: 0x003f_ffff_ffff_ffff,
        });
        assert!(scs_operands.descriptor_flag_5c8);
        let entry = g17_submission::prepare_g17p_cl_kick_entry(
            g17_submission::G17PClKickProducerState {
                queue_id: 4,
                num_stamps: 4,
                entry_stride: 0x200,
                completion_selector: 2,
                qos: g17_submission::G17PClQosConfig {
                    queue_byte_32: 0,
                    class_40: 0,
                    word_48: 0,
                },
                submitted: 0,
                current_timestamp: 0,
                last_add_kicks_timestamp: 0,
                base_timestamp: 0,
                previous_valid: false,
                previous_timestamp: 0,
            },
            operands,
        )
        .unwrap();
        assert_eq!(fnv1a64(&entry.entry), 0xbcc4_342d_108e_f9d0);

        let mut descriptor = [0u8; g17_compute::COMPUTE_DESCRIPTOR_SIZE];
        build_compute_descriptor(
            &translated,
            ComputeInternalState {
                preempt_base: 0x100_0404_0000,
                dispatch_identity: 0x1234_5678_0000_0001,
                context_id: 3,
                work_ordinal: 7,
                robustness: 0x100_0430_0000,
                operand_state_base: 0x70_0022_8000,
                usc_flist_id: 2,
            },
            g17_compute::ComputeDescriptorObjects {
                scheduler_record: 0xffff_fc20_c099_8100,
                descriptor_low_alias: 0x70_0034_0000,
                dispatch_a: 0xffff_fc20_001c_8028,
                dispatch_b: 0xffff_fc20_c07c_0028,
                status_a: 0xffff_fc20_0002_4c68,
                status_b: 0xffff_fc20_0002_4c70,
                shared_control: 0xffff_fc20_c095_0000,
                usage_record: 0xffff_fc20_0171_0000,
            },
            g17_compute::ComputeDescriptorMetadata {
                submit_sequence: 5,
                context_id: 3,
                grid_index: 2,
                work_ordinal: 7,
                queue_submission: 8,
                queue_ordinal: 7,
                submission_index: 9,
                support_control: 0xe0a0_0001,
                support_flags: 0,
            },
            ComputeTimestampAddresses {
                start: 0x200_0000_4000,
                end: 0x200_0000_4008,
            },
            &mut descriptor,
        )
        .unwrap();
        assert_eq!(
            u64::from_le_bytes(descriptor[0x0ee0..0x0ee8].try_into().unwrap()),
            translated.control_stream_terminator
        );
        assert_eq!(
            u64::from_le_bytes(descriptor[0x0f8c..0x0f94].try_into().unwrap()),
            0x200_0000_4000
        );
        assert_eq!(
            u64::from_le_bytes(descriptor[0x8c..0x94].try_into().unwrap()),
            translated.usc_exec_base
        );
        assert_eq!(completed_compute_timestamps(0, 9), None);
        assert_eq!(completed_compute_timestamps(7, 0), None);
        assert_eq!(completed_compute_timestamps(7, 9), Some([7, 9]));
    }

    #[test]
    fn add3_proof_maps_exactly_three_buffers_into_the_retained_table() {
        let mut parser = UapiCommandParser::new(&COMPUTE_FIXTURE);
        let (mut payload, _) = match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Compute {
                payload,
                attachments,
                ..
            } => (payload, attachments),
            _ => panic!("wrong command kind"),
        };
        payload.flags = COMPUTE_G17P_ADD3_PROOF;
        let mut attachments = UapiAttachmentList::EMPTY;
        attachments.entries[..3].copy_from_slice(&[
            UapiAttachment {
                address: 0x3_0000,
                size: G17P_ADD3_BUFFER_BYTES,
            },
            UapiAttachment {
                address: 0x3_8000,
                size: G17P_ADD3_BUFFER_BYTES,
            },
            UapiAttachment {
                address: 0x4_0000,
                size: G17P_ADD3_BUFFER_BYTES,
            },
        ]);
        attachments.count = 3;

        let translated =
            translate_compute_command(payload, attachments, window(), &TestAddressSpace).unwrap();
        assert_eq!(
            translated.g17p_add3_buffers,
            Some([0x3_0000, 0x3_8000, 0x4_0000])
        );

        let mut short = attachments;
        short.entries[2].size = G17P_ADD3_BUFFER_BYTES - 1;
        assert_eq!(
            translate_compute_command(payload, short, window(), &TestAddressSpace),
            Err(UapiTranslateError::Add3ProofAttachments)
        );
    }

    #[test]
    fn parser_zero_extends_old_payloads_and_checks_new_tail_bytes() {
        let mut command = [0u8; UAPI_COMMAND_HEADER_SIZE + 8];
        command[0..2].copy_from_slice(&COMMAND_COMPUTE.to_le_bytes());
        command[2..4].copy_from_slice(&8u16.to_le_bytes());
        command[4..6].copy_from_slice(&UAPI_BARRIER_NONE.to_le_bytes());
        command[6..8].copy_from_slice(&UAPI_BARRIER_NONE.to_le_bytes());
        let mut parser = UapiCommandParser::new(&command);
        match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Compute { payload, .. } => {
                assert_eq!(
                    payload,
                    UapiComputeCommand {
                        flags: 0,
                        sampler_count: 0,
                        control_stream_base: 0,
                        control_stream_end: 0,
                        sampler_heap: 0,
                        helper: UapiHelperProgram {
                            binary: 0,
                            config: 0,
                            data: 0,
                        },
                        timestamps: UapiTimestamps {
                            start: UapiTimestamp {
                                handle: 0,
                                offset: 0,
                            },
                            end: UapiTimestamp {
                                handle: 0,
                                offset: 0,
                            },
                        },
                        scs_layout: 0,
                        g18_resources: [0; 16],
                        prior_vdm: 0,
                        prior_cdm: 0,
                    }
                );
            }
            _ => panic!("wrong command kind"),
        }

        let mut extended = [0u8; UAPI_COMPUTE_SIZE + 1];
        extended[UAPI_COMPUTE_SIZE] = 1;
        assert_eq!(
            UapiComputeCommand::parse(&extended),
            Err(UapiTranslateError::PayloadTail)
        );
    }

    #[test]
    fn translation_checks_sampler_program_and_attachment_ranges() {
        let mut parser = UapiCommandParser::new(&RENDER_FIXTURE);
        let (mut payload, vertex, fragment) = match parser.next_hardware().unwrap().unwrap() {
            ParsedHardwareCommand::Render {
                payload,
                vertex_attachments,
                fragment_attachments,
                ..
            } => (payload, vertex_attachments, fragment_attachments),
            _ => panic!("wrong command kind"),
        };
        payload.sampler_count = 1;
        assert_eq!(
            translate_render_command(
                payload,
                vertex,
                fragment,
                window(),
                &TestAddressSpace,
                RenderInternalState {
                    parameters: G17pRenderParameters::source_rsrc8(),
                },
            ),
            Err(UapiTranslateError::SamplerPair)
        );
        payload.sampler_count = 0;
        payload.end_of_tile.usc = 0;
        assert_eq!(
            translate_render_command(
                payload,
                vertex,
                fragment,
                window(),
                &TestAddressSpace,
                RenderInternalState {
                    parameters: G17pRenderParameters::source_rsrc8(),
                },
            ),
            Err(UapiTranslateError::ProgramOffset)
        );
    }
}


#[cfg(test)]
mod exact_prior_payload_tests {
    use super::*;

    #[test]
    fn historical_wire_offsets_preserve_scs_and_render_high_words() {
        let mut render = [0u8; 272];
        render[0xf0..0xf4].copy_from_slice(&0x12345678u32.to_le_bytes());
        render[256..264].copy_from_slice(&0x123456789abcdef0u64.to_le_bytes());
        render[264..272].copy_from_slice(&0xfedcba9876543210u64.to_le_bytes());
        let p = UapiRenderCommand::parse(&render).unwrap();
        assert_eq!(p.prior_vdm, 0x123456789abcdef0);
        assert_eq!(p.prior_cdm, 0xfedcba9876543210);
        let old = UapiRenderCommand::parse(&render[..256]).unwrap();
        assert_eq!(old.end_of_tile, p.end_of_tile);
        assert_eq!((old.prior_vdm, old.prior_cdm), (0, 0));
        let mut compute = [0u8; 88];
        compute[64..72].copy_from_slice(&0x8877665544332211u64.to_le_bytes());
        compute[72..80].copy_from_slice(&0x0123456789abcdefu64.to_le_bytes());
        compute[80..88].copy_from_slice(&0xfedcba9876543210u64.to_le_bytes());
        let c = UapiComputeCommand::parse(&compute).unwrap();
        assert_eq!(c.scs_layout, 0x8877665544332211);
        assert_eq!((c.prior_vdm, c.prior_cdm), (0x0123456789abcdef, 0xfedcba9876543210));
        let old72 = UapiComputeCommand::parse(&compute[..72]).unwrap();
        assert_eq!(old72.scs_layout, c.scs_layout);
        assert_eq!((old72.prior_vdm, old72.prior_cdm), (0, 0));
        let old64 = UapiComputeCommand::parse(&compute[..64]).unwrap();
        assert_eq!((old64.scs_layout, old64.prior_vdm, old64.prior_cdm), (0, 0, 0));
    }

    #[test]
    fn truncated_extensions_zero_fill_and_unknown_nonzero_tails_reject() {
        let mut render = [0u8; 280];
        render[256] = 7;
        assert_eq!(UapiRenderCommand::parse(&render[..257]).unwrap().prior_vdm, 7);
        assert!(UapiRenderCommand::parse(&render).is_ok());
        render[272] = 1;
        assert_eq!(UapiRenderCommand::parse(&render), Err(UapiTranslateError::PayloadTail));
        let mut compute = [0u8; 96];
        compute[80] = 9;
        assert_eq!(UapiComputeCommand::parse(&compute[..81]).unwrap().prior_cdm, 9);
        assert!(UapiComputeCommand::parse(&compute).is_ok());
        compute[88] = 1;
        assert_eq!(UapiComputeCommand::parse(&compute), Err(UapiTranslateError::PayloadTail));
    }
}
