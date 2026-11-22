// SPDX-License-Identifier: GPL-2.0-only OR MIT

use crate::m5::{render_desc, submission};

use render_desc::{
    G17pRenderParameters, RenderBuildError,
};

pub(crate) const UAPI_COMMAND_HEADER_SIZE: usize = 8;

pub(crate) const UAPI_RENDER_SIZE: usize = 272;

pub(crate) const UAPI_COMPUTE_SIZE: usize = 88;

pub(crate) const UAPI_G18_COMPUTE_SIZE: usize = 136;

pub(crate) const COMPUTE_G18_RESOURCES: u32 = 1 << 2;

/// `drm_asahi_cmd_render_g18`: the render payload plus its 80-byte suffix.
pub(crate) const UAPI_G18_RENDER_SIZE: usize = UAPI_RENDER_SIZE + 80;

/// `DRM_ASAHI_RENDER_G18_RESOURCES`. Only the G18 adapter may consume and
/// strip it; the shared translator's RENDER_FLAGS keeps refusing it.
pub(crate) const RENDER_G18_RESOURCES: u32 = 1 << 21;

pub(crate) const UAPI_ATTACHMENT_SIZE: usize = 24;

pub(crate) const UAPI_MAX_ATTACHMENTS: usize = 16;

pub(crate) const UAPI_MAX_HARDWARE_COMMANDS: u32 = 64;

pub(crate) const M5_MAX_RENDER_DIMENSION: u32 = 16384;

pub(crate) const UAPI_BARRIER_NONE: u16 = 0xffff;

pub(crate) const USC_WINDOW_SIZE: u64 = 1 << 32;

pub(crate) const COMPUTE_G17P_ADD3_PROOF: u32 =
    kernel::uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_G17P_ADD3_PROOF as u32;

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
    RenderBuild(RenderBuildError),
}

impl From<RenderBuildError> for UapiTranslateError {
    fn from(value: RenderBuildError) -> Self {
        Self::RenderBuild(value)
    }
}

impl UapiTranslateError {
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
        // Legacy payloads are zero-extended exactly as before. A flagged G18
        // suffix must be complete; padding could hide a truncated field.
        let has_resources = raw.len() >= 4 && get_u32(raw, 0) & RENDER_G18_RESOURCES != 0;
        if has_resources && raw.len() < UAPI_G18_RENDER_SIZE {
            return Err(UapiTranslateError::PayloadTruncated);
        }
        if !has_resources && raw.len() > UAPI_RENDER_SIZE
            && raw[UAPI_RENDER_SIZE..].iter().any(|byte| *byte != 0)
        {
            return Err(UapiTranslateError::PayloadTail);
        }
        let raw = padded_payload::<UAPI_G18_RENDER_SIZE>(raw)?;
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
            prior_vdm: get_u64(&raw, 0x100),
            prior_cdm: get_u64(&raw, 0x108),
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
        // Legacy compute payloads are zero-extended as before. A flagged
        // metadata payload must be complete; padding it could hide truncation
        // of a valid zero-valued field or reserved word.
        let has_resources = raw.len() >= 4 && get_u32(raw, 0) & COMPUTE_G18_RESOURCES != 0;
        if has_resources && raw.len() < UAPI_G18_COMPUTE_SIZE {
            return Err(UapiTranslateError::PayloadTruncated);
        }
        if !has_resources && raw.len() > UAPI_COMPUTE_SIZE
            && raw[UAPI_COMPUTE_SIZE..].iter().any(|byte| *byte != 0)
        {
            return Err(UapiTranslateError::PayloadTail);
        }
        let raw = padded_payload::<UAPI_G18_COMPUTE_SIZE>(raw)?;
        let mut g18_resources = [0; 16];
        if has_resources {
            for (i, word) in g18_resources.iter_mut().enumerate() {
                *word = get_u32(&raw, 72 + i * 4);
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
            scs_layout: get_u64(&raw, 0x40),
            g18_resources,
            prior_vdm: if has_resources { 0 } else { get_u64(&raw, 0x48) },
            prior_cdm: if has_resources { 0 } else { get_u64(&raw, 0x50) },
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
    pub(crate) render_order: submission::G17PRenderOrder,
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

/// Validate the complete single-layer M5 Z/S allocations before their unused
/// array strides are discarded. AIL folds the sample count into the data
/// plane's element size, so 2x Z16 tiles as 4-byte elements over the pixel
/// dimensions; the compression metadata uses the expanded surface instead
/// (2x doubles Y, 4x doubles both axes). The dimensions register stays in
/// pixels either way.
pub(crate) fn validate_m5_reference_depth<A: GpuAddressSpace>(
    command: &UapiRenderCommand,
    address_space: &A,
) -> Result<(), UapiTranslateError> {
    let width = u32::from(command.width);
    let height = u32::from(command.height);
    if !(1..=M5_MAX_RENDER_DIMENSION).contains(&width)
        || !(1..=M5_MAX_RENDER_DIMENSION).contains(&height)
        || command.layers != 1 || !matches!(command.samples, 1 | 2 | 4) {
        return Err(UapiTranslateError::ZlsFields);
    }
    let sample_width = width * if command.samples == 4 { 2 } else { 1 };
    let sample_height = height * if command.samples > 1 { 2 } else { 1 };
    let depth = command.depth;
    let stencil = command.stencil;
    let z16 = command.zls_control & (3 << 25) == (2 << 25);
    let mut expected_control = 0;
    for (plane, bytes_per_sample, transfers, compression) in [
        (depth, if z16 { 2u64 } else { 4u64 }, 0x88000u64, 0x44u64),
        (stencil, 1u64, 0x44000u64, 0x110u64),
    ] {
        if plane.base == 0 {
            if plane.compression_base != 0 || plane.stride != 0
                || plane.compression_stride != 0 {
                return Err(UapiTranslateError::ZlsFields);
            }
            continue;
        }
        let compressed = plane.compression_base != 0;
        // Compressed Z16 remains outside the captured layout. Both Z32 and
        // S8 compression use 8-byte metadata records for 16x16 sample tiles.
        if plane.base & 0x7f != 0 || plane.compression_base & 0x7f != 0
            || (!compressed && plane.compression_stride != 0)
            || (compressed && (bytes_per_sample == 2
                || sample_width < 16 || sample_height < 16)) {
            return Err(UapiTranslateError::ZlsFields);
        }
        expected_control |= command.zls_control & transfers;
        if compressed { expected_control |= compression; }
        // AIL tiles contain 16 KiB and their shape follows the element size:
        // 64x64 at four bytes, 128x64 at two, 128x128 at one, 64x32 at eight
        // and 32x32 at sixteen. The sample count scales that element size,
        // so the tile shape must be picked from bytes_per_sample * samples
        // and the tiles counted over the pixel dimensions.
        // A miptail smaller than either tile axis is power-of-two padded and
        // cacheline aligned, rather than rounded to a whole 16-KiB page.
        let bytes_per_element = bytes_per_sample * u64::from(command.samples);
        let (tile_width, tile_height) = match bytes_per_element {
            1 => (128u32, 128u32),
            2 => (128, 64),
            4 => (64, 64),
            8 => (64, 32),
            16 => (32, 32),
            _ => return Err(UapiTranslateError::ZlsFields),
        };
        let data_bytes = if width >= tile_width && height >= tile_height {
            u64::from(width.div_ceil(tile_width))
                * u64::from(height.div_ceil(tile_height)) * 0x4000
        } else {
            (u64::from(width.next_power_of_two())
                * u64::from(height.next_power_of_two()) * bytes_per_element)
                .div_ceil(128) * 128
        };
        require_range(address_space, plane.base, data_bytes, GpuAccess::ReadWrite)?;
        if compressed {
            let metadata_bytes = (u64::from(sample_width.next_power_of_two().div_ceil(16))
                * u64::from(sample_height.next_power_of_two().div_ceil(16)) * 8)
                .div_ceil(128) * 128;
            require_range(address_space, plane.compression_base, metadata_bytes,
                GpuAccess::ReadWrite)?;
        }
    }
    if depth.base != 0 {
        if z16 { expected_control |= 2 << 25; }
        if (command.depth_clear & 0x7fffffff) > 0x7f800000 {
            return Err(UapiTranslateError::ZlsFields);
        }
    }
    let dimensions = if depth.base != 0 || stencil.base != 0 {
        (width - 1) | ((height - 1) << 16)
    } else { 0 };
    // Exact matching rejects unknown format bits and transfer requests for
    // an absent plane, as well as mismatched compression/base-address pairs.
    if command.zls_control != expected_control || command.depth_dimensions != dimensions {
        return Err(UapiTranslateError::ZlsFields);
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
    parameters.samples = command.samples;
    parameters.sample_size = command.sample_size;
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
        render_order: submission::G17PRenderOrder::default(),
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

impl TranslatedComputeCommand {

}
