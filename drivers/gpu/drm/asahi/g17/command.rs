// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Decode a submission and validate its user resources before publishing any work.

use super::fw::{
    compute::{IntersectionTables, ScratchRequest},
    kick::WriteRange,
    render::{RenderPass, UscProgram, ZlsSurface},
    render_rt::RayQueryStage,
};
use crate::util::{AnyBitPattern, Reader};
use core::ops::Range;
use kernel::{prelude::*, uapi};

const _: () = assert!(uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_AS_BUILD == 1 << 2);

const MAX_ATTACHMENTS: usize = crate::microseq::MAX_ATTACHMENTS;
const USC_WINDOW_SIZE: u64 = 1 << 32;
const MAX_SAMPLERS: u32 = 1024;
const RENDER_BASE_SIZE: usize = core::mem::offset_of!(uapi::drm_asahi_cmd_render, prior_cdm) + size_of::<u64>();
const COMPUTE_BASE_SIZE: usize = core::mem::offset_of!(uapi::drm_asahi_cmd_compute, prior_cdm) + size_of::<u64>();
/// The timestamp-copy arrays of the common command layout, which G17P does not implement.
const RENDER_TIMESTAMP_COPIES: Range<usize> = core::mem::offset_of!(uapi::drm_asahi_cmd_render, ts_frag_end_copies)
    ..core::mem::offset_of!(uapi::drm_asahi_cmd_render, prior_vdm);
const COMPUTE_TIMESTAMP_COPIES: Range<usize> = core::mem::offset_of!(uapi::drm_asahi_cmd_compute, ts_end_copies)
    ..core::mem::offset_of!(uapi::drm_asahi_cmd_compute, scs_layout);
const SAMPLER_SIZE: u64 = 8;
const PAGE_SIZE: u64 = 0x4000;
const ZLS_METADATA_STRIDE: u64 = 0x80;
const MAX_DIMENSION: u16 = 16384;
const MAX_LAYERS: u16 = 2048;
const MAX_TILE_BUFFER: u64 = 32768;
const PROCESS_EMPTY: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES;
const RSRC_SPEC_HI: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_RSRC_SPEC_HI;
const DBIAS_INT: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_DBIAS_IS_INT;
const FEW_PRIMITIVES: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_FEW_PRIMITIVES;
const FRAGMENT_BARRIERS: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_FRAGMENT_BARRIERS;
const RENDER_FLAGS: u32 =
    PROCESS_EMPTY | RSRC_SPEC_HI | DBIAS_INT | FEW_PRIMITIVES | FRAGMENT_BARRIERS;

/// Attachment settings persist until replaced, including by an empty list.
#[derive(Clone, Copy)]
pub(crate) struct Attachments {
    ranges: [WriteRange; MAX_ATTACHMENTS],
    count: usize,
}

impl Attachments {
    const EMPTY: Self = Self {
        ranges: [WriteRange { va: 0, size: 0 }; MAX_ATTACHMENTS],
        count: 0,
    };

    pub(crate) fn as_slice(&self) -> &[WriteRange] {
        &self.ranges[..self.count]
    }

    fn parse(bytes: &[u8]) -> Result<Self> {
        let size = size_of::<uapi::drm_asahi_attachment>();
        if bytes.len() % size != 0 || bytes.len() / size > MAX_ATTACHMENTS {
            return Err(EINVAL);
        }
        let mut list = Self::EMPTY;
        let mut reader = Reader::new(bytes);
        for range in &mut list.ranges[..bytes.len() / size] {
            let item: uapi::drm_asahi_attachment = reader.read()?;
            if item.flags != 0 || item.pad != 0 {
                return Err(EINVAL);
            }
            *range = WriteRange {
                va: item.pointer,
                size: item.size,
            };
            list.count += 1;
        }
        Ok(list)
    }

    fn validate(&self, space: &impl AddressSpace) -> Result {
        for range in self.as_slice() {
            require_range(space, range.va, range.size, Access::Write)?;
        }
        Ok(())
    }
}

// Commands stay inline in the submission plan, avoiding an allocation per render pass.
#[allow(clippy::large_enum_variant)]
pub(crate) enum Payload {
    Render(uapi::drm_asahi_cmd_render, [Attachments; 2], [RayQueryStage; 2]),
    Compute(uapi::drm_asahi_cmd_compute, Attachments, IntersectionTables),
}

pub(crate) struct Command {
    /// Render and compute prefix indices, before this command.
    pub(crate) barriers: [u16; 2],
    pub(crate) payload: Payload,
}

/// Parser over the single userspace snapshot taken by the submit ioctl.
/// Call `finish` after consuming it to reject submissions with no hardware work.
pub(crate) struct Parser<'a> {
    bytes: &'a [u8],
    counts: [u16; 2],
    attachments: [Attachments; 3],
}

impl<'a> Parser<'a> {
    pub(crate) fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            counts: [0; 2],
            attachments: [Attachments::EMPTY; 3],
        }
    }

    pub(crate) fn next(&mut self) -> Result<Option<Command>> {
        while !self.bytes.is_empty() {
            let mut reader = Reader::new(self.bytes);
            let header: uapi::drm_asahi_cmd_header = reader.read()?;
            let start = size_of::<uapi::drm_asahi_cmd_header>();
            let end = start + header.size as usize;
            let bytes = self.bytes.get(start..end).ok_or(EINVAL)?;
            self.bytes = &self.bytes[end..];
            let barriers = [header.vdm_barrier, header.cdm_barrier];
            match u32::from(header.cmd_type) {
                uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER
                | uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE => {
                    if barriers.iter().zip(self.counts).any(|(&barrier, count)| {
                        barrier != uapi::DRM_ASAHI_BARRIER_NONE as u16 && barrier > count
                    }) || u32::from(self.counts[0]) + u32::from(self.counts[1])
                        >= crate::file::MAX_COMMANDS_PER_SUBMISSION
                    {
                        return Err(EINVAL);
                    }
                    let payload = if u32::from(header.cmd_type)
                        == uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER
                    {
                        self.counts[0] += 1;
                        no_timestamp_copies(bytes, RENDER_TIMESTAMP_COPIES)?;
                        Payload::Render(
                            payload(&bytes[..bytes.len().min(RENDER_BASE_SIZE)])?,
                            [self.attachments[0], self.attachments[1]],
                            RayQueryStage::parse_pair(bytes.get(RENDER_BASE_SIZE..).unwrap_or(&[]))?,
                        )
                    } else {
                        self.counts[1] += 1;
                        no_timestamp_copies(bytes, COMPUTE_TIMESTAMP_COPIES)?;
                        Payload::Compute(
                            payload(&bytes[..bytes.len().min(COMPUTE_BASE_SIZE)])?,
                            self.attachments[2],
                            IntersectionTables::parse(bytes.get(COMPUTE_BASE_SIZE..).unwrap_or(&[]))?,
                        )
                    };
                    return Ok(Some(Command { barriers, payload }));
                }
                uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_VERTEX_ATTACHMENTS
                | uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_FRAGMENT_ATTACHMENTS
                | uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_COMPUTE_ATTACHMENTS => {
                    if barriers != [uapi::DRM_ASAHI_BARRIER_NONE as u16; 2] {
                        return Err(EINVAL);
                    }
                    let slot = header.cmd_type as usize
                        - uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_VERTEX_ATTACHMENTS as usize;
                    self.attachments[slot] = Attachments::parse(bytes)?;
                }
                _ => return Err(EINVAL),
            }
        }
        Ok(None)
    }

    pub(crate) fn finish(&self) -> Result {
        if !self.bytes.is_empty() || self.counts == [0; 2] {
            return Err(EINVAL);
        }
        Ok(())
    }
}

/// Reject a command that asks for timestamp copies.
fn no_timestamp_copies(bytes: &[u8], copies: Range<usize>) -> Result {
    let end = copies.end.min(bytes.len());
    let start = copies.start.min(end);
    if bytes[start..end].iter().any(|&b| b != 0) {
        return Err(EINVAL);
    }
    Ok(())
}

fn payload<T: AnyBitPattern>(bytes: &[u8]) -> Result<T> {
    if bytes
        .get(size_of::<T>()..)
        .is_some_and(|tail| tail.iter().any(|&b| b != 0))
    {
        return Err(EINVAL);
    }
    Reader::new(bytes).read_up_to(bytes.len())
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Access {
    Read,
    Write,
    ReadWrite,
}

/// The caller retains the VM mappings while validating and executing the command.
pub(crate) trait AddressSpace {
    fn covers(&self, address: u64, size: u64, access: Access) -> bool;
}

fn require_range(space: &impl AddressSpace, address: u64, size: u64, access: Access) -> Result {
    if size == 0 || address.checked_add(size).is_none() || !space.covers(address, size, access) {
        return Err(EINVAL);
    }
    Ok(())
}

/// A validated four-GiB USC window owned by one queue.
pub(crate) struct UscWindow {
    base: u64,
}
impl UscWindow {
    pub(crate) fn new(base: u64, range: Range<u64>) -> Result<Self> {
        if base & (USC_WINDOW_SIZE - 1) != 0
            || base < range.start
            || base
                .checked_add(USC_WINDOW_SIZE)
                .is_none_or(|end| end > range.end)
        {
            return Err(EINVAL);
        }
        Ok(Self { base })
    }

    fn program(&self, space: &impl AddressSpace, tagged_offset: u32) -> Result {
        if tagged_offset == 0 {
            return Err(EINVAL);
        }
        require_range(
            space,
            self.base + u64::from(tagged_offset & !7),
            4,
            Access::Read,
        )
    }
}

fn sampler(space: &impl AddressSpace, heap: u64, count: u32) -> Result {
    if count > MAX_SAMPLERS || (heap == 0) != (count == 0) || heap & (SAMPLER_SIZE - 1) != 0 {
        return Err(EINVAL);
    }
    if count != 0 {
        require_range(space, heap, u64::from(count) * SAMPLER_SIZE, Access::Read)?;
    }
    Ok(())
}

fn helper_is_empty(helper: &uapi::drm_asahi_helper_program) -> bool {
    helper.binary == 0 && helper.cfg == 0 && helper.data == 0
}

fn surface(
    space: &impl AddressSpace,
    zls: &uapi::drm_asahi_zls_buffer,
    layers: u16,
) -> Result<ZlsSurface> {
    if (zls.base == 0 && (zls.comp_base != 0 || zls.stride != 0 || zls.comp_stride != 0))
        || (zls.comp_base == 0 && zls.comp_stride != 0)
        || (layers > 1 && zls.base != 0 && zls.stride == 0)
        || (zls.stride != 0 && u64::from(zls.stride) & (PAGE_SIZE - 1) != 1)
        || u64::from(zls.comp_stride) & (PAGE_SIZE - 1) != 0
    {
        return Err(EINVAL);
    }
    if zls.base != 0 {
        let stride = if zls.stride == 0 {
            0
        } else {
            (u64::from(zls.stride) / PAGE_SIZE + 1) * PAGE_SIZE
        };
        require_range(
            space,
            zls.base,
            stride * u64::from(layers - 1) + 1,
            Access::ReadWrite,
        )?;
    }
    if zls.comp_base != 0 {
        let stride = (u64::from(zls.comp_stride) / PAGE_SIZE + 1) * ZLS_METADATA_STRIDE;
        require_range(
            space,
            zls.comp_base,
            stride * u64::from(layers - 1) + 1,
            Access::ReadWrite,
        )?;
    }
    Ok(ZlsSurface {
        va: zls.base,
        compression_va: zls.comp_base,
        stride: zls.stride.into(),
        compression_stride: zls.comp_stride.into(),
    })
}

// As with Payload, retain the bounded attachment lists inside the submission plan.
#[allow(clippy::large_enum_variant)]
pub(crate) enum Validated {
    Render {
        long_job: bool,
        pass: RenderPass,
        ray_query: [RayQueryStage; 2],
        attachments: [Attachments; 2],
        timestamps: [uapi::drm_asahi_timestamps; 2],
    },
    Compute {
        long_job: bool,
        intersection: IntersectionTables,
        scratch: ScratchRequest,
        usc_base: u64,
        cdm_va: u64,
        cdm_end_va: u64,
        sampler_heap_va: u64,
        sampler_count: u32,
        timestamps: uapi::drm_asahi_timestamps,
    },
}

impl Validated {
    pub(crate) fn long_job(&self) -> bool {
        match self {
            Self::Render { long_job, .. } | Self::Compute { long_job, .. } => *long_job,
        }
    }
    pub(crate) fn allow_long_job(&mut self) {
        match self {
            Self::Render { long_job, .. } | Self::Compute { long_job, .. } => *long_job = true,
        }
    }
}

impl Payload {
    pub(crate) fn prior(&self) -> [u64; 2] {
        match self {
            Self::Render(cmd, _, _) => [cmd.prior_vdm, cmd.prior_cdm],
            Self::Compute(cmd, _, _) => [cmd.prior_vdm, cmd.prior_cdm],
        }
    }

    pub(crate) fn validate(
        &self,
        window: &UscWindow,
        variant: &super::fw::Variant,
        space: &impl AddressSpace,
    ) -> Result<Validated> {
        match self {
            Self::Render(cmd, attachments, ray_query) => {
                if cmd.flags & !RENDER_FLAGS != 0
                    || !(1..=MAX_DIMENSION).contains(&cmd.width_px)
                    || !(1..=MAX_DIMENSION).contains(&cmd.height_px)
                    || !(1..=MAX_LAYERS).contains(&cmd.layers)
                    || !matches!(
                        (cmd.utile_width_px, cmd.utile_height_px),
                        (16, 16) | (32, 16) | (32, 32)
                    )
                {
                    return Err(EINVAL);
                }
                let samples_log2 = match cmd.samples {
                    1 => 0,
                    2 => 1,
                    4 => 2,
                    _ => return Err(EINVAL),
                };
                if u64::from(cmd.sample_size_B)
                    * u64::from(cmd.utile_width_px)
                    * u64::from(cmd.utile_height_px)
                    * u64::from(cmd.samples)
                    > MAX_TILE_BUFFER
                    || cmd.vdm_ctrl_stream_base == 0
                    || cmd.vdm_ctrl_stream_base & 3 != 0
                    || cmd.isp_scissor_base == 0
                    || cmd.isp_scissor_base & 7 != 0
                {
                    return Err(EINVAL);
                }
                require_range(space, cmd.vdm_ctrl_stream_base, 4, Access::Read)?;
                require_range(space, cmd.isp_scissor_base, 8, Access::Read)?;
                for (va, access) in [
                    (cmd.isp_dbias_base, Access::Read),
                    (cmd.isp_oclqry_base, Access::Write),
                ] {
                    if va != 0 {
                        if va & 7 != 0 {
                            return Err(EINVAL);
                        }
                        require_range(space, va, 8, access)?;
                    }
                }
                let depth = surface(space, &cmd.depth, cmd.layers)?;
                let stencil = surface(space, &cmd.stencil, cmd.layers)?;
                sampler(space, cmd.sampler_heap, cmd.sampler_count.into())?;
                if !helper_is_empty(&cmd.vertex_helper) || !helper_is_empty(&cmd.fragment_helper) {
                    return Err(EINVAL);
                }
                for program in [&cmd.bg, &cmd.eot, &cmd.partial_bg, &cmd.partial_eot] {
                    window.program(space, program.usc)?;
                }
                for list in attachments {
                    list.validate(space)?;
                }
                let pass = RenderPass {
                    width: cmd.width_px.into(),
                    height: cmd.height_px.into(),
                    layers: cmd.layers,
                    utile_width: cmd.utile_width_px,
                    utile_height: cmd.utile_height_px,
                    samples_log2,
                    sample_size: cmd.sample_size_B,
                    process_empty_tiles: cmd.flags & PROCESS_EMPTY != 0,
                    msaa_control: cmd.ppp_multisamplectl,
                    ppp_control: cmd.ppp_ctrl.into(),
                    vdm_va: cmd.vdm_ctrl_stream_base,
                    scissor_va: cmd.isp_scissor_base,
                    depth_bias_va: cmd.isp_dbias_base,
                    depth_bias_is_int: cmd.flags & DBIAS_INT != 0,
                    few_primitives: cmd.flags & FEW_PRIMITIVES != 0 && variant.few_primitives,
                    fragment_barriers: cmd.flags & FRAGMENT_BARRIERS != 0,
                    occlusion_query_va: cmd.isp_oclqry_base,
                    sampler_heap_va: cmd.sampler_heap,
                    sampler_count: cmd.sampler_count.into(),
                    zls_control: cmd.zls_ctrl,
                    depth_dimensions: cmd.isp_zls_pixels.into(),
                    depth,
                    stencil,
                    depth_clear: cmd.isp_bgobjdepth,
                    stencil_clear: cmd.isp_bgobjvals,
                    merge_upper_x: cmd.isp_merge_upper_x,
                    merge_upper_y: cmd.isp_merge_upper_y,
                    usc_base: window.base,
                    bg: UscProgram {
                        offset: cmd.bg.usc,
                        rsrc_spec: u64::from(cmd.bg.rsrc_spec)
                            | (u64::from(cmd.bg_rsrc_spec_hi) << 32),
                    },
                    partial_bg: UscProgram {
                        offset: cmd.partial_bg.usc,
                        rsrc_spec: u64::from(cmd.partial_bg.rsrc_spec)
                            | (u64::from(cmd.bg_partial_rsrc_spec_hi) << 32),
                    },
                    eot_offset: cmd.eot.usc,
                    eot_rsrc_spec: u64::from(cmd.eot.rsrc_spec)
                        | (u64::from(cmd.bg_eot_rsrc_spec_hi) << 32),
                    partial_eot_offset: cmd.partial_eot.usc,
                    partial_eot_rsrc_spec: u64::from(cmd.partial_eot.rsrc_spec)
                        | (u64::from(cmd.bg_eot_partial_rsrc_spec_hi) << 32),
                    rsrc_spec_hi: cmd.flags & RSRC_SPEC_HI != 0,
                };
                for stage in ray_query {
                    if (stage.intersection.enabled() || stage.scratch.enabled())
                        && !variant.ray_traversal
                    {
                        return Err(refuse_ray_traversal());
                    }
                    if stage.intersection.enabled() {
                        require_range(space, stage.intersection.pool, stage.intersection.pool_bytes, Access::Read)?;
                        let code = stage.intersection.default_program & 0x0000_ffff_ffff_ffff;
                        if code != 0 { require_range(space, code, 64, Access::Read)?; }
                    }
                }
                Ok(Validated::Render {
                    long_job: super::timeout::select(false,
                        ray_query.map(|stage| stage.intersection.enabled()), false),
                    pass,
                    ray_query: *ray_query,
                    attachments: *attachments,
                    timestamps: [cmd.ts_vtx, cmd.ts_frag],
                })
            }
            Self::Compute(cmd, attachments, intersection) => {
                let as_build = super::timeout::as_build(cmd.flags).ok_or(EINVAL)?;
                if cmd.cdm_ctrl_stream_base & 3 != 0
                    || cmd.cdm_ctrl_stream_end & 3 != 0
                    || cmd.cdm_ctrl_stream_end <= cmd.cdm_ctrl_stream_base
                    || !helper_is_empty(&cmd.helper)
                {
                    return Err(EINVAL);
                }
                require_range(
                    space,
                    cmd.cdm_ctrl_stream_base,
                    cmd.cdm_ctrl_stream_end - cmd.cdm_ctrl_stream_base,
                    Access::Read,
                )?;
                sampler(space, cmd.sampler_heap, cmd.sampler_count)?;
                attachments.validate(space)?;
                let scratch = ScratchRequest::new(cmd.scs_layout)?;
                if (scratch.enabled() || intersection.enabled()) && !variant.ray_traversal {
                    return Err(refuse_ray_traversal());
                }
                if intersection.enabled() {
                    require_range(space, intersection.pool, intersection.pool_bytes, Access::Read)?;
                    let code = intersection.default_program & 0x0000_ffff_ffff_ffff;
                    if code != 0 { require_range(space, code, 64, Access::Read)?; }
                }
                Ok(Validated::Compute {
                    long_job: super::timeout::select(intersection.enabled(), [false; 2], as_build),
                    intersection: *intersection,
                    scratch,
                    usc_base: window.base,
                    cdm_va: cmd.cdm_ctrl_stream_base,
                    cdm_end_va: cmd.cdm_ctrl_stream_end - 4,
                    sampler_heap_va: cmd.sampler_heap,
                    sampler_count: cmd.sampler_count,
                    timestamps: cmd.ts,
                })
            }
        }
    }
}

/// Ray-traversal work on a firmware build that does not run it ([`super::fw::Variant`]).
fn refuse_ray_traversal() -> Error {
    kernel::pr_warn_once!("G17: ray-traversal work refused on this firmware\n");
    Error::from_errno(-(kernel::bindings::EOPNOTSUPP as i32))
}
