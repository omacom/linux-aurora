// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! SUBMIT parsing and validation into engine commands.
//!
//! The rules, checked in this order: the whole stream's structure ([`check_stream`]), then the
//! timestamp handles and the historical priors ([`prepare`]), then each command's mapped spans
//! and gates ([`Prepared::validate`]):
//!
//! - Headers 8 bytes, payloads multiples of 8, at most 4 MiB; short payloads zero-filled, bytes
//!   past the known struct must be zero; 1..=64 render+compute commands; attachments (types
//!   2..4) with NONE barriers, at most 16 x 24 bytes, pad/flags zero, inside the VM range.
//! - Header barriers: NONE or at most the number of earlier same-engine commands in this
//!   SUBMIT; `prior_vdm`/`prior_cdm` at most the commands of that engine accepted on the queue
//!   before this ioctl (from 1); NONE does not suppress a prior ([`check_order`], the semantics
//!   of `g17::job::Order`). Validated only: the total order satisfies them.
//! - Render (256, 272 with EXACT_PRIOR, up to 384 bytes): flags PROCESS_EMPTY_TILES,
//!   NO_VERTEX_CLUSTERING, RSRC_SPEC_HI, DBIAS_IS_INT (+ FRAGMENT_BARRIERS only with
//!   `g15_early_tiling`); VERTEX_SCRATCH, FEW_PRIMITIVES, helpers, sampler heap/count,
//!   `ray_query`, nonzero partial-EOT high word: EINVAL; RSRC_SPEC_HI needs >= 256 bytes;
//!   without it every `*_spec_hi` is zero; utile 16 or 32; `sample_size_B` multiple of 8 within
//!   the tile buffer; `ppp_ctrl & !0x203 == 0`; stencil compression, or depth compression with
//!   layers or samples != 1: EOPNOTSUPP; VDM base 4-aligned inside the compact window; scissor
//!   8-aligned; every referenced span mapped with the right access (ENOENT/EACCES); geometry
//!   too large: E2BIG.
//! - Compute (64, 136 bytes): flags, `scs_layout`, `cdm_auxiliary_allocation`, `reserved`,
//!   `intersection_*`, helpers, samplers zero; stream base/end 4-aligned.
//! - Timestamps resolve BIND_OBJECT handles to firmware arena VAs ([`super::timestamp`]).
//! - Stream gates ([`super::gate`]) when `g15_stream_gate`.

use core::mem::{offset_of, size_of};
use core::ops::Range;
use kernel::{prelude::*, sync::Arc, uapi, xarray};

use super::{gate, hw};
use crate::{file, mmu, util::AnyBitPattern, util::Reader};

/// Largest command buffer.
pub(crate) const MAX_CMDBUF: usize = 4 << 20;
/// Most input plus output syncs of one SUBMIT.
pub(crate) const MAX_SYNCS: usize = 4096;
/// Hardware commands per SUBMIT.
const MAX_COMMANDS: u32 = crate::file::MAX_COMMANDS_PER_SUBMISSION;

const HEADER_SIZE: usize = size_of::<uapi::drm_asahi_cmd_header>();
const ATTACHMENT_SIZE: usize = size_of::<uapi::drm_asahi_attachment>();
/// A render payload with RSRC_SPEC_HI carries every high word.
const RSRC_SPEC_HI_MIN: usize =
    offset_of!(uapi::drm_asahi_cmd_render, bg_partial_rsrc_spec_hi) + size_of::<u32>();
/// Render payload bytes from the ray-query stages on (RENDER_RAY_QUERY is never advertised).
const RAY_QUERY: usize = offset_of!(uapi::drm_asahi_cmd_render, ray_query);
const NONE: u16 = uapi::DRM_ASAHI_BARRIER_NONE as u16;

const VERTEX_SCRATCH: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_VERTEX_SCRATCH;
const PROCESS_EMPTY_TILES: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES;
const NO_VERTEX_CLUSTERING: u32 =
    uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING;
const RSRC_SPEC_HI: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_RSRC_SPEC_HI;
const DBIAS_IS_INT: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_DBIAS_IS_INT;
const FRAGMENT_BARRIERS: u32 = uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_FRAGMENT_BARRIERS;

const CMD_RENDER: u32 = uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_RENDER;
const CMD_COMPUTE: u32 = uapi::drm_asahi_cmd_type_DRM_ASAHI_CMD_COMPUTE;
const SET_VERTEX_ATTACHMENTS: u32 = uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_VERTEX_ATTACHMENTS;
const SET_FRAGMENT_ATTACHMENTS: u32 = uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_FRAGMENT_ATTACHMENTS;
const SET_COMPUTE_ATTACHMENTS: u32 = uapi::drm_asahi_cmd_type_DRM_ASAHI_SET_COMPUTE_ATTACHMENTS;

const MAX_DIMENSION: u16 = 16384;
const MAX_LAYERS: u16 = 2048;
const MAX_TILE_BUFFER: u32 = 32768;
/// Size of a queue's USC window.
pub(crate) const USC_WINDOW: u64 = 1 << 32;

/// Engine index of render commands (VDM) in the accepted counts.
pub(crate) const RENDER: usize = 0;
/// Engine index of compute commands (CDM).
pub(crate) const COMPUTE: usize = 1;

/// Checks the ordering of one command (the semantics of `g17::job::Order`) after `preceding`
/// commands of this SUBMIT, on a queue that accepted `base` commands before it. A zero barrier
/// waits for every earlier SUBMIT; NONE waits for nothing in this SUBMIT; a prior names an
/// ordinal accepted before it. The ordinals themselves are not kept: the engine's total order
/// satisfies every valid one.
fn check_order(base: [u64; 2], preceding: [u16; 2], barriers: [u16; 2], prior: [u64; 2]) -> Result {
    for index in 0..2 {
        // Earlier commands in this ioctl are not historical commands.
        if prior[index] > base[index] {
            return Err(EINVAL);
        }
        base[index]
            .checked_add(u64::from(preceding[index]))
            .and_then(|value| value.checked_add(1))
            .ok_or(EOVERFLOW)?;
        if barriers[index] != NONE {
            if barriers[index] > preceding[index] {
                return Err(EINVAL);
            }
            base[index]
                .checked_add(u64::from(barriers[index]))
                .ok_or(EOVERFLOW)?;
        }
    }
    Ok(())
}

/// A validated render command.
pub(crate) struct RenderCommand {
    /// The payload, zero-extended to the full UAPI structure.
    pub(crate) cmd: uapi::drm_asahi_cmd_render,
    /// Firmware arena VAs of the timestamp slots (vertex start/end, fragment start/end), 0 if
    /// unused.
    pub(crate) timestamps: [u64; 4],
}

/// A validated compute command.
pub(crate) struct ComputeCommand {
    pub(crate) cmd: uapi::drm_asahi_cmd_compute,
    /// The compute attachments in effect (validated writable).
    pub(crate) attachments: KVec<uapi::drm_asahi_attachment>,
    /// Firmware arena VAs of the start and end timestamp slots, 0 if unused.
    pub(crate) timestamps: [u64; 2],
}

/// A validated command.
pub(crate) enum Command {
    Render(KBox<RenderCommand>),
    Compute(KBox<ComputeCommand>),
}

/// A validated SUBMIT.
pub(crate) struct Submission {
    /// The commands, in submission order.
    pub(crate) commands: KVec<Command>,
    /// Render and compute commands, for the queue's EXACT_PRIOR ordinals.
    pub(crate) counts: [u64; 2],
    /// The timestamp buffers the commands name, retained with the job.
    pub(crate) timestamps: KVec<Arc<mmu::KernelMapping>>,
}

/// What the per-command checks need.
pub(crate) struct Context<'a> {
    /// The submitting VM.
    pub(crate) vm: &'a mmu::Vm,
    /// The queue's USC window base.
    pub(crate) usc_base: u64,
}

/// A stream whose timestamps and ordinals are resolved ([`prepare`]), before its spans are
/// checked ([`Prepared::validate`]).
pub(crate) struct Prepared<'a> {
    bytes: &'a [u8],
    /// Arena VAs of each command's timestamp slots.
    resolved: KVec<[u64; 4]>,
    counts: [u64; 2],
    timestamps: KVec<Arc<mmu::KernelMapping>>,
}

/// One command of the stream.
struct Entry<'a> {
    kind: u32,
    barriers: [u16; 2],
    payload: &'a [u8],
}

/// Iterates the commands of a structurally valid stream.
struct Stream<'a> {
    bytes: &'a [u8],
}

impl<'a> Iterator for Stream<'a> {
    type Item = Result<Entry<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.bytes.is_empty() {
            return None;
        }
        let entry = (|| {
            if self.bytes.len() < HEADER_SIZE {
                return Err(EINVAL);
            }
            let header: uapi::drm_asahi_cmd_header = Reader::new(self.bytes).read()?;
            let size = header.size as usize;
            let payload = self
                .bytes
                .get(HEADER_SIZE..HEADER_SIZE + size)
                .ok_or(EINVAL)?;
            if size % 8 != 0 {
                return Err(EINVAL);
            }
            self.bytes = &self.bytes[HEADER_SIZE + size..];
            Ok(Entry {
                kind: header.cmd_type.into(),
                barriers: [header.vdm_barrier, header.cdm_barrier],
                payload,
            })
        })();
        if entry.is_err() {
            self.bytes = &[];
        }
        Some(entry)
    }
}

fn stream(bytes: &[u8]) -> Stream<'_> {
    Stream { bytes }
}

/// Decodes a payload: short ones are zero-filled, bytes past the known structure must be zero.
fn payload<T: AnyBitPattern>(bytes: &[u8]) -> Result<T> {
    if bytes
        .get(size_of::<T>()..)
        .is_some_and(|tail| tail.iter().any(|&byte| byte != 0))
    {
        return Err(EINVAL);
    }
    Reader::new(bytes).read_up_to(bytes.len())
}

/// Whether `[start, start + size)` is a nonempty range inside `range`.
fn inside(start: u64, size: u64, range: &Range<u64>) -> bool {
    size != 0 && start >= range.start && start < range.end && size <= range.end - start
}

/// Whether no timestamp copy is requested (the copy arrays of the common command layout, which
/// G15 does not implement).
fn timestamps_empty(copies: &[uapi::drm_asahi_timestamp]) -> bool {
    copies.iter().all(|ts| ts.handle == 0 && ts.offset == 0)
}

fn helper_is_empty(helper: &uapi::drm_asahi_helper_program) -> bool {
    helper.binary == 0 && helper.cfg == 0 && helper.data == 0
}

/// Structural validation of a whole command stream, before anything is looked up.
pub(crate) fn check_stream(bytes: &[u8], early_tiling: bool) -> Result {
    if bytes.is_empty() || bytes.len() > MAX_CMDBUF || bytes.len() % 8 != 0 {
        return Err(EINVAL);
    }
    let range = hw::USER_RANGE;
    let mut counts = [0u32; 2];
    for entry in stream(bytes) {
        let entry = entry?;
        match entry.kind {
            SET_VERTEX_ATTACHMENTS | SET_FRAGMENT_ATTACHMENTS | SET_COMPUTE_ATTACHMENTS => {
                if entry.barriers != [NONE; 2]
                    || entry.payload.len() % ATTACHMENT_SIZE != 0
                    || entry.payload.len() / ATTACHMENT_SIZE > hw::sizes::MAX_ATTACHMENTS
                {
                    return Err(EINVAL);
                }
                for bytes in entry.payload.chunks_exact(ATTACHMENT_SIZE) {
                    let attachment: uapi::drm_asahi_attachment = Reader::new(bytes).read()?;
                    if attachment.pad != 0
                        || attachment.flags != 0
                        || !inside(attachment.pointer, attachment.size, &range)
                    {
                        return Err(EINVAL);
                    }
                }
                continue;
            }
            _ => {}
        }
        if (entry.barriers[RENDER] != NONE && u32::from(entry.barriers[RENDER]) > counts[RENDER])
            || (entry.barriers[COMPUTE] != NONE
                && u32::from(entry.barriers[COMPUTE]) > counts[COMPUTE])
        {
            return Err(EINVAL);
        }
        match entry.kind {
            CMD_RENDER => {
                let cmd: uapi::drm_asahi_cmd_render = payload(entry.payload)?;
                // G15S resource words are 64-bit: RSRC_SPEC_HI is always accepted.
                let allowed = VERTEX_SCRATCH
                    | PROCESS_EMPTY_TILES
                    | NO_VERTEX_CLUSTERING
                    | DBIAS_IS_INT
                    | RSRC_SPEC_HI
                    | if early_tiling { FRAGMENT_BARRIERS } else { 0 };
                if cmd.flags & !allowed != 0
                    || !(1..=MAX_DIMENSION).contains(&cmd.width_px)
                    || !(1..=MAX_DIMENSION).contains(&cmd.height_px)
                    || !(1..=MAX_LAYERS).contains(&cmd.layers)
                    || !matches!(cmd.samples, 1 | 2 | 4)
                {
                    return Err(EINVAL);
                }
                // RENDER_RAY_QUERY is never advertised.
                if entry
                    .payload
                    .get(RAY_QUERY..)
                    .is_some_and(|tail| tail.iter().any(|&byte| byte != 0))
                {
                    return Err(EINVAL);
                }
                // RENDER_TIMESTAMP_COPIES is never advertised: the copies must be empty.
                if !timestamps_empty(&cmd.ts_frag_end_copies) {
                    return Err(EINVAL);
                }
                // Never discard unflagged high words, nor accept the flag without them.
                if cmd.flags & RSRC_SPEC_HI != 0 && entry.payload.len() < RSRC_SPEC_HI_MIN {
                    return Err(EINVAL);
                }
                if cmd.flags & RSRC_SPEC_HI == 0
                    && (cmd.bg_eot_rsrc_spec_hi != 0
                        || cmd.bg_eot_partial_rsrc_spec_hi != 0
                        || cmd.bg_rsrc_spec_hi != 0
                        || cmd.bg_partial_rsrc_spec_hi != 0)
                {
                    return Err(EINVAL);
                }
                counts[RENDER] += 1;
            }
            CMD_COMPUTE => {
                let cmd: uapi::drm_asahi_cmd_compute = payload(entry.payload)?;
                if cmd.flags != 0
                    || cmd.cdm_ctrl_stream_end <= cmd.cdm_ctrl_stream_base
                    || !inside(
                        cmd.cdm_ctrl_stream_base,
                        cmd.cdm_ctrl_stream_end - cmd.cdm_ctrl_stream_base,
                        &range,
                    )
                {
                    return Err(EINVAL);
                }
                // COMPUTE_TIMESTAMP_COPIES is never advertised: the copies must be empty.
                if !timestamps_empty(&cmd.ts_end_copies) {
                    return Err(EINVAL);
                }
                // The G17P scratch layout and intersection tables: never on G15.
                if cmd.scs_layout != 0
                    || cmd.cdm_auxiliary_allocation != 0
                    || cmd.reserved != 0
                    || cmd.intersection_pool != 0
                    || cmd.intersection_pool_bytes != 0
                    || cmd.intersection_first != 0
                    || cmd.intersection_count != 0
                    || cmd.intersection_default_program != 0
                    || cmd.intersection_default_control != 0
                {
                    return Err(EINVAL);
                }
                counts[COMPUTE] += 1;
            }
            _ => return Err(EINVAL),
        }
        if counts[RENDER] + counts[COMPUTE] > MAX_COMMANDS {
            return Err(EINVAL);
        }
    }
    if counts == [0; 2] {
        return Err(EINVAL);
    }
    Ok(())
}

/// The timestamp buffers named by a submission, resolved to firmware arena VAs.
struct Timestamps<'a> {
    objects: Pin<&'a xarray::XArray<KBox<file::Object>>>,
    held: KVec<Arc<mmu::KernelMapping>>,
}

impl Timestamps<'_> {
    /// The arena VA of one requested timestamp, 0 if none: unknown handle ENOENT; offset not
    /// 8-aligned or not 8 bytes inside the bound range EINVAL.
    fn resolve(&mut self, timestamp: &uapi::drm_asahi_timestamp) -> Result<u64> {
        if timestamp.handle == 0 {
            return Ok(0);
        }
        let object = self
            .objects
            .lock()
            .get(timestamp.handle.try_into()?)
            .ok_or(ENOENT)?
            .clone();
        let file::Object::TimestampBuffer(mapping) = object;
        let offset = timestamp.offset as usize;
        if offset % 8 != 0 || offset >= mapping.size() || mapping.size() - offset < 8 {
            return Err(EINVAL);
        }
        let va = mapping.iova() + offset as u64;
        if !self.held.iter().any(|held| Arc::ptr_eq(held, &mapping)) {
            self.held.push(mapping, GFP_KERNEL)?;
        }
        Ok(va)
    }
}

/// Checks that `[start, end)` (rounded out to pages) is user memory mapped with the requested
/// GPU access: a driver range ENOENT, outside the user range EINVAL, an unmapped page ENOENT, a
/// page without the access EACCES.
pub(crate) fn cover(vm: &mmu::Vm, start: u64, end: u64, read: bool, write: bool) -> Result {
    let mask = mmu::UAT_PGMSK as u64;
    if end <= start {
        return Err(EINVAL);
    }
    let first = start & !mask;
    let last = end.checked_add(mask).ok_or(EINVAL)? & !mask;
    // Every G15 driver mapping of a user VM is a client alias inside the fixed reserved ranges
    // (`vm::ClientAlias::check`), so testing those three ranges finds every driver range without
    // walking the alias mappings under the VM's lifetime lock on each span.
    if super::vm::reserved_ranges()
        .iter()
        .any(|reserved| first < reserved.end && reserved.start < last)
    {
        return Err(ENOENT);
    }
    let range = hw::USER_RANGE;
    if first < range.start || last > range.end {
        return Err(EINVAL);
    }
    if vm.covers_range(first, last - first, read, write) {
        Ok(())
    } else if vm.covers_range(first, last - first, false, false) {
        Err(EACCES)
    } else {
        Err(ENOENT)
    }
}

/// Attachment spans, 128-byte granular, actually mapped with `write` or read access.
fn check_attachments(vm: &mmu::Vm, list: &[uapi::drm_asahi_attachment], write: bool) -> Result {
    for attachment in list {
        let end = attachment
            .pointer
            .checked_add(attachment.size)
            .ok_or(EINVAL)?;
        if end > u64::MAX - 127 {
            return Err(EINVAL);
        }
        let first = attachment.pointer & !127;
        let last = (end + 127) & !127;
        if (last - first) / 128 > u64::from(u32::MAX) {
            return Err(EINVAL);
        }
        cover(vm, first, last, !write, write)?;
    }
    Ok(())
}

/// Parses the attachment list of a SET_*_ATTACHMENTS command.
fn attachment_list(bytes: &[u8]) -> Result<KVec<uapi::drm_asahi_attachment>> {
    let mut list = KVec::with_capacity(bytes.len() / ATTACHMENT_SIZE, GFP_KERNEL)?;
    for bytes in bytes.chunks_exact(ATTACHMENT_SIZE) {
        list.push(Reader::new(bytes).read()?, GFP_KERNEL)?;
    }
    Ok(list)
}

fn copy_list(lists: &[&[uapi::drm_asahi_attachment]]) -> Result<KVec<uapi::drm_asahi_attachment>> {
    let mut copy = KVec::with_capacity(lists.iter().map(|list| list.len()).sum(), GFP_KERNEL)?;
    for list in lists {
        for attachment in list.iter() {
            copy.push(*attachment, GFP_KERNEL)?;
        }
    }
    Ok(copy)
}

/// Whether a GVA can be named by a 32-bit compact pointer.
fn compact(gva: u64) -> bool {
    hw::client::COMPACT_WINDOW.contains(&gva)
}

/// TPC and tilemap bytes of a render pass (the encoder computes the full geometry with the same
/// formulas).
fn geometry_bytes(cmd: &uapi::drm_asahi_cmd_render) -> (u64, u64) {
    let tiles_x = u64::from(cmd.width_px).div_ceil(32);
    let tiles_y = u64::from(cmd.height_px).div_ceil(32);
    let blocks_x = tiles_x.div_ceil(4).next_multiple_of(4);
    let blocks_y = tiles_y.div_ceil(4).next_multiple_of(4);
    let sx = blocks_x * if cmd.utile_width_px == 16 { 2 } else { 1 };
    let sy = blocks_y * if cmd.utile_height_px == 16 { 2 } else { 1 };
    let e = (4 * sx) * (4 * sy);
    let layers = u64::from(cmd.layers);
    let tilemap = (5 * e * layers).next_multiple_of(0x1000);
    let tpc = 2 * e * 8 * layers;
    (tpc, tilemap)
}

/// Per-command render admission.
fn check_render(context: &Context<'_>, cmd: &uapi::drm_asahi_cmd_render) -> Result {
    let vm = context.vm;
    let allowed = PROCESS_EMPTY_TILES
        | NO_VERTEX_CLUSTERING
        | RSRC_SPEC_HI
        | DBIAS_IS_INT
        | if super::params::early_tiling() {
            FRAGMENT_BARRIERS
        } else {
            0
        };
    let tile_buffer = u32::from(cmd.sample_size_B)
        * u32::from(cmd.utile_width_px)
        * u32::from(cmd.utile_height_px)
        * u32::from(cmd.samples);
    if cmd.flags & !allowed != 0
        || !(1..=MAX_DIMENSION).contains(&cmd.width_px)
        || !(1..=MAX_DIMENSION).contains(&cmd.height_px)
        || !(1..=MAX_LAYERS).contains(&cmd.layers)
        || !matches!(cmd.samples, 1 | 2 | 4)
        || !matches!(cmd.utile_width_px, 16 | 32)
        || !matches!(cmd.utile_height_px, 16 | 32)
        || cmd.sample_size_B % 8 != 0
        || tile_buffer > MAX_TILE_BUFFER
        || cmd.sampler_heap != 0
        || cmd.sampler_count != 0
        || !helper_is_empty(&cmd.vertex_helper)
        || !helper_is_empty(&cmd.fragment_helper)
        || cmd.ppp_ctrl & !0x203 != 0
    {
        return Err(EINVAL);
    }
    if cmd.stencil.comp_base != 0
        || cmd.stencil.comp_stride != 0
        || (cmd.depth.comp_base != 0 && (cmd.layers != 1 || cmd.samples != 1))
    {
        return Err(Error::from_errno(-(kernel::bindings::EOPNOTSUPP as i32)));
    }
    // The FS work's partial-EOT resource slots are 32 bits wide: a high word could only be
    // dropped.
    if cmd.flags & RSRC_SPEC_HI != 0 && cmd.bg_eot_partial_rsrc_spec_hi != 0 {
        return Err(EINVAL);
    }
    // VDM: compact-addressable and readable; scissor: 8-aligned and readable.
    let vdm = cmd.vdm_ctrl_stream_base;
    let scissor = cmd.isp_scissor_base;
    if vdm & 3 != 0 || !compact(vdm) || scissor == 0 || scissor & 7 != 0 {
        return Err(EINVAL);
    }
    cover(vm, vdm, vdm + 4, true, false)?;
    gate::vdm(vm, vdm)?;
    cover(vm, scissor, scissor.checked_add(8).ok_or(EINVAL)?, true, false)?;
    if cmd.isp_dbias_base != 0 {
        let base = cmd.isp_dbias_base;
        cover(vm, base, base.checked_add(8).ok_or(EINVAL)?, true, false)?;
    }
    if cmd.isp_oclqry_base != 0 {
        let base = cmd.isp_oclqry_base & !15;
        cover(vm, base, base.checked_add(8).ok_or(EINVAL)?, false, true)?;
    }
    for program in [&cmd.bg, &cmd.eot, &cmd.partial_bg, &cmd.partial_eot] {
        if program.usc == 0 {
            continue;
        }
        let address = context
            .usc_base
            .checked_add(u64::from(program.usc & !63))
            .ok_or(EINVAL)?;
        cover(
            vm,
            address,
            address.checked_add(64).ok_or(EINVAL)?,
            true,
            false,
        )?;
    }
    let (tpc, tilemap) = geometry_bytes(cmd);
    if tpc > hw::client::TPC_SIZE || tilemap > hw::client::TILEMAP_SIZE {
        return Err(E2BIG);
    }
    Ok(())
}

/// Per-command compute admission.
fn check_compute(
    context: &Context<'_>,
    cmd: &uapi::drm_asahi_cmd_compute,
    attachments: &[uapi::drm_asahi_attachment],
) -> Result {
    let vm = context.vm;
    let (base, end) = (cmd.cdm_ctrl_stream_base, cmd.cdm_ctrl_stream_end);
    if cmd.flags != 0
        || cmd.sampler_count != 0
        || cmd.sampler_heap != 0
        || !helper_is_empty(&cmd.helper)
        || base & 3 != 0
        || end & 3 != 0
        || end <= base
    {
        return Err(EINVAL);
    }
    cover(vm, base, end, true, false)?;
    gate::cdm(vm, context.usc_base, base)?;
    check_attachments(vm, attachments, true)
}

/// Resolves the timestamps and EXACT_PRIOR ordinals of a structurally valid stream
/// ([`check_stream`]) for a queue that accepted `accepted` render and compute commands before
/// this ioctl.
pub(crate) fn prepare<'a>(
    bytes: &'a [u8],
    accepted: [u64; 2],
    objects: Pin<&xarray::XArray<KBox<file::Object>>>,
) -> Result<Prepared<'a>> {
    // Timestamps of every command first.
    let mut timestamps = Timestamps {
        objects,
        held: KVec::new(),
    };
    let mut resolved: KVec<[u64; 4]> = KVec::new();
    for entry in stream(bytes) {
        let entry = entry?;
        let slots = match entry.kind {
            CMD_RENDER => {
                let cmd: uapi::drm_asahi_cmd_render = payload(entry.payload)?;
                [
                    timestamps.resolve(&cmd.ts_vtx.start)?,
                    timestamps.resolve(&cmd.ts_vtx.end)?,
                    timestamps.resolve(&cmd.ts_frag.start)?,
                    timestamps.resolve(&cmd.ts_frag.end)?,
                ]
            }
            CMD_COMPUTE => {
                let cmd: uapi::drm_asahi_cmd_compute = payload(entry.payload)?;
                [
                    timestamps.resolve(&cmd.ts.start)?,
                    timestamps.resolve(&cmd.ts.end)?,
                    0,
                    0,
                ]
            }
            _ => continue,
        };
        resolved.push(slots, GFP_KERNEL)?;
    }

    // Historical priors and header barriers.
    let mut preceding = [0u16; 2];
    for entry in stream(bytes) {
        let entry = entry?;
        let (engine, prior) = match entry.kind {
            CMD_RENDER => {
                let cmd: uapi::drm_asahi_cmd_render = payload(entry.payload)?;
                (RENDER, [cmd.prior_vdm, cmd.prior_cdm])
            }
            CMD_COMPUTE => {
                let cmd: uapi::drm_asahi_cmd_compute = payload(entry.payload)?;
                (COMPUTE, [cmd.prior_vdm, cmd.prior_cdm])
            }
            _ => continue,
        };
        check_order(accepted, preceding, entry.barriers, prior)?;
        preceding[engine] += 1;
    }

    Ok(Prepared {
        bytes,
        resolved,
        counts: [u64::from(preceding[RENDER]), u64::from(preceding[COMPUTE])],
        timestamps: timestamps.held,
    })
}

impl Prepared<'_> {
    /// Checks each command's spans, programs and stream gates, with a job reference on the VM
    /// already held so that the checked mappings are retained until the job retires.
    pub(crate) fn validate(self, context: &Context<'_>) -> Result<Submission> {
        let Prepared {
            bytes,
            resolved,
            counts,
            timestamps,
        } = self;
        let mut commands = KVec::with_capacity(resolved.len(), GFP_KERNEL)?;
        let mut compute = KVec::new();
        let mut index = 0;
        for entry in stream(bytes) {
            let entry = entry?;
            match entry.kind {
                SET_VERTEX_ATTACHMENTS | SET_FRAGMENT_ATTACHMENTS => {
                    // Validated readable, never serialized on G15.
                    check_attachments(context.vm, &attachment_list(entry.payload)?, false)?;
                }
                SET_COMPUTE_ATTACHMENTS => compute = attachment_list(entry.payload)?,
                CMD_RENDER => {
                    let cmd: uapi::drm_asahi_cmd_render = payload(entry.payload)?;
                    check_render(context, &cmd)?;
                    commands.push(
                        Command::Render(KBox::new(
                            RenderCommand {
                                cmd,
                                timestamps: *resolved.get(index).ok_or(EINVAL)?,
                            },
                            GFP_KERNEL,
                        )?),
                        GFP_KERNEL,
                    )?;
                    index += 1;
                }
                CMD_COMPUTE => {
                    let cmd: uapi::drm_asahi_cmd_compute = payload(entry.payload)?;
                    check_compute(context, &cmd, &compute)?;
                    let slots = resolved.get(index).ok_or(EINVAL)?;
                    commands.push(
                        Command::Compute(KBox::new(
                            ComputeCommand {
                                cmd,
                                attachments: copy_list(&[&compute])?,
                                timestamps: [slots[0], slots[1]],
                            },
                            GFP_KERNEL,
                        )?),
                        GFP_KERNEL,
                    )?;
                    index += 1;
                }
                _ => return Err(EINVAL),
            }
        }
        if commands.is_empty() {
            return Err(EINVAL);
        }
        Ok(Submission {
            counts,
            commands,
            timestamps,
        })
    }
}
