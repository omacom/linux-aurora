// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Work-image and microsequence encoders.
//!
//! Stamps for ordinal n start from the seeds compute 0xc1000000, TA 0x7a000000 and FS
//! 0x3d000000. The compute image sets the compute fault mask at start +0x30 = 1 and interior
//! +0x804 = 1. [`Geometry`] holds the tiling geometry and register words derived from
//! `drm_asahi_cmd_render`; [`ClusterLayout`] the 64 segments and M1..M4 of clustered tiling.
//! Program addresses are `usc_base + (usc & !63)`; resource words take optional high 32 bits
//! (G15S resource words are 64-bit). The TA image sets register #33 PBREQ = initial PB size and
//! interior +0x880 = 1; the FS image carries the partial background/EOT programs for partial
//! renders (the partial-EOT resource word is 32-bit). The scene and InitBM records prefix a
//! render.
//!
//! Pure byte builders over [`super::fw::work`] layouts: no locks, no allocation, no
//! publication. Every address an image names comes in through [`ComputeRefs`], [`RenderRefs`]
//! and [`PassRefs`], resolved by the engine from its firmware objects; the firmware writes
//! timestamps in ns directly through the timestamp arena VAs. The words are written at literal
//! offsets in image order. This file only depends on `core`, the UAPI and the `hw`/`fw::work`
//! constants.

use kernel::uapi;

use super::fw::work::{self, barrier, compute, fs, initbm, scene, ta, tail, ts_record, REG_RECORD};
use super::hw;

/// Stores `v` little-endian at `off`. Every offset is a literal inside a fixed-size image; an
/// out-of-range store is dropped rather than panicking.
#[inline(always)]
fn put(b: &mut [u8], off: usize, v: &[u8]) {
    if let Some(dst) = b.get_mut(off..off + v.len()) {
        dst.copy_from_slice(v);
    }
}

#[inline(always)]
fn put8(b: &mut [u8], off: usize, v: u8) {
    put(b, off, &[v]);
}

#[inline(always)]
fn put16(b: &mut [u8], off: usize, v: u16) {
    put(b, off, &v.to_le_bytes());
}

#[inline(always)]
fn put32(b: &mut [u8], off: usize, v: u32) {
    put(b, off, &v.to_le_bytes());
}

#[inline(always)]
fn put64(b: &mut [u8], off: usize, v: u64) {
    put(b, off, &v.to_le_bytes());
}

/// `DIV_ROUND_UP`.
const fn div_round_up(n: u64, d: u64) -> u64 {
    n.div_ceil(d)
}

/// `round_up` / `ALIGN` to a power of two, wrapping on overflow (no panic).
const fn align(n: u64, a: u64) -> u64 {
    n.wrapping_add(a - 1) & !(a - 1)
}

/// Firmware stamp value of ordinal `n` on the queue with this seed.
/// Stamps step by 0x100 through 0xffffff values and never read 0.
pub(crate) const fn stamp_seed(seed: u32, n: u64) -> u32 {
    let base = (seed / 0x100) as u64;
    ((((base - 1 + n % 0xffffff) % 0xffffff) + 1) * 0x100) as u32
}

/// Compute stamp of ordinal `n`.
pub(crate) const fn stamp(n: u64) -> u32 {
    stamp_seed(hw::engine::COMPUTE_SEED, n)
}

/// 32-bit compact pointer of a GVA in the compact window.
pub(crate) const fn compact(gva: u64) -> u64 {
    gva.wrapping_sub(0x1000000000)
}

/// Addresses the compute encoder names: compute graph and compute slot buffers.
#[derive(Copy, Clone, Default)]
pub(crate) struct ComputeRefs {
    /// HardwareData (firmware VA): the activity records the sequence names.
    pub(crate) hwdata: u64,
    /// Graph: notifier, queue info, user/firmware stamps, UMA manager (firmware VAs).
    pub(crate) notifier: u64,
    pub(crate) qinfo: u64,
    pub(crate) user_stamp: u64,
    pub(crate) fw_stamp: u64,
    pub(crate) manager: u64,
    /// Slot: work image (firmware VA and client GVA of the payload), sequence, preemption
    /// buffer (client GVA), internal timestamp cells, tail and sequence scratch (firmware VAs).
    pub(crate) work: u64,
    pub(crate) work_gva: u64,
    pub(crate) seq: u64,
    pub(crate) preempt_gva: u64,
    pub(crate) its: u64,
    pub(crate) tail: u64,
    pub(crate) scratch: [u64; 5],
}

/// Encodes compute command `c` as ordinal `n`. The engine copies the result into the slot
/// after zeroing the slot's images.
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_compute(
    r: &ComputeRefs,
    regs_cfg: &hw::ComputeRegs,
    c: &uapi::drm_asahi_cmd_compute,
    att: &[uapi::drm_asahi_attachment],
    ts_va: &[u64; 2],
    ctx: u32,
    n: u64,
    fault_mask: u32,
    w: &mut [u8; compute::WORK_SIZE],
    q: &mut [u8; compute::SEQ_SIZE],
) {
    let (wv, qv, h, pre) = (r.work, r.qinfo, r.hwdata, r.preempt_gva);
    let t = stamp(n);
    let att_count = att.len().min(hw::sizes::MAX_ATTACHMENTS);
    // The saved-state pointers into the preemption buffer and the two 0x1c words; a SoC
    // encoded "fresh" carries zero in all six, as a first command does.
    let (saved, word) = if regs_cfg.fresh {
        (0, 0)
    } else {
        (pre + regs_cfg.tail, 0x1c)
    };
    let step = |i: u64| if saved == 0 { 0 } else { saved + 8 * i };
    let val: [u64; 22] = [
        pre,
        c.cdm_ctrl_stream_base,
        step(0),
        step(1),
        step(2),
        step(3),
        0x154024201,
        0x10c08860,
        word,
        word,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        regs_cfg.reg_1a0e9,
        0xff0000,
        regs_cfg.ptc,
        regs_cfg.dupm,
    ];

    w.fill(0);
    q.fill(0);

    // Work image.
    put32(w, 0x0, work::work_type::COMPUTE);
    put64(w, 0x4, (n as u32).wrapping_sub(1) as u64);
    put32(w, 0x10, ctx);
    put64(w, 0x14, r.notifier);
    regs(&mut w[compute::REGS..], &compute::SEL, &val);
    put64(w, 0x720, r.work_gva + compute::REGS as u64);
    put16(w, 0x728, 22);
    put16(w, 0x72a, (22 * REG_RECORD) as u16);
    put64(w, 0x760, r.seq);
    put32(w, 0x768, compute::SEQ_SIZE as u32);
    put64(w, 0x794, pre);
    put64(w, 0x79c, c.cdm_ctrl_stream_end.wrapping_sub(4));
    put64(w, 0x7c4, 0x154024201);
    put32(w, 0x7e0, 0xffffffff);
    put64(w, 0x7e8, r.user_stamp);
    put64(w, 0x7f0, r.fw_stamp);
    put32(w, 0x7f8, t);
    put32(w, 0x7fc, hw::engine::EVENT_SLOT);
    put32(w, 0x804, 1);
    put32(w, 0x808, hw::engine::COMPUTE_UUID);
    put32(w, 0x80c, (n as u32).wrapping_sub(1));
    put64(w, 0x818, r.its);
    put64(w, 0x820, r.its + 8);
    put64(w, 0x828, ts_va[0]);
    put64(w, 0x830, ts_va[1]);
    put64(w, 0x83e, r.manager);
    put8(w, 0x846, 1);
    put64(w, 0x847, 0xad0000);
    put64(w, 0x84f, 0x1600000);
    put64(w, 0x857, r.tail);
    put8(w, 0x85f, 1);

    // Microsequence: start.
    put32(q, 0x0, work::op::COMPUTE_START);
    put64(q, 0x14, wv + 0x20);
    put64(q, 0x1c, h + compute::HWDATA);
    put64(q, 0x24, qv);
    put32(q, 0x2c, ctx);
    put32(q, 0x30, if fault_mask != 0 { 1 } else { 0 });
    put32(q, 0x34, 0);
    put64(q, 0x44, wv + 0x76c);
    put32(q, 0x50, hw::engine::COMPUTE_UUID);
    for (a, at) in att.iter().take(att_count).enumerate() {
        let base = at.pointer & !127;
        let end = align(at.pointer.wrapping_add(at.size), 128);
        let rec = compute::ATTACHMENTS + compute::ATTACHMENT_STRIDE * a;

        put64(q, rec, base);
        put32(q, rec + 8, (end.wrapping_sub(base) / 128) as u32);
        put16(q, rec + 0xc, 0x17);
        put16(q, rec + 0xe, 1);
    }
    put32(q, 0x154, att_count as u32);
    put64(q, 0x15c, r.manager);
    put64(q, 0x174, r.scratch[0]);
    put64(q, 0x184, r.scratch[1]);
    put64(q, 0x18c, r.scratch[2]);
    put64(q, 0x194, r.scratch[3]);
    put64(q, 0x19c, r.scratch[4]);
    put64(q, 0x1a4, wv + 0x880);
    put64(q, 0x1b4, r.notifier + 0xa8);
    // Start and end timestamp records: the 0x3c record over the cells at W + 0x810. Both
    // records name the user pair base when any output is held.
    let requested = ts_va[0] != 0 || ts_va[1] != 0;
    for t_end in [false, true] {
        let at = if t_end {
            compute::TS_END
        } else {
            compute::TS_START
        };
        ts_record(
            &mut q[at..],
            t_end,
            wv,
            compute::TS_CELLS,
            qv,
            requested,
            hw::engine::COMPUTE_UUID,
        );
    }
    put32(q, 0x1f8, 1);
    // Finalize and END.
    let f = compute::FINALIZE;
    put32(q, f, work::op::COMPUTE_FINALIZE);
    put64(q, f + 0x4, h + compute::HWDATA);
    put64(q, f + 0xc, qv);
    put32(q, f + 0x14, ctx);
    put64(q, f + 0x18, wv + 0x76c);
    put32(q, f + 0x24, hw::engine::COMPUTE_UUID);
    put64(q, f + 0x28, r.fw_stamp);
    put32(q, f + 0x30, t);
    put64(q, f + 0x58, wv + 0x83e);
    put32(q, f + 0x60, 0u32.wrapping_sub(f as u32));
    put8(q, f + 0x64, if att_count != 0 { 1 } else { 0 });
    put64(q, f + 0x65, wv + 0x880);
    put64(q, f + 0x6d, wv + 0x860);
    put32(q, compute::END, work::op::END);
}

/// Tiling geometry and register words of a render (accelerator count 2).
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct Geometry {
    pub(crate) tx: u32,
    pub(crate) ty: u32,
    pub(crate) bx: u32,
    pub(crate) by: u32,
    pub(crate) sx: u32,
    pub(crate) sy: u32,
    pub(crate) pixels: u32,
    pub(crate) screen: u32,
    pub(crate) blocks: u32,
    pub(crate) tiles: u32,
    pub(crate) region_stride: u32,
    pub(crate) tpc_stride: u32,
    pub(crate) isp_blocks: u32,
    pub(crate) utile_config: u32,
    pub(crate) layer_mode: u32,
    pub(crate) tilebuffer_control: u32,
    pub(crate) tile_mode: u32,
    pub(crate) isp_control: u32,
    pub(crate) e: u64,
    pub(crate) x_blocks: u64,
    pub(crate) y_blocks: u64,
    pub(crate) tilemap_bytes: u64,
    pub(crate) tpc_bytes: u64,
    pub(crate) layer_metadata: u64,
    pub(crate) native_tiles: u64,
}

impl Geometry {
    /// The geometry of render `u` with layer metadata at `heap`. The render was validated
    /// (width, height, layers >= 1, samples 1, 2 or 4, utiles 16 or 32).
    pub(crate) fn new(u: &uapi::drm_asahi_cmd_render, heap: u64) -> Self {
        let (w, h, l) = (u.width_px as u32, u.height_px as u32, u.layers as u32);
        let empty =
            u.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES != 0;
        let log2s = (u.samples as u32).max(1).ilog2();
        let mut g = Geometry::default();

        g.tx = div_round_up(w as u64, 32) as u32;
        g.ty = div_round_up(h as u64, 32) as u32;
        g.bx = align(div_round_up(g.tx as u64, 4), 4) as u32;
        g.by = align(div_round_up(g.ty as u64, 4), 4) as u32;
        g.sx = g.bx * if u.utile_width_px == 16 { 2 } else { 1 };
        g.sy = g.by * if u.utile_height_px == 16 { 2 } else { 1 };
        g.e = (4 * g.sx as u64) * (4 * g.sy as u64);
        g.pixels = w.wrapping_sub(1) | h.wrapping_sub(1) << 16;
        g.screen = g.tx.wrapping_sub(1) | g.ty.wrapping_sub(1) << 12;
        g.x_blocks = (g.bx as u64) << 10 | (g.bx as u64) << 18 | 3 * g.bx as u64;
        g.y_blocks = (g.by as u64) << 18 | (g.by as u64) << 10 | 3 * g.by as u64;
        g.blocks = g.bx * g.by;
        g.tiles = g.tx * g.ty;
        g.region_stride = (5 * g.e / 64) as u32;
        g.tpc_stride = (g.e / 8) as u32;
        g.isp_blocks = ((g.sx & 0x3fd) << 16) | (g.sy & 0x3fd);
        let utile_w = if u.utile_width_px == 32 {
            0x2000
        } else {
            0x1000
        };
        let utile_h = if u.utile_height_px == 32 {
            0x8000
        } else {
            0x4000
        };
        g.utile_config = utile_w | utile_h | log2s;
        g.tilemap_bytes = align(5 * g.e * l as u64, 0x1000);
        g.tpc_bytes = 2 * g.e * 8 * l as u64;
        g.layer_metadata = heap + if l > 1 { 0x200 } else { 0 };
        g.layer_mode = if l == 1 {
            0x8000
        } else {
            0xc000 | (if empty { 0x2000 } else { 0x1000 }) | l.wrapping_sub(1)
        };
        g.tilebuffer_control = div_round_up(
            u.sample_size_B as u64
                * u.utile_width_px as u64
                * u.utile_height_px as u64
                * u.samples as u64,
            2048,
        ) as u32
            | log2s << 17;
        g.tile_mode = 0x280 | (l > 1) as u32 | if empty { 1 << 16 } else { 0 };
        g.isp_control = 0xc000 | (u.flags & (1 << 18));
        g.native_tiles = 221201u64
            | ((l > 1) as u64) << 32
            | (g.tx.wrapping_sub(1) as u64) << 44
            | (g.ty.wrapping_sub(1) as u64) << 53
            | 137438953472u64
            | ((g.utile_config & 0xf000) as u64) << 28;
        g
    }
}

/// Clustered tiling layout from `base`: 64 segments of 5E bytes, then M2 (0xc00), M3 (0xa00),
/// M4 ((Q + 1) x 512 for Q = pages of the owned PB / 0x1800, rounded up) and the 128-aligned
/// 0x80-byte M1.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub(crate) struct ClusterLayout {
    pub(crate) ct: u64,
    pub(crate) m1: u64,
    pub(crate) m2: u64,
    pub(crate) m3: u64,
    pub(crate) m4: u64,
    pub(crate) bytes: u64,
}

impl ClusterLayout {
    pub(crate) fn new(geo: &Geometry, base: u64) -> Self {
        let q = div_round_up(div_round_up(hw::client::PB_HEAP_SIZE, 0x8000), 0x1800);
        let ct = base;
        let m2 = base + align(64 * 5 * geo.e, 32);
        let m3 = m2 + 0xc00;
        let m4 = m3 + 0xa00;
        let m1 = base + align(m4 - base + (q + 1) * 8 * 64, 128);
        ClusterLayout {
            ct,
            m1,
            m2,
            m3,
            m4,
            bytes: m1 + 0x80 - base,
        }
    }
}

/// Clustered tiling eligibility: one layer, one sample, 32x32 utiles, unless the submission
/// asked for single-cluster vertex work. The engine then also needs a clustering buffer that
/// holds [`ClusterLayout::bytes`].
pub(crate) fn cluster_eligible(u: &uapi::drm_asahi_cmd_render, geo: &Geometry) -> bool {
    hw::tuning::VERTEX_CLUSTERING
        && u.layers == 1
        && geo.utile_config == 0xa000
        && u.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_NO_VERTEX_CLUSTERING == 0
}

/// Program address.
pub(crate) fn prog_addr(usc_base: u64, p: &uapi::drm_asahi_bg_eot) -> u64 {
    usc_base.wrapping_add((p.usc & !63) as u64)
}

/// Program resource word, 64-bit with RSRC_SPEC_HI.
pub(crate) fn prog_rsrc(
    u: &uapi::drm_asahi_cmd_render,
    p: &uapi::drm_asahi_bg_eot,
    hi: u32,
) -> u64 {
    if u.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_RSRC_SPEC_HI != 0 {
        (hi as u64) << 32 | p.rsrc_spec as u64
    } else {
        p.rsrc_spec as u64
    }
}

/// Register records.
pub(crate) fn regs(dst: &mut [u8], sel: &[u32], val: &[u64]) {
    for (i, (s, v)) in sel.iter().zip(val.iter()).enumerate() {
        put32(dst, REG_RECORD * i, *s);
        put64(dst, REG_RECORD * i + 4, *v);
    }
}

/// The 0x3c timestamp record.
pub(crate) fn ts_record(
    rec: &mut [u8],
    end: bool,
    w: u64,
    b: u64,
    q: u64,
    requested: bool,
    uuid: u32,
) {
    let rec = match rec.get_mut(..ts_record::SIZE) {
        Some(rec) => rec,
        None => return,
    };
    put32(
        rec,
        0x0,
        if end {
            work::op::TIMESTAMP_END
        } else {
            work::op::TIMESTAMP
        },
    );
    put64(rec, 0x4, w + b);
    put64(rec, 0xc, w + b + 8);
    put64(rec, 0x14, w + b + if end { 0x10 } else { 8 });
    put64(rec, 0x1c, q);
    put64(rec, 0x24, if requested { w + b + 0x18 } else { 0 });
    put64(rec, 0x2c, w + b + 0x58);
    put32(rec, 0x34, uuid);
}

/// Shared work tail (TA +0x8a8, FS +0xbf0); mirrors the compute tail.
pub(crate) fn work_tail(
    t: &mut [u8],
    its_start: u64,
    its_end: u64,
    user_start: u64,
    user_end: u64,
    uma_mgr: u64,
    scratch: u64,
) {
    put64(t, tail::ITS_START, its_start);
    put64(t, tail::ITS_END, its_end);
    put64(t, tail::USER_START, user_start);
    put64(t, tail::USER_END, user_end);
    put64(t, tail::UMA_MANAGER, uma_mgr);
    put8(t, 0x36, 1);
    put64(t, 0x37, 0xad0000);
    put64(t, 0x3f, 0x1600000);
    put64(t, tail::SCRATCH, scratch);
    put8(t, 0x4f, 1);
}

/// A barrier record waiting until the stamp at `stamp_va` reaches `wait`:
/// `event` names the producing engine (0 TA, 1 FS, 2 compute), `own` is the waiter's stamp.
pub(crate) fn barrier(
    stamp_va: u64,
    wait: u32,
    event: u32,
    own: u32,
    uuid: u32,
    ty: work::BarrierType,
) -> [u8; barrier::SIZE] {
    let mut x = [0u8; barrier::SIZE];

    put32(&mut x, 0x0, work::op::BARRIER);
    put64(&mut x, barrier::STAMP_VA, stamp_va);
    put64(&mut x, barrier::STAMP_VA2, stamp_va);
    put32(&mut x, barrier::WAIT, wait);
    put32(&mut x, barrier::EVENT, event);
    put32(&mut x, barrier::OWN, own);
    put32(&mut x, barrier::UUID, uuid);
    put32(&mut x, barrier::TYPE, ty as u32);
    x
}

/// Addresses the render encoders name that belong to the render graph and its allocators
/// (the UMA/PB objects are in `Pools`).
#[derive(Copy, Clone, Default)]
pub(crate) struct RenderRefs {
    /// HardwareData (firmware VA): the activity records the sequences name.
    pub(crate) hwdata: u64,
    pub(crate) notifier: u64,
    pub(crate) legacy_empty: u64,
    /// Queue info, user and firmware stamps of the TA (0) and FS (1) queues.
    pub(crate) qinfo: [u64; 2],
    pub(crate) user_stamp: [u64; 2],
    pub(crate) fw_stamp: [u64; 2],
    /// Allocators in use (after a rebuild possibly the spare set): UMA managers of TA and FS,
    /// PB manager, scene list, misc record; and the initial PB blocks.
    pub(crate) uma_mgr: [u64; 2],
    pub(crate) pb_mgr: u64,
    pub(crate) scene_list: u64,
    pub(crate) mgr_misc: u64,
    pub(crate) pb_init_blocks: u32,
}

/// Addresses of one pass slot.
#[derive(Copy, Clone, Default)]
pub(crate) struct PassRefs {
    /// Work images: firmware VA and client GVA of the payload.
    pub(crate) ta_work: u64,
    pub(crate) ta_work_gva: u64,
    pub(crate) fs_work: u64,
    pub(crate) fs_work_gva: u64,
    pub(crate) ta_seq: u64,
    pub(crate) fs_seq: u64,
    pub(crate) scene: u64,
    /// Client GVAs.
    pub(crate) tpc_gva: u64,
    pub(crate) tilemap_gva: u64,
    pub(crate) heap_gva: u64,
    pub(crate) scene_user_gva: u64,
    pub(crate) pre_gva: [u64; 3],
    pub(crate) fs_scratch_gva: u64,
    pub(crate) aux_gva: u64,
    /// Firmware VAs.
    pub(crate) its: u64,
    pub(crate) ta_scratch: u64,
}

/// Encodes the TA work image and microsequence of a render.
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_ta(
    r: &RenderRefs,
    p: &PassRefs,
    u: &uapi::drm_asahi_cmd_render,
    ts_va: &[u64; 4],
    ctx: u32,
    tta: u32,
    geo: &Geometry,
    cl: Option<&ClusterLayout>,
    w: &mut [u8; ta::WORK_SIZE],
    q: &mut [u8; ta::SEQ_SIZE],
) {
    let (wv, uv, qv, h) = (p.ta_work, p.ta_seq, r.qinfo[0], r.hwdata);
    let (heap, su) = (p.heap_gva, p.scene_user_gva);
    let ts = ts_va[0] != 0 || ts_va[1] != 0;
    let mut val: [u64; 72] = [
        1,
        0x200,
        compact(p.tilemap_gva),
        compact(p.tilemap_gva),
        0,
        0,
        compact(p.tpc_gva),
        compact(heap) | 1 << 63,
        compact(heap) | 1 << 63,
        0x003a0012006b0003,
        1,
        geo.utile_config as u64,
        u.ppp_multisamplectl,
        compact(p.pre_gva[0]),
        compact(p.pre_gva[0]),
        compact(p.pre_gva[1]),
        compact(p.pre_gva[1]),
        1,
        compact(p.pre_gva[2]) | 1 << 50,
        0,
        compact(u.vdm_ctrl_stream_base),
        0,
        0,
        0,
        1,
        0,
        compact(geo.layer_metadata),
        compact(geo.layer_metadata),
        0,
        0,
        0,
        0,
        4 * r.pb_init_blocks as u64,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0x8860,
        geo.region_stride as u64,
        geo.region_stride as u64,
        0x88,
        (2 | (u.ppp_ctrl & 0x201)) as u64,
        geo.pixels as u64,
        geo.screen as u64,
        geo.x_blocks,
        geo.y_blocks,
        geo.blocks as u64,
        geo.tpc_stride as u64,
        0x100,
        geo.layer_mode as u64,
        0x1c,
        0x1c,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        8,
        0xff0000,
        0x7f00400020,
        0x200000001,
        0,
        compact(su + 0x40),
        compact(su + 0x40),
        0xa000000000 | ((r.scene_list + 4) & 0xfffffffff) | 1,
    ];

    if let Some(cl) = cl {
        // The ten clustered TA values, compact form.
        val[4] = compact(cl.ct);
        val[5] = compact(cl.ct);
        val[22] = compact(cl.m2);
        val[23] = compact(cl.m3);
        val[24] = 0x200440;
        val[25] = 4;
        val[30] = compact(cl.m4);
        val[31] = compact(cl.m4);
        val[ta::REG_PBREQ] = hw::sizes::CLUSTER_PB_PAGES as u64;
        val[33] = compact(cl.m1);
    }

    w.fill(0);
    q.fill(0);

    // TA work image.
    put32(w, 0x0, work::work_type::TA);
    put64(w, 0x4, 1);
    put32(w, 0xc, ctx);
    put64(w, 0x14, r.notifier);
    put64(w, 0x24, r.pb_mgr);
    put64(w, 0x2c, p.scene);
    put64(w, 0x34, r.legacy_empty);
    regs(&mut w[ta::REGS..], &ta::SEL, &val);
    put64(w, 0x740, p.ta_work_gva + ta::REGS as u64);
    put16(w, 0x748, 72);
    put16(w, 0x74a, (72 * REG_RECORD) as u16);
    put64(w, 0x760, p.tpc_gva);
    put64(w, 0x768, geo.tpc_bytes);
    put64(w, 0x770, uv);
    put32(w, 0x778, ta::SEQ_SIZE as u32);
    put32(w, 0x77c, 1);
    put32(w, 0x780, tta);
    put64(w, 0x7a8, compact(p.pre_gva[0]));
    put32(w, 0x840, 0x67);
    put32(w, 0x848, 0xffffffff);
    put32(w, 0x854, 1);
    put64(w, 0x864, r.user_stamp[0]);
    put64(w, 0x86c, r.fw_stamp[0]);
    put32(w, 0x874, tta);
    put32(w, 0x880, 1);
    put32(w, 0x884, hw::engine::TA_UUID);
    work_tail(
        &mut w[ta::TAIL..],
        p.its,
        p.its + 8,
        ts_va[0],
        ts_va[1],
        r.uma_mgr[0],
        p.ta_scratch,
    );

    // TA microsequence.
    put32(q, 0x0, work::op::TA_START);
    put64(q, 0x14, wv + 0x40);
    put64(q, 0x1c, r.pb_mgr);
    put64(q, 0x24, p.scene);
    put64(q, 0x2c, h + ta::HWDATA);
    put64(q, 0x34, qv);
    put64(q, 0x3c, wv + 0x84c);
    put32(q, 0x44, ctx);
    put64(q, 0x64, wv + 0x784);
    put64(q, 0x6c, wv + 0x890);
    put64(q, 0x78, (hw::engine::TA_UUID as u64) << 32);
    put64(q, 0x18c, r.uma_mgr[0]);
    put64(q, 0x1a4, p.ta_scratch);
    put64(q, 0x1b4, 1);
    put64(q, 0x1bc, r.notifier + 0xa8);
    ts_record(
        &mut q[ta::TS_START..],
        false,
        wv,
        ta::TAIL as u64,
        qv,
        ts,
        hw::engine::TA_UUID,
    );
    put32(q, 0x208, 1);
    ts_record(
        &mut q[ta::TS_END..],
        true,
        wv,
        ta::TAIL as u64,
        qv,
        ts,
        hw::engine::TA_UUID,
    );
    let f = ta::FINALIZE;
    put32(q, f, work::op::TA_FINALIZE);
    put64(q, f + 0x4, p.scene);
    put64(q, f + 0xc, r.pb_mgr);
    put64(q, f + 0x14, h + ta::HWDATA);
    put64(q, f + 0x1c, qv);
    put64(q, f + 0x24, wv + 0x84c);
    put32(q, f + 0x2c, ctx);
    put64(q, f + 0x34, wv + 0x784);
    put32(q, f + 0x40, hw::engine::TA_UUID);
    put64(q, f + 0x44, r.fw_stamp[0]);
    put32(q, f + 0x4c, tta);
    put64(q, f + 0x74, uv + 0x18c);
    put32(q, f + 0x7c, 0u32.wrapping_sub(f as u32));
    put64(q, f + 0x84, wv + 0x8f8);
    put32(q, ta::END, work::op::END);
}

/// Encodes the FS work image and microsequence of a render.
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_fs(
    r: &RenderRefs,
    p: &PassRefs,
    u: &uapi::drm_asahi_cmd_render,
    ts_va: &[u64; 4],
    ctx: u32,
    usc: u64,
    n: u64,
    tfs: u32,
    geo: &Geometry,
    w: &mut [u8; fs::WORK_SIZE],
    q: &mut [u8; fs::SEQ_SIZE],
) {
    let (wv, uv, qv, h) = (p.fs_work, p.fs_seq, r.qinfo[1], r.hwdata);
    let (heap, su) = (p.heap_gva, p.scene_user_gva);
    let ts = ts_va[2] != 0 || ts_va[3] != 0;
    let (d, s) = (&u.depth, &u.stencil);
    let val: [u64; 79] = [
        1,
        geo.utile_config as u64,
        prog_rsrc(u, &u.eot, u.bg_eot_rsrc_spec_hi),
        prog_addr(usc, &u.eot),
        prog_rsrc(u, &u.bg, u.bg_rsrc_spec_hi),
        prog_addr(usc, &u.bg),
        u.isp_merge_upper_x as u64,
        u.isp_merge_upper_y as u64,
        0,
        0,
        0,
        0,
        u.ppp_multisamplectl,
        geo.isp_blocks as u64,
        geo.isp_blocks as u64,
        geo.screen as u64,
        0xff0200,
        heap,
        u.isp_scissor_base,
        u.isp_dbias_base,
        geo.isp_control as u64,
        (u.height_px as u64) << 32 | u.width_px as u64,
        0x100000,
        geo.tilebuffer_control as u64,
        u.isp_zls_pixels as u64,
        u.isp_bgobjdepth as u64,
        u.isp_bgobjvals as u64,
        u.isp_oclqry_base & !15,
        u.zls_ctrl,
        0x04040404,
        0,
        d.base,
        d.base,
        s.base,
        s.base,
        0,
        0,
        0,
        0,
        d.stride as u64,
        d.stride as u64,
        s.stride as u64,
        s.stride as u64,
        d.comp_base,
        d.comp_stride as u64,
        d.comp_base,
        d.comp_stride as u64,
        s.comp_base,
        s.comp_stride as u64,
        s.comp_base,
        s.comp_stride as u64,
        p.tilemap_gva,
        geo.layer_metadata,
        (4 * geo.region_stride as u64) << 24,
        geo.tile_mode as u64,
        0,
        0,
        0,
        0x8860,
        p.aux_gva,
        p.aux_gva,
        0x1c,
        0x1c,
        0,
        geo.native_tiles,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        8,
        0,
        compact(su + 0x40) & !15,
        0x6000400020,
        0x200000001,
    ];
    let pbg_r = prog_rsrc(u, &u.partial_bg, u.bg_partial_rsrc_spec_hi);
    let pbg_a = prog_addr(usc, &u.partial_bg);
    let peot_a = prog_addr(usc, &u.partial_eot);
    let pp = fs::PARAMS;

    w.fill(0);
    q.fill(0);

    // FS work image.
    put32(w, 0x0, work::work_type::FS);
    put64(w, 0x4, 0);
    put32(w, 0xc, ctx);
    put64(w, 0x14, uv);
    put32(w, 0x1c, fs::SEQ_SIZE as u32);
    put64(w, 0x20, r.notifier);
    put64(w, 0x28, r.pb_mgr);
    put64(w, 0x30, p.scene);
    put64(w, 0x38, r.legacy_empty);
    put64(w, 0x40, p.tilemap_gva);
    put64(w, 0x48, 0x88);
    put32(w, 0x50, 1);
    put16(w, 0x54, (geo.y_blocks >> 18) as u16);
    put16(w, 0x56, (geo.x_blocks >> 18) as u16);
    put32(w, 0x68, u.isp_merge_upper_x);
    put32(w, 0x6c, u.isp_merge_upper_y);
    put64(w, 0x78, geo.tiles as u64);
    regs(&mut w[fs::REGS..], &fs::SEL, &val);
    put64(w, 0x780, p.fs_work_gva + fs::REGS as u64);
    put16(w, 0x788, 79);
    put16(w, 0x78a, (79 * REG_RECORD) as u16);
    // Parameters / firmware mirrors at +0x790.
    put64(w, pp + 0x0, u.isp_dbias_base);
    put64(w, pp + 0x10, u.isp_scissor_base);
    put64(w, pp + 0x20, u.isp_oclqry_base);
    put64(w, pp + 0x168, pbg_r);
    put64(w, pp + 0x170, pbg_a);
    put64(w, pp + 0x198, pbg_r);
    put64(w, pp + 0x1a0, pbg_a);
    put64(w, pp + 0x1a8, u.zls_ctrl);
    put32(w, pp + 0x1b0, 0x04040404);
    put64(w, pp + 0x1b8, d.base);
    put64(w, pp + 0x1c0, d.stride as u64);
    put64(w, pp + 0x1c8, d.comp_stride as u64);
    put64(w, pp + 0x1d0, d.base);
    put64(w, pp + 0x1d8, d.base);
    put64(w, pp + 0x1e0, d.comp_base);
    put64(w, pp + 0x1e8, s.base);
    put64(w, pp + 0x1f0, s.stride as u64);
    put64(w, pp + 0x1f8, s.comp_stride as u64);
    put64(w, pp + 0x200, s.base);
    put64(w, pp + 0x208, s.base);
    put64(w, pp + 0x210, s.comp_base);
    put32(w, pp + 0x228, geo.tilebuffer_control);
    put32(w, pp + 0x230, geo.isp_control);
    put32(w, pp + 0x238, u.width_px as u32);
    put32(w, pp + 0x23c, u.height_px as u32);
    put32(w, pp + 0x240, 0x100000);
    put32(w, pp + 0x248, geo.tile_mode);
    put32(w, pp + 0x264, u.partial_eot.rsrc_spec);
    put64(w, pp + 0x26c, peot_a);
    put32(w, pp + 0x284, u.partial_eot.rsrc_spec);
    put64(w, pp + 0x28c, peot_a);
    put32(w, pp + 0x298, u.isp_bgobjdepth);
    put32(w, pp + 0x29c, u.isp_bgobjvals);
    put32(w, pp + 0x2a0, u.sample_size_B as u32);
    put64(w, pp + 0x2c0, u.isp_zls_pixels as u64);
    put32(w, 0xb88, 0x67);
    put32(w, 0xb90, 0xffffffff);
    put32(
        w,
        0xb94,
        (u.flags & uapi::drm_asahi_render_flags_DRM_ASAHI_RENDER_PROCESS_EMPTY_TILES != 0) as u32,
    );
    put32(w, 0xb98, 1);
    put64(w, 0xbac, r.user_stamp[1]);
    put64(w, 0xbb4, r.fw_stamp[1]);
    put32(w, 0xbbc, tfs);
    put32(w, 0xbc0, 1);
    put32(w, 0xbc8, 1);
    put32(w, 0xbcc, hw::engine::FS_UUID);
    put32(w, 0xbe8, 1);
    work_tail(
        &mut w[fs::TAIL..],
        p.its + 0x10,
        p.its + 0x18,
        ts_va[2],
        ts_va[3],
        r.uma_mgr[1],
        p.fs_scratch_gva,
    );

    // FS microsequence.
    put32(q, 0x0, work::op::FS_START);
    put64(q, 0x14, wv + 0x80);
    put64(q, 0x1c, p.scene);
    put64(q, 0x24, h + fs::HWDATA);
    put64(q, 0x2c, wv + 0xb70);
    put64(q, 0x34, wv + 0xb74);
    put64(q, 0x3c, wv + 0xba0);
    put64(q, 0x44, qv);
    put64(q, 0x4c, wv);
    put32(q, 0x54, ctx);
    put32(q, 0x58, 1);
    put64(q, 0x68, (n as u32).wrapping_sub(1) as u64);
    put64(q, 0x74, wv + 0xa58);
    put64(q, 0x7c, wv + 0xbd8);
    put32(q, 0x98, hw::engine::FS_UUID);
    put64(q, 0x1a4, r.uma_mgr[1]);
    put64(q, 0x1bc, p.fs_scratch_gva);
    put64(q, 0x1d4, r.notifier + 0xa8);
    ts_record(
        &mut q[fs::TS_START..],
        false,
        wv,
        fs::TAIL as u64,
        qv,
        ts,
        hw::engine::FS_UUID,
    );
    put32(q, 0x218, 1);
    ts_record(
        &mut q[fs::TS_END..],
        true,
        wv,
        fs::TAIL as u64,
        qv,
        ts,
        hw::engine::FS_UUID,
    );
    let f = fs::FINALIZE;
    put32(q, f, work::op::FS_FINALIZE);
    put32(q, f + 0x4, hw::engine::FS_UUID);
    put64(q, f + 0xc, r.fw_stamp[1]);
    put32(q, f + 0x14, tfs);
    put64(q, f + 0x1c, p.scene);
    put64(q, f + 0x24, r.pb_mgr);
    put64(q, f + 0x2c, 1);
    put64(q, f + 0x34, h + fs::HWDATA);
    put64(q, f + 0x3c, wv + 0xba0);
    put64(q, f + 0x44, wv + 0xb70);
    put64(q, f + 0x4c, qv);
    put64(q, f + 0x54, wv);
    put64(q, f + 0x5c, ctx as u64);
    put64(q, f + 0x64, wv + 0xa58);
    put64(q, f + 0x9c, uv + 0x1a4);
    put32(q, f + 0xa4, 0u32.wrapping_sub(f as u32));
    put32(q, f + 0xa8, 1);
    put64(q, f + 0xac, wv + 0xc40);
    put32(q, fs::END, work::op::END);
}

/// The pass slot's scene record for draw ordinal `n`.
pub(crate) fn scene(r: &RenderRefs, p: &PassRefs, n: u64) -> [u8; scene::SIZE] {
    let mut x = [0u8; scene::SIZE];
    let list = r.scene_list + 4 * (n % scene::LIST_SLOTS);

    put64(&mut x, 0x0, list);
    put64(&mut x, 0x8, list);
    put64(&mut x, 0x28, compact(p.scene_user_gva + 0x40));
    put32(&mut x, 0x30, compact(p.scene_user_gva + 0x900) as u32);
    put64(&mut x, 0x40, r.mgr_misc);
    x
}

/// The InitBM record that (re)initializes the PB before a render.
pub(crate) fn initbm(ctx: u32, pb_init_blocks: u32, pb_mgr: u64, tta: u32) -> [u8; initbm::SIZE] {
    let mut x = [0u8; initbm::SIZE];

    put32(&mut x, 0x0, work::op::INIT_BM);
    put32(&mut x, initbm::CTX, ctx);
    put32(&mut x, initbm::BLOCKS, pb_init_blocks);
    put64(&mut x, initbm::PB_MANAGER, pb_mgr);
    put32(&mut x, initbm::STAMP, tta);
    x
}
