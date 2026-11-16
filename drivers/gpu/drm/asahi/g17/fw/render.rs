// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Render work descriptors.
//!
//! A render pass is kicked as a tiling (TA) descriptor on one queue and a
//! fragment (3D) descriptor on the queue paired with it. Each descriptor
//! embeds the register arrays its kick entry binds: one for tiling, and for
//! fragment the main array followed by the partial-render store, resume and
//! load arrays.
//!
//! Descriptors are built in place with [`TaDescriptor::write`] and
//! [`FragmentSlot::write`]; the completion link and the fragment MCache table
//! are added just before publication.

pub(crate) mod storage;

use kernel::prelude::*;

use super::kick::{
    KickTimestamp, McacheClass, McacheRange, McacheTable, RegisterArray, RegisterArrayBinding,
    RegisterWrite, WriteRange, MCACHE_INLINE_RANGES,
};
use super::queue::CommandTag;
use super::{
    clear, offset_va, sampler_max, stamp_value, timestamp_end_va, work_key, ObjectIds, UNK_0A599,
    UNK_0D411, UNK_101D9, UNK_1C8F8,
};

/// Size of a tiling descriptor.
pub(crate) const TA_DESCRIPTOR_SIZE: usize = 0x9c0;
/// Size of a [`FragmentSlot`]: the descriptor and its MCache spill table.
pub(crate) const FRAGMENT_SLOT_SIZE: usize = 0x2380;
/// Write ranges of one fragment pass: its attachments and the auxiliary
/// framebuffer.
pub(crate) const MCACHE_RANGES_MAX: usize = 17;
/// Size of the auxiliary framebuffer of a pass.
pub(crate) const AUX_FB_SIZE: u64 = 0x8000;

/// QoS class of tiling kicks.
pub(crate) const TA_QOS_CLASS: u8 = 0x0c;
/// QoS class of fragment kicks.
pub(crate) const FRAGMENT_QOS_CLASS: u8 = 0x18;
/// Event mask of render kicks.
pub(crate) const RENDER_KICK_EVENT_MASK: [u64; 4] = [0; 4];

/// Base of the user address window. Tiling resource registers hold offsets
/// from it rather than addresses.
const USER_BASE: u64 = 0x10_0000_0000;
/// The VDM control-stream register holds a 39-bit offset from [`USER_BASE`].
const VDM_OFFSET_LIMIT: u64 = 1 << 39;
/// The deflake (context) buffer holds three sub-buffers: at its start and at two offsets
/// that follow the control words of each cluster.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Deflake {
    /// Offsets of the second and third sub-buffers.
    pub(crate) sub: [u64; 2],
    /// The host writes the viewport words at `+0x900`; with more clusters that offset lies in
    /// the third sub-buffer and the buffer is left to the GPU.
    pub(crate) viewport: bool,
}

impl Deflake {
    /// One cluster (A18 Pro).
    pub(crate) const ONE_CLUSTER: Deflake = Deflake {
        sub: [0x20, 0x2a0],
        viewport: true,
    };
    /// Two clusters (M4 Pro).
    pub(crate) const TWO_CLUSTERS: Deflake = Deflake {
        sub: [0xa0, 0x820],
        viewport: false,
    };
}

/// The register programs of a firmware build's render descriptors.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum RegisterPrograms {
    /// The A18 Pro firmware's (26A428).
    G17P,
    /// The 25G83 build's: other tiling and fragment arrays, another tile-state base, and the
    /// tiler parameter sub-buffers in the TPC object.
    V25G83,
}

/// The 25G83 tiling and main fragment arrays.
const TA_REGISTERS_25G83: usize = 83;
const FRAGMENT_REGISTERS_25G83: usize = 91;
const TA_CHECKPOINTS_25G83: [u16; 3] = [81, 83, 83];
const FRAGMENT_CHECKPOINTS_25G83: [u16; 3] = [87, 88, 88];
/// Index of the free-list slot register (0x14308) in the 25G83 tiling array.
const TA_SLOT_INDEX_25G83: u8 = 79;
/// Index of the free-list slot register (0x14048) in the 25G83 fragment array.
const FRAGMENT_SLOT_INDEX_25G83: u8 = 84;
/// A 25G83 stage with shader-context scratch (ray queries) also writes the SCS enable 0x017d9 = 1,
/// which the compute program and both A18 Pro render programs carry for such work: first in the
/// tiling array, after 0x01739 in the fragment array. Every count and index after it moves by one.
const SCS_ENABLE: u32 = 0x017d9;

const TA_REGISTERS: usize = 73;
const TA_REGISTERS_MAX: usize = TA_REGISTERS + 1;
const TA_SCS_ENABLE_INDEX: usize = 1;
const FRAGMENT_REGISTERS: usize = 89;
const FRAGMENT_REGISTERS_MAX: usize = FRAGMENT_REGISTERS + 1;
const FRAGMENT_SCS_ENABLE_INDEX: usize = 1;
const PARTIAL_STORE_REGISTERS: usize = 16;
const PARTIAL_RESUME_REGISTERS: usize = 23;
const PARTIAL_LOAD_REGISTERS: usize = 10;

/// Checkpoints into the tiling array, as entry counts: before the final two
/// entries, then after the last one (twice).
const TA_CHECKPOINTS: [u16; 3] = [
    TA_REGISTERS as u16 - 2,
    TA_REGISTERS as u16,
    TA_REGISTERS as u16,
];
/// Checkpoints into the main fragment array, as entry counts: after the first
/// of the four closing entries, then after the second (twice).
const FRAGMENT_CHECKPOINTS: [u16; 3] = [
    FRAGMENT_REGISTERS as u16 - 3,
    FRAGMENT_REGISTERS as u16 - 2,
    FRAGMENT_REGISTERS as u16 - 2,
];

const TILE_SIZE: u64 = 32;
const MACRO_TILES: u64 = 4;
const REGION_ENTRY_SIZE: u64 = 5;
/// Highest number of samples per pixel, as a power of two.
const SAMPLES_LOG2_MAX: u8 = 2;
/// Bytes of tile buffer per block.
const TIB_BLOCK_SIZE: u64 = 2048;
const TILE_CONFIG_BASE: u64 = 0x280;
const TILE_CONFIG_LAYERED: u64 = 1 << 0;
const TILE_CONFIG_PROCESS_EMPTY_TILES: u64 = 1 << 16;
const AUX_FB_FLAGS_BASE: u64 = 0xc000;
/// The pass rasterizes at most four known primitives (userspace's promise).
const AUX_FB_FLAGS_FEW_PRIMITIVES: u64 = 1 << 0;
const AUX_FB_FLAGS_DBIAS_INT: u64 = 1 << 18;
/// Tiling layer mode: layer count less one, bit 15, and for layered targets
/// bit 14 plus the empty-tile policy. The second copy keeps only the count and
/// bit 14.
const LAYER_MODE_BASE: u64 = 1 << 15;
const LAYER_MODE_LAYERED: u64 = 1 << 14;
const LAYER_MODE_PROCESS_EMPTY: u64 = 1 << 13;
const LAYER_MODE_SKIP_EMPTY: u64 = 1 << 12;
const LAYER_MODE_PIPE_MASK: u64 = 0x47ff;
/// Fragment tile state: required bits 0x3717f and 37, layered targets in bit
/// 32, the utile size bits of the tile-buffer configuration in bits 43:40,
/// tiles in X less one from bit 44 and tiles in Y less one from bit 53.
const TILE_STATE_BASE: u64 = 0x3717f | 1 << 37;
/// The 25G83 build's low tile-state bits, the ones the M4 Pro has run with. With the A18 Pro's,
/// a pass more than 80 tiles wide and with more than 4096 tiles never completes its fragment
/// stage (the 3456x2234 panel is 108 x 70 tiles).
const TILE_STATE_BASE_25G83: u64 = 0x36011 | 1 << 37;
const TILE_STATE_LAYERED: u64 = 1 << 32;
const TILE_STATE_UTILE_SHIFT: u32 = 28;
const TILE_STATE_TILES_X_SHIFT: u32 = 44;
const TILE_STATE_TILES_Y_SHIFT: u32 = 53;
/// Tile-buffer configuration: utile width / 16 in bits 13:12, utile height /
/// 16 in bits 15:14, samples (log2) in bits 1:0.
const UTILE_CONFIG_WIDTH_SHIFT: u32 = 12;
const UTILE_CONFIG_HEIGHT_SHIFT: u32 = 14;
const UTILE_CONFIG_SIZE_MASK: u64 = 0xf000;
/// The partial-render phases load and store every attached depth and stencil
/// surface.
const ZLS_PARTIAL_DEPTH: u64 = 1 << 15 | 1 << 19;
const ZLS_PARTIAL_STENCIL: u64 = 1 << 14 | 1 << 18;
const STENCIL_CLEAR_BASE: u32 = 0x300;
/// Value of the ZLS swizzle registers and descriptor field.
const ZLS_SWIZZLE: u32 = 0x0404_0404;
/// Value of register 0x15359 in the partial store and resume arrays; the
/// main array writes zero.
const UNK_15359_PARTIAL: u64 = 0x20;
/// Required value of register 0x15049 and of fragment descriptor field
/// +0x1f50.
const UNK_15049: u64 = 0x10_0000;
/// USC program offsets carry tag bits that are not part of the address.
const USC_OFFSET_TAG_MASK: u32 = 7;

/// A depth or stencil surface.
#[derive(Copy, Clone, Debug)]
pub(crate) struct ZlsSurface {
    /// GPU address of the surface, or zero when it is not attached.
    pub(crate) va: u64,
    /// GPU address of its compression metadata.
    pub(crate) compression_va: u64,
    /// Stride of the surface.
    pub(crate) stride: u64,
    /// Stride of the compression metadata.
    pub(crate) compression_stride: u64,
}

/// A USC program and the resource specifier it is launched with.
#[derive(Copy, Clone, Debug)]
pub(crate) struct UscProgram {
    /// Tagged offset of the program in the queue's USC window.
    pub(crate) offset: u32,
    /// Resource specifier.
    pub(crate) rsrc_spec: u64,
}

/// Render-pass state from the userspace command.
#[derive(Copy, Clone, Debug)]
pub(crate) struct RenderPass {
    /// Width of the render target in pixels.
    pub(crate) width: u32,
    /// Height of the render target in pixels.
    pub(crate) height: u32,
    /// Number of layers, 1 to 2048.
    pub(crate) layers: u16,
    /// Utile width: 16 or 32.
    pub(crate) utile_width: u8,
    /// Utile height: 16 or 32, at most the width.
    pub(crate) utile_height: u8,
    /// Samples per pixel, as a power of two: 0 to 2.
    pub(crate) samples_log2: u8,
    /// Bytes of tile buffer per sample.
    pub(crate) sample_size: u8,
    /// Process tiles no primitive touches.
    pub(crate) process_empty_tiles: bool,
    /// Multisample control.
    pub(crate) msaa_control: u64,
    /// PPP control.
    pub(crate) ppp_control: u64,
    /// GPU address of the VDM control stream.
    pub(crate) vdm_va: u64,
    /// GPU address of the scissor array.
    pub(crate) scissor_va: u64,
    /// GPU address of the depth-bias array, or zero.
    pub(crate) depth_bias_va: u64,
    /// Depth-bias values are integers.
    pub(crate) depth_bias_is_int: bool,
    /// At most four known primitives; sets the auxiliary framebuffer bit 0.
    pub(crate) few_primitives: bool,
    /// Host-only: the resolved barriers order the fragment kick, not the tiling kick.
    pub(crate) fragment_barriers: bool,
    /// GPU address of the occlusion-query results, or zero.
    pub(crate) occlusion_query_va: u64,
    /// GPU address of the sampler heap, or zero.
    pub(crate) sampler_heap_va: u64,
    /// Number of samplers in the heap.
    pub(crate) sampler_count: u32,
    /// ZLS control at the pass boundaries.
    pub(crate) zls_control: u64,
    /// Depth surface dimensions.
    pub(crate) depth_dimensions: u64,
    /// Depth surface.
    pub(crate) depth: ZlsSurface,
    /// Stencil surface.
    pub(crate) stencil: ZlsSurface,
    /// Depth clear value (IEEE-754 bits).
    pub(crate) depth_clear: u32,
    /// Stencil clear value (background object register value).
    pub(crate) stencil_clear: u32,
    /// Merge upper bound in X (IEEE-754 bits).
    pub(crate) merge_upper_x: u32,
    /// Merge upper bound in Y (IEEE-754 bits).
    pub(crate) merge_upper_y: u32,
    /// Base of the queue's USC window.
    pub(crate) usc_base: u64,
    /// Background program.
    pub(crate) bg: UscProgram,
    /// Partial-render background program.
    pub(crate) partial_bg: UscProgram,
    /// Tagged USC offset of the end-of-tile program.
    pub(crate) eot_offset: u32,
    /// Resource specifier of the end-of-tile program.
    pub(crate) eot_rsrc_spec: u64,
    /// Tagged USC offset of the partial-render end-of-tile program.
    pub(crate) partial_eot_offset: u32,
    /// Resource specifier of the partial-render end-of-tile program.
    pub(crate) partial_eot_rsrc_spec: u64,
    /// The resource specifiers have their high 32 bits.
    pub(crate) rsrc_spec_hi: bool,
}

/// Kernel-owned objects and identities a render pass uses.
#[derive(Copy, Clone, Debug)]
pub(crate) struct RenderResources {
    /// UAT context the pass runs in.
    pub(crate) context_id: u16,
    /// Allocation generation of that context.
    pub(crate) context_generation: u8,
    /// Buffer slot of the parameter buffer.
    pub(crate) pb_slot: u32,
    /// Buffer slot of the USC private-memory free list.
    pub(crate) free_list_slot: u32,
    /// GPU address of the free list's shared control object.
    pub(crate) free_list_control_va: u64,
    /// GPU address of the pass's kick record, in which the firmware counts
    /// the pass's completed kicks.
    pub(crate) kick_record_va: u64,
    /// GPU address of the parameter-buffer state object.
    pub(crate) pb_va: u64,
    /// GPU address of the parameter-buffer scene entry of the pass.
    pub(crate) scene_va: u64,
    /// GPU address following the last scene entry.
    pub(crate) scene_end_va: u64,
    /// Parameter-buffer scratch offset register value.
    pub(crate) pm_scratch: u64,
    /// Parameter-buffer metric pointer register value.
    pub(crate) pm_metric: u64,
    /// GPU address of the tile map.
    pub(crate) tilemap_va: u64,
    /// GPU address of the layer metadata.
    pub(crate) layermeta_va: u64,
    /// GPU address of the heap metadata.
    pub(crate) heapmeta_va: u64,
    /// GPU address of the tail-pointer cache.
    pub(crate) tpc_va: u64,
    /// Size of the tail-pointer cache the target needs.
    pub(crate) tpc_size: u64,
    /// GPU addresses of the tiler parameter sub-buffers (entry 0 is the TPC); zero for
    /// an empty one and on other GPUs.
    pub(crate) gtp_va: [u64; 8],
    /// GPU address of the deflake buffer.
    pub(crate) deflake_va: u64,
    /// GPU address of the tiling status word.
    pub(crate) ta_status_va: u64,
    /// GPU address of the fragment status word, as seen by the pass's VM.
    pub(crate) fragment_status_va: u64,
    /// Firmware alias of the fragment status word.
    pub(crate) fragment_status_fw_va: u64,
    /// GPU address of the auxiliary framebuffer.
    pub(crate) aux_fb_va: u64,
    /// GPU address of the pass's start/end timestamp pair.
    pub(crate) timestamp_va: u64,
    /// Object IDs of the tiling stage.
    pub(crate) ta_ids: ObjectIds,
    /// Object IDs of the fragment stage.
    pub(crate) fragment_ids: ObjectIds,
    /// Value of fragment descriptor field +0x2104: bit 0 of GPU register
    /// 0xe0141c, read at probe.
    pub(crate) unk_2104: u32,
    /// Value of fragment descriptor field +0x210c: bits 3:0 of GPU register
    /// 0xe01480, read at probe.
    pub(crate) unk_210c: u32,
}

/// Per-stage inputs of a render pass.
#[derive(Copy, Clone, Debug)]
pub(crate) struct RenderStage {
    /// Queue the stage is kicked on.
    pub(crate) qid: u8,
    /// GPU address of the stage's descriptor, as seen by the pass's VM.
    pub(crate) descriptor_va: u64,
    /// Command-buffer state word of the stage.
    pub(crate) state_word: u32,
    /// GPU address of the queue's completion stamp word.
    pub(crate) stamp_va: u64,
    /// GPU address of the queue's word in the auxiliary stamp page.
    pub(crate) aux_stamp_va: u64,
}

/// Inputs of both render descriptors.
pub(crate) struct RenderArgs<'a> {
    /// Render-pass state.
    pub(crate) pass: &'a RenderPass,
    pub(crate) ray_query: [super::render_rt::RayQueryStage; 2],
    /// Kernel-owned objects.
    pub(crate) resources: &'a RenderResources,
    /// Submission ordinal of the render queue pair, from zero.
    pub(crate) ordinal: u64,
    /// Tiling stage.
    pub(crate) ta: RenderStage,
    /// Fragment stage.
    pub(crate) fragment: RenderStage,
    /// Register programs of the firmware build.
    pub(crate) programs: RegisterPrograms,
    /// Deflake-buffer layout of the GPU.
    pub(crate) deflake: Deflake,
}

impl RenderArgs<'_> {
    /// Completion stamp value of this pass: the pass's sequence number is the
    /// pair's submission ordinal plus one.
    fn stamp_value(&self) -> u32 {
        stamp_value((self.ordinal as u32).wrapping_add(1))
    }
}

/// Tile-grid geometry of a render target, in the units its registers take.
struct Geometry {
    tiles_x: u64,
    tiles_y: u64,
    /// Size of the region array.
    region_size: u64,
    /// Number of macro tiles.
    macro_tiles: u64,
    /// Two utile slots per utile of every macro tile.
    utile_slots: u64,
    /// Macro-tile columns times 3, 2 and 1, in 9-bit fields from bit 0.
    x_blocks: u64,
    /// Macro-tile rows times 3, 2 and 1, in 9-bit fields from bit 0.
    y_blocks: u64,
    /// Tiles in Y less one from bit 12, tiles in X less one below.
    screen: u64,
    /// Height less one from bit 16, width less one below.
    pixels: u64,
    /// Macro-tile width in utiles from bit 16, height in utiles below.
    macro_size: u64,
}

impl Geometry {
    fn new(pass: &RenderPass) -> Result<Self> {
        if pass.width == 0 || pass.height == 0 || pass.layers == 0 || pass.layers > 2048 {
            return Err(EINVAL);
        }
        if !matches!(
            (pass.utile_width, pass.utile_height),
            (32, 32) | (32, 16) | (16, 16)
        ) {
            return Err(EINVAL);
        }
        let width = u64::from(pass.width);
        let height = u64::from(pass.height);
        let tiles_x = width.div_ceil(TILE_SIZE);
        let tiles_y = height.div_ceil(TILE_SIZE);
        let utiles_x = TILE_SIZE / u64::from(pass.utile_width);
        let utiles_y = TILE_SIZE / u64::from(pass.utile_height);
        let utiles = utiles_x * utiles_y;
        let mtile_x = tiles_x.div_ceil(MACRO_TILES).next_multiple_of(4);
        let mtile_y = tiles_y.div_ceil(MACRO_TILES).next_multiple_of(4);
        let macro_tiles = mtile_x * mtile_y;
        Ok(Self {
            tiles_x,
            tiles_y,
            region_size: (REGION_ENTRY_SIZE * macro_tiles * utiles).div_ceil(4),
            macro_tiles,
            utile_slots: 2 * macro_tiles * utiles,
            x_blocks: (3 * mtile_x) | ((2 * mtile_x) << 9) | (mtile_x << 18),
            y_blocks: (3 * mtile_y) | ((2 * mtile_y) << 9) | (mtile_y << 18),
            screen: (tiles_y - 1) << 12 | (tiles_x - 1),
            pixels: (width - 1) | (height - 1) << 16,
            macro_size: (mtile_y * utiles_y) | ((mtile_x * utiles_x) << 16),
        })
    }
}

/// Register and descriptor words derived from the [`RenderPass`].
struct PassWords {
    utile_config: u64,
    tile_config: u64,
    layer_mode: u64,
    tib_blocks: u64,
    partial_zls_control: u64,
    aux_fb_flags: u64,
    bg_rsrc_spec: u64,
    bg_va: u64,
    partial_bg_rsrc_spec: u64,
    partial_bg_va: u64,
    eot_va: u64,
    eot_rsrc_spec: u64,
    partial_eot_va: u64,
    partial_eot_rsrc_spec: u64,
}

impl PassWords {
    fn new(pass: &RenderPass) -> Result<Self> {
        if pass.samples_log2 > SAMPLES_LOG2_MAX {
            return Err(EINVAL);
        }
        let samples = 1u64 << pass.samples_log2;
        let layered = pass.layers > 1;

        // The tile configuration carries the empty-tile request for every
        // target; the tiling layer mode only for layered ones.
        let mut tile_config = TILE_CONFIG_BASE;
        let mut layer_mode = LAYER_MODE_BASE | (u64::from(pass.layers) - 1);
        if layered {
            tile_config |= TILE_CONFIG_LAYERED;
            layer_mode |= LAYER_MODE_LAYERED;
            layer_mode |= match pass.process_empty_tiles {
                true => LAYER_MODE_PROCESS_EMPTY,
                false => LAYER_MODE_SKIP_EMPTY,
            };
        }
        if pass.process_empty_tiles {
            tile_config |= TILE_CONFIG_PROCESS_EMPTY_TILES;
        }

        let mut partial_zls_control = pass.zls_control;
        if pass.depth.va != 0 {
            partial_zls_control |= ZLS_PARTIAL_DEPTH;
        }
        if pass.stencil.va != 0 {
            partial_zls_control |= ZLS_PARTIAL_STENCIL;
        }

        let mut aux_fb_flags = AUX_FB_FLAGS_BASE;
        if pass.depth_bias_is_int {
            aux_fb_flags |= AUX_FB_FLAGS_DBIAS_INT;
        }
        if pass.few_primitives {
            aux_fb_flags |= AUX_FB_FLAGS_FEW_PRIMITIVES;
        }
        let rsrc_spec = |spec: u64| match pass.rsrc_spec_hi {
            true => spec,
            false => spec & u64::from(u32::MAX),
        };
        let usc = |offset: u32| offset_va(pass.usc_base, u64::from(offset & !USC_OFFSET_TAG_MASK));

        Ok(Self {
            utile_config: (u64::from(pass.utile_width) / 16) << UTILE_CONFIG_WIDTH_SHIFT
                | (u64::from(pass.utile_height) / 16) << UTILE_CONFIG_HEIGHT_SHIFT
                | u64::from(pass.samples_log2),
            tile_config,
            layer_mode,
            tib_blocks: (u64::from(pass.sample_size)
                * u64::from(pass.utile_width)
                * u64::from(pass.utile_height)
                * samples)
                .div_ceil(TIB_BLOCK_SIZE),
            partial_zls_control,
            aux_fb_flags,
            bg_rsrc_spec: rsrc_spec(pass.bg.rsrc_spec),
            bg_va: usc(pass.bg.offset)?,
            partial_bg_rsrc_spec: rsrc_spec(pass.partial_bg.rsrc_spec),
            partial_bg_va: usc(pass.partial_bg.offset)?,
            eot_va: usc(pass.eot_offset)?,
            eot_rsrc_spec: rsrc_spec(pass.eot_rsrc_spec),
            partial_eot_va: usc(pass.partial_eot_offset)?,
            partial_eot_rsrc_spec: rsrc_spec(pass.partial_eot_rsrc_spec),
        })
    }
}

/// Offset of an address from the user window base.
fn user_offset(va: u64) -> Result<u64> {
    va.checked_sub(USER_BASE).ok_or(EINVAL)
}

/// Offset of an address from the user window base that fits 32 bits.
fn user_offset32(va: u64) -> Result<u32> {
    u32::try_from(user_offset(va)?).map_err(|_| EINVAL)
}

fn ta_registers(
    args: &RenderArgs<'_>,
    g: &Geometry,
    w: &PassWords,
) -> Result<[RegisterWrite; TA_REGISTERS_MAX]> {
    let p = args.pass;
    let r = args.resources;
    let tilemap = user_offset(r.tilemap_va)?;
    let layermeta = user_offset(r.layermeta_va)?;
    let heapmeta = user_offset(r.heapmeta_va)? | 1 << 63;
    let tpc = user_offset(r.tpc_va)?;
    let [deflake_20, deflake_2a0] = args.deflake.sub;
    let deflake = u64::from(user_offset32(r.deflake_va)?);
    let deflake_20 = u64::from(user_offset32(offset_va(r.deflake_va, deflake_20)?)?);
    let deflake_2a0 = u64::from(user_offset32(offset_va(r.deflake_va, deflake_2a0)?)?);
    let vdm = user_offset(p.vdm_va)?;
    if vdm >= VDM_OFFSET_LIMIT {
        return Err(EINVAL);
    }
    let key = work_key(r.context_id, args.ta.state_word);
    let ids = r.ta_ids.word();
    let rt = args.ray_query[0];
    let ift = rt.intersection.words();

    Ok([
        RegisterWrite::new(0x01748, 1),
        RegisterWrite::new(0x017d9, 1),
        RegisterWrite::new(0x10141, 0x200),
        RegisterWrite::new(0x1c039, tilemap),
        RegisterWrite::new(0x1c9c8, tilemap),
        RegisterWrite::new(0x1c0a1, tpc),
        RegisterWrite::new(0x1c031, heapmeta),
        RegisterWrite::new(0x1c9c0, heapmeta),
        RegisterWrite::new(0x1c051, 0x003a_0012_006b_0003),
        RegisterWrite::new(0x1c061, 1),
        RegisterWrite::new(0x10149, w.utile_config),
        RegisterWrite::new(0x10139, p.msaa_control),
        RegisterWrite::new(0x10111, deflake_2a0),
        RegisterWrite::new(0x1c9b0, deflake_2a0),
        RegisterWrite::new(0x10119, deflake_20),
        RegisterWrite::new(0x1c9b8, deflake_20),
        RegisterWrite::new(0x1c958, 1),
        RegisterWrite::new(0x1c950, deflake | 1 << 50),
        RegisterWrite::new(0x1c930, 0),
        RegisterWrite::new(0x1c880, vdm),
        RegisterWrite::new(0x1c079, layermeta),
        RegisterWrite::new(0x1c9d8, layermeta),
        RegisterWrite::new(0x10151, 0),
        RegisterWrite::new(0x1c199, 0),
        RegisterWrite::new(0x1c1a1, 0),
        RegisterWrite::new(0x1c1a9, 0),
        RegisterWrite::new(0x1c1b1, 0),
        RegisterWrite::new(0x1c1b9, 0),
        RegisterWrite::new(0x1c8f8, UNK_1C8F8),
        RegisterWrite::new(0x1c0b1, g.region_size),
        RegisterWrite::new(0x1c850, g.region_size),
        RegisterWrite::new(0x10131, p.msaa_control),
        RegisterWrite::new(0x10121, p.ppp_control),
        RegisterWrite::new(0x10129, g.pixels),
        RegisterWrite::new(0x101b9, g.screen),
        RegisterWrite::new(0x1c069, g.x_blocks),
        RegisterWrite::new(0x1c071, g.y_blocks),
        RegisterWrite::new(0x1c081, g.macro_tiles),
        RegisterWrite::new(0x1c0a9, g.utile_slots),
        RegisterWrite::new(0x10171, 0x100),
        RegisterWrite::new(0x10169, w.layer_mode),
        RegisterWrite::new(0x0a309, 0),
        RegisterWrite::new(0x1c8e0, u64::MAX),
        RegisterWrite::new(0x1c8e8, u64::MAX),
        RegisterWrite::new(0x1c898, 0),
        RegisterWrite::new(0x101e1, UNK_101D9),
        RegisterWrite::new(0x1c9e8, w.layer_mode & LAYER_MODE_PIPE_MASK),
        RegisterWrite::new(0x1a099, ift[0]),
        RegisterWrite::new(0x1a0a1, ift[1]),
        RegisterWrite::new(0x1a069, ift[2]),
        RegisterWrite::new(0x1a071, ift[3]),
        RegisterWrite::new(0x1a0c9, ift[4]),
        RegisterWrite::new(0x1a0d1, ift[5]),
        RegisterWrite::new(0x101c9, rt.scratch.size_word()),
        RegisterWrite::new(0x0d471, rt.scratch.tail_word()),
        RegisterWrite::new(0x1a0f1, rt.scratch.stack_word()),
        RegisterWrite::new(0x10799, rt.scratch.window_word()),
        RegisterWrite::new(0x1c830, u64::from(r.pb_slot)),
        RegisterWrite::new(0x1ca30, r.pm_scratch),
        RegisterWrite::new(0x16c39, r.pm_scratch),
        RegisterWrite::new(0x1c910, r.pm_metric),
        RegisterWrite::new(0x0a5a1, UNK_0A599),
        RegisterWrite::new(0x0d419, UNK_0D411),
        RegisterWrite::new(0x1ca10, ids),
        RegisterWrite::new(0x014a1, ids),
        RegisterWrite::new(0x0a349, ids),
        RegisterWrite::new(0x10209, key),
        RegisterWrite::new(0x1c9f0, key),
        RegisterWrite::new(0x14320, key),
        RegisterWrite::new(0x14308, u64::from(r.free_list_slot)),
        RegisterWrite::new(0x14318, r.ta_status_va | 1),
        RegisterWrite::new(0x01740, 1),
        RegisterWrite::new(0x1c880, deflake_2a0),
        RegisterWrite::new(0x1c898, 1),
    ])
}

/// The 25G83 tiling array: the A18 Pro's without 0x1748 and 0x1740 (and 0x17d9 unless the stage
/// has SCS), with the tiler parameter sub-buffer registers and 0x10131 = 0x88, in the order the
/// M4 Pro has run with. Returns the array and its used entry count.
fn ta_registers_25g83(
    args: &RenderArgs<'_>,
    g: &Geometry,
    w: &PassWords,
) -> Result<([RegisterWrite; TA_REGISTERS_25G83 + 1], usize)> {
    let a = ta_registers(args, g, w)?;
    let r = args.resources;
    let gtp = |index: usize| -> Result<u64> {
        match r.gtp_va[index] {
            0 => Ok(0),
            va => user_offset(va),
        }
    };
    let find = |reg: u32, nth: usize| -> Result<RegisterWrite> {
        a.iter()
            .filter(|write| write.reg == reg)
            .nth(nth)
            .copied()
            .ok_or(EINVAL)
    };
    let gtp1 = gtp(1)?;
    let gtp2 = gtp(2)?;
    let gtp6 = gtp(6)? & !127;
    let scs = usize::from(args.ray_query[0].scratch.enabled());
    let mut out = [RegisterWrite::new(0, 0); TA_REGISTERS_25G83 + 1];
    let mut n = 0;
    let mut push = |write: RegisterWrite| {
        out[n] = write;
        n += 1;
    };
    if scs != 0 {
        push(find(SCS_ENABLE, 0)?);
    }
    for reg in [0x10141, 0x1c039, 0x1c9c8] {
        push(find(reg, 0)?);
    }
    push(RegisterWrite::new(0x1c041, gtp1));
    push(RegisterWrite::new(0x1c9d0, gtp1));
    for reg in [
        0x1c0a1, 0x1c031, 0x1c9c0, 0x1c051, 0x1c061, 0x10149, 0x10139, 0x10111, 0x1c9b0, 0x10119,
        0x1c9b8, 0x1c958, 0x1c950, 0x1c930,
    ] {
        push(find(reg, 0)?);
    }
    push(find(0x1c880, 0)?);
    push(RegisterWrite::new(0x1c948, gtp(4)?));
    push(RegisterWrite::new(0x1c888, gtp(5)?));
    push(RegisterWrite::new(0x1c890, 0));
    push(RegisterWrite::new(0x1c918, 0));
    push(find(0x1c079, 0)?);
    push(find(0x1c9d8, 0)?);
    push(RegisterWrite::new(0x1c089, gtp2));
    push(RegisterWrite::new(0x1c9e0, gtp2));
    push(RegisterWrite::new(0x16c41, gtp6));
    push(RegisterWrite::new(0x1ca40, gtp6));
    push(RegisterWrite::new(0x1c9a8, 0));
    push(RegisterWrite::new(0x1c920, gtp(7)?));
    for reg in [0x10151, 0x1c199, 0x1c1a1, 0x1c1a9, 0x1c1b1, 0x1c1b9, 0x1c8f8, 0x1c0b1, 0x1c850] {
        push(find(reg, 0)?);
    }
    push(RegisterWrite::new(0x10131, 0x88));
    for reg in [
        0x10121, 0x10129, 0x101b9, 0x1c069, 0x1c071, 0x1c081, 0x1c0a9, 0x10171, 0x10169, 0x0a309,
        0x1c8e0, 0x1c8e8,
    ] {
        push(find(reg, 0)?);
    }
    push(find(0x1c898, 0)?);
    for reg in [
        0x101e1, 0x1c9e8, 0x1a099, 0x1a0a1, 0x1a069, 0x1a071, 0x1a0c9, 0x1a0d1, 0x101c9, 0x0d471,
        0x1a0f1, 0x10799, 0x1c830, 0x1ca30, 0x16c39, 0x1c910, 0x0a5a1, 0x0d419, 0x1ca10, 0x014a1,
        0x0a349, 0x10209, 0x1c9f0, 0x14320, 0x14308, 0x14318,
    ] {
        push(find(reg, 0)?);
    }
    push(find(0x1c880, 1)?);
    push(find(0x1c898, 1)?);
    if n != TA_REGISTERS_25G83 + scs || out[TA_SLOT_INDEX_25G83 as usize + scs].reg != 0x14308 {
        return Err(EINVAL);
    }
    Ok((out, n))
}

/// The 25G83 main fragment array: the A18 Pro's (without 0x17d9 unless the stage has SCS), with
/// 0x160b8 = 0 after the tile state and 0x160a8 = 0x20 before the closing 0x16068, as the M4
/// Pro has run with. Returns the array and its used entry count.
fn fragment_registers_25g83(
    args: &RenderArgs<'_>,
    g: &Geometry,
    w: &PassWords,
) -> Result<([RegisterWrite; FRAGMENT_REGISTERS_25G83 + 1], usize)> {
    let a = fragment_registers(args, g, w);
    let scs = usize::from(args.ray_query[1].scratch.enabled());
    let mut out = [RegisterWrite::new(0, 0); FRAGMENT_REGISTERS_25G83 + 1];
    let mut n = 0;
    let mut tile_state = false;
    for (index, write) in a.iter().enumerate() {
        if write.reg == SCS_ENABLE && scs == 0 {
            continue;
        }
        if index == a.len() - 1 {
            *out.get_mut(n).ok_or(EINVAL)? = RegisterWrite::new(0x160a8, 0x20);
            n += 1;
        }
        *out.get_mut(n).ok_or(EINVAL)? = *write;
        n += 1;
        if write.reg == 0x16068 && !tile_state {
            tile_state = true;
            *out.get_mut(n).ok_or(EINVAL)? = RegisterWrite::new(0x160b8, 0);
            n += 1;
        }
    }
    if n != FRAGMENT_REGISTERS_25G83 + scs
        || out[FRAGMENT_SLOT_INDEX_25G83 as usize + scs].reg != 0x14048
    {
        return Err(EINVAL);
    }
    Ok((out, n))
}

fn fragment_registers(
    args: &RenderArgs<'_>,
    g: &Geometry,
    w: &PassWords,
) -> [RegisterWrite; FRAGMENT_REGISTERS_MAX] {
    let p = args.pass;
    let r = args.resources;
    let tile_state_base = if args.programs == RegisterPrograms::V25G83 {
        TILE_STATE_BASE_25G83
    } else {
        TILE_STATE_BASE
    };
    let mut tile_state = tile_state_base
        | (w.utile_config & UTILE_CONFIG_SIZE_MASK) << TILE_STATE_UTILE_SHIFT
        | (g.tiles_x - 1) << TILE_STATE_TILES_X_SHIFT
        | (g.tiles_y - 1) << TILE_STATE_TILES_Y_SHIFT;
    if p.layers > 1 {
        tile_state |= TILE_STATE_LAYERED;
    }
    let key = work_key(r.context_id, args.fragment.state_word);
    let ids = r.fragment_ids.word();
    let rt = args.ray_query[1];
    let ift = rt.intersection.words();

    [
        RegisterWrite::new(0x01739, 1),
        RegisterWrite::new(0x017d9, 1),
        RegisterWrite::new(0x10009, w.utile_config),
        // End-of-tile resource specifier.
        RegisterWrite::new(0x15379, w.eot_rsrc_spec),
        RegisterWrite::new(0x15381, w.eot_va),
        RegisterWrite::new(0x15369, w.bg_rsrc_spec),
        RegisterWrite::new(0x15371, w.bg_va),
        RegisterWrite::new(0x15131, u64::from(p.merge_upper_x)),
        RegisterWrite::new(0x15139, u64::from(p.merge_upper_y)),
        RegisterWrite::new(0x100a1, 0),
        RegisterWrite::new(0x15069, 0),
        RegisterWrite::new(0x15071, 0),
        RegisterWrite::new(0x16058, 0),
        RegisterWrite::new(0x10019, p.msaa_control),
        RegisterWrite::new(0x100b1, g.macro_size),
        RegisterWrite::new(0x16030, g.macro_size),
        RegisterWrite::new(0x100d9, g.screen),
        RegisterWrite::new(0x0a301, 0),
        RegisterWrite::new(0x10791, rt.scratch.window_word() | 1 << 9),
        RegisterWrite::new(0x16098, r.heapmeta_va),
        RegisterWrite::new(0x15109, p.scissor_va),
        RegisterWrite::new(0x15101, p.depth_bias_va),
        RegisterWrite::new(0x15021, w.aux_fb_flags),
        RegisterWrite::new(0x15211, u64::from(p.height) << 32 | u64::from(p.width)),
        RegisterWrite::new(0x15049, UNK_15049),
        RegisterWrite::new(0x10051, w.tib_blocks),
        RegisterWrite::new(0x15321, p.depth_dimensions),
        RegisterWrite::new(0x15301, u64::from(p.depth_clear)),
        RegisterWrite::new(0x15309, u64::from(p.stencil_clear | STENCIL_CLEAR_BASE)),
        RegisterWrite::new(0x15311, p.occlusion_query_va),
        RegisterWrite::new(0x15319, p.zls_control),
        RegisterWrite::new(0x15349, u64::from(ZLS_SWIZZLE)),
        RegisterWrite::new(0x15351, 0),
        RegisterWrite::new(0x15329, p.depth.va),
        RegisterWrite::new(0x15331, p.depth.va),
        RegisterWrite::new(0x15339, p.stencil.va),
        RegisterWrite::new(0x15341, p.stencil.va),
        RegisterWrite::new(0x15231, 0),
        RegisterWrite::new(0x15221, 0),
        RegisterWrite::new(0x15239, 0),
        RegisterWrite::new(0x15229, 0),
        RegisterWrite::new(0x15401, p.depth.stride),
        RegisterWrite::new(0x15421, p.depth.stride),
        RegisterWrite::new(0x15409, p.stencil.stride),
        RegisterWrite::new(0x15429, p.stencil.stride),
        RegisterWrite::new(0x153c1, p.depth.compression_va),
        RegisterWrite::new(0x15411, p.depth.compression_stride),
        RegisterWrite::new(0x153c9, p.depth.compression_va),
        RegisterWrite::new(0x15431, p.depth.compression_stride),
        RegisterWrite::new(0x153d1, p.stencil.compression_va),
        RegisterWrite::new(0x15419, p.stencil.compression_stride),
        RegisterWrite::new(0x153d9, p.stencil.compression_va),
        RegisterWrite::new(0x15439, p.stencil.compression_stride),
        RegisterWrite::new(0x16429, r.tilemap_va),
        RegisterWrite::new(0x16060, r.layermeta_va),
        RegisterWrite::new(0x16431, (4 * g.region_size) << 24),
        RegisterWrite::new(0x10039, w.tile_config),
        RegisterWrite::new(0x16020, 0),
        RegisterWrite::new(0x16451, 0),
        RegisterWrite::new(0x15359, 0),
        RegisterWrite::new(0x100b8, UNK_1C8F8),
        RegisterWrite::new(0x16461, r.aux_fb_va),
        RegisterWrite::new(0x16090, r.aux_fb_va),
        RegisterWrite::new(0x101e9, UNK_101D9),
        RegisterWrite::new(0x160a8, 0),
        RegisterWrite::new(0x16068, tile_state),
        RegisterWrite::new(0x1a0a9, ift[0]),
        RegisterWrite::new(0x1a0b1, ift[1]),
        RegisterWrite::new(0x1a079, ift[2]),
        RegisterWrite::new(0x1a081, ift[3]),
        RegisterWrite::new(0x1a0d9, ift[4]),
        RegisterWrite::new(0x1a0e1, ift[5]),
        RegisterWrite::new(0x101c1, rt.scratch.size_word()),
        RegisterWrite::new(0x0d469, rt.scratch.tail_word()),
        RegisterWrite::new(0x1a0f9, rt.scratch.stack_word()),
        RegisterWrite::new(0x0a5a9, UNK_0A599),
        RegisterWrite::new(0x0d429, UNK_0D411),
        RegisterWrite::new(0x160e0, ids),
        RegisterWrite::new(0x01499, ids),
        RegisterWrite::new(0x0a341, ids),
        RegisterWrite::new(0x1c838, u64::from(r.pb_slot)),
        RegisterWrite::new(0x1ca28, r.pm_scratch),
        RegisterWrite::new(0x10211, key),
        RegisterWrite::new(0x10420, key),
        RegisterWrite::new(0x14048, u64::from(r.free_list_slot)),
        RegisterWrite::new(0x14080, r.fragment_status_va | 1),
        RegisterWrite::new(0x01731, 1),
        RegisterWrite::new(0x16020, 1),
        RegisterWrite::new(0x16020, 0),
        RegisterWrite::new(0x16068, 0x4_0000),
    ]
}

fn partial_store_registers(
    p: &RenderPass,
    w: &PassWords,
) -> [RegisterWrite; PARTIAL_STORE_REGISTERS] {
    [
        // End-of-tile resource specifier.
        RegisterWrite::new(0x15379, w.partial_eot_rsrc_spec),
        RegisterWrite::new(0x15381, w.partial_eot_va),
        RegisterWrite::new(0x10039, w.tile_config),
        RegisterWrite::new(0x15359, UNK_15359_PARTIAL),
        RegisterWrite::new(0x15331, p.depth.va),
        RegisterWrite::new(0x153c9, p.depth.compression_va),
        RegisterWrite::new(0x15341, p.stencil.va),
        RegisterWrite::new(0x153d9, p.stencil.compression_va),
        RegisterWrite::new(0x15421, p.depth.stride),
        RegisterWrite::new(0x15431, p.depth.compression_stride),
        RegisterWrite::new(0x15429, p.stencil.stride),
        RegisterWrite::new(0x15439, p.stencil.compression_stride),
        RegisterWrite::new(0x15221, 0),
        RegisterWrite::new(0x15229, 0),
        RegisterWrite::new(0x15319, w.partial_zls_control),
        RegisterWrite::new(0x15349, u64::from(ZLS_SWIZZLE)),
    ]
}

fn partial_resume_registers(
    p: &RenderPass,
    w: &PassWords,
) -> [RegisterWrite; PARTIAL_RESUME_REGISTERS] {
    [
        // End-of-tile resource specifier.
        RegisterWrite::new(0x15379, w.partial_eot_rsrc_spec),
        RegisterWrite::new(0x15381, w.partial_eot_va),
        RegisterWrite::new(0x15369, w.partial_bg_rsrc_spec),
        RegisterWrite::new(0x15371, w.partial_bg_va),
        RegisterWrite::new(0x10039, w.tile_config & !TILE_CONFIG_PROCESS_EMPTY_TILES),
        RegisterWrite::new(0x15359, UNK_15359_PARTIAL),
        RegisterWrite::new(0x15331, p.depth.va),
        RegisterWrite::new(0x153c9, p.depth.compression_va),
        RegisterWrite::new(0x15341, p.stencil.va),
        RegisterWrite::new(0x153d9, p.stencil.compression_va),
        RegisterWrite::new(0x15421, p.depth.stride),
        RegisterWrite::new(0x15431, p.depth.compression_stride),
        RegisterWrite::new(0x15429, p.stencil.stride),
        RegisterWrite::new(0x15439, p.stencil.compression_stride),
        RegisterWrite::new(0x15221, 0),
        RegisterWrite::new(0x15229, 0),
        RegisterWrite::new(0x15309, u64::from(p.stencil_clear | STENCIL_CLEAR_BASE)),
        RegisterWrite::new(0x15329, p.depth.va),
        RegisterWrite::new(0x153c1, p.depth.compression_va),
        RegisterWrite::new(0x15339, p.stencil.va),
        RegisterWrite::new(0x153d1, p.stencil.compression_va),
        RegisterWrite::new(0x15319, w.partial_zls_control),
        RegisterWrite::new(0x15349, u64::from(ZLS_SWIZZLE)),
    ]
}

fn partial_load_registers(
    p: &RenderPass,
    w: &PassWords,
) -> [RegisterWrite; PARTIAL_LOAD_REGISTERS] {
    [
        RegisterWrite::new(0x15369, w.partial_bg_rsrc_spec),
        RegisterWrite::new(0x15371, w.partial_bg_va),
        RegisterWrite::new(0x10039, w.tile_config & !TILE_CONFIG_PROCESS_EMPTY_TILES),
        RegisterWrite::new(0x15309, u64::from(p.stencil_clear | STENCIL_CLEAR_BASE)),
        RegisterWrite::new(0x15329, p.depth.va),
        RegisterWrite::new(0x153c1, p.depth.compression_va),
        RegisterWrite::new(0x15339, p.stencil.va),
        RegisterWrite::new(0x153d1, p.stencil.compression_va),
        RegisterWrite::new(0x15319, w.partial_zls_control),
        RegisterWrite::new(0x15349, u64::from(ZLS_SWIZZLE)),
    ]
}

/// Tiling (TA) work descriptor (tag 0).
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct TaDescriptor {
    /// [`CommandTag::Tiling`].
    pub(crate) tag: u32,
    /// Twice the pair's submission ordinal, plus one.
    pub(crate) submit_sequence: u64,
    /// UAT context of the pass.
    pub(crate) context_id: u32,
    /// GPU address of the pass's kick record.
    pub(crate) kick_record_va: u64,
    /// Buffer slot of the parameter buffer.
    pub(crate) pb_slot: u32,
    pub(crate) unk_1c: u32,
    /// GPU address of the parameter-buffer state object.
    pub(crate) pb_va: u64,
    /// GPU address of the parameter-buffer scene entry.
    pub(crate) scene_va: u64,
    /// GPU address following the last scene entry.
    pub(crate) scene_end_va: u64,
    /// Checkpoints into the register array.
    pub(crate) checkpoints: [u16; 3],
    pub(crate) unk_3e: [u8; 0xa],
    /// The pair's submission ordinal plus half of it.
    pub(crate) unk_48: u32,
    pub(crate) unk_4c: [u8; 0x14],
    /// Register array.
    pub(crate) registers: RegisterArray,
    /// GPU address of the tail-pointer cache.
    pub(crate) tpc_va: u64,
    /// Size of the tail-pointer cache the target needs.
    pub(crate) tpc_size: u64,
    pub(crate) unk_790: [u8; 0xc],
    /// QID of the fragment queue that consumes this pass.
    pub(crate) fragment_qid: u32,
    /// Completion stamp value of the pass.
    pub(crate) stamp_value: u32,
    /// Non-zero when the fragment queue had earlier kicks outstanding.
    pub(crate) fragment_busy: u32,
    /// Timestamp of the fragment kick of this pass.
    pub(crate) fragment_kick: u64,
    /// Render-pass sequence of the pass, stored twice.
    pub(crate) pass_seq: [u8; 2],
    pub(crate) unk_7b2: [u8; 0x24],
    /// Offset from the user base of the deflake sub-buffer at +0x2a0.
    pub(crate) deflake_2a0_offset: u32,
    pub(crate) unk_7da: [u8; 0x94],
    /// Predecessor object ID of the tiling stage.
    pub(crate) predecessor_id: u32,
    pub(crate) unk_872: u32,
    pub(crate) unk_876: u32,
    /// GPU address of the sampler heap.
    pub(crate) sampler_heap_va: u64,
    /// Number of samplers.
    pub(crate) sampler_count: u32,
    /// Number of samplers plus one, or zero without samplers.
    pub(crate) sampler_max: u32,
    pub(crate) unk_88a: [u8; 8],
    pub(crate) unk_892: u32,
    pub(crate) unk_896: [u8; 0x10],
    /// GPU address of the tiling queue's completion stamp word.
    pub(crate) stamp_va: u64,
    /// GPU address of the tiling queue's auxiliary stamp word.
    pub(crate) aux_stamp_va: u64,
    /// Completion stamp value of the pass.
    pub(crate) stamp_value_2: u32,
    /// QID of the tiling queue.
    pub(crate) qid: u32,
    pub(crate) unk_8be: u32,
    /// Non-zero on the first kick of a queue pair, like the fragment descriptor's word at the
    /// same position. Set on every kick, the firmware also notifies the host of each pass's
    /// tiling completion, which no retirement needs. The 25G83 firmware keeps it set on every
    /// kick until this is measured there.
    pub(crate) first_kick: u32,
    /// Completion value of the tiling kick (see
    /// [`KickTimestamp::completion_value`]).
    pub(crate) completion: u64,
    /// Current object ID of the tiling stage.
    pub(crate) current_id: u64,
    /// Submission ordinal of the pair.
    pub(crate) ordinal: u32,
    pub(crate) unk_8da: [u8; 0x24],
    /// GPU address the pass start timestamp is written to.
    pub(crate) timestamp_start_va: u64,
    /// Zero: the tiling stage writes no end timestamp.
    pub(crate) timestamp_end_va: u64,
    pub(crate) unk_90e: [u8; 0x24],
    pub(crate) unk_932: u8,
    pub(crate) unk_933: u8,
    /// GPU address of the USC free list's shared control object.
    pub(crate) free_list_control_va: u64,
    pub(crate) unk_93c: u8,
    pub(crate) unk_93d: u8,
    /// Set on the first kick of a queue pair.
    pub(crate) unk_93e: [u8; 2],
    pub(crate) unk_940: [u8; 5],
    /// GPU address of the tiling status word.
    pub(crate) status_va: u64,
    /// Allocation generation of the UAT context.
    pub(crate) context_generation: u8,
    pub(crate) unk_94e: [u8; 0x72],
}

static_assert!(size_of::<TaDescriptor>() == TA_DESCRIPTOR_SIZE);
static_assert!(core::mem::offset_of!(TaDescriptor, unk_48) == 0x48);
static_assert!(core::mem::offset_of!(TaDescriptor, registers) == 0x60);
static_assert!(core::mem::offset_of!(TaDescriptor, tpc_va) == 0x780);
static_assert!(core::mem::offset_of!(TaDescriptor, fragment_qid) == 0x79c);
static_assert!(core::mem::offset_of!(TaDescriptor, deflake_2a0_offset) == 0x7d6);
static_assert!(core::mem::offset_of!(TaDescriptor, predecessor_id) == 0x86e);
static_assert!(core::mem::offset_of!(TaDescriptor, stamp_va) == 0x8a6);
static_assert!(core::mem::offset_of!(TaDescriptor, completion) == 0x8c6);
static_assert!(core::mem::offset_of!(TaDescriptor, timestamp_start_va) == 0x8fe);
static_assert!(core::mem::offset_of!(TaDescriptor, free_list_control_va) == 0x934);
static_assert!(core::mem::offset_of!(TaDescriptor, status_va) == 0x945);

// SAFETY: `TaDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for TaDescriptor {}

impl TaDescriptor {
    const ARRAY_OFFSET: u64 = core::mem::offset_of!(Self, registers) as u64;
    const UNK_876: u32 = u32::MAX;
    const UNK_892: u32 = 1;
    const UNK_932: u8 = 0x44;
    const UNK_93C: u8 = 1;
    const UNK_93E_FIRST_KICK: [u8; 2] = [0xd0, 0x91];

    /// Writes the complete descriptor except the completion link.
    pub(crate) fn write(&mut self, args: &RenderArgs<'_>) -> Result {
        let pass = args.pass;
        let resources = args.resources;
        let geometry = Geometry::new(pass)?;
        let words = PassWords::new(pass)?;
        let registers = ta_registers(args, &geometry, &words)?;
        let deflake_2a0_offset =
            user_offset32(offset_va(resources.deflake_va, args.deflake.sub[1])?)?;
        let array_va = offset_va(args.ta.descriptor_va, Self::ARRAY_OFFSET)?;
        let sampler_max = sampler_max(pass.sampler_count)?;
        let stamp_value = args.stamp_value();

        clear(self);
        self.tag = CommandTag::Tiling as u32;
        self.submit_sequence = args.ordinal.wrapping_mul(2).wrapping_add(1);
        self.context_id = resources.context_id.into();
        self.kick_record_va = resources.kick_record_va;
        self.pb_slot = resources.pb_slot;
        self.pb_va = resources.pb_va;
        self.scene_va = resources.scene_va;
        self.scene_end_va = resources.scene_end_va;
        let scs_register = args.ray_query[0].scratch.enabled();
        self.checkpoints = TA_CHECKPOINTS.map(|count| count + u16::from(scs_register));
        self.unk_48 = (args.ordinal as u32).wrapping_add((args.ordinal / 2) as u32);
        if args.programs == RegisterPrograms::V25G83 {
            self.checkpoints = TA_CHECKPOINTS_25G83.map(|count| count + u16::from(scs_register));
            let (registers, count) = ta_registers_25g83(args, &geometry, &words)?;
            self.registers.set(array_va, &registers[..count])?;
        } else if scs_register {
            self.registers.set(array_va, &registers)?;
        } else {
            self.registers.set_without(array_va, &registers, TA_SCS_ENABLE_INDEX)?;
        }
        self.tpc_va = resources.tpc_va;
        self.tpc_size = resources.tpc_size;
        self.fragment_qid = args.fragment.qid.into();
        self.stamp_value = stamp_value;
        self.deflake_2a0_offset = deflake_2a0_offset;
        self.predecessor_id = resources.ta_ids.predecessor;
        self.unk_876 = Self::UNK_876;
        self.sampler_heap_va = pass.sampler_heap_va;
        self.sampler_count = pass.sampler_count;
        self.sampler_max = sampler_max;
        self.unk_892 = Self::UNK_892;
        self.stamp_va = args.ta.stamp_va;
        self.aux_stamp_va = args.ta.aux_stamp_va;
        self.stamp_value_2 = stamp_value;
        self.qid = args.ta.qid.into();
        self.first_kick = match args.programs == RegisterPrograms::V25G83 {
            true => 1,
            false => u32::from(args.ordinal == 0),
        };
        self.current_id = resources.ta_ids.current.into();
        self.ordinal = args.ordinal as u32;
        self.timestamp_start_va = resources.timestamp_va;
        self.unk_932 = match args.programs == RegisterPrograms::V25G83 {
            true => TA_SLOT_INDEX_25G83 + u8::from(scs_register),
            false => Self::UNK_932,
        };
        self.free_list_control_va = resources.free_list_control_va;
        self.unk_93c = Self::UNK_93C;
        if args.ordinal == 0 {
            self.unk_93e = Self::UNK_93E_FIRST_KICK;
        }
        self.status_va = resources.ta_status_va;
        self.context_generation = resources.context_generation;
        Ok(())
    }

    /// Links the pass to the timestamps its two kicks are published at.
    ///
    /// Must be called before the descriptor is published. `pass_seq` is the
    /// low byte of the logical queue's render-pass count (one for its first
    /// pass); `fragment_busy` tells whether the fragment queue had not yet
    /// completed every earlier kick.
    pub(crate) fn set_completion_link(
        &mut self,
        ta: KickTimestamp,
        fragment: KickTimestamp,
        pass_seq: u8,
        fragment_busy: bool,
    ) {
        self.fragment_busy = u32::from(fragment_busy);
        self.fragment_kick = fragment.get();
        self.pass_seq = [pass_seq; 2];
        self.completion = ta.completion_value(pass_seq);
    }

    /// Returns the register arrays a tiling kick binds; `descriptor_va` is the
    /// descriptor's address as seen by the pass's VM.
    pub(crate) fn register_bindings(
        descriptor_va: u64,
        scs_register: bool,
        programs: RegisterPrograms,
    ) -> Result<[Option<RegisterArrayBinding>; 4]> {
        let checkpoint = match programs == RegisterPrograms::V25G83 {
            true => TA_CHECKPOINTS_25G83[0] as u8,
            false => TA_CHECKPOINTS[0] as u8,
        } + u8::from(scs_register);
        let array =
            RegisterArrayBinding::new(offset_va(descriptor_va, Self::ARRAY_OFFSET)?, checkpoint)?;
        Ok([Some(array), None, None, None])
    }
}

/// Fragment (3D) work descriptor (tag 1).
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct FragmentDescriptor {
    /// [`CommandTag::Fragment`].
    pub(crate) tag: u32,
    /// Twice the pair's submission ordinal.
    pub(crate) submit_sequence: u64,
    /// UAT context of the pass.
    pub(crate) context_id: u32,
    pub(crate) unk_10: [u8; 0x10],
    /// GPU address of the pass's kick record.
    pub(crate) kick_record_va: u64,
    /// GPU address of the parameter-buffer state object.
    pub(crate) pb_va: u64,
    /// GPU address of the parameter-buffer scene entry.
    pub(crate) scene_va: u64,
    /// GPU address following the last scene entry.
    pub(crate) scene_end_va: u64,
    /// GPU address of the tile map.
    pub(crate) tilemap_va: u64,
    /// Multisample control.
    pub(crate) msaa_control: u64,
    /// Samples per pixel.
    pub(crate) samples: u32,
    /// Macro-tile dimensions in utiles: height in bits 15:0, width above.
    pub(crate) macro_size: u32,
    pub(crate) unk_58: [u8; 0x10],
    /// Merge upper bound in X.
    pub(crate) merge_upper_x: u32,
    /// Merge upper bound in Y.
    pub(crate) merge_upper_y: u32,
    pub(crate) unk_70: u64,
    /// Number of tiles.
    pub(crate) tiles: u64,
    /// Checkpoints into the main register array.
    pub(crate) checkpoints: [u16; 3],
    pub(crate) unk_86: u16,
    /// Entry count of the main register array.
    pub(crate) entries: u16,
    pub(crate) unk_8a: [u8; 6],
    /// Render-pass sequence of the pass.
    pub(crate) pass_seq: u8,
    pub(crate) unk_91: [u8; 0xf],
    /// Main, partial-store, partial-resume and partial-load register arrays.
    pub(crate) arrays: [RegisterArray; 4],
    /// GPU address of the depth-bias array.
    pub(crate) depth_bias_va: u64,
    pub(crate) unk_1d28: u64,
    /// GPU address of the scissor array.
    pub(crate) scissor_va: u64,
    pub(crate) unk_1d38: u64,
    /// GPU address of the occlusion-query results.
    pub(crate) occlusion_query_va: u64,
    pub(crate) unk_1d48: [u8; 0x130],
    /// Background resource specifier.
    pub(crate) bg_rsrc_spec: u64,
    /// GPU address of the background program.
    pub(crate) bg_va: u64,
    pub(crate) unk_1e88: [u8; 0x20],
    /// Partial-render background resource specifier.
    pub(crate) partial_bg_rsrc_spec: u64,
    /// GPU address of the partial-render background program.
    pub(crate) partial_bg_va: u64,
    pub(crate) unk_1eb8: u64,
    /// ZLS swizzle.
    pub(crate) zls_swizzle: u32,
    pub(crate) unk_1ec4: [u8; 0x74],
    /// Tile-buffer blocks per utile.
    pub(crate) tib_blocks: u64,
    /// Auxiliary framebuffer flags.
    pub(crate) aux_fb_flags: u64,
    /// Width of the render target.
    pub(crate) width: u32,
    /// Height of the render target.
    pub(crate) height: u32,
    /// [`UNK_15049`].
    pub(crate) unk_1f50: u64,
    /// Tile configuration.
    pub(crate) tile_config: u64,
    pub(crate) unk_1f60: [u8; 0x18],
    /// Low word of the end-of-tile resource specifier.
    pub(crate) eot_rsrc_spec: u32,
    /// GPU address of the end-of-tile program.
    pub(crate) eot_va: u64,
    pub(crate) unk_1f84: [u8; 0x14],
    /// Low word of the partial-render end-of-tile resource specifier.
    pub(crate) partial_eot_rsrc_spec: u32,
    /// GPU address of the partial-render end-of-tile program.
    pub(crate) partial_eot_va: u64,
    pub(crate) unk_1fa4: u32,
    /// Depth clear value.
    pub(crate) depth_clear: u32,
    /// Tile-buffer blocks in bits 63:33 above a constant.
    pub(crate) unk_1fac: u64,
    pub(crate) unk_1fb4: [u8; 0x150],
    /// See [`RenderResources::unk_2104`].
    pub(crate) unk_2104: u32,
    /// Predecessor object ID of the fragment stage.
    pub(crate) predecessor_id: u32,
    /// See [`RenderResources::unk_210c`].
    pub(crate) unk_210c: u32,
    pub(crate) unk_2110: u32,
    pub(crate) unk_2114: [u8; 0x10],
    pub(crate) unk_2124: [u32; 2],
    pub(crate) unk_212c: [u8; 0x14],
    /// GPU address of the fragment queue's completion stamp word.
    pub(crate) stamp_va: u64,
    /// GPU address of the fragment queue's auxiliary stamp word.
    pub(crate) aux_stamp_va: u64,
    /// Completion stamp value of the pass.
    pub(crate) stamp_value: u32,
    /// QID of the fragment queue.
    pub(crate) qid: u32,
    pub(crate) unk_2158: u32,
    /// Non-zero on the first kick of a queue pair.
    pub(crate) first_kick: u32,
    /// Completion value of the fragment kick (see
    /// [`KickTimestamp::completion_value`]).
    pub(crate) completion: u64,
    /// Current object ID of the fragment stage.
    pub(crate) current_id: u64,
    /// Submission ordinal of the pair.
    pub(crate) ordinal: u32,
    pub(crate) unk_2174: [u8; 0x24],
    /// GPU address the pass start timestamp is written to.
    pub(crate) timestamp_start_va: u64,
    /// GPU address the pass end timestamp is written to.
    pub(crate) timestamp_end_va: u64,
    pub(crate) unk_21a8: [u8; 0x24],
    pub(crate) unk_21cc: u8,
    pub(crate) unk_21cd: u8,
    /// GPU address of the USC free list's shared control object.
    pub(crate) free_list_control_va: u64,
    pub(crate) unk_21d6: u8,
    /// GPU address of the fragment status word, as seen by the pass's VM.
    pub(crate) status_va: u64,
    /// Firmware alias of the fragment status word.
    pub(crate) status_fw_va: u64,
    /// Allocation generation of the UAT context.
    pub(crate) context_generation: u8,
    pub(crate) unk_21e8: [u8; 0x20],
    pub(crate) unk_2208: u8,
    pub(crate) unk_2209: [u8; 3],
    pub(crate) unk_220c: u8,
    pub(crate) unk_220d: [u8; 0x20],
    pub(crate) unk_222d: u8,
    pub(crate) unk_222e: [u8; 0x12],
}

static_assert!(size_of::<FragmentDescriptor>() == 0x2240);
static_assert!(core::mem::offset_of!(FragmentDescriptor, checkpoints) == 0x80);
static_assert!(core::mem::offset_of!(FragmentDescriptor, pass_seq) == 0x90);
static_assert!(core::mem::offset_of!(FragmentDescriptor, arrays) == 0xa0);
static_assert!(core::mem::offset_of!(FragmentDescriptor, depth_bias_va) == 0x1d20);
static_assert!(core::mem::offset_of!(FragmentDescriptor, bg_rsrc_spec) == 0x1e78);
static_assert!(core::mem::offset_of!(FragmentDescriptor, tib_blocks) == 0x1f38);
static_assert!(core::mem::offset_of!(FragmentDescriptor, unk_1f50) == 0x1f50);
static_assert!(core::mem::offset_of!(FragmentDescriptor, eot_va) == 0x1f7c);
static_assert!(core::mem::offset_of!(FragmentDescriptor, depth_clear) == 0x1fa8);
static_assert!(core::mem::offset_of!(FragmentDescriptor, unk_2104) == 0x2104);
static_assert!(core::mem::offset_of!(FragmentDescriptor, stamp_va) == 0x2140);
static_assert!(core::mem::offset_of!(FragmentDescriptor, completion) == 0x2160);
static_assert!(core::mem::offset_of!(FragmentDescriptor, timestamp_start_va) == 0x2198);
static_assert!(core::mem::offset_of!(FragmentDescriptor, free_list_control_va) == 0x21ce);
static_assert!(core::mem::offset_of!(FragmentDescriptor, status_va) == 0x21d7);
static_assert!(core::mem::offset_of!(FragmentDescriptor, unk_222d) == 0x222d);

// SAFETY: `FragmentDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for FragmentDescriptor {}

impl FragmentDescriptor {
    /// Offset of register array `index` within the descriptor.
    const fn array_offset(index: usize) -> u64 {
        (core::mem::offset_of!(Self, arrays) + index * size_of::<RegisterArray>()) as u64
    }

    /// Returns the register arrays a fragment kick binds; `descriptor_va` is
    /// the descriptor's address as seen by the pass's VM.
    pub(crate) fn register_bindings(
        descriptor_va: u64,
        scs_register: bool,
        programs: RegisterPrograms,
    ) -> Result<[Option<RegisterArrayBinding>; 4]> {
        let array = |index, checkpoint: usize| -> Result<Option<RegisterArrayBinding>> {
            let va = offset_va(descriptor_va, Self::array_offset(index))?;
            Ok(Some(RegisterArrayBinding::new(va, checkpoint as u8)?))
        };
        let checkpoint = match programs == RegisterPrograms::V25G83 {
            true => FRAGMENT_CHECKPOINTS_25G83[0] as usize,
            false => FRAGMENT_CHECKPOINTS[0] as usize,
        } + usize::from(scs_register);
        Ok([
            array(0, checkpoint)?,
            array(1, PARTIAL_STORE_REGISTERS)?,
            array(2, PARTIAL_RESUME_REGISTERS)?,
            array(3, PARTIAL_LOAD_REGISTERS)?,
        ])
    }
}

/// A fragment descriptor followed by room for an MCache table that does not
/// fit its first register array.
#[repr(C)]
#[derive(Debug)]
pub(crate) struct FragmentSlot {
    /// The descriptor.
    pub(crate) descriptor: FragmentDescriptor,
    /// MCache table of more than [`MCACHE_INLINE_RANGES`] ranges.
    pub(crate) mcache: [McacheRange; MCACHE_RANGES_MAX],
    pub(crate) pad_2350: [u8; 0x30],
}

static_assert!(size_of::<FragmentSlot>() == FRAGMENT_SLOT_SIZE);

impl FragmentSlot {
    const INLINE_MCACHE_OFFSET: u64 =
        FragmentDescriptor::array_offset(0) + core::mem::offset_of!(RegisterArray, mcache) as u64;
    const SPILL_MCACHE_OFFSET: u64 = core::mem::offset_of!(Self, mcache) as u64;
    const UNK_2110: u32 = u32::MAX;
    const UNK_2124: [u32; 2] = [1, 1];
    const UNK_21CC: u8 = 0x53;
    const UNK_21D6: u8 = 1;
    const UNK_2208: u8 = 1;
    const UNK_220C: u8 = 1;
    const UNK_222D: u8 = 1;
    const UNK_1FAC_BASE: u64 = 0x300;
    const UNK_1FAC_TIB_SHIFT: u32 = 33;

    /// Writes the complete descriptor except the completion link and the
    /// MCache table.
    ///
    /// The spill table is left as it is: a kick entry names it only after
    /// [`Self::set_mcache`] rewrote it, so stale bytes there are never read.
    pub(crate) fn write(&mut self, args: &RenderArgs<'_>) -> Result {
        let pass = args.pass;
        let resources = args.resources;
        let geometry = Geometry::new(pass)?;
        let words = PassWords::new(pass)?;
        let base = args.fragment.descriptor_va;
        let array_va = |index| offset_va(base, FragmentDescriptor::array_offset(index));
        let arrays = [array_va(0)?, array_va(1)?, array_va(2)?, array_va(3)?];
        let timestamp_end_va = timestamp_end_va(resources.timestamp_va)?;
        let desc = &mut self.descriptor;

        clear(desc);
        desc.tag = CommandTag::Fragment as u32;
        desc.submit_sequence = args.ordinal.wrapping_mul(2);
        desc.context_id = resources.context_id.into();
        desc.kick_record_va = resources.kick_record_va;
        desc.pb_va = resources.pb_va;
        desc.scene_va = resources.scene_va;
        desc.scene_end_va = resources.scene_end_va;
        desc.tilemap_va = resources.tilemap_va;
        desc.msaa_control = pass.msaa_control;
        desc.samples = 1 << pass.samples_log2;
        desc.macro_size = geometry.macro_size as u32;
        desc.merge_upper_x = pass.merge_upper_x;
        desc.merge_upper_y = pass.merge_upper_y;
        desc.tiles = geometry.tiles_x * geometry.tiles_y;
        let scs_register = args.ray_query[1].scratch.enabled();
        let extra = u16::from(scs_register);
        desc.checkpoints = FRAGMENT_CHECKPOINTS.map(|count| count + extra);
        desc.entries = FRAGMENT_REGISTERS as u16 + extra;
        let registers = fragment_registers(args, &geometry, &words);
        if args.programs == RegisterPrograms::V25G83 {
            desc.checkpoints = FRAGMENT_CHECKPOINTS_25G83.map(|count| count + extra);
            desc.entries = FRAGMENT_REGISTERS_25G83 as u16 + extra;
            let (registers, count) = fragment_registers_25g83(args, &geometry, &words)?;
            desc.arrays[0].set(arrays[0], &registers[..count])?;
        } else if scs_register {
            desc.arrays[0].set(arrays[0], &registers)?;
        } else {
            desc.arrays[0].set_without(arrays[0], &registers, FRAGMENT_SCS_ENABLE_INDEX)?;
        }
        desc.arrays[1].set(arrays[1], &partial_store_registers(pass, &words))?;
        desc.arrays[2].set(arrays[2], &partial_resume_registers(pass, &words))?;
        desc.arrays[3].set(arrays[3], &partial_load_registers(pass, &words))?;
        desc.depth_bias_va = pass.depth_bias_va;
        desc.scissor_va = pass.scissor_va;
        desc.occlusion_query_va = pass.occlusion_query_va;
        desc.bg_rsrc_spec = words.bg_rsrc_spec;
        desc.bg_va = words.bg_va;
        desc.partial_bg_rsrc_spec = words.partial_bg_rsrc_spec;
        desc.partial_bg_va = words.partial_bg_va;
        desc.zls_swizzle = ZLS_SWIZZLE;
        desc.tib_blocks = words.tib_blocks;
        desc.aux_fb_flags = words.aux_fb_flags;
        desc.width = pass.width;
        desc.height = pass.height;
        desc.unk_1f50 = UNK_15049;
        desc.tile_config = words.tile_config;
        desc.eot_rsrc_spec = words.eot_rsrc_spec as u32;
        desc.eot_va = words.eot_va;
        desc.partial_eot_rsrc_spec = words.partial_eot_rsrc_spec as u32;
        desc.partial_eot_va = words.partial_eot_va;
        desc.depth_clear = pass.depth_clear;
        desc.unk_1fac = words.tib_blocks << Self::UNK_1FAC_TIB_SHIFT | Self::UNK_1FAC_BASE;
        desc.unk_2104 = resources.unk_2104;
        desc.predecessor_id = resources.fragment_ids.predecessor;
        desc.unk_210c = resources.unk_210c;
        desc.unk_2110 = Self::UNK_2110;
        desc.unk_2124 = Self::UNK_2124;
        desc.stamp_va = args.fragment.stamp_va;
        desc.aux_stamp_va = args.fragment.aux_stamp_va;
        desc.stamp_value = args.stamp_value();
        desc.qid = args.fragment.qid.into();
        desc.first_kick = u32::from(args.ordinal == 0);
        desc.current_id = resources.fragment_ids.current.into();
        desc.ordinal = args.ordinal as u32;
        desc.timestamp_start_va = resources.timestamp_va;
        desc.timestamp_end_va = timestamp_end_va;
        desc.unk_21cc = match args.programs == RegisterPrograms::V25G83 {
            true => FRAGMENT_SLOT_INDEX_25G83 + u8::from(args.ray_query[1].scratch.enabled()),
            false => Self::UNK_21CC,
        };
        desc.free_list_control_va = resources.free_list_control_va;
        desc.unk_21d6 = Self::UNK_21D6;
        desc.status_va = resources.fragment_status_va;
        desc.status_fw_va = resources.fragment_status_fw_va;
        desc.context_generation = resources.context_generation;
        desc.unk_2208 = Self::UNK_2208;
        desc.unk_220c = Self::UNK_220C;
        desc.unk_222d = Self::UNK_222D;
        Ok(())
    }

    /// Links the pass to the timestamp of its fragment kick.
    ///
    /// Must be called before the descriptor is published; see
    /// [`TaDescriptor::set_completion_link`].
    pub(crate) fn set_completion_link(&mut self, fragment: KickTimestamp, pass_seq: u8) {
        self.descriptor.pass_seq = pass_seq;
        self.descriptor.completion = fragment.completion_value(pass_seq);
    }

    /// Writes the MCache table of the pass: the auxiliary framebuffer at
    /// `aux_fb_va`, then `attachments`, translated in UAT context `context`.
    /// `slot_va` is the GPU address of this slot as seen by the pass's VM.
    ///
    /// Returns the table for the fragment kick entry, or `None` when the pass
    /// has no attachments.
    pub(crate) fn set_mcache(
        &mut self,
        slot_va: u64,
        aux_fb_va: u64,
        attachments: &[WriteRange],
        context: u8,
    ) -> Result<Option<McacheTable>> {
        if attachments.is_empty() {
            return Ok(None);
        }
        let count = attachments.len() + 1;
        if count > MCACHE_RANGES_MAX {
            return Err(EINVAL);
        }
        let (offset, ranges) = if count <= MCACHE_INLINE_RANGES {
            (
                Self::INLINE_MCACHE_OFFSET,
                &mut self.descriptor.arrays[0].mcache[..count],
            )
        } else {
            (Self::SPILL_MCACHE_OFFSET, &mut self.mcache[..count])
        };
        let table = McacheTable::new(offset_va(slot_va, offset)?, count)?;
        let aux_fb = WriteRange {
            va: aux_fb_va,
            size: AUX_FB_SIZE,
        };
        ranges[0] = McacheRange::new(&aux_fb, McacheClass::AuxFramebuffer, context)?;
        for (range, attachment) in ranges[1..].iter_mut().zip(attachments) {
            *range = McacheRange::new(attachment, McacheClass::Attachment, context)?;
        }
        Ok(Some(table))
    }
}
