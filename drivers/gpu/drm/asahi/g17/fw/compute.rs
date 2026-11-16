// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Compute work descriptor.
//!
//! A compute kick launches one [`ComputeDescriptor`]. The descriptor embeds
//! the two register arrays its kick entry binds: the main array and a
//! four-entry array that writes the operand-state registers with their
//! regions exchanged.

use kernel::prelude::*;

use super::kick::{KickTimestamp, RegisterArray, RegisterArrayBinding, RegisterWrite};
use super::queue::CommandTag;
use super::{
    offset_va, sampler_max, stamp_value, timestamp_end_va, work_key, ObjectIds, UNK_0A599,
    UNK_0D411, UNK_101D9, UNK_10791, UNK_1A0E9, UNK_1C8F8,
};

/// Size of a compute descriptor.
pub(crate) const COMPUTE_DESCRIPTOR_SIZE: usize = 0x4000;
/// QoS class of compute kicks.
pub(crate) const COMPUTE_QOS_CLASS: u8 = 0x08;
/// Event mask of compute kicks.
pub(crate) const COMPUTE_KICK_EVENT_MASK: [u64; 4] = [0, 0, 4, 0];

const COMPUTE_REGISTERS: usize = 41;
const COMPUTE_REGISTERS_MAX: usize = COMPUTE_REGISTERS + 1;

/// Validated per-command shader-context scratch request.
///
/// The layout word is Apple's packed SCS layout: depth (bits 0:15), levels
/// (16:23), cliques (24:31), user (32:47) and driver (48:63).  The region the
/// hardware needs is derived with G17P's Gen1 section model: fixed prefixes
/// totalling [`Self::FIXED_BYTES`], then per-thread stack, per-thread driver
/// and per-thread user bytes. Gen1 has no per-clique level payload. The scratch
/// registers the program writes are the native `generateRegisterList`
/// formulas use `com.apple.AGXG17P`; the
/// disabled program keeps the native disabled constants.
#[derive(Clone, Copy)]
pub(crate) struct ScratchRequest {
    pages: u16,
    window: u64,
    stack: u64,
    driver: u64,
    tail: u64,
}

impl ScratchRequest {
    /// Native ScsLayoutGen1::getSectionEnd(4).
    const FIXED_BYTES: u64 = 0x58000;
    const THREADS_PER_CLIQUE: u64 = 32;
    const BYTES_PER_STACK_ENTRY: u64 = 8;
    const PAGE_SHIFT: u32 = 12;
    const PAGE_SIZE: u64 = 1 << Self::PAGE_SHIFT;
    const PAGES_PER_MIB: u64 = 1 << (20 - Self::PAGE_SHIFT);
    /// Window word when enabled: bit 24 plus the clique count at 16:23.
    const WINDOW_ENABLED: u64 = 1 << 24;
    /// Native disabled window word, [`UNK_10791`].
    const WINDOW_DISABLED: u64 = UNK_10791;
    /// Native disabled stack word, [`UNK_1A0E9`].
    const STACK_DISABLED: u64 = UNK_1A0E9;
    /// Native tail reservation when enabled.
    const TAIL_ENABLED: u64 = 8;

    /// Payload-free layouts request no scratch even if depth/cliques are set.
    /// Otherwise derive the Gen1 end and round it up with the native two-page
    /// guard; the per-register words are the native formulas, so they are
    /// checked here against Apple's own values before they reach hardware.
    pub(crate) fn new(layout: u64) -> Result<Self> {
        let depth = layout & u64::from(u16::MAX);
        let cliques = (layout >> 24) & u64::from(u8::MAX);
        let user = (layout >> 32) & u64::from(u16::MAX);
        let driver = (layout >> 48) & u64::from(u16::MAX);
        if user == 0 && driver == 0 {
            return Ok(Self {
                pages: 0,
                window: Self::WINDOW_DISABLED,
                stack: Self::STACK_DISABLED,
                driver: 0,
                tail: 0,
            });
        }
        let threads = cliques.checked_mul(Self::THREADS_PER_CLIQUE).ok_or(EINVAL)?;
        let stack_bytes = threads
            .checked_mul(depth.checked_mul(Self::BYTES_PER_STACK_ENTRY).ok_or(EINVAL)?)
            .ok_or(EINVAL)?;
        let driver_bytes = threads.checked_mul(driver).ok_or(EINVAL)?;
        let user_bytes = threads.checked_mul(user).ok_or(EINVAL)?;
        let end10 = Self::FIXED_BYTES
            .checked_add(stack_bytes)
            .and_then(|bytes| bytes.checked_add(driver_bytes))
            .ok_or(EINVAL)?;
        let end = end10
            .checked_add(user_bytes)
            .and_then(|bytes| bytes.checked_add(2 * Self::PAGE_SIZE - 1))
            .ok_or(EINVAL)?;
        let pages = end >> Self::PAGE_SHIFT;
        Ok(Self {
            pages: u16::try_from(pages).map_err(|_| EINVAL)?,
            window: Self::WINDOW_ENABLED | (cliques << 16),
            stack: depth & !7,
            // G17P's native register list does not publish the Gen2 level
            // region register. Preserve the historical zero slot.
            driver: 0,
            tail: Self::TAIL_ENABLED,
        })
    }

    pub(crate) fn enabled(self) -> bool {
        self.pages != 0
    }

    pub(crate) fn pages(self) -> u16 { self.pages }

    pub(crate) fn size_word(self) -> u64 {
        let pages = u64::from(self.pages);
        pages | (pages.div_ceil(Self::PAGES_PER_MIB) << 16)
    }

    pub(crate) fn window_word(self) -> u64 {
        self.window
    }

    pub(crate) fn stack_word(self) -> u64 {
        self.stack
    }

    fn driver_word(self) -> u64 {
        self.driver
    }

    pub(crate) fn tail_word(self) -> u64 {
        self.tail
    }

    fn checkpoints(self) -> [u16; 4] {
        CHECKPOINTS.map(|count| count + u16::from(self.enabled()))
    }
}
const ALT_REGISTERS: usize = 4;

/// Object IDs of every compute kick.
const OBJECT_IDS: ObjectIds = ObjectIds {
    predecessor: 0x0100_01d7,
    current: 0x0200_01dc,
};
/// Values of the first and second write of register 0x1a440.
const UNK_1A440: u64 = 0x0000_0001_5402_4201;
const UNK_1A440_FINAL: u64 = 0x0000_0001_5402_4209;
/// Value of register 0x1a458: [`UNK_1C8F8`] below bits 31:20 = 0x10c.
const UNK_1A458: u64 = 0x10c0_0000 | UNK_1C8F8;

/// Offsets in the preempt buffer of the four state words the program names.
const PREEMPT_STATE: [u64; 4] = [0x1480, 0x1488, 0x1490, 0x1498];
/// Layout of a queue's operand-state buffer, which depends on the GPU's core layout: offsets of
/// the regions the program names (two preemption banks of three 0x800-byte records per core,
/// tagged | 5 at their start, then their end, then the per-cluster tails) and its size.
pub(crate) struct OperandState {
    r10229: u64,
    r140a8: u64,
    r10099: u64,
    r10091: u64,
    r0a5c1: u64,
    r0a5c9: u64,
    /// Size of the buffer.
    pub(crate) size: usize,
}

impl OperandState {
    /// One cluster of six cores (A18 Pro).
    pub(crate) const ONE_BY_SIX: OperandState = OperandState {
        r10229: 0x12800,
        r140a8: 0x13000,
        r10099: 0x9405,
        r10091: 0x12400,
        r0a5c1: 0x0005,
        r0a5c9: 0x9000,
        size: 0x14000,
    };
    /// Two clusters of ten cores (M4 Pro): bank stride 0x1e400, tails 0x1000.
    pub(crate) const TWO_BY_TEN: OperandState = OperandState {
        r10229: 0x3c800,
        r140a8: 0x3d800,
        r10099: 0x1e405,
        r10091: 0x3c400,
        r0a5c1: 0x0005,
        r0a5c9: 0x1e000,
        size: 0x44000,
    };
}

/// Checkpoints into the main register array, as entry counts: after the
/// first 0x0a5c9 write, after the final 0x1a440 write (twice), and the full
/// count.
const CHECKPOINTS: [u16; 4] = [39, 40, 40, COMPUTE_REGISTERS as u16];

#[derive(Clone, Copy)]
pub(crate) struct IntersectionTables {
    pub(crate) pool: u64,
    pub(crate) pool_bytes: u64,
    auxiliary: u32,
    first: u32,
    count: u32,
    pub(crate) default_program: u64,
    default_control: u64,
}

impl IntersectionTables {
    pub(crate) const NONE: Self = Self {
        pool: 0, pool_bytes: 0, auxiliary: 0, first: 0, count: 0,
        default_program: 0, default_control: 0,
    };

    pub(crate) fn new(pool: u64, pool_bytes: u64, first: u32, count: u32,
                      default_program: u64, default_control: u64) -> Result<Self> {
        if pool == 0 {
            if pool_bytes != 0 || first != 0 || count != 0
                || default_program != 0 || default_control != 0 {
                return Err(EINVAL);
            }
            return Ok(Self::NONE);
        }
        // G17P HAL200 Gen1 publishes a byte-sized table length (25 bits),
        // with a plain aligned address. Unlike G17X HAL300, its address has
        // no bit 51 and its two pool registers use fixed native values.
        // Keep the shared UAPI's 128-byte blocks and 16 KiB allocation units.
        let end = pool.checked_add(pool_bytes).ok_or(EINVAL)?;
        let blocks_end = u64::from(first) + u64::from(count);
        let code = default_program & 0x0000_ffff_ffff_ffff;
        if pool & 0x3fff != 0 || pool_bytes == 0 || pool_bytes & 0x3fff != 0
            || end > (1u64 << 48) || (pool_bytes >> 14) >= (1u64 << 25)
            || count == 0 || count >= (1 << 18) || blocks_end * 128 > pool_bytes
            || code & 63 != 0
            || default_control & !0x0000_7fff_0000_3fff != 0
            || (code == 0 && (default_program != 0 || default_control != 0))
            || (code != 0 && default_control & (1u64 << 40) == 0) {
            return Err(EINVAL);
        }
        Ok(Self { pool, pool_bytes, auxiliary: 0, first, count, default_program, default_control })
    }

    pub(crate) fn parse(bytes: &[u8]) -> Result<Self> {
        if bytes.get(48..).is_some_and(|tail| tail.iter().any(|&b| b != 0)) {
            return Err(EINVAL);
        }
        let mut tail = [0u8; 48];
        let count = bytes.len().min(tail.len());
        tail[..count].copy_from_slice(&bytes[..count]);
        let auxiliary = u32::from_le_bytes(tail[..4].try_into().unwrap());
        if tail[4..8].iter().any(|&b| b != 0)
            || (auxiliary != 0 && !(8..=32).contains(&auxiliary)) {
            return Err(EINVAL);
        }
        let mut tables = Self::new(
            u64::from_le_bytes(tail[8..16].try_into().unwrap()),
            u64::from_le_bytes(tail[16..24].try_into().unwrap()),
            u32::from_le_bytes(tail[24..28].try_into().unwrap()),
            u32::from_le_bytes(tail[28..32].try_into().unwrap()),
            u64::from_le_bytes(tail[32..40].try_into().unwrap()),
            u64::from_le_bytes(tail[40..48].try_into().unwrap()),
        )?;
        if auxiliary != 0 && !tables.enabled() { return Err(EINVAL); }
        tables.auxiliary = auxiliary;
        Ok(tables)
    }

    pub(crate) fn enabled(self) -> bool { self.pool != 0 }

    pub(crate) fn words(self) -> [u64; 6] {
        if !self.enabled() { return [0; 6]; }
        // AGXMetalG17P 26A428 endComputePass, both command encodings;
        // pinned AGXG17P processComputeSetup copies these six words as-is.
        [self.pool + u64::from(self.first) * 128,
         0xc0000000 | (u64::from(self.count) << 7), 0x6f00000000,
         0x3feff, self.default_program, self.default_control]
    }
}


/// Inputs of one compute descriptor.
pub(crate) struct ComputeArgs {
    pub(crate) intersection: IntersectionTables,
    /// Scratch request validated with the userspace command.
    pub(crate) scratch: ScratchRequest,
    /// Base of the queue's USC window.
    pub(crate) usc_base: u64,
    /// GPU address of the control stream.
    pub(crate) cdm_va: u64,
    /// GPU address of the control-stream terminator.
    pub(crate) cdm_end_va: u64,
    /// GPU address of the sampler heap, or zero.
    pub(crate) sampler_heap_va: u64,
    /// Number of samplers in the heap.
    pub(crate) sampler_count: u32,
    /// GPU address of this descriptor, as seen by the work's VM.
    pub(crate) descriptor_va: u64,
    /// UAT context the work runs in.
    pub(crate) context_id: u16,
    /// Allocation generation of that context.
    pub(crate) context_generation: u8,
    /// State word of the logical queue's work-state node.
    pub(crate) work_state: u32,
    /// GPU address of the logical queue's work-state node.
    pub(crate) work_state_va: u64,
    /// Queue the work is kicked on.
    pub(crate) qid: u8,
    /// Number of earlier kicks of the queue.
    pub(crate) kick_count: u32,
    /// Timestamp the work is kicked at.
    pub(crate) kick: KickTimestamp,
    /// GPU address of the preempt buffer, as seen by the work's VM.
    pub(crate) preempt_va: u64,
    /// GPU address of the operand-state buffer, as seen by the work's VM.
    pub(crate) operand_state_va: u64,
    /// Layout of the operand-state buffer.
    pub(crate) operand: &'static OperandState,
    /// GPU address of the private-memory usage record, as seen by the work's
    /// VM.
    pub(crate) usage_va: u64,
    /// Firmware alias of the private-memory usage record.
    pub(crate) usage_fw_va: u64,
    /// Buffer slot of the USC private-memory free list.
    pub(crate) free_list_slot: u32,
    /// GPU address of the free list's shared control object.
    pub(crate) free_list_control_va: u64,
    /// GPU address the completion stamp value is stored to.
    pub(crate) stamp_va: u64,
    /// GPU address of the word following the completion stamp.
    pub(crate) aux_stamp_va: u64,
    /// GPU addresses of the two status words.
    pub(crate) status_va: [u64; 2],
    /// GPU address of the start/end timestamp pair.
    pub(crate) timestamp_va: u64,
}

fn compute_registers(args: &ComputeArgs) -> Result<[RegisterWrite; COMPUTE_REGISTERS_MAX]> {
    let preempt = args.preempt_va;
    let operand = args.operand_state_va;
    let state = args.operand;
    let key = work_key(args.context_id, args.work_state);
    let ids = OBJECT_IDS.word();
    let intersection = args.intersection.words();

    Ok([
        RegisterWrite::new(0x017d9, 1),
        RegisterWrite::new(0x1a510, preempt),
        RegisterWrite::new(0x1a420, args.cdm_va),
        RegisterWrite::new(0x1a4d0, offset_va(preempt, PREEMPT_STATE[0])?),
        RegisterWrite::new(0x1a4d8, offset_va(preempt, PREEMPT_STATE[1])?),
        RegisterWrite::new(0x1a4e0, offset_va(preempt, PREEMPT_STATE[2])?),
        RegisterWrite::new(0x1a4e8, offset_va(preempt, PREEMPT_STATE[3])?),
        RegisterWrite::new(0x10071, args.usc_base),
        // Helper program binary, data and configuration: none.
        RegisterWrite::new(0x11841, 0),
        RegisterWrite::new(0x11849, 0),
        RegisterWrite::new(0x11f81, 0),
        RegisterWrite::new(0x1a440, UNK_1A440),
        RegisterWrite::new(0x1a458, UNK_1A458),
        RegisterWrite::new(0x101d9, UNK_101D9),
        RegisterWrite::new(0x1a089, intersection[0]),
        RegisterWrite::new(0x1a091, intersection[1]),
        RegisterWrite::new(0x1a059, intersection[2]),
        RegisterWrite::new(0x1a061, intersection[3]),
        RegisterWrite::new(0x1a0b9, intersection[4]),
        RegisterWrite::new(0x1a0c1, intersection[5]),
        RegisterWrite::new(0x101d1, args.scratch.size_word()),
        RegisterWrite::new(0x0d479, if args.intersection.auxiliary == 0 {
            args.scratch.tail_word()
        } else {
            u64::from(args.intersection.auxiliary)
        }),
        RegisterWrite::new(0x1a0e9, args.scratch.stack_word()),
        RegisterWrite::new(0x1a141, args.scratch.driver_word()),
        RegisterWrite::new(0x107a1, args.scratch.window_word()),
        RegisterWrite::new(0x0a599, UNK_0A599),
        RegisterWrite::new(0x0d411, UNK_0D411),
        RegisterWrite::new(0x1a540, ids),
        RegisterWrite::new(0x014a9, ids),
        RegisterWrite::new(0x0a351, ids),
        RegisterWrite::new(0x10201, key),
        RegisterWrite::new(0x10428, key),
        RegisterWrite::new(0x14028, u64::from(args.free_list_slot)),
        RegisterWrite::new(0x14070, args.usage_va | 1),
        RegisterWrite::new(0x10229, offset_va(operand, state.r10229)?),
        RegisterWrite::new(0x140a8, offset_va(operand, state.r140a8)?),
        RegisterWrite::new(0x10099, offset_va(operand, state.r10099)?),
        RegisterWrite::new(0x10091, offset_va(operand, state.r10091)?),
        RegisterWrite::new(0x0a5c1, offset_va(operand, state.r0a5c1)?),
        RegisterWrite::new(0x0a5c9, offset_va(operand, state.r0a5c9)?),
        RegisterWrite::new(0x1a440, UNK_1A440_FINAL),
        RegisterWrite::new(0x0a599, UNK_0A599),
    ])
}

/// The second array exchanges the operand-state regions of registers
/// 0x10099/0x0a5c1 and 0x10091/0x0a5c9.
fn alt_registers(args: &ComputeArgs) -> Result<[RegisterWrite; ALT_REGISTERS]> {
    let operand = args.operand_state_va;
    let state = args.operand;
    Ok([
        RegisterWrite::new(0x10099, offset_va(operand, state.r0a5c1)?),
        RegisterWrite::new(0x10091, offset_va(operand, state.r0a5c9)?),
        RegisterWrite::new(0x0a5c1, offset_va(operand, state.r10099)?),
        RegisterWrite::new(0x0a5c9, offset_va(operand, state.r10091)?),
    ])
}

/// Compute work descriptor (tag 3).
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct ComputeDescriptor {
    /// [`CommandTag::Compute`].
    pub(crate) tag: u32,
    pub(crate) unk_04: u64,
    /// UAT context of the work.
    pub(crate) context_id: u32,
    /// GPU address of the logical queue's work-state node.
    pub(crate) work_state_va: u64,
    /// Checkpoints into the main register array.
    pub(crate) checkpoints: [u16; 4],
    pub(crate) unk_20: [u8; 0x20],
    /// Main register array.
    pub(crate) registers: RegisterArray,
    /// Second register array.
    pub(crate) alt_registers: RegisterArray,
    pub(crate) unk_e80: [u8; 0x58],
    /// GPU address of the preempt buffer.
    pub(crate) preempt_va: u64,
    /// GPU address of the control-stream terminator.
    pub(crate) cdm_end_va: u64,
    pub(crate) unk_ee8: [u8; 0x20],
    /// First value of register 0x1a440.
    pub(crate) unk_f08: u64,
    pub(crate) unk_f10: [u8; 0x10],
    /// Predecessor object ID.
    pub(crate) predecessor_id: u32,
    pub(crate) unk_f24: u32,
    pub(crate) unk_f28: u32,
    /// GPU address of the sampler heap.
    pub(crate) sampler_heap_va: u64,
    /// Number of samplers.
    pub(crate) sampler_count: u32,
    /// Number of samplers plus one, or zero without samplers.
    pub(crate) sampler_max: u32,
    pub(crate) unk_f3c: u32,
    /// GPU address the firmware stores `stamp_value` to on completion.
    pub(crate) stamp_va: u64,
    /// GPU address of the word following the completion stamp.
    pub(crate) aux_stamp_va: u64,
    /// Completion stamp value of the kick timestamp.
    pub(crate) stamp_value: u32,
    /// QID of the compute queue.
    pub(crate) qid: u32,
    pub(crate) unk_f58: u32,
    pub(crate) unk_f5c: u32,
    /// Kick timestamp.
    pub(crate) kick_timestamp: u64,
    /// Current object ID.
    pub(crate) current_id: u64,
    /// Number of earlier kicks of the queue.
    pub(crate) kick_count: u32,
    pub(crate) unk_f74: [u8; 8],
    /// GPU addresses of the two status words.
    pub(crate) status_va: [u64; 2],
    /// GPU address the start timestamp is written to.
    pub(crate) timestamp_start_va: u64,
    /// GPU address the end timestamp is written to.
    pub(crate) timestamp_end_va: u64,
    pub(crate) unk_f9c: [u8; 0x14],
    pub(crate) unk_fb0: u16,
    /// GPU address of the USC free list's shared control object.
    pub(crate) free_list_control_va: u64,
    pub(crate) unk_fba: u32,
    pub(crate) unk_fbe: [u8; 7],
    pub(crate) unk_fc5: u8,
    pub(crate) unk_fc6: [u8; 5],
    /// Firmware alias of the private-memory usage record.
    pub(crate) usage_va: u64,
    /// Allocation generation of the UAT context.
    pub(crate) context_generation: u8,
    pub(crate) unk_fd4: [u8; 0x302c],
}

static_assert!(size_of::<ComputeDescriptor>() == COMPUTE_DESCRIPTOR_SIZE);
static_assert!(core::mem::offset_of!(ComputeDescriptor, registers) == 0x40);
static_assert!(core::mem::offset_of!(ComputeDescriptor, alt_registers) == 0x760);
static_assert!(core::mem::offset_of!(ComputeDescriptor, preempt_va) == 0xed8);
static_assert!(core::mem::offset_of!(ComputeDescriptor, unk_f08) == 0xf08);
static_assert!(core::mem::offset_of!(ComputeDescriptor, predecessor_id) == 0xf20);
static_assert!(core::mem::offset_of!(ComputeDescriptor, sampler_heap_va) == 0xf2c);
static_assert!(core::mem::offset_of!(ComputeDescriptor, stamp_va) == 0xf40);
static_assert!(core::mem::offset_of!(ComputeDescriptor, kick_timestamp) == 0xf60);
static_assert!(core::mem::offset_of!(ComputeDescriptor, current_id) == 0xf68);
static_assert!(core::mem::offset_of!(ComputeDescriptor, status_va) == 0xf7c);
static_assert!(core::mem::offset_of!(ComputeDescriptor, free_list_control_va) == 0xfb2);
static_assert!(core::mem::offset_of!(ComputeDescriptor, usage_va) == 0xfcb);
static_assert!(core::mem::offset_of!(ComputeDescriptor, context_generation) == 0xfd3);

// SAFETY: `ComputeDescriptor` consists of integers and arrays of integers only.
unsafe impl Zeroable for ComputeDescriptor {}

impl ComputeDescriptor {
    const REGISTERS_OFFSET: u64 = core::mem::offset_of!(Self, registers) as u64;
    const ALT_REGISTERS_OFFSET: u64 = core::mem::offset_of!(Self, alt_registers) as u64;
    const UNK_F28: u32 = u32::MAX;
    const UNK_FB0: u16 = 0x1a;
    const UNK_FBA: u32 = 0xe0a0_0001;
    const UNK_FC5: u8 = 0x9f;

    /// Fills a descriptor cleared by `Object::initialize`. The unused firmware
    /// fields and padding retain that zero initialization.
    pub(crate) fn write_zeroed(&mut self, args: &ComputeArgs) -> Result {
        if args.cdm_end_va < args.cdm_va
            || (args.sampler_heap_va == 0) != (args.sampler_count == 0)
            || args.sampler_heap_va & 7 != 0
        {
            return Err(EINVAL);
        }
        let registers = compute_registers(args)?;
        let alt_registers = alt_registers(args)?;
        let registers_va = offset_va(args.descriptor_va, Self::REGISTERS_OFFSET)?;
        let alt_registers_va = offset_va(args.descriptor_va, Self::ALT_REGISTERS_OFFSET)?;
        let timestamp_end_va = timestamp_end_va(args.timestamp_va)?;
        let sampler_max = sampler_max(args.sampler_count)?;

        self.tag = CommandTag::Compute as u32;
        self.context_id = args.context_id.into();
        self.work_state_va = args.work_state_va;
        self.checkpoints = args.scratch.checkpoints();
        // The enabled program prepends one register; disabled descriptors keep
        // the same forty writes, trailer count and resume checkpoints.
        let first = usize::from(!args.scratch.enabled());
        self.registers.set(registers_va, &registers[first..])?;
        self.alt_registers.set(alt_registers_va, &alt_registers)?;
        self.preempt_va = args.preempt_va;
        self.cdm_end_va = args.cdm_end_va;
        self.unk_f08 = UNK_1A440;
        self.predecessor_id = OBJECT_IDS.predecessor;
        self.unk_f28 = Self::UNK_F28;
        self.sampler_heap_va = args.sampler_heap_va;
        self.sampler_count = args.sampler_count;
        self.sampler_max = sampler_max;
        self.stamp_va = args.stamp_va;
        self.aux_stamp_va = args.aux_stamp_va;
        self.stamp_value = stamp_value(args.kick.get() as u32);
        self.qid = args.qid.into();
        self.kick_timestamp = args.kick.get();
        self.current_id = OBJECT_IDS.current.into();
        self.kick_count = args.kick_count;
        self.status_va = args.status_va;
        self.timestamp_start_va = args.timestamp_va;
        self.timestamp_end_va = timestamp_end_va;
        self.unk_fb0 = Self::UNK_FB0;
        self.free_list_control_va = args.free_list_control_va;
        self.unk_fba = Self::UNK_FBA;
        self.unk_fc5 = Self::UNK_FC5;
        self.usage_va = args.usage_fw_va;
        self.context_generation = args.context_generation;
        Ok(())
    }

    /// Returns the register arrays a compute kick binds; `descriptor_va` is
    /// the descriptor's address as seen by the work's VM.
    pub(crate) fn register_bindings(
        descriptor_va: u64,
        scratch: ScratchRequest,
    ) -> Result<[Option<RegisterArrayBinding>; 4]> {
        let registers = RegisterArrayBinding::new(
            offset_va(descriptor_va, Self::REGISTERS_OFFSET)?,
            scratch.checkpoints()[0] as u8,
        )?;
        let alt_registers = RegisterArrayBinding::new(
            offset_va(descriptor_va, Self::ALT_REGISTERS_OFFSET)?,
            ALT_REGISTERS as u8,
        )?;
        Ok([Some(registers), Some(alt_registers), None, None])
    }
}

/// One retained queue-context record. The descriptor pointer is refreshed for each compute
/// publication; the remaining fields are installed once with the queue.
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct QueueContextItem {
    unk_00: u64,
    unk_08: u64,
    descriptor_va: u64,
    queue_va: u64,
    unk_20: u64,
    qid: u64,
    unk_30: [u8; 0x100],
    unk_130: u64,
    unk_138: u64,
    unk_140: [u8; 0x10],
    unk_150: u64,
    unk_158: u64,
    unk_160: [u8; 0x18],
    unk_178: u64,
    unk_180: [u8; 0x80],
}
impl QueueContextItem {
    pub(crate) fn new(descriptor_va: u64, queue_va: u64, qid: u8) -> Self {
        Self {
            unk_00: 0x1000_1000_0000_0004,
            unk_08: 0,
            descriptor_va,
            queue_va,
            unk_20: 0xffff_0801_0000_0001,
            qid: u64::from(qid) << 40,
            unk_30: [0; 0x100],
            unk_130: 0,
            unk_138: 4,
            unk_140: [0; 0x10],
            unk_150: 0x0001_1003_8001_a002,
            unk_158: 0x0000_2003_8001_a03b,
            unk_160: [0; 0x18],
            unk_178: 0x003f_ffff_ffff_ffff,
            unk_180: [0; 0x80],
        }
    }
}
static_assert!(size_of::<QueueContextItem>() == 0x200);
