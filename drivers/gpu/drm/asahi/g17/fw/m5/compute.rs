// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! M5 compute register programs. The operand and preemption layouts differ
//! from the other G17 profile; sharing a register count does not imply a shared
//! register program. M5 scratch has its own layout and two control writes.

use kernel::prelude::*;
use super::super::{kick::RegisterWrite, offset_va, work_key};
use super::initdata::Board;

pub(crate) const MAIN_COUNT: usize = 40;
pub(crate) const ALTERNATE_COUNT: usize = 4;
pub(crate) const PREEMPT_BYTES: usize = 0x4000;
pub(crate) const CHECKPOINTS: [u16; 4] = [37, 39, 39, 40];

/// A dispatch control word for `board`: above four clusters it carries the cluster-topology term,
/// `0x9c` in the form for the firmware feature the 27.0 profile leaves clear (`0x1c` is the
/// feature-set form). The term is an OR. Compute and render dispatch words both take it.
pub(crate) fn dispatch_control(board: &Board, base: u64) -> u64 {
    topology_dispatch_control(base, board.clusters)
}

/// [`dispatch_control`] for `clusters`.
pub(crate) const fn topology_dispatch_control(base: u64, clusters: u32) -> u64 {
    if clusters > 4 { base | 0x9c } else { base }
}

const _: () = {
    assert!(topology_dispatch_control(0x14c0_8a60, 8) == 0x14c0_8afc);
    assert!(topology_dispatch_control(0x10c0_8860, 8) == 0x10c0_88fc);
    assert!(topology_dispatch_control(0x14c0_8a60, 4) == 0x14c0_8a60);
};

/// The SCS scratch allocation. Pages are 4 KiB USC pool pages, independent
/// of the host's CPU page size. Layout sections and control registers are
/// specific to M5; the other G17 profile uses a different fixed region.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct ScratchRequest {
    layout: u64,
    driver_end: u32,
    pages: u16,
}

impl ScratchRequest {
    pub(crate) const NONE: Self = Self { layout: 0, driver_end: 0, pages: 0 };

    pub(crate) fn new(layout: u64) -> Result<Self> {
        let cliques = (layout >> 24) & 0xff;
        if cliques == 0 { return Ok(Self::NONE); }
        let depth = layout & 0xffff;
        let levels = (layout >> 16) & 0xff;
        let user = (layout >> 32) & 0xffff;
        let driver = layout >> 48;
        let threads = cliques * 32;
        // Four fixed sections followed by per-thread stacks, driver/user
        // payloads and per-level state. All inputs are bounded bitfields.
        let driver_end = 0x68000 + threads * (depth * 8 + driver);
        // The register naming the end of the driver section holds a 24-bit
        // byte offset. Reject truncation even if the total page count fits.
        if driver_end > 0xffffff { return Err(EINVAL); }
        let bytes = driver_end + threads * user
            + cliques * 256 * (levels + u64::from(levels > 1));
        let pages = (bytes + 0x1fff) >> 12;
        Ok(Self { layout, driver_end: driver_end as u32,
            pages: u16::try_from(pages).map_err(|_| EINVAL)? })
    }

    pub(crate) fn enabled(self) -> bool { self.pages != 0 }
    pub(crate) fn layout(self) -> u64 { self.layout }
    pub(crate) fn pages(self) -> u16 { self.pages }
    pub(crate) fn size_word(self) -> u64 {
        let pages = u64::from(self.pages);
        pages | (pages.div_ceil(256) << 16)
    }
    pub(crate) fn window_word(self) -> u64 {
        if self.enabled() { (1 << 24) | (((self.layout >> 24) & 0xff) << 16) }
        else { 0xff0000 }
    }
    pub(crate) fn stack_word(self) -> u64 {
        if self.enabled() { self.layout & 0xfff8 } else { 8 }
    }
    pub(crate) fn driver_word(self) -> u64 {
        if !self.enabled() { return 0; }
        let levels = (self.layout >> 16) & 0xff;
        u64::from(self.driver_end) | ((levels + u64::from(levels > 1)) << 24)
    }
    fn checkpoints(self) -> [u16; 4] {
        CHECKPOINTS.map(|count| count + if self.enabled() { 2 } else { 0 })
    }
}

/// Per-command intersection table bindings. The pool contains 128-byte
/// blocks of eight 16-byte program entries. Addresses are UAT user addresses;
/// ownership and readable ranges are checked by command validation.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct IntersectionTables {
    pub(crate) pool: u64,
    pub(crate) pool_bytes: u64,
    first: u32,
    count: u32,
    pub(crate) default_program: u64,
    default_control: u64,
}

impl IntersectionTables {
    pub(crate) const NONE: Self = Self {
        pool: 0, pool_bytes: 0, first: 0, count: 0,
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
        // Resource sizes are encoded in 16 KiB pages. Table lengths
        // have 18 block-count bits; truncating either field changes addressing.
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
        Ok(Self { pool, pool_bytes, first, count, default_program, default_control })
    }

    pub(crate) fn enabled(self) -> bool { self.pool != 0 }

    pub(crate) fn words(self) -> [u64; 6] {
        if !self.enabled() { return [0; 6]; }
        [self.pool + u64::from(self.first) * 128 | (1u64 << 51),
         0xc0000000 | (u64::from(self.count) << 7), self.pool,
         self.pool_bytes >> 14, self.default_program, self.default_control]
    }
}

/// Addresses and policy owned by a queued compute command.
pub(crate) struct RegisterArgs {
    pub(crate) matrix_ops: bool,
    pub(crate) scratch: ScratchRequest,
    pub(crate) intersection: IntersectionTables,
    /// Userspace's opaque auxiliary allocation word, independent of the SCS layout.
    pub(crate) auxiliary_allocation: u32,
    pub(crate) preempt_va: u64,
    pub(crate) cdm_va: u64,
    pub(crate) operand_va: u64,
    pub(crate) usage_va: u64,
    /// Private-memory pool slot, as named by the usage record.
    pub(crate) free_list_slot: u32,
    pub(crate) context_id: u16,
    pub(crate) work_state: u32,
    /// Packed predecessor/current object identifiers.
    pub(crate) object_ids: u64,
    /// Pool target and resource policy for the program's first resource-policy write.
    pub(crate) resource_policy: u64,
    /// Start of the four eight-byte state words in the preemption buffer.
    pub(crate) preempt_state_offset: u32,
    /// Size of one operand-state region; the program exchanges two regions.
    pub(crate) operand_region_bytes: u32,
    pub(crate) operand_aux_offset: u32,
    /// Size of the operand-state buffer at `operand_va`.
    pub(crate) operand_bytes: usize,
    pub(crate) resource_control: u64,
    pub(crate) dispatch_control: u64,
    pub(crate) auxiliary_control: u64,
}

impl RegisterArgs {
    fn validate(&self) -> Result {
        if self.preempt_va & 0x3fff != 0 || self.operand_va & 0x3fff != 0
            || self.cdm_va & 7 != 0 || self.usage_va & 7 != 0 || self.context_id >= 64 {
            return Err(EINVAL);
        }
        offset_va(self.preempt_va, PREEMPT_BYTES as u64)?;
        offset_va(self.operand_va, self.operand_bytes as u64)?;
        if self.preempt_state_offset & 7 != 0
            || u64::from(self.preempt_state_offset) + 32 > PREEMPT_BYTES as u64
            || self.operand_region_bytes == 0 || self.operand_region_bytes & 0xfff != 0
            || u64::from(self.operand_region_bytes) * 2 + 0x808 > self.operand_bytes as u64
            || self.operand_aux_offset & 7 != 0
            || u64::from(self.operand_aux_offset) + 8 > self.operand_bytes as u64 {
            return Err(EINVAL);
        }
        Ok(())
    }
}

/// Construct the complete primary program before publishing its descriptor.
pub(crate) fn main(args: &RegisterArgs) -> Result<[RegisterWrite; MAIN_COUNT]> {
    args.validate()?;
    let preempt = args.preempt_va;
    let operand = args.operand_va;
    let ids = args.object_ids;
    let intersection = args.intersection.words();
    let state = u64::from(args.preempt_state_offset);
    let region = u64::from(args.operand_region_bytes);
    let key = work_key(args.context_id, args.work_state);
    Ok([
        RegisterWrite::new(0x1a510, preempt),
        RegisterWrite::new(0x1a420, args.cdm_va),
        RegisterWrite::new(0x1a4d0, offset_va(preempt, state)?),
        RegisterWrite::new(0x1a4d8, offset_va(preempt, state + 8)?),
        RegisterWrite::new(0x1a4e0, offset_va(preempt, state + 16)?),
        RegisterWrite::new(0x1a4e8, offset_va(preempt, state + 24)?),
        RegisterWrite::new(0x1a440, 0x154024201),
        RegisterWrite::new(0x0d4a1, args.resource_control),
        RegisterWrite::new(0x0d411, 0x200000001),
        RegisterWrite::new(0x0a599, args.resource_policy),
        RegisterWrite::new(0x1a458, args.dispatch_control),
        RegisterWrite::new(0x101d9, 0x1c),
        RegisterWrite::new(0x1a089, intersection[0]),
        RegisterWrite::new(0x1a091, intersection[1]),
        RegisterWrite::new(0x1a059, intersection[2]),
        RegisterWrite::new(0x1a061, intersection[3]),
        RegisterWrite::new(0x1a0b9, intersection[4]),
        RegisterWrite::new(0x1a0c1, intersection[5]),
        RegisterWrite::new(0x101d1, args.scratch.size_word()),
        RegisterWrite::new(0x0d479, u64::from(args.auxiliary_allocation)),
        RegisterWrite::new(0x1a0e9, args.scratch.stack_word()),
        RegisterWrite::new(0x1a141, args.scratch.driver_word()),
        RegisterWrite::new(0x107a1, args.scratch.window_word() | (u64::from(args.matrix_ops) << 25)),
        RegisterWrite::new(0x1a540, ids),
        RegisterWrite::new(0x014a9, ids),
        RegisterWrite::new(0x0a351, ids),
        RegisterWrite::new(0x0a731, args.auxiliary_control),
        RegisterWrite::new(0x10201, key),
        RegisterWrite::new(0x10428, key),
        RegisterWrite::new(0x14028, args.free_list_slot.into()),
        RegisterWrite::new(0x14070, args.usage_va | 1),
        RegisterWrite::new(0x10229, offset_va(operand, region * 2 + 0x800)?),
        RegisterWrite::new(0x140a8, offset_va(operand, args.operand_aux_offset.into())?),
        RegisterWrite::new(0x10099, offset_va(operand, region + 0x405)?),
        RegisterWrite::new(0x10091, offset_va(operand, region * 2 + 0x400)?),
        RegisterWrite::new(0x0a5c1, offset_va(operand, 5)?),
        RegisterWrite::new(0x0a5c9, offset_va(operand, region)?),
        RegisterWrite::new(0x0a731, 0),
        RegisterWrite::new(0x1a440, 0x154024209),
        RegisterWrite::new(0x0a599, 0x6000400020),
])
}

fn program(args: &RegisterArgs) -> Result<([RegisterWrite; MAIN_COUNT + 4], usize)> {
    let main = main(args)?;
    let mut writes = [RegisterWrite::new(0, 0); MAIN_COUNT + 4];
    let mut count = 0;
    if args.scratch.enabled() {
        writes[count] = RegisterWrite::new(0x17e1, 1);
        count += 1;
    }
    if args.matrix_ops {
        writes[count] = RegisterWrite::new(0x17f1, 1);
        count += 1;
    }
    for (index, register) in main.iter().enumerate() {
        if index == 33 {
            if args.scratch.enabled() {
                writes[count] = RegisterWrite::new(0x17d9, 1);
                count += 1;
            }
            if args.matrix_ops {
                writes[count] = RegisterWrite::new(0x17e9, 1);
                count += 1;
            }
        }
        writes[count] = *register;
        count += 1;
    }
    Ok((writes, count))
}

/// Operand-state exchange used by the alternate program.
pub(crate) fn alternate(args: &RegisterArgs) -> Result<[RegisterWrite; ALTERNATE_COUNT]> {
    args.validate()?;
    let operand = args.operand_va;
    let region = u64::from(args.operand_region_bytes);
    Ok([
        RegisterWrite::new(0x10099, offset_va(operand, 5)?),
        RegisterWrite::new(0x10091, offset_va(operand, region)?),
        RegisterWrite::new(0x0a5c1, offset_va(operand, region + 0x405)?),
        RegisterWrite::new(0x0a5c9, offset_va(operand, region * 2 + 0x400)?),
    ])
}

/// Register-array bindings in a compute kick. The first checkpoint follows
/// the initial operand-region write, before the second auxiliary-control write.
pub(crate) fn bindings(descriptor_va: u64, scratch: ScratchRequest, matrix_ops: bool)
    -> Result<[Option<super::super::kick::RegisterArrayBinding>; 4]>
{
    use super::super::kick::RegisterArrayBinding;
    Ok([
        Some(RegisterArrayBinding::new(offset_va(descriptor_va, 0x40)?, (scratch.checkpoints()[0] + if matrix_ops { 2 } else { 0 }) as u8)?),
        Some(RegisterArrayBinding::new(offset_va(descriptor_va, 0x760)?, ALTERNATE_COUNT as u8)?),
        None,
        None,
    ])
}

/// The fixed prefix of a compute descriptor. Storage beyond this prefix is
/// owned and initialized by the runtime allocation, not described here.
pub(crate) const DESCRIPTOR_PREFIX_BYTES: usize = 0x1000;

/// Per-command fields outside the two register programs. Fields whose bit
/// meanings are not established remain explicit opaque values.
pub(crate) struct DescriptorArgs {
    pub(crate) registers: RegisterArgs,
    pub(crate) descriptor_va: u64,
    pub(crate) submit_sequence: u64,
    pub(crate) work_state_va: u64,
    pub(crate) cdm_end_va: u64,
    pub(crate) stamp_va: [u64; 2],
    pub(crate) stamp_value: u32,
    pub(crate) qid: u32,
    pub(crate) kick_timestamp: u64,
    pub(crate) completion_ordinal: u32,
    pub(crate) status_va: [u64; 2],
    pub(crate) timestamp_va: [u64; 2],
    pub(crate) shared_control_va: u64,
    pub(crate) event_metadata: [u8; 16],
    pub(crate) usage_fw_va: u64,
    pub(crate) completion_identity: u64,
}

/// The M5 descriptor has a shorter reserved region before shared-control
/// state. Register-array offsets are shared with G17, tail offsets are not.
#[repr(C, packed)]
pub(crate) struct Descriptor {
    tag: u32,
    submit_sequence: u64,
    context_id: u32,
    work_state_va: u64,
    checkpoints: [u16; 4],
    reserved_20: [u8; 0x20],
    registers: super::super::kick::RegisterArray,
    alternate: super::super::kick::RegisterArray,
    reserved_e80: [u8; 0x58],
    preempt_va: u64,
    cdm_end_va: u64,
    reserved_ee8: [u8; 0x20],
    control: u64,
    reserved_f10: [u8; 0x10],
    predecessor_id: u32,
    reserved_f24: u32,
    unk_f28: u32,
    firmware_timestamps: [u64; 2],
    reserved_f3c: u32,
    stamp_va: [u64; 2],
    stamp_value: u32,
    qid: u32,
    reserved_f58: u64,
    kick_timestamp: u64,
    current_id: u64,
    completion_ordinal: u32,
    reserved_f74: [u8; 8],
    status_va: [u64; 2],
    timestamp_va: [u64; 2],
    reserved_f9c: [u8; 0x10],
    shared_control_va: u64,
    shared_control_enabled: u8,
    event_metadata: [u8; 16],
    usage_fw_va: u64,
    completion_identity: u64,
    reserved_fd5: [u8; 0x2b],
}

static_assert!(size_of::<Descriptor>() == DESCRIPTOR_PREFIX_BYTES);
static_assert!(core::mem::offset_of!(Descriptor, registers) == 0x40);
static_assert!(core::mem::offset_of!(Descriptor, alternate) == 0x760);
static_assert!(core::mem::offset_of!(Descriptor, stamp_va) == 0xf40);
static_assert!(core::mem::offset_of!(Descriptor, completion_ordinal) == 0xf70);
static_assert!(core::mem::offset_of!(Descriptor, shared_control_va) == 0xfac);
static_assert!(core::mem::offset_of!(Descriptor, usage_fw_va) == 0xfc5);
static_assert!(core::mem::offset_of!(Descriptor, completion_identity) == 0xfcd);

// SAFETY: Every field consists only of integer values and their arrays.
unsafe impl Zeroable for Descriptor {}

impl Descriptor {
    /// Fill the entire prefix; no address or register becomes visible before
    /// validation completes. The owner publishes only after this returns.
    pub(crate) fn write(&mut self, args: &DescriptorArgs) -> Result {
        self.populate::<true>(args)
    }

    /// Populate a prefix already cleared by `KernelObject::initialize`.
    /// The caller must provide all-zero storage so reserved bytes stay zero.
    pub(crate) fn write_zeroed(&mut self, args: &DescriptorArgs) -> Result {
        self.populate::<false>(args)
    }

    fn populate<const CLEAR: bool>(&mut self, args: &DescriptorArgs) -> Result {
        if args.cdm_end_va < args.registers.cdm_va || args.qid >= 128 {
            return Err(EINVAL);
        }
        let (main, count) = program(&args.registers)?;
        let alternate = alternate(&args.registers)?;
        let main_va = offset_va(args.descriptor_va, 0x40)?;
        let alternate_va = offset_va(args.descriptor_va, 0x760)?;
        // Also validate the kick's encodable addresses before mutating storage.
        bindings(args.descriptor_va, args.registers.scratch, args.registers.matrix_ops)?;
        if CLEAR {
            super::super::clear(self);
        }
        self.tag = 3;
        self.submit_sequence = args.submit_sequence;
        self.context_id = args.registers.context_id.into();
        self.work_state_va = args.work_state_va;
        self.checkpoints = args.registers.scratch.checkpoints().map(|n| n + if args.registers.matrix_ops { 2 } else { 0 });
        super::set_register_array(&mut self.registers, main_va, &main[..count])?;
        super::set_register_array(&mut self.alternate, alternate_va, &alternate)?;
        self.preempt_va = args.registers.preempt_va;
        self.cdm_end_va = args.cdm_end_va;
        self.control = 0x154024201;
        self.predecessor_id = (args.registers.object_ids >> 32) as u32;
        self.unk_f28 = u32::MAX;
        self.stamp_va = args.stamp_va;
        self.stamp_value = args.stamp_value;
        self.qid = args.qid;
        self.kick_timestamp = args.kick_timestamp;
        self.current_id = args.registers.object_ids as u32 as u64;
        self.completion_ordinal = args.completion_ordinal;
        self.status_va = args.status_va;
        self.timestamp_va = args.timestamp_va;
        self.shared_control_va = args.shared_control_va;
        self.shared_control_enabled = u8::from(args.shared_control_va != 0);
        self.event_metadata = args.event_metadata;
        self.usage_fw_va = args.usage_fw_va;
        self.completion_identity = args.completion_identity;
        Ok(())
    }
}
