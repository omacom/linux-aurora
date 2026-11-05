// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Storage of one render pair and its per-command generations. Client aliases
//! precede their backing owners so teardown cannot leave a dangling GPU view.

use super::retirement::Cursors;
use crate::g17::{
    buffer::{self, target},
    context::Context,
    fw::{
        self, queue,
        render::{self as abi, storage},
    },
    object::{Allocator, CpuMap, Family, KernelObject, Pool, PooledObject},
};
use crate::{
    hw::t8140::{dynamic, kernel_window, low, queues, resources as cfg},
    mmu,
};
use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};
use crate::g17::fw::m5::initdata::Board;

pub(super) const RECORDS: usize = 32;
pub(super) const TA_BYTES: usize = 0x14000;
pub(super) const FRAGMENT_BYTES: usize = 0x48000;
pub(super) const GRAPH_SIZE: usize = 0x1c000;
pub(super) const POINTERS: [usize; 2] = [0x9000, 0x9080];
pub(super) const ITEMS: [usize; 2] = [0xc000, 0x10000];
pub(super) const ANNOUNCE_STRIDE: usize = 0x80;
pub(super) const ANNOUNCE: [usize; 2] = [0x17000, 0x1b000];
pub(super) const JOB_LIST: usize = 0xa000;
pub(super) const CONFIG: [usize; 2] = [0, 0x4000];
pub(super) const CONFIG_STRIDE: usize = 0x180;
/// Usage records of one M5 command, both stages.
const M5_USAGE_BYTES: usize = 0x40;
const M5_USAGE_PAIR_BYTES: usize = 2 * M5_USAGE_BYTES;
/// The lead command of a stage (InitBM before tiling), after the stage's configuration record
/// in its slot ([`crate::g17::fw::Variant::lead_commands`]).
pub(super) const LEAD: usize = 0xc0;
const DEPTH: usize = queues::RENDER_DEPTH as usize;
/// Offsets of the priority pair and the policy word in a queue record.
const PRIORITY: usize = core::mem::offset_of!(queue::QueueRecord, priority);
const POLICY: usize = core::mem::offset_of!(queue::QueueRecord, policy);

struct Aliases {
    client: mmu::KernelMapping,
    _global: mmu::KernelMapping,
}
impl Aliases {
    fn new(
        object: &KernelObject,
        alloc: &Allocator<'_>,
        vm: &mmu::Vm,
        source: core::ops::Range<usize>,
    ) -> Result<Self> {
        let mut range = alloc.uat.kernel_lower_dynamic_range();
        let size = (source.end - source.start) as u64;
        loop {
            // Descriptor aliases are read-only to GPU consumers; the canonical
            // firmware mapping remains writable for command state updates.
            let global = object.map_range(
                alloc.uat.kernel_lower_vm(),
                source.clone(),
                range.clone(),
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RO,
                0,
            )?;
            let address = global.iova();
            let end = address.checked_add(size).ok_or(EOVERFLOW)?;
            match object.map_range(
                vm,
                source.clone(),
                address..end,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RO,
                0,
            ) {
                Ok(client) => {
                    return Ok(Self {
                        client,
                        _global: global,
                    })
                }
                Err(error) if error == ENOSPC && end < range.end => {
                    drop(global);
                    range.start = end;
                }
                Err(error) => return Err(error),
            }
        }
    }
}

/// Stable queue records and rotating descriptors/configuration records.
pub(super) struct Graph {
    /// One alias of all M5 command storage, or the G17P tiling and fragment ones.
    aliases: [Option<Aliases>; 2],
    pub(super) descriptors: PooledObject,
    pub(super) queues: PooledObject,
    pub(super) config: PooledObject,
    pub(super) support: KernelObject,
    pub(super) owners: [Option<u64>; RECORDS],
    /// Host words of the pointer blocks in this firmware build.
    pub(super) pointers: fw::variant::PointerLayout,
    m5: bool,
}
impl Graph {
    pub(super) fn is_m5(&self) -> bool { self.m5 }

    /// Out of line: inlined, its temporaries stay reserved in `Memory::new`'s frame while the
    /// rest of the pair (`buffer::Manager::new` and its allocations) is built below it, which
    /// overflowed the submit ioctl's kernel stack.
    #[inline(never)]
    pub(super) fn new(
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        context: &Context,
        qids: [u8; 2],
        pointers: fw::variant::PointerLayout,
    ) -> Result<Self> {
        let prot = mmu::PROT_GPU_FW_SHARED_RW;
        let m5 = alloc.uat.is_m5();
        // M5 selects cached firmware attributes at initial mapping creation;
        // write-combined CPU stores also support the uncached capacity fallback.
        let cpu = if m5 { CpuMap::WriteCombined } else { CpuMap::WriteBack };
        let bytes = if m5 { RECORDS * fw::m5::render_assembly::COMMAND_STORAGE_BYTES }
            else { TA_BYTES + FRAGMENT_BYTES };
        let descriptors = pool.allocate(
            alloc,
            Family::Descriptor,
            bytes,
            bytes,
            prot,
            cpu,
        )?;
        let ta = Aliases::new(&descriptors, alloc, context.vm(),
            0..if m5 { bytes } else { TA_BYTES })?;
        let fragment = if m5 { None } else { Some(Aliases::new(
            &descriptors,
            alloc,
            context.vm(),
            TA_BYTES..TA_BYTES + FRAGMENT_BYTES,
        )?) };
        let mut queues = pool.allocate(
            alloc,
            Family::QueueState,
            GRAPH_SIZE,
            0x8000,
            prot,
            CpuMap::WriteCombined,
        )?;
        let config = pool.allocate(
            alloc,
            Family::QueueConfig,
            0x8000,
            0x8000,
            prot,
            cpu,
        )?;
        let support = alloc.kernel(
            if m5 { mmu::UAT_PGSZ } else { storage::SUPPORT_SIZE },
            mmu::UAT_PGSZ as u64,
            prot,
            CpuMap::WriteCombined,
        )?;
        let address = queues.gpu_va();
        for stage in 0..2 {
            let args = queue::QueueRecordArgs {
                pointers_va: address + POINTERS[stage] as u64,
                items_va: address + ITEMS[stage] as u64,
                job_list_va: context.work_head_va(),
                scheduler_va: context.scheduler_va(),
                owner_pid: context.owner_pid(),
                policy: context.policy(),
            };
            if m5 {
                queues.write(qids[stage] as usize * size_of::<fw::m5::queue::QueueRecord>(),
                    fw::m5::queue::QueueRecord::new(&args)?)?;
            } else {
                queues.write(
                    qids[stage] as usize * size_of::<queue::QueueRecord>(),
                    queue::QueueRecord::new(&args),
                )?;
            }
            queues.initialize::<queue::PointerBlock>(POINTERS[stage], |view| {
                *view = queue::PointerBlock::new(&pointers)?;
                Ok(())
            })?;
        }
        queues.write(JOB_LIST, queue::JobListHead::new(address + JOB_LIST as u64))?;
        Ok(Self {
            aliases: [Some(ta), fragment],
            descriptors,
            queues,
            config,
            support,
            owners: [None; RECORDS],
            pointers,
            m5,
        })
    }
    pub(super) fn queue_offset(&self, qid: u8) -> usize {
        usize::from(qid) * if self.m5 { size_of::<fw::m5::queue::QueueRecord>() }
            else { size_of::<queue::QueueRecord>() }
    }
    pub(super) fn queue_va(&self, qid: u8) -> u64 {
        self.queues.gpu_va() + self.queue_offset(qid) as u64
    }
    pub(super) fn offset(&self, stage: usize, ordinal: u64) -> usize {
        let slot = ordinal as usize % RECORDS;
        if self.m5 {
            slot * fw::m5::render_assembly::COMMAND_STORAGE_BYTES
                + stage * fw::m5::render_assembly::FRAGMENT_OFFSET
        } else if stage == 0 {
            slot * abi::TA_DESCRIPTOR_SIZE
        } else {
            TA_BYTES + slot * abi::FRAGMENT_SLOT_SIZE
        }
    }
    pub(super) fn descriptor_va(&self, stage: usize, ordinal: u64) -> u64 {
        self.descriptors.gpu_va() + self.offset(stage, ordinal) as u64
    }
    pub(super) fn descriptor_client(&self, stage: usize, ordinal: u64) -> Result<u64> {
        if self.m5 {
            return Ok(self.aliases[0].as_ref().ok_or(EIO)?.client.iova()
                + self.offset(stage, ordinal) as u64);
        }
        let stride = if stage == 0 {
            abi::TA_DESCRIPTOR_SIZE
        } else {
            abi::FRAGMENT_SLOT_SIZE
        };
        Ok(self.aliases[stage].as_ref().ok_or(EIO)?.client.iova()
            + (ordinal as usize % RECORDS * stride) as u64)
    }
    /// The command storage of `ordinal` on M5: both stages' commands.
    pub(super) fn m5_storage(&mut self, ordinal: u64) -> Result<&mut [u8]> {
        if !self.m5 || !self.owns(ordinal) { return Err(EINVAL); }
        let start = self.offset(0, ordinal);
        self.descriptors.bytes_mut()
            .get_mut(start..start + fw::m5::render_assembly::COMMAND_STORAGE_BYTES)
            .ok_or(EINVAL)
    }
    /// Reset the usage entries owned by this descriptor generation. They stay
    /// resident with the graph until both stages release the descriptor slot.
    pub(super) fn m5_usage(&mut self, ordinal: u64) -> Result<[u64; 2]> {
        if !self.m5 || !self.owns(ordinal) { return Err(EINVAL); }
        let start = ordinal as usize % RECORDS * M5_USAGE_PAIR_BYTES;
        self.support.bytes_mut().get_mut(start..start + M5_USAGE_PAIR_BYTES)
            .ok_or(EINVAL)?.fill(0);
        let address = self.support.gpu_va() + start as u64;
        Ok([address, address + M5_USAGE_BYTES as u64])
    }
    pub(super) fn publish_recovery_targets(&self) -> Result {
        let Some(target) = self.pointers.recovery_target else {
            return Ok(());
        };
        for at in POINTERS {
            let producer = self.queues.word(at + self.pointers.producer)?
                .load(Ordering::Relaxed);
            self.queues.word(at + target)?.store(producer, Ordering::Relaxed);
        }
        Ok(())
    }

    pub(super) fn cursors(&self) -> Result<[Cursors; 2]> {
        let mut values = [Cursors {
            done: 0,
            read: 0,
            write: 0,
        }; 2];
        for stage in 0..2 {
            let at = POINTERS[stage];
            values[stage] = Cursors {
                done: self.queues.word(at)?.load(Ordering::Relaxed),
                read: self.queues.word(at + 0x30)?.load(Ordering::Relaxed),
                write: self.queues.word(at + self.pointers.producer)?.load(Ordering::Relaxed),
            };
        }
        Ok(values)
    }
    pub(super) fn claim(&mut self, ordinal: u64) -> Result {
        let owner = &mut self.owners[ordinal as usize % RECORDS];
        if owner.is_some_and(|old| old != ordinal) {
            return Err(EBUSY);
        }
        *owner = Some(ordinal);
        Ok(())
    }
    pub(super) fn owns(&self, ordinal: u64) -> bool {
        self.owners[ordinal as usize % RECORDS] == Some(ordinal)
    }
    pub(super) fn release(&mut self, ordinal: u64) -> Result {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        self.owners[ordinal as usize % RECORDS] = None;
        Ok(())
    }
}

struct Compact {
    mapping: Option<mmu::KernelMapping>,
    address: u64,
    _backing: KernelObject,
}
/// The mesh storage chunk: chunk bases are 64 KiB aligned and the storage
/// register counts 64 KiB chunks.
const MESH_STORAGE_CHUNK: usize = 0x10000;

impl Compact {
    /// M5 shared-RW object in the compact window with an explicit VA alignment.
    fn new_aligned(alloc: &Allocator<'_>, vm: &mmu::Vm, size: usize, align: u64) -> Result<Self> {
        let size = size.checked_add(mmu::UAT_PGSZ - 1).ok_or(EOVERFLOW)? & !(mmu::UAT_PGSZ - 1);
        let backing = KernelObject::backing(alloc.dev, size, CpuMap::WriteCombined)?;
        let mapping = backing.map_range(vm, 0..size, buffer::BlockGeometry::M5_COMPACT,
            align, mmu::PROT_GPU_SHARED_RW, mmu::UAT_PGSZ)?;
        if mapping.iova() % align != 0 { return Err(EINVAL); }
        Ok(Self { address: mapping.iova(), mapping: Some(mapping), _backing: backing })
    }
    fn new(alloc: &Allocator<'_>, vm: &mmu::Vm, size: usize) -> Result<Self> {
        if alloc.uat.is_m5() {
            return Self::new_aligned(alloc, vm, size, cfg::TVB_PAGE);
        }
        let backing = KernelObject::backing(alloc.dev, size, CpuMap::WriteCombined)?;
        let mapping = buffer::compact_alias(
            &backing,
            vm,
            0..size,
            cfg::TVB_PAGE,
            mmu::PROT_GPU_SHARED_RW,
            mmu::UAT_PGSZ,
        )?;
        Ok(Self {
            address: mapping.iova(),
            mapping: Some(mapping),
            _backing: backing,
        })
    }
}

/// Per-command state of a pair: the G17P one below or the M5 one
/// ([`super::generation::Generation`]).
pub(super) enum Generation {
    Neo(KBox<NeoGeneration>),
    M5(KBox<super::generation::Generation>),
}
impl Generation {
    fn allocate_state(alloc: &Allocator<'_>, vm: &mmu::Vm, board: Option<&'static Board>)
        -> Result<KernelObject> {
        if alloc.uat.is_m5() {
            super::generation::Generation::allocate_state(alloc, vm, board)
        } else {
            NeoGeneration::allocate_state(alloc, vm)
        }
    }
    fn new(alloc: &Allocator<'_>, vm: &mmu::Vm, state: KernelObject,
        board: Option<&'static Board>) -> Result<Self> {
        Ok(if alloc.uat.is_m5() {
            Self::M5(super::generation::Generation::new(alloc, vm, state, board)?)
        } else {
            Self::Neo(NeoGeneration::new(alloc, vm, state)?)
        })
    }
    pub(super) fn ordinal(&self) -> Option<u64> {
        match self {
            Self::Neo(generation) => generation.ordinal,
            Self::M5(generation) => generation.ordinal,
        }
    }
    /// Gives up the claim of an unpublished or retired command.
    pub(super) fn clear(&mut self) {
        match self {
            Self::Neo(generation) => generation.ordinal = None,
            Self::M5(generation) => generation.ordinal = None,
        }
    }
    /// `viewport` applies to the G17P deflake buffer ([`abi::Deflake::viewport`]).
    pub(super) fn prepare(&mut self, ordinal: u64, pass: &abi::RenderPass, viewport: bool) -> Result {
        match self {
            Self::Neo(generation) => generation.prepare(ordinal, pass, viewport),
            Self::M5(generation) => generation.prepare(ordinal, pass),
        }
    }
    pub(super) fn addresses(&self) -> [u64; 5] {
        match self {
            Self::Neo(generation) => generation.addresses(),
            Self::M5(generation) => generation.addresses(),
        }
    }
}

pub(super) struct NeoGeneration {
    deflake: mmu::KernelMapping,
    fragment_client: mmu::KernelMapping,
    state: KernelObject,
    fragment: KernelObject,
    pub(super) ordinal: Option<u64>,
}
impl NeoGeneration {
    fn allocate_state(alloc: &Allocator<'_>, vm: &mmu::Vm) -> Result<KernelObject> {
        KernelObject::new(
            alloc.dev,
            vm,
            crate::g17::object::Placement::In(dynamic::CLIENT_LOWER, cfg::TVB_PAGE),
            storage::STATE_SIZE,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )
    }
    fn new(alloc: &Allocator<'_>, vm: &mmu::Vm, state: KernelObject) -> Result<KBox<Self>> {
        let deflake = buffer::compact_alias(
            &state,
            vm,
            storage::DEFLAKE..storage::DEFLAKE + mmu::UAT_PGSZ,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            mmu::UAT_PGSZ,
        )?;
        let fragment = alloc.kernel(
            mmu::UAT_PGSZ,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let fragment_client = buffer::compact_alias(
            &fragment,
            vm,
            0..mmu::UAT_PGSZ,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RW,
            mmu::UAT_PGSZ,
        )?;
        KBox::new(
            Self {
                deflake,
                fragment_client,
                state,
                fragment,
                ordinal: None,
            },
            GFP_KERNEL,
        )
        .map_err(Into::into)
    }
    /// With `viewport` false the deflake buffer has no room for the viewport words
    /// ([`abi::Deflake::viewport`]).
    pub(super) fn prepare(
        &mut self,
        ordinal: u64,
        pass: &abi::RenderPass,
        viewport: bool,
    ) -> Result {
        if self.ordinal.is_some_and(|old| old != ordinal) {
            return Err(EBUSY);
        }
        let viewport = match viewport {
            true => Some(storage::Viewport::new(pass.width, pass.height)?),
            false => None,
        };
        self.state.bytes_mut()[storage::AUXILIARY..storage::AUXILIARY + storage::AUXILIARY_SIZE]
            .fill(0);
        if let Some(viewport) = viewport {
            let destination = self
                .state
                .pointer(storage::DEFLAKE + 0x900, size_of::<storage::Viewport>())?
                .cast::<storage::Viewport>();
            if destination.align_offset(align_of::<storage::Viewport>()) != 0 {
                return Err(EINVAL);
            }
            // SAFETY: The complete typed integer view is in the retained CPU mapping
            // and naturally aligned. This command generation is exclusively owned
            // and unpublished; no prior command can access it until reassigned.
            unsafe { viewport.apply(&mut *destination) };
        }
        self.fragment.bytes_mut().fill(0);
        self.ordinal = Some(ordinal);
        Ok(())
    }
    pub(super) fn addresses(&self) -> [u64; 5] {
        [
            self.deflake.iova(),
            self.state.gpu_va() + storage::TA_STATUS as u64,
            self.fragment_client.iova(),
            self.fragment.gpu_va(),
            // The retained state already maps these pages into the client VM.
            // The auxiliary framebuffer accepts a full GPU address; reusing this
            // view avoids exhausting the fixed 4 MiB alias arena across pairs.
            self.state.gpu_va() + storage::AUXILIARY as u64,
        ]
    }
}

pub(super) struct Memory {
    _fragment_view: Option<mmu::KernelMapping>,
    pub(super) graph: Graph,
    scratch: Compact,
    discard: Compact,
    /// M5 mesh rendering buffer of the queue's task-stage kicks. Shaders read
    /// its per-workgroup 64 KiB chunk base, so it is mapped in the context VM.
    mesh_storage: Option<Compact>,
    mesh_storage_chunks: u32,
    pub(super) target: KBox<target::Target>,
    generations: [Option<Generation>; DEPTH],
    generation_count: usize,
    pub(super) timestamps: KernelObject,
    pub(super) manager: buffer::Manager,
    pub(super) lifecycle: [fw::ObjectIds; 2],
    // The extra client views retain their allocation positions even though
    // kick entries consume the register arrays embedded in the descriptors.
    registers: Option<KernelObject>,
    m5_workspace: Option<KBox<fw::m5::render_registers::Workspace>>,
    pub(super) context: Arc<Context>,
    pub(super) installed_pool: Option<u8>,
    /// The M5 board ([`crate::g17::Shared::m5_board`]); `None` off M5.
    pub(super) board: Option<&'static Board>,
}
impl Memory {
    #[inline(never)]
    #[expect(
        clippy::too_many_arguments,
        reason = "Render memory construction keeps each allocation and firmware input explicit."
    )]
    pub(super) fn new(
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        context: Arc<Context>,
        qids: [u8; 2],
        layout: &target::Layout,
        metrics: buffer::MetricsLease,
        buffer_id: (u8, u64),
        pm_generation: u64,
        ordinal: u64,
        word: u32,
        variant: &'static fw::Variant,
        board: Option<&'static Board>,
    ) -> Result<(KBox<Self>, KVec<mmu::KernelMapping>)> {
        let storage = KBox::<Self>::new_uninit(GFP_KERNEL)?;
        let vm = context.vm();
        let m5 = alloc.uat.is_m5();
        if m5 != layout.is_m5() { return Err(EINVAL); }
        let mut graph = Graph::new(alloc, pool, &context, qids, variant.pointers)?;
        let mut target = target::Target::new(alloc.dev, vm, layout)?;
        target.prepare(ordinal, layout)?;
        let state = Generation::allocate_state(alloc, vm, board)?;
        let (tvb, mappings, scratch, discard) = if m5 {
            let geometry = super::m5_render_submission::parameter_geometry(layout.tvb_blocks)?;
            let bytes = fw::m5::pool::parameter_scratch_stride(geometry.capacity, 0)
                .map_err(|_| EINVAL)?.checked_mul(fw::m5::pool::SCENE_TABLE_ENTRIES)
                .ok_or(EOVERFLOW)?;
            let (tvb, mappings) = buffer::Tvb::with_geometry(alloc.dev, vm, layout.tvb_blocks, geometry)?;
            let scratch = Compact::new(alloc, vm, bytes)?;
            let discard = Compact::new(alloc, vm, 0x8000)?;
            (tvb, mappings, scratch, discard)
        } else {
            let scratch = Compact::new(alloc, vm, cfg::SCENE_SIZE as usize)?;
            let discard = Compact::new(alloc, vm, cfg::DISCARD_SIZE as usize)?;
            let (tvb, mappings) = buffer::Tvb::new(alloc.dev, vm, layout.tvb_blocks)?;
            (tvb, mappings, scratch, discard)
        };
        let count = DEPTH;
        let initial = if ordinal == 0 {
            0
        } else {
            (ordinal ^ 1) as usize % count
        };
        let mut generations = core::array::from_fn(|_| None);
        generations[initial] = Some(Generation::new(alloc, vm, state, board)?);
        for offset in 1..count {
            let index = (initial + offset) % count;
            let state = Generation::allocate_state(alloc, vm, board)?;
            generations[index] = Some(Generation::new(alloc, vm, state, board)?);
        }
        let timestamps = alloc.kernel(
            mmu::UAT_PGSZ,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let manager = buffer::Manager::new(
            alloc,
            vm,
            tvb,
            metrics,
            buffer::ManagerArgs {
                scratch: scratch.address,
                discard: discard.address,
                buffer_id: buffer_id.0,
                generation: pm_generation,
                first_ordinal: ordinal,
            },
        )?;
        let predecessor = crate::g17::ids::OBJECTS.object()?;
        let (fragment, tiling) = crate::g17::ids::OBJECTS.render_pair(predecessor)?;
        let support_va = graph.support.gpu_va();
        if !m5 {
            graph.support.initialize::<storage::Support>(0, |view| {
                view.init(
                    support_va,
                    alloc.uat.geometry().kernel_range().start + kernel_window::INDEX,
                    low::RENDER_RUN_LIST,
                    low::RENDER_PAGE_LIST,
                    word,
                )
            })?;
        }
        let registers = if m5 { None } else { Some(KernelObject::new(
            alloc.dev,
            vm,
            crate::g17::object::Placement::In(dynamic::CLIENT_LOWER, mmu::UAT_PGSZ as u64),
            mmu::UAT_PGSZ,
            mmu::PROT_GPU_SHARED_RO,
            CpuMap::WriteCombined,
        )?) };
        let m5_workspace = if m5 {
            let mut workspace = KBox::<fw::m5::render_registers::Workspace>::new_uninit(GFP_KERNEL)?;
            // SAFETY: Workspace is integer-only and has a valid zero representation.
            unsafe { core::ptr::write_bytes(workspace.as_mut_ptr(), 0, 1); }
            // SAFETY: The entire value was initialized above.
            Some(unsafe { workspace.assume_init() })
        } else { None };
        let fragment_view = if m5 { None } else { Some(graph.descriptors.map_range(
            vm,
            TA_BYTES..TA_BYTES + mmu::UAT_PGSZ,
            dynamic::CLIENT_LOWER,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RO,
            0,
        )?) };
        Ok((
            storage.write_init(kernel::try_init!(Self {
                graph,
                scratch,
                discard: discard,
                mesh_storage: None,
                mesh_storage_chunks: 0,
                target,
                generations,
                generation_count: count,
                timestamps,
                manager,
                lifecycle: [tiling, fragment],
                _fragment_view: fragment_view,
                registers: registers,
                m5_workspace,
                context,
                installed_pool: None,
                board,
            }))?,
            mappings,
        ))
    }
    /// The mesh storage register value of a task-stage kick. The buffer is
    /// allocated on the queue's first such kick and kept for the queue's lifetime,
    /// so other contexts pay nothing. A failed allocation leaves no partial state.
    /// A zero chunk parameter disables the binding.
    pub(super) fn mesh_storage_register(&mut self, alloc: &Allocator<'_>) -> Result<u64> {
        if self.mesh_storage.is_none() {
            let chunks = crate::module_parameters::m5_mesh_storage_chunks.value();
            if chunks == 0 { return Ok(0); }
            if chunks > 0xffff { return Err(EINVAL); }
            let bytes = (chunks as usize).checked_mul(MESH_STORAGE_CHUNK).ok_or(EOVERFLOW)?;
            let storage = Compact::new_aligned(alloc, self.context.vm(), bytes,
                MESH_STORAGE_CHUNK as u64)?;
            self.mesh_storage = Some(storage);
            self.mesh_storage_chunks = chunks;
        }
        let storage = self.mesh_storage.as_ref().ok_or(ENOMEM)?;
        Ok((storage.address & 0xffff_ffff_ffff) | (u64::from(self.mesh_storage_chunks) << 48))
    }
    pub(super) fn generation(&mut self, ordinal: u64) -> Result<&mut Generation> {
        self.generations[ordinal as usize % self.generation_count]
            .as_mut()
            .ok_or(EIO)
    }
    pub(super) fn owns(&self, ordinal: u64) -> bool {
        self.graph.owns(ordinal)
            && self.generations[ordinal as usize % self.generation_count]
                .as_ref()
                .is_some_and(|generation| generation.ordinal() == Some(ordinal))
    }
    /// M5 commands record four stage timestamps, G17P ones two.
    pub(super) fn timestamp_offset(&self, ordinal: u64) -> usize {
        ordinal as usize % RECORDS * if self.graph.m5 { 32 } else { 16 }
    }
    pub(super) fn clear_timestamps(&mut self, ordinal: u64) -> Result {
        let at = self.timestamp_offset(ordinal);
        if self.graph.m5 { self.timestamps.write(at, [0u64; 4]) }
        else { self.timestamps.write(at, [0u64; 2]) }
    }
    pub(super) fn pass(&self, ordinal: u64) -> Result<[u64; 2]> {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        let offset = self.timestamp_offset(ordinal);
        Ok([
            self.timestamps.dword(offset)?.load(Ordering::Relaxed),
            self.timestamps.dword(offset + if self.graph.m5 { 24 } else { 8 })?
                .load(Ordering::Relaxed),
        ])
    }
    /// Export the four M5 stage words only after the observer has acquired
    /// both completion stamps. Keep the two-word G17P representation intact.
    pub(super) fn stage_timestamps(&self, ordinal: u64) -> Result<[u64; 4]> {
        if !self.graph.m5 {
            let [start, end] = self.pass(ordinal)?;
            return Ok([start, start, start, end]);
        }
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        let offset = self.timestamp_offset(ordinal);
        Ok([
            self.timestamps.dword(offset)?.load(Ordering::Relaxed),
            self.timestamps.dword(offset + 8)?.load(Ordering::Relaxed),
            self.timestamps.dword(offset + 16)?.load(Ordering::Relaxed),
            self.timestamps.dword(offset + 24)?.load(Ordering::Relaxed),
        ])
    }
    pub(super) fn owners(&self) -> [Option<u64>; DEPTH] {
        core::array::from_fn(|index| self.generations[index].as_ref().and_then(|g| g.ordinal()))
    }
    pub(super) fn retire(&mut self, ordinal: u64) -> Result {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        self.target.retire(ordinal)?;
        self.manager.retire_scene(ordinal)?;
        self.graph.release(ordinal)?;
        self.generation(ordinal)?.clear();
        Ok(())
    }
}

const _: () = {
    // Every unretired render of a pair holds its own command record.
    assert!(DEPTH <= RECORDS);
    assert!(abi::TA_DESCRIPTOR_SIZE * RECORDS <= TA_BYTES);
    assert!(abi::FRAGMENT_SLOT_SIZE * RECORDS <= FRAGMENT_BYTES);
    assert!(RECORDS * CONFIG_STRIDE <= 0x4000);
    assert!(size_of::<queue::QueueConfig>() <= LEAD);
    assert!(LEAD + size_of::<queue::InitBufferManager>() <= CONFIG_STRIDE);
};

impl Memory {
    /// Requests the tiling descriptor slot and both queue configuration records of
    /// `ordinal` for writing. The firmware accessed them for the command that used them
    /// last, so the host no longer holds their lines; requesting them before the rest of
    /// the preparation overlaps those transfers with it instead of stalling the writes one
    /// line at a time.
    pub(super) fn prefetch_descriptors(&self, ordinal: u64) {
        let record = ordinal as usize % RECORDS;
        let config = size_of::<queue::QueueConfig>();
        let ranges = [
            (&self.graph.descriptors, self.graph.offset(0, ordinal), size_of::<abi::TaDescriptor>()),
            (&self.graph.config, CONFIG[0] + record * CONFIG_STRIDE, config),
            (&self.graph.config, CONFIG[1] + record * CONFIG_STRIDE, config),
        ];
        for (object, offset, size) in ranges {
            let Ok(base) = object.pointer(offset, size) else {
                continue;
            };
            for line in (0..size).step_by(64) {
                // SAFETY: A prefetch hint inside the retained mapping; it has no
                // architectural effect on memory.
                unsafe {
                    core::arch::asm!("prfm pstl1keep, [{0}]", in(reg) base.add(line),
                        options(nostack, preserves_flags, readonly));
                }
            }
        }
    }

    /// Encodes both M5 commands of `args.ordinal` into its command storage and
    /// returns the kicks' register arrays and header flags.
    pub(super) fn write_m5_commands(
        &mut self,
        args: &abi::RenderArgs<'_>,
        layout: &target::Layout,
        plan: Option<crate::g17::render_resources::Plan>,
        mesh_class: u8,
    ) -> Result<([[Option<fw::kick::RegisterArrayBinding>; 4]; 2], fw::kick::StageKickFlags)> {
        use super::m5_render_submission as render_submission;
        use fw::m5::{render_assembly, render_registers};
        if !self.graph.m5 {
            return Err(EINVAL);
        }
        let mut controls = render_submission::controls(args.pass, self.board.ok_or(EINVAL)?)?;
        if let Some(stages) = plan.and_then(|p| p.stages) {
            controls.ta.scs = stages[0].scs;
            controls.fragment.scs = stages[1].scs;
            controls.ta.resource_flags = stages[0].resource_flags;
            controls.ta.ptc_max = stages[0].ptc_max;
            controls.fragment.resource_flags = stages[1].resource_flags;
            controls.fragment.ptc_max = stages[1].ptc_max;
        }
        let usage = self.graph.m5_usage(args.ordinal)?;
        let mut inputs = render_submission::inputs(args, usage)?;
        inputs.ta.shifted_usage_hint = fw::m5::render::mesh_class_usage_hint(mesh_class);
        render_registers::apply_pass(args, &controls, &mut inputs)?;
        let extra = self.target.m5_extra(layout)?;
        let low = self.graph.descriptor_client(0, args.ordinal)?;
        let workspace = self.m5_workspace.as_deref_mut().ok_or(EIO)?;
        let encoded = render_assembly::encode_into(self.graph.m5_storage(args.ordinal)?, low,
            args, &controls, extra, render_registers::FragmentDispatch { words: [0; 2] },
            &inputs, workspace)?;
        Ok((encoded.rce, encoded.kick_flags))
    }

    pub(super) fn write_descriptors(
        &mut self,
        args: &abi::RenderArgs<'_>,
        attachments: &[fw::kick::WriteRange],
        context: u8,
    ) -> Result<Option<fw::kick::McacheTable>> {
        let tiling = self
            .graph
            .descriptors
            .pointer(
                self.graph.offset(0, args.ordinal),
                size_of::<abi::TaDescriptor>(),
            )?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(
                self.graph.offset(1, args.ordinal),
                size_of::<abi::FragmentSlot>(),
            )?
            .cast::<abi::FragmentSlot>();
        if tiling.align_offset(align_of::<abi::TaDescriptor>()) != 0
            || fragment.align_offset(align_of::<abi::FragmentSlot>()) != 0
        {
            return Err(EINVAL);
        }
        // SAFETY: Both complete integer-only views lie in the retained mapping,
        // are aligned and disjoint. The full ordinal has an exclusive slot
        // claim and no producer refers to this command yet.
        unsafe {
            (&mut *tiling).write(args)?;
            let fragment = &mut *fragment;
            fragment.write(args)?;
            // Kick entries bind the arrays embedded in this descriptor
            // (register_bindings); the client register page is never read.
            for array in &fragment.descriptor.arrays {
                array.entries.get(..(array.size & 0xffff) as usize).ok_or(EINVAL)?;
            }
            fragment.set_mcache(
                args.fragment.descriptor_va,
                args.resources.aux_fb_va,
                attachments,
                context,
            )
        }
    }
}

impl Memory {
    pub(super) fn completion_links(
        &mut self,
        ordinal: u64,
        kicks: [fw::kick::KickTimestamp; 2],
        payload: u8,
        fragment_busy: bool,
    ) -> Result {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        if self.graph.m5 {
            return fw::m5::render_assembly::link_completions(
                self.graph.m5_storage(ordinal)?, kicks, payload, fragment_busy);
        }
        let tiling = self
            .graph
            .descriptors
            .pointer(self.graph.offset(0, ordinal), size_of::<abi::TaDescriptor>())?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(self.graph.offset(1, ordinal), size_of::<abi::FragmentSlot>())?
            .cast::<abi::FragmentSlot>();
        // SAFETY: The complete views were bounds/alignment checked during
        // preparation. The unchanged full ordinal still exclusively owns both
        // slots and neither descriptor has been published.
        unsafe {
            (&mut *tiling).set_completion_link(kicks[0], kicks[1], payload, fragment_busy);
            (&mut *fragment).set_completion_link(kicks[1], payload);
        }
        Ok(())
    }

    /// Writes the lead commands of `ordinal` from its linked descriptors: InitBM before the
    /// tiling descriptor when `init_manager`. Returns which stages have one.
    pub(super) fn write_leads(&mut self, ordinal: u64, init_manager: bool) -> Result<[bool; 2]> {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        let ta = self
            .graph
            .descriptors
            .pointer(self.graph.offset(0, ordinal), size_of::<abi::TaDescriptor>())?
            .cast::<abi::TaDescriptor>();
        // SAFETY: The view was checked during preparation and the ordinal still owns it; only
        // integer fields are read.
        let ta = unsafe { &*ta };
        let slot = ordinal as usize % RECORDS * CONFIG_STRIDE + LEAD;
        if init_manager {
            let init = queue::InitBufferManager::new(
                ta.context_id,
                ta.pb_slot,
                ta.pb_va,
                ta.stamp_value,
            );
            self.graph.config.write(CONFIG[0] + slot, init)?;
        }
        Ok([init_manager, false])
    }
}

impl Memory {
    /// Preserve the initial mapping order while moving VM-lifetime mappings
    /// out of the graph. Reserve all host storage before moving any mapping.
    /// Moves this drained graph's pool mappings out of its VM slot, so a new
    /// M5 graph of the same VM can install on the pair (see bind_owner).
    pub(super) fn take_pool_mappings(&mut self) -> Option<(u8, mmu::RenderPoolMappings)> {
        let slot = self.installed_pool?;
        let id = u64::from(self.lifecycle[0].predecessor);
        let mappings = self.context.vm().take_render_pool_mappings(slot, id)?;
        self.installed_pool = None;
        Some((slot, mappings))
    }
    /// Returns mappings moved out by `take_pool_mappings` after a failed install.
    pub(super) fn put_pool_mappings(
        &mut self,
        slot: u8,
        mappings: mmu::RenderPoolMappings,
    ) -> Result {
        self.context.vm().put_render_pool_mappings(slot, mappings)?;
        self.installed_pool = Some(slot);
        Ok(())
    }

    pub(super) fn take_persistent(
        &mut self,
        tvb: &mut KVec<mmu::KernelMapping>,
    ) -> Result<KVec<mmu::KernelMapping>> {
        if self.scratch.mapping.is_none()
            || self.discard.mapping.is_none()
            || (!self.graph.m5 && self.registers.as_ref().is_none_or(|r| r.gpu_va() == 0))
        {
            return Err(EIO);
        }
        if self.mesh_storage.as_ref().is_some_and(|m| m.mapping.is_none()) {
            return Err(EIO);
        }
        let mut aliases = KVec::new();
        aliases.reserve(tvb.len().checked_add(4).ok_or(EOVERFLOW)?, GFP_KERNEL)?;
        aliases.push(self.scratch.mapping.take().ok_or(EIO)?, GFP_KERNEL)?;
        aliases.push(self.discard.mapping.take().ok_or(EIO)?, GFP_KERNEL)?;
        if let Some(storage) = &mut self.mesh_storage {
            aliases.push(storage.mapping.take().ok_or(EIO)?, GFP_KERNEL)?;
        }
        if let Some(registers) = &mut self.registers {
            aliases.push(registers.take_mapping()?, GFP_KERNEL)?;
        }
        for mapping in core::mem::take(tvb) {
            aliases.push(mapping, GFP_KERNEL)?;
        }
        Ok(aliases)
    }
}

impl Memory {
    pub(super) fn bind_work_slot(&mut self, ordinal: u64, qids: [u8; 2], slot: u32) -> Result {
        if slot >= 4 || !self.owns(ordinal) {
            return Err(EINVAL);
        }
        if self.graph.m5 {
            return fw::m5::render_assembly::bind_work_slot(self.graph.m5_storage(ordinal)?, slot);
        }
        let queues = [
            self.graph
                .queues
                .word(usize::from(qids[1]) * size_of::<queue::QueueRecord>() + 0x38)?,
            self.graph
                .queues
                .word(usize::from(qids[0]) * size_of::<queue::QueueRecord>() + 0x38)?,
        ];
        let ta = self
            .graph
            .descriptors
            .pointer(self.graph.offset(0, ordinal), size_of::<abi::TaDescriptor>())?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(self.graph.offset(1, ordinal), size_of::<abi::FragmentSlot>())?
            .cast::<abi::FragmentSlot>();
        for word in queues {
            word.store(slot, Ordering::Relaxed);
        }
        // SAFETY: The generation owns these prepared, unpublished POD views.
        // Packed fields are assigned by value without an unaligned reference.
        unsafe {
            (*ta).unk_8be = slot;
            (*fragment).descriptor.unk_2158 = slot;
        }
        core::sync::atomic::fence(Ordering::Release);
        Ok(())
    }

    /// Retargets both retired queue records to `context`. A context of another
    /// scheduling profile also replaces the priority and policy words, after
    /// the owner fields; the same profile leaves them untouched.
    /// `rewrite`: store the M5 records even when their host-owned words
    /// already name `context`; without it that no-op read-modify-write, which
    /// would also store back firmware-owned words of a possibly running queue,
    /// is skipped. G17P always retargets.
    pub(super) fn set_owner(&self, qids: [u8; 2], context: &Context, rewrite: bool) -> Result {
        if self.graph.m5 {
            let records = qids.map(|qid| self.graph.queues.pointer(self.graph.queue_offset(qid),
                size_of::<fw::m5::queue::QueueRecord>()));
            let [first, second] = records;
            let pointers = [first?, second?].map(|p| p.cast::<fw::m5::queue::QueueRecord>());
            // SAFETY: Both complete records are in bounds. The caller owns
            // retired queues exclusively; build both updates before storing.
            unsafe {
                let mut values = pointers.map(|p| p.read_unaligned());
                if !rewrite {
                    let mut named = true;
                    for value in &values {
                        named &= value.names_owner(context.work_head_va(), context.scheduler_va(),
                            context.owner_pid(), context.policy())?;
                    }
                    if named {
                        return Ok(());
                    }
                }
                for value in &mut values {
                    value.set_compute_owner(context.work_head_va(), context.scheduler_va(),
                        context.owner_pid(), context.policy())?;
                }
                for (pointer, value) in pointers.into_iter().zip(values) {
                    pointer.write_unaligned(value);
                }
            }
            return Ok(());
        }
        let offsets = qids.map(|qid| usize::from(qid) * size_of::<queue::QueueRecord>());
        let base = self.graph.queues.pointer(0, GRAPH_SIZE)?;
        // Check every field before the first update, retaining the required
        // job-head pair, PID pair, then packed scheduler-pointer pair order.
        for at in offsets {
            self.graph.queues.pointer(at + 0x10, 8)?;
            self.graph.queues.pointer(at + 0x48, 4)?;
            self.graph.queues.pointer(at + 0x9c, 8)?;
            self.graph.queues.pointer(at + PRIORITY, 8)?;
            self.graph.queues.pointer(at + POLICY, 4)?;
        }
        let policy = context.policy();
        // SAFETY: Both queues are retired before retargeting. Complete byte
        // views were checked above; packed scheduler words use unaligned stores.
        unsafe {
            for at in offsets {
                base.add(at + 0x10)
                    .cast::<u64>()
                    .write_unaligned(context.work_head_va());
            }
            for at in offsets {
                base.add(at + 0x48)
                    .cast::<u32>()
                    .write_unaligned(context.owner_pid());
            }
            for at in offsets {
                base.add(at + 0x9c)
                    .cast::<u64>()
                    .write_unaligned(context.scheduler_va());
            }
            for at in offsets {
                let words = base.add(at + PRIORITY).cast::<[u32; 2]>();
                let priority = [u32::from(policy.priority()); 2];
                if words.read_unaligned() != priority {
                    words.write_unaligned(priority);
                }
                let word = base.add(at + POLICY).cast::<u32>();
                if word.read_unaligned() != u32::from(policy.policy()) {
                    word.write_unaligned(u32::from(policy.policy()));
                }
            }
        }
        Ok(())
    }
}

impl Drop for Memory {
    fn drop(&mut self) {
        if let Some(slot) = self.installed_pool {
            self.context.vm().clear_render_pool_mappings_if_owner(
                slot,
                u64::from(self.lifecycle[0].predecessor),
            );
        }
    }
}
