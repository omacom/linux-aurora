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
        let mut range = dynamic::LOWER;
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
    aliases: [Aliases; 2],
    pub(super) descriptors: PooledObject,
    pub(super) queues: PooledObject,
    pub(super) config: PooledObject,
    pub(super) support: KernelObject,
    pub(super) owners: [Option<u64>; RECORDS],
    /// Host words of the pointer blocks in this firmware build.
    pub(super) pointers: fw::variant::PointerLayout,
}
impl Graph {
    pub(super) fn new(
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        context: &Context,
        qids: [u8; 2],
        pointers: fw::variant::PointerLayout,
    ) -> Result<Self> {
        let prot = mmu::PROT_GPU_FW_SHARED_RW;
        let descriptors = pool.allocate(
            alloc,
            Family::Descriptor,
            TA_BYTES + FRAGMENT_BYTES,
            TA_BYTES + FRAGMENT_BYTES,
            prot,
            CpuMap::WriteBack,
        )?;
        let ta = Aliases::new(&descriptors, alloc, context.vm(), 0..TA_BYTES)?;
        let fragment = Aliases::new(
            &descriptors,
            alloc,
            context.vm(),
            TA_BYTES..TA_BYTES + FRAGMENT_BYTES,
        )?;
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
            CpuMap::WriteBack,
        )?;
        let support = alloc.kernel(
            storage::SUPPORT_SIZE,
            mmu::UAT_PGSZ as u64,
            prot,
            CpuMap::WriteCombined,
        )?;
        let address = queues.gpu_va();
        for stage in 0..2 {
            queues.write(
                qids[stage] as usize * size_of::<queue::QueueRecord>(),
                queue::QueueRecord::new(&queue::QueueRecordArgs {
                    pointers_va: address + POINTERS[stage] as u64,
                    items_va: address + ITEMS[stage] as u64,
                    job_list_va: context.work_head_va(),
                    scheduler_va: context.scheduler_va(),
                    owner_pid: context.owner_pid(),
                    policy: context.policy(),
                }),
            )?;
            queues.initialize::<queue::PointerBlock>(POINTERS[stage], |view| {
                *view = queue::PointerBlock::new(&pointers)?;
                Ok(())
            })?;
        }
        queues.write(JOB_LIST, queue::JobListHead::new(address + JOB_LIST as u64))?;
        Ok(Self {
            aliases: [ta, fragment],
            descriptors,
            queues,
            config,
            support,
            owners: [None; RECORDS],
            pointers,
        })
    }
    pub(super) fn offset(stage: usize, ordinal: u64) -> usize {
        let slot = ordinal as usize % RECORDS;
        if stage == 0 {
            slot * abi::TA_DESCRIPTOR_SIZE
        } else {
            TA_BYTES + slot * abi::FRAGMENT_SLOT_SIZE
        }
    }
    pub(super) fn descriptor_va(&self, stage: usize, ordinal: u64) -> u64 {
        self.descriptors.gpu_va() + Self::offset(stage, ordinal) as u64
    }
    pub(super) fn descriptor_client(&self, stage: usize, ordinal: u64) -> u64 {
        let stride = if stage == 0 {
            abi::TA_DESCRIPTOR_SIZE
        } else {
            abi::FRAGMENT_SLOT_SIZE
        };
        self.aliases[stage].client.iova() + (ordinal as usize % RECORDS * stride) as u64
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
impl Compact {
    fn new(alloc: &Allocator<'_>, vm: &mmu::Vm, size: usize) -> Result<Self> {
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

pub(super) struct Generation {
    deflake: mmu::KernelMapping,
    fragment_client: mmu::KernelMapping,
    state: KernelObject,
    fragment: KernelObject,
    pub(super) ordinal: Option<u64>,
}
impl Generation {
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
    _fragment_view: mmu::KernelMapping,
    pub(super) graph: Graph,
    scratch: Compact,
    discard: Compact,
    pub(super) target: KBox<target::Target>,
    generations: [Option<KBox<Generation>>; DEPTH],
    generation_count: usize,
    pub(super) timestamps: KernelObject,
    pub(super) manager: buffer::Manager,
    pub(super) lifecycle: [fw::ObjectIds; 2],
    // The extra client views retain their allocation positions even though
    // kick entries consume the register arrays embedded in the descriptors.
    registers: KernelObject,
    pub(super) context: Arc<Context>,
    pub(super) installed_pool: Option<u8>,
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
    ) -> Result<(KBox<Self>, KVec<mmu::KernelMapping>)> {
        let storage = KBox::<Self>::new_uninit(GFP_KERNEL)?;
        let vm = context.vm();
        let mut graph = Graph::new(alloc, pool, &context, qids, variant.pointers)?;
        let mut target = target::Target::new(alloc.dev, vm, layout)?;
        target.prepare(ordinal, layout)?;
        let state = Generation::allocate_state(alloc, vm)?;
        let scratch = Compact::new(alloc, vm, cfg::SCENE_SIZE as usize)?;
        let discard = Compact::new(alloc, vm, cfg::DISCARD_SIZE as usize)?;
        let (tvb, mappings) = buffer::Tvb::new(alloc.dev, vm, layout.tvb_blocks)?;
        let count = DEPTH;
        let initial = if ordinal == 0 {
            0
        } else {
            (ordinal ^ 1) as usize % count
        };
        let mut generations = core::array::from_fn(|_| None);
        generations[initial] = Some(Generation::new(alloc, vm, state)?);
        for offset in 1..count {
            let index = (initial + offset) % count;
            let state = Generation::allocate_state(alloc, vm)?;
            generations[index] = Some(Generation::new(alloc, vm, state)?);
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
        graph.support.initialize::<storage::Support>(0, |view| {
            view.init(
                support_va,
                alloc.uat.geometry().kernel_range().start + kernel_window::INDEX,
                low::RENDER_RUN_LIST,
                low::RENDER_PAGE_LIST,
                word,
            )
        })?;
        let registers = KernelObject::new(
            alloc.dev,
            vm,
            crate::g17::object::Placement::In(dynamic::CLIENT_LOWER, mmu::UAT_PGSZ as u64),
            mmu::UAT_PGSZ,
            mmu::PROT_GPU_SHARED_RO,
            CpuMap::WriteCombined,
        )?;
        let fragment_view = graph.descriptors.map_range(
            vm,
            TA_BYTES..TA_BYTES + mmu::UAT_PGSZ,
            dynamic::CLIENT_LOWER,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RO,
            0,
        )?;
        Ok((
            storage.write_init(kernel::try_init!(Self {
                graph,
                scratch,
                discard: discard,
                target,
                generations,
                generation_count: count,
                timestamps,
                manager,
                lifecycle: [tiling, fragment],
                _fragment_view: fragment_view,
                registers: registers,
                context,
                installed_pool: None,
            }))?,
            mappings,
        ))
    }
    pub(super) fn generation(&mut self, ordinal: u64) -> Result<&mut Generation> {
        self.generations[ordinal as usize % self.generation_count]
            .as_deref_mut()
            .ok_or(EIO)
    }
    pub(super) fn owns(&self, ordinal: u64) -> bool {
        self.graph.owns(ordinal)
            && self.generations[ordinal as usize % self.generation_count]
                .as_ref()
                .is_some_and(|generation| generation.ordinal == Some(ordinal))
    }
    pub(super) fn pass(&self, ordinal: u64) -> Result<[u64; 2]> {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        let offset = ordinal as usize % RECORDS * 16;
        Ok([
            self.timestamps.dword(offset)?.load(Ordering::Relaxed),
            self.timestamps.dword(offset + 8)?.load(Ordering::Relaxed),
        ])
    }
    pub(super) fn owners(&self) -> [Option<u64>; DEPTH] {
        core::array::from_fn(|index| self.generations[index].as_ref().and_then(|g| g.ordinal))
    }
    pub(super) fn retire(&mut self, ordinal: u64) -> Result {
        if !self.owns(ordinal) {
            return Err(EIO);
        }
        self.target.retire(ordinal)?;
        self.manager.retire_scene(ordinal)?;
        self.graph.release(ordinal)?;
        self.generation(ordinal)?.ordinal = None;
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
            (&self.graph.descriptors, Graph::offset(0, ordinal), size_of::<abi::TaDescriptor>()),
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
                Graph::offset(0, args.ordinal),
                size_of::<abi::TaDescriptor>(),
            )?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(
                Graph::offset(1, args.ordinal),
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
        let tiling = self
            .graph
            .descriptors
            .pointer(Graph::offset(0, ordinal), size_of::<abi::TaDescriptor>())?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(Graph::offset(1, ordinal), size_of::<abi::FragmentSlot>())?
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
            .pointer(Graph::offset(0, ordinal), size_of::<abi::TaDescriptor>())?
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
    pub(super) fn take_persistent(
        &mut self,
        tvb: &mut KVec<mmu::KernelMapping>,
    ) -> Result<KVec<mmu::KernelMapping>> {
        if self.scratch.mapping.is_none()
            || self.discard.mapping.is_none()
            || self.registers.gpu_va() == 0
        {
            return Err(EIO);
        }
        let mut aliases = KVec::new();
        aliases.reserve(tvb.len().checked_add(3).ok_or(EOVERFLOW)?, GFP_KERNEL)?;
        aliases.push(self.scratch.mapping.take().ok_or(EIO)?, GFP_KERNEL)?;
        aliases.push(self.discard.mapping.take().ok_or(EIO)?, GFP_KERNEL)?;
        aliases.push(self.registers.take_mapping()?, GFP_KERNEL)?;
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
            .pointer(Graph::offset(0, ordinal), size_of::<abi::TaDescriptor>())?
            .cast::<abi::TaDescriptor>();
        let fragment = self
            .graph
            .descriptors
            .pointer(Graph::offset(1, ordinal), size_of::<abi::FragmentSlot>())?
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
    pub(super) fn set_owner(&self, qids: [u8; 2], context: &Context) -> Result {
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
