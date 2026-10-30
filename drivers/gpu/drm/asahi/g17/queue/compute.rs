// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Retained physical compute queues. Logical ownership and command retirement never free an
//! installed graph; the device retains it until both firmware processors have stopped.

pub(crate) mod exit;
pub(crate) mod retirement;

use crate::g17::{
    channel::Rings,
    context::Context,
    fw::{
        channels::ControlRecord,
        compute::{OperandState, COMPUTE_DESCRIPTOR_SIZE},
        queue::*,
        variant::PointerLayout,
    },
    object::{Allocator, CpuMap, Family, KernelObject, Pool, PooledObject},
};
use crate::{hw::t8140::dynamic, mmu};
use core::sync::atomic::{fence, Ordering};
use kernel::{prelude::*, sync::Arc};
use retirement::{Cursors, RECORD_SLOTS};

use crate::g17::now_ns;

/// Descriptor slots per queue. A kick's descriptor is rewritten when the kick 32 slots later
/// is published; with IN_FLIGHT kicks outstanding at most, that kick has completed by then.
const DESCRIPTORS: usize = 32;
const GRAPH_SIZE: usize = 0x8000;
const CONTEXT_SIZE: usize = 0x20000;
const RECORD_STRIDE: usize = 0x1000;
const NATIVE_WAIT_OFFSET: usize = 0x900;
const NATIVE_WAIT_STRIDE: usize = 0x60;
// A submission can name every physical compute channel. Keep the complete
// wait list inside its own cached slot, including the last retained slot.
const CACHED_RECORD_STRIDE: usize = (NATIVE_WAIT_OFFSET
    + crate::g17::kick::COMPUTE_CHANNELS * NATIVE_WAIT_STRIDE)
    .next_power_of_two();
const CONFIG: usize = 0;
const ANNOUNCE: usize = 0x800;
const PREDECESSOR: usize = 0x400;
const POINTERS: usize = 0x100;
/// Producer/consumer words live on their own uncached page after the graph,
/// which the firmware reads through a cached mapping (queue record, job-list
/// head and item ring), as native does.
const CURSORS: usize = GRAPH_SIZE;
const ITEMS: usize = 0x4000;
const STAMP: usize = 0x300;

#[repr(C)]
struct GraphLayout {
    queue: QueueRecord,
    unk_c0: [u8; 0x40],
    pointers: PointerBlock,
    unk_180: [u8; 0x80],
    jobs: JobListHead,
    unk_210: [u8; 0x3df0],
    items: [u64; ITEM_RING_ENTRIES as usize],
    unk_6800: [u8; 0x1800],
}
// SAFETY: The layout contains only integer storage and integer atomics, all valid when zeroed.
unsafe impl Zeroable for GraphLayout {}
static_assert!(core::mem::size_of::<GraphLayout>() == GRAPH_SIZE);
static_assert!(core::mem::offset_of!(GraphLayout, pointers) == POINTERS);
static_assert!(core::mem::offset_of!(GraphLayout, items) == ITEMS);

/// Aliases are destroyed before the logical root they name. Superseded bindings remain owned
/// until the first exact successful publication under their replacement has retired.
/// Backing of a vacated pool and descriptor ring, freed outside the device
/// mutex. Field order drops each client alias before the storage it names.
pub(crate) struct Vacated {
    _alias: Option<mmu::KernelMapping>,
    _pages: Option<crate::g17::freelist::Pages>,
    _descriptors: Option<mmu::KernelMapping>,
    _ring: Option<DescriptorRing>,
}

pub(crate) struct Binding {
    _kick_alias: mmu::KernelMapping,
    /// Client alias of the USC pool pages; dropped when the pool is vacated and
    /// recreated when an owner binds a re-armed pool (it pins the backing).
    pool_alias: Option<mmu::KernelMapping>,
    /// Client alias of the descriptor ring; dropped with the ring when the queue
    /// is vacated and recreated when an owner binds a re-armed queue.
    descriptors: Option<mmu::KernelMapping>,
    preempt: mmu::KernelMapping,
    usage: mmu::KernelMapping,
    operand: mmu::KernelMapping,
    _queue_context: mmu::KernelMapping,
    context: Arc<Context>,
}

/// Descriptor ring of one compute QID with its kernel-VM alias. Built without
/// the device mutex; installed into a vacant graph before an owner binds.
pub(crate) struct DescriptorRing {
    // Drop the GPU alias before the retained storage returns to its pool.
    low: mmu::KernelMapping,
    object: PooledObject,
}

impl DescriptorRing {
    /// Firmware reads every descriptor through a cached kernel mapping, as it
    /// does for render. The pool keeps that mapping until processor stop; the
    /// hardware reads the same pages through an uncached canonical low alias.
    pub(crate) fn new(alloc: &Allocator<'_>, objects: &Arc<Pool>) -> Result<Self> {
        let size = DESCRIPTORS * COMPUTE_DESCRIPTOR_SIZE;
        let object = objects.allocate(
            alloc,
            Family::Descriptor,
            size,
            size,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let low = object.alias_in(
            alloc.uat.kernel_lower_vm(),
            dynamic::LOWER,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RW,
        )?;
        Ok(Self { low, object })
    }
}

/// Firmware-shared storage of one compute QID, separate from its common kick ring.
/// `ring` is `None` while the queue is vacant (idle, no owner, pool given back).
struct Graph {
    ring: Option<DescriptorRing>,
    preempt: KernelObject,
    usage: KernelObject,
    operand: KernelObject,
    support: KernelObject,
    graph: PooledObject,
    context_low: KernelObject,
    context_high: KernelObject,
    /// Announcements, which stay uncached for the firmware.
    records: KernelObject,
    /// Configuration, predecessor and completion-wait records at the same
    /// offsets, which the firmware reads through a cached mapping.
    cached_records: PooledObject,
    record_ends: [Option<u32>; RECORD_SLOTS],
    reclaimed: u32,
    /// Host words of the pointer block in this firmware build.
    pointers: PointerLayout,
    /// Layout of the operand-state buffer for the GPU's cores.
    operand_state: &'static OperandState,
}

impl Graph {
    fn new(
        alloc: &Allocator<'_>,
        objects: &Arc<Pool>,
        context: &Context,
        pointers: PointerLayout,
        operand_state: &'static OperandState,
    ) -> Result<Self> {
        let page = mmu::UAT_PGSZ as u64;
        let prot = mmu::PROT_GPU_FW_SHARED_RW;
        let cpu = CpuMap::WriteCombined;
        let ring = DescriptorRing::new(alloc, objects)?;
        let preempt = alloc.kernel(0x4000, page, prot, cpu)?;
        let usage = alloc.kernel(mmu::UAT_PGSZ, page, prot, cpu)?;
        let operand = alloc.lower(operand_state.size, page, prot, cpu)?;
        let mut support = alloc.kernel(0xc000, page, prot, cpu)?;
        let support_va = support.gpu_va();
        support.write(0, support_va + page)?;
        support.write(0x100, support_va + page + 4)?;
        support.write(0x110, 0x50u32)?;
        support.write(mmu::UAT_PGSZ + 4, 1u32)?;
        let mut graph = objects.allocate(
            alloc,
            Family::QueueState,
            GRAPH_SIZE + mmu::UAT_PGSZ,
            GRAPH_SIZE,
            prot,
            cpu,
        )?;
        let graph_va = graph.gpu_va();
        graph.initialize::<GraphLayout>(0, |view| {
            view.queue = QueueRecord::new(&QueueRecordArgs {
                pointers_va: graph_va + CURSORS as u64,
                items_va: graph_va + ITEMS as u64,
                job_list_va: context.work_head_va(),
                scheduler_va: context.scheduler_va(),
                owner_pid: context.owner_pid(),
                policy: context.policy(),
            });
            view.pointers = PointerBlock::new(&pointers)?;
            view.jobs = JobListHead::new(graph_va + 0x200);
            Ok(())
        })?;
        graph.initialize::<PointerBlock>(CURSORS, |view| {
            *view = PointerBlock::new(&pointers)?;
            Ok(())
        })?;
        let context_low = alloc.lower(CONTEXT_SIZE, page, prot, cpu)?;
        let context_high = alloc.kernel(CONTEXT_SIZE, page, prot, cpu)?;
        let records = alloc.kernel(RECORD_SLOTS * RECORD_STRIDE, page, prot, cpu)?;
        let cached_records = objects.allocate(
            alloc,
            Family::QueueConfig,
            RECORD_SLOTS * CACHED_RECORD_STRIDE,
            RECORD_SLOTS * CACHED_RECORD_STRIDE,
            prot,
            cpu,
        )?;
        Ok(Self {
            ring: Some(ring),
            preempt,
            usage,
            operand,
            support,
            graph,
            context_low,
            context_high,
            records,
            cached_records,
            record_ends: [None; RECORD_SLOTS],
            reclaimed: 0,
            pointers,
            operand_state,
        })
    }

    /// All fallible mapping work finishes before replacing a physical queue's logical owner.
    fn bind(
        &mut self,
        context: Arc<Context>,
        kick_alias: mmu::KernelMapping,
        pool_alias: mmu::KernelMapping,
    ) -> Result<Binding> {
        if !context.is_current() {
            return Err(EFAULT);
        }
        let vm = context.vm();
        vm.install_job_context_aliases()?;
        let descriptors = self.descriptor_alias(vm)?;
        let prot = mmu::PROT_GPU_FW_SHARED_RW;
        let page = mmu::UAT_PGSZ as u64;
        let preempt = self
            .preempt
            .alias_in(vm, dynamic::CLIENT_LOWER, page, prot)?;
        let usage = self.usage.alias_in(vm, dynamic::CLIENT_LOWER, page, prot)?;
        let operand = self
            .operand
            .alias_in(vm, dynamic::CLIENT_LOWER, page, prot)?;
        let queue_context = self
            .context_low
            .alias_in(vm, dynamic::CLIENT_LOWER, page, prot)?;
        Ok(Binding {
            _kick_alias: kick_alias,
            pool_alias: Some(pool_alias),
            descriptors: Some(descriptors),
            preempt,
            usage,
            operand,
            _queue_context: queue_context,
            context,
        })
    }

    /// Caller has excluded publication and proved the retained graph idle.
    fn set_owner(&mut self, context: &Context) -> Result {
        let pointer = self
            .graph
            .pointer(0, core::mem::size_of::<QueueRecord>())?
            .cast::<QueueRecord>();
        // SAFETY: The checked record is in the retained mapping, has byte alignment and consists
        // only of integer storage. The device mutex and idle witness exclude simultaneous access.
        unsafe {
            (*pointer).set_compute_owner(
                context.work_head_va(),
                context.scheduler_va(),
                context.owner_pid(),
                context.policy(),
            )
        };
        Ok(())
    }

    fn cursors(&self) -> Result<Cursors> {
        Ok(Cursors {
            producer: self
                .graph
                .word(CURSORS + self.pointers.producer)?
                .load(Ordering::Relaxed),
            consumer: self.graph.word(CURSORS)?.load(Ordering::Relaxed),
            count: self.graph.word(CURSORS + 0x60)?.load(Ordering::Relaxed),
        })
    }
    fn stamp(&self) -> Result<u32> {
        fence(Ordering::Acquire);
        Ok(self.support.word(STAMP)?.load(Ordering::Relaxed))
    }
    /// EAGAIN while the queue is vacant: the owner must re-arm it first.
    fn descriptor_vas(&self, slot: u8) -> Result<(u64, u64)> {
        let ring = self.ring.as_ref().ok_or(EAGAIN)?;
        let offset = (usize::from(slot) % DESCRIPTORS * COMPUTE_DESCRIPTOR_SIZE) as u64;
        Ok((ring.object.gpu_va() + offset, ring.low.iova() + offset))
    }
    fn descriptor_alias(&self, vm: &mmu::Vm) -> Result<mmu::KernelMapping> {
        let ring = self.ring.as_ref().ok_or(EAGAIN)?;
        ring.object
            .map_alias(vm, ring.low.iova(), mmu::PROT_GPU_SHARED_RO)
    }
    /// Detaches the descriptor ring of an idle vacant queue; the caller frees it
    /// off-lock. The job list and the context item only name consumed
    /// descriptors at this point.
    fn vacate_ring(&mut self) -> Option<DescriptorRing> {
        self.ring.take()
    }
    fn rearm_ring(&mut self, ring: DescriptorRing) -> Result {
        if self.ring.is_some() {
            return Err(EBUSY);
        }
        self.ring = Some(ring);
        Ok(())
    }
    fn ensure_record_slot(&self, slot: usize) -> Result {
        if let Some(end) = *self.record_ends.get(slot).ok_or(EINVAL)? {
            let cursors = self.cursors()?;
            if !cursors.consumed(end, cursors.count)? {
                return Err(EAGAIN);
            }
        }
        Ok(())
    }

    /// A killed queue's ring restarts empty at the firmware's consumer cursor: every
    /// item slot is cleared and the host producer is set to the consumer. Only
    /// valid after the firmware acknowledged the kill and no command is pending.
    fn reset_items(&mut self) -> Result {
        let cursors = self.cursors()?;
        if cursors.count != ITEM_RING_ENTRIES || cursors.consumer >= cursors.count {
            return Err(EIO);
        }
        for index in 0..ITEM_RING_ENTRIES as usize {
            self.graph
                .dword(ITEMS + index * 8)?
                .store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        self.graph
            .word(CURSORS + self.pointers.producer)?
            .store(cursors.consumer, Ordering::Relaxed);
        self.reclaimed = cursors.consumer;
        fence(Ordering::SeqCst);
        Ok(())
    }
    /// Reclaim only consumed item pointers, with individual aligned stores and a release fence.
    /// This leaves records still named by firmware untouched across arbitrarily many wraps.
    fn recycle(&mut self, cursors: Cursors) -> Result {
        if cursors.count != ITEM_RING_ENTRIES {
            return Err(EIO);
        }
        if cursors.consumer >= cursors.count || self.reclaimed >= cursors.count {
            return Err(ERANGE);
        }
        let done = cursors.consumer;
        while self.reclaimed != done {
            self.graph
                .dword(ITEMS + self.reclaimed as usize * 8)?
                .store(0, Ordering::Relaxed);
            self.reclaimed = (self.reclaimed + 1) % cursors.count;
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
}

use crate::g17::{
    completion::Deferred, context::WorkStateLease, freelist::ComputePool, job::Packet, kick, qos,
    recovery, status::VmStatus,
};
use retirement::{Batch, Observation, Recordless, Ticket, IN_FLIGHT};

struct Fifo<T> {
    entries: [Option<T>; IN_FLIGHT],
    head: usize,
    len: usize,
}
impl<T> Fifo<T> {
    fn new() -> Self {
        Self {
            entries: core::array::from_fn(|_| None),
            head: 0,
            len: 0,
        }
    }
    fn front(&self) -> Option<&T> {
        self.entries[self.head].as_ref()
    }
    fn back(&self) -> Option<&T> {
        if self.len == 0 {
            None
        } else {
            self.entries[(self.head + self.len - 1) % IN_FLIGHT].as_ref()
        }
    }
    fn back_mut(&mut self) -> Option<&mut T> {
        if self.len == 0 {
            None
        } else {
            self.entries[(self.head + self.len - 1) % IN_FLIGHT].as_mut()
        }
    }
    fn push(&mut self, value: T) -> core::result::Result<(), T> {
        if self.len == IN_FLIGHT {
            return Err(value);
        }
        self.entries[(self.head + self.len) % IN_FLIGHT] = Some(value);
        self.len += 1;
        Ok(())
    }
    fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let value = self.entries[self.head].take();
        self.head = (self.head + 1) % IN_FLIGHT;
        self.len -= 1;
        value
    }
    fn pop_back(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        self.len -= 1;
        self.entries[(self.head + self.len) % IN_FLIGHT].take()
    }
    fn iter(&self) -> impl Iterator<Item = &T> {
        Iterator::chain(
            self.entries[self.head..].iter(),
            self.entries[..self.head].iter(),
        )
        .take(self.len)
        .filter_map(Option::as_ref)
    }
    fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        let (before, after) = self.entries.split_at_mut(self.head);
        Iterator::chain(after.iter_mut(), before.iter_mut())
            .take(self.len)
            .filter_map(Option::as_mut)
    }
}

struct Active {
    packet: Arc<Packet>,
    published: u64,
    work: Arc<WorkStateLease>,
    ticket: Option<Ticket>,
    observed: Option<Observation>,
    /// Host time the completion record was first observed.
    observed_at: Option<u64>,
    failure_deferred: bool,
    qos_pending: bool,
}
struct Previous<B = Binding, C = Arc<Context>> {
    binding: Option<B>,
    context: C,
    publication: Option<(u64, u64)>,
}
impl<B, C> Previous<B, C> {
    /// The exact drain witness covers the replacement publication even when
    /// its public fence failed. An unpublished replacement still needs rollback.
    fn retire_witnessed(previous: &mut Option<Self>) {
        if previous.as_ref().is_some_and(|p| p.publication.is_some()) {
            drop(previous.take());
        }
    }
}

/// Per-queue addresses of the bound owner, read under the publication preflight.
struct BindingAddresses {
    preempt: u64,
    operand: u64,
    usage: u64,
}

#[derive(Copy, Clone, PartialEq, Eq)]
struct ConfigIdentity {
    scheduler: u64,
    qos: u8,
    context: u32,
}

/// A private physical pool whose descriptor row is installed before allocating its kick ring.
/// The QID remains unpublished throughout construction.
pub(crate) struct PoolBacking {
    pool: ComputePool,
    id: kick::Id,
}

impl PoolBacking {
    pub(crate) fn new(alloc: &Allocator<'_>, id: kick::Id, blocks: usize) -> Result<Self> {
        let pool = ComputePool::new(alloc, id.qid(), blocks)?;
        Ok(Self { pool, id })
    }

    /// Fresh page-pool row data, installed once while the QID is still unpublished.
    pub(crate) fn pool_descriptor(&self) -> Result<(u16, u64)> {
        Ok((self.pool.id(), self.pool.page_list_va()?))
    }
}

/// A physical pool and kick ring prepared before the device-global compute scheduler and
/// descriptor graph. No logical VM aliases or firmware publication exist at this stage.
pub(crate) struct Backing {
    pool: ComputePool,
    kick: kick::Queue,
}

impl Backing {
    pub(crate) fn new(alloc: &Allocator<'_>, backing: PoolBacking) -> Result<Self> {
        let PoolBacking { pool, id } = backing;
        let kick = kick::Queue::compute(alloc, id)?;
        Ok(Self { pool, kick })
    }
}

/// Installed graph and its exact logical owner. A released idle graph may change owners only
/// after a retirement witness. Failed work retains its pointers independently of fence state.
/// USC pool backing of a retained queue. A release is published only for an idle
/// queue without an owner; the backing is dropped once the firmware consumed it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum PoolPhase {
    Populated,
    ReleaseSent(u32),
    Vacant,
}

/// Builder ownership of one physical release transaction. The preparation
/// lease pins this registry until abort/commit; owner close may remove lookups.
#[derive(Clone, Copy)]
pub(crate) struct Growth {
    qid: u8,
    phase: PoolPhase,
}

pub(crate) struct Queue {
    owner: Option<u64>,
    /// Scheduler/work pages of an exited last owner whose context was dropped;
    /// the installed queue record still names them until the next owner binds.
    retained_owner: Option<crate::g17::context::FirmwarePages>,
    pool_phase: PoolPhase,
    release_sent_ns: Option<u64>,
    growing: bool,
    growth_blocks: usize,
    /// A killed queue was recycled: its pool/ring must be vacated and re-armed
    /// fresh before any owner binds it (set by `recycle_killed`, cleared by `rearm`).
    needs_fresh_backing: bool,
    /// Monotonic time the last owner released this queue (vacate order).
    released_at: u64,
    /// Free-list generation of the last published configuration.
    free_list_generation: u64,
    /// `None` after an exited owner was detached; a new owner binds afresh.
    binding: Option<Binding>,
    previous: Option<Previous>,
    /// This binding has never crossed the successor visibility boundary.
    unpublished_binding: bool,
    kick: kick::Queue,
    pool: ComputePool,
    graph: Graph,
    active: Fifo<Active>,
    replays: Fifo<Arc<Packet>>,
    replay_retirement: bool,
    replay_error: Option<Error>,
    replay_prepared: bool,
    exit: Option<exit::Transaction>,
    /// Since when the kill of this failed queue's context waits for other contexts' compute,
    /// and whether that wait was reported.
    kill_wait: Option<(u64, bool)>,
    installed: bool,
    /// Bit `p` set once a work slot of priority class `p` named this queue.
    outer_started: u8,
    published_config: Option<ConfigIdentity>,
    /// Last configuration record the firmware consumed, and its recovery
    /// epoch. A command repeats the record only when it differs.
    config_cache: Option<(QueueConfig, u64)>,
    ordinal: u32,
    submitted: u32,
    /// The stamp of the newest command the firmware reported failed ([`Self::note_fault`]),
    /// while it is newer than the stored stamp.
    fault_stamp: Option<u32>,
    /// Prepared packets of the bound owner about to publish (see [`Self::claim`]).
    claims: u32,
    last_published: u64,
    last_end: u64,
    released: bool,
    retirement_ready: bool,
    quarantined: bool,
    retired_by_teardown: bool,
    retire_pending: bool,
    awaiting_witness: bool,
    spared_quarantine: bool,
    spared_deferred: bool,
    terminal: Option<(Arc<VmStatus>, Error)>,
    quarantine_error: Option<Error>,
    failure_status_pending: bool,
    retirement_proved: bool,
    source_readers: Arc<core::sync::atomic::AtomicU32>,
    source_hint: crate::g17::sync_input::ReaderHint,
}

impl Queue {
    /// The caller reserves a QID first, cancels it on construction failure, and retains every
    /// successfully installed queue until processor stop. Shared VM apertures precede this call.
    pub(crate) fn new(
        alloc: &Allocator<'_>,
        objects: &Arc<Pool>,
        backing: Backing,
        owner: u64,
        context: Arc<Context>,
        pointers: PointerLayout,
        operand_state: &'static OperandState,
    ) -> Result<Self> {
        let Backing { mut pool, mut kick } = backing;
        let mut graph = Graph::new(alloc, objects, &context, pointers, operand_state)?;
        let kick_alias = kick.map_client(context.vm())?;
        let pool_alias = pool.map_client(context.vm())?;
        let binding = graph.bind(context.clone(), kick_alias, pool_alias)?;
        context.mark_published();
        context.note_compute_binding(true);
        crate::g17::trace::bind(crate::g17::trace::COMPUTE, kick.id().qid(), context.id(), 0);
        Ok(Self {
            owner: Some(owner),
            retained_owner: None,
            pool_phase: PoolPhase::Populated,
            release_sent_ns: None,
            growing: false,
            growth_blocks: 0,
            needs_fresh_backing: false,
            released_at: 0,
            free_list_generation: 0,
            binding: Some(binding),
            previous: None,
            unpublished_binding: true,
            kick,
            pool,
            graph,
            active: Fifo::new(),
            replays: Fifo::new(),
            replay_retirement: false,
            replay_error: None,
            replay_prepared: false,
            exit: None,
            kill_wait: None,
            installed: false,
            outer_started: 0,
            published_config: None,
            config_cache: None,
            ordinal: 0,
            submitted: 0,
            fault_stamp: None,
            claims: 0,
            last_published: 0,
            last_end: 0,
            released: false,
            retirement_ready: false,
            quarantined: false,
            retired_by_teardown: false,
            retire_pending: false,
            awaiting_witness: false,
            spared_quarantine: false,
            spared_deferred: false,
            terminal: None,
            quarantine_error: None,
            failure_status_pending: false,
            retirement_proved: false,
            source_readers: Arc::new(core::sync::atomic::AtomicU32::new(0), GFP_KERNEL)?,
            source_hint: crate::g17::sync_input::ReaderHint::new(),
        })
    }
    /// The firmware failed the command whose completion stamp is `stamp`. It writes a
    /// zero-duration completion record for it, which quarantines the queue, but stores no
    /// stamp: without this the quarantine would wait for its retirement witness forever and
    /// keep the queue's backing. Commands published after the failed one still need their own
    /// stamp or record.
    pub(crate) fn note_fault(&mut self, stamp: u32) {
        if self
            .fault_stamp
            .is_none_or(|old| retirement::stamp_newer(stamp, old))
        {
            self.fault_stamp = Some(stamp);
        }
    }

    /// The firmware failed a command of this quarantined queue, and the queue still waits for
    /// its retirement witness: commands published after the failed one, which the firmware
    /// never runs, or the failed command itself, whose ring entry it does not consume (a ray
    /// traversal's, on Neo). Only a kill of the queue's context gives them back.
    pub(crate) fn fault_pending(&self) -> bool {
        self.fault_stamp.is_some()
            && self.quarantined
            && self.awaiting_witness
            && self.exit.is_none()
    }

    /// The stamp that proves retirement: the stored one, or a newer reported fault's.
    fn witness_stamp(&self) -> Result<u32> {
        let stored = self.graph.stamp()?;
        Ok(match self.fault_stamp {
            Some(fault) if retirement::stamp_newer(fault, stored) => fault,
            _ => stored,
        })
    }

    pub(crate) fn qid(&self) -> u8 {
        self.kick.id().qid()
    }
    pub(crate) fn owner(&self) -> Option<u64> {
        self.owner
    }
    /// The bound owner's context; `None` once an exited owner was detached.
    pub(crate) fn context(&self) -> Option<&Arc<Context>> {
        self.binding.as_ref().map(|binding| &binding.context)
    }
    /// The first command after an owner change away from `context` has been
    /// published and not retired: the queue still runs that scheduler's state.
    pub(crate) fn switching_from(&self, context: &Arc<Context>) -> bool {
        self.previous.as_ref().is_some_and(|previous| {
            previous.publication.is_some() && Arc::ptr_eq(&previous.context, context)
        })
    }
    /// The context these queue records were bound to before an owner change
    /// whose first command is not published yet.
    pub(crate) fn pending_switch_from(&self) -> Option<&Arc<Context>> {
        self.previous
            .as_ref()
            .filter(|previous| previous.publication.is_none())
            .map(|previous| &previous.context)
    }
    /// Quarantined commands still retain the scheduler until their retirement witness.
    pub(crate) fn idle_for_context(&self, context: &Arc<Context>) -> bool {
        !self
            .context()
            .is_some_and(|bound| Arc::ptr_eq(bound, context))
            || (self.active.len == 0 && self.replays.len == 0)
    }
    pub(crate) fn in_flight(&self) -> bool {
        self.active.len != 0 && !self.quarantined
    }
    /// The oldest command is a published long job (ray traversal or AS build) whose
    /// completion record the driver has not seen yet.
    pub(crate) fn long_job_running(&self) -> bool {
        !self.quarantined
            && self.active.front().is_some_and(|active| {
                active.observed.is_none()
                    && active.packet.is_published()
                    && active.packet.command.long_job()
            })
    }
    /// Quarantines without a pending witness no longer describe live pipe order.
    pub(crate) fn timeout_head(&self) -> Option<crate::g17::timeout::Head> {
        if self.quarantined && !self.awaiting_witness {
            return None;
        }
        self.active.front().map(|active| crate::g17::timeout::Head {
            vm: active.packet.context.vm().id(),
            published: active.published,
            finished: active.observed.is_some(),
            failed: self.quarantined,
        })
    }

    pub(crate) fn can_classify_timeout(&self, packet: &Arc<Packet>) -> bool {
        !self.quarantined && self.owns_packet(packet)
    }

    pub(crate) fn oldest_spared(&self) -> Option<bool> {
        self.active
            .front()
            .map(|a| a.packet.completion.spared() && a.packet.completion.replay_count() == 0)
    }
    pub(crate) fn visit_owners(&self, visit: &mut dyn FnMut(&Arc<VmStatus>)) {
        for active in self.active.iter() {
            visit(active.packet.completion.status());
        }
    }
    /// A recovery record names this queue and the completion stamp of the
    /// command it interrupted, `kick << 8` like the queue's stamp. Without a
    /// stamp match the oldest published command is marked.
    pub(crate) fn note_interrupted(&self, stamp: u32) -> recovery::Interrupted {
        let sequence = stamp >> 8;
        if let Some(active) = self.active.iter().find(|active| {
            active
                .ticket
                .is_some_and(|ticket| (ticket.kick as u32) & 0xff_ffff == sequence)
        }) {
            active.packet.completion.note_interrupted();
            return recovery::Interrupted::Exact;
        }
        match self.active.iter().find(|active| active.ticket.is_some()) {
            Some(active) => {
                active.packet.completion.note_interrupted();
                recovery::Interrupted::Oldest
            }
            None => recovery::Interrupted::None,
        }
    }
    pub(crate) fn classify(&self, class: &dyn Fn(&Arc<VmStatus>) -> recovery::Class) {
        for active in self.active.iter() {
            active
                .packet
                .completion
                .classify(class(active.packet.completion.status()));
        }
    }
    fn qos_owner(&self) -> qos::Owner {
        qos::Owner {
            qid: self.qid(),
            // Accounting completes only for bound owners; a detached queue has no
            // pending work and never reaches this.
            qos: self
                .binding
                .as_ref()
                .map_or(0, |binding| binding.context.qos_id()),
            data_master: DataMaster::Compute as u8,
        }
    }
    /// A completion hint can require a second visibility/recordless pass even
    /// when firmware has no more events to send. Do not poll idle or failed queues.
    pub(crate) fn retirement_polling(&self) -> bool {
        !self.quarantined && self.retire_pending
    }

    /// A release has no completion event: keep observing the control consumer
    /// until its witness lets the idle physical queue be re-armed.
    pub(crate) fn pool_release_pending(&self) -> bool {
        matches!(self.pool_phase, PoolPhase::ReleaseSent(_))
    }

    pub(crate) fn pool_release_stalled(&self, now: u64) -> bool {
        self.pool_release_pending()
            && crate::g17::timeout::background_stalled(self.release_sent_ns, now)
    }

    /// A closed owner can still retain its last commands or their completion
    /// stamps. Admission may wait for retirement, without treating that close
    /// as permission to reuse any firmware-owned backing.
    pub(crate) fn release_settling(&self) -> bool {
        // release_owner keeps the old owner while its last commands retire.
        // Waiting only once owner is None misses precisely that interval.
        release_pending(
            self.released,
            self.quarantined,
            self.active.len,
            self.retire_pending,
        )
    }

    fn selected(&self, masks: Option<[u64; 2]>) -> bool {
        self.retired_by_teardown
            || self.retire_pending
            || masks.is_none_or(|m| m[self.qid() as usize / 64] & (1 << (self.qid() % 64)) != 0)
    }
    /// Stage every unlatched exact identity before the device's single shared-ring scan,
    /// whichever queues the scan's batch retires.
    pub(crate) fn request_observations(&self, batch: &mut Batch) -> Result<bool> {
        if self.quarantined {
            return Ok(false);
        }
        batch.stage_frontier(self.qid(), self.kick.timestamp().get())?;
        let mut requested = false;
        let mut stamp = None;
        for active in self.active.iter() {
            let Some(ticket) = active.ticket else {
                break;
            };
            if active.observed.is_none() {
                let stamp = match stamp {
                    Some(stamp) => stamp,
                    None => *stamp.insert(self.graph.stamp()?),
                };
                batch.request(&ticket, retirement::stamp_covers(stamp, ticket.kick))?;
                requested = true;
            }
        }
        Ok(requested)
    }
    pub(crate) fn latch_observations(&mut self, batch: &mut Batch) -> Result {
        if self.quarantined {
            return Ok(());
        }
        for active in self.active.iter_mut() {
            let Some(ticket) = active.ticket else {
                break;
            };
            if active.observed.is_none() {
                active.observed = batch.take(&ticket)?;
                if active.observed.is_some() {
                    active.observed_at = Some(now_ns());
                }
            }
        }
        Ok(())
    }
    fn front_finished(&self) -> bool {
        self.active.front().is_some_and(|active| {
            active.observed.is_some()
                || active.ticket.is_some_and(|ticket| {
                    self.graph
                        .cursors()
                        .and_then(|c| c.consumed(ticket.item_producer, ticket.item_count))
                        .unwrap_or(false)
                })
        })
    }
    fn valid_ticket(&self, active: &Active, ticket: &Ticket) -> Result {
        if ticket.qid != self.qid()
            || ticket.descriptor == 0
            || ticket.item_count == 0
            || ticket.work_node != active.work.node_va()?
            || self.submitted == 0
            || self.owner != Some(ticket.owner)
            || active.packet.order.sequence != ticket.submission
            || active.packet.context.id() != ticket.context_id
            || self
                .binding
                .as_ref()
                .is_none_or(|binding| active.packet.context.id() != binding.context.id())
        {
            return Err(EIO);
        }
        Ok(())
    }

    /// Return VM identity for queue-local failure fanout. Deferred callbacks are collected under
    /// the mutex and invoked afterwards; failed packets stay retained until their witness.
    pub(crate) fn quarantine(
        &mut self,
        error: Error,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<Option<Arc<VmStatus>>> {
        if !self.quarantined {
            self.quarantined = true;
            self.quarantine_error = Some(error);
            self.failure_status_pending = true;
            self.retirement_proved = false;
            self.retired_by_teardown = false;
            let all_spared =
                self.active.len != 0 && self.active.iter().all(|a| a.packet.completion.spared());
            self.spared_quarantine = all_spared && error == ENODATA;
            self.spared_deferred = self.spared_quarantine;
        }
        let error = self.quarantine_error.ok_or(EIO)?;
        let mut terminal = None;
        for active in self.active.iter_mut() {
            if !self.spared_deferred && !active.failure_deferred {
                defer(Deferred::FailedOwned(
                    active.packet.completion.clone(),
                    error,
                ))?;
                active.failure_deferred = true;
            }
            if self.failure_status_pending
                && !active.packet.completion.spared()
                && terminal.is_none()
            {
                terminal = Some(active.packet.completion.status().clone());
            }
        }
        self.failure_status_pending = false;
        self.awaiting_witness =
            self.active.len != 0 && self.active.iter().all(|a| a.ticket.is_some());
        Ok(terminal)
    }

    /// Poll only the ordered prefix; the next pass may recheck a finished front on another QID's
    /// event. A final identity-less scan allows recordless retirement without the one-pass grace.
    pub(crate) fn retire(
        &mut self,
        masks: Option<[u64; 2]>,
        epoch: u64,
        recovery_state: u32,
        mut complete_qos: impl FnMut(qos::Owner) -> Result,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        if self.quarantined || !self.selected(masks) {
            return Ok(());
        }
        let finished_before = self.retire_pending;
        self.retire_pending = false;
        let mut oldest = true;
        while let Some(front) = self.active.front() {
            let mut restart = None;
            let result = (|| {
                let ticket = front.ticket.ok_or(EIO)?;
                self.valid_ticket(front, &ticket)?;
                if recovery_state != 0 {
                    return Err(EAGAIN);
                }
                let recordless = if epoch != ticket.epoch {
                    Recordless::CrossedRecovery
                } else if masks.is_none() || (finished_before && oldest) {
                    Recordless::Allowed
                } else {
                    Recordless::NotYet
                };
                let next_end = self
                    .active
                    .iter()
                    .skip(1)
                    .find_map(|a| a.observed.map(|o| o.timestamps[1]));
                let stamp = self.graph.stamp()?;
                let record_aged = !retirement::stamp_covers(stamp, ticket.kick)
                    && match retirement::stamp_wait(front.observed_at, now_ns(), || {
                        // A node that cannot be read keeps the bound from the record.
                        front.work.drained().unwrap_or(true)
                    }) {
                        retirement::StampWait::Waiting => false,
                        retirement::StampWait::Restart(now) => {
                            restart = Some(now);
                            false
                        }
                        retirement::StampWait::Aged => true,
                    };
                let timestamps = retirement::poll(
                    &ticket,
                    front.observed,
                    self.graph.cursors()?,
                    stamp,
                    recordless,
                    self.last_end,
                    next_end,
                    record_aged,
                )?;
                if timestamps.is_some() && !retirement::stamp_covers(stamp, ticket.kick) {
                    retirement::note_stampless(self.qid(), ticket.kick);
                }
                if timestamps.is_some()
                    && self.previous.as_ref().is_some_and(|p| {
                        p.publication != Some((front.packet.order.sequence, ticket.epoch))
                    })
                {
                    return Err(EIO);
                }
                Ok(timestamps)
            })();
            oldest = false;
            match result {
                Ok(Some(timestamps)) => {
                    let submitted = self.submitted.checked_sub(1).ok_or(EIO)?;
                    if front.qos_pending {
                        complete_qos(self.qos_owner())?;
                        self.active.iter_mut().next().ok_or(EIO)?.qos_pending = false;
                    }
                    let front = self.active.front().ok_or(EIO)?;
                    if let Some(ticket) = front.ticket {
                        crate::g17::recovery::note_retired();
                        crate::g17::trace::complete(
                            crate::g17::trace::COMPUTE,
                            ticket.qid,
                            ticket.ordinal,
                            &front.packet,
                            [timestamps[0], timestamps[1]],
                        );
                    }
                    self.retire_pending = true;
                    defer(Deferred::Retired(
                        front.packet.completion.clone(),
                        Ok([timestamps[0], timestamps[1], 0, 0]),
                    ))?;
                    self.retire_pending = false;
                    self.submitted = submitted;
                    let active = self.active.pop().ok_or(EIO)?;
                    if active.observed.is_some() {
                        self.last_end = self.last_end.max(timestamps[1]);
                    }
                    drop(self.previous.take());
                    self.retirement_ready = self.active.len == 0;
                    if self.released && self.retirement_ready {
                        self.owner = None;
                        self.released_at = now_ns();
                    }
                }
                Ok(None) | Err(EAGAIN) => {
                    if restart.is_some() {
                        self.active.iter_mut().next().ok_or(EIO)?.observed_at = restart;
                    }
                    self.retire_pending = self.front_finished();
                    break;
                }
                Err(error) => {
                    self.terminal = self.quarantine(error, defer)?.map(|status| (status, error));
                    return Ok(());
                }
            }
        }
        Ok(())
    }

    /// Exact terminal witness releases command pins, not installed graph ownership. Spared
    /// zero-duration work can return its queue to service after settling its accounting.
    pub(crate) fn release_witnessed(
        &mut self,
        recovery_state: u32,
        mut complete_qos: impl FnMut(qos::Owner) -> Result,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        self.release_witnessed_inner(recovery_state, false, &mut complete_qos, defer)
    }

    pub(crate) fn prepare_replays(
        &mut self,
        recovery_state: u32,
        complete_qos: impl FnMut(qos::Owner) -> Result,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        self.release_witnessed_inner(recovery_state, true, complete_qos, defer)
    }

    fn release_witnessed_inner(
        &mut self,
        recovery_state: u32,
        replay: bool,
        mut complete_qos: impl FnMut(qos::Owner) -> Result,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        if !self.awaiting_witness || recovery_state != 0 {
            return Ok(());
        }
        if !self.retirement_proved {
            for active in self.active.iter() {
                self.valid_ticket(active, &active.ticket.ok_or(EIO)?)?;
            }
            let newest = self.active.back().and_then(|a| a.ticket).ok_or(EIO)?;
            if !retirement::quarantined_retired(
                &newest,
                self.graph.cursors()?,
                self.witness_stamp()?,
                self.active.len as u32,
                self.submitted,
            )? {
                return Ok(());
            }
            let candidate = self.spared_deferred
                && self.spared_quarantine
                && !self.released
                && self.owner == Some(newest.owner)
                && !self
                    .active
                    .iter()
                    .any(|active| active.packet.completion.has_firmware_consumer())
                && self.active.iter().all(|active| {
                    active
                        .packet
                        .completion
                        .replay_witness_words()
                        .is_some_and(|words| words[0] == words[1])
                });
            if candidate && !replay {
                return Ok(());
            }
            self.replay_retirement = candidate;
            self.retirement_proved = true;
        }
        while let Some(front) = self.active.front() {
            if front.qos_pending {
                complete_qos(self.qos_owner())?;
                self.active.iter_mut().next().ok_or(EIO)?.qos_pending = false;
            }
            let front = self.active.front().ok_or(EIO)?;
            let completion = front.packet.completion.clone();
            if self.replay_retirement {
                self.replays.push(front.packet.clone()).map_err(|_| EIO)?;
            } else if self.spared_deferred {
                defer(Deferred::Retired(completion, Err(ENODATA)))?;
            } else {
                defer(Deferred::Release(completion))?;
            }
            drop(self.active.pop());
        }
        self.awaiting_witness = false;
        self.fault_stamp = None;
        self.retirement_proved = false;
        self.replay_retirement = false;
        self.spared_deferred = false;
        // The stamp and fully consumed ring proved the replacement retired.
        // Keeping its predecessor only on the fault path prevents recycling
        // forever: neither another retirement nor an exit kill can release it.
        Previous::retire_witnessed(&mut self.previous);
        if self.spared_quarantine {
            self.spared_quarantine = false;
            self.quarantine_error = None;
            self.submitted = 0;
            self.kick.clear_parent_after_recovery();
            self.quarantined = false;
            self.retire_pending = false;
            self.retirement_ready = true;
            if self.released {
                self.owner = None;
                self.released_at = now_ns();
            }
        }
        Ok(())
    }

    /// Both FIFOs are bounded by IN_FLIGHT. Count all retained commands of
    /// queues which this pass can release, replay or quarantine on failure.
    pub(crate) fn replay_commands(&self) -> usize {
        if self.awaiting_witness || self.replays.len != 0 {
            self.active.len + self.replays.len
        } else {
            0
        }
    }

    pub(crate) fn replay_front(&self) -> Option<Arc<Packet>> {
        self.replays.front().cloned()
    }

    /// Drop the witnessed owner only after either publication retained it or
    /// the off-lock completion collector accepted its terminal result.
    pub(crate) fn finish_replay(&mut self, packet: &Arc<Packet>) -> Result {
        if !self
            .replays
            .front()
            .is_some_and(|front| Arc::ptr_eq(front, packet))
        {
            return Err(EIO);
        }
        drop(self.replays.pop());
        self.replay_prepared = false;
        if self.replays.len == 0 {
            self.replay_error = None;
        }
        Ok(())
    }

    pub(crate) fn prepare_replay(&mut self, packet: &Arc<Packet>) -> Result {
        if !self
            .replays
            .front()
            .is_some_and(|front| Arc::ptr_eq(front, packet))
        {
            return Err(EIO);
        }
        if let Some(error) = self.replay_error {
            return Err(error);
        }
        if !self.replay_prepared {
            if !packet.completion.begin_replay() {
                return Err(EAGAIN);
            }
            packet.completion.reset_for_replay()?;
            self.replay_prepared = true;
        }
        Ok(())
    }

    pub(crate) fn fail_replays(&mut self, error: Error) {
        if self.replay_error.is_none() {
            self.replay_error = Some(error);
        }
    }

    /// Caller already stopped both processors. No firmware owner can access retained mappings.
    pub(crate) fn stopped(&mut self, defer: &mut impl FnMut(Deferred) -> Result) -> Result {
        while let Some(packet) = self.replays.front() {
            defer(Deferred::Retired(packet.completion.clone(), Err(EIO)))?;
            drop(self.replays.pop());
        }
        while let Some(active) = self.active.front() {
            defer(Deferred::Retired(
                active.packet.completion.clone(),
                Err(EIO),
            ))?;
            drop(self.active.pop());
        }
        Ok(())
    }
}

/// Validate the complete unpublished item budget before changing descriptors,
/// work counts or queue producers. Leave one item empty across ring wrap.
fn item_capacity(cursors: Cursors, entry_signal: bool) -> Result {
    if cursors.count != ITEM_RING_ENTRIES {
        return Err(EIO);
    }
    if cursors.producer >= cursors.count || cursors.consumer >= cursors.count {
        return Err(ERANGE);
    }
    let free = (cursors.consumer + cursors.count - cursors.producer - 1) % cursors.count;
    if free < if entry_signal { 4 } else { 3 } {
        let _ = crate::g17::capacity::refused(crate::g17::capacity::Site::ComputeItems);
        return Err(EAGAIN);
    }
    Ok(())
}

/// Retained commands are capacity, not a failed VM. Nothing has been written
/// when this check runs, so the outer publisher can wait for retirement under
/// its original admission deadline, just as it does for a full firmware ring.
fn retained_capacity(
    quarantined: bool,
    released: bool,
    active: usize,
    replays: usize,
    replay_front: bool,
) -> Result {
    if quarantined || released {
        return Err(EBUSY);
    }
    if active >= IN_FLIGHT || (replays != 0 && !replay_front) {
        return Err(EAGAIN);
    }
    Ok(())
}

/// Logical close promises a future healthy handoff, not immediate reuse.
/// Exact retirement still owns the transition.
fn release_pending(released: bool, quarantined: bool, active: usize, retiring: bool) -> bool {
    released && !quarantined && (active != 0 || retiring)
}

#[kunit_tests(asahi_g17_compute_admission)]
mod admission_tests {
    use super::*;

    #[test]
    fn complete_native_wait_list_fits_every_cached_slot() {
        let waits = crate::g17::kick::COMPUTE_CHANNELS;
        for slot in 0..RECORD_SLOTS {
            for index in 0..waits {
                let start = record_start(
                    slot,
                    NATIVE_WAIT_OFFSET + index * NATIVE_WAIT_STRIDE,
                    NATIVE_WAIT_STRIDE,
                    CACHED_RECORD_STRIDE,
                )
                .unwrap();
                assert!(start >= slot * CACHED_RECORD_STRIDE);
                assert!(start + NATIVE_WAIT_STRIDE <= (slot + 1) * CACHED_RECORD_STRIDE);
            }
        }
    }

    #[test]
    fn record_window_cannot_overwrite_the_next_slot() {
        // The nineteenth wait used to cross the old slot boundary silently.
        let offset = NATIVE_WAIT_OFFSET + 18 * NATIVE_WAIT_STRIDE;
        assert_eq!(
            record_start(0, offset, NATIVE_WAIT_STRIDE, RECORD_STRIDE),
            Err(EINVAL)
        );
        assert!(record_start(0, offset, NATIVE_WAIT_STRIDE, CACHED_RECORD_STRIDE).is_ok());
        assert_eq!(
            record_start(RECORD_SLOTS, 0, 1, CACHED_RECORD_STRIDE),
            Err(EINVAL)
        );
        assert_eq!(
            record_start(0, usize::MAX, 1, CACHED_RECORD_STRIDE),
            Err(EINVAL)
        );
        assert_eq!(
            record_start(0, CACHED_RECORD_STRIDE, 1, CACHED_RECORD_STRIDE),
            Err(EINVAL)
        );
        assert_eq!(
            record_start(0, CACHED_RECORD_STRIDE - 1, 1, CACHED_RECORD_STRIDE),
            Ok(CACHED_RECORD_STRIDE - 1)
        );
    }

    #[test]
    fn retained_ring_pressure_waits_without_discarding_work() {
        let mut active = Fifo::new();
        for sequence in 0..IN_FLIGHT {
            active.push(sequence).unwrap();
        }
        assert_eq!(
            retained_capacity(false, false, active.len, 0, false),
            Err(EAGAIN)
        );
        assert_eq!(active.front(), Some(&0));
        assert_eq!(active.len, IN_FLIGHT);
        assert_eq!(active.pop(), Some(0));
        assert_eq!(
            retained_capacity(false, false, active.len, 0, false),
            Ok(())
        );
        assert_eq!(active.front(), Some(&1));
    }

    #[test]
    fn replay_front_and_terminal_ownership_remain_distinct() {
        assert_eq!(retained_capacity(false, false, 0, 1, false), Err(EAGAIN));
        assert_eq!(retained_capacity(false, false, 0, 1, true), Ok(()));
        assert_eq!(
            retained_capacity(false, false, IN_FLIGHT, 1, true),
            Err(EAGAIN)
        );
        for (quarantined, released) in [(true, false), (false, true), (true, true)] {
            assert_eq!(
                retained_capacity(quarantined, released, 0, 1, true),
                Err(EBUSY)
            );
        }
    }

    #[test]
    fn item_refusal_is_retryable_and_preserves_the_empty_slot() {
        let n = ITEM_RING_ENTRIES;
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: 0,
                    consumer: 4,
                    count: n
                },
                false
            ),
            Ok(())
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: 0,
                    consumer: 4,
                    count: n
                },
                true
            ),
            Err(EAGAIN)
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: n - 2,
                    consumer: 2,
                    count: n
                },
                false
            ),
            Ok(())
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: n - 2,
                    consumer: 2,
                    count: n
                },
                true
            ),
            Err(EAGAIN)
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: 10,
                    consumer: 10,
                    count: n
                },
                true
            ),
            Ok(())
        );
    }

    #[test]
    fn bad_item_cursors_are_not_backpressure() {
        let n = ITEM_RING_ENTRIES;
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: 0,
                    consumer: 0,
                    count: 0
                },
                false
            ),
            Err(EIO)
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: n,
                    consumer: 0,
                    count: n
                },
                false
            ),
            Err(ERANGE)
        );
        assert_eq!(
            item_capacity(
                Cursors {
                    producer: 0,
                    consumer: n,
                    count: n
                },
                true
            ),
            Err(ERANGE)
        );
    }

    #[test]
    fn closed_owner_waits_for_its_last_command() {
        assert!(release_pending(true, false, 1, false));
        assert!(release_pending(true, false, 0, true));
        assert!(!release_pending(true, false, 0, false));
    }

    #[test]
    fn live_or_quarantined_owners_do_not_promise_capacity() {
        assert!(!release_pending(false, false, 1, true));
        assert!(!release_pending(true, true, 1, true));
    }
}

impl Queue {
    pub(crate) fn matches(&self, owner: u64, qid: u8, context: &Arc<Context>) -> Result<bool> {
        if self.owner != Some(owner) {
            return Ok(false);
        }
        if self.released
            || self.quarantined
            || self.qid() != qid
            || !self
                .binding
                .as_ref()
                .is_some_and(|binding| Arc::ptr_eq(&binding.context, context))
            || !context.is_current()
        {
            return Err(EIO);
        }
        Ok(true)
    }
    pub(crate) fn reusable(&self) -> bool {
        if self.growing {
            return false;
        }
        self.installed
            // A pool release in flight must be witnessed before any successor.
            && !matches!(self.pool_phase, PoolPhase::ReleaseSent(_))
            // A recycled killed queue is offered only once its backing is vacant (re-arm path).
            && (!self.needs_fresh_backing || self.pool_phase == PoolPhase::Vacant)
            // An issued kill still names this graph even if its commands retire
            // before the acknowledgement. It cannot acquire a successor owner.
            && self.exit.is_none()
            && self.owner.is_none()
            && self.released
            && self.retirement_ready
            && !self.quarantined
            && self.active.len == 0
            && self.replays.len == 0
            && self.previous.is_none()
            // Borrow acquisition is serialized by the device mutex. Avoid
            // touching the separate loan counter for an otherwise busy queue.
            && !self.source_hint.has_readers(&self.source_readers)
    }
    pub(crate) fn bind_owner(&mut self, owner: u64, context: Arc<Context>) -> Result {
        if !self.reusable() {
            return Err(EBUSY);
        }
        self.source_hint.reset_for_owner(&self.source_readers)?;
        if !context.is_current() {
            return Err(EFAULT);
        }
        let old = self.binding.as_ref().map(|binding| binding.context.clone());
        let same_vm = old
            .as_ref()
            .is_some_and(|old| Arc::ptr_eq(old.status(), context.status()) && old.is_current());
        if same_vm {
            let binding = self.binding.as_mut().ok_or(EIO)?;
            if binding.pool_alias.is_none() {
                // The same VM takes back a re-armed pool: alias the new backing.
                binding.pool_alias = Some(self.pool.map_client(context.vm())?);
            }
            if binding.descriptors.is_none() {
                binding.descriptors = Some(self.graph.descriptor_alias(context.vm())?);
            }
            self.graph.set_owner(&context)?;
            binding.context = context.clone();
            self.previous = Some(Previous {
                binding: None,
                context: old.ok_or(EIO)?,
                publication: None,
            });
        } else {
            let kick_alias = self.kick.map_client(context.vm())?;
            let pool_alias = self.pool.map_client(context.vm())?;
            let binding = self.graph.bind(context.clone(), kick_alias, pool_alias)?;
            self.graph.set_owner(&context)?;
            let old_binding = self.binding.replace(binding);
            // A detached queue (exited owner already dropped) has nothing to supersede;
            // the new owner's record fields replace the retained pages' references.
            self.previous = old.map(|old| Previous {
                binding: old_binding,
                context: old,
                publication: None,
            });
            self.retained_owner = None;
        }
        context.mark_published();
        context.note_compute_binding(true);
        crate::g17::trace::bind(
            crate::g17::trace::COMPUTE,
            self.qid(),
            context.id(),
            if same_vm { 2 } else { 1 },
        );
        self.owner = Some(owner);
        self.unpublished_binding = true;
        self.released = false;
        self.retirement_ready = false;
        self.retired_by_teardown = false;
        Ok(())
    }
    /// True allows the registry to cancel the never-published QID and drop this graph. An
    /// installed graph remains retained even after the logical owner exits without work.
    pub(crate) fn release_owner(&mut self) -> Result<bool> {
        if let Some(binding) = &self.binding {
            binding.context.note_compute_binding(false);
        }
        if !self.installed && self.active.len == 0 && !self.quarantined && self.previous.is_none() {
            return Ok(true);
        }
        self.released = true;
        if self.active.len == 0 && !self.quarantined {
            if let Some(previous) = self.previous.as_ref().filter(|p| p.publication.is_none()) {
                if let Err(error) = self.graph.set_owner(&previous.context) {
                    self.quarantined = true;
                    self.retired_by_teardown = true;
                    return Err(error);
                }
                let mut previous = self.previous.take().ok_or(EIO)?;
                if let Some(binding) = previous.binding.take() {
                    self.binding = Some(binding);
                } else {
                    self.binding.as_mut().ok_or(EIO)?.context = previous.context;
                }
                self.retirement_ready = true;
            }
            // Rebinding an already detached queue has no Previous. If its
            // first command is rejected or cancelled before publication, the
            // empty active FIFO is still the original idle witness. No future
            // completion can turn retirement_ready back on for this owner.
            if self.previous.is_none() {
                self.retirement_ready = true;
            }
            if self.retirement_ready {
                self.owner = None;
                self.released_at = now_ns();
            }
        }
        Ok(false)
    }
    /// The last owner's file and VM are gone: its execution root was released
    /// after the scheduler-state release and command retirement were witnessed
    /// (`Context::release_execution`), and this queue is idle without an owner.
    /// Drop the binding (every alias into that VM) and the context, keeping only
    /// the scheduler page and work storage the installed queue record names.
    /// The returned binding (aliases into the exited VM and the last reference
    /// to its context) is dropped by the caller after the device mutex is
    /// released: unmapping and VM teardown must not delay other clients.
    pub(crate) fn detach_exited_owner(&mut self) -> Option<Binding> {
        if !self.reusable() {
            return None;
        }
        if self.binding.as_ref()?.context.is_current() {
            return None;
        }
        let binding = self.binding.take()?;
        self.retained_owner = Some(binding.context.retain_firmware_pages());
        Some(binding)
    }
    /// An idle retained queue whose USC backing may be given back.
    pub(crate) fn vacate_candidate(&self) -> bool {
        self.pool_phase == PoolPhase::Populated
            && self.pool.populated()
            && (self.reusable() || self.recycled_pending())
            && !self.source_hint.has_readers(&self.source_readers)
    }
    /// A recycled killed queue still holding the backing its killed work used.
    pub(crate) fn recycled_pending(&self) -> bool {
        self.needs_fresh_backing
            && self.installed
            && self.exit.is_none()
            && self.owner.is_none()
            && !self.quarantined
            && self.active.len == 0
            && self.replays.len == 0
            && self.previous.is_none()
    }
    /// Recycled queues are vacated regardless of the warm cap.
    pub(crate) fn needs_fresh_backing(&self) -> bool {
        self.needs_fresh_backing
    }
    /// The firmware acknowledged the kill of this queue's closed owner (`ContextKilled`)
    /// and every killed command was settled: return the QID to service. The item
    /// ring is reset to empty at the firmware's consumer cursor; the USC pool and
    /// descriptor ring are vacated and re-armed fresh before the next owner binds.
    pub(crate) fn recycle_killed(&mut self) -> Result<bool> {
        if !self.installed
            || !self.exit.as_ref().is_some_and(|exit| exit.released())
            || self.active.len != 0
            || self.replays.len != 0
            || self.previous.is_some()
        {
            return Ok(false);
        }
        self.recycle_settled()?;
        Ok(true)
    }
    /// A quarantined queue whose owner closed and whose every retained command was
    /// already settled, by the exact retirement witness or by an acknowledged kill,
    /// holds no pending command: the firmware consumed its item ring exactly as after
    /// a kill. Neither `reusable` nor `exit_candidate` matches it, so without this it
    /// keeps its QID and USC pool backing until processor stop, and repeated client
    /// failures exhaust the compute pool VA range. Return it to service as
    /// `recycle_killed` does.
    pub(crate) fn recycle_quarantined_idle(&mut self) -> Result<bool> {
        if !self.installed
            || !self.quarantined
            || !self.released
            || self.growing
            || self.exit.is_some()
            || self.awaiting_witness
            || self.active.len != 0
            || self.replays.len != 0
            || self.previous.is_some()
        {
            return Ok(false);
        }
        self.recycle_settled()?;
        Ok(true)
    }
    fn recycle_settled(&mut self) -> Result {
        self.graph.reset_items()?;
        self.config_cache = None;
        self.exit = None;
        self.quarantined = false;
        self.quarantine_error = None;
        self.failure_status_pending = false;
        self.retired_by_teardown = false;
        self.spared_quarantine = false;
        self.spared_deferred = false;
        self.retire_pending = false;
        self.awaiting_witness = false;
        self.fault_stamp = None;
        self.retirement_proved = false;
        self.replay_retirement = false;
        self.replay_error = None;
        self.replay_prepared = false;
        self.submitted = 0;
        self.kick.clear_parent_after_recovery();
        self.owner = None;
        self.released = true;
        self.retirement_ready = true;
        self.released_at = now_ns();
        self.needs_fresh_backing = true;
        Ok(())
    }
    pub(crate) fn released_at(&self) -> u64 {
        self.released_at
    }
    pub(crate) fn pool_vacant(&self) -> bool {
        self.pool_phase == PoolPhase::Vacant
    }
    /// Publishes the release of this idle queue's free list. The caller holds the
    /// device mutex and rings the control doorbell afterwards.
    pub(crate) fn vacate(&mut self, init: &crate::g17::initdata::InitData) -> Result {
        if !self.vacate_candidate() {
            return Err(EBUSY);
        }
        let record =
            ControlRecord::FreeListRelease(self.pool.release_record(self.free_list_generation));
        let cursor = Rings::publish_control(init, &record)?;
        self.pool_phase = PoolPhase::ReleaseSent(cursor);
        self.release_sent_ns = Some(crate::g17::now_ns());
        fence(Ordering::SeqCst);
        Ok(())
    }
    /// Detaches the USC backing once the firmware's control consumer passed the
    /// release. The caller frees the returned backing after releasing the
    /// device mutex.
    pub(crate) fn observe_vacancy(
        &mut self,
        consumer: u32,
        producer: u32,
    ) -> Result<Option<Vacated>> {
        if self.growing {
            return Ok(None);
        }
        let PoolPhase::ReleaseSent(cursor) = self.pool_phase else {
            return Ok(None);
        };
        if !crate::g17::freelist::control_consumed(consumer, producer, cursor)? {
            return Ok(None);
        }
        // The retained owner binding's client alias pins the backing: detach it
        // first (the owner has exited; `reusable()` proved no previous binding).
        let alias = self
            .binding
            .as_mut()
            .and_then(|binding| binding.pool_alias.take());
        let pages = self.pool.take_pages();
        let descriptors = self
            .binding
            .as_mut()
            .and_then(|binding| binding.descriptors.take());
        let ring = self.graph.vacate_ring();
        self.pool_phase = PoolPhase::Vacant;
        self.release_sent_ns = None;
        Ok(Some(Vacated {
            _alias: alias,
            _pages: pages,
            _descriptors: descriptors,
            _ring: ring,
        }))
    }
    /// Installs fresh backing into a vacant pool; returns the descriptor row data
    /// the caller writes before binding an owner.
    pub(crate) fn rearm(
        &mut self,
        pages: crate::g17::freelist::Pages,
        ring: DescriptorRing,
    ) -> Result<(u16, u64)> {
        if !self.can_rearm() {
            return Err(EAGAIN);
        }
        self.graph.rearm_ring(ring)?;
        self.pool.rearm(pages)?;
        self.pool_phase = PoolPhase::Populated;
        self.release_sent_ns = None;
        self.needs_fresh_backing = false;
        Ok((self.pool.id(), self.pool.page_list_va()?))
    }
    pub(crate) fn pool_fits(&self, required: usize) -> bool {
        !self.growing && self.pool_phase == PoolPhase::Populated && self.pool.fits(required)
    }

    /// Only a capacity increase waits for this queue's previously published work.
    /// Other queues and ordinary submissions keep their existing concurrency.
    pub(crate) fn begin_growth(
        &mut self,
        init: &crate::g17::initdata::InitData,
        blocks: usize,
    ) -> Result<bool> {
        if self.growing {
            return Err(EAGAIN);
        }
        if self.quarantined
            || self.released
            || self.exit.is_some()
            || self.active.len != 0
            || self.replays.len != 0
            || self.previous.is_some()
            || (self.installed && !self.retirement_ready)
            || self.pool_phase != PoolPhase::Populated
        {
            return Err(EAGAIN);
        }
        if self.installed {
            let record =
                ControlRecord::FreeListRelease(self.pool.release_record(self.free_list_generation));
            let cursor = Rings::publish_control(init, &record)?;
            self.pool_phase = PoolPhase::ReleaseSent(cursor);
            self.release_sent_ns = Some(crate::g17::now_ns());
        } else {
            self.pool_phase = PoolPhase::Vacant;
            self.release_sent_ns = None;
        }
        self.growing = true;
        self.growth_blocks = blocks;
        Ok(self.installed)
    }

    pub(crate) fn finish_growth(
        &mut self,
        init: &crate::g17::initdata::InitData,
        pages: &mut Option<crate::g17::freelist::Pages>,
        alias: &mut Option<mmu::KernelMapping>,
        blocks: usize,
    ) -> Result<Vacated> {
        if !self.growing || blocks < self.growth_blocks {
            return Err(EAGAIN);
        }
        if let PoolPhase::ReleaseSent(cursor) = self.pool_phase {
            let consumer = init.control_consumer()?.load(Ordering::Acquire);
            let producer = init.control_producer()?.load(Ordering::Acquire);
            if !crate::g17::freelist::control_consumed(consumer, producer, cursor)? {
                return Err(EAGAIN);
            }
        }
        if alias.is_none() {
            return Err(EINVAL);
        }
        let fresh = pages.as_ref().ok_or(EINVAL)?;
        let binding = self.binding.as_mut().ok_or(EFAULT)?;
        // No ownership moves before every fallible descriptor/header operation.
        // The old release is consumed; no queue can use this row until commit.
        init.initialize_compute_pool_blocks(self.pool.id(), fresh.page_list_va(), blocks)?;
        self.pool.replace_pages(pages)?;
        let old_alias = core::mem::replace(&mut binding.pool_alias, alias.take());
        self.pool_phase = PoolPhase::Populated;
        self.release_sent_ns = None;
        self.growing = false;
        self.growth_blocks = 0;
        Ok(Vacated {
            _alias: old_alias,
            _pages: pages.take(),
            _descriptors: None,
            _ring: None,
        })
    }

    pub(crate) fn growth_token(&self) -> Option<Growth> {
        self.growing.then_some(Growth {
            qid: self.qid(),
            phase: self.pool_phase,
        })
    }

    /// Transfer an abandoned builder's release back to the event worker. Do not
    /// revoke the published release or free backing here: vacancy still requires
    /// its exact control-consumer witness. A different transaction is untouched.
    pub(crate) fn abort_growth(&mut self, token: Growth) -> Result<Option<bool>> {
        if !self.growing || self.qid() != token.qid || self.pool_phase != token.phase {
            return Ok(None);
        }
        self.growing = false;
        self.growth_blocks = 0;
        self.release_owner().map(Some)
    }

    /// Exact host identity remains valid even when several commands share an ioctl ID.
    pub(crate) fn owns_packet(&self, packet: &Arc<Packet>) -> bool {
        self.active
            .iter()
            .any(|active| Arc::ptr_eq(&active.packet, packet))
    }

    /// A watchdog settles spared, deferred waiters without changing their VM status.
    /// Fully ticketed spared work can still return this queue after its drain witness.
    pub(crate) fn timeout(
        &mut self,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<Option<Arc<VmStatus>>> {
        let fresh = !self.quarantined;
        let return_spared = self.active.len != 0
            && self
                .active
                .iter()
                .all(|active| active.packet.completion.spared() && active.ticket.is_some());
        if self.spared_deferred {
            for active in self.active.iter_mut() {
                if !active.failure_deferred {
                    defer(Deferred::FailedOwned(
                        active.packet.completion.clone(),
                        ETIMEDOUT,
                    ))?;
                    active.failure_deferred = true;
                }
            }
            self.spared_deferred = false;
        }
        let result = self.quarantine(ETIMEDOUT, defer);
        if fresh {
            self.spared_quarantine = return_spared;
        }
        result
    }

    pub(crate) fn cancel(
        &mut self,
        packet: &Arc<Packet>,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        if !self.owns_packet(packet) {
            return Ok(false);
        }
        if self.quarantined {
            if self.spared_deferred {
                for active in self.active.iter_mut() {
                    if !active.failure_deferred {
                        defer(Deferred::FailedOwned(
                            active.packet.completion.clone(),
                            ECANCELED,
                        ))?;
                        active.failure_deferred = true;
                    }
                }
                self.spared_deferred = false;
            }
            return Ok(true);
        }
        self.released = true;
        self.retired_by_teardown = true;
        if let Some(active) = self
            .active
            .iter_mut()
            .find(|a| Arc::ptr_eq(&a.packet, packet))
        {
            if !active.failure_deferred {
                defer(Deferred::FailedOwned(
                    active.packet.completion.clone(),
                    ECANCELED,
                ))?;
                active.failure_deferred = true;
            }
        }
        Ok(true)
    }
    /// Firmware is halted and publication is closed while its target is copied.
    pub(crate) fn publish_recovery_target(&self) -> Result {
        let Some(target) = self.graph.pointers.recovery_target else {
            return Ok(());
        };
        let producer = self
            .graph
            .graph
            .word(CURSORS + self.graph.pointers.producer)?
            .load(Ordering::Relaxed);
        self.graph
            .graph
            .word(CURSORS + target)?
            .store(producer, Ordering::Relaxed);
        Ok(())
    }

    /// The newest covered kick of a QID subsumes its earlier prefix; later independent work
    /// must not become a dependency. Errors are observed before returning any firmware token.
    pub(crate) fn dependency(
        &self,
        owner: u64,
        status: &Arc<VmStatus>,
        through: Option<u64>,
        recovery_state: u32,
    ) -> Result<Option<crate::g17::fw::kick::KickDependency>> {
        if through.is_none() || self.owner != Some(owner) {
            return Ok(None);
        }
        let Some(active) = self
            .active
            .iter()
            .filter(|a| crate::g17::job::Order::contains(through, a.packet.order.sequence))
            .last()
        else {
            return Ok(None);
        };
        if self.quarantined && self.spared_quarantine {
            return Err(ECANCELED);
        }
        if !Arc::ptr_eq(active.packet.completion.status(), status)
            || self.quarantined
            || recovery_state != 0
        {
            return Err(EIO);
        }
        let ticket = active.ticket.ok_or(EIO)?;
        let state = crate::g17::job::fence_status(&active.packet.completion.fence());
        if state < 0 {
            return Err(crate::g17::dependency::inherited_error(Error::from_errno(
                state,
            )));
        }
        if state > 0 {
            return Ok(None);
        }
        active.packet.completion.note_firmware_consumer();
        let kick = crate::g17::fw::kick::KickTimestamp::new(ticket.kick).ok_or(EIO)?;
        Ok(Some(
            crate::g17::fw::kick::KickDependency::new(self.qid(), kick).ok_or(EIO)?,
        ))
    }
}

/// Validate the individual slot before deriving an offset into its allocation.
/// Allocation-wide checks cannot detect a write into the following live slot.
fn record_start(slot: usize, offset: usize, window: usize, stride: usize) -> Result<usize> {
    if slot >= RECORD_SLOTS || offset > stride || window > stride - offset {
        return Err(EINVAL);
    }
    slot.checked_mul(stride)
        .and_then(|start| start.checked_add(offset))
        .ok_or(EINVAL)
}

impl Graph {
    fn record<T: Copy>(
        &mut self,
        slot: Option<usize>,
        offset: usize,
        window: usize,
        value: T,
    ) -> Result<u64> {
        let (object, start) = if let Some(slot) = slot {
            let (object, stride): (&mut KernelObject, usize) = if offset == ANNOUNCE {
                (&mut self.records, RECORD_STRIDE)
            } else {
                (&mut self.cached_records, CACHED_RECORD_STRIDE)
            };
            (object, record_start(slot, offset, window, stride)?)
        } else {
            let start = record_start(0, offset, window, GRAPH_SIZE - 0x6800)?;
            (&mut *self.graph, 0x6800 + start)
        };
        if core::mem::size_of::<T>() > window {
            return Err(EINVAL);
        }
        let pointer = object.pointer(start, window)?;
        // SAFETY: The checked record window is not named by any unconsumed item pointer. The
        // device mutex excludes other host publishers, and no Rust reference to it is formed.
        unsafe { pointer.write_bytes(0, window) };
        object.write(start, value)?;
        fence(Ordering::SeqCst);
        Ok(object.gpu_va() + start as u64)
    }
    fn append(&self, expected: u32, address: u64, before_publish: impl FnOnce()) -> Result<u32> {
        let cursors = self.cursors()?;
        if cursors.producer != expected
            || cursors.count != ITEM_RING_ENTRIES
            || expected >= ITEM_RING_ENTRIES
        {
            return Err(EBUSY);
        }
        let next = (expected + 1) % ITEM_RING_ENTRIES;
        if next == cursors.consumer {
            return Err(EBUSY);
        }
        let slot = self.graph.dword(ITEMS + expected as usize * 8)?;
        let producer = self.graph.word(CURSORS + self.pointers.producer)?;
        if slot.load(Ordering::Relaxed) != 0 {
            return Err(EBUSY);
        }
        before_publish();
        slot.store(address, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        producer.store(next, Ordering::Relaxed);
        Ok(next)
    }
}

/// Borrowed device operations used during one publication. Implementations borrow the initdata,
/// coprocessor and accounting fields separately from the Registry that owns physical queues.
pub(crate) trait Host {
    fn next_compute_publication(&mut self) -> Result<u64>;
    fn epoch(&self) -> Result<(u64, u32)>;
    fn prepare_compute_shared(&mut self) -> Result;
    fn qos_publish(
        &mut self,
        owner: qos::Owner,
        scheduler: u64,
        policy: crate::g17::fw::queue::Policy,
    ) -> Result<qos::Publication>;
    fn qos_cancel(&mut self, publication: qos::Publication) -> Result;
    fn publish_qid(&mut self, id: kick::Id) -> Result;
    fn publish_outer(&mut self, priority: u8, slot: &crate::g17::fw::channels::WorkSlot) -> Result;
    fn notify(&mut self, message: u64) -> Result;
    fn note_submission(&mut self) -> Result;
}

impl Queue {
    pub(crate) fn spared_pending(&self) -> bool {
        self.quarantined && self.spared_quarantine && self.awaiting_witness
    }

    /// Retains the packet before any potentially visible store. An error before publication
    /// rolls back its work count; every ambiguous error quarantines the graph and packet.
    pub(crate) fn publish(
        &mut self,
        packet: Arc<Packet>,
        dependencies: &[crate::g17::fw::kick::KickDependency],
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        self.publish_with_native_waits(packet, dependencies, &[], host, defer)
    }

    pub(in crate::g17) fn publish_with_native_waits(
        &mut self,
        packet: Arc<Packet>,
        dependencies: &[crate::g17::fw::kick::KickDependency],
        native: &[crate::g17::sync_input::NativeWait],
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        use crate::g17::fw::{
            channels::WorkSlot,
            compute::{ComputeArgs, ComputeDescriptor, COMPUTE_KICK_EVENT_MASK, COMPUTE_QOS_CLASS},
            kick::{KickArgs, KickQos},
        };
        let (epoch, addresses) = self.publication_preflight(&packet, host)?;
        let crate::g17::command::Validated::Compute {
            intersection,
            scratch,
            usc_base,
            cdm_va,
            cdm_end_va,
            sampler_heap_va,
            sampler_count,
            ..
        } = &packet.command
        else {
            return Err(EINVAL);
        };
        let work = packet.completion.work_state()?;
        let work_word = work.word();
        if work_word == u32::MAX {
            return Err(EINVAL);
        }
        let work_node = work.node_va()?;
        let timestamp_va = packet.completion.compute_timestamp_va()?;
        let kick_timestamp = self.kick.timestamp();
        let kick_parent = self.kick.parent();
        let qid = self.qid();
        let ordinal = self.ordinal.checked_add(1).ok_or(EOVERFLOW)?;
        let (descriptor_high, descriptor_low) =
            self.graph.descriptor_vas(kick_timestamp.slot() as u8)?;
        let first = !self.installed && self.ordinal == 0;
        // The first prototype only waits on an installed consumer channel.
        // A retry without a published source falls back to completion on CPU.
        if first && !native.is_empty() {
            return Err(EAGAIN);
        }
        if native.len() > crate::g17::kick::COMPUTE_CHANNELS {
            return Err(EINVAL);
        }
        let entry_signal = first
            || dependencies
                .iter()
                .any(|dependency| dependency.qid() != qid);
        item_capacity(self.graph.cursors()?, entry_signal)?;
        if !native.is_empty() {
            let cursors = self.graph.cursors()?;
            let free = (cursors.consumer + cursors.count - cursors.producer - 1) % cursors.count;
            if free < (if entry_signal { 4 } else { 3 }) + native.len() as u32 {
                return Err(EAGAIN);
            }
        }
        let record_slot = (!first).then_some(self.ordinal as usize % RECORD_SLOTS);
        if let Some(slot) = record_slot {
            self.graph.ensure_record_slot(slot)?;
        }
        let args = ComputeArgs {
            intersection: *intersection,
            scratch: *scratch,
            usc_base: *usc_base,
            cdm_va: *cdm_va,
            cdm_end_va: *cdm_end_va,
            sampler_heap_va: *sampler_heap_va,
            sampler_count: *sampler_count,
            descriptor_va: descriptor_low,
            context_id: packet.context.id().try_into()?,
            context_generation: packet.context.generation(),
            work_state: work_word,
            work_state_va: work_node,
            qid,
            kick_count: self.ordinal,
            kick: kick_timestamp,
            preempt_va: addresses.preempt,
            operand_state_va: addresses.operand,
            operand: self.graph.operand_state,
            usage_va: addresses.usage,
            usage_fw_va: self.graph.usage.gpu_va(),
            free_list_slot: u32::from(self.pool.id()),
            free_list_control_va: self.pool.control_va(),
            stamp_va: self.graph.support.gpu_va() + STAMP as u64,
            aux_stamp_va: self.graph.support.gpu_va() + STAMP as u64 + 8,
            status_va: [
                self.graph.support.gpu_va() + 0x310,
                self.graph.support.gpu_va() + 0x318,
            ],
            timestamp_va,
        };
        let identity = ConfigIdentity {
            scheduler: packet.context.scheduler_va(),
            qos: packet.context.qos_id(),
            context: packet.context.id(),
        };
        self.free_list_generation = u64::from(packet.context.scheduler_generation());
        let policy = packet.context.policy();
        let ring = 1u8 << policy.priority();
        let config = self.queue_config(&identity, first, args.context_id, policy, &packet)?;
        let cached = self
            .config_cache
            .and_then(|(config, at)| (at == epoch).then_some(config));
        let send_config = first || super::render::publication::required(cached, config);
        let announce =
            KickAnnounce::new(qid, DataMaster::Compute, kick_timestamp, policy.priority())?;
        let predecessor = KickPredecessor::new(qid, kick_parent)?;
        let kick_args = KickArgs {
            qid,
            timestamp: kick_timestamp,
            parent: kick_parent,
            barriers: dependencies,
            descriptor_va: descriptor_high,
            queue_va: self.graph.graph.gpu_va(),
            qos: KickQos {
                slot: identity.qos,
                class: COMPUTE_QOS_CLASS,
            },
            mcache: None,
            event_mask: COMPUTE_KICK_EVENT_MASK,
            register_arrays: ComputeDescriptor::register_bindings(descriptor_low, *scratch)?,
            scs_enabled: scratch.enabled(),
            priority: policy.priority(),
        };
        let published = host.next_compute_publication()?;
        self.last_published = published;
        self.active
            .push(Active {
                packet: packet.clone(),
                published,
                work: work.clone(),
                ticket: None,
                observed: None,
                observed_at: None,
                failure_deferred: false,
                qos_pending: false,
            })
            .map_err(|_| EBUSY)?;
        self.retirement_ready = false;
        let mut visible = false;
        let mut accounted = false;
        let result = (|| -> Result {
            work.add_submitted_kicks(1)?;
            accounted = true;
            self.graph
                .ring
                .as_mut()
                .ok_or(EAGAIN)?
                .object
                .initialize::<ComputeDescriptor>(
                    kick_timestamp.slot() % DESCRIPTORS * COMPUTE_DESCRIPTOR_SIZE,
                    |descriptor| descriptor.write_zeroed(&args),
                )?;
            if first {
                self.graph.context_high.write(
                    0x200,
                    crate::g17::fw::compute::QueueContextItem::new(
                        descriptor_high,
                        self.graph.graph.gpu_va(),
                        qid,
                    ),
                )?;
            } else {
                // The retained context item remains at index one; only its descriptor changes.
                self.graph.context_high.write(0x210, descriptor_high)?;
                let cursors = self.graph.cursors()?;
                let _ = self.graph.recycle(cursors);
            }
            let config_va = if send_config {
                Some(self.graph.record(record_slot, CONFIG, 0xc0, config)?)
            } else {
                None
            };
            let cursors = self.graph.cursors()?;
            let mut wait_vas = KVec::with_capacity(native.len(), GFP_KERNEL)?;
            for (index, wait) in native.iter().enumerate() {
                let record = wait.bytes(
                    crate::g17::fw::stamp_value(kick_timestamp.get() as u32),
                    packet.order.sequence,
                );
                let address = self.graph.record(
                    record_slot,
                    NATIVE_WAIT_OFFSET + index * NATIVE_WAIT_STRIDE,
                    NATIVE_WAIT_STRIDE,
                    record,
                )?;
                wait_vas.push(address, GFP_KERNEL)?;
            }
            let prepared_kick = self.kick.prepare_entry(&kick_args)?;
            let mut producer;
            if first {
                producer = self.graph.first_descriptor(descriptor_high)?;
                host.prepare_compute_shared()?;
                packet.mark_published();
                visible = true;
                self.unpublished_binding = false;
                let qos = host.qos_publish(
                    qos::Owner {
                        qid,
                        qos: identity.qos,
                        data_master: 2,
                    },
                    identity.scheduler,
                    policy,
                )?;
                self.active.back_mut().ok_or(EIO)?.qos_pending = true;
                let target = self.pool.reserve_submission().or_else(|error| {
                    host.qos_cancel(qos)?;
                    self.active.back_mut().ok_or(EIO)?.qos_pending = false;
                    Err(error)
                })?;
                producer = self
                    .graph
                    .append(producer, config_va.ok_or(EINVAL)?, || {})
                    .or_else(|error| {
                        self.pool.cancel_unpublished(target)?;
                        host.qos_cancel(qos)?;
                        self.active.back_mut().ok_or(EIO)?.qos_pending = false;
                        Err(error)
                    })?;
            } else {
                let target = self.pool.reserve_submission()?;
                producer = (|| -> Result<u32> {
                    let mut before_descriptor = cursors.producer;
                    for &wait in &wait_vas {
                        before_descriptor = self.graph.append(before_descriptor, wait, || {
                            packet.mark_published();
                            visible = true;
                        })?;
                    }
                    self.graph.append(before_descriptor, descriptor_high, || {
                        packet.mark_published();
                        visible = true;
                    })
                })()
                .or_else(|error| {
                    if !visible {
                        self.pool.cancel_unpublished(target)?;
                    }
                    Err(error)
                })?;
                visible = true;
                self.unpublished_binding = false;
                let qos = host.qos_publish(
                    qos::Owner {
                        qid,
                        qos: identity.qos,
                        data_master: 2,
                    },
                    identity.scheduler,
                    policy,
                )?;
                self.active.back_mut().ok_or(EIO)?.qos_pending = true;
                if let Some(config_va) = config_va {
                    producer = self
                        .graph
                        .append(producer, config_va, || {})
                        .or_else(|error| {
                            host.qos_cancel(qos)?;
                            self.active.back_mut().ok_or(EIO)?.qos_pending = false;
                            Err(error)
                        })?;
                }
            }
            if first {
                prepared_kick.prepare_compute_install()?;
            }
            prepared_kick.commit();
            self.submitted = self.submitted.checked_add(1).ok_or(EOVERFLOW)?;
            let announce_va = self.graph.record(record_slot, ANNOUNCE, 0x40, announce)?;
            producer = self.graph.append(producer, announce_va, || {})?;
            fence(Ordering::SeqCst);
            if entry_signal {
                let predecessor_va =
                    self.graph
                        .record(record_slot, PREDECESSOR, 0x400, predecessor)?;
                producer = self.graph.append(producer, predecessor_va, || {})?;
            }
            if let Some(slot) = record_slot {
                self.graph.record_ends[slot] = Some(producer);
            }
            host.publish_qid(self.kick.id())?;
            host.publish_outer(
                policy.priority(),
                &WorkSlot::new(
                    DataMaster::Compute,
                    self.graph.graph.gpu_va(),
                    qid,
                    producer.try_into()?,
                    self.outer_started & ring == 0,
                )?,
            )?;
            self.outer_started |= ring;
            self.installed = true;
            self.published_config = Some(identity);
            if send_config {
                self.config_cache = Some((config, epoch));
            }
            self.ordinal = ordinal;
            let (accepted, state) = host.epoch()?;
            if state != 0 || accepted != epoch {
                return Err(EIO);
            }
            let ticket = Ticket {
                owner: self.owner.ok_or(EIO)?,
                submission: packet.order.sequence,
                context_id: packet.context.id(),
                qid,
                descriptor: descriptor_high,
                work_node,
                kick: kick_timestamp.get(),
                ordinal,
                item_producer: producer,
                item_count: ITEM_RING_ENTRIES,
                epoch,
            };
            self.active.back_mut().ok_or(EIO)?.ticket = Some(ticket);
            if let Some(previous) = self.previous.as_mut() {
                if previous.publication.is_none() {
                    previous.publication = Some((packet.order.sequence, epoch));
                }
            }
            // Every publication rings its own pipe and priority class, as render
            // publication does: pipe 2 (compute) in bits 1:0, priority class in
            // bits 3:2. The generic work doorbell left the firmware to find the
            // ring itself and delayed dispatch.
            fence(Ordering::SeqCst);
            host.notify((0x83 << 48) | 0x02 | u64::from(policy.priority()) << 2)?;
            crate::g17::trace::publish(crate::g17::trace::COMPUTE, qid, ordinal, &packet);
            host.note_submission()
        })();
        if let Err(error) = result {
            if !visible {
                if accounted {
                    if let Err(error) = work.cancel_unpublished_kicks(1) {
                        self.terminal =
                            self.quarantine(error, defer)?.map(|status| (status, error));
                        return Err(error);
                    }
                }
                drop(self.active.pop_back());
                return Err(error);
            }
            self.terminal = self.quarantine(error, defer)?.map(|status| (status, error));
            return Err(error);
        }
        Ok(())
    }

    /// Everything that must hold before a publication changes any state: a healthy device
    /// epoch, room in the active ring (or this packet at the replay front), the bound
    /// owner's current context and a clean completion status. Returns the epoch and the
    /// bound owner's per-queue addresses.
    fn publication_preflight(
        &self,
        packet: &Arc<Packet>,
        host: &mut impl Host,
    ) -> Result<(u64, BindingAddresses)> {
        let (epoch, state) = host.epoch()?;
        if state != 0 {
            return Err(EBUSY);
        }
        let replay_front = self
            .replays
            .front()
            .is_some_and(|front| Arc::ptr_eq(front, packet));
        retained_capacity(
            self.quarantined,
            self.released,
            self.active.len,
            self.replays.len,
            replay_front,
        )?;
        if !packet.context.is_current()
            || !self
                .binding
                .as_ref()
                .is_some_and(|binding| Arc::ptr_eq(&packet.context, &binding.context))
        {
            return Err(EFAULT);
        }
        let binding = self.binding.as_ref().ok_or(EFAULT)?;
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        Ok((
            epoch,
            BindingAddresses {
                preempt: binding.preempt.iova(),
                operand: binding.operand.iova(),
                usage: binding.usage.iova(),
            },
        ))
    }

    /// The queue configuration record for this publication: a fresh install on the first
    /// one, and QoS/context updates only when the identity changed since the last publication.
    fn queue_config(
        &self,
        identity: &ConfigIdentity,
        first: bool,
        context_id: u16,
        policy: crate::g17::fw::queue::Policy,
        packet: &Arc<Packet>,
    ) -> Result<QueueConfig> {
        QueueConfig::new(&QueueConfigArgs {
            target: QueueConfigTarget::Compute,
            kick_ring_va: self.kick.low_va(),
            kick_ring_fw_va: self.kick.firmware_va(),
            qid: self.qid(),
            install: first,
            completion_seed: None,
            context_id,
            free_list: FreeListBinding {
                va: self.pool.control_va(),
                generation: u64::from(packet.context.scheduler_generation()),
                slot: u32::from(self.pool.id()),
            },
            scheduler_va: identity.scheduler,
            policy,
            qos_slot: identity.qos,
            qos_update: self
                .published_config
                .is_none_or(|p| p.scheduler != identity.scheduler || p.qos != identity.qos),
            owner_pid: packet.context.owner_pid(),
            context_update: self
                .published_config
                .is_none_or(|p| p.scheduler != identity.scheduler || p.context != identity.context),
        })
    }
}

impl Graph {
    /// The queue has never been installed. Retry staging replaces its initial three item slots;
    /// no firmware reader can name this graph until its first complete outer publication.
    fn first_descriptor(&self, address: u64) -> Result<u32> {
        let first = self.graph.dword(ITEMS)?;
        let second = self.graph.dword(ITEMS + 8)?;
        let third = self.graph.dword(ITEMS + 16)?;
        let producer = self.graph.word(CURSORS + self.pointers.producer)?;
        first.store(address, Ordering::Relaxed);
        second.store(0, Ordering::Relaxed);
        third.store(0, Ordering::Relaxed);
        producer.store(1, Ordering::Relaxed);
        Ok(1)
    }
}
impl Queue {
    pub(crate) fn take_terminal(&mut self) -> Option<(Arc<VmStatus>, Error)> {
        self.terminal.take()
    }
}

impl Queue {
    pub(crate) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        write!(f, "compute_queue qid={} owned={} released={} reusable={} quarantined={} awaiting_witness={} previous={} exit={} growing={} pool={:?} active={} total={} ",
               self.qid(), u8::from(self.owner.is_some()), u8::from(self.released), u8::from(self.reusable()),
               u8::from(self.quarantined), u8::from(self.awaiting_witness), u8::from(self.previous.is_some()),
               u8::from(self.exit.is_some()), u8::from(self.growing), self.pool_phase, self.active.len, IN_FLIGHT)?;
        match self.graph.cursors() {
            Ok(cursors) if cursors.count != 0 => writeln!(
                f,
                "items_used={} items_total={}",
                cursors.producer.wrapping_sub(cursors.consumer) % cursors.count,
                cursors.count - 1
            ),
            _ => writeln!(f, "items_unavailable=1"),
        }?;
        writeln!(
            f,
            "compute_settling qid={} release={} pool_release={}",
            self.qid(),
            u8::from(self.release_settling()),
            u8::from(self.pool_release_pending())
        )?;
        if let Some(binding) = self.binding.as_ref() {
            binding
                .context
                .write_capacity(f, "compute", usize::from(self.qid()))?;
        }
        if let Some(previous) = self.previous.as_ref() {
            previous
                .context
                .write_capacity(f, "compute_previous", usize::from(self.qid()))?;
        }
        Ok(())
    }
}

#[kunit_tests(asahi_g17_fault_ownership)]
mod fault_ownership_tests {
    use super::*;
    use core::sync::atomic::AtomicUsize;

    struct Lease(Arc<AtomicUsize>);
    impl Lease {
        fn new(used: &Arc<AtomicUsize>) -> Self {
            used.fetch_add(1, Ordering::Relaxed);
            Self(used.clone())
        }
    }
    impl Drop for Lease {
        fn drop(&mut self) {
            self.0.fetch_sub(1, Ordering::Relaxed);
        }
    }

    // This tests the production predecessor ownership operation without firmware
    // storage. Runtime tests must separately establish its drain/kill witness.
    fn faulting_clients(publication: Option<(u64, u64)>) {
        let used = Arc::new(AtomicUsize::new(0), GFP_KERNEL).unwrap();
        for _ in 0..512 {
            let mut previous = Some(Previous {
                binding: Some(Lease::new(&used)),
                context: Lease::new(&used),
                publication,
            });
            assert_eq!(used.load(Ordering::Relaxed), 2);
            Previous::retire_witnessed(&mut previous);
            assert_eq!(previous.is_none(), publication.is_some());
            if publication.is_none() {
                assert_eq!(used.load(Ordering::Relaxed), 2);
                // An unpublished replacement returns its original owner on close.
                drop(previous.take());
            }
            assert_eq!(used.load(Ordering::Relaxed), 0);
            Previous::retire_witnessed(&mut previous);
            assert_eq!(used.load(Ordering::Relaxed), 0);
        }
    }

    #[test]
    fn faulted_published_replacements_return_to_baseline() {
        faulting_clients(Some((1, 7)));
    }

    #[test]
    fn unpublished_replacements_survive_until_rollback() {
        faulting_clients(None);
    }

    #[test]
    fn same_vm_replacement_without_binding_returns_its_context() {
        let used = Arc::new(AtomicUsize::new(0), GFP_KERNEL).unwrap();
        let mut previous = Some(Previous::<Lease, Lease> {
            binding: None,
            context: Lease::new(&used),
            publication: Some((9, 3)),
        });
        Previous::retire_witnessed(&mut previous);
        assert_eq!(used.load(Ordering::Relaxed), 0);
    }
}

impl Queue {
    /// Offer an open client's idle physical channel to another ready job.
    /// Logical queue/context ownership is unchanged. The existing binding is
    /// retained through the successor's exact replacement witness.
    pub(crate) fn share_idle(&mut self) -> Result<bool> {
        if !self.shareable() {
            return Ok(false);
        }
        self.release_owner()?;
        Ok(self.reusable())
    }

    /// Another owner could take this channel now: idle and released, shareable, or vacant.
    pub(crate) fn channel_free(&self) -> bool {
        self.reusable() || self.shareable() || self.can_rearm()
    }

    /// A pool replacement is in flight for this queue.
    pub(crate) fn growing(&self) -> bool {
        self.growing
    }

    /// The queue's first command has been published: its identity and configuration exist.
    pub(crate) fn installed(&self) -> bool {
        self.installed
    }

    /// A packet of the bound owner passed prepare_job and will publish here.
    pub(crate) fn claim(&mut self) {
        self.claims = self.claims.saturating_add(1);
    }

    pub(crate) fn release_claim(&mut self) {
        self.claims = self.claims.saturating_sub(1);
    }

    /// An open client's idle channel that [`Self::share_idle`] can hand over.
    pub(crate) fn shareable(&self) -> bool {
        self.claims == 0
            && self.installed
            && self.owner.is_some()
            && self.retirement_ready
            && !self.growing
            && !self.quarantined
            && !self.awaiting_witness
            && self.exit.is_none()
            && self.active.len == 0
            && self.replays.len == 0
            && self.previous.is_none()
            && self.pool_phase == PoolPhase::Populated
            && !self.source_hint.has_readers(&self.source_readers)
    }

    /// Device publication order of this channel's newest command: lower is
    /// idle longer.
    pub(crate) fn last_published(&self) -> u64 {
        self.last_published
    }
}

impl Queue {
    pub(crate) fn can_rearm(&self) -> bool {
        self.pool_phase == PoolPhase::Vacant
            && (self.reusable() || self.recycled_pending())
            && !self.source_hint.has_readers(&self.source_readers)
    }
}

impl Queue {
    /// A failed first attempt has no future completion to undo its binding.
    /// Return the replaced aliases to the caller for off-lock destruction. The
    /// caller validates the current preparation epoch and cancels only a true
    /// result's never-published QID; installed rings and identities stay intact.
    pub(crate) fn rollback_unpublished(
        &mut self,
        owner: u64,
        context: &Arc<Context>,
    ) -> Result<(bool, Option<Binding>)> {
        if self.owner != Some(owner)
            || !self.unpublished_binding
            || self
                .binding
                .as_ref()
                .is_none_or(|binding| !Arc::ptr_eq(&binding.context, context))
            || self.growing
            || self.quarantined
            || self.awaiting_witness
            || self.exit.is_some()
            || self.active.len != 0
            || self.replays.len != 0
            || self.retire_pending
            || self.failure_status_pending
            || self.pool_phase != PoolPhase::Populated
            || self
                .previous
                .as_ref()
                .is_some_and(|previous| previous.publication.is_some())
        {
            return Ok((false, None));
        }
        if !self.installed {
            return Ok((true, None));
        }
        let mut retired = None;
        if let Some(previous) = self.previous.as_ref() {
            self.graph.set_owner(&previous.context)?;
        }
        if let Some(mut previous) = self.previous.take() {
            if let Some(binding) = previous.binding.take() {
                retired = self.binding.replace(binding);
            } else {
                self.binding.as_mut().ok_or(EIO)?.context = previous.context;
            }
        }
        self.unpublished_binding = false;
        self.retirement_ready = true;
        self.released = true;
        self.owner = None;
        self.released_at = now_ns();
        Ok((false, retired))
    }
}

impl Queue {
    /// Same immutable singleton identity and current physical epoch.
    pub(crate) fn has_input_source(
        &self,
        source: &Arc<crate::g17::fence::Submission>,
        status: &Arc<VmStatus>,
        epoch: u64,
    ) -> bool {
        !self.quarantined
            && self.replays.len == 0
            && self.active.iter().any(|active| {
                active.ticket.is_some_and(|ticket| ticket.epoch == epoch)
                    && Arc::ptr_eq(active.packet.completion.status(), status)
                    && active.packet.completion.is_input_source(source)
            })
    }

    pub(crate) fn source_dependency(
        &mut self,
        source: &crate::g17::sync_input::Source,
        status: &Arc<VmStatus>,
        epoch: u64,
    ) -> Result<Option<SourceDependency>> {
        for active in self.active.iter() {
            if !active.packet.completion.is_input_source(&source.submission) {
                continue;
            }
            if !Arc::ptr_eq(active.packet.completion.status(), status) {
                return Err(EINVAL);
            }
            if self.quarantined || self.replays.len != 0 {
                return Err(EAGAIN);
            }
            let ticket = active.ticket.ok_or(EAGAIN)?;
            if ticket.epoch != epoch {
                return Err(EAGAIN);
            }
            self.valid_ticket(active, &ticket)?;
            if active.packet.cancel_requested() {
                return Err(ECANCELED);
            }
            source.submission.source_status()?;
            let dependency = crate::g17::fw::kick::KickDependency::new(
                self.qid(),
                crate::g17::fw::kick::KickTimestamp::new(ticket.kick).ok_or(EIO)?,
            )
            .ok_or(EIO)?;
            let hold = self.source_hint.hold(&self.source_readers)?;
            return Ok(Some((
                active.published,
                dependency,
                self.graph.support.gpu_va() + STAMP as u64,
                u32::from(self.qid()),
                hold,
            )));
        }
        Ok(None)
    }

    /// Preparing a possible wait does not create a firmware consumer. Mark it
    /// only once the consumer's first item is visible, under the device mutex.
    pub(crate) fn note_input_consumer(&self, source: &crate::g17::sync_input::Source) {
        for active in self.active.iter() {
            if active.packet.completion.is_input_source(&source.submission) {
                active.packet.completion.note_firmware_consumer();
                break;
            }
        }
    }
}

/// Publication, dependency, stamp address, queue and the retained source hold.
pub(crate) type SourceDependency = (
    u64,
    crate::g17::fw::kick::KickDependency,
    u64,
    u32,
    crate::g17::sync_input::Hold,
);
