// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Logical queue ownership of scheduler, QoS and work-state identities.
//!
//! Installed bindings retain this context until replacement is acknowledged.
//! A job keeps its work-state lease through exact retirement, even if its error
//! fence has already been signalled. Context-local accounting never waits for GPU work.

mod identity;
use super::{
    freelist::{RenderIds, RenderPool},
    fw::{context as fw, queue::Policy},
    object::{Allocator, CpuMap, Family, KernelObject, Pool, PooledObject},
    status::VmStatus,
};
use crate::mmu::{self, ExecutionContext, Vm, UAT_PGSZ};
use core::sync::atomic::{fence, AtomicBool, AtomicU32, AtomicUsize, Ordering};
use identity::{RenderSlots, WorkStates, FREE, NODE_STRIDE, STATE_COUNT};
use kernel::{
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
};

/// QoS IDs belong to one device; the two lowest values are reserved.
#[pin_data]
pub(crate) struct QosIds {
    #[pin]
    used: Mutex<u128>,
}
impl QosIds {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self { used <- new_mutex!(3, "G17 QoS IDs") }),
            GFP_KERNEL,
        )
    }
    fn acquire(self: &Arc<Self>) -> Result<QosLease> {
        let mut used = self.used.lock();
        let id = used.trailing_ones();
        if id == 128 {
            return Err(super::capacity::refused(super::capacity::Site::QosIds));
        }
        *used |= 1u128 << id;
        Ok(QosLease {
            ids: self.clone(),
            id: id as u8,
        })
    }
}
struct QosLease {
    ids: Arc<QosIds>,
    id: u8,
}
impl Drop for QosLease {
    fn drop(&mut self) {
        *self.ids.used.lock() &= !(1u128 << self.id);
    }
}

struct WorkStorage {
    object: PooledObject,
    words: [Option<u32>; STATE_COUNT],
    counts: [u32; STATE_COUNT],
    render: RenderSlots,
}
impl WorkStorage {
    fn initialize(&mut self, word: u32) -> Result {
        let slot = (word & 63) as usize;
        if slot >= STATE_COUNT {
            return Err(EINVAL);
        }
        let counter = UAT_PGSZ + slot * 4;
        let node = fw::WorkNode::new(self.object.gpu_va() + counter as u64, word).ok_or(EINVAL)?;
        // The exact work-state lease excludes a previous firmware owner. Only
        // the node's 0xc8 initialized bytes are replaced; the stride tail is retained.
        self.object.write(slot * NODE_STRIDE, node)?;
        self.object.word(counter)?.store(0, Ordering::Relaxed);
        self.words[slot] = Some(word);
        self.counts[slot] = 0;
        fence(Ordering::Release);
        Ok(())
    }
    /// Whether the firmware completed every kick counted so far on node `word`.
    fn drained(&self, word: u32) -> Result<bool> {
        let slot = (word & 63) as usize;
        if self.words.get(slot) != Some(&Some(word)) {
            return Err(EINVAL);
        }
        let completed = self
            .object
            .word(slot * NODE_STRIDE + core::mem::offset_of!(fw::WorkNode, completed))?
            .load(Ordering::Relaxed);
        fence(Ordering::Acquire);
        Ok(completed >= self.counts[slot])
    }
    fn change_count(&mut self, word: u32, kicks: u32, add: bool) -> Result<u32> {
        let slot = (word & 63) as usize;
        if self.words.get(slot) != Some(&Some(word)) {
            return Err(EINVAL);
        }
        let value = if add {
            self.counts[slot].checked_add(kicks)
        } else {
            self.counts[slot].checked_sub(kicks)
        }
        .ok_or(EOVERFLOW)?;
        self.object
            .word(UAT_PGSZ + slot * 4)?
            .store(value, Ordering::Relaxed);
        self.counts[slot] = value;
        fence(Ordering::SeqCst);
        Ok(value)
    }
}

/// Firmware-visible pages of a context, retained by an installed queue whose
/// last owner has exited (see [`Context::retain_firmware_pages`]).
pub(crate) struct FirmwarePages {
    _scheduler: Arc<KernelObject>,
    _work: Arc<Mutex<WorkStorage>>,
}

pub(crate) struct Context {
    execution: Arc<ExecutionContext>,
    status: Arc<VmStatus>,
    scheduler: Arc<KernelObject>,
    scheduler_generation: u32,
    published: AtomicBool,
    /// A kill of this context was published, through a compute queue or on its render pairs'
    /// behalf. A context is killed at most once.
    kill_published: AtomicBool,
    /// Advisory only: publication still checks the physical binding under the
    /// device mutex. False skips an inline attempt known to need admission.
    compute_binding_hint: AtomicBool,
    /// Mixed-engine contexts keep the original inline publication path.
    render_seen: AtomicBool,
    qos: QosLease,
    owner_pid: u32,
    states: WorkStates,
    /// Signalled when a work state returns; publishers that found none wait on it.
    state_returns: Arc<super::availability::Availability>,
    work: Arc<Mutex<WorkStorage>>,
    work_base: u64,
    render: Arc<Mutex<Option<Arc<RenderPool>>>>,
    render_blocks: AtomicUsize,
    /// Timestamp pages of retired compute commands, mapped in this context's VM.
    compute_timestamps: Arc<super::completion::ComputeTimestampCache>,
    /// Fixed for the context's life. Physical queues bound to it carry this
    /// profile in every record, ring, doorbell and registration.
    policy: Policy,
}
impl Context {
    pub(crate) fn new(
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        qos: &Arc<QosIds>,
        execution: Arc<ExecutionContext>,
        owner_pid: u32,
        policy: Policy,
        state_returns: Arc<super::availability::Availability>,
    ) -> Result<Self> {
        let status = execution.vm().status().ok_or(EINVAL)?.clone();
        let qos = qos.acquire()?;
        let mut scheduler = alloc.kernel(
            UAT_PGSZ,
            UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        scheduler.initialize::<fw::Scheduler>(0, |value| {
            value.init();
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        let scheduler_generation = super::ids::OBJECTS.object()?;
        let scheduler = Arc::new(scheduler, GFP_KERNEL)?;
        let mut object = pool.allocate(
            alloc,
            Family::WorkState,
            2 * UAT_PGSZ,
            UAT_PGSZ,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let work_base = object.gpu_va();
        object.write(
            UAT_PGSZ + 0x400,
            fw::WorkHead {
                next: 0,
                previous: work_base + (UAT_PGSZ + 0x400) as u64,
                unk10: 0,
                unk14: 0,
            },
        )?;
        fence(Ordering::Release);
        let work = Arc::pin_init(
            new_mutex!(
                WorkStorage {
                    object,
                    words: [None; STATE_COUNT],
                    counts: [0; STATE_COUNT],
                    render: RenderSlots::new()
                },
                "G17 work states"
            ),
            GFP_KERNEL,
        )?;
        let render = Arc::pin_init(new_mutex!(None, "G17 context private memory"), GFP_KERNEL)?;
        Ok(Self {
            execution,
            status,
            scheduler,
            scheduler_generation,
            published: AtomicBool::new(false),
            kill_published: AtomicBool::new(false),
            compute_binding_hint: AtomicBool::new(false),
            render_seen: AtomicBool::new(false),
            qos,
            owner_pid,
            states: WorkStates::new(),
            state_returns,
            work,
            work_base,
            render,
            render_blocks: AtomicUsize::new(super::fw::freelist::RENDER_BLOCKS),
            compute_timestamps: super::completion::ComputeTimestampCache::new()?,
            policy,
        })
    }
    pub(crate) fn vm(&self) -> &Vm {
        self.execution.vm()
    }
    pub(crate) fn compute_binding_hint(&self) -> bool {
        self.render_seen.load(Ordering::Acquire)
            || self.compute_binding_hint.load(Ordering::Relaxed)
    }
    /// This logical queue has submitted render work, so its compute may have a render prefix.
    pub(crate) fn render_seen(&self) -> bool {
        self.render_seen.load(Ordering::Acquire)
    }
    pub(crate) fn note_render(&self) {
        if !self.render_seen.load(Ordering::Relaxed) {
            self.render_seen.store(true, Ordering::Release);
        }
    }
    pub(crate) fn note_compute_binding(&self, owned: bool) {
        self.compute_binding_hint.store(owned, Ordering::Relaxed);
    }
    pub(crate) fn compute_timestamps(&self) -> &Arc<super::completion::ComputeTimestampCache> {
        &self.compute_timestamps
    }
    pub(crate) fn status(&self) -> &Arc<VmStatus> {
        &self.status
    }
    pub(crate) fn id(&self) -> u32 {
        self.execution.id()
    }

    pub(crate) fn generation(&self) -> u8 {
        self.execution.generation()
    }
    pub(crate) fn is_current(&self) -> bool {
        self.execution.is_current()
    }
    pub(crate) fn owner_pid(&self) -> u32 {
        self.owner_pid
    }
    pub(crate) fn policy(&self) -> Policy {
        self.policy
    }
    pub(crate) fn qos_id(&self) -> u8 {
        self.qos.id
    }
    pub(crate) fn scheduler_va(&self) -> u64 {
        self.scheduler.gpu_va()
    }
    /// Installed physical channels retain this page after the logical execution
    /// context and its VM lease have completed their scheduler release.
    pub(crate) fn scheduler_owner(&self) -> Arc<KernelObject> {
        self.scheduler.clone()
    }
    /// Scheduler release and command retirement have both been witnessed.
    /// Keep installed storage alive while returning the finite TTBAT identity.
    pub(crate) fn release_execution(&self) {
        self.execution.release();
    }
    pub(crate) fn scheduler_generation(&self) -> u32 {
        self.scheduler_generation
    }
    /// The pages an installed queue record of this context still names: the
    /// scheduler page and the work storage holding the job-list head. A retained
    /// physical queue keeps these after dropping the context itself.
    pub(crate) fn retain_firmware_pages(&self) -> FirmwarePages {
        FirmwarePages {
            _scheduler: self.scheduler.clone(),
            _work: self.work.clone(),
        }
    }
    /// Called once the firmware consumed this context's scheduler release. If its work list
    /// still links a node, the firmware keeps referencing that node: on Neo a render command
    /// that ran behind a failed compute command is never unlinked, and a later context given
    /// the same work storage (the object cache hands it out again) made the firmware recover
    /// the GPU. Keep the storage mapped and out of reuse until firmware stop; returns whether
    /// it was linked.
    pub(crate) fn retain_linked_work(&self, pool: &Arc<Pool>) -> bool {
        // An unreadable head counts as linked: retaining is the safe side.
        let linked = self.work_head().map_or(true, |head| head != [0, self.work_head_va()]);
        if linked {
            self.work.lock().object.retain_until_stop(pool);
        }
        linked
    }

    /// The firmware may still walk this context's work list into memory it names: a node is
    /// linked and the scheduler page shows no kill. Unreadable state counts as linked.
    pub(crate) fn firmware_may_link(&self) -> bool {
        let killed = super::queue::compute::exit::State::read(self).map_or(0, |state| state.killed);
        must_retain(self.work_head().ok(), self.work_head_va(), killed)
    }

    pub(crate) fn work_head_va(&self) -> u64 {
        self.work_base + (UAT_PGSZ + 0x400) as u64
    }
    /// Shared scheduler list head, sampled under the context's storage lock.
    pub(crate) fn work_head(&self) -> Result<[u64; 2]> {
        let work = self.work.lock();
        let head = [
            work.object.dword(UAT_PGSZ + 0x400)?.load(Ordering::Relaxed),
            work.object.dword(UAT_PGSZ + 0x408)?.load(Ordering::Relaxed),
        ];
        fence(Ordering::Acquire);
        Ok(head)
    }

    /// Record that a kill of this context was published (see `kill_published`).
    pub(crate) fn note_kill_published(&self) {
        self.kill_published.store(true, Ordering::Release);
    }

    /// A kill of this context was published.
    pub(crate) fn kill_published(&self) -> bool {
        self.kill_published.load(Ordering::Acquire)
    }

    pub(crate) fn is_published(&self) -> bool {
        self.published.load(Ordering::Acquire)
    }
    pub(crate) fn mark_published(&self) {
        self.published.store(true, Ordering::Release);
    }

    pub(crate) fn release_identity(&self) -> Result<[u8; 4]> {
        let mut result = [0; 4];
        fence(Ordering::Acquire);
        for (value, offset) in result.iter_mut().zip([0, 1, 4, 0x26]) {
            let pointer = self.scheduler.pointer(offset, 1)?;
            // SAFETY: pointer checked against retained scheduler allocation;
            // a byte read observes firmware state without borrowing its fields.
            *value = unsafe { core::ptr::read_volatile(pointer) };
        }
        Ok(result)
    }
    pub(crate) fn reserve_render_blocks(&self, blocks: usize) -> Result {
        if self.is_published() || self.render.lock().is_some() { return Err(EBUSY); }
        self.render_blocks.store(blocks, Ordering::Release);
        Ok(())
    }
    pub(crate) fn render_blocks(&self) -> usize {
        self.render_blocks.load(Ordering::Acquire)
    }
    pub(crate) fn render_pool(
        &self,
        alloc: &Allocator<'_>,
        ids: &Arc<RenderIds>,
        render_global: &Vm,
    ) -> Result<Arc<RenderPool>> {
        if !self.is_current() {
            return Err(EFAULT);
        }
        let mut render = self.render.lock();
        if let Some(pool) = &*render {
            // A pool vacated under memory pressure is re-armed before reuse.
            pool.ensure_backing(alloc, render_global, self.vm())?;
            return Ok(pool.clone());
        }
        let pool = Arc::new(
            RenderPool::new(alloc, ids, render_global, self.vm(), self.render_blocks())?,
            GFP_KERNEL,
        )?;
        *render = Some(pool.clone());
        Ok(pool)
    }
    /// Drops the pool once its free-list release is consumed and the closed
    /// scheduler is detached. Installed compute bindings may retain the
    /// context itself until replacement; they never use the render pool.
    pub(crate) fn take_render_pool(&self) -> Option<Arc<RenderPool>> {
        self.render.lock().take()
    }
    pub(crate) fn render_pools_released(&self, released: impl Fn(u32, u64) -> bool) -> bool {
        self.render
            .lock()
            .as_ref()
            .is_none_or(|pool| released(u32::from(pool.id()), pool.control_va()))
    }
}

/// Shared by all engines of one ioctl. Queued work initially consumes no node.
pub(crate) struct WorkStateLease {
    context: Arc<Context>,
    word: AtomicU32,
}
impl WorkStateLease {
    pub(super) fn status(&self) -> &Arc<VmStatus> { self.context.status() }
    pub(crate) fn deferred(context: Arc<Context>) -> Self {
        Self {
            context,
            word: AtomicU32::new(FREE),
        }
    }
    pub(crate) fn acquire(&self) -> Result<u32> {
        let current = self.word();
        if current != FREE {
            return Ok(current);
        }
        let Some(candidate) = self.context.states.allocate() else {
            let current = self.word();
            return if current != FREE {
                Ok(current)
            } else {
                Err(EBUSY)
            };
        };
        let initialized = self.context.work.lock().initialize(candidate);
        if let Err(error) = initialized {
            self.context.release_state(candidate);
            return Err(error);
        }
        match self
            .word
            .compare_exchange(FREE, candidate, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => Ok(candidate),
            Err(installed) => {
                self.context.release_state(candidate);
                Ok(installed)
            }
        }
    }
    /// The domain a publisher samples before acquire() and waits on after EBUSY.
    pub(crate) fn returns(&self) -> &Arc<super::availability::Availability> {
        &self.context.state_returns
    }
    pub(crate) fn word(&self) -> u32 {
        self.word.load(Ordering::Acquire)
    }
    pub(crate) fn node_va(&self) -> Result<u64> {
        let word = self.word();
        if word == FREE {
            return Err(EINVAL);
        }
        Ok(self.context.work_base + u64::from(word & 63) * NODE_STRIDE as u64)
    }
    /// Whether the firmware completed every kick counted so far on this ioctl's node. It stores
    /// the completion stamps of the node's commands only then, so a command whose record is
    /// present still waits for its stamp while other commands of its ioctl run.
    pub(crate) fn drained(&self) -> Result<bool> {
        let word = self.word();
        if word == FREE {
            return Err(EINVAL);
        }
        self.context.work.lock().drained(word)
    }
    pub(crate) fn add_submitted_kicks(&self, kicks: u32) -> Result<u32> {
        self.context
            .work
            .lock()
            .change_count(self.word(), kicks, true)
    }
    pub(crate) fn cancel_unpublished_kicks(&self, kicks: u32) -> Result<u32> {
        self.context
            .work
            .lock()
            .change_count(self.word(), kicks, false)
    }

    pub(crate) fn render_slot(&self, pair: u8) -> Result<u32> {
        self.context.work.lock().render.get(pair).ok_or_else(||
            super::capacity::refused(super::capacity::Site::RenderWorkSlots))
    }
    pub(crate) fn channel_payload(&self, pair: u8, ordinal: u64) -> Result<u8> {
        self.context
            .work
            .lock()
            .render
            .payload(pair, ordinal)
            .ok_or(EINVAL)
    }
}
impl Drop for WorkStateLease {
    fn drop(&mut self) {
        let word = self.word();
        if word != FREE {
            self.context.release_state(word);
        }
    }
}

impl Context {
    /// Every return bumps the domain: a publisher that sampled it before its
    /// failed acquire() then never sleeps through this node.
    fn release_state(&self, word: u32) {
        if self.states.release(word) {
            self.state_returns.notify();
        }
    }
}

impl QosIds {
    pub(crate) fn used(&self) -> u32 {
        (*self.used.lock() & !3u128).count_ones()
    }
}

/// Memory a context's work list may lead the firmware to stays allocated: a linked node (or
/// an unreadable head) and no acknowledged kill.
fn must_retain(head: Option<[u64; 2]>, head_va: u64, killed: u32) -> bool {
    killed == 0 && head.is_none_or(|head| head != [0, head_va])
}

#[kunit_tests(asahi_g17_qos_capacity)]
mod capacity_tests {
    use super::*;

    #[test]
    fn linked_unkilled_work_is_retained() {
        let va = 0xfffffc20_0000_4400;
        assert!(!must_retain(Some([0, va]), va, 0));
        assert!(must_retain(Some([0xfffffc20_0000_0000, 0xfffffc20_0000_0098]), va, 0));
        assert!(must_retain(None, va, 0));
        assert!(!must_retain(Some([0xfffffc20_0000_0000, 0xfffffc20_0000_0098]), va, 1));
        assert!(!must_retain(None, va, 1));
    }

    #[test]
    fn failed_client_construction_returns_every_qos_id() {
        let ids = QosIds::new().unwrap();
        for _ in 0..8 {
            let mut clients = KVec::with_capacity(126, GFP_KERNEL).unwrap();
            for _ in 0..126 {
                clients.push(ids.acquire().unwrap(), GFP_KERNEL).unwrap();
            }
            assert_eq!(ids.used(), 126);
            assert!(matches!(ids.acquire(), Err(ENOSPC)));
            drop(clients);
            assert_eq!(ids.used(), 0);
            assert_eq!(*ids.used.lock(), 3);
        }
    }
}

impl Context {
    pub(crate) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>, scope: &str, index: usize) -> kernel::fmt::Result {
        writeln!(f, "context scope={} index={} id={} work_used={} work_total={} render_used={} render_total=2",
                 scope, index, self.id(), self.states.used(), STATE_COUNT, self.work.lock().render.used())
    }
}
