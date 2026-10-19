// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Compute slots, render pass slots and clustering buffers, and their background scrub.
//!
//! States CLEAN/USED/DIRTY/CLEARING/STALE. A retired slot becomes DIRTY and is reset on a
//! dedicated unbound, reclaim-safe workqueue (work images, sequences, TA/FS scratch, stamps,
//! dirty TPC/tilemap spans, aux/heap/scene-user, the 3 preemption areas); a claim resets
//! synchronously only if the slot is not CLEAN. Slot 0 keeps the maximum TPC/tilemap; a 17th
//! TPC slot would collide with the clustering arena.
//!
//! The slot objects never change after allocation and live in [`Scrub`], shared by the engine
//! (under the device lock) and the scrub work (which never takes the device lock); the states
//! and the dirty spans are under the scrub spinlock. A claim may flush the scrub work under the
//! device lock: the scrub work runs on its own `WQ_UNBOUND | WQ_MEM_RECLAIM` queue because
//! flushing a system-queue work from the reclaim-safe scheduler work WARNs.

use kernel::{
    bindings, c_str, impl_has_work, new_spinlock, new_work,
    prelude::*,
    sync::{Arc, SpinLock},
    workqueue::{self, HasWork, Work, WorkItem},
};

use super::encode::{Geometry, PassRefs};
use super::engine::wmb;
use super::fw::work::{barrier, compute, fs, ta};
use super::graph::{alloc, cmd_fw, cmd_shared, Client};
use super::hw::{client, sizes, tuning};
use super::object::{Class, FwObject, Objects};

/// Reset state of a slot's mutable storage.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum SlotState {
    /// All zero (scrubbed) and free.
    Clean,
    /// Held by a prefix being prepared or published.
    Used,
    /// Retired, waiting for the scrub.
    Dirty,
    /// Being scrubbed by the scrub work.
    Clearing,
    /// Retired without slot rotation: scrubbed at its next claim.
    Stale,
}

/// One compute command slot.
pub(crate) struct ComputeSlot {
    pub(crate) its: Arc<FwObject>,
    /// Part of the slot's firmware layout; the host does not use it.
    _uts: Arc<FwObject>,
    pub(crate) preempt: Arc<FwObject>,
    pub(crate) work: Arc<FwObject>,
    pub(crate) tail: Arc<FwObject>,
    pub(crate) seq: Arc<FwObject>,
    pub(crate) scratch: KVec<Arc<FwObject>>,
    /// Mixed prefix: the wait on the previous render's FS or the previous compute.
    pub(crate) barrier: Arc<FwObject>,
}

/// A compute slot's objects before its barrier exists (the barriers are allocated after the
/// render graph).
struct ComputeParts {
    its: Arc<FwObject>,
    uts: Arc<FwObject>,
    preempt: Arc<FwObject>,
    work: Arc<FwObject>,
    tail: Arc<FwObject>,
    seq: Arc<FwObject>,
    scratch: KVec<Arc<FwObject>>,
}

impl ComputeParts {
    /// Allocates compute slot `k`'s objects.
    fn new(o: &mut Objects, k: u64) -> Result<Self> {
        let sh = Class::Shared;
        let its = alloc(o, 16, sh, None, 0, false)?;
        let uts = alloc(o, 16, Class::Fw, None, 0, false)?;
        let preempt = alloc(
            o,
            0x10000,
            sh,
            Client::nc(client::PREEMPT + k * client::PREEMPT_STRIDE),
            0,
            false,
        )?;
        let work = alloc(
            o,
            compute::WORK_SIZE,
            cmd_shared(),
            Client::nc(client::WORK_PAGE + k * client::CWORK_STRIDE),
            client::WORK_OFF as usize,
            true,
        )?;
        let tail = alloc(o, 0x10000, sh, None, 0, false)?;
        let seq = alloc(o, compute::SEQ_OBJECT, cmd_fw(), None, 0, false)?;
        let mut scratch = KVec::with_capacity(5, GFP_KERNEL)?;
        for _ in 0..5 {
            scratch.push(alloc(o, 0x10000, sh, None, 0, false)?, GFP_KERNEL)?;
        }
        Ok(Self {
            its,
            uts,
            preempt,
            work,
            tail,
            seq,
            scratch,
        })
    }

    fn with_barrier(self, barrier: Arc<FwObject>) -> ComputeSlot {
        ComputeSlot {
            its: self.its,
            _uts: self.uts,
            preempt: self.preempt,
            work: self.work,
            tail: self.tail,
            seq: self.seq,
            scratch: self.scratch,
            barrier,
        }
    }
}

impl ComputeSlot {
    /// Per-use reset of a retired slot's runtime storage: timestamp cells, tail and sequence
    /// scratch zeroed, preemption poisoned with 0xcc.
    fn scrub(&self) -> Result {
        self.its.fill(0, 0, 16)?;
        self.tail.fill(0, 0, 0x10000)?;
        for s in self.scratch.iter() {
            s.fill(0, 0, 0x10000)?;
        }
        self.preempt.fill(0, 0xcc, 0x10000)
    }
}

/// One render pass slot.
pub(crate) struct PassSlot {
    pub(crate) aux: Arc<FwObject>,
    pub(crate) pre: KVec<Arc<FwObject>>,
    pub(crate) tpc: Arc<FwObject>,
    pub(crate) tilemap: Arc<FwObject>,
    pub(crate) heap: Arc<FwObject>,
    pub(crate) scene_user: Arc<FwObject>,
    pub(crate) fs_scratch: Arc<FwObject>,
    pub(crate) fs_work: Arc<FwObject>,
    pub(crate) ta_work: Arc<FwObject>,
    pub(crate) ta_seq: Arc<FwObject>,
    pub(crate) fs_seq: Arc<FwObject>,
    pub(crate) ta_scratch: Arc<FwObject>,
    pub(crate) scene: Arc<FwObject>,
    pub(crate) its: Arc<FwObject>,
    /// Part of the slot's firmware layout; the host does not use it.
    _uts: Arc<FwObject>,
    pub(crate) own_barrier: Arc<FwObject>,
    pub(crate) prev_barrier: Arc<FwObject>,
    pub(crate) fs_dep: Arc<FwObject>,
}

impl PassSlot {
    /// Allocates pass slot `i`'s objects, with their client leaf classes.
    fn new(o: &mut Objects, i: u64, pre_records: u32) -> Result<Self> {
        let sh = Class::Shared;
        let (ps, ws) = (i * client::PASS_STRIDE, i * client::WORK_STRIDE);
        let tpc = if i != 0 {
            client::TPC_SLOT_SIZE
        } else {
            client::TPC_SIZE
        };
        let tilemap = if i != 0 {
            client::TILEMAP_SLOT_SIZE
        } else {
            client::TILEMAP_SIZE
        };
        let r = pre_records as usize;

        let aux = alloc(
            o,
            0x20000,
            sh,
            Client::nc(client::AUX + i * client::AUX_STRIDE),
            0,
            false,
        )?;
        let mut pre = KVec::with_capacity(3, GFP_KERNEL)?;
        pre.push(
            alloc(
                o,
                r * 0x540,
                sh,
                Client::cached(client::PRE0_PAGE + ps),
                0x3580,
                false,
            )?,
            GFP_KERNEL,
        )?;
        pre.push(
            alloc(
                o,
                r * 0x280,
                sh,
                Client::cached(client::PRE1_PAGE + ps),
                0x3b00,
                false,
            )?,
            GFP_KERNEL,
        )?;
        pre.push(
            alloc(
                o,
                r * 0x40,
                sh,
                Client::cached(client::PRE2_PAGE + ps),
                0x3f80,
                false,
            )?,
            GFP_KERNEL,
        )?;
        let tpc = alloc(
            o,
            tpc as usize,
            sh,
            Client::cached(client::TPC + i * client::TPC_SLOT_STRIDE),
            0,
            false,
        )?;
        let tilemap = alloc(
            o,
            tilemap as usize,
            sh,
            Client::cached(client::TILEMAP + i * client::TPC_SLOT_STRIDE),
            0,
            false,
        )?;
        let heap = alloc(
            o,
            0x1000,
            sh,
            Client::cached(client::HEAP_META_PAGE + ps),
            0x3000,
            false,
        )?;
        let scene_user = alloc(
            o,
            0x10000,
            sh,
            Client::cached(client::SCENE_USER + ps),
            0,
            false,
        )?;
        let fs_scratch = alloc(
            o,
            0x10000,
            sh,
            Client::cached(client::FS_SCRATCH + ps),
            0,
            false,
        )?;
        let fs_work = alloc(
            o,
            fs::WORK_SIZE,
            cmd_shared(),
            Client::nc(client::FS_WORK_PAGE + ws),
            client::FS_WORK_OFF as usize,
            true,
        )?;
        let ta_work = alloc(
            o,
            ta::WORK_SIZE,
            cmd_shared(),
            Client::nc(client::TA_WORK_PAGE + ws),
            client::TA_WORK_OFF as usize,
            true,
        )?;
        Ok(Self {
            aux,
            pre,
            tpc,
            tilemap,
            heap,
            scene_user,
            fs_scratch,
            fs_work,
            ta_work,
            ta_seq: alloc(o, ta::SEQ_OBJECT, cmd_fw(), None, 0, false)?,
            fs_seq: alloc(o, fs::SEQ_OBJECT, cmd_fw(), None, 0, false)?,
            ta_scratch: alloc(o, 0x10000, sh, None, 0, false)?,
            scene: alloc(o, super::fw::work::scene::SIZE, sh, None, 0, false)?,
            // Internal TA start/end, FS start/end; legacy user cells likewise.
            its: alloc(o, 32, sh, None, 0, false)?,
            _uts: alloc(o, 32, Class::Fw, None, 0, false)?,
            own_barrier: alloc(o, barrier::SIZE, cmd_fw(), None, 0, false)?,
            prev_barrier: alloc(o, barrier::SIZE, cmd_fw(), None, 0, false)?,
            fs_dep: alloc(o, barrier::SIZE, cmd_fw(), None, 0, false)?,
        })
    }

    /// Per-use reset of a pass slot's storage: work images, microsequences, scratch, stage
    /// stamps, the TPC/tilemap spans its last use wrote, auxiliary, heap metadata and
    /// scene-user (`PASS_FULL_RESET`) and the preemption save areas. The slot is neither
    /// published nor claimed; the dirty spans are passed in.
    fn scrub(&self, pre_records: u32, tpc_dirty: u64, tilemap_dirty: u64) -> Result {
        let r = pre_records as usize;

        self.ta_work.fill(0, 0, ta::WORK_SIZE)?;
        self.fs_work.fill(0, 0, fs::WORK_SIZE)?;
        self.ta_seq.fill(0, 0, ta::SEQ_OBJECT)?;
        self.fs_seq.fill(0, 0, fs::SEQ_OBJECT)?;
        self.ta_scratch.fill(0, 0, 0x10000)?;
        self.fs_scratch.fill(0, 0, 0x10000)?;
        self.its.fill(0, 0, 32)?;
        self.tpc.fill(0, 0, tpc_dirty as usize)?;
        self.tilemap.fill(0, 0, tilemap_dirty as usize)?;
        if tuning::PASS_FULL_RESET {
            self.aux.fill(0, 0, 0x20000)?;
            self.heap.fill(0, 0, 0x1000)?;
            self.scene_user.fill(0, 0, 0x10000)?;
        }
        // Preemption save areas start zeroed on every use.
        self.pre[0].fill(0, 0, r * 0x540)?;
        self.pre[1].fill(0, 0, r * 0x280)?;
        self.pre[2].fill(0, 0, r * 0x40)
    }

    /// The addresses the render encoders name.
    pub(crate) fn refs(&self) -> PassRefs {
        PassRefs {
            ta_work: self.ta_work.va,
            ta_work_gva: self.ta_work.gva.unwrap_or(0),
            fs_work: self.fs_work.va,
            fs_work_gva: self.fs_work.gva.unwrap_or(0),
            ta_seq: self.ta_seq.va,
            fs_seq: self.fs_seq.va,
            scene: self.scene.va,
            tpc_gva: self.tpc.gva.unwrap_or(0),
            tilemap_gva: self.tilemap.gva.unwrap_or(0),
            heap_gva: self.heap.gva.unwrap_or(0),
            scene_user_gva: self.scene_user.gva.unwrap_or(0),
            pre_gva: [
                self.pre[0].gva.unwrap_or(0),
                self.pre[1].gva.unwrap_or(0),
                self.pre[2].gva.unwrap_or(0),
            ],
            fs_scratch_gva: self.fs_scratch.gva.unwrap_or(0),
            aux_gva: self.aux.gva.unwrap_or(0),
            its: self.its.va,
            ta_scratch: self.ta_scratch.va,
        }
    }
}

/// Reset states and dirty spans, under the scrub spinlock.
struct States {
    compute: [SlotState; sizes::COMPUTE_SLOTS],
    pass: [SlotState; sizes::RENDER_SLOTS],
    /// The TPC/tilemap spans a pass slot's last use may have written.
    tpc_dirty: [u64; sizes::RENDER_SLOTS],
    tilemap_dirty: [u64; sizes::RENDER_SLOTS],
    ct: [SlotState; sizes::CT_BUFS],
    /// The span a clustering buffer's pass may have written.
    ct_dirty: [u64; sizes::CT_BUFS],
}

/// An owned `WQ_UNBOUND | WQ_MEM_RECLAIM` workqueue with one active work (allocated ordered,
/// which is the same for its single work item).
struct ClearQueue(*mut bindings::workqueue_struct);

// SAFETY: The workqueue API is thread-safe; this only holds the pointer.
unsafe impl Send for ClearQueue {}
// SAFETY: The workqueue API is thread-safe; this only holds the pointer.
unsafe impl Sync for ClearQueue {}

impl ClearQueue {
    fn new() -> Result<Self> {
        // SAFETY: The name is a valid NUL-terminated string the C side copies.
        let ptr = unsafe {
            bindings::alloc_ordered_workqueue(
                c_str!("asahi-g15-clear").as_char_ptr(),
                bindings::wq_flags_WQ_MEM_RECLAIM,
            )
        };
        if ptr.is_null() {
            return Err(ENOMEM);
        }
        Ok(Self(ptr))
    }

    fn queue(&self) -> &workqueue::Queue {
        // SAFETY: The pointer is a live workqueue until `self` is dropped.
        unsafe { workqueue::Queue::from_raw(self.0) }
    }
}

impl Drop for ClearQueue {
    fn drop(&mut self) {
        // SAFETY: The pointer came from `alloc_ordered_workqueue` and is not used afterwards;
        // `destroy_workqueue` drains it first.
        unsafe { bindings::destroy_workqueue(self.0) };
    }
}

/// The slot objects and the background scrub.
#[pin_data]
pub(crate) struct Scrub {
    pub(crate) compute: KVec<ComputeSlot>,
    pub(crate) passes: KVec<PassSlot>,
    /// Clustering buffers: a GPU-cached client view of 64 tilemap segments and four metadata
    /// regions, after the ordinary TPC/tilemap slots.
    pub(crate) cts: KVec<Arc<FwObject>>,
    /// Preemption records per buffer: 2, 3 with clustering.
    pub(crate) pre_records: u32,
    #[pin]
    states: SpinLock<States>,
    #[pin]
    work: Work<Scrub>,
    queue: ClearQueue,
}

impl_has_work! {
    impl HasWork<Scrub> for Scrub { self.work }
}

impl WorkItem for Scrub {
    type Pointer = Arc<Scrub>;

    fn run(this: Arc<Scrub>) {
        this.clear_all();
    }
}

impl Scrub {
    /// Resets every DIRTY pass slot, compute slot and clustering buffer, outside the device
    /// lock.
    fn clear_all(&self) {
        loop {
            let taken = {
                let mut st = self.states.lock();
                let k = st.pass.iter().position(|s| *s == SlotState::Dirty);
                k.map(|k| {
                    st.pass[k] = SlotState::Clearing;
                    let dirty = (st.tpc_dirty[k], st.tilemap_dirty[k]);
                    st.tpc_dirty[k] = 0;
                    st.tilemap_dirty[k] = 0;
                    (k, dirty)
                })
            };
            let Some((k, (tpc, tilemap))) = taken else {
                break;
            };
            let _ = self.passes[k].scrub(self.pre_records, tpc, tilemap);
            wmb(); // reset before the slot can be claimed
            self.states.lock().pass[k] = SlotState::Clean;
        }
        loop {
            let taken = {
                let mut st = self.states.lock();
                let k = st.compute.iter().position(|s| *s == SlotState::Dirty);
                if let Some(k) = k {
                    st.compute[k] = SlotState::Clearing;
                }
                k
            };
            let Some(k) = taken else {
                break;
            };
            let _ = self.compute[k].scrub();
            wmb(); // scrubbed before the slot can be claimed
            self.states.lock().compute[k] = SlotState::Clean;
        }
        loop {
            let taken = {
                let mut st = self.states.lock();
                let k = st.ct.iter().position(|s| *s == SlotState::Dirty);
                k.map(|k| {
                    st.ct[k] = SlotState::Clearing;
                    (k, st.ct_dirty[k])
                })
            };
            let Some((k, dirty)) = taken else {
                return;
            };
            let _ = self.cts[k].fill(0, 0, dirty as usize);
            wmb(); // zeroes complete before the buffer can be claimed
            let mut st = self.states.lock();
            st.ct_dirty[k] = 0;
            st.ct[k] = SlotState::Clean;
        }
    }

    /// Waits for the scrub work.
    fn flush(self: &Arc<Self>) {
        // SAFETY: `self` is a live `Arc<Scrub>`, so its `work` field is a valid, initialized
        // `work_struct` for the duration of the call.
        unsafe {
            let work = <Scrub as HasWork<Scrub>>::raw_get_work(Arc::as_ptr(self).cast_mut());
            bindings::flush_work(Work::raw_get(work));
        }
    }

    /// Queues the scrub work on its own queue.
    fn kick(self: &Arc<Self>) {
        // Already queued if this fails, which is just as good.
        let _ = self.queue.queue().enqueue(self.clone());
    }
}

/// The storage claims of a prefix: compute slots, pass slots and clustering buffers.
#[derive(Copy, Clone)]
pub(crate) struct Claims {
    /// Compute slot of compute member j.
    pub(crate) slot: [u32; sizes::COMPUTE_SLOTS],
    pub(crate) slot_count: u32,
    /// Pass slot of render member j, and the set of them.
    pub(crate) pass: [u32; sizes::RENDER_SLOTS],
    pub(crate) pass_mask: u32,
    /// Clustering buffers claimed.
    pub(crate) ct: [u32; sizes::RENDER_SLOTS],
    pub(crate) ct_count: u32,
    /// The compute slots rotate (`tuning::SLOT_ROTATE` at the first member).
    pub(crate) rotating: bool,
}

impl Claims {
    pub(crate) const fn new() -> Self {
        Self {
            slot: [0; sizes::COMPUTE_SLOTS],
            slot_count: 0,
            pass: [0; sizes::RENDER_SLOTS],
            pass_mask: 0,
            ct: [0; sizes::RENDER_SLOTS],
            ct_count: 0,
            rotating: false,
        }
    }
}

/// Slot accounting statistics (`asahi-g15/stats`).
#[derive(Copy, Clone, Default)]
pub(crate) struct SlotStats {
    /// Clustering buffers zeroed at claim, pass slots reset at claim, compute slots scrubbed
    /// at claim.
    pub(crate) ct_sync: u64,
    pub(crate) pass_sync: u64,
    pub(crate) slot_sync: u64,
    /// Claims that waited for the scrub work, and how long (timed only with debug_flags bit 49).
    pub(crate) flushes: u64,
    pub(crate) flush_ns: u64,
}

/// Slot and storage state.
pub(crate) struct Slots {
    pub(crate) scrub: Arc<Scrub>,
    /// Next compute slot to try.
    pub(crate) cursor: u32,
    /// Pass slots held by published or prepared prefixes.
    pub(crate) pass_busy: u32,
    /// Next small pass slot (1..slots-1) to try.
    pub(crate) pass_cursor: u32,
    pub(crate) stats: SlotStats,
}

impl Slots {
    /// Waits for the scrub work, counted.
    fn flush_scrub(&mut self, scrub: &Arc<Scrub>) {
        let mut lap = super::engine::Lap::start();
        scrub.flush();
        self.stats.flushes += 1;
        lap.lap(&mut self.stats.flush_ns);
    }

    /// Allocates the compute slots, the pass slots, the compute slots' barriers, the clustering
    /// buffers and the scrub workqueue.
    pub(crate) fn new(objects: &mut Objects) -> Result<Self> {
        // Clustered tiling needs three records in each preemption buffer (two per-cluster plus
        // an aggregate), each buffer then spanning two pages from the same payload offset.
        let pre_records = if tuning::VERTEX_CLUSTERING { 3 } else { 2 };
        let mut compute = KVec::with_capacity(sizes::COMPUTE_SLOTS, GFP_KERNEL)?;
        let mut passes = KVec::with_capacity(sizes::RENDER_SLOTS, GFP_KERNEL)?;
        let mut cts = KVec::with_capacity(sizes::CT_BUFS, GFP_KERNEL)?;

        // Allocation order: the compute slots, the pass slots, then the compute slots' barriers
        // and the clustering buffers.
        let mut parts = KVec::with_capacity(sizes::COMPUTE_SLOTS, GFP_KERNEL)?;
        for k in 0..sizes::COMPUTE_SLOTS as u64 {
            parts.push(ComputeParts::new(objects, k)?, GFP_KERNEL)?;
        }
        for i in 0..sizes::RENDER_SLOTS as u64 {
            passes.push(PassSlot::new(objects, i, pre_records)?, GFP_KERNEL)?;
        }
        for part in parts.drain_all() {
            let barrier = alloc(objects, barrier::SIZE, cmd_fw(), None, 0, false)?;
            compute.push(part.with_barrier(barrier), GFP_KERNEL)?;
        }
        // Clustering storage: GPU-cached client view and shared upper alias.
        if pre_records == 3 {
            for k in 0..sizes::CT_BUFS as u64 {
                let ct = alloc(
                    objects,
                    client::CT_SIZE as usize,
                    Class::Shared,
                    Client::cached(client::CT + k * client::CT_STRIDE),
                    0,
                    false,
                )?;
                cts.push(ct, GFP_KERNEL)?;
            }
        }
        let scrub = Arc::pin_init(
            try_pin_init!(Scrub {
                compute,
                passes,
                cts,
                pre_records,
                states <- new_spinlock!(
                    States {
                        compute: [SlotState::Clean; sizes::COMPUTE_SLOTS],
                        pass: [SlotState::Clean; sizes::RENDER_SLOTS],
                        tpc_dirty: [0; sizes::RENDER_SLOTS],
                        tilemap_dirty: [0; sizes::RENDER_SLOTS],
                        ct: [SlotState::Clean; sizes::CT_BUFS],
                        ct_dirty: [0; sizes::CT_BUFS],
                    },
                    "g15::Scrub::states"
                ),
                work <- new_work!("g15::Scrub::work"),
                queue: ClearQueue::new()?,
            }),
            GFP_KERNEL,
        )?;
        Ok(Self {
            scrub,
            cursor: 0,
            pass_busy: 0,
            pass_cursor: 0,
            stats: SlotStats::default(),
        })
    }

    /// Scrubs every compute slot and marks it CLEAN. Pass slots and clustering buffers start as
    /// fresh zero pages.
    pub(crate) fn init(&self) -> Result {
        for s in self.scrub.compute.iter() {
            s.scrub()?;
        }
        let mut st = self.scrub.states.lock();
        st.compute = [SlotState::Clean; sizes::COMPUTE_SLOTS];
        Ok(())
    }

    /// Compute slots a new prefix can still claim: not held by a published one.
    pub(crate) fn free_compute(&self) -> u32 {
        let st = self.scrub.states.lock();
        st.compute.iter().filter(|s| **s != SlotState::Used).count() as u32
    }

    /// A scrubbed compute slot for compute member `j` of the prefix being prepared: the first
    /// clean slot from the cursor, which follows the previous prefix's slots, so they can be
    /// scrubbed while this one runs; with none clean, a retired slot is scrubbed here. Slots
    /// of published prefixes are never taken. Without rotation member j takes slot j when it
    /// is free. None: all held.
    pub(crate) fn claim_compute(&mut self, f: &mut Claims, j: u32) -> Result<Option<usize>> {
        let scrub = self.scrub.clone();
        let slots = sizes::COMPUTE_SLOTS;
        let mut found = None;
        let mut sync = false;

        if j == 0 {
            f.rotating = tuning::SLOT_ROTATE;
        }
        if !f.rotating && (j as usize) < slots {
            let k = j as usize;
            let mut st = scrub.states.lock();
            while st.compute[k] == SlotState::Clearing {
                drop(st);
                self.flush_scrub(&scrub);
                st = scrub.states.lock();
            }
            if st.compute[k] != SlotState::Used {
                sync = st.compute[k] != SlotState::Clean;
                st.compute[k] = SlotState::Used;
                found = Some(k);
            }
        }
        for _attempt in 0..2 {
            if found.is_some() {
                break;
            }
            let mut clearing = false;
            {
                let mut st = scrub.states.lock();
                for i in 0..slots {
                    let k = (self.cursor as usize + i) % slots;
                    if st.compute[k] == SlotState::Clean {
                        found = Some(k);
                        break;
                    }
                }
                if found.is_none() {
                    for i in 0..slots {
                        let k = (self.cursor as usize + i) % slots;
                        let s = st.compute[k];
                        if found.is_none() && (s == SlotState::Dirty || s == SlotState::Stale) {
                            found = Some(k);
                        }
                        clearing |= s == SlotState::Clearing;
                    }
                }
                if let Some(k) = found {
                    sync = st.compute[k] != SlotState::Clean;
                    st.compute[k] = SlotState::Used;
                }
            }
            if found.is_none() && !clearing {
                break; // all held by published prefixes
            }
            if found.is_none() {
                self.flush_scrub(&scrub); // the rest are being scrubbed
            }
        }
        let Some(k) = found else {
            return Ok(None);
        };
        if sync {
            scrub.compute[k].scrub()?;
            self.stats.slot_sync += 1;
        }
        self.cursor = ((k + 1) % slots) as u32;
        f.slot[f.slot_count as usize] = k as u32;
        f.slot_count += 1;
        Ok(Some(k))
    }

    /// A pass slot for a render of the given geometry, not held and not already picked: the
    /// next small slot after the previous prefix's, else slot 0 (which holds any admitted
    /// geometry); None if none is free.
    pub(crate) fn pick_pass(&self, geo: &Geometry, picked: u32) -> Option<usize> {
        let busy = self.pass_busy | picked;
        let slots = sizes::RENDER_SLOTS as u32;
        let passes = &self.scrub.passes;
        let fits = |k: usize| {
            geo.tpc_bytes <= passes[k].tpc.size as u64
                && geo.tilemap_bytes <= passes[k].tilemap.size as u64
        };

        if slots > 1 {
            for j in 0..slots - 1 {
                let k = 1 + (self.pass_cursor + j) % (slots - 1);
                if busy & (1 << k) == 0 && fits(k as usize) {
                    return Some(k as usize);
                }
            }
        }
        if busy & 1 == 0 && fits(0) {
            return Some(0);
        }
        None
    }

    /// Claims pass slot `k` for a render of the prefix being prepared: reset now unless the
    /// background reset already did.
    pub(crate) fn claim_pass(&mut self, k: usize) -> Result {
        let scrub = self.scrub.clone();
        let (sync, tpc, tilemap) = {
            let mut st = scrub.states.lock();
            while st.pass[k] == SlotState::Clearing {
                drop(st);
                self.flush_scrub(&scrub);
                st = scrub.states.lock();
            }
            let sync = st.pass[k] != SlotState::Clean;
            let dirty = (st.tpc_dirty[k], st.tilemap_dirty[k]);
            if sync {
                st.tpc_dirty[k] = 0;
                st.tilemap_dirty[k] = 0;
            }
            st.pass[k] = SlotState::Used;
            (sync, dirty.0, dirty.1)
        };
        if sync {
            scrub.passes[k].scrub(scrub.pre_records, tpc, tilemap)?;
            self.stats.pass_sync += 1;
        }
        Ok(())
    }

    /// Records the TPC/tilemap spans a claimed pass slot's render may write.
    pub(crate) fn set_pass_dirty(&mut self, k: usize, tpc: u64, tilemap: u64) {
        let mut st = self.scrub.states.lock();
        st.tpc_dirty[k] = tpc;
        st.tilemap_dirty[k] = tilemap;
    }

    /// A zeroed clustering buffer of at least `bytes` for a pass of the prefix being
    /// prepared, or None: none provisioned or too small.
    pub(crate) fn claim_ct(&mut self, f: &mut Claims, bytes: u64) -> Result<Option<usize>> {
        let scrub = self.scrub.clone();
        let mut found = None;
        let mut clear = 0;

        if scrub.cts.is_empty() || bytes > scrub.cts[0].size as u64 {
            return Ok(None);
        }
        for _attempt in 0..2 {
            {
                let mut st = scrub.states.lock();
                found = st.ct.iter().position(|s| *s == SlotState::Clean);
                // None clean: take a retired one and zero it here.
                if found.is_none() {
                    found = st.ct.iter().position(|s| *s == SlotState::Dirty);
                }
                if let Some(k) = found {
                    clear = if st.ct[k] == SlotState::Dirty {
                        st.ct_dirty[k]
                    } else {
                        0
                    };
                    st.ct[k] = SlotState::Used;
                }
            }
            if found.is_some() {
                break;
            }
            self.flush_scrub(&scrub); // the rest are being cleared
        }
        let Some(k) = found else {
            return Ok(None);
        };
        if clear != 0 {
            scrub.cts[k].fill(0, 0, clear as usize)?;
            self.stats.ct_sync += 1;
        }
        scrub.states.lock().ct_dirty[k] = bytes;
        f.ct[f.ct_count as usize] = k as u32;
        f.ct_count += 1;
        Ok(Some(k))
    }

    /// The storage of a retired (or never published) prefix: compute slots and clustering
    /// buffers are reset in the background while later prefixes run; its pass slots become
    /// free again.
    pub(crate) fn release(&mut self, f: &mut Claims) {
        let mut mask = f.pass_mask;

        self.pass_busy &= !f.pass_mask;
        f.pass_mask = 0;
        if f.ct_count == 0 && f.slot_count == 0 && mask == 0 {
            return;
        }
        {
            let mut st = self.scrub.states.lock();
            let mut k = 0;
            while mask != 0 {
                // Only a slot this prefix used is dirty; one it claimed but never wrote (a
                // publication that failed before its records) keeps its state.
                if mask & 1 != 0 && st.pass[k] == SlotState::Used {
                    st.pass[k] = SlotState::Dirty;
                }
                k += 1;
                mask >>= 1;
            }
            for c in &f.ct[..f.ct_count as usize] {
                st.ct[*c as usize] = SlotState::Dirty;
            }
            // Without rotation the next claim of a slot scrubs it (STALE).
            for s in &f.slot[..f.slot_count as usize] {
                st.compute[*s as usize] = if f.rotating {
                    SlotState::Dirty
                } else {
                    SlotState::Stale
                };
            }
        }
        f.ct_count = 0;
        f.slot_count = 0;
        if tuning::CT_ASYNC {
            self.scrub.kick();
        }
    }
}
