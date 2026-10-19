// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The persistent compute and render graphs.
//!
//! The compute graph holds 22 x 4 MiB compute pools, the freelist (8192 seeds), the pool table,
//! the manager, scheduling context, notifier, threshold, job list, queue info/state, ring,
//! queue scratch and the user and firmware stamps. The render graph holds the render queues,
//! scheduling context, notifier, threshold, job list, the empty legacy record and InitBM. The
//! compute slots and the render pass slots are [`super::slots`]; the render UMA/PB allocators
//! and their spare set are [`super::pools`], initialized from [`Firmware::init_graphs`].
//! Classes per [`super::hw::tuning`]. Graph buffers with a client GVA are aliased into every
//! user VM ([`Request::client`]).
//!
//! The compute queue's Globals +0x9b0 = 0 (unchanged-work monitor off) is written with a waited
//! DEVCTL 0x13 before the queue is first exposed.

use kernel::{
    device,
    prelude::*,
    sync::{aref::ARef, Arc},
};

use super::channel::Channel;
use super::fw::graph::{job_list, manager, notifier, pools, qinfo, qinfo_row2, qstate, sched_ctx};
use super::hw::{client, engine, pte, sizes, tuning};
use super::object::{Class, FwObject, Objects, Placement, Request};
use super::Firmware;
use crate::driver::AsahiDevice;

/// Firmware class of firmware-maintained command objects.
pub(crate) const fn cmd_fw() -> Class {
    if tuning::FW_CACHED {
        Class::FwCached
    } else {
        Class::Fw
    }
}

/// The firmware + GPU twin of [`cmd_fw`].
pub(crate) const fn cmd_shared() -> Class {
    if tuning::FW_CACHED {
        Class::SharedCached
    } else {
        Class::Shared
    }
}

/// Notifiers: firmware-only, cached as selected by `tuning::NOTIFIER_CACHED` (bit 0 compute,
/// bit 1 render).
pub(crate) const fn notifier_class(bit: u32) -> Class {
    if tuning::NOTIFIER_CACHED & bit != 0 {
        Class::FwCached
    } else {
        Class::Fw
    }
}

/// Allocator managers: their index-2 class or its AttrIndex 0 twin.
pub(crate) const fn manager_class(class: Class) -> Class {
    if !tuning::MANAGER_CACHED {
        return class;
    }
    match class {
        Class::Shared => Class::SharedCached,
        _ => Class::FwCached,
    }
}

/// Queue scratch: firmware-private cached, or the shared index-2 class.
pub(crate) const fn queue_scratch() -> Class {
    if tuning::FW_CACHED {
        Class::FwCached
    } else {
        Class::Shared
    }
}

/// A client alias: the GVA of the page holding the payload and the leaf word.
#[derive(Copy, Clone)]
pub(crate) struct Client {
    pub(crate) page: u64,
    pub(crate) leaf: u64,
}

impl Client {
    /// GPU-only NC RW.
    pub(crate) const fn nc(page: u64) -> Option<Self> {
        Some(Self {
            page,
            leaf: pte::LEAF_NC,
        })
    }

    /// GPU-cached.
    pub(crate) const fn cached(page: u64) -> Option<Self> {
        Some(Self {
            page,
            leaf: pte::LEAF_CACHED,
        })
    }
}

/// Allocates a zeroed firmware object of `size` payload bytes at `offset` in its first page,
/// mapped in the firmware window with `class`, optionally aliased in every user VM and, with
/// `lower`, in the context-0 lower tree at the client page.
pub(crate) fn alloc(
    objects: &mut Objects,
    size: usize,
    class: Class,
    client: Option<Client>,
    offset: usize,
    lower: bool,
) -> Result<Arc<FwObject>> {
    objects.alloc(Request {
        size,
        class,
        placement: Placement::Anywhere,
        offset,
        client: client.map(|c| (c.page, c.leaf)),
        lower: if lower { client.map(|c| c.page) } else { None },
        cpu: true,
    })
}

/// The compute graph, without its slots.
pub(crate) struct ComputeGraph {
    /// The empty legacy record: allocated for the firmware's layout, never written.
    _legacy: Arc<FwObject>,
    pub(crate) pools: KVec<Arc<FwObject>>,
    pub(crate) freelist: Arc<FwObject>,
    pub(crate) pool_table: Arc<FwObject>,
    pub(crate) manager: Arc<FwObject>,
    pub(crate) counter: Arc<FwObject>,
    pub(crate) sched_ctx: Arc<FwObject>,
    pub(crate) notifier: Arc<FwObject>,
    pub(crate) threshold: Arc<FwObject>,
    pub(crate) job_list: Arc<FwObject>,
    pub(crate) qinfo: Arc<FwObject>,
    pub(crate) qstate: Arc<FwObject>,
    pub(crate) ring: Arc<FwObject>,
    pub(crate) qscratch: Arc<FwObject>,
    pub(crate) user_stamp: Arc<FwObject>,
    pub(crate) fw_stamp: Arc<FwObject>,
}

impl ComputeGraph {
    /// Allocates the compute graph objects.
    fn new(objects: &mut Objects) -> Result<Self> {
        let (sh, fw) = (Class::Shared, Class::Fw);
        let o = objects;

        let legacy = alloc(o, 0x4000, sh, None, 0, false)?;
        let mut pools = KVec::with_capacity(sizes::POOLS, GFP_KERNEL)?;
        for i in 0..sizes::POOLS as u64 {
            let pool = alloc(
                o,
                sizes::POOL_SIZE as usize,
                sh,
                Client::nc(client::pool(i)),
                0,
                false,
            )?;
            pools.push(pool, GFP_KERNEL)?;
        }
        Ok(Self {
            _legacy: legacy,
            pools,
            freelist: alloc(o, 0x400000, sh, Client::nc(client::FREELIST), 0, false)?,
            pool_table: alloc(
                o,
                0x1000,
                sh,
                Client::nc(client::POOL_TABLE_PAGE),
                client::POOL_TABLE_OFF as usize,
                false,
            )?,
            manager: alloc(o, manager::SIZE, manager_class(sh), None, 0, false)?,
            counter: alloc(o, 0x40, sh, None, 0, false)?,
            sched_ctx: alloc(o, sched_ctx::SIZE, cmd_fw(), None, 0, false)?,
            notifier: alloc(o, notifier::SIZE, notifier_class(1), None, 0, false)?,
            threshold: alloc(o, 4, fw, None, 0, false)?,
            job_list: alloc(o, job_list::SIZE, cmd_fw(), None, 0, false)?,
            qinfo: alloc(o, qinfo::SIZE, cmd_fw(), None, 0, false)?,
            qstate: alloc(o, qstate::SIZE, fw, None, 0, false)?,
            ring: alloc(o, sizes::QUEUE_CAP as usize * 8, fw, None, 0, false)?,
            qscratch: alloc(o, 0x40000, queue_scratch(), None, 0, false)?,
            user_stamp: alloc(o, 4, sh, None, 0, false)?,
            fw_stamp: alloc(o, 4, sh, None, 0, false)?,
        })
    }

    /// Initial contents; the firmware owns them afterwards. The slots are scrubbed by
    /// [`super::slots`].
    fn init(&self) -> Result {
        let mut entry = 0u64;

        // Freelist: the first eight 4 MiB pools in 0x1000 units.
        for pool in self.pools.iter().take(pools::SEEDED_POOLS) {
            let gva = pool.gva.ok_or(EINVAL)?;
            let mut off = 0;
            while off < sizes::POOL_SIZE {
                self.freelist.w64(8 * entry as usize, gva + off)?;
                entry += 1;
                off += pools::SEED_BYTES;
            }
        }
        for (k, pool) in self.pools.iter().enumerate() {
            let gva = pool.gva.ok_or(EINVAL)?;
            self.pool_table
                .w64(pools::TABLE_STRIDE * k, gva | pools::TABLE_VALID)?;
        }
        let m = &self.manager;
        m.w32(manager::ID, manager::COMPUTE_ID)?;
        m.w32(manager::INDEX, manager::COMPUTE_INDEX)?;
        m.w32(manager::ONE, 1)?;
        m.w64(manager::FREELIST, self.freelist.gva.ok_or(EINVAL)?)?;
        m.w32(manager::UNK_1C, manager::UNK_1C_VALUE)?;
        m.w32(manager::SEEDS_A, sizes::FREELIST_SEED)?;
        m.w32(manager::SEEDS_B, sizes::FREELIST_SEED)?;
        m.w64(manager::POOL_TABLE, self.pool_table.gva.ok_or(EINVAL)?)?;
        m.w32(manager::UNK_44, manager::UNK_44_VALUE)?;
        m.w64(manager::COUNTER, self.counter.va)?;
        self.counter.w32(0, 1)?;

        init_sched_ctx(&self.sched_ctx)?;
        init_notifier(&self.notifier, &self.threshold)?;
        self.job_list.w64(job_list::SELF, self.job_list.va)?;
        self.threshold.w32(0, 1)?;

        let q = &self.qinfo;
        q.w64(qinfo::STATE, self.qstate.va)?;
        q.w64(qinfo::RING, self.ring.va)?;
        q.w64(qinfo::JOB_LIST, self.job_list.va)?;
        q.w64(qinfo::SCRATCH, self.qscratch.va)?;
        q.w32(qinfo::UNK_2C, 0xffffffff)?;
        q.w64(qinfo_row2::MASK, qinfo_row2::ROW0_MASK_VALUE)?;
        q.w32(qinfo_row2::UNK_40, 1)?;
        q.w32(qinfo_row2::UNK_48, 1)?;
        if tuning::COMPUTE_QUEUE_CLASS == 2 {
            // Row 2: 2, 2, ffff000000000000, 0, 0, 2.
            q.w32(qinfo_row2::CLASS, 2)?;
            q.w32(qinfo_row2::CLASS_B, 2)?;
            q.w64(qinfo_row2::MASK, qinfo_row2::MASK_VALUE)?;
            q.w32(qinfo_row2::UNK_40, 0)?;
            q.w32(qinfo_row2::UNK_44, 0)?;
            q.w32(qinfo_row2::UNK_48, 2)?;
        }
        q.w32(qinfo::UNK_4C, 0xffffffff)?;
        q.w32(qinfo::UUID, engine::COMPUTE_UUID)?;
        q.w64(qinfo::SCHED_CTX, self.sched_ctx.va)?;
        q.w64(qinfo::UNK_AC, 4)?;
        self.qstate.w32(qstate::CAP, sizes::QUEUE_CAP)?;

        self.user_stamp.w32(0, engine::COMPUTE_SEED)?;
        self.fw_stamp.w32(0, 0)
    }
}

/// The render graph, without the pass slots, the clustering buffers and the render
/// allocators.
pub(crate) struct RenderGraph {
    /// TA (0) and FS (1) queues.
    pub(crate) qinfo: [Arc<FwObject>; 2],
    pub(crate) qstate: [Arc<FwObject>; 2],
    pub(crate) ring: [Arc<FwObject>; 2],
    pub(crate) qscratch: [Arc<FwObject>; 2],
    pub(crate) user_stamp: [Arc<FwObject>; 2],
    pub(crate) fw_stamp: [Arc<FwObject>; 2],
    pub(crate) sched_ctx: Arc<FwObject>,
    pub(crate) notifier: Arc<FwObject>,
    pub(crate) threshold: Arc<FwObject>,
    pub(crate) job_list: Arc<FwObject>,
    pub(crate) legacy_empty: Arc<FwObject>,
    pub(crate) initbm: Arc<FwObject>,
}

/// One render engine's queue objects, allocated together.
struct RenderQueue {
    qinfo: Arc<FwObject>,
    qstate: Arc<FwObject>,
    ring: Arc<FwObject>,
    qscratch: Arc<FwObject>,
    user_stamp: Arc<FwObject>,
    fw_stamp: Arc<FwObject>,
}

impl RenderQueue {
    fn new(o: &mut Objects) -> Result<Self> {
        Ok(Self {
            qinfo: alloc(o, qinfo::SIZE, cmd_fw(), None, 0, false)?,
            qstate: alloc(o, qstate::SIZE, Class::Fw, None, 0, false)?,
            ring: alloc(o, sizes::QUEUE_CAP as usize * 8, Class::Fw, None, 0, false)?,
            qscratch: alloc(o, 0x40000, queue_scratch(), None, 0, false)?,
            user_stamp: alloc(o, 4, Class::Shared, None, 0, false)?,
            fw_stamp: alloc(o, 4, Class::Shared, None, 0, false)?,
        })
    }
}

impl RenderGraph {
    /// Allocates the render graph objects.
    fn new(o: &mut Objects) -> Result<Self> {
        let [ta, fs] = [RenderQueue::new(o)?, RenderQueue::new(o)?];
        let sched_ctx = alloc(o, sched_ctx::SIZE, cmd_fw(), None, 0, false)?;
        let notifier = alloc(o, notifier::SIZE, notifier_class(2), None, 0, false)?;
        let threshold = alloc(o, 4, Class::Fw, None, 0, false)?;
        let job_list = alloc(o, job_list::SIZE, cmd_fw(), None, 0, false)?;
        let legacy_empty = alloc(o, 0x40, Class::Shared, None, 0, false)?;
        let initbm = alloc(o, super::fw::work::initbm::SIZE, cmd_fw(), None, 0, false)?;
        Ok(Self {
            qinfo: [ta.qinfo, fs.qinfo],
            qstate: [ta.qstate, fs.qstate],
            ring: [ta.ring, fs.ring],
            qscratch: [ta.qscratch, fs.qscratch],
            user_stamp: [ta.user_stamp, fs.user_stamp],
            fw_stamp: [ta.fw_stamp, fs.fw_stamp],
            sched_ctx,
            notifier,
            threshold,
            job_list,
            legacy_empty,
            initbm,
        })
    }

    /// Initial contents; the render allocators are initialized separately.
    fn init(&self) -> Result {
        let uuid = [engine::TA_UUID, engine::FS_UUID];
        let seed = [engine::TA_SEED, engine::FS_SEED];

        for e in 0..2 {
            let q = &self.qinfo[e];
            q.w64(qinfo::STATE, self.qstate[e].va)?;
            q.w64(qinfo::RING, self.ring[e].va)?;
            q.w64(qinfo::JOB_LIST, self.job_list.va)?;
            q.w64(qinfo::SCRATCH, self.qscratch[e].va)?;
            q.w32(qinfo::UNK_2C, 0xffffffff)?;
            if tuning::RENDER_QUEUE_CLASS == 2 {
                // Recognized row 2: class/policy 2, mask, +0x40/+0x44 0.
                q.w32(qinfo_row2::CLASS, 2)?;
                q.w32(qinfo_row2::CLASS_B, 2)?;
                q.w64(qinfo_row2::MASK, qinfo_row2::MASK_VALUE)?;
                q.w32(qinfo_row2::UNK_48, 2)?;
            } else {
                q.w64(qinfo_row2::MASK, qinfo_row2::ROW0_MASK_VALUE)?;
                q.w32(qinfo_row2::UNK_40, 1)?;
                q.w32(qinfo_row2::UNK_48, 1)?;
            }
            q.w32(qinfo::UNK_4C, 0xffffffff)?;
            q.w32(qinfo::UUID, uuid[e])?;
            q.w64(qinfo::SCHED_CTX, self.sched_ctx.va)?;
            q.w64(qinfo::UNK_AC, 4)?;
            self.qstate[e].w32(qstate::CAP, sizes::QUEUE_CAP)?;
            self.ring[e].fill(0, 0xcc, sizes::QUEUE_CAP as usize * 8)?;
            self.user_stamp[e].w32(0, seed[e])?;
            self.fw_stamp[e].w32(0, seed[e])?;
        }
        init_sched_ctx(&self.sched_ctx)?;
        init_notifier(&self.notifier, &self.threshold)?;
        self.job_list.w64(job_list::SELF, self.job_list.va)?;
        self.threshold.w32(0, 2)
    }
}

/// Scheduling context initial contents, shared by both graphs.
fn init_sched_ctx(s: &FwObject) -> Result {
    s.fill(0x00, 0xff, 2)?;
    s.fill(0x05, 0x01, 1)?;
    s.fill(0x1e, 0xff, 1)?;
    s.fill(0x23, 0x02, 1)
}

/// Notifier initial contents: the only host stores the notifier ever gets.
fn init_notifier(n: &FwObject, threshold: &FwObject) -> Result {
    n.w64(notifier::THRESHOLD, threshold.va)?;
    n.w32(notifier::UNK_10, notifier::UNK_10_VALUE)?;
    n.fill(notifier::MASK, 0xff, 8)
}

/// The compute and render graph objects.
pub(crate) struct Graphs {
    /// The GPU device, for the engine's log messages.
    pub(crate) dev: ARef<device::Device>,
    pub(crate) compute: ComputeGraph,
    pub(crate) render: RenderGraph,
}

impl Graphs {
    /// Allocates both graphs.
    pub(crate) fn new(dev: &AsahiDevice, objects: &mut Objects) -> Result<Self> {
        Ok(Self {
            dev: dev.as_ref().into(),
            compute: ComputeGraph::new(objects)?,
            render: RenderGraph::new(objects)?,
        })
    }
}

impl Firmware {
    /// Initializes both graphs, the slots and the render allocators, then turns the compute
    /// monitor off before the queues are first exposed.
    pub(crate) fn init_graphs(&mut self) -> Result {
        // TA/FS/compute channels must start empty.
        for ch in [Channel::Ta, Channel::Fs, Channel::Compute] {
            if self.channels.read(ch)? != 0 || self.channels.write_cell(ch)? != 0 {
                return Err(Error::from_errno(-(kernel::bindings::EPROTO as i32)));
            }
        }
        self.graph.compute.init()?;
        self.slots.init()?;
        super::engine::wmb();
        self.graph.render.init()?;
        self.init_render_pools(false)?;
        // New firmware and context-0 translations before any reference.
        super::uat::tlbi_all();
        dev_info!(
            self.graph.dev.as_ref(),
            "G15: compute graph: work {:#x}/{:#x} pools {:#x} freelist {:#x} table {:#x}\n",
            self.slots.scrub.compute[0].work.va,
            self.slots.scrub.compute[0].work.gva.unwrap_or(0),
            self.graph.compute.pools[0].gva.unwrap_or(0),
            self.graph.compute.freelist.gva.unwrap_or(0),
            self.graph.compute.pool_table.gva.unwrap_or(0)
        );
        if tuning::COMPUTE_QUEUE_CLASS == 2 {
            // Class-2 compute profile: the unchanged-work compute monitor (Globals +0x9b0)
            // off before the queue's first exposure.
            self.compute_monitor_off()?;
        }
        Ok(())
    }
}
