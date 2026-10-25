// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Render UMA/PB allocators, their spare set and parameter-buffer growth.
//!
//! Record layouts: [`super::fw::buffer`]; the ledger and request checks: [`growth`]
//! (host-tested).
//!
//! - One 512 MiB PB heap at [`super::hw::client::PB_HEAP`] (4096 blocks of 128 KiB), backed on
//!   demand in chunks of [`growth::CHUNK_BLOCKS`] blocks (16 MiB) from the heap base: when the
//!   GPU starts, the chunks of the initial posting and one reserve chunk
//!   ([`growth::Ledger::chunks_for`]). The backing ([`PbChunk`]) is mapped only into user VMs, at
//!   the fixed client GVA: each VM's before its prefixes are published ([`Firmware::map_pb`]),
//!   and a chunk backed later also into the VM of the flights; nothing names its pages through
//!   the host window or context 0 (the records hold client GVAs only). Initial
//!   `g15_pb_blocks` (64), at most [`super::hw::tuning::PB_MAX_BLOCKS`] (4095) posted, so the
//!   producer never returns to 0; the firmware is given [`Pools::soft_max`] (default 3072) as
//!   its growth maximum. Truthful K = 4096 in every
//!   profile (PB reconstruction, also in recovery, synchronizes 8 * K bytes of the block list:
//!   K = 0x26cd overran the 0x8000-byte list and crashed the second recovery).
//! - Backing: a request is granted from the backed blocks above the posted count; after the
//!   replies of a pass, while fewer than a reserve chunk remain, [`Firmware::refill_backing`]
//!   backs the next chunk, so a reply never waits for the allocator. Only a reserve that ran dry
//!   makes a request wait: it stays queued, the pass backs a chunk and the next pass answers it.
//! - Reclaim ([`Firmware::reclaim_check`], `g15_pb_reclaim`): posted blocks never come back
//!   while a PB lives (the firmware recycles pages inside its own pool; the host has no view of
//!   it), so an engine idle for `g15_pb_idle_ms` with more than `g15_pb_reclaim_blocks` posted
//!   re-initializes the PB at its next render through the re-initialization path of a recovered
//!   abort (fresh lists and manager, InitBM of the initial blocks, no halt) and frees the backing
//!   above the kept chunks (also with fewer posted, when more is backed than they need); an
//!   engine idle for 50 ms (a client boundary) with more than half the maximum posted
//!   re-initializes it the same way but keeps the backing; if no render follows, with
//!   `g15_pb_reclaim` 2 the backing above the kept chunks is freed after `g15_pb_idle_ms`.
//! - Growth: the event drain queues type-7 requests; [`Firmware::answer_growth`] answers each
//!   one in the poll pass: type 7, PB slot 0, the context of a live published render prefix,
//!   counter >= next expected, epoch == current halt count; write records `(G - 0x1000000000)
//!   >> 15` at producer % K, barrier, block_ctl+4, barrier, then DEVCTL 7 with +4 = 1 echoing
//!   slot, context, counter, position and epoch; +10 blocks per request. Above the firmware's
//!   growth maximum the requests are answered as before, for the synchronous growth of partial
//!   renders (Mesa's partial background/EOT programs). At the ledger's maximum the reply is +4
//!   = 0, a refusal, twice per initialization; later requests are left unanswered. A refusal
//!   gives no pages and can leave the TA stalled; the poll pass returns, and the host deadline
//!   and its recovery path apply, without a guarantee that they end the stall. Invalid requests
//!   are logged and left unanswered, so the host deadline applies. Recovery drops pending
//!   requests (epoch gate).
//! - Managers ASC-cached with an uncached spare set allocated with the graph; the first
//!   recovered render abort swaps to the spare and never rewrites the cached set (the ASC may
//!   still hold dirty lines of it); later aborts rewrite the spare in place. block_ctl stays
//!   uncached when growth is on (a live host producer).
//!
//! The engine uses: [`Firmware::init_render_pools`] (render graph init: `false`; the priming
//! prefix after an abort: `true` when [`super::recovery::Recovery::reinit`] is set), the VAs
//! for its encoders ([`Pools::allocator_refs`]), [`Firmware::publish_pb`] after the render
//! threshold, and [`Pools::write_stats`] for the debugfs `pb:` line.

mod growth;

use core::fmt;
use kernel::{
    device,
    prelude::*,
    sync::{aref::ARef, Arc},
};

use super::fw::buffer::{self, block_ctl, counter, misc, pb_manager, uma_manager, uma_pools};
use super::fw::channels::DevCtl;
use super::fw::initdata::control;
use super::hw::{client, pte, sizes, tuning};
use super::object::{self, Class, FwObject, HeldPages, Objects, Placement, Request};
use super::vm::VmState;
use super::Firmware;
use crate::driver::AsahiDrmDevice as AsahiDevice;
use crate::{gem, mmu};

pub(crate) use growth::{Answer, IdleClock, Ledger};

static_assert!(buffer::MAX_BLOCKS == sizes::PB_BLOCKS);
static_assert!(buffer::MAX_PAGES == sizes::PB_PAGES);
static_assert!(buffer::BLOCK_LIST_RECORDS == sizes::PB_LIST_RECORDS);
static_assert!(buffer::GROW_STEP == sizes::PB_GROW_STEP);
static_assert!(buffer::MAX_BLOCKS as u64 * buffer::BLOCK_BYTES == client::PB_HEAP_SIZE);
static_assert!(buffer::PAGE_LIST_SIZE as u64 == client::PB_PAGE_LIST_SIZE);
static_assert!(uma_manager::SEEDS == sizes::FREELIST_SEED);
static_assert!(uma_pools::POOL_SIZE == sizes::POOL_SIZE);
// The ledger keeps the driver's PB maximum: it is not capped by [`growth::MAX_POSTED`]. The
// initial posting (`g15_pb_blocks`) is clamped to MIN_BLOCKS..=PB_MAX_BLOCKS.
static_assert!(tuning::PB_MAX_BLOCKS <= growth::MAX_POSTED);
static_assert!(buffer::MIN_BLOCKS <= tuning::PB_MAX_BLOCKS);
// The PB alias is GPU-only (not cached and firmware-visible): dropping one is a plain unmap,
// with no firmware cache flush (`KernelMapping::drop`).
static_assert!(!mmu::Prot::from_pte(pte::LEAF_CACHED).is_cached_noncoherent());
static_assert!(growth::MAX_CHUNKS * growth::CHUNK_BLOCKS == buffer::MAX_BLOCKS);

/// Bytes of one backing chunk.
pub(crate) const CHUNK_BYTES: usize = growth::CHUNK_BLOCKS as usize * buffer::BLOCK_BYTES as usize;

/// Client GVA of backing chunk `i`.
pub(crate) const fn chunk_gva(i: usize) -> u64 {
    client::PB_HEAP + (i * CHUNK_BYTES) as u64
}

/// Physical backing of one chunk of PB blocks: a zeroed GEM kernel object of [`CHUNK_BYTES`],
/// pinned and DMA-mapped ([`object::gpu_backing`]), aliased only into user VMs at its fixed
/// client GVA ([`chunk_gva`], [`VmState::map_heap`]). Each alias also holds the object.
struct PbChunk {
    _pages: HeldPages,
    backing: gem::ObjectRef,
}

impl PbChunk {
    fn new(dev: &AsahiDevice) -> Result<Self> {
        let (backing, pages) = object::gpu_backing(dev, CHUNK_BYTES)?;
        Ok(Self {
            _pages: pages,
            backing,
        })
    }

    /// The GEM object, for the VM mappings.
    fn gem(&self) -> ARef<gem::Object> {
        self.backing.gem.clone()
    }
}

/// The allocator state a rebuild after an abort moves to the spare.
struct AllocatorSet {
    /// TA (0) and FS (1) UMA managers and their counters.
    uma_manager: [Arc<FwObject>; 2],
    uma_counter: [Arc<FwObject>; 2],
    block_ctl: Arc<FwObject>,
    pb_counter: Arc<FwObject>,
    misc: Arc<FwObject>,
    scene_list: Arc<FwObject>,
    pb_manager: Arc<FwObject>,
}

/// One render engine's UMA pools: eight GPU-only 4 MiB pools, the freelist seeded with them and
/// the pool table.
struct UmaEngine {
    pools: KVec<Arc<FwObject>>,
    freelist: Arc<FwObject>,
    pool_table: Arc<FwObject>,
}

/// Firmware VAs of the render allocators in use (after a rebuild possibly the spare set), as
/// the TA/FS images, the scene and InitBM records name them.
#[derive(Copy, Clone, Default)]
pub(crate) struct AllocatorRefs {
    /// UMA managers of TA (0) and FS (1).
    pub(crate) uma_mgr: [u64; 2],
    /// PB manager (InitBM +0x14, TA/FS work and sequences), scene list (scene +0x0/+0x8 at
    /// `+ 4 * (draw % 48)`, TA work) and misc record (scene +0x40).
    pub(crate) pb_mgr: u64,
    pub(crate) scene_list: u64,
    pub(crate) mgr_misc: u64,
    /// Initial PB blocks of the current initialization (InitBM +0x10, TA register #33).
    pub(crate) init_blocks: u32,
}

/// PB growth statistics.
#[derive(Default)]
pub(crate) struct PbStats {
    pub(crate) requests: u64,
    pub(crate) grown: u64,
    pub(crate) refused: u64,
    pub(crate) bad: u64,
    /// Requests that waited a pass for backing.
    pub(crate) waits: u64,
    /// Chunks backed after the GPU started.
    pub(crate) refills: u64,
    /// Idle re-initializations, and the chunks they freed.
    pub(crate) reclaims: u64,
    pub(crate) freed: u64,
}

/// Render allocator and PB state: the allocators, the PB ledger and the spare set.
pub(crate) struct Pools {
    /// The GPU device, for logging (also used by recovery).
    pub(crate) dev: ARef<device::Device>,
    /// The DRM device, for the PB backing objects.
    drm: ARef<AsahiDevice>,
    uma: [UmaEngine; 2],
    /// The allocator set in use.
    live: AllocatorSet,
    /// The never-exposed uncached set the first rebuild moves to; afterwards the replaced
    /// cached set, retained (only with `manager_cached`).
    spare: Option<AllocatorSet>,
    /// The spare set has not been used yet.
    spare_ready: bool,
    pb_pages: Arc<FwObject>,
    pb_blocks: Arc<FwObject>,
    /// The PB backing chunks, from the heap base (chunk `i` at [`chunk_gva`]`(i)`). Its
    /// capacity, reserved up front, is the whole heap's [`growth::MAX_CHUNKS`].
    chunks: KVec<PbChunk>,
    /// The PB growth ledger since the last (re)initialization.
    pub(crate) ledger: Ledger,
    /// C1 names the current initial pages.
    c1_written: bool,
    /// The unanswered-at-the-maximum warning was given since the last initialization.
    exhausted_logged: bool,
    /// The engine's idle clock (nothing published, nothing pending), for the reclaim.
    idle: IdleClock,
    /// The pending re-initialization was set by a reclaim ([`Firmware::reclaim_check`]), not by
    /// a recovery; cleared by [`Self::init`].
    reclaim_pending: bool,
    pub(crate) stats: PbStats,
}

/// The ASC-cached twin of an allocator manager's class.
const fn manager_class(class: Class) -> Class {
    if !tuning::MANAGER_CACHED {
        return class;
    }
    match class {
        Class::Shared => Class::SharedCached,
        _ => Class::FwCached,
    }
}

/// Allocates a zeroed firmware object anywhere in the firmware window: `client` names the
/// fixed client page and leaf of its alias in every VM, `cpu` is false for GPU-only storage.
fn alloc(
    objects: &mut Objects,
    size: u64,
    class: Class,
    client: Option<(u64, u64)>,
    offset: u64,
    cpu: bool,
) -> Result<Arc<FwObject>> {
    objects.alloc(Request {
        size: size as usize,
        class,
        placement: Placement::Anywhere,
        offset: offset as usize,
        client,
        lower: None,
        cpu,
    })
}

/// Writes `count` little-endian records of `W` bytes, record `i` at `offset + i * W`, in chunks.
fn write_records<const W: usize>(
    object: &FwObject,
    offset: usize,
    count: usize,
    record: impl Fn(usize) -> [u8; W],
) -> Result {
    const CHUNK: usize = 512;
    let mut buf = [0u8; CHUNK];
    let per_chunk = CHUNK / W;
    let mut i = 0;
    while i < count {
        let n = per_chunk.min(count - i);
        for j in 0..n {
            buf[j * W..(j + 1) * W].copy_from_slice(&record(i + j));
        }
        object.write(offset + i * W, &buf[..n * W])?;
        i += n;
    }
    Ok(())
}

/// A u64 record.
fn le64(value: u64) -> [u8; 8] {
    value.to_le_bytes()
}

impl UmaEngine {
    /// Allocates engine `e`'s UMA pools, freelist and pool table, then its manager and counter,
    /// in this order.
    fn alloc(objects: &mut Objects, e: u64) -> Result<(Self, Arc<FwObject>, Arc<FwObject>)> {
        let mut pools = KVec::with_capacity(uma_pools::POOLS, GFP_KERNEL)?;
        for k in 0..uma_pools::POOLS as u64 {
            pools.push(
                alloc(
                    objects,
                    uma_pools::POOL_SIZE,
                    Class::Shared,
                    Some((client::uma(e, k), pte::LEAF_NC)),
                    0,
                    false,
                )?,
                GFP_KERNEL,
            )?;
        }
        let freelist = alloc(
            objects,
            uma_pools::FREELIST_SIZE as u64,
            Class::Shared,
            Some((client::uma_freelist(e), pte::LEAF_NC)),
            0,
            true,
        )?;
        let pool_table = alloc(
            objects,
            uma_pools::POOL_TABLE_SIZE as u64,
            Class::Shared,
            Some((client::uma_table_page(e), pte::LEAF_NC)),
            uma_pools::POOL_TABLE_OFFSET as u64,
            true,
        )?;
        let manager = alloc(
            objects,
            uma_manager::SIZE as u64,
            manager_class(Class::Shared),
            None,
            0,
            true,
        )?;
        let counter = alloc(objects, counter::SIZE as u64, Class::Shared, None, 0, true)?;
        Ok((
            Self {
                pools,
                freelist,
                pool_table,
            },
            manager,
            counter,
        ))
    }
}

/// The client GVA of an object with a client alias.
fn gva(object: &FwObject) -> Result<u64> {
    object.gva.ok_or(EINVAL)
}

impl Pools {
    /// Allocates the render allocator objects: per engine the UMA pools, freelist, pool table,
    /// manager and counter; block control, PB counter, misc, scene list and PB manager; the
    /// uncached spare set; the PB page list, block list and heap. Contents are written by
    /// [`Firmware::init_render_pools`].
    pub(crate) fn new(dev: &AsahiDevice, objects: &mut Objects) -> Result<Self> {
        let (uma0, manager0, counter0) = UmaEngine::alloc(objects, 0)?;
        let (uma1, manager1, counter1) = UmaEngine::alloc(objects, 1)?;
        let small = |objects: &mut Objects, size: usize, class| {
            alloc(objects, size as u64, class, None, 0, true)
        };
        // A live producer with growth: firmware index 2.
        let block_ctl_class = if tuning::PB_GROW {
            Class::Fw
        } else {
            manager_class(Class::Fw)
        };
        let block_ctl = small(objects, block_ctl::SIZE, block_ctl_class)?;
        let pb_counter = small(objects, counter::SIZE, Class::Fw)?;
        let misc = small(objects, misc::SIZE, manager_class(Class::Fw))?;
        let scene_list = small(objects, buffer::SCENE_LIST_SIZE, Class::Shared)?;
        let pb_manager = small(objects, pb_manager::SIZE, manager_class(Class::Fw))?;
        let live = AllocatorSet {
            uma_manager: [manager0, manager1],
            uma_counter: [counter0, counter1],
            block_ctl,
            pb_counter,
            misc,
            scene_list,
            pb_manager,
        };
        // The uncached spare set for the first rebuild after an abort (manager_cached).
        let spare = if tuning::MANAGER_CACHED {
            let m0 = small(objects, uma_manager::SIZE, Class::Shared)?;
            let c0 = small(objects, counter::SIZE, Class::Shared)?;
            let m1 = small(objects, uma_manager::SIZE, Class::Shared)?;
            let c1 = small(objects, counter::SIZE, Class::Shared)?;
            Some(AllocatorSet {
                uma_manager: [m0, m1],
                uma_counter: [c0, c1],
                block_ctl: small(objects, block_ctl::SIZE, Class::Fw)?,
                pb_counter: small(objects, counter::SIZE, Class::Fw)?,
                misc: small(objects, misc::SIZE, Class::Fw)?,
                scene_list: small(objects, buffer::SCENE_LIST_SIZE, Class::Shared)?,
                pb_manager: small(objects, pb_manager::SIZE, Class::Fw)?,
            })
        } else {
            None
        };
        let pb_pages = alloc(
            objects,
            buffer::PAGE_LIST_SIZE as u64,
            Class::Shared,
            Some((client::PB_PAGE_LIST, pte::LEAF_PB_LIST)),
            0,
            true,
        )?;
        let pb_blocks = alloc(
            objects,
            buffer::BLOCK_LIST_SIZE as u64,
            Class::Shared,
            Some((client::PB_BLOCK_LIST, pte::LEAF_PB_LIST)),
            0,
            true,
        )?;
        // Backing for the initial blocks and the reserve; growth adds the rest.
        let mut ledger = Ledger::new(
            tuning::PB_GROW,
            super::params::pb_blocks(),
            tuning::PB_MAX_BLOCKS,
        );
        let mut chunks = KVec::with_capacity(growth::MAX_CHUNKS as usize, GFP_KERNEL)?;
        for _ in 0..Ledger::chunks_for(ledger.initial) {
            chunks.push(PbChunk::new(dev)?, GFP_KERNEL)?;
            ledger.back_chunk();
        }
        dev_info!(
            dev.as_ref(),
            "G15: PB heap: {} blocks posted at start, {} chunks of {} MiB backed\n",
            ledger.initial,
            chunks.len(),
            CHUNK_BYTES >> 20
        );
        Ok(Self {
            dev: dev.as_ref().as_ref().into(),
            drm: dev.into(),
            uma: [uma0, uma1],
            live,
            spare_ready: spare.is_some(),
            spare,
            pb_pages,
            pb_blocks,
            chunks,
            ledger,
            c1_written: false,
            exhausted_logged: false,
            idle: IdleClock::default(),
            reclaim_pending: false,
            stats: PbStats::default(),
        })
    }

    /// First rebuild after an abort with cached managers: the cached set may still have dirty
    /// lines in the ASC, so it is retained untouched and the graph moves to the never-exposed
    /// uncached spare (and its own counters and scene list), which [`Self::init`] then
    /// initializes.
    fn take_spare(&mut self) {
        if let Some(spare) = self.spare.as_mut() {
            core::mem::swap(&mut self.live, spare);
        }
        self.spare_ready = false;
        dev_info!(
            self.dev,
            "render allocators moved to a fresh uncached set; the cached set is retained\n"
        );
    }

    /// Writes the initial allocator state: UMA pools, freelists and managers; the PB lists,
    /// manager and counters. The firmware owns them once published; rewritten (zeroed first)
    /// only after a recovered render abort, followed by a fresh InitBM.
    fn init(&mut self) -> Result {
        for e in 0..2 {
            let uma = &self.uma[e];
            let manager = &self.live.uma_manager[e];
            let count = &self.live.uma_counter[e];
            manager.fill(0, 0, uma_manager::SIZE)?;
            count.fill(0, 0, counter::SIZE)?;
            // Freelist: the eight 4 MiB pools in 0x1000 units.
            let per_pool = (uma_pools::POOL_SIZE / uma_pools::SEED_BYTES) as usize;
            let mut bases = [0u64; uma_pools::POOLS];
            for (k, pool) in uma.pools.iter().enumerate() {
                bases[k] = gva(pool)?;
            }
            write_records(&uma.freelist, 0, uma_pools::POOLS * per_pool, |i| {
                le64(bases[i / per_pool] + (i % per_pool) as u64 * uma_pools::SEED_BYTES)
            })?;
            for (k, base) in bases.iter().enumerate() {
                uma.pool_table
                    .w64(uma_pools::POOL_TABLE_STRIDE * k, uma_pools::row(*base))?;
            }
            manager.write(
                0,
                &buffer::uma_manager_image(
                    e as u32 + 1,
                    e as u32,
                    gva(&uma.freelist)?,
                    gva(&uma.pool_table)?,
                    count.va,
                ),
            )?;
            count.w32(0, 1)?;
        }

        // PB: 16384 pages of 0x8000 in 4096 blocks of 0x20000; with growth the first initial
        // blocks of them, at most 4095 posted, and page limits at the firmware's growth maximum.
        // The backing is kept. In every profile the block list declares its real 4096-record
        // capacity.
        self.ledger.reinit();
        self.exhausted_logged = false;
        self.reclaim_pending = false;
        if tuning::PB_GROW {
            // C1 names the initial pages again.
            self.c1_written = false;
        }
        let set = &self.live;
        set.pb_manager.fill(0, 0, pb_manager::SIZE)?;
        set.block_ctl.fill(0, 0, block_ctl::SIZE)?;
        set.misc.fill(0, 0, misc::SIZE)?;
        set.pb_counter.fill(0, 0, counter::SIZE)?;
        set.scene_list.fill(0, 0, buffer::SCENE_LIST_SIZE)?;
        if tuning::PB_GROW {
            self.pb_pages.fill(0, 0, buffer::PAGE_LIST_SIZE)?;
            self.pb_blocks.fill(0, 0, buffer::BLOCK_LIST_SIZE)?;
        }
        let heap = client::PB_HEAP;
        let initial = self.ledger.initial;
        if initial > self.ledger.backed {
            return Err(EINVAL);
        }
        write_records(
            &self.pb_pages,
            0,
            (buffer::PAGES_PER_BLOCK * initial) as usize,
            |i| buffer::page_record(heap, i as u32).to_le_bytes(),
        )?;
        // The high word of every block record is zero (zero-filled, or only ever written 0).
        write_records(&self.pb_blocks, 0, initial as usize, |i| {
            le64(buffer::block_record(heap, i as u32) as u64)
        })?;
        set.pb_manager.write(
            0,
            // The maximum the firmware sees is the asynchronous growth limit, below the mapped
            // heap: the blocks above it answer the synchronous requests a partial render makes
            // (the firmware cannot finish a scene at its maximum without them; refused, its TA
            // stalls and no kill ends it: ticket malik-na/linux-aurora#69).
            &buffer::pb_manager_image(
                initial,
                self.soft_max(),
                gva(&self.pb_pages)?,
                gva(&self.pb_blocks)?,
                set.block_ctl.va,
                set.pb_counter.va,
            ),
        )?;
        set.block_ctl.write(0, &buffer::block_ctl_image(initial))?;
        set.misc.w32(misc::ENABLE, 1)?;
        set.pb_counter.w32(0, 1)?;
        crate::mem::sync();
        Ok(())
    }

    /// The allocator addresses the render encoders name (the live set: after the first
    /// rebuild, the spare).
    pub(crate) fn allocator_refs(&self) -> AllocatorRefs {
        let set = &self.live;
        AllocatorRefs {
            uma_mgr: [set.uma_manager[0].va, set.uma_manager[1].va],
            pb_mgr: set.pb_manager.va,
            scene_list: set.scene_list.va,
            mgr_misc: set.misc.va,
            init_blocks: self.ledger.initial,
        }
    }

    /// The firmware's growth maximum in blocks (`g15_pb_max_blocks`), at least the initial size
    /// and at most the mapped heap.
    pub(crate) fn soft_max(&self) -> u32 {
        super::params::pb_max_blocks().clamp(self.ledger.initial, self.ledger.max)
    }

    /// Maps the backed chunks `vm` does not hold yet ([`VmState::map_heap`]).
    fn map_heap(&self, vm: &VmState) -> Result {
        vm.map_heap(self.chunks.iter().map(PbChunk::gem))
    }

    /// Frees the backing chunks from index `keep` on (their VM mappings are dropped by the
    /// caller); the RAM returns when the last mapping of a chunk is gone.
    fn free_chunks(&mut self, keep: usize) -> usize {
        let freed = self.chunks.len().saturating_sub(keep);
        self.chunks.truncate(keep);
        self.ledger.unback_to(self.chunks.len() as u32);
        freed
    }

    /// Backs one more chunk; the caller maps it where needed before the ledger posts from it.
    fn back_chunk(&mut self) -> Result<ARef<gem::Object>> {
        if self.chunks.len() >= growth::MAX_CHUNKS as usize {
            return Err(ENOSPC);
        }
        let chunk = PbChunk::new(&self.drm)?;
        let gem = chunk.gem();
        self.chunks.push(chunk, GFP_KERNEL)?;
        Ok(gem)
    }

    /// The debugfs line `pb: blocks initial/posted/firmware-max/max backed blocks/chunks
    /// requests grown refused bad waits refills reclaims freed` (new keys are appended).
    pub(crate) fn write_stats(&self, f: &mut impl fmt::Write) -> fmt::Result {
        writeln!(
            f,
            "pb: blocks {}/{}/{}/{} backed {}/{} requests {} grown {} refused {} bad {} waits {} refills {} reclaims {} freed {}",
            self.ledger.initial,
            self.ledger.posted,
            self.soft_max(),
            self.ledger.max,
            self.ledger.backed,
            self.chunks.len(),
            self.stats.requests,
            self.stats.grown,
            self.stats.refused,
            self.stats.bad,
            self.stats.waits,
            self.stats.refills,
            self.stats.reclaims,
            self.stats.freed
        )
    }
}

impl Firmware {
    /// Maps the PB backing into `vm`, before anything of a prefix of the VM is published (the
    /// start of `publish_prefix_into`). Render work runs in one VM at a time
    /// (`Engine::publish`), so the VM about to run is the only one through which the GPU can
    /// reach a posted block next; a VM created or idle while the backing grew catches up here,
    /// also a closed VM whose accepted jobs still run. A chunk is mapped only once
    /// ([`VmState::map_heap`]); one that fails leaves the chunks mapped before it, and the
    /// prefix is not published. The leaf is GPU-only, so dropping an alias is a plain unmap.
    ///
    /// The one-VM rule is checked first: a flight of another VM fails with EPROTO before
    /// anything is mapped.
    pub(crate) fn map_pb(&mut self, vm: &VmState) -> Result {
        let others = self
            .engine
            .flights
            .iter()
            .filter(|f| !core::ptr::eq(&*f.job.vm, vm))
            .count();
        if others != 0 {
            dev_err!(
                self.dev(),
                "PB backing: VM {} maps {} chunks with {} flights of another VM\n",
                vm.id,
                self.pools.chunks.len(),
                others
            );
            return Err(super::boot::eproto());
        }
        self.pools.map_heap(vm)
    }

    /// Backs chunks until the reserve is whole, mapping each into the VMs of the flights (by
    /// the one-VM rule, the VM that runs) before the ledger may post from it; any other VM
    /// maps it at its next publication ([`Self::map_pb`]). A failure is logged and leaves the
    /// reserve short: requests then wait, and the next pass tries again.
    pub(crate) fn refill_backing(&mut self) {
        // A pending re-initialization is about to drop the posted count: nothing to back for,
        // unless a render published before it still runs (the flights behind a fragment kill
        // keep asking for growth until they retire).
        let render = (0..self.engine.flights.len()).any(|i| self.receipts(i).renders != 0);
        if self.recovery.reinit && !render {
            return;
        }
        while self.pools.ledger.refill_due() {
            let result = (|| -> Result {
                let gem = self.pools.back_chunk()?;
                let i = self.pools.chunks.len() - 1;
                for flight in self.engine.flights.iter() {
                    flight.job.vm.map_chunk(i, gem.clone())?;
                }
                Ok(())
            })();
            match result {
                Ok(()) => {
                    self.pools.ledger.back_chunk();
                    self.pools.stats.refills += 1;
                    dev_dbg!(
                        self.dev(),
                        "PB heap: chunk {} backed ({} of {} blocks posted)\n",
                        self.pools.chunks.len() - 1,
                        self.pools.ledger.posted,
                        self.pools.ledger.backed
                    );
                }
                Err(e) => {
                    // A chunk that could not be mapped into a flight's VM is not counted: drop
                    // it (and its aliases in the VMs) again.
                    let backed = (self.pools.ledger.backed / growth::CHUNK_BLOCKS) as usize;
                    if self.pools.chunks.len() > backed {
                        for flight in self.engine.flights.iter() {
                            flight.job.vm.unmap_heap_above(backed);
                        }
                        self.pools.chunks.truncate(backed);
                    }
                    dev_warn!(
                        self.dev(),
                        "PB heap: backing chunk {} failed: {} ({} of {} blocks posted)\n",
                        backed,
                        e.to_errno(),
                        self.pools.ledger.posted,
                        self.pools.ledger.backed
                    );
                    return;
                }
            }
        }
    }

    /// Notes the engine idle (nothing published, nothing pending) or busy; the idle time is
    /// what [`Self::reclaim_check`] measures. Called at the end of every poll pass and
    /// publication (the pass that retires the last flight is not re-armed) and at a job's
    /// arrival.
    pub(crate) fn note_idle(&mut self) {
        let idle = self.engine.flights.is_empty() && self.engine.pending.is_empty();
        self.pools.idle.note(idle, super::engine::now_ns());
    }

    /// Reclaims the PB when the engine has been idle for `g15_pb_idle_ms` with more than
    /// `g15_pb_reclaim_blocks` posted or more backing than the posted blocks need, or for
    /// [`growth::DRAIN_IDLE_NS`] with more than half the maximum posted
    /// ([`growth::Ledger::drain_reinit_due`]; re-initialization only): the next render prefix
    /// re-initializes the render allocators and the PB (InitBM of the initial blocks;
    /// `recovery.reinit`, as after a recovered abort but with no halt), and with
    /// `g15_pb_reclaim` 2 the backing chunks above those of the initial posting and the reserve
    /// are freed now, unmapped from every VM of the registry (a VM that publishes later drops
    /// its stale mappings at [`VmState::map_heap`]). While a reclaim's re-initialization is
    /// pending (a client-boundary one kept the backing), `g15_pb_reclaim` 2 frees the backing
    /// above those chunks once the engine has been idle for `g15_pb_idle_ms`
    /// ([`growth::Ledger::pending_free_due`]). Called under the device lock when a job arrives
    /// at an idle engine and in idle poll passes.
    pub(crate) fn reclaim_check(&mut self) {
        let level = super::params::pb_reclaim();
        if level == 0
            || !self.engine.flights.is_empty()
            || !self.engine.pending.is_empty()
            || (self.recovery.reinit && !self.pools.reclaim_pending)
            || self.recovery.armed.is_some()
            || self.engine.frozen
            || self.recovery.failed().is_some()
        {
            return;
        }
        let idle_ns = self.pools.idle.idle_ns(super::engine::now_ns());
        let floor = super::params::pb_reclaim_blocks();
        let min_idle = super::params::pb_idle_ns();
        let ledger = &self.pools.ledger;
        if self.recovery.reinit {
            if level >= 2 && ledger.pending_free_due(idle_ns, min_idle) {
                let posted = ledger.posted;
                let freed = self.free_backing();
                dev_info!(
                    self.dev(),
                    "PB heap: idle {} ms with {} blocks posted and the re-initialization pending: {} chunks freed, {} backed\n",
                    idle_ns / 1_000_000,
                    posted,
                    freed,
                    self.pools.chunks.len()
                );
            }
            return;
        }
        let idle = ledger.reclaim_due(floor, idle_ns, min_idle)
            || (level >= 2 && ledger.free_due(idle_ns, min_idle));
        if !idle && !ledger.drain_reinit_due(idle_ns) {
            return;
        }
        self.recovery.reinit = true;
        self.pools.reclaim_pending = true;
        self.pools.stats.reclaims += 1;
        let posted = self.pools.ledger.posted;
        let mut freed = 0;
        // A short idle keeps the backing: the next scene would allocate it again.
        if level >= 2 && idle {
            freed = self.free_backing();
        }
        dev_info!(
            self.dev(),
            "PB heap: idle {} ms with {} blocks posted: re-initialization to {} at the next render, {} chunks freed, {} backed\n",
            idle_ns / 1_000_000,
            posted,
            self.pools.ledger.initial,
            freed,
            self.pools.chunks.len()
        );
    }

    /// Frees the backing chunks above those a re-initialized heap keeps
    /// ([`growth::Ledger::keep_chunks`]) and drops their mappings in every VM that could
    /// publish; returns the chunks freed. If the VMs cannot be walked, their mappings keep the
    /// pages until each VM drops its stale aliases ([`VmState::map_heap`]): nothing is counted
    /// freed.
    fn free_backing(&mut self) -> usize {
        let keep = self.pools.ledger.keep_chunks() as usize;
        let dropped = self.pools.free_chunks(keep);
        // The mappings hold the chunks' pages: drop them in every VM that could publish.
        let freed = match self.vms.open() {
            Ok(states) => {
                for state in states.iter() {
                    state.unmap_heap_above(keep);
                }
                dropped
            }
            Err(e) => {
                dev_warn!(
                    self.dev(),
                    "PB heap: reclaim could not walk the VMs: {}\n",
                    e.to_errno()
                );
                0
            }
        };
        self.pools.stats.freed += freed as u64;
        freed
    }

    /// (Re)initializes the render UMA managers, PB lists and PB manager. `rebuild`: the priming
    /// prefix after a recovered abort: the first one moves to the uncached spare set, later
    /// ones rewrite it in place.
    pub(crate) fn init_render_pools(&mut self, rebuild: bool) -> Result {
        if rebuild && self.pools.spare_ready {
            self.pools.take_spare();
        }
        self.pools.init()
    }

    /// The PB counters of a publication with renders, after the render threshold: the PB
    /// counter names the prefix's last draw, and C1 = {0x07400000, 4 * initial blocks} is
    /// written once per (re)initialization. An error leaves the prefix unpublished.
    pub(crate) fn publish_pb(&mut self, draw: u32) -> Result {
        self.pools.live.pb_counter.w32(0, draw)?;
        if !self.pools.c1_written {
            self.write_c1(&buffer::c1_record(self.pools.ledger.initial))?;
            self.pools.c1_written = true;
        }
        Ok(())
    }

    /// Answers the firmware's PB-growth requests, outside the event drain.
    /// A request names PB slot 0, the context of a published render prefix and the next
    /// request counter; anything else is logged and left unanswered (its TA then stays stalled
    /// and the deadline applies). With growth up to ten more blocks of the mapped heap are
    /// written to unposted block-list records, then the live block_ctl+4 producer advances and
    /// DEVCTL 7 reports success; at the ledger's maximum the reply is the refusal (+4 = 0),
    /// which gives the firmware no new pages, and after two refusals requests are left
    /// unanswered. The firmware imports the blocks itself: P is never rewritten. Only the
    /// requests queued before the pass are taken. A request with no backed block left waits:
    /// it goes back to the head of the queue for the next pass, and this pass ends its answers
    /// there. After the answers the reserve is refilled ([`Self::refill_backing`]). A failed
    /// reply is logged; a request that cannot be processed at all (a firmware object access
    /// failed) is logged and left unanswered like an invalid one. Never fails: the poll pass
    /// continues.
    pub(crate) fn answer_growth(&mut self) -> Result {
        // Only the requests queued before this pass: a reply's wait drains the rings and may
        // queue the next request, which the next pass answers (the device lock is released in
        // between).
        let queued = self.events.grow.len();
        for _ in 0..queued {
            let Ok(request) = self.events.grow.remove(0) else {
                break;
            };
            match self.answer_request(&request) {
                Ok(true) => self.pools.stats.requests += 1,
                Ok(false) => {
                    // Waits for backing: back at the head, counted once answered.
                    self.pools.stats.waits += 1;
                    // The queue has room: this request was just taken from it.
                    if self.events.grow.insert_within_capacity(0, request).is_err() {
                        self.pools.stats.bad += 1;
                    }
                    break;
                }
                Err(e) => {
                    self.pools.stats.requests += 1;
                    self.pools.stats.bad += 1;
                    dev_err!(
                        self.dev(),
                        "PB growth request {} ctx {} not answered: {}\n",
                        request[buffer::grow_request::COUNTER],
                        request[buffer::grow_request::CONTEXT],
                        e.to_errno()
                    );
                }
            }
        }
        self.refill_backing();
        Ok(())
    }

    /// How long the idle engine must stay idle before [`Self::reclaim_check`] acts, if it can
    /// act without new work ([`growth::Ledger::reclaim_delay`], or with a reclaim's
    /// re-initialization pending [`growth::Ledger::pending_free_delay`]); `None` while busy,
    /// while a recovery or its re-initialization is pending, or when nothing would become due.
    /// An idle engine runs no poll pass by itself, so the poll work is armed for this delay.
    pub(crate) fn reclaim_delay_ns(&self) -> Option<u64> {
        if !self.engine.flights.is_empty()
            || !self.engine.pending.is_empty()
            || (self.recovery.reinit && !self.pools.reclaim_pending)
            || self.recovery.armed.is_some()
            || self.engine.frozen
            || self.recovery.failed().is_some()
        {
            return None;
        }
        let idle_ns = self.pools.idle.idle_ns(super::engine::now_ns());
        if self.recovery.reinit {
            return self.pools.ledger.pending_free_delay(
                super::params::pb_reclaim(),
                idle_ns,
                super::params::pb_idle_ns(),
            );
        }
        self.pools.ledger.reclaim_delay(
            super::params::pb_reclaim(),
            super::params::pb_reclaim_blocks(),
            idle_ns,
            super::params::pb_idle_ns(),
        )
    }

    /// Processes one growth request: `Ok(false)` when it waits for backing.
    fn answer_request(&mut self, request: &[u32; 14]) -> Result<bool> {
        let ctl = self.init.control.clone().ok_or(ENODEV)?;
        let epoch =
            ctl.r32(control::HALT_COUNT)? as u64 | (ctl.r32(control::HALT_COUNT + 4)? as u64) << 32;
        let ctx = request[buffer::grow_request::CONTEXT];
        let live = self
            .engine
            .flights
            .iter()
            .enumerate()
            .any(|(i, f)| self.receipts(i).renders != 0 && f.job.slot() == ctx);
        // At the maximum, after the refusals, the request is left unanswered: the poll pass
        // returns, and the host deadline and its recovery path apply to a stalled TA.
        if self.pools.ledger.exhausted() {
            self.pools.stats.bad += 1;
            if !self.pools.exhausted_logged {
                self.pools.exhausted_logged = true;
                dev_warn!(
                    self.dev(),
                    "PB growth request {} ctx {} left unanswered: {} refusals at the maximum ({} blocks); the host deadline applies\n",
                    request[buffer::grow_request::COUNTER],
                    ctx,
                    self.pools.ledger.refusals,
                    self.pools.ledger.max
                );
            }
            return Ok(true);
        }
        // A request from before a halt has no authority over the PB now.
        if !self
            .pools
            .ledger
            .admits(request, tuning::PB_GROW, live, epoch)
        {
            self.pools.stats.bad += 1;
            dev_warn!(
                self.dev(),
                "PB growth request not answered: ctx {} slot {} counter {} (next {}) epoch {:#x} (now {:#x}) live {} pb_grow {}\n",
                ctx,
                request[buffer::grow_request::SLOT],
                request[buffer::grow_request::COUNTER],
                self.pools.ledger.next_request,
                growth::request_epoch(request),
                epoch,
                live as u32,
                tuning::PB_GROW as u32
            );
            return Ok(true);
        }
        // A grant posts only backed blocks, and every backed chunk is mapped in the VM of the
        // flights (at its publication, or at the refill that backed it).
        let (first, n) = match self.pools.ledger.accept(request) {
            Answer::Grant { first, count } => (first, count),
            Answer::Refuse => (self.pools.ledger.posted, 0),
            Answer::Wait => return Ok(false),
        };
        let heap = client::PB_HEAP;
        for b in first..first + n {
            self.pools.pb_blocks.w64(
                buffer::block_record_offset(b),
                buffer::block_record(heap, b) as u64,
            )?;
        }
        if n != 0 {
            // Records before their producer.
            crate::mem::sync();
            let producer = self.pools.ledger.post(n);
            self.pools
                .live
                .block_ctl
                .w32(block_ctl::PRODUCER, producer)?;
            // Producer before the reply.
            crate::mem::sync();
            self.pools.stats.grown += n as u64;
        } else {
            self.pools.stats.refused += 1;
        }
        let reply = DevCtl {
            words: buffer::grow_reply_words(request, n != 0),
        };
        let ret = self.devctl(&reply, true);
        let ledger = self.pools.ledger;
        let soft = self.pools.soft_max();
        // Every answer past the firmware's maximum is logged with its TA position: a scene
        // that keeps asking at one position is a runaway, one that advances is large.
        let past_soft = n != 0 && ledger.posted > soft;
        let info = self.pools.ledger.log_answer(request, n, ret.is_err()) || past_soft;
        let (counter, position) = (
            request[buffer::grow_request::COUNTER],
            request[buffer::grow_request::POSITION],
        );
        let ret = match ret {
            Ok(()) => 0,
            Err(e) => e.to_errno(),
        };
        if info {
            dev_info!(
                self.dev(),
                "PB growth request {} ctx {} position {}: +{} blocks ({} of {}), DEVCTL 7: {}\n",
                counter,
                ctx,
                position,
                n,
                ledger.posted,
                ledger.max,
                ret
            );
        } else {
            dev_dbg!(
                self.dev(),
                "PB growth request {} ctx {} position {}: +{} blocks ({} of {}), DEVCTL 7: {}\n",
                counter,
                ctx,
                position,
                n,
                ledger.posted,
                ledger.max,
                ret
            );
        }
        Ok(true)
    }
}
