// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Physical kick-ring backing, queue identities and entry publication.
//!
//! The device mutex serializes identities and publication. Installed identities
//! and their ring backing stay owned until both firmware processors stop;
//! command completion alone does not detach a hardware queue.

use core::{
    mem::ManuallyDrop,
    sync::atomic::{fence, Ordering},
};
use kernel::{
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
};

use super::{
    fw::{kick::*, queue::DataMaster},
    object::{Allocator, CpuMap, KernelObject},
};
use crate::{hw::t8140, mem, mmu};

pub(super) const QID_COUNT: usize = QID_MAX as usize + 1;

/// Render identities that must stay reachable for `queues` logical queues:
/// one tiling/fragment pair each, plus the two bootstrap reservations.
///
/// Installed identities never change engine before processor stop, so a
/// compute channel permanently narrows what render can still install. Compute
/// may take a free QID only while every open logical queue, and
/// [`RENDER_HEADROOM`] queues opened later, can still install a dedicated
/// pair; logical-queue admission refuses a queue whose pair no longer fits.
pub(super) const fn render_reserve(queues: usize) -> usize {
    let queues = queues + RENDER_HEADROOM;
    let max = crate::mmu::MAX_EXECUTION_CONTEXTS as usize;
    2 * if queues < max { queues } else { max } + 2
}
/// Logical queues that may still open after compute growth without losing a
/// dedicated render pair.
const RENDER_HEADROOM: usize = 24;
/// The render reserve with every execution context open.
const FULL_RENDER_RESERVE: usize = render_reserve(crate::mmu::MAX_EXECUTION_CONTEXTS as usize);
/// Compute channels that stay reachable with every execution context open.
#[cfg(CONFIG_KUNIT = "y")]
pub(super) const COMPUTE_FLOOR: usize = QID_COUNT - FULL_RENDER_RESERVE;
/// Upper bound on distinct physical compute channels named by one kick.
pub(super) const COMPUTE_CHANNELS: usize = QID_COUNT;
/// Maximum logical queues after firmware-owned QIDs are excluded.
pub(super) const fn queue_limit(floor: usize) -> u32 {
    ((FULL_RENDER_RESERVE - 2 - floor) / 2) as u32
}

const _: () = {
    assert!(queue_limit(0) == crate::mmu::MAX_EXECUTION_CONTEXTS);
    // Compute keeps this many channels even with every execution context open.
    assert!(QID_COUNT - FULL_RENDER_RESERVE >= 4);
    assert!(crate::hw::t8140::queues::RENDER_PAIRS_PER_OWNER == 1);
};

/// Stable physical-channel identity. The owner must not be reused within a
/// device lifetime, including after an unpublished allocation is cancelled.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Id {
    qid: u8,
    owner: u64,
    engine: DataMaster,
}

impl Id {
    pub(crate) fn qid(self) -> u8 {
        self.qid
    }
    pub(crate) fn engine(self) -> DataMaster {
        self.engine
    }
}

#[derive(Copy, Clone)]
struct Identity {
    id: Id,
    published: bool,
}

/// One shared namespace for tiling, fragment and compute queues.
pub(crate) struct QueueIds {
    entries: [Option<Identity>; QID_COUNT],
    initial_render: [Option<Id>; 2],
    leased: usize,
    /// First QID the host may use ([`crate::g17::fw::Variant::qid_floor`]).
    floor: usize,
}

impl QueueIds {
    pub(crate) fn new(floor: usize) -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            entries <- pin_init::init_array_from_fn(|_| None),
            initial_render: [None; 2],
            leased: 0,
            // The reserve must keep render capacity above the floor.
            floor: if floor < FULL_RENDER_RESERVE { floor } else { return Err(EINVAL) },
        })
    }

    /// Keep the initial render reservations until another engine needs their
    /// capacity. No graph receives these tokens, so an unpublished reservation
    /// can be replaced without changing an installed physical channel.
    pub(crate) fn reserve_initial_render(&mut self) -> Result {
        if self.leased != 0 {
            return Err(EINVAL);
        }
        for (qid, engine) in [DataMaster::Tiling, DataMaster::Fragment]
            .into_iter()
            .enumerate()
        {
            let id = self.reserve(qid as u64 + 1, engine, Some(qid as u8))?;
            self.initial_render[qid] = Some(id);
        }
        Ok(())
    }

    /// Reuses only an exact owner/engine match. Preferred placement is for
    /// bootstrapping; it does not reserve a partition for an engine.
    pub(crate) fn reserve(
        &mut self,
        owner: u64,
        engine: DataMaster,
        preferred: Option<u8>,
    ) -> Result<Id> {
        self.reserve_for(owner, engine, preferred, crate::mmu::MAX_EXECUTION_CONTEXTS as usize)
    }

    fn render_ids(&self) -> usize {
        self.entries
            .iter()
            .flatten()
            .filter(|entry| matches!(entry.id.engine, DataMaster::Tiling | DataMaster::Fragment))
            .count()
    }

    /// Whether `queues` open logical queues can each still install a dedicated
    /// render pair, beside the bootstrap reservations and installed compute.
    pub(crate) fn render_fits(&self, queues: usize) -> bool {
        self.render_ids() + (QID_COUNT - self.floor - self.leased) >= 2 * queues + 2
    }

    /// Whether a render pair not yet built can still reserve both of its identities.
    pub(crate) fn render_pair_fits(&self) -> bool {
        QID_COUNT - self.floor - self.leased >= 2
    }

    /// Whether a fresh compute identity fits beside the render reserve of
    /// `queues` open logical queues, without reserving it.
    pub(crate) fn compute_fits(&self, queues: usize) -> bool {
        let initial = self.leased == QID_COUNT - self.floor
            && self.initial_render.iter().flatten().any(|id| self.can_cancel_unpublished(*id));
        self.compute_room(queues, usize::from(initial))
    }

    fn compute_room(&self, queues: usize, replacement: usize) -> bool {
        // Existing render identities already satisfy this reservation. Counting
        // them twice can refuse compute even while physical QIDs remain free.
        // Replacing an initial reservation consumes one render identity.
        // Include that transition when protecting render's minimum, and
        // count the replacement as available compute capacity.
        let render = self.render_ids();
        QID_COUNT - self.floor - self.leased + replacement
            > render_reserve(queues)
                .min(FULL_RENDER_RESERVE - self.floor)
                .saturating_sub(render - replacement)
    }

    /// As [`Self::reserve`], with the compute reserve sized for `queues` open
    /// logical queues instead of every possible execution context.
    pub(crate) fn reserve_for(
        &mut self,
        owner: u64,
        engine: DataMaster,
        preferred: Option<u8>,
        queues: usize,
    ) -> Result<Id> {
        if owner == 0 {
            return Err(EINVAL);
        }
        if let Some(entry) = self
            .entries
            .iter()
            .flatten()
            .find(|entry| entry.id.owner == owner)
        {
            return if entry.id.engine == engine {
                Ok(entry.id)
            } else {
                Err(EINVAL)
            };
        }
        // The QIDs below the floor are never available to the host.
        let total = QID_COUNT - self.floor;
        let initial = if engine == DataMaster::Compute && self.leased == total {
            self.initial_render
                .iter()
                .copied()
                .enumerate()
                .find_map(|(slot, id)| {
                    id.filter(|id| self.can_cancel_unpublished(*id))
                        .map(|id| (slot, id))
                })
        } else {
            None
        };
        // Published identities always retain their original lifetime rules.
        if engine == DataMaster::Compute && !self.compute_room(queues, usize::from(initial.is_some())) {
            return Err(crate::g17::capacity::refused(crate::g17::capacity::Site::ComputeReserve));
        }
        let index = preferred
            .map(|index| usize::from(index) + self.floor)
            .filter(|index| self.entries.get(*index).is_some_and(Option::is_none))
            .or_else(|| {
                self.entries
                    .iter()
                    .skip(self.floor)
                    .position(Option::is_none)
                    .map(|index| index + self.floor)
            })
            .or_else(|| initial.map(|(_, id)| usize::from(id.qid)))
            .ok_or_else(|| crate::g17::capacity::refused(crate::g17::capacity::Site::PhysicalQids))?;
        let id = Id {
            qid: index as u8,
            owner,
            engine,
        };
        self.entries[index] = Some(Identity {
            id,
            published: false,
        });
        if let Some((slot, _)) = initial {
            self.initial_render[slot] = None;
        } else {
            self.leased += 1;
        }
        Ok(id)
    }

    /// Call after preflighting the complete installation, before any address
    /// becomes firmware-visible. A failed or partial installation retains it.
    pub(crate) fn publish(&mut self, id: Id) -> Result {
        let entry = self
            .entries
            .get_mut(usize::from(id.qid))
            .and_then(Option::as_mut)
            .ok_or(EINVAL)?;
        if entry.id != id {
            return Err(EINVAL);
        }
        entry.published = true;
        Ok(())
    }

    pub(crate) fn can_cancel_unpublished(&self, id: Id) -> bool {
        self.entries
            .get(usize::from(id.qid))
            .and_then(Option::as_ref)
            .is_some_and(|entry| entry.id == id && !entry.published)
    }

    pub(crate) fn cancel_unpublished(&mut self, id: Id) -> Result {
        let slot = self.entries.get_mut(usize::from(id.qid)).ok_or(EINVAL)?;
        match slot {
            Some(entry) if entry.id == id && !entry.published => {
                *slot = None;
                self.leased -= 1;
                Ok(())
            }
            _ => Err(EINVAL),
        }
    }
}

#[kunit_tests(asahi_g17_qid_accounting)]
mod accounting_tests {
    use super::*;

    fn table() -> KBox<QueueIds> {
        KBox::init(QueueIds::new(0), GFP_KERNEL).unwrap()
    }

    #[test]
    fn render_identities_satisfy_the_compute_reserve() {
        let mut ids = table();
        for owner in 1..=FULL_RENDER_RESERVE as u64 {
            ids.reserve(owner, DataMaster::Tiling, None).unwrap();
        }
        for owner in FULL_RENDER_RESERVE as u64 + 1..=QID_COUNT as u64 {
            ids.reserve(owner, DataMaster::Compute, None).unwrap();
        }
        assert_eq!(ids.leased, QID_COUNT);
        assert_eq!(ids.reserve(1000, DataMaster::Compute, None), Err(ENOSPC));
        assert_eq!(ids.leased, QID_COUNT);
    }

    #[test]
    fn compute_cannot_consume_the_render_minimum() {
        let mut ids = table();
        for owner in 1..=(QID_COUNT - FULL_RENDER_RESERVE) as u64 {
            ids.reserve(owner, DataMaster::Compute, None).unwrap();
        }
        assert_eq!(ids.reserve(1000, DataMaster::Compute, None), Err(ENOSPC));
        let tiling = ids.reserve(1001, DataMaster::Tiling, None).unwrap();
        assert_eq!(ids.reserve(1000, DataMaster::Compute, None), Err(ENOSPC));
        ids.cancel_unpublished(tiling).unwrap();
        assert_eq!(ids.leased, QID_COUNT - FULL_RENDER_RESERVE);
    }

    #[test]
    fn dynamic_compute_growth_preserves_render_headroom_above_firmware_floor() {
        for floor in [0, 4, 32] {
            let mut ids = KBox::init(QueueIds::new(floor), GFP_KERNEL).unwrap();
            let queues = 2;
            let mut owner = 1;
            while ids.compute_fits(queues) {
                let id = ids.reserve_for(owner, DataMaster::Compute, None, queues).unwrap();
                assert!(usize::from(id.qid()) >= floor);
                ids.publish(id).unwrap();
                owner += 1;
            }
            assert_eq!(ids.leased, QID_COUNT - floor - render_reserve(queues));
            assert_eq!(ids.reserve_for(owner, DataMaster::Compute, None, queues), Err(ENOSPC));
            assert!(ids.render_fits(queues + RENDER_HEADROOM));
            assert!(!ids.render_fits(queues + RENDER_HEADROOM + 1));
        }
    }

    #[test]
    fn full_logical_queue_budget_keeps_compute_floor_and_excludes_firmware_ids() {
        for floor in [0, 4, 32] {
            let mut ids = KBox::init(QueueIds::new(floor), GFP_KERNEL).unwrap();
            let queues = queue_limit(floor) as usize;
            for owner in 1..=COMPUTE_FLOOR as u64 {
                let id = ids.reserve_for(owner, DataMaster::Compute, None, queues).unwrap();
                assert!(usize::from(id.qid()) >= floor);
                ids.publish(id).unwrap();
            }
            assert!(!ids.compute_fits(queues));
            assert!(ids.render_fits(queues));
            assert!(!ids.render_fits(queues + 1));
        }
    }

    #[test]
    fn initial_reservations_yield_but_published_ids_do_not() {
        let mut ids = table();
        ids.reserve_initial_render().unwrap();
        for owner in 3..=QID_COUNT as u64 {
            ids.reserve(owner, DataMaster::Tiling, None).unwrap();
        }
        let first = ids.reserve(1000, DataMaster::Compute, None).unwrap();
        let second = ids.reserve(1001, DataMaster::Compute, None).unwrap();
        assert_ne!(first.qid(), second.qid());
        assert!(ids.initial_render.iter().all(Option::is_none));
        assert_eq!(ids.leased, QID_COUNT);
        ids.publish(first).unwrap();
        assert_eq!(ids.cancel_unpublished(first), Err(EINVAL));
        assert_eq!(ids.leased, QID_COUNT);
        ids.cancel_unpublished(second).unwrap();
        assert_eq!(ids.leased, QID_COUNT - 1);
        assert_eq!(ids.cancel_unpublished(second), Err(EINVAL));
        assert_eq!(ids.leased, QID_COUNT - 1);
    }

    #[test]
    fn failed_second_render_id_returns_only_the_unpublished_first() {
        let mut ids = table();
        for owner in 1..QID_COUNT as u64 {
            let id = ids.reserve(owner, DataMaster::Tiling, None).unwrap();
            ids.publish(id).unwrap();
        }
        let first = ids.reserve(1000, DataMaster::Tiling, None).unwrap();
        assert_eq!(ids.reserve(1001, DataMaster::Fragment, None), Err(ENOSPC));
        ids.cancel_unpublished(first).unwrap();
        assert_eq!(ids.leased, QID_COUNT - 1);
        let replacement = ids.reserve(1002, DataMaster::Compute, None).unwrap();
        assert_eq!(replacement.qid(), first.qid());
        assert_ne!(replacement, first);
        assert_eq!(ids.reserve(1002, DataMaster::Compute, None), Ok(replacement));
        assert_eq!(ids.leased, QID_COUNT);
        assert_eq!(ids.reserve(1002, DataMaster::Fragment, None), Err(EINVAL));
    }
}

// Aliases drop before their backing. A compute queue is allocated low first;
// a render queue is allocated high first.
enum Backing {
    Compute {
        high: mmu::KernelMapping,
        low: KernelObject,
    },
    Render(RenderRing),
}

/// A render ring's pages and the shared lower reservation clients alias.
struct RenderStorage {
    // The lower alias drops before its backing.
    low: mmu::KernelMapping,
    high: KernelObject,
}

/// Render ring storage outlives its rings for the life of the device.
///
/// A client VM keeps each render ring alias it installs until the VM closes,
/// while an installed pair whose identities were never published can be
/// reclaimed and its rings dropped. Freeing that storage would hand its shared
/// lower address to another ring; a client still aliasing the old pages there
/// that later receives the new ring could not alias it (`prepare_client_alias`
/// fails with EFAULT). Retired storage is therefore kept and reused by later
/// rings: an address keeps naming the same pages, so every alias stays valid.
#[pin_data]
pub(crate) struct RenderRings {
    #[pin]
    stock: Mutex<RingStock>,
}

struct RingStock {
    /// Retired storage; `true` marks storage of a ring that never wrote an
    /// entry, which a later ring may reuse. Written rings stay retired.
    retired: KVec<(bool, RenderStorage)>,
    /// Storage created so far; `retired` keeps capacity for all of it, so a
    /// dropped ring always returns its storage without allocating.
    created: usize,
}

impl RenderRings {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                stock <- new_mutex!(
                    RingStock { retired: KVec::new(), created: 0 },
                    "G17 render kick rings"
                ),
            }),
            GFP_KERNEL,
        )
    }

    fn take(&self) -> Option<RenderStorage> {
        let mut stock = self.stock.lock();
        let index = stock.retired.iter().rposition(|(reusable, _)| *reusable)?;
        let last = stock.retired.len() - 1;
        stock.retired.swap(index, last);
        stock.retired.pop().map(|(_, storage)| storage)
    }

    /// Reserves the return of one more, just created storage.
    fn admit(&self) -> Result {
        let mut stock = self.stock.lock();
        let created = stock.created.checked_add(1).ok_or(EOVERFLOW)?;
        let additional = created.saturating_sub(stock.retired.len());
        stock.retired.reserve(additional, GFP_KERNEL)?;
        stock.created = created;
        Ok(())
    }

    fn put(&self, storage: RenderStorage, reusable: bool) {
        let mut stock = self.stock.lock();
        if let Err(rejected) = stock.retired.push_within_capacity((reusable, storage)) {
            // `admit` reserved room for every created storage. Never free it.
            core::mem::forget(rejected);
        }
    }
}

/// Storage lent to one render ring; returned to the stock when the ring drops.
struct RenderRing {
    storage: ManuallyDrop<RenderStorage>,
    rings: Arc<RenderRings>,
    /// Set once an entry was written: such storage is never reused.
    written: bool,
}

impl Drop for RenderRing {
    fn drop(&mut self) {
        // SAFETY: `storage` is taken exactly once, here, and not used afterwards.
        let storage = unsafe { ManuallyDrop::take(&mut self.storage) };
        self.rings.put(storage, !self.written);
    }
}

/// An installed queue's permanent ring. Client aliases are held by the bound
/// context graph and must be dropped before this object.
pub(crate) struct Queue {
    backing: Backing,
    id: Id,
    timestamp: KickTimestamp,
    previous: Option<KickTimestamp>,
}

/// A checked entry and destination held without changing the shared ring.
/// Discarding it leaves both the slot and the next timestamp unchanged.
pub(crate) struct PreparedKick<'a> {
    queue: &'a mut Queue,
    timestamp: KickTimestamp,
    entry: KickEntry,
    destination: *mut KickEntry,
}

impl PreparedKick<'_> {
    /// Publish translation updates after the queue configuration pointer and before its entry.
    pub(crate) fn prepare_compute_install(&self) -> Result {
        self.queue.prepare_compute_install()
    }

    /// Writes and fences the complete entry, then advances host state. The
    /// caller invokes this between the queue configuration and announcement.
    pub(crate) fn commit(self) -> KickTimestamp {
        // SAFETY: prepare_entry checked the complete destination range and
        // this token exclusively borrows the queue and retains its mapping.
        // The caller only prepares retired ring slots; no firmware access to
        // the new entry starts before the outer command is published.
        unsafe { self.destination.write_unaligned(self.entry) };
        fence(Ordering::SeqCst);
        if let Backing::Render(ring) = &mut self.queue.backing {
            ring.written = true;
        }
        self.queue.previous = Some(self.timestamp);
        self.queue.timestamp = self.timestamp.next();
        self.timestamp
    }
}

impl Queue {
    pub(crate) fn compute(alloc: &Allocator<'_>, id: Id) -> Result<Self> {
        if id.engine != DataMaster::Compute {
            return Err(EINVAL);
        }
        let low = alloc.lower(
            KICK_RING_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let mut range = alloc.uat.geometry().kernel_range();
        range.start += t8140::dynamic::KERNEL_OFFSET;
        let high = low.alias_in(
            alloc.uat.kernel_vm(),
            range,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_FW_SHARED_RW,
        )?;
        fence(Ordering::SeqCst);
        Ok(Self {
            backing: Backing::Compute { high, low },
            id,
            timestamp: KickTimestamp::FIRST,
            previous: None,
        })
    }

    /// Retired storage of a never-written ring is reused first. Its address
    /// names the same pages in every client that aliases it, so an existing
    /// alias in `client` is kept (None) and a missing one is created.
    pub(crate) fn render(
        alloc: &Allocator<'_>,
        id: Id,
        client: &mmu::Vm,
        rings: &Arc<RenderRings>,
    ) -> Result<(Self, Option<mmu::KernelMapping>)> {
        if id.engine == DataMaster::Compute {
            return Err(EINVAL);
        }
        if let Some(storage) = rings.take() {
            let mut ring = RenderRing {
                storage: ManuallyDrop::new(storage),
                rings: rings.clone(),
                written: false,
            };
            // A fresh ring is zeroed; no firmware ever read this one.
            ring.storage.high.bytes_mut().fill(0);
            fence(Ordering::SeqCst);
            let address = ring.storage.low.iova();
            let alias =
                ring.storage
                    .high
                    .prepare_alias(client, address, mmu::PROT_GPU_SHARED_RW, true);
            // On failure the storage returns to the stock and a fresh ring is built.
            if let Ok(alias) = alias {
                return Ok((
                    Self {
                        backing: Backing::Render(ring),
                        id,
                        timestamp: KickTimestamp::FIRST,
                        previous: None,
                    },
                    alias,
                ));
            }
        }
        let high = alloc.kernel(
            KICK_RING_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let mut range = t8140::dynamic::LOWER;
        let (low, client) = loop {
            let low = high.alias_in(
                alloc.uat.kernel_lower_vm(),
                range.clone(),
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
            )?;
            let end = low
                .iova()
                .checked_add(KICK_RING_SIZE as u64)
                .ok_or(EOVERFLOW)?;
            match high.map_alias(client, low.iova(), mmu::PROT_GPU_SHARED_RW) {
                Ok(client) => break (low, client),
                Err(error) if error == ENOSPC && end < range.end => {
                    drop(low);
                    range.start = end;
                }
                Err(error) => return Err(error),
            }
        };
        let storage = RenderStorage { low, high };
        // Count the storage only once it exists. Without room to return it,
        // nothing has aliased it but `client`, which drops first.
        if let Err(error) = rings.admit() {
            drop(client);
            drop(storage);
            return Err(error);
        }
        Ok((
            Self {
                backing: Backing::Render(RenderRing {
                    storage: ManuallyDrop::new(storage),
                    rings: rings.clone(),
                    written: false,
                }),
                id,
                timestamp: KickTimestamp::FIRST,
                previous: None,
            },
            Some(client),
        ))
    }

    pub(crate) fn id(&self) -> Id {
        self.id
    }
    pub(crate) fn low_va(&self) -> u64 {
        match &self.backing {
            Backing::Compute { low, .. } => low.gpu_va(),
            Backing::Render(ring) => ring.storage.low.iova(),
        }
    }
    pub(crate) fn firmware_va(&self) -> u64 {
        match &self.backing {
            Backing::Compute { high, .. } => high.iova(),
            Backing::Render(ring) => ring.storage.high.gpu_va(),
        }
    }
    fn object(&mut self) -> &mut KernelObject {
        match &mut self.backing {
            Backing::Compute { low, .. } => low,
            Backing::Render(ring) => &mut ring.storage.high,
        }
    }
    pub(crate) fn map_client(&mut self, vm: &mmu::Vm) -> Result<mmu::KernelMapping> {
        let address = self.low_va();
        self.object()
            .map_alias(vm, address, mmu::PROT_GPU_SHARED_RW)
    }

    /// Reuse a permanent client view only when every page names this ring.
    pub(crate) fn prepare_client_alias(&self, vm: &mmu::Vm) -> Result<Option<mmu::KernelMapping>> {
        let object = match &self.backing {
            Backing::Compute { low, .. } => low,
            Backing::Render(ring) => &ring.storage.high,
        };
        object.prepare_alias(vm, self.low_va(), mmu::PROT_GPU_SHARED_RW, true)
    }

    /// Both aliases are owned mappings of the same full ring. Publish their
    /// page-table updates to each live translation regime before installation.
    pub(crate) fn prepare_compute_install(&self) -> Result {
        if self.id.engine != DataMaster::Compute {
            return Err(EINVAL);
        }
        mem::sync();
        for asid in 0..4 {
            mem::tlbi_range_or_asid(asid, self.low_va() as usize, KICK_RING_SIZE);
            mem::tlbi_range_or_asid(asid, self.firmware_va() as usize, KICK_RING_SIZE);
        }
        mem::sync();
        Ok(())
    }

    /// Detaches the implicit dependency after firmware recovery, retaining
    /// the monotonically advancing 40-bit producer.
    pub(crate) fn clear_parent_after_recovery(&mut self) {
        self.previous = None;
    }

    pub(crate) fn timestamp(&self) -> KickTimestamp {
        self.timestamp
    }
    pub(crate) fn parent(&self) -> KickTimestamp {
        self.previous.unwrap_or_else(|| match self.id.engine {
            DataMaster::Compute => KickTimestamp::ZERO,
            _ => self.timestamp.prev(),
        })
    }
    pub(crate) fn prepare_entry(&mut self, args: &KickArgs<'_>) -> Result<PreparedKick<'_>> {
        if args.qid != self.id.qid
            || args.timestamp != self.timestamp
            || args.parent != self.parent()
        {
            return Err(EINVAL);
        }
        let entry = KickEntry::new(args)?;
        let destination = self
            .object()
            .pointer(args.timestamp.entry_offset(), size_of::<KickEntry>())?
            .cast();
        let timestamp = self.timestamp;
        Ok(PreparedKick {
            queue: self,
            timestamp,
            entry,
            destination,
        })
    }
}

impl QueueIds {
    pub(crate) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let mut engines = [0usize; 3];
        let mut published = 0;
        for entry in self.entries.iter().flatten() {
            engines[match entry.id.engine {
                DataMaster::Tiling => 0,
                DataMaster::Fragment => 1,
                DataMaster::Compute => 2,
            }] += 1;
            published += usize::from(entry.published);
        }
        writeln!(f, "physical_qids used={} total={} published={} tiling={} fragment={} compute={}",
                 self.leased, QID_COUNT, published, engines[0], engines[1], engines[2])
    }
}
