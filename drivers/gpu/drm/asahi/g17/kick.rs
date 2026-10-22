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

/// Minimum render identities protected from compute-only allocation.
const MIN_RENDER_IDENTITIES: usize = 16;

/// Render identities compute leaves free while `queues` logical queues are
/// open: one pair for each of them, existing pairs included. A published QID
/// never changes engine (no firmware operation uninstalls one) and a live owner
/// keeps its last render pair, so render capacity handed to compute could not
/// be recovered while those owners live. Compute takes any QID beyond this and
/// changes owners only when none remains.
fn render_floor(queues: usize) -> usize {
    MIN_RENDER_IDENTITIES.max(queues.saturating_mul(2)).min(QID_COUNT - 2)
}

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
    compute_publication: u64,
    leased: usize,
}

impl QueueIds {
    pub(crate) fn new() -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            entries <- pin_init::init_array_from_fn(|_| None),
            initial_render: [None; 2],
            compute_publication: 0,
            leased: 0,
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
    /// bootstrapping; it does not reserve a partition for an engine. Compute
    /// reservations use `reserve_compute`, which knows the open queues.
    pub(crate) fn reserve(
        &mut self,
        owner: u64,
        engine: DataMaster,
        preferred: Option<u8>,
    ) -> Result<Id> {
        self.reserve_for(owner, engine, preferred, 0)
    }

    /// A compute reservation while `queues` logical queues are open.
    pub(crate) fn reserve_compute(
        &mut self,
        owner: u64,
        preferred: Option<u8>,
        queues: usize,
    ) -> Result<Id> {
        self.reserve_for(owner, DataMaster::Compute, preferred, queues)
    }

    fn reserve_for(
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
        // Existing render pairs already count towards the reserve. The initial
        // reservations do not: no render pair can use them, so compute may
        // replace an unpublished one instead of taking reserved capacity.
        // Published identities always retain their original lifetime rules.
        let initial = if engine == DataMaster::Compute {
            let need = render_floor(queues).saturating_sub(self.pair_identities());
            if QID_COUNT - self.leased > need {
                None
            } else {
                Some(
                    self.initial_render
                        .iter()
                        .copied()
                        .enumerate()
                        .find_map(|(slot, id)| {
                            id.filter(|id| self.can_cancel_unpublished(*id))
                                .map(|id| (slot, id))
                        })
                        .ok_or(ENOSPC)?,
                )
            }
        } else {
            None
        };
        let index = match initial {
            Some((_, id)) => usize::from(id.qid),
            None => preferred
                .map(usize::from)
                .filter(|index| self.entries.get(*index).is_some_and(Option::is_none))
                .or_else(|| self.entries.iter().position(Option::is_none))
                .ok_or(ENOSPC)?,
        };
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

    /// Leased IDs, render pair identities and compute identities, for diagnostics.
    pub(crate) fn census(&self) -> (usize, usize, usize) {
        let compute = self
            .entries
            .iter()
            .flatten()
            .filter(|entry| entry.id.engine == DataMaster::Compute)
            .count();
        (self.leased, self.pair_identities(), compute)
    }

    /// Tiling and fragment identities of render pairs, excluding the initial
    /// reservations, which no pair uses.
    fn pair_identities(&self) -> usize {
        let render = self
            .entries
            .iter()
            .flatten()
            .filter(|entry| matches!(entry.id.engine, DataMaster::Tiling | DataMaster::Fragment))
            .count();
        render.saturating_sub(self.initial_render.iter().flatten().count())
    }

    /// Render identities beyond the reserve for `queues` open queues. Only
    /// these become compute capacity when an unused render reservation is
    /// cancelled.
    pub(crate) fn render_surplus(&self, queues: usize) -> usize {
        self.pair_identities().saturating_sub(render_floor(queues))
    }

    /// A compute reservation would succeed now: a fresh QID beyond the render
    /// reserve, or a replaceable initial reservation.
    pub(crate) fn compute_available(&self, queues: usize) -> bool {
        QID_COUNT - self.leased > render_floor(queues).saturating_sub(self.pair_identities())
            || self
                .initial_render
                .iter()
                .flatten()
                .any(|id| self.can_cancel_unpublished(*id))
    }

    /// Ordered under the device mutex; no atomic operation on the submission path.
    pub(crate) fn next_compute_publication(&mut self) -> Result<u64> {
        self.compute_publication = self.compute_publication.checked_add(1).ok_or(EOVERFLOW)?;
        Ok(self.compute_publication)
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
            let low = high
                .alias_in(
                    alloc.uat.kernel_lower_vm(),
                    range.clone(),
                    mmu::UAT_PGSZ as u64,
                    mmu::PROT_GPU_SHARED_RW,
                )
                .map_err(|error| {
                    if error == ENOSPC {
                        resource_exhausted!("shared lower GPU VA (render kick ring)")
                    } else {
                        error
                    }
                })?;
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
                Err(error) if error == ENOSPC => {
                    return Err(resource_exhausted!("client GPU VA for a render kick ring"))
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
