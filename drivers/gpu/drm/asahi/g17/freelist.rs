// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Retained USC private-memory pools and their submission counters.
//!
//! A logical context owns one render pool, shared by its render pairs. A
//! physical compute queue owns a separate pool. All mutation and publication
//! runs under the device mutex. Pool aliases must outlive every firmware
//! reference and are released only after the owner's teardown is acknowledged.

use super::fw::freelist::*;
use super::fw::initdata::{FreeListArgs, FreeListClass, FreeListControl, FREE_LIST_BLOCK_PAGES, FREE_LIST_BLOCK_SIZE};
use super::object::{Allocator, CpuMap, KernelObject};
use crate::mmu;
use core::sync::atomic::{fence, AtomicBool, AtomicU16, AtomicU64, Ordering};
use kernel::{
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
};

mod render_ids;

const NO_RENDER_ID: u16 = u16::MAX;
static NEXT_RENDER_OWNER: AtomicU64 = AtomicU64::new(1);

struct RenderIdCell {
    owner: u64,
    slot: AtomicU16,
    generation: AtomicU64,
    closed: AtomicBool,
    storage: Arc<RenderStorage>,
}
impl RenderIdCell {
    fn has_storage(&self) -> bool {
        self.storage.backing.lock().is_some() || self.storage.spare.lock().is_some()
    }
}

/// Detached after the device's final free-list release. Drop outside the
/// device and identity locks because mappings may take memory-management locks.
#[must_use]
pub(crate) struct ReclaimedRenderId {
    _backing: Option<RenderBacking>,
    spare: Option<Pages>,
}
impl ReclaimedRenderId {
    /// Caller is outside the device lock. Remove the previous client's alias
    /// before handing physical storage to another logical pool.
    fn into_pages(self) -> Option<Pages> {
        self._backing.map(RenderBacking::into_pages).or(self.spare)
    }
}

struct RenderIdState {
    leases: render_ids::State,
    cells: [Option<Arc<RenderIdCell>>; render_ids::COUNT],
}

/// Render pools occupy 129..255; compute uses its QID plus one, in 1..128.
/// First-free allocation keeps the shared firmware namespace deterministic.
#[pin_data]
pub(crate) struct RenderIds {
    used: [AtomicU64; 2],
    available: Arc<super::availability::Availability>,
    #[pin]
    state: Mutex<RenderIdState>,
}
impl RenderIds {
    #[inline(never)]
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(try_pin_init!(Self {
            used: [AtomicU64::new(0), AtomicU64::new(1 << 63)],
            available: super::availability::Availability::new()?,
            state <- new_mutex!(RenderIdState {
                leases: render_ids::State::new(),
                cells: [const { None }; render_ids::COUNT],
            }, "G17 render identities"),
        }), GFP_KERNEL)
    }

    pub(crate) fn availability(&self) -> &Arc<super::availability::Availability> {
        &self.available
    }

    fn pin(self: &Arc<Self>, cell: &Arc<RenderIdCell>, assigned: impl FnOnce()) -> Result<RenderIdPin> {
        let mut state = self.state.lock();
        let slot = cell.slot.load(Ordering::Acquire);
        if slot != NO_RENDER_ID {
            let lease = render_ids::Lease {
                slot, owner: cell.owner, generation: cell.generation.load(Ordering::Acquire),
            };
            match state.leases.pin(lease) {
                Ok(_) => return Ok(RenderIdPin { allocator: self.clone(), cell: cell.clone(), lease }),
                Err(render_ids::Error::Overflow) => return Err(EOVERFLOW),
                Err(_) => {}
            }
        }
        let claim = state.leases.claim(cell.owner, |_| false).map_err(|error| match error {
            render_ids::Error::Busy | render_ids::Error::PendingRelease => ENOSPC,
            render_ids::Error::Overflow => EOVERFLOW,
            _ => EIO,
        })?;
        let index = usize::from(claim.lease.slot - render_ids::FIRST);
        state.cells[index] = Some(cell.clone());
        assigned();
        cell.generation.store(claim.lease.generation, Ordering::Relaxed);
        cell.slot.store(claim.lease.slot, Ordering::Release);
        Ok(RenderIdPin { allocator: self.clone(), cell: cell.clone(), lease: claim.lease })
    }

    fn close(&self, cell: &Arc<RenderIdCell>) {
        cell.closed.store(true, Ordering::Release);
        let mut state = self.state.lock();
        let slot = cell.slot.load(Ordering::Acquire);
        if slot == NO_RENDER_ID { return; }
        let index = usize::from(slot - render_ids::FIRST);
        if state.cells[index].as_ref().is_some_and(|owner| Arc::ptr_eq(owner, cell))
            && state.leases.remove(cell.owner, |_| true).is_ok() {
            let old = state.cells[index].take();
            cell.slot.store(NO_RENDER_ID, Ordering::Release);
            drop(state);
            drop(old);
            self.available.notify();
        }
    }

    /// Caller holds the device mutex and supplies the exact final release proof.
    pub(crate) fn reclaim_one(&self, released: impl FnMut(u16) -> bool) -> Option<ReclaimedRenderId> {
        self.reclaim(false, released)
    }
    /// Reuse backing for a pool that already has an ID, without evicting an
    /// empty metadata-only owner. Caller supplies the same release proof.
    pub(crate) fn reclaim_backing(&self, released: impl FnMut(u16) -> bool) -> Option<ReclaimedRenderId> {
        self.reclaim(true, released)
    }
    fn reclaim(&self, only_storage: bool, mut released: impl FnMut(u16) -> bool) -> Option<ReclaimedRenderId> {
        let mut state = self.state.lock();
        let RenderIdState { leases, cells } = &mut *state;
        let mut select = |require_storage: bool| leases.reclaim_one(|lease| {
            cells[usize::from(lease.slot - render_ids::FIRST)].as_ref().is_some_and(|cell|
                cell.owner == lease.owner
                    && cell.slot.load(Ordering::Acquire) == lease.slot
                    && cell.generation.load(Ordering::Acquire) == lease.generation
                    && (!require_storage || cell.has_storage()))
                && released(lease.slot)
        });
        let lease = select(true).or_else(|| if only_storage { None } else { select(false) })?;
        let old = state.cells[usize::from(lease.slot - render_ids::FIRST)].take();
        let (backing, spare) = if let Some(cell) = old.as_ref() {
            let backing = cell.storage.backing.lock().take();
            let spare = cell.storage.spare.lock().take();
            cell.storage.page_list.store(0, Ordering::Release);
            cell.slot.store(NO_RENDER_ID, Ordering::Release);
            (backing, spare)
        } else { (None, None) };
        drop(state);
        drop(old);
        self.available.notify();
        Some(ReclaimedRenderId { _backing: backing, spare })
    }
}

pub(super) struct RenderIdPin {
    allocator: Arc<RenderIds>,
    cell: Arc<RenderIdCell>,
    lease: render_ids::Lease,
}
impl Drop for RenderIdPin {
    fn drop(&mut self) {
        let available = self.allocator.state.lock().leases.unpin(self.lease).unwrap_or(false);
        if self.cell.closed.load(Ordering::Acquire) { self.allocator.close(&self.cell); }
        if available { self.allocator.available.notify(); }
    }
}

enum RenderIdKind {
    Fixed(u16),
    Rebindable(Arc<RenderIdCell>),
}
pub(super) struct RenderId {
    kind: RenderIdKind,
    allocator: Arc<RenderIds>,
}
impl RenderId {
    pub(super) fn slot(&self) -> u16 {
        match &self.kind {
            RenderIdKind::Fixed(slot) => *slot,
            RenderIdKind::Rebindable(cell) => cell.slot.load(Ordering::Acquire),
        }
    }
    pub(super) fn logical(allocator: &Arc<RenderIds>) -> Result<Self> {
        let owner = NEXT_RENDER_OWNER.fetch_update(Ordering::Relaxed, Ordering::Relaxed,
            |next| next.checked_add(1)).map_err(|_| EOVERFLOW)?;
        let storage = RenderStorage::new(None)?;
        Ok(Self { kind: RenderIdKind::Rebindable(Arc::new(RenderIdCell {
            owner, slot: AtomicU16::new(NO_RENDER_ID), generation: AtomicU64::new(0),
            closed: AtomicBool::new(false),
            storage,
        }, GFP_KERNEL)?), allocator: allocator.clone() })
    }
    fn storage(&self) -> Option<Arc<RenderStorage>> {
        match &self.kind {
            RenderIdKind::Rebindable(cell) => Some(cell.storage.clone()),
            RenderIdKind::Fixed(_) => None,
        }
    }
    fn pin(&self, assigned: impl FnOnce()) -> Result<Option<RenderIdPin>> {
        match &self.kind {
            RenderIdKind::Fixed(_) => Ok(None),
            RenderIdKind::Rebindable(cell) => self.allocator.pin(cell, assigned).map(Some),
        }
    }
    pub(super) fn new(allocator: &Arc<RenderIds>) -> Result<Self> {
        for (bank, used) in allocator.used.iter().enumerate() {
            let mut value = used.load(Ordering::Acquire);
            loop {
                let bit = value.trailing_ones();
                if bit == 64 {
                    break;
                }
                match used.compare_exchange_weak(
                    value,
                    value | (1u64 << bit),
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => {
                        return Ok(Self {
                            kind: RenderIdKind::Fixed(129 + bank as u16 * 64 + bit as u16),
                            allocator: allocator.clone(),
                        })
                    }
                    Err(current) => value = current,
                }
            }
        }
        Err(super::capacity::refused(super::capacity::Site::RenderPoolIds))
    }
}
impl Drop for RenderId {
    fn drop(&mut self) {
        match &self.kind {
            RenderIdKind::Fixed(slot) => {
                let bit = *slot as usize - 129;
                self.allocator.used[bit / 64].fetch_and(!(1u64 << (bit % 64)), Ordering::Release);
            }
            RenderIdKind::Rebindable(cell) => self.allocator.close(cell),
        }
    }
}

/// What one command needs of its compute queue's USC pool, in the family's
/// terms: G17P pools count blocks, M5 pools have the command plan's geometry
/// (`None`: the default M5 pool). Derived once per command
/// (`Backend::compute_capacity`); every admission check and allocation takes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ComputeCapacity {
    Blocks(usize),
    M5(Option<super::fw::m5::pool::PoolGeometry>),
}

/// Profile-specific descriptor data for a fresh physical pool. The runtime
/// installs this only before publication, or after an acknowledged release.
pub(crate) enum PoolDescriptor {
    Neo { slot: u16, page_list: u64, blocks: usize },
    M5 { slot: u16, words: [u64; 8] },
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PageLayout {
    Neo { blocks: usize, runs: RunLayout },
    M5(super::fw::m5::pool::PoolGeometry),
}

/// Free-list backing is by far the largest user of the shared lower GPU VA
/// window, and placing it populates every page before the window is searched.
/// After a refused placement, builders waiting for capacity would allocate and
/// free tens of MiB per attempt while nothing in the window changed. A refused
/// size is therefore refused again without allocating until free-list backing
/// is freed, except for one probe per interval across all callers.
static LOWER_FREED: AtomicU64 = AtomicU64::new(0);
/// `LOWER_FREED + 1` at the latest refusal, or zero after a placement fits.
static LOWER_REFUSED: AtomicU64 = AtomicU64::new(0);
static LOWER_REFUSED_SIZE: AtomicU64 = AtomicU64::new(0);
static LOWER_PROBE_AT: AtomicU64 = AtomicU64::new(0);
const LOWER_PROBE_NS: u64 = 100_000_000;

/// The window refused free-list backing and nothing was freed since.
pub(crate) fn lower_pressure() -> bool {
    let refused = LOWER_REFUSED.load(Ordering::Acquire);
    refused != 0 && refused == LOWER_FREED.load(Ordering::Acquire).wrapping_add(1)
}

/// Place free-list backing of `size` bytes unless the same or a smaller size
/// was refused with nothing freed since; `place` allocates and maps it.
fn place_lower<T>(size: usize, place: impl FnOnce() -> Result<T>) -> Result<T> {
    let size = size as u64;
    if lower_pressure() && size >= LOWER_REFUSED_SIZE.load(Ordering::Relaxed) {
        let now = super::now_ns();
        let at = LOWER_PROBE_AT.load(Ordering::Relaxed);
        if now < at
            || LOWER_PROBE_AT
                .compare_exchange(at, now + LOWER_PROBE_NS, Ordering::AcqRel, Ordering::Relaxed)
                .is_err()
        {
            return Err(ENOSPC);
        }
    }
    match place() {
        Err(error) if error == ENOSPC => {
            LOWER_REFUSED_SIZE.store(size, Ordering::Relaxed);
            LOWER_PROBE_AT.store(super::now_ns() + LOWER_PROBE_NS, Ordering::Relaxed);
            LOWER_REFUSED.store(
                LOWER_FREED.load(Ordering::Acquire).wrapping_add(1),
                Ordering::Release,
            );
            Err(error)
        }
        Ok(placed) => {
            if size >= LOWER_REFUSED_SIZE.load(Ordering::Relaxed) {
                LOWER_REFUSED.store(0, Ordering::Release);
            }
            Ok(placed)
        }
        Err(error) => Err(error),
    }
}

pub(crate) struct Pages {
    backing: KernelObject,
    layout: PageLayout,
}
impl Drop for Pages {
    fn drop(&mut self) {
        LOWER_FREED.fetch_add(1, Ordering::AcqRel);
    }
}
impl Pages {
    fn new(alloc: &Allocator<'_>, blocks: usize, runs: RunLayout) -> Result<Self> {
        let size = TABLE_SIZE + blocks * FREE_LIST_BLOCK_SIZE;
        let backing = place_lower(size, || {
            alloc.lower(
                size,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
                CpuMap::WriteCombined,
            )
        })?;
        Self::from_backing(backing, blocks, runs)
    }
    fn from_backing(mut backing: KernelObject, blocks: usize, runs: RunLayout) -> Result<Self> {
        let base = backing.gpu_va();
        backing.initialize::<PoolTables>(0, |tables| tables.init(base, blocks, runs))?;
        Ok(Self {
            backing,
            layout: PageLayout::Neo { blocks, runs },
        })
    }
    /// Fresh compute backing at the admitted capacity, built off-lock, in the
    /// capacity's family format.
    pub(crate) fn for_compute(alloc: &Allocator<'_>, capacity: ComputeCapacity) -> Result<Self> {
        match capacity {
            ComputeCapacity::Blocks(blocks) => Self::compute_sized(alloc, blocks),
            ComputeCapacity::M5(geometry) => Self::compute_m5(alloc, geometry),
        }
    }
    fn compute_sized(alloc: &Allocator<'_>, blocks: usize) -> Result<Self> {
        if blocks < COMPUTE_BLOCKS || blocks > PAGE_LIST_SIZE / (FREE_LIST_BLOCK_PAGES as usize * 8) {
            return Err(EINVAL);
        }
        Self::new(alloc, blocks, RunLayout::Compute)
    }
    /// Fresh zeroed M5 compute backing, built without the device mutex. M5 keeps
    /// its directory, block list and initial eight render blocks in one retained
    /// allocation.
    fn compute_m5(alloc: &Allocator<'_>, required: Option<super::fw::m5::pool::PoolGeometry>) -> Result<Self> {
        use super::fw::m5::pool;
        let geometry = match required { Some(geometry) => geometry,
            None => pool::PoolGeometry::g17s(8).map_err(|_| EINVAL)?, };
        Self::m5(alloc, geometry)
    }
    fn m5(alloc: &Allocator<'_>, geometry: super::fw::m5::pool::PoolGeometry) -> Result<Self> {
        let tables = geometry.directory_bytes.checked_add(geometry.freelist_bytes).ok_or(EOVERFLOW)?;
        let size = geometry.blocks.checked_mul(geometry.block_bytes())
            .and_then(|n| n.checked_add(tables)).ok_or(EOVERFLOW)?;
        let backing = alloc.lower(size, mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_SHARED_RW, CpuMap::WriteCombined)?;
        let mut pages = Self { backing, layout: PageLayout::M5(geometry) };
        pages.populate_m5()?;
        Ok(pages)
    }
    fn populate_m5(&mut self) -> Result {
        use super::fw::m5::pool;
        let PageLayout::M5(geometry) = self.layout else { return Err(EINVAL); };
        let tables = geometry.directory_bytes.checked_add(geometry.freelist_bytes).ok_or(EOVERFLOW)?;
        let base = self.backing.gpu_va();
        let mut blocks = KVec::with_capacity(geometry.blocks, GFP_KERNEL)?;
        for index in 0..geometry.blocks {
            blocks.push(base + (tables + index * geometry.block_bytes()) as u64, GFP_KERNEL)?;
        }
        pool::encode_directory(geometry, &blocks,
            &mut self.backing.bytes_mut()[..geometry.directory_bytes]).map_err(|_| EINVAL)?;
        pool::encode_freelist(geometry, &blocks,
            &mut self.backing.bytes_mut()[geometry.directory_bytes..tables]).map_err(|_| EINVAL)?;
        Ok(())
    }
    fn reset_for_client(&mut self) -> Result {
        // Released storage may contain the previous client's private data.
        // Clear it before exposing an alias in a different client VM.
        self.backing.bytes_mut().fill(0);
        self.populate_m5()?;
        fence(Ordering::SeqCst);
        Ok(())
    }
    pub(crate) fn map_client(&self, vm: &mmu::Vm) -> Result<mmu::KernelMapping> {
        self.backing.map_alias(vm, self.page_list_va(), mmu::PROT_GPU_SHARED_RW)
    }
    fn fits(&self, required: Option<super::fw::m5::pool::PoolGeometry>) -> bool {
        match (self.layout, required) {
            (PageLayout::M5(current), Some(required)) =>
                current.block_bytes() == required.block_bytes() && current.blocks >= required.blocks,
            (_, None) => true,
            _ => false,
        }
    }
    /// Whether this backing holds `required`; backing of the other family never does.
    pub(crate) fn fits_capacity(&self, required: ComputeCapacity) -> bool {
        match (self.layout, required) {
            (PageLayout::Neo { blocks, .. }, ComputeCapacity::Blocks(required)) => blocks >= required,
            (PageLayout::M5(_), ComputeCapacity::M5(geometry)) => self.fits(geometry),
            _ => false,
        }
    }
    fn is_m5(&self) -> bool { matches!(self.layout, PageLayout::M5(_)) }
    pub(crate) fn page_list_va(&self) -> u64 { self.backing.gpu_va() }
    fn run_list_va(&self) -> u64 {
        self.page_list_va() + match self.layout {
            PageLayout::Neo { .. } => PAGE_LIST_SIZE,
            PageLayout::M5(g) => g.directory_bytes,
        } as u64
    }
    fn m5_layout(&self, slot: u16) -> Result<super::fw::m5::pool::PoolLayout> {
        let PageLayout::M5(geometry) = self.layout else { return Err(EINVAL); };
        Ok(super::fw::m5::pool::PoolLayout { geometry, directory: self.page_list_va(),
            freelist: self.run_list_va(), hardware_id: u32::from(slot) })
    }
    /// The descriptor row of this backing installed at `slot`.
    pub(crate) fn descriptor(&self, slot: u16) -> Result<PoolDescriptor> {
        match self.layout {
            PageLayout::M5(_) => {
                let words = super::fw::m5::pool::initial_uma_descriptor(self.m5_layout(slot)?)
                    .map_err(|_| EINVAL)?;
                Ok(PoolDescriptor::M5 { slot, words })
            }
            PageLayout::Neo { blocks, .. } =>
                Ok(PoolDescriptor::Neo { slot, page_list: self.page_list_va(), blocks }),
        }
    }
    fn repopulate(&self) -> Result {
        let PageLayout::Neo { blocks, runs } = self.layout else { return Err(EINVAL); };
        let table = self.backing.pointer(0, TABLE_SIZE)?.cast::<u64>();
        PoolTables::for_each_word(
            self.page_list_va(),
            blocks,
            runs,
            |index, value| {
                // SAFETY: The page-aligned backing was validated for the complete
                // tables. The enumerator bounds every index inside those tables;
                // the owner has proven firmware release before this update.
                unsafe { table.add(index).write_volatile(value) };
            },
        )
    }
}

/// Pool-object identities are independent of recycled firmware IDs and VAs.
static RENDER_POOL_SERIAL: AtomicU64 = AtomicU64::new(0);

/// The USC backing of a render pool and its two aliases. Field order drops
/// both aliases before the backing.
pub(crate) struct RenderBacking {
    _client: mmu::KernelMapping,
    _global: Option<mmu::KernelMapping>,
    pages: Pages,
}

impl RenderBacking {
    fn into_pages(self) -> Pages {
        let Self { _client, _global, pages } = self;
        drop(_client);
        drop(_global);
        pages
    }
    /// Fresh populated backing, built without the device mutex.
    pub(crate) fn new(
        alloc: &Allocator<'_>,
        render_global: &mmu::Vm,
        client: &mmu::Vm,
        blocks: usize,
    ) -> Result<Self> {
        if blocks < RENDER_BLOCKS || blocks > PAGE_LIST_SIZE / (FREE_LIST_BLOCK_PAGES as usize * 8) {
            return Err(EINVAL);
        }
        let size = TABLE_SIZE + blocks * FREE_LIST_BLOCK_SIZE;
        let (backing, [global, client]) = place_lower(size, || {
            alloc.lower_with_aliases(
                size,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_GPU_SHARED_RW,
                CpuMap::WriteCombined,
                [render_global, client],
            )
        })?;
        let pages = Pages::from_backing(backing, blocks, RunLayout::Render)?;
        Ok(Self {
            _client: client,
            _global: Some(global),
            pages,
        })
    }
    /// Fresh M5 backing, aliased into the client VM only: M5 firmware uses
    /// the canonical and client mappings of these pages.
    fn m5(alloc: &Allocator<'_>, client: &mmu::Vm,
        geometry: super::fw::m5::pool::PoolGeometry) -> Result<Self> {
        let pages = Pages::m5(alloc, geometry)?;
        let client = pages.map_client(client)?;
        Ok(Self { _client: client, _global: None, pages })
    }
}

#[pin_data]
struct RenderStorage {
    #[pin]
    backing: Mutex<Option<RenderBacking>>,
    #[pin]
    spare: Mutex<Option<Pages>>,
    page_list: AtomicU64,
}
impl RenderStorage {
    fn new(backing: Option<RenderBacking>) -> Result<Arc<Self>> {
        let address = backing.as_ref().map_or(0, |backing| backing.pages.page_list_va());
        Arc::pin_init(try_pin_init!(Self {
            backing <- new_mutex!(backing, "G17 render pool backing"),
            spare <- new_mutex!(None, "G17 render pool spare"),
            page_list: AtomicU64::new(address),
        }), GFP_KERNEL)
    }
}

/// A render pool retained by a logical context and every graph referencing it.
/// Its backing can be given back while the pool's free-list release has been
/// consumed and the client VM is evicted; a fresh backing is installed before
/// the pool is staged again.
pub(crate) struct RenderPool {
    storage: Arc<RenderStorage>,
    /// Counts installed backings, so a re-armed backing is a new free-list
    /// owner even if its addresses repeat.
    epoch: AtomicU64,
    /// Residency of the client VM the backing is aliased into.
    client_residency: Option<Arc<mmu::ResidencyGate>>,
    /// The one VM (`mmu::Vm::id`) this pool's backing is aliased into.
    client_vm: u64,
    control: KernelObject,
    state: KernelObject,
    served: AtomicBool,
    serial: u64,
    id: RenderId,
    layout: PageLayout,
}

/// Keeps both a physical identity and its logical pool alive during use.
/// Drop the identity pin before releasing the retained pool reference.
pub(crate) struct PoolPin {
    _identity: RenderIdPin,
    _pool: Arc<RenderPool>,
}

impl RenderPool {
    /// The caller retains this guard across construction and all firmware use.
    pub(crate) fn pin_id(self: &Arc<Self>) -> Result<Option<PoolPin>> {
        let pin = self.id.pin(|| {
            self.epoch.fetch_add(1, Ordering::AcqRel);
            self.served.store(true, Ordering::Release);
        })?;
        Ok(pin.map(|pin| PoolPin { _identity: pin, _pool: self.clone() }))
    }

    pub(crate) fn new(
        alloc: &Allocator<'_>,
        ids: &Arc<RenderIds>,
        render_global: &mmu::Vm,
        client: &mmu::Vm,
        blocks: usize,
    ) -> Result<Self> {
        let id = RenderId::new(ids)?;
        let backing = RenderBacking::new(alloc, render_global, client, blocks)?;
        Self::with_backing(alloc, id, client, backing)
    }

    /// Explicit initial geometry; allocation policy remains with the caller.
    /// M5 firmware uses the canonical and client mappings of these pages.
    pub(super) fn m5(alloc: &Allocator<'_>, id: RenderId, client: &mmu::Vm,
        geometry: super::fw::m5::pool::PoolGeometry) -> Result<Self> {
        if !alloc.uat.is_m5() { return Err(EINVAL); }
        // Logical metadata is cheap; physical backing is installed only once
        // preparation owns both an identity pin and admission capacity.
        let storage = id.storage().ok_or(EINVAL)?;
        let state = alloc.kernel(CONTROL_SIZE, mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW, CpuMap::WriteCombined)?;
        let control = alloc.kernel(CONTROL_SIZE, mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW, CpuMap::WriteCombined)?;
        let serial = RENDER_POOL_SERIAL.fetch_update(Ordering::Relaxed, Ordering::Relaxed,
            |value| value.checked_add(1)).map_err(|_| EOVERFLOW)? + 1;
        Ok(Self { storage, epoch: AtomicU64::new(0), client_residency: client.residency_gate(),
            client_vm: client.id(),
            control, state, served: AtomicBool::new(true), serial, id,
            layout: PageLayout::M5(geometry) })
    }

    fn with_backing(alloc: &Allocator<'_>, id: RenderId, client: &mmu::Vm,
        backing: RenderBacking) -> Result<Self> {
        let layout = backing.pages.layout;
        let address = backing.pages.page_list_va();
        let run_list_va = backing.pages.run_list_va();
        let state = alloc.kernel(
            CONTROL_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let mut control = alloc.kernel(
            CONTROL_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        match layout {
            PageLayout::M5(_) => {
                super::fw::m5::pool::encode_g17s_render_state(backing.pages.m5_layout(id.slot())?,
                    state.gpu_va(), control.bytes_mut()).map_err(|_| EINVAL)?;
            }
            PageLayout::Neo { blocks, .. } => control.write(
                0,
                FreeListControl::render(
                    &FreeListArgs {
                        buffer_slot: u32::from(id.slot()),
                        class: FreeListClass::Two,
                        page_list_va: address,
                        run_list_va,
                        blocks: blocks as u32,
                        state_va: state.gpu_va(),
                    },
                    0,
                ),
            )?,
        }
        fence(Ordering::SeqCst);
        let serial = RENDER_POOL_SERIAL
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| value.checked_add(1))
            .map_err(|_| EOVERFLOW)? + 1;
        let storage = RenderStorage::new(Some(backing))?;
        Ok(Self {
            storage,
            epoch: AtomicU64::new(0),
            client_residency: client.residency_gate(),
            client_vm: client.id(),
            control,
            state,
            served: AtomicBool::new(false),
            serial,
            id,
            layout,
        })
    }

    /// No backing is installed: the pool must be re-armed before staging.
    pub(crate) fn vacant(&self) -> bool {
        self.storage.page_list.load(Ordering::Acquire) == 0
    }
    pub(crate) fn needs_storage(&self) -> bool {
        self.vacant() && self.storage.spare.lock().is_none()
    }
    /// Pressure reclaim evicted the client VM, which is idle.
    pub(crate) fn client_evicted(&self) -> bool {
        self.client_residency.as_ref().is_some_and(|gate| gate.evicted())
    }
    /// Removes the backing. The caller holds the device mutex and observed this
    /// pool's free-list release consumed, so no firmware reference remains; the
    /// next staging requires a re-armed backing. The caller drops the result
    /// after releasing the device mutex.
    pub(crate) fn vacate(&self) -> Option<RenderBacking> {
        let backing = self.storage.backing.lock().take();
        if backing.is_some() {
            self.storage.page_list.store(0, Ordering::Release);
        }
        backing
    }
    /// Installs fresh backing into a vacant pool, unless another caller did.
    /// The next staging rewrites the control header and descriptor row for it
    /// (its run-list address changes the free-list owner identity).
    pub(crate) fn rearm(&self, fresh: RenderBacking) -> Result {
        let mut backing = self.storage.backing.lock();
        if backing.is_some() {
            return Ok(());
        }
        let address = fresh.pages.page_list_va();
        *backing = Some(fresh);
        self.epoch.fetch_add(1, Ordering::AcqRel);
        self.served.store(true, Ordering::Release);
        self.storage.page_list.store(address, Ordering::Release);
        Ok(())
    }
    /// Keep released physical storage for the incoming owner, without keeping
    /// the old VM alias or assigning backing to every logical context.
    pub(crate) fn cache_reclaimed(&self, reclaimed: ReclaimedRenderId) {
        let pages = reclaimed.into_pages();
        let backing = self.storage.backing.lock();
        let mut spare = self.storage.spare.lock();
        let mut pages = pages;
        if backing.is_none() && spare.is_none()
            && pages.as_ref().is_some_and(|pages| pages.layout == self.layout) {
            *spare = pages.take();
        }
        drop(spare);
        drop(backing);
        drop(pages);
    }
    /// Re-arms a vacant pool with fresh backing built here, without the device
    /// mutex. `client` is the VM of the context that owns this pool; EPERM
    /// for any other VM.
    pub(crate) fn ensure_backing(
        &self,
        alloc: &Allocator<'_>,
        render_global: &mmu::Vm,
        client: &mmu::Vm,
    ) -> Result {
        if client.id() != self.client_vm {
            return Err(EPERM);
        }
        let geometry = match self.layout {
            PageLayout::M5(geometry) => geometry,
            PageLayout::Neo { blocks, .. } => {
                if !self.vacant() {
                    return Ok(());
                }
                return self.rearm(RenderBacking::new(alloc, render_global, client, blocks)?);
            }
        };
        let mut backing = self.storage.backing.lock();
        if backing.is_some() { return Ok(()); }
        let spare = self.storage.spare.lock().take();
        let fresh = if let Some(mut pages) = spare {
            pages.reset_for_client()?;
            let alias = pages.map_client(client)?;
            RenderBacking { _client: alias, _global: None, pages }
        } else {
            RenderBacking::m5(alloc, client, geometry)?
        };
        let address = fresh.pages.page_list_va();
        *backing = Some(fresh);
        self.epoch.fetch_add(1, Ordering::AcqRel);
        self.served.store(true, Ordering::Release);
        self.storage.page_list.store(address, Ordering::Release);
        Ok(())
    }

    pub(crate) fn serial(&self) -> u64 {
        self.serial
    }
    pub(crate) fn epoch(&self) -> u64 {
        self.epoch.load(Ordering::Acquire)
    }
    pub(crate) fn id(&self) -> u16 {
        self.id.slot()
    }
    pub(crate) fn control_va(&self) -> u64 {
        self.control.gpu_va()
    }
    /// Zero while vacant.
    pub(crate) fn page_list_va(&self) -> u64 {
        self.storage.page_list.load(Ordering::Acquire)
    }
    pub(crate) fn run_list_va(&self) -> u64 {
        self.page_list_va() + match self.layout {
            PageLayout::Neo { .. } => PAGE_LIST_SIZE,
            PageLayout::M5(g) => g.directory_bytes,
        } as u64
    }
    pub(crate) fn end_va(&self) -> u64 {
        self.run_list_va() + match self.layout {
            PageLayout::Neo { blocks, .. } => blocks * size_of::<super::fw::initdata::FreeListRunSlot>(),
            PageLayout::M5(g) => g.blocks * 8,
        } as u64
    }

    /// Fresh pools are already populated. Reuse requires the previous release
    /// to have been consumed; command completion alone does not establish it.
    pub(crate) fn repopulate(&self) -> Result {
        if !self.served.swap(true, Ordering::AcqRel) {
            return Ok(());
        }
        let submitted = self.state.dword(0)?.load(Ordering::Relaxed);
        let blocks = match self.layout {
            PageLayout::Neo { blocks, .. } => blocks,
            PageLayout::M5(_) => {
                let backing = self.storage.backing.lock();
                let pages = &backing.as_ref().ok_or(EAGAIN)?.pages;
                if pages.page_list_va() != self.page_list_va() { return Err(EIO); }
                let mut bytes = [0; super::fw::m5::pool::STATE_BYTES];
                super::fw::m5::pool::encode_g17s_render_state(pages.m5_layout(self.id())?,
                    self.state.gpu_va(), &mut bytes).map_err(|_| EINVAL)?;
                bytes[super::fw::m5::pool::COMPLETED..super::fw::m5::pool::COMPLETED + 8]
                    .copy_from_slice(&submitted.to_le_bytes());
                let raw = self.control.pointer(0, bytes.len())?;
                for (index, byte) in bytes.into_iter().enumerate() {
                    // SAFETY: The whole retained range was checked and the caller
                    // observed final release before installing fresh pool backing.
                    unsafe { raw.add(index).write_volatile(byte) };
                }
                fence(Ordering::SeqCst);
                return Ok(());
            }
        };
        let header = FreeListControl::render(
            &FreeListArgs {
                buffer_slot: u32::from(self.id()),
                class: FreeListClass::Two,
                page_list_va: self.page_list_va(),
                run_list_va: self.run_list_va(),
                blocks: blocks as u32,
                state_va: self.state.gpu_va(),
            },
            submitted,
        );
        let mut image = KVVec::from_elem(0u64, CONTROL_SIZE / 8, GFP_KERNEL)?;
        // SAFETY: `image` owns a zeroed control page, is large enough for the
        // header, and every header byte is initialized integer data.
        unsafe {
            image
                .as_mut_ptr()
                .cast::<FreeListControl>()
                .write_unaligned(header)
        };
        let control = self.control.pointer(0, CONTROL_SIZE)?.cast::<u64>();
        let backing = self.storage.backing.lock();
        let pages = &backing.as_ref().ok_or(EAGAIN)?.pages;
        if pages.page_list_va() != self.page_list_va() {
            return Err(EIO);
        }
        pages.repopulate()?;
        for (index, value) in image.iter().enumerate() {
            // SAFETY: The entire retained page was checked before any store;
            // the allocation is page-aligned and release excludes the firmware.
            unsafe { control.add(index).write_volatile(*value) };
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Submitted is host-owned and aligned. Completed is the packed firmware
    /// counter, so retain the byte accesses required by that interface.
    pub(crate) fn counters(&self) -> Result<[u64; 2]> {
        let submitted = self.state.dword(0)?.load(Ordering::Relaxed);
        let offset = if self.is_m5() { super::fw::m5::pool::COMPLETED }
            else { FreeListControl::COMPLETED };
        let completed = self.control.pointer(offset, 8)?;
        let mut bytes = [0; 8];
        for (index, byte) in bytes.iter_mut().enumerate() {
            // SAFETY: The checked eight-byte range stays mapped; byte accesses
            // permit the unaligned counter without widening shared accesses.
            *byte = unsafe { completed.add(index).read_volatile() };
        }
        Ok([submitted, u64::from_le_bytes(bytes)])
    }

    pub(crate) fn fits(&self, geometry: Option<super::fw::m5::pool::PoolGeometry>) -> bool {
        match (self.layout, geometry) {
            (PageLayout::M5(current), Some(required)) =>
                current.block_bytes() == required.block_bytes() && current.blocks >= required.blocks,
            (PageLayout::Neo { .. }, None) => true,
            _ => false,
        }
    }

    pub(crate) fn is_m5(&self) -> bool { matches!(self.layout, PageLayout::M5(_)) }

    fn m5_state(&self) -> Result<super::fw::m5::pool::PoolState> {
        if !self.is_m5() { return Err(EINVAL); }
        let mut state = super::fw::m5::pool::PoolState([0; super::fw::m5::pool::STATE_BYTES]);
        let raw = self.control.pointer(0, state.0.len())?;
        fence(Ordering::Acquire);
        for (index, byte) in state.0.iter_mut().enumerate() {
            // SAFETY: The entire retained header range was checked above.
            *byte = unsafe { raw.add(index).read_volatile() };
        }
        Ok(state)
    }

    pub(crate) fn m5_descriptor(&self) -> Result<[u64; 8]> { Ok(self.m5_state()?.descriptor()) }

    pub(crate) fn store_m5_descriptor(&self, words: [u64; 8]) -> Result {
        let mut state = self.m5_state()?;
        if !state.matches(&words) { return Err(EIO); }
        state.restore(words);
        for offset in super::fw::m5::pool::SAVED_FIELDS {
            let raw = self.control.pointer(offset, 4)?;
            for index in 0..4 {
                // SAFETY: Each mutable field was checked; the caller owns the
                // retired descriptor, and immutable header bytes stay intact.
                unsafe { raw.add(index).write_volatile(state.0[offset + index]) };
            }
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Snapshot the unaligned shared fields before loading their descriptor.
    pub(crate) fn page_pool_state(&self) -> Result<PagePoolState> {
        let raw = self.control.pointer(0x14, 0x1c)?;
        fence(Ordering::Acquire);
        let read = |offset: usize, count: usize| -> u64 {
            let mut bytes = [0; 8];
            for (index, byte) in bytes.iter_mut().take(count).enumerate() {
                // SAFETY: The complete 0x14..0x30 span was checked. These fixed
                // field ranges lie inside it; byte loads permit unaligned words.
                *byte = unsafe { raw.add(offset + index).read_volatile() };
            }
            u64::from_le_bytes(bytes)
        };
        Ok(PagePoolState {
            page_list_va: read(0, 8),
            unk_1c: read(8, 4) as u32,
            unk_20: read(12, 4) as u32,
            unk_24: read(16, 4) as u32,
            unk_28: read(20, 4) as u32,
            page_count: read(24, 4) as u32,
        })
    }

    /// Save mutable descriptor state only after both engines and free-list
    /// references retire, before the physical slot changes owners.
    pub(crate) fn store_page_pool_state(&self, state: PagePoolState) -> Result {
        let raw = self.control.pointer(0x20, 0x10)?;
        for (word, value) in [state.unk_20, state.unk_24, state.unk_28, state.page_count]
            .into_iter()
            .enumerate()
        {
            for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
                // SAFETY: The entire mutable range was checked, and each
                // enumerated byte lies inside it. Caller owns the retired pool.
                unsafe { raw.add(word * 4 + index).write_volatile(byte) };
            }
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Prepares the counter before any producer is written. Commit belongs to
    /// the publication transaction; dropping this plan does not reserve work.
    pub(crate) fn prepare_submission(&self, commands: u32) -> Result<Submitted<'_>> {
        if commands > 2 {
            return Err(EINVAL);
        }
        let counter = self.state.dword(0)?;
        Ok(Submitted {
            counter,
            next: counter
                .load(Ordering::Relaxed)
                .wrapping_add(u64::from(commands)),
        })
    }
}

/// One checked, infallible counter update in a submission transaction.
pub(crate) struct Submitted<'a> {
    counter: &'a AtomicU64,
    next: u64,
}
impl Submitted<'_> {
    pub(crate) fn commit(self) {
        self.counter.store(self.next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
    }
}

/// A pool owned by one physical compute queue, with fixed QID-derived slot.
/// Idle retained queues may give their USC backing back (`pages` is `None`)
/// once the firmware consumed the pool's release; the control and state
/// objects the queue record names stay until processor stop.
///
/// A backing serves one VM: its blocks hold the stack and scratch memory of
/// that VM's work, so a view of it in another VM is refused
/// ([`Self::map_client`]). Another VM gets the queue only after the backing
/// was given back and fresh, zeroed backing installed ([`Self::rearm`]).
pub(crate) struct ComputePool {
    pages: Option<Pages>,
    /// The VM (`mmu::Vm::id`) the backing serves, from its first client view
    /// until it is given back; zero while no client has had a view of it.
    client_vm: u64,
    control: KernelObject,
    state: KernelObject,
    id: u16,
    m5: bool,
}
impl ComputePool {
    /// Capacity check for admission and growth.
    pub(crate) fn fits_capacity(&self, required: ComputeCapacity) -> bool {
        self.pages.as_ref().is_some_and(|pages| pages.fits_capacity(required))
    }

    /// A pool at the admitted capacity, in the capacity's family format.
    pub(crate) fn new(alloc: &Allocator<'_>, qid: u8, required: ComputeCapacity) -> Result<Self> {
        if qid > super::fw::kick::QID_MAX {
            return Err(EINVAL);
        }
        let pages = Pages::for_compute(alloc, required)?;
        let m5 = pages.is_m5();
        Self::with_pages(alloc, qid, m5, pages)
    }
    fn with_pages(alloc: &Allocator<'_>, qid: u8, m5: bool, pages: Pages) -> Result<Self> {
        let id = u16::from(qid) + 1;
        let mut control = alloc.kernel(
            CONTROL_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let state = alloc.kernel(
            CONTROL_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        Self::initialize_control(&mut control, &pages, id, state.gpu_va(), [0; 8])?;
        fence(Ordering::SeqCst);
        Ok(Self { pages: Some(pages), client_vm: 0, control, state, id, m5 })
    }
    fn initialize_control(control: &mut KernelObject, pages: &Pages, id: u16,
        submitted: u64, completed: [u8; 8]) -> Result {
        match pages.layout {
            PageLayout::M5(_) => {
                use super::fw::m5::pool;
                pool::encode_g17s_compute_state(pages.m5_layout(id)?, submitted,
                    &mut control.bytes_mut()[..pool::STATE_BYTES]).map_err(|_| EINVAL)?;
                control.bytes_mut()[pool::COMPLETED..pool::COMPLETED + 8].copy_from_slice(&completed);
                Ok(())
            }
            PageLayout::Neo { blocks, .. } => {
                let mut header = FreeListControl::compute(&FreeListArgs {
                    buffer_slot: u32::from(id), class: FreeListClass::One,
                    page_list_va: pages.page_list_va(), run_list_va: pages.run_list_va(),
                    blocks: blocks as u32, state_va: submitted,
                });
                header.unk_54[..8].copy_from_slice(&completed);
                control.write(0, header)
            }
        }
    }
    pub(crate) fn descriptor(&self) -> Result<PoolDescriptor> {
        self.pages.as_ref().ok_or(EAGAIN)?.descriptor(self.id)
    }
    pub(crate) fn completed_submissions(&self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.completed_bytes()?))
    }
    fn completed_bytes(&self) -> Result<[u8; 8]> {
        let offset = if self.m5 { super::fw::m5::pool::COMPLETED } else { FreeListControl::COMPLETED };
        let pointer = self.control.pointer(offset, 8)?;
        let mut bytes = [0; 8];
        for (index, byte) in bytes.iter_mut().enumerate() {
            // SAFETY: The checked packed counter remains mapped. Firmware may
            // update it; callers use it only with the queue's completion proof.
            *byte = unsafe { pointer.add(index).read_volatile() };
        }
        Ok(bytes)
    }
    pub(crate) fn id(&self) -> u16 {
        self.id
    }
    pub(crate) fn control_va(&self) -> u64 {
        self.control.gpu_va()
    }
    pub(crate) fn fits(&self, required: Option<super::fw::m5::pool::PoolGeometry>) -> bool {
        self.pages.as_ref().is_some_and(|pages| pages.fits(required))
    }
    pub(crate) fn populated(&self) -> bool {
        self.pages.is_some()
    }
    /// Whether a view of the backing may be given to `vm`.
    pub(crate) fn serves(&self, vm: &mmu::Vm) -> bool {
        self.client_vm == 0 || self.client_vm == vm.id()
    }
    /// The release record for this pool's buffer slot under the free-list
    /// generation the queue last published.
    pub(crate) fn release_record(&self, generation: u64) -> super::fw::channels::FreeListRelease {
        super::fw::channels::FreeListRelease::new(generation, u32::from(self.id))
    }
    /// The caller has observed the firmware consume this pool's release and
    /// frees the returned backing after releasing the device mutex.
    pub(crate) fn take_pages(&mut self) -> Option<Pages> {
        self.client_vm = 0;
        self.pages.take()
    }
    /// Rewrites the shared control header for fresh backing. The caller then
    /// rewrites the page-pool descriptor row before the queue is kicked again.
    /// As render repopulation does, the firmware-completed counter is carried
    /// over so it stays consistent with the host submitted counter in `state`.
    pub(crate) fn rearm(&mut self, pages: Pages) -> Result {
        if self.pages.is_some() {
            return Err(EBUSY);
        }
        self.write_pages(&pages)?;
        self.pages = Some(pages);
        Ok(())
    }
    /// Preserve both sets of pages on failure. The caller owns the consumed
    /// release and returns old pages for destruction outside the device mutex.
    /// The grown backing's alias is in the VM the old backing served, which it
    /// keeps serving.
    pub(crate) fn replace_pages(&mut self, pages: &mut Option<Pages>) -> Result {
        self.write_pages(pages.as_ref().ok_or(EINVAL)?)?;
        core::mem::swap(&mut self.pages, pages);
        Ok(())
    }
    fn write_pages(&mut self, pages: &Pages) -> Result {
        if pages.is_m5() != self.m5 { return Err(EINVAL); }
        let completed = self.completed_bytes()?;
        Self::initialize_control(&mut self.control, pages, self.id, self.state.gpu_va(), completed)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// EPERM for a VM other than the one the backing serves.
    pub(crate) fn map_client(&mut self, vm: &mmu::Vm) -> Result<mmu::KernelMapping> {
        if !self.serves(vm) {
            return Err(EPERM);
        }
        let pages = self.pages.as_mut().ok_or(EAGAIN)?;
        let address = pages.page_list_va();
        let alias = pages
            .backing
            .map_alias(vm, address, mmu::PROT_GPU_SHARED_RW)?;
        self.client_vm = vm.id();
        Ok(alias)
    }
    pub(crate) fn reserve_submission(&self) -> Result<u64> {
        let counter = self.state.dword(0)?;
        let next = counter
            .load(Ordering::Relaxed)
            .checked_add(1)
            .ok_or(EOVERFLOW)?;
        counter.store(next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(next)
    }
    /// The caller has proved no pointer from this submission was published.
    pub(crate) fn cancel_unpublished(&self, target: u64) -> Result {
        let counter = self.state.dword(0)?;
        if target == 0 || counter.load(Ordering::Acquire) != target {
            return Err(EINVAL);
        }
        counter.store(target - 1, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(())
    }
}

/// Registration identity held by a free-list lease. The cookie is the end of
/// the run list; it remains stable while overlapping command pairs share it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Owner {
    /// Identifies the host pool object even when every firmware address recycles.
    pub(crate) serial: u64,
    /// Identifies the pool's installed backing (render pools are re-armed).
    pub(crate) epoch: u64,
    pub(crate) buffer_slot: u32,
    pub(crate) control_va: u64,
    pub(crate) ring_va: u64,
    pub(crate) cookie: u64,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum LeasePhase {
    Preparing,
    Ready,
    /// No reference since the given time. The registration stays live so that
    /// the owner's next render reuses it without a control-ring release; the
    /// release becomes pending when the linger ends.
    Lingering(u64),
    ReleaseSent(u32),
    Released,
}

/// How long an idle registration of an open owner may stay live: the
/// firmware's effort idle hold.
pub(crate) const LINGER_NS: u64 = 16_000_000;

/// References to one prepopulated owner: preparation and two engine commands
/// per render. Published releases do not gate reacquisition; their consumption
/// witnesses are carried into the successor.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Lease {
    owner: Owner,
    generation: u64,
    references: u32,
    commands: u32,
    phase: LeasePhase,
    previous_release: Option<u32>,
}

impl Lease {
    /// Acquires an unchanged overlapping owner, or an owner whose previous
    /// release was sent. A changed owner additionally requires caller-side
    /// proof of old-pool release before its backing can be repopulated.
    pub(crate) fn acquire(previous: Option<&Self>, owner: Owner) -> Result<Self> {
        let Some(previous) = previous else {
            return Ok(Self {
                owner,
                generation: 0,
                references: 3,
                commands: 2,
                phase: LeasePhase::Preparing,
                previous_release: None,
            });
        };
        if previous.owner.buffer_slot != owner.buffer_slot {
            return Err(EINVAL);
        }
        let generation = previous.generation.checked_add(1).ok_or(EOVERFLOW)?;
        let same_owner = previous.owner == owner;
        // A lingering registration was never released: it continues like an
        // overlap of the owner's previous render.
        let overlapping = same_owner
            && match previous.phase {
                LeasePhase::Ready => previous.references >= 2 && previous.commands >= 2,
                LeasePhase::Lingering(_) => true,
                _ => false,
            };
        let reacquire = previous.reacquirable();
        if !overlapping && !reacquire {
            return Err(EBUSY);
        }
        let references = previous.references.checked_add(3).ok_or(EOVERFLOW)?;
        let commands = previous.commands.checked_add(2).ok_or(EOVERFLOW)?;
        Ok(Self {
            owner,
            generation: if overlapping {
                previous.generation
            } else {
                generation
            },
            references,
            commands,
            phase: LeasePhase::Preparing,
            previous_release: previous.release_witness(),
        })
    }

    pub(crate) fn owner(&self) -> Owner {
        self.owner
    }
    pub(crate) fn generation(&self) -> u64 {
        self.generation
    }
    pub(crate) fn reacquirable(&self) -> bool {
        matches!(
            self.phase,
            LeasePhase::ReleaseSent(_) | LeasePhase::Released
        ) && self.references == 0
            && self.commands == 0
    }
    pub(crate) fn released(&self) -> bool {
        self.phase == LeasePhase::Released && self.previous_release.is_none()
    }
    pub(crate) fn release_witness(&self) -> Option<u32> {
        match self.phase {
            LeasePhase::ReleaseSent(cursor) => Some(cursor),
            _ => self.previous_release,
        }
    }

    /// Publication retires preparation independently of the command references.
    pub(crate) fn retire_prepare(&mut self) -> Result {
        if self.phase != LeasePhase::Preparing || self.references < 3 {
            return Err(EINVAL);
        }
        self.references -= 1;
        self.phase = LeasePhase::Ready;
        Ok(())
    }
    /// Fixed prepopulated pools never submit an asynchronous growth request.
    /// An incoming growth completion therefore cannot name a live request.
    pub(crate) fn complete_grow(
        &mut self,
        _event: super::fw::channels::FreeListCompletion,
    ) -> Result {
        Err(EIO)
    }
    /// Both engines have retired this render's exact retained submission.
    /// With `linger`, a registration left without references stays live
    /// instead of making its final release pending.
    pub(crate) fn complete_render(
        &mut self,
        linger: bool,
    ) -> Result<Option<super::fw::channels::FreeListRelease>> {
        if self.phase != LeasePhase::Ready || self.references < 2 || self.commands < 2 {
            return Err(EINVAL);
        }
        self.references -= 2;
        self.commands -= 2;
        if self.references == 0 && linger {
            self.phase = LeasePhase::Lingering(super::now_ns().max(1));
        }
        Ok(self.pending_release())
    }
    /// The time since which an idle registration lingers, if it does.
    pub(crate) fn lingering(&self) -> Option<u64> {
        match self.phase {
            LeasePhase::Lingering(since) => Some(since),
            _ => None,
        }
    }
    /// Ends a linger; the final release becomes pending.
    pub(crate) fn end_linger(&mut self) {
        if matches!(self.phase, LeasePhase::Lingering(_)) {
            self.phase = LeasePhase::Ready;
        }
    }
    pub(crate) fn pending_release(&self) -> Option<super::fw::channels::FreeListRelease> {
        (self.phase == LeasePhase::Ready && self.references == 0).then(|| {
            super::fw::channels::FreeListRelease::new(self.generation, self.owner.buffer_slot)
        })
    }
    /// Records the producer-after cursor once the final release was published.
    pub(crate) fn mark_release_sent(&mut self, cursor: u32) -> Result {
        if self.pending_release().is_none() || cursor >= super::fw::channels::DEVICE_CONTROL_SLOTS {
            return Err(EINVAL);
        }
        self.phase = LeasePhase::ReleaseSent(cursor);
        Ok(())
    }
    /// Only sent releases and inherited release witnesses consume cursors.
    pub(crate) fn needs_release_observation(&self) -> bool {
        self.previous_release.is_some() || matches!(self.phase, LeasePhase::ReleaseSent(_))
    }

    /// Observes all older release records passed by one consumer snapshot.
    pub(crate) fn observe_releases(&mut self, consumer: u32, producer: u32) -> Result {
        if let Some(cursor) = self.previous_release {
            if control_consumed(consumer, producer, cursor)? {
                self.previous_release = None;
            }
        }
        if let LeasePhase::ReleaseSent(cursor) = self.phase {
            if control_consumed(consumer, producer, cursor)? {
                self.phase = LeasePhase::Released;
            }
        }
        Ok(())
    }
}

/// A publication is retired once its producer-after cursor leaves the
/// current `(consumer, producer]` interval, including wrapped intervals.
pub(crate) fn control_consumed(consumer: u32, producer: u32, cursor: u32) -> Result<bool> {
    let slots = super::fw::channels::DEVICE_CONTROL_SLOTS;
    if consumer >= slots || producer >= slots || cursor >= slots {
        return Err(EIO);
    }
    let pending = producer.wrapping_sub(consumer) & (slots - 1);
    let ahead = cursor.wrapping_sub(consumer) & (slots - 1);
    Ok(ahead == 0 || ahead > pending)
}

impl RenderIds {
    pub(crate) fn used(&self) -> u32 {
        self.used[0].load(Ordering::Acquire).count_ones()
            + (self.used[1].load(Ordering::Acquire) & !(1u64 << 63)).count_ones()
    }
}

#[kunit_tests(asahi_g17_render_id_capacity)]
mod capacity_tests {
    use super::*;

    #[test]
    fn rejected_pool_construction_returns_all_render_ids() {
        let ids = RenderIds::new().unwrap();
        for _ in 0..8 {
            let mut pools = KVec::with_capacity(127, GFP_KERNEL).unwrap();
            for _ in 0..127 {
                pools.push(RenderId::new(&ids).unwrap(), GFP_KERNEL).unwrap();
            }
            assert_eq!(ids.used(), 127);
            assert!(matches!(RenderId::new(&ids), Err(ENOSPC)));
            drop(pools);
            assert_eq!(ids.used(), 0);
        }
    }
}
