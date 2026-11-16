// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Parameter-buffer IDs, scene ownership and sparse tiler heap growth.
//!
//! The device mutex serializes the shared ID allocator. A render graph owns
//! its buffer ID independently of the submission references using that graph;
//! a published command keeps its reference until firmware retirement is witnessed.

use super::fw::buffer as fw;
use crate::hw::t8140::resources as cfg;
pub(crate) mod target;

use kernel::prelude::*;

const BUFFER_IDS: usize = 127;

#[derive(Copy, Clone)]
struct BufferId {
    references: u8,
    owner: bool,
    token: u64,
}
impl BufferId {
    fn free(self) -> bool {
        self.references == 0 && !self.owner
    }
}

/// Device-wide identities for retained render parameter buffers.
pub(crate) struct BufferIds {
    entries: [BufferId; BUFFER_IDS],
    returned: [u8; BUFFER_IDS],
    returned_count: usize,
    next_token: u64,
}
impl BufferIds {
    pub(crate) fn new() -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            entries <- pin_init::init_array_from_fn(|_| BufferId {
                references: 0,
                owner: false,
                token: 0,
            }),
            returned <- pin_init::init_array_from_fn(|index| (BUFFER_IDS - index - 1) as u8),
            returned_count: BUFFER_IDS,
            next_token: 0,
        })
    }
    fn remove_returned(&mut self, id: u8) {
        if let Some(at) = self.returned[..self.returned_count]
            .iter()
            .position(|entry| *entry == id)
        {
            self.returned.copy_within(at + 1..self.returned_count, at);
            self.returned_count -= 1;
        }
    }
    fn return_id(&mut self, id: u8) {
        if !self.returned[..self.returned_count].contains(&id) && self.returned_count < BUFFER_IDS {
            self.returned[self.returned_count] = id;
            self.returned_count += 1;
        }
    }
    fn fresh_token(&mut self) -> u64 {
        let token = if self.next_token == u64::MAX {
            0
        } else {
            self.next_token
        };
        self.next_token = token + 1;
        token
    }
    pub(crate) fn allocate_render(&mut self) -> Result<(u8, u64)> {
        let id = if self.returned_count != 0 {
            self.returned_count -= 1;
            self.returned[self.returned_count]
        } else {
            self.entries
                .iter()
                .position(|entry| entry.free())
                .ok_or_else(|| super::capacity::refused(super::capacity::Site::ParameterBuffers))? as u8
        };
        if !self.entries[usize::from(id)].free() {
            return Err(EIO);
        }
        let token = self.fresh_token();
        self.entries[usize::from(id)] = BufferId {
            references: 1,
            owner: false,
            token,
        };
        Ok((id, token))
    }

    /// Retains the existing identity for a graph independently of its jobs.
    pub(crate) fn reserve_owner(&mut self, id: u8) -> Result {
        let entry = self.entries.get(usize::from(id)).ok_or(EINVAL)?;
        if entry.owner {
            return Ok(());
        }
        if entry.free() {
            let token = self.fresh_token();
            self.entries[usize::from(id)].token = token;
        }
        self.entries[usize::from(id)].owner = true;
        self.remove_returned(id);
        Ok(())
    }
    pub(crate) fn acquire_owner(&mut self, id: u8) -> Result<(u8, u64)> {
        let entry = self.entries.get_mut(usize::from(id)).ok_or(EINVAL)?;
        if !entry.owner {
            return Err(EINVAL);
        }
        entry.references = entry.references.checked_add(1).ok_or(ENOSPC)?;
        Ok((id, entry.token))
    }
    /// Preflight shared completion bookkeeping before any of its counters change.
    pub(crate) fn can_release(&self, id: u8) -> Result {
        let entry = self.entries.get(usize::from(id)).ok_or(EINVAL)?;
        if entry.references == 0 {
            return Err(EINVAL);
        }
        Ok(())
    }
    pub(crate) fn release(&mut self, id: u8) -> Result {
        let entry = self.entries.get_mut(usize::from(id)).ok_or(EINVAL)?;
        entry.references = entry.references.checked_sub(1).ok_or(EINVAL)?;
        if entry.free() {
            self.return_id(id);
        }
        Ok(())
    }
    pub(crate) fn unreserve_owner(&mut self, id: u8) -> Result {
        let entry = self.entries.get_mut(usize::from(id)).ok_or(EINVAL)?;
        if !entry.owner {
            return Ok(());
        }
        entry.owner = false;
        if entry.free() {
            self.return_id(id);
        }
        Ok(())
    }
}

/// Each scene and its shorter scratch slot are one lifetime. A completed
/// paired render releases both; merely publishing a later scene does not.
pub(crate) struct Scenes {
    owners: [Option<(u64, u8)>; fw::SCRATCH_SLOTS],
    next: usize,
}
impl Scenes {
    pub(crate) const fn new() -> Self {
        Self {
            owners: [None; fw::SCRATCH_SLOTS],
            next: fw::FIRST_SCENE,
        }
    }
    pub(crate) fn select(&self, ordinal: u64) -> Result<usize> {
        if let Some((_, scene)) = self
            .owners
            .iter()
            .flatten()
            .find(|(owner, _)| *owner == ordinal)
        {
            return Ok(usize::from(*scene));
        }
        (0..fw::SCENES)
            .map(|offset| (self.next + offset) % fw::SCENES)
            .find(|scene| self.owners[scene % fw::SCRATCH_SLOTS].is_none())
            .ok_or(ENOSPC)
    }
    pub(crate) fn commit(&mut self, ordinal: u64, scene: usize) -> Result {
        if self.select(ordinal)? != scene {
            return Err(EINVAL);
        }
        let slot = &mut self.owners[scene % fw::SCRATCH_SLOTS];
        if slot.is_none() {
            *slot = Some((ordinal, scene as u8));
            self.next = (scene + 1) % fw::SCENES;
        }
        Ok(())
    }
    pub(crate) fn release(&mut self, ordinal: u64) -> Result {
        let slot = self
            .owners
            .iter_mut()
            .find(|entry| entry.is_some_and(|(owner, _)| owner == ordinal))
            .ok_or(EINVAL)?;
        *slot = None;
        Ok(())
    }
}

/// A snapshot of the block ring; all values are modulo its fixed capacity.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Cursors {
    pub(crate) total: u32,
    pub(crate) committed: u32,
    pub(crate) read: u32,
}
impl Cursors {
    fn valid(self) -> bool {
        [self.total, self.committed, self.read]
            .into_iter()
            .all(|v| (v as usize) < cfg::TVB_BLOCK_SLOTS)
    }
    /// Firmware-paused growth permits consumption of the already committed
    /// prefix. Retired-host growth requires the complete snapshot unchanged.
    pub(crate) fn permits(self, after: Self, firmware_request: bool) -> bool {
        if !self.valid()
            || !after.valid()
            || self.total != after.total
            || self.committed != after.committed
        {
            return false;
        }
        if !firmware_request {
            return self == after;
        }
        let capacity = cfg::TVB_BLOCK_SLOTS as u32;
        (after.read + capacity - self.read) % capacity
            <= (self.committed + capacity - self.read) % capacity
    }
}

/// Checked block-table indices for one bulk allocation. One ring slot stays
/// empty even when all physical backing has been retained by the owner.
pub(crate) struct Growth {
    before: Cursors,
    added: usize,
    retained: usize,
    target: usize,
}
impl Growth {
    pub(crate) fn new(before: Cursors, added: usize) -> Result<Self> {
        if !before.valid() || before.total != before.committed || added == 0 {
            return Err(EINVAL);
        }
        let retained = (before.total as usize + cfg::TVB_BLOCK_SLOTS - before.read as usize)
            % cfg::TVB_BLOCK_SLOTS;
        if retained.checked_add(added).ok_or(EOVERFLOW)? >= cfg::TVB_BLOCK_SLOTS {
            return Err(ENOSPC);
        }
        let target = (before.total as usize + added) % cfg::TVB_BLOCK_SLOTS;
        Ok(Self {
            before,
            added,
            retained,
            target,
        })
    }
    pub(crate) fn target(&self) -> u32 {
        self.target as u32
    }
    pub(crate) fn retained(&self) -> usize {
        self.retained
    }
    pub(crate) fn available(&self) -> usize {
        self.retained + self.added
    }
    pub(crate) fn added_slot(&self, index: usize) -> Result<usize> {
        if index >= self.added {
            return Err(EINVAL);
        }
        Ok((self.before.total as usize + index) % cfg::TVB_BLOCK_SLOTS)
    }
    pub(crate) fn available_slot(&self, index: usize) -> Result<usize> {
        if index >= self.available() {
            return Err(EINVAL);
        }
        Ok((self.before.read as usize + index) % cfg::TVB_BLOCK_SLOTS)
    }
}

pub(crate) fn firmware_growth_target(current: usize) -> usize {
    current
        .saturating_add(fw::GROW_BLOCKS)
        .min(cfg::TVB_MAX_BLOCKS)
}

/// The high-water sample estimates capacity, not current occupancy. A growth
/// reserves three times the demand only once twice that demand no longer fits.
pub(crate) fn automatic_growth_target(used_pages: usize, current: usize) -> Option<usize> {
    let maximum = fw::NORMAL_MAX_BLOCKS;
    if current >= maximum {
        return None;
    }
    let need = used_pages
        .saturating_mul(2)
        .div_ceil(cfg::TVB_PAGES_PER_BLOCK)
        .min(maximum);
    if need <= current {
        return None;
    }
    Some(
        used_pages
            .saturating_mul(3)
            .div_ceil(cfg::TVB_PAGES_PER_BLOCK)
            .min(maximum),
    )
}

use super::object::{CpuMap, KernelObject};
use crate::{driver::AsahiDrmDevice, mmu};
use core::ops::Range;

pub(crate) fn compact_alias(
    object: &KernelObject,
    vm: &mmu::Vm,
    source: Range<usize>,
    align: u64,
    prot: mmu::Prot,
    guard: usize,
) -> Result<mmu::KernelMapping> {
    for range in cfg::compact_ranges() {
        match object.map_range(vm, source.clone(), range, align, prot, guard) {
            Ok(mapping) => return Ok(mapping),
            Err(error) if error == ENOSPC => {}
            Err(error) => return Err(error),
        }
    }
    Err(ENOSPC)
}

/// Allocated aliases remain private until the owner installs them in its VM
/// cache. On rollback mappings are destroyed before backing memory.
pub(crate) struct TvbGrowth {
    mappings: KVec<mmu::KernelMapping>,
    addresses: KVec<u64>,
    extension: KernelObject,
}

/// Aliases and backing of growth extensions a shrink removed from an idle
/// heap. Field order unmaps the aliases before freeing their backing.
pub(crate) struct TvbRelease {
    _mappings: KVec<mmu::KernelMapping>,
    _extensions: KVec<KernelObject>,
}

/// Packed 128-KiB data blocks mapped at guarded 160-KiB virtual strides.
/// Firmware owns their contents; CPU zeroing occurs only on fresh backing.
pub(crate) struct Tvb {
    addresses: KVec<u64>,
    // Retain sparse-block backing until the pool and its aliases retire.
    _primary: KernelObject,
    extensions: KVec<KernelObject>,
}
impl Tvb {
    /// Blocks are packed in the backing; only their VA placement keeps the
    /// stride and its trailing guard.
    fn bytes(blocks: usize) -> Result<usize> {
        blocks.checked_mul(cfg::TVB_BLOCK_SIZE).ok_or(EOVERFLOW)
    }
    fn map_block(
        object: &KernelObject,
        vm: &mmu::Vm,
        relative: usize,
    ) -> Result<mmu::KernelMapping> {
        let start = relative.checked_mul(cfg::TVB_BLOCK_SIZE).ok_or(EOVERFLOW)?;
        let source = start..start.checked_add(cfg::TVB_BLOCK_SIZE).ok_or(EOVERFLOW)?;
        let mapping = compact_alias(
            object,
            vm,
            source,
            cfg::TVB_PAGE,
            mmu::PROT_GPU_SHARED_RW,
            cfg::TVB_BLOCK_STRIDE as usize - cfg::TVB_BLOCK_SIZE,
        )?;
        fw::compact_page(mapping.iova())?;
        Ok(mapping)
    }
    pub(crate) fn new(
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        blocks: usize,
    ) -> Result<(Self, KVec<mmu::KernelMapping>)> {
        if blocks == 0 || blocks > cfg::TVB_MAX_BLOCKS {
            return Err(ERANGE);
        }
        let backing = KernelObject::backing(dev, Self::bytes(blocks)?, CpuMap::WriteCombined)?;
        let mut mappings = KVec::with_capacity(blocks, GFP_KERNEL)?;
        let mut addresses = KVec::with_capacity(blocks, GFP_KERNEL)?;
        for index in 0..blocks {
            let mapping = Self::map_block(&backing, vm, index)?;
            addresses.push(mapping.iova(), GFP_KERNEL)?;
            mappings.push(mapping, GFP_KERNEL)?;
        }
        Ok((
            Self {
                addresses,
                _primary: backing,
                extensions: KVec::new(),
            },
            mappings,
        ))
    }
    pub(crate) fn blocks(&self) -> usize {
        self.addresses.len()
    }
    pub(crate) fn addresses(&self) -> &[u64] {
        &self.addresses
    }
    /// Blocks of each growth extension; extensions are packed like the primary.
    fn extension_blocks(extension: &KernelObject) -> usize {
        extension.size() / cfg::TVB_BLOCK_SIZE
    }
    /// Blocks of the primary backing, which a heap never gives back.
    fn primary_blocks(&self) -> usize {
        self.extensions
            .iter()
            .fold(self.blocks(), |blocks, extension| {
                blocks.saturating_sub(Self::extension_blocks(extension))
            })
    }

    pub(crate) fn prepare_growth(
        &mut self,
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        target: usize,
    ) -> Result<Option<TvbGrowth>> {
        if target <= self.blocks() {
            return Ok(None);
        }
        if target > cfg::TVB_MAX_BLOCKS {
            return Err(ERANGE);
        }
        let added = target - self.blocks();
        self.extensions.reserve(1, GFP_KERNEL)?;
        self.addresses.reserve(added, GFP_KERNEL)?;
        let backing = KernelObject::backing(dev, Self::bytes(added)?, CpuMap::WriteCombined)?;
        let mut mappings = KVec::with_capacity(added, GFP_KERNEL)?;
        let mut addresses = KVec::with_capacity(added, GFP_KERNEL)?;
        for relative in 0..added {
            let mapping = Self::map_block(&backing, vm, relative)?;
            addresses.push(mapping.iova(), GFP_KERNEL)?;
            mappings.push(mapping, GFP_KERNEL)?;
        }
        Ok(Some(TvbGrowth {
            mappings,
            addresses,
            extension: backing,
        }))
    }
}

use super::object::Allocator;
use core::sync::atomic::{fence, AtomicU32, AtomicU64, Ordering};
use kernel::sync::Arc;

/// First-free metrics entries are shared by all parameter-buffer graphs.
/// An entry returns only when the graph and its firmware references retire.
pub(crate) struct MetricsIds {
    used: [AtomicU64; 2],
}
impl MetricsIds {
    pub(crate) fn new() -> Self {
        Self {
            used: [
                AtomicU64::new(0),
                AtomicU64::new(!((1u64 << (fw::METRICS_SLOTS - 64)) - 1)),
            ],
        }
    }
}

pub(crate) struct MetricsLease {
    _backing: Arc<KernelObject>,
    allocator: Arc<MetricsIds>,
    slot: usize,
    addresses: fw::MetricsAddresses,
}
impl MetricsLease {
    /// Clears only the selected retired metrics entry before reusing it.
    pub(crate) fn new(
        ids: &Arc<MetricsIds>,
        object: &Arc<KernelObject>,
        bases: fw::MetricsAddresses,
    ) -> Result<Self> {
        let mut selected = None;
        for (bank, used) in ids.used.iter().enumerate() {
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
                        selected = Some(bank * 64 + bit as usize);
                        break;
                    }
                    Err(current) => value = current,
                }
            }
            if selected.is_some() {
                break;
            }
        }
        let slot = selected.ok_or_else(|| super::capacity::refused(super::capacity::Site::MetricsIds))?;
        // Construct the lease before any fallible address/view validation so
        // every constructor failure returns the selected bit.
        let mut lease = Self {
            _backing: object.clone(),
            allocator: ids.clone(),
            slot,
            addresses: fw::MetricsAddresses::default(),
        };
        let offset = slot * fw::METRICS_STRIDE;
        lease.addresses = fw::MetricsAddresses {
            firmware: bases.firmware.checked_add(offset as u64).ok_or(EOVERFLOW)?,
            client: bases.client.checked_add(offset as u64).ok_or(EOVERFLOW)?,
        };
        let words = object.pointer(offset, fw::METRICS_STRIDE)?.cast::<u32>();
        object.word(offset)?;
        for index in 0..fw::SCENES {
            // SAFETY: The complete naturally aligned metrics entry is inside
            // the retained object; the exclusive lease excludes other owners.
            unsafe { words.add(index).write_volatile(0) };
        }
        Ok(lease)
    }
}
impl Drop for MetricsLease {
    fn drop(&mut self) {
        self.allocator.used[self.slot / 64]
            .fetch_and(!(1u64 << (self.slot % 64)), Ordering::Release);
    }
}

pub(crate) struct ManagerArgs {
    pub(crate) scratch: u64,
    pub(crate) discard: u64,
    pub(crate) buffer_id: u8,
    pub(crate) generation: u64,
    pub(crate) first_ordinal: u64,
}

/// A retained parameter-buffer graph and all of its sparse backing. Client
/// mappings live in the VM's owner cache; graph replacement never returns a
/// published backing early merely because an error fence was signalled.
pub(crate) struct Manager {
    _page_list: mmu::KernelMapping,
    graph: KernelObject,
    tvb: Tvb,
    metrics: MetricsLease,
    scenes: Scenes,
    page_list_client: u64,
    scratch: u64,
    buffer_id: u8,
    generation: u64,
    shrink: Shrink,
    /// A tiling pass initialized this buffer manager (25G83 InitBM published).
    initialized: bool,
}

/// Native tiler-heap shrink policy, matching the heap behaviour seen
/// under macOS. Once per
/// period the page high-water folds into an average,
/// `max((sample + 4 * average) / 5, sample)`. Each period whose average
/// stays below a quarter of the capacity earns one 512 KiB block of shrink,
/// and a heap idle for ten seconds returns to its minimum. Only an idle heap
/// is rebuilt here, by whole growth extensions.
///
/// Unlike macOS, the minimum keeps what the recent work needs: never below
/// the size at which the automatic growth check would grow the heap again
/// for the largest high-water sample of the last ten active periods. A
/// shrink then never makes the next pass at the recent load grow back.
const SHRINK_PERIOD_NS: u64 = 1_000_000_000;
const SHRINK_IDLE_NS: u64 = 10_000_000_000;
/// Blocks in one native 512 KiB parameter-buffer block.
const SHRINK_STEP_BLOCKS: usize = 0x8_0000 / cfg::TVB_BLOCK_SIZE;
/// Active periods in the recent-load window.
const SHRINK_WINDOW: usize = (SHRINK_IDLE_NS / SHRINK_PERIOD_NS) as usize;

#[derive(Default)]
struct Shrink {
    last_busy: u64,
    period_start: u64,
    /// High-water samples since the last period and the last growth check.
    period_max: u32,
    growth_max: u32,
    average: u32,
    credit: usize,
    /// Blocks the most recent pass layout required.
    required: usize,
    /// High-water samples of the last active periods (with a sample or a
    /// pass) and their largest value. Idle periods leave both unchanged.
    window: [u32; SHRINK_WINDOW],
    window_next: usize,
    busy_peak: u32,
}

pub(crate) struct Submitted<'a> {
    word: &'a AtomicU32,
    next: u32,
}
impl Submitted<'_> {
    pub(crate) fn commit(self) {
        self.word.store(self.next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
    }
}

impl Manager {
    pub(crate) fn new(
        alloc: &Allocator<'_>,
        vm: &mmu::Vm,
        tvb: Tvb,
        metrics: MetricsLease,
        args: ManagerArgs,
    ) -> Result<Self> {
        let mut graph = alloc.kernel(
            fw::SIZE,
            cfg::TVB_PAGE,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let source = fw::PAGE_LIST..fw::PAGE_LIST + fw::PAGE_LIST_BYTES;
        let page_list = compact_alias(
            &graph,
            vm,
            source,
            cfg::TVB_PAGE,
            mmu::PROT_GPU_FW_SHARED_RW,
            mmu::UAT_PGSZ,
        )?;
        let address = graph.gpu_va();
        graph.initialize::<fw::Graph>(0, |view| {
            view.init(&fw::GraphArgs {
                fw_va: address,
                page_list_client: page_list.iova(),
                metrics: metrics.addresses,
                blocks: tvb.addresses(),
                scratch: args.scratch,
                discard: args.discard,
                buffer_id: args.buffer_id.into(),
                generation: args.generation,
            })
        })?;
        fence(Ordering::SeqCst);
        let mut scenes = Scenes::new();
        scenes.commit(args.first_ordinal, fw::FIRST_SCENE)?;
        let page_list_client = page_list.iova();
        Ok(Self {
            _page_list: page_list,
            graph,
            tvb,
            metrics,
            scenes,
            page_list_client,
            scratch: args.scratch,
            buffer_id: args.buffer_id,
            generation: args.generation,
            shrink: Shrink::default(),
            initialized: false,
        })
    }

    /// Whether a published tiling pass initialized this manager (25G83 InitBM).
    pub(crate) fn initialized(&self) -> bool {
        self.initialized
    }

    pub(crate) fn mark_initialized(&mut self) {
        self.initialized = true;
    }
    pub(crate) fn buffer_id(&self) -> u8 {
        self.buffer_id
    }
    pub(crate) fn blocks(&self) -> usize {
        self.tvb.blocks()
    }
    pub(crate) fn state_va(&self) -> u64 {
        self.graph.gpu_va()
    }
    pub(crate) fn page_list_va(&self) -> u64 {
        self.page_list_client
    }
    pub(crate) fn trailer_va(&self) -> u64 {
        self.graph.gpu_va() + fw::SCENE_TABLE as u64 + (fw::SCENES * size_of::<fw::Scene>()) as u64
    }
    pub(crate) fn scene_va(&self, index: usize) -> Result<u64> {
        if index >= fw::SCENES {
            return Err(EINVAL);
        }
        Ok(self.graph.gpu_va() + fw::SCENE_TABLE as u64 + (index * size_of::<fw::Scene>()) as u64)
    }
    pub(crate) fn scene_registers(&self, index: usize) -> Result<(u64, u64)> {
        fw::scene_registers(index, self.scratch, self.metrics.addresses.client)
    }
    pub(crate) fn prepare_scene(&mut self, ordinal: u64) -> Result<usize> {
        let scene = self.scenes.select(ordinal)?;
        let offset = fw::SCENE_TABLE + scene * size_of::<fw::Scene>();
        let used = self
            .graph
            .word(offset + core::mem::offset_of!(fw::Scene, used))?;
        let generation = self
            .graph
            .pointer(offset + core::mem::offset_of!(fw::Scene, generation), 8)?;
        let id = self
            .graph
            .word(core::mem::offset_of!(fw::State, buffer_id))?;
        used.store(0, Ordering::Relaxed);
        // SAFETY: The complete packed field was checked before any write.
        // This scene is exclusively retained or still unclaimed.
        unsafe { generation.cast::<u64>().write_unaligned(self.generation) };
        id.store(u32::from(self.buffer_id), Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(scene)
    }
    pub(crate) fn claim_scene(&mut self, ordinal: u64, scene: usize) -> Result {
        self.scenes.commit(ordinal, scene)
    }
    pub(crate) fn retire_scene(&mut self, ordinal: u64) -> Result {
        self.scenes.release(ordinal)
    }
    pub(crate) fn prepare_submission(&mut self) -> Result<Submitted<'_>> {
        let word = self.graph.word(fw::COUNTER)?;
        Ok(Submitted {
            word,
            next: word.load(Ordering::Relaxed).wrapping_add(1),
        })
    }
    /// Reads the firmware page high-water and requests its reset. Both the
    /// growth check and the shrink period see every sample.
    fn sample_max_pages(&mut self) -> Result<u32> {
        let max_pages = self
            .graph
            .word(fw::CONTROL + core::mem::offset_of!(fw::Control, max_pages))?;
        let reset = self
            .graph
            .word(fw::CONTROL + core::mem::offset_of!(fw::Control, reset))?;
        fence(Ordering::Acquire);
        let pages = max_pages.load(Ordering::Relaxed);
        fence(Ordering::Release);
        reset.store(1, Ordering::Relaxed);
        self.shrink.period_max = self.shrink.period_max.max(pages);
        self.shrink.growth_max = self.shrink.growth_max.max(pages);
        Ok(pages)
    }
    pub(crate) fn automatic_target(&mut self) -> Result<Option<usize>> {
        self.sample_max_pages()?;
        let pages = core::mem::take(&mut self.shrink.growth_max);
        Ok(automatic_growth_target(pages as usize, self.blocks()))
    }
    /// Records the blocks a pass layout requires, the floor of any shrink.
    /// A pass is being prepared at `now`; the heap counts as busy.
    pub(crate) fn note_required(&mut self, blocks: usize, now: u64) {
        self.shrink.required = blocks;
        self.shrink.last_busy = now;
    }
    /// The smallest heap a shrink may leave: the primary backing, the blocks
    /// of the last pass layout, and the size below which the automatic growth
    /// check would grow again for the recent peak.
    fn shrink_floor(&self) -> usize {
        let recent = (self.shrink.busy_peak as usize)
            .saturating_mul(2)
            .div_ceil(cfg::TVB_PAGES_PER_BLOCK);
        self.tvb.primary_blocks().max(self.shrink.required).max(recent)
    }
    /// Whether the heap holds growth beyond its primary backing.
    pub(crate) fn grown(&self) -> bool {
        !self.tvb.extensions.is_empty()
    }
    /// Whether the heap holds growth that a shrink could give back.
    pub(crate) fn shrinkable(&self) -> bool {
        self.tvb.extensions.last().is_some_and(|extension| {
            self.blocks() - Tvb::extension_blocks(extension) >= self.shrink_floor()
        })
    }
    /// Advances the shrink policy at `now` and returns the block count an idle
    /// heap should shrink to. `busy` is true while the pair has work in flight
    /// or is borrowed; a busy heap only updates its average and recent peak.
    /// Call it for every grown heap, so the recent peak can also fall.
    pub(crate) fn shrink_target(&mut self, now: u64, busy: bool) -> Result<Option<usize>> {
        if busy || self.shrink.last_busy == 0 {
            self.shrink.last_busy = now;
        }
        if self.shrink.period_start == 0 {
            self.shrink.period_start = now;
        }
        if now.saturating_sub(self.shrink.period_start) >= SHRINK_PERIOD_NS {
            self.sample_max_pages()?;
            let sample = core::mem::take(&mut self.shrink.period_max);
            let average = (u64::from(sample) + 4 * u64::from(self.shrink.average)) / 5;
            self.shrink.average = (average as u32).max(sample);
            if sample != 0 || self.shrink.last_busy >= self.shrink.period_start {
                let next = self.shrink.window_next;
                self.shrink.window[next] = sample;
                self.shrink.window_next = (next + 1) % SHRINK_WINDOW;
                self.shrink.busy_peak = self.shrink.window.iter().copied().max().unwrap_or(0);
            }
            self.shrink.period_start = now;
            let capacity = self.blocks() * cfg::TVB_PAGES_PER_BLOCK;
            if (self.shrink.average as usize).saturating_mul(4) < capacity {
                self.shrink.credit = self.shrink.credit.saturating_add(SHRINK_STEP_BLOCKS);
            } else {
                self.shrink.credit = 0;
            }
        }
        if !self.shrinkable() {
            self.shrink.credit = 0;
            return Ok(None);
        }
        if busy {
            return Ok(None);
        }
        if now.saturating_sub(self.shrink.last_busy) >= SHRINK_IDLE_NS {
            return Ok(Some(self.shrink_floor()));
        }
        let last = self.tvb.extensions.last().map_or(0, Tvb::extension_blocks);
        Ok((last != 0 && self.shrink.credit >= last).then(|| self.blocks() - last))
    }
    fn cursors(&self) -> Result<Cursors> {
        Ok(Cursors {
            total: self.graph.word(fw::CONTROL)?.load(Ordering::Relaxed),
            committed: self.graph.word(fw::CONTROL + 4)?.load(Ordering::Relaxed),
            read: self.graph.word(fw::CONTROL + 8)?.load(Ordering::Relaxed),
        })
    }
}

/// All allocations and inventory words prepared without publishing shared state.
/// The pair remains exclusively borrowed until commit or discard.
pub(crate) struct PreparedGrowth {
    owner: u64,
    previous: usize,
    target: usize,
    before: Cursors,
    plan: Growth,
    growth: TvbGrowth,
    added: KVec<(usize, u32)>,
    rebuilt: KVec<u32>,
    pages: u32,
    descriptor: Option<[u32; 4]>,
}
impl Manager {
    /// Validate every retained sparse block and its reserved trailing guard in
    /// one VM snapshot before constructing more aliases.
    pub(crate) fn validate_vm(&self, vm: &mmu::Vm) -> Result {
        if self.blocks() < cfg::TVB_INITIAL_PAGES.len()
            || self.blocks() > cfg::TVB_MAX_BLOCKS
            || !vm.covers_sparse_blocks_with_guards(
                self.tvb.addresses(),
                cfg::TVB_BLOCK_SIZE as u64,
                cfg::TVB_BLOCK_STRIDE,
            )
        {
            return Err(EFAULT);
        }
        Ok(())
    }

    /// Allocate growth and precompute its words while the pair is exclusively
    /// borrowed. A descriptor snapshot requests idle-host growth; None leaves
    /// the firmware-owned page list and PB state untouched.
    pub(crate) fn prepare_growth(
        &mut self,
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        target: usize,
        descriptor: Option<[u32; 4]>,
    ) -> Result<Option<PreparedGrowth>> {
        let previous = self.blocks();
        if target <= previous {
            return Ok(None);
        }
        self.validate_vm(vm)?;
        let Some(growth) = self.tvb.prepare_growth(dev, vm, target)? else {
            return Ok(None);
        };
        let before = self.cursors()?;
        let plan = Growth::new(before, target - previous)?;
        let firmware_request = descriptor.is_none();
        let pages = target
            .checked_mul(cfg::TVB_PAGES_PER_BLOCK)
            .ok_or(EOVERFLOW)? as u32;
        let mut added = KVec::with_capacity(growth.addresses.len(), GFP_KERNEL)?;
        for (index, &address) in growth.addresses.iter().enumerate() {
            let first = fw::compact_page(address)?;
            first
                .checked_add(cfg::TVB_PAGES_PER_BLOCK as u32 - 1)
                .ok_or(EOVERFLOW)?;
            added.push((plan.added_slot(index)?, first), GFP_KERNEL)?;
        }
        let rebuilt_count = if firmware_request {
            0
        } else {
            plan.available() * cfg::TVB_PAGES_PER_BLOCK
        };
        let mut rebuilt = KVec::with_capacity(rebuilt_count, GFP_KERNEL)?;
        if !firmware_request {
            if self.cursors()? != before {
                return Err(EAGAIN);
            }
            for ordinal in 0..plan.available() {
                let first = if ordinal < plan.retained() {
                    let slot = plan.available_slot(ordinal)?;
                    self.graph
                        .word(fw::BLOCK_TABLE + slot * size_of::<fw::Block>())?
                        .load(Ordering::Relaxed)
                } else {
                    added[ordinal - plan.retained()].1
                };
                first
                    .checked_add(cfg::TVB_PAGES_PER_BLOCK as u32 - 1)
                    .ok_or(EOVERFLOW)?;
                for page in 0..cfg::TVB_PAGES_PER_BLOCK {
                    rebuilt.push(first + page as u32, GFP_KERNEL)?;
                }
            }
        }
        let descriptor =
            descriptor.map(|old| fw::reload_descriptor(old, self.page_list_client, pages));
        Ok(Some(PreparedGrowth {
            owner: self.graph.gpu_va(),
            previous,
            target,
            before,
            plan,
            growth,
            added,
            rebuilt,
            pages,
            descriptor,
        }))
    }

    /// The caller holds the device mutex after revalidating its preparation
    /// epoch. The final alias installation and all firmware writes occur under
    /// that same exclusion. The returned PB row is committed there as well.
    pub(crate) fn commit_growth(
        &mut self,
        prepared: &mut Option<PreparedGrowth>,
        install: impl FnOnce(usize, usize, &mut KVec<mmu::KernelMapping>) -> Result,
    ) -> Result<Option<[u32; 4]>> {
        let candidate = prepared.as_mut().ok_or(EINVAL)?;
        if self.graph.gpu_va() != candidate.owner || self.blocks() != candidate.previous {
            return Err(EBUSY);
        }
        let firmware_request = candidate.descriptor.is_none();
        let base = self.graph.pointer(0, fw::SIZE)?.cast::<u32>();
        // Bounds, actual alignment and every reserved host entry are checked
        // before transferring aliases or consuming the caller-owned plan.
        self.graph.word(0)?;
        if self.tvb.extensions.len() == self.tvb.extensions.capacity()
            || candidate.growth.addresses.len()
                > self.tvb.addresses.capacity() - self.tvb.addresses.len()
        {
            return Err(EIO);
        }
        if !candidate.before.permits(self.cursors()?, firmware_request) {
            return Err(EAGAIN);
        }
        // The callback retains its input on error. The caller can therefore
        // drop every rejected mapping/backing after releasing the device lock.
        install(
            candidate.previous,
            candidate.target,
            &mut candidate.growth.mappings,
        )?;
        if !candidate.growth.mappings.is_empty() {
            return Err(EIO);
        }
        let PreparedGrowth {
            before,
            plan,
            growth,
            added,
            rebuilt,
            pages,
            descriptor,
            ..
        } = prepared.take().ok_or(EIO)?;
        let TvbGrowth {
            addresses,
            extension,
            ..
        } = growth;

        // From here all vector capacity, indices, addresses and CPU views have
        // been checked. Retain backing before publishing its block entries.
        for address in addresses.into_iter() {
            append_reserved(&mut self.tvb.addresses, address);
        }
        let write = |offset: usize, value: u32| {
            // SAFETY: The full graph and base alignment were checked above.
            // Every offset is an aligned field or a checked array index of
            // Graph. No mutable reference aliases firmware-owned fields.
            unsafe { AtomicU32::from_ptr(base.add(offset / 4)).store(value, Ordering::Relaxed) };
        };
        for (slot, first) in added.into_iter() {
            let at = fw::BLOCK_TABLE + slot * size_of::<fw::Block>();
            write(at, first);
            write(at + 4, 0);
        }
        if descriptor.is_some() {
            for (entry, value) in rebuilt.into_iter().enumerate() {
                write(fw::PAGE_LIST + entry * 4, value);
            }
            write(core::mem::offset_of!(fw::State, pages), pages);
            write(core::mem::offset_of!(fw::State, producer), plan.target());
            write(core::mem::offset_of!(fw::State, consumer), before.read);
            write(core::mem::offset_of!(fw::State, last_page), pages - 1);
        }
        append_reserved(&mut self.tvb.extensions, extension);
        write(fw::CONTROL, plan.target());
        fence(Ordering::SeqCst);
        write(fw::CONTROL + 4, plan.target());
        fence(Ordering::SeqCst);
        Ok(descriptor)
    }
}

impl Manager {
    /// Rebuilds an idle heap with whole trailing growth extensions removed,
    /// keeping at least `target` blocks. The kept blocks are laid out as
    /// `Graph::init` lays out a fresh heap, starting at the firmware read
    /// cursor. This runs only while every block is back in the inventory, as
    /// an idle-host growth requires the complete snapshot unchanged.
    ///
    /// `truncate(blocks, kept, out)` moves the removed blocks' client aliases
    /// out of the VM cache. Returns the reloaded PB row and the released
    /// aliases and backing, which the caller frees after the device mutex.
    /// None leaves the heap, the firmware records and the VM unchanged.
    pub(crate) fn shrink(
        &mut self,
        target: usize,
        descriptor: [u32; 4],
        truncate: impl FnOnce(usize, usize, &mut KVec<mmu::KernelMapping>) -> Result,
    ) -> Result<Option<([u32; 4], TvbRelease)>> {
        let blocks = self.blocks();
        let mut kept = blocks;
        let mut dropped = 0;
        for extension in self.tvb.extensions.iter().rev() {
            let size = Tvb::extension_blocks(extension);
            if kept - size < target.max(self.tvb.primary_blocks()) {
                break;
            }
            kept -= size;
            dropped += 1;
        }
        if dropped == 0 {
            return Ok(None);
        }
        let before = self.cursors()?;
        let retained = (before.total as usize + cfg::TVB_BLOCK_SLOTS - before.read as usize)
            % cfg::TVB_BLOCK_SLOTS;
        if !before.valid() || before.total != before.committed || retained != blocks {
            return Ok(None);
        }
        // Allocation failure only postpones the shrink to a later pass.
        let (Ok(mappings), Ok(extensions)) = (
            KVec::with_capacity(blocks - kept, GFP_NOWAIT),
            KVec::with_capacity(dropped, GFP_NOWAIT),
        ) else {
            return Ok(None);
        };
        let mut release = TvbRelease {
            _mappings: mappings,
            _extensions: extensions,
        };
        for &address in &self.tvb.addresses[..kept] {
            fw::compact_page(address)?
                .checked_add(cfg::TVB_PAGES_PER_BLOCK as u32 - 1)
                .ok_or(EOVERFLOW)?;
        }
        let base = self.graph.pointer(0, fw::SIZE)?.cast::<u32>();
        self.graph.word(0)?;
        // The callback moves aliases only after checking the cached owner,
        // its block count and the spare capacity.
        truncate(blocks, kept, &mut release._mappings)?;

        // From here every index, address and CPU view has been checked.
        let write = |offset: usize, value: u32| {
            // SAFETY: The full graph and base alignment were checked above.
            // Every offset is an aligned field or a checked array index of
            // Graph. No mutable reference aliases firmware-owned fields.
            unsafe { AtomicU32::from_ptr(base.add(offset / 4)).store(value, Ordering::Relaxed) };
        };
        for (index, &address) in self.tvb.addresses[..kept].iter().enumerate() {
            // The same pure conversion succeeded for every kept block above.
            let Ok(first) = fw::compact_page(address) else {
                continue;
            };
            let slot = (before.read as usize + index) % cfg::TVB_BLOCK_SLOTS;
            let at = fw::BLOCK_TABLE + slot * size_of::<fw::Block>();
            write(at, first);
            write(at + 4, 0);
            for page in 0..cfg::TVB_PAGES_PER_BLOCK {
                let entry = index * cfg::TVB_PAGES_PER_BLOCK + page;
                write(fw::PAGE_LIST + entry * 4, first + page as u32);
            }
        }
        let total = ((before.read as usize + kept) % cfg::TVB_BLOCK_SLOTS) as u32;
        let pages = (kept * cfg::TVB_PAGES_PER_BLOCK) as u32;
        write(core::mem::offset_of!(fw::State, pages), pages);
        write(core::mem::offset_of!(fw::State, producer), total);
        write(core::mem::offset_of!(fw::State, consumer), before.read);
        write(core::mem::offset_of!(fw::State, last_page), pages - 1);
        write(fw::CONTROL, total);
        fence(Ordering::SeqCst);
        write(fw::CONTROL + 4, total);
        fence(Ordering::SeqCst);

        self.tvb.addresses.truncate(kept);
        for _ in 0..dropped {
            if let Some(extension) = self.tvb.extensions.pop() {
                append_reserved(&mut release._extensions, extension);
            }
        }
        self.shrink.credit = self.shrink.credit.saturating_sub(blocks - kept);
        Ok(Some((
            fw::reload_descriptor(descriptor, self.page_list_client, pages),
            release,
        )))
    }
}

/// Caller reserves all needed entries before any firmware-visible mutation.
fn append_reserved<T>(values: &mut KVec<T>, value: T) {
    let length = values.len();
    values.spare_capacity_mut()[0].write(value);
    // SAFETY: All callers preflighted this spare entry and initialized it once.
    unsafe { values.set_len(length + 1) };
}

impl BufferIds {
    pub(crate) fn write_capacity(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "parameter_buffers used={} total={} owned={} referenced={}",
                 self.entries.iter().filter(|entry| !entry.free()).count(), BUFFER_IDS,
                 self.entries.iter().filter(|entry| entry.owner).count(),
                 self.entries.iter().filter(|entry| entry.references != 0).count())
    }
}
impl MetricsIds {
    pub(crate) fn used(&self) -> u32 {
        self.used.iter().map(|bits| bits.load(Ordering::Acquire).count_ones()).sum::<u32>()
            - (128 - fw::METRICS_SLOTS as u32)
    }
}

#[kunit_tests(asahi_g17_buffer_capacity)]
mod capacity_tests {
    use super::*;

    #[test]
    fn failed_build_and_fault_retirement_return_parameter_buffers() {
        let mut ids = KBox::init(BufferIds::new(), GFP_KERNEL).unwrap();
        for _ in 0..8 {
            let mut clients = KVec::with_capacity(BUFFER_IDS, GFP_KERNEL).unwrap();
            for _ in 0..BUFFER_IDS {
                clients.push(ids.allocate_render().unwrap().0, GFP_KERNEL).unwrap();
            }
            assert_eq!(ids.allocate_render(), Err(ENOSPC));
            for (index, id) in clients.into_iter().enumerate() {
                if index % 2 == 0 {
                    // Failed constructor never installed the graph owner.
                    ids.release(id).unwrap();
                } else {
                    ids.reserve_owner(id).unwrap();
                    ids.release(id).unwrap();
                    ids.acquire_owner(id).unwrap();
                    // Close cannot return a still-owned failed command.
                    ids.unreserve_owner(id).unwrap();
                    assert!(!ids.entries[id as usize].free());
                    ids.release(id).unwrap();
                }
            }
            assert!(ids.entries.iter().all(|entry| entry.free()));
            assert_eq!(ids.returned_count, BUFFER_IDS);
        }
    }
}
