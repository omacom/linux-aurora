// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Allocates, initializes and retains the graph shared by the two firmware instances.
//!
//! The owner is installed in the firmware session before construction starts. Even if an
//! allocation fails, the session stops both coprocessors before releasing any published memory.

use super::{
    fw::initdata as abi,
    object::{CpuMap, KernelObject, Placement},
    Config, Role,
};
use crate::{driver::AsahiDrmDevice, hw::t8140, mmu};
use core::ops::Range;
use core::sync::atomic::{fence, AtomicU32, Ordering};
use kernel::{c_str, prelude::*};
use t8140::{kernel_window as high, low};

mod v25g83;

const EVENT_RING_RECHECK: u32 = 256;

const INDEX_SIZE: usize = 0x10000;
const CONTEXT_SIZE: usize = 0x20000;
const REGION_A_SIZE: usize = core::mem::size_of::<abi::RegionA>();
const REGION_C_SIZE: usize = 0x1000;

const COMPUTE_BLOCKS_MAPPED: usize = 22;
const COMPUTE_BLOCKS_POPULATED: usize = 21;
const COMPUTE_CONTROL_BLOCKS: u32 = 25;
const COMPUTE_CONTROL_CURSOR: u32 = 0x18;
const COMPUTE_RUN_ADDRESS_MASK: u64 = 0x0000_ffff_ffff_f000;

/// A borrowed completion slot. Read the identity before requesting its timestamps;
/// unrelated QIDs need only the control word. Each access is an aligned live load.
pub(crate) struct ComputeCompletion<'a> {
    object: &'a KernelObject,
    offset: usize,
}

/// Records in the compute completion ring that the host scans.
pub(super) const COMPUTE_COMPLETION_SLOTS: usize = {
    let slots = abi::COMPLETION_RING_SIZE / 0x40;
    if slots < 0x800 { slots } else { 0x800 }
};

/// Timestamp bits of a completion record's start and end words.
const COMPLETION_TIME_MASK: u64 = 0x003f_ffff_ffff_ffff;

/// What one completion-ring slot gave a scan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CompletionFold {
    /// No new observation of a pending request; the scan continues.
    Passed,
    /// Every request now has an observation.
    Done,
    /// A pending request's record whose payload the firmware is still writing.
    Incomplete,
}

impl ComputeCompletion<'_> {
    pub(crate) fn read(&self, word: usize) -> Result<u64> {
        if word >= 8 {
            return Err(EINVAL);
        }
        // The scanner bounds offset to a 0x40-byte slot in the completion ring.
        Ok(self
            .object
            .dword(self.offset + word * 8)?
            .load(Ordering::Relaxed))
    }
}

/// Unpublished device-wide compute backing, prepared without the device mutex. The cache
/// identity belongs to this allocation, not to any one physical queue or client VM.
pub(super) struct ComputeShared {
    id: u64,
    pages: KernelObject,
    runs: KernelObject,
    blocks: KVec<KernelObject>,
}

impl ComputeShared {
    pub(super) fn new(alloc: &super::object::Allocator<'_>) -> Result<Self> {
        let pages =
            KernelObject::backing(alloc.dev, low::RENDER_PAGE_LIST_SIZE, CpuMap::WriteCombined)?;
        let runs =
            KernelObject::backing(alloc.dev, low::RENDER_RUN_LIST_SIZE, CpuMap::WriteCombined)?;
        let mut blocks = KVec::with_capacity(COMPUTE_BLOCKS_MAPPED, GFP_KERNEL)?;
        for _ in 0..COMPUTE_BLOCKS_MAPPED {
            blocks.push(
                KernelObject::backing(alloc.dev, abi::FREE_LIST_BLOCK_SIZE, CpuMap::WriteCombined)?,
                GFP_KERNEL,
            )?;
        }
        let id = u64::from(super::ids::OBJECTS.object()?);
        Ok(Self {
            id,
            pages,
            runs,
            blocks,
        })
    }
}

/// A device-global scheduler, retained independently of whichever physical queue first
/// needed it. Its generation reservation precedes later graph allocation failures.
pub(super) struct ComputeScheduler {
    object: KernelObject,
}

impl ComputeScheduler {
    pub(super) fn new(alloc: &super::object::Allocator<'_>) -> Result<Self> {
        let mut object = alloc.kernel(
            mmu::UAT_PGSZ,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        object.initialize::<super::fw::context::Scheduler>(0, |state| {
            state.init();
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        // Scheduler creation reserves a generation even when only its address is used by work.
        super::ids::OBJECTS.object()?;
        Ok(Self { object })
    }
}

/// Checked views used only within one publication call. Every fallible operation completes
/// before either member of a paired render publication is copied into shared memory.
struct WorkPublication<'a> {
    record: &'a super::fw::channels::WorkSlot,
    destination: *mut u8,
    producer: &'a AtomicU32,
    next: u32,
}

impl WorkPublication<'_> {
    fn copy_record(&self) {
        // SAFETY: work_publication checked the complete destination range. The record is a
        // fully initialized padding-free ABI value, and the device mutex owns publication.
        unsafe {
            core::ptr::copy_nonoverlapping(
                core::ptr::from_ref(self.record).cast::<u8>(),
                self.destination,
                core::mem::size_of_val(self.record),
            );
        }
    }
}

/// Session-long storage and mappings. Aliases are released before their backing objects.
pub(super) struct InitData {
    // Live publication uses interior writes under the device mutex. This owner is Send but not
    // Sync, so safe host code cannot issue those writes concurrently through shared references.
    _single_publisher: core::marker::PhantomData<core::cell::Cell<()>>,
    aliases: KVec<mmu::KernelMapping>,
    objects: KVec<KernelObject>,
    roots: usize,
    cluster: usize,
    bundle: usize,
    region_c: usize,
    qos: usize,
    pb: usize,
    page_pool: usize,
    compute_global: usize,
    boot_control: usize,
    boot_state: usize,
    metrics_fw_va: u64,
    /// Client VA of the parameter-buffer metrics object (`Config::pm_metrics`).
    pm_metrics_va: u64,
    /// `Config::recovery_slot_keys`.
    recovery_slot_keys: Option<u64>,
    /// Primary status A and status B in the cluster.
    status_a: usize,
    status_b: usize,
    /// Separately allocated views; `None` when they are packed into the bundle.
    views: Option<[usize; 5]>,
    compute_shared: Option<ComputeShared>,
    compute_scheduler: Option<ComputeScheduler>,
    compute_active: core::cell::Cell<bool>,
    event_ring_checked: [core::cell::Cell<(u64, u64, u32)>; 2],
    completion_rings: [usize; 2],
    stamps: [usize; 2],
    render_pages: usize,
    render_runs: usize,
    render_blocks: core::ops::Range<usize>,
}

/// Placement inputs shared by the build steps.
struct BuildEnv<'a> {
    dev: &'a AsahiDrmDevice,
    vm: &'a mmu::Vm,
    lower: &'a mmu::Vm,
    render: &'a mmu::Vm,
    base: u64,
    dynamic: Range<u64>,
    /// GPU clusters, as the GPU reports them.
    clusters: u32,
}

impl BuildEnv<'_> {
    /// An object at a fixed offset of the kernel window.
    fn fixed(&self, offset: u64, size: usize, prot: mmu::Prot, cpu: CpuMap) -> Result<KernelObject> {
        KernelObject::new(
            self.dev,
            self.vm,
            Placement::At(self.base + offset),
            size,
            prot,
            cpu,
        )
    }

    /// An object placed in the dynamic part of the kernel window.
    fn dynamic(&self, size: usize, align: u64, prot: mmu::Prot, cpu: CpuMap) -> Result<KernelObject> {
        KernelObject::new(
            self.dev,
            self.vm,
            Placement::In(self.dynamic.clone(), align),
            size,
            prot,
            cpu,
        )
    }
}

/// Object indices the firmware-object initialization needs from the table step.
struct DescriptorTables {
    pb: usize,
    page_pool: usize,
    fwctl: usize,
}

/// Object indices and ring addresses the firmware-object initialization needs from the bundle step.
struct BundleObjects {
    zero: usize,
    completion_rings: [abi::CompletionRing; 2],
    region_a: usize,
    qos: usize,
}

impl InitData {
    /// Empty construction state, retained even if graph construction fails.
    pub(super) fn new() -> Self {
        Self {
            _single_publisher: core::marker::PhantomData,
            aliases: KVec::new(),
            objects: KVec::new(),
            roots: 0,
            cluster: 0,
            bundle: 0,
            region_c: 0,
            qos: 0,
            pb: 0,
            page_pool: 0,
            compute_global: 0,
            boot_control: 0,
            boot_state: 0,
            metrics_fw_va: 0,
            pm_metrics_va: 0,
            recovery_slot_keys: None,
            status_a: abi::private::PRIMARY_STATUS_A,
            status_b: abi::private::STATUS_B,
            views: None,
            compute_shared: None,
            compute_scheduler: None,
            compute_active: core::cell::Cell::new(false),
            event_ring_checked: [const { core::cell::Cell::new((0, 0, 0)) }; 2],
            completion_rings: [0; 2],
            stamps: [0; 2],
            render_pages: 0,
            render_runs: 0,
            render_blocks: 0..0,
        }
    }

    pub(super) fn pm_metrics_va(&self) -> u64 {
        self.pm_metrics_va
    }

    pub(super) fn metrics_fw_va(&self) -> Result<u64> {
        if self.metrics_fw_va == 0 {
            return Err(EIO);
        }
        Ok(self.metrics_fw_va)
    }

    pub(super) fn pb_descriptor_table(&self) -> Result<&KernelObject> {
        self.object(self.pb)
    }

    pub(super) fn page_pool_descriptor_table(&self) -> Result<&KernelObject> {
        self.object(self.page_pool)
    }

    pub(super) fn stamp_addresses(&self, qid: u8) -> Result<[u64; 2]> {
        if qid > super::fw::kick::QID_MAX {
            return Err(EINVAL);
        }
        let offset = u64::from(qid) * core::mem::size_of::<u32>() as u64;
        Ok([
            self.object(self.stamps[0])?.gpu_va() + offset,
            self.object(self.stamps[1])?.gpu_va() + offset,
        ])
    }

    /// The caller reads both engine lanes before its acquire fence. Page A is the completed
    /// GPU-maintenance witness; page B stays under firmware ownership.
    pub(super) fn stamp(&self, qid: u8) -> Result<&AtomicU32> {
        if qid > super::fw::kick::QID_MAX {
            return Err(EINVAL);
        }
        self.object(self.stamps[0])?
            .word(usize::from(qid) * core::mem::size_of::<u32>())
    }

    /// Initialize a fresh physical compute QID's row before exposing its queue. An installed
    /// or rebound queue keeps the row's firmware-owned counters and must never call this again.
    /// Called for a fresh row or after its free-list release was consumed.
    pub(super) fn initialize_compute_pool_blocks(&self, slot: u16, page_list_va: u64, blocks: usize) -> Result {
        if slot == 0 || slot > u16::from(super::fw::kick::QID_MAX) + 1 {
            return Err(EINVAL);
        }
        let entry = abi::PagePoolDescriptor::compute_blocks(page_list_va, blocks);
        let size = core::mem::size_of::<abi::PagePoolDescriptor>();
        let offset = usize::from(slot) * size;
        let object = self.object(self.page_pool)?;
        let pointer = object.pointer(offset, size)?;
        let page_list = object.dword(offset)?;
        let counters =
            object.dword(offset + core::mem::offset_of!(abi::PagePoolDescriptor, counters))?;
        let count =
            object.dword(offset + core::mem::offset_of!(abi::PagePoolDescriptor, page_count))?;
        // SAFETY: the whole fresh row was bounds checked, this is a fresh QID or its release
        // was consumed, and the device mutex excludes another host publisher. No other row is touched.
        unsafe {
            pointer.write_bytes(0, size);
        }
        page_list.store(entry.page_list, Ordering::Relaxed);
        counters.store(entry.counters, Ordering::Relaxed);
        count.store(entry.page_count, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(())
    }

    pub(super) fn qos(&self) -> Result<&KernelObject> {
        self.object(self.qos)
    }

    pub(super) fn has_compute_scheduler(&self) -> bool {
        self.compute_scheduler.is_some()
    }

    /// Only the reserved singleton builder may install a scheduler. Retain it before building
    /// the first descriptor graph so later construction failure does not allocate it again.
    pub(super) fn install_compute_scheduler(&mut self, scheduler: ComputeScheduler) -> Result {
        if self.compute_scheduler.is_some() {
            return Err(EBUSY);
        }
        self.compute_scheduler = Some(scheduler);
        Ok(())
    }

    pub(super) fn has_compute_shared(&self) -> bool {
        self.compute_shared.is_some()
    }

    /// Install a prepared aperture once while holding the device mutex. A competing preparer
    /// may have installed its allocation first; the redundant unpublished backing then drops.
    pub(super) fn install_compute_shared(&mut self, mut shared: ComputeShared) -> Result {
        if self.compute_shared.is_some() {
            return Ok(());
        }
        let state_va = self.object(self.boot_state)?.gpu_va();
        let mut control = abi::FreeListControl::new(&abi::FreeListArgs {
            buffer_slot: u32::MAX,
            class: abi::FreeListClass::Two,
            page_list_va: low::COMPUTE_POOL_PAGE_LIST,
            run_list_va: low::COMPUTE_POOL_RUN_LIST,
            blocks: COMPUTE_CONTROL_BLOCKS,
            state_va,
        });
        control.run_cursor = COMPUTE_CONTROL_CURSOR;
        self.object_mut(self.boot_control)?.bytes_mut().fill(0);
        self.object_mut(self.boot_control)?.write(0, control)?;
        self.object_mut(self.boot_state)?.bytes_mut().fill(0);
        shared
            .pages
            .initialize::<[u64; low::RENDER_PAGE_LIST_SIZE / 8]>(0, |pages| {
                abi::fill_page_list(pages, (0..COMPUTE_BLOCKS_POPULATED).map(low::compute_block))
            })?;
        shared
            .runs
            .initialize::<[u64; low::RENDER_RUN_LIST_SIZE / 8]>(0, |runs| {
                for (index, run) in runs.iter_mut().take(COMPUTE_BLOCKS_POPULATED).enumerate() {
                    *run = (low::compute_block(index) & COMPUTE_RUN_ADDRESS_MASK)
                        | (u64::from(abi::FREE_LIST_BLOCK_PAGES) << 52);
                }
                Ok(())
            })?;
        fence(Ordering::SeqCst);
        self.compute_shared = Some(shared);
        Ok(())
    }

    pub(super) fn compute_vm_cached(&self, vm: &mmu::Vm) -> bool {
        self.compute_shared
            .as_ref()
            .is_some_and(|shared| vm.compute_pool_mappings_match(shared.id))
    }

    pub(super) fn cache_compute_vm(&self, vm: &mmu::Vm) -> Result {
        let shared = self.compute_shared.as_ref().ok_or(EIO)?;
        if vm.compute_pool_mappings_match(shared.id) {
            return Ok(());
        }
        vm.clear_compute_pool_mappings();
        let mut mappings = KVec::with_capacity(26, GFP_KERNEL)?;
        mappings.push(
            shared
                .pages
                .map_alias(vm, low::COMPUTE_POOL_PAGE_LIST, mmu::PROT_GPU_SHARED_RW)?,
            GFP_KERNEL,
        )?;
        mappings.push(
            shared
                .runs
                .map_alias(vm, low::COMPUTE_POOL_RUN_LIST, mmu::PROT_GPU_SHARED_RW)?,
            GFP_KERNEL,
        )?;
        for (index, block) in shared.blocks.iter().take(COMPUTE_BLOCKS_MAPPED).enumerate() {
            mappings.push(
                block.map_alias(vm, low::compute_block(index), mmu::PROT_GPU_SHARED_RW)?,
                GFP_KERNEL,
            )?;
        }
        for slot in 0..2 {
            mappings.push(
                self.object(self.completion_rings[slot])?.map_alias(
                    vm,
                    low::COMPLETION_RINGS[slot],
                    mmu::PROT_GPU_SHARED_RW,
                )?,
                GFP_KERNEL,
            )?;
        }
        vm.install_compute_pool_mappings(shared.id, mappings)
    }

    /// The descriptor-table views remain immutable for the firmware session.
    pub(super) fn validate_descriptor_views(&self) -> Result {
        let values = [
            low::PB_DESCRIPTORS,
            self.object(self.pb)?.gpu_va(),
            low::PAGE_POOL_DESCRIPTORS,
            self.object(self.page_pool)?.gpu_va(),
        ];
        let base = abi::bundle::MAIN_CONFIG[Role::Primary as usize]
            + core::mem::offset_of!(abi::MainConfig, pb_descriptors_low_va);
        let bundle = self.object(self.bundle)?;
        let mut changed = false;
        for (index, value) in values.into_iter().enumerate() {
            changed |= bundle.dword(base + index * 8)?.load(Ordering::Relaxed) != value;
        }
        fence(Ordering::SeqCst);
        if changed {
            Err(EIO)
        } else {
            Ok(())
        }
    }

    pub(super) fn activate_compute(&self) -> Result {
        self.validate_descriptor_views()?;
        if self.compute_active.get() {
            return Ok(());
        }
        let scheduler = self.compute_scheduler.as_ref().ok_or(EIO)?.object.gpu_va();
        let object = self.object(self.compute_global)?;
        let state = object
            .pointer(0, core::mem::size_of::<abi::ComputeGlobalState>())?
            .cast::<abi::ComputeGlobalState>();
        // SAFETY: The complete typed view is in the retained page-aligned object. InitData is
        // not Sync and its caller holds the sole publication mutex. Each named field is
        // naturally aligned and written individually; no reference aliases firmware state.
        unsafe {
            core::ptr::addr_of_mut!((*state).unk_0).write_volatile(0xff);
            core::ptr::addr_of_mut!((*state).unk_8).write_volatile(0xff);
            core::ptr::addr_of_mut!((*state).unk_10).write_volatile(0x0f01);
            core::ptr::addr_of_mut!((*state).unk_18).write_volatile(0x0f01);
            core::ptr::addr_of_mut!((*state).unk_408).write_volatile(2);
            core::ptr::addr_of_mut!((*state).unk_40c).write_volatile(2);
            core::ptr::addr_of_mut!((*state).unk_604).write_volatile(4);
            core::ptr::addr_of_mut!((*state).scheduler_va).write_volatile(scheduler);
            core::ptr::addr_of_mut!((*state).unk_c04).write_volatile(0x1000);
            core::ptr::addr_of_mut!((*state).unk_e00).write_volatile(7);
            core::ptr::addr_of_mut!((*state).unk_e08).write_volatile(0x0000_0001_0e22_618a);
            core::ptr::addr_of_mut!((*state).unk_e20).write_volatile(4);
            core::ptr::addr_of_mut!((*state).unk_e28).write_volatile(4);
        }
        self.compute_active.set(true);
        Ok(())
    }

    /// The sole host publisher holds the device mutex through the following
    /// inner and outer publication. Firmware consumers can only free capacity;
    /// neither this check nor waiting for room reserves or limits other queues.
    /// `priority` selects the engine's ring of that priority class.
    pub(super) fn work_ready(
        &self,
        priority: u8,
        engine: super::fw::queue::DataMaster,
    ) -> Result<bool> {
        let [consumer, consumer_b, producer] = self.work_cursors(priority, engine)?;
        let next = (producer + 1) % super::fw::channels::WORK_RING_SLOTS;
        Ok(next != consumer && next != consumer_b)
    }

    fn work_publication<'a>(
        &'a self,
        priority: u8,
        record: &'a super::fw::channels::WorkSlot,
    ) -> Result<WorkPublication<'a>> {
        use super::fw::{channels, queue::DataMaster};
        let engine = match record.data_master {
            0 => DataMaster::Tiling,
            1 => DataMaster::Fragment,
            // Compute publications reserve their slots through the port.
            _ => return Err(EINVAL),
        };
        let channel = channels::work_ring_index(priority, engine).ok_or(EINVAL)?;
        let [consumer, consumer_b, producer] = self.work_cursors(priority, engine)?;
        let next = (producer + 1) % channels::WORK_RING_SLOTS;
        if next == consumer || next == consumer_b {
            return Err(EINVAL);
        }
        let destination = self.object(self.bundle)?.pointer(
            abi::work_ring_offset(channel) + producer as usize * core::mem::size_of_val(record),
            core::mem::size_of_val(record),
        )?;
        let producer = self.object(self.cluster)?.word(
            self.state_offset(Role::Primary)
                + abi::work_state_offset(channel)
                + abi::ChannelState::PRODUCER,
        )?;
        Ok(WorkPublication {
            record,
            destination,
            producer,
            next,
        })
    }

    /// Two firmware consumers and the host producer, sampled in that order.
    pub(super) fn work_cursors(
        &self,
        priority: u8,
        engine: super::fw::queue::DataMaster,
    ) -> Result<[u32; 3]> {
        use super::fw::channels;
        let channel = channels::work_ring_index(priority, engine).ok_or(EINVAL)?;
        let base = self.state_offset(Role::Primary) + abi::work_state_offset(channel);
        let state = self.object(self.cluster)?;
        let mut values = [0; 3];
        for (value, offset) in values.iter_mut().zip([
            abi::ChannelState::CONSUMER,
            abi::ChannelState::UNK_10,
            abi::ChannelState::PRODUCER,
        ]) {
            *value = state.word(base + offset)?.load(Ordering::Relaxed);
        }
        if values
            .iter()
            .any(|&value| value >= channels::WORK_RING_SLOTS)
        {
            return Err(EINVAL);
        }
        Ok(values)
    }

    /// Validate both destinations before exposing fragment, then tiling. The accounting token
    /// is committed after copying fragment; late tiling state is published before its slot.
    /// Deferred pairs retain both slot bodies without exposing either producer.
    pub(super) fn publish_work_pair(
        &self,
        priority: u8,
        fragment: &super::fw::channels::WorkSlot,
        tiling: &super::fw::channels::WorkSlot,
        deferred: bool,
        commit_accounting: impl FnOnce() -> Result,
        publish_late_tiling: impl FnOnce(),
    ) -> Result<[u32; 2]> {
        use super::fw::queue::DataMaster;
        if fragment.data_master != DataMaster::Fragment as u32
            || tiling.data_master != DataMaster::Tiling as u32
        {
            return Err(EINVAL);
        }
        let fragment = self.work_publication(priority, fragment)?;
        let tiling = self.work_publication(priority, tiling)?;
        fragment.copy_record();
        commit_accounting()?;
        fence(Ordering::SeqCst);
        if !deferred {
            fragment.producer.store(fragment.next, Ordering::Relaxed);
        }
        publish_late_tiling();
        tiling.copy_record();
        fence(Ordering::SeqCst);
        if !deferred {
            tiling.producer.store(tiling.next, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok([fragment.next, tiling.next])
    }

    fn retained_work_publication<'a>(
        &'a self,
        priority: u8,
        record: &'a super::fw::channels::WorkSlot,
        next: u32,
    ) -> Result<WorkPublication<'a>> {
        use super::fw::{channels, queue::DataMaster};
        let engine = match record.data_master {
            0 => DataMaster::Tiling,
            1 => DataMaster::Fragment,
            _ => return Err(EINVAL),
        };
        let channel = channels::work_ring_index(priority, engine).ok_or(EINVAL)?;
        let producer = self.object(self.cluster)?.word(
            self.state_offset(Role::Primary)
                + abi::work_state_offset(channel)
                + abi::ChannelState::PRODUCER,
        )?;
        let current = producer.load(Ordering::Relaxed);
        if current >= channels::WORK_RING_SLOTS || next != (current + 1) % channels::WORK_RING_SLOTS
        {
            return Err(EBUSY);
        }
        let destination = self.object(self.bundle)?.pointer(
            abi::work_ring_offset(channel) + current as usize * core::mem::size_of_val(record),
            core::mem::size_of_val(record),
        )?;
        Ok(WorkPublication {
            record,
            destination,
            producer,
            next,
        })
    }

    /// Release a retained pair after its control transaction. Revalidate the original producer
    /// witnesses before either copy; both bodies precede the fragment and tiling publications.
    pub(super) fn publish_retained_work_pair(
        &self,
        priority: u8,
        fragment: &super::fw::channels::WorkSlot,
        tiling: &super::fw::channels::WorkSlot,
        next: [u32; 2],
    ) -> Result {
        use super::fw::queue::DataMaster;
        if fragment.data_master != DataMaster::Fragment as u32
            || tiling.data_master != DataMaster::Tiling as u32
        {
            return Err(EINVAL);
        }
        let fragment = self.retained_work_publication(priority, fragment, next[0])?;
        let tiling = self.retained_work_publication(priority, tiling, next[1])?;
        fragment.copy_record();
        tiling.copy_record();
        fence(Ordering::SeqCst);
        fragment.producer.store(fragment.next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        tiling.producer.store(tiling.next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Raw views of the memory publishers share without the device mutex: every work ring,
    /// the completion stamps, the QoS table and the recovery state word. They stay valid for
    /// this session.
    pub(super) fn port_views(
        &self,
        mailbox: *const super::Mailbox,
    ) -> Result<super::port::Views> {
        use super::fw::{channels, queue::{DataMaster, Policy}};
        let ring_of = |priority: u8, engine: DataMaster| -> Result<super::port::RingView> {
            let channel = channels::work_ring_index(priority, engine).ok_or(EINVAL)?;
            let base = self.state_offset(Role::Primary) + abi::work_state_offset(channel);
            let state = self.object(self.cluster)?;
            let word = |offset: usize| -> Result<*const AtomicU32> {
                Ok(state.word(base + offset)?)
            };
            Ok(super::port::RingView {
                slots: self.object(self.bundle)?.pointer(
                    abi::work_ring_offset(channel),
                    channels::WORK_RING_SLOTS as usize
                        * core::mem::size_of::<channels::WorkSlot>(),
                )?,
                consumers: [
                    word(abi::ChannelState::CONSUMER)?,
                    word(abi::ChannelState::UNK_10)?,
                ],
                producer: word(abi::ChannelState::PRODUCER)?,
            })
        };
        let rings = |engine: DataMaster| -> Result<[super::port::RingView; Policy::PRIORITIES as usize]> {
            let mut views = [ring_of(0, engine)?; Policy::PRIORITIES as usize];
            for (priority, view) in views.iter_mut().enumerate().skip(1) {
                *view = ring_of(priority as u8, engine)?;
            }
            Ok(views)
        };
        Ok(super::port::Views {
            compute: rings(DataMaster::Compute)?,
            tiling: rings(DataMaster::Tiling)?,
            fragment: rings(DataMaster::Fragment)?,
            stamps: self.object(self.stamps[0])?.word(0)?,
            qos: self
                .qos()?
                .pointer(0, core::mem::size_of::<super::fw::qos::Qos>())?,
            recovery: self
                .object(self.cluster)?
                .word(abi::private::STATUS_B + abi::PrimaryStatusB::RECOVERY_STATE)?,
            mailbox,
        })
    }

    /// Snapshot the shared compute completion ring once per service transaction. The queue
    /// scanner latches exact identities; firmware retains sole ownership of these slots.
    /// Folds the compute completion records the firmware appended since `start`, in ring
    /// order. `seen` holds every slot's control word as of the previous pass over it: the
    /// first slot still holding it is where the firmware writes next, and the pass ends there
    /// instead of reading the rest of the ring. A pass also ends once `record` has found every
    /// request, or at a requested record whose payload is still being written, which the next
    /// pass reads again. Returns the slot the next pass starts at.
    pub(super) fn fold_compute_completions(
        &self,
        start: usize,
        seen: &mut [u64],
        mut record: impl FnMut(ComputeCompletion<'_>) -> Result<CompletionFold>,
    ) -> Result<usize> {
        let object = self.object(self.completion_rings[0])?;
        if seen.len() != COMPUTE_COMPLETION_SLOTS {
            return Err(EINVAL);
        }
        fence(Ordering::Acquire);
        let completion = |slot: usize| ComputeCompletion {
            object,
            offset: slot * 0x40,
        };
        super::queue::compute::retirement::walk_completions(
            start,
            seen,
            |slot| completion(slot).read(0),
            |slot| record(completion(slot)),
        )
    }

    /// Folds every slot of the compute completion ring and records its control word in
    /// `seen`. For a pass that stopped before a record the firmware must already have
    /// written. Returns the slot after the newest record (latest end timestamp), where the
    /// firmware writes next.
    pub(super) fn resync_compute_completions(
        &self,
        seen: &mut [u64],
        mut record: impl FnMut(ComputeCompletion<'_>) -> Result,
    ) -> Result<usize> {
        let object = self.object(self.completion_rings[0])?;
        if seen.len() != COMPUTE_COMPLETION_SLOTS {
            return Err(EINVAL);
        }
        fence(Ordering::Acquire);
        let mut newest = None;
        for slot in 0..COMPUTE_COMPLETION_SLOTS {
            let completion = ComputeCompletion {
                object,
                offset: slot * 0x40,
            };
            let control = completion.read(0)?;
            seen[slot] = control;
            if control != 0 {
                let end = completion.read(5)? & COMPLETION_TIME_MASK;
                if newest.is_none_or(|(_, latest)| end > latest) {
                    newest = Some((slot, end));
                }
            }
            record(completion)?;
        }
        Ok(newest.map_or(0, |(slot, _)| (slot + 1) % COMPUTE_COMPLETION_SLOTS))
    }

    fn object(&self, index: usize) -> Result<&KernelObject> {
        self.objects.get(index).ok_or(EINVAL)
    }

    fn object_mut(&mut self, index: usize) -> Result<&mut KernelObject> {
        self.objects.get_mut(index).ok_or(EINVAL)
    }

    fn add(&mut self, object: KernelObject) -> Result<usize> {
        let index = self.objects.len();
        self.objects.push(object, GFP_KERNEL)?;
        Ok(index)
    }

    fn alias(&mut self, index: usize, vm: &mmu::Vm, va: u64, prot: mmu::Prot) -> Result {
        let alias = self.object_mut(index)?.map_alias(vm, va, prot)?;
        self.aliases.push(alias, GFP_KERNEL)?;
        Ok(())
    }

    /// Constructs the graph after the firmware has published the high-root entries.
    pub(super) fn build(
        &mut self,
        dev: &AsahiDrmDevice,
        cfg: &Config,
        uat: &mmu::Uat,
        render: &mmu::Vm,
        metrics: &KernelObject,
        clusters: u32,
        active_cores: u32,
        variant: &super::fw::Variant,
    ) -> Result {
        self.pm_metrics_va = cfg.pm_metrics.va;
        self.recovery_slot_keys = cfg.recovery_slot_keys;
        let range = uat.geometry().kernel_range();
        let env = BuildEnv {
            dev,
            vm: uat.kernel_vm(),
            lower: uat.kernel_lower_vm(),
            render,
            base: range.start,
            dynamic: range.start + t8140::dynamic::KERNEL_OFFSET..range.end,
            clusters,
        };
        if variant.graph == super::fw::variant::InitGraph::V25G83 {
            return self.build_25g83(&env, cfg, metrics, active_cores);
        }
        self.map_register_windows(&env, cfg.windows)?;
        let tables = self.build_descriptor_tables(&env, abi::private::SIZE)?;
        self.build_render_free_lists(&env)?;
        let first_block = self.build_render_pool(&env)?;
        self.build_compute_pools(&env, first_block)?;
        let bundle = self.build_bundle(&env, cfg, metrics)?;
        self.initialize_firmware_objects(&env, cfg, &tables, &bundle)
    }

    fn map_register_windows(
        &mut self,
        env: &BuildEnv<'_>,
        windows: &[abi::RegisterWindow],
    ) -> Result {
        for window in windows {
            for instance in 0..window.instances {
                let (phys, va, size) = window.mapping(instance).ok_or(EINVAL)?;
                self.aliases.push(
                    env.vm.map_io(
                        env.dev.as_ref().as_ref(),
                        va,
                        phys.try_into()?,
                        size.try_into()?,
                        mmu::PROT_FW_MMIO_RW,
                    )?,
                    GFP_KERNEL,
                )?;
            }
        }
        Ok(())
    }

    /// The private cluster, zero page and the three descriptor tables with their low aliases.
    fn build_descriptor_tables(
        &mut self,
        env: &BuildEnv<'_>,
        cluster_size: usize,
    ) -> Result<DescriptorTables> {
        use mmu::{
            PROT_FW_PRIV_RW as FP, PROT_FW_SHARED_RW as FS, PROT_GPU_FW_SHARED_RW as GFS,
            PROT_GPU_SHARED_RW as GS,
        };
        use CpuMap::{WriteBack as WB, WriteCombined as WC};
        self.cluster = self.add(env.fixed(high::PRIVATE_CLUSTER, cluster_size, FS, WC)?)?;
        self.compute_global = self.add(env.fixed(high::ZERO_PAGE, mmu::UAT_PGSZ, FP, WC)?)?;
        let pb = self.add(env.fixed(high::PB_DESCRIPTORS, mmu::UAT_PGSZ, FS, WC)?)?;
        self.object_mut(pb)?
            .initialize::<abi::PbDescriptorTable>(0, |table| {
                table.init();
                Ok(())
            })?;
        let page_pool = self.add(env.fixed(high::PAGE_POOL_DESCRIPTORS, mmu::UAT_PGSZ, FS, WC)?)?;
        self.object_mut(page_pool)?
            .initialize::<abi::PagePoolDescriptorTable>(0, |table| {
                table.init(low::RENDER_PAGE_LIST, low::COMPUTE_POOL_PAGE_LIST);
                Ok(())
            })?;
        self.pb = pb;
        self.page_pool = page_pool;
        self.alias(pb, env.lower, low::PB_DESCRIPTORS, GS)?;
        self.alias(page_pool, env.lower, low::PAGE_POOL_DESCRIPTORS, GS)?;
        let index = self.add(env.fixed(high::INDEX, INDEX_SIZE, GFS, WB)?)?;
        self.object_mut(index)?
            .initialize::<abi::IndexTable>(0, |table| {
                table.init();
                Ok(())
            })?;
        self.alias(index, env.lower, low::INDEX, GFS)?;
        let fwctl = self.add(env.fixed(high::FWCTL, abi::FWCTL_SIZE, FS, WB)?)?;
        Ok(DescriptorTables {
            pb,
            page_pool,
            fwctl,
        })
    }

    /// Render and boot free-list controls, descriptor pages and the two queue contexts.
    fn build_render_free_lists(&mut self, env: &BuildEnv<'_>) -> Result {
        use mmu::{
            PROT_FW_PRIV_RW as FP, PROT_FW_SHARED_RW as FS, PROT_GPU_FW_PRIV_RW as GFP,
            PROT_GPU_FW_SHARED_RW as GFS,
        };
        use CpuMap::WriteCombined as WC;
        for (offset, state, class, prot, initial) in [
            (
                high::RENDER_FREE_LIST,
                high::RENDER_FREE_LIST_STATE,
                abi::FreeListClass::One,
                FP,
                abi::FreeListState::RENDER,
            ),
            (
                high::BOOT_FREE_LIST,
                high::BOOT_FREE_LIST_STATE,
                abi::FreeListClass::Two,
                GFP,
                abi::FreeListState::BOOT,
            ),
        ] {
            let control = self.add(env.fixed(offset, mmu::UAT_PGSZ, prot, WC)?)?;
            self.object_mut(control)?.write(
                0,
                abi::FreeListControl::new(&abi::FreeListArgs {
                    buffer_slot: 0,
                    class,
                    page_list_va: low::RENDER_PAGE_LIST,
                    run_list_va: low::RENDER_RUN_LIST,
                    blocks: low::RENDER_BLOCKS_INITIAL,
                    state_va: env.base + state,
                }),
            )?;
            let state_prot = if class == abi::FreeListClass::One {
                FS
            } else {
                GFS
            };
            let state = self.add(env.fixed(state, mmu::UAT_PGSZ, state_prot, WC)?)?;
            self.object_mut(state)?.write(0, initial)?;
            if class == abi::FreeListClass::Two {
                self.boot_control = control;
                self.boot_state = state;
            }
        }
        self.stamps[0] = self.add(env.fixed(high::DESCRIPTOR_PAGE_A, mmu::UAT_PGSZ, GFS, WC)?)?;
        self.stamps[1] = self.add(env.fixed(high::DESCRIPTOR_PAGE_B, mmu::UAT_PGSZ, GFP, WC)?)?;
        self.add(env.fixed(high::TA_CONTEXT, CONTEXT_SIZE, FS, WC)?)?;
        self.add(env.fixed(high::FRAGMENT_CONTEXT, CONTEXT_SIZE, FS, WC)?)?;
        Ok(())
    }

    /// Render page list, run list and blocks aliased into the render-global root; returns
    /// the index of the first block.
    fn build_render_pool(&mut self, env: &BuildEnv<'_>) -> Result<usize> {
        use mmu::PROT_GPU_SHARED_RW as GS;
        use CpuMap::WriteBack as WB;
        let pages = self.add(KernelObject::backing(env.dev, low::RENDER_PAGE_LIST_SIZE, WB)?)?;
        self.object_mut(pages)?
            .initialize::<[u64; low::RENDER_PAGE_LIST_SIZE / 8]>(0, |list| {
                abi::fill_page_list(
                    list,
                    (0..low::RENDER_BLOCKS_INITIAL as usize).map(low::render_block),
                )
            })?;
        let runs = self.add(KernelObject::backing(env.dev, low::RENDER_RUN_LIST_SIZE, WB)?)?;
        let first_block = self.objects.len();
        for _ in 0..low::RENDER_BLOCKS_ALLOCATED {
            self.add(KernelObject::backing(env.dev, abi::FREE_LIST_BLOCK_SIZE, WB)?)?;
        }
        self.render_pages = pages;
        self.render_runs = runs;
        self.render_blocks = first_block..self.objects.len();
        self.alias(pages, env.render, low::RENDER_PAGE_LIST, GS)?;
        self.alias(runs, env.render, low::RENDER_RUN_LIST, GS)?;
        for block in 0..low::RENDER_BLOCKS_MAPPED {
            self.alias(first_block + block, env.render, low::render_block(block), GS)?;
        }
        Ok(first_block)
    }

    /// Compute free-list controls and states, zero blocks, run list and the compute page
    /// list inside the render blocks.
    fn build_compute_pools(&mut self, env: &BuildEnv<'_>, first_block: usize) -> Result {
        use mmu::PROT_GPU_FW_SHARED_RW as GFS;
        use CpuMap::{WriteBack as WB, WriteCombined as WC};
        let compute = self.add(env.fixed(high::COMPUTE_FREE_LIST, mmu::UAT_PGSZ, GFS, WC)?)?;
        self.object_mut(compute)?.write(
            0,
            abi::FreeListControl::new(&abi::FreeListArgs {
                buffer_slot: 0,
                class: abi::FreeListClass::One,
                page_list_va: low::COMPUTE_PAGE_LIST,
                run_list_va: low::COMPUTE_RUN_LIST,
                blocks: low::COMPUTE_BLOCKS,
                state_va: env.base + high::COMPUTE_FREE_LIST_STATE,
            }),
        )?;
        let compute_state = self.add(KernelObject::backing(env.dev, mmu::UAT_PGSZ, WB)?)?;
        self.object_mut(compute_state)?
            .write(0, abi::FreeListState::COMPUTE)?;
        let compute_runs = self.add(KernelObject::new(
            env.dev,
            env.lower,
            Placement::At(low::COMPUTE_RUN_LIST),
            mmu::UAT_PGSZ,
            GFS,
            WC,
        )?)?;
        self.object_mut(compute_runs)?
            .initialize::<[abi::FreeListRunSlot; mmu::UAT_PGSZ / 0x40]>(0, |list| {
                abi::fill_run_list(list, low::COMPUTE_FREE_LIST_BLOCKS)
            })?;
        let ready = self.add(env.fixed(
            high::COMPUTE_READY_FREE_LIST,
            mmu::UAT_PGSZ,
            GFS,
            WC,
        )?)?;
        self.object_mut(ready)?.write(
            0,
            abi::FreeListControl::new(&abi::FreeListArgs {
                buffer_slot: 1,
                class: abi::FreeListClass::One,
                page_list_va: low::COMPUTE_POOL_PAGE_LIST,
                run_list_va: low::COMPUTE_POOL_RUN_LIST,
                blocks: low::COMPUTE_READY_BLOCKS,
                state_va: env.base + high::COMPUTE_READY_FREE_LIST_STATE,
            }),
        )?;
        for state in [high::COMPUTE_READY_FREE_LIST_STATE, high::COMPUTE_STATE] {
            let obj = self.add(env.fixed(state, mmu::UAT_PGSZ, GFS, WC)?)?;
            self.object_mut(obj)?
                .write(0, abi::FreeListState::COMPUTE)?;
        }
        self.object_mut(first_block + low::COMPUTE_PAGE_LIST_BLOCK)?
            .initialize::<[u64; abi::FREE_LIST_BLOCK_SIZE / 8]>(0, |list| {
                abi::fill_page_list(list, low::COMPUTE_FREE_LIST_BLOCKS)
            })?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// The bundle with its zero page and completion rings, the QoS table, the metrics alias,
    /// the roots and regions A and C.
    fn build_bundle(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        metrics: &KernelObject,
    ) -> Result<BundleObjects> {
        use mmu::{
            PROT_FW_PRIV_RW as FP, PROT_FW_SHARED_RW as FS, PROT_GPU_FW_SHARED_RW as GFS,
            PROT_GPU_SHARED_RW as GS,
        };
        use CpuMap::{WriteBack as WB, WriteCombined as WC};
        self.bundle = self.add(env.fixed(high::BUNDLE, abi::bundle::SIZE, FP, WC)?)?;
        let zero = self.add(env.fixed(
            high::BUNDLE + abi::bundle::ZERO_PAGE as u64,
            mmu::UAT_PGSZ,
            GFS,
            WC,
        )?)?;
        let mut completion_rings = [abi::CompletionRing {
            low_va: 0,
            high_va: 0,
        }; 2];
        for (slot, completion_ring) in completion_rings.iter_mut().enumerate() {
            let offset = high::BUNDLE + abi::bundle::COMPLETION_RINGS[slot] as u64;
            let ring = self.add(env.fixed(offset, abi::COMPLETION_RING_SIZE, FP, WB)?)?;
            self.completion_rings[slot] = ring;
            self.alias(ring, env.lower, low::COMPLETION_RINGS[slot], GS)?;
            *completion_ring = abi::CompletionRing {
                low_va: low::COMPLETION_RINGS[slot],
                high_va: env.base + offset,
            };
        }
        let qos = self.add(env.dynamic(abi::QOS_SIZE, mmu::UAT_PGSZ as u64, FS, WC)?)?;
        self.qos = qos;
        fence(Ordering::SeqCst);
        let metrics_alias =
            metrics.alias_in(env.vm, env.dynamic.clone(), cfg.pm_metrics.size as u64, GFS)?;
        self.metrics_fw_va = metrics_alias.iova();
        self.aliases.push(metrics_alias, GFP_KERNEL)?;
        fence(Ordering::SeqCst);
        self.roots = self.add(env.dynamic(
            abi::ROOTS_SIZE,
            abi::SECONDARY_ROOT_OFFSET as u64,
            mmu::PROT_FW_SHARED_RO,
            WB,
        )?)?;
        let region_a = self.add(env.dynamic(
            REGION_A_SIZE,
            mmu::UAT_PGSZ as u64,
            mmu::PROT_FW_SHARED_RO,
            WB,
        )?)?;
        self.region_c = self.add(env.dynamic(REGION_C_SIZE, mmu::UAT_PGSZ as u64, FS, WB)?)?;
        Ok(BundleObjects {
            zero,
            completion_rings,
            region_a,
            qos,
        })
    }

    /// Writes every firmware object the host builds from constants and addresses, then the
    /// opening control-ring state for both roles.
    fn initialize_firmware_objects(
        &mut self,
        env: &BuildEnv<'_>,
        cfg: &Config,
        tables: &DescriptorTables,
        bundle: &BundleObjects,
    ) -> Result {
        let base = env.base;
        let opps = Self::opps(env.dev, cfg.opps, env.clusters)?;
        let hw_args = abi::HwDataArgs {
            chip_id: cfg.chip_id,
            windows: cfg.windows,
            reserved_slots: cfg.reserved_slots,
            opps: &opps,
            perf: cfg.perf,
            completion_rings: bundle.completion_rings,
            qos_va: self.object(bundle.qos)?.gpu_va(),
        };
        self.object_mut(self.bundle)?
            .initialize::<abi::HwData>(abi::bundle::HW_DATA, |hw| hw.init(&hw_args))?;
        self.object_mut(self.region_c)?
            .initialize::<abi::RegionC>(0, |region| {
                region.init(cfg.region_c);
                Ok(())
            })?;
        self.object_mut(self.cluster)?
            .write(abi::private::PRIMARY_STATUS_A, abi::StatusBlock::new())?;
        self.object_mut(self.cluster)?
            .write(abi::private::SECONDARY_STATUS_A, abi::StatusBlock::new())?;
        let fwctl_va = self.object(tables.fwctl)?.gpu_va();
        self.object_mut(self.cluster)?
            .initialize::<abi::PrimaryStatusB>(abi::private::STATUS_B, |status| {
                status.init(fwctl_va);
                Ok(())
            })?;
        self.object_mut(self.cluster)?
            .initialize::<abi::SecondaryConfig>(
                abi::private::SECONDARY_STATE + abi::private::STATUS_B,
                |config| {
                    config.init();
                    Ok(())
                },
            )?;
        self.object_mut(self.bundle)?
            .initialize::<abi::PowerConfig>(abi::bundle::POWER_CONFIG, |power| {
                power.init();
                Ok(())
            })?;
        let secondary_status = base
            + high::PRIVATE_CLUSTER
            + (abi::private::SECONDARY_STATE + abi::private::SECONDARY_STATUS) as u64;
        self.object_mut(self.bundle)?
            .initialize::<abi::HwDataAux>(abi::bundle::AUX, |aux| {
                aux.init(secondary_status);
                Ok(())
            })?;
        self.object_mut(self.bundle)?
            .initialize::<abi::PowerConfigHead>(abi::bundle::POWER_CONFIG_COPY, |power| {
                power.init();
                Ok(())
            })?;
        let views = abi::PrimaryViews {
            zero_page_va: self.object(bundle.zero)?.gpu_va(),
            pb_descriptors_va: self.object(tables.pb)?.gpu_va(),
            page_pool_descriptors_va: self.object(tables.page_pool)?.gpu_va(),
        };
        for role in [Role::Primary, Role::Secondary] {
            let slot = role as usize;
            let main_offset = abi::bundle::MAIN_CONFIG[slot];
            let main_va = base + high::BUNDLE + main_offset as u64;
            let state_va = base + high::PRIVATE_CLUSTER + self.state_offset(role) as u64;
            let status_va = base + high::PRIVATE_CLUSTER + self.status_offset(role) as u64;
            let args = abi::MainConfigArgs {
                bundle_va: base + high::BUNDLE,
                main_config_va: main_va,
                state_grid_va: state_va,
                status_a_va: status_va,
                pb_descriptors_low_va: low::PB_DESCRIPTORS,
                page_pool_descriptors_low_va: low::PAGE_POOL_DESCRIPTORS,
            };
            self.object_mut(self.bundle)?
                .initialize::<abi::MainConfig>(main_offset, |main| {
                    match role {
                        Role::Primary => main.init_primary(&args, &views),
                        Role::Secondary => main.init_secondary(&args),
                    };
                    Ok(())
                })?;
            let abi_role = match role {
                Role::Primary => abi::Role::Primary,
                Role::Secondary => abi::Role::Secondary,
            };
            self.object_mut(self.bundle)?.write(
                main_offset + abi::MainConfig::CONTROL_RING,
                abi::OpeningRecord::new(abi_role),
            )?;
            let root = abi::RootArgs {
                region_a_va: self.object(bundle.region_a)?.gpu_va(),
                region_c_va: self.object(self.region_c)?.gpu_va(),
                main_config_va: main_va,
                status_a_va: status_va,
                ias: cfg.uat_ias,
                oas: cfg.uat_oas,
            };
            match role {
                Role::Primary => self.object_mut(self.roots)?.write(
                    0,
                    abi::Root::primary(&root, state_va + abi::private::STATUS_B as u64),
                )?,
                Role::Secondary => self.object_mut(self.roots)?.write(
                    abi::SECONDARY_ROOT_OFFSET,
                    abi::SecondaryRoot::new(
                        &root,
                        base + high::PRIVATE_CLUSTER
                            + (abi::private::STATUS_B + abi::PrimaryStatusB::POWER_STATE) as u64,
                        state_va + abi::private::STATUS_B as u64,
                    ),
                )?,
            }
        }
        for role in [Role::Primary, Role::Secondary] {
            for (offset, value) in [
                (0, 0),
                (0x10, 0),
                (abi::ChannelState::PRODUCER, abi::OPENING_PRODUCER),
            ] {
                self.object(self.cluster)?
                    .word(self.state_offset(role) + abi::CONTROL_STATE_OFFSET + offset)?
                    .store(value, Ordering::Relaxed);
            }
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Retained fixed aliases for an ordinary client VM. The final block of the render-global
    /// pool overlaps completion mappings in a client root and is deliberately not installed.
    pub(super) fn user_aliases(
        &mut self,
        vm: &mmu::Vm,
        metrics: &KernelObject,
    ) -> Result<KVec<mmu::KernelMapping>> {
        let mut mappings = KVec::with_capacity(24, GFP_KERNEL)?;
        mappings.push(
            self.object_mut(self.render_pages)?.map_alias(
                vm,
                low::RENDER_PAGE_LIST,
                mmu::PROT_GPU_SHARED_RW,
            )?,
            GFP_KERNEL,
        )?;
        mappings.push(
            self.object_mut(self.render_runs)?.map_alias(
                vm,
                low::RENDER_RUN_LIST,
                mmu::PROT_GPU_SHARED_RW,
            )?,
            GFP_KERNEL,
        )?;
        for (block, index) in self.render_blocks.clone().take(21).enumerate() {
            mappings.push(
                self.object_mut(index)?.map_alias(
                    vm,
                    low::render_block(block),
                    mmu::PROT_GPU_SHARED_RW,
                )?,
                GFP_KERNEL,
            )?;
        }
        mappings.push(
            metrics.map_alias(vm, self.pm_metrics_va, mmu::PROT_GPU_FW_SHARED_RW)?,
            GFP_KERNEL,
        )?;
        Ok(mappings)
    }

    /// Writes the effort input in firmware order, then cleans its cache line to coherency.
    pub(super) fn publish_effort(&self, effort: u32) -> Result {
        let region = self.object(self.region_c)?;
        let override_word = region.word(abi::RegionC::UNK_E8)?;
        let effort_word = region.word(abi::RegionC::EFFORT)?;
        let line = region.pointer(abi::RegionC::EFFORT, 4)?;
        override_word.store(0, Ordering::Relaxed);
        effort_word.store(effort, Ordering::Relaxed);
        // SAFETY: line points into the retained kernel mapping; cleaning this host-owned cache
        // line and the following system barrier publish the two naturally aligned stores.
        unsafe {
            core::arch::asm!("dc cvac, {line}", "dsb sy", line = in(reg) line,
            options(nostack, preserves_flags))
        };
        Ok(())
    }

    pub(super) fn effort(&self) -> Result<u32> {
        Ok(self
            .object(self.region_c)?
            .word(abi::RegionC::EFFORT)?
            .load(Ordering::Relaxed))
    }

    /// The GPU's operating points: `count` of them, each with one voltage cell for the whole
    /// GPU or one per cluster.
    pub(super) fn opps(
        dev: &AsahiDrmDevice,
        count: usize,
        clusters: u32,
    ) -> Result<KVec<abi::Opp>> {
        let opps = Self::opp_table(dev, c_str!("operating-points-v2"))?;
        if opps.len() != count {
            return Err(EINVAL);
        }
        let cells = opps.first().map_or(0, |opp| opp.cells);
        if cells != 1 && u32::try_from(cells) != Ok(clusters) {
            dev_err!(
                dev.as_ref(),
                "GPU operating points have {} voltage cells, expected 1 or {}\n",
                cells,
                clusters
            );
            return Err(EINVAL);
        }
        Ok(opps)
    }

    /// The operating points of the table that the GPU node's property `name` names; all have
    /// the same number of voltage cells.
    fn opp_table(dev: &AsahiDrmDevice, name: &kernel::str::CStr) -> Result<KVec<abi::Opp>> {
        let node = dev.as_ref().as_ref().of_node().ok_or(EIO)?;
        let table = node.parse_phandle(name, 0).ok_or(EIO)?;
        let mut opps = KVec::new();
        for opp in table.children() {
            // One cell for the whole GPU, or one per cluster.
            let cells: KVec<u32> = opp.get_property(c_str!("opp-microvolt"))?;
            if cells.is_empty() || cells.len() > abi::OPP_CELLS {
                return Err(EINVAL);
            }
            let mut volt_uv = [0; abi::OPP_CELLS];
            volt_uv[..cells.len()].copy_from_slice(&cells);
            opps.push(
                abi::Opp {
                    freq_hz: opp.get_property(c_str!("opp-hz"))?,
                    volt_uv,
                    cells: cells.len(),
                },
                GFP_KERNEL,
            )?;
        }
        if opps.iter().any(|opp| opp.cells != opps[0].cells) {
            return Err(EINVAL);
        }
        Ok(opps)
    }

    fn state_offset(&self, role: Role) -> usize {
        match role {
            Role::Primary => abi::private::PRIMARY_STATE,
            Role::Secondary => abi::private::SECONDARY_STATE,
        }
    }

    fn status_offset(&self, role: Role) -> usize {
        match role {
            Role::Primary => self.status_a,
            Role::Secondary => abi::private::SECONDARY_STATUS_A,
        }
    }

    /// Recheck the host-authored event pointer pair after a bounded number of drains.
    /// The device mutex serializes the cache and all event consumption.
    pub(super) fn validate_event_ring(&self, role: Role) -> Result {
        let status = self.object(self.cluster)?.gpu_va() + self.status_offset(role) as u64;
        let expected = (
            status + abi::status::EVENT_STATE,
            status + abi::status::EVENT_RING,
        );
        let cache = &self.event_ring_checked[role as usize];
        let checked = cache.get();
        if checked.2 != 0 && (checked.0, checked.1) == expected {
            cache.set((checked.0, checked.1, checked.2 - 1));
            return Ok(());
        }
        let offset = abi::bundle::MAIN_CONFIG[role as usize]
            + core::mem::offset_of!(abi::MainConfig, channels)
            + abi::EVENT_CHANNEL * size_of::<abi::ChannelEntry>();
        let bundle = self.object(self.bundle)?;
        let actual = (
            bundle.dword(offset)?.load(Ordering::Relaxed),
            bundle
                .dword(offset + size_of::<u64>())?
                .load(Ordering::Relaxed),
        );
        if actual != expected {
            cache.set((0, 0, 0));
            return Err(EINVAL);
        }
        cache.set((actual.0, actual.1, EVENT_RING_RECHECK));
        Ok(())
    }

    /// Whether a firmware that acknowledges in status A accepted the graph (`+0x14` = 1).
    pub(super) fn init_accepted(&self) -> Result<bool> {
        Ok(self.status_word(Role::Primary, 0x14)?.load(Ordering::Acquire) == 1)
    }

    pub(super) fn status_word(&self, role: Role, offset: usize) -> Result<&AtomicU32> {
        self.object(self.cluster)?.word(
            self.status_offset(role)
                .checked_add(offset)
                .ok_or(EINVAL)?,
        )
    }

    pub(super) fn status_read_words<const N: usize>(
        &self,
        role: Role,
        offset: usize,
    ) -> Result<[u32; N]> {
        self.object(self.cluster)?.read_words(
            self.status_offset(role)
                .checked_add(offset)
                .ok_or(EINVAL)?,
        )
    }

    pub(super) fn control_counters(&self) -> Result<[u32; 3]> {
        let state = self.object(self.cluster)?;
        let mut counters = [0; 3];
        fence(Ordering::Acquire);
        for (counter, offset) in counters.iter_mut().zip([
            abi::ChannelState::CONSUMER,
            abi::ChannelState::UNK_10,
            abi::ChannelState::PRODUCER,
        ]) {
            *counter = state
                .word(abi::CONTROL_STATE_OFFSET + offset)?
                .load(Ordering::Relaxed);
        }
        Ok(counters)
    }

    pub(super) fn control_consumer(&self) -> Result<&AtomicU32> {
        self.object(self.cluster)?
            .word(abi::CONTROL_STATE_OFFSET + abi::ChannelState::CONSUMER)
    }

    pub(super) fn control_producer(&self) -> Result<&AtomicU32> {
        self.object(self.cluster)?
            .word(abi::CONTROL_STATE_OFFSET + abi::ChannelState::PRODUCER)
    }

    /// Publishes a control record under the device lock. All checked views are acquired before
    /// the first write; record and packed power tally precede the aligned producer store.
    pub(super) fn publish_control(
        &self,
        slot: u32,
        record: &[u8; 64],
        asserts_power: bool,
        next: u32,
    ) -> Result {
        let offset = abi::bundle::MAIN_CONFIG[Role::Primary as usize]
            + abi::MainConfig::CONTROL_RING
            + (slot as usize).checked_mul(record.len()).ok_or(EINVAL)?;
        let ring = self.object(self.bundle)?.pointer(offset, record.len())?;
        let power = self
            .object(self.region_c)?
            .pointer(abi::RegionC::POWER_ASSERT_TALLY, 4)?;
        let producer = self.control_producer()?;
        // SAFETY: The checked pointers remain valid for the lifetime of this non-Sync owner.
        // Firmware only reads records after the producer store and the device mutex serializes
        // host publishers. The unaligned power word uses the interface's four-byte copy access.
        unsafe {
            core::ptr::copy_nonoverlapping(record.as_ptr(), ring, record.len());
            if asserts_power {
                let mut tally = [0u8; 4];
                core::ptr::copy_nonoverlapping(power, tally.as_mut_ptr(), 4);
                let value = u32::from_le_bytes(tally).wrapping_add(1).to_le_bytes();
                core::ptr::copy_nonoverlapping(value.as_ptr(), power, 4);
            }
        }
        fence(Ordering::SeqCst);
        producer.store(next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(())
    }

    pub(super) fn root_va(&self, role: Role) -> Result<u64> {
        Ok(self.object(self.roots)?.gpu_va() + role as u64 * abi::SECONDARY_ROOT_OFFSET as u64)
    }

    pub(super) fn opening_retired(&self, role: Role) -> Result<bool> {
        for offset in [0, 0x10, abi::ChannelState::PRODUCER] {
            if self
                .object(self.cluster)?
                .word(self.state_offset(role) + abi::CONTROL_STATE_OFFSET + offset)?
                .load(Ordering::SeqCst)
                != abi::OPENING_PRODUCER
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

impl InitData {
    /// Validate the serialized view before reading its retained host mapping.
    fn recovery_view_words<const N: usize>(&self, view: usize, offset: usize) -> Result<[u32; N]> {
        let bundle = self.object(self.bundle)?;
        let extent = *abi::recovery::VIEW_EXTENTS.get(view).ok_or(EINVAL)?;
        // Separately allocated views live at offset 0 of their own object.
        let (object, base) = match self.views {
            Some(views) => (self.object(*views.get(view).ok_or(EINVAL)?)?, 0),
            None => (bundle, *abi::bundle::VIEWS.get(view).ok_or(EINVAL)?),
        };
        let end = offset.checked_add(N.checked_mul(size_of::<u32>()).ok_or(EOVERFLOW)?)
            .ok_or(EOVERFLOW)?;
        if end > extent {
            return Err(EINVAL);
        }
        let pointer_at = abi::bundle::MAIN_CONFIG[Role::Primary as usize]
            + core::mem::offset_of!(abi::MainConfig, view_va) + view * size_of::<u64>();
        fence(Ordering::Acquire);
        let pointer = bundle.read_words::<2>(pointer_at)?;
        let actual = u64::from(pointer[0]) | u64::from(pointer[1]) << 32;
        if actual != object.gpu_va().checked_add(base as u64).ok_or(EOVERFLOW)? {
            return Err(EINVAL);
        }
        object.read_words(base + offset)
    }
}

impl super::recovery::Memory for InitData {
    fn fault_sources(&self) -> Result<super::recovery::Sources> {
        let mask = self.object(self.cluster)?
            .dword(self.status_b + abi::PrimaryStatusB::RECOVERY_SLOT_MASK)?
            .load(Ordering::Relaxed);
        let mut sources = super::recovery::Sources::sample(mask as u32, |slot| {
            // Progress is independently available even when this firmware's key
            // address is unknown. Zero keeps ownership unknown and cannot override
            // firmware blame, but preserves the progress evidence for diagnostics.
            let key = super::recovery::slot_key(slot, self.recovery_slot_keys).unwrap_or(0);
            let monitor = self.recovery_view_words::<6>(abi::recovery::PROGRESS_VIEW,
                abi::recovery::PROGRESS_START + slot * abi::recovery::PROGRESS_STRIDE)
                .ok().map(|words| words[0])?;
            Some((key, monitor))
        });
        sources.reason = self.recovery_view_words::<1>(abi::recovery::REPORT_VIEW, abi::recovery::REASON)
            .ok().map(|words| words[0]);
        Ok(sources)
    }

    fn recovery_state(&self) -> Result<u32> {
        fence(Ordering::Acquire);
        Ok(self
            .object(self.cluster)?
            .word(self.status_b + abi::PrimaryStatusB::RECOVERY_STATE)?
            .load(Ordering::Relaxed))
    }

    fn host_recovery(&self) -> Result<u32> {
        Ok(self
            .object(self.cluster)?
            .word(self.status_b + abi::PrimaryStatusB::HOST_RECOVERY)?
            .load(Ordering::Relaxed))
    }

    fn transition(&self, expected: u32, next: u32) -> Result {
        let state = self
            .object(self.cluster)?
            .word(self.status_b + abi::PrimaryStatusB::RECOVERY_STATE)?;
        fence(Ordering::Acquire);
        if state.load(Ordering::Relaxed) != expected {
            return Err(EBUSY);
        }
        fence(Ordering::SeqCst);
        state.store(next, Ordering::Relaxed);
        fence(Ordering::SeqCst);
        Ok(())
    }

    fn clear_timestamps(&self) -> Result {
        // The per-QID timestamp table is view 4; separately allocated views live at offset 0 of
        // their own object.
        let (table, offset) = match self.views {
            Some(views) => (self.object(*views.get(4).ok_or(EINVAL)?)?, 0),
            None => (self.object(self.bundle)?, abi::bundle::VIEWS[4]),
        };
        // Validate the complete table before publishing its first live word.
        const ENTRIES: usize = 128;
        const STRIDE: usize = 0x10;
        table.pointer(offset, ENTRIES * STRIDE)?;
        for qid in 0..ENTRIES {
            table
                .word(offset + qid * STRIDE)?
                .store(0, Ordering::Relaxed);
            table
                .dword(offset + qid * STRIDE + 8)?
                .store(0, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    fn entry(&self, index: usize) -> Result<(u32, u32)> {
        if index >= super::recovery::RECOVERY_RECORDS {
            return Err(EINVAL);
        }
        let cluster = self.object(self.cluster)?;
        let offset = self.status_b
            + abi::PrimaryStatusB::RECOVERY_INFO
            + index * core::mem::size_of::<abi::RecoveryInfoEntry>();
        fence(Ordering::Acquire);
        let flags = cluster
            .word(offset + core::mem::offset_of!(abi::RecoveryInfoEntry, flags))?
            .load(Ordering::Relaxed);
        let stamp = cluster
            .word(offset + core::mem::offset_of!(abi::RecoveryInfoEntry, stamp))?
            .load(Ordering::Relaxed);
        Ok((flags, stamp))
    }

    fn wait_tick(&self) {
        kernel::time::delay::fsleep(kernel::time::Delta::from_millis(1));
    }
}
