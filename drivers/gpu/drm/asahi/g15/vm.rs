// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! User VMs on G15.
//!
//! A user VM is a shared-layer [`mmu::Vm`] created by the adopted UAT, with tracked status
//! ([`mmu::Vm::with_tracked_status`]: VM_STATUS, mirror, job guards, deferred close; no residency
//! reclaim). At creation:
//!
//! - install the client aliases of the firmware objects marked for every user VM
//!   ([`object::Objects::clients`]) as driver mappings at their fixed GVAs, and reserve
//!   [`super::hw::client::CLIENT_APERTURE`], the pass-slot range and the PB page list
//!   (`vm.install_driver_mappings`): user binds there fail (EINVAL). The aperture holds the
//!   whole PB range, whose backing chunks are mapped later, before the VM's prefixes are
//!   published (`Firmware::map_pb`) and, for the VM of the flights, when a chunk is backed
//!   ([`VmState::map_heap`], [`VmState::map_chunk`]);
//! - leave it unbound until a runnable job needs a TTBAT slot. Each published job holds
//!   its binding through completion; failed published jobs retain theirs until reboot.
//!
//! The registry keeps a VM's state while its client handle is open. Queues and jobs retain
//! closed VMs until their last reference drops.
//!
//! User page tables are cleaned to the point of coherency by the shared `pgtable.rs` (enabled by
//! the adopted UAT); user objects are prepared at creation (`gpu::Gpu::prepare_new_objects`).
//! User leaves use the shared GPU-shared protections (AttrIndex 2) for NC and WRITEBACK objects
//! alike.

use core::ops::Range;
use core::sync::atomic::Ordering;
use kernel::{
    drm::gem::BaseObject,
    new_mutex,
    prelude::*,
    sync::{aref::ARef, Arc, Mutex},
};

use super::{hw, object, Gpu};
use crate::{debug::*, g17::status::VmStatus, gem, mmu};

const DEBUG_CLASS: DebugFlags = DebugFlags::Mmu;

/// Descriptor bits the shared page-table code adds to every user leaf: valid page, non-global.
const USER_LEAF_BITS: u64 = 0x803;

/// A firmware object aliased at a fixed GVA in every user VM.
struct ClientAlias {
    /// The object (a GEM kernel object); its first `size` bytes are mapped.
    gem: ARef<gem::Object>,
    /// Page-aligned client GVA of the object's first page.
    gva: u64,
    /// Mapped size, whole pages.
    size: usize,
    /// The exact client leaf word ([`hw::pte::LEAF_NC`], [`hw::pte::LEAF_CACHED`] or
    /// [`hw::pte::LEAF_PB_LIST`]).
    leaf: u64,
}

impl ClientAlias {
    /// The client aliases of the firmware object allocator, validated: whole pages of the
    /// object, inside a range reserved for the kernel, and a leaf word that round-trips exactly
    /// through the shared page-table code.
    fn collect(objects: &object::Objects) -> Result<KVec<ClientAlias>> {
        let clients = objects.clients();
        let mut aliases = KVec::with_capacity(clients.len(), GFP_KERNEL)?;
        for object in clients {
            let Some((gva, leaf)) = object.client() else {
                continue;
            };
            let alias = ClientAlias {
                gem: object.gem(),
                gva,
                size: object.bytes(),
                leaf,
            };
            alias.check()?;
            aliases.push(alias, GFP_KERNEL)?;
        }
        Ok(aliases)
    }

    fn check(&self) -> Result {
        let end = self.gva.checked_add(self.size as u64).ok_or(EINVAL)?;
        let leaf = mmu::Prot::from_pte(self.leaf).leaf_bits() | USER_LEAF_BITS;
        if self.size == 0
            || (self.gva | self.size as u64) & mmu::UAT_PGMSK as u64 != 0
            || self.size > self.gem.size()
            || leaf != self.leaf
            || !reserved_ranges()
                .iter()
                .any(|range| range.start <= self.gva && end <= range.end)
        {
            pr_err!(
                "G15: refusing client alias {:#x}+{:#x} leaf {:#x}\n",
                self.gva,
                self.size,
                self.leaf
            );
            return Err(EINVAL);
        }
        Ok(())
    }
}

/// A PB heap chunk mapped in a VM at its fixed GVA ([`super::pools::chunk_gva`]).
struct HeapMapping {
    gem: ARef<gem::Object>,
    _mapping: mmu::KernelMapping,
}

/// The PB heap chunks mapped in a VM, in heap order (chunk `i` at `chunk_gva(i)`). Changed
/// under the device lock (refill, publication).
#[pin_data]
struct HeapMappings {
    #[pin]
    list: Mutex<KVec<HeapMapping>>,
}

/// G15 state of one user VM, shared by the registry, its queues and its jobs.
pub(crate) struct VmState {
    /// The `mmu::Vm` identifier (`Gpu::ids.vm`).
    pub(crate) id: u64,
    /// The sticky status (shared with `file.rs`).
    pub(crate) status: Arc<VmStatus>,
    /// The VM: span checks, stream reads and job guards.
    pub(crate) vm: mmu::Vm,
    /// The adopted UAT, used to bind published jobs without waiting for a slot.
    uat: Arc<mmu::Uat>,
    registry: Arc<Registry>,
    /// The PB heap chunks mapped in this VM.
    heap: Arc<HeapMappings>,
}

impl VmState {
    pub(crate) fn try_bind(&self) -> Result<mmu::VmBind> {
        self.uat.try_bind(&self.vm)
    }

    /// Drops the registry's references to closed VMs (the queues call this when destroyed).
    pub(crate) fn prune_registry(&self) {
        self.registry.prune();
    }

    /// Maps PB heap chunk `i`, the next one this VM lacks, at its GVA. The alias is held only
    /// once it is in the list.
    fn map_one(&self, heap: &mut KVec<HeapMapping>, i: usize, gem: ARef<gem::Object>) -> Result {
        if i != heap.len() {
            return Err(EINVAL);
        }
        heap.reserve(1, GFP_KERNEL)?;
        let mapping = self.vm.map_at(
            super::pools::chunk_gva(i),
            super::pools::CHUNK_BYTES,
            &gem,
            mmu::Prot::from_pte(hw::pte::LEAF_CACHED),
            false,
        )?;
        heap.push_within_capacity(HeapMapping {
            gem,
            _mapping: mapping,
        })
        .map_err(|_| ENOMEM)
    }

    /// Maps the backed chunks (`gems`, in heap order) this VM does not hold yet. A chunk whose
    /// object changed since (freed and backed again) is unmapped first, with everything above
    /// it, and so are chunks past the backing.
    pub(crate) fn map_heap(&self, gems: impl Iterator<Item = ARef<gem::Object>>) -> Result {
        let mut heap = self.heap.list.lock();
        let mut n = 0;
        for (i, gem) in gems.enumerate() {
            n = i + 1;
            if let Some(have) = heap.get(i) {
                if core::ptr::eq(&*have.gem, &*gem) {
                    continue;
                }
                heap.truncate(i);
            }
            self.map_one(&mut heap, i, gem)?;
        }
        heap.truncate(n);
        Ok(())
    }

    /// Maps the newly backed chunk `i` into this VM (the VM of a flight, at a refill).
    pub(crate) fn map_chunk(&self, i: usize, gem: ARef<gem::Object>) -> Result {
        let mut heap = self.heap.list.lock();
        if i < heap.len() {
            // Already mapped (a flight of the same VM came first).
            return Ok(());
        }
        self.map_one(&mut heap, i, gem)
    }

    /// Unmaps the heap chunks from index `keep` on.
    pub(crate) fn unmap_heap_above(&self, keep: usize) {
        self.heap.list.lock().truncate(keep);
    }
}

/// The VMs whose client handle is open.
#[pin_data]
pub(crate) struct Registry {
    #[pin]
    vms: Mutex<KVec<Arc<VmState>>>,
}

impl Registry {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                vms <- new_mutex!(KVec::new(), "g15::Registry::vms"),
            }),
            GFP_KERNEL,
        )
    }

    /// Drops the registry's references to VMs whose client handle is closed. Called on VM close
    /// ([`closed`]), on queue destruction and before a new VM is created: a closed VM's page
    /// tables and bound objects go as soon as its last queue or job is gone.
    fn prune(&self) {
        let mut closed = KVec::new();
        {
            let mut vms = self.vms.lock();
            let mut index = 0;
            while index < vms.len() {
                if vms[index].vm.is_closed() && closed.push(vms[index].clone(), GFP_KERNEL).is_ok()
                {
                    // Cannot fail: `index` is in range.
                    let _ = vms.remove(index);
                } else {
                    index += 1;
                }
            }
        }
        // VM teardown may take GPUVM and reservation locks: not under the registry lock.
        drop(closed);
    }

    /// The driver mappings and reserved ranges of a new VM.
    fn driver_mappings(
        vm: &mmu::Vm,
        aliases: KVec<ClientAlias>,
    ) -> Result<(KVec<mmu::KernelMapping>, KVec<Range<u64>>)> {
        let mut mappings = KVec::with_capacity(aliases.len(), GFP_KERNEL)?;
        for alias in aliases {
            mappings.push(
                vm.map_at(
                    alias.gva,
                    alias.size,
                    &alias.gem,
                    mmu::Prot::from_pte(alias.leaf),
                    false,
                )?,
                GFP_KERNEL,
            )?;
        }
        let ranges = reserved_ranges();
        let mut reserved = KVec::with_capacity(ranges.len(), GFP_KERNEL)?;
        for range in ranges {
            reserved.push(range, GFP_KERNEL)?;
        }
        Ok((mappings, reserved))
    }

    /// The state of a VM whose client handle is open.
    pub(crate) fn find(&self, vm: &mmu::Vm) -> Result<Arc<VmState>> {
        let id = vm.id();
        self.vms
            .lock()
            .iter()
            .find(|state| state.id == id)
            .cloned()
            .ok_or(ENOENT)
    }

    /// The VMs whose client handle is open (and the closed ones not pruned yet), for a walk
    /// that takes VM locks: not under the registry lock.
    pub(crate) fn open(&self) -> Result<KVec<Arc<VmState>>> {
        let vms = self.vms.lock();
        let mut out = KVec::with_capacity(vms.len(), GFP_KERNEL)?;
        for state in vms.iter() {
            out.push(state.clone(), GFP_KERNEL)?;
        }
        Ok(out)
    }

    /// Latches `error` on every VM whose client handle is open. The fail-stop path calls it under
    /// the device lock.
    pub(crate) fn fail_all(&self, error: Error) {
        for state in self.vms.lock().iter() {
            state.status.record(error);
        }
    }
}

/// User VA ranges that belong to the kernel in every VM: the client aperture, the render pass
/// slots and the PB page list (the last two need 32-bit compact pointers). Every client alias
/// lies inside one of them ([`ClientAlias::check`]).
pub(crate) fn reserved_ranges() -> [Range<u64>; 3] {
    use hw::client::*;
    [
        CLIENT_APERTURE,
        PRE0_PAGE..PRE0_PAGE + hw::sizes::RENDER_SLOTS as u64 * PASS_STRIDE,
        PB_PAGE_LIST..PB_PAGE_LIST + PB_PAGE_LIST_SIZE,
    ]
}

/// Creates a user VM (`gpu::Gpu::new_vm`).
pub(crate) fn new_vm(gpu: &Gpu, kernel_range: Range<u64>) -> Result<mmu::Vm> {
    if gpu.shared.crashed.load(Ordering::Acquire) {
        return Err(EIO);
    }
    // Snapshot the registry and the client aliases; map them without the device lock.
    let (registry, aliases) = {
        let state = gpu.shared.state.lock();
        let firmware = state.as_ref().ok_or(ENODEV)?;
        (
            firmware.vms.clone(),
            ClientAlias::collect(&firmware.objects)?,
        )
    };
    registry.prune();
    let id = gpu.ids.vm.next();
    let vm = gpu.uat.new_vm(id, kernel_range)?.with_tracked_status()?;
    let status = vm.status().ok_or(EINVAL)?.clone();
    let (mappings, reserved) = Registry::driver_mappings(&vm, aliases)?;
    vm.install_driver_mappings(mappings, reserved)?;
    // No PB chunk yet: they are mapped before the VM's first prefix (`Firmware::map_pb`).
    let heap = Arc::pin_init(
        pin_init!(HeapMappings {
            list <- new_mutex!(KVec::new(), "g15::HeapMappings::list"),
        }),
        GFP_KERNEL,
    )?;
    let state = Arc::new(
        VmState {
            id,
            status,
            vm: vm.clone(),
            uat: gpu.uat.clone(),
            registry: registry.clone(),
            heap,
        },
        GFP_KERNEL,
    )?;
    registry.vms.lock().push(state, GFP_KERNEL)?;
    mod_pr_debug!("G15: VM {} created\n", id);
    Ok(vm)
}

/// A client's VM handle is gone (`gpu::Gpu::vm_closed`): drop the registry's references to
/// closed VMs now, outside the device lock, instead of at the next VM creation.
pub(crate) fn closed(gpu: &Gpu) {
    let registry = gpu
        .shared
        .state
        .lock()
        .as_ref()
        .map(|firmware| firmware.vms.clone());
    if let Some(registry) = registry {
        registry.prune();
    }
}

/// The state of a VM for a new queue.
pub(crate) fn lookup(gpu: &Gpu, vm: &mmu::Vm) -> Result<Arc<VmState>> {
    let registry = gpu.shared.state.lock().as_ref().ok_or(ENODEV)?.vms.clone();
    registry.find(vm)
}
