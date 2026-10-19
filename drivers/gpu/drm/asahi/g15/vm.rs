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
//!   (`vm.install_driver_mappings`): user binds there fail (EINVAL);
//! - bind it to an exclusive TTBAT slot ([`mmu::Uat::bind`]) for as long as its [`VmState`]
//!   lives; at most [`super::hw::sizes::MAX_VMS`] such VMs (ENOSPC beyond, checked before
//!   binding so the slot allocator never blocks).
//!
//! The [`VmState`] of a VM is shared by the [`Registry`] (while the client's VM handle exists),
//! the VM's queues and its jobs. It cannot be owned by the [`mmu::Vm`] itself: the slot binding
//! holds a `Vm` reference. The registry drops its reference once the shared lifetime reports the
//! client's handle closed ([`mmu::Vm::is_closed`]), at the next VM creation; the slot returns when
//! the last queue or job of the VM is gone (failed jobs are retained until reboot, and their
//! VM's slot with them).
//!
//! User page tables are cleaned to the point of coherency by the shared `pgtable.rs` (enabled by
//! the adopted UAT); user objects are prepared at creation (`gpu::Gpu::prepare_new_objects`).
//! User leaves use the shared GPU-shared protections (AttrIndex 2) for NC and WRITEBACK objects
//! alike.

use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};
use kernel::{
    drm::gem::BaseObject,
    new_mutex,
    prelude::*,
    sync::{aref::ARef, Arc, Mutex},
};

use super::{hw, object, Gpu};
use crate::{debug::*, g17::status::VmStatus, gem, mmu, module_parameters};

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

/// G15 state of one user VM, shared by the registry, its queues and its jobs.
pub(crate) struct VmState {
    /// The `mmu::Vm` identifier (`Gpu::ids.vm`).
    pub(crate) id: u64,
    /// The VM's TTBAT slot (ASID), for the work images' context.
    pub(crate) slot: u32,
    /// The sticky status (shared with `file.rs`).
    pub(crate) status: Arc<VmStatus>,
    /// The VM: span checks, stream reads and job guards.
    pub(crate) vm: mmu::Vm,
    /// The exclusive slot binding, released before the live count drops.
    bind: Option<mmu::VmBind>,
    registry: Arc<Registry>,
}

impl VmState {
    /// Drops the registry's references to closed VMs (the queues call this when destroyed).
    pub(crate) fn prune_registry(&self) {
        self.registry.prune();
    }
}

impl Drop for VmState {
    fn drop(&mut self) {
        mod_pr_debug!("G15: VM {} releases context {}\n", self.id, self.slot);
        drop(self.bind.take());
        self.registry.live.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Live-VM accounting and the VMs whose client handle is open.
#[pin_data]
pub(crate) struct Registry {
    /// VMs whose client handle may still be open, oldest first.
    #[pin]
    vms: Mutex<KVec<Arc<VmState>>>,
    /// Live [`VmState`]s (each holds a TTBAT slot) plus reservations being created.
    live: AtomicU32,
}

/// A reserved place among the live VMs, released unless a [`VmState`] takes it over.
struct Reservation<'a>(Option<&'a Registry>);

impl Reservation<'_> {
    fn into_state(mut self) {
        self.0 = None;
    }
}

impl Drop for Reservation<'_> {
    fn drop(&mut self) {
        if let Some(registry) = self.0 {
            registry.live.fetch_sub(1, Ordering::AcqRel);
        }
    }
}

impl Registry {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                vms <- new_mutex!(KVec::new(), "g15::Registry::vms"),
                live: AtomicU32::new(0),
            }),
            GFP_KERNEL,
        )
    }

    /// Reserves a place for a new VM after dropping the registry's references to closed VMs.
    fn reserve(&self) -> Result<Reservation<'_>> {
        self.prune();
        // The shared slot allocator hands out a single slot with robust isolation.
        let limit = if *module_parameters::robust_isolation.value() != 0 {
            1
        } else {
            hw::sizes::MAX_VMS
        };
        self.live
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |live| {
                (live < limit).then_some(live + 1)
            })
            .map_err(|_| ENOSPC)?;
        Ok(Reservation(Some(self)))
    }

    /// Drops the registry's references to VMs whose client handle is closed. Called on VM close
    /// ([`closed`]), on queue destruction and before a new VM is created: a closed VM's page
    /// tables, bound objects and TTBAT slot go as soon as its last queue or job is gone.
    pub(crate) fn prune(&self) {
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
                    alias.gem,
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
    let reservation = registry.reserve()?;
    let id = gpu.ids.vm.next();
    let vm = gpu.uat.new_vm(id, kernel_range)?.with_tracked_status()?;
    let status = vm.status().ok_or(EINVAL)?.clone();
    let (mappings, reserved) = Registry::driver_mappings(&vm, aliases)?;
    vm.install_driver_mappings(mappings, reserved)?;
    // The context names the root only now: everything above is already visible to the walker.
    let bind = gpu.uat.bind(&vm)?;
    let slot = bind.slot();
    // From here on the state (even one that fails to allocate) gives the place back.
    reservation.into_state();
    let state = Arc::new(
        VmState {
            id,
            slot,
            status,
            vm: vm.clone(),
            bind: Some(bind),
            registry: registry.clone(),
        },
        GFP_KERNEL,
    )?;
    registry.vms.lock().push(state, GFP_KERNEL)?;
    mod_pr_debug!("G15: VM {} context {}\n", id, slot);
    Ok(vm)
}

/// A client's VM handle is gone (`DrmGpu::vm_closed`): drop the registry's references to closed
/// VMs now, outside the device lock, instead of at the next VM creation.
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
