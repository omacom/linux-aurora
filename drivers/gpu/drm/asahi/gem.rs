// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Asahi driver GEM object implementation
//!
//! Basic wrappers and adaptations between generic GEM shmem objects and this driver's
//! view of what a GPU buffer object is. It is in charge of keeping track of all mappings for
//! each GEM object so we can remove them when a client (File) or a Vm are destroyed, as well as
//! implementing RTKit buffers on top of GEM objects for firmware use.

use kernel::{
    drm::{
        self,
        gem::{
            shmem::{
                self,
                VMapOwned, //
            },
            BaseObject,
            DriverObject,
            IntoGEMObject, //
        }, //
    },
    error::Result,
    mm::Mm,
    new_mutex,
    prelude::*,
    sync::aref::ARef,
    sync::Mutex,
    uapi, //
};

use core::ops::Range;
use core::sync::atomic::{
    AtomicBool,
    AtomicI64,
    AtomicU64,
    AtomicUsize,
    Ordering, //
};

use crate::{
    debug::*,
    driver::{
        AsahiDrmDevice,
        AsahiDrmDriver, //
    },
    file, memacct, mmu,
};

const DEBUG_CLASS: DebugFlags = DebugFlags::Gem;

/// Represents the inner data of a GEM object for this driver.
#[pin_data]
pub(crate) struct AsahiObject {
    /// ID for debug
    id: u64,
    /// Object size retained for live-object accounting.
    size: usize,
    /// Object creation flags.
    flags: u32,
    /// Whether this object can be exported.
    exportable: bool,
    /// Whether this is a kernel-created object.
    kernel: bool,
    /// Driver aliases outside the user GPUVA tree; they pin the backing.
    kernel_mappings: AtomicU64,
    /// Memory accounting and process attribution of the held backing.
    charge: memacct::Charge,
    /// Userspace asked to keep the backing resident (DRM_IOCTL_ASAHI_GEM_RESIDENCY): idle
    /// reclaim of the object's VM releases it after the VM's other objects, and only if those
    /// did not meet the scan target. Changed and read under the object's reservation.
    keep_resident: AtomicBool,
    /// The pinned pages of a user-memory object (DRM_IOCTL_ASAHI_GEM_USERPTR), its backing.
    /// The shmem layer unmaps them from the device before this is dropped.
    user_memory: Option<UserPages>,
    /// Serializes overlapping timestamp writes through every binding of this BO.
    #[pin]
    pub(crate) timestamp_lock: Mutex<()>,
}

/// Type alias for the shmem GEM object type for this driver.
pub(crate) type Object = shmem::Object<AsahiObject>;

/// Marks whether the pages of `gem` are held for the device.
///
/// Populated asahi objects keep their pages referenced (device SG table, kernel vmap or a PFN
/// CPU mapping), so the pages cannot migrate. Unmarked, the compaction migrate scanner still
/// isolates them from the unevictable LRU and every migration fails. The shmem mapping of a
/// held object is therefore marked inaccessible, which compaction and migration check and which
/// comes with unevictable, as `mapping_set_inaccessible()` sets it; nothing reads or writes a GEM
/// shmem file through its mapping. Releasing idle backing or purging clears the mark, so the
/// released pages can be swapped and migrated again.
///
/// The same transitions charge and uncharge the held pages (see [`memacct`]): a held user object
/// counts towards the resident set of the process that created it.
pub(crate) fn set_mapping_held(gem: &Object, held: bool) {
    let raw = gem.as_raw();
    // SAFETY: `gem` keeps the GEM object alive. A shmem-backed object's `filp` and its
    // `f_mapping` are set at initialization and never change; imported objects have none.
    let mapping = unsafe {
        let filp = (*raw).filp;
        if filp.is_null() {
            return;
        }
        (*filp).f_mapping
    };
    if mapping.is_null() {
        return;
    }
    // SAFETY: `flags` is the aligned flag word of the live mapping. The kernel updates it only
    // with atomic bit operations on the same word.
    let flags =
        unsafe { AtomicUsize::from_ptr(core::ptr::addr_of_mut!((*mapping).flags).cast::<usize>()) };
    let inaccessible = 1usize << kernel::bindings::mapping_flags_AS_INACCESSIBLE;
    let unevictable = 1usize << kernel::bindings::mapping_flags_AS_UNEVICTABLE;
    if held {
        flags.fetch_or(inaccessible | unevictable, Ordering::Relaxed);
    } else {
        flags.fetch_and(!inaccessible, Ordering::Relaxed);
    }
    gem.charge.set_held(held);
}

/// Takes the SG table of `gem`, which holds its pages for the device, and marks the
/// object held (see [`set_mapping_held`]).
///
/// For mappings the driver creates for itself or the firmware (kernel aliases, timestamp
/// buffers, firmware structures), which user-memory objects never back: those are refused.
pub(crate) fn held_sg_table<'a>(
    gem: &'a Object,
    dev: &'a kernel::device::Device<kernel::device::Bound>,
) -> Result<shmem::SGTable<'a, AsahiObject>> {
    if gem.user_memory.is_some() {
        return Err(EINVAL);
    }
    mapped_sg_table(gem, dev)
}

/// Takes the SG table of `gem` for a user VM binding with `prot`, as [`held_sg_table`].
///
/// A read-only user-memory object takes only GPU read-only bindings (`EACCES` otherwise): its
/// pages may be another file's page cache, which the caller may not write.
pub(crate) fn held_user_sg_table<'a>(
    gem: &'a Object,
    dev: &'a kernel::device::Device<kernel::device::Bound>,
    prot: mmu::Prot,
) -> Result<shmem::SGTable<'a, AsahiObject>> {
    if gem.user_memory.as_ref().is_some_and(|user| !user.writable)
        && prot != mmu::PROT_GPU_SHARED_RO
    {
        return Err(EACCES);
    }
    mapped_sg_table(gem, dev)
}

/// Takes the SG table of `gem` for a device mapping that [`held_sg_table`] or
/// [`held_user_sg_table`] already admitted (the caller holds a lease from one of them, or the
/// mapping exists).
pub(crate) fn mapped_sg_table<'a>(
    gem: &'a Object,
    dev: &'a kernel::device::Device<kernel::device::Bound>,
) -> Result<shmem::SGTable<'a, AsahiObject>> {
    let sgt = gem.sg_table(dev)?;
    set_mapping_held(gem, true);
    Ok(sgt)
}

/// VM-private BOs share their assigned VM's reservation object. Validate this
/// before acquiring mapping state; generic GPUVM accepts external shareable BOs.
pub(crate) fn validate_vm_binding(gem: &Object, vm: &mmu::Vm) -> Result {
    if gem.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0 && vm.is_extobj(gem) {
        return Err(EINVAL);
    }
    Ok(())
}

/// A shared reference to a GEM object for this driver.
pub(crate) struct ObjectRef {
    /// The underlying GEM object reference
    pub(crate) gem: ARef<Object>,
    /// The kernel-side VMap of this object, if needed
    // vmap: Option<VMapOwned::<u8, AsahiObject>>,
    vmap: Option<VMapOwned<AsahiObject, 0>>,
}

crate::no_debug!(ObjectRef);

static GEM_ID: AtomicU64 = AtomicU64::new(0);

impl ObjectRef {
    /// Create a new wrapper for a raw GEM object reference.
    pub(crate) fn new(gem: ARef<Object>) -> ObjectRef {
        ObjectRef { gem, vmap: None }
    }

    /// Return the `VMap` for this object, creating it if necessary.
    pub(crate) fn vmap(&mut self) -> Result<&shmem::VMapOwned<AsahiObject, 0>> {
        if self.vmap.is_none() {
            self.vmap = Some(self.gem.owned_vmap::<0>()?);
        }
        self.vmap.as_ref().ok_or(EINVAL)
    }

    /// Returns the size of an object in bytes
    pub(crate) fn size(&self) -> usize {
        self.gem.size()
    }

    /// Maps an object into a given `Vm` at any free address within a given range.
    pub(crate) fn map_into_range(
        &mut self,
        vm: &crate::mmu::Vm,
        range: Range<u64>,
        alignment: u64,
        prot: mmu::Prot,
        guard: bool,
    ) -> Result<crate::mmu::KernelMapping> {
        // Only used for kernel objects now
        if !self.gem.kernel {
            return Err(EINVAL);
        }
        vm.map_in_range(&self.gem, 0..self.gem.size(), alignment, range, prot, guard)
    }

    /// Maps a range within an object into a given `Vm` at any free address within a given range.
    pub(crate) fn map_range_into_range(
        &mut self,
        vm: &crate::mmu::Vm,
        obj_range: Range<usize>,
        range: Range<u64>,
        alignment: u64,
        prot: mmu::Prot,
        guard: bool,
    ) -> Result<crate::mmu::KernelMapping> {
        if obj_range.end > self.gem.size() {
            return Err(EINVAL);
        }
        validate_vm_binding(&self.gem, vm)?;
        vm.map_in_range(&self.gem, obj_range, alignment, range, prot, guard)
    }

    /// Maps an object into a given `Vm` at a specific address.
    ///
    /// Returns Err(ENOSPC) if the requested address is already busy.
    pub(crate) fn map_at(
        &mut self,
        vm: &crate::mmu::Vm,
        addr: u64,
        prot: mmu::Prot,
        guard: bool,
    ) -> Result<crate::mmu::KernelMapping> {
        validate_vm_binding(&self.gem, vm)?;

        vm.map_at(addr, self.gem.size(), &self.gem, prot, guard)
    }

    /// Map at a specific address, reserving `guard_pages` unmapped pages
    /// after the object in that Vm's allocator (T8152, module 1134).
    #[cfg(CONFIG_DRM_ASAHI_G18)]
    pub(crate) fn map_at_with_guards(
        &mut self,
        vm: &crate::mmu::Vm,
        addr: u64,
        prot: mmu::Prot,
        guard_pages: usize,
    ) -> Result<crate::mmu::KernelMapping> {
        validate_vm_binding(&self.gem, vm)?;

        vm.map_at_with_guards(addr, self.gem.size(), &self.gem, prot, guard_pages)
    }
}

pub(crate) struct AsahiObjConfig {
    flags: u32,
    exportable: bool,
    kernel: bool,
    charge: memacct::Charge,
    user_memory: Option<UserPages>,
}

/// Create a new kernel-owned GEM object.
pub(crate) fn new_kernel_object(dev: &AsahiDrmDevice, size: usize) -> Result<ObjectRef> {
    new_kernel_object_mapped(dev, size, false, true)
}

/// Create a new kernel-owned GEM object whose kernel mapping is write-combined.
///
/// Used for memory that a non-coherent agent reads as soon as the host has stored to it and
/// issued a barrier, without cache maintenance.
pub(crate) fn new_kernel_object_wc(dev: &AsahiDrmDevice, size: usize) -> Result<ObjectRef> {
    new_kernel_object_mapped(dev, size, true, true)
}

/// Create a new kernel-owned write-combined GEM object that is not counted as kernel memory:
/// its owner holds a user charge for it ([`memacct::Charge::user`]), and counting both would
/// take its size twice from the device limit.
pub(crate) fn new_kernel_object_wc_user_charged(
    dev: &AsahiDrmDevice,
    size: usize,
) -> Result<ObjectRef> {
    new_kernel_object_mapped(dev, size, true, false)
}

/// Validate before rounding: wrapping a user size can create a much smaller
/// backing than requested (or panic with overflow checks). Kernel callers use
/// the same boundary so malformed calculated sizes cannot bypass this check.
pub(crate) fn checked_object_size(size: usize) -> Result<usize> {
    if size == 0 {
        return Err(EINVAL);
    }
    size.checked_add(mmu::UAT_PGMSK)
        .map(|rounded| rounded & !mmu::UAT_PGMSK)
        .ok_or(EOVERFLOW)
}

fn new_kernel_object_mapped(
    dev: &AsahiDrmDevice,
    size: usize,
    map_wc: bool,
    kernel_charge: bool,
) -> Result<ObjectRef> {
    let size = checked_object_size(size)?;
    let charge = if kernel_charge {
        memacct::Charge::kernel(&memacct::LEDGER, size as u64)
    } else {
        memacct::Charge::none()
    };
    let gem = shmem::Object::<AsahiObject>::new(
        dev,
        size,
        shmem::ObjectConfig::<AsahiObject> {
            reclaimable_cpu_mappings: false,
            pages_bounded_reclaim: false,
            map_wc,
            parent_resv_obj: None,
        },
        AsahiObjConfig {
            flags: 0,
            exportable: false,
            kernel: true,
            charge,
            user_memory: None,
        },
    )?;

    set_mapping_held(&gem, true);
    mod_pr_debug!("AsahiObject new kernel object id={}\n", gem.id);
    Ok(ObjectRef::new(gem))
}

/// Create a new user-owned GEM object with the given flags.
///
/// `commit` is the admission of `size` (already rounded by [`checked_object_size`]) for the
/// creating client; the object is attributed to the current process.
pub(crate) fn new_object(
    dev: &AsahiDrmDevice,
    size: usize,
    flags: u32,
    parent_object: Option<&shmem::Object<AsahiObject>>,
    reclaim_cpu: bool,
    pages_bounded_reclaim: bool,
    commit: memacct::Commit,
) -> Result<ARef<Object>> {
    if (flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0) != parent_object.is_some()
    {
        return Err(EINVAL);
    }

    let gem = shmem::Object::<AsahiObject>::new(
        dev,
        checked_object_size(size)?,
        shmem::ObjectConfig::<AsahiObject> {
            reclaimable_cpu_mappings: parent_object.is_some() && reclaim_cpu,
            pages_bounded_reclaim,
            map_wc: flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_WRITEBACK == 0,
            parent_resv_obj: parent_object,
        },
        AsahiObjConfig {
            flags,
            exportable: parent_object.is_none(),
            kernel: false,
            charge: memacct::Charge::user_current(commit),
            user_memory: None,
        },
    )?;

    mod_pr_debug!("AsahiObject new user object: id={}\n", gem.id);
    Ok(gem)
}

/// Create a user-memory object over `pages`, the caller's memory pinned by [`UserPages::pin`].
///
/// `commit` is the admission of the pages' size for the creating client. The pages count as held
/// by the GPU for the object's whole life, but are not attributed to the process again: they are
/// already part of its memory (anonymous memory or page cache it maps). The object cannot be
/// exported, CPU-mapped through the driver, vmapped, purged or idle-reclaimed.
pub(crate) fn new_user_memory_object(
    dev: &AsahiDrmDevice,
    parent: &kernel::device::Device<kernel::device::Bound>,
    pages: UserPages,
    flags: u32,
    parent_object: Option<&shmem::Object<AsahiObject>>,
    commit: memacct::Commit,
) -> Result<ARef<Object>> {
    if (flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE != 0) != parent_object.is_some()
    {
        return Err(EINVAL);
    }
    let writable = pages.writable;
    // The page array is a heap allocation: moving `pages` into the object leaves it in place.
    let array = core::ptr::slice_from_raw_parts(pages.pages.as_ptr(), pages.pages.len());
    // SAFETY:
    // - `array` is the array of `pages`, which the object's data holds from its initialization
    //   on: the array stays allocated and its pages pinned until `pages` is dropped with that
    //   data.
    // - Coherent access. The object is only ever mapped by user VM binds
    //   (`held_user_sg_table()`; `held_sg_table()` refuses it for every mapping the driver makes
    //   for itself or for the firmware), and only on the GPUs of `File::supports_user_memory()`,
    //   where a user VM bind is a set of GPU-only entries (`mmu::PROT_GPU_SHARED_*`) of the VM's
    //   page tables at the DMA addresses of the pages. The GPU reads and writes the pages
    //   through those entries and in no other way. This driver depends on such accesses being
    //   coherent with the CPU caches for its ordinary objects already: an object created with
    //   DRM_ASAHI_GEM_WRITEBACK is mapped cacheable into its process, which stores to it and
    //   loads from it while the GPU uses it through entries of the same kind, the driver can do
    //   no cache maintenance for those accesses, and the UAPI has no call for it. Whether the
    //   device is marked DMA-coherent does not enter into it: where it is not, the DMA layer
    //   would clean and invalidate CPU caches when it maps and unmaps, which is what the
    //   object's mapping skips.
    let gem = unsafe {
        shmem::Object::<AsahiObject>::new_external(
            dev,
            parent,
            array,
            writable,
            shmem::ObjectConfig::<AsahiObject> {
                parent_resv_obj: parent_object,
                ..Default::default()
            },
            AsahiObjConfig {
                flags,
                exportable: false,
                kernel: false,
                charge: memacct::Charge::user(commit, None),
                user_memory: Some(pages),
            },
        )
    }?;
    gem.charge.set_held(true);
    mod_pr_debug!("AsahiObject new user-memory object: id={}\n", gem.id);
    Ok(gem)
}

/// Pages of the calling process pinned as the backing of a user-memory object, and counted in its
/// mm's `pinned_vm` against RLIMIT_MEMLOCK.
pub(crate) struct UserPages {
    pages: KVVec<*mut kernel::bindings::page>,
    writable: bool,
    /// The mm whose `pinned_vm` counts the pages; it may outlive the address space.
    mm: ARef<Mm>,
    /// Pages counted in `pinned_vm`: all of the range, including any not pinned yet.
    counted: i64,
    /// The fragment pins of a process with a smaller page size, which hold `pages` (null when
    /// `pages` are ordinary pins).
    fragments: *mut core::ffi::c_void,
}

// SAFETY: The pinned pages and fragment records may be released from any thread; the mm
// reference is thread-safe.
unsafe impl Send for UserPages {}
// SAFETY: Shared access only reads the array.
unsafe impl Sync for UserPages {}

/// Pages pinned per `pin_user_pages_fast()` call.
const PIN_CHUNK: usize = 16384;

impl UserPages {
    /// Pins the `size` bytes at `addr` in the calling process, for device writes as well if
    /// `writable`. Both must be multiples of the page size, and `size` non-zero.
    ///
    /// All of the range is pinned, long-term, or none of it. Fails with `EFAULT` when a page cannot
    /// be pinned: unmapped, a device or PFN mapping, or a writable pin of a mapping that is not
    /// writable or whose filesystem tracks writes (`FOLL_LONGTERM` refuses those). In a process
    /// whose page size is smaller than the native one, a native page is accepted only where the
    /// process maps all of it, in order at consecutive addresses (`EFAULT` otherwise). Fails with
    /// `ENOMEM` when the pages would take the process past RLIMIT_MEMLOCK without CAP_IPC_LOCK,
    /// counting every page pinned this way or by other long-term pinners.
    ///
    /// Anonymous pages pinned here stay with this process across fork(): the child gets copies.
    /// Pages unmapped by the process stay pinned, and backing the object, until it is freed.
    pub(crate) fn pin(addr: usize, size: usize, writable: bool) -> Result<Self> {
        let page_size = kernel::page::PAGE_SIZE;
        if size == 0 || (addr | size) & (page_size - 1) != 0 {
            return Err(EINVAL);
        }
        addr.checked_add(size).ok_or(EFAULT)?;
        let count = size / page_size;
        let mm: ARef<Mm> = kernel::current!()
            .mm()
            .map(|mm| ARef::from(&**mm))
            .ok_or(EFAULT)?;
        let pinned_vm = Self::pinned_vm(&mm);
        let added = i64::try_from(count).map_err(|_| ENOMEM)?;
        let total = pinned_vm
            .fetch_add(added, Ordering::AcqRel)
            .saturating_add(added);
        if !memlock_allows(total, memlock_limit_pages()) {
            // SAFETY: `capable()` has no preconditions.
            if !unsafe { kernel::bindings::capable(kernel::bindings::CAP_IPC_LOCK as i32) } {
                pinned_vm.fetch_sub(added, Ordering::AcqRel);
                return Err(ENOMEM);
            }
        }
        // From here on, dropping `this` unpins what it holds and uncounts all `count` pages.
        let mut this = Self {
            pages: KVVec::new(),
            writable,
            mm,
            counted: added,
            fragments: core::ptr::null_mut(),
        };
        this.pages.reserve(count, GFP_KERNEL)?;
        let mut flags = kernel::bindings::FOLL_LONGTERM;
        if writable {
            flags |= kernel::bindings::FOLL_WRITE;
        }
        // SAFETY: No preconditions; it reads the current mm.
        if unsafe { asahi_user_mm_is_subpage() } {
            // SAFETY: The array has room for `count` entries. On success the call stored `count`
            // pages, which the records it returned in `fragments` hold.
            let ret = unsafe {
                asahi_pin_user_fragments(
                    addr as _,
                    count as _,
                    flags,
                    this.pages.as_mut_ptr().cast(),
                    &mut this.fragments,
                )
            };
            if ret < 0 {
                return Err(Error::from_errno(ret as i32));
            }
            // SAFETY: See above.
            unsafe { this.pages.inc_len(count) };
            return Ok(this);
        }
        while this.pages.len() < count {
            let done = this.pages.len();
            let chunk = (count - done).min(PIN_CHUNK);
            // SAFETY: The array has room for `count` entries, of which `done` are in use; the
            // call fills at most `chunk` more.
            let ret = unsafe {
                kernel::bindings::pin_user_pages_fast(
                    (addr + done * page_size) as _,
                    chunk as _,
                    flags,
                    this.pages.as_mut_ptr().add(done),
                )
            };
            if ret <= 0 {
                return Err(if ret < 0 {
                    Error::from_errno(ret)
                } else {
                    EFAULT
                });
            }
            let ret = (ret as usize).min(chunk);
            // SAFETY: The call stored `ret` valid page pointers after the first `done`.
            unsafe { this.pages.inc_len(ret) };
        }
        Ok(this)
    }

    fn pinned_vm(mm: &Mm) -> &AtomicI64 {
        // SAFETY: `pinned_vm` is an aligned atomic64 of the live mm, which the kernel only changes
        // atomically.
        unsafe {
            AtomicI64::from_ptr(core::ptr::addr_of_mut!(
                (*mm.as_raw()).__bindgen_anon_1.pinned_vm.counter
            ))
        }
    }
}

impl Drop for UserPages {
    fn drop(&mut self) {
        if !self.fragments.is_null() {
            // SAFETY: The records hold every page of the array and are released only here.
            // Pages the device may have written are marked dirty.
            unsafe { asahi_unpin_user_fragments(self.fragments, self.writable) };
        } else if !self.pages.is_empty() {
            // SAFETY: The array holds pages pinned by `pin_user_pages_fast()` and not yet
            // unpinned. Pages the device may have written are marked dirty.
            unsafe {
                kernel::bindings::unpin_user_pages_dirty_lock(
                    self.pages.as_mut_ptr(),
                    self.pages.len() as _,
                    self.writable,
                )
            };
        }
        Self::pinned_vm(&self.mm).fetch_sub(self.counted, Ordering::AcqRel);
    }
}

/// RLIMIT_MEMLOCK of the calling process, in pages.
fn memlock_limit_pages() -> u64 {
    let task = kernel::current!().as_ptr();
    // SAFETY: The current task and its signal structure are live; the limit is read once, as
    // `rlimit()` does.
    let bytes = unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(
            (*(*task).signal).rlim[kernel::bindings::RLIMIT_MEMLOCK as usize].rlim_cur
        ))
    };
    (bytes >> kernel::page::PAGE_SHIFT) as u64
}

/// Whether `pinned` pages stay within a memlock limit of `limit` pages.
fn memlock_allows(pinned: i64, limit: u64) -> bool {
    u64::try_from(pinned).is_ok_and(|pinned| pinned <= limit)
}

#[vtable]
impl DriverObject for AsahiObject {
    type Driver = AsahiDrmDriver;
    type Args = AsahiObjConfig;

    const HAS_EXPORT: bool = true;

    /// Callback to create the inner data of a GEM object
    fn new(_dev: &AsahiDrmDevice, size: usize, args: Self::Args) -> impl PinInit<Self, Error> {
        let id = GEM_ID.fetch_add(1, Ordering::Relaxed);
        mod_pr_debug!("AsahiObject::new id={}\n", id);
        try_pin_init!(AsahiObject {
            id,
            size,
            flags: args.flags,
            exportable: args.exportable,
            kernel: args.kernel,
            kernel_mappings: AtomicU64::new(0),
            keep_resident: AtomicBool::new(false),
            charge: args.charge,
            user_memory: args.user_memory,
            timestamp_lock <- new_mutex!((), "Asahi timestamp stores"),
        })
    }

    /// Objects the DRM core allocates for us (dma-bufs imported from other devices and dumb
    /// buffers) are shared user objects, like a GEM_CREATE object without flags.
    ///
    /// Their memory is not counted: an import's backing belongs to, and is charged by, its
    /// exporter, and the driver offers no dumb buffers (it has no KMS).
    fn core_object_args(_dev: &AsahiDrmDevice, _size: usize) -> Result<Self::Args> {
        Ok(AsahiObjConfig {
            flags: 0,
            exportable: true,
            kernel: false,
            charge: memacct::Charge::none(),
            user_memory: None,
        })
    }

    /// Callback to drop all mappings for a GEM object owned by a given `File`
    fn close(obj: &<Self::Driver as drm::Driver>::Object, file: &drm::gem::DriverFile<Self>) {
        // fn close(obj: &Object, file: &DrmFile) {
        mod_pr_debug!("AsahiObject::close id={}\n", obj.id);
        if file::File::unbind_gem_object(file, obj).is_err() {
            pr_err!("AsahiObject::close: Failed to unbind GEM object\n");
        }
    }

    /// Optional handle for exporting a gem object.
    fn export(
        obj: &<Self::Driver as drm::Driver>::Object,
        flags: u32,
    ) -> Result<drm::gem::DmaBuf<Object>> {
        if !obj.exportable {
            return Err(EINVAL);
        }

        obj.prime_export(flags)
    }
}

/// A driver-created alias of a GEM object, possibly in a root the user GPUVA
/// tree does not describe. It keeps the object out of idle reclaim.
pub(crate) struct KernelMappingPin {
    object: ARef<Object>,
}

impl KernelMappingPin {
    pub(crate) fn new(object: &Object) -> Self {
        object.kernel_mappings.fetch_add(1, Ordering::AcqRel);
        Self {
            object: object.into(),
        }
    }
}

impl core::ops::Deref for KernelMappingPin {
    type Target = Object;
    fn deref(&self) -> &Object {
        &self.object
    }
}

impl Drop for KernelMappingPin {
    fn drop(&mut self) {
        self.object.kernel_mappings.fetch_sub(1, Ordering::Release);
    }
}

impl AsahiObject {
    /// Whether this is a kernel-created object.
    #[cfg(CONFIG_DRM_ASAHI_G18)]
    pub(crate) fn is_kernel(&self) -> bool {
        self.kernel
    }

    /// Whether CPU mappings use the cacheable WRITEBACK policy.
    pub(crate) fn is_writeback(&self) -> bool {
        self.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_WRITEBACK != 0
    }

    pub(crate) fn idle_reclaim_candidate(&self) -> bool {
        !self.kernel
            && !self.exportable
            && self.user_memory.is_none()
            && self.kernel_mappings.load(Ordering::Acquire) == 0
    }

    /// Whether userspace asked to keep this object's backing resident. Idle reclaim releases
    /// the backing of such an object after the VM's other objects, and only if those did not
    /// meet the scan target. Purge of contents userspace advised away does not look at this:
    /// the advice says the contents are not needed.
    ///
    /// The caller holds the object's reservation.
    pub(crate) fn keep_resident(&self) -> bool {
        self.keep_resident.load(Ordering::Relaxed)
    }

    /// Whether this object is backed by the creating process's own memory.
    pub(crate) fn is_user_memory(&self) -> bool {
        self.user_memory.is_some()
    }
}

/// Populates and pins a new user object through its DMA mapping, before any CPU view exists.
///
/// On a device that is not DMA-coherent, the DMA mapping cleans the cacheable kernel alias of the
/// freshly zeroed pages to the point of coherency, so that no dirty line can later overwrite data
/// written through a write-combined view. The pin keeps reclaim from creating a new cacheable
/// alias. Only used when the GPU asks for it ([`crate::gpu::Gpu::prepare_new_objects`]).
///
pub(crate) fn prepare_new_object(
    gem: &Object,
    dev: &kernel::device::Device<kernel::device::Bound>,
) -> Result {
    // The native backend requires a non-coherent DMA device. Use its DMA mapping
    // rather than interpreting DMA addresses as CPU physical addresses.
    let _sgt = held_sg_table(gem, dev)?;
    Ok(())
}

/// Purge advice for a VM-private, non-exportable object. Returns whether the
/// contents are retained (0 once purged).
pub(crate) fn madvise(gem: &Object, dontneed: bool) -> Result<u32> {
    if gem.flags & uapi::drm_asahi_gem_flags_DRM_ASAHI_GEM_VM_PRIVATE == 0
        || gem.exportable
        || gem.user_memory.is_some()
    {
        return Err(EINVAL);
    }
    Ok(u32::from(gem.madvise(i32::from(dontneed))))
}

/// Keep (or stop keeping) the backing of a user object resident: once this returns, idle reclaim
/// treats the object as kept (released after the VM's other objects, where those did not meet
/// the scan target) or as any other. This is an order among the objects idle reclaim may
/// release, not an exemption. Backing already released stays released until the VM's next job
/// or binding restores it. Kernel objects are refused.
pub(crate) fn set_keep_resident(gem: &Object, keep: bool) -> Result {
    if gem.kernel {
        return Err(EINVAL);
    }
    // SAFETY: A live object's reservation pointer is set at creation and never changes.
    let resv = unsafe { (*gem.as_raw()).resv };
    // SAFETY: The reservation of a live object. Idle reclaim tests the flag while it holds the
    // reservation of the object's VM, which a VM-private object shares, so after the unlock no
    // reclaim pass can still act on the old value; other objects are never reclaimed.
    unsafe { kernel::bindings::dma_resv_lock(resv, core::ptr::null_mut()) };
    gem.keep_resident.store(keep, Ordering::Relaxed);
    gem.charge.set_kept(keep);
    // SAFETY: Locked above.
    unsafe { kernel::bindings::dma_resv_unlock(resv) };
    Ok(())
}

/// Large user BOs may release clean page cache but never swap out the rest of the
/// system per folio; they get a quick ENOMEM when they cannot fit. Keep ordinary
/// resource allocations and kernel-owned backing on their existing policy. 256 MiB
/// also matches Mesa's maximum reusable BO-cache footprint.
pub(crate) fn user_pages_bounded_reclaim(size: usize, g17: bool) -> bool {
    g17 && size >= 256 * 1024 * 1024
}

#[kernel::macros::kunit_tests(asahi_gem_allocation_policy)]
mod allocation_policy_tests {
    use super::*;

    #[test]
    fn large_user_backing() {
        assert!(!user_pages_bounded_reclaim(256 * 1024 * 1024 - 1, true));
        assert!(user_pages_bounded_reclaim(256 * 1024 * 1024, true));
        assert!(user_pages_bounded_reclaim(3_257_113_344, true));
        assert!(!user_pages_bounded_reclaim(3_257_113_344, false));
    }
}

#[kernel::macros::kunit_tests(asahi_gem_user_memory)]
mod user_memory_tests {
    use super::*;
    use kernel::{
        bindings,
        page::PAGE_SIZE,
        uaccess::{UserPtr, UserSlice},
    };

    const PROT_READ: u64 = 1;
    const PROT_WRITE: u64 = 2;
    const MAP_PRIVATE: u64 = 2;
    const MAP_ANONYMOUS: u64 = 0x20;

    /// Maps `len` bytes of anonymous memory into the test's user address space.
    fn map(len: usize, prot: u64) -> Result<usize> {
        // SAFETY: Called on a KUnit test thread. KUnit attaches a user mm for the test and
        // removes the mapping with it when the test ends.
        let addr = unsafe {
            bindings::kunit_vm_mmap(
                bindings::kunit_get_current_test(),
                core::ptr::null_mut(),
                0,
                len as _,
                prot as _,
                (MAP_PRIVATE | MAP_ANONYMOUS) as _,
                0,
            )
        } as usize;
        if addr == 0 || addr > usize::MAX - 4095 {
            return Err(ENOMEM);
        }
        Ok(addr)
    }

    fn pinned_vm() -> Result<i64> {
        let mm: ARef<Mm> = kernel::current!()
            .mm()
            .map(|mm| ARef::from(&**mm))
            .ok_or(EINVAL)?;
        Ok(UserPages::pinned_vm(&mm).load(Ordering::Acquire))
    }

    fn read_pinned(pages: &UserPages, index: usize, offset: usize) -> Result<u32> {
        let page = *pages.pages.get(index).ok_or(EINVAL)?;
        // SAFETY: The page is pinned while `pages` lives; the word is inside it.
        Ok(unsafe {
            let p = bindings::kmap_local_page(page);
            let value = core::ptr::read_volatile(p.cast::<u8>().add(offset).cast::<u32>());
            bindings::kunmap_local(p);
            value
        })
    }

    #[test]
    fn memlock_limit_policy() {
        assert!(memlock_allows(0, 0));
        assert!(memlock_allows(512, 512));
        assert!(!memlock_allows(513, 512));
        assert!(!memlock_allows(-1, 512));
        assert!(memlock_allows(i64::MAX, u64::MAX));
    }

    #[test]
    fn pins_anonymous_memory() -> Result {
        let addr = map(4 * PAGE_SIZE, PROT_READ | PROT_WRITE)?;
        let before = pinned_vm()?;
        let pages = UserPages::pin(addr, 4 * PAGE_SIZE, true)?;
        assert_eq!(pages.pages.len(), 4);
        assert_eq!(pinned_vm()?, before + 4);
        // CPU writes through the caller's mapping land in the pinned pages.
        UserSlice::new(UserPtr::from_addr(addr + PAGE_SIZE + 8), 4)
            .writer()
            .write_slice(&0x1234_5678u32.to_ne_bytes())?;
        assert_eq!(read_pinned(&pages, 1, 8)?, 0x1234_5678);
        drop(pages);
        assert_eq!(pinned_vm()?, before);
        Ok(())
    }

    #[test]
    fn bad_ranges_pin_nothing() -> Result {
        let addr = map(3 * PAGE_SIZE, PROT_READ | PROT_WRITE)?;
        // SAFETY: Unmaps the last page of the test's own mapping.
        assert_eq!(
            unsafe { bindings::vm_munmap((addr + 2 * PAGE_SIZE) as _, PAGE_SIZE) },
            0
        );
        let before = pinned_vm()?;
        assert_eq!(
            UserPages::pin(addr + 1, PAGE_SIZE, true).err(),
            Some(EINVAL)
        );
        assert_eq!(UserPages::pin(addr, 0, true).err(), Some(EINVAL));
        assert_eq!(
            UserPages::pin(addr, PAGE_SIZE + 1, true).err(),
            Some(EINVAL)
        );
        // Two pages pin before the hole: those are unpinned again.
        assert_eq!(
            UserPages::pin(addr, 3 * PAGE_SIZE, true).err(),
            Some(EFAULT)
        );
        assert_eq!(
            UserPages::pin(addr + 2 * PAGE_SIZE, PAGE_SIZE, false).err(),
            Some(EFAULT)
        );
        let top = usize::MAX & !(PAGE_SIZE - 1);
        assert_eq!(
            UserPages::pin(top, 2 * PAGE_SIZE, false).err(),
            Some(EFAULT)
        );
        assert_eq!(pinned_vm()?, before);
        // What remains of the mapping still pins.
        drop(UserPages::pin(addr, 2 * PAGE_SIZE, true)?);
        assert_eq!(pinned_vm()?, before);
        Ok(())
    }

    #[test]
    fn read_only_mappings_pin_read_only() -> Result {
        let addr = map(2 * PAGE_SIZE, PROT_READ)?;
        let before = pinned_vm()?;
        assert_eq!(
            UserPages::pin(addr, 2 * PAGE_SIZE, true).err(),
            Some(EFAULT)
        );
        assert_eq!(pinned_vm()?, before);
        let pages = UserPages::pin(addr, 2 * PAGE_SIZE, false)?;
        assert_eq!(pages.pages.len(), 2);
        assert!(!pages.writable);
        assert_eq!(pinned_vm()?, before + 2);
        drop(pages);
        assert_eq!(pinned_vm()?, before);
        Ok(())
    }

    #[test]
    fn pages_outlive_the_mapping() -> Result {
        let addr = map(PAGE_SIZE, PROT_READ | PROT_WRITE)?;
        UserSlice::new(UserPtr::from_addr(addr + 16), 4)
            .writer()
            .write_slice(&0xa5a5_5a5au32.to_ne_bytes())?;
        let pages = UserPages::pin(addr, PAGE_SIZE, true)?;
        // SAFETY: Unmaps the test's own mapping; the pinned page stays allocated.
        assert_eq!(unsafe { bindings::vm_munmap(addr as _, PAGE_SIZE) }, 0);
        assert_eq!(read_pinned(&pages, 0, 16)?, 0xa5a5_5a5a);
        drop(pages);
        Ok(())
    }
}

/// Populates and pins a new user object through its DMA mapping, before any CPU view exists.
///
/// On a device that is not DMA-coherent, the DMA mapping cleans the cacheable kernel alias of the
/// freshly zeroed pages to the point of coherency, so that no dirty line can later overwrite data
/// written through a write-combined view. The pin keeps reclaim from creating a new cacheable
/// alias. Only used when the GPU asks for it ([`crate::gpu::Gpu::prepare_new_objects`]).
///
/// The DMA mapping only cleans if the device is not marked DMA-coherent (G15's synthesized node
/// has no `dma-coherent`). Should it ever be marked so, the pages are cleaned here instead.
#[cfg(CONFIG_DRM_ASAHI_G15)]
pub(crate) fn prepare_new_object_g15(gem: &Object) -> Result {
    // SAFETY: G15 objects belong to the retained bound platform device; user object
    // creation occurs only after registration and before a file can outlive its device.
    let dev: &kernel::device::Device = gem.dev().as_ref().as_ref();
    let sgt = held_sg_table(gem, unsafe { dev.as_bound() })?;
    // SAFETY: The device outlives its GEM objects; the flag is fixed when the device is probed.
    if !unsafe { asahi_g15_dma_coherent(dev.as_raw().cast()) } {
        return Ok(());
    }
    static WARNED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if !WARNED.swap(true, Ordering::Relaxed) {
        dev_warn!(
            dev,
            "GPU marked DMA-coherent: cleaning new objects to the point of coherency explicitly\n"
        );
    }
    let page_size = kernel::page::PAGE_SIZE;
    for segment in sgt.iter() {
        // Without an IOMMU in front of the GPU, its DMA addresses are physical addresses.
        let start = segment.dma_address() as kernel::addr::PhysicalAddr;
        let len = segment.dma_len() as usize;
        let mut offset = 0;
        while offset < len {
            let phys = start + offset as kernel::addr::PhysicalAddr;
            // SAFETY: The pinned SG table keeps these shmem pages allocated.
            let page = unsafe { kernel::page::Page::borrow_phys(&phys) }.ok_or(EIO)?;
            page.with_pointer_into_page(0, page_size, |p| {
                crate::pgtable::clean_poc(p, page_size);
                Ok(())
            })?;
            offset += page_size;
        }
    }
    crate::pgtable::poc_barrier();
    Ok(())
}

// user_pages.c: long-term pins in a process whose page size is smaller than the native one.
extern "C" {
    fn asahi_user_mm_is_subpage() -> bool;
    fn asahi_pin_user_fragments(
        start: core::ffi::c_ulong,
        nr_pages: core::ffi::c_ulong,
        gup_flags: core::ffi::c_uint,
        pages: *mut *mut core::ffi::c_void,
        out: *mut *mut core::ffi::c_void,
    ) -> core::ffi::c_long;
    fn asahi_unpin_user_fragments(frags: *mut core::ffi::c_void, dirty: bool);
}

#[cfg(CONFIG_DRM_ASAHI_G15)]
extern "C" {
    fn asahi_g15_dma_coherent(dev: *mut core::ffi::c_void) -> bool;
}
