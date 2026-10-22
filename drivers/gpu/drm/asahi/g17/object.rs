// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Kernel objects shared with the G17 firmware.
//!
//! Each firmware object is backed by its own GEM object, mapped into a kernel address space
//! either at an address the firmware interface fixes or anywhere in a given range, and kept
//! mapped into the kernel CPU address space for its whole lifetime. The host builds an object's
//! contents through [`KernelObject::bytes_mut`] and [`KernelObject::write`] before it publishes
//! its address; after that, words the firmware accesses concurrently are only touched through
//! the single-access helpers.

use core::ops::Range;
use core::ptr::NonNull;
use core::sync::atomic::{
    AtomicU32,
    AtomicU64, //
};

use kernel::{
    drm::gem::shmem,
    prelude::*, //
};

use crate::driver::AsahiDrmDevice;
use crate::{
    gem,
    mmu, //
};

/// CPU caching of the kernel mapping of a [`KernelObject`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum CpuMap {
    /// Write-back, for memory the firmware only reads through a cached mapping or that the host
    /// only reads.
    WriteBack,
    /// Write-combined, for memory the firmware reads uncached as soon as the host published it.
    WriteCombined,
}

/// Where a [`KernelObject`] is mapped.
pub(crate) enum Placement {
    /// At this address, which the firmware interface fixes.
    At(u64),
    /// Anywhere in this range, at this alignment, followed by an unmapped guard page.
    In(Range<u64>, u64),
}

/// A zero-initialized kernel object mapped into a GPU address space and the kernel.
///
/// # Invariants
///
/// `ptr` points to the start of the kernel mapping of `obj`, which is `size` bytes long and stays
/// valid as long as `obj` exists, since `obj` holds an owned kernel mapping.
pub(crate) struct KernelObject {
    // Unmap from the GPU before dropping the backing object.
    mapping: Option<mmu::KernelMapping>,
    obj: gem::ObjectRef,
    ptr: NonNull<u8>,
    size: usize,
}

// SAFETY: The object only refers to kernel memory; concurrent access goes through `&self`
// methods that use atomic or volatile accesses.
unsafe impl Send for KernelObject {}
// SAFETY: See `Send`.
unsafe impl Sync for KernelObject {}

impl KernelObject {
    pub(crate) fn classify_memory(&self, category: crate::agx_memory_stats::KernelCategory) {
        self.obj.gem.classify_memory(category);
    }
    /// Allocates a zeroed object of `size` bytes and maps it into `vm`.
    pub(crate) fn new(
        dev: &AsahiDrmDevice,
        vm: &mmu::Vm,
        placement: Placement,
        size: usize,
        prot: mmu::Prot,
        cpu: CpuMap,
    ) -> Result<Self> {
        let mut this = Self::backing(dev, size, cpu)?;
        let mapping = match placement {
            Placement::At(addr) => this.obj.map_at(vm, addr, prot, false)?,
            Placement::In(range, align) => this.obj.map_into_range(vm, range, align, prot, true)?,
        };
        this.mapping = Some(mapping);
        Ok(this)
    }

    /// Allocates CPU-accessible backing whose GPU mappings will be installed later.
    pub(crate) fn backing(dev: &AsahiDrmDevice, size: usize, cpu: CpuMap) -> Result<Self> {
        let mut obj = match cpu {
            CpuMap::WriteBack => gem::new_kernel_object(dev, size)?,
            CpuMap::WriteCombined => gem::new_kernel_object_wc(dev, size)?,
        };
        let vmap = obj.vmap()?;
        let ptr = NonNull::new(vmap.addr.cast::<u8>()).ok_or(ENOMEM)?;
        // SAFETY: The mapping covers the whole object.
        unsafe { ptr.as_ptr().write_bytes(0, kernel::drm::gem::BaseObject::size(vmap.owner())) };

        // INVARIANT: `vmap()` created the owned kernel mapping of `obj` that `ptr` points into,
        // and `obj` keeps it until it is dropped.
        Ok(Self {
            mapping: None,
            obj,
            ptr,
            size,
        })
    }

    /// Maps this object into another address space at `addr`, as an alias of the same memory.
    pub(crate) fn map_alias(
        &self,
        vm: &mmu::Vm,
        addr: u64,
        prot: mmu::Prot,
    ) -> Result<mmu::KernelMapping> {
        gem::ObjectRef::new(self.obj.gem.clone()).map_at(vm, addr, prot, false)
    }

    /// Reuses only a complete alias of this exact retained backing. Permanent
    /// queue aliases stay owned by the client VM after a physical pair moves;
    /// returning to that VM must not replace or duplicate those mappings.
    pub(crate) fn prepare_alias(
        &self,
        vm: &mmu::Vm,
        address: u64,
        prot: mmu::Prot,
        requires_write: bool,
    ) -> Result<Option<mmu::KernelMapping>> {
        if self.size == 0 || self.size % mmu::UAT_PGSZ != 0 || address & mmu::UAT_PGMSK as u64 != 0
        {
            return Err(EINVAL);
        }
        address.checked_add(self.size as u64).ok_or(EOVERFLOW)?;
        let source = self.mapping.as_ref().ok_or(EINVAL)?;
        let mut existing = 0;
        for offset in (0..self.size).step_by(mmu::UAT_PGSZ) {
            let client = address + offset as u64;
            if vm.covers_range(client, mmu::UAT_PGSZ as u64, true, requires_write) {
                if vm.translate_iova(client)? != source.translate_offset(offset)? {
                    return Err(EFAULT);
                }
                existing += 1;
            }
        }
        if existing != 0 {
            if existing != self.size / mmu::UAT_PGSZ {
                return Err(EFAULT);
            }
            return Ok(None);
        }
        self.map_alias(vm, address, prot).map(Some)
    }

    /// Maps this backing into a dynamically placed kernel alias.
    pub(crate) fn alias_in(
        &self,
        vm: &mmu::Vm,
        range: Range<u64>,
        align: u64,
        prot: mmu::Prot,
    ) -> Result<mmu::KernelMapping> {
        gem::ObjectRef::new(self.obj.gem.clone()).map_into_range(vm, range, align, prot, false)
    }

    fn alias_in_lowest(
        &self,
        vm: &mmu::Vm,
        range: Range<u64>,
        align: u64,
        prot: mmu::Prot,
        guard_size: usize,
    ) -> Result<mmu::KernelMapping> {
        vm.map_in_range_lowest(&self.obj.gem, 0..self.size, align, range, prot, guard_size)
    }

    /// Maps an exact backing subrange, retaining an explicit unmapped guard.
    pub(crate) fn map_range(
        &self,
        vm: &mmu::Vm,
        source: Range<usize>,
        range: Range<u64>,
        align: u64,
        prot: mmu::Prot,
        guard_size: usize,
    ) -> Result<mmu::KernelMapping> {
        vm.map_in_range_with_guard_size(&self.obj.gem, source, align, range, prot, guard_size)
    }

    /// Initializes a typed view before publishing the object to the firmware.
    ///
    /// The view is cleared first, making every `Zeroable` representation valid.
    pub(crate) fn initialize<T: Zeroable>(
        &mut self,
        offset: usize,
        init: impl FnOnce(&mut T) -> Result,
    ) -> Result {
        let ptr = self.aligned::<T>(offset)?;
        // SAFETY: The checked view lies within the exclusively borrowed object. Zeroable makes
        // the cleared bytes a valid T, and the closure cannot retain the reference.
        unsafe {
            ptr.write_bytes(0, 1);
            init(&mut *ptr)
        }
    }

    /// Returns a reference-counted kernel mapping of the object.
    pub(crate) fn kernel_map(&self) -> Result<shmem::VMapOwned<gem::AsahiObject>> {
        self.obj.gem.owned_vmap()
    }

    /// Transfers an unpublished object's GPU mapping to a longer-lived owner.
    /// The CPU view remains valid; gpu_va returns zero after this transfer.
    pub(crate) fn take_mapping(&mut self) -> Result<mmu::KernelMapping> {
        self.mapping.take().ok_or(EINVAL)
    }

    /// Returns the GPU address of the object.
    pub(crate) fn gpu_va(&self) -> u64 {
        self.mapping.as_ref().map_or(0, |mapping| mapping.iova())
    }

    /// Returns the size of the object in bytes.
    pub(crate) fn size(&self) -> usize {
        self.size
    }

    /// Returns the contents of the object.
    ///
    /// Only for building an object the firmware does not access concurrently, before its address
    /// is published or while the firmware is known not to use it.
    pub(crate) fn bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: Per the type invariant `ptr` is valid for `size` bytes, and the exclusive borrow
        // excludes other host accesses.
        unsafe { core::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.size) }
    }

    /// Writes `value` at `offset`, with the same restrictions as [`Self::bytes_mut`].
    pub(crate) fn write<T: Copy>(&mut self, offset: usize, value: T) -> Result {
        let end = offset
            .checked_add(core::mem::size_of::<T>())
            .ok_or(EINVAL)?;
        if end > self.size {
            return Err(EINVAL);
        }
        // SAFETY: The range was checked to be within the object, and the exclusive borrow excludes
        // other host accesses. The destination may be unaligned.
        unsafe {
            self.ptr
                .as_ptr()
                .add(offset)
                .cast::<T>()
                .write_unaligned(value)
        };
        Ok(())
    }

    /// Returns a pointer to the naturally aligned `T` at `offset`.
    fn aligned<T>(&self, offset: usize) -> Result<*mut T> {
        let end = offset
            .checked_add(core::mem::size_of::<T>())
            .ok_or(EINVAL)?;
        if end > self.size {
            return Err(EINVAL);
        }
        // SAFETY: The checked offset lies within the retained CPU mapping.
        let address = unsafe { self.ptr.as_ptr().add(offset) };
        if address as usize % core::mem::align_of::<T>() != 0 {
            return Err(EINVAL);
        }
        // The actual address, not only the offset, satisfies T's alignment.
        Ok(address.cast())
    }

    /// Returns a checked raw view; dereferencing it requires external serialization.
    pub(super) fn pointer(&self, offset: usize, length: usize) -> Result<*mut u8> {
        if offset.checked_add(length).ok_or(EINVAL)? > self.size {
            return Err(EINVAL);
        }
        // SAFETY: The offset was checked against the live object mapping.
        Ok(unsafe { self.ptr.as_ptr().add(offset) })
    }

    /// Returns the live 32-bit word at `offset`, which must be naturally aligned.
    pub(crate) fn word(&self, offset: usize) -> Result<&AtomicU32> {
        // SAFETY: `aligned()` returns a valid, aligned pointer into the object, which lives as long
        // as `self`. Other agents only access the word with single 32-bit accesses.
        Ok(unsafe { AtomicU32::from_ptr(self.aligned(offset)?) })
    }

    /// Returns the live 64-bit word at `offset`, which must be naturally aligned.
    pub(crate) fn dword(&self, offset: usize) -> Result<&AtomicU64> {
        // SAFETY: As for `word()`, with 64-bit accesses.
        Ok(unsafe { AtomicU64::from_ptr(self.aligned(offset)?) })
    }

    /// Copies `N` 32-bit words that start at `offset` out of the object, one volatile access each.
    ///
    /// Used for records another agent has published, which it no longer modifies.
    pub(crate) fn read_words<const N: usize>(&self, offset: usize) -> Result<[u32; N]> {
        let base = self.aligned::<[u32; N]>(offset)?.cast::<u32>();
        let mut words = [0u32; N];
        for (i, word) in words.iter_mut().enumerate() {
            // SAFETY: `aligned()` checked that all `N` words are within the object and aligned.
            *word = unsafe { base.add(i).read_volatile() };
        }
        Ok(words)
    }
}

/// Allocation domains shared by the G17 resource builders. Dynamic objects
/// retain a guard page; aliases keep the existing backing and its exact VA.
pub(crate) struct Allocator<'a> {
    pub(crate) dev: &'a AsahiDrmDevice,
    pub(crate) uat: &'a mmu::Uat,
}

impl Allocator<'_> {
    /// Allocates in the dynamic part of the firmware's upper address space.
    pub(crate) fn kernel(
        &self,
        size: usize,
        align: u64,
        prot: mmu::Prot,
        cpu: CpuMap,
    ) -> Result<KernelObject> {
        let mut range = self.uat.geometry().kernel_range();
        range.start += crate::hw::t8140::dynamic::KERNEL_OFFSET;
        KernelObject::new(
            self.dev,
            self.uat.kernel_vm(),
            Placement::In(range, align),
            size,
            prot,
            cpu,
        )
        .map_err(|error| {
            if error == ENOSPC {
                resource_exhausted!("firmware GPU VA")
            } else {
                error
            }
        })
    }

    /// Allocates a canonical low mapping, which can be aliased at the same
    /// address into an execution context without overlapping client objects.
    pub(crate) fn lower(
        &self,
        size: usize,
        align: u64,
        prot: mmu::Prot,
        cpu: CpuMap,
    ) -> Result<KernelObject> {
        KernelObject::new(
            self.dev,
            self.uat.kernel_lower_vm(),
            Placement::In(crate::hw::t8140::dynamic::LOWER, align),
            size,
            prot,
            cpu,
        )
        .map_err(|error| {
            if error == ENOSPC {
                resource_exhausted!("shared lower GPU VA")
            } else {
                error
            }
        })
    }

    /// Place fresh backing at one address in the canonical and both alias VMs.
    /// A client may retain mappings after their canonical reservation is freed.
    /// Retry only address collisions, keeping the same backing throughout.
    pub(crate) fn lower_with_aliases(
        &self,
        size: usize,
        align: u64,
        prot: mmu::Prot,
        cpu: CpuMap,
        aliases: [&mmu::Vm; 2],
    ) -> Result<(KernelObject, [mmu::KernelMapping; 2])> {
        if size == 0 || size % mmu::UAT_PGSZ != 0 {
            return Err(EINVAL);
        }
        let mut object = KernelObject::backing(self.dev, size, cpu)?;
        let mut range = crate::hw::t8140::dynamic::LOWER;
        let mut searching = false;
        loop {
            // Keep ordinary best-fit placement on the uncontended path. A
            // collision starts a lowest-address search over the whole arena.
            let mapping = if searching {
                object.alias_in_lowest(
                    self.uat.kernel_lower_vm(), range.clone(), align, prot, mmu::UAT_PGSZ,
                )?
            } else {
                object.obj.map_into_range(
                    self.uat.kernel_lower_vm(), range.clone(), align, prot, true,
                )?
            };
            let address = mapping.iova();
            let end = address.checked_add(size as u64).ok_or(EOVERFLOW)?;
            // Partial aliases are destroyed before releasing the canonical
            // reservation, so a retry cannot expose its address prematurely.
            let mapped: core::result::Result<_, (Error, usize)> = (|| {
                let first = object.map_alias(aliases[0], address, prot)
                    .map_err(|error| (error, 0))?;
                let second = object.map_alias(aliases[1], address, prot)
                    .map_err(|error| (error, 1))?;
                Ok([first, second])
            })();
            match mapped {
                Ok(mapped) => {
                    object.mapping = Some(mapping);
                    return Ok((object, mapped));
                }
                Err((error, index)) if error == ENOSPC && end < range.end => {
                    drop(mapping);
                    let start = if searching {
                        address + mmu::UAT_PGSZ as u64
                    } else {
                        range.start
                    };
                    // Jump to an actual hole in the VM that refused the alias,
                    // rather than remapping backing at every intervening page.
                    let candidate = object.alias_in_lowest(
                        aliases[index], start..range.end, align, prot, 0,
                    )?;
                    range.start = candidate.iova();
                    drop(candidate);
                    searching = true;
                }
                Err((error, _)) => return Err(error),
            }
        }
    }
}

/// Families whose upper mappings remain cached and installed until firmware
/// stops, even after the logical owner releases the object.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Family {
    Descriptor = 0,
    WorkState = 1,
    QueueState = 2,
    QueueConfig = 3,
}

const POOL_FAMILIES: usize = 4;
const POOL_SLOTS: usize = 64;
const POOL_BYTES: usize = 128 * 1024 * 1024;

struct Retained {
    object: KernelObject,
    prefix: usize,
    cpu: CpuMap,
    poisoned: bool,
}

struct PoolInner {
    returned: KVec<Option<Retained>>,
    reserved: [usize; POOL_FAMILIES],
    bytes: usize,
    stopped: bool,
}

/// Per-device cache-safe object recycling. Every converted mapping reserves
/// one return slot before its page-table attributes change. Active leases and
/// poisoned mappings count against the same capacity as reusable objects.
#[pin_data]
pub(crate) struct Pool {
    #[pin]
    inner: kernel::sync::Mutex<PoolInner>,
}

struct CacheLease {
    pool: kernel::sync::Arc<Pool>,
    family: Family,
    prefix: usize,
    cpu: CpuMap,
    poisoned: bool,
}

/// A firmware object whose cached mapping is returned to its device pool
/// instead of being unmapped while either firmware processor can reference it.
pub(crate) struct PooledObject {
    object: core::mem::ManuallyDrop<KernelObject>,
    lease: Option<CacheLease>,
}

impl core::ops::Deref for PooledObject {
    type Target = KernelObject;
    fn deref(&self) -> &Self::Target {
        &self.object
    }
}
impl core::ops::DerefMut for PooledObject {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.object
    }
}
impl Drop for PooledObject {
    fn drop(&mut self) {
        // SAFETY: This destructor is the only operation that moves `object`
        // out; ManuallyDrop prevents a second implicit drop of the field.
        let object = unsafe { core::mem::ManuallyDrop::take(&mut self.object) };
        match self.lease.take() {
            Some(lease) => lease.pool.return_object(object, &lease),
            None => drop(object),
        }
    }
}

impl Pool {
    pub(crate) fn new() -> Result<kernel::sync::Arc<Self>> {
        let mut returned = KVec::with_capacity(POOL_FAMILIES * POOL_SLOTS, GFP_KERNEL)?;
        for _ in 0..POOL_FAMILIES * POOL_SLOTS {
            returned.push(None, GFP_KERNEL)?;
        }
        kernel::sync::Arc::pin_init(
            pin_init!(Self {
                inner <- kernel::new_mutex!(PoolInner { returned, reserved: [0; POOL_FAMILIES], bytes: 0, stopped: false }, "g17::object::Pool"),
            }),
            GFP_KERNEL,
        )
    }

    /// Allocates or reuses a matching object. Capacity pressure leaves a fresh
    /// mapping uncached, preserving allocation and submission concurrency.
    pub(crate) fn allocate(
        self: &kernel::sync::Arc<Self>,
        alloc: &Allocator<'_>,
        family: Family,
        size: usize,
        prefix: usize,
        prot: mmu::Prot,
        cpu: CpuMap,
    ) -> Result<PooledObject> {
        let family_index = family as usize;
        let begin = family_index * POOL_SLOTS;
        let reused = {
            let mut inner = self.inner.lock();
            if inner.stopped {
                return Err(ENODEV);
            }
            let slot = inner.returned[begin..begin + POOL_SLOTS]
                .iter_mut()
                .find(|entry| {
                    entry.as_ref().is_some_and(|entry| {
                        !entry.poisoned
                            && entry.object.size() == size
                            && entry.prefix == prefix
                            && entry.cpu == cpu
                    })
                });
            slot.and_then(Option::take)
        };
        let lease = || CacheLease {
            pool: self.clone(),
            family,
            prefix,
            cpu,
            poisoned: false,
        };
        if let Some(mut entry) = reused {
            entry.object.bytes_mut().fill(0);
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            return Ok(PooledObject {
                object: core::mem::ManuallyDrop::new(entry.object),
                lease: Some(lease()),
            });
        }
        let object = alloc.kernel(size, mmu::UAT_PGSZ as u64, prot, cpu)?;
        use crate::agx_memory_stats::KernelCategory;
        object.classify_memory(match family {
            Family::Descriptor => KernelCategory::Descriptor,
            Family::WorkState => KernelCategory::WorkState,
            Family::QueueState => KernelCategory::QueueState,
            Family::QueueConfig => KernelCategory::QueueConfig,
        });
        let reserved = {
            let mut inner = self.inner.lock();
            if inner.stopped {
                return Err(ENODEV);
            }
            if inner.reserved[family_index] == POOL_SLOTS
                || inner
                    .bytes
                    .checked_add(size)
                    .is_none_or(|bytes| bytes > POOL_BYTES)
            {
                false
            } else {
                inner.reserved[family_index] += 1;
                inner.bytes += size;
                true
            }
        };
        let mut result = PooledObject {
            object: core::mem::ManuallyDrop::new(object),
            lease: reserved.then(lease),
        };
        if !reserved {
            return Ok(result);
        }
        let (proof, mutated) = match result.mapping.as_ref() {
            // SAFETY: The allocator just created the private mapping. No
            // pointer or alias has been published; its reserved lease retains
            // it until processor stop if any attribute change was attempted.
            Some(mapping) => unsafe { mapping.set_fw_cached_prefix(prefix) },
            None => (Err(EINVAL), false),
        };
        match proof {
            Ok(_) => Ok(result),
            Err(error) if mutated => {
                if let Some(lease) = &mut result.lease {
                    lease.poisoned = true;
                }
                // Drop retains the unproven mapping but never makes it reusable.
                Err(error)
            }
            Err(_) => {
                result.lease = None;
                let mut inner = self.inner.lock();
                inner.reserved[family_index] -= 1;
                inner.bytes -= size;
                Ok(result)
            }
        }
    }

    fn return_object(&self, object: KernelObject, lease: &CacheLease) {
        let mut inner = self.inner.lock();
        let family = lease.family as usize;
        if inner.stopped {
            inner.reserved[family] -= 1;
            inner.bytes -= object.size();
            drop(inner);
            drop(object);
            return;
        }
        let begin = family * POOL_SLOTS;
        for slot in &mut inner.returned[begin..begin + POOL_SLOTS] {
            if slot.is_none() {
                *slot = Some(Retained {
                    object,
                    prefix: lease.prefix,
                    cpu: lease.cpu,
                    poisoned: lease.poisoned,
                });
                return;
            }
        }
        // A private CacheLease reserves capacity before conversion, so a
        // missing slot means host accounting corruption. Never unmap a cached
        // object the firmware may still hold in that failure case.
        pr_err!("G17 cached-object return capacity was lost\n");
        core::mem::forget(object);
    }

    /// Releases retained mappings after both firmware processors have stopped.
    /// Active objects returned later observe the same stop witness.
    pub(crate) fn stop_after_firmware(&self) {
        self.inner.lock().stopped = true;
        for index in 0..POOL_FAMILIES * POOL_SLOTS {
            let entry = {
                let mut inner = self.inner.lock();
                let entry = inner.returned[index].take();
                if let Some(entry) = &entry {
                    inner.reserved[index / POOL_SLOTS] -= 1;
                    inner.bytes -= entry.object.size();
                }
                entry
            };
            // Mapping teardown must not run while the pool mutex is held.
            drop(entry);
        }
    }
}
