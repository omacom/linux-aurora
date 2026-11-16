// SPDX-License-Identifier: GPL-2.0

//! DRM GEM shmem helper objects
//!
//! C header: [`include/linux/drm/drm_gem_shmem_helper.h`](srctree/include/drm/drm_gem_shmem_helper.h)

// TODO:
// - There are a number of spots here that manually acquire/release the DMA reservation lock using
//   dma_resv_(un)lock(). In the future we should add support for ww mutex, expose a method to
//   acquire a reference to the WwMutex, and then use that directly instead of the C functions here.

use crate::{
    container_of,
    device::{
        self,
        Bound, //
    },
    devres::*,
    drm::{
        driver,
        gem,
        private::Sealed,
        Device, //
    },
    error::{
        from_err_ptr,
        to_result, //
    },
    io::{
        IoBase,
        Region,
        SysMem,
        SysMemBackend, //
    },
    prelude::*,
    scatterlist,
    sync::{
        aref::ARef,
        new_mutex,
        Mutex,
        SetOnce, //
    },
    types::{
        NotThreadSafe,
        Opaque, //
    },
};
use core::{
    ffi::c_void,
    mem::{
        ManuallyDrop,
        MaybeUninit, //
    },
    ops::{
        Deref,
        DerefMut, //
    },
    ptr::{
        self,
        NonNull, //
    },
    sync::atomic::{
        AtomicUsize,
        Ordering, //
    },
};
use gem::{
    BaseObject,
    BaseObjectPrivate,
    DriverObject,
    IntoGEMObject, //
};

/// A struct for controlling the creation of shmem-backed GEM objects.
///
/// This is used with [`Object::new()`] to control various properties that can only be set when
/// initially creating a shmem-backed GEM object.
pub struct ObjectConfig<'a, T: DriverObject> {
    /// Whether to set the write-combine map flag.
    pub map_wc: bool,

    /// Opt in to revocable CPU VMAs, with driver-controlled idle eviction.
    pub reclaimable_cpu_mappings: bool,

    /// Fail backing population when it would need direct reclaim.
    ///
    /// Background reclaim and ordinary allocation placement are preserved.
    /// Intended for large, fallible pin requests, not essential kernel backing.
    pub pages_no_reclaim: bool,

    /// Reuse the DMA reservation from another GEM object.
    ///
    /// The newly created [`Object`] will hold an owned refcount to `parent_resv_obj` if specified.
    pub parent_resv_obj: Option<&'a Object<T>>,
}

impl<'a, T: DriverObject> Default for ObjectConfig<'a, T> {
    #[inline(always)]
    fn default() -> Self {
        Self {
            map_wc: false,
            reclaimable_cpu_mappings: false,
            pages_no_reclaim: false,
            parent_resv_obj: None,
        }
    }
}

const SGT_RECLAIMING: usize = 1 << (usize::BITS - 1);
const SGT_USERS: usize = SGT_RECLAIMING - 1;

/// A shmem-backed GEM object.
///
/// # Invariants
///
/// - `obj` contains a valid initialized `struct drm_gem_shmem_object` for the lifetime of this
///   object.
#[repr(C)]
#[pin_data]
pub struct Object<T: DriverObject> {
    #[pin]
    obj: Opaque<bindings::drm_gem_shmem_object>,
    /// Parent object that owns this object's DMA reservation object.
    parent_resv_obj: Option<ARef<Object<T>>>,
    /// One cleanup registration for the C SG-table cache; never holds a table borrow.
    sgt_cleanup: ManuallyDrop<SetOnce<Devres<SGTableCleanup<T>>>>,
    #[pin]
    /// Serializes cleanup registration and table acquisition.
    sgt_lock: Mutex<()>,
    /// Counted SG leases exclude backing eviction; the high bit excludes new leases.
    sgt_access: AtomicUsize,
    #[pin]
    inner: T,
}

super::impl_aref_for_gem_obj! {
    impl<T> for Object<T>
    where
        T: DriverObject
}

// SAFETY: All GEM objects are thread-safe.
unsafe impl<T: DriverObject> Send for Object<T> {}

// SAFETY: All GEM objects are thread-safe.
unsafe impl<T: DriverObject> Sync for Object<T> {}

impl<T: DriverObject> Object<T> {
    /// `drm_gem_object_funcs` vtable suitable for GEM shmem objects.
    const VTABLE: bindings::drm_gem_object_funcs = bindings::drm_gem_object_funcs {
        free: Some(Self::free_callback),
        open: Some(super::open_callback::<T>),
        close: Some(super::close_callback::<T>),
        print_info: Some(bindings::drm_gem_shmem_object_print_info),
        export: if T::HAS_EXPORT {
            Some(super::export_callback::<T>)
        } else {
            None
        },
        pin: Some(bindings::drm_gem_shmem_object_pin),
        unpin: Some(bindings::drm_gem_shmem_object_unpin),
        get_sg_table: Some(bindings::drm_gem_shmem_object_get_sg_table),
        vmap: Some(bindings::drm_gem_shmem_object_vmap),
        vunmap: Some(bindings::drm_gem_shmem_object_vunmap),
        mmap: Some(bindings::drm_gem_shmem_object_mmap),
        status: None,
        rss: None,
        #[allow(unused_unsafe, reason = "Safe since Rust 1.82.0")]
        // SAFETY: `drm_gem_shmem_vm_ops` is a valid, static const on the C side.
        vm_ops: unsafe { &raw const bindings::drm_gem_shmem_vm_ops },
        evict: None,
    };

    /// Return a raw pointer to the embedded drm_gem_shmem_object.
    fn as_raw_shmem(&self) -> *mut bindings::drm_gem_shmem_object {
        self.obj.get()
    }

    /// Returns the `Device` that owns this GEM object.
    pub fn dev(&self) -> &Device<T::Driver> {
        // SAFETY: `dev` will have been initialized in `Self::new()` by `drm_gem_shmem_init()`.
        unsafe { Device::from_raw((*self.as_raw()).dev) }
    }

    extern "C" fn free_callback(obj: *mut bindings::drm_gem_object) {
        // SAFETY:
        // - DRM always passes a valid gem object here
        // - We used drm_gem_shmem_create() in our create_gem_object callback, so we know that
        //   `obj` is contained within a drm_gem_shmem_object
        let base = unsafe { container_of!(obj, bindings::drm_gem_shmem_object, base) };

        // SAFETY:
        // - We verified above that `obj` is valid, which makes `this` valid
        // - This function is set in AllocOps, so we know that `this` is contained within an
        //   `Object<T>`
        let this = unsafe { container_of!(Opaque::cast_from(base), Self, obj) }.cast_mut();

        // We need to drop `sgt_cleanup` first, since doing so requires that the GEM object is still
        // alive.
        // SAFETY:
        // - We verified above that `this` is valid.
        // - We are in free_callback, guaranteeing we have exclusive access to `this` and that
        //   `sgt_cleanup` will not be used after dropping it here.
        unsafe { ManuallyDrop::drop(&mut (*this).sgt_cleanup) };

        // SAFETY:
        // - We're in free_callback - so this function is safe to call.
        // - We won't be using the gem resources on `this` after this call.
        unsafe { bindings::drm_gem_shmem_release(base) };

        // SAFETY: We're recovering the Kbox<> we created in gem_create_object()
        let _ = unsafe { KBox::from_raw(this) };
    }

    /// Attempt to create a vmap from the gem object, and confirm the size of said vmap.
    fn make_vmap<'a, R, const SIZE: usize>(&'a self) -> Result<VMap<T, R, SIZE>>
    where
        R: Deref<Target = Self> + From<&'a Self>,
    {
        // INVARIANT: We check here that the gem object is at least as large as `SIZE`.
        if self.size() < SIZE {
            return Err(ENOSPC);
        }

        let mut map: MaybeUninit<bindings::iosys_map> = MaybeUninit::uninit();
        let guard = DmaResvGuard::new(self);

        // SAFETY: `drm_gem_shmem_vmap()` can be called with the DMA reservation lock held.
        to_result(unsafe {
            bindings::drm_gem_shmem_vmap_locked(self.as_raw_shmem(), map.as_mut_ptr())
        })?;

        // Drop the guard explicitly here, since we may need to call `raw_vunmap()` (which
        // re-acquires the lock).
        drop(guard);

        // SAFETY: The call to `drm_gem_shmem_vmap_locked()` succeeded above, so we are guaranteed
        // that map is properly initialized.
        let map = unsafe { map.assume_init() };

        // XXX: We don't currently support iomem allocations
        if map.is_iomem {
            // SAFETY: The vmap operation above succeeded, guaranteeing that `map` points to a valid
            // memory mapping.
            unsafe { self.raw_vunmap(map) };

            Err(ENOTSUPP)
        } else {
            Ok(VMap {
                // INVARIANT: `addr` remains valid for as long as `owner` does, which extends to the
                // lifetime of `VMap` itself.
                // SAFETY: We checked that this is not an iomem allocation, making it safe to read
                // vaddr.
                addr: unsafe { map.__bindgen_anon_1.vaddr },
                owner: self.into(),
            })
        }
    }

    /// Unmap a vmap from the gem object.
    ///
    /// # Safety
    ///
    /// - The caller promises that `map` is a valid vmap on this gem object.
    /// - The caller promises that the memory pointed to by map will no longer be accesed through
    ///   this instance.
    unsafe fn raw_vunmap(&self, mut map: bindings::iosys_map) {
        let _guard = DmaResvGuard::new(self);

        // SAFETY:
        // - This function is safe to call with the DMA reservation lock held.
        // - The caller promises that `map` is a valid vmap on this gem object.
        unsafe { bindings::drm_gem_shmem_vunmap_locked(self.as_raw_shmem(), &mut map) };
    }

    /// Creates and returns a virtual kernel memory mapping for this object.
    #[inline]
    pub fn vmap<const SIZE: usize>(&self) -> Result<VMapRef<'_, T, SIZE>> {
        self.make_vmap()
    }

    /// Acquire a revocable lease on this object's DMA scatter-gather table.
    ///
    /// Keep the lease for as long as any device mapping or copied DMA address uses the backing.
    /// The table borrow cannot outlive the lease. The device must be this object's parent.
    /// This may acquire the DMA reservation; call before locking it for a VM_BIND transaction.
    pub fn sg_table<'a>(&'a self, dev: &'a device::Device<Bound>) -> Result<SGTable<'a, T>> {
        let parent = self.dev().as_ref();
        if dev.as_raw() != parent.as_ref().as_raw() {
            return Err(EINVAL);
        }

        // Release and purge set the reclaim bit only while they hold this object's reservation.
        // Wait for that holder instead of failing a concurrent bind with a transient EBUSY.
        while let Err(e) = self.acquire_sgt_access() {
            if e != EBUSY {
                return Err(e);
            }
            drop(DmaResvGuard::new(self));
        }
        // Every failure after incrementing the count drops this lease.
        let lease = SGLease { obj: self.into() };

        // SAFETY: Import attachment ownership is immutable after publication.
        let sgt = if !unsafe { (*self.as_raw()).import_attach }.is_null() {
            // Imports belong to the exporter. Never register them for Rust devres cleanup,
            // rebuild their tables, or release their backing.
            // SAFETY: The attachment table stays valid until this object is freed.
            let sgt = unsafe { (*self.as_raw_shmem()).sgt };
            if sgt.is_null() {
                return Err(EINVAL);
            }
            sgt
        } else {
            // Register cleanup once, but cache the actual table only in C's shmem->sgt.
            // Reclaim clears that pointer. No Rust reference to it survives without a lease.
            let _guard = self.sgt_lock.lock();
            if self.sgt_cleanup.as_ref().is_none() {
                self.sgt_cleanup.populate(Devres::new(
                    dev,
                    Ok::<_, Error>(SGTableCleanup { obj: self.into() }),
                )?);
            }
            self.sgt_cleanup.as_ref().ok_or(EIO)?.access(dev)?;
            // SAFETY: The live object and lease exclude backing release. The helper locks
            // the reservation and either returns the cached table or rebuilds it.
            from_err_ptr(unsafe { bindings::drm_gem_shmem_get_pages_sgt(self.as_raw_shmem()) })?
        };

        Ok(SGTable {
            // SAFETY: The lease excludes release; the Bound device excludes devres revocation.
            table: unsafe { scatterlist::SGTable::from_raw(sgt) },
            lease,
        })
    }

    /// Create a new shmem-backed DRM object of the given size.
    ///
    /// Additional config options can be specified using `config`.
    pub fn new(
        dev: &Device<T::Driver>,
        size: usize,
        config: ObjectConfig<'_, T>,
        args: T::Args,
    ) -> Result<ARef<Self>> {
        let new: Pin<KBox<Self>> = KBox::try_pin_init(
            try_pin_init!(Self {
                obj <- Opaque::init_zeroed(),
                parent_resv_obj: config.parent_resv_obj.map(|p| p.into()),
                sgt_cleanup: ManuallyDrop::new(SetOnce::new()),
                sgt_lock <- new_mutex!(()),
                sgt_access: AtomicUsize::new(0),
                inner <- T::new(dev, size, args),
            }),
            GFP_KERNEL,
        )?;

        // SAFETY: `obj.as_raw()` is guaranteed to be valid by the initialization above.
        unsafe { (*new.as_raw()).funcs = &Self::VTABLE };

        // SAFETY: The object is initialized but not published. The C initializer
        // reads this policy when setting up the backing mapping's GFP mask.
        unsafe { (*new.as_raw_shmem()).pages_no_reclaim = config.pages_no_reclaim };

        // SAFETY: The arguments are all valid via the type invariants.
        to_result(unsafe { bindings::drm_gem_shmem_init(dev.as_raw(), new.as_raw_shmem(), size) })?;

        // SAFETY: We never move out of `self`.
        let new = KBox::into_raw(unsafe { Pin::into_inner_unchecked(new) });

        // SAFETY: We're taking over the owned refcount from `drm_gem_shmem_init`.
        let obj = unsafe { ARef::from_raw(NonNull::new_unchecked(new)) };

        // Start filling out values from `config`
        if let Some(parent_resv) = config.parent_resv_obj {
            // SAFETY: We have yet to expose the new gem object outside of this function, so it is
            // safe to modify this field.
            unsafe { (*obj.obj.get()).base.resv = parent_resv.raw_dma_resv() };
        }

        // SAFETY: We have yet to expose this object outside of this function, so we're guaranteed
        // to have exclusive access - thus making this safe to hold a mutable reference to.
        let shmem = unsafe { &mut *obj.as_raw_shmem() };
        shmem.set_map_wc(config.map_wc);
        shmem.reclaimable_cpu_mappings = config.reclaimable_cpu_mappings;

        Ok(obj)
    }

    /// `gem_create_object` callback, for the objects the DRM core allocates on the driver's
    /// behalf (dma-buf imports and dumb buffers) before it initializes their shmem part.
    ///
    /// Every GEM object of the driver is taken to be an `Object<T>`, these included, so build one
    /// with the driver data from [`DriverObject::core_object_args`].
    extern "C" fn create_object_callback(
        raw_dev: *mut bindings::drm_device,
        size: usize,
    ) -> *mut bindings::drm_gem_object {
        // SAFETY: The DRM core only calls this for a device of `T::Driver`, which outlives the
        // call.
        let dev = unsafe { Device::<T::Driver>::from_raw(raw_dev) };

        let new = T::core_object_args(dev, size).and_then(|args| {
            KBox::try_pin_init(
                try_pin_init!(Self {
                    obj <- Opaque::init_zeroed(),
                    parent_resv_obj: None,
                    sgt_cleanup: ManuallyDrop::new(SetOnce::new()),
                    sgt_lock <- new_mutex!(()),
                    sgt_access: AtomicUsize::new(0),
                    inner <- T::new(dev, size, args),
                }),
                GFP_KERNEL,
            )
        });
        let new = match new {
            Ok(new) => new,
            Err(e) => return e.to_ptr(),
        };

        // SAFETY: We never move out of `self`. The DRM core owns the object from here on and
        // releases it through `free_callback()`. An object the core then fails to initialize is
        // freed with `kfree()`, which the `KBox` allocator allows, without dropping `inner`.
        let new = KBox::into_raw(unsafe { Pin::into_inner_unchecked(new) });

        // SAFETY: `new` is a valid object that nothing else references yet.
        unsafe {
            let obj = (*new).obj.get();
            (*obj).base.funcs = &Self::VTABLE;
            &raw mut (*obj).base
        }
    }

    fn acquire_sgt_access(&self) -> Result {
        let mut state = self.sgt_access.load(Ordering::Acquire);
        loop {
            if state & SGT_RECLAIMING != 0 {
                return Err(EBUSY);
            }
            if state & SGT_USERS == SGT_USERS {
                return Err(EOVERFLOW);
            }
            match self.sgt_access.compare_exchange_weak(
                state,
                state + 1,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Ok(()),
                Err(current) => state = current,
            }
        }
    }

    /// Whether the caller's sole SG lease is the last user of the backing.
    ///
    /// # Safety
    ///
    /// The caller holds the object's DMA reservation lock.
    pub unsafe fn sole_sg_lease_releasable_locked(&self) -> bool {
        self.sgt_access.load(Ordering::Acquire) == 1 &&
            // SAFETY: The caller establishes the reservation-lock contract.
            unsafe { bindings::drm_gem_shmem_idle_pages_releasable_locked(self.as_raw_shmem()) }
    }

    /// Whether the object still has a cached device SG table.
    ///
    /// # Safety
    ///
    /// The caller holds the object's DMA reservation lock.
    pub unsafe fn has_sg_table_locked(&self) -> bool {
        // SAFETY: Reservation ownership serializes the C table pointer.
        !unsafe { (*self.as_raw_shmem()).sgt }.is_null()
    }

    /// Whether backing without a Rust SG lease can be reclaimed or purged.
    ///
    /// Also accepts backing whose SG table was already released by idle reclaim.
    ///
    /// # Safety
    ///
    /// The caller holds the object's DMA reservation lock.
    pub unsafe fn unleased_pages_releasable_locked(&self) -> bool {
        self.sgt_access.load(Ordering::Acquire) == 0 &&
            // SAFETY: The caller establishes the reservation-lock contract.
            unsafe { bindings::drm_gem_shmem_idle_pages_releasable_locked(self.as_raw_shmem()) }
    }

    /// Release idle backing without discarding contents. A later SG table may have different DMA
    /// addresses.
    ///
    /// Refused with `EBUSY` while any SG lease remains. The cached C table is cleared on
    /// success and the next [`Self::sg_table`] rebuilds it before exposing DMA addresses.
    ///
    /// # Safety
    ///
    /// The caller holds this object's DMA reservation lock, has removed and invalidated every
    /// device mapping, and excludes new device access.
    pub unsafe fn release_idle_pages_locked(&self) -> Result {
        self.sgt_access
            .compare_exchange(0, SGT_RECLAIMING, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| EBUSY)?;
        // SAFETY: The caller's contract plus the exclusive state above.
        let result = to_result(unsafe {
            bindings::drm_gem_shmem_release_idle_pages_locked(self.as_raw_shmem())
        });
        self.sgt_access.store(0, Ordering::Release);
        result
    }

    /// Apply purge advice (0 = needed, >0 = may be purged) under the
    /// object's reservation lock. Returns false if the object was already
    /// purged; the advice is then ignored.
    pub fn madvise(&self, madv: i32) -> bool {
        let _guard = DmaResvGuard::new(self);
        // SAFETY: The reservation lock is held, as the C helper requires.
        unsafe { bindings::drm_gem_shmem_madvise_locked(self.as_raw_shmem(), madv) != 0 }
    }

    /// Current purge advice: negative once purged.
    ///
    /// # Safety
    ///
    /// The caller holds the object's DMA reservation lock.
    pub unsafe fn madv_locked(&self) -> i32 {
        // SAFETY: Reservation ownership serializes the field.
        unsafe { (*self.as_raw_shmem()).madv }
    }

    /// Discard the backing of a purgeable object. The handle and logical
    /// mappings remain; the contents are gone and `madv_locked()` turns negative.
    ///
    /// # Safety
    ///
    /// The caller holds this object's DMA reservation lock, has removed and
    /// invalidated every device mapping, and no Rust SG lease remains.
    pub unsafe fn purge_locked(&self) -> Result {
        // SAFETY: The caller's contract; the exclusive state below excludes new leases.
        if !unsafe { bindings::drm_gem_shmem_is_purgeable(self.as_raw_shmem()) }
            || !unsafe { bindings::drm_gem_shmem_idle_pages_releasable_locked(self.as_raw_shmem()) }
        {
            return Err(EBUSY);
        }
        self.sgt_access
            .compare_exchange(0, SGT_RECLAIMING, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| EBUSY)?;
        // SAFETY: See above.
        unsafe { bindings::drm_gem_shmem_purge_locked(self.as_raw_shmem()) };
        self.sgt_access.store(0, Ordering::Release);
        Ok(())
    }

    /// Creates and returns an owned reference to a virtual kernel memory mapping for this object.
    #[inline]
    pub fn owned_vmap<const SIZE: usize>(&self) -> Result<VMapOwned<T, SIZE>> {
        self.make_vmap()
    }
}

impl<T: DriverObject> Deref for Object<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: DriverObject> DerefMut for Object<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: DriverObject> Sealed for Object<T> {}

impl<T: DriverObject> gem::IntoGEMObject for Object<T> {
    fn as_raw(&self) -> *mut bindings::drm_gem_object {
        // SAFETY:
        // - Our immutable reference is proof that this is safe to dereference.
        // - `obj` is always a valid drm_gem_shmem_object via our type invariants.
        unsafe { &raw mut (*self.obj.get()).base }
    }

    unsafe fn from_raw<'a>(obj: *mut bindings::drm_gem_object) -> &'a Self {
        // SAFETY: The safety contract of from_gem_obj() guarantees that `obj` is contained within
        // `Self`
        unsafe {
            let obj = Opaque::cast_from(container_of!(obj, bindings::drm_gem_shmem_object, base));

            &*container_of!(obj, Self, obj)
        }
    }
}

impl<T: DriverObject> driver::AllocImpl for Object<T> {
    type Driver = T::Driver;

    const ALLOC_OPS: driver::AllocOps = driver::AllocOps {
        gem_create_object: if T::HAS_CORE_OBJECT_ARGS {
            Some(Self::create_object_callback)
        } else {
            None
        },
        prime_handle_to_fd: None,
        prime_fd_to_handle: None,
        gem_prime_import: None,
        gem_prime_import_sg_table: Some(bindings::drm_gem_shmem_prime_import_sg_table),
        dumb_create: Some(bindings::drm_gem_shmem_dumb_create),
        dumb_map_offset: None,
    };
}

/// Private helper-type for holding the `dma_resv` object for a GEM shmem object.
///
/// When this is dropped, the `dma_resv` lock is dropped as well.
///
// TODO: This should be replace with a WwMutex equivalent once we have such bindings in the kernel.
struct DmaResvGuard<'a, T: DriverObject>(&'a Object<T>, NotThreadSafe);

impl<'a, T: DriverObject> DmaResvGuard<'a, T> {
    #[inline]
    fn new(obj: &'a Object<T>) -> Self {
        // SAFETY: This lock is initialized throughout the lifetime of `object`.
        unsafe { bindings::dma_resv_lock(obj.raw_dma_resv(), ptr::null_mut()) };

        Self(obj, NotThreadSafe)
    }
}

impl<'a, T: DriverObject> Drop for DmaResvGuard<'a, T> {
    #[inline]
    fn drop(&mut self) {
        // SAFETY: We are releasing the lock grabbed during the creation of this object.
        unsafe { bindings::dma_resv_unlock(self.0.raw_dma_resv()) };
    }
}

/// A reference to a virtual mapping for an shmem-based GEM object in kernel address space.
///
/// # Invariants
///
/// - The size of `owner` is >= SIZE.
/// - The memory pointed to by `addr` remains valid at least until this object is dropped.
pub struct VMap<D, R, const SIZE: usize = 0>
where
    D: DriverObject,
    R: Deref<Target = Object<D>>,
{
    /// FIXME: Should not be public!
    pub addr: *mut c_void,
    owner: R,
}

/// An alias type for a reference to a shmem-based GEM object's VMap.
pub type VMapRef<'a, D, const SIZE: usize = 0> = VMap<D, &'a Object<D>, SIZE>;

/// An alias type for an owned reference to a shmem-based GEM object's VMap.
pub type VMapOwned<D, const SIZE: usize = 0> = VMap<D, ARef<Object<D>>, SIZE>;

impl<D, R, const SIZE: usize> VMap<D, R, SIZE>
where
    D: DriverObject,
    R: Deref<Target = Object<D>>,
{
    /// Borrows a reference to the object that owns this virtual mapping.
    #[inline]
    pub fn owner(&self) -> &Object<D> {
        &self.owner
    }
}

impl<'a, D, R, const SIZE: usize> IoBase<'a> for &'a VMap<D, R, SIZE>
where
    D: DriverObject,
    R: Deref<Target = Object<D>>,
{
    type Backend = SysMemBackend;
    type Target = Region<SIZE>;

    #[inline]
    fn as_view(self) -> SysMem<'a, Region<SIZE>> {
        let ptr = Region::ptr_from_raw_parts_mut(self.addr.cast(), self.owner.size());

        // SAFETY: Per type invariants of `VMap`:
        // - `addr .. addr + owner.size()` is a valid kernel accessible memory region.
        // - `addr` is page-aligned, which satisfies `Region`'s 4-byte alignment requirement.
        // - The memory remains valid until this `VMap` is dropped; since `self` is `&'a VMap`,
        //   the borrow prevents the `VMap` from being dropped for the lifetime `'a`.
        unsafe { SysMem::new(ptr) }
    }
}

impl<D, R, const SIZE: usize> Drop for VMap<D, R, SIZE>
where
    D: DriverObject,
    R: Deref<Target = Object<D>>,
{
    #[inline]
    fn drop(&mut self) {
        // SAFETY:
        // - Our existence is proof that this map was previously created using self.owner.
        // - Since we are in Drop, we are guaranteed that no one will access the memory
        //   through this mapping after calling this.
        unsafe {
            self.owner.raw_vunmap(bindings::iosys_map {
                is_iomem: false,
                __bindgen_anon_1: bindings::iosys_map__bindgen_ty_1 { vaddr: self.addr },
            })
        };
    }
}

// SAFETY: `addr` points to a valid memory address for as long as `owner` exists, meaning that so
// long as `owner` is `Send` so is `VMap`.
unsafe impl<D, R, const SIZE: usize> Send for VMap<D, R, SIZE>
where
    D: DriverObject,
    R: Deref<Target = Object<D>> + Send,
{
}

// SAFETY: `addr` points to a valid memory address for as long as `owner` exists, meaning that so
// long as `owner` is `Sync` so is `VMap`.
unsafe impl<D, R, const SIZE: usize> Sync for VMap<D, R, SIZE>
where
    D: DriverObject,
    R: Deref<Target = Object<D>> + Sync,
{
}

/// An owned backing lease, suitable for retaining copied DMA segments.
///
/// Dropping the last lease permits reclaim after device translations have been invalidated.
/// This does not itself provide access to the table: table access also requires a Bound device.
#[must_use = "retain until all device uses of the backing have ended"]
pub struct SGLease<T: DriverObject> {
    obj: ARef<Object<T>>,
}

impl<T: DriverObject> Drop for SGLease<T> {
    fn drop(&mut self) {
        let old = self.obj.sgt_access.fetch_sub(1, Ordering::Release);
        debug_assert!(old & SGT_USERS != 0 && old & SGT_RECLAIMING == 0);
    }
}

/// A leased SG-table borrow. References obtained from it cannot outlive the lease.
#[must_use = "retain while using the SG table or transfer into an owned lease"]
pub struct SGTable<'a, T: DriverObject> {
    table: &'a scatterlist::SGTable,
    lease: SGLease<T>,
}

impl<T: DriverObject> Deref for SGTable<'_, T> {
    type Target = scatterlist::SGTable;

    fn deref(&self) -> &Self::Target {
        self.table
    }
}

impl<T: DriverObject> SGTable<'_, T> {
    /// Retain the backing after copying the DMA addresses, without retaining a table borrow.
    pub fn into_lease(self) -> SGLease<T> {
        self.lease
    }
}

/// Device-unbind cleanup registration for locally allocated SG tables.
///
/// This token owns no table reference: reclaim may clear shmem->sgt and a later lease may
/// rebuild it. It is dropped before the GEM object, and is never constructed for imports.
struct SGTableCleanup<T: DriverObject> {
    obj: NonNull<Object<T>>,
}

impl<T: DriverObject> Drop for SGTableCleanup<T> {
    fn drop(&mut self) {
        // SAFETY: The registration is dropped before its GEM object.
        let obj = unsafe { self.obj.as_ref() };
        let _lock = DmaResvGuard::new(obj);

        // SAFETY: Reservation ownership serializes shmem->sgt. Imports never get this token.
        // Reclaim may already have freed the table; the C free helper requires a non-null one.
        unsafe {
            if !(*obj.as_raw_shmem()).sgt.is_null() {
                bindings::__drm_gem_shmem_free_sgt_locked(obj.as_raw_shmem());
            }
        }
    }
}

// SAFETY: The NonNull is valid until cleanup, and its GEM object is thread-safe.
unsafe impl<T: DriverObject> Send for SGTableCleanup<T> {}
// SAFETY: The NonNull is valid until cleanup, and its GEM object is thread-safe.
unsafe impl<T: DriverObject> Sync for SGTableCleanup<T> {}

#[kunit_tests(rust_drm_gem_shmem)]
mod tests {
    use super::*;
    use crate::{
        drm::{
            self,
            UnregisteredDevice, //
        },
        faux,
        io::Io,
        page::PAGE_SIZE, //
    };

    // The bare minimum needed to create a fake drm driver for kunit

    #[pin_data]
    struct KunitData {}
    struct KunitDriver;
    struct KunitFile;
    #[pin_data]
    struct KunitObject {}

    const INFO: drm::DriverInfo = drm::DriverInfo {
        major: 0,
        minor: 0,
        patchlevel: 0,
        name: c"kunit",
        desc: c"Kunit",
    };

    impl drm::file::DriverFile for KunitFile {
        type Driver = KunitDriver;

        fn open(_dev: &drm::Device<KunitDriver, drm::Registered>) -> Result<Pin<KBox<Self>>> {
            // These object tests never create client files.
            Err(ENODEV)
        }

        fn as_raw(&self) -> *mut bindings::drm_file {
            // A successful open is required before a raw file can be requested.
            unreachable!("KUnit object tests do not open DRM files")
        }
    }

    #[vtable]
    impl gem::DriverObject for KunitObject {
        type Driver = KunitDriver;
        type Args = ();

        fn new(
            _dev: &drm::Device<KunitDriver>,
            _size: usize,
            _args: Self::Args,
        ) -> impl PinInit<Self, Error> {
            try_pin_init!(KunitObject {})
        }
    }

    #[vtable]
    impl drm::Driver for KunitDriver {
        type Data = KunitData;
        type RegistrationData<'a> = ();
        type File = KunitFile;
        type Object = Object<KunitObject>;
        type ParentDevice<Ctx: device::DeviceContext> = faux::Device<Ctx>;

        const INFO: drm::DriverInfo = INFO;
        const IOCTLS: &'static [drm::ioctl::DrmIoctlDescriptor] = &[];
    }

    fn create_drm_dev() -> Result<(faux::Registration, UnregisteredDevice<KunitDriver>)> {
        // Create a faux DRM device so we can test gem object creation.
        let data = try_pin_init!(KunitData {});
        let reg = faux::Registration::new(c"Kunit", None)?;
        let fdev = reg.as_ref();
        let dev: &device::Device<Bound> = fdev.as_ref();
        // SAFETY: This new faux device is private to the fixture. Use direct DMA with a
        // 64-bit mask, as the C shmem KUnit fixture does, before publishing any mappings.
        unsafe {
            let raw = dev.as_raw();
            (*raw).coherent_dma_mask = u64::MAX;
            (*raw).dma_mask = &raw mut (*raw).coherent_dma_mask;
        }
        let drm = UnregisteredDevice::new(fdev, data)?;

        Ok((reg, drm))
    }

    #[test]
    fn compile_time_vmap_sizes() -> Result {
        let (_dev, drm) = create_drm_dev()?;

        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;

        // Try creating a normal vmap
        obj.vmap::<PAGE_SIZE>()?;

        // Try creating a vmap that's smaller then the size we specified
        let vmap = obj.vmap::<{ PAGE_SIZE - 100 }>()?;

        // Verify the owner matches
        assert!(ptr::eq(vmap.owner(), obj.deref()));

        // Verify the size matches the actual object size
        assert_eq!(vmap.size(), PAGE_SIZE);

        // Make sure creating a vmap that's too large fails
        assert!(obj.vmap::<{ PAGE_SIZE + 200 }>().is_err());

        Ok(())
    }

    #[test]
    fn vmap_io() -> Result {
        let (_dev, drm) = create_drm_dev()?;

        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;

        let vmap = obj.vmap::<PAGE_SIZE>()?;

        vmap.write8(0xDE, 0x0);
        assert_eq!(vmap.read8(0x0), 0xDE);
        vmap.write32(0xFEDCBA98, 0x20);

        assert_eq!(vmap.read32(0x20), 0xFEDCBA98);

        // Ensure the ordering in memory is correct
        let expected = 0xFEDCBA98_u32.to_ne_bytes().into_iter();
        for (offset, expected) in (0x20..=0x23).zip(expected) {
            assert_eq!(vmap.try_read8(offset).unwrap(), expected);
        }

        Ok(())
    }

    #[test]
    fn sg_lease_counting() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        let first = obj.sg_table(dev.as_ref().as_ref())?.into_lease();
        let second = obj.sg_table(dev.as_ref().as_ref())?;
        assert_eq!(obj.sgt_access.load(Ordering::Acquire), 2);
        drop(first);
        assert_eq!(obj.sgt_access.load(Ordering::Acquire), 1);
        drop(second);
        assert_eq!(obj.sgt_access.load(Ordering::Acquire), 0);
        Ok(())
    }

    #[test]
    fn release_idle_without_mappings() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        drop(obj.sg_table(dev.as_ref().as_ref())?);
        let _guard = DmaResvGuard::new(&obj);
        // SAFETY: Reservation held, and no device mappings or SG leases exist in this test.
        unsafe { obj.release_idle_pages_locked()? };
        // SAFETY: Reservation ownership serializes these fields.
        let empty = unsafe {
            (
                (*obj.as_raw_shmem()).sgt.is_null(),
                (*obj.as_raw_shmem()).pages.is_null(),
            )
        };
        assert!(empty.0);
        assert!(empty.1);
        // SAFETY: Still no device mappings or leases; an already idle release succeeds too.
        unsafe { obj.release_idle_pages_locked()? };
        Ok(())
    }

    #[test]
    fn release_refused_while_leased() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        let mapping = obj.sg_table(dev.as_ref().as_ref())?.into_lease();
        let bind = obj.sg_table(dev.as_ref().as_ref())?.into_lease();
        let _guard = DmaResvGuard::new(&obj);
        // SAFETY: Reservation held; no actual device mappings were installed.
        assert!(!unsafe { obj.sole_sg_lease_releasable_locked() });
        // SAFETY: Reservation held; no actual device mappings were installed.
        assert_eq!(unsafe { obj.release_idle_pages_locked() }, Err(EBUSY));
        drop(bind);
        // SAFETY: Reservation held; the mapping lease is now the sole owner.
        assert!(unsafe { obj.sole_sg_lease_releasable_locked() });
        // SAFETY: Reservation held; the remaining lease must still exclude release.
        assert_eq!(unsafe { obj.release_idle_pages_locked() }, Err(EBUSY));
        drop(mapping);
        // SAFETY: Reservation held; no device mapping or SG lease remains.
        unsafe { obj.release_idle_pages_locked()? };
        Ok(())
    }

    #[test]
    fn purge_madvised_after_last_lease() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        let mapping = obj.sg_table(dev.as_ref().as_ref())?.into_lease();
        assert!(obj.madvise(1));
        {
            let _guard = DmaResvGuard::new(&obj);
            // SAFETY: Reservation held; no real device access.
            assert_eq!(unsafe { obj.purge_locked() }, Err(EBUSY));
            drop(mapping);
            // SAFETY: Reservation held; all device uses and SG leases have ended.
            unsafe { obj.purge_locked()? };
            // SAFETY: Reservation ownership serializes the backing fields and advice.
            let (empty_sgt, empty_pages, madv) = unsafe {
                (
                    (*obj.as_raw_shmem()).sgt.is_null(),
                    (*obj.as_raw_shmem()).pages.is_null(),
                    obj.madv_locked(),
                )
            };
            assert!(empty_sgt);
            assert!(empty_pages);
            assert_eq!(madv, -1);
        }
        assert!(!obj.madvise(0));
        assert!(obj.sg_table(dev.as_ref().as_ref()).is_err());
        assert_eq!(obj.sgt_access.load(Ordering::Acquire), 0);
        Ok(())
    }

    #[test]
    fn sg_rebuild_after_release() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        obj.vmap::<4>()?.as_view().write32(0x12345678, 0);
        for _ in 0..3 {
            let table = obj.sg_table(dev.as_ref().as_ref())?;
            assert!(table.iter().count() > 0);
            drop(table);
            {
                let _guard = DmaResvGuard::new(&obj);
                // SAFETY: Reservation held; all leases have dropped, with no device mappings.
                unsafe { obj.release_idle_pages_locked()? };
                // SAFETY: Reservation ownership serializes the table pointer.
                assert!(unsafe { (*obj.as_raw_shmem()).sgt.is_null() });
            }
            assert_eq!(obj.vmap::<4>()?.as_view().read32(0), 0x12345678);
        }
        // Drop with a rebuilt table exercises the surviving devres cleanup registration too.
        drop(obj.sg_table(dev.as_ref().as_ref())?);
        Ok(())
    }

    #[test]
    fn purge_after_idle_release() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        drop(obj.sg_table(dev.as_ref().as_ref())?);
        {
            let _guard = DmaResvGuard::new(&obj);
            // SAFETY: Reservation held; no device access or SG leases remain.
            unsafe { obj.release_idle_pages_locked()? };
        }
        assert!(obj.madvise(1));
        let _guard = DmaResvGuard::new(&obj);
        // SAFETY: Reservation held; no device access or SG leases remain.
        assert!(unsafe { obj.unleased_pages_releasable_locked() });
        // SAFETY: Reservation held; no device access or SG lease remains.
        unsafe { obj.purge_locked()? };
        // SAFETY: Reservation ownership serializes the advice and table pointer.
        let (madv, empty) = unsafe { (obj.madv_locked(), (*obj.as_raw_shmem()).sgt.is_null()) };
        assert_eq!(madv, -1);
        assert!(empty);
        Ok(())
    }

    #[test]
    fn purge_refuses_kernel_vmap() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        drop(obj.sg_table(dev.as_ref().as_ref())?);
        let vmap = obj.vmap::<4>()?;
        assert!(obj.madvise(1));
        {
            let _guard = DmaResvGuard::new(&obj);
            // SAFETY: Reservation held; the vmap must prevent the backing from being freed.
            assert_eq!(unsafe { obj.purge_locked() }, Err(EBUSY));
            // SAFETY: Reservation held; the vmap must exclude idle release too.
            assert_eq!(unsafe { obj.release_idle_pages_locked() }, Err(EBUSY));
        }
        drop(vmap);
        let _guard = DmaResvGuard::new(&obj);
        // SAFETY: Reservation held; no device, vmap or SG users remain.
        unsafe { obj.purge_locked()? };
        Ok(())
    }

    #[test]
    fn sg_reclaim_excludes_new_leases() -> Result {
        let (_dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        obj.sgt_access.store(SGT_RECLAIMING, Ordering::Release);
        assert_eq!(obj.acquire_sgt_access(), Err(EBUSY));
        assert_eq!(obj.sgt_access.load(Ordering::Acquire), SGT_RECLAIMING);
        obj.sgt_access.store(SGT_USERS, Ordering::Release);
        assert_eq!(obj.acquire_sgt_access(), Err(EOVERFLOW));
        obj.sgt_access.store(0, Ordering::Release);
        Ok(())
    }

    #[test]
    fn sg_import_attachment_is_not_owned() -> Result {
        let (dev, drm) = create_drm_dev()?;
        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;
        let mut table = MaybeUninit::<bindings::sg_table>::zeroed();
        let mut attachment = MaybeUninit::<bindings::dma_buf_attachment>::zeroed();
        // SAFETY: Private fixture; neither mock is dereferenced by the import path. Both stay
        // live until the lease drops and are detached before the real object destructor runs.
        unsafe {
            (*obj.as_raw()).import_attach = attachment.as_mut_ptr();
            (*obj.as_raw_shmem()).sgt = table.as_mut_ptr();
        }
        let result = obj.sg_table(dev.as_ref().as_ref()).map(|lease| {
            let no_cleanup = obj.sgt_cleanup.as_ref().is_none();
            let _guard = DmaResvGuard::new(&obj);
            // SAFETY: Reservation held. Capture results before restoring the fixture, so a
            // failed assertion cannot leave a mock attachment for the GEM destructor.
            unsafe {
                let held = obj.release_idle_pages_locked();
                drop(lease);
                (
                    no_cleanup,
                    held,
                    obj.release_idle_pages_locked(),
                    obj.purge_locked(),
                    (*obj.as_raw_shmem()).sgt == table.as_mut_ptr(),
                )
            }
        });
        // SAFETY: The mock table borrow has ended; restore the private fixture for normal free.
        unsafe {
            (*obj.as_raw()).import_attach = ptr::null_mut();
            (*obj.as_raw_shmem()).sgt = ptr::null_mut();
        }
        let (no_cleanup, held, unleased, purge, unchanged) = result?;
        assert!(no_cleanup);
        assert_eq!(held, Err(EBUSY));
        assert_eq!(unleased, Err(EINVAL));
        assert_eq!(purge, Err(EBUSY));
        assert!(unchanged);
        Ok(())
    }

    // A wrong device must fail before registering cleanup or taking any SG lease.
    #[test]
    fn fail_sg_table_on_wrong_dev() -> Result {
        let (_dev, drm) = create_drm_dev()?;
        let reg = faux::Registration::new(c"EvilKunit", None)?;
        let wrong_dev = reg.as_ref();

        let obj = Object::<KunitObject>::new(&drm, PAGE_SIZE, ObjectConfig::default(), ())?;

        assert_eq!(obj.sg_table(wrong_dev.as_ref()).err().unwrap(), EINVAL);

        // A failed lookup must not bind the cleanup registration to the wrong device.
        assert_eq!(obj.sg_table(wrong_dev.as_ref()).err().unwrap(), EINVAL);

        assert_eq!(obj.sgt_access.load(Ordering::Acquire), 0);

        Ok(())
    }
}
