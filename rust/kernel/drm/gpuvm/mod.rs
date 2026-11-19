// SPDX-License-Identifier: GPL-2.0 OR MIT

#![cfg(CONFIG_RUST_DRM_GPUVM)]

//! DRM GPUVM in immediate mode
//!
//! Rust abstractions for using GPUVM in immediate mode. This is when the GPUVM state is updated
//! during `run_job()`, i.e., in the DMA fence signalling critical path, to ensure that the GPUVM
//! and the GPU's virtual address space has the same state at all times.
//!
//! C header: [`include/drm/drm_gpuvm.h`](srctree/include/drm/drm_gpuvm.h)

use kernel::{
    alloc::{
        AllocError,
        Flags as AllocFlags, //
    },
    bindings,
    drm,
    drm::gem::{
        BaseObject,
        IntoGEMObject, //
    },
    error::to_result,
    prelude::*,
    sync::aref::{
        ARef,
        AlwaysRefCounted, //
    },
    types::{
        NotThreadSafe,
        Opaque, //
    }, //
};

use core::{
    cell::UnsafeCell,
    marker::PhantomData,
    mem::{
        ManuallyDrop,
        MaybeUninit, //
    },
    ops::{
        Deref,
        DerefMut,
        Range, //
    },
    ptr::{
        self,
        NonNull, //
    }, //
};

mod sm_ops;
pub use self::sm_ops::*;

mod vm_bo;
pub use self::vm_bo::*;

mod va;
pub use self::va::*;

pub(super) unsafe extern "C" fn exec_lock_gem_object(
    vm_exec: *mut bindings::drm_gpuvm_exec,
) -> core::ffi::c_int {
    // SAFETY: The gpuvm_exec object is valid and priv_ is a GEM object pointer
    // when this callback is used
    unsafe { bindings::drm_exec_lock_obj(&mut (*vm_exec).exec, (*vm_exec).extra.priv_ as *mut _) }
}

/// A DRM GPU VA manager.
///
/// This object is refcounted, but the locations of mapped ranges may only be accessed or changed
/// via the special unique handle [`UniqueRefGpuVm`].
///
/// # Invariants
///
/// * Stored in an allocation managed by the refcount in `self.vm`.
/// * Access to `data` and the gpuvm interval tree is controlled via the [`UniqueRefGpuVm`] type.
/// * Does not contain any sparse [`GpuVa<T>`] instances.
#[pin_data]
pub struct GpuVm<T: DriverGpuVm> {
    #[pin]
    vm: Opaque<bindings::drm_gpuvm>,
    /// Accessed only through the [`UniqueRefGpuVm`] reference.
    data: UnsafeCell<T>,
}

// SAFETY: It is safe to send a `GpuVm<T>` to another thread: all data reachable through it
// (`T`, `T::VmBoData`, and the GEM `T::Object`) is `Send` by the `DriverGpuVm` bounds.
unsafe impl<T: DriverGpuVm> Send for GpuVm<T> {}
// SAFETY: It is safe to share a `&GpuVm<T>` between threads: `&self` methods only alias data
// that is `Sync` by the `DriverGpuVm` bounds, and any thread may drop that data, or upgrade the
// reference and ultimately drop `T`, which the same bounds make `Send`.
unsafe impl<T: DriverGpuVm> Sync for GpuVm<T> {}

// SAFETY: By type invariants, the allocation is managed by the refcount in `self.vm`.
unsafe impl<T: DriverGpuVm> AlwaysRefCounted for GpuVm<T> {
    fn inc_ref(&self) {
        // SAFETY: By type invariants, the allocation is managed by the refcount in `self.vm`.
        unsafe { bindings::drm_gpuvm_get(self.vm.get()) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: By type invariants, the allocation is managed by the refcount in `self.vm`.
        unsafe { bindings::drm_gpuvm_put((*obj.as_ptr()).vm.get()) };
    }
}

impl<T: DriverGpuVm> PartialEq for GpuVm<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.as_raw(), other.as_raw())
    }
}
impl<T: DriverGpuVm> Eq for GpuVm<T> {}

impl<T: DriverGpuVm> GpuVm<T> {
    const fn vtable() -> &'static bindings::drm_gpuvm_ops {
        &bindings::drm_gpuvm_ops {
            vm_free: Some(Self::vm_free),
            op_alloc: None,
            op_free: None,
            vm_bo_alloc: GpuVmBo::<T>::ALLOC_FN,
            vm_bo_free: GpuVmBo::<T>::FREE_FN,
            vm_bo_validate: None,
            sm_step_map: Some(Self::sm_step_map),
            sm_step_unmap: Some(Self::sm_step_unmap),
            sm_step_remap: Some(Self::sm_step_remap),
            sm_can_merge_flags: None,
        }
    }

    /// Creates a GPUVM instance.
    #[expect(clippy::new_ret_no_self)]
    pub fn new<E, Ctx: drm::DeviceContext>(
        name: &'static CStr,
        dev: &drm::Device<T::Driver, Ctx>,
        r_obj: &T::Object,
        range: Range<u64>,
        reserve_range: Range<u64>,
        data: T,
    ) -> Result<UniqueRefGpuVm<T>, E>
    where
        E: From<AllocError>,
        E: From<core::convert::Infallible>,
    {
        let obj = KBox::try_pin_init::<E>(
            try_pin_init!(Self {
                data: UnsafeCell::new(data),
                vm <- Opaque::ffi_init(|vm| {
                    // SAFETY: These arguments are valid. `vm` is valid until refcount drops to
                    // zero. The `vm` is zeroed before calling this method by `__GFP_ZERO` flag
                    // below.
                    unsafe {
                        bindings::drm_gpuvm_init(
                            vm,
                            name.as_char_ptr(),
                            bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE
                                | bindings::drm_gpuvm_flags_DRM_GPUVM_RESV_PROTECTED,
                            dev.as_raw(),
                            r_obj.as_raw(),
                            range.start,
                            range.end - range.start,
                            reserve_range.start,
                            reserve_range.end - reserve_range.start,
                            const { Self::vtable() },
                        )
                    }
                }),
            }? E),
            GFP_KERNEL | __GFP_ZERO,
        )?;
        // SAFETY: This transfers the initial refcount to the ARef.
        let aref = unsafe {
            ARef::from_raw(NonNull::new_unchecked(KBox::into_raw(
                Pin::into_inner_unchecked(obj),
            )))
        };
        // INVARIANT: This reference is unique.
        Ok(UniqueRefGpuVm(aref))
    }

    /// Access this [`GpuVm`] from a raw pointer.
    ///
    /// # Safety
    ///
    /// The pointer must reference the `struct drm_gpuvm` in a valid [`GpuVm<T>`] that remains
    /// valid for at least `'a`.
    #[inline]
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::drm_gpuvm) -> &'a Self {
        // SAFETY: Caller passes a pointer to the `drm_gpuvm` in a `GpuVm<T>`. Caller ensures the
        // pointer is valid for 'a.
        unsafe { &*kernel::container_of!(Opaque::cast_from(ptr), Self, vm) }
    }

    /// Returns a raw pointer to the embedded `struct drm_gpuvm`.
    #[inline]
    pub fn as_raw(&self) -> *mut bindings::drm_gpuvm {
        self.vm.get()
    }

    /// The start of the VA space.
    #[inline]
    pub fn va_start(&self) -> u64 {
        // SAFETY: The `mm_start` field is immutable.
        unsafe { (*self.as_raw()).mm_start }
    }

    /// The length of the GPU's virtual address space.
    #[inline]
    pub fn va_length(&self) -> u64 {
        // SAFETY: The `mm_range` field is immutable.
        unsafe { (*self.as_raw()).mm_range }
    }

    /// Returns the range of the GPU virtual address space.
    #[inline]
    pub fn va_range(&self) -> Range<u64> {
        let start = self.va_start();
        // OVERFLOW: This reconstructs the Range<u64> passed to the constructor, so it won't fail.
        let end = start + self.va_length();
        Range { start, end }
    }

    /// Get or create the [`GpuVmBo`] for this gem object.
    #[inline]
    pub fn obtain(
        &self,
        obj: &T::Object,
        data: impl PinInit<T::VmBoData>,
    ) -> Result<ARef<GpuVmBo<T>>, AllocError> {
        Ok(GpuVmBoAlloc::new(self, obj, data)?.obtain())
    }

    /// Clean up buffer objects that are no longer used.
    #[inline]
    pub fn deferred_cleanup(&self) {
        // SAFETY: This GPUVM uses immediate mode.
        unsafe { bindings::drm_gpuvm_bo_deferred_cleanup(self.as_raw()) }
    }

    /// Check if this GEM object is an external object for this GPUVM.
    #[inline]
    pub fn is_extobj(&self, obj: &T::Object) -> bool {
        // SAFETY: We may call this with any GPUVM and GEM object.
        unsafe { bindings::drm_gpuvm_is_extobj(self.as_raw(), obj.as_raw()) }
    }

    /// HACK: added for the asahi downstream driver. Validate a range against the immutable
    /// address-space bounds and the kernel cutout.
    pub fn range_valid(&self, addr: u64, range: u64) -> bool {
        // SAFETY: The borrowed VM retains its immutable geometry.
        unsafe { bindings::drm_gpuvm_range_valid(self.as_raw(), addr, range) }
    }

    /// HACK: added for compatibility with asahi downstream driver. Only safe to use
    /// when the gpuvm user forgot the UniqueRefGpuVm it got from `GpuVm::new()`.
    pub unsafe fn exec_lock<'a>(
        &'a self,
        obj: Option<&T::Object>,
        interruptible: bool,
    ) -> Result<GpuVmExec<T>> {
        // Do not try to lock the object if it is internal (since it is already locked).
        let is_ext = obj.map(|a| self.is_extobj(a)).unwrap_or(false);

        // vm_exec needs to be pinned, so stick it in a Box.
        let mut vm_exec = KBox::init(
            init!(bindings::drm_gpuvm_exec {
                vm: self.as_raw(),
                // Two GEM objects can share one reservation (a dma-buf imported again while
                // the closed object still has mappings); lock it once, as C drm_exec users do.
                flags: bindings::DRM_EXEC_IGNORE_DUPLICATES
                    | if interruptible {
                        bindings::DRM_EXEC_INTERRUPTIBLE_WAIT
                    } else {
                        0
                    },
                exec: Default::default(),
                extra: match (is_ext, obj) {
                    (true, Some(obj)) => bindings::drm_gpuvm_exec__bindgen_ty_1 {
                        fn_: Some(exec_lock_gem_object),
                        priv_: obj.as_raw() as *mut _,
                    },
                    _ => Default::default(),
                },
                num_fences: 0,
            }),
            GFP_KERNEL,
        )?;

        // SAFETY: The object is valid and was initialized above. On failure,
        // `drm_gpuvm_exec_lock()` has already finalized the exec context, so the box only needs to
        // be freed, which dropping it does.
        to_result(unsafe { bindings::drm_gpuvm_exec_lock(&mut *vm_exec) })?;

        // Build the guard, and take its VM reference, only once the locks are held: its `Drop`
        // unlocks them.
        Ok(GpuVmExec {
            vm_exec,
            unique_gpuvm: UniqueRefGpuVm(self.into()),
        })
    }

    /// Free this GPUVM.
    ///
    /// # Safety
    ///
    /// Called when refcount hits zero.
    unsafe extern "C" fn vm_free(me: *mut bindings::drm_gpuvm) {
        // SAFETY: Caller passes a pointer to the `drm_gpuvm` in a `GpuVm<T>`.
        let me = unsafe { kernel::container_of!(Opaque::cast_from(me), Self, vm).cast_mut() };
        // SAFETY: By type invariants we can free it when refcount hits zero.
        drop(unsafe { KBox::from_raw(me) })
    }

    #[inline]
    fn raw_resv(&self) -> *mut bindings::dma_resv {
        // SAFETY: `r_obj` is immutable and valid for duration of GPUVM.
        unsafe { (*(*self.as_raw()).r_obj).resv }
    }
}

impl<T: DriverGpuVm> GpuVm<T> {
    /// HACK: added for the asahi downstream driver. Lock only the VM's shared reservation.
    ///
    /// This takes the same shared reservation as [`GpuVm::exec_lock`], but does not lock
    /// external BO reservations or allocate an exec context. The returned guard exposes only the
    /// driver data, not GPUVA operations requiring those other reservations. Callers must not
    /// already hold a reservation lock.
    pub fn lock_inner(&self) -> GpuVmInnerGuard<'_, T> {
        // SAFETY: The borrowed GPUVM retains `r_obj` and its reservation for the guard's lifetime.
        // A non-interruptible single-reservation lock without a ww context cannot fail.
        unsafe { bindings::dma_resv_lock(self.raw_resv(), ptr::null_mut()) };
        GpuVmInnerGuard {
            gpuvm: self,
            _not_send: NotThreadSafe,
        }
    }
}

impl<T: DriverGpuVm> GpuVm<T> {
    /// Set or clear `INVALIDATED` on the exact linked GPUVA `addr..addr + range`.
    ///
    /// # Safety
    ///
    /// The caller owns the VM reservation and keeps the exact logical VA linked.
    unsafe fn set_invalidated_locked(&self, addr: u64, range: u64, invalidated: bool) -> Result {
        if range == 0 || addr.checked_add(range).is_none() {
            return Err(EINVAL);
        }
        // SAFETY: The caller owns the VM reservation and keeps the VA linked.
        let raw = unsafe { bindings::drm_gpuva_find(self.as_raw(), addr, range) };
        if raw.is_null() {
            return Err(EINVAL);
        }
        let flag = u32::from(GpuVaFlags::from(GpuVaFlag::INVALIDATED));
        // SAFETY: Reservation ownership excludes all concurrent VA mutation.
        unsafe {
            if (*raw).va.addr != addr || (*raw).va.range != range {
                return Err(EINVAL);
            }
            if invalidated {
                (*raw).flags |= flag;
            } else {
                (*raw).flags &= !flag;
            }
        }
        Ok(())
    }

    /// HACK: added for the asahi downstream driver. Try to lock only the VM reservation, without
    /// allocation or waiting.
    ///
    /// The guard protects VM metadata and private BOs, not external backing. Callers must not
    /// already hold a reservation lock.
    pub fn try_lock_private(&self) -> Option<PrivateGpuVmGuard<'_, T>> {
        let resv = self.raw_resv();
        // SAFETY: The reservation outlives the returned borrow and guard.
        if !unsafe { bindings::dma_resv_trylock(resv) } {
            return None;
        }
        Some(PrivateGpuVmGuard {
            gpuvm: self,
            resv,
            _not_send: NotThreadSafe,
        })
    }
}

impl<T: DriverGpuVm> GpuVmExec<T> {
    /// HACK: added for the asahi downstream driver. Mark the exact retained logical mapping after
    /// page-table restoration.
    pub fn set_invalidated(&mut self, addr: u64, range: u64, invalidated: bool) -> Result {
        // SAFETY: The exec guard owns the VM reservation.
        unsafe {
            self.unique_gpuvm
                .set_invalidated_locked(addr, range, invalidated)
        }
    }
}

/// HACK: added for the asahi downstream driver. Logical binding retained independently of
/// hardware page-table residency.
///
/// Owns a GEM reference and copied metadata, never a [`GpuVmBo`] reference. Driver metadata
/// cloning must not introduce a reservation-taking destructor.
pub struct MappingSnapshot<T: DriverGpuVm>
where
    T::Object: AlwaysRefCounted,
{
    /// The mapped object.
    pub object: ARef<T::Object>,
    /// Start of the mapping in the GPU address space.
    pub addr: u64,
    /// Length of the mapping.
    pub range: u64,
    /// Offset of the mapping in the object.
    pub offset: u64,
    /// Flags of the GPUVA.
    pub flags: GpuVaFlags,
    /// Driver metadata of the GPUVA.
    pub inner: T::VaData,
}

/// HACK: added for the asahi downstream driver. Nonblocking reservation ownership for pressure
/// scans of private objects.
pub struct PrivateGpuVmGuard<'a, T: DriverGpuVm> {
    gpuvm: &'a GpuVm<T>,
    resv: *mut bindings::dma_resv,
    // DMA reservation ownership must remain on the acquiring task.
    _not_send: NotThreadSafe,
}

impl<T: DriverGpuVm> PrivateGpuVmGuard<'_, T> {
    /// Visit each private BO with linked logical mappings once, without allocating or retaining
    /// references. Returning false stops the walk. Contended GEM lists are skipped.
    ///
    /// # Safety
    ///
    /// The callback must not insert or remove logical mappings or modify either GPUVA list. This
    /// reservation keeps their linked BO/GEM references alive for the complete callback. The
    /// callback must not retain a borrowed BO or acquire its GEM GPUVA mutex recursively.
    pub unsafe fn for_each_private_bo(
        &mut self,
        mut visit: impl FnMut(&mut Self, &GpuVmBo<T>) -> Result<bool>,
    ) -> Result {
        let vm = self.gpuvm.as_raw();
        // SAFETY: The VM is live while this guard borrows it.
        if unsafe { (*vm).flags } & bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE == 0 {
            return Err(EINVAL);
        }
        // SAFETY: The VM reservation protects this ordered mapping list. The callback does not
        // mutate its links, and no borrowed or owned reference outlives the guard.
        let head = unsafe { &raw const (*vm).rb.list };
        // SAFETY: As above.
        let mut node = unsafe { (*head).next };
        while node != head.cast_mut() {
            // SAFETY: `node` is a live list entry embedded in a `drm_gpuva`.
            let (next, raw) = unsafe {
                (
                    (*node).next,
                    kernel::container_of!(node, bindings::drm_gpuva, rb.entry),
                )
            };
            // SAFETY: The linked GPUVA is live under the reservation.
            let va = unsafe { &*raw };
            if !va.gem.obj.is_null() && !va.vm_bo.is_null() {
                // SAFETY: The linked VA retains this driver's object.
                let object = unsafe { <T::Object as IntoGEMObject>::from_raw(va.gem.obj) };
                if !self.gpuvm.is_extobj(object) {
                    // SAFETY: The GEM object is live; its GPUVA list mutex is initialized.
                    let lock = unsafe { &raw mut (*va.gem.obj).gpuva.lock };
                    // Never block direct reclaim on a GEM-list owner. The canonical alias is
                    // the list's first-linked node, not its lowest address or current residency.
                    // SAFETY: `lock` is a valid mutex.
                    if unsafe { bindings::mutex_trylock(lock) } != 0 {
                        // SAFETY: The list mutex protects these link reads.
                        let canonical = unsafe {
                            (*va.vm_bo).list.gpuva.next == (&raw const va.gem.entry).cast_mut()
                        };
                        // SAFETY: Locked just above.
                        unsafe { bindings::mutex_unlock(lock) };
                        if canonical {
                            // SAFETY: The VA's `vm_bo` is a live `GpuVmBo<T>` of this VM.
                            let bo = unsafe { GpuVmBo::<T>::from_raw(va.vm_bo) };
                            if !visit(self, bo)? {
                                break;
                            }
                        }
                    }
                }
            }
            node = next;
        }
        Ok(())
    }

    /// Mark and visit every logical alias of this private BO without a snapshot allocation.
    ///
    /// GEM-list locking is nonblocking and held only while reading a cursor, never during the
    /// callback. Failure (including contention) can leave a prefix marked `INVALIDATED`; callers
    /// must retain backing unless this method succeeds for the entire association.
    ///
    /// # Safety
    ///
    /// The callback must not insert or remove logical mappings, alter list links, or release the
    /// BO's SG/backing lease. The BO must remain linked throughout the walk. Exclude new jobs and
    /// binding edits for the whole invalidation and backing-release transaction.
    pub unsafe fn invalidate_private_bo_mappings(
        &mut self,
        bo: &GpuVmBo<T>,
        mut visit: impl FnMut(&mut Self, u64, u64) -> Result,
    ) -> Result {
        let vm = self.gpuvm.as_raw();
        let raw_bo = bo.as_raw();
        // SAFETY: `bo` and the VM are live while borrowed.
        if unsafe { (*raw_bo).vm } != vm
            || self.gpuvm.is_extobj(bo.obj())
            || unsafe { (*vm).flags } & bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE == 0
        {
            return Err(EINVAL);
        }
        let flag = u32::from(GpuVaFlags::from(GpuVaFlag::INVALIDATED));
        // SAFETY: `bo` is live while borrowed.
        let head = unsafe { &raw const (*raw_bo).list.gpuva };
        // SAFETY: The GEM object of a live BO is live; its GPUVA list mutex is initialized.
        let lock = unsafe { &raw mut (*(*raw_bo).obj).gpuva.lock };
        // SAFETY: `lock` is a valid mutex.
        if unsafe { bindings::mutex_trylock(lock) } == 0 {
            return Err(EAGAIN);
        }
        // SAFETY: The list mutex protects the link read.
        let mut node = unsafe { (*head).next };
        // SAFETY: Locked just above.
        unsafe { bindings::mutex_unlock(lock) };
        while node != head.cast_mut() {
            // SAFETY: `lock` is a valid mutex.
            if unsafe { bindings::mutex_trylock(lock) } == 0 {
                return Err(EAGAIN);
            }
            // SAFETY: The list mutex protects link reads; this reservation and the caller's
            // no-unlink contract keep current/next and their association alive after unlocking.
            let (next, raw) = unsafe {
                (
                    (*node).next,
                    kernel::container_of!(node, bindings::drm_gpuva, gem.entry),
                )
            };
            // SAFETY: Locked just above.
            unsafe { bindings::mutex_unlock(lock) };
            // SAFETY: As above.
            let va = unsafe { &mut *raw };
            if va.vm != vm
                || va.vm_bo != raw_bo
                || va.va.range == 0
                || va.va.addr.checked_add(va.va.range).is_none()
            {
                return Err(EINVAL);
            }
            // Flags are serialized by the VM reservation, independently of GEM's list mutex.
            va.flags |= flag;
            visit(self, va.va.addr, va.va.range)?;
            node = next;
        }
        Ok(())
    }

    /// Retain the next private mapping containing all `required_flags`.
    ///
    /// `cursor` is an exclusive end address and advances past skipped mappings as well. No
    /// collection is allocated; only the returned object is retained. Callers must drop this
    /// reservation guard before releasing that reference or acquiring backing. To cover a stable
    /// set across calls, separately exclude logical binding edits (including retirement) for the
    /// whole walk.
    pub fn next_private_mapping(
        &self,
        cursor: &mut u64,
        required_flags: GpuVaFlags,
    ) -> Result<Option<MappingSnapshot<T>>>
    where
        T::VaData: Clone,
        T::Object: AlwaysRefCounted,
    {
        while *cursor != u64::MAX {
            // SAFETY: The VM reservation serializes mutation of its VA tree.
            let raw = unsafe {
                bindings::drm_gpuva_find_first(self.gpuvm.as_raw(), *cursor, u64::MAX - *cursor)
            };
            if raw.is_null() {
                return Ok(None);
            }
            // SAFETY: `raw` stays linked and alive while this guard is held.
            let va = unsafe { &*raw };
            let end = va.va.addr.checked_add(va.va.range).ok_or(EINVAL)?;
            if end <= *cursor {
                return Err(EINVAL);
            }
            *cursor = end;
            let required = u32::from(required_flags);
            if va.flags & required != required
                || va.gem.obj.is_null()
                || va.vm_bo.is_null()
            {
                continue;
            }
            // SAFETY: The linked VA retains this driver's object.
            let obj = unsafe { <T::Object as IntoGEMObject>::from_raw(va.gem.obj) };
            if self.gpuvm.is_extobj(obj) {
                continue;
            }
            // SAFETY: Driver-owned mappings are `GpuVa<T>`. The kernel cutout has no GEM object
            // and was skipped above.
            let typed = unsafe { GpuVa::<T>::from_raw(raw) };
            return Ok(Some(MappingSnapshot {
                object: obj.into(),
                addr: va.va.addr,
                range: va.va.range,
                offset: va.gem.offset,
                flags: GpuVaFlags::from(va.flags),
                inner: typed.data().clone(),
            }));
        }
        Ok(None)
    }
}

impl<T: DriverGpuVm> Deref for PrivateGpuVmGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        // SAFETY: The reservation serializes all access to the VM's driver data.
        unsafe { &*self.gpuvm.data.get() }
    }
}

impl<T: DriverGpuVm> DerefMut for PrivateGpuVmGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: This guard has exclusive reservation ownership.
        unsafe { &mut *self.gpuvm.data.get() }
    }
}

impl<T: DriverGpuVm> Drop for PrivateGpuVmGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY: `try_lock_private()` acquired exactly this reservation.
        unsafe { bindings::dma_resv_unlock(self.resv) };
    }
}

/// HACK: added for the asahi downstream driver. Access to the VM driver data under its shared
/// reservation, without external BO locks.
pub struct GpuVmInnerGuard<'a, T: DriverGpuVm> {
    gpuvm: &'a GpuVm<T>,
    // Reservation locks must be released by the task that acquired them.
    _not_send: NotThreadSafe,
}

impl<T: DriverGpuVm> GpuVmInnerGuard<'_, T> {
    /// Run immediate-mode unmap steps without allocating an exec context.
    ///
    /// # Safety
    ///
    /// The driver's unmap and remap callbacks must touch only VM metadata and translations
    /// protected by this reservation. They must not access external BO backing that requires its
    /// own reservation. Any split-node storage must already be owned by `context`.
    pub unsafe fn sm_unmap_inner(
        &mut self,
        addr: u64,
        length: u64,
        context: &mut T::SmContext<'_>,
    ) -> Result {
        // SAFETY: This guard retains the VM and its shared reservation.
        if unsafe { (*self.gpuvm.as_raw()).flags }
            & bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE
            == 0
        {
            return Err(EINVAL);
        }
        // INVARIANT: The shared reservation held by this guard excludes every other
        // `UniqueRefGpuVm`, as for `GpuVm::exec_lock()`.
        let mut unique = UniqueRefGpuVm(self.gpuvm.into());
        unique.sm_unmap(addr, length, context)
    }

    /// Remove every driver GPUVA of this immediate-mode VM, preserving its kernel cutout.
    ///
    /// `unmap` must remove the GPU translation of each complete GPUVA before returning success.
    /// Only the VM's shared reservation is held: no external BO reservation is taken and no split
    /// node or operation list is allocated. Call [`GpuVm::deferred_cleanup`] after releasing this
    /// guard, including when a callback failed after earlier mappings were removed.
    pub fn unmap_all(&mut self, unmap: impl FnMut(&mut T, &GpuVa<T>) -> Result) -> Result {
        self.unmap_matching(None, |_| true, unmap)
    }

    /// Allocation-free whole-node removal for one GEM object in an immediate-mode VM.
    ///
    /// Uses the same translation-removal and deferred-put contract as [`Self::unmap_all`]. This
    /// scans the VM list and can be used when the per-object operation list is unavailable.
    pub fn unmap_object(
        &mut self,
        object: &T::Object,
        unmap: impl FnMut(&mut T, &GpuVa<T>) -> Result,
    ) -> Result {
        self.unmap_object_where(object, |_| true, unmap)
    }

    /// Remove only mappings of an object selected by `matches`, without allocating.
    ///
    /// This scans the whole VM and uses the same translation-removal and deferred-put contract as
    /// [`Self::unmap_all`]. If a callback fails, mappings already visited remain removed.
    pub fn unmap_object_where(
        &mut self,
        object: &T::Object,
        matches: impl FnMut(&GpuVa<T>) -> bool,
        unmap: impl FnMut(&mut T, &GpuVa<T>) -> Result,
    ) -> Result {
        self.unmap_matching(Some(object), matches, unmap)
    }

    fn unmap_matching(
        &mut self,
        object: Option<&T::Object>,
        mut matches: impl FnMut(&GpuVa<T>) -> bool,
        mut unmap: impl FnMut(&mut T, &GpuVa<T>) -> Result,
    ) -> Result {
        let vm = self.gpuvm.as_raw();
        // SAFETY: The guard owns this live VM's shared reservation, and the VM is in immediate
        // mode, so unlinking takes the GEM GPUVA mutex independently of its reservation. Every
        // listed GPUVA other than the kernel cutout is a `GpuVa<T>` owned by this VM.
        unsafe {
            if (*vm).flags & bindings::drm_gpuvm_flags_DRM_GPUVM_IMMEDIATE_MODE == 0 {
                return Err(EINVAL);
            }
            let head = &raw mut (*vm).rb.list;
            let mut entry = (*head).next;
            while entry != head {
                let va = kernel::container_of!(entry, bindings::drm_gpuva, rb.entry);
                // Save the successor before removing and freeing this GPUVA.
                entry = (*entry).next;
                // This node is embedded in the C GPUVM, not a driver `GpuVa<T>`.
                if ptr::eq(va, &raw const (*vm).kernel_alloc_node) {
                    continue;
                }
                if object.is_some_and(|object| (*va).gem.obj != object.as_raw()) {
                    continue;
                }
                let typed_va = GpuVa::<T>::from_raw(va);
                if !matches(typed_va) {
                    continue;
                }
                unmap(&mut *self.gpuvm.data.get(), typed_va)?;
                // Remove it as `OpUnmap::remove()` does.
                let op = bindings::drm_gpuva_op_unmap { va, keep: false };
                bindings::drm_gpuva_unmap(&op);
                bindings::drm_gpuva_unlink_defer(va);
                drop(GpuVaRemoved::<T>::from_raw(va));
            }
        }
        Ok(())
    }
}

impl<T: DriverGpuVm> Deref for GpuVmInnerGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        // SAFETY: The shared reservation excludes every other access to the driver data,
        // including `GpuVm::exec_lock()` and its callbacks.
        unsafe { &*self.gpuvm.data.get() }
    }
}

impl<T: DriverGpuVm> DerefMut for GpuVmInnerGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: As above, with exclusive access to this guard.
        unsafe { &mut *self.gpuvm.data.get() }
    }
}

impl<T: DriverGpuVm> Drop for GpuVmInnerGuard<'_, T> {
    fn drop(&mut self) {
        // SAFETY: This non-Send guard owns the reservation on this task, and its GPUVM borrow
        // keeps `r_obj` alive until after the unlock.
        unsafe { bindings::dma_resv_unlock(self.gpuvm.raw_resv()) };
    }
}

/// The manager for a GPUVM.
pub trait DriverGpuVm: Sized + Send + Sync {
    /// Parent `Driver` for this object.
    type Driver: drm::Driver;

    /// The kind of GEM object stored in this GPUVM.
    type Object: drm::driver::AllocImpl<Driver = Self::Driver> + Send + Sync;

    /// Data stored with each [`struct drm_gpuva`](struct@GpuVa).
    ///
    /// Only `Send` is required: the data has a single owner at all times, moving
    /// between threads by value (handed back as a [`GpuVaRemoved`]) but never
    /// accessed by two threads concurrently.
    type VaData: Send;

    /// Data stored with each [`struct drm_gpuvm_bo`](struct@GpuVmBo).
    type VmBoData: Send + Sync;

    /// The private data passed to callbacks.
    type SmContext<'ctx>
    where
        Self: 'ctx;

    /// Indicates that a new mapping should be created.
    fn sm_step_map<'op, 'ctx>(
        &mut self,
        op: OpMap<'op, Self>,
        context: &mut Self::SmContext<'ctx>,
    ) -> Result<OpMapped<'op, Self>, Error>;

    /// Indicates that an existing mapping should be removed.
    fn sm_step_unmap<'op, 'ctx>(
        &mut self,
        op: OpUnmap<'op, Self>,
        context: &mut Self::SmContext<'ctx>,
    ) -> Result<OpUnmapped<'op, Self>, Error>;

    /// Indicates that an existing mapping should be split up.
    fn sm_step_remap<'op, 'ctx>(
        &mut self,
        op: OpRemap<'op, Self>,
        context: &mut Self::SmContext<'ctx>,
    ) -> Result<OpRemapped<'op, Self>, Error>;
}

/// HACK: added for compatibility with asahi downstream driver.
pub struct GpuVmExec<T: DriverGpuVm> {
    vm_exec: KBox<bindings::drm_gpuvm_exec>,
    /// alternative way to get UniqueRefGpuVm when using `GpuVm::exec_lock()`.
    pub unique_gpuvm: UniqueRefGpuVm<T>,
}

impl<T: DriverGpuVm> Drop for GpuVmExec<T> {
    fn drop(&mut self) {
        // SAFETY: We hold the lock, so it's safe to unlock
        unsafe {
            bindings::drm_gpuvm_exec_unlock(&mut *self.vm_exec);
        }
    }
}

/// The core of the DRM GPU VA manager.
///
/// This object is a unique reference to the VM that can access the interval tree and the Rust
/// `data` field.
///
/// # Invariants
///
/// Each `GpuVm` instance has at most one `UniqueRefGpuVm` reference.
// `Send`/`Sync` derive from `ARef<GpuVm<T>>`; the trait bounds make them correct for the unique
// handle's `&mut T` access.
pub struct UniqueRefGpuVm<T: DriverGpuVm>(ARef<GpuVm<T>>);

impl<T: DriverGpuVm> UniqueRefGpuVm<T> {
    /// Access the data owned by this `UniqueRefGpuVm` immutably.
    #[inline]
    pub fn data_ref(&self) -> &T {
        // SAFETY: By the type invariants we may access `data`.
        unsafe { &*self.0.data.get() }
    }

    /// Access the data owned by this `UniqueRefGpuVm` mutably.
    #[inline]
    pub fn data(&mut self) -> &mut T {
        // SAFETY: By the type invariants we may access `data`.
        unsafe { &mut *self.0.data.get() }
    }

    /// Get the [`GpuVmBo`] for this gem object if it exists.
    pub fn find_bo(&mut self, obj: &T::Object) -> Option<ARef<GpuVmBo<T>>> {
        obj.lock_gpuva();
        // SAFETY: drm_gem_object.gpuva.lock was just locked.
        let ptr = unsafe {
            bindings::drm_gpuvm_bo_find(self.as_raw() as *mut _, obj.as_raw() as *const _ as *mut _)
        };
        obj.unlock_gpuva();
        if ptr.is_null() {
            None
        } else {
            // SAFETY: we just checked that ptr is non-null
            let nonnull = unsafe { NonNull::new_unchecked(ptr.cast()) };

            // INVARIANTS: `drm_gpuvm_bo_obtain_prealloc` ensures that the bo is in the GEM list.
            // SAFETY: We received one refcount from `drm_gpuvm_bo_obtain_prealloc`.
            Some(unsafe { ARef::<GpuVmBo<T>>::from_raw(nonnull) })
        }
    }
}

impl<T: DriverGpuVm> Deref for UniqueRefGpuVm<T> {
    type Target = GpuVm<T>;

    #[inline]
    fn deref(&self) -> &GpuVm<T> {
        &self.0
    }
}
