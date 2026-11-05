// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM Scheduler
//!
//! C header: [`include/drm/gpu_scheduler.h`](../../../../include/drm/gpu_scheduler.h)

use crate::{
    bindings, device,
    dma_fence::*,
    error::{to_result, Result},
    prelude::*,
    sync::{Arc, UniqueArc},
    time::{self, msecs_to_jiffies},
};
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::ptr::{addr_of, addr_of_mut};

/// Scheduler status after timeout recovery
#[repr(u32)]
pub enum Status {
    /// Device recovered from the timeout and can execute jobs again
    Nominal = bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_RESET,
    /// Device is no longer available
    NoDevice = bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_ENODEV,
    /// Reinsert the job and rearm its timeout without resetting the scheduler.
    NoHang = bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_NO_HANG,
}

/// Scheduler priorities
#[repr(i32)]
pub enum Priority {
    /// Low userspace priority
    Low = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_LOW,
    /// Normal userspace priority
    Normal = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_NORMAL,
    /// High userspace priority
    High = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_HIGH,
    /// Kernel priority (highest)
    Kernel = bindings::drm_sched_priority_DRM_SCHED_PRIORITY_KERNEL,
}

/// Trait to be implemented by driver job objects.
pub trait JobImpl: Sized {
    /// State owned exclusively by the timeout worker. It must not borrow the
    /// mutable submission state: timeout can run before `run` returns.
    type TimeoutData;

    /// Snapshot or independently retain timeout state before scheduler ownership.
    fn timeout_data(&self) -> Self::TimeoutData;

    /// Module implementing the callbacks. Its caller must retain the module
    /// through entity destruction; the core pins detached kill-job callbacks.
    const MODULE: Option<&'static crate::ThisModule> = None;

    /// Handle a scoped timeout without resetting the C scheduler.
    ///
    /// The backend must settle/quarantine its exact hardware work, or retain
    /// that owner until an in-progress host transaction can classify it. It
    /// must not invoke drm_sched_stop/start or replay scheduler jobs. The core
    /// reinserts the detached guilty job through NO_HANG, preserving every
    /// parent callback and the backend's actual success/error fence result.
    const MANAGED_TIMEOUT_RECOVERY: bool = false;

    /// Reconsider a timeout before stopping the scheduler or settling fences.
    /// Returning true asks the core to reinsert the retained job and rearm.
    /// The backend must not stop/start the scheduler from this hook.
    fn false_timeout(_data: &mut Self::TimeoutData, _finished: bool) -> bool { false }


    /// Called when the scheduler is considering scheduling this job next, to get another Fence
    /// for this job to block on. Once it returns None, run() may be called.
    fn prepare(_job: &mut Self) -> Option<Fence> {
        None // Equivalent to NULL function pointer
    }

    /// Called to execute the job once all of the dependencies have been resolved. This may be
    /// called multiple times, if timed_out() has happened and drm_sched_job_recovery() decides
    /// to try it again.
    fn run(job: &mut Self) -> Result<Option<Fence>>;

    /// Called when a job has taken too long to execute, to trigger GPU recovery.
    ///
    /// This method is called in a workqueue context.
    fn timed_out(data: &mut Self::TimeoutData, finished: bool) -> Status;

    /// Called for remaining jobs in drm_sched_fini() to ensure the job's fences
    /// get signalled before the scheduler is torn down.
    fn cancel(job: &mut Self);
}

unsafe extern "C" fn prepare_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
    _s_entity: *mut bindings::drm_sched_entity,
) -> *mut bindings::dma_fence {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // SAFETY: prepare/run are serialized by submit work. Only borrow inner;
    // the timeout worker independently owns timeout_data and C owns job.
    match T::prepare(unsafe { &mut *addr_of_mut!((*p).inner) }) {
        None => core::ptr::null_mut(),
        Some(fence) => fence.into_raw(),
    }
}

unsafe extern "C" fn run_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
) -> *mut bindings::dma_fence {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // SAFETY: submit work owns inner until return, even while timeout work
    // accesses the disjoint timeout_data field. Do not borrow the whole Job.
    match T::run(unsafe { &mut *addr_of_mut!((*p).inner) }) {
        Err(e) => e.to_ptr(),
        Ok(None) => core::ptr::null_mut(),
        Ok(Some(fence)) => fence.into_raw(),
    }
}

unsafe extern "C" fn timedout_job_cb<T: JobImpl>(
    sched_job: *mut bindings::drm_sched_job,
) -> bindings::drm_gpu_sched_stat {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // The C timeout worker owns this job outside pending_list until return.
    // Free work cannot release it even if its parent signals concurrently.
    let finished = unsafe { addr_of_mut!((*(*sched_job).s_fence).finished) };
    let is_finished = || unsafe { bindings::dma_fence_get_status(finished) != 0 };
    if T::MANAGED_TIMEOUT_RECOVERY {
        if is_finished() {
            return bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_NO_HANG;
        }
    }
    // SAFETY: the C core retains the detached job and serializes its timeout
    // worker. This field is disjoint from inner, which run may still mutate.
    let data = unsafe { &mut *addr_of_mut!((*p).timeout_data) };
    if T::false_timeout(data, is_finished()) {
        return bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_NO_HANG;
    }
    let status = T::timed_out(data, is_finished());
    if T::MANAGED_TIMEOUT_RECOVERY {
        // This is scoped fence handling, not C reset recovery. stop would
        // detach and discard healthy parents; start would cancel those jobs.
        // NO_HANG reinserts the exact job and queues free work if it completed
        // during the callback. It does not change the parent's error or replay
        // work. A deferred exact owner remains eligible for its real callback.
        bindings::drm_gpu_sched_stat_DRM_GPU_SCHED_STAT_NO_HANG
    } else {
        status as bindings::drm_gpu_sched_stat
    }
}

unsafe extern "C" fn free_job_cb<T: JobImpl>(sched_job: *mut bindings::drm_sched_job) {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // Fault-only ownership witness: this callback is where the core returns
    // ownership of an errored job. A signalled fence alone is not that proof.
    // SAFETY: A job reaching free_job has a live scheduler fence; its error
    // was fixed before signal and the core's completion callback has finished.
    if T::MANAGED_TIMEOUT_RECOVERY
        && unsafe { (*(*sched_job).s_fence).finished.error } == ETIMEDOUT.to_errno()
    {
        // Use numeric identity: the fmt adapter borrows arguments, and
        // Pointer formatting of that reference can print a stack address.
        pr_info!("DRM Rust scheduler: released timed-out job={:#x} context={} seqno={}\n",
            sched_job as usize,
            unsafe { (*(*sched_job).s_fence).finished.context },
            unsafe { (*(*sched_job).s_fence).finished.seqno });
    }
    // Convert the job back to a Box and drop it
    // SAFETY: All of our Job<T>s are created inside a box.
    unsafe { drop(KBox::from_raw(p)) };
}

unsafe extern "C" fn cancel_job_cb<T: JobImpl>(sched_job: *mut bindings::drm_sched_job) {
    // SAFETY: All of our jobs are Job<T>.
    let p = unsafe { crate::container_of!(sched_job, Job<T>, job) as *mut Job<T> };

    // fini has stopped/synchronized submission and timeout workers. A parent
    // callback can still run on the device completion thread. The scheduler
    // fence retains the parent reference while we remove its callback under
    // the parent's lock; that also waits for an already-running callback.
    let s_fence = unsafe { (*sched_job).s_fence };
    let parent = unsafe { (*s_fence).parent };
    let removed_parent_callback = if !parent.is_null() {
        // SAFETY: run_job installed (or attempted to install on an already
        // signaled parent) this callback before fini synchronized submit work.
        unsafe { bindings::dma_fence_remove_callback(parent, addr_of_mut!((*sched_job).cb)) }
    } else { false };
    let finish_here = removed_parent_callback || (parent.is_null()
        && unsafe { bindings::dma_fence_get_status(addr_of_mut!((*s_fence).finished)) == 0 });
    // Null/error run_job calls job_done before submit work returns, so its
    // finished fence is already signaled. NULL+unfinished can instead survive
    // a legacy backend's prior reset stop: C stop dropped the parent and its
    // credits, but not its score. Do not subtract those credits twice.

    // SAFETY: cancellation cannot race a scheduler callback using this job
    // after the removal/parent-lock synchronization above. The backend must
    // retain any still-hardware-owned resources independently of Job storage.
    T::cancel(unsafe { &mut *addr_of_mut!((*p).inner) });

    if finish_here {
        // Match drm_sched_job_done's accounting exactly once, only when its
        // callback did not do it. Submission/free work is stopped, so fini
        // itself unlinks and frees this job; no workqueue rearm is appropriate.
        let sched = unsafe { (*sched_job).sched };
        unsafe {
            if removed_parent_callback {
                bindings::atomic_sub((*sched_job).credits as i32,
                    addr_of_mut!((*sched).credit_count));
            }
            bindings::atomic_sub(1, (*sched).score);
        }
        let fence = unsafe { Fence::get_raw(addr_of_mut!((*s_fence).finished)) };
        fence.set_error(ECANCELED);
        let _ = fence.signal();
    }
    // Do not clear/put parent here: s_fence still owns that reference and its
    // normal release path drops it. A completed callback's result is retained.
}

/// A DRM scheduler job.
pub struct Job<T: JobImpl> {
    job: bindings::drm_sched_job,
    inner: T,
    timeout_data: T::TimeoutData,
    // Entity destruction can leave kill_jobs_work queued on the system
    // workqueue or waiting for a dependency. That work still dereferences
    // job->sched->ops after drm_sched_fini has stopped the scheduler workers.
    // Retain only the allocation: retaining SchedulerOwner here would keep
    // fini from running while pending jobs still need its cancellation.
    _scheduler: Arc<SchedulerInner<T>>,
}

impl<T: JobImpl> Job<T> {
    /// Returns whether an armed job has finished, successfully or not.
    pub fn is_finished(&self) -> bool {
        if self.job.sched.is_null() {
            return false;
        }
        // SAFETY: An armed job has an initialized scheduler fence, retained
        // for its lifetime, including while the timeout worker owns the job.
        unsafe { bindings::dma_fence_get_status(addr_of_mut!((*self.job.s_fence).finished)) != 0 }
    }
}

impl<T: JobImpl> Deref for Job<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: JobImpl> DerefMut for Job<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: JobImpl> Drop for Job<T> {
    fn drop(&mut self) {
        // SAFETY: At this point the job has either been submitted and this is being called from
        // `free_job_cb` above, or it hasn't and it is safe to call `drm_sched_job_cleanup`.
        unsafe { bindings::drm_sched_job_cleanup(&mut self.job) };
    }
}

/// A pending DRM scheduler job (not yet armed)
pub struct PendingJob<'a, T: JobImpl>(KBox<Job<T>>, PhantomData<&'a T>);

impl<'a, T: JobImpl> PendingJob<'a, T> {
    /// Add a fence as a dependency to the job
    pub fn add_dependency(&mut self, fence: Fence) -> Result {
        // SAFETY: C call with correct arguments
        to_result(unsafe {
            bindings::drm_sched_job_add_dependency(&mut self.0.job, fence.into_raw())
        })
    }

    /// Arm the job to make it ready for execution
    pub fn arm(mut self) -> ArmedJob<'a, T> {
        // SAFETY: C call with correct arguments
        unsafe { bindings::drm_sched_job_arm(&mut self.0.job) };
        ArmedJob(self.0, PhantomData)
    }
}

impl<'a, T: JobImpl> Deref for PendingJob<'a, T> {
    type Target = Job<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, T: JobImpl> DerefMut for PendingJob<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// An armed DRM scheduler job (not yet submitted)
pub struct ArmedJob<'a, T: JobImpl>(KBox<Job<T>>, PhantomData<&'a T>);

impl<'a, T: JobImpl> ArmedJob<'a, T> {
    /// Returns the job fences
    pub fn fences(&mut self) -> JobFences<'_> {
        // SAFETY: s_fence is always a valid drm_sched_fence pointer
        JobFences(unsafe { &mut *self.0.job.s_fence })
    }

    /// Push the job for execution into the scheduler
    pub fn push(self) {
        // After this point, the job is submitted and owned by the scheduler
        let ptr = match self {
            ArmedJob(job, _) => KBox::<Job<T>>::into_raw(job),
        };

        // SAFETY: We are passing in ownership of a valid Box raw pointer.
        unsafe { bindings::drm_sched_entity_push_job(addr_of_mut!((*ptr).job)) };
    }
}
impl<'a, T: JobImpl> Deref for ArmedJob<'a, T> {
    type Target = Job<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, T: JobImpl> DerefMut for ArmedJob<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Reference to the bundle of fences attached to a DRM scheduler job
pub struct JobFences<'a>(&'a mut bindings::drm_sched_fence);

impl<'a> JobFences<'a> {
    /// Returns a new reference to the job scheduled fence.
    pub fn scheduled(&mut self) -> Fence {
        // SAFETY: self.0.scheduled is always a valid fence
        unsafe { Fence::get_raw(&mut self.0.scheduled) }
    }

    /// Returns a new reference to the job finished fence.
    pub fn finished(&mut self) -> Fence {
        // SAFETY: self.0.finished is always a valid fence
        unsafe { Fence::get_raw(&mut self.0.finished) }
    }
}

struct EntityInner<T: JobImpl> {
    entity: bindings::drm_sched_entity,
    // TODO: Allow users to share guilty flag between entities
    sched: Arc<SchedulerOwner<T>>,
    guilty: bindings::atomic_t,
    _p: PhantomData<T>,
}

impl<T: JobImpl> Drop for EntityInner<T> {
    fn drop(&mut self) {
        // SAFETY: The EntityInner is initialized. Jobs removed from the entity
        // may still await kill_jobs_work; each retains the scheduler allocation.
        unsafe { bindings::drm_sched_entity_destroy(&mut self.entity) };
    }
}

// SAFETY: TODO
unsafe impl<T: JobImpl> Sync for EntityInner<T> {}
// SAFETY: TODO
unsafe impl<T: JobImpl> Send for EntityInner<T> {}

/// A DRM scheduler entity.
pub struct Entity<T: JobImpl>(Pin<KBox<EntityInner<T>>>);

impl<T: JobImpl> Entity<T> {
    /// Create a new scheduler entity.
    pub fn new(sched: &Scheduler<T>, priority: Priority) -> Result<Self> {
        let mut entity: KBox<MaybeUninit<EntityInner<T>>> =
            KBox::new_uninit(GFP_KERNEL | __GFP_ZERO)?;

        let mut sched_ptr = &sched.0.allocation.sched as *const _ as *mut _;

        // SAFETY: The Box is allocated above and valid.
        unsafe {
            bindings::drm_sched_entity_init(
                addr_of_mut!((*entity.as_mut_ptr()).entity),
                priority as _,
                &mut sched_ptr,
                1,
                addr_of_mut!((*entity.as_mut_ptr()).guilty),
            )
        };

        // SAFETY: The Box is allocated above and valid.
        unsafe { addr_of_mut!((*entity.as_mut_ptr()).sched).write(sched.0.clone()) };

        // SAFETY: entity is now initialized.
        Ok(Self(Pin::from(unsafe { entity.assume_init() })))
    }

    /// Create a new job on this entity.
    ///
    /// The entity must outlive the pending job until it transitions into the submitted state,
    /// after which the scheduler owns it. Since jobs must be submitted in creation order,
    /// this requires a mutable reference to the entity, ensuring that only one new job can be
    /// in flight at once.
    pub fn new_job(&mut self, credits: u32, inner: T) -> Result<PendingJob<'_, T>> {
        let mut job: KBox<MaybeUninit<Job<T>>> = Box::new_uninit(GFP_KERNEL | __GFP_ZERO)?;
        let timeout_data = inner.timeout_data();

        // SAFETY: We hold a reference to the entity (which is a valid pointer),
        // and the job object was just allocated above.
        to_result(unsafe {
            bindings::drm_sched_job_init(
                addr_of_mut!((*job.as_mut_ptr()).job),
                &self.0.as_ref().get_ref().entity as *const _ as *mut _,
                credits,
                core::ptr::null_mut(),
                0,
            )
        })?;

        // SAFETY: The Box pointer is valid, and this initializes the inner member.
        unsafe { addr_of_mut!((*job.as_mut_ptr()).inner).write(inner) };
        // SAFETY: initialize the independent timeout worker's owned state.
        unsafe { addr_of_mut!((*job.as_mut_ptr()).timeout_data).write(timeout_data) };

        // SAFETY: The job allocation is valid and this initializes its last
        // field. Arc::clone is infallible and the entity still owns the live
        // scheduler, so every successfully initialized job retains its C
        // callback target before it can be armed or submitted.
        unsafe {
            addr_of_mut!((*job.as_mut_ptr())._scheduler)
                .write(self.0.as_ref().get_ref().sched.allocation.clone())
        };

        // SAFETY: All fields of the Job<T> are now initialized.
        Ok(PendingJob(unsafe { job.assume_init() }, PhantomData))
    }
}

/// DRM scheduler allocation, retained by live owners and deferred job callbacks.
pub struct SchedulerInner<T: JobImpl> {
    sched: bindings::drm_gpu_scheduler,
    _p: PhantomData<T>,
}

// Only public scheduler/entity handles keep this owner alive. Jobs retain
// SchedulerInner instead, so outstanding work cannot prevent finalization.
struct SchedulerOwner<T: JobImpl> {
    allocation: Arc<SchedulerInner<T>>,
    /// The driver's submission queue, if it supplied one. Fields drop after `Drop::drop`, so the
    /// queue is destroyed only after `drm_sched_fini` has stopped the scheduler's work on it.
    _submit_wq: Option<crate::workqueue::OwnedQueue>,
}

impl<T: JobImpl> Drop for SchedulerOwner<T> {
    fn drop(&mut self) {
        // SAFETY: fini stops submission, synchronizes the timeout worker and
        // cancels/frees remaining jobs through our synchronized cancel callback.
        // The C contract explicitly prohibits drm_sched_stop before fini.
        // Deferred entity-kill callbacks are on a different workqueue and may
        // survive this call. Their job guards retain allocation through cleanup.
        unsafe {
            bindings::drm_sched_fini(addr_of!(self.allocation.sched).cast_mut())
        };
    }
}

// SAFETY: TODO
unsafe impl<T: JobImpl> Sync for SchedulerInner<T> {}
// SAFETY: TODO
unsafe impl<T: JobImpl> Send for SchedulerInner<T> {}

/// A DRM Scheduler
pub struct Scheduler<T: JobImpl>(Arc<SchedulerOwner<T>>);

impl<T: JobImpl> Scheduler<T> {
    const OPS: bindings::drm_sched_backend_ops = bindings::drm_sched_backend_ops {
        prepare_job: Some(prepare_job_cb::<T>),
        run_job: Some(run_job_cb::<T>),
        timedout_job: Some(timedout_job_cb::<T>),
        free_job: Some(free_job_cb::<T>),
        cancel_job: Some(cancel_job_cb::<T>),
        // A terminal backend result must not orphan the detached Job. The
        // core restores ownership after all Rust callback borrows end.
        retain_job_on_enodev: true,
        owner: match T::MODULE {
            Some(module) => module.as_ptr(),
            None => core::ptr::null_mut(),
        },
    };
    /// Creates a new DRM Scheduler object
    // TODO: Shared timeout workqueues & scores
    pub fn new(
        device: &device::Device,
        num_rqs: u32,
        credit_limit: u32,
        hang_limit: u32,
        timeout_ms: time::Msecs,
        name: &'static CStr,
    ) -> Result<Scheduler<T>> {
        Self::init(
            UniqueArc::new_uninit(GFP_KERNEL)?,
            device,
            num_rqs,
            credit_limit,
            hang_limit,
            timeout_ms,
            name,
            None,
        )
    }

    /// Creates a new DRM Scheduler object whose run and free work goes to `submit_wq`.
    ///
    /// The scheduler owns the queue and destroys it after it has been finalized. Like the queue
    /// drm_sched allocates itself, it must be ordered and usable for memory reclaim (see
    /// [`OwnedQueue::new_ordered_reclaim`]), so that one scheduler's jobs run and are freed one at a
    /// time.
    ///
    /// [`OwnedQueue::new_ordered_reclaim`]: crate::workqueue::OwnedQueue::new_ordered_reclaim
    pub fn new_with_submit_wq(
        device: &device::Device,
        num_rqs: u32,
        credit_limit: u32,
        hang_limit: u32,
        timeout_ms: time::Msecs,
        name: &'static CStr,
        submit_wq: crate::workqueue::OwnedQueue,
    ) -> Result<Scheduler<T>> {
        Self::init(
            UniqueArc::new_uninit(GFP_KERNEL)?,
            device,
            num_rqs,
            credit_limit,
            hang_limit,
            timeout_ms,
            name,
            Some(submit_wq),
        )
    }

    /// Initializes a scheduler in `sched`, whatever that storage held before.
    fn init(
        mut sched: UniqueArc<MaybeUninit<SchedulerInner<T>>>,
        device: &device::Device,
        num_rqs: u32,
        credit_limit: u32,
        hang_limit: u32,
        timeout_ms: time::Msecs,
        name: &'static CStr,
        submit_wq: Option<crate::workqueue::OwnedQueue>,
    ) -> Result<Scheduler<T>> {
        // Allocate the owner before starting C workers: no fallible allocation
        // may abandon an initialized scheduler without running drm_sched_fini.
        let owner: UniqueArc<MaybeUninit<SchedulerOwner<T>>> =
            UniqueArc::new_uninit(GFP_KERNEL)?;

        // SAFETY: zero sched->sched_rq as drm_sched_init() uses it to exit early withoput initialisation
        // TODO: allocate sched zzeroed instead
        unsafe {
            (*sched.as_mut_ptr()).sched.sched_rq = core::ptr::null_mut();
        };

        let init_ops = bindings::drm_sched_init_args {
            ops: &Self::OPS,
            submit_wq: submit_wq.as_ref().map_or(core::ptr::null_mut(), |queue| queue.as_raw()),
            timeout_wq: core::ptr::null_mut(),
            num_rqs,
            credit_limit,
            hang_limit,
            timeout: msecs_to_jiffies(timeout_ms).try_into()?,
            score: core::ptr::null_mut(),
            name: name.as_char_ptr(),
            dev: device.as_raw(),
        };

        // SAFETY: The drm_sched pointer is valid and pinned as it was just allocated above.
        //         `device` is valid by its type invarants
        to_result(unsafe {
            bindings::drm_sched_init(
                addr_of_mut!((*sched.as_mut_ptr()).sched),
                addr_of!(init_ops),
            )
        })?;

        // SAFETY: C initialized sched and PhantomData has no stored state.
        // No fallible operation follows successful initialization.
        let allocation = unsafe { sched.assume_init() }.into();
        Ok(Scheduler(
            owner
                .write(SchedulerOwner {
                    allocation,
                    _submit_wq: submit_wq,
                })
                .into(),
        ))
    }
}
