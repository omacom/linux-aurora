// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Job fences.
//!
//! One fence per job (= per SUBMIT, the aggregate of its commands) on a per-queue context
//! ([`Timeline`]), signalled strictly in queue order. The VM status is latched (and mirrored)
//! before a failed job's fence signals ([`super::job::Job`]). Signalling happens after the device
//! lock is dropped.
//!
//! The engine retires a queue's jobs in order, but a later job may still be completed first
//! (a prefix failed before an earlier job's last prefix retired, or a never-run job cancelled at
//! queue teardown). Fences of one context must signal in order, so a [`Timeline`] holds a
//! completed fence back until every earlier fence of the queue has completed.

use kernel::{
    c_str,
    dma_fence::{Fence, FenceContexts, FenceObject, FenceOps, RawDmaFence, UserFence},
    new_mutex,
    prelude::*,
    str::CStr,
    sync::{Arc, LockClassKey, Mutex},
};

use crate::file::SyncItem;

static TIMELINE_KEY: Pin<&LockClassKey> = kernel::static_lock_class!();
static ERROR_KEY: Pin<&LockClassKey> = kernel::static_lock_class!();

/// Fence operations of G15 job fences.
pub(crate) struct JobFenceOps;

#[vtable]
impl FenceOps for JobFenceOps {
    fn get_driver_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("asahi")
    }

    fn get_timeline_name<'a>(self: &'a FenceObject<Self>) -> &'a CStr {
        c_str!("g15")
    }
}

/// An accepted fence of a timeline that has not signalled yet.
struct Pending {
    fence: UserFence<JobFenceOps>,
    /// The result, once the job completed.
    result: Option<Result>,
}

/// The fence timeline of one logical queue.
#[pin_data]
pub(crate) struct Timeline {
    /// One context for job fences, signalled in creation order.
    jobs: FenceContexts,
    /// One context for the error fences of refused submissions, each signalled at creation.
    errors: FenceContexts,
    /// Accepted fences that have not signalled, oldest first.
    #[pin]
    pending: Mutex<KVec<Pending>>,
    /// Pre-signalled ENOMEM fence for a refused submission when no error fence can be allocated.
    allocation_failure: Fence,
}

impl Timeline {
    pub(crate) fn new() -> Result<Arc<Self>> {
        let errors = FenceContexts::new(1, c_str!("asahi_g15_refused"), ERROR_KEY)?;
        let allocation_failure = error_fence(&errors, ENOMEM)?;
        Arc::pin_init(
            try_pin_init!(Self {
                jobs: FenceContexts::new(1, c_str!("asahi_g15_queue"), TIMELINE_KEY)?,
                errors,
                pending <- new_mutex!(KVec::new(), "g15::Timeline::pending"),
                allocation_failure,
            }),
            GFP_KERNEL,
        )
    }

    /// Creates the fence of the next accepted job. Fences are created in submission order: the
    /// caller holds the queue (SUBMIT is serialized per queue).
    pub(crate) fn next(self: &Arc<Self>) -> Result<JobFence> {
        let mut pending = self.pending.lock();
        pending.reserve(1, GFP_KERNEL)?;
        let fence: UserFence<JobFenceOps> = self.jobs.new_fence(0, JobFenceOps)?.into();
        pending
            .push_within_capacity(Pending {
                fence: fence.clone(),
                result: None,
            })
            .map_err(|_| ENOMEM)?;
        Ok(JobFence {
            fence,
            timeline: self.clone(),
        })
    }

    /// Records the result of `fence` and signals every completed fence at the head, in order.
    fn complete(&self, fence: &UserFence<JobFenceOps>, result: Result) {
        let mut pending = self.pending.lock();
        let Some(entry) = pending
            .iter_mut()
            .find(|entry| entry.fence.raw() == fence.raw())
        else {
            // Already signalled.
            return;
        };
        if entry.result.is_some() {
            return;
        }
        entry.result = Some(result);
        let mut signalled = 0;
        for entry in pending.iter() {
            let Some(result) = entry.result else {
                break;
            };
            if let Err(error) = result {
                entry.fence.set_error(error);
            }
            entry.fence.signal();
            signalled += 1;
        }
        drop(pending.drain(..signalled));
    }

    /// A signalled fence carrying `error`, for the output syncs of a refused submission.
    pub(crate) fn refused(&self, error: Error) -> Fence {
        error_fence(&self.errors, error).unwrap_or_else(|_| self.allocation_failure.clone())
    }
}

/// A fence on `contexts` signalled with `error`. Every fence of an error context signals at
/// creation, so the context's fences signal in order.
fn error_fence(contexts: &FenceContexts, error: Error) -> Result<Fence> {
    let fence = contexts.new_fence(0, JobFenceOps)?;
    fence.set_error(error);
    fence.signal();
    Ok(Fence::from_fence(&fence))
}

/// The hardware fence of one job.
pub(crate) struct JobFence {
    fence: UserFence<JobFenceOps>,
    timeline: Arc<Timeline>,
}

impl JobFence {
    /// Signals the fence, with an error on failure, after every earlier fence of the queue. Only
    /// the first call has an effect.
    pub(crate) fn signal(&self, result: Result) {
        self.timeline.complete(&self.fence, result);
    }

    /// A reference to the fence.
    pub(crate) fn fence(&self) -> Fence {
        Fence::from_fence(&self.fence)
    }
}

impl Drop for JobFence {
    fn drop(&mut self) {
        // Every fence of a timeline must complete, or no later fence of the queue would signal.
        // A no-op for a completed fence (a job that was never accepted, or a bug, is cancelled).
        self.timeline.complete(&self.fence, Err(ECANCELED));
    }
}

/// Installs a fence in the output syncs of a submission.
fn install(outputs: impl Iterator<Item = SyncItem>, fence: &Fence) {
    for mut sync in outputs {
        if let Some(chain) = sync.chain_fence.take() {
            sync.syncobj.add_point(chain, fence, sync.timeline_value);
        } else {
            sync.syncobj.replace_fence(Some(fence));
        }
    }
}

/// The syncs of one SUBMIT. Every output sync gets a fence: the job's fence once the job is
/// accepted, a signalled error fence when the submission is refused (the shared `file.rs`
/// contract of `tracked_submission`).
pub(crate) struct Syncs {
    syncs: KVec<SyncItem>,
    input_count: usize,
}

impl Syncs {
    pub(crate) fn new(syncs: KVec<SyncItem>, input_count: usize) -> Self {
        Self { syncs, input_count }
    }

    /// The input syncs (all resolved to fences or already signalled).
    pub(crate) fn inputs(&self) -> &[SyncItem] {
        &self.syncs[..self.input_count.min(self.syncs.len())]
    }

    /// Publishes the accepted job's fence in every output sync.
    pub(crate) fn accept(mut self, fence: &Fence) {
        let first = self.input_count.min(self.syncs.len());
        install(self.syncs.drain(first..), fence);
    }

    /// Publishes a signalled error fence in every output sync of a refused submission.
    pub(crate) fn refuse(mut self, timeline: &Timeline, error: Error) {
        let first = self.input_count.min(self.syncs.len());
        if first < self.syncs.len() {
            install(self.syncs.drain(first..), &timeline.refused(error));
        }
    }
}
