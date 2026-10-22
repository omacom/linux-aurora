// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Retained scheduler-state releases. Logical close arms a release; render and compute drain,
//! private-memory release and control-ring consumption are separate ownership witnesses.
//! The device mutex serializes this state with the Registry's displaced graph ownership.

use super::{context::Context, freelist::control_consumed, fw::channels::SchedulerStateRelease};
use core::sync::atomic::{fence, Ordering};
use kernel::{prelude::*, sync::Arc};

/// Entries preallocated at device start; a burst of closes beyond it grows the table.
const CAPACITY: usize = 128;
const POLL_NS: u64 = 2_000_000_000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Deferred,
    Draining,
    Released(u32),
}

struct Entry {
    context: Arc<Context>,
    phase: Phase,
    armed: u64,
    announce_pending: bool,
}

/// Publication and announcement errors retain the entry; no timeout frees
/// memory the firmware may still own.
#[derive(Clone, Copy)]
pub(crate) enum Failure {
    Publication(Error),
    Announcement(Error),
}

/// Borrowed device operations. Readiness covers every matching installed and displaced graph,
/// including pools kept by the Context after a same-VM successor took over its physical pair.
/// `detach` must retain all detached backing for off-lock destruction before returning success.
/// It compares logical Arc identity so a successor sharing an execution ID remains installed.
pub(crate) trait Host {
    fn render_idle(&mut self, context: &Arc<Context>) -> Result<bool>;
    fn compute_idle(&self, context: &Arc<Context>) -> bool;
    fn flists_released(&self, context: &Arc<Context>) -> bool;
    fn control_backpressured(&self) -> bool;
    fn publish(&mut self, record: &SchedulerStateRelease) -> Result<u32>;
    fn notify(&mut self) -> Result;
    fn detach(&mut self, context: &Arc<Context>) -> Result;
    fn report(&mut self, context: &Arc<Context>, failure: Failure);
}

pub(crate) struct Pending {
    entries: KVec<Entry>,
}

impl Pending {
    pub(crate) fn new() -> Result<Self> {
        Ok(Self {
            entries: KVec::with_capacity(CAPACITY, GFP_KERNEL)?,
        })
    }

    pub(crate) fn closed(&self, index: usize, now: u64) -> Option<(Arc<Context>, u64)> {
        self.entries
            .get(index)
            .filter(|entry| entry.phase == Phase::Draining)
            .map(|entry| (entry.context.clone(), now.saturating_sub(entry.armed)))
    }

    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The context's scheduler release is armed or published, and the queues it
    /// last ran have not been detached yet. Until then the firmware may still
    /// process the release against those queues: none of them may change owner.
    pub(crate) fn releasing(&self, context: &Arc<Context>) -> bool {
        self.find(context)
            .is_some_and(|index| self.entries[index].phase != Phase::Deferred)
    }

    /// `releasing` for the context that owns scheduler page `scheduler`.
    pub(crate) fn releasing_scheduler(&self, scheduler: u64) -> bool {
        self.entries.iter().any(|entry| {
            entry.phase != Phase::Deferred && entry.context.scheduler_va() == scheduler
        })
    }

    /// Entries by phase (displaced but open, draining, release published), for diagnostics.
    pub(crate) fn phases(&self) -> [usize; 3] {
        let mut phases = [0; 3];
        for entry in self.entries.iter() {
            phases[match entry.phase {
                Phase::Deferred => 0,
                Phase::Draining => 1,
                Phase::Released(_) => 2,
            }] += 1;
        }
        phases
    }

    fn find(&self, context: &Arc<Context>) -> Option<usize> {
        self.entries
            .iter()
            .position(|entry| Arc::ptr_eq(&entry.context, context))
    }

    /// Closes outpacing firmware release consumption grow the table instead of
    /// failing the close: a failed close stops the device for every client.
    fn insert(&mut self, context: &Arc<Context>, phase: Phase, now: u64) -> Result {
        let entry = Entry {
            context: context.clone(),
            phase,
            armed: now,
            announce_pending: false,
        };
        self.entries.push(entry, GFP_KERNEL)?;
        Ok(())
    }

    /// Call before replacing a graph's logical owner. Failure leaves that graph with its
    /// original owner. The Registry, rather than this state machine, retains displaced backing.
    pub(crate) fn displace(
        &mut self,
        context: &Arc<Context>,
        owner_closed: bool,
        now: u64,
    ) -> Result {
        if self.find(context).is_some() {
            return Ok(());
        }
        let phase = if owner_closed {
            Phase::Draining
        } else {
            Phase::Deferred
        };
        self.insert(context, phase, now)
    }

    /// No allocation is needed at close. A scheduler page never exposed to firmware requires
    /// no release, unless the Registry already retained one of its displaced graphs.
    pub(crate) fn close(&mut self, context: &Arc<Context>, now: u64) -> Result {
        if let Some(index) = self.find(context) {
            let entry = &mut self.entries[index];
            if entry.phase == Phase::Deferred {
                entry.phase = Phase::Draining;
                entry.armed = now;
            }
            return Ok(());
        }
        if context.is_published() {
            self.insert(context, Phase::Draining, now)?;
        }
        Ok(())
    }

    /// Publish in insertion order. Mandatory control replies and a full control ring stop this
    /// pass; an error for one page leaves it retained while other ready releases may progress.
    pub(crate) fn publish_ready(&mut self, host: &mut impl Host, recovery: bool, now: u64) -> Result {
        if !recovery {
            for entry in self.entries.iter_mut() {
                // The ring record is already published. Retry only its doorbell;
                // never duplicate a scheduler release after a transport error.
                if matches!(entry.phase, Phase::Released(_)) {
                    if entry.announce_pending && host.notify().is_ok() {
                        entry.announce_pending = false;
                        // One successful retry starts its consumption window.
                        // Failed retries never slide the polling deadline.
                        entry.armed = now;
                    }
                    continue;
                }
                if entry.phase != Phase::Draining
                    || !host.render_idle(&entry.context)?
                    || !host.flists_released(&entry.context)
                    || !host.compute_idle(&entry.context)
                {
                    continue;
                }
                if host.control_backpressured() {
                    break;
                }
                let result = (|| {
                    let id = entry.context.release_identity()?;
                    let record = SchedulerStateRelease::new(
                        entry.context.scheduler_va(),
                        [id[3], id[0], id[1], id[2]],
                    );
                    host.publish(&record)
                })();
                let next = match result {
                    Ok(next) => next,
                    Err(EAGAIN) => break,
                    Err(error) => {
                        host.report(&entry.context, Failure::Publication(error));
                        continue;
                    }
                };
                entry.phase = Phase::Released(next);
                // A long command drain must not consume the acknowledgement
                // polling window before this release exists on the control ring.
                entry.armed = now;
                entry.announce_pending = true;
                fence(Ordering::SeqCst);
                match host.notify() {
                    Ok(()) => entry.announce_pending = false,
                    Err(error) => host.report(&entry.context, Failure::Announcement(error)),
                }
            }
        }
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// The caller snapshots both control cursors once for this settlement pass and repeatedly
    /// takes consumed entries. It retains each returned Context until its detached graphs and
    /// VM aliases can be destroyed outside the device mutex.
    pub(crate) fn take_consumed(
        &mut self,
        consumer: u32,
        producer: u32,
        host: &mut impl Host,
    ) -> Result<Option<Arc<Context>>> {
        for index in 0..self.entries.len() {
            let entry = &self.entries[index];
            let Phase::Released(target) = entry.phase else {
                continue;
            };
            if !control_consumed(consumer, producer, target)?
                || !host.flists_released(&entry.context)
            {
                continue;
            }
            match host.detach(&entry.context) {
                Ok(()) => {}
                Err(EBUSY) => continue,
                Err(error) => return Err(error),
            }
            let entry = self.entries.remove(index).map_err(|_| EIO)?;
            return Ok(Some(entry.context));
        }
        Ok(None)
    }

    /// Poll for a bounded window at close and again at actual release publication.
    /// Failed announcements remain retryable on later service even after polling
    /// expires. Time passage never substitutes for a firmware-consumption witness.
    pub(crate) fn polling(&self, now: u64) -> bool {
        self.entries.iter().any(|entry| {
            entry.phase != Phase::Deferred && now.saturating_sub(entry.armed) < POLL_NS
        })
    }
}
