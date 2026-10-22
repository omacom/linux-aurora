// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scheduler callbacks and the off-lock fence boundary of a logical queue.

use super::{Backend, DeferredBatch};
use crate::g17::{
    completion::Deferred,
    job::{self, Engine, Packet},
};
use core::sync::atomic::Ordering;
use kernel::{dma_fence::Fence, drm::sched, prelude::*, sync::Arc};

impl Backend {
    pub(super) fn release_backlog(&self, packet: &Packet) {
        if packet.take_backlog() {
            match packet.engine() {
                Engine::Render => self.backlog.fetch_sub(1, Ordering::AcqRel),
                Engine::Compute => self.compute_backlog.fetch_sub(1, Ordering::AcqRel),
            };
        }
    }

    fn stop_after_error(&self) {
        self.shared.lose_device_quietly();
        self.shared.stop_queues();
    }

    /// Complete all failure fanout before signalling this packet's unpublished
    /// error. Visible failures keep the engine's retained owner through a witness.
    pub(super) fn finish_publication(
        &self,
        packet: &Arc<Packet>,
        result: Result,
        deferred: &mut DeferredBatch,
    ) -> Result<Fence> {
        if let Err(error) = result {
            if [EIO, ETIMEDOUT, ENODEV].contains(&error) || packet.completion.status().get() != 0 {
                <Self as job::Backend>::fail_vm(self, error);
            }
            deferred.finish();
            if !packet.is_published() {
                packet.completion.fail_unpublished(error);
            }
            Err(error)
        } else {
            deferred.finish();
            Ok(packet.completion.scheduler_fence())
        }
    }

    fn release_queue(&self) -> Result {
        let mut state = self.shared.state.lock();
        let Some(firmware) = (*state).as_deref_mut() else {
            return Ok(());
        };
        // Retain a published scheduler before a never-kicked physical graph is
        // removed. The teardown entry owns its root until control consumption.
        firmware
            .queues
            .teardown
            .close(&self.context, super::now_ns())?;
        firmware.queues.release_compute(self.owner)?;
        firmware.queues.release_render(self.owner)?;
        firmware.queues.admission.release_owner(self.owner);
        drop(state);
        self.shared.queue_events();
        Ok(())
    }
}

impl job::Backend for Backend {
    fn feed(&self) -> Option<Arc<crate::g17::feed::Feed>> {
        Some(self.shared.feed.clone())
    }

    fn render_scs_blocks(&self, pages: [u16; 2]) -> Result<usize> {
        crate::g17::fw::render_rt::required_blocks(self.shared.active_cores, pages)
    }

    fn ensure_compute(&self, scs_pages: u16) -> Result {
        Backend::ensure_compute(self, scs_pages)
    }

    fn fail_vm(&self, error: Error) {
        self.context.status().record(error);
        let mut deferred = match DeferredBatch::worker() {
            Ok(batch) => batch,
            Err(_) => {
                self.stop_after_error();
                return;
            }
        };
        let result = (|| {
            let mut state = self.shared.state.lock();
            let Some(firmware) = (*state).as_deref_mut() else {
                return Ok(());
            };
            // Final observations win over this VM's terminal fanout.
            firmware.scan_compute(None, &mut deferred)?;
            firmware.settle_render_vm(self.context.status(), error, &mut deferred)?;
            firmware
                .queues
                .fail_compute_vm(self.context.status(), error, &mut deferred)
        })();
        deferred.finish();
        if result.is_err() {
            self.stop_after_error();
        }
    }

    fn release_owner(&self) {
        if self.release_queue().is_err() {
            self.stop_after_error();
        }
    }

    /// Firmware kick parents order the same engine. Explicit cross-engine and
    /// input dependencies were already attached to the DRM scheduled job.
    fn prepare(&self, _packet: &Packet) -> Option<Fence> {
        None
    }

    fn publish(&self, packet: Arc<Packet>) -> Result<Fence> {
        if let Some(result) = packet.early_result() {
            return result.map(|()| packet.completion.scheduler_fence());
        }
        let mut deferred = match DeferredBatch::publication() {
            Ok(batch) => batch,
            Err(error) => {
                self.release_backlog(&packet);
                if packet.completion.status().get() != 0 {
                    <Self as job::Backend>::fail_vm(self, error);
                }
                packet.completion.fail_unpublished(error);
                return Err(error);
            }
        };
        let result = if !packet.begin_publication() {
            Err(ECANCELED)
        } else if packet.completion.status().get() != 0 {
            Err(EIO)
        } else {
            match packet.engine() {
                Engine::Compute => {
                    self.compute_packet(&packet, &mut deferred, super::Wait::scheduled())
                }
                Engine::Render => self.render_packet(&packet, &mut deferred),
            }
        };
        self.release_backlog(&packet);
        self.finish_publication(&packet, result, &mut deferred)
    }

    fn timed_out(&self, packet: Arc<Packet>) -> sched::Status {
        // A packet still in publication keeps its place in the backlog: its
        // publisher waits on, and a later dependency-free command must not be
        // published from its ioctl ahead of it. Publication, cancellation and
        // job drop release it.
        if packet.is_published() {
            self.release_backlog(&packet);
        }
        let result = match packet.engine() {
            Engine::Compute => self.timeout_compute(&packet),
            Engine::Render => self.timeout_render(&packet),
        };
        if result.is_err() {
            self.stop_after_error();
        }
        sched::Status::NoHang
    }

    fn cancel(&self, packet: Arc<Packet>) {
        self.release_backlog(&packet);
        if packet.request_cancel() {
            Deferred::Unpublished(packet.completion.clone(), ECANCELED).finish();
            return;
        }
        let result = match packet.engine() {
            Engine::Compute => self.cancel_compute(&packet),
            Engine::Render => self.cancel_render(&packet),
        };
        if result.is_err() {
            self.stop_after_error();
        }
    }

    fn defer_timeout(&self, packet: &Arc<Packet>, renewals: u8) -> bool {
        if packet.engine() == Engine::Compute {
            return self.defer_compute_timeout(packet, renewals);
        }
        let mut state = self.shared.state.lock();
        (*state)
            .as_deref_mut()
            .is_some_and(|firmware| firmware.queues.defer_render_timeout(packet, ETIMEDOUT))
    }

    fn try_early(&self, packet: &Arc<Packet>) -> bool {
        match packet.engine() {
            Engine::Render => self.try_render_early(packet),
            Engine::Compute => self.try_compute_early(packet),
        }
    }

    fn count_backlog(&self, packet: &Packet) {
        packet.count_backlog();
        match packet.engine() {
            Engine::Render => self.backlog.fetch_add(1, Ordering::AcqRel),
            Engine::Compute => self.compute_backlog.fetch_add(1, Ordering::AcqRel),
        };
    }
}
