// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scheduler release and off-lock destruction of acknowledged logical queue resources.

use super::{ComputeEntry, Registry};
use crate::g17::{
    buffer::BufferIds,
    channel::Rings,
    context::Context,
    fw::channels::{ControlRecord, SchedulerStateRelease},
    initdata::InitData,
    queue::render::RetiredBinding,
    recovery::Memory as _,
    teardown::{Failure, Host},
    Coprocessor, Firmware, MSG_CONTROL_NOTIFY,
};
use crate::hw::t8140::queues::RENDER_SLOTS;
use core::sync::atomic::Ordering;
use kernel::{prelude::*, sync::Arc};

const CONTEXTS: usize = 128;

/// Pop under the reclaim mutex, then finish after dropping every device lock.
pub(in crate::g17) enum Reclaim {
    Binding(RetiredBinding),
    Context(Arc<Context>),
}

impl Reclaim {
    pub(in crate::g17) fn finish(self) {
        match self {
            Self::Binding(binding) => drop(binding),
            Self::Context(context) => {
                context.release_execution();
                drop(context.take_render_pool());
                drop(context);
            }
        }
    }
}

/// All slots are allocated at probe. A full collector postpones detach. Empty
/// slots let a validated transfer move ownership without a fallible vector push.
pub(in crate::g17) struct ReclaimBatch {
    bindings: KVVec<Option<RetiredBinding>>,
    contexts: KVec<Option<Arc<Context>>>,
}

impl ReclaimBatch {
    pub(in crate::g17) fn new() -> Result<Self> {
        let mut bindings = KVVec::with_capacity(CONTEXTS + RENDER_SLOTS, GFP_KERNEL)?;
        for _ in 0..CONTEXTS + RENDER_SLOTS {
            bindings.push(None, GFP_KERNEL)?;
        }
        let mut contexts = KVec::with_capacity(CONTEXTS, GFP_KERNEL)?;
        for _ in 0..CONTEXTS {
            contexts.push(None, GFP_KERNEL)?;
        }
        Ok(Self { bindings, contexts })
    }

    pub(in crate::g17) fn is_empty(&self) -> bool {
        self.bindings.iter().all(Option::is_none) && self.contexts.iter().all(Option::is_none)
    }

    /// Graphs go before the Context owners that armed their scheduler releases.
    pub(in crate::g17) fn take_one(&mut self) -> Option<Reclaim> {
        if let Some(slot) = self.bindings.iter_mut().find(|slot| slot.is_some()) {
            return slot.take().map(Reclaim::Binding);
        }
        self.contexts
            .iter_mut()
            .find(|slot| slot.is_some())
            .and_then(Option::take)
            .map(Reclaim::Context)
    }

    pub(in crate::g17::runtime) fn has_room(&self, bindings: usize, contexts: usize) -> bool {
        self.bindings.iter().filter(|slot| slot.is_none()).count() >= bindings
            && self.contexts.iter().filter(|slot| slot.is_none()).count() >= contexts
    }

    pub(in crate::g17::runtime) fn binding_slot(&mut self) -> Result<&mut Option<RetiredBinding>> {
        self.bindings
            .iter_mut()
            .find(|slot| slot.is_none())
            .ok_or(EBUSY)
    }

    fn context(&mut self, context: &Arc<Context>) -> Result {
        let slot = self
            .contexts
            .iter_mut()
            .find(|slot| slot.is_none())
            .ok_or(EBUSY)?;
        *slot = Some(context.clone());
        Ok(())
    }
}

struct Service<'a> {
    render: &'a mut super::render::RenderState,
    compute: &'a [Option<ComputeEntry>],
    buffers: &'a mut BufferIds,
    init: &'a InitData,
    primary: &'a mut Coprocessor,
    reclaim: &'a mut ReclaimBatch,
}

impl Host for Service<'_> {
    fn render_idle(&mut self, context: &Arc<Context>) -> Result<bool> {
        self.render.teardown_idle(context)
    }
    fn compute_idle(&self, context: &Arc<Context>) -> bool {
        self.compute
            .iter()
            .flatten()
            .filter_map(|entry| entry.queue.as_ref())
            .all(|queue| queue.idle_for_context(context) && !queue.switching_from(context))
    }
    fn flists_released(&self, context: &Arc<Context>) -> bool {
        self.render.teardown_pools_released(context)
    }
    fn control_backpressured(&self) -> bool {
        self.render.control_backpressured()
    }
    fn publish(&mut self, record: &SchedulerStateRelease) -> Result<u32> {
        Rings::publish_control(
            self.init,
            &ControlRecord::SchedulerStateRelease(SchedulerStateRelease::new(
                record.page_va,
                record.id,
            )),
        )
    }
    fn notify(&mut self) -> Result {
        self.primary.notify(MSG_CONTROL_NOTIFY)
    }
    fn detach(&mut self, context: &Arc<Context>) -> Result {
        self.render
            .detach_context(context, self.buffers, self.reclaim)?;
        // detach_context preflights this slot together with every graph slot.
        // Retain it before Pending removes its reference under the device lock.
        self.reclaim.context(context)
    }
    fn report(&mut self, context: &Arc<Context>, failure: Failure) {
        let dev = self.primary.state.shared.dev.as_ref();
        match failure {
            Failure::Publication(error) => dev_err!(
                dev,
                "Scheduler {} release failed: {:?}\n",
                context.id(),
                error
            ),
            Failure::Announcement(error) => dev_warn!(
                dev,
                "Scheduler {} release announcement failed: {:?}\n",
                context.id(),
                error
            ),
        }
    }
}

impl Firmware {
    /// The event worker has already reclaimed drained pairs and reannounced
    /// pending controls. Recovery blocks new releases, never an observed ack.
    pub(in crate::g17) fn service_teardown(&mut self, reclaim: &mut ReclaimBatch) -> Result {
        self.queues.render.collect_reclaimable(reclaim)?;
        if self.queues.teardown.is_empty() {
            return Ok(());
        }
        self.queues.render.observe_teardown_releases(&self.init)?;
        let recovery = self.recovery.pending() || self.init.recovery_state()? != 0;
        let Registry {
            render,
            compute,
            buffers,
            teardown,
            ..
        } = &mut self.queues;
        let mut service = Service {
            render,
            compute,
            buffers,
            init: &self.init,
            primary: &mut self.primary,
            reclaim,
        };
        teardown.publish_ready(&mut service, recovery, super::now_ns())?;
        let consumer = self.init.control_consumer()?.load(Ordering::Relaxed);
        let producer = self.init.control_producer()?.load(Ordering::Relaxed);
        while teardown
            .take_consumed(consumer, producer, &mut service)?
            .is_some()
        {}
        Ok(())
    }
}
