// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Completion work owned only by the M3 runtime.

use kernel::{prelude::*, sync::Arc, workqueue::{self, impl_has_work, new_work, Work, WorkItem}};
use crate::m3::{m3_drm::Shared, m3_rtkit::State};

#[pin_data]
pub(crate) struct Completion {
    shared: Shared,
    #[pin]
    work: Work<Completion>,
}
impl Completion {
    pub(crate) fn new(shared: Shared) -> Result<Arc<Self>> {
        Arc::pin_init(try_pin_init!(Self {
            shared,
            work <- new_work!("M3 completion"),
        }), GFP_KERNEL)
    }
}
impl_has_work! { impl HasWork<Completion> for Completion { self.work } }
impl WorkItem for Completion {
    type Pointer = Arc<Self>;
    fn run(owner: Arc<Self>) {
        use crate::m3::m3_runtime::Service;
        // No deliberate spin: fsleep(5) selects udelay, not a scheduler sleep.
        // Check immediately on IRQ, with backoff for trailing coherent witnesses.
        let mut policy = super::m3_completion_policy::WaitPolicy::new();
        loop {
            let (outcome, events) = {
                let mut guard = owner.shared.lock();
                let Some(runtime) = Option::as_mut(&mut *guard) else { return; };
                let outcome=runtime.service_job();
                let events=runtime.events();
                // Preserve the root-only CPU-overlap diagnostic: disabled waits retain
                // the runtime lock so VM/timestamp preparation cannot overlap the GPU.
                if let Service::Waiting(messages)=outcome {
                    if !events.cpu_overlap() {
                        events.wait_past(messages, policy.next_wait_us(events.wait_ceiling_us()));
                        continue;
                    }
                }
                (outcome,events)
            };
            match outcome {
                Service::Idle => break,
                Service::Progress => policy.progress(),
                Service::Waiting(messages) => {
                    // IRQs and timer expiry alone are not retirement progress.
                    // Keep backing off until service publishes or retires work.
                    events.wait_past(messages, policy.next_wait_us(events.wait_ceiling_us()));
                }
            }
        }
    }
}
pub(crate) fn queue(state: &State) {
    if let Some(work) = state.completion.as_ref() {
        let _ = workqueue::system_highpri().enqueue(work.clone());
    }
}
