// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Scheduler side of [`crate::g17::traversal`]: the device's admission state over job fences,
//! the question the scheduler asks it, and the look the inline publication paths take at it.

use super::Backend;
use crate::g17::{job::{Engine, Packet}, traversal::Admission};
use core::sync::atomic::{AtomicBool, Ordering};
use kernel::{
    dma_fence::{Fence, RawDmaFence},
    new_mutex,
    prelude::*,
    sync::{Arc, Mutex},
};

/// A scheduler job, by the fence that signals when it ends.
#[derive(Clone)]
struct Job(Fence);

impl PartialEq for Job {
    fn eq(&self, other: &Self) -> bool {
        self.0.raw() == other.0.raw()
    }
}

impl Job {
    /// The job completed, failed or was cancelled.
    fn ended(&self) -> bool {
        // SAFETY: The owned fence keeps the raw DMA fence alive for the call.
        unsafe { kernel::bindings::dma_fence_get_status(self.0.raw()) != 0 }
    }
}

/// The device's admission state. An ended job's fence stays referenced until the next command
/// asks.
#[pin_data]
pub(crate) struct Gate {
    #[pin]
    state: Mutex<Admission<Job>>,
    /// Something is admitted or waiting; compute commands that trace no rays skip the lock
    /// otherwise.
    busy: AtomicBool,
}

impl Gate {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            try_pin_init!(Self {
                state <- new_mutex!(Admission::new(), "g17::traversal::Gate"),
                busy: AtomicBool::new(false),
            }),
            GFP_KERNEL,
        )
    }

    /// Admission of the command whose job fence is `mine`, of `client`; `exclusive`: it runs
    /// for one client at a time, as a command that traces rays. `None`: it may go on.
    /// `Some(fence)`: wait for that fence, then ask again.
    fn admit(&self, client: u64, exclusive: bool, mine: &Fence) -> Option<Fence> {
        if !exclusive && !self.busy.load(Ordering::Acquire) {
            return None;
        }
        let job = Job(mine.clone());
        let mut state = self.state.lock();
        let wait = if exclusive {
            state.traversal(client, &job, Job::ended)
        } else {
            state.compute(client, &job, Job::ended)
        };
        self.busy.store(state.busy(), Ordering::Release);
        wait.map(|job| job.0)
    }

    /// Nothing is admitted or waiting.
    fn idle(&self) -> bool {
        !self.busy.load(Ordering::Acquire)
    }
}

impl Backend {
    /// The last step of a command's admission in its scheduler: `Some(fence)` while the
    /// ray-traversal rule ([`crate::g17::traversal`]) keeps the command out of the GPU.
    ///
    /// Only the scheduler asks, for a job whose dependencies and capacity are there: a command
    /// that waits here holds other clients' ray-traversal work, so it must be able to run. A
    /// job that ends early stops holding others when its fence signals, which bounds their
    /// wait by its watchdog.
    pub(super) fn traversal_admission(&self, packet: &Arc<Packet>) -> Option<Fence> {
        if !self.shared.variant.traversal_exclusion {
            return None;
        }
        let exclusive = Self::exclusive(packet);
        if !exclusive && packet.engine() != Engine::Compute {
            return None;
        }
        if packet.early_result().is_some()
            || packet.cancel_requested()
            || packet.completion.status().get() != 0
        {
            return None;
        }
        self.shared.traversal.admit(
            packet.context.vm().id(),
            exclusive,
            &packet.completion.scheduler_fence(),
        )
    }

    /// The command runs for one client at a time: it traces rays, or it is a render with mesh
    /// shading. Two clients' mesh renders in the GPU together lose whole mesh workgroups or give
    /// one view's primitives another view's per-primitive data; one after the other they are
    /// correct. `aurora.g17_mesh_exclusion=0` lets mesh renders overlap again, for an A/B.
    fn exclusive(packet: &Packet) -> bool {
        packet.command.traces_rays()
            || (packet.engine() == Engine::Render
                && packet.command.mesh_output()
                && mesh_exclusion())
    }

    /// Whether the command may be published from its submit ioctl: never one that runs for one
    /// client at a time ([`Self::exclusive`]), and a compute command only while nothing is
    /// admitted or waiting. This only looks.
    pub(super) fn traversal_inline(&self, packet: &Arc<Packet>) -> bool {
        if !self.shared.variant.traversal_exclusion {
            return true;
        }
        if Self::exclusive(packet) {
            return false;
        }
        packet.engine() != Engine::Compute || self.shared.traversal.idle()
    }
}

unsafe extern "C" {
    fn asahi_g17_mesh_exclusion() -> u32;
}

/// `aurora.g17_mesh_exclusion` (g17/mesh_exclusion.c): renders with mesh shading run for one
/// client at a time.
fn mesh_exclusion() -> bool {
    // SAFETY: Reads a module parameter; no other effect.
    unsafe { asahi_g17_mesh_exclusion() != 0 }
}
