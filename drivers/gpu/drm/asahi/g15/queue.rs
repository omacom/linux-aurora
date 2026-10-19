// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Logical queues: `crate::queue::Queue` for G15.
//!
//! Logical queues own no firmware state: they are independent scheduling entities over the one
//! shared graph, executed in one total order across queues and VMs. QUEUE_CREATE validates
//! `usc_exec_base` (4 GiB aligned, the window inside the user range) and refuses a failed VM or
//! GPU (EIO); HIGH/REALTIME are refused by the shared `file.rs`. Ray-query scratch layouts are
//! refused by the shared default `reserve_render_scs` (RENDER_RAY_QUERY is never advertised).
//!
//! QUEUE_DESTROY (dropping the queue) drains: the scheduler entity is torn down first, which
//! waits (killable) until every accepted job has been handed to the engine; the engine then runs
//! them to completion and their fences signal in order. A closing process waits at most a second
//! for jobs still waiting for inputs and cancels them (ECANCELED, in fence order).
//!
//! SUBMIT checks, in order: the stream's structure and the sync count (EINVAL), the VM's state
//! (a failed VM refuses with EIO), the timestamp handles, the EXACT_PRIOR ordinals against the
//! commands accepted on this queue before the ioctl, a job reference on the VM, then every
//! command's spans and stream gates ([`super::command`]). The accepted job is pushed to the
//! queue's scheduler and its fence published in every output sync; a refused SUBMIT publishes a
//! signalled error fence instead.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use kernel::{dma_fence::Fence, prelude::*, sync::Arc, uapi, xarray};

use super::{
    command::{self, USC_WINDOW},
    fence::{Syncs, Timeline},
    hw,
    job::{Job, Scheduler},
    params,
    vm::{self, VmState},
    Gpu, Shared,
};
use crate::{file, mmu};

/// Per-queue state the engine reads.
pub(crate) struct QueueState {
    /// Host priority: `DRM_ASAHI_PRIORITY_REALTIME - priority` as `file.rs` passes it (MEDIUM 2,
    /// LOW 3; `file.rs` refuses HIGH and REALTIME). Smaller is served first.
    pub(crate) priority: u32,
    /// USC executable window base.
    pub(crate) usc_base: u64,
    /// Engine service turn of this queue (least recently served first).
    pub(crate) turn: AtomicU64,
    /// Owned-job counts at the queue's last 32 arrivals (compute horizon ahead), recorded by the
    /// engine when a job arrives.
    pub(crate) owned: [AtomicU32; 32],
    pub(crate) owned_index: AtomicU32,
}

/// A G15 logical queue.
pub(crate) struct Queue {
    // Dropped first: QUEUE_DESTROY drains the scheduler before anything else goes.
    scheduler: Scheduler,
    pub(crate) state: Arc<QueueState>,
    /// Commands accepted per engine (render, compute) since creation, for EXACT_PRIOR.
    pub(crate) accepted: [u64; 2],
    vm: Arc<VmState>,
    shared: Arc<Shared>,
    timeline: Arc<Timeline>,
}

impl Drop for Queue {
    fn drop(&mut self) {
        // The registry's references to closed VMs (this queue's VM among them, if its handle is
        // gone) go now; this VM's state lives on until the scheduler below has drained.
        self.vm.prune_registry();
    }
}

impl Queue {
    /// Creates a logical queue. `priority` is `DRM_ASAHI_PRIORITY_REALTIME - priority`, as
    /// `file.rs` passes it.
    pub(crate) fn new(gpu: &Gpu, vm: mmu::Vm, priority: u32, usc_exec_base: u64) -> Result<Self> {
        if priority > uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME {
            return Err(EINVAL);
        }
        let range = hw::USER_RANGE;
        if usc_exec_base & (USC_WINDOW - 1) != 0
            || usc_exec_base < range.start
            || usc_exec_base
                .checked_add(USC_WINDOW)
                .is_none_or(|end| end > range.end)
        {
            return Err(EINVAL);
        }
        if gpu.shared.crashed.load(Ordering::Acquire) {
            return Err(EIO);
        }
        let vm_state = vm::lookup(gpu, &vm)?;
        if vm_state.status.get() != 0 {
            return Err(EIO);
        }
        let timeline = Timeline::new()?;
        let scheduler = Scheduler::new(&gpu.shared.dev)?;
        let state = Arc::new(
            QueueState {
                priority,
                usc_base: usc_exec_base,
                turn: AtomicU64::new(0),
                owned: [const { AtomicU32::new(0) }; 32],
                owned_index: AtomicU32::new(0),
            },
            GFP_KERNEL,
        )?;
        Ok(Self {
            scheduler,
            state,
            accepted: [0; 2],
            vm: vm_state,
            shared: gpu.shared.clone(),
            timeline,
        })
    }

    /// Validates a SUBMIT and pushes its job; returns the job's fence.
    fn submit_job(
        &mut self,
        id: u64,
        syncs: &Syncs,
        sync_count: usize,
        bytes: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result<Fence> {
        if sync_count > command::MAX_SYNCS {
            return Err(EINVAL);
        }
        command::check_stream(bytes, params::early_tiling())?;
        if self.shared.crashed.load(Ordering::Acquire) {
            return Err(EIO);
        }
        if self.vm.status.get() != 0 {
            return Err(EIO);
        }
        let prepared = command::prepare(bytes, self.accepted, objects)?;
        // The job reference first, so that the mappings checked below stay retained.
        let guard = self.vm.vm.retain_first_job()?;
        let submission = prepared.validate(&command::Context {
            vm: &self.vm.vm,
            usc_base: self.state.usc_base,
        })?;
        let mut inputs = KVec::with_capacity(syncs.inputs().len(), GFP_KERNEL)?;
        for sync in syncs.inputs() {
            if let Some(fence) = &sync.fence {
                inputs.push(fence.clone(), GFP_KERNEL)?;
            }
        }
        let fence = self.timeline.next()?;
        let job = Job::new(
            id,
            self.state.clone(),
            self.vm.clone(),
            submission.commands,
            fence,
            submission.timestamps,
            guard,
        )?;
        let published = job.fence.fence();
        self.scheduler.push(job, self.shared.clone(), inputs)?;
        // Accepted: the whole SUBMIT counts for later EXACT_PRIOR ordinals.
        self.accepted[command::RENDER] += submission.counts[command::RENDER];
        self.accepted[command::COMPUTE] += submission.counts[command::COMPUTE];
        Ok(published)
    }
}

impl crate::queue::Queue for Queue {
    fn submit(
        &mut self,
        id: u64,
        syncs: KVec<file::SyncItem>,
        in_sync_count: usize,
        cmdbuf_raw: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result {
        let sync_count = syncs.len();
        let syncs = Syncs::new(syncs, in_sync_count);
        match self.submit_job(id, &syncs, sync_count, cmdbuf_raw, objects) {
            Ok(fence) => {
                syncs.accept(&fence);
                Ok(())
            }
            Err(error) => {
                // A refused submission leaves the VM usable unless the device is lost.
                self.vm.status.record_if_device_loss(error);
                syncs.refuse(&self.timeline, error);
                Err(error)
            }
        }
    }
}
