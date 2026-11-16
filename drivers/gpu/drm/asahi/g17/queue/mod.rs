// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Userspace submission queues: validate whole ioctls, retain mappings, and enqueue
//! independent engines behind user fences and explicit logical prefixes.

pub(crate) mod compute;
pub(crate) mod render;

use super::{
    command::{Parser, UscWindow, Validated},
    completion::{Completion, Destinations, PreparedPublication},
    context::Context,
    fence::Outputs,
    job::{self, Backend, Engine, Fences, Order, Packet, Scheduler},
    validation::Proof,
};
use crate::{
    driver::{AsahiDevRef, AsahiDrmDevice},
    file, mmu,
};
use core::ops::Range;
use kernel::{
    c_str,
    dma_fence::{Fence, FenceContexts, RawDmaFence},
    prelude::*,
    sync::{Arc, LockClassKey},
    xarray,
};

static FENCE_KEY: Pin<&LockClassKey> = kernel::static_lock_class!();

pub(crate) mod admission;

/// Host identities for retained physical channels, separate from firmware object generations.
/// The device mutex protects allocation; cancelled preparations never reuse IDs.
pub(crate) struct Identities(u64);

impl Identities {
    pub(crate) const fn new() -> Self {
        Self(3)
    }

    /// The first two identities remain reserved with QIDs 0 and 1.
    pub(crate) fn reserve(&mut self, count: u64) -> Result<Range<u64>> {
        let first = self.0;
        let end = first.checked_add(count).ok_or(EOVERFLOW)?;
        self.0 = end;
        Ok(first..end)
    }
}

/// Validated frontend state prepared before allocating a logical execution context.
pub(crate) struct Frontend {
    fences: FenceContexts,
    allocation_failure: Fence,
    range: Range<u64>,
    window: UscWindow,
    variant: &'static crate::g17::fw::Variant,
}

impl Frontend {
    pub(crate) fn new(
        priority: u32,
        usc_base: u64,
        range: Range<u64>,
        variant: &'static crate::g17::fw::Variant,
    ) -> Result<Self> {
        if priority > 3 {
            return Err(EINVAL);
        }
        let window = UscWindow::new(usc_base, range.clone())?;
        let fences = FenceContexts::new(1, c_str!("asahi_g17_queue"), FENCE_KEY)?;
        let allocation_failure = super::fence::allocation_failure(&fences)?;
        Ok(Self {
            fences,
            allocation_failure,
            range,
            window,
            variant,
        })
    }
}

pub(crate) struct Queue<B: Backend> {
    dev: AsahiDevRef,
    context: Arc<Context>,
    backend: Arc<B>,
    render: Option<Scheduler<B>>,
    compute: Option<Scheduler<B>>,
    fences: FenceContexts,
    allocation_failure: Fence,
    range: Range<u64>,
    window: UscWindow,
    variant: &'static crate::g17::fw::Variant,
    proof: Proof,
    sequence: [u64; 2],
    frontiers: [KVec<(u64, Fences)>; 2],
    /// Finished frontier entries pruned during this ioctl. Their last references
    /// can tear a whole completion down; that waits until the ioctl has published.
    finished: KVec<Fences>,
    failures: [Option<(u64, Fence)>; 2],
    // Closing the userspace queue releases admission even if firmware still
    // retains one of its execution contexts or installed physical graphs.
    _slot: admission::QueueSlot,
}

type ValidatedBatch = (KVec<(Validated, Order, Destinations)>, KVec<mmu::VmJobGuard>);

impl<B: Backend> Queue<B> {
    pub(crate) fn new(
        dev: &AsahiDrmDevice,
        context: Arc<Context>,
        backend: Arc<B>,
        frontend: Frontend,
        slot: admission::QueueSlot,
    ) -> Result<Self> {
        let Frontend {
            fences,
            allocation_failure,
            range,
            window,
            variant,
        } = frontend;
        let render = Some(Scheduler::new(dev, backend.clone(), Engine::Render)?);
        Ok(Self {
            dev: dev.into(),
            context,
            backend,
            render,
            compute: None,
            fences,
            allocation_failure,
            range,
            window,
            variant,
            proof: Proof::new(),
            sequence: [0; 2],
            frontiers: core::array::from_fn(|_| KVec::new()),
            finished: KVec::new(),
            failures: [None, None],
            _slot: slot,
        })
    }

    fn ensure_compute(&mut self, scs_pages: u16) -> Result {
        // Validate every request, including an existing scheduler. Physical
        // admission happens after its DRM dependencies, in backend publication.
        self.backend.ensure_compute(scs_pages)?;
        if self.compute.is_none() {
            self.compute = Some(Scheduler::new(
                &self.dev,
                self.backend.clone(),
                Engine::Compute,
            )?);
        }
        Ok(())
    }

    fn prune_frontier(&mut self, index: usize) {
        let failure = &mut self.failures[index];
        // Without room the entries are released here, as they always could be.
        let mut finished = if self.finished.reserve(self.frontiers[index].len(), GFP_KERNEL).is_ok() {
            Some(&mut self.finished)
        } else {
            None
        };
        self.frontiers[index].retain(|(sequence, fences)| {
            let status = fences.status();
            if status == 0 {
                return true;
            }
            if let Some(finished) = finished.as_mut() {
                let _ = finished.push_within_capacity(fences.clone());
            }
            // One earliest failed ordinal covers every later prefix without
            // retaining its packet, mappings, or every failed fence.
            if status < 0 && failure.as_ref().is_none_or(|(first, _)| *sequence < *first) {
                *failure = Some((*sequence, fences.completed.clone()));
            }
            false
        });
    }

    #[expect(
        clippy::too_many_arguments,
        reason = "Enqueue keeps validated command, ordering, and destination owners explicit."
    )]
    fn enqueue(
        &mut self,
        id: u64,
        command: Validated,
        order: Order,
        guard: mmu::VmJobGuard,
        aggregate: &super::fence::Aggregate,
        inputs: &[file::SyncItem],
        destinations: Destinations,
    ) -> Result {
        let engine = match &command {
            Validated::Render { .. } => Engine::Render,
            Validated::Compute { .. } => Engine::Compute,
        };
        let index = engine.index();
        if let Validated::Render { ray_query, .. } = &command {
            let blocks = self.backend.render_scs_blocks(ray_query.map(|stage| stage.scratch.pages()))?;
            if blocks > self.context.render_blocks() { return Err(EINVAL); }
        }
        self.prune_frontier(index);
        self.frontiers[index].reserve(1, GFP_KERNEL)?;
        if engine == Engine::Compute {
            let Validated::Compute { scratch, .. } = &command else { return Err(EINVAL); };
            self.ensure_compute(scratch.pages())?;
        }
        // Only convert inputs when no independent host-prefix wait could keep
        // the retained source bookkeeping queued behind an external signal.
        let sources = if engine == Engine::Compute && !inputs.is_empty()
            && !destinations.has_outputs()
            && self.frontiers[0].iter().all(|(sequence, fences)|
                !Order::contains(order.wait_through[0], *sequence)
                    || fences.status() > 0)
            && self.frontiers[1].iter().all(|(sequence, fences)|
                !Order::contains(order.wait_through[1], *sequence)
                    || fences.ready.raw() != fences.completed.raw()
                    || fences.status() > 0)
            && self.failures.iter().enumerate().all(|(index, failure)|
                failure.as_ref().is_none_or(|(sequence, _)|
                    !Order::contains(order.wait_through[index], *sequence)))
        { self.backend.input_sources(inputs) } else { None };

        // Only earlier render ordinals covered by the explicit logical prefix
        // can delay host publication. These links never become GPU dependencies.
        let host_predecessors = self.frontiers[index]
            .iter()
            .take_while(|(sequence, _)| {
                engine == Engine::Render
                    && Order::contains(order.wait_through[index], *sequence)
            })
            .filter_map(|(_, fences)| fences.publication.as_ref());
        let completion = Completion::new(
            &self.dev,
            &self.fences,
            &self.context,
            aggregate.member(),
            guard,
            destinations,
            engine == Engine::Compute,
            host_predecessors.clone().next().is_some(),
            self.backend.feed(),
            sources.clone(),
        )?;
        let publication = completion.publication()
            .map(|target| PreparedPublication::new(target, host_predecessors))
            .transpose()?;
        if engine == Engine::Compute {
            self.prune_frontier(Engine::Render.index());
        }
        let other = 1 - index;
        let prefix = order.wait_through[other];
        let frontier = if prefix.is_some() {
            let frontier = self.frontiers[other].as_slice();
            // Accepted sequences increase from one, and retain preserves their
            // order, so all members of a logical prefix form one slice.
            let end = frontier.partition_point(|(sequence, _)| Order::contains(prefix, *sequence));
            &frontier[..end]
        } else {
            &[]
        };
        let count = frontier.len();
        let timestamp_prefix = order.wait_through[index];
        let timestamps = if engine == Engine::Compute && timestamp_prefix.is_some() {
            let frontier = self.frontiers[index].as_slice();
            let end = frontier
                .partition_point(|(sequence, _)| Order::contains(timestamp_prefix, *sequence));
            &frontier[..end]
        } else {
            &[]
        };
        let timestamp_count = timestamps
            .iter()
            .filter(|(_, fences)| fences.ready.raw() == fences.completed.raw())
            .count();
        let failures: [Option<&Fence>; 2] = core::array::from_fn(|index| {
            self.failures[index].as_ref().and_then(|(sequence, fence)| {
                Order::contains(order.wait_through[index], *sequence).then_some(fence)
            })
        });
        let failure_count = failures.iter().flatten().count();
        let mut dependencies =
            KVec::with_capacity(inputs.len() + count + timestamp_count + failure_count, GFP_KERNEL)?;
        let mut checked_inputs =
            KVec::with_capacity(inputs.len() + timestamp_count + failure_count, GFP_KERNEL)?;
        let mut firmware = KVec::with_capacity(count, GFP_KERNEL)?;
        for fence in failures.into_iter().flatten() {
            checked_inputs.push(fence.clone(), GFP_KERNEL)?;
            dependencies.push(fence.clone(), GFP_KERNEL)?;
        }
        // Same-engine parents order GPU execution, not the host's timestamp
        // writes. Only timestamp publishers need these completion waits and
        // status checks; ordinary compute keeps its existing queue ordering.
        for (_, fences) in timestamps {
            if fences.ready.raw() == fences.completed.raw() {
                checked_inputs.push(fences.completed.clone(), GFP_KERNEL)?;
                dependencies.push(fences.ready.clone(), GFP_KERNEL)?;
            }
        }
        for (_, fences) in frontier {
            // Successful completion includes host timestamp publication. Its
            // ready fence adds no ordering and would prevent shallow inline
            // publication. Failed and pending producers still need checking.
            if fences.status() > 0 {
                continue;
            }
            dependencies.push(fences.ready.clone(), GFP_KERNEL)?;
            firmware.push(fences.completed.clone(), GFP_KERNEL)?;
        }
        // Each command owns every input fence; independent engines may publish in
        // either order and must each consume an input's error status before running.
        for sync in inputs {
            if let Some(fence) = &sync.fence {
                // An input that already signalled without error orders nothing and has no status
                // left to consume (like a completed frontier producer above). Keeping it would
                // send the render through the scheduler thread instead of inline publication:
                // most compositor and client renders carry such already-signalled buffer fences.
                if job::fence_status(fence) > 0 {
                    continue;
                }
                if sources.as_ref().is_some_and(|sources| sources.contains_input(fence)) {
                    continue;
                }
                dependencies.push(fence.clone(), GFP_KERNEL)?;
                checked_inputs.push(fence.clone(), GFP_KERNEL)?;
            }
        }
        if let Some(sources) = &sources {
            for ready in sources.publication_waits() {
                dependencies.push(ready.clone(), GFP_KERNEL)?;
                checked_inputs.push(ready.clone(), GFP_KERNEL)?;
            }
        }
        // Publish from the ioctl whenever nothing has to wait, at any depth.
        // The backend falls back to the scheduler behind any backlog or on any
        // capacity miss, so submission order is preserved.
        let early = dependencies.is_empty();
        let packet = Packet::new(
            id,
            self.context.clone(),
            command,
            order,
            completion,
            checked_inputs,
            firmware,
        )?;
        let scheduler = match engine {
            Engine::Render => &mut self.render,
            Engine::Compute => &mut self.compute,
        };
        let fences = scheduler
            .as_mut()
            .ok_or(ENODEV)?
            .enqueue(packet, dependencies, early, publication)?;
        self.sequence[index] = order.sequence;
        self.frontiers[index]
            .push_within_capacity((order.sequence, fences))
            .map_err(|_| ENOMEM)?;
        Ok(())
    }
    /// Finish parsing, validation and VM pinning before entering publication.
    /// Keeping this frame separate bounds the scheduler/ioctl stack depth.
    #[inline(never)]
    fn prepare_submission(
        &mut self,
        bytes: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result<ValidatedBatch> {
        let parse_guard = self.context.vm().retain_first_job()?;
        let mut parser = Parser::new(bytes);
        // All historical ordinals use the same entry counters; publication
        // starts only after the entire ioctl has been validated and pinned.
        let base = self.sequence;
        let mut preceding = [0; 2];
        let mut pending = KVec::with_capacity(1, GFP_KERNEL)?;
        while let Some(command) = parser.next()? {
            let payload = self.proof.validate(
                self.context.vm(),
                self.range.clone(),
                &self.window,
                self.variant,
                &command.payload,
            )?;
            let engine = match &payload {
                Validated::Render { .. } => Engine::Render,
                Validated::Compute { .. } => Engine::Compute,
            };
            let order = Order::new(
                base,
                preceding,
                command.barriers,
                command.payload.prior(),
                engine,
            )?;
            preceding[engine.index()] += 1;
            let destinations = match &payload {
                Validated::Render { timestamps, .. } => Destinations::resolve(objects, timestamps)?,
                Validated::Compute { timestamps, .. } => {
                    Destinations::resolve(objects, core::slice::from_ref(timestamps))?
                }
            };
            pending.push((payload, order, destinations), GFP_KERNEL)?;
        }
        parser.finish()?;
        // Every accepted command pins the VM before any command is enqueued.
        let mut guards = KVec::with_capacity(pending.len(), GFP_KERNEL)?;
        guards
            .push_within_capacity(parse_guard)
            .map_err(|_| ENOMEM)?;
        while guards.len() < pending.len() {
            guards
                .push_within_capacity(self.context.vm().retain_job()?)
                .map_err(|_| ENOMEM)?;
        }
        Ok((pending, guards))
    }
}

impl<B: Backend> crate::queue::Queue for Queue<B> {
    fn reserve_render_scs(&mut self, layouts: [u64; 2]) -> Result {
        let pages = [super::fw::compute::ScratchRequest::new(layouts[0])?.pages(),
                     super::fw::compute::ScratchRequest::new(layouts[1])?.pages()];
        let blocks = self.backend.render_scs_blocks(pages)?;
        self.context.reserve_render_blocks(blocks)
    }

    fn submit(
        &mut self,
        _reg_data: &crate::driver::AsahiRegData<'_>,
        id: u64,
        syncs: KVec<file::SyncItem>,
        input_count: usize,
        bytes: &[u8],
        objects: Pin<&xarray::XArray<KBox<file::Object>>>,
    ) -> Result {
        let mut outputs = Outputs::new(
            &self.fences,
            syncs,
            input_count,
            &self.allocation_failure,
        )?;
        let result = (|| {
            if self.context.status().get() != 0 {
                return Err(EIO);
            }
            let (mut pending, guards) = self.prepare_submission(bytes, objects)?;
            if pending.iter().any(|(command, _, _)| matches!(command, Validated::Render { .. })) {
                self.context.note_render();
            }
            if super::timeout::select_submission(pending.iter().map(|(cmd, _, _)| cmd.long_job())) {
                for (command, _, _) in pending.iter_mut() { command.allow_long_job(); }
            }
            let mut guards = guards.into_iter();
            let firmware_source = pending.len() == 1
                && matches!(&pending[0].0, Validated::Compute { .. })
                && !pending[0].2.has_outputs();
            let aggregate = outputs.publish(self.context.clone(), firmware_source)?;
            for (command, order, destinations) in pending {
                let guard = guards.next().ok_or(EIO)?;
                self.enqueue(
                    id,
                    command,
                    order,
                    guard,
                    &aggregate,
                    outputs.inputs(),
                    destinations,
                )?;
            }
            Ok(())
        })();
        // Everything this ioctl publishes is visible; release what it pruned.
        self.finished.clear();
        if let Err(error) = result {
            self.context.status().record_if_device_loss(error);
            if [EIO, ETIMEDOUT, ENODEV].contains(&error) || self.context.status().get() != 0 {
                self.backend.fail_vm(error);
            }
        }
        outputs.finish(result)
    }
}

impl<B: Backend> Drop for Queue<B> {
    fn drop(&mut self) {
        drop(self.render.take());
        drop(self.compute.take());
        self.backend.release_owner();
    }
}
