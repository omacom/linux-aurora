// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Installed kick storage retained after a logical scheduler release is acknowledged.

use super::{install_memory, memory, target, Pair, RetiredBinding, Tracker};
use crate::g17::{
    buffer,
    command::Validated,
    context::Context,
    freelist::RenderPool,
    job::Packet,
    kick,
    object::{Allocator, KernelObject, Pool},
};
use crate::mmu;
use kernel::{prelude::*, sync::Arc};

/// The empty physical pair retains installed addresses and the last scheduler page,
/// without retaining that scheduler's execution context or client root. A prior
/// context awaiting an executed replacement keeps its separate retirement owner.
pub(crate) struct ParkedPair {
    kicks: Option<[kick::Queue; 2]>,
    _scheduler: Arc<KernelObject>,
    ids: [kick::Id; 2],
    slot: u8,
    ordinal: u64,
    clusters: u32,
    deflake: crate::g17::fw::render::Deflake,
    descriptor_flags: [u32; 2],
    variant: &'static crate::g17::fw::Variant,
    installed: [bool; 2],
    recovery_generation: u64,
    registered: Option<u8>,
    outer_started: u8,
    previous: Option<RetiredBinding>,
    /// Scheduler page these queues last ran a command for. A pair parked
    /// without a consumed release (an unused warm pair taken for another
    /// owner) keeps it, so its next owner change still waits while that
    /// scheduler is released.
    executed: u64,
}

/// All new graph ownership stays separate from the installed kick storage until commit.
pub(crate) struct Prepared {
    memory: Option<KBox<memory::Memory>>,
    pool: Arc<RenderPool>,
    aliases: Option<KVec<mmu::KernelMapping>>,
    kicks: Option<[Option<mmu::KernelMapping>; 2]>,
    ids: [kick::Id; 2],
    slot: u8,
    ordinal: u64,
    owner: u64,
    initial_packet: u64,
    installed: bool,
}

impl Pair {
    /// The current graph never entered command publication. Physical identities
    /// may have older users; parking retains their rings and retirement owners.
    pub(crate) fn can_yield_unused(&self) -> Result<bool> {
        Ok(!self.graph_started && self.can_park_backing(self.context())?)
    }

    /// The registry also checks the shared free-list release and scheduler-release witness.
    /// Drain displaced owners before this preflight; they have their own logical releases.
    pub(crate) fn can_park(&self, context: &Arc<Context>) -> Result<bool> {
        Ok(self.owner.is_none() && self.can_park_backing(context)?)
    }

    fn can_park_backing(&self, context: &Arc<Context>) -> Result<bool> {
        Ok(Arc::ptr_eq(context, self.context())
            && self
                .previous
                .as_ref()
                .is_none_or(|previous| previous.memory.is_none() && !previous.ready)
            && self.displaced.is_none()
            && self.pending_aliases.is_none()
            && self.kick_aliases.is_none()
            && self.idle_for_context(context)?)
    }

    /// Consumes the pair only after can_park and all collector capacity checks succeeded
    /// under the same device mutex. Moving these owners performs no firmware operation.
    pub(crate) fn park(self) -> (ParkedPair, RetiredBinding) {
        let context = self.memory.context.clone();
        let scheduler = context.scheduler_owner();
        let buffer_id = self.buffer_id();
        let parked = ParkedPair {
            ids: self.kicks.each_ref().map(|queue| queue.id()),
            kicks: Some(self.kicks),
            _scheduler: scheduler,
            slot: self.slot,
            ordinal: self.ordinal,
            clusters: self.clusters,
            deflake: self.deflake,
            descriptor_flags: self.descriptor_flags,
            variant: self.variant,
            installed: self.installed,
            recovery_generation: self.recovery_generation,
            registered: self.registered,
            outer_started: self.outer_started,
            previous: self.previous,
            executed: self.executed,
        };
        let retired = RetiredBinding {
            memory: Some(self.memory),
            context,
            pool: self.free_list,
            buffer_id,
            publication: None,
            ready: true,
        };
        (parked, retired)
    }
}

impl ParkedPair {
    /// The release of `scheduler` has been consumed: a later owner change no
    /// longer has to wait for it.
    pub(crate) fn release_consumed(&mut self, scheduler: u64) {
        if self.executed == scheduler {
            self.executed = 0;
        }
    }
    pub(crate) fn qids(&self) -> [u8; 2] {
        self.ids.map(kick::Id::qid)
    }

    #[inline(never)]
    #[expect(
        clippy::too_many_arguments,
        reason = "Preparation forwards the render binding inputs without changing their ownership."
    )]
    pub(crate) fn prepare(
        &self,
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        owner: u64,
        context: Arc<Context>,
        free_list: Arc<RenderPool>,
        metrics: buffer::MetricsLease,
        buffer_id: (u8, u64),
        pm_generation: u64,
        first: &Arc<Packet>,
    ) -> Result<Prepared> {
        if owner == 0 || !Arc::ptr_eq(&context, &first.context) || !context.is_current() {
            return Err(EINVAL);
        }
        let kicks = self.kicks.as_ref().ok_or(EBUSY)?;
        let Validated::Render { pass, .. } = &first.command else {
            return Err(EINVAL);
        };
        let word = first.completion.work_state()?.word();
        if word == u32::MAX {
            return Err(EINVAL);
        }
        alloc.uat.vm_context_mask(context.vm())?;
        let layout = target::Layout::new(pass, self.clusters, self.variant.programs)?;
        let aliases = [
            kicks[0].prepare_client_alias(context.vm())?,
            kicks[1].prepare_client_alias(context.vm())?,
        ];
        let (memory, persistent) = memory::Memory::new(
            alloc,
            pool,
            context,
            self.qids(),
            &layout,
            metrics,
            buffer_id,
            pm_generation,
            self.ordinal,
            word,
            self.variant,
        )?;
        Ok(Prepared {
            memory: Some(memory),
            pool: free_list,
            aliases: Some(persistent),
            kicks: Some(aliases),
            ids: self.ids,
            slot: self.slot,
            ordinal: self.ordinal,
            owner,
            initial_packet: first.id,
            installed: false,
        })
    }

    fn matches(&self, prepared: &Prepared) -> bool {
        self.kicks.is_some()
            && self.ids == prepared.ids
            && self.slot == prepared.slot
            && self.ordinal == prepared.ordinal
            && prepared
                .memory
                .as_ref()
                .is_some_and(|memory| memory.context.is_current())
    }

    /// All fallible graph installation precedes taking the installed kicks. Keep the empty
    /// parked owner until the caller drops it off-lock, including its old scheduler page.
    pub(crate) fn commit(
        &mut self,
        prepared: &mut Option<Prepared>,
        storage: KBox<core::mem::MaybeUninit<Pair>>,
    ) -> Result<KBox<Pair>> {
        let candidate = prepared.as_ref().ok_or(EINVAL)?;
        if !candidate.installed || !self.matches(candidate) {
            return Err(EBUSY);
        }
        let kicks = self.kicks.take().ok_or(EIO)?;
        // The candidate was validated above and cannot change through the exclusive borrow.
        let Some(prepared) = prepared.take() else {
            self.kicks = Some(kicks);
            return Err(EIO);
        };
        let mut next = prepared;
        let Some(memory) = next.memory.take() else {
            self.kicks = Some(kicks);
            return Err(EIO);
        };
        storage.write_init(kernel::try_init!(Pair {
            graph_started: false,
            kick_aliases: None,
            memory,
            kicks,
            free_list: next.pool.clone(),
            pending_aliases: None,
            prepared: None,
            active: Tracker::new(),
            slot: self.slot,
            owner: Some(next.owner),
            ordinal: self.ordinal,
            initial_packet: next.initial_packet,
            clusters: self.clusters,
            deflake: self.deflake,
            descriptor_flags: self.descriptor_flags,
            variant: self.variant,
            installed: self.installed,
            configs: [None; 2],
            cursors: [0; 2],
            recovery_generation: self.recovery_generation,
            registered: self.registered,
            outer_started: self.outer_started,
            quarantined: false,
            terminal: None,
            previous: self.previous.take(),
            displaced: None,
            replay: Tracker::new(),
            replay_since: None,
            context_update: true,
            executed: self.executed,
            switch: None,
            published_completions: None,
        }))
    }
}

impl Prepared {
    /// Called after the registry revalidates its preparation epoch and parked pair identity.
    pub(crate) fn finish_install(
        &mut self,
        parked: &ParkedPair,
        alloc: &Allocator<'_>,
        descriptors: &KernelObject,
    ) -> Result {
        if !parked.matches(self) {
            return Err(EBUSY);
        }
        install_memory(
            alloc,
            descriptors,
            self.slot,
            self.memory.as_mut().ok_or(EIO)?,
            &mut self.aliases,
            &mut self.kicks,
        )?;
        self.installed = true;
        Ok(())
    }
}
