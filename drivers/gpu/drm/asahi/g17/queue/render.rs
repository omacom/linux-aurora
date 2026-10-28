// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Physical paired tiling and fragment queues and their retained command storage.

mod observation;
pub(crate) use observation::Observed;
mod dependencies;
mod memory;
pub(crate) mod parked;
pub(super) mod publication;
pub(crate) mod retirement;
mod warm;

use crate::g17::{
    buffer::{self, target},
    command::Validated,
    completion::Deferred,
    context::Context,
    freelist::RenderPool,
    fw::{
        channels::WorkSlot,
        kick::{KickDependency, KickTimestamp, McacheTable, RenderRegistration},
        queue::DataMaster,
        render as abi,
    },
    job::Packet,
    kick,
    object::{Allocator, Pool},
    qos, recovery,
    status::VmStatus,
};
use crate::{hw::t8140::queues, mmu};
pub(crate) use dependencies::Dependencies;
use kernel::{prelude::*, sync::Arc};
use retirement::{Ticket, Tracker};

/// Firmware dependencies of one render, kept by origin until the kick entries
/// are built: the covered earlier renders (their fragment completions) and the
/// covered compute completion. Which kick waits for the render prefix is the
/// command's choice (`RenderPass::fragment_barriers`); compute always orders
/// the tiling kick, since compute output may feed vertex work.
#[derive(Copy, Clone)]
pub(crate) struct RenderDependencies {
    pub(crate) render: [KickDependency; 2],
    pub(crate) render_count: usize,
    pub(crate) tiling: [KickDependency; 2],
    pub(crate) tiling_count: usize,
    pub(crate) compute: Option<KickDependency>,
    /// Fragment kicks of other pairs' overlapping same-VM renders.
    pub(crate) hazards: [KickDependency; crate::g17::hazard::PUBLICATION_BARRIERS],
    pub(crate) hazard_count: usize,
}

impl RenderDependencies {
    pub(crate) const EMPTY: Self = Self {
        render: [KickDependency::ZERO; 2],
        render_count: 0,
        tiling: [KickDependency::ZERO; 2],
        tiling_count: 0,
        compute: None,
        hazards: [KickDependency::ZERO; crate::g17::hazard::PUBLICATION_BARRIERS],
        hazard_count: 0,
    };
    pub(crate) fn render(&self) -> &[KickDependency] {
        &self.render[..self.render_count]
    }
}

#[derive(Copy, Clone)]
pub(crate) struct RenderAddresses {
    /// Tiling then fragment; each pair contains the primary and auxiliary stamp.
    pub(crate) stamps: [[u64; 2]; 2],
}

#[derive(Copy, Clone)]
pub(crate) struct PublicationLease {
    pub(crate) free_list_generation: u64,
    pub(crate) buffer_token: u64,
}

/// Host services borrow the device-locked firmware state. Resource construction
/// and command geometry preparation do not borrow this interface.
pub(crate) trait Host {
    fn epoch(&self) -> Result<(u64, u32)>;
    fn stamps(&self, qids: [u8; 2]) -> Result<[u32; 2]>;
    /// Publishes `owners` in order with one pass over the QoS table.
    fn qos_publish_pair(
        &mut self,
        owners: [qos::Owner; 2],
        scheduler: u64,
        policy: crate::g17::fw::queue::Policy,
    ) -> Result<[qos::Publication; 2]>;
    fn qos_cancel(&mut self, publication: qos::Publication) -> Result;
    fn install_pair(&mut self, registration: &RenderRegistration) -> Result;
    fn publish_qid(&mut self, id: kick::Id) -> Result;
    /// The next publication number, shared with compute (timeout ordering).
    fn next_publication(&mut self) -> Result<u64>;
    fn publish_outer_pair(
        &mut self,
        priority: u8,
        fragment: &crate::g17::fw::channels::WorkSlot,
        tiling: &crate::g17::fw::channels::WorkSlot,
        commit: impl FnOnce() -> Result,
        late_tiling: impl FnOnce(),
    ) -> Result<[u32; 2]>;
    fn activate_render_pool(&mut self, id: u16, generation: u64) -> Result;
    fn publish_retained_pair(
        &mut self,
        priority: u8,
        fragment: &WorkSlot,
        tiling: &WorkSlot,
        next: [u32; 2],
    ) -> Result;
    fn notify_render(&mut self, priority: u8) -> Result;
    fn note_submission(&mut self) -> Result;
    fn complete_render(
        &mut self,
        owners: Option<[qos::Owner; 2]>,
        pool_id: u16,
        generation: u64,
        buffer_id: u8,
    ) -> Result;
}

struct Prepared {
    packet: Arc<Packet>,
    ordinal: u64,
    mcache: Option<McacheTable>,
}
struct Active {
    /// Publication number, shared with compute (crate::g17::timeout).
    published: u64,
    qos_completed: bool,
    flist_completed: bool,
    outer_published: bool,
    generation: u64,
    packet: Arc<Packet>,
    ticket: Ticket,
    kicks: [KickTimestamp; 2],
    context_update: bool,
    failed: bool,
    drained_result: Option<Result<[u64; 4]>>,
    /// First observation of this group retired and unlinked without execution.
    abandoned_since: Option<u64>,
    /// First observation of this group executed and consumed while its stamps lagged.
    stamp_wait_since: Option<u64>,
}

/// A firmware-retired, unexecuted group must hold on two observations this far
/// apart before it is republished; a single sample may straddle an update.
const ABANDON_GRACE_NS: u64 = 2_000_000;

/// Installed kick backing remains owned even when this pair has no logical
/// owner. Commands retain their exact descriptor and allocation generations.
pub(crate) struct Pair {
    kick_aliases: Option<[Option<mmu::KernelMapping>; 2]>,
    memory: KBox<memory::Memory>,
    kicks: [kick::Queue; 2],
    free_list: Arc<RenderPool>,
    pending_aliases: Option<KVec<mmu::KernelMapping>>,
    prepared: Option<Prepared>,
    active: Tracker<Active>,
    slot: u8,
    owner: Option<u64>,
    ordinal: u64,
    initial_packet: u64,
    graph_started: bool,
    clusters: u32,
    deflake: abi::Deflake,
    descriptor_flags: [u32; 2],
    variant: &'static crate::g17::fw::Variant,
    installed: [bool; 2],
    configs: [Option<crate::g17::fw::queue::QueueConfig>; 2],
    cursors: [u32; 2],
    recovery_generation: u64,
    /// Priority class of the pair's installed registration, if any. A pair
    /// rebound to an owner of another class is registered again.
    registered: Option<u8>,
    /// Bit `p` set once a work slot of priority class `p` named the pair.
    outer_started: u8,
    quarantined: bool,
    terminal: Option<(Arc<VmStatus>, Error)>,
    previous: Option<RetiredBinding>,
    displaced: Option<RetiredBinding>,
    context_update: bool,
    /// Retired, never-executed groups awaiting republication on this pair, in
    /// their original order. Nothing else is prepared on the pair before them.
    replay: Tracker<Arc<Packet>>,
    /// When the oldest parked group was parked or the last one republished.
    replay_since: Option<u64>,
    /// Scheduler page of the context of the last published command: the
    /// scheduler the firmware last ran these queues for. Zero for a fresh graph.
    executed: u64,
    /// An owner change in flight: the previous scheduler page and the ordinal of
    /// the new owner's first command, until that command leaves the pair.
    switch: Option<(u64, u64)>,
    /// Completion values of the two kicks of the last publication, kept with the configuration
    /// cache: what each queue holds once that publication completed.
    published_completions: Option<[u64; 2]>,
}
impl Pair {
    #[inline(never)]
    #[expect(
        clippy::too_many_arguments,
        reason = "The packet constructor spells out the paired render firmware inputs."
    )]
    pub(crate) fn new(
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        rings: &Arc<kick::RenderRings>,
        ids: [kick::Id; 2],
        slot: u8,
        owner: u64,
        context: Arc<Context>,
        free_list: Arc<RenderPool>,
        metrics: buffer::MetricsLease,
        buffer_id: (u8, u64),
        pm_generation: u64,
        first: &Arc<Packet>,
        clusters: u32,
        deflake: abi::Deflake,
        descriptor_flags: [u32; 2],
        variant: &'static crate::g17::fw::Variant,
    ) -> Result<KBox<Self>> {
        if slot == 0
            || usize::from(slot) >= queues::RENDER_SLOTS
            || owner == 0
            || ids[0].engine() != DataMaster::Tiling
            || ids[1].engine() != DataMaster::Fragment
            || !Arc::ptr_eq(&context, &first.context)
            || !context.is_current()
        {
            return Err(EINVAL);
        }
        if let Err(error) = alloc.uat.vm_context_mask(context.vm()) {
            if error == ENOENT {
                context.status().record(error);
            }
            return Err(error);
        }
        let Validated::Render { pass, .. } = &first.command else {
            return Err(EINVAL);
        };
        let word = first.completion.work_state()?.word();
        if word == u32::MAX {
            return Err(EINVAL);
        }
        let layout = target::Layout::new(pass, clusters, variant.programs)?;
        let storage = KBox::<Self>::new_uninit(GFP_KERNEL)?;
        let (tiling, ta_alias) = kick::Queue::render(alloc, ids[0], context.vm(), rings)?;
        let (fragment, fragment_alias) =
            kick::Queue::render(alloc, ids[1], context.vm(), rings)?;
        let (memory, aliases) = memory::Memory::new(
            alloc,
            pool,
            context,
            [ids[0].qid(), ids[1].qid()],
            &layout,
            metrics,
            buffer_id,
            pm_generation,
            0,
            word,
            variant,
        )?;
        storage.write_init(kernel::try_init!(Self {
            kick_aliases: Some([ta_alias, fragment_alias]),
            memory,
            kicks: [tiling, fragment],
            free_list,
            pending_aliases: Some(aliases),
            prepared: None,
            active: Tracker::new(),
            slot,
            owner: Some(owner),
            ordinal: 0,
            initial_packet: first.id,
            graph_started: false,
            clusters,
            deflake,
            descriptor_flags,
            variant,
            installed: [false; 2],
            configs: [None; 2],
            cursors: [0; 2],
            recovery_generation: 0,
            registered: None,
            outer_started: 0,
            quarantined: false,
            terminal: None,
            previous: None,
            displaced: None,
            replay: Tracker::new(),
            replay_since: None,
            context_update: true,
            executed: 0,
            switch: None,
            published_completions: None,
        }))
    }
    pub(crate) fn qids(&self) -> [u8; 2] {
        self.kicks.each_ref().map(|kick| kick.id().qid())
    }
    pub(crate) fn slot(&self) -> u8 {
        self.slot
    }
    pub(crate) fn owner(&self) -> Option<u64> {
        self.owner
    }
    pub(crate) fn context(&self) -> &Arc<Context> {
        &self.memory.context
    }
    pub(crate) fn pool(&self) -> &Arc<RenderPool> {
        &self.free_list
    }
    pub(crate) fn pool_id(&self) -> u64 {
        u64::from(self.memory.lifecycle[0].predecessor)
    }
    pub(crate) fn buffer_id(&self) -> u8 {
        self.memory.manager.buffer_id()
    }
    pub(crate) fn manager(&mut self) -> &mut buffer::Manager {
        &mut self.memory.manager
    }
    pub(crate) fn in_flight(&self) -> bool {
        self.active.len() != 0 || self.replay.len() != 0
    }
    /// A replay or an abandonment grace period needs the worker to poll.
    pub(crate) fn replay_polling(&self) -> bool {
        self.replay.len() != 0 || self.active.iter().any(|a| a.abandoned_since.is_some())
    }
    pub(crate) fn replay_stalled(&self, now: u64) -> bool {
        self.replay.len() != 0 && crate::g17::timeout::background_stalled(self.replay_since, now)
    }
    pub(crate) fn replay_front(&self) -> Option<Arc<Packet>> {
        self.replay.front().cloned()
    }
    pub(crate) fn finish_replay(&mut self, packet: &Arc<Packet>) -> Result {
        if !self.replay.front().is_some_and(|front| Arc::ptr_eq(front, packet)) {
            return Err(EIO);
        }
        self.replay.pop();
        // Progress: the expiry restarts for the next parked group, if any.
        self.replay_since = (self.replay.len() != 0).then(crate::g17::now_ns);
        Ok(())
    }
    pub(crate) fn published_in_flight(&self) -> bool {
        !self.quarantined && self.active.iter().any(|active| active.outer_published)
    }
    pub(crate) fn room(&self) -> bool {
        !self.quarantined && self.owner.is_some() && self.prepared.is_none() && self.active.room()
    }
    pub(crate) fn take_terminal(&mut self) -> Option<(Arc<VmStatus>, Error)> {
        self.terminal.take()
    }

    /// The registry owns this pair exclusively during potentially allocating
    /// preparation. No queue producer or shared submission counter changes.
    pub(crate) fn prepare(
        &mut self,
        alloc: &Allocator<'_>,
        packet: &Arc<Packet>,
        addresses: RenderAddresses,
    ) -> Result {
        if !self.room()
            || self.replay.front().is_some_and(|front| !Arc::ptr_eq(front, packet))
            || !Arc::ptr_eq(&self.memory.context, &packet.context)
            || !packet.context.is_current()
        {
            return Err(EBUSY);
        }
        let Validated::Render {
            pass, attachments, ray_query, ..
        } = &packet.command
        else {
            return Err(EINVAL);
        };
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let ordinal = self.ordinal;
        ordinal.checked_add(1).ok_or(EOVERFLOW)?;
        self.memory.prefetch_descriptors(ordinal);
        let work = packet.completion.work_state()?;
        let node = work.node_va()?;
        let word = work.word();
        let layout = target::Layout::new(pass, self.clusters, self.variant.programs)?;
        let deflake_layout = self.deflake;
        let owners = self.memory.owners();
        self.memory
            .target
            .ensure_capacity(alloc.dev, packet.context.vm(), &layout, &owners)?;
        let target = self.memory.target.prepare(ordinal, &layout)?;
        let scene = self.memory.manager.prepare_scene(ordinal)?;
        self.memory
            .generation(ordinal)?
            .prepare(ordinal, pass, deflake_layout.viewport)?;
        let [deflake, ta_status, fragment_status, fragment_fw, auxiliary] =
            self.memory.generation(ordinal)?.addresses();
        let lifecycle = if ordinal != 0 || packet.id != self.initial_packet {
            let (fragment, tiling) =
                crate::g17::ids::OBJECTS.render_pair(self.memory.lifecycle[0].predecessor)?;
            [tiling, fragment]
        } else {
            self.memory.lifecycle
        };
        let (scratch, metric) = self.memory.manager.scene_registers(scene)?;
        let timestamp_offset = ordinal as usize % memory::RECORDS * 16;
        let timestamp = self.memory.timestamps.gpu_va() + timestamp_offset as u64;
        let resources = abi::RenderResources {
            context_id: packet.context.id().try_into()?,
            context_generation: packet.context.generation(),
            pb_slot: self.buffer_id().into(),
            free_list_slot: self.free_list.id().into(),
            free_list_control_va: self.free_list.control_va(),
            kick_record_va: node,
            pb_va: self.memory.manager.state_va(),
            scene_va: self.memory.manager.scene_va(scene)?,
            scene_end_va: self.memory.manager.trailer_va(),
            pm_scratch: scratch,
            pm_metric: metric,
            tilemap_va: target.tilemap,
            layermeta_va: target.layer,
            heapmeta_va: target.heap,
            tpc_va: self.memory.target.tpc_va(),
            tpc_size: layout.tpc_size as u64,
            gtp_va: self.memory.target.gtp_va(&layout)?,
            deflake_va: deflake,
            ta_status_va: ta_status,
            fragment_status_va: fragment_status,
            fragment_status_fw_va: fragment_fw,
            aux_fb_va: auxiliary,
            timestamp_va: timestamp,
            ta_ids: lifecycle[0],
            fragment_ids: lifecycle[1],
            unk_2104: self.descriptor_flags[0],
            unk_210c: self.descriptor_flags[1],
        };
        let qids = self.qids();
        let args = abi::RenderArgs {
            pass,
            ray_query: *ray_query,
            resources: &resources,
            ordinal,
            ta: abi::RenderStage {
                qid: qids[0],
                descriptor_va: self.memory.graph.descriptor_client(0, ordinal),
                state_word: word,
                stamp_va: addresses.stamps[0][0],
                aux_stamp_va: addresses.stamps[0][1],
            },
            fragment: abi::RenderStage {
                qid: qids[1],
                descriptor_va: self.memory.graph.descriptor_client(1, ordinal),
                state_word: word,
                stamp_va: addresses.stamps[1][0],
                aux_stamp_va: addresses.stamps[1][1],
            },
            programs: self.variant.programs,
            deflake: deflake_layout,
        };
        self.memory.graph.claim(ordinal)?;
        // Without an MCache table kick-entry bit 58 stays clear.
        let attachments = match self.variant.mcache {
            true => attachments[1].as_slice(),
            false => &[][..],
        };
        let result = self.memory.write_descriptors(
            &args,
            attachments,
            packet.context.id().try_into()?,
        );
        let mcache = match result {
            Ok(mcache) => mcache,
            Err(error) => {
                self.memory.graph.release(ordinal)?;
                self.memory.generation(ordinal)?.ordinal = None;
                self.memory.target.cancel(ordinal);
                return Err(error);
            }
        };
        self.memory.timestamps.write(timestamp_offset, [0u64; 2])?;
        self.memory.manager.claim_scene(ordinal, scene)?;
        self.prepared = Some(Prepared {
            packet: packet.clone(),
            ordinal,
            mcache,
        });
        Ok(())
    }
}

/// Kick barriers of one render. The fragment kick always waits for its own tiling kick;
/// with the fragment-barriers flag the resolved prefix orders the fragment kick too and the
/// tiling kick preserves the TA prefix across physical pairs. Otherwise the render prefix
/// orders the tiling kick. A compute dependency always orders the tiling kick.
struct RenderBarriers {
    tiling: [KickDependency; 3],
    tiling_count: usize,
    fragment: [KickDependency; 3 + crate::g17::hazard::PUBLICATION_BARRIERS],
    fragment_count: usize,
}

fn render_barriers(
    fragment_stage: bool,
    dependencies: &RenderDependencies,
    own_tiling: KickDependency,
) -> RenderBarriers {
    let mut tiling_barriers = [KickDependency::ZERO; 3];
    let mut tiling_count = 0;
    let mut fragment_barriers = [own_tiling; 3 + crate::g17::hazard::PUBLICATION_BARRIERS];
    let mut fragment_count = 1;
    for dependency in dependencies.render() {
        if fragment_stage {
            fragment_barriers[fragment_count] = *dependency;
            fragment_count += 1;
        } else {
            tiling_barriers[tiling_count] = *dependency;
            tiling_count += 1;
        }
    }
    if fragment_stage {
        // The implicit parent covers only this physical ring. Preserve the logical
        // TA prefix when a predecessor was published on another pair, without
        // making this TA kick wait for that predecessor's fragment completion.
        for dependency in &dependencies.tiling[..dependencies.tiling_count] {
            if dependency.qid() != own_tiling.qid() {
                tiling_barriers[tiling_count] = *dependency;
                tiling_count += 1;
            }
        }
    }
    if let Some(compute) = dependencies.compute {
        tiling_barriers[tiling_count] = compute;
        tiling_count += 1;
    }
    // Only tile stores overlap: the tiling kick stays concurrent.
    for hazard in &dependencies.hazards[..dependencies.hazard_count] {
        fragment_barriers[fragment_count] = *hazard;
        fragment_count += 1;
    }
    RenderBarriers {
        tiling: tiling_barriers,
        tiling_count,
        fragment: fragment_barriers,
        fragment_count,
    }
}

impl Pair {
    /// All descriptor bytes and retained allocations are ready before entry.
    /// The first inner producer establishes firmware ownership, including on
    /// later errors. Such failures retain the active packet for recovery.
    pub(crate) fn publish(
        &mut self,
        packet: Arc<Packet>,
        dependencies: &RenderDependencies,
        lease: PublicationLease,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        use crate::g17::fw::{
            channels::WorkSlot,
            kick::{KickArgs, KickQos},
            queue::{
                self, FreeListBinding, QueueConfig, QueueConfigArgs, QueueConfigTarget,
                TilingBinding,
            },
        };
        use core::sync::atomic::{fence, Ordering};
        let (epoch, mcache, policy) = self.publication_preflight(&packet, host)?;
        self.graph_started = true;
        let ordinal = self.ordinal;
        let next_ordinal = ordinal.checked_add(1).ok_or(EOVERFLOW)?;
        let qids = self.qids();
        let kick_ids = self.kicks.each_ref().map(kick::Queue::id);
        // Command preparation assigns context channel state before publishing
        // producers. Pin identities before that assignment; warming never
        // enters this path and can still give its unused identities back.
        for id in kick_ids {
            host.publish_qid(id)?;
        }
        let kicks = self.kicks.each_ref().map(kick::Queue::timestamp);
        let parents = self.kicks.each_ref().map(kick::Queue::parent);
        let work = packet.completion.work_state()?;
        let work_slot = work.render_slot(self.slot)?;
        self.memory.bind_work_slot(ordinal, qids, work_slot)?;
        let payload = work.channel_payload(self.slot, ordinal)?;
        let stamps = host.stamps(qids)?;
        fence(Ordering::Acquire);
        let covered = stamps.map(|stamp| retirement::stamp_covers(stamp, ordinal));
        self.memory.completion_links(
            ordinal,
            kicks,
            payload,
            self.variant.fragment_busy && !covered[1],
        )?;
        let leads = match self.variant.lead_commands {
            true => {
                let init_manager = !self.memory.manager.initialized();
                self.memory.write_leads(ordinal, init_manager)?
            }
            false => [false; 2],
        };
        let context = &packet.context;
        let ring = 1u8 << policy.priority();
        let qos = context.qos_id();
        let qos_owners = [
            qos::Owner {
                qid: qids[0],
                qos,
                data_master: DataMaster::Tiling as u8,
            },
            qos::Owner {
                qid: qids[1],
                qos,
                data_master: DataMaster::Fragment as u8,
            },
        ];
        let mut configs = [None; 2];
        for stage in 0..2 {
            configs[stage] = Some(QueueConfig::new(&QueueConfigArgs {
                target: if stage == 0 {
                    QueueConfigTarget::Tiling(TilingBinding {
                        pb_va: self.memory.manager.state_va(),
                        pb_token: lease.buffer_token,
                        pb_slot: self.buffer_id().into(),
                        fragment_qid: qids[1],
                    })
                } else {
                    QueueConfigTarget::Fragment
                },
                kick_ring_va: self.kicks[stage].low_va(),
                kick_ring_fw_va: self.kicks[stage].firmware_va(),
                qid: qids[stage],
                install: !self.installed[stage],
                completion_seed: covered[stage].then(|| {
                    kicks[stage]
                        .prev()
                        .completion_value(payload.wrapping_sub(1))
                }),
                context_id: context.id().try_into()?,
                free_list: FreeListBinding {
                    va: self.free_list.control_va(),
                    generation: lease.free_list_generation,
                    slot: self.free_list.id().into(),
                },
                scheduler_va: context.scheduler_va(),
                policy,
                qos_slot: qos,
                qos_update: self.context_update,
                owner_pid: context.owner_pid(),
                context_update: self.context_update,
            })?);
        }
        let configs = [configs[0].ok_or(EIO)?, configs[1].ok_or(EIO)?];
        let cache = if self.recovery_generation == epoch {
            self.configs
        } else {
            [None; 2]
        };
        // A covered queue holds the completion value of its newest kick. When that kick is the
        // previous publication of this pair in this recovery epoch, the seed would reload the
        // value already held, so the configuration record is needed only for its identity.
        // Each record costs the firmware time before it starts the kick.
        let held = self
            .published_completions
            .take()
            .filter(|_| self.recovery_generation == epoch);
        let present = [0, 1].map(|stage| {
            let mut config = configs[stage];
            let reload = kicks[stage].prev().completion_value(payload.wrapping_sub(1));
            if config.completion_seed_valid != 0
                && self.installed[stage]
                && held.is_some_and(|held| held[stage] == reload)
            {
                config.completion_seed_valid = 0;
                config.completion_seed = 0;
            }
            publication::required(cache[stage], config)
        });
        let cursors = self.memory.graph.cursors()?;
        if cursors[0].write != self.cursors[0] || cursors[1].write != self.cursors[1] {
            return Err(EBUSY);
        }
        let queues = qids.map(|qid| {
            self.memory.graph.queues.gpu_va()
                + u64::from(qid) * size_of::<queue::QueueRecord>() as u64
        });
        let Validated::Render { pass, ray_query, .. } = &packet.command else {
            return Err(EINVAL);
        };
        let own_tiling = KickDependency::new(qids[0], kicks[0]).ok_or(EINVAL)?;
        let barriers = render_barriers(pass.fragment_barriers, dependencies, own_tiling);
        let tiling_barriers = &barriers.tiling[..barriers.tiling_count];
        let args = |stage: usize, barriers, arrays, mcache| KickArgs {
            qid: qids[stage],
            timestamp: kicks[stage],
            parent: parents[stage],
            barriers,
            descriptor_va: self.memory.graph.descriptor_va(stage, ordinal),
            queue_va: queues[stage],
            qos: KickQos {
                slot: qos,
                class: if stage == 0 {
                    abi::TA_QOS_CLASS
                } else {
                    abi::FRAGMENT_QOS_CLASS
                },
            },
            mcache,
            event_mask: abi::RENDER_KICK_EVENT_MASK,
            register_arrays: arrays,
            scs_enabled: ray_query[stage].scratch.enabled(),
            priority: policy.priority(),
        };
        let ta_args = args(
            0,
            tiling_barriers,
            abi::TaDescriptor::register_bindings(
                self.memory.graph.descriptor_client(0, ordinal),
                ray_query[0].scratch.enabled(),
                self.variant.programs,
            )?,
            None,
        );
        let fragment_args = args(
            1,
            &barriers.fragment[..barriers.fragment_count],
            abi::FragmentDescriptor::register_bindings(
                self.memory.graph.descriptor_client(1, ordinal),
                ray_query[1].scratch.enabled(),
                self.variant.programs,
            )?,
            mcache,
        );
        let registration = RenderRegistration::new(
            (qids[1], self.kicks[1].low_va()),
            (qids[0], self.kicks[0].low_va()),
            policy.priority(),
        )?;
        let [tiling_kick, fragment_kick] = &mut self.kicks;
        let tiling_kick = tiling_kick.prepare_entry(&ta_args)?;
        let fragment_kick = fragment_kick.prepare_entry(&fragment_args)?;
        let memory = &mut *self.memory;
        let lanes = [
            publication::Lane::new(
                &memory.graph,
                0,
                ordinal,
                self.cursors[0],
                configs[0],
                present[0],
                kicks[0],
                leads[0],
            )?,
            publication::Lane::new(
                &memory.graph,
                1,
                ordinal,
                self.cursors[1],
                configs[1],
                present[1],
                kicks[1],
                leads[1],
            )?,
        ];
        let windows = [lanes[0].window, lanes[1].window];
        let previous = self.active.front().map_or(cursors, |active| {
            active.ticket.windows.map(|window| {
                let target = window.target();
                retirement::Cursors {
                    done: target,
                    read: target,
                    write: target,
                }
            })
        });
        let fragment_slot = WorkSlot::new(
            DataMaster::Fragment,
            queues[1],
            qids[1],
            windows[1].target().try_into()?,
            self.outer_started & ring == 0,
        )?;
        let tiling_slot = WorkSlot::new(
            DataMaster::Tiling,
            queues[0],
            qids[0],
            windows[0].target().try_into()?,
            self.outer_started & ring == 0,
        )?;
        let event_slot = memory
            .graph
            .support
            .word(abi::storage::EVENT_SLOTS + 4)?;
        let next_event = event_slot.load(Ordering::Relaxed).wrapping_add(1);
        let inner = memory.graph.support.word(0x28000)?;
        let next_inner = (next_ordinal as u32).wrapping_mul(2);
        let flist_submission = self.free_list.prepare_submission(2)?;
        let pm_submission = memory.manager.prepare_submission()?;
        let published = host.next_publication()?;
        self.active
            .push(Active {
                published,
                qos_completed: false,
                flist_completed: false,
                outer_published: false,
                generation: lease.free_list_generation,
                packet: packet.clone(),
                ticket: Ticket {
                    ordinal,
                    windows,
                    previous,
                },
                kicks,
                context_update: self.context_update,
                failed: false,
                drained_result: None,
                abandoned_since: None,
                stamp_wait_since: None,
            })
            .map_err(|_| EBUSY)?;
        self.prepared = None;
        packet.mark_published();
        lanes[1].publish_prefix();
        fragment_kick.commit();
        lanes[1].publish_kick();
        tiling_kick.commit();
        let mut qos_publications = [None; 2];
        let result = (|| {
            let [fragment, tiling] = host.qos_publish_pair(
                [qos_owners[1], qos_owners[0]],
                context.scheduler_va(),
                policy,
            )?;
            qos_publications = [Some(tiling), Some(fragment)];
            // A class change happens only at a rebind, after the pair drained.
            if self.registered != Some(policy.priority()) {
                host.install_pair(&registration)?;
                self.registered = Some(policy.priority());
            }
            host.publish_outer_pair(
                policy.priority(),
                &fragment_slot,
                &tiling_slot,
                || {
                    work.add_submitted_kicks(2)?;
                    flist_submission.commit();
                    pm_submission.commit();
                    Ok(())
                },
                || lanes[0].publish_tiling(event_slot, next_event, inner, next_inner),
            )
        })();
        let outer_next = match result {
            Ok(next) => {
                if leads[0] {
                    self.memory.manager.mark_initialized();
                }
                next
            }
            Err(error) => {
                self.quarantined = true;
                if !packet.completion.spared() && self.terminal.is_none() {
                    self.terminal = Some((packet.completion.status().clone(), error));
                }
                self.active
                    .back()
                    .ok_or(EIO)?
                    .packet
                    .completion
                    .quarantine_render();
                for publication in qos_publications.into_iter().flatten() {
                    host.qos_cancel(publication)?;
                }
                self.active.back_mut().ok_or(EIO)?.qos_completed = true;
                self.quarantined = true;
                self.record_publication_failure(&packet, error, defer)?;
                return Err(error);
            }
        };
        self.ordinal = next_ordinal;
        if let Some(fragment) = KickDependency::new(qids[1], kicks[1]) {
            packet.completion.note_render_publication(fragment);
        }
        self.cursors = windows.map(|window| window.target());
        for stage in 0..2 {
            if present[stage] {
                self.configs[stage] = Some(configs[stage]);
            }
        }
        self.recovery_generation = epoch;
        self.installed = [true; 2];
        self.published_completions = Some(kicks.map(|kick| kick.completion_value(payload)));
        self.outer_started |= ring;
        if let Some(previous) = self.previous.as_mut().filter(|p| p.publication.is_none()) {
            previous.publication = Some(ordinal);
        }
        let scheduler = packet.context.scheduler_va();
        if self.executed != 0 && self.executed != scheduler {
            self.switch = Some((self.executed, ordinal));
        }
        self.executed = scheduler;
        packet.context.mark_published();
        self.announce_publication(
            host,
            (&fragment_slot, &tiling_slot, outer_next),
            lease.free_list_generation,
            policy.priority(),
            &packet,
            defer,
        )
    }

    /// Everything that must hold before a publication changes any state: no alias work
    /// pending, a healthy device epoch, room in the active ring, this slot's packet, the
    /// prepared descriptors for this ordinal, a clean completion status and a registration
    /// class the pair can still take. Returns the epoch, the prepared memory-cache setting
    /// and the owner's policy.
    fn publication_preflight(
        &self,
        packet: &Arc<Packet>,
        host: &mut impl Host,
    ) -> Result<(u64, Option<McacheTable>, crate::g17::fw::queue::Policy)> {
        if self.pending_aliases.is_some() || self.kick_aliases.is_some() {
            return Err(EBUSY);
        }
        let (epoch, recovery_state) = host.epoch()?;
        if recovery_state != 0
            || self.quarantined
            || !self.active.room()
            || packet.completion.render_slot() != Some(self.slot)
        {
            return Err(EBUSY);
        }
        let prepared = self.prepared.as_ref().ok_or(EINVAL)?;
        if !Arc::ptr_eq(&prepared.packet, packet)
            || prepared.ordinal != self.ordinal
            || !self.memory.owns(self.ordinal)
            || !packet.context.is_current()
        {
            return Err(EIO);
        }
        if packet.completion.status().get() != 0 {
            return Err(EIO);
        }
        let policy = packet.context.policy();
        // A registration of another class can only follow a rebind, which
        // requires the pair to have drained.
        if self.registered.is_some_and(|class| class != policy.priority()) && self.in_flight() {
            return Err(EIO);
        }
        Ok((epoch, prepared.mcache, policy))
    }

    /// Records a failed publication on the retained active entry: the terminal status
    /// (once), the deferred failure and the entry's failed flag.
    fn record_publication_failure(
        &mut self,
        packet: &Arc<Packet>,
        error: Error,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        if !packet.completion.spared() && self.terminal.is_none() {
            self.terminal = Some((packet.completion.status().clone(), error));
        }
        defer(Deferred::FailedOwned(packet.completion.clone(), error))?;
        self.active.back_mut().ok_or(EIO)?.failed = true;
        Ok(())
    }

    /// Activates the pool lease and announces the retained pair to the firmware; a failure
    /// here quarantines the pair with the packet still retained for recovery.
    fn announce_publication(
        &mut self,
        host: &mut impl Host,
        (fragment_slot, tiling_slot, outer_next): (&WorkSlot, &WorkSlot, [u32; 2]),
        free_list_generation: u64,
        priority: u8,
        packet: &Arc<Packet>,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        let publish = (|| {
            host.activate_render_pool(self.free_list.id(), free_list_generation)?;
            host.publish_retained_pair(priority, fragment_slot, tiling_slot, outer_next)?;
            self.active.back_mut().ok_or(EIO)?.outer_published = true;
            host.notify_render(priority)?;
            crate::g17::trace::publish(crate::g17::trace::RENDER, self.qids()[1], self.ordinal as u32,
                packet);
            host.note_submission()
        })();
        if let Err(error) = publish {
            self.quarantined = true;
            self.active
                .back()
                .ok_or(EIO)?
                .packet
                .completion
                .quarantine_render();
            self.record_publication_failure(packet, error, defer)?;
            return Err(error);
        }
        Ok(())
    }
}

struct Observer<'a, H> {
    memory: &'a memory::Memory,
    qids: [u8; 2],
    host: &'a mut H,
}
impl<H: Host> retirement::Memory for Observer<'_, H> {
    fn snapshot(&mut self, ordinal: u64) -> Result<retirement::Snapshot> {
        observation::snapshot(self.memory, ordinal)
    }

    fn stamps(&mut self) -> Result<[u32; 2]> {
        self.host.stamps(self.qids)
    }
    fn pass(&mut self, ordinal: u64) -> Result<[u64; 2]> {
        self.memory.pass(ordinal)
    }
}

impl Pair {
    fn qos_owners(&self) -> [qos::Owner; 2] {
        let qids = self.qids();
        let qos = self.context().qos_id();
        [
            qos::Owner {
                qid: qids[0],
                qos,
                data_master: DataMaster::Tiling as u8,
            },
            qos::Owner {
                qid: qids[1],
                qos,
                data_master: DataMaster::Fragment as u8,
            },
        ]
    }
    /// Bounded first-fault evidence while the recovery owner has firmware
    /// halted, before it acknowledges state 1. Never changes retirement.
    pub(crate) fn report_recovery(&self, stamps: Result<[u32; 2]>) {
        pr_warn!("G17: recovery render slot={} qids={:?} active={} prepared={} quarantined={} stamps={:?} freelist={:?}\n",
            self.slot, self.qids(), self.active.len(), self.prepared.is_some(),
            self.quarantined, stamps, self.free_list.counters());
        self.memory.manager.report_recovery();
        for active in self.active.iter().take(4) {
            if let Validated::Render { pass, .. } = &active.packet.command {
                pr_warn!("G17: recovery render geometry ordinal={} width={} height={} layers={} layout={:?}\n",
                    active.ticket.ordinal, pass.width, pass.height, pass.layers,
                    target::Layout::new(pass, self.clusters, self.variant.programs));
            }
            pr_warn!("G17: recovery render ordinal={} published={} ctx={} seq={} snapshot={:?}\n",
                active.ticket.ordinal, active.outer_published, active.packet.context.id(),
                active.packet.order.sequence, observation::snapshot(&self.memory, active.ticket.ordinal));
        }
    }

    /// Latches when the front group executed and the firmware consumed its ring entries while
    /// `stamps` still lag it, and reports whether that has held for
    /// [`retirement::STAMP_FALLBACK_NS`] (`now`, host ns). The observation scan then retires it
    /// on its pass record instead of waiting for stamps no later event will move.
    pub(crate) fn note_stamp_wait(&mut self, stamps: [u32; 2], state: u32, now: u64) -> Result<bool> {
        if self.quarantined || self.prepared.is_some() || state != 0 {
            return Ok(false);
        }
        let Some(front) = self.active.front_mut() else {
            return Ok(false);
        };
        if !front.outer_published {
            return Ok(false);
        }
        let published = front.ticket.ordinal.checked_add(1).ok_or(EOVERFLOW)?;
        if stamps.into_iter().all(|stamp| retirement::stamp_covers(stamp, published)) {
            front.stamp_wait_since = None;
            return Ok(false);
        }
        let snapshot = observation::snapshot(&self.memory, front.ticket.ordinal)?;
        if !retirement::observe(&front.ticket, snapshot).is_ok_and(|seen| seen.complete) {
            front.stamp_wait_since = None;
            return Ok(false);
        }
        let since = *front.stamp_wait_since.get_or_insert(now);
        Ok(now.saturating_sub(since) >= retirement::STAMP_FALLBACK_NS)
    }

    /// Whether the front group's stamp wait is latched: the observation scan rechecks the pair
    /// without a completion notification, which may never come for such a group.
    pub(crate) fn stamp_waiting(&self) -> bool {
        self.active.front().is_some_and(|front| front.stamp_wait_since.is_some())
    }

    /// The watchdog of `packet` expired while it is this pair's front: if the firmware executed
    /// it and consumed its ring entries while the stamps lag, it retires on its pass record now
    /// instead of failing. Failing would give up on the stamps as well.
    pub(crate) fn expire_stamp_wait(&mut self, packet: &Arc<Packet>, stamps: [u32; 2], state: u32) -> Result<bool> {
        if !self.active.front().is_some_and(|front| Arc::ptr_eq(&front.packet, packet)) {
            return Ok(false);
        }
        if !self.note_stamp_wait(stamps, state, crate::g17::now_ns())? && !self.stamp_waiting() {
            return Ok(false);
        }
        if let Some(front) = self.active.front_mut() {
            front.stamp_wait_since = Some(0);
        }
        Ok(true)
    }

    fn selected(&self, masks: Option<[u64; 2]>) -> bool {
        masks.is_none_or(|masks| completion_noted(self.qids(), masks))
    }
    /// The oldest published group still running, for cross-VM timeout attribution
    /// (crate::g17::timeout). A quarantined pair's work holds the GPU until the firmware drains
    /// it, so it stays visible to other VMs' watchdogs, as compute does, marked as failed.
    pub(crate) fn timeout_head(&self) -> Option<crate::g17::timeout::Head> {
        self.active
            .iter()
            .filter(|active| active.outer_published)
            .find(|active| active.drained_result.is_none())
            .map(|active| crate::g17::timeout::Head {
                vm: active.packet.context.vm().id(),
                published: active.published,
                finished: false,
                failed: self.quarantined,
            })
    }

    pub(crate) fn contains_packet(&self, packet: &Arc<Packet>) -> bool {
        self.active
            .iter()
            .any(|active| Arc::ptr_eq(&active.packet, packet))
            || self
                .prepared
                .as_ref()
                .is_some_and(|prepared| Arc::ptr_eq(&prepared.packet, packet))
    }
    pub(crate) fn replay_contains(&self, packet: &Arc<Packet>) -> bool {
        self.replay.iter().any(|replay| Arc::ptr_eq(replay, packet))
    }
    pub(crate) fn take_deferred_timeout(&self) -> Option<(Arc<Packet>, Error)> {
        self.active.iter().find_map(|active| {
            active
                .packet
                .completion
                .take_deferred_timeout()
                .map(|error| (active.packet.clone(), error))
        })
    }

    pub(crate) fn oldest_packet(&self) -> Option<Arc<Packet>> {
        self.active.front().map(|active| active.packet.clone())
    }
    /// Sample PM feedback only after this pair's exact retirement. Busy pairs
    /// keep their current capacity and let firmware request growth as needed.
    pub(crate) fn host_growth_target(&mut self, packet: &Packet) -> Result<Option<usize>> {
        let Validated::Render { pass, .. } = &packet.command else {
            return Err(EINVAL);
        };
        let required = target::Layout::new(pass, self.clusters, self.variant.programs)?.tvb_blocks;
        self.memory.manager.note_required(required, crate::g17::now_ns());
        if self.in_flight() {
            return Ok(None);
        }
        if !self.idle_for_context(self.context())? {
            return Err(EBUSY);
        }
        let feedback = if self.ordinal != 0 {
            self.memory.manager.automatic_target()?.unwrap_or(0)
        } else {
            0
        };
        let required = required.max(feedback);
        Ok((required > self.memory.manager.blocks()).then_some(required))
    }

    pub(crate) fn needs_drain(&self) -> bool {
        self.quarantined
    }

    pub(crate) fn oldest_spared(&self) -> Option<bool> {
        self.active
            .front()
            .map(|active| active.packet.completion.spared())
    }
    /// A recovery record names one of this pair's queues and the completion
    /// stamp of the group it interrupted, `(ordinal + 1) << 8` like the pair's
    /// stamps. Without a stamp match the oldest published group is marked.
    pub(crate) fn note_interrupted(&self, stamp: u32) -> recovery::Interrupted {
        let sequence = stamp >> 8;
        if let Some(active) = self
            .active
            .iter()
            .filter(|active| active.outer_published)
            .find(|active| (active.ticket.ordinal.wrapping_add(1) as u32) & 0xff_ffff == sequence)
        {
            active.packet.completion.note_interrupted();
            return recovery::Interrupted::Exact;
        }
        match self.active.iter().find(|active| active.outer_published) {
            Some(active) => {
                active.packet.completion.note_interrupted();
                recovery::Interrupted::Oldest
            }
            None => recovery::Interrupted::None,
        }
    }
    pub(crate) fn visit_owners(&self, visit: &mut dyn FnMut(&Arc<VmStatus>)) {
        for active in self.active.iter() {
            visit(active.packet.completion.status());
        }
    }
    pub(crate) fn classify(&self, classify: &dyn Fn(&Arc<VmStatus>) -> recovery::Class) {
        for active in self.active.iter() {
            active
                .packet
                .completion
                .classify(classify(active.packet.completion.status()));
        }
    }
    /// A slot stamp must name retained work before its execution markers are meaningful.
    pub(crate) fn pass_started(&self, stamp: u64) -> Option<bool> {
        const MASK: u64 = (1 << 40) - 1;
        let ordinal = self.active.iter().map(|active| active.ticket.ordinal)
            .find(|ordinal| ordinal.wrapping_add(1) & MASK == stamp & MASK)?;
        Some(observation::snapshot(&self.memory, ordinal).ok()?.pass[0] != 0)
    }

    /// Firmware is halted and all graph borrowers have joined before this copy.
    pub(crate) fn publish_recovery_targets(&self) -> Result {
        self.memory.graph.publish_recovery_targets()
    }

    pub(crate) fn resync(&mut self) {
        self.configs = [None; 2];
        self.published_completions = None;
        for kick in &mut self.kicks {
            kick.clear_parent_after_recovery();
        }
    }
    pub(crate) fn matches(&self, owner: u64, context: &Arc<Context>) -> Result<bool> {
        Ok(self.owner == Some(owner)
            && Arc::ptr_eq(self.context(), context)
            && context.is_current())
    }
    /// Classification controls VM poisoning. Every failed render fence settles,
    /// while command memory remains owned until paired retirement or stop.
    pub(crate) fn quarantine(
        &mut self,
        error: Error,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        self.quarantined = true;
        // Successors published after recovery can still fail behind its spared
        // head. Classify the whole failing group before any fence can settle.
        if self.active.iter().any(|active| active.packet.completion.spared()) {
            for active in self.active.iter() {
                active.packet.completion.classify(recovery::Class::Spared);
            }
        }
        for active in self.active.iter_mut() {
            active.packet.completion.quarantine_render();
            if !active.failed {
                defer(Deferred::FailedOwned(
                    active.packet.completion.clone(),
                    error,
                ))?;
                active.failed = true;
                if !active.packet.completion.spared() && self.terminal.is_none() {
                    self.terminal = Some((active.packet.completion.status().clone(), error));
                }
            }
        }
        Ok(())
    }
    pub(crate) fn retire(
        &mut self,
        masks: Option<[u64; 2]>,
        _epoch: u64,
        state: u32,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        self.retire_front(masks, state, None, host, defer)
    }

    pub(crate) fn retire_observed(
        &mut self,
        observed: Observed,
        state: u32,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        self.retire_front(None, state, Some(observed), host, defer)
    }

    fn retire_front(
        &mut self,
        masks: Option<[u64; 2]>,
        state: u32,
        observed: Option<Observed>,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        if self.quarantined || state != 0 || !self.selected(masks) {
            return Ok(false);
        }
        let qids = self.qids();
        let qos = self.qos_owners();
        let pool = self.free_list.id();
        let buffer = self.buffer_id();
        let Some(front) = self.active.front_mut() else {
            return Ok(false);
        };
        if !front.outer_published {
            return Ok(false);
        }
        if !Arc::ptr_eq(&front.packet.context, &self.memory.context)
            || !self.memory.owns(front.ticket.ordinal)
        {
            return Err(EIO);
        }
        let stampless = front.stamp_wait_since.is_some_and(|since| {
            crate::g17::now_ns().saturating_sub(since) >= retirement::STAMP_FALLBACK_NS
        });
        let mut observer = Observer {
            memory: &self.memory,
            qids,
            host,
        };
        let result = if let Some(observed) = observed {
            if observed.ticket != front.ticket {
                return Err(EIO);
            }
            if stampless {
                retirement::poll_stampless(&mut front.ticket, &mut observer)
            } else {
                retirement::poll_observed(&mut front.ticket, &mut observer, observed.snapshot)
            }
        } else if stampless {
            retirement::poll_stampless(&mut front.ticket, &mut observer)
        } else {
            retirement::poll(&mut front.ticket, &mut observer)
        };
        let timestamps = match result {
            Ok(retirement::Poll::Complete(timestamps)) => timestamps,
            Ok(retirement::Poll::Pending) => return self.replay_abandoned(host, defer, true),
            Ok(retirement::Poll::Retry) => return Ok(false),
            Err(error) => {
                self.quarantine(error, defer)?;
                return Ok(false);
            }
        };
        let ordinal = front.ticket.ordinal;
        let context_update = front.context_update;
        if stampless {
            retirement::note_stampless(u16::from(self.slot), ordinal);
        }
        self.note_retirement(ordinal, context_update);
        let front = self.active.front_mut().ok_or(EIO)?;
        if !front.flist_completed {
            host.complete_render(
                (!front.qos_completed).then_some(qos),
                pool,
                front.generation,
                buffer,
            )?;
            front.qos_completed = true;
            front.flist_completed = true;
        }
        crate::g17::recovery::note_retired();
        crate::g17::trace::complete(crate::g17::trace::RENDER, qids[1], ordinal as u32,
            &front.packet, [timestamps[0], timestamps[3]]);
        defer(Deferred::Retired(
            front.packet.completion.clone(),
            Ok(timestamps),
        ))?;
        self.memory.retire(ordinal)?;
        self.active.pop().ok_or(EIO)?;
        Ok(true)
    }

    /// Every retained group was retired and unlinked by firmware without
    /// executing (job list empty, stamps cover it, no recovery, fragment end
    /// never written): after another VM's recovery the firmware drops an
    /// innocent pair's queued groups this way (seen on 25G83 and 26A428).
    /// Nothing of them reached their attachments, so the same commands are
    /// published again, in order, instead of failing their owners. Only the
    /// whole set is replayed; a replayed prefix would run after a live
    /// successor. `grace` asks for the strict witness twice ABANDON_GRACE_NS
    /// apart (worker polling); the render timeout, long after publication,
    /// takes it once as its retirement does. A single sample can still
    /// straddle a firmware update of the fragment end record; the cost is
    /// that a group which did execute runs a second time (blending twice).
    fn replay_abandoned(
        &mut self,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
        grace: bool,
    ) -> Result<bool> {
        use retirement::Memory;
        if self.quarantined || self.active.len() == 0 || self.replay.len() != 0 {
            return Ok(false);
        }
        if let Some(progressed) = self.settle_interrupted_front(host, defer, grace)? {
            return Ok(progressed);
        }
        use crate::g17::replay::{self, Decision, Group};
        let qids = self.qids();
        let memory = &self.memory;
        let mut failure = None;
        let groups = self.active.iter().map(|active| {
            let innocent =
                active.outer_published && !active.failed && !active.packet.completion.guilty();
            let blamed = active.packet.completion.blamed();
            let exhausted =
                active.packet.completion.replay_count() >= crate::g17::completion::MAX_REPLAYS;
            // decide() stops at the first group that is still live, an error included.
            let (abandoned, ran, lost) = if innocent && !blamed {
                (|| -> Result<(bool, bool, bool)> {
                    let mut observer = Observer {
                        memory,
                        qids,
                        host: &mut *host,
                    };
                    core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
                    let snapshot = observer.snapshot(active.ticket.ordinal)?;
                    let stamps = observer.stamps()?;
                    let (_, state) = observer.host.epoch()?;
                    // A torn sample is no proof; a later poll or the timeout looks again.
                    let witness = |executed| {
                        matches!(
                            retirement::abandoned(&active.ticket, snapshot, stamps, state, executed),
                            Ok(true)
                        )
                    };
                    let unexecuted = witness(false);
                    // Interrupted work is run again only if it never started, and work that
                    // another queue waited on never: the wait ended when firmware retired it.
                    let lost = (active.packet.completion.interrupted() && !snapshot.untouched())
                        || active.packet.completion.has_firmware_consumer();
                    Ok((unexecuted && !lost, !unexecuted && witness(true), unexecuted && lost))
                })()
                .unwrap_or_else(|error: Error| {
                    failure = Some(error);
                    (false, false, false)
                })
            } else {
                (false, false, false)
            };
            Group {
                innocent,
                blamed,
                abandoned,
                exhausted,
                ran,
                lost,
            }
        });
        let decision = replay::decide(groups);
        if let Some(error) = failure {
            return Err(error);
        }
        let now = crate::g17::now_ns();
        let front = self.active.front_mut().ok_or(EIO)?;
        match decision {
            Decision::Refuse => return Ok(false),
            Decision::Wait => {
                front.abandoned_since = None;
                return Ok(false);
            }
            Decision::Replay | Decision::Fail => {}
        }
        if grace {
            let Some(since) = front.abandoned_since else {
                front.abandoned_since = Some(now);
                return Ok(false);
            };
            if now.saturating_sub(since) < ABANDON_GRACE_NS {
                return Ok(false);
            }
        }
        if decision == Decision::Fail {
            return self.fail_dropped(host, defer);
        }
        // decide() lets only the front be exhausted. An exhausted front fails, and its
        // successors, which consume its output, fail with it rather than run without it.
        let mut failing = self.active.front().is_some_and(|front| {
            front.packet.completion.replay_count() >= crate::g17::completion::MAX_REPLAYS
        });
        if failing {
            pr_warn!(
                "G17: render slot {}: {} groups retired without execution; replay limit reached, failing them\n",
                self.slot,
                self.active.len()
            );
        } else if !grace || crate::debug::debug_enabled(crate::debug::DebugFlags::VerboseFaults) {
            pr_warn!(
                "G17: render slot {}: {} groups retired without execution; replaying\n",
                self.slot,
                self.active.len()
            );
        }
        let qos = self.qos_owners();
        let buffer = self.buffer_id();
        let pool = self.free_list.id();
        // Every group parked below shares this expiry (an early return keeps it).
        self.replay_since = Some(now);
        while let Some(front) = self.active.front_mut() {
            if !front.flist_completed {
                host.complete_render(
                    (!front.qos_completed).then_some(qos),
                    pool,
                    front.generation,
                    buffer,
                )?;
                front.flist_completed = true;
                front.qos_completed = true;
            }
            let ordinal = front.ticket.ordinal;
            let packet = front.packet.clone();
            failing = failing || !packet.completion.begin_replay();
            if failing {
                if !packet.completion.spared() {
                    packet.completion.status().record(EIO);
                    if self.terminal.is_none() {
                        self.terminal = Some((packet.completion.status().clone(), EIO));
                    }
                }
                defer(Deferred::Retired(packet.completion.clone(), Err(EIO)))?;
                self.memory.retire(ordinal)?;
                self.active.pop().ok_or(EIO)?;
                continue;
            }
            self.memory.retire(ordinal)?;
            self.active.pop().ok_or(EIO)?;
            packet.reset_publication_for_replay();
            self.published_completions = None;
            self.replay.push(packet).map_err(|_| EIO)?;
        }
        Ok(true)
    }

    /// Firmware retired every group of the pair after a restart, but some were dropped
    /// unexecuted and cannot run again in order (see `replay::decide`). Every group fails now,
    /// the ones that ran too: they consumed output the dropped ones never wrote. The work is
    /// innocent, so it fails without poisoning its VM, with ENODATA, which its owner sees,
    /// instead of waiting for the render timeout to fail it with ETIMEDOUT.
    fn fail_dropped(
        &mut self,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        pr_warn!(
            "G17: render slot {}: a GPU recovery dropped groups that cannot run again in order; failing {} groups\n",
            self.slot,
            self.active.len()
        );
        let qos = self.qos_owners();
        let buffer = self.buffer_id();
        let pool = self.free_list.id();
        while let Some(front) = self.active.front_mut() {
            if !front.flist_completed {
                host.complete_render(
                    (!front.qos_completed).then_some(qos),
                    pool,
                    front.generation,
                    buffer,
                )?;
                front.flist_completed = true;
                front.qos_completed = true;
            }
            let ordinal = front.ticket.ordinal;
            let completion = front.packet.completion.clone();
            completion.classify(recovery::Class::Spared);
            defer(Deferred::Retired(completion, Err(ENODATA)))?;
            self.memory.retire(ordinal)?;
            self.active.pop().ok_or(EIO)?;
        }
        Ok(true)
    }

    /// A front group a restart interrupted after its tiler started, which firmware
    /// then retired without writing its fragment end, may have run partly. It is
    /// not replayed (blending or storage writes could apply twice) and fails now,
    /// reported to its owner, instead of waiting out the render timeout. Returns
    /// None when the front is not such a group.
    fn settle_interrupted_front(
        &mut self,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
        grace: bool,
    ) -> Result<Option<bool>> {
        use retirement::Memory;
        let qids = self.qids();
        let Some(front) = self.active.front() else {
            return Ok(None);
        };
        if !front.outer_published || front.failed || !front.packet.completion.interrupted() {
            return Ok(None);
        }
        let ordinal = front.ticket.ordinal;
        let ticket = front.ticket;
        let mut observer = Observer {
            memory: &self.memory,
            qids,
            host: &mut *host,
        };
        core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
        let snapshot = observer.snapshot(ordinal)?;
        let stamps = observer.stamps()?;
        let (_, state) = observer.host.epoch()?;
        if snapshot.untouched()
            || !matches!(
                retirement::abandoned(&ticket, snapshot, stamps, state, false),
                Ok(true)
            )
        {
            return Ok(None);
        }
        let now = crate::g17::now_ns();
        let front = self.active.front_mut().ok_or(EIO)?;
        if grace {
            let Some(since) = front.abandoned_since else {
                front.abandoned_since = Some(now);
                return Ok(Some(false));
            };
            if now.saturating_sub(since) < ABANDON_GRACE_NS {
                return Ok(Some(false));
            }
        }
        if !front.flist_completed {
            let qos = self.qos_owners();
            let buffer = self.buffer_id();
            let pool = self.free_list.id();
            let front = self.active.front_mut().ok_or(EIO)?;
            host.complete_render(
                (!front.qos_completed).then_some(qos),
                pool,
                front.generation,
                buffer,
            )?;
            front.flist_completed = true;
            front.qos_completed = true;
        }
        pr_warn!(
            "G17: render slot {}: group {} was interrupted by a GPU recovery after it started; failing it\n",
            self.slot,
            ordinal
        );
        let packet = self.active.front().ok_or(EIO)?.packet.clone();
        defer(Deferred::Retired(packet.completion.clone(), Err(ECANCELED)))?;
        self.memory.retire(ordinal)?;
        self.active.pop().ok_or(EIO)?;
        Ok(Some(true))
    }

    /// The pending replay cannot proceed (closed owner, cancellation or a
    /// publication refusal). Its successors cannot run without it and fail with
    /// it. ECANCELED leaves the VM usable; any other error is the ordinary
    /// device-loss outcome the replay tried to avoid.
    pub(crate) fn fail_replay(
        &mut self,
        error: Error,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        self.replay_since = None;
        while let Some(packet) = self.replay.pop() {
            defer(Deferred::Retired(packet.completion.clone(), Err(error)))?;
        }
        Ok(())
    }

    /// A closed preparation epoch invalidates only this unpublished descriptor.
    /// Its completion remains available for a retry under the next epoch.
    pub(crate) fn discard_prepared(&mut self, packet: &Arc<Packet>) -> Result {
        let Some(prepared) = self.prepared.as_ref() else {
            return Ok(());
        };
        if !Arc::ptr_eq(&prepared.packet, packet) {
            return Err(EBUSY);
        }
        self.memory.retire(prepared.ordinal)?;
        self.prepared = None;
        Ok(())
    }
}

impl Pair {
    /// Call before and again after servicing firmware events. FList completion
    /// counters are full-width, including beyond the low-word wrap boundary.
    pub(crate) fn firmware_drained(&self, host: &impl Host) -> Result<bool> {
        if self.prepared.is_some() || self.active.iter().any(|a| !a.outer_published) {
            return Ok(false);
        }
        let counters = self.free_list.counters()?;
        let stamps = host.stamps(self.qids())?;
        let (_, state) = host.epoch()?;
        Ok(retirement::drained(self.ordinal, stamps, counters, state)
            || (state == 0 && self.executed_unstamped() && self.context_killed()))
    }

    /// The firmware acknowledged a kill of this pair's context (scheduler page): it no longer
    /// walks that context's work list, so the commands it never unlinked can be released.
    fn context_killed(&self) -> bool {
        crate::g17::queue::compute::exit::State::read(&self.memory.context).is_ok_and(|state| state.killed != 0)
    }

    /// The pair's context may be killed although its commands were not retired: they all ran
    /// (`executed_unstamped`), so the kill leaves nothing queued for the firmware to run.
    /// Its memory stays until the kill is acknowledged.
    pub(crate) fn executed_for_kill(&self, context: &Arc<Context>) -> bool {
        Arc::ptr_eq(context, &self.memory.context) && self.executed_unstamped()
    }

    /// Every command of this quarantined pair ran, its ring entries were consumed and its
    /// free-list blocks returned, yet the firmware never stores the stamp of the last one.
    /// On Neo this is the render work of a context whose compute command failed: it runs
    /// about a second after the failure, but its stamp never arrives and it stays linked in
    /// the firmware's job list, so neither the stamp proof nor the abandoned-group proof
    /// ever holds and the pair would stay quarantined, keeping its context, until the
    /// processors stop.
    fn executed_unstamped(&self) -> bool {
        self.quarantined
            // Only for a closed owner: a live owner would go on publishing into a context
            // whose work list still links the unstamped node.
            && self.owner.is_none()
            && self.prepared.is_none()
            && self.active.len() != 0
            && self.free_list.counters().is_ok_and(|counters| counters[0] == counters[1])
            && self.active.iter().all(|active| {
                active.outer_published
                    && observation::snapshot(&self.memory, active.ticket.ordinal)
                        .is_ok_and(retirement::Snapshot::executed_and_consumed)
            })
    }

    /// Account one witnessed tracker, preserving all completion ownership.
    /// The caller services its pending FList release before accounting another
    /// tracker and invokes release_witnessed only after that control stage.
    pub(crate) fn account_drained(&mut self, host: &mut impl Host) -> Result<bool> {
        use retirement::Memory;
        if !self.firmware_drained(host)? {
            return Ok(false);
        }
        let qids = self.qids();
        let qos = self.qos_owners();
        let buffer = self.buffer_id();
        // Commands the firmware never unlinked are released only after it acknowledged the
        // kill of their context, on every path (see `firmware_drained`).
        let unstamped = self.executed_unstamped();
        let killed = unstamped && self.context_killed();
        let Some(active) = self.active.iter_mut().find(|a| !a.flist_completed) else {
            return Ok(false);
        };
        let mut observer = Observer {
            memory: &self.memory,
            qids,
            host,
        };
        let result = match retirement::poll(&mut active.ticket, &mut observer)? {
            retirement::Poll::Complete(ticks) => Ok(ticks),
            // Its commands ran without a stamp (see `executed_unstamped`): they already failed
            // with the quarantine, and the client sees them cancelled.
            retirement::Poll::Pending | retirement::Poll::Retry if killed => {
                crate::g17::trace::teardown(2, u64::from(self.slot), active.ticket.ordinal, 0);
                Err(ECANCELED)
            }
            retirement::Poll::Pending | retirement::Poll::Retry => {
                core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
                let snapshot = observer.snapshot(active.ticket.ordinal)?;
                let stamps = observer.stamps()?;
                let (_, state) = observer.host.epoch()?;
                if !retirement::abandoned(&active.ticket, snapshot, stamps, state, true)? {
                    return Ok(false);
                }
                Err(EIO)
            }
        };
        let ordinal = active.ticket.ordinal;
        let context_update = result.is_ok() && active.context_update;
        self.note_retirement(ordinal, context_update);
        let active = self
            .active
            .iter_mut()
            .find(|active| active.ticket.ordinal == ordinal)
            .ok_or(EIO)?;
        host.complete_render(
            (!active.qos_completed).then_some(qos),
            self.free_list.id(),
            active.generation,
            buffer,
        )?;
        active.qos_completed = true;
        active.flist_completed = true;
        active.drained_result = Some(result);
        Ok(true)
    }

    /// Fresh proof after asynchronous control publication is the ownership
    /// boundary. A refused deferred item leaves the precise owner available.
    pub(crate) fn release_witnessed(
        &mut self,
        host: &impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result {
        if self.active.iter().any(|a| !a.flist_completed) || !self.firmware_drained(host)? {
            return Ok(());
        }
        while let Some(active) = self.active.front() {
            let result = active.drained_result.ok_or(EIO)?;
            defer(Deferred::Drained(active.packet.completion.clone(), result))?;
            let ordinal = active.ticket.ordinal;
            self.memory.retire(ordinal)?;
            self.active.pop().ok_or(EIO)?;
        }
        self.quarantined = false;
        self.resync();
        Ok(())
    }

    /// The registry first polls the completed prefix through this exact Arc.
    /// With successors, source policy quarantines every remaining group. A
    /// single unexecuted group may use the strict ownership witness instead.
    pub(crate) fn fail_after_observation(
        &mut self,
        packet: &Arc<Packet>,
        error: Error,
        host: &mut impl Host,
        defer: &mut impl FnMut(Deferred) -> Result,
    ) -> Result<bool> {
        use retirement::Memory;
        if !self.active.iter().any(|a| Arc::ptr_eq(&a.packet, packet)) {
            return Ok(false);
        }
        // Groups the firmware dropped unexecuted are published again, with their successors.
        if self.replay_abandoned(host, defer, false)? {
            return Ok(true);
        }
        // Fallback when a live or exhausted group prevents an ordered replay: a spared front group
        // the firmware retired without executing during another VM's recovery is retired on its
        // own even with successors queued, so the pair is not quarantined into a sticky EIO. It
        // fails with ECANCELED, which spared work never reports as device loss; consecutive
        // abandoned spared successors are retired the same way.
        let front_spared = self.active.front().is_some_and(|a| {
            Arc::ptr_eq(&a.packet, packet) && a.packet.completion.spared()
        });
        if !self.quarantined && (self.active.len() == 1 || front_spared) {
            let mut retired = false;
            loop {
                let Some(active) = self.active.front() else {
                    break;
                };
                let spared = active.packet.completion.spared();
                if retired && !spared {
                    break;
                }
                let mut observer = Observer {
                    memory: &self.memory,
                    qids: self.qids(),
                    host,
                };
                core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
                let snapshot = observer.snapshot(active.ticket.ordinal)?;
                let stamps = observer.stamps()?;
                let (_, state) = observer.host.epoch()?;
                // A torn sample is no proof, as in replay_abandoned(): the group stays retained
                // (quarantined with the pair if nothing was retired) instead of the error
                // stopping the runtime for every client.
                if !active.outer_published
                    || !matches!(
                        retirement::abandoned(&active.ticket, snapshot, stamps, state, false),
                        Ok(true)
                    )
                {
                    break;
                }
                let qos = self.qos_owners();
                let buffer = self.buffer_id();
                let left = self.active.len() - 1;
                let active = self.active.front_mut().ok_or(EIO)?;
                if !active.flist_completed {
                    host.complete_render(
                        (!active.qos_completed).then_some(qos),
                        self.free_list.id(),
                        active.generation,
                        buffer,
                    )?;
                    active.flist_completed = true;
                    active.qos_completed = true;
                }
                if !active.failed && !spared {
                    active.packet.completion.status().record(EIO);
                    if self.terminal.is_none() {
                        self.terminal = Some((active.packet.completion.status().clone(), error));
                    }
                }
                pr_warn!(
                    "G17: render retired unexecuted after a recovery (ordinal {}, {} groups left)\n",
                    active.ticket.ordinal,
                    left
                );
                defer(Deferred::Retired(
                    active.packet.completion.clone(),
                    Err(if spared { ECANCELED } else { EIO }),
                ))?;
                let ordinal = active.ticket.ordinal;
                self.memory.retire(ordinal)?;
                self.active.pop().ok_or(EIO)?;
                retired = true;
                if !spared {
                    break;
                }
            }
            if retired {
                return Ok(true);
            }
        }
        self.quarantine(error, defer)?;
        Ok(true)
    }

    /// Both processors have stopped before the caller reaches this method.
    /// VM ownership settles before quarantined admission is rehabilitated.
    pub(crate) fn stopped(&mut self, defer: &mut impl FnMut(Deferred) -> Result) -> Result {
        if let Some(prepared) = self.prepared.as_ref() {
            defer(Deferred::Unpublished(
                prepared.packet.completion.clone(),
                EIO,
            ))?;
            self.memory.retire(prepared.ordinal)?;
            self.prepared = None;
        }
        while let Some(active) = self.active.front() {
            defer(Deferred::Drained(
                active.packet.completion.clone(),
                Err(EIO),
            ))?;
            let ordinal = active.ticket.ordinal;
            self.memory.retire(ordinal)?;
            self.active.pop().ok_or(EIO)?;
        }
        self.replay_since = None;
        while let Some(packet) = self.replay.pop() {
            defer(Deferred::Drained(packet.completion.clone(), Err(EIO)))?;
        }
        if let Some(previous) = self.previous.as_mut() {
            previous.ready = true;
        }
        Ok(())
    }
}

impl Pair {
    /// Commit a cold graph only after the registry revalidates its preparation
    /// lease. The original PB row is restored on every later failure.
    pub(crate) fn finish_install(
        &mut self,
        alloc: &Allocator<'_>,
        descriptors: &crate::g17::object::KernelObject,
    ) -> Result {
        install_memory(
            alloc,
            descriptors,
            self.slot,
            &mut self.memory,
            &mut self.pending_aliases,
            &mut self.kick_aliases,
        )
    }
}

fn install_memory(
    alloc: &Allocator<'_>,
    descriptors: &crate::g17::object::KernelObject,
    slot: u8,
    memory: &mut memory::Memory,
    pending_aliases: &mut Option<KVec<mmu::KernelMapping>>,
    kick_aliases: &mut Option<[Option<mmu::KernelMapping>; 2]>,
) -> Result {
    use core::sync::atomic::{fence, Ordering};
    if pending_aliases.is_none() && kick_aliases.is_none() {
        return Ok(());
    }
    if !memory.context.is_current() || kick_aliases.is_none() {
        return Err(EIO);
    }
    let mut driver = KVec::new();
    driver.reserve(2, GFP_KERNEL)?;
    let aliases = memory.take_persistent(pending_aliases.as_mut().ok_or(EIO)?)?;
    let context = memory.context.clone();
    let vm = context.vm();
    let pool_id = u64::from(memory.lifecycle[0].predecessor);
    let mut staged: Option<[u32; 4]> = None;
    let offset = usize::from(memory.manager.buffer_id()) * 16;
    let result: Result = (|| {
        vm.install_render_pool_mappings(slot, pool_id, memory.manager.blocks(), aliases)?;
        memory.installed_pool = Some(slot);
        alloc.uat.flush_vm_contexts(vm)?;
        let words = [
            descriptors.word(offset)?,
            descriptors.word(offset + 4)?,
            descriptors.word(offset + 8)?,
            descriptors.word(offset + 12)?,
        ];
        let previous = words.map(|word| word.load(Ordering::Relaxed));
        let pages: u32 = (memory.manager.blocks()
            * crate::hw::t8140::resources::TVB_PAGES_PER_BLOCK)
            .try_into()?;
        let next = crate::g17::fw::buffer::reload_descriptor(
            previous,
            memory.manager.page_list_va(),
            pages,
        );
        for (word, value) in words.into_iter().zip(next) {
            word.store(value, Ordering::Relaxed);
        }
        fence(Ordering::SeqCst);
        staged = Some(previous);
        for mapping in kick_aliases.take().ok_or(EIO)?.into_iter().flatten() {
            driver.push(mapping, GFP_KERNEL)?;
        }
        vm.append_driver_mappings(driver)?;
        Ok(())
    })();
    if let Err(error) = result {
        if error == ENOENT {
            context.status().record(error);
        }
        if let Some(previous) = staged {
            for (index, value) in previous.into_iter().enumerate() {
                descriptors
                    .word(offset + index * 4)?
                    .store(value, Ordering::Relaxed);
            }
            fence(Ordering::SeqCst);
        }
        vm.clear_render_pool_mappings_if_owner(slot, pool_id);
        memory.installed_pool = None;
        return Err(error);
    }
    *pending_aliases = None;
    Ok(())
}

/// A binding prepared while the registry exclusively lends the idle pair.
/// Its frontier prevents a stale off-lock result from replacing another owner.
pub(crate) struct PreparedBinding {
    previous_context: Arc<Context>,
    previous_owner: Option<u64>,
    ordinal: u64,
    owner: u64,
    context: Arc<Context>,
    pool: Arc<RenderPool>,
    memory: Option<KBox<memory::Memory>>,
    aliases: Option<KVec<mmu::KernelMapping>>,
    kicks: Option<[Option<mmu::KernelMapping>; 2]>,
    initial_packet: u64,
}

/// The registry retains displaced backing until this exact context's release
/// is acknowledged. A warm binding retains the old context without its graph.
pub(crate) struct RetiredBinding {
    memory: Option<KBox<memory::Memory>>,
    context: Arc<Context>,
    pool: Arc<RenderPool>,
    buffer_id: u8,
    publication: Option<u64>,
    ready: bool,
}
impl RetiredBinding {
    pub(crate) fn context(&self) -> &Arc<Context> {
        &self.context
    }
    pub(crate) fn pool(&self) -> &Arc<RenderPool> {
        &self.pool
    }
    pub(crate) fn buffer_id(&self) -> Option<u8> {
        self.memory.as_ref().map(|_| self.buffer_id)
    }
}

impl Pair {
    fn note_retirement(&mut self, ordinal: u64, context_update: bool) {
        if !context_update {
            return;
        }
        self.context_update = false;
        if let Some(previous) = self.previous.as_mut() {
            if previous
                .publication
                .is_some_and(|published| published <= ordinal)
            {
                previous.ready = true;
            }
        }
    }
    /// The scheduler page these queues last ran a command for, when the next
    /// publication for `context` changes the queues' owner.
    /// The steady state in which a command may publish without the device
    /// mutex: the graph is installed and registered at `priority`, it last ran
    /// `context`'s scheduler, and nothing is replayed, settled, aliased or
    /// switched on it.
    pub(crate) fn direct_ready(&self, context: &Context, priority: u8) -> bool {
        self.graph_started
            && self.installed == [true; 2]
            && self.registered == Some(priority)
            && priority < 8
            && self.outer_started & (1u8 << priority) != 0
            && self.executed == context.scheduler_va()
            && self.switch.is_none()
            && self.owner.is_some()
            && !self.quarantined
            && self.terminal.is_none()
            && self.pending_aliases.is_none()
            && self.kick_aliases.is_none()
            && self.prepared.is_none()
            && self.replay.len() == 0
            && self.displaced.is_none()
            && self.active.room()
    }
    /// The render prefix of `packet` on this pair, as the registry collects it
    /// for an owner whose only pair this is.
    pub(crate) fn own_dependencies(&self, owner: u64, packet: &Arc<Packet>) -> Result<RenderDependencies> {
        let mut dependencies = RenderDependencies::EMPTY;
        let mut frontier = crate::g17::dependency::Frontier::new();
        self.collect_dependencies(owner, packet.completion.status(), packet.order.wait_through[0], &mut frontier, false)?;
        for dependency in frontier.as_slice() {
            *dependencies.render.get_mut(dependencies.render_count).ok_or(EOVERFLOW)? = *dependency;
            dependencies.render_count += 1;
        }
        if dependencies.render_count != 0
            && matches!(&packet.command, Validated::Render { pass, .. } if pass.fragment_barriers)
        {
            let mut tiling = crate::g17::dependency::Frontier::new();
            self.collect_tiling_dependencies(
                owner,
                packet.completion.status(),
                packet.order.wait_through[0],
                &mut tiling,
            )?;
            for dependency in tiling.as_slice() {
                *dependencies.tiling.get_mut(dependencies.tiling_count).ok_or(EOVERFLOW)? = *dependency;
                dependencies.tiling_count += 1;
            }
        }
        Ok(dependencies)
    }
    pub(crate) fn owner_switch_from(&self, context: &Context) -> Option<u64> {
        (self.executed != 0 && self.executed != context.scheduler_va()).then_some(self.executed)
    }
    /// The previous scheduler page of an owner change whose first command has
    /// not left the pair yet.
    pub(crate) fn switch_in_flight(&self) -> Option<u64> {
        let (from, first) = self.switch?;
        (self.replay.len() != 0 || self.active.iter().any(|active| active.ticket.ordinal == first))
            .then_some(from)
    }
    pub(crate) fn take_retired_binding(&mut self) -> Option<RetiredBinding> {
        if self.displaced.is_some() {
            return self.displaced.take();
        }
        if self.previous.as_ref().is_some_and(|p| p.ready) {
            self.previous.take()
        } else {
            None
        }
    }
    pub(crate) fn idle_for_context(&self, context: &Arc<Context>) -> Result<bool> {
        if !Arc::ptr_eq(context, self.context()) {
            return Ok(true);
        }
        if self.prepared.is_some() || self.in_flight() || self.quarantined {
            return Ok(false);
        }
        if self.ordinal == 0 {
            return Ok(true);
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
        Ok(self
            .memory
            .graph
            .cursors()?
            .into_iter()
            .all(|c| c.done == c.read && c.read == c.write))
    }
    pub(crate) fn reusable(&self) -> Result<bool> {
        let previous_ready = self
            .previous
            .as_ref()
            .is_none_or(|p| p.memory.is_none() && self.context_update && !p.ready);
        Ok(self.owner.is_none()
            && self.displaced.is_none()
            && previous_ready
            && self.idle_for_context(self.context())?)
    }
    /// Logical close leaves the physical channel identity and backing retained.
    pub(crate) fn release_owner(&mut self) {
        self.owner = None;
    }
    #[inline(never)]
    #[expect(
        clippy::too_many_arguments,
        reason = "Binding keeps the render owners and first packet explicit."
    )]
    pub(crate) fn prepare_bind(
        &mut self,
        alloc: &Allocator<'_>,
        pool: &Arc<Pool>,
        owner: u64,
        context: Arc<Context>,
        free_list: Arc<RenderPool>,
        metrics: Option<buffer::MetricsLease>,
        buffer_id: Option<(u8, u64)>,
        pm_generation: Option<u64>,
        first: &Arc<Packet>,
    ) -> Result<PreparedBinding> {
        if !self.reusable()?
            || owner == 0
            || !Arc::ptr_eq(&context, &first.context)
            || !context.is_current()
        {
            return Err(EBUSY);
        }
        alloc.uat.vm_context_mask(context.vm())?;
        let same_vm = Arc::ptr_eq(self.context().status(), context.status());
        let (memory, aliases, kicks) = if same_vm {
            if metrics.is_some() || buffer_id.is_some() || pm_generation.is_some() {
                return Err(EINVAL);
            }
            (None, None, None)
        } else {
            let metrics = metrics.ok_or(EINVAL)?;
            let buffer_id = buffer_id.ok_or(EINVAL)?;
            let pm_generation = pm_generation.ok_or(EINVAL)?;
            let Validated::Render { pass, .. } = &first.command else {
                return Err(EINVAL);
            };
            let layout = target::Layout::new(pass, self.clusters, self.variant.programs)?;
            let aliases = [
                self.kicks[0].prepare_client_alias(context.vm())?,
                self.kicks[1].prepare_client_alias(context.vm())?,
            ];
            let (memory, persistent) = memory::Memory::new(
                alloc,
                pool,
                context.clone(),
                self.qids(),
                &layout,
                metrics,
                buffer_id,
                pm_generation,
                self.ordinal,
                first.completion.work_state()?.word(),
                self.variant,
            )?;
            (Some(memory), Some(persistent), Some(aliases))
        };
        Ok(PreparedBinding {
            previous_context: self.context().clone(),
            previous_owner: self.owner,
            ordinal: self.ordinal,
            owner,
            context,
            pool: free_list,
            memory,
            aliases,
            kicks,
            initial_packet: first.id,
        })
    }
    pub(crate) fn bind_owner(
        &mut self,
        binding: &mut PreparedBinding,
        alloc: &Allocator<'_>,
        descriptors: &crate::g17::object::KernelObject,
    ) -> Result {
        if !self.reusable()?
            || binding.ordinal != self.ordinal
            || binding.previous_owner != self.owner
            || !Arc::ptr_eq(&binding.previous_context, self.context())
            || !binding.context.is_current()
        {
            return Err(EBUSY);
        }
        let foreign = binding.memory.is_some();
        if let Some(memory) = binding.memory.as_mut() {
            install_memory(
                alloc,
                descriptors,
                self.slot,
                memory,
                &mut binding.aliases,
                &mut binding.kicks,
            )?;
        } else {
            self.memory.set_owner(self.qids(), &binding.context)?;
        }
        let old_context = self.context().clone();
        let old_buffer = self.buffer_id();
        let old_pool = core::mem::replace(&mut self.free_list, binding.pool.clone());
        let old_memory = binding
            .memory
            .take()
            .map(|memory| core::mem::replace(&mut self.memory, memory));
        let retired = RetiredBinding {
            memory: old_memory,
            context: old_context,
            pool: old_pool,
            buffer_id: old_buffer,
            publication: None,
            ready: foreign,
        };
        if foreign {
            self.displaced = Some(retired);
        } else if self.previous.is_none() {
            self.previous = Some(retired);
        }
        self.memory.context = binding.context.clone();
        self.owner = Some(binding.owner);
        self.initial_packet = binding.initial_packet;
        if foreign { self.graph_started = false; }
        self.context_update = true;
        if foreign {
            self.cursors = [0; 2];
            self.configs = [None; 2];
        }
        Ok(())
    }
}

/// Whether `masks`, the stamp notifications collected for a pair whose tiling and fragment
/// queues are `qids`, can mean a finished pass: the fragment queue's bit. A fragment kick starts
/// only after its fragment barrier saw the tiling stamp, and retirement still checks both stamps
/// in memory, so the tiling notification is not needed to retire.
pub(crate) fn completion_noted(qids: [u8; 2], masks: [u64; 2]) -> bool {
    let fragment = usize::from(qids[1]);
    masks[fragment / 64] & (1u64 << (fragment % 64)) != 0
}

#[kunit_tests(asahi_g17_render_completion_noted)]
mod completion_noted_tests {
    use super::completion_noted;

    fn bit(qid: u8) -> [u64; 2] {
        let mut masks = [0; 2];
        masks[usize::from(qid) / 64] |= 1u64 << (qid % 64);
        masks
    }

    #[test]
    fn fragment_notification_alone_selects() {
        assert!(completion_noted([3, 4], bit(4)));
        assert!(completion_noted([70, 100], bit(100)));
    }

    #[test]
    fn tiling_notification_alone_does_not_select() {
        assert!(!completion_noted([3, 4], bit(3)));
        assert!(!completion_noted([70, 100], bit(70)));
        assert!(!completion_noted([3, 4], [0; 2]));
    }

    #[test]
    fn other_queues_do_not_select() {
        assert!(!completion_noted([3, 4], bit(5)));
        assert!(!completion_noted([3, 68], bit(4)));
        let mut both = bit(3);
        both[1] |= bit(68)[1];
        assert!(completion_noted([3, 68], both));
    }
}

