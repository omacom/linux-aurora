// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! J873 (26A428) TA/3D queue registration, binding and one render pass.
//!
//! Two SKSM kick queues, TA qid 9 (stage 0) and 3D qid 10 (stage 1), follow
//! the proven compute protocol of g18_queue_probe.rs (qid 11) step by step:
//! install (inner producer 1), scheduler bind (2), UMA pool + PB bind (3),
//! then one render pass [command, context config, kick] (6). Both queues share
//! ONE scheduler control record and ONE job list (M5 "Shared graphics
//! completion list"), and every inner command is encoded by
//! g18_render_command.rs, whose bytes equal the original TA/3D submitBuffer.
//!
//! Firmware references (GFX b000, SHA 2c63b343...): tag15 handler 0x12824
//! (0x98 bytes), queue slot table 0xb90b8 + qid*0x28 (stage word at +0x20),
//! scheduler table 0xb7468 + qid*8, UMA pool slots 0x5f660 + slot*0x100,
//! per-queue pool table 0x6f660 + qid*0x38, PB slot table 0x71420 + slot*0x50.
//!
//! Outer CL_2 records (24 bytes each) are returned, not published: the
//! firmware-memory owner (g18_resources.rs) owns the CL_2 ring and producer.
//! Addresses come from the address map; nothing here chooses a VA.

#![allow(dead_code)]

#[cfg(not(test))]
#[path = "graphics_application.rs"]
mod application;

use crate::g18::fw::render_command::{self as cmd, ChannelForm, G18ChannelConfig, G18ConfigUma,
    G18ConfigParameterBuffer, G18RenderStage, G18_CHANNEL_CONFIG_BYTES, G18_KICK_BYTES};

pub(crate) const G18_GRAPHICS_QUEUE_IDS: [u8; 2] = [9, 10];
pub(crate) const G18_GRAPHICS_STAGES: [G18RenderStage; 2] = [G18RenderStage::Ta, G18RenderStage::ThreeD];
pub(crate) const G18_ENTRY_RING_BYTES: usize = 0x20000; // b000 0x1257c: 256 * 0x200
pub(crate) const G18_GRAPH_BYTES: usize = 0x8000;
pub(crate) const G18_OUTER_RECORD_BYTES: usize = 24;
const PRIORITY: u32 = 2;
/// AGXChannel::getSubPriority value the compute path proved (config +0x5e).
const SUB_PRIORITY: u32 = 2;

// Graph layout (one firmware-private object; offsets are this module's own).
const CHANNEL: [usize; 2] = [0x0, 0x100];     // 0x80-byte M6 channel state
const POINTERS: [usize; 2] = [0x200, 0x300];  // pointer state (+0 consumer, +0x40 producer)
const JOB_LIST: usize = 0x400;                // shared scheduler job list
const CONTROL: usize = 0x500;                 // shared scheduler control (0x38)
const ITEMS: [usize; 2] = [0x800, 0x1000];    // u64 command-pointer rings
const CAPACITY: u32 = 0x100;
const CONFIGS: [usize; 2] = [0x1800, 0x2000]; // install/bind/pool/draw tag15 at 0x100 stride
const KICKS: [usize; 2] = [0x2800, 0x2c00];   // 0x40 stride

const _: () = assert!(ITEMS[1] + CAPACITY as usize * 8 <= CONFIGS[0]);
const _: () = assert!(CONFIGS[1] + 0x400 <= KICKS[0] && KICKS[1] + 0x400 <= G18_GRAPH_BYTES);

/// Addresses from the address map (see the G18 render-command notes, section 5).
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18GraphicsQueueAddresses {
    /// SKSM entry rings: firmware VA (kernel_vm, PRIV_RW) and the kernel low
    /// alias (kernel_lower_vm, GPU_SHARED_RW) of the same pages, per stage.
    pub entry_high: [u64; 2],
    pub entry_low: [u64; 2],
    /// Queue graph, firmware VA (kernel_vm, PRIV_RW), G18_GRAPH_BYTES.
    pub graph: u64,
}

/// One 24-byte outer CL_2 record: b000 walkers 0x274cc..0x27598 read +8
/// queue address, +0x10 ring DM (2), +0x14 inner producer, +0x16 qid, +0x17
/// cache flag. Both stage queues travel on the CL_2 ring.
pub(crate) fn encode_g18_outer_record(queue_address: u64, queue_id: u8, inner_head: u16)
    -> [u8; G18_OUTER_RECORD_BYTES] {
    let mut record = [0; G18_OUTER_RECORD_BYTES];
    record[8..16].copy_from_slice(&queue_address.to_le_bytes());
    record[16..20].copy_from_slice(&2u32.to_le_bytes());
    record[20..22].copy_from_slice(&inner_head.to_le_bytes());
    record[22] = queue_id;
    record[23] = 1;
    record
}

/// Everything one render pass needs beyond the queues themselves.
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18QueuedPass {
    /// TA and 3D command firmware VAs (SKSM payload[0], inner item 0).
    pub command_high: [u64; 2],
    pub context_id: u32,
    pub scheduler_control: u64,
    pub scheduler_index: u32,
    pub uma: G18ConfigUma,
    /// TA only; `None` disables the PB tuple (3D never carries one).
    pub parameter_buffer: Option<G18ConfigParameterBuffer>,
    /// RCE lists returned by encode_g18_ta_command / encode_g18_3d_command.
    pub rce: [[m5_rce::M5SksmRce; 4]; 2],
    /// Event stamp of each stage (0x100 per generation) and the queues'
    /// event-machine generation for it.
    pub stamps: [u32; 2],
    pub event_generation: [u64; 2],
    /// TA barrier record flag for the 3D's dependency (0: kind 1, the
    /// M5-proven choice; 1: HAL400 3D dependency type 3).
    pub ta_barrier_flag: u32,
}

use crate::g18::fw::sksm as m5_rce;

/// Kick-queue state the kernel mirrors (native queue +0x48/+0x98/+0xa0/+0x90).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct G18KickQueueState {
    pub current_stamp: u64,
    pub previous_stamp: Option<u64>,
    pub base: u64,
    /// Passes submitted on this queue (host bookkeeping only).
    pub submitted_kicks: u16,
}

/// Build one stage's SKSM entry and kick for a pass. Pure; the owner stores
/// them. `queue_address` is the stage's channel state (SKSM payload[1], the
/// native channel +0x80).
pub(crate) fn g18_stage_kick(
    stage: usize,
    pass: &G18QueuedPass,
    queue: G18KickQueueState,
    queue_address: u64,
) -> Result<(m5_rce::PreparedM5SksmEntry, [u8; G18_KICK_BYTES]), cmd::G18RenderCommandError> {
    let qid = G18_GRAPHICS_QUEUE_IDS[stage];
    let mut barriers = [cmd::G18Barrier { stamp: 0, queue_index: 0, flag: 0, generation: 0 }; 1];
    let barrier_count = if stage == 1 {
        barriers[0] = cmd::G18Barrier {
            stamp: pass.stamps[0], queue_index: G18_GRAPHICS_QUEUE_IDS[0],
            flag: pass.ta_barrier_flag, generation: pass.event_generation[0],
        };
        cmd::g18_order_3d_barriers(&mut barriers, u32::from(G18_GRAPHICS_QUEUE_IDS[0]));
        1
    } else { 0 };
    let entry = cmd::encode_g18_kick_entry(&cmd::G18KickEntryInputs {
        stage: G18_GRAPHICS_STAGES[stage],
        queue: m5_rce::M5SksmQueue { id: qid, priority: PRIORITY as u8,
            current_stamp: queue.current_stamp, submitted: 0, previous_stamp: queue.previous_stamp },
        entries: 256, base: queue.base, stamp: pass.stamps[stage],
        generation: pass.event_generation[stage], barriers: &barriers[..barrier_count],
        pipeline_dependency: 0, scheduler_index: pass.scheduler_index as u8, qos_word: 0xffff,
        payload: [pass.command_high[stage], queue_address], rce: pass.rce[stage],
        flag_bit1: false, flag_bit61: false, flag_bit63: false, event_flags: 0,
        auxiliary: 0x3f_ffff_ffff_ffff,
    })?;
    // Native kick count = this descriptor's new entries (NOPs + 1); Linux
    // never needs NOPs because every stamp is consecutive (encodeNops 0).
    let kick = cmd::encode_g18_kick(G18_GRAPHICS_STAGES[stage], qid, PRIORITY as u8,
        queue.current_stamp, 1)?;
    Ok((entry, kick))
}

/// The tag15 configuration for a stage and protocol step (compute-proven
/// staging: 0 install, 1 scheduler bind, 2 pool/PB bind, 3 pass context).
pub(crate) fn g18_stage_config(
    stage: usize,
    step: u32,
    entry: (u64, u64),
    pass: &G18QueuedPass,
) -> [u8; G18_CHANNEL_CONFIG_BYTES] {
    let mut out = [0u8; G18_CHANNEL_CONFIG_BYTES];
    let bound = step >= 2;
    let config = G18ChannelConfig {
        ring_low: entry.0, ring_high: entry.1, queue_id: u16::from(G18_GRAPHICS_QUEUE_IDS[stage]),
        install: step == 0, priority: PRIORITY, data_master: stage as u32,
        state_update_flag: false, state_update: 0, context_id: pass.context_id,
        uma: if bound { pass.uma } else { G18ConfigUma { state: 0, generation: None, slot: u32::MAX } },
        channel_control: pass.scheduler_control, update_scheduler: step == 1,
        scheduler_index: 0, pid: 0, sub_priority: SUB_PRIORITY,
        update_context: step == 3, gart_flag: step == 3,
        extension_flag: false, extension: 0,
        parameter_buffer: if bound && stage == 0 { pass.parameter_buffer } else { None },
    };
    // Infallible: the output is exactly one M6 record.
    let _ = cmd::encode_channel_config(ChannelForm::M6, &config, &mut out);
    out
}

// ---- Kernel owner -------------------------------------------------------------

#[cfg(not(test))]
use crate::g18::buffer::object::MappedObject;
#[cfg(not(test))]
use crate::{driver::AsahiDrmDevice, mmu};
#[cfg(not(test))]
use core::sync::atomic::{fence, Ordering};
#[cfg(not(test))]
use kernel::{c_str, io, prelude::*};

#[cfg(not(test))]
const DATA_SIZE: usize = 0xbc000;
#[cfg(not(test))]
const DATA_BASE: usize = 0x50000;
#[cfg(not(test))]
const SLOT_TABLE: usize = 0xb90b8 - DATA_BASE;
#[cfg(not(test))]
const SCHEDULER_TABLE: usize = 0xb7468 - DATA_BASE;
#[cfg(not(test))]
const POOL_TABLE: usize = 0x5f660 - DATA_BASE;
#[cfg(not(test))]
const QUEUE_POOL_TABLE: usize = 0x6f660 - DATA_BASE;
#[cfg(not(test))]
const PB_TABLE: usize = 0x71420 - DATA_BASE;

/// Read-only view of reserved GFX DATA (same rules as g18_queue_probe.rs:
/// invalidate before reading, never clean firmware lines).
#[cfg(not(test))]
struct DataView {
    map: io::mem::Mem,
    cache_line: usize,
}
// SAFETY: bounded volatile reads of a fixed reserved mapping only.
#[cfg(not(test))]
unsafe impl Send for DataView {}
// SAFETY: concurrent reads cannot mutate host-owned state.
#[cfg(not(test))]
unsafe impl Sync for DataView {}

#[cfg(not(test))]
impl DataView {
    fn new(dev: &AsahiDrmDevice) -> Result<Self> {
        let resource = dev.as_ref().as_ref().of_node().ok_or(EINVAL)?
            .reserved_mem_region_to_resource_byname(c_str!("gfx-data"))?;
        if resource.size() != DATA_SIZE as u64 || resource.start() < 0x100_0000_0000
            || resource.start() & 0x3fff != 0
            || resource.start().checked_add(resource.size()).ok_or(EOVERFLOW)? > 0x104_0000_0000 {
            return Err(EINVAL);
        }
        let ctr: u64;
        // SAFETY: architectural read-only cache geometry register.
        unsafe { core::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr,
            options(nomem, nostack, preserves_flags)) };
        let cache_line = 4usize << ((ctr >> 16) & 0xf);
        if cache_line > 0x4000 { return Err(EINVAL); }
        // SAFETY: WB matches reserved System RAM; the host never dirties it.
        let map = unsafe { io::mem::Mem::try_new(resource, io::mem::MemFlag::WB.into()) }?;
        Ok(Self { map, cache_line })
    }

    fn words<const N: usize>(&self, offset: usize) -> Result<[u64; N]> {
        let start = offset & !(self.cache_line - 1);
        let end = (offset + N * 8 + self.cache_line - 1) & !(self.cache_line - 1);
        if offset % 8 != 0 || end > DATA_SIZE { return Err(EINVAL); }
        for line in (start..end).step_by(self.cache_line) {
            // SAFETY: bounds checked above; invalidate only.
            unsafe { core::arch::asm!("dc ivac, {addr}",
                addr = in(reg) self.map.ptr().add(line), options(nostack, preserves_flags)) };
        }
        // SAFETY: complete invalidation before observing firmware stores.
        unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)) };
        let mut out = [0; N];
        for (index, word) in out.iter_mut().enumerate() {
            // SAFETY: aligned reads inside the checked extent.
            *word = unsafe { core::ptr::read_volatile(
                self.map.ptr().add(offset + index * 8).cast::<u64>()) };
        }
        fence(Ordering::Acquire);
        Ok(out)
    }
}

/// Firmware-side observations for both queues.
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18GraphicsFirmwareStatus {
    /// Queue slot (5 words, +0x20 low half is the stage word) per stage.
    pub slots: [[u64; 5]; 2],
    /// Scheduler table entry per stage (the control pointer).
    pub schedulers: [u64; 2],
    /// Control header, membership words.
    pub control: [u64; 3],
}

#[cfg(not(test))]
pub(crate) struct G18GraphicsQueues {
    aliases: KVec<mmu::KernelMapping>,
    entries: KVec<MappedObject>,
    graph: MappedObject,
    firmware: DataView,
    addresses: G18GraphicsQueueAddresses,
    step: u32,
    queues: [G18KickQueueState; 2],
}

#[cfg(not(test))]
fn put32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}
#[cfg(not(test))]
fn put64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

#[cfg(not(test))]
impl G18GraphicsQueues {
    /// Allocate both entry rings (dual-mapped, same pages proven) and the
    /// graph; initialize channel/pointer states (AGXChannel::resetChannelState
    /// 0xfffffe0008bdd178), the shared job list and the shared control
    /// (AGXCommandQueueReclaimableState::init 0xfffffe0008bf9014). Nothing is
    /// published: all inner producers stay 0.
    pub(crate) fn new(dev: &AsahiDrmDevice, uat: &mmu::Uat, a: G18GraphicsQueueAddresses) -> Result<Self> {
        if !uat.is_t8152_geometry() { return Err(ENODEV); }
        let mut entries = KVec::new();
        let mut aliases = KVec::new();
        for stage in 0..2 {
            let mut entry = MappedObject::new_wc_at(dev, uat.kernel_vm(), a.entry_high[stage],
                G18_ENTRY_RING_BYTES, mmu::PROT_FW_PRIV_RW)?;
            aliases.push(entry.map_alias_at(uat.kernel_lower_vm(), a.entry_low[stage],
                mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
            for offset in (0..G18_ENTRY_RING_BYTES).step_by(mmu::UAT_PGSZ) {
                if uat.kernel_vm().translate_iova(a.entry_high[stage] + offset as u64)?
                    != uat.kernel_lower_vm().translate_iova(a.entry_low[stage] + offset as u64)? {
                    return Err(EFAULT);
                }
            }
            entries.push(entry, GFP_KERNEL)?;
        }
        let mut graph = MappedObject::new_wc_at(dev, uat.kernel_vm(), a.graph, G18_GRAPH_BYTES,
                                                mmu::PROT_FW_PRIV_RW)?;
        graph.with_bytes_mut(|out| {
            for stage in 0..2 {
                let channel = CHANNEL[stage];
                put64(out, channel, a.graph + POINTERS[stage] as u64);
                put64(out, channel + 8, a.graph + ITEMS[stage] as u64);
                // One job list for both queues (M5 lesson).
                put64(out, channel + 0x10, a.graph + JOB_LIST as u64);
                put32(out, channel + 0x24, u32::MAX);
                put32(out, channel + 0x28, 4);
                put32(out, channel + 0x40, u32::MAX);
                put32(out, channel + 0x44, 0);
                put64(out, channel + 0x70, a.graph + CONTROL as u64);
                put32(out, POINTERS[stage] + 0x50, u32::MAX);
                put32(out, POINTERS[stage] + 0x60, CAPACITY);
            }
            put64(out, JOB_LIST + 8, a.graph + JOB_LIST as u64);
            out[CONTROL..CONTROL + 2].fill(0xff);
            out[CONTROL + 5] = 1;
            out[CONTROL + 0x33] = 0xff;
            out[CONTROL + 0x26] = 2;
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        Ok(Self { aliases, entries, graph, firmware: DataView::new(dev)?, addresses: a, step: 0,
                  queues: [G18KickQueueState { current_stamp: 1, previous_stamp: None, base: 1,
                                               submitted_kicks: 0 }; 2] })
    }

    /// Channel-state firmware VA: outer record +8 and SKSM payload[1].
    pub(crate) fn queue_address(&self, stage: usize) -> u64 {
        self.addresses.graph + CHANNEL[stage & 1] as u64
    }

    pub(crate) fn scheduler_control(&self) -> u64 { self.addresses.graph + CONTROL as u64 }

    pub(crate) fn job_list(&self) -> u64 { self.addresses.graph + JOB_LIST as u64 }

    pub(crate) fn counters(&mut self) -> Result<[[u32; 3]; 2]> {
        let mut result = [[0; 3]; 2];
        for stage in 0..2 {
            let base = self.addresses.graph + POINTERS[stage] as u64;
            result[stage] = [self.graph.read_u32(base)?, self.graph.read_u32(base + 0x30)?,
                             self.graph.read_u32(base + 0x40)?];
        }
        Ok(result)
    }

    pub(crate) fn completion_list(&mut self) -> Result<[u64; 2]> {
        let list = self.job_list();
        Ok([self.graph.read_u64(list)?, self.graph.read_u64(list + 8)?])
    }

    pub(crate) fn firmware_status(&mut self) -> Result<G18GraphicsFirmwareStatus> {
        let mut slots = [[0; 5]; 2];
        let mut schedulers = [0; 2];
        for stage in 0..2 {
            let qid = G18_GRAPHICS_QUEUE_IDS[stage] as usize;
            slots[stage] = self.firmware.words(SLOT_TABLE + qid * 0x28)?;
            schedulers[stage] = self.firmware.words::<1>(SCHEDULER_TABLE + qid * 8)?[0];
        }
        let control = self.scheduler_control();
        Ok(G18GraphicsFirmwareStatus { slots, schedulers,
            control: [self.graph.read_u64(control)?, self.graph.read_u64(control + 0x12)?,
                      self.graph.read_u64(control + 0x1a)?] })
    }

    /// Both firmware slots name our rings with the right stage words
    /// (b000 0x12cb4..0x12cd4; +0x20 = tag15 +0x22).
    pub(crate) fn installed(status: &G18GraphicsFirmwareStatus, a: &G18GraphicsQueueAddresses) -> bool {
        (0..2).all(|stage| {
            let slot = status.slots[stage];
            slot[0] & 1 != 0 && slot[1] == a.entry_high[stage] && slot[2] == a.entry_low[stage]
                && slot[3] == 4u64 << 32 && slot[4] as u32 == stage as u32
        })
    }

    /// Scheduler bound for both queues: shared control published in both
    /// scheduler entries, allocated indices, state 4, membership bits 9/10.
    pub(crate) fn scheduler_bound(&self, status: &G18GraphicsFirmwareStatus) -> bool {
        let header = status.control[0];
        let members = (1u64 << G18_GRAPHICS_QUEUE_IDS[0]) | (1u64 << G18_GRAPHICS_QUEUE_IDS[1]);
        status.schedulers.iter().all(|&pointer| pointer == self.scheduler_control())
            && header & 0xff != 0xff && (header >> 8) & 0xff != 0xff
            && (header >> 16) & 0xff == 4 && status.control[1] == members && status.control[2] == 0
    }

    /// Firmware UMA-pool and PB slot entries, for the pool/PB bind gate.
    pub(crate) fn pool_and_pb_status(&mut self, uma_slot: u32, pb_slot: u32) -> Result<([u64; 5], [u64; 6])> {
        let pool = self.firmware.words::<3>(POOL_TABLE + uma_slot as usize * 0x100)?;
        let members = self.firmware.words::<2>(POOL_TABLE + uma_slot as usize * 0x100 + 0x78)?;
        let pb = self.firmware.words::<6>(PB_TABLE + pb_slot as usize * 0x50)?;
        Ok(([pool[0], pool[1], pool[2], members[0], members[1]], pb))
    }

    pub(crate) fn stage_gate(&mut self, phase: u32, pass: &G18QueuedPass) -> Result<bool> {
        let status = self.firmware_status()?;
        if self.counters()? != [[phase; 3]; 2]
            || !Self::installed(&status, &self.addresses) { return Ok(false); }
        if phase >= 2 && !self.scheduler_bound(&status) { return Ok(false); }
        if phase >= 3 {
            let pb = pass.parameter_buffer.ok_or(EINVAL)?;
            let (pool, slot) = self.pool_and_pb_status(pass.uma.slot, pb.slot)?;
            // b000 0x13708..0x137c0 sets the queue bit at slot +0x18 and the
            // manager bit at +0x28; +0x20 stays clear. Stage33b hardware read
            // exactly [state, 1, 3, 0x200, 0, 0x400] after the pool records.
            if pool != [pass.uma.state, pass.uma.generation.ok_or(EINVAL)?, (2u64 << 32) | 3,
                (1 << 9) | (1 << 10), 0]
                || slot != [pb.state, pb.generation, u64::from(pass.context_id), 1 << 9, 0, 1 << 10] {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Gate before preparing any work item. Both rows, context and resource
    /// fields are checked by the job/table owners; these are the queue checks.
    pub(crate) fn audit_before_pass(&mut self, uat: &mmu::Uat, pass: &G18QueuedPass) -> Result {
        if self.step != 3 || !self.stage_gate(3, pass)? { return Err(EIO); }
        let a = self.addresses;
        for stage in 0..2 {
            self.graph.with_bytes_mut(|out| {
                if out[ITEMS[stage]+24..ITEMS[stage]+48].iter().any(|&b| b != 0) { return Err(EBUSY); }
                for phase in 0..3 {
                    let expected = g18_stage_config(stage, phase, (a.entry_low[stage], a.entry_high[stage]), pass);
                    let off = CONFIGS[stage]+phase as usize*0x100;
                    if out[off..off+G18_CHANNEL_CONFIG_BYTES] != expected { return Err(EIO); }
                }
                Ok(())
            })?;
            self.entries[stage].with_bytes_mut(|out| {
                if out[0x200..0x400].iter().any(|&b| b != 0) { return Err(EBUSY); }
                Ok(())
            })?;
            for off in (0..G18_ENTRY_RING_BYTES).step_by(mmu::UAT_PGSZ) {
                if uat.kernel_vm().translate_iova(a.entry_high[stage] + off as u64)?
                    != uat.kernel_lower_vm().translate_iova(a.entry_low[stage] + off as u64)? { return Err(EFAULT); }
            }
        }
        for off in (0..G18_GRAPH_BYTES).step_by(mmu::UAT_PGSZ) {
            uat.kernel_vm().translate_iova(a.graph + off as u64)?;
        }
        Ok(())
    }

    /// Compare the retained command records to the shared pure encoders and
    /// print their actual bytes' digest before producer 6 can be published.
    pub(crate) fn audit_prepared_pass(&mut self, dev: &AsahiDrmDevice, pass: &G18QueuedPass) -> Result {
        if self.step != 4 || self.counters()? != [[3; 3]; 2] { return Err(EBUSY); }
        let a = self.addresses;
        for stage in 0..2 {
            for phase in 0..4 {
                let expected = g18_stage_config(stage, phase, (a.entry_low[stage], a.entry_high[stage]), pass);
                self.graph.with_bytes_mut(|out| {
                    let off = CONFIGS[stage] + phase as usize*0x100;
                    let raw = &out[off..off+G18_CHANNEL_CONFIG_BYTES];
                    if raw != expected { return Err(EIO); }
                    dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=tag15-{}-{} fnv={:#018x}\n", stage, phase,
                        crate::g18::fw::graphics_payload::fnv64(raw));
                    Ok(())
                })?;
                let outer = encode_g18_outer_record(self.queue_address(stage), G18_GRAPHICS_QUEUE_IDS[stage],
                    [1,2,3,6][phase as usize]);
                dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=outer-{}-{} fnv={:#018x}\n", stage, phase,
                    crate::g18::fw::graphics_payload::fnv64(&outer));
            }
            let (entry, kick) = g18_stage_kick(stage, pass,
                G18KickQueueState { current_stamp: 1, previous_stamp: None, base: 1, submitted_kicks: 0 },
                self.queue_address(stage)).map_err(|_| EINVAL)?;
            self.entries[stage].with_bytes_mut(|out| {
                let raw = &out[0x200..0x400];
                if raw != entry.bytes { return Err(EIO); }
                dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=sksm-{} fnv={:#018x}\n",stage,
                    crate::g18::fw::graphics_payload::fnv64(raw));
                Ok(())
            })?;
            self.graph.with_bytes_mut(|out| {
                let raw = &out[KICKS[stage]..KICKS[stage]+G18_KICK_BYTES];
                if raw != kick { return Err(EIO); }
                dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=kick-{} fnv={:#018x}\n",stage,
                    crate::g18::fw::graphics_payload::fnv64(raw));
                Ok(())
            })?;
        }
        Ok(())
    }

    fn stage_step(&mut self, step: u32, pass: &G18QueuedPass) -> Result<[[u8; G18_OUTER_RECORD_BYTES]; 2]> {
        if self.step != step || step > 2 { return Err(EBUSY); }
        let expected = [step; 3];
        if self.counters()? != [expected; 2] { return Err(EBUSY); }
        let a = self.addresses;
        let mut records = [[0; G18_OUTER_RECORD_BYTES]; 2];
        for stage in 0..2 {
            let config = g18_stage_config(stage, step, (a.entry_low[stage], a.entry_high[stage]), pass);
            let at = CONFIGS[stage] + step as usize * 0x100;
            let item = ITEMS[stage] + step as usize * 8;
            self.graph.with_bytes_mut(|out| {
                if out[at..at + 0x100].iter().any(|b| *b != 0)
                    || out[item..item + 8].iter().any(|b| *b != 0) { return Err(EBUSY); }
                out[at..at + G18_CHANNEL_CONFIG_BYTES].copy_from_slice(&config);
                put64(out, item, a.graph + at as u64);
                Ok(())
            })?;
            records[stage] = encode_g18_outer_record(self.queue_address(stage),
                G18_GRAPHICS_QUEUE_IDS[stage], step as u16 + 1);
        }
        fence(Ordering::SeqCst);
        for stage in 0..2 {
            self.graph.write_u32(a.graph + POINTERS[stage] as u64 + 0x40, step + 1)?;
        }
        fence(Ordering::SeqCst);
        self.step = step + 1;
        Ok([records[1], records[0]])
    }

    /// Step 1: install both queues (tag15 install=1, stage words 0/1).
    pub(crate) fn install_records(&mut self, pass: &G18QueuedPass) -> Result<[[u8; G18_OUTER_RECORD_BYTES]; 2]> {
        self.stage_step(0, pass)
    }

    /// Step 2 (after `installed`): bind both queues to the shared control.
    pub(crate) fn scheduler_records(&mut self, pass: &G18QueuedPass) -> Result<[[u8; G18_OUTER_RECORD_BYTES]; 2]> {
        self.stage_step(1, pass)
    }

    /// Step 3 (after `scheduler_bound`): UMA pool on both, PB tuple on TA.
    /// The UMA descriptor and PB row must already be loaded and read back.
    pub(crate) fn pool_records(&mut self, pass: &G18QueuedPass) -> Result<[[u8; G18_OUTER_RECORD_BYTES]; 2]> {
        if pass.uma.slot == u32::MAX || pass.uma.state == 0 { return Err(EINVAL); }
        if let Some(pb) = pass.parameter_buffer {
            if pb.generation == 0 || pb.state == 0 { return Err(EINVAL); }
        }
        self.stage_step(2, pass)
    }

    /// Step 4: store both SKSM entries and [command, context config, kick]
    /// per queue, unpublished. The job must have encoded both commands,
    /// incremented the PB counter, loaded its roots and passed every
    /// pre-kick check before `publish_pass`.
    pub(crate) fn prepare_pass(&mut self, pass: &G18QueuedPass) -> Result {
        if self.step != 3 || self.counters()? != [[3; 3]; 2] { return Err(EBUSY); }
        let a = self.addresses;
        for stage in [1, 0] {
            let (entry, kick) = g18_stage_kick(stage, pass, self.queues[stage],
                self.queue_address(stage)).map_err(|_| EINVAL)?;
            let offset = entry.offset as usize;
            self.entries[stage].with_bytes_mut(|out| {
                let slot = out.get_mut(offset..offset + entry.bytes.len()).ok_or(EINVAL)?;
                if slot.iter().any(|b| *b != 0) { return Err(EBUSY); }
                slot.copy_from_slice(&entry.bytes);
                Ok(())
            })?;
            let config = g18_stage_config(stage, 3, (a.entry_low[stage], a.entry_high[stage]), pass);
            let at = CONFIGS[stage] + 0x300;
            self.graph.with_bytes_mut(|out| {
                if out[at..at + 0x100].iter().any(|b| *b != 0)
                    || out[KICKS[stage]..KICKS[stage] + 0x40].iter().any(|b| *b != 0)
                    || out[ITEMS[stage] + 24..ITEMS[stage] + 48].iter().any(|b| *b != 0) {
                    return Err(EBUSY);
                }
                out[at..at + G18_CHANNEL_CONFIG_BYTES].copy_from_slice(&config);
                out[KICKS[stage]..KICKS[stage] + G18_KICK_BYTES].copy_from_slice(&kick);
                for (index, address) in [pass.command_high[stage], a.graph + at as u64,
                                         a.graph + KICKS[stage] as u64].into_iter().enumerate() {
                    put64(out, ITEMS[stage] + (index + 3) * 8, address);
                }
                Ok(())
            })?;
            self.queues[stage] = G18KickQueueState {
                current_stamp: (self.queues[stage].current_stamp + 1) & ((1 << 40) - 1),
                previous_stamp: Some(self.queues[stage].current_stamp),
                base: self.queues[stage].base,
                submitted_kicks: self.queues[stage].submitted_kicks + 1,
            };
        }
        fence(Ordering::SeqCst);
        self.step = 4;
        Ok(())
    }

    /// Publish both inner producers (6) and return the two outer records.
    /// A later failure never authorizes resetting rings or entries.
    pub(crate) fn publish_pass(&mut self) -> Result<[[u8; G18_OUTER_RECORD_BYTES]; 2]> {
        if self.step != 4 || self.counters()? != [[3; 3]; 2] { return Err(EBUSY); }
        fence(Ordering::SeqCst);
        let mut records = [[0; G18_OUTER_RECORD_BYTES]; 2];
        for stage in [1, 0] {
            self.graph.write_u32(self.addresses.graph + POINTERS[stage] as u64 + 0x40, 6)?;
            records[stage] = encode_g18_outer_record(self.queue_address(stage),
                G18_GRAPHICS_QUEUE_IDS[stage], 6);
        }
        fence(Ordering::SeqCst);
        self.step = 5;
        Ok([records[1], records[0]])
    }
}

// ---- Parameter-buffer kernel owner -------------------------------------------
//
// The PM object graph below keeps the layout native AGXParameterManagement
// init (0xfffffe0008c5eee4) builds and that M5 proved on hardware
// (m5_graphics_job.rs PM_BLOCKS/PM_SHARED/PM_COUNTER/SCENES): a per-block page
// base table (state +0x44 = PM +0xc0, u32 at 8-byte stride = PM +0x90), a
// shared header (state +0x4c = PM +0xc8; scene records link to its +0x40 =
// PM +0xa0), the submitted-operations counter (state +0x64) and 0x80-byte scene
// records (TA +0x28 = PM +0xa8[scene], TA +0x30 = PM +0xb8 = the record after
// the last). Scene record fields follow the native init loop (0x8c5f850..
// 0x8c5f874) and setupSceneState (0x8c61ba8..0x8c61bc8); the shared header
// words are M5-derived and marked [OPEN] in the G18 render-command notes.

use crate::g18::fw::parameter_buffer::*;

const PM_GRAPH_BYTES: usize = 0x8000;
// The block table holds an entry for each of the PB_CAPACITY ring slots the
// firmware is given. It stays at the start of the graph (behind the scene
// records, at +0x8000 of a larger graph, the first frame faulted); the shared
// header, the counter and the scene records follow it.
const BLOCK_TABLE: usize = 0;
const SHARED: usize = 0x3400;
const COUNTER: usize = 0x3500;
const SCENES: usize = 0x4000;
const _: () = assert!(
    BLOCK_TABLE + crate::g18::uapi::render_policy::PB_CAPACITY as usize * 8 <= SHARED
);
const SCENE_BYTES: usize = 0x80;

fn pb_put32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn pb_put64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}
/// Native metrics spacing: device PM config word 1 (0x40) rounded (0x8c5f7b0).
const METRICS_STRIDE: u64 = 0x40;
const METRICS_BYTES: usize = 0x4000;
const SCENE_SCRATCH_BYTES: usize = 0x4000;
const DISCARD_BYTES: usize = 0x8000;
pub(crate) const G18_PB_MAX_SCENES: u32 = ((PM_GRAPH_BYTES - SCENES) / SCENE_BYTES - 1) as u32;

/// Fixed addresses from the address map. `*_high` are firmware VAs (global,
/// kernel_vm); `*_low` are GPU VAs that the TA/3D hardware dereferences in
/// the job's context: they are created in the caller's low VM (the job VM)
/// and may be re-mapped into further windows with `alias_low_window`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18PbAddresses {
    /// PB state object (0x4000 allocation, 0xc0 used). Firmware writes +0x08.
    pub state_high: u64,
    /// PM graph: block table, shared header, counter, scene records (0x8000).
    pub graph_high: u64,
    pub page_list_high: u64,
    pub page_list_low: u64,
    /// The page-list VA written to PB state +0x28 and the PB row: the job
    /// context's view the PM decodes (see `g18_pb_row_address_valid`).
    pub page_list_row: u64,
    pub metrics_high: u64,
    pub metrics_low: u64,
    pub scene_scratch_low: u64,
    /// Block 0; blocks are contiguous at 0x80000 stride.
    pub blocks_low: u64,
    pub discard_low: u64,
    /// Added (wrapping) to a low VA to form the user-process view native
    /// uses via getGPUVirtualAddressForUserProcess (block page bases, discard
    /// page, scene cycle). 0 when the job maps the PB unbiased; the address
    /// map's COMPACT_BIAS when the page list must name the biased window.
    pub user_view_delta: u64,
}

/// Host policy for one PB. Every field is a documented choice, not a default
/// silently inherited from M5 (see the G18 render-command notes, section 4).
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18PbPolicy {
    /// Backed blocks (native initial allocation); must be < ring capacity.
    pub blocks: u32,
    /// PM +0x148 ring capacity, +0x154/+0x15c, +0x1d4.
    pub capacity: u32,
    pub pm_154: u32,
    pub pm_15c: u32,
    pub pm_1d4: u32,
    /// Scene records and the selected scene (descriptor PM +0x2c).
    pub scene_count: u32,
    pub scene: u32,
    /// PB table slot (TA HardwareBufferID).
    pub slot: u32,
    /// tag15 +0x82. Must differ from the firmware slot's cached generation
    /// (0 on a fresh boot) or firmware never fetches the PB state.
    pub generation: u64,
}

/// Values the TA/3D commands take from the PB.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct G18PbCommandFields {
    pub pb_slot: u32,
    pub pm_1d4: u32,
    pub pb_state: u64,
    pub pm_scene: u64,
    pub pm_b8: u64,
}

/// Pre-kick readback of the published PB objects.
#[derive(Clone, Copy, Debug)]
pub(crate) struct G18PbReadback {
    pub row: [u8; G18_PB_ROW_BYTES],
    pub state: [u8; G18_PB_STATE_FIRMWARE_BYTES],
    pub counter: u32,
}

#[cfg(not(test))]
pub(crate) struct G18ParameterBuffer {
    // Window aliases go first: that VM's roots may still be live.
    window_aliases: KVec<mmu::KernelMapping>,
    low_aliases: KVec<mmu::KernelMapping>,
    state: MappedObject,
    graph: MappedObject,
    page_list: MappedObject,
    metrics: MappedObject,
    scene_scratch: MappedObject,
    blocks: KVec<MappedObject>,
    /// The user charges of the grown blocks ([`Self::grow`]).
    charges: KVec<crate::memacct::Charge>,
    discard: MappedObject,
    addresses: G18PbAddresses,
    policy: G18PbPolicy,
    inputs: G18PbStateInputs,
    row: Option<[u8; G18_PB_ROW_BYTES]>,
}

#[cfg(not(test))]
impl G18ParameterBuffer {
    /// Allocate, map and encode every PB object. Nothing is published to
    /// firmware: the PB-table row is written separately by `load_row`, and
    /// the tag15 that names the PB comes later still.
    pub(crate) fn new(
        dev: &AsahiDrmDevice,
        uat: &mmu::Uat,
        low_vm: &mmu::Vm,
        a: G18PbAddresses,
        policy: G18PbPolicy,
    ) -> Result<Self> {
        if !uat.is_t8152_geometry() || mmu::UAT_PGSZ != 0x4000 {
            return Err(ENODEV);
        }
        if policy.blocks == 0 || policy.blocks >= policy.capacity
            || policy.scene_count == 0 || policy.scene_count > G18_PB_MAX_SCENES
            || policy.scene >= policy.scene_count || policy.slot >= G18_PB_TABLE_ROWS
            || policy.generation == 0
            || u64::from(policy.scene_count) * METRICS_STRIDE > METRICS_BYTES as u64
            || a.blocks_low & (G18_PB_BLOCK_BYTES - 1) != 0 || a.discard_low & 0x7fff != 0
            || a.user_view_delta & 0x7fff != 0
            || !g18_pb_row_address_valid(a.page_list_row) {
            return Err(EINVAL);
        }
        let list_entries = usize::try_from(policy.capacity)?
            .checked_mul(G18_PB_PAGES_PER_BLOCK as usize).ok_or(EOVERFLOW)?;
        let list_bytes = list_entries.checked_mul(4).ok_or(EOVERFLOW)?
            .checked_add(mmu::UAT_PGSZ - 1).ok_or(EOVERFLOW)? & !(mmu::UAT_PGSZ - 1);
        let high = |address, size| MappedObject::new_wc_at(dev, uat.kernel_vm(), address, size,
                                                          mmu::PROT_FW_SHARED_RW);
        let low = |address, size| MappedObject::new_wc_at(dev, low_vm, address,
                                                         size, mmu::PROT_GPU_SHARED_RW);
        let state = high(a.state_high, mmu::UAT_PGSZ)?;
        let graph = high(a.graph_high, PM_GRAPH_BYTES)?;
        let mut page_list = high(a.page_list_high, list_bytes)?;
        let mut metrics = high(a.metrics_high, METRICS_BYTES)?;
        let mut low_aliases = KVec::new();
        for (object, address, size) in [(&mut page_list, a.page_list_low, list_bytes),
                                        (&mut metrics, a.metrics_low, METRICS_BYTES)] {
            low_aliases.push(object.map_alias_at(low_vm, address,
                                                 mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
            for offset in (0..size).step_by(mmu::UAT_PGSZ) {
                if uat.kernel_vm().translate_iova(object.iova() + offset as u64)?
                    != low_vm.translate_iova(address + offset as u64)? {
                    return Err(EFAULT);
                }
            }
        }
        let scene_scratch = low(a.scene_scratch_low, SCENE_SCRATCH_BYTES)?;
        let discard = low(a.discard_low, DISCARD_BYTES)?;
        let mut blocks = KVec::new();
        for index in 0..u64::from(policy.blocks) {
            let address = a.blocks_low + index * G18_PB_BLOCK_BYTES;
            blocks.push(low(address, G18_PB_BLOCK_BYTES as usize)?, GFP_KERNEL)?;
        }
        let config = G18PmConfig {
            pm_148: policy.capacity,
            pm_138: u64::from(policy.blocks) << 19,
            pm_1d8: G18_PB_PAGES_PER_BLOCK,
            pm_70: G18_PB_BLOCK_BYTES,
            pm_c0: a.graph_high + BLOCK_TABLE as u64,
            pm_c8: a.graph_high + SHARED as u64,
            pm_154: policy.pm_154,
            pm_15c: policy.pm_15c,
            pm_1d4: policy.pm_1d4,
        };
        let inputs = G18PbStateInputs {
            config,
            page_list_firmware: a.page_list_high,
            page_list_gpu: a.page_list_row,
            page_list_bytes: u32::try_from(list_bytes)?,
            counter_gpu: a.graph_high + COUNTER as u64,
            discard_gpu: a.discard_low.wrapping_add(a.user_view_delta),
            slot: policy.slot,
            start_block: 0,
            end_block: policy.blocks,
        };
        let mut owner = Self {
            window_aliases: KVec::new(), low_aliases, state, graph, page_list, metrics,
            scene_scratch, blocks, charges: KVec::new(), discard, addresses: a, policy,
            inputs, row: None,
        };
        owner.encode()?;
        dev_info!(dev.as_ref(),
            "G18 PB: state={:#x} graph={:#x} list={:#x}/{:#x} blocks={}x{:#x}@{:#x} slot={} gen={}; unpublished\n",
            a.state_high, a.graph_high, a.page_list_high, a.page_list_low, policy.blocks,
            G18_PB_BLOCK_BYTES, a.blocks_low, policy.slot, policy.generation);
        Ok(owner)
    }

    fn page_bases(&self) -> Result<KVec<u32>> {
        let mut bases = KVec::with_capacity(self.policy.capacity as usize, GFP_KERNEL)?;
        for slot in 0..u64::from(self.policy.capacity) {
            // Unbacked ring slots are never reached before growth, which is
            // not implemented; they repeat block 0 so no stray page appears.
            let block = if slot < u64::from(self.policy.blocks) { slot } else { 0 };
            let va = (self.addresses.blocks_low + block * G18_PB_BLOCK_BYTES)
                .wrapping_add(self.addresses.user_view_delta);
            let base = u32::try_from(va >> 15).map_err(|_| ERANGE)?;
            if base >> 25 != 0 { return Err(ERANGE); }
            bases.push(base, GFP_KERNEL)?;
        }
        Ok(bases)
    }

    fn scene_cycle_at(&self, scene: u32) -> u64 {
        // Scene scratch in the user-process view (setupSceneState 0x8c61bb8).
        self.addresses.scene_scratch_low.wrapping_add(self.addresses.user_view_delta) + 0x40
            + u64::from(scene) * 0x40
    }

    fn encode(&mut self) -> Result {
        let bases = self.page_bases()?;
        let inputs = self.inputs;
        let blocks = self.policy.blocks as usize;
        self.page_list.with_bytes_mut(|out| {
            g18_pb_page_list(out, &inputs.config, &bases, 0, inputs.end_block)
                .map_err(|_| EINVAL)?;
            Ok(())
        })?;
        let a = self.addresses;
        let policy = self.policy;
        let mut cycles = KVec::with_capacity(policy.scene_count as usize, GFP_KERNEL)?;
        for scene in 0..policy.scene_count {
            cycles.push(self.scene_cycle_at(scene), GFP_KERNEL)?;
        }
        self.graph.with_bytes_mut(|out| {
            for (index, base) in bases.iter().take(blocks).enumerate() {
                pb_put32(out, BLOCK_TABLE + index * 8, *base);
            }
            // [OPEN] M5-proven shared header; +8 is buildPageList's first block.
            pb_put32(out, SHARED, policy.blocks);
            pb_put32(out, SHARED + 4, policy.blocks);
            pb_put32(out, SHARED + 8, 0);
            pb_put32(out, SHARED + 0x60, 1);
            pb_put32(out, COUNTER, 0);
            for scene in 0..policy.scene_count as usize {
                let record = SCENES + scene * SCENE_BYTES;
                pb_put64(out, record, a.metrics_low + scene as u64 * METRICS_STRIDE);
                pb_put64(out, record + 8, a.metrics_high + scene as u64 * METRICS_STRIDE);
                pb_put64(out, record + 0x28, cycles[scene]);
                pb_put64(out, record + 0x40, a.graph_high + SHARED as u64 + 0x40);
            }
            Ok(())
        })?;
        self.state.with_bytes_mut(|out| {
            let state: &mut [u8; G18_PB_STATE_BYTES] =
                (&mut out[..G18_PB_STATE_BYTES]).try_into().map_err(|_| EINVAL)?;
            encode_g18_pb_state(state, &inputs, true);
            Ok(())
        })?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    /// Map every low object again into `vm` at `low VA + delta` (delta 0:
    /// another VM at the same addresses; COMPACT_BIAS: the biased window of
    /// the dual low map). Verifies every page against `low_vm`, the VM the
    /// objects were created in. Call before the job's roots are published.
    pub(crate) fn alias_low_window(&mut self, low_vm: &mmu::Vm, vm: &mmu::Vm, delta: u64) -> Result {
        let a = self.addresses;
        let mut aliases = KVec::new();
        let mut ranges = KVec::new();
        let list_bytes = self.inputs.page_list_bytes as u64;
        let at = |address: u64| address.checked_add(delta).ok_or(EOVERFLOW);
        aliases.push(self.page_list.map_alias_at(vm, at(a.page_list_low)?, mmu::PROT_GPU_SHARED_RW)?,
                     GFP_KERNEL)?;
        ranges.push((a.page_list_low, list_bytes), GFP_KERNEL)?;
        aliases.push(self.metrics.map_alias_at(vm, at(a.metrics_low)?, mmu::PROT_GPU_SHARED_RW)?,
                     GFP_KERNEL)?;
        ranges.push((a.metrics_low, METRICS_BYTES as u64), GFP_KERNEL)?;
        aliases.push(self.scene_scratch.map_alias_at(vm, at(a.scene_scratch_low)?,
            mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
        ranges.push((a.scene_scratch_low, SCENE_SCRATCH_BYTES as u64), GFP_KERNEL)?;
        aliases.push(self.discard.map_alias_at(vm, at(a.discard_low)?, mmu::PROT_GPU_SHARED_RW)?,
                     GFP_KERNEL)?;
        ranges.push((a.discard_low, DISCARD_BYTES as u64), GFP_KERNEL)?;
        for (index, block) in self.blocks.iter_mut().enumerate() {
            let address = a.blocks_low + index as u64 * G18_PB_BLOCK_BYTES;
            aliases.push(block.map_alias_at(vm, at(address)?, mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
            ranges.push((address, G18_PB_BLOCK_BYTES), GFP_KERNEL)?;
        }
        for &(address, bytes) in ranges.iter() {
            for offset in (0..bytes).step_by(mmu::UAT_PGSZ) {
                if vm.translate_iova(at(address)? + offset)?
                    != low_vm.translate_iova(address + offset)? {
                    return Err(EFAULT);
                }
            }
        }
        fence(Ordering::SeqCst);
        for alias in aliases {
            self.window_aliases.push(alias, GFP_KERNEL)?;
        }
        Ok(())
    }

    /// The row halLoadPBDesc would write over `old` (ring zero after
    /// buildPageList).
    pub(crate) fn pb_row(&self, old: [u8; G18_PB_ROW_BYTES]) -> [u8; G18_PB_ROW_BYTES] {
        g18_pb_row(self.inputs.page_list_gpu, g18_pb_page_count(&self.inputs.config),
                   G18PbRing::default(), old)
    }

    /// The complete encoded PB state (firmware owns +0x08 after binding).
    pub(crate) fn encode_pm_state(&self, out: &mut [u8; G18_PB_STATE_BYTES]) {
        encode_g18_pb_state(out, &self.inputs, true);
    }

    /// Native loadPBDesc order: state +0x0c (already encoded) then the row.
    /// Refuses an occupied row or a second load; plain stores, then a fence,
    /// before any tag15 names this PB.
    pub(crate) fn load_row(&mut self, table: &mut MappedObject) -> Result<[u8; G18_PB_ROW_BYTES]> {
        if self.row.is_some() { return Err(EBUSY); }
        let offset = self.policy.slot as usize * G18_PB_ROW_BYTES;
        let row = table.with_bytes_mut(|out| {
            if out.len() != G18_PB_TABLE_ROWS as usize * G18_PB_ROW_BYTES { return Err(EINVAL); }
            let old: [u8; G18_PB_ROW_BYTES] = out[offset..offset + G18_PB_ROW_BYTES]
                .try_into().map_err(|_| EINVAL)?;
            if old != [0; G18_PB_ROW_BYTES] { return Err(EBUSY); }
            let row = self.pb_row(old);
            out[offset..offset + G18_PB_ROW_BYTES].copy_from_slice(&row);
            Ok(row)
        })?;
        fence(Ordering::SeqCst);
        self.row = Some(row);
        Ok(row)
    }

    /// Pre-kick self-check: re-read the table row, the firmware-visible state
    /// prefix (except the firmware-owned context word) and the page list
    /// head, and compare with the encoders. Any difference refuses the kick.
    pub(crate) fn verify_before_kick(&mut self, table: &mut MappedObject) -> Result<G18PbReadback> {
        let expected_row = self.row.ok_or(EINVAL)?;
        let offset = self.policy.slot as usize * G18_PB_ROW_BYTES;
        let row: [u8; G18_PB_ROW_BYTES] = table.with_bytes_mut(|out| {
            out.get(offset..offset + G18_PB_ROW_BYTES).ok_or(EINVAL)?.try_into().map_err(|_| EINVAL)
        })?;
        if row != expected_row || row == [0; G18_PB_ROW_BYTES] { return Err(EIO); }
        if g18_pb_row_decoded_address(&row) != Some(self.inputs.page_list_gpu) { return Err(EIO); }
        let mut expected = [0u8; G18_PB_STATE_BYTES];
        self.encode_pm_state(&mut expected);
        let state: [u8; G18_PB_STATE_FIRMWARE_BYTES] = self.state.with_bytes_mut(|out| {
            out.get(..G18_PB_STATE_FIRMWARE_BYTES).ok_or(EINVAL)?.try_into().map_err(|_| EINVAL)
        })?;
        for (index, (actual, wanted)) in state.iter().zip(expected.iter()).enumerate() {
            let firmware_owned = (8..12).contains(&index);
            let unwritten = (0x1c..0x20).contains(&index);
            if !firmware_owned && !unwritten && actual != wanted { return Err(EIO); }
        }
        let bases = self.page_bases()?;
        let head = self.page_list.read_u32(self.addresses.page_list_high)?;
        if head != bases[0] { return Err(EIO); }
        for index in 0..self.policy.blocks as usize {
            if self.graph.read_u32(self.addresses.graph_high + (index * 8) as u64)? != bases[index] {
                return Err(EIO);
            }
            for page in 0..G18_PB_PAGES_PER_BLOCK as usize {
                if self.page_list.read_u32(self.addresses.page_list_high + ((index * 16 + page) * 4) as u64)?
                    != bases[index] + page as u32 { return Err(EIO); }
            }
        }
        let counter = self.graph.read_u32(self.addresses.graph_high + COUNTER as u64)?;
        Ok(G18PbReadback { row, state, counter })
    }

    /// Native incrementPMSubmittedOperations (0x8bbcd50): one per 3D submit,
    /// before the 3D command is published.
    pub(crate) fn count_submitted_render(&mut self) -> Result<u32> {
        let address = self.addresses.graph_high + COUNTER as u64;
        let value = self.graph.read_u32(address)?.checked_add(1).ok_or(EOVERFLOW)?;
        self.graph.write_u32(address, value)?;
        fence(Ordering::SeqCst);
        Ok(value)
    }

    pub(crate) fn command_fields(&self) -> G18PbCommandFields {
        let a = self.addresses;
        G18PbCommandFields {
            pb_slot: self.policy.slot,
            pm_1d4: self.policy.pm_1d4,
            pb_state: a.state_high,
            pm_scene: a.graph_high + (SCENES + self.policy.scene as usize * SCENE_BYTES) as u64,
            pm_b8: a.graph_high + (SCENES + self.policy.scene_count as usize * SCENE_BYTES) as u64,
        }
    }

    pub(crate) fn audit_job_mappings(&self, uat: &mmu::Uat, vm: &mmu::Vm, bias: u64) -> Result<usize> {
        let a = self.addresses;
        let mut pages = 0;
        for (address, bytes) in [(a.page_list_low, self.inputs.page_list_bytes as usize),
            (a.metrics_low, METRICS_BYTES), (a.scene_scratch_low, SCENE_SCRATCH_BYTES),
            (a.discard_low, DISCARD_BYTES), (a.blocks_low, self.policy.blocks as usize * G18_PB_BLOCK_BYTES as usize)] {
            let low = vm.translate_pages(address, bytes)?;
            if vm.translate_pages(bias + address, bytes)? != low { return Err(EFAULT); }
            pages += low.len();
        }
        for object in [&self.state, &self.graph, &self.page_list, &self.metrics] {
            uat.kernel_vm().translate_pages(object.iova(), object.logical_size)?;
        }
        Ok(pages)
    }

    pub(crate) fn print_h1_digests(&mut self, dev: &AsahiDrmDevice, row: &[u8;16]) -> Result {
        // Context +8 is firmware-owned and separately checked. Normalize only
        // that field and the unwritten +1c word for host reproducibility.
        self.state.with_bytes_mut(|out| {
            let mut bytes = [0u8; G18_PB_STATE_BYTES];
            bytes.copy_from_slice(&out[..G18_PB_STATE_BYTES]);
            bytes[8..12].fill(0); bytes[0x1c..0x20].fill(0);
            dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=pb-state fnv={:#018x}\n", crate::g18::fw::graphics_payload::fnv64(&bytes));
            Ok(())
        })?;
        self.page_list.with_bytes_mut(|bytes| {
            dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=pb-page-list fnv={:#018x}\n", crate::g18::fw::graphics_payload::fnv64(bytes)); Ok(())
        })?;
        dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=pb-row fnv={:#018x}\n", crate::g18::fw::graphics_payload::fnv64(row));
        Ok(())
    }

    /// tag15 PB tuple for the TA queue; `manager` is the linked 3D queue index.
    pub(crate) fn binding(&self, manager: u32) -> G18ConfigParameterBuffer {
        G18ConfigParameterBuffer {
            state: self.addresses.state_high, generation: self.policy.generation,
            slot: self.policy.slot, manager,
        }
    }

    /// Register inputs owned by the PB (G17pRenderParameters.cycle and the
    /// metrics record for register 0x1c910), for render-regs.
    pub(crate) fn scene_cycle(&self) -> u64 { self.scene_cycle_at(self.policy.scene) }

    pub(crate) fn metrics_record(&self) -> u64 {
        self.addresses.metrics_low + u64::from(self.policy.scene) * METRICS_STRIDE
    }

    /// Firmware PB slot table entry (b000 0x71420 + slot * 0x50) expected
    /// after the TA pool bind: state, generation, context, queue bit, manager bit.
    pub(crate) fn expected_firmware_slot(&self, context: u32, ta_qid: u8, manager: u32)
        -> [u64; 6] {
        [self.addresses.state_high, self.policy.generation, u64::from(context),
         1u64 << ta_qid, 0, 1u64 << manager]
    }
}
