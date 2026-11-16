// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Queue records and the commands of a queue's item ring.
//!
//! A hardware queue is described by a [`QueueRecord`]. Its [`PointerBlock`]
//! holds the cursors of the queue's item ring, an array of [`ITEM_RING_ENTRIES`]
//! work pointers: 64-bit GPU addresses of commands. Every command starts with
//! a 32-bit [`CommandTag`]: work descriptors carry the tiling, fragment and
//! compute tags (see `render.rs` and `compute.rs`), and the per-kick records
//! defined here carry the others.
//!
//! One kick appends, in order, the work descriptor, a [`QueueConfig`], a
//! [`KickAnnounce`] record and, when a compute queue is first installed or
//! explicitly waits on another queue, a [`KickPredecessor`].
//! Compute kicks always carry the [`QueueConfig`]. A render kick carries it
//! only when it installs the queue, updates the context or QoS identity,
//! carries a completion seed, or configures the queue differently from the
//! last [`QueueConfig`] published on it.

use core::sync::atomic::{AtomicU32, Ordering};
use kernel::prelude::*;

use super::variant::PointerLayout;

use super::kick::{KickTimestamp, QID_MAX};

/// Firmware scheduling profile of a queue.
///
/// Every record that names a queue repeats its priority class: the queue
/// record (twice), [`QueueConfig`], [`KickAnnounce`], the kick entry header,
/// the work ring the queue is published on (`channels.rs`), the doorbell and
/// the render registration token. The queue record and [`QueueConfig`] also
/// carry the scheduling policy word; the QoS row and share word carry the QoS
/// class and share. All of them must agree for one queue.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct Policy {
    priority: u8,
    policy: u16,
    qos_class: u8,
    qos_share: u32,
}

impl Policy {
    /// Number of firmware priority classes.
    pub(crate) const PRIORITIES: u8 = 4;

    /// Returns the profile with priority class `priority`, policy word
    /// `policy`, QoS class `qos_class` and QoS share `qos_share`.
    pub(crate) const fn new(priority: u8, policy: u16, qos_class: u8, qos_share: u32) -> Self {
        assert!(priority < Self::PRIORITIES);
        Self {
            priority,
            policy,
            qos_class,
            qos_share,
        }
    }

    /// Priority class, below [`Self::PRIORITIES`].
    pub(crate) const fn priority(&self) -> u8 {
        self.priority
    }

    /// Scheduling policy word.
    pub(crate) const fn policy(&self) -> u16 {
        self.policy
    }

    /// QoS class byte of the queue's QoS row.
    pub(crate) const fn qos_class(&self) -> u8 {
        self.qos_class
    }

    /// QoS share word of the owner's QoS slot.
    pub(crate) const fn qos_share(&self) -> u32 {
        self.qos_share
    }
}

/// Number of work pointers in a queue's item ring.
pub(crate) const ITEM_RING_ENTRIES: u32 = 0x500;

/// Tag in the first word of every command.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum CommandTag {
    /// Tiling (TA) work descriptor.
    Tiling = 0,
    /// Fragment (3D) work descriptor.
    Fragment = 1,
    /// Compute work descriptor.
    Compute = 3,
    /// [`KickAnnounce`].
    KickAnnounce = 0x0e,
    /// [`QueueConfig`].
    QueueConfig = 0x0f,
    /// [`KickPredecessor`].
    KickPredecessor = 0x10,
    /// [`FragmentBarrier`] (25G83 firmware).
    FragmentBarrier = 4,
    /// [`InitBufferManager`] (25G83 firmware).
    InitBufferManager = 5,
}

/// The engine that executes a queue's work.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum DataMaster {
    /// Tiling (TA).
    Tiling = 0,
    /// Fragment (3D).
    Fragment = 1,
    /// Compute (CL).
    Compute = 2,
}

/// Cursors of a queue's item ring.
///
/// The firmware writes `done` and `unk_30`. The host writes the block on
/// three paths, never with a structure copy once the queue was published:
///
/// - creation: the whole block, [`PointerBlock::new`], before the queue is
///   first published;
/// - publication: the producer, with one aligned 32-bit store per publication
///   step, after a barrier that orders the work pointers and commands before
///   it;
/// - reuse of a render queue for a new owner: [`PointerBlock::rebase`].
///
/// Where the producer and the recovery target live depends on the firmware
/// build ([`PointerLayout`]).
#[repr(C)]
#[derive(Debug)]
pub(crate) struct PointerBlock {
    /// Item-ring position up to which the firmware has finished commands.
    /// Free ring space is measured from it, so it bounds the producer.
    pub(crate) done: AtomicU32,
    pub(crate) unk_04: [u32; 11],
    /// Written by the firmware. The host treats the queue as idle only when
    /// this cursor, `done` and the producer are equal.
    pub(crate) unk_30: AtomicU32,
    pub(crate) unk_34: [u32; 3],
    /// 26A428: item-ring position after the last work pointer the host published.
    word_40: AtomicU32,
    pub(crate) unk_44: [u32; 3],
    /// 26A428: on recovery, the host copies the producer here before acknowledging resume.
    word_50: AtomicU32,
    pub(crate) unk_54: [u32; 3],
    /// Number of entries in the item ring.
    pub(crate) entries: u32,
    pub(crate) unk_64: [u32; 7],
}

static_assert!(size_of::<PointerBlock>() == 0x80);
static_assert!(core::mem::offset_of!(PointerBlock, unk_30) == 0x30);
static_assert!(core::mem::offset_of!(PointerBlock, word_40) == 0x40);
static_assert!(core::mem::offset_of!(PointerBlock, word_50) == 0x50);
static_assert!(core::mem::offset_of!(PointerBlock, entries) == 0x60);

// SAFETY: PointerBlock contains only integers and integer atomics.
unsafe impl Zeroable for PointerBlock {}

impl PointerBlock {
    /// Offsets of the two host words a [`PointerLayout`] may name.
    const WORD_40: usize = core::mem::offset_of!(Self, word_40);
    const WORD_50: usize = core::mem::offset_of!(Self, word_50);

    /// Whether `offset` is one of the host words a [`PointerLayout`] may name.
    const fn is_host_word(offset: usize) -> bool {
        offset == Self::WORD_40 || offset == Self::WORD_50
    }

    /// Whether `layout` names host words of the block, and two different ones when it has a
    /// recovery target. Every firmware variant's layout is checked at build time.
    pub(crate) const fn valid_layout(layout: &PointerLayout) -> bool {
        match layout.recovery_target {
            None => Self::is_host_word(layout.producer),
            Some(target) => {
                Self::is_host_word(layout.producer)
                    && Self::is_host_word(target)
                    && target != layout.producer
            }
        }
    }

    /// Returns the cursors of an empty item ring.
    pub(crate) fn new(layout: &PointerLayout) -> Result<Self> {
        let block = Self {
            done: AtomicU32::new(0),
            unk_04: [0; 11],
            unk_30: AtomicU32::new(0),
            unk_34: [0; 3],
            word_40: AtomicU32::new(0),
            unk_44: [0; 3],
            word_50: AtomicU32::new(0),
            unk_54: [0; 3],
            entries: ITEM_RING_ENTRIES,
            unk_64: [0; 7],
        };
        if let Some(target) = layout.recovery_target {
            block.word(target)?.store(u32::MAX, Ordering::Relaxed);
        }
        Ok(block)
    }

    /// The host word at `offset`, which must be one a [`PointerLayout`] may name.
    fn word(&self, offset: usize) -> Result<&AtomicU32> {
        match offset {
            Self::WORD_40 => Ok(&self.word_40),
            Self::WORD_50 => Ok(&self.word_50),
            _ => Err(EINVAL),
        }
    }

    /// Moves the cursors of an empty queue to item-ring position `base`.
    ///
    /// Used when a render queue pair that was kicked before is handed to a new
    /// owner, with `base` the position the host tracks for the new owner's
    /// queues. Only valid while the firmware does not run the queue, with all
    /// three cursors zero. Writes `done`, `unk_30` and the producer, in this
    /// order, with single stores; the caller issues a full barrier afterwards.
    pub(crate) fn rebase(&self, layout: &PointerLayout, base: u32) -> Result {
        let producer = self.word(layout.producer)?;
        self.done.store(base, Ordering::Relaxed);
        self.unk_30.store(base, Ordering::Relaxed);
        producer.store(base, Ordering::Relaxed);
        Ok(())
    }
}

/// Head of a queue's job list.
///
/// Live state: the firmware links and unlinks jobs; the host writes the head
/// only at queue creation.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct JobListHead {
    /// GPU address of the first job, or zero.
    pub(crate) first_va: u64,
    /// GPU address of the last link: the head itself when the list is empty.
    pub(crate) last_va: u64,
}

impl JobListHead {
    /// Returns an empty list whose head is at `self_va`.
    pub(crate) const fn new(self_va: u64) -> Self {
        Self {
            first_va: 0,
            last_va: self_va,
        }
    }
}

/// Addresses and owner a [`QueueRecord`] names.
pub(crate) struct QueueRecordArgs {
    /// GPU address of the queue's [`PointerBlock`].
    pub(crate) pointers_va: u64,
    /// GPU address of the queue's item ring.
    pub(crate) items_va: u64,
    /// GPU address of the job list the queue's work is linked into.
    pub(crate) job_list_va: u64,
    /// GPU address of the owning logical queue's scheduler-state page.
    pub(crate) scheduler_va: u64,
    /// Thread-group ID of the owning process (0 while unowned).
    pub(crate) owner_pid: u32,
    /// Scheduling profile of the owning logical queue.
    pub(crate) policy: Policy,
}

/// Description of one hardware queue, named by its work-ring slots.
///
/// The host writes it whole at queue creation and changes the owner fields of
/// an idle queue with [`QueueRecord::set_compute_owner`] or, for render
/// queues, field by field (`render/memory.rs`).
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct QueueRecord {
    /// GPU address of the [`PointerBlock`].
    pub(crate) pointers_va: u64,
    /// GPU address of the item ring.
    pub(crate) items_va: u64,
    /// GPU address of the job list.
    pub(crate) job_list_va: u64,
    pub(crate) unk_18: [u8; 0xc],
    pub(crate) unk_24: u32,
    /// Queue priority, stored twice.
    pub(crate) priority: [u32; 2],
    pub(crate) unk_30: [u8; 6],
    pub(crate) unk_36: u16,
    /// Zero at creation; cleared again when a compute queue changes owner.
    pub(crate) unk_38: u32,
    pub(crate) unk_3c: u32,
    /// Scheduling policy.
    pub(crate) policy: u32,
    pub(crate) unk_44: u32,
    /// Thread-group ID of the owning process.
    pub(crate) owner_pid: u32,
    pub(crate) unk_4c: [u8; 0x50],
    /// GPU address of the owner's scheduler-state page.
    pub(crate) scheduler_va: u64,
    pub(crate) unk_a4: [u8; 0x1c],
}

static_assert!(size_of::<QueueRecord>() == 0xc0);
static_assert!(core::mem::offset_of!(QueueRecord, priority) == 0x28);
static_assert!(core::mem::offset_of!(QueueRecord, unk_38) == 0x38);
static_assert!(core::mem::offset_of!(QueueRecord, policy) == 0x40);
static_assert!(core::mem::offset_of!(QueueRecord, scheduler_va) == 0x9c);

impl QueueRecord {
    /// Builds the record of a newly created queue.
    pub(crate) fn new(args: &QueueRecordArgs) -> Self {
        Self {
            pointers_va: args.pointers_va,
            items_va: args.items_va,
            job_list_va: args.job_list_va,
            unk_18: [0; 0xc],
            unk_24: u32::MAX,
            priority: [u32::from(args.policy.priority()); 2],
            unk_30: [0; 6],
            unk_36: u16::MAX,
            unk_38: 0,
            unk_3c: 0,
            policy: u32::from(args.policy.policy()),
            unk_44: u32::MAX,
            owner_pid: args.owner_pid,
            unk_4c: [0; 0x50],
            scheduler_va: args.scheduler_va,
            unk_a4: [0; 0x1c],
        }
    }

    /// Hands an idle compute queue to a new owner.
    ///
    /// Replaces the job head, process identity and scheduler page, and clears `unk_38`.
    /// The priority and policy words are written only when `policy` differs from the
    /// profile the record holds.
    pub(crate) fn set_compute_owner(
        &mut self,
        job_list_va: u64,
        scheduler_va: u64,
        owner_pid: u32,
        policy: Policy,
    ) {
        self.job_list_va = job_list_va;
        self.unk_38 = 0;
        self.owner_pid = owner_pid;
        self.scheduler_va = scheduler_va;
        self.set_policy(policy);
    }

    /// Replaces the priority and policy words of an idle queue's record when they differ
    /// from `policy`.
    pub(crate) fn set_policy(&mut self, policy: Policy) {
        let priority = [u32::from(policy.priority()); 2];
        if { self.priority } != priority {
            self.priority = priority;
        }
        if { self.policy } != u32::from(policy.policy()) {
            self.policy = u32::from(policy.policy());
        }
    }
}

/// Free list a queue's USC private memory is allocated from.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct FreeListBinding {
    /// GPU address of the free list's shared control object.
    pub(crate) va: u64,
    /// Generation of the free list's registration.
    pub(crate) generation: u64,
    /// Buffer slot of the free list.
    pub(crate) slot: u32,
}

/// Parameter-buffer binding of a tiling queue.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct TilingBinding {
    /// GPU address of the parameter-buffer state object.
    pub(crate) pb_va: u64,
    /// Allocation token of the parameter buffer's buffer slot.
    pub(crate) pb_token: u64,
    /// Buffer slot of the parameter buffer.
    pub(crate) pb_slot: u32,
    /// QID of the fragment queue paired with this tiling queue.
    pub(crate) fragment_qid: u8,
}

/// Queue whose configuration a [`QueueConfig`] carries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum QueueConfigTarget {
    /// A tiling queue and its parameter-buffer binding.
    Tiling(TilingBinding),
    /// A fragment queue.
    Fragment,
    /// A compute queue.
    Compute,
}

/// Inputs of one [`QueueConfig`].
pub(crate) struct QueueConfigArgs {
    /// Kind of queue.
    pub(crate) target: QueueConfigTarget,
    /// GPU address of the queue's kick ring.
    pub(crate) kick_ring_va: u64,
    /// Firmware alias of the queue's kick ring.
    pub(crate) kick_ring_fw_va: u64,
    /// Configured queue.
    pub(crate) qid: u8,
    /// The firmware has not yet installed this QID.
    pub(crate) install: bool,
    /// Completion value the firmware loads for the queue, or `None` to keep
    /// the value it holds.
    pub(crate) completion_seed: Option<u64>,
    /// UAT context the queue's work runs in.
    pub(crate) context_id: u16,
    /// USC private-memory free list.
    pub(crate) free_list: FreeListBinding,
    /// GPU address of the owner's scheduler-state page.
    pub(crate) scheduler_va: u64,
    /// Scheduling profile of the owning logical queue.
    pub(crate) policy: Policy,
    /// QoS buffer slot of the logical queue.
    pub(crate) qos_slot: u8,
    /// The QoS identity or queue membership changed since the last update.
    pub(crate) qos_update: bool,
    /// Thread-group ID of the owning process.
    pub(crate) owner_pid: u32,
    /// The queue's UAT context changed since the last update.
    pub(crate) context_update: bool,
}

/// Configuration of one hardware queue (tag 15).
///
/// The firmware applies it when it consumes the command, which may be after
/// later kicks were published, so a record must not be rewritten while a
/// work pointer names it.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct QueueConfig {
    /// [`CommandTag::QueueConfig`].
    pub(crate) tag: u32,
    pub(crate) unk_04: u32,
    /// GPU address of the queue's kick ring.
    pub(crate) kick_ring_va: u64,
    /// Firmware alias of the queue's kick ring.
    pub(crate) kick_ring_fw_va: u64,
    /// Configured queue.
    pub(crate) qid: u16,
    /// Non-zero to install the QID.
    pub(crate) install: u16,
    pub(crate) unk_1c: u16,
    /// Queue priority.
    pub(crate) priority: u16,
    pub(crate) unk_20: u16,
    /// [`DataMaster`] of the queue.
    pub(crate) data_master: u16,
    pub(crate) unk_24: u16,
    /// Non-zero when `completion_seed` is valid.
    pub(crate) completion_seed_valid: u32,
    /// Completion value: kick timestamp in bits 39:0, render-pass sequence in
    /// bits 47:40 (see [`KickTimestamp::completion_value`]).
    pub(crate) completion_seed: u64,
    /// UAT context of the queue's work.
    pub(crate) context_id: u16,
    pub(crate) unk_34: u16,
    /// GPU address of the USC free list's shared control object.
    pub(crate) free_list_va: u64,
    /// Generation of the USC free list.
    pub(crate) free_list_generation: u64,
    /// Buffer slot of the USC free list.
    pub(crate) free_list_slot: u32,
    /// GPU address of the owner's scheduler-state page.
    pub(crate) scheduler_va: u64,
    /// Non-zero when the QoS identity or membership changed.
    pub(crate) qos_update: u32,
    /// QoS buffer slot.
    pub(crate) qos_slot: u16,
    pub(crate) unk_58: u16,
    /// Thread-group ID of the owning process.
    pub(crate) owner_pid: u32,
    /// Scheduling policy.
    pub(crate) policy: u16,
    pub(crate) unk_60: u16,
    /// Non-zero when the UAT context changed.
    pub(crate) context_update: u16,
    pub(crate) unk_64: u16,
    pub(crate) unk_66: u16,
    pub(crate) unk_68: [u8; 6],
    /// Tiling queues: GPU address of the parameter-buffer state; else zero.
    pub(crate) pb_va: u64,
    /// Tiling queues: parameter-buffer slot allocation token; else all ones.
    pub(crate) pb_token: u64,
    /// Tiling queues: parameter-buffer slot; else all ones.
    pub(crate) pb_slot: u32,
    /// Tiling queues: QID of the paired fragment queue; else all ones.
    pub(crate) fragment_qid: u32,
    pub(crate) unk_86: [u8; 0x3a],
}

static_assert!(size_of::<QueueConfig>() == 0xc0);
static_assert!(core::mem::offset_of!(QueueConfig, completion_seed_valid) == 0x26);
static_assert!(core::mem::offset_of!(QueueConfig, free_list_va) == 0x36);
static_assert!(core::mem::offset_of!(QueueConfig, scheduler_va) == 0x4a);
static_assert!(core::mem::offset_of!(QueueConfig, qos_slot) == 0x56);
static_assert!(core::mem::offset_of!(QueueConfig, owner_pid) == 0x5a);
static_assert!(core::mem::offset_of!(QueueConfig, pb_va) == 0x6e);
static_assert!(core::mem::offset_of!(QueueConfig, fragment_qid) == 0x82);

impl QueueConfig {
    const UNK_66: u16 = 1;

    /// Builds the record described by `args`.
    pub(crate) fn new(args: &QueueConfigArgs) -> Result<Self> {
        if args.qid > QID_MAX {
            return Err(EINVAL);
        }
        let (data_master, pb_va, pb_token, pb_slot, fragment_qid) = match args.target {
            QueueConfigTarget::Tiling(binding) => {
                if binding.fragment_qid > QID_MAX {
                    return Err(EINVAL);
                }
                (
                    DataMaster::Tiling,
                    binding.pb_va,
                    binding.pb_token,
                    binding.pb_slot,
                    u32::from(binding.fragment_qid),
                )
            }
            QueueConfigTarget::Fragment => (DataMaster::Fragment, 0, u64::MAX, u32::MAX, u32::MAX),
            QueueConfigTarget::Compute => (DataMaster::Compute, 0, u64::MAX, u32::MAX, u32::MAX),
        };
        Ok(Self {
            tag: CommandTag::QueueConfig as u32,
            unk_04: 0,
            kick_ring_va: args.kick_ring_va,
            kick_ring_fw_va: args.kick_ring_fw_va,
            qid: u16::from(args.qid),
            install: u16::from(args.install),
            unk_1c: 0,
            priority: u16::from(args.policy.priority()),
            unk_20: 0,
            data_master: data_master as u16,
            unk_24: 0,
            completion_seed_valid: u32::from(args.completion_seed.is_some()),
            completion_seed: args.completion_seed.unwrap_or(0),
            context_id: args.context_id,
            unk_34: 0,
            free_list_va: args.free_list.va,
            free_list_generation: args.free_list.generation,
            free_list_slot: args.free_list.slot,
            scheduler_va: args.scheduler_va,
            qos_update: u32::from(args.qos_update),
            qos_slot: u16::from(args.qos_slot),
            unk_58: 0,
            owner_pid: args.owner_pid,
            policy: args.policy.policy(),
            unk_60: 0,
            context_update: u16::from(args.context_update),
            unk_64: 0,
            unk_66: Self::UNK_66,
            unk_68: [0; 6],
            pb_va,
            pb_token,
            pb_slot,
            fragment_qid,
            unk_86: [0; 0x3a],
        })
    }
}

/// The 25G83 firmware's barrier before every fragment descriptor (tag 4): the fragment work
/// waits for the paired tiling stamp, in addition to the kick-entry dependency.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct FragmentBarrier {
    /// [`CommandTag::FragmentBarrier`].
    pub(crate) tag: u32,
    /// GPU address of the tiling queue's completion stamp word.
    pub(crate) stamp_va: u64,
    /// Timestamp of the tiling kick.
    pub(crate) tiling_kick: u64,
    /// QID of the tiling queue.
    pub(crate) tiling_qid: u32,
    /// Tiling stamp value waited for.
    pub(crate) wait: u32,
    pub(crate) unk_1c: [u8; 8],
    /// Event of the wait: the tiling QID.
    pub(crate) event: u32,
    /// Fragment stamp value.
    pub(crate) stamp_self: u32,
    /// Current object ID of the fragment stage.
    pub(crate) fragment_id: u64,
    pub(crate) unk_34: u32,
    pub(crate) unk_38: [u8; 0x10],
}

static_assert!(size_of::<FragmentBarrier>() == 0x48);
static_assert!(core::mem::offset_of!(FragmentBarrier, wait) == 0x18);
static_assert!(core::mem::offset_of!(FragmentBarrier, event) == 0x24);
static_assert!(core::mem::offset_of!(FragmentBarrier, fragment_id) == 0x2c);

impl FragmentBarrier {
    /// The barrier of a pass whose tiling stage, kicked at `tiling_kick` on `tiling_qid`,
    /// signals `stamp_value` at `stamp_va`.
    pub(crate) fn new(
        stamp_va: u64,
        tiling_kick: KickTimestamp,
        tiling_qid: u32,
        stamp_value: u32,
        fragment_id: u64,
    ) -> Self {
        Self {
            tag: CommandTag::FragmentBarrier as u32,
            stamp_va,
            tiling_kick: tiling_kick.get(),
            tiling_qid,
            wait: stamp_value,
            unk_1c: [0; 8],
            event: tiling_qid,
            stamp_self: stamp_value,
            fragment_id,
            unk_34: 1,
            unk_38: [0; 0x10],
        }
    }
}

/// The 25G83 firmware's buffer-manager initialization (tag 5), before the first tiling
/// descriptor that uses a parameter buffer. The block count is read from the manager.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct InitBufferManager {
    /// [`CommandTag::InitBufferManager`].
    pub(crate) tag: u32,
    /// UAT context of the pass.
    pub(crate) context_id: u32,
    /// Buffer slot of the parameter buffer.
    pub(crate) pb_slot: u32,
    /// GPU address of the parameter-buffer state object.
    pub(crate) pb_va: u64,
    /// Tiling stamp value of the pass.
    pub(crate) stamp_value: u32,
}

static_assert!(size_of::<InitBufferManager>() == 0x18);

impl InitBufferManager {
    pub(crate) fn new(context_id: u32, pb_slot: u32, pb_va: u64, stamp_value: u32) -> Self {
        Self {
            tag: CommandTag::InitBufferManager as u32,
            context_id,
            pb_slot,
            pb_va,
            stamp_value,
        }
    }
}

/// Announcement of a newly written kick entry of a queue (tag 14).
///
/// A record slot larger than this structure must be zero beyond it.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct KickAnnounce {
    /// [`CommandTag::KickAnnounce`].
    pub(crate) tag: u32,
    /// Kicked queue.
    pub(crate) qid: u16,
    /// Number of entries announced.
    pub(crate) count: u16,
    /// Queue priority.
    pub(crate) priority: u8,
    /// Timestamp of the announced entry.
    pub(crate) timestamp: u64,
    /// [`DataMaster`] of the queue.
    pub(crate) data_master: u32,
    pub(crate) unk_15: [u8; 3],
}

static_assert!(size_of::<KickAnnounce>() == 0x18);
static_assert!(core::mem::offset_of!(KickAnnounce, timestamp) == 0x09);
static_assert!(core::mem::offset_of!(KickAnnounce, data_master) == 0x11);

impl KickAnnounce {
    /// Announces the entry of queue `qid` at priority class `priority`,
    /// written at `timestamp`.
    pub(crate) fn new(
        qid: u8,
        data_master: DataMaster,
        timestamp: KickTimestamp,
        priority: u8,
    ) -> Result<Self> {
        if qid > QID_MAX || priority >= Policy::PRIORITIES {
            return Err(EINVAL);
        }
        Ok(Self {
            tag: CommandTag::KickAnnounce as u32,
            qid: u16::from(qid),
            count: 1,
            priority,
            timestamp: timestamp.get(),
            data_master: data_master as u32,
            unk_15: [0; 3],
        })
    }
}

/// Names the kick of a compute queue that precedes the announced one
/// (tag 16).
///
/// A record slot larger than this structure must be zero beyond it.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct KickPredecessor {
    /// [`CommandTag::KickPredecessor`].
    pub(crate) tag: u32,
    /// Kicked queue.
    pub(crate) qid: u16,
    /// Timestamp of the queue's kick preceding the announced one.
    pub(crate) previous: u64,
    pub(crate) unk_0e: u32,
}

static_assert!(size_of::<KickPredecessor>() == 0x12);

impl KickPredecessor {
    const UNK_0E: u32 = 0xfe;

    /// Names `previous` as the kick of queue `qid` preceding the announced
    /// one.
    pub(crate) fn new(qid: u8, previous: KickTimestamp) -> Result<Self> {
        if qid > QID_MAX {
            return Err(EINVAL);
        }
        Ok(Self {
            tag: CommandTag::KickPredecessor as u32,
            qid: u16::from(qid),
            previous: previous.get(),
            unk_0e: Self::UNK_0E,
        })
    }
}
