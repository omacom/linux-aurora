// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device-control records and work-ring slots.
//!
//! The host sends out-of-band requests and replies through the device-control
//! ring, whose 0x40-byte records start with a [`DeviceControlOp`]. It hands
//! hardware queues to the firmware through work rings, one per priority and
//! engine, each holding [`WORK_RING_SLOTS`] [`WorkSlot`]s.
//!
//! Producers and consumers of both rings are live words in the primary state
//! grid; they are not part of these structures. Before publishing a
//! device-control record whose opcode [`DeviceControlOp::asserts_gpu_power`],
//! the host also increments its power-assert tally.

use kernel::prelude::*;

use super::kick::QID_MAX;
use super::queue::DataMaster;

/// Size of a device-control record.
pub(crate) const DEVICE_CONTROL_RECORD_SIZE: usize = 0x40;
/// Number of records in the device-control ring.
pub(crate) const DEVICE_CONTROL_SLOTS: u32 = 256;

/// Entries in either coprocessor's event ring.
pub(crate) const EVENT_SLOTS: u32 = 256;
/// Entries in either coprocessor's trace ring.
pub(crate) const TRACE_SLOTS: u32 = 512;
/// Entries in either coprocessor's statistics ring.
pub(crate) const STATISTICS_SLOTS: u32 = 256;
/// Words in an event, trace or statistics entry.
pub(crate) const EVENT_WORDS: usize = 18;
/// Byte stride of an event, trace or statistics entry.
pub(crate) const EVENT_RECORD_SIZE: usize = EVENT_WORDS * size_of::<u32>();
/// Producer offset within a firmware-produced ring's state block.
pub(crate) const RX_PRODUCER_OFFSET: usize = 0x20;
/// Trace records use type 5.
pub(crate) const TRACE_TYPES: u32 = 1 << 5;
/// Statistics record types accepted by the host.
pub(crate) const STATISTICS_TYPES: u32 = 0x5fff_0000;

/// Completion stamp indices signalled by the firmware.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct StampEventRecord {
    tag: u32,
    masks: [u64; 2],
    diagnostic_pairs: u16,
    unk_16: [u8; 0x32],
}

/// Restart generation and the affected stamp, or -1 without attribution.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct RestartEventRecord {
    tag: u32,
    generation: u64,
    stamp: i32,
    unk_10: [u8; 0x38],
}

/// Parameter-buffer growth request. The reply moves `halt_count` to +0x18.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct TvbGrowEventRecord {
    tag: u32,
    vm_slot: i32,
    buffer_slot: u32,
    counter: u32,
    subpipe: u32,
    halt_count: u64,
    unk_1c: [u8; 0x2c],
}

/// Completed asynchronous free-list growth.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct FreeListEventRecord {
    tag: u32,
    stamp: i32,
    buffer_slot: u32,
    control_va: u64,
    ring_va: u64,
    cookie: u64,
    result: u32,
    unk_28: [u8; 0x20],
}

/// A snapshot of one firmware event. Every variant occupies a complete slot;
/// callers copy the entry before publishing its consumer cursor.
#[repr(C)]
#[derive(Copy, Clone)]
pub(crate) union EventRecord {
    words: [u32; EVENT_WORDS],
    stamp: StampEventRecord,
    restart: RestartEventRecord,
    tvb: TvbGrowEventRecord,
    free_list: FreeListEventRecord,
}

static_assert!(size_of::<StampEventRecord>() == EVENT_RECORD_SIZE);
static_assert!(size_of::<RestartEventRecord>() == EVENT_RECORD_SIZE);
static_assert!(size_of::<TvbGrowEventRecord>() == EVENT_RECORD_SIZE);
static_assert!(size_of::<FreeListEventRecord>() == EVENT_RECORD_SIZE);
static_assert!(size_of::<EventRecord>() == EVENT_RECORD_SIZE);
static_assert!(core::mem::offset_of!(StampEventRecord, masks) == 4);
static_assert!(core::mem::offset_of!(TvbGrowEventRecord, halt_count) == 0x14);
static_assert!(core::mem::offset_of!(FreeListEventRecord, cookie) == 0x1c);

impl core::fmt::Debug for EventRecord {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EventRecord")
            .field("tag", &self.tag())
            .finish()
    }
}

/// A decoded asynchronous free-list growth completion. Resource ownership and
/// the expected cookie/result are checked by the free-list owner.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct FreeListCompletion {
    pub(crate) stamp: i32,
    pub(crate) buffer_slot: u32,
    pub(crate) control_va: u64,
    pub(crate) ring_va: u64,
    pub(crate) cookie: u64,
    pub(crate) result: u32,
}

/// Host actions requested by a firmware event. Unknown tags are consumed so
/// one unrecognized record cannot block later completions.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    Stamp([u64; 2]),
    Restart { generation: u64, stamp: i32 },
    TvbGrow(TvbGrowRequest),
    FreeListGrow(FreeListCompletion),
    ContextKilled(u64),
    /// The firmware failed a compute command of queue `qid` and stores no completion stamp for
    /// it: `stamp` is the value that command would have stored.
    ComputeFault { qid: u8, stamp: u32 },
    Unknown(u32),
}

impl EventRecord {
    const STAMP: u32 = 1;
    const RESTART: u32 = 4;
    const TVB_GROW: u32 = 6;
    const FREE_LIST_GROW: u32 = 13;
    const CONTEXT_KILLED: u32 = 12;
    const COMPUTE_FAULT: u32 = 7;
    const COMPUTE_QIDS: u32 = 128;
    const DIAGNOSTIC_PAIRS_MAX: u16 = 5;
    const TVB_SLOTS: u32 = 127;
    const SUBPIPES: u32 = 64;
    const FREE_LIST_SLOTS: u32 = 256;

    pub(crate) const fn from_words(words: [u32; EVENT_WORDS]) -> Self {
        Self { words }
    }

    pub(crate) fn tag(&self) -> u32 {
        // SAFETY: All constructors initialize the entire union. Every bit
        // pattern is valid for its u32 words, and word zero is the tag.
        unsafe { self.words[0] }
    }

    /// Validates the record's fields without interpreting resource ownership.
    /// A malformed known event is an error, unlike an unknown tag.
    pub(crate) fn decode(&self) -> Result<Event> {
        match self.tag() {
            Self::STAMP => {
                // SAFETY: The entire slot is initialized, the tag selects
                // this layout, and its integer fields have no invalid values.
                let record = unsafe { self.stamp };
                if record.diagnostic_pairs > Self::DIAGNOSTIC_PAIRS_MAX {
                    return Err(EIO);
                }
                Ok(Event::Stamp(record.masks))
            }
            Self::RESTART => {
                // SAFETY: As above, for the restart layout.
                let record = unsafe { self.restart };
                Ok(Event::Restart {
                    generation: record.generation,
                    stamp: record.stamp,
                })
            }
            Self::TVB_GROW => {
                // SAFETY: As above, for the parameter-buffer growth layout.
                let record = unsafe { self.tvb };
                if record.vm_slot < -1
                    || record.buffer_slot >= Self::TVB_SLOTS
                    || record.subpipe >= Self::SUBPIPES
                {
                    return Err(EIO);
                }
                Ok(Event::TvbGrow(TvbGrowRequest {
                    buffer_slot: record.buffer_slot,
                    vm_slot: record.vm_slot,
                    counter: record.counter,
                    subpipe: record.subpipe,
                    halt_count: record.halt_count,
                }))
            }
            Self::COMPUTE_FAULT => {
                // SAFETY: Every constructor initializes all integer words.
                let words = unsafe { self.words };
                // Only compute faults are known to use this type: any other layout stays an
                // unknown event, consumed and ignored as before.
                if words[3] >= Self::COMPUTE_QIDS || words[4] & 0xff != 0 {
                    return Ok(Event::Unknown(Self::COMPUTE_FAULT));
                }
                Ok(Event::ComputeFault {
                    qid: words[3] as u8,
                    stamp: words[4],
                })
            }
            Self::CONTEXT_KILLED => {
                // SAFETY: Every constructor initializes all integer words.
                let words = unsafe { self.words };
                Ok(Event::ContextKilled(
                    u64::from(words[1]) | (u64::from(words[2]) << 32),
                ))
            }
            Self::FREE_LIST_GROW => {
                // SAFETY: As above, for the free-list growth layout.
                let record = unsafe { self.free_list };
                if record.stamp < -1
                    || record.buffer_slot >= Self::FREE_LIST_SLOTS
                    || record.control_va == 0
                    || record.ring_va == 0
                {
                    return Err(EIO);
                }
                Ok(Event::FreeListGrow(FreeListCompletion {
                    stamp: record.stamp,
                    buffer_slot: record.buffer_slot,
                    control_va: record.control_va,
                    ring_va: record.ring_va,
                    cookie: record.cookie,
                    result: record.result,
                }))
            }
            tag => Ok(Event::Unknown(tag)),
        }
    }
}

/// A complete device-control request. Its variant supplies the power policy;
/// callers construct the contained record with its typed builder.
#[derive(Debug)]
pub(crate) enum ControlRecord {
    TvbGrow(TvbGrowReply),
    IdlePowerOff(IdlePowerOff),
    SchedulerStateRelease(SchedulerStateRelease),
    FreeListRelease(FreeListRelease),
    ContextKill(ContextKill),
}

impl ControlRecord {
    pub(crate) const fn opcode(&self) -> DeviceControlOp {
        match self {
            Self::TvbGrow(_) => DeviceControlOp::TvbGrowReply,
            Self::IdlePowerOff(_) => DeviceControlOp::IdlePowerOff,
            Self::SchedulerStateRelease(_) => DeviceControlOp::SchedulerStateRelease,
            Self::FreeListRelease(_) => DeviceControlOp::FreeListRelease,
            Self::ContextKill(_) => DeviceControlOp::ContextKill,
        }
    }

    /// Byte view of the initialized record, excluding the host-only enum tag.
    pub(crate) fn bytes(&self) -> &[u8; DEVICE_CONTROL_RECORD_SIZE] {
        let record = match self {
            Self::TvbGrow(record) => core::ptr::from_ref(record).cast(),
            Self::IdlePowerOff(record) => core::ptr::from_ref(record).cast(),
            Self::SchedulerStateRelease(record) => core::ptr::from_ref(record).cast(),
            Self::FreeListRelease(record) => core::ptr::from_ref(record).cast(),
            Self::ContextKill(record) => core::ptr::from_ref(record).cast(),
        };
        // SAFETY: Each variant has a statically checked size of 0x40 bytes,
        // contains only initialized integer fields with no implicit padding,
        // and is borrowed for the lifetime of `self`. The byte view has align 1.
        unsafe { &*record }
    }
}

/// Device-control opcodes the driver sends.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum DeviceControlOp {
    /// Reply to a parameter-buffer grow request ([`TvbGrowReply`]).
    TvbGrowReply = 0x08,
    /// Set the idle power-off policy ([`IdlePowerOff`]).
    IdlePowerOff = 0x0a,
    /// Release a logical queue's scheduler state ([`SchedulerStateRelease`]).
    SchedulerStateRelease = 0x14,
    /// Release a free list's buffer slot ([`FreeListRelease`]).
    FreeListRelease = 0x2e,
    /// Stop the queues named by a closed logical scheduler.
    ContextKill = 0x29,
}

impl DeviceControlOp {
    /// Tells whether records with this opcode count towards the host
    /// power-assert tally. The firmware compares the tally with the counted
    /// records it has drained and keeps the GPU cores powered while they
    /// differ.
    pub(crate) const fn asserts_gpu_power(self) -> bool {
        !matches!(self, Self::FreeListRelease)
    }
}

/// The fields of a parameter-buffer grow request event that its reply
/// echoes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct TvbGrowRequest {
    /// Buffer slot of the parameter buffer.
    pub(crate) buffer_slot: u32,
    /// VM slot of the request, or -1.
    pub(crate) vm_slot: i32,
    /// Request counter.
    pub(crate) counter: u32,
    /// Sub-pipe of the request.
    pub(crate) subpipe: u32,
    /// Firmware halt count the request was made under.
    pub(crate) halt_count: u64,
}

/// Reply to a parameter-buffer grow request.
#[repr(C)]
#[derive(Debug)]
pub(crate) struct TvbGrowReply {
    /// [`DeviceControlOp::TvbGrowReply`].
    pub(crate) opcode: u32,
    /// Non-zero when the parameter buffer grew.
    pub(crate) grew: u32,
    /// Buffer slot of the request.
    pub(crate) buffer_slot: u32,
    /// VM slot of the request.
    pub(crate) vm_slot: i32,
    /// Request counter.
    pub(crate) counter: u32,
    /// Sub-pipe of the request.
    pub(crate) subpipe: u32,
    /// Halt count of the request.
    pub(crate) halt_count: u64,
    pub(crate) unk_20: [u8; 0x20],
}

static_assert!(size_of::<TvbGrowReply>() == DEVICE_CONTROL_RECORD_SIZE);

impl TvbGrowReply {
    /// Replies to `request`; `grew` tells whether the buffer was grown.
    pub(crate) const fn new(request: &TvbGrowRequest, grew: bool) -> Self {
        Self {
            opcode: DeviceControlOp::TvbGrowReply as u32,
            grew: grew as u32,
            buffer_slot: request.buffer_slot,
            vm_slot: request.vm_slot,
            counter: request.counter,
            subpipe: request.subpipe,
            halt_count: request.halt_count,
            unk_20: [0; 0x20],
        }
    }
}

/// Idle power-off policy argument of [`IdlePowerOff`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum IdlePolicy {
    /// Keep the GPU cores powered and power them up now.
    KeepPowered = 0,
    /// Let the firmware power the GPU cores off when idle.
    AllowPowerOff = 1,
}

/// Sets the firmware's idle power-off policy.
#[repr(C)]
#[derive(Debug)]
pub(crate) struct IdlePowerOff {
    /// [`DeviceControlOp::IdlePowerOff`].
    pub(crate) opcode: u32,
    /// [`IdlePolicy`].
    pub(crate) policy: u32,
    pub(crate) unk_08: [u8; 0x38],
}

static_assert!(size_of::<IdlePowerOff>() == DEVICE_CONTROL_RECORD_SIZE);

impl IdlePowerOff {
    /// Requests `policy`.
    pub(crate) const fn new(policy: IdlePolicy) -> Self {
        Self {
            opcode: DeviceControlOp::IdlePowerOff as u32,
            policy: policy as u32,
            unk_08: [0; 0x38],
        }
    }
}

/// Releases the scheduler state of a logical queue whose work has drained.
///
/// The page may be freed once the firmware has consumed the record.
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct SchedulerStateRelease {
    /// [`DeviceControlOp::SchedulerStateRelease`].
    pub(crate) opcode: u32,
    pub(crate) unk_04: u32,
    /// The scheduler-page bytes at offsets 0x26, 0, 1 and 4, in record order.
    /// Read when the release record is built.
    pub(crate) id: [u8; 4],
    /// GPU address of the scheduler-state page.
    pub(crate) page_va: u64,
    pub(crate) unk_14: [u8; 0x2c],
}

static_assert!(size_of::<SchedulerStateRelease>() == DEVICE_CONTROL_RECORD_SIZE);
static_assert!(core::mem::offset_of!(SchedulerStateRelease, page_va) == 0x0c);

impl SchedulerStateRelease {
    /// Releases the scheduler page with its identity bytes in record order.
    pub(crate) const fn new(page_va: u64, id: [u8; 4]) -> Self {
        Self {
            opcode: DeviceControlOp::SchedulerStateRelease as u32,
            unk_04: 0,
            id,
            page_va,
            unk_14: [0; 0x2c],
        }
    }
}

/// Releases the buffer slot of a free list after its last user.
#[repr(C, packed)]
#[derive(Debug)]
pub(crate) struct FreeListRelease {
    /// [`DeviceControlOp::FreeListRelease`].
    pub(crate) opcode: u32,
    /// Free-list generation of the registration being released.
    pub(crate) generation: u64,
    /// Buffer slot of the free list.
    pub(crate) buffer_slot: u32,
    pub(crate) unk_10: [u8; 0x30],
}

static_assert!(size_of::<FreeListRelease>() == DEVICE_CONTROL_RECORD_SIZE);
static_assert!(core::mem::offset_of!(FreeListRelease, buffer_slot) == 0x0c);

impl FreeListRelease {
    /// Releases `buffer_slot`, registered at `generation`.
    pub(crate) const fn new(generation: u64, buffer_slot: u32) -> Self {
        Self {
            opcode: DeviceControlOp::FreeListRelease as u32,
            generation,
            buffer_slot,
            unk_10: [0; 0x30],
        }
    }
}

/// Number of slots in a work ring.
pub(crate) const WORK_RING_SLOTS: u32 = 256;
/// Number of engines with a work ring per priority.
const WORK_RINGS_PER_PRIORITY: usize = 3;

/// Index in the channel table of the work ring for `data_master` at priority
/// class `priority`, or `None` if `priority` is not a class.
pub(crate) const fn work_ring_index(priority: u8, data_master: DataMaster) -> Option<usize> {
    if priority >= super::queue::Policy::PRIORITIES {
        return None;
    }
    Some(priority as usize * WORK_RINGS_PER_PRIORITY + data_master as usize)
}

/// One work-ring slot: a request to run a queue's newly published commands.
#[repr(C)]
#[derive(Debug)]
pub(crate) struct WorkSlot {
    pub(crate) unk_00: u64,
    /// GPU address of the queue record.
    pub(crate) queue_va: u64,
    /// [`DataMaster`] of the queue.
    pub(crate) data_master: u32,
    /// Item-ring producer after the published commands.
    pub(crate) items_end: u16,
    /// Queue to run.
    pub(crate) qid: u8,
    /// One in the first slot naming this queue on this ring. Each priority
    /// class has its own rings, so a queue whose class changed names the new
    /// ring with a first slot again.
    pub(crate) first: u8,
}

static_assert!(size_of::<WorkSlot>() == 0x18);

impl WorkSlot {
    /// Builds the slot asking the firmware to run queue `qid` up to item-ring
    /// position `items_end`. `first` tells whether this is the first slot
    /// naming the queue on this ring.
    pub(crate) fn new(
        data_master: DataMaster,
        queue_va: u64,
        qid: u8,
        items_end: u16,
        first: bool,
    ) -> Result<Self> {
        if qid > QID_MAX {
            return Err(EINVAL);
        }
        Ok(Self {
            unk_00: 0,
            queue_va,
            data_master: data_master as u32,
            items_end,
            qid,
            first: u8::from(first),
        })
    }
}

/// Stop a closed context's scheduler page and echo its cookie through event 12.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct ContextKill {
    opcode: u32,
    page_va: u64,
    cookie: u64,
    reserved: [u8; 0x2c],
}
static_assert!(size_of::<ContextKill>() == DEVICE_CONTROL_RECORD_SIZE);
static_assert!(core::mem::offset_of!(ContextKill, page_va) == 4);
static_assert!(core::mem::offset_of!(ContextKill, cookie) == 0x0c);
impl ContextKill {
    pub(crate) fn new(page_va: u64, cookie: u64) -> Self {
        Self {
            opcode: DeviceControlOp::ContextKill as u32,
            page_va,
            cookie,
            reserved: [0; 0x2c],
        }
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_event_records))]
mod tests {
    use super::*;

    fn record(words: &[u32]) -> EventRecord {
        let mut all = [0u32; EVENT_WORDS];
        all[..words.len()].copy_from_slice(words);
        EventRecord::from_words(all)
    }

    #[test]
    fn a_compute_fault_names_its_queue_and_stamp() {
        let event = record(&[7, 1, 2, 0x39, 0x300, 0xffff_fc00, 1]).decode();
        assert_eq!(event, Ok(Event::ComputeFault { qid: 0x39, stamp: 0x300 }));
    }

    #[test]
    fn an_unrecognized_type_7_layout_stays_unknown() {
        assert_eq!(record(&[7, 1, 2, 128, 0x300]).decode(), Ok(Event::Unknown(7)));
        assert_eq!(record(&[7, 1, 2, 9, 0x301]).decode(), Ok(Event::Unknown(7)));
    }
}
