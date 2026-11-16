// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Kick-queue entries and the objects they reference.
//!
//! Every hardware queue (QID) owns a kick ring of [`KICK_SLOTS`] entries spaced
//! [`KICK_ENTRY_STRIDE`] bytes apart. A [`KickTimestamp`] names one entry: its
//! low eight bits select the ring slot. To kick, the host writes the complete
//! slot selected by the queue's current timestamp, issues a barrier, advances
//! the timestamp and then announces the entry with a
//! [`super::queue::KickAnnounce`] record.
//!
//! An entry points at a work descriptor and its queue record, lists the kicks
//! it depends on, binds up to four [`RegisterArray`]s embedded in the
//! descriptor and may name a table of [`McacheRange`]s.
//!
//! Constructors of plain values ([`KickTimestamp::new`],
//! [`KickDependency::new`], [`KickDependency::from_word`]) return `None` for
//! values outside their domain; builders of firmware records and of encoded
//! words return `EINVAL`.

use kernel::prelude::*;

use super::queue::Policy;

/// Number of entries in a kick ring.
pub(crate) const KICK_SLOTS: usize = 256;
/// Distance between consecutive kick ring entries.
pub(crate) const KICK_ENTRY_STRIDE: usize = 0x200;
/// Size of a kick ring.
pub(crate) const KICK_RING_SIZE: usize = KICK_SLOTS * KICK_ENTRY_STRIDE;
/// Dependencies one entry can carry: explicit barriers plus the implicit parent.
pub(crate) const KICK_DEPENDENCIES: usize = 32;
/// Explicit barriers one entry can carry.
pub(crate) const KICK_BARRIERS_MAX: usize = KICK_DEPENDENCIES - 1;
/// Highest hardware queue ID.
pub(crate) const QID_MAX: u8 = 0x7f;
/// Highest UAT context ID.
pub(crate) const CONTEXT_MAX: u8 = 0x3f;

const KICK_TIMESTAMP_BITS: u32 = 40;
const KICK_TIMESTAMP_MASK: u64 = (1 << KICK_TIMESTAMP_BITS) - 1;
const KICK_SLOT_MASK: u64 = KICK_SLOTS as u64 - 1;
/// GPU addresses packed into kick entries are 32-byte aligned and 43 bits wide.
const PACKED_VA_SHIFT: u32 = 5;
const PACKED_VA_LIMIT: u64 = 1 << (38 + PACKED_VA_SHIFT);

/// Packs a 32-byte aligned GPU address into the low 38 bits of a word.
fn packed_va(va: u64) -> Result<u64> {
    if va & ((1 << PACKED_VA_SHIFT) - 1) != 0 || va >= PACKED_VA_LIMIT {
        return Err(EINVAL);
    }
    Ok(va >> PACKED_VA_SHIFT)
}

/// A kick timestamp.
///
/// The value is a 40-bit counter that advances by one per kick of a queue;
/// bits 7:0 select the kick ring slot. A queue's first kick uses
/// [`KickTimestamp::FIRST`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct KickTimestamp(u64);

impl KickTimestamp {
    /// The timestamp preceding a queue's first kick.
    pub(crate) const ZERO: Self = Self(0);
    /// The timestamp of a queue's first kick.
    pub(crate) const FIRST: Self = Self(1);

    /// Returns the timestamp with raw value `value`, if it fits 40 bits.
    pub(crate) const fn new(value: u64) -> Option<Self> {
        if value & !KICK_TIMESTAMP_MASK != 0 {
            None
        } else {
            Some(Self(value))
        }
    }

    /// Returns the raw 40-bit value.
    pub(crate) const fn get(self) -> u64 {
        self.0
    }

    /// Returns the kick ring slot this timestamp selects.
    pub(crate) const fn slot(self) -> usize {
        (self.0 & KICK_SLOT_MASK) as usize
    }

    /// Returns the byte offset of the selected entry within the kick ring.
    pub(crate) const fn entry_offset(self) -> usize {
        self.slot() * KICK_ENTRY_STRIDE
    }

    /// Returns the timestamp of the following kick.
    pub(crate) const fn next(self) -> Self {
        Self(self.0.wrapping_add(1) & KICK_TIMESTAMP_MASK)
    }

    /// Returns the timestamp of the preceding kick.
    pub(crate) const fn prev(self) -> Self {
        Self(self.0.wrapping_sub(1) & KICK_TIMESTAMP_MASK)
    }

    /// Returns the completion value of this kick: the timestamp in bits 39:0
    /// and `pass_seq`, the low byte of the logical queue's render-pass count,
    /// in bits 47:40.
    pub(crate) const fn completion_value(self, pass_seq: u8) -> u64 {
        self.0 | (pass_seq as u64) << KICK_TIMESTAMP_BITS
    }
}

/// A dependency on the completion of one kick.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct KickDependency {
    qid: u8,
    timestamp: KickTimestamp,
}

impl KickDependency {
    const QID_SHIFT: u32 = KICK_TIMESTAMP_BITS;

    /// Initial value for unused entries in a host-side dependency array.
    pub(crate) const ZERO: Self = Self {
        qid: 0,
        timestamp: KickTimestamp::ZERO,
    };

    /// Returns the dependency on kick `timestamp` of queue `qid`.
    pub(crate) const fn new(qid: u8, timestamp: KickTimestamp) -> Option<Self> {
        if qid > QID_MAX {
            None
        } else {
            Some(Self { qid, timestamp })
        }
    }

    /// Returns the queue that was kicked.
    pub(crate) const fn qid(self) -> u8 {
        self.qid
    }

    /// Returns the dependency word: the timestamp in bits 39:0 and the QID in
    /// bits 46:40.
    pub(crate) const fn word(self) -> u64 {
        self.timestamp.0 | (self.qid as u64) << Self::QID_SHIFT
    }

    /// Decodes a dependency word; bits 63:47 must be clear.
    pub(crate) const fn from_word(word: u64) -> Option<Self> {
        if word >> (Self::QID_SHIFT + 7) != 0 {
            return None;
        }
        Some(Self {
            qid: (word >> Self::QID_SHIFT) as u8,
            timestamp: KickTimestamp(word & KICK_TIMESTAMP_MASK),
        })
    }
}

/// One register write of a [`RegisterArray`].
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegisterWrite {
    /// Register number.
    pub(crate) reg: u32,
    /// Value written to the register.
    pub(crate) value: u64,
}

impl RegisterWrite {
    /// Returns the write of `value` to register `reg`.
    pub(crate) const fn new(reg: u32, value: u64) -> Self {
        Self { reg, value }
    }
}

/// A buffer that work writes.
#[derive(Copy, Clone, Debug)]
pub(crate) struct WriteRange {
    /// GPU address of the buffer.
    pub(crate) va: u64,
    /// Size of the buffer in bytes.
    pub(crate) size: u64,
}

/// Class of an [`McacheRange`].
///
/// The auxiliary framebuffer range takes class 2 and attachment ranges take
/// class 1.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum McacheClass {
    /// A render-target attachment.
    Attachment = 1,
    /// The auxiliary framebuffer.
    AuxFramebuffer = 2,
}

/// One MCache range: a GPU address range that work writes, in 128-byte cache
/// lines.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct McacheRange {
    /// First cache line in bits 40:0, UAT context in bits 46:41.
    pub(crate) start: u64,
    /// End cache line (exclusive) in bits 40:0 and the class above.
    pub(crate) end: u64,
}

impl McacheRange {
    const LINE_SIZE: u64 = 0x80;
    /// Ranges cover fewer than 2^32 cache lines (512 GiB).
    const LINES_MAX: u64 = u32::MAX as u64;
    const LINE_LIMIT: u64 = 1 << 41;
    const CONTEXT_SHIFT: u32 = 41;
    /// Bits 55:47 of the start word hold 0x118 in every range.
    const UNK_START: u64 = 0x118 << 47;
    /// Bit 41 of the end word is set in every range.
    const UNK_END: u64 = 1 << 41;
    const CLASS_SHIFT: u32 = 42;

    /// Encodes the cache lines `range` touches, translated in UAT context
    /// `context`.
    pub(crate) fn new(range: &WriteRange, class: McacheClass, context: u8) -> Result<Self> {
        if range.size == 0 || context > CONTEXT_MAX {
            return Err(EINVAL);
        }
        let start = range.va / Self::LINE_SIZE;
        let end = range
            .va
            .checked_add(range.size)
            .ok_or(EINVAL)?
            .div_ceil(Self::LINE_SIZE);
        if end >= Self::LINE_LIMIT || end - start > Self::LINES_MAX {
            return Err(EINVAL);
        }
        Ok(Self {
            start: start | u64::from(context) << Self::CONTEXT_SHIFT | Self::UNK_START,
            end: end | Self::UNK_END | 1 << (Self::CLASS_SHIFT + class as u32),
        })
    }
}

/// Register-write entries a [`RegisterArray`] holds.
pub(crate) const REGISTER_ARRAY_ENTRIES: usize = 128;
/// MCache ranges that fit in the tail of a [`RegisterArray`].
pub(crate) const MCACHE_INLINE_RANGES: usize = 16;

/// A register array embedded in a work descriptor and bound to its kick entry
/// by a [`RegisterArrayBinding`].
///
/// The array ends in a trailer that points back at the array and holds its
/// size. It is only written as part of a cleared descriptor: [`Self::set`]
/// leaves the unused entries and the MCache space untouched.
#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegisterArray {
    /// Register writes, in order.
    pub(crate) entries: [RegisterWrite; REGISTER_ARRAY_ENTRIES],
    /// Space after the entries. The first fragment array keeps its MCache
    /// table here when the table fits; it is zero everywhere else.
    pub(crate) mcache: [McacheRange; MCACHE_INLINE_RANGES],
    /// GPU address of this array, as seen by the work's VM.
    pub(crate) self_va: u64,
    /// Size of the used entries in bytes (bits 31:16) and their count (15:0).
    pub(crate) size: u32,
    pub(crate) unk_70c: [u8; 0x14],
}

impl RegisterArray {
    /// Stores `writes` and the trailer into a zeroed array.
    pub(crate) fn set(&mut self, self_va: u64, writes: &[RegisterWrite]) -> Result {
        let count = writes.len();
        if count > REGISTER_ARRAY_ENTRIES {
            return Err(EINVAL);
        }
        self.entries[..count].copy_from_slice(writes);
        self.self_va = self_va;
        self.size = (size_of_val(writes) as u32) << 16 | count as u32;
        Ok(())
    }

    /// Stores `writes` without the entry at `skip`, exactly as [`Self::set`] stores the
    /// remaining writes, without first copying them elsewhere.
    pub(crate) fn set_without(&mut self, self_va: u64, writes: &[RegisterWrite], skip: usize) -> Result {
        let count = writes.len().checked_sub(1).ok_or(EINVAL)?;
        if skip > count || count > REGISTER_ARRAY_ENTRIES {
            return Err(EINVAL);
        }
        self.entries[..skip].copy_from_slice(&writes[..skip]);
        self.entries[skip..count].copy_from_slice(&writes[skip + 1..]);
        self.self_va = self_va;
        self.size = ((count * size_of::<RegisterWrite>()) as u32) << 16 | count as u32;
        Ok(())
    }
}

/// Binding of one [`RegisterArray`] to a kick entry.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct RegisterArrayBinding {
    word: u64,
}

impl RegisterArrayBinding {
    const CHECKPOINT_SHIFT: u32 = 43;
    const CHECKPOINT_MAX: u8 = 0x7f;

    /// Binds the array at `array_va`, as seen by the work's VM, whose first
    /// checkpoint is after `checkpoint` entries.
    pub(crate) fn new(array_va: u64, checkpoint: u8) -> Result<Self> {
        if checkpoint > Self::CHECKPOINT_MAX {
            return Err(EINVAL);
        }
        Ok(Self {
            word: packed_va(array_va)? | u64::from(checkpoint) << Self::CHECKPOINT_SHIFT,
        })
    }

    fn word(binding: Option<Self>) -> u64 {
        binding.map_or(0, |binding| binding.word)
    }
}

/// Location of the MCache table a kick entry names.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct McacheTable {
    va: u64,
    count: u8,
}

impl McacheTable {
    const VA_SHIFT: u32 = 6;
    const COUNT_SHIFT: u32 = 54;
    const COUNT_MAX: usize = 64;

    /// Names the `count` ranges at `va`, as seen by the work's VM.
    pub(crate) fn new(va: u64, count: usize) -> Result<Self> {
        if count == 0 || count > Self::COUNT_MAX {
            return Err(EINVAL);
        }
        packed_va(va)?;
        Ok(Self {
            va,
            count: count as u8,
        })
    }

    fn word(self) -> u64 {
        (self.va >> PACKED_VA_SHIFT) << Self::VA_SHIFT
            | u64::from(self.count - 1) << Self::COUNT_SHIFT
    }
}

/// Quality-of-service identity of the kicked queue.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct KickQos {
    /// QoS buffer slot of the logical queue.
    pub(crate) slot: u8,
    /// Scheduling class of the engine, at most 0x1f.
    pub(crate) class: u8,
}

/// Inputs of one kick entry.
pub(crate) struct KickArgs<'a> {
    /// Kicked queue.
    pub(crate) qid: u8,
    /// Timestamp of this kick; it selects the ring slot.
    pub(crate) timestamp: KickTimestamp,
    /// Kick this kick implicitly depends on: the previous kick of the same
    /// queue. A queue's first kick names `timestamp.prev()` on render queues
    /// and [`KickTimestamp::ZERO`] on compute queues.
    pub(crate) parent: KickTimestamp,
    /// Kicks of other queues this kick must wait for.
    pub(crate) barriers: &'a [KickDependency],
    /// GPU address of the work descriptor (firmware alias).
    pub(crate) descriptor_va: u64,
    /// GPU address of the queue record.
    pub(crate) queue_va: u64,
    /// Queue QoS identity.
    pub(crate) qos: KickQos,
    /// MCache table of the work, if any.
    pub(crate) mcache: Option<McacheTable>,
    /// Engine-specific event mask.
    pub(crate) event_mask: [u64; 4],
    /// Register arrays bound to the kick.
    pub(crate) register_arrays: [Option<RegisterArrayBinding>; 4],
    /// The compute register program includes a shader-context scratch request.
    pub(crate) scs_enabled: bool,
    /// Priority class of the queue.
    pub(crate) priority: u8,
}

/// One kick ring entry.
///
/// Live state: none. The host writes the whole entry before announcing it and
/// does not touch it again until the slot is reused.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct KickEntry {
    /// Timestamp in bits 41:2, QID in bits 48:42, MCache table present in bit
    /// 58, queue priority in bits 60:59 and compute scratch enabled in bit 61.
    pub(crate) header: u64,
    /// MCache table: address >> 5 in bits 43:6, count - 1 in bits 59:54.
    pub(crate) mcache: u64,
    /// GPU address of the work descriptor.
    pub(crate) descriptor_va: u64,
    /// GPU address of the queue record.
    pub(crate) queue_va: u64,
    /// Valid-dependency mask in bits 31:0, QoS slot in bits 39:32, class in
    /// bits 47:40; bits 63:48 are all ones.
    pub(crate) dependency_control: u64,
    /// Explicit barriers followed by the implicit parent.
    pub(crate) dependencies: [u64; KICK_DEPENDENCIES],
    /// Engine-specific event mask.
    pub(crate) event_mask: [u64; 4],
    pub(crate) unk_148: u64,
    /// Register-array bindings: address >> 5 in bits 37:0, checkpoint in bits
    /// 49:43.
    pub(crate) register_arrays: [u64; 4],
    pub(crate) unk_170: u64,
    pub(crate) unk_178: u64,
    pub(crate) pad_180: [u8; KICK_ENTRY_STRIDE - 0x180],
}

static_assert!(size_of::<KickEntry>() == KICK_ENTRY_STRIDE);
static_assert!(core::mem::offset_of!(KickEntry, dependency_control) == 0x20);
static_assert!(core::mem::offset_of!(KickEntry, event_mask) == 0x128);
static_assert!(core::mem::offset_of!(KickEntry, register_arrays) == 0x150);
static_assert!(core::mem::offset_of!(KickEntry, unk_178) == 0x178);
static_assert!(size_of::<RegisterWrite>() == 0xc);
static_assert!(size_of::<McacheRange>() == 0x10);
static_assert!(size_of::<RegisterArray>() == 0x720);
static_assert!(core::mem::offset_of!(RegisterArray, mcache) == 0x600);
static_assert!(core::mem::offset_of!(RegisterArray, self_va) == 0x700);

impl KickEntry {
    const TIMESTAMP_SHIFT: u32 = 2;
    const QID_SHIFT: u32 = 42;
    const MCACHE_PRESENT: u64 = 1 << 58;
    const PRIORITY_SHIFT: u32 = 59;
    const SCS_ENABLED: u64 = 1 << 61;
    const QOS_SLOT_SHIFT: u32 = 32;
    const QOS_CLASS_SHIFT: u32 = 40;
    const QOS_CLASS_MAX: u8 = 0x1f;
    /// Bits 63:48 of the dependency control word.
    const UNK_DEPENDENCY_CONTROL: u64 = 0xffff << 48;
    /// Required value of the last word of the entry.
    const UNK_178: u64 = 0x003f_ffff_ffff_ffff;

    /// Builds the entry described by `args`.
    pub(crate) fn new(args: &KickArgs<'_>) -> Result<Self> {
        let barriers = args.barriers.len();
        if args.qid > QID_MAX
            || barriers > KICK_BARRIERS_MAX
            || args.qos.class > Self::QOS_CLASS_MAX
            || args.priority >= Policy::PRIORITIES
        {
            return Err(EINVAL);
        }

        let mut header = args.timestamp.get() << Self::TIMESTAMP_SHIFT
            | u64::from(args.qid) << Self::QID_SHIFT
            | u64::from(args.priority) << Self::PRIORITY_SHIFT;
        if args.scs_enabled {
            header |= Self::SCS_ENABLED;
        }
        let mcache = match args.mcache {
            Some(table) => {
                header |= Self::MCACHE_PRESENT;
                table.word()
            }
            None => 0,
        };

        let mut dependencies = [0; KICK_DEPENDENCIES];
        for (slot, barrier) in dependencies.iter_mut().zip(args.barriers) {
            *slot = barrier.word();
        }
        dependencies[barriers] = KickDependency {
            qid: args.qid,
            timestamp: args.parent,
        }
        .word();
        let count = barriers + 1;
        let mask = if count == KICK_DEPENDENCIES {
            u32::MAX
        } else {
            (1u32 << count) - 1
        };

        Ok(Self {
            header,
            mcache,
            descriptor_va: args.descriptor_va,
            queue_va: args.queue_va,
            dependency_control: u64::from(mask)
                | u64::from(args.qos.slot) << Self::QOS_SLOT_SHIFT
                | u64::from(args.qos.class) << Self::QOS_CLASS_SHIFT
                | Self::UNK_DEPENDENCY_CONTROL,
            dependencies,
            event_mask: args.event_mask,
            unk_148: 0,
            register_arrays: args.register_arrays.map(RegisterArrayBinding::word),
            unk_170: 0,
            unk_178: Self::UNK_178,
            pad_180: [0; KICK_ENTRY_STRIDE - 0x180],
        })
    }
}

/// The complete write-only installation of a fragment/tiling queue pair.
/// Firmware must see both queues configured before either outer command.
pub(crate) struct RenderRegistration {
    writes: [(usize, u64); 8],
}

impl RenderRegistration {
    const TAG_PORT: usize = 0x4090;
    const DATA_PORT: usize = 0x4098;
    const VALID_LOW: usize = 0x21068;
    const VALID_HIGH: usize = 0x21070;
    const CONFIG_TAG: u64 = 0x0000_0400_0002_1008;
    const ADDRESS_MASK: u64 = 0x0000_07ff_ffff_ffe0;

    /// Validates both queues without touching MMIO. The resulting transaction
    /// writes fragment configuration and validity before tiling configuration
    /// and validity. Each validity mask is stored high before low. Both queues
    /// are registered at priority class `priority`.
    pub(crate) fn new(fragment: (u8, u64), tiling: (u8, u64), priority: u8) -> Result<Self> {
        if fragment.0 == tiling.0 || priority >= Policy::PRIORITIES {
            return Err(EINVAL);
        }
        let mut writes = [(0, 0); 8];
        for (slot, ((qid, va), engine)) in [
            (fragment, super::queue::DataMaster::Fragment),
            (tiling, super::queue::DataMaster::Tiling),
        ]
        .into_iter()
        .enumerate()
        {
            if qid > QID_MAX || va & !Self::ADDRESS_MASK != 0 {
                return Err(EINVAL);
            }
            let completion = (3 * (4 - u64::from(priority)) + 0x3fe).wrapping_shl(54);
            let payload = ((engine as u64 + 1) << 60) | va | completion;
            let (low, high) = if qid < 64 {
                (1u64 << qid, 0)
            } else {
                (0, 1u64 << (qid - 64))
            };
            writes[slot * 4..slot * 4 + 4].copy_from_slice(&[
                (Self::TAG_PORT, Self::CONFIG_TAG | (u64::from(qid) << 43)),
                (Self::DATA_PORT, payload),
                (Self::VALID_HIGH, high),
                (Self::VALID_LOW, low),
            ]);
        }
        Ok(Self { writes })
    }

    pub(crate) fn writes(&self) -> &[(usize, u64); 8] {
        &self.writes
    }
}
