// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact identities and visibility witnesses for a physical compute queue.
//! A shared work-state counter can include other engines and cannot prove this queue retired.

use crate::g17::initdata::ComputeCompletion;
use kernel::prelude::*;

pub(crate) const IN_FLIGHT: usize = 16;
pub(crate) const RECORD_SLOTS: usize = 32;
const QIDS: usize = 128;
const TIME_MASK: u64 = 0x003f_ffff_ffff_ffff;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Ticket {
    pub(crate) owner: u64,
    pub(crate) submission: u64,
    pub(crate) context_id: u32,
    pub(crate) qid: u8,
    pub(crate) descriptor: u64,
    pub(crate) work_node: u64,
    pub(crate) kick: u64,
    pub(crate) ordinal: u32,
    pub(crate) item_producer: u32,
    pub(crate) item_count: u32,
    pub(crate) epoch: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Observation {
    pub(crate) timestamps: [u64; 2],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Cursors {
    pub(crate) producer: u32,
    pub(crate) consumer: u32,
    pub(crate) count: u32,
}

impl Cursors {
    /// Whether firmware consumed every record of the given publication, allowing later work.
    pub(crate) fn consumed(&self, end: u32, count: u32) -> Result<bool> {
        if self.count == 0 || self.count != count {
            return Err(EIO);
        }
        let producer = self.producer % count;
        let consumer = self.consumer % count;
        let unconsumed = (producer + count - consumer) % count;
        let following = (producer + count - end % count) % count;
        Ok(unconsumed <= following)
    }
}

/// Outstanding kicks span fewer than half the 24-bit stamp sequence. Zero is a valid wrap.
pub(crate) const fn stamp_covers(word: u32, kick: u64) -> bool {
    let delta = word.wrapping_sub((kick as u32).wrapping_shl(8));
    delta & 0xff == 0 && delta < 0x8000_0000
}

#[derive(Clone, Copy)]
pub(crate) enum Recordless {
    Allowed,
    NotYet,
    CrossedRecovery,
}

/// How long a command whose exact completion record is present waits for its completion stamp
/// before it retires on the record alone. The firmware stores the stamp after the cache
/// maintenance that follows the record; it occasionally never stores it for a command, most
/// visibly while a render of the same VM is running.
pub(crate) const STAMP_GRACE_NS: u64 = 1_000_000;

static STAMPLESS: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Counts a command that retired on its record without its completion stamp; logs the first and
/// every 64th.
pub(crate) fn note_stampless(qid: u8, kick: u64) {
    let count = STAMPLESS.fetch_add(1, core::sync::atomic::Ordering::Relaxed) + 1;
    if count == 1 || count % 64 == 0 {
        pr_info!(
            "G17: {} compute commands retired on their completion record without a completion stamp (latest: queue {} kick {:#x})\n",
            count,
            qid,
            kick
        );
    }
}

/// A zero-duration record fails before visibility polling. It proves execution did not occur;
/// ownership still requires a separate retirement witness before mappings can be released.
/// The command is complete when its stamp covers it, or when its exact completion record has
/// been present for [`STAMP_GRACE_NS`] (`record_aged`); in both cases its items must be consumed.
pub(crate) fn poll(
    ticket: &Ticket,
    observed: Option<Observation>,
    cursors: Cursors,
    stamp: u32,
    recordless: Recordless,
    floor: u64,
    next_end: Option<u64>,
    record_aged: bool,
) -> Result<Option<[u64; 2]>> {
    if observed.is_some_and(|record| record.timestamps[0] == record.timestamps[1]) {
        return Err(ENODATA);
    }
    let completed = stamp_covers(stamp, ticket.kick) || (observed.is_some() && record_aged);
    if !completed || !cursors.consumed(ticket.item_producer, ticket.item_count)? {
        return Ok(None);
    }
    Ok(Some(match observed {
        Some(record) => record.timestamps,
        None => match recordless {
            Recordless::Allowed => [floor, next_end.unwrap_or(floor).max(floor)],
            Recordless::NotYet => return Ok(None),
            Recordless::CrossedRecovery => return Err(ENODATA),
        },
    }))
}

/// The newest exact publication covers all preceding kicks, but only a fully consumed ring
/// permits releasing quarantined command pins. The installed graph remains retained.
pub(crate) fn quarantined_retired(
    ticket: &Ticket,
    cursors: Cursors,
    stamp: u32,
    pending: u32,
    submitted: u32,
) -> Result<bool> {
    if ticket.item_count == 0 || ticket.descriptor == 0 || pending == 0 {
        return Err(EINVAL);
    }
    if cursors.count == 0 || cursors.count != ticket.item_count {
        return Err(EIO);
    }
    let end = ticket.item_producer % cursors.count;
    Ok(stamp_covers(stamp, ticket.kick)
        && cursors.producer % cursors.count == end
        && cursors.consumer % cursors.count == end
        && submitted == pending)
}

#[derive(Clone, Copy)]
struct Request {
    descriptor: u64,
    kick: u64,
    observed: Option<Observation>,
}

/// Reusable heap scratch for one shared-ring scan. QID requests are contiguous and bounded;
/// staging and folding never allocate under the device mutex.
pub(crate) struct Batch {
    requests: KVVec<Request>,
    slots: [(usize, usize); QIDS],
    open: Option<u8>,
}

impl Batch {
    /// Initialized in place: the slot table is too large to stage on the probe stack.
    pub(crate) fn new() -> impl Init<Self, Error> {
        kernel::try_init!(Self {
            requests: KVVec::with_capacity(QIDS * IN_FLIGHT, GFP_KERNEL)?,
            slots <- pin_init::init_zeroed(),
            open: None,
        })
    }
    pub(crate) fn clear(&mut self) {
        self.requests.clear();
        self.slots.fill((0, 0));
        self.open = None;
    }
    pub(crate) fn request(&mut self, ticket: &Ticket) -> Result {
        if ticket.descriptor == 0 || ticket.kick == 0 {
            return Err(EINVAL);
        }
        let slot = self.slots.get_mut(ticket.qid as usize).ok_or(EINVAL)?;
        if slot.1 != 0 && self.open != Some(ticket.qid) {
            return Err(EBUSY);
        }
        if slot.1 >= IN_FLIGHT {
            return Err(ENOSPC);
        }
        let start = if slot.1 == 0 {
            self.requests.len()
        } else {
            slot.0
        };
        self.requests
            .push_within_capacity(Request {
                descriptor: ticket.descriptor,
                kick: ticket.kick,
                observed: None,
            })
            .map_err(|_| ENOSPC)?;
        *slot = (start, slot.1 + 1);
        self.open = Some(ticket.qid);
        Ok(())
    }
    /// Capture the control word once, then read only a requested identity's payload.
    /// Records can be overwritten during a scan: incomplete or older identities are skipped.
    pub(crate) fn fold(&mut self, record: ComputeCompletion<'_>) -> Result {
        let control = record.read(0)?;
        let qid = ((control >> 9) & 0x7f) as usize;
        let (start, count) = self.slots[qid];
        if count == 0 {
            return Ok(());
        }
        let descriptor = record.read(6)? & !1;
        if descriptor == 0 || record.read(7)? == 0 {
            return Ok(());
        }
        let kick = (((control >> 8) & 0xffff_ffff_00) | ((control >> 48) & 0xff)) & 0xff_ffff_ffff;
        let requests = self.requests.get_mut(start..start + count).ok_or(EIO)?;
        let Some(request) = requests
            .iter_mut()
            .find(|r| r.descriptor == descriptor && r.kick == kick)
        else {
            return Ok(());
        };
        let timestamps = [record.read(2)? & TIME_MASK, record.read(5)? & TIME_MASK];
        if timestamps[0] == 0
            || timestamps[1] == 0
            || request
                .observed
                .is_some_and(|old| old.timestamps[1] >= timestamps[1])
        {
            return Ok(());
        }
        request.observed = Some(Observation { timestamps });
        Ok(())
    }
    pub(crate) fn take(&mut self, ticket: &Ticket) -> Result<Option<Observation>> {
        let &(start, count) = self.slots.get(ticket.qid as usize).ok_or(EINVAL)?;
        let request = self
            .requests
            .get_mut(start..start + count)
            .ok_or(EIO)?
            .iter_mut()
            .find(|r| r.descriptor == ticket.descriptor && r.kick == ticket.kick)
            .ok_or(EIO)?;
        Ok(request.observed.take())
    }
}
