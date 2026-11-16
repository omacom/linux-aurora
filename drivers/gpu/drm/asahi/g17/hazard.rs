// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Attachment overlap ordering across physical render pairs and shared images.
//! Tile stores can overlap even when the application's pixel regions do not.
//! Keep independent attachments concurrent; order only conflicting live spans.
//!
//! Within one VM, a render of another logical owner is ordered at publication:
//! the device mutex totally orders publications, and the later render's
//! fragment kick waits in firmware for the newest overlapping fragment kick of
//! every other pair. Admission never waits for such an overlap. Only a
//! cross-VM overlap through a shared object waits on the host for retirement,
//! so that one VM's fault can never hold another VM's kick in firmware.

use super::job::Order;

const MAX_SPANS: usize = 24;
pub(crate) const WAIT_MS: u32 = 1000;
/// Firmware barriers one render publication may add for same-VM overlaps,
/// one per other pair.
pub(crate) const PUBLICATION_BARRIERS: usize = 7;

#[derive(Clone, Copy)]
struct Span {
    key: u64,
    start: u64,
    end: u64,
}

/// Up to sixteen attachment ranges and eight shared-object ranges.
pub(crate) struct Footprint {
    vm: u64,
    spans: [Span; MAX_SPANS],
    count: usize,
    shared: bool,
}

impl Footprint {
    pub(crate) fn new(vm: u64) -> Self {
        Self {
            vm,
            spans: [Span {
                key: 0,
                start: 0,
                end: 0,
            }; MAX_SPANS],
            count: 0,
            shared: false,
        }
    }

    fn push(&mut self, key: u64, start: u64, end: u64) {
        if start < end && self.count < MAX_SPANS {
            self.spans[self.count] = Span { key, start, end };
            self.count += 1;
        }
    }

    pub(crate) fn push_vm(&mut self, start: u64, end: u64) {
        self.push(self.vm << 1, start, end);
    }

    /// An aligned GEM object address is an identity, never dereferenced here.
    pub(crate) fn push_object(&mut self, object: usize, start: u64, end: u64) {
        let before = self.count;
        self.push(object as u64 | 1, start, end);
        self.shared |= self.count != before;
    }

    /// Whether a publication orders an overlap with `other` in firmware.
    pub(crate) fn same_vm(&self, other: &Self) -> bool {
        self.vm == other.vm
    }

    pub(crate) fn overlaps(&self, other: &Self) -> bool {
        if self.vm != other.vm && !(self.shared && other.shared) {
            return false;
        }
        self.spans[..self.count].iter().any(|a| {
            other.spans[..other.count]
                .iter()
                .any(|b| a.key == b.key && a.start < b.end && b.start < a.end)
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Plan {
    Any,
    Pair(u8),
    Wait,
}

pub(crate) struct Live<'a> {
    pub(crate) owner: u64,
    pub(crate) sequence: u64,
    pub(crate) slot: Option<u8>,
    pub(crate) ticket: u64,
    pub(crate) footprint: &'a Footprint,
    pub(crate) quarantined: bool,
}

/// A queue prefix already orders covered predecessors. Pending arrivals order
/// different owners fairly without imposing order on unrelated attachments.
///
/// A same-VM overlap with another owner's admitted render needs no wait; its
/// publication carries one barrier per other pair. That count stays within
/// [`PUBLICATION_BARRIERS`] because such a render is admitted only while fewer
/// than `PUBLICATION_BARRIERS + 1` other pairs of its VM hold admitted renders:
/// of any set of renders overlapping one publication, the last admitted saw all
/// the others live.
pub(crate) fn plan<'a>(
    owner: u64,
    prefix: Option<u64>,
    ticket: Option<u64>,
    footprint: &Footprint,
    live: impl IntoIterator<Item = Live<'a>>,
) -> Plan {
    let mut selected = None;
    let mut ordered = false;
    let mut pairs = [0u8; PUBLICATION_BARRIERS];
    let mut count = 0;
    let mut crowded = false;
    for entry in live {
        let foreign = entry.owner != owner && footprint.same_vm(entry.footprint);
        if let Some(slot) = entry.slot.filter(|_| foreign) {
            if !pairs[..count].contains(&slot) {
                if count == PUBLICATION_BARRIERS {
                    crowded = true;
                } else {
                    pairs[count] = slot;
                    count += 1;
                }
            }
        }
        if entry.quarantined || !footprint.overlaps(entry.footprint) {
            continue;
        }
        if foreign {
            ordered |= entry.slot.is_some();
            continue;
        }
        let Some(slot) = entry.slot else {
            if entry.owner != owner && ticket.is_none_or(|mine| entry.ticket < mine) {
                return Plan::Wait;
            }
            continue;
        };
        if entry.owner != owner {
            return Plan::Wait;
        }
        if Order::contains(prefix, entry.sequence) {
            continue;
        }
        match selected {
            None => selected = Some(slot),
            Some(prior) if prior == slot => {}
            Some(_) => return Plan::Wait,
        }
    }
    if ordered && crowded {
        return Plan::Wait;
    }
    selected.map_or(Plan::Any, Plan::Pair)
}
