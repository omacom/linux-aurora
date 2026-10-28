// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Collapse a logical prefix to its newest still-live firmware kick per QID.

use super::{
    fw::kick::{KickDependency, KICK_BARRIERS_MAX},
    job::Order,
};
use kernel::prelude::*;

/// A failed producer does not establish a fault in its consumer's VM.
pub(crate) fn inherited_error(error: Error) -> Error {
    if error == EIO || error == ENODEV || error == ETIMEDOUT {
        ECANCELED
    } else {
        error
    }
}

/// A replay's original host inputs must still be signalled. Treat an unmet
/// input as a failed replay, never as a wait under the device mutex.
pub(crate) fn replay_status(status: i32) -> Result {
    if status == 0 { Err(EBUSY) }
    else if status < 0 { Err(inherited_error(Error::from_errno(status))) }
    else { Ok(()) }
}

/// Whether a failed command may still run or write: it holds its VM pin until it has physically
/// retired, or was never published.
pub(crate) trait Retention {
    fn retained(&self) -> bool;
}

/// The failed ordinals of one engine that later commands must not bypass. A command whose prefix
/// covers one is cancelled, so none runs before a failed producer the firmware may still replay.
/// Once that producer has retired nothing of it can run or write any more, so it stops counting:
/// otherwise one cancelled command cancelled every later command of its queue for good.
pub(crate) struct FailureRecord<T> {
    retained: KVec<(u64, T)>,
    /// A failure that could not be tracked: it keeps cancelling for good, as all failures once did.
    untracked: Option<(u64, T)>,
}

impl<T: Retention + Clone> FailureRecord<T> {
    pub(crate) const fn new() -> Self {
        Self { retained: KVec::new(), untracked: None }
    }

    /// A finished frontier entry, at pruning.
    pub(crate) fn note(&mut self, sequence: u64, status: i32, entry: &T) {
        if status >= 0 || !entry.retained() {
            return;
        }
        if self.retained.push((sequence, entry.clone()), GFP_KERNEL).is_err()
            && self.untracked.as_ref().is_none_or(|(first, _)| sequence < *first)
        {
            self.untracked = Some((sequence, entry.clone()));
        }
    }

    /// The earliest failure later prefixes must not bypass.
    pub(crate) fn earliest(&mut self) -> Option<(u64, &T)> {
        self.retained.retain(|(_, entry)| entry.retained());
        Iterator::chain(self.retained.iter(), self.untracked.iter())
            .min_by_key(|(sequence, _)| *sequence)
            .map(|(sequence, entry)| (*sequence, entry))
    }
}

pub(crate) struct Frontier {
    dependencies: [KickDependency; KICK_BARRIERS_MAX],
    sequences: [u64; KICK_BARRIERS_MAX],
    count: usize,
}

impl Frontier {
    pub(crate) const fn new() -> Self {
        Self {
            dependencies: [KickDependency::ZERO; KICK_BARRIERS_MAX],
            sequences: [0; KICK_BARRIERS_MAX],
            count: 0,
        }
    }

    pub(crate) fn as_slice(&self) -> &[KickDependency] {
        &self.dependencies[..self.count]
    }

    /// Each QID's implicit parent supplies transitive ordering. Choose its newest
    /// logical sequence rather than comparing wrapped firmware timestamps.
    pub(crate) fn insert(&mut self, sequence: u64, dependency: KickDependency) -> Result {
        if let Some(index) = self
            .as_slice()
            .iter()
            .position(|prior| prior.qid() == dependency.qid())
        {
            if sequence > self.sequences[index] {
                self.dependencies[index] = dependency;
                self.sequences[index] = sequence;
            }
            return Ok(());
        }
        if self.count == KICK_BARRIERS_MAX {
            return Err(EIO);
        }
        self.dependencies[self.count] = dependency;
        self.sequences[self.count] = sequence;
        self.count += 1;
        Ok(())
    }

    /// The caller has matched the logical owner and VM. Check every covered
    /// predecessor for failure, including one a newer kick would subsume.
    pub(crate) fn include(
        &mut self,
        prefix: Option<u64>,
        sequence: u64,
        status: i32,
        word: u64,
    ) -> Result {
        if !Order::contains(prefix, sequence) {
            return Ok(());
        }
        if status < 0 {
            return Err(ECANCELED);
        }
        if status > 0 {
            return Ok(());
        }
        let dependency = KickDependency::from_word(word).ok_or(EIO)?;
        self.insert(sequence, dependency)
    }
}

#[kunit_tests(asahi_g17_dependency)]
mod tests {
    use super::*;

    #[derive(Clone)]
    struct Probe(kernel::sync::Arc<core::sync::atomic::AtomicBool>);

    impl Probe {
        fn new(retained: bool) -> Self {
            Self(kernel::sync::Arc::new(core::sync::atomic::AtomicBool::new(retained), GFP_KERNEL).unwrap())
        }
        fn retire(&self) {
            self.0.store(false, core::sync::atomic::Ordering::Relaxed);
        }
    }

    impl Retention for Probe {
        fn retained(&self) -> bool {
            self.0.load(core::sync::atomic::Ordering::Relaxed)
        }
    }

    fn earliest(record: &mut FailureRecord<Probe>) -> Option<u64> {
        record.earliest().map(|(sequence, _)| sequence)
    }

    #[test]
    fn a_cancelled_command_does_not_poison_its_queue() {
        // Cancelled or failed before publication: it released its pin and can never run.
        let mut record = FailureRecord::new();
        record.note(5, ECANCELED.to_errno(), &Probe::new(false));
        assert_eq!(earliest(&mut record), None);
    }

    #[test]
    fn a_retained_failure_cancels_until_it_retires() {
        let mut record = FailureRecord::new();
        let failed = Probe::new(true);
        record.note(5, EIO.to_errno(), &failed);
        assert_eq!(earliest(&mut record), Some(5));
        failed.retire();
        assert_eq!(earliest(&mut record), None);
    }

    #[test]
    fn a_later_retained_failure_survives_an_earlier_retirement() {
        let mut record = FailureRecord::new();
        let (first, second) = (Probe::new(true), Probe::new(true));
        record.note(5, EIO.to_errno(), &first);
        record.note(9, ECANCELED.to_errno(), &second);
        assert_eq!(earliest(&mut record), Some(5));
        first.retire();
        assert_eq!(earliest(&mut record), Some(9));
        second.retire();
        assert_eq!(earliest(&mut record), None);
    }

    #[test]
    fn successes_are_not_recorded() {
        let mut record = FailureRecord::new();
        record.note(5, 1, &Probe::new(true));
        assert_eq!(earliest(&mut record), None);
    }

    #[test]
    fn failed_predecessor_cannot_be_bypassed() {
        let mut frontier = Frontier::new();
        for error in [ECANCELED, EIO, ETIMEDOUT] {
            assert_eq!(frontier.include(Some(5), 5, error.to_errno(), 0), Err(ECANCELED));
        }
        assert_eq!(frontier.include(Some(5), 5, 1, 0), Ok(()));
        assert_eq!(frontier.include(Some(5), 6, EIO.to_errno(), 0), Ok(()));
        assert_eq!(frontier.count, 0);
    }
}
