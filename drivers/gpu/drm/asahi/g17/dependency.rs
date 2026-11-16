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
