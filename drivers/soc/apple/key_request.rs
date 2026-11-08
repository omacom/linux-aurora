// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! One owned application-key request, shared by its file, waiter and worker.

use crate::keys::{KeyRequest, Output};
use kernel::prelude::*;
use kernel::sync::{new_condvar, Arc, CondVar, CondVarTimeoutResult, Mutex};

enum Phase {
    Pending,
    Complete(Result<Output>),
    Abandoned(Error),
    Claimed,
}

#[pin_data]
pub(crate) struct Request {
    pub(crate) id: u64,
    pub(crate) data: KeyRequest,
    #[pin]
    phase: Mutex<Phase>,
    #[pin]
    changed: CondVar,
}

impl Request {
    pub(crate) fn new(id: u64, data: KeyRequest) -> Result<Arc<Self>> {
        Arc::pin_init(try_pin_init!(Self {
            id,
            data,
            phase <- kernel::new_mutex!(Phase::Pending),
            changed <- new_condvar!("sep_key_request"),
        }), GFP_KERNEL)
    }

    pub(crate) fn is_live(&self) -> bool {
        matches!(*self.phase.lock(), Phase::Pending)
            && crate::shim::monotonic_ns() < self.data.deadline_ns
    }

    pub(crate) fn complete(&self, result: Result<Output>) {
        let mut phase = self.phase.lock();
        if matches!(*phase, Phase::Pending) {
            // Worker errors must never restart an accepted private-key syscall.
            // EBUSY is reserved for contention before hardware submission.
            let result = result.map_err(|error| match error {
                ERESTARTSYS | ERESTARTNOINTR | ERESTARTNOHAND | ERESTART_RESTARTBLOCK => EINTR,
                EBUSY => EIO,
                _ => error,
            });
            *phase = Phase::Complete(if crate::shim::monotonic_ns() >= self.data.deadline_ns {
                Err(ETIMEDOUT)
            } else { result });
        }
        drop(phase);
        self.changed.notify_all();
    }

    /// Cancellation revokes delivery, not firmware work already submitted.
    pub(crate) fn cancel(&self, reason: Error) -> Result {
        let mut phase = self.phase.lock();
        match &*phase {
            Phase::Claimed => return Err(EALREADY),
            Phase::Complete(Err(_)) | Phase::Abandoned(_) => return Err(ENOENT),
            _ => *phase = Phase::Abandoned(reason),
        }
        drop(phase);
        self.changed.notify_all();
        Ok(())
    }

    pub(crate) fn wait(&self) -> Result<Output> {
        let mut phase = self.phase.lock();
        loop {
            let now = crate::shim::monotonic_ns();
            if now >= self.data.deadline_ns
                && matches!(*phase, Phase::Pending | Phase::Complete(Ok(_)))
            {
                *phase = Phase::Abandoned(ETIMEDOUT);
            }
            match &*phase {
                Phase::Abandoned(error) => return Err(*error),
                Phase::Claimed => return Err(ENODATA),
                Phase::Complete(Err(error)) => return Err(*error),
                Phase::Complete(_) => {
                    let Phase::Complete(result) = core::mem::replace(&mut *phase, Phase::Claimed) else {
                        return Err(EIO);
                    };
                    return result;
                }
                Phase::Pending => {},
            }
            let timeout = kernel::time::msecs_to_jiffies(
                self.data.deadline_ns.saturating_sub(now).div_ceil(1_000_000) as u32,
            );
            if matches!(self.changed.wait_interruptible_timeout(&mut phase, timeout),
                CondVarTimeoutResult::Signal { .. })
            {
                // Never ERESTARTSYS after submission: a retry could perform a
                // second private-key operation. Also discard a raced success.
                if matches!(*phase, Phase::Pending | Phase::Complete(Ok(_))) {
                    *phase = Phase::Abandoned(EINTR);
                }
                return match &*phase {
                    Phase::Complete(Err(error)) | Phase::Abandoned(error) => Err(*error),
                    _ => Err(EINTR),
                };
            }
        }
    }
}
