// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! VM admission errors and client failure reporting. No firmware state is accessed here.

use crate::debug::*;
use crate::gem;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use kernel::{new_mutex, prelude::*, sync::Mutex};

const DEBUG_CLASS: DebugFlags = DebugFlags::Errors;

const MIRROR_MAX_SIZE: usize = 64 * 1024;

/// A VM and its accepted jobs share this status, including after queue destruction.
/// Client failure is separate from hardware admission: recovery may preserve the
/// VM while failed accepted work must still be reported before signalling fences.
#[pin_data]
pub(crate) struct VmStatus {
    error: AtomicI32,
    reported_error: AtomicI32,
    /// The client's VM handle is gone: no client can see a lost device any more.
    client_closed: AtomicBool,
    /// A failure of this VM has been logged as a warning.
    warned: AtomicBool,
    /// The client's mapped status word, retained with its object only while the
    /// client's VM handle exists; released by `release_mirror` at VM close.
    #[pin]
    mirror: Mutex<Option<Mirror>>,
}

/// One aligned word inside the retained, vmapped mirror object.
struct Mirror {
    word: *mut i32,
    _owner: gem::ObjectRef,
}
// SAFETY: The word points into the mapping the retained owner keeps alive; it is
// only accessed atomically through this pointer while the mutex is held.
unsafe impl Send for Mirror {}

impl VmStatus {
    pub(crate) fn new() -> impl PinInit<Self> {
        pin_init!(Self {
            error: AtomicI32::new(0),
            reported_error: AtomicI32::new(0),
            client_closed: AtomicBool::new(false),
            warned: AtomicBool::new(false),
            mirror <- new_mutex!(None, "Asahi VM status mirror"),
        })
    }

    pub(crate) fn get(&self) -> i32 {
        self.error.load(Ordering::Acquire)
    }

    /// Record permanent admission failure and report it to the client.
    #[track_caller]
    pub(crate) fn record(&self, error: Error) {
        self.report_failure(error);
        let _ = self.error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
    }

    pub(crate) fn reported_error(&self) -> i32 {
        self.reported_error.load(Ordering::Acquire)
    }

    /// Report failed accepted work without preventing recovery of the VM.
    /// Every reporter publishes the first error before returning: a losing
    /// reporter may signal its fence before the winning reporter resumes.
    ///
    /// The first failure reported to a VM is logged once, with the reporting site: the client treats
    /// any reported failure as a lost device, and nothing else in the kernel log names it.
    /// Teardown of a closed client fails its queued work with ENOENT (VM context gone) or
    /// ECANCELED (queue destroyed); nobody can see that, so it is only a debug message, and the
    /// first other failure of that VM, such as device loss (EIO, ETIMEDOUT, ENODEV), is still
    /// logged as a warning.
    #[track_caller]
    pub(crate) fn report_failure(&self, error: Error) {
        let first = self.reported_error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
        let at = core::panic::Location::caller();
        let file = at.file().rsplit("asahi/").next().unwrap_or(at.file());
        if self.unobserved_teardown(error) {
            if first.is_ok() {
                mod_pr_debug!(
                    "GPU work of a closed client failed ({:?}, reported at {}:{})\n",
                    error,
                    file,
                    at.line()
                );
            }
        } else if !self.warned.swap(true, Ordering::AcqRel) {
            pr_warn!(
                "GPU work of a client failed ({:?}, reported at {}:{}); the client sees a lost device\n",
                error,
                file,
                at.line()
            );
        }
        let error = match first {
            Ok(_) => error.to_errno(),
            Err(first) => first,
        };
        let mirror = self.mirror.lock();
        if let Some(mirror) = mirror.as_ref() {
            // SAFETY: attach_mirror stored only an aligned word within a mapping the
            // retained owner keeps alive while this entry exists under the mutex.
            unsafe { AtomicI32::from_ptr(mirror.word) }.store(error, Ordering::SeqCst);
        }
    }

    /// A teardown error of a client whose VM handle is already gone.
    fn unobserved_teardown(&self, error: Error) -> bool {
        (error == ENOENT || error == ECANCELED) && self.client_closed.load(Ordering::Acquire)
    }

    /// Rejected submissions and contention leave the VM usable.
    #[track_caller]
    pub(crate) fn record_if_device_loss(&self, error: Error) {
        if error == EIO || error == ETIMEDOUT || error == ENODEV {
            self.record(error);
        }
    }

    /// Retain one mapped GEM word until the client's VM handle is dropped.
    pub(crate) fn attach_mirror(&self, mut owner: gem::ObjectRef, offset: usize) -> Result {
        if owner.size() > MIRROR_MAX_SIZE
            || offset % core::mem::align_of::<i32>() != 0
            || offset
                .checked_add(core::mem::size_of::<i32>())
                .is_none_or(|end| end > owner.size())
        {
            return Err(EINVAL);
        }
        let base = owner.vmap()?.addr.cast::<u8>();
        // SAFETY: The checked range is inside the retained, page-aligned mapping.
        let word = unsafe { base.add(offset) }.cast::<i32>();
        let mut mirror = self.mirror.lock();
        if mirror.is_some() {
            return Err(EBUSY);
        }
        // No reporter runs while the mutex is held, so the word holds the first
        // reported error (or zero) when it becomes visible.
        let error = self.reported_error.load(Ordering::SeqCst);
        // SAFETY: The retained owner keeps this checked, aligned word mapped.
        unsafe { AtomicI32::from_ptr(word) }.store(error, Ordering::SeqCst);
        *mirror = Some(Mirror {
            word,
            _owner: owner,
        });
        Ok(())
    }

    /// The client's VM handle is gone and nobody reads the mirror any more. The
    /// mapping and object reference drop outside the mutex (GEM release may sleep).
    pub(crate) fn release_mirror(&self) {
        self.client_closed.store(true, Ordering::Release);
        let taken = self.mirror.lock().take();
        drop(taken);
    }
}

#[kernel::macros::kunit_tests(asahi_g17_vm_status)]
mod tests {
    use super::VmStatus;
    use core::sync::atomic::Ordering;
    use kernel::prelude::*;

    #[test]
    fn only_a_closed_client_hides_teardown_errors() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        assert!(!status.unobserved_teardown(ENOENT));
        assert!(!status.unobserved_teardown(ECANCELED));
        status.release_mirror();
        assert!(status.unobserved_teardown(ENOENT));
        assert!(status.unobserved_teardown(ECANCELED));
        assert!(!status.unobserved_teardown(ETIMEDOUT));
        assert!(!status.unobserved_teardown(EIO));
        assert!(!status.unobserved_teardown(ENODEV));
    }

    #[test]
    fn a_closed_client_still_keeps_its_first_error() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        status.release_mirror();
        status.report_failure(ENOENT);
        assert!(!status.warned.load(Ordering::Acquire));
        status.report_failure(ETIMEDOUT);
        assert_eq!(status.reported_error(), ENOENT.to_errno());
        assert_eq!(status.get(), 0);
        // Device loss after a hidden teardown error is still warned about.
        assert!(status.warned.load(Ordering::Acquire));
    }

    #[test]
    fn an_open_client_warns_on_its_first_error_only() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        status.report_failure(ENOENT);
        assert!(status.warned.load(Ordering::Acquire));
        assert_eq!(status.reported_error(), ENOENT.to_errno());
        status.report_failure(ETIMEDOUT);
        assert_eq!(status.reported_error(), ENOENT.to_errno());
    }
}
