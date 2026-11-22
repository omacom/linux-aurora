// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! VM admission errors and client failure reporting. No firmware state is accessed here.

use crate::debug::*;
use crate::gem;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use kernel::{new_mutex, prelude::*, str::BStr, sync::Mutex, task::Pid};

const DEBUG_CLASS: DebugFlags = DebugFlags::Errors;

const MIRROR_MAX_SIZE: usize = 64 * 1024;

/// [`VmStatus::restarts_left`] of a VM no restart has been charged to.
const UNCHARGED: u32 = u32::MAX;

/// A VM and its accepted jobs share this status, including after queue destruction.
/// Client failure is separate from hardware admission: recovery may preserve the
/// VM while failed accepted work must still be reported before signalling fences.
#[pin_data]
pub(crate) struct VmStatus {
    error: AtomicI32,
    reported_error: AtomicI32,
    /// The client's VM handle is gone: no client can see a lost device any more.
    client_closed: AtomicBool,
    /// A queue of this VM went away while its work was still in the GPU, and the wait for that
    /// work ended. Never cleared.
    teardown_expired: AtomicBool,
    /// A failure of this VM has been logged as a warning.
    warned: AtomicBool,
    /// A GPU recovery found this VM's work guilty.
    recovery_guilty: AtomicBool,
    /// Further restarts that may find this VM guilty and still be charged to it
    /// ([`Self::charge_restart`]), or [`UNCHARGED`] before the first.
    restarts_left: AtomicU32,
    /// The process that created the VM, named in the failure report.
    tgid: Pid,
    comm: [u8; 16],
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
    /// Called from the creating client's ioctl: the VM records who it belongs to.
    pub(crate) fn new() -> impl PinInit<Self> {
        let task = kernel::current!();
        // SAFETY: `comm` is a fixed array inside the live current task. A concurrent rename by
        // another thread of the process may tear the copy, as with C's unlocked `current->comm`.
        let comm: [u8; 16] = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!((*task.as_ptr()).comm).cast())
        };
        let tgid = task.tgid();
        pin_init!(Self {
            error: AtomicI32::new(0),
            reported_error: AtomicI32::new(0),
            client_closed: AtomicBool::new(false),
            teardown_expired: AtomicBool::new(false),
            warned: AtomicBool::new(false),
            recovery_guilty: AtomicBool::new(false),
            restarts_left: AtomicU32::new(UNCHARGED),
            tgid,
            comm,
            mirror <- new_mutex!(None, "Asahi VM status mirror"),
        })
    }

    pub(crate) fn get(&self) -> i32 {
        self.error.load(Ordering::Acquire)
    }

    /// The process that created the VM: its name and thread-group ID.
    pub(crate) fn client(&self) -> (&BStr, Pid) {
        let len = self
            .comm
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.comm.len());
        (BStr::from_bytes(&self.comm[..len]), self.tgid)
    }

    pub(crate) fn note_recovery_guilty(&self) {
        self.recovery_guilty.store(true, Ordering::Release);
    }

    /// A GPU recovery found this VM guilty: it hung or faulted the GPU, unlike a VM that
    /// failed only for its own client.
    pub(crate) fn recovery_guilty(&self) -> bool {
        self.recovery_guilty.load(Ordering::Acquire)
    }

    /// Charges a firmware restart that found this VM guilty to it, with `in_flight` of its
    /// commands still held then. The first such restart costs its client the device. After it the
    /// VM publishes nothing new, but each command it had in flight may still hang or fault the
    /// GPU once as it runs. Returns false once restarts have found the VM guilty more often than
    /// that: the firmware keeps blaming it for something it no longer runs. Called only from a
    /// recovery, under the device mutex.
    pub(crate) fn charge_restart(&self, in_flight: u32) -> bool {
        let left = match self.restarts_left.load(Ordering::Relaxed) {
            UNCHARGED => in_flight.min(UNCHARGED - 1),
            0 => return false,
            left => left - 1,
        };
        self.restarts_left.store(left, Ordering::Relaxed);
        true
    }

    /// The client's VM handle is gone (`release_mirror`): the client submits nothing more.
    pub(crate) fn client_closed(&self) -> bool {
        self.client_closed.load(Ordering::Acquire)
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
    /// Teardown of a closed client fails its queued work with ENOENT (VM context gone),
    /// ECANCELED (queue destroyed) or, when the wait for work still in the GPU ends
    /// ([`Self::note_teardown_expired`]), ETIMEDOUT; nobody can see that, so it is only a debug
    /// message, and the first other failure of that VM, such as device loss (EIO, ENODEV, or
    /// ETIMEDOUT while no teardown wait of the VM has ended), is still logged as a warning.
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
            let len = self.comm.iter().position(|&b| b == 0).unwrap_or(self.comm.len());
            pr_warn!(
                "GPU work of a client failed ({:?}, reported at {}:{}, client {}[{}]); the client sees a lost device\n",
                error,
                file,
                at.line(),
                BStr::from_bytes(&self.comm[..len]),
                self.tgid
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
    pub(crate) fn unobserved_teardown(&self, error: Error) -> bool {
        self.client_closed.load(Ordering::Acquire)
            && (error == ENOENT
                || error == ECANCELED
                || (error == ETIMEDOUT && self.teardown_expired.load(Ordering::Acquire)))
    }

    /// A queue teardown stopped waiting for this VM's work, which now fails with ETIMEDOUT.
    pub(crate) fn note_teardown_expired(&self) {
        self.teardown_expired.store(true, Ordering::Release);
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

    /// The client closed its DRM file: nobody sees the teardown errors of its queued work any
    /// more, even before its VM handle goes (the file drops its queues first).
    pub(crate) fn mark_client_closed(&self) {
        self.client_closed.store(true, Ordering::Release);
    }

    /// The client's VM handle is gone and nobody reads the mirror any more. The
    /// mapping and object reference drop outside the mutex (GEM release may sleep).
    pub(crate) fn release_mirror(&self) {
        self.mark_client_closed();
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
    fn a_closed_client_hides_the_end_of_its_teardown_wait() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        // A client that only destroyed a queue still sees the failure.
        status.note_teardown_expired();
        assert!(!status.unobserved_teardown(ETIMEDOUT));
        status.mark_client_closed();
        assert!(status.unobserved_teardown(ETIMEDOUT));
        assert!(!status.unobserved_teardown(EIO));
        status.report_failure(ETIMEDOUT);
        assert!(!status.warned.load(Ordering::Acquire));
    }

    #[test]
    fn a_closing_file_hides_teardown_errors_before_its_vm_goes() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        status.mark_client_closed();
        assert!(status.unobserved_teardown(ENOENT));
        assert!(status.unobserved_teardown(ECANCELED));
        assert!(!status.unobserved_teardown(ETIMEDOUT));
        status.report_failure(ENOENT);
        assert!(!status.warned.load(Ordering::Acquire));
        assert!(status.mirror.lock().is_none());
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
    fn restarts_are_charged_to_a_vm_only_for_the_work_it_had_in_flight() {
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        // The first restart is charged whatever the VM holds, then one per command it held.
        assert!(status.charge_restart(3));
        for _ in 0..3 {
            assert!(status.charge_restart(5));
        }
        assert!(!status.charge_restart(5));
        assert!(!status.charge_restart(0));
        let Ok(status) = KBox::pin_init(VmStatus::new(), GFP_KERNEL) else {
            panic!("allocation failed");
        };
        assert!(status.charge_restart(0));
        assert!(!status.charge_restart(1));
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
