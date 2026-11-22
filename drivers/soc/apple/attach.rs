// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! The initrd attach phase and its real-root completion.

use kernel::{prelude::*, sync::Arc};

use crate::SepData;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Phase {
    Normal,
    KeyStoreOnly,
    Complete,
}

impl Phase {
    pub(crate) fn new(keystore_only: bool) -> Self {
        if keystore_only {
            Self::KeyStoreOnly
        } else {
            Self::Normal
        }
    }

    pub(crate) fn needs_completion(self) -> Result<bool> {
        match self {
            Self::Normal => Err(EINVAL),
            Self::KeyStoreOnly => Ok(true),
            Self::Complete => Ok(false),
        }
    }

    /// Serialize this under attach_phase. A failed operation stays retryable;
    /// success never repeats the operation, even with concurrent sysfs writers.
    pub(crate) fn complete(&mut self, operation: impl FnOnce() -> Result<()>) -> Result<()> {
        if self.needs_completion()? {
            operation()?;
            *self = Self::Complete;
        }
        Ok(())
    }
}

/// An enclave success cannot override a refused persistent-state write.
pub(crate) fn unseal_allowed(before: u32, after: u32) -> bool {
    before == after
}

extern "C" {
    fn sep_keystore_only() -> bool;
    fn sep_keystore_user_bags() -> bool;
    fn sep_complete_attach_register(
        dev: *mut c_void,
        ctx: *mut c_void,
        complete: unsafe extern "C" fn(*mut c_void) -> c_int,
    ) -> c_int;
    fn sep_complete_attach_unregister(dev: *mut c_void);
}

pub(crate) fn requested() -> bool {
    // SAFETY: the parameter is immutable after module loading.
    unsafe { sep_keystore_only() }
}

/// `keystore_only=2`: the initrd also unlocks per-user key bags, so that a
/// volume sealed to a user's password can open before the real root.
pub(crate) fn user_bags_requested() -> bool {
    // SAFETY: the parameter is immutable after module loading.
    unsafe { sep_keystore_user_bags() }
}

/// Whether the initrd phase may let the enclave write the gigalocker. A
/// per-user bag cannot be loaded or unlocked without those writes, but the
/// xART guard keeps its record list in host files on the real root, which
/// the initrd must not write. So writes are allowed only for user bags and
/// only when no guard runs; with a guard the bags wait for complete_attach.
pub(crate) fn initrd_xart_writes(user_bags: bool, guard: Option<u8>) -> bool {
    user_bags && guard.is_none()
}

/// Holds the callback context alive until sysfs has drained all writers.
pub(crate) struct Attribute {
    sep: Arc<SepData>,
    registered: bool,
}

impl Attribute {
    pub(crate) fn register(sep: Arc<SepData>) -> Result<Self> {
        // SAFETY: the owned Arc pins the context until unregister has drained
        // sysfs callbacks. The shim keeps no pointer after unregister returns.
        kernel::error::to_result(unsafe {
            sep_complete_attach_register(
                sep.dev.as_raw().cast(),
                Arc::as_ptr(&sep).cast_mut().cast(),
                complete,
            )
        })?;
        Ok(Self {
            sep,
            registered: true,
        })
    }

    pub(crate) fn unregister(&mut self) {
        if self.registered {
            // SAFETY: matched to the successful register and called only once.
            unsafe { sep_complete_attach_unregister(self.sep.dev.as_raw().cast()) };
            self.registered = false;
        }
    }
}

impl Drop for Attribute {
    fn drop(&mut self) {
        self.unregister();
    }
}

/// # Safety
/// The shim calls with the context pinned by Attribute, under sysfs active
/// protection; unregister waits for this callback before releasing the Arc.
unsafe extern "C" fn complete(ctx: *mut c_void) -> c_int {
    // SAFETY: guaranteed by the registration above.
    let sep = unsafe { &*ctx.cast::<SepData>() };
    match sep.complete_attach() {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

#[kernel::macros::kunit_tests(apple_sep_attach)]
mod tests {
    use super::*;

    #[test]
    fn normal_attach_rejects_completion() {
        let mut phase = Phase::new(false);
        assert_eq!(
            phase.complete(|| panic!("must not touch stores")),
            Err(EINVAL)
        );
    }

    #[test]
    fn completion_is_idempotent() {
        let mut phase = Phase::new(true);
        let mut calls = 0;
        assert_eq!(
            phase.complete(|| {
                calls += 1;
                Ok(())
            }),
            Ok(())
        );
        assert_eq!(phase.complete(|| panic!("must not repeat attach")), Ok(()));
        assert_eq!(calls, 1);
    }

    #[test]
    fn failed_completion_can_retry() {
        let mut phase = Phase::new(true);
        assert_eq!(phase.complete(|| Err(EIO)), Err(EIO));
        assert_eq!(phase.needs_completion(), Ok(true));
        assert_eq!(phase.complete(|| Ok(())), Ok(()));
        assert_eq!(phase.needs_completion(), Ok(false));
    }

    #[test]
    fn initrd_writes_only_for_user_bags_without_a_guard() {
        assert!(initrd_xart_writes(true, None));
        assert!(!initrd_xart_writes(false, None));
        assert!(!initrd_xart_writes(true, Some(1)));
        assert!(!initrd_xart_writes(true, Some(2)));
    }

    #[test]
    fn refused_initrd_write_fails_unseal_closed() {
        assert!(unseal_allowed(0, 0));
        assert!(unseal_allowed(17, 17));
        assert!(!unseal_allowed(0, 1));
        assert!(!unseal_allowed(u32::MAX, 0));
    }
}
