// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU memory attribution, accounting and admission limits.
//!
//! The backing of a GEM object that the GPU holds is pinned shmem: it sits on the
//! unevictable LRU and cannot be swapped out. It is charged to the memory cgroup of
//! the task that faulted it in, but the OOM killer scores a task by the resident set
//! of its mm, and GPU-only pages are mapped into no process. A game holding
//! gigabytes of buffers therefore scores like a tiny process, and so does a
//! compositor; which one an OOM kill hits is left to the other terms of the score.
//!
//! This module closes that gap:
//!
//! - Every user object remembers the mm of the process that created it (an `mmgrab`
//!   reference, which keeps only the `mm_struct` itself). While the GPU holds the
//!   object's backing, its pages are added to that mm's `MM_SHMEMPAGES` counter.
//!   They are shmem pages, so this is where `oom_badness()`, the OOM report's
//!   `rss_shmem` column and `RssShmem`/`VmRSS` in `/proc/<pid>/status` look. Pages
//!   are uncharged when idle reclaim or purge releases the backing and when the
//!   object is freed; the mm reference is dropped after the last uncharge, so the
//!   counter is back to its own value before `__mmdrop()` checks it.
//! - Each client (DRM file) and the device keep totals of created and held bytes,
//!   reported in fdinfo (`drm-total-system`, `drm-resident-system`) and in the
//!   `gpu_memory` sysfs file next to `agx_stats`.
//! - A new user object is admitted only while its client stays within a per-client
//!   limit and the device within a device-wide limit, both fractions of RAM.
//!   A refused allocation fails with `ENOMEM`, which userspace reports as
//!   out-of-device-memory, instead of pushing the machine into a global OOM.
//!
//! Imported dma-bufs are charged to their exporter and are not counted here.
//! Kernel objects (firmware structures, per-queue buffers and the global heaps)
//! count towards the device total and its limit, but are attributed to no process.
//!
//! The limits and the RSS attribution apply only to G17 for now: only its probe
//! calls [`Ledger::configure`]. Other GPUs keep the totals, with no limit and no
//! attribution.

use core::sync::atomic::{
    AtomicBool,
    AtomicU64,
    Ordering, //
};

use kernel::{
    mm::{
        Mm,
        RssCounter, //
    },
    page::PAGE_SIZE,
    prelude::*,
    sync::{aref::ARef, Arc},
};

use crate::module_parameters;

/// Device-wide GPU memory totals, in bytes.
///
/// The layout is shared with the C sysfs shim (`struct asahi_gpu_memory`).
#[repr(C)]
pub(crate) struct Ledger {
    /// Size of every live user object (created, not imported).
    pub(crate) user_committed: AtomicU64,
    /// Size of the live user objects whose backing the GPU currently holds.
    pub(crate) user_resident: AtomicU64,
    /// Size of the live kernel objects whose backing the GPU holds.
    pub(crate) kernel_resident: AtomicU64,
    /// User object creations refused by a limit.
    pub(crate) refused: AtomicU64,
    /// Device-wide limit on user committed plus kernel resident bytes (0: none).
    pub(crate) limit: AtomicU64,
    /// Per-client limit on committed bytes (0: none).
    pub(crate) client_limit: AtomicU64,
}

// Keep the Rust half of the C sysfs layout contract checked as well.
const _: () = {
    assert!(core::mem::offset_of!(Ledger, refused) == 24);
    assert!(core::mem::offset_of!(Ledger, client_limit) == 40);
    assert!(core::mem::size_of::<Ledger>() == 48);
};

/// The ledger of the (single) AGX GPU.
pub(crate) static LEDGER: Ledger = Ledger::new();

/// Whether held user objects are charged to their creator's RSS. Set by [`Ledger::configure`].
static ATTRIBUTE_RSS: AtomicBool = AtomicBool::new(false);

/// The limits that follow from the amount of RAM and two percentages.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct Limits {
    /// Device-wide limit in bytes (0: none).
    pub(crate) device: u64,
    /// Per-client limit in bytes (0: none).
    pub(crate) client: u64,
}

fn pct_of(total: u64, pct: u32) -> u64 {
    match pct {
        0 => 0,
        // Above 100% cannot be held anyway; treat it as "no limit".
        p if p >= 100 => 0,
        p => total / 100 * u64::from(p) + total % 100 * u64::from(p) / 100,
    }
}

/// Computes the limits for `total_ram` bytes of RAM.
///
/// A percentage of 0 or 100 and above disables that limit. A per-client limit
/// is never larger than the device limit.
pub(crate) fn limits_for(total_ram: u64, device_pct: u32, client_pct: u32) -> Limits {
    let device = pct_of(total_ram, device_pct);
    let mut client = pct_of(total_ram, client_pct);
    if device != 0 && (client == 0 || client > device) {
        client = device;
    }
    Limits { device, client }
}

/// Adds `bytes` to `counter` unless the result would exceed `limit` (0: no limit).
///
/// Returns whether the bytes were added. Concurrent callers cannot overshoot.
fn try_add_within(counter: &AtomicU64, bytes: u64, limit: u64, extra: u64) -> bool {
    let mut cur = counter.load(Ordering::Relaxed);
    loop {
        let Some(new) = cur.checked_add(bytes) else {
            return false;
        };
        if limit != 0 && new.saturating_add(extra) > limit {
            return false;
        }
        match counter.compare_exchange_weak(cur, new, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => return true,
            Err(actual) => cur = actual,
        }
    }
}

fn sub(counter: &AtomicU64, bytes: u64) {
    let prev = counter.fetch_sub(bytes, Ordering::AcqRel);
    if prev < bytes {
        pr_warn!("asahi: GPU memory accounting underflow ({} < {})\n", prev, bytes);
        counter.store(0, Ordering::Relaxed);
    }
}

impl Ledger {
    pub(crate) const fn new() -> Self {
        Self {
            user_committed: AtomicU64::new(0),
            user_resident: AtomicU64::new(0),
            kernel_resident: AtomicU64::new(0),
            refused: AtomicU64::new(0),
            limit: AtomicU64::new(0),
            client_limit: AtomicU64::new(0),
        }
    }

    /// Sets the limits and the RSS attribution from the module parameters and the amount of RAM.
    pub(crate) fn configure(&self) {
        // SAFETY: `struct sysinfo` is plain integers, for which all zeroes is a valid value.
        let mut info: kernel::bindings::sysinfo = unsafe { core::mem::zeroed() };
        // SAFETY: `si_meminfo()` only fills in the structure it is given.
        unsafe { kernel::bindings::si_meminfo(&mut info) };
        let total = (info.totalram as u64).saturating_mul(u64::from(info.mem_unit));
        let limits = limits_for(
            total,
            *module_parameters::mem_limit_pct.value(),
            *module_parameters::client_mem_limit_pct.value(),
        );
        self.set_limits(limits);
        ATTRIBUTE_RSS.store(
            *module_parameters::oom_attribution.value() != 0,
            Ordering::Relaxed,
        );
        pr_info!(
            "asahi: GPU memory limits: device {} MiB, per client {} MiB (0 = none)\n",
            limits.device >> 20,
            limits.client >> 20
        );
    }

    /// Admission capacities exported in the optional GET_PARAMS tail.
    pub(crate) fn limits(&self) -> Limits {
        Limits {
            device: self.limit.load(Ordering::Relaxed),
            client: self.client_limit.load(Ordering::Relaxed),
        }
    }

    pub(crate) fn set_limits(&self, limits: Limits) {
        self.limit.store(limits.device, Ordering::Relaxed);
        self.client_limit.store(limits.client, Ordering::Relaxed);
    }

    /// Admits a new user object of `bytes` for `client`.
    ///
    /// Reserves the bytes against both limits; the returned [`Commit`] gives
    /// them back when it is dropped.
    pub(crate) fn admit(&'static self, client: &Arc<Client>, bytes: u64) -> Result<Commit> {
        let client_limit = self.client_limit.load(Ordering::Relaxed);
        if !try_add_within(&client.committed, bytes, client_limit, 0) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return Err(ENOMEM);
        }
        // Kernel objects share the device limit but are never refused.
        let kernel = self.kernel_resident.load(Ordering::Relaxed);
        if !try_add_within(
            &self.user_committed,
            bytes,
            self.limit.load(Ordering::Relaxed),
            kernel,
        ) {
            sub(&client.committed, bytes);
            self.refused.fetch_add(1, Ordering::Relaxed);
            return Err(ENOMEM);
        }
        Ok(Commit {
            ledger: self,
            client: client.clone(),
            bytes,
        })
    }
}

/// Per-client (DRM file) totals, in bytes.
pub(crate) struct Client {
    /// Size of the live user objects this client created.
    committed: AtomicU64,
    /// Size of those whose backing the GPU currently holds.
    resident: AtomicU64,
}

impl Client {
    pub(crate) fn new() -> Result<Arc<Self>> {
        Ok(Arc::new(
            Self {
                committed: AtomicU64::new(0),
                resident: AtomicU64::new(0),
            },
            GFP_KERNEL,
        )?)
    }

    pub(crate) fn committed(&self) -> u64 {
        self.committed.load(Ordering::Relaxed)
    }

    pub(crate) fn resident(&self) -> u64 {
        self.resident.load(Ordering::Relaxed)
    }
}

/// Bytes of a user object reserved against the limits.
pub(crate) struct Commit {
    ledger: &'static Ledger,
    client: Arc<Client>,
    bytes: u64,
}

impl Drop for Commit {
    fn drop(&mut self) {
        sub(&self.client.committed, self.bytes);
        sub(&self.ledger.user_committed, self.bytes);
    }
}

enum Kind {
    /// An object the DRM core created (a dma-buf import): charged to its exporter.
    None,
    /// A driver-owned object.
    Kernel(&'static Ledger),
    /// A user object, attributed to the process that created it.
    User {
        commit: Commit,
        mm: Option<ARef<Mm>>,
    },
}

/// The accounting state of one GEM object.
pub(crate) struct Charge {
    kind: Kind,
    bytes: u64,
    /// Whether the bytes are currently counted as held by the GPU.
    held: AtomicBool,
}

// SAFETY: `ARef<Mm>` may be dropped on any thread and the counters are atomic.
unsafe impl Send for Charge {}
// SAFETY: Shared access only touches atomics and the per-cpu RSS counters.
unsafe impl Sync for Charge {}

impl Charge {
    /// An object that is not counted (a dma-buf import).
    pub(crate) fn none() -> Self {
        Self {
            kind: Kind::None,
            bytes: 0,
            held: AtomicBool::new(false),
        }
    }

    /// A kernel object of `bytes`.
    pub(crate) fn kernel(ledger: &'static Ledger, bytes: u64) -> Self {
        Self {
            kind: Kind::Kernel(ledger),
            bytes,
            held: AtomicBool::new(false),
        }
    }

    /// A user object admitted by `commit`, attributed to `mm` if there is one.
    pub(crate) fn user(commit: Commit, mm: Option<ARef<Mm>>) -> Self {
        let bytes = commit.bytes;
        Self {
            kind: Kind::User { commit, mm },
            bytes,
            held: AtomicBool::new(false),
        }
    }

    /// A user object attributed to the current process, if the ledger attributes RSS.
    pub(crate) fn user_current(commit: Commit) -> Self {
        let mm = if ATTRIBUTE_RSS.load(Ordering::Relaxed) {
            kernel::current!().mm().map(|mm| ARef::from(&**mm))
        } else {
            None
        };
        Self::user(commit, mm)
    }

    fn pages(&self) -> isize {
        isize::try_from(self.bytes / PAGE_SIZE as u64).unwrap_or(isize::MAX)
    }

    fn apply(&self, held: bool) {
        match &self.kind {
            Kind::None => {}
            Kind::Kernel(ledger) => {
                if held {
                    ledger.kernel_resident.fetch_add(self.bytes, Ordering::AcqRel);
                } else {
                    sub(&ledger.kernel_resident, self.bytes);
                }
            }
            Kind::User { commit, mm } => {
                if held {
                    commit.client.resident.fetch_add(self.bytes, Ordering::AcqRel);
                    commit
                        .ledger
                        .user_resident
                        .fetch_add(self.bytes, Ordering::AcqRel);
                } else {
                    sub(&commit.client.resident, self.bytes);
                    sub(&commit.ledger.user_resident, self.bytes);
                }
                if let Some(mm) = mm {
                    let pages = self.pages();
                    mm.add_rss_counter(RssCounter::Shmem, if held { pages } else { -pages });
                }
            }
        }
    }

    /// Records that the GPU now holds (or no longer holds) the object's backing.
    ///
    /// Idempotent: only a change of state is counted.
    pub(crate) fn set_held(&self, held: bool) {
        if self.held.swap(held, Ordering::AcqRel) != held {
            self.apply(held);
        }
    }
}

impl Drop for Charge {
    fn drop(&mut self) {
        // Uncharge before `kind` (and with it the mm reference) is dropped.
        self.set_held(false);
    }
}

#[kunit_tests(asahi_memacct)]
mod tests {
    use super::*;

    const GIB: u64 = 1 << 30;
    const MIB: u64 = 1 << 20;

    #[test]
    fn default_limits_follow_ram() {
        // The module parameter defaults.
        let l = limits_for(8 * GIB, 67, 50);
        assert_eq!(l.device, 8 * GIB * 67 / 100);
        assert_eq!(l.client, 4 * GIB);
    }

    #[test]
    fn disabled_and_clamped_limits() {
        assert_eq!(limits_for(8 * GIB, 0, 0), Limits { device: 0, client: 0 });
        assert_eq!(limits_for(8 * GIB, 100, 0), Limits { device: 0, client: 0 });
        // A client limit above the device limit is clamped to it.
        let l = limits_for(8 * GIB, 25, 50);
        assert_eq!(l.client, l.device);
        // No client limit with a device limit means the device limit.
        let l = limits_for(8 * GIB, 50, 0);
        assert_eq!(l.client, 4 * GIB);
        // No device limit leaves the client limit alone.
        let l = limits_for(8 * GIB, 0, 25);
        assert_eq!(l, Limits { device: 0, client: 2 * GIB });
    }

    #[test]
    fn percentages_do_not_overflow() {
        let l = limits_for(u64::MAX, 99, 98);
        assert!(l.device > l.client);
        assert!(l.device <= u64::MAX / 100 * 99 + 99);
    }

    #[test]
    fn try_add_respects_limit_and_extra() {
        let c = AtomicU64::new(0);
        assert!(try_add_within(&c, 6, 10, 0));
        assert!(!try_add_within(&c, 5, 10, 0));
        assert!(try_add_within(&c, 4, 10, 0));
        assert_eq!(c.load(Ordering::Relaxed), 10);
        let d = AtomicU64::new(0);
        assert!(!try_add_within(&d, 6, 10, 5));
        assert!(try_add_within(&d, 5, 10, 5));
        let e = AtomicU64::new(u64::MAX - 1);
        assert!(!try_add_within(&e, 2, 0, 0));
        assert!(try_add_within(&e, 1, 0, 0));
    }

    static ADMIT: Ledger = Ledger::new();

    #[test]
    fn admission_reserves_and_releases() -> Result {
        ADMIT.set_limits(Limits { device: 100 * MIB, client: 60 * MIB });
        let a = Client::new()?;
        let b = Client::new()?;
        let c = Client::new()?;
        let a1 = ADMIT.admit(&a, 40 * MIB)?;
        // The client limit refuses a's second 40 MiB; nothing stays reserved.
        assert!(ADMIT.admit(&a, 40 * MIB).is_err());
        assert_eq!(a.committed(), 40 * MIB);
        assert_eq!(ADMIT.user_committed.load(Ordering::Relaxed), 40 * MIB);
        let b1 = ADMIT.admit(&b, 50 * MIB)?;
        // The device limit refuses c's 20 MiB and rolls back c's reservation.
        assert!(ADMIT.admit(&c, 20 * MIB).is_err());
        assert_eq!(c.committed(), 0);
        assert_eq!(ADMIT.user_committed.load(Ordering::Relaxed), 90 * MIB);
        assert_eq!(ADMIT.refused.load(Ordering::Relaxed), 2);
        drop(a1);
        assert_eq!(a.committed(), 0);
        let c1 = ADMIT.admit(&c, 20 * MIB)?;
        drop((b1, c1));
        assert_eq!(b.committed(), 0);
        assert_eq!(c.committed(), 0);
        assert_eq!(ADMIT.user_committed.load(Ordering::Relaxed), 0);
        Ok(())
    }

    static KERNEL: Ledger = Ledger::new();

    #[test]
    fn kernel_objects_count_against_the_device_limit() -> Result {
        KERNEL.set_limits(Limits { device: 64 * MIB, client: 64 * MIB });
        let k = Charge::kernel(&KERNEL, 48 * MIB);
        k.set_held(true);
        let a = Client::new()?;
        assert!(KERNEL.admit(&a, 32 * MIB).is_err());
        let c = KERNEL.admit(&a, 16 * MIB)?;
        drop(k);
        assert_eq!(KERNEL.kernel_resident.load(Ordering::Relaxed), 0);
        drop(c);
        Ok(())
    }

    static HELD: Ledger = Ledger::new();

    #[test]
    fn held_transitions_are_idempotent() -> Result {
        HELD.set_limits(Limits { device: 0, client: 0 });
        let a = Client::new()?;
        let charge = Charge::user(HELD.admit(&a, 8 * MIB)?, None);
        assert!(!charge.held.load(Ordering::Relaxed));
        charge.set_held(true);
        charge.set_held(true);
        assert_eq!(a.resident(), 8 * MIB);
        assert_eq!(HELD.user_resident.load(Ordering::Relaxed), 8 * MIB);
        charge.set_held(false);
        charge.set_held(false);
        assert_eq!(a.resident(), 0);
        charge.set_held(true);
        // Dropping a held object uncharges it and returns its commit.
        drop(charge);
        assert_eq!(a.resident(), 0);
        assert_eq!(a.committed(), 0);
        assert_eq!(HELD.user_resident.load(Ordering::Relaxed), 0);
        assert_eq!(HELD.user_committed.load(Ordering::Relaxed), 0);
        Ok(())
    }

    #[test]
    fn imports_are_not_counted() {
        let charge = Charge::none();
        charge.set_held(true);
        assert!(charge.held.load(Ordering::Relaxed));
        drop(charge);
    }

    static MM: Ledger = Ledger::new();

    // `mm_alloc()` is only exported to modules in the KUnit namespace.
    #[cfg(not(MODULE))]
    #[test]
    fn held_pages_are_charged_to_the_creating_mm() -> Result {
        MM.set_limits(Limits { device: 0, client: 0 });
        // SAFETY: `mm_alloc()` returns a new mm with one user and one count, or NULL.
        let raw = unsafe { kernel::bindings::mm_alloc() };
        if raw.is_null() {
            return Err(ENOMEM);
        }
        // SAFETY: `raw` is a live mm until the `mmput()` below.
        let mm = ARef::<Mm>::from(unsafe { Mm::from_raw(raw) });
        let a = Client::new()?;
        let charge = Charge::user(MM.admit(&a, 4 * PAGE_SIZE as u64)?, Some(mm.clone()));
        assert_eq!(mm.rss_counter(RssCounter::Shmem), 0);
        charge.set_held(true);
        assert_eq!(mm.rss_counter(RssCounter::Shmem), 4);
        assert_eq!(mm.rss_counter(RssCounter::Anon), 0);
        charge.set_held(false);
        assert_eq!(mm.rss_counter(RssCounter::Shmem), 0);
        charge.set_held(true);
        // Freeing a held object uncharges it before dropping its mm reference.
        drop(charge);
        assert_eq!(mm.rss_counter(RssCounter::Shmem), 0);
        drop(mm);
        // SAFETY: Drops the user reference `mm_alloc()` returned. `__mmdrop()` then
        // checks that every RSS counter is back to zero.
        unsafe { kernel::bindings::mmput(raw) };
        Ok(())
    }
}

#[kernel::macros::kunit_tests(asahi_gem_memory_limits)]
mod exported_limits_tests {
    use super::*;

    #[test]
    fn exported_capacities_match_admission() {
        let ledger = Ledger::new();
        assert_eq!(ledger.limits(), Limits { device: 0, client: 0 });
        let limits = limits_for(8 << 30, 67, 50);
        ledger.set_limits(limits);
        assert_eq!(ledger.limits(), limits);
        assert!(limits.client < limits.device);
        assert_eq!(limits.client, 4 << 30);
    }
}
