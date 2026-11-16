// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Detects sustained host refill gaps and temporarily requests CPU capacity.
//! Detector updates are lock-free. The worker and policy callbacks serialize
//! frequency requests without nesting their mutex inside a publication lock.
//! Shutdown first closes the detector, joins its producers and the worker,
//! then removes requests. All request storage and policy references are owned
//! by this device, independently of other GPU instances.
//! Policy creation and the initial snapshot take the policy rwsem before the
//! requests mutex. Removal callbacks and the worker take only requests;
//! notifier unregistration joins callbacks without holding that mutex.

use crate::hw::t8140::qos::CLOCK_HZ;
use core::{
    pin::Pin,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
};
use kernel::{
    prelude::*,
    sync::Arc,
    time::{delay::fsleep, ClockSource, Delta, Monotonic},
};

#[cfg(CONFIG_CPU_FREQ)]
use core::ptr;
#[cfg(CONFIG_CPU_FREQ)]
use kernel::{bindings, new_mutex, sync::Mutex, types::Opaque};

const MAX_REFILL_TICKS: u64 = CLOCK_HZ / 500;
const STREAK_ON: u32 = 8;
const STREAK_MAX: u32 = 64;
const LINGER_NS: u64 = 8_000_000;
const POLL_MS: i64 = 2;
#[cfg(CONFIG_CPU_FREQ)]
const FLOOR_PERCENT: u64 = 50;
const WAIT_UTIL_MIN: u32 = 512;
const WAIT_HINT_MS: u32 = 8;
#[cfg(CONFIG_CPU_FREQ)]
const MAX_POLICIES: usize = 8;

struct Detector {
    last_end: AtomicU64,
    idle: AtomicU64,
    busy: AtomicU64,
    streak: AtomicU32,
    until: AtomicU64,
    active: AtomicBool,
    pending: AtomicBool,
    stopping: AtomicBool,
}

impl Detector {
    const fn new() -> Self {
        Self {
            last_end: AtomicU64::new(0),
            idle: AtomicU64::new(0),
            busy: AtomicU64::new(0),
            streak: AtomicU32::new(0),
            until: AtomicU64::new(0),
            active: AtomicBool::new(false),
            pending: AtomicBool::new(false),
            stopping: AtomicBool::new(false),
        }
    }

    fn decay_add(sum: &AtomicU64, add: u64) -> u64 {
        let mut next = 0;
        let _ = sum.fetch_update(Ordering::AcqRel, Ordering::Acquire, |value| {
            next = value - value / 8 + add;
            Some(next)
        });
        next
    }

    fn note(&self, start: u64, end: u64, now: u64) {
        if end <= start {
            return;
        }
        let previous = self.last_end.fetch_max(end, Ordering::AcqRel);
        let (idle, busy, paced) = if previous == 0 {
            (0, end - start, true)
        } else if start >= previous {
            let gap = start - previous;
            if gap < MAX_REFILL_TICKS {
                (gap, end - start, false)
            } else {
                (0, end - start, true)
            }
        } else if end > previous {
            (0, end - previous, false)
        } else {
            (0, 0, false)
        };
        let idle_sum = Self::decay_add(&self.idle, idle);
        let busy_sum = Self::decay_add(&self.busy, busy);
        let streak = if paced {
            self.streak.store(0, Ordering::Release);
            0
        } else {
            let previous = self
                .streak
                .fetch_update(Ordering::AcqRel, Ordering::Acquire, |value| {
                    Some((value + 1).min(STREAK_MAX))
                });
            // The closure always returns Some; fetch_update retries contention.
            match previous {
                Ok(value) => value + 1,
                Err(_) => 0,
            }
        };
        if streak >= STREAK_ON && idle_sum.saturating_mul(4) >= busy_sum {
            self.until.store(now + LINGER_NS, Ordering::Release);
            if !self.active.swap(true, Ordering::AcqRel) {
                self.pending.store(true, Ordering::Release);
            }
        }
    }

    fn wanted(&self, now: u64) -> bool {
        !self.stopping.load(Ordering::Acquire) && now < self.until.load(Ordering::Acquire)
    }
}

#[cfg(CONFIG_CPU_FREQ)]
#[pin_data]
struct Requests {
    #[pin]
    requests: Opaque<[bindings::freq_qos_request; MAX_POLICIES]>,
    policies: [*mut bindings::cpufreq_policy; MAX_POLICIES],
    maxima: [u32; MAX_POLICIES],
    applied: [u32; MAX_POLICIES],
}

// SAFETY: Each nonnull policy owns a kobject reference. The device requests
// mutex serializes the pinned slots with the worker and policy callbacks.
#[cfg(CONFIG_CPU_FREQ)]
unsafe impl Send for Requests {}

#[cfg(CONFIG_CPU_FREQ)]
impl Requests {
    fn new() -> Self {
        Self {
            requests: Opaque::zeroed(),
            policies: [ptr::null_mut(); MAX_POLICIES],
            maxima: [0; MAX_POLICIES],
            applied: [0; MAX_POLICIES],
        }
    }

    /// Add a policy to the pinned requests.
    ///
    /// # Safety
    ///
    /// The caller holds the policy's read or write lock and keeps it alive.
    /// Policy removal must subsequently pass through this device's notifier.
    unsafe fn add(self: Pin<&mut Self>, policy: *mut bindings::cpufreq_policy) {
        let this = self.project();
        if this.policies.contains(&policy) {
            return;
        }
        let Some(index) = this.policies.iter().position(|entry| entry.is_null()) else {
            return;
        };
        // SAFETY: The caller keeps the initialized policy alive. The vacant
        // request slot is pinned and exclusively owned under the requests mutex.
        let result = unsafe {
            bindings::freq_qos_add_request(
                ptr::addr_of_mut!((*policy).constraints),
                this.requests
                    .get()
                    .cast::<bindings::freq_qos_request>()
                    .add(index),
                bindings::freq_qos_req_type_FREQ_QOS_MIN,
                bindings::FREQ_QOS_MIN_DEFAULT_VALUE as i32,
            )
        };
        if result >= 0 {
            // SAFETY: The caller's policy reference or CREATE callback keeps the
            // kobject alive. Retain it until REMOVE or device shutdown removes
            // this request; cpuinfo is protected by the caller's policy lock.
            unsafe {
                bindings::kobject_get(ptr::addr_of_mut!((*policy).kobj));
                this.maxima[index] = (*policy).cpuinfo.max_freq;
            }
            this.policies[index] = policy;
            this.applied[index] = bindings::FREQ_QOS_MIN_DEFAULT_VALUE;
        }
    }

    fn apply(self: Pin<&mut Self>, enabled: bool) -> bool {
        let this = self.project();
        let mut applied = false;
        for index in 0..MAX_POLICIES {
            if this.policies[index].is_null() {
                continue;
            }
            let value = if enabled {
                (u64::from(this.maxima[index]) * FLOOR_PERCENT / 100) as u32
            } else {
                bindings::FREQ_QOS_MIN_DEFAULT_VALUE
            };
            if this.applied[index] != value {
                // SAFETY: This slot is registered, pinned and exclusively locked;
                // the policy reference keeps its constraints alive through REMOVE.
                let result = unsafe {
                    bindings::freq_qos_update_request(
                        this.requests
                            .get()
                            .cast::<bindings::freq_qos_request>()
                            .add(index),
                        value as i32,
                    )
                };
                if result >= 0 {
                    this.applied[index] = value;
                }
            }
            applied |= this.applied[index] != bindings::FREQ_QOS_MIN_DEFAULT_VALUE;
        }
        applied
    }

    fn remove(self: Pin<&mut Self>, policy: *mut bindings::cpufreq_policy) {
        let this = self.project();
        let Some(index) = this.policies.iter().position(|entry| *entry == policy) else {
            return;
        };
        // SAFETY: This registered pinned slot is exclusively locked. The policy
        // cannot finish removal until this reference is dropped. Keep all other
        // request slots at their original addresses when releasing this slot.
        unsafe {
            bindings::freq_qos_remove_request(
                this.requests
                    .get()
                    .cast::<bindings::freq_qos_request>()
                    .add(index),
            );
            bindings::cpufreq_cpu_put(policy);
        }
        this.policies[index] = ptr::null_mut();
        this.maxima[index] = 0;
        this.applied[index] = bindings::FREQ_QOS_MIN_DEFAULT_VALUE;
    }

    fn clear(mut self: Pin<&mut Self>) {
        for index in 0..MAX_POLICIES {
            let policy = self.policies[index];
            if !policy.is_null() {
                self.as_mut().remove(policy);
            }
        }
    }
}

/// Only the blocking notifier core accesses this block after registration.
#[cfg(CONFIG_CPU_FREQ)]
#[pin_data]
struct PolicyNotifier {
    #[pin]
    block: Opaque<bindings::notifier_block>,
}

// SAFETY: The pinned notifier is registered once before publishing Feed and
// unregistered at shutdown. The blocking notifier core serializes its links;
// callbacks use only Feed's atomic state and request mutex.
#[cfg(CONFIG_CPU_FREQ)]
unsafe impl Send for PolicyNotifier {}
// SAFETY: As above; no Rust access mutates the registered block.
#[cfg(CONFIG_CPU_FREQ)]
unsafe impl Sync for PolicyNotifier {}

#[pin_data(PinnedDrop)]
pub(crate) struct Feed {
    detector: Detector,
    #[cfg(CONFIG_CPU_FREQ)]
    #[pin]
    requests: Mutex<Requests>,
    #[cfg(CONFIG_CPU_FREQ)]
    #[pin]
    notifier: PolicyNotifier,
    #[cfg(CONFIG_CPU_FREQ)]
    registered: AtomicBool,
    applied: AtomicBool,
}

impl Feed {
    pub(crate) fn new() -> Result<Arc<Self>> {
        let feed = Arc::pin_init(
            pin_init!(Self {
                detector: Detector::new(),
                #[cfg(CONFIG_CPU_FREQ)]
                requests <- new_mutex!(Requests::new(), "G17 CPU feed"),
                #[cfg(CONFIG_CPU_FREQ)]
                notifier <- pin_init!(PolicyNotifier {
                    block: Opaque::new(bindings::notifier_block {
                        notifier_call: Some(Self::policy_event),
                        next: ptr::null_mut(),
                        priority: 0,
                    }),
                }),
                #[cfg(CONFIG_CPU_FREQ)]
                registered: AtomicBool::new(false),
                applied: AtomicBool::new(false),
            }),
            GFP_KERNEL,
        )?;
        #[cfg(CONFIG_CPU_FREQ)]
        feed.register_policies();
        Ok(feed)
    }

    /// Seed existing policies once; later creation/removal is event driven.
    #[cfg(CONFIG_CPU_FREQ)]
    fn register_policies(&self) {
        // SAFETY: Feed is pinned in its Arc and has not been published. The
        // notifier block lives until shutdown unregisters and joins callbacks.
        let result = unsafe {
            bindings::cpufreq_register_notifier(
                self.notifier.block.get(),
                bindings::CPUFREQ_POLICY_NOTIFIER,
            )
        };
        if result < 0 {
            // CPUFreq can be disabled at boot independently of this driver.
            return;
        }
        self.registered.store(true, Ordering::Release);
        // SAFETY: nr_cpu_ids is initialized before driver probe.
        let cpus = unsafe { bindings::nr_cpu_ids };
        for cpu in 0..cpus {
            // SAFETY: The bounded CPU index is valid; a nonnull result owns a
            // reference even if removal begins before acquiring the policy lock.
            let policy = unsafe { bindings::cpufreq_cpu_get(cpu) };
            if policy.is_null() {
                continue;
            }
            // CREATE holds the policy write lock before taking requests. Use
            // the same lock order and wait out partial policy initialization.
            // SAFETY: The retained reference keeps the policy and rwsem alive.
            unsafe { bindings::down_read(ptr::addr_of_mut!((*policy).rwsem)) };
            {
                let mut requests = self.requests.lock();
                // Removal unpublishes the CPU pointer before its notifier takes
                // requests. Rechecking under this mutex makes either the seed
                // skip the policy or the subsequent REMOVE release its slot.
                // SAFETY: The CPU index is valid; balance this extra reference below.
                let current = unsafe { bindings::cpufreq_cpu_get(cpu) };
                // SAFETY: The policy read lock excludes unfinished CREATE. If
                // this policy is still published, REMOVE cannot yet have removed
                // the core max request without passing our held requests mutex.
                let initialized =
                    current == policy && unsafe { !(*policy).max_freq_req.qos.is_null() };
                if initialized {
                    // SAFETY: The reference and read lock keep the initialized
                    // policy alive; registered REMOVE callbacks cover its lifetime.
                    unsafe { requests.as_mut().add(policy) };
                }
                if !current.is_null() {
                    // SAFETY: Balances the second lookup, independently of its identity.
                    unsafe { bindings::cpufreq_cpu_put(current) };
                }
            }
            // SAFETY: Balance the read lock and first lookup after releasing requests.
            unsafe {
                bindings::up_read(ptr::addr_of_mut!((*policy).rwsem));
                bindings::cpufreq_cpu_put(policy);
            }
        }
    }

    /// Process a cpufreq policy notification.
    ///
    /// # Safety
    ///
    /// `block` must be this pinned Feed's registered notifier, and `data` must
    /// point to the live policy supplied by the cpufreq notifier core.
    /// CREATE must hold its write lock; REMOVE must precede policy release.
    #[cfg(CONFIG_CPU_FREQ)]
    unsafe extern "C" fn policy_event(
        block: *mut bindings::notifier_block,
        event: c_ulong,
        data: *mut core::ffi::c_void,
    ) -> core::ffi::c_int {
        // SAFETY: This callback is installed only on Feed::notifier.block. Its
        // owner remains pinned until unregister joins every callback.
        let notifier =
            unsafe { kernel::container_of!(Opaque::cast_from(block), PolicyNotifier, block) };
        // SAFETY: The notifier is the embedded, pinned field of this live Feed.
        let feed = unsafe { &*kernel::container_of!(notifier, Feed, notifier) };
        let policy = data.cast::<bindings::cpufreq_policy>();
        let mut requests = feed.requests.lock();
        if event == bindings::CPUFREQ_REMOVE_POLICY as c_ulong {
            requests.as_mut().remove(policy);
        } else if event == bindings::CPUFREQ_CREATE_POLICY as c_ulong
            && !feed.detector.stopping.load(Ordering::Acquire)
        {
            // SAFETY: CREATE supplies an initialized policy with its write lock
            // held and guarantees REMOVE before the core releases its storage.
            unsafe { requests.as_mut().add(policy) };
        } else {
            return bindings::NOTIFY_DONE as i32;
        }
        let enabled = feed.detector.wanted(Self::now());
        let applied = requests.as_mut().apply(enabled);
        feed.applied.store(applied, Ordering::Relaxed);
        bindings::NOTIFY_OK as i32
    }

    fn now() -> u64 {
        Monotonic::ktime_get() as u64
    }

    pub(crate) fn note_pass(&self, start: u64, end: u64) {
        if cfg!(CONFIG_CPU_FREQ) {
            self.detector.note(start, end, Self::now());
        }
    }

    /// Called after a firmware event pass, before queuing the single feed work item.
    pub(crate) fn take_pending(&self) -> bool {
        self.detector.pending.swap(false, Ordering::AcqRel)
    }

    #[cfg(CONFIG_CPU_FREQ)]
    fn apply(&self, enabled: bool) {
        let mut requests = self.requests.lock();
        let enabled = enabled && !self.detector.stopping.load(Ordering::Acquire);
        let applied = requests.as_mut().apply(enabled);
        self.applied.store(applied, Ordering::Relaxed);
    }

    #[cfg(not(CONFIG_CPU_FREQ))]
    fn apply(&self, _enabled: bool) {}

    /// Worker context; the linger deadline has no firmware event of its own.
    pub(crate) fn run(&self) {
        loop {
            if self.detector.wanted(Self::now()) {
                self.apply(true);
                fsleep(Delta::from_millis(POLL_MS));
                continue;
            }
            self.apply(false);
            self.detector.active.store(false, Ordering::SeqCst);
            // A detector can extend the deadline after observing the old active
            // bit. Reclaim that work unless it already queued another owner.
            if self.detector.wanted(Self::now())
                && !self.detector.active.swap(true, Ordering::AcqRel)
            {
                continue;
            }
            return;
        }
    }

    pub(crate) fn wait_hint(&self) -> Option<(u32, u32)> {
        if cfg!(CONFIG_UCLAMP_TASK)
            && self.detector.wanted(Self::now())
            && self.applied.load(Ordering::Relaxed)
        {
            Some((WAIT_UTIL_MIN, WAIT_HINT_MS))
        } else {
            None
        }
    }

    /// Call before disabling/joining the event producer and actuator work items.
    pub(crate) fn begin_shutdown(&self) {
        self.detector.stopping.store(true, Ordering::Release);
    }

    /// Call after work items are joined. Removing requests withdraws their floor.
    pub(crate) fn shutdown(&self) {
        self.begin_shutdown();
        #[cfg(CONFIG_CPU_FREQ)]
        {
            // Unregister joins callbacks, which take requests: never hold that
            // mutex here. Device shutdown has already joined the feed worker.
            if self.registered.swap(false, Ordering::AcqRel) {
                // SAFETY: The block was registered successfully and remains
                // pinned and alive until this synchronous removal returns.
                unsafe {
                    bindings::cpufreq_unregister_notifier(
                        self.notifier.block.get(),
                        bindings::CPUFREQ_POLICY_NOTIFIER,
                    );
                }
            }
            self.requests.lock().as_mut().clear();
        }
        self.applied.store(false, Ordering::Relaxed);
    }
}

#[pinned_drop]
impl PinnedDrop for Feed {
    fn drop(self: Pin<&mut Self>) {
        self.shutdown();
    }
}
