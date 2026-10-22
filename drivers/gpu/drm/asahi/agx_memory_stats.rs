// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Bounded host accounting for driver-owned GEM extents and M3 coherent backing.
//!
//! These are allocation extents, not resident/pinned pages or unique imported
//! physical memory. GEM construction/unwind is included. Counters are per
//! module instance and each read is observational, not a transactional snapshot.
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use kernel::prelude::*;

struct Counters {
    objects: AtomicU64,
    bytes: AtomicU64,
    peak_objects: AtomicU64,
    peak_bytes: AtomicU64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backing_owner_drop_releases_extent_once_and_keeps_peak() {
        static COUNTERS: Counters = Counters::new();
        let first = Allocation::new(&COUNTERS, 0x4000).unwrap();
        let second = Allocation::new(&COUNTERS, 0x8000).unwrap();
        assert_eq!(COUNTERS.snapshot(), [2, 0xc000, 2, 0xc000]);
        let shared_owner = std::sync::Arc::new(first);
        let retained = shared_owner.clone();
        drop(shared_owner);
        drop(second);
        assert_eq!(COUNTERS.snapshot(), [1, 0x4000, 2, 0xc000]);
        drop(retained);
        assert_eq!(COUNTERS.snapshot(), [0, 0, 2, 0xc000]);
    }

    #[test]
    fn refused_extent_does_not_change_live_object_count() {
        static COUNTERS: Counters = Counters::new();
        assert!(matches!(Allocation::new(&COUNTERS, 0), Err(EINVAL)));
        assert_eq!(COUNTERS.snapshot(), [0; 4]);
        COUNTERS.bytes.store(u64::MAX, Ordering::Relaxed);
        assert!(matches!(Allocation::new(&COUNTERS, 1), Err(EOVERFLOW)));
        assert_eq!(COUNTERS.objects.load(Ordering::Relaxed), 0);
        assert_eq!(COUNTERS.bytes.load(Ordering::Relaxed), u64::MAX);
    }

    #[test]
    fn concurrent_aliases_classify_one_backing_and_release_it_once() {
        let allocation = std::sync::Arc::new(Allocation::gem(0x4000, true).unwrap());
        std::thread::scope(|scope| {
            for category in [
                KernelCategory::Firmware,
                KernelCategory::Descriptor,
                KernelCategory::QueueConfig,
                KernelCategory::FreeList,
            ] {
                let alias = allocation.clone();
                scope.spawn(move || alias.classify_kernel(category));
            }
        });
        let class_bytes: u64 = KERNEL_CATEGORIES.iter().map(|c| c.snapshot()[1]).sum();
        let class_objects: u64 = KERNEL_CATEGORIES.iter().map(|c| c.snapshot()[0]).sum();
        assert_eq!((class_objects, class_bytes), (1, 0x4000));
        assert_eq!(KERNEL_CATEGORIES[0].snapshot()[0], 0);
        let retained = allocation.clone();
        drop(allocation);
        assert_eq!(KERNEL_GEM.snapshot()[1], 0x4000);
        drop(retained);
        assert_eq!(KERNEL_GEM.snapshot()[1], 0);
        assert!(KERNEL_CATEGORIES
            .iter()
            .all(|c| c.snapshot()[0] == 0 && c.snapshot()[1] == 0));
    }
}
impl Counters {
    const fn new() -> Self {
        Self {
            objects: AtomicU64::new(0),
            bytes: AtomicU64::new(0),
            peak_objects: AtomicU64::new(0),
            peak_bytes: AtomicU64::new(0),
        }
    }
    fn snapshot(&self) -> [u64; 4] {
        let objects = self.objects.load(Ordering::Relaxed);
        let bytes = self.bytes.load(Ordering::Relaxed);
        [
            objects,
            bytes,
            self.peak_objects.load(Ordering::Relaxed).max(objects),
            self.peak_bytes.load(Ordering::Relaxed).max(bytes),
        ]
    }
}
static USER_GEM: Counters = Counters::new();
static KERNEL_GEM: Counters = Counters::new();
// Core-created objects may be local dumb buffers or wrappers around imported backing.
static CORE_GEM: Counters = Counters::new();
static COHERENT: Counters = Counters::new();

/// A breakdown of kernel GEM extents, not an additional memory total.
#[derive(Copy, Clone)]
#[repr(u32)]
pub(crate) enum KernelCategory {
    Other = 0,
    Firmware = 1,
    Descriptor = 2,
    WorkState = 3,
    QueueState = 4,
    QueueConfig = 5,
    FreeList = 6,
    RenderGeneration = 7,
    RenderTarget = 8,
    TileBuffer = 9,
    QueueSupport = 10,
    Scratch = 11,
    Scheduler = 12,
}
const CATEGORY_NAMES: [&str; 13] = [
    "other",
    "firmware",
    "descriptor",
    "work_state",
    "queue_state",
    "queue_config",
    "free_list",
    "render_generation",
    "render_target",
    "tile_buffer",
    "queue_support",
    "scratch",
    "scheduler",
];
static KERNEL_CATEGORIES: [Counters; 13] = [const { Counters::new() }; 13];
const NO_CATEGORY: u32 = u32::MAX;

/// Drop with the actual backing owner, never with an individual GEM handle or
/// its mapping. Retained fault owners therefore remain included after exit.
pub(crate) struct Allocation {
    counters: &'static Counters,
    bytes: u64,
    kernel_category: AtomicU32,
}
impl Allocation {
    fn new(counters: &'static Counters, size: usize) -> Result<Self> {
        let bytes = size as u64;
        if bytes == 0 {
            return Err(EINVAL);
        }
        let previous = counters
            .bytes
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |current| {
                current.checked_add(bytes)
            })
            .map_err(|_| EOVERFLOW)?;
        let objects = counters.objects.fetch_add(1, Ordering::Relaxed) + 1;
        counters
            .peak_bytes
            .fetch_max(previous + bytes, Ordering::Relaxed);
        counters.peak_objects.fetch_max(objects, Ordering::Relaxed);
        Ok(Self {
            counters,
            bytes,
            kernel_category: AtomicU32::new(NO_CATEGORY),
        })
    }
    pub(crate) fn gem(size: usize, kernel: bool) -> Result<Self> {
        let allocation = Self::new(if kernel { &KERNEL_GEM } else { &USER_GEM }, size)?;
        if kernel {
            Self::add_category(0, allocation.bytes);
            allocation.kernel_category.store(0, Ordering::Relaxed);
        }
        Ok(allocation)
    }
    pub(crate) fn coherent(size: usize) -> Result<Self> {
        Self::new(&COHERENT, size)
    }
    pub(crate) fn core(size: usize) -> Result<Self> {
        Self::new(&CORE_GEM, size)
    }

    fn add_category(index: usize, bytes: u64) {
        let counters = &KERNEL_CATEGORIES[index];
        let previous = counters.bytes.fetch_add(bytes, Ordering::Relaxed);
        let objects = counters.objects.fetch_add(1, Ordering::Relaxed) + 1;
        counters
            .peak_bytes
            .fetch_max(previous + bytes, Ordering::Relaxed);
        counters.peak_objects.fetch_max(objects, Ordering::Relaxed);
    }

    /// Refine an unclassified backing once. Aliases do not create another allocation.
    pub(crate) fn classify_kernel(&self, category: KernelCategory) {
        let index = category as u32;
        if index != 0
            && self
                .kernel_category
                .compare_exchange(0, index, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
        {
            Self::add_category(index as usize, self.bytes);
            KERNEL_CATEGORIES[0]
                .bytes
                .fetch_sub(self.bytes, Ordering::Relaxed);
            KERNEL_CATEGORIES[0].objects.fetch_sub(1, Ordering::Relaxed);
        }
    }
}
impl Drop for Allocation {
    fn drop(&mut self) {
        let category = self.kernel_category.load(Ordering::Relaxed);
        if category != NO_CATEGORY {
            KERNEL_CATEGORIES[category as usize]
                .bytes
                .fetch_sub(self.bytes, Ordering::Relaxed);
            KERNEL_CATEGORIES[category as usize]
                .objects
                .fetch_sub(1, Ordering::Relaxed);
        }
        self.counters.bytes.fetch_sub(self.bytes, Ordering::Relaxed);
        self.counters.objects.fetch_sub(1, Ordering::Relaxed);
    }
}

pub(crate) struct View;
impl kernel::debugfs::Writer for View {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        writeln!(f, "version=1 unit=allocation_extent_bytes")?;
        for (name, counters) in [
            ("user_gem", &USER_GEM),
            ("kernel_gem", &KERNEL_GEM),
            ("core_gem_may_alias_imports", &CORE_GEM),
            ("m3_coherent", &COHERENT),
        ] {
            let [objects, bytes, peak_objects, peak_bytes] = counters.snapshot();
            writeln!(
                f,
                "{} objects={} bytes={} peak_objects={} peak_bytes={}",
                name, objects, bytes, peak_objects, peak_bytes
            )?;
        }
        writeln!(f, "kernel_categories=subset_of_kernel_gem")?;
        for (name, counters) in CATEGORY_NAMES.iter().zip(&KERNEL_CATEGORIES) {
            let [objects, bytes, peak_objects, peak_bytes] = counters.snapshot();
            writeln!(
                f,
                "kernel_{} objects={} bytes={} peak_objects={} peak_bytes={}",
                name, objects, bytes, peak_objects, peak_bytes
            )?;
        }
        Ok(())
    }
}
