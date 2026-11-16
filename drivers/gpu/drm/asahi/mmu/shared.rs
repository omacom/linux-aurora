// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Shared-object identities for attachment ordering across user address spaces.
//! These records do not own mappings. The VM's job guard protects the mappings
//! while submission captures the intersecting object offsets. Mapping commits
//! hold the GPUVM exec lock through metadata updates, then release it. The lock
//! order is GPUVM exec followed by bindings; footprint readers take only bindings.

use super::*;
use crate::g17::{hazard::Footprint, Attachments};

struct Binding {
    start: u64,
    end: u64,
    object: usize,
    offset: u64,
    bind_sequence: u64,
}

#[pin_data]
pub(super) struct SharedBindings {
    #[pin]
    bindings: Mutex<KVec<Binding>>,
    count: AtomicU32,
}

impl SharedBindings {
    pub(super) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                bindings <- new_mutex!(KVec::new(), "Shared attachment bindings"),
                count: AtomicU32::new(0),
            }),
            GFP_KERNEL,
        )
    }
}

impl Vm {
    pub(super) fn track_shared_binding(
        &self,
        gem: &gem::Object,
        address: u64,
        size: u64,
        offset: u64,
        single_page: bool,
        bind_sequence: u64,
    ) {
        let Some(shared) = &self.shared_bindings else {
            return;
        };
        let Some(end) = address.checked_add(size) else {
            return;
        };
        let external = !single_page && self.is_extobj(gem);
        if !external && shared.count.load(Ordering::Relaxed) == 0 {
            return;
        }
        let mut bindings = shared.bindings.lock();
        bindings.retain(|entry| entry.end <= address || end <= entry.start);
        if external {
            let _ = bindings.push(
                Binding {
                    start: address,
                    end,
                    object: core::ptr::from_ref(gem) as usize,
                    offset,
                    bind_sequence,
                },
                GFP_KERNEL,
            );
        }
        shared.count.store(bindings.len() as u32, Ordering::Relaxed);
    }

    pub(super) fn untrack_shared_range(&self, address: u64, size: u64) {
        let Some(shared) = &self.shared_bindings else {
            return;
        };
        if shared.count.load(Ordering::Relaxed) == 0 {
            return;
        }
        let Some(end) = address.checked_add(size) else {
            return;
        };
        let mut bindings = shared.bindings.lock();
        bindings.retain(|entry| entry.end <= address || end <= entry.start);
        shared.count.store(bindings.len() as u32, Ordering::Relaxed);
    }

    pub(crate) fn untrack_shared_object(&self, gem: &gem::Object) {
        self.untrack_shared_object_before(gem, u64::MAX);
    }

    pub(super) fn untrack_shared_object_before(&self, gem: &gem::Object, sequence: u64) {
        let Some(shared) = &self.shared_bindings else {
            return;
        };
        if shared.count.load(Ordering::Relaxed) == 0 {
            return;
        }
        let object = core::ptr::from_ref(gem) as usize;
        let mut bindings = shared.bindings.lock();
        bindings.retain(|entry| entry.object != object || entry.bind_sequence > sequence);
        shared.count.store(bindings.len() as u32, Ordering::Relaxed);
    }

    pub(crate) fn render_footprint(
        &self,
        attachments: &Attachments,
    ) -> Result<Option<Arc<Footprint>>> {
        if attachments.as_slice().is_empty() {
            return Ok(None);
        }
        let mut footprint = Footprint::new(self.id);
        let bindings = self.shared_bindings.as_ref().and_then(|shared| {
            (shared.count.load(Ordering::Relaxed) != 0).then(|| shared.bindings.lock())
        });
        for attachment in attachments.as_slice() {
            let end = attachment.va.saturating_add(attachment.size);
            footprint.push_vm(attachment.va, end);
            let Some(bindings) = &bindings else {
                continue;
            };
            for entry in bindings.iter() {
                let first = entry.start.max(attachment.va);
                let last = entry.end.min(end);
                if first < last {
                    let offset = entry.offset.saturating_add(first - entry.start);
                    footprint.push_object(
                        entry.object,
                        offset,
                        offset.saturating_add(last - first),
                    );
                }
            }
        }
        drop(bindings);
        Ok(Some(Arc::new(footprint, GFP_KERNEL)?))
    }
}
