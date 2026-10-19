// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Firmware objects and their cache classes.
//!
//! Each object is a GEM kernel object (write-combined CPU mapping) mapped into the host window
//! at a fixed VA or in [`super::hw::va::FW_ALLOC`] (a bump allocator with an unmapped guard page
//! after each object), with these optional aliases of the same pages:
//!
//! - a context-0 lower alias (`Uat::kernel_lower_vm`, [`super::hw::pte::LOWER`]);
//! - a client alias at a fixed GVA in every user VM: recorded here ([`Objects::clients`]) and
//!   installed by `vm.rs` as driver mappings when the VM is created.
//!
//! The backing is populated and DMA-mapped before any CPU view exists: on this non-DMA-coherent
//! device the DMA mapping cleans the freshly zeroed pages' cacheable alias to the point of
//! coherency, so no dirty line can later overwrite what the host writes through the
//! write-combined view. The firmware-side leaf word is chosen by [`Class`] and must be exact
//! ([`Objects::self_check`] proves the shared page-table code writes it). Host accesses
//! after publication use single, aligned volatile accesses (`r32`/`w32`/`w64`). Objects are
//! retained until reboot: nothing the firmware can reach is ever unmapped or freed.

use core::ptr::NonNull;

use kernel::{prelude::*, sync::aref::ARef, sync::Arc};

use super::hw::{self, pte};
use super::regs::wmb;
use super::uat;
use crate::driver::AsahiDevice;
use crate::{gem, mmu};

/// Firmware-side cache class of an object (the leaf words of [`super::hw::pte`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Class {
    /// Firmware-only, AttrIndex 2.
    Fw,
    /// Firmware-only, ASC-cached (AttrIndex 0). Never for rings or anything the host writes
    /// after the firmware first touched it.
    FwCached,
    /// Firmware and GPU, AttrIndex 2.
    Shared,
    /// Firmware and GPU, ASC-cached.
    SharedCached,
    /// Firmware device mapping.
    Mmio,
}

impl Class {
    /// The exact firmware leaf word of this class.
    pub(crate) const fn pte(self) -> u64 {
        match self {
            Class::Fw => pte::FW,
            Class::FwCached => pte::FW_CACHED,
            Class::Shared => pte::SHARED,
            Class::SharedCached => pte::SHARED_CACHED,
            Class::Mmio => pte::MMIO,
        }
    }
}

/// Where an object is placed in the host window.
pub(crate) enum Placement {
    /// At this firmware VA (canonical initdata objects). The payload starts at this VA; the
    /// object starts at its page.
    At(u64),
    /// Anywhere in the firmware allocator window, followed by an unmapped guard page.
    Anywhere,
}

/// A firmware object.
///
/// # Invariants
///
/// `cpu`, if set, points to a live write-combined kernel mapping of the whole backing (`bytes`
/// bytes), kept alive by `backing`.
pub(crate) struct FwObject {
    /// Firmware (host-window) VA of byte 0 of the payload.
    pub(crate) va: u64,
    /// Client GVA of byte 0, if aliased into every user VM.
    pub(crate) gva: Option<u64>,
    /// Payload size.
    pub(crate) size: usize,
    /// Payload offset inside the first page.
    offset: usize,
    /// Mapped bytes: the payload rounded up to whole pages.
    bytes: usize,
    /// Client alias leaf word.
    client_leaf: u64,
    cpu: Option<NonNull<u8>>,
    /// Host-window and lower mappings; never dropped while the firmware runs.
    _mappings: KVec<mmu::KernelMapping>,
    /// Holds the pages for the device.
    _sgt: kernel::drm::gem::shmem::SGTable<gem::AsahiObject>,
    backing: gem::ObjectRef,
}

// SAFETY: The object only refers to kernel memory shared with the firmware; every host access
// goes through raw pointers with volatile or byte-copy semantics, serialized by the device lock
// where the protocol needs it.
unsafe impl Send for FwObject {}
// SAFETY: See `Send`.
unsafe impl Sync for FwObject {}

impl FwObject {
    /// Pointer to payload byte `offset` for `len` bytes, bounded by the mapped pages.
    fn ptr(&self, offset: usize, len: usize) -> Result<*mut u8> {
        let cpu = self.cpu.ok_or(EINVAL)?;
        let at = self.offset.checked_add(offset).ok_or(EINVAL)?;
        if at.checked_add(len).ok_or(EINVAL)? > self.bytes {
            return Err(EINVAL);
        }
        // SAFETY: By the type invariant `cpu` maps `bytes` bytes; `at + len` was checked.
        Ok(unsafe { cpu.as_ptr().add(at) })
    }

    /// Copies bytes into the object before it is published, or into host-owned words.
    pub(crate) fn write(&self, offset: usize, bytes: &[u8]) -> Result {
        let p = self.ptr(offset, bytes.len())?;
        // SAFETY: `p` is valid for `bytes.len()` bytes of the object's mapping, which no Rust
        // reference aliases.
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), p, bytes.len()) };
        Ok(())
    }

    /// Fills bytes.
    pub(crate) fn fill(&self, offset: usize, value: u8, len: usize) -> Result {
        let p = self.ptr(offset, len)?;
        // SAFETY: As for `write`.
        unsafe { core::ptr::write_bytes(p, value, len) };
        Ok(())
    }

    /// 32-bit store: a single aligned access, or bytes at an unaligned offset.
    pub(crate) fn w32(&self, offset: usize, value: u32) -> Result {
        let p = self.ptr(offset, 4)?;
        if p as usize % 4 == 0 {
            // SAFETY: `p` is valid and aligned for a u32.
            unsafe { (p as *mut u32).write_volatile(value.to_le()) };
            Ok(())
        } else {
            self.write(offset, &value.to_le_bytes())
        }
    }

    /// 64-bit store: a single aligned access, or bytes at an unaligned offset.
    pub(crate) fn w64(&self, offset: usize, value: u64) -> Result {
        let p = self.ptr(offset, 8)?;
        if p as usize % 8 == 0 {
            // SAFETY: `p` is valid and aligned for a u64.
            unsafe { (p as *mut u64).write_volatile(value.to_le()) };
            Ok(())
        } else {
            self.write(offset, &value.to_le_bytes())
        }
    }

    /// 32-bit load of a firmware- or GPU-written word: a single aligned access, or an unaligned
    /// load (words of the unaligned canonical objects).
    pub(crate) fn r32(&self, offset: usize) -> Result<u32> {
        let p = self.ptr(offset, 4)?;
        // SAFETY: `p` is valid for 4 bytes; Normal non-cacheable memory permits an unaligned
        // load.
        Ok(u32::from_le(unsafe {
            if p as usize % 4 == 0 {
                (p as *const u32).read_volatile()
            } else {
                (p as *const u32).read_unaligned()
            }
        }))
    }

    /// Mapped bytes (the payload offset plus size, rounded up to pages).
    pub(crate) fn bytes(&self) -> usize {
        self.bytes
    }

    /// The client alias: GVA of the first page and its leaf word. `vm.rs` maps [`Self::gem`]
    /// there in every user VM.
    pub(crate) fn client(&self) -> Option<(u64, u64)> {
        self.gva
            .map(|gva| (gva - self.offset as u64, self.client_leaf))
    }

    /// The backing GEM object, for aliases in other address spaces.
    pub(crate) fn gem(&self) -> ARef<gem::Object> {
        self.backing.gem.clone()
    }
}

/// Allocation request.
pub(crate) struct Request {
    pub(crate) size: usize,
    pub(crate) class: Class,
    pub(crate) placement: Placement,
    /// Payload offset inside the first page (e.g. work images at +0x3760). Ignored
    /// for [`Placement::At`], whose VA gives it.
    pub(crate) offset: usize,
    /// Client alias GVA of the page holding the payload, and its leaf word
    /// ([`pte::LEAF_NC`], [`pte::LEAF_CACHED`] or [`pte::LEAF_PB_LIST`]).
    pub(crate) client: Option<(u64, u64)>,
    /// Context-0 lower alias GVA of the first page (the client alias page).
    pub(crate) lower: Option<u64>,
    /// GPU-only objects need no CPU mapping (zeroed at allocation).
    pub(crate) cpu: bool,
}

/// The firmware object allocator and the set of retained objects.
pub(crate) struct Objects {
    dev: ARef<AsahiDevice>,
    uat: Arc<mmu::Uat>,
    /// The host window: shared L2 mirror and PPL handoff.
    pub(crate) window: uat::Window,
    /// Next free VA of the firmware allocator window.
    fw_next: u64,
    /// Every object, retained until reboot.
    retained: KVec<Arc<FwObject>>,
    /// Objects aliased into every user VM.
    clients: KVec<Arc<FwObject>>,
    /// Device aliases and self-check mappings, retained until reboot.
    mappings: KVec<mmu::KernelMapping>,
    /// Host-window pages backed by objects.
    pages: usize,
}

impl Objects {
    pub(crate) fn new(dev: &AsahiDevice, uat: Arc<mmu::Uat>, window: uat::Window) -> Self {
        Self {
            dev: dev.into(),
            uat,
            window,
            fw_next: hw::va::FW_ALLOC.start,
            retained: KVec::new(),
            clients: KVec::new(),
            mappings: KVec::new(),
            pages: 0,
        }
    }

    /// Objects aliased into every user VM, in allocation order.
    pub(crate) fn clients(&self) -> &[Arc<FwObject>] {
        &self.clients
    }

    /// Host-window pages backed by firmware objects.
    pub(crate) fn pages(&self) -> usize {
        self.pages
    }

    /// Mirrors host-window mappings made outside this allocator (timestamp buffers) into the
    /// firmware's view before their addresses are used.
    pub(crate) fn sync_window(&mut self, range: core::ops::Range<u64>) -> Result {
        self.window.sync_range(&self.uat, range)
    }

    /// Allocates and maps a zeroed firmware object.
    pub(crate) fn alloc(&mut self, req: Request) -> Result<Arc<FwObject>> {
        self.alloc_split(req, None)
    }

    /// Like [`Self::alloc`], with the first `cached` mapped bytes (a whole number of pages) in
    /// `class` and the rest in the request's class (HardwareData).
    pub(super) fn alloc_split(
        &mut self,
        req: Request,
        prefix: Option<(usize, Class)>,
    ) -> Result<Arc<FwObject>> {
        let page = hw::PAGE as usize;
        let (page_va, offset) = match req.placement {
            Placement::At(va) => (va & !(hw::PAGE - 1), (va & (hw::PAGE - 1)) as usize),
            Placement::Anywhere => (self.fw_next, req.offset),
        };
        if req.size == 0 || offset >= page {
            return Err(EINVAL);
        }
        let bytes = offset
            .checked_add(req.size)
            .ok_or(EINVAL)?
            .next_multiple_of(page);
        if page_va < hw::HOST || page_va + bytes as u64 > hw::HOST + hw::HOST_SIZE {
            return Err(EINVAL);
        }
        if let Placement::Anywhere = req.placement {
            if page_va + bytes as u64 + hw::PAGE > hw::va::FW_ALLOC.end {
                return Err(ENOSPC);
            }
        }
        if let Some((gpage, _)) = req.client {
            if gpage & (hw::PAGE - 1) != 0 || gpage + bytes as u64 > hw::USER_RANGE.end {
                return Err(EINVAL);
            }
            if self.clients.len() == hw::sizes::MAX_CLIENT {
                return Err(ENOSPC);
            }
        }
        if let Some((cached, _)) = prefix {
            if cached == 0 || cached % page != 0 || cached >= bytes {
                return Err(EINVAL);
            }
        }

        // Populate and DMA-map first: this cleans the zeroed pages' cacheable alias to the
        // point of coherency before any write-combined access.
        let mut backing = gem::new_kernel_object_wc(&self.dev, bytes)?;
        let sgt = gem::held_sg_table(&backing.gem)?;
        for range in sgt.iter() {
            let pa = range.dma_address() as u64;
            let len = range.dma_len() as u64;
            // EPERM for a page the UAT cannot name.
            if pa == 0
                || (pa | len) & (hw::PAGE - 1) != 0
                || (pa + len - 1) & !(hw::PA_MASK | (hw::PAGE - 1)) != 0
            {
                return Err(EPERM);
            }
        }
        let cpu = if req.cpu {
            let mut vmap = backing.vmap()?;
            vmap.memset(0);
            let ptr = NonNull::new(vmap.as_mut_ptr()).ok_or(ENOMEM)?;
            core::mem::drop(vmap);
            Some(ptr)
        } else {
            None
        };

        let mut mappings = KVec::new();
        let kernel_vm = self.uat.kernel_vm();
        let guard = matches!(req.placement, Placement::Anywhere);
        match prefix {
            None => {
                mappings.push(
                    kernel_vm.map_at(
                        page_va,
                        bytes,
                        backing.gem.clone(),
                        mmu::Prot::from_pte(req.class.pte()),
                        guard,
                    )?,
                    GFP_KERNEL,
                )?;
            }
            Some((cached, class)) => {
                mappings.push(
                    kernel_vm.map_in_range(
                        &backing.gem,
                        0..cached,
                        hw::PAGE,
                        page_va..page_va + cached as u64,
                        mmu::Prot::from_pte(class.pte()),
                        false,
                    )?,
                    GFP_KERNEL,
                )?;
                let rest = page_va + cached as u64;
                mappings.push(
                    kernel_vm.map_in_range(
                        &backing.gem,
                        cached..bytes,
                        hw::PAGE,
                        rest..page_va + bytes as u64,
                        mmu::Prot::from_pte(req.class.pte()),
                        guard,
                    )?,
                    GFP_KERNEL,
                )?;
                if mappings[0].iova() != page_va || mappings[1].iova() != rest {
                    return Err(EINVAL);
                }
            }
        }
        self.window
            .sync_range(&self.uat, page_va..page_va + bytes as u64)?;
        if let Placement::Anywhere = req.placement {
            // Skip an unmapped guard page after the object.
            self.fw_next += bytes as u64 + hw::PAGE;
        }
        if let Some(lower) = req.lower {
            if lower & (hw::PAGE - 1) != 0 {
                return Err(EINVAL);
            }
            mappings.push(
                self.uat.kernel_lower_vm().map_at(
                    lower,
                    bytes,
                    backing.gem.clone(),
                    mmu::Prot::from_pte(pte::LOWER),
                    false,
                )?,
                GFP_KERNEL,
            )?;
            uat::clean_lower(&self.uat)?;
        }
        wmb();

        // INVARIANT: `cpu` came from the owned vmap that `backing` keeps for its lifetime.
        let object = Arc::new(
            FwObject {
                va: page_va + offset as u64,
                gva: req.client.map(|(gpage, _)| gpage + offset as u64),
                size: req.size,
                offset,
                bytes,
                client_leaf: req.client.map_or(pte::LEAF_NC, |(_, leaf)| leaf),
                cpu,
                _mappings: mappings,
                _sgt: sgt,
                backing,
            },
            GFP_KERNEL,
        )?;
        self.pages += bytes / page;
        self.retained.push(object.clone(), GFP_KERNEL)?;
        if req.client.is_some() {
            self.clients.push(object.clone(), GFP_KERNEL)?;
        }
        Ok(object)
    }

    /// Maps a device range (an IOMapping record) for the firmware as [`Class::Mmio`].
    pub(crate) fn map_mmio(&mut self, va: u64, pa: u64, size: u64) -> Result {
        let mapping = self.uat.kernel_vm().map_io(
            va,
            usize::try_from(pa)?,
            usize::try_from(size)?,
            mmu::Prot::from_pte(Class::Mmio.pte()),
        )?;
        self.mappings.push(mapping, GFP_KERNEL)?;
        self.window.sync_range(&self.uat, va..va + size)
    }

    /// Boot-time check that the shared page-table code writes every firmware leaf word exactly:
    /// each class word is mapped on a scratch page with `Prot::from_pte(word)`, the leaf is read
    /// back from the tables and must equal `pa | word`. Host-window classes are checked in the
    /// (global) host window, the lower and client classes in the (non-global) context-0 lower
    /// tree, which user VMs share the leaf format with. Runs before anything firmware-visible is
    /// written; the scratch mappings are retained (they are never published). A mismatch refuses
    /// the boot.
    pub(crate) fn self_check(&mut self) -> Result {
        let window = [
            pte::FW,
            pte::SHARED,
            pte::FW_CACHED,
            pte::SHARED_CACHED,
            pte::MMIO,
        ];
        let lower = [
            pte::LOWER,
            pte::LEAF_NC,
            pte::LEAF_CACHED,
            pte::LEAF_PB_LIST,
        ];
        let scratch = gem::new_kernel_object_wc(&self.dev, hw::PAGE as usize)?;
        let sgt = gem::held_sg_table(&scratch.gem)?;
        let pa = sgt.iter().next().ok_or(EINVAL)?.dma_address() as u64;
        let checks = Iterator::chain(
            window
                .iter()
                .enumerate()
                .map(|(i, &w)| (true, hw::va::SELF_CHECK + i as u64 * hw::PAGE, w)),
            lower
                .iter()
                .enumerate()
                .map(|(i, &w)| (false, hw::va::SELF_CHECK_LOWER + i as u64 * hw::PAGE, w)),
        );
        let mut count = 0;
        for (host, va, word) in checks {
            let (vm, root) = if host {
                (self.uat.kernel_vm(), self.uat.kernel_window_root())
            } else {
                (self.uat.kernel_lower_vm(), self.uat.kernel_lower_root())
            };
            let mapping = vm.map_at(
                va,
                hw::PAGE as usize,
                scratch.gem.clone(),
                mmu::Prot::from_pte(word),
                false,
            )?;
            self.mappings.push(mapping, GFP_KERNEL)?;
            let leaf = uat::read_leaf(root, va)?;
            if leaf != Some(pa | word) {
                dev_err!(
                    self.dev.as_ref(),
                    "G15: UAT leaf self-check failed at {:#x}: want {:#x}, tables hold {:#x}\n",
                    va,
                    pa | word,
                    leaf.unwrap_or(0)
                );
                return Err(super::boot::eproto());
            }
            count += 1;
        }
        // The scratch page stays mapped: retain its backing with the mappings.
        core::mem::forget(sgt);
        core::mem::forget(scratch);
        dev_info!(
            self.dev.as_ref(),
            "G15: UAT leaf self-check passed ({} classes)\n",
            count
        );
        Ok(())
    }
}
