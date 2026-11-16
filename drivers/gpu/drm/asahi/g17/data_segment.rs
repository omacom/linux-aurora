// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2026 The Gravity Linux Contributors
//
// Derived from the crash buffer mapping of Gravity Linux's M4 GPU driver (CrashBuffer and
// BootOps::shmem_map in drivers/gpu/drm/asahi/g16.rs), written by Cody Ho:
// https://github.com/GravityLinux/linux/commit/bd25d194b0a295a75692a0e6caed3793603b9e65
// Gravity's g16.rs is adapted from Niklas Sheth's linux-m4-integration prototype.

//! Crash log buffers in the GPU firmware's data segment.
//!
//! A firmware that places its own crash log ([`super::CrashBuffers::FirmwareData`]) asks for it
//! by physical address inside its data segment, the `fw-data` reserved region, which the boot
//! loader keeps for the loaded firmware until reboot.

use kernel::{
    c_str, device,
    io::{
        self,
        IoSysMap,
        Region, //
    },
    prelude::*,
};

/// `size` bytes at `offset` of the firmware's data segment, mapped write-back.
pub(super) struct DataSegmentLog {
    mem: io::mem::Mem,
    offset: usize,
    size: usize,
}

impl DataSegmentLog {
    /// Maps the `size` bytes at physical address `iova` that the firmware asked for, which
    /// must lie inside the `fw-data` region of `dev`.
    pub(super) fn map(dev: &device::Device, iova: usize, size: usize) -> Result<Self> {
        let node = dev.of_node().ok_or(EINVAL)?;
        let res = node.reserved_mem_region_to_resource_byname(c_str!("fw-data"))?;
        let offset = iova.checked_sub(res.start().try_into()?).ok_or(EINVAL)?;
        if offset.checked_add(size).ok_or(EINVAL)? > res.size().try_into()? {
            return Err(EINVAL);
        }
        // SAFETY: The boot loader reserved this RAM for the loaded GPU firmware, which keeps
        // it until reboot; only the requested subrange is exposed.
        let mem = unsafe { io::mem::Mem::try_new(res, io::mem::MemFlag::WB.into())? };
        Ok(Self { mem, offset, size })
    }

    /// The CPU view of the buffer.
    pub(super) fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
        // SAFETY: map() checked that the range lies inside the mapping.
        let base = unsafe { self.mem.ptr().add(self.offset) };
        let ptr = Region::<0>::ptr_try_from_raw_parts_mut(base, self.size)?;
        // SAFETY: The range is mapped RAM for as long as this borrow of the mapping.
        Ok(IoSysMap::Sys(unsafe { io::SysMem::new(ptr) }))
    }
}
