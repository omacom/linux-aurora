// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact firmware and hardware identity.
//!
//! The firmware build is the SHA-256 of TEXT with the 8 entropy bytes after the single
//! `GKTS\x08\0\0\0` marker zeroed, plus the segment sizes, looked up in the SoC
//! configuration's `firmware` list; the hardware revision is checked against its `hw_ids` at
//! start ([`super::boot`]). An unknown build or revision is refused with its measured identity.
//! The canonical object VAs, the initdata graph, the board images and the work layouts all
//! belong to one build, so nothing is written before the build is identified.
//!
//! Bring-up escape hatch: `g15_accept_text_prefix=<hex>` admits an unpinned build whose digest
//! starts with those 8 bytes (the full digest and marker offset are logged, so that the build
//! can be pinned by the next kernel).

use kernel::{bindings, device, fmt, prelude::*};

use super::boot::glue;
use super::hw::FirmwareBuild;

/// Lowercase hex of a byte string, as `%*phN` prints it.
pub(crate) struct Hex<'a>(pub(crate) &'a [u8]);

impl fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for b in self.0 {
            write!(f, "{:02x}", b)?;
        }
        Ok(())
    }
}

/// Copies `size` bytes at physical `phys`, a range outside the kernel's System RAM, into a
/// private copy through a temporary write-back mapping.
pub(crate) fn copy_physical(phys: u64, size: usize) -> Result<KVVec<u8>> {
    // SAFETY: memremap() of a boot-loader range creates a temporary mapping; it has no other
    // side effects.
    let map = unsafe { bindings::memremap(phys, size, bindings::MEMREMAP_WB as _) };
    if map.is_null() {
        return Err(ENOMEM);
    }
    let copy = KVVec::from_elem(0u8, size, GFP_KERNEL).map(|mut copy| {
        // SAFETY: `map` is a live mapping of `size` bytes and `copy` holds `size` bytes; the two do
        // not overlap.
        unsafe { core::ptr::copy_nonoverlapping(map as *const u8, copy.as_mut_ptr(), size) };
        copy
    });
    // SAFETY: `map` came from memremap() above and is not used afterwards.
    unsafe { bindings::memunmap(map) };
    Ok(copy?)
}

/// The GKTS marker: the 8 bytes after it are per-boot entropy.
const MARKER: [u8; 8] = [0x47, 0x4b, 0x54, 0x53, 0x08, 0, 0, 0];

/// Identifies the loaded firmware build: the pinned build, or `None` for a build admitted by
/// its digest prefix (`g15_accept_text_prefix`).
pub(crate) fn firmware(
    dev: &device::Device,
    adt: &super::adt::GpuAdt,
    builds: &'static [FirmwareBuild],
) -> Result<Option<&'static FirmwareBuild>> {
    let size = usize::try_from(adt.text.end - adt.text.start)?;
    let data_size = adt.data.end - adt.data.start;
    let mut copy = copy_physical(adt.text.start, size)?;
    let mut at = 0usize;
    let mut found = 0usize;
    let mut off = 0usize;
    while off + 16 <= size {
        if copy[off..off + 8] == MARKER {
            at = off;
            found += 1;
        }
        off += 8;
    }
    if found != 1 {
        dev_err!(
            dev,
            "GPU firmware TEXT has {} GKTS markers, need one\n",
            found
        );
        return Err(EINVAL);
    }
    copy[at + 8..at + 16].fill(0);
    let digest = glue::sha256(&copy);
    for build in builds {
        if build.text_size == size as u64
            && build.data_size == data_size
            && build.marker as usize == at
            && build.text_sha256 == digest
        {
            dev_info!(dev, "GPU firmware build: {}\n", build.name);
            return Ok(Some(build));
        }
    }
    let prefix = u64::from_be_bytes(digest[..8].try_into().map_err(|_| EINVAL)?);
    let accepted = super::params::accept_text_prefix();
    if accepted != 0 && accepted == prefix {
        dev_warn!(
            dev,
            "GPU firmware build admitted by g15_accept_text_prefix: TEXT {:#x} DATA {:#x} GKTS {:#x} sha256 {}\n",
            size,
            data_size,
            at,
            Hex(&digest)
        );
        return Ok(None);
    }
    dev_err!(
        dev,
        "unsupported GPU firmware build: TEXT {:#x} DATA {:#x} GKTS {:#x} sha256 {}\n",
        size,
        data_size,
        at,
        Hex(&digest)
    );
    for build in builds {
        dev_err!(dev, "supported: {}\n", build.name);
    }
    Err(EINVAL)
}
