// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact firmware and hardware identity.
//!
//! The firmware build is the SHA-256 of TEXT with the 8 entropy bytes after the single
//! `GKTS\x08\0\0\0` marker zeroed, plus the segment sizes, looked up in the SoC
//! configuration's `firmware` list; the hardware revision is checked against its `hw_ids` at
//! start ([`super::boot`]). An unknown build or revision is refused with its measured identity;
//! an unknown build's refusal also says what to report so that it can be pinned. The pure
//! lookup is in [`lookup`] (host-tested).
//! The canonical object VAs, the initdata graph, the board images and the work layouts all
//! belong to one build, so nothing is written before the build is identified.
//!
//! Bring-up escape hatch: `g15_accept_text_prefix=<hex>` admits an unpinned build whose digest
//! starts with those 8 bytes (the full digest and marker offset are logged, so that the build
//! can be pinned by the next kernel).

use kernel::{bindings, device, fmt, prelude::*};

use super::boot::glue;
use super::hw::FirmwareBuild;

mod lookup;
mod matching;

use lookup::{Measured, Verdict};

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

impl From<&FirmwareBuild> for Measured {
    fn from(build: &FirmwareBuild) -> Self {
        Measured {
            text_size: build.text_size,
            data_size: build.data_size,
            marker: build.marker,
            sha256: build.text_sha256,
        }
    }
}

/// Identifies the loaded firmware build: the pinned build, or `None` for a build admitted by
/// its digest prefix (`g15_accept_text_prefix`).
/// With `allow_prefix` false, only a complete pinned identity may be admitted.
pub(crate) fn firmware(
    dev: &device::Device,
    adt: &super::adt::GpuAdt,
    builds: &'static [FirmwareBuild],
    allow_prefix: bool,
) -> Result<Option<&'static FirmwareBuild>> {
    let size = usize::try_from(adt.text.end - adt.text.start)?;
    let data_size = adt.data.end - adt.data.start;
    let mut copy = copy_physical(adt.text.start, size)?;
    let at = lookup::marker(&copy).map_err(|found| {
        dev_err!(
            dev,
            "GPU firmware TEXT has {} GKTS markers, need one\n",
            found
        );
        EINVAL
    })?;
    copy[at + 8..at + 16].fill(0);
    let m = Measured {
        text_size: size as u64,
        data_size,
        marker: u32::try_from(at)?,
        sha256: glue::sha256(&copy),
    };
    // A qualified build pins only on the machine it names.
    let qualified = builds
        .iter()
        .map(|b| (Measured::from(b), b.machine.map_or(true, glue::machine)));
    // A profile may require exact identities even after its first build is
    // pinned. Empty profiles remain measurement-only in either policy.
    let accept_prefix = if !allow_prefix || builds.is_empty() {
        0
    } else {
        super::params::accept_text_prefix()
    };
    match lookup::verdict(&m, qualified, accept_prefix) {
        Verdict::Pinned(i) => {
            let build = builds.get(i).ok_or(EINVAL)?;
            dev_info!(dev, "GPU firmware build: {}\n", build.name);
            return Ok(Some(build));
        }
        Verdict::Admitted => {
            dev_warn!(
                dev,
                "GPU firmware build admitted by g15_accept_text_prefix: TEXT {:#x} DATA {:#x} GKTS {:#x} sha256 {}\n",
                size,
                data_size,
                at,
                Hex(&m.sha256)
            );
            return Ok(None);
        }
        Verdict::Refused => {}
    }
    dev_err!(
        dev,
        "unsupported GPU firmware build: TEXT {:#x} DATA {:#x} GKTS {:#x} sha256 {}; {}\n",
        size,
        data_size,
        at,
        Hex(&m.sha256),
        lookup::REPORT_HINT
    );
    for build in builds {
        dev_err!(dev, "supported: {}\n", build.name);
    }
    Err(EINVAL)
}
