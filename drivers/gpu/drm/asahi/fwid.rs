// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact GPU firmware identity.
//!
//! A build is the SHA-256 of the loaded TEXT with the 8 entropy bytes of its two entropy records
//! (`GKTS` and `ECAP`, each a tag, the length 8 and the bytes) zeroed, plus the TEXT and DATA
//! sizes and the records' offsets. An unknown build is refused with its
//! measured identity, so nothing is written to a firmware the driver was not written for.

use core::ops::Range;
use kernel::{bindings, device, fmt, prelude::*};

/// A qualified GPU firmware build, identified by its loaded TEXT.
pub(crate) struct FirmwareBuild {
    pub(crate) name: &'static str,
    pub(crate) text_size: u64,
    pub(crate) data_size: u64,
    /// Offsets of the GKTS and ECAP entropy records in TEXT; their 8 entropy bytes are zeroed
    /// before hashing.
    pub(crate) entropy: [u32; 2],
    pub(crate) text_sha256: [u8; 32],
}

extern "C" {
    fn asahi_fw_sha256(data: *const u8, len: usize, out: *mut u8);
}

/// SHA-256 (`sha256()`).
pub(crate) fn sha256(data: &[u8]) -> [u8; 32] {
    let mut out = [0u8; 32];
    // SAFETY: `data` is valid for its length and `out` holds the 32-byte digest.
    unsafe { asahi_fw_sha256(data.as_ptr(), data.len(), out.as_mut_ptr()) };
    out
}

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
/// private copy through a temporary write-combined mapping.
pub(crate) fn copy_physical(phys: u64, size: usize) -> Result<KVVec<u8>> {
    // SAFETY: memremap() of a boot-loader range creates a temporary mapping; it has no other
    // side effects.
    let map = unsafe { bindings::memremap(phys, size, bindings::MEMREMAP_WC as _) };
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

/// Entropy record tags, in TEXT order.
const ENTROPY_TAGS: [[u8; 4]; 2] = [*b"GKTS", *b"ECAP"];

/// The SHA-256 of `image` with the 8 entropy bytes of the records at `at` zeroed, or `None` if a
/// record (tag, then the length 8) is not there. `image` is left as it was.
fn digest_at(image: &mut [u8], at: [u32; 2]) -> Option<[u8; 32]> {
    let mut saved = [[0u8; 8]; 2];
    for ((off, tag), save) in at.into_iter().zip(ENTROPY_TAGS).zip(saved.iter_mut()) {
        let off = off as usize;
        if image.get(off..off + 4) != Some(tag.as_slice())
            || image.get(off + 4..off + 8) != Some(8u32.to_le_bytes().as_slice())
        {
            return None;
        }
        save.copy_from_slice(image.get(off + 8..off + 16)?);
    }
    for off in at {
        image[off as usize + 8..off as usize + 16].fill(0);
    }
    let digest = sha256(image);
    for (off, save) in at.into_iter().zip(saved) {
        image[off as usize + 8..off as usize + 16].copy_from_slice(&save);
    }
    Some(digest)
}

/// The first record of each tag in `image`, to report an unknown build.
fn find_entropy(image: &[u8]) -> Option<[u32; 2]> {
    let mut found = [0u32; 2];
    for (slot, tag) in found.iter_mut().zip(ENTROPY_TAGS) {
        let mut record = [0u8; 8];
        record[..4].copy_from_slice(&tag);
        record[4..].copy_from_slice(&8u32.to_le_bytes());
        *slot = u32::try_from(image.windows(8).position(|w| w == record)?).ok()?;
    }
    Some(found)
}

/// Identifies the firmware build loaded at `text` (DATA `data_size` bytes) among `builds`,
/// before any firmware-visible write.
pub(crate) fn identify(
    dev: &device::Device,
    text: Range<u64>,
    data_size: u64,
    builds: &'static [FirmwareBuild],
) -> Result<&'static FirmwareBuild> {
    let size = usize::try_from(text.end - text.start)?;
    let mut copy = copy_physical(text.start, size)?;
    let again = copy_physical(text.start, size)?;
    if copy.as_slice() != again.as_slice() {
        dev_err!(dev, "GPU firmware TEXT changed while it was measured\n");
        return Err(EAGAIN);
    }
    for build in builds {
        if build.text_size == size as u64
            && build.data_size == data_size
            && digest_at(&mut copy, build.entropy) == Some(build.text_sha256)
        {
            dev_info!(dev, "GPU firmware build: {}\n", build.name);
            return Ok(build);
        }
    }
    let Some((entropy, digest)) =
        find_entropy(&copy).and_then(|at| Some((at, digest_at(&mut copy, at)?)))
    else {
        dev_err!(dev, "unsupported GPU firmware build: TEXT {:#x} has no entropy records\n", size);
        return Err(EINVAL);
    };
    dev_err!(
        dev,
        "unsupported GPU firmware build: TEXT {:#x} DATA {:#x} entropy {:#x}/{:#x} sha256 {}\n",
        size,
        data_size,
        entropy[0],
        entropy[1],
        Hex(&digest)
    );
    for build in builds {
        dev_err!(dev, "supported: {}\n", build.name);
    }
    Err(EINVAL)
}
