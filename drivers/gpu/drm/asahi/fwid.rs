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

/// Finds an exact identity, measuring each candidate at its own entropy offsets.
fn lookup(
    builds: &'static [FirmwareBuild],
    text_size: u64,
    data_size: u64,
    mut digest: impl FnMut([u32; 2]) -> Option<[u8; 32]>,
) -> Option<&'static FirmwareBuild> {
    builds.iter().find(|build| {
        build.text_size == text_size
            && build.data_size == data_size
            && digest(build.entropy) == Some(build.text_sha256)
    })
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
    if let Some(build) = lookup(builds, size as u64, data_size, |at| {
        digest_at(&mut copy, at)
    }) {
        dev_info!(dev, "GPU firmware build: {}\n", build.name);
        return Ok(build);
    }
    let Some((entropy, digest)) =
        find_entropy(&copy).and_then(|at| Some((at, digest_at(&mut copy, at)?)))
    else {
        dev_err!(
            dev,
            "unsupported GPU firmware build: TEXT {:#x} has no entropy records\n",
            size
        );
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

#[kunit_tests(asahi_fwid)]
mod tests {
    use super::*;

    fn image() -> [u8; 64] {
        let mut image = [0x5a; 64];
        for (off, tag) in [0usize, 32].into_iter().zip(ENTROPY_TAGS) {
            image[off..off + 4].copy_from_slice(&tag);
            image[off + 4..off + 8].copy_from_slice(&8u32.to_le_bytes());
        }
        image
    }

    #[test]
    fn entropy_is_normalized_without_changing_the_image() {
        let mut first = image();
        let original = first;
        let digest = digest_at(&mut first, [0, 32]).unwrap();
        assert_eq!(first, original);
        let mut second = first;
        second[8..16].fill(0xff);
        second[40..48].fill(0x01);
        assert_eq!(digest_at(&mut second, [0, 32]), Some(digest));
        second[63] ^= 1;
        assert_ne!(digest_at(&mut second, [0, 32]), Some(digest));
    }

    #[test]
    fn malformed_records_are_refused_without_changing_the_image() {
        let mut image = image();
        assert_eq!(digest_at(&mut image, [0, 56]), None);
        image[36] = 7;
        let original = image;
        assert_eq!(digest_at(&mut image, [0, 32]), None);
        assert_eq!(image, original);
        assert_eq!(find_entropy(&image), None);
    }

    fn measured(
        builds: &'static [FirmwareBuild],
        data_size: u64,
        entropy: [u32; 2],
        digest: [u8; 32],
    ) -> Option<&'static FirmwareBuild> {
        lookup(builds, 0x60000, data_size, |at| {
            (at == entropy).then_some(digest)
        })
    }

    #[test]
    fn t6040_accepts_both_loaded_images() {
        let crate::g17::fw::variant::Firmware::Identified(builds) =
            crate::hw::t6040::CONFIG.firmware
        else {
            panic!("T6040 must identify its loaded firmware");
        };
        let build = builds.iter().find(|b| b.os_version == [26, 6, 2]).unwrap();
        let digests = [
            [
                0x03, 0xdf, 0x75, 0x48, 0x4a, 0x7f, 0xdd, 0x6e, 0x4b, 0x47, 0x7f, 0x95, 0x6c, 0xaa,
                0x5b, 0x44, 0x06, 0xdf, 0xff, 0x4d, 0x19, 0x17, 0x28, 0x62, 0x7f, 0x53, 0x10, 0xe0,
                0x63, 0x87, 0x6d, 0xcd,
            ],
            [
                0x19, 0x0c, 0xef, 0x19, 0x46, 0xa2, 0x3d, 0xfb, 0xde, 0x18, 0x05, 0x92, 0x62, 0x96,
                0x31, 0xc3, 0xb2, 0x8d, 0x05, 0x09, 0xa1, 0x7e, 0xb7, 0x61, 0x08, 0xb7, 0xc9, 0xa5,
                0x2a, 0xcc, 0xd9, 0xac,
            ],
        ];
        for (i, digest) in digests.into_iter().enumerate() {
            let found = measured(build.images, 0x138000, [0x59839, 0x5991c], digest).unwrap();
            assert!(core::ptr::eq(found, &build.images[i]));
            assert!(measured(build.images, 0x13c000, [0x59839, 0x5991c], digest).is_none());
            assert!(measured(build.images, 0x138000, [0x5c8df, 0x5c9c2], digest).is_none());
            assert!(lookup(build.images, 0x60001, 0x138000, |_| Some(digest)).is_none());
        }
        assert!(measured(build.images, 0x138000, [0x59839, 0x5991c], [0; 32]).is_none());
    }

    #[cfg(CONFIG_DRM_ASAHI_G16)]
    #[test]
    fn j713_accepts_both_loaded_images() {
        let builds = crate::g16::hw::t8132::CONFIG.firmware;
        let digests = [
            [
                0x5f, 0x65, 0xf6, 0x14, 0x58, 0x72, 0x54, 0x70, 0xb4, 0xb0, 0xc0, 0x20, 0x28, 0x47,
                0x7b, 0xf2, 0xea, 0x57, 0x3a, 0xfb, 0x4e, 0x0c, 0x49, 0x6e, 0xf9, 0x95, 0xe9, 0x6a,
                0xff, 0xb0, 0xd8, 0x6c,
            ],
            [
                0x3b, 0x36, 0x8e, 0xe4, 0x2d, 0xcc, 0xc6, 0x26, 0xe8, 0x49, 0x67, 0xff, 0x40, 0x87,
                0x64, 0x71, 0x0a, 0x64, 0xbf, 0xf1, 0x59, 0x4f, 0x3c, 0xfd, 0x2c, 0x6e, 0x4d, 0x34,
                0xd0, 0x65, 0x24, 0x8b,
            ],
        ];
        for (i, digest) in digests.into_iter().enumerate() {
            let found = measured(builds, 0x13c000, [0x5c8df, 0x5c9c2], digest).unwrap();
            assert!(core::ptr::eq(found, &builds[i]));
            assert!(measured(builds, 0x138000, [0x5c8df, 0x5c9c2], digest).is_none());
        }
        assert!(measured(builds, 0x13c000, [0x5c8df, 0x5c9c2], [0; 32]).is_none());
    }
}
