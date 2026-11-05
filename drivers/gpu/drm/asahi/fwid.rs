// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact GPU firmware identity.
//!
//! A build is the SHA-256 of the loaded TEXT with the 8 entropy bytes of its entropy records
//! (`GKTS` and `ECAP`, each a tag, the length 8 and the bytes) zeroed, plus the TEXT and DATA
//! sizes and the records' offsets. One TEXT region can hold several firmware roles back to back
//! (two GPU roles' TEXT loaded one after the other), each with its own pair of records. A build
//! can also name other ranges the boot loader fills (two sections it patches with the boot
//! arguments and the image header's load fields, which can change from boot to boot); they are
//! zeroed too, so the digest is that of the firmware's TEXT with the same ranges zeroed. An
//! unknown build is refused with its measured identity, so nothing is written to a firmware the
//! driver was not written for.

use core::ops::Range;
use kernel::{bindings, device, fmt, prelude::*};

/// A qualified GPU firmware build, identified by its loaded TEXT.
pub(crate) struct FirmwareBuild {
    pub(crate) name: &'static str,
    pub(crate) text_size: u64,
    pub(crate) data_size: u64,
    /// Offsets of each role's GKTS and ECAP entropy records in TEXT, in TEXT order; their 8
    /// entropy bytes are zeroed before hashing.
    pub(crate) entropy: &'static [[u32; 2]],
    /// Other (offset, length) ranges of TEXT the boot loader fills, zeroed before hashing.
    pub(crate) boot_filled: &'static [(u32, u32)],
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
    let map = unsafe { bindings::memremap(phys, size, bindings::MEMREMAP_WC as usize) };
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

/// Entropy record tags, in TEXT order within a role.
const ENTROPY_TAGS: [[u8; 4]; 2] = [*b"GKTS", *b"ECAP"];

/// Firmware roles one TEXT region can hold.
const MAX_ROLES: usize = 2;

/// The record (tag, then the length 8) that holds a role's entropy of `tag`.
fn record(tag: [u8; 4]) -> [u8; 8] {
    let mut record = [0u8; 8];
    record[..4].copy_from_slice(&tag);
    record[4..].copy_from_slice(&8u32.to_le_bytes());
    record
}

/// Whether `image` holds a whole entropy record of `tag` at `off`: the tag, the length 8 and the
/// 8 entropy bytes, all inside `image`.
fn record_at(image: &[u8], off: usize, tag: [u8; 4]) -> bool {
    off.checked_add(16)
        .and_then(|end| image.get(off..end))
        .is_some_and(|whole| whole[..8] == record(tag))
}

/// The SHA-256 of `image` with the 8 entropy bytes of the records at `at` (one GKTS/ECAP pair
/// per role) and the `filled` ranges zeroed, or `None` if a record or range is not there.
/// `image` is left as it was.
fn digest_at(image: &mut [u8], at: &[[u32; 2]], filled: &[(u32, u32)]) -> Option<[u8; 32]> {
    if at.is_empty() || at.len() > MAX_ROLES {
        return None;
    }
    if !filled.is_empty() {
        for (off, tag) in at.iter().flat_map(|pair| pair.iter().zip(ENTROPY_TAGS)) {
            if !record_at(image, *off as usize, tag) {
                return None;
            }
        }
        let mut copy = KVVec::from_elem(0u8, image.len(), GFP_KERNEL).ok()?;
        copy.copy_from_slice(image);
        for off in at.iter().flatten() {
            copy[*off as usize + 8..*off as usize + 16].fill(0);
        }
        for &(off, len) in filled {
            let off = off as usize;
            copy.get_mut(off..off.checked_add(len as usize)?)?.fill(0);
        }
        return Some(sha256(&copy));
    }
    let records = || {
        at.iter()
            .flat_map(|pair| pair.iter().map(|&off| off as usize).zip(ENTROPY_TAGS))
    };
    let mut saved = [[0u8; 8]; 2 * MAX_ROLES];
    for ((off, tag), save) in records().zip(saved.iter_mut()) {
        if !record_at(image, off, tag) {
            return None;
        }
        save.copy_from_slice(&image[off + 8..off + 16]);
    }
    for (off, _) in records() {
        image[off + 8..off + 16].fill(0);
    }
    let digest = sha256(image);
    for ((off, _), save) in records().zip(saved) {
        image[off + 8..off + 16].copy_from_slice(&save);
    }
    Some(digest)
}

/// The entropy records of each role found in `image`, to report an unknown build: each GKTS
/// record with the first ECAP record after it, at most [`MAX_ROLES`] pairs.
struct Records {
    at: [[u32; 2]; MAX_ROLES],
    roles: usize,
}

impl Records {
    fn find(image: &[u8]) -> Option<Records> {
        let [gkts, ecap] = ENTROPY_TAGS.map(record);
        let mut found = Records {
            at: [[0; 2]; MAX_ROLES],
            roles: 0,
        };
        let mut from = 0;
        while found.roles < MAX_ROLES {
            let Some(g) = image.get(from..)?.windows(8).position(|w| w == gkts) else {
                break;
            };
            let g = from + g;
            let Some(e) = image.get(g + 8..)?.windows(8).position(|w| w == ecap) else {
                break;
            };
            let e = g + 8 + e;
            // A record whose entropy bytes run past TEXT is no record ([`record_at`]).
            if !record_at(image, g, ENTROPY_TAGS[0]) || !record_at(image, e, ENTROPY_TAGS[1]) {
                break;
            }
            found.at[found.roles] = [u32::try_from(g).ok()?, u32::try_from(e).ok()?];
            found.roles += 1;
            from = e + 8;
        }
        (found.roles > 0).then_some(found)
    }

    fn as_slice(&self) -> &[[u32; 2]] {
        &self.at[..self.roles]
    }
}

/// Finds an exact identity, measuring each candidate at its own entropy offsets.
fn lookup(
    builds: &'static [FirmwareBuild],
    text_size: u64,
    data_size: u64,
    mut digest: impl FnMut(&FirmwareBuild) -> Option<[u8; 32]>,
) -> Option<&'static FirmwareBuild> {
    builds.iter().find(|build| {
        build.text_size == text_size
            && build.data_size == data_size
            && digest(build) == Some(build.text_sha256)
    })
}

/// A private, stable copy of a loaded TEXT, with the size of its DATA.
pub(crate) struct Measured {
    text: KVVec<u8>,
    data_size: u64,
}

/// Copies the TEXT loaded at `text` (DATA `data_size` bytes) twice, before any firmware-visible
/// write, and keeps the copy if both agree (`EAGAIN` if the TEXT changed meanwhile).
pub(crate) fn measure(text: Range<u64>, data_size: u64) -> Result<Measured> {
    let size = usize::try_from(text.end.checked_sub(text.start).ok_or(EINVAL)?)?;
    let copy = copy_physical(text.start, size)?;
    let again = copy_physical(text.start, size)?;
    if copy.as_slice() != again.as_slice() {
        return Err(EAGAIN);
    }
    Ok(Measured {
        text: copy,
        data_size,
    })
}

impl Measured {
    /// The build among `builds` this TEXT is exactly, if any.
    pub(crate) fn find(
        &mut self,
        builds: &'static [FirmwareBuild],
    ) -> Option<&'static FirmwareBuild> {
        let text_size = self.text.len() as u64;
        let text = &mut self.text;
        lookup(builds, text_size, self.data_size, |build| {
            digest_at(&mut text[..], build.entropy, build.boot_filled)
        })
    }

    /// The measured identity, to report a build that is not admitted: normalized as the first
    /// of `builds` with this TEXT size would be, else with only its entropy records zeroed.
    pub(crate) fn identity(
        &mut self,
        builds: impl IntoIterator<Item = &'static FirmwareBuild>,
    ) -> Identity {
        let text_size = self.text.len() as u64;
        let filled = builds
            .into_iter()
            .find(|build| build.text_size == text_size && !build.boot_filled.is_empty())
            .map_or(&[][..], |build| build.boot_filled);
        let records = Records::find(&self.text);
        let digest = records
            .as_ref()
            .and_then(|r| digest_at(&mut self.text, r.as_slice(), filled));
        Identity {
            text_size,
            data_size: self.data_size,
            filled: filled.len(),
            measured: records.zip(digest),
        }
    }
}

/// A measured identity, as a pinned [`FirmwareBuild`] would state it.
pub(crate) struct Identity {
    text_size: u64,
    data_size: u64,
    /// How many boot-filled ranges were zeroed besides the entropy.
    filled: usize,
    measured: Option<(Records, [u8; 32])>,
}

impl fmt::Display for Identity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "TEXT {:#x} DATA {:#x}", self.text_size, self.data_size)?;
        let Some((records, digest)) = &self.measured else {
            return write!(f, " has no entropy records");
        };
        write!(f, " entropy")?;
        for (i, [gkts, ecap]) in records.as_slice().iter().enumerate() {
            write!(
                f,
                "{}{:#x}/{:#x}",
                if i == 0 { " " } else { "," },
                gkts,
                ecap
            )?;
        }
        if self.filled != 0 {
            write!(f, " ({} boot-filled ranges zeroed)", self.filled)?;
        }
        write!(f, " sha256 ")?;
        fmt::Display::fmt(&Hex(digest), f)
    }
}

/// Identifies the firmware build loaded at `text` (DATA `data_size` bytes) among `builds`,
/// before any firmware-visible write.
#[cfg_attr(not(CONFIG_DRM_ASAHI_G16), allow(dead_code))]
pub(crate) fn identify(
    dev: &device::Device,
    text: Range<u64>,
    data_size: u64,
    builds: &'static [FirmwareBuild],
) -> Result<&'static FirmwareBuild> {
    let mut measured = measure(text, data_size).inspect_err(|e| {
        if *e == EAGAIN {
            dev_err!(dev, "GPU firmware TEXT changed while it was measured\n");
        }
    })?;
    if let Some(build) = measured.find(builds) {
        dev_info!(dev, "GPU firmware build: {}\n", build.name);
        return Ok(build);
    }
    dev_err!(
        dev,
        "unsupported GPU firmware build: {}\n",
        measured.identity(builds)
    );
    for build in builds {
        dev_err!(dev, "supported: {}\n", build.name);
    }
    Err(EINVAL)
}

#[kunit_tests(asahi_fwid)]
mod tests {
    use super::*;

    /// `roles` GKTS/ECAP pairs, at 0/32 within each 64 bytes.
    fn image<const N: usize>() -> [u8; N] {
        let mut image = [0x5a; N];
        for role in 0..N / 64 {
            for (off, tag) in [0usize, 32].into_iter().zip(ENTROPY_TAGS) {
                image[role * 64 + off..role * 64 + off + 8].copy_from_slice(&record(tag));
            }
        }
        image
    }

    #[test]
    fn entropy_is_normalized_without_changing_the_image() {
        let mut first = image::<64>();
        let original = first;
        let digest = digest_at(&mut first, &[[0, 32]], &[]).unwrap();
        assert_eq!(first, original);
        let mut second = first;
        second[8..16].fill(0xff);
        second[40..48].fill(0x01);
        assert_eq!(digest_at(&mut second, &[[0, 32]], &[]), Some(digest));
        second[63] ^= 1;
        assert_ne!(digest_at(&mut second, &[[0, 32]], &[]), Some(digest));
    }

    #[test]
    fn every_role_is_normalized() {
        let mut first = image::<128>();
        let original = first;
        let both = &[[0, 32], [64, 96]];
        let digest = digest_at(&mut first, both, &[]).unwrap();
        assert_eq!(first, original);
        let mut second = first;
        second[72..80].fill(0xff);
        second[104..112].fill(0x01);
        assert_eq!(digest_at(&mut second, both, &[]), Some(digest));
        // The second role's entropy is part of the digest unless its records are named.
        assert_ne!(
            digest_at(&mut second, &[[0, 32]], &[]),
            digest_at(&mut first, &[[0, 32]], &[])
        );
        second[127] ^= 1;
        assert_ne!(digest_at(&mut second, both, &[]), Some(digest));
        let found = Records::find(&first).unwrap();
        assert_eq!(found.as_slice(), both);
        assert_eq!(
            digest_at(&mut first, &[[0, 32], [64, 96], [0, 32]], &[]),
            None
        );
    }

    #[test]
    fn boot_filled_ranges_are_normalized() {
        let mut first = image::<128>();
        let original = first;
        let filled = &[(16, 8), (100, 28)];
        let digest = digest_at(&mut first, &[[0, 32]], filled).unwrap();
        assert_eq!(first, original);
        let mut second = first;
        second[16..24].fill(0x11);
        second[100..128].fill(0x22);
        assert_eq!(digest_at(&mut second, &[[0, 32]], filled), Some(digest));
        second[99] ^= 1;
        assert_ne!(digest_at(&mut second, &[[0, 32]], filled), Some(digest));
        assert_eq!(digest_at(&mut first, &[[0, 32]], &[(120, 9)]), None);
    }

    #[test]
    fn a_record_without_its_entropy_bytes_is_refused() {
        // The role's ECAP record is in the last 8 bytes, so it has no entropy bytes.
        let mut tail = image::<64>();
        tail[32..40].fill(0x5a);
        tail[56..64].copy_from_slice(&record(ENTROPY_TAGS[1]));
        let original = tail;
        assert_eq!(digest_at(&mut tail, &[[0, 56]], &[(16, 8)]), None);
        assert_eq!(digest_at(&mut tail, &[[0, 56]], &[]), None);
        assert_eq!(tail, original);
        assert!(Records::find(&tail).is_none());
        // An unknown build with a pinned build's TEXT size is reported, not a panic.
        static PINNED: [FirmwareBuild; 1] = [FirmwareBuild {
            name: "pinned",
            text_size: 64,
            data_size: 0,
            entropy: &[[0, 32]],
            boot_filled: &[(16, 8)],
            text_sha256: [0; 32],
        }];
        let mut text = KVVec::from_elem(0u8, tail.len(), GFP_KERNEL).unwrap();
        text.copy_from_slice(&tail);
        let mut measured = Measured { text, data_size: 0 };
        assert!(measured.find(&PINNED).is_none());
        assert!(measured.identity(&PINNED).measured.is_none());
    }

    #[test]
    fn malformed_records_are_refused_without_changing_the_image() {
        let mut image = image::<64>();
        assert_eq!(digest_at(&mut image, &[[0, 56]], &[]), None);
        assert_eq!(digest_at(&mut image, &[], &[]), None);
        image[36] = 7;
        let original = image;
        assert_eq!(digest_at(&mut image, &[[0, 32]], &[]), None);
        assert_eq!(image, original);
        assert!(Records::find(&image).is_none());
    }

    fn measured(
        builds: &'static [FirmwareBuild],
        data_size: u64,
        entropy: &[[u32; 2]],
        digest: [u8; 32],
    ) -> Option<&'static FirmwareBuild> {
        lookup(builds, 0x60000, data_size, |build| {
            (build.entropy == entropy).then_some(digest)
        })
    }

    #[test]
    fn t6040_accepts_all_loaded_images() {
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
            [
                0xd7, 0x25, 0x26, 0x4d, 0x48, 0x31, 0xb9, 0x5a, 0x5b, 0x8a, 0x37, 0x5a, 0xbf, 0x2f,
                0xb5, 0xcd, 0xfb, 0xd6, 0xd7, 0xcb, 0xc7, 0xf1, 0xba, 0xa2, 0xcc, 0x8b, 0x37, 0xee,
                0x5d, 0x6e, 0x5e, 0x98,
            ],
        ];
        for (i, digest) in digests.into_iter().enumerate() {
            let found = measured(build.images, 0x138000, &[[0x59839, 0x5991c]], digest).unwrap();
            assert!(core::ptr::eq(found, &build.images[i]));
            assert!(measured(build.images, 0x13c000, &[[0x59839, 0x5991c]], digest).is_none());
            assert!(measured(build.images, 0x138000, &[[0x5c8df, 0x5c9c2]], digest).is_none());
            assert!(lookup(build.images, 0x60001, 0x138000, |_| Some(digest)).is_none());
        }
        assert!(measured(build.images, 0x138000, &[[0x59839, 0x5991c]], [0; 32]).is_none());
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
            let found = measured(builds, 0x13c000, &[[0x5c8df, 0x5c9c2]], digest).unwrap();
            assert!(core::ptr::eq(found, &builds[i]));
            assert!(measured(builds, 0x138000, &[[0x5c8df, 0x5c9c2]], digest).is_none());
        }
        assert!(measured(builds, 0x13c000, &[[0x5c8df, 0x5c9c2]], [0; 32]).is_none());
    }
}
