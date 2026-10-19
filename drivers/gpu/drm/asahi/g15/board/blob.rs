// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Board images from a firmware file.
//!
//! A SoC whose board images are not built-in templates names a firmware file
//! ([`super::super::hw::Board::Blob`]), built into the kernel with `CONFIG_EXTRA_FIRMWARE` or
//! loaded from `/lib/firmware`, and pins its SHA-256. The file holds the three images as the
//! firmware of that machine accepted them; the build then writes the pointers and policy words
//! it owns into them (TTBAT, IOMapping VAs, the timestamp arena, the idle policy).
//!
//! Layout, little-endian: the magic `T6031BRD`, u32 format version (1), u32 image count (3),
//! then per image `u32 kind, u32 size` with kinds 0 (HardwareData), 1 (Globals) and
//! 2 (Power), then the images' bytes in that order. Sizes must match the SoC's layout.

use kernel::{device, firmware::Firmware, prelude::*};

use super::super::boot::glue;
use super::super::identity::Hex;

const MAGIC: &[u8; 8] = b"T6031BRD";
const VERSION: u32 = 1;
const IMAGES: usize = 3;

/// The three images.
pub(crate) struct Images {
    pub(crate) hardware: KVVec<u8>,
    pub(crate) globals: KVVec<u8>,
    pub(crate) power: KVVec<u8>,
}

fn le32(b: &[u8], off: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(
        b.get(off..off + 4)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    ))
}

fn copy(b: &[u8], range: core::ops::Range<usize>) -> Result<KVVec<u8>> {
    let src = b.get(range).ok_or(EINVAL)?;
    let mut out = KVVec::from_elem(0u8, src.len(), GFP_KERNEL)?;
    out.copy_from_slice(src);
    Ok(out)
}

/// Requests `name`, verifies its digest against `sha256` and splits it into the images, each
/// of the size `sizes` gives for its kind.
pub(crate) fn load(
    dev: &device::Device,
    name: &CStr,
    sha256: &[u8; 32],
    sizes: [usize; IMAGES],
) -> Result<Images> {
    let fw = Firmware::request(name, dev).inspect_err(|e| {
        dev_err!(dev, "board images {}: not available ({:?})\n", name, e)
    })?;
    let data = fw.data();
    let digest = glue::sha256(data);
    if digest != *sha256 {
        dev_err!(
            dev,
            "board images {}: sha256 {} is not the pinned one\n",
            name,
            Hex(&digest)
        );
        return Err(EINVAL);
    }
    if data.len() < 16 || &data[..8] != MAGIC || le32(data, 8)? != VERSION {
        dev_err!(dev, "board images {}: unknown header\n", name);
        return Err(EINVAL);
    }
    if le32(data, 12)? as usize != IMAGES {
        dev_err!(dev, "board images {}: not {} images\n", name, IMAGES);
        return Err(EINVAL);
    }
    let mut at = 16 + IMAGES * 8;
    let mut images: [Option<KVVec<u8>>; IMAGES] = [None, None, None];
    for i in 0..IMAGES {
        let kind = le32(data, 16 + i * 8)? as usize;
        let size = le32(data, 16 + i * 8 + 4)? as usize;
        if kind != i || size != sizes[i] {
            dev_err!(
                dev,
                "board images {}: image {} is kind {} of {} bytes, need {}\n",
                name,
                i,
                kind,
                size,
                sizes[i]
            );
            return Err(EINVAL);
        }
        images[i] = Some(copy(data, at..at + size)?);
        at += size;
    }
    if at != data.len() {
        dev_err!(dev, "board images {}: trailing bytes\n", name);
        return Err(EINVAL);
    }
    let [hardware, globals, power] = images;
    dev_info!(
        dev,
        "board images {}: sha256 {} accepted\n",
        name,
        Hex(&digest)
    );
    Ok(Images {
        hardware: hardware.ok_or(EINVAL)?,
        globals: globals.ok_or(EINVAL)?,
        power: power.ok_or(EINVAL)?,
    })
}
