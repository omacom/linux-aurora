// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! File-backed persistence for the SEP catacombs.
//!
//! A catacomb is the opaque identity blob the enclave returns from `0x6c`; each
//! lives in its own variable-length file, too large for the block store.

use crate::shim;
use crate::store::crc16_ccitt_false;
use kernel::prelude::*;

const MAGIC: [u8; 8] = *b"SEPCTMB1";
/// magic(8) + len(4, LE) + crc(2, LE) + pad(2)
const HEADER: usize = 16;
const MAX_CATACOMB: usize = 1 << 20;

fn path_for(kind: u8) -> Option<&'static CStr> {
    Some(match kind {
        crate::PRIVATE_TYPE_CATACOMB_MASTER => c"/var/lib/apple-sep-catacomb-master.bin",
        crate::PRIVATE_TYPE_CATACOMB_OWNER => c"/var/lib/apple-sep-catacomb-owner.bin",
        crate::PRIVATE_TYPE_CATACOMB_USER => c"/var/lib/apple-sep-catacomb-user.bin",
        _ => return None,
    })
}

pub(crate) fn is_kind(kind: u8) -> bool {
    path_for(kind).is_some()
}

pub(crate) fn write(kind: u8, blob: &[u8]) -> Result<()> {
    let path = path_for(kind).ok_or(EINVAL)?;
    if blob.len() > MAX_CATACOMB {
        return Err(ENOSPC);
    }
    let mut head = [0u8; HEADER];
    head[..MAGIC.len()].copy_from_slice(&MAGIC);
    head[8..12].copy_from_slice(&(blob.len() as u32).to_le_bytes());
    head[12..14].copy_from_slice(&crc16_ccitt_false(blob).to_le_bytes());

    let file = shim::StoreFile::open_trunc(path)?;
    file.write_all(0, &head)?;
    if !blob.is_empty() {
        file.write_all(HEADER as u64, blob)?;
    }
    file.sync()
}

/// `Ok(None)` only when the file is missing. A file that exists but cannot be
/// read or fails validation is an error: "cannot tell" must never read as
/// "nothing saved", or a caller may select a fresh context over a saved one.
pub(crate) fn read(kind: u8) -> Result<Option<KVec<u8>>> {
    let path = path_for(kind).ok_or(EINVAL)?;
    let file = match shim::StoreFile::open_readonly(path) {
        Ok(file) => file,
        Err(e) if e == ENOENT => return Ok(None),
        Err(e) => return Err(e),
    };
    let size = file.size()?;
    if size < HEADER as u64 {
        return Err(EINVAL);
    }
    let mut head = [0u8; HEADER];
    file.read_exact(0, &mut head)?;
    if head[..MAGIC.len()] != MAGIC {
        return Err(EINVAL);
    }
    let len = u32::from_le_bytes([head[8], head[9], head[10], head[11]]) as usize;
    let crc = u16::from_le_bytes([head[12], head[13]]);
    if len == 0 || len > MAX_CATACOMB || size != (HEADER + len) as u64 {
        return Err(EINVAL);
    }
    let mut blob: KVec<u8> = KVec::with_capacity(len, GFP_KERNEL)?;
    blob.resize(len, 0, GFP_KERNEL)?;
    file.read_exact(HEADER as u64, &mut blob)?;
    if crc16_ccitt_false(&blob) != crc {
        return Err(EINVAL);
    }
    Ok(Some(blob))
}

/// True only for a present, valid file.
pub(crate) fn is_saved(kind: u8) -> bool {
    matches!(read(kind), Ok(Some(_)))
}

/// True only when the file is provably missing.
pub(crate) fn is_absent(kind: u8) -> bool {
    matches!(read(kind), Ok(None))
}
