// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Narrow, read-only APFS lookup for the existing xART `.gl` file.
//!
//! This is deliberately not a general APFS implementation. Unsupported tree
//! shapes, encryption, fragmented files, and bad metadata fail closed. In
//! particular, finding an extent does not authorize raw writes to that file.

use crate::shim;
use kernel::prelude::*;

const BLOCK: usize = 4096;
const ROOT_INFO_SIZE: usize = 40;
const XART_ROLE: u16 = 0x100;
const FILE_SIZE: u64 = 0x600000;
const FS_UNENCRYPTED: u64 = 1;
const INODE_WAS_CLONED: u64 = 0x10;
const INODE_WAS_EVER_CLONED: u64 = 0x400;
const MOD: u64 = 0xffff_ffff;

fn le16(data: &[u8], off: usize) -> Result<u16> {
    Ok(u16::from_le_bytes(
        data.get(off..off + 2)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    ))
}

fn le32(data: &[u8], off: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(
        data.get(off..off + 4)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    ))
}

fn le64(data: &[u8], off: usize) -> Result<u64> {
    Ok(u64::from_le_bytes(
        data.get(off..off + 8)
            .ok_or(EINVAL)?
            .try_into()
            .map_err(|_| EINVAL)?,
    ))
}

/// APFS's Fletcher-64 over the little-endian words after the checksum field.
fn checksum(data: &[u8]) -> Result<u64> {
    if data.len() != BLOCK {
        return Err(EINVAL);
    }
    let mut s1 = 0u64;
    let mut s2 = 0u64;
    for word in data[8..].chunks_exact(4) {
        let value = u32::from_le_bytes(word.try_into().map_err(|_| EINVAL)?) as u64;
        s1 = (s1 + value) % MOD;
        s2 = (s2 + s1) % MOD;
    }
    let c1 = (MOD - (s1 + s2) % MOD) % MOD;
    let c2 = (MOD - (s1 + c1) % MOD) % MOD;
    Ok((c2 << 32) | c1)
}

fn read_object(file: &shim::StoreFile, size: u64, paddr: u64) -> Result<KVec<u8>> {
    let start = paddr.checked_mul(BLOCK as u64).ok_or(EINVAL)?;
    if start.checked_add(BLOCK as u64).ok_or(EINVAL)? > size {
        return Err(EINVAL);
    }
    let mut data = KVec::new();
    data.resize(BLOCK, 0, GFP_KERNEL)?;
    file.read_block_exact(start, &mut data)?;
    if le64(&data, 0)? != checksum(&data)? {
        return Err(EIO);
    }
    Ok(data)
}

/// The descriptor ring holds mapping blocks followed by its superblock. A
/// checksummed superblock alone is not a complete checkpoint: verify every
/// mapping and its ephemeral object before accepting that transaction.
fn valid_checkpoint(
    file: &shim::StoreFile,
    size: u64,
    sb: &[u8],
    sb_paddr: u64,
    desc_base: u64,
    desc_count: u64,
) -> Result<()> {
    if le64(sb, 112)? != desc_base || le32(sb, 104)? as u64 != desc_count {
        return Err(ENOTSUPP);
    }
    let index = le32(sb, 136)? as u64;
    let len = le32(sb, 140)? as u64;
    if index >= desc_count || len < 2 || len > desc_count {
        return Err(EINVAL);
    }
    let last = desc_base + (index + len - 1) % desc_count;
    if last != sb_paddr {
        return Err(EINVAL);
    }
    let xid = le64(sb, 16)?;
    for j in 0..len - 1 {
        let paddr = desc_base + (index + j) % desc_count;
        let map = read_object(file, size, paddr)?;
        if le32(&map, 24)? & 0xffff != 12 || le64(&map, 16)? != xid {
            return Err(EINVAL);
        }
        let flags = le32(&map, 32)?;
        if flags & 1 != u32::from(j == len - 2) {
            return Err(EINVAL);
        }
        let count = le32(&map, 36)? as usize;
        if count > (BLOCK - 40) / 40 {
            return Err(EINVAL);
        }
        for i in 0..count {
            let at = 40 + i * 40;
            if le32(&map, at + 8)? != BLOCK as u32 {
                return Err(ENOTSUPP);
            }
            let object = read_object(file, size, le64(&map, at + 32)?)?;
            if le64(&object, 8)? != le64(&map, at + 24)?
                || le64(&object, 16)? != xid
                || le32(&object, 24)? != le32(&map, at)?
            {
                return Err(EINVAL);
            }
        }
    }
    Ok(())
}

fn node_entry(node: &[u8], index: usize, fixed: bool) -> Result<(&[u8], &[u8])> {
    if node.len() != BLOCK || le16(node, 34)? != 0 {
        return Err(ENOTSUPP);
    }
    let flags = le16(node, 32)?;
    if (flags & 4 != 0) != fixed {
        return Err(ENOTSUPP);
    }
    let count = le32(node, 36)? as usize;
    if index >= count || count > 512 {
        return Err(EINVAL);
    }
    let toc = 56usize
        .checked_add(le16(node, 40)? as usize)
        .ok_or(EINVAL)?;
    let table_len = le16(node, 42)? as usize;
    let key_start = toc.checked_add(table_len).ok_or(EINVAL)?;
    let val_end = BLOCK - if flags & 1 != 0 { ROOT_INFO_SIZE } else { 0 };
    let stride = if fixed { 4 } else { 8 };
    if key_start > val_end || toc.checked_add(count * stride).ok_or(EINVAL)? > key_start {
        return Err(EINVAL);
    }
    let item = toc.checked_add(index * stride).ok_or(EINVAL)?;
    let (ko, kl, vo, vl) = if fixed {
        (
            le16(node, item)? as usize,
            16,
            le16(node, item + 2)? as usize,
            16,
        )
    } else {
        (
            le16(node, item)? as usize,
            le16(node, item + 2)? as usize,
            le16(node, item + 4)? as usize,
            le16(node, item + 6)? as usize,
        )
    };
    let ks = key_start.checked_add(ko).ok_or(EINVAL)?;
    let vs = val_end.checked_sub(vo).ok_or(EINVAL)?;
    let ke = ks.checked_add(kl).ok_or(EINVAL)?;
    let ve = vs.checked_add(vl).ok_or(EINVAL)?;
    if ks < key_start || ke > val_end || vs < key_start || ve > val_end {
        return Err(EINVAL);
    }
    Ok((&node[ks..ke], &node[vs..ve]))
}

fn omap_lookup(
    file: &shim::StoreFile,
    size: u64,
    omap_paddr: u64,
    oid: u64,
    xid: u64,
) -> Result<u64> {
    let omap = read_object(file, size, omap_paddr)?;
    if le64(&omap, 8)? != omap_paddr {
        return Err(EINVAL);
    }
    let tree_paddr = le64(&omap, 48)?;
    let tree = read_object(file, size, tree_paddr)?;
    if le64(&tree, 8)? != tree_paddr || le16(&tree, 34)? != 0 || le16(&tree, 32)? & 4 == 0 {
        return Err(ENOTSUPP);
    }
    let mut selected: Option<(u64, u32, u32, u64)> = None;
    for i in 0..le32(&tree, 36)? as usize {
        let (key, val) = node_entry(&tree, i, true)?;
        let key_xid = le64(key, 8)?;
        if le64(key, 0)? == oid && key_xid <= xid && selected.is_none_or(|v| key_xid > v.0) {
            selected = Some((key_xid, le32(val, 0)?, le32(val, 4)?, le64(val, 8)?));
        }
    }
    let (_, flags, object_size, paddr) = selected.ok_or(ENODATA)?;
    if flags & 1 != 0 || object_size != BLOCK as u32 {
        return Err(ENOTSUPP);
    }
    Ok(paddr)
}

fn is_gigalocker_name(name: &[u8]) -> bool {
    if name == b".gl" {
        return true;
    }
    if name.len() != 39 || &name[36..] != b".gl" {
        return false;
    }
    name[..36].iter().enumerate().all(|(i, byte)| {
        if matches!(i, 8 | 13 | 18 | 23) {
            *byte == b'-'
        } else {
            byte.is_ascii_hexdigit()
        }
    })
}

fn validate_write_ownership(
    file: &shim::StoreFile,
    size: u64,
    vol: &[u8],
    xid: u64,
    private_id: u64,
    inode_flags: u64,
    physical: u64,
) -> Result<()> {
    // This is deliberately narrower than general APFS write support. On the
    // tested J414s the .gl file has one unshared, unsnapshotted extent. Never
    // raw-write a clone, snapshot, pending revert, or unknown extent layout.
    if le64(vol, 264)? != FS_UNENCRYPTED
        || le64(vol, 216)? != 0
        || le64(vol, 160)? != 0
        || le64(vol, 168)? != 0
        || inode_flags & (INODE_WAS_CLONED | INODE_WAS_EVER_CLONED) != 0
    {
        return Err(ENOTSUPP);
    }
    let tree_paddr = le64(vol, 144)?;
    let tree = read_object(file, size, tree_paddr)?;
    if le64(&tree, 8)? != tree_paddr
        || le64(&tree, 16)? > xid
        || le32(&tree, 24)? != 0x4000_0002
        || le32(&tree, 28)? != 0x0f
        || le16(&tree, 32)? != 3
        || le16(&tree, 34)? != 0
    {
        return Err(ENOTSUPP);
    }
    // This is a volume-wide extent-reference tree, not a per-file tree.
    // Other files can have disjoint records alongside the gigalocker's.
    let count = le32(&tree, 36)? as usize;
    if count == 0 || count > 512 {
        return Err(ENOTSUPP);
    }
    let target_blocks = FILE_SIZE / BLOCK as u64;
    let target_end = physical.checked_add(target_blocks).ok_or(EINVAL)?;
    let mut previous_end = None;
    let mut matched = false;
    for i in 0..count {
        let (key, val) = node_entry(&tree, i, false)?;
        if key.len() != 8 || val.len() < 20 {
            return Err(ENOTSUPP);
        }
        let raw_key = le64(key, 0)?;
        let raw_len = le64(val, 0)?;
        let start = raw_key & ((1u64 << 60) - 1);
        let blocks = raw_len & ((1u64 << 60) - 1);
        let end = start.checked_add(blocks).ok_or(EINVAL)?;
        // Keep support narrow: ordinary allocated extents, sorted and
        // non-overlapping, wholly inside this checksummed container.
        if raw_key >> 60 != 2
            || raw_len >> 60 != 1
            || blocks == 0
            || end > size / BLOCK as u64
            || previous_end.is_some_and(|previous| start < previous)
            || le32(val, 16)? == 0
            || le32(val, 16)? > i32::MAX as u32
        {
            return Err(ENOTSUPP);
        }
        previous_end = Some(end);
        if start < target_end && physical < end {
            if matched
                || start != physical
                || blocks != target_blocks
                || le64(val, 8)? != private_id
                || le32(val, 16)? != 1
            {
                return Err(ENOTSUPP);
            }
            matched = true;
        }
    }
    if !matched {
        return Err(ENOTSUPP);
    }
    Ok(())
}

fn xart_extent(
    file: &shim::StoreFile,
    size: u64,
    vol: &[u8],
    xid: u64,
    writes_enabled: bool,
) -> Result<u64> {
    if le64(vol, 264)? & FS_UNENCRYPTED == 0 {
        return Err(ENOTSUPP);
    }
    let omap = le64(vol, 128)?;
    let root_oid = le64(vol, 136)?;
    let root_paddr = omap_lookup(file, size, omap, root_oid, xid)?;
    let root = read_object(file, size, root_paddr)?;
    if le64(&root, 8)? != root_oid || le16(&root, 34)? != 0 || le16(&root, 32)? & 4 != 0 {
        return Err(ENOTSUPP);
    }

    let count = le32(&root, 36)? as usize;
    let mut gl_file: Option<u64> = None;
    for i in 0..count {
        let (key, val) = node_entry(&root, i, false)?;
        if key.len() < 12 || val.len() < 8 || le64(key, 0)? != ((9u64 << 60) | 2) {
            continue;
        }
        let name_len = (le32(key, 8)? & 0x3ff) as usize;
        if name_len == 0 || 12 + name_len > key.len() || key[12 + name_len - 1] != 0 {
            return Err(EINVAL);
        }
        if is_gigalocker_name(&key[12..12 + name_len - 1]) {
            if gl_file.replace(le64(val, 0)?).is_some() {
                return Err(ENOTSUPP);
            }
        }
    }
    let gl_file = gl_file.ok_or(ENODATA)?;
    let mut private_id: Option<u64> = None;
    let mut inode_flags: Option<u64> = None;
    for i in 0..count {
        let (key, val) = node_entry(&root, i, false)?;
        if key.len() >= 8 && le64(key, 0)? == ((3u64 << 60) | gl_file) {
            if val.len() < 92 || private_id.replace(le64(val, 8)?).is_some() {
                return Err(EINVAL);
            }
            inode_flags = Some(le64(val, 48)?);
        }
    }
    let private_id = private_id.ok_or(ENODATA)?;
    let mut extent: Option<u64> = None;
    for i in 0..count {
        let (key, val) = node_entry(&root, i, false)?;
        if key.len() < 16 || le64(key, 0)? != ((8u64 << 60) | private_id) {
            continue;
        }
        if val.len() < 24 || le64(key, 8)? != 0 || le64(val, 0)? != FILE_SIZE || le64(val, 16)? != 0
        {
            return Err(ENOTSUPP);
        }
        let paddr = le64(val, 8)?;
        let start = paddr.checked_mul(BLOCK as u64).ok_or(EINVAL)?;
        if start.checked_add(FILE_SIZE).ok_or(EINVAL)? > size || extent.replace(start).is_some() {
            return Err(EINVAL);
        }
    }
    let extent = extent.ok_or(ENODATA)?;
    if writes_enabled {
        validate_write_ownership(
            file,
            size,
            vol,
            xid,
            private_id,
            inode_flags.ok_or(ENODATA)?,
            extent / BLOCK as u64,
        )?;
    }
    Ok(extent)
}

/// Return the byte offset of the sole contiguous 6 MiB `.gl` file extent.
pub(crate) fn locate(file: &shim::StoreFile, writes_enabled: bool) -> Result<u64> {
    let device_size = file.size()?;
    let base = read_object(file, device_size, 0)?;
    if &base[32..36] != b"NXSB" || le32(&base, 36)? != BLOCK as u32 {
        return Err(ENOTSUPP);
    }
    let blocks = le64(&base, 40)?;
    if blocks == 0
        || blocks > device_size / BLOCK as u64
        || le64(&base, 56)? != 0
        || le64(&base, 64)? != 2
    {
        return Err(ENOTSUPP);
    }
    let size = blocks.checked_mul(BLOCK as u64).ok_or(EINVAL)?;
    let desc_count_raw = le32(&base, 104)?;
    if desc_count_raw & (1 << 31) != 0 {
        return Err(ENOTSUPP);
    }
    let desc_count = desc_count_raw as u64;
    let desc_base = le64(&base, 112)?;
    if desc_count == 0
        || desc_count > 1024
        || desc_base.checked_add(desc_count).ok_or(EINVAL)? > size / BLOCK as u64
    {
        return Err(EINVAL);
    }
    let mut best: Option<KVec<u8>> = None;
    let mut best_xid = 0u64;
    for paddr in desc_base..desc_base + desc_count {
        let Ok(candidate) = read_object(file, size, paddr) else {
            continue;
        };
        let candidate_xid = le64(&candidate, 16)?;
        if &candidate[32..36] == b"NXSB"
            && le32(&candidate, 36)? == BLOCK as u32
            && le64(&candidate, 40)? == blocks
            && le64(&candidate, 56)? == 0
            && le64(&candidate, 64)? == 2
            && valid_checkpoint(file, size, &candidate, paddr, desc_base, desc_count).is_ok()
            && candidate_xid > best_xid
        {
            best_xid = candidate_xid;
            best = Some(candidate);
        }
    }
    let best = best.ok_or(ENODATA)?;
    let xid = le64(&best, 16)?;
    let omap = le64(&best, 160)?;
    let mut found: Option<u64> = None;
    for i in 0..100usize {
        let oid = le64(&best, 184 + i * 8)?;
        if oid == 0 {
            continue;
        }
        let paddr = omap_lookup(file, size, omap, oid, xid)?;
        let vol = read_object(file, size, paddr)?;
        if &vol[32..36] != b"APSB" || le64(&vol, 8)? != oid || le64(&vol, 16)? > xid {
            return Err(EINVAL);
        }
        if le16(&vol, 964)? == XART_ROLE {
            if found
                .replace(xart_extent(
                    file,
                    size,
                    &vol,
                    le64(&vol, 16)?,
                    writes_enabled,
                )?)
                .is_some()
            {
                return Err(ENOTSUPP);
            }
        }
    }
    found.ok_or(ENODATA)
}
