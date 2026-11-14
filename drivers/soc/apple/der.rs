// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! The key-store configuration blob, which is DER.

use kernel::prelude::*;

const TAG_SET: u8 = 0x31;
const TAG_SEQUENCE: u8 = 0x30;
const TAG_UTF8: u8 = 0x0c;
pub(crate) const TAG_INTEGER: u8 = 0x02;
pub(crate) const TAG_OCTET_STRING: u8 = 0x04;

const MAX_ITEM: usize = 4096;

fn put_header(out: &mut KVec<u8>, tag: u8, len: usize) -> Result<()> {
    out.push(tag, GFP_KERNEL)?;
    if len < 0x80 {
        out.push(len as u8, GFP_KERNEL)?;
    } else if len <= 0xff {
        out.push(0x81, GFP_KERNEL)?;
        out.push(len as u8, GFP_KERNEL)?;
    } else if len <= 0xffff {
        out.push(0x82, GFP_KERNEL)?;
        out.push((len >> 8) as u8, GFP_KERNEL)?;
        out.push((len & 0xff) as u8, GFP_KERNEL)?;
    } else {
        return Err(EINVAL);
    }
    Ok(())
}

fn integer_bytes(v: u32) -> Result<KVec<u8>> {
    let mut raw = KVec::new();
    let be = v.to_be_bytes();
    let mut i = 0usize;
    while i < 3 && be[i] == 0 {
        i += 1;
    }
    if be[i] & 0x80 != 0 {
        raw.push(0, GFP_KERNEL)?;
    }
    raw.extend_from_slice(&be[i..], GFP_KERNEL)?;
    Ok(raw)
}

#[derive(Clone, Copy)]
pub(crate) enum RefKeyValue<'a> {
    /// op mnemonic: `oc`/`ow`/`ouw`.
    Utf8(&'a [u8]),
    Boolean(bool),
    Integer(u32),
    Octets(&'a [u8]),
    Der(&'a [u8]),
}

/// Apple key-store `SET OF SEQUENCE`, sorted by complete encoded member.
pub(crate) fn encode_refkey_set(items: &[(&[u8], RefKeyValue<'_>)]) -> Result<KVec<u8>> {
    let mut members: KVec<(&[u8], KVec<u8>)> = KVec::new();
    for (key, value) in items {
        if key.is_empty() || key.len() > MAX_ITEM {
            return Err(EINVAL);
        }
        let mut inner = KVec::new();
        put_header(&mut inner, TAG_UTF8, key.len())?;
        inner.extend_from_slice(key, GFP_KERNEL)?;
        match value {
            RefKeyValue::Boolean(v) => {
                // Apple's access-control serializer uses 0x01 for true.
                inner.extend_from_slice(&[0x01, 0x01, u8::from(*v)], GFP_KERNEL)?;
            }
            RefKeyValue::Utf8(s) => {
                if s.len() > MAX_ITEM {
                    return Err(EINVAL);
                }
                put_header(&mut inner, TAG_UTF8, s.len())?;
                inner.extend_from_slice(s, GFP_KERNEL)?;
            }
            RefKeyValue::Integer(v) => {
                let raw = integer_bytes(*v)?;
                put_header(&mut inner, TAG_INTEGER, raw.len())?;
                inner.extend_from_slice(&raw, GFP_KERNEL)?;
            }
            RefKeyValue::Octets(b) => {
                if b.len() > MAX_ITEM {
                    return Err(EINVAL);
                }
                put_header(&mut inner, TAG_OCTET_STRING, b.len())?;
                inner.extend_from_slice(b, GFP_KERNEL)?;
            }
            RefKeyValue::Der(d) => inner.extend_from_slice(d, GFP_KERNEL)?,
        }
        let mut seq = KVec::new();
        put_header(&mut seq, TAG_SEQUENCE, inner.len())?;
        seq.extend_from_slice(&inner, GFP_KERNEL)?;
        members.push((key, seq), GFP_KERNEL)?;
    }
    members.sort_unstable_by(|a, b| a.1.as_slice().cmp(b.1.as_slice()));
    let mut body = KVec::new();
    for (_, seq) in members.iter() {
        body.extend_from_slice(seq, GFP_KERNEL)?;
    }
    let mut out = KVec::new();
    put_header(&mut out, TAG_SET, body.len())?;
    out.extend_from_slice(&body, GFP_KERNEL)?;
    Ok(out)
}

fn take_tlv(buf: &[u8]) -> Option<(u8, &[u8], &[u8])> {
    let tag = *buf.first()?;
    let first = *buf.get(1)?;
    let (len, off) = if first < 0x80 {
        (first as usize, 2usize)
    } else if first == 0x81 {
        (*buf.get(2)? as usize, 3usize)
    } else if first == 0x82 {
        let hi = *buf.get(2)? as usize;
        let lo = *buf.get(3)? as usize;
        ((hi << 8) | lo, 4usize)
    } else {
        return None;
    };
    if len > MAX_ITEM {
        return None;
    }
    let end = off.checked_add(len)?;
    if buf.len() < end {
        return None;
    }
    Some((tag, &buf[off..end], &buf[end..]))
}

pub(crate) fn refkey_find<'a>(set_blob: &'a [u8], key: &[u8]) -> Option<&'a [u8]> {
    let mut body = match take_tlv(set_blob) {
        Some((TAG_SET, body, [])) => body,
        _ => return None,
    };
    while !body.is_empty() {
        let (seq_tag, seq, next) = take_tlv(body)?;
        if seq_tag != TAG_SEQUENCE {
            return None;
        }
        let (key_tag, this_key, after_key) = take_tlv(seq)?;
        if key_tag != TAG_UTF8 {
            return None;
        }
        if this_key == key {
            let (_val_tag, _value, after_val) = take_tlv(after_key)?;
            if !after_val.is_empty() {
                return None;
            }
            return Some(&after_key[..after_key.len() - after_val.len()]);
        }
        body = next;
    }
    None
}

pub(crate) fn octet_string_body(tlv: &[u8]) -> Option<&[u8]> {
    match take_tlv(tlv) {
        Some((TAG_OCTET_STRING, body, [])) => Some(body),
        _ => None,
    }
}
