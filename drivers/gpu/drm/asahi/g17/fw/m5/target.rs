// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 render-target suballocation geometry.


pub(crate) fn cluster_control(clusters: u32, pb_pages: u64) -> Option<[u64; 3]> {
    let difference = match clusters {
        1 => return Some([1, 0, pb_pages]),
        2 => 4u64,
        4 => 5u64,
        8 => 6u64,
        _ => return None,
    };
    Some([0x40 | (difference << 8) | (difference << 19), 4, pb_pages])
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TargetSlices {
    pub(crate) sizes: [usize; 8],
    pub(crate) alignments: [usize; 8],
    pub(crate) offsets: [usize; 8],
    pub(crate) bytes: usize,
}

impl TargetSlices {
    pub(crate) fn new(tilemap_bytes: usize, tpc_used_bytes: usize, layers: u16,
        kind: u32, segments: u32, pb_bytes: usize) -> Option<Self> {
        if tilemap_bytes == 0 || tpc_used_bytes == 0 || layers == 0 || kind == 0 {
            return None;
        }
        let mut sizes = [0; 8];
        let mut alignments = [0; 8];
        sizes[0] = tpc_used_bytes;
        alignments[0] = 0x40;
        if kind >= 2 {
            if segments == 0 || pb_bytes == 0 { return None; }
            let segments = segments as usize;
            sizes[1] = tilemap_bytes.checked_mul(segments)?;
            sizes[2] = if layers > 1 { 0x100usize.checked_mul(segments)? } else { 0 };
            sizes[4] = (kind as usize).checked_mul(0x300)?.checked_add(0x680)?;
            sizes[5] = (kind as usize).checked_mul(0x5c0)?;

            let pages = pb_bytes.checked_add(0x7fff)? / 0x8000;
            let groups = (pages.checked_add(0x17ff)? / 0x1800).checked_add(1)?;
            sizes[6] = groups.checked_mul(8)?.checked_mul(segments)?;
            sizes[7] = 0x80;
            alignments = [0x40, 0x40, 0x100, 0, 0x40, 0x40, 0x80, 0x40];
        }
        let mut bytes = 0usize;
        let mut offsets = [0; 8];
        for i in 0..8 {
            if alignments[i] != 0 {
                bytes = bytes.checked_add(alignments[i] - 1)? & !(alignments[i] - 1);
            }
            offsets[i] = bytes;
            bytes = bytes.checked_add(sizes[i])?;
        }
        Some(Self { sizes, alignments, offsets, bytes })
    }
}
