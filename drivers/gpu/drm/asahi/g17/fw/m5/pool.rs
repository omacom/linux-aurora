// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 private-memory allocation encodings.
#![allow(dead_code)]

use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

use super::initdata::{Board, J714};

pub(crate) const BLOCK_BYTES: usize = 0x20_0000;
// Render block size and private-memory address inventory of the M5
// part, selected at probe (`Board::render_pool`). The inventory sizes the
// directory, not the physical allocation; each pool allocates only its
// requested initial blocks.
static RENDER_BLOCK_BYTES: AtomicUsize = AtomicUsize::new(J714.render_pool.0);
static POOL_BYTES: AtomicU64 = AtomicU64::new(J714.render_pool.1);

/// Selects the board's render pool before any pool is built.
pub(crate) fn select_render_pool(board: &Board) {
    let (block, pool) = board.render_pool;
    RENDER_BLOCK_BYTES.store(block, Ordering::Relaxed);
    POOL_BYTES.store(pool, Ordering::Relaxed);
}

pub(crate) fn render_block_bytes() -> usize {
    RENDER_BLOCK_BYTES.load(Ordering::Relaxed)
}

const RING_MAX_PAGES: u64 = 0x3ff800;
pub(crate) const OPERAND_BYTES: usize = 0x2_0000;
pub(crate) const STATE_BYTES: usize = 0x100;
/// Unaligned firmware-completed submission counter.
pub(crate) const COMPLETED: usize = 0x74;
/// Mutable words saved from the descriptor after the pool's final release.
pub(crate) const SAVED_FIELDS: [usize; 8] = [0x28, 0x20, 0x24, 0x2c, 0x3c, 0x48, 0x38, 0x4c];
const GPU_PAGE: usize = 0x4000;
const PRIVATE_TOP: u64 = 1 << 42;
pub(crate) const SCENE_TABLE_ENTRIES: usize = 0x50;
pub(crate) const SCENE_STRIDE: usize = 0x80;
pub(crate) const SCENE_TRAILER_SIZE: usize = 0x40;

pub(crate) fn scene_table_end(address: u64) -> Option<u64> {
    address.checked_add((SCENE_TABLE_ENTRIES * SCENE_STRIDE) as u64)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PoolError { Geometry, Address, Buffer }

/// Snapshot of the retained pool header. Address and allocation geometry are
/// host-owned; only SAVED_FIELDS may be restored from a retired descriptor.
pub(crate) struct PoolState(pub(crate) [u8; STATE_BYTES]);

impl PoolState {
    fn u32(&self, offset: usize) -> u64 {
        u64::from(u32::from_le_bytes(self.0[offset..offset + 4].try_into().unwrap()))
    }
    fn u64(&self, offset: usize) -> u64 {
        u64::from_le_bytes(self.0[offset..offset + 8].try_into().unwrap())
    }
    pub(crate) fn descriptor(&self) -> [u64; 8] {
        [self.u64(0x14) >> 7 | self.u32(0x28) << 61,
         self.u64(0x20), self.u32(0x2c) | self.u32(0x1c) << 32, 0,
         self.u64(0x30) >> 6 | self.u32(0x40) << 44,
         self.u32(0x3c) | self.u32(0x44) << 31,
         self.u32(0x48), self.u32(0x38) | self.u32(0x4c) << 32]
    }
    pub(crate) fn restore(&mut self, words: [u64; 8]) {
        let values = [words[0] >> 61 & 1, words[1] & 0x0fff_ffff,
            words[1] >> 32 & 0x0fff_ffff, words[2] & 0x0fff_ffff,
            words[5] & 0x0fff_ffff, words[6] & 0xffff,
            words[7] & 0x0fff_ffff, words[7] >> 32 & 0xffff];
        for (offset, value) in SAVED_FIELDS.into_iter().zip(values) {
            self.0[offset..offset + 4].copy_from_slice(&(value as u32).to_le_bytes());
        }
    }
    /// Reject a descriptor belonging to different backing before restoring
    /// its mutable counters. Reserved bits are not host address bits.
    pub(crate) fn matches(&self, words: &[u64; 8]) -> bool {
        let own = self.descriptor();
        words[0] & ((1 << 35) - 1) == own[0] & ((1 << 35) - 1)
            && words[2] >> 32 == own[2] >> 32
            && words[4] == own[4]
    }
}

/// Parameter-buffer capacity follows the memory budget and initial demand.
/// Capacity is rounded to groups of 33 half-MiB blocks; it reserves list space,
/// not physical backing for every possible block.
pub(crate) fn parameter_capacity(memory_bytes: u64, initial_bytes: u64) -> Result<usize, PoolError> {
    const GROUP: u64 = 33 * 0x80000;
    let scaled = memory_bytes.checked_mul(5).ok_or(PoolError::Geometry)? >> 2;
    let budget = (((u128::from(scaled) * 0x28f5_c28f_5c28_f5c3) >> 64) as u64) >> 2;
    let groups = |bytes: u64| bytes.checked_add(GROUP - 1)
        .map(|n| n / GROUP).ok_or(PoolError::Geometry);
    let groups = groups(budget)?.max(groups(initial_bytes)?);
    usize::try_from(groups.checked_mul(33).ok_or(PoolError::Geometry)?)
        .map_err(|_| PoolError::Geometry)
}

/// Per-scene scratch covers the complete parameter-buffer inventory. Optional
/// cluster entries are supplied by the dispatch policy, not the active draw's
/// physical block count. Each entry and the header occupy eight bytes.
pub(crate) fn parameter_scratch_stride(capacity: usize, cluster_entries: usize)
    -> Result<usize, PoolError>
{
    if capacity == 0 { return Err(PoolError::Geometry); }
    let pages = capacity.checked_mul(16).ok_or(PoolError::Geometry)?;
    let entries = pages.checked_add(0x17ff).ok_or(PoolError::Geometry)? / 0x1800;
    let entries = entries.checked_add(cluster_entries).ok_or(PoolError::Geometry)?;
    entries.checked_mul(8).and_then(|n| n.checked_add(8 + 63))
        .map(|n| n & !63).ok_or(PoolError::Geometry)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PoolGeometry {
    pub(crate) blocks: usize,
    pub(crate) directory_entries: u32,
    pub(crate) pages: u32,
    pub(crate) directory_bytes: usize,
    pub(crate) freelist_bytes: usize,
    g17s_render_blocks: bool,
}

fn round_page(bytes: usize) -> Result<usize, PoolError> {
    bytes.checked_add(GPU_PAGE - 1).map(|n| n & !(GPU_PAGE - 1)).ok_or(PoolError::Geometry)
}

impl PoolGeometry {
    pub(crate) fn g17s(blocks: usize) -> Result<Self, PoolError> {
        let bytes = POOL_BYTES.load(Ordering::Relaxed).min(RING_MAX_PAGES << 12)
            & !(render_block_bytes() as u64 - 1);
        Self::g17s_render(blocks, (bytes >> 12) as u32)
    }
    pub(crate) fn new(blocks: usize, directory_entries: u32) -> Result<Self, PoolError> {
        Self::with_kind(blocks, directory_entries, false)
    }

    pub(crate) fn g17s_render(blocks: usize, directory_entries: u32) -> Result<Self, PoolError> {
        Self::with_kind(blocks, directory_entries, true)
    }

    pub(crate) fn block_bytes(self) -> usize {
        if self.g17s_render_blocks { render_block_bytes() } else { BLOCK_BYTES }
    }

    fn with_kind(blocks: usize, directory_entries: u32, g17s_render_blocks: bool) -> Result<Self, PoolError> {
        let block_bytes = if g17s_render_blocks { render_block_bytes() } else { BLOCK_BYTES };
        let pages = blocks.checked_mul(block_bytes / 0x1000)
            .and_then(|n| u32::try_from(n).ok()).ok_or(PoolError::Geometry)?;
        if blocks == 0 || pages > directory_entries { return Err(PoolError::Geometry); }
        let directory_bytes = round_page((directory_entries as usize).checked_mul(8)
            .ok_or(PoolError::Geometry)?)?;
        let stride = if g17s_render_blocks { 8 } else { 0x40 };
        let freelist_bytes = round_page(blocks.checked_mul(stride).ok_or(PoolError::Geometry)?)?;

        if freelist_bytes >> 12 >= 1 << 20 { return Err(PoolError::Geometry); }
        Ok(Self { blocks, directory_entries, pages, directory_bytes, freelist_bytes, g17s_render_blocks })
    }
}

fn private_range(address: u64, bytes: usize) -> Result<(), PoolError> {
    if address == 0 || address & (GPU_PAGE as u64 - 1) != 0
        || address.checked_add(bytes as u64).is_none_or(|end| end > PRIVATE_TOP) {
        Err(PoolError::Address)
    } else { Ok(()) }
}

fn validate_blocks(g: PoolGeometry, blocks: &[u64]) -> Result<(), PoolError> {
    if PoolGeometry::with_kind(g.blocks, g.directory_entries, g.g17s_render_blocks)? != g { return Err(PoolError::Geometry); }
    if blocks.len() != g.blocks { return Err(PoolError::Geometry); }
    for address in blocks { private_range(*address, g.block_bytes())?; }
    Ok(())
}

pub(crate) fn encode_directory(g: PoolGeometry, blocks: &[u64], out: &mut [u8])
    -> Result<(), PoolError>
{
    validate_blocks(g, blocks)?;
    if out.len() < g.directory_bytes { return Err(PoolError::Buffer); }
    out[..g.directory_bytes].fill(0);

    let pages_per_block = g.block_bytes() / 0x1000;
    for (block, address) in blocks.iter().enumerate() {
        for page in 0..pages_per_block {
            let offset = (block * pages_per_block + page) * 8;
            out[offset..offset + 8].copy_from_slice(&(address + page as u64 * 0x1000).to_le_bytes());
        }
    }
    Ok(())
}

pub(crate) fn encode_freelist(g: PoolGeometry, blocks: &[u64], out: &mut [u8])
    -> Result<(), PoolError>
{
    validate_blocks(g, blocks)?;
    if out.len() < g.freelist_bytes { return Err(PoolError::Buffer); }
    out[..g.freelist_bytes].fill(0);
    for (index, address) in blocks.iter().enumerate() {
        let (offset, value) = if g.g17s_render_blocks { (index * 8, *address) }
            else { (index * 0x40, address | (1 << 61)) };
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PoolLayout {
    pub(crate) geometry: PoolGeometry,
    pub(crate) directory: u64,
    pub(crate) freelist: u64,

    pub(crate) hardware_id: u32,
}

fn validate_layout(p: PoolLayout) -> Result<(), PoolError> {
    if PoolGeometry::with_kind(p.geometry.blocks, p.geometry.directory_entries, p.geometry.g17s_render_blocks)? != p.geometry {
        return Err(PoolError::Geometry);
    }
    private_range(p.directory, p.geometry.directory_bytes)?;
    private_range(p.freelist, p.geometry.freelist_bytes)
}

pub(crate) fn initial_uma_descriptor(p: PoolLayout) -> Result<[u64; 8], PoolError> {
    validate_layout(p)?;
    if p.geometry.g17s_render_blocks {
        let blocks = p.geometry.blocks as u64;
        return Ok([p.directory >> 7, u64::from(p.geometry.pages) << 32,
            u64::from(p.geometry.pages) | (u64::from(p.geometry.directory_entries) << 32),
            0, (p.freelist >> 6) | ((p.geometry.block_bytes() as u64 >> 12) << 44),
            blocks, 0, blocks]);
    }
    Ok([p.directory >> 7, u64::from(p.geometry.pages) << 32,
        u64::from(p.geometry.pages) | (u64::from(p.geometry.directory_entries) << 32),
        0, (p.freelist >> 6) | ((p.geometry.freelist_bytes as u64 >> 12) << 44),
        0, 0, 0])
}

pub(crate) fn encode_state(p: PoolLayout, submitted: u64, out: &mut [u8])
    -> Result<(), PoolError>
{
    validate_layout(p)?;
    if submitted == 0 || submitted & 3 != 0 { return Err(PoolError::Address); }
    if out.len() < STATE_BYTES { return Err(PoolError::Buffer); }
    out[..STATE_BYTES].fill(0);
    for (offset, value) in [(0, 1u32), (8, p.hardware_id), (0x10, 2),
        (0x1c, p.geometry.directory_entries), (0x24, p.geometry.pages),
        (0x2c, p.geometry.pages), (0x40, (p.geometry.freelist_bytes >> 12) as u32),
        (0x60, 2), (0x80, 1)] {
        out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    for (offset, value) in [(0x14, p.directory), (0x30, p.freelist),
        (0x50, p.freelist), (0x6c, submitted)] {
        out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
    if p.geometry.g17s_render_blocks {

        for (offset, value) in [(0x0c, 2), (0x38, p.geometry.blocks as u32),
            (0x3c, p.geometry.blocks as u32),
            (0x40, (p.geometry.block_bytes() >> 12) as u32), (0x44, 0),
            (0x60, 0),
            (0x68, p.geometry.blocks as u32), (0x80, 4)] {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    Ok(())
}

pub(crate) fn encode_g17s_render_state(p: PoolLayout, submitted: u64, out: &mut [u8])
    -> Result<(), PoolError>
{
    encode_state(p, submitted, out)?;
    out[0x80..0x84].copy_from_slice(&4u32.to_le_bytes());
    Ok(())
}

pub(crate) fn encode_g17s_compute_state(p: PoolLayout, submitted: u64, out: &mut [u8])
    -> Result<(), PoolError>
{
    if !p.geometry.g17s_render_blocks { return Err(PoolError::Geometry); }
    encode_state(p, submitted, out)?;
    out[0x7c..0x80].copy_from_slice(&1u32.to_le_bytes());
    out[0x80..0x84].copy_from_slice(&1u32.to_le_bytes());
    Ok(())
}
