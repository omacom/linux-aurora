// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Parameter-buffer and UMA allocator records of the 2419 firmware.
//!
//! The PB is one 256 MiB heap of [`MAX_PAGES`] pages of 0x8000 bytes in [`MAX_BLOCKS`] blocks of
//! four pages. The firmware sees it through the page list (u32 records `G >> 15` of compact
//! addresses `G = gva - 0x1000000000`, one per page), the block list (8-byte records, the low
//! word `G >> 15` of the block's first page) and the PB manager P: P+0x28 = 4 * blocks pages,
//! P+0x2c K = 4096 (truthful: 8 * K bytes is the block-list span the firmware synchronizes on
//! PB reconstruction, also during recovery), P+0x30 = blocks, P+0x48 = 4 * blocks - 1,
//! P+0x70/+0x74 = 4 * maximum pages (the firmware asks to grow only while its current size is
//! below them). block_ctl +0 = blocks, +4 = the host producer of block-list records, modulo K,
//! advanced by every growth answer.
//!
//! Nothing here allocates, locks or publishes: the functions build the exact record bytes (a
//! zero fill followed by the field stores) and are host-tested.

/// Bytes of one PB page.
pub(crate) const PAGE_BYTES: u64 = 0x8000;
/// Pages per block, and bytes of one block.
pub(crate) const PAGES_PER_BLOCK: u32 = 4;
pub(crate) const BLOCK_BYTES: u64 = 0x20000;
/// Blocks and pages of the mapped 256 MiB heap.
pub(crate) const MAX_BLOCKS: u32 = 2048;
pub(crate) const MAX_PAGES: u32 = 8192;
/// Records of the 0x8000-byte block list: the truthful K.
pub(crate) const BLOCK_LIST_RECORDS: u32 = 4096;
/// Sizes of the page list and the block list.
pub(crate) const PAGE_LIST_SIZE: usize = 0x28000;
pub(crate) const BLOCK_LIST_SIZE: usize = 0x8000;
/// Bytes of one block-list record.
pub(crate) const BLOCK_RECORD_BYTES: usize = 8;
/// Blocks per growth request (the native growImmediately).
pub(crate) const GROW_STEP: u32 = 10;
/// Smallest initial PB with growth.
pub(crate) const MIN_BLOCKS: u32 = 64;
/// Base of the 32-bit compact pointer window.
pub(crate) const COMPACT_BASE: u64 = 0x1000000000;

/// Compact pointer of a GPU VA.
pub(crate) const fn compact(gva: u64) -> u64 {
    gva.wrapping_sub(COMPACT_BASE)
}

/// Page-list record of PB page `page` of the heap at `heap_gva`.
pub(crate) const fn page_record(heap_gva: u64, page: u32) -> u32 {
    (compact(heap_gva + page as u64 * PAGE_BYTES) / PAGE_BYTES) as u32
}

/// Block-list record of PB block `block`: `(G - 0x1000000000) >> 15` of its first page.
pub(crate) const fn block_record(heap_gva: u64, block: u32) -> u32 {
    (compact(heap_gva + block as u64 * BLOCK_BYTES) / PAGE_BYTES) as u32
}

/// Byte offset of the block-list record that block `block` goes to: `8 * (block % K)`.
pub(crate) const fn block_record_offset(block: u32) -> usize {
    BLOCK_RECORD_BYTES * (block % BLOCK_LIST_RECORDS) as usize
}

/// PB manager offsets (0xac bytes, firmware-only).
pub(crate) mod pb_manager {
    pub(crate) const SIZE: usize = 0xac;
    /// u64 page-list GVA, and its size in bytes.
    pub(crate) const PAGE_LIST: usize = 0x1c;
    pub(crate) const PAGE_LIST_BYTES: usize = 0x24;
    pub(crate) const PAGES: usize = 0x28;
    pub(crate) const K: usize = 0x2c;
    pub(crate) const K_VALUE: u32 = super::BLOCK_LIST_RECORDS;
    pub(crate) const BLOCKS: usize = 0x30;
    /// u64 block-list GVA.
    pub(crate) const BLOCK_LIST: usize = 0x38;
    /// u64 block_ctl firmware VA.
    pub(crate) const BLOCK_CTL: usize = 0x40;
    pub(crate) const LAST_PAGE: usize = 0x48;
    pub(crate) const BLOCK_BYTES: usize = 0x4c;
    /// u64 PB counter firmware VA.
    pub(crate) const COUNTER: usize = 0x58;
    pub(crate) const MAX_PAGES_A: usize = 0x70;
    pub(crate) const MAX_PAGES_B: usize = 0x74;
}

/// UMA allocator manager offsets (0x84 bytes; render managers firmware+GPU class).
pub(crate) mod uma_manager {
    pub(crate) const SIZE: usize = 0x84;
    pub(crate) const ID: usize = 0x00;
    pub(crate) const INDEX: usize = 0x08;
    pub(crate) const ONE: usize = 0x0c;
    /// u64 freelist GVA.
    pub(crate) const FREELIST: usize = 0x14;
    pub(crate) const UNK_1C: usize = 0x1c;
    pub(crate) const UNK_1C_VALUE: u32 = 0x80000;
    /// Freelist seeds available and total.
    pub(crate) const SEEDS_A: usize = 0x24;
    pub(crate) const SEEDS_B: usize = 0x2c;
    pub(crate) const SEEDS: u32 = 8192;
    /// u64 pool-table GVA.
    pub(crate) const POOL_TABLE: usize = 0x34;
    pub(crate) const UNK_44: usize = 0x44;
    pub(crate) const UNK_44_VALUE: u32 = 0x40;
    /// u64 counter firmware VA.
    pub(crate) const COUNTER: usize = 0x48;
}

/// UMA pools of one render engine: eight 4 MiB pools seeded 0x1000 at a time into the freelist
/// (8 * 1024 = [`uma_manager::SEEDS`] u64 records), pool-table rows of 0x40 bytes.
pub(crate) mod uma_pools {
    pub(crate) const POOLS: usize = 8;
    pub(crate) const POOL_SIZE: u64 = 0x400000;
    pub(crate) const SEED_BYTES: u64 = 0x1000;
    pub(crate) const FREELIST_SIZE: usize = 0x400000;
    pub(crate) const POOL_TABLE_SIZE: usize = 0x1000;
    pub(crate) const POOL_TABLE_OFFSET: usize = 0x3000;
    pub(crate) const POOL_TABLE_STRIDE: usize = 0x40;
    /// Pool-table row: the pool's GVA with bit 62.
    pub(crate) const fn row(pool_gva: u64) -> u64 {
        pool_gva | (1 << 62)
    }
}

/// Block control (0x40 bytes): +0 blocks, +4 the live producer of block-list records.
pub(crate) mod block_ctl {
    pub(crate) const SIZE: usize = 0x40;
    pub(crate) const BLOCKS: usize = 0;
    pub(crate) const PRODUCER: usize = 4;
}

/// Misc record (0x40 bytes): +0x20 = 1.
pub(crate) mod misc {
    pub(crate) const SIZE: usize = 0x40;
    pub(crate) const ENABLE: usize = 0x20;
}

/// UMA and PB counters (0x40 bytes): +0 = 1 at initialization.
pub(crate) mod counter {
    pub(crate) const SIZE: usize = 0x40;
}

/// Scene list (0xc0 bytes), zeroed at initialization.
pub(crate) const SCENE_LIST_SIZE: usize = 0xc0;

/// PB growth reply (DEVCTL 7) words.
pub(crate) mod grow_reply {
    pub(crate) const OPCODE: usize = 0;
    /// 1: grown, 0: refused (the firmware then renders partially).
    pub(crate) const GREW: usize = 1;
    pub(crate) const SLOT: usize = 2;
    pub(crate) const CONTEXT: usize = 3;
    pub(crate) const COUNTER: usize = 4;
    pub(crate) const POSITION: usize = 5;
    /// u64 epoch (the Control halt count) at words 6..7.
    pub(crate) const EPOCH: usize = 6;
}

/// PB growth request (event type 7) words, copied whole by the event drain.
pub(crate) mod grow_request {
    pub(crate) const TYPE: usize = 0;
    pub(crate) const CONTEXT: usize = 1;
    pub(crate) const SLOT: usize = 2;
    /// The firmware's request counter (P+0x60 before its increment).
    pub(crate) const COUNTER: usize = 3;
    /// The TA position.
    pub(crate) const POSITION: usize = 4;
    /// u64 halt epoch at words 5..6.
    pub(crate) const EPOCH: usize = 5;
}

/// The C1 record written once per (re)initialization before the first render prefix that
/// follows it: {0x07400000, 4 * initial blocks}.
pub(crate) const fn c1_record(initial_blocks: u32) -> [u8; 16] {
    let a = 0x07400000u32.to_le_bytes();
    let b = (PAGES_PER_BLOCK * initial_blocks).to_le_bytes();
    [
        a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], 0, 0, 0, 0, 0, 0, 0, 0,
    ]
}

fn put32(image: &mut [u8], offset: usize, value: u32) {
    image[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put64(image: &mut [u8], offset: usize, value: u64) {
    image[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// The PB manager image.
pub(crate) fn pb_manager_image(
    initial: u32,
    max: u32,
    page_list_gva: u64,
    block_list_gva: u64,
    block_ctl_va: u64,
    counter_va: u64,
) -> [u8; pb_manager::SIZE] {
    let mut p = [0u8; pb_manager::SIZE];
    put64(&mut p, pb_manager::PAGE_LIST, page_list_gva);
    put32(&mut p, pb_manager::PAGE_LIST_BYTES, PAGE_LIST_SIZE as u32);
    put32(&mut p, pb_manager::PAGES, PAGES_PER_BLOCK * initial);
    put32(&mut p, pb_manager::K, pb_manager::K_VALUE);
    put32(&mut p, pb_manager::BLOCKS, initial);
    put64(&mut p, pb_manager::BLOCK_LIST, block_list_gva);
    put64(&mut p, pb_manager::BLOCK_CTL, block_ctl_va);
    put32(&mut p, pb_manager::LAST_PAGE, PAGES_PER_BLOCK * initial - 1);
    put32(&mut p, pb_manager::BLOCK_BYTES, BLOCK_BYTES as u32);
    put64(&mut p, pb_manager::COUNTER, counter_va);
    put32(&mut p, pb_manager::MAX_PAGES_A, PAGES_PER_BLOCK * max);
    put32(&mut p, pb_manager::MAX_PAGES_B, PAGES_PER_BLOCK * max);
    p
}

/// A UMA manager image. The render managers have `id` = engine + 1, `index` = engine; the
/// compute graph's manager is id 3, index 2.
pub(crate) fn uma_manager_image(
    id: u32,
    index: u32,
    freelist_gva: u64,
    pool_table_gva: u64,
    counter_va: u64,
) -> [u8; uma_manager::SIZE] {
    let mut m = [0u8; uma_manager::SIZE];
    put32(&mut m, uma_manager::ID, id);
    put32(&mut m, uma_manager::INDEX, index);
    put32(&mut m, uma_manager::ONE, 1);
    put64(&mut m, uma_manager::FREELIST, freelist_gva);
    put32(&mut m, uma_manager::UNK_1C, uma_manager::UNK_1C_VALUE);
    put32(&mut m, uma_manager::SEEDS_A, uma_manager::SEEDS);
    put32(&mut m, uma_manager::SEEDS_B, uma_manager::SEEDS);
    put64(&mut m, uma_manager::POOL_TABLE, pool_table_gva);
    put32(&mut m, uma_manager::UNK_44, uma_manager::UNK_44_VALUE);
    put64(&mut m, uma_manager::COUNTER, counter_va);
    m
}

/// The block-control image.
pub(crate) fn block_ctl_image(initial: u32) -> [u8; block_ctl::SIZE] {
    let mut b = [0u8; block_ctl::SIZE];
    put32(&mut b, block_ctl::BLOCKS, initial);
    put32(&mut b, block_ctl::PRODUCER, initial);
    b
}

/// The DEVCTL 7 reply to growth request `request`.
pub(crate) fn grow_reply_words(request: &[u32; 14], grew: bool) -> [u32; 14] {
    let mut w = [0u32; 14];
    w[grow_reply::OPCODE] = super::channels::devctl::GROW_REPLY;
    w[grow_reply::GREW] = u32::from(grew);
    w[grow_reply::SLOT] = request[grow_request::SLOT];
    w[grow_reply::CONTEXT] = request[grow_request::CONTEXT];
    w[grow_reply::COUNTER] = request[grow_request::COUNTER];
    w[grow_reply::POSITION] = request[grow_request::POSITION];
    w[grow_reply::EPOCH] = request[grow_request::EPOCH];
    w[grow_reply::EPOCH + 1] = request[grow_request::EPOCH + 1];
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get32(b: &[u8], o: usize) -> u32 {
        u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
    }

    fn get64(b: &[u8], o: usize) -> u64 {
        u64::from_le_bytes(b[o..o + 8].try_into().unwrap())
    }

    const HEAP: u64 = 0x70c0000000;

    #[test]
    fn records() {
        // (0x70c0000000 - 0x1000000000) >> 15 = 0xc18000.
        assert_eq!(page_record(HEAP, 0), 0xc18000);
        assert_eq!(page_record(HEAP, 1), 0xc18001);
        assert_eq!(block_record(HEAP, 0), 0xc18000);
        assert_eq!(block_record(HEAP, 1), 0xc18004);
        assert_eq!(block_record(HEAP, 2047), 0xc18000 + 4 * 2047);
        // The last page of the 256 MiB heap.
        assert_eq!(page_record(HEAP, MAX_PAGES - 1), 0xc18000 + 8191);
        assert_eq!(block_record_offset(0), 0);
        assert_eq!(block_record_offset(1023), 8 * 1023);
        assert_eq!(block_record_offset(4096), 0);
        assert_eq!(block_record_offset(4097), 8);
        // The block list holds K records exactly.
        assert_eq!(
            BLOCK_LIST_RECORDS as usize * BLOCK_RECORD_BYTES,
            BLOCK_LIST_SIZE
        );
        // The page list holds every page of the heap.
        assert!(MAX_PAGES as usize * 4 <= PAGE_LIST_SIZE);
        assert_eq!(MAX_BLOCKS as u64 * BLOCK_BYTES, 0x10000000);
    }

    #[test]
    fn pb_manager() {
        let p = pb_manager_image(
            1024,
            2048,
            0x1074000000,
            0x70e0000000,
            0xfffffc2d00010000,
            0xfffffc2d00020000,
        );
        assert_eq!(get64(&p, 0x1c), 0x1074000000);
        assert_eq!(get32(&p, 0x24), 0x28000);
        assert_eq!(get32(&p, 0x28), 4096);
        assert_eq!(get32(&p, 0x2c), 4096);
        assert_eq!(get32(&p, 0x30), 1024);
        assert_eq!(get64(&p, 0x38), 0x70e0000000);
        assert_eq!(get64(&p, 0x40), 0xfffffc2d00010000);
        assert_eq!(get32(&p, 0x48), 4095);
        assert_eq!(get32(&p, 0x4c), 0x20000);
        assert_eq!(get64(&p, 0x58), 0xfffffc2d00020000);
        assert_eq!(get32(&p, 0x70), 8192);
        assert_eq!(get32(&p, 0x74), 8192);
        // Every other byte is zero (the zero fill).
        let written = [
            (0x1c, 8),
            (0x24, 4),
            (0x28, 4),
            (0x2c, 4),
            (0x30, 4),
            (0x38, 8),
            (0x40, 8),
            (0x48, 4),
            (0x4c, 4),
            (0x58, 8),
            (0x70, 4),
            (0x74, 4),
        ];
        for (i, byte) in p.iter().enumerate() {
            if !written.iter().any(|&(o, n)| i >= o && i < o + n) {
                assert_eq!(*byte, 0, "byte {i:#x}");
            }
        }
    }

    #[test]
    fn uma_manager() {
        let m = uma_manager_image(2, 1, 0x7014490000, 0x701489b000, 0xfffffc2d00030000);
        assert_eq!(get32(&m, 0), 2);
        assert_eq!(get32(&m, 8), 1);
        assert_eq!(get32(&m, 0xc), 1);
        assert_eq!(get64(&m, 0x14), 0x7014490000);
        assert_eq!(get32(&m, 0x1c), 0x80000);
        assert_eq!(get32(&m, 0x24), 8192);
        assert_eq!(get32(&m, 0x2c), 8192);
        assert_eq!(get64(&m, 0x34), 0x701489b000);
        assert_eq!(get32(&m, 0x44), 0x40);
        assert_eq!(get64(&m, 0x48), 0xfffffc2d00030000);
        assert_eq!(&m[0x50..], &[0u8; 0x84 - 0x50][..]);
        // Freelist seeds = 8 pools of 4 MiB in 0x1000 units.
        assert_eq!(
            uma_pools::POOLS as u64 * uma_pools::POOL_SIZE / uma_pools::SEED_BYTES,
            uma_manager::SEEDS as u64
        );
        // The seeded records fit the freelist.
        assert!(uma_manager::SEEDS as usize * 8 <= uma_pools::FREELIST_SIZE);
    }

    #[test]
    fn small_records() {
        let b = block_ctl_image(1024);
        assert_eq!(get32(&b, 0), 1024);
        assert_eq!(get32(&b, 4), 1024);
        assert!(b[8..].iter().all(|&x| x == 0));
        assert_eq!(uma_pools::row(0x7010000000), 0x4000007010000000);
        let c1 = c1_record(1024);
        assert_eq!(get32(&c1, 0), 0x07400000);
        assert_eq!(get32(&c1, 4), 4096);
        assert!(c1[8..].iter().all(|&x| x == 0));
    }

    #[test]
    fn reply() {
        let mut q = [0u32; 14];
        q[0] = 7;
        q[1] = 5; // context
        q[2] = 0; // slot
        q[3] = 12; // counter
        q[4] = 0x345; // position
        q[5] = 3; // epoch low
        q[6] = 1; // epoch high
        let w = grow_reply_words(&q, true);
        assert_eq!(&w[..8], &[7, 1, 0, 5, 12, 0x345, 3, 1]);
        assert!(w[8..].iter().all(|&x| x == 0));
        let w = grow_reply_words(&q, false);
        assert_eq!(w[1], 0);
    }
}
