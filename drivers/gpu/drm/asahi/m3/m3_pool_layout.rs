// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::m3_queue_layout::{Error, FirmwareVa};
pub(crate) const MANAGER_SIZE: usize = 0x84;
pub(crate) const BACKING_SIZE: usize = 4 * 1024 * 1024;
pub(crate) const PAGE_SIZE: usize = 0x1000;
pub(crate) const PAGE_CAPACITY: usize = 0x80000;
pub(crate) const INITIAL_PAGES: usize = 8 * BACKING_SIZE / PAGE_SIZE;
pub(crate) const POOL_TABLE_SIZE: usize = 0x1000;
pub(crate) const POOL_ENTRY_SIZE: usize = 0x40;
const POOL_TAG: u64 = 1 << 62; // Qualified ownership tag; precise meaning unresolved.
/// An owned page-aligned region in G15's lower 42-bit UAT root. A raw firmware
/// pointer, tagged table entry or compact scene pointer cannot pass this check.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GpuRegion { base: u64, size: usize }
impl GpuRegion {
    pub(crate) fn base(self) -> u64 { self.base }
    pub(crate) fn size(self) -> usize { self.size }
    pub(crate) fn new(base: u64, size: usize) -> Result<Self, Error> {
        let end = base.checked_add(size as u64).ok_or(Error::Address)?;
        if base == 0 || base & (PAGE_SIZE as u64 - 1) != 0 || size == 0 || end > 1 << 42 {
            return Err(Error::Address);
        }
        Ok(Self { base, size })
    }
}
pub(crate) struct Pool<'a> {
    backings: &'a [GpuRegion], pages: GpuRegion, table: GpuRegion,
    counter: FirmwareVa, table_index: u8,
}
impl<'a> Pool<'a> {
    pub(crate) fn new(backings: &'a [GpuRegion], pages: GpuRegion, table: GpuRegion,
        counter: FirmwareVa, table_index: u8) -> Result<Self, Error> {
        if !(8..=22).contains(&backings.len()) || backings.iter().any(|b| b.size != BACKING_SIZE)
            || pages.size < PAGE_CAPACITY * 8 || table.size < POOL_TABLE_SIZE {
            return Err(Error::Size);
        }
        Ok(Self { backings, pages, table, counter, table_index })
    }
    /// Address of an initially available 4-KiB page; the remaining freelist is
    /// zero from allocation. Firmware subsequently advances its own cursors.
    pub(crate) fn page(&self, index: usize) -> Result<u64, Error> {
        if index >= INITIAL_PAGES { return Err(Error::Index); }
        Ok(self.backings[index / (BACKING_SIZE / PAGE_SIZE)].base
            + (index % (BACKING_SIZE / PAGE_SIZE) * PAGE_SIZE) as u64)
    }
    pub(crate) fn table_entry(&self, index: usize) -> Result<[u8; POOL_ENTRY_SIZE], Error> {
        let backing = self.backings.get(index).ok_or(Error::Index)?;
        let mut bytes = [0; POOL_ENTRY_SIZE];
        bytes[..8].copy_from_slice(&(backing.base | POOL_TAG).to_le_bytes());
        Ok(bytes)
    }
    /// Exact packed layout, including unaligned pointer fields. All validation
    /// precedes mutation and caller bytes beyond MANAGER_SIZE are untouched.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let bytes = out.get_mut(..MANAGER_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for (offset, value) in [(0x00, u32::from(self.table_index) + 1), // software pool ID
            (0x08, u32::from(self.table_index)), // HW table index, FW34584
            (0x0c, 1), // retained unk_c
            (0x1c, PAGE_CAPACITY as u32),
            (0x24, INITIAL_PAGES as u32), // initial producer cursor
            (0x2c, INITIAL_PAGES as u32), // retained available-page count
            (0x44, 0x40)] { // retained unk_44; not the number of allocations
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [(0x14, self.pages.base), (0x34, self.table.base), (0x48, self.counter.get())] {
            bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        Ok(())
    }
}

/// Extended firmware's private-memory pool descriptor. The owners and initial
/// population are supplied separately from the descriptor's firmware state.
pub(crate) struct ModernPool {
    pub(crate) identity: FirmwareVa,
    pub(crate) slot: u8,
    pub(crate) entries: GpuRegion,
    pub(crate) groups: GpuRegion,
    pub(crate) submitted: FirmwareVa,
    pub(crate) capacity: u32,
    pub(crate) populated: u32,
    pub(crate) compute_only: bool,
}

impl ModernPool {
    pub(crate) const SIZE: usize = 0xa0;

    pub(crate) fn encode(&self, output: &mut [u8]) -> Result<(), Error> {
        let entry_bytes = (self.capacity as usize).checked_mul(8).ok_or(Error::Size)?;
        let group_bytes = (self.populated as usize / 512).checked_mul(8).ok_or(Error::Size)?;
        if !(1..=64).contains(&self.slot) || self.capacity == 0 || self.populated >= self.capacity || self.populated % 512 != 0
            || self.entries.base() & 0x3fff != 0 || self.groups.base() & 0x3fff != 0
            || self.entries.size() < entry_bytes || self.groups.size() < group_bytes
            || self.submitted.get() & 7 != 0 || self.identity.get() & 7 != 0 { return Err(Error::Address); }
        let group_count: u32 = group_bytes.try_into().map_err(|_| Error::Size)?;
        let bytes = output.get_mut(..Self::SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        let word = |bytes: &mut [u8], offset: usize, value: u32| {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        };
        bytes[..8].copy_from_slice(&self.identity.get().to_le_bytes());
        word(bytes, 8, self.slot.into());
        word(bytes, 12, (self.populated == 0).into());
        word(bytes, 16, 2);
        bytes[0x14..0x1c].copy_from_slice(&self.entries.base().to_le_bytes());
        word(bytes, 0x1c, self.capacity);
        word(bytes, 0x24, self.populated);
        word(bytes, 0x28, (self.populated == 0).into());
        word(bytes, 0x2c, self.populated);
        bytes[0x30..0x38].copy_from_slice(&self.groups.base().to_le_bytes());
        word(bytes, 0x40, 4);
        word(bytes, 0x48, group_count);
        bytes[0x4c..0x54].copy_from_slice(&self.submitted.get().to_le_bytes());
        word(bytes, 0x5c, (self.compute_only && self.populated != 0).into());
        word(bytes, 0x84, (self.populated != 0).into());
        word(bytes, 0x94, 2);
        Ok(())
    }
}

#[cfg(test)]
mod modern_tests {
    use super::*;
    fn pool(compute_only: bool) -> ModernPool {
        ModernPool { identity: FirmwareVa::new(0xffff_fc21_9100_0000).unwrap(), slot: 1,
            entries: GpuRegion::new(0x5000_0000, 0x20000).unwrap(),
            groups: GpuRegion::new(0x6000_0000, 0x4000).unwrap(),
            submitted: FirmwareVa::new(0xffff_fc21_9000_0000).unwrap(),
            capacity: 0x4000, populated: 512, compute_only }
    }
    fn word(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }
    #[test] fn shared_render_pool_never_gets_compute_exclusion() {
        for compute in [false, true] {
            let mut output = [0xa5; ModernPool::SIZE + 8];
            pool(compute).encode(&mut output).unwrap();
            assert_eq!(word(&output, 0x5c), compute.into());
            assert_eq!(word(&output, 0x48), 8);
            assert_eq!(word(&output, 0x84), 1);
            assert_eq!(&output[ModernPool::SIZE..], &[0xa5; 8]);
        }
    }
    #[test] fn bad_population_and_insufficient_owners_reject_before_mutation() {
        for pages in [1, 511, 513, 0x4000] {
            let mut value = pool(true); value.populated = pages;
            let mut output = [0xa5; ModernPool::SIZE];
            assert!(value.encode(&mut output).is_err());
            assert_eq!(output, [0xa5; ModernPool::SIZE]);
        }
        let mut value = pool(true); value.capacity = 0x4001;
        let mut output = [0xa5; ModernPool::SIZE];
        assert!(value.encode(&mut output).is_err());
        assert_eq!(output, [0xa5; ModernPool::SIZE]);
    }
}

/// Populate the extended freelist from independently retained 4-MiB owners.
/// Hardware group records describe 2-MiB subregions, not entire owners.
pub(crate) struct ModernBacking<'a>(pub(crate) &'a [GpuRegion]);
impl ModernBacking<'_> {
    pub(crate) fn pages(&self) -> Result<usize, Error> {
        if self.0.is_empty() || self.0.len() > 8
            || self.0.iter().any(|owner| owner.size() != BACKING_SIZE || owner.base() & 0x3fff != 0) {
            return Err(Error::Size);
        }
        Ok(self.0.len() * BACKING_SIZE / PAGE_SIZE)
    }
    pub(crate) fn page(&self, index: usize) -> Result<u64, Error> {
        if index >= self.pages()? { return Err(Error::Index); }
        let per_owner = BACKING_SIZE / PAGE_SIZE;
        Ok(self.0[index / per_owner].base() + (index % per_owner * PAGE_SIZE) as u64)
    }
    pub(crate) fn group(&self, index: usize) -> Result<[u8; 64], Error> {
        if index >= self.pages()? / 512 { return Err(Error::Index); }
        let base = self.page(index * 512)?;
        let mut record = [0; 64];
        record[..8].copy_from_slice(&(base | (1 << 61)).to_le_bytes());
        Ok(record)
    }
}

#[cfg(test)]
mod backing_tests {
    use super::*;
    #[test] fn group_records_never_cross_a_noncontiguous_backing_owner() {
        let owners = [GpuRegion::new(0x5000_0000, BACKING_SIZE).unwrap(),
            GpuRegion::new(0x6000_0000, BACKING_SIZE).unwrap()];
        let backing = ModernBacking(&owners);
        assert_eq!(backing.pages(), Ok(2048));
        assert_eq!(backing.page(1024), Ok(0x6000_0000));
        let second = backing.group(2).unwrap();
        assert_eq!(u64::from_le_bytes(second[..8].try_into().unwrap()), (1 << 61) | 0x6000_0000);
        assert_eq!(&second[8..], &[0; 56]);
        assert!(backing.group(4).is_err());
        assert!(backing.page(2048).is_err());
    }
}
