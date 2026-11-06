// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T8122 firmware MMIO descriptors. Mapping ownership stays with the M3 runtime.

use super::m3_init_layout::{Error, Region};

pub(crate) const SLOTS: usize = 53;
pub(crate) const RECORD_BYTES: usize = 40;
pub(crate) const TABLE_OFFSET: usize = 0x640;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Area {
    pub(crate) physical: u64,
    pub(crate) bytes: usize,
    relative_register: u64,
    writable: bool,
    split: bool,
}

impl Area {
    pub(crate) fn page_offset(self) -> usize {
        (self.physical & 0x3fff) as usize
    }

    pub(crate) fn mapped_bytes(self) -> usize {
        (self.page_offset() + self.bytes + 0x3fff) & !0x3fff
    }

    /// At most two physical banks; the second bank occupies the next virtual pages.
    pub(crate) fn banks(self) -> [(u64, usize, usize); 2] {
        if self.split {
            [
                (self.physical, self.bytes / 2, 0),
                (self.physical + 0x200_0000, self.bytes / 2, self.bytes / 2),
            ]
        } else {
            [(self.physical & !0x3fff, self.mapped_bytes(), 0), (0, 0, 0)]
        }
    }
}

/// Register resources obtained from the admitted T8122 hardware configuration.
/// Their physical addresses are supplied by the owner, not duplicated in a wire table.
#[derive(Clone, Copy)]
pub(crate) struct RegisterBases {
    pub(crate) gpu: u64,
    pub(crate) interrupt: u64,
    pub(crate) memory: u64,
    pub(crate) telemetry_write: u64,
    pub(crate) telemetry_read: u64,
    pub(crate) neural: u64,
    pub(crate) metrology: u64,
}

/// Select the wire slot and map its logical hardware block independently of its VA.
pub(crate) fn area(slot: usize, bases: RegisterBases) -> Result<Option<Area>, Error> {
    let (base, offset, bytes, tagged, writable, split) = match slot {
        0 => (bases.interrupt, 0, 0x4000, false, true, false),
        3 => (bases.memory, 0, 0x30000, false, true, true),
        9 => (bases.telemetry_write, 0, 0x1000, false, true, false),
        10 => (bases.telemetry_read, 0, 0x2000, false, false, false),
        12 => (bases.neural, 0, 0x4000, false, true, false),
        14 => (bases.metrology, 0, 0x8000, false, false, false),
        17 => (bases.gpu, 0, 0x20000, false, true, false),
        26 => (bases.gpu, 0xd04000, 0x8000, true, true, false),
        27 => (bases.gpu, 0xd0d000, 0x1000, true, true, false),
        28 => (bases.gpu, 0xd50000, 0x8000, true, true, false),
        29 => (bases.gpu, 0xd10000, 0x4000, true, true, false),
        31 => (bases.gpu, 0xd40000, 0x4000, true, true, false),
        32 => (bases.gpu, 0xd60000, 0x8000, true, true, false),
        35 => (bases.gpu, 0xe00000, 0x4000, true, true, false),
        39 => (bases.gpu, 0xe08000, 0x8000, false, true, false),
        40 => (bases.gpu, 0xe1c000, 0x4000, true, true, false),
        0..SLOTS => return Ok(None),
        _ => return Err(Error::Bounds),
    };
    let physical = base.checked_add(offset).ok_or(Error::Address)?;
    if physical == 0 || physical.checked_add(bytes as u64).ok_or(Error::Address)? > 1 << 42 {
        return Err(Error::Address);
    }
    Ok(Some(Area {
        physical,
        bytes,
        relative_register: if tagged { offset } else { 0 },
        writable,
        split,
    }))
}

fn empty_mode(slot: usize) -> u64 {
    if matches!(slot, 2 | 5..=8 | 30 | 33 | 34 | 37 | 38 | 41..=43 | 46..=49 | 52) {
        2
    } else {
        0
    }
}

/// Encode a complete record after checking the owned mapping; errors preserve all output.
pub(crate) fn encode(
    slot: usize,
    bases: RegisterBases,
    mapping: Option<Region>,
    output: &mut [u8],
) -> Result<(), Error> {
    let descriptor = area(slot, bases)?;
    let (physical, va, bytes, requested, register, mode) = match (descriptor, mapping) {
        (None, None) => (0, 0, 0, 0, 0, empty_mode(slot)),
        (Some(item), Some(mapping)) => {
            let page = mapping.at(0, item.mapped_bytes())?;
            if page & 0x3fff != 0 {
                return Err(Error::Address);
            }
            let va = page
                .checked_add(item.page_offset() as u64)
                .ok_or(Error::Address)?;
            (
                item.physical,
                va,
                item.bytes as u32,
                if item.split {
                    (item.bytes / 2) as u32
                } else {
                    item.bytes as u32
                },
                item.relative_register,
                if item.writable { 2 } else { 0 },
            )
        }
        _ => return Err(Error::Address),
    };
    let bytes_out = output.get_mut(..RECORD_BYTES).ok_or(Error::Size)?;
    bytes_out.fill(0);
    bytes_out[0..8].copy_from_slice(&physical.to_le_bytes());
    bytes_out[8..16].copy_from_slice(&va.to_le_bytes());
    bytes_out[16..20].copy_from_slice(&bytes.to_le_bytes());
    bytes_out[20..24].copy_from_slice(&requested.to_le_bytes());
    bytes_out[24..32].copy_from_slice(&register.to_le_bytes());
    bytes_out[32..40].copy_from_slice(&mode.to_le_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Register identities already present in the public T8122 configuration;
    // memory-controller bank geometry is shared with the public 25G83 encoder.
    const BASES: RegisterBases = RegisterBases {
        gpu: 0x2_9000_0000,
        interrupt: 0x2_d101_4000,
        memory: 0x2_201c_4000,
        telemetry_write: 0x2_d03d_0000,
        telemetry_read: 0x2_d03c_0000,
        neural: 0x3_1145_c000,
        metrology: 0x2_d028_0000,
    };

    #[test]
    fn j613_slot28_bank_and_wire_tag_agree() {
        let bank = area(28, BASES).unwrap().unwrap();
        assert_eq!(bank.physical, BASES.gpu + 0xd50000);
        assert_eq!(bank.bytes, 0x8000);
        let mapped = Region::new(0xffff_fc23_01c0_0000, 0x8000).unwrap();
        let mut bytes = [0; RECORD_BYTES];
        encode(28, BASES, Some(mapped), &mut bytes).unwrap();
        assert_eq!(&bytes[0..8], &(BASES.gpu + 0xd50000).to_le_bytes());
        assert_eq!(&bytes[24..32], &0xd50000u64.to_le_bytes());
        assert_ne!(bank.physical, BASES.gpu + 0xd58000);
    }

    #[test]
    fn descriptor_inventory_ends_before_the_retention_pointer() {
        assert_eq!(TABLE_OFFSET + SLOTS * RECORD_BYTES, 0xe88);
        assert_eq!(
            (0..SLOTS)
                .filter(|slot| area(*slot, BASES).unwrap().is_some())
                .count(),
            16
        );
        assert!(area(SLOTS, BASES).is_err());
    }

    #[test]
    fn discontiguous_memory_controller_banks_have_distinct_physical_owners() {
        let item = area(3, BASES).unwrap().unwrap();
        assert_eq!(
            item.banks(),
            [
                (0x2_201c_4000, 0x18000, 0),
                (0x2_221c_4000, 0x18000, 0x18000)
            ]
        );
        let mapping = Region::new(0xffff_fc23_0000_0000, 0x30000).unwrap();
        let mut bytes = [0xa5; RECORD_BYTES];
        encode(3, BASES, Some(mapping), &mut bytes).unwrap();
        assert_eq!(
            u32::from_le_bytes(bytes[16..20].try_into().unwrap()),
            0x30000
        );
        assert_eq!(
            u32::from_le_bytes(bytes[20..24].try_into().unwrap()),
            0x18000
        );
    }

    #[test]
    fn subpage_mapping_keeps_the_register_offset() {
        let item = area(27, BASES).unwrap().unwrap();
        assert_eq!(item.page_offset(), 0x1000);
        assert_eq!(item.banks()[0], (0x2_90d0_c000, 0x4000, 0));
        let mut output = [0xa5; RECORD_BYTES + 8];
        encode(
            27,
            BASES,
            Some(Region::new(0xffff_fc23_0000_0000, 0x4000).unwrap()),
            &mut output,
        )
        .unwrap();
        assert_eq!(
            u64::from_le_bytes(output[8..16].try_into().unwrap()),
            0xffff_fc23_0000_1000
        );
        assert_eq!(&output[RECORD_BYTES..], &[0xa5; 8]);
    }

    #[test]
    fn empty_slots_keep_modes_without_fabricating_mappings() {
        for slot in 0..SLOTS {
            if area(slot, BASES).unwrap().is_some() {
                continue;
            }
            let mut output = [0xa5; RECORD_BYTES];
            encode(slot, BASES, None, &mut output).unwrap();
            assert_eq!(&output[..32], &[0; 32]);
            assert_eq!(
                u64::from_le_bytes(output[32..40].try_into().unwrap()),
                empty_mode(slot)
            );
        }
    }

    #[test]
    fn invalid_mapping_or_output_never_partially_changes_a_descriptor() {
        let good = Region::new(0xffff_fc23_0000_0000, 0x4000).unwrap();
        for (slot, mapping) in [
            (SLOTS, None),
            (0, None),
            (1, Some(good)),
            (0, Some(Region::new(0xffff_fc23_0000_0001, 0x4000).unwrap())),
            (0, Some(Region::new(0xffff_fc23_0000_0000, 0x3fff).unwrap())),
        ] {
            let mut output = [0xa5; RECORD_BYTES];
            assert!(encode(slot, BASES, mapping, &mut output).is_err());
            assert_eq!(output, [0xa5; RECORD_BYTES]);
        }
        for length in 0..RECORD_BYTES {
            let mut output = [0xa5; RECORD_BYTES];
            assert_eq!(
                encode(0, BASES, Some(good), &mut output[..length]),
                Err(Error::Size)
            );
            assert_eq!(output, [0xa5; RECORD_BYTES]);
        }
    }
}
