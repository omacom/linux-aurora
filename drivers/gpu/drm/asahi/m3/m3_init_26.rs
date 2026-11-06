// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Allocation and pointer records for the extended M3 initialization format.
//! Owners supply mapped regions; these builders never allocate or publish DMA.

use super::m3_init_layout::{Error, Region, Root};
use super::m3_init_storage::{self as storage, Allocation, Initial};

pub(crate) const COUNT: usize = 56;
pub(crate) const DYNAMIC_STATE: usize = 49;
pub(crate) const PERFORMANCE_CONTROL: usize = 50;
pub(crate) const RECOVERY_ACTIVITY: usize = 51;
pub(crate) const SENSOR_VALUE: usize = 52;
pub(crate) const RETENTION_STATE: usize = 53;
pub(crate) const GLOBAL_POOL: usize = 54;
pub(crate) const GLOBAL_PARAMETERS: usize = 55;
pub(crate) const RUNTIME_SIZE: usize = 0x4b8;
pub(crate) const LOG_THREADS: usize = 9;
pub(crate) const RX_BYTES: usize = 0x48;
pub(crate) const LOG_PAYLOAD_BYTES: usize = 0xd8;

/// Extend the existing M3 owner indices instead of introducing another owner graph.
pub(crate) fn allocation(index: usize) -> Result<Allocation, Error> {
    let size = match index {
        0..=23 if index % 2 == 0 => 0x30,
        0..=23 => 0x1800,
        24 | 26 | 30 | 32 => 0x30,
        25 => 0x4000,
        27 | 33 => 0x4800,
        28 => LOG_THREADS * 0x30,
        29 => LOG_THREADS * 256 * RX_BYTES,
        31 => 512 * RX_BYTES,
        34 => 0x40,
        35 => 0x3fc0,
        36 => 0xc8,
        37 => 0x4000,
        38 => RUNTIME_SIZE,
        39 => 0x2658,
        40 => 0x88,
        41 => LOG_THREADS * 256 * LOG_PAYLOAD_BYTES,
        42 => 0xc18,
        43 => 0x1248,
        44 => 0xe10,
        45 => 0xe48,
        46 => 0x670,
        47 => 0xeaf0,
        48 => 0x20,
        DYNAMIC_STATE | RETENTION_STATE | GLOBAL_POOL | GLOBAL_PARAMETERS => 0x4000,
        PERFORMANCE_CONTROL => 0x6578,
        RECOVERY_ACTIVITY => 0x60,
        SENSOR_VALUE => 4,
        _ => return Err(Error::Bounds),
    };
    // Separate page-aligned owners, with room for the largest receive allocation.
    // This namespace is disjoint from the existing fixed work and timestamp arenas.
    Ok(Allocation {
        address: 0xffff_fc21_0000_0000 + (index as u64) * 0x10_0000,
        size,
        gpu_shared: matches!(index, GLOBAL_POOL | GLOBAL_PARAMETERS),
        initial: Initial::Zero,
    })
}

pub(crate) fn cached(index: usize) -> bool {
    matches!(index, 38 | 39 | 42 | 43 | 44 | PERFORMANCE_CONTROL)
}

pub(crate) struct PointerGraph<'a> {
    pub(crate) owners: &'a [Region; COUNT],
    pub(crate) scratch: Region,
    pub(crate) pool_alias: u64,
    pub(crate) parameter_alias: u64,
}

impl PointerGraph<'_> {
    /// Check the complete graph, including padded mapping ownership, before encoding it.
    fn addresses(&self) -> Result<[u64; COUNT], Error> {
        let mut addresses = [0; COUNT];
        for (index, owner) in self.owners.iter().enumerate() {
            let size = allocation(index)?.size;
            addresses[index] = owner.at(0, size)?;
            if addresses[index] & 0x3fff != 0
                || addresses[index] < 0xffff_fc20_0000_0000
                || addresses[index] >= 0xffff_fc2f_fc00_0000
            {
                return Err(Error::Address);
            }
            let mapped = (size + 0x3fff) & !0x3fff;
            let end = addresses[index]
                .checked_add(mapped as u64)
                .ok_or(Error::Address)?;
            if end > 0xffff_fc2f_fc00_0000 {
                return Err(Error::Address);
            }
            for prior in 0..index {
                let prior_size = (allocation(prior)?.size + 0x3fff) & !0x3fff;
                if addresses[index] < addresses[prior] + prior_size as u64
                    && addresses[prior] < addresses[index] + mapped as u64
                {
                    return Err(Error::Address);
                }
            }
        }
        let scratch = self.scratch.at(0, 0x8000)?;
        let scratch_end = scratch.checked_add(0x8000).ok_or(Error::Address)?;
        if scratch & 0x3fff != 0 || scratch_end > 0xffff_fc2f_fc00_0000 {
            return Err(Error::Address);
        }
        for (index, address) in addresses.iter().enumerate() {
            let bytes = (allocation(index)?.size + 0x3fff) & !0x3fff;
            if scratch < *address + bytes as u64 && *address < scratch_end {
                return Err(Error::Address);
            }
        }
        for alias in [self.pool_alias, self.parameter_alias] {
            if alias & 0x3fff != 0 || !(0x70_0000_0000..0x70_0200_0000).contains(&alias) {
                return Err(Error::Address);
            }
        }
        if self.pool_alias == self.parameter_alias {
            return Err(Error::Address);
        }
        Ok(addresses)
    }

    pub(crate) fn root(&self, output: &mut [u8]) -> Result<(), Error> {
        self.addresses()?;
        Root {
            region_a: self.owners[storage::REGION_A],
            runtime: self.owners[storage::RUNTIME_POINTERS],
            globals: self.owners[storage::GLOBALS],
            new_a: self.owners[storage::RUNTIME_FLAGS],
            control: self.owners[storage::CONTROL_REGION],
            power: self.owners[storage::GLOBALS_POWER],
        }
        .encode_26(output, self.owners[DYNAMIC_STATE])
    }

    pub(crate) fn runtime(&self, output: &mut [u8]) -> Result<(), Error> {
        let addresses = self.addresses()?;
        let scratch = self.scratch.at(0, 0x8000)?;
        let bytes = output.get_mut(..RUNTIME_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        let pointer = |bytes: &mut [u8], offset: usize, value: u64| {
            bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        };
        pointer(bytes, 0x18, scratch);
        pointer(bytes, 0x2d0, self.parameter_alias);
        pointer(bytes, 0x2e0, self.pool_alias);
        bytes[0x250..0x254].copy_from_slice(&1u32.to_le_bytes());
        bytes[0x2f8..0x2fc].copy_from_slice(&4u32.to_le_bytes());
        for (offset, owner) in [
            (0, storage::HARDWARE_DATA),
            (0x10, storage::UNKNOWN_PAIR),
            (0x1c0, 26),
            (0x1c8, 27),
            (0x1d0, 28),
            (0x1d8, 29),
            (0x1e0, 30),
            (0x1e8, 31),
            (0x1f0, 32),
            (0x1f8, 33),
            (0x200, storage::FWLOG_PAYLOAD),
            (0x254, storage::UNKNOWN_C0),
            (0x25c, storage::UNKNOWN_C1),
            (0x264, storage::UNKNOWN_C3),
            (0x26c, RECOVERY_ACTIVITY),
            (0x2d8, GLOBAL_PARAMETERS),
            (0x2e8, GLOBAL_POOL),
            (0x469, PERFORMANCE_CONTROL),
        ] {
            pointer(bytes, offset, addresses[owner]);
        }
        for channel in 0..13 {
            let offset = 0x20 + channel * 0x20;
            let state = addresses[channel * 2];
            pointer(bytes, offset, state);
            pointer(bytes, offset + 8, state + 0x10);
            pointer(bytes, offset + 16, state + 0x20);
            pointer(bytes, offset + 24, addresses[channel * 2 + 1]);
        }
        Ok(())
    }

    pub(crate) fn firmware_control(&self, output: &mut [u8]) -> Result<(), Error> {
        let addresses = self.addresses()?;
        let bytes = output
            .get_mut(..allocation(storage::CONTROL_REGION)?.size)
            .ok_or(Error::Size)?;
        bytes.fill(0);
        bytes[0x4f88..0x4f90].copy_from_slice(&addresses[storage::FW_CONTROL_STATE].to_le_bytes());
        bytes[0x4f90..0x4f98].copy_from_slice(&addresses[storage::FW_CONTROL_RING].to_le_bytes());
        bytes[0x4fd4..0x4fd8].copy_from_slice(&1u32.to_le_bytes());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn owners(delta: u64) -> [Region; COUNT] {
        core::array::from_fn(|index| {
            let item = allocation(index).unwrap();
            Region::new(item.address + delta, item.size).unwrap()
        })
    }

    fn graph(owners: &[Region; COUNT]) -> PointerGraph<'_> {
        PointerGraph {
            owners,
            scratch: Region::new(0xffff_fc22_0000_0000, 0x8000).unwrap(),
            pool_alias: 0x70_0000_0000,
            parameter_alias: 0x70_0000_8000,
        }
    }

    #[test]
    fn complete_plan_has_disjoint_page_owners_and_nine_log_threads() {
        let owners = owners(0);
        assert!(graph(&owners).addresses().is_ok());
        assert_eq!(allocation(28).unwrap().size, 0x1b0);
        assert_eq!(allocation(29).unwrap().size, 0x28800);
        assert_eq!(allocation(41).unwrap().size, 0x79800);
        assert!(allocation(COUNT).is_err());
    }

    #[test]
    fn runtime_references_relocate_and_keep_the_packed_power_pointer() {
        for delta in [0, 0x1000_0000] {
            let owners = owners(delta);
            let mut output = [0xa5; RUNTIME_SIZE + 8];
            graph(&owners).runtime(&mut output).unwrap();
            let read = |at| u64::from_le_bytes(output[at..at + 8].try_into().unwrap());
            assert_eq!(
                read(0x469),
                allocation(PERFORMANCE_CONTROL).unwrap().address + delta
            );
            assert_eq!(read(0x1a0), allocation(24).unwrap().address + delta);
            assert_eq!(read(0x1b8), allocation(25).unwrap().address + delta);
            assert_eq!(read(0x2d0), 0x70_0000_8000);
            assert_eq!(&output[RUNTIME_SIZE..], &[0xa5; 8]);
        }
    }

    #[test]
    fn every_short_runtime_rejects_before_mutation() {
        let owners = owners(0);
        for length in 0..RUNTIME_SIZE {
            let mut output = [0xa5; RUNTIME_SIZE];
            assert_eq!(
                graph(&owners).runtime(&mut output[..length]),
                Err(Error::Size)
            );
            assert_eq!(output, [0xa5; RUNTIME_SIZE]);
        }
    }

    #[test]
    fn every_owner_is_checked_before_any_record_is_written() {
        for index in 0..COUNT {
            let mut owners = owners(0);
            let item = allocation(index).unwrap();
            owners[index] = Region::new(item.address, item.size - 1).unwrap();
            let mut output = [0xa5; RUNTIME_SIZE];
            assert!(graph(&owners).runtime(&mut output).is_err());
            assert_eq!(output, [0xa5; RUNTIME_SIZE]);
        }
        let mut owners = owners(0);
        owners[1] = owners[0];
        let mut output = [0xa5; RUNTIME_SIZE];
        assert!(graph(&owners).runtime(&mut output).is_err());
        assert_eq!(output, [0xa5; RUNTIME_SIZE]);
    }
}
