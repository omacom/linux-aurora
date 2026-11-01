// SPDX-License-Identifier: GPL-2.0-only OR MIT

pub(crate) const ROOT_SIZE: usize = 0xc0;
pub(crate) const RUNTIME_SIZE: usize = 0x4b3;
pub(crate) const CONTROL_SIZE: usize = 0xc3d0;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error { Address, Bounds, Size }

pub(crate) mod fwlog {
    use super::Error;
    pub(crate) const CHANNELS: usize = 6;
    pub(crate) const SLOTS: usize = 256;
    pub(crate) const ENTRY_SIZE: usize = 0x38;
    pub(crate) const PAYLOAD_SIZE: usize = 0xd8;
    pub(crate) const RING_BYTES: usize = CHANNELS * SLOTS * ENTRY_SIZE;
    pub(crate) const PAYLOAD_BYTES: usize = CHANNELS * SLOTS * PAYLOAD_SIZE;

    pub(crate) fn payload_offset(channel: usize, index: u64) -> Result<usize, Error> {
        if channel >= CHANNELS || index >= SLOTS as u64 { return Err(Error::Bounds); }
        Ok((channel * SLOTS + index as usize) * PAYLOAD_SIZE)
    }
}

/// An owned firmware region, not a physical, GPU or compact address. Unlike
/// queue references, byte alignment is intentional here: the qualified runtime
/// record starts at ...7b4d. Packed references must not become aligned loads.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Region { base: u64, size: usize }
impl Region {
    pub(crate) fn new(base: u64, size: usize) -> Result<Self, Error> {
        if base < 0xffff_fc00_0000_0000 || size == 0 || base.checked_add(size as u64).is_none() {
            return Err(Error::Address);
        }
        Ok(Self { base, size })
    }
    pub(crate) fn at(self, offset: usize, width: usize) -> Result<u64, Error> {
        if offset.checked_add(width).ok_or(Error::Bounds)? > self.size { return Err(Error::Bounds); }
        Ok(self.base + offset as u64)
    }
}
fn word(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}
fn pointer(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}
mod root {
    pub(crate) const VERSION: usize = 0;
    pub(crate) const REGION_A: usize = 8;
    pub(crate) const RUNTIME: usize = 0x18;
    pub(crate) const GLOBALS: usize = 0x20;
    pub(crate) const HOST_ALLOCATIONS: usize = 0x2c;
    pub(crate) const PAGE_SIZE: usize = 0x30;
    pub(crate) const PAGE_BITS: usize = 0x32;
    pub(crate) const LEVEL_COUNT: usize = 0x33;
    pub(crate) const LEVELS: usize = 0x34;
    pub(crate) const LEVEL_STRIDE: usize = 0x20;
    pub(crate) const NEW_A: usize = 0xa8;
    pub(crate) const CONTROL: usize = 0xb0;
    pub(crate) const POWER: usize = 0xb8;
}
pub(crate) struct Root {
    pub(crate) region_a: Region,
    pub(crate) runtime: Region,
    pub(crate) globals: Region,
    pub(crate) new_a: Region,
    pub(crate) control: Region,
    pub(crate) power: Region,
}
impl Root {
    /// All bounds are checked before mutation; trailing storage is preserved.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let references = [
            (root::REGION_A, self.region_a.at(0, 0x4000)?),
            (root::RUNTIME, self.runtime.at(0, RUNTIME_SIZE)?),
            (root::GLOBALS, self.globals.at(0, 15868)?),
            (root::NEW_A, self.new_a.at(0, 0x40)?),
            (root::CONTROL, self.control.at(0, CONTROL_SIZE)?),
            (root::POWER, self.power.at(0, 2292)?),
        ];
        Self::write_root(out, ROOT_SIZE, 0x0c08_e21e_8380_0490, &references)
    }

    /// Encode the extended root without admitting a firmware image or publishing it.
    pub(crate) fn encode_26(&self, out: &mut [u8], extra: Region) -> Result<(), Error> {
        let references = [
            (root::REGION_A, self.region_a.at(0, 0x4000)?),
            (root::RUNTIME, self.runtime.at(0, 0x4b8)?),
            (root::GLOBALS, self.globals.at(0, 0xe48)?),
            (root::NEW_A, self.new_a.at(0, 0x20)?),
            (root::CONTROL, self.control.at(0, 0xeaf0)?),
            (root::POWER, self.power.at(0, 0x670)?),
            (0xc0, extra.at(0, 0x4000)?),
        ];
        if references.iter().any(|(_, address)| address & 7 != 0) {
            return Err(Error::Address);
        }
        Self::write_root(out, 0xc8, 0x0c89_c357_8392_04b8, &references)
    }

    fn write_root(
        out: &mut [u8],
        size: usize,
        version: u64,
        references: &[(usize, u64)],
    ) -> Result<(), Error> {
        let bytes = out.get_mut(..size).ok_or(Error::Size)?;
        bytes.fill(0);
        pointer(bytes, root::VERSION, version);
        for &(offset, value) in references {
            pointer(bytes, offset, value);
        }
        word(bytes, root::HOST_ALLOCATIONS, 1);
        bytes[root::PAGE_SIZE..root::PAGE_SIZE + 2].copy_from_slice(&0x4000u16.to_le_bytes());
        bytes[root::PAGE_BITS] = 14;
        bytes[root::LEVEL_COUNT] = 3;
        for (i, (shift, entries)) in [(36u8, 64u16), (25, 2048), (14, 2048)].iter().enumerate() {
            let start = root::LEVELS + i * root::LEVEL_STRIDE;
            bytes[start..start + 4].copy_from_slice(&[8, 14, 14, *shift]);
            bytes[start + 4..start + 6].copy_from_slice(&entries.to_le_bytes());
            bytes[start + 6..start + 8].copy_from_slice(&0x4000u16.to_le_bytes());
            pointer(bytes, start + 8, 1); // UatLevelInfo.unk_8
            pointer(bytes, start + 16, 0x3ffffffc000); // G15 physical page mask.
            pointer(bytes, start + 24, (u64::from(*entries) - 1) << shift);
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Channel { pub(crate) state: Region, pub(crate) ring: Region }
mod runtime {
    pub(crate) const HARDWARE: usize = 0;
    pub(crate) const UNKNOWN_PAIR: [usize; 2] = [8, 16];
    pub(crate) const HOST_CHANNELS: usize = 0x18;
    pub(crate) const HOST_CHANNEL_COUNT: usize = 13;
    pub(crate) const HOST_CHANNEL_STRIDE: usize = 0x20;
    pub(crate) const FIRMWARE_CHANNELS: usize = 0x1b8;
    pub(crate) const FIRMWARE_CHANNEL_STRIDE: usize = 0x10;
    pub(crate) const FWLOG_PAYLOAD: usize = 0x1f8;
    pub(crate) const HARDWARE_B_VIEWS: [usize; 4] = [0x234, 0x23c, 0x244, 0x24c];
    pub(crate) const UNKNOWN_C0: usize = 0x2a8;
    pub(crate) const UNKNOWN_C1: [usize; 2] = [0x2b0, 0x2b8];
    pub(crate) const UNKNOWN_C3: [usize; 2] = [0x2c0, 0x2c8];
    pub(crate) const UNKNOWN_2D0: usize = 0x2d0;
    pub(crate) const UNKNOWN_3B0: usize = 0x3b0;
    pub(crate) const HARDWARE_A: usize = 0x441;
}
pub(crate) struct RuntimePointers {
    pub(crate) hardware: Region,
    pub(crate) unknown_pair: Region,
    pub(crate) fwlog_payload: Region,
    pub(crate) unknown_c0: Region,
    pub(crate) unknown_c1: Region,
    pub(crate) unknown_c3: Region,
    /// Twelve TA/3D/CL host channels, DevCtrl, then Event/FWLog/KTrace/Stats.
    pub(crate) channels: [Channel; 17],
}
impl RuntimePointers {
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let hardware = self.hardware.at(0, 35332)?;
        let pair = self.unknown_pair.at(0, 0x100)?;
        let fwlog_payload = self.fwlog_payload.at(0, fwlog::PAYLOAD_BYTES)?;
        let c0 = self.unknown_c0.at(0, 0x1000)?;
        let c1 = self.unknown_c1.at(0, 0x4000)?;
        let c3 = self.unknown_c3.at(0, 0x4000)?;
        let mut channels = [[0u64; 4]; 17];
        for (index, channel) in self.channels.iter().enumerate() {
            channels[index] = [channel.state.at(0, 0x30)?, channel.state.at(0x10, 4)?,
                channel.state.at(0x20, 4)?, channel.ring.at(0, 1)?];
        }
        let bytes = out.get_mut(..RUNTIME_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        pointer(bytes, runtime::HARDWARE, hardware);
        for offset in runtime::UNKNOWN_PAIR { pointer(bytes, offset, pair); }
        for (index, values) in channels.iter().enumerate() {
            if index < runtime::HOST_CHANNEL_COUNT {
                let start = runtime::HOST_CHANNELS + index * runtime::HOST_CHANNEL_STRIDE;
                for (field, value) in values.iter().enumerate() { pointer(bytes, start + field * 8, *value); }
            } else {
                let start = runtime::FIRMWARE_CHANNELS + (index - runtime::HOST_CHANNEL_COUNT) * runtime::FIRMWARE_CHANNEL_STRIDE;
                pointer(bytes, start, values[0]);
                pointer(bytes, start + 8, values[3]);
            }
        }
        pointer(bytes, runtime::FWLOG_PAYLOAD, fwlog_payload);
        // These are views of the retained hardware data, not new allocations.
        for (offset, delta) in runtime::HARDWARE_B_VIEWS.iter().zip([0x1880u64, 0x24c0, 0x3740, 0x8900]) {
            pointer(bytes, *offset, hardware + delta);
        }
        pointer(bytes, runtime::UNKNOWN_C0, c0);
        for offset in runtime::UNKNOWN_C1 { pointer(bytes, offset, c1); }
        for offset in runtime::UNKNOWN_C3 { pointer(bytes, offset, c3); }
        word(bytes, runtime::UNKNOWN_2D0, 4);
        word(bytes, runtime::UNKNOWN_3B0, 0xff);
        pointer(bytes, runtime::HARDWARE_A, hardware + 0x4580);
        Ok(())
    }
}

mod control {
    pub(crate) const UNKNOWN_4: usize = 4;
    pub(crate) const CHANNEL_STATE: usize = 0x4568;
    pub(crate) const CHANNEL_RING: usize = 0x4570;
    pub(crate) const UNKNOWN_45C4: usize = 0x45c4;
}
pub(crate) struct ControlRegion { pub(crate) channel: Channel }
impl ControlRegion {
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let state = self.channel.state.at(0, 0x30)?;
        let ring = self.channel.ring.at(0, 1)?;
        let bytes = out.get_mut(..CONTROL_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        word(bytes, control::UNKNOWN_4, 1);
        pointer(bytes, control::CHANNEL_STATE, state);
        pointer(bytes, control::CHANNEL_RING, ring);
        word(bytes, control::UNKNOWN_45C4, 1);
        Ok(())
    }
}

#[cfg(test)]
mod root_format_tests {
    use super::*;

    fn allocations() -> (Root, Region) {
        let region =
            |index: u64| Region::new(0xffff_fc01_0000_0000 + index * 0x10000, 0x10000).unwrap();
        (
            Root {
                region_a: region(1),
                runtime: region(2),
                globals: region(3),
                new_a: region(4),
                control: region(5),
                power: region(6),
            },
            region(7),
        )
    }

    #[test]
    fn root_revision_changes_only_the_version_and_extension() {
        let (root, extra) = allocations();
        let mut older = [0x55; 0xd0];
        let mut newer = older;
        root.encode(&mut older).unwrap();
        root.encode_26(&mut newer, extra).unwrap();
        assert_eq!(&older[8..0xc0], &newer[8..0xc0]);
        assert_eq!(
            u64::from_le_bytes(newer[..8].try_into().unwrap()),
            0x0c89_c357_8392_04b8
        );
        assert_eq!(
            u64::from_le_bytes(newer[0xc0..0xc8].try_into().unwrap()),
            0xffff_fc01_0007_0000
        );
        assert_eq!(&older[0xc0..], &[0x55; 16]);
        assert_eq!(&newer[0xc8..], &[0x55; 8]);
        for (offset, index) in [
            (8, 1),
            (0x18, 2),
            (0x20, 3),
            (0xa8, 4),
            (0xb0, 5),
            (0xb8, 6),
        ] {
            assert_eq!(
                u64::from_le_bytes(newer[offset..offset + 8].try_into().unwrap()),
                0xffff_fc01_0000_0000 + index * 0x10000
            );
        }
    }

    #[test]
    fn every_short_destination_is_unchanged() {
        let (root, extra) = allocations();
        for length in 0..0xc8 {
            let mut output = [0xa5; 0xc8];
            assert_eq!(
                root.encode_26(&mut output[..length], extra),
                Err(Error::Size)
            );
            assert_eq!(output, [0xa5; 0xc8]);
        }
    }

    #[test]
    fn every_reference_requires_its_own_complete_allocation() {
        for (index, required) in [0x4000, 0x4b8, 0xe48, 0x20, 0xeaf0, 0x670, 0x4000]
            .into_iter()
            .enumerate()
        {
            for (size, aligned, valid) in [
                (required - 1, true, false),
                (required, false, false),
                (required, true, true),
            ] {
                let (mut root, mut extra) = allocations();
                let reference = match index {
                    0 => &mut root.region_a,
                    1 => &mut root.runtime,
                    2 => &mut root.globals,
                    3 => &mut root.new_a,
                    4 => &mut root.control,
                    5 => &mut root.power,
                    _ => &mut extra,
                };
                *reference =
                    Region::new(0xffff_fc02_0000_0000 + u64::from(!aligned), size).unwrap();
                let mut output = [0xa5; 0xc8];
                let result = root.encode_26(&mut output, extra);
                assert_eq!(
                    result.is_ok(),
                    valid,
                    "reference {index}, size {size}, aligned {aligned}"
                );
                if !valid {
                    assert_eq!(output, [0xa5; 0xc8]);
                }
            }
        }
    }
}
