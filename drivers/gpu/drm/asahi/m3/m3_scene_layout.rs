// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::m3_queue_layout::{Error, FirmwareVa};
pub(crate) const SIZE: usize = 0x80;
const GPU_APERTURE: u64 = 0x10_0000_0000;
/// A compact pointer is an offset in the qualified low GPU aperture, not a
/// firmware VA. Both subobjects must lie in the same owned GPU allocation.
#[derive(Clone, Copy)]
pub(crate) struct UserBuffer { header: u32, auxiliary: u32 }
impl UserBuffer {
    pub(crate) fn new(gpu: u64, size: usize) -> Result<Self, Error> {
        let base = gpu.checked_sub(GPU_APERTURE).ok_or(Error::Address)?;
        if gpu & 0x3f != 0 || size < 0x904 { return Err(Error::Address); }
        gpu.checked_add(size as u64).ok_or(Error::Address)?;
        let header = u32::try_from(base.checked_add(0x40).ok_or(Error::Address)?)
            .map_err(|_| Error::Address)?;
        let auxiliary = u32::try_from(base.checked_add(0x900).ok_or(Error::Address)?)
            .map_err(|_| Error::Address)?;
        Ok(Self { header, auxiliary })
    }
}
pub(crate) struct Scene {
    /// One u32 entry in the shared 48-entry buffer-manager scene list.
    pub(crate) manager_entry: FirmwareVa,
    pub(crate) manager_misc: FirmwareVa,
    pub(crate) user_buffer: UserBuffer,
}
impl Scene {
    /// All validation precedes mutation; trailing caller storage is untouched.
    /// Called after per-pass reset, never while this slot is firmware-owned.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let bytes = out.get_mut(..SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for offset in [0, 8] {
            bytes[offset..offset + 8].copy_from_slice(&self.manager_entry.get().to_le_bytes());
        }
        bytes[0x28..0x30].copy_from_slice(&u64::from(self.user_buffer.header).to_le_bytes());
        bytes[0x30..0x34].copy_from_slice(&self.user_buffer.auxiliary.to_le_bytes());
        bytes[0x40..0x48].copy_from_slice(&self.manager_misc.get().to_le_bytes());
        Ok(())
    }
}

pub(crate) const EXTENDED_SIZE: usize = 0x400;

/// Encode a native scene header from its retained aliases. The local
/// statistics and parameter scratch remain part of this same owned record.
pub(crate) fn encode_extended(output: &mut [u8], owner: super::m3_init_layout::Region,
    scratch_gpu: super::m3_pool_layout::GpuRegion, scratch_fw: FirmwareVa,
    scene_list: super::m3_pool_layout::GpuRegion) -> Result<(), Error> {
    let statistics = owner.at(0x200, 0x40).map_err(|_| Error::Size)?;
    owner.at(0, EXTENDED_SIZE).map_err(|_| Error::Size)?;
    if scratch_gpu.size() < 0x4000 || scene_list.size() < 0x4000 { return Err(Error::Size); }
    let compact = super::agx_render_state::compact(scene_list.base()).map_err(|_| Error::Address)?;
    let bytes = output.get_mut(..EXTENDED_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    bytes[..8].copy_from_slice(&scratch_gpu.base().to_le_bytes());
    bytes[8..16].copy_from_slice(&scratch_fw.get().to_le_bytes());
    bytes[0x28..0x30].copy_from_slice(&compact.to_le_bytes());
    bytes[0x40..0x48].copy_from_slice(&statistics.to_le_bytes());
    Ok(())
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    use super::super::{m3_init_layout::Region, m3_pool_layout::GpuRegion};
    #[test] fn native_scene_keeps_scratch_aliases_and_local_statistics_distinct() {
        let owner = Region::new(0xffff_fc25_0010_0000, EXTENDED_SIZE).unwrap();
        let scratch = GpuRegion::new(0x10_0200_0000, 0x4000).unwrap();
        let scratch_fw = FirmwareVa::new(0xffff_fc25_0020_0000).unwrap();
        let list = GpuRegion::new(0x10_0300_0000, 0x4000).unwrap();
        let mut bytes = [0xa5; EXTENDED_SIZE + 8];
        encode_extended(&mut bytes, owner, scratch, scratch_fw, list).unwrap();
        let read = |at| u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap());
        assert_eq!(read(0), scratch.base());
        assert_eq!(read(8), scratch_fw.get());
        assert_eq!(read(0x28), 0x300_0000);
        assert_eq!(read(0x40), owner.at(0x200, 0x40).unwrap());
        assert_eq!(&bytes[EXTENDED_SIZE..], &[0xa5; 8]);
    }
}
