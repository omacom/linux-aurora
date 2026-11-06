// SPDX-License-Identifier: GPL-2.0-only OR MIT

pub(crate) const INFO_SIZE: usize = 0xb4;
pub(crate) const STATE_SIZE: usize = 0x60;
pub(crate) const DONE: usize = 0x00;
pub(crate) const READ: usize = 0x30;
pub(crate) const WRITE: usize = 0x40;
pub(crate) const CAPACITY: usize = 0x50;
pub(crate) const QUALIFIED_CAPACITY: u32 = 0x500;

mod info {
    pub(crate) const STATE: usize = 0x00;
    pub(crate) const RING: usize = 0x08;
    pub(crate) const JOB_LIST: usize = 0x10;
    pub(crate) const GPU_SCRATCH: usize = 0x18;
    pub(crate) const EVENT_ID: usize = 0x2c;
    pub(crate) const PRIORITY_UNKNOWN_38: usize = 0x38;
    pub(crate) const PRIORITY_UNKNOWN_40: usize = 0x40;
    pub(crate) const PRIORITY_5: usize = 0x48;
    pub(crate) const UNKNOWN_4C: usize = 0x4c;
    pub(crate) const UUID: usize = 0x50;
    // V14_8_3 adds a four-byte field before this unaligned pointer.
    pub(crate) const CONTEXT: usize = 0xa4;
    pub(crate) const UNKNOWN_TAIL: usize = 0xac;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Error { Address, Size, Index }

/// Aligned, non-null firmware reference. Construction at the allocation
/// boundary prevents accidentally passing a low user GPU or compact address.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FirmwareVa(u64);
impl FirmwareVa {
    pub(crate) fn get(self) -> u64 { self.0 }
    pub(crate) fn new(value: u64) -> Result<Self, Error> {
        if value < 0xffff_fc00_0000_0000 || value & 3 != 0 {
            return Err(Error::Address);
        }
        Ok(Self(value))
    }
}

/// Insert a barrier ahead of the last unpublished command: overwrite that
/// command's ring slot, move the command to the producer slot, then advance.
pub(crate) fn insert_before_last(head:u16, capacity:u32)->Result<([u16;2],u16),Error> {
    if capacity<2 || capacity>u16::MAX as u32 || u32::from(head)>=capacity {
        return Err(Error::Index);
    }
    let previous=(u32::from(head)+capacity-1)%capacity;
    let next=(u32::from(head)+1)%capacity;
    Ok(([previous as u16,head],next as u16))
}

pub(crate) struct QueueInfo {
    pub(crate) state: FirmwareVa,
    pub(crate) ring: FirmwareVa,
    pub(crate) job_list: FirmwareVa,
    pub(crate) gpu_scratch: FirmwareVa,
    pub(crate) context: FirmwareVa,
    pub(crate) uuid: u32,
}

impl QueueInfo {
    /// Initialize exactly INFO_SIZE bytes, preserving any trailing storage.
    /// All validation precedes writes: failure leaves the buffer unchanged.
    pub(crate) fn encode(&self, output: &mut [u8]) -> Result<(), Error> {
        let bytes = output.get_mut(..INFO_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for (offset, value) in [
            (info::STATE, self.state.0), (info::RING, self.ring.0),
            (info::JOB_LIST, self.job_list.0), (info::GPU_SCRATCH, self.gpu_scratch.0),
            (info::CONTEXT, self.context.0),
            (info::PRIORITY_UNKNOWN_38, 0xffff_ffff_ffff_0000),
            (info::UNKNOWN_TAIL, 4),
        ] { bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes()); }
        for (offset, value) in [
            (info::EVENT_ID, u32::MAX), (info::UNKNOWN_4C, u32::MAX),
            (info::PRIORITY_UNKNOWN_40, 1), (info::PRIORITY_5, 1),
            (info::UUID, self.uuid),
        ] { bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes()); }
        Ok(())
    }
}

pub(crate) struct RingState {
    pub(crate) capacity: u32,
    pub(crate) done: u32,
    pub(crate) read: u32,
    pub(crate) write: u32,
}
impl RingState {
    /// Same no-mutation-on-error and trailing-storage contract as QueueInfo.
    pub(crate) fn encode(&self, output: &mut [u8]) -> Result<(), Error> {
        if self.capacity == 0 || self.capacity > 65536
            || self.done >= self.capacity || self.read >= self.capacity || self.write >= self.capacity {
            return Err(Error::Index);
        }
        let bytes = output.get_mut(..STATE_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for (offset, value) in [(DONE, self.done), (READ, self.read),
            (WRITE, self.write), (CAPACITY, self.capacity)] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        Ok(())
    }
}

/// Preserve the offline builder's initial entries and poison unused
/// entries. Runtime starts with write=0 and publishes live entries before
/// advancing WRITE, so no poisoned or seed entry is dispatched accidentally.
pub(crate) fn ring_entry(index: u32, capacity: u32, seeds: &[FirmwareVa]) -> Result<[u8; 8], Error> {
    if seeds.is_empty() || seeds.len() > capacity as usize || capacity > 65536 || index >= capacity { return Err(Error::Index); }
    Ok(if (index as usize) < seeds.len() { seeds[index as usize].0.to_le_bytes() } else { [0xcc; 8] })
}

impl QueueInfo {
    pub(crate) const EXTENDED_SIZE: usize = 0x24c0;

    /// Scheduler storage belongs to this same retained allocation; it is not
    /// interchangeable with the legacy GPU scratch object.
    pub(crate) fn encode_extended(&self, owner: FirmwareVa, output: &mut [u8]) -> Result<(), Error> {
        let end = owner.get().checked_add(Self::EXTENDED_SIZE as u64).ok_or(Error::Address)?;
        if end > 0xffff_fc30_0000_0000 { return Err(Error::Address); }
        let bytes = output.get_mut(..Self::EXTENDED_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for (offset, value) in [(0, self.state.get()), (8, self.ring.get()),
            (16, self.job_list.get()), (24, owner.get() + 0xb0), (0xa4, self.context.get())] {
            bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
        for offset in [0x2c, 0x4c] { bytes[offset..offset + 4].fill(0xff); }
        for offset in [0x30, 0x34, 0x48] {
            bytes[offset..offset + 4].copy_from_slice(&2u32.to_le_bytes());
        }
        bytes[0x38..0x40].copy_from_slice(&0xffff_0000_0000_0000u64.to_le_bytes());
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    #[test] fn scheduler_pointer_relocates_with_its_retained_queue_owner() {
        let fw = |i: u64| FirmwareVa::new(0xffff_fc21_4000_0000 + i * 0x4000).unwrap();
        let info = QueueInfo { state: fw(1), ring: fw(2), job_list: fw(3),
            gpu_scratch: fw(4), context: fw(5), uuid: 9 };
        let mut output = vec![0xa5; QueueInfo::EXTENDED_SIZE + 8];
        info.encode_extended(fw(0), &mut output).unwrap();
        assert_eq!(u64::from_le_bytes(output[24..32].try_into().unwrap()), fw(0).get() + 0xb0);
        assert_eq!(&output[0xb0..QueueInfo::EXTENDED_SIZE], &vec![0; QueueInfo::EXTENDED_SIZE - 0xb0]);
        assert_eq!(&output[QueueInfo::EXTENDED_SIZE..], &[0xa5; 8]);
        let mut short = [0xa5; INFO_SIZE];
        assert!(info.encode_extended(fw(0), &mut short).is_err());
        assert_eq!(short, [0xa5; INFO_SIZE]);
    }
}
