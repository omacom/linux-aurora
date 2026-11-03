// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::m3_queue_layout::{Error, FirmwareVa};

pub(crate) const INIT_BM_SIZE: usize = 0x20;
pub(crate) const BARRIER_SIZE: usize = 0x3e;
mod init {
    pub(crate) const OPCODE: usize = 0;
    pub(crate) const CONTEXT: usize = 4;
    pub(crate) const SLOT: usize = 8;
    // 0xc: unresolved block_count2, zero in the qualified producer.
    pub(crate) const BLOCK_COUNT: usize = 0x10;
    pub(crate) const MANAGER: usize = 0x14;
    pub(crate) const STAMP: usize = 0x1c;
}
mod barrier {
    pub(crate) const OPCODE: usize = 0;
    pub(crate) const STAMP: usize = 4;
    pub(crate) const STAMP2: usize = 0xc;
    pub(crate) const WAIT_VALUE: usize = 0x14;
    pub(crate) const EVENT: usize = 0x18;
    pub(crate) const SELF_VALUE: usize = 0x1c;
    pub(crate) const UUID: usize = 0x20;
    // 0x24: unresolved u16, 0x26: external-barrier u32, both zero.
    pub(crate) const INTERNAL_TYPE: usize = 0x2a;
}
fn u32_at(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}
fn address_at(bytes: &mut [u8], offset: usize, value: FirmwareVa) {
    bytes[offset..offset + 8].copy_from_slice(&value.get().to_le_bytes());
}

pub(crate) struct InitBufferManager {
    pub(crate) context: u32,
    pub(crate) slot: u32,
    pub(crate) block_count: u32,
    pub(crate) manager: FirmwareVa,
    pub(crate) stamp: u32,
}
impl InitBufferManager {
    /// Check the output extent before mutation and preserve trailing storage.
    /// Capacity validation belongs to the owner of the block/page allocations.
    pub(crate) fn encode(&self, output: &mut [u8]) -> Result<(), Error> {
        let bytes = output.get_mut(..INIT_BM_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        for (offset, value) in [(init::OPCODE, 6), (init::CONTEXT, self.context),
            (init::SLOT, self.slot), (init::BLOCK_COUNT, self.block_count),
            (init::STAMP, self.stamp)] { u32_at(bytes, offset, value); }
        address_at(bytes, init::MANAGER, self.manager);
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(crate) enum Dependency {
    /// The fragment queue waits for this pass's tiler completion.
    TilerToFragment,
    /// A queue waits for an earlier pass; qualified G15 internal type is one.
    PreviousPass,
}
/// Completion of an earlier command whose stamp allocation remains owned by
/// the runtime until every queue in the publication has retired.
#[derive(Clone, Copy)]
pub(crate) struct CompletionDependency {
    pub(crate) stamp: FirmwareVa,
    pub(crate) value: u32,
    pub(crate) event: u32,
}
pub(crate) struct Barrier {
    pub(crate) stamp: FirmwareVa,
    pub(crate) wait_value: u32,
    pub(crate) event: u32,
    pub(crate) self_value: u32,
    pub(crate) uuid: u32,
    pub(crate) dependency: Dependency,
}
impl Barrier {
    /// Both stamp pointers refer to the same owned firmware stamp, as in all
    /// qualified render paths. No external barrier or arbitrary type is exposed.
    pub(crate) fn encode(&self, output: &mut [u8]) -> Result<(), Error> {
        let bytes = output.get_mut(..BARRIER_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        address_at(bytes, barrier::STAMP, self.stamp);
        address_at(bytes, barrier::STAMP2, self.stamp);
        let kind = match self.dependency { Dependency::TilerToFragment => 0, Dependency::PreviousPass => 1 };
        for (offset, value) in [(barrier::OPCODE, 4), (barrier::WAIT_VALUE, self.wait_value),
            (barrier::EVENT, self.event), (barrier::SELF_VALUE, self.self_value),
            (barrier::UUID, self.uuid), (barrier::INTERNAL_TYPE, kind)] {
            u32_at(bytes, offset, value);
        }
        Ok(())
    }
}

impl InitBufferManager {
    /// The extended rebuild path requires a zero staged-block count. Appending
    /// already-staged blocks would leave a previous hardware descriptor live.
    pub(crate) fn encode_extended(&self, output: &mut [u8]) -> Result<(), Error> {
        if self.context >= 64 || self.slot >= 32 { return Err(Error::Address); }
        Self { context: self.context, slot: self.slot, block_count: 0,
            manager: self.manager, stamp: self.stamp }.encode(output)
    }
}

impl Barrier {
    pub(crate) const EXTENDED_SIZE: usize = 0x40;

    pub(crate) fn encode_extended(&self, output: &mut [u8]) -> Result<(), Error> {
        if self.event >= 128 || self.wait_value == 0 || self.self_value == 0 {
            return Err(Error::Address);
        }
        let bytes = output.get_mut(..Self::EXTENDED_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        u32_at(bytes, 0, 4);
        address_at(bytes, 4, self.stamp);
        address_at(bytes, 12, self.stamp);
        u32_at(bytes, 0x14, self.wait_value);
        u32_at(bytes, 0x20, self.event);
        u32_at(bytes, 0x24, self.self_value);
        if matches!(self.dependency, Dependency::PreviousPass) { u32_at(bytes, 0x3c, 1); }
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    #[test] fn full_rebuild_never_uses_the_legacy_staged_count() {
        let mut bytes = [0xa5; INIT_BM_SIZE];
        InitBufferManager { context: 3, slot: 7, block_count: 2048,
            manager: FirmwareVa::new(0xffff_fc24_0010_0000).unwrap(), stamp: 0x100 }
            .encode_extended(&mut bytes).unwrap();
        assert_eq!(&bytes[12..20], &[0; 8]);
    }
    #[test] fn extended_dependencies_use_their_native_tail_and_preserve_trailing_storage() {
        for previous in [false, true] {
            let mut bytes = [0xa5; Barrier::EXTENDED_SIZE + 8];
            Barrier { stamp: FirmwareVa::new(0xffff_fc24_0010_0000).unwrap(), wait_value: 0x100,
                event: 2, self_value: 0x200, uuid: 9,
                dependency: if previous { Dependency::PreviousPass } else { Dependency::TilerToFragment } }
                .encode_extended(&mut bytes).unwrap();
            assert_eq!(&bytes[0x18..0x20], &[0; 8]);
            assert_eq!(&bytes[0x20..0x24], &[2, 0, 0, 0]);
            assert_eq!(u32::from_le_bytes(bytes[0x3c..0x40].try_into().unwrap()), previous.into());
            assert_eq!(&bytes[Barrier::EXTENDED_SIZE..], &[0xa5; 8]);
        }
    }
}
