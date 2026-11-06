// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::m3_queue_layout::{Error, FirmwareVa};
pub(crate) const CONTEXT_SIZE: usize = 0x40;
pub(crate) const NOTIFIER_SIZE: usize = 0x108;
pub(crate) const NOTIFIER_CONTEXT: usize = 0x24;
pub(crate) const JOB_LIST_SIZE: usize = 0x18;
pub(crate) const MANAGER_STATE_SIZE: usize = 0x40;

fn word(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}
/// Qualified unresolved context flags; this is not M4's different 0x38 layout.
pub(crate) fn context(out: &mut [u8]) -> Result<(), Error> {
    let bytes = out.get_mut(..CONTEXT_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    bytes[0] = 0xff; // queue_table_index, unregistered
    bytes[1] = 0xff; // pid_table_index, unregistered
    bytes[5] = 1; // unk_5
    bytes[0x1e] = 0xff; // unk_1e
    bytes[0x23] = 2; // unk_23, retained producer value
    Ok(())
}
pub(crate) struct Notifier {
    pub(crate) threshold: FirmwareVa,
    pub(crate) context: u32,
}
impl Notifier {
    /// Preserve generation zero and the eight 0xff slot sentinels. The rest,
    /// including the firmware-written tail, starts zero. All validation precedes
    /// writes; any trailing caller storage remains untouched.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        let bytes = out.get_mut(..NOTIFIER_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        bytes[..8].copy_from_slice(&self.threshold.get().to_le_bytes());
        word(bytes, 0x10, 0x50); // EventControl.unk_10
        word(bytes, NOTIFIER_CONTEXT, self.context);
        bytes[0xa8..0xb0].fill(0xff); // EventControl.unk_buf
        Ok(())
    }
}
/// Empty circular list owned by the scheduling graph, before any job publishes.
pub(crate) fn job_list(out: &mut [u8], owner: FirmwareVa) -> Result<(), Error> {
    let bytes = out.get_mut(..JOB_LIST_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    bytes[8..16].copy_from_slice(&owner.get().to_le_bytes()); // last_head
    Ok(())
}
/// A scalar owner may have reserved trailing storage; initialize all of it.
/// Counts and stamps remain caller-named inputs, not anonymous byte templates.
pub(crate) fn counter(out: &mut [u8], value: u32) -> Result<(), Error> {
    if out.len() < 4 { return Err(Error::Size); }
    out.fill(0);
    word(out, 0, value);
    Ok(())
}
pub(crate) fn block_control(out: &mut [u8], total: u32, available: u32) -> Result<(), Error> {
    let bytes = out.get_mut(..MANAGER_STATE_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    word(bytes, 0, total);
    word(bytes, 4, available);
    Ok(())
}
pub(crate) fn manager_misc(out: &mut [u8]) -> Result<(), Error> {
    let bytes = out.get_mut(..MANAGER_STATE_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    word(bytes, 0x20, 1); // cpu_flag / reset
    Ok(())
}

pub(crate) const EXTENDED_CONTEXT_SIZE: usize = 0x38;
pub(crate) const EXTENDED_NOTIFIER_SIZE: usize = 0x200;

pub(crate) fn context_extended(output: &mut [u8]) -> Result<(), Error> {
    let bytes = output.get_mut(..EXTENDED_CONTEXT_SIZE).ok_or(Error::Size)?;
    bytes.fill(0);
    bytes[..2].fill(0xff);
    bytes[5] = 1;
    bytes[0x26] = 2;
    bytes[0x33] = 0xff;
    Ok(())
}

impl Notifier {
    pub(crate) fn encode_extended(&self, output: &mut [u8]) -> Result<(), Error> {
        if self.context >= 64 { return Err(Error::Address); }
        let bytes = output.get_mut(..EXTENDED_NOTIFIER_SIZE).ok_or(Error::Size)?;
        bytes.fill(0);
        bytes[..8].copy_from_slice(&self.threshold.get().to_le_bytes());
        word(bytes, 0x10, 0x50);
        word(bytes, NOTIFIER_CONTEXT, self.context);
        bytes[0xa8..0xb0].fill(0xff);
        word(bytes, 0x114, 1);
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    #[test] fn registration_state_is_left_to_firmware() {
        let mut bytes = [0xa5; EXTENDED_CONTEXT_SIZE + 8];
        context_extended(&mut bytes).unwrap();
        assert_eq!(&bytes[..6], &[255, 255, 0, 0, 0, 1]);
        assert_eq!(bytes[0x26], 2);
        assert_eq!(bytes[0x33], 255);
        assert_eq!(&bytes[EXTENDED_CONTEXT_SIZE..], &[0xa5; 8]);
    }
    #[test] fn notifier_has_zero_generation_and_distinct_firmware_storage() {
        let notifier = Notifier { threshold: FirmwareVa::new(0xffff_fc21_7000_0000).unwrap(), context: 3 };
        let mut bytes = [0xa5; EXTENDED_NOTIFIER_SIZE + 8];
        notifier.encode_extended(&mut bytes).unwrap();
        assert_eq!(&bytes[8..12], &[0; 4]);
        assert_eq!(&bytes[0xa8..0xb0], &[255; 8]);
        assert_eq!(&bytes[0x114..0x118], &[1, 0, 0, 0]);
        assert_eq!(&bytes[EXTENDED_NOTIFIER_SIZE..], &[0xa5; 8]);
        let mut short = [0xa5; NOTIFIER_SIZE];
        assert!(notifier.encode_extended(&mut short).is_err());
        assert_eq!(short, [0xa5; NOTIFIER_SIZE]);
    }
}
