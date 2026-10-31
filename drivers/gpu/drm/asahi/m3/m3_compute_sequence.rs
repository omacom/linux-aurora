// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::{m3_compute_layout as command, m3_init_layout::{Error, Region}, m3_queue_layout::FirmwareVa};
pub(crate) const START: usize = 0;
pub(crate) const TIMESTAMP_START: usize = 0x1bc;
pub(crate) const WAIT: usize = 0x1f8;
pub(crate) const TIMESTAMP_END: usize = 0x1fc;
pub(crate) const FINALIZE: usize = 0x238;
pub(crate) const END: usize = 0x2b4;
pub(crate) const LENGTH: usize = 0x2b8;
pub(crate) const STORAGE: usize = 0x1000;
pub(crate) const GENERATION: usize = START + 0x34;
pub(crate) const ATTACHMENTS: usize = START + 0x54;
pub(crate) const HAS_ATTACHMENTS: usize = FINALIZE + 0x64;
pub(crate) const STAMP_VALUE: usize = FINALIZE + 0x30;
pub(crate) const USER_TIMESTAMP_POINTER: usize = 0x24;
const COMPUTE_INFO2: usize = 0x76c;
const TAIL_POOL: usize = command::TAIL + 0x2e;
const TAIL_STORE_REQUEST: usize = command::TAIL + 0x50;
const TAIL_STORE_COMPLETE: usize = command::TAIL + 0x58;
const TAIL_FLAG2: usize = command::TAIL + 0x70;
const STATS_OFFSET: usize = 0x3740; // retained HardwareData.compute statistics
fn word(out: &mut [u8], off: usize, value: u32) { out[off..off+4].copy_from_slice(&value.to_le_bytes()); }
fn pointer(out: &mut [u8], off: usize, value: u64) { out[off..off+8].copy_from_slice(&value.to_le_bytes()); }
pub(crate) struct Sequence {
    pub(crate) command: Region,
    pub(crate) stats: Region,
    pub(crate) notifier: Region,
    pub(crate) queue: FirmwareVa,
    pub(crate) context: u32,
    pub(crate) generation: u32,
    pub(crate) pool: FirmwareVa,
    /// Five distinct firmware-owned scratch buffers. They are not low client GPU pointers.
    pub(crate) scratch: [FirmwareVa; 5],
    pub(crate) fw_stamp: FirmwareVa,
    pub(crate) stamp_value: u32,
}
impl Sequence {
    /// All references and output extents are checked before output mutation.
    /// Fill the entire owned 4-KiB sequence slot, including unused zero tail.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        if out.len() < STORAGE { return Err(Error::Size); }
        if self.context >= 64 { return Err(Error::Address); }
        let registers = self.command.at(command::REGISTERS, command::REGISTER_COUNT * command::REGISTER_STRIDE)?;
        let compute_info = self.command.at(COMPUTE_INFO2, 0x60)?;
        let command_time = self.command.at(command::TAIL, 8)?;
        let timestamps = self.command.at(command::TAIL + 8, 16)?;
        let end_timestamp = self.command.at(command::TAIL + 16, 8)?;
        let pool_field = self.command.at(TAIL_POOL, 8)?;
        let store_request = self.command.at(TAIL_STORE_REQUEST, 8)?;
        let store_complete = self.command.at(TAIL_STORE_COMPLETE, 8)?;
        let flag = self.command.at(TAIL_FLAG2, 4)?;
        let notifier_slots = self.notifier.at(0xa8, 8)?;
        let stats = self.stats.at(STATS_OFFSET, 8)?;
        let bytes = &mut out[..STORAGE];bytes.fill(0);
        word(bytes, START, 0x0b);
        for (offset, value) in [(0x14,registers),(0x1c,stats),(0x24,self.queue.get()),
            (0x44,compute_info),(0x15c,self.pool.get()),(0x174,self.scratch[0].get()),
            (0x184,self.scratch[1].get()),(0x18c,self.scratch[2].get()),
            (0x194,self.scratch[3].get()),(0x19c,self.scratch[4].get()),
            (0x1a4,flag),(0x1b4,notifier_slots)] { pointer(bytes, START+offset, value); }
        word(bytes, START+0x2c, self.context);
        word(bytes, START+0x30, 1); // retained unk_28
        word(bytes, GENERATION, self.generation);
        word(bytes, START+0x50, 0xc10000); // UUID
        for (start, target, selected) in [(TIMESTAMP_START,0x80000003,timestamps),(TIMESTAMP_END,3,end_timestamp)] {
            word(bytes, start, target);
            for (offset, value) in [(4,command_time),(0xc,timestamps),(0x14,selected),
                (0x1c,self.queue.get()),(0x2c,store_complete)] { pointer(bytes,start+offset,value); }
            word(bytes,start+0x34,0xc10000);
        }
        word(bytes, WAIT, 1); // WaitForIdle2; G15 opcode
        word(bytes, FINALIZE, 0x0c);
        for (offset, value) in [(4,stats),(0xc,self.queue.get()),(0x18,compute_info),
            (0x28,self.fw_stamp.get()),(0x58,pool_field),(0x65,flag),(0x6d,store_request)] {
            pointer(bytes,FINALIZE+offset,value);
        }
        word(bytes,FINALIZE+0x14,self.context);
        word(bytes,FINALIZE+0x24,0xc10000);
        word(bytes,STAMP_VALUE,self.stamp_value);
        word(bytes,FINALIZE+0x60,(START as i32-FINALIZE as i32) as u32);
        word(bytes,END,0x40000002); // End with qualified completion flag
        Ok(())
    }
}

impl Sequence {
    pub(crate) const EXTENDED_LENGTH: usize = 0x300;
    pub(crate) const EXTENDED_ATTACHMENTS: usize = 0x58;
    pub(crate) const EXTENDED_HAS_ATTACHMENTS: usize = 0x2c0;

    /// The extended firmware operates on a native G15 CDM stream. Its firmware
    /// microsequence has wider timestamp operations and different engine state.
    pub(crate) fn encode_extended(&self, auxiliary: command::GpuVa,
        context_store: super::m3_pool_layout::GpuRegion, queue_sequence: u64,
        output: &mut [u8]) -> Result<(), Error> {
        if self.context >= 64 || self.generation != 0 { return Err(Error::Address); }
        if context_store.size() < 0x20000 { return Err(Error::Size); }
        let registers = self.command.at(command::REGISTERS, 24 * command::REGISTER_STRIDE)?;
        let engine = self.command.at(0x76c, 0x60)?;
        let clock = self.command.at(0x82c, 8)?;
        let pairs = self.command.at(0x834, 16)?;
        let end_pair = self.command.at(0x83c, 8)?;
        let pool_pointer = self.command.at(0x86a, 8)?;
        let status = self.command.at(0x8ac, 4)?;
        let time_aux = self.command.at(0x89c, 8)?;
        let elapsed = self.command.at(0x7dc, 8)?;
        let tail = self.command.at(0x88c, 8)?;
        let activity = self.stats.at(0, 8)?;
        let bytes = output.get_mut(..STORAGE).ok_or(Error::Size)?;
        bytes.fill(0);
        word(bytes, 0, 0x0b);
        pointer(bytes, 0x14, registers);
        pointer(bytes, 0x1c, activity);
        pointer(bytes, 0x24, self.queue.get());
        word(bytes, 0x2c, self.context);
        word(bytes, 0x30, 1);
        pointer(bytes, 0x38, queue_sequence);
        pointer(bytes, 0x44, engine);
        word(bytes, 0x50, 1);
        pointer(bytes, 0x160, self.pool.get());
        pointer(bytes, 0x178, auxiliary.get());
        for (offset, delta) in [0, 0xf400, 0x1e800, 0x1f000].into_iter().enumerate() {
            pointer(bytes, 0x188 + offset * 8, context_store.base() + delta);
        }
        pointer(bytes, 0x1a8, status);
        word(bytes, 0x1b8, 2);
        // Timestamp records are 76 bytes; WFI sits between the two records.
        for (offset, opcode, selected) in [(0x1bc, 0x8000_0003, pairs), (0x20c, 3, end_pair)] {
            word(bytes, offset, opcode);
            pointer(bytes, offset + 4, clock);
            pointer(bytes, offset + 0xc, pairs);
            pointer(bytes, offset + 0x14, selected);
            pointer(bytes, offset + 0x1c, self.queue.get());
            pointer(bytes, offset + 0x2c, time_aux);
            pointer(bytes, offset + 0x34, elapsed);
            pointer(bytes, offset + 0x44, 0xc10000);
        }
        word(bytes, 0x208, 1);
        let finish = 0x258;
        word(bytes, finish, 0x0c);
        pointer(bytes, finish + 4, activity);
        pointer(bytes, finish + 0xc, self.queue.get());
        word(bytes, finish + 0x14, self.context);
        pointer(bytes, finish + 0x18, engine);
        pointer(bytes, finish + 0x24, 0xc10000);
        pointer(bytes, finish + 0x2c, self.fw_stamp.get());
        word(bytes, finish + 0x34, self.stamp_value);
        pointer(bytes, finish + 0x5c, pool_pointer);
        word(bytes, finish + 0x64, (-(finish as i32)) as u32);
        pointer(bytes, finish + 0x69, status);
        pointer(bytes, finish + 0x71, tail);
        word(bytes, 0x2d4, 0x4000_0002);
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    fn fixture() -> Sequence {
        let fw = |i: u64| FirmwareVa::new(0xffff_fc21_a000_0000 + i * 0x4000).unwrap();
        Sequence { command: Region::new(fw(0).get(), command::EXTENDED_SIZE).unwrap(),
            stats: Region::new(fw(1).get(), 0xe10).unwrap(),
            notifier: Region::new(fw(2).get(), 0x200).unwrap(), queue: fw(3),
            context: 3, generation: 0, pool: fw(4), scratch: [fw(5); 5],
            fw_stamp: fw(6), stamp_value: 0x700 }
    }
    fn ptr(bytes: &[u8], at: usize) -> u64 { u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap()) }
    #[test] fn extended_timestamps_and_retirement_refer_to_their_live_owners() {
        let s = fixture();
        let mut output = [0xa5; STORAGE + 8];
        s.encode_extended(command::GpuVa::new(0x5000_0000).unwrap(),
            super::super::m3_pool_layout::GpuRegion::new(0x6000_0000, 0x20000).unwrap(), 7, &mut output).unwrap();
        assert_eq!(ptr(&output, 0x1c), s.stats.at(0, 8).unwrap());
        assert_eq!(ptr(&output, 0x38), 7);
        assert_eq!(ptr(&output, 0x188), 0x6000_0000);
        assert_eq!(ptr(&output, 0x1bc + 0x14), s.command.at(0x834, 8).unwrap());
        assert_eq!(ptr(&output, 0x20c + 0x14), s.command.at(0x83c, 8).unwrap());
        assert_eq!(ptr(&output, 0x258 + 0x2c), s.fw_stamp.get());
        assert_eq!(ptr(&output, 0x258 + 0x5c), s.command.at(0x86a, 8).unwrap());
        assert_eq!(&output[STORAGE..], &[0xa5; 8]);
    }
    #[test] fn extended_sequence_rejects_legacy_command_extents_and_event_generations() {
        for generation in [0, 1] {
            let mut s = fixture();
            s.generation = generation;
            if generation == 0 { s.command = Region::new(s.command.at(0, 1).unwrap(), command::SIZE).unwrap(); }
            let mut output = [0xa5; STORAGE];
            assert!(s.encode_extended(command::GpuVa::new(0x5000_0000).unwrap(),
                super::super::m3_pool_layout::GpuRegion::new(0x6000_0000, 0x20000).unwrap(), 0, &mut output).is_err());
            assert_eq!(output, [0xa5; STORAGE]);
        }
    }
}
