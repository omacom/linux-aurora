// SPDX-License-Identifier: GPL-2.0-only OR MIT
use crate::{m3_pool_layout::GpuRegion, m3_queue_layout::{Error, FirmwareVa}};
pub(crate) const SIZE: usize = 0x893;
pub(crate) const REGISTERS: usize = 0x20;
pub(crate) const REGISTER_COUNT: usize = 22;
/// G15G takes the same list without its last two registers (0xa599, 0xd411).
pub(crate) const G15G_REGISTER_COUNT: usize = 20;
pub(crate) const REGISTER_STRIDE: usize = 12;
pub(crate) const CONTEXT: usize = 0x10;
pub(crate) const COUNTER: usize = 4;
pub(crate) const META: usize = 0x7e8;
pub(crate) const STAMP_VALUE: usize = META + 0x10;
pub(crate) const FLUSH_STAMPS: usize = META + 0x1c;
pub(crate) const QUEUE_COUNT: usize = META + 0x24;
pub(crate) const TAIL: usize = 0x810;
pub(crate) const USER_TIMESTAMPS: usize = TAIL + 0x18;
const QUALIFIED_COMPUTE_CONTROL: u64 = 0x154024201;
fn word(out: &mut [u8], off: usize, value: u32) { out[off..off+4].copy_from_slice(&value.to_le_bytes()); }
fn pointer(out: &mut [u8], off: usize, value: u64) { out[off..off+8].copy_from_slice(&value.to_le_bytes()); }
/// A four-byte-aligned byte address in the G15 lower 42-bit GPU root. This
/// accepts subobjects (including packed register data), unlike page owners.
/// The register lists a GPU's firmware expects in its commands: G15G's, which drop some of the
/// G15S registers and keep the order, entry layout and placement of the rest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RegisterSet {
    G15G,
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct GpuVa(u64);
impl GpuVa {
    pub(crate) fn new(address: u64) -> Result<Self, Error> {
        if address == 0 || address & 3 != 0 || address >= 1 << 42 { return Err(Error::Address); }
        Ok(Self(address))
    }
    pub(crate) fn get(self) -> u64 { self.0 }
    fn add(self, delta: u64) -> Result<Self, Error> {
        Self::new(self.0.checked_add(delta).ok_or(Error::Address)?)
    }
}
pub(crate) struct Command {
    pub(crate) context: u32,
    pub(crate) counter: u32,
    pub(crate) notifier: FirmwareVa,
    pub(crate) preemption: GpuRegion,
    pub(crate) cdm: GpuVa,
    /// Inclusive last dword of the client CDM stream, not an end pointer.
    pub(crate) cdm_last: GpuVa,
    /// The command's lower GPU alias; firmware's canonical VA is distinct.
    pub(crate) gpu_alias: GpuVa,
    pub(crate) microsequence: FirmwareVa,
    pub(crate) microsequence_size: u32,
    pub(crate) stamp: FirmwareVa,
    pub(crate) fw_stamp: FirmwareVa,
    pub(crate) stamp_value: u32,
    pub(crate) timestamps: [FirmwareVa; 2],
    pub(crate) pool: FirmwareVa,
    pub(crate) tail_scratch: FirmwareVa,
    /// Old fixture is zero; prepare_completion requests one before dispatch.
    pub(crate) flush_stamps: bool,
    pub(crate) register_set: RegisterSet,
}
impl Command {
    /// Validate before any output mutation. Encoding operates on ordinary host
    /// bytes; the caller copies them into the owned unpublished command buffer.
    pub(crate) fn encode(&self, out: &mut [u8]) -> Result<(), Error> {
        if out.len() < SIZE { return Err(Error::Size); }
        if self.context >= 64 || self.cdm_last.get() < self.cdm.get() { return Err(Error::Address); }
        if self.preemption.size() < 0x14a0 || self.microsequence_size == 0 || self.microsequence_size > 4096 {
            return Err(Error::Size);
        }
        self.gpu_alias.add((SIZE - 3) as u64)?; // inclusive final aligned dword
        let registers = self.gpu_alias.add(REGISTERS as u64)?;
        let preemption = self.preemption.base();
        let values = [
            (0x1a510, preemption), (0x1a420, self.cdm.get()),
            (0x1a4d0, preemption+0x1480), (0x1a4d8, preemption+0x1488),
            (0x1a4e0, preemption+0x1490), (0x1a4e8, preemption+0x1498),
            (0x1a440, QUALIFIED_COMPUTE_CONTROL), (0x1a458, 0x10c08860),
            (0x12091, 0x1c), (0x101d9, 0x1c),
            (0x1a089, 0), (0x1a091, 0), (0x1a059, 0), (0x1a061, 0),
            (0x1a0b9, 0), (0x1a0c1, 0), (0x101d1, 0), (0xd479, 0),
            (0x1a0e9, 8), (0x107a1, 0xff0000), (0xa599, 0x13200400020), (0xd411, 0x200000001),
        ];
        let bytes = &mut out[..SIZE];bytes.fill(0);
        word(bytes, 0, 3); // compute opcode
        pointer(bytes, COUNTER, u64::from(self.counter));
        word(bytes, CONTEXT, self.context);
        pointer(bytes, 0x14, self.notifier.get());
        let count = match self.register_set {
            RegisterSet::G15G => G15G_REGISTER_COUNT,
        };
        for (n, (register, value)) in values.into_iter().take(count).enumerate() {
            word(bytes, REGISTERS+n*REGISTER_STRIDE, register);
            pointer(bytes, REGISTERS+n*REGISTER_STRIDE+4, value);
        }
        pointer(bytes, 0x720, registers.get());
        bytes[0x728..0x72a].copy_from_slice(&(count as u16).to_le_bytes());
        bytes[0x72a..0x72c].copy_from_slice(&((count*REGISTER_STRIDE) as u16).to_le_bytes());
        pointer(bytes, 0x760, self.microsequence.get());
        word(bytes, 0x768, self.microsequence_size);
        // ComputeInfo2 mirrors the preemption buffer, inclusive CDM end and
        // qualified control word from the register list.
        pointer(bytes, 0x76c+0x28, preemption);
        pointer(bytes, 0x76c+0x30, self.cdm_last.get());
        pointer(bytes, 0x76c+0x58, QUALIFIED_COMPUTE_CONTROL);
        word(bytes, 0x7cc+0x14, u32::MAX); // EncoderParams.unk_mask
        pointer(bytes, META, self.stamp.get());
        pointer(bytes, META+8, self.fw_stamp.get());
        word(bytes, STAMP_VALUE, self.stamp_value);
        word(bytes, META+0x14, 2); // qualified compute event slot
        word(bytes, FLUSH_STAMPS, u32::from(self.flush_stamps));
        word(bytes, META+0x20, 0xc10000); // producer UUID
        word(bytes, QUEUE_COUNT, self.counter);
        pointer(bytes, TAIL+8, self.timestamps[0].get());
        pointer(bytes, TAIL+16, self.timestamps[1].get());
        pointer(bytes, TAIL+0x2e, self.pool.get());
        bytes[TAIL+0x36] = 1; // SharedCommandTail.unk_846
        pointer(bytes, TAIL+0x37, 0xad0000);
        pointer(bytes, TAIL+0x3f, 0x1600000);
        pointer(bytes, TAIL+0x47, self.tail_scratch.get());
        bytes[TAIL+0x4f] = 1; // SharedCommandTail.unk_85f
        Ok(())
    }
}
