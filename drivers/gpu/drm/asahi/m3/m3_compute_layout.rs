// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::{m3_pool_layout::GpuRegion, m3_queue_layout::{Error, FirmwareVa}};
pub(crate) const SIZE: usize = 0x893;
pub(crate) const EXTENDED_SIZE: usize = 0x8b0;
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
/// The register lists a GPU's firmware expects in its commands: G15S's, or G15G's, which drop
/// some G15S registers and keep the order, entry layout and placement of the rest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RegisterSet {
    G15S,
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
        self.encode_for(super::m3_profile::Profile::Legacy, 0, 0, out)
    }

    /// Firmware envelopes vary independently of the GPU's CDM stream format.
    pub(crate) fn encode_for(&self, profile: super::m3_profile::Profile,
        asid_generation: u8, event_slot: u8, out: &mut [u8]) -> Result<(), Error> {
        let extended = profile == super::m3_profile::Profile::Extended;
        let size = if extended { EXTENDED_SIZE } else { SIZE };
        if out.len() < size { return Err(Error::Size); }
        if extended && (self.register_set != RegisterSet::G15G || event_slot >= 4
            || (self.context != 0 && asid_generation == 0)) { return Err(Error::Address); }
        if self.context >= 64 || self.cdm_last.get() < self.cdm.get() { return Err(Error::Address); }
        if self.preemption.size() < 0x14a0 || self.microsequence_size == 0 || self.microsequence_size > 4096 {
            return Err(Error::Size);
        }
        self.gpu_alias.add(((size - 1) & !3) as u64)?; // inclusive final aligned dword
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
        let bytes = &mut out[..size];bytes.fill(0);
        word(bytes, 0, 3); // compute opcode
        if extended {
            word(bytes, 0xc, self.context);
            pointer(bytes, 0x10, self.notifier.get());
        } else {
            pointer(bytes, COUNTER, u64::from(self.counter));
            word(bytes, CONTEXT, self.context);
            pointer(bytes, 0x14, self.notifier.get());
        }
        let legacy_count = match self.register_set {
            RegisterSet::G15S => REGISTER_COUNT,
            RegisterSet::G15G => G15G_REGISTER_COUNT,
        };
        let mut count = 0;
        let mut emit = |register, value| {
            word(bytes, REGISTERS + count * REGISTER_STRIDE, register);
            pointer(bytes, REGISTERS + count * REGISTER_STRIDE + 4, value);
            count += 1;
        };
        let limit = if extended { REGISTER_COUNT } else { legacy_count };
        for (register, value) in values.into_iter().take(limit) {
            if extended && register == 0x12091 { continue; }
            emit(register, value);
        }
        if extended {
            emit(0x1a540, 1);
            emit(0x014a9, 1);
            emit(0x0a351, 1);
        }
        pointer(bytes, 0x720, registers.get());
        bytes[0x728..0x72a].copy_from_slice(&(count as u16).to_le_bytes());
        bytes[0x72a..0x72c].copy_from_slice(&((count*REGISTER_STRIDE) as u16).to_le_bytes());
        pointer(bytes, 0x760, self.microsequence.get());
        word(bytes, 0x768, self.microsequence_size);
        let info = if extended { 0x770 } else { 0x76c };
        pointer(bytes, info + 0x28, preemption);
        pointer(bytes, info + 0x30, self.cdm_last.get());
        pointer(bytes, info + 0x58, QUALIFIED_COMPUTE_CONTROL);
        word(bytes, if extended { 0x7e8 } else { 0x7e0 }, u32::MAX);
        let meta = if extended { 0x800 } else { META };
        pointer(bytes, meta, self.stamp.get());
        pointer(bytes, meta + 8, self.fw_stamp.get());
        word(bytes, meta + 0x10, self.stamp_value);
        word(bytes, meta + 0x14, 2);
        if extended { word(bytes, meta + 0x18, event_slot.into()); }
        word(bytes, meta + 0x1c, u32::from(self.flush_stamps));
        word(bytes, meta + 0x20, 0xc10000);
        word(bytes, meta + 0x24, self.counter);
        let tail = if extended { 0x82c } else { TAIL };
        pointer(bytes, tail + 8, self.timestamps[0].get());
        pointer(bytes, tail + 16, self.timestamps[1].get());
        pointer(bytes, tail + if extended { 0x3e } else { 0x2e }, self.pool.get());
        bytes[tail + if extended { 0x46 } else { 0x36 }] = 1;
        pointer(bytes, tail + if extended { 0x47 } else { 0x37 },
            if extended { 0x1588000 } else { 0xad0000 });
        pointer(bytes, tail + if extended { 0x4f } else { 0x3f },
            if extended { 0xef8000 } else { 0x1600000 });
        pointer(bytes, tail + if extended { 0x57 } else { 0x47 }, self.tail_scratch.get());
        if extended { bytes[0x88b] = asid_generation; }
        else { bytes[TAIL + 0x4f] = 1; }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::m3_profile::Profile;

    fn fixture() -> Command {
        let firmware = |n: u64| FirmwareVa::new(0xffff_fc21_8000_0000 + n * 0x4000).unwrap();
        Command {
            context: 3, counter: 7, notifier: firmware(0),
            preemption: GpuRegion::new(0x5000_0000, 0x4000).unwrap(),
            cdm: GpuVa::new(0x6000_0000).unwrap(),
            cdm_last: GpuVa::new(0x6000_0ffc).unwrap(),
            gpu_alias: GpuVa::new(0x7000_0000).unwrap(),
            microsequence: firmware(1), microsequence_size: 0x300,
            stamp: firmware(2), fw_stamp: firmware(3), stamp_value: 0x700,
            timestamps: [firmware(4), firmware(5)], pool: firmware(6),
            tail_scratch: firmware(7), flush_stamps: true,
            register_set: RegisterSet::G15G,
        }
    }
    fn read32(b: &[u8], o: usize) -> u32 {
        u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
    }
    fn read64(b: &[u8], o: usize) -> u64 {
        u64::from_le_bytes(b[o..o + 8].try_into().unwrap())
    }

    #[test]
    fn extended_envelope_preserves_stream_and_native_visibility_boundary() {
        let c = fixture();
        let mut b = [0xa5; EXTENDED_SIZE + 16];
        c.encode_for(Profile::Extended, 9, 2, &mut b).unwrap();
        assert_eq!(read32(&b, 0xc), c.context);
        assert_eq!(read64(&b, 0x10), c.notifier.get());
        assert_eq!(read32(&b, 0x728) & 0xffff, 24);
        let count = (read32(&b, 0x728) & 0xffff) as usize;
        let cdm = (0..count).find(|n| read32(&b, 0x20 + n * 12) == 0x1a420).unwrap();
        assert_eq!(read64(&b, 0x24 + cdm * 12), c.cdm.get());
        assert_eq!(read64(&b, 0x7a0), c.cdm_last.get());
        assert_eq!(read64(&b, 0x800), c.stamp.get());
        assert_eq!(read64(&b, 0x808), c.fw_stamp.get());
        assert_eq!(read32(&b, 0x818), 2);
        assert_eq!(read32(&b, 0x81c), 1);
        assert_eq!(read64(&b, 0x834), c.timestamps[0].get());
        assert_eq!(read64(&b, 0x86a), c.pool.get());
        assert_eq!(read64(&b, 0x883), c.tail_scratch.get());
        assert_eq!(b[0x88b], 9);
        assert_eq!(&b[EXTENDED_SIZE..], &[0xa5; 16]);
    }

    #[test]
    fn extended_validation_never_mutates_a_rejected_envelope() {
        for (generation, slot) in [(0, 0), (1, 4), (1, 255)] {
            let mut b = [0xa5; EXTENDED_SIZE];
            assert!(fixture().encode_for(Profile::Extended, generation, slot, &mut b).is_err());
            assert_eq!(b, [0xa5; EXTENDED_SIZE]);
        }
        let mut b = [0xa5; EXTENDED_SIZE - 1];
        assert!(fixture().encode_for(Profile::Extended, 1, 0, &mut b).is_err());
        assert_eq!(b, [0xa5; EXTENDED_SIZE - 1]);
    }

    #[test]
    fn legacy_wrapper_keeps_legacy_geometry_and_register_order() {
        let c = fixture();
        let mut b = [0xa5; EXTENDED_SIZE];
        c.encode(&mut b).unwrap();
        assert_eq!(read32(&b, CONTEXT), c.context);
        assert_eq!(read64(&b, 0x14), c.notifier.get());
        assert_eq!(read32(&b, 0x728) & 0xffff, 20);
        assert_eq!(read32(&b, REGISTERS + 8 * REGISTER_STRIDE), 0x12091);
        assert_eq!(read64(&b, META), c.stamp.get());
        assert_eq!(&b[SIZE..], &[0xa5; EXTENDED_SIZE - SIZE]);
    }
}
