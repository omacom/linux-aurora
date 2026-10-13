// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The 20-word block at HwDataA +0x418c..+0x41db that the runtime backend writes on a T8122
//! with the G15 14.8.3 firmware, unless `asahi.t8122_hwdata_418c=0`. Plain data only, so that
//! the host tests run it (tools/asahi).
//!
//! The words were measured from the InitData that macOS 26 writes on a J613 (10 GPU cores),
//! read under the m1n1 hypervisor. They were identical across six boots, and they are not in
//! the device tree. What they mean is not known, and they have been measured on that one
//! machine only. In a reported 14.8.3 boot without them, the firmware left the range at zero.
//!
//! The range is covered by the G13-era leakage-coefficient fields of the HwDataA structure
//! (`core_leak_coef[4..8]`, `sram_leak_coef` and `aux_leak_coef`), which the G15 builder
//! leaves at zero. The measured words are small integers, not floats, so those names probably
//! do not describe this range on G15.
//!
//! Without the block the GPU start of an M3 MacBook Air on the 14.8.3 firmware stalled at the
//! first device control (0x13 timed out) in every reported boot; with it, a 10-core and an
//! 8-core J613 started (reported in omacom/linux-aurora#26).

/// Offset of the block in HwDataA.
pub(crate) const OFFSET: usize = 0x418c;

/// The words, in order. Each is stored as a little-endian u32.
pub(crate) const WORDS: [u32; 20] = [
    1, 500, 5, //
    12500, 12500, 12500, 12500, 12500, //
    3000, 3000, 3000, //
    12500, 12500, 12500, 12500, 12500, 12500, 12500, 12500, //
    1,
];

/// Size of the block in bytes.
pub(crate) const SIZE: usize = WORDS.len() * 4;

/// `asahi.t8122_hwdata_418c` values: 0 leaves the range as the builder made it (zero), 1 writes
/// the block. The default is 1.
pub(crate) const OFF: u64 = 0;
pub(crate) const ON: u64 = 1;
pub(crate) const DEFAULT: u64 = ON;

const _: () = assert!(SIZE == 0x50 && OFFSET % 4 == 0);

/// Parse `asahi.t8122_hwdata_418c`: `0` or `1`. Anything else is refused.
pub(crate) fn parse(text: &str) -> Option<u64> {
    match text.trim() {
        "0" => Some(OFF),
        "1" => Some(ON),
        _ => None,
    }
}

/// Whether a parameter value asks for the block.
pub(crate) fn enabled(value: u64) -> bool {
    value == ON
}

/// Write the block into `hwdata_a`, the bytes of HwDataA. Nothing outside
/// `OFFSET..OFFSET + SIZE` changes. Err, with nothing written, when `hwdata_a` is too short.
pub(crate) fn write(hwdata_a: &mut [u8]) -> Result<(), ()> {
    let block = hwdata_a.get_mut(OFFSET..OFFSET + SIZE).ok_or(())?;
    for (bytes, word) in block.chunks_exact_mut(4).zip(WORDS) {
        bytes.copy_from_slice(&word.to_le_bytes());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The size of the G15 14.8.3 HwDataA structure.
    const HWDATA_A_SIZE: usize = 0x4360;

    #[test]
    fn the_words_are_the_measured_ones() {
        let mut expected = [12500u32; 20];
        expected[0] = 1;
        expected[1] = 500;
        expected[2] = 5;
        expected[8..11].fill(3000);
        expected[19] = 1;
        assert_eq!(WORDS, expected);
        assert_eq!(WORDS[3], 0x30d4);
        assert_eq!(WORDS[8], 0xbb8);
        assert_eq!(WORDS[1], 0x1f4);
    }

    #[test]
    fn the_block_lands_at_418c_to_41db_and_nothing_else_changes() {
        let fill = |i: usize| (i as u8).wrapping_mul(7) ^ 0xa5;
        let mut a: Vec<u8> = (0..HWDATA_A_SIZE).map(fill).collect();
        write(&mut a).unwrap();
        for (i, byte) in a.iter().enumerate() {
            if !(0x418c..0x41dc).contains(&i) {
                assert_eq!(*byte, fill(i), "byte {i:#x} changed");
            }
        }
        let words: Vec<u32> = a[0x418c..0x41dc]
            .chunks_exact(4)
            .map(|w| u32::from_le_bytes(w.try_into().unwrap()))
            .collect();
        assert_eq!(words, WORDS);
        // Little-endian: the first word is 01 00 00 00, the second f4 01 00 00.
        assert_eq!(a[0x418c..0x4194], [1, 0, 0, 0, 0xf4, 1, 0, 0]);
        assert_eq!(a[0x41d8..0x41dc], [1, 0, 0, 0]);
    }

    #[test]
    fn a_short_buffer_is_refused_and_left_alone() {
        let mut a = vec![0u8; OFFSET + SIZE - 1];
        assert_eq!(write(&mut a), Err(()));
        assert!(a.iter().all(|b| *b == 0));
        let mut exact = vec![0u8; OFFSET + SIZE];
        assert_eq!(write(&mut exact), Ok(()));
    }

    #[test]
    fn the_parameter_accepts_0_and_1_only_and_defaults_to_1() {
        assert_eq!(parse("0"), Some(OFF));
        assert_eq!(parse("1"), Some(ON));
        assert_eq!(parse(" 1\n"), Some(ON));
        for text in ["", "2", "-1", "+1", "0x1", "on", "yes", "01"] {
            assert_eq!(parse(text), None, "{text:?}");
        }
        assert!(enabled(DEFAULT));
        assert!(!enabled(OFF));
    }
}
