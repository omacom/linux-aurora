# SPDX-License-Identifier: GPL-2.0-only
"""Host regressions for xART location and write ownership."""

import contextlib
import importlib.util
import io
import shutil
import subprocess
from pathlib import Path
import struct
import tempfile
import unittest
import zlib


SCRIPT = Path(__file__).resolve().parents[1] / "apfs-xart-inspect.py"
SPEC = importlib.util.spec_from_file_location("apfs_xart_inspect", SCRIPT)
INSPECTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSPECTOR)


def record(kind, revision):
    out = bytearray(0x9000)
    payload = b"test"
    out[1] = kind
    struct.pack_into("<I", out, 0x12, len(payload))
    struct.pack_into("<I", out, 0x16, zlib.crc32(payload))
    struct.pack_into("<Q", out, 0x1a, revision)
    out[0x22:0x26] = payload
    return out


def extent_reference(start, blocks, owner, refcount=1, kind=1):
    key = struct.pack("<Q", (2 << 60) | start)
    value = struct.pack("<QQi", (kind << 60) | blocks, owner, refcount)
    return key, value


class ExtentReferenceTests(unittest.TestCase):
    def test_unshared_whole_file_reference(self):
        expected = (1236, 1536, 1, 16, 1)
        self.assertEqual(
            INSPECTOR.validate_extent_references(
                [extent_reference(1236, 1536, 16)], 1236, 1536, 16),
            expected)

    def test_unrelated_volume_extents_do_not_share_the_gigalocker(self):
        entries = [extent_reference(1236, 1536, 16),
                   extent_reference(3000, 1, 17),
                   extent_reference(4000, 1, 18)]
        self.assertEqual(
            INSPECTOR.validate_extent_references(entries, 1236, 1536, 16),
            (1236, 1536, 1, 16, 1))

    def test_shared_or_mismatched_reference_fails_closed(self):
        cases = [
            [],
            [extent_reference(1236, 1536, 16, refcount=2)],
            [extent_reference(1236, 1536, 17)],
            [extent_reference(1236, 1536, 16, kind=2)],
            [extent_reference(1237, 1535, 16)],
            [extent_reference(1235, 1537, 16)],
            [extent_reference(1236, 1536, 16)] * 2,
        ]
        for entries in cases:
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                INSPECTOR.validate_extent_references(entries, 1236, 1536, 16)


class RawLocatorTests(unittest.TestCase):
    def test_gigalocker_filename_forms(self):
        self.assertTrue(INSPECTOR.is_gigalocker_name(b".gl"))
        self.assertTrue(INSPECTOR.is_gigalocker_name(
            b"01234567-89AB-5CDE-8F01-23456789ABCD.gl"))
        self.assertTrue(INSPECTOR.is_gigalocker_name(
            b"01234567-89ab-5cde-8f01-23456789abcd.gl"))
        self.assertFalse(INSPECTOR.is_gigalocker_name(b"01234567.gl"))
        self.assertFalse(INSPECTOR.is_gigalocker_name(
            b"01234567-89AB-5CDE-8F01-23456789ABCG.gl"))
        self.assertFalse(INSPECTOR.is_gigalocker_name(b"other.gl"))

    def test_roots_moved_forward_pick_window_outside_real_extent(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "synthetic-container"
            with path.open("wb") as stream:
                stream.truncate(12 * 1024 * 1024)
            true_block = 100
            true_base = true_block * 4096
            with path.open("r+b") as stream:
                for slot, kind in ((10, 1), (20, 2), (21, 4)):
                    stream.seek(true_base + slot * 0x9000)
                    stream.write(record(kind, slot))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                INSPECTOR.inspect_raw_locator(INSPECTOR.Disk(path), true_block)
            self.assertIn("shift_blocks=90", output.getvalue())


@unittest.skipUnless(shutil.which("rustc"), "rustc required for driver ownership tests")
class DriverOwnershipTests(unittest.TestCase):
    def test_actual_driver_write_ownership(self):
        # Compile the actual parser with in-memory I/O, not a second parser.
        driver = SCRIPT.parents[2] / "drivers/soc/apple/xart_apfs.rs"
        harness = r'''
extern crate self as kernel;
pub mod prelude {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Error(pub i32);
    pub const EINVAL: Error = Error(22);
    pub const EIO: Error = Error(5);
    pub const ENODATA: Error = Error(61);
    pub const ENOTSUPP: Error = Error(524);
    pub const GFP_KERNEL: u32 = 0;
    pub type Result<T> = std::result::Result<T, Error>;
    pub struct KVec<T>(Vec<T>);
    impl<T: Clone> KVec<T> {
        pub fn new() -> Self {
            Self(Vec::new())
        }
        pub fn resize(&mut self, n: usize, v: T, _: u32) -> Result<()> {
            self.0.resize(n, v);
            Ok(())
        }
    }
    impl<T> std::ops::Deref for KVec<T> {
        type Target = [T];
        fn deref(&self) -> &[T] {
            &self.0
        }
    }
    impl<T> std::ops::DerefMut for KVec<T> {
        fn deref_mut(&mut self) -> &mut [T] {
            &mut self.0
        }
    }
}
pub mod shim {
    use crate::prelude::*;
    pub struct StoreFile {
        pub data: Vec<u8>,
    }
    impl StoreFile {
        pub fn size(&self) -> Result<u64> {
            Ok(self.data.len() as u64)
        }
        pub fn read_block_exact(&self, off: u64, dst: &mut [u8]) -> Result<()> {
            let a = usize::try_from(off).map_err(|_| EINVAL)?;
            let end = a.checked_add(dst.len()).ok_or(EINVAL)?;
            dst.copy_from_slice(self.data.get(a..end).ok_or(EIO)?);
            Ok(())
        }
    }
}

#[path = "xart_apfs.rs"]
mod xart_apfs;
'''
        cases = r'''
#[cfg(test)]
mod ownership_tests {
    use super::*;
    const PHYS: u64 = 128;
    const OWNER: u64 = 42;
    const SIZE: u64 = 4096 * 4096;
    #[derive(Clone, Copy)]
    struct Record {
        start: u64,
        blocks: u64,
        owner: u64,
        refs: u32,
        key_type: u64,
        kind: u64,
    }
    fn record(start: u64, blocks: u64, owner: u64, refs: u32) -> Record {
        Record {
            start,
            blocks,
            owner,
            refs,
            key_type: 2,
            kind: 1,
        }
    }
    fn target() -> Record {
        record(PHYS, FILE_SIZE / BLOCK as u64, OWNER, 1)
    }
    fn p16(b: &mut [u8], o: usize, n: u16) {
        b[o..o + 2].copy_from_slice(&n.to_le_bytes());
    }
    fn p32(b: &mut [u8], o: usize, n: u32) {
        b[o..o + 4].copy_from_slice(&n.to_le_bytes());
    }
    fn p64(b: &mut [u8], o: usize, n: u64) {
        b[o..o + 8].copy_from_slice(&n.to_le_bytes());
    }
    fn fixture(records: &[Record]) -> (shim::StoreFile, Vec<u8>) {
        let mut v = vec![0; BLOCK];
        p64(&mut v, 264, 1);
        p64(&mut v, 144, 2);
        let mut t = vec![0; BLOCK];
        p64(&mut t, 8, 2);
        p64(&mut t, 16, 9);
        p32(&mut t, 24, 0x40000002);
        p32(&mut t, 28, 15);
        p16(&mut t, 32, 3);
        p32(&mut t, 36, records.len() as u32);
        p16(&mut t, 42, (records.len() * 8) as u16);
        let ks = 56 + records.len() * 8;
        for (i, r) in records.iter().enumerate() {
            let item = 56 + i * 8;
            let ko = i * 8;
            let vo = (i + 1) * 24;
            let vs = BLOCK - 40 - vo;
            p16(&mut t, item, ko as u16);
            p16(&mut t, item + 2, 8);
            p16(&mut t, item + 4, vo as u16);
            p16(&mut t, item + 6, 20);
            p64(&mut t, ks + ko, (r.key_type << 60) | r.start);
            p64(&mut t, vs, (r.kind << 60) | r.blocks);
            p64(&mut t, vs + 8, r.owner);
            p32(&mut t, vs + 16, r.refs);
        }
        let sum = checksum(&t).unwrap();
        p64(&mut t, 0, sum);
        let mut data = vec![0; SIZE as usize];
        data[2 * BLOCK..3 * BLOCK].copy_from_slice(&t);
        (shim::StoreFile { data }, v)
    }
    fn check(records: &[Record]) -> Result<()> {
        let (f, v) = fixture(records);
        validate_write_ownership(&f, SIZE, &v, 10, OWNER, 0, PHYS)
    }
    #[test]
    fn original_single_record_passes() {
        assert_eq!(check(&[target()]), Ok(()));
    }
    #[test]
    fn three_disjoint_records_pass() {
        assert_eq!(
            check(&[target(), record(2000, 1, 43, 1), record(2100, 1, 44, 1)]),
            Ok(())
        );
    }
    #[test]
    fn target_in_last_position_passes() {
        assert_eq!(
            check(&[record(10, 1, 43, 1), record(20, 1, 44, 1), target()]),
            Ok(())
        );
    }
    #[test]
    fn target_in_middle_passes() {
        assert_eq!(
            check(&[record(10, 1, 43, 1), target(), record(2100, 1, 44, 1)]),
            Ok(())
        );
    }
    #[test]
    fn shared_target_rejected() {
        let mut t = target();
        t.refs = 2;
        assert!(check(&[t]).is_err());
    }
    #[test]
    fn wrong_owner_rejected() {
        let mut t = target();
        t.owner = 43;
        assert!(check(&[t]).is_err());
    }
    #[test]
    fn wrong_target_length_rejected() {
        let mut t = target();
        t.blocks -= 1;
        assert!(check(&[t]).is_err());
    }
    #[test]
    fn missing_target_rejected() {
        assert!(check(&[record(2000, 1, 43, 1)]).is_err());
    }
    #[test]
    fn duplicate_target_rejected() {
        assert!(check(&[target(), target()]).is_err());
    }
    #[test]
    fn overlapping_other_record_rejected() {
        assert!(check(&[target(), record(200, 1, 43, 1)]).is_err());
    }
    #[test]
    fn encompassing_record_rejected() {
        assert!(check(&[record(100, 1600, OWNER, 1)]).is_err());
    }
    #[test]
    fn overlapping_unrelated_records_rejected() {
        assert!(check(&[target(), record(2000, 2, 43, 1), record(2001, 1, 44, 1)]).is_err());
    }
    #[test]
    fn unsorted_records_rejected() {
        assert!(check(&[record(2000, 1, 43, 1), target()]).is_err());
    }
    #[test]
    fn zero_extent_length_rejected() {
        assert!(check(&[target(), record(2000, 0, 43, 1)]).is_err());
    }
    #[test]
    fn negative_reference_count_rejected() {
        assert!(check(&[target(), record(2000, 1, 43, u32::MAX)]).is_err());
    }
    #[test]
    fn zero_reference_count_rejected() {
        assert!(check(&[target(), record(2000, 1, 43, 0)]).is_err());
    }
    #[test]
    fn out_of_container_extent_rejected() {
        assert!(check(&[target(), record(4095, 2, 43, 1)]).is_err());
    }
    #[test]
    fn unknown_record_type_rejected() {
        let mut t = target();
        t.key_type = 3;
        assert!(check(&[t]).is_err());
    }
    #[test]
    fn unknown_extent_kind_rejected() {
        let mut t = target();
        t.kind = 2;
        assert!(check(&[t]).is_err());
    }
    #[test]
    fn empty_tree_rejected() {
        assert!(check(&[]).is_err());
    }
    #[test]
    fn snapshot_still_rejected() {
        let (f, mut v) = fixture(&[target()]);
        p64(&mut v, 216, 1);
        assert!(validate_write_ownership(&f, SIZE, &v, 10, OWNER, 0, PHYS).is_err());
    }
    #[test]
    fn pending_revert_still_rejected() {
        let (f, mut v) = fixture(&[target()]);
        p64(&mut v, 160, 1);
        assert!(validate_write_ownership(&f, SIZE, &v, 10, OWNER, 0, PHYS).is_err());
    }
    #[test]
    fn cloned_inode_still_rejected() {
        let (f, v) = fixture(&[target()]);
        for flags in [INODE_WAS_CLONED, INODE_WAS_EVER_CLONED] {
            assert!(validate_write_ownership(&f, SIZE, &v, 10, OWNER, flags, PHYS).is_err());
        }
    }
    #[test]
    fn damaged_metadata_checksum_rejected() {
        let (mut f, v) = fixture(&[target()]);
        f.data[2 * BLOCK + 100] ^= 1;
        assert!(validate_write_ownership(&f, SIZE, &v, 10, OWNER, 0, PHYS).is_err());
    }
}
'''
        with tempfile.TemporaryDirectory(prefix="sep-xart-ownership-") as directory:
            root = Path(directory)
            source, binary = root / "harness.rs", root / "ownership-tests"
            (root / "xart_apfs.rs").write_text(driver.read_text() + cases)
            source.write_text(harness)
            subprocess.run(["rustc", "--edition=2021", "--test", "-Adead_code",
                            str(source), "-o", str(binary)], check=True, timeout=60)
            subprocess.run([str(binary), "--test-threads=1"], check=True, timeout=60)


if __name__ == "__main__":
    unittest.main()
