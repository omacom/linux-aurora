#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production ALS calibration message builder: the 80-byte message and the 193-byte sensor blob."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def source():
    aop = (ROOT / 'drivers/soc/apple/aop.rs').read_text()
    consts = '\n'.join(line for line in aop.splitlines() if line.startswith('const ALS_'))
    readers = aop[aop.index('fn le_u32('):aop.index('/// The calibration file as the message')]
    start = aop.index('fn als_calibration_message(')
    builder = aop[start:aop.index('\n}\n', start) + 3]
    return consts + '\n' + readers + builder + TESTS


TESTS = r'''
fn op_message(len: usize) -> Vec<u8> {
    let mut m = vec![0x5au8; len];
    m[..8].copy_from_slice(&0x0746_b8d6_6515_2e31u64.to_le_bytes());
    m[8..16].copy_from_slice(&56u64.to_le_bytes());
    m
}

#[test]
fn t8140_message_passes_through() {
    let m = op_message(80);
    let (out, len) = als_calibration_message(&m).unwrap();
    assert_eq!(len, 80);
    assert_eq!(&out[..80], &m[..]);
}

#[test]
fn t8140_message_with_wrong_op_or_body_length_is_refused() {
    let mut m = op_message(80);
    m[0] ^= 1;
    assert!(als_calibration_message(&m).is_none());
    let mut m = op_message(80);
    m[8] = 57;
    assert!(als_calibration_message(&m).is_none());
}

#[test]
fn sensor_blob_is_framed_in_216_bytes() {
    let blob: Vec<u8> = (0..193u32).map(|i| (i as u8) | 1).collect();
    let (out, len) = als_calibration_message(&blob).unwrap();
    assert_eq!(len, 216);
    assert_eq!(le_u64(&out, 0), 0x0746_b8d6_6515_2e31);
    assert_eq!(le_u64(&out, 8), 193);
    assert_eq!(&out[16..209], &blob[..]);
    assert!(out[209..216].iter().all(|b| *b == 0));
}

#[test]
fn other_sizes_are_refused() {
    for n in [0usize, 1, 56, 79, 81, 192, 194, 216, 4096] {
        let m = vec![0u8; n];
        assert!(als_calibration_message(&m).is_none(), "{n}");
    }
}
'''


def main():
    code = source()
    mutations = {
        'blob-length-field': ('(ALS_SENSOR_BLOB_LEN as u64).to_le_bytes()', '56u64.to_le_bytes()'),
        'blob-offset': ('message[16..16 + ALS_SENSOR_BLOB_LEN]', 'message[15..15 + ALS_SENSOR_BLOB_LEN]'),
        'frame-length': ('Some((message, ALS_SENSOR_MESSAGE_LEN))', 'Some((message, ALS_SENSOR_BLOB_LEN + 16))'),
        'op-unchecked': ('le_u64(data, 0) == ALS_CALIBRATION_OPERATION', 'true'),
        'body-unchecked': ('le_u64(data, 8) == ALS_CALIBRATION_BODY_LEN', 'true'),
        'blob-size-loose': ('data.len() == ALS_SENSOR_BLOB_LEN', 'data.len() >= ALS_SENSOR_BLOB_LEN'),
    }
    with tempfile.TemporaryDirectory(prefix='aop-als-cal-') as directory:
        directory = Path(directory)
        for name, text in [('production', code)] + [
                (name, code.replace(old, new)) for name, (old, new) in mutations.items()]:
            assert text != code or name == 'production', name
            path = directory / 'cal.rs'
            path.write_text(text)
            binary = directory / 'cal'
            subprocess.run(['rustc', '--edition=2021', '--test', '-Adead_code', str(path), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], text=True, capture_output=True)
            if name == 'production':
                print(result.stdout)
                assert result.returncode == 0, result.stdout + result.stderr
            else:
                assert result.returncode == 101 and 'panicked at' in result.stdout + result.stderr, (name, result)
                print('killed:', name)


if __name__ == '__main__':
    main()
