#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile the T6040 wire builders and calibration checks without a GPU."""

from pathlib import Path
import re
import subprocess
import tempfile

DRIVER = Path(__file__).resolve().parents[2] / "drivers/gpu/drm/asahi"



HEADER = r'''
#![allow(dead_code)]
extern crate self as kernel;
mod prelude {
    pub type KVec<T> = Vec<T>;
    pub use std::mem::size_of;
    pub type Result<T = ()> = std::result::Result<T, Error>;
    #[derive(Debug)]
    pub struct Error;
    impl From<std::num::TryFromIntError> for Error {
        fn from(_: std::num::TryFromIntError) -> Self { Self }
    }
    pub const EINVAL: Error = Error;
    pub const EIO: Error = Error;
    pub const EOVERFLOW: Error = Error;
    pub unsafe trait Zeroable {}
}
macro_rules! static_assert { ($e:expr) => { const _: () = assert!($e); } }
fn clear<T>(value: &mut T) {
    unsafe { std::ptr::write_bytes(value, 0, 1); }
}
#[macro_export]
macro_rules! c_str {
    ($s:literal) => { std::ffi::CStr::from_bytes_with_nul(concat!($s, "\0").as_bytes()).unwrap() };
}
pub mod str { pub use std::ffi::CStr; }
mod of {
    use crate::prelude::*;
    #[derive(Clone)]
    pub struct Node(pub std::collections::BTreeMap<String, Vec<u32>>);
    pub trait Property: Sized { fn decode(value: &[u32]) -> Result<Self>; }
    impl Property for u32 {
        fn decode(value: &[u32]) -> Result<Self> {
            if value.len() == 1 { Ok(value[0]) } else { Err(EINVAL) }
        }
    }
    impl Property for Vec<u32> { fn decode(value: &[u32]) -> Result<Self> { Ok(value.to_vec()) } }
    impl Node {
        pub fn get_property<T: Property>(&self, name: &crate::str::CStr) -> Result<T> {
            T::decode(self.0.get(name.to_str().unwrap()).ok_or(EINVAL)?)
        }
    }
}
mod driver {
    pub struct AsahiDrmDevice(pub DrmDevice);
    pub struct DrmDevice(pub Device);
    pub struct Device(pub crate::of::Node);
    impl AsRef<DrmDevice> for AsahiDrmDevice { fn as_ref(&self) -> &DrmDevice { &self.0 } }
    impl AsRef<Device> for DrmDevice { fn as_ref(&self) -> &Device { &self.0 } }
    impl Device { pub fn of_node(&self) -> Option<crate::of::Node> { Some(self.0.clone()) } }
}
mod float;
mod initdata;
mod g17 {
    pub mod fw { pub(crate) use crate::initdata; }
    mod calibration;
    pub fn check_property_adapter() {
        use crate::{driver::*, of::Node};
        let fields: &[(&str, &[u32])] = &[
            ("apple,t6040-handoff-version", &[2]),
            ("apple,core-mask", &[0xfffff]),
            ("apple,max-power-mw", &[70000]),
            ("apple,perf-base-pstate", &[1]),
            ("apple,perf-map-a", &[0,1,2,3,4,5,6,7,8,9,15,0,0,0,0,0]),
            ("apple,perf-map-b", &[0,1,2,3,4,5,6,7,8,9,15,0,0,0,0,0]),
            ("apple,perf-relative-a", &[0,10,20,30,40,50,60,70,80,90,100,0,0,0,0,0]),
            ("apple,perf-relative-b", &[0,0,11,22,33,44,55,66,77,88,100,0,0,0,0,0]),
            ("apple,core-leak-coef", &[0x44480000, 0x44800000]),
            ("apple,sram-leak-coef", &[0x43000000, 0x43000000]),
            ("apple,cs-leak-coef", &[0x43800000]),
            ("apple,afr-leak-coef", &[0x44000000]),
        ];
        let node = Node(fields.iter().map(|(k,v)| (k.to_string(), v.to_vec())).collect());
        let read = |n| calibration::Calibration::read(&AsahiDrmDevice(DrmDevice(Device(n))));
        let chip = read(node.clone()).unwrap();
        assert_eq!(chip.max_power_mw, 70000);
        assert_eq!(chip.leakage[0], 0x44480000);
        assert_eq!(chip.leakage[1], 0x44800000);
        assert_eq!(chip.leakage[32..34], [0x43000000; 2]);
        assert_eq!(chip.leakage[52], 0x43800000);
        assert_eq!(chip.leakage[54], 0x44000000);
        for (key, _) in fields {
            let mut missing = node.clone(); missing.0.remove(*key);
            assert!(read(missing).is_err(), "accepted missing {key}");
            let mut short = node.clone(); short.0.get_mut(*key).unwrap().pop();
            assert!(read(short).is_err(), "accepted short {key}");
        }
        for key in ["apple,perf-map-a", "apple,perf-map-b", "apple,perf-relative-a", "apple,perf-relative-b"] {
            for index in 11..16 {
                let mut bad = node.clone(); bad.0.get_mut(key).unwrap()[index] = 1;
                assert!(read(bad).is_err(), "accepted nonzero tail {key}[{index}]");
            }
        }
        for (key, value) in [
            ("apple,t6040-handoff-version", 1),
            ("apple,core-mask", 0x100000),
            ("apple,max-power-mw", 0),
            ("apple,perf-base-pstate", 0),
            ("apple,perf-base-pstate", 2),
            ("apple,cs-leak-coef", 0x7fc00000),
            ("apple,afr-leak-coef", 0xbf800000),
        ] {
            let mut bad = node.clone(); bad.0.insert(key.to_string(), vec![value]);
            assert!(read(bad).is_err(), "accepted invalid {key}");
        }
    }
}
'''

TEST = r'''
#[test]
fn property_adapter_requires_complete_chip_data() { g17::check_property_adapter(); }

#[test]
fn power_configuration_uses_only_its_inputs() {
    use initdata::v25g83::PowerConfig25G83;
    let mut first = PowerConfig25G83([0xcc; 0x6510]);
    let mut leakage = [0; 56];
    leakage[0] = 0x44480000;
    leakage[1] = 0x44800000;
    leakage[32] = 0x43000000;
    leakage[33] = 0x43000000;
    leakage[52] = 0x43800000;
    leakage[54] = 0x44000000;
    first.init(0x12340000, 0x56780000, 0xabcdef, 76543, &leakage);
    let word = |bytes: &[u8], off| u32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
    for off in [0xa04, 0xabc, 0x28c8, 0x28cc, 0x28d0, 0x2a08] {
        assert_eq!(word(&first.0, off), 76543);
    }
    for off in [0x9e0, 0x2a04] {
        assert_eq!(word(&first.0, off), (76543f32).to_bits());
    }
    for (i, value) in leakage.iter().enumerate() {
        assert_eq!(word(&first.0, 0x642c + 4 * i), *value);
    }
    assert_eq!(u64::from_le_bytes(first.0[0x3b50..0x3b58].try_into().unwrap()), 0x12340080);
    assert_eq!(u64::from_le_bytes(first.0[0x6318..0x6320].try_into().unwrap()), 0x56780000);
    assert_eq!(word(&first.0, 0x63d4), 0xabcdef);
    let mut second = PowerConfig25G83([0; 0x6510]);
    second.init(0x12340000, 0x56780000, 0xabcdef, 76543, &leakage);
    assert_eq!(first.0, second.0);
    second.init(0x12340000, 0x56780000, 0xabcdef, 80000, &[0; 56]);
    assert_eq!(word(&second.0, 0xa04), 80000);
    assert_eq!(word(&second.0, 0x9e0), (80000f32).to_bits());
    for i in 0..56 {
        assert_eq!(word(&second.0, 0x642c + 4 * i), 0);
    }
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="asahi-t6040-") as tmp:
        root = Path(tmp)
        (root / "initdata").mkdir()
        (root / "g17").mkdir()
        for source, target in [
            ("g17/fw/initdata.rs", "initdata.rs"),
            ("g17/fw/initdata/v25g83.rs", "initdata/v25g83.rs"),
            ("g17/fw/initdata/v26b123.rs", "initdata/v26b123.rs"),
        ]:
            text = (DRIVER / source).read_text()
            (root / target).write_text(text.replace("use kernel::prelude::*;", "use crate::prelude::*;"))
        text = (DRIVER / "float.rs").read_text().split("impl<'a> TryFrom<of::Property")[0]
        text = re.sub(r"use kernel::\{.*?\};", "use crate::prelude::*;", text, flags=re.S)
        (root / "float.rs").write_text(text)
        text = (DRIVER / "g17/calibration.rs").read_text()
        text = text.replace("use kernel::{c_str,", "use kernel::{")
        text = text.replace("#[kunit_tests(asahi_t6040_calibration)]", "#[cfg(test)]")
        (root / "g17/calibration.rs").write_text(text)
        (root / "main.rs").write_text(HEADER + TEST)
        binary = root / "test"
        subprocess.run(["rustc", "--edition=2021", "--test", "-Dwarnings",
                        str(root / "main.rs"), "-o", str(binary)], check=True)
        subprocess.run([str(binary), "--test-threads=1"], check=True)


if __name__ == "__main__":
    main()
