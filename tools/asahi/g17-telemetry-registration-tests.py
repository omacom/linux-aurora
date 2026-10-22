#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Check actual probe registration branches against a shared sysfs namespace.

The namespace and optional-registration failures are host boundary stubs, not
kernel execution. A negative control restores the duplicate placeholder probe.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2] / "drivers/gpu/drm/asahi"


def fixture():
    gpu = (ROOT / "g17/mod.rs").read_text()
    probe = (ROOT / "driver.rs").read_text().split("    fn probe_g17", 1)[1]
    telemetry = gpu[gpu.index("        let telemetry ="):gpu.index("        let active_cores =")]
    platform = probe[probe.index("        let raw_stats ="):probe.index("        Ok(AsahiPlatformDriverData")]
    names = [re.search(r'attr\.attr\.name = "([^"]+)"', (ROOT / name).read_text()).group(1)
             for name in ["sysfs.c", "asahi_telemetry.c"]]
    assert names == ["agx_stats", "agx_stats"], names
    return r'''
#![allow(dead_code, non_upper_case_globals)]
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering::SeqCst};
type Result<T> = std::result::Result<T, i32>;
static EXPORT: AtomicU8 = AtomicU8::new(1);
static PRESENT: AtomicBool = AtomicBool::new(false);
static FAIL: AtomicBool = AtomicBool::new(false);
static STOPPED: AtomicBool = AtomicBool::new(false);
static DUPLICATE: AtomicBool = AtomicBool::new(false);
macro_rules! dev_warn { ($dev:expr, $($t:tt)*) => {{ let _ = $dev; let _ = format_args!($($t)*); }}; }
mod module_parameters {
    pub struct Parameter;
    impl Parameter {
        pub fn value(&self) -> Box<u8> { Box::new(super::EXPORT.load(super::SeqCst)) }
    }
    pub static stats_export: Parameter = Parameter;
}
struct Device;
impl AsRef<Device> for Device { fn as_ref(&self) -> &Self { self } }
impl Device { fn as_raw(&self) -> *mut () { std::ptr::dangling_mut() } }
struct Gpu;
impl Gpu { fn shutdown(&self) { STOPPED.store(true, SeqCst); } }
fn create() -> Result<*mut ()> {
    if FAIL.load(SeqCst) { return Err(-12); }
    if PRESENT.swap(true, SeqCst) {
        DUPLICATE.store(true, SeqCst);
        return Err(-17);
    }
    Ok(std::ptr::dangling_mut())
}
fn remove() { assert!(PRESENT.swap(false, SeqCst)); }
mod telemetry {
    use super::*;
    pub struct Telemetry;
    impl Telemetry {
        pub fn new(_: &Device, _: u32) -> Result<Self> { create()?; Ok(Self) }
    }
    impl Drop for Telemetry { fn drop(&mut self) { remove(); } }
}
mod sysfs_exports {
    use super::*;
    pub fn register(_: *mut (), snapshot: *const (), enabled: bool) -> Result<*mut ()> {
        assert!(snapshot.is_null() && !enabled);
        create()
    }
}
fn gpu_registration(pdev: &Device) -> Option<telemetry::Telemetry> {
    let statistics = 0u32;
TELEMETRY
    telemetry
}
fn platform_registration(pdev: &Device, gpu: &Gpu) -> Result<*mut ()> {
    let raw_dev = pdev.as_raw();
PLATFORM
    Ok(raw_stats)
}
fn main() {
    for export in [1, 0, 2, 0] {
        for fail in [false, true] {
            EXPORT.store(export, SeqCst); FAIL.store(fail, SeqCst);
            STOPPED.store(false, SeqCst);
            let telemetry = gpu_registration(&Device);
            let platform = platform_registration(&Device, &Gpu);
            assert!(!DUPLICATE.load(SeqCst), "both probe owners created agx_stats");
            if export != 0 {
                assert_eq!(telemetry.is_some(), !fail);
                assert!(platform.unwrap().is_null());
                assert!(!STOPPED.load(SeqCst), "optional telemetry stopped GPU probe");
            } else if fail {
                assert_eq!(platform, Err(-12));
                assert!(STOPPED.load(SeqCst));
            } else {
                assert!(telemetry.is_none());
                assert!(!platform.unwrap().is_null());
                remove();
            }
            drop(telemetry);
            assert!(!PRESENT.load(SeqCst), "registration outlived its owner");
        }
    }
    println!("PASS enabled/disabled probe ownership, allocation failures and repeated teardown");
}
'''.replace("TELEMETRY", telemetry).replace("PLATFORM", platform)


def main():
    code = fixture()
    with tempfile.TemporaryDirectory(prefix="g17-telemetry-registration-") as temporary:
        root = Path(temporary)
        for mutant in [False, True]:
            source = root / f"registration-{mutant}.rs"
            source.write_text(code.replace(
                "let raw_stats = if *module_parameters::stats_export.value() == 0",
                "let raw_stats = if true") if mutant else code)
            binary = source.with_suffix("")
            subprocess.run([os.environ.get("RUSTC", "rustc"), "--edition=2021",
                            str(source), "-o", str(binary)], check=True)
            result = subprocess.run([str(binary)], text=True, capture_output=True)
            if mutant:
                assert result.returncode != 0 and "both probe owners" in result.stderr
                print("PASS negative control rejects duplicate agx_stats probe registration")
            else:
                assert result.returncode == 0, result.stderr
                print(result.stdout.strip())


if __name__ == "__main__":
    main()
