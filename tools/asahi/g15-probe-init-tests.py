#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the actual G15 probe with the vendored pin-init crate and proc macro.

Builds the actual vendored host proc macro without a kernel build or downloaded
crates. An existing kernel-build macro may optionally be supplied for comparison.
The G15 probe, platform callback's dispatch block, Device::set_drvdata and
try_pin_init macro are extracted from production. GPU startup and the allocator
are fault-injection models. KBox's actual source is checked to allocate before
write_pin_init; the host uses the same vendored write_pin_init implementation.
This checks allocation/init/binding order, not a GPU or a kernel probe on hardware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OLD_REF = "564f61b8189e"
PROBE_PATH = "drivers/gpu/drm/asahi/g15/mod.rs"
# Verbatim inherited eager probe before the lazy initializer fix.
OLD_PROBE = r'''    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let cfg = *info.ok_or(ENODEV)?;
        dev_info!(pdev.as_ref(), "G15: probing ({})\n", cfg.name);
        match crate::driver::AsahiPlatformDriver::probe_g15(pdev, cfg) {
            Ok(driver) => Ok(Driver { _driver: Some(driver) }),
            Err(error) if boot::glue::retained() => {
                dev_err!(
                    pdev.as_ref(),
                    "G15: probe failed after the firmware start ({:?}); device kept bound, firmware retained until reboot\n",
                    error
                );
                Ok(Driver { _driver: None })
            }
            Err(error) => Err(error),
        }
    }'''


def item(text, marker):
    start = text.index(marker)
    at = text.index("{", start)
    depth = 1
    end = at + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


PREFIX = r'''
#![allow(dead_code, unused_variables)]
extern crate self as kernel;
use std::{cell::Cell, marker::PhantomData, pin::Pin, sync::atomic::{AtomicBool, Ordering}};
use pin_init::{pin_data, PinInit, InPlaceWrite};
pub mod error { pub type Error = i32; }
use error::Error;
type Result<T = ()> = std::result::Result<T, Error>;
const ENODEV: i32 = -19;
const EPROBE_DEFER: i32 = -517;
const GFP_KERNEL: u32 = 0;
macro_rules! dev_info { ($($x:tt)*) => {{}}; }
macro_rules! dev_err { ($($x:tt)*) => {{}}; }
thread_local! {
    static FAIL_ALLOC: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u32> = const { Cell::new(0) };
    static PROBES: Cell<u32> = const { Cell::new(0) };
    static TOUCH: Cell<bool> = const { Cell::new(false) };
    static RETAINED: Cell<bool> = const { Cell::new(false) };
    static FAIL_REGISTRATION: Cell<u8> = const { Cell::new(0) };
    static COMPLETED_REGISTRATION: Cell<u8> = const { Cell::new(0) };
    static ACTIVATED: Cell<bool> = const { Cell::new(false) };
    static BINDING_READY: Cell<bool> = const { Cell::new(false) };
    static ATTACH_ERROR: Cell<bool> = const { Cell::new(false) };
    static PROBE_ERROR: Cell<i32> = const { Cell::new(0) };
    static PRE_ACTIVATION: Cell<u32> = const { Cell::new(0) };
}
struct Core<'a>(PhantomData<&'a ()>);
struct Device { data: Cell<*mut core::ffi::c_void> }
impl Device {
    fn as_raw(&self) -> *mut Device { self as *const Device as *mut Device }
    // SET_DRVDATA
}
mod bindings {
    pub unsafe fn dev_set_drvdata(dev: *mut crate::Device, data: *mut core::ffi::c_void) {
        unsafe { (*dev).data.set(data); }
    }
}
struct Owned<T>(Pin<Box<T>>);
impl<T> Owned<T> {
    fn into_foreign(self) -> *mut T {
        // Test ownership transfer mirrors the kernel's retained pinned KBox.
        unsafe { Box::into_raw(Pin::into_inner_unchecked(self.0)) }
    }
}
struct KBox;
impl KBox {
    fn pin_init<T>(init: impl PinInit<T, Error>, _flags: u32) -> Result<Owned<T>> {
        ALLOCS.with(|v| v.set(v.get() + 1));
        if FAIL_ALLOC.with(Cell::get) { return Err(-12); }
        // Kernel KBox::try_pin_init has this same ordering. Allocation is the only
        // mocked operation here; write_pin_init is the actual vendored implementation.
        let slot = Box::<T>::new_uninit();
        Ok(Owned(slot.write_pin_init(init)?))
    }
}
mod platform {
    use super::*;
    pub struct Device<Ctx> { pub base: super::Device, pub ctx: PhantomData<Ctx> }
    // The fixture runs serially on one thread; this only permits a static probe target.
    unsafe impl<Ctx> Sync for Device<Ctx> {}
    pub struct Adapter<T>(pub PhantomData<T>);
    impl<Ctx> AsRef<super::Device> for Device<Ctx> {
        fn as_ref(&self) -> &super::Device { &self.base }
    }
    pub trait Driver {
        type IdInfo;
        type Data<'bound>;
        fn probe<'bound>(
            pdev: &'bound Device<Core<'_>>,
            info: Option<&'bound Self::IdInfo>,
        ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound;
    }
}
struct Config { name: &'static str, chip_id: u32 }
macro_rules! c_str { ($s:literal) => { $s } }
// The J516M probe swaps in its own profile; this fixture's device is never a J516M.
mod hw { pub mod j516m { pub static CONFIG: crate::Config = crate::Config { name: "j516m", chip_id: 0x6034 }; } }
pub struct ThisModule;
pub trait ModuleMetadata { const NAME: &'static std::ffi::CStr; }
pub trait InPlaceModule: Sized {
    fn init(module: &'static ThisModule) -> impl PinInit<Self, Error>;
}
// Registry synchronization is covered by binding-lifetime-tests; this adapter
// verifies that the actual module initializer orders it before registration.
mod binding {
    pub unsafe fn init() { crate::BINDING_READY.set(true); }
}
mod cleanup {
    pub struct QueueOwner;
    impl QueueOwner { pub fn new() -> crate::Result<Self> { Ok(Self) } }
}
mod driver {
    use super::*;
    #[pin_data]
    pub struct Registration<T> { stage: u8, marker: PhantomData<T> }
    impl<T: 'static> Registration<T> {
        pub fn new(name: &'static std::ffi::CStr, _module: &'static ThisModule) -> impl PinInit<Self, Error> {
            assert!(BINDING_READY.get(), "adapter registered before binding registry init");
            try_pin_init!(Self {
                stage: {
                    let stage = if name.to_bytes() == b"asahi-g15" { 1 } else { 2 };
                    // Model driver_attach and a late driver_add_groups failure. A deferred
                    // worker can also retry G15 while the other registration is in progress.
                    let ready = MODULE_READY.load(Ordering::Acquire);
                    let started = PROBES.get();
                    let result = callback::<Driver<'static>>(&PDEV, Some(&&CFG));
                    // A probe that started firmware here is the readiness-gate failure, checked below.
                    if !ready && PROBES.get() == started {
                        // A deferral would keep the GPU in devices_deferred after a later
                        // terminal failure; before activation the probe must decline outright.
                        assert_eq!(result, Err(ENODEV), "pre-activation probe deferred instead of ENODEV");
                        PRE_ACTIVATION.set(PRE_ACTIVATION.get() + 1);
                    }
                    if FAIL_REGISTRATION.get() == stage { return Err(-5); }
                    COMPLETED_REGISTRATION.set(stage);
                    stage
                },
                marker: PhantomData,
            })
        }
    }
    pub struct AsahiPlatformDriverData<'bound> { value: u8, bound: PhantomData<&'bound ()> }
    pub struct AsahiPlatformDriver;
    impl AsahiPlatformDriver {
        pub fn probe_g15<'bound>(
            _pdev: &'bound platform::Device<Core<'_>>, _cfg: &'static Config,
        ) -> Result<AsahiPlatformDriverData<'bound>> {
            PROBES.with(|v| v.set(v.get() + 1));
            RETAINED.with(|v| v.set(TOUCH.with(Cell::get)));
            match PROBE_ERROR.with(Cell::get) {
                0 => Ok(AsahiPlatformDriverData { value: 1, bound: PhantomData }),
                e => Err(e),
            }
        }
    }
}
mod boot { pub mod glue {
    pub fn retained() -> bool { crate::RETAINED.with(core::cell::Cell::get) }
    pub fn machine(_compatible: &str) -> bool { false }
    pub unsafe fn asahi_g15_reprobe() { crate::activation_reprobe(); }
    // Reads the device tree and logs; no driver state.
    pub unsafe fn asahi_g15_report_loader_gpu() {}
}}
'''

SUFFIX = r'''
fn callback<'bound, T: platform::Driver>(
    pdev: &'bound platform::Device<Core<'_>>,
    info: Option<&'bound T::IdInfo>,
) -> Result<i32> {
    // CALLBACK
}
fn reset(fail_alloc: bool, touch: bool, error: i32) {
    MODULE_READY.store(true, Ordering::Release);
    BINDING_READY.set(false);
    FAIL_ALLOC.set(fail_alloc); TOUCH.set(touch); PROBE_ERROR.set(error);
    RETAINED.set(false); ALLOCS.set(0); PROBES.set(0);
}
static CFG: Config = Config { name: "test", chip_id: 0x6034 };
static PDEV: platform::Device<Core<'static>> = platform::Device {
    base: Device { data: Cell::new(core::ptr::null_mut()) }, ctx: PhantomData,
};
fn activation_reprobe() {
    assert_eq!(COMPLETED_REGISTRATION.get(), 2, "activation before all registrations completed");
    ACTIVATED.set(true);
    let _ = callback::<Driver<'static>>(&PDEV, Some(&&CFG));
    // An attach pass may fail after another device starts. Its error is logged, not returned
    // to module initialization. The actual helper's void signature is checked separately.
    if ATTACH_ERROR.get() { println!("attach failure after probe was not propagated"); }
}
fn module_cases() {
    static MODULE: ThisModule = ThisModule;
    for failure in [1, 2, 0] {
        reset(false, true, 0);
        PDEV.base.data.set(core::ptr::null_mut());
        MODULE_READY.store(false, Ordering::Release);
        COMPLETED_REGISTRATION.set(0); FAIL_REGISTRATION.set(failure);
        ACTIVATED.set(false); ATTACH_ERROR.set(failure == 0); PRE_ACTIVATION.set(0);
        let init = <DriverModule as InPlaceModule>::init(&MODULE);
        assert_eq!(PROBES.get(), 0, "module initializer eagerly started firmware");
        let result = KBox::pin_init(init, GFP_KERNEL);
        if failure != 0 {
            assert!(result.is_err(), "late registration failure must fail module init");
            assert_eq!(PROBES.get(), 0, "late registration failure started firmware");
            assert!(!MODULE_READY.load(Ordering::Acquire));
            assert!(!ACTIVATED.get());
        } else {
            assert!(result.is_ok(), "attach failure must not unwind a live session");
            assert_eq!(PROBES.get(), 1);
            assert!(MODULE_READY.load(Ordering::Acquire));
            assert!(ACTIVATED.get());
            assert!(!PDEV.base.data.get().is_null());
            unsafe { drop(Box::from_raw(PDEV.base.data.get().cast::<Driver<'_>>())); }
        }
        // Every probe before activation declined: one per completed or failing registration.
        assert_eq!(PRE_ACTIVATION.get(), if failure == 1 { 1 } else { 2 }, "pre-activation probes");
        println!("module-registration-{failure}: PASS");
    }
}
fn main() {
    let cfg = &CFG;
    for (name, allocation_fails, touched, error, expected, bound, calls) in [
        ("allocation-before-start", true, true, 0, Err(-12), false, 0),
        ("allocation-before-retained-failure", true, true, -5, Err(-12), false, 0),
        ("pre-start-failure", false, false, -19, Err(-19), false, 1),
        ("retained-start-failure-binds", false, true, -5, Ok(0), true, 1),
        ("retained-registration-failure-binds", false, true, -12, Ok(0), true, 1),
        ("successful-registration-binds", false, true, 0, Ok(0), true, 1),
    ] {
        reset(allocation_fails, touched, error);
        let pdev = platform::Device { base: Device { data: Cell::new(core::ptr::null_mut()) }, ctx: PhantomData };
        let result = callback::<Driver<'static>>(&pdev, Some(&cfg));
        assert_eq!(result, expected, "{name}: callback result");
        assert_eq!(PROBES.get(), calls, "{name}: firmware probe before drvdata allocation");
        assert_eq!(ALLOCS.get(), 1, "{name}: driver-data allocations");
        assert_eq!(!pdev.base.data.get().is_null(), bound, "{name}: bound data");
        if bound {
            // Fake kernel teardown after asserting installation. Production suppresses manual unbind.
            let data = unsafe { Box::from_raw(pdev.base.data.get().cast::<Driver<'_>>()) };
            assert_eq!(data._driver.is_some(), error == 0, "{name}: retained failure state");
        }
        println!("{name}: PASS");
    }
    module_cases();
}
'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--kernel-build", type=Path, help="Use this existing host proc macro instead of compiling vendored host dependencies")
    p.add_argument("--output", type=Path)
    p.add_argument("--mutations", action="store_true")
    p.add_argument("--rustc", default=os.environ.get("HOSTRUSTC", "rustc"))
    a = p.parse_args()
    out = (a.output or Path(tempfile.mkdtemp(prefix="g15-probe-init-"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if a.kernel_build is None:
        a.kernel_build = out / "host-deps"
        dependencies = a.kernel_build / "rust"
        dependencies.mkdir(parents=True, exist_ok=True)
        crates = [
            ("proc_macro2", "rust/proc-macro2/lib.rs", "2021", [],
             ['feature="proc-macro"', "wrap_proc_macro", "proc_macro_span_file", "proc_macro_span_location"]),
            ("quote", "rust/quote/lib.rs", "2018", ["proc_macro2"], ['feature="proc-macro"']),
            ("syn", "rust/syn/lib.rs", "2021", ["proc_macro2", "quote"],
             ['feature="' + feature + '"' for feature in ["clone-impls", "derive", "full", "parsing", "printing", "proc-macro", "visit", "visit-mut"]]),
            ("pin_init_internal", "rust/pin-init/internal/src/lib.rs", "2021", ["proc_macro2", "quote", "syn"],
             ["kernel", "USE_RUSTC_FEATURES"]),
        ]
        commands = []
        for name, path, edition, external, cfgs in crates:
            macro_crate = name == "pin_init_internal"
            output = dependencies / ("lib" + name + (".so" if macro_crate else ".rlib"))
            command = [a.rustc, "--edition=" + edition, "--crate-name=" + name,
                       "--crate-type=" + ("proc-macro" if macro_crate else "rlib"),
                       "--cap-lints=allow", str(ROOT / path), "-L", "dependency=" + str(dependencies), "-o", str(output)]
            if macro_crate:
                command += ["--extern", "proc_macro"]
            for dependency in external:
                command += ["--extern", dependency + "=" + str(dependencies / ("lib" + dependency + ".rlib"))]
            for cfg in cfgs:
                command += ["--cfg", cfg]
            commands.append(command)
            built = subprocess.run(command, capture_output=True, text=True)
            (out / (name + "-build.log")).write_text(built.stdout + built.stderr)
            if built.returncode:
                raise RuntimeError("vendored host dependency failed: " + name + "\n" + built.stderr)
        (out / "host-dependency-commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    sources = {name: (ROOT / name).read_text() for name in [PROBE_PATH, "rust/kernel/platform.rs", "rust/kernel/device.rs", "rust/kernel/init.rs", "rust/kernel/alloc/kbox.rs", "drivers/gpu/drm/asahi/asahi_core.rs", "drivers/gpu/drm/asahi/g15/glue.c"]}
    probe = item(sources[PROBE_PATH], "    fn probe<'bound>(")
    driver = item(sources[PROBE_PATH], "#[pin_data]\npub(crate) struct Driver<'bound>").replace("pub(crate) ", "")
    macro = item(sources["rust/kernel/init.rs"], "macro_rules! try_pin_init")
    set_data = item(sources["rust/kernel/device.rs"], "    pub fn set_drvdata<T>")
    callback = sources["rust/kernel/platform.rs"]
    callback = callback[callback.index("            let data = T::probe(pdev, info);"):]
    callback = callback[:callback.index("            Ok(0)") + len("            Ok(0)")]
    ready = "static MODULE_READY: AtomicBool = AtomicBool::new(false);"
    assert ready in sources[PROBE_PATH]
    activate = item(sources[PROBE_PATH], "pub(crate) fn activate()")
    core = sources["drivers/gpu/drm/asahi/asahi_core.rs"]
    module_struct = item(core, "#[pin_data]\nstruct DriverModule")
    module_init = item(core, "    fn init(module:")
    module_code = module_struct + "\nimpl InPlaceModule for DriverModule {\n" + module_init + "\n}\nimpl ModuleMetadata for DriverModule { const NAME: &'static std::ffi::CStr = c\"asahi\"; }\n"
    kbox = item(sources["rust/kernel/alloc/kbox.rs"], "    fn try_pin_init<E>(")
    assert "Box::<_, A>::new_uninit(flags)?.write_pin_init(init)" in kbox
    reprobe = item(sources["drivers/gpu/drm/asahi/g15/glue.c"], "void asahi_g15_reprobe(void)\n{")
    c_prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <errno.h>
struct device_driver { int unused; };
static struct device_driver driver;
static int platform_bus_type;
static bool present;
static int attach_result, finds, attaches, logs, started;
#define pr_err(...) ((void)++logs)
static const struct device_driver *driver_find(const char *name, const int *bus) {
    (void)name; assert(bus == &platform_bus_type); finds++;
    return present ? &driver : 0;
}
static int driver_attach(const struct device_driver *drv) {
    assert(drv == &driver); attaches++; started++; return attach_result;
}
'''
    c_suffix = r'''
int main(void) {
    for (int test = 0; test < 3; test++) {
        present = test != 0; attach_result = test == 2 ? -ENOMEM : 0;
        finds = attaches = logs = started = 0;
        asahi_g15_reprobe();
        assert(finds == 1);
        assert(attaches == (test != 0));
        assert(started == (test != 0));
        assert(logs == (test == 0 || test == 2));
    }
}
'''
    c_source = out / "actual-activation.c"
    c_source.write_text(c_prefix + reprobe + c_suffix)
    c_binary = out / "actual-activation"
    subprocess.run([os.environ.get("HOSTCC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror", str(c_source), "-o", str(c_binary)], check=True)
    subprocess.run([str(c_binary)], check=True)
    library = out / "libpin_init.rlib"
    subprocess.run([a.rustc, "--edition=2021", "--crate-type=rlib", "--crate-name=pin_init", "--cfg", 'feature="std"', str(ROOT / "rust/pin-init/src/lib.rs"), "--extern", f"pin_init_internal={a.kernel_build / 'rust/libpin_init_internal.so'}", "-L", f"dependency={a.kernel_build / 'rust'}", "-o", str(library)], check=True)
    receipt = {"rustc": subprocess.check_output([a.rustc, "--version"], text=True).strip(), "pin_init_source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT / "rust/pin-init").rglob("*.rs")}, "proc_macro_sha256": hashlib.sha256((a.kernel_build / "rust/libpin_init_internal.so").read_bytes()).hexdigest(), "source_sha256": {n: hashlib.sha256(s.encode()).hexdigest() for n, s in sources.items()}, "cases": 9, "actual_C_activation_cases": 3, "mutants": {}, "scope": __doc__}
    variants = {"actual": (probe, module_code)}
    if a.mutations:
        try:
            old = subprocess.check_output(["git", "show", f"{OLD_REF}:{PROBE_PATH}"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL)
            assert item(old, "    fn probe<'bound>(") == OLD_PROBE
            receipt["old_reference_checked_against_git"] = True
        except subprocess.CalledProcessError:
            receipt["old_reference_checked_against_git"] = False
        variants["eager-probe-before-allocation"] = (OLD_PROBE, module_code)
        needle = "                        None\n"
        assert probe.count(needle) == 1
        variants["retained-error-unbinds"] = (probe.replace(needle, "                        return Err(error);\n"), module_code)
        gate = "                if !MODULE_READY.load(Ordering::Acquire) {\n                    return Err(ENODEV);\n                }\n"
        assert probe.count(gate) == 1
        variants["no-registration-readiness-gate"] = (probe.replace(gate, ""), module_code)
        variants["pre-activation-defers"] = (probe.replace(gate, gate.replace("ENODEV", "EPROBE_DEFER")), module_code)
        late = "            #[cfg(CONFIG_DRM_ASAHI_G15)]\n            _g15_ready: g15::activate(),\n"
        assert module_code.count(late) == 1
        early = module_code.replace(late, "").replace("            _driver <-", late + "            _driver <-", 1)
        variants["activation-before-last-registration"] = (probe, early)
    for name, (method, module_variant) in variants.items():
        # Macro definitions precede the fake registration, just as kernel prelude exports them.
        source = macro + "\n" + PREFIX.replace("#![allow(dead_code, unused_variables)]\n", "").replace("    // SET_DRVDATA", set_data) + "\n" + ready + "\n" + driver + "\nimpl platform::Driver for Driver<'_> {\n    type IdInfo = &'static Config;\n    type Data<'bound> = Driver<'bound>;\n" + method + "\n}\nmod g15 { use super::*; pub type Driver<'a> = super::Driver<'a>; " + activate + "\n}\ntype G15Registration = driver::Registration<platform::Adapter<g15::Driver<'static>>>;\n" + module_variant + "\n" + SUFFIX.replace("    // CALLBACK", callback)
        source = "#![allow(dead_code, unused_variables, private_bounds, private_interfaces)]\n" + source
        src = out / (name + ".rs"); src.write_text(source)
        binary = out / name
        subprocess.run([a.rustc, "--edition=2021", "--cfg", "CONFIG_DRM_ASAHI_G15", str(src), "--extern", f"pin_init={library}", "-L", f"dependency={a.kernel_build / 'rust'}", "-o", str(binary)], check=True)
        r = subprocess.run([str(binary)], capture_output=True, text=True)
        receipt["mutants" if name != "actual" else "control"] = receipt.get("mutants" if name != "actual" else "control", {})
        result = {"exit": r.returncode, "stdout": r.stdout, "stderr": r.stderr}
        if name == "actual":
            receipt["control"] = result
            if r.returncode: raise SystemExit(r.stderr)
            if r.stdout.count(": PASS\n") != 9: raise SystemExit("control did not run every case")
        else:
            receipt["mutants"][name] = result
            if r.returncode == 0: raise SystemExit("SURVIVED: " + name)
            intended = {"eager-probe-before-allocation": "allocation-before-start: firmware probe before drvdata allocation", "retained-error-unbinds": "retained-start-failure-binds: callback result", "no-registration-readiness-gate": "late registration failure started firmware", "activation-before-last-registration": "activation before all registrations completed", "pre-activation-defers": "pre-activation probe deferred instead of ENODEV"}[name]
            result["intended_failure"] = intended in r.stderr
            if not result["intended_failure"]: raise SystemExit("INVALID unrelated failure: " + name + "\n" + r.stderr)
    (out / "result.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
