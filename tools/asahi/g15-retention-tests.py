#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run actual G15 retention/backend/boot-completion bodies with drop-count fault injection.

Allocators, firmware objects and hardware operations are host models. Production
ownership transfers and error handling are extracted unchanged. No firmware or
device is accessed. Mutants must build and fail an intended runtime assertion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
G15 = ROOT / "drivers/gpu/drm/asahi/g15"
CONTROL_REF = "31da169251bd34d5810a0874563d9a019b85bf56"


def item(text, marker):
    start = text.index(marker)
    end = text.index("{", start) + 1
    depth = 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


PREFIX = r'''
#![allow(dead_code, unused_variables)]
use std::{cell::{Cell, RefCell, RefMut}, mem::MaybeUninit, ops::{Deref, DerefMut},
          sync::{Arc, atomic::{AtomicBool, Ordering}}};
type Error = i32;
trait FromErrno { fn from_errno(value: i32) -> Self; }
impl FromErrno for Error { fn from_errno(value: i32) -> Self { value } }
type Result<T = ()> = std::result::Result<T, Error>;
const GFP_KERNEL: u32 = 0;
const EINVAL: Error = -22;
macro_rules! dev_info { ($($args:tt)*) => {{}}; }
macro_rules! dev_err { ($($args:tt)*) => {{}}; }
thread_local! {
    static FAIL: Cell<u32> = const { Cell::new(0) };
    static OWNER_DROPS: Cell<u32> = const { Cell::new(0) };
    static BUFFER_DROPS: Cell<u32> = const { Cell::new(0) };
    static PMP_CALLS: Cell<u32> = const { Cell::new(0) };
}
fn step(site: u32) -> Result {
    if FAIL.get() & (1 << site) != 0 { Err(-100 - site as i32) } else { Ok(()) }
}
struct Owner;
impl Drop for Owner { fn drop(&mut self) { OWNER_DROPS.set(OWNER_DROPS.get() + 1); } }
struct Buffer;
impl Drop for Buffer { fn drop(&mut self) { BUFFER_DROPS.set(BUFFER_DROPS.get() + 1); } }
struct Mutex<T>(RefCell<T>);
impl<T> Mutex<T> {
    fn new(value: T) -> Self { Self(RefCell::new(value)) }
    fn lock(&self) -> RefMut<'_, T> { self.0.borrow_mut() }
}
struct KBox<T>(Box<T>);
impl<T> KBox<T> {
    fn new_uninit(_flags: u32) -> Result<KBox<MaybeUninit<T>>> {
        step(if std::any::type_name::<T>().ends_with("Orphan") { 7 } else { 5 })?;
        Ok(KBox(Box::new_uninit()))
    }
    // KBOX_NEW
    fn into_inner(value: Self) -> T { *value.0 }
}
impl<T> KBox<MaybeUninit<T>> {
    fn write(storage: Self, value: T) -> KBox<T> { KBox(Box::write(storage.0, value)) }
}
impl<T> Deref for KBox<T> { type Target = T; fn deref(&self) -> &T { &self.0 } }
impl<T> DerefMut for KBox<T> { fn deref_mut(&mut self) -> &mut T { &mut self.0 } }
struct AsahiDevice;
impl AsahiDevice { fn as_ref(&self) -> &Self { self } }
mod hw {
    #[derive(PartialEq)]
    pub enum Platform { Adt, AdtWithPmp }
}
struct Config { platform: hw::Platform }
static CONFIG: Config = Config { platform: hw::Platform::Adt };
static PMP_CONFIG: Config = Config { platform: hw::Platform::AdtWithPmp };
mod mmu { pub struct Uat; }
mod regs {
    pub struct Identity;
    pub struct Regs;
    pub const ASC_RUN: u32 = 1;
    impl Regs { pub fn set_asc_control(&self, _: u32) -> super::Result { super::step(11) } }
}
mod object {
    use super::*;
    pub struct Objects { _owner: Owner, retained: Vec<Arc<Buffer>> }
    impl Objects {
        pub fn new() -> Self { Self { _owner: Owner, retained: vec![] } }
        pub fn buffer(&mut self) -> Arc<Buffer> {
            let buffer = Arc::new(Buffer); self.retained.push(buffer.clone()); buffer
        }
    }
}
mod initdata { pub struct InitData(pub super::Owner); }
mod channel { pub struct Channels(pub super::Owner); }
mod event {
    pub struct Events(pub super::Owner);
    impl Events {
        pub fn new() -> Self { Self(super::Owner) }
        pub fn reserve(&mut self) -> super::Result { super::step(12) }
    }
}
mod graph {
    use super::*;
    pub struct Graphs(Arc<Buffer>);
    impl Graphs {
        pub fn new(_: &AsahiDevice, objects: &mut object::Objects) -> Result<Self> {
            let buffer = objects.buffer(); step(1)?; Ok(Self(buffer))
        }
    }
}
mod slots {
    use super::*;
    pub struct Slots(Arc<Buffer>);
    impl Slots {
        pub fn new(objects: &mut object::Objects) -> Result<Self> {
            let buffer = objects.buffer(); step(2)?; Ok(Self(buffer))
        }
    }
}
mod pools {
    use super::*;
    pub struct Pools(Arc<Buffer>);
    impl Pools {
        pub fn new(_: &AsahiDevice, objects: &mut object::Objects) -> Result<Self> {
            let buffer = objects.buffer(); step(3)?; Ok(Self(buffer))
        }
    }
}
mod vm {
    use super::*;
    pub struct Registry;
    impl Registry { pub fn new() -> Result<Arc<Self>> { step(4)?; Ok(Arc::new(Self)) } }
}
mod engine { pub struct Engine; impl Engine { pub fn new() -> Self { Self } } }
mod recovery { pub struct Recovery; impl Recovery { pub fn new() -> Self { Self } } }
struct Shared {
    dev: AsahiDevice,
    orphan: Mutex<Option<KBox<boot::Orphan>>>,
    orphaned: AtomicBool,
}
impl Shared {
    fn new() -> Arc<Self> {
        Arc::new(Self { dev: AsahiDevice, orphan: Mutex::new(None), orphaned: AtomicBool::new(false) })
    }
    // ORPHAN
}
// FIRMWARE
impl Firmware {
    fn init_graphs(&mut self) -> Result { step(6) }
    // BACKEND
}
mod boot {
    use super::*;
    pub mod glue {
        pub unsafe fn asahi_g15_pmp_ready() -> i32 {
            super::PMP_CALLS.set(super::PMP_CALLS.get() + 1);
            match super::step(13) { Ok(()) => 0, Err(e) => e }
        }
    }
    use super::{object::Objects, initdata::InitData, channel::Channels, event::Events};
    pub struct Session { _owner: Owner, uat: Arc<mmu::Uat>, regs: regs::Regs }
    impl Session {
        pub fn new() -> Self { Self { _owner: Owner, uat: Arc::new(mmu::Uat), regs: regs::Regs } }
        // UAT
    }
    // BOOTED
    // ORPHAN_STRUCT
    fn run(_: &mut Session, _: &Config, _: &mut Objects, _: &mut Events,
           built: &mut Option<(InitData, Channels, u32)>, _: ()) -> Result {
        step(8)?;
        *built = Some((InitData(Owner), Channels(Owner), 1234));
        step(9)
    }
    pub fn start(shared: &Arc<Shared>) -> Result<Booted> {
        let mut session = Session::new();
        let mut objects = Objects::new();
        let d = AsahiDevice;
        let cfg = &CONFIG;
        let control = 0;
        let id = regs::Identity;
        let board_images = ();
        // START_TAIL
}
fn booted() -> boot::Booted {
    boot::Booted { session: boot::Session::new(), init: initdata::InitData(Owner),
        objects: object::Objects::new(), channels: channel::Channels(Owner),
        events: event::Events::new(), id: regs::Identity, max_frequency_khz: 1234 }
}
fn orphan() -> boot::Orphan {
    let boot::Booted { session, init, objects, channels, events, .. } = booted();
    boot::Orphan { session, _objects: objects, channels: Some(channels), _init: Some(init), events }
}
fn reset(fail: u32) {
    FAIL.set(fail); OWNER_DROPS.set(0); BUFFER_DROPS.set(0); PMP_CALLS.set(0);
}
fn retained(label: &str) {
    assert_eq!(OWNER_DROPS.get(), 0, "{label}: live session owner dropped");
    assert_eq!(BUFFER_DROPS.get(), 0, "{label}: firmware buffer dropped");
}
fn main() {
    for oom in [false, true] {
        reset(if oom { 1 << 7 } else { 0 });
        let shared = Shared::new(); shared.orphan(orphan());
        retained("orphan allocation");
        assert_eq!(shared.orphaned.load(Ordering::Acquire), !oom);
        assert_eq!(shared.orphan.lock().is_some(), !oom);
        std::mem::forget(shared);
    }
    for site in 1..=6 {
        for orphan_oom in [false, true] {
            reset((1 << site) | if orphan_oom { 1 << 7 } else { 0 });
            let shared = Shared::new();
            let result = Firmware::start_backend(&AsahiDevice, &CONFIG, booted(), &shared);
            assert!(matches!(result, Err(e) if e == -100 - site as i32), "backend error changed");
            retained("backend failure");
            assert_eq!(shared.orphan.lock().is_some(), !orphan_oom);
            std::mem::forget(shared);
        }
    }
    for site in [8, 9, 11, 12] {
        for orphan_oom in [false, true] {
            reset((1 << site) | if orphan_oom { 1 << 7 } else { 0 });
            let shared = Shared::new(); let result = boot::start(&shared);
            let expected = -100 - site as i32;
            assert!(matches!(result, Err(e) if e == expected), "boot error changed");
            retained("boot completion");
            assert_eq!(shared.orphan.lock().is_some(), !orphan_oom);
            std::mem::forget(shared);
        }
    }
    for orphan_oom in [false, true] {
        reset((1 << 13) | if orphan_oom { 1 << 7 } else { 0 });
        let shared = Shared::new();
        let result = Firmware::start_backend(&AsahiDevice, &PMP_CONFIG, booted(), &shared);
        assert!(matches!(result, Err(-113)), "PMP readiness error changed");
        assert_eq!(PMP_CALLS.get(), 1, "PMP readiness was not reported");
        retained("PMP readiness failure");
        assert_eq!(shared.orphan.lock().is_some(), !orphan_oom);
        std::mem::forget(shared);
    }
    for cfg in [&CONFIG, &PMP_CONFIG] {
    // A pending PMP error must be irrelevant on a platform without PMP.
    reset(if cfg.platform == hw::Platform::Adt { 1 << 13 } else { 0 });
    let shared = Shared::new();
    let booted = boot::start(&shared).expect("normal boot");
    assert_eq!(booted.max_frequency_khz, 1234);
    let uat: Arc<mmu::Uat> = booted.session.uat();
    let firmware = Firmware::start_backend(&AsahiDevice, cfg, booted, &shared).expect("normal backend");
    assert_eq!(PMP_CALLS.get(), u32::from(cfg.platform == hw::Platform::AdtWithPmp),
               "PMP readiness used on the wrong platform");
    assert!(Arc::ptr_eq(&uat, &firmware.session.uat()));
    assert!(!shared.orphaned.load(Ordering::Acquire));
    retained("normal success");
    std::mem::forget((shared, firmware));
    }
    println!("PASS 26 retention cases: orphan OOM, six backend failure sites, four boot-completion sites, PMP readiness failure and both platform success paths");
    println!("normal success owner_drops={} buffer_drops={}", OWNER_DROPS.get(), BUFFER_DROPS.get());
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mutations", action="store_true")
    args = parser.parse_args()
    out = (args.output or Path(tempfile.mkdtemp(prefix="g15-retention-"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    mod = (G15 / "mod.rs").read_text()
    boot = (G15 / "boot.rs").read_text()
    kbox = (ROOT / "rust/kernel/alloc/kbox.rs").read_text()
    orphan = item(mod, "    pub(crate) fn orphan(")
    backend = item(mod, "    fn start_backend(")
    start = item(boot, "pub(crate) fn start(")
    tail = start[start.index("    let mut events = Events::new();"):]
    source = PREFIX.replace("    // ORPHAN\n", orphan + "\n")
    allocator_new = item(kbox, "    pub fn new(x: T, flags: Flags)")
    allocator_new = allocator_new.replace("pub fn new(x: T, flags: Flags) -> Result<Self, AllocError>",
                                          "fn new(x: T, flags: u32) -> Result<Self>")
    source = source.replace("    // KBOX_NEW", allocator_new.replace("Box::write", "KBox::write"))
    source = source.replace("// FIRMWARE", item(mod, "pub(crate) struct Firmware"))
    source = source.replace("    // BACKEND", backend)
    source = source.replace("        // UAT", item(boot, "    pub(crate) fn uat("))
    source = source.replace("    // BOOTED", item(boot, "pub(crate) struct Booted"))
    source = source.replace("    // ORPHAN_STRUCT", item(boot, "pub(crate) struct Orphan"))
    source = source.replace("        // START_TAIL", tail)
    variants = {"actual": source}
    if args.mutations:
        old = subprocess.check_output(
            ["git", "show", CONTROL_REF + ":drivers/gpu/drm/asahi/g15/mod.rs"],
            cwd=ROOT, text=True)
        variants["consume-orphan-before-allocation"] = source.replace(orphan, item(old, "    pub(crate) fn orphan("))
        variants["unguarded-registry-allocation"] = source.replace(backend, item(old, "    fn start_backend("))
        variants["ignore-pmp-readiness-error"] = source.replace(
            "let ret = unsafe { boot::glue::asahi_g15_pmp_ready() };",
            "let _ret = unsafe { boot::glue::asahi_g15_pmp_ready() }; let ret = 0;")
        variants["pmp-on-other-platform"] = source.replace(
            "if cfg.platform != hw::Platform::AdtWithPmp {", "if false {")
    receipt = {"scope": __doc__, "sources": {
        name: hashlib.sha256(text.encode()).hexdigest()
        for name, text in [("mod.rs", mod), ("boot.rs", boot), ("kbox.rs", kbox)]},
        "rustc": subprocess.check_output([os.environ.get("HOSTRUSTC", "rustc"), "--version"], text=True).strip(),
        "cases": 26, "public_old_source_control": CONTROL_REF, "variants": {}}
    for name, text in variants.items():
        if name != "actual" and text == source:
            raise RuntimeError("NOT_APPLIED " + name)
        path = out / (name + ".rs")
        path.write_text(text)
        binary = out / name
        subprocess.run([os.environ.get("HOSTRUSTC", "rustc"), "--edition=2021",
                        str(path), "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        receipt["variants"][name] = {"exit": result.returncode, "stdout": result.stdout,
                                    "stderr": result.stderr}
        if name == "actual":
            if result.returncode:
                raise RuntimeError(result.stderr)
        else:
            expected = {"consume-orphan-before-allocation": "orphan allocation: live session owner dropped",
                        "unguarded-registry-allocation": "backend failure: live session owner dropped",
                        "ignore-pmp-readiness-error": "PMP readiness error changed",
                        "pmp-on-other-platform": "normal backend: -113"}[name]
            if result.returncode == 0 or expected not in result.stderr:
                raise RuntimeError("INVALID mutant outcome " + name + "\n" + result.stderr)
            if name in ("consume-orphan-before-allocation", "unguarded-registry-allocation"):
                if "left: 5" not in result.stderr:
                    raise RuntimeError("INVALID owner-drop count " + name)
                receipt["variants"][name]["intended_owner_drops"] = 5
            receipt["variants"][name]["intended_assertion"] = expected
    after = subprocess.run([str(out / "actual")], capture_output=True, text=True, check=True)
    receipt["actual_after_mutants"] = {"exit": after.returncode, "stdout": after.stdout, "stderr": after.stderr}
    receipt["fixed_invariants"] = {"live_session_owner_drops": 0, "firmware_buffer_drops": 0,
                                   "checked_cases": 26}
    (out / "result.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
