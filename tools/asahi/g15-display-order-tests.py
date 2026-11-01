#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the actual G15 native-display start-order bodies on host models.

The publication slots, the shared link helper and both SoC wrappers are taken
from drivers/gpu/drm/apple/apple_drv.c, the T6031 glue from g15/glue.c and
admit() from g15/boot.rs, unchanged. OF lookups, device references, links and
the admission steps are host models; no device or firmware is touched. Each
mutant must build and fail an intended assertion.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
APPLE = "drivers/gpu/drm/apple/apple_drv.c"
GLUE = "drivers/gpu/drm/asahi/g15/glue.c"
BOOT = "drivers/gpu/drm/asahi/g15/boot.rs"
MODULE = "drivers/gpu/drm/asahi/g15/mod.rs"


def item(text, marker):
    start = text.index(marker)
    end = text.index("{", start) + 1
    depth = 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def c_source(root):
    apple = (root / APPLE).read_text()
    glue = (root / GLUE).read_text()
    state = [line for line in apple.splitlines()
             if line in ("static DEFINE_MUTEX(apple_g15_display_lock);",
                         "static struct device *apple_t6030_display;",
                         "static struct device *apple_t6031_display;")]
    assert len(state) == 3, f"publication state not found: {state}"
    bodies = [item(apple, marker) for marker in (
        "\nstatic struct device **apple_g15_display_slot(struct device *dev)\n{",
        "\nstatic int apple_g15_display_publish(struct device *dev)\n{",
        "\nstatic void apple_g15_display_withdraw(struct device *dev)\n{",
        "\nstatic int apple_g15_link_device(",
        "\nint apple_drm_t6030_link_device(struct device *consumer)\n{",
        "\nint apple_drm_t6031_link_device(struct device *consumer)\n{",
    )]
    bodies.append(item(glue, "\nint asahi_g15_display_prepare(struct device *dev)\n{"))
    return "\n".join(state) + "\n\n" + "\n\n".join(bodies) + "\n"


C_PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ENODEV 19
#define EBUSY 16
#define EINVAL 22
#define EPROBE_DEFER 517
#define DL_FLAG_AUTOREMOVE_CONSUMER 1u
#define EXPORT_SYMBOL_GPL(sym) extern int sym##_exported

struct device_node { const char *compatible; bool available; int refs; };
struct device { struct device_node *of_node; int refs; };
struct device_link { struct device *consumer, *supplier; unsigned int flags; };
struct mutex { bool held; };

static const char *machine_compat[2];
static struct device_node *nodes[4];
static int links_made, link_fail, infos;
static struct device_link last_link;

static int of_machine_is_compatible(const char *compat)
{
	for (int i = 0; i < 2; i++)
		if (machine_compat[i] && !strcmp(machine_compat[i], compat))
			return 1;
	return 0;
}
static int of_device_is_compatible(const struct device_node *np, const char *compat)
{
	return np && !strcmp(np->compatible, compat);
}
static struct device_node *of_find_compatible_node(struct device_node *from, const char *type,
						   const char *compat)
{
	(void)from; (void)type;
	for (int i = 0; i < 4; i++)
		if (nodes[i] && !strcmp(nodes[i]->compatible, compat)) {
			nodes[i]->refs++;
			return nodes[i];
		}
	return NULL;
}
static bool of_device_is_available(const struct device_node *np) { return np && np->available; }
static void of_node_put(struct device_node *np) { if (np) np->refs--; }
static struct device *get_device(struct device *dev) { dev->refs++; return dev; }
static void put_device(struct device *dev) { if (dev) dev->refs--; }
static struct device_link *device_link_add(struct device *consumer, struct device *supplier,
					   unsigned int flags)
{
	if (link_fail)
		return NULL;
	links_made++;
	last_link = (struct device_link){ consumer, supplier, flags };
	return &last_link;
}
#define dev_info_once(dev, ...) do { static bool once_; (void)(dev); if (!once_) { once_ = true; infos++; } } while (0)

static void unlock_(struct mutex **m) { if (*m) { assert((*m)->held); (*m)->held = false; } }
static struct mutex *lock_(struct mutex *m) { assert(!m->held); m->held = true; return m; }
#define DEFINE_MUTEX(name) struct mutex name = { false }
#define guard(type) struct mutex *guard_ __attribute__((cleanup(unlock_))) = lock_
#define scoped_guard(type, m) for (struct mutex *sg_ __attribute__((cleanup(unlock_))) = lock_(m), *done_ = NULL; !done_; done_ = (struct mutex *)1)
'''

C_MAIN = r'''
static const char *scenario;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: %s\n", scenario, __LINE__, #c); exit(1); } } while (0)

static struct device_node gpu31 = { "apple,agx-t6031", true }, gpu30 = { "apple,agx-t6030", true };
static struct device_node disp31 = { "apple,t6031-display-subsystem", false };
static struct device_node disp30 = { "apple,t6030-display-subsystem", true };
static struct device_node other = { "apple,display-subsystem", true };
static struct device g31 = { &gpu31 }, g30 = { &gpu30 }, none = { NULL };
static struct device d31 = { &disp31 }, d31b = { &disp31 }, d30 = { &disp30 }, dother = { &other };

static void machine(const char *soc, const char *board)
{
	machine_compat[0] = board;
	machine_compat[1] = soc;
}
static void clean(void)
{
	CHECK(!apple_g15_display_lock.held);
	for (int i = 0; i < 4; i++)
		CHECK(!nodes[i] || nodes[i]->refs == 0);
}
static int prepare(struct device *dev) { int ret = asahi_g15_display_prepare(dev); clean(); return ret; }
static int link30(struct device *dev) { int ret = apple_drm_t6030_link_device(dev); clean(); return ret; }
static int link31(struct device *dev) { int ret = apple_drm_t6031_link_device(dev); clean(); return ret; }
static int publish(struct device *dev) { int ret = apple_g15_display_publish(dev); clean(); return ret; }
static void withdraw(struct device *dev) { apple_g15_display_withdraw(dev); clean(); }

int main(int argc, char **argv)
{
	scenario = argv[1];
	machine("apple,t6031", "apple,j516c");
	nodes[0] = &gpu31;
	nodes[1] = &disp31;
	nodes[2] = &disp30;
	nodes[3] = &other;
	if (!strcmp(scenario, "disabled")) {
		/* The gate left the display disabled: no start, no success, said once. */
		CHECK(prepare(&g31) == -EPROBE_DEFER && infos == 1);
		CHECK(prepare(&g31) == -EPROBE_DEFER && infos == 1 && links_made == 0);
	} else if (!strcmp(scenario, "absent")) {
		nodes[1] = NULL;
		CHECK(prepare(&g31) == -EPROBE_DEFER && infos == 1 && links_made == 0);
	} else if (!strcmp(scenario, "pending")) {
		/* Enabled but not published: no start, no success, said once. */
		disp31.available = true;
		CHECK(prepare(&g31) == -EPROBE_DEFER && infos == 1 && links_made == 0);
		CHECK(prepare(&g31) == -EPROBE_DEFER && infos == 1 && links_made == 0);
	} else if (!strcmp(scenario, "ready")) {
		disp31.available = true;
		CHECK(publish(&d31) == 0 && d31.refs == 1);
		CHECK(prepare(&g31) == 0 && links_made == 1 && infos == 0);
		CHECK(last_link.consumer == &g31 && last_link.supplier == &d31 &&
		      last_link.flags == DL_FLAG_AUTOREMOVE_CONSUMER);
		link_fail = 1;
		CHECK(prepare(&g31) == -EINVAL);
		link_fail = 0;
		CHECK(publish(&d31) == 0 && d31.refs == 1);
		CHECK(publish(&d31b) == -EBUSY && d31b.refs == 0);
		withdraw(&d31b);
		CHECK(prepare(&g31) == 0);
		withdraw(&d31);
		CHECK(d31.refs == 0 && prepare(&g31) == -EPROBE_DEFER);
	} else if (!strcmp(scenario, "isolation")) {
		disp31.available = true;
		CHECK(publish(&d30) == 0 && d30.refs == 1);
		CHECK(publish(&dother) == 0 && dother.refs == 0);
		withdraw(&dother);
		CHECK(prepare(&g31) == -EPROBE_DEFER && links_made == 0);
	} else if (!strcmp(scenario, "foreign")) {
		disp31.available = true;
		CHECK(publish(&d31) == 0);
		CHECK(prepare(&g30) == -ENODEV && link31(&g30) == -ENODEV);
		CHECK(prepare(&none) == -ENODEV && link31(&none) == -ENODEV);
		machine("apple,t6030", "apple,j516s");
		CHECK(link31(&g31) == -ENODEV && links_made == 0);
	} else if (!strcmp(scenario, "t6030")) {
		machine("apple,t6030", "apple,j516s");
		CHECK(link30(&g30) == -EPROBE_DEFER);
		disp31.available = true;
		CHECK(publish(&d31) == 0 && link30(&g30) == -EPROBE_DEFER);
		CHECK(publish(&d30) == 0 && link30(&g30) == 0 && last_link.supplier == &d30);
		withdraw(&d30);
		CHECK(d30.refs == 0 && link30(&g30) == -EPROBE_DEFER);
		CHECK(link30(&g31) == -ENODEV);
	} else {
		CHECK(!"unknown scenario");
	}
	return 0;
}
'''

C_SCENARIOS = ["disabled", "absent", "pending", "ready", "isolation", "foreign", "t6030"]

C_MUTANTS = {
    "unpublished-succeeds": ("if (!*display)\n\t\treturn -EPROBE_DEFER;", "if (!*display)\n\t\treturn 0;"),
    "disabled-display-skipped": (re.compile(r"\t\tdev_info_once\(dev,[^;]*;\n"), "\t\t{ of_node_put(np); return 0; }\n"),
    "pending-display-silent": (re.compile(r"\telse\n\t\tdev_info_once\(dev,[^;]*;\n"), ""),
    "t6031-reads-t6030-slot": ("&apple_t6031_display);", "&apple_t6030_display);"),
    "t6031-published-as-t6030": ("return &apple_t6031_display;", "return &apple_t6030_display;"),
    "no-machine-check": ("!of_machine_is_compatible(machine) ||", ""),
    "withdraw-keeps-display": ("*slot = NULL;", ";"),
    "publish-without-reference": ("*slot = get_device(dev);", "*slot = dev;"),
}

RS_PREFIX = r'''
#![allow(dead_code, unused_variables, unused_imports, unused_macros, non_snake_case)]
use std::cell::RefCell;
use std::marker::PhantomData;
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
struct Error(i32);
impl Error { fn from_errno(errno: i32) -> Self { Self(errno) } }
type Result<T = ()> = std::result::Result<T, Error>;
const ENODEV: Error = Error(-19);
const EINVAL: Error = Error(-22);
const EIO: Error = Error(-5);
const EPROBE_DEFER: Error = Error(-517);
macro_rules! dev_err { ($($args:tt)*) => {{}}; }
macro_rules! dev_info { ($($args:tt)*) => {{}}; }
macro_rules! dev_warn { ($($args:tt)*) => {{ warned(); }}; }
struct State { board: bool, machine: bool, pmp: i32, display: i32, sleep_hold: bool, asc_fail: bool }
thread_local! {
    static CALLS: RefCell<Vec<&'static str>> = const { RefCell::new(Vec::new()) };
    static STATE: RefCell<State> = const { RefCell::new(State { board: true, machine: true, pmp: 0, display: 0, sleep_hold: true, asc_fail: false }) };
    static HOLD: RefCell<Option<bool>> = const { RefCell::new(None) };
    static WARNINGS: RefCell<u32> = const { RefCell::new(0) };
}
fn warned() { WARNINGS.with(|w| *w.borrow_mut() += 1); }
mod params {
    pub fn sleep_hold() -> bool { super::STATE.with(|s| s.borrow().sleep_hold) }
}
fn call(name: &'static str) { CALLS.with(|c| c.borrow_mut().push(name)); }
mod device {
    pub struct Device;
    impl Device { pub fn as_raw(&self) -> *mut u8 { std::ptr::null_mut() } }
}
mod platform {
    pub struct Device<C>(pub super::device::Device, pub C);
    impl<C> Device<C> { pub fn as_ref(&self) -> &super::device::Device { &self.0 } }
}
struct Core<'a>(PhantomData<&'a ()>);
struct AsahiDevice;
mod hw {
    #[derive(Copy, Clone, PartialEq, Eq)] pub enum Platform { AdtWithPmp, DeviceTree }
    #[derive(Copy, Clone, PartialEq, Eq)] pub enum Board { None, Templates, Blob }
}
struct Config { platform: hw::Platform, machine: &'static str, name: &'static str, firmware: (), board: hw::Board }
mod adt {
    pub struct GpuAdt;
    pub fn read(_: &super::device::Device, _: &super::Config) -> super::Result<GpuAdt> { super::call("adt"); Ok(GpuAdt) }
}
mod identity {
    pub fn firmware(_: &super::device::Device, _: &super::adt::GpuAdt, _: ()) -> super::Result { super::call("identity"); Ok(()) }
}
mod initdata {
    pub fn prepare_board(_: &super::device::Device, _: &super::Config, _: &super::adt::GpuAdt) -> super::Result<u32> {
        super::call("board_images"); Ok(1)
    }
}
mod regs {
    pub struct Regs;
    impl Regs {
        pub fn new<C>(_: &super::platform::Device<C>) -> super::Result<Self> { super::call("regs"); Ok(Self) }
        pub fn asc_control(&self) -> super::Result<u32> {
            if super::STATE.with(|s| s.borrow().asc_fail) { Err(super::EIO) } else { Ok(0) }
        }
    }
}
mod glue {
    use super::{call, STATE};
    pub fn board() -> bool { call("board"); STATE.with(|s| s.borrow().board) }
    pub fn machine(_: &str) -> bool { call("machine"); STATE.with(|s| s.borrow().machine) }
    pub unsafe fn asahi_g15_pmp_prepare(_: *mut core::ffi::c_void) -> core::ffi::c_int { call("pmp"); STATE.with(|s| s.borrow().pmp) }
    pub unsafe fn asahi_g15_display_prepare(_: *mut core::ffi::c_void) -> core::ffi::c_int { call("display"); STATE.with(|s| s.borrow().display) }
    pub fn dma_coherent(_: &super::device::Device) -> bool { call("coherent"); false }
@HOLD_POWER@
}
struct Admitted { regs: regs::Regs, adt: adt::GpuAdt, board_images: u32 }
'''

RS_MAIN = r'''
static T6031: Config = Config { platform: hw::Platform::DeviceTree, machine: "apple,j516c", name: "T6031 G15C", firmware: (), board: hw::Board::Blob };
static T6030: Config = Config { platform: hw::Platform::AdtWithPmp, machine: "apple,t6030", name: "T6030 G15S", firmware: (), board: hw::Board::Templates };
const TAIL: [&str; 6] = ["adt", "identity", "coherent", "board_images", "regs", "hold"];
fn run(cfg: &'static Config, machine: bool, pmp: i32, display: i32) -> (Result<()>, Vec<&'static str>) {
    CALLS.with(|c| c.borrow_mut().clear());
    STATE.with(|s| *s.borrow_mut() = State { board: true, machine, pmp, display, sleep_hold: true, asc_fail: false });
    HOLD.with(|h| *h.borrow_mut() = None);
    WARNINGS.with(|w| *w.borrow_mut() = 0);
    let pdev = platform::Device(device::Device, Core(PhantomData));
    let ret = admit(&pdev, &AsahiDevice, cfg).map(|_| ());
    (ret, CALLS.with(|c| c.borrow().clone()))
}
fn with_tail(head: &[&'static str]) -> Vec<&'static str> { head.iter().chain(TAIL.iter()).copied().collect() }
@SLEEP_HOLD_RUN@
fn main() {
    assert_eq!(run(&T6031, true, 0, -517), (Err(EPROBE_DEFER), vec!["machine", "display"]), "T6031 defers before firmware");
    assert_eq!(run(&T6031, true, 0, -19), (Err(ENODEV), vec!["machine", "display"]), "T6031 refusal stops admission");
    assert_eq!(run(&T6031, true, 0, 0), (Ok(()), with_tail(&["machine", "display"])), "T6031 admitted after its display");
    assert_eq!(run(&T6031, false, 0, 0), (Err(ENODEV), vec!["machine"]), "the machine check comes first");
    assert_eq!(run(&T6030, true, 0, -517), (Ok(()), with_tail(&["board", "pmp"])), "T6030 path unchanged");
    assert_eq!(run(&T6030, true, -517, 0), (Err(EPROBE_DEFER), vec!["board", "pmp"]), "T6030 still defers on its PMP link");
@SLEEP_HOLD_CHECKS@
}
'''

HOLD_PLAIN = '    pub fn hold_power(_: &super::device::Device) { call("hold"); }'
HOLD_SLEEP = ('    pub fn hold_power(_: &super::device::Device, sleep_hold: bool) {\n'
              '        call("hold"); super::HOLD.with(|h| *h.borrow_mut() = Some(sleep_hold));\n    }')
SLEEP_HOLD_RUN = r'''
fn run_hold(cfg: &'static Config, sleep_hold: bool, asc_fail: bool) -> (Result<()>, Vec<&'static str>, Option<bool>, u32) {
    CALLS.with(|c| c.borrow_mut().clear());
    STATE.with(|s| *s.borrow_mut() = State { board: true, machine: true, pmp: 0, display: 0, sleep_hold, asc_fail });
    HOLD.with(|h| *h.borrow_mut() = None);
    WARNINGS.with(|w| *w.borrow_mut() = 0);
    let pdev = platform::Device(device::Device, Core(PhantomData));
    let ret = admit(&pdev, &AsahiDevice, cfg).map(|_| ());
    (ret, CALLS.with(|c| c.borrow().clone()), HOLD.with(|h| *h.borrow()), WARNINGS.with(|w| *w.borrow()))
}
fn before_hold(head: &[&'static str]) -> Vec<&'static str> { head.iter().chain(TAIL[..TAIL.len() - 1].iter()).copied().collect() }
'''
SLEEP_HOLD_CHECKS = r'''
    // The system-sleep hold: T6031 only, and only while g15_sleep_hold is on; off is warned about.
    assert_eq!(run_hold(&T6031, true, false), (Ok(()), with_tail(&["machine", "display"]), Some(true), 0), "T6031 holds gfx through sleep");
    assert_eq!(run_hold(&T6031, false, false), (Ok(()), with_tail(&["machine", "display"]), Some(false), 1), "g15_sleep_hold=0: no sleep hold, one warning");
    assert_eq!(run_hold(&T6030, true, false), (Ok(()), with_tail(&["board", "pmp"]), Some(false), 0), "T6030 never holds through sleep");
    // The ASC read comes before the hold: a failure there leaves no hold behind.
    assert_eq!(run_hold(&T6031, true, true), (Err(EIO), before_hold(&["machine", "display"]), None, 0), "T6031: ASC failure takes no hold");
    assert_eq!(run_hold(&T6030, true, true), (Err(EIO), before_hold(&["board", "pmp"]), None, 0), "T6030: ASC failure takes no hold");
'''
SLEEP_HOLD_MUTANTS = {
    "sleep-hold-any-platform": ("let sleep_hold = t6031 && params::sleep_hold();", "let sleep_hold = params::sleep_hold();"),
    "sleep-hold-ignores-parameter": ("let sleep_hold = t6031 && params::sleep_hold();", "let sleep_hold = t6031;"),
    "no-warning-when-off": ("} else if t6031 {", "} else if false {"),
    "asc-read-after-hold": (re.compile(r"(\n\s*)let asc = regs\.asc_control\(\)\?;(.*?glue::hold_power\(dev, sleep_hold\);)", re.S),
                            r"\2\1let asc = regs.asc_control()?;"),
}

RS_MUTANTS = {
    "display-refusal-ignored": ("dev_err!(dev, \"native display not admitted: {:?}\\n\", err);\n            }\n            return Err(err);",
                                "dev_err!(dev, \"native display not admitted: {:?}\\n\", err);\n            }"),
    "display-order-skipped": ("let ret = unsafe { glue::asahi_g15_display_prepare(dev.as_raw().cast()) };", "let ret = 0;"),
}


def mutate(source, change):
    old, new = change
    if isinstance(old, re.Pattern):
        mutated, count = old.subn(new, source, count=1)
    else:
        count = source.count(old)
        mutated = source.replace(old, new, 1)
    assert count >= 1, f"mutation anchor not found: {old!r}"
    return mutated


def compile_c(work, source, name):
    src = work / f"{name}.c"
    exe = work / name
    src.write_text(C_PREFIX + source + C_MAIN)
    result = subprocess.run(["gcc", "-std=gnu11", "-Wall", "-Werror", "-Wno-unused-function",
                             "-o", str(exe), str(src)], capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(f"{name}: does not build:\n{result.stderr}")
    return exe


def c_failures(exe):
    return [s for s in C_SCENARIOS if subprocess.run([str(exe), s], capture_output=True).returncode]


def rs_model(admit):
    """The prelude and main for this admit(): with the system-sleep hold argument (g15_sleep_hold) or without."""
    sleep = "glue::hold_power(dev, sleep_hold);" in admit
    prefix = RS_PREFIX.replace("@HOLD_POWER@", HOLD_SLEEP if sleep else HOLD_PLAIN)
    main = RS_MAIN.replace("@SLEEP_HOLD_RUN@", SLEEP_HOLD_RUN if sleep else "").replace("@SLEEP_HOLD_CHECKS@", SLEEP_HOLD_CHECKS if sleep else "")
    return sleep, prefix, main


def compile_rs(work, rustc, source, name, model):
    src = work / f"{name}.rs"
    exe = work / name
    src.write_text(model[1] + source + model[2])
    result = subprocess.run([rustc, "--edition=2021", "-o", str(exe), str(src)], capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(f"{name}: does not build with {rustc}:\n{result.stderr}")
    return exe


def static_checks(root):
    apple = (root / APPLE).read_text()
    module = (root / MODULE).read_text()
    glue = (root / GLUE).read_text()
    compat = re.compile(r'of::DeviceId::new\(c_str!\("([^"]+)"\)\)')
    start = module.index("kernel::of_device_table!(\n    AUTOLOAD_OF_TABLE,")
    autoload = module[start:module.index("\n);", start)]
    start = module.index("const OF_TABLE:")
    matched = module[start:module.index("]);", start)]
    assert compat.findall(autoload) == ["apple,agx-t6030", "apple,agx-t6031"], "autoload table"
    assert compat.findall(matched) == ["apple,agx-t6030", "apple,agx-t6031"], "match table"
    init = item(apple, "\nstatic int apple_drm_init(struct device *dev)\n{")
    order = [init.index(s) for s in ("drm_dev_register(", "drm_client_setup_with_fourcc(", "apple_g15_display_publish(dev)")]
    assert order == sorted(order), "publication must follow DRM registration and client setup"
    uninit = item(apple, "\nstatic void apple_drm_uninit(struct device *dev)\n{")
    assert uninit.index("apple_g15_display_withdraw(dev)") < uninit.index("component_unbind_all("), "withdraw before unbind"
    assert "apple_t6030_display_publish" not in apple and "apple_t6030_display_withdraw" not in apple
    pmp = item(glue, "\nint asahi_g15_pmp_prepare(struct device *dev)\n{")
    assert pmp.index("apple_drm_t6030_link_device(dev)") < pmp.index("apple_pmp_link_device(dev)"), "T6030 order"
    return 5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--rustc", action="append", default=[], help="rustc to build the admit() model with")
    args = parser.parse_args()
    rustcs = args.rustc or ["rustc"]
    checks = static_checks(args.root)
    source = c_source(args.root)
    admit = item((args.root / BOOT).read_text(), "pub(crate) fn admit(")
    model = rs_model(admit)
    rs_mutants = dict(RS_MUTANTS, **(SLEEP_HOLD_MUTANTS if model[0] else {}))
    with tempfile.TemporaryDirectory(prefix="g15-display-order-", dir=os.environ.get("TMPDIR")) as tmp:
        work = Path(tmp)
        failed = c_failures(compile_c(work, source, "control"))
        assert not failed, f"control scenarios failed: {failed}"
        for name, change in C_MUTANTS.items():
            failed = c_failures(compile_c(work, mutate(source, change), f"mutant-{name}"))
            assert failed, f"C mutant {name} survived"
            print(f"mutant {name}: caught by {', '.join(failed)}")
        for rustc in rustcs:
            exe = compile_rs(work, rustc, admit, "admit", model)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            assert run.returncode == 0, f"admit() control failed with {rustc}:\n{run.stderr}"
            for name, change in rs_mutants.items():
                exe = compile_rs(work, rustc, mutate(admit, change), f"admit-{name}", model)
                run = subprocess.run([str(exe)], capture_output=True, text=True)
                assert run.returncode, f"admit() mutant {name} survived with {rustc}"
                print(f"mutant {name} ({rustc}): caught")
    print(f"PASS: {checks} static checks, {len(C_SCENARIOS)} C scenarios, "
          f"{len(C_MUTANTS)} C mutants, admit() on {len(rustcs)} rustc, {len(rs_mutants)} admit() mutants"
          + (", system-sleep hold checks" if model[0] else ""))


if __name__ == "__main__":
    sys.exit(main())
