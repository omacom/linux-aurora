#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Check production DWC3 role/reset ordering with bounded host MMIO mocks.

Exercises the actual role writer, reset helper, initial-role admission, Apple
probe/retry callbacks, and the core-probe sequence after role discovery. Resource
mapping, runtime PM and physical clock availability are outside this CPU model.
"""
import os
from pathlib import Path
import re
import resource
import shlex
import subprocess
import tempfile


def function(source, name):
    match = re.search(r"^(?:static )?(?:inline )?(?:int|void|bool|u32) " + name + r"\(", source, re.M)
    assert match, name
    brace = source.index("{", match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


def main():
    root = Path(__file__).resolve().parents[5]
    path = root / "drivers/usb/dwc3/core.c"
    if not path.exists():
        print("# SKIP: requires matching kernel source tree")
        return 4
    core = path.read_text()
    apple = (root / "drivers/usb/dwc3/dwc3-apple.c").read_text()
    states = (root / "drivers/usb/dwc3/dwc3-apple-state.h").read_text()
    enum = states[states.index("enum dwc3_apple_state {"):]
    enum = enum[:enum.index("};") + 2]
    sleep_phase = apple[apple.index("enum dwc3_apple_sleep_phase {"):]
    sleep_phase = sleep_phase[:sleep_phase.index("};") + 2]
    probe = function(core, "dwc3_core_probe")
    sequence = probe[probe.index("\tret = dwc3_get_dr_mode(dwc);"):probe.index("\tdwc3_check_params(dwc);")]
    preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint32_t u32;
#define BIT(n) (1U<<(n))
#define GENMASK(h,l) ((~0U<<(l)) & (~0U>>(31-(h))))
#define FIELD_PREP(m,v) (((v)<<__builtin_ctz(m)) & (m))
#define DWC3_GCTL 0xc110
#define DWC3_DCTL 0xc704
#define DWC3_GCTL_PRTCAP_HOST 1
#define DWC3_GCTL_PRTCAP_DEVICE 2
#define DWC3_GCTL_PRTCAP_OTG 3
#define DWC3_GCTL_PRTCAPDIR(v) ((v)<<12)
#define DWC3_GCTL_PRTCAP(v) (((v)>>12)&3)
#define DWC3_DCTL_CSFTRST (1U<<30)
#define DWC3_DCTL_RUN_STOP (1U<<31)
#define DWC3_GHWPARAMS0_MODE(v) (v)
#define DWC3_GHWPARAMS0_MODE_DRD 2
#define DWC3_VER_IS_WITHIN(...) false
#define DWC3_IP_IS(...) false
#define trace_dwc3_set_prtcap(...) ((void)0)
#define spin_lock_irqsave(lock,flags) ((flags)=0)
#define spin_unlock_irqrestore(lock,flags) ((void)(flags))
#define guard(x) fake_guard
#define lockdep_assert_held(x) ((void)0)
#define WARN_ON_ONCE(x) ((void)(x))
#define dev_warn(...) ((void)0)
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define DWC3_GCTL_PWRDNSCALE(n) ((n)<<19)
#define DWC3_GCTL_PWRDNSCALE_MASK GENMASK(31,19)
#define DWC3_GCTL_GBLHIBERNATIONEN BIT(1)
#define dev_err_probe(dev,err,...) fake_dev_err_probe(err)
#define GFP_KERNEL 0
#define DWC3_DEFAULT_PROPERTIES ((struct properties){0})
enum usb_role { USB_ROLE_NONE, USB_ROLE_HOST, USB_ROLE_DEVICE };
enum usb_dr_mode { USB_DR_MODE_UNKNOWN, USB_DR_MODE_HOST, USB_DR_MODE_PERIPHERAL, USB_DR_MODE_OTG };
struct device { const char *of_node; bool host_only; void *driver_data; };
static void dev_set_drvdata(struct device *d,void *data) { d->driver_data=data; }
struct dwc3_glue_ops { int unused; };
static const struct dwc3_glue_ops dwc3_apple_glue_ops;
struct dwc3 {
    struct device *dev;
    const struct dwc3_glue_ops *glue_ops;
    unsigned lock;
    struct { unsigned hwparams0; } hwparams;
    enum usb_dr_mode dr_mode;
    u32 current_dr_role, gctl, dctl;
    unsigned num_usb2_ports, num_usb3_ports;
};
struct properties { bool xhci_run_in_s2idle; };
struct dwc3_probe_data {
    struct dwc3 *dwc;
    void *res;
    bool ignore_clocks_and_resets, skip_core_init_mode;
    enum usb_role initial_role;
    struct properties properties;
};
struct dwc3_apple {
    struct dwc3 dwc;
    struct device *dev;
    void *mmio_resource;
    unsigned lock;
    int state;
    void *apple_regs, *reset, *usb2_phy;
    bool reset_pending, core_active;
    unsigned usb2_power_off_pending, usb3_power_off_pending;
    bool idle_rpm, rpm_held, pre_core_rpm, sleep_phy_wait, sleep_syscore;
    unsigned sleep_pm_lock;
    enum dwc3_apple_sleep_phase sleep_phase;
};
enum phy_mode { PHY_MODE_INVALID, PHY_MODE_USB_HOST, PHY_MODE_USB_DEVICE };
static bool j713=true, stall_reset=true, io_ready;
static unsigned role_writes, dctl_writes, core_calls, releases, mode_at_init, dr_mode_at_init;
static u32 cio[64];
static unsigned cio_writes, tunable_calls, rpm_gets;
static bool syscore;
static int stop_error;
static int fake_dev_err_probe(int error) { return error; }
static void fake_guard(void *p) { (void)p; }
static bool of_machine_is_compatible(const char *s) { return j713 && !strcmp(s,"apple,j713"); }
static bool of_device_is_compatible(const char *node,const char *s) { return !strcmp(node,s); }
static void msleep(unsigned n) { (void)n; }
static void udelay(unsigned n) { (void)n; }
static u32 dwc3_readl(struct dwc3 *d,u32 off)
{
    assert(io_ready);
    if(off==DWC3_GCTL) return d->gctl;
    assert(off==DWC3_DCTL);
    if(!stall_reset) d->dctl &= ~DWC3_DCTL_CSFTRST;
    return d->dctl;
}
static void dwc3_writel(struct dwc3 *d,u32 off,u32 value)
{
    assert(io_ready);
    if(off==DWC3_GCTL) { role_writes++; d->gctl=value; }
    else { assert(off==DWC3_DCTL); dctl_writes++; d->dctl=value; }
}
static void dwc3_gadget_dctl_write_safe(struct dwc3 *d,u32 value) { dwc3_writel(d,DWC3_DCTL,value); }
static void dwc3_enable_susphy(struct dwc3 *d,bool on) { (void)d; (void)on; }
static void *devres_open_group(struct device *d,void *id,int flags)
{ (void)id; (void)flags; return d; }
static void devres_release_group(struct device *d,void *group) { (void)d; (void)group; releases++; }
static void devres_remove_group(struct device *d,void *group) { (void)d; (void)group; }
static int dwc3_get_dr_mode(struct dwc3 *d)
{ d->dr_mode=d->dev->host_only ? USB_DR_MODE_HOST : USB_DR_MODE_OTG; return 0; }
static u32 readl(void *p)
{ assert(io_ready && core_calls); assert((uintptr_t)p>=(uintptr_t)cio &&
  (uintptr_t)p<(uintptr_t)cio+sizeof(cio)); return *(u32 *)p; }
static void writel(u32 value,void *p) { (void)readl(p); cio_writes++; *(u32 *)p=value; }
static int phy_set_mode(void *p,enum phy_mode mode) { (void)p; (void)mode; return 0; }
static int reset_control_deassert(void *p) { (void)p; return 0; }
static void dev_pm_syscore_device(struct device *d,bool on) { (void)d; syscore=on; }
static void pm_runtime_get_noresume(struct device *d) { (void)d; rpm_gets++; }
static void pm_runtime_allow(struct device *d) { (void)d; }
static int pm_runtime_resume_and_get(struct device *d) { (void)d; rpm_gets++; return 0; }
static void pm_runtime_disable(struct device *d) { (void)d; }
static void pm_runtime_put_noidle(struct device *d) { (void)d; }
/* The ADT tunables are outside this model; the core reset undoes them, so they follow it. */
static void dwc3_apple_apply_tunables(struct dwc3_apple *a) { (void)a; assert(core_calls); tunable_calls++; }
static int dwc3_apple_core_stop(struct dwc3_apple *a) { (void)a; return stop_error; }
'''
    cio_defines = "\n".join(line for line in apple.splitlines()
                            if line.startswith(("#define APPLE_DWC3_",
                                                "#define DWC3_APPLE_T8140_PWRDNSCALE"))) + "\n"
    program = preamble.replace("struct dwc3_apple {", sleep_phase + "\nstruct dwc3_apple {", 1)
    program += enum + "\n" + cio_defines + function(core,"dwc3_set_prtcap")
    program += function(core,"dwc3_core_soft_reset") + function(core,"dwc3_core_set_initial_role")
    program += r'''
static int dwc3_core_init(struct dwc3 *dwc)
{ core_calls++; mode_at_init=DWC3_GCTL_PRTCAP(dwc->gctl); dr_mode_at_init=dwc->dr_mode;
  return dwc3_core_soft_reset(dwc); }
static int dwc3_core_probe(const struct dwc3_probe_data *data)
{
    struct dwc3 *dwc=data->dwc;
    int ret;
    /* The production slice runs after mapping, clock enable, lock init and hwparams. */
    io_ready=true;
    dwc->hwparams.hwparams0=DWC3_GHWPARAMS0_MODE_DRD;
''' + sequence + r'''
    return 0;
err_free_event_buffers:
    return ret;
}
'''
    # The board selection comes from <linux/soc/apple/t8132.h>.
    t8132 = (root / "include/linux/soc/apple/t8132.h").read_text()
    for name in ["apple_t8132_air", "apple_t8132_pro_mini_imac",
                 "apple_t8132_usb_host_first"]:
        program += function(t8132,name)
    for name in ["dwc3_apple_syscore", "dwc3_apple_set_ptrcap", "dwc3_apple_host_before_core_init",
                 "dwc3_apple_core_probe", "dwc3_apple_core_init"]:
        program += function(apple,name)
    program += function(states,"dwc3_apple_usb2_mode_before_reset")
    for name in ["dwc3_apple_readl", "dwc3_apple_writel", "dwc3_apple_mask",
                 "dwc3_apple_setup_hibernation", "dwc3_apple_setup_cio", "dwc3_apple_core_start"]:
        program += function(apple,name)
    program += r'''
static struct device dev;
static struct dwc3_apple a;
static void clean(void)
{
    dev=(struct device){ .of_node="apple,t8132-dwc3", .host_only=true };
    a=(struct dwc3_apple){ .dev=&dev, .state=DWC3_APPLE_PROBE_PENDING, .apple_regs=cio };
    a.dwc.gctl=0xa5a50000;
    j713=stall_reset=true; io_ready=false;
    role_writes=dctl_writes=core_calls=releases=mode_at_init=dr_mode_at_init=0;
    cio_writes=tunable_calls=rpm_gets=0; syscore=false; stop_error=0; memset(cio,0xa5,sizeof(cio));
}
int main(void)
{
    clean();
    assert(!dwc3_apple_core_init(&a,DWC3_APPLE_HOST));
    assert(a.state==DWC3_APPLE_NO_CABLE && a.dwc.current_dr_role==DWC3_GCTL_PRTCAP_HOST);
    assert(a.dwc.gctl==(0xa5a50000|DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST)));
    assert(role_writes==1 && !dctl_writes && core_calls==1 && mode_at_init==DWC3_GCTL_PRTCAP_HOST);
    puts("ok 1 - first M4 host writes real GCTL HOST before core reset selection");
    a.dwc.gctl=0; a.dwc.current_dr_role=0;
    a.dwc.dr_mode=USB_DR_MODE_PERIPHERAL;
    assert(!dwc3_apple_core_init(&a,DWC3_APPLE_HOST));
    assert(role_writes==2 && !dctl_writes && core_calls==2 && mode_at_init==DWC3_GCTL_PRTCAP_HOST);
    assert(dr_mode_at_init==USB_DR_MODE_HOST);
    puts("ok 2 - subsequent physical-reset session restores HOST before core init");
    clean(); dev.of_node="apple,t8103-dwc3"; stall_reset=false;
    assert(!dwc3_apple_core_init(&a,DWC3_APPLE_HOST) && !role_writes && dctl_writes==1);
    clean(); j713=false; stall_reset=false;
    assert(!dwc3_apple_core_init(&a,DWC3_APPLE_HOST) && !role_writes && dctl_writes==1);
    clean(); dev.host_only=false;
    assert(dwc3_apple_core_init(&a,DWC3_APPLE_DEVICE)==-ETIMEDOUT);
    assert(!role_writes && dctl_writes==1 && releases==1 && a.state==DWC3_APPLE_PROBE_PENDING);
    assert(!a.dwc.current_dr_role);
    puts("ok 3 - older boards and device paths keep original reset and timeout failure");
    clean(); io_ready=true; a.dwc.dev=&dev;
    struct dwc3_probe_data data={ .dwc=&a.dwc, .initial_role=USB_ROLE_HOST };
    assert(dwc3_core_set_initial_role(&data)==-EINVAL && !role_writes);
    data.skip_core_init_mode=true; a.dwc.dr_mode=USB_DR_MODE_PERIPHERAL;
    assert(dwc3_core_set_initial_role(&data)==-EINVAL && !role_writes);
    data.initial_role=USB_ROLE_DEVICE; a.dwc.dr_mode=USB_DR_MODE_HOST;
    assert(dwc3_core_set_initial_role(&data)==-EINVAL && !role_writes);
    data.initial_role=99;
    assert(dwc3_core_set_initial_role(&data)==-EINVAL && !role_writes);
    data.initial_role=USB_ROLE_NONE; data.skip_core_init_mode=false;
    assert(!dwc3_core_set_initial_role(&data) && !role_writes);
    puts("ok 4 - initial-role policy rejects invalid ownership and incompatible modes");
    data.initial_role=USB_ROLE_DEVICE; data.skip_core_init_mode=true;
    a.dwc.dr_mode=USB_DR_MODE_PERIPHERAL;
    assert(!dwc3_core_set_initial_role(&data) && a.dwc.current_dr_role==DWC3_GCTL_PRTCAP_DEVICE);
    assert(dwc3_core_soft_reset(&a.dwc)==-ETIMEDOUT && dctl_writes==1);
    puts("ok 5 - generic device role still performs and reports its required reset");
    clean();
    assert(!dwc3_apple_core_start(&a,DWC3_APPLE_HOST));
    assert(cio_writes==5 && cio[0x38/4]==0x0f800f80 && cio[0x3c/4]==0x0fc00fc0);
    assert(cio[0x40/4]==0xa5140a10 && a.core_active);
    assert(tunable_calls==1 && syscore && a.sleep_syscore && a.rpm_held && rpm_gets==1);
    assert(a.dwc.glue_ops==&dwc3_apple_glue_ops);
    clean(); stop_error=-EBUSY;
    assert(dwc3_apple_core_start(&a,DWC3_APPLE_HOST)==-EBUSY && !cio_writes);
    clean(); dev.host_only=false;
    assert(dwc3_apple_core_start(&a,DWC3_APPLE_DEVICE)==-ETIMEDOUT && !cio_writes);
    clean(); dev.host_only=false; stall_reset=false;
    assert(!dwc3_apple_core_start(&a,DWC3_APPLE_DEVICE) && !cio_writes);
    clean(); j713=false; stall_reset=false;
    assert(!dwc3_apple_core_start(&a,DWC3_APPLE_HOST) && !cio_writes);
    clean(); dev.of_node="apple,t8103-dwc3"; stall_reset=false;
    assert(!dwc3_apple_core_start(&a,DWC3_APPLE_HOST) && cio_writes==5);
    puts("ok 6 - J713 native CIO values after core admission, old and device paths preserved");
    return 0;
}
'''
    role_call = "\tret = dwc3_core_set_initial_role(data);\n\tif (ret)\n\t\tgoto err_free_event_buffers;\n\n"
    assert program.count(role_call)==1
    assert program.count(sequence)==1
    late = program.replace(sequence,sequence.replace(role_call,"")+role_call)
    retry = program.replace("\t\t\tdwc3_apple_set_ptrcap(appledwc, DWC3_GCTL_PRTCAP_HOST);",
                            "\t\t\t(void)0;",1)
    assert retry != program
    no_cio = program.replace(
        ' ||\n\t    dwc3_apple_host_before_core_init(appledwc, state)', '', 1)
    assert no_cio != program
    print("TAP version 13\n1..7",flush=True)
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    with tempfile.TemporaryDirectory(prefix="dwc3-initial-role-") as directory:
        tmp=Path(directory)
        compiler=shlex.split(os.environ.get("HOSTCC","cc"))
        for name,text in [("production",program),("late-initial-role",late),
                          ("missing-retry-role",retry),("missing-native-cio",no_cio)]:
            c=tmp/(name+".c")
            c.write_text(text)
            binary=tmp/name
            options=[] if name=="production" else ["-Wno-unused-function"]
            subprocess.run(compiler+["-std=gnu11","-Wall","-Werror"]+options+
                           [str(c),"-o",str(binary)],check=True)
            result=subprocess.run([str(binary)],capture_output=name!="production")
            if name=="production": result.check_returncode()
            elif result.returncode==0: raise RuntimeError(f"negative control passed: {name}")
        print("ok 7 - late first-role, missing retry-role and native-CIO negative controls rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
