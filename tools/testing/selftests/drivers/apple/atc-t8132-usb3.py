#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production T8132 USB3 sequences against a bounded CPU MMIO model."""
import re
import resource
from pathlib import Path
import subprocess
import tempfile


def function(source, name):
    # The first definition, skipping forward declarations.
    for match in re.finditer(r"^static [^\n]*\b" + name + r"\(", source, re.M):
        rest = source[match.end():]
        if rest.find(";") < 0 or rest.find("{") < rest.find(";"):
            break
    else:
        match = None
    assert match, name
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


def enum(source, name):
    start = source.index("enum " + name + " {")
    return source[start:source.index("};", start) + 2] + "\n"


def defines(source):
    lines = iter(source.splitlines(keepends=True))
    result = []
    for line in lines:
        if line.startswith("#define "):
            result.append(line)
            while line.rstrip().endswith("\\"):
                line = next(lines)
                result.append(line)
    return "".join(result)


def run(tmp, name, program, expect_failure=False):
    path = tmp / (name + ".c")
    path.write_text(program)
    binary = tmp / name
    subprocess.run(["cc", "-std=gnu11", "-Wall", "-Werror", "-Wno-unused-function", str(path), "-o", str(binary)],
                   check=True)
    result = subprocess.run([str(binary)], capture_output=expect_failure)
    if expect_failure:
        if result.returncode == 0:
            raise RuntimeError(f"negative control passed: {name}")
    else:
        result.check_returncode()


def main():
    root = Path(__file__).resolve().parents[5]
    source = root / "drivers/phy/apple/atc.c"
    if not source.exists():
        print("# SKIP: requires the matching kernel source tree")
        return 4
    source = source.read_text()
    print("TAP version 13\n1..13", flush=True)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define GENMASK(h,l) ((~0U << (l)) & (~0U >> (31-(h))))
#define FIELD_PREP(m,v) (((v) << __builtin_ctz(m)) & (m))
#define __iomem
#define guard(x) fake_guard
#define dev_warn(...) ((void)0)
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define READ_ONCE(x) (x)
#define lockdep_assert_held(x) ((void)0)
static void fake_guard(void *p) { (void)p; }
static void delay_event(char type, unsigned n);
static void udelay(unsigned n) { delay_event('u',n); }
static void ndelay(unsigned n) { delay_event('n',n); }
static void fsleep(unsigned n) { (void)n; }
'''
    declarations = r'''
struct apple_tunable { unsigned id; };
struct atcphy_hw { int gen; bool usb2_only, has_usb2phy_reg, park_pipe_unlocked; u32 usb2phy_sig_host; };
enum suspend_state { PM_SUSPEND_TO_IDLE = 1, PM_SUSPEND_MEM = 3 };
static enum suspend_state pm_suspend_target_state;
static bool rpm_suspended;
static unsigned tunnel_stops, dp_stops;
struct device;
struct atcphy_mode_configuration { u32 crossbar, crossbar_dp_single_pma; bool crossbar_dp_both_pma;
    unsigned lane_mode[2]; bool set_swap; };
struct apple_atcphy {
    const struct atcphy_hw *hw;
    struct device *dev;
    bool t8132_ss, t8132_common_owned, t8132_suspended, t8132_failed, host_active;
    bool ss_tunables, mode_dirty, pipe_pending, swap_lanes;
    bool cio_tunables, t8132_dp_alt, fixed_usb2, hibernated, pd_was_off, idle_rpm;
    bool t8103_dummy_pending, t8103_usb3_workaround;
    unsigned tunnel_users;
    struct { bool prepared, changed, restored; } cio_sleep;
    int mode, pipe_state, lock;
    struct { void *regs, *resource; struct apple_tunable *tunable; } t8132_banks[6];
    struct { struct apple_tunable *lane_usb3[2], *usb2phy_reg_dflt, *usb3_dp; } tunables;
    struct { void *core, *usb2phy, *usb2phy_reg, *pipehandler; } regs;
};
enum phy_mode { PHY_MODE_INVALID, PHY_MODE_USB_HOST, PHY_MODE_USB_DEVICE };
struct phy;
struct phy_ops { int (*init)(struct phy *); int (*exit)(struct phy *); int (*power_off)(struct phy *); };
struct phy { struct apple_atcphy *atc; unsigned power_count,init_count,mutex;
    const struct phy_ops *ops;void *pwr; };
struct device { struct apple_atcphy *atc; };
static struct apple_atcphy *dev_get_drvdata(struct device *d) { return d->atc; }
static struct apple_atcphy *phy_get_drvdata(struct phy *p) { return p->atc; }
static u32 core[0x24000/4], usb2[0x4000/4], pipe_regs[0x4000/4];
static unsigned tunes[16], tune_count, reset_releases, bist_phase, writes, core_reads;
static unsigned stall_power, stall_bist, expected_tunes, stall_sleep;
static bool stop_response, stall_shim, stall_rcal, trace;
struct event { char type; u32 off, value, interval, bound; };
static struct event events[512];
static unsigned nevents, release_event;
static void event(char type,u32 off,u32 value,u32 interval,u32 bound)
{
    if (!trace) return;
    assert(nevents<512); events[nevents++]=(struct event){type,off,value,interval,bound};
}
static void delay_event(char type,unsigned n) { event(type,0,n,0,0); }
static void poll_event(const void *p,unsigned interval,unsigned bound)
{
    if ((uintptr_t)p<(uintptr_t)core || (uintptr_t)p>=(uintptr_t)core+sizeof(core)) return;
    unsigned off=(uintptr_t)p-(uintptr_t)core;
    event('p',off,0,interval,bound);
    if(off==AUS_COMMON_DIG_RCAL1) assert(interval==1 && bound==100);
    if(stop_response && off==ACIOPHY_BIST_MAC_USBPHY_STATUS2) assert(interval==1 && bound==200);
    if(stop_response && off==ATCPHY_POWER_STAT) assert(interval==1 && bound==1000);
}
static u32 readl(const void *p)
{
    if ((uintptr_t)p >= (uintptr_t)core && (uintptr_t)p < (uintptr_t)core + sizeof(core))
        core_reads++;
    return *(const u32 *)p;
}
static void writel(u32 value, void *p)
{
    u32 old = *(u32 *)p;
    writes++;
    if ((uintptr_t)p>=(uintptr_t)core && (uintptr_t)p<(uintptr_t)core+sizeof(core))
        event('w',(uintptr_t)p-(uintptr_t)core,value,0,0);
    if (p == core + AUS_COMMON_SHIM_BLK_BIAS_REG/4 && (value&BIT(1)) && !stall_rcal)
        core[AUS_COMMON_DIG_RCAL1/4]=BIT(0);
    if (p == core + ATCPHY_POWER_CTRL/4) {
        core[ATCPHY_POWER_STAT/4] = ((value & 3) & ~stall_power) |
            (stop_response ? core[ATCPHY_POWER_STAT/4] & stall_sleep : 0);
        if ((value & ATCPHY_POWER_PHY_RESET_N) && !(old & ATCPHY_POWER_PHY_RESET_N)) {
            assert(tune_count == expected_tunes);
            reset_releases++; release_event=nevents;
        }
    }
    if (p == usb2 + USB2PHY_SIG/4 && ((old ^ value) & USB2PHY_SIG_HOST_T8132))
        assert(!(usb2[USB2PHY_USBCTL/4] & USB2PHY_USBCTL_RUN));
    if (p == core + ACIOPHY_BIST_CFG1/4 && (value & BIT(2)) && !(old & BIT(2))) {
        bist_phase++;
        if (stop_response)
            core[ACIOPHY_BIST_MAC_USBPHY_STATUS2/4] = stall_shim?0:BIT(7);
        else if (bist_phase == 1)
            core[ACIOPHY_BIST_MAC_USBPHY_STATUS2/4] = stall_bist == 1 ? BIT(7) : 0;
        if (!stop_response && bist_phase == 2)
            core[ACIOPHY_BIST_MAC_USBPHY_STATUS2/4] = stall_bist == 2 ? 0 : BIT(6);
    }
    *(u32 *)p = value;
}
static void mask32(void *p, u32 mask, u32 set) { writel((readl(p) & ~mask) | set, p); }
static void set32(void *p, u32 set) { mask32(p, 0, set); }
static void clear32(void *p, u32 clear) { mask32(p, clear, 0); }
static void core_mask32(struct apple_atcphy *a, u32 off, u32 mask, u32 set)
{ mask32(a->regs.core + off, mask, set); }
static void core_set32(struct apple_atcphy *a, u32 off, u32 set)
{ set32(a->regs.core + off, set); }
static void core_clear32(struct apple_atcphy *a, u32 off, u32 clear)
{ clear32(a->regs.core + off, clear); }
static u32 core_read32(struct apple_atcphy *a, u32 off) { return readl(a->regs.core + off); }
#define readl_poll_timeout(p,v,condition,delay,bound) ({ \
    int poll_ret = -ETIMEDOUT; \
    poll_event((p),(delay),(bound)); \
    for (unsigned poll_i=0; poll_i<4; poll_i++) { \
        (v) = readl(p); if (condition) { poll_ret=0; break; } \
    } poll_ret; })
static int atcphy_disable_dp_aux(struct apple_atcphy *a) { (void)a; assert(0); return -1; }
/* No DP tunnel clock, DP alt mode or CIO runs on this port: the stops are no-ops. */
static int atc_t8122_tunnel_stop(struct apple_atcphy *a) { (void)a; tunnel_stops++; return 0; }
static void atcphy_dp_stop_t8122(struct apple_atcphy *a) { (void)a; dp_stops++; }
static void atcphy_dp_aux_on_t8122(struct apple_atcphy *a) { (void)a; assert(0); }
static void atcphy_t8132_cio_start(struct apple_atcphy *a) { (void)a; assert(0); }
static const struct atcphy_mode_configuration *
atcphy_get_mode_config(struct apple_atcphy *a, enum atcphy_mode m) { (void)a; (void)m; assert(0); return 0; }
static inline bool pm_runtime_status_suspended(struct device *d) { (void)d; return rpm_suspended; }
static inline void atcphy_restore_after_pd_off(struct apple_atcphy *a) { (void)a; }
static void apple_tunable_apply(void *regs, struct apple_tunable *t)
{
    (void)regs;
    if (!t) return;
    assert(!(core[ATCPHY_POWER_CTRL/4] & ATCPHY_POWER_PHY_RESET_N));
    assert(tune_count < 16);
    tunes[tune_count++] = t->id;
}
'''
    names = ["atcphy_mode_is_cio", "atcphy_usb2_sig_host", "atcphy_t8132_dp_on",
             "atcphy_usb2_power_off", "atcphy_power_off", "atcphy_usb2_power_on",
             "atcphy_t8132_cold", "atcphy_t8132_common_start", "atcphy_t8132_phy_stop",
             "atcphy_t8132_full_stop",
             "atcphy_configure_t8132", "atcphy_usb2_set_mode", "atcphy_t8132_bist_pulse",
             "atcphy_t8132_bist_ack", "atcphy_configure_pipehandler_usb3_host_t8132",
             "atcphy_suspend", "atcphy_resume"]
    tests = r'''
static struct atcphy_hw hw = { .gen=ATCPHY_GENERATION_T8132, .usb2_only=true,
                              .usb2phy_sig_host=USB2PHY_SIG_HOST_T8132 };
static struct apple_tunable calibration[9];
static void init(struct apple_atcphy *a)
{
    memset(a,0,sizeof(*a)); memset(core,0,sizeof(core)); memset(usb2,0,sizeof(usb2));
    memset(pipe_regs,0,sizeof(pipe_regs));
    tune_count=reset_releases=bist_phase=writes=core_reads=stall_power=stall_bist=0;
    expected_tunes=7; stop_response=stall_shim=stall_rcal=trace=false;
    stall_sleep=nevents=release_event=0;
    core[ATCPHY_POWER_CTRL/4]=ATCPHY_POWER_CLAMP_EN;
    a->hw=&hw; a->t8132_ss=a->ss_tunables=true;
    a->mode=APPLE_ATCPHY_MODE_OFF; a->pipe_state=ATCPHY_PIPEHANDLER_STATE_DUMMY;
    a->regs.core=core; a->regs.usb2phy=usb2; a->regs.pipehandler=pipe_regs;
    for(unsigned i=0;i<9;i++) calibration[i].id=i;
    for(unsigned i=0;i<6;i++) { a->t8132_banks[i].regs=core; a->t8132_banks[i].tunable=&calibration[i]; }
    a->tunables.lane_usb3[0]=&calibration[6]; a->tunables.lane_usb3[1]=&calibration[7];
}
int main(void)
{
    struct apple_atcphy a;
    struct phy phy = { &a };
    for(unsigned swapped=0; swapped<2; swapped++) {
        init(&a); a.swap_lanes=swapped;
        assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3));
        assert(a.mode==APPLE_ATCPHY_MODE_USB3 && !a.mode_dirty && reset_releases==1);
        assert(a.t8132_common_owned);
        unsigned expected[]={0,1,2,3,4,swapped?7:6,5};
        assert(tune_count==7 && !memcmp(expected,tunes,sizeof(expected)));
        assert((core[0x60/4]&0xfff)==(swapped?0x252:0x489));
        assert((core[0x64/4]&0x1ffff)==(swapped?0x111:0x110));
        assert(!!(core[ATCPHY_MISC/4]&BIT(2))==swapped);
        assert(!atcphy_usb2_set_mode(&phy,PHY_MODE_USB_HOST,0));
        assert(usb2[USB2PHY_USBCTL/4]&USB2PHY_USBCTL_RUN);
        assert((usb2[USB2PHY_SIG/4]&USB2PHY_SIG_HOST_T8132)==USB2PHY_SIG_HOST_T8132);
        assert(!atcphy_usb2_set_mode(&phy,PHY_MODE_USB_HOST,0));
        assert(usb2[USB2PHY_USBCTL/4]&USB2PHY_USBCTL_RUN);
        assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF));
        assert(!a.mode_dirty && a.mode==APPLE_ATCPHY_MODE_OFF && !a.t8132_common_owned);
        assert(core[ATCPHY_POWER_CTRL/4]==4 && !core[ATCPHY_POWER_STAT/4]);
        assert(!(usb2[USB2PHY_USBCTL/4]&USB2PHY_USBCTL_RUN));
    }
    puts("ok 1 - ordered calibration, both routes, role replay and shutdown");
    for(unsigned bit=1; bit<=2; bit++) {
        init(&a); stall_power=bit;
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-ETIMEDOUT);
        assert(a.mode_dirty && a.mode==APPLE_ATCPHY_MODE_OFF && !tune_count && !reset_releases);
        assert(a.t8132_common_owned);
        stall_power=0;
        unsigned touched=writes;
        assert(a.t8132_failed);
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF)==-EIO);
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EIO);
        assert(a.t8132_common_owned && writes==touched);
    }
    init(&a); core[ATCPHY_POWER_CTRL/4]|=BIT(20);
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY);
    assert(a.mode_dirty && !tune_count && !reset_releases && !writes);
    init(&a); core[ATCPHY_POWER_CTRL/4]=0x1b;
    core[ATCPHY_POWER_STAT/4]=3; core[ATCPHY_MISC/4]=1;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY);
    assert(!writes && !a.t8132_common_owned);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB2));
    assert(core[ATCPHY_POWER_CTRL/4]==0x1b && core[ATCPHY_POWER_STAT/4]==3);
    puts("ok 2 - startup timeouts and unknown inherited state fail closed");
    init(&a); a.pipe_pending=true;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY && !writes);
    init(&a); a.pipe_state=ATCPHY_PIPEHANDLER_STATE_USB3;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY && !writes);
    init(&a); a.ss_tunables=false;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EOPNOTSUPP && !writes);
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_DP)==-EOPNOTSUPP && !writes);
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB4)==-EOPNOTSUPP && !writes);
    puts("ok 3 - live PIPE, incomplete calibration and unsupported modes refused");
    init(&a);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB2));
    assert(!core_reads && !a.t8132_common_owned);
    init(&a); a.t8132_ss=a.ss_tunables=false;
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB2));
    assert(a.mode==APPLE_ATCPHY_MODE_USB2 && !tune_count && !reset_releases && !core_reads);
    assert(!atcphy_usb2_set_mode(&phy,PHY_MODE_USB_HOST,0));
    assert(usb2[USB2PHY_USBCTL/4]&USB2PHY_USBCTL_RUN);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF));
    puts("ok 4 - legacy USB2-only topology remains functional");
    init(&a);
    assert(!atcphy_configure_pipehandler_usb3_host_t8132(&a));
    assert(bist_phase==3 && (core[ACIOPHY_BIST_MAC_USBPHY_STATUS2/4]&BIT(6)));
    assert(!(core[ACIOPHY_BIST_CFG1/4] & (BIT(0)|BIT(2))));
    assert((pipe_regs[PIPEHANDLER_NONSELECTED_OVERRIDE/4]&15)==3);
    for(unsigned phase=1;phase<=2;phase++) {
        init(&a);stall_bist=phase;
        assert(atcphy_configure_pipehandler_usb3_host_t8132(&a)==-ETIMEDOUT);
        assert(bist_phase==phase);
        assert(!(pipe_regs[PIPEHANDLER_NONSELECTED_OVERRIDE/4]&15));
    }
    puts("ok 5 - T8132 BIST bit-6 success and both failed transitions");
    struct device dev = { &a };
    init(&a);
    assert(!atcphy_suspend(&dev) && a.t8132_suspended);
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY);
    assert(!writes && !core_reads && !a.t8132_common_owned && !a.mode_dirty);
    assert(!atcphy_resume(&dev) && !a.t8132_suspended);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3));
    assert(atcphy_suspend(&dev)==-EBUSY && !a.t8132_suspended);
    /* A running host keeps the epoch through suspend-to-idle only (cda0923e3d56). */
    a.host_active=true; pm_suspend_target_state=PM_SUSPEND_MEM;
    assert(atcphy_suspend(&dev)==-EBUSY && !a.t8132_suspended);
    pm_suspend_target_state=PM_SUSPEND_TO_IDLE;
    unsigned kept=writes;
    assert(!atcphy_suspend(&dev) && a.t8132_suspended && a.t8132_common_owned);
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-EBUSY && writes==kept);
    assert(!atcphy_resume(&dev) && !a.t8132_suspended && a.t8132_common_owned);
    a.host_active=false; pm_suspend_target_state=0;
    assert(atcphy_suspend(&dev)==-EBUSY && !a.t8132_suspended);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF));
    assert(!a.t8132_common_owned && !a.mode_dirty && !atcphy_suspend(&dev));
    init(&a); stall_power=1;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3)==-ETIMEDOUT);
    assert(a.t8132_common_owned && a.mode_dirty);
    assert(atcphy_suspend(&dev)==-EBUSY);
    init(&a); a.mode_dirty=true;
    assert(atcphy_suspend(&dev)==-EBUSY && !a.t8132_common_owned);
    init(&a); a.t8132_ss=a.ss_tunables=false; a.mode_dirty=true;
    assert(!atcphy_suspend(&dev) && !a.t8132_suspended);
    struct atcphy_hw old_hw=hw; old_hw.gen=ATCPHY_GENERATION_T8103;
    init(&a); a.hw=&old_hw; a.t8132_common_owned=a.mode_dirty=true;
    assert(!atcphy_suspend(&dev) && !a.t8132_suspended);
    puts("ok 6 - sleep refuses active/failed M4 common epochs and prevents late admission");
    return 0;
}
'''
    program = preamble + defines(source)
    for name in ["atcphy_generation", "atcphy_mode", "atcphy_pipehandler_state", "atcphy_t8132_bank"]:
        program += enum(source, name)
    program += declarations + "\n".join(function(source, name) for name in names) + tests
    with tempfile.TemporaryDirectory(prefix="atc-t8132-usb3-") as directory:
        run(Path(directory), "atc-t8132", program)
        dwc = (root / "drivers/usb/dwc3/dwc3-apple.c").read_text()
        states = (root / "drivers/usb/dwc3/dwc3-apple-state.h").read_text()
        role_program = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define lockdep_assert_held(x) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_err(...) ((void)0)
#define WARN_ON_ONCE(x) ((void)(x))
#define USB_DR_MODE_HOST 1
#define USB_DR_MODE_PERIPHERAL 2
#define DWC3_GCTL_PRTCAP_HOST 1
#define DWC3_GCTL_PRTCAP_DEVICE 2
enum phy_mode { PHY_MODE_USB_HOST, PHY_MODE_USB_DEVICE };
struct device { const char *of_node; };
struct dwc3 { int dr_mode, current_dr_role; bool dis_u3_susphy_quirk; };
struct dwc3_apple { struct dwc3 dwc; struct device *dev; void *usb2_phy, *usb3_phy; int lock, state;
    bool dt_dis_u3_susphy, host_only, sleep_ddr; };
static int phy_error, host_calls, gadget_calls, stops, syscore;
static bool of_device_is_compatible(const char *node, const char *name) { return !strcmp(node,name); }
static int phy_set_mode(void *phy, int mode) { (void)phy; (void)mode; return phy_error; }
static void dwc3_apple_set_ptrcap(struct dwc3_apple *a, int cap) { (void)a; (void)cap; }
static void dwc3_enable_susphy(struct dwc3 *d, bool on) { (void)d; assert(on); }
static int dwc3_host_init(struct dwc3 *d) { (void)d; host_calls++; return 0; }
static int dwc3_gadget_init(struct dwc3 *d) { (void)d; gadget_calls++; return 0; }
static int phy_reset(void *phy) { (void)phy; return 0; }
static void dwc3_host_exit(struct dwc3 *d) { (void)d; assert(0); }
static void apple_sleep_usb_slp_ddr(bool on) { (void)on; assert(0); }
static void dwc3_apple_usb3_pipe_reset(struct dwc3_apple *a) { (void)a; assert(0); }
''' + enum(states, "dwc3_apple_state") + r'''
static int dwc3_apple_core_start(struct dwc3_apple *a, enum dwc3_apple_state s) { (void)a; (void)s; return 0; }
static int dwc3_apple_core_stop(struct dwc3_apple *a) { (void)a; stops++; return 0; }
static void dwc3_apple_syscore(struct dwc3_apple *a, bool on) { (void)a; assert(on); syscore++; }
''' + function(dwc, "dwc3_apple_set_role") + function(dwc, "dwc3_apple_init") + r'''
int main(void)
{
    struct device dev = { "apple,t8132-dwc3" };
    struct dwc3_apple a = { .dev=&dev, .usb3_phy=&dev, .state=DWC3_APPLE_NO_CABLE };
    phy_error=-ETIMEDOUT;
    assert(dwc3_apple_init(&a,DWC3_APPLE_HOST)==-ETIMEDOUT);
    assert(!host_calls && !gadget_calls && stops==1 && a.state==DWC3_APPLE_NO_CABLE);
    phy_error=-EOPNOTSUPP;
    assert(dwc3_apple_init(&a,DWC3_APPLE_DEVICE)==-EOPNOTSUPP);
    assert(!gadget_calls && stops==2);
    phy_error=0;
    assert(!dwc3_apple_init(&a,DWC3_APPLE_HOST));
    assert(host_calls==1 && a.state==DWC3_APPLE_HOST && syscore==1);
    /* A T8132 host keeps its USB3 PHY out of P3. */
    assert(a.dwc.dis_u3_susphy_quirk);
    dev.of_node="apple,t8103-dwc3"; phy_error=-ETIMEDOUT;
    assert(!dwc3_apple_init(&a,DWC3_APPLE_HOST) && host_calls==2 && syscore==2);
    assert(!a.dwc.dis_u3_susphy_quirk);
    puts("ok 7 - T8132 PHY failures stop before xHCI, older fallback preserved");
    return 0;
}
'''
        run(Path(directory), "dwc3-t8132", role_program)
        permissive = program.replace(
            "(!atcphy->host_active || pm_suspend_target_state != PM_SUSPEND_TO_IDLE)", "false", 1)
        assert permissive != program
        run(Path(directory), "suspend-deep-negative-control", permissive, expect_failure=True)
        permissive = program.replace(
            "if (atcphy->mode_dirty)\n\t\treturn -EBUSY;", "if (false)\n\t\treturn -EBUSY;", 1)
        assert permissive != program
        run(Path(directory), "suspend-negative-control", permissive, expect_failure=True)
        late_admit = program.replace(
            "atcphy->t8132_suspended)",
            "false)", 1)
        assert late_admit != program
        run(Path(directory), "late-admission-negative-control", late_admit, expect_failure=True)
        print("ok 8 - permissive suspend and late common-admission negative controls rejected",
              flush=True)
        mux = (root / "drivers/usb/typec/mux.c").read_text()
        # The public entry point is not static; use the same function parser.
        mux = mux.replace("\nint typec_mux_apply(", "\nstatic int typec_mux_apply(")
        mux_program = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#define WARN_ON_ONCE(x) (x)
#define READ_ONCE(x) (x)
#define current ((void *)1)
#define fallthrough __attribute__((fallthrough))
''' + enum(source, "atcphy_generation") + enum(source, "atcphy_mode") + r'''
struct atcphy_hw { int gen; bool has_usb4, has_t8140_dp, dp_t8122; };
struct apple_atcphy {
    const struct atcphy_hw *hw;
    bool dp_only, dp_tunables, ss_tunables, cio_tunables, t8132_dp_alt;
    struct { void *usb3_dp; } tunables;
};
struct { bool enable_dp_aux; } atcphy_modes[APPLE_ATCPHY_MODE_DP + 1];
''' + function(source, "atcphy_mode_is_cio") + function(source, "atcphy_t8132_dp_on") + \
    function(source, "atcphy_mux_check") + r'''
struct typec_mux_dev {
    int (*set)(struct typec_mux_dev *, int *);
    bool transient_timeouts;
    struct apple_atcphy *atc;
};
struct typec_mux {
    void *owner;
    bool applied, timeout_transient, aborted;
    unsigned int num_mux_devs;
    struct typec_mux_dev **mux_devs;
    int transaction;
};
static bool is_error_or_null(void *p) { return !p; }
#define IS_ERR_OR_NULL(p) is_error_or_null(p)
static void typec_mux_abort(struct typec_mux *mux) { mux->aborted = true; }
static int set(struct typec_mux_dev *dev, int *mode)
{ return atcphy_mux_check(dev->atc, *mode); }
''' + function(mux, "typec_mux_apply") + r'''
int main(void)
{
    struct atcphy_hw hw = { .gen = ATCPHY_GENERATION_T8132 };
    struct apple_atcphy atc = { .hw = &hw, .ss_tunables = true };
    struct typec_mux_dev dev = { .set = set, .atc = &atc };
    struct typec_mux_dev *providers[] = { &dev };
    for (int mode = APPLE_ATCPHY_MODE_OFF; mode <= APPLE_ATCPHY_MODE_DP; mode++) {
        struct typec_mux mux = { .owner = current, .num_mux_devs = 1,
                                .mux_devs = providers, .transaction = mode };
        int ret = typec_mux_apply(&mux);
        bool supported = mode == APPLE_ATCPHY_MODE_OFF ||
                         mode == APPLE_ATCPHY_MODE_USB2 || mode == APPLE_ATCPHY_MODE_USB3;
        if (supported) {
            assert(!ret && mux.applied && !mux.aborted);
        } else {
            assert(ret == -ENODEV && !mux.applied && mux.aborted);
        }
    }
    /* CIO and 4-lane DP alt mode are admitted only with their calibration. */
    atc.cio_tunables = true;
    assert(!atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_TBT));
    assert(!atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_USB4));
    assert(atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_DP) == -ENODEV);
    atc.t8132_dp_alt = true;
    assert(atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_DP) == -ENODEV);
    assert(atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_USB3_DP) == -ENODEV);
    atc.tunables.usb3_dp = &atc;
    assert(!atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_DP));
    assert(!atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_USB3_DP));
    atc.ss_tunables = false;
    assert(atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_DP) == -ENODEV);
    atc.cio_tunables = atc.t8132_dp_alt = false; atc.ss_tunables = true; atc.tunables.usb3_dp = 0;
    /* Preserve unrelated older providers' EOPNOTSUPP semantics. */
    hw.gen = ATCPHY_GENERATION_T8103;
    assert(atcphy_mux_check(&atc, APPLE_ATCPHY_MODE_USB4) == -EOPNOTSUPP);
    puts("ok 9 - unsupported M4 lane modes reach the Type-C consumer as failures");
    return 0;
}
'''
        run(Path(directory), "typec-atc-admission", mux_program)
        ambiguous = mux_program.replace("return -ENODEV;", "return -EOPNOTSUPP;", 1)
        assert ambiguous != mux_program
        run(Path(directory), "typec-atc-errno-negative-control", ambiguous,
            expect_failure=True)
        mode4_tests = tests[:tests.index("int main(void)")] + r'''
static void expect_trace(unsigned start,const struct event *expected,unsigned count)
{
    assert(nevents>=start+count);
    for(unsigned i=0;i<count;i++) {
        const struct event *got=&events[start+i], *want=&expected[i];
        if(got->type!=want->type || got->off!=want->off || got->value!=want->value ||
           got->interval!=want->interval || got->bound!=want->bound) {
            fprintf(stderr,"trace %u: %c/%x/%x/%u/%u wanted %c/%x/%x/%u/%u\n",i,
                    got->type,got->off,got->value,got->interval,got->bound,
                    want->type,want->off,want->value,want->interval,want->bound);
            assert(0);
        }
    }
}
#define W(o,v) {'w',o,v,0,0}
#define U(v) {'u',0,v,0,0}
#define N(v) {'n',0,v,0,0}
#define P(o,i,b) {'p',o,0,i,b}
static void mode4_init(struct apple_atcphy *a,bool flipped)
{
    init(a);a->swap_lanes=flipped; a->tunables.usb3_dp=&calibration[8];
    expected_tunes=8;
}
int main(void)
{
    struct apple_atcphy a; struct phy phy={&a};
    for(unsigned flipped=0;flipped<2;flipped++) {
        mode4_init(&a,flipped); trace=true;
        core[ACIOPHY_CFG0/4]=0x80000010;
        assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3_DP));
        assert(a.mode==APPLE_ATCPHY_MODE_USB3_DP && a.t8132_common_owned && !a.mode_dirty);
        unsigned expected[]={0,1,2,3,4,flipped?7:6,8,5};
        assert(tune_count==8 && !memcmp(expected,tunes,sizeof(expected)));
        const struct event wake[]={U(100),W(8,0x80000014),W(8,0x8000001c),U(1),
            W(8,0x8000001d),W(8,0x8000001f),U(1),W(8,0x8000000f),W(8,0x8000002f),U(1),
            W(0xa00,2),U(15),P(0x804,1,100)};
        expect_trace(release_event,wake,sizeof(wake)/sizeof(wake[0]));
        assert(!atcphy_usb3_exit(&phy)); /* unpowered init lease */
        phy.power_count=1;
        assert(atcphy_usb3_exit(&phy)==-EBUSY);
        unsigned before=writes; a.host_active=true;
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF)==-EBUSY && writes==before);
        assert(!a.mode_dirty);a.host_active=false; stop_response=true;
        unsigned first=nevents;
        assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF));
        const struct event stop[]={W(0x17000,1),W(0x17088,0x04000000),W(0x1708c,0x220),
            W(0x17000,5),W(0x17000,1),P(0x17080,1,200),W(0x17000,0),
            W(0xa00,0),U(1),W(8,0x8000003f),U(1),W(8,0x8000003e),U(1),W(8,0x8000003a),U(1),
            W(0x10c,0x300),U(1),W(0x10c,0x700),U(1),W(0x10c,0x704),U(1),W(0x10c,0x744),U(1),
            W(0x10c,0xf44),U(1),W(0x10c,0xf4c),U(1),W(0x10c,0xfcc),U(1),
            W(0x20000,0xb),U(1),W(0x20000,3),N(1),W(0x20000,7),N(1),
            W(0x20000,5),P(0x20004,1,1000),W(0x20000,4),P(0x20004,1,1000),N(1),
            W(0x20008,flipped?4:0),N(1)};
        expect_trace(first,stop,sizeof(stop)/sizeof(stop[0]));
        assert(!a.t8132_common_owned && !a.t8132_failed && !a.mode_dirty);
        assert(core[ATCPHY_MISC/4]==(flipped?4U:0U));
        phy.power_count=0;
        assert(!atcphy_usb3_exit(&phy));
        /* A native flipped full-off admits the next epoch without clearing orientation. */
        stop_response=false; trace=false; tune_count=0; a.swap_lanes=!flipped;
        assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3_DP));
        assert(!!(core[ATCPHY_MISC/4]&4)==!flipped);
    }
    /* Missing optional metadata never admits mode4, while mode1 stays usable. */
    init(&a);unsigned before=writes;
    assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3_DP)==-ENODEV && writes==before);
    assert(!atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3));
    /* Every touched failure remains owned and cannot be replayed as a full stop. */
    for(unsigned fault=0;fault<4;fault++) {
        mode4_init(&a,false);
        if(!fault) stall_rcal=true;
        int ret=atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3_DP);
        if(fault) {
            assert(!ret);stop_response=true;
            if(fault==1) stall_shim=true;
            else stall_sleep=fault==2?2:1;
            ret=atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF);
        }
        assert(ret==-ETIMEDOUT && a.t8132_common_owned && a.t8132_failed && a.mode_dirty);
        before=writes;
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_OFF)==-EIO);
        assert(atcphy_configure_t8132(&a,APPLE_ATCPHY_MODE_USB3_DP)==-EIO);
        phy.power_count=fault?1:0;
        assert(atcphy_usb3_exit(&phy)==(fault?-EBUSY:0) && writes==before);
        assert(a.t8132_common_owned && a.t8132_failed);
        struct device d={&a};assert(atcphy_suspend(&d)==-EBUSY);
    }
    puts("ok 10 - native mode4 ordered wake/full-off, flipped readmission and latched failures");
    return 0;
}
'''
        mode4_program = preamble + defines(source)
        for name in ["atcphy_generation", "atcphy_mode", "atcphy_pipehandler_state", "atcphy_t8132_bank"]:
            mode4_program += enum(source,name)
        mode4_names = [name for name in names if name not in
                       {"atcphy_usb2_set_mode", "atcphy_usb2_sig_host", "atcphy_resume", "atcphy_t8132_bist_pulse",
                        "atcphy_t8132_bist_ack", "atcphy_configure_pipehandler_usb3_host_t8132"}]
        mode4_program += declarations + "\n".join(function(source,name) for name in mode4_names)
        mode4_program += function(source,"atcphy_usb3_exit") + mode4_tests
        run(Path(directory),"atc-native-mode4",mode4_program)
        controls = {
            "missing-aux": mode4_program.replace(
                "apple_tunable_apply(atcphy->regs.core, atcphy->tunables.usb3_dp);", "(void)0;",1),
            "long-rcal": mode4_program.replace(
                "reg & AUS_COMMON_DIG_RCAL1_ALL_CODES_DONE, 1, 100);",
                "reg & AUS_COMMON_DIG_RCAL1_ALL_CODES_DONE, 1, 1000);",1),
            "missing-control": mode4_program.replace(
                "core_set32(atcphy, ACIOPHY_T8132_COMMON_CTRL, 0xc0);", "(void)0;",1),
            "unlatched-failure": mode4_program.replace("atcphy->t8132_failed = true;", "atcphy->t8132_failed = false;"),
        }
        for name,broken in controls.items():
            assert broken != mode4_program,name
            run(Path(directory),"mode4-negative-"+name,broken,expect_failure=True)
        shutdown = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#define BIT(n) (1U<<(n))
#define guard(x) fake_guard
#define lockdep_assert_held(x) ((void)0)
#define WARN_ON_ONCE(x) ((void)(x))
#define dev_err(...) ((void)0)
#define GFP_KERNEL 0
#define USB_ROLE_HOST 1
#define USB_ROLE_NONE 0
#define USB_ROLE_DEVICE 2
#define DWC3_DEFAULT_PROPERTIES ((struct props){0})
''' + enum(states,"dwc3_apple_state") + enum(source,"atcphy_generation") + enum(source,"atcphy_pipehandler_state") + r'''
struct device { const char *of_node; void *data; };
struct platform_device { struct device dev; };
struct phy { unsigned type; };
struct dwc3 {
    struct device *dev;unsigned num_usb3_ports,num_usb2_ports;
    struct phy *usb3_generic_phy[1],*usb2_generic_phy[1];void *usb3_phy,*usb2_phy;
    const void *glue_ops;bool dis_u3_susphy_quirk;
};
struct dwc3_apple {
    struct dwc3 dwc; struct device *dev; void *mmio_resource,*reset,*usb2_phy;
    int lock,state; bool core_active,core_exit_pending,reset_pending,shutting_down,fixed_hub;
    unsigned usb2_power_off_pending,usb3_power_off_pending,hub_port;
    bool host_only,idle_rpm,rpm_held,dt_dis_u3_susphy,sleep_ddr,sleep_in_place;
    int sleep_pm_lock,sleep_start_work,sleep_stop_work,hub_check;
};
#define to_dwc3_apple(d) ((struct dwc3_apple *)(d))
struct props { bool xhci_run_in_s2idle; };
struct dwc3_probe_data { struct dwc3 *dwc; void *res; bool ignore_clocks_and_resets,skip_core_init_mode;
    int initial_role;struct props properties; };
struct atcphy_hw { int gen; };
struct apple_atcphy {
    const struct atcphy_hw *hw;struct device *dev;int lock,pipe_state;
    bool t8132_ss,t8132_suspended,t8132_common_owned,t8132_failed,host_active,pipe_pending;
};
#define APPLE_ATCPHY_MODE_OFF 0
static struct apple_atcphy atc;
static unsigned count,stage[64],fail_phy,reset_calls,provider_stops,failed_probe;
static void event(unsigned n) { assert(count<64);stage[count++]=n; }
static void fake_guard(void *p) { (void)p; }
static bool of_device_is_compatible(const char *node,const char *name) { return !strcmp(node,name); }
static bool dwc3_apple_host_before_core_init(struct dwc3_apple *a,int state)
{ return state==DWC3_APPLE_HOST && of_device_is_compatible(a->dev->of_node,"apple,t8132-dwc3"); }
static void *platform_get_drvdata(struct platform_device *p) { return p->dev.data; }
static void platform_set_drvdata(struct platform_device *p,void *data) { p->dev.data=data; }
static void dev_set_drvdata(struct device *d,void *data) { d->data=data; }
static void dwc3_enable_susphy(struct dwc3 *d,bool on) { (void)d;assert(on);event(2); }
static void dwc3_event_buffers_cleanup(struct dwc3 *d) { (void)d;event(3); }
static int phy_power_off(struct phy *p) {
    event(p->type?5:4); if(fail_phy && !p->type) { atc.t8132_failed=true;return -ETIMEDOUT; }
    if(!p->type) { assert(!atc.host_active);atc.t8132_common_owned=false; }
    return 0;
}
static void usb_phy_set_suspend(void *p,int v) { (void)p;assert(v==1); }
static void dwc3_core_exit_after_phy_power_off(struct dwc3 *d) {
    (void)d;assert(!atc.t8132_common_owned);event(6);
}
static int reset_control_assert(void *p) { (void)p;reset_calls++;event(7);return 0; }
static unsigned rpm_puts,cancels;
static const int dwc3_apple_glue_ops;
static void pm_runtime_get_noresume(struct device *d) { (void)d;assert(0); }
static void pm_runtime_allow(struct device *d) { (void)d;assert(0); }
static void pm_runtime_mark_last_busy(struct device *d) { (void)d; }
static void pm_runtime_put_autosuspend(struct device *d) { (void)d;rpm_puts++; }
static void dwc3_apple_syscore(struct dwc3_apple *a,bool on) { (void)a;assert(!on); }
static void apple_sleep_usb_slp_ddr(bool on) { (void)on;assert(0); }
#define scoped_guard(t,l) for (int scoped_once=1; scoped_once; scoped_once=0)
static void devm_release_action(struct device *d,void *f,void *data) { (void)d;(void)f;(void)data; }
static void dwc3_apple_pm_notifier_remove(void *data) { (void)data; }
static void cancel_work_sync(int *w) { (void)w;cancels++; }
static void cancel_delayed_work_sync(int *w) { (void)w;cancels++; }
static void apple_sleep_usb_set_wake_ports(unsigned m) { (void)m;assert(0); }
static void apple_atcphy_sleep_finish_async(void *p,bool ok) { (void)p;(void)ok;assert(0); }
static void dwc3_host_exit(struct dwc3 *d) { (void)d;atc.host_active=false;event(1); }
static void dwc3_gadget_exit(struct dwc3 *d) { (void)d;assert(0); }
static int atcphy_configure_t8132(struct apple_atcphy *a,int mode) {
    assert(mode==APPLE_ATCPHY_MODE_OFF && !a->host_active && !a->t8132_failed);
    provider_stops++;a->t8132_common_owned=false;return 0;
}
static void *devres_open_group(struct device *d,void *id,int f) { (void)id;(void)f;return d; }
static void devres_release_group(struct device *d,void *g) { (void)d;(void)g; }
static void devres_remove_group(struct device *d,void *g) { (void)d;(void)g; }
static int dwc3_core_probe(struct dwc3_probe_data *d) {
    if(failed_probe) { d->dwc->dev->data=NULL;return -EIO; }
    return 0;
}
''' + function(dwc,"dwc3_apple_core_probe") + function(dwc,"dwc3_apple_core_stop") + function(dwc,"dwc3_apple_exit") + function(dwc,"dwc3_apple_pm_quiesce") + function(dwc,"dwc3_apple_shutdown") + function(source,"atcphy_shutdown")
        # Execute the actual early glue-probe prefix through stable owner publication.
        probe = function(dwc,"dwc3_apple_probe")
        prefix = probe[probe.index("\tappledwc->dev ="):probe.index("\n\tappledwc->reset =")]
        shutdown += r'''
static void mutex_init(int *p) { *p=0; }
static bool dwc3_apple_has_fixed_hub(struct device *d,unsigned *port) { (void)d;(void)port;return false; }
static bool dwc3_apple_host_only(struct device *d) { (void)d;return false; }
static bool device_property_read_bool(struct device *d,const char *name) { (void)d;(void)name;return false; }
static void spin_lock_init(int *p) { *p=0; }
static void publish_owner(struct platform_device *pdev,struct dwc3_apple *appledwc)
{
    struct device *dev=&pdev->dev;
''' + prefix + r'''
}
static struct atcphy_hw hw={ATCPHY_GENERATION_T8132};
static struct phy ss={0},hs={1};
static void init(struct platform_device *p,struct dwc3_apple *a)
{
    memset(p,0,sizeof(*p));memset(a,0,sizeof(*a));
    p->dev.of_node="apple,t8132-dwc3";publish_owner(p,a);
    a->state=DWC3_APPLE_PROBE_PENDING;a->dwc.num_usb3_ports=a->dwc.num_usb2_ports=1;
    a->dwc.usb3_generic_phy[0]=&ss;a->dwc.usb2_generic_phy[0]=&hs;
    atc=(struct apple_atcphy){.hw=&hw,.t8132_ss=true,.pipe_state=ATCPHY_PIPEHANDLER_STATE_DUMMY};
    count=fail_phy=reset_calls=provider_stops=failed_probe=rpm_puts=cancels=0;
}
int main(void)
{
    struct platform_device p,supplier;struct dwc3_apple a;
    for(unsigned first_failure=0;first_failure<2;first_failure++) {
        init(&p,&a);assert(platform_get_drvdata(&p)==&a.dwc);
        if(first_failure) {
            failed_probe=1;assert(dwc3_apple_core_probe(&a,DWC3_APPLE_HOST)==-EIO);
            assert(platform_get_drvdata(&p)==&a.dwc);
        }
        dwc3_apple_shutdown(&p);assert(a.shutting_down && !count && !reset_calls && cancels==3);
    }
    init(&p,&a);a.state=DWC3_APPLE_HOST;a.core_active=a.reset_pending=true;
    a.usb3_power_off_pending=a.usb2_power_off_pending=1;
    atc.t8132_common_owned=atc.host_active=true;a.rpm_held=true;
    dwc3_apple_shutdown(&p);
    unsigned expected[]={1,2,3,4,5,6,7};
    assert(count==7 && !memcmp(stage,expected,sizeof(expected)));
    assert(!a.rpm_held && rpm_puts==1);
    assert(!a.core_active && !a.reset_pending && a.state==DWC3_APPLE_NO_CABLE);
    supplier.dev.data=&atc;atcphy_shutdown(&supplier);
    assert(atc.t8132_suspended && !provider_stops && !atc.t8132_common_owned);
    /* A failed PHY release never reaches core exit or controller reset. */
    init(&p,&a);a.state=DWC3_APPLE_HOST;a.core_active=a.reset_pending=true;
    a.usb3_power_off_pending=a.usb2_power_off_pending=1;
    atc.t8132_common_owned=atc.host_active=true;fail_phy=1;
    dwc3_apple_shutdown(&p);
    assert(count==4 && stage[0]==1 && stage[3]==4 && !reset_calls);
    assert(atc.t8132_common_owned && atc.t8132_failed && a.core_active && a.reset_pending);
    supplier.dev.data=&atc;atcphy_shutdown(&supplier);assert(!provider_stops);
    /* Provider may finish a successful configured epoch with no consumer. */
    init(&p,&a);atc.t8132_common_owned=true;supplier.dev.data=&atc;
    atcphy_shutdown(&supplier);assert(provider_stops==1 && !atc.t8132_common_owned);
    for(unsigned blocked=0;blocked<4;blocked++) {
        init(&p,&a);atc.t8132_common_owned=true;supplier.dev.data=&atc;
        if(blocked==0) atc.t8132_failed=true;
        if(blocked==1) atc.host_active=true;
        if(blocked==2) atc.pipe_pending=true;
        if(blocked==3) atc.pipe_state=ATCPHY_PIPEHANDLER_STATE_USB3;
        atcphy_shutdown(&supplier);assert(!provider_stops && atc.t8132_common_owned);
    }
    /* Other controllers are quiesced (a6671210b88c), never torn down. */
    init(&p,&a);p.dev.of_node="apple,t8103-dwc3";
    dwc3_apple_shutdown(&p);assert(a.shutting_down && !count && !reset_calls);
    puts("ok 11 - consumer/provider shutdown ordering, probe-pending lifetime and retained failures");
    return 0;
}
'''
        run(Path(directory),"usb-native-shutdown",shutdown)
        missing_owner=shutdown.replace("platform_set_drvdata(pdev, &appledwc->dwc);","(void)0;",1)
        missing_restore=shutdown.replace("dev_set_drvdata(appledwc->dev, &appledwc->dwc);","(void)0;",1)
        unsafe_release=shutdown.replace("if (ret)\n\t\t\t\treturn ret;","if (false)\n\t\t\t\treturn ret;",1)
        for name,broken in {"missing-owner":missing_owner,"missing-restore":missing_restore,
                            "unsafe-release":unsafe_release}.items():
            assert broken!=shutdown,name
            # The removed setter becomes unused in these deliberate mutants.
            broken=broken.replace("static void platform_set_drvdata", "static inline void platform_set_drvdata")
            broken=broken.replace("static void dev_set_drvdata", "static inline void dev_set_drvdata")
            run(Path(directory),"shutdown-negative-"+name,broken,expect_failure=True)
        framework = (root / "drivers/phy/phy-core.c").read_text()
        for name in ["phy_init", "phy_exit", "phy_power_off"]:
            framework = framework.replace("\nint "+name+"(", "\nstatic int "+name+"(")
        counters = mode4_program[:mode4_program.index("static void expect_trace(")]
        counters += r'''
#define ENOTSUPP 524
#define mutex_lock(x) ((void)(x))
#define mutex_unlock(x) ((void)(x))
static int phy_pm_runtime_get_sync(struct phy *p) { (void)p;return 0; }
static void phy_pm_runtime_put(struct phy *p) { (void)p; }
static void regulator_disable(void *p) { (void)p; }
static bool release_fails;
static unsigned release_calls;
static int fail_release(struct phy *p) {
    release_calls++;
    if(release_fails) {p->atc->t8132_failed=true;return -ETIMEDOUT;}
    p->atc->t8132_common_owned=false;return 0;
}
#define dev_dbg(...) ((void)0)
static unsigned park_calls;
static int atcphy_configure_pipehandler_dummy(struct apple_atcphy *a,bool lock)
{ (void)a;(void)lock;park_calls++;return -ETIMEDOUT; }
/* The M1 live-PIPE workaround never applies to T8132. */
static int atcphy_park_live_t8103_pipehandler(struct apple_atcphy *a) { (void)a;assert(0);return -1; }
''' + "\n".join(function(framework,name) for name in ["phy_init","phy_exit","phy_power_off"]) + function(source,"atcphy_park_pipehandler") + r'''
int main(void)
{
    (void)atcphy_configure_t8132;(void)atcphy_suspend;
    struct apple_atcphy a; init(&a);
    struct phy_ops ops={.exit=atcphy_usb3_exit,.power_off=fail_release};
    struct phy p={.atc=&a,.ops=&ops};
    a.t8132_common_owned=true;
    /* Early core reset fails before power_on: drop only the unpowered init lease. */
    for(unsigned retry=0;retry<3;retry++) {
        assert(!phy_init(&p) && p.init_count==1 && !p.power_count);
        assert(!phy_exit(&p) && !p.init_count && !p.power_count);
        assert(a.t8132_common_owned && !a.t8132_failed);
    }
    assert(!phy_init(&p));p.power_count=1;release_fails=true;
    assert(phy_power_off(&p)==-ETIMEDOUT && p.power_count==1);
    assert(phy_exit(&p)==-EBUSY && p.init_count==1 && p.power_count==1);
    assert(phy_exit(&p)==-EBUSY && p.init_count==1);
    assert(a.t8132_common_owned && a.t8132_failed);
    for(unsigned fail=0;fail<2;fail++) {
        init(&a);a.t8132_common_owned=true;
        p=(struct phy){.atc=&a,.ops=&ops};release_calls=0;release_fails=fail;
        assert(!phy_init(&p) && !phy_init(&p) && p.init_count==2);p.power_count=2;
        assert(!phy_power_off(&p) && p.power_count==1 && !release_calls);
        assert(!phy_exit(&p) && p.init_count==1 && a.t8132_common_owned);
        if(fail) {
            assert(phy_power_off(&p)==-ETIMEDOUT && p.power_count==1 && release_calls==1);
            assert(phy_exit(&p)==-EBUSY && p.init_count==1 && a.t8132_common_owned);
        } else {
            assert(!phy_power_off(&p) && !p.power_count && release_calls==1);
            assert(!phy_exit(&p) && !p.init_count && !a.t8132_common_owned);
        }
    }
    init(&a);a.t8132_common_owned=true;a.pipe_state=ATCPHY_PIPEHANDLER_STATE_USB3;
    assert(atcphy_park_pipehandler(&a,false)==-ETIMEDOUT);
    assert(a.t8132_failed && a.mode_dirty && a.t8132_common_owned && park_calls==1);
    assert(atcphy_park_pipehandler(&a,false)==-EIO && park_calls==1);
    puts("ok 12 - production PHY counters, early init retry, failed release and direct park retention");
    return 0;
}
'''
        run(Path(directory),"phy-native-counters",counters)
        leaked_init=counters.replace(" || !phy->power_count", "",1)
        assert leaked_init!=counters
        run(Path(directory),"phy-counter-negative-init-leak",leaked_init,expect_failure=True)
        early_release=counters.replace("phy->power_count == 1 && phy->ops->power_off", "phy->ops->power_off",1)
        assert early_release!=counters
        run(Path(directory),"phy-counter-negative-early-parent-release",early_release,expect_failure=True)
        loader = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define IS_ERR(p) ((uintptr_t)(p)>=(uintptr_t)-4095)
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define dev_err_probe(d,e,...) (e)
#define dev_warn(...) ((void)0)
''' + enum(source,"atcphy_t8132_bank") + r'''
struct apple_tunable { unsigned sz; };
struct apple_atcphy {
    void *dev,*np;struct { void *core; } res;
    struct { void *resource; struct apple_tunable *tunable; } t8132_banks[6];
    struct { struct apple_tunable *lane_usb3[2],*usb3_dp; } tunables;
    bool ss_tunables;
};
static struct apple_tunable groups[8],aux={1},empty={0};
static unsigned required_calls,aux_calls,aux_kind;
static int missing;
static struct apple_tunable *result(unsigned kind,struct apple_tunable *valid)
{ return kind==0?(void *)(intptr_t)-ENOENT:kind==1?valid:kind==2?(void *)(intptr_t)-EINVAL:&empty; }
static struct apple_tunable *devm_apple_tunable_parse(void *d,void *n,const char *name,void *r)
{
    (void)d;(void)n;(void)r;
    if(!strcmp(name,"apple,tunable-usb3-dp")) {aux_calls++;return result(aux_kind,&aux);}
    assert(required_calls<8);unsigned i=required_calls++;
    return (int)i==missing?(void *)(intptr_t)-ENOENT:&groups[i];
}
''' + function(source,"atcphy_load_t8132_tunables") + r'''
static void reset(struct apple_atcphy *a)
{
    memset(a,0,sizeof(*a));for(unsigned i=0;i<8;i++) groups[i].sz=1;
    required_calls=aux_calls=0;aux_kind=1;missing=-1;
}
int main(void)
{
    struct apple_atcphy a;
    reset(&a);assert(!atcphy_load_t8132_tunables(&a));
    assert(a.ss_tunables && a.tunables.usb3_dp==&aux && aux_calls==1);
    for(unsigned bad=2;bad<=3;bad++) {
        reset(&a);aux_kind=bad;assert(atcphy_load_t8132_tunables(&a)==-EINVAL);
    }
    reset(&a);aux_kind=0;assert(!atcphy_load_t8132_tunables(&a));
    assert(a.ss_tunables && !a.tunables.usb3_dp);
    reset(&a);missing=7;assert(!atcphy_load_t8132_tunables(&a));
    assert(!a.ss_tunables && !aux_calls);
    puts("ok 13 - optional AUX_TOP calibration, USB-only fallback and malformed refusal");
    return 0;
}
'''
        loader=loader.replace("#include <assert.h>","#include <assert.h>\n#include <stdbool.h>",1)
        run(Path(directory),"atc-mode4-calibration-loader",loader)
        downgraded=loader.replace('if (ret != -ENOENT)', 'if (false)',1)
        assert downgraded!=loader
        run(Path(directory),"loader-negative-invalid-downgrade",downgraded,expect_failure=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
