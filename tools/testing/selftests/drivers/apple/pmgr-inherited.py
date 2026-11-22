#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run production inherited PMGR admission against host register mocks."""
import os
from pathlib import Path
import re
import resource
import shlex
import subprocess
import tempfile


def function(source, name):
    match = re.search(r"^static [^\n]*\b" + name + r"\(", source, re.M)
    assert match, name
    start = source.index("{", match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


def main():
    root = Path(__file__).resolve().parents[5]
    path = root / "drivers/pmdomain/apple/pmgr-pwrstate.c"
    if not path.exists():
        print("# SKIP: requires the matching kernel source tree")
        return 4
    source = path.read_text()
    start = source.index("#define APPLE_PMGR_RESET")
    end = source.index("\n", source.index("#define rcdev_to_apple_pmgr_ps", start))
    definitions = source[start:end] + "\n"
    preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define GENMASK(h,l) ((~0U << (l)) & (~0U >> (31-(h))))
#define FIELD_GET(m,v) (((v) & (m)) >> __builtin_ctz(m))
#define FIELD_PREP(m,v) (((v) << __builtin_ctz(m)) & (m))
#define container_of(p,t,m) ((t *)((char *)(p) - offsetof(t,m)))
#define GENPD_FLAG_IRQ_SAFE BIT(0)
#define GENPD_FLAG_ALWAYS_ON BIT(1)
#define GENPD_FLAG_DEFER_OFF BIT(2)
#define GENPD_FLAG_ACTIVE_WAKEUP BIT(3)
#define GENPD_STATE_OFF 0
#define GFP_KERNEL 0
#define THIS_MODULE NULL
#define EPROBE_DEFER 517
#define MAX_PHANDLE_ARGS 8
#define IS_ERR(p) false
#define PTR_ERR(p) (-EINVAL)
#define spin_lock_irqsave(lock,flags) ((flags)=0)
#define spin_unlock_irqrestore(lock,flags) ((void)(flags))
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_err_ratelimited(...) ((void)0)
#define dev_err_probe(dev,err,...) (err)
#define usleep_range(...) ((void)0)
#define __iomem
#define WRITE_ONCE(x,v) ((x)=(v))
#define PD_FLAG_NO_DEV_LINK BIT(0)
struct device_node {
    struct device_node *parent;
    const char *name, *conflict;
    bool inherited, firmware;
};
struct device { struct device_node *of_node; };
struct platform_device { struct device dev; };
struct dev_pm_ops { int (*runtime_suspend)(struct device *); };
struct dev_pm_domain { struct dev_pm_ops ops; };
struct generic_pm_domain {
    struct dev_pm_domain domain;
    const char *name;
    unsigned flags, status, slock;
    int (*power_on)(struct generic_pm_domain *), (*power_off)(struct generic_pm_domain *);
};
struct of_phandle_args { struct device_node *np; int args_count, args[8]; };
struct of_phandle_iterator { struct device_node *node; };
struct reset_controller_dev {
    void *owner, *ops;
    struct device_node *of_node;
    unsigned nr_resets, of_reset_n_cells;
    int (*of_xlate)(struct reset_controller_dev *, const struct of_phandle_args *);
};
struct resource { unsigned long long start; };
struct dev_pm_domain_attach_data { unsigned pd_flags; };
struct dev_pm_domain_list { struct device **pd_devs; unsigned num_pds; };
struct regmap { u32 value; int error; unsigned reads, writes; };
static struct regmap map;
static unsigned init_count, provider_count, reset_count;
static bool initialized_off;
static int apple_pmgr_reset_ops;
static int (*apple_pmgr_genpd_runtime_suspend)(struct device *dev);
static unsigned parent_attach_count;
static bool apple_t8132_firmware_path(void) { return true; }
static bool apple_sleep_domain_is_down(struct device_node *n) { (void)n; return false; }
static struct device_node *of_parse_phandle(struct device_node *n, const char *p, int i)
{ (void)n; (void)p; (void)i; return NULL; }
static bool of_device_is_compatible(struct device_node *n, const char *p)
{ return n && !strcmp(p,"apple,t8132-pmgr-pwrstate"); }
static bool of_machine_is_compatible(const char *p) { (void)p; return false; }
static bool of_property_present(struct device_node *n, const char *p)
{ return !strcmp(p,"status") || (n->conflict && !strcmp(n->conflict,p)) ||
    (n->inherited && !strcmp(p,"apple,inherited-on")) ||
    (n->firmware && !strcmp(p,"apple,firmware-owned")); }
static bool of_property_read_bool(struct device_node *n, const char *p)
{ return of_property_present(n,p); }
static int of_property_read_string(struct device_node *n, const char *p, const char **v)
{ (void)n; (void)p; *v="atc1_common"; return 0; }
static int of_property_read_u32(struct device_node *n, const char *p, u32 *v)
{ if (!strcmp(p,"reg")) { *v=0x428; return 0; }
  if (of_property_present(n,p)) { *v=0; return 0; } return -EINVAL; }
static int of_property_count_u32_elems(struct device_node *n, const char *p)
{ (void)n; (void)p; return -EINVAL; }
static int of_property_read_u32_index(struct device_node *n, const char *p, u32 i, u32 *v)
{ (void)n; (void)p; (void)i; (void)v; return -EINVAL; }
static int of_parse_phandle_with_fixed_args(struct device_node *n, const char *p, int c, int i,
                                            struct of_phandle_args *a)
{ (void)n; (void)p; (void)c; (void)i; (void)a; return -EINVAL; }
static int of_address_to_resource(struct device_node *n, int i, struct resource *r)
{ (void)n; (void)i; (void)r; return -EINVAL; }
static void *devm_ioremap(struct device *d, unsigned long long a, size_t s)
{ (void)d; (void)a; (void)s; return NULL; }
static int of_count_phandle_with_args(struct device_node *n, const char *p, const char *c)
{ (void)n; (void)p; (void)c; return 1; }
static int dev_pm_domain_attach_list(struct device *d, const struct dev_pm_domain_attach_data *a,
                                     struct dev_pm_domain_list **l)
{ (void)d; (void)a; (void)l; parent_attach_count++; return -EINVAL; }
static void dev_pm_domain_detach_list(struct dev_pm_domain_list *l) { (void)l; }
static int pm_runtime_resume_and_get(struct device *d) { (void)d; return 0; }
static void pm_runtime_put(struct device *d) { (void)d; }
static void of_node_put(struct device_node *n) { (void)n; }
static struct regmap *syscon_node_to_regmap(struct device_node *n) { (void)n; return &map; }
static void *devm_kzalloc(struct device *d, size_t s, int flags)
{ (void)d; (void)flags; return calloc(1,s); }
static void *driver_data;
static void platform_set_drvdata(struct platform_device *p, void *v) { (void)p; driver_data=v; }
static int regmap_read(struct regmap *m, u32 offset, u32 *v)
{ (void)offset; m->reads++; *v=m->value; return m->error; }
static int regmap_update_bits(struct regmap *m, u32 offset, u32 mask, u32 val)
{ (void)offset; m->writes++; m->value=(m->value&~mask)|val; return 0; }
#define regmap_read_poll_timeout_atomic(m,o,v,cond,delay,bound) \
    (regmap_read(m,o,&v) ?: ((cond) ? 0 : -ETIMEDOUT))
static int pm_genpd_init(struct generic_pm_domain *g, void *gov, bool off)
{ (void)g; (void)gov; init_count++; initialized_off=off; return 0; }
static int of_genpd_add_provider_simple(struct device_node *n, struct generic_pm_domain *g)
{ (void)n; (void)g; provider_count++; return 0; }
static void pm_genpd_remove_device(struct device *d) { (void)d; }
static void pm_genpd_remove(struct generic_pm_domain *g) { (void)g; }
static void of_genpd_del_provider(struct device_node *n) { (void)n; }
static int devm_reset_controller_register(struct device *d, struct reset_controller_dev *r)
{ (void)d; (void)r; reset_count++; return 0; }
#define of_for_each_phandle(it,ret,...) for ((ret)=0; false; )
static int of_phandle_iterator_args(struct of_phandle_iterator *it, int *a, unsigned n)
{ (void)it; (void)a; (void)n; return 0; }
static int of_genpd_add_subdomain(struct of_phandle_args *p, struct of_phandle_args *c)
{ (void)p; (void)c; return 0; }
'''
    fake_set = r'''
static int apple_pmgr_ps_set(struct generic_pm_domain *g, u32 state, bool auto_enable)
{ struct apple_pmgr_ps *ps=genpd_to_apple_pmgr_ps(g);
  (void)state; (void)auto_enable; ps->regmap->writes++; return 0; }
'''
    names = ["apple_pmgr_ps_is_active", "apple_pmgr_ps_check_inherited", "apple_pmgr_ps_power_on",
             "apple_pmgr_ps_power_off", "apple_pmgr_reset_assert", "apple_pmgr_reset_deassert",
             "apple_pmgr_reset_reset", "apple_pmgr_reset_status", "apple_pmgr_reset_xlate",
             "apple_pmgr_ps_hold_parents", "apple_pmgr_ps_release_parents",
             "apple_pmgr_ps_always_on", "apple_pmgr_ps_probe"]
    tests = r'''
static void clean(u32 value, int error)
{ map=(struct regmap){ .value=value, .error=error }; init_count=provider_count=reset_count=0;
  initialized_off=true; free(driver_data); driver_data=NULL; }
static int probe(struct device_node *node, u32 value, int error)
{ clean(value,error); struct platform_device p={ .dev.of_node=node }; return apple_pmgr_ps_probe(&p); }
int main(void)
{
    struct device_node n={ .name="power-controller@428", .inherited=true };
    assert(!probe(&n,0xff,0));
    struct apple_pmgr_ps *ps=driver_data;
    assert(map.reads==1 && !map.writes && init_count==1 && provider_count==1 && reset_count==1);
    assert(!parent_attach_count);
    assert(!initialized_off && (ps->genpd.flags&GENPD_FLAG_ALWAYS_ON));
    assert(!apple_pmgr_ps_power_on(&ps->genpd) && map.reads==2 && !map.writes);
    unsigned reads=map.reads;
    assert(apple_pmgr_ps_power_off(&ps->genpd)==-EBUSY);
    assert(apple_pmgr_reset_assert(&ps->rcdev,0)==-EPERM);
    assert(apple_pmgr_reset_deassert(&ps->rcdev,0)==-EPERM);
    assert(apple_pmgr_reset_reset(&ps->rcdev,0)==-EPERM);
    assert(apple_pmgr_reset_status(&ps->rcdev,0)==-EPERM);
    assert(map.reads==reads && !map.writes);
    puts("ok 1 - inherited active probe and lifecycle perform no register writes");
    for(unsigned target=0;target<16;target++) for(unsigned actual=0;actual<16;actual++) {
        int ret=probe(&n,target|(actual<<4),0);
        assert(ret==((target==15 && actual==15)?0:-EBUSY));
        assert(map.reads==1 && !map.writes);
        assert(provider_count==(unsigned)!ret && init_count==(unsigned)!ret);
    }
    const u32 blocked[]={APPLE_PMGR_RESET,APPLE_PMGR_AUTO_ENABLE,APPLE_PMGR_PS_RESET,
                         APPLE_PMGR_BUSY,APPLE_PMGR_DEV_DISABLE};
    for(unsigned i=0;i<sizeof(blocked)/sizeof(*blocked);i++) {
        assert(probe(&n,0xff|blocked[i],0)==-EBUSY);
        assert(map.reads==1 && !map.writes && !provider_count);
    }
    assert(!probe(&n,0xff|APPLE_PMGR_FLAGS|FIELD_PREP(APPLE_PMGR_PS_MIN,4),0));
    assert(!map.writes);
    puts("ok 2 - all target/actual pairs and forbidden handoff indications fail closed");
    assert(probe(&n,0xff,-EIO)==-EIO && map.reads==1 && !map.writes && !provider_count);
    assert(!probe(&n,0xff,0)); ps=driver_data;
    map.value=0x4f;
    assert(apple_pmgr_ps_power_on(&ps->genpd)==-EBUSY && !map.writes);
    map.value=0xff; map.error=-EACCES;
    assert(apple_pmgr_ps_power_on(&ps->genpd)==-EACCES && !map.writes);
    ps->system_sleep=true;
    assert(apple_pmgr_ps_power_off(&ps->genpd)==-EBUSY && !map.writes);
    puts("ok 3 - read errors and lost inherited activity never request a transition");
    const char *conflicts[]={"apple,firmware-owned","apple,sleep-retained","apple,always-on","apple,min-state",
        "apple,force-disable","apple,force-reset","apple,externally-clocked"};
    for(unsigned i=0;i<sizeof(conflicts)/sizeof(*conflicts);i++) {
        n.conflict=conflicts[i];
        assert(probe(&n,0xff,0)==-EINVAL);
        assert(!map.reads && !map.writes && !provider_count && !reset_count);
    }
    n.conflict=NULL; n.inherited=false; n.firmware=true;
    assert(!probe(&n,0,-EACCES)); ps=driver_data;
    assert(!map.reads && !map.writes && (ps->genpd.flags&GENPD_FLAG_ALWAYS_ON));
    assert(!apple_pmgr_ps_power_on(&ps->genpd) && !apple_pmgr_ps_power_off(&ps->genpd));
    assert(!map.reads && !map.writes);
    free(driver_data); driver_data=NULL;
    puts("ok 4 - contradictory policy fails before MMIO; firmware policy remains unchanged");
    return 0;
}
'''
    program = preamble + definitions + fake_set + "\n".join(function(source,n) for n in names) + tests
    print("TAP version 13\n1..5", flush=True)
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    with tempfile.TemporaryDirectory(prefix="apple-pmgr-inherited-") as temporary:
        tmp=Path(temporary)
        c=tmp/"test.c"
        c.write_text(program)
        compiler=shlex.split(os.environ.get("HOSTCC","cc"))
        subprocess.run(compiler+["-std=gnu11","-Wall","-Werror","-Wno-unused-function",
                                str(c),"-o",str(tmp/"test")],check=True)
        subprocess.run([str(tmp/"test")],check=True)
        controls = {
            "unchecked-active-state": program.replace("return -EBUSY;", "return 0;", 1),
            "reset-without-ownership": program.replace(
                "ps->firmware_owned || ps->inherited_on", "ps->firmware_owned", 1),
        }
        for name, negative in controls.items():
            assert negative != program
            c.write_text(negative)
            subprocess.run(compiler+["-std=gnu11","-Wall","-Werror","-Wno-unused-function",
                                    str(c),"-o",str(tmp/"test")],check=True)
            result = subprocess.run([str(tmp/"test")],capture_output=True)
            if result.returncode == 0:
                raise RuntimeError(f"negative control passed: {name}")
        print("ok 5 - unchecked admission and unowned reset negative controls are rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
