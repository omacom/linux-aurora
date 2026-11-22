#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise PCIe-C admission and s2idle domain-hold policy using production C."""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    start = re.search(r'^(?:static\s+)?(?:bool|int|void)\s+' + name + r'\(',
                      source, re.M).start()
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


FIXTURE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define GENMASK(h, l) ((~0U << (l)) & (~0U >> (31 - (h))))
struct device_node {
    const char *compat[3];
    bool kernel_init, tunables;
    int status, refs;
    struct device_node *dart, *child;
};
struct of_device_id { const char *compatible; const void *data; };
struct of_phandle_args { struct device_node *np; };
static int maps;
static bool of_device_is_compatible(const struct device_node *np, const char *compat)
{
    for (int i = 0; np && i < 3 && np->compat[i]; i++)
        if (!strcmp(np->compat[i], compat))
            return true;
    return false;
}
static const struct of_device_id *of_match_node(const struct of_device_id *ids,
                                                struct device_node *np)
{
    for (int i = 0; np && i < 3 && np->compat[i]; i++)
        for (const struct of_device_id *id = ids; id->compatible; id++)
            if (!strcmp(np->compat[i], id->compatible))
                return id;
    return NULL;
}
static bool of_property_read_bool(struct device_node *np, const char *name)
{
    assert(!strcmp(name, "apple,pciec-kernel-init"));
    return np->kernel_init;
}
static bool of_property_present(struct device_node *np, const char *name)
{
    assert(!strcmp(name, "apple,tunable"));
    return np->tunables;
}
static int of_property_read_u32(struct device_node *np, const char *name, u32 *v)
{
    assert(!strcmp(name, "apple,pciec-preinit-status"));
    if (np->status < 0)
        return -1;
    *v = np->status;
    return 0;
}
static int of_map_iommu_id(struct device_node *np, int rid, struct of_phandle_args *a)
{
    assert(rid == 0);
    maps++;
    if (!np->dart)
        return -1;
    a->np = np->dart;
    a->np->refs++;
    return 0;
}
static void of_node_put(struct device_node *np) { assert(np->refs > 0); np->refs--; }
#define pr_warn_once(...) ((void)0)
#define for_each_available_child_of_node_scoped(parent, child) \
    for (struct device_node *child = (parent)->child; child; child = NULL)
/* PRODUCTION */
static void expect(struct device_node *host, bool cold, bool admitted)
{
    struct device_node parent = { .child = host };
    maps = 0;
    assert(apple_pcie_tunnel_needs_cold_init(host) == cold);
    assert(apple_cio_pcie_tunnel_is_preinitialized(&parent) == admitted);
    assert(!host->dart || host->dart->refs == 0);
    if (!host->kernel_init || host->status == 1)
        assert(maps == 0);
}
int main(void)
{
    struct device_node dart = { .tunables = true };
    const char *intr2axi[] = { "apple,t8112-pciec", "apple,t6000-pciec" };
    for (unsigned int i = 0; i < 2; i++) {
        /* M2 must win over its T8103 fallback, without a board allowlist. */
        struct device_node host = { .compat = { intr2axi[i], "apple,t8103-pciec" },
                                   .status = -1, .dart = &dart };
        const struct of_device_id *id = of_match_node(apple_pcie_of_match, &host);
        assert(id && ((const struct hw_info *)id->data)->tunnel_intr2axi);
        expect(&host, false, false);  /* No implicit cold reset. */
        host.kernel_init = true;
        expect(&host, true, true);
        dart.tunables = false;
        expect(&host, false, false); /* Do not let the fallback bypass DART admission. */
        host.status = 1;
        expect(&host, false, true);  /* Preserve a firmware handoff. */
        host.status = 0;
        expect(&host, false, false);
        dart.tunables = true;
        expect(&host, true, true);  /* Failed/missing handoff can be initialized. */
        host.dart = NULL;
        expect(&host, false, false);
    }
    struct device_node m1 = { .compat = { "apple,t8103-pciec" }, .status = -1 };
    expect(&m1, false, false);
    m1.kernel_init = true;
    expect(&m1, true, true);        /* Original M1 does not need T8110 DART tunables. */
    m1.status = 1;
    expect(&m1, false, true);
    struct device_node regular = { .compat = { "apple,pcie" }, .kernel_init = true,
                                  .status = -1 };
    expect(&regular, false, false);
    regular.compat[0] = "apple,unknown-pciec";
    expect(&regular, false, false);
    puts("PASS: M1, M1 Pro, M2, firmware handoff, missing DART and DT opt-in");
    return 0;
}
'''

S2IDLE_FIXTURE = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define GENMASK(h, l) ((~0U << (l)) & (~0U >> (31 - (h))))
#define READ_ONCE(x) (x)
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#define NOTIFY_OK 0x0001
#define NOTIFY_STOP_MASK 0x8000
#define GENPD_NOTIFY_PRE_OFF 0
#define GENPD_NOTIFY_ON 1
#define PM_SUSPEND_ON 0
#define PM_SUSPEND_TO_IDLE 1
#define PM_SUSPEND_MEM 3
static inline int notifier_from_errno(int err)
{
    return err ? (NOTIFY_STOP_MASK | (NOTIFY_OK - err)) : NOTIFY_OK;
}
static inline int notifier_to_errno(int ret)
{
    ret &= ~NOTIFY_STOP_MASK;
    return ret > NOTIFY_OK ? NOTIFY_OK - ret : 0;
}
struct of_device_id { const char *compatible; const void *data; };
struct notifier_block;
typedef int (*notifier_fn_t)(struct notifier_block *nb,
                             unsigned long action, void *data);
struct notifier_block { notifier_fn_t notifier_call; };
struct device {
    struct apple_pcie *pcie;
    struct notifier_block *genpd_nb;
    bool genpd_on;
    int genpd_add_ret;
    unsigned int add_calls, remove_calls;
};
struct apple_pcie_pd {
    struct device *dev;
    struct notifier_block nb;
    unsigned int vetoed;
};
struct apple_dart_cio { int dummy; };
struct hw_info;
struct apple_pcie {
    struct device *dev;
    const struct hw_info *hw;
    bool power_retained, kernel_init, tunnel_cold_init;
    bool bus_stopped, resume_failed, link_kept;
    struct apple_pcie_pd *pds;
    unsigned int num_pds, pd_held;
    struct apple_dart_cio *cio_dart;
    bool cio_prepared;
};
static bool s2idle_keep_link = true;
static int pm_suspend_target_state = PM_SUSPEND_TO_IDLE;
static bool link_healthy = true;
static unsigned int radio_prepare_calls, warn_calls;
static bool pm_suspend_no_platform(void)
{
    return pm_suspend_target_state == PM_SUSPEND_TO_IDLE;
}
static struct apple_pcie *apple_pcie_lookup(struct device *dev)
{
    return dev ? dev->pcie : NULL;
}
static bool apple_pcie_tunnel_link_healthy(struct apple_pcie *pcie)
{
    (void)pcie;
    return link_healthy;
}
static int apple_radio_prepare(struct device *dev)
{
    (void)dev;
    radio_prepare_calls++;
    return 0;
}
static int dev_pm_genpd_add_notifier(struct device *dev,
                                     struct notifier_block *nb)
{
    dev->add_calls++;
    if (dev->genpd_add_ret)
        return dev->genpd_add_ret;
    assert(!dev->genpd_nb);
    dev->genpd_nb = nb;
    return 0;
}
static int dev_pm_genpd_remove_notifier(struct device *dev)
{
    dev->remove_calls++;
    assert(dev->genpd_nb != NULL);
    dev->genpd_nb = NULL;
    return 0;
}
static bool dev_pm_genpd_is_on(struct device *dev) { return dev->genpd_on; }
static inline void dev_dbg(struct device *dev, const char *fmt, ...)
{
    (void)dev;
    (void)fmt;
}
static inline void dev_warn(struct device *dev, const char *fmt, ...)
{
    (void)dev;
    (void)fmt;
    warn_calls++;
}
/* PRODUCTION */
static const struct hw_info *match_compat(const char *compat)
{
    for (const struct of_device_id *id = apple_pcie_of_match;
         id->compatible; id++)
        if (!strcmp(id->compatible, compat))
            return id->data;
    return NULL;
}
static void reset_host(struct device *dev, struct apple_pcie *pcie,
                       struct apple_pcie_pd *pd, const struct hw_info *hw,
                       struct apple_dart_cio *cio_dart)
{
    memset(dev, 0, sizeof(*dev));
    memset(pcie, 0, sizeof(*pcie));
    memset(pd, 0, sizeof(*pd));
    dev->pcie = pcie;
    dev->genpd_on = true;
    pd->dev = dev;
    pcie->dev = dev;
    pcie->hw = hw;
    pcie->kernel_init = true;
    pcie->pds = pd;
    pcie->num_pds = 1;
    pcie->cio_dart = cio_dart;
    s2idle_keep_link = true;
    pm_suspend_target_state = PM_SUSPEND_TO_IDLE;
    link_healthy = true;
    radio_prepare_calls = 0;
    warn_calls = 0;
}
int main(void)
{
    struct apple_dart_cio cio_dart = { 1 };
    struct device dev;
    struct apple_pcie pcie;
    struct apple_pcie_pd pd;
    const struct hw_info *t6050 = match_compat("apple,t6050-pciec");
    const struct hw_info *t8132 = match_compat("apple,t8132-pciec");
    const struct hw_info *t6040 = match_compat("apple,t6040-pciec");
    const struct hw_info *t8103 = match_compat("apple,t8103-pciec");
    int nret;

    /* T6050 PCIe-C opts into s2idle_hold_domain like T8132 and T6040. */
    assert(t6050 && t6050->tunneled && t6050->s2idle_hold_domain);
    assert(t8132 && t8132->s2idle_hold_domain);
    assert(t6040 && t6040->s2idle_hold_domain);
    assert(t8103 && t8103->tunneled && !t8103->s2idle_hold_domain);

    /* Non-retained T6050 cio_dart host (tunnel_cold_init) holds domain in s2idle and releases. */
    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.kernel_init = false;
    pcie.tunnel_cold_init = true;
    assert(apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == 0);
    assert(radio_prepare_calls == 1 && warn_calls == 0);
    assert(apple_pcie_keep_link(&pcie));
    assert(pcie.pd_held == 1 && dev.genpd_nb == &pd.nb);
    assert(dev.add_calls == 1 && dev.remove_calls == 0);
    assert(dev.genpd_nb->notifier_call(dev.genpd_nb, GENPD_NOTIFY_ON, NULL) == NOTIFY_OK);
    assert(pd.vetoed == 0);
    nret = dev.genpd_nb->notifier_call(dev.genpd_nb, GENPD_NOTIFY_PRE_OFF, NULL);
    assert(notifier_to_errno(nret) == -EBUSY && pd.vetoed == 1);
    apple_pcie_release_domains(&pcie);
    assert(pcie.pd_held == 0 && dev.genpd_nb == NULL && dev.remove_calls == 1);

    /* Non-retained T8132 cio_dart host (kernel_init) also passes prepare and holds domain. */
    reset_host(&dev, &pcie, &pd, t8132, &cio_dart);
    assert(apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);
    assert(apple_pcie_keep_link(&pcie) && pcie.pd_held == 1 && dev.add_calls == 1);
    apple_pcie_release_domains(&pcie);
    assert(pcie.pd_held == 0 && dev.genpd_nb == NULL && dev.remove_calls == 1);

    /* Domain already off or unhealthy link passes prepare and cleans up notifier in keep_link. */
    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.kernel_init = false;
    pcie.tunnel_cold_init = true;
    dev.genpd_on = false;
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);
    assert(!apple_pcie_keep_link(&pcie));
    assert(pcie.pd_held == 0 && dev.genpd_nb == NULL && dev.remove_calls == 1);

    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.kernel_init = false;
    pcie.tunnel_cold_init = true;
    link_healthy = false;
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);
    assert(!apple_pcie_keep_link(&pcie));
    assert(pcie.pd_held == 0 && dev.genpd_nb == NULL && dev.remove_calls == 1);

    /* Deep sleep is untouched: unprepared refuses early, prepared passes without holding. */
    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pm_suspend_target_state = PM_SUSPEND_MEM;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    pcie.cio_prepared = true;
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.genpd_nb == NULL);

    /* PM_SUSPEND_ON (non-sleep) does not refuse in prepare even without cio_prepared. */
    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pm_suspend_target_state = PM_SUSPEND_ON;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);

    /* Every pre-suspend refusal term in apple_pcie_can_keep_link() refuses early in prepare. */
    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    s2idle_keep_link = false;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.kernel_init = false;
    pcie.tunnel_cold_init = false;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.bus_stopped = true;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    reset_host(&dev, &pcie, &pd, t6050, &cio_dart);
    pcie.resume_failed = true;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    /* Non-retained cio_dart host without s2idle_hold_domain refuses early in prepare. */
    reset_host(&dev, &pcie, &pd, t8103, &cio_dart);
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == -EBUSY && radio_prepare_calls == 0 && warn_calls == 1);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    /* Non-cio_dart host is not refused in prepare (noirq falls back to stop_for_sleep). */
    reset_host(&dev, &pcie, &pd, t8103, NULL);
    pcie.bus_stopped = true;
    assert(!apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);
    assert(!apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    /* Retained power domain on a non-holding host passes prepare and keeps link without hold. */
    reset_host(&dev, &pcie, &pd, t8103, &cio_dart);
    pcie.power_retained = true;
    assert(apple_pcie_can_keep_link(&pcie));
    assert(apple_pcie_prepare(&dev) == 0 && radio_prepare_calls == 1 && warn_calls == 0);
    assert(apple_pcie_keep_link(&pcie) && pcie.pd_held == 0 && dev.add_calls == 0);

    puts("PASS: T6050 PCIe-C s2idle domain hold, release, deep sleep and prepare refusal");
    return 0;
}
'''


def run_cases(out, cases, extra_flags=()):
    for name, body in cases.items():
        source = out / (name + '.c')
        source.write_text(body)
        binary = out / name
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror',
                        '-Wno-unused-function', *extra_flags,
                        str(source), '-o', str(binary)], check=True)
        run = subprocess.run([str(binary)], capture_output=True, text=True)
        (out / (name + '.log')).write_text(run.stdout + run.stderr)
        if name.endswith('production'):
            assert run.returncode == 0, run.stderr
            print(run.stdout, end='')
        else:
            assert run.returncode < 0 and 'Assertion' in run.stderr, (
                name, run.returncode, run.stderr)
            print('PASS negative control:', name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    pci = (ROOT / 'drivers/pci/controller/pcie-apple.c').read_text()
    acio = (ROOT / 'drivers/thunderbolt/apple.c').read_text()
    defines = '\n'.join(re.findall(r'^#define (?:[^\n]*\\\n)*[^\n]*$', pci, re.M))
    info = pci[pci.index('struct hw_info {'):pci.index('static const struct of_device_id apple_pcie_of_match[];')]
    matches = re.search(r'static const struct of_device_id apple_pcie_of_match\[\] = \{.*?\n\};', pci, re.S)[0]
    production = '\n'.join((defines, info, matches,
                            function(pci, 'apple_pcie_tunnel_needs_cold_init'),
                            function(acio, 'apple_cio_pciec_eligible'),
                            function(acio, 'apple_cio_pcie_tunnel_is_preinitialized')))
    code = FIXTURE.replace('/* PRODUCTION */', production)
    s2idle_prod = '\n'.join((defines, info, matches,
                             *(function(pci, n) for n in (
                                 'apple_pcie_pd_notify',
                                 'apple_pcie_release_domains',
                                 'apple_pcie_hold_domains',
                                 'apple_pcie_can_keep_link',
                                 'apple_pcie_keep_link',
                                 'apple_pcie_prepare'))))
    s2idle_code = S2IDLE_FIXTURE.replace('/* PRODUCTION */', s2idle_prod)
    t6050_old = '\t.mps_timers\t\t= true,\n\t.s2idle_hold_domain\t= true,\n'
    s2idle_muts = {
        'drop-t6050-hold': (t6050_old, '\t.mps_timers\t\t= true,\n'),
        'drop-prepare-guard': (
            'pm_suspend_target_state != PM_SUSPEND_ON &&\n'
            '\t    !apple_pcie_can_keep_link(pcie)',
            'pm_suspend_target_state != PM_SUSPEND_ON &&\n\t    false'),
        'drop-keep-link-param': (
            'if (!READ_ONCE(s2idle_keep_link) || !pm_suspend_no_platform())',
            'if (!pm_suspend_no_platform())'),
        'drop-no-platform-check': (
            'if (!READ_ONCE(s2idle_keep_link) || !pm_suspend_no_platform())',
            'if (!READ_ONCE(s2idle_keep_link))'),
        'drop-init-check': (
            'if (!(pcie->kernel_init || pcie->tunnel_cold_init))',
            'if (false)'),
        'drop-bus-stopped-check': (
            'if (pcie->bus_stopped || pcie->resume_failed)',
            'if (pcie->resume_failed)'),
        'drop-resume-failed-check': (
            'if (pcie->bus_stopped || pcie->resume_failed)',
            'if (pcie->bus_stopped)'),
        'drop-domain-policy-check': (
            'if (!pcie->power_retained && !pcie->hw->s2idle_hold_domain)',
            'if (false)'),
        'skip-domain-release': (
            'dev_pm_genpd_remove_notifier(pd->dev);', '(void)pd;'),
        'ignore-domain-off': (
            'if (!dev_pm_genpd_is_on(pd->dev)) {', 'if (false) {'),
    }
    s2idle_cases = {'s2idle-production': s2idle_code}
    for name, (old, new) in s2idle_muts.items():
        assert s2idle_code.count(old) == 1, name
        s2idle_cases[name] = s2idle_code.replace(old, new)
    with tempfile.TemporaryDirectory(prefix='pciec-policy-') as tmp:
        out = args.out or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        cases = {
            'production': code,
            'm2-fallback': code.replace(
                '{ .compatible = "apple,t8112-pciec",\t.data = &t8112_pciec_hw },', ''),
            'dart-bypass': code.replace(
                'tunables = of_property_present(iommu_spec.np, "apple,tunable");',
                'tunables = true;'),
        }
        run_cases(out, cases)
        run_cases(out, s2idle_cases,
                  extra_flags=('-Wextra', '-Wno-unused-parameter', '-g',
                               '-fsanitize=address,undefined'))


if __name__ == '__main__':
    main()
