#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile actual N1 recovery/PM source bodies with deterministic operation mocks.

This exercises refusal and ownership ordering, not device firmware or hardware DMA.
"""
import argparse
from pathlib import Path
import os
import re
import resource
import shlex
import signal
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / 'drivers/net/wireless/apple/centauri/t6050/centauri_pci.c').read_text()


def block(name):
    match = re.search(r'^(?:static )?(?:int|void|bool) ' + name + r'\([^;]*?\)\n\{', SOURCE, re.M)
    if not match:
        raise RuntimeError(f'Missing production function: {name}')
    brace = SOURCE.index('{', match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (SOURCE[end] == '{') - (SOURCE[end] == '}')
        end += 1
    return SOURCE[match.start():end]


PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define WRITE_ONCE(x,v) ((x)=(v))
#define smp_store_release(p,v) (*(p)=(v))
#define PCI_DEVFN(s,f) (((s)<<3)|(f))
#define PCI_SLOT(f) ((f)>>3)
#define CEN_BOOT_STOPPED 5
#define guard(x) mock_guard
#define list_for_each_entry(v,h,n) for ((v)=owner; (v); (v)=NULL)
struct device { int lock; void *data; };
struct pci_bus;
struct pci_dev { struct device dev; struct pci_bus *bus, *subordinate; unsigned devfn; };
struct pci_bus { struct pci_dev *self; struct pci_bus *subordinate; };
struct kref { int refs; };
struct work_struct { int queued; };
struct centauri_t6050_ipc { bool fenced; };
struct centauri_t6050_rewake {
    struct pci_dev *bridge, *control, *beta;
    unsigned slot, generation, work_generation;
    bool active, rerun;
    int error, node;
    void *arena;
    struct kref ref;
    struct work_struct work;
};
struct centauri_t6050 {
    struct centauri_t6050_rewake *recovery;
    struct pci_dev *pdev;
    struct centauri_t6050_ipc *ipc;
    bool removing, ready, firmware_sleep, pm_prepared;
    int boot_work, boot_lock, alpha_irqs, irqs, boot_phase;
    void *firmware, *descriptor;
};
static int cen_recovery_lock, cen_recovery_owners, cen_rewake_wq;
static bool cen_exiting, supported_sleep=true;
static struct centauri_t6050_rewake *owner;
static struct pci_dev *slots[3];
static int events[100], nevents, stop_error, beta_error, release_error;
static int wifi_error, requested_error, requests, queues;
enum { STOP_BOOT=1, QUIESCE, WIFI_STOP, IRQ_STOP, CLEAR, FENCE, RELEASE, FREE,
       PM_ERROR, PM_COMPLETE, REQUEST };
static void event(int n) { assert(nevents < 100); events[nevents++]=n; }
static void mutex_lock(int *p) { assert(!*p); *p=1; }
static void mutex_unlock(int *p) { assert(*p); *p=0; }
static void mock_guard(int *p) { (void)p; }
static void kref_get(struct kref *p) { p->refs++; }
static bool queue_work(int wq, struct work_struct *w)
{ (void)wq; assert(!w->queued); w->queued=1; queues++; return true; }
static struct pci_dev *pci_get_slot(struct pci_bus *b, unsigned f)
{ (void)b; return slots[f & 7]; }
static void pci_dev_put(struct pci_dev *p) { (void)p; }
static void pci_clear_master(struct pci_dev *p)
{ assert(p); event(CLEAR); }
static bool centauri_t6050_stop_dma(struct pci_dev *p)
{ assert(p); event(FENCE); return !stop_error; }
static bool centauri_t6050_ipc_fenced(struct centauri_t6050_ipc *p)
{ return !p || p->fenced; }
static void cancel_delayed_work_sync(int *p) { (void)p; event(STOP_BOOT); }
static int centauri_t6050_beta_call(struct centauri_t6050_rewake *r, bool release)
{
    struct centauri_t6050 *c = r->control->dev.data;
    assert(c && !c->boot_lock && c->removing && !c->ready);
    event(release ? RELEASE : QUIESCE);
    return release ? release_error : beta_error;
}
static void centauri_t6050_wifi_unregister(struct centauri_t6050 *c)
{ assert(c->boot_lock); event(WIFI_STOP); }
static void centauri_t6050_free_irqs(int *p) { (void)p; event(IRQ_STOP); }
static void centauri_t6050_free_boot(struct centauri_t6050 *c)
{ assert(c->boot_lock); event(FREE); c->ipc=NULL; c->firmware=c->descriptor=NULL; }
static struct pci_dev *to_pci_dev(struct device *d)
{ return (struct pci_dev *)((char *)d - offsetof(struct pci_dev,dev)); }
static void *pci_get_drvdata(struct pci_dev *p) { return p->dev.data; }
static bool apple_centauri_sleep_reboot(struct pci_dev *p)
{ (void)p; return supported_sleep; }
static void centauri_t6050_wifi_pm_prepare(struct centauri_t6050 *c)
{ assert(c->boot_lock); wifi_error=0; }
static int centauri_t6050_wifi_pm_error(struct centauri_t6050 *c)
{ assert(c->boot_lock); event(PM_ERROR); return wifi_error; }
static void centauri_t6050_wifi_pm_complete(struct centauri_t6050 *c)
{ assert(c->boot_lock); event(PM_COMPLETE); wifi_error=0; }
'''

TESTS = r'''
static void reset(struct centauri_t6050 *c, struct centauri_t6050_rewake *rw,
                  struct pci_dev fn[3], struct pci_dev *bridge, struct pci_bus *bus)
{
    memset(c,0,sizeof(*c)); memset(rw,0,sizeof(*rw));
    memset(fn,0,3*sizeof(*fn)); memset(bridge,0,sizeof(*bridge));
    *bus=(struct pci_bus){.self=bridge}; bridge->subordinate=bus;
    for (int i=0;i<3;i++) { fn[i].bus=bus; fn[i].devfn=i; slots[i]=&fn[i]; }
    owner=rw; rw->bridge=bridge; rw->control=&fn[0]; rw->beta=&fn[2];
    rw->ref.refs=1; rw->generation=1;
    c->pdev=&fn[0]; c->recovery=rw; c->ready=true;
    fn[0].dev.data=c;
    nevents=stop_error=beta_error=release_error=wifi_error=requests=queues=0;
    cen_exiting=false;
}
static int count(int value)
{ int n=0; for(int i=0;i<nevents;i++) n += events[i]==value; return n; }
static int first(int value)
{ for(int i=0;i<nevents;i++) if(events[i]==value) return i; return -1; }
int main(void)
{
    struct centauri_t6050 c;
    struct centauri_t6050_rewake rw;
    struct pci_dev fn[3], bridge;
    struct pci_bus bus;
    struct centauri_t6050_ipc ipc={.fenced=true};
    reset(&c,&rw,fn,&bridge,&bus);
    c.ipc=&ipc; c.firmware=&c; c.descriptor=&rw;
    assert(!centauri_t6050_quiesce(&c));
    assert(first(STOP_BOOT)<first(QUIESCE));
    assert(first(QUIESCE)<first(WIFI_STOP));
    assert(first(WIFI_STOP)<first(CLEAR));
    assert(count(CLEAR)==3 && count(FENCE)==3);
    assert(first(FENCE)>first(CLEAR)+1);
    assert(first(RELEASE)>first(FENCE) && first(FREE)>first(RELEASE));
    assert(!c.ipc && !c.firmware && !c.descriptor && !c.boot_lock);

    reset(&c,&rw,fn,&bridge,&bus);
    c.ipc=&ipc; c.firmware=&c; c.firmware_sleep=true; /* retry retained sleep */
    stop_error=1;
    assert(centauri_t6050_quiesce(&c)==-EBUSY);
    assert(c.ipc==&ipc && c.firmware==&c);
    assert(count(FENCE)==3 && !count(RELEASE) && !count(FREE));
    stop_error=0;
    assert(!centauri_t6050_quiesce(&c));
    assert(count(RELEASE)==1 && count(FREE)==1); /* firmware_sleep cannot skip */

    reset(&c,&rw,fn,&bridge,&bus); beta_error=-EIO;
    assert(centauri_t6050_quiesce(&c)==-EIO);
    assert(!count(FENCE) && !count(RELEASE) && !count(FREE));
    reset(&c,&rw,fn,&bridge,&bus); release_error=-EIO;
    assert(centauri_t6050_quiesce(&c)==-EIO && !count(FREE));
    reset(&c,&rw,fn,&bridge,&bus); ipc.fenced=false; c.ipc=&ipc;
    assert(centauri_t6050_quiesce(&c)==-EBUSY && !count(RELEASE) && !count(FREE));
    reset(&c,&rw,fn,&bridge,&bus); slots[2]=NULL;
    assert(!centauri_t6050_domain_fence(&rw)); /* old-owner identity cannot vanish */

    reset(&c,&rw,fn,&bridge,&bus);
    assert(!centauri_t6050_prepare(&fn[0].dev) && c.pm_prepared);
    wifi_error=-ETIMEDOUT; /* cfg80211 abort: F0 suspend never called */
    centauri_t6050_complete(&fn[0].dev);
    assert(!c.pm_prepared && rw.active && rw.error==-ETIMEDOUT && queues==1);
    assert(first(PM_ERROR)<first(PM_COMPLETE));
    centauri_t6050_complete(&fn[0].dev);
    assert(queues==1); /* double complete cannot enqueue */
    apple_centauri_sleep_recovery_needed(&fn[2],-EIO);
    assert(queues==1 && rw.ref.refs==2 && rw.error==-ETIMEDOUT && !rw.rerun);
    rw.generation++; /* a new probe can fail during the old work's rescan */
    apple_centauri_sleep_recovery_needed(&fn[2],-EIO);
    assert(rw.rerun && queues==1 && rw.ref.refs==2);
    cen_exiting=true;
    rw.active=false;
    apple_centauri_sleep_recovery_needed(&fn[2],-EIO);
    assert(queues==1);
    reset(&c,&rw,fn,&bridge,&bus); c.recovery=NULL;
    assert(centauri_t6050_prepare(&fn[0].dev)==-ENODEV && !c.pm_prepared);
    reset(&c,&rw,fn,&bridge,&bus);
    supported_sleep=false;
    assert(!centauri_t6050_prepare(&fn[0].dev) && !c.pm_prepared);
    wifi_error=-ETIMEDOUT;
    centauri_t6050_complete(&fn[0].dev);
    assert(!queues && !count(PM_ERROR) && !count(PM_COMPLETE));
    puts("N1 production recovery/PM ordering: PASS");
}
'''


OWNERSHIP_PRELUDE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#define PCI_FUNC(f) ((f)&7)
#define PCI_SLOT(f) ((f)>>3)
#define PCI_DEVFN(s,f) (((s)<<3)|(f))
#define IS_ENABLED(x) 0
#define IS_ERR(p) false
#define PTR_ERR(p) (-ENODEV)
#define EXPORT_SYMBOL_GPL(x)
#define list_for_each_entry(v,h,n) for ((v)=owner;(v);(v)=NULL)
typedef uint64_t dma_addr_t;
struct module { int refs; };
struct device { int lock, leases, refs; };
struct pci_bus;
struct pci_dev { struct device dev; struct pci_bus *bus, *subordinate; unsigned devfn; };
struct pci_bus { struct pci_dev *self; };
struct kref { int refs; };
struct apple_centauri_sleep_ops {
    struct module *owner;
    int (*quiesce)(void *);
    int (*release)(void *);
    bool (*retained)(void *);
};
struct centauri_t6050_rewake {
    struct pci_dev *bridge, *beta;
    unsigned slot;
    const struct apple_centauri_sleep_ops *ops;
    void *data, *arena;
    dma_addr_t arena_dma;
    size_t arena_size;
    bool beta_lease;
    struct kref ref;
};
static int cen_recovery_lock, cen_recovery_owners, lease_error, release_error;
static int frees, callbacks;
static bool host_quiet, domain_fenced;
static struct centauri_t6050_rewake *owner;
static struct pci_dev *slot;
static void *bound_arena;
static void mutex_lock(int *p) { assert(!*p); *p=1; }
static void mutex_unlock(int *p) { assert(*p); *p=0; }
static void device_lock(struct device *p) { mutex_lock(&p->lock); }
static void device_unlock(struct device *p) { mutex_unlock(&p->lock); }
static struct pci_dev *pci_get_slot(struct pci_bus *b, unsigned f)
{ (void)b; (void)f; if(slot) slot->dev.refs++; return slot; }
static struct pci_dev *pci_dev_get(struct pci_dev *p) { p->dev.refs++; return p; }
static void pci_dev_put(struct pci_dev *p) { if(p) { assert(p->dev.refs>0); p->dev.refs--; } }
static void centauri_t6050_owner_free(struct kref *p) { (void)p; assert(0); }
static void kref_put(struct kref *p, void (*f)(struct kref *))
{ assert(p->refs>1); p->refs--; (void)f; }
static struct centauri_t6050_rewake *centauri_t6050_owner_get(struct pci_dev *p)
{ (void)p; owner->ref.refs++; return owner; }
static int iommu_device_use_default_domain(struct device *p)
{ assert(p->lock); if(lease_error) return lease_error; p->leases++; return 0; }
static void iommu_device_unuse_default_domain(struct device *p)
{ assert(p->lock && p->leases>0); p->leases--; }
static bool try_module_get(struct module *m) { m->refs++; return true; }
static void module_put(struct module *m) { assert(m->refs>0); m->refs--; }
static void memzero_explicit(void *p, size_t n) { memset(p,0,n); }
static void dma_free_coherent(struct device *p, size_t n, void *a, dma_addr_t dma)
{ assert(p->lock && p->leases && n==32 && a && dma==0x1234 && domain_fenced); frees++; }
static int quiesce(void *data)
{ assert(data && slot->dev.lock && !cen_recovery_lock); host_quiet=true; callbacks++; return 0; }
static int release(void *data)
{
    assert(data && slot->dev.lock && !cen_recovery_lock && host_quiet && domain_fenced);
    callbacks++;
    if(release_error) return release_error;
    dma_free_coherent(&slot->dev,32,bound_arena,0x1234); bound_arena=NULL;
    return 0;
}
static bool retained(void *data)
{ assert(data && slot->dev.lock && !cen_recovery_lock); return bound_arena; }
"""

OWNERSHIP_TESTS = r"""
int main(void)
{
    struct centauri_t6050_rewake rw={.ref.refs=1};
    struct pci_dev bridge={0}, fn={.devfn=2};
    struct pci_bus bus={.self=&bridge}; fn.bus=&bus; bridge.subordinate=&bus;
    struct module module={0};
    struct apple_centauri_sleep_ops ops={&module,quiesce,release,retained};
    char arena[32]; int data;
    owner=&rw; slot=&fn; rw.bridge=&bridge;
    device_lock(&fn.dev);
    lease_error=-EBUSY;
    assert(apple_centauri_sleep_register(&fn,&ops,&data,arena,0x1234,32)==-EBUSY);
    assert(!rw.beta && !fn.dev.refs && !fn.dev.leases && rw.ref.refs==1);
    lease_error=0; bound_arena=arena;
    assert(!apple_centauri_sleep_register(&fn,&ops,&data,arena,0x1234,32));
    assert(rw.arena==arena && rw.beta==&fn && fn.dev.leases==1 && fn.dev.refs==1);
    device_unlock(&fn.dev);
    struct pci_dev replacement={.devfn=2,.bus=&bus};
    slot=&replacement;
    assert(centauri_t6050_beta_call(&rw,false)==-ENODEV && !callbacks);
    slot=&fn;
    assert(!centauri_t6050_beta_call(&rw,false));
    domain_fenced=true; release_error=-EBUSY;
    assert(centauri_t6050_beta_call(&rw,true)==-EBUSY);
    assert(rw.arena==arena && !frees && !module.refs);
    release_error=0;
    assert(!centauri_t6050_beta_call(&rw,true));
    assert(!rw.arena && !bound_arena && frees==1 && !module.refs);
    device_lock(&fn.dev);
    apple_centauri_sleep_unregister(&fn);
    assert(!rw.beta && !fn.dev.leases && !fn.dev.refs && rw.ref.refs==1);
    assert(!rw.ops && !rw.data);
    bound_arena=arena;
    assert(!apple_centauri_sleep_register(&fn,&ops,&data,arena,0x1234,32));
    apple_centauri_sleep_unregister(&fn); /* no callback/data may survive devm */
    assert(!rw.ops && !rw.data && rw.arena==arena && rw.beta==&fn);
    assert(fn.dev.leases==1 && fn.dev.refs==1 && rw.ref.refs==1);
    bound_arena=NULL; /* local binding relinquishes tuple without freeing */
    device_unlock(&fn.dev);
    int before=callbacks;
    assert(!centauri_t6050_beta_call(&rw,false));
    assert(!centauri_t6050_beta_call(&rw,true));
    assert(callbacks==before && frees==2 && !rw.arena && !rw.beta);
    assert(!fn.dev.leases && !fn.dev.refs);
    puts("N1 production registration/orphan ownership: PASS");
}
"""


RACE_PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint64_t dma_addr_t;
#define PCI_DEVFN(s,f) (((s)<<3)|(f))
#define SZ_16K 16384
struct device { bool locked; void *driver; void *data; };
struct pci_bus;
struct pci_dev { struct device dev; struct pci_bus *subordinate; int refs, leases; };
struct pci_bus { struct pci_dev *fn; };
struct kref { int refs; };
struct centauri_t6050_ipc { bool fenced; int frees; };
struct centauri_t6050_rewake {
    struct pci_dev *bridge, *control;
    unsigned slot;
    struct centauri_t6050_ipc *ipc;
    void *firmware, *descriptor, *arena;
    dma_addr_t firmware_dma, descriptor_dma;
    size_t firmware_size;
    bool control_lease;
    struct kref ref;
};
struct centauri_t6050 {
    struct centauri_t6050_rewake *recovery;
    struct centauri_t6050_ipc *ipc;
    void *firmware, *descriptor;
    dma_addr_t firmware_dma, descriptor_dma;
    size_t firmware_size;
};
static struct { int driver; } centauri_t6050_driver;
static struct pci_dev *original;
static bool inject_remove, remove_pending, remove_done, dispatching_remove;
static int dma_frees, requests, quiesces;
static void centauri_t6050_remove(struct pci_dev *fn);
static void run_remove(void);
static struct pci_dev *pci_get_slot(struct pci_bus *bus, unsigned devfn)
{ (void)devfn; if(bus->fn) bus->fn->refs++; return bus->fn; }
static void pci_dev_put(struct pci_dev *fn) { assert(fn->refs>0); fn->refs--; }
static void *pci_get_drvdata(struct pci_dev *fn) { return fn->dev.data; }
static void device_lock(struct device *dev) { assert(!dev->locked); dev->locked=true; }
static void device_unlock(struct device *dev)
{
    assert(dev->locked); dev->locked=false;
    if(remove_pending && !dispatching_remove) run_remove();
}
static void iommu_device_unuse_default_domain(struct device *dev)
{ assert(dev==&original->dev && dev->locked && original->leases>0); original->leases--; }
static void centauri_t6050_owner_free(struct kref *ref) { (void)ref; assert(0); }
static void kref_put(struct kref *ref, void (*freefn)(struct kref *))
{ assert(ref->refs>1); ref->refs--; (void)freefn; }
static int centauri_t6050_quiesce(struct centauri_t6050 *c)
{
    assert(original->dev.locked); quiesces++;
    /* Successful bound quiesce has already released all binding resources. */
    assert(!c->ipc && !c->firmware && !c->descriptor);
    return 0;
}
static void centauri_t6050_detach_ipc(struct centauri_t6050_ipc *ipc) { (void)ipc; }
static void apple_centauri_sleep_recovery_needed(struct pci_dev *fn, int ret)
{ (void)fn; (void)ret; requests++; }
static int centauri_t6050_beta_call(struct centauri_t6050_rewake *rw, bool release)
{
    if(release && inject_remove) {
        inject_remove=false; remove_pending=true;
        /* Deterministic scheduler: an unbind reaching a locked device waits. */
        if(!original->dev.locked) run_remove();
        assert(!remove_done && original->dev.locked && rw->control==original);
    }
    return 0;
}
static bool centauri_t6050_domain_fence(struct centauri_t6050_rewake *rw)
{ assert(original->dev.locked); return !rw->control || rw->control==original; }
static bool centauri_t6050_ipc_fenced(struct centauri_t6050_ipc *ipc)
{ assert(original->dev.locked); if(ipc) ipc->fenced=true; return true; }
static void centauri_t6050_release_ipc(struct centauri_t6050_ipc *ipc)
{ assert(original->dev.locked); if(ipc) { assert(ipc->fenced); ipc->frees++; } }
static void dma_free_coherent(struct device *dev, size_t size, void *cpu, dma_addr_t dma)
{
    assert(original->dev.locked && dev==&original->dev && cpu && size);
    assert(dma==0x1234 || dma==0x5678); dma_frees++;
}
'''

RACE_TESTS = r'''
static void run_remove(void)
{
    assert(!original->dev.locked);
    dispatching_remove=true; remove_pending=false;
    device_lock(&original->dev);
    centauri_t6050_remove(original);
    original->dev.driver=NULL;
    remove_done=true;
    device_unlock(&original->dev);
    dispatching_remove=false;
}
int main(void)
{
    struct pci_dev fn={.refs=1,.leases=1}, bridge={0};
    struct pci_bus bus={.fn=&fn}; bridge.subordinate=&bus;
    struct centauri_t6050_rewake rw={.bridge=&bridge,.control=&fn,
                                   .control_lease=true,.ref.refs=3};
    struct centauri_t6050 c={.recovery=&rw};
    fn.dev.data=&c; fn.dev.driver=&centauri_t6050_driver.driver; original=&fn;
    inject_remove=true;
    assert(!centauri_t6050_rewake_teardown(&rw));
    assert(remove_done && !remove_pending && !rw.control && !fn.dev.locked);
    assert(!rw.control_lease && !fn.leases && !fn.refs && !requests && quiesces==2);
    /* Already unbound F0 still holds the original fenced allocation tuples. */
    struct centauri_t6050_ipc ipc={0}; char firmware[32], descriptor[32];
    fn.refs=1; fn.leases=1; rw.control=&fn; rw.control_lease=true;
    rw.ipc=&ipc; rw.firmware=firmware; rw.firmware_dma=0x1234; rw.firmware_size=32;
    rw.descriptor=descriptor; rw.descriptor_dma=0x5678;
    assert(!centauri_t6050_rewake_teardown(&rw));
    assert(ipc.fenced && ipc.frees==1 && dma_frees==2);
    assert(!rw.ipc && !rw.firmware && !rw.descriptor && !rw.control);
    assert(!fn.refs && !fn.leases && !fn.dev.locked);
    struct pci_dev replacement={.refs=1}; bus.fn=&replacement; rw.control=&fn;
    assert(centauri_t6050_rewake_teardown(&rw)==-ENODEV);
    assert(rw.control==&fn && replacement.refs==1 && !replacement.dev.locked);
    bus.fn=NULL;
    assert(centauri_t6050_rewake_teardown(&rw)==-ENODEV && rw.control==&fn);
    puts("N1 production teardown/remove interleaving: PASS");
}
'''


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (2 * 1024**3, 2 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--negative-controls', action='store_true')
    args = parser.parse_args()
    names = ['apple_centauri_sleep_recovery_needed', 'centauri_t6050_domain_fence',
             'centauri_t6050_quiesce', 'centauri_t6050_prepare', 'centauri_t6050_complete']
    bodies = '\n'.join(block(name) for name in names)
    variants = [('baseline', bodies),
                ('negative-skip-fence', bodies.replace('!centauri_t6050_domain_fence(c->recovery)', 'false')),
                ('negative-skip-release', bodies.replace('ret = centauri_t6050_beta_call(c->recovery, true);', 'ret = 0;')),
                ('negative-lost-pm-error', bodies.replace('error = centauri_t6050_wifi_pm_error(c);', 'error = 0;')),
                ('negative-drop-new-generation', bodies.replace('rw->rerun = true;', 'rw->rerun = false;'))]
    ownership = '\n'.join(block(name) for name in [
        'apple_centauri_sleep_register', 'apple_centauri_sleep_unregister',
        'centauri_t6050_beta_call'])
    ownership_variants = [('ownership', ownership),
        ('negative-lost-orphan', ownership.replace('rw->ops = NULL;', 'rw->arena = NULL; rw->ops = NULL;')),
        ('negative-lost-domain-lease', ownership.replace('rw->beta_lease = true;', 'rw->beta_lease = false;'))]
    race = '\n'.join(block(name) for name in [
        'centauri_t6050_remove', 'centauri_t6050_rewake_teardown'])
    race_variants = [('remove-interleaving', race),
        ('negative-unlock-before-tuples', race.replace(
            '\tif (!ret)\n\t\tret = centauri_t6050_beta_call(rw, true);',
            '\tdevice_unlock(&fn->dev);\n\tif (!ret)\n\t\tret = centauri_t6050_beta_call(rw, true);'))]
    failures = {
        'negative-unlock-before-tuples': '!remove_done && original->dev.locked && rw->control==original',
        'negative-skip-fence': 'first(WIFI_STOP)<first(CLEAR)',
        'negative-skip-release': 'first(RELEASE)>first(FENCE) && first(FREE)>first(RELEASE)',
        'negative-lost-pm-error': '!c.pm_prepared && rw.active && rw.error==-ETIMEDOUT && queues==1',
        'negative-drop-new-generation': 'rw.rerun && queues==1 && rw.ref.refs==2',
        'negative-lost-orphan': '!rw.ops && !rw.data && rw.arena==arena && rw.beta==&fn',
        'negative-lost-domain-lease': '!rw.beta && !fn.dev.leases && !fn.dev.refs && rw.ref.refs==1',
    }
    with tempfile.TemporaryDirectory(prefix='centauri-recovery-') as tmp:
        out = args.out or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        compiler = shlex.split(os.environ.get('CC', 'cc'))
        for name, body in variants + ownership_variants + race_variants:
            if name.startswith('negative-') and not args.negative_controls:
                continue
            src, exe = out / f'{name}.c', out / name
            is_ownership = name in {n for n, _ in ownership_variants}
            is_race = name in {n for n, _ in race_variants}
            prelude = RACE_PRELUDE if is_race else OWNERSHIP_PRELUDE if is_ownership else PRELUDE
            tests = RACE_TESTS if is_race else OWNERSHIP_TESTS if is_ownership else TESTS
            src.write_text(prelude + body + tests)
            subprocess.run(compiler + ['-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-function', '-Wno-unused-variable',
                            str(src), '-o', str(exe)], check=True, preexec_fn=limits)
            result = subprocess.run([str(exe)], capture_output=True, text=True,
                                    preexec_fn=limits)
            (out / (name + '.log')).write_text(result.stdout + result.stderr)
            if name in ('baseline', 'ownership', 'remove-interleaving'):
                assert result.returncode == 0, result.stderr
                print(result.stdout.strip())
            else:
                assert (result.returncode == -signal.SIGABRT and 'Assertion' in result.stderr
                        and failures[name] in result.stderr), (name, result)
                print(f'{name} rejected by runtime assertion')


if __name__ == '__main__':
    run()
