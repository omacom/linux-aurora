/* SPDX-License-Identifier: GPL-2.0-only */
/* Offline environment for extracted production functions; no hardware I/O. */
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define assert(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d (%s): %s\n", \
 __LINE__, __FILE__, #x); exit(1); } } while (0)
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define smp_store_release(p, v) (*(p) = (v))
#define PCI_DEVFN(s, f) (((s) << 3) | (f))
#define PCI_COMMAND 4
#define PCI_COMMAND_MASTER 4
#define PCI_EXP_LNKSTA 18
#define PCI_EXP_LNKSTA_DLLLA 0x2000
#define CEN_BOOT_STOPPED 5
#define SZ_16K 16384
#define CEN_ARENA_SIZE 65536
typedef uint16_t u16;
typedef int64_t s64;
typedef int64_t ktime_t;
typedef uint64_t dma_addr_t;
struct device { void *driver; int locked, runtime; };
struct pci_dev;
struct pci_bus { struct pci_dev *self, *slot[3]; };
struct pci_dev {
 struct device dev;
 struct pci_bus *bus, *subordinate;
 void *drvdata;
 int devfn, refs, command, reads, writes;
 bool visible, pending;
};
struct work_struct { int unused; };
struct kref { int count; };
struct centauri_t6050_ipc {
 bool owned[3]; struct pci_dev *function[3]; void *buffer;
};
struct apple_centauri_sleep_ops {
 void *owner;
 int (*quiesce)(void *), (*release)(void *);
 bool (*retained)(void *);
};
struct centauri_t6050_rewake {
 struct work_struct work; struct kref ref;
 struct pci_dev *bridge, *control, *beta;
 unsigned int slot, generation, work_generation;
 bool active, rerun, control_lease, beta_lease;
 int error;
 struct centauri_t6050_ipc *ipc;
 void *firmware, *descriptor, *arena, *data;
 dma_addr_t firmware_dma, descriptor_dma, arena_dma;
 size_t firmware_size, arena_size;
 const struct apple_centauri_sleep_ops *ops;
};
struct centauri_t6050_irqs { struct pci_dev *pdev; int count; };
struct centauri_t6050 {
 struct pci_dev *pdev;
 struct centauri_t6050_rewake *recovery;
 struct centauri_t6050_ipc *ipc;
 void *firmware, *descriptor;
 dma_addr_t firmware_dma, descriptor_dma;
 size_t firmware_size;
 bool removing, ready, firmware_sleep, link_ready, boot_attempted;
 int boot_lock, boot_work, boot_phase;
 struct centauri_t6050_irqs irqs, alpha_irqs;
};
struct cen_bt {
 struct pci_dev *pdev; void *arena, *hdev;
 dma_addr_t dma;
 bool host_quiesced, running, irq_vectors, ipc_requested;
 int irq_count, io_work;
};
static struct { int driver; } centauri_t6050_driver;
static int cen_recovery_lock, cen_rewake_wq;
static bool cen_exiting, thawed, callbacks_done, warm_done, link_active;
static bool force_pending, fail_beta_quiesce, fail_beta_release, module_available;
static int rescan_lock, removes, rescans, recycles, refusals, log_error, log_original;
static int module_refs, wifi_users, irq_users, boot_users, frees;
static ktime_t fake_time;
static struct pci_dev devs[4], replacement;
static struct pci_bus bus;
static struct centauri_t6050 control;
static struct centauri_t6050_rewake owner;
static struct cen_bt beta;
static struct centauri_t6050_ipc ipc;
static struct allocation {
 int token; bool live, published; struct pci_dev *owner;
} allocations[8];

static int centauri_t6050_quiesce(struct centauri_t6050 *c);
static void centauri_t6050_remove(struct pci_dev *pdev);
static void centauri_t6050_free_ipc(struct centauri_t6050 *c);
static bool centauri_t6050_ipc_fenced(struct centauri_t6050_ipc *p);
static void centauri_t6050_release_ipc(struct centauri_t6050_ipc *p);
static void mutex_lock(int *p) { assert(!*p); *p = 1; }
static void mutex_unlock(int *p) { assert(*p); *p = 0; }
static void device_lock(struct device *p) { mutex_lock(&p->locked); }
static void device_unlock(struct device *p) { mutex_unlock(&p->locked); }
static struct pci_dev *pci_get_slot(struct pci_bus *b, int devfn)
{ assert(devfn < 3); struct pci_dev *p = b->slot[devfn]; if (p) p->refs++; return p; }
static void pci_dev_put(struct pci_dev *p) { if (p) { assert(p->refs > 0); p->refs--; } }
static void *pci_get_drvdata(struct pci_dev *p) { return p->drvdata; }
static bool can_config(struct pci_dev *p) { return link_active && p->visible; }
static void pci_clear_master(struct pci_dev *p)
{ p->reads++; p->writes++; if (can_config(p)) p->command &= ~PCI_COMMAND_MASTER; }
static int pci_read_config_word(struct pci_dev *p, int where, u16 *v)
{ (void)where; p->reads++; *v = can_config(p) ? p->command : 0xffff; return 0; }
static int pcibios_err_to_errno(int x) { return x ? -EIO : 0; }
static bool pci_wait_for_pending_transaction(struct pci_dev *p)
{ return can_config(p) && !p->pending && !force_pending; }
static int work_busy(int *p) { return *p; }
static void cancel_delayed_work_sync(int *p) { *p = 0; boot_users = 0; }
static void centauri_t6050_wifi_unregister(struct centauri_t6050 *c)
{ assert(c->removing); wifi_users = 0; }
static int pci_irq_vector(struct pci_dev *p, int n) { (void)p; return n; }
static void free_irq(int n, void *c) { (void)n; (void)c; irq_users = 0; }
static void centauri_t6050_free_irqs(struct centauri_t6050_irqs *p)
{ p->count = 0; p->pdev = NULL; irq_users = 0; }
static void iommu_device_unuse_default_domain(struct device *p) { (void)p; }
static bool try_module_get(void *p) { (void)p; if (!module_available) return false; module_refs++; return true; }
static void module_put(void *p) { (void)p; assert(module_refs > 0); module_refs--; }
static void memzero_explicit(void *p, size_t n) { (void)p; (void)n; }
static void dma_free_coherent(struct device *d, size_t size, void *p, dma_addr_t dma)
{
 (void)size; (void)dma;
 assert(!wifi_users && !irq_users && !boot_users);
 for (unsigned int i = 0; i < 8; i++) if (p == &allocations[i].token) {
  assert(allocations[i].live && &allocations[i].owner->dev == d);
  assert(allocations[i].owner == &devs[2] ? owner.beta_lease : owner.control_lease);
  if (allocations[i].published) for (int f = 0; f < 3; f++)
   assert(can_config(&devs[f]) && !(devs[f].command & PCI_COMMAND_MASTER) &&
          !devs[f].pending && !force_pending);
  allocations[i].live = false; frees++; return;
 }
 assert(false);
}
static void centauri_t6050_release_ipc(struct centauri_t6050_ipc *p)
{ if (p && p->buffer) { dma_free_coherent(&p->function[0]->dev, 1, p->buffer, 0); p->buffer = NULL; } }
static void centauri_t6050_detach_ipc(struct centauri_t6050_ipc *p) { (void)p; }
static int model_beta_quiesce(void *p)
{
 struct cen_bt *b = p;
 if (fail_beta_quiesce) return -EIO;
 b->host_quiesced = true; b->hdev = NULL; b->running = false;
 b->irq_count = b->io_work = 0; b->irq_vectors = false;
 return 0;
}
static int cen_quiesce(void *p) { return model_beta_quiesce(p); }
static void centauri_t6050_owner_free(struct kref *r) { (void)r; assert(false); }
static void kref_put(struct kref *r, void (*f)(struct kref *))
{ assert(r->count > 1); if (!--r->count) f(r); }
static void apple_centauri_sleep_recovery_needed(struct pci_dev *p, int e)
{ (void)p; assert(e); }
static void pm_runtime_get_sync(struct device *d) { d->runtime++; }
static void pm_runtime_put(struct device *d) { assert(d->runtime > 0); d->runtime--; }
static void pci_lock_rescan_remove(void)
{ assert(thawed && callbacks_done && !rescan_lock); rescan_lock = 1; }
static void pci_unlock_rescan_remove(void) { assert(rescan_lock); rescan_lock = 0; }
static bool apple_pcie_t6050_warm_reset_done(struct pci_dev *p) { (void)p; return warm_done; }
static int apple_pcie_t6050_port_recycle(struct pci_dev *p)
{ (void)p; assert(!bus.slot[0] && !bus.slot[1] && !bus.slot[2]); recycles++; warm_done = true; return 0; }
static ktime_t ktime_get(void) { return fake_time += 1000; }
static s64 ktime_ms_delta(ktime_t a, ktime_t b) { return a - b; }
static ktime_t apple_pcie_t6050_link_up_time(struct pci_dev *p) { (void)p; return 1; }
static int pcie_capability_read_word(struct pci_dev *p, int w, u16 *v)
{ (void)p; (void)w; *v = link_active ? PCI_EXP_LNKSTA_DLLLA : 0; return 0; }
static void usleep_range(int a, int b) { (void)a; (void)b; }
static void msleep(s64 n) { (void)n; }
static const char *pci_name(struct pci_dev *p) { (void)p; return "model"; }
static void pci_rescan_bus(struct pci_bus *b)
{ assert(!b->slot[0] && !b->slot[1] && !b->slot[2]); rescans++; b->slot[0] = &replacement; }
static void queue_work(int q, struct work_struct *w) { (void)q; (void)w; assert(false); }
#define dev_warn(...) ((void)0)
#define pci_dbg(p, ...) model_debug(__VA_ARGS__)
static void model_debug(const char *fmt, ...) { (void)fmt; }
#define pci_err(p, fmt, ...) model_error(fmt, __VA_ARGS__)
static void model_error(const char *fmt, ...)
{
 assert(strstr(fmt, "recovery refused"));
 va_list ap; va_start(ap, fmt); log_error = va_arg(ap, int); log_original = va_arg(ap, int);
 va_end(ap); refusals++;
}
static void pci_stop_and_remove_bus_device(struct pci_dev *p);

/* ACTUAL_FUNCTIONS */

static int model_beta_release(void *p)
{ return fail_beta_release ? -EIO : cen_release(p); }
static const struct apple_centauri_sleep_ops ops = {
 .quiesce = cen_quiesce, .release = model_beta_release, .retained = cen_retained,
};

static int live_allocations(void)
{ int n = 0; for (int i = 0; i < 8; i++) n += allocations[i].live; return n; }

static void *allocation(int n, int fn)
{
 allocations[n].live = allocations[n].published = true;
 allocations[n].owner = &devs[fn];
 return &allocations[n].token;
}

static void fixture(void)
{
 memset(devs, 0, sizeof(devs)); memset(&replacement, 0, sizeof(replacement));
 memset(&control, 0, sizeof(control)); memset(&owner, 0, sizeof(owner));
 memset(&beta, 0, sizeof(beta)); memset(&ipc, 0, sizeof(ipc));
 memset(&bus, 0, sizeof(bus)); memset(allocations, 0, sizeof(allocations));
 rescan_lock = removes = rescans = recycles = refusals = frees = module_refs = 0;
 log_error = log_original = 0; cen_recovery_lock = 0;
 force_pending = fail_beta_quiesce = fail_beta_release = cen_exiting = false;
 callbacks_done = thawed = false; module_available = link_active = true;
 warm_done = false; fake_time = 0;
 wifi_users = irq_users = boot_users = 1;
 bus.self = &devs[3]; devs[3].subordinate = &bus;
 for (int i = 0; i < 3; i++) {
  bus.slot[i] = &devs[i]; devs[i].bus = &bus; devs[i].devfn = i;
  devs[i].visible = true; devs[i].command = PCI_COMMAND_MASTER;
  devs[i].refs = 10; ipc.function[i] = &devs[i]; ipc.owned[i] = true;
 }
 owner.bridge = &devs[3]; owner.control = &devs[0]; owner.beta = &devs[2];
 owner.control_lease = owner.beta_lease = true; owner.active = true; owner.ref.count = 2;
 owner.ops = &ops; owner.data = &beta;
 control.pdev = &devs[0]; control.recovery = &owner; control.link_ready = true;
 control.boot_attempted = control.ready = true;
 control.irqs.pdev = &devs[0]; control.alpha_irqs.pdev = &devs[1];
 control.irqs.count = control.alpha_irqs.count = 1;
 devs[0].dev.driver = &centauri_t6050_driver.driver; devs[0].drvdata = &control;
 beta.pdev = &devs[2]; beta.running = beta.irq_vectors = beta.ipc_requested = true;
 beta.hdev = &beta; beta.irq_count = 2; beta.io_work = 1;
}

static void publish_all(void)
{
 control.firmware = allocation(0, 0); control.descriptor = allocation(1, 0);
 control.ipc = &ipc; ipc.buffer = allocation(2, 0);
 beta.arena = owner.arena = allocation(3, 2);
}

/* Reset has returned N1 to ROM; old Linux F1/F2 objects remain registered. */
static void physical_reset(void)
{
 assert(!live_allocations());
 warm_done = true;
 for (int i = 0; i < 3; i++) {
  devs[i].command = 0; devs[i].reads = devs[i].writes = 0;
  devs[i].visible = i == 0;
 }
 assert(bus.slot[1] && bus.slot[2]);
}

static void thaw_and_run(void)
{
 callbacks_done = thawed = true;
 centauri_t6050_rewake_worker(&owner.work);
 assert(!rescan_lock && !devs[3].dev.runtime && !module_refs);
 assert(!devs[0].dev.locked && !devs[2].dev.locked && !control.boot_lock);
 assert(!owner.active && !owner.error && owner.ref.count == 1);
}

static void pci_stop_and_remove_bus_device(struct pci_dev *p)
{
 assert(rescan_lock && !live_allocations());
 if (p->devfn == 2) {
  assert(!cen_stop(&beta));
  assert(!beta.arena && !owner.arena);
  owner.ops = NULL; owner.data = NULL;
  owner.beta = NULL; owner.beta_lease = false;
 } else if (p->devfn == 0 && p->dev.driver) {
  /* Driver core holds its device lock for .remove. */
  device_lock(&p->dev); owner.ref.count++;
  centauri_t6050_remove(p); device_unlock(&p->dev);
 }
 bus.slot[p->devfn] = NULL; removes++;
}

static void test_physical(void)
{
 fixture(); publish_all();
 assert(!centauri_t6050_quiesce(&control));
 assert(!live_allocations() && frees == 4 && !owner.arena && !control.ipc);
 assert(!wifi_users && !irq_users && !boot_users && !control.ready);
 physical_reset();
 thaw_and_run();
 printf("physical: refusals=%d error=%d old F1 reads=%d F2 reads=%d removes=%d rescans=%d\n",
        refusals, log_error, devs[1].reads, devs[2].reads, removes, rescans);
 assert(!refusals && removes == 3 && rescans == 1 && !recycles && frees == 4);
 /* cen_stop still makes its harmless best-effort clear during F2 removal. */
 assert(!devs[1].reads && devs[2].reads == 1);
 assert(!owner.control && !owner.beta && !owner.control_lease && !owner.beta_lease);
}

static void test_devices_only_and_aborted_suspend(void)
{
 for (int completed = 0; completed < 2; completed++) {
  fixture(); publish_all();
  if (completed) assert(!centauri_t6050_quiesce(&control));
  /* Link never went down; a thawed recovery must remove before recycle. */
  thaw_and_run();
  assert(!refusals && removes == 3 && rescans == 1 && recycles == 1);
  assert(!live_allocations() && frees == 4);
 }
 puts("devices-only and abort-before-F0: PASS");
}

static void only_tuple(int kind, bool independent)
{
 fixture();
 wifi_users = irq_users = boot_users = 0;
 control.removing = control.firmware_sleep = true; control.ready = false;
 control.irqs.pdev = control.alpha_irqs.pdev = NULL;
 beta.host_quiesced = true; beta.running = beta.irq_vectors = beta.ipc_requested = false;
 beta.hdev = NULL; beta.irq_count = beta.io_work = 0;
 if (independent) { devs[0].dev.driver = NULL; owner.ops = NULL; owner.data = NULL; }
 if (kind == 0) {
  ipc.buffer = allocation(0, 0);
  if (independent) owner.ipc = &ipc; else control.ipc = &ipc;
 } else if (kind == 1) {
  if (independent) owner.firmware = allocation(0, 0); else control.firmware = allocation(0, 0);
 } else if (kind == 2) {
  if (independent) owner.descriptor = allocation(0, 0); else control.descriptor = allocation(0, 0);
 } else { owner.arena = allocation(0, 2); if (!independent) beta.arena = owner.arena; }
}

static void test_each_retained_tuple(void)
{
 for (int independent = 0; independent < 2; independent++) for (int kind = 0; kind < 4; kind++) {
  only_tuple(kind, independent);
  /* Unknown hardware state must never become a synthetic fence receipt. */
  devs[1].visible = devs[2].visible = false;
  owner.error = -ETIMEDOUT;
  thaw_and_run();
  assert(refusals == 1 && log_error == -EBUSY && log_original == -ETIMEDOUT);
  assert(live_allocations() == 1 && !frees && !removes && !rescans && !recycles);
  assert(owner.control == &devs[0] && owner.beta == &devs[2]);
  assert(owner.control_lease && owner.beta_lease);
 }
 puts("eight individual bound/independent retained tuples: refuse and retain PASS");
}

static void test_pending_and_refusal_retry(void)
{
 for (int independent = 0; independent < 2; independent++) {
  only_tuple(1, independent); force_pending = true;
  thaw_and_run();
  assert(refusals == 1 && !frees && live_allocations() == 1 && !removes);
  force_pending = false; owner.active = true; owner.ref.count++;
  thaw_and_run();
  assert(refusals == 1 && frees == 1 && !live_allocations() && removes == 3 && rescans == 1);
 }
 puts("pending transaction refusal and same-owner retry: PASS");
}

static void test_lost_port_and_identity(void)
{
 fixture(); publish_all(); link_active = false;
 thaw_and_run();
 assert(refusals == 1 && log_error == -EBUSY && live_allocations() == 4 && !frees && !removes);
 fixture(); publish_all(); owner.control = &replacement;
 thaw_and_run();
 assert(refusals == 1 && log_error == -ENODEV && live_allocations() == 4 && !frees && !removes);
 fixture(); publish_all(); owner.beta = &replacement;
 thaw_and_run();
 assert(refusals == 1 && log_error == -ENODEV && live_allocations() == 4 && !frees && !removes);
 puts("lost port and changed Control/Beta identity: PASS");
}

static void test_callback_refusals(void)
{
 for (int fault = 0; fault < 3; fault++) {
  fixture(); publish_all();
  if (fault == 0) fail_beta_quiesce = true;
  if (fault == 1) fail_beta_release = true;
  if (fault == 2) module_available = false;
  thaw_and_run();
  assert(refusals == 1 && live_allocations() == 4 && !frees && !removes && !rescans);
 }
 puts("producer/release/module refusal cleanup: PASS");
}

int main(void)
{
 test_physical(); test_devices_only_and_aborted_suspend(); test_each_retained_tuple();
 test_pending_and_refusal_retry(); test_lost_port_and_identity(); test_callback_refusals();
 puts("PASS: production teardown/worker reset-domain and refusal suite");
 return 0;
}
