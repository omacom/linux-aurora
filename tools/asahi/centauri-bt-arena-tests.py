#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""CPU fault injection for actual Centauri Bluetooth ownership helpers.

This compiles the driver functions themselves with kernel-operation stubs. It
checks host-side ordering and retention only; it cannot prove physical DMA fences,
HCI-core concurrency, firmware behavior, or the Control owner's lifetime protocol.
Run directly; requires a C compiler and no kernel build or target hardware.
"""
import argparse
import os
import resource
import shlex
import signal
from pathlib import Path
import subprocess
import tempfile

SOURCE = Path(__file__).resolve().parents[2] / "drivers/bluetooth/btcentauri_t6050.c"


def function(source, name):
    # The selected helpers contain no braces in strings or comments.
    import re
    match = re.search(r"^static [^\n]+\b" + name + r"\([^;]+?\)\n\{", source, re.M)
    if not match:
        raise RuntimeError(f"missing driver helper: {name}")
    start = match.start()
    pos = source.index("{", start)
    depth = 1
    end = pos + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


STUBS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
typedef uint16_t u16;
#define PCI_COMMAND 4
#define PCI_COMMAND_MASTER 4
#define CEN_ARENA_SIZE (1024 * 1024)
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
struct device { int dummy; };
struct pci_dev { struct device dev; void *data; };
struct hci_dev { int dummy; };
struct cen_bt {
    struct pci_dev *pdev;
    struct hci_dev *hdev;
    void *arena, *bar;
    uintptr_t dma;
    int lock, io_work, txq, acl_txq, le_txq;
    unsigned int irq_count;
    bool irq_vectors, running, quiescing, host_quiesced;
    bool hci_registered, ipc_requested, asleep, sleep_registered;
    bool transport_paused, transport_sleep_sent;
    int suspend_error;
};
static struct cen_bt *active;
static int config_error, pending, command, saves, save_error, requests, recovery_error;
static int unregisters, closes, hci_frees, arena_frees, irq_frees;
static int owner_unregisters, handoffs, fallbacks, transport_sleeps, transport_wakes;
static int transport_sleep_error;
static unsigned int bar_exec, bar_ipc;
static void *handed_arena;
static uintptr_t handed_dma;
static bool sleep_reboot;
static void mutex_lock(int *p) { (void)p; }
static void mutex_unlock(int *p) { (void)p; }
static void cancel_work_sync(int *p) { *p = 0; }
static void schedule_work(int *p) { *p = 1; }
static int work_busy(int *p) { return *p; }
static void skb_queue_purge(int *p) { *p = 0; }
static unsigned int pci_irq_vector(struct pci_dev *p, unsigned int nr) { (void)p; return nr; }
static void synchronize_irq(unsigned int irq) { (void)irq; }
static unsigned int cen_mock_readl(void *addr) {
    if (!active || !active->bar)
        return 0;
    return (char *)addr - (char *)active->bar == 0x18 ? bar_exec : bar_ipc;
}
#define readl cen_mock_readl
static int cen_transport_state(struct cen_bt *bt, bool asleep) {
    assert(bt == active && bt->arena && bt->bar);
    if (asleep) {
        transport_sleeps++;
        return transport_sleep_error;
    }
    transport_wakes++;
    return 0;
}
static void apple_centauri_sleep_mark_fallback(struct pci_dev *p) {
    assert(p == active->pdev);
    fallbacks++;
}
static void *hci_get_drvdata(struct hci_dev *p) { assert(p); return active; }
static void hci_unregister_dev(struct hci_dev *p) {
    assert(p && active->arena && active->quiescing);
    /* The arena must remain available through HCI's close barrier. */
    unregisters++;
}
static int cen_hci_close(struct hci_dev *p) {
    assert(p && active->arena);
    active->running = false;
    closes++;
    return 0;
}
static void hci_free_dev(struct hci_dev *p) {
    assert(p && !active->running && !active->io_work && !active->irq_count);
    hci_frees++;
}
static void cen_free_irqs(struct cen_bt *bt) {
    if (bt->irq_count || bt->irq_vectors) irq_frees++;
    bt->irq_count = 0;
    bt->irq_vectors = false;
    /* Model a final handler enqueuing I/O before free_irq returns. */
    bt->io_work = 1;
}
static int pci_read_config_word(struct pci_dev *p, int reg, u16 *out) {
    assert(p == active->pdev && reg == PCI_COMMAND);
    *out = command;
    return config_error;
}
static int pcibios_err_to_errno(int error) { return error ? -EIO : 0; }
static bool pci_wait_for_pending_transaction(struct pci_dev *p) {
    assert(p == active->pdev);
    return pending;
}
static void pci_clear_master(struct pci_dev *p) {
    assert(p == active->pdev);
    command &= ~PCI_COMMAND_MASTER;
}
static void dma_free_coherent(struct device *dev, int size, void *cpu, uintptr_t dma) {
    assert(dev == &active->pdev->dev && size == CEN_ARENA_SIZE);
    assert(cpu == active->arena && dma == active->dma && cpu && dma);
    assert(active->host_quiesced && !active->hdev && !active->io_work);
    arena_frees++;
}
static struct pci_dev *to_pci_dev(struct device *dev) { return (void *)dev; }
static void *pci_get_drvdata(struct pci_dev *p) { return p->data; }
static bool apple_centauri_sleep_reboot(struct pci_dev *p) { (void)p; return sleep_reboot; }
static void apple_centauri_sleep_recovery_needed(struct pci_dev *p, int error) {
    (void)p; requests++;
    if (!recovery_error) recovery_error = error;
}
static void apple_centauri_sleep_unregister(struct pci_dev *p) {
    assert(p == active->pdev && active->host_quiesced);
    owner_unregisters++;
    if (active->arena) {
        assert(active->ipc_requested && active->dma && !active->hdev);
        handed_arena = active->arena;
        handed_dma = active->dma;
        handoffs++;
    }
}
static int pci_save_state(struct pci_dev *p) { (void)p; saves++; return save_error; }
#define dev_warn(...) ((void)0)
'''

TESTS = r'''
static struct pci_dev dev;
static struct hci_dev hdev;
static struct cen_bt bt;
static void reset(void) {
    memset(&bt, 0, sizeof(bt));
    active = &bt;
    bt.pdev = &dev; dev.data = &bt;
    bt.hdev = &hdev; bt.hci_registered = true;
    bt.arena = (void *)0x1000; bt.dma = 0x2000;
    bt.running = true; bt.ipc_requested = true; bt.sleep_registered = true;
    bt.irq_count = 2; bt.irq_vectors = true;
    bt.txq = bt.acl_txq = bt.le_txq = 1;
    pending = 1; command = PCI_COMMAND_MASTER;
    config_error = saves = save_error = requests = recovery_error = 0;
    unregisters = closes = hci_frees = arena_frees = irq_frees = 0;
    owner_unregisters = handoffs = fallbacks = transport_sleeps = transport_wakes = 0;
    transport_sleep_error = 0;
    bar_exec = bar_ipc = 2;
    handed_arena = NULL; handed_dma = 0;
    sleep_reboot = true;
}
static void stopped(void) {
    assert(cen_quiesce(&bt) == 0);
    assert(bt.arena && bt.dma && bt.ipc_requested);
    assert(bt.host_quiesced && !bt.hdev && !bt.running);
    assert(!bt.io_work && !bt.txq && !bt.acl_txq && !bt.le_txq);
    assert(unregisters == 1 && hci_frees == 1 && irq_frees == 1);
    assert(cen_quiesce(&bt) == 0);
    assert(unregisters == 1 && hci_frees == 1 && irq_frees == 1);
}
int main(void) {
    reset();
    assert(cen_release(&bt) == -EBUSY && cen_retained(&bt));
    stopped();
    /* Every local fence failure retains the exact published allocation tuple. */
    assert(cen_release(&bt) == -EBUSY);
    command = 0; config_error = 1;
    assert(cen_release(&bt) == -EIO);
    config_error = 0; pending = 0;
    assert(cen_release(&bt) == -EBUSY);
    assert(bt.arena == (void *)0x1000 && bt.dma == 0x2000);
    assert(bt.ipc_requested && arena_frees == 0);
    pending = 1; bt.io_work = 1;
    assert(cen_release(&bt) == -EBUSY);
    bt.io_work = 0; bt.irq_vectors = true;
    assert(cen_release(&bt) == -EBUSY);
    bt.irq_vectors = false;
    assert(cen_release(&bt) == 0);
    assert(!cen_retained(&bt) && !bt.dma && !bt.ipc_requested);
    assert(cen_release(&bt) == 0 && arena_frees == 1);
    /* Closed transport admission cannot be reopened after host quiesce. */
    assert(cen_hci_open(&hdev) == -ENETDOWN && !bt.running);
    /* Successful local suspend leaves release to the all-function owner. */
    reset();
    assert(cen_suspend(&dev.dev) == 0);
    assert(bt.asleep && bt.arena && arena_frees == 0 && requests == 0);
    assert(saves == 1);
    /* Final save refusal must request recovery even if own resume is skipped. */
    reset(); save_error = -ENOMEM;
    assert(cen_suspend(&dev.dev) == -ENOMEM);
    assert(bt.asleep && bt.arena && requests == 1 && recovery_error == -ENOMEM);
    save_error = 0;
    assert(cen_suspend(&dev.dev) == -ENOMEM && requests == 2);
    /* A later save failure must not overwrite the original local fence error. */
    reset(); config_error = 1; save_error = -ENOMEM;
    assert(cen_suspend(&dev.dev) == -EIO);
    assert(bt.arena && bt.dma == 0x2000 && requests == 1 && saves == 1);
    assert(recovery_error == -EIO);
    /* An HCI device that was already closed still unregisters exactly once. */
    reset(); bt.running = false;
    stopped();
    /* Unpublished partial-probe resources can be reclaimed locally. */
    reset(); bt.hdev = NULL; bt.hci_registered = false; bt.ipc_requested = false;
    config_error = 1; pending = 0;
    assert(cen_stop(&bt) == 0 && arena_frees == 1);
    /* Unbind of suspended Beta hands its tuple over before devm destruction. */
    reset(); save_error = -ENOMEM;
    assert(cen_suspend(&dev.dev) == -ENOMEM);
    cen_remove(&dev);
    assert(owner_unregisters == 1 && handoffs == 1 && arena_frees == 0);
    assert(handed_arena == (void *)0x1000 && handed_dma == 0x2000);
    assert(!bt.arena && !bt.dma && !bt.sleep_registered);
    assert(unregisters == 1 && hci_frees == 1 && recovery_error == -ENOMEM);
    /* Managed recovery releases while bound; final remove consumes that state. */
    reset(); stopped(); command = 0;
    assert(cen_release(&bt) == 0);
    cen_remove(&dev);
    assert(owner_unregisters == 1 && handoffs == 0 && arena_frees == 1);
    assert(unregisters == 1 && hci_frees == 1 && requests == 0);
    /* Healthy keep-alive ACIPC host-sleep preserves HCI and arena. */
    reset(); bt.bar = (void *)0x3000;
    assert(cen_suspend(&dev.dev) == 0);
    assert(bt.transport_sleep_sent && !bt.asleep && bt.hdev == &hdev);
    assert(bt.arena && arena_frees == 0 && transport_sleeps == 1 && fallbacks == 0);
    assert(saves == 1 && requests == 0);
    /* Keep-alive save failure unwinds ACIPC sleep and falls back to teardown. */
    reset(); bt.bar = (void *)0x3000; save_error = -ENOMEM;
    assert(cen_suspend(&dev.dev) == -ENOMEM);
    assert(!bt.transport_sleep_sent && bt.asleep && bt.arena && !bt.hdev);
    assert(transport_sleeps == 1 && transport_wakes == 1 && fallbacks == 1);
    assert(requests == 1 && recovery_error == -ENOMEM);
    /* Keep-alive ACIPC failure falls back cleanly to teardown. */
    reset(); bt.bar = (void *)0x3000; transport_sleep_error = -EIO;
    assert(cen_suspend(&dev.dev) == 0);
    assert(!bt.transport_sleep_sent && bt.asleep && bt.arena && !bt.hdev);
    assert(transport_sleeps == 1 && transport_wakes == 0 && fallbacks == 1);
    assert(saves == 1 && requests == 0);
    /* A published arena cannot be reclaimed by a local-only detach. */
    reset();
    assert(cen_stop(&bt) == -EBUSY && bt.arena && arena_frees == 0);
    puts("Centauri Bluetooth ownership fault injection: PASS");
    return 0;
}
'''


# The entire production probe is compiled, including its real registration
# branch and stop/unregister unwind. Only external kernel operations are stubbed.
PROBE_ENV = r"""
typedef unsigned char u8;
struct pci_device_id { int dummy; };
struct device_link { int dummy; };
struct pci_host_bridge { struct device dev; };
#define GFP_KERNEL 0
#define SZ_64K 65536
#define EPROBE_DEFER 517
#define BIT(n) (1U << (n))
#define DMA_BIT_MASK(n) ((1ULL << (n)) - 1)
#define PCI_FUNC(n) ((n) & 7)
#define PCI_SLOT(n) ((n) >> 3)
#define PCI_DEVFN(slot, fn) (((slot) << 3) | (fn))
#define DL_FLAG_AUTOREMOVE_CONSUMER 0
#define HCI_PCI 1
#define HCI_QUIRK_FIXUP_LE_EXT_ADV_REPORT_PHY 0
#define INIT_WORK(work, fn) (*(work) = 0)
#define SET_HCIDEV_DEV(hdev, dev) ((void)(hdev), (void)(dev))
#define dev_err(...) ((void)0)
#define dev_err_probe(dev, error, ...) (error)
static const int cen_sleep_ops;
static struct device host_parent;
static struct pci_host_bridge host = { .dev.parent = &host_parent };
static struct device_link link;
static struct pci_dev control;
static struct hci_dev probe_hdev;
static u8 probe_arena[CEN_ARENA_SIZE];
static int probe_register_result;
static struct pci_host_bridge *pci_find_host_bridge(struct pci_bus *bus) { (void)bus; return &host; }
static bool of_device_is_compatible(void *node, const char *name) { (void)node; (void)name; return true; }
static int pci_domain_nr(struct pci_bus *bus) { (void)bus; return 0; }
static struct pci_dev *pci_get_domain_bus_and_slot(int domain, int bus, int fn) {
    (void)domain; (void)bus; (void)fn; return &control;
}
static void pci_dev_put(struct pci_dev *p) { (void)p; }
static struct device_link *device_link_add(struct device *dev, struct device *supplier, int flags) {
    (void)dev; (void)supplier; (void)flags; return &link;
}
static bool apple_centauri_control_ready(struct pci_dev *p) { (void)p; return true; }
static void *devm_kzalloc(struct device *dev, size_t size, int flags) {
    (void)dev; (void)flags; assert(size == sizeof(*active));
    memset(active, 0, size); return active;
}
static void mutex_init(int *lock) { *lock = 0; }
static void skb_queue_head_init(int *queue) { *queue = 0; }
static int pcim_enable_device(struct pci_dev *p) { (void)p; return 0; }
static size_t pci_resource_len(struct pci_dev *p, int bar) { (void)p; (void)bar; return SZ_64K; }
static int pcim_iomap_regions(struct pci_dev *p, int bars, const char *name) {
    (void)p; (void)bars; (void)name; return 0;
}
static void **pcim_iomap_table(struct pci_dev *p) {
    static void *bars[] = { (void *)0x3000 }; (void)p; return bars;
}
static unsigned int probe_readl(void *bar) { return (uintptr_t)bar == 0x3018 ? 2 : 0; }
#undef readl
#define readl probe_readl
static int dma_set_mask_and_coherent(struct device *dev, uint64_t mask) { (void)dev; (void)mask; return 0; }
static void *dma_alloc_coherent(struct device *dev, size_t size, uintptr_t *dma, int flags) {
    (void)dev; (void)flags; assert(size == sizeof(probe_arena)); *dma = 0x2000; return probe_arena;
}
static void pci_set_drvdata(struct pci_dev *p, void *data) { p->data = data; }
static int apple_centauri_sleep_register(struct pci_dev *p, const int *ops, void *data,
                                        void *arena, uintptr_t dma, size_t size) {
    assert(p == active->pdev && ops == &cen_sleep_ops && data == active);
    assert(arena == probe_arena && dma == 0x2000 && size == CEN_ARENA_SIZE); return 0;
}
static int cen_request_irqs(struct cen_bt *bt) { bt->irq_count = 2; bt->irq_vectors = true; return 0; }
static int cen_start(struct cen_bt *bt) { bt->ipc_requested = true; return 0; }
static struct hci_dev *hci_alloc_dev(void) { return &probe_hdev; }
static int cen_hci_flush(struct hci_dev *dev) { (void)dev; return 0; }
static int cen_hci_send(struct hci_dev *dev, struct sk_buff *skb) { (void)dev; (void)skb; return 0; }
static int cen_hci_setup(struct hci_dev *dev) { (void)dev; return 0; }
static int cen_hci_post_init(struct hci_dev *dev) { (void)dev; return 0; }
static int cen_hci_reset_dup_filter(struct hci_dev *dev) { (void)dev; return 0; }
static void hci_set_quirk(struct hci_dev *dev, int quirk) { (void)dev; (void)quirk; }
static void hci_set_drvdata(struct hci_dev *dev, void *data) { (void)dev; assert(data == active); }
static int hci_register_dev(struct hci_dev *dev) {
    assert(dev == &probe_hdev && !probe_hci_registered);
    if (probe_register_result >= 0) probe_hci_registered = true;
    return probe_register_result;
}
"""

PROBE_TESTS = r"""
int main(void) {
    static struct cen_bt bt;
    struct pci_bus bus = { .number = 1 };
    struct pci_dev dev = { .bus = &bus, .devfn = 2 };
    const int ids[] = { 0, 1, 7 };
    active = &bt;
    for (unsigned int i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        probe_register_result = ids[i];
        unregisters = hci_frees = closes = irq_frees = 0;
        assert(cen_probe(&dev, NULL) == 0);
        assert(bt.hci_registered && probe_hci_registered && bt.hdev == &probe_hdev);
        assert(unregisters == 0 && hci_frees == 0);
        assert(cen_hci_open(bt.hdev) == 0);
        assert(cen_quiesce(&bt) == 0);
        assert(cen_quiesce(&bt) == 0);
        assert(unregisters == 1 && hci_frees == 1 && !probe_hci_registered);
    }
    probe_register_result = -ENOSPC;
    unregisters = hci_frees = owner_unregisters = requests = recovery_error = 0;
    assert(cen_probe(&dev, NULL) == -ENOSPC);
    assert(!bt.hci_registered && !probe_hci_registered && !bt.hdev);
    assert(unregisters == 0 && hci_frees == 1);
    assert(owner_unregisters == 1 && requests == 1 && recovery_error == -ENOSPC);
    assert(handoffs == 1 && handed_arena == probe_arena && handed_dma == 0x2000);
    assert(saves == 0 && arena_frees == 0);
    puts("Centauri production probe HCI IDs 0/1/7 and negative error: PASS");
    return 0;
}
"""


def probe_stubs():
    stubs = STUBS.replace('struct device { int dummy; };',
                         'struct device { struct device *parent; void *of_node; };')
    stubs = stubs.replace('struct pci_dev { struct device dev; void *data; };',
                         'struct pci_bus { int number; };\n'
                         'struct pci_dev { struct device dev; void *data; struct pci_bus *bus; int devfn; };')
    stubs = stubs.replace('struct hci_dev { int dummy; };', 'struct sk_buff;\nstruct hci_dev {\n    int bus;\n    int (*open)(struct hci_dev *), (*close)(struct hci_dev *);\n    int (*flush)(struct hci_dev *), (*setup)(struct hci_dev *);\n    int (*post_init)(struct hci_dev *), (*reset_dup_filter)(struct hci_dev *);\n    int (*send)(struct hci_dev *, struct sk_buff *);\n};')
    stubs = stubs.replace('static struct cen_bt *active;',
                         'static struct cen_bt *active;\nstatic bool probe_hci_registered;')
    stubs = stubs.replace('    unregisters++;',
                         '    assert(probe_hci_registered); probe_hci_registered = false;\n    unregisters++;')
    stubs = stubs.replace('    hci_frees++;', '    assert(!probe_hci_registered);\n    hci_frees++;')
    return stubs + PROBE_ENV


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument("--negative-controls", action="store_true")
    args = parser.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    source = SOURCE.read_text()
    helpers = "\n".join(function(source, name) for name in (
        "cen_hci_open", "cen_quiesce", "cen_dma_stopped", "cen_release",
        "cen_retained", "cen_stop", "cen_suspend", "cen_remove"))
    with tempfile.TemporaryDirectory(prefix="centauri-bt-test-") as tmp:
        out = args.out or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        compiler = shlex.split(os.environ.get('CC', 'cc'))

        def run(name, body, stubs=STUBS, tests=TESTS, flags=()):
            c, exe = out / (name + '.c'), out / name
            c.write_text(stubs + body + tests)
            subprocess.run(compiler + ['-std=gnu11', '-Wall', '-Wextra', '-Werror',
                                      '-O0'] + list(flags) + [str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            (out / (name + '.log')).write_text(result.stdout + result.stderr)
            return result

        result = run('production', helpers)
        if result.returncode:
            raise SystemExit(result.stdout + result.stderr)
        print(result.stdout, end='')
        probe_helpers = "\n".join(function(source, name) for name in (
            'cen_hci_open', 'cen_quiesce', 'cen_dma_stopped', 'cen_release',
            'cen_stop', 'cen_probe'))
        # Shared stub definitions also serve the ownership helpers; the full
        # probe's unused kernel callback arguments are normal production code.
        probe_flags = ('-Wno-unused-parameter', '-Wno-unused-function')
        result = run('production-probe', probe_helpers, probe_stubs(), PROBE_TESTS, probe_flags)
        if result.returncode:
            raise SystemExit(result.stdout + result.stderr)
        print(result.stdout, end='')
        if args.negative_controls:
            old = 'ret = hci_register_dev(hdev);\n\tif (ret < 0)'
            assert probe_helpers.count(old) == 1
            result = run('positive-hci-id-rejected', probe_helpers.replace(
                old, 'ret = hci_register_dev(hdev);\n\tif (ret)'),
                probe_stubs(), PROBE_TESTS, probe_flags)
            assert (result.returncode == -signal.SIGABRT
                    and '!probe_hci_registered' in result.stderr), result.stderr
            print('Runtime negative control: positive-hci-id-rejected: PASS')
            controls = {
                "lost-retained-tuple": (
                    "save_ret = pci_save_state(pdev);",
                    "bt->arena = NULL; save_ret = pci_save_state(pdev);"),
                "missing-pending-fence": (
                    "if (!pci_wait_for_pending_transaction(bt->pdev))",
                    "if (false && !pci_wait_for_pending_transaction(bt->pdev))"),
                "missing-save-recovery": (
                    "apple_centauri_sleep_recovery_needed(pdev, bt->suspend_error);",
                    "(void)pdev;"),
                "missing-final-work-drain": (
                    "cancel_work_sync(&bt->io_work);\n\tskb_queue_purge(&bt->txq);",
                    "if (false) cancel_work_sync(&bt->io_work);\n\tskb_queue_purge(&bt->txq);"),
            }
            failures = {
                'lost-retained-tuple': 'bt.asleep && bt.arena && arena_frees == 0 && requests == 0',
                'missing-pending-fence': 'cen_release(&bt) == -EBUSY',
                'missing-save-recovery': 'bt.asleep && bt.arena && requests == 1 && recovery_error == -ENOMEM',
                'missing-final-work-drain': 'p && !active->running && !active->io_work && !active->irq_count',
            }
            for name, (old, new) in controls.items():
                assert helpers.count(old) == 1, name
                result = run(name, helpers.replace(old, new))
                assert (result.returncode == -signal.SIGABRT and "Assertion" in result.stderr
                        and failures[name] in result.stderr), (
                    name, result.returncode, result.stderr)
                print(f"Runtime negative control: {name}: PASS")


if __name__ == "__main__":
    main()
