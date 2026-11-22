#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Adapted from betizzel/omarchy-m5 patches/linux-aurora-n1/smoke-bootstrap.py.
"""Run the actual Centauri PCI enumeration against a deterministic probe race.

No hardware access. The driver-core fixture holds Beta's device lock with
its driver pointer set, as a probe which will return -EPROBE_DEFER does.
The enumeration thread must wait for that probe, not report permanent
ownership and abort shared Wi-Fi/Bluetooth startup. Also cover genuine
ownership and lock release after early/late resource failures.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

PRELUDE = r"""
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct device {
    pthread_mutex_t mutex;
    _Atomic(void *) driver;
    int function;
};
struct pci_bus { int number; };
struct pci_dev {
    struct device dev;
    struct pci_bus *bus;
    uint16_t vendor, device;
    unsigned cfg_size, devfn, multifunction;
    bool master;
};
struct centauri_t6050_ipc {
    struct pci_dev *function[3];
    void *bar[3];
    bool owned[3], enabled[3], domain_lease[3], alpha_pm_domain;
};
struct centauri_t6050 {
    struct pci_dev *pdev;
    struct centauri_t6050_ipc *ipc;
    int alpha_irqs;
};
static struct pci_bus bus;
static struct pci_dev functions[3];
static struct centauri_t6050_ipc ipc;
static struct centauri_t6050 control;
static int centauri_t6050_alpha_pm_domain;
static int bound_driver;
static enum { NONE, DOMAIN_FAILURE, MAP_FAILURE, ENABLE_FAILURE, DMA_FAILURE, IRQ_FAILURE } fault;
static int domain_users[3];
static pthread_mutex_t gate = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t changed = PTHREAD_COND_INITIALIZER;
static bool beta_lock_attempted, enumeration_done;
static int enumeration_result;

#define PCI_VENDOR_ID_APPLE 0x106b
#define IORESOURCE_MEM 1
#define DMA_BIT_MASK(bits) ((UINT64_C(1) << (bits)) - 1)
#define PCI_SLOT(devfn) ((devfn) >> 3)
#define PCI_DEVFN(slot, function) (((slot) << 3) | (function))
#define pci_domain_nr(bus) 0
#define pci_rescan_bus(bus) ((void)0)
#define pci_resource_flags(pdev, bar) IORESOURCE_MEM
#define pci_resource_len(pdev, bar) 0x10000
#define centauri_t6050_alpha_irq 0

static void device_lock(struct device *dev)
{
    assert(dev->function != 0);
    if (dev->function == 2) {
        assert(!pthread_mutex_lock(&gate));
        beta_lock_attempted = true;
        assert(!pthread_cond_signal(&changed));
        assert(!pthread_mutex_unlock(&gate));
    }
    assert(!pthread_mutex_lock(&dev->mutex));
}
static void device_unlock(struct device *dev)
{
    assert(!pthread_mutex_unlock(&dev->mutex));
}
static struct pci_dev *pci_get_domain_bus_and_slot(int domain, int busno, int devfn)
{
    (void)domain;
    (void)busno;
    assert(devfn >= 0 && devfn < 3);
    return &functions[devfn];
}
static int iommu_device_use_default_domain(struct device *dev)
{
    /* A sibling's DMA domain is claimed only under its device lock. */
    assert(dev->function != 0);
    assert(pthread_mutex_trylock(&dev->mutex) == EBUSY);
    if (fault == DOMAIN_FAILURE && dev->function == 2)
        return -EBUSY;
    domain_users[dev->function]++;
    return 0;
}
static void *pci_iomap(struct pci_dev *pdev, int bar, int length)
{
    (void)bar;
    (void)length;
    return fault == MAP_FAILURE && pdev->dev.function == 1 ? NULL : pdev;
}
static int pci_enable_device_mem(struct pci_dev *pdev)
{
    return fault == ENABLE_FAILURE && pdev->dev.function == 1 ? -EIO : 0;
}
static void dev_pm_domain_set(struct device *dev, int *domain)
{
    assert(pthread_mutex_trylock(&dev->mutex) == EBUSY);
    (void)domain;
}
static int dma_set_mask_and_coherent(struct device *dev, uint64_t mask)
{
    (void)mask;
    return fault == DMA_FAILURE && dev->function == 2 ? -ERANGE : 0;
}
static int centauri_t6050_request_irqs(int *irqs, struct pci_dev *pdev,
                                 unsigned count, int handler, void *data)
{
    (void)irqs;
    (void)pdev;
    (void)count;
    (void)handler;
    (void)data;
    return fault == IRQ_FAILURE ? -ENOSPC : 0;
}
static void pci_set_master(struct pci_dev *pdev)
{
    pdev->master = true;
}
"""

TESTS = r"""
static void reset_fixture(void)
{
    memset(&ipc, 0, sizeof(ipc));
    memset(domain_users, 0, sizeof(domain_users));
    fault = NONE;
    beta_lock_attempted = enumeration_done = false;
    for (int i = 0; i < 3; i++) {
        functions[i].dev.driver = NULL;
        functions[i].master = false;
    }
}
static void assert_sibling_locks_released(void)
{
    for (int i = 1; i < 3; i++) {
        assert(!pthread_mutex_trylock(&functions[i].dev.mutex));
        assert(!pthread_mutex_unlock(&functions[i].dev.mutex));
    }
}
static void *enumeration_thread(void *unused)
{
    (void)unused;
    enumeration_result = enumerate_functions(&control);
    assert(!pthread_mutex_lock(&gate));
    enumeration_done = true;
    assert(!pthread_cond_signal(&changed));
    assert(!pthread_mutex_unlock(&gate));
    return NULL;
}
int main(void)
{
    pthread_t thread;

    for (int i = 0; i < 3; i++) {
        functions[i].dev.function = i;
        assert(!pthread_mutex_init(&functions[i].dev.mutex, NULL));
        functions[i].bus = &bus;
        functions[i].vendor = PCI_VENDOR_ID_APPLE;
        functions[i].device = i ? 0x1901 + i : 0x1900;
        functions[i].cfg_size = 0x1000;
        functions[i].devfn = i;
    }
    control.pdev = &functions[0];
    control.ipc = &ipc;

    reset_fixture();
    /* Emulate driver core inside Beta's not-ready probe. No sleeps or races. */
    assert(!pthread_mutex_lock(&functions[2].dev.mutex));
    functions[2].dev.driver = &bound_driver;
    assert(!pthread_create(&thread, NULL, enumeration_thread, NULL));
    assert(!pthread_mutex_lock(&gate));
    while (!beta_lock_attempted && !enumeration_done)
        assert(!pthread_cond_wait(&changed, &gate));
    assert(!pthread_mutex_unlock(&gate));
    /* Returning -EPROBE_DEFER clears dev.driver before unlocking the device. */
    functions[2].dev.driver = NULL;
    assert(!pthread_mutex_unlock(&functions[2].dev.mutex));
    assert(!pthread_join(thread, NULL));
    if (enumeration_result) {
        fprintf(stderr, "FAIL: deferred Beta probe aborted IPC: %d\n", enumeration_result);
        return 1;
    }
    for (int i = 0; i < 3; i++)
        assert(ipc.owned[i] && ipc.bar[i] && functions[i].master);
    assert(ipc.enabled[1] && ipc.enabled[2]);
    /* F0's lease belongs to the Control owner; siblings take exactly one each. */
    assert(!ipc.domain_lease[0] && ipc.domain_lease[1] && ipc.domain_lease[2]);
    assert(!domain_users[0] && domain_users[1] == 1 && domain_users[2] == 1);
    assert_sibling_locks_released();
    puts("PASS: concurrent deferred Beta probe does not abort shared startup");

    reset_fixture();
    functions[2].dev.driver = &bound_driver;
    assert(enumerate_functions(&control) == -EBUSY);
    assert(!ipc.owned[2] && !ipc.bar[2] && !functions[2].master);
    assert(functions[2].dev.driver == &bound_driver);
    assert(!ipc.domain_lease[2] && !domain_users[2]);
    assert_sibling_locks_released();
    puts("PASS: genuinely bound Beta is rejected without taking ownership");

    reset_fixture();
    fault = DOMAIN_FAILURE;
    assert(enumerate_functions(&control) == -EBUSY);
    assert(ipc.owned[1] && ipc.domain_lease[1] && domain_users[1] == 1);
    assert(!ipc.owned[2] && !ipc.domain_lease[2] && !ipc.bar[2] && !functions[2].master);
    assert_sibling_locks_released();
    puts("PASS: DMA-domain lease failure takes no ownership and releases the lock");

    reset_fixture();
    fault = MAP_FAILURE;
    assert(enumerate_functions(&control) == -ENOMEM);
    assert(!ipc.enabled[1] && !functions[1].master);
    assert_sibling_locks_released();
    puts("PASS: early mapping failure releases the sibling device lock");

    reset_fixture();
    fault = ENABLE_FAILURE;
    assert(enumerate_functions(&control) == -EIO);
    assert(ipc.owned[1] && ipc.bar[1] && !ipc.enabled[1] && !functions[1].master);
    assert_sibling_locks_released();
    puts("PASS: device-enable failure releases the sibling device lock");

    reset_fixture();
    fault = DMA_FAILURE;
    assert(enumerate_functions(&control) == -ERANGE);
    assert(ipc.owned[2] && ipc.enabled[2] && !functions[2].master);
    assert_sibling_locks_released();
    puts("PASS: DMA-mask failure releases the sibling device lock");

    reset_fixture();
    fault = IRQ_FAILURE;
    assert(enumerate_functions(&control) == -ENOSPC);
    assert(ipc.enabled[1] && !functions[1].master && !ipc.owned[2]);
    assert_sibling_locks_released();
    puts("PASS: IRQ allocation failure releases the sibling device lock");
    return 0;
}
"""


def main():
    root = Path(__file__).resolve().parents[5]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=root)
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    path = args.source / "drivers/net/wireless/apple/centauri/t6050/centauri_ipc.c"
    source = path.read_text()
    start = source.index("static int enumerate_functions(")
    end = source.index("\nstatic int centauri_t6050_start_ipc_locked(", start)
    with tempfile.TemporaryDirectory(prefix="centauri-bootstrap-smoke-") as directory:
        directory = Path(directory)
        unit = directory / "bootstrap.c"
        unit.write_text(PRELUDE + source[start:end] + TESTS)
        binary = directory / "bootstrap"
        subprocess.run([args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-function", "-pthread", str(unit), "-o", str(binary)],
                       check=True)
        subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    main()
