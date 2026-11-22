/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_APPLE_CENTAURI_H
#define _LINUX_APPLE_CENTAURI_H

#include <linux/dma-mapping.h>
#include <linux/types.h>

struct pci_dev;
struct module;

/*
 * Callbacks and register/unregister require fn->dev's device lock. No registry
 * lock is held across callbacks. quiesce drains producers while still bound;
 * release is called only after every function's DMA fence, and must leave its
 * original allocation tuple intact on failure. retained reports that tuple.
 */
struct apple_centauri_sleep_ops {
	struct module *owner;
	int (*quiesce)(void *data);
	int (*release)(void *data);
	bool (*retained)(void *data);
};
int apple_centauri_sleep_register(struct pci_dev *fn,
				  const struct apple_centauri_sleep_ops *ops, void *data,
				  void *arena, dma_addr_t dma, size_t size);
/*
 * Drops all callback/data references synchronously. If retained is true, the
 * cached arena and original DMA-domain lease transfer to the independent owner;
 * the caller must relinquish its local alias without freeing the allocation.
 * This covers driver unbind; arbitrary physical PCI device removal is outside
 * the retention contract. Managed recovery refuses removal with retained DMA.
 */
void apple_centauri_sleep_unregister(struct pci_dev *fn);
void apple_centauri_sleep_recovery_needed(struct pci_dev *fn, int error);

/* The caller must hold a managed device link to the Control supplier. */
bool apple_centauri_control_ready(struct pci_dev *pdev);
/*
 * True in a firmware sleep's suspend callbacks when the N1 (function fn's) is warm-reset and
 * booted again after the wake: its functions' DMA must all be released before the link goes down.
 */
bool apple_centauri_sleep_reboot(struct pci_dev *fn);
/* From a resume that found its function torn down by a suspend that stopped short */
void apple_centauri_sleep_rewake(struct pci_dev *fn);
/* Beta fell back to teardown during suspend; Control must also tear down and warm-reset. */
void apple_centauri_sleep_mark_fallback(struct pci_dev *fn);

#endif
