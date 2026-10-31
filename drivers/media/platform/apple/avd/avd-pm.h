/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef AVD_PM_H_
#define AVD_PM_H_

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/wait.h>

#include "avd-queue.h"

struct avd_dma_release;

struct avd_queue_pm_ops {
	/* Snapshot under the queue lock: all FREE, no context draining. */
	bool (*idle)(struct avd_dev *avd);
	/* Scheduling is paused; finish/retire all accepted frames. May sleep. */
	int (*drain)(struct avd_dev *avd);
	/* Kick prepared RESERVED entries only, respecting dependencies. */
	void (*resubmit)(struct avd_dev *avd);
};

struct avd_deferred_unmap {
	struct list_head node;
	void (*unmap)(struct avd_dev *avd, struct avd_deferred_unmap *unmap);
};

struct avd_pm {
	/* Serializes DMA releases against DART activation and firmware boot. */
	struct mutex lock;
	struct list_head deferred_unmaps;
	bool dart_active;
	int irq;
	bool irq_enabled;
	/*
	 * irq_lock guards hw_on and irq_gen. The threaded handler touches the
	 * decoder only under irq_lock with hw_on set and an unchanged irq_gen,
	 * so turning the IRQ off never has to wait for the handler thread
	 * (which may itself be waiting for queue_mutex).
	 */
	spinlock_t irq_lock;
	bool hw_on;
	unsigned int irq_gen;
	void __iomem *piodma;
	phys_addr_t sram_phys;
	wait_queue_head_t idle_wait;
	/* Protected by queue_lock; counts drain operations, not stopped streams. */
	unsigned int draining;
	const struct avd_queue_pm_ops *queue;
};

/* The caller holds the queue lock, including the draining-context count. */
bool avd_pm_can_suspend(const struct avd_frame *frames, unsigned int draining);

void avd_pm_init(struct avd_dev *avd, const struct avd_backend_ops *backend,
		 const struct avd_queue_pm_ops *queue);
void avd_pm_enable(struct avd_dev *avd);
int avd_pm_map_piodma(struct avd_dev *avd);
void avd_pm_prepare_piodma(struct avd_dev *avd);
/* After queue cleanup: release deferred DMA mappings before the final off. */
void avd_pm_disable(struct avd_dev *avd);
/* Called in process context, after the last frame has retired its mapping. */
void avd_pm_unmap(struct avd_dev *avd, struct avd_deferred_unmap *unmap);
int avd_pm_power_on(struct avd_dev *avd);
int avd_pm_power_off(struct avd_dev *avd);
/* Caller holds a PM reference and queue_mutex; leaves the host IRQ closed. */
int avd_pm_recover(struct avd_dev *avd);
/* After recovery has invalidated every previously firmware-owned slot. */
void avd_pm_enable_irq(struct avd_dev *avd);
irqreturn_t avd_pm_irq(struct avd_dev *avd, unsigned int gen);
void avd_pm_drain_begin(struct avd_dev *avd);
void avd_pm_drain_end(struct avd_dev *avd);
void avd_pm_queue_changed(struct avd_dev *avd);
extern const struct avd_queue_pm_ops avd_pm_queue_ops;

/* Backends may share these when suspend loses all firmware context. */
int avd_pm_backend_suspend(struct avd_dev *avd);
int avd_pm_backend_resume(struct avd_dev *avd);
extern const struct dev_pm_ops avd_pm_ops;

#endif /* AVD_PM_H_ */
