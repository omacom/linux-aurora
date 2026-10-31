// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "avd.h"

/* [C] Keep Linux's existing default until the idle-timer capture is available. */
#define AVD_AUTOSUSPEND_MS 100
#define AVD_DRAIN_TIMEOUT_MS 5000

static void avd_pm_disable_irq(struct avd_dev *avd)
{
	if (avd->pm.irq >= 0 && avd->pm.irq_enabled) {
		/* Wait out threaded handlers before the domain can disappear. */
		disable_irq(avd->pm.irq);
		avd->pm.irq_enabled = false;
	}
}

struct avd_dma_release {
	struct avd_deferred_unmap unmap;
	void *cpu;
	dma_addr_t addr;
	size_t size;
};

static void avd_dma_unmap(struct avd_dev *avd,
			  struct avd_deferred_unmap *unmap)
{
	struct avd_dma_release *release;

	release = container_of(unmap, struct avd_dma_release, unmap);
	dma_free_coherent(avd->dev, release->size, release->cpu, release->addr);
	kfree(release);
}

int avd_buf_alloc(struct avd_dev *avd, struct avd_buf *buf, size_t size)
{
	struct avd_dma_release *release;

	avd_buf_free(avd, buf);
	if (!size)
		return -ENOMEM;

	/* Allocate the release record now: freeing must never need to allocate. */
	release = kzalloc_obj(*release);
	if (!release)
		return -ENOMEM;
	INIT_LIST_HEAD(&release->unmap.node);
	release->unmap.unmap = avd_dma_unmap;
	release->size = size;
	release->cpu = dma_alloc_coherent(avd->dev, size, &release->addr, GFP_KERNEL);
	if (!release->cpu) {
		kfree(release);
		return -ENOMEM;
	}

	/* Payload length, not an allocation rounding, is used by codec packers. */
	buf->cpu = release->cpu;
	buf->addr = release->addr;
	buf->size = size;
	buf->release = release;
	return 0;
}

void avd_buf_free(struct avd_dev *avd, struct avd_buf *buf)
{
	if (buf->release)
		avd_pm_unmap(avd, &buf->release->unmap);
	/* The record outlives the context when unmapping has to be deferred. */
	memset(buf, 0, sizeof(*buf));
}

bool avd_pm_can_suspend(const struct avd_frame *frames, unsigned int draining)
{
	unsigned int i;

	if (draining)
		return false;

	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (frames[i].state != AVD_FRAME_FREE)
			return false;

	return true;
}

static bool avd_pm_queue_idle(struct avd_dev *avd)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&avd->queue_lock, flags);
	idle = avd_pm_can_suspend(avd->frames, avd->pm.draining);
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	return idle;
}

void avd_pm_queue_changed(struct avd_dev *avd)
{
	wake_up_all(&avd->pm.idle_wait);
}

void avd_pm_drain_begin(struct avd_dev *avd)
{
	unsigned long flags;

	spin_lock_irqsave(&avd->queue_lock, flags);
	avd->pm.draining++;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
}

void avd_pm_drain_end(struct avd_dev *avd)
{
	unsigned long flags;

	spin_lock_irqsave(&avd->queue_lock, flags);
	avd->pm.draining--;
	spin_unlock_irqrestore(&avd->queue_lock, flags);
	avd_pm_queue_changed(avd);
}

static void avd_pm_resubmit(struct avd_dev *avd)
{
	/* queue_mutex may be held by the caller of pm_runtime_resume_and_get(). */
	mod_delayed_work(system_wq, &avd->queue_work, 0);
}

static int avd_pm_queue_drain(struct avd_dev *avd)
{
	avd_pm_resubmit(avd);
	if (!wait_event_timeout(avd->pm.idle_wait, avd_pm_queue_idle(avd),
				msecs_to_jiffies(AVD_DRAIN_TIMEOUT_MS)))
		return -EBUSY;
	return 0;
}

const struct avd_queue_pm_ops avd_pm_queue_ops = {
	.idle = avd_pm_queue_idle,
	.drain = avd_pm_queue_drain,
	.resubmit = avd_pm_resubmit,
};

void avd_pm_init(struct avd_dev *avd, const struct avd_backend_ops *backend,
		 const struct avd_queue_pm_ops *queue)
{
	mutex_init(&avd->pm.lock);
	INIT_LIST_HEAD(&avd->pm.deferred_unmaps);
	init_waitqueue_head(&avd->pm.idle_wait);
	avd->pm.dart_active = false;
	avd->pm.irq = -1;
	avd->pm.irq_enabled = false;
	avd->backend = backend;
	avd->pm.queue = queue;
}

void avd_pm_enable(struct avd_dev *avd)
{
	pm_runtime_set_autosuspend_delay(avd->dev, AVD_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(avd->dev);
	pm_runtime_enable(avd->dev);
}

void avd_pm_disable(struct avd_dev *avd)
{
	int ret;

	/* Power suppliers before flushing frees queued by cleanup while idle. */
	ret = pm_runtime_resume_and_get(avd->dev);
	if (ret >= 0)
		ret = pm_runtime_put_sync_suspend(avd->dev);
	if (ret < 0)
		dev_warn(avd->dev, "final power transition failed: %d\n", ret);
	pm_runtime_disable(avd->dev);
	/* Also close the host IRQ if the final hardware transition failed. */
	avd_pm_disable_irq(avd);
	pm_runtime_dont_use_autosuspend(avd->dev);
	/* Resume may have kicked the already-stopped queue. */
	cancel_delayed_work_sync(&avd->queue_work);
	WARN_ON(!list_empty(&avd->pm.deferred_unmaps));
}

/* pm.lock held; release records own their storage and may free themselves. */
static void avd_pm_drain_unmaps(struct avd_dev *avd)
{
	struct avd_deferred_unmap *unmap, *next;

	list_for_each_entry_safe(unmap, next, &avd->pm.deferred_unmaps, node) {
		list_del_init(&unmap->node);
		unmap->unmap(avd, unmap);
	}
}

void avd_pm_unmap(struct avd_dev *avd, struct avd_deferred_unmap *unmap)
{
	mutex_lock(&avd->pm.lock);
	if (avd->pm.dart_active)
		unmap->unmap(avd, unmap);
	else
		list_add_tail(&unmap->node, &avd->pm.deferred_unmaps);
	mutex_unlock(&avd->pm.lock);
}

int avd_pm_backend_suspend(struct avd_dev *avd)
{
	avd->backend->shutdown(avd);
	return 0;
}

int avd_pm_backend_resume(struct avd_dev *avd)
{
	int ret;

	/* Keep initial image-based selection, but never change ABI after use. */
	if (!avd->fw_generation)
		return avd_backend_boot(avd);
	/* Apple boot re-verifies ITCM and relinks even when the image survived. */
	ret = avd->backend->boot(avd);

	if (!ret)
		avd->fw_generation++;
	return ret;
}

int avd_pm_power_on(struct avd_dev *avd)
{
	const struct avd_backend_ops *backend = avd->backend;
	int ret;

	/* genpd and the IOMMU suppliers resume before this runtime callback. */
	mutex_lock(&avd->pm.lock);
	avd->pm.dart_active = true;
	avd_pm_drain_unmaps(avd);
	mutex_unlock(&avd->pm.lock);

	/* Do not hold pm.lock: boot may allocate or release DMA buffers. */
	ret = backend->resume ? backend->resume(avd) : avd_pm_backend_resume(avd);
	if (ret) {
		avd->backend->shutdown(avd);
		mutex_lock(&avd->pm.lock);
		avd->pm.dart_active = false;
		mutex_unlock(&avd->pm.lock);
		return ret;
	}
	if (avd->pm.irq >= 0 && !avd->pm.irq_enabled) {
		enable_irq(avd->pm.irq);
		avd->pm.irq_enabled = true;
	}

	if (avd->pm.queue->resubmit)
		avd->pm.queue->resubmit(avd);

	return 0;
}

int avd_pm_power_off(struct avd_dev *avd)
{
	const struct avd_backend_ops *backend = avd->backend;
	int ret;

	/* PM references cover reservation through retirement, including drain. */
	if (!avd->pm.queue->idle(avd))
		return -EBUSY;

	if (backend->suspend) {
		ret = backend->suspend(avd);
		if (ret)
			return ret;
	} else {
		backend->shutdown(avd);
	}
	avd_pm_disable_irq(avd);

	/* No more firmware DMA; suppliers stay on until this callback returns. */
	mutex_lock(&avd->pm.lock);
	avd_pm_drain_unmaps(avd);
	avd->pm.dart_active = false;
	mutex_unlock(&avd->pm.lock);
	return 0;
}

static int avd_runtime_resume(struct device *dev)
{
	return avd_pm_power_on(dev_get_drvdata(dev));
}

static int avd_runtime_suspend(struct device *dev)
{
	return avd_pm_power_off(dev_get_drvdata(dev));
}

static int avd_system_suspend(struct device *dev)
{
	struct avd_dev *avd = dev_get_drvdata(dev);
	int ret;

	v4l2_m2m_suspend(avd->m2m_dev);
	ret = avd->pm.queue->drain(avd);
	if (!ret)
		ret = pm_runtime_force_suspend(dev);
	if (ret)
		v4l2_m2m_resume(avd->m2m_dev);
	return ret;
}

static int avd_system_resume(struct device *dev)
{
	struct avd_dev *avd = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;
	v4l2_m2m_resume(avd->m2m_dev);
	return 0;
}

const struct dev_pm_ops avd_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(avd_system_suspend, avd_system_resume)
	RUNTIME_PM_OPS(avd_runtime_suspend, avd_runtime_resume, NULL)
};
