// SPDX-License-Identifier: GPL-2.0-only OR MIT
//
// Per-device sysfs shim. Rust owns the snapshot through an Arc in the bound
// driver. The attribute wrapper is removed and its readers drained before
// that Arc is dropped. Atomic fields may reflect different sample instants.

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include "sysfs.h"

/* Mirror of repr(C) Rust atomics; READ_ONCE accesses the current values. */
struct asahi_stats_snapshot {
	u32 util1;
	u32 util2;
	u32 util3;
	u32 util4;
	u32 pstate;
	u32 avg_power_mw;
	u32 temperature_raw;
	u32 temperature_scale;
	u32 temperature_tmin;
	u32 temperature_tmax;
	u64 busy_ns;
	u64 jobs;
};

/* Mirror of the repr(C) Rust `memacct::Ledger`, in bytes. */
struct asahi_gpu_memory {
	u64 user_committed;
	u64 user_resident;
	u64 kernel_resident;
	u64 refused;
	u64 limit;
	u64 client_limit;
};

struct asahi_stats_attribute {
	struct device_attribute attr;
	struct device_attribute memory_attr;
	const struct asahi_stats_snapshot *snapshot;
	const struct asahi_gpu_memory *memory;
	bool export_enabled;
};

static ssize_t agx_stats_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct asahi_stats_attribute *stats =
		container_of(attr, struct asahi_stats_attribute, attr);
	const struct asahi_stats_snapshot *snap = stats->snapshot;
	ssize_t n = 0;

	if (!stats->export_enabled)
		return sysfs_emit(buf, "unsupported\n");
	n += scnprintf(buf + n, PAGE_SIZE - n, "busy_ns %llu\n",
		       (unsigned long long)READ_ONCE(snap->busy_ns));
	n += scnprintf(buf + n, PAGE_SIZE - n, "jobs %llu\n",
		       (unsigned long long)READ_ONCE(snap->jobs));
	n += scnprintf(buf + n, PAGE_SIZE - n, "pstate %u\n",
		       (u32)READ_ONCE(snap->pstate));
	n += scnprintf(buf + n, PAGE_SIZE - n, "power_mw %u\n",
		       (u32)READ_ONCE(snap->avg_power_mw));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util1 %u\n",
		       (u32)READ_ONCE(snap->util1));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util2 %u\n",
		       (u32)READ_ONCE(snap->util2));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util3 %u\n",
		       (u32)READ_ONCE(snap->util3));
	n += scnprintf(buf + n, PAGE_SIZE - n, "util4 %u\n",
		       (u32)READ_ONCE(snap->util4));
	n += scnprintf(buf + n, PAGE_SIZE - n, "temperature_raw %u\n",
		       (u32)READ_ONCE(snap->temperature_raw));
	n += scnprintf(buf + n, PAGE_SIZE - n, "temperature_scale %u\n",
		       (u32)READ_ONCE(snap->temperature_scale));

	return n;
}

/*
 * GPU memory held by the driver, in bytes: user objects created (committed)
 * and held by the GPU (resident), held kernel objects, the limits that admit
 * user objects (0: none) and the number of refused creations.
 */
static ssize_t gpu_memory_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct asahi_stats_attribute *stats =
		container_of(attr, struct asahi_stats_attribute, memory_attr);
	const struct asahi_gpu_memory *mem = stats->memory;
	ssize_t n = 0;

	n += scnprintf(buf + n, PAGE_SIZE - n, "user_committed %llu\n",
		       (unsigned long long)READ_ONCE(mem->user_committed));
	n += scnprintf(buf + n, PAGE_SIZE - n, "user_resident %llu\n",
		       (unsigned long long)READ_ONCE(mem->user_resident));
	n += scnprintf(buf + n, PAGE_SIZE - n, "kernel_resident %llu\n",
		       (unsigned long long)READ_ONCE(mem->kernel_resident));
	n += scnprintf(buf + n, PAGE_SIZE - n, "limit %llu\n",
		       (unsigned long long)READ_ONCE(mem->limit));
	n += scnprintf(buf + n, PAGE_SIZE - n, "client_limit %llu\n",
		       (unsigned long long)READ_ONCE(mem->client_limit));
	n += scnprintf(buf + n, PAGE_SIZE - n, "refused %llu\n",
		       (unsigned long long)READ_ONCE(mem->refused));

	return n;
}

/*
 * Called from Rust's `AsahiDriver::probe` after the DRM device is
 * registered. Returns 0 on success, or a negative errno.
 */
int asahi_sysfs_register(struct device *dev, const void *snapshot,
			 int export_enabled, const void *memory, void **handle)
{
	struct asahi_stats_attribute *stats;
	int ret;

	if (!dev || !handle || !memory || (export_enabled && !snapshot))
		return -ENODEV;

	BUILD_BUG_ON(offsetof(struct asahi_gpu_memory, refused) != 24);
	BUILD_BUG_ON(offsetof(struct asahi_gpu_memory, client_limit) != 40);
	BUILD_BUG_ON(sizeof(struct asahi_gpu_memory) != 48);

	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, pstate) != 16);
	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, busy_ns) != 40);
	BUILD_BUG_ON(offsetof(struct asahi_stats_snapshot, jobs) != 48);
	BUILD_BUG_ON(sizeof(struct asahi_stats_snapshot) != 56);

	stats = kzalloc_obj(*stats);
	if (!stats)
		return -ENOMEM;

	sysfs_attr_init(&stats->attr.attr);
	stats->attr.attr.name = "agx_stats";
	stats->attr.attr.mode = 0444;
	stats->attr.show = agx_stats_show;
	stats->snapshot = snapshot;
	stats->export_enabled = export_enabled;

	sysfs_attr_init(&stats->memory_attr.attr);
	stats->memory_attr.attr.name = "gpu_memory";
	stats->memory_attr.attr.mode = 0444;
	stats->memory_attr.show = gpu_memory_show;
	stats->memory = memory;

	ret = device_create_file(dev, &stats->attr);
	if (ret) {
		kfree(stats);
		return ret;
	}

	ret = device_create_file(dev, &stats->memory_attr);
	if (ret) {
		device_remove_file(dev, &stats->attr);
		kfree(stats);
		return ret;
	}

	*handle = stats;
	return 0;
}
EXPORT_SYMBOL_GPL(asahi_sysfs_register);

/*
 * G17: GPU idle power-off delay (ms, 2-1000). On T6040/25G83 the firmware powers the GPU cores
 * off after 2 ms idle and reads its own delay only at boot; above 2 the driver holds the cores
 * powered until the GPU has been idle this long (power::IDLE_HOLD_MS). Each wake-up there costs
 * about 1 ms, so userspace may lengthen it on AC power.
 */
static u32 *asahi_idle_word;

static ssize_t idle_off_delay_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(*asahi_idle_word));
}

static ssize_t idle_off_delay_ms_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	u32 ms;

	if (kstrtou32(buf, 0, &ms) || ms < 2 || ms > 1000)
		return -EINVAL;
	WRITE_ONCE(*asahi_idle_word, ms);
	return count;
}

static DEVICE_ATTR_RW(idle_off_delay_ms);

int asahi_sysfs_register_idle(struct device *dev, u32 *word)
{
	int ret;

	if (!dev || !word)
		return -ENODEV;
	asahi_idle_word = word;
	ret = device_create_file(dev, &dev_attr_idle_off_delay_ms);
	if (ret)
		asahi_idle_word = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(asahi_sysfs_register_idle);

void asahi_sysfs_unregister(struct device *dev, void *handle)
{
	struct asahi_stats_attribute *stats = handle;

	if (!dev || !stats)
		return;

	/* Removal drains active show callbacks before either allocation dies. */
	device_remove_file(dev, &stats->memory_attr);
	device_remove_file(dev, &stats->attr);
	if (asahi_idle_word) {
		device_remove_file(dev, &dev_attr_idle_off_delay_ms);
		asahi_idle_word = NULL;
	}
	kfree(stats);
}
EXPORT_SYMBOL_GPL(asahi_sysfs_unregister);

MODULE_AUTHOR("AGX driver maintainer");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Asahi AGX firmware stats sysfs shim");
