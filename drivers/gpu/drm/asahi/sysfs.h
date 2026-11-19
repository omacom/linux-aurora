/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef _ASAHI_SYSFS_H
#define _ASAHI_SYSFS_H

#include <linux/types.h>

struct device;

/* Mirrors g17::telemetry::Readout, filled under the Rust device mutex. */
struct asahi_qualified_stats {
	u32 util_status;
	u32 pstate_status;
	u32 tiler;
	u32 renderer;
	u32 device;
	u32 pstate;
	u64 util_host_ns;
	u64 pstate_host_ns;
	u64 util_samples;
	u64 pstate_timestamp;
};

typedef void (*asahi_stats_read_fn)(const void *snapshot, struct asahi_qualified_stats *out);
typedef void (*asahi_stats_suspend_fn)(const void *snapshot, bool suspended);

int asahi_sysfs_register(struct device *dev, const void *snapshot,
			 int export_enabled, const void *memory, void **handle,
			 asahi_stats_read_fn read, asahi_stats_suspend_fn suspend);
int asahi_sysfs_register_idle(struct device *dev, u32 *word);
void asahi_sysfs_unregister(struct device *dev, void *handle);

#endif /* _ASAHI_SYSFS_H */
