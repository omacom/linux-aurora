/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef _ASAHI_SYSFS_H
#define _ASAHI_SYSFS_H

#include <linux/types.h>

struct device;

/*
 * Mirrors g17::telemetry::Readout, filled under the Rust device mutex.
 * Status words: 0 unavailable, 1 valid, 2 stale, 3 unsupported by firmware.
 */
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
	u64 busy_ns;
	u64 achieved_frequency_hz;
	u32 busy_status;
	u32 util_source;
};

typedef void (*asahi_stats_read_fn)(const void *snapshot, struct asahi_qualified_stats *out);
typedef void (*asahi_stats_suspend_fn)(const void *snapshot, bool suspended);
typedef bool (*asahi_stats_frequency_fn)(const void *snapshot, u32 pstate, u64 *frequency_hz);

int asahi_sysfs_register(struct device *dev, const void *snapshot,
			 int export_enabled, const void *memory, void **handle,
			 asahi_stats_read_fn read, asahi_stats_suspend_fn suspend,
			 asahi_stats_frequency_fn frequency);
int asahi_sysfs_register_idle(struct device *dev, u32 *word);
void asahi_sysfs_unregister(struct device *dev, void *handle);

void asahi_set_mem_limit_pct(u32 pct);

#endif /* _ASAHI_SYSFS_H */
