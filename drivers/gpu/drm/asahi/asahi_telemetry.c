// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* A device attribute with an owned Rust snapshot callback. */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

struct asahi_load_export {
	u32 valid, power_known, powered, utilization[4], clock_hz;
	u64 samples, received_ns, timestamp_ticks, power_timestamp_ticks;
	u64 powered_ticks, unpowered_ticks, invalid, age_ns;
};

typedef int (*asahi_snapshot_fn)(const void *, void *);
struct asahi_telemetry_attribute {
	struct device_attribute attr;
	const void *data;
	asahi_snapshot_fn snapshot;
};

static ssize_t agx_stats_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct asahi_telemetry_attribute *stats =
		container_of(attr, struct asahi_telemetry_attribute, attr);
	struct asahi_load_export value;
	int ret = stats->snapshot(stats->data, &value);

	if (ret)
		return ret;
	return sysfs_emit(buf,
		"version 2\nsource firmware\nload_valid %u\n"
		"gpu_power_known %u\ngpu_powered %u\n"
		"util1 %u\nutil2 %u\nutil3 %u\nutil4 %u\nbusy_percent %u\n"
		"samples %llu\nsample_received_ns %llu\nsample_age_ns %llu\n"
		"sample_timestamp_ticks %llu\npower_timestamp_ticks %llu\n"
		"clock_hz %u\npowered_ticks %llu\nunpowered_ticks %llu\n"
		"invalid_samples %llu\n",
		value.valid, value.power_known, value.powered,
		value.utilization[0], value.utilization[1], value.utilization[2],
		value.utilization[3], value.utilization[3],
		(unsigned long long)value.samples,
		(unsigned long long)value.received_ns,
		(unsigned long long)value.age_ns,
		(unsigned long long)value.timestamp_ticks,
		(unsigned long long)value.power_timestamp_ticks,
		value.clock_hz,
		(unsigned long long)value.powered_ticks,
		(unsigned long long)value.unpowered_ticks,
		(unsigned long long)value.invalid);
}

int asahi_telemetry_register(struct device *dev, const void *data,
			     asahi_snapshot_fn snapshot, void **handle);
void asahi_telemetry_unregister(struct device *dev, void *handle);

int asahi_telemetry_register(struct device *dev, const void *data,
			     asahi_snapshot_fn snapshot, void **handle)
{
	struct asahi_telemetry_attribute *stats;
	int ret;

	if (!dev || !data || !snapshot || !handle)
		return -EINVAL;
	BUILD_BUG_ON(sizeof(struct asahi_load_export) != 96);
	BUILD_BUG_ON(offsetof(struct asahi_load_export, samples) != 32);
	stats = kzalloc(sizeof(*stats), GFP_KERNEL);
	if (!stats)
		return -ENOMEM;
	sysfs_attr_init(&stats->attr.attr);
	stats->attr.attr.name = "agx_stats";
	stats->attr.attr.mode = 0444;
	stats->attr.show = agx_stats_show;
	stats->data = data;
	stats->snapshot = snapshot;
	ret = device_create_file(dev, &stats->attr);
	if (ret) {
		kfree(stats);
		return ret;
	}
	*handle = stats;
	return 0;
}
EXPORT_SYMBOL_GPL(asahi_telemetry_register);

void asahi_telemetry_unregister(struct device *dev, void *handle)
{
	struct asahi_telemetry_attribute *stats = handle;

	if (!stats)
		return;
	device_remove_file(dev, &stats->attr);
	kfree(stats);
}
EXPORT_SYMBOL_GPL(asahi_telemetry_unregister);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Asahi firmware telemetry sysfs registration");
