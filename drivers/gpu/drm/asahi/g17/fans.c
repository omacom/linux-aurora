// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The M5 thermal governor's sensor and fan calls (thermal.rs). The thermal
 * core and the SMC hwmon cell have stubs where they are not built; Rust
 * reaches them through these.
 */
#include <linux/err.h>
#include <linux/mfd/macsmc.h>
#include <linux/thermal.h>
#include "fans.h"

int asahi_zone_temp(const char *name, int *temp)
{
	struct thermal_zone_device *zone = thermal_zone_get_zone_by_name(name);

	if (IS_ERR_OR_NULL(zone))
		return zone ? PTR_ERR(zone) : -ENODEV;
	return thermal_zone_get_temp(zone, temp);
}

int asahi_fans_request(struct device *dev, unsigned int permille, unsigned int min_step_rpm)
{
	return macsmc_hwmon_fans_request(dev, permille, min_step_rpm);
}
