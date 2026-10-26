/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple SoC PMP report region
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef _LINUX_SOC_APPLE_PMP_REPORT_H_
#define _LINUX_SOC_APPLE_PMP_REPORT_H_

#include <linux/errno.h>
#include <linux/types.h>

struct device_node;
struct device;

#if IS_ENABLED(CONFIG_APPLE_PMP_REPORT)
int apple_pmp_report_wait_supplier_ready(struct device *supplier, unsigned int timeout_ms);
/* Caller holds the PMP owner's vote lock and keeps the supplier bound. */
int apple_pmp_report_check_t6030_gfx(struct device *supplier);
int apple_pmp_report_set_t6030_gfx(struct device *supplier, bool enable);
int apple_pmp_report_wait_ready(struct device_node *entry, unsigned long timeout);
int apple_pmp_report_fast_die_effort(const struct device_node *np,
				     unsigned int lane, u8 *effort);
#else
static inline int apple_pmp_report_wait_supplier_ready(struct device *supplier,
						unsigned int timeout_ms)
{
	return -ENODEV;
}

static inline int apple_pmp_report_check_t6030_gfx(struct device *supplier)
{
	return -ENODEV;
}

static inline int apple_pmp_report_set_t6030_gfx(struct device *supplier, bool enable)
{
	return -ENODEV;
}

static inline int apple_pmp_report_wait_ready(struct device_node *entry,
					      unsigned long timeout)
{
	return -ENODEV;
}

static inline int apple_pmp_report_fast_die_effort(const struct device_node *np,
						   unsigned int lane, u8 *effort)
{
	return -ENODEV;
}
#endif

#endif /* _LINUX_SOC_APPLE_PMP_REPORT_H_ */
