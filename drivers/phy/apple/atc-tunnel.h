/* SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause */
#ifndef _APPLE_ATC_TUNNEL_H
#define _APPLE_ATC_TUNNEL_H

#include <linux/errno.h>
#include <linux/of.h>
#include <linux/types.h>

static inline bool apple_atc_t602x_qualified(const struct device_node *root)
{
	return of_device_is_compatible(root, "apple,t6020") ||
	       of_device_is_compatible(root, "apple,t6021") ||
	       of_device_is_compatible(root, "apple,t6022");
}

static inline bool apple_atc_t602x_core_valid(const struct device_node *np, u64 base, u64 size)
{
	return of_device_is_compatible(np, "apple,t6020-atcphy") &&
	       (base == 0x703000000ULL || base == 0xb03000000ULL || base == 0xf03000000ULL ||
		base == 0x1303000000ULL) &&
	       size >= 0x7048;
}

static inline int apple_atc_t602x_gate(bool own_wiring, bool routes_present, bool valid_core)
{
	if (!routes_present)
		return -EOPNOTSUPP;
	if (!valid_core)
		return -EINVAL;
	return own_wiring ? 0 : -EOPNOTSUPP;
}

#endif
