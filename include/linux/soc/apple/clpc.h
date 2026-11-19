/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_CLPC_H
#define _LINUX_SOC_APPLE_CLPC_H

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/property.h>

struct apple_clpc_gpu;

#if IS_REACHABLE(CONFIG_APPLE_CLPC)
int apple_clpc_gpu_get(struct device *dev, struct apple_clpc_gpu **out);
void apple_clpc_gpu_set_active(struct apple_clpc_gpu *gpu, bool active);
void apple_clpc_gpu_put(struct apple_clpc_gpu *gpu);
#else
static inline int apple_clpc_gpu_get(struct device *dev, struct apple_clpc_gpu **out)
{
	*out = NULL;
	return device_property_present(dev, "apple,clpc") ? -ENODEV : 0;
}
static inline void apple_clpc_gpu_set_active(struct apple_clpc_gpu *gpu, bool active) {}
static inline void apple_clpc_gpu_put(struct apple_clpc_gpu *gpu) {}
#endif
#endif
