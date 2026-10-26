/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_PMP_H
#define _LINUX_SOC_APPLE_PMP_H
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>
struct device;
#if IS_REACHABLE(CONFIG_APPLE_PMP)
int apple_pmp_link_device(struct device *consumer);
int apple_pmp_set_device_power(u8 command, u16 device_id, u32 enabled);
/* After the acknowledged AGX vote and successful GPU firmware startup. */
int apple_pmp_t6030_gpu_ready(void);
#else
static inline int apple_pmp_link_device(struct device *consumer)
{
	return -ENODEV;
}
static inline int apple_pmp_set_device_power(u8 command, u16 device_id, u32 enabled)
{
	return -ENODEV;
}
static inline int apple_pmp_t6030_gpu_ready(void)
{
	return -ENODEV;
}
#endif
/* Registration is private to the PMP Rust driver and its C bridge. */
int apple_pmp_register(struct device *dev, const void *data);
void apple_pmp_unregister(const void *data);
int apple_pmp_send_power_command(const void *data, u8 command,
				u16 device_id, u32 enabled);
#endif
