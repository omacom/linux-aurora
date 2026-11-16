/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_DISPLAY_H_
#define _LINUX_SOC_APPLE_DISPLAY_H_

struct device;

/* Initial native display admission, before starting retained G15 firmware. */
int apple_drm_t6030_link_device(struct device *consumer);
int apple_drm_t6031_link_device(struct device *consumer);
int apple_drm_t6034_link_device(struct device *consumer);

#endif
