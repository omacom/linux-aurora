/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef _ASAHI_SYSFS_H
#define _ASAHI_SYSFS_H

struct device;

int asahi_sysfs_register(struct device *dev, const void *snapshot,
			 int export_enabled, const void *memory, void **handle);
int asahi_sysfs_register_idle(struct device *dev, u32 *word);
void asahi_sysfs_unregister(struct device *dev, void *handle);

#endif /* _ASAHI_SYSFS_H */
