/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _ASAHI_G17_FANS_H
#define _ASAHI_G17_FANS_H
struct device;
int asahi_zone_temp(const char *name, int *temp);
int asahi_fans_request(struct device *dev, unsigned int permille, unsigned int min_step_rpm);
#endif
