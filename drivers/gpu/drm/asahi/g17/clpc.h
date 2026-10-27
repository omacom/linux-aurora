/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _ASAHI_G17_CLPC_H
#define _ASAHI_G17_CLPC_H
struct device;
struct apple_clpc_gpu;
int asahi_clpc_gpu_get(struct device *dev, struct apple_clpc_gpu **out);
void asahi_clpc_gpu_set_active(struct apple_clpc_gpu *gpu, bool active);
void asahi_clpc_gpu_put(struct apple_clpc_gpu *gpu);
#endif
