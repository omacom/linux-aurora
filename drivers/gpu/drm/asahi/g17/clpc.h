/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _ASAHI_G17_CLPC_H
#define _ASAHI_G17_CLPC_H
#include <linux/types.h>
struct apple_clpc_gpu;
/*
 * The CLPC's GPU activity input (include/linux/soc/apple/clpc.h). @dev is the
 * GPU's struct device, opaque to Rust, whose bindings cannot name it here.
 */
int asahi_clpc_gpu_get(void *dev, struct apple_clpc_gpu **out);
void asahi_clpc_gpu_set_active(struct apple_clpc_gpu *gpu, bool active);
void asahi_clpc_gpu_put(struct apple_clpc_gpu *gpu);
#endif
