// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/soc/apple/clpc.h>
#include "clpc.h"

int asahi_clpc_gpu_get(struct device *dev, struct apple_clpc_gpu **out)
{
	return apple_clpc_gpu_get(dev, out);
}
void asahi_clpc_gpu_set_active(struct apple_clpc_gpu *gpu, bool active)
{
	apple_clpc_gpu_set_active(gpu, active);
}
void asahi_clpc_gpu_put(struct apple_clpc_gpu *gpu)
{
	apple_clpc_gpu_put(gpu);
}
