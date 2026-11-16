/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef ASAHI_G17_SYNC_INPUT_H
#define ASAHI_G17_SYNC_INPUT_H

#include <linux/types.h>

struct dma_fence;

struct asahi_g17_fence_leaf {
	struct dma_fence *fence;
	int status;
};

int asahi_g17_collect_fences(struct dma_fence *head,
			   struct asahi_g17_fence_leaf *out, size_t capacity);
void asahi_g17_put_fence(struct dma_fence *fence);

#endif
