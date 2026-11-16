/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef ASAHI_G17_SYNC_SCOPE_H
#define ASAHI_G17_SYNC_SCOPE_H
#include <linux/types.h>
struct dma_fence;
void asahi_g17_signal_fence(struct dma_fence *fence);
/* Only for explicitly signalled driver completion fences, with no poll op. */
int asahi_g17_completion_fence_status(struct dma_fence *fence);

struct dma_fence *asahi_g17_scope_create(struct dma_fence *previous,
				      struct dma_fence *submitted, u64 point);
void asahi_g17_scope_bind(struct dma_fence *scope, struct dma_fence *root);
bool asahi_g17_is_scope(struct dma_fence *fence);
/* Returns owned child references, or zero once the scope is signalled. */
unsigned int asahi_g17_scope_children(struct dma_fence *fence,
				     struct dma_fence *children[2]);
#endif
