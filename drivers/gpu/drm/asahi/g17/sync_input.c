// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/errno.h>
#include "sync_input.h"
#include "sync_scope.h"

static int collect_leaf(struct dma_fence *leaf,
			struct asahi_g17_fence_leaf *out, size_t capacity,
			size_t *count)
{
	if (*count == capacity)
		return -ENOSPC;
	out[*count].fence = dma_fence_get(leaf);
	out[*count].status = dma_fence_get_status(leaf);
	++*count;
	return 0;
}

/* Called without the device mutex: status polling may run fence callbacks. */
int asahi_g17_collect_fences(struct dma_fence *head,
			   struct asahi_g17_fence_leaf *out, size_t capacity)
{
	struct dma_fence *pending[128];
	size_t count = 0, depth = 1;
	int error = 0;

	pending[0] = dma_fence_get(head);
	while (depth && !error) {
		struct dma_fence *node = pending[--depth];
		struct dma_fence_unwrap cursor;
		struct dma_fence *leaf;

		if (asahi_g17_is_scope(node)) {
			struct dma_fence *children[2];
			unsigned int n = asahi_g17_scope_children(node, children), i;
			if (!n) {
				/* Preserve a failed scope as a failed leaf, never drop its error. */
				if (dma_fence_get_status(node) < 0)
					error = collect_leaf(node, out, capacity, &count);
			} else if (depth + n > ARRAY_SIZE(pending)) {
				for (i = 0; i < n; ++i)
					dma_fence_put(children[i]);
				error = -ENOSPC;
			} else {
				for (i = 0; i < n; ++i)
					pending[depth++] = children[i];
			}
			dma_fence_put(node);
			continue;
		}

		dma_fence_unwrap_for_each(leaf, &cursor, node) {
			if (asahi_g17_is_scope(leaf)) {
				if (depth == ARRAY_SIZE(pending))
					error = -ENOSPC;
				else
					pending[depth++] = dma_fence_get(leaf);
			} else if (dma_fence_is_container(leaf)) {
				error = -EOPNOTSUPP;
			} else {
				error = collect_leaf(leaf, out, capacity, &count);
			}
			if (error)
				break;
		}
		/* A bounded early exit still owns the iterator's current chain. */
		dma_fence_put(cursor.chain);
		dma_fence_put(node);
	}
	while (depth)
		dma_fence_put(pending[--depth]);
	if (error) {
		while (count)
			dma_fence_put(out[--count].fence);
		return error;
	}
	return count;
}

void asahi_g17_put_fence(struct dma_fence *fence)
{
	dma_fence_put(fence);
}
