// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>

int asahi_g18_wait_foreign_fences(struct dma_fence *head,
				  const struct dma_fence_ops *own);
int asahi_g18_fence_error(struct dma_fence *head);

/*
 * Wait, interruptibly, for every fence of @head, inside chains and arrays
 * too, whose ops are not @own. A signalled fence has no ops left and its
 * wait returns at once. Returns 0 or -ERESTARTSYS.
 */
int asahi_g18_wait_foreign_fences(struct dma_fence *head,
				  const struct dma_fence_ops *own)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *fence;

	dma_fence_unwrap_for_each(fence, &cursor, head) {
		long error;

		if (rcu_access_pointer(fence->ops) == own)
			continue;
		error = dma_fence_wait(fence, true);
		if (error) {
			/* The walk holds a reference on its container. */
			dma_fence_put(cursor.chain);
			return error;
		}
	}
	return 0;
}

/*
 * The first error of a signalled fence of @head, inside chains and arrays
 * too, or 0. A container does not carry its fences' errors: a
 * dma_fence_chain (a timeline point) never copies them, and a
 * dma_fence_array whose signalling was never enabled clears its pending
 * error when it is found signalled. So each fence is checked itself.
 */
int asahi_g18_fence_error(struct dma_fence *head)
{
	struct dma_fence_unwrap cursor;
	struct dma_fence *fence;

	dma_fence_unwrap_for_each(fence, &cursor, head) {
		int status = dma_fence_get_status(fence);

		if (status < 0) {
			/* The walk holds a reference on its container. */
			dma_fence_put(cursor.chain);
			return status;
		}
	}
	return 0;
}
