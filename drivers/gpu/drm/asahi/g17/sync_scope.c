// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <linux/dma-fence-chain.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/irq_work.h>
#include <linux/irqflags.h>
#include <linux/llist.h>
#include <linux/percpu.h>
#include <linux/preempt.h>
#include <linux/slab.h>
#include "sync_scope.h"
#include "sync_input.h"

int asahi_g17_completion_fence_status(struct dma_fence *fence)
{
	/* These private fences have no signaled callback. A pending flag is
	 * sufficient to keep the frontier member; a concurrent signal can only
	 * defer pruning until the next submission. Read terminal errors through
	 * the normal locked API after observing the signalled flag.
	 */
	if (!dma_fence_test_signaled_flag(fence))
		return 0;
	return dma_fence_get_status(fence);
}

struct asahi_g17_scope;
struct asahi_g17_scope_callback {
	struct dma_fence_cb callback;
	struct asahi_g17_scope *scope;
};

struct asahi_g17_scope {
	struct dma_fence base;
	atomic_t pending;
	struct dma_fence *children[2];
	struct asahi_g17_scope_callback callbacks[2];
	struct irq_work work;
	struct llist_node ready;
	/* Identifies the exact anchor point this scope replaced. */
	u64 anchor_context;
	u64 anchor_point;
};

static const struct dma_fence_ops scope_ops;
static DEFINE_PER_CPU(struct llist_head *, active_scopes);
#define SCOPE_ATOMIC_BATCH 64

bool asahi_g17_is_scope(struct dma_fence *fence)
{
	return fence && fence->ops == &scope_ops;
}

static struct asahi_g17_scope *to_scope(struct dma_fence *fence)
{
	return container_of(fence, struct asahi_g17_scope, base);
}

static const char *scope_driver_name(struct dma_fence *fence)
{
	return "asahi";
}

static const char *scope_timeline_name(struct dma_fence *fence)
{
	return "g17-queue-scope";
}

static bool scope_signaled(struct dma_fence *fence)
{
	struct asahi_g17_scope *scope = to_scope(fence);
	if (atomic_read_acquire(&scope->pending))
		return false;
	cmpxchg(&fence->error, 1, 0);
	return true;
}

static bool scope_enable_signaling(struct dma_fence *fence)
{
	/* Child callbacks are already armed; no recursive callback installation. */
	return !scope_signaled(fence);
}

unsigned int asahi_g17_scope_children(struct dma_fence *fence,
				     struct dma_fence *children[2])
{
	struct asahi_g17_scope *scope = to_scope(fence);
	unsigned long flags;
	unsigned int i, count = 0;

	spin_lock_irqsave(&fence->inline_lock, flags);
	if (!test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags))
		for (i = 0; i < ARRAY_SIZE(scope->children); ++i)
			if (scope->children[i])
				children[count++] = dma_fence_get(scope->children[i]);
	spin_unlock_irqrestore(&fence->inline_lock, flags);
	return count;
}

static void scope_release_completed(struct asahi_g17_scope *scope)
{
	struct dma_fence *children[2];
	unsigned long flags;
	unsigned int i;

	/* Readers take references under this lock. Signalled scopes need no history. */
	spin_lock_irqsave(&scope->base.inline_lock, flags);
	for (i = 0; i < ARRAY_SIZE(children); ++i) {
		children[i] = scope->children[i];
		scope->children[i] = NULL;
	}
	spin_unlock_irqrestore(&scope->base.inline_lock, flags);
	for (i = 0; i < ARRAY_SIZE(children); ++i)
		dma_fence_put(children[i]);
	dma_fence_put(&scope->base);
}

static void scope_complete(struct asahi_g17_scope *scope)
{
	cmpxchg(&scope->base.error, 1, 0);
	dma_fence_signal(&scope->base);
	scope_release_completed(scope);
}

static void scope_work(struct irq_work *work)
{
	struct asahi_g17_scope *scope = container_of(work, struct asahi_g17_scope, work);
	cmpxchg(&scope->base.error, 1, 0);
	asahi_g17_signal_fence(&scope->base);
	scope_release_completed(scope);
}

static void scope_defer_list(struct llist_node *nodes)
{
	struct llist_node *node, *next;
	llist_for_each_safe(node, next, nodes) {
		struct asahi_g17_scope *scope =
			container_of(node, struct asahi_g17_scope, ready);
		/* Transfer the ready-list reference to the irq_work callback. */
		irq_work_queue(&scope->work);
	}
}

void asahi_g17_signal_fence(struct dma_fence *fence)
{
	struct llist_head ready = LLIST_HEAD_INIT(ready);
	struct llist_node *nodes, *node, *next;
	unsigned long flags;
	unsigned int completed = 0;

	/* Fence callbacks are atomic. Keep the local ready list pinned while
	 * signaling, then finish ready scopes after the source fence is unlocked.
	 * New ready scopes append to this list instead of recursing or sending IPIs.
	 */
	preempt_disable();
	if (__this_cpu_read(active_scopes)) {
		dma_fence_signal(fence);
		preempt_enable();
		return;
	}
	__this_cpu_write(active_scopes, &ready);
	dma_fence_signal(fence);
	for (;;) {
		/* An interrupt must not append after the final empty test. */
		local_irq_save(flags);
		nodes = llist_del_all(&ready);
		if (!nodes)
			__this_cpu_write(active_scopes, NULL);
		local_irq_restore(flags);
		if (!nodes)
			break;
		llist_for_each_safe(node, next, nodes) {
			if (completed == SCOPE_ATOMIC_BATCH) {
				struct llist_node *more;
				local_irq_save(flags);
				more = llist_del_all(&ready);
				__this_cpu_write(active_scopes, NULL);
				local_irq_restore(flags);
				preempt_enable();
				scope_defer_list(node);
				scope_defer_list(more);
				return;
			}
			scope_complete(container_of(node, struct asahi_g17_scope, ready));
			++completed;
		}
	}
	preempt_enable();
}

static void scope_arrived(struct asahi_g17_scope *scope, int error)
{
	if (error < 0)
		cmpxchg(&scope->base.error, 1, error);
	if (atomic_dec_and_test(&scope->pending)) {
		struct llist_head *ready;
		dma_fence_get(&scope->base);
		preempt_disable();
		ready = __this_cpu_read(active_scopes);
		if (ready)
			llist_add(&scope->ready, ready);
		preempt_enable();
		/* Foreign fence signalers have no driver completion pass to drain us. */
		if (!ready)
			irq_work_queue(&scope->work);
	}
}

static void scope_callback(struct dma_fence *child, struct dma_fence_cb *callback)
{
	struct asahi_g17_scope_callback *cb =
		container_of(callback, typeof(*cb), callback);
	scope_arrived(cb->scope, child->error);
	dma_fence_put(&cb->scope->base);
}

static void scope_release(struct dma_fence *fence)
{
	struct asahi_g17_scope *scope = to_scope(fence);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(scope->children); ++i)
		dma_fence_put(scope->children[i]);
	dma_fence_free(fence);
}

static void scope_set_deadline(struct dma_fence *fence, ktime_t deadline)
{
	struct asahi_g17_fence_leaf *leaves;
	int count, i;

	if (dma_fence_is_signaled(fence))
		return;
	/* A deadline is an advisory hint. Keep its traversal bounded and iterative,
	 * including when a foreign timeline boundary contains earlier scopes.
	 */
	leaves = kmalloc_array(128, sizeof(*leaves), GFP_ATOMIC | __GFP_NOWARN);
	if (!leaves)
		return;
	count = asahi_g17_collect_fences(fence, leaves, 128);
	for (i = 0; i < count; ++i) {
		if (!asahi_g17_is_scope(leaves[i].fence))
			dma_fence_set_deadline(leaves[i].fence, deadline);
		dma_fence_put(leaves[i].fence);
	}
	kfree(leaves);
}

static const struct dma_fence_ops scope_ops = {
	.get_driver_name = scope_driver_name,
	.get_timeline_name = scope_timeline_name,
	.enable_signaling = scope_enable_signaling,
	.signaled = scope_signaled,
	.release = scope_release,
	.set_deadline = scope_set_deadline,
};

void asahi_g17_scope_bind(struct dma_fence *fence, struct dma_fence *root)
{
	struct asahi_g17_scope *scope = to_scope(fence);
	/* A concurrent raw syncobj reset is invalid for the private queue anchor,
	 * but must not turn a hostile userspace race into a kernel dereference.
	 */
	if (!root || dma_fence_chain_contained(root) != fence)
		return;
	/* Written once before the submit returns; an unmatched snapshot takes fallback. */
	WRITE_ONCE(scope->anchor_point, root->seqno);
	WRITE_ONCE(scope->anchor_context, root->context);
}

struct dma_fence *asahi_g17_scope_create(struct dma_fence *previous,
				      struct dma_fence *submitted, u64 point)
{
	struct dma_fence *contained = previous ? dma_fence_chain_contained(previous) : NULL;
	struct dma_fence *parent = NULL;
	struct asahi_g17_scope *scope, *prior = NULL;
	u64 context;
	unsigned int i;

	if (asahi_g17_is_scope(contained)) {
		prior = to_scope(contained);
		if (READ_ONCE(prior->anchor_context) != previous->context ||
		    READ_ONCE(prior->anchor_point) != previous->seqno ||
		    point <= contained->seqno)
			prior = NULL;
	}
	if (prior) {
		parent = dma_fence_get(contained);
		context = contained->context;
	} else {
		/* Empty submissions or external edits may insert a foreign timeline point.
		 * Flatten that boundary once; the ordinary path retains just the prior scope.
		 */
		if (previous) {
			parent = dma_fence_unwrap_merge(previous);
			if (!parent)
				return NULL;
		}
		context = dma_fence_context_alloc(1);
	}
	scope = kzalloc_obj(*scope);
	if (!scope) {
		dma_fence_put(parent);
		return NULL;
	}
	dma_fence_init64(&scope->base, &scope_ops, NULL, context, point);
	init_irq_work(&scope->work, scope_work);
	scope->children[0] = parent;
	scope->children[1] = dma_fence_get(submitted);
	/* The constructor guard prevents completion while callbacks are being installed. */
	atomic_set(&scope->pending, 3);
	scope->base.error = 1;
	for (i = 0; i < ARRAY_SIZE(scope->children); ++i) {
		struct dma_fence *child = scope->children[i];
		if (!child) {
			scope_arrived(scope, 0);
			continue;
		}
		scope->callbacks[i].scope = scope;
		dma_fence_get(&scope->base);
		if (dma_fence_add_callback(child, &scope->callbacks[i].callback, scope_callback)) {
			scope_arrived(scope, child->error);
			dma_fence_put(&scope->base);
		}
	}
	scope_arrived(scope, 0);
	return &scope->base;
}
