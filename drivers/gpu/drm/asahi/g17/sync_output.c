// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/slab.h>
#include <drm/drm_syncobj.h>
#include "sync_output.h"
#include "sync_scope.h"

struct asahi_g17_signal_target {
	struct drm_syncobj *syncobj;
	struct dma_fence_chain *chain;
	u64 point;
};

struct asahi_g17_signal_bundle {
	size_t count;
	size_t anchor;
	bool published;
	struct asahi_g17_signal_target targets[];
};

void asahi_g17_signals_free(struct asahi_g17_signal_bundle *bundle)
{
	size_t i;

	if (!bundle)
		return;
	for (i = 0; i < bundle->count; ++i) {
		struct asahi_g17_signal_target *target = &bundle->targets[i];
		if (target->chain)
			dma_fence_chain_free(target->chain);
		if (target->syncobj)
			drm_syncobj_put(target->syncobj);
	}
	kfree(bundle);
}

/* The Rust caller supplies its validated, immutable snapshot, never a user pointer. */
struct asahi_g17_signal_bundle *
asahi_g17_signals_new(const struct asahi_g17_signal_spec *syncs,
		     size_t count, size_t anchor, int *error)
{
	struct asahi_g17_signal_bundle *bundle;
	size_t i;

	*error = -EINVAL;
	if (!count || anchor >= count || !syncs[anchor].point)
		return NULL;
	*error = -ENOMEM;
	bundle = kzalloc(struct_size(bundle, targets, count), GFP_KERNEL);
	if (!bundle)
		return NULL;
	bundle->count = count;
	bundle->anchor = anchor;
	for (i = 0; i < count; ++i) {
		struct asahi_g17_signal_target *target = &bundle->targets[i];
		target->syncobj = syncs[i].syncobj;
		if (target->syncobj)
			drm_syncobj_get(target->syncobj);
		if (!target->syncobj) {
			*error = -ENOENT;
			goto fail;
		}
		target->point = syncs[i].point;
		if (syncs[i].timeline) {
			target->chain = dma_fence_chain_alloc();
			if (!target->chain)
				goto fail;
		}
		if (i != anchor && target->syncobj == bundle->targets[anchor].syncobj) {
			*error = -EINVAL;
			goto fail;
		}
	}
	/* The anchor may follow the other outputs. Reject aliases in either order. */
	for (i = 0; i < count; ++i) {
		if (i != anchor && bundle->targets[i].syncobj == bundle->targets[anchor].syncobj) {
			*error = -EINVAL;
			goto fail;
		}
	}
	*error = 0;
	return bundle;
fail:
	asahi_g17_signals_free(bundle);
	return NULL;
}

static void asahi_g17_signal_install(struct asahi_g17_signal_target *target,
				     struct dma_fence *fence)
{
	if (target->chain) {
		drm_syncobj_add_point(target->syncobj, target->chain, fence, target->point);
		target->chain = NULL;
	} else {
		drm_syncobj_replace_fence(target->syncobj, fence);
	}
}

int asahi_g17_signals_publish(struct asahi_g17_signal_bundle *bundle,
			      struct dma_fence *submitted)
{
	struct dma_fence *previous, *scope;
	size_t i;

	if (bundle->published)
		return -EINVAL;
	previous = drm_syncobj_fence_get(bundle->targets[bundle->anchor].syncobj);
	scope = asahi_g17_scope_create(previous, submitted,
				      bundle->targets[bundle->anchor].point);
	dma_fence_put(previous);
	if (!scope)
		return -ENOMEM;
	/* Only exported completion includes earlier work. GPU execution gets no new wait. */
	asahi_g17_signal_install(&bundle->targets[bundle->anchor], scope);
	previous = drm_syncobj_fence_get(bundle->targets[bundle->anchor].syncobj);
	asahi_g17_scope_bind(scope, previous);
	dma_fence_put(previous);
	for (i = 0; i < bundle->count; ++i)
		if (i != bundle->anchor)
			asahi_g17_signal_install(&bundle->targets[i], scope);
	bundle->published = true;
	dma_fence_put(scope);
	return 0;
}

/* Allocation or validation failed before enqueue: install an error promise without allocating. */
void asahi_g17_signals_fail(struct asahi_g17_signal_bundle *bundle,
			    struct dma_fence *failure)
{
	size_t i;

	if (bundle->published)
		return;
	for (i = 0; i < bundle->count; ++i)
		asahi_g17_signal_install(&bundle->targets[i], failure);
	bundle->published = true;
}
