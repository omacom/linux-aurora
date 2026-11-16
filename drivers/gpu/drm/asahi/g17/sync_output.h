/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef ASAHI_G17_SYNC_OUTPUT_H
#define ASAHI_G17_SYNC_OUTPUT_H
#include <linux/types.h>

struct dma_fence;
struct drm_syncobj;
struct asahi_g17_signal_bundle;
struct asahi_g17_signal_spec {
	struct drm_syncobj *syncobj;
	u64 point;
	u32 timeline;
};

struct asahi_g17_signal_bundle *
asahi_g17_signals_new(const struct asahi_g17_signal_spec *specs,
		     size_t count, size_t anchor, int *error);
void asahi_g17_signals_free(struct asahi_g17_signal_bundle *bundle);
int asahi_g17_signals_publish(struct asahi_g17_signal_bundle *bundle,
			      struct dma_fence *submitted);
void asahi_g17_signals_fail(struct asahi_g17_signal_bundle *bundle,
			    struct dma_fence *failure);
#endif
