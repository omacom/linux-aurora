/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef __APPLE_IOMFB_V14_7_LINK_H__
#define __APPLE_IOMFB_V14_7_LINK_H__

#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "dcp-link.h"

#define DCP_V14_LINK_MAX_EVENTS		32
#define DCP_V14_LINK_MAX_CALLBACKS	8

/* Method and callback tags: 'A' or 'D' followed by three decimal digits. */
#define DCP_V14_TAG(c, n) (((u32)(c) << 24) | ((u32)('0' + (n) / 100) << 16) | \
			   ((u32)('0' + (n) / 10 % 10) << 8) | (u32)('0' + (n) % 10))

struct apple_rtkit;
struct work_struct;

/*
 * Handles one callback from the firmware. @out is zeroed and is sent back
 * as the reply when the handler returns 0; any error stops the link.
 */
typedef int (*dcp_v14_callback_fn)(void *cookie, u32 tag, const void *in,
				   u32 in_size, void *out, u32 out_size);

/* Called under the link lock immediately before a packet is submitted. */
typedef bool (*dcp_v14_admit_fn)(void *cookie);

struct dcp_v14_link {
	struct device *dev;
	struct apple_rtkit *rtk;
	void *rpc;
	dma_addr_t rpc_dva;

	/* Protects everything below; never held across a callback handler. */
	struct mutex lock;
	wait_queue_head_t wait;
	struct list_head events;
	unsigned int event_count;
	unsigned int callback_count;
	/* Callback handlers running on the caller's stack. */
	unsigned int callback_depth;
	bool failed;
	bool enabled;
	struct work_struct *idle_work;

	struct completion ready;
	int ready_result;
	/* The swap the firmware dropped: its D589, if any, completes nothing. */
	u32 dropped_completion;

	/* Our calls: [0] outer, [1] made from inside a callback. */
	struct {
		bool active;
		bool remote;
		u32 offset;
		u32 size;
		struct apple_dcp_link_rpc_header header;
	} calls[2];

	/*
	 * Firmware callbacks awaiting a reply, in arrival order. Callbacks
	 * from separate firmware threads can be outstanding at once and are
	 * answered in the order they are handled, not necessarily the last
	 * one first.
	 */
	struct {
		u64 message;
		void *output;
		u32 size;
		u32 call;
	} callbacks[DCP_V14_LINK_MAX_CALLBACKS];
	/* The callback each running handler answers, innermost last. */
	u64 handling[DCP_V14_LINK_MAX_CALLBACKS];
};

void dcp_v14_link_init(struct dcp_v14_link *link, struct device *dev,
		       struct apple_rtkit *rtk);
int dcp_v14_link_start(struct dcp_v14_link *link);
void dcp_v14_link_receive(struct dcp_v14_link *link, u64 message);
void dcp_v14_link_fail(struct dcp_v14_link *link);
bool dcp_v14_link_failed(struct dcp_v14_link *link);
void dcp_v14_link_set_idle_work(struct dcp_v14_link *link, struct work_struct *work);
int dcp_v14_link_call(struct dcp_v14_link *link, u32 tag, const void *input,
		      u32 input_size, void *output, u32 output_size,
		      u32 completion_id, dcp_v14_callback_fn callback, void *cookie);
int dcp_v14_link_call_guarded(struct dcp_v14_link *link, u32 tag, const void *input,
		      u32 input_size, void *output, u32 output_size,
		      u32 completion_id, dcp_v14_callback_fn callback, void *cookie,
		      dcp_v14_admit_fn admit, void *admit_cookie, bool *vetoed);
int dcp_v14_link_pump(struct dcp_v14_link *link, unsigned long timeout,
		      dcp_v14_callback_fn callback, void *cookie);

#endif /* __APPLE_IOMFB_V14_7_LINK_H__ */
