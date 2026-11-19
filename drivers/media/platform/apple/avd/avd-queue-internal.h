/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef AVD_QUEUE_INTERNAL_H_
#define AVD_QUEUE_INTERNAL_H_

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/limits.h>

#include "avd-queue.h"

/* Caller serializes the pool; tags are never recycled during device lifetime. */
static inline int avd_slot_reserve(struct avd_frame *f, struct avd_ctx *ctx,
				   u64 seq)
{
	if (f->state != AVD_FRAME_FREE)
		return -EBUSY;
	if (!seq || seq > (U32_MAX >> 4))
		return -EOVERFLOW;
	f->seq = seq;
	f->tag = (seq << 4) | f->idx;
	f->ctx = ctx;
	f->state = AVD_FRAME_RESERVED;
	f->dep = AVD_DEP_NONE;
	f->status = 0;
	f->vp_done = false;
	f->vp_retired = false;
	f->prog_words = 0;
	f->codec_priv = NULL;
	return 0;
}

/* A full inbox owns nothing: keep the prepared slot, tag and deadline. */
static inline void avd_slot_submit_result(struct avd_frame *f, int status)
{
	if (status == -EBUSY) {
		f->state = AVD_FRAME_RESERVED;
	} else if (status) {
		f->status = status;
		f->state = AVD_FRAME_DONE;
	}
}

static inline bool avd_slot_complete(struct avd_frame *f, u32 tag, int status)
{
	if (f->state != AVD_FRAME_SUBMITTED || f->tag != tag)
		return false;
	f->status = status;
	f->state = AVD_FRAME_DONE;
	return true;
}

static inline bool avd_slot_first(const struct avd_frame *pool,
				  const struct avd_frame *f)
{
	unsigned int i;

	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (pool[i].state != AVD_FRAME_FREE && pool[i].ctx == f->ctx &&
		    pool[i].seq < f->seq)
			return false;
	return true;
}

static inline bool avd_slot_expired(const struct avd_frame *f, unsigned long now)
{
	return (f->state == AVD_FRAME_SUBMITTED ||
		f->state == AVD_FRAME_RESERVED) && time_after_eq(now, f->deadline);
}

/* Posted frames may overlap, but never pass an earlier reserved frame. */
static inline bool avd_slot_submit_first(const struct avd_frame *pool,
					const struct avd_frame *f)
{
	unsigned int i;

	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (pool[i].state == AVD_FRAME_RESERVED && pool[i].ctx == f->ctx &&
		    pool[i].seq < f->seq)
			return false;
	return true;
}

static inline bool avd_inbox_full(unsigned int producer, unsigned int consumer)
{
	return ((producer + 1) & 7) == consumer;
}

#endif /* AVD_QUEUE_INTERNAL_H_ */
