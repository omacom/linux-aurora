/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * When the T6001 AGX request (PS-REQ bit 9 of the PMP report) is set and cleared.
 *
 * The GPU driver counts operations in flight (begin/end). The request is set when the first operation starts and cleared
 * idle_ms after the last one ends, so a decode stream (a burst every ~10 ms) holds one vote and an idle GPU drops it. All state
 * changes are in these functions; the caller serializes them with one spinlock and does the register write between
 * agx_vote_decide() and agx_vote_applied(). Times are milliseconds from any wrapping clock.
 */
#ifndef PMP_AGX_VOTE_H
#define PMP_AGX_VOTE_H

#ifdef __KERNEL__
#include <linux/types.h>
/* jiffies_to_msecs() returns u32: the clock wraps every 49.7 days, and every compare below is done in 32 bits. */
typedef u32 agx_ms_t;
typedef s32 agx_sms_t;
#else
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t agx_ms_t;
typedef int32_t agx_sms_t;
#endif

/* After a set that was not acknowledged (or a PMP that is not running), wait this long before trying again. */
#define AGX_VOTE_RETRY_MS 1000

struct agx_vote {
	bool enabled;		/* the module parameter */
	bool voted;		/* bit 9 is set and was acknowledged */
	unsigned int users;	/* GPU operations in flight */
	agx_ms_t idle_ms;	/* keep the vote this long after the last operation ends */
	agx_ms_t idle_at;	/* when users last fell to 0 */
	unsigned int failures;	/* consecutive failed sets; the backoff applies only while this is nonzero */
	agx_ms_t retry_at;	/* with failures: no new set before this */
};

enum agx_vote_action {
	AGX_VOTE_NONE,
	AGX_VOTE_SET,
	AGX_VOTE_CLEAR,
};

static inline bool agx_vote_reached(agx_ms_t now, agx_ms_t t)
{
	return (agx_sms_t)(now - t) >= 0;
}

/* An operation starts. True: run the reconcile step now. */
static inline bool agx_vote_begin(struct agx_vote *v)
{
	v->users++;
	return v->enabled && !v->voted;
}

/* An operation ends. Returns the delay in ms after which the reconcile step must run, 0 when none is needed. */
static inline agx_ms_t agx_vote_end(struct agx_vote *v, agx_ms_t now)
{
	if (!v->users)		/* unbalanced end: ignored */
		return 0;
	if (--v->users)
		return 0;
	v->idle_at = now;
	if (!v->enabled || !v->voted)
		return 0;
	return v->idle_ms ? v->idle_ms : 1;
}

/* What the reconcile step must do now. */
static inline enum agx_vote_action agx_vote_decide(const struct agx_vote *v, agx_ms_t now)
{
	if (!v->enabled)
		return v->voted ? AGX_VOTE_CLEAR : AGX_VOTE_NONE;
	if (v->users)
		return !v->voted && (!v->failures || agx_vote_reached(now, v->retry_at)) ? AGX_VOTE_SET : AGX_VOTE_NONE;
	if (v->voted && agx_vote_reached(now, v->idle_at + v->idle_ms))
		return AGX_VOTE_CLEAR;
	return AGX_VOTE_NONE;
}

/* Delay in ms until the reconcile step must run again, 0 when no timer is needed. */
static inline agx_ms_t agx_vote_next(const struct agx_vote *v, agx_ms_t now)
{
	agx_ms_t at;

	if (!v->enabled)
		return 0;
	if (v->users && !v->voted && v->failures)
		at = v->retry_at;
	else if (!v->users && v->voted)
		at = v->idle_at + v->idle_ms;
	else
		return 0;
	return agx_vote_reached(now, at) ? 0 : at - now;
}

/* Record the outcome of the register write for `action` (ret 0: acknowledged). */
static inline void agx_vote_applied(struct agx_vote *v, enum agx_vote_action action, int ret, agx_ms_t now)
{
	if (action == AGX_VOTE_SET) {
		if (ret) {
			v->failures++;
			v->retry_at = now + AGX_VOTE_RETRY_MS;
		} else {
			v->voted = true;
			v->failures = 0;
		}
	} else if (action == AGX_VOTE_CLEAR) {
		v->voted = false;	/* the write was made; the caller logs an unacknowledged clear */
	}
}

/* The GPU driver unbinds: drop the vote now. True when bit 9 is set and the caller must write the clear. */
static inline bool agx_vote_release(struct agx_vote *v)
{
	bool was_voted = v->voted;

	v->voted = false;
	return was_voted;
}

#endif /* PMP_AGX_VOTE_H */
