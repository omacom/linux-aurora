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

/* How long the request is held after the last operation ends. w7X measured idle gaps of up to 111.5 ms inside one 9B generation, so 100 ms
 * was not enough; 250 ms is about 2.2 times the largest gap seen. The parameter pmp_report.agx_vote_idle_ms overrides it at run time. */
#define AGX_VOTE_IDLE_MS_DEFAULT 250

/* After a set that was not acknowledged (or a PMP that is not running), wait this long before trying again. */
#define AGX_VOTE_RETRY_MS 1000

struct agx_vote {
	bool enabled;		/* the module parameter */
	bool voted;		/* bit 9 is set and was acknowledged */
	bool stopped;		/* released (GPU unbind, report removal, sleep): no new vote until agx_vote_resume() */
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

/* A retry deadline more than one backoff window in the future cannot be real: an idle gap beyond half the
 * 32-bit millisecond range makes the wrapped arithmetic read an expired deadline as future. Treat it as expired. */
static inline bool agx_vote_retry_reached(const struct agx_vote *v, agx_ms_t now)
{
	return agx_vote_reached(now, v->retry_at)
	       || (agx_ms_t)(v->retry_at - now) > AGX_VOTE_RETRY_MS;
}

/* An operation starts. now is the operation's start time in ms (any wrapping clock; the caller reads it
 * under its lock). Returns the delay in ms until the reconcile step must run, in the same terms as
 * agx_vote_end(): 1 runs it now, 0 means nothing is pending. A begin during a failed-set backoff returns 0
 * because next() already arms the retry from the failure state, and the caller's queueing can only pull a
 * deadline earlier, never push it out; it returns 1 once the deadline has passed. */
static inline agx_ms_t agx_vote_begin(struct agx_vote *v, agx_ms_t now)
{
	v->users++;
	if (!v->enabled || v->stopped)
		return 0;
	if (v->voted)
		return 0;
	return (!v->failures || agx_vote_retry_reached(v, now)) ? 1 : v->retry_at - now;
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
	if (!v->enabled || v->stopped)
		return v->voted ? AGX_VOTE_CLEAR : AGX_VOTE_NONE;
	if (v->users)
		return !v->voted && (!v->failures || agx_vote_retry_reached(v, now)) ? AGX_VOTE_SET : AGX_VOTE_NONE;
	if (v->voted && agx_vote_reached(now, v->idle_at + v->idle_ms))
		return AGX_VOTE_CLEAR;
	return AGX_VOTE_NONE;
}

/*
 * Delay in ms until the reconcile step must run again, 0 when no timer is needed. 1 when work is due now: an
 * operation began while a clear was being written (begin saw voted still true and did not kick), or an end
 * landed inside the SET write so the vote is set with an already-reached idle deadline. The caller must run the
 * reconcile those cases would otherwise skip; it cannot read 1 as "nothing pending".
 */
static inline agx_ms_t agx_vote_next(const struct agx_vote *v, agx_ms_t now)
{
	agx_ms_t at;

	if (!v->enabled || v->stopped)
		return 0;
	if (v->users && !v->voted && !v->failures)
		return 1;
	if (v->users && !v->voted && v->failures)
		at = agx_vote_retry_reached(v, now) ? now : v->retry_at;
	else if (!v->users && v->voted)
		at = v->idle_at + v->idle_ms;
	else if (!v->users && !v->voted && v->failures)
		/* The write outlasted the operation: arm the retry from the failure state
		 * itself. The caller clears failures when this reconcile finds nobody waiting
		 * (decide returns NONE), so it fires once. */
		at = agx_vote_retry_reached(v, now) ? now : v->retry_at;
	else
		return 0;
	/* An already-reached deadline returns 1: there is work, due now. The SET-write race leaves exactly
	 * this state (voted, no user, deadline passed), and 0 here would strand the vote set forever. */
	return agx_vote_reached(now, at) ? 1 : at - now;
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

/*
 * The GPU driver unbinds, the report goes away, or the system sleeps: drop the vote now and refuse new ones. True when bit 9 is set and the caller must
 * write the clear. An operation that begins afterwards (an ioctl already past the unplug check) counts as a user but does not vote.
 */
static inline bool agx_vote_release(struct agx_vote *v)
{
	bool was_voted = v->voted;

	v->voted = false;
	v->stopped = true;
	return was_voted;
}

/* Refuse new votes without touching the one that is set (the first step of a release, before the work item is waited out). */
static inline void agx_vote_refuse(struct agx_vote *v)
{
	v->stopped = true;
}

/* The GPU probes (or the report comes back, or the system resumes): votes are allowed again. True when work is already in flight and the caller must run the reconcile step. */
static inline bool agx_vote_resume(struct agx_vote *v)
{
	v->stopped = false;
	return v->enabled && v->users && !v->voted;
}

#endif /* PMP_AGX_VOTE_H */
