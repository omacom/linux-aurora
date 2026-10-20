/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __CD321X_PM_H__
#define __CD321X_PM_H__

#include <linux/errno.h>

#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/usb/typec_dp.h>

#define CD321X_RESUME_ATTEMPTS 3
#define CD321X_LINK_EVENTS 3
#define CD321X_ACK_RETRY_MS 500
#define CD321X_ACK_RETRY_MAX_MS 30000
#define CD321X_MUX_RETRY_MS 500
#define CD321X_MUX_RETRY_MAX_MS 10000
/* Two minutes, over twenty times the longest hold-up seen */
#define CD321X_MUX_HELD_MS 120000
/*
 * Ordinary controller events bring retries about a second apart, with a
 * debounce for the read and another for the update. Tunnel notifications skip
 * the read debounce, but the 500 ms update debounce alone keeps this count
 * from ending a series before CD321X_MUX_HELD_MS. It only bounds its warnings.
 */
#define CD321X_MUX_RETRIES 240

enum cd321x_pm_phase {
	CD321X_PM_RUNNING,
	CD321X_PM_PREPARED,
	CD321X_PM_REVALIDATE,
	CD321X_PM_APPLY,
	CD321X_PM_STALE,
	CD321X_PM_REMOVED,
	CD321X_PM_INITIALIZING,
};

/* All transitions and the associated work scheduling hold tps->lock. */
struct cd321x_pm_state {
	enum cd321x_pm_phase phase;
	unsigned int attempts_left;
	unsigned int link_events_left;
	/* Non-zero while events may be left unacknowledged: the retry delay. */
	unsigned int ack_retry_ms;
	/* Non-zero while a mux that held a change up is retried: the retry delay. */
	unsigned int mux_retry_ms;
	/* The retries of that change so far, up to CD321X_MUX_RETRIES. */
	unsigned int mux_retries;
	/* When the change was first held up, in milliseconds. */
	unsigned int mux_since_ms;
	bool force_reconnect;
	bool provider_busy;
	bool setup_pending;
	unsigned int setup_attempts_left;
};

static inline void cd321x_pm_new_connection(struct cd321x_pm_state *pm)
{
	pm->link_events_left = CD321X_LINK_EVENTS;
	pm->provider_busy = false;
}

static inline void cd321x_pm_init(struct cd321x_pm_state *pm)
{
	pm->phase = CD321X_PM_INITIALIZING;
	cd321x_pm_new_connection(pm);
}

static inline bool cd321x_pm_ready(struct cd321x_pm_state *pm)
{
	if (pm->phase != CD321X_PM_INITIALIZING)
		return false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	if (pm->setup_pending)
		pm->setup_attempts_left = CD321X_RESUME_ATTEMPTS;
	return true;
}

static inline bool cd321x_pm_can_update(const struct cd321x_pm_state *pm)
{
	return !pm->setup_pending &&
	       (pm->phase == CD321X_PM_RUNNING || pm->phase == CD321X_PM_APPLY);
}

/*
 * Ends the retries of a change that a mux held up, see cd321x_pm_mux_held().
 * Returns true if there were any.
 */
static inline bool cd321x_pm_mux_done(struct cd321x_pm_state *pm)
{
	bool retried = pm->mux_retry_ms;

	pm->mux_retry_ms = 0;
	pm->mux_retries = 0;
	pm->mux_since_ms = 0;
	return retried;
}

static inline void cd321x_pm_prepare(struct cd321x_pm_state *pm)
{
	if (pm->phase != CD321X_PM_REMOVED)
		pm->phase = CD321X_PM_PREPARED;
	pm->attempts_left = 0;
	/* The resume read applies the state afresh. */
	cd321x_pm_mux_done(pm);
	pm->setup_attempts_left = 0;
	pm->provider_busy = false;
}

static inline bool cd321x_pm_resume(struct cd321x_pm_state *pm)
{
	if (pm->phase != CD321X_PM_PREPARED)
		return false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	if (pm->setup_pending)
		pm->setup_attempts_left = CD321X_RESUME_ATTEMPTS;
	cd321x_pm_new_connection(pm);
	return true;
}

static inline bool cd321x_pm_link_event(struct cd321x_pm_state *pm)
{
	if (pm->phase == CD321X_PM_INITIALIZING ||
	    pm->phase == CD321X_PM_PREPARED || pm->phase == CD321X_PM_REMOVED ||
	    !pm->link_events_left)
		return false;

	/* Host reprobes must not create an unlimited series of recovery runs. */
	pm->link_events_left--;
	pm->provider_busy = false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	/* A stale notification does not itself prove the current session failed. */
	return true;
}

static inline bool cd321x_pm_activation_ready(struct cd321x_pm_state *pm)
{
	/* A completion cannot refill failed recovery or restart a healthy cable. */
	if (!pm->provider_busy || pm->force_reconnect ||
	    (pm->phase != CD321X_PM_REVALIDATE && pm->phase != CD321X_PM_APPLY &&
	     pm->phase != CD321X_PM_STALE))
		return false;

	pm->provider_busy = false;
	pm->phase = CD321X_PM_REVALIDATE;
	if (!pm->attempts_left)
		pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	return true;
}

static inline bool cd321x_pm_restore(struct cd321x_pm_state *pm)
{
	/* A prepared port revalidates on its own resume. */
	if (pm->phase == CD321X_PM_INITIALIZING ||
	    pm->phase == CD321X_PM_PREPARED || pm->phase == CD321X_PM_REMOVED)
		return false;

	/* One bounded fresh read per request; not charged to the fault budget. */
	pm->provider_busy = false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	return true;
}

#define CD321X_PM_FAULT_ACCEPTED	0x1U
#define CD321X_PM_FAULT_REFUSED		0x2U
#define CD321X_PM_READY_ACCEPTED	0x4U
#define CD321X_PM_RESTORE_ACCEPTED	0x8U

/*
 * Apply the link fault and activation completion one worker pass collected.
 * An accepted fault clears provider_busy, so a completion that came with it
 * changes nothing more. A fault the budget refuses must not swallow the
 * completion.
 */
static inline unsigned int cd321x_pm_notifications(struct cd321x_pm_state *pm,
						   bool fault, bool ready,
						   bool restore)
{
	unsigned int res = 0;

	if (restore && cd321x_pm_restore(pm))
		res |= CD321X_PM_RESTORE_ACCEPTED;
	if (fault)
		res |= cd321x_pm_link_event(pm) ? CD321X_PM_FAULT_ACCEPTED :
						  CD321X_PM_FAULT_REFUSED;
	if (ready && cd321x_pm_activation_ready(pm))
		res |= CD321X_PM_READY_ACCEPTED;
	return res;
}

static inline bool cd321x_pm_begin_read(struct cd321x_pm_state *pm)
{
	if (pm->phase != CD321X_PM_REVALIDATE || !pm->attempts_left)
		return false;
	pm->attempts_left--;
	return true;
}

static inline void cd321x_pm_snapshot_ready(struct cd321x_pm_state *pm)
{
	pm->phase = CD321X_PM_APPLY;
}

static inline bool cd321x_pm_retry(struct cd321x_pm_state *pm, bool reconnect)
{
	/* Any other failure ends a hold-up: this one is budgeted. */
	cd321x_pm_mux_done(pm);
	/* Runtime cable updates need the same bounded fresh-read recovery. */
	if (pm->phase == CD321X_PM_RUNNING) {
		pm->provider_busy = !reconnect;
		pm->force_reconnect |= reconnect;
		pm->phase = CD321X_PM_REVALIDATE;
		pm->attempts_left = CD321X_RESUME_ATTEMPTS;
		return true;
	}
	if (pm->phase != CD321X_PM_REVALIDATE && pm->phase != CD321X_PM_APPLY)
		return false;
	pm->provider_busy = !reconnect;
	pm->force_reconnect |= reconnect;
	if (pm->attempts_left) {
		pm->phase = CD321X_PM_REVALIDATE;
		return true;
	}
	/* Only a fresh read, triggered by a later event, can reopen updates. */
	pm->phase = CD321X_PM_STALE;
	return false;
}

static inline bool cd321x_pm_event(struct cd321x_pm_state *pm)
{
	if (pm->phase != CD321X_PM_STALE)
		return false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	return true;
}

/*
 * Events that could not be read and acknowledged stay latched in the
 * controller, which then sends no further interrupt. Only a fresh read can
 * acknowledge them, so request one with a full budget even if an earlier
 * recovery gave up. Sleep and probe already end with such a read.
 */
static inline bool cd321x_pm_event_lost(struct cd321x_pm_state *pm)
{
	if (pm->phase == CD321X_PM_INITIALIZING ||
	    pm->phase == CD321X_PM_PREPARED || pm->phase == CD321X_PM_REMOVED)
		return false;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	return true;
}

/*
 * Until the latched events are acknowledged, the controller raises no
 * interrupt that could restart a recovery that gave up. So a snapshot read
 * that fails to acknowledge them does not count against the cable-state
 * budget, and is never given up: it gives back the attempt taken by
 * cd321x_pm_begin_read() and is retried after a delay that doubles up to
 * CD321X_ACK_RETRY_MAX_MS. Sleep cancels the retry and the resume read takes
 * it over; removal ends it. Returns the delay in milliseconds.
 */
static inline unsigned int cd321x_pm_ack_failed(struct cd321x_pm_state *pm)
{
	pm->attempts_left++;
	pm->ack_retry_ms = clamp(2 * pm->ack_retry_ms, CD321X_ACK_RETRY_MS,
				 CD321X_ACK_RETRY_MAX_MS);
	return pm->ack_retry_ms;
}

/* Returns true if acknowledging the events had failed before. */
static inline bool cd321x_pm_acked(struct cd321x_pm_state *pm)
{
	bool retried = pm->ack_retry_ms;

	pm->ack_retry_ms = 0;
	return retried;
}

/*
 * The mux provider that failed a change timed out because its hardware held
 * the change up (typec_mux_timeout_transient()). On a J700 the PHY cannot park
 * the PIPE while the controller behind the fixed hub still drives it, and a
 * partner renegotiating power can keep that up for longer than the cable-state
 * budget lasts, after which nothing would apply the state again. The cable
 * state itself was read fine, so such a failure does not count against the
 * budget: it gives back the attempt taken by cd321x_pm_begin_read(), and the
 * whole state is applied again from a fresh read, starting with SAFE, after a
 * delay that doubles up to CD321X_MUX_RETRY_MAX_MS. A later event brings the
 * retry forward, see cd321x_pm_mux_event(); sleep cancels it and the resume
 * read takes it over; removal or any other failure ends it.
 *
 * A hold-up that outlasts CD321X_MUX_HELD_MS is taken for a fault, as the
 * provider cannot tell its timeouts apart: the retries end as the budget does,
 * and only a later event starts over. That limit is on time, counted from the
 * first failure of the series, as events that bring retries forward do not
 * make the hold-up any shorter. @now_ms is the time of this failure, in
 * milliseconds on a clock that may wrap. Returns the delay in milliseconds, or
 * 0 once the retries have ended.
 */
static inline unsigned int cd321x_pm_mux_held(struct cd321x_pm_state *pm,
					      unsigned int now_ms)
{
	pm->force_reconnect = true;
	if (!pm->mux_retry_ms)
		pm->mux_since_ms = now_ms;
	if (now_ms - pm->mux_since_ms >= CD321X_MUX_HELD_MS ||
	    pm->mux_retries >= CD321X_MUX_RETRIES) {
		cd321x_pm_mux_done(pm);
		pm->phase = CD321X_PM_STALE;
		pm->attempts_left = 0;
		return 0;
	}
	if (pm->phase == CD321X_PM_RUNNING)
		pm->attempts_left = CD321X_RESUME_ATTEMPTS;
	else
		pm->attempts_left++;
	pm->phase = CD321X_PM_REVALIDATE;
	pm->mux_retries++;
	pm->mux_retry_ms = clamp(2 * pm->mux_retry_ms, CD321X_MUX_RETRY_MS,
				 CD321X_MUX_RETRY_MAX_MS);
	return pm->mux_retry_ms;
}

/*
 * Whether a controller event should bring forward the retry of a held mux: the
 * event's state is then read and applied after the usual debounce, instead of
 * waiting out the backoff.
 */
static inline bool cd321x_pm_mux_event(const struct cd321x_pm_state *pm)
{
	return pm->mux_retry_ms && pm->phase == CD321X_PM_REVALIDATE;
}

static inline void cd321x_pm_complete(struct cd321x_pm_state *pm)
{
	if (pm->phase == CD321X_PM_APPLY) {
		pm->phase = CD321X_PM_RUNNING;
		pm->attempts_left = 0;
		pm->force_reconnect = false;
		pm->provider_busy = false;
	}
}

static inline void cd321x_pm_remove(struct cd321x_pm_state *pm)
{
	pm->phase = CD321X_PM_REMOVED;
	pm->attempts_left = 0;
	cd321x_pm_mux_done(pm);
}

/*
 * A display mux reports -ENOSPC when every display pipeline is driving
 * another port.  That lasts until the other display goes away, so the port's
 * USB path is set up without the display and nothing is retried.  Any other
 * error, such as -EBUSY from a PHY in the middle of a transition, is retried.
 */
static inline bool cd321x_display_taken(int ret)
{
	return ret == -ENOSPC;
}

enum cd321x_dp_send {
	CD321X_DP_SEND_NONE,	/* the display route holds this status */
	CD321X_DP_SEND,		/* send the status */
	CD321X_DP_SEND_EDGE,	/* send it without IRQ_HPD first, then with it */
};

/*
 * What a DP alt mode update with DP status @status sends to the display
 * route, which holds @applied for the same pin assignment and configuration.
 * The route drops a status equal to the one it holds, and so does this: an
 * IRQ_HPD that the partner's last status still carries is not a new request.
 * @pulse is a new request: the sink dropped HPD and raised it again since the
 * last update (@status has IRQ_HPD set for it). When the route holds IRQ_HPD
 * already, the request is sent as an edge, so that the route sees it.
 */
static inline enum cd321x_dp_send cd321x_dp_send(u32 applied, u32 status,
						 bool pulse)
{
	if (pulse)
		return applied & DP_STATUS_IRQ_HPD ? CD321X_DP_SEND_EDGE :
						     CD321X_DP_SEND;
	return applied == status ? CD321X_DP_SEND_NONE : CD321X_DP_SEND;
}

#endif /* __CD321X_PM_H__ */
