// SPDX-License-Identifier: GPL-2.0-only
/* Pure display-handoff decisions; the caller owns serialization and effects. */
#include <linux/errno.h>

#include "apple-dpin-state.h"
#include "nhi.h"

const struct apple_dpin_policy apple_dpin_disabled;

const struct apple_dpin_policy apple_dpin_m1 = {
	.flow = APPLE_DPIN_CHANGED,
	.capacity_retry = true,
	.setup_irqs = true,
	.defer_new_bringup = true,
	.host_policy = TB_HOST_DP_NOTIFY,
};

const struct apple_dpin_policy apple_dpin_m2 = {
	.flow = APPLE_DPIN_PRE_POST,
	/* Capacity retry remains disabled pending M2 hardware qualification. */
	.t602x_handshake = true,
	.host_policy = TB_HOST_DP_HPD_ON_ACTIVATE |
		       TB_HOST_DP_ACTIVE_BEFORE_DPRX |
		       TB_HOST_DP_KEEP_DPRX_TIMEOUT |
		       TB_HOST_DP_ADAPTER_QUIRKS |
		       TB_HOST_DP_INITIAL_BW_GRANT,
};

const struct apple_dpin_policy apple_dpin_m3 = {
	.flow = APPLE_DPIN_CHANGED,
	.capacity_retry = true,
	.defer_new_bringup = true,
	.host_policy = TB_HOST_DP_NOTIFY,
};

/*
 * T6031 (M3 Max): the T6030 flow, without bandwidth allocation mode. Its DCP
 * DPTX firmware never requests bandwidth through the DP IN adapter, so an
 * allocation-mode tunnel carries none and the sink drops the stream right
 * after the mode set (seen on an iVANKY USB4 dock, DP OUT x2 HBR3).
 */
const struct apple_dpin_policy apple_dpin_m3_max = {
	.flow = APPLE_DPIN_CHANGED,
	.capacity_retry = true,
	.defer_new_bringup = true,
	.host_policy = TB_HOST_DP_NOTIFY | TB_HOST_DP_NO_BW_ALLOC,
};

enum apple_dpin_flow apple_dpin_hooks(const struct apple_dpin_policy *policy,
				      bool queue_present, bool display_enabled)
{
	if (!queue_present || (policy->flow == APPLE_DPIN_CHANGED && !display_enabled))
		return APPLE_DPIN_DISABLED;
	return policy->flow;
}

bool apple_dpin_readiness_retry(bool active, int result, unsigned int tries)
{
	if (!active || (result != -ENODEV && result != -EAGAIN))
		return false;
	return tries < (result == -EAGAIN ? APPLE_DP_FIRMWARE_TRIES :
					  APPLE_DP_CONNECT_TRIES);
}

bool apple_dpin_token_request(struct apple_dpin_tokens *t, u64 generation, bool active)
{
	if (!generation)
		return false;
	if (active) {
		if (generation <= t->latest && t->requested != generation)
			return false;
		t->latest = generation;
		if (t->requested != generation)
			t->admitted = 0;
		t->requested = generation;
		return true;
	}
	if (t->requested != generation)
		return false;
	t->requested = 0;
	t->admitted = 0;
	return true;
}

bool apple_dpin_token_admit(struct apple_dpin_tokens *t, u64 generation)
{
	if (!generation || t->requested != generation)
		return false;
	t->inflight = generation;
	t->admitted = generation;
	return true;
}

bool apple_dpin_token_complete(struct apple_dpin_tokens *t, u64 generation)
{
	if (t->inflight == generation)
		t->inflight = 0;
	if (!apple_dpin_token_access(t, generation))
		return false;
	t->handed = generation;
	return true;
}

void apple_dpin_token_revoke(struct apple_dpin_tokens *t, u64 generation)
{
	if (t->admitted == generation)
		t->admitted = 0;
	if (t->inflight == generation)
		t->inflight = 0;
	if (t->handed == generation)
		t->handed = 0;
}

bool apple_dpin_token_access(const struct apple_dpin_tokens *t, u64 generation)
{
	return generation && t->requested == generation && t->admitted == generation;
}

bool apple_dpin_admission_blocked(const struct apple_dpin_state *s,
				  const struct apple_dpin_policy *p, bool admitted)
{
	return p->defer_new_bringup && s->paused && !admitted;
}

bool apple_dpin_awaits_display(const struct apple_dpin_state *s,
			       const struct apple_dpin_policy *p)
{
	return s->alive && (s->waiting || (p->defer_new_bringup && s->deferred_first));
}

enum apple_dpin_event apple_dpin_request_event(const struct apple_dpin_state *s,
					       const struct apple_dpin_tokens *t,
					       u64 generation, bool active)
{
	if (!active)
		return APPLE_DPIN_DOWN;
	return s->handed && t->requested != generation ? APPLE_DPIN_REARM : APPLE_DPIN_UP;
}

unsigned int apple_dpin_step(struct apple_dpin_state *s,
			     const struct apple_dpin_policy *p,
			     enum apple_dpin_event event, bool mapped, int result)
{
	unsigned int actions = 0;
	bool wait;

	switch (event) {
	case APPLE_DPIN_REARM:
		s->rearm = true;
		fallthrough;
	case APPLE_DPIN_UP:
		s->alive = true;
		if (p->defer_new_bringup && s->paused && (!s->handed || s->rearm) &&
		    !s->waiting) {
			s->deferred_first = true;
			s->replay_queued = false;
		}
		s->phase = s->handed ? APPLE_DPIN_HANDED : APPLE_DPIN_ACTIVATING;
		return APPLE_DPIN_QUEUE;
	case APPLE_DPIN_DOWN:
		s->alive = false;
		s->deferred_first = false;
		s->replay_queued = false;
		s->phase = s->handed || mapped ? APPLE_DPIN_TEARDOWN : APPLE_DPIN_IDLE;
		return APPLE_DPIN_QUEUE;
	case APPLE_DPIN_WORK:
		if (!s->alive) {
			s->deferred_first = false;
			s->replay_queued = false;
			s->rearm = false;
			s->waiting = false;
			s->phase = s->handed || mapped ? APPLE_DPIN_TEARDOWN :
						       APPLE_DPIN_IDLE;
			return APPLE_DPIN_CANCEL_RETRY |
			       (s->handed || mapped ? APPLE_DPIN_DROP : 0);
		}
		if (s->rearm && s->handed) {
			s->rearm = false;
			s->phase = APPLE_DPIN_TEARDOWN;
			return APPLE_DPIN_DROP | APPLE_DPIN_AGAIN;
		}
		s->rearm = false;
		if (s->handed) {
			s->phase = APPLE_DPIN_HANDED;
			return 0;
		}
		if (apple_dpin_admission_blocked(s, p, false)) {
			if (!s->waiting)
				s->deferred_first = true;
			s->replay_queued = false;
			s->phase = s->waiting ? APPLE_DPIN_WAITING_PIPELINE :
						APPLE_DPIN_SLEEP_DEFERRED;
			return 0;
		}
		/* Actual map/IRQ admission clears deferral, after WORK drops its lock. */
		s->phase = APPLE_DPIN_ACTIVATING;
		return APPLE_DPIN_ATTACH;
	case APPLE_DPIN_RESULT:
		s->deferred_first = false;
		s->replay_queued = false;
		if (!result) {
			s->handed = true;
			if (s->waiting)
				actions |= APPLE_DPIN_RECOVERED;
			s->waiting = false;
			s->phase = APPLE_DPIN_HANDED;
			return actions | APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN;
		}
		wait = (result == -EBUSY || result == -EAGAIN) && p->capacity_retry;
		if (s->alive && wait) {
			if (!s->waiting)
				actions |= APPLE_DPIN_FIRST_WAIT;
			s->waiting = true;
		} else {
			s->waiting = false;
		}
		if (!wait)
			actions |= APPLE_DPIN_CANCEL_RETRY;
		if (!s->alive) {
			s->phase = mapped ? APPLE_DPIN_TEARDOWN : APPLE_DPIN_IDLE;
			return actions | APPLE_DPIN_AGAIN;
		}
		if (wait) {
			s->phase = APPLE_DPIN_WAITING_PIPELINE;
			if (!s->paused)
				actions |= APPLE_DPIN_ARM_RETRY;
			return actions;
		}
		s->phase = APPLE_DPIN_FAILED;
		return actions | APPLE_DPIN_WARN;
	case APPLE_DPIN_RETRY:
		return s->alive && s->waiting && !s->paused ? APPLE_DPIN_QUEUE : 0;
	case APPLE_DPIN_DROPPED:
		s->handed = false;
		s->phase = s->alive ? APPLE_DPIN_ACTIVATING : APPLE_DPIN_IDLE;
		return 0;
	case APPLE_DPIN_PAUSE:
		s->paused = true;
		if (p->defer_new_bringup && s->alive && !s->handed && !s->waiting &&
		    s->phase == APPLE_DPIN_ACTIVATING)
			s->deferred_first = true;
		return 0;
	case APPLE_DPIN_RESUME:
		s->paused = false;
		if (p->defer_new_bringup && s->alive && s->deferred_first && !s->replay_queued) {
			s->replay_queued = true;
			s->phase = APPLE_DPIN_ACTIVATING;
			return APPLE_DPIN_QUEUE;
		}
		return s->alive && s->waiting ? APPLE_DPIN_ARM_RETRY : 0;
	case APPLE_DPIN_DEFER_FIRST:
		if (!s->alive)
			return APPLE_DPIN_AGAIN;
		if (!s->waiting)
			s->deferred_first = true;
		s->replay_queued = false;
		s->phase = s->waiting ? APPLE_DPIN_WAITING_PIPELINE :
					APPLE_DPIN_SLEEP_DEFERRED;
		/* Complete may have raced this already-admitted worker. */
		if (!s->paused)
			return s->waiting ? APPLE_DPIN_ARM_RETRY : APPLE_DPIN_AGAIN;
		return 0;
	case APPLE_DPIN_ADMITTED:
		s->phase = APPLE_DPIN_CONNECTING;
		s->deferred_first = false;
		s->replay_queued = false;
		return 0;
	case APPLE_DPIN_END_PM_GATE:
		/* The owning NHI has stopped producers and drained the old workers. */
		s->paused = false;
		s->deferred_first = false;
		s->replay_queued = false;
		return 0;
	}
	return 0;
}
