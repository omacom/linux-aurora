// SPDX-License-Identifier: GPL-2.0-only
/* Trace oracle transcribed from the worker at a3828b73e0ef. */
#include <kunit/test.h>
#include <linux/errno.h>
#include <linux/of.h>

#include "apple-dpin-platform.h"
#include "apple-dpin-state.h"
#include "apple-dpin-request.h"
#include "tb.h"

struct legacy_dpin {
	bool alive;
	bool handed;
	bool rearm;
	bool waiting;
	bool paused;
};

/* Keep the old boolean rules independent of the extracted state transitions. */
static unsigned int legacy_step(struct legacy_dpin *c, bool can_wait,
				enum apple_dpin_event event, bool mapped, int ret)
{
	bool want, rearm, wait, first;
	unsigned int trace = 0;

	switch (event) {
	case APPLE_DPIN_UP:
		c->alive = true;
		return APPLE_DPIN_QUEUE;
	case APPLE_DPIN_REARM:
		c->alive = true;
		c->rearm = true;
		return APPLE_DPIN_QUEUE;
	case APPLE_DPIN_DOWN:
		c->alive = false;
		return APPLE_DPIN_QUEUE;
	case APPLE_DPIN_RETRY:
		if (c->alive && c->waiting && !c->paused)
			return APPLE_DPIN_QUEUE;
		return 0;
	case APPLE_DPIN_PAUSE:
		c->paused = true;
		return 0;
	case APPLE_DPIN_RESUME:
		c->paused = false;
		if (c->alive && c->waiting)
			return APPLE_DPIN_ARM_RETRY;
		return 0;
	case APPLE_DPIN_DROPPED:
		c->handed = false;
		return 0;
	case APPLE_DPIN_WORK:
		want = c->alive;
		rearm = c->rearm;
		c->rearm = false;
		if (!want) {
			c->waiting = false;
			trace |= APPLE_DPIN_CANCEL_RETRY;
			if (c->handed || mapped)
				trace |= APPLE_DPIN_DROP;
			return trace;
		}
		if (rearm && c->handed)
			return APPLE_DPIN_DROP | APPLE_DPIN_AGAIN;
		if (c->handed)
			return 0;
		return APPLE_DPIN_ATTACH;
	case APPLE_DPIN_ADMITTED:
	case APPLE_DPIN_END_PM_GATE:
	case APPLE_DPIN_DEFER_FIRST:
		/* No such outcome existed in the baseline. */
		return 0;
	case APPLE_DPIN_RESULT:
		if (ret) {
			wait = (ret == -EBUSY || ret == -EAGAIN) && can_wait;
			want = c->alive;
			first = false;
			if (want && wait) {
				first = !c->waiting;
				c->waiting = true;
			} else {
				c->waiting = false;
			}
			if (!wait)
				trace |= APPLE_DPIN_CANCEL_RETRY;
			if (!want)
				return trace | APPLE_DPIN_AGAIN;
			if (wait) {
				if (first)
					trace |= APPLE_DPIN_FIRST_WAIT;
				if (!c->paused)
					trace |= APPLE_DPIN_ARM_RETRY;
				return trace;
			}
			return trace | APPLE_DPIN_WARN;
		}
		c->handed = true;
		if (c->waiting)
			trace |= APPLE_DPIN_RECOVERED;
		c->waiting = false;
		return trace | APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN;
	}
	return 0;
}

static void expect_legacy_fields(struct kunit *test,
				 const struct apple_dpin_state *s,
				 const struct legacy_dpin *c)
{
	KUNIT_EXPECT_EQ(test, s->alive, c->alive);
	KUNIT_EXPECT_EQ(test, s->handed, c->handed);
	KUNIT_EXPECT_EQ(test, s->rearm, c->rearm);
	KUNIT_EXPECT_EQ(test, s->waiting, c->waiting);
	KUNIT_EXPECT_EQ(test, s->paused, c->paused);
}

static void compare_transition(struct kunit *test,
			       const struct apple_dpin_policy *profile,
			       unsigned int mask, bool mapped,
			       enum apple_dpin_event event, int result)
{
	struct apple_dpin_state s = {
		.alive = !!(mask & BIT(0)),
		.handed = !!(mask & BIT(1)),
		.rearm = !!(mask & BIT(2)),
		.waiting = !!(mask & BIT(3)),
		.paused = !!(mask & BIT(4)),
	};
	struct legacy_dpin c = {
		.alive = s.alive, .handed = s.handed,
		.rearm = s.rearm, .waiting = s.waiting, .paused = s.paused,
	};
	unsigned int expected, actual;
	struct apple_dpin_policy baseline = *profile;

	baseline.defer_new_bringup = false;
	expected = legacy_step(&c, profile->capacity_retry, event, mapped, result);
	actual = apple_dpin_step(&s, &baseline, event, mapped, result);
	KUNIT_ASSERT_EQ_MSG(test, actual, expected, "mask=%u mapped=%u event=%u ret=%d",
			    mask, mapped, event, result);
	expect_legacy_fields(test, &s, &c);
}

static void apple_dpin_exhaustive_equivalence(struct kunit *test)
{
	const struct apple_dpin_policy * const profiles[] = {
		&apple_dpin_disabled, &apple_dpin_m1, &apple_dpin_m2, &apple_dpin_m3,
	};
	const int results[] = {
		0, -EBUSY, -EAGAIN, -ENODEV, -EADDRINUSE, -EOPNOTSUPP,
		-ESHUTDOWN, -ENOMEM, -ETIMEDOUT,
	};
	unsigned int profile, mask, mapped, event, result;

	for (profile = 0; profile < ARRAY_SIZE(profiles); profile++)
		for (mask = 0; mask < 32; mask++)
			for (mapped = 0; mapped < 2; mapped++)
				for (event = APPLE_DPIN_UP; event <= APPLE_DPIN_RESUME; event++)
					for (result = 0; result < ARRAY_SIZE(results); result++)
						compare_transition(test, profiles[profile], mask,
								   mapped, event, results[result]);
}

struct dpin_trace {
	enum apple_dpin_event event;
	bool mapped;
	int result;
	unsigned int actions;
	enum apple_dpin_phase phase;
};

static void run_trace(struct kunit *test, const struct apple_dpin_policy *profile,
		      const struct dpin_trace *trace, unsigned int count)
{
	struct apple_dpin_state state = {};
	struct legacy_dpin legacy = {};
	struct apple_dpin_policy baseline = *profile;
	unsigned int i, actual, expected;

	baseline.defer_new_bringup = false;

	for (i = 0; i < count; i++) {
		expected = legacy_step(&legacy, profile->capacity_retry,
				       trace[i].event, trace[i].mapped, trace[i].result);
		actual = apple_dpin_step(&state, &baseline, trace[i].event,
					 trace[i].mapped, trace[i].result);
		KUNIT_ASSERT_EQ_MSG(test, actual, trace[i].actions, "step %u", i);
		KUNIT_ASSERT_EQ_MSG(test, actual, expected, "oracle step %u", i);
		KUNIT_EXPECT_EQ_MSG(test, state.phase, trace[i].phase, "step %u", i);
		expect_legacy_fields(test, &state, &legacy);
	}
}

static void apple_dpin_capacity_trace(struct kunit *test)
{
	const struct dpin_trace trace[] = {
		{ APPLE_DPIN_UP, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, -EBUSY,
		  APPLE_DPIN_FIRST_WAIT | APPLE_DPIN_ARM_RETRY, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_PAUSE, true, 0, 0, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_RETRY, true, 0, 0, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_RESUME, true, 0, APPLE_DPIN_ARM_RETRY, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_RETRY, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_WORK, true, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, 0,
		  APPLE_DPIN_RECOVERED | APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN,
		  APPLE_DPIN_HANDED },
		{ APPLE_DPIN_WORK, true, 0, 0, APPLE_DPIN_HANDED },
		{ APPLE_DPIN_DOWN, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_WORK, true, 0,
		  APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_DROP, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_DROPPED, false, 0, 0, APPLE_DPIN_IDLE },
		{ APPLE_DPIN_DOWN, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_IDLE },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_CANCEL_RETRY, APPLE_DPIN_IDLE },
	};

	run_trace(test, &apple_dpin_m1, trace, ARRAY_SIZE(trace));
	run_trace(test, &apple_dpin_m3, trace, ARRAY_SIZE(trace));
}

static void apple_dpin_t602x_rearm_trace(struct kunit *test)
{
	const struct dpin_trace trace[] = {
		{ APPLE_DPIN_REARM, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, 0,
		  APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN, APPLE_DPIN_HANDED },
		{ APPLE_DPIN_REARM, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_HANDED },
		{ APPLE_DPIN_WORK, true, 0,
		  APPLE_DPIN_DROP | APPLE_DPIN_AGAIN, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_DROPPED, false, 0, 0, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, -EBUSY,
		  APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_WARN, APPLE_DPIN_FAILED },
		{ APPLE_DPIN_RETRY, true, 0, 0, APPLE_DPIN_FAILED },
	};

	run_trace(test, &apple_dpin_m2, trace, ARRAY_SIZE(trace));
}

static void apple_dpin_preserve_paused_event(struct kunit *test)
{
	const struct dpin_trace trace[] = {
		{ APPLE_DPIN_PAUSE, false, 0, 0, APPLE_DPIN_IDLE },
		{ APPLE_DPIN_UP, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_ACTIVATING },
		/* Keep the pre-fix behavior in the baseline oracle. */
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, -EAGAIN,
		  APPLE_DPIN_FIRST_WAIT, APPLE_DPIN_WAITING_PIPELINE },
		{ APPLE_DPIN_DOWN, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_WORK, true, 0,
		  APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_DROP, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_DROPPED, false, 0, 0, APPLE_DPIN_IDLE },
		{ APPLE_DPIN_RESUME, false, 0, 0, APPLE_DPIN_IDLE },
	};

	run_trace(test, &apple_dpin_m1, trace, ARRAY_SIZE(trace));
}

static void apple_dpin_preserve_coalesced_replace(struct kunit *test)
{
	const struct dpin_trace trace[] = {
		{ APPLE_DPIN_UP, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_RESULT, true, 0,
		  APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN, APPLE_DPIN_HANDED },
		{ APPLE_DPIN_DOWN, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_UP, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_HANDED },
		/* Raw UP preserves the oracle; the production cookie selector emits REARM. */
		{ APPLE_DPIN_WORK, true, 0, 0, APPLE_DPIN_HANDED },
	};

	run_trace(test, &apple_dpin_m1, trace, ARRAY_SIZE(trace));
	run_trace(test, &apple_dpin_m3, trace, ARRAY_SIZE(trace));
}

static void apple_dpin_unplug_during_attach(struct kunit *test)
{
	const struct dpin_trace trace[] = {
		{ APPLE_DPIN_UP, false, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_WORK, false, 0, APPLE_DPIN_ATTACH, APPLE_DPIN_ACTIVATING },
		{ APPLE_DPIN_DOWN, true, 0, APPLE_DPIN_QUEUE, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_RESULT, true, 0,
		  APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN, APPLE_DPIN_HANDED },
		{ APPLE_DPIN_WORK, true, 0,
		  APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_DROP, APPLE_DPIN_TEARDOWN },
		{ APPLE_DPIN_DROPPED, false, 0, 0, APPLE_DPIN_IDLE },
	};

	run_trace(test, &apple_dpin_m1, trace, ARRAY_SIZE(trace));
	run_trace(test, &apple_dpin_m2, trace, ARRAY_SIZE(trace));
	run_trace(test, &apple_dpin_m3, trace, ARRAY_SIZE(trace));
}

static void apple_dpin_terminal_ends_wait(struct kunit *test)
{
	const int errors[] = { -ENODEV, -EADDRINUSE, -EOPNOTSUPP, -ESHUTDOWN, -ENOMEM };
	unsigned int i, actions;

	for (i = 0; i < ARRAY_SIZE(errors); i++) {
		struct apple_dpin_state s = { .alive = true, .waiting = true };

		actions = apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT,
					  true, errors[i]);
		KUNIT_EXPECT_EQ(test, actions, APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_WARN);
		KUNIT_EXPECT_FALSE(test, s.waiting);
		KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_FAILED);
		KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, &apple_dpin_m1,
						      APPLE_DPIN_RETRY, true, 0), 0U);
	}
}

static void apple_dpin_readiness_budgets(struct kunit *test)
{
	const int results[] = { 0, -ENODEV, -EAGAIN, -EBUSY, -EADDRINUSE, -ESHUTDOWN };
	unsigned int active, tries, i;

	for (active = 0; active < 2; active++) {
		for (tries = 1; tries <= 241; tries++) {
			for (i = 0; i < ARRAY_SIZE(results); i++) {
				int ret = results[i];
				bool old_retry, new_retry;

				old_retry = !(!active || (ret != -ENODEV && ret != -EAGAIN) ||
					      tries >= (ret == -EAGAIN ? 240 : 60));
				new_retry = apple_dpin_readiness_retry(active, ret, tries);
				KUNIT_EXPECT_EQ(test, new_retry, old_retry);
			}
		}
	}
	/* The same counter is used if firmware readiness changes back to ENODEV. */
	KUNIT_EXPECT_TRUE(test, apple_dpin_readiness_retry(true, -EAGAIN, 100));
	KUNIT_EXPECT_FALSE(test, apple_dpin_readiness_retry(true, -ENODEV, 100));
	KUNIT_EXPECT_EQ(test, APPLE_DP_CONNECT_WAIT_MS, 500);
	KUNIT_EXPECT_EQ(test, APPLE_DPIN_RETRY_MS, 2000);
}

static void apple_dpin_qualification_profiles(struct kunit *test)
{
	const struct {
		const char *soc;
		const struct apple_dpin_policy *hw;
		bool routes;
		const struct apple_dpin_policy *expected;
	} cases[] = {
		{ "apple,t8103", &apple_dpin_m1, false, &apple_dpin_m1 },
		{ "apple,t6000", &apple_dpin_m1, false, &apple_dpin_m1 },
		{ "apple,t6001", &apple_dpin_m1, false, &apple_dpin_m1 },
		{ "apple,t6002", &apple_dpin_m1, true, &apple_dpin_disabled },
		{ "apple,t8112", &apple_dpin_m1, true, &apple_dpin_m1 },
		{ "apple,t8112", &apple_dpin_m1, false, &apple_dpin_m1 },
		{ "apple,t6020", &apple_dpin_m2, true, &apple_dpin_m2 },
		{ "apple,t6021", &apple_dpin_m2, true, &apple_dpin_m2 },
		{ "apple,t6020", &apple_dpin_m2, false, &apple_dpin_disabled },
		{ "apple,t6021", &apple_dpin_m2, false, &apple_dpin_disabled },
		{ "apple,t6022", &apple_dpin_m2, true, &apple_dpin_m2 },
		{ "apple,t6022", &apple_dpin_m2, false, &apple_dpin_disabled },
		{ "apple,t6030", &apple_dpin_m3, true, &apple_dpin_m3 },
		{ "apple,t6031", &apple_dpin_m3, true, &apple_dpin_disabled },
		{ "apple,unknown", &apple_dpin_m1, true, &apple_dpin_disabled },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct property compatible = {
			.name = "compatible", .value = (void *)cases[i].soc,
			.length = strlen(cases[i].soc) + 1,
		};
		struct device_node root = { .properties = &compatible };
		const struct apple_dpin_policy *policy;

		policy = apple_dpin_policy_select(cases[i].hw, &root, cases[i].routes);
		KUNIT_EXPECT_PTR_EQ(test, policy, cases[i].expected);
	}
	KUNIT_EXPECT_PTR_EQ(test, apple_dpin_policy_select(&apple_dpin_m1, NULL, true),
			    &apple_dpin_disabled);
}

static void apple_dpin_all_policy_fields(struct kunit *test)
{
	const struct apple_dpin_policy * const actual[] = {
		&apple_dpin_disabled, &apple_dpin_m1, &apple_dpin_m2, &apple_dpin_m3,
	};
	const struct apple_dpin_policy expected[] = {
		{},
		{ APPLE_DPIN_CHANGED, true, true, false, true, TB_HOST_DP_NOTIFY },
		{ APPLE_DPIN_PRE_POST, false, false, true, false,
		  TB_HOST_DP_HPD_ON_ACTIVATE | TB_HOST_DP_ACTIVE_BEFORE_DPRX |
		  TB_HOST_DP_KEEP_DPRX_TIMEOUT | TB_HOST_DP_ADAPTER_QUIRKS |
		  TB_HOST_DP_INITIAL_BW_GRANT },
		{ APPLE_DPIN_CHANGED, true, false, false, true, TB_HOST_DP_NOTIFY },
	};
	unsigned int i, queue, display;

	KUNIT_ASSERT_EQ(test, ARRAY_SIZE(actual), ARRAY_SIZE(expected));
	for (i = 0; i < ARRAY_SIZE(actual); i++) {
		KUNIT_EXPECT_EQ(test, actual[i]->flow, expected[i].flow);
		KUNIT_EXPECT_EQ(test, actual[i]->capacity_retry, expected[i].capacity_retry);
		KUNIT_EXPECT_EQ(test, actual[i]->setup_irqs, expected[i].setup_irqs);
		KUNIT_EXPECT_EQ(test, actual[i]->t602x_handshake, expected[i].t602x_handshake);
		KUNIT_EXPECT_EQ(test, actual[i]->host_policy, expected[i].host_policy);
		KUNIT_EXPECT_EQ(test, actual[i]->defer_new_bringup, expected[i].defer_new_bringup);
		for (queue = 0; queue < 2; queue++) {
			for (display = 0; display < 2; display++) {
				enum apple_dpin_flow old_hooks, new_hooks;
				unsigned long flags;

				/* Baseline: M2 pre/post ignores dp_display entirely. */
				old_hooks = queue && (i == 2 || (display && (i == 1 || i == 3))) ?
					    expected[i].flow : APPLE_DPIN_DISABLED;
				new_hooks = apple_dpin_hooks(actual[i], queue, display);
				KUNIT_EXPECT_EQ(test, new_hooks, old_hooks);
				flags = new_hooks ? actual[i]->host_policy : 0;
				KUNIT_EXPECT_EQ(test, flags,
						old_hooks ? expected[i].host_policy : 0UL);
				/* NFC credit quirk follows the installed hook policy. */
				KUNIT_EXPECT_EQ(test, !!flags, old_hooks != APPLE_DPIN_DISABLED);
			}
		}
	}
}

static void apple_dpin_policy_independent_of_hooks(struct kunit *test)
{
	struct tb_nhi_ops ops = {};
	struct tb_nhi nhi = { .ops = &ops };
	struct tb tb = { .nhi = &nhi };
	struct tb_switch sw = { .tb = &tb };
	struct tb_port in = { .sw = &sw, .config.type = TB_TYPE_DP_HDMI_IN };

	KUNIT_EXPECT_FALSE(test, tb_port_is_apple_host_dpin(&in));
	nhi.host_dp_policy = apple_dpin_m1.host_policy;
	KUNIT_EXPECT_TRUE(test, tb_port_is_apple_host_dpin(&in));
	/* M2 host quirks do not acquire the changed-notify LTTPR policy. */
	nhi.host_dp_policy = apple_dpin_m2.host_policy;
	KUNIT_EXPECT_FALSE(test, tb_port_is_apple_host_dpin(&in));
	nhi.host_dp_policy = apple_dpin_m3.host_policy;
	sw.config.route_lo = 1;
	KUNIT_EXPECT_FALSE(test, tb_port_is_apple_host_dpin(&in));
}

static unsigned int sleep_step(struct apple_dpin_state *s, enum apple_dpin_event event)
{
	return apple_dpin_step(s, &apple_dpin_m1, event, false, 0);
}

static void apple_dpin_sleep_deferred_first(struct kunit *test)
{
	struct apple_dpin_state s = {};

	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_PAUSE), 0U);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_UP), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), 0U);
	KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_SLEEP_DEFERRED);
	KUNIT_EXPECT_TRUE(test, s.deferred_first);
	KUNIT_EXPECT_FALSE(test, s.waiting);
	/* complete runs for normal resume and when another device aborts suspend. */
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_TRUE(test, s.deferred_first);
	KUNIT_EXPECT_TRUE(test, s.replay_queued);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
}

static void apple_dpin_sleep_aborted_suspend(struct kunit *test)
{
	struct apple_dpin_state s = {};

	/* Work queued before prepare has not entered DCP yet. */
	sleep_step(&s, APPLE_DPIN_UP);
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), 0U);
	/* A later device rejects suspend: PM complete must reopen admission. */
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
	KUNIT_EXPECT_FALSE(test, s.paused);
}

static void apple_dpin_sleep_unplug_before_complete(struct kunit *test)
{
	struct apple_dpin_state s = {};

	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_UP);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), 0U);
	sleep_step(&s, APPLE_DPIN_DOWN);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_CANCEL_RETRY);
	KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_IDLE);
}

static void apple_dpin_sleep_pre_call_gate(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };

	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
	/* Pause wins before up() admits mapping, IRQ setup and the first call. */
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_DEFER_FIRST), 0U);
	KUNIT_EXPECT_TRUE(test, s.deferred_first);
	KUNIT_EXPECT_FALSE(test, s.waiting);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
}

static void apple_dpin_sleep_interrupted_readiness(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };
	unsigned int actions;

	sleep_step(&s, APPLE_DPIN_WORK);
	sleep_step(&s, APPLE_DPIN_PAUSE);
	/* At least one call entered: retain the legacy capacity retry at complete. */
	actions = apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT, true, -EAGAIN);
	KUNIT_EXPECT_EQ(test, actions, APPLE_DPIN_FIRST_WAIT);
	KUNIT_EXPECT_TRUE(test, s.waiting);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_ARM_RETRY);
	/* A queued capacity retry losing the pre-call race keeps the same timing. */
	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_DEFER_FIRST);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_ARM_RETRY);
}

static void apple_dpin_sleep_handed_and_policy_scope(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true, .handed = true,
				      .phase = APPLE_DPIN_HANDED };
	const struct apple_dpin_policy * const unaffected[] = {
		&apple_dpin_m2,
	};
	unsigned int i, actions;

	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_HANDED);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), 0U);
	KUNIT_EXPECT_TRUE(test, s.handed);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_HANDED);
	for (i = 0; i < ARRAY_SIZE(unaffected); i++) {
		s = (struct apple_dpin_state){ .alive = true, .paused = true };
		KUNIT_EXPECT_FALSE(test, unaffected[i]->defer_new_bringup);
		actions = apple_dpin_step(&s, unaffected[i], APPLE_DPIN_WORK, false, 0);
		KUNIT_EXPECT_EQ(test, actions, APPLE_DPIN_ATTACH);
	}
}

static void apple_dpin_sleep_complete_races_deferral(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };

	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_DEFER_FIRST), APPLE_DPIN_AGAIN);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
}

static void apple_dpin_sleep_map_connect_pause(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };
	unsigned int actions;

	/* up() checks this under the same lock as mapping and IRQ enable. */
	KUNIT_EXPECT_FALSE(test, apple_dpin_admission_blocked(&s, &apple_dpin_m1, false));
	sleep_step(&s, APPLE_DPIN_PAUSE);
	/* Pause between map and connect cannot undo the already admitted call. */
	KUNIT_EXPECT_FALSE(test, apple_dpin_admission_blocked(&s, &apple_dpin_m1, true));
	KUNIT_EXPECT_TRUE(test, apple_dpin_admission_blocked(&s, &apple_dpin_m1, false));
	/* Its first call entered: readiness interrupted by sleep is capacity wait. */
	actions = apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT, true, -EAGAIN);
	KUNIT_EXPECT_EQ(test, actions, APPLE_DPIN_FIRST_WAIT);
	KUNIT_EXPECT_TRUE(test, s.waiting);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_ARM_RETRY);
}

static void apple_dpin_sleep_dprx_retention(struct kunit *test)
{
	struct apple_dpin_state s = {};

	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_UP);
	/* DPRX can run before the DP-IN worker latches deferred_first. */
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_WORK);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_RESUME);
	/* Complete queued the replay, but it has not admitted a first call yet. */
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_WORK);
	/* WORK plans ATTACH but actual up() admission has not happened yet. */
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_ADMITTED);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	s.waiting = true;
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_DOWN);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	s = (struct apple_dpin_state){ .alive = true, .handed = true, .paused = true };
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	s.handed = false;
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m2));
	apple_dpin_step(&s, &apple_dpin_m3, APPLE_DPIN_UP, false, 0);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m3));
}

static void apple_dpin_sleep_complete_before_work(struct kunit *test)
{
	struct apple_dpin_state s = {};

	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_UP);
	KUNIT_EXPECT_TRUE(test, s.deferred_first);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_QUEUE);
	/* An expired DPRX poll running before the first WORK must still retain. */
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	sleep_step(&s, APPLE_DPIN_ADMITTED);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	/* A new handoff replacing an old handed route has the same retention. */
	s = (struct apple_dpin_state){ .alive = true, .handed = true };
	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_REARM);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK),
			APPLE_DPIN_DROP | APPLE_DPIN_AGAIN);
	sleep_step(&s, APPLE_DPIN_DROPPED);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	/* Admission failure also ends retention instead of keeping a dead route. */
	s = (struct apple_dpin_state){ .alive = true, .deferred_first = true };
	apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT, false, -ENOMEM);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
}

static void apple_dpin_sleep_terminal_no_retention(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };

	apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT, true, -EIO);
	KUNIT_EXPECT_EQ(test, s.phase, APPLE_DPIN_FAILED);
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), 0U);
	/* Pending work before prepare is retained; an entered call is drained. */
	sleep_step(&s, APPLE_DPIN_UP);
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	sleep_step(&s, APPLE_DPIN_RESUME);
	sleep_step(&s, APPLE_DPIN_WORK);
	sleep_step(&s, APPLE_DPIN_ADMITTED);
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
}

static void apple_dpin_sleep_capacity_work_pause(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true };

	apple_dpin_step(&s, &apple_dpin_m1, APPLE_DPIN_RESULT, true, -EBUSY);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RETRY), APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
	/* Existing capacity retry loses the race between WORK and up admission. */
	sleep_step(&s, APPLE_DPIN_PAUSE);
	KUNIT_EXPECT_TRUE(test, s.waiting);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, &apple_dpin_m1));
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_RESUME), APPLE_DPIN_ARM_RETRY);
	/* Late deferral result must also retain the 2-second capacity policy. */
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_DEFER_FIRST), APPLE_DPIN_ARM_RETRY);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
}

static void apple_dpin_sleep_nhi_replacement(struct kunit *test)
{
	struct apple_dpin_state s = {};

	sleep_step(&s, APPLE_DPIN_PAUSE);
	sleep_step(&s, APPLE_DPIN_UP);
	sleep_step(&s, APPLE_DPIN_WORK);
	sleep_step(&s, APPLE_DPIN_DOWN);
	/* Ordinary unplug cannot reopen another device's prepare gate. */
	KUNIT_EXPECT_TRUE(test, s.paused);
	sleep_step(&s, APPLE_DPIN_WORK);
	/* NHI removal ends the gate only after stopping producers and draining. */
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_END_PM_GATE), 0U);
	KUNIT_EXPECT_FALSE(test, s.paused);
	KUNIT_EXPECT_FALSE(test, s.deferred_first);
	KUNIT_EXPECT_FALSE(test, s.replay_queued);
	/* Replacement NHI brings up its first tunnel without old complete(). */
	sleep_step(&s, APPLE_DPIN_UP);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
}

static void apple_dpin_cookie_initial_activate(struct kunit *test)
{
	struct apple_dpin_tokens t = {};

	KUNIT_EXPECT_FALSE(test, apple_dpin_token_request(&t, 0, true));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&t, 1, true));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 1));
	/* up() admits before fabric attach, whose Activate can be synchronous. */
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&t, 1));
	KUNIT_EXPECT_EQ(test, t.inflight, 1ULL);
	KUNIT_EXPECT_EQ(test, t.handed, 0ULL);
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 1));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&t, 1));
	KUNIT_EXPECT_EQ(test, t.inflight, 0ULL);
	KUNIT_EXPECT_EQ(test, t.handed, 1ULL);
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 1));
}

static void apple_dpin_cookie_superseded_attach(struct kunit *test)
{
	struct apple_dpin_tokens t = {};

	apple_dpin_token_request(&t, 1, true);
	apple_dpin_token_admit(&t, 1);
	/* A new request synchronously closes the old MMIO gate before drain. */
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&t, 2, true));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 1));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_complete(&t, 1));
	KUNIT_EXPECT_EQ(test, t.handed, 0ULL);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&t, 2));
	/* Old cleanup cannot revoke a new attempt or turn down its request. */
	apple_dpin_token_revoke(&t, 1);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_request(&t, 1, false));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_request(&t, 1, true));
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 2));
	KUNIT_EXPECT_EQ(test, t.inflight, 2ULL);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&t, 2));
	KUNIT_EXPECT_EQ(test, t.handed, 2ULL);
}

static void apple_dpin_cookie_down_reuse(struct kunit *test)
{
	struct apple_dpin_tokens t = {};

	apple_dpin_token_request(&t, 7, true);
	apple_dpin_token_admit(&t, 7);
	apple_dpin_token_complete(&t, 7);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&t, 7, false));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 7));
	apple_dpin_token_revoke(&t, 7);
	/* A late old UP cannot resurrect a retired handoff in a reused context. */
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_request(&t, 7, true));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_request(&t, 6, true));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&t, 8, true));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&t, 8));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 7));
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 8));
}

static void apple_dpin_cookie_readiness_retry(struct kunit *test)
{
	struct apple_dpin_tokens t = {};

	apple_dpin_token_request(&t, 11, true);
	apple_dpin_token_admit(&t, 11);
	/* Failed attach returns only after draining callbacks, then revokes. */
	apple_dpin_token_revoke(&t, 11);
	KUNIT_EXPECT_EQ(test, t.requested, 11ULL);
	KUNIT_EXPECT_EQ(test, t.latest, 11ULL);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 11));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&t, 11));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&t, 11));
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 11));
	KUNIT_EXPECT_EQ(test, t.handed, 11ULL);
}

static void apple_dpin_cookie_coalesced_rearm(struct kunit *test)
{
	struct apple_dpin_state s = { .alive = true, .handed = true };
	struct apple_dpin_tokens t = {};
	enum apple_dpin_event event;

	apple_dpin_token_request(&t, 21, true);
	apple_dpin_token_admit(&t, 21);
	apple_dpin_token_complete(&t, 21);
	/* DOWN and new UP coalesce before ordered work executes. */
	apple_dpin_token_request(&t, 21, false);
	sleep_step(&s, APPLE_DPIN_DOWN);
	event = apple_dpin_request_event(&s, &t, 22, true);
	KUNIT_EXPECT_EQ(test, event, APPLE_DPIN_REARM);
	apple_dpin_token_request(&t, 22, true);
	sleep_step(&s, event);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK),
			APPLE_DPIN_DROP | APPLE_DPIN_AGAIN);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&t, 21));
	apple_dpin_token_revoke(&t, 21);
	sleep_step(&s, APPLE_DPIN_DROPPED);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), APPLE_DPIN_ATTACH);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&t, 22));
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&t, 22));
	apple_dpin_token_complete(&t, 22);
	s.handed = true;
	/* Repeated active=true on the same handed token stays idempotent. */
	event = apple_dpin_request_event(&s, &t, 22, true);
	KUNIT_EXPECT_EQ(test, event, APPLE_DPIN_UP);
	sleep_step(&s, event);
	KUNIT_EXPECT_EQ(test, sleep_step(&s, APPLE_DPIN_WORK), 0U);
}

struct dpin_request_fake {
	struct kunit *test;
	struct apple_dpin_tokens *tokens;
	u64 old_generation;
	u64 requested;
	unsigned int inactive;
	unsigned int masked;
};

static void dpin_request_fake_inactive(void *data)
{
	struct dpin_request_fake *fake = data;

	/* The exact production executor must revoke before its first MMIO effect. */
	KUNIT_EXPECT_FALSE(fake->test,
			   apple_dpin_token_access(fake->tokens, fake->old_generation));
	KUNIT_EXPECT_EQ(fake->test, fake->tokens->requested, fake->requested);
	fake->inactive++;
}

static void dpin_request_fake_mask(void *data)
{
	struct dpin_request_fake *fake = data;

	KUNIT_EXPECT_EQ(fake->test, fake->inactive, 1U);
	fake->masked++;
}

static void apple_dpin_cookie_powered_request(struct kunit *test)
{
	const struct {
		const char *name;
		bool active;
		bool alive;
		bool mapped;
		const struct apple_dpin_policy *policy;
		u64 generation;
		bool accepted;
		unsigned int inactive;
		unsigned int masked;
	} cases[] = {
		{ "M1 replacement without DOWN", true, true, true, &apple_dpin_m1, 32, true, 1, 0 },
		{ "M2 replacement without DOWN", true, true, true, &apple_dpin_m2, 32, true, 1, 0 },
		{ "M3 replacement without DOWN", true, true, true, &apple_dpin_m3, 32, true, 1, 0 },
		{ "same active token", true, true, true, &apple_dpin_m1, 31, true, 0, 0 },
		{ "stale active token", true, true, true, &apple_dpin_m1, 30, false, 0, 0 },
		{ "stale DOWN", false, true, true, &apple_dpin_m1, 30, false, 0, 0 },
		{ "M1 powered DOWN", false, true, true, &apple_dpin_m1, 31, true, 1, 1 },
		{ "M2 powered DOWN", false, true, true, &apple_dpin_m2, 31, true, 1, 0 },
		{ "unmapped first activation", true, false, false, &apple_dpin_m1, 32, true, 0, 0 },
		{ "unpowered DOWN", false, false, true, &apple_dpin_m1, 31, true, 0, 0 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct apple_dpin_tokens tokens = {};
		struct apple_dpin_state state = {
			.alive = cases[i].alive, .handed = cases[i].mapped,
		};
		struct dpin_request_fake fake = {
			.test = test, .tokens = &tokens, .old_generation = 31,
		};
		const struct apple_dpin_request_ops ops = {
			.inactive = dpin_request_fake_inactive,
			.mask_irqs = dpin_request_fake_mask,
			.ctx = &fake,
		};
		bool accepted;

		if (cases[i].active)
			fake.requested = cases[i].generation;
		KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&tokens, 31, true));
		KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&tokens, 31));
		KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&tokens, 31));
		accepted = apple_dpin_powered_request(&state, &tokens, cases[i].policy,
						      cases[i].generation, cases[i].active,
						      cases[i].mapped, &ops);
		KUNIT_EXPECT_EQ_MSG(test, accepted, cases[i].accepted, "%s", cases[i].name);
		KUNIT_EXPECT_EQ_MSG(test, fake.inactive, cases[i].inactive, "%s", cases[i].name);
		KUNIT_EXPECT_EQ_MSG(test, fake.masked, cases[i].masked, "%s", cases[i].name);
		if (accepted && (cases[i].generation != 31 || !cases[i].active))
			KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&tokens, 31));
		else
			KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&tokens, 31));
		if (accepted) {
			KUNIT_EXPECT_EQ(test, state.alive, cases[i].active);
			KUNIT_EXPECT_EQ(test, state.rearm,
					cases[i].active && cases[i].mapped &&
					cases[i].generation != 31);
		}
	}
}

static void apple_dpin_cookie_post_inflight_replacement(struct kunit *test)
{
	struct apple_dpin_tokens tokens = {};
	struct apple_dpin_state state = { .alive = true, .phase = APPLE_DPIN_CONNECTING };
	struct dpin_request_fake fake = {
		.test = test, .tokens = &tokens, .old_generation = 31, .requested = 32,
	};
	const struct apple_dpin_request_ops ops = {
		.inactive = dpin_request_fake_inactive,
		.mask_irqs = dpin_request_fake_mask,
		.ctx = &fake,
	};

	/* An M2 post arrives while the previous attach has not returned. */
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_request(&tokens, 31, true));
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&tokens, 31));
	KUNIT_ASSERT_TRUE(test, apple_dpin_powered_request(&state, &tokens, &apple_dpin_m2,
							   32, true, true, &ops));
	KUNIT_EXPECT_EQ(test, fake.inactive, 1U);
	/* Token and event must both be visible when the hook drops the lock. */
	KUNIT_EXPECT_EQ(test, tokens.requested, 32ULL);
	KUNIT_EXPECT_EQ(test, state.phase, APPLE_DPIN_ACTIVATING);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_complete(&tokens, 31));
	apple_dpin_token_revoke(&tokens, 31);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&state, &apple_dpin_m2,
					      APPLE_DPIN_WORK, true, 0), APPLE_DPIN_ATTACH);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_admit(&tokens, 32));
	apple_dpin_step(&state, &apple_dpin_m2, APPLE_DPIN_ADMITTED, true, 0);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&tokens, 32));
	apple_dpin_step(&state, &apple_dpin_m2, APPLE_DPIN_RESULT, true, 0);
	/* Post's later code only queues: it cannot inject a delayed REARM. */
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&state, &apple_dpin_m2,
					      APPLE_DPIN_WORK, true, 0), 0U);
	KUNIT_ASSERT_TRUE(test, apple_dpin_powered_request(&state, &tokens, &apple_dpin_m2,
							   32, true, true, &ops));
	KUNIT_EXPECT_EQ(test, fake.inactive, 1U);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&state, &apple_dpin_m2,
					      APPLE_DPIN_WORK, true, 0), 0U);
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&tokens, 32));
}

struct dpin_provider_fake {
	struct kunit *test;
	struct apple_dpin_tokens tokens;
	bool available;
	bool callback_live;
	unsigned int references;
	unsigned int gets;
	unsigned int puts;
	unsigned int calls;
	unsigned int drains;
	int attach_result;
};

static bool dpin_provider_fake_get(void *data)
{
	struct dpin_provider_fake *fake = data;

	fake->gets++;
	if (!fake->available)
		return false;
	KUNIT_EXPECT_EQ(fake->test, fake->references, 0U);
	fake->references++;
	return true;
}

static bool dpin_provider_fake_held(void *data)
{
	struct dpin_provider_fake *fake = data;

	return fake->references != 0;
}

static int dpin_provider_fake_call(void *data, bool active)
{
	struct dpin_provider_fake *fake = data;

	fake->calls++;
	KUNIT_EXPECT_EQ(fake->test, fake->references, 1U);
	if (active) {
		/* Initial Activate can call the binding before attach returns. */
		fake->callback_live = true;
		KUNIT_EXPECT_TRUE(fake->test, apple_dpin_token_access(&fake->tokens, 7));
		if (!fake->attach_result)
			return 0;
	} else {
		/* Deactivate may still call the binding while detach drains it. */
		KUNIT_EXPECT_FALSE(fake->test, apple_dpin_token_access(&fake->tokens, 7));
	}
	fake->callback_live = false;
	fake->drains++;
	return active ? fake->attach_result : 0;
}

static void dpin_provider_fake_put(void *data)
{
	struct dpin_provider_fake *fake = data;

	KUNIT_EXPECT_EQ(fake->test, fake->references, 1U);
	KUNIT_EXPECT_FALSE(fake->test, fake->callback_live);
	fake->references--;
	fake->puts++;
}

static struct apple_dpin_provider_ops dpin_provider_fake_ops(struct dpin_provider_fake *fake)
{
	return (struct apple_dpin_provider_ops) {
		.get = dpin_provider_fake_get,
		.held = dpin_provider_fake_held,
		.call = dpin_provider_fake_call,
		.put = dpin_provider_fake_put,
		.ctx = fake,
	};
}

static void apple_dpin_provider_handed_lifetime(struct kunit *test)
{
	struct dpin_provider_fake fake = { .test = test, .available = true };
	struct apple_dpin_provider_ops ops = dpin_provider_fake_ops(&fake);

	apple_dpin_token_request(&fake.tokens, 7, true);
	apple_dpin_token_admit(&fake.tokens, 7);
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(true, &ops), 0);
	KUNIT_ASSERT_TRUE(test, apple_dpin_token_complete(&fake.tokens, 7));
	KUNIT_EXPECT_EQ(test, fake.references, 1U);
	KUNIT_EXPECT_EQ(test, fake.puts, 0U);
	KUNIT_EXPECT_TRUE(test, fake.callback_live);
	/* A live reference keeps ordinary module unload from entering exit. */
	KUNIT_EXPECT_TRUE(test, dpin_provider_fake_held(&fake));
	/* Detach uses that reference even when another lookup would be denied. */
	fake.available = false;
	apple_dpin_token_revoke(&fake.tokens, 7);
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(false, &ops), 0);
	KUNIT_EXPECT_EQ(test, fake.gets, 1U);
	KUNIT_EXPECT_EQ(test, fake.calls, 2U);
	KUNIT_EXPECT_EQ(test, fake.drains, 1U);
	KUNIT_EXPECT_EQ(test, fake.puts, 1U);
	KUNIT_EXPECT_EQ(test, fake.references, 0U);
	KUNIT_EXPECT_FALSE(test, fake.callback_live);
}

static void apple_dpin_provider_failed_retry(struct kunit *test)
{
	struct dpin_provider_fake fake = {
		.test = test, .available = true, .attach_result = -EAGAIN,
	};
	struct apple_dpin_provider_ops ops = dpin_provider_fake_ops(&fake);

	apple_dpin_token_request(&fake.tokens, 7, true);
	apple_dpin_token_admit(&fake.tokens, 7);
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(true, &ops), -EAGAIN);
	KUNIT_EXPECT_EQ(test, fake.references, 0U);
	KUNIT_EXPECT_EQ(test, fake.puts, 1U);
	KUNIT_EXPECT_EQ(test, fake.drains, 1U);
	KUNIT_EXPECT_FALSE(test, fake.callback_live);
	/* A failed attempt has no held binding to detach or resolve again. */
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(false, &ops), 0);
	KUNIT_EXPECT_EQ(test, fake.gets, 1U);
	KUNIT_EXPECT_EQ(test, fake.calls, 1U);
	apple_dpin_token_revoke(&fake.tokens, 7);
	apple_dpin_token_admit(&fake.tokens, 7);
	fake.attach_result = 0;
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(true, &ops), 0);
	KUNIT_EXPECT_EQ(test, fake.references, 1U);
	/* A late successful attach still owns its reference until stale cleanup. */
	apple_dpin_token_request(&fake.tokens, 8, true);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_complete(&fake.tokens, 7));
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(false, &ops), 0);
	KUNIT_EXPECT_EQ(test, fake.gets, 2U);
	KUNIT_EXPECT_EQ(test, fake.puts, 2U);
	KUNIT_EXPECT_EQ(test, fake.drains, 2U);
	KUNIT_EXPECT_EQ(test, fake.references, 0U);
	KUNIT_EXPECT_FALSE(test, fake.callback_live);
}

static void apple_dpin_provider_unavailable(struct kunit *test)
{
	struct dpin_provider_fake fake = { .test = test };
	struct apple_dpin_provider_ops ops = dpin_provider_fake_ops(&fake);

	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(true, &ops), -ENODEV);
	KUNIT_ASSERT_EQ(test, apple_dpin_provider_call(false, &ops), 0);
	KUNIT_EXPECT_EQ(test, fake.gets, 1U);
	KUNIT_EXPECT_EQ(test, fake.calls, 0U);
	KUNIT_EXPECT_EQ(test, fake.puts, 0U);
	KUNIT_EXPECT_EQ(test, fake.references, 0U);
}

static void apple_dpin_m3_capacity_suspend(struct kunit *test)
{
	struct apple_dpin_state s = {};
	struct apple_dpin_tokens tokens = {};
	const struct apple_dpin_policy *p = &apple_dpin_m3;

	apple_dpin_step(&s, p, APPLE_DPIN_UP, false, 0);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, false, 0),
			APPLE_DPIN_ATTACH);
	apple_dpin_step(&s, p, APPLE_DPIN_ADMITTED, true, 0);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RESULT, true, -EBUSY),
			APPLE_DPIN_FIRST_WAIT | APPLE_DPIN_ARM_RETRY);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, p));
	apple_dpin_step(&s, p, APPLE_DPIN_PAUSE, true, 0);
	KUNIT_EXPECT_TRUE(test, apple_dpin_admission_blocked(&s, p, false));
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RETRY, true, 0), 0U);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, true, 0), 0U);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RESUME, true, 0),
			APPLE_DPIN_ARM_RETRY);
	KUNIT_EXPECT_FALSE(test, apple_dpin_admission_blocked(&s, p, false));
	apple_dpin_step(&s, p, APPLE_DPIN_RETRY, true, 0);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, true, 0),
			APPLE_DPIN_ATTACH);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RESULT, true, 0),
			APPLE_DPIN_RECOVERED | APPLE_DPIN_LOG_CONNECTED | APPLE_DPIN_AGAIN);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, p));
	/* A handed callback remains valid while new admission is paused. */
	apple_dpin_token_request(&tokens, 1, true);
	apple_dpin_token_admit(&tokens, 1);
	apple_dpin_token_complete(&tokens, 1);
	apple_dpin_step(&s, p, APPLE_DPIN_PAUSE, true, 0);
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&tokens, 1));
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, p));
	/* A replacement request revokes the old token during the same gate. */
	apple_dpin_token_request(&tokens, 2, true);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&tokens, 1));
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_complete(&tokens, 1));
	apple_dpin_step(&s, p, APPLE_DPIN_DOWN, true, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_WORK, true, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_DROPPED, false, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_UP, false, 0);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, false, 0), 0U);
	KUNIT_EXPECT_TRUE(test, apple_dpin_awaits_display(&s, p));
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RESUME, false, 0),
			APPLE_DPIN_QUEUE);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, false, 0),
			APPLE_DPIN_ATTACH);
	apple_dpin_token_admit(&tokens, 2);
	KUNIT_EXPECT_TRUE(test, apple_dpin_token_access(&tokens, 2));
	/* Removal drains the request, then resets a gate with no complete(). */
	apple_dpin_step(&s, p, APPLE_DPIN_PAUSE, true, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_DOWN, true, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_WORK, true, 0);
	apple_dpin_step(&s, p, APPLE_DPIN_DROPPED, false, 0);
	apple_dpin_token_revoke(&tokens, 2);
	apple_dpin_step(&s, p, APPLE_DPIN_END_PM_GATE, false, 0);
	KUNIT_EXPECT_FALSE(test, s.paused);
	KUNIT_EXPECT_FALSE(test, apple_dpin_token_access(&tokens, 2));
	apple_dpin_step(&s, p, APPLE_DPIN_UP, false, 0);
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_WORK, false, 0),
			APPLE_DPIN_ATTACH);
	/* An unsupported DP IN ends the wait instead of retaining DPRX forever. */
	KUNIT_EXPECT_EQ(test, apple_dpin_step(&s, p, APPLE_DPIN_RESULT, true, -EOPNOTSUPP),
			APPLE_DPIN_CANCEL_RETRY | APPLE_DPIN_WARN);
	KUNIT_EXPECT_FALSE(test, apple_dpin_awaits_display(&s, p));
}

static struct kunit_case apple_dpin_cases[] = {
	KUNIT_CASE(apple_dpin_m3_capacity_suspend),
	KUNIT_CASE(apple_dpin_provider_handed_lifetime),
	KUNIT_CASE(apple_dpin_provider_failed_retry),
	KUNIT_CASE(apple_dpin_provider_unavailable),
	KUNIT_CASE(apple_dpin_cookie_post_inflight_replacement),
	KUNIT_CASE(apple_dpin_cookie_powered_request),
	KUNIT_CASE(apple_dpin_cookie_initial_activate),
	KUNIT_CASE(apple_dpin_cookie_superseded_attach),
	KUNIT_CASE(apple_dpin_cookie_down_reuse),
	KUNIT_CASE(apple_dpin_cookie_readiness_retry),
	KUNIT_CASE(apple_dpin_cookie_coalesced_rearm),
	KUNIT_CASE(apple_dpin_sleep_capacity_work_pause),
	KUNIT_CASE(apple_dpin_sleep_terminal_no_retention),
	KUNIT_CASE(apple_dpin_sleep_complete_before_work),
	KUNIT_CASE(apple_dpin_sleep_map_connect_pause),
	KUNIT_CASE(apple_dpin_sleep_dprx_retention),
	KUNIT_CASE(apple_dpin_sleep_nhi_replacement),
	KUNIT_CASE(apple_dpin_sleep_deferred_first),
	KUNIT_CASE(apple_dpin_sleep_aborted_suspend),
	KUNIT_CASE(apple_dpin_sleep_unplug_before_complete),
	KUNIT_CASE(apple_dpin_sleep_pre_call_gate),
	KUNIT_CASE(apple_dpin_sleep_interrupted_readiness),
	KUNIT_CASE(apple_dpin_sleep_handed_and_policy_scope),
	KUNIT_CASE(apple_dpin_sleep_complete_races_deferral),
	KUNIT_CASE(apple_dpin_exhaustive_equivalence),
	KUNIT_CASE(apple_dpin_capacity_trace),
	KUNIT_CASE(apple_dpin_t602x_rearm_trace),
	KUNIT_CASE(apple_dpin_preserve_paused_event),
	KUNIT_CASE(apple_dpin_preserve_coalesced_replace),
	KUNIT_CASE(apple_dpin_unplug_during_attach),
	KUNIT_CASE(apple_dpin_terminal_ends_wait),
	KUNIT_CASE(apple_dpin_readiness_budgets),
	KUNIT_CASE(apple_dpin_qualification_profiles),
	KUNIT_CASE(apple_dpin_all_policy_fields),
	KUNIT_CASE(apple_dpin_policy_independent_of_hooks),
	{},
};

static struct kunit_suite apple_dpin_suite = {
	.name = "apple-dpin-lifecycle",
	.test_cases = apple_dpin_cases,
};

kunit_test_suite(apple_dpin_suite);
