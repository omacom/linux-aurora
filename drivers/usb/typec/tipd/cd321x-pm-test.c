// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/module.h>

#include "cd321x-pm.h"
#include "cd321x-setup.h"

static void cd321x_resume_requires_snapshot_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	/* A queued cable update may not apply the pre-sleep snapshot. */
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_can_update(&pm));
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_RUNNING);
}

static void cd321x_revalidation_errors_bounded_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true),
				i + 1 < CD321X_RESUME_ATTEMPTS);
	}
	/* Give up without permitting an update from the old cable snapshot. */
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
}

static void cd321x_activation_failure_rereads_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	/* A PHY or ACIO error must not be treated as successful restoration. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_EQ(test, pm.attempts_left, 0U);
}

static void cd321x_abort_and_duplicate_completion_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	/* Prepare followed by completion, with no device suspend, is an abort. */
	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, pm.attempts_left, CD321X_RESUME_ATTEMPTS - 1U);
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_can_update(&pm));
}

static void cd321x_repeat_suspend_cancels_retry_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	/* The next prepare wins over a delayed retry from the previous wake. */
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, pm.attempts_left, (unsigned int)CD321X_RESUME_ATTEMPTS);
}

static void cd321x_suspend_before_apply_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	/* The debounce timer has not applied the freshly read state yet. */
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_retry(&pm, true));
}

static void cd321x_remove_drains_resume_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REMOVED);
}

static void cd321x_read_and_apply_share_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		if (i & 1)
			cd321x_pm_snapshot_ready(&pm);
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true),
				i + 1 < CD321X_RESUME_ATTEMPTS);
	}
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
}

static void cd321x_status_event_after_exhaustion_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_retry(&pm, true);
	}
	/* A power/plug-only interrupt has not read the data or cable registers. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_event(&pm));
	KUNIT_EXPECT_EQ(test, pm.attempts_left, (unsigned int)CD321X_RESUME_ATTEMPTS);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_can_update(&pm));
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_event(&pm));
}

static void cd321x_runtime_failure_revalidates_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	/* A failed ordinary cable update has already consumed its IRQ changes. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true),
				i + 1 < CD321X_RESUME_ATTEMPTS);
	}
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	/* A later physical event can retry, but cannot directly apply stale data. */
	KUNIT_EXPECT_TRUE(test, cd321x_pm_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
}

static void cd321x_healthy_resume_keeps_connection_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	/* A successful resume and an aborted suspend both revalidate in place. */
	for (i = 0; i < 2; i++) {
		cd321x_pm_prepare(&pm);
		KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
		cd321x_pm_complete(&pm);
		KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
	}
}

static void cd321x_failed_resume_forces_reconnect_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	/* Fresh cable data cannot override a provider's recorded resume failure. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	/* A new suspend must preserve recovery that has not been applied yet. */
	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
}

static void cd321x_stale_recovery_keeps_reconnect_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_retry(&pm, true);
	}
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_event(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
}

static void cd321x_late_link_failure_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	/* The link drops after the final successful resume health check. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	/* Only the provider's failed check requests a fresh forced reconnect. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, CD321X_LINK_EVENTS - 1U);
}

static void cd321x_link_event_during_final_apply_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_prepare(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		if (i + 1 < CD321X_RESUME_ATTEMPTS)
			KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	}
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_ASSERT_EQ(test, pm.attempts_left, 0U);
	/* A queued APPLY must not hide an event with its exhausted read budget. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
}

static void cd321x_link_reprobe_storm_bounded_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int event, attempt;

	cd321x_pm_new_connection(&pm);
	for (event = 0; event < CD321X_LINK_EVENTS; event++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
		for (attempt = 0; attempt < CD321X_RESUME_ATTEMPTS; attempt++) {
			KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
			cd321x_pm_snapshot_ready(&pm);
			KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true),
					attempt + 1 < CD321X_RESUME_ATTEMPTS);
		}
	}
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
}

static void cd321x_success_does_not_refill_link_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_new_connection(&pm);
	for (i = 0; i < CD321X_LINK_EVENTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		/* An old host event can reach a now healthy session: keep it. */
		KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
		cd321x_pm_complete(&pm);
	}
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_RUNNING);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
}

static void cd321x_link_event_init_and_remove_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_init(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_ready(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_ready(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
}

static void cd321x_link_event_suspend_and_rearm_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_new_connection(&pm);
	for (i = 0; i < CD321X_LINK_EVENTS; i++)
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, (unsigned int)CD321X_LINK_EVENTS);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	/* An actual attachment change also permits a bounded new sequence. */
	cd321x_pm_new_connection(&pm);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, (unsigned int)CD321X_LINK_EVENTS);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
}

static void cd321x_busy_provider_preserves_connection_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_new_connection(&pm);
	/* PCIe activation runs asynchronously after the Type-C switch returns. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_RUNNING);
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
}

static void cd321x_busy_provider_completion_reopens_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_new_connection(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		cd321x_pm_retry(&pm, false);
	}
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
	/* The PCIe worker's completion notification permits a fresh check. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, (unsigned int)CD321X_LINK_EVENTS);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_RUNNING);
	KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
}

static void cd321x_busy_provider_keeps_real_failure_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	/* Busy retries must not erase recovery requested by a real failure. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
}

static void cd321x_lost_event_rereads_test(struct kunit *test)
{
	static const enum cd321x_pm_phase phases[] = {
		CD321X_PM_RUNNING, CD321X_PM_REVALIDATE,
		CD321X_PM_APPLY, CD321X_PM_STALE,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(phases); i++) {
		struct cd321x_pm_state pm = { .phase = phases[i] };

		/* Even an exhausted recovery must read the latched events. */
		KUNIT_ASSERT_TRUE(test, cd321x_pm_event_lost(&pm));
		KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REVALIDATE);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
		KUNIT_EXPECT_FALSE(test, pm.force_reconnect);
		KUNIT_EXPECT_TRUE(test, cd321x_pm_begin_read(&pm));
		KUNIT_EXPECT_EQ(test, pm.attempts_left, CD321X_RESUME_ATTEMPTS - 1U);
	}
}

static void cd321x_lost_event_left_to_resume_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_init(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_event_lost(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_INITIALIZING);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_ready(&pm));

	/* A read failing during suspend is redone by the resume read. */
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_event_lost(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_PREPARED);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_TRUE(test, cd321x_pm_begin_read(&pm));

	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_event_lost(&pm));
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REMOVED);
}

static void cd321x_lost_event_keeps_reconnect_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_event_lost(&pm));
	/* A failed read is no evidence either way about the cable. */
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
}

static void cd321x_unacked_events_outlast_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i, delay = 0;

	KUNIT_ASSERT_TRUE(test, cd321x_pm_event_lost(&pm));
	for (i = 0; i < 4 * CD321X_RESUME_ATTEMPTS; i++) {
		unsigned int next;

		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		next = cd321x_pm_ack_failed(&pm);
		/* Never given up, and not charged to the cable-state budget... */
		KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REVALIDATE);
		KUNIT_EXPECT_EQ(test, pm.attempts_left, (unsigned int)CD321X_RESUME_ATTEMPTS);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
		/* ...only retried less and less often, down to a limit. */
		KUNIT_EXPECT_EQ(test, next, i ? min(2 * delay, CD321X_ACK_RETRY_MAX_MS) :
					     CD321X_ACK_RETRY_MS);
		delay = next;
	}
	KUNIT_EXPECT_EQ(test, delay, CD321X_ACK_RETRY_MAX_MS);

	/* Once acknowledged, the cable state has its bounded budget again. */
	KUNIT_EXPECT_TRUE(test, cd321x_pm_acked(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_acked(&pm));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true),
				i + 1 < CD321X_RESUME_ATTEMPTS);
	}
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
}

static void cd321x_unacked_events_across_sleep_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int delay;

	KUNIT_ASSERT_TRUE(test, cd321x_pm_event_lost(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	delay = cd321x_pm_ack_failed(&pm);
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	/* The resume read takes over the retries, without starting afresh. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_GT(test, cd321x_pm_ack_failed(&pm), delay);
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
}

/* A millisecond clock that wraps half way through the time a hold-up is retried */
#define TEST_WRAP_MS (0U - CD321X_MUX_HELD_MS / 2)

static void cd321x_held_mux_outlasts_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };
	unsigned int i, delay = 0, now = TEST_WRAP_MS;

	for (i = 0; i < 4 * CD321X_RESUME_ATTEMPTS; i++) {
		unsigned int next = cd321x_pm_mux_held(&pm, now);

		/* Never given up, and not charged to the cable-state budget... */
		KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REVALIDATE);
		KUNIT_EXPECT_EQ(test, pm.attempts_left, (unsigned int)CD321X_RESUME_ATTEMPTS);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
		/* ...but applied afresh from SAFE, less and less often, down to a limit. */
		KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
		KUNIT_EXPECT_EQ(test, next, i ? min(2 * delay, CD321X_MUX_RETRY_MAX_MS) :
					     CD321X_MUX_RETRY_MS);
		delay = next;
		now += delay;
		/* An event brings the retry forward, but not a read already under way. */
		KUNIT_EXPECT_TRUE(test, cd321x_pm_mux_event(&pm));
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
	}
	KUNIT_EXPECT_EQ(test, delay, CD321X_MUX_RETRY_MAX_MS);

	/* Once through, the next failure has the bounded budget again. */
	KUNIT_EXPECT_TRUE(test, cd321x_pm_mux_done(&pm));
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_done(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
	for (i = 0; i <= CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, true), i < CD321X_RESUME_ATTEMPTS);
		if (i == CD321X_RESUME_ATTEMPTS)
			break;
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
	}
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
}

static void cd321x_held_mux_across_sleep_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };

	cd321x_pm_mux_held(&pm, 0);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, CD321X_MUX_RETRY_MS),
			2 * CD321X_MUX_RETRY_MS);

	/* The resume read takes the retry over, and a new hold-up starts afresh. */
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, CD321X_MUX_HELD_MS),
			CD321X_MUX_RETRY_MS);

	/* Removal ends it. */
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
}

/* A failure of a held mux, which must be retried after a delay */
static void cd321x_held_mux_retried(struct kunit *test, struct cd321x_pm_state *pm,
				    unsigned int now)
{
	KUNIT_EXPECT_GT(test, cd321x_pm_mux_held(pm, now), 0);
	KUNIT_EXPECT_EQ(test, pm->phase, CD321X_PM_REVALIDATE);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(pm));
	cd321x_pm_snapshot_ready(pm);
}

/* A hold-up is retried for CD321X_MUX_HELD_MS from its first failure... */
static void cd321x_held_mux_gives_up_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };
	unsigned int now;

	for (now = TEST_WRAP_MS; now - TEST_WRAP_MS < CD321X_MUX_HELD_MS;
	     now += CD321X_MUX_RETRY_MAX_MS)
		cd321x_held_mux_retried(test, &pm, now);
	cd321x_held_mux_retried(test, &pm, TEST_WRAP_MS + CD321X_MUX_HELD_MS - 1);
	KUNIT_EXPECT_LT(test, pm.mux_retries, CD321X_MUX_RETRIES);

	/* ...and is then taken for a fault... */
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, TEST_WRAP_MS + CD321X_MUX_HELD_MS), 0);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_EQ(test, pm.mux_retry_ms, 0);
	KUNIT_EXPECT_EQ(test, pm.mux_retries, 0);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));

	/* ...until a later event applies the whole state again, afresh. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_event(&pm));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	now = TEST_WRAP_MS + 2 * CD321X_MUX_HELD_MS;
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, now), CD321X_MUX_RETRY_MS);
	KUNIT_EXPECT_EQ(test, pm.mux_retries, 1);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_held_mux_retried(test, &pm, now + CD321X_MUX_HELD_MS - 1);
}

/*
 * ...however many retries events bring forward: they come no closer together
 * than the two debounces, about a second.
 */
static void cd321x_held_mux_brought_forward_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };
	unsigned int i;

	for (i = 0; i < CD321X_MUX_HELD_MS / 1000; i++)
		cd321x_held_mux_retried(test, &pm, TEST_WRAP_MS + i * 1000);
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, TEST_WRAP_MS + i * 1000), 0);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
}

/* Should retries ever come faster, their number still bounds the warnings. */
static void cd321x_held_mux_retries_bounded_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };
	unsigned int i;

	for (i = 0; i < CD321X_MUX_RETRIES; i++)
		cd321x_held_mux_retried(test, &pm, TEST_WRAP_MS);
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, TEST_WRAP_MS), 0);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_EQ(test, pm.mux_retries, 0);
}

static void cd321x_held_mux_other_failure_test(struct kunit *test)
{
	struct cd321x_pm_state pm = { .phase = CD321X_PM_RUNNING };
	unsigned int i;

	for (i = 0; i < 4; i++) {
		cd321x_pm_mux_held(&pm, i * CD321X_MUX_RETRY_MS);
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
	}
	KUNIT_ASSERT_EQ(test, pm.mux_retry_ms, 8 * CD321X_MUX_RETRY_MS);

	/*
	 * A budgeted failure ends the hold-up, and what is left of the budget,
	 * all but the read under way, runs out as usual.
	 */
	for (i = 0; cd321x_pm_retry(&pm, true); i++) {
		KUNIT_EXPECT_EQ(test, pm.mux_retry_ms, 0);
		KUNIT_EXPECT_EQ(test, pm.mux_retries, 0);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_mux_event(&pm));
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
	}
	KUNIT_EXPECT_EQ(test, i, CD321X_RESUME_ATTEMPTS - 1);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_STALE);
	KUNIT_EXPECT_EQ(test, pm.mux_retry_ms, 0);

	/* The next hold-up starts over, and removal ends it. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_event(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_pm_mux_held(&pm, CD321X_MUX_HELD_MS), CD321X_MUX_RETRY_MS);
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_EQ(test, pm.mux_retry_ms, 0);
	KUNIT_EXPECT_EQ(test, pm.mux_retries, 0);
}

/* Only a pipeline owned by another port skips the retry. */
static void cd321x_display_taken_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, cd321x_display_taken(-ENOSPC));
	/* A PHY mid-transition or a pending SAFE state is retried, on every SoC. */
	KUNIT_EXPECT_FALSE(test, cd321x_display_taken(-EBUSY));
	KUNIT_EXPECT_FALSE(test, cd321x_display_taken(-EAGAIN));
	KUNIT_EXPECT_FALSE(test, cd321x_display_taken(-ETIMEDOUT));
	KUNIT_EXPECT_FALSE(test, cd321x_display_taken(0));
}

static void cd321x_completion_failure_sequence_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_new_connection(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, (unsigned int)CD321X_LINK_EVENTS);

	KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);
	/* Completion after a successful forced restart must spend no budget. */
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, CD321X_LINK_EVENTS - 1U);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, CD321X_LINK_EVENTS - 2U);
}

static void cd321x_ready_keeps_failed_recovery_bounded_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int event, attempt;

	cd321x_pm_new_connection(&pm);
	for (event = 0; event < CD321X_LINK_EVENTS; event++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
		for (attempt = 0; attempt < CD321X_RESUME_ATTEMPTS; attempt++) {
			KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
			cd321x_pm_snapshot_ready(&pm);
			cd321x_pm_retry(&pm, true);
		}
		KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	}
	KUNIT_EXPECT_FALSE(test, cd321x_pm_link_event(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, 0U);
}

static void cd321x_ready_preserves_failure_and_lifecycle_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_init(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, true));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_EXPECT_TRUE(test, pm.force_reconnect);
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
}

static void cd321x_ready_with_exhausted_fault_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_new_connection(&pm);
	for (i = 0; i < CD321X_LINK_EVENTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		cd321x_pm_complete(&pm);
	}
	/* The provider stays busy activating a host until the reads run out. */
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	for (i = 0; i < CD321X_RESUME_ATTEMPTS; i++) {
		KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
		cd321x_pm_snapshot_ready(&pm);
		KUNIT_EXPECT_EQ(test, cd321x_pm_retry(&pm, false),
				i + 1 < CD321X_RESUME_ATTEMPTS);
	}
	KUNIT_ASSERT_EQ(test, pm.phase, CD321X_PM_STALE);
	/* The refused fault must not swallow the completion that came with it. */
	KUNIT_EXPECT_EQ(test, cd321x_pm_notifications(&pm, true, true, false),
			CD321X_PM_FAULT_REFUSED | CD321X_PM_READY_ACCEPTED);
	KUNIT_EXPECT_FALSE(test, pm.provider_busy);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, 0U);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_begin_read(&pm));
}

static void cd321x_fault_outranks_coalesced_ready_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_new_connection(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	/* An accepted fault already cleared provider_busy: READY adds nothing. */
	KUNIT_EXPECT_EQ(test, cd321x_pm_notifications(&pm, true, true, false),
			CD321X_PM_FAULT_ACCEPTED);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, CD321X_LINK_EVENTS - 1U);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REVALIDATE);
	KUNIT_EXPECT_EQ(test, cd321x_pm_notifications(&pm, false, false, false), 0U);
}

static void cd321x_ready_during_apply_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};

	cd321x_pm_new_connection(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_retry(&pm, false));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	cd321x_pm_snapshot_ready(&pm);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_activation_ready(&pm));
	/* The old queued APPLY cannot erase the requested fresh read. */
	cd321x_pm_complete(&pm);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
	KUNIT_EXPECT_FALSE(test, cd321x_pm_activation_ready(&pm));
	KUNIT_ASSERT_TRUE(test, cd321x_pm_begin_read(&pm));
	KUNIT_EXPECT_EQ(test, pm.link_events_left, (unsigned int)CD321X_LINK_EVENTS);
}

struct cd321x_setup_fixture {
	u8 state;
	u64 mask;
	unsigned int reads, writes;
	unsigned int fail_read, fail_s0, fail_mask;
};

static int cd321x_fake_read_state(void *ctx, u8 *state)
{
	struct cd321x_setup_fixture *f = ctx;

	f->reads++;
	if (f->fail_read) {
		f->fail_read--;
		return -EIO;
	}
	*state = f->state;
	return 0;
}

static int cd321x_fake_read_mask(void *ctx, u64 *mask)
{
	struct cd321x_setup_fixture *f = ctx;

	f->reads++;
	*mask = f->mask;
	return 0;
}

static int cd321x_fake_set_s0(void *ctx)
{
	struct cd321x_setup_fixture *f = ctx;

	f->writes++;
	if (f->fail_s0) {
		f->fail_s0--;
		return -EIO;
	}
	f->state = 0;
	return 0;
}

static int cd321x_fake_set_mask(void *ctx, u64 mask)
{
	struct cd321x_setup_fixture *f = ctx;

	f->writes++;
	if (f->fail_mask) {
		f->fail_mask--;
		return -EIO;
	}
	f->mask = mask;
	return 0;
}

static const struct cd321x_setup_ops cd321x_fake_setup_ops = {
	.read_state = cd321x_fake_read_state,
	.read_mask = cd321x_fake_read_mask,
	.set_s0 = cd321x_fake_set_s0,
	.set_mask = cd321x_fake_set_mask,
};

static void cd321x_reset_setup_retry_test(struct kunit *test)
{
	unsigned int failure;

	for (failure = 0; failure < 3; failure++) {
		struct cd321x_pm_state pm = {};
		struct cd321x_setup_fixture f = { .state = 7, .mask = 0x0507 };

		if (!failure)
			f.fail_read = 1;
		else if (failure == 1)
			f.fail_s0 = 1;
		else
			f.fail_mask = 1;
		KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), -EIO);
		KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
		/* No further interrupt follows the cleared reset event. */
		KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 0, 0x602,
						    &cd321x_fake_setup_ops, &f), 1);
		KUNIT_EXPECT_EQ(test, f.state, (u8)0);
		KUNIT_EXPECT_EQ(test, f.mask, (u64)0x602);
		KUNIT_EXPECT_FALSE(test, pm.setup_pending);
	}
}

static void cd321x_reset_setup_bound_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	struct cd321x_setup_fixture f = { .state = 7, .mask = 0x0507, .fail_s0 = 99 };
	unsigned int i;

	for (i = 0; i < 10; i++)
		KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), -EIO);
	KUNIT_EXPECT_EQ(test, f.writes, (unsigned int)CD321X_RESUME_ATTEMPTS);
	KUNIT_EXPECT_EQ(test, pm.setup_attempts_left, 0U);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&pm));
}

static void cd321x_reset_setup_pm_fence_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	struct cd321x_setup_fixture f = { .state = 7, .mask = 0x0507, .fail_mask = 1 };

	KUNIT_ASSERT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), -EIO);
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_EQ(test, f.writes, 2U);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 0, 0x602,
						    &cd321x_fake_setup_ops, &f), 1);
	KUNIT_EXPECT_FALSE(test, pm.setup_pending);
	cd321x_pm_remove(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_EQ(test, f.writes, 4U);
	cd321x_pm_init(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_EQ(test, f.writes, 4U);
}

static void cd321x_reset_setup_healthy_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	struct cd321x_setup_fixture f = { .mask = 0x602 };

	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 0x602, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_EQ(test, f.reads, 0U);
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_EQ(test, f.reads, 2U);
	KUNIT_EXPECT_EQ(test, f.writes, 0U);
	KUNIT_EXPECT_FALSE(test, pm.setup_pending);
}

static void cd321x_reset_first_irq_during_prepare_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	struct cd321x_setup_fixture f = { .state = 7, .mask = 0x0507 };

	cd321x_pm_prepare(&pm);
	/* The IRQ is cleared before suspend has disabled the controller IRQ. */
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_TRUE(test, pm.setup_pending);
	KUNIT_EXPECT_EQ(test, f.reads, 0U);
	KUNIT_EXPECT_EQ(test, f.writes, 0U);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_PREPARED);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_resume(&pm));
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 0, 0x602,
						    &cd321x_fake_setup_ops, &f), 1);
	KUNIT_EXPECT_EQ(test, f.state, (u8)0);
	KUNIT_EXPECT_EQ(test, f.mask, (u64)0x602);
	KUNIT_EXPECT_FALSE(test, pm.setup_pending);
}

static void cd321x_reset_first_irq_before_ready_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	struct cd321x_setup_fixture f = { .state = 7, .mask = 0x0507 };

	cd321x_pm_init(&pm);
	/* The IRQ is registered before port initialization publishes ready. */
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 1, 0x602,
						    &cd321x_fake_setup_ops, &f), 0);
	KUNIT_EXPECT_TRUE(test, pm.setup_pending);
	KUNIT_EXPECT_EQ(test, f.reads, 0U);
	KUNIT_EXPECT_EQ(test, f.writes, 0U);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_INITIALIZING);
	KUNIT_ASSERT_TRUE(test, cd321x_pm_ready(&pm));
	KUNIT_EXPECT_EQ(test, cd321x_setup_run(&pm, 0, 0x602,
						    &cd321x_fake_setup_ops, &f), 1);
	KUNIT_EXPECT_EQ(test, f.state, (u8)0);
	KUNIT_EXPECT_EQ(test, f.mask, (u64)0x602);
	KUNIT_EXPECT_FALSE(test, pm.setup_pending);
}

static void cd321x_restore_ignores_fault_budget_test(struct kunit *test)
{
	struct cd321x_pm_state pm = {};
	unsigned int i;

	cd321x_pm_new_connection(&pm);
	for (i = 0; i < CD321X_LINK_EVENTS; i++)
		KUNIT_ASSERT_TRUE(test, cd321x_pm_link_event(&pm));
	KUNIT_ASSERT_FALSE(test, cd321x_pm_link_event(&pm));
	cd321x_pm_snapshot_ready(&pm);
	cd321x_pm_complete(&pm);

	/* A sleep restore still gets its bounded fresh read. */
	KUNIT_EXPECT_EQ(test, cd321x_pm_notifications(&pm, false, false, true),
			CD321X_PM_RESTORE_ACCEPTED);
	KUNIT_EXPECT_EQ(test, pm.phase, CD321X_PM_REVALIDATE);
	KUNIT_EXPECT_EQ(test, pm.attempts_left, (unsigned int)CD321X_RESUME_ATTEMPTS);
	KUNIT_EXPECT_EQ(test, pm.link_events_left, 0U);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_begin_read(&pm));

	/* A prepared port revalidates on its own resume instead. */
	cd321x_pm_prepare(&pm);
	KUNIT_EXPECT_EQ(test, cd321x_pm_notifications(&pm, false, false, true), 0U);
}

static void cd321x_dp_send_test(struct kunit *test)
{
	const u32 hpd = DP_STATUS_HPD_STATE | DP_STATUS_CON_UFP_D;
	const u32 irq = hpd | DP_STATUS_IRQ_HPD;

	/* An unchanged status is not sent, also one that carries IRQ_HPD. */
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(hpd, hpd, false), CD321X_DP_SEND_NONE);
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(irq, irq, false), CD321X_DP_SEND_NONE);
	/* A new IRQ_HPD from the partner is a changed status. */
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(hpd, irq, false), CD321X_DP_SEND);
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(irq, hpd, false), CD321X_DP_SEND);
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(hpd, 0, false), CD321X_DP_SEND);
	/* An HPD pulse is sent once, as an edge if the route holds IRQ_HPD. */
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(hpd, irq, true), CD321X_DP_SEND);
	KUNIT_EXPECT_EQ(test, cd321x_dp_send(irq, irq, true), CD321X_DP_SEND_EDGE);
}

static struct kunit_case cd321x_pm_cases[] = {
	KUNIT_CASE(cd321x_reset_first_irq_before_ready_test),
	KUNIT_CASE(cd321x_reset_first_irq_during_prepare_test),
	KUNIT_CASE(cd321x_reset_setup_retry_test),
	KUNIT_CASE(cd321x_reset_setup_bound_test),
	KUNIT_CASE(cd321x_reset_setup_pm_fence_test),
	KUNIT_CASE(cd321x_reset_setup_healthy_test),
	KUNIT_CASE(cd321x_completion_failure_sequence_test),
	KUNIT_CASE(cd321x_ready_keeps_failed_recovery_bounded_test),
	KUNIT_CASE(cd321x_ready_preserves_failure_and_lifecycle_test),
	KUNIT_CASE(cd321x_ready_with_exhausted_fault_budget_test),
	KUNIT_CASE(cd321x_fault_outranks_coalesced_ready_test),
	KUNIT_CASE(cd321x_ready_during_apply_test),
	KUNIT_CASE(cd321x_busy_provider_preserves_connection_test),
	KUNIT_CASE(cd321x_busy_provider_completion_reopens_test),
	KUNIT_CASE(cd321x_busy_provider_keeps_real_failure_test),
	KUNIT_CASE(cd321x_late_link_failure_test),
	KUNIT_CASE(cd321x_link_event_during_final_apply_test),
	KUNIT_CASE(cd321x_link_reprobe_storm_bounded_test),
	KUNIT_CASE(cd321x_success_does_not_refill_link_budget_test),
	KUNIT_CASE(cd321x_link_event_init_and_remove_test),
	KUNIT_CASE(cd321x_link_event_suspend_and_rearm_test),
	KUNIT_CASE(cd321x_healthy_resume_keeps_connection_test),
	KUNIT_CASE(cd321x_failed_resume_forces_reconnect_test),
	KUNIT_CASE(cd321x_stale_recovery_keeps_reconnect_test),
	KUNIT_CASE(cd321x_resume_requires_snapshot_test),
	KUNIT_CASE(cd321x_revalidation_errors_bounded_test),
	KUNIT_CASE(cd321x_activation_failure_rereads_test),
	KUNIT_CASE(cd321x_abort_and_duplicate_completion_test),
	KUNIT_CASE(cd321x_repeat_suspend_cancels_retry_test),
	KUNIT_CASE(cd321x_suspend_before_apply_test),
	KUNIT_CASE(cd321x_remove_drains_resume_test),
	KUNIT_CASE(cd321x_read_and_apply_share_budget_test),
	KUNIT_CASE(cd321x_status_event_after_exhaustion_test),
	KUNIT_CASE(cd321x_runtime_failure_revalidates_test),
	KUNIT_CASE(cd321x_lost_event_rereads_test),
	KUNIT_CASE(cd321x_lost_event_left_to_resume_test),
	KUNIT_CASE(cd321x_lost_event_keeps_reconnect_test),
	KUNIT_CASE(cd321x_unacked_events_outlast_budget_test),
	KUNIT_CASE(cd321x_unacked_events_across_sleep_test),
	KUNIT_CASE(cd321x_held_mux_outlasts_budget_test),
	KUNIT_CASE(cd321x_held_mux_across_sleep_test),
	KUNIT_CASE(cd321x_held_mux_gives_up_test),
	KUNIT_CASE(cd321x_held_mux_brought_forward_test),
	KUNIT_CASE(cd321x_held_mux_retries_bounded_test),
	KUNIT_CASE(cd321x_held_mux_other_failure_test),
	KUNIT_CASE(cd321x_display_taken_test),
	KUNIT_CASE(cd321x_dp_send_test),
	KUNIT_CASE(cd321x_restore_ignores_fault_budget_test),
	{}
};

static struct kunit_suite cd321x_pm_suite = {
	.name = "cd321x-pm",
	.test_cases = cd321x_pm_cases,
};

kunit_test_suite(cd321x_pm_suite);
MODULE_DESCRIPTION("KUnit tests for CD321x system sleep and cable-state revalidation");
MODULE_LICENSE("GPL");
