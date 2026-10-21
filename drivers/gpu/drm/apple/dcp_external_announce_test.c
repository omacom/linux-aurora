// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2026 Ryan Murray */

#include <kunit/platform_device.h>
#include <kunit/test.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <drm/drm_device.h>

#include "connector.h"
#include "dcp.h"
#include "dcp-internal.h"

/*
 * The announcement of an external H17P display, through the detect(), the
 * hotplug worker, the timeout work and the teardown the driver runs.  A
 * stub stands in for the hotplug worker and counts the events it would
 * send, except where the real one has to race with teardown.
 */
struct announce_test {
	struct apple_dcp dcp;
	struct platform_device *pdev;
	struct apple_connector con;
	struct drm_device drm;
	atomic_t events;

	/* dcp_test_point() to pause at once, or -1 */
	int pause_at;
	struct completion paused, resume;

	struct completion hotplug_entered, hotplug_release, hotplug_done;
	struct task_struct *teardown;
	struct completion torn_down;
};

static void announce_test_noop(struct work_struct *work)
{
}

static void announce_test_event(struct work_struct *work)
{
	struct announce_test *t = container_of(work, struct announce_test, con.hotplug_wq);

	atomic_inc(&t->events);
}

/* The real hotplug worker, held back until the test lets it run. */
static void announce_test_running_hotplug(struct work_struct *work)
{
	struct announce_test *t = container_of(work, struct announce_test, con.hotplug_wq);

	complete(&t->hotplug_entered);
	wait_for_completion(&t->hotplug_release);
	dcp_hotplug(work);
	complete(&t->hotplug_done);
}

static void announce_test_point(struct apple_dcp *dcp, enum dcp_test_point point)
{
	struct announce_test *t = container_of(dcp, struct announce_test, dcp);

	if (cmpxchg(&t->pause_at, point, -1) != point)
		return;
	complete(&t->paused);
	wait_for_completion(&t->resume);
}

static void announce_test_exit(void *data)
{
	struct announce_test *t = data;

	WRITE_ONCE(t->pause_at, -1);
	complete_all(&t->resume);
	complete_all(&t->hotplug_release);
	if (t->teardown)
		wait_for_completion_timeout(&t->torn_down, 10 * HZ);
	disable_delayed_work_sync(&t->dcp.announce_wq);
	cancel_work_sync(&t->con.hotplug_wq);
}

/*
 * A display on cable generation 2, connected, whose modes are not in: no
 * catalogue has arrived for this connection.
 */
static struct announce_test *announce_test_init(struct kunit *test)
{
	struct announce_test *t;
	struct apple_dcp *dcp;

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, t);
	dcp = &t->dcp;
	t->pdev = kunit_platform_device_alloc(test, "dcp-announce", PLATFORM_DEVID_AUTO);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->pdev);
	KUNIT_ASSERT_EQ(test, kunit_platform_device_add(test, t->pdev), 0);
	platform_set_drvdata(t->pdev, dcp);

	dcp->dev = &t->pdev->dev;
	dcp->hw.iomfb_method_profile = DCP_IOMFB_METHODS_H17P_EXTERNAL;
	iomfb_queue_init(dcp);
	mutex_init(&dcp->hpd_mutex);
	spin_lock_init(&dcp->relink_lock);
	mutex_init(&dcp->dcpavserv.lock);
	spin_lock_init(&dcp->metadata.lock);
	INIT_DELAYED_WORK(&dcp->announce_wq, dcp_announce_work);
	INIT_DELAYED_WORK(&dcp->typec_reconnect_wq, announce_test_noop);
	INIT_DELAYED_WORK(&dcp->placeholder_edid_wq, announce_test_noop);
	INIT_DELAYED_WORK(&dcp->bl_fallback_wq, announce_test_noop);
	INIT_DELAYED_WORK(&dcp->backlight.fade_work, announce_test_noop);
	INIT_WORK(&dcp->external_stop_work, announce_test_noop);
	INIT_DELAYED_WORK(&dcp->external_stop_retry_wq, announce_test_noop);
	INIT_WORK(&dcp->vblank_wq, announce_test_noop);
	/* dcp_comp_unbind() calls disable_work_sync(&dcp->dimensions_wq); the
	 * real driver initialises it in dcp_modes_init() (iomfb.c), which this
	 * fixture does not run. Leaving it uninitialised makes the sync warn in
	 * __flush_work and, under KVM timing, derails the unbind so the test's
	 * pause point is missed and it times out. Stub it like the others. */
	INIT_WORK(&dcp->dimensions_wq, announce_test_noop);
	INIT_WORK(&t->con.hotplug_wq, announce_test_event);
	mutex_init(&t->drm.mode_config.mutex);

	t->pause_at = -1;
	init_completion(&t->paused);
	init_completion(&t->resume);
	init_completion(&t->hotplug_entered);
	init_completion(&t->hotplug_release);
	init_completion(&t->hotplug_done);
	init_completion(&t->torn_down);
	dcp->test_point = announce_test_point;
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, announce_test_exit, t), 0);

	t->con.base.dev = &t->drm;
	t->con.dcp = t->pdev;
	t->con.connected = true;
	dcp->connector = &t->con;
	dcp->typec_cable_connected = true;
	dcp->typec_generation = 2;
	dcp_firmware_metadata_connection(dcp, 2, false);

	return t;
}

/* A hold of @generation whose timeout ends at @deadline. */
static void announce_test_hold(struct announce_test *t, bool announced, u64 generation,
			       unsigned long deadline)
{
	struct dcp_announce *a = &t->dcp.announce;

	a->announced = announced;
	a->holding = true;
	a->timed = true;
	a->generation = generation;
	a->deadline = deadline;
}

static bool announce_test_detect(struct announce_test *t)
{
	bool connected = dcp_connector_connected(&t->con);

	flush_work(&t->con.hotplug_wq);
	return connected;
}

/*
 * A new connection whose timeout expired, probed before the timeout work
 * runs: the probe reports it and stops the timer, so it has to queue the
 * hotplug event the timer would have.  Nothing else will.
 */
static void announce_cold_timeout_probe_test(struct kunit *test)
{
	struct announce_test *t = announce_test_init(test);

	announce_test_hold(t, false, 2, jiffies - 1);
	mod_delayed_work(system_freezable_wq, &t->dcp.announce_wq, 60 * HZ);

	KUNIT_ASSERT_TRUE(test, announce_test_detect(t));
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&t->dcp.announce_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 1);
}

/*
 * A held event whose timeout expired, released by a probe while the
 * timeout work is still running: the event goes out once.
 */
static void announce_timer_probe_once_test(struct kunit *test)
{
	struct announce_test *t = announce_test_init(test);

	announce_test_hold(t, true, 2, jiffies - 1);
	t->pause_at = DCP_TEST_ANNOUNCE_TIMER_DONE;
	mod_delayed_work(system_freezable_wq, &t->dcp.announce_wq, 0);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->paused, HZ), 0);

	KUNIT_EXPECT_TRUE(test, announce_test_detect(t));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 1);
	KUNIT_EXPECT_FALSE(test, t->dcp.announce.holding);

	complete_all(&t->resume);
	flush_delayed_work(&t->dcp.announce_wq);
	flush_work(&t->con.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 1);
}

/*
 * The timeout work of a hold that outlived its cable generation, as when
 * work is enabled after another sleep: it starts a full timeout for the
 * current generation rather than wait out the deadline of the last one.
 * Nothing is ready, so no event goes out yet.
 */
static void announce_timer_next_generation_test(struct kunit *test)
{
	struct announce_test *t = announce_test_init(test);
	struct dcp_announce *a = &t->dcp.announce;
	unsigned long old_deadline = jiffies + HZ, before = jiffies;

	announce_test_hold(t, true, 1, old_deadline);
	dcp_announce_work(&t->dcp.announce_wq.work);
	flush_work(&t->con.hotplug_wq);

	KUNIT_EXPECT_TRUE(test, a->holding);
	KUNIT_EXPECT_EQ(test, a->generation, t->dcp.typec_generation);
	KUNIT_EXPECT_TRUE(test, a->timed);
	KUNIT_EXPECT_NE(test, a->deadline, old_deadline);
	KUNIT_EXPECT_TRUE(test, time_after_eq(a->deadline, before +
			  msecs_to_jiffies(DCP_ANNOUNCE_RESUME_TIMEOUT_MS)));
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&t->dcp.announce_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 0);
}

static int announce_test_unbind(void *data)
{
	struct announce_test *t = data;

	dcp_comp_unbind(t->dcp.dev, NULL, NULL);
	complete(&t->torn_down);
	return 0;
}

static int announce_test_shutdown(void *data)
{
	struct announce_test *t = data;

	dcp_platform_shutdown(t->pdev);
	complete(&t->torn_down);
	return 0;
}

/*
 * The real hotplug worker evaluates a held connection after a teardown that
 * retains the DMA resources stopped the timeout work: neither may be left
 * queued once the teardown returns.
 */
static void announce_teardown_running_hotplug(struct kunit *test,
					      int (*teardown)(void *data))
{
	struct announce_test *t = announce_test_init(test);
	struct apple_dcp *dcp = &t->dcp;

	dcp->retain_dma = true;
	announce_test_hold(t, true, 2, jiffies + 60 * HZ);
	mod_delayed_work(system_freezable_wq, &dcp->announce_wq, 60 * HZ);
	INIT_WORK(&t->con.hotplug_wq, announce_test_running_hotplug);
	schedule_work(&t->con.hotplug_wq);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->hotplug_entered, HZ), 0);

	t->pause_at = DCP_TEST_UNBIND_ANNOUNCE_STOPPED;
	t->teardown = kthread_run(teardown, t, "dcp-announce-teardown");
	KUNIT_ASSERT_FALSE(test, IS_ERR(t->teardown));
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->paused, HZ), 0);
	complete(&t->hotplug_release);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->hotplug_done, HZ), 0);
	complete_all(&t->resume);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->torn_down, HZ), 0);

	KUNIT_EXPECT_TRUE(test, dcp->quiescing);
	KUNIT_EXPECT_TRUE(test, dcp->announce.holding);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&dcp->announce_wq));
	KUNIT_EXPECT_FALSE(test, work_pending(&t->con.hotplug_wq));
}

static void announce_unbind_running_hotplug_test(struct kunit *test)
{
	announce_teardown_running_hotplug(test, announce_test_unbind);
}

static void announce_shutdown_running_hotplug_test(struct kunit *test)
{
	announce_teardown_running_hotplug(test, announce_test_shutdown);
}

/*
 * The DRM device outlives a teardown that retains the DMA resources, so
 * userspace can still probe the connector.  A probe that ends an expired
 * hold queues no work for the stopped pipeline.
 */
static void announce_probe_after_unbind_test(struct kunit *test)
{
	struct announce_test *t = announce_test_init(test);

	t->dcp.retain_dma = true;
	announce_test_hold(t, true, 2, jiffies - 1);
	dcp_comp_unbind(t->dcp.dev, NULL, NULL);

	KUNIT_EXPECT_TRUE(test, announce_test_detect(t));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 0);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&t->dcp.announce_wq));
}

/*
 * A new connection released by a probe while its work is disabled for
 * sleep: its event goes out once the work is enabled again.
 */
static void announce_release_while_stopped_test(struct kunit *test)
{
	struct announce_test *t = announce_test_init(test);

	announce_test_hold(t, false, 2, jiffies + 60 * HZ);
	dcp_disable_typec_work(&t->dcp, false);
	t->dcp.external_error = -ETIMEDOUT;

	KUNIT_EXPECT_TRUE(test, announce_test_detect(t));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 0);

	dcp_enable_typec_work(&t->dcp);
	flush_work(&t->con.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 1);
}

/*
 * A hold whose timeout had just started when the system slept again.  The
 * timeout work that enabling queues runs at once, not at the deadline of
 * the last cable generation, and waits a full timeout for the new one, as
 * for a hold that crossed a single sleep.  Nothing flushes that work, as
 * a flush runs delayed work at once.
 */
static void announce_test_sleep_again(struct kunit *test, bool announced,
				      unsigned int timeout_ms)
{
	struct announce_test *t = announce_test_init(test);
	struct dcp_announce *a = &t->dcp.announce;
	unsigned long timeout = msecs_to_jiffies(timeout_ms), before;

	announce_test_hold(t, announced, 2, jiffies + timeout);
	mod_delayed_work(system_freezable_wq, &t->dcp.announce_wq, timeout);
	dcp_disable_typec_work(&t->dcp, false);
	t->pause_at = DCP_TEST_ANNOUNCE_TIMER_DONE;
	before = jiffies;
	dcp_enable_typec_work(&t->dcp);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->paused, timeout / 2), 0);

	KUNIT_EXPECT_TRUE(test, a->holding);
	KUNIT_EXPECT_EQ(test, a->generation, 3);
	KUNIT_EXPECT_TRUE(test, a->timed);
	KUNIT_EXPECT_TRUE(test, time_after_eq(a->deadline, before + timeout));
	KUNIT_EXPECT_TRUE(test, time_before_eq(a->deadline, jiffies + timeout));
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&t->dcp.announce_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&t->events), 0);
	complete_all(&t->resume);
}

static void announce_resume_hold_sleep_again_test(struct kunit *test)
{
	announce_test_sleep_again(test, true, DCP_ANNOUNCE_RESUME_TIMEOUT_MS);
}

static void announce_new_connection_sleep_again_test(struct kunit *test)
{
	announce_test_sleep_again(test, false, DCP_ANNOUNCE_TIMEOUT_MS);
}

static struct kunit_case external_announce_cases[] = {
	KUNIT_CASE(announce_cold_timeout_probe_test),
	KUNIT_CASE(announce_timer_probe_once_test),
	KUNIT_CASE(announce_timer_next_generation_test),
	KUNIT_CASE(announce_unbind_running_hotplug_test),
	KUNIT_CASE(announce_shutdown_running_hotplug_test),
	KUNIT_CASE(announce_probe_after_unbind_test),
	KUNIT_CASE(announce_release_while_stopped_test),
	KUNIT_CASE(announce_resume_hold_sleep_again_test),
	KUNIT_CASE(announce_new_connection_sleep_again_test),
	{}
};

static struct kunit_suite external_announce_suite = {
	.name = "apple-dcp-external-announce",
	.test_cases = external_announce_cases,
};

kunit_test_suite(external_announce_suite);
