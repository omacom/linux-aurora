// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * A display on a direct Type-C route, or on the fixed HDMI output, whose
 * description the firmware withdraws while the port keeps HPD: what a
 * compositor that probes the connector reads, and which hotplug events it
 * gets. The probes run the production detect, get_modes and mode_valid
 * callbacks through the DRM probe helper, as DRM_IOCTL_MODE_GETCONNECTOR
 * does.
 */
#include <kunit/static_stub.h>
#include <kunit/test.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_client.h>
#include <drm/drm_edid.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

#include "../tests/drm_kunit_edid.h"
#include "connector.h"
#include "dcp.h"
#include "dcp-fabric.h"
#include "dcp-internal.h"
#include "iomfb_v14_7.h"
#include "parser.h"

#define HOLD_MAX_RETRIES 3	/* DCP_EXTERNAL_RETRIES */
#define HOLD_RELINK_MS 2000	/* DCP_V14_DESC_RELINK_MS */

struct hold_fixture {
	struct drm_device *drm;
	struct apple_connector connector;
	struct apple_dcp dcp;
	struct platform_device pipeline;
	struct apple_dcp_typec_route route;
	struct dcp_display_mode mode;
	struct drm_client_dev observer;
	enum drm_connector_status observed_status;
	unsigned int notifications;
	atomic_t relinks;
	/* The HDMI HPD level, for the fixed HDMI output. */
	bool hdmi_hpd;
	/* HDMI relinks asked for, and the HPD level each one found. */
	unsigned int hdmi_relinks;
	bool hdmi_relink_hpd;
};

static const struct drm_display_mode hold_1080p = {
	DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED, 148352,
		 1920, 2008, 2052, 2200, 0, 1080, 1084, 1089, 1125, 0,
		 DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)
};

static const struct drm_connector_funcs hold_connector_funcs = {
	.detect = apple_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs hold_connector_helper_funcs = {
	.get_modes = dcp_get_modes,
	.mode_valid = dcp_mode_valid,
};

static int hold_observe(struct drm_client_dev *client)
{
	struct hold_fixture *f = container_of(client, struct hold_fixture, observer);

	f->notifications++;
	f->observed_status = f->connector.base.status;
	return 0;
}

static const struct drm_client_funcs hold_observer_funcs = {
	.hotplug = hold_observe,
};

/* Stands in for the Type-C reconnect: counts the relinks asked for. */
static void hold_relink_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					     typec_reconnect_wq);

	atomic_inc(&container_of(dcp, struct hold_fixture, dcp)->relinks);
}

static void hold_cleanup(void *data)
{
	struct hold_fixture *f = data;

	mutex_lock(&f->drm->clientlist_mutex);
	list_del_init(&f->observer.list);
	mutex_unlock(&f->drm->clientlist_mutex);
	cancel_delayed_work_sync(&f->dcp.external_retry_wq);
	cancel_delayed_work_sync(&f->dcp.typec_reconnect_wq);
	cancel_work_sync(&f->connector.hotplug_wq);
	cancel_work_sync(&f->connector.bl_sync_wq);
	cancel_work_sync(&f->connector.bl_commit_wq);
	apple_connector_invalidate_edid(&f->connector);
	mutex_destroy(&f->connector.edid_lock);
	mutex_destroy(&f->connector.bl_lock);
	mutex_destroy(&f->dcp.modes_lock);
	mutex_destroy(&f->dcp.hpd_mutex);
	mutex_destroy(&f->dcp.tb_lock);
}

static int hold_test_init(struct kunit *test)
{
	struct hold_fixture *f;
	struct drm_connector *connector;
	struct apple_dcp *dcp;
	struct device *dev;
	int ret;

	f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f);
	test->priv = f;
	dev = drm_kunit_helper_alloc_device(test);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	f->drm = __drm_kunit_helper_alloc_drm_device(test, dev, sizeof(*f->drm), 0,
						     DRIVER_MODESET | DRIVER_ATOMIC);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->drm);
	connector = &f->connector.base;
	ret = drmm_connector_init(f->drm, connector, &hold_connector_funcs,
				  DRM_MODE_CONNECTOR_DisplayPort, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	drm_connector_helper_add(connector, &hold_connector_helper_funcs);
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	drm_atomic_helper_connector_reset(connector);
	KUNIT_ASSERT_NOT_NULL(test, connector->state);
	apple_connector_edid_init(&f->connector);
	apple_connector_backlight_init(&f->connector);
	INIT_WORK(&f->connector.hotplug_wq, dcp_hotplug);

	/* A T6030 external pipe that drives a Type-C port, DP-alt, no tunnel. */
	dcp = &f->dcp;
	dcp->dev = dev;
	dcp->connector = &f->connector;
	dcp->external = true;
	dcp->external_native = true;
	dcp->fw_compat = DCP_FIRMWARE_V_14_7;
	dcp->fixed_connector_type = DRM_MODE_CONNECTOR_USB;
	dcp->typec_cable_connected = true;
	f->mode.mode = hold_1080p;
	dcp->modes = &f->mode;
	dcp->nr_modes = 1;
	dcp->modes_admitted = true;
	mutex_init(&dcp->modes_lock);
	mutex_init(&dcp->hpd_mutex);
	mutex_init(&dcp->tb_lock);
	spin_lock_init(&dcp->mode_state.lock);
	spin_lock_init(&dcp->dcpavserv.lock);
	init_completion(&dcp->typec_iomfb_hpd_ready);
	INIT_DELAYED_WORK(&dcp->external_retry_wq, dcp_external_retry_work);
	INIT_DELAYED_WORK(&dcp->typec_reconnect_wq, hold_relink_work);
	platform_set_drvdata(&f->pipeline, dcp);
	apple_connector_set_pipeline(&f->connector, &f->pipeline);

	f->observer.dev = f->drm;
	f->observer.name = "apple-hold-observer";
	f->observer.funcs = &hold_observer_funcs;
	mutex_lock(&f->drm->clientlist_mutex);
	list_add(&f->observer.list, &f->drm->clientlist);
	mutex_unlock(&f->drm->clientlist_mutex);
	return kunit_add_action_or_reset(test, hold_cleanup, f);
}

/* What DRM_IOCTL_MODE_GETCONNECTOR reports to the DRM master. */
static int hold_probe(struct hold_fixture *f, enum drm_connector_status *status,
		      bool *edid)
{
	int count;

	mutex_lock(&f->drm->mode_config.mutex);
	count = drm_helper_probe_single_connector_modes(&f->connector.base, 8192, 8192);
	*status = f->connector.base.status;
	if (edid)
		*edid = !!f->connector.base.edid_blob_ptr;
	mutex_unlock(&f->drm->mode_config.mutex);
	return count;
}

/* The firmware published the display's timings: the production attach. */
static void hold_attach(struct kunit *test, struct hold_fixture *f)
{
	enum drm_connector_status status;

	KUNIT_ASSERT_EQ(test, hold_probe(f, &status, NULL), 0);
	KUNIT_ASSERT_EQ(test, status, connector_status_disconnected);
	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_ASSERT_EQ(test, f->notifications, 1U);
	KUNIT_ASSERT_EQ(test, f->observed_status, connector_status_connected);
}

/* Stands in for the HDMI HPD GPIO, which the test machine does not have. */
static bool hold_hdmi_hpd_high(struct apple_dcp *dcp)
{
	return READ_ONCE(container_of(dcp, struct hold_fixture, dcp)->hdmi_hpd);
}

/* Stands in for the HDMI relink through the fabric: counts it. */
static void hold_hdmi_relink(struct apple_dcp *dcp)
{
	struct hold_fixture *f = container_of(dcp, struct hold_fixture, dcp);

	f->hdmi_relinks++;
	f->hdmi_relink_hpd = READ_ONCE(f->hdmi_hpd);
}

/*
 * The pipe drives its fixed HDMI output, with HPD at @hpd. The HPD GPIO is
 * a value no GPIO call accepts: only the stubs above read it.
 */
static void hold_use_hdmi(struct kunit *test, struct hold_fixture *f, bool hpd)
{
	f->dcp.fixed_connector_type = DRM_MODE_CONNECTOR_HDMIA;
	f->dcp.fixed_connector = &f->connector;
	f->dcp.hdmi_hpd = ERR_PTR(-ENODEV);
	f->dcp.typec_cable_connected = false;
	f->hdmi_hpd = hpd;
	kunit_activate_static_stub(test, dcp_hdmi_hpd_high, hold_hdmi_hpd_high);
	kunit_activate_static_stub(test, dcp_fabric_hdmi_retry, hold_hdmi_relink);
}

/* The queued retry is due in @ms, as one queued earlier would be. */
static void hold_retry_due_in(struct kunit *test, struct hold_fixture *f, unsigned int ms)
{
	KUNIT_ASSERT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	mod_delayed_work(system_freezable_wq, &f->dcp.external_retry_wq, msecs_to_jiffies(ms));
}

/* How long until the queued retry fires, in jiffies. */
static long hold_retry_due(struct hold_fixture *f)
{
	return (long)(READ_ONCE(f->dcp.external_retry_wq.timer.expires) - jiffies);
}

/* The desktop set the mode, which also resets the retry count. */
static void hold_mode_set(struct hold_fixture *f)
{
	dcp_mode_set_valid(&f->dcp.mode_state, true);
	atomic_set(&f->dcp.external_retries, 0);
}

/* The connector has an EDID, as after the compositor's first probe. */
static void hold_install_edid(struct kunit *test, struct hold_fixture *f)
{
	const struct drm_edid *edid;
	u64 generation;

	KUNIT_ASSERT_TRUE(test, apple_connector_edid_begin(&f->connector, &f->pipeline,
							   &generation));
	edid = drm_edid_alloc(test_edid_dvi_1080p, sizeof(test_edid_dvi_1080p));
	KUNIT_ASSERT_NOT_NULL(test, edid);
	KUNIT_ASSERT_TRUE(test, apple_connector_edid_install(&f->connector, &f->pipeline,
							     generation, edid));
}

/* The retry's timer fires now. */
static void hold_fire_retry(struct kunit *test, struct hold_fixture *f)
{
	KUNIT_ASSERT_TRUE(test, cancel_delayed_work_sync(&f->dcp.external_retry_wq));
	dcp_external_retry_work(&f->dcp.external_retry_wq.work);
	flush_delayed_work(&f->dcp.typec_reconnect_wq);
	flush_work(&f->connector.hotplug_wq);
}

/*
 * The first notification of an attach already finds the modes: the timings
 * are admitted before the connector is marked connected.
 */
static void hold_attach_has_modes(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	enum drm_connector_status status;

	hold_attach(test, f);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, NULL), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);
}

/*
 * The firmware withdraws the description right after the attach and
 * publishes it again before the retry: a probe in between still reads the
 * display, with its modes and EDID, and the return sends one event.
 */
static void hold_relink_keeps_modes(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	struct apple_connector *connector = &f->connector;
	enum drm_connector_status status;
	const struct drm_edid *edid;
	u64 generation;
	bool has_edid;

	hold_attach(test, f);
	KUNIT_ASSERT_TRUE(test, apple_connector_edid_begin(connector, &f->pipeline,
							   &generation));
	edid = drm_edid_alloc(test_edid_dvi_1080p, sizeof(test_edid_dvi_1080p));
	KUNIT_ASSERT_NOT_NULL(test, edid);
	KUNIT_ASSERT_TRUE(test, apple_connector_edid_install(connector, &f->pipeline,
							     generation, edid));
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, &has_edid), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);
	KUNIT_EXPECT_TRUE(test, has_edid);

	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->dcp.mode_state.valid));
	KUNIT_EXPECT_EQ(test, f->notifications, 1U);
	/* The read that came back empty before: connected, one mode, EDID. */
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, &has_edid), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);
	KUNIT_EXPECT_TRUE(test, has_edid);

	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_connected);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, NULL), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);

	/* The retry then finds the display back: no relink, no disconnect. */
	hold_fire_retry(test, f);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->relinks), 0);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(connector->connected));
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_connected);
}

/* Not described again by the retry: one disconnect event, then the relink. */
static void hold_not_back_releases(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	enum drm_connector_status status;

	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, f->notifications, 1U);

	hold_fire_retry(test, f);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->relinks), 1);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, NULL), 0);
	KUNIT_EXPECT_EQ(test, status, connector_status_disconnected);
}

/*
 * Only the first withdrawal of a connection is held: a display that keeps
 * being withdrawn disconnects as before. The next connection may be held.
 */
static void hold_once_per_connection(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);

	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 3U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);

	/* Unplugged and attached again: a new connection. */
	f->dcp.typec_generation++;
	cancel_delayed_work_sync(&f->dcp.external_retry_wq);
	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, f->notifications, 4U);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 4U);
}

/* With no retry left to bound it, a withdrawal disconnects as before. */
static void hold_needs_queued_retry(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_attach(test, f);
	atomic_set(&f->dcp.external_retries, HOLD_MAX_RETRIES);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
}

/*
 * The HDMI port withdraws the description after a connect and after mode
 * sets, with HPD high. A probe in the window reads the display, with its
 * modes and EDID; the return sends one event and no status change.
 */
static void hold_hdmi_keeps_modes(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	enum drm_connector_status status;
	bool has_edid;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	hold_install_edid(test, f);

	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->dcp.mode_state.valid));
	KUNIT_EXPECT_EQ(test, f->notifications, 1U);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, &has_edid), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);
	KUNIT_EXPECT_TRUE(test, has_edid);

	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_connected);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, &has_edid), 1);
	KUNIT_EXPECT_EQ(test, status, connector_status_connected);
	KUNIT_EXPECT_TRUE(test, has_edid);

	/* Set again before the retry fires: it finds the display set, silently. */
	hold_mode_set(f);
	hold_fire_retry(test, f);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->relinks), 0);
}

/*
 * Unlike a Type-C connection, every HDMI withdrawal with HPD high is held,
 * here more often than there are retries: each mode set that succeeds gives
 * the retries back. Each return sends one event; none reports a disconnect.
 */
static void hold_hdmi_every_withdrawal(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	unsigned int i;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	for (i = 0; i < HOLD_MAX_RETRIES + 1; i++) {
		iomfb_v14_7_external_withdrawn(&f->dcp);
		flush_work(&f->connector.hotplug_wq);
		KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
		KUNIT_EXPECT_TRUE(test, READ_ONCE(f->connector.connected));
		KUNIT_EXPECT_EQ(test, f->notifications, 1U + i);

		dcp_v14_external_hotplug(&f->dcp, true);
		flush_work(&f->connector.hotplug_wq);
		KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
		KUNIT_EXPECT_EQ(test, f->notifications, 2U + i);
		KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_connected);

		hold_mode_set(f);
		hold_fire_retry(test, f);
		KUNIT_EXPECT_EQ(test, f->notifications, 2U + i);
	}
}

/*
 * Not described again, and the display unplugged meanwhile: the retry
 * reports one disconnect, then goes to the HDMI relink, which finds no HPD.
 */
static void hold_hdmi_not_back_releases(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	enum drm_connector_status status;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_EQ(test, f->notifications, 1U);

	f->hdmi_hpd = false;
	hold_fire_retry(test, f);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
	KUNIT_EXPECT_EQ(test, hold_probe(f, &status, NULL), 0);
	KUNIT_EXPECT_EQ(test, status, connector_status_disconnected);
	KUNIT_EXPECT_EQ(test, f->hdmi_relinks, 1U);
	KUNIT_EXPECT_FALSE(test, f->hdmi_relink_hpd);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->relinks), 0);
}

/*
 * Not described again with the display still plugged in: one disconnect,
 * then the HDMI relink with HPD high, not the Type-C one.
 */
static void hold_hdmi_not_back_relinks(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);

	hold_fire_retry(test, f);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->hdmi_relinks, 1U);
	KUNIT_EXPECT_TRUE(test, f->hdmi_relink_hpd);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->relinks), 0);
}

/*
 * A mode set's withdrawal while the retry of an earlier one is still
 * queued, and about to fire: the hold gets the whole relink delay, and no
 * retry is charged for it. A retry due later is left as it is.
 */
static void hold_hdmi_earlier_retry_deferred(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	unsigned long expires;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	hold_mode_set(f);
	KUNIT_ASSERT_EQ(test, f->notifications, 2U);

	/* The first retry is nearly due when the mode set's withdrawal comes. */
	hold_retry_due_in(test, f, 200);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_GT(test, hold_retry_due(f),
			(long)msecs_to_jiffies(HOLD_RELINK_MS - 100));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_retries), 0);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);

	/* Back, and withdrawn again with the retry due well after the delay. */
	dcp_v14_external_hotplug(&f->dcp, true);
	flush_work(&f->connector.hotplug_wq);
	hold_retry_due_in(test, f, 4 * HOLD_RELINK_MS);
	expires = READ_ONCE(f->dcp.external_retry_wq.timer.expires);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_EQ(test, READ_ONCE(f->dcp.external_retry_wq.timer.expires), expires);
}

/*
 * A withdrawal while the pipe's connector is not its fixed HDMI one (a
 * Type-C display being handed back), or before the HDMI display was
 * reported connected: no HDMI retry and no hold, as before HDMI holds.
 */
static void hold_hdmi_only_reported_display(struct kunit *test)
{
	struct hold_fixture *f = test->priv;
	struct apple_connector *other;

	other = kunit_kzalloc(test, sizeof(*other), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, other);
	hold_use_hdmi(test, f, true);

	/* Not reported connected yet: the withdrawal changes nothing. */
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_retries), 0);

	/* Connected, but the fixed connector is another one. */
	hold_attach(test, f);
	f->dcp.fixed_connector = other;
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
}

/* The HDMI port counts as attached only with its own HPD, not lent out. */
static void hold_hdmi_port_attached_requires(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	KUNIT_EXPECT_FALSE(test, dcp_hdmi_port_attached(&f->dcp));
	hold_use_hdmi(test, f, false);
	KUNIT_EXPECT_FALSE(test, dcp_hdmi_port_attached(&f->dcp));
	f->hdmi_hpd = true;
	KUNIT_EXPECT_TRUE(test, dcp_hdmi_port_attached(&f->dcp));
	f->route.dcp = &f->dcp;
	f->dcp.active_typec_route = &f->route;
	KUNIT_EXPECT_FALSE(test, dcp_hdmi_port_attached(&f->dcp));
	f->dcp.active_typec_route = NULL;
}

/*
 * An HDMI attach while a display is held keeps the retry that bounds the
 * hold, and its count. Once the held connector was reported disconnected,
 * by any path, the attach clears the hold and starts its retries afresh.
 */
static void hold_hdmi_attach_keeps_bounding_retry(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_ASSERT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_ASSERT_EQ(test, atomic_read(&f->dcp.external_retries), 1);

	dcp_hdmi_attach_retries(&f->dcp);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_retries), 1);
	KUNIT_EXPECT_TRUE(test, iomfb_v14_7_external_holding(&f->dcp));

	WRITE_ONCE(f->connector.connected, false);
	KUNIT_EXPECT_FALSE(test, iomfb_v14_7_external_holding(&f->dcp));
	dcp_hdmi_attach_retries(&f->dcp);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_retries), 0);
}

/*
 * The retry bounding a hold is dropped because a display was attached or
 * detached since: that change owns the connector, and nothing stays held.
 */
static void hold_dropped_retry_ends_hold(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_ASSERT_EQ(test, atomic_read(&f->dcp.external_held), 1);

	f->dcp.typec_generation++;
	hold_fire_retry(test, f);
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_EQ(test, f->hdmi_relinks, 0U);
	KUNIT_EXPECT_EQ(test, f->notifications, 1U);
}

/* HDMI HPD low at the withdrawal: a disconnect at once, no retry. */
static void hold_hdmi_hpd_low_disconnects(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	f->hdmi_hpd = false;
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
}

/* HDMI with no retry left to bound the hold: a disconnect, as before. */
static void hold_hdmi_needs_queued_retry(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	hold_use_hdmi(test, f, true);
	hold_attach(test, f);
	atomic_set(&f->dcp.external_retries, HOLD_MAX_RETRIES);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	KUNIT_EXPECT_EQ(test, f->notifications, 2U);
	KUNIT_EXPECT_EQ(test, f->observed_status, connector_status_disconnected);
}

/* A Thunderbolt/USB4 tunnel disconnects at once, as before. */
static void hold_not_for_tunnel(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	/* The worker would look up the tunnel's port, which this has none of. */
	disable_work_sync(&f->connector.hotplug_wq);
	f->route.dcp = &f->dcp;
	f->route.tunnel = true;
	f->dcp.active_typec_route = &f->route;
	WRITE_ONCE(f->connector.connected, true);
	iomfb_v14_7_external_withdrawn(&f->dcp);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&f->dcp.external_retry_wq));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	KUNIT_EXPECT_FALSE(test, READ_ONCE(f->connector.connected));
	f->dcp.active_typec_route = NULL;
}

/* A hold left over from a display unplugged meanwhile reports nothing. */
static void hold_stale_release_ignored(struct kunit *test)
{
	struct hold_fixture *f = test->priv;

	atomic_set(&f->dcp.external_held, 1);
	WRITE_ONCE(f->connector.connected, false);
	KUNIT_EXPECT_FALSE(test, iomfb_v14_7_external_release(&f->dcp));
	KUNIT_EXPECT_EQ(test, atomic_read(&f->dcp.external_held), 0);
	flush_work(&f->connector.hotplug_wq);
	KUNIT_EXPECT_EQ(test, f->notifications, 0U);
}

static struct kunit_case hold_test_cases[] = {
	KUNIT_CASE(hold_attach_has_modes),
	KUNIT_CASE(hold_relink_keeps_modes),
	KUNIT_CASE(hold_not_back_releases),
	KUNIT_CASE(hold_once_per_connection),
	KUNIT_CASE(hold_needs_queued_retry),
	KUNIT_CASE(hold_hdmi_keeps_modes),
	KUNIT_CASE(hold_hdmi_every_withdrawal),
	KUNIT_CASE(hold_hdmi_not_back_releases),
	KUNIT_CASE(hold_hdmi_not_back_relinks),
	KUNIT_CASE(hold_hdmi_earlier_retry_deferred),
	KUNIT_CASE(hold_hdmi_only_reported_display),
	KUNIT_CASE(hold_hdmi_port_attached_requires),
	KUNIT_CASE(hold_hdmi_attach_keeps_bounding_retry),
	KUNIT_CASE(hold_dropped_retry_ends_hold),
	KUNIT_CASE(hold_hdmi_hpd_low_disconnects),
	KUNIT_CASE(hold_hdmi_needs_queued_retry),
	KUNIT_CASE(hold_not_for_tunnel),
	KUNIT_CASE(hold_stale_release_ignored),
	{ }
};

static struct kunit_suite hold_test_suite = {
	.name = "apple-display-external-hold",
	.init = hold_test_init,
	.test_cases = hold_test_cases,
};

kunit_test_suite(hold_test_suite);

MODULE_LICENSE("Dual MIT/GPL");
