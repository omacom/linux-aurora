// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the CD321x USB4/Thunderbolt router lookup
 */

#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>
#include <kunit/test-bug.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#include "tps6598x.h"

/*
 * A Type-C connector whose port@2 is linked to a USB4 router, as on the M1
 * and M2 machines, and a connector without such a link.
 */
static const struct software_node cd321x_test_router_ep;
static const struct software_node cd321x_test_connector_ep;

static const struct property_entry cd321x_test_router_props[] = {
	PROPERTY_ENTRY_BOOL("thunderbolt-switch"),
	{ }
};

static const struct software_node cd321x_test_router = {
	.name = "cd321x-test-router",
	.properties = cd321x_test_router_props,
};

static const struct software_node cd321x_test_router_port = {
	.name = "port@1",
	.parent = &cd321x_test_router,
};

static const struct software_node_ref_args cd321x_test_router_refs[] = {
	SOFTWARE_NODE_REFERENCE(&cd321x_test_connector_ep),
};

static const struct property_entry cd321x_test_router_ep_props[] = {
	PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", cd321x_test_router_refs),
	{ }
};

static const struct software_node cd321x_test_router_ep = {
	.name = "endpoint",
	.parent = &cd321x_test_router_port,
	.properties = cd321x_test_router_ep_props,
};

static const struct software_node cd321x_test_connector = {
	.name = "cd321x-test-connector",
};

static const struct software_node cd321x_test_connector_port = {
	.name = "port@2",
	.parent = &cd321x_test_connector,
};

static const struct software_node_ref_args cd321x_test_connector_refs[] = {
	SOFTWARE_NODE_REFERENCE(&cd321x_test_router_ep),
};

static const struct property_entry cd321x_test_connector_ep_props[] = {
	PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", cd321x_test_connector_refs),
	{ }
};

static const struct software_node cd321x_test_connector_ep = {
	.name = "endpoint",
	.parent = &cd321x_test_connector_port,
	.properties = cd321x_test_connector_ep_props,
};

static const struct software_node cd321x_test_plain_connector = {
	.name = "cd321x-test-plain-connector",
};

/* The router of another port */
static const struct software_node cd321x_test_other_router = {
	.name = "cd321x-test-other-router",
	.properties = cd321x_test_router_props,
};

static const struct software_node * const cd321x_test_nodes[] = {
	&cd321x_test_router,
	&cd321x_test_router_port,
	&cd321x_test_router_ep,
	&cd321x_test_connector,
	&cd321x_test_connector_port,
	&cd321x_test_connector_ep,
	&cd321x_test_plain_connector,
	&cd321x_test_other_router,
	NULL
};

struct cd321x_tbt_test {
	struct device *dev;
	bool keep_waiting;
	unsigned int keep_waiting_calls;
	unsigned int set_calls;
	int last_state;
};

/* Stands in for the driver core's answer on whether to keep waiting. */
static bool cd321x_tbt_test_keep_waiting(struct device *dev)
{
	struct cd321x_tbt_test *ctx = kunit_get_current_test()->priv;

	ctx->keep_waiting_calls++;
	return ctx->keep_waiting;
}

static void cd321x_tbt_test_unregister_nodes(void *unused)
{
	software_node_unregister_node_group(cd321x_test_nodes);
}

static int cd321x_tbt_test_init(struct kunit *test)
{
	struct cd321x_tbt_test *ctx;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;

	ret = software_node_register_node_group(cd321x_test_nodes);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = kunit_add_action_or_reset(test, cd321x_tbt_test_unregister_nodes,
					NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ctx->dev = kunit_device_register(test, "cd321x");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->dev);

	kunit_activate_static_stub(test, cd321x_tbt_switch_keep_waiting,
				   cd321x_tbt_test_keep_waiting);
	return 0;
}

static int cd321x_tbt_test_set(struct typec_thunderbolt_switch_dev *sw,
			       const struct typec_thunderbolt_switch_data *data)
{
	struct cd321x_tbt_test *ctx = kunit_get_current_test()->priv;

	ctx->set_calls++;
	ctx->last_state = data->state;
	return 0;
}

static void cd321x_tbt_test_unregister_switch(void *sw)
{
	typec_thunderbolt_switch_unregister(sw);
}

/* A connector without a router link never waits. */
static void cd321x_tbt_no_router_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct typec_thunderbolt_switch *sw;
	bool late = true;

	sw = cd321x_tbt_switch_get(ctx->dev,
				   software_node_fwnode(&cd321x_test_plain_connector),
				   &late);
	KUNIT_EXPECT_NULL(test, sw);
	KUNIT_EXPECT_FALSE(test, late);
	KUNIT_EXPECT_EQ(test, ctx->keep_waiting_calls, 0);
}

/* A router whose driver has registered its switch is used. */
static void cd321x_tbt_router_bound_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = software_node_fwnode(&cd321x_test_router),
		.set = cd321x_tbt_test_set,
	};
	struct typec_thunderbolt_switch_dev *sw_dev;
	struct typec_thunderbolt_switch *sw;
	struct device *router;
	bool late = true;
	int ret;

	router = kunit_device_register(test, "router");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, router);
	/* The lookup takes a reference on the router driver's module. */
	KUNIT_ASSERT_NOT_NULL(test, router->driver);

	sw_dev = typec_thunderbolt_switch_register(router, &desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sw_dev);
	ret = kunit_add_action_or_reset(test, cd321x_tbt_test_unregister_switch,
					sw_dev);
	KUNIT_ASSERT_EQ(test, ret, 0);

	sw = cd321x_tbt_switch_get(ctx->dev,
				   software_node_fwnode(&cd321x_test_connector),
				   &late);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, sw);
	KUNIT_EXPECT_FALSE(test, late);
	KUNIT_EXPECT_EQ(test, ctx->keep_waiting_calls, 0);
	typec_thunderbolt_switch_put(sw);
}

/* A router that has not registered its switch yet defers the port. */
static void cd321x_tbt_router_pending_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct typec_thunderbolt_switch *sw;
	bool late = true;

	ctx->keep_waiting = true;
	sw = cd321x_tbt_switch_get(ctx->dev,
				   software_node_fwnode(&cd321x_test_connector),
				   &late);
	KUNIT_EXPECT_PTR_EQ(test, sw, ERR_PTR(-EPROBE_DEFER));
	KUNIT_EXPECT_FALSE(test, late);
	KUNIT_EXPECT_EQ(test, ctx->keep_waiting_calls, 1);
}

/*
 * Once the driver core stops waiting for missing dependencies, a router that
 * has not registered its switch no longer keeps the port from probing, and the
 * port is told to take the router if it binds later.
 */
static void cd321x_tbt_router_never_binds_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct typec_thunderbolt_switch *sw;
	bool late = false;

	ctx->keep_waiting = false;
	sw = cd321x_tbt_switch_get(ctx->dev,
				   software_node_fwnode(&cd321x_test_connector),
				   &late);
	KUNIT_EXPECT_NULL(test, sw);
	KUNIT_EXPECT_TRUE(test, late);
	KUNIT_EXPECT_EQ(test, ctx->keep_waiting_calls, 1);
}

static int cd321x_tbt_test_notify(struct notifier_block *nb,
				  unsigned long event, void *data)
{
	return NOTIFY_OK;
}

/* Safe to repeat: tests release a port early to check the result. */
static void cd321x_tbt_test_release_port(struct cd321x *cd)
{
	cd321x_stop_work(cd);
	typec_thunderbolt_switch_put(cd->tbt_switch);
	cd->tbt_switch = NULL;
	fwnode_handle_put(cd->tbt_fwnode);
	cd->tbt_fwnode = NULL;
}

static void cd321x_tbt_test_release_port_action(void *cd)
{
	cd321x_tbt_test_release_port(cd);
}

/* A port that probed without its router, with no hardware behind it */
static struct cd321x *cd321x_tbt_test_late_port(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct cd321x *cd;
	int ret;

	cd = kunit_kzalloc(test, sizeof(*cd), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, cd);
	mutex_init(&cd->tps.lock);
	cd->tps.dev = ctx->dev;
	/* Still initializing: the queued cable read stops before the hardware. */
	cd321x_init_work(cd);
	cd->tbt_notifier.notifier_call = cd321x_tbt_test_notify;
	/* Leave no freed port on the global watch list if an assertion fails. */
	ret = kunit_add_action_or_reset(test, cd321x_tbt_test_release_port_action,
					cd);
	KUNIT_ASSERT_EQ(test, ret, 0);
	return cd;
}

static void cd321x_tbt_test_add_router(struct kunit *test,
				       const struct software_node *node)
{
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = software_node_fwnode(node),
		.set = cd321x_tbt_test_set,
		.name = node->name,
	};
	struct typec_thunderbolt_switch_dev *sw_dev;
	struct device *router;
	int ret;

	router = kunit_device_register(test, node->name);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, router);
	sw_dev = typec_thunderbolt_switch_register(router, &desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sw_dev);
	ret = kunit_add_action_or_reset(test, cd321x_tbt_test_unregister_switch,
					sw_dev);
	KUNIT_ASSERT_EQ(test, ret, 0);
}

/* A router that registers after the port gave up is taken; the cable is reread. */
static void cd321x_tbt_router_binds_late_test(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	int ret;

	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, cd->tbt_watch_registered);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 0);

	cd321x_tbt_test_add_router(test, &cd321x_test_router);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_TRUE(test, cd->tbt_notifier_registered);
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 1);

	cd321x_tbt_test_release_port(cd);
	KUNIT_EXPECT_FALSE(test, cd->tbt_watch_registered);
	KUNIT_EXPECT_FALSE(test, cd->tbt_notifier_registered);
}

/* A router that registered before the watch started is not missed. */
static void cd321x_tbt_router_bound_before_watch_test(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	int ret;

	cd321x_tbt_test_add_router(test, &cd321x_test_router);
	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_NOT_ERR_OR_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 1);
}

/* Another port's router binding leaves this port waiting for its own. */
static void cd321x_tbt_other_router_binds_test(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	int ret;

	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	cd321x_tbt_test_add_router(test, &cd321x_test_other_router);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 0);
	KUNIT_EXPECT_TRUE(test, cd->tbt_watch_registered);
}

/* A port that is going away no longer takes a router that binds. */
static void cd321x_tbt_router_binds_after_removal_test(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	int ret;

	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	cd321x_stop_work(cd);
	cd321x_tbt_test_add_router(test, &cd321x_test_router);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_FALSE(test, cd->tbt_notifier_registered);
}

/* A router that unbinds and binds again is taken again; the cable is reread. */
static void cd321x_tbt_router_rebinds_test(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = software_node_fwnode(&cd321x_test_router),
		.set = cd321x_tbt_test_set,
		.name = "router-first",
	};
	struct typec_thunderbolt_switch_dev *sw_dev;
	struct device *router;
	int ret;

	router = kunit_device_register(test, "router-first");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, router);
	sw_dev = typec_thunderbolt_switch_register(router, &desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sw_dev);

	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	flush_work(&cd->tbt_attach_work);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cd->tbt_switch);
	KUNIT_EXPECT_TRUE(test, typec_thunderbolt_switch_is_live(cd->tbt_switch));
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 1);
	atomic_set(&cd->restore_event, 0);

	typec_thunderbolt_switch_unregister(sw_dev);
	KUNIT_EXPECT_FALSE(test, typec_thunderbolt_switch_is_live(cd->tbt_switch));

	cd321x_tbt_test_add_router(test, &cd321x_test_router);
	flush_work(&cd->tbt_attach_work);
	KUNIT_EXPECT_TRUE(test, typec_thunderbolt_switch_is_live(cd->tbt_switch));
	KUNIT_EXPECT_TRUE(test, cd->tbt_notifier_registered);
	KUNIT_EXPECT_EQ(test, atomic_read(&cd->restore_event), 1);

	cd321x_tbt_test_release_port(cd);
	KUNIT_EXPECT_FALSE(test, cd->tbt_notifier_registered);
}

/* A port with a live router, as the cable events find it. */
static struct cd321x *cd321x_tbt_test_live_port(struct kunit *test)
{
	struct cd321x *cd = cd321x_tbt_test_late_port(test);
	int ret;

	ret = cd321x_tbt_watch_late(cd, software_node_fwnode(&cd321x_test_connector));
	KUNIT_ASSERT_EQ(test, ret, 0);
	flush_work(&cd->tbt_attach_work);
	cd321x_tbt_test_add_router(test, &cd321x_test_router);
	flush_work(&cd->tbt_attach_work);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cd->tbt_switch);
	scoped_guard(mutex, &cd->tps.lock)
		cd->pm.phase = CD321X_PM_RUNNING;
	return cd;
}

/* A reboot closes the router's session at once and keeps cable events away. */
static void cd321x_tbt_reboot_closes_session_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct cd321x *cd = cd321x_tbt_test_live_port(test);

	KUNIT_EXPECT_EQ(test, cd321x_reboot_notify(&cd->reboot_nb, SYS_RESTART, NULL),
			NOTIFY_OK);
	KUNIT_EXPECT_EQ(test, ctx->set_calls, 1);
	KUNIT_EXPECT_EQ(test, ctx->last_state, TYPEC_THUNDERBOLT_SWITCH_OFF);
	KUNIT_EXPECT_FALSE(test, cd321x_pm_can_update(&cd->pm));
}

/* Only a reboot, halt or power off closes the session. */
static void cd321x_tbt_reboot_other_action_test(struct kunit *test)
{
	struct cd321x_tbt_test *ctx = test->priv;
	struct cd321x *cd = cd321x_tbt_test_live_port(test);

	KUNIT_EXPECT_EQ(test, cd321x_reboot_notify(&cd->reboot_nb, 0, NULL), NOTIFY_DONE);
	KUNIT_EXPECT_EQ(test, ctx->set_calls, 0);
	KUNIT_EXPECT_TRUE(test, cd321x_pm_can_update(&cd->pm));
}

static struct kunit_case cd321x_tbt_cases[] = {
	KUNIT_CASE(cd321x_tbt_no_router_test),
	KUNIT_CASE(cd321x_tbt_router_bound_test),
	KUNIT_CASE(cd321x_tbt_router_pending_test),
	KUNIT_CASE(cd321x_tbt_router_never_binds_test),
	KUNIT_CASE(cd321x_tbt_router_binds_late_test),
	KUNIT_CASE(cd321x_tbt_router_bound_before_watch_test),
	KUNIT_CASE(cd321x_tbt_other_router_binds_test),
	KUNIT_CASE(cd321x_tbt_router_binds_after_removal_test),
	KUNIT_CASE(cd321x_tbt_router_rebinds_test),
	KUNIT_CASE(cd321x_tbt_reboot_closes_session_test),
	KUNIT_CASE(cd321x_tbt_reboot_other_action_test),
	{}
};

static struct kunit_suite cd321x_tbt_suite = {
	.name = "cd321x-tbt",
	.init = cd321x_tbt_test_init,
	.test_cases = cd321x_tbt_cases,
};

kunit_test_suite(cd321x_tbt_suite);
MODULE_DESCRIPTION("KUnit tests for the CD321x USB4/Thunderbolt router lookup");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
