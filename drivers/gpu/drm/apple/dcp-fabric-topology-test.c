// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Included by dcp-fabric.c to exercise its connector publication paths. */
#include <kunit/device.h>
#include <kunit/of.h>
#include <kunit/platform_device.h>
#include <kunit/test.h>

OF_OVERLAY_DECLARE(dcp_route_supplier_test);

struct dcp_topology_test {
	struct apple_dcp_typec_port port[3];
	struct apple_dcp_typec_route route[3][2];
	struct apple_connector connector[3][2];
	struct drm_encoder encoder[3][2];
	struct apple_dcp dcp[2];
	struct apple_crtc crtc[2];
	struct platform_device *pdev[2];
	struct drm_device drm;
};

static bool topology_fixed_crtcs(void)
{
	return true;
}

static void topology_noop(struct work_struct *work)
{
}

static void topology_cleanup(void *data)
{
	struct dcp_topology_test *t = data;
	unsigned int p, i;

	for (i = 0; i < 2; i++)
		cancel_work_sync(&t->dcp[i].tb_event_work);
	for (p = 0; p < 3; p++)
		for (i = 0; i < 2; i++)
			cancel_work_sync(&t->connector[p][i].bl_sync_wq);
}

static struct dcp_topology_test *topology_init(struct kunit *test)
{
	struct dcp_topology_test *t;
	unsigned int p, i;

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, t);
	kunit_activate_static_stub(test, dcp_typec_fixed_crtcs, topology_fixed_crtcs);
	for (i = 0; i < 2; i++) {
		struct apple_dcp *dcp = &t->dcp[i];

		t->pdev[i] = kunit_platform_device_alloc(test, "dcp-topology",
							 PLATFORM_DEVID_AUTO);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->pdev[i]);
		KUNIT_ASSERT_EQ(test, kunit_platform_device_add(test, t->pdev[i]), 0);
		platform_set_drvdata(t->pdev[i], dcp);
		dcp->dev = &t->pdev[i]->dev;
		dcp->crtc = &t->crtc[i];
		t->crtc[i].base.index = 1 + 2 * i;
		t->crtc[i].base.dev = &t->drm;
		dcp->fixed_connector_type = DRM_MODE_CONNECTOR_USB;
		mutex_init(&dcp->tb_event_lock);
		mutex_init(&dcp->tb_lock);
		mutex_init(&dcp->external_lock);
		mutex_init(&dcp->modes_lock);
		spin_lock_init(&dcp->metadata.lock);
		INIT_WORK(&dcp->tb_event_work, topology_noop);
	}
	for (p = 0; p < 3; p++) {
		INIT_LIST_HEAD(&t->port[p].routes);
		for (i = 0; i < 2; i++) {
			struct apple_connector *connector = &t->connector[p][i];
			struct apple_dcp_typec_route *route = &t->route[p][i];

			route->dcp = &t->dcp[i];
			route->port = &t->port[p];
			list_add_tail(&route->port_link, &t->port[p].routes);
			connector->base.dev = &t->drm;
			connector->port_encoder = &t->encoder[p][i];
			connector->candidate_crtcs = drm_crtc_mask(&t->crtc[i].base);
			t->encoder[p][i].possible_crtcs = connector->candidate_crtcs;
			apple_connector_edid_init(connector);
			INIT_WORK(&connector->bl_sync_wq, topology_noop);
		}
	}
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, topology_cleanup, t), 0);
	return t;
}

/* The DP IN ordinal must never choose the connector on fixed-CRTC ports. */
static void topology_pipeline_identity_test(struct kunit *test)
{
	struct dcp_topology_test *t = topology_init(test);
	unsigned int p, i, dpin;

	guard(mutex)(&dcp_typec_fabric_lock);
	for (p = 0; p < 3; p++) {
		for (i = 0; i < 2; i++) {
			struct apple_dcp_typec_route *route = &t->route[p][i];
			struct apple_connector *connector = &t->connector[p][i];

			dcp_typec_port_bind_connector(&t->port[p], false, connector, t->pdev[i]);
			KUNIT_EXPECT_PTR_EQ(test, connector->dcp, NULL);
			KUNIT_EXPECT_FALSE(test, dcp_typec_narrows(&t->dcp[i]));
			for (dpin = 0; dpin < DCP_FABRIC_DPIN_MAX; dpin++) {
				route->tunnel_dpin = dpin;
				KUNIT_EXPECT_PTR_EQ(test,
						    dcp_typec_route_connector(route, true),
						    connector);
				KUNIT_EXPECT_PTR_EQ(test,
						    dcp_typec_route_connector(route, false),
						    connector);
			}
		}
	}
}

/* Boot adoption and replug can put either tunnel slot on either processor. */
static void topology_adopt_release_test(struct kunit *test)
{
	struct dcp_topology_test *t = topology_init(test);
	unsigned int p, i;

	guard(mutex)(&dcp_typec_fabric_lock);
	for (p = 0; p < 3; p++) {
		for (i = 0; i < 2; i++) {
			struct apple_dcp_typec_route *route = &t->route[p][i];
			struct apple_connector *connector = &t->connector[p][i];
			u32 mask = connector->port_encoder->possible_crtcs;

			/* Opposite of the connector creation order, as after replug. */
			t->port[p].owner = &t->route[p][1];
			t->port[p].secondary_owner = &t->route[p][0];
			t->dcp[i].active_typec_route = route;
			dcp_typec_port_bind_connector(&t->port[p], false, connector, t->pdev[i]);
			KUNIT_EXPECT_PTR_EQ(test, t->dcp[i].typec_connector, connector);
			KUNIT_EXPECT_PTR_EQ(test, connector->dcp, t->pdev[i]);
			KUNIT_EXPECT_EQ(test, connector->port_encoder->possible_crtcs, mask);
			connector->connected = true;
			/* The hardware has acknowledged release before this retry. */
			route->deactivate_xbar_done = true;
			KUNIT_ASSERT_EQ(test, dcp_typec_route_deactivate(route), 0);
			KUNIT_EXPECT_FALSE(test, connector->connected);
			KUNIT_EXPECT_PTR_EQ(test, connector->dcp, NULL);
			KUNIT_EXPECT_PTR_EQ(test, t->dcp[i].typec_connector, NULL);
			KUNIT_EXPECT_EQ(test, connector->port_encoder->possible_crtcs, mask);
			KUNIT_EXPECT_PTR_EQ(test, route->connector, connector);
		}
		t->port[p].owner = NULL;
		t->port[p].secondary_owner = NULL;
	}
}

static void topology_unbind_test(struct kunit *test)
{
	struct dcp_topology_test *t = topology_init(test);
	unsigned int p, i;

	guard(mutex)(&dcp_typec_fabric_lock);
	for (p = 0; p < 3; p++) {
		for (i = 0; i < 2; i++)
			dcp_typec_port_bind_connector(&t->port[p], false,
						      &t->connector[p][i], t->pdev[i]);
		dcp_typec_port_unlink(&t->port[p], &t->drm);
		for (i = 0; i < 2; i++)
			KUNIT_EXPECT_PTR_EQ(test, t->route[p][i].connector, NULL);
	}
}

/* The J714s's display fabric: three ports, three pipelines, the second tunnel-only */
enum { TOPO_RIGHT, TOPO_LEFT_FRONT, TOPO_LEFT_BACK, TOPO_PORTS };
enum { TOPO_DCPEXT0, TOPO_DCPEXT1, TOPO_DCPEXT2, TOPO_PIPES };
#define TOPO_PAIR(port, pipe)	((port) * TOPO_PIPES + (pipe))

struct topology_j714s {
	struct apple_dcp_typec_port port[TOPO_PORTS];
	struct apple_dcp_typec_route route[TOPO_PORTS][TOPO_PIPES];
	struct apple_dcp dcp[TOPO_PIPES];
	struct platform_device *pdev[TOPO_PIPES];
	unsigned int first;
	/* The passes the ports already registered (a real J714s's) take */
	unsigned int passes;
};

static void topology_j714s_unlist(void *data)
{
	struct topology_j714s *t = data;
	unsigned int p;

	guard(mutex)(&dcp_typec_fabric_lock);
	for (p = 0; p < TOPO_PORTS; p++)
		list_del(&t->port[p].link);
}

static struct topology_j714s *topology_j714s_init(struct kunit *test)
{
	struct topology_j714s *t;
	unsigned int p, i;

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, t);
	kunit_activate_static_stub(test, dcp_typec_fixed_crtcs, topology_fixed_crtcs);
	for (i = 0; i < TOPO_PIPES; i++) {
		t->pdev[i] = kunit_platform_device_alloc(test, "dcp-j714s", PLATFORM_DEVID_AUTO);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->pdev[i]);
		t->dcp[i].dev = &t->pdev[i]->dev;
	}
	t->first = dcp_typec_nr_ports();
	t->passes = dcp_typec_nr_passes();
	scoped_guard(mutex, &dcp_typec_fabric_lock) {
		for (p = 0; p < TOPO_PORTS; p++) {
			INIT_LIST_HEAD(&t->port[p].routes);
			for (i = 0; i < TOPO_PIPES; i++) {
				struct apple_dcp_typec_route *route = &t->route[p][i];

				route->dcp = &t->dcp[i];
				route->port = &t->port[p];
				route->core.tunnel_only = i == TOPO_DCPEXT1;
				list_add_tail(&route->port_link, &t->port[p].routes);
			}
			list_add_tail(&t->port[p].link, &dcp_typec_ports);
		}
	}
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, topology_j714s_unlist, t), 0);
	return t;
}

/*
 * Lists the connectors of the first @nr_ports ports in
 * apple_probe_typec_ports()'s order, as TOPO_PAIR(port, pipeline), and
 * returns how many. It mirrors that loop: passes outermost, then pipelines
 * (dcp[idx / nr_ports]), then ports (idx % nr_ports).
 */
static unsigned int topology_connector_order(struct topology_j714s *t, unsigned int nr_ports,
					     unsigned int *order)
{
	unsigned int pass, i, p, n = 0;

	for (pass = 0; pass < dcp_typec_nr_passes(); pass++)
		for (i = 0; i < TOPO_PIPES; i++)
			for (p = 0; p < nr_ports; p++)
				if (dcp_typec_port_has_candidate(t->first + p, t->pdev[i]) &&
				    dcp_typec_port_pass(t->first + p, t->pdev[i]) == pass)
					order[n++] = TOPO_PAIR(p, i);
	return n;
}

/* The passes with @passes needed by the fixture's own routes */
static unsigned int topology_passes(struct topology_j714s *t, unsigned int passes)
{
	return max(t->passes, passes);
}

/*
 * The J714s's connector names: DP-1..DP-6 for the right and left-front ports
 * as before the left-back port was added, whose routes are in connector group
 * 1 and get DP-7..DP-9. In group 0 they would take DP-3 and push the rest on.
 */
static void topology_connector_group_test(struct kunit *test)
{
	static const unsigned int two_ports[] = {
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT0),		/* DP-1 */
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT0),	/* DP-2 */
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT2),		/* DP-3 */
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT2),	/* DP-4 */
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT1),		/* DP-5 */
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT1),	/* DP-6 */
	};
	static const unsigned int three_ports[] = {
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT0),
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT0),
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT2),
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT2),
		TOPO_PAIR(TOPO_RIGHT, TOPO_DCPEXT1),
		TOPO_PAIR(TOPO_LEFT_FRONT, TOPO_DCPEXT1),
		TOPO_PAIR(TOPO_LEFT_BACK, TOPO_DCPEXT0),	/* DP-7 */
		TOPO_PAIR(TOPO_LEFT_BACK, TOPO_DCPEXT2),	/* DP-8 */
		TOPO_PAIR(TOPO_LEFT_BACK, TOPO_DCPEXT1),	/* DP-9 */
	};
	struct topology_j714s *t = topology_j714s_init(test);
	unsigned int order[TOPO_PORTS * TOPO_PIPES], i, n;

	/* Everything in group 0: two passes */
	KUNIT_EXPECT_EQ(test, dcp_typec_nr_passes(), topology_passes(t, 2));
	n = topology_connector_order(t, 2, order);
	KUNIT_ASSERT_EQ(test, n, ARRAY_SIZE(two_ports));
	KUNIT_EXPECT_MEMEQ(test, order, two_ports, sizeof(two_ports));

	/* The left-back port in group 0 too: its first pair would be DP-3 */
	n = topology_connector_order(t, 3, order);
	KUNIT_ASSERT_EQ(test, n, ARRAY_SIZE(three_ports));
	KUNIT_EXPECT_EQ(test, order[2], TOPO_PAIR(TOPO_LEFT_BACK, TOPO_DCPEXT0));

	/* In group 1, as the J714s DT has it */
	for (i = 0; i < TOPO_PIPES; i++)
		t->route[TOPO_LEFT_BACK][i].connector_group = 1;
	KUNIT_EXPECT_EQ(test, dcp_typec_nr_passes(), topology_passes(t, 4));
	KUNIT_EXPECT_EQ(test, dcp_typec_port_pass(t->first + TOPO_LEFT_BACK,
						  t->pdev[TOPO_DCPEXT0]), 2);
	KUNIT_EXPECT_EQ(test, dcp_typec_port_pass(t->first + TOPO_LEFT_BACK,
						  t->pdev[TOPO_DCPEXT1]), 3);
	/* A port's shared connector (no fixed CRTCs) takes its group's first pass */
	KUNIT_EXPECT_EQ(test, dcp_typec_port_pass(t->first + TOPO_LEFT_BACK, NULL), 2);
	KUNIT_EXPECT_EQ(test, dcp_typec_port_pass(t->first + TOPO_RIGHT, NULL), 0);
	n = topology_connector_order(t, 3, order);
	KUNIT_ASSERT_EQ(test, n, ARRAY_SIZE(three_ports));
	KUNIT_EXPECT_MEMEQ(test, order, three_ports, sizeof(three_ports));

	/* A later group adds passes; the order is the same */
	for (i = 0; i < TOPO_PIPES; i++)
		t->route[TOPO_LEFT_BACK][i].connector_group = 3;
	KUNIT_EXPECT_EQ(test, dcp_typec_nr_passes(), topology_passes(t, 8));
	n = topology_connector_order(t, 3, order);
	KUNIT_ASSERT_EQ(test, n, ARRAY_SIZE(three_ports));
	KUNIT_EXPECT_MEMEQ(test, order, three_ports, sizeof(three_ports));
}

static void topology_route_supplier_put(void *data)
{
	struct device *dev = data;

	of_node_put(dev->of_node);
	dev->of_node = NULL;
}

/*
 * Only a route whose PHY or crossbar node is disabled is skipped. A missing
 * or malformed entry counts as available, so that its lookup fails the probe.
 */
static void topology_route_supplier_test(struct kunit *test)
{
	struct device *dev;

	KUNIT_ASSERT_EQ(test, 0, of_overlay_apply_kunit(test, dcp_route_supplier_test));
	dev = kunit_device_register(test, "dcp-route-supplier");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dev->of_node = of_find_node_by_path("/dcp-route-supplier-test/dcp");
	KUNIT_ASSERT_NOT_NULL(test, dev->of_node);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, topology_route_supplier_put, dev), 0);

	KUNIT_EXPECT_TRUE(test, dcp_typec_route_supplier_available(dev, "phys", "#phy-cells",
								   "phy-names", "typec0"));
	KUNIT_EXPECT_FALSE(test, dcp_typec_route_supplier_available(dev, "phys", "#phy-cells",
								    "phy-names", "typec1"));
	/* No such name, and no crossbar entry for typec2: the lookups report those */
	KUNIT_EXPECT_TRUE(test, dcp_typec_route_supplier_available(dev, "phys", "#phy-cells",
								   "phy-names", "typec4"));
	KUNIT_EXPECT_TRUE(test, dcp_typec_route_supplier_available(dev, "mux-controls",
								   "#mux-control-cells",
								   "mux-control-names", "typec2"));
	KUNIT_EXPECT_TRUE(test, dcp_typec_route_supplier_available(dev, "mux-controls",
								   "#mux-control-cells",
								   "mux-control-names", "typec1"));

	/*
	 * The per-route decision a failsafe dcpext's supplier preflight
	 * (dcp_ext_route_suppliers()) and route registration share
	 */
	KUNIT_EXPECT_NULL(test, dcp_typec_route_supplier_disabled(dev, "typec0"));
	KUNIT_EXPECT_STREQ(test, dcp_typec_route_supplier_disabled(dev, "typec1") ?: "",
			   "DP PHY");
	KUNIT_EXPECT_NULL(test, dcp_typec_route_supplier_disabled(dev, "typec2"));
	KUNIT_EXPECT_STREQ(test, dcp_typec_route_supplier_disabled(dev, "typec3") ?: "",
			   "display crossbar");

	/* Available suppliers: the route is not skipped, and nothing is changed */
	KUNIT_EXPECT_NULL(test, dcp_typec_route_skip(dev, dev->of_node, "typec0"));
	KUNIT_EXPECT_NULL(test, dcp_typec_route_skip(dev, dev->of_node, "typec2"));
	KUNIT_EXPECT_TRUE(test, of_device_is_available(dev->of_node));
}

static void topology_route_node_put(void *data)
{
	of_node_put(data);
}

/*
 * A route whose PHY is disabled is skipped and disabled itself, so its port
 * stops waiting for it. The changeset is reverted, for the overlay's removal.
 */
static void topology_route_skip_test(struct kunit *test)
{
	struct device_node *route_np;
	struct of_changeset *cs;
	struct device *dev;

	KUNIT_ASSERT_EQ(test, 0, of_overlay_apply_kunit(test, dcp_route_supplier_test));
	dev = kunit_device_register(test, "dcp-route-skip");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dev->of_node = of_find_node_by_path("/dcp-route-supplier-test/dcp");
	KUNIT_ASSERT_NOT_NULL(test, dev->of_node);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, topology_route_supplier_put, dev), 0);
	route_np = of_find_node_by_path("/dcp-route-supplier-test/dcp/typec-routes/route@1");
	KUNIT_ASSERT_NOT_NULL(test, route_np);
	KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, topology_route_node_put, route_np), 0);
	KUNIT_ASSERT_TRUE(test, of_device_is_available(route_np));

	cs = dcp_typec_route_skip(dev, route_np, "typec1");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cs);
	KUNIT_EXPECT_FALSE(test, of_device_is_available(route_np));

	/* A changeset still applied must not be destroyed */
	KUNIT_ASSERT_EQ(test, of_changeset_revert(cs), 0);
	of_changeset_destroy(cs);
	kfree(cs);
	KUNIT_EXPECT_TRUE(test, of_device_is_available(route_np));
}

static struct kunit_case dcp_topology_cases[] = {
	KUNIT_CASE(topology_pipeline_identity_test),
	KUNIT_CASE(topology_adopt_release_test),
	KUNIT_CASE(topology_unbind_test),
	KUNIT_CASE(topology_connector_group_test),
	KUNIT_CASE(topology_route_supplier_test),
	KUNIT_CASE(topology_route_skip_test),
	{}
};

static struct kunit_suite dcp_topology_suite = {
	.name = "apple-dcp-topology",
	.test_cases = dcp_topology_cases,
};

kunit_test_suite(dcp_topology_suite);
