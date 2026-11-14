// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/limits.h>
#include <linux/soc/apple/dp-tunnel.h>

#include "dcp-fabric-core.h"
#include "dcp-fabric-effects.h"
#include "dcp-fabric-session.h"
#include "dcp-relink.h"

struct fabric_fixture {
	struct dcp_fabric_pipeline pipeline[2];
	struct dcp_fabric_route route[3][2];
	struct dcp_fabric_port port[3];
	struct dcp_fabric_plan plan[3];
	struct dcp_fabric_policy policy;
};

static void fabric_init(struct fabric_fixture *f, bool dual)
{
	unsigned int p, r;

	memset(f, 0, sizeof(*f));
	f->policy.dual_stream = dual;
	f->pipeline[0].next = &f->pipeline[1];
	for (r = 0; r < 2; r++) {
		f->pipeline[r].bound = true;
		f->pipeline[r].crtc_index = r + 1;
		f->pipeline[r].services_ready = true;
	}
	f->pipeline[0].has_fixed = true;
	for (p = 0; p < 3; p++) {
		f->port[p].key = p + 1;
		f->port[p].routes = &f->route[p][0];
		f->port[p].plan = &f->plan[p];
		f->port[p].candidate_crtcs[0] = BIT(1) | BIT(2);
		f->port[p].candidate_crtcs[1] = BIT(2);
		if (p < 2)
			f->port[p].next = &f->port[p + 1];
		for (r = 0; r < 2; r++) {
			f->route[p][r].pipeline = &f->pipeline[r];
			if (!r)
				f->route[p][r].next = &f->route[p][r + 1];
		}
	}
}

/* Fake effects only commit ownership after the real decision succeeds. */
static int fabric_direct(struct fabric_fixture *f, unsigned int p)
{
	struct dcp_fabric_route *route;

	f->port[p].wanted = true;
	f->port[p].hpd = true;
	route = dcp_fabric_free_route(&f->port[p], &f->policy);
	if (!route)
		return -EBUSY;
	route->pipeline->owned = true;
	f->port[p].owner[0] = route;
	return 0;
}

static int fabric_tunnel(struct fabric_fixture *f, unsigned int p,
			 unsigned int dpin)
{
	struct dcp_fabric_route *route;
	int error;

	error = dcp_fabric_tunnel_slot(&f->port[p], dpin);
	if (error)
		return error > 0 ? 0 : error;
	route = dcp_fabric_tunnel_candidate(&f->port[p], &f->policy, NULL,
					    false, dpin, true, &error);
	if (!route)
		return error;
	route->pipeline->owned = true;
	route->pipeline->tunnel_held = true;
	route->tunnel = true;
	route->dpin = dpin;
	f->port[p].owner[dpin] = route;
	return 0;
}

static void fabric_release(struct fabric_fixture *f, unsigned int p,
			   unsigned int dpin)
{
	struct dcp_fabric_route *route = f->port[p].owner[dpin];

	if (!route)
		return;
	route->pipeline->owned = false;
	route->pipeline->tunnel_held = false;
	route->tunnel = false;
	f->port[p].preferred = route;
	f->port[p].owner[dpin] = NULL;
	f->port[p].wanted = false;
	f->port[p].hpd = false;
}

static void fabric_promote(struct fabric_fixture *f)
{
	unsigned int p;

	if (dcp_fabric_capacity_action(false) != DCP_FABRIC_PROMOTE)
		return;
	for (p = 0; p < 3; p++)
		if (dcp_fabric_waiting(&f->port[p]))
			fabric_direct(f, p);
}

/* Both borrowers consume the same hardirq/sample/expiry decisions. */
static void fabric_presence_scenario(struct kunit *test,
				     struct fabric_fixture *f, unsigned int id)
{
	struct dcp_fabric_presence presence = {};
	const unsigned long window = 10000;
	u64 edge, newer;
	unsigned int events = 0;

	f->pipeline[1].owned = true;
	edge = dcp_fabric_presence_edge(&presence, 100, window);
	KUNIT_EXPECT_EQ(test, presence.deadline, 10100UL);
	f->pipeline[0].presence = presence.state;
	KUNIT_EXPECT_EQ(test, fabric_tunnel(f, 0, 0), -EBUSY);
	KUNIT_EXPECT_EQ(test, fabric_direct(f, 1), -EBUSY);
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_expire(&presence, edge, false, 10100 - 1));

	if (id == 10) {
		/* A waking monitor returns before the window ends. */
		newer = dcp_fabric_presence_edge(&presence, 1100, window);
		KUNIT_ASSERT_TRUE(test,
				  dcp_fabric_presence_sample(&presence, newer, true, 1100, window));
		KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_PRESENT);
		KUNIT_EXPECT_EQ(test, presence.deadline, 0UL);
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_presence_expire(&presence, edge, false, 10100));
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_presence_expire(&presence, newer, false, 11100));
		f->pipeline[0].presence = presence.state;
		f->pipeline[0].fixed_busy = true;
		KUNIT_EXPECT_EQ(test, fabric_tunnel(f, 0, 0), -EBUSY);
		return;
	}
	if (id == 16) {
		enum dcp_fabric_resume_step steps[2];
		bool irq_enabled = false, sampled = false;
		unsigned int i;

		/* Fake IRQ/GPIO effects consume the production ordering decision. */
		dcp_fabric_resume_steps(steps);
		for (i = 0; i < ARRAY_SIZE(steps); i++) {
			switch (steps[i]) {
			case DCP_FABRIC_ENABLE_HPD_IRQ:
				irq_enabled = true;
				break;
			case DCP_FABRIC_SAMPLE_HPD:
				KUNIT_EXPECT_TRUE(test, irq_enabled);
				sampled = true;
				break;
			}
		}
		KUNIT_EXPECT_TRUE(test, sampled);
		/* Resume starts after IRQ enable: low gets the full window. */
		newer = dcp_fabric_presence_edge(&presence, 20100, window);
		KUNIT_ASSERT_TRUE(test,
				  dcp_fabric_presence_sample(&presence, newer, false,
							     20200, window));
		KUNIT_EXPECT_EQ(test, presence.deadline, 30200UL);
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_presence_expire(&presence, edge, false, 30200));
		/* A high sample clears the edge hold and invalidates its expiry. */
		KUNIT_ASSERT_TRUE(test,
				  dcp_fabric_presence_sample(&presence, newer, true,
							     20300, window));
		KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_PRESENT);
		KUNIT_EXPECT_EQ(test, presence.deadline, 0UL);
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_presence_expire(&presence, newer, false, 30200));
		/* A falling edge after the sample wins; stale high cannot undo it. */
		edge = dcp_fabric_presence_edge(&presence, 20400, window);
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_presence_sample(&presence, newer, true,
							      20500, window));
		KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_SETTLING);
		KUNIT_EXPECT_EQ(test, presence.deadline, 30400UL);
		f->pipeline[0].presence = presence.state;
		KUNIT_EXPECT_EQ(test, fabric_tunnel(f, 0, 0), -EBUSY);
		KUNIT_EXPECT_EQ(test, fabric_direct(f, 1), -EBUSY);
		return;
	}

	KUNIT_ASSERT_TRUE(test,
			  dcp_fabric_presence_expire(&presence, edge, false, 10100));
	events++;
	f->pipeline[0].presence = presence.state;
	KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_ABSENT);
	/* The event is consumed once, even if the expiry callback is repeated. */
	if (dcp_fabric_presence_expire(&presence, edge, false, 10101))
		events++;
	KUNIT_EXPECT_EQ(test, events, 1U);
	if (id == 11) {
		KUNIT_EXPECT_EQ(test, fabric_tunnel(f, 0, 0), 0);
		KUNIT_EXPECT_PTR_EQ(test, f->port[0].owner[0], &f->route[0][0]);
	} else {
		fabric_promote(f);
		KUNIT_EXPECT_PTR_EQ(test, f->port[1].owner[0], &f->route[1][0]);
	}
}

struct fabric_scenario {
	const char *name;
	unsigned int id;
};

static const struct fabric_scenario scenarios[] = {
	{ "S1_lone_direct", 1 },
	{ "S2_direct_then_hdmi", 2 },
	{ "S3_lone_tunnel", 3 },
	{ "S4_both_typec_orders", 4 },
	{ "S5_full_hdmi_direct_tunnel", 5 },
	{ "S6_direct_release_then_tunnel", 6 },
	{ "S7_tunnel_release_promotes_direct", 7 },
	{ "S8_frozen_hdmi_waits_core", 8 },
	{ "S9_parked_fixed_reselect", 9 },
	{ "S10_hdmi_blink", 10 },
	{ "S11_hdmi_absence", 11 },
	{ "S12_direct_settling", 12 },
	{ "S13_boot_three_outputs", 13 },
	{ "S14_boot_two_typec", 14 },
	{ "S15_boot_hdmi_tunnel", 15 },
	{ "S16_resume_arms_settle_core", 16 },
	{ "S17_occupied_slot_errors", 17 },
	{ "S18_connector_mask", 18 },
	{ "S20_base_capacity", 20 },
	{ "S21_base_lone", 21 },
	{ "S30_dual_pairing_plan", 30 },
	{ "S31_dual_dpin_candidates", 31 },
	{ "S32_dual_hdmi_reclaim_core", 32 },
	{ "S33_dual_scores", 33 },
	{ "S34_dual_park_reselect", 34 },
	{ "S35_dual_retry_off", 35 },
	{ "S40_desktop_promotion", 40 },
	{ "S41_dedicated_hdmi", 41 },
	{ "S50a_panel_has_no_routes_core", 50 },
	{ "S50b_external_service_readiness_core", 51 },
	{ "S50c_terminal_scanout_core", 52 },
	{ "S50d_external_attach_dispatch_core", 53 },
	{ "S50e_t6030_crossbar_core", 54 },
	{ "S50f_unwired_m3_ports_core", 55 },
};

static void fabric_scenario_desc(const struct fabric_scenario *scenario,
				 char *desc)
{
	strscpy(desc, scenario->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(fabric_scenario, scenarios, fabric_scenario_desc);

static void fabric_scenario_test(struct kunit *test)
{
	const struct fabric_scenario *scenario = test->param_value;
	struct fabric_fixture f;
	const struct dcp_fabric_port *found;
	enum dcp_fabric_fixed_step steps[2];
	unsigned int count, order;

	fabric_init(&f, scenario->id >= 30 && scenario->id <= 35);
	switch (scenario->id) {
	case 1:
		f.port[0].preferred = &f.route[0][0];
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
		break;
	case 2:
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		f.pipeline[0].fixed_busy = true;
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
		KUNIT_EXPECT_FALSE(test, dcp_fabric_available(&f.pipeline[0], &f.policy));
		break;
	case 3:
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
		break;
	case 4:
	case 14:
		for (order = 0; order < 2; order++) {
			fabric_init(&f, false);
			KUNIT_ASSERT_EQ(test,
					order ? fabric_direct(&f, 0) :
					fabric_tunnel(&f, 0, 0),
					0);
			KUNIT_ASSERT_EQ(test,
					order ? fabric_tunnel(&f, 1, 0) :
					fabric_direct(&f, 1),
					0);
			KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0],
					    &f.route[0][1]);
			KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0],
					    &f.route[1][0]);
		}
		break;
	case 5:
	case 6:
	case 13:
		f.pipeline[0].fixed_busy = true;
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 1, 0), -EBUSY);
		KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], NULL);
		if (scenario->id == 6) {
			fabric_release(&f, 0, 0);
			KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 1, 0), 0);
			KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0],
					    &f.route[1][1]);
		}
		break;
	case 7:
	case 20:
	case 40:
		if (scenario->id != 7) {
			for (order = 0; order < 3; order++)
				f.port[order].routes = &f.route[order][1];
		}
		f.pipeline[0].fixed_busy = true;
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		KUNIT_EXPECT_EQ(test, fabric_direct(&f, 1), -EBUSY);
		fabric_release(&f, 0, 0);
		fabric_promote(&f);
		KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], &f.route[1][1]);
		break;
	case 8:
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 1), 0);
		fabric_release(&f, 0, 0);
		f.pipeline[0].fixed_busy = true;
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_keep_order(false, true, true));
		KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], &f.route[1][0]);
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_fixed_steps(true, true, true, steps),
				0U);
		fabric_release(&f, 1, 0);
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_fixed_steps(true, false, true, steps),
				2U);
		break;
	case 9:
	case 34:
		count = dcp_fabric_fixed_steps(true, false, true, steps);
		KUNIT_ASSERT_EQ(test, count, 2U);
		/* Fake PHY/mux trace consumes the production decision in order. */
		KUNIT_EXPECT_EQ(test, steps[0], DCP_FABRIC_RESTORE_PHY);
		KUNIT_EXPECT_EQ(test, steps[1], DCP_FABRIC_SELECT_MUX);
		break;
	case 10:
	case 11:
	case 12:
	case 16:
		fabric_presence_scenario(test, &f, scenario->id);
		break;
	case 15:
		f.pipeline[0].fixed_busy = true;
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
		break;
	case 17:
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		f.route[0][1].dpin = 1;
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_slot(&f.port[0], 0),
				-EADDRINUSE);
		fabric_init(&f, false);
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_slot(&f.port[0], 1),
				-EBUSY);
		/* Preserve the original occupied-primary errno in this extraction. */
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_slot(&f.port[0], 0),
				-EADDRINUSE);
		break;
	case 18:
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_connector_mask(false, true, true, 2,
							  BIT(1) | BIT(2)),
				(u32)BIT(2));
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_connector_mask(false, true, false, 2,
							  BIT(1) | BIT(2)),
				(u32)(BIT(1) | BIT(2)));
		break;
	case 21:
	case 41:
		/* A dedicated fixed pipeline has no edge in this port topology. */
		f.port[0].routes = &f.route[0][1];
		KUNIT_ASSERT_EQ(test, fabric_direct(&f, 0), 0);
		KUNIT_EXPECT_FALSE(test, f.pipeline[0].owned);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
		break;
	case 30:
		f.port[0].wanted = true;
		f.port[0].hpd = true;
		f.port[1].wanted = true;
		f.port[1].hpd = true;
		dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
		KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][0]);
		KUNIT_EXPECT_PTR_EQ(test, f.plan[1].target[0], &f.route[1][1]);
		KUNIT_EXPECT_TRUE(test,
				  dcp_fabric_keep_order(true, true, false));
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_keep_order(true, true, true));
		break;
	case 31:
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][0]);
		KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 1), 0);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[1], &f.route[0][1]);
		break;
	case 32:
		f.port[0].wanted = true;
		f.port[0].hpd = true;
		f.pipeline[0].fixed_busy = true;
		dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
		KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][1]);
		KUNIT_EXPECT_FALSE(test,
				   dcp_fabric_keep_order(true, true, true));
		break;
	case 33:
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_score(&f.pipeline[0], &f.policy), 1U);
		KUNIT_EXPECT_EQ(test,
				dcp_fabric_score(&f.pipeline[1], &f.policy), 2U);
		f.port[0].preferred = &f.route[0][1];
		KUNIT_EXPECT_PTR_EQ(test,
				    dcp_fabric_free_route(&f.port[0], &f.policy),
				    &f.route[0][1]);
		f.pipeline[0].presence = DCP_FABRIC_SETTLING;
		KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 1, 0), 0);
		break;
	case 35:
		kunit_skip(test, "TB match-data retry policy is tested by W3");
		break;
	case 50:
		f.port[0].routes = NULL;
		KUNIT_EXPECT_PTR_EQ(test,
				    dcp_fabric_free_route(&f.port[0], &f.policy),
				    NULL);
		KUNIT_EXPECT_FALSE(test, f.pipeline[0].owned);
		break;
	case 51:
	case 52:
		f.port[0].routes = &f.route[0][1];
		f.pipeline[1].external = true;
		f.pipeline[1].services_ready = false;
		f.pipeline[1].terminal = scenario->id == 52;
		KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0),
				scenario->id == 52 ? -ESHUTDOWN : -EAGAIN);
		KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], NULL);
		f.pipeline[1].services_ready = true;
		if (scenario->id == 51)
			KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
		break;
	case 53:
		KUNIT_EXPECT_EQ(test, dcp_fabric_attach_action(true),
				DCP_FABRIC_ATTACH_WORK);
		KUNIT_EXPECT_EQ(test, dcp_fabric_attach_action(false),
				DCP_FABRIC_ATTACH_OOB);
		break;
	case 54:
		KUNIT_EXPECT_TRUE(test, dcp_fabric_t6030_link(true, true, true));
		KUNIT_EXPECT_FALSE(test, dcp_fabric_t6030_link(true, true, false));
		KUNIT_EXPECT_FALSE(test, dcp_fabric_t6030_link(true, false, true));
		break;
	case 55:
		f.port[0].next = NULL;
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_request(f.port, 2, 0, &found), -ENODEV);
		KUNIT_EXPECT_PTR_EQ(test, found, NULL);
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_request(f.port, 3, 0, &found), -ENODEV);
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_request(f.port, 0, 0, &found), -EINVAL);
		KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_request(f.port, 1, 2, &found), -EINVAL);
		break;
	}
}

static void fabric_dark_tunnel_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_init(&f, true);
	/* A direct stream precedes an immutable tunnel on its planned pipeline. */
	f.port[0].wanted = true;
	f.port[0].hpd = true;
	f.port[1].owner[0] = &f.route[1][0];
	f.route[1][0].tunnel = true;
	f.pipeline[0].tunnel_held = true;
	f.pipeline[0].owned = true;
	dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
	KUNIT_EXPECT_TRUE(test, f.plan[0].dark);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], NULL);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[1].target[0], &f.route[1][0]);
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_movable(&f.route[1][0], &f.route[1][1]));
}

static void fabric_effect_failure_core_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_init(&f, true);
	f.port[0].wanted = true;
	f.port[0].hpd = true;
	f.port[1].wanted = true;
	f.port[1].hpd = true;
	dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][0]);
	/* Failed activation rolled back to a live fixed output: fresh sample. */
	f.pipeline[0].fixed_busy = true;
	dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][1]);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[1].target[0], NULL);
	KUNIT_EXPECT_FALSE(test, f.pipeline[0].owned);

	fabric_init(&f, false);
	f.port[0].wanted = true;
	f.port[0].hpd = true;
	f.port[1].wanted = true;
	f.port[1].hpd = true;
	/* Port zero's failed mux acquisition committed no ownership. */
	KUNIT_EXPECT_PTR_EQ(test, dcp_fabric_free_route(&f.port[0], &f.policy),
			    &f.route[0][1]);
	KUNIT_EXPECT_EQ(test, fabric_direct(&f, 1), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], &f.route[1][1]);
}

static void fabric_deactivate_failure_test(struct kunit *test)
{
	struct fabric_fixture f;
	enum dcp_fabric_deactivate_step steps[3];
	unsigned int count, i, connects = 0, replans = 0;
	bool selected = true;

	fabric_init(&f, true);
	f.port[0].wanted = true;
	f.port[0].hpd = true;
	f.port[0].owner[0] = &f.route[0][0];
	f.pipeline[0].owned = true;
	dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
	/* Fake deactivate releases the route, restores PHY, then mux fails. */
	selected = false;
	f.pipeline[0].owned = false;
	f.pipeline[0].fixed_busy = true;
	count = dcp_fabric_deactivate_steps(-EIO, selected, true, steps);
	KUNIT_ASSERT_EQ(test, count, 3U);
	KUNIT_EXPECT_EQ(test, steps[0], DCP_FABRIC_CLEAR_OWNER);
	KUNIT_EXPECT_EQ(test, steps[1], DCP_FABRIC_CONNECT_FIXED);
	KUNIT_EXPECT_EQ(test, steps[2], DCP_FABRIC_REPLAN);
	for (i = 0; i < count; i++) {
		switch (steps[i]) {
		case DCP_FABRIC_KEEP_OWNER:
			KUNIT_FAIL(test, "released route cannot retain ownership");
			break;
		case DCP_FABRIC_CLEAR_OWNER:
			f.port[0].owner[0] = NULL;
			break;
		case DCP_FABRIC_CONNECT_FIXED:
			KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], NULL);
			connects++;
			break;
		case DCP_FABRIC_REPLAN:
			KUNIT_EXPECT_EQ(test, connects, 1U);
			dcp_fabric_plan(f.pipeline, f.port, NULL, 0);
			replans++;
			break;
		}
	}
	KUNIT_EXPECT_EQ(test, connects, 1U);
	KUNIT_EXPECT_EQ(test, replans, 1U);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][1]);
	/* Success uses the same parent clear/connect sequence, without replan. */
	count = dcp_fabric_deactivate_steps(0, false, true, steps);
	KUNIT_EXPECT_EQ(test, count, 2U);
	KUNIT_EXPECT_EQ(test, steps[0], DCP_FABRIC_CLEAR_OWNER);
	KUNIT_EXPECT_EQ(test, steps[1], DCP_FABRIC_CONNECT_FIXED);
}

static void fabric_unbound_and_mask_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_init(&f, false);
	f.pipeline[0].bound = false;
	KUNIT_EXPECT_EQ(test, dcp_fabric_score(&f.pipeline[0], &f.policy),
			UINT_MAX - 1);
	KUNIT_EXPECT_TRUE(test,
			  dcp_fabric_fixed_busy(false, false, true, false));
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_fixed_busy(false, true, true, true));
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_fixed_busy(true, false, true, true));
	f.policy.dual_stream = true;
	KUNIT_EXPECT_TRUE(test,
			  dcp_fabric_fits(&f.pipeline[0], &f.policy, 0, true));
	KUNIT_EXPECT_FALSE(test, dcp_fabric_fits(&f.pipeline[1], &f.policy,
						 BIT(1), true));
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_connector_mask(true, true, true, 2,
						  BIT(1) | BIT(2)),
			(u32)(BIT(1) | BIT(2)));
}

static void fabric_follow_test(struct kunit *test)
{
	struct fabric_fixture f;
	struct dcp_fabric_route *lg, *hybrid;

	/* A direct port on the second pipeline is paired with the first CRTC. */
	fabric_init(&f, false);
	lg = &f.route[0][1];
	hybrid = &f.route[0][0];
	f.pipeline[1].owned = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_MOVE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, lg, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_STAY);
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(NULL, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, NULL, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);

	/*
	 * A live HDMI display keeps the hybrid.  The settling sample taken at
	 * probe does not; a recent HDMI edge does (fabric_follow_recent_fixed).
	 */
	f.pipeline[0].fixed_busy = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	f.pipeline[0].fixed_busy = false;
	f.pipeline[0].presence = DCP_FABRIC_SETTLING;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_MOVE);
	f.pipeline[0].presence = DCP_FABRIC_ABSENT;

	/* #39 on the M1 Pro: a Thunderbolt display alone follows to the hybrid. */
	lg->tunnel = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_MOVE);
	lg->tunnel = false;

	/* Owned with no route to swap with (a retiring tunnel), unbound, not up. */
	f.pipeline[0].owned = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	f.pipeline[0].owned = false;
	f.pipeline[0].bound = false;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	f.pipeline[0].bound = true;
	f.pipeline[0].services_ready = false;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(lg, hybrid, NULL, NULL, false,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	f.pipeline[0].services_ready = true;

	/* #39 on the M2 Max: DPMS on pairs two direct ports the other way round. */
	fabric_init(&f, true);
	f.pipeline[0].owned = true;
	f.pipeline[1].owned = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
						&f.route[1][0], &f.route[1][1],
						true, &f.policy),
			DCP_FABRIC_FOLLOW_SWAP);
	/* the holder's display is still lit elsewhere */
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
						&f.route[1][0], &f.route[1][1],
						false, &f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	/* the holder cannot reach the pipeline it would be given */
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
						&f.route[1][0], NULL, true,
						&f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
						&f.route[1][0], &f.route[1][0],
						true, &f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	/* Tunnels follow and swap like direct routes, keeping their binding. */
	f.route[1][0].tunnel = true;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
					  &f.route[1][0], &f.route[1][1], true, &f.policy),
			DCP_FABRIC_FOLLOW_SWAP);
	f.route[1][0].tunnel = false;
	f.route[0][1].tunnel = true;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
					  &f.route[1][0], &f.route[1][1], true, &f.policy),
			DCP_FABRIC_FOLLOW_SWAP);
}

static void fabric_follow_arrival_test(struct kunit *test)
{
	struct fabric_fixture f;

	/* Where routes follow their CRTC, streams arrive on the lowest CRTC. */
	fabric_init(&f, false);
	KUNIT_EXPECT_PTR_EQ(test, dcp_fabric_free_route(&f.port[0], &f.policy),
			    &f.route[0][1]);
	f.policy.follow = true;
	KUNIT_EXPECT_PTR_EQ(test, dcp_fabric_free_route(&f.port[0], &f.policy),
			    &f.route[0][0]);
	/* still not on a hybrid whose HDMI display is live */
	f.pipeline[0].fixed_busy = true;
	KUNIT_EXPECT_PTR_EQ(test, dcp_fabric_free_route(&f.port[0], &f.policy),
			    &f.route[0][1]);
	/* dual-stream ranking is unchanged */
	fabric_init(&f, true);
	f.policy.follow = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_score(&f.pipeline[0], &f.policy),
			dcp_fabric_score(&f.pipeline[0], &(struct dcp_fabric_policy){
				.dual_stream = true }));
}

static void fabric_follow_recent_fixed(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_init(&f, true);
	f.pipeline[0].fixed_recent = true;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
					  NULL, NULL, false, &f.policy),
			DCP_FABRIC_FOLLOW_REFUSE);
	f.pipeline[0].fixed_recent = false;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
					  NULL, NULL, false, &f.policy),
			DCP_FABRIC_FOLLOW_MOVE);
	f.pipeline[0].fixed_recent = true;
	f.pipeline[0].owned = true;
	f.pipeline[1].owned = true;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_follow(&f.route[0][1], &f.route[0][0],
					  &f.route[1][0], &f.route[1][1], true, &f.policy),
			DCP_FABRIC_FOLLOW_SWAP);
}

struct follow_effect_fixture {
	int owner[2];
	int prepare_error;
	int validate_error;
	int attach_error;
	int restore_error;
	int destination_detach_error;
	unsigned int prepared;
	unsigned int detached;
	unsigned int published;
	unsigned int lost;
};

static int follow_effect_prepare(void *data, unsigned int slot)
{
	struct follow_effect_fixture *f = data;

	f->prepared++;
	return f->prepare_error == slot + 1 ? -ENOMEM : 0;
}

static int follow_effect_validate(void *data, unsigned int slot)
{
	struct follow_effect_fixture *f = data;

	return f->validate_error == slot + 1 ? -ESTALE : 0;
}

static int follow_effect_detach(void *data, unsigned int slot, bool destination)
{
	struct follow_effect_fixture *f = data;

	f->owner[destination ? 1 - slot : slot] = -1;
	f->detached++;
	return destination && f->destination_detach_error == slot + 1 ? -EIO : 0;
}

static int follow_effect_attach(void *data, unsigned int slot, bool restore)
{
	struct follow_effect_fixture *f = data;

	if ((!restore && f->attach_error == slot + 1) ||
	    (restore && f->restore_error == slot + 1))
		return -EIO;
	f->owner[restore ? slot : 1 - slot] = slot;
	return 0;
}

static void follow_effect_publish(void *data, unsigned int slot, bool restore)
{
	struct follow_effect_fixture *f = data;

	f->published |= BIT(slot);
}

static void follow_effect_lost(void *data, unsigned int slot)
{
	struct follow_effect_fixture *f = data;

	f->lost |= BIT(slot);
}

static const struct dcp_fabric_follow_ops follow_effect_ops = {
	.prepare = follow_effect_prepare,
	.validate = follow_effect_validate,
	.detach = follow_effect_detach,
	.attach = follow_effect_attach,
	.publish = follow_effect_publish,
	.lost = follow_effect_lost,
};

static void fabric_follow_effect_swap(struct kunit *test)
{
	struct follow_effect_fixture f = { .owner = { 0, 1 } };

	KUNIT_ASSERT_EQ(test, dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), 0);
	KUNIT_EXPECT_EQ(test, f.owner[0], 1);
	KUNIT_EXPECT_EQ(test, f.owner[1], 0);
	KUNIT_EXPECT_EQ(test, f.prepared, 2U);
	KUNIT_EXPECT_EQ(test, f.published, 3U);
}

static void fabric_follow_effect_oom(struct kunit *test)
{
	unsigned int slot;

	for (slot = 1; slot <= 2; slot++) {
		struct follow_effect_fixture f = { .owner = { 0, 1 }, .prepare_error = slot };

		KUNIT_EXPECT_EQ(test,
				dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), -ENOMEM);
		KUNIT_EXPECT_EQ(test, f.detached, 0U);
		KUNIT_EXPECT_EQ(test, f.owner[0], 0);
		KUNIT_EXPECT_EQ(test, f.owner[1], 1);
		KUNIT_EXPECT_EQ(test, f.published, 0U);
	}
}

static void fabric_follow_effect_stale(struct kunit *test)
{
	struct follow_effect_fixture f = { .owner = { 0, 1 }, .validate_error = 2 };

	KUNIT_EXPECT_EQ(test, dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), -ESTALE);
	KUNIT_EXPECT_EQ(test, f.detached, 0U);
	KUNIT_EXPECT_EQ(test, f.owner[0], 0);
	KUNIT_EXPECT_EQ(test, f.owner[1], 1);
}

static void fabric_follow_effect_rollback(struct kunit *test)
{
	unsigned int slot;

	for (slot = 1; slot <= 2; slot++) {
		struct follow_effect_fixture f = { .owner = { 0, 1 }, .attach_error = slot };

		KUNIT_EXPECT_EQ(test, dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), -EIO);
		KUNIT_EXPECT_EQ(test, f.owner[0], 0);
		KUNIT_EXPECT_EQ(test, f.owner[1], 1);
		KUNIT_EXPECT_EQ(test, f.published, 3U);
		KUNIT_EXPECT_EQ(test, f.lost, 0U);
	}
}

static void fabric_follow_effect_release_restore(struct kunit *test)
{
	struct follow_effect_fixture f = {
		.owner = { 0, 1 }, .attach_error = 2, .destination_detach_error = 1,
	};

	KUNIT_EXPECT_EQ(test, dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), -EIO);
	KUNIT_EXPECT_EQ(test, f.owner[0], 0);
	KUNIT_EXPECT_EQ(test, f.owner[1], 1);
	KUNIT_EXPECT_EQ(test, f.lost, 0U);
	KUNIT_EXPECT_EQ(test, f.published, 3U);
}

static void fabric_follow_effect_lost(struct kunit *test)
{
	struct follow_effect_fixture f = {
		.owner = { 0, 1 }, .attach_error = 2, .restore_error = 1,
	};

	KUNIT_EXPECT_EQ(test, dcp_fabric_follow_execute(&follow_effect_ops, &f, 2), -EIO);
	KUNIT_EXPECT_EQ(test, f.owner[0], -1);
	KUNIT_EXPECT_EQ(test, f.owner[1], 1);
	KUNIT_EXPECT_EQ(test, f.lost, (unsigned int)BIT(0));
	KUNIT_EXPECT_EQ(test, f.published, (unsigned int)BIT(1));
}

static void fabric_presence_wrap_test(struct kunit *test)
{
	struct dcp_fabric_presence presence = {};
	u64 generation;

	generation = dcp_fabric_presence_edge(&presence, ULONG_MAX - 50, 100);
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_expire(&presence, generation, false, ULONG_MAX - 1));
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_expire(&presence, generation, false, 48));
	KUNIT_EXPECT_TRUE(test,
			  dcp_fabric_presence_expire(&presence, generation, false, 49));
	generation = dcp_fabric_presence_edge(&presence, 100, 100);
	/* An expiry which samples high cannot advertise capacity. */
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_expire(&presence, generation, true, 200));
	KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_PRESENT);
	KUNIT_EXPECT_EQ(test, presence.deadline, 0UL);
}

static void fabric_masked_edge_test(struct kunit *test)
{
	struct fabric_fixture f;
	struct dcp_fabric_presence presence = {};
	u64 generation;
	bool irq_masked = true, gpio_high = true;

	fabric_init(&f, false);
	f.pipeline[1].owned = true;
	generation = dcp_fabric_presence_edge(&presence, 100, 10000);
	KUNIT_ASSERT_TRUE(test,
			  dcp_fabric_presence_sample(&presence, generation, gpio_high, 100, 10000));
	KUNIT_ASSERT_EQ(test, presence.state, DCP_FABRIC_PRESENT);
	/* Fake oneshot debounce: fall with detector off, no hardirq generation. */
	gpio_high = false;
	KUNIT_EXPECT_TRUE(test, irq_masked);
	KUNIT_EXPECT_EQ(test, presence.generation, generation);
	/* IRQ completion unmasks before the queued synchronized recheck. */
	irq_masked = false;
	KUNIT_EXPECT_FALSE(test, irq_masked);
	KUNIT_ASSERT_TRUE(test,
			  dcp_fabric_presence_recheck(&presence, generation, gpio_high,
						      600, 10000));
	KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_SETTLING);
	KUNIT_EXPECT_EQ(test, presence.deadline, 10600UL);
	KUNIT_EXPECT_NE(test, presence.generation, generation);
	f.pipeline[0].presence = presence.state;
	KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0), -EBUSY);
	KUNIT_EXPECT_EQ(test, fabric_direct(&f, 1), -EBUSY);
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_recheck(&presence, generation, true, 601, 10000));
	generation = presence.generation;
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_recheck(&presence, generation, false, 602, 10000));
	KUNIT_EXPECT_EQ(test, presence.deadline, 10600UL);
	KUNIT_EXPECT_TRUE(test,
			  dcp_fabric_presence_expire(&presence, generation, false, 10600));
	f.pipeline[0].presence = presence.state;
	fabric_promote(&f);
	KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], &f.route[1][0]);
	/* A rise lost during a low handler clears the hold on post-unmask read. */
	generation = dcp_fabric_presence_edge(&presence, 20000, 10000);
	KUNIT_EXPECT_TRUE(test,
			  dcp_fabric_presence_recheck(&presence, generation, true, 20500, 10000));
	KUNIT_EXPECT_EQ(test, presence.state, DCP_FABRIC_PRESENT);
	KUNIT_EXPECT_EQ(test, presence.deadline, 0UL);
}

struct fabric_wiring_case {
	const char *name;
	bool dpin0;
	bool dpin1;
	bool legacy;
	unsigned int endpoints;
	bool soc_support;
	enum dcp_fabric_wiring expected;
	bool flow;
};

static const struct fabric_wiring_case wiring_cases[] = {
	{ "M1_base", false, false, false, 1, false, DCP_FABRIC_SINGLE_STREAM, false },
	{ "M1_pro_j314_two_candidates", false, false, false, 2, false,
	  DCP_FABRIC_SINGLE_STREAM, false },
	{ "M1_ultra", false, false, false, 1, false, DCP_FABRIC_SINGLE_STREAM, false },
	{ "M2_base", false, false, false, 1, false, DCP_FABRIC_SINGLE_STREAM, false },
	{ "M2_desktop", false, false, false, 1, true, DCP_FABRIC_SINGLE_STREAM, false },
	{ "M2_j414_new_dtb", true, true, false, 2, true, DCP_FABRIC_DUAL_NAMED, true },
	{ "M2_j416_new_dtb", true, true, false, 2, true, DCP_FABRIC_DUAL_NAMED, true },
	{ "M2_j414_old_esp_dtb", false, false, true, 2, true, DCP_FABRIC_DUAL_LEGACY, true },
	{ "M2_j416_old_esp_dtb", false, false, true, 2, true, DCP_FABRIC_DUAL_LEGACY, true },
	{ "M3_shared_crossbar_single", false, false, false, 1, false,
	  DCP_FABRIC_SINGLE_STREAM, false },
	{ "named_wins_over_legacy", true, true, true, 2, true, DCP_FABRIC_DUAL_NAMED, true },
	{ "dpin0_only_invalid", true, false, false, 2, true, DCP_FABRIC_INVALID_WIRING, false },
	{ "dpin1_only_invalid", false, true, true, 2, true, DCP_FABRIC_INVALID_WIRING, false },
	{ "no_graph", true, true, false, 0, true, DCP_FABRIC_SINGLE_STREAM, false },
	{ "one_candidate_named", true, true, false, 1, true, DCP_FABRIC_SINGLE_STREAM, false },
	{ "one_candidate_legacy", false, false, true, 1, true, DCP_FABRIC_SINGLE_STREAM, false },
	{ "future_wiring_cannot_enable_soc_flow", true, true, false, 2, false,
	  DCP_FABRIC_DUAL_NAMED, false },
};

static void fabric_wiring_desc(const struct fabric_wiring_case *row, char *desc)
{
	strscpy(desc, row->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(fabric_wiring, wiring_cases, fabric_wiring_desc);

static void fabric_wiring_test(struct kunit *test)
{
	const struct fabric_wiring_case *row = test->param_value;
	enum dcp_fabric_wiring wiring;
	bool dual;

	wiring = dcp_fabric_wiring(row->dpin0, row->dpin1, row->legacy, row->endpoints);
	KUNIT_EXPECT_EQ(test, wiring, row->expected);
	dual = wiring == DCP_FABRIC_DUAL_NAMED || wiring == DCP_FABRIC_DUAL_LEGACY;
	KUNIT_EXPECT_EQ(test, dcp_fabric_t6020_flow(true, row->soc_support, dual), row->flow);
	/* SoC and wiring qualification must never change HDMI or DP-alt flow. */
	KUNIT_EXPECT_FALSE(test, dcp_fabric_t6020_flow(false, row->soc_support, dual));
}

static void fabric_shared_wiring_test(struct kunit *test)
{
	const struct fabric_wiring_case *row = test->param_value;
	bool dual = row->expected == DCP_FABRIC_DUAL_NAMED ||
		    row->expected == DCP_FABRIC_DUAL_LEGACY;

	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_wiring_dual(row->endpoints, row->dpin0,
							  row->dpin1, row->legacy), dual);
	KUNIT_EXPECT_FALSE(test, apple_dp_tunnel_dual_stream(NULL));
}

enum fabric_fake_effect {
	FX_IRQ_ENABLE,
	FX_SAMPLE,
	FX_RESTORE_PHY,
	FX_SELECT_MUX,
	FX_MUX_FAIL,
	FX_CONNECT_FIXED,
	FX_WAIT,
	FX_WAITING,
	FX_REBALANCE,
	FX_REGISTER,
	FX_CANDIDATE,
	FX_BIND,
	FX_ACTIVATE,
	FX_ROLLBACK,
	FX_CLAIM,
	FX_QUEUE,
	FX_OOB,
	FX_SOC,
	FX_XBAR,
	FX_RECORD,
	FX_PLAN,
	FX_DEACTIVATE,
	FX_CLEAR,
	FX_KEEP,
};

struct fabric_fake {
	struct kunit *test;
	struct fabric_fixture f;
	struct dcp_fabric_presence presence;
	struct dcp_fabric_route *best;
	enum fabric_fake_effect trace[64];
	unsigned int count, reads, wait_ms, plans, attempts[2], releasing;
	unsigned int failed_activations, nr_typec_routes;
	int levels[3], activate_error, deactivate_error;
	bool borrowed, frozen, connector, irq_enabled, inject_edge;
	bool selected, fixed_live, has_xbar, t6030, t6020_xbar, persistent;
};

static void fabric_fake_init(struct fabric_fake *f, struct kunit *test, bool dual)
{
	memset(f, 0, sizeof(*f));
	f->test = test;
	f->connector = true;
	f->has_xbar = true;
	f->nr_typec_routes = 3;
	fabric_init(&f->f, dual);
}

static void fabric_fake_trace(struct fabric_fake *f, enum fabric_fake_effect effect)
{
	KUNIT_ASSERT_LT(f->test, f->count, (unsigned int)ARRAY_SIZE(f->trace));
	f->trace[f->count++] = effect;
}

static bool fabric_fake_borrowed(void *ctx)
{
	return ((struct fabric_fake *)ctx)->borrowed;
}

static bool fabric_fake_keep_order(void *ctx)
{
	struct fabric_fake *f = ctx;

	return dcp_fabric_keep_order(f->f.policy.dual_stream, true, f->frozen);
}

static bool fabric_fake_dual(void *ctx)
{
	return ((struct fabric_fake *)ctx)->f.policy.dual_stream;
}

static int fabric_fake_hpd(void *ctx)
{
	struct fabric_fake *f = ctx;
	unsigned int index = min(f->reads++, 2U);

	if (f->inject_edge) {
		f->inject_edge = false;
		dcp_fabric_presence_edge(&f->presence, 1001, 10000);
	}
	return f->levels[index];
}

static void fabric_fake_wait(void *ctx, unsigned int ms)
{
	struct fabric_fake *f = ctx;

	f->wait_ms += ms;
	fabric_fake_trace(f, FX_WAIT);
}

static void fabric_fake_waiting(void *ctx)
{
	fabric_fake_trace(ctx, FX_WAITING);
}

static void fabric_fake_rebalance(void *ctx)
{
	fabric_fake_trace(ctx, FX_REBALANCE);
}

static void fabric_fake_observed(void *ctx, bool high, bool debounced)
{
}

static void fabric_fake_restore_phy(void *ctx)
{
	fabric_fake_trace(ctx, FX_RESTORE_PHY);
}

static int fabric_fake_select_mux(void *ctx)
{
	fabric_fake_trace(ctx, FX_SELECT_MUX);
	return 0;
}

static const struct dcp_fabric_fixed_ops fabric_fake_fixed_ops = {
	.restore_phy = fabric_fake_restore_phy,
	.select_mux = fabric_fake_select_mux,
};

static void fabric_fake_connect(void *ctx)
{
	fabric_fake_trace(ctx, FX_CONNECT_FIXED);
}

static void fabric_fake_select_connect(void *ctx)
{
	struct fabric_fake *f = ctx;

	if (!dcp_fabric_run_fixed(true, f->borrowed, true, &fabric_fake_fixed_ops, f))
		fabric_fake_connect(f);
}

static const struct dcp_fabric_hdmi_ops fabric_fake_hdmi_ops = {
	.borrowed = fabric_fake_borrowed,
	.keep_order = fabric_fake_keep_order,
	.dual_stream = fabric_fake_dual,
	.read_hpd = fabric_fake_hpd,
	.wait = fabric_fake_wait,
	.rebalance = fabric_fake_rebalance,
	.waiting = fabric_fake_waiting,
	.observed = fabric_fake_observed,
	.connect_fixed = fabric_fake_select_connect,
};

static void fabric_fake_enable_irq(void *ctx)
{
	struct fabric_fake *f = ctx;

	f->irq_enabled = true;
	fabric_fake_trace(f, FX_IRQ_ENABLE);
}

static u64 fabric_fake_edge(void *ctx)
{
	struct fabric_fake *f = ctx;

	return dcp_fabric_presence_edge(&f->presence, 1000, 10000);
}

static bool fabric_fake_sample(void *ctx, u64 generation, int level)
{
	struct fabric_fake *f = ctx;

	KUNIT_EXPECT_TRUE(f->test, f->irq_enabled);
	fabric_fake_trace(f, FX_SAMPLE);
	return dcp_fabric_presence_sample(&f->presence, generation, level > 0, 1000, 10000);
}

static const struct dcp_fabric_resume_sample_ops fabric_fake_resume_sample_ops = {
	.edge = fabric_fake_edge,
	.read_hpd = fabric_fake_hpd,
	.sample = fabric_fake_sample,
};

static void fabric_fake_resume_sample(void *ctx)
{
	struct fabric_fake *f = ctx;
	bool enabled = dcp_fabric_hdmi_settle_enabled(f->f.pipeline[0].has_fixed,
						   f->nr_typec_routes, f->f.policy.dual_stream);

	dcp_fabric_run_resume_sample(enabled, &fabric_fake_resume_sample_ops, ctx);
}

static const struct dcp_fabric_resume_ops fabric_fake_resume_ops = {
	.enable_irq = fabric_fake_enable_irq,
	.sample = fabric_fake_resume_sample,
};

static int fabric_fake_register(void *ctx)
{
	fabric_fake_trace(ctx, FX_REGISTER);
	return 0;
}

static int fabric_fake_candidate(void *ctx)
{
	struct fabric_fake *f = ctx;
	int error;

	fabric_fake_trace(f, FX_CANDIDATE);
	f->best = dcp_fabric_tunnel_candidate(&f->f.port[0], &f->f.policy, NULL,
					      false, 0, f->connector, &error);
	return f->best ? 0 : error;
}

static int fabric_fake_activate(void *ctx)
{
	struct fabric_fake *f = ctx;

	fabric_fake_trace(f, FX_BIND);
	fabric_fake_trace(f, FX_ACTIVATE);
	/* Fault injection may bypass admission: reject a missing target safely. */
	return f->best ? f->activate_error : -EINVAL;
}

static void fabric_fake_rollback(void *ctx)
{
	fabric_fake_trace(ctx, FX_ROLLBACK);
}

static void fabric_fake_claim(void *ctx)
{
	struct fabric_fake *f = ctx;

	fabric_fake_trace(f, FX_CLAIM);
	f->f.port[0].owner[0] = f->best;
	f->best->pipeline->owned = true;
}

static bool fabric_fake_external(void *ctx)
{
	struct fabric_fake *f = ctx;

	return f->best->pipeline->external;
}

static bool fabric_fake_connector(void *ctx)
{
	return ((struct fabric_fake *)ctx)->connector;
}

static void fabric_fake_queue(void *ctx)
{
	fabric_fake_trace(ctx, FX_QUEUE);
}

static void fabric_fake_oob(void *ctx)
{
	fabric_fake_trace(ctx, FX_OOB);
}

static const struct dcp_fabric_attach_ops fabric_fake_attach_ops = {
	.candidate = fabric_fake_candidate,
	.activate = fabric_fake_activate,
	.rollback = fabric_fake_rollback,
	.claim = fabric_fake_claim,
	.external = fabric_fake_external,
	.has_connector = fabric_fake_connector,
	.queue_reconnect = fabric_fake_queue,
	.connect_oob = fabric_fake_oob,
};

static int fabric_fake_dispatch(void *ctx, const struct dcp_fabric_port *port)
{
	struct fabric_fake *f = ctx;

	KUNIT_EXPECT_PTR_EQ(f->test, port, &f->f.port[0]);
	return dcp_fabric_run_attach(&fabric_fake_attach_ops, f);
}

static bool fabric_fake_has_xbar(void *ctx)
{
	return ((struct fabric_fake *)ctx)->has_xbar;
}

static bool fabric_fake_tunnel(void *ctx)
{
	return true;
}

static bool fabric_fake_xbar(void *ctx)
{
	struct fabric_fake *f = ctx;

	fabric_fake_trace(f, FX_XBAR);
	return f->t6020_xbar;
}

static bool fabric_fake_soc(void *ctx)
{
	struct fabric_fake *f = ctx;

	fabric_fake_trace(f, FX_SOC);
	return f->t6030;
}

static void fabric_fake_record_soc(void *ctx, bool t6030)
{
	struct fabric_fake *f = ctx;

	f->f.pipeline[0].t6030_dpin = t6030;
	fabric_fake_trace(f, FX_RECORD);
}

static const struct dcp_fabric_link_ops fabric_fake_link_ops = {
	.has_xbar = fabric_fake_has_xbar,
	.tunnel = fabric_fake_tunnel,
	.t6020_xbar = fabric_fake_xbar,
	.t6030_soc = fabric_fake_soc,
	.record_soc = fabric_fake_record_soc,
};

static int fabric_fake_deactivate(void *ctx)
{
	struct fabric_fake *f = ctx;
	struct dcp_fabric_route *owner = f->f.port[f->releasing].owner[0];
	int ret = f->deactivate_error;

	fabric_fake_trace(f, FX_DEACTIVATE);
	/* Model the real failure: release ownership, restore PHY, fail mux. */
	owner->pipeline->owned = false;
	f->selected = false;
	fabric_fake_trace(f, FX_RESTORE_PHY);
	if (ret)
		fabric_fake_trace(f, FX_MUX_FAIL);
	f->deactivate_error = 0;
	return ret;
}

static bool fabric_fake_selected(void *ctx)
{
	return ((struct fabric_fake *)ctx)->selected;
}

static bool fabric_fake_fixed_live(void *ctx)
{
	return ((struct fabric_fake *)ctx)->fixed_live;
}

static void fabric_fake_keep(void *ctx)
{
	fabric_fake_trace(ctx, FX_KEEP);
}

static void fabric_fake_clear(void *ctx)
{
	struct fabric_fake *f = ctx;

	f->f.port[f->releasing].owner[0] = NULL;
	fabric_fake_trace(f, FX_CLEAR);
}

static const struct dcp_fabric_deactivate_ops fabric_fake_deactivate_ops = {
	.deactivate = fabric_fake_deactivate,
	.selected = fabric_fake_selected,
	.fixed_live = fabric_fake_fixed_live,
	.keep_owner = fabric_fake_keep,
	.clear_owner = fabric_fake_clear,
	.connect_fixed = fabric_fake_connect,
};

static void fabric_fake_plan(void *ctx)
{
	struct fabric_fake *f = ctx;

	f->plans++;
	fabric_fake_trace(f, FX_PLAN);
	dcp_fabric_plan(f->f.pipeline, f->f.port, NULL, 0);
}

static void *fabric_fake_first(void *ctx)
{
	struct fabric_fake *f = ctx;

	return &f->f.port[0];
}

static void *fabric_fake_next(void *ctx, void *entry)
{
	struct fabric_fake *f = ctx;
	struct dcp_fabric_port *port = entry;
	unsigned int p = port - f->f.port;

	return p < 2 ? &f->f.port[p + 1] : NULL;
}

static bool fabric_fake_deactivate_one(void *ctx, void *entry)
{
	struct fabric_fake *f = ctx;
	struct dcp_fabric_port *port = entry;
	unsigned int p = port - f->f.port;

	if (!dcp_fabric_movable(port->owner[0], f->f.plan[p].target[0]))
		return false;
	f->releasing = p;
	return dcp_fabric_run_deactivate(&fabric_fake_deactivate_ops, f);
}

static bool fabric_fake_activate_one(void *ctx, void *entry)
{
	struct fabric_fake *f = ctx;
	struct dcp_fabric_port *port = entry;
	unsigned int p = port - f->f.port;
	struct dcp_fabric_route *target = f->f.plan[p].target[0];
	unsigned int index;

	if (port->owner[0] || !target || !port->wanted || !port->hpd)
		return false;
	index = target->pipeline == &f->f.pipeline[0] ? 0 : 1;
	f->attempts[index]++;
	fabric_fake_trace(f, FX_ACTIVATE);
	if (f->failed_activations || (f->persistent && f->plans < 4)) {
		if (f->failed_activations)
			f->failed_activations--;
		f->f.pipeline[0].fixed_busy = true;
		return true;
	}
	target->pipeline->owned = true;
	port->owner[0] = target;
	return false;
}

static const struct dcp_fabric_rebalance_ops fabric_fake_rebalance_ops = {
	.plan = fabric_fake_plan,
	.first = fabric_fake_first,
	.next = fabric_fake_next,
	.deactivate = fabric_fake_deactivate_one,
	.activate = fabric_fake_activate_one,
};

static void fabric_effect_s8_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	f.borrowed = true;
	f.frozen = true;
	f.levels[0] = 1;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 1U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_WAITING);
	/* When the direct route leaves, the same handler restores and connects. */
	f.borrowed = false;
	f.reads = 0;
	f.levels[1] = 1;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 5U);
	KUNIT_EXPECT_EQ(test, f.trace[1], FX_WAIT);
	KUNIT_EXPECT_EQ(test, f.trace[2], FX_RESTORE_PHY);
	KUNIT_EXPECT_EQ(test, f.trace[3], FX_SELECT_MUX);
	KUNIT_EXPECT_EQ(test, f.trace[4], FX_CONNECT_FIXED);
}

static void fabric_effect_s32_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, true);
	f.borrowed = true;
	f.levels[0] = 1;
	f.levels[1] = 1;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 2U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_WAIT);
	KUNIT_EXPECT_EQ(test, f.trace[1], FX_REBALANCE);
	KUNIT_EXPECT_EQ(test, f.wait_ms, 500U);
	/* The post-debounce level and compositor gate both suppress reclaim. */
	f.count = 0;
	f.reads = 0;
	f.wait_ms = 0;
	f.levels[1] = 0;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 1U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_WAIT);
	f.count = 0;
	f.reads = 0;
	f.frozen = true;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_EXPECT_EQ(test, f.count, 0U);
}

static void fabric_effect_s16_test(struct kunit *test)
{
	struct fabric_fake f;
	u64 generation;

	fabric_fake_init(&f, test, false);
	dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 2U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_IRQ_ENABLE);
	KUNIT_EXPECT_EQ(test, f.trace[1], FX_SAMPLE);
	KUNIT_EXPECT_EQ(test, f.presence.state, DCP_FABRIC_SETTLING);
	KUNIT_EXPECT_EQ(test, f.presence.deadline, 11000UL);
	generation = f.presence.generation;
	f.count = 0;
	f.reads = 0;
	f.levels[0] = 1;
	dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 2U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_IRQ_ENABLE);
	KUNIT_EXPECT_EQ(test, f.trace[1], FX_SAMPLE);
	KUNIT_EXPECT_EQ(test, f.presence.state, DCP_FABRIC_PRESENT);
	KUNIT_EXPECT_EQ(test, f.presence.deadline, 0UL);
	KUNIT_EXPECT_FALSE(test,
			   dcp_fabric_presence_expire(&f.presence, generation, false, 11000));
	/* A falling hardirq during GPIO sampling invalidates the high sample. */
	f.count = 0;
	f.reads = 0;
	f.inject_edge = true;
	dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
	KUNIT_EXPECT_EQ(test, f.count, 2U);
	KUNIT_EXPECT_EQ(test, f.presence.state, DCP_FABRIC_SETTLING);
}

static void fabric_effect_s50a_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_probe(false, fabric_fake_register, &f), 0);
	KUNIT_EXPECT_EQ(test, f.count, 0U);
	KUNIT_EXPECT_FALSE(test, f.f.pipeline[0].owned);
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_probe(true, fabric_fake_register, &f), 0);
	KUNIT_ASSERT_EQ(test, f.count, 1U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_REGISTER);
}

static void fabric_effect_s50bc_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	f.f.port[0].routes = &f.f.route[0][1];
	f.f.pipeline[1].external = true;
	f.f.pipeline[1].services_ready = false;
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), -EAGAIN);
	KUNIT_ASSERT_EQ(test, f.count, 1U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_CANDIDATE);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], NULL);
	f.count = 0;
	f.f.pipeline[1].terminal = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), -ESHUTDOWN);
	KUNIT_EXPECT_EQ(test, f.count, 1U);
	f.count = 0;
	f.f.pipeline[1].terminal = false;
	f.f.pipeline[1].services_ready = true;
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), 0);
	KUNIT_ASSERT_EQ(test, f.count, 5U);
	KUNIT_EXPECT_EQ(test, f.trace[4], FX_QUEUE);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], &f.f.route[0][1]);
}

static void fabric_effect_s50d_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	KUNIT_ASSERT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), 0);
	KUNIT_ASSERT_EQ(test, f.count, 5U);
	KUNIT_EXPECT_EQ(test, f.trace[4], FX_OOB);
	fabric_fake_init(&f, test, false);
	f.f.pipeline[1].external = true;
	KUNIT_ASSERT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), 0);
	KUNIT_ASSERT_EQ(test, f.count, 5U);
	KUNIT_EXPECT_EQ(test, f.trace[4], FX_QUEUE);
	fabric_fake_init(&f, test, false);
	f.activate_error = -EIO;
	KUNIT_EXPECT_EQ(test, dcp_fabric_run_attach(&fabric_fake_attach_ops, &f), -EIO);
	KUNIT_ASSERT_EQ(test, f.count, 4U);
	KUNIT_EXPECT_EQ(test, f.trace[3], FX_ROLLBACK);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], NULL);
}

static void fabric_effect_s50e_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	f.t6030 = true;
	f.t6020_xbar = true;
	KUNIT_EXPECT_TRUE(test, dcp_fabric_run_t6030_link(&fabric_fake_link_ops, &f));
	KUNIT_EXPECT_TRUE(test, f.f.pipeline[0].t6030_dpin);
	f.t6030 = false;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_run_t6030_link(&fabric_fake_link_ops, &f));
	KUNIT_EXPECT_FALSE(test, f.f.pipeline[0].t6030_dpin);
	f.t6030 = true;
	f.t6020_xbar = false;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_run_t6030_link(&fabric_fake_link_ops, &f));
	f.has_xbar = false;
	f.count = 0;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_run_t6030_link(&fabric_fake_link_ops, &f));
	KUNIT_EXPECT_EQ(test, f.count, 0U);
}

static void fabric_effect_s50f_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, false);
	f.f.port[0].next = NULL;
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_run_request(f.f.port, 2, 0, fabric_fake_dispatch, &f), -ENODEV);
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_run_request(f.f.port, 3, 0, fabric_fake_dispatch, &f), -ENODEV);
	KUNIT_EXPECT_EQ(test, f.count, 0U);
	KUNIT_EXPECT_EQ(test,
			dcp_fabric_run_request(f.f.port, 1, 0, fabric_fake_dispatch, &f), 0);
	KUNIT_EXPECT_EQ(test, f.count, 5U);
}

static void fabric_effect_failure_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, true);
	f.f.port[0].wanted = true;
	f.f.port[0].hpd = true;
	f.f.port[1].wanted = true;
	f.f.port[1].hpd = true;
	f.failed_activations = 1;
	dcp_fabric_run_rebalance(&fabric_fake_rebalance_ops, &f);
	KUNIT_EXPECT_EQ(test, f.plans, 2U);
	KUNIT_EXPECT_EQ(test, f.attempts[0], 1U);
	KUNIT_EXPECT_EQ(test, f.attempts[1], 1U);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], &f.f.route[0][1]);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[1].owner[0], NULL);
	/* Persistent failure still takes the third fresh snapshot and stops. */
	fabric_fake_init(&f, test, true);
	f.f.port[0].wanted = true;
	f.f.port[0].hpd = true;
	f.f.port[1].wanted = true;
	f.f.port[1].hpd = true;
	f.persistent = true;
	dcp_fabric_run_rebalance(&fabric_fake_rebalance_ops, &f);
	KUNIT_EXPECT_EQ(test, f.plans, 3U);
	KUNIT_EXPECT_EQ(test, f.attempts[0], 1U);
	KUNIT_EXPECT_EQ(test, f.attempts[1], 1U);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], NULL);
	KUNIT_EXPECT_PTR_EQ(test, f.f.plan[0].target[0], &f.f.route[0][1]);
}

static void fabric_effect_deactivate_failure_test(struct kunit *test)
{
	struct fabric_fake f;

	fabric_fake_init(&f, test, true);
	f.f.port[0].wanted = true;
	f.f.port[0].hpd = true;
	f.f.port[0].owner[0] = &f.f.route[0][0];
	f.f.pipeline[0].owned = true;
	f.f.pipeline[0].fixed_busy = true;
	f.deactivate_error = -EIO;
	f.fixed_live = true;
	dcp_fabric_run_rebalance(&fabric_fake_rebalance_ops, &f);
	KUNIT_ASSERT_GE(test, f.count, 8U);
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_PLAN);
	KUNIT_EXPECT_EQ(test, f.trace[1], FX_DEACTIVATE);
	KUNIT_EXPECT_EQ(test, f.trace[2], FX_RESTORE_PHY);
	KUNIT_EXPECT_EQ(test, f.trace[3], FX_MUX_FAIL);
	KUNIT_EXPECT_EQ(test, f.trace[4], FX_CLEAR);
	KUNIT_EXPECT_EQ(test, f.trace[5], FX_CONNECT_FIXED);
	KUNIT_EXPECT_EQ(test, f.trace[6], FX_PLAN);
	KUNIT_EXPECT_EQ(test, f.plans, 2U);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], &f.f.route[0][1]);
}

static void fabric_capacity_notification_test(struct kunit *test)
{
	struct fabric_fake f;
	u64 generation;
	bool available;

	fabric_fake_init(&f, test, false);
	generation = dcp_fabric_presence_edge(&f.presence, 1000, 10000);
	for (unsigned int i = 0; i < 3; i++) {
		available = dcp_fabric_presence_expire(&f.presence, generation,
						       false, i ? 11000 : 10999);
		dcp_fabric_run_capacity(available, fabric_fake_rebalance, &f);
		KUNIT_EXPECT_EQ(test, f.count, i ? 1U : 0U);
	}
	KUNIT_EXPECT_EQ(test, f.trace[0], FX_REBALANCE);
}

static void fabric_connector_flow_core_test(struct kunit *test)
{
	bool single_stream = apple_dp_tunnel_wiring_dual(2, false, false, false);

	/* Plain dcpext2/3 routes inherit the physical connector's capacity. */
	for (unsigned int dcp = 0; dcp < 4; dcp++) {
		bool route_has_dpin = dcp < 2;
		bool connector_wired = apple_dp_tunnel_wiring_dual(2, true, true, false);

		KUNIT_EXPECT_TRUE(test, dcp_fabric_t6020_flow(true, true, connector_wired));
		if (!route_has_dpin)
			KUNIT_EXPECT_FALSE(test, dcp_fabric_t6020_flow(true, true, route_has_dpin));
		KUNIT_EXPECT_FALSE(test, dcp_fabric_t6020_flow(false, true, connector_wired));
		KUNIT_EXPECT_FALSE(test, dcp_fabric_t6020_flow(true, false, connector_wired));
	}
	/* Two endpoints alone do not enable the flow on a single-stream port. */
	KUNIT_EXPECT_FALSE(test, dcp_fabric_t6020_flow(true, true, single_stream));
}

static void fabric_resume_no_connect_test(struct kunit *test)
{
	for (unsigned int row = 0; row < 8; row++) {
		struct fabric_fake f;

		fabric_fake_init(&f, test, !!(row & 4));
		f.borrowed = !!(row & 2);
		f.levels[0] = !!(row & 1);
		dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
		KUNIT_ASSERT_EQ(test, f.count, (row & 4) ? 1U : 2U);
		KUNIT_EXPECT_EQ(test, f.trace[0], FX_IRQ_ENABLE);
		if (!(row & 4)) {
			KUNIT_EXPECT_EQ(test, f.trace[1], FX_SAMPLE);
			KUNIT_EXPECT_EQ(test, f.presence.state,
					(row & 1) ? DCP_FABRIC_PRESENT : DCP_FABRIC_SETTLING);
		} else {
			KUNIT_EXPECT_EQ(test, f.presence.state, DCP_FABRIC_ABSENT);
		}
	}
}

static void fabric_nonhybrid_hdmi_resume_test(struct kunit *test)
{
	/* HDMI-only pipelines and Type-C-only pipelines never arm the hold. */
	for (unsigned int row = 0; row < 6; row++) {
		struct fabric_fake f;

		fabric_fake_init(&f, test, row == 5);
		f.nr_typec_routes = row < 4 ? 0 : 3;
		f.f.pipeline[0].has_fixed = row != 4;
		f.levels[0] = 1;
		dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
		KUNIT_ASSERT_EQ(test, f.count, 1U);
		KUNIT_EXPECT_EQ(test, f.trace[0], FX_IRQ_ENABLE);
		KUNIT_EXPECT_EQ(test, f.reads, 0U);
		KUNIT_EXPECT_EQ(test, f.presence.state, DCP_FABRIC_ABSENT);
		KUNIT_EXPECT_EQ(test, f.presence.generation, 0ULL);
		KUNIT_EXPECT_EQ(test, f.presence.deadline, 0UL);
	}
}

static void fabric_rebind_presence_test(struct kunit *test)
{
	for (unsigned int high = 0; high < 2; high++) {
		struct fabric_fake f;
		u64 previous;
		bool expired;

		fabric_fake_init(&f, test, false);
		previous = dcp_fabric_presence_edge(&f.presence, 0, 10000);
		/* Unbind drained the expiry; bind re-enables work before this sample. */
		f.levels[0] = high;
		dcp_fabric_run_resume(&fabric_fake_resume_ops, &f);
		KUNIT_ASSERT_EQ(test, f.count, 2U);
		KUNIT_EXPECT_EQ(test, f.trace[0], FX_IRQ_ENABLE);
		KUNIT_EXPECT_EQ(test, f.trace[1], FX_SAMPLE);
		KUNIT_EXPECT_EQ(test, f.presence.state,
				high ? DCP_FABRIC_PRESENT : DCP_FABRIC_SETTLING);
		KUNIT_EXPECT_EQ(test, f.presence.deadline, high ? 0UL : 11000UL);
		expired = dcp_fabric_presence_expire(&f.presence, previous, false, 11000);
		KUNIT_EXPECT_FALSE(test, expired);
		if (!high) {
			expired = dcp_fabric_presence_expire(&f.presence,
							     f.presence.generation, false, 11000);
			KUNIT_EXPECT_TRUE(test, expired);
		}
	}
}

struct reclaim_fixture {
	int release_error, move_error, restore_error;
	unsigned int events[8], count;
	int owner;
	bool reported_lost;
};

static int reclaim_release(void *data)
{
	struct reclaim_fixture *f = data;

	f->events[f->count++] = 1;
	f->owner = -1;
	return f->release_error;
}

static void reclaim_unplug(void *data)
{
	struct reclaim_fixture *f = data;

	f->events[f->count++] = 2;
}

static void reclaim_hdmi(void *data)
{
	struct reclaim_fixture *f = data;

	f->events[f->count++] = 3;
}

static int reclaim_activate(void *data, bool restore)
{
	struct reclaim_fixture *f = data;
	int ret = restore ? f->restore_error : f->move_error;

	f->events[f->count++] = restore ? 5 : 4;
	if (!ret)
		f->owner = restore ? 0 : 1;
	return ret;
}

static void reclaim_publish(void *data, bool restore)
{
	struct reclaim_fixture *f = data;

	f->events[f->count++] = restore ? 7 : 6;
}

static void reclaim_lost(void *data)
{
	struct reclaim_fixture *f = data;

	f->events[f->count++] = 8;
	f->reported_lost = true;
}

static const struct dcp_fabric_reclaim_ops reclaim_ops = {
	.release = reclaim_release,
	.unplug = reclaim_unplug,
	.connect_hdmi = reclaim_hdmi,
	.activate = reclaim_activate,
	.publish = reclaim_publish,
	.lost = reclaim_lost,
};

static void fabric_reclaim_transaction_test(struct kunit *test)
{
	struct reclaim_fixture success = {}, move = { .move_error = -EIO };
	struct reclaim_fixture release = { .release_error = -EIO };
	struct reclaim_fixture lost = { .move_error = -EIO, .restore_error = -EIO };
	unsigned int i;
	const unsigned int expected[] = { 1, 2, 3, 4, 6 };

	KUNIT_EXPECT_EQ(test, dcp_fabric_reclaim_execute(&reclaim_ops, &success), 0);
	KUNIT_ASSERT_EQ(test, success.count, (unsigned int)ARRAY_SIZE(expected));
	for (i = 0; i < ARRAY_SIZE(expected); i++)
		KUNIT_EXPECT_EQ(test, success.events[i], expected[i]);
	KUNIT_EXPECT_EQ(test, success.owner, 1);
	KUNIT_EXPECT_EQ(test, dcp_fabric_reclaim_execute(&reclaim_ops, &move), -EIO);
	KUNIT_EXPECT_EQ(test, move.owner, 0);
	KUNIT_EXPECT_EQ(test, move.events[4], 5U);
	KUNIT_EXPECT_EQ(test, move.events[5], 7U);
	KUNIT_EXPECT_FALSE(test, move.reported_lost);
	KUNIT_EXPECT_EQ(test, dcp_fabric_reclaim_execute(&reclaim_ops, &release), -EIO);
	KUNIT_EXPECT_EQ(test, release.owner, 0);
	KUNIT_ASSERT_EQ(test, release.count, 4U);
	KUNIT_EXPECT_EQ(test, release.events[2], 5U);
	KUNIT_EXPECT_EQ(test, release.events[3], 7U);
	KUNIT_EXPECT_EQ(test, dcp_fabric_reclaim_execute(&reclaim_ops, &lost), -EIO);
	KUNIT_EXPECT_EQ(test, lost.owner, -1);
	KUNIT_EXPECT_TRUE(test, lost.reported_lost);
	KUNIT_EXPECT_EQ(test, lost.events[5], 8U);
}

static void fabric_t6030_init(struct fabric_fixture *f)
{
	unsigned int p;

	fabric_init(f, dcp_fabric_ordered_dual_stream(true, true));
	for (p = 0; p < 3; p++) {
		f->port[p].candidate_crtcs[1] = BIT(1) | BIT(2);
		f->route[p][0].tunnel_clock_blocked =
			dcp_fabric_tunnel_clock_blocked(true, 0);
		f->route[p][1].tunnel_clock_blocked =
			dcp_fabric_tunnel_clock_blocked(true, 2);
	}
}

static void fabric_t6030_dual_policy_test(struct kunit *test)
{
	struct fabric_fake f;
	u32 mask;

	fabric_fake_init(&f, test, dcp_fabric_ordered_dual_stream(true, true));
	KUNIT_EXPECT_FALSE(test, f.f.policy.dual_stream);
	KUNIT_EXPECT_TRUE(test, dcp_fabric_ordered_dual_stream(true, false));
	KUNIT_EXPECT_FALSE(test, dcp_fabric_ordered_dual_stream(false, true));
	/* A primary dock stream leaves the fixed processor free for HDMI. */
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f.f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], &f.f.route[0][1]);
	f.borrowed = f.f.pipeline[0].owned;
	f.frozen = true;
	f.levels[0] = f.levels[1] = 1;
	dcp_fabric_run_hdmi(&fabric_fake_hdmi_ops, &f);
	KUNIT_ASSERT_EQ(test, f.count, 4U);
	KUNIT_EXPECT_EQ(test, f.trace[3], FX_CONNECT_FIXED);
	/* HDMI owns one processor; the other still takes the first tunnel. */
	fabric_release(&f.f, 0, 0);
	f.f.pipeline[0].fixed_busy = true;
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f.f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[0], &f.f.route[0][1]);
	f.f.pipeline[0].fixed_busy = false;
	f.f.port[0].candidate_crtcs[1] = BIT(1) | BIT(2);
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f.f, 0, 1), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.f.port[0].owner[1], &f.f.route[0][0]);
	/* Both encoders expose candidates, then narrow to the actual owner. */
	mask = dcp_fabric_port_crtc_mask(true, false, false, 0, BIT(1));
	mask = dcp_fabric_port_crtc_mask(true, false, true, mask, BIT(2));
	KUNIT_EXPECT_EQ(test, mask, (u32)(BIT(1) | BIT(2)));
	KUNIT_EXPECT_EQ(test, dcp_fabric_connector_mask(false, true, true, 1, mask), (u32)BIT(1));
	KUNIT_EXPECT_EQ(test, dcp_fabric_connector_mask(false, true, true, 2, mask), (u32)BIT(2));
	/* Ordered M2 secondary connectors retain the Type-C-only processor. */
	mask = dcp_fabric_port_crtc_mask(true, true, false, 0, BIT(1));
	mask = dcp_fabric_port_crtc_mask(true, true, true, mask, BIT(2));
	KUNIT_EXPECT_EQ(test, mask, (u32)BIT(2));
	mask = dcp_fabric_port_crtc_mask(false, true, false, mask, BIT(1));
	KUNIT_EXPECT_EQ(test, mask, (u32)(BIT(1) | BIT(2)));
}

/*
 * M2 Ultra Mac Studio: HDMI runs from die 1, so all four die-0 pipelines on a
 * rear port are Type-C-only.  The second stream of a daisy chain or dock must
 * not be fixed to the first stream's pipeline.
 */
static void fabric_secondary_crtc_test(struct kunit *test)
{
	static const u32 ultra[] = { BIT(0), BIT(1), BIT(2), BIT(3) };
	static const bool ultra_tc[] = { true, true, true, true };
	static const u32 laptop[] = { BIT(1), BIT(2), BIT(3), BIT(4) };
	static const bool laptop_tc[] = { false, true, true, true };
	static const u32 unsorted[] = { BIT(3), BIT(2), BIT(0), BIT(1) };
	static const bool single_tc[] = { true };
	static const bool none_tc[] = { false, false };
	struct dcp_fabric_pipeline pipeline[4] = {};
	struct dcp_fabric_route route[4] = {};
	struct dcp_fabric_plan plan = {};
	struct dcp_fabric_port port = {};
	struct dcp_fabric_policy policy = { .dual_stream = true };
	struct dcp_fabric_route *chosen;
	unsigned int i;
	int error;

	/* Laptops (hybrid dcpext0 lowest) keep dcpext1, with or without dcpext2/3. */
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(laptop, laptop_tc, 2), (u32)BIT(2));
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(laptop, laptop_tc, 4), (u32)BIT(2));
	/* All Type-C-only: skip the first stream's pipeline, in any order. */
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(ultra, ultra_tc, 2), (u32)BIT(1));
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(ultra, ultra_tc, 4), (u32)BIT(1));
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(unsorted, ultra_tc, 4), (u32)BIT(1));
	/* One Type-C-only pipeline keeps it; none gives no candidate. */
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(ultra, single_tc, 1), (u32)BIT(0));
	KUNIT_EXPECT_EQ(test, dcp_fabric_secondary_crtc(laptop, none_tc, 2), 0U);

	for (i = 0; i < 4; i++) {
		pipeline[i].bound = true;
		pipeline[i].crtc_index = i;
		pipeline[i].services_ready = true;
		if (i < 3) {
			pipeline[i].next = &pipeline[i + 1];
			route[i].next = &route[i + 1];
		}
		route[i].pipeline = &pipeline[i];
	}
	port.key = 1;
	port.routes = &route[0];
	port.plan = &plan;
	port.candidate_crtcs[0] = BIT(0) | BIT(1) | BIT(2) | BIT(3);
	port.candidate_crtcs[1] = dcp_fabric_secondary_crtc(ultra, ultra_tc, 4);

	/* The first stream takes dcpext0 (CRTC 0), as on the hardware. */
	chosen = dcp_fabric_tunnel_candidate(&port, &policy, NULL, false, 0,
					     true, &error);
	KUNIT_ASSERT_PTR_EQ(test, chosen, &route[0]);
	pipeline[0].owned = true;
	pipeline[0].tunnel_held = true;
	route[0].tunnel = true;
	port.owner[0] = &route[0];

	/* The second stream gets dcpext1 instead of -EBUSY. */
	chosen = dcp_fabric_tunnel_candidate(&port, &policy, NULL, false, 1,
					     true, &error);
	KUNIT_EXPECT_PTR_EQ(test, chosen, &route[1]);
	KUNIT_EXPECT_EQ(test, error, 0);

	/* So does the ordered pairing pass. */
	dcp_fabric_plan(&pipeline[0], &port, &port, 1);
	KUNIT_EXPECT_PTR_EQ(test, plan.target[0], &route[0]);
	KUNIT_EXPECT_PTR_EQ(test, plan.target[1], &route[1]);

	/* The old rule fixed it to the first stream's CRTC: nothing fits. */
	port.candidate_crtcs[1] = BIT(0);
	chosen = dcp_fabric_tunnel_candidate(&port, &policy, NULL, false, 1,
					     true, &error);
	KUNIT_EXPECT_NULL(test, chosen);
	KUNIT_EXPECT_EQ(test, error, -EBUSY);
	dcp_fabric_plan(&pipeline[0], &port, &port, 1);
	KUNIT_EXPECT_NULL(test, plan.target[1]);
}

static void fabric_t6030_clock_header_parity_test(struct kunit *test)
{
	static const struct {
		unsigned int source;
		unsigned int dpin;
		int pclk;
	} cases[] = {
		{ 0, 0, 0 }, { 0, 1, 1 }, { 2, 0, 0 }, { 2, 1, 1 },
		{ 1, 0, -EOPNOTSUPP }, { 1, 1, -EOPNOTSUPP },
		{ 3, 0, -EOPNOTSUPP }, { 4, 0, -EOPNOTSUPP },
		{ UINT_MAX, 0, -EOPNOTSUPP },
		{ 0, 2, -EOPNOTSUPP }, { 2, 2, -EOPNOTSUPP },
		{ 0, UINT_MAX, -EOPNOTSUPP },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		KUNIT_EXPECT_EQ(test,
			apple_dp_tunnel_t6030_pclk(cases[i].source, cases[i].dpin),
			cases[i].pclk);
		KUNIT_EXPECT_EQ(test,
			dcp_fabric_t6030_pclk(cases[i].source, cases[i].dpin),
			cases[i].pclk);
	}
}

static void fabric_t6030_clock_routes_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_t6030_init(&f);
	/* Reserve the hybrid processor for HDMI, even before HDMI arrives. */
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
	KUNIT_EXPECT_FALSE(test, f.pipeline[0].owned);
	f.pipeline[0].fixed_busy = true;
	KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	fabric_release(&f, 0, 0);
	/* HDMI first: the USB-C-only processor still drives the dock. */
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
	fabric_release(&f, 0, 0);
	f.pipeline[0].fixed_busy = false;
	/* A direct USB-C display can hold source 2; source 0 remains a fallback. */
	KUNIT_ASSERT_EQ(test, fabric_direct(&f, 1), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[1].owner[0], &f.route[1][1]);
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][0]);
	fabric_release(&f, 0, 0);
	f.pipeline[0].fixed_busy = true;
	KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0), -EBUSY);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], NULL);
	fabric_release(&f, 1, 0);
	f.pipeline[0].fixed_busy = false;
	/* A processor still starting does not displace the ready fallback. */
	f.pipeline[1].services_ready = false;
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][0]);
	fabric_release(&f, 0, 0);
	f.pipeline[0].services_ready = false;
	f.pipeline[1].services_ready = false;
	KUNIT_EXPECT_EQ(test, fabric_tunnel(&f, 0, 0), -EAGAIN);
	f.pipeline[1].services_ready = true;
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
	fabric_release(&f, 0, 0);
	/* Separate dock monitors hold separate processors and pixel clocks. */
	f.pipeline[0].services_ready = true;
	f.pipeline[1].services_ready = true;
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 0), 0);
	KUNIT_ASSERT_EQ(test, fabric_tunnel(&f, 0, 1), 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[0], &f.route[0][1]);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[1], &f.route[0][0]);
	fabric_release(&f, 0, 0);
	KUNIT_EXPECT_PTR_EQ(test, f.port[0].owner[1], &f.route[0][0]);
	fabric_release(&f, 0, 1);
	KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_clock_blocked(true, 1), (u8)3);
	KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_clock_blocked(true, 4), (u8)3);
	KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_clock_blocked(false, 0), (u8)0);
	KUNIT_EXPECT_EQ(test, dcp_fabric_tunnel_clock_blocked(false, 2), (u8)0);
}

static void fabric_t6030_plan_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_t6030_init(&f);
	dcp_fabric_plan(f.pipeline, f.port, &f.port[0], 0);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][0]);
	f.pipeline[0].fixed_busy = true;
	dcp_fabric_plan(f.pipeline, f.port, &f.port[0], 0);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[0], &f.route[0][1]);
	f.port[1].wanted = true;
	f.port[1].hpd = true;
	dcp_fabric_plan(f.pipeline, f.port, &f.port[0], 0);
	/* HDMI and the dock exhaust both external processors. */
	KUNIT_EXPECT_PTR_EQ(test, f.plan[1].target[0], NULL);
	dcp_fabric_plan(f.pipeline, f.port, &f.port[0], 1);
	KUNIT_EXPECT_PTR_EQ(test, f.plan[0].target[1], NULL);
}

static void fabric_t6030_follow_test(struct kunit *test)
{
	struct fabric_fixture f;

	fabric_t6030_init(&f);
	f.route[0][0].tunnel = true;
	f.route[0][0].dpin = 0;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][0], &f.route[0][1],
					      NULL, NULL, false, &f.policy), DCP_FABRIC_FOLLOW_MOVE);
	f.route[0][0].dpin = 1;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][0], &f.route[0][1],
					      NULL, NULL, false, &f.policy), DCP_FABRIC_FOLLOW_MOVE);
	f.route[0][0].tunnel = false;
	KUNIT_EXPECT_EQ(test, dcp_fabric_follow(&f.route[0][0], &f.route[0][1],
					      NULL, NULL, false, &f.policy), DCP_FABRIC_FOLLOW_MOVE);
}

static void fabric_binding_cookie_test(struct kunit *test)
{
	/* First admission may callback before attach returns/owner is published. */
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(0, 11, true, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_fabric_callback_valid(11, 11));
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(11, 11, true, true), 1);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(11, 11, true, false), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(11, 12, true, true), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(11, 10, false, true), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(11, 11, false, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(0, 11, false, true), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(0, 0, true, true), -EINVAL);
	KUNIT_EXPECT_FALSE(test, dcp_fabric_callback_valid(11, 0));
	KUNIT_EXPECT_FALSE(test, dcp_fabric_callback_valid(12, 11));
	/* Complete drain removes the lease; a reloaded TB allocator may start at 1. */
	KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(0, 1, true, false), 0);
}

static void fabric_reconnect_session_test(struct kunit *test)
{
	struct dcp_fabric_session queued = { .generation = 7, .cookie = 11 };
	struct dcp_fabric_session live_session = queued;

	KUNIT_EXPECT_TRUE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, false, false));
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, true, true));
	live_session.generation++;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	live_session = queued;
	live_session.cookie++;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	live_session.cookie = 0;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	/* Retained sleep keeps the TB cookie but invalidates the DCP connection. */
	live_session = queued;
	live_session.generation++;
	queued = live_session;
	KUNIT_EXPECT_TRUE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	/* Direct DP-alt uses cookie zero; local generation still guards queued work. */
	queued.cookie = 0;
	live_session.cookie = 0;
	KUNIT_EXPECT_TRUE(test, dcp_fabric_session_valid(queued, live_session, true, false));
	live_session.generation++;
	KUNIT_EXPECT_FALSE(test, dcp_fabric_session_valid(queued, live_session, true, false));
}

struct fabric_drain_fake {
	struct kunit *test;
	unsigned int step;
	u64 lease;
	u64 admitted;
	bool fabric_locked;
	bool reserved;
	bool cable;
	bool work_running;
};

static void fabric_drain_reserve(void *data)
{
	struct fabric_drain_fake *f = data;

	KUNIT_EXPECT_EQ(f->test, f->step++, 0U);
	KUNIT_EXPECT_TRUE(f->test, f->fabric_locked);
	f->reserved = true;
	f->admitted = 0;
}

static void fabric_drain_invalidate(void *data)
{
	struct fabric_drain_fake *f = data;

	KUNIT_EXPECT_EQ(f->test, f->step++, 1U);
	KUNIT_EXPECT_FALSE(f->test, dcp_fabric_callback_valid(f->lease, f->admitted));
	f->cable = false;
}

static void fabric_drain_unlock(void *data)
{
	struct fabric_drain_fake *f = data;

	KUNIT_EXPECT_EQ(f->test, f->step++, 2U);
	f->fabric_locked = false;
}

static void fabric_drain_wait(void *data)
{
	struct fabric_drain_fake *f = data;

	KUNIT_EXPECT_EQ(f->test, f->step++, 3U);
	KUNIT_EXPECT_FALSE(f->test, f->fabric_locked);
	KUNIT_EXPECT_TRUE(f->test, f->reserved);
	KUNIT_EXPECT_EQ(f->test, f->lease, 11ULL);
	KUNIT_EXPECT_FALSE(f->test, f->cable);
	KUNIT_EXPECT_TRUE(f->test, f->work_running);
	f->work_running = false;
}

static void fabric_drain_lock(void *data)
{
	struct fabric_drain_fake *f = data;

	KUNIT_EXPECT_EQ(f->test, f->step++, 4U);
	KUNIT_EXPECT_FALSE(f->test, f->work_running);
	f->fabric_locked = true;
}

static void fabric_binding_drain_test(struct kunit *test)
{
	static const struct dcp_fabric_drain_ops ops = {
		.reserve_revoke = fabric_drain_reserve,
		.invalidate = fabric_drain_invalidate,
		.unlock = fabric_drain_unlock,
		.drain = fabric_drain_wait,
		.lock = fabric_drain_lock,
	};
	unsigned int i;

	/* The shared executor drives both failed-attach and detach drain paths. */
	for (i = 0; i < 2; i++) {
		struct fabric_drain_fake f = {
			.test = test, .lease = 11, .admitted = 11,
			.fabric_locked = true, .cable = true, .work_running = true,
		};

		dcp_fabric_drain_binding(&ops, &f);
		KUNIT_EXPECT_EQ(test, f.step, 5U);
		KUNIT_EXPECT_FALSE(test, f.work_running);
		KUNIT_EXPECT_TRUE(test, f.fabric_locked);
		/* Only now may the caller release its lease/reservation and return. */
		f.lease = 0;
		f.reserved = false;
		KUNIT_EXPECT_EQ(test, dcp_fabric_binding_request(f.lease, 1, true, false), 0);
	}
}

static unsigned long relink_ms(unsigned long t, unsigned int ms)
{
	return t + msecs_to_jiffies(ms);
}

/* An enabled state for attachment @generation, whose link came up @t. */
static void relink_fresh(struct dcp_relink_state *s, u64 generation,
			 unsigned long t)
{
	*s = (struct dcp_relink_state){};
	dcp_relink_enable(s, true);
	dcp_relink_attach(s, generation);
	dcp_relink_link_up(s, generation, t);
}

/* DCP dropping a sink right after its link came up, and only then. */
static void relink_unplug_test(struct kunit *test)
{
	unsigned long t = 100000;
	struct dcp_relink_state s;

	relink_fresh(&s, 5, t);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, relink_ms(t, 2000)));
	KUNIT_EXPECT_EQ(test, s.tries, 1U);
	KUNIT_EXPECT_TRUE(test, s.active);

	/* a sink dropped long after its link came up is unplugged */
	relink_fresh(&s, 5, t);
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 5,
				relink_ms(t, DCP_RELINK_LINKUP_MS + 1)));
	/* also one of an attachment whose link-up was not seen */
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 6, relink_ms(t, 10)));
	KUNIT_EXPECT_FALSE(test, s.active);
	KUNIT_EXPECT_EQ(test, s.tries, 0U);

	/* a link that came up late, after a failed first connect */
	relink_fresh(&s, 5, t);
	dcp_relink_link_up(&s, 5, relink_ms(t, 30000));
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, relink_ms(t, 32000)));

	/* a reconnect under a new generation is no link-up of the attachment */
	relink_fresh(&s, 5, t);
	dcp_relink_link_up(&s, 6, relink_ms(t, 30000));
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 6, relink_ms(t, 32000)));
	dcp_relink_attach(&s, 7);
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 7, relink_ms(t, 32000)));

	/* a DPMS wake brings the link up again */
	dcp_relink_wake(&s, 7, relink_ms(t, 60000));
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 7, relink_ms(t, 62000)));
}

/* One relink at a time; the replay keeps the retries of its attachment. */
static void relink_running_test(struct kunit *test)
{
	unsigned long t = 100000;
	struct dcp_relink_state s;

	relink_fresh(&s, 5, t);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, t));
	/* another drop meanwhile is an unplug, and starts nothing */
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 5, t));
	KUNIT_EXPECT_EQ(test, s.tries, 1U);
	KUNIT_EXPECT_TRUE(test, s.active);

	/* its own unplug and replug change the generation */
	dcp_relink_reattach(&s, 7, relink_ms(t, 1000));
	dcp_relink_link_up(&s, 7, relink_ms(t, 1200));
	dcp_relink_end(&s);
	KUNIT_EXPECT_FALSE(test, s.active);
	KUNIT_EXPECT_EQ(test, s.tries, 1U);

	/* dropped again right after the relink: the next try */
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 7, relink_ms(t, 3200)));
	KUNIT_EXPECT_EQ(test, s.tries, 2U);
}

/* Three relinks per attachment or wake; a new attachment starts afresh. */
static void relink_budget_test(struct kunit *test)
{
	unsigned long t = 100000;
	struct dcp_relink_state s;
	unsigned int i;

	relink_fresh(&s, 5, t);
	for (i = 0; i < DCP_RELINK_RETRIES; i++) {
		KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, t));
		dcp_relink_reattach(&s, 5, t);
		dcp_relink_end(&s);
	}
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 5, t));
	KUNIT_EXPECT_EQ(test, s.tries, (u32)DCP_RELINK_RETRIES);

	dcp_relink_wake(&s, 5, t);
	KUNIT_EXPECT_EQ(test, s.tries, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, t));
	dcp_relink_end(&s);

	dcp_relink_attach(&s, 9);
	dcp_relink_link_up(&s, 9, t);
	KUNIT_EXPECT_EQ(test, s.tries, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 9, t));
	KUNIT_EXPECT_EQ(test, s.tries, 1U);
}

/* Disabled relinks are never queued, and a queued one is dropped. */
static void relink_disabled_test(struct kunit *test)
{
	struct dcp_relink_state s;

	relink_fresh(&s, 5, 1000);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, 1000));
	dcp_relink_enable(&s, false);
	KUNIT_EXPECT_FALSE(test, s.active);
	KUNIT_EXPECT_FALSE(test, dcp_relink_unplug(&s, 5, 1000));
	KUNIT_EXPECT_FALSE(test, s.active);
	dcp_relink_enable(&s, true);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 5, 1000));
}

/* A jiffies value of 0 still marks the link-up time. */
static void relink_stamp_test(struct kunit *test)
{
	struct dcp_relink_state s;

	relink_fresh(&s, 1, 0);
	KUNIT_EXPECT_NE(test, s.linked, 0UL);
	KUNIT_EXPECT_TRUE(test, dcp_relink_unplug(&s, 1, 3));
}

static struct kunit_case fabric_tests[] = {
	KUNIT_CASE(relink_unplug_test),
	KUNIT_CASE(relink_running_test),
	KUNIT_CASE(relink_budget_test),
	KUNIT_CASE(relink_disabled_test),
	KUNIT_CASE(relink_stamp_test),
	KUNIT_CASE(fabric_t6030_dual_policy_test),
	KUNIT_CASE(fabric_t6030_clock_routes_test),
	KUNIT_CASE(fabric_secondary_crtc_test),
	KUNIT_CASE(fabric_t6030_clock_header_parity_test),
	KUNIT_CASE(fabric_t6030_plan_test),
	KUNIT_CASE(fabric_t6030_follow_test),
	KUNIT_CASE(fabric_reclaim_transaction_test),
	KUNIT_CASE_PARAM(fabric_shared_wiring_test, fabric_wiring_gen_params),
	KUNIT_CASE(fabric_binding_cookie_test),
	KUNIT_CASE(fabric_reconnect_session_test),
	KUNIT_CASE(fabric_binding_drain_test),
	KUNIT_CASE(fabric_rebind_presence_test),
	KUNIT_CASE(fabric_nonhybrid_hdmi_resume_test),
	KUNIT_CASE(fabric_resume_no_connect_test),
	KUNIT_CASE(fabric_connector_flow_core_test),
	KUNIT_CASE(fabric_capacity_notification_test),
	KUNIT_CASE_PARAM(fabric_scenario_test, fabric_scenario_gen_params),
	KUNIT_CASE(fabric_dark_tunnel_test),
	KUNIT_CASE(fabric_effect_failure_core_test),
	KUNIT_CASE(fabric_deactivate_failure_test),
	KUNIT_CASE(fabric_unbound_and_mask_test),
	KUNIT_CASE(fabric_follow_test),
	KUNIT_CASE(fabric_follow_arrival_test),
	KUNIT_CASE(fabric_follow_recent_fixed),
	KUNIT_CASE(fabric_follow_effect_swap),
	KUNIT_CASE(fabric_follow_effect_oom),
	KUNIT_CASE(fabric_follow_effect_stale),
	KUNIT_CASE(fabric_follow_effect_rollback),
	KUNIT_CASE(fabric_follow_effect_lost),
	KUNIT_CASE(fabric_follow_effect_release_restore),
	KUNIT_CASE(fabric_presence_wrap_test),
	KUNIT_CASE(fabric_masked_edge_test),
	KUNIT_CASE_PARAM(fabric_wiring_test, fabric_wiring_gen_params),
	KUNIT_CASE(fabric_effect_s8_test),
	KUNIT_CASE(fabric_effect_s16_test),
	KUNIT_CASE(fabric_effect_s32_test),
	KUNIT_CASE(fabric_effect_s50a_test),
	KUNIT_CASE(fabric_effect_s50bc_test),
	KUNIT_CASE(fabric_effect_s50d_test),
	KUNIT_CASE(fabric_effect_s50e_test),
	KUNIT_CASE(fabric_effect_s50f_test),
	KUNIT_CASE(fabric_effect_failure_test),
	KUNIT_CASE(fabric_effect_deactivate_failure_test),
	{}
};

static struct kunit_suite fabric_suite = {
	.name = "apple-dcp-fabric",
	.test_cases = fabric_tests,
};

kunit_test_suite(fabric_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple DCP display fabric");
MODULE_LICENSE("Dual MIT/GPL");
