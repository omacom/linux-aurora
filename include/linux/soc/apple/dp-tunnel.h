/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * DisplayPort over Thunderbolt on Apple silicon: the pieces that bring up a
 * DP tunnel live in the Thunderbolt glue, appledrm, the ATC PHY and the
 * display crossbar. They find each other with symbol_get(), so none of these
 * modules has to be built or loaded for the others to work.
 */
#ifndef _LINUX_SOC_APPLE_DP_TUNNEL_H_
#define _LINUX_SOC_APPLE_DP_TUNNEL_H_

#include <linux/kernel.h>
#include <linux/of_graph.h>
#include <linux/of.h>
#include <linux/types.h>

struct device_node;
struct mux_control;
struct phy;

/*
 * appledrm: route a display pipeline to (active) or away from (!active) the
 * crossbar output of DP IN adapter @dpin (0/1) of the router wired to
 * @connector_np. @set_active(@binding, active) runs the DP IN adapter's
 * DPTX_INACTIVE handshake and is called back from DCP's link activation.
 * Activation returns -EBUSY while no pipeline is free for the stream (or a
 * direct DP-alt route still holds the port), which may change and is worth
 * asking again, and -EADDRINUSE if another tunnel stream holds the port.
 * @generation is a nonzero TB-provider-lifetime handoff identity, unchanged
 * by retries. TB admits the immutable @binding before this call: synchronous
 * Activate may run before attach returns success. A matching active request
 * is idempotent; a mismatched live cookie/binding returns -ESTALE before
 * mutation. Detach and failed attach retire/drain callbacks and reconnect
 * before returning, so TB may then free the binding. No provider tombstone
 * survives complete drain: TB rejects stale queued requests in its context
 * and drains every binding before module unload/counter reset.
 */
int apple_dcp_tb_dp_tunnel(struct device_node *connector_np, unsigned int dpin,
			   u64 generation, bool active,
			   int (*set_active)(void *binding, bool active), void *binding);

/*
 * ATC PHY: start the pixel clock for a DP tunnel at DP link rate code @rate,
 * or stop it (@rate == 0). The PHY must be in Thunderbolt/USB4 mode.
 */
int apple_atc_dp_tunnel_rate(struct phy *phy, unsigned int dpin, u8 rate);

/* T6030's external DPTX port 0 sources feed DP IN0/1 through PCLK1/2. */
static inline int apple_dp_tunnel_t6030_pclk(unsigned int source, unsigned int dpin)
{
	return dpin < 2 && (source == 0 || source == 2) ? (int)dpin : -EOPNOTSUPP;
}

/* T6030: keep the source owning the pixel clock across rate changes and stop. */
int apple_atc_dp_tunnel_rate_source(struct phy *phy, unsigned int source,
				    unsigned int dpin, u8 rate);

/*
 * ATC PHY: wake the DP clock path of a PHY in Thunderbolt/USB4 mode ahead of
 * apple_atc_dp_tunnel_rate(), before DCP is told about the display.
 * -EOPNOTSUPP where the rate call does all of it (everything but t600x).
 */
int apple_atc_dp_tunnel_open(struct phy *phy);

/*
 * Display crossbar: take the connection of an already selected output down or
 * bring it back up (clock gates and enables) around a link reconfiguration.
 * The mux selection is left alone; link_down() also leaves the ATC output
 * enable set and link_up() re-asserts it. The caller keeps the output
 * selected (holds the mux) around both. Supports t8103 and T602X crossbars.
 */
int apple_dpxbar_link_down(struct mux_control *mux);
int apple_dpxbar_link_up(struct mux_control *mux);
int apple_dpxbar_tunnel_select_source(struct mux_control *mux, int state);

/*
 * Display crossbar: point a DP IN output that is not selected yet at source
 * @state without bringing the connection up, or back at its idle source
 * (MUX_IDLE_DISCONNECT). The later mux selection does the rest.
 * -EOPNOTSUPP on T602X crossbars.
 */
int apple_dpxbar_preselect(struct mux_control *mux, int state);

/* Shared with the pure fabric classifier; a partial named pair is invalid. */
static inline bool apple_dp_tunnel_wiring_dual(unsigned int endpoints,
					       bool dpin0, bool dpin1, bool legacy)
{
	return endpoints >= 2 && ((dpin0 && dpin1) || (!dpin0 && !dpin1 && legacy));
}

static inline bool apple_dp_tunnel_mux_matches(struct device_node *dcp,
					       const char *name,
					       struct device_node *xbar, unsigned int cell)
{
	struct of_phandle_args args;
	bool matches;
	int index;

	index = of_property_match_string(dcp, "mux-control-names", name);
	if (index < 0 || of_parse_phandle_with_args(dcp, "mux-controls",
						    "#mux-control-cells", index, &args))
		return false;
	matches = args.np == xbar && args.args_count == 1 && args.args[0] == cell;
	of_node_put(args.np);
	return matches;
}

/* T6030 boot trees without DPIN names use the same three-output crossbar.
 * Explicit DPIN/USB4 names remain authoritative, including invalid wiring. */
static inline bool apple_dp_tunnel_t6030_legacy_mux(struct device_node *dcp,
						 unsigned int port)
{
	struct of_phandle_args base;
	char name[24];
	u32 source;
	bool matches;
	int index, i;

	if (!of_machine_is_compatible("apple,t6030") || port >= 4 ||
	    of_property_read_u32_index(dcp, "apple,typec-mux-indices", port, &source) ||
	    apple_dp_tunnel_t6030_pclk(source, 0) < 0)
		return false;
	for (i = 0; i < 2; i++) {
		snprintf(name, sizeof(name), "typec%u-dpin%u", port, i);
		if (of_property_match_string(dcp, "mux-control-names", name) >= 0)
			return false;
	}
	snprintf(name, sizeof(name), "typec%u-usb4", port);
	if (of_property_match_string(dcp, "mux-control-names", name) >= 0)
		return false;
	snprintf(name, sizeof(name), "typec%u", port);
	index = of_property_match_string(dcp, "mux-control-names", name);
	if (index < 0 || of_parse_phandle_with_args(dcp, "mux-controls",
						"#mux-control-cells", index, &base))
		return false;
	matches = base.args_count == 1 && base.args[0] == 0 &&
		  of_device_is_compatible(base.np, "apple,t6020-display-crossbar");
	of_node_put(base.np);
	return matches;
}

static inline bool apple_dp_tunnel_route_wired(struct device_node *route,
					       unsigned int endpoints)
{
	struct device_node *routes __free(device_node) = of_get_parent(route);
	struct device_node *dcp __free(device_node) = of_get_parent(routes);
	struct of_phandle_args base;
	char name[24];
	bool named[2], wired, legacy = false;
	u32 index;
	int mux, i;

	if (!of_node_name_eq(routes, "typec-routes") || !dcp ||
	    of_property_read_u32(route, "reg", &index) || index >= 4)
		return false;
	snprintf(name, sizeof(name), "typec%u", index);
	mux = of_property_match_string(dcp, "mux-control-names", name);
	if (mux < 0 || of_parse_phandle_with_args(dcp, "mux-controls",
						  "#mux-control-cells", mux, &base))
		return false;
	wired = base.args_count == 1 && base.args[0] == 0;
	for (i = 0; i < 2; i++) {
		snprintf(name, sizeof(name), "typec%u-dpin%u", index, i);
		named[i] = of_property_match_string(dcp, "mux-control-names", name) >= 0;
		if (named[i])
			wired &= apple_dp_tunnel_mux_matches(dcp, name, base.np, 1 + i);
	}
	if (!named[0] && !named[1]) {
		snprintf(name, sizeof(name), "typec%u-usb4", index);
		legacy = apple_dp_tunnel_mux_matches(dcp, name, base.np, 1) ||
			 apple_dp_tunnel_t6030_legacy_mux(dcp, index);
	}
	of_node_put(base.np);
	return wired && apple_dp_tunnel_wiring_dual(endpoints, named[0], named[1], legacy);
}

/*
 * DT wiring only: no DCP registration, firmware or display status.
 * Two candidates alone (e.g. j314) are single-stream. Named DPIN0/1 controls,
 * or old-DTB typecN-usb4 cell 1, must share the DP-alt crossbar (cell 0).
 * T6030 old trees may omit DPIN names on their three-output T602X crossbar.
 * Consumers qualify their SoC flow separately and never select these muxes.
 */
/*
 * SoCs whose DP IN tunnels run the T6030 (M3) flow: T8122-generation ATC PHYs
 * clocking both DP IN outputs, and a crossbar that routes every output itself.
 * T6034 has the same ATC generation.
 */
static inline bool apple_dp_tunnel_m3_soc(void)
{
	return of_machine_is_compatible("apple,t6030") ||
	       of_machine_is_compatible("apple,t6034");
}

static inline bool apple_dp_tunnel_dual_stream(struct device_node *connector_np)
{
	struct device_node *port __free(device_node) = NULL;
	unsigned int endpoints = 0;

	if (!connector_np)
		return false;
	port = of_graph_get_port_by_id(connector_np, 3);
	for_each_child_of_node_scoped(port, endpoint)
		endpoints++;
	if (endpoints < 2)
		return false;
	for_each_child_of_node_scoped(port, endpoint) {
		struct device_node *route __free(device_node) =
			of_graph_get_remote_port_parent(endpoint);

		if (route && apple_dp_tunnel_route_wired(route, endpoints))
			return true;
	}
	return false;
}


#endif
