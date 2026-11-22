/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __APPLE_DCP_FABRIC_H__
#define __APPLE_DCP_FABRIC_H__

#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/types.h>

#include "dcp-fabric-core.h"

struct apple_connector;
struct apple_crtc;
struct drm_atomic_commit;
struct drm_crtc;
struct drm_display_mode;
struct device_node;
struct drm_device;
struct mux_control;
struct platform_device;
struct phy;
struct typec_mux_dev;
struct work_struct;

#define DCP_MAX_TYPEC_ROUTES 4

struct apple_dcp;
struct apple_dcp_typec_port;

struct apple_dcp_typec_route {
	struct dcp_fabric_route core;
	struct apple_dcp *dcp;
	struct apple_dcp_typec_port *port;
	/* T6050: this port/pipeline pair has a connector with a fixed CRTC. */
	struct apple_connector *connector;
	struct list_head port_link;
	struct phy *phy;
	struct mux_control *xbar;
	struct mux_control *dpin[DCP_FABRIC_DPIN_MAX];
	bool dual_stream;
	/* apple,connector-group: its connector's order group */
	u32 connector_group;
	struct typec_mux_dev *typec_mux;
	u32 dptx_phy;
	u32 mux_index;
	bool selected;
	/* desktop 26.x: the DP PHY is initialised and powered for this route */
	bool phy_powered;
	/* crossbar output actually selected: xbar (dpphy) or a Thunderbolt dpin */
	unsigned int tunnel_dpin;
	/* T6050: DP link rate code the tunnel's pixel clock runs at, 0 if stopped */
	u8 tunnel_rate;
	u64 tunnel_generation;
	struct mux_control *active_xbar;
	bool tunnel;
	/* tunnel: crossbar brought up (at DidChangeLinkConfiguration) */
	bool xbar_up;
	/* Lower mux release is consumed even when idle programming fails. */
	bool deactivate_xbar_done;
	int deactivate_error;
	/* Its crossbar output is down for a sleep, and not held. */
	bool sleep_parked;
};

int dcp_typec_phy_on(struct apple_dcp *dcp);
bool dcp_is_typec_output(struct apple_dcp *dcp);
bool dcp_is_usb4_output(struct apple_dcp *dcp);
bool dcp_uses_t6020_tunnel_flow(struct apple_dcp *dcp);
/* The fixed HDMI output, not lent to a Type-C display, has HPD high. */
bool dcp_hdmi_port_attached(struct apple_dcp *dcp);

bool dcp_has_typec_routes(struct platform_device *pdev);

/*
 * The Type-C display fabric.  Ports are enumerated in device-tree order, not
 * DCP probe order, so a given physical port keeps the same DRM connector index
 * across boots -- userspace keys its per-monitor configuration on that name.
 */
unsigned int dcp_typec_nr_ports(void);
struct device_node *dcp_typec_port_of_node(unsigned int idx);
bool dcp_typec_port_has_candidate(unsigned int idx, struct platform_device *pdev);
/* Connectors by apple,connector-group; T6050's tunnel-only pairs after the others in each */
#define DCP_TYPEC_CONNECTOR_GROUPS 4
unsigned int dcp_typec_nr_passes(void);
unsigned int dcp_typec_port_pass(unsigned int idx, struct platform_device *pdev);
bool dcp_typec_port_unusable(unsigned int idx);
void dcp_typec_port_set_connector(unsigned int idx, bool secondary,
				  struct apple_connector *connector,
				  struct platform_device *pipeline);
bool dcp_typec_fixed_crtcs(void);
bool dcp_typec_dual_stream(void);
bool dcp_typec_ordered_dual_stream(void);
void dcp_typec_reorder(void);
bool dcp_typec_follows_crtc(struct apple_dcp *dcp);
bool dcp_typec_follow_holder_off(struct drm_atomic_commit *state, struct drm_crtc *crtc,
				 struct apple_connector *connector, struct apple_dcp *back);
int dcp_typec_follow_check(struct apple_dcp *dcp, struct drm_crtc *crtc,
			   struct drm_atomic_commit *state,
			   const struct drm_display_mode *mode);
int dcp_typec_follow_crtc(struct drm_crtc *crtc, struct drm_atomic_commit *state,
			  bool *locked);
void dcp_typec_follow_done(bool locked);
bool dcp_is_typec_only(struct platform_device *pdev);
void dcp_link(struct platform_device *pdev, struct apple_crtc *apple,
	      struct apple_connector *connector);
void dcp_unlink(struct drm_device *drm);

/* Thunderbolt DP tunnels, called from the DPTX endpoint */
int dcp_tunnel_crossbar_up(struct apple_dcp *dcp, u32 unit);
int dcp_tunnel_crossbar_down(struct apple_dcp *dcp, u32 unit);
int dcp_direct_crossbar_link(struct apple_dcp *dcp, bool up);
int dcp_dpxbar_link(struct mux_control *mux, bool up);
/*
 * T6030 ATC PHY: keep the source owning the pixel clock across rate changes
 * and stop (linux/soc/apple/dp-tunnel.h, 170-usb). Declared here until that
 * header has it; called through symbol_get() only.
 */
int apple_atc_dp_tunnel_rate_source(struct phy *phy, unsigned int source,
				    unsigned int dpin, u8 rate);
int dcp_tunnel_set_rate(struct apple_dcp *dcp, struct phy *phy, u32 unit,
			u32 link_rate);
int dcp_tunnel_dpin_activate(struct apple_dcp *dcp, u32 unit, bool active);

/* An adopted T8132 dcpext. */
bool dcp_is_t8132_ext(const struct apple_dcp *dcp);

/* DCP probe, resume and endpoint teardown hooks. */
int dcp_register_typec_routes(struct apple_dcp *dcp);
void dcp_typec_route_park(struct apple_dcp *dcp);
void dcp_typec_route_unpark(struct apple_dcp *dcp);
int dcp_fabric_sleep_release(struct apple_dcp *dcp, s64 ms[2]);
int dcp_fabric_sleep_withdraw(struct apple_dcp *dcp, s64 ms[2]);
void dcp_fabric_sleep_done(struct apple_dcp *dcp);
void dcp_typec_routes_disable(struct apple_dcp *dcp);
void dcp_typec_routes_disable_dev(struct device *dev);
const char *dcp_typec_route_supplier_disabled(struct device *dev, const char *name);
void dcp_typec_retrain_work(struct work_struct *work);
void dcp_fabric_init(struct apple_dcp *dcp);
void dcp_fabric_hdmi_resume(struct apple_dcp *dcp);
void dcp_fabric_hdmi_reinit(struct apple_dcp *dcp, const char *why);
void dcp_fabric_hdmi_retry(struct apple_dcp *dcp);
void dcp_dp2hdmi_record_edge(struct apple_dcp *dcp);
irqreturn_t dcp_dp2hdmi_hpd_edge(int irq, void *data);
irqreturn_t dcp_dp2hdmi_hpd(int irq, void *data);
void dcp_fabric_shutdown_dptx(struct apple_dcp *dcp);

/* DPTX operations supplied by the DCP firmware layer. */
int dcp_dptx_connect(struct apple_dcp *dcp, u32 port);
int dcp_dptx_disconnect(struct apple_dcp *dcp, u32 port);
bool dcp_is_external_h17p(struct apple_dcp *dcp);
bool dcp_is_retained_external(struct apple_dcp *dcp);

int dcp_typec_test_link(struct platform_device *pdev, bool enabled);

#endif /* __APPLE_DCP_FABRIC_H__ */
