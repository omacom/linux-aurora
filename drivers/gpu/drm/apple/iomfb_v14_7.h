/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef __APPLE_IOMFB_V14_7_H__
#define __APPLE_IOMFB_V14_7_H__

#include <linux/types.h>

struct apple_dcp;
struct apple_dcp_v14;
struct dcp_v14_board;
struct device;
struct drm_atomic_state;
struct drm_crtc;
struct drm_crtc_state;

/*
 * The board of an internal 14.x DCP node (by compatible and machine): NULL if
 * the node is not one, ERR_PTR(-ENODEV) if this machine has no board record.
 */
const struct dcp_v14_board *iomfb_v14_7_board(struct device *dev);
const char *iomfb_v14_7_board_name(const struct dcp_v14_board *board);

/* Platform probe: firmware identity, boot loader handoff, PMP acknowledgment. */
int iomfb_v14_7_probe(struct apple_dcp *dcp);
/* Component bind: the running DCP's RTKit session. */
int iomfb_v14_7_bind(struct apple_dcp *dcp);
int iomfb_v14_7_external_start(struct apple_dcp *dcp);
/*
 * Native external processor: probe-time checks, then a firmware start on
 * first use. _ready() is true once DPTX can be used, and requests the start
 * if nothing has yet; _open() opens the display interface after the first
 * display is attached; _busy() says why system sleep must wait, or NULL.
 */
int iomfb_v14_7_external_prepare(struct apple_dcp *dcp);
bool iomfb_v14_7_external_ready(struct apple_dcp *dcp);
void iomfb_v14_7_external_refuse(struct apple_dcp *dcp, const char *why);
int iomfb_v14_7_external_open(struct apple_dcp *dcp);
const char *iomfb_v14_7_external_busy(struct apple_dcp *dcp);
/* The external session can no longer be used until reboot. */
bool iomfb_v14_7_external_failed(struct apple_dcp *dcp);
/* The firmware runs this mode, set last, and shows swaps in it. */
bool iomfb_v14_7_external_showing(struct apple_dcp *dcp,
				  const struct drm_display_mode *mode);
/*
 * The attached display came or went (_hotplug), or its timings were
 * withdrawn (_withdrawn); _release() reports a display that was kept
 * connected through a withdrawal, and is not described again, gone.
 */
void dcp_v14_external_hotplug(struct apple_dcp *dcp, bool connected);
void iomfb_v14_7_external_withdrawn(struct apple_dcp *dcp);
bool iomfb_v14_7_external_release(struct apple_dcp *dcp);
/* Component unbind: KMS goes away; the firmware session is kept until reboot. */
void iomfb_v14_7_unbind(struct apple_dcp *dcp);
/* Platform remove: the apple_dcp is freed next; the firmware session is kept. */
void iomfb_v14_7_remove(struct apple_dcp *dcp);
/* DCPLink, start signal, first client open and the panel mode. */
int iomfb_v14_7_start(struct apple_dcp *dcp);
/*
 * J613 internal pipe with a Type-C route (dcp->clamshell): _clamshell_start()
 * after the panel started; _set_lid() tells IOMFB the lid state and powers
 * the panel off with the lid closed, on again with it open if its CRTC is on.
 */
int iomfb_v14_7_clamshell_start(struct apple_dcp *dcp);
void iomfb_v14_7_set_lid(struct apple_dcp *dcp, bool closed);
/*
 * The lid is closed and IOMFB was told so (A475(2)), and the panel and its
 * CRTC are off: the pipe is free for a Type-C display.
 */
bool iomfb_v14_7_clamshell_panel_off(struct apple_dcp *dcp);
/* IOMFB was told the lid is open (A475(1)). */
bool iomfb_v14_7_lid_open(struct apple_dcp *dcp);
/*
 * The pipe drives the display on its Type-C route from _typec_attach() to
 * _typec_detach(), which gives the catalog back to the panel;
 * _typec_power_off() powers that display off (A472(0)).
 */
void iomfb_v14_7_typec_attach(struct apple_dcp *dcp);
void iomfb_v14_7_typec_power_off(struct apple_dcp *dcp);
void iomfb_v14_7_typec_detach(struct apple_dcp *dcp);

int iomfb_v14_7_atomic_check(struct apple_dcp *dcp, struct drm_crtc *crtc,
			     struct drm_atomic_state *state);
int iomfb_v14_7_modeset(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state);
void iomfb_v14_7_flush(struct apple_dcp *dcp, struct drm_crtc *crtc,
		       struct drm_atomic_state *state);
void iomfb_v14_7_poweron(struct apple_dcp *dcp);
void iomfb_v14_7_poweroff(struct apple_dcp *dcp);

#endif /* __APPLE_IOMFB_V14_7_H__ */
