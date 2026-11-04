// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_H__
#define __APPLE_DCP_H__

#include <drm/drm_atomic.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>

#include "connector.h"
#include "dcp-internal.h"
#include "dcp-fabric.h"
#include "parser.h"

bool dcp_h17p_external_supported(struct device_node *np);
bool dcp_h17p_external_ready(struct device_node *np);

struct apple_atomic_state {
	struct drm_atomic_commit base;
	/* Failed route transactions suppress both CRTC enables in this commit. */
	u32 failed_routes;
};

#define to_apple_atomic_state(s) container_of(s, struct apple_atomic_state, base)
struct apple_crtc {
	struct drm_crtc base;
	struct drm_pending_vblank_event *event;
	bool vsync_disabled;

	/* Reference to the DCP device owning this CRTC */
	struct platform_device *dcp;

	/* External scanout keeps the fabric up while the CRTC is active. */
	bool fabric_held;
};

#define to_apple_crtc(x) container_of(x, struct apple_crtc, base)

/*
 * The pmgr-misc fabric hold (linux/soc/apple/pmgr-misc.h, 010-soc). Declared
 * here until that header is in this tree; apple_drv.c resolves both with
 * symbol_get() and behaves as their !CONFIG_APPLE_PMGR_MISC stubs without a
 * provider.
 */
#ifndef __LINUX_SOC_APPLE_PMGR_MISC_H
int apple_pmgr_fabric_hold(void);
void apple_pmgr_fabric_release(void);
#endif

struct apple_encoder {
	struct drm_encoder base;
};

#define to_apple_encoder(x) container_of(x, struct apple_encoder, base)

void dcp_poweroff(struct platform_device *pdev);
int dcp_typec_prepare_route(struct apple_dcp *dcp);
void dcp_poweron(struct platform_device *pdev);
void dcp_external_recover(struct platform_device *pdev);
int dcp_set_crc(struct drm_crtc *crtc, bool enabled);
int dcp_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_commit *commit);
int dcp_get_connector_type(struct platform_device *pdev);
bool dcp_fw_compat_is_12_x(struct platform_device *pdev);
bool dcp_fw_compat_is_14_7(struct platform_device *pdev);
bool dcp_ext_unusable(struct platform_device *pdev);
unsigned long* dcp_get_iomfb_surfaces(struct platform_device *pdev);
int dcp_start(struct platform_device *pdev);
int dcp_wait_ready(struct platform_device *pdev, u64 timeout);
void dcp_flush(struct drm_crtc *crtc, struct drm_atomic_commit *commit);
void dcp_retain_framebuffer(struct platform_device *pdev,
			    struct dcp_fb_reference *entry);
void dcp_arm_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id);
void dcp_unarm_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id);
void dcp_fb_mirror(struct apple_dcp *dcp, struct drm_framebuffer *fb);
void dcp_release_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id);
void dcp_release_all_retained_framebuffers(struct apple_dcp *dcp);
bool dcp_is_initialized(struct platform_device *pdev);
void apple_crtc_vblank(struct apple_crtc *apple);
void dcp_drm_crtc_vblank(struct apple_crtc *crtc);
int dcp_get_modes(struct drm_connector *connector);
enum drm_mode_status dcp_mode_valid(struct drm_connector *connector,
				    const struct drm_display_mode *mode);
int dcp_crtc_atomic_modeset(struct drm_crtc *crtc,
			    struct drm_atomic_commit *commit);
bool dcp_crtc_can_retrain(struct drm_crtc *crtc, struct apple_connector *connector);
bool dcp_crtc_needs_route_start(struct apple_dcp *dcp);
bool dcp_crtc_route_ready(struct drm_crtc *crtc, struct drm_atomic_commit *commit,
			  bool fresh);
bool dcp_has_mode(struct apple_dcp *dcp, const struct drm_display_mode *mode);
bool dcp_crtc_mode_fixup(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);
void dcp_set_dimensions(struct apple_dcp *dcp, u64 generation);
void dcp_update_connector_dimensions(struct apple_dcp *dcp,
				     struct drm_connector *connector);
void dcp_send_message(struct apple_dcp *dcp, u8 endpoint, u64 message);

int dcp_dptx_connect_oob(struct platform_device *pdev, u32 port);
bool dcp_t6030_ext_native(const struct device_node *np);
void dcp_external_ready(struct apple_dcp *dcp);
void dcp_external_retry(struct apple_dcp *dcp, const char *why, int error,
			unsigned int base_ms);
void dcp_external_retry_work(struct work_struct *work);
void dcp_external_sink_irq(struct apple_dcp *dcp);
void dcp_queue_typec_reconnect(struct apple_dcp *dcp, unsigned long delay);
int dcp_dptx_recover_irq(struct apple_dcp *dcp);
int dcp_dptx_park(struct apple_dcp *dcp);
int dcp_dptx_disconnect_oob(struct platform_device *pdev, u32 port);
bool dcp_hold_resume_event(struct platform_device *pdev);

int iomfb_start_rtkit(struct apple_dcp *dcp);
void iomfb_shutdown(struct apple_dcp *dcp);
/* rtkit message handler for IOMFB messages */
void iomfb_recv_msg(struct apple_dcp *dcp, u64 message);

int systemep_init(struct apple_dcp *dcp);
int dptxep_init(struct apple_dcp *dcp);
int dptx_controller_init(struct apple_dcp *dcp);
int ibootep_init(struct apple_dcp *dcp);
int dpavservep_init(struct apple_dcp *dcp);
int avep_init(struct apple_dcp *dcp);


int __init dcp_register(void);
void dcp_unregister(void);

int __init dcp_audio_register(void);
void dcp_audio_unregister(void);

int apple_j613_25g83_coldplug(void);
unsigned int apple_j613_25g83_clock_hz(void);
/* A J613, or a J615 whose boot loader carries the experimental opt-in. */
bool apple_t8122_25g83_board(void);

#endif
