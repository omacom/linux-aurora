// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device/driver.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/j613-display.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/string.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "afk.h"
#include "av.h"
#include "dcp.h"
#include "dcp-fabric-effects.h"
#include "dcp-fabric.h"
#include "dcp-hdmi.h"
#include "dcpext_scanout.h"
#include "dcp-internal.h"
#include "dcp-external-bw.h"
#include "dptx-service.h"
#include "iomfb.h"
#include "ibootep.h"
#include "parser.h"
#include "trace.h"

#define APPLE_DCP_COPROC_CPU_CONTROL	 0x44
#define APPLE_DCP_COPROC_CPU_CONTROL_RUN BIT(4)
#define APPLE_DCP_COPROC_CPU_STATUS 0x48
#define APPLE_DCP_COPROC_CPU_STATUS_STOPPED BIT(1)
#define APPLE_DCP_COPROC_CPU_STATUS_IDLE BIT(5)

#define DCP_BOOT_TIMEOUT msecs_to_jiffies(1000)

static bool show_notch;
module_param(show_notch, bool, 0644);
MODULE_PARM_DESC(show_notch, "Use the full display height and shows the notch");

static bool t6030_show_notch;
module_param(t6030_show_notch, bool, 0444);
MODULE_PARM_DESC(t6030_show_notch,
		 "M3 internal display (14.x and 26.6 IOMFB): use the full height and show the notch");

bool hdmi_audio;
module_param(hdmi_audio, bool, 0644);
MODULE_PARM_DESC(hdmi_audio, "Enable unstable HDMI audio support");

static bool unstable_edid = true;
module_param(unstable_edid, bool, 0644);
MODULE_PARM_DESC(unstable_edid, "Enable unstable EDID retrival support");

/*
 * H17-generation firmware (T8140 H17P, T8142 H17G) runs aurora-wip's
 * processor lifecycle, below in the H17P section. Shared entry points
 * hand those processors over to it.
 */
static void dcp_recv_msg_h17p(struct apple_dcp *dcp, u8 endpoint, u64 message);
static void dcp_rtk_crashed_h17p(struct apple_dcp *dcp);
static int dcp_rtk_shmem_setup_h17p(struct apple_dcp *dcp, struct apple_rtkit_shmem *bfr);
static int dcp_crtc_atomic_check_h17p(struct drm_crtc *crtc, struct drm_atomic_state *state);
static int dcp_dptx_connect_h17p(struct apple_dcp *dcp, u32 port);
static int dcp_dptx_disconnect_h17p(struct apple_dcp *dcp, u32 port);
static int dcp_dptx_connect_oob_h17p(struct platform_device *pdev, u32 port);
static int dcp_dptx_disconnect_oob_h17p(struct platform_device *pdev, u32 port);
static int dcp_start_h17p(struct platform_device *pdev);
static void dcp_poweron_h17p(struct platform_device *pdev);
static void dcp_poweroff_h17p(struct platform_device *pdev);
static void dcp_disable_typec_work_h17p(struct apple_dcp *dcp, bool release_cable);
static void dcp_enable_typec_work_h17p(struct apple_dcp *dcp);
static int dcp_comp_bind_h17p(struct device *dev, struct device *main, void *data);
static void dcp_comp_unbind_h17p(struct device *dev, struct device *main, void *data);
static int dcp_platform_probe_h17p(struct platform_device *pdev);
static void dcp_platform_shutdown_h17p(struct platform_device *pdev);
static int dcp_platform_suspend_h17p(struct device *dev);
static int dcp_platform_resume_h17p(struct device *dev);

/* copied and simplified from drm_vblank.c */
static void send_vblank_event(struct drm_device *dev,
		struct drm_pending_vblank_event *e,
		u64 seq, ktime_t now)
{
	struct timespec64 tv;

	if (e->event.base.type != DRM_EVENT_FLIP_COMPLETE)
		return;

	tv = ktime_to_timespec64(now);
	e->event.vbl.sequence = seq;
	/*
		* e->event is a user space structure, with hardcoded unsigned
		* 32-bit seconds/microseconds. This is safe as we always use
		* monotonic timestamps since linux-4.15
		*/
	e->event.vbl.tv_sec = tv.tv_sec;
	e->event.vbl.tv_usec = tv.tv_nsec / 1000;

	/*
	 * Use the same timestamp for any associated fence signal to avoid
	 * mismatch in timestamps for vsync & fence events triggered by the
	 * same HW event. Frameworks like SurfaceFlinger in Android expects the
	 * retire-fence timestamp to match exactly with HW vsync as it uses it
	 * for its software vsync modeling.
	 */
	drm_send_event_timestamp_locked(dev, &e->base, now);
}

/**
 * dcp_crtc_send_page_flip_event - helper to send vblank event after pageflip
 *
 * Compensate for unknown slack between page flip and arrival of the
 * swap_complete callback. Minimal observed duration on DCP with HDMI output
 * was around 2.3 ms. If the fb swap was submitted closer to the expected
 * swap_complete it gets a penalty of one frame duration. This is on the border
 * of unreasonable considering that Apple advertises support for 240 Hz (frame
 * duration of 4.167 ms).
 * It is unreasonable considering kwin's kms commit scheduling. Kwin commits
 * 1.5 ms + the mode's vblank time before the expected next page flip
 * completion. This results in presenting at half the display's rate for HDMI
 * outputs.
 * This might be a difference between dcp and dcpext.
 */
static void dcp_crtc_send_page_flip_event(struct apple_crtc *crtc,
					  struct drm_pending_vblank_event *e,
					  ktime_t now, ktime_t start)
{
	struct drm_device *dev = crtc->base.dev;
	u64 seq;
	unsigned int pipe = drm_crtc_index(&crtc->base);
	ktime_t flip;

	seq = 0;
	if (start != KTIME_MIN) {
		s64 delta = ktime_us_delta(now, start);
		if (delta <= 500)
			flip = now;
		else if (delta >= 2500)
			flip = ktime_sub_us(now, 1000);
		else
			flip = ktime_sub_us(now, (delta - 500) / 2);
	} else {
		flip = now;
	}
	e->pipe = pipe;
	send_vblank_event(dev, e, seq, flip);
}

/* HACK: moved here to avoid circular dependency between apple_drv and dcp */
void dcp_drm_crtc_vblank(struct apple_crtc *crtc)
{
	unsigned long flags;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	if (crtc->event) {
		drm_crtc_send_vblank_event(&crtc->base, crtc->event);
		crtc->event = NULL;
	}
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
}

void dcp_drm_crtc_page_flip(struct apple_dcp *dcp, ktime_t now)
{
	unsigned long flags;
	struct apple_crtc *crtc = dcp->crtc;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	if (crtc->event) {
		if (crtc->event->event.base.type == DRM_EVENT_FLIP_COMPLETE)
			dcp_crtc_send_page_flip_event(crtc, crtc->event, now, dcp->swap_start);
		else
			drm_crtc_send_vblank_event(&crtc->base, crtc->event);
		crtc->event = NULL;
		dcp->swap_start = KTIME_MIN;
	}
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
}

void dcp_set_dimensions(struct apple_dcp *dcp, u64 generation)
{
	struct apple_connector *apple_connector = READ_ONCE(dcp->connector);
	struct drm_connector *connector = apple_connector ? &apple_connector->base : NULL;
	int width_mm, height_mm, i;

	if (connector)
		mutex_lock(&connector->dev->mode_config.mutex);
	mutex_lock(&dcp->modes_lock);
	if (generation != dcp->modes_generation ||
	    apple_connector != READ_ONCE(dcp->connector))
		goto out_unlock;
	width_mm = dcp->width_mm;
	height_mm = dcp->height_mm;
	if (!width_mm || !height_mm) {
		width_mm = dcp->panel.width_mm;
		height_mm = dcp->panel.height_mm;
	}
	if (connector) {
		connector->display_info.width_mm = width_mm;
		connector->display_info.height_mm = height_mm;
	}
	for (i = 0; i < dcp->nr_modes; ++i) {
		dcp->modes[i].mode.width_mm = width_mm;
		dcp->modes[i].mode.height_mm = height_mm;
	}
out_unlock:
	mutex_unlock(&dcp->modes_lock);
	if (connector)
		mutex_unlock(&connector->dev->mode_config.mutex);
}

bool dcp_has_panel(struct apple_dcp *dcp)
{
	return dcp->panel.width_mm > 0;
}

int dcp_set_crc(struct drm_crtc *crtc, bool enabled)
{
	struct apple_crtc *ac = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(ac->dcp);

	dcp->crc_enabled = enabled;

	return 0;
}

/*
 * Helper to send a DRM vblank event. We do not know how call swap_submit_dcp
 * without surfaces. To avoid timeouts in drm_atomic_helper_wait_for_vblanks
 * send a vblank event via a workqueue.
 */
static void dcp_delayed_vblank(struct work_struct *work)
{
	struct apple_dcp *dcp;

	struct apple_crtc *crtc;

	dcp = container_of(work, struct apple_dcp, vblank_wq);
	/* RTKit can report a crash before the pipeline has a CRTC. */
	crtc = READ_ONCE(dcp->crtc);
	if (!crtc)
		return;
	mdelay(5);
	dcp_drm_crtc_vblank(crtc);
}

#define DCP_SWAP_WATCHDOG_MS		1000
#define DCP_SWAP_WATCHDOG_RETRAINS	5

/*
 * DCP drops swaps without completing them while an external pipe is not
 * enabled, for instance after a modeset that raced a Type-C sink which had
 * not asserted HPD yet. Userspace would then wait for the flip until the
 * commit times out, stalling every output it drives. Complete the flip,
 * mark the mode invalid so later commits do not wait for DCP, and let the
 * hotplug worker re-apply the active CRTC.
 */
static void dcp_swap_watchdog(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     swap_watchdog_wq);

	dev_warn(dcp->dev, "swap not completed, retraining the display\n");
	dcp_mode_invalidate(&dcp->mode_state);
	dcp_drm_crtc_vblank(dcp->crtc);
	if (dcp->connector &&
	    dcp->swap_watchdog_retrains++ < DCP_SWAP_WATCHDOG_RETRAINS)
		dcp_queue_hotplug(dcp->connector);
}

void dcp_swap_watchdog_arm(struct apple_dcp *dcp)
{
	if (dcp_is_typec_output(dcp))
		mod_delayed_work(system_wq, &dcp->swap_watchdog_wq,
				 msecs_to_jiffies(DCP_SWAP_WATCHDOG_MS));
}

void dcp_swap_watchdog_complete(struct apple_dcp *dcp)
{
	cancel_delayed_work(&dcp->swap_watchdog_wq);
	dcp->swap_watchdog_retrains = 0;
}

static void dcp_recv_msg(void *cookie, u8 endpoint, u64 message)
{
	struct apple_dcp *dcp = cookie;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_recv_msg_h17p(dcp, endpoint, message);

	trace_dcp_recv_msg(dcp, endpoint, message);

	/* A failed T8122 dcpext: its firmware may still talk, nobody listens. */
	if (READ_ONCE(dcp->ext_dead))
		return;

	switch (endpoint) {
	case IOMFB_ENDPOINT:
		return iomfb_recv_msg(dcp, message);
	case AV_ENDPOINT:
		afk_receive_message(dcp->avep, message);
		return;
	case SYSTEM_ENDPOINT:
		afk_receive_message(dcp->systemep, message);
		return;
	case DISP0_ENDPOINT:
		afk_receive_message(dcp->ibootep, message);
		return;
	case DPAVSERV_ENDPOINT:
		afk_receive_message(dcp->dcpavservep, message);
		return;
	case DPTX_ENDPOINT:
		afk_receive_message(dcp->dptxep, message);
		return;
	default:
		WARN(endpoint, "unknown DCP endpoint %hhu\n", endpoint);
	}
}

static void dcp_rtk_crashed(void *cookie, const void *crashlog, size_t crashlog_size)
{
	struct apple_dcp *dcp = cookie;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_rtk_crashed_h17p(dcp);

	/*
	 * A failed T8122 dcpext: its start never ran (start_done is not set
	 * up) and it has no connector. Its firmware may still report a crash.
	 */
	if (READ_ONCE(dcp->ext_dead)) {
		dcp->crashed = true;
		dev_err(dcp->dev, "unusable external display processor has crashed\n");
		return;
	}
	dcp->crashed = true;
	dev_err(dcp->dev, "DCP has crashed\n");
	if (dcp->connector) {
		dcp->connector->connected = 0;
		apple_connector_edid_set_live(dcp->connector, false);
		dcp_queue_hotplug(dcp->connector);
	}
	complete(&dcp->start_done);
}

static int dcp_rtk_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_rtk_shmem_setup_h17p(dcp, bfr);

	if (bfr->iova) {
		struct iommu_domain *domain =
			iommu_get_domain_for_dev(dcp->dev);
		phys_addr_t phy_addr;

		if (!domain)
			return -ENOMEM;

		// TODO: get map from device-tree
		phy_addr = iommu_iova_to_phys(domain, bfr->iova);
		if (!phy_addr)
			return -ENOMEM;

		// TODO: verify phy_addr, cache attribute
		bfr->buffer = memremap(phy_addr, bfr->size, MEMREMAP_WB);
		if (!bfr->buffer)
			return -ENOMEM;

		bfr->is_mapped = true;
		dev_info(dcp->dev,
			 "shmem_setup: iova: %lx -> pa: %lx -> iomem: %lx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)phy_addr,
			 (uintptr_t)bfr->buffer);
	} else {
		bfr->buffer = dma_alloc_coherent(dcp->dev, bfr->size,
						 &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;

		dev_info(dcp->dev, "shmem_setup: iova: %lx, buffer: %lx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)bfr->buffer);
	}

	return 0;
}

static void dcp_rtk_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp *dcp = cookie;

	if (bfr->is_mapped)
		memunmap(bfr->buffer);
	else
		dma_free_coherent(dcp->dev, bfr->size, bfr->buffer, bfr->iova);
}

static struct apple_rtkit_ops rtkit_ops = {
	.crashed = dcp_rtk_crashed,
	.recv_message = dcp_recv_msg,
	.shmem_setup = dcp_rtk_shmem_setup,
	.shmem_destroy = dcp_rtk_shmem_destroy,
};

void dcp_send_message(struct apple_dcp *dcp, u8 endpoint, u64 message)
{
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P) {
		if (READ_ONCE(dcp->quiescing))
			return;
		trace_dcp_send_msg(dcp, endpoint, message);
		apple_rtkit_send_message(dcp->rtk, endpoint, message, NULL,
					 true);
		return;
	}

	trace_dcp_send_msg(dcp, endpoint, message);
	/*
	 * The adopted 14.7 session shares this mailbox with the panel link.
	 * A non-sleeping send fails while that FIFO is full, and the
	 * DisplayPort handshake then waits for a reply that was never sent.
	 */
	ret = apple_rtkit_send_message(dcp->rtk, endpoint, message, NULL,
				       dcp->fw_compat != DCP_FIRMWARE_V_14_7 ||
				       in_atomic());
	if (ret)
		dev_warn_ratelimited(dcp->dev, "DCP send ep %02x failed: %d\n",
				     endpoint, ret);
}

int dcp_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_crtc_state *crtc_state;
	bool needs_modeset;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_crtc_atomic_check_h17p(crtc, state);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6)
		return iomfb_v26_6_atomic_check(dcp, crtc, state);
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7)
		return iomfb_v14_7_atomic_check(dcp, crtc, state);

	if (dcp->crashed)
		return -EINVAL;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/*
	 * A Type-C display routed to another pipeline: its route follows
	 * this CRTC if it can (see dcp_typec_follow_check()), and the mode is
	 * one of the display's, which only the pipeline driving it knows.
	 */
	if (crtc_state->enable && drm_atomic_crtc_needs_modeset(crtc_state) &&
	    dcp_typec_follows_crtc(dcp)) {
		int ret = dcp_typec_follow_check(dcp, crtc, state,
					      &crtc_state->mode);

		if (ret)
			return ret;
	}

	needs_modeset = drm_atomic_crtc_needs_modeset(crtc_state) ||
			!READ_ONCE(dcp->mode_state.valid);
	if (!needs_modeset && (!dcp->connector || !dcp->connector->connected)) {
		/*
		 * Resume restores the mode before the firmware reports the
		 * display back, so a plane-only commit lands here while the
		 * connector is still marked disconnected.  Rejecting it makes
		 * the compositor fail every flip and give up on the output;
		 * dcp_flush() defers the commit until the link returns.
		 */
		dev_dbg(dcp->dev,
			"crtc_atomic_check: deferring commit, link still down\n");
	}

	return 0;
}

int dcp_get_connector_type(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fixed_connector_type;
}

#define DPTX_CONNECT_TIMEOUT msecs_to_jiffies(2000)
#define DPTX_TUNNEL_CONNECT_TIMEOUT msecs_to_jiffies(8000)
#define DPTX_RECONNECT_DELAY msecs_to_jiffies(1000)
#define DPTX_RECONNECT_RETRIES 5

static struct dcp_fabric_session dcp_session_locked(struct apple_dcp *dcp)
{
	struct dcp_fabric_session session = { .generation = dcp->typec_generation };

	lockdep_assert_held(&dcp->hpd_mutex);
	scoped_guard(mutex, &dcp->tb_lock)
		session.cookie = dcp->tb_generation;
	return session;
}

static bool dcp_session_valid_locked(struct apple_dcp *dcp,
				     struct dcp_fabric_session session, bool cable)
{
	return dcp_fabric_session_valid(session, dcp_session_locked(dcp),
					!cable || dcp->typec_cable_connected,
					READ_ONCE(dcp->tb_retiring) ||
					READ_ONCE(dcp->typec_follow_retiring));
}

/* Recovery must not wake a deliberately parked or disabled output. */
static bool dcp_connect_session_valid_locked(struct apple_dcp *dcp,
					    struct dcp_fabric_session session,
					    bool cable, bool recovery)
{
	return (!recovery || (dcp->typec_work_enabled &&
			     !READ_ONCE(dcp->typec_crtc_off) &&
			     !READ_ONCE(dcp->crashed))) &&
	       dcp_session_valid_locked(dcp, session, cable);
}

/* Capture at enqueue, not when a stale worker eventually starts running. */
void dcp_queue_typec_reconnect(struct apple_dcp *dcp, unsigned long delay)
{
	guard(mutex)(&dcp->hpd_mutex);

	if (!dcp->typec_cable_connected || READ_ONCE(dcp->tb_retiring) ||
	    READ_ONCE(dcp->typec_follow_retiring))
		return;
	dcp->typec_reconnect_session = dcp_session_locked(dcp);
	mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq, delay);
}

static int dcp_dptx_connect_session(struct apple_dcp *dcp, u32 port,
				    struct dcp_fabric_session session, bool cable,
				    bool recovery)
{
	/* T8122 dcpext: HPD only after the link wait, see dcp_ext_dptx_607() */
	bool late_hpd = dcp_ext_dptx_607(dcp);
	unsigned long timeout;
	u8 dfp_port;
	int ret = 0;

	if (!dcp->phy) {
		dev_warn(dcp->dev, "dcp_dptx_connect: missing phy\n");
		return -ENODEV;
	}
	/* A native external processor starts here; its start retries this. */
	if (dcp->external_native && !iomfb_v14_7_external_ready(dcp))
		return -EAGAIN;
	/* @port selects the upstream RemotePort service/core. dptx_dfp_port
	 * is the downstream address: dpphy=0, dpin0=1, dpin1=2.
	 */
	dev_info(dcp->dev,
		 "%s(port=%d) die=%u atc=%u dfp_port=%u tunnel=%d typec=%d route=%s conn_type=%d connected=%d\n",
		 __func__, port, dcp->dptx_die, dcp->dptx_phy, dcp->dptx_dfp_port,
		 dcp->dptx_tunnel, dcp_is_typec_output(dcp),
		 dcp->active_typec_route ? "borrowed" : "fixed",
		 dcp->connector_type, dcp->dptxport[port].connected);

	mutex_lock(&dcp->hpd_mutex);
	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	if (dcp->external && dcpext_scanout_terminal(dcp)) {
		mutex_unlock(&dcp->hpd_mutex);
		return -ESHUTDOWN;
	}
	if (!dcp->dptxport[port].enabled) {
		dev_warn(dcp->dev, "dcp_dptx_connect: dptx service for port %d not enabled\n", port);
		ret = -ENODEV;
		goto out_unlock;
	}

	if (dcp->dptxport[port].connected)
		goto out_unlock;
	if (dcp->external)
		smp_store_release(&dcp->external_link_ready, false);

	reinit_completion(&dcp->dptxport[port].linkcfg_completion);
	WRITE_ONCE(dcp->dptxport[port].tile_hint, false);
	dcp->dptxport[port].atcphy = dcp->phy;
	/* a tiled display's second half comes in on the port's dpin1 */
	dfp_port = port && READ_ONCE(dcp->split.active) ? 2 : dcp->dptx_dfp_port;
	if (!dcp_ext_dptx_607(dcp)) {
		ret = dptxport_validate_connection(dcp->dptxport[port].service,
						   dfp_port,
						   dcp->dptx_phy, dcp->dptx_die);
		if (ret) {
			dev_err(dcp->dev,
				"dcp_dptx_connect: failed to validate DPTX target %u:%u: %d\n",
				dcp->dptx_die, dcp->dptx_phy, ret);
			goto out_unlock;
		}
	}

	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	ret = dptxport_connect(dcp->dptxport[port].service,
			       dfp_port,
			       dcp->dptx_phy, dcp->dptx_die,
			       dcp_is_typec_output(dcp) && !dcp_ext_dptx_607(dcp));
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to connect DPTX target %u:%u: %d\n",
			dcp->dptx_die, dcp->dptx_phy, ret);
		goto out_unlock;
	}

	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	ret = dptxport_request_display(dcp->dptxport[port].service);
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to request display: %d\n",
			ret);
		goto out_release;
	}
	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_release;
	}
	dcp->dptxport[port].connected = true;
	/* The HDMI converter's service, while the firmware holds its sink. */
	if (!port)
		dcp_hdmi_sink_held(dcp, true);
	if (dcp_is_typec_output(dcp) && !late_hpd) {
		if (dcp_uses_t6020_tunnel_flow(dcp))
			ret = dptxport_set_hpd_timeout(dcp->dptxport[port].service,
						       true, 8000);
		else
			ret = dptxport_set_hpd(dcp->dptxport[port].service, true);
		if (ret) {
			dev_err(dcp->dev,
				"dcp_dptx_connect: failed to assert Type-C HPD: %d\n",
				ret);
			dcp->dptxport[port].connected = false;
			goto out_release;
		}
	}

	mutex_unlock(&dcp->hpd_mutex);
	/*
	 * The display interface of a native external processor opens once a
	 * display is attached; the firmware then describes that display.
	 */
	if (dcp->external_native) {
		ret = iomfb_v14_7_external_open(dcp);
		if (ret) {
			mutex_lock(&dcp->hpd_mutex);
			goto out_disconnect;
		}
	}
	timeout = dcp_uses_t6020_tunnel_flow(dcp) ?
		  DPTX_TUNNEL_CONNECT_TIMEOUT : DPTX_CONNECT_TIMEOUT;
	ret = wait_for_completion_timeout(&dcp->dptxport[port].linkcfg_completion,
					  timeout);
	mutex_lock(&dcp->hpd_mutex);
	/* A revoked wait must not disconnect or publish readiness for a new session. */
	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	if (!ret && late_hpd) {
		/* Without HPD this firmware does not train: assert it now. */
		dev_dbg(dcp->dev,
			"dcp_dptx_connect: no link configuration on port %u yet, asserting HPD\n",
			port);
	} else if (!ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: timed out waiting for port %u link configuration\n",
			port);
		ret = -ETIMEDOUT;
		goto out_disconnect;
	} else {
		dev_dbg(dcp->dev, "dcp_dptx_connect: waited %d ms for link\n",
			jiffies_to_msecs(timeout - ret));
	}

	usleep_range(5, 10);
	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}

	if (dcp->connector_type == DRM_MODE_CONNECTOR_DisplayPort || late_hpd) {
		ret = dptxport_set_hpd(dcp->dptxport[port].service, true);
		if (ret && late_hpd)
			dev_err(dcp->dev,
				"dcp_dptx_connect: failed to assert HPD after the link wait: %d\n",
				ret);
		if (ret && (dcp->external || late_hpd))
			goto out_disconnect;
	}
	if (!dcp_connect_session_valid_locked(dcp, session, cable, recovery)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	if (dcp->external) {
		/*
		 * The port must still have its display: a Type-C cable, or HPD
		 * on the fixed HDMI output, which has no cable flag.
		 */
		if (!dcp->dptxport[port].connected ||
		    !(READ_ONCE(dcp->typec_cable_connected) || dcp_hdmi_port_attached(dcp)) ||
		    READ_ONCE(dcp->crashed) || dcpext_scanout_terminal(dcp)) {
			ret = -ENOLINK;
			goto out_unlock;
		}
		smp_store_release(&dcp->external_link_ready, true);
		dcpext_scanout_link_restored(dcp);
	}

	if (dcp->avep)
		av_service_connect(dcp);

	/* The sink's link comes up now, see dcp_typec_relink(). */
	if (!port && dcp_is_typec_output(dcp)) {
		scoped_guard(spinlock_irqsave, &dcp->relink_lock)
			dcp_relink_link_up(&dcp->relink, dcp->typec_generation,
					   jiffies);
	}

	ret = 0;
	goto out_unlock;

out_disconnect:
	dcp->dptxport[port].connected = false;
out_release:
	dptxport_release_display(dcp->dptxport[port].service);

out_unlock:
	mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

/*
 * A T6030 external processor runs as a pipe of the main DRM device when the
 * display gate handed it its display DART: a piodma child on stream 4 of
 * that DART, and stream 0 among the display subsystem's iommus, so that
 * framebuffers are mapped for it as for the panel.
 */
bool dcp_t6030_ext_native(const struct device_node *np)
{
	struct of_phandle_args dart, args;
	struct device_node *piodma, *display;
	bool native = false;
	u32 marker;
	int i, n;

	if (!of_device_is_compatible(np, "apple,t6030-dcpext"))
		return false;
	piodma = of_get_child_by_name(np, "piodma");
	if (!piodma || !of_device_is_available(piodma) ||
	    of_property_read_u32(piodma, "apple,t6030-dispext-handoff", &marker) ||
	    marker != 1 ||
	    of_parse_phandle_with_args(piodma, "iommus", "#iommu-cells", 0, &dart)) {
		of_node_put(piodma);
		return false;
	}
	of_node_put(piodma);
	if (dart.args_count != 1 || dart.args[0] != 4 || !of_device_is_available(dart.np))
		goto out;
	display = of_find_compatible_node(NULL, NULL, "apple,t6030-display-subsystem");
	n = display && of_device_is_available(display) ?
		of_count_phandle_with_args(display, "iommus", "#iommu-cells") : 0;
	for (i = 0; i < n && !native; i++) {
		if (of_parse_phandle_with_args(display, "iommus", "#iommu-cells", i, &args))
			break;
		native = args.np == dart.np && args.args_count == 1 && !args.args[0];
		of_node_put(args.np);
	}
	of_node_put(display);
out:
	of_node_put(dart.np);
	return native;
}

/*
 * A native external processor is running, or has announced a DPTX port:
 * connect the display that is already waiting for it.
 */
void dcp_external_ready(struct apple_dcp *dcp)
{
	if (READ_ONCE(dcp->typec_cable_connected)) {
		dcp->typec_reconnect_tries = 0;
		dcp_queue_typec_reconnect(dcp, 0);
	} else if (dcp->hdmi_hpd && dcp->active &&
		   gpiod_get_value_cansleep(dcp->hdmi_hpd)) {
		int ret = dcp_dptx_connect(dcp, 0);

		if (ret && ret != -EAGAIN && ret != -ESTALE && ret != -ESHUTDOWN)
			dcp_external_retry(dcp, "HDMI display link not set up", ret, 1000);
	}
}

#define DCP_EXTERNAL_RETRIES	3

static int dcp_dptx_release_locked(struct apple_dcp *dcp, u32 port);

/*
 * Bounded recovery for a native external pipe: re-apply the display mode,
 * or redo the display link, up to DCP_EXTERNAL_RETRIES times per attached
 * display, @base_ms, then twice and four times as long apart. A stopped
 * firmware session cannot be recovered, and is not retried.
 */
void dcp_external_retry(struct apple_dcp *dcp, const char *why, int error,
			unsigned int base_ms)
{
	unsigned int n, delay;

	if (!dcp->external_native)
		return;
	if (iomfb_v14_7_external_failed(dcp)) {
		dev_err(dcp->dev, "%s (%d): the external display processor stopped; no retry until reboot\n",
			why, error);
		return;
	}
	if (delayed_work_pending(&dcp->external_retry_wq)) {
		dev_info(dcp->dev, "%s (%d): a retry is already queued\n", why, error);
		return;
	}
	n = atomic_inc_return(&dcp->external_retries);
	if (n > DCP_EXTERNAL_RETRIES) {
		atomic_set(&dcp->external_retries, DCP_EXTERNAL_RETRIES);
		dev_warn(dcp->dev, "%s (%d): no retry left after %u; replug the display\n",
			 why, error, DCP_EXTERNAL_RETRIES);
		return;
	}
	delay = base_ms << (n - 1);
	dev_info(dcp->dev, "%s (%d): retry %u of %u in %u ms\n", why, error, n,
		 DCP_EXTERNAL_RETRIES, delay);
	/* An attach or detach after this makes it stale; see the work. */
	WRITE_ONCE(dcp->external_retry_generation, READ_ONCE(dcp->typec_generation));
	mod_delayed_work(system_freezable_wq, &dcp->external_retry_wq, msecs_to_jiffies(delay));
}

/* The active CRTC's mode is the one the firmware runs and shows swaps in. */
static bool dcp_external_crtc_showing(struct apple_dcp *dcp)
{
	struct drm_crtc *crtc = dcp->crtc ? &dcp->crtc->base : NULL;
	bool showing = false;

	if (!crtc)
		return false;
	drm_modeset_lock(&crtc->mutex, NULL);
	if (crtc->state && crtc->state->active)
		showing = iomfb_v14_7_external_showing(dcp, &crtc->state->mode);
	drm_modeset_unlock(&crtc->mutex);
	return showing;
}

void dcp_external_retry_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					     external_retry_wq);
	struct apple_connector *connector = READ_ONCE(dcp->connector);
	u64 generation = READ_ONCE(dcp->external_retry_generation);
	bool released;

	if (READ_ONCE(dcp->typec_generation) != generation) {
		dev_info(dcp->dev, "display retry: the display was attached or detached since; dropped\n");
		return;
	}
	/* Kept connected through a withdrawal, and not described again. */
	released = iomfb_v14_7_external_release(dcp);
	if (iomfb_v14_7_external_failed(dcp)) {
		dev_err(dcp->dev, "display retry: the external display processor stopped; no retry until reboot\n");
		return;
	}
	/* Described again: re-apply the mode if it is not set. */
	if (!released && connector && READ_ONCE(connector->connected)) {
		if (READ_ONCE(dcp->mode_state.valid)) {
			dev_info(dcp->dev, "display retry: the display is back and set\n");
			return;
		}
		if (dcp_external_crtc_showing(dcp)) {
			dev_info(dcp->dev, "display retry: the display is back in its mode and showing\n");
			dcp_mode_set_valid(&dcp->mode_state, true);
			return;
		}
		dev_info(dcp->dev, "display retry: setting the display mode again\n");
		dcp_mode_invalidate(&dcp->mode_state);
		dcp_queue_hotplug(connector);
		return;
	}
	/* An HDMI output connects through the fabric, as on its HPD. */
	if (!dcp_is_typec_output(dcp) && dcp->hdmi_hpd) {
		dcp_fabric_hdmi_retry(dcp);
		return;
	}
	if (!READ_ONCE(dcp->typec_cable_connected)) {
		dev_info(dcp->dev, "display retry: the port has no display any more\n");
		return;
	}
	/*
	 * The port holds a display that the firmware no longer describes:
	 * release the link if it is still up, and connect it again, which
	 * makes the firmware describe the display anew.
	 */
	scoped_guard(mutex, &dcp->hpd_mutex) {
		/* Not under a connect or release of another display. */
		if (dcp->typec_generation != generation || !dcp->typec_cable_connected) {
			dev_info(dcp->dev, "display retry: the display was attached or detached since; dropped\n");
			return;
		}
		dev_info(dcp->dev, "display retry: reconnecting the display link\n");
		if (dcp->dptxport[0].enabled && dcp->dptxport[0].connected) {
			int ret = dptxport_set_hpd(dcp->dptxport[0].service, false);

			if (ret) {
				dev_warn(dcp->dev, "display retry: HPD deassert failed: %d\n", ret);
				return;
			}
			ret = dcp_dptx_release_locked(dcp, 0);
			if (ret)
				return;
		}
		dcp->typec_reconnect_tries = 0;
	}
	dcp_queue_typec_reconnect(dcp, 0);
}

/*
 * The sink of a native external pipe raised IRQ_HPD with HPD staying high,
 * as a DP branch device does when the display behind it changes. Pass the
 * request to the firmware, which services it as a DP source does. If it
 * cannot take the request, connect the display link anew, which makes the
 * firmware read the display again.
 */
void dcp_external_sink_irq(struct apple_dcp *dcp)
{
	struct platform_device *pdev = to_platform_device(dcp->dev);
	struct apple_epic_service *service = READ_ONCE(dcp->dptxport[0].service);
	int ret;

	/* With no link up, the next connect reads the display anyway. */
	if (!dcp->external_native || !service || !READ_ONCE(dcp->dptxport[0].enabled) ||
	    !READ_ONCE(dcp->dptxport[0].connected))
		return;
	ret = dptxport_sink_irq(service);
	if (!ret) {
		dev_info(dcp->dev, "display sink IRQ_HPD passed to the firmware\n");
		return;
	}
	dev_info(dcp->dev, "display sink IRQ_HPD not passed (%d): connecting the display anew\n",
		 ret);
	ret = dcp_dptx_disconnect_oob(pdev, 0);
	if (ret)
		return;
	dcp_dptx_connect_oob(pdev, 0);
}

/*
 * tiled_split: DPTX port 1 carries the second half of a tiled display on
 * this pipeline. Bring it back whenever port 0 (re)connects, as DCP pairs
 * the tiles only while both ports are connected.
 */
static int dcp_dptx_connect_tile(struct apple_dcp *dcp)
{
	bool split;
	int ret;

	scoped_guard(mutex, &dcp->tb_lock)
		split = dcp->split.active && dcp->split.ready &&
			dcp->split.generation;
	if (!split)
		return 0;
	ret = dcp_dptx_connect(dcp, 1);
	if (ret)
		dev_warn(dcp->dev, "tiled: DPTX port 1 reconnect failed: %d\n", ret);
	return ret;
}

int dcp_dptx_connect(struct apple_dcp *dcp, u32 port)
{
	struct dcp_fabric_session session;
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_dptx_connect_h17p(dcp, port);

	scoped_guard(mutex, &dcp->hpd_mutex)
		session = dcp_session_locked(dcp);
	ret = dcp_dptx_connect_session(dcp, port, session, !!session.cookie, false);
	if (!ret && !port)
		ret = dcp_dptx_connect_tile(dcp);
	return ret;
}

static bool dcp_edid_is_placeholder(const struct drm_edid *drm_edid)
{
	const u8 *raw = (const u8 *)drm_edid_raw(drm_edid);
	static const u8 name[] = "Non-PnP";
	unsigned int i;

	if (!raw)
		return false;

	/* EDID contains interior NUL bytes, so this cannot use strnstr(). */
	for (i = 0; i + sizeof(name) - 1 <= sizeof(struct edid); i++) {
		if (!memcmp(raw + i, name, sizeof(name) - 1))
			return true;
	}

	return false;
}

void dcp_retry_placeholder_edid(struct apple_dcp *dcp,
				const struct drm_edid *drm_edid)
{
	guard(mutex)(&dcp->hpd_mutex);
	if (!dcp_is_typec_output(dcp) || !dcp->typec_cable_connected ||
	    dcp->placeholder_retried)
		return;
	if (!dcp_edid_is_placeholder(drm_edid))
		return;

	dcp->placeholder_retried = true;
	dcp->placeholder_generation = dcp->typec_generation;
	schedule_delayed_work(&dcp->placeholder_edid_wq, msecs_to_jiffies(300));
}

static void dcp_placeholder_edid_work(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     placeholder_edid_wq);
	struct apple_epic_service *service;
	u64 generation;
	int ret;

	mutex_lock(&dcp->hpd_mutex);
	generation = dcp->placeholder_generation;
	if (!dcp->typec_cable_connected || !dcp->dptxport[0].connected ||
	    !dcp->dptxport[0].enabled || generation != dcp->typec_generation)
		goto out_unlock;
	service = dcp->dptxport[0].service;

	/*
	 * Some adapters answer the first connection with a 1024x768
	 * placeholder and publish the panel EDID only after HPD drops
	 * and returns. One pulse; a second placeholder is left alone.
	 * DCP reports the drop as an unplug, which is not a lost link.
	 */
	WRITE_ONCE(dcp->typec_hpd_held, true);
	ret = dptxport_set_hpd(service, false);
	if (ret) {
		dev_info(dcp->dev, "placeholder EDID: HPD drop failed: %d\n",
			 ret);
		goto out_unlock;
	}
	mutex_unlock(&dcp->hpd_mutex);

	msleep(1000);

	mutex_lock(&dcp->hpd_mutex);
	if (!dcp->typec_cable_connected || !dcp->dptxport[0].connected ||
	    generation != dcp->typec_generation)
		goto out_unlock;

	ret = dptxport_set_hpd(service, true);
	if (ret)
		dev_info(dcp->dev, "placeholder EDID: HPD assert failed: %d\n",
			 ret);
out_unlock:
	WRITE_ONCE(dcp->typec_hpd_held, false);
	mutex_unlock(&dcp->hpd_mutex);
}

static void dcp_typec_reconnect_work(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     typec_reconnect_wq);
	struct dcp_fabric_session session;
	int ret;

	scoped_guard(mutex, &dcp->hpd_mutex) {
		session = dcp->typec_reconnect_session;
		if ((dcp->external && dcpext_scanout_terminal(dcp)) ||
		    !dcp_session_valid_locked(dcp, session, true))
			return;
	}
	ret = dcp_dptx_connect_session(dcp, 0, session, true, false);
	if (!ret)
		ret = dcp_dptx_connect_tile(dcp);
	guard(mutex)(&dcp->hpd_mutex);
	if (!dcp_session_valid_locked(dcp, session, true))
		return;
	if (!ret) {
		dcp->typec_reconnect_tries = 0;
		return;
	}
	if (++dcp->typec_reconnect_tries <
	    (dcp_uses_t6020_tunnel_flow(dcp) ? 1 : DPTX_RECONNECT_RETRIES)) {
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq,
				 DPTX_RECONNECT_DELAY);
		return;
	}
	dev_err(dcp->dev, "Type-C DPTX reconnect failed after %u retries: %d\n",
		dcp->typec_reconnect_tries, ret);
}

/* A display is attached to the Type-C output: a new attachment. */
static void dcp_typec_attach_locked(struct apple_dcp *dcp)
{
	lockdep_assert_held(&dcp->hpd_mutex);
	dcp->typec_generation++;
	WRITE_ONCE(dcp->typec_cable_connected, true);
	dcp->typec_reconnect_tries = 0;
	dcp->placeholder_retried = false;
	cancel_delayed_work(&dcp->typec_reconnect_wq);
	/* A newly attached display gets its own retries. */
	if (dcp->external_native) {
		atomic_set(&dcp->external_retries, 0);
		cancel_delayed_work(&dcp->external_retry_wq);
	}
}

/* Connect the link of an attached display, retrying later if that fails. */
static int dcp_dptx_connect_attached(struct apple_dcp *dcp, u32 port)
{
	int ret = dcp_dptx_connect(dcp, port);

	if (ret && ret != -ESHUTDOWN && dcp_is_typec_output(dcp))
		dcp_queue_typec_reconnect(dcp, DPTX_RECONNECT_DELAY);

	return ret;
}

/*
 * A USB-C monitor with a built-in hub can lose the DP link when the hub
 * brings its SuperSpeed link up right around DCP's link training for the
 * first modeset. DCP may then report the sink as unplugged about two
 * seconds after HPD was asserted for it, although the sink still asserts
 * HPD, and nothing else connects it again: replay an unplug and replug of
 * the sink ("relink"), which trains the link once the hub has settled, as
 * a cable replug does. A sink's IRQ_HPD gets a retrain, as before.
 *
 * The retries cover one attachment or one DPMS wake: the replay itself
 * does not reset them, dcp_poweron() does. dcp-relink.h holds the
 * decisions; relink_lock protects their state.
 */
static bool dcp_typec_relink_eligible(struct apple_dcp *dcp)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);

	/* H17P and native external pipes recover through their own lifecycle */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P || dcp->external_native ||
	    !dcp_is_typec_output(dcp) || dcp_is_usb4_output(dcp) ||
	    dcp_is_external_h17p(dcp) || !dcp->typec_connector)
		return false;
	/*
	 * Only a sink that is attached, shown and still connected for DRM:
	 * not one the driver itself let go, holds HPD low for (see
	 * dcp_placeholder_edid_work()) or moves to another pipeline.
	 */
	return READ_ONCE(dcp->typec_cable_connected) &&
	       !READ_ONCE(dcp->typec_crtc_off) &&
	       !READ_ONCE(dcp->typec_hpd_held) &&
	       !READ_ONCE(dcp->tb_retiring) &&
	       !READ_ONCE(dcp->typec_follow_retiring) &&
	       connector && READ_ONCE(connector->connected);
}

/* DCP reported an unplug of a Type-C sink that still asserts HPD */
bool dcp_typec_relink(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool start;
	u32 tries;

	if (!dcp_typec_relink_eligible(dcp))
		return false;

	spin_lock_irqsave(&dcp->relink_lock, flags);
	start = dcp_relink_unplug(&dcp->relink, READ_ONCE(dcp->typec_generation),
				  jiffies);
	if (start)
		mod_delayed_work(system_freezable_wq, &dcp->typec_relink_wq, 0);
	tries = dcp->relink.tries;
	spin_unlock_irqrestore(&dcp->relink_lock, flags);

	if (start)
		dev_info(dcp->dev,
			 "DCP dropped a sink that asserts HPD, relinking the Type-C sink (%u/%u)\n",
			 tries, DCP_RELINK_RETRIES);
	return start;
}

/*
 * The sink asked for attention: IRQ_HPD or a coalesced HPD pulse. The
 * caller retrains the link, as for any such request, unless a relink is
 * queued or runs: that connects the sink anew, and until it does, the
 * sink has no link to retrain.
 */
bool dcp_typec_sink_relink(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool active;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return false;

	spin_lock_irqsave(&dcp->relink_lock, flags);
	active = dcp->relink.active;
	spin_unlock_irqrestore(&dcp->relink_lock, flags);

	return active;
}

static void dcp_typec_relink_work(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     typec_relink_wq);
	struct platform_device *pdev = to_platform_device(dcp->dev);
	unsigned long flags;
	u64 generation;
	bool run;

	scoped_guard(mutex, &dcp->hpd_mutex) {
		generation = dcp->typec_generation;
		spin_lock_irqsave(&dcp->relink_lock, flags);
		run = dcp->relink.active && dcp->relink.generation == generation &&
		      dcp->typec_cable_connected;
		spin_unlock_irqrestore(&dcp->relink_lock, flags);
	}
	if (!run)
		goto out;

	dcp_dptx_disconnect_oob(pdev, 0);
	msleep(DCP_RELINK_UNPLUG_MS);

	/*
	 * Attach the sink again as a replug does, unless a real unplug or
	 * replug came meanwhile, which then takes over. Checked and done
	 * under one hold of hpd_mutex, so that neither can come in between.
	 */
	cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (dcp->typec_generation != generation + 1)
			goto out;
		dcp_typec_attach_locked(dcp);
		spin_lock_irqsave(&dcp->relink_lock, flags);
		dcp_relink_reattach(&dcp->relink, dcp->typec_generation, jiffies);
		spin_unlock_irqrestore(&dcp->relink_lock, flags);
	}
	dcp_dptx_connect_attached(dcp, 0);
out:
	spin_lock_irqsave(&dcp->relink_lock, flags);
	dcp_relink_end(&dcp->relink);
	spin_unlock_irqrestore(&dcp->relink_lock, flags);
}

static void disconnected_hpd_event(struct apple_connector *con)
{
	apple_connector_edid_set_live(con, false);
	if (con && con->connected) {
		struct platform_device *pdev = READ_ONCE(con->dcp);

		if (pdev) {
			struct apple_dcp *dcp = platform_get_drvdata(pdev);

			WRITE_ONCE(dcp->ext_backlight, false);
		}
		con->connected = 0;
		drm_kms_helper_connector_hotplug_event(&con->base);
		/*
		 * Drop the display's backlight, outside the caller's locks. Not
		 * the hotplug work, which would send a second hotplug event.
		 */
		schedule_work(&con->bl_sync_wq);
	}
}

static int dcp_dptx_release_locked(struct apple_dcp *dcp, u32 port)
{
	int ret;

	lockdep_assert_held(&dcp->hpd_mutex);
	if (dcp->external) {
		smp_store_release(&dcp->external_link_ready, false);
		dcpext_scanout_invalidate(dcp);
	}
	if (dcp->dptxport[port].enabled && dcp->dptxport[port].connected) {
		ret = dptxport_release_display(dcp->dptxport[port].service);
		if (ret)
			return ret;
		dcp->dptxport[port].connected = false;
	}
	if (!port)
		dcp_hdmi_sink_held(dcp, false);
	/* A released sink is no longer inactive: the next one starts afresh. */
	WRITE_ONCE(dcp->dptxport[port].sink_inactive, false);
	/*
	 * T6030: the firmware takes a direct crossbar output down when it
	 * stops the link. Make sure of it for a display released here, also
	 * when the firmware did not get that far.
	 */
	if (dcp->external_native)
		dcp_direct_crossbar_link(dcp, false);
	return 0;
}

int dcp_dptx_disconnect(struct apple_dcp *dcp, u32 port)
{
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_dptx_disconnect_h17p(dcp, port);

	/* Release the caller's RemotePort service, not the downstream DFP port. */
	dev_info(dcp->dev, "%s(port=%d)\n", __func__, port);

	mutex_lock(&dcp->hpd_mutex);
	ret = dcp_dptx_release_locked(dcp, port);
	if (!ret)
		WRITE_ONCE(dcp->dptxport[port].tile_hint, false);
	mutex_unlock(&dcp->hpd_mutex);

	return ret;
}

int dcp_dptx_connect_oob(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_dptx_connect_oob_h17p(pdev, port);

	if (dcp_is_typec_output(dcp)) {
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		guard(mutex)(&dcp->hpd_mutex);
		dcp_typec_attach_locked(dcp);
		scoped_guard(spinlock_irqsave, &dcp->relink_lock)
			dcp_relink_attach(&dcp->relink, dcp->typec_generation);
	}

	return dcp_dptx_connect_attached(dcp, port);
}

int dcp_dptx_disconnect_oob(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_dptx_disconnect_oob_h17p(pdev, port);

	if (dcp_is_typec_output(dcp)) {
		scoped_guard(mutex, &dcp->hpd_mutex) {
			WRITE_ONCE(dcp->typec_cable_connected, false);
			dcp->typec_generation++;
			dcp->typec_unplugs++;
			/* Nothing left to retry for the display that went. */
			if (dcp->external_native)
				cancel_delayed_work(&dcp->external_retry_wq);
		}
		WRITE_ONCE(dcp->typec_crtc_off, false);
		reinit_completion(&dcp->typec_iomfb_hpd_ready);
		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
	}

	return dcp_dptx_disconnect_drained(dcp, port);
}

/* A delivered sink IRQ can follow a firmware unplug with cable HPD still high. */
int dcp_dptx_recover_irq(struct apple_dcp *dcp)
{
	struct dcp_fabric_session session;
	int ret;

	mutex_lock(&dcp->hpd_mutex);
	if (dcp->external_native || !dcp_is_typec_output(dcp) ||
	    dcp_is_usb4_output(dcp) || !dcp->typec_connector ||
	    READ_ONCE(dcp->typec_connector->connected) ||
	    READ_ONCE(dcp->typec_crtc_off)) {
		ret = 0;
		goto out_unlock;
	}
	if (!dcp->typec_work_enabled || READ_ONCE(dcp->crashed) ||
	    !dcp->dptxport[0].enabled) {
		ret = -EAGAIN;
		goto out_unlock;
	}
	session = dcp_session_locked(dcp);
	if (!dcp_connect_session_valid_locked(dcp, session, true, true)) {
		ret = -ESTALE;
		goto out_unlock;
	}

	/* Revoke older reconnect waits without changing physical cable state. */
	dcp->typec_generation++;
	session = dcp_session_locked(dcp);
	cancel_delayed_work(&dcp->typec_reconnect_wq);
	cancel_delayed_work(&dcp->placeholder_edid_wq);
	if (dcp->avep)
		av_service_disconnect(dcp);
	ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
	if (ret)
		goto out_unlock;
	if (!dcp_connect_session_valid_locked(dcp, session, true, true)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	/* A failed earlier connect may also have left an unconfirmed release. */
	ret = dptxport_release_display(dcp->dptxport[0].service);
	if (ret)
		goto out_unlock;
	if (!dcp_connect_session_valid_locked(dcp, session, true, true)) {
		ret = -ESTALE;
		goto out_unlock;
	}
	dcp->dptxport[0].connected = false;
	mutex_unlock(&dcp->hpd_mutex);

	/* Another delivered IRQ must confirm release before a failed attempt retries. */
	return dcp_dptx_connect_session(dcp, 0, session, true, true);

out_unlock:
	mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

/*
 * Release the display link of a Type-C output while its display stays
 * attached, as a CRTC power-off does (see dcp_poweroff()): the firmware's
 * unplug for it is ignored, and dcp_poweron() connects the link again.
 */
int dcp_dptx_park(struct apple_dcp *dcp)
{
	int ret;

	scoped_guard(mutex, &dcp->hpd_mutex) {
		WRITE_ONCE(dcp->typec_crtc_off, true);
		dcp->typec_generation++;
	}
	/* Session checks around reconnect waits reject the revoked generation. */
	cancel_delayed_work(&dcp->typec_reconnect_wq);
	cancel_delayed_work(&dcp->placeholder_edid_wq);
	if (dcp->avep)
		av_service_disconnect(dcp);
	if (READ_ONCE(dcp->split.active) && dcp->dptxport[1].enabled &&
	    dcp->dptxport[1].connected) {
		ret = dptxport_set_hpd(dcp->dptxport[1].service, false);
		if (ret)
			return ret;
		ret = dcp_dptx_disconnect(dcp, 1);
		if (ret)
			return ret;
	}
	if (dcp->dptxport[0].enabled && dcp->dptxport[0].connected) {
		ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
		if (ret) {
			dev_warn(dcp->dev, "failed to deassert Type-C DPTX HPD: %d\n", ret);
			return ret;
		}
		return dcp_dptx_disconnect(dcp, 0);
	}
	return 0;
}

int dcp_dptx_disconnect_drained(struct apple_dcp *dcp, u32 port)
{
	int ret;

	WRITE_ONCE(dcp->typec_crtc_off, false);
	reinit_completion(&dcp->typec_iomfb_hpd_ready);

	disconnected_hpd_event(dcp->connector);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dcp->dptxport[port].enabled) {
		ret = dptxport_set_hpd(dcp->dptxport[port].service, false);
		if (ret)
			return ret;
	}

	return dcp_dptx_disconnect(dcp, port);
}

bool dcp_fw_compat_is_12_x(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fw_compat == DCP_FIRMWARE_V_12_3;
}

/* A failed T8122 dcpext: no pipeline of the DRM device. */
bool dcp_ext_unusable(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return !dcp || READ_ONCE(dcp->ext_dead);
}

bool dcp_fw_compat_is_14_7(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->fw_compat == DCP_FIRMWARE_V_14_7 ||
	       dcp->fw_compat == DCP_FIRMWARE_V_26_6;
}

/*
 * Whether the CRTC gets DRM colour management. A 14.7 pipe sends a colour
 * matrix only where its firmware image has a tested setter; elsewhere the
 * property would be taken and never applied.
 */
bool dcp_ctm_supported(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7)
		return iomfb_v14_7_ctm_supported(dcp);
	return true;
}

unsigned long* dcp_get_iomfb_surfaces(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->iomfb_surfaces;
}

int dcp_start(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_start_h17p(pdev);

	init_completion(&dcp->start_done);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6)
		return iomfb_v26_6_start(dcp);

	/*
	 * The T6030 firmware session is adopted for the internal panel.
	 * Open DPTX on that same RTKit only when the firmware advertised it.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7) {
		if (dcp->external) {
			/* A native pipe is usable now; its firmware starts on first use. */
			if (dcp->external_native)
				dcp->active = true;
			complete(&dcp->start_done);
			return 0;
		}
		ret = iomfb_v14_7_start(dcp);
		if (ret)
			return ret;
		return 0;
	}

	/* start RTKit endpoints */
	ret = systemep_init(dcp);
	if (ret)
		dev_warn(dcp->dev, "Failed to start system endpoint: %d\n", ret);

	if (unstable_edid && !dcp_has_panel(dcp)) {
		ret = dpavservep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start DPAVSERV endpoint: %d",
				 ret);
	}

	if (dcp->phy && dcp->fw_compat >= DCP_FIRMWARE_V_13_5) {
		ret = ibootep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start IBOOT endpoint: %d\n",
				 ret);

		ret = dptxep_init(dcp);
		if (ret) {
			dev_warn(dcp->dev, "Failed to start DPTX endpoint: %d\n",
				 ret);
#ifdef DCP_DPTX_DISCONNECT_ON_INIT
		/*
		 * This disconnect / connect cycle on init is only necessary
		 * when using dcp0 on j473, j474s and presumedly j475c.
		 * Since dcp0 is not used at the moment let's avoid this
		 * since it is possibly the cause for startup issues.
		 */
		} else if (dcp->dptxport[0].enabled) {
			bool connected;
			/* force disconnect on start - necessary if the display
			 * is already up from m1n1
			 */
			dptxport_set_hpd(dcp->dptxport[0].service, false);
			dptxport_release_display(dcp->dptxport[0].service);
			usleep_range(10 * USEC_PER_MSEC, 25 * USEC_PER_MSEC);

			connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
			dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

			// necessary on j473/j474 but not on j314c
			if (connected)
				dcp_dptx_connect(dcp, 0);
#endif
		}
	} else if (dcp->phy) {
		dev_warn(dcp->dev, "OS firmware incompatible with dptxport EP\n");
	}
	ret = iomfb_start_rtkit(dcp);
	if (ret) {
		dev_err(dcp->dev, "Failed to start IOMFB endpoint: %d\n", ret);
		return ret;
	}

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	if (hdmi_audio) {
		ret = avep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start AV endpoint: %d", ret);
		ret = 0;
	}
#endif

	return ret;
}

/*
 * A native T6030 external processor: its DP/HDMI audio endpoint (EP 0x29)
 * on the driver's opt-in (hdmi_audio), as the other DCPs start it, for a
 * processor whose port@0 links an audio device; once per session, and after
 * the first display's EDID was read: with the endpoint's interfaces
 * published, a CopyEDID has the firmware rebuild them and re-plug the sink,
 * so the endpoint starts only once the first EDID is cached.
 */
static void dcp_external_audio_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, external_audio_work);

	/* Queued after an EDID came through the running session: RTKit is up. */
	if (!IS_ENABLED(CONFIG_DRM_APPLE_AUDIO) || !hdmi_audio || !dcp->external_native ||
	    dcp->external_audio_started || READ_ONCE(dcp->crashed) || !dcp->rtk)
		return;
	dcp->external_audio_started = true;
	if (!apple_rtkit_has_endpoint(dcp->rtk, AV_ENDPOINT)) {
		dev_info(dcp->dev, "no AV audio endpoint\n");
		return;
	}
#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	{
		int ret = avep_init(dcp);

		if (ret)
			dev_warn(dcp->dev, "AV audio endpoint not started: %d\n", ret);
	}
#endif
}

void dcp_external_audio_start(struct apple_dcp *dcp)
{
	if (!dcp->external_native || dcp->external_audio_started)
		return;
	queue_work(system_freezable_wq, &dcp->external_audio_work);
}

static void _dcp_poweroff(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_poweroff_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_poweroff_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_poweroff_v14_7_0(dcp);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_v26_6_poweroff(dcp);
		break;
	case DCP_FIRMWARE_V_14_7:
		iomfb_v14_7_poweroff(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_poweroff_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

static void dcp_resume_enable_irq(void *ctx)
{
	struct apple_dcp *dcp = ctx;

	enable_irq(dcp->hdmi_hpd_irq);
}

static void dcp_resume_sample(void *ctx)
{
	dcp_fabric_hdmi_resume(ctx);
}

static const struct dcp_fabric_resume_ops dcp_resume_ops = {
	.enable_irq = dcp_resume_enable_irq,
	.sample = dcp_resume_sample,
};

static int dcp_enable_dp2hdmi_hpd(struct apple_dcp *dcp)
{
	if (dcp_is_typec_output(dcp)) {
		if (READ_ONCE(dcp->typec_cable_connected))
			dcp_dptx_connect(dcp, 0);
	} else if (dcp->hdmi_hpd) {
		/* Check HPD before enabling the edge-triggered IRQ. */
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

		if (connected)
			dcp_dptx_connect(dcp, 0);
		else
			_dcp_poweroff(dcp);
	}

	if (dcp->hdmi_hpd_irq)
		dcp_fabric_run_resume(&dcp_resume_ops, dcp);

	return 0;
}

int dcp_wait_ready(struct platform_device *pdev, u64 timeout)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp->crashed)
		return -ENODEV;
	if (dcp->active)
		return dcp_enable_dp2hdmi_hpd(dcp);
	if (timeout <= 0)
		return -ETIMEDOUT;

	ret = wait_for_completion_timeout(&dcp->start_done, timeout);
	if (ret < 0)
		return ret;

	if (dcp->crashed)
		return -ENODEV;

	if (dcp->active)
		dcp_enable_dp2hdmi_hpd(dcp);

	return dcp->active ? 0 : -ETIMEDOUT;
}

static void __maybe_unused dcp_sleep(struct apple_dcp *dcp)
{
	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_sleep_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_sleep_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_sleep_v14_7_0(dcp);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_v26_6_poweroff(dcp);
		break;
	case DCP_FIRMWARE_V_14_7:
		iomfb_v14_7_poweroff(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_sleep_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
}

int dcp_typec_prepare_route(struct apple_dcp *dcp)
{
	u64 generation;
	unsigned long ready;
	int ret;

	if (!dcp_crtc_needs_route_start(dcp))
		return 0;
	scoped_guard(mutex, &dcp->hpd_mutex)
		generation = dcp->typec_generation;
	ret = dcp_dptx_connect(dcp, 0);
	if (ret)
		return ret;
	ready = wait_for_completion_timeout(&dcp->typec_iomfb_hpd_ready,
					    msecs_to_jiffies(3000));
	guard(mutex)(&dcp->hpd_mutex);
	if (generation != dcp->typec_generation || !dcp->typec_cable_connected)
		return -ESTALE;
	return ready ? 0 : -ETIMEDOUT;
}

void dcp_poweron(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	bool wait_for_typec_hpd = false;
	unsigned long remaining;
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_poweron_h17p(pdev);

	if (dcp_is_typec_output(dcp)) {
		/* T8122 dcpext only, see dcp_poweroff() */
		if (dcp->ext_failsafe) {
			scoped_guard(mutex, &dcp->hpd_mutex)
				dcp->crtc_unplugs = dcp->typec_unplugs;
		}
		wait_for_typec_hpd = READ_ONCE(dcp->typec_crtc_off) &&
				    READ_ONCE(dcp->typec_cable_connected);
		WRITE_ONCE(dcp->typec_crtc_off, false);
		WRITE_ONCE(dcp->typec_follow_start, false);
		/*
		 * A DPMS wake retrains the link like a replug: give it a fresh
		 * relink budget. No relink runs while the CRTC was off.
		 */
		if (wait_for_typec_hpd) {
			unsigned long flags;

			spin_lock_irqsave(&dcp->relink_lock, flags);
			dcp_relink_wake(&dcp->relink,
					READ_ONCE(dcp->typec_generation), jiffies);
			spin_unlock_irqrestore(&dcp->relink_lock, flags);
		}

		/*
		 * A Type-C CRTC disable releases its DPTX session. Re-establish it
		 * synchronously before IOMFB is powered back on.
		 */
		if (READ_ONCE(dcp->typec_cable_connected)) {
			cancel_delayed_work(&dcp->typec_reconnect_wq);
			dcp->typec_reconnect_tries = 0;
			/* the modeset of this commit follows, see dcpep_cb_hotplug() */
			WRITE_ONCE(dcp->typec_poweron_hpd, true);
			ret = dcp_dptx_connect(dcp, 0);
			if (ret)
				dcp_queue_typec_reconnect(dcp, DPTX_RECONNECT_DELAY);
			else if (wait_for_typec_hpd) {
				remaining = wait_for_completion_timeout(
					&dcp->typec_iomfb_hpd_ready,
					msecs_to_jiffies(3000));
				if (!remaining)
					dev_warn(dcp->dev,
						 "Type-C IOMFB hotplug not ready on wake\n");
			}
			WRITE_ONCE(dcp->typec_poweron_hpd, false);
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

		if (connected)
			dcp_dptx_connect(dcp, 0);
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_poweron_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_poweron_v13_3(dcp);
		break;
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
		iomfb_poweron_v14_7_0(dcp);
		break;
	case DCP_FIRMWARE_V_26_6:
		iomfb_v26_6_poweron(dcp);
		break;
	case DCP_FIRMWARE_V_14_7:
		iomfb_v14_7_poweron(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
	if (dcp->avep)
		av_service_connect(dcp);
}

void dcp_poweroff(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	bool display_gone = false;
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_poweroff_h17p(pdev);

	cancel_delayed_work(&dcp->swap_watchdog_wq);
	if (dcp->avep)
		av_service_disconnect(dcp);

	/*
	 * The display this CRTC was powered on for has been unplugged since:
	 * its link went down with the cable (dcp_dptx_disconnect_oob()), and
	 * the desktop only now turns the CRTC off. By then the pipeline may
	 * already drive the next display, on this port or on another one that
	 * was waiting for it, and that connection is not the CRTC's to
	 * release: dropping its HPD makes the firmware report the new display
	 * inactive, and it stays dark.
	 *
	 * Only the T8122 dcpext (ext_failsafe) takes this path, the one
	 * pipeline where it was run. Every other Type-C output releases its
	 * connection below as before: the M1/M2 dcpext routes, and the T6030
	 * split second port and external retry.
	 */
	if (dcp->ext_failsafe && dcp_is_typec_output(dcp)) {
		scoped_guard(mutex, &dcp->hpd_mutex)
			display_gone = dcp->typec_unplugs != dcp->crtc_unplugs;
		if (display_gone) {
			dev_dbg(dcp->dev,
				"CRTC off after its display was unplugged: leaving the current connection alone\n");
			_dcp_poweroff(dcp);
			return;
		}
	}

	/*
	 * Powering a Type-C CRTC off drops DCP's synthetic HPD, and the firmware
	 * reports that as an unplug. The display is still attached: keep the
	 * connector connected (see dcpep_cb_hotplug()) and let dcp_poweron()
	 * re-establish the DPTX session. Recreating it here instead makes the
	 * display vanish and come back, and compositors light a returning
	 * display, so DPMS off never sticks. Cable removal is reported through
	 * the Type-C mux.
	 */
	if (dcp_is_typec_output(dcp)) {
		reinit_completion(&dcp->typec_iomfb_hpd_ready);
		if (READ_ONCE(dcp->typec_cable_connected))
			WRITE_ONCE(dcp->typec_crtc_off, true);
		/* dcp_poweron() reconnects the link on DPMS wake. */
		cancel_delayed_work(&dcp->typec_reconnect_wq);
	}

	_dcp_poweroff(dcp);

	if (dcp_is_typec_output(dcp)) {
		ret = 0;
		/* DCP owns a synthetic HPD for Type-C. Release it with the CRTC. */
		if (dcp->dptxport[0].enabled && dcp->dptxport[0].connected) {
			ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
			if (ret)
				dev_warn(dcp->dev,
					 "failed to deassert Type-C DPTX HPD: %d\n", ret);
			dcp_dptx_disconnect(dcp, 0);
		}
		/*
		 * A native external pipe whose display is no longer described
		 * (it went away during a hotplug bounce) is powered off by the
		 * desktop for good: with HPD still high, connect the link again
		 * so that the firmware describes the display anew.
		 */
		if (dcp->external_native && READ_ONCE(dcp->typec_cable_connected) &&
		    (ret || !dcp->connector || !READ_ONCE(dcp->connector->connected)))
			dcp_external_retry(dcp, ret ? "display link released with an HPD error" :
					   "display link released with no display described",
					   ret, 500);
		/* and the second tile's, which dcp_poweron() brings back too */
		if (READ_ONCE(dcp->split.active) && dcp->dptxport[1].enabled &&
		    dcp->dptxport[1].connected) {
			dptxport_set_hpd(dcp->dptxport[1].service, false);
			dcp_dptx_disconnect(dcp, 1);
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		if (!connected) {
			disconnected_hpd_event(dcp->connector);
			dcp_dptx_disconnect(dcp, 0);
		}
	}
}

static void dcp_work_register_backlight(struct work_struct *work)
{
	int ret;
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, bl_register_wq);

	mutex_lock(&dcp->bl_register_mutex);
	if (dcp->brightness.bl_dev)
		goto out_unlock;

	/* try to register backlight device, */
	ret = dcp_backlight_register(dcp);
	if (ret) {
		dev_err(dcp->dev, "Unable to register backlight device\n");
		dcp->brightness.maximum = 0;
	}

out_unlock:
	mutex_unlock(&dcp->bl_register_mutex);
}

static void dcp_work_update_backlight(struct work_struct *work)
{
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, bl_update_wq);

	dcp_backlight_update(dcp);
}

static int dcp_create_piodma_iommu_dev(struct apple_dcp *dcp)
{
	int ret;
	struct device_node *node __free(device_node) = of_get_child_by_name(dcp->dev->of_node, "piodma");

	if (!node)
		return dev_err_probe(dcp->dev, -ENODEV,
				     "Failed to get piodma child DT node\n");

	dcp->piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->piodma)
		return dev_err_probe(dcp->dev, -ENODEV, "Failed to create piodma pdev for %pOF\n", node);

	ret = dma_set_mask_and_coherent(&dcp->piodma->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err_destroy_pdev;

	ret = of_dma_configure(&dcp->piodma->dev, node, true);
	if (ret) {
		ret = dev_err_probe(dcp->dev, ret,
			"Failed to configure IOMMU child DMA\n");
		goto err_destroy_pdev;
	}

	dcp->iommu_dom = iommu_get_domain_for_dev(&dcp->piodma->dev);
	if (IS_ERR(dcp->iommu_dom)) {
		ret = dev_err_probe(dcp->dev, PTR_ERR(dcp->iommu_dom),
				    "Failed to get default iommu domain for "
				    "piodma device\n");
		dcp->iommu_dom = NULL;
		goto err_destroy_pdev;
	}

	return 0;
err_destroy_pdev:
	of_platform_device_destroy(&dcp->piodma->dev, NULL);
	return ret;
}

static int dcp_get_bw_scratch_reg(struct apple_dcp *dcp, u32 expected)
{
	struct of_phandle_args ph_args;
	u32 addr_idx, disp_idx, offset;
	int ret;

	ret = of_parse_phandle_with_args(dcp->dev->of_node, "apple,bw-scratch",
				   "#apple,bw-scratch-cells", 0, &ph_args);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to read 'apple,bw-scratch': %d\n", ret);
		return ret;
	}

	if (ph_args.args_count != 3) {
		dev_err(dcp->dev, "Unexpected 'apple,bw-scratch' arg count %d\n",
			ph_args.args_count);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	addr_idx = ph_args.args[0];
	disp_idx = ph_args.args[1];
	offset = ph_args.args[2];

	if (disp_idx != expected || disp_idx >= MAX_DISP_REGISTERS) {
		dev_err(dcp->dev, "Unexpected disp_reg value in 'apple,bw-scratch': %d\n",
			disp_idx);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	ret = of_address_to_resource(ph_args.np, addr_idx, &dcp->disp_bw_scratch_res);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to get 'apple,bw-scratch' resource %d from %pOF\n",
			addr_idx, ph_args.np);
		goto err_of_node_put;
	}
	if (offset > resource_size(&dcp->disp_bw_scratch_res) - 4) {
		ret = -EINVAL;
		goto err_of_node_put;
	}

	dcp->disp_registers[disp_idx] = &dcp->disp_bw_scratch_res;
	dcp->disp_bw_scratch_index = disp_idx;
	dcp->disp_bw_scratch_offset = offset;
	ret = 0;

err_of_node_put:
	of_node_put(ph_args.np);
	return ret;
}

static int dcp_get_bw_doorbell_reg(struct apple_dcp *dcp, u32 expected)
{
	struct of_phandle_args ph_args;
	u32 addr_idx, disp_idx;
	int ret;

	ret = of_parse_phandle_with_args(dcp->dev->of_node, "apple,bw-doorbell",
				   "#apple,bw-doorbell-cells", 0, &ph_args);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to read 'apple,bw-doorbell': %d\n", ret);
		return ret;
	}

	if (ph_args.args_count != 2) {
		dev_err(dcp->dev, "Unexpected 'apple,bw-doorbell' arg count %d\n",
			ph_args.args_count);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	addr_idx = ph_args.args[0];
	disp_idx = ph_args.args[1];

	if (disp_idx != expected || disp_idx >= MAX_DISP_REGISTERS) {
		dev_err(dcp->dev, "Unexpected disp_reg value in 'apple,bw-doorbell': %d\n",
			disp_idx);
		ret = -EINVAL;
		goto err_of_node_put;
	}

	ret = of_address_to_resource(ph_args.np, addr_idx, &dcp->disp_bw_doorbell_res);
	if (ret < 0) {
		dev_err(dcp->dev, "Failed to get 'apple,bw-doorbell' resource %d from %pOF\n",
			addr_idx, ph_args.np);
		goto err_of_node_put;
	}
	dcp->disp_bw_doorbell_index = disp_idx;
	dcp->disp_registers[disp_idx] = &dcp->disp_bw_doorbell_res;
	ret = 0;

err_of_node_put:
	of_node_put(ph_args.np);
	return ret;
}

static int dcp_get_disp_regs(struct apple_dcp *dcp)
{
	struct platform_device *pdev = to_platform_device(dcp->dev);
	int count = pdev->num_resources - 1;
	int i, ret;

	if (count <= 0 || count > MAX_DISP_REGISTERS)
		return -EINVAL;

	for (i = 0; i < count; ++i) {
		dcp->disp_registers[i] =
			platform_get_resource(pdev, IORESOURCE_MEM, 1 + i);
	}

	/* load pmgr bandwidth scratch resource and offset */
	ret = dcp_get_bw_scratch_reg(dcp, count);
	if (ret < 0)
		return ret;
	count += 1;

	/* load pmgr bandwidth doorbell resource if present (only on t8103) */
	if (of_property_present(dcp->dev->of_node, "apple,bw-doorbell")) {
		ret = dcp_get_bw_doorbell_reg(dcp, count);
		if (ret < 0)
			return ret;
		count += 1;
	}

	dcp->nr_disp_registers = count;
	return 0;
}

#define DCP_FW_VERSION_MIN_LEN	3
#define DCP_FW_VERSION_MAX_LEN	5
#define DCP_FW_VERSION_STR_LEN	(DCP_FW_VERSION_MAX_LEN * 4)

static int dcp_read_fw_version(struct device *dev, const char *name,
			       char *version_str)
{
	u32 ver[DCP_FW_VERSION_MAX_LEN];
	int len_str;
	int len;

	len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						  DCP_FW_VERSION_MIN_LEN,
						  DCP_FW_VERSION_MAX_LEN);

	switch (len) {
	case 3:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d", ver[0], ver[1], ver[2]);
		break;
	case 4:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3]);
		break;
	case 5:
		len_str = scnprintf(version_str, DCP_FW_VERSION_STR_LEN,
				    "%d.%d.%d.%d.%d", ver[0], ver[1], ver[2],
				    ver[3], ver[4]);
		break;
	default:
		len_str = strscpy(version_str, "UNKNOWN",
				  DCP_FW_VERSION_STR_LEN);
		if (len >= 0)
			len = -EOVERFLOW;
		break;
	}

	if (len_str >= DCP_FW_VERSION_STR_LEN)
		dev_warn(dev, "'%s' truncated: '%s'\n", name, version_str);

	return len;
}

static enum dcp_firmware_version dcp_check_firmware_version(struct device *dev)
{
	const struct apple_dcp_hw_data *hw = of_device_get_match_data(dev);
	const struct dcp_v14_board *v14_board;
	char compat_str[DCP_FW_VERSION_STR_LEN];
	char fw_str[DCP_FW_VERSION_STR_LEN];
	int ret;

	/*
	 * SoCs introduced with H17-generation firmware pin its interface; the
	 * loader may not recognise their boot firmware version.
	 */
	if (hw && hw->firmware_compat != DCP_FIRMWARE_UNKNOWN)
		return hw->firmware_compat;

	/* firmware version is just informative */
	dcp_read_fw_version(dev, "apple,firmware-version", fw_str);

	ret = dcp_read_fw_version(dev, "apple,firmware-compat", compat_str);
	if (ret < 0) {
		dev_err(dev, "Could not read 'apple,firmware-compat': %d\n", ret);
		return DCP_FIRMWARE_UNKNOWN;
	}

	/* 25G83 (J613; J615 experimental) has a distinct callback table, never a v14 fallback. */
	if (of_device_is_compatible(dev->of_node, "apple,t8122-dcp") &&
	    of_property_present(dev->of_node, "apple,j613-25g83-profile")) {
		const char *uuid;
		u32 profile;

		if (apple_t8122_25g83_board() &&
		    !of_property_read_u32(dev->of_node, "apple,j613-25g83-profile", &profile) &&
		    profile == 1 && !strcmp(compat_str, "26.6.2") &&
		    !of_property_read_string(dev->of_node, "apple,firmware-uuid", &uuid) &&
		    !strcmp(uuid, "C042E95C-B9D8-3F0E-94B3-582A08AA6FDD"))
			return DCP_FIRMWARE_V_26_6;
		return DCP_FIRMWARE_UNKNOWN;
	}

	/*
	 * The T8122 external processor: Linux starts it and drives 14.7
	 * firmware through the classic IOMFB template, as on M1/M2.
	 */
	if (of_device_is_compatible(dev->of_node, "apple,t8122-dcpext")) {
		if (!strcmp(compat_str, "14.7.0"))
			return DCP_FIRMWARE_V_14_7_TEMPLATE;
		dev_err(dev, "T8122 external display not started: DCP firmware-compat %s is not 14.7.0\n",
			compat_str);
		return DCP_FIRMWARE_UNKNOWN;
	}

	/* The T6030 external processors: the 14.x firmware IOMFB only. */
	if (of_device_is_compatible(dev->of_node, "apple,t6030-dcpext")) {
		if (!strcmp(compat_str, "14.7.0"))
			return DCP_FIRMWARE_V_14_7;
		dev_err(dev, "T6030 display not started: DCP firmware-compat %s is not 14.7.0\n",
			compat_str);
		return DCP_FIRMWARE_UNKNOWN;
	}

	/* The internal panels with a board record (T6030, J613): the same. */
	v14_board = iomfb_v14_7_board(dev);
	if (IS_ERR(v14_board))
		return DCP_FIRMWARE_UNKNOWN;
	if (v14_board) {
		if (!strcmp(compat_str, "14.7.0"))
			return DCP_FIRMWARE_V_14_7;
		dev_err(dev, "%s display not started: DCP firmware-compat %s is not 14.7.0\n",
			iomfb_v14_7_board_name(v14_board), compat_str);
		return DCP_FIRMWARE_UNKNOWN;
	}

	if (strncmp(compat_str, "12.3.0", sizeof(compat_str)) == 0)
		return DCP_FIRMWARE_V_12_3;
	/*
	 * m1n1 reports firmware version 13.5 as compatible with 13.3. This is
	 * only true for the iomfb endpoint. The interface for the dptx-port
	 * endpoint changed between 13.3 and 13.5. The driver will only support
	 * firmware 13.5. Check the actual firmware version for compat version
	 * 13.3 until m1n1 reports 13.5 as "firmware-compat".
	 */
	else if ((strncmp(compat_str, "13.3.0", sizeof(compat_str)) == 0) &&
		 (strncmp(fw_str, "13.5.0", sizeof(compat_str)) == 0))
		return DCP_FIRMWARE_V_13_5;
	else if (strncmp(compat_str, "13.5.0", sizeof(compat_str)) == 0)
		return DCP_FIRMWARE_V_13_5;

	dev_err(dev, "DCP firmware-compat %s (FW: %s) is not supported\n",
		compat_str, fw_str);

	return DCP_FIRMWARE_UNKNOWN;
}

static int dcp_connector_type_from_dt(struct device_node *np)
{
	if (of_property_match_string(np, "apple,connector-type", "HDMI-A") >= 0)
		return DRM_MODE_CONNECTOR_HDMIA;
	if (of_property_match_string(np, "apple,connector-type", "DP") >= 0)
		return DRM_MODE_CONNECTOR_DisplayPort;
	if (of_property_match_string(np, "apple,connector-type", "USB-C") >= 0)
		return DRM_MODE_CONNECTOR_USB;

	return DRM_MODE_CONNECTOR_Unknown;
}

static void dcp_disable_typec_work(struct apple_dcp *dcp, bool release_cable)
{
	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_disable_typec_work_h17p(dcp, release_cable);
	scoped_guard(mutex, &dcp->hpd_mutex) {
		dcp->typec_work_enabled = false;
		if (release_cable)
			WRITE_ONCE(dcp->typec_cable_connected, false);
		dcp->typec_generation++;
	}
	/* Block new enqueues as well as draining users of the AFK endpoints. */
	disable_delayed_work_sync(&dcp->typec_reconnect_wq);
	disable_delayed_work_sync(&dcp->placeholder_edid_wq);
	scoped_guard(spinlock_irqsave, &dcp->relink_lock)
		dcp_relink_enable(&dcp->relink, false);
	disable_delayed_work_sync(&dcp->typec_relink_wq);
	disable_delayed_work_sync(&dcp->hdmi_settle_wq);
	disable_delayed_work_sync(&dcp->hdmi_recheck_wq);
	dcp_hdmi_disable(dcp);
}

static void dcp_enable_typec_work(struct apple_dcp *dcp)
{
	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_enable_typec_work_h17p(dcp);
	enable_delayed_work(&dcp->typec_reconnect_wq);
	enable_delayed_work(&dcp->placeholder_edid_wq);
	enable_delayed_work(&dcp->typec_relink_wq);
	scoped_guard(spinlock_irqsave, &dcp->relink_lock)
		dcp_relink_enable(&dcp->relink, true);
	enable_delayed_work(&dcp->hdmi_settle_wq);
	enable_delayed_work(&dcp->hdmi_recheck_wq);
	dcp_hdmi_enable(dcp);
	scoped_guard(mutex, &dcp->hpd_mutex)
		dcp->typec_work_enabled = true;
	/* A cable can be routed before the DRM component binds. */
	if (READ_ONCE(dcp->typec_cable_connected))
		dcp_queue_typec_reconnect(dcp, 0);
}

/*
 * The M3 internal displays (14.x and 26.6 IOMFB) also accept
 * appledrm.t6030_show_notch=1, the switch existing J613/T6030 installs pass,
 * so the panel comes up at its full height (2560x1664 on the J613) instead
 * of the module refusing an unknown parameter.
 */
static bool dcp_show_notch(struct apple_dcp *dcp)
{
	if (dcp->fw_compat != DCP_FIRMWARE_V_14_7 &&
	    dcp->fw_compat != DCP_FIRMWARE_V_26_6)
		return show_notch;
	return show_notch || t6030_show_notch;
}

/*
 * T8122 dcpext (DCP_FIRMWARE_V_14_7_TEMPLATE): if the PMP runs, the display
 * gate added its DISPEXT request and pointed the dcpext at it
 * (apple,pmp-report). As on T6030, run the processor only once the PMP has
 * acknowledged that request: the running PMP owns the SoC power and
 * performance states and treats a block whose request it has not taken as
 * idle. The internal display started the PMP long before this, so the wait
 * is short; it is bounded, and without an acknowledgement the processor is
 * started anyway (as before the request existed), with a warning.
 */
#define DCP_EXT_PMP_TIMEOUT_MS	3000

static void dcp_ext_wait_pmp(struct apple_dcp *dcp)
{
	struct device_node *entry;
	unsigned long deadline;
	int ret;

	if (dcp->fw_compat != DCP_FIRMWARE_V_14_7_TEMPLATE)
		return;
	entry = of_parse_phandle(dcp->dev->of_node, "apple,pmp-report", 0);
	if (!entry) {
		dev_info(dcp->dev, "no PMP DISPEXT request; starting without one\n");
		return;
	}
	deadline = jiffies + msecs_to_jiffies(DCP_EXT_PMP_TIMEOUT_MS);
	for (;;) {
		ret = apple_pmp_report_wait_ready(entry,
						  msecs_to_jiffies(DCP_EXT_PMP_TIMEOUT_MS));
		/* -EIO: the PMP runs, but has not acknowledged the request yet. */
		if (ret != -EIO || time_after(jiffies, deadline))
			break;
		msleep(20);
	}
	of_node_put(entry);
	if (ret)
		dev_warn(dcp->dev,
			 "the PMP has not acknowledged the DISPEXT request (%d); starting anyway\n",
			 ret);
	else
		dev_info(dcp->dev, "PMP acknowledged the DISPEXT request\n");
}

static int dcp_comp_bind_pipe(struct device *dev, struct device *main, void *data)
{
	struct device_node *panel_np;
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	u32 cpu_ctrl;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	/* A native external processor mapped these at probe, for its start work. */
	if (!dcp->external_native) {
		dcp->coproc_reg = devm_platform_ioremap_resource_byname(to_platform_device(dev),
									"coproc");
		if (IS_ERR(dcp->coproc_reg))
			return PTR_ERR(dcp->coproc_reg);
	}

	if (dcp->index || dcp->dptx_phy || dcp->dptx_die)
		dev_info(dev, "DCP index:%u dptx target phy: %u dptx die: %u\n",
			 dcp->index, dcp->dptx_phy, dcp->dptx_die);

	if (!dcp_show_notch(dcp))
		ret = of_property_read_u32(dev->of_node, "apple,notch-height",
					   &dcp->notch_height);

	if (dcp->notch_height > MAX_NOTCH_HEIGHT)
		dcp->notch_height = MAX_NOTCH_HEIGHT;
	if (dcp->notch_height > 0)
		dev_info(dev, "Detected display with notch of %u pixel\n", dcp->notch_height);

	/* initialize brightness scale to a sensible default to avoid divide by 0*/
	dcp->brightness.scale = 65536;
	panel_np = of_get_compatible_child(dev->of_node, "apple,panel-mini-led");
	if (panel_np)
		dcp->panel.has_mini_led = true;
	else
		panel_np = of_get_compatible_child(dev->of_node, "apple,panel");

	if (panel_np) {
		const char height_prop[2][16] = { "adj-height-mm", "height-mm" };

		if (of_device_is_available(panel_np)) {
			ret = of_property_read_u32(panel_np, "apple,max-brightness",
						   &dcp->brightness.maximum);
			if (ret)
				dev_err(dev, "Missing property 'apple,max-brightness'\n");
		}

		of_property_read_u32(panel_np, "width-mm", &dcp->panel.width_mm);
		/* use adjusted height as long as the notch is hidden */
		of_property_read_u32(panel_np, height_prop[!dcp->notch_height],
				     &dcp->panel.height_mm);

		of_node_put(panel_np);
		dcp->fixed_connector_type = DRM_MODE_CONNECTOR_eDP;
		dcp->connector_type = DRM_MODE_CONNECTOR_eDP;
		INIT_WORK(&dcp->bl_register_wq, dcp_work_register_backlight);
		mutex_init(&dcp->bl_register_mutex);
		INIT_WORK(&dcp->bl_update_wq, dcp_work_update_backlight);
	}

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6) {
		ret = iomfb_v26_6_bind(dcp);
		if (!ret)
			enable_work(&dcp->dimensions_wq);
		return ret;
	}

	/* The running T6030 firmware is adopted as is. */
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7) {
		if (dcp->external_native) {
			WRITE_ONCE(dcp->external_detached, false);
			enable_work(&dcp->vblank_wq);
			enable_work(&dcp->external_audio_work);
			enable_delayed_work(&dcp->swap_watchdog_wq);
			enable_delayed_work(&dcp->external_retry_wq);
			enable_work(&dcp->dimensions_wq);
			dcp_enable_typec_work(dcp);
			return 0;
		}
		if (dcp->external)
			return 0;
		ret = iomfb_v14_7_bind(dcp);
		if (!ret)
			enable_work(&dcp->dimensions_wq);
		return ret;
	}

	ret = dcp_create_piodma_iommu_dev(dcp);
	if (ret || !dcp->iommu_dom)
		return dev_err_probe(dev, ret,
				"Failed to created PIODMA iommu child device");

	ret = dcp_get_disp_regs(dcp);
	if (ret) {
		dev_err(dev, "failed to find display registers\n");
		return ret;
	}

	dcp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(dcp->clk))
		return dev_err_probe(dev, PTR_ERR(dcp->clk),
				     "Unable to find clock\n");

	bitmap_zero(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	// TDOD: mem_desc IDs start at 1, for simplicity just skip '0' entry
	set_bit(0, dcp->memdesc_map);

	INIT_WORK(&dcp->vblank_wq, dcp_delayed_vblank);
	INIT_DELAYED_WORK(&dcp->swap_watchdog_wq, dcp_swap_watchdog);

	dcp->swapped_out_fbs =
		(struct list_head)LIST_HEAD_INIT(dcp->swapped_out_fbs);

	dcp_ext_wait_pmp(dcp);

	cpu_ctrl =
		readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	writel_relaxed(cpu_ctrl | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);

	dcp->rtk = devm_apple_rtkit_init(dev, dcp, "mbox", 0, &rtkit_ops);
	if (IS_ERR(dcp->rtk))
		return dev_err_probe(dev, PTR_ERR(dcp->rtk),
				     "Failed to initialize RTKit\n");

	ret = apple_rtkit_wake(dcp->rtk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to boot RTKit: %d\n", ret);
	enable_work(&dcp->dimensions_wq);
	schedule_work(&dcp->dimensions_wq);
	dcp_enable_typec_work(dcp);
	return ret;
}

static void dcp_ext_give_up(struct apple_dcp *dcp, int err, const char *when);

/*
 * The T8122 dcpext boots its firmware here. If that fails, it must not take
 * the internal panel's DRM device down with it: keep it as an unusable pipe.
 */
static int dcp_comp_bind(struct device *dev, struct device *main, void *data)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_comp_bind_h17p(dev, main, data);

	if (READ_ONCE(dcp->ext_dead))
		return 0;
	ret = dcp_comp_bind_pipe(dev, main, data);
	if (ret && dcp->ext_failsafe) {
		dcp_ext_give_up(dcp, ret, "bind");
		return 0;
	}
	return ret;
}

/*
 * We need to shutdown DCP before tearing down the display subsystem. Otherwise
 * the DCP will crash and briefly flash a green screen of death.
 */
static void dcp_comp_unbind(struct device *dev, struct device *main, void *data)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp && dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_comp_unbind_h17p(dev, main, data);
	if (!dcp || READ_ONCE(dcp->ext_dead))
		return;

	disable_work_sync(&dcp->dimensions_wq);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6) {
		iomfb_v26_6_unbind(dcp);
		return;
	}
	if (dcp->fw_compat == DCP_FIRMWARE_V_14_7) {
		if (dcp->external_native) {
			/* Bind's wait_ready enabled it, as for any HDMI port. */
			if (dcp->hdmi_hpd_irq)
				disable_irq(dcp->hdmi_hpd_irq);
			dcp_disable_typec_work(dcp, true);
			disable_delayed_work_sync(&dcp->external_retry_wq);
			disable_delayed_work_sync(&dcp->swap_watchdog_wq);
			disable_work_sync(&dcp->vblank_wq);
			disable_work_sync(&dcp->external_audio_work);
		}
		iomfb_v14_7_unbind(dcp);
		return;
	}

	if (dcp->hdmi_hpd_irq)
		disable_irq(dcp->hdmi_hpd_irq);

	dcp_disable_typec_work(dcp, true);
	/* RTKit is released after this unbind callback, and can still deliver
	 * a final swap completion or hotplug while its receive queue drains.
	 */
	disable_delayed_work_sync(&dcp->swap_watchdog_wq);
	disable_work_sync(&dcp->vblank_wq);
	typec_mux_put(dcp->typec_mux);

	if (dcp->fixed_connector_type == DRM_MODE_CONNECTOR_eDP) {
		/* Registration runs asynchronously and its devres can belong
		 * to the platform device rather than the component bind group.
		 * Stop callbacks and userspace writes while the CRTC, RTKit and
		 * IOMMU they use are still available.
		 */
		disable_work_sync(&dcp->bl_register_wq);
		disable_work_sync(&dcp->bl_update_wq);
		if (dcp->brightness.bl_dev) {
			devm_backlight_device_unregister(dev, dcp->brightness.bl_dev);
			dcp->brightness.bl_dev = NULL;
		}
	}

	if (dcp->avep) {
		av_service_disconnect(dcp);
		afk_shutdown(dcp->avep);
		dcp->avep = NULL;
	}

	dcp_fabric_shutdown_dptx(dcp);

	if (dcp->ibootep) {
		afk_shutdown(dcp->ibootep);
		dcp->ibootep = NULL;
	}

	if (dcp->systemep) {
		afk_shutdown(dcp->systemep);
		dcp->systemep = NULL;
	}

	if (dcp->dcpavservep) {
		afk_shutdown(dcp->dcpavservep);
		dpavservep_detach(dcp);
		dcp->dcpavservep = NULL;
	}

	if (dcp->shmem)
		iomfb_shutdown(dcp);

	if (dcp->piodma) {
		dcp->iommu_dom = NULL;
		of_platform_device_destroy(&dcp->piodma->dev, NULL);
		dcp->piodma = NULL;
	}

	devm_clk_put(dev, dcp->clk);
	dcp->clk = NULL;
}

static const struct component_ops dcp_comp_ops = {
	.bind	= dcp_comp_bind,
	.unbind	= dcp_comp_unbind,
};

/*
 * A T6030 external processor's fixed (HDMI) output is optional. Its Type-C
 * routes must register whatever happens to that output: every USB-C port
 * lists them and waits for them, as the panel's DRM device waits for this
 * processor. Keep deferring while the HDMI parts may still appear, then go
 * on as a Type-C-only pipe. Elsewhere the error ends the probe, as before.
 */
static int dcp_fixed_output_error(struct apple_dcp *dcp, int err, const char *what)
{
	if (!dcp->external_native)
		return err;
	if (err == -EPROBE_DEFER) {
		err = driver_deferred_probe_check_state(dcp->dev);
		if (err == -EPROBE_DEFER)
			return err;
	}
	dev_err(dcp->dev, "HDMI output unusable (%s: %d); USB-C displays are not affected\n",
		what, err);
	if (dcp->fixed_route_selected)
		mux_control_deselect(dcp->xbar);
	dcp->fixed_route_selected = false;
	dcp->xbar = NULL;
	if (!IS_ERR_OR_NULL(dcp->typec_mux))
		typec_mux_put(dcp->typec_mux);
	dcp->typec_mux = NULL;
	dcp->phy_managed_by_typec = false;
	/* A requested HPD interrupt stays disabled: nothing enables it now. */
	dcp->hdmi_hpd_irq = 0;
	dcp->hdmi_hpd = NULL;
	dcp->hdmi_pwren = NULL;
	dcp->dp2hdmi_pwren = NULL;
	dcp->phy = NULL;
	dcp->fixed_phy = NULL;
	dcp->fixed_connector_type = DRM_MODE_CONNECTOR_USB;
	dcp->connector_type = DRM_MODE_CONNECTOR_USB;
	return 0;
}

/*
 * A T6030 external processor whose Type-C routes did not register: disable
 * them in the device tree so that the ports stop waiting for them, and add
 * the processor to the DRM device as a pipe that never starts, so that the
 * panel is not held up either.
 */
static int dcp_native_routes_error(struct apple_dcp *dcp, int err)
{
	if (!dcp->external_native)
		return err;
	if (err == -EPROBE_DEFER) {
		err = driver_deferred_probe_check_state(dcp->dev);
		if (err == -EPROBE_DEFER)
			return err;
	}
	dev_err(dcp->dev, "Type-C display routes unusable: %d; USB-C ports go on without this processor\n",
		err);
	dcp_typec_routes_disable(dcp);
	return 0;
}

static int dcp_platform_probe_pipe(struct platform_device *pdev)
{
	enum dcp_firmware_version fw_compat;
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	int ret, surf, num_surfs;
	bool routes_failed = false;
	u32 surf_en;
	u32 mux_index;

	fw_compat = dcp_check_firmware_version(dev);
	if (fw_compat == DCP_FIRMWARE_UNKNOWN)
		return -ENODEV;

	/* Check for "apple,bw-scratch" to avoid probing appledrm with outdated
	 * device trees. This prevents replacing simpledrm and ending up without
	 * display.
	 */
	if (fw_compat != DCP_FIRMWARE_V_14_7 &&
	    fw_compat != DCP_FIRMWARE_V_26_6 &&
	    !of_property_present(dev->of_node, "apple,bw-scratch"))
		return dev_err_probe(dev, -ENODEV, "Incompatible devicetree! "
			"Use devicetree matching this kernel.\n");

	dcp = devm_kzalloc(dev, sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;

	dcp->fw_compat = fw_compat;
	dcp->ext_failsafe = of_device_is_compatible(dev->of_node, "apple,t8122-dcpext");
	dcp->external = of_device_is_compatible(dev->of_node, "apple,t6030-dcpext");
	dcp->external_native = dcp->external && dcp_t6030_ext_native(dev->of_node);
	INIT_WORK(&dcp->external_audio_work, dcp_external_audio_work);
	dcp->dev = dev;
	/*
	 * Type-C and Thunderbolt routes can be activated as soon as they are
	 * registered below, before the DRM device binds.
	 */
	dcp_modes_init(dcp);
	/* Registered before callback resources, so they drain before the catalog. */
	ret = devm_add_action_or_reset(dev, dcp_modes_release, dcp);
	if (ret)
		return ret;
	mutex_init(&dcp->hpd_mutex);
	mutex_init(&dcp->tb_lock);
	dcp_fabric_init(dcp);
	spin_lock_init(&dcp->mode_state.lock);
	mutex_init(&dcp->dcpavserv.lock);
	dcp->hw = *(struct apple_dcp_hw_data *)of_device_get_match_data(dev);
	dcp->fixed_connector_type = dcp_connector_type_from_dt(dev->of_node);
	dcp->connector_type = dcp->fixed_connector_type;
	of_property_read_u32(dev->of_node, "apple,dcp-index", &dcp->index);
	of_property_read_u32(dev->of_node, "apple,dptx-phy", &dcp->dptx_phy);
	of_property_read_u32(dev->of_node, "apple,dptx-die", &dcp->dptx_die);
	dcp->fixed_dptx_phy = dcp->dptx_phy;
	init_completion(&dcp->typec_iomfb_hpd_ready);
	INIT_DELAYED_WORK(&dcp->typec_reconnect_wq,
			  dcp_typec_reconnect_work);
	INIT_DELAYED_WORK(&dcp->placeholder_edid_wq,
			  dcp_placeholder_edid_work);
	INIT_DELAYED_WORK(&dcp->typec_relink_wq, dcp_typec_relink_work);
	spin_lock_init(&dcp->relink_lock);
	/* Balanced by enable at successful component bind. */
	disable_delayed_work(&dcp->typec_reconnect_wq);
	disable_delayed_work(&dcp->placeholder_edid_wq);
	disable_delayed_work(&dcp->typec_relink_wq);
	/*
	 * A native external pipe's bind enables these; firmware callbacks may
	 * come before it. The 14.7 panel never uses them, but suspend cancels
	 * the watchdog of every processor.
	 */
	if (fw_compat == DCP_FIRMWARE_V_14_7 || fw_compat == DCP_FIRMWARE_V_26_6) {
		INIT_WORK(&dcp->vblank_wq, dcp_delayed_vblank);
		INIT_DELAYED_WORK(&dcp->swap_watchdog_wq, dcp_swap_watchdog);
		disable_work(&dcp->vblank_wq);
		disable_delayed_work(&dcp->swap_watchdog_wq);
	}

	platform_set_drvdata(pdev, dcp);

	if (fw_compat == DCP_FIRMWARE_V_26_6) {
		ret = iomfb_v26_6_probe(dcp);
		if (ret)
			return ret;
	}
	if (fw_compat == DCP_FIRMWARE_V_14_7 && !dcp->external) {
		/* The panel's level policy (dcp_backlight.c), presented in nits. */
		ret = dcp_backlight_init(dcp, dev);
		if (ret)
			return ret;
		ret = iomfb_v14_7_probe(dcp);
		if (ret)
			return ret;
	}
	if (dcp->external)
		dcp->hw.num_dptx_ports = 2;

	dcp->phy = devm_phy_optional_get(dev, "dp-phy");
	if (IS_ERR(dcp->phy) && dcp->external_native) {
		ret = dcp_fixed_output_error(dcp, PTR_ERR(dcp->phy), "dp-phy");
		if (ret)
			return ret;
	}
	if (IS_ERR(dcp->phy)) {
		dev_err(dev, "Failed to get dp-phy: %ld\n", PTR_ERR(dcp->phy));
		return PTR_ERR(dcp->phy);
	}
	dcp->fixed_phy = dcp->phy;

	bitmap_zero(dcp->iomfb_surfaces, DCP_MAX_PLANES);
	if (!of_property_present(dev->of_node, "apple,iomfb-surfaces"))
		num_surfs = 0;
	else
		num_surfs = of_property_count_elems_of_size(dev->of_node,
						    "apple,iomfb-surfaces",
						    sizeof(u32));

	if (fw_compat == DCP_FIRMWARE_V_14_7 || fw_compat == DCP_FIRMWARE_V_26_6) {
		set_bit(0, dcp->iomfb_surfaces);
	} else if (num_surfs == 0 || num_surfs == -ENODATA) {
		set_bit(0, dcp->iomfb_surfaces);
		set_bit(1, dcp->iomfb_surfaces);
	} else if (num_surfs < 0) {
		return num_surfs;
	} else if (num_surfs > DCP_MAX_PLANES) {
		dev_err(dev, "Number of iomfb-surfaces (%d) exceeds DCP_MAX_PLANES\n",
			num_surfs);
		return -EINVAL;
	}

	surf = 0;
	of_property_for_each_u32(dev->of_node, "apple,iomfb-surfaces", surf_en) {
		if (surf_en)
			set_bit(surf, dcp->iomfb_surfaces);
		surf++;
	}

	if (dcp->phy) {
		int ret;
		/*
		 * Request DP2HDMI related GPIOs as optional for DP-altmode
		 * compatibility. J180D misses a dp2hdmi-pwren GPIO in the
		 * template ADT. TODO: check device ADT
		 */
		dcp->hdmi_hpd = devm_gpiod_get_optional(dev, "hdmi-hpd", GPIOD_IN);
		if (IS_ERR(dcp->hdmi_hpd) && dcp->external_native) {
			ret = dcp_fixed_output_error(dcp, PTR_ERR(dcp->hdmi_hpd), "hdmi-hpd");
			if (ret)
				return ret;
			goto fixed_done;
		}
		if (IS_ERR(dcp->hdmi_hpd))
			return PTR_ERR(dcp->hdmi_hpd);
		ret = dcp_hdmi_init(dcp);
		if (ret)
			return ret;
		if (dcp->hdmi_hpd) {
			int irq = gpiod_to_irq(dcp->hdmi_hpd);
			if (irq < 0 && dcp->external_native) {
				ret = dcp_fixed_output_error(dcp, irq, "hdmi-hpd interrupt");
				if (ret)
					return ret;
				goto fixed_done;
			}
			if (irq < 0) {
				dev_err(dev, "failed to translate HDMI hpd GPIO to IRQ\n");
				return irq;
			}
			dcp->hdmi_hpd_irq = irq;

			ret = devm_request_threaded_irq(dev, dcp->hdmi_hpd_irq,
						dcp_dp2hdmi_hpd_edge, dcp_dp2hdmi_hpd,
						IRQF_ONESHOT | IRQF_NO_AUTOEN |
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						"dp2hdmi-hpd-irq", dcp);
			if (ret < 0 && dcp->external_native) {
				ret = dcp_fixed_output_error(dcp, ret, "hdmi-hpd interrupt");
				if (ret)
					return ret;
				goto fixed_done;
			}
			if (ret < 0) {
				dev_err(dev, "failed to request HDMI hpd irq %d: %d\n",
					irq, ret);
				return ret;
			}
		}

		/*
		 * Power DP2HDMI on as it is required for the HPD irq.
		 * TODO: check if one is sufficient for the hpd to save power
		 *       on battery powered Macbooks.
		 */
		dcp->hdmi_pwren = devm_gpiod_get_optional(dev, "hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->hdmi_pwren) && dcp->external_native) {
			ret = dcp_fixed_output_error(dcp, PTR_ERR(dcp->hdmi_pwren), "hdmi-pwren");
			if (ret)
				return ret;
			goto fixed_done;
		}
		if (IS_ERR(dcp->hdmi_pwren))
			return PTR_ERR(dcp->hdmi_pwren);

		dcp->dp2hdmi_pwren = devm_gpiod_get_optional(dev, "dp2hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->dp2hdmi_pwren) && dcp->external_native) {
			ret = dcp_fixed_output_error(dcp, PTR_ERR(dcp->dp2hdmi_pwren), "dp2hdmi-pwren");
			if (ret)
				return ret;
			goto fixed_done;
		}
		if (IS_ERR(dcp->dp2hdmi_pwren))
			return PTR_ERR(dcp->dp2hdmi_pwren);

		/*
		 * A DCP may have both a fixed HDMI/DP route and allocatable Type-C
		 * routes. Keep the fixed route selected until the allocator borrows
		 * this otherwise-idle pipeline for a Type-C display.
		 */
		ret = dcp->fixed_phy ?
			of_property_read_u32(dev->of_node, "mux-index", &mux_index) :
			-ENODATA;
		if (!ret) {
			dcp->fixed_mux_index = mux_index;
			dcp->xbar = devm_mux_control_get(dev, "dp-xbar");
			if (IS_ERR(dcp->xbar) && dcp->external_native) {
				ret = dcp_fixed_output_error(dcp, PTR_ERR(dcp->xbar), "dp-xbar");
				if (ret)
					return ret;
				goto fixed_done;
			}
			if (IS_ERR(dcp->xbar)) {
				dev_err(dev, "Failed to get dp-xbar: %ld\n", PTR_ERR(dcp->xbar));
				return PTR_ERR(dcp->xbar);
			}
			ret = mux_control_select(dcp->xbar, mux_index);
			if (ret)
				dev_warn(dev, "mux_control_select failed: %d\n", ret);
			else
				dcp->fixed_route_selected = true;

			/*
			 * Switch atcphy to DP-only. should move to a Macbook Pro
			 * 14-/16-inch specific DP-to-HDMI drm_bridge.
			 */
			dcp->typec_mux = fwnode_typec_mux_get(dev_fwnode(dcp->dev));
			if (!IS_ERR_OR_NULL(dcp->typec_mux)) {
				struct typec_altmode alt = {
					.svid = USB_TYPEC_DP_SID,
				};
				struct typec_mux_state state = {
					.alt = &alt,
					.mode = TYPEC_DP_STATE_C,
				};
				int ret = typec_mux_set(dcp->typec_mux, &state);
				dev_info(dev, "typec_mux_set() returned: %d\n", ret);
				if (!ret)
					dcp->phy_managed_by_typec = true;
			} else {
				dev_info(dev, "fwnode_typec_mux_get() returned: %ld\n",
						IS_ERR(dcp->typec_mux) ? PTR_ERR(dcp->typec_mux) : 0);
				dcp->typec_mux = NULL;
			}
		}
	}

fixed_done:
	ret = dcp_register_typec_routes(dcp);
	if (ret) {
		ret = dcp_native_routes_error(dcp, ret);
		if (ret)
			return ret;
		routes_failed = true;
	}

	/*
	 * The manual external path stays out of the panel's DRM device. A
	 * native external processor joins it, and probe completes for it even
	 * when its handoff is unusable, so that the panel is never held up.
	 */
	if (dcp->external && !dcp->external_native)
		return iomfb_v14_7_external_start(dcp);
	if (dcp->external_native) {
		ret = iomfb_v14_7_external_prepare(dcp);
		if (ret)
			return ret;
		if (routes_failed)
			iomfb_v14_7_external_refuse(dcp, "its Type-C display routes did not register");
	}

	ret = component_add(&pdev->dev, &dcp_comp_ops);
	if (ret && dcp->fw_compat == DCP_FIRMWARE_V_26_6)
		iomfb_v26_6_unbind(dcp);
	/* A failed bind run from here may already have started RTKit. */
	if (ret && dcp->fw_compat == DCP_FIRMWARE_V_14_7)
		iomfb_v14_7_remove(dcp);
	return ret;
}

/*
 * Marks a T8122 dcpext unusable, for good: its Type-C routes are disabled in
 * the device tree, so that a USB-C port controller probing later does not
 * wait for them, and a route already registered ignores mode changes (see
 * dcp_typec_route_set()), so that USB on the port goes on.
 */
static void dcp_ext_give_up(struct apple_dcp *dcp, int err, const char *when)
{
	WRITE_ONCE(dcp->ext_dead, true);
	dev_err(dcp->dev, "external display processor unusable (%s failed: %d); the internal display goes on without it\n",
		when, err);
	dcp_typec_routes_disable(dcp);
}

/*
 * Whether the DP PHY and display crossbar of every Type-C route are there.
 * Routes register one at a time, and one registered cannot be taken back,
 * since a port controller may hold it: a later route waiting for its
 * supplier would leave the dcpext unusable for the boot. Returns
 * -EPROBE_DEFER until all of them are; other errors are left to the route
 * probe, which reports them.
 */
static int dcp_ext_route_suppliers(struct device *dev)
{
	struct device_node *routes __free(device_node) =
		of_get_child_by_name(dev->of_node, "typec-routes");
	char name[16];
	u32 index;

	if (!routes)
		return 0;
	for_each_available_child_of_node_scoped(routes, route_np) {
		struct mux_control *xbar;
		struct phy *phy;

		if (of_property_read_u32(route_np, "reg", &index))
			continue;
		snprintf(name, sizeof(name), "typec%u", index);
		phy = of_phy_get(dev->of_node, name);
		if (PTR_ERR_OR_ZERO(phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (!IS_ERR(phy))
			of_phy_put(phy);
		xbar = mux_control_get(dev, name);
		if (PTR_ERR_OR_ZERO(xbar) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (!IS_ERR(xbar))
			mux_control_put(xbar);
	}
	return 0;
}

/*
 * The T8122 dcpext is a component of the panel's DRM device, so it must
 * always register one, or the panel stays on the boot framebuffer. Keep
 * deferring while its suppliers may still appear (bounded by the deferred
 * probe timeout); on any other failure join as an unusable pipe.
 */
static void dcp_unuse_dma_domain(void *data)
{
	iommu_device_unuse_default_domain(data);
}

static int dcp_platform_probe(struct platform_device *pdev)
{
	const struct apple_dcp_hw_data *hw = of_device_get_match_data(&pdev->dev);
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	void *group;
	int ret;

	if (hw && hw->firmware_compat == DCP_FIRMWARE_H17P)
		return dcp_platform_probe_h17p(pdev);

	/*
	 * The driver manages DMA ownership itself (driver_managed_dma), so
	 * that a retained H17P session keeps its domain use across unbind.
	 * Every other processor uses its default domain while bound, as the
	 * driver core would.
	 */
	ret = iommu_device_use_default_domain(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, dcp_unuse_dma_domain, dev);
	if (ret)
		return ret;

	if (!of_device_is_compatible(dev->of_node, "apple,t8122-dcpext"))
		return dcp_platform_probe_pipe(pdev);

	ret = dcp_ext_route_suppliers(dev);
	if (ret)
		goto unusable;

	group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;
	ret = dcp_platform_probe_pipe(pdev);
	if (!ret) {
		devres_remove_group(dev, group);
		return 0;
	}

	dcp = platform_get_drvdata(pdev);
	if (dcp && dcp->nr_typec_routes) {
		/* Routes may be in use already: keep them, and the pipe, as they are. */
		devres_remove_group(dev, group);
		dcp_ext_give_up(dcp, ret, "probe");
		return component_add(dev, &dcp_comp_ops);
	}

	devres_release_group(dev, group);
	platform_set_drvdata(pdev, NULL);
unusable:
	if (ret == -EPROBE_DEFER) {
		ret = driver_deferred_probe_check_state(dev);
		if (ret == -EPROBE_DEFER)
			return ret;
	}

	dcp = devm_kzalloc(dev, sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;
	dcp->dev = dev;
	dcp->ext_failsafe = true;
	platform_set_drvdata(pdev, dcp);
	dcp_ext_give_up(dcp, ret, "probe");
	return component_add(dev, &dcp_comp_ops);
}

static void dcp_platform_remove(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp && dcp->external && !dcp->external_native) {
		iomfb_v14_7_remove(dcp);
		return;
	}
	if (dcp && dcp->external_native)
		cancel_work_sync(&dcp->external_audio_work);
	component_del(&pdev->dev, &dcp_comp_ops);
	if (dcp && dcp->fw_compat == DCP_FIRMWARE_V_26_6)
		iomfb_v26_6_unbind(dcp);
	/* Unbind does not wait for RTKit callbacks, or run if bind failed late. */
	if (dcp && dcp->fw_compat == DCP_FIRMWARE_V_14_7)
		iomfb_v14_7_remove(dcp);
}

static void dcp_platform_shutdown(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp && dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_platform_shutdown_h17p(pdev);

	if (dcp && dcp->external && !dcp->external_native)
		return;
	component_del(&pdev->dev, &dcp_comp_ops);
}

/*
 * Why an external processor must keep the system awake, or NULL. The manual
 * path refuses sleep once its firmware was ever started. A native processor
 * refuses while its firmware starts or holds a display link, and while a
 * display is attached to it at all, lit or not: resume does not bring an
 * attached display back. Idle with nothing attached, its firmware stays
 * powered across s2idle like the panel's (the gate holds its CPU domain at
 * the active floor) and the next attach reconnects through it.
 *
 * A processor that failed (its start, or its session later) never refuses
 * sleep: it drives no display until reboot, attached or not, and its power
 * domains stay on across s2idle as they are, like those of a working one.
 */
static const char *dcp_external_sleep_vetoed(struct apple_dcp *dcp)
{
	const char *why;

	if (!dcp->external_native)
		return (atomic_read(&dcp->external_requested) ||
			(dcp->rtk && apple_rtkit_is_running(dcp->rtk)) ||
			dcpext_scanout_requested(dcp)) ?
		       "external firmware/scanout attempted" : NULL;
	why = iomfb_v14_7_external_busy(dcp);
	if (why)
		return why;
	if (iomfb_v14_7_external_failed(dcp)) {
		if (dcp->v14)
			dev_info(dcp->dev, "external display processor failed earlier: sleeping with it as it is\n");
		return NULL;
	}
	if (READ_ONCE(dcp->active_typec_route) || READ_ONCE(dcp->typec_cable_connected))
		return "a USB-C display is attached";
	if (dcp->hdmi_hpd && gpiod_get_value_cansleep(dcp->hdmi_hpd) > 0)
		return "an HDMI display is attached";
	return NULL;
}

/* dpm_prepare completes for every device before any dpm_suspend callback.
 * Veto early so Thunderbolt cannot tear down a working tunnel first. The
 * same HPD lock serializes this gate against explicit firmware startup.
 */
static int dcp_platform_prepare(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	const char *why;

	if (!dcp->external)
		return 0;
	mutex_lock(&dcp->hpd_mutex);
	why = dcp_external_sleep_vetoed(dcp);
	if (why) {
		mutex_unlock(&dcp->hpd_mutex);
		if (dcp->external_native)
			dev_warn(dev, "refusing system sleep: %s\n", why);
		else
			dev_warn(dev, "external firmware/scanout attempted: refusing PM prepare; reboot required for retained DMA\n");
		return -EBUSY;
	}
	WRITE_ONCE(dcp->external_suspended, true);
	mutex_unlock(&dcp->hpd_mutex);
	return 0;
}

static void dcp_platform_complete(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (!dcp->external)
		return;
	mutex_lock(&dcp->hpd_mutex);
	WRITE_ONCE(dcp->external_suspended, false);
	mutex_unlock(&dcp->hpd_mutex);
}

static int dcp_platform_suspend(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_platform_suspend_h17p(dev);
	if (READ_ONCE(dcp->ext_dead))
		return 0;
	/* The 14.7 panel: its messages wait in the mailbox until resume. */
	if (dcp_is_v14_7_panel(dcp))
		iomfb_v14_7_suspend(dcp);

	/* Serialize PM against explicit bring-up, including queued startup work.
	 * Before a startup attempt the device remains suspendable. After any
	 * attempt we cannot prove DMA/firmware quiescence, even on failure.
	 */
	if (dcp->external) {
		const char *why;

		mutex_lock(&dcp->hpd_mutex);
		why = dcp_external_sleep_vetoed(dcp);
		if (why) {
			mutex_unlock(&dcp->hpd_mutex);
			if (dcp->external_native)
				dev_warn(dev, "suspend refused: %s\n", why);
			else
				dev_warn(dev, "external firmware/scanout attempted: suspend refused; retained DMA requires reboot\n");
			return -EBUSY;
		}
		WRITE_ONCE(dcp->external_suspended, true);
		mutex_unlock(&dcp->hpd_mutex);
	}
	/*
	 * The Type-C route reports cable removal through
	 * dcp_dptx_disconnect_oob(). A DP tunnel kept through the sleep stays
	 * connected, and resume powers the CRTC back up through dcp_poweron()
	 * as after DPMS off.
	 */
	dcp_disable_typec_work(dcp, false);
	cancel_delayed_work_sync(&dcp->swap_watchdog_wq);
	if (dcp->external_native)
		disable_work_sync(&dcp->external_audio_work);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dcp->hdmi_hpd_irq) {
		disable_irq(dcp->hdmi_hpd_irq);
		if (!dcp->active_typec_route) {
			disconnected_hpd_event(dcp->connector);
			dcp_dptx_disconnect(dcp, 0);
		}
	}
	/*
	 * Set the device as a wakeup device, which forces its power
	 * domains to stay on. We need this as we do not support full
	 * shutdown properly yet.
	 */
	device_set_wakeup_path(dev);

	return 0;
}

static int dcp_platform_resume(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return dcp_platform_resume_h17p(dev);
	if (READ_ONCE(dcp->ext_dead))
		return 0;
	if (dcp_is_v14_7_panel(dcp))
		iomfb_v14_7_resume(dcp);

	dcp_enable_typec_work(dcp);
	/* The HDMI output's PHY and crossbar may have lost their setup in sleep. */
	if (dcp->external_native)
		dcp_fabric_hdmi_reinit(dcp, "after system sleep");
	/* Observe future edges before sampling any edges lost in sleep. */
	if (dcp->hdmi_hpd_irq)
		dcp_fabric_run_resume(&dcp_resume_ops, dcp);
	if (dcp->external_native)
		enable_work(&dcp->external_audio_work);

	if (dcp->avep)
		av_service_connect(dcp);

	return 0;
}


/* ---------------------------------------------------------------------------
 * H17-generation firmware: T8140 (H17P, internal and the external dispext0)
 * and T8142 (H17G). aurora-wip's processor lifecycle.
 * ------------------------------------------------------------------------- */

struct dcp_session_ref {
	struct list_head link;
	struct device *dev;
	struct iommu_group *group;
};

static LIST_HEAD(dcp_retained_sessions);
static DEFINE_MUTEX(dcp_retained_sessions_lock);

static bool dcp_session_retained(struct device *dev)
{
	struct dcp_session_ref *session;
	bool retained = false;

	mutex_lock(&dcp_retained_sessions_lock);
	list_for_each_entry(session, &dcp_retained_sessions, link) {
		if (dev_fwnode(session->dev) == dev_fwnode(dev)) {
			retained = true;
			break;
		}
	}
	mutex_unlock(&dcp_retained_sessions_lock);
	return retained;
}

static int dcp_pin_live_session(struct apple_dcp *dcp)
{
	struct dcp_session_ref *session;

	if ((!dcp->hw.adopt_live_session &&
	     dcp->hw.iomfb_method_profile !=
	     DCP_IOMFB_METHODS_H17P_EXTERNAL) ||
	    dcp->fw_compat != DCP_FIRMWARE_H17P ||
	    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G || dcp->retain_dma)
		return 0;
	session = kzalloc_obj(*session);
	if (!session)
		return -ENOMEM;

	/* Keep the DMA device, domain usage and driver code valid until reset. */
	session->dev = get_device(dcp->dev);
	session->group = iommu_group_get(dcp->dev);
	__module_get(THIS_MODULE);
	WRITE_ONCE(dcp->retain_dma, true);
	mutex_lock(&dcp_retained_sessions_lock);
	list_add_tail(&session->link, &dcp_retained_sessions);
	mutex_unlock(&dcp_retained_sessions_lock);
	dev_info(dcp->dev, "live H17P DMA resources retained until reboot; rebind unsupported\n");
	return 0;
}

static void dcp_release_dma_domain(void *data)
{
	struct apple_dcp *dcp = data;

	if (!dcp->retain_dma)
		iommu_device_unuse_default_domain(dcp->dev);
}

static void dcp_release_context(void *data)
{
	struct apple_dcp *dcp = data;

	if (!dcp->retain_dma)
		kfree(dcp);
}

static void dcp_release_rtkit(void *data)
{
	struct apple_dcp *dcp = data;

	if (dcp->retain_dma) {
		WRITE_ONCE(dcp->quiescing, true);
		apple_rtkit_free_retaining_buffers(dcp->rtk);
	} else {
		apple_rtkit_free(dcp->rtk);
	}
	dcp->rtk = NULL;
}

bool dcp_h17p_external_supported(struct device_node *np)
{
	u32 firmware[DCP_EXTERNAL_FW_CELLS], compat[DCP_EXTERNAL_FW_CELLS];
	int firmware_cells, compat_cells;

	if (!of_machine_is_compatible("apple,j700") ||
	    !of_device_is_compatible(np, "apple,t8140-dcpext"))
		return false;
	firmware_cells = of_property_read_variable_u32_array(
		np, "apple,firmware-version", firmware,
		DCP_EXTERNAL_FW_CELLS, DCP_EXTERNAL_FW_CELLS);
	compat_cells = of_property_read_variable_u32_array(
		np, "apple,firmware-compat", compat,
		DCP_EXTERNAL_FW_CELLS, DCP_EXTERNAL_FW_CELLS);
	return apple_dcp_external_firmware_supported(firmware, firmware_cells,
						    compat, compat_cells);
}

bool dcp_h17p_external_ready(struct device_node *np)
{
	struct platform_device *pdev;
	struct apple_dcp *dcp;
	bool ready = false;

	if (!dcp_h17p_external_supported(np))
		return false;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return false;
	/*
	 * Called after component binding. component_add() may synchronously
	 * bind the master before the platform probe itself returns, so the
	 * driver core's bound flag is not a readiness condition here.
	 */
	dcp = platform_get_drvdata(pdev);
	ready = dcp && READ_ONCE(dcp->external_prepared) &&
		!READ_ONCE(dcp->quiescing) && !READ_ONCE(dcp->crashed);
	put_device(&pdev->dev);
	return ready;
}

static bool dcp_firmware_metadata_enabled(struct apple_dcp *dcp)
{
	return dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL;
}

void dcp_firmware_metadata_connection(struct apple_dcp *dcp, u64 generation, bool connected)
{
	unsigned long flags;

	if (!dcp_firmware_metadata_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	if (generation != dcp->metadata.epoch.generation || !connected) {
		dcp->rate_proof.connection_sequence++;
		dcp->rate_proof.sequence++;
		dcp->rate_proof.rate.valid = false;
		dcp->rate_proof.lanes.valid = false;
		dcp->metadata.attributes_valid = false;
		dcp->metadata.timing_valid = false;
		dcp->metadata.hpd_seen = false;
	}
	dcp_metadata_epoch_connection(&dcp->metadata.epoch, generation, connected);
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
}

void dcp_firmware_metadata_hotplug(struct apple_dcp *dcp, bool connected)
{
	struct apple_connector *con = READ_ONCE(dcp->connector);
	unsigned long flags;

	if (!dcp_firmware_metadata_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	if (connected) {
		dcp->metadata.hpd_seen = true;
		/*
		 * Fresh proof may precede HPD assertion after a same-cable loss.
		 * Keep invalidated firmware metadata and old receipt tickets revoked.
		 */
		if (READ_ONCE(dcp->typec_cable_connected) &&
		    dcp->metadata.epoch.generation == READ_ONCE(dcp->typec_generation))
			dcp->metadata.epoch.connected = true;
	} else if (dcp->metadata.hpd_seen) {
		dcp->metadata.hpd_seen = false;
		dcp->rate_proof.connection_sequence++;
		dcp->rate_proof.sequence++;
		dcp->rate_proof.rate.valid = false;
		dcp->rate_proof.lanes.valid = false;
		if (con && READ_ONCE(con->dcp) == to_platform_device(dcp->dev))
			WRITE_ONCE(con->edid_invalidation_pending, true);
		dcp_metadata_epoch_connection(&dcp->metadata.epoch,
					      dcp->metadata.epoch.generation, false);
		dcp->metadata.attributes_valid = false;
		dcp->metadata.timing_valid = false;
	}
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	if (connected)
		dcp_180hz_notify(dcp);
}

void dcp_firmware_metadata_begin(struct apple_dcp *dcp)
{
	unsigned long flags;

	spin_lock_irqsave(&dcp->metadata.lock, flags);
	dcp->chunks.metadata_ticket = dcp_metadata_begin(&dcp->metadata.epoch);
	dcp->chunks.received = 0;
	dcp->chunks.contiguous = true;
	dcp->chunks.payload_complete = false;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
}

static void dcp_firmware_metadata_notify(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool ready;

	spin_lock_irqsave(&dcp->metadata.lock, flags);
	ready = dcp->metadata.attributes_valid && dcp->metadata.timing_valid &&
		dcp_metadata_matches(&dcp->metadata.epoch, dcp->metadata.attributes_ticket) &&
		dcp_metadata_matches(&dcp->metadata.epoch, dcp->metadata.timing_ticket);
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	if (ready && dcp->connector && READ_ONCE(dcp->connector->connected))
		schedule_work(&dcp->connector->hotplug_wq);
}

static bool dcp_firmware_metadata_complete(struct apple_dcp *dcp)
{
	return dcp->chunks.contiguous && dcp->chunks.payload_complete &&
	       dcp_metadata_matches(&dcp->metadata.epoch, dcp->chunks.metadata_ticket);
}

void dcp_firmware_metadata_attributes(struct apple_dcp *dcp,
				      const struct dcp_display_metadata *attributes)
{
	unsigned long flags;

	if (!dcp_firmware_metadata_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	if (dcp_metadata_matches(&dcp->metadata.epoch, dcp->chunks.metadata_ticket)) {
		dcp->metadata.attributes_valid = attributes && dcp_firmware_metadata_complete(dcp);
		if (dcp->metadata.attributes_valid) {
			dcp->metadata.attributes = *attributes;
			dcp->metadata.attributes_ticket = dcp->chunks.metadata_ticket;
		}
	}
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	dcp_firmware_metadata_notify(dcp);
}

void dcp_firmware_metadata_timing(struct apple_dcp *dcp)
{
	unsigned long flags;
	unsigned int i;

	if (!dcp_firmware_metadata_enabled(dcp))
		return;
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	if (!dcp_metadata_matches(&dcp->metadata.epoch, dcp->chunks.metadata_ticket))
		goto unlock;
	dcp->metadata.timing_valid = false;
	if (!dcp_firmware_metadata_complete(dcp))
		goto unlock;
	for (i = 0; i < dcp->nr_modes; i++) {
		if (!(dcp->modes[i].mode.type & DRM_MODE_TYPE_PREFERRED) ||
		    !dcp->modes[i].metadata_timing_valid)
			continue;
		dcp->metadata.preferred = dcp->modes[i].mode;
		dcp->metadata.timing_ticket = dcp->chunks.metadata_ticket;
		dcp->metadata.timing_valid = true;
		break;
	}
unlock:
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	dcp_firmware_metadata_notify(dcp);
}

bool dcp_firmware_metadata_current(struct apple_dcp *dcp, u64 generation)
{
	unsigned long flags;
	bool valid;

	spin_lock_irqsave(&dcp->metadata.lock, flags);
	valid = !dcp->metadata.epoch.invalidated && dcp->metadata.epoch.connected &&
		generation == dcp->metadata.epoch.generation;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	return valid;
}

const struct drm_edid *dcp_firmware_edid(struct apple_dcp *dcp, u64 *generation)
{
	struct dcp_display_metadata attributes;
	struct drm_display_mode preferred;
	const struct drm_edid *edid;
	unsigned long flags;
	bool valid;

	if (!dcp_firmware_metadata_enabled(dcp))
		return ERR_PTR(-ENODEV);
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	valid = dcp->metadata.attributes_valid && dcp->metadata.timing_valid &&
		dcp_metadata_matches(&dcp->metadata.epoch, dcp->metadata.attributes_ticket) &&
		dcp_metadata_matches(&dcp->metadata.epoch, dcp->metadata.timing_ticket);
	if (valid) {
		attributes = dcp->metadata.attributes;
		preferred = dcp->metadata.preferred;
		*generation = dcp->metadata.epoch.generation;
	}
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	if (!valid)
		return ERR_PTR(-ENODATA);
	edid = dcp_metadata_edid(&attributes, &preferred);
	spin_lock_irqsave(&dcp->metadata.lock, flags);
	valid = !dcp->metadata.epoch.invalidated && dcp->metadata.epoch.connected &&
		*generation == dcp->metadata.epoch.generation;
	spin_unlock_irqrestore(&dcp->metadata.lock, flags);
	if (!valid && !IS_ERR(edid)) {
		drm_edid_free(edid);
		return ERR_PTR(-ESTALE);
	}
	return edid;
}

/*
 * H17P: deferred to the connector hotplug work, which updates the connector
 * with dcp_update_connector_dimensions().
 */
void dcp_set_dimensions_h17p(struct apple_dcp *dcp)
{
	int i;
	int width_mm = dcp->width_mm;
	int height_mm = dcp->height_mm;

	if (width_mm == 0 || height_mm == 0) {
		width_mm = dcp->panel.width_mm;
		height_mm = dcp->panel.height_mm;
	}

	/*
	 * This callback runs on the ordered RTKit RX worker. A mode probe
	 * can hold mode_config.mutex while waiting for an AFK reply which
	 * that same worker still has to dispatch. Defer the DRM update.
	 */
	if (dcp->connector && !READ_ONCE(dcp->quiescing))
		schedule_work(&dcp->connector->hotplug_wq);

	/*
	 * Fix up any probed modes. Modes are created when parsing
	 * TimingElements, dimensions are calculated when parsing
	 * DisplayAttributes, and TimingElements may be sent first
	 */
	for (i = 0; i < dcp->nr_modes; ++i) {
		dcp->modes[i].mode.width_mm = width_mm;
		dcp->modes[i].mode.height_mm = height_mm;
	}
}

void dcp_update_connector_dimensions(struct apple_dcp *dcp,
				     struct drm_connector *connector)
{
	int width_mm = READ_ONCE(dcp->width_mm);
	int height_mm = READ_ONCE(dcp->height_mm);

	lockdep_assert_held(&connector->dev->mode_config.mutex);
	if (!width_mm || !height_mm) {
		width_mm = dcp->panel.width_mm;
		height_mm = dcp->panel.height_mm;
	}
	connector->display_info.width_mm = width_mm;
	connector->display_info.height_mm = height_mm;
}


static struct apple_dcp_afkep *dcp_afkep(struct apple_dcp *dcp, u8 endpoint)
{
	/* DPAV 0x2a on H17P overlaps the legacy DPTX endpoint number. */
	if (dcp->dcpavservep && endpoint == dcp->dcpavservep->endpoint)
		return dcp->dcpavservep;
	if (dcp->dptxep && endpoint == dcp->dptxep->endpoint)
		return dcp->dptxep;
	switch (endpoint) {
	case AV_ENDPOINT:
		return dcp->avep;
	case SYSTEM_ENDPOINT:
		return dcp->systemep;
	case DISP0_ENDPOINT:
		return dcp->ibootep;
	case DPTX_CONTROLLER_ENDPOINT:
		return dcp->dptx_controller_ep;
	default:
		return NULL;
	}
}


static void dcp_recv_msg_h17p(struct apple_dcp *dcp, u8 endpoint, u64 message)
{
	struct apple_dcp_afkep *ep;

	if (READ_ONCE(dcp->quiescing))
		return;

	trace_dcp_recv_msg(dcp, endpoint, message);

	/*
	 * H17P starts endpoints that have no AFK instance behind them (the
	 * remote allocator endpoint, for one).  Log their messages rather than
	 * dereferencing a NULL afkep or warning about an unknown endpoint.
	 */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P && endpoint != IOMFB_ENDPOINT &&
	    !dcp_afkep(dcp, endpoint)) {
		dev_dbg_ratelimited(dcp->dev, "ep %#04x: %#llx\n", endpoint,
				    message);
		return;
	}

	if (endpoint == IOMFB_ENDPOINT)
		return iomfb_recv_msg(dcp, message);

	ep = dcp_afkep(dcp, endpoint);

	if (!ep) {
		dev_warn_ratelimited(dcp->dev,
				     "dropping message for unhandled endpoint %#x\n",
				     endpoint);
		return;
	}

	afk_receive_message(ep, message);
}

static void dcp_rtk_crashed_h17p(struct apple_dcp *dcp)
{
	if (READ_ONCE(dcp->quiescing))
		return;

	dcp->crashed = true;
	dev_err(dcp->dev, "DCP has crashed\n");
	if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		/* No in-place restart exists; the last scanout stays retained. */
		dev_err(dcp->dev, "a reboot is required to restore the display\n");
		/*
		 * Nothing will complete a present that was in flight.  Before
		 * the pipeline is linked there is no event to complete.
		 */
		if (READ_ONCE(dcp->crtc))
			schedule_work(&dcp->vblank_wq);
	}
	if (dcp->connector) {
		dcp->connector->connected = 0;
		/* The cached EDID is owned under edid_lock. */
		apple_connector_invalidate_edid(dcp->connector);
		schedule_work(&dcp->connector->hotplug_wq);
	}
	complete(&dcp->start_done);
}

/*
 * Check that every IOMMU page of [iova, iova + size) translates to the same
 * offset from @phys as it has from @iova.
 */
static int dcp_check_iova_contiguous(struct iommu_domain *domain,
				     dma_addr_t iova, size_t size,
				     phys_addr_t phys)
{
	unsigned long pgsize = domain->pgsize_bitmap ?
			       1UL << __ffs(domain->pgsize_bitmap) : PAGE_SIZE;
	dma_addr_t end = iova + size;
	dma_addr_t addr = iova;

	while (addr < end) {
		if (iommu_iova_to_phys(domain, addr) != phys + (addr - iova))
			return -EINVAL;
		addr = ALIGN_DOWN(addr, pgsize) + pgsize;
	}

	return 0;
}

static int dcp_rtk_shmem_setup_h17p(struct apple_dcp *dcp, struct apple_rtkit_shmem *bfr)
{
	int ret;

	if (bfr->iova) {
		struct iommu_domain *domain =
			iommu_get_domain_for_dev(dcp->dev);
		phys_addr_t phy_addr;

		if (!domain)
			return -ENOMEM;

		// TODO: get map from device-tree
		phy_addr = iommu_iova_to_phys(domain, bfr->iova);
		if (!phy_addr)
			return -ENOMEM;

		/* memremap() below needs the whole buffer physically contiguous */
		ret = dcp_check_iova_contiguous(domain, bfr->iova, bfr->size,
						phy_addr);
		if (ret) {
			dev_err(dcp->dev,
				"shmem_setup: iova %pad (%#zx bytes) is not physically contiguous\n",
				&bfr->iova, bfr->size);
			return ret;
		}

		/* Firmware writes without CPU cache maintenance. */
		bfr->buffer = memremap(phy_addr, bfr->size, MEMREMAP_WC);
		if (!bfr->buffer)
			return -ENOMEM;

		bfr->is_mapped = true;
		dev_info(dcp->dev,
			 "shmem_setup: iova: %lx -> pa: %lx -> iomem: %lx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)phy_addr,
			 (uintptr_t)bfr->buffer);
	} else {
		bfr->buffer = dma_alloc_coherent(dcp->dev, bfr->size,
						 &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;

		dev_info(dcp->dev, "shmem_setup: iova: %lx, buffer: %lx\n",
			 (uintptr_t)bfr->iova, (uintptr_t)bfr->buffer);
	}

	return 0;
}

static const struct apple_rtkit_ops external_h17p_rtkit_ops = {
	.flags = APPLE_RTKIT_COPROC_PLACES_BUFFERS,
	.crashed = dcp_rtk_crashed,
	.recv_message = dcp_recv_msg,
	.shmem_setup = dcp_rtk_shmem_setup,
	.shmem_destroy = dcp_rtk_shmem_destroy,
};

static int dcp_crtc_atomic_check_h17p(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	struct platform_device *pdev = to_apple_crtc(crtc)->dcp;
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	struct drm_crtc_state *crtc_state;
	bool needs_modeset;

	if (dcp->crashed)
		return -EINVAL;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* External H17P does not program adaptive sync. */
	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	    crtc_state->vrr_enabled)
		return -EINVAL;

	needs_modeset = drm_atomic_crtc_needs_modeset(crtc_state) ||
			!READ_ONCE(dcp->mode_state.valid);
	if (!needs_modeset && (!dcp->connector || !dcp->connector->connected)) {
		/*
		 * Resume restores the mode before the firmware reports the
		 * display back, so a plane-only commit lands here while the
		 * connector is still marked disconnected.  Rejecting it makes
		 * the compositor fail every flip and give up on the output;
		 * dcp_flush() defers the commit until the link returns.
		 */
		dev_dbg(dcp->dev,
			"crtc_atomic_check: deferring commit, link still down\n");
	}

	return 0;
}

static int dcp_dptx_connect_raw(struct apple_dcp *dcp, u32 port, bool *waiting_service)
{
	unsigned long timeout;
	int ret = 0;

	if (waiting_service)
		*waiting_service = false;
	if (!dcp->phy) {
		dev_warn(dcp->dev, "dcp_dptx_connect: missing phy\n");
		return -ENODEV;
	}
	dev_dbg(dcp->dev,
		"%s(port=%d) target=%u:%u typec=%d route=%s conn_type=%d connected=%d\n",
		__func__, port, dcp->dptx_die, dcp->dptx_phy,
		dcp_is_typec_output(dcp),
		dcp->active_typec_route ? "borrowed" : "fixed",
		dcp->connector_type, dcp->dptxport[port].connected);

	mutex_lock(&dcp->hpd_mutex);
	if (!dptx_service_available(&dcp->dptxport[port])) {
		dev_warn(dcp->dev, "dcp_dptx_connect: dptx service for port %d not enabled\n", port);
		ret = -ENODEV;
		/* No firmware request has been made; the first publication can retry. */
		if (waiting_service && !READ_ONCE(dcp->dptxport[port].closed)) {
			*waiting_service = true;
			ret = -EAGAIN;
		}
		goto out_unlock;
	}

	if (dcp->dptxport[port].connected)
		goto out_unlock;

	reinit_completion(&dcp->dptxport[port].linkcfg_completion);
	dcp->dptxport[port].atcphy = dcp->phy;
	ret = dptxport_validate_connection(dcp->dptxport[port].service,
					   dcp->dptx_dfp_port,
					   dcp->dptx_phy, dcp->dptx_die);
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to validate DPTX target %u:%u: %d\n",
			dcp->dptx_die, dcp->dptx_phy, ret);
		goto out_unlock;
	}

	ret = dptxport_connect(dcp->dptxport[port].service,
			       dcp->dptx_dfp_port,
			       dcp->dptx_phy, dcp->dptx_die,
		       dcp_is_typec_output(dcp));
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to connect DPTX target %u:%u: %d\n",
			dcp->dptx_die, dcp->dptx_phy, ret);
		goto out_unlock;
	}

	ret = dptxport_request_display(dcp->dptxport[port].service);
	if (ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: failed to request display: %d\n",
			ret);
		goto out_release;
	}
	dcp->dptxport[port].connected = true;
	/* Not a T602x USB4 tunnel: the plain HPD assertion and timeout apply. */
	if (dcp_is_typec_output(dcp)) {
		ret = dptxport_set_hpd(dcp->dptxport[port].service, true);
		if (ret) {
			dev_err(dcp->dev,
				"dcp_dptx_connect: failed to assert Type-C HPD: %d\n",
				ret);
			dcp->dptxport[port].connected = false;
			goto out_release;
		}
	}

	mutex_unlock(&dcp->hpd_mutex);
	timeout = DPTX_CONNECT_TIMEOUT;
	ret = wait_for_completion_timeout(&dcp->dptxport[port].linkcfg_completion,
					  timeout);
	if (!ret) {
		dev_err(dcp->dev,
			"dcp_dptx_connect: timed out waiting for port %u link configuration\n",
			port);
		ret = -ETIMEDOUT;
		goto out_disconnect;
	}

	dev_dbg(dcp->dev, "dcp_dptx_connect: waited %d ms for link\n",
		jiffies_to_msecs(timeout - ret));

	usleep_range(5, 10);

	if (dcp->connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		dptxport_set_hpd(dcp->dptxport[port].service, true);

	if (dcp->avep)
		av_service_connect(dcp);

	return 0;

out_disconnect:
	mutex_lock(&dcp->hpd_mutex);
	dcp->dptxport[port].connected = false;
out_release:
	dptxport_release_display(dcp->dptxport[port].service);

out_unlock:
	mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

static void dcp_external_report_disconnected(struct apple_dcp *dcp)
{
	unsigned int action;

	guard(mutex)(&dcp->iomfb.lock);
	/* A paused callback from the old sink must not be replayed on wake. */
	dcp->external_hotplug_pending = false;
	action = dcp_mode_hotplug(&dcp->mode_state, false,
				  dcp->connector ? &dcp->connector->connected : NULL);
	if (dcp->connector) {
		WRITE_ONCE(dcp->connector->edid_invalidation_pending, true);
		action |= DCP_HOTPLUG_NOTIFY;
	}
	/* The existing worker delivers DRM notifications outside our locks. */
	dcp_handle_hotplug_actions(dcp, action);
}

static void dcp_external_publish_generation(struct apple_dcp *dcp)
{
	lockdep_assert_held(&dcp->external_lock);
	guard(mutex)(&dcp->iomfb.lock);
	dcp->external_rx_generation = dcp->external_generation;
}

/* external_lock is never taken by the serialized IOMFB receive callbacks. */
static int dcp_external_link_locked(struct apple_dcp *dcp)
{
	bool waiting_service;
	int ret;

	lockdep_assert_held(&dcp->external_lock);
	if (!dcp->external_prepared)
		return -EAGAIN;
	if (dcp->external_error)
		return dcp->external_error;
	if (!dcp->external_cable)
		return -ENODEV;
	ret = dcp_dptx_connect_raw(dcp, 0, &waiting_service);
	if (ret && !waiting_service)
		dcp->external_error = ret;
	return ret;
}

static int dcp_external_start_locked(struct apple_dcp *dcp)
{
	int ret;

	lockdep_assert_held(&dcp->external_lock);
	ret = dcp_external_link_locked(dcp);
	if (ret || dcp->external_running)
		return ret;
	iomfb_pause_presentation(dcp, true);
	/* Even a timed-out first activation can have changed firmware state. */
	dcp->external_power_attempted = true;
	ret = iomfb_external_power_h17p(dcp, true);
	if (ret) {
		dcp->external_error = ret;
		return ret;
	}
	dcp->external_running = true;
	dcp->external_stopped = false;
	WRITE_ONCE(dcp->pipe_enabled_h17p, true);
	WRITE_ONCE(dcp->typec_crtc_off, false);
	iomfb_pause_presentation(dcp, false);
	return 0;
}

static int dcp_dptx_connect_h17p(struct apple_dcp *dcp, u32 port)
{
	if (!dcp_is_external_h17p(dcp))
		return dcp_dptx_connect_raw(dcp, port, NULL);
	guard(mutex)(&dcp->external_lock);
	return dcp_external_link_locked(dcp);
}

/* The generation token belongs to the CRTC that was actually enabled. */
static int dcp_external_stop_locked(struct apple_dcp *dcp, u64 generation)
{
	int ret;

	lockdep_assert_held(&dcp->external_lock);
	if (generation != dcp->external_generation)
		return 0;
	if (dcp->external_error)
		return dcp->external_error;
	if (dcp->external_stopped && !READ_ONCE(dcp->dptxport[0].connected))
		return 0;
	iomfb_pause_presentation(dcp, true);
	dcp->external_running = false;
	/* Route setup can call this before IOMFB or its shared memory exists. */
	if (dcp->external_stopped || !dcp->external_power_attempted)
		goto release_link;
	ret = iomfb_queue_settle(dcp, msecs_to_jiffies(500));
	if (!ret)
		ret = iomfb_external_power_h17p(dcp, false);
	if (!ret)
		ret = iomfb_queue_settle(dcp, msecs_to_jiffies(500));
	if (ret)
		goto failed;
	/* This is a transport acknowledgment, not permission to free scanout. */
	WRITE_ONCE(dcp->pipe_enabled_h17p, false);
release_link:
	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (dcp->dptxport[0].connected) {
			if (!dptx_service_available(&dcp->dptxport[0])) {
				ret = -ENODEV;
				goto failed;
			}
			ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
			if (!ret)
				ret = dptxport_release_display(dcp->dptxport[0].service);
			if (ret)
				goto failed;
			dcp->dptxport[0].connected = false;
		}
	}
	dcp->external_stopped = true;
	WRITE_ONCE(dcp->typec_crtc_off, true);
	if (dcp->avep)
		av_service_disconnect(dcp);
	return 0;
failed:
	/* A retry could race a late reply. Keep this session blocked and owned. */
	dcp->external_error = ret;
	return ret;
}

static int dcp_external_disconnect(struct apple_dcp *dcp, bool physical)
{
	int ret;

	cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
	cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
	guard(mutex)(&dcp->external_lock);
	/* Invalidate physical EDID before waiting for firmware power/link teardown. */
	if (physical) {
		WRITE_ONCE(dcp->external_physically_absent, true);
		scoped_guard(mutex, &dcp->hpd_mutex) {
			WRITE_ONCE(dcp->typec_cable_connected, false);
			dcp->typec_generation++;
			dcp_firmware_metadata_connection(dcp, dcp->typec_generation, false);
		}
		/* HPD is fenced; retire the AV owner after releasing hpd_mutex. */
		dpavservep_detach(dcp);
	}
	ret = dcp_external_stop_locked(dcp, dcp->external_generation);
	if (physical) {
		dcp->external_cable = false;
		dcp->external_generation++;
		dcp_external_publish_generation(dcp);
		/* A missing cable is reported even when its hardware remains owned. */
		dcp_external_report_disconnected(dcp);
	}
	return ret;
}

static void dcp_external_stop_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, external_stop_work);

	u64 generation;

	scoped_guard(mutex, &dcp->iomfb.lock)
		generation = dcp->external_stop_generation;
	cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
	cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
	guard(mutex)(&dcp->external_lock);
	if (generation == dcp->external_generation) {
		/* The deferred disconnect also covers missing out-of-band HPD. */
		dpavservep_detach(dcp);
		dcp_external_stop_locked(dcp, generation);
		dcp_external_report_disconnected(dcp);
	}
}

bool dcp_external_hotplug(struct apple_dcp *dcp, bool connected)
{
	if (!dcp_is_external_h17p(dcp))
		return false;
	/* No lifecycle or HPD mutex may be acquired from the RX callback. */
	lockdep_assert_held(&dcp->iomfb.lock);
	if (READ_ONCE(dcp->quiescing) || READ_ONCE(dcp->external_physically_absent))
		return true;
	if (dcp->iomfb.presentation_paused) {
		dcp->external_hotplug_pending = true;
		dcp->external_hotplug_connected = connected;
		/* Publish initial/new-cable discovery, never same-cable DPMS wake. */
		if (connected && READ_ONCE(dcp->external_cable) &&
		    !READ_ONCE(dcp->external_error) &&
		    (!READ_ONCE(dcp->external_power_attempted) ||
		     (READ_ONCE(dcp->external_stopped) &&
		      READ_ONCE(dcp->external_rx_generation) !=
		      READ_ONCE(dcp->external_crtc_generation) &&
		      READ_ONCE(dcp->dptxport[0].connected)))) {
			unsigned int action = dcp_mode_hotplug(&dcp->mode_state, true,
							     dcp->connector ?
							     &dcp->connector->connected : NULL);

			dcp_handle_hotplug_actions(dcp, action);
		}
		return true;
	}
	if (!connected && !READ_ONCE(dcp->iomfb.presentation_paused)) {
		/* Fence admission in RX before a busy actor can delay teardown. */
		dcp_firmware_metadata_hotplug(dcp, false);
		dcp->external_stop_generation = dcp->external_rx_generation;
		schedule_work(&dcp->external_stop_work);
		return true;
	}
	return READ_ONCE(dcp->iomfb.presentation_paused);
}


static void dcp_placeholder_edid_work_h17p(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     placeholder_edid_wq);
	struct apple_epic_service *service;
	u64 generation;
	int ret;

	mutex_lock(&dcp->hpd_mutex);
	generation = dcp->placeholder_generation;
	if (!dcp->typec_cable_connected || !dcp->dptxport[0].connected ||
	    !dptx_service_available(&dcp->dptxport[0]) || generation != dcp->typec_generation)
		goto out_unlock;
	service = dcp->dptxport[0].service;

	/*
	 * Some adapters answer the first connection with a 1024x768
	 * placeholder and publish the panel EDID only after HPD drops
	 * and returns. One pulse; a second placeholder is left alone.
	 */
	ret = dptxport_set_hpd(service, false);
	if (ret) {
		dev_info(dcp->dev, "placeholder EDID: HPD drop failed: %d\n",
			 ret);
		goto out_unlock;
	}
	mutex_unlock(&dcp->hpd_mutex);

	msleep(1000);

	mutex_lock(&dcp->hpd_mutex);
	if (!dcp->typec_cable_connected || !dcp->dptxport[0].connected ||
	    generation != dcp->typec_generation)
		goto out_unlock;

	ret = dptxport_set_hpd(service, true);
	if (ret)
		dev_info(dcp->dev, "placeholder EDID: HPD assert failed: %d\n",
			 ret);
out_unlock:
	mutex_unlock(&dcp->hpd_mutex);
}

static void dcp_typec_reconnect_work_h17p(struct work_struct *work)
{
	struct apple_dcp *dcp =
		container_of(to_delayed_work(work), struct apple_dcp,
			     typec_reconnect_wq);
	int ret;

	if (!READ_ONCE(dcp->typec_cable_connected))
		return;

	ret = dcp_dptx_connect(dcp, 0);
	if (!ret) {
		dcp->typec_reconnect_tries = 0;
		return;
	}

	/* Not a T602x USB4 tunnel: the full retry budget applies. */
	if (++dcp->typec_reconnect_tries < DPTX_RECONNECT_RETRIES) {
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq,
				 DPTX_RECONNECT_DELAY);
		return;
	}

	dev_err(dcp->dev, "Type-C DPTX reconnect failed after %u retries: %d\n",
		dcp->typec_reconnect_tries, ret);
}

static void disconnected_hpd_event_h17p(struct apple_connector *con)
{
	if (!con)
		return;

	/* May run with DRM modeset locks held; defer property/cache cleanup. */
	WRITE_ONCE(con->connected, false);
	WRITE_ONCE(con->edid_invalidation_pending, true);
	schedule_work(&con->hotplug_wq);
}

static int dcp_dptx_disconnect_h17p(struct apple_dcp *dcp, u32 port)
{
	if (dcp_is_external_h17p(dcp))
		return dcp_external_disconnect(dcp, false);

	dev_info(dcp->dev, "%s(port=%d)\n", __func__, port);

	mutex_lock(&dcp->hpd_mutex);
	if (dptx_service_available(&dcp->dptxport[port]) && dcp->dptxport[port].connected) {
		dptxport_release_display(dcp->dptxport[port].service);
		dcp->dptxport[port].connected = false;
	}
	mutex_unlock(&dcp->hpd_mutex);

	return 0;
}

static int dcp_dptx_connect_oob_h17p(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp_is_external_h17p(dcp)) {
		bool new_cable;

		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		cancel_work_sync(&dcp->external_stop_work);
		guard(mutex)(&dcp->external_lock);
		new_cable = !dcp->external_cable;
		if (new_cable)
			dcp->external_generation++;
		dcp->external_cable = true;
		WRITE_ONCE(dcp->external_physically_absent, false);
		dcp_external_publish_generation(dcp);
		scoped_guard(mutex, &dcp->hpd_mutex) {
			WRITE_ONCE(dcp->typec_cable_connected, true);
			if (new_cable) {
				dcp->typec_generation++;
				dcp_firmware_metadata_connection(dcp, dcp->typec_generation, true);
			}
			dcp->placeholder_retried = false;
		}
		return dcp_external_link_locked(dcp);
	}

	if (dcp_is_typec_output(dcp)) {
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		guard(mutex)(&dcp->hpd_mutex);
		dcp->typec_generation++;
		WRITE_ONCE(dcp->typec_cable_connected, true);
		dcp_firmware_metadata_connection(dcp, dcp->typec_generation, true);
		dcp->typec_reconnect_tries = 0;
		dcp->placeholder_retried = false;
		cancel_delayed_work(&dcp->typec_reconnect_wq);
	}

	ret = dcp_dptx_connect(dcp, port);
	if (ret && dcp_is_typec_output(dcp))
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq,
				 DPTX_RECONNECT_DELAY);

	return ret;
}

static int dcp_dptx_disconnect_oob_h17p(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp_is_external_h17p(dcp))
		return dcp_external_disconnect(dcp, true);

	if (dcp_is_typec_output(dcp)) {
		scoped_guard(mutex, &dcp->hpd_mutex) {
			WRITE_ONCE(dcp->typec_cable_connected, false);
			dcp->typec_generation++;
			dcp_firmware_metadata_connection(dcp, dcp->typec_generation, false);
		}
		WRITE_ONCE(dcp->typec_crtc_off, false);
		reinit_completion(&dcp->typec_iomfb_hpd_ready);
		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
	}

	disconnected_hpd_event_h17p(dcp->connector);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dptx_service_available(&dcp->dptxport[port]))
		dptxport_set_hpd(dcp->dptxport[port].service, false);

	return dcp_dptx_disconnect(dcp, port);
}

static int dcp_start_h17p(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (!dcp->rtk)
		return -ENODEV;

	init_completion(&dcp->start_done);

	/*
	 * The coprocessor start can fail (or be recovered from) without tearing
	 * the DRM side down, so this can be reached with no RTKit instance at
	 * all.  Starting an endpoint then dereferences NULL inside
	 * apple_rtkit_start_ep() and takes the machine with it.
	 */
	if (!dcp->rtk || IS_ERR(dcp->rtk)) {
		dev_err(dcp->dev,
			"cannot start endpoints: RTKit is not initialised\n");
		return -ENODEV;
	}

	/* Make the H17P allocator available before external service discovery. */
	if (dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		ret = apple_rtkit_start_ep(dcp->rtk, REMOTE_ALLOC_ENDPOINT);
		if (ret)
			return dev_err_probe(dcp->dev, ret, "Failed to start remote allocator\n");
	}

	/* start RTKit endpoints */
	ret = systemep_init(dcp);
	if (ret)
		dev_warn(dcp->dev, "Failed to start system endpoint: %d\n", ret);

	if (unstable_edid && !dcp_has_panel(dcp)) {
		ret = dpavservep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start DPAVSERV endpoint: %d",
				 ret);
	}

	if (dcp->phy && dcp->fw_compat >= DCP_FIRMWARE_V_13_5) {
		ret = ibootep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start IBOOT endpoint: %d\n",
				 ret);

		if (dcp->hw.iomfb_method_profile ==
		    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
			ret = dptx_controller_init(dcp);
			if (ret)
				return dev_err_probe(dcp->dev, ret,
						     "Failed to start DPTX controller\n");
		}

		ret = dptxep_init(dcp);
		if (ret) {
			dev_warn(dcp->dev, "Failed to start DPTX endpoint: %d\n",
				 ret);
#ifdef DCP_DPTX_DISCONNECT_ON_INIT
		/*
		 * This disconnect / connect cycle on init is only necessary
		 * when using dcp0 on j473, j474s and presumedly j475c.
		 * Since dcp0 is not used at the moment let's avoid this
		 * since it is possibly the cause for startup issues.
		 */
		} else if (dptx_service_available(&dcp->dptxport[0])) {
			bool connected;
			/* force disconnect on start - necessary if the display
			 * is already up from m1n1
			 */
			dptxport_set_hpd(dcp->dptxport[0].service, false);
			dptxport_release_display(dcp->dptxport[0].service);
			usleep_range(10 * USEC_PER_MSEC, 25 * USEC_PER_MSEC);

			connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
			dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

			// necessary on j473/j474 but not on j314c
			if (connected)
				dcp_dptx_connect(dcp, 0);
#endif
		}
	} else if (dcp->phy) {
		dev_warn(dcp->dev, "OS firmware incompatible with dptxport EP\n");
	}
	ret = iomfb_start_rtkit(dcp);
	if (ret)
		dev_err(dcp->dev, "Failed to start IOMFB endpoint: %d\n", ret);

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	if (hdmi_audio) {
		ret = avep_init(dcp);
		if (ret)
			dev_warn(dcp->dev, "Failed to start AV endpoint: %d", ret);
		ret = 0;
	}
#endif

	return ret;
}

#define DCP_BL_FALLBACK_DELAY msecs_to_jiffies(2000)

static bool dcp_uses_soft_dpms(struct apple_dcp *dcp)
{
	return (dcp_backlight_active(dcp) && dcp_has_panel(dcp)) ||
	       (dcp->fw_compat == DCP_FIRMWARE_H17P &&
		dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
		dcp->connector_type == DRM_MODE_CONNECTOR_eDP);
}

static void dcp_poweron_h17p(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	bool wait_for_typec_hpd = false;
	unsigned long remaining;
	int ret;

	if (dcp_is_external_h17p(dcp)) {
		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		cancel_work_sync(&dcp->external_stop_work);
		guard(mutex)(&dcp->external_lock);
		ret = dcp_external_start_locked(dcp);
		if (!ret)
			dcp->external_crtc_generation = dcp->external_generation;
		else
			dev_err(dcp->dev, "external display activation failed: %d\n", ret);
		return;
	}

	if (dcp_uses_soft_dpms(dcp)) {
		if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
		    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G &&
		    !READ_ONCE(dcp->pipe_enabled_h17p)) {
			iomfb_poweron_h17p(dcp);
			if (!READ_ONCE(dcp->pipe_enabled_h17p)) {
				dev_err(dcp->dev, "initial display pipe enable failed\n");
				WRITE_ONCE(dcp->crashed, true);
				return;
			}
			/* The powerlog takeover report normally precedes this. */
			if (dcp_has_panel(dcp))
				queue_delayed_work(system_freezable_wq,
						   &dcp->bl_fallback_wq,
						   DCP_BL_FALLBACK_DELAY);
		}
		ret = dcp_backlight_dpms(dcp, true);
		if (ret)
			dev_warn(dcp->dev, "backlight restore unavailable: %d\n", ret);
		return;
	}

	if (dcp_is_typec_output(dcp)) {
		wait_for_typec_hpd = READ_ONCE(dcp->typec_crtc_off) &&
				    READ_ONCE(dcp->typec_cable_connected);
		WRITE_ONCE(dcp->typec_crtc_off, false);

		/*
		 * A Type-C CRTC disable releases its DPTX session. Re-establish it
		 * synchronously before IOMFB is powered back on.
		 */
		if (READ_ONCE(dcp->typec_cable_connected)) {
			cancel_delayed_work(&dcp->typec_reconnect_wq);
			dcp->typec_reconnect_tries = 0;
			ret = dcp_dptx_connect(dcp, 0);
			if (ret)
				mod_delayed_work(system_freezable_wq,
						 &dcp->typec_reconnect_wq,
						 DPTX_RECONNECT_DELAY);
			else if (wait_for_typec_hpd) {
				remaining = wait_for_completion_timeout(
					&dcp->typec_iomfb_hpd_ready,
					msecs_to_jiffies(3000));
				if (!remaining)
					dev_warn(dcp->dev,
						 "Type-C IOMFB hotplug not ready on wake\n");
			}
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		dev_info(dcp->dev, "%s: DP2HDMI HPD connected:%d\n", __func__, connected);

		if (connected)
			dcp_dptx_connect(dcp, 0);
	}

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		iomfb_poweron_v12_3(dcp);
		break;
	case DCP_FIRMWARE_V_13_5:
		iomfb_poweron_v13_3(dcp);
		break;
	case DCP_FIRMWARE_H17P:
		iomfb_poweron_h17p(dcp);
		break;
	default:
		WARN_ONCE(true, "Unexpected firmware version: %u\n", dcp->fw_compat);
		break;
	}
	if (dcp->avep)
		av_service_connect(dcp);
}

/* Same-CRTC helper dependencies order this token refresh before later disables. */
void dcp_external_recover(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp_is_external_h17p(dcp))
		dcp_poweron(pdev);
}

static void dcp_poweroff_h17p(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

	if (dcp_is_external_h17p(dcp)) {
		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		guard(mutex)(&dcp->external_lock);
		ret = dcp_external_stop_locked(dcp, dcp->external_crtc_generation);
		if (ret)
			dev_err(dcp->dev, "external display stop failed: %d\n", ret);
		return;
	}

	cancel_delayed_work(&dcp->swap_watchdog_wq);

	/* Internal H17P DPMS is a brightness present, never a pipe stop. */
	if (dcp_uses_soft_dpms(dcp)) {
		ret = dcp_backlight_dpms(dcp, false);
		if (ret)
			dev_warn(dcp->dev, "backlight blank unavailable: %d\n", ret);
		return;
	}

	if (dcp->avep)
		av_service_disconnect(dcp);

	/*
	 * Powering a Type-C CRTC off drops DCP's synthetic HPD, and the firmware
	 * reports that as an unplug. The display is still attached: keep the
	 * connector connected (see dcpep_cb_hotplug()) and let dcp_poweron()
	 * re-establish the DPTX session. Recreating it here instead makes the
	 * display vanish and come back, and compositors light a returning
	 * display, so DPMS off never sticks. Cable removal is reported through
	 * the Type-C mux.
	 */
	if (dcp_is_typec_output(dcp)) {
		reinit_completion(&dcp->typec_iomfb_hpd_ready);
		if (READ_ONCE(dcp->typec_cable_connected))
			WRITE_ONCE(dcp->typec_crtc_off, true);
		/* dcp_poweron() reconnects the link on DPMS wake. */
		cancel_delayed_work(&dcp->typec_reconnect_wq);
	}

	_dcp_poweroff(dcp);

	if (dcp_is_typec_output(dcp)) {
		/* DCP owns a synthetic HPD for Type-C. Release it with the CRTC. */
		if (dptx_service_available(&dcp->dptxport[0]) && dcp->dptxport[0].connected) {
			ret = dptxport_set_hpd(dcp->dptxport[0].service, false);
			if (ret)
				dev_warn(dcp->dev,
					 "failed to deassert Type-C DPTX HPD: %d\n", ret);
			dcp_dptx_disconnect(dcp, 0);
		}
	} else if (dcp->hdmi_hpd) {
		bool connected = gpiod_get_value_cansleep(dcp->hdmi_hpd);
		if (!connected) {
			disconnected_hpd_event_h17p(dcp->connector);
			dcp_dptx_disconnect(dcp, 0);
		}
	}
}

static void dcp_work_register_backlight_h17p(struct work_struct *work)
{
	int ret;
	struct apple_dcp *dcp;

	dcp = container_of(work, struct apple_dcp, bl_register_wq);

	mutex_lock(&dcp->bl_register_mutex);
	if (dcp->brightness.bl_dev)
		goto out_unlock;

	/* try to register backlight device, */
	ret = dcp_backlight_register(dcp);
	if (ret == -ENODATA)
		goto out_unlock;
	if (ret) {
		dev_err(dcp->dev, "Unable to register backlight device: %d\n", ret);
		/*
		 * The H17P policy encodes the panel ceiling into every present
		 * and keeps working without a class device; only older
		 * firmware stops sending brightness here.
		 */
		if (!dcp_backlight_active(dcp))
			dcp->brightness.maximum = 0;
	}

out_unlock:
	mutex_unlock(&dcp->bl_register_mutex);
}

/*
 * H17P never publishes the panel level, and the loader's level arrives only
 * as an optional powerlog report.  Without one, register the backlight at
 * the middle of the panel range.  That value is only reported: no level is
 * presented until userspace writes one, so the panel keeps the loader's
 * level.
 */
static void dcp_work_backlight_fallback(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					     struct apple_dcp, bl_fallback_wq);
	u32 nits = dcp->brightness.maximum / 2;
	int ret;

	if (READ_ONCE(dcp->quiescing) || READ_ONCE(dcp->crashed) ||
	    dcp_backlight_active(dcp))
		return;

	ret = iomfb_configure_backlight_h17p(dcp, dcp->brightness.maximum,
					     false, 0, true, nits);
	if (!ret)
		dev_info(dcp->dev,
			 "no loader brightness reported; backlight starts at %u nits without a panel change\n",
			 nits);
	else if (ret != -EBUSY)
		dev_warn(dcp->dev, "backlight registration failed: %d\n", ret);
}

static void dcp_release_piodma_iommu_dev(struct apple_dcp *dcp)
{
	if (dcp->retain_dma) {
		WRITE_ONCE(dcp->quiescing, true);
		return;
	}
	if (dcp->piodma) {
		if (dcp->piodma_created)
			of_platform_device_destroy(&dcp->piodma->dev, NULL);
		else
			put_device(&dcp->piodma->dev);
	}
	dcp->piodma = NULL;
	dcp->iommu_dom = NULL;
	dcp->piodma_created = false;
}

void dcp_retain_framebuffer(struct platform_device *pdev,
			    struct dcp_fb_reference *entry)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	mutex_lock(&dcp->swapped_out_fbs_lock);
	list_add_tail(&entry->head, &dcp->swapped_out_fbs);
	mutex_unlock(&dcp->swapped_out_fbs_lock);
}

void dcp_arm_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id)
{
	struct dcp_fb_reference *entry;

	mutex_lock(&dcp->swapped_out_fbs_lock);
	list_for_each_entry(entry, &dcp->swapped_out_fbs, head) {
		if (entry->armed)
			continue;
		entry->swap_id = swap_id;
		entry->armed = true;
	}
	mutex_unlock(&dcp->swapped_out_fbs_lock);
}

void dcp_unarm_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id)
{
	struct dcp_fb_reference *entry;

	mutex_lock(&dcp->swapped_out_fbs_lock);
	list_for_each_entry(entry, &dcp->swapped_out_fbs, head)
		if (entry->armed && entry->swap_id == swap_id)
			entry->armed = false;
	mutex_unlock(&dcp->swapped_out_fbs_lock);
}

void dcp_release_retained_framebuffers(struct apple_dcp *dcp, u32 swap_id)
{
	struct dcp_fb_reference *entry, *tmp;
	LIST_HEAD(completed);

	mutex_lock(&dcp->swapped_out_fbs_lock);
	list_for_each_entry_safe(entry, tmp, &dcp->swapped_out_fbs, head) {
		if (!entry->armed || entry->swap_id != swap_id)
			continue;
		list_move_tail(&entry->head, &completed);
	}
	mutex_unlock(&dcp->swapped_out_fbs_lock);

	list_for_each_entry_safe(entry, tmp, &completed, head) {
		list_del(&entry->head);
		drm_framebuffer_put(entry->fb);
		kfree(entry);
	}
}

void dcp_release_all_retained_framebuffers(struct apple_dcp *dcp)
{
	struct dcp_fb_reference *entry, *tmp;
	LIST_HEAD(completed);

	mutex_lock(&dcp->swapped_out_fbs_lock);
	list_splice_init(&dcp->swapped_out_fbs, &completed);
	mutex_unlock(&dcp->swapped_out_fbs_lock);

	list_for_each_entry_safe(entry, tmp, &completed, head) {
		list_del(&entry->head);
		drm_framebuffer_put(entry->fb);
		kfree(entry);
	}
}


static int dcp_create_piodma_iommu_dev_h17p(struct apple_dcp *dcp)
{
	int ret;
	struct device_node *node __free(device_node) = of_get_child_by_name(dcp->dev->of_node, "piodma");

	if (!node)
		return dev_err_probe(dcp->dev, -ENODEV,
				     "Failed to get piodma child DT node\n");

	dcp->piodma = of_find_device_by_node(node);
	dcp->piodma_created = !dcp->piodma;
	if (dcp->piodma_created)
		dcp->piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->piodma)
		return dev_err_probe(dcp->dev, -ENODEV,
				     "Failed to create piodma pdev for %pOF\n", node);

	ret = dma_set_mask_and_coherent(&dcp->piodma->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err_destroy_pdev;

	ret = of_dma_configure(&dcp->piodma->dev, node, true);
	if (ret) {
		ret = dev_err_probe(dcp->dev, ret,
			"Failed to configure IOMMU child DMA\n");
		goto err_destroy_pdev;
	}

	dcp->iommu_dom = iommu_get_domain_for_dev(&dcp->piodma->dev);
	if (IS_ERR(dcp->iommu_dom)) {
		ret = dev_err_probe(dcp->dev, PTR_ERR(dcp->iommu_dom),
				    "Failed to get default iommu domain for "
				    "piodma device\n");
		dcp->iommu_dom = NULL;
		goto err_destroy_pdev;
	}

	return 0;
err_destroy_pdev:
	dcp_release_piodma_iommu_dev(dcp);
	return ret;
}

static int dcp_get_disp_regs_h17p(struct apple_dcp *dcp)
{
	struct platform_device *pdev = to_platform_device(dcp->dev);
	int count = 0;
	int i, ret;
	char name[16];
	const char *reg_name;
	struct resource *res;

	if (of_property_present(dcp->dev->of_node, "reg-names")) {
		ret = of_property_count_strings(dcp->dev->of_node, "reg-names");
		if (ret < 0)
			return ret;

		for (i = 0; i < ret; i++) {
			if (of_property_read_string_index(dcp->dev->of_node,
							  "reg-names", i, &reg_name))
				return -EINVAL;
			if (!strncmp(reg_name, "disp-", 5))
				count++;
		}
	} else {
		count = pdev->num_resources - 1;
	}

	if (count <= 0 || count > MAX_DISP_REGISTERS)
		return -EINVAL;

	for (i = 0; i < count; ++i) {
		if (of_property_present(dcp->dev->of_node, "reg-names")) {
			snprintf(name, sizeof(name), "disp-%d", i);
			res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							   name);
		} else {
			res = platform_get_resource(pdev, IORESOURCE_MEM, 1 + i);
		}
		if (!res)
			return -EINVAL;
		dcp->disp_registers[i] = res;
	}

	if (dcp->hw.firmware_clock) {
		unsigned int windows = dcp->hw.firmware_display_windows;
		bool external = dcp->hw.iomfb_method_profile ==
			DCP_IOMFB_METHODS_H17P_EXTERNAL;

		if (!windows || windows >= MAX_DISP_REGISTERS)
			return -EINVAL;
		if (external) {
			/* The external clock aperture is already described by disp-4. */
			if (count != windows + 1)
				return -EINVAL;
			res = dcp->disp_registers[windows];
			if (!apple_dcp_external_clock_window(res->start,
							     resource_size(res),
							     dcp->hw.firmware_clock->start,
							     resource_size(dcp->hw.firmware_clock)))
				return -EINVAL;
			count = windows;
		}
		if (count != windows || resource_size(dcp->hw.firmware_clock) < 4 ||
		    dcp->hw.firmware_scratch > resource_size(dcp->hw.firmware_clock) - 4 ||
		    dcp->hw.firmware_request > resource_size(dcp->hw.firmware_clock) - 4)
			return -EINVAL;

		dcp->disp_bw_scratch_res = *dcp->hw.firmware_clock;
		dcp->disp_bw_scratch_index = count;
		dcp->disp_bw_scratch_offset = dcp->hw.firmware_scratch;
		dcp->disp_bw_doorbell_res = dcp->disp_bw_scratch_res;
		dcp->disp_bw_doorbell_res.start += dcp->hw.firmware_request;
		dcp->disp_bw_doorbell_res.end = dcp->disp_bw_doorbell_res.start + 3;
		dcp->disp_registers[count] = &dcp->disp_bw_scratch_res;
		dcp->nr_disp_registers = count + 1;
		return 0;
	}

	/* load pmgr bandwidth scratch resource and offset */
	ret = dcp_get_bw_scratch_reg(dcp, count);
	if (ret < 0)
		return ret;
	count += 1;

	/* load pmgr bandwidth doorbell resource if present (only on t8103) */
	if (of_property_present(dcp->dev->of_node, "apple,bw-doorbell")) {
		ret = dcp_get_bw_doorbell_reg(dcp, count);
		if (ret < 0)
			return ret;
		count += 1;
	}

	dcp->nr_disp_registers = count;
	return 0;
}

static void dcp_disable_typec_work_h17p(struct apple_dcp *dcp, bool release_cable)
{
	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (release_cable)
			WRITE_ONCE(dcp->typec_cable_connected, false);
		dcp->typec_generation++;
		dcp_firmware_metadata_connection(dcp, dcp->typec_generation, false);
	}
	if (release_cable && dcp_is_external_h17p(dcp))
		dpavservep_detach(dcp);
	/* Block new enqueues as well as draining users of the AFK endpoints. */
	disable_delayed_work_sync(&dcp->typec_reconnect_wq);
	disable_delayed_work_sync(&dcp->placeholder_edid_wq);
	cancel_work_sync(&dcp->external_stop_work);
}

static void dcp_enable_typec_work_h17p(struct apple_dcp *dcp)
{
	enable_delayed_work(&dcp->typec_reconnect_wq);
	enable_delayed_work(&dcp->placeholder_edid_wq);
	/* A cable can be routed before the DRM component binds. */
	if (READ_ONCE(dcp->typec_cable_connected))
		mod_delayed_work(system_freezable_wq, &dcp->typec_reconnect_wq, 0);
}

static bool dcp_external_is_idle(struct apple_dcp *dcp)
{
	u32 control = readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	u32 status = readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_STATUS);

	/* Same externally inherited idle state accepted by the public loader. */
	return (control & APPLE_DCP_COPROC_CPU_CONTROL_RUN) &&
	       (status & APPLE_DCP_COPROC_CPU_STATUS_STOPPED) &&
	       (status & APPLE_DCP_COPROC_CPU_STATUS_IDLE);
}

static int dcp_comp_bind_h17p(struct device *dev, struct device *main, void *data)
{
	struct device_node *panel_np;
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	u32 cpu_ctrl;
	int ret;

	if (READ_ONCE(dcp->quiescing))
		return dev_err_probe(dev, -EBUSY,
				     "Live DCP session cannot be rebound; reboot required\n");

	if (dcp->external_prepared)
		return 0;

	/* A timed-out prior session may still scan these mappings. */
	mutex_lock(&dcp->swapped_out_fbs_lock);
	ret = list_empty(&dcp->swapped_out_fbs) ? 0 : -EBUSY;
	mutex_unlock(&dcp->swapped_out_fbs_lock);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Previous scanout has not been stopped\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	dcp->coproc_reg = devm_platform_ioremap_resource_byname(to_platform_device(dev), "coproc");
	if (IS_ERR(dcp->coproc_reg))
		return PTR_ERR(dcp->coproc_reg);

	/*
	 * Display coprocessors running H17-generation firmware are left running
	 * by the bootloader with their firmware mapped through a locked DART.
	 * Linux attaches to that session with a standard RTKit INIT/HELLO
	 * exchange and must not stop or restart the ASC.
	 */
	if (dcp->hw.adopt_live_session) {
		cpu_ctrl = readl_relaxed(dcp->coproc_reg +
					 APPLE_DCP_COPROC_CPU_CONTROL);
		if (!(cpu_ctrl & APPLE_DCP_COPROC_CPU_CONTROL_RUN))
			return dev_err_probe(dev, -EOPNOTSUPP,
					     "DCP was not left running by the bootloader\n");
	}

	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	    !dcp_external_is_idle(dcp))
		return dev_err_probe(dev, -EBUSY,
				     "External DCP is not in the inherited idle state\n");

	if (dcp->index || dcp->dptx_phy || dcp->dptx_die)
		dev_info(dev, "DCP index:%u dptx target phy: %u dptx die: %u\n",
			 dcp->index, dcp->dptx_phy, dcp->dptx_die);

	if (!show_notch)
		ret = of_property_read_u32(dev->of_node, "apple,notch-height",
					   &dcp->notch_height);

	if (dcp->notch_height > MAX_NOTCH_HEIGHT)
		dcp->notch_height = MAX_NOTCH_HEIGHT;
	if (dcp->notch_height > 0)
		dev_info(dev, "Detected display with notch of %u pixel\n", dcp->notch_height);

	/* initialize brightness scale to a sensible default to avoid divide by 0*/
	dcp->brightness.scale = 65536;
	panel_np = of_get_compatible_child(dev->of_node, "apple,panel-mini-led");
	if (panel_np)
		dcp->panel.has_mini_led = true;
	else
		panel_np = of_get_compatible_child(dev->of_node, "apple,panel");

	if (panel_np) {
		const char height_prop[2][16] = { "adj-height-mm", "height-mm" };

		if (of_device_is_available(panel_np)) {
			ret = of_property_read_u32(panel_np, "apple,max-brightness",
						   &dcp->brightness.maximum);
			if (ret)
				dev_err(dev, "Missing property 'apple,max-brightness'\n");
		}

		of_property_read_u32(panel_np, "width-mm", &dcp->panel.width_mm);
		/* use adjusted height as long as the notch is hidden */
		of_property_read_u32(panel_np, height_prop[!dcp->notch_height],
				     &dcp->panel.height_mm);

		of_node_put(panel_np);
		dcp->fixed_connector_type = DRM_MODE_CONNECTOR_eDP;
		dcp->connector_type = DRM_MODE_CONNECTOR_eDP;
		INIT_WORK(&dcp->bl_register_wq, dcp_work_register_backlight_h17p);
		mutex_init(&dcp->bl_register_mutex);
		INIT_WORK(&dcp->bl_update_wq, dcp_work_update_backlight);
	}

	ret = dcp_create_piodma_iommu_dev_h17p(dcp);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to create PIODMA iommu child device\n");
	if (!dcp->iommu_dom) {
		ret = dev_err_probe(dev, -EPROBE_DEFER,
				    "PIODMA iommu domain is unavailable\n");
		goto err_piodma;
	}

	ret = dcp_get_disp_regs_h17p(dcp);
	if (ret) {
		dev_err(dev, "failed to find display registers\n");
		goto err_piodma;
	}

	dcp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(dcp->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(dcp->clk),
				    "Unable to find clock\n");
		goto err_piodma;
	}
	dcp->clk_194 = devm_clk_get_optional(dev, "clock-194");
	if (IS_ERR(dcp->clk_194)) {
		ret = dev_err_probe(dev, PTR_ERR(dcp->clk_194),
				    "Unable to find clock 0x194\n");
		goto err_piodma;
	}

	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL &&
	    (!dcp->clk_194 || !clk_get_rate(dcp->clk) ||
	     !clk_get_rate(dcp->clk_194))) {
		ret = dev_err_probe(dev, -EINVAL,
				    "External display clocks are unavailable\n");
		goto err_piodma;
	}

	bitmap_zero(dcp->memdesc_map, DCP_MAX_MAPPINGS);
	// TDOD: mem_desc IDs start at 1, for simplicity just skip '0' entry
	set_bit(0, dcp->memdesc_map);

	INIT_WORK(&dcp->vblank_wq, dcp_delayed_vblank);
	INIT_DELAYED_WORK(&dcp->swap_watchdog_wq, dcp_swap_watchdog);

	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		/* Pin before the first write or mailbox wake can expose DMA. */
		ret = dcp_pin_live_session(dcp);
		if (ret)
			goto err_piodma;
		if (!dcp_external_is_idle(dcp)) {
			ret = dev_err_probe(dev, -EBUSY,
					    "External DCP changed state before startup\n");
			goto err_piodma;
		}
	}

	if (!dcp->hw.adopt_live_session) {
		cpu_ctrl =
			readl_relaxed(dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
		writel_relaxed(cpu_ctrl | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
			       dcp->coproc_reg + APPLE_DCP_COPROC_CPU_CONTROL);
	}

	/* Registering the mailbox can already expose RTKit buffer requests. */
	ret = dcp_pin_live_session(dcp);
	if (ret)
		goto err_piodma;
	/* External H17P supplies shared-buffer addresses during RTKit startup. */
	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		dcp->rtk = apple_rtkit_init(dev, dcp, "mbox", 0,
					    &external_h17p_rtkit_ops);
	else
		dcp->rtk = apple_rtkit_init(dev, dcp, "mbox", 0, &rtkit_ops);
	if (IS_ERR(dcp->rtk)) {
		ret = dev_err_probe(dev, PTR_ERR(dcp->rtk),
				    "Failed to initialize RTKit\n");
		goto err_piodma;
	}
	ret = devm_add_action_or_reset(dev, dcp_release_rtkit, dcp);
	if (ret)
		goto err_piodma;

	if (dcp->hw.adopt_live_session)
		dev_info(dev, "negotiating RTKit with the running DCP\n");

	/* Announce host readiness before awaiting external IOP readiness. */
	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL)
		ret = apple_rtkit_wake_early_ap(dcp->rtk);
	else
		ret = apple_rtkit_wake(dcp->rtk);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to boot RTKit: %d\n", ret);
		goto err_piodma;
	}
	dcp_enable_typec_work_h17p(dcp);
	return 0;

err_piodma:
	dcp_release_piodma_iommu_dev(dcp);
	return ret;
}

/*
 * We need to shutdown DCP before tearing down the display subsystem. Otherwise
 * the DCP will crash and briefly flash a green screen of death.
 */
static void dcp_comp_unbind_h17p(struct device *dev, struct device *main, void *data)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (!dcp)
		return;

	if (dcp->retain_dma) {
		/* No firmware stop is acknowledged: detach host users, never DMA. */
		scoped_guard(mutex, &dcp->iomfb.lock)
			WRITE_ONCE(dcp->quiescing, true);
		cancel_work_sync(&dcp->external_stop_work);
		if (dcp->crtc && !dcp->drm_retained) {
			drm_dev_get(dcp->crtc->base.dev);
			dcp->drm_retained = true;
		}
		iomfb_queue_stop(dcp);
		afk_quiesce(dcp->avep);
		afk_quiesce(dcp->dptxep);
		afk_quiesce(dcp->dptx_controller_ep);
		afk_quiesce(dcp->ibootep);
		afk_quiesce(dcp->systemep);
		afk_quiesce(dcp->dcpavservep);
		cancel_delayed_work_sync(&dcp->bl_fallback_wq);
		dcp_backlight_stop(dcp);
		if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP) {
			cancel_work_sync(&dcp->bl_register_wq);
			cancel_work_sync(&dcp->bl_update_wq);
		}
		cancel_delayed_work_sync(&dcp->typec_reconnect_wq);
		cancel_work_sync(&dcp->vblank_wq);
		if (dcp->connector)
			cancel_work_sync(&dcp->connector->hotplug_wq);
		if (dcp->typec_connector && dcp->typec_connector != dcp->connector)
			cancel_work_sync(&dcp->typec_connector->hotplug_wq);
		dev_warn(dev, "retaining live DCP DMA resources after unbind; reboot required\n");
		return;
	}

	if (dcp->hdmi_hpd_irq)
		disable_irq(dcp->hdmi_hpd_irq);

	dcp_disable_typec_work_h17p(dcp, true);
	typec_mux_put(dcp->typec_mux);

	if (dcp->avep) {
		av_service_disconnect(dcp);
		afk_shutdown(dcp->avep);
		dcp->avep = NULL;
	}

	if (dcp->dptxep) {
		afk_shutdown(dcp->dptxep);
		dcp->dptxep = NULL;
	}

	if (!IS_ERR_OR_NULL(dcp->dptx_controller_ep)) {
		afk_shutdown(dcp->dptx_controller_ep);
		dcp->dptx_controller_ep = NULL;
	}

	if (dcp->ibootep) {
		afk_shutdown(dcp->ibootep);
		dcp->ibootep = NULL;
	}

	if (dcp->systemep) {
		afk_shutdown(dcp->systemep);
		dcp->systemep = NULL;
	}

	if (dcp->dcpavservep) {
		afk_shutdown(dcp->dcpavservep);
		dpavservep_detach(dcp);
		dcp->dcpavservep = NULL;
	}

	if (dcp->shmem)
		iomfb_shutdown(dcp);

	iomfb_queue_stop(dcp);
	dcp_release_piodma_iommu_dev(dcp);

	cancel_delayed_work_sync(&dcp->bl_fallback_wq);
	dcp_backlight_stop(dcp);
	if (dcp->connector_type == DRM_MODE_CONNECTOR_eDP) {
		cancel_work_sync(&dcp->bl_register_wq);
		cancel_work_sync(&dcp->bl_update_wq);
	}
	cancel_delayed_work_sync(&dcp->swap_watchdog_wq);
	cancel_work_sync(&dcp->vblank_wq);

	devm_clk_put(dev, dcp->clk);
	dcp->clk = NULL;
	/* optional and possibly NULL, released with the bind's devres group */
	dcp->clk_194 = NULL;
}

static int dcp_platform_probe_h17p(struct platform_device *pdev)
{
	const struct apple_dcp_hw_data *hw = of_device_get_match_data(&pdev->dev);
	enum dcp_firmware_version fw_compat;
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	int ret, surf, num_surfs;
	u32 surf_en;
	u32 mux_index;

	if (dcp_session_retained(dev))
		return dev_err_probe(dev, -EBUSY, "Previous live DCP session requires a reboot\n");

	/* Reject unsupported external firmware before replacing the boot display. */
	if (of_device_is_compatible(dev->of_node, "apple,t8140-dcpext") &&
	    !dcp_h17p_external_supported(dev->of_node)) {
		dev_err(dev, "Unsupported external display hardware or firmware handoff\n");
		return -ENODEV;
	}

	fw_compat = dcp_check_firmware_version(dev);
	if (fw_compat == DCP_FIRMWARE_UNKNOWN)
		return -ENODEV;

	/* Check for "apple,bw-scratch" to avoid probing appledrm with outdated
	 * device trees. This prevents replacing simpledrm and ending up without
	 * display.
	 */
	if (!of_property_present(dev->of_node, "apple,bw-scratch") &&
	    !hw->firmware_clock)
		return dev_err_probe(dev, -ENODEV, "Incompatible devicetree! "
			"Use devicetree matching this kernel.\n");

	dcp = kzalloc_obj(*dcp);
	if (!dcp)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, dcp_release_context, dcp);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&dcp->swapped_out_fbs);
	mutex_init(&dcp->swapped_out_fbs_lock);
	ret = dcp_backlight_init(dcp, dev);
	if (ret)
		return ret;
	iomfb_queue_init(dcp);

	dcp->fw_compat = fw_compat;
	dcp->dev = dev;
	/*
	 * Type-C and Thunderbolt routes can be activated as soon as they are
	 * registered below, before the DRM device binds.
	 */
	/* The shared Type-C fabric takes the catalog lock and HDMI presence. */
	dcp_modes_init(dcp);
	ret = devm_add_action_or_reset(dev, dcp_modes_release, dcp);
	if (ret)
		return ret;
	dcp_fabric_init(dcp);
	mutex_init(&dcp->hpd_mutex);
	mutex_init(&dcp->external_lock);
	INIT_WORK(&dcp->external_stop_work, dcp_external_stop_work);
	mutex_init(&dcp->tb_lock);
	spin_lock_init(&dcp->mode_state.lock);
	spin_lock_init(&dcp->metadata.lock);
	mutex_init(&dcp->dcpavserv.lock);
	dcp->hw = *(struct apple_dcp_hw_data *)of_device_get_match_data(dev);
	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		if (!dcp->hw.firmware_clock ||
		    !apple_dcp_external_bw_valid(resource_size(dcp->hw.firmware_clock),
					 dcp->hw.firmware_scratch,
					 dcp->hw.firmware_request))
			return dev_err_probe(dev, -EINVAL,
					     "Invalid external profile bandwidth resources\n");
		iomfb_pause_presentation(dcp, true);
	}


	/* The domain use must survive driver-core DMA cleanup for a live session. */
	ret = iommu_device_use_default_domain(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, dcp_release_dma_domain, dcp);
	if (ret)
		return ret;
	dcp->fixed_connector_type = dcp_connector_type_from_dt(dev->of_node);
	dcp->connector_type = dcp->fixed_connector_type;
	of_property_read_u32(dev->of_node, "apple,dcp-index", &dcp->index);
	of_property_read_u32(dev->of_node, "apple,dptx-phy", &dcp->dptx_phy);
	of_property_read_u32(dev->of_node, "apple,dptx-die", &dcp->dptx_die);
	dcp->fixed_dptx_phy = dcp->dptx_phy;
	init_completion(&dcp->typec_iomfb_hpd_ready);
	INIT_DELAYED_WORK(&dcp->typec_reconnect_wq,
			  dcp_typec_reconnect_work_h17p);
	INIT_DELAYED_WORK(&dcp->placeholder_edid_wq,
			  dcp_placeholder_edid_work_h17p);
	INIT_DELAYED_WORK(&dcp->bl_fallback_wq, dcp_work_backlight_fallback);
	/* Balanced by enable at successful component bind. */
	disable_delayed_work(&dcp->typec_reconnect_wq);
	disable_delayed_work(&dcp->placeholder_edid_wq);

	platform_set_drvdata(pdev, dcp);

	dcp->phy = devm_phy_optional_get(dev, "dp-phy");
	if (IS_ERR(dcp->phy)) {
		dev_err(dev, "Failed to get dp-phy: %ld\n", PTR_ERR(dcp->phy));
		return PTR_ERR(dcp->phy);
	}
	dcp->fixed_phy = dcp->phy;

	bitmap_zero(dcp->iomfb_surfaces, DCP_MAX_PLANES);
	if (!of_property_present(dev->of_node, "apple,iomfb-surfaces"))
		num_surfs = 0;
	else
		num_surfs = of_property_count_elems_of_size(dev->of_node,
						    "apple,iomfb-surfaces",
						    sizeof(u32));

	if (num_surfs == 0 || num_surfs == -ENODATA) {
		set_bit(0, dcp->iomfb_surfaces);
		set_bit(1, dcp->iomfb_surfaces);
	} else if (num_surfs < 0) {
		return num_surfs;
	} else if (num_surfs > DCP_MAX_PLANES) {
		dev_err(dev, "Number of iomfb-surfaces (%d) exceeds DCP_MAX_PLANES\n",
			num_surfs);
		return -EINVAL;
	}

	surf = 0;
	of_property_for_each_u32(dev->of_node, "apple,iomfb-surfaces", surf_en) {
		if (surf_en)
			set_bit(surf, dcp->iomfb_surfaces);
		surf++;
	}
	/* The J700 loader names surface zero rather than enabling it. */
	if (of_machine_is_compatible("apple,j700") &&
	    of_device_is_compatible(dev->of_node, "apple,t8140-dcp") &&
	    num_surfs == 1 &&
	    !of_property_read_u32(dev->of_node, "apple,iomfb-surfaces", &surf_en) &&
	    surf_en == 0)
		set_bit(0, dcp->iomfb_surfaces);

	if (dcp->phy) {
		int ret;
		/*
		 * Request DP2HDMI related GPIOs as optional for DP-altmode
		 * compatibility. J180D misses a dp2hdmi-pwren GPIO in the
		 * template ADT. TODO: check device ADT
		 */
		dcp->hdmi_hpd = devm_gpiod_get_optional(dev, "hdmi-hpd", GPIOD_IN);
		if (IS_ERR(dcp->hdmi_hpd))
			return PTR_ERR(dcp->hdmi_hpd);
		if (dcp->hdmi_hpd) {
			int irq = gpiod_to_irq(dcp->hdmi_hpd);
			if (irq < 0) {
				dev_err(dev, "failed to translate HDMI hpd GPIO to IRQ\n");
				return irq;
			}
			dcp->hdmi_hpd_irq = irq;

			ret = devm_request_threaded_irq(dev, dcp->hdmi_hpd_irq,
						NULL, dcp_dp2hdmi_hpd,
						IRQF_ONESHOT | IRQF_NO_AUTOEN |
						IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						"dp2hdmi-hpd-irq", dcp);
			if (ret < 0) {
				dev_err(dev, "failed to request HDMI hpd irq %d: %d\n",
					irq, ret);
				return ret;
			}
		}

		/*
		 * Power DP2HDMI on as it is required for the HPD irq.
		 * TODO: check if one is sufficient for the hpd to save power
		 *       on battery powered Macbooks.
		 */
		dcp->hdmi_pwren = devm_gpiod_get_optional(dev, "hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->hdmi_pwren))
			return PTR_ERR(dcp->hdmi_pwren);

		dcp->dp2hdmi_pwren = devm_gpiod_get_optional(dev, "dp2hdmi-pwren", GPIOD_OUT_HIGH);
		if (IS_ERR(dcp->dp2hdmi_pwren))
			return PTR_ERR(dcp->dp2hdmi_pwren);

		/*
		 * A DCP may have both a fixed HDMI/DP route and allocatable Type-C
		 * routes. Keep the fixed route selected until the allocator borrows
		 * this otherwise-idle pipeline for a Type-C display.
		 */
		ret = dcp->fixed_phy ?
			of_property_read_u32(dev->of_node, "mux-index", &mux_index) :
			-ENODATA;
		if (!ret) {
			dcp->fixed_mux_index = mux_index;
			dcp->xbar = devm_mux_control_get(dev, "dp-xbar");
			if (IS_ERR(dcp->xbar)) {
				dev_err(dev, "Failed to get dp-xbar: %ld\n", PTR_ERR(dcp->xbar));
				return PTR_ERR(dcp->xbar);
			}
			ret = mux_control_select(dcp->xbar, mux_index);
			if (ret)
				dev_warn(dev, "mux_control_select failed: %d\n", ret);
			else
				dcp->fixed_route_selected = true;

			/*
			 * Switch atcphy to DP-only. should move to a Macbook Pro
			 * 14-/16-inch specific DP-to-HDMI drm_bridge.
			 */
			dcp->typec_mux = fwnode_typec_mux_get(dev_fwnode(dcp->dev));
			if (!IS_ERR_OR_NULL(dcp->typec_mux)) {
				struct typec_altmode alt = {
					.svid = USB_TYPEC_DP_SID,
				};
				struct typec_mux_state state = {
					.alt = &alt,
					.mode = TYPEC_DP_STATE_C,
				};
				int ret = typec_mux_set(dcp->typec_mux, &state);
				dev_info(dev, "typec_mux_set() returned: %d\n", ret);
				if (!ret)
					dcp->phy_managed_by_typec = true;
			} else {
				dev_info(dev, "fwnode_typec_mux_get() returned: %ld\n",
						IS_ERR(dcp->typec_mux) ? PTR_ERR(dcp->typec_mux) : 0);
				dcp->typec_mux = NULL;
			}
		}
	}

	ret = dcp_register_typec_routes(dcp);
	if (ret)
		return ret;

	if (dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		/* Complete optional startup before the DRM master can depend on it. */
		ret = dcp_comp_bind_h17p(dev, NULL, NULL);
		if (ret) {
			dcp_disable_typec_work_h17p(dcp, true);
			return ret;
		}
		WRITE_ONCE(dcp->external_prepared, true);
	}

	ret = component_add(&pdev->dev, &dcp_comp_ops);
	if (ret && dcp->external_prepared) {
		dcp_disable_typec_work_h17p(dcp, true);
	}
	return ret;
}

static void dcp_platform_shutdown_h17p(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	if (dcp && dcp->retain_dma && dcp->hw.iomfb_method_profile ==
	    DCP_IOMFB_METHODS_H17P_EXTERNAL) {
		dcp_comp_unbind_h17p(&pdev->dev, NULL, NULL);
		return;
	}

	if (dcp && dcp->hw.adopt_live_session &&
	    dcp->fw_compat == DCP_FIRMWARE_H17P &&
	    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
		/*
		 * This firmware keeps scanning across soft DPMS and has no
		 * known stop sequence. Component unbind releases the RTKit
		 * devres group and the piodma domain while firmware may still
		 * access them. Stop host submissions but keep the component and
		 * its DMA resources alive until the system resets.
		 */
		iomfb_queue_stop(dcp);
		cancel_delayed_work_sync(&dcp->bl_fallback_wq);
		dcp_backlight_stop(dcp);
		cancel_work_sync(&dcp->bl_register_wq);
		cancel_work_sync(&dcp->bl_update_wq);
		return;
	}

	component_del(&pdev->dev, &dcp_comp_ops);
}

/*
 * The display stays powered across system sleep, and the blanking present
 * queued by the DRM and backlight suspend handlers must reach the firmware
 * before its mailbox interrupt is suspended.
 */
static void dcp_drain_for_sleep(struct apple_dcp *dcp)
{
	int pass;

	if (!dcp->hw.adopt_live_session || dcp->fw_compat != DCP_FIRMWARE_H17P ||
	    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G)
		return;

	/*
	 * A completed present can schedule one more brightness present, and a
	 * rejected one a delayed retry; run a pending retry now instead of
	 * letting it fire after the mailbox has suspended.  Four passes cover
	 * the first attempt and the policy's three retries.  A fade still
	 * running ends at its target first.
	 */
	if (dcp_has_panel(dcp))
		dcp_backlight_finish_fade(dcp);
	for (pass = 0; pass < 4; pass++) {
		flush_delayed_work(&dcp->iomfb.backlight_retry);
		if (dcp_has_panel(dcp))
			flush_work(&dcp->bl_update_wq);
		if (!iomfb_queue_drain(dcp, msecs_to_jiffies(500))) {
			dev_warn(dcp->dev, "display updates still pending at suspend\n");
			return;
		}
		/*
		 * A crashed or stopped DCP takes no more presents, so a level
		 * still pending there can never be sent and is not waited for.
		 * Checking after the drain, which returns at once for such a
		 * DCP, also covers a crash during the pass.
		 */
		if (READ_ONCE(dcp->crashed) || READ_ONCE(dcp->iomfb.stopped))
			return;
		if (!dcp_backlight_active(dcp) || !dcp_backlight_pending(dcp))
			return;
	}
	dev_warn(dcp->dev, "brightness change still pending at suspend\n");
}

/*
 * DPMS off leaves the internal H17P pipe scanning out (see dcp_poweroff()),
 * and while it scans out the firmware publishes a property to the AP on every
 * frame.  The mailbox interrupt is not suspended, so each of those messages
 * wakes a CPU out of suspend-to-idle about 60 times a second.  Hold the
 * messages in the mailbox for system sleep and process them at resume.
 * apple-drm is a consumer of this device, so its suspend handler has blanked
 * the panel before this runs and its resume handler restores the CRTC after.
 */
static void dcp_hold_for_sleep(struct apple_dcp *dcp)
{
	if (!dcp_uses_soft_dpms(dcp) || dcp->fw_compat != DCP_FIRMWARE_H17P ||
	    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G ||
	    READ_ONCE(dcp->crashed))
		return;

	iomfb_pause_presentation(dcp, true);
	if (iomfb_queue_settle(dcp, msecs_to_jiffies(500))) {
		iomfb_pause_presentation(dcp, false);
		return;
	}
	apple_rtkit_hold_rx(dcp->rtk);
	dcp->rx_held = true;
}

static void dcp_release_after_sleep(struct apple_dcp *dcp)
{
	if (!dcp->rx_held)
		return;
	dcp->rx_held = false;
	apple_rtkit_resume_rx(dcp->rtk);
	iomfb_pause_presentation(dcp, false);
}

static int dcp_platform_suspend_h17p(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp_is_external_h17p(dcp)) {
		int ret;

		dcp_disable_typec_work_h17p(dcp, false);
		ret = dcp_external_disconnect(dcp, false);
		if (ret) {
			dcp_enable_typec_work_h17p(dcp);
			return ret;
		}
		device_set_wakeup_path(dev);
		return 0;
	}

	dcp_drain_for_sleep(dcp);
	/*
	 * The Type-C route reports cable removal through
	 * dcp_dptx_disconnect_oob(). A DP tunnel kept through the sleep stays
	 * connected, and resume powers the CRTC back up through dcp_poweron()
	 * as after DPMS off.
	 */
	dcp_disable_typec_work_h17p(dcp, false);
	cancel_delayed_work_sync(&dcp->swap_watchdog_wq);

	if (dcp->avep)
		av_service_disconnect(dcp);

	if (dcp->hdmi_hpd_irq) {
		disable_irq(dcp->hdmi_hpd_irq);
		if (!dcp->active_typec_route) {
			disconnected_hpd_event_h17p(dcp->connector);
			dcp_dptx_disconnect_h17p(dcp, 0);
		}
	}
	/*
	 * Set the device as a wakeup device, which forces its power
	 * domains to stay on. We need this as we do not support full
	 * shutdown properly yet.
	 */
	device_set_wakeup_path(dev);
	/* Last: the steps above still talk to the coprocessor. */
	dcp_hold_for_sleep(dcp);

	return 0;
}

static int dcp_platform_resume_h17p(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	dcp_release_after_sleep(dcp);
	dcp_enable_typec_work_h17p(dcp);
	if (dcp->hdmi_hpd_irq)
		enable_irq(dcp->hdmi_hpd_irq);

	if (dcp->avep)
		av_service_connect(dcp);

	return 0;
}

static const struct dev_pm_ops dcp_platform_pm_ops = {
	.prepare = pm_sleep_ptr(dcp_platform_prepare),
	.complete = pm_sleep_ptr(dcp_platform_complete),
	SYSTEM_SLEEP_PM_OPS(dcp_platform_suspend, dcp_platform_resume)
};


static const struct apple_dcp_hw_data apple_dcp_hw_t6020 = {
	.num_dptx_ports = 1,
	.t6020_tunnel_flow = true,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t6020_dcpext = {
	.num_dptx_ports = 2,
	.t6020_tunnel_flow = true,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t8112 = {
	.num_dptx_ports = 2,
};

static const struct apple_dcp_hw_data apple_dcp_hw_dcp = {
	.num_dptx_ports = 0,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t6030 = {
	.num_dptx_ports = 1,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t8122 = {
	.num_dptx_ports = 2,
};

/* The internal panel only, through the 14.x IOMFB, as on T6030. */
static const struct apple_dcp_hw_data apple_dcp_hw_t6031 = {
	.num_dptx_ports = 1,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t6030_dcpext = {
	.num_dptx_ports = 1,
};

static const struct apple_dcp_hw_data apple_dcp_hw_t8122_dcpext = {
	.num_dptx_ports = 2,
};

static const struct apple_dcp_hw_data apple_dcp_hw_dcpext = {
	.num_dptx_ports = 2,
};

/* The internal T8140 endpoint uses the H17P method profile. */
static const struct resource t8140_firmware_clock =
	DEFINE_RES_MEM(0x302800000, 0xbc000);

static const struct apple_dcp_hw_data apple_dcp_hw_t8140 = {
	.num_dptx_ports = 0,
	.adopt_live_session = true,
	.firmware_compat = DCP_FIRMWARE_H17P,
	.firmware_clock = &t8140_firmware_clock,
	.firmware_display_windows = 5,
	.firmware_scratch = 0x20000,
	.firmware_request = 0x68000,
};

/* Separate from the established internal-panel H17P profile. */
static const struct apple_dcp_hw_data apple_dcp_hw_t8140_external = {
	.num_dptx_ports = 1,
	.iomfb_method_profile = DCP_IOMFB_METHODS_H17P_EXTERNAL,
	.adopt_live_session = false,
	.firmware_compat = DCP_FIRMWARE_H17P,
	.firmware_clock = &t8140_firmware_clock,
	.firmware_display_windows = 4,
	/* PMGR BWR agent 2 and boost agent 3 for DISPEXT0. */
	.firmware_scratch = 0x28000,
	.firmware_request = 0x70000,
};

/*
 * M5 (T8142) runs H17-generation DCP firmware with the H17G method numbering
 * and is left running by the bootloader.
 */
static const struct apple_dcp_hw_data apple_dcp_hw_t8142 = {
	.num_dptx_ports = 0,
	.iomfb_method_profile = DCP_IOMFB_METHODS_H17G,
	.adopt_live_session = true,
	.firmware_compat = DCP_FIRMWARE_H17P,
};

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,t6020-dcp", .data = &apple_dcp_hw_t6020,  },
	{ .compatible = "apple,t6020-dcpext", .data = &apple_dcp_hw_t6020_dcpext, },
	{ .compatible = "apple,t8112-dcp", .data = &apple_dcp_hw_t8112,  },
	{ .compatible = "apple,t6030-dcp", .data = &apple_dcp_hw_t6030, },
	{ .compatible = "apple,t6030-dcpext", .data = &apple_dcp_hw_t6030_dcpext, },
	{ .compatible = "apple,t8122-dcp", .data = &apple_dcp_hw_t8122, },
	{ .compatible = "apple,t6031-dcp", .data = &apple_dcp_hw_t6031, },
	{ .compatible = "apple,t8122-dcpext", .data = &apple_dcp_hw_t8122_dcpext, },
	{ .compatible = "apple,t8140-dcp", .data = &apple_dcp_hw_t8140,  },
	{ .compatible = "apple,t8140-dcpext", .data = &apple_dcp_hw_t8140_external, },
	{ .compatible = "apple,t8142-dcp", .data = &apple_dcp_hw_t8142,  },
	{ .compatible = "apple,dcp",       .data = &apple_dcp_hw_dcp,    },
	{ .compatible = "apple,dcpext",    .data = &apple_dcp_hw_dcpext, },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver apple_platform_driver = {
	.driver_managed_dma = true,
	.probe		= dcp_platform_probe,
	.remove		= dcp_platform_remove,
	.shutdown	= dcp_platform_shutdown,
	.driver	= {
		.name = "apple-dcp",
		.of_match_table	= of_match,
		.pm = pm_sleep_ptr(&dcp_platform_pm_ops),
	},
};

int __init dcp_register(void)
{
	return platform_driver_register(&apple_platform_driver);
}

void __exit dcp_unregister(void)
{
	platform_driver_unregister(&apple_platform_driver);
}
