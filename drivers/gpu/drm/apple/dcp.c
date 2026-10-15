// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <linux/align.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/moduleparam.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
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
#include "iomfb.h"
#include "ibootep.h"
#include "parser.h"
#include "trace.h"

#define APPLE_DCP_COPROC_CPU_CONTROL	 0x44
#define APPLE_DCP_COPROC_CPU_CONTROL_RUN BIT(4)

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

	dcp = container_of(work, struct apple_dcp, vblank_wq);
	mdelay(5);
	dcp_drm_crtc_vblank(dcp->crtc);
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
	int ret;

	if (dcp_is_typec_output(dcp)) {
		cancel_delayed_work_sync(&dcp->placeholder_edid_wq);
		guard(mutex)(&dcp->hpd_mutex);
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

	ret = dcp_dptx_connect(dcp, port);
	if (ret && ret != -ESHUTDOWN && dcp_is_typec_output(dcp))
		dcp_queue_typec_reconnect(dcp, DPTX_RECONNECT_DELAY);

	return ret;
}

int dcp_dptx_disconnect_oob(struct platform_device *pdev, u32 port)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

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

unsigned long* dcp_get_iomfb_surfaces(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);

	return dcp->iomfb_surfaces;
}

/*
 * The lid switch, for the internal pipe that also has a Type-C route
 * (dcp->clamshell, J613; one per machine). As macOS, the driver tells the
 * panel's firmware the lid state and powers the panel off while the lid is
 * closed (iomfb_v14_7_set_lid()).
 *
 * A closed lid is a panel that is not there: its connector reads
 * disconnected until the lid opens (clamshell_panel_away), from boot on if
 * the lid is closed then, so that no compositor lights a panel nobody sees.
 * That is also what keeps the firmware alive: once told the lid is closed it
 * takes the pipe for a second external display at power-on and swallows
 * every panel frame, which then never completes.
 *
 * With the lid closed and the panel and its CRTC off the pipe is free for
 * the display on its Type-C route (dcp->clamshell_free), and ranked before
 * dcpext then: the compositor gives that display the pipe's one CRTC, the
 * lowest free one (see dcp_typec_pipeline_changed()). As macOS opens the
 * lid, that display lets go of the pipe first, then IOMFB is told; the
 * panel's connector comes back once the compositor switched the CRTC off,
 * or two seconds later, and at once if no display had the pipe. A display
 * that goes to dcpext meanwhile reads disconnected until then
 * (DCP_CLAMSHELL_HOLD_MS), so that the panel has its CRTC back first.
 */
static DEFINE_MUTEX(dcp_lid_lock);
static struct apple_dcp *dcp_lid_dcp;	/* dcp_lid_lock; the handler is registered */
static atomic_t dcp_lid_closed = ATOMIC_INIT(-1);	/* -1: no lid switch yet */

#define DCP_CLAMSHELL_PANEL_MS	2000

/*
 * As the lid opens, a display on the left-front port that leaves the
 * internal pipe for a free dcpext reads disconnected until the panel's
 * connector is connected again, for at most this long: aquamarine pairs a
 * connector that comes up with the lowest free CRTC it can use, and until
 * the panel's connector is back that is the internal pipe's, which the
 * panel needs (its only one) and the display is not routed to.
 */
#define DCP_CLAMSHELL_HOLD_MS	3000

static unsigned long dcp_clamshell_hold_until;	/* jiffies */
static struct apple_connector *dcp_clamshell_held;
static void dcp_clamshell_hold_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(dcp_clamshell_hold_work, dcp_clamshell_hold_work_fn);

/* The panel's connector reads disconnected until the lid opens. */
static void dcp_clamshell_panel_away(struct apple_dcp *dcp)
{
	struct apple_connector *panel = dcp->fixed_connector;

	cancel_delayed_work(&dcp->clamshell_panel_work);
	if (!panel || !READ_ONCE(panel->connected))
		return;
	WRITE_ONCE(dcp->clamshell_panel_away, true);
	WRITE_ONCE(panel->connected, false);
	dcp_queue_hotplug(panel);
	dev_info(dcp->dev, "lid closed: %s disconnected until the lid opens\n",
		 panel->base.name);
}

static void dcp_lid_work_fn(struct work_struct *work)
{
	int closed = atomic_read(&dcp_lid_closed);
	bool gave_back = false, free;
	struct apple_dcp *dcp;

	guard(mutex)(&dcp_lid_lock);
	dcp = dcp_lid_dcp;
	if (!dcp || closed < 0)
		return;
	if (!closed && READ_ONCE(dcp->clamshell_panel_away)) {
		WRITE_ONCE(dcp_clamshell_hold_until,
			   jiffies + msecs_to_jiffies(DCP_CLAMSHELL_HOLD_MS));
		mod_delayed_work(system_freezable_wq, &dcp_clamshell_hold_work,
				 msecs_to_jiffies(DCP_CLAMSHELL_HOLD_MS));
	}
	if (!closed && READ_ONCE(dcp->clamshell_free)) {
		WRITE_ONCE(dcp->clamshell_free, false);
		gave_back = dcp_typec_pipeline_changed(dcp);
	}
	iomfb_v14_7_set_lid(dcp, closed);
	if (closed)
		dcp_clamshell_panel_away(dcp);
	free = closed && iomfb_v14_7_clamshell_panel_off(dcp);
	if (free != READ_ONCE(dcp->clamshell_free)) {
		WRITE_ONCE(dcp->clamshell_free, free);
		gave_back |= dcp_typec_pipeline_changed(dcp);
	}
	/*
	 * A display that let go of the pipe still has the CRTC until the
	 * compositor switches it off (dcp_poweroff() brings the panel back
	 * then); if it has done so already, the panel comes back at once.
	 */
	if (!closed && READ_ONCE(dcp->clamshell_panel_away))
		mod_delayed_work(system_freezable_wq, &dcp->clamshell_panel_work,
				 gave_back && READ_ONCE(dcp->clamshell_crtc_on) ?
				 msecs_to_jiffies(DCP_CLAMSHELL_PANEL_MS) : 0);
}

/*
 * Freezable: a lid change seen while the system resumes (opening the lid
 * wakes it) is applied once the DRM device has restored its state. Applied
 * before, it hands the internal pipe back to the panel under the restore
 * commit, which then fails for every CRTC.
 */
static DECLARE_WORK(dcp_lid_work, dcp_lid_work_fn);

/* The panel was powered off or on: the pipe may be free, or not. */
void dcp_clamshell_changed(struct apple_dcp *dcp)
{
	queue_work(system_freezable_wq, &dcp_lid_work);
}

/*
 * The route took a display, with the lid closed and the panel off (from the
 * fabric, under its lock): the pipe drives that display now.
 */
void dcp_clamshell_attached(struct apple_dcp *dcp, struct device_node *connector_np)
{
	const char *label = NULL;

	iomfb_v14_7_typec_attach(dcp);
	of_property_read_string(connector_np, "label", &label);
	dev_info(dcp->dev, "lid closed, panel off: the internal pipe drives the display on %s\n",
		 label ?: "a USB-C port");
	/* already disconnected with the lid closed; makes sure */
	dcp_clamshell_panel_away(dcp);
}

static void dcp_clamshell_panel_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work), struct apple_dcp,
					     clamshell_panel_work);
	struct apple_connector *panel = dcp->fixed_connector;

	/* Only once the display let go and IOMFB heard the lid open. */
	if (!panel || !READ_ONCE(dcp->clamshell_panel_away) ||
	    READ_ONCE(dcp->active_typec_route) || !iomfb_v14_7_lid_open(dcp))
		return;
	WRITE_ONCE(dcp->clamshell_panel_away, false);
	WRITE_ONCE(panel->connected, true);
	dcp_queue_hotplug(panel);
	dev_info(dcp->dev, "lid open: %s connected again\n", panel->base.name);
	/* a held display after it, so that it is paired on its own */
	if (READ_ONCE(dcp_clamshell_held))
		mod_delayed_work(system_freezable_wq, &dcp_clamshell_hold_work,
				 msecs_to_jiffies(200));
}

/*
 * Should @connector, a Type-C connector that has just come up, still read
 * disconnected (see DCP_CLAMSHELL_HOLD_MS)? Only one the compositor has
 * not seen connected and that could be paired with the internal pipe's CRTC.
 */
bool dcp_clamshell_hold_connector(struct apple_connector *connector)
{
	struct apple_dcp *dcp = READ_ONCE(dcp_lid_dcp);

	if (!dcp || !dcp->crtc || connector == dcp->fixed_connector ||
	    connector->base.status == connector_status_connected ||
	    !(connector->candidate_crtcs & BIT(drm_crtc_index(&dcp->crtc->base))) ||
	    atomic_read(&dcp_lid_closed) != 0 ||
	    !READ_ONCE(dcp->clamshell_panel_away) ||
	    !time_before(jiffies, READ_ONCE(dcp_clamshell_hold_until)))
		return false;
	if (xchg(&dcp_clamshell_held, connector) != connector)
		dev_info(dcp->dev, "lid open: %s reads disconnected until %s is connected again\n",
			 connector->base.name,
			 dcp->fixed_connector ? dcp->fixed_connector->base.name : "the panel");
	return true;
}

/* The panel's connector is back, or the hold ran out: report the display. */
static void dcp_clamshell_hold_work_fn(struct work_struct *work)
{
	struct apple_connector *held = xchg(&dcp_clamshell_held, NULL);
	struct apple_dcp *dcp = READ_ONCE(dcp_lid_dcp);

	if (!held)
		return;
	if (dcp && READ_ONCE(dcp->clamshell_panel_away))
		dev_info(dcp->dev, "lid open: %s connected after %u ms without the panel\n",
			 held->base.name, DCP_CLAMSHELL_HOLD_MS);
	dcp_queue_hotplug(held);
}

static void dcp_lid_event(struct input_handle *handle, unsigned int type,
			  unsigned int code, int value)
{
	if (type != EV_SW || code != SW_LID)
		return;
	atomic_set(&dcp_lid_closed, !!value);
	queue_work(system_freezable_wq, &dcp_lid_work);
}

static int dcp_lid_connect(struct input_handler *handler, struct input_dev *dev,
			   const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	/* A switch made from user space (uinput) has no parent: not the lid. */
	if (!dev->dev.parent)
		return -ENODEV;
	handle = kzalloc_obj(*handle);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "appledrm-lid";
	ret = input_register_handle(handle);
	if (ret)
		goto free;
	ret = input_open_device(handle);
	if (ret)
		goto unregister;
	atomic_set(&dcp_lid_closed, test_bit(SW_LID, dev->sw));
	pr_info("appledrm: lid switch %s, lid %s\n", dev->name,
		test_bit(SW_LID, dev->sw) ? "closed" : "open");
	queue_work(system_freezable_wq, &dcp_lid_work);
	return 0;

unregister:
	input_unregister_handle(handle);
free:
	kfree(handle);
	return ret;
}

static void dcp_lid_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dcp_lid_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { [BIT_WORD(SW_LID)] = BIT_MASK(SW_LID) },
	},
	{}
};

static struct input_handler dcp_lid_handler = {
	.event = dcp_lid_event,
	.connect = dcp_lid_connect,
	.disconnect = dcp_lid_disconnect,
	.name = "appledrm-lid",
	.id_table = dcp_lid_ids,
};

static void dcp_lid_start(struct apple_dcp *dcp)
{
	int ret;

	scoped_guard(mutex, &dcp_lid_lock) {
		if (dcp_lid_dcp)
			return;
		dcp_lid_dcp = dcp;
	}
	ret = input_register_handler(&dcp_lid_handler);
	if (!ret)
		return;
	dev_err(dcp->dev, "lid switch not followed: %d\n", ret);
	scoped_guard(mutex, &dcp_lid_lock)
		dcp_lid_dcp = NULL;
}

static void dcp_lid_stop(struct apple_dcp *dcp)
{
	scoped_guard(mutex, &dcp_lid_lock) {
		if (dcp_lid_dcp != dcp)
			return;
		dcp_lid_dcp = NULL;
	}
	input_unregister_handler(&dcp_lid_handler);
	cancel_work_sync(&dcp_lid_work);
	cancel_delayed_work_sync(&dcp_clamshell_hold_work);
	WRITE_ONCE(dcp_clamshell_held, NULL);
	atomic_set(&dcp_lid_closed, -1);
}

static void dcp_enable_typec_work(struct apple_dcp *dcp);

/* After the panel started, which the DRM device does not wait for. */
static void dcp_clamshell_start(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, clamshell_start_work);

	if (iomfb_v14_7_clamshell_start(dcp))
		return;
	dcp_enable_typec_work(dcp);
	dcp_lid_start(dcp);
}

int dcp_start(struct platform_device *pdev)
{
	struct apple_dcp *dcp = platform_get_drvdata(pdev);
	int ret;

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
		if (dcp->clamshell)
			schedule_work(&dcp->clamshell_start_work);
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

	if (dcp->clamshell)
		WRITE_ONCE(dcp->clamshell_crtc_on, true);
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
		 * A Type-C CRTC disable releases its DPTX session. Re-establish it
		 * synchronously before IOMFB is powered back on.
		 */
		if (READ_ONCE(dcp->typec_cable_connected)) {
			cancel_delayed_work(&dcp->typec_reconnect_wq);
			dcp->typec_reconnect_tries = 0;
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

	cancel_delayed_work(&dcp->swap_watchdog_wq);
	if (dcp->avep)
		av_service_disconnect(dcp);
	/*
	 * The CRTC a Type-C display let go of: its panel can come back now,
	 * or, while the lid work is still taking that display's route down,
	 * once it has (see dcp_lid_work_fn()).
	 */
	if (dcp->clamshell) {
		WRITE_ONCE(dcp->clamshell_crtc_on, false);
		if (READ_ONCE(dcp->clamshell_panel_away))
			mod_delayed_work(system_freezable_wq, &dcp->clamshell_panel_work, 0);
	}

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
	const struct dcp_v14_board *v14_board;
	char compat_str[DCP_FW_VERSION_STR_LEN];
	char fw_str[DCP_FW_VERSION_STR_LEN];
	int ret;

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
	scoped_guard(mutex, &dcp->hpd_mutex) {
		dcp->typec_work_enabled = false;
		if (release_cable)
			WRITE_ONCE(dcp->typec_cable_connected, false);
		dcp->typec_generation++;
	}
	/* Block new enqueues as well as draining users of the AFK endpoints. */
	disable_delayed_work_sync(&dcp->typec_reconnect_wq);
	disable_delayed_work_sync(&dcp->placeholder_edid_wq);
	disable_delayed_work_sync(&dcp->hdmi_settle_wq);
	disable_delayed_work_sync(&dcp->hdmi_recheck_wq);
	dcp_hdmi_disable(dcp);
}

static void dcp_enable_typec_work(struct apple_dcp *dcp)
{
	enable_delayed_work(&dcp->typec_reconnect_wq);
	enable_delayed_work(&dcp->placeholder_edid_wq);
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
		}
		if (dcp->clamshell) {
			cancel_work_sync(&dcp->clamshell_start_work);
			dcp_lid_stop(dcp);
			/* A display on the route lets go of the pipe going away. */
			if (READ_ONCE(dcp->clamshell_free)) {
				WRITE_ONCE(dcp->clamshell_free, false);
				dcp_typec_pipeline_changed(dcp);
			}
			cancel_delayed_work_sync(&dcp->clamshell_panel_work);
			dcp_disable_typec_work(dcp, true);
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
 * panel is not held up either. The same for the internal panel's processor
 * (J613): its route is an extra, the panel goes on without it.
 */
static int dcp_native_routes_error(struct apple_dcp *dcp, int err)
{
	if (!dcp->external_native && !dcp_is_v14_panel(dcp))
		return err;
	if (err == -EPROBE_DEFER) {
		err = driver_deferred_probe_check_state(dcp->dev);
		if (err == -EPROBE_DEFER)
			return err;
	}
	if (dcp->external_native)
		dev_err(dcp->dev, "Type-C display routes unusable: %d; USB-C ports go on without this processor\n",
			err);
	else
		dev_err(dcp->dev, "Type-C display route unusable: %d; the panel goes on without it\n",
			err);
	dcp_typec_routes_disable(dcp);
	return 0;
}

static int dcp_platform_probe_pipe(struct platform_device *pdev)
{
	if (of_machine_is_compatible("apple,t8140"))
		return -ENODEV;

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
	spin_lock_init(&dcp->dcpavserv.lock);
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
	/* Balanced by enable at successful component bind. */
	disable_delayed_work(&dcp->typec_reconnect_wq);
	disable_delayed_work(&dcp->placeholder_edid_wq);
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
	/* J613: the panel's pipe with a Type-C route the display gate enabled. */
	dcp->clamshell = dcp_is_v14_panel(dcp) && dcp->nr_typec_routes;
	if (dcp->clamshell) {
		INIT_WORK(&dcp->clamshell_start_work, dcp_clamshell_start);
		INIT_DELAYED_WORK(&dcp->clamshell_panel_work, dcp_clamshell_panel_work);
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
 * J613: the display gate enabled the internal processor's Type-C route, and
 * the left-front port waits for it to register. If the probe fails (any
 * error, not only the route's own), disable the route, so that the port goes
 * on without it, as for a route that does not register. A deferral is not a
 * failure: the processor may still bind, and its route register, later. The
 * port is probed again with the next driver that binds.
 */
static void dcp_panel_routes_give_up(struct device *dev, int err)
{
	struct device_node *routes __free(device_node) =
		of_get_child_by_name(dev->of_node, "typec-routes");

	if (!of_device_is_compatible(dev->of_node, "apple,t8122-dcp") ||
	    !routes || !of_get_available_child_count(routes))
		return;
	if (err == -EPROBE_DEFER)
		return;
	dev_err(dev, "probe failed (%d): Type-C display route disabled, the left-front port goes on without it\n",
		err);
	dcp_typec_routes_disable_dev(dev);
}

/*
 * The T8122 dcpext is a component of the panel's DRM device, so it must
 * always register one, or the panel stays on the boot framebuffer. Keep
 * deferring while its suppliers may still appear (bounded by the deferred
 * probe timeout); on any other failure join as an unusable pipe.
 */
static int dcp_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_dcp *dcp;
	void *group;
	int ret;

	if (!of_device_is_compatible(dev->of_node, "apple,t8122-dcpext")) {
		ret = dcp_platform_probe_pipe(pdev);
		if (ret)
			dcp_panel_routes_give_up(dev, ret);
		return ret;
	}

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

	if (READ_ONCE(dcp->ext_dead))
		return 0;

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

	if (READ_ONCE(dcp->ext_dead))
		return 0;

	dcp_enable_typec_work(dcp);
	/* The HDMI output's PHY and crossbar may have lost their setup in sleep. */
	if (dcp->external_native)
		dcp_fabric_hdmi_reinit(dcp, "after system sleep");
	/* Observe future edges before sampling any edges lost in sleep. */
	if (dcp->hdmi_hpd_irq)
		dcp_fabric_run_resume(&dcp_resume_ops, dcp);

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

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,t6020-dcp", .data = &apple_dcp_hw_t6020,  },
	{ .compatible = "apple,t6020-dcpext", .data = &apple_dcp_hw_t6020_dcpext, },
	{ .compatible = "apple,t8112-dcp", .data = &apple_dcp_hw_t8112,  },
	{ .compatible = "apple,t6030-dcp", .data = &apple_dcp_hw_t6030, },
	{ .compatible = "apple,t6030-dcpext", .data = &apple_dcp_hw_t6030_dcpext, },
	{ .compatible = "apple,t8122-dcp", .data = &apple_dcp_hw_t8122, },
	{ .compatible = "apple,t6031-dcp", .data = &apple_dcp_hw_t6031, },
	{ .compatible = "apple,t8122-dcpext", .data = &apple_dcp_hw_t8122_dcpext, },
	{ .compatible = "apple,dcp",       .data = &apple_dcp_hw_dcp,    },
	{ .compatible = "apple,dcpext",    .data = &apple_dcp_hw_dcpext, },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver apple_platform_driver = {
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
