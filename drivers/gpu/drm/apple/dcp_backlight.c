// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright (C) The Asahi Linux Contributors */

#include <drm/drm_atomic.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_print.h>

#include <linux/backlight.h>
#include <linux/math64.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include "linux/jiffies.h"

#include "connector.h"
#include "dcp.h"
#include "dcp-internal.h"

#define MIN_BRIGHTNESS_PART1	2U
#define MAX_BRIGHTNESS_PART1	99U
#define MIN_BRIGHTNESS_PART2	103U
#define MAX_BRIGHTNESS_PART2	510U

/*
 * lookup for display brightness 2 to 99 nits
 * */
static u32 brightness_part1[] = {
	0x0000000, 0x0810038, 0x0f000bd, 0x143011c,
	0x1850165, 0x1bc01a1, 0x1eb01d4, 0x2140200,
	0x2380227, 0x2590249, 0x2770269, 0x2930285,
	0x2ac02a0, 0x2c402b8, 0x2d902cf, 0x2ee02e4,
	0x30102f8, 0x314030b, 0x325031c, 0x335032d,
	0x345033d, 0x354034d, 0x362035b, 0x3700369,
	0x37d0377, 0x38a0384, 0x3960390, 0x3a2039c,
	0x3ad03a7, 0x3b803b3, 0x3c303bd, 0x3cd03c8,
	0x3d703d2, 0x3e103dc, 0x3ea03e5, 0x3f303ef,
	0x3fc03f8, 0x4050400, 0x40d0409, 0x4150411,
	0x41d0419, 0x4250421, 0x42d0429, 0x4340431,
	0x43c0438, 0x443043f, 0x44a0446, 0x451044d,
	0x4570454, 0x45e045b, 0x4640461, 0x46b0468,
	0x471046e, 0x4770474, 0x47d047a, 0x4830480,
	0x4890486, 0x48e048b, 0x4940491, 0x4990497,
	0x49f049c, 0x4a404a1, 0x4a904a7, 0x4ae04ac,
	0x4b304b1, 0x4b804b6, 0x4bd04bb, 0x4c204c0,
	0x4c704c5, 0x4cc04c9, 0x4d004ce, 0x4d504d3,
	0x4d904d7, 0x4de04dc, 0x4e204e0, 0x4e704e4,
	0x4eb04e9, 0x4ef04ed, 0x4f304f1, 0x4f704f5,
	0x4fb04f9, 0x4ff04fd, 0x5030501, 0x5070505,
	0x50b0509, 0x50f050d, 0x5130511, 0x5160515,
	0x51a0518, 0x51e051c, 0x5210520, 0x5250523,
	0x5290527, 0x52c052a, 0x52f052e, 0x5330531,
	0x5360535, 0x53a0538, 0x53d053b, 0x540053f,
	0x5440542, 0x5470545, 0x54a0548, 0x54d054c,
	0x550054f, 0x5530552, 0x5560555, 0x5590558,
	0x55c055b, 0x55f055e, 0x5620561, 0x5650564,
	0x5680567, 0x56b056a, 0x56e056d, 0x571056f,
	0x5740572, 0x5760575, 0x5790578, 0x57c057b,
	0x57f057d, 0x5810580, 0x5840583, 0x5870585,
	0x5890588, 0x58c058b, 0x58f058d
};

static u32 brightness_part12[] = { 0x58f058d, 0x59d058f };

/*
 * lookup table for display brightness 103.3 to 510 nits
 * */
static u32 brightness_part2[] = {
	0x59d058f, 0x5b805ab, 0x5d105c5, 0x5e805dd,
	0x5fe05f3, 0x6120608, 0x625061c, 0x637062e,
	0x6480640, 0x6580650, 0x6680660, 0x677066f,
	0x685067e, 0x693068c, 0x6a00699, 0x6ac06a6,
	0x6b806b2, 0x6c406be, 0x6cf06ca, 0x6da06d5,
	0x6e506df, 0x6ef06ea, 0x6f906f4, 0x70206fe,
	0x70c0707, 0x7150710, 0x71e0719, 0x7260722,
	0x72f072a, 0x7370733, 0x73f073b, 0x7470743,
	0x74e074a, 0x7560752, 0x75d0759, 0x7640760,
	0x76b0768, 0x772076e, 0x7780775, 0x77f077c,
	0x7850782, 0x78c0789, 0x792078f, 0x7980795,
	0x79e079b, 0x7a407a1, 0x7aa07a7, 0x7af07ac,
	0x7b507b2, 0x7ba07b8, 0x7c007bd, 0x7c507c2,
	0x7ca07c8, 0x7cf07cd, 0x7d407d2, 0x7d907d7,
	0x7de07dc, 0x7e307e1, 0x7e807e5, 0x7ec07ea,
	0x7f107ef, 0x7f607f3, 0x7fa07f8, 0x7fe07fc
};


/*
 * A new level fades in over DCP_BL_FADE_MS, one brightness-only present per
 * step at most.  Steps that come faster than presents complete merge, so the
 * step only bounds how smooth the fade can be.
 */
#define DCP_BL_FADE_MS		250U
#define DCP_BL_FADE_STEP_MS	16U

/* Panels whose presents carry the level in nits: H17P, and the 14.7 panel. */
static bool dcp_backlight_presents_nits(struct apple_dcp *dcp)
{
	return (dcp->fw_compat == DCP_FIRMWARE_H17P &&
		dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) ||
	       dcp_is_v14_7_panel(dcp);
}

bool dcp_backlight_active(struct apple_dcp *dcp)
{
	return dcp_backlight_presents_nits(dcp) &&
		READ_ONCE(dcp->backlight.state.ready);
}

int dcp_backlight_takeover(struct apple_dcp *dcp, u32 millinits)
{
	u32 maximum, nits;
	int ret;

	if (dcp_backlight_active(dcp))
		return 0;
	/* H17P takes nits up to the panel ceiling, not the older DAC table's. */
	maximum = dcp->brightness.maximum;
	ret = dcp_bl_takeover_nits(maximum, millinits, &nits);
	if (ret)
		return ret;

	/* Keep the first bounded hint. Later reports are not user requests. */
	return iomfb_configure_backlight_h17p(dcp, maximum, true, nits, false, 0);
}

static int dcp_get_brightness(struct backlight_device *bd)
{
	struct apple_dcp *dcp = bl_get_data(bd);
	unsigned long flags;
	u32 actual;

	if (READ_ONCE(dcp->quiescing))
		return -ENODEV;

	if (dcp_backlight_active(dcp)) {
		spin_lock_irqsave(&dcp->backlight.lock, flags);
		actual = dcp->backlight.state.actual;
		spin_unlock_irqrestore(&dcp->backlight.lock, flags);
		return actual;
	}

	return dcp->brightness.nits;
}

int dcp_backlight_configure(struct apple_dcp *dcp, u32 maximum,
			    bool inherited_valid, u32 inherited,
			    bool default_valid, u32 default_nits,
			    void (*kick)(struct apple_dcp *dcp))
{
	unsigned long flags;
	int ret;

	if ((dcp->fw_compat != DCP_FIRMWARE_H17P && !dcp_is_v14_7_panel(dcp)) ||
	    !dcp_has_panel(dcp) || !dcp->crtc || !kick)
		return -EINVAL;

	mutex_lock(&dcp->bl_register_mutex);
	spin_lock_irqsave(&dcp->backlight.lock, flags);
	if (dcp->backlight.state.ready || dcp->brightness.bl_dev) {
		ret = -EBUSY;
	} else {
		ret = dcp_bl_init(&dcp->backlight.state, maximum,
				  inherited_valid, inherited,
				  default_valid, default_nits);
		if (!ret && dcp_is_v14_7_panel(dcp)) {
			/*
			 * The J514S panel jumps to new levels, and its first
			 * present carries the requested level.
			 */
			dcp_bl_jump(&dcp->backlight.state);
			dcp->backlight.kick = kick;
		} else if (!ret) {
			dcp->backlight.state.fade = true;
			dcp->backlight.kick = kick;
		}
	}
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	mutex_unlock(&dcp->bl_register_mutex);
	if (!ret)
		schedule_work(&dcp->bl_register_wq);
	return ret;
}

bool dcp_backlight_seed(struct apple_dcp *dcp, u32 nits)
{
	unsigned long flags;
	bool seeded;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	seeded = dcp_bl_seed(&dcp->backlight.state, nits);
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	return seeded;
}

static void dcp_backlight_kick(struct apple_dcp *dcp)
{
	void (*kick)(struct apple_dcp *dcp) = READ_ONCE(dcp->backlight.kick);

	if (kick)
		kick(dcp);
}

static void dcp_backlight_fade_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(to_delayed_work(work),
					     struct apple_dcp,
					     backlight.fade_work);
	unsigned long flags;
	bool running, kick;
	u32 elapsed;

	if (READ_ONCE(dcp->quiescing) || READ_ONCE(dcp->crashed) ||
	    READ_ONCE(dcp->iomfb.stopped))
		return;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	elapsed = jiffies_to_msecs(jiffies - dcp->backlight.fade_start);
	running = dcp_bl_fade_step(&dcp->backlight.state, elapsed,
				   DCP_BL_FADE_MS);
	/* A rejected present is retried with backoff, carrying the new level. */
	kick = dcp->backlight.state.dirty && !dcp->backlight.state.retries;
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);

	if (kick)
		dcp_backlight_kick(dcp);
	if (running)
		queue_delayed_work(system_wq, &dcp->backlight.fade_work,
				   msecs_to_jiffies(DCP_BL_FADE_STEP_MS));
}

void dcp_backlight_stop(struct apple_dcp *dcp)
{
	cancel_delayed_work_sync(&dcp->backlight.fade_work);
}

static void dcp_backlight_release(void *data)
{
	dcp_backlight_stop(data);
}

/*
 * The backlight device is a devres of @dev, registered after probe, so it is
 * unregistered before this action runs, and no request can start a fade
 * afterwards.
 */
int dcp_backlight_init(struct apple_dcp *dcp, struct device *dev)
{
	spin_lock_init(&dcp->backlight.lock);
	INIT_DELAYED_WORK(&dcp->backlight.fade_work, dcp_backlight_fade_work);
	return devm_add_action_or_reset(dev, dcp_backlight_release, dcp);
}

/* Present the target of a running fade at once, e.g. before system sleep. */
void dcp_backlight_finish_fade(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool kick;

	cancel_delayed_work_sync(&dcp->backlight.fade_work);
	spin_lock_irqsave(&dcp->backlight.lock, flags);
	dcp_bl_fade_step(&dcp->backlight.state, DCP_BL_FADE_MS, DCP_BL_FADE_MS);
	kick = dcp->backlight.state.dirty;
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	if (kick)
		dcp_backlight_kick(dcp);
}

int dcp_backlight_dpms(struct apple_dcp *dcp, bool on)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	ret = dcp_bl_dpms(&dcp->backlight.state, on);
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	if (!ret)
		dcp_backlight_kick(dcp);
	return ret;
}

int dcp_backlight_prepare(struct apple_dcp *dcp, bool have_surface,
			  struct dcp_backlight_present *present)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	ret = dcp_bl_prepare(&dcp->backlight.state, have_surface, present);
	/* No reservation: carry the last completed level on an ordinary present. */
	if (ret == -EALREADY)
		present->nits = dcp->backlight.state.actual;
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	return ret;
}

bool dcp_backlight_complete(struct apple_dcp *dcp, u64 sequence, bool accepted)
{
	unsigned long flags;
	bool completed;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	completed = dcp_bl_complete(&dcp->backlight.state, sequence, accepted);
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	/* Queue retries after the central queue has finished this transaction. */
	return completed;
}

bool dcp_backlight_pending(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	pending = dcp->backlight.state.dirty && !dcp->backlight.state.in_flight;
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	return pending;
}

/*
 * Called from the receive path, which must not queue a transaction itself;
 * the caller schedules the update worker when this returns true.  Requests
 * are honoured at most once per second, so a firmware that repeats the
 * request after every brightness present cannot keep the queue busy.
 */
bool dcp_backlight_resend(struct apple_dcp *dcp)
{
	unsigned long flags;
	bool resend = false;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	if (!dcp->backlight.resent ||
	    time_after(jiffies, dcp->backlight.resent_at + HZ)) {
		resend = dcp_bl_resend(&dcp->backlight.state);
		if (resend) {
			dcp->backlight.resent = true;
			dcp->backlight.resent_at = jiffies;
		}
	}
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	return resend;
}

unsigned int dcp_backlight_retry_delay(struct apple_dcp *dcp)
{
	unsigned long flags;
	unsigned int delay;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	delay = dcp_bl_retry_delay(&dcp->backlight.state);
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	return delay;
}

#define SCALE_FACTOR (1 << 10)

static u32 interpolate(int val, int min, int max, u32 *tbl, size_t tbl_size)
{
	u32 frac;
	u64 low, high;
	u32 interpolated = (tbl_size - 1) * ((val - min) * SCALE_FACTOR) / (max - min);

	size_t index = interpolated / SCALE_FACTOR;

	if (WARN(index + 1 >= tbl_size, "invalid index %zu for brightness %u\n", index, val))
		return tbl[tbl_size / 2];

	frac = interpolated & (SCALE_FACTOR - 1);
	low = tbl[index];
	high = tbl[index + 1];

	return ((frac * high) + ((SCALE_FACTOR - frac) * low)) / SCALE_FACTOR;
}

static u32 calculate_dac(struct apple_dcp *dcp, int val)
{
	u32 dac;

	if (val <= MIN_BRIGHTNESS_PART1)
		return 16 * brightness_part1[0];
	else if (val == MAX_BRIGHTNESS_PART1)
		return 16 * brightness_part1[ARRAY_SIZE(brightness_part1) - 1];
	else if (val == MIN_BRIGHTNESS_PART2)
		return 16 * brightness_part2[0];
	else if (val >= MAX_BRIGHTNESS_PART2)
		return brightness_part2[ARRAY_SIZE(brightness_part2) - 1];

	if (val < MAX_BRIGHTNESS_PART1) {
		dac = interpolate(val, MIN_BRIGHTNESS_PART1, MAX_BRIGHTNESS_PART1,
				  brightness_part1, ARRAY_SIZE(brightness_part1));
	} else if (val > MIN_BRIGHTNESS_PART2) {
		dac = interpolate(val, MIN_BRIGHTNESS_PART2, MAX_BRIGHTNESS_PART2,
				  brightness_part2, ARRAY_SIZE(brightness_part2));
	} else {
		dac = interpolate(val, MAX_BRIGHTNESS_PART1, MIN_BRIGHTNESS_PART2,
				  brightness_part12, ARRAY_SIZE(brightness_part12));
	}

	return 16 * dac;
}

static int drm_crtc_set_brightness(struct apple_dcp *dcp)
{
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc = &dcp->crtc->base;
	int ret = 0;

	drm_modeset_acquire_init(&ctx, DRM_MODESET_ACQUIRE_INTERRUPTIBLE);
	ret = drm_modeset_lock(&crtc->mutex, &ctx);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		drm_modeset_acquire_fini(&ctx);
		return -EDEADLK;
	} else if (ret == -ERESTARTSYS) {
		drm_modeset_acquire_fini(&ctx);
		return -ERESTARTSYS;
	}

	if (!dcp->brightness.update)
		goto done;

	state = drm_atomic_state_alloc(crtc->dev);
	if (!state) {
		ret = -ENOMEM;
		goto done;
	}

	state->acquire_ctx = &ctx;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto fail;
	}

	crtc_state->color_mgmt_changed |= true;

	ret = drm_atomic_commit(state);

fail:
	drm_atomic_state_put(state);
done:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;
}

/*
 * H17P (T8142 H17G): brightness-only commits re-present the scanout and back
 * off within the call.
 */
static int drm_crtc_set_brightness_h17p(struct apple_dcp *dcp)
{
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc *crtc = &dcp->crtc->base;
	int ret = 0;

	state = drm_atomic_state_alloc(crtc->dev);
	if (!state)
		return -ENOMEM;

	drm_modeset_acquire_init(&ctx, DRM_MODESET_ACQUIRE_INTERRUPTIBLE);
	state->acquire_ctx = &ctx;

retry:
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto out;
	}

	if (!dcp->brightness.update)
		goto out;

	/* Re-present the current scanout when brightness is the only change. */
	ret = drm_atomic_add_affected_planes(state, crtc);
	if (ret)
		goto out;

	crtc_state->color_mgmt_changed |= true;

	ret = drm_atomic_commit(state);

out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		ret = drm_modeset_backoff(&ctx);
		if (!ret)
			goto retry;
	}

	drm_atomic_state_put(state);
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;
}

int dcp_backlight_update(struct apple_dcp *dcp)
{
	if (READ_ONCE(dcp->quiescing))
		return -ENODEV;
	if (dcp_backlight_active(dcp)) {
		dcp_backlight_kick(dcp);
		return 0;
	}
	if (dcp_backlight_presents_nits(dcp))
		return -ENODATA;

	/*
	 * Do not actively try to change brightness if no mode is set.
	 * TODO: should this be reflected the in backlight's power property?
	 *       defer this hopefully until it becomes irrelevant due to proper
	 *       drm integrated backlight handling
	 */
	if (!READ_ONCE(dcp->mode_state.valid))
		return 0;

	/* Wait 1 vblank cycle in the hope an atomic swap has already updated
	 * the brightness */
	msleep((1001 + 23) / 24); // 42ms for 23.976 fps

	if (dcp->fw_compat == DCP_FIRMWARE_H17P)
		return drm_crtc_set_brightness_h17p(dcp);
	return drm_crtc_set_brightness(dcp);
}

static int dcp_set_brightness_h17p(struct backlight_device *bd)
{
	int ret = 0;
	struct apple_dcp *dcp = bl_get_data(bd);
	int brightness = backlight_get_brightness(bd);
	unsigned long flags;

	if (READ_ONCE(dcp->quiescing))
		return -ENODEV;

	if (dcp_backlight_active(dcp)) {
		struct dcp_backlight_state *state = &dcp->backlight.state;
		bool fade;
		u32 old;

		/* Preserve the requested level even while the core forces zero. */
		spin_lock_irqsave(&dcp->backlight.lock, flags);
		old = state->target;
		ret = dcp_bl_request(state, bd->props.brightness,
				     backlight_is_blank(bd),
				     bd->props.state & BL_CORE_SUSPENDED);
		/* A new level restarts the fade from the level presented last. */
		fade = !ret && state->fading && state->target != old;
		if (fade)
			dcp->backlight.fade_start = jiffies;
		spin_unlock_irqrestore(&dcp->backlight.lock, flags);
		if (fade)
			queue_delayed_work(system_wq, &dcp->backlight.fade_work,
					   msecs_to_jiffies(DCP_BL_FADE_STEP_MS));
		else if (!ret)
			dcp_backlight_kick(dcp);
		return ret;
	}

	ret = drm_modeset_lock_single_interruptible(&dcp->crtc->base.mutex);
	if (ret)
		return ret;

	dcp->brightness.dac = calculate_dac(dcp, brightness);
	dcp->brightness.update = true;

	drm_modeset_unlock(&dcp->crtc->base.mutex);

	return dcp_backlight_update(dcp);
}

static int dcp_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	struct apple_dcp *dcp = bl_get_data(bd);
	struct drm_modeset_acquire_ctx ctx;
	int brightness = backlight_get_brightness(bd);

	/* The 14.7 panel takes the H17P policy: a level request, no DAC. */
	if (dcp->fw_compat == DCP_FIRMWARE_H17P || dcp_is_v14_7_panel(dcp))
		return dcp_set_brightness_h17p(bd);

	drm_modeset_acquire_init(&ctx, DRM_MODESET_ACQUIRE_INTERRUPTIBLE);
	ret = drm_modeset_lock(&dcp->crtc->base.mutex, &ctx);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		drm_modeset_acquire_fini(&ctx);
		return -EDEADLK;
	} else if (ret == -ERESTARTSYS) {
		drm_modeset_acquire_fini(&ctx);
		return -ERESTARTSYS;
	}

	dcp->brightness.dac = calculate_dac(dcp, brightness);
	dcp->brightness.update = true;

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return dcp_backlight_update(dcp);
}

static bool dcp_backlight_controls_device(struct backlight_device *bd,
					  struct device *display_dev)
{
	struct apple_dcp *dcp = bl_get_data(bd);

	return !display_dev || display_dev == dcp->dev ||
		(dcp->crtc && display_dev == dcp->crtc->base.dev->dev);
}

static const struct backlight_ops dcp_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = dcp_get_brightness,
	.update_status = dcp_set_brightness,
};

/* H17P: fbdev blanking follows only this display. */
static const struct backlight_ops dcp_backlight_ops_h17p = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = dcp_get_brightness,
	.update_status = dcp_set_brightness,
	.controls_device = dcp_backlight_controls_device,
};

int dcp_backlight_register(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	struct backlight_device *bl_dev;
	unsigned long flags;
	struct backlight_properties props = {
		.type = BACKLIGHT_PLATFORM,
		.brightness = dcp->brightness.nits,
		.scale = BACKLIGHT_SCALE_LINEAR,
	};
	props.max_brightness = min(dcp->brightness.maximum, MAX_BRIGHTNESS_PART2 - 1);
	if (dcp_backlight_presents_nits(dcp) && !dcp_backlight_active(dcp))
		return -ENODATA;
	if (dcp_backlight_active(dcp)) {
		spin_lock_irqsave(&dcp->backlight.lock, flags);
		if (!dcp->backlight.state.ready) {
			spin_unlock_irqrestore(&dcp->backlight.lock, flags);
			return -ENODATA;
		}
		/* Freeze takeover before userspace can discover the device. */
		dcp->backlight.state.controlled = true;
		props.brightness = dcp->backlight.state.target;
		props.max_brightness = dcp->backlight.state.maximum;
		spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	}

	bl_dev = devm_backlight_device_register(dev, "apple-panel-bl", dev, dcp,
						dcp->fw_compat == DCP_FIRMWARE_H17P ?
						&dcp_backlight_ops_h17p :
						&dcp_backlight_ops, &props);
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	dcp->brightness.bl_dev = bl_dev;
	if (!dcp_backlight_active(dcp))
		dcp->brightness.dac = calculate_dac(dcp, dcp->brightness.nits);

	return 0;
}

/*
 * External displays that report SupportsBacklightControl, such as the Apple
 * Studio Display, take the same swap backlight fields as an integrated
 * panel.  bl_value is then a signed 32-bit level from 0 (dimmest) to S32_MAX
 * (brightest); DCP converts it for the display.  The backlight device belongs
 * to the connector, so userspace can tell which output it dims.
 */
#define DCP_EXT_BL_MAX	1000

/*
 * The bl_value for the display on this DCP, or -1 to send none: it has no
 * backlight, or userspace has not chosen a level yet, so the display keeps
 * its own.
 */
s32 dcp_ext_backlight_value(struct apple_dcp *dcp)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);
	int level;

	if (!connector || !READ_ONCE(dcp->ext_backlight))
		return -1;

	level = READ_ONCE(connector->bl_level);
	if (level < 0)
		return -1;

	return div_u64((u64)level * S32_MAX, DCP_EXT_BL_MAX);
}

static int dcp_ext_backlight_update_status(struct backlight_device *bd)
{
	struct apple_connector *connector = bl_get_data(bd);
	int level = bd->props.brightness;
	struct platform_device *pdev;
	struct apple_dcp *dcp;

	/*
	 * The display's power belongs to DRM: fbdev blanking never reaches
	 * this device (controls_device), and bl_power does not turn the
	 * display off, so the level is stored and sent whatever their state.
	 *
	 * Until a level is chosen the device reports DCP_EXT_BL_MAX, and
	 * systemd-backlight writes what it reads back when it has nothing
	 * saved. Sending that would turn the display to full brightness.
	 */
	if (READ_ONCE(connector->bl_level) < 0 && level == DCP_EXT_BL_MAX)
		return 0;

	WRITE_ONCE(connector->bl_level, level);

	/* An unrouted port picks the level up when it powers on again. */
	pdev = READ_ONCE(connector->dcp);
	if (!pdev)
		return 0;

	dcp = platform_get_drvdata(pdev);
	/* Pairs with do_swap(): it sees the level or leaves update set. */
	smp_store_release(&dcp->brightness.update, true);

	/*
	 * Commit from a work item: a slider's stream of writes collapses into
	 * a few commits, and none of them blocks with ops_lock held.
	 */
	schedule_work(&connector->bl_commit_wq);
	return 0;
}

static void apple_connector_backlight_commit_work(struct work_struct *work)
{
	struct apple_connector *connector =
		container_of(work, struct apple_connector, bl_commit_wq);
	struct platform_device *pdev = READ_ONCE(connector->dcp);
	int ret;

	if (!pdev)
		return;

	do {
		ret = dcp_backlight_update(platform_get_drvdata(pdev));
	} while (ret == -EDEADLK);

	/* update stays set, so the next swap carries the level anyway */
	if (ret)
		drm_dbg(connector->base.dev, "%s: backlight commit: %d\n",
			connector->base.name, ret);
}

/* Never follow fbdev blanking: DRM owns the external display's power. */
static bool dcp_ext_backlight_controls_device(struct backlight_device *bd,
					      struct device *display_dev)
{
	return false;
}

static const struct backlight_ops dcp_ext_backlight_ops = {
	.update_status = dcp_ext_backlight_update_status,
	.controls_device = dcp_ext_backlight_controls_device,
};

static void apple_connector_backlight_register(struct apple_connector *connector)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = DCP_EXT_BL_MAX,
		.brightness = connector->bl_level < 0 ? DCP_EXT_BL_MAX :
							 connector->bl_level,
	};
	struct backlight_device *bd;
	char name[32];

	/*
	 * Sorts after the integrated panel's apple-panel-bl in any locale:
	 * userspace that takes the first backlight as the built-in screen's
	 * (Omarchy's omarchy-hw-display) keeps the panel.
	 */
	snprintf(name, sizeof(name), "dcp-%s-bl", connector->base.name);
	bd = backlight_device_register(name, connector->base.kdev, connector,
				       &dcp_ext_backlight_ops, &props);
	if (IS_ERR(bd)) {
		drm_warn(connector->base.dev,
			 "%s: failed to register backlight: %pe\n",
			 connector->base.name, bd);
		return;
	}

	connector->bl_dev = bd;
}

static void apple_connector_backlight_unregister(struct apple_connector *connector)
{
	backlight_device_unregister(connector->bl_dev);
	connector->bl_dev = NULL;
}

static void apple_connector_backlight_sync_work(struct work_struct *work)
{
	apple_connector_backlight_sync(container_of(work, struct apple_connector,
						    bl_sync_wq));
}

void apple_connector_backlight_init(struct apple_connector *connector)
{
	mutex_init(&connector->bl_lock);
	INIT_WORK(&connector->bl_sync_wq, apple_connector_backlight_sync_work);
	INIT_WORK(&connector->bl_commit_wq, apple_connector_backlight_commit_work);
	connector->bl_level = -1;
}

/*
 * Register the backlight while a display that supports it is connected, and
 * drop it otherwise.  Called from the connector's hotplug work.
 */
void apple_connector_backlight_sync(struct apple_connector *connector)
{
	struct platform_device *pdev;
	bool want = false;

	mutex_lock(&connector->bl_lock);

	pdev = READ_ONCE(connector->dcp);
	if (connector->bl_allowed && READ_ONCE(connector->connected) && pdev) {
		struct apple_dcp *dcp = platform_get_drvdata(pdev);

		want = READ_ONCE(dcp->ext_backlight);
	}

	if (want && !connector->bl_dev)
		apple_connector_backlight_register(connector);
	else if (!want && connector->bl_dev)
		apple_connector_backlight_unregister(connector);

	/* Attributes and HPD can arrive after the first swap on reconnect.
	 * Replay a chosen level on the current pipeline, even on a static
	 * desktop where userspace has no further commits to submit.
	 */
	if (want && READ_ONCE(connector->bl_level) >= 0) {
		struct apple_dcp *dcp = platform_get_drvdata(pdev);

		/* Publish the retained level before do_swap() consumes update. */
		smp_store_release(&dcp->brightness.update, true);
		schedule_work(&connector->bl_commit_wq);
	}

	mutex_unlock(&connector->bl_lock);
}

int apple_connector_backlight_late_register(struct apple_connector *connector)
{
	mutex_lock(&connector->bl_lock);
	connector->bl_allowed = true;
	mutex_unlock(&connector->bl_lock);

	/* a display may have connected before the connector was registered */
	apple_connector_backlight_sync(connector);

	return 0;
}

void apple_connector_backlight_early_unregister(struct apple_connector *connector)
{
	mutex_lock(&connector->bl_lock);
	connector->bl_allowed = false;
	if (connector->bl_dev)
		apple_connector_backlight_unregister(connector);
	mutex_unlock(&connector->bl_lock);
	/* nothing can queue it again once the device is gone */
	cancel_work_sync(&connector->bl_commit_wq);
}
