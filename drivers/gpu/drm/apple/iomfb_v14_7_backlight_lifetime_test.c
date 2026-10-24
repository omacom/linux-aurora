// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Port-specific controls: run the real swap/present, worker, DPMS dispatch
 * and class teardown. Only the RPC transport is replaced. Threads and a
 * workqueue context bridge retain KUnit's per-task static-stub context.
 */
#include <drm/drm_kunit_helpers.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kthread.h>

struct v14_bl_rpc;
struct v14_bl_actor {
	struct v14_bl_rpc *ctx;
	struct task_struct *task;
	struct completion entered, done;
	unsigned int op;
	bool shown;
};

struct v14_bl_rpc {
	struct kunit *test;
	struct v14_bl_fixture f;
	struct drm_device *drm;
	struct drm_framebuffer fb[2];
	struct drm_gem_dma_object gem[2];
	unsigned int initialized, gems_initialized;
	struct v14_bl_actor actors[2];
	struct completion rpc_entered, rpc_release;
	atomic_t submits;
	u64 iova[5], nits[5];
	bool nits_present[5];
	struct drm_framebuffer *owner[5];
	bool block_first, refuse;
};

static struct v14_bl_rpc *v14_bl_rpc_current;

static int v14_bl_rpc_call(struct apple_dcp_v14 *v14, u32 tag, const void *in, u32 in_size,
			   void *out, u32 out_size, u32 completion,
			   dcp_v14_admit_fn admit, void *admit_cookie)
{
	struct v14_bl_rpc *c = v14_bl_rpc_current;
	unsigned int n;

	KUNIT_EXPECT_TRUE(c->test, mutex_is_locked(&v14->lock));
	memset(out, 0, out_size);
	if (tag == A(406)) {
		put_unaligned_le32(1, out);
		return 0;
	}
	KUNIT_EXPECT_EQ(c->test, tag, A(407));
	KUNIT_EXPECT_EQ(c->test, in_size, (u32)DCP_V14_SWAP_SIZE);
	n = atomic_inc_return(&c->submits) - 1;
	if (n < ARRAY_SIZE(c->iova)) {
		c->iova[n] = get_unaligned_le64((const u8 *)in + DCP_V14_SWAP_IOVA(0));
		c->nits_present[n] = ((const u8 *)in)[DCP_V14_SWAP_NITS_PRESENT];
		c->nits[n] = get_unaligned_le64((const u8 *)in + DCP_V14_SWAP_NITS);
		c->owner[n] = v14->active_fb;
	}
	if (!n && c->block_first) {
		complete(&c->rpc_entered);
		wait_for_completion(&c->rpc_release);
	}
	if (c->refuse)
		put_unaligned_le32(1, (u8 *)out + 5);
	return 0;
}

static void v14_bl_rpc_work(struct work_struct *work)
{
	struct kunit *previous = current->kunit_test;

	current->kunit_test = v14_bl_rpc_current->test;
	dcp_v14_backlight_work(work);
	current->kunit_test = previous;
}

static int v14_bl_actor_run(void *data)
{
	struct v14_bl_actor *a = data;
	struct v14_bl_rpc *c = a->ctx;

	current->kunit_test = c->test;
	complete(&a->entered);
	if (a->op < 2)
		a->shown = dcp_v14_present(c->f.v14, &c->fb[a->op], 64, 64, 0, NULL);
	else if (a->op == 2)
		iomfb_v14_7_backlight_suspend(c->f.dcp);
	else
		iomfb_v14_7_unbind(c->f.dcp);
	complete(&a->done);
	current->kunit_test = NULL;
	return 0;
}

static void v14_bl_rpc_cleanup(void *data)
{
	struct v14_bl_rpc *c = data;
	unsigned int i;

	complete_all(&c->rpc_release);
	for (i = 0; i < ARRAY_SIZE(c->actors); i++)
		if (!IS_ERR_OR_NULL(c->actors[i].task))
			kthread_stop_put(c->actors[i].task);
	dcp_v14_stop_backlight(c->f.v14);
	if (c->f.v14->active_fb) {
		drm_framebuffer_put(c->f.v14->active_fb);
		c->f.v14->active_fb = NULL;
	}
	for (i = 0; i < c->initialized; i++)
		drm_framebuffer_put(&c->fb[i]);
	for (i = 0; i < c->gems_initialized; i++)
		drm_gem_object_put(&c->gem[i].base);
	v14_bl_rpc_current = NULL;
}

static void v14_bl_fb_destroy(struct drm_framebuffer *fb)
{
	drm_framebuffer_cleanup(fb);
}

static void v14_bl_gem_free(struct drm_gem_object *obj)
{
	drm_gem_object_release(obj);
}

static const struct drm_gem_object_funcs v14_bl_gem_funcs = {
	.free = v14_bl_gem_free,
};

static const struct drm_framebuffer_funcs v14_bl_fb_funcs = {
	.destroy = v14_bl_fb_destroy,
};

static struct v14_bl_rpc *v14_bl_rpc_setup(struct kunit *test)
{
	struct v14_bl_rpc *c = kunit_kzalloc(test, sizeof(*c), GFP_KERNEL);
	struct device *dev;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, c);
	c->test = test;
	c->f = v14_bl_fixture(test, &dcp_v14_board_t6030, true);
	dev = drm_kunit_helper_alloc_device(test);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	c->drm = __drm_kunit_helper_alloc_drm_device(test, dev, sizeof(*c->drm), 0,
						     DRIVER_MODESET | DRIVER_GEM);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, c->drm);
	init_completion(&c->rpc_entered);
	init_completion(&c->rpc_release);
	atomic_set(&c->submits, 0);
	v14_bl_rpc_current = c;
	kunit_activate_static_stub(test, dcp_v14_call_guarded, v14_bl_rpc_call);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_bl_rpc_cleanup, c), 0);
	for (i = 0; i < ARRAY_SIZE(c->fb); i++) {
		c->fb[i].dev = c->drm;
		c->fb[i].format = drm_format_info(DRM_FORMAT_XRGB8888);
		c->fb[i].width = 64;
		c->fb[i].height = 64;
		c->fb[i].pitches[0] = 256;
		drm_gem_private_object_init(c->drm, &c->gem[i].base, PAGE_SIZE);
		c->gem[i].base.funcs = &v14_bl_gem_funcs;
		c->gems_initialized++;
		c->gem[i].dma_addr = 0x100000 + i * 0x10000;
		c->fb[i].obj[0] = &c->gem[i].base;
		KUNIT_ASSERT_EQ(test, drm_framebuffer_init(c->drm, &c->fb[i], &v14_bl_fb_funcs), 0);
		c->initialized++;
	}
	c->f.v14->panel_width = 64;
	c->f.v14->panel_height = 64;
	c->f.dcp->fixed_connector_type = DRM_MODE_CONNECTOR_eDP;
	c->f.dcp->connector_type = DRM_MODE_CONNECTOR_eDP;
	c->f.dcp->panel.width_mm = 200;
	c->f.dcp->mode_state.valid = true;
	c->f.dcp->crtc = devm_kzalloc(c->f.dcp->dev, sizeof(*c->f.dcp->crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, c->f.dcp->crtc);
	c->f.dcp->crtc->base.dev = c->drm;
	INIT_DELAYED_WORK(&c->f.v14->backlight_work, v14_bl_rpc_work);
	return c;
}

static struct v14_bl_actor *v14_bl_actor_start(struct kunit *test,
					       struct v14_bl_rpc *c, unsigned int slot,
					       unsigned int op)
{
	struct v14_bl_actor *a = &c->actors[slot];

	a->ctx = c;
	a->op = op;
	init_completion(&a->entered);
	init_completion(&a->done);
	a->task = kthread_create(v14_bl_actor_run, a, "v14-backlight-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, a->task);
	get_task_struct(a->task);
	wake_up_process(a->task);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&a->entered, HZ), 0UL);
	return a;
}

static struct drm_framebuffer *v14_bl_pause_publication(struct apple_dcp_v14 *v14,
							struct drm_framebuffer *fb,
							const u8 *surface, u64 iova,
							u32 width, u32 height, u32 dst_y)
{
	struct v14_bl_rpc *c = v14_bl_rpc_current;

	/* Only the first publication pauses; its eventual call runs the real body. */
	kunit_deactivate_static_stub(c->test, dcp_v14_publish_surface);
	complete(&c->rpc_entered);
	wait_for_completion(&c->rpc_release);
	return dcp_v14_publish_surface(v14, fb, surface, iova, width, height, dst_y);
}

/* Swap and framebuffer publication cannot be reordered by another present. */
static void v14_bl_present_publication_serialized(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);
	struct v14_bl_actor *first, *second;

	kunit_activate_static_stub(test, dcp_v14_publish_surface, v14_bl_pause_publication);
	first = v14_bl_actor_start(test, c, 0, 0);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&c->rpc_entered, HZ), 0UL);
	second = v14_bl_actor_start(test, c, 1, 1);
	KUNIT_EXPECT_EQ(test,
			wait_for_completion_timeout(&second->done, msecs_to_jiffies(20)), 0UL);
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 1);
	complete_all(&c->rpc_release);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&first->done, HZ), 0UL);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&second->done, HZ), 0UL);
	KUNIT_EXPECT_TRUE(test, first->shown);
	KUNIT_EXPECT_TRUE(test, second->shown);
	KUNIT_EXPECT_PTR_EQ(test, c->owner[1], &c->fb[0]);
	KUNIT_EXPECT_PTR_EQ(test, c->f.v14->active_fb, &c->fb[1]);
	KUNIT_EXPECT_EQ(test, c->f.v14->shown_iova, (u64)c->gem[1].dma_addr);
	KUNIT_EXPECT_EQ(test, kref_read(&c->fb[0].base.refcount), 1U);
	KUNIT_EXPECT_EQ(test, kref_read(&c->fb[1].base.refcount), 2U);
}

/* A running brightness RPC pins its copied surface until suspend drains it. */
static void v14_bl_running_work_suspend(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);
	struct v14_bl_actor *stop;

	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	atomic_set(&c->submits, 0);
	c->block_first = true;
	v14_bl_request(test, c->f.dcp, 200);
	dcp_v14_backlight_kick(c->f.dcp);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&c->rpc_entered, HZ), 0UL);
	stop = v14_bl_actor_start(test, c, 0, 2);
	KUNIT_EXPECT_EQ(test, wait_for_completion_timeout(&stop->done, msecs_to_jiffies(20)), 0UL);
	KUNIT_EXPECT_PTR_EQ(test, c->owner[0], &c->fb[0]);
	KUNIT_EXPECT_EQ(test, kref_read(&c->fb[0].base.refcount), 2U);
	complete_all(&c->rpc_release);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&stop->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, c->iova[0], (u64)c->gem[0].dma_addr);
	KUNIT_EXPECT_EQ(test, c->f.v14->bl_gate, DCP_V14_BL_SUSPENDED);
	v14_bl_request(test, c->f.dcp, 250);
	dcp_v14_backlight_kick(c->f.dcp);
	KUNIT_EXPECT_FALSE(test, work_busy(&c->f.v14->backlight_work.work));
	/* A callback queued before the gate shut must also refuse the pending level. */
	dcp_v14_backlight_work(&c->f.v14->backlight_work.work);
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 1);
	iomfb_v14_7_backlight_resume(c->f.dcp);
	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 2);
	KUNIT_EXPECT_TRUE(test, c->nits_present[1]);
	KUNIT_EXPECT_EQ(test, c->nits[1], dcp_v14_nits_binary64(250));
}

/* Actual dcp_poweroff/on must reach the 14.7 blank/restore hooks. */
static void v14_bl_dpms_dispatch(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);
	struct platform_device *pdev = kunit_kzalloc(test, sizeof(*pdev), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, pdev);
	platform_set_drvdata(pdev, c->f.dcp);
	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	KUNIT_EXPECT_FALSE(test, c->nits_present[0]);
	KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(c->f.dcp));
	dcp_poweroff(pdev);
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 2);
	KUNIT_EXPECT_FALSE(test, c->nits_present[1]);
	KUNIT_EXPECT_PTR_EQ(test, c->f.v14->active_fb, NULL);
	/* Even the advertised default must be an explicit first request. */
	dcp_poweron(pdev);
	v14_bl_request(test, c->f.dcp, DCP_V14_NITS_DEFAULT);
	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	KUNIT_EXPECT_TRUE(test, c->nits_present[2]);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_level_known(c->f.dcp));
	dcp_poweroff(pdev);
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 4);
	KUNIT_EXPECT_TRUE(test, c->nits_present[3]);
	KUNIT_EXPECT_EQ(test, c->nits[3], 0ULL);
	KUNIT_EXPECT_PTR_EQ(test, c->f.v14->active_fb, NULL);
	dcp_poweron(pdev);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_pending(c->f.dcp));
	KUNIT_EXPECT_FALSE(test, c->f.dcp->backlight.state.dpms_off);
	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 5);
	KUNIT_EXPECT_TRUE(test, c->nits_present[4]);
	KUNIT_EXPECT_EQ(test, c->nits[4], dcp_v14_nits_binary64(DCP_V14_NITS_DEFAULT));
	KUNIT_EXPECT_FALSE(test, dcp_backlight_pending(c->f.dcp));
}

/* An actual failed submit must leave the loader's level unknown. */
static void v14_bl_failed_submit_unknown(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);

	c->refuse = true;
	v14_bl_request(test, c->f.dcp, 240);
	mutex_lock(&c->f.v14->lock);
	KUNIT_EXPECT_EQ(test, dcp_v14_swap_locked(c->f.v14, NULL, 0, 0, 0, 0, NULL), -EIO);
	mutex_unlock(&c->f.v14->lock);
	KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(c->f.dcp));
	KUNIT_EXPECT_EQ(test, c->f.v14->nits_presents, 0ULL);
	KUNIT_EXPECT_TRUE(test, c->f.v14->failed);
	KUNIT_EXPECT_FALSE(test, work_busy(&c->f.v14->backlight_work.work));
	/* Neither a later kick nor a work callback may retry the failed panel. */
	dcp_v14_backlight_kick(c->f.dcp);
	flush_delayed_work(&c->f.v14->backlight_work);
	dcp_v14_backlight_work(&c->f.v14->backlight_work.work);
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 1);
	KUNIT_EXPECT_FALSE(test, work_busy(&c->f.v14->backlight_work.work));
}

static void v14_bl_mutex_unlock(void *data)
{
	mutex_unlock(data);
}

/* Unbind closes class admission before draining a blocked real registrar. */
static void v14_bl_registration_unbind_race(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);
	struct v14_bl_actor *stop;
	unsigned long deadline;
	bool running = false;

	mutex_lock(&c->f.dcp->bl_register_mutex);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_bl_mutex_unlock,
							&c->f.dcp->bl_register_mutex), 0);
	KUNIT_EXPECT_TRUE(test, schedule_work(&c->f.dcp->bl_register_wq));
	deadline = jiffies + HZ;
	do {
		running = work_busy(&c->f.dcp->bl_register_wq) & WORK_BUSY_RUNNING;
		if (running)
			break;
		usleep_range(50, 100);
	} while (time_before(jiffies, deadline));
	KUNIT_EXPECT_TRUE(test, running);
	stop = v14_bl_actor_start(test, c, 0, 3);
	KUNIT_EXPECT_EQ(test, wait_for_completion_timeout(&stop->done, msecs_to_jiffies(20)), 0UL);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(c->f.dcp->quiescing));
	kunit_release_action(test, v14_bl_mutex_unlock, &c->f.dcp->bl_register_mutex);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&stop->done, HZ), 0UL);
	KUNIT_EXPECT_PTR_EQ(test, c->f.dcp->brightness.bl_dev, NULL);
	KUNIT_EXPECT_PTR_EQ(test, c->f.v14->dcp, NULL);
	KUNIT_EXPECT_EQ(test, c->f.v14->bl_gate, DCP_V14_BL_CLOSED);
	/* A late shared producer is refused as well. */
	schedule_work(&c->f.dcp->bl_register_wq);
	flush_work(&c->f.dcp->bl_register_wq);
	KUNIT_EXPECT_PTR_EQ(test, c->f.dcp->brightness.bl_dev, NULL);
}

static void v14_bl_class_unregistered(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);

	schedule_work(&c->f.dcp->bl_register_wq);
	flush_work(&c->f.dcp->bl_register_wq);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, c->f.dcp->brightness.bl_dev);
	iomfb_v14_7_unbind(c->f.dcp);
	KUNIT_EXPECT_PTR_EQ(test, c->f.dcp->brightness.bl_dev, NULL);
	KUNIT_EXPECT_TRUE(test, c->f.dcp->quiescing);
	iomfb_v14_7_backlight_resume(c->f.dcp);
	KUNIT_EXPECT_EQ(test, c->f.v14->bl_gate, DCP_V14_BL_CLOSED);
}

/* fbdev cannot turn the advertised default into the first brightness request. */
static void v14_bl_fbdev_unknown_level(struct kunit *test)
{
	struct v14_bl_rpc *c = v14_bl_rpc_setup(test);
	struct device unrelated = {};
	struct backlight_device *bd;

	schedule_work(&c->f.dcp->bl_register_wq);
	flush_work(&c->f.dcp->bl_register_wq);
	bd = c->f.dcp->brightness.bl_dev;
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, bd);
	KUNIT_EXPECT_FALSE(test, bd->ops->controls_device(bd, NULL));
	KUNIT_EXPECT_FALSE(test, dcp_backlight_pending(c->f.dcp));
	KUNIT_EXPECT_EQ(test, atomic_read(&c->submits), 0);
	v14_bl_request(test, c->f.dcp, 250);
	KUNIT_ASSERT_TRUE(test, dcp_v14_present(c->f.v14, &c->fb[0], 64, 64, 0, NULL));
	KUNIT_EXPECT_TRUE(test, bd->ops->controls_device(bd, NULL));
	KUNIT_EXPECT_TRUE(test, bd->ops->controls_device(bd, c->f.dcp->dev));
	KUNIT_EXPECT_FALSE(test, bd->ops->controls_device(bd, &unrelated));
}

static struct kunit_case v14_bl_lifetime_cases[] = {
	KUNIT_CASE(v14_bl_present_publication_serialized),
	KUNIT_CASE(v14_bl_running_work_suspend),
	KUNIT_CASE(v14_bl_dpms_dispatch),
	KUNIT_CASE(v14_bl_failed_submit_unknown),
	KUNIT_CASE(v14_bl_registration_unbind_race),
	KUNIT_CASE(v14_bl_class_unregistered),
	KUNIT_CASE(v14_bl_fbdev_unknown_level),
	{}
};

static struct kunit_suite v14_bl_lifetime_suite = {
	.name = "dcp_v14_backlight_lifetime",
	.test_cases = v14_bl_lifetime_cases,
};

kunit_test_suite(v14_bl_lifetime_suite);
