// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/io.h>

#include "avd.h"
#include "avd-regs.h"

struct avd_pm_test {
	struct avd_dev avd;
	bool queue_initialized;
	struct avd_deferred_unmap unmap;
	unsigned int draining;
	unsigned int step;
	unsigned int unmap_step;
	unsigned int boot_step;
	unsigned int resubmit_step;
	unsigned int shutdowns;
	unsigned int submissions;
	int boot_error;
	int suspend_error;
	bool unmapped_active;
};

static struct avd_pm_test *avd_pm_fixture(struct avd_dev *avd)
{
	return container_of(avd, struct avd_pm_test, avd);
}

static bool avd_pm_test_idle(struct avd_dev *avd)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);

	return avd_pm_can_suspend(fixture->avd.frames, fixture->draining);
}

static int avd_pm_test_drain(struct avd_dev *avd)
{
	return avd_pm_test_idle(avd) ? 0 : -EBUSY;
}

static int avd_pm_test_boot(struct avd_dev *avd)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);

	fixture->boot_step = ++fixture->step;
	return fixture->boot_error;
}

static void avd_pm_test_shutdown(struct avd_dev *avd)
{
	avd_pm_fixture(avd)->shutdowns++;
}

static int avd_pm_test_suspend(struct avd_dev *avd)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);

	if (fixture->suspend_error)
		return fixture->suspend_error;
	return avd_pm_backend_suspend(avd);
}

static int avd_pm_test_submit(struct avd_dev *avd, struct avd_frame *frame)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);

	fixture->resubmit_step = ++fixture->step;
	fixture->submissions++;
	frame->state = AVD_FRAME_SUBMITTED;
	return 0;
}

static void avd_pm_test_resubmit(struct avd_dev *avd)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);
	unsigned int i;

	fixture->resubmit_step = ++fixture->step;
	for (i = 0; i < AVD_MAX_FRAMES; i++)
		if (fixture->avd.frames[i].state == AVD_FRAME_RESERVED)
			avd->backend->submit(avd, &fixture->avd.frames[i]);
}

static void avd_pm_test_unmap(struct avd_dev *avd,
			      struct avd_deferred_unmap *unmap)
{
	struct avd_pm_test *fixture = avd_pm_fixture(avd);

	fixture->unmap_step = ++fixture->step;
	fixture->unmapped_active = avd->pm.dart_active;
}

static const struct avd_queue_pm_ops avd_pm_test_queue = {
	.idle = avd_pm_test_idle,
	.drain = avd_pm_test_drain,
	.resubmit = avd_pm_test_resubmit,
};

static const struct avd_backend_ops avd_pm_test_backend = {
	.name = "test",
	.boot = avd_pm_test_boot,
	.shutdown = avd_pm_test_shutdown,
	.submit = avd_pm_test_submit,
	.suspend = avd_pm_test_suspend,
	.resume = avd_pm_backend_resume,
};

static int avd_pm_test_init(struct kunit *test)
{
	struct avd_pm_test *fixture;

	fixture = kunit_kzalloc(test, sizeof(*fixture), GFP_KERNEL);
	if (!fixture)
		return -ENOMEM;
	test->priv = fixture;
	avd_pm_init(&fixture->avd, &avd_pm_test_backend, &avd_pm_test_queue);
	INIT_LIST_HEAD(&fixture->unmap.node);
	fixture->unmap.unmap = avd_pm_test_unmap;
	return 0;
}

static void avd_pm_suspend_gate_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	unsigned int i, state;

	KUNIT_EXPECT_TRUE(test, avd_pm_test_idle(&fixture->avd));
	fixture->draining = 1;
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(&fixture->avd), -EBUSY);
	fixture->draining = 0;

	/* An occupied slot anywhere vetoes idle, even after completion IRQ. */
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		for (state = AVD_FRAME_RESERVED; state <= AVD_FRAME_DONE; state++) {
			fixture->avd.frames[i].state = state;
			KUNIT_EXPECT_EQ(test, avd_pm_power_off(&fixture->avd), -EBUSY);
		}
		fixture->avd.frames[i].state = AVD_FRAME_FREE;
	}
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 0U);
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(&fixture->avd), 0);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 1U);
}

static void avd_pm_resume_order_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;

	avd_pm_unmap(avd, &fixture->unmap);
	KUNIT_EXPECT_EQ(test, fixture->unmap_step, 0U);
	fixture->avd.frames[0].state = AVD_FRAME_RESERVED;
	fixture->avd.frames[1].state = AVD_FRAME_SUBMITTED;
	fixture->avd.frames[2].state = AVD_FRAME_DONE;
	fixture->avd.frames[15].state = AVD_FRAME_RESERVED;
	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	KUNIT_EXPECT_TRUE(test, fixture->unmapped_active);
	KUNIT_EXPECT_LT(test, fixture->unmap_step, fixture->boot_step);
	KUNIT_EXPECT_LT(test, fixture->boot_step, fixture->resubmit_step);
	KUNIT_EXPECT_EQ(test, fixture->submissions, 2U);
	KUNIT_EXPECT_EQ(test, fixture->avd.frames[0].state, AVD_FRAME_SUBMITTED);
	KUNIT_EXPECT_EQ(test, fixture->avd.frames[2].state, AVD_FRAME_DONE);
	KUNIT_EXPECT_TRUE(test, list_empty(&avd->pm.deferred_unmaps));
}

static void avd_pm_resume_failure_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;

	fixture->avd.frames[0].state = AVD_FRAME_RESERVED;
	fixture->boot_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, avd_pm_power_on(avd), -ETIMEDOUT);
	KUNIT_EXPECT_FALSE(test, avd->pm.dart_active);
	KUNIT_EXPECT_EQ(test, fixture->resubmit_step, 0U);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 1U);
	KUNIT_EXPECT_EQ(test, fixture->avd.frames[0].state, AVD_FRAME_RESERVED);

	/* A later successful resume retries the still-unposted frame once. */
	fixture->boot_error = 0;
	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	KUNIT_EXPECT_EQ(test, fixture->submissions, 1U);
}

static void avd_pm_suspend_failure_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;

	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	fixture->suspend_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(avd), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, avd->pm.dart_active);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 0U);
}

static void avd_pm_deferred_unmap_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;
	unsigned int step;

	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	avd_pm_unmap(avd, &fixture->unmap);
	KUNIT_EXPECT_TRUE(test, fixture->unmapped_active);
	KUNIT_EXPECT_TRUE(test, list_empty(&avd->pm.deferred_unmaps));
	KUNIT_ASSERT_EQ(test, avd_pm_power_off(avd), 0);
	step = fixture->unmap_step;
	avd_pm_unmap(avd, &fixture->unmap);
	KUNIT_EXPECT_EQ(test, fixture->unmap_step, step);
	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	KUNIT_EXPECT_GT(test, fixture->unmap_step, step);
	KUNIT_EXPECT_TRUE(test, list_empty(&avd->pm.deferred_unmaps));
}

static void avd_pm_real_queue_resume_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;
	struct avd_ctx *ctx;
	unsigned int i;

	avd->dev = kunit_device_register(test, "avd-pm-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, avd->dev);
	KUNIT_ASSERT_EQ(test, avd_queue_init(avd), 0);
	fixture->queue_initialized = true;
	avd->backend = &avd_pm_test_backend;
	avd->pm.queue = &avd_pm_queue_ops;
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->dev = avd;
	for (i = 0; i < 3; i++) {
		avd->frames[i].ctx = ctx;
		avd->frames[i].seq = i + 1;
		avd->frames[i].deadline = jiffies + msecs_to_jiffies(60000);
		avd->frames[i].state = AVD_FRAME_RESERVED;
	}
	/* This one was posted before the power transition; never replay it. */
	avd->frames[1].state = AVD_FRAME_SUBMITTED;
	avd->inflight = 3;
	KUNIT_ASSERT_EQ(test, avd_pm_power_on(avd), 0);
	flush_delayed_work(&avd->queue_work);
	cancel_delayed_work_sync(&avd->queue_work);
	KUNIT_EXPECT_EQ(test, fixture->submissions, 2U);
	KUNIT_EXPECT_LT(test, fixture->boot_step, fixture->resubmit_step);
	KUNIT_EXPECT_EQ(test, avd->frames[0].state, AVD_FRAME_SUBMITTED);
	KUNIT_EXPECT_EQ(test, avd->frames[2].state, AVD_FRAME_SUBMITTED);
}

static void avd_pm_real_queue_gate_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;

	spin_lock_init(&avd->queue_lock);
	avd->pm.queue = &avd_pm_queue_ops;
	avd_pm_drain_begin(avd);
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(avd), -EBUSY);
	avd_pm_drain_end(avd);
	avd->frames[15].state = AVD_FRAME_DONE;
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(avd), -EBUSY);
	avd->frames[15].state = AVD_FRAME_FREE;
	KUNIT_EXPECT_EQ(test, avd_pm_power_off(avd), 0);
}

static void avd_pm_open_boot_replay_test(struct kunit *test)
{
	static const struct avd_variant variant = { .revision = 3 };
	static const u8 image[] = { 0x12, 0x34, 0x56, 0x78 };
	static const struct firmware fw = { .data = image, .size = sizeof(image) };
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;
	struct resource code = {
		.name = "code", .flags = IORESOURCE_MEM,
		.start = 0x1000, .end = 0x10ff,
	};
	struct platform_device *pdev;
	u8 observed[sizeof(image)];
	unsigned int i;

	pdev = kunit_kzalloc(test, sizeof(*pdev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pdev);
	pdev->resource = &code;
	pdev->num_resources = 1;
	avd->pdev = pdev;
	avd->variant = &variant;
	avd->fw = &fw;
	avd->backend = &avd_open_backend;
	/* Ordinary test RAM, never a mapped target aperture. */
	avd->code = (__force void __iomem *)kunit_kzalloc(test, 0x100, GFP_KERNEL);
	avd->mbox = (__force void __iomem *)kunit_kzalloc(test, 0x100, GFP_KERNEL);
	avd->pm.piodma = (__force void __iomem *)kunit_kzalloc(test, 0x28, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, avd->code);
	KUNIT_ASSERT_NOT_NULL(test, avd->mbox);
	KUNIT_ASSERT_NOT_NULL(test, avd->pm.piodma);
	avd->pm.sram_phys = 0x123456780ULL;
	for (i = 0; i < 2; i++) {
		/* Model loss of both ITCM and PIODMA configuration between boots. */
		memset_io(avd->code, 0, sizeof(image));
		writel(0, avd->pm.piodma + 0x24);
		/* The fixture supplies firmware readiness; no firmware is executed. */
		writel(1, avd->mbox + AVD_REG_FLAG0_SET);
		KUNIT_ASSERT_EQ(test, avd_backend_boot(avd), 0);
		KUNIT_EXPECT_EQ(test, readl(avd->pm.piodma + 0x24), 0x12345678U);
		memcpy_fromio(observed, avd->code, sizeof(observed));
		KUNIT_EXPECT_MEMEQ(test, observed, image, sizeof(image));
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_RUN_CTRL),
				(u32)AVD_RUN_CTRL_UNK_RUN);
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_MBOX1_STATUS),
				(u32)AVD_MBOX_ENABLE);
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_MBOX_IRQ_ENABLE),
				(u32)AVD_MBOX1_NOT_EMPTY);
		avd_shutdown(avd);
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_RUN_CTRL),
				(u32)(AVD_RUN_CTRL_UNK_STOP));
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_FLAG0_CLR), 1U);
		KUNIT_EXPECT_EQ(test, readl(avd->mbox + AVD_REG_MBOX_IRQ_ENABLE), 0U);
	}
}

static void avd_pm_recovery_dart_gate_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;
	struct iommu_domain domain = {};

	/* A stopped device cannot be reset through powered-off registers. */
	KUNIT_EXPECT_EQ(test, avd_pm_recover(avd), -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 0U);
	avd->pm.dart_active = true;
	avd->domain = &domain;
	/* No empty domain means no valid DART flush: never reboot anyway. */
	KUNIT_EXPECT_EQ(test, avd_pm_recover(avd), -ENODEV);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 1U);
	KUNIT_EXPECT_EQ(test, fixture->boot_step, 0U);
}

static void avd_pm_recovery_boot_test(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;
	struct avd_dev *avd = &fixture->avd;

	avd->pm.dart_active = true;
	fixture->boot_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, avd_pm_recover(avd), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, fixture->shutdowns, 2U);
	KUNIT_EXPECT_EQ(test, fixture->resubmit_step, 0U);
	fixture->boot_error = 0;
	KUNIT_EXPECT_EQ(test, avd_pm_recover(avd), 0);
	KUNIT_EXPECT_EQ(test, avd->fw_generation, 1ULL);
	/* Recovery must fail old slots before core reopens the host IRQ. */
	KUNIT_EXPECT_FALSE(test, avd->pm.irq_enabled);
	KUNIT_EXPECT_EQ(test, fixture->resubmit_step, 0U);
}

static void avd_pm_test_exit(struct kunit *test)
{
	struct avd_pm_test *fixture = test->priv;

	if (fixture->queue_initialized)
		cancel_delayed_work_sync(&fixture->avd.queue_work);
}

static struct kunit_case avd_pm_test_cases[] = {
	KUNIT_CASE(avd_pm_suspend_gate_test),
	KUNIT_CASE(avd_pm_resume_order_test),
	KUNIT_CASE(avd_pm_resume_failure_test),
	KUNIT_CASE(avd_pm_suspend_failure_test),
	KUNIT_CASE(avd_pm_deferred_unmap_test),
	KUNIT_CASE(avd_pm_real_queue_resume_test),
	KUNIT_CASE(avd_pm_real_queue_gate_test),
	KUNIT_CASE(avd_pm_open_boot_replay_test),
	KUNIT_CASE(avd_pm_recovery_dart_gate_test),
	KUNIT_CASE(avd_pm_recovery_boot_test),
	{}
};

static struct kunit_suite avd_pm_test_suite = {
	.name = "apple-avd-pm",
	.init = avd_pm_test_init,
	.exit = avd_pm_test_exit,
	.test_cases = avd_pm_test_cases,
};

kunit_test_suite(avd_pm_test_suite);
