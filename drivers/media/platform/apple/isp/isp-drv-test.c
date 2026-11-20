// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the per-SoC data of the Apple ISP driver, built into
 * isp-drv.c: the DART address filter windows and the firmware start. They
 * need no hardware.
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/unaligned.h>

/* Every address filter window must lie inside a window the firmware is mapped. */
static void isp_test_dapf_in_fw_mmio(struct kunit *test)
{
	const struct of_device_id *match;

	for (match = apple_isp_of_match; match->compatible[0]; match++) {
		const struct apple_isp_hw *hw = match->data;

		for (unsigned int i = 0; i < hw->num_dapf; i++) {
			const struct isp_dapf_window *w = &hw->dapf[i];
			bool inside = false;

			KUNIT_EXPECT_LE_MSG(test, w->start, w->end, "%s window %u",
					    match->compatible, i);
			for (unsigned int j = 0; j < hw->num_fw_mmio; j++) {
				const struct isp_mmio_window *m = &hw->fw_mmio[j];

				if (w->start >= m->base && w->end < m->base + m->size)
					inside = true;
			}
			KUNIT_EXPECT_TRUE_MSG(test, inside,
					      "%s window %u 0x%llx-0x%llx is not in fw_mmio",
					      match->compatible, i, w->start, w->end);
		}
	}
}

#define ISP_TEST_DAPF_FILL	0xa5

/*
 * Each window fills its own 0x40-byte slot with exactly these words, and
 * leaves every other byte alone. The bank is plain memory here: the driver
 * is arm64 only, where the MMIO accessors work on it unchanged.
 */
static void isp_test_dapf_program_mask(struct kunit *test, u32 sidmask)
{
	static const struct isp_dapf_window windows[] = {
		{ 0x380704000, 0x380704063 },
		{ 0x382824000, 0x382824004 },
	};
	const struct apple_isp_hw hw = {
		.dapf = windows,
		.num_dapf = ARRAY_SIZE(windows),
		.dapf_sidmask = sidmask,
	};
	size_t size = (ARRAY_SIZE(windows) + 1) * DAPF_WINDOW_STRIDE;
	u8 expect[DAPF_WINDOW_STRIDE];
	struct apple_isp *isp;
	u8 *bank;

	isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp);
	isp->dev = kunit_device_register(test, "apple-isp-dapf-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp->dev);
	bank = kunit_kmalloc(test, size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, bank);
	memset(bank, ISP_TEST_DAPF_FILL, size);
	isp->hw = &hw;
	isp->dapf = (__force void __iomem *)bank;

	KUNIT_ASSERT_EQ(test, apple_isp_program_dapf(isp), 0);

	for (unsigned int i = 0; i < ARRAY_SIZE(windows); i++) {
		memset(expect, ISP_TEST_DAPF_FILL, sizeof(expect));
		put_unaligned_le32(0x31, &expect[0x00]);
		put_unaligned_le32(0, &expect[0x04]);
		put_unaligned_le64(windows[i].start, &expect[0x08]);
		put_unaligned_le64(windows[i].end, &expect[0x10]);
		put_unaligned_le32(sidmask ?: 1, &expect[0x20]);
		KUNIT_EXPECT_MEMEQ_MSG(test, bank + i * DAPF_WINDOW_STRIDE, expect,
				       sizeof(expect), "window %u", i);
	}

	/* The slot after the last window is untouched. */
	KUNIT_EXPECT_NULL(test, memchr_inv(bank + ARRAY_SIZE(windows) * DAPF_WINDOW_STRIDE,
					   ISP_TEST_DAPF_FILL, DAPF_WINDOW_STRIDE));
}

static void isp_test_dapf_program(struct kunit *test)
{
	isp_test_dapf_program_mask(test, 0);
}

static void isp_test_dapf_program_t6050(struct kunit *test)
{
	isp_test_dapf_program_mask(test, 0xcc3);
}

/* The J713 ADT's ISP DART address filter, as the boot loader copies it */
static const struct isp_dapf_window isp_test_dapf_j713[] = {
	{ 0x380704000, 0x380704063 },
	{ 0x380730000, 0x380733fff },
	{ 0x380e3c000, 0x380e3c003 },
	{ 0x39062c000, 0x390648003 },
	{ 0x501660000, 0x501660003 },
	{ 0x220004000, 0x2200162fc },
	{ 0x220044000, 0x2200562fc },
	{ 0x2201c4000, 0x2201d62fc },
	{ 0x222004000, 0x2220162fc },
	{ 0x222044000, 0x2220562fc },
	{ 0x2221c4000, 0x2221d62fc },
	{ 0x3803c0000, 0x3803e4000 },
	{ 0x382824000, 0x382824004 },
};

/*
 * The MMIO windows made from the address filter are whole 16 KiB pages,
 * sorted and apart from each other, and hold every filter window.
 */
static void isp_test_dapf_mmio_windows(struct kunit *test)
{
	const unsigned int num = ARRAY_SIZE(isp_test_dapf_j713);
	struct isp_mmio_window mmio[ARRAY_SIZE(isp_test_dapf_j713)];
	unsigned int used = isp_dapf_mmio_windows(isp_test_dapf_j713, num, mmio);

	/* None of the J713's windows share a page. */
	KUNIT_EXPECT_EQ(test, used, num);
	for (unsigned int i = 0; i < used; i++) {
		KUNIT_EXPECT_EQ(test, mmio[i].base % SZ_16K, 0);
		KUNIT_EXPECT_EQ(test, mmio[i].size % SZ_16K, 0);
		KUNIT_EXPECT_GT(test, mmio[i].size, 0);
		if (i)
			KUNIT_EXPECT_GT(test, mmio[i].base,
					mmio[i - 1].base + mmio[i - 1].size);
	}
	for (unsigned int i = 0; i < num; i++) {
		const struct isp_dapf_window *w = &isp_test_dapf_j713[i];
		bool inside = false;

		for (unsigned int j = 0; j < used; j++)
			if (w->start >= mmio[j].base &&
			    w->end < mmio[j].base + mmio[j].size)
				inside = true;
		KUNIT_EXPECT_TRUE_MSG(test, inside, "window %u 0x%llx-0x%llx",
				      i, w->start, w->end);
	}
	/* The PMGR window ends on a page boundary, inclusively. */
	KUNIT_EXPECT_EQ(test, mmio[6].base, 0x3803c0000);
	KUNIT_EXPECT_EQ(test, mmio[6].size, 0x28000);
}

/* Windows in one page, or in adjacent pages, are mapped as one. */
static void isp_test_dapf_mmio_merge(struct kunit *test)
{
	static const struct isp_dapf_window dapf[] = {
		{ 0x108000, 0x108003 },
		{ 0x101000, 0x101003 },
		{ 0x103ffc, 0x104003 },
		{ 0x200000, 0x200003 },
	};
	struct isp_mmio_window mmio[ARRAY_SIZE(dapf)];
	unsigned int used = isp_dapf_mmio_windows(dapf, ARRAY_SIZE(dapf), mmio);

	KUNIT_ASSERT_EQ(test, used, 2);
	KUNIT_EXPECT_EQ(test, mmio[0].base, 0x100000);
	KUNIT_EXPECT_EQ(test, mmio[0].size, 0xc000);
	KUNIT_EXPECT_EQ(test, mmio[1].base, 0x200000);
	KUNIT_EXPECT_EQ(test, mmio[1].size, 0x4000);
}

static struct kunit_case isp_dapf_test_cases[] = {
	KUNIT_CASE(isp_test_dapf_in_fw_mmio),
	KUNIT_CASE(isp_test_dapf_mmio_windows),
	KUNIT_CASE(isp_test_dapf_mmio_merge),
	KUNIT_CASE(isp_test_dapf_program),
	KUNIT_CASE(isp_test_dapf_program_t6050),
	{}
};

static struct kunit_suite isp_dapf_test_suite = {
	.name = "apple-isp-dapf",
	.test_cases = isp_dapf_test_cases,
};

kunit_test_suite(isp_dapf_test_suite);

/*
 * A deferred start detects the camera once: its firmware must keep running
 * or start again. Only firmware that starts again, from a restored copy or
 * from its own memory, is quiesced before the power-down.
 */
static void isp_test_firmware_lifetime(struct kunit *test)
{
	const struct of_device_id *match;

	for (match = apple_isp_of_match; match->compatible[0]; match++) {
		const struct apple_isp_hw *hw = match->data;

		if (hw->deferred_start)
			KUNIT_EXPECT_TRUE_MSG(test, hw->resident_fw || hw->fw_restore,
					      "%s", match->compatible);
		KUNIT_EXPECT_FALSE_MSG(test, hw->resident_fw && hw->fw_restore, "%s",
				       match->compatible);
		if (hw->coproc_quiesce)
			KUNIT_EXPECT_TRUE_MSG(test, hw->fw_restore || hw->fw_restart, "%s",
					      match->compatible);
		/* Only firmware that starts again idles before it stops. */
		if (hw->fw_idle_ms)
			KUNIT_EXPECT_TRUE_MSG(test, hw->fw_restore || hw->fw_restart, "%s",
					      match->compatible);
		/* The fallback keeps restartable firmware resident. */
		if (hw->resident_fallback)
			KUNIT_EXPECT_TRUE_MSG(test, hw->fw_restore && hw->num_dapf,
					      "%s", match->compatible);
	}
}

static void isp_test_sensor_sysfs(struct kunit *test)
{
	struct device *dev = kunit_device_register(test, "apple-isp-sensor-test");
	struct apple_isp *isp;
	char buf[32];

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	KUNIT_EXPECT_EQ(test, sensor_show(dev, &dev_attr_sensor, buf), -ENODATA);

	isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp);
	mutex_init(&isp->video_lock);
	dev_set_drvdata(dev, isp);

	/* Before detection on deferred-start hardware, report -ENODATA. */
	KUNIT_EXPECT_EQ(test, sensor_show(dev, &dev_attr_sensor, buf), -ENODATA);

	isp_get_current_format(isp)->id = ISP_IMX958_2224_01;
	isp->camera_detected = true;
	KUNIT_EXPECT_EQ(test, sensor_show(dev, &dev_attr_sensor, buf),
			(ssize_t)strlen("imx958-uw\n"));
	KUNIT_EXPECT_STREQ(test, buf, "imx958-uw\n");
}

static struct kunit_case isp_hw_test_cases[] = {
	KUNIT_CASE(isp_test_firmware_lifetime),
	KUNIT_CASE(isp_test_sensor_sysfs),
	{}
};

static struct kunit_suite isp_hw_test_suite = {
	.name = "apple-isp-hw",
	.test_cases = isp_hw_test_cases,
};

kunit_test_suite(isp_hw_test_suite);
