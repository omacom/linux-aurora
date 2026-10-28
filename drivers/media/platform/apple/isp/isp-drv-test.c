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
static void isp_test_dapf_program(struct kunit *test)
{
	static const struct isp_dapf_window windows[] = {
		{ 0x380704000, 0x380704063 },
		{ 0x382824000, 0x382824004 },
	};
	static const struct apple_isp_hw hw = {
		.dapf = windows,
		.num_dapf = ARRAY_SIZE(windows),
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
		put_unaligned_le32(1, &expect[0x20]);
		KUNIT_EXPECT_MEMEQ_MSG(test, bank + i * DAPF_WINDOW_STRIDE, expect,
				       sizeof(expect), "window %u", i);
	}

	/* The slot after the last window is untouched. */
	KUNIT_EXPECT_NULL(test, memchr_inv(bank + ARRAY_SIZE(windows) * DAPF_WINDOW_STRIDE,
					   ISP_TEST_DAPF_FILL, DAPF_WINDOW_STRIDE));
}

static struct kunit_case isp_dapf_test_cases[] = {
	KUNIT_CASE(isp_test_dapf_in_fw_mmio),
	KUNIT_CASE(isp_test_dapf_program),
	{}
};

static struct kunit_suite isp_dapf_test_suite = {
	.name = "apple-isp-dapf",
	.test_cases = isp_dapf_test_cases,
};

kunit_test_suite(isp_dapf_test_suite);

/* Only resident firmware may defer its start: it is detected once. */
static void isp_test_deferred_is_resident(struct kunit *test)
{
	const struct of_device_id *match;

	for (match = apple_isp_of_match; match->compatible[0]; match++) {
		const struct apple_isp_hw *hw = match->data;

		if (hw->resident_fw_deferred)
			KUNIT_EXPECT_TRUE_MSG(test, hw->resident_fw, "%s",
					      match->compatible);
	}
}

static struct kunit_case isp_hw_test_cases[] = {
	KUNIT_CASE(isp_test_deferred_is_resident),
	{}
};

static struct kunit_suite isp_hw_test_suite = {
	.name = "apple-isp-hw",
	.test_cases = isp_hw_test_cases,
};

kunit_test_suite(isp_hw_test_suite);
