// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>

#include "sleep_roam.h"

static void brcmf_sleep_roam_chip(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, brcmf_is_bcm4378(0x4378));
	KUNIT_EXPECT_FALSE(test, brcmf_is_bcm4378(0x4377));
	KUNIT_EXPECT_FALSE(test, brcmf_is_bcm4378(0x4387));
	KUNIT_EXPECT_FALSE(test, brcmf_is_bcm4378(0x4388));
	KUNIT_EXPECT_FALSE(test, brcmf_is_bcm4378(0));
}

static void brcmf_sleep_roam_txstatus_wait(struct kunit *test)
{
	KUNIT_EXPECT_FALSE(test, brcmf_flowring_wait_txstatus(0x4378));
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_wait_txstatus(0x4377));
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_wait_txstatus(0x4388));
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_wait_txstatus(0x4350));
}

static void brcmf_sleep_roam_assoc_retry(struct kunit *test)
{
	unsigned int failures = 0;

	KUNIT_EXPECT_EQ(test, BRCMF_ASSOC_FW_ERROR, -EBADE);
	KUNIT_EXPECT_TRUE(test, brcmf_assoc_info_should_retry(0x4378, -EBADE,
							      0, true));
	KUNIT_EXPECT_TRUE(test, brcmf_assoc_info_should_retry(0x4378, -EBADE,
							      6, true));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4378, -EBADE,
							       7, true));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4378, -EBADE,
							       0, false));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4378, -EIO,
							       0, true));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4378, -ETIMEDOUT,
							       0, true));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4377, -EBADE,
							       0, true));
	KUNIT_EXPECT_FALSE(test, brcmf_assoc_info_should_retry(0x4387, -EBADE,
							       0, true));

	while (brcmf_assoc_info_should_retry(0x4378, -EBADE, failures, true))
		failures++;
	KUNIT_EXPECT_EQ(test, failures, BRCMF_ASSOC_INFO_ATTEMPTS - 1);
	KUNIT_EXPECT_EQ(test, failures * BRCMF_ASSOC_INFO_RETRY_MS, 175U);
}

static struct kunit_case brcmf_sleep_roam_cases[] = {
	KUNIT_CASE(brcmf_sleep_roam_chip),
	KUNIT_CASE(brcmf_sleep_roam_txstatus_wait),
	KUNIT_CASE(brcmf_sleep_roam_assoc_retry),
	{}
};

static struct kunit_suite brcmf_sleep_roam_suite = {
	.name = "brcmfmac-sleep-roam",
	.test_cases = brcmf_sleep_roam_cases,
};

kunit_test_suite(brcmf_sleep_roam_suite);

MODULE_DESCRIPTION("KUnit tests for BCM4378 sleep and roam timeouts");
MODULE_LICENSE("GPL");
