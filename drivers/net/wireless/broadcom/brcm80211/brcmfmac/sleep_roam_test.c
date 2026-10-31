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

static void brcmf_sleep_roam_reclaim_lifetime(struct kunit *test)
{
	/* Still owned by firmware: bus up, delete not completed. */
	KUNIT_EXPECT_FALSE(test, brcmf_flowring_reclaim_pktids(0x4378, true,
							       false));
	/* Bus down, or firmware has completed FLOW_RING_DELETE. */
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_reclaim_pktids(0x4378, false,
							      false));
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_reclaim_pktids(0x4378, true,
							      true));
	KUNIT_EXPECT_TRUE(test, brcmf_flowring_reclaim_pktids(0x4378, false,
							      true));
	/* Other chips wait for txstatus instead of this reclaim. */
	KUNIT_EXPECT_FALSE(test, brcmf_flowring_reclaim_pktids(0x4377, false,
							       true));
	KUNIT_EXPECT_FALSE(test, brcmf_flowring_reclaim_pktids(0x4388, true,
							       true));
}

static void brcmf_sleep_roam_tx_request_id(struct kunit *test)
{
	u32 idx;
	u16 gen;
	u32 old_id;
	u32 new_id;
	u32 old_idx;
	u32 new_idx;
	u16 old_gen;
	u16 new_gen;

	KUNIT_EXPECT_EQ(test, brcmf_tx_pktid_next_gen(0), 1);
	KUNIT_EXPECT_EQ(test, brcmf_tx_pktid_next_gen(1), 2);
	KUNIT_EXPECT_EQ(test, brcmf_tx_pktid_next_gen(0xffff), 1);

	KUNIT_EXPECT_EQ(test, brcmf_tx_request_id(0, 1), 0x00010001);
	KUNIT_EXPECT_EQ(test, brcmf_tx_request_id(2047, 2),
			(2u << 16) | 2048);
	KUNIT_EXPECT_TRUE(test, brcmf_tx_request_id_decode(
			       brcmf_tx_request_id(7, 3), &idx, &gen));
	KUNIT_EXPECT_EQ(test, idx, 7);
	KUNIT_EXPECT_EQ(test, gen, 3);

	/* A zero id, or one whose generation was truncated, matches nothing. */
	KUNIT_EXPECT_FALSE(test, brcmf_tx_request_id_decode(0, &idx, &gen));
	KUNIT_EXPECT_FALSE(test, brcmf_tx_request_id_decode(5, &idx, &gen));

	/* The same slot with a newer generation is a different cookie. */
	old_id = brcmf_tx_request_id(7, 1);
	new_id = brcmf_tx_request_id(7, 2);
	KUNIT_EXPECT_TRUE(test, brcmf_tx_request_id_decode(old_id, &old_idx,
							   &old_gen));
	KUNIT_EXPECT_TRUE(test, brcmf_tx_request_id_decode(new_id, &new_idx,
							   &new_gen));
	KUNIT_EXPECT_EQ(test, old_idx, new_idx);
	KUNIT_EXPECT_NE(test, old_gen, new_gen);
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
	KUNIT_CASE(brcmf_sleep_roam_reclaim_lifetime),
	KUNIT_CASE(brcmf_sleep_roam_tx_request_id),
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
