/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The capture sync of the cacheable buffers: which ranges are invalidated
 * for an advance of the DMA, laps included. And how late a capture samples,
 * from the apple,rx-delay-bclks of its source ports.
 */
#include <kunit/test.h>

#define MCA_TEST_RATE		48000
#define MCA_TEST_PERIOD		512
#define MCA_TEST_BUFFER		(4 * MCA_TEST_PERIOD)
/* 12 channels of S16 */
#define MCA_TEST_FRAME		24
#define MCA_TEST_BYTES		(MCA_TEST_BUFFER * MCA_TEST_FRAME)

static s64 mca_test_frames_ns(u64 frames)
{
	return div_u64(frames * NSEC_PER_SEC, MCA_TEST_RATE);
}

static bool mca_test_may_lap(s64 elapsed_ns)
{
	return mca_capture_may_lap(MCA_TEST_RATE, MCA_TEST_BUFFER, MCA_TEST_PERIOD, elapsed_ns);
}

/* A capture samples as late as the slowest of its source ports */
static void mca_rx_delay_of_ports(struct kunit *test)
{
	struct mca_data *mca = kunit_kzalloc(test, struct_size(mca, clusters, 6), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mca->nclusters = 6;

	/* No apple,rx-delay-bclks: no port is late */
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, 0x3), 0);

	/* J615: the two speaker ports are one BCLK late, the jack port is not */
	mca->clusters[0].rx_delay = 1;
	mca->clusters[1].rx_delay = 1;
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, BIT(0) | BIT(1)), 1);
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, BIT(1)), 1);
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, BIT(2)), 0);
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, BIT(0) | BIT(2)), 1);

	/* Bits past the clusters are ignored */
	KUNIT_EXPECT_EQ(test, mca_ports_rx_delay(mca, BIT(6) | BIT(2)), 0);
}

static void mca_capture_lap_by_time(struct kunit *test)
{
	/* The usual cadence: a period, or a little more under IRQ latency */
	KUNIT_EXPECT_FALSE(test, mca_test_may_lap(0));
	KUNIT_EXPECT_FALSE(test, mca_test_may_lap(mca_test_frames_ns(MCA_TEST_PERIOD)));
	KUNIT_EXPECT_FALSE(test, mca_test_may_lap(mca_test_frames_ns(2 * MCA_TEST_PERIOD)));
	/* From a buffer less a period on, the position may hide a lap */
	KUNIT_EXPECT_TRUE(test,
			  mca_test_may_lap(mca_test_frames_ns(MCA_TEST_BUFFER - MCA_TEST_PERIOD)));
	KUNIT_EXPECT_TRUE(test, mca_test_may_lap(mca_test_frames_ns(MCA_TEST_BUFFER)));
	KUNIT_EXPECT_TRUE(test, mca_test_may_lap(mca_test_frames_ns(3 * MCA_TEST_BUFFER)));
	/* No rate, or a clock that went back: assume the worst */
	KUNIT_EXPECT_TRUE(test, mca_capture_may_lap(0, MCA_TEST_BUFFER, MCA_TEST_PERIOD, 0));
	KUNIT_EXPECT_TRUE(test, mca_test_may_lap(-1));
}

static void mca_capture_ranges_follow_the_dma(struct kunit *test)
{
	size_t off[2], len[2];
	const size_t p = MCA_TEST_PERIOD * MCA_TEST_FRAME;

	/* No advance and no lap: nothing */
	KUNIT_EXPECT_EQ(test, mca_capture_ranges(p, p, MCA_TEST_BYTES, false, off, len), 0U);

	/* Forward */
	KUNIT_ASSERT_EQ(test, mca_capture_ranges(p, 2 * p, MCA_TEST_BYTES, false, off, len), 1U);
	KUNIT_EXPECT_EQ(test, off[0], p);
	KUNIT_EXPECT_EQ(test, len[0], p);

	/* Across the end */
	KUNIT_ASSERT_EQ(test, mca_capture_ranges(3 * p, p, MCA_TEST_BYTES, false, off, len), 2U);
	KUNIT_EXPECT_EQ(test, off[0], 3 * p);
	KUNIT_EXPECT_EQ(test, len[0], (size_t)MCA_TEST_BYTES - 3 * p);
	KUNIT_EXPECT_EQ(test, off[1], (size_t)0);
	KUNIT_EXPECT_EQ(test, len[1], p);

	/* Onto the end exactly */
	KUNIT_ASSERT_EQ(test, mca_capture_ranges(3 * p, 0, MCA_TEST_BYTES, false, off, len), 1U);
	KUNIT_EXPECT_EQ(test, off[0], 3 * p);
	KUNIT_EXPECT_EQ(test, len[0], p);
}

/* An exact lap, or more than one, brings the same or a nearby position back */
static void mca_capture_lap_syncs_everything(struct kunit *test)
{
	size_t off[2], len[2];
	const size_t p = MCA_TEST_PERIOD * MCA_TEST_FRAME;
	bool lap;

	/* Exactly one lap: the same position */
	lap = mca_test_may_lap(mca_test_frames_ns(MCA_TEST_BUFFER));
	KUNIT_ASSERT_EQ(test, mca_capture_ranges(p, p, MCA_TEST_BYTES, lap, off, len), 1U);
	KUNIT_EXPECT_EQ(test, off[0], (size_t)0);
	KUNIT_EXPECT_EQ(test, len[0], (size_t)MCA_TEST_BYTES);

	/* Two laps and a period: what the position shows is only the period */
	lap = mca_test_may_lap(mca_test_frames_ns(2 * MCA_TEST_BUFFER + MCA_TEST_PERIOD));
	KUNIT_ASSERT_EQ(test, mca_capture_ranges(p, 2 * p, MCA_TEST_BYTES, lap, off, len), 1U);
	KUNIT_EXPECT_EQ(test, off[0], (size_t)0);
	KUNIT_EXPECT_EQ(test, len[0], (size_t)MCA_TEST_BYTES);
}

static struct kunit_case mca_capture_sync_cases[] = {
	KUNIT_CASE(mca_rx_delay_of_ports),
	KUNIT_CASE(mca_capture_lap_by_time),
	KUNIT_CASE(mca_capture_ranges_follow_the_dma),
	KUNIT_CASE(mca_capture_lap_syncs_everything),
	{}
};

static struct kunit_suite mca_capture_sync_suite = {
	.name = "apple-mca-capture-sync",
	.test_cases = mca_capture_sync_cases,
};
kunit_test_suite(mca_capture_sync_suite);
