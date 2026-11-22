// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * KUnit cases for the PMP CPU memory-floor governor logic
 * (pmp-floor-gov.h). The host harness tools/asahi/pmp-floor-tests.py
 * covers the same header in more depth, with mutants.
 */
#include <kunit/test.h>

#include "pmp-floor-gov.h"

#define MS PMP_FLOOR_NS_PER_MS

static const struct pmp_floor_gov_cfg defaults = {
	.cpu_busy_pct = 80, .cpu_sum_pct = 90, .gpu_busy_pct = 40, .raise_samples = 2,
	.hold_ms = 500, .min_interval_ms = 100,
};

/* A request-class lane: class 0x21 at bits 35:32 up to class 0x25 at 51:48. */
#define LANE(cls, level) ((unsigned long long)(level) << (32 + 4 * ((cls) - 0x21)))

/* apple,cpu-memory-floor: J613 <0x21 2 0x25 4>, T6000 <0x21 3 0x25 2>. */
static void floor_levels(struct kunit *test)
{
	static const unsigned int j613[] = { 0x21, 2, 0x25, 4 };
	static const unsigned int t6000[] = { 0x21, 3, 0x25, 2 };
	static const unsigned int one[] = { 0x23, 15 };
	static const unsigned int zero[] = { 0x21, 0, 0x25, 0 };
	static const unsigned int twice[] = { 0x21, 1, 0x21, 2 };
	static const unsigned int low[] = { 0x20, 1 };
	static const unsigned int high[] = { 0x26, 1 };
	static const unsigned int wide[] = { 0x21, 16 };
	static const unsigned int all[] = { 0x21, 1, 0x22, 1, 0x23, 1, 0x24, 1, 0x25, 1, 0x21, 1 };
	unsigned long long w = 0;

	KUNIT_EXPECT_TRUE(test, pmp_floor_word_of(j613, 4, &w));
	KUNIT_EXPECT_EQ(test, w, PMP_FLOOR_VALID | LANE(0x21, 2) | LANE(0x25, 4));
	KUNIT_EXPECT_TRUE(test, pmp_floor_word_of(t6000, 4, &w));
	KUNIT_EXPECT_EQ(test, w, PMP_FLOOR_VALID | LANE(0x21, 3) | LANE(0x25, 2));
	KUNIT_EXPECT_EQ(test, pmp_floor_level(w, 0x21), 3U);
	KUNIT_EXPECT_EQ(test, pmp_floor_level(w, 0x25), 2U);
	KUNIT_EXPECT_EQ(test, pmp_floor_level(w, 0x23), 0U);
	KUNIT_EXPECT_TRUE(test, pmp_floor_word_of(one, 2, &w));
	KUNIT_EXPECT_EQ(test, w, PMP_FLOOR_VALID | LANE(0x23, 15));
	KUNIT_EXPECT_TRUE(test, pmp_floor_word_of(all, 10, &w));
	w = 1;
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(zero, 4, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(twice, 4, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(low, 2, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(high, 2, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(wide, 2, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(j613, 3, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(j613, 0, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(all, 12, &w));
	KUNIT_EXPECT_FALSE(test, pmp_floor_word_of(NULL, 2, &w));
	KUNIT_EXPECT_EQ(test, w, 1ULL);	/* untouched on refusal */
}

static void floor_words(struct kunit *test)
{
	unsigned long long j613 = PMP_FLOOR_VALID | LANE(0x21, 2) | LANE(0x25, 4);

	KUNIT_EXPECT_TRUE(test, pmp_floor_initial_ok(0));
	KUNIT_EXPECT_TRUE(test, pmp_floor_initial_ok(PMP_FLOOR_VALID));
	KUNIT_EXPECT_FALSE(test, pmp_floor_initial_ok(j613));
	/* Measured J613 responses: 0x3224 held, 0x3204 part-way, 0x3200 released. */
	KUNIT_EXPECT_TRUE(test, pmp_floor_raise_confirmed(0x3224, j613));
	KUNIT_EXPECT_FALSE(test, pmp_floor_raise_confirmed(0x3204, j613));
	KUNIT_EXPECT_FALSE(test, pmp_floor_raise_confirmed(0x3200, j613));
	KUNIT_EXPECT_TRUE(test, pmp_floor_raise_confirmed(0x3204, PMP_FLOOR_VALID | LANE(0x25, 4)));
	KUNIT_EXPECT_TRUE(test, pmp_floor_drop_confirmed(0x3200, 0x3204));
	/* J613 ADT: SOC-DEV-DVFS at PTD 0x110, DVFS-STATE at 8. */
	{
		struct pmp_floor_layout l = { .cpu_first = 0x110, .state_first = 8 };

		KUNIT_EXPECT_EQ(test, pmp_floor_req_update(&l), 0x10888ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_req_read(&l), 0x1110ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_state_read(&l, 0), 0x80ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_layout_end(&l), 0x10890ULL);
	}
	/* T6000 ADT: SOC-DEV-DVFS at PTD 0x108 (EACC0, PACC0, PACC1, ...), DVFS-STATE at 8. */
	{
		struct pmp_floor_layout l = { .cpu_first = 0x108, .state_first = 8 };

		KUNIT_EXPECT_EQ(test, pmp_floor_req_update(&l), 0x10848ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_req_read(&l), 0x1090ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_state_read(&l, 0), 0x80ULL);
		KUNIT_EXPECT_EQ(test, pmp_floor_layout_end(&l), 0x10850ULL);
	}
}

/* Records named EACC0 and PACC0 (and PACC1), and the two ranges. */
static void floor_tables(struct kunit *test)
{
	static unsigned char dev[10 * PMP_FLOOR_REC_SIZE], rng[2 * PMP_FLOOR_RANGE_SIZE];
	struct pmp_floor_layout l = {};

	memset(dev, 0, sizeof(dev));
	memset(rng, 0, sizeof(rng));
	memcpy(dev + 6 * PMP_FLOOR_REC_SIZE + PMP_FLOOR_REC_NAME, "EACC0", 5);
	memcpy(dev + 7 * PMP_FLOOR_REC_SIZE + PMP_FLOOR_REC_NAME, "PACC0", 5);
	rng[0] = PMP_FLOOR_STATE_RANGE;
	rng[PMP_FLOOR_RANGE_BASE] = 0x8;
	rng[PMP_FLOOR_RANGE_COUNT] = 4;
	memcpy(rng + PMP_FLOOR_RANGE_NAME, "DVFS-STATE", 10);
	rng[PMP_FLOOR_RANGE_SIZE] = PMP_FLOOR_CPU_RANGE;
	rng[PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_BASE] = 0x08;
	rng[PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_BASE + 1] = 0x01;
	rng[PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_COUNT] = 0x28;
	memcpy(rng + PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_NAME, "SOC-DEV-DVFS", 12);

	/* J613-like: two CPU clients; the update aliases end after PACC0's. */
	KUNIT_EXPECT_NULL(test, pmp_floor_tables_check(dev, sizeof(dev), rng, sizeof(rng), &l));
	KUNIT_EXPECT_EQ(test, l.cpu_first, 0x108U);
	KUNIT_EXPECT_EQ(test, l.state_first, 0x8U);
	KUNIT_EXPECT_EQ(test, l.clients, 2U);
	KUNIT_EXPECT_EQ(test, pmp_floor_layout_end(&l), 0x10850ULL);
	/* T6000: PACC1 named too, three clients, aliases through the third entry. */
	memcpy(dev + 8 * PMP_FLOOR_REC_SIZE + PMP_FLOOR_REC_NAME, "PACC1", 5);
	KUNIT_EXPECT_NULL(test, pmp_floor_tables_check(dev, sizeof(dev), rng, sizeof(rng), &l));
	KUNIT_EXPECT_EQ(test, l.clients, 3U);
	KUNIT_EXPECT_EQ(test, pmp_floor_layout_end(&l), 0x10858ULL);
	/* A range too short for the three clients is refused. */
	rng[PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_COUNT] = 2;
	KUNIT_EXPECT_NOT_NULL(test, pmp_floor_tables_check(dev, sizeof(dev), rng, sizeof(rng), &l));
	rng[PMP_FLOOR_RANGE_SIZE + PMP_FLOOR_RANGE_COUNT] = 0x28;
	memcpy(dev + 9 * PMP_FLOOR_REC_SIZE + PMP_FLOOR_REC_NAME, "PACC0", 5);
	KUNIT_EXPECT_NOT_NULL(test, pmp_floor_tables_check(dev, sizeof(dev), rng, sizeof(rng), &l));
}

static void floor_busy(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, pmp_floor_cpu_pct(100, 1000, 106, 1030), 80U);
	KUNIT_EXPECT_EQ(test, pmp_floor_cpu_pct(100, 1000, 130, 1030), 0U);
	KUNIT_EXPECT_EQ(test, pmp_floor_cpu_pct(100, 1000, 90, 1030), 0U);
	KUNIT_EXPECT_EQ(test, pmp_floor_busy_pct(20, 10), 100U);
	KUNIT_EXPECT_TRUE(test, pmp_floor_cpu_demand(&defaults, 50, 95));
	KUNIT_EXPECT_FALSE(test, pmp_floor_cpu_demand(&defaults, 20, 80));
}

static void floor_raise_hold_drop(struct kunit *test)
{
	struct pmp_floor_gov g = {};
	unsigned long long t = 30 * MS;
	unsigned int i;

	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &defaults, t, true), PMP_FLOOR_KEEP);
	t += 30 * MS;
	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &defaults, t, true), PMP_FLOOR_RAISE);
	for (i = 0; i < 16; i++) {
		t += 30 * MS;
		KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &defaults, t, false), PMP_FLOOR_KEEP);
	}
	t += 30 * MS;
	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &defaults, t, false), PMP_FLOOR_DROP);
	KUNIT_EXPECT_FALSE(test, g.raised);
}

static void floor_rate_limit(struct kunit *test)
{
	struct pmp_floor_gov_cfg c = defaults;
	struct pmp_floor_gov g = {};
	unsigned long long t = 0;

	c.raise_samples = 1;
	c.hold_ms = 100;
	c.min_interval_ms = 300;
	t += 10 * MS;
	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &c, t, true), PMP_FLOOR_RAISE);
	t += 200 * MS;	/* hold passed, interval not */
	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &c, t, false), PMP_FLOOR_KEEP);
	KUNIT_EXPECT_EQ(test, g.rate_limited, 1ULL);
	t += 100 * MS;
	KUNIT_EXPECT_EQ(test, pmp_floor_gov_step(&g, &c, t, false), PMP_FLOOR_DROP);
	pmp_floor_gov_reset(&g, t);
	KUNIT_EXPECT_FALSE(test, g.raised);
}

static void floor_shared_release(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
		pmp_floor_drop_timeout(2000 * MS, 1000 * MS, 0, false, 1000));
	KUNIT_EXPECT_FALSE(test,
		pmp_floor_drop_timeout(9000 * MS, 1000 * MS, 1500 * MS, true, 1000));
	KUNIT_EXPECT_FALSE(test,
		pmp_floor_drop_timeout(2500 * MS, 1000 * MS, 2000 * MS, false, 1000));
	KUNIT_EXPECT_TRUE(test,
		pmp_floor_drop_timeout(3000 * MS, 1000 * MS, 2000 * MS, false, 1000));
	KUNIT_EXPECT_TRUE(test,
		pmp_floor_drop_timeout(2000 * MS, 1000 * MS, 500 * MS, false, 1000));
	KUNIT_EXPECT_FALSE(test,
		pmp_floor_drop_timeout(2000 * MS, 1000 * MS, 2001 * MS, false, 1000));
}

static struct kunit_case pmp_floor_cases[] = {
	KUNIT_CASE(floor_levels),
	KUNIT_CASE(floor_shared_release),
	KUNIT_CASE(floor_words),
	KUNIT_CASE(floor_tables),
	KUNIT_CASE(floor_busy),
	KUNIT_CASE(floor_raise_hold_drop),
	KUNIT_CASE(floor_rate_limit),
	{}
};

static struct kunit_suite pmp_floor_suite = {
	.name = "pmp-floor-gov",
	.test_cases = pmp_floor_cases,
};
kunit_test_suite(pmp_floor_suite);

MODULE_DESCRIPTION("KUnit tests for the PMP CPU memory-floor governor");
MODULE_LICENSE("Dual MIT/GPL");
