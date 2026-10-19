// SPDX-License-Identifier: GPL-2.0-only OR MIT

#include <kunit/test.h>

#include "pmp-report-validation.h"

static void pmp_range(u8 *table, u32 id, u32 base, u32 count)
{
	memset(table, 0, PMP_PTD_RANGE_SIZE);
	put_unaligned_le32(id, table);
	put_unaligned_le32(base, table + 4);
	put_unaligned_le32(count, table + 8);
}

static void pmp_ranges_t6030(struct kunit *test)
{
	u8 table[2 * PMP_PTD_RANGE_SIZE];

	pmp_range(table, PMP_PTD_RANGE_REQUEST, 0x118, 4);
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_ACK, 0x11c, 4);
	KUNIT_EXPECT_TRUE(test, apple_pmp_ranges_valid(table, sizeof(table),
						       0x1180, 0x108c0, 0x11c0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1000, 0x10800, 0x1080));
}

static void pmp_ranges_t8122(struct kunit *test)
{
	u8 table[2 * PMP_PTD_RANGE_SIZE];

	pmp_range(table, PMP_PTD_RANGE_ACK, 0x108, 4);
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_REQUEST, 0x100, 4);
	KUNIT_EXPECT_TRUE(test, apple_pmp_ranges_valid(table, sizeof(table),
						       0x1000, 0x10800, 0x1080));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1180, 0x108c0, 0x11c0));
}

/* T6031 (J516C ADT): SOC-DEV-PS-REQ at 0x200, SOC-DEV-PS-ACK at 0x208. */
static void pmp_ranges_t6031(struct kunit *test)
{
	u8 table[3 * PMP_PTD_RANGE_SIZE];

	pmp_range(table, 1, 0x1, 1);
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_REQUEST, 0x200, 8);
	pmp_range(table + 2 * PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_ACK, 0x208, 8);
	KUNIT_EXPECT_TRUE(test, apple_pmp_ranges_valid(table, sizeof(table),
						       0x2000, 0x11000, 0x2080));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1180, 0x108c0, 0x11c0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1000, 0x10800, 0x1080));

	/* A T6030 table is refused with the T6031 apertures. */
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_REQUEST, 0x118, 4);
	pmp_range(table + 2 * PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_ACK, 0x11c, 4);
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x2000, 0x11000, 0x2080));
}

/*
 * T6031: 35 soc-device records in id order. DISPINT (id 17) and DISPEXT0-3
 * (ids 18-21) are acknowledged, ANS (id 34) is not. The T6031 overlay seeds
 * bits 0x10 (DISPINT, acknowledged) and 0x21 (ANS).
 */
static void pmp_devices_t6031(struct kunit *test)
{
	const size_t len = 35 * PMP_SOC_DEVICE_SIZE;
	u64 seed = BIT_ULL(0x10) | BIT_ULL(0x21);
	u64 ack = BIT_ULL(0x10);
	u8 *table = kunit_kzalloc(test, len, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, table);
	for (i = 0; i < 35; i++) {
		put_unaligned_le32(i + 1, table + i * PMP_SOC_DEVICE_SIZE);
		if (i + 1 >= 17 && i + 1 <= 21)
			put_unaligned_le32(PMP_SOC_DEVICE_ACK,
					   table + i * PMP_SOC_DEVICE_SIZE + 8);
	}
	KUNIT_EXPECT_TRUE(test, apple_pmp_devices_valid(table, len, seed, ack, true));
	/* With the external display requests seeded and acknowledged too. */
	KUNIT_EXPECT_TRUE(test, apple_pmp_devices_valid(table, len, seed | GENMASK_ULL(0x14, 0x11),
							ack | GENMASK_ULL(0x14, 0x11), true));
	/* Storage is not acknowledged; the display is. */
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, seed, true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, 0, true));
	/* The T6030 entries (DISP bit 7, ANS bit 16) do not describe this table. */
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, BIT_ULL(7) | BIT_ULL(16),
							 BIT_ULL(7), true));
	/* Out of id order. */
	put_unaligned_le32(18, table + 16 * PMP_SOC_DEVICE_SIZE);
	put_unaligned_le32(17, table + 17 * PMP_SOC_DEVICE_SIZE);
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, ack, true));
}

static void pmp_ranges_reject_bad_records(struct kunit *test)
{
	u8 table[3 * PMP_PTD_RANGE_SIZE];

	pmp_range(table, PMP_PTD_RANGE_REQUEST, 0x118, 4);
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_ACK, 0x11c, 4);
	pmp_range(table + 2 * PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_REQUEST, 0x118, 4);
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1180, 0x108c0, 0x11c0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, PMP_PTD_RANGE_SIZE,
							0x1180, 0x108c0, 0x11c0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, 33, 0x1180, 0x108c0, 0x11c0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(NULL, 0, 0x1180, 0x108c0, 0x11c0));
	pmp_range(table, PMP_PTD_RANGE_REQUEST, 0x118, 0);
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, 2 * PMP_PTD_RANGE_SIZE,
							0x1180, 0x108c0, 0x11c0));
}

/* J516S ADT: DVFS-STATE (id 3) at 8, 4 entries; SOC-DEV-DVFS (id 12) at 288, 16. */
static void pmp_range_named(u8 *table, u32 id, u32 base, u32 count, const char *name)
{
	pmp_range(table, id, base, count);
	strscpy((char *)table + PMP_PTD_RANGE_NAME, name, PMP_PTD_RANGE_NAME_LEN);
}

static void pmp_ranges_find_floor_entries(struct kunit *test)
{
	u8 table[3 * PMP_PTD_RANGE_SIZE];
	u32 base = 0, count = 0;

	pmp_range_named(table, PMP_PTD_RANGE_DVFS_STATE, 8, 4, "DVFS-STATE");
	pmp_range_named(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_REQUEST, 0x118, 4,
			"SOC-DEV-PS-REQ");
	pmp_range_named(table + 2 * PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_SOC_DEV_DVFS, 288, 16,
			"SOC-DEV-DVFS");
	KUNIT_EXPECT_TRUE(test, apple_pmp_range_find(table, sizeof(table),
						     PMP_PTD_RANGE_SOC_DEV_DVFS, "SOC-DEV-DVFS",
						     &base, &count));
	KUNIT_EXPECT_EQ(test, base, 288);
	KUNIT_EXPECT_EQ(test, count, 16);
	KUNIT_EXPECT_TRUE(test, apple_pmp_range_find(table, sizeof(table),
						     PMP_PTD_RANGE_DVFS_STATE, "DVFS-STATE",
						     &base, &count));
	KUNIT_EXPECT_EQ(test, base, 8);
	KUNIT_EXPECT_EQ(test, count, 4);
	/* The name must match the id, the range must exist and be unique. */
	KUNIT_EXPECT_FALSE(test, apple_pmp_range_find(table, sizeof(table),
						      PMP_PTD_RANGE_SOC_DEV_DVFS, "DVFS-STATE",
						      &base, &count));
	KUNIT_EXPECT_FALSE(test, apple_pmp_range_find(table, sizeof(table), 13, "SOC-DEV-BWR",
						      &base, &count));
	pmp_range_named(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_SOC_DEV_DVFS, 288, 16,
			"SOC-DEV-DVFS");
	KUNIT_EXPECT_FALSE(test, apple_pmp_range_find(table, sizeof(table),
						      PMP_PTD_RANGE_SOC_DEV_DVFS, "SOC-DEV-DVFS",
						      &base, &count));
	KUNIT_EXPECT_FALSE(test, apple_pmp_range_find(table, 33, PMP_PTD_RANGE_DVFS_STATE,
						      "DVFS-STATE", &base, &count));
}

static void pmp_ranges_reject_wrapped_base(struct kunit *test)
{
	u8 table[2 * PMP_PTD_RANGE_SIZE];

	/* Both request addresses would match after 32-bit multiplication wraps. */
	pmp_range(table, PMP_PTD_RANGE_REQUEST, 0x20000118, 4);
	pmp_range(table + PMP_PTD_RANGE_SIZE, PMP_PTD_RANGE_ACK, 0x1000011c, 4);
	KUNIT_EXPECT_FALSE(test, apple_pmp_ranges_valid(table, sizeof(table),
							0x1180, 0x108c0, 0x11c0));
}

static void pmp_devices_seed_and_ack(struct kunit *test)
{
	const size_t len = 64 * PMP_SOC_DEVICE_SIZE;
	u64 seed = BIT_ULL(7) | BIT_ULL(16) | BIT_ULL(63);
	u64 ack = BIT_ULL(7) | BIT_ULL(63);
	u8 *table = kunit_kzalloc(test, len, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, table);
	for (i = 0; i < 64; i++) {
		put_unaligned_le32(i + 1, table + i * PMP_SOC_DEVICE_SIZE);
		if (ack & BIT_ULL(i))
			put_unaligned_le32(PMP_SOC_DEVICE_ACK,
					   table + i * PMP_SOC_DEVICE_SIZE + 8);
	}
	KUNIT_EXPECT_TRUE(test, apple_pmp_devices_valid(table, len, seed, ack, true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, seed, true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, ack | BIT_ULL(8), true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len - 1, seed, ack, true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, 63 * PMP_SOC_DEVICE_SIZE,
							 seed, ack, true));
	put_unaligned_le32(7, table + 7 * PMP_SOC_DEVICE_SIZE);
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, ack, true));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(NULL, 0, seed, ack, true));
}

static void pmp_args(u8 *args)
{
	memset(args, 0, 24);
	put_unaligned_le32(0x42444944, args);
	put_unaligned_le32(4, args + 4);
	put_unaligned_le32(0x44564944, args + 12);
	put_unaligned_le32(4, args + 16);
}

static void pmp_devices_reordered(struct kunit *test)
{
	static const u32 ids[] = {
		1, 2, 19, 20, 16, 17, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18,
	};
	const size_t len = ARRAY_SIZE(ids) * PMP_SOC_DEVICE_SIZE;
	u64 seed = BIT_ULL(7) | BIT_ULL(8);
	u8 *table = kunit_kzalloc(test, len, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, table);
	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		put_unaligned_le32(ids[i], table + i * PMP_SOC_DEVICE_SIZE);
		if (ids[i] == 8 || ids[i] == 9)
			put_unaligned_le32(0xa, table + i * PMP_SOC_DEVICE_SIZE + 8);
	}
	KUNIT_EXPECT_TRUE(test, apple_pmp_devices_valid(table, len, seed, seed, false));
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, seed, true));
	put_unaligned_le32(8, table);
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, seed, false));
	put_unaligned_le32(1, table);
	put_unaligned_le32(0, table + 11 * PMP_SOC_DEVICE_SIZE);
	KUNIT_EXPECT_FALSE(test, apple_pmp_devices_valid(table, len, seed, seed, false));
}

/* T8122 (J613): PTD request bits are soc-device record indices, not ID - 1. */
static void pmp_records_t8122(struct kunit *test)
{
	static const u32 ids[] = {
		1, 2, 19, 20, 16, 17, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18,
	};
	const size_t len = ARRAY_SIZE(ids) * PMP_SOC_DEVICE_SIZE;
	u64 seed = BIT_ULL(11) | BIT_ULL(12);
	u8 *table = kunit_kzalloc(test, len, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, table);
	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		put_unaligned_le32(ids[i], table + i * PMP_SOC_DEVICE_SIZE);
		if (ids[i] == 8 || ids[i] == 9)
			put_unaligned_le32(0xa, table + i * PMP_SOC_DEVICE_SIZE + 8);
	}
	/* DISP (8), DISPEXT (9), ANS (18) */
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 8), 11);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 9), 12);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 18), 19);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 21), -ENOENT);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 0), -ENOENT);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len - 1, 8), -ENOENT);
	KUNIT_EXPECT_TRUE(test, apple_pmp_records_valid(table, len, seed, seed));
	KUNIT_EXPECT_FALSE(test, apple_pmp_records_valid(table, len, seed, 0));
	KUNIT_EXPECT_FALSE(test, apple_pmp_records_valid(table, len, BIT_ULL(20), 0));
	put_unaligned_le32(8, table + 12 * PMP_SOC_DEVICE_SIZE);
	KUNIT_EXPECT_EQ(test, apple_pmp_record_of_id(table, len, 8), -ENOENT);
}

static void pmp_bootargs_valid_ids(struct kunit *test)
{
	u8 args[32];

	pmp_args(args);
	KUNIT_EXPECT_TRUE(test, apple_pmp_bootargs_valid(args, 24));
	put_unaligned_le32(8, args + 16);
	memset(args + 24, 0, 4);
	KUNIT_EXPECT_TRUE(test, apple_pmp_bootargs_valid(args, 28));
	KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, 12));
	KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(NULL, 0));
}

static void pmp_bootargs_reject_truncation(struct kunit *test)
{
	u8 args[32];
	unsigned int len;

	pmp_args(args);
	memset(args + 24, 0, 8);
	for (len = 1; len < 24; len++)
		KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, len));
	for (len = 25; len < 32; len++)
		KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, len));
	put_unaligned_le32(U32_MAX, args + 16);
	KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, 24));
}

static void pmp_bootargs_reject_integer_width(struct kunit *test)
{
	u8 args[40];

	pmp_args(args);
	put_unaligned_le32(0, args + 16);
	KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, 20));
	put_unaligned_le32(9, args + 16);
	memset(args + 20, 0, 20);
	KUNIT_EXPECT_FALSE(test, apple_pmp_bootargs_valid(args, 29));
}

static struct kunit_case apple_pmp_report_cases[] = {
	KUNIT_CASE(pmp_ranges_t6030),
	KUNIT_CASE(pmp_ranges_t8122),
	KUNIT_CASE(pmp_ranges_t6031),
	KUNIT_CASE(pmp_ranges_reject_bad_records),
	KUNIT_CASE(pmp_ranges_reject_wrapped_base),
	KUNIT_CASE(pmp_ranges_find_floor_entries),
	KUNIT_CASE(pmp_devices_seed_and_ack),
	KUNIT_CASE(pmp_devices_reordered),
	KUNIT_CASE(pmp_devices_t6031),
	KUNIT_CASE(pmp_records_t8122),
	KUNIT_CASE(pmp_bootargs_valid_ids),
	KUNIT_CASE(pmp_bootargs_reject_truncation),
	KUNIT_CASE(pmp_bootargs_reject_integer_width),
	{}
};

static struct kunit_suite apple_pmp_report_suite = {
	.name = "apple-pmp-report",
	.test_cases = apple_pmp_report_cases,
};

kunit_test_suite(apple_pmp_report_suite);
MODULE_LICENSE("Dual MIT/GPL");
