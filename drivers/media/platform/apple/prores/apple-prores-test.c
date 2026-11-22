// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the Apple ProRes descriptor and table builders.
 *
 * Expected register words are the values the engine is known to run with
 * (m1n1 proxyclient experiments/prores.py, MIT).
 */
#include <kunit/test.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "apple-prores.h"

static void apr_test_qmat_regs(struct kunit *test)
{
	static const struct {
		enum apr_qtable table;
		bool chroma;
		unsigned int index;
		u32 value;
	} cases[] = {
		{ APR_QTABLE_XQ, false, 0, 0x802802 },
		{ APR_QTABLE_XQ, false, 15, 0x803802 },
		{ APR_QTABLE_XQ, false, 27, 0x804803 },
		{ APR_QTABLE_XQ, true, 23, 0x806805 },
		{ APR_QTABLE_XQ, true, 31, 0x807807 },
		{ APR_QTABLE_HQ, false, 0, 0x804804 },
		{ APR_QTABLE_HQ, true, 27, 0x807806 },
		{ APR_QTABLE_STANDARD, false, 2, 0x807806 },
		{ APR_QTABLE_STANDARD, false, 23, 0x80f80c },
		{ APR_QTABLE_STANDARD, true, 31, 0x815811 },
		{ APR_QTABLE_LT, false, 0, 0x805804 },
		{ APR_QTABLE_LT, false, 27, 0x82381c },
		{ APR_QTABLE_LT, true, 31, 0x829823 },
		{ APR_QTABLE_PROXY, false, 0, 0x807804 },
		{ APR_QTABLE_PROXY, false, 3, 0xfff83f },
		{ APR_QTABLE_PROXY, false, 6, 0x83f80e },
		{ APR_QTABLE_PROXY, true, 17, 0xfff80e },
		{ APR_QTABLE_PROXY, true, 24, 0xfff80d },
		{ APR_QTABLE_PROXY, true, 31, 0xffffff },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		const u8 *qm = cases[i].chroma ? apr_qmat_chroma[cases[i].table] :
						 apr_qmat_luma[cases[i].table];

		KUNIT_EXPECT_EQ_MSG(test, apr_qmat_reg(qm, cases[i].index),
				    cases[i].value, "case %u", i);
	}
	KUNIT_EXPECT_EQ(test, apr_qmat_reg(apr_qmat_luma[0], 32), 0);
}

static void apr_test_dc_scale(struct kunit *test)
{
	static const struct { unsigned int index; u32 value; } cases[] = {
		{ 0, 0x401 }, { 1, 0x803 }, { 31, 0x803f }, { 62, 0xfc7d },
		{ 63, 0x1007f }, { 64, 0x11084 }, { 79, 0x200fc },
		{ 110, 0x3f1f4 }, { 111, 0x1fc },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++)
		KUNIT_EXPECT_EQ(test, apr_dc_scale_reg(cases[i].index), cases[i].value);
	KUNIT_EXPECT_EQ(test, apr_dc_scale_reg(APR_DC_SCALE_ENTRIES), 0);
}

static void apr_test_profiles(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_PROXY), APR_QTABLE_PROXY);
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_LT), APR_QTABLE_LT);
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_STANDARD), APR_QTABLE_STANDARD);
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_HQ), APR_QTABLE_HQ);
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_4444), APR_QTABLE_HQ);
	KUNIT_EXPECT_EQ(test, apr_profile_qtable(APR_PROFILE_4444_XQ), APR_QTABLE_XQ);
	KUNIT_EXPECT_TRUE(test, apr_profile_is_444(APR_PROFILE_4444_XQ));
	KUNIT_EXPECT_FALSE(test, apr_profile_is_444(APR_PROFILE_HQ));
}

static struct apr_frame apr_test_frame(void)
{
	return (struct apr_frame){
		.width = 320, .height = 240,
		.input = APR_INPUT_422_16, .profile = APR_PROFILE_STANDARD,
		.luma_stride = 640, .chroma_stride = 640,
		.luma = 0x10000100000ULL, .chroma = 0x10000200000ULL,
		.output = 0x10000400000ULL, .output_size = SZ_1M,
		.primaries = 1, .transfer = 1, .matrix = 1,
	};
}

static void apr_test_desc_422(struct kunit *test)
{
	struct apr_frame f = apr_test_frame();
	u8 *d = kunit_kzalloc(test, APR_DESC_BYTES, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, d);
	KUNIT_ASSERT_EQ(test, apr_build_desc(&f, d), 0);
	/* Layout facts from the working T8140 job: flags, strides, header. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d), 0x363b);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x08), f.output);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x10), SZ_1M);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x18), 320);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x1a), 240);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x20), 10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x22), 10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x28), f.luma);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x38), f.chroma);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x48), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x58), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x70), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x7c), 148);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x84), 320);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x86), 240);
	KUNIT_EXPECT_EQ(test, d[0x88], 0x80);
	KUNIT_EXPECT_EQ(test, d[0x89], 1);
	KUNIT_EXPECT_EQ(test, d[0x8b], 1);
	KUNIT_EXPECT_EQ(test, d[0x8f], 0x03);
	/* 20 macroblocks per row: two 8-wide slices and one 4-wide. */
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x78), 45);
	KUNIT_EXPECT_EQ(test, d[0x7a], 0x30);
	KUNIT_EXPECT_EQ(test, d[0x7b], 2);	/* 422 starting quantiser */
	/* 320x240 422: 174762 and 196607 bytes over 300 macroblocks. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x110), 4640);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x112), 5224);
	KUNIT_EXPECT_EQ(test, d[0x90], 0);	/* matrices live in the engine */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x170), 0x23);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x174), 0);
}

static void apr_test_desc_444(struct kunit *test)
{
	struct apr_frame f = apr_test_frame();
	u8 *d = kunit_kzalloc(test, APR_DESC_BYTES, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, d);
	f.input = APR_INPUT_444_8;
	f.profile = APR_PROFILE_4444_XQ;
	f.luma_stride = 320;
	f.chroma_stride = 640;
	KUNIT_ASSERT_EQ(test, apr_build_desc(&f, d), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d), 0x363c);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x20), 5);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x22), 10);
	KUNIT_EXPECT_EQ(test, d[0x7f], 1);
	KUNIT_EXPECT_EQ(test, d[0x88], 0xc0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x170), 0x03);
}

static void apr_test_desc_rejects(struct kunit *test)
{
	u8 *d = kunit_kzalloc(test, APR_DESC_BYTES, GFP_KERNEL);
	struct apr_frame f;

	KUNIT_ASSERT_NOT_NULL(test, d);
	f = apr_test_frame();
	f.profile = APR_PROFILE_4444;		/* 4:2:2 input, 4:4:4 profile */
	KUNIT_EXPECT_EQ(test, apr_build_desc(&f, d), -EINVAL);
	KUNIT_EXPECT_EQ(test, apr_build_desc_v0(&f, d), -EINVAL);
	f = apr_test_frame();
	f.luma_stride = 600;			/* not a multiple of 64 */
	KUNIT_EXPECT_EQ(test, apr_build_desc(&f, d), -EINVAL);
	KUNIT_EXPECT_EQ(test, apr_build_desc_v0(&f, d), -EINVAL);
	f = apr_test_frame();
	f.chroma_stride = 576;			/* shorter than a row */
	KUNIT_EXPECT_EQ(test, apr_build_desc(&f, d), -EINVAL);
	KUNIT_EXPECT_EQ(test, apr_build_desc_v0(&f, d), -EINVAL);
	f = apr_test_frame();
	f.width = 321;
	KUNIT_EXPECT_EQ(test, apr_build_desc(&f, d), -EINVAL);
	KUNIT_EXPECT_EQ(test, apr_build_desc_v0(&f, d), -EINVAL);
	f = apr_test_frame();
	f.output = 0;
	KUNIT_EXPECT_EQ(test, apr_build_desc(&f, d), -EINVAL);
	KUNIT_EXPECT_EQ(test, apr_build_desc_v0(&f, d), -EINVAL);
}

/* Legacy words from MIT m1n1 f3ac615eb137 experiments/prores.py:607-644. */
static void apr_test_desc_v0(struct kunit *test)
{
	static const u32 legacy[] = {
		0xffffffff, 0x15100c0c, 0x80004020, 0x002c8080,
		0x00880080, 0x004e00c5, 0x009000d0, 0x00200122,
		0x00400200, 0x00400200, 0x00400200, 0x00400200,
		0x00400200, 0x00400200, 0x00400200, 0x00400200,
		0x00400200, 0x00400200, 0x00400200, 0x00400200,
		0x00400200, 0x00400200, 0x00400200, 0x00400200, 0x23,
	};
	struct apr_frame f = apr_test_frame();
	u8 *d = kunit_kmalloc(test, APR_DESC_BYTES + 1, GFP_KERNEL);
	unsigned int i;
	u32 budget;

	KUNIT_ASSERT_NOT_NULL(test, d);
	memset(d, 0xa5, APR_DESC_BYTES + 1);
	KUNIT_ASSERT_EQ(test, apr_build_desc_v0(&f, d), 0);
	KUNIT_EXPECT_EQ(test, d[APR_DESC_BYTES], 0xa5);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d), 0x363b);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 4), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 8), f.output);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x10), f.output_size);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x18), 320);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x1a), 240);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x20), 10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(d + 0x22), 10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x28), f.luma);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x38), f.chroma);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x58), 148);
	KUNIT_EXPECT_EQ(test, d[0x5b], 0);
	KUNIT_EXPECT_MEMEQ(test, d + 0x5c, "aurr", 4);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x60), 320);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x62), 240);
	KUNIT_EXPECT_EQ(test, d[0x64], 0x80);
	KUNIT_EXPECT_EQ(test, d[0x65], 0);
	KUNIT_EXPECT_EQ(test, d[0x66], f.primaries);
	KUNIT_EXPECT_EQ(test, d[0x67], f.transfer);
	KUNIT_EXPECT_EQ(test, d[0x68], f.matrix);
	KUNIT_EXPECT_EQ(test, d[0x69], 0);
	KUNIT_EXPECT_EQ(test, d[0x6a], 0);
	KUNIT_EXPECT_EQ(test, d[0x6b], 3);
	for (i = 0x6c; i < 0xee; i++)
		KUNIT_EXPECT_EQ_MSG(test, d[i], 0, "padding/slices %#x", i);
	KUNIT_EXPECT_EQ(test, d[0xee], 0x30);
	KUNIT_EXPECT_EQ(test, d[0xef], 2);
	/* Word 0 is the rate budget, checked against v1 below. */
	for (i = 1; i < ARRAY_SIZE(legacy); i++)
		KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le32(d + 0xf0 + 4 * i),
				    legacy[i], "legacy word %u", i);
	budget = get_unaligned_le32(d + 0xf0);
	for (i = 0x154; i < APR_DESC_BYTES; i++)
		KUNIT_EXPECT_EQ_MSG(test, d[i], 0, "v0 trailing padding %#x", i);

	/* Reusing the buffer for v1 must clear the old frame header. */
	KUNIT_ASSERT_EQ(test, apr_build_desc(&f, d), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x58), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_be16(d + 0x7c), 148);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x170), 0x23);
	/* v0 carries v1's target and maximum budget words. */
	KUNIT_EXPECT_EQ(test, budget, get_unaligned_le32(d + 0x110));
	KUNIT_EXPECT_NE(test, budget, 0xffffffff);
}

static void apr_test_desc_v0_profiles(struct kunit *test)
{
	struct apr_frame f = apr_test_frame();
	u8 *d = kunit_kmalloc(test, APR_DESC_BYTES, GFP_KERNEL);
	unsigned int profile;

	KUNIT_ASSERT_NOT_NULL(test, d);
	for (profile = 0; profile < APR_PROFILES; profile++) {
		bool is444 = apr_profile_is_444(profile);

		f.profile = profile;
		f.input = is444 ? APR_INPUT_444_8 : APR_INPUT_422_16;
		KUNIT_ASSERT_EQ(test, apr_build_desc_v0(&f, d), 0);
		KUNIT_EXPECT_EQ(test, d[0x5b], is444 ? 1 : 0);
		KUNIT_EXPECT_EQ(test, d[0x64], is444 ? 0xc0 : 0x80);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x150),
				apr_profile_qtable(profile) << 4 | 3);
		/* The v1 extension is never written into a v0 descriptor. */
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x170), 0);
	}
}

static void apr_test_budgets(struct kunit *test)
{
	/* Target, maximum and the Q3 per-macroblock words for 1080p and 2160p. */
	static const struct {
		enum apr_profile profile;
		u32 w, h, target, max;
		u16 q_target, q_max;
	} cases[] = {
		{ APR_PROFILE_PROXY, 1920, 1080, 189326, 212991, 176, 200 },
		{ APR_PROFILE_LT, 1920, 1080, 425984, 479232, 408, 456 },
		{ APR_PROFILE_STANDARD, 1920, 1080, 611669, 688127, 584, 664 },
		{ APR_PROFILE_HQ, 1920, 1080, 917504, 1032192, 888, 1000 },
		{ APR_PROFILE_4444, 1920, 1080, 1376256, 1548288, 1336, 1504 },
		{ APR_PROFILE_4444_XQ, 1920, 1080, 2064384, 2322432, 2008, 2264 },
		{ APR_PROFILE_PROXY, 3840, 2160, 757304, 851967, 176, 200 },
		{ APR_PROFILE_LT, 3840, 2160, 1711217, 1925119, 408, 464 },
		{ APR_PROFILE_STANDARD, 3840, 2160, 2457600, 2764800, 592, 672 },
		{ APR_PROFILE_HQ, 3840, 2160, 3684579, 4145151, 896, 1008 },
		{ APR_PROFILE_4444, 3840, 2160, 5526869, 6217727, 1352, 1520 },
		{ APR_PROFILE_4444_XQ, 3840, 2160, 8293944, 9330687, 2032, 2288 },
	};
	u8 *d = kunit_kzalloc(test, APR_DESC_BYTES, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, d);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct apr_frame f = apr_test_frame();
		bool is444 = apr_profile_is_444(cases[i].profile);
		u32 target, max;

		apr_frame_budget(cases[i].profile, cases[i].w, cases[i].h, &target, &max);
		KUNIT_EXPECT_EQ_MSG(test, target, cases[i].target, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, max, cases[i].max, "case %u", i);
		f.profile = cases[i].profile;
		f.input = is444 ? APR_INPUT_444_8 : APR_INPUT_422_8;
		f.width = cases[i].w;
		f.height = cases[i].h;
		f.luma_stride = cases[i].w;
		f.chroma_stride = cases[i].w * (is444 ? 2 : 1);
		KUNIT_ASSERT_EQ(test, apr_build_desc(&f, d), 0);
		KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le16(d + 0x110), cases[i].q_target,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le16(d + 0x112), cases[i].q_max,
				    "case %u", i);
	}
}

static void apr_test_slices(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, apr_slice_count(320, 240, 3), 45);
	/* 1920x1080: 120 MBs per row = 15 slices, 68 rows. */
	KUNIT_EXPECT_EQ(test, apr_slice_count(1920, 1080, 3), 15 * 68);
	/* 1366 wide: 86 MBs = 10 x 8 + 4 + 2 -> 12 slices per row. */
	KUNIT_EXPECT_EQ(test, apr_slice_count(1366, 16, 3), 12);
}

static void apr_test_output(struct kunit *test)
{
	u8 *out = kunit_kzalloc(test, SZ_4K, GFP_KERNEL);
	u32 bytes = 0;

	KUNIT_ASSERT_NOT_NULL(test, out);
	put_unaligned_be32(2000, out);
	memcpy(out + 4, "icpf", 4);
	KUNIT_EXPECT_EQ(test, apr_check_output(out, SZ_4K, &bytes), 0);
	KUNIT_EXPECT_EQ(test, bytes, 2000);
	KUNIT_EXPECT_EQ(test, apr_check_output(out, 1999, &bytes), -EIO);
	put_unaligned_be32(100, out);
	KUNIT_EXPECT_EQ(test, apr_check_output(out, SZ_4K, &bytes), -EIO);
	put_unaligned_be32(2000, out);
	out[4] = 'x';
	KUNIT_EXPECT_EQ(test, apr_check_output(out, SZ_4K, &bytes), -EIO);
}

static const size_t apri_test_block_headers[APRI_T6041_BLOCKS] = {
	[APRI_SYSTEM_BLOCK] = 16,
	[APRI_CORE_BLOCK] = 16 + 8 + APRI_SYSTEM_COUNT * 12,
	[APRI_WRAPPER_BLOCK] = 16 + 8 + APRI_SYSTEM_COUNT * 12 +
			       8 + APRI_CORE_COUNT * 12,
};

static const size_t apri_test_entry_bases[APRI_T6041_BLOCKS] = {
	[APRI_SYSTEM_BLOCK] = 16 + 8,
	[APRI_CORE_BLOCK] = 16 + 8 + APRI_SYSTEM_COUNT * 12 + 8,
	[APRI_WRAPPER_BLOCK] = 16 + 8 + APRI_SYSTEM_COUNT * 12 +
			     8 + APRI_CORE_COUNT * 12 + 8,
};

static void apri_test_build_fixture(u8 *data)
{
	static const u32 counts[APRI_T6041_BLOCKS] = {
		[APRI_SYSTEM_BLOCK] = APRI_SYSTEM_COUNT,
		[APRI_CORE_BLOCK] = APRI_CORE_COUNT,
		[APRI_WRAPPER_BLOCK] = APRI_WRAPPER_COUNT,
	};
	u32 block, entry;

	memset(data, 0, APRI_T6041_FIRMWARE_SIZE);
	memcpy(data, "APRI", 4);
	put_unaligned_le32(1, data + 4);
	put_unaligned_le32(0x6041, data + 8);
	put_unaligned_le32(APRI_T6041_BLOCKS, data + 12);
	for (block = 0; block < APRI_T6041_BLOCKS; block++) {
		put_unaligned_le32(block + 1, data + apri_test_block_headers[block]);
		put_unaligned_le32(counts[block], data + apri_test_block_headers[block] + 4);
		for (entry = 0; entry < counts[block]; entry++) {
			size_t pos = apri_test_entry_bases[block] + entry * 12;

			put_unaligned_le32(entry * 4, data + pos);
			put_unaligned_le32(0xff, data + pos + 4);
			put_unaligned_le32(entry & 0xff, data + pos + 8);
		}
	}
}

static void apr_test_apri_valid(struct kunit *test)
{
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kmalloc(test, APRI_T6041_FIRMWARE_SIZE, GFP_KERNEL);
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	apri_test_build_fixture(data);
	ret = apr_parse_t6041_init(data, APRI_T6041_FIRMWARE_SIZE, init);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_SYSTEM_BLOCK].count,
			(u32)APRI_SYSTEM_COUNT);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_CORE_BLOCK].count,
			(u32)APRI_CORE_COUNT);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_WRAPPER_BLOCK].count,
			(u32)APRI_WRAPPER_COUNT);
	KUNIT_EXPECT_EQ(test,
			init->blocks[APRI_WRAPPER_BLOCK].entries[61].offset,
			(u32)61 * 4);
}

/* A T6050 container: T6041's three blocks, then the control block. */
static void apr_test_apri_t6050(struct kunit *test)
{
	static const u32 counts[APRI_T6050_BLOCKS] = {
		APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_WRAPPER_COUNT, APRI_CONTROL_COUNT,
	};
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kzalloc(test, APRI_T6050_FIRMWARE_SIZE, GFP_KERNEL);
	size_t pos = 16;
	u32 block, entry;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	memcpy(data, "APRI", 4);
	put_unaligned_le32(1, data + 4);
	put_unaligned_le32(0x6050, data + 8);
	put_unaligned_le32(APRI_T6050_BLOCKS, data + 12);
	for (block = 0; block < APRI_T6050_BLOCKS; block++) {
		put_unaligned_le32(block + 1, data + pos);
		put_unaligned_le32(counts[block], data + pos + 4);
		pos += 8;
		for (entry = 0; entry < counts[block]; entry++, pos += 12) {
			put_unaligned_le32(entry * 4, data + pos);
			put_unaligned_le32(0xffffffff, data + pos + 4);
			put_unaligned_le32(entry, data + pos + 8);
		}
	}
	KUNIT_ASSERT_EQ(test, pos, (size_t)APRI_T6050_FIRMWARE_SIZE);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6050, data, pos, init), 0);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_CONTROL_BLOCK].count,
			(u32)APRI_CONTROL_COUNT);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_CONTROL_BLOCK].entries[11].offset,
			(u32)11 * 4);
	/* Neither SoC takes the other's container. */
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6041, data, pos, init), -EINVAL);
	put_unaligned_le32(0x6041, data + 8);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6050, data, pos, init), -EINVAL);
	put_unaligned_le32(0x6050, data + 8);
	/* A control row past its window. */
	put_unaligned_le32(0x4000, data + pos - 12);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6050, data, pos, init), -EINVAL);
}

/* A T8132 container: T6041's blocks with a 48-row wrapper table. */
static void apr_test_apri_t8132(struct kunit *test)
{
	static const u32 counts[APRI_T6041_BLOCKS] = {
		APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_T8132_WRAPPER_COUNT,
	};
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kzalloc(test, APRI_T8132_FIRMWARE_SIZE, GFP_KERNEL);
	size_t pos = 16;
	u32 block, entry;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	memcpy(data, "APRI", 4);
	put_unaligned_le32(1, data + 4);
	put_unaligned_le32(0x8132, data + 8);
	put_unaligned_le32(APRI_T6041_BLOCKS, data + 12);
	for (block = 0; block < APRI_T6041_BLOCKS; block++) {
		put_unaligned_le32(block + 1, data + pos);
		put_unaligned_le32(counts[block], data + pos + 4);
		pos += 8;
		for (entry = 0; entry < counts[block]; entry++, pos += 12) {
			put_unaligned_le32(entry * 4, data + pos);
			put_unaligned_le32(0xffffffff, data + pos + 4);
			put_unaligned_le32(entry, data + pos + 8);
		}
	}
	KUNIT_ASSERT_EQ(test, pos, (size_t)APRI_T8132_FIRMWARE_SIZE);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8132, data, pos, init), 0);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_WRAPPER_BLOCK].count,
			(u32)APRI_T8132_WRAPPER_COUNT);
	/* T6041 does not take it, nor T8132 a T6041 SoC field. */
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6041, data, pos, init), -EINVAL);
	put_unaligned_le32(0x6041, data + 8);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8132, data, pos, init), -EINVAL);
	put_unaligned_le32(0x8132, data + 8);
	/* A wrapper row past its window. */
	put_unaligned_le32(0x1000000, data + pos - 12);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8132, data, pos, init), -EINVAL);
}

/* A T8142 container: T6050's blocks with a 38-row wrapper. */
static void apr_test_apri_t8142(struct kunit *test)
{
	static const u32 counts[APRI_T8142_BLOCKS] = {
		APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_T8142_WRAPPER_COUNT,
		APRI_CONTROL_COUNT,
	};
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kzalloc(test, APRI_T8142_FIRMWARE_SIZE, GFP_KERNEL);
	size_t pos = 16;
	u32 block, entry;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	memcpy(data, "APRI", 4);
	put_unaligned_le32(1, data + 4);
	put_unaligned_le32(0x8142, data + 8);
	put_unaligned_le32(APRI_T8142_BLOCKS, data + 12);
	for (block = 0; block < APRI_T8142_BLOCKS; block++) {
		put_unaligned_le32(block + 1, data + pos);
		put_unaligned_le32(counts[block], data + pos + 4);
		pos += 8;
		for (entry = 0; entry < counts[block]; entry++, pos += 12) {
			put_unaligned_le32(entry * 4, data + pos);
			put_unaligned_le32(0xffffffff, data + pos + 4);
			put_unaligned_le32(entry, data + pos + 8);
		}
	}
	KUNIT_ASSERT_EQ(test, pos, (size_t)APRI_T8142_FIRMWARE_SIZE);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8142, data, pos, init), 0);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_CONTROL_BLOCK].count,
			(u32)APRI_CONTROL_COUNT);
	KUNIT_EXPECT_EQ(test, init->blocks[APRI_CONTROL_BLOCK].entries[11].offset,
			(u32)11 * 4);
	/* T6050 refuses it by size, T8142 refuses T6050's SoC id. */
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t6050, data, pos, init), -EINVAL);
	put_unaligned_le32(0x6050, data + 8);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8142, data, pos, init), -EINVAL);
	put_unaligned_le32(0x8142, data + 8);
	/* A control row past its window. */
	put_unaligned_le32(0x4000, data + pos - 12);
	KUNIT_EXPECT_EQ(test, apr_parse_init(&apri_t8142, data, pos, init), -EINVAL);
}

static void apri_test_expect_rejected(struct kunit *test, const u8 *data,
				      size_t size, struct apr_init_data *init)
{
	KUNIT_EXPECT_EQ(test, apr_parse_t6041_init(data, size, init), -EINVAL);
}

static void apr_test_apri_header_rejects(struct kunit *test)
{
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kmalloc(test, APRI_T6041_FIRMWARE_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	apri_test_build_fixture(data);
	data[0] = 'X';
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	put_unaligned_le32(2, data + 4);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	put_unaligned_le32(0x6040, data + 8);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	put_unaligned_le32(4, data + 12);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE - 1, init);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE + 1, init);
}

static void apr_test_apri_shape_rejects(struct kunit *test)
{
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kmalloc(test, APRI_T6041_FIRMWARE_SIZE, GFP_KERNEL);
	u8 *core_count;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	apri_test_build_fixture(data);
	put_unaligned_le32(1, data + apri_test_block_headers[APRI_CORE_BLOCK]);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	put_unaligned_le32(4, data + apri_test_block_headers[APRI_CORE_BLOCK]);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
	apri_test_build_fixture(data);
	core_count = data + apri_test_block_headers[APRI_CORE_BLOCK] + 4;
	put_unaligned_le32(APRI_CORE_COUNT - 1, core_count);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
}

static void apr_test_apri_rows_reject(struct kunit *test, u32 block,
				      u32 entry, u32 field, u32 value)
{
	struct apr_init_data *init = kunit_kzalloc(test, sizeof(*init), GFP_KERNEL);
	u8 *data = kunit_kmalloc(test, APRI_T6041_FIRMWARE_SIZE, GFP_KERNEL);
	size_t pos;

	KUNIT_ASSERT_NOT_NULL(test, init);
	KUNIT_ASSERT_NOT_NULL(test, data);
	apri_test_build_fixture(data);
	pos = apri_test_entry_bases[block] + entry * 12 + field;
	put_unaligned_le32(value, data + pos);
	apri_test_expect_rejected(test, data, APRI_T6041_FIRMWARE_SIZE, init);
}

static void apr_test_apri_row_rejects(struct kunit *test)
{
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 0, 0, 2);
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 1, 0, 0);
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 0, 0, 0x4000);
	apr_test_apri_rows_reject(test, APRI_CORE_BLOCK, 0, 0, 0x4000);
	apr_test_apri_rows_reject(test, APRI_WRAPPER_BLOCK, 0, 0, 0x1000000);
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 0, 4, 0);
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 2, 4, 1);
	/* A value bit outside its mask. */
	apr_test_apri_rows_reject(test, APRI_SYSTEM_BLOCK, 0, 8, 0x100);
}

static struct kunit_case apr_test_cases[] = {
	KUNIT_CASE(apr_test_qmat_regs),
	KUNIT_CASE(apr_test_dc_scale),
	KUNIT_CASE(apr_test_profiles),
	KUNIT_CASE(apr_test_desc_422),
	KUNIT_CASE(apr_test_desc_444),
	KUNIT_CASE(apr_test_desc_rejects),
	KUNIT_CASE(apr_test_desc_v0),
	KUNIT_CASE(apr_test_desc_v0_profiles),
	KUNIT_CASE(apr_test_budgets),
	KUNIT_CASE(apr_test_slices),
	KUNIT_CASE(apr_test_output),
	KUNIT_CASE(apr_test_apri_valid),
	KUNIT_CASE(apr_test_apri_t6050),
	KUNIT_CASE(apr_test_apri_t8132),
	KUNIT_CASE(apr_test_apri_t8142),
	KUNIT_CASE(apr_test_apri_header_rejects),
	KUNIT_CASE(apr_test_apri_shape_rejects),
	KUNIT_CASE(apr_test_apri_row_rejects),
	{}
};

static struct kunit_suite apr_test_suite = {
	.name = "apple-prores",
	.test_cases = apr_test_cases,
};
kunit_test_suite(apr_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple ProRes encoder");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
