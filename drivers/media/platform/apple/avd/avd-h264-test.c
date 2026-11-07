// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the H.264 slice checks of the Apple video decoder.
 */

#include <kunit/test.h>
#include <linux/sizes.h>

#include "avd.h"

struct avd_slice_data_case {
	const char *desc;
	const u8 *data;
	u32 len;
	u32 header_len;
	bool found;
	u32 off;
};

static const u8 avd_test_slice[] = {
	0x25, 0xb8, 0x00, 0x00, 0x04, 0x01, 0xaa, 0xbb,
};

/* The same slice with an emulation prevention byte in its header. */
static const u8 avd_test_slice_epb[] = {
	0x25, 0xb8, 0x00, 0x00, 0x03, 0x01, 0xaa, 0xbb,
};

static const struct avd_slice_data_case avd_slice_data_cases[] = {
	{ "empty payload", avd_test_slice, 0, 0 },
	{ "one-byte payload", avd_test_slice, 1, 0 },
	{ "two-byte payload", avd_test_slice, 2, 0 },
	{ "one byte of data", avd_test_slice, 3, 2, true, 2 },
	{ "header ends one byte early", avd_test_slice, 8, 7, true, 7 },
	{ "header ends at the end", avd_test_slice, 8, 8 },
	{ "header beyond the end", avd_test_slice, 8, 12 },
	{ "header without emulation prevention", avd_test_slice, 8, 5, true, 5 },
	{ "emulation prevention in the header", avd_test_slice_epb, 8, 5, true, 6 },
};

KUNIT_ARRAY_PARAM_DESC(avd_slice_data, avd_slice_data_cases, desc);

static void avd_slice_data_test(struct kunit *test)
{
	const struct avd_slice_data_case *c = test->param_value;
	u32 off = U32_MAX;

	KUNIT_EXPECT_EQ(test, avd_h264_find_slice_data(c->data, c->len,
						       c->header_len, &off),
			c->found);
	if (c->found)
		KUNIT_EXPECT_EQ(test, off, c->off);
	else
		KUNIT_EXPECT_EQ(test, off, U32_MAX);
}

struct avd_slice_fits_case {
	const char *desc;
	/* the negotiated coded geometry, in macroblocks */
	u16 width_mbs;
	u16 height_map_units;
	/* the SPS control geometry, which does not have to match */
	u16 sps_width_mbs;
	u16 sps_height_map_units;
	bool fields;
	size_t held;
	size_t held_bytes;
	u32 len;
	size_t decoded_size;
	bool fits;
};

static const struct avd_slice_fits_case avd_slice_fits_cases[] = {
	{ "one-macroblock frame, first slice", 1, 1, 1, 1, false,
	  0, 0, 16, 384, true },
	{ "one-macroblock frame, second slice", 1, 1, 1, 1, false,
	  1, PAGE_SIZE, 16, 384 },
	{ "one-macroblock field pair, second slice", 1, 1, 1, 1, true,
	  1, PAGE_SIZE, 16, 384, true },
	{ "one-macroblock field pair, third slice", 1, 1, 1, 1, true,
	  2, 2 * PAGE_SIZE, 16, 384 },
	{ "320x240, last slice", 20, 15, 20, 15, false,
	  299, 299 * PAGE_SIZE, 16, 115200, true },
	{ "320x240, one slice over", 20, 15, 20, 15, false,
	  300, 300 * PAGE_SIZE, 16, 115200 },
	{ "1920x1088, slice 8159 of 8160", 120, 68, 120, 68, false,
	  8159, 8159 * PAGE_SIZE, 1, 3133440, true },
	{ "1920x1088, slice 8160 of 8160", 120, 68, 120, 68, false,
	  8160, 8160 * PAGE_SIZE, 1, 3133440 },
	{ "1920x1088, slice 8160 with no data", 120, 68, 120, 68, false,
	  8160, 0, 1, 3133440 },
	{ "data up to the budget", 120, 68, 120, 68, false, 1,
	  SZ_512M - PAGE_SIZE, PAGE_SIZE, SZ_256M, true },
	{ "data one byte over the budget", 120, 68, 120, 68, false, 1,
	  SZ_512M - PAGE_SIZE, PAGE_SIZE + 1, SZ_256M },
	{ "no decoded size, data up to the page budget", 120, 68, 120, 68, false,
	  1, 8159 * PAGE_SIZE, 1, 0, true },
	{ "no decoded size, data over the page budget", 120, 68, 120, 68, false,
	  1, 8160 * PAGE_SIZE, 1, 0 },
	{ "slice of almost 4 GiB", 120, 68, 120, 68, false, 0, 0, U32_MAX, SZ_256M },
	/* The SPS claims a geometry 16 times the negotiated coded size; the
	 * negotiated geometry sets the bound, not the SPS.
	 */
	{ "64x64 negotiated, SPS 256x256, first slice", 4, 4, 256, 256, false,
	  0, 0, 16, 384, true },
	{ "64x64 negotiated, SPS 256x256, 17th slice", 4, 4, 256, 256, false,
	  16, 16 * PAGE_SIZE, 1, 384 },
};

KUNIT_ARRAY_PARAM_DESC(avd_slice_fits, avd_slice_fits_cases, desc);

static void avd_slice_fits_test(struct kunit *test)
{
	const struct avd_slice_fits_case *c = test->param_value;
	struct v4l2_pix_format_mplane pix = {
		.width = (u32)c->width_mbs * 16,
		.height = (u32)c->height_map_units * 16,
	};
	struct v4l2_ctrl_h264_sps sps = {
		.pic_width_in_mbs_minus1 = c->sps_width_mbs - 1,
		.pic_height_in_map_units_minus1 = c->sps_height_map_units - 1,
		.flags = c->fields ? 0 : V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY,
	};

	KUNIT_EXPECT_EQ(test, avd_h264_slice_fits(&pix, &sps, c->held,
						  c->held_bytes,
						  c->len, c->decoded_size),
			c->fits);
}

static struct kunit_case avd_h264_test_cases[] = {
	KUNIT_CASE_PARAM(avd_slice_data_test, avd_slice_data_gen_params),
	KUNIT_CASE_PARAM(avd_slice_fits_test, avd_slice_fits_gen_params),
	{}
};

static struct kunit_suite avd_h264_test_suite = {
	.name = "apple-avd-h264",
	.test_cases = avd_h264_test_cases,
};

kunit_test_suite(avd_h264_test_suite);
