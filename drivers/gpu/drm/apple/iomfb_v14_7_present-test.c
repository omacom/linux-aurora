// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

/*
 * The 14.7 panel present encoding (iomfb_v14_7_swap.h) is checked byte for
 * byte: every case's 0x1b58-byte A407 request must match the CRC32 of a
 * reference request recorded for the J514S panel's hardware-qualified
 * profile (a single linear XRGB8888 or ARGB8888 surface in slot 0, full
 * panel and unscaled, transfer 13 / primaries 12, update masks 0x80000007
 * over a black background, the level in nits only when it changes).
 */

#include <kunit/test.h>
#include <linux/crc32.h>
#include <linux/stddef.h>
#include <linux/unaligned.h>

#include "iomfb_v14_7_swap.h"

#define PANEL_W 3024
#define PANEL_H 1964

/* Frames of the representative cases. */
static const struct {
	u64 iova;
	u32 stride;
	bool opaque;		/* XRGB8888; ARGB8888 otherwise */
} v14_7_frames[] = {
	[1] = { .iova = 0x10000004000ULL, .stride = 12096, .opaque = true },
	[2] = { .iova = 0x10040000000ULL, .stride = 12160, .opaque = false },
};

enum v14_7_path {
	/* An atomic present: the frame (or the background) with its swap ID and level. */
	V14_7_ATOMIC,
	/* A brightness-only re-present of the frame shown. */
	V14_7_REPRESENT,
	/* The soft-DPMS background at the frame's place. */
	V14_7_DPMS_BLANK,
};

struct v14_7_case {
	const char *name;
	enum v14_7_path path;
	u32 swap_id;
	unsigned int frame;	/* index into v14_7_frames; 0 is the background */
	bool change;
	u32 nits;
	u32 crc32;		/* standard CRC-32 of the 0x1b58-byte request */
};

static const struct v14_7_case v14_7_cases[] = {
	{ "primary frame", V14_7_ATOMIC, 0x2a, 1, false, 0, 0xf38285f2 },
	{ "first present at 140 nits", V14_7_ATOMIC, 7, 1, true, 140, 0xd05cf8c9 },
	{ "soft DPMS blank at 0 nits", V14_7_DPMS_BLANK, 0x1001, 1, true, 0, 0xdb3c1a63 },
	{ "brightness-only re-present", V14_7_REPRESENT, 0x1002, 1, true, 300, 0xe7387727 },
	{ "ARGB8888 at the 500-nit maximum", V14_7_ATOMIC, 0xfffffffe, 2, true, 500, 0x5909dbe7 },
	{ "background without a level", V14_7_ATOMIC, 1, 0, false, 0, 0xa5a06192 },
	{ "nits change to 1 on a re-present", V14_7_REPRESENT, 3, 2, true, 1, 0x18dcd7c0 },
};

static void v14_7_case_desc(const struct v14_7_case *c, char *desc)
{
	strscpy(desc, c->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(v14_7_case, v14_7_cases, v14_7_case_desc);

static u32 v14_7_crc(const void *buf)
{
	return crc32_le(~0U, buf, DCP_V14_SWAP_SIZE) ^ ~0U;
}

/*
 * Build a case the way the driver does (dcp_v14_swap_locked()): the surface
 * of the frame, the swap record with the ID A406 gave, then the level the
 * backlight policy hands the present. A brightness-only present repeats the
 * same surface; the soft-DPMS blank is the background alone.
 */
static void v14_7_encode_case(struct kunit *test, u8 *swap, const struct v14_7_case *c)
{
	u8 surface[DCP_V14_SURFACE_SIZE];

	if (c->frame)
		dcp_v14_encode_surface(surface, v14_7_frames[c->frame].stride, PANEL_W, PANEL_H,
				       v14_7_frames[c->frame].opaque);
	dcp_v14_encode_swap(swap, c->swap_id, 0xff000000,
			    c->frame && c->path != V14_7_DPMS_BLANK ? surface : NULL,
			    c->frame ? v14_7_frames[c->frame].iova : 0,
			    c->frame ? PANEL_W : 0, c->frame ? PANEL_H : 0, 0, 0);
	KUNIT_ASSERT_EQ(test, dcp_v14_encode_nits(swap, c->change, c->nits), 0);
}

static void v14_7_present_encoding_test(struct kunit *test)
{
	const struct v14_7_case *c = test->param_value;
	u8 *swap;

	swap = kunit_kmalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, swap);
	memset(swap, 0xa5, DCP_V14_SWAP_SIZE);
	v14_7_encode_case(test, swap, c);
	KUNIT_EXPECT_EQ_MSG(test, v14_7_crc(swap), c->crc32, "%s", c->name);
}

/* The wire landmarks of the panel's profile, and its colour bytes. */
static void v14_7_layout_test(struct kunit *test)
{
	u8 surface[DCP_V14_SURFACE_SIZE];
	u8 *b;

	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_SIZE, 0x1b58);
	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_SURFACE(0), 0x508);
	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_IOVA(0), 0xdb8);
	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_SURF_NULL(0), 0x1b4b);
	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_NITS_PRESENT, 0x354);
	KUNIT_EXPECT_EQ(test, DCP_V14_SWAP_NITS, 0x35c);

	b = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, b);
	dcp_v14_encode_surface(surface, 12096, PANEL_W, PANEL_H, true);
	dcp_v14_encode_swap(b, 0x2a, 0xff000000, surface, v14_7_frames[1].iova, PANEL_W,
			    PANEL_H, 0, 0);
	/* Surface 0: transfer 13, primaries 12, premultiplied XRGB, identity 1. */
	KUNIT_EXPECT_EQ(test, b[0x51b], 13);
	KUNIT_EXPECT_EQ(test, b[0x51c], 12);
	KUNIT_EXPECT_EQ(test, b[0x50a], 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x508 + 0x35), 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(b + 0x508 + 0x71), 4U);
	/* Swap: identity 1 in slot 0, per-layer word 0, full masks, black. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x98), 0x2aU);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x9c), 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0xec), 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x14c), 0x80000007U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x150), 0x80000007U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(b + 0x154), 0xff000000U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(b + 0xdb8), v14_7_frames[1].iova);
	KUNIT_EXPECT_EQ(test, b[0x354], 0);
	/* Ten null flags, surface 0's cleared; the two tail flags set. */
	KUNIT_EXPECT_EQ(test, b[0x1b4b], 0);
	KUNIT_EXPECT_EQ(test, b[0x1b4c], 1);
	KUNIT_EXPECT_EQ(test, b[0x1b54], 1);
	KUNIT_EXPECT_EQ(test, b[0x1b56], 1);
	KUNIT_EXPECT_EQ(test, b[0x1b57], 1);
	/* ARGB is straight alpha. */
	dcp_v14_encode_surface(surface, 12160, PANEL_W, PANEL_H, false);
	KUNIT_EXPECT_EQ(test, surface[0x02], 0);
}

static void v14_7_nits_test(struct kunit *test)
{
	u8 *swap;
	u32 i;

	KUNIT_EXPECT_EQ(test, dcp_v14_nits_binary64(0), 0ULL);
	KUNIT_EXPECT_EQ(test, dcp_v14_nits_binary64(1), 0x3ff0000000000000ULL);
	KUNIT_EXPECT_EQ(test, dcp_v14_nits_binary64(140), 0x4061800000000000ULL);
	KUNIT_EXPECT_EQ(test, dcp_v14_nits_binary64(500), 0x407f400000000000ULL);
	swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, swap);
	/* Above the panel maximum the level is refused. */
	KUNIT_EXPECT_EQ(test, dcp_v14_encode_nits(swap, true, 501), -EINVAL);
	KUNIT_EXPECT_EQ(test, swap[0x354], 0);
	/* Clearing the level leaves no stale value behind. */
	KUNIT_ASSERT_EQ(test, dcp_v14_encode_nits(swap, true, 333), 0);
	KUNIT_EXPECT_EQ(test, swap[0x354], 1);
	KUNIT_ASSERT_EQ(test, dcp_v14_encode_nits(swap, false, 333), 0);
	for (i = 0x354; i < 0x364; i++)
		KUNIT_EXPECT_EQ(test, swap[i], 0);
}

static struct kunit_case v14_7_present_cases[] = {
	KUNIT_CASE_PARAM(v14_7_present_encoding_test, v14_7_case_gen_params),
	KUNIT_CASE(v14_7_layout_test),
	KUNIT_CASE(v14_7_nits_test),
	{}
};

static struct kunit_suite v14_7_present_suite = {
	.name = "apple-dcp-v14_7-present",
	.test_cases = v14_7_present_cases,
};

kunit_test_suite(v14_7_present_suite);
