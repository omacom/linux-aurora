// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

/*
 * KUnit for the T6030 external processor's sink catalog: the common parser's
 * external SDR path in its fw14 RGB8 profile, the DSC admission, and the
 * link budget, on small fixtures derived from two captured fw14
 * TimingElements publications (a Philips monitor and a Sony TV).
 */

#include <kunit/test.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include <drm/drm_modes.h>

#include "dptxep.h"
#include "parser.h"

/* ---- An OSSerialize writer and fw14-shaped fixtures ---- */

struct oss {
	struct kunit *test;
	u8 *buf;
	size_t len, cap;
};

static void oss_put(struct oss *o, const void *data, size_t n)
{
	KUNIT_ASSERT_LE(o->test, o->len + n + 4, o->cap);
	memcpy(o->buf + o->len, data, n);
	o->len += n;
}

static void oss_tag(struct oss *o, u32 type, u32 count)
{
	u8 tag[4];

	while (o->len % 4)
		o->buf[o->len++] = 0;
	put_unaligned_le32(type << 24 | count, tag);
	oss_put(o, tag, 4);
}

static void oss_num(struct oss *o, u64 value)
{
	u8 v[8];

	oss_tag(o, 4, 64);
	put_unaligned_le64(value, v);
	oss_put(o, v, 8);
}

static void oss_str(struct oss *o, const char *s)
{
	oss_tag(o, 9, strlen(s));
	oss_put(o, s, strlen(s));
}

static void oss_data(struct oss *o, size_t n)
{
	static const u8 zero[96];

	oss_tag(o, 10, n);
	oss_put(o, zero, n);
}

static void oss_bool(struct oss *o, const char *key, bool b)
{
	oss_str(o, key);
	oss_tag(o, 11, b);
}

static void oss_knum(struct oss *o, const char *key, u64 value)
{
	oss_str(o, key);
	oss_num(o, value);
}

/* A colour element as fw14 publishes it; score < 0 omits Score (virtual). */
struct fix_color {
	u32 id;
	bool virtual, supports_dsc;
	s64 score;
	u32 depth, encoding, range, eotf, colorimetry;
};

struct fix_axis {
	u32 active, total, front, sync, back, polarity, repetition, precise;
};

struct fix_timing {
	u32 id;
	s64 score;
	bool virtual, interlaced, split, preferred;
	s32 hpipes, vpipes;	/* < 0: absent, as on fw14 */
	struct fix_axis h, v;
	const struct fix_color *colors;
	unsigned int ncolors;
	const u32 *dsc;
	unsigned int ndsc;
	const u32 *unsafe;
	unsigned int nunsafe;
};

static void fix_axis(struct oss *o, const char *key, const struct fix_axis *a)
{
	oss_str(o, key);
	oss_tag(o, 1, 9);
	oss_knum(o, "Total", a->total);
	oss_knum(o, "FrontPorch", a->front);
	oss_knum(o, "SyncWidth", a->sync);
	oss_knum(o, "BackPorch", a->back);
	oss_knum(o, "Active", a->active);
	oss_knum(o, "SyncRate", a->precise);
	oss_knum(o, "PreciseSyncRate", a->precise);
	oss_knum(o, "SyncPolarity", a->polarity);
	oss_knum(o, "PixelRepetition", a->repetition);
}

static void fix_color(struct oss *o, const struct fix_color *c)
{
	bool metadata = !c->virtual && c->colorimetry == 16;

	oss_tag(o, 1, 11 + (c->score >= 0) + (metadata ? 3 : 1) - c->virtual);
	if (c->score >= 0)
		oss_knum(o, "Score", c->score);
	oss_knum(o, "ID", c->id);
	oss_bool(o, "IsVirtual", c->virtual);
	oss_knum(o, "ElementType", 1);
	oss_str(o, "ElementData");
	oss_data(o, 32);
	oss_knum(o, "Depth", c->depth);
	oss_knum(o, "PixelEncoding", c->encoding);
	oss_knum(o, "DynamicRange", c->range);
	oss_knum(o, "EOTF", c->eotf);
	oss_knum(o, "Colorimetry", c->colorimetry);
	oss_knum(o, "SupportsDSC", c->supports_dsc);
	oss_knum(o, "StandardType", 2);
	if (metadata) {
		/* Nested metadata the selection skips. */
		oss_str(o, "Chromaticity");
		oss_tag(o, 1, 3);
		oss_str(o, "Red");
		oss_tag(o, 1, 2);
		oss_knum(o, "X", 42432);
		oss_knum(o, "Y", 22336);
		oss_str(o, "Green");
		oss_tag(o, 1, 2);
		oss_knum(o, "X", 20864);
		oss_knum(o, "Y", 40512);
		oss_str(o, "Blue");
		oss_tag(o, 1, 2);
		oss_knum(o, "X", 9984);
		oss_knum(o, "Y", 3840);
		oss_str(o, "WhitePoint");
		oss_tag(o, 1, 3);
		oss_knum(o, "X", 20544);
		oss_knum(o, "Y", 21568);
		oss_knum(o, "Gamma", 144179);
	}
	if (!c->virtual) {
		oss_str(o, "HDRStaticMetadataTypes");
		oss_tag(o, 2, 0);
	}
}

static void fix_ids(struct oss *o, const char *key, const u32 *ids, unsigned int n)
{
	unsigned int i;

	oss_str(o, key);
	oss_tag(o, 2, n);
	for (i = 0; i < n; i++)
		oss_num(o, ids[i]);
}

static void fix_timing(struct oss *o, const struct fix_timing *t)
{
	unsigned int keys = 26 + (t->hpipes >= 0) + (t->vpipes >= 0), i;

	oss_tag(o, 1, keys);
	oss_knum(o, "Score", t->score);
	oss_knum(o, "ID", t->id);
	oss_bool(o, "IsVirtual", t->virtual);
	oss_knum(o, "ElementType", 0);
	oss_str(o, "ElementData");
	oss_data(o, 80);
	oss_str(o, "ColorModes");
	oss_tag(o, 2, t->ncolors);
	for (i = 0; i < t->ncolors; i++)
		fix_color(o, &t->colors[i]);
	fix_ids(o, "UnsafeColorElementIDs", t->unsafe, t->nunsafe);
	fix_ids(o, "DSCRequiredColorElementIDs", t->dsc, t->ndsc);
	fix_ids(o, "DiscreteMediaRefreshRates", NULL, 0);
	oss_bool(o, "IsInterlaced", t->interlaced);
	oss_bool(o, "IsSplit", t->split);
	oss_bool(o, "IsOverscanned", true);
	oss_bool(o, "IsPreferred", t->preferred);
	oss_bool(o, "IsPromoted", t->preferred);
	oss_knum(o, "ScanInformation", 1);
	oss_knum(o, "ValidPixelEncodings", 7037);
	oss_knum(o, "StandardType", 2);
	oss_knum(o, "TimingType", 4);
	oss_knum(o, "AspectRatio", 15);
	oss_knum(o, "PreciseAspectRatio", 116508);
	oss_knum(o, "TimingStandard", 4);
	oss_knum(o, "CEAShortID", 16);
	if (t->hpipes >= 0)
		oss_knum(o, "HorizontalPipeCount", t->hpipes);
	if (t->vpipes >= 0)
		oss_knum(o, "VerticalPipeCount", t->vpipes);
	fix_axis(o, "HorizontalAttributes", &t->h);
	fix_axis(o, "VerticalAttributes", &t->v);
	oss_knum(o, "MinimumVariableRefreshRate", 0);
	oss_knum(o, "MaximumVariableRefreshRate", 0);
}

/* The Philips monitor's colour list (timing 60 and its siblings). */
static const struct fix_color philips_colors[] = {
	{ .id = 1, .virtual = true, .score = -1, .depth = 8, .colorimetry = 10 },
	{ .id = 2, .virtual = true, .score = -1, .depth = 8, .range = 1, .colorimetry = 10 },
	{ .id = 73, .score = 232999, .depth = 8, .encoding = 3, .range = 1, .colorimetry = 1 },
	{ .id = 70, .score = 240999, .depth = 8, .colorimetry = 16 },
	{ .id = 71, .score = 240999, .depth = 8, .range = 1, .colorimetry = 16 },
	{ .id = 72, .score = 231999, .depth = 8, .encoding = 3, .range = 1 },
};

#define CEA_1080_H { 1920, 2200, 88, 44, 148, 1, 0, 4423680 }

/*
 * The Sony TV's admissible colours: 8-bit sRGB, AdobeRGB, 10-bit sRGB, each
 * taken compressed by the TV (SupportsDSC 1, as published on the HDMI port).
 */
static const struct fix_color sony_colors[] = {
	{ .id = 202, .score = 242999, .depth = 8, .colorimetry = 10, .supports_dsc = true },
	{ .id = 204, .score = 242999, .depth = 8, .range = 1, .colorimetry = 10,
	  .supports_dsc = true },
	{ .id = 5, .virtual = true, .score = -1, .depth = 8, .encoding = 3, .range = 1,
	  .colorimetry = 1 },
	{ .id = 203, .score = 244999, .depth = 8, .colorimetry = 13, .supports_dsc = true },
	{ .id = 211, .score = 442999, .depth = 10, .colorimetry = 10, .supports_dsc = true },
};

/* The same colours from a sink path without DSC (SupportsDSC 0). */
static const struct fix_color sony_colors_no_dsc[] = {
	{ .id = 202, .score = 242999, .depth = 8, .colorimetry = 10 },
	{ .id = 204, .score = 242999, .depth = 8, .range = 1, .colorimetry = 10 },
	{ .id = 203, .score = 244999, .depth = 8, .colorimetry = 13 },
	{ .id = 211, .score = 442999, .depth = 10, .colorimetry = 10 },
};

/* A sink that offers 10-bit RGB only (integration's common fallback keeps it). */
static const struct fix_color ten_bit_only[] = {
	{ .id = 211, .score = 442999, .depth = 10, .colorimetry = 10 },
};

static const u32 sony_dsc[] = { 202, 204, 5, 203, 211 };

/* Derived from the captured publications (and variants of their timing 60). */
static const struct fix_timing catalog[] = {
	/* Philips 60: 1080p60, preferred. */
	{ .id = 60, .score = 16224, .preferred = true, .hpipes = -1, .vpipes = -1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3932160 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	/* Philips 59: 1080p59.94, same score, listed later. */
	{ .id = 59, .score = 16224, .hpipes = -1, .vpipes = -1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3928232 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	/* Philips 6: virtual 480p. */
	{ .id = 6, .virtual = true, .hpipes = -1, .vpipes = -1,
	  .h = { 720, 858, 16, 62, 60, 0, 0, 2062322 },
	  .v = { 480, 525, 9, 6, 30, 0, 0, 3928232 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	/* Philips 24: DMT 640x480@75, no horizontal precise rate. */
	{ .id = 24, .score = 15098, .hpipes = -1, .vpipes = -1,
	  .h = { 640, 840, 16, 64, 120, 0, 0, 0 },
	  .v = { 480, 500, 1, 3, 16, 0, 0, 4915200 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	/* Sony 167: 2160p60, 594 MHz: four lanes only. */
	{ .id = 167, .score = 16598, .preferred = true, .hpipes = -1, .vpipes = -1,
	  .h = { 3840, 4400, 176, 88, 296, 1, 0, 8847360 },
	  .v = { 2160, 2250, 8, 10, 72, 1, 0, 3932160 },
	  .colors = sony_colors, .ncolors = ARRAY_SIZE(sony_colors) },
	/* Sony 156: 2160p30. */
	{ .id = 156, .score = 15379, .hpipes = -1, .vpipes = -1,
	  .h = { 3840, 4400, 176, 88, 296, 1, 0, 4423680 },
	  .v = { 2160, 2250, 8, 10, 72, 1, 0, 1966080 },
	  .colors = sony_colors, .ncolors = ARRAY_SIZE(sony_colors) },
	/* Sony 164: 2160p120, its colours DSC-required. */
	{ .id = 164, .score = 16215, .hpipes = -1, .vpipes = -1,
	  .h = { 3840, 4400, 176, 88, 296, 1, 0, 17694720 },
	  .v = { 2160, 2250, 8, 10, 72, 1, 0, 7864320 },
	  .colors = sony_colors, .ncolors = ARRAY_SIZE(sony_colors),
	  .dsc = sony_dsc, .ndsc = ARRAY_SIZE(sony_dsc) },
	/* Sony 173: 4096x2160p60. */
	{ .id = 173, .score = 16631, .hpipes = -1, .vpipes = -1,
	  .h = { 4096, 4400, 88, 88, 128, 1, 0, 8847360 },
	  .v = { 2160, 2250, 8, 10, 72, 1, 0, 3932160 },
	  .colors = sony_colors, .ncolors = ARRAY_SIZE(sony_colors) },
	/* Sony 158: 2160p23.976. */
	{ .id = 158, .score = 15323, .hpipes = -1, .vpipes = -1,
	  .h = { 3840, 5500, 1276, 88, 296, 1, 0, 3535409 },
	  .v = { 2160, 2250, 8, 10, 72, 1, 0, 1571293 },
	  .colors = sony_colors, .ncolors = ARRAY_SIZE(sony_colors) },
	/* Variants of Philips 60 outside the profile. */
	{ .id = 90, .score = 1, .interlaced = true, .hpipes = -1, .vpipes = -1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3932160 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	{ .id = 91, .score = 1, .split = true, .hpipes = -1, .vpipes = -1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3932160 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	{ .id = 92, .score = 1, .hpipes = 2, .vpipes = 1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3932160 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	{ .id = 93, .score = 1, .hpipes = -1, .vpipes = -1,
	  .h = { 1920, 2200, 88, 44, 148, 1, 1, 4423680 },
	  .v = { 1080, 1125, 4, 5, 36, 1, 0, 3932160 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
	/* Present pipe counts of one are accepted. */
	{ .id = 94, .score = 2, .hpipes = 1, .vpipes = 1,
	  .h = CEA_1080_H, .v = { 1080, 1125, 4, 5, 36, 1, 0, 3276800 },
	  .colors = philips_colors, .ncolors = ARRAY_SIZE(philips_colors) },
};

static void *build_catalog(struct kunit *test, const struct fix_timing *t,
			   unsigned int n, size_t *size)
{
	struct oss o = { .test = test, .cap = SZ_64K };
	unsigned int i;
	u8 magic[4];

	o.buf = kunit_kzalloc(test, o.cap, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, o.buf);
	put_unaligned_le32(0xd3, magic);
	oss_put(&o, magic, 4);
	oss_tag(&o, 2, n);
	for (i = 0; i < n; i++)
		fix_timing(&o, &t[i]);
	*size = o.len;
	return o.buf;
}

static struct dcp_display_mode *catalog_parse(struct kunit *test, const void *blob,
					      size_t size, unsigned int *count, bool dsc)
{
	struct dcp_display_mode *modes;
	struct dcp_parse_ctx ctx;

	KUNIT_ASSERT_EQ(test, parse(blob, size, &ctx), 0);
	ctx.v14_7_rgb8 = true;
	ctx.v14_7_dsc = dsc;
	modes = enumerate_modes(&ctx, count, 0, 0, 0, false, true);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(modes));
	return modes;
}

static struct dcp_display_mode *catalog_modes(struct kunit *test, const void *blob,
					      size_t size, unsigned int *count)
{
	return catalog_parse(test, blob, size, count, false);
}

static const struct dcp_display_mode *find_timing(const struct dcp_display_mode *modes,
						  unsigned int count, u32 id)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (modes[i].timing_mode_id == id)
			return &modes[i];
	return NULL;
}

/* fw14 shape: virtual colours without Score, no pipe counts, nested metadata. */
static void catalog_fw14_shape_test(struct kunit *test)
{
	const struct dcp_display_mode *m;
	struct dcp_display_mode *modes;
	unsigned int count, kept, rejected;
	size_t size;
	void *blob;

	blob = build_catalog(test, catalog, ARRAY_SIZE(catalog), &size);
	modes = catalog_modes(test, blob, size, &count);

	/* Parser: virtual 6, the variants 90-93 and DSC-only 164 are refused. */
	KUNIT_EXPECT_EQ(test, count, 8);
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 6));
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 90));
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 91));
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 92));
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 93));
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 164));

	/* Philips 60: colour 70 (limited range 71 and YCbCr 72/73 lose). */
	m = find_timing(modes, count, 60);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_EQ(test, m->color_mode_id, 70);
	KUNIT_EXPECT_EQ(test, m->mode.clock, 148500);
	KUNIT_EXPECT_EQ(test, m->mode.hdisplay, 1920);
	KUNIT_EXPECT_EQ(test, m->mode.hsync_start, 2008);
	KUNIT_EXPECT_EQ(test, m->mode.hsync_end, 2052);
	KUNIT_EXPECT_EQ(test, m->mode.htotal, 2200);
	KUNIT_EXPECT_EQ(test, m->mode.vdisplay, 1080);
	KUNIT_EXPECT_EQ(test, m->mode.vsync_start, 1084);
	KUNIT_EXPECT_EQ(test, m->mode.vsync_end, 1089);
	KUNIT_EXPECT_EQ(test, m->mode.vtotal, 1125);
	KUNIT_EXPECT_EQ(test, m->mode.flags, DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	KUNIT_EXPECT_FALSE(test, m->vrr);
	/* DMT 640x480@75 without a horizontal precise rate is admitted. */
	m = find_timing(modes, count, 24);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_EQ(test, m->mode.clock, 31500);
	KUNIT_EXPECT_EQ(test, m->mode.flags, DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	/* Sony: the 8-bit sRGB colour, never 10-bit 211 or AdobeRGB 203. */
	m = find_timing(modes, count, 167);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_EQ(test, m->color_mode_id, 202);
	KUNIT_EXPECT_EQ(test, m->mode.clock, 594000);
	KUNIT_ASSERT_NOT_NULL(test, find_timing(modes, count, 94));

	/* Four lanes of HBR2 (17.28 Gb/s): every uncompressed timing fits. */
	kept = dcp_catalog_budget(modes, count, 4, LINK_RATE_HBR2, &rejected);
	KUNIT_EXPECT_EQ(test, kept, 8);
	KUNIT_EXPECT_EQ(test, rejected, 0);
	/* The firmware's highest score (173) stays preferred. */
	m = find_timing(modes, kept, 173);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_TRUE(test, m->mode.type & DRM_MODE_TYPE_PREFERRED);
	kfree(modes);
}

/* Two lanes (pin D, 8.64 Gb/s): 4K60 is over budget, 4K30 is the largest. */
static void catalog_budget_test(struct kunit *test)
{
	const struct dcp_display_mode *m;
	struct dcp_display_mode *modes;
	unsigned int count, kept, rejected, i, preferred = 0;
	size_t size;
	void *blob;

	blob = build_catalog(test, catalog, ARRAY_SIZE(catalog), &size);
	modes = catalog_modes(test, blob, size, &count);
	kept = dcp_catalog_budget(modes, count, 2, LINK_RATE_HBR2, &rejected);
	KUNIT_EXPECT_EQ(test, kept, 6);
	KUNIT_EXPECT_EQ(test, rejected, 2);
	KUNIT_EXPECT_NULL(test, find_timing(modes, kept, 167));
	KUNIT_EXPECT_NULL(test, find_timing(modes, kept, 173));
	/* The preferred timing went: the largest, then fastest, is preferred. */
	m = find_timing(modes, kept, 156);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_TRUE(test, m->mode.type & DRM_MODE_TYPE_PREFERRED);
	for (i = 0; i < kept; i++)
		preferred += !!(modes[i].mode.type & DRM_MODE_TYPE_PREFERRED);
	KUNIT_EXPECT_EQ(test, preferred, 1);
	/* No route: nothing fits. */
	KUNIT_EXPECT_EQ(test, dcp_catalog_budget(modes, kept, 0, LINK_RATE_HBR2, &rejected), 0);
	kfree(modes);

	/* Two lanes of HBR3 (12.96 Gb/s) carry 4K60 (14.26 Gb/s) neither. */
	modes = catalog_modes(test, blob, size, &count);
	kept = dcp_catalog_budget(modes, count, 2, LINK_RATE_HBR3, &rejected);
	KUNIT_EXPECT_NULL(test, find_timing(modes, kept, 167));
	kfree(modes);
	/* Four lanes of HBR3 do, as four of HBR2. */
	modes = catalog_modes(test, blob, size, &count);
	kept = dcp_catalog_budget(modes, count, 4, LINK_RATE_HBR3, &rejected);
	KUNIT_EXPECT_NOT_NULL(test, find_timing(modes, kept, 167));
	kfree(modes);
}

/* The firmware's own preferred timing stays preferred when it fits. */
static void catalog_preferred_test(struct kunit *test)
{
	const struct dcp_display_mode *m;
	struct dcp_display_mode *modes;
	unsigned int count, kept, rejected;
	size_t size;
	void *blob;

	blob = build_catalog(test, catalog, 2, &size);
	modes = catalog_modes(test, blob, size, &count);
	KUNIT_ASSERT_EQ(test, count, 2);
	kept = dcp_catalog_budget(modes, count, 2, LINK_RATE_HBR2, &rejected);
	KUNIT_EXPECT_EQ(test, kept, 2);
	m = find_timing(modes, kept, 60);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_TRUE(test, m->mode.type & DRM_MODE_TYPE_PREFERRED);
	m = find_timing(modes, kept, 59);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_FALSE(test, m->mode.type & DRM_MODE_TYPE_PREFERRED);
	kfree(modes);
}

/* Without the fw14 profile the external SDR path keeps its own (10-bit) choice. */
static void catalog_profile_scope_test(struct kunit *test)
{
	struct dcp_display_mode *modes;
	struct dcp_parse_ctx ctx;
	unsigned int count;
	size_t size;
	void *blob;

	blob = build_catalog(test, &catalog[4], 1, &size);
	KUNIT_ASSERT_EQ(test, parse(blob, size, &ctx), 0);
	modes = enumerate_modes(&ctx, &count, 0, 0, 0, false, true);
	KUNIT_ASSERT_FALSE(test, IS_ERR_OR_NULL(modes));
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 211);
	kfree(modes);
}

/* Equal scores: the lower colour ID wins. */
static void catalog_color_tie_test(struct kunit *test)
{
	static const struct fix_color tie[] = {
		{ .id = 77, .score = 5000, .depth = 8, .colorimetry = 16 },
		{ .id = 75, .score = 5000, .depth = 8, .colorimetry = 10 },
		{ .id = 76, .score = 4000, .depth = 8, .colorimetry = 1 },
	};
	struct fix_timing t = catalog[0];
	struct dcp_display_mode *modes;
	unsigned int count;
	size_t size;
	void *blob;

	t.colors = tie;
	t.ncolors = ARRAY_SIZE(tie);
	blob = build_catalog(test, &t, 1, &size);
	modes = catalog_modes(test, blob, size, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 75);
	kfree(modes);
}

/*
 * A timing without an 8-bit RGB colour keeps the common fallback (here the
 * 10-bit sRGB colour) instead of being refused: no sink loses its modes to
 * the profile.
 */
static void catalog_fallback_test(struct kunit *test)
{
	struct fix_timing t = catalog[4];
	struct dcp_display_mode *modes;
	unsigned int count;
	size_t size;
	void *blob;

	t.colors = ten_bit_only;
	t.ncolors = ARRAY_SIZE(ten_bit_only);
	blob = build_catalog(test, &t, 1, &size);
	modes = catalog_modes(test, blob, size, &count);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 211);
	KUNIT_EXPECT_FALSE(test, modes[0].dsc);
	kfree(modes);
}

/*
 * DSC: Sony 164 (2160p120, 1188 MHz) has only DSC-required colours. It is
 * admitted with its 8-bit sRGB colour when the sink takes it compressed and
 * 8 bpp fits the lanes less FEC: four lanes of HBR2 (pin C) or HBR3 (the
 * HDMI port), not two of HBR2 (pin D). 4K60 stays uncompressed.
 */
static void catalog_dsc_test(struct kunit *test)
{
	struct fix_timing sony[] = { catalog[4], catalog[6] };
	const struct dcp_display_mode *m;
	struct dcp_display_mode *modes;
	unsigned int count, kept, rejected;
	size_t size;
	void *blob;

	blob = build_catalog(test, sony, ARRAY_SIZE(sony), &size);
	modes = catalog_parse(test, blob, size, &count, true);
	KUNIT_ASSERT_EQ(test, count, 2);
	m = find_timing(modes, count, 164);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_TRUE(test, m->dsc);
	KUNIT_EXPECT_EQ(test, m->color_mode_id, 202);
	KUNIT_EXPECT_EQ(test, m->mode.clock, 1188000);
	m = find_timing(modes, count, 167);
	KUNIT_ASSERT_NOT_NULL(test, m);
	KUNIT_EXPECT_FALSE(test, m->dsc);
	KUNIT_EXPECT_EQ(test, m->color_mode_id, 202);

	kept = dcp_catalog_budget(modes, count, 4, LINK_RATE_HBR2, &rejected);
	KUNIT_EXPECT_EQ(test, kept, 2);
	KUNIT_EXPECT_EQ(test, rejected, 0);
	kfree(modes);
	modes = catalog_parse(test, blob, size, &count, true);
	kept = dcp_catalog_budget(modes, count, 4, LINK_RATE_HBR3, &rejected);
	KUNIT_EXPECT_NOT_NULL(test, find_timing(modes, kept, 164));
	kfree(modes);
	/* Pin D: 1188 MHz at 8 bpp is 9.5 Gb/s, over 8.64 less FEC; 4K60 too. */
	modes = catalog_parse(test, blob, size, &count, true);
	kept = dcp_catalog_budget(modes, count, 2, LINK_RATE_HBR2, &rejected);
	KUNIT_EXPECT_EQ(test, kept, 0);
	KUNIT_EXPECT_EQ(test, rejected, 2);
	kfree(modes);

	/* A sink path without DSC: 164 is refused, 167 unchanged. */
	sony[0].colors = sony[1].colors = sony_colors_no_dsc;
	sony[0].ncolors = sony[1].ncolors = ARRAY_SIZE(sony_colors_no_dsc);
	blob = build_catalog(test, sony, ARRAY_SIZE(sony), &size);
	modes = catalog_parse(test, blob, size, &count, true);
	KUNIT_EXPECT_EQ(test, count, 1);
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 164));
	KUNIT_EXPECT_NOT_NULL(test, find_timing(modes, count, 167));
	kfree(modes);

	/* An unsafe DSC colour is never taken (202 is the only full-range sRGB). */
	sony[0].colors = sony[1].colors = sony_colors;
	sony[0].ncolors = sony[1].ncolors = ARRAY_SIZE(sony_colors);
	sony[1].unsafe = sony_dsc;
	sony[1].nunsafe = 1;
	blob = build_catalog(test, sony, ARRAY_SIZE(sony), &size);
	modes = catalog_parse(test, blob, size, &count, true);
	KUNIT_EXPECT_EQ(test, count, 1);
	KUNIT_EXPECT_NULL(test, find_timing(modes, count, 164));
	kfree(modes);
}

static struct kunit_case catalog_cases[] = {
	KUNIT_CASE(catalog_fw14_shape_test),
	KUNIT_CASE(catalog_budget_test),
	KUNIT_CASE(catalog_preferred_test),
	KUNIT_CASE(catalog_profile_scope_test),
	KUNIT_CASE(catalog_color_tie_test),
	KUNIT_CASE(catalog_fallback_test),
	KUNIT_CASE(catalog_dsc_test),
	{}
};

static struct kunit_suite catalog_suite = {
	.name = "apple-dcp-t6030-catalog",
	.test_cases = catalog_cases,
};

kunit_test_suite(catalog_suite);
