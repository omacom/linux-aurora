// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_atomic_state_helper.h>

#include "dcp.h"
#include <linux/unaligned.h>

#include "connector.h"
#include "dcp-internal.h"
#include "dcp-property.h"
#include "iomfb_internal.h"
#include "parser.h"

static struct device parser_device = {
	.init_name = "apple-display-parser-test",
};

static struct apple_dcp parser_dcp = {
	.dev = &parser_device,
};

struct parser_blob {
	u8 data[8192];
	u32 size;
};

struct parser_record {
	s64 id, score, color_score, color_id;
	s64 active, total, porch, sync, rate, horizontal_rate;
	u32 missing, horizontal_missing, color_missing, second_color_missing;
	s64 first_format;
	bool bad_score_type;
	bool vrr;
};

enum parser_missing {
	MISSING_SCORE = BIT(0),
	MISSING_ACTIVE = BIT(1),
	MISSING_TOTAL = BIT(2),
	MISSING_PORCH = BIT(3),
	MISSING_SYNC = BIT(4),
	MISSING_RATE = BIT(5),
	MISSING_COLORS = BIT(6),
	MISSING_COLOR_SCORE = BIT(0),
	MISSING_COLOR_DEPTH = BIT(1),
};

static void blob_tag(struct parser_blob *blob, u32 type, u32 count)
{
	blob->size = ALIGN(blob->size, 4);
	put_unaligned_le32(count | type << 24, blob->data + blob->size);
	blob->size += 4;
}

static void blob_key(struct parser_blob *blob, const char *key)
{
	size_t length = strlen(key);

	blob_tag(blob, 9, length);
	memcpy(blob->data + blob->size, key, length);
	blob->size += length;
}

static void blob_int(struct parser_blob *blob, const char *key, s64 value)
{
	blob_key(blob, key);
	blob_tag(blob, 4, 0);
	put_unaligned_le64(value, blob->data + blob->size);
	blob->size += 8;
}

static void blob_bool(struct parser_blob *blob, const char *key, bool value)
{
	blob_key(blob, key);
	blob_tag(blob, 11, value);
}

static void blob_dimension(struct parser_blob *blob, const char *key,
			   const struct parser_record *record, bool vertical)
{
	u32 missing = vertical ? record->missing >> 1 & 0x1f : record->horizontal_missing;

	blob_key(blob, key);
	blob_tag(blob, 1, 5 - hweight32(missing));
	if (!(missing & BIT(0)))
		blob_int(blob, "Active", vertical ? record->active : 1920);
	if (!(missing & BIT(1)))
		blob_int(blob, "Total", vertical ? record->total : 2200);
	if (!(missing & BIT(2)))
		blob_int(blob, "FrontPorch", vertical ? record->porch : 88);
	if (!(missing & BIT(3)))
		blob_int(blob, "SyncWidth", vertical ? record->sync : 44);
	if (!(missing & BIT(4)))
		blob_int(blob, "PreciseSyncRate",
			 vertical ? record->rate : record->horizontal_rate);
}

static void blob_color(struct parser_blob *blob, s64 id, s64 score,
		       u32 missing, s64 format)
{
	blob_tag(blob, 1, 8 - hweight32(missing));
	blob_bool(blob, "IsVirtual", false);
	blob_int(blob, "ID", id);
	if (!(missing & MISSING_COLOR_SCORE))
		blob_int(blob, "Score", score);
	if (!(missing & MISSING_COLOR_DEPTH))
		blob_int(blob, "Depth", 8);
	blob_int(blob, "Colorimetry", DCP_COLORIMETRY_RGB);
	blob_int(blob, "DynamicRange", DCP_COLOR_YCBCR_RANGE_FULL);
	blob_int(blob, "EOTF", DCP_EOTF_SDR_GAMMA);
	blob_int(blob, "PixelEncoding", format);
}

static void blob_record(struct parser_blob *blob, const struct parser_record *record)
{
	unsigned int count = 6 + (record->vrr ? 2 : 0);

	count -= !!(record->missing & MISSING_SCORE);
	count -= !!(record->missing & MISSING_COLORS);
	blob_tag(blob, 1, count);
	blob_bool(blob, "IsVirtual", false);
	blob_int(blob, "ID", record->id);
	if (!(record->missing & MISSING_SCORE)) {
		if (record->bad_score_type) {
			blob_key(blob, "Score");
			blob_tag(blob, 9, 1);
			blob->data[blob->size++] = 'x';
		} else {
			blob_int(blob, "Score", record->score);
		}
	}
	blob_dimension(blob, "HorizontalAttributes", record, false);
	blob_dimension(blob, "VerticalAttributes", record, true);
	if (!(record->missing & MISSING_COLORS)) {
		blob_key(blob, "ColorModes");
		blob_tag(blob, 2, 2);
		blob_color(blob, record->color_id, record->color_score,
			   record->color_missing, record->first_format);
		blob_color(blob, 101, record->color_score,
			   record->second_color_missing, DCP_COLOR_FORMAT_RGB);
	}
	if (record->vrr) {
		blob_int(blob, "MinimumVariableRefreshRate", 48 << 16);
		blob_int(blob, "MaximumVariableRefreshRate", 120 << 16);
	}
}

static struct parser_record valid_record(s64 id, s64 score)
{
	return (struct parser_record) {
		.id = id, .score = score, .color_score = 10, .color_id = 100,
		.active = 1080, .total = 1125, .porch = 4, .sync = 5,
		.rate = 60 << 16, .horizontal_rate = (675U << 16) / 10,
	};
}

static struct parser_blob *make_blob(struct kunit *test,
				     const struct parser_record *records, unsigned int count)
{
	struct parser_blob *blob = kunit_kzalloc(test, sizeof(*blob), GFP_KERNEL);
	unsigned int i;

	if (!blob)
		return NULL;
	put_unaligned_le32(0xd3, blob->data);
	blob->size = 4;
	blob_tag(blob, 2, count);
	for (i = 0; i < count; i++)
		blob_record(blob, &records[i]);
	return blob;
}

static struct dcp_display_mode *parse_blob(struct parser_blob *blob,
					   unsigned int *count, unsigned int notch, bool internal)
{
	struct dcp_parse_ctx ctx;
	int ret = parse(blob->data, blob->size, &ctx);

	if (ret)
		return ERR_PTR(ret);
	ctx.dcp = &parser_dcp;
	return enumerate_modes(&ctx, count, 500, 300, notch, internal);
}

static void parser_score_presence(struct kunit *test)
{
	struct parser_record records[] = { valid_record(7, 100), valid_record(8, 0) };
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count;

	records[1].missing = MISSING_SCORE;
	blob = make_blob(test, records, ARRAY_SIZE(records));
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].timing_mode_id, 7U);
	KUNIT_EXPECT_TRUE(test, modes[0].mode.type & DRM_MODE_TYPE_PREFERRED);
	kfree(modes);

	records[0].missing = MISSING_SCORE;
	records[1] = valid_record(8, 0);
	blob = make_blob(test, records, ARRAY_SIZE(records));
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].timing_mode_id, 8U);
	kfree(modes);
}

static void parser_legacy_ties(struct kunit *test)
{
	struct parser_record records[] = { valid_record(7, 50), valid_record(8, 50) };
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count, internal;

	for (internal = 0; internal < 2; internal++) {
		blob = make_blob(test, records, ARRAY_SIZE(records));
		KUNIT_ASSERT_NOT_NULL(test, blob);
		modes = parse_blob(blob, &count, 0, internal);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
		KUNIT_EXPECT_EQ(test, count, 2U);
		KUNIT_EXPECT_TRUE(test, modes[0].mode.type & DRM_MODE_TYPE_PREFERRED);
		KUNIT_EXPECT_FALSE(test, modes[1].mode.type & DRM_MODE_TYPE_PREFERRED);
		KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 100U);
		kfree(modes);
	}
	records[1].score = 51;
	blob = make_blob(test, records, ARRAY_SIZE(records));
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_FALSE(test, modes[0].mode.type & DRM_MODE_TYPE_PREFERRED);
	KUNIT_EXPECT_TRUE(test, modes[1].mode.type & DRM_MODE_TYPE_PREFERRED);
	kfree(modes);
}

static void parser_color_presence(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count, missing;

	for (missing = MISSING_COLOR_SCORE; missing <= MISSING_COLOR_DEPTH; missing <<= 1) {
		record.color_missing = missing;
		blob = make_blob(test, &record, 1);
		KUNIT_ASSERT_NOT_NULL(test, blob);
		modes = parse_blob(blob, &count, 0, false);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
		KUNIT_EXPECT_EQ(test, count, 1U);
		KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 101U);
		kfree(modes);
	}
	record.color_missing = 0;
	record.first_format = DCP_COLOR_FORMAT_YCBCR420;
	record.second_color_missing = MISSING_COLOR_SCORE;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].color_mode_id, 100U);
	KUNIT_EXPECT_EQ(test, modes[0].sdr_rgb.score, -1LL);
	kfree(modes);
	record.missing = MISSING_COLORS;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 0U);
	kfree(modes);
}

static void parser_geometry(struct kunit *test)
{
	struct parser_record record;
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count, bit;

	for (bit = MISSING_ACTIVE; bit <= MISSING_RATE; bit <<= 1) {
		record = valid_record(7, 10);
		record.missing = bit;
		blob = make_blob(test, &record, 1);
		KUNIT_ASSERT_NOT_NULL(test, blob);
		modes = parse_blob(blob, &count, 0, false);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
		KUNIT_EXPECT_EQ(test, count, 0U);
		kfree(modes);
	}
}

static void parser_horizontal_rate(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count, bit;

	record.horizontal_missing = BIT(4);
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].mode.clock, 148500);
	kfree(modes);
	record.horizontal_missing = 0;
	record.horizontal_rate = 0;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].mode.clock, 148500);
	kfree(modes);
	record.horizontal_rate = -1;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 0U);
	kfree(modes);
	record.horizontal_rate = (675U << 16) / 10;
	for (bit = BIT(0); bit <= BIT(3); bit <<= 1) {
		record.horizontal_missing = bit;
		blob = make_blob(test, &record, 1);
		KUNIT_ASSERT_NOT_NULL(test, blob);
		modes = parse_blob(blob, &count, 0, false);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
		KUNIT_EXPECT_EQ(test, count, 0U);
		kfree(modes);
	}
}

static void parser_bad_score(struct kunit *test)
{
	struct parser_record records[] = { valid_record(7, 10), valid_record(8, 20) };
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count;

	records[0].bad_score_type = true;
	blob = make_blob(test, records, ARRAY_SIZE(records));
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].timing_mode_id, 8U);
	kfree(modes);
	records[0].bad_score_type = false;
	records[0].score = -1;
	blob = make_blob(test, records, ARRAY_SIZE(records));
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].timing_mode_id, 8U);
	kfree(modes);
}

static void parser_bad_geometry(struct kunit *test)
{
	static const s64 values[][5] = {
		{ 0, 1125, 4, 5, 60 << 16 },
		{ -1, 1125, 4, 5, 60 << 16 },
		{ 1200, 1125, 4, 5, 60 << 16 },
		{ 1080, 1125, -1, 5, 60 << 16 },
		{ 1080, 1125, 4, -1, 60 << 16 },
		{ 1080, 1125, 4, 5, 0 },
		{ 1080, 1125, 4, 5, 1 },
		{ 65536, 65536, 4, 5, 60 << 16 },
		{ 1080, 65535, 4, 5, U32_MAX },
	};
	struct parser_record record;
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count, i;

	for (i = 0; i < ARRAY_SIZE(values); i++) {
		record = valid_record(7, 10);
		record.active = values[i][0];
		record.total = values[i][1];
		record.porch = values[i][2];
		record.sync = values[i][3];
		record.rate = values[i][4];
		blob = make_blob(test, &record, 1);
		KUNIT_ASSERT_NOT_NULL(test, blob);
		modes = parse_blob(blob, &count, 0, false);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
		KUNIT_EXPECT_EQ(test, count, 0U);
		kfree(modes);
	}
}

static void parser_notch_vrr(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct dcp_display_mode *modes;
	struct parser_blob *blob;
	unsigned int count;

	record.rate = 120 << 16;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 1080, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 0U);
	kfree(modes);
	modes = parse_blob(blob, &count, 32, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, modes[0].mode.vdisplay, 1048U);
	KUNIT_EXPECT_TRUE(test, modes[0].vrr);
	KUNIT_EXPECT_EQ(test, modes[0].min_vrr, 24U << 16);
	kfree(modes);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_FALSE(test, modes[0].vrr);
	kfree(modes);
	record.vrr = true;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	modes = parse_blob(blob, &count, 0, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, modes);
	KUNIT_EXPECT_EQ(test, modes[0].min_vrr, 48U << 16);
	KUNIT_EXPECT_EQ(test, modes[0].max_vrr, 120U << 16);
	kfree(modes);
}

static void parser_replacement(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct dcp_display_mode selected, *previous;
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;
	struct drm_display_mode requested;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	dcp_modes_init(dcp);
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	requested = dcp->modes[0].mode;
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, &selected));
	KUNIT_EXPECT_EQ(test, selected.timing_mode_id, 7U);
	previous = dcp->modes;
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size - 1, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_LT(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, previous);
	KUNIT_EXPECT_EQ(test, dcp->nr_modes, 1U);
	record.id = 8;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	KUNIT_EXPECT_EQ(test, dcp->modes[0].timing_mode_id, 8U);
	KUNIT_EXPECT_EQ(test, selected.timing_mode_id, 7U);
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, &selected));
	KUNIT_EXPECT_EQ(test, selected.timing_mode_id, 8U);
	requested.clock++;
	KUNIT_EXPECT_FALSE(test, lookup_mode(dcp, &requested, NULL));
	kfree(dcp->modes);
}

/*
 * Described again after a withdrawal it was kept connected through, the same
 * display keeps its EDID; any other replacement, a placeholder being
 * retried, or a hold whose connector was reported disconnected since drops
 * it.
 */
static void parser_held_keeps_edid(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_connector *connector = kunit_kzalloc(test, sizeof(*connector), GFP_KERNEL);
	struct platform_device *pipeline = kunit_kzalloc(test, sizeof(*pipeline), GFP_KERNEL);
	u8 raw[EDID_LENGTH] = { 0 };
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;
	unsigned int pass;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, connector);
	KUNIT_ASSERT_NOT_NULL(test, pipeline);
	dcp_modes_init(dcp);
	apple_connector_edid_init(connector);
	apple_connector_set_pipeline(connector, pipeline);
	dcp->connector = connector;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	/*
	 * Pass 0: not held. Pass 1: held. Pass 2: held, placeholder retried.
	 * Pass 3: held, but the connector was reported disconnected since.
	 */
	for (pass = 0; pass < 4; pass++) {
		const struct drm_edid *edid;
		u64 generation;

		KUNIT_ASSERT_TRUE(test, apple_connector_edid_begin(connector, pipeline,
								   &generation));
		raw[0] = pass + 1;
		edid = drm_edid_alloc(raw, sizeof(raw));
		KUNIT_ASSERT_NOT_NULL(test, edid);
		KUNIT_ASSERT_TRUE(test, apple_connector_edid_install(connector, pipeline,
								     generation, edid));
		atomic_set(&dcp->external_held, pass != 0);
		dcp->placeholder_retried = pass == 2;
		WRITE_ONCE(connector->connected, pass != 3);
		KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
		ctx.dcp = &parser_dcp;
		KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx,
							dcp_modes_transfer_begin(dcp)), 0);
		if (pass == 1)
			KUNIT_EXPECT_NOT_NULL(test, connector->drm_edid);
		else
			KUNIT_EXPECT_NULL(test, connector->drm_edid);
		apple_connector_invalidate_edid(connector);
	}
	mutex_destroy(&connector->edid_lock);
	kfree(dcp->modes);
}

static void parser_attachment_admission(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct dcp_display_mode selected, *previous;
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;
	struct drm_display_mode requested;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	dcp_modes_init(dcp);
	spin_lock_init(&dcp->mode_state.lock);
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	requested = dcp->modes[0].mode;
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, &selected));
	previous = dcp->modes;
	dcp_modes_begin_attachment(dcp);
	KUNIT_EXPECT_FALSE(test, lookup_mode(dcp, &requested, NULL));
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size - 1, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_LT(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, previous);
	KUNIT_EXPECT_FALSE(test, lookup_mode(dcp, &requested, NULL));
	record.id = 8;
	record.active = 720;
	record.total = 750;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	KUNIT_EXPECT_FALSE(test, lookup_mode(dcp, &requested, NULL));
	requested = dcp->modes[0].mode;
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, &selected));
	KUNIT_EXPECT_EQ(test, selected.timing_mode_id, 8U);
	/* Link recovery changes modeset state, never catalog admission. */
	dcp_mode_invalidate(&dcp->mode_state);
	dcp_mode_hotplug(&dcp->mode_state, false, NULL);
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, &selected));
	KUNIT_EXPECT_EQ(test, selected.timing_mode_id, 8U);
	kfree(dcp->modes);
}

static void parser_stale_transfer(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;
	struct drm_display_mode requested;
	u64 old_generation, current_generation;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	dcp_modes_init(dcp);
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	old_generation = dcp_modes_transfer_begin(dcp);
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, old_generation), 0);
	requested = dcp->modes[0].mode;

	/* An old transfer cannot admit a catalog on a new attachment. */
	dcp_modes_begin_attachment(dcp);
	current_generation = dcp_modes_transfer_begin(dcp);
	KUNIT_ASSERT_NE(test, old_generation, current_generation);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, old_generation), -ESTALE);
	KUNIT_EXPECT_FALSE(test, lookup_mode(dcp, &requested, NULL));

	record.id = 8;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, current_generation), 0);
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, NULL));
	KUNIT_EXPECT_EQ(test, dcp->modes[0].timing_mode_id, 8U);

	/* A late old transfer cannot overwrite an already admitted new one. */
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, old_generation), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp->modes[0].timing_mode_id, 8U);
	KUNIT_EXPECT_TRUE(test, lookup_mode(dcp, &requested, NULL));
	kfree(dcp->modes);
}

struct parser_catalog_observer {
	struct apple_connector connector;
	unsigned int notifications;
	unsigned int reasons;
	bool locked;
	struct apple_dcp *dcp;
};

static void parser_catalog_ready(struct work_struct *work)
{
	struct apple_connector *connector = container_of(work, struct apple_connector, hotplug_wq);
	struct parser_catalog_observer *observer =
		container_of(connector, struct parser_catalog_observer, connector);

	observer->notifications++;
	observer->reasons = atomic_xchg(&connector->hotplug_reasons, 0);
	observer->locked = mutex_is_locked(&observer->dcp->modes_lock);
}

static void parser_route_catalog_handoff(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct parser_catalog_observer *observer =
		kunit_kzalloc(test, sizeof(*observer), GFP_KERNEL);
	struct apple_connector *connector = observer ? &observer->connector : NULL;
	struct apple_connector *other = kunit_kzalloc(test, sizeof(*other), GFP_KERNEL);
	struct platform_device *pipeline = kunit_kzalloc(test, sizeof(*pipeline), GFP_KERNEL);
	struct dcp_display_mode *copy;
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;
	unsigned int count;
	u64 generation;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, connector);
	KUNIT_ASSERT_NOT_NULL(test, other);
	KUNIT_ASSERT_NOT_NULL(test, pipeline);
	dcp_modes_init(dcp);
	dcp->dev = &pipeline->dev;
	connector->dcp = pipeline;
	observer->dcp = dcp;
	INIT_WORK(&connector->hotplug_wq, parser_catalog_ready);
	apple_connector_edid_init(connector);
	apple_connector_edid_init(other);
	dcp->connector = connector;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, dcp_modes_transfer_begin(dcp)), 0);
	copy = dcp_modes_dup(dcp, other, &count, &generation);
	KUNIT_EXPECT_EQ(test, PTR_ERR(copy), (long)-ENODATA);
	copy = dcp_modes_dup(dcp, connector, &count, &generation);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, copy);
	KUNIT_EXPECT_EQ(test, count, 1U);
	dcp_modes_begin_attachment(dcp);
	dcp_modes_adopt(dcp, connector, generation, copy, count);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);

	copy = kmemdup(dcp->modes, sizeof(*copy), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, copy);
	generation = dcp_modes_transfer_begin(dcp);
	dcp_modes_adopt(dcp, other, generation, copy, 1);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);
	copy = kmemdup(dcp->modes, sizeof(*copy), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, copy);
	dcp_modes_adopt(dcp, connector, generation, copy, 1);
	KUNIT_EXPECT_TRUE(test, dcp->modes_admitted);
	KUNIT_EXPECT_TRUE(test, dcp->modes_provisional);

	/* Stale and rejected replacement cannot announce readiness. */
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, generation - 1), -ESTALE);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, observer->notifications, 0U);
	record.score = -1;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, generation), -EINVAL);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, observer->notifications, 0U);
	record.score = 10;
	record.id = 9;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, generation), 0);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, observer->notifications, 1U);
	KUNIT_EXPECT_NE(test, observer->reasons, 0U);
	KUNIT_EXPECT_FALSE(test, observer->locked);
	KUNIT_EXPECT_FALSE(test, dcp->modes_provisional);
	KUNIT_EXPECT_EQ(test, dcp->modes[0].timing_mode_id, 9U);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, generation), 0);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, observer->notifications, 1U);
	/* A provisional transition for a connector owned elsewhere sends no READY. */
	connector->dcp = NULL;
	dcp->modes_provisional = true;
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, generation), 0);
	flush_work(&connector->hotplug_wq);
	KUNIT_EXPECT_EQ(test, observer->notifications, 1U);
	connector->dcp = pipeline;
	/* A copy arriving after the destination's catalog cannot replace its IDs. */
	copy = kmemdup(dcp->modes, sizeof(*copy), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, copy);
	copy->timing_mode_id = 7;
	dcp_modes_adopt(dcp, connector, generation, copy, 1);
	KUNIT_EXPECT_EQ(test, dcp->modes[0].timing_mode_id, 9U);
	kfree(dcp->modes);
}

static const struct drm_connector_funcs parser_route_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void parser_route_programming_guard(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_crtc *crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	struct platform_device *pipeline = kunit_kzalloc(test, sizeof(*pipeline), GFP_KERNEL);
	struct platform_device *other = kunit_kzalloc(test, sizeof(*other), GFP_KERNEL);
	struct apple_connector *connector = kunit_kzalloc(test, sizeof(*connector), GFP_KERNEL);
	struct drm_connector_state conn_state = {};
	struct drm_crtc_state crtc_state = {};
	struct __drm_crtcs_state crtcs[1] = { { .new_state = &crtc_state } };
	struct __drm_connnectors_state connections[1] = {};
	struct drm_atomic_state state = {
		.connectors = connections, .num_connector = 1, .crtcs = crtcs,
	};
	struct drm_encoder encoder = {};
	struct device *dev = drm_kunit_helper_alloc_device(test);
	struct drm_device *drm;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	drm = __drm_kunit_helper_alloc_drm_device(test, dev, sizeof(*drm), 0,
						  DRIVER_MODESET | DRIVER_ATOMIC);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, drm);
	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, crtc);
	KUNIT_ASSERT_NOT_NULL(test, pipeline);
	KUNIT_ASSERT_NOT_NULL(test, other);
	KUNIT_ASSERT_NOT_NULL(test, connector);
	ret = drmm_connector_init(drm, &connector->base, &parser_route_connector_funcs,
				  DRM_MODE_CONNECTOR_DisplayPort, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	crtc->base.dev = drm;
	drm_atomic_helper_connector_reset(&connector->base);
	KUNIT_ASSERT_NOT_NULL(test, connector->base.state);
	connector->base.state->crtc = &crtc->base;
	crtc_state.connector_mask = drm_connector_mask(&connector->base);
	dcp_modes_init(dcp);
	dcp->modes_admitted = true;
	dcp->fw_compat = DCP_FIRMWARE_V_13_5;
	dcp->nr_typec_routes = 1;
	dcp->connector = connector;
	crtc->dcp = pipeline;
	platform_set_drvdata(pipeline, dcp);
	connector->port_encoder = &encoder;
	connector->dcp = other;
	connector->connected = true;
	conn_state.crtc = &crtc->base;
	connections[0].ptr = &connector->base;
	connections[0].new_state = &conn_state;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, false));
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	connector->dcp = pipeline;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_can_retrain(&crtc->base, connector));
	connector->dcp = other;
	platform_set_drvdata(other, dcp);
	KUNIT_EXPECT_FALSE(test, dcp_crtc_can_retrain(&crtc->base, connector));
	connector->dcp = pipeline;
	dcp->modes_provisional = true;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, false));
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	dcp->modes_provisional = false;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	/* A plane-only commit omits connector entries but keeps its complete mask. */
	state.num_connector = 0;
	crtc_state.connector_mask = BIT_ULL(1);
	dcp->active_typec_route = &dcp->typec_routes[0];
	dcp->typec_crtc_off = true;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_needs_route_start(dcp));
	dcp->typec_follow_start = true;
	dcp->typec_generation = 5;
	dcp->typec_follow_gen = 5;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_needs_route_start(dcp));
	dcp->typec_follow_gen = 4;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_needs_route_start(dcp));
	dcp->typec_follow_gen = 5;
	dcp->active_typec_route->tunnel = true;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_needs_route_start(dcp));
	dcp->active_typec_route->tunnel = false;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	crtc_state.connector_mask = 0;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	crtc_state.connector_mask = BIT_ULL(0);
	dcp->modes_admitted = false;
	dcp->typec_follow_start = false;
	/* Paired reattachment keeps baseline power-on while lookup rejects modes. */
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	dcp->typec_follow_start = true;
	/* A current handoff must acquire its own admitted destination catalog. */
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	dcp->modes_admitted = true;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	dcp->active_typec_route = NULL;
	dcp->typec_follow_start = false;
	connector->dcp = other;
	KUNIT_EXPECT_FALSE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	connector->dcp = pipeline;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	state.num_connector = 1;
	/* An unrouted disconnected resume keeps its existing deferred path. */
	connector->dcp = NULL;
	connector->connected = false;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, false));
	/* Native/14.7 continues to use its existing atomic machinery. */
	dcp->fw_compat = DCP_FIRMWARE_V_14_7;
	connector->dcp = other;
	KUNIT_EXPECT_TRUE(test, dcp_crtc_route_ready(&crtc->base, &state, true));
	connector->base.state->crtc = NULL;
}

static void parser_route_invalid_own_mode(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_crtc *crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	struct platform_device *pdev = kunit_kzalloc(test, sizeof(*pdev), GFP_KERNEL);
	struct drm_crtc_state crtc_state = { .enable = true, .mode_changed = true };
	struct __drm_crtcs_state crtcs[1] = { { .new_state = &crtc_state } };
	struct drm_atomic_state state = { .crtcs = crtcs };

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, crtc);
	KUNIT_ASSERT_NOT_NULL(test, pdev);
	dcp_modes_init(dcp);
	dcp->fw_compat = DCP_FIRMWARE_V_13_5;
	dcp->nr_typec_routes = 1;
	crtc->dcp = pdev;
	platform_set_drvdata(pdev, dcp);
	/* No crossing selected: a fixed/paired output still needs its own mode. */
	KUNIT_EXPECT_TRUE(test, dcp_crtc_mode_fixup(&crtc->base, &crtc_state.mode, NULL));
	KUNIT_EXPECT_EQ(test, dcp_crtc_atomic_check(&crtc->base, &state), -EINVAL);
}

static void parser_connector_ownership(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_connector *fixed = kunit_kzalloc(test, sizeof(*fixed), GFP_KERNEL);
	struct apple_connector *borrowed = kunit_kzalloc(test, sizeof(*borrowed), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, fixed);
	KUNIT_ASSERT_NOT_NULL(test, borrowed);
	dcp_modes_init(dcp);
	mutex_lock(&dcp->modes_lock);
	dcp->connector = borrowed;
	dcp->modes_admitted = true;
	/* Fixed outputs retain their pipeline even when probing is forced. */
	fixed->base.force = DRM_FORCE_ON;
	KUNIT_EXPECT_FALSE(test, dcp_modes_for_connector(dcp, fixed));
	KUNIT_EXPECT_TRUE(test, dcp_modes_for_connector(dcp, borrowed));
	dcp->connector = fixed;
	dcp->modes_admitted = false;
	KUNIT_EXPECT_FALSE(test, dcp_modes_for_connector(dcp, fixed));
	dcp->modes_admitted = true;
	KUNIT_EXPECT_TRUE(test, dcp_modes_for_connector(dcp, fixed));
	KUNIT_EXPECT_FALSE(test, dcp_modes_for_connector(dcp, borrowed));
	mutex_unlock(&dcp->modes_lock);
}

static void parser_route_retirement(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_connector *fixed = kunit_kzalloc(test, sizeof(*fixed), GFP_KERNEL);
	struct apple_dcp_typec_route *route = kunit_kzalloc(test, 1, GFP_KERNEL);
	struct apple_dcp_typec_route *other = kunit_kzalloc(test, 1, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, fixed);
	KUNIT_ASSERT_NOT_NULL(test, route);
	KUNIT_ASSERT_NOT_NULL(test, other);
	dcp_modes_init(dcp);
	dcp->connector = fixed;
	dcp->active_typec_route = route;
	dcp->modes_admitted = true;
	KUNIT_EXPECT_FALSE(test, dcp_modes_end_typec(dcp, other));
	KUNIT_EXPECT_EQ(test, dcp_modes_transfer_begin(dcp), 0ULL);
	KUNIT_EXPECT_TRUE(test, dcp_modes_end_typec(dcp, route));
	KUNIT_EXPECT_EQ(test, dcp_modes_transfer_begin(dcp), 1ULL);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);
	dcp->active_typec_route = NULL;
	KUNIT_EXPECT_FALSE(test, dcp_modes_end_typec(dcp, route));
	KUNIT_EXPECT_EQ(test, dcp_modes_transfer_begin(dcp), 1ULL);
	dcp_modes_release(dcp);
}

static void parser_rejected_replacement(struct kunit *test)
{
	struct parser_record record = valid_record(7, 10);
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct dcp_display_mode *previous;
	struct dcp_parse_ctx ctx;
	struct parser_blob *blob;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	dcp_modes_init(dcp);
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, 0), 0);
	previous = dcp->modes;
	record.missing = MISSING_SCORE;
	blob = make_blob(test, &record, 1);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, 0), -EINVAL);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, previous);
	KUNIT_EXPECT_EQ(test, dcp->nr_modes, 1U);
	KUNIT_EXPECT_TRUE(test, dcp->modes_admitted);
	dcp_modes_begin_attachment(dcp);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_EXPECT_EQ(test, dcp_modes_replace(dcp, &ctx, 1), -EINVAL);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);
	blob = make_blob(test, &record, 0);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	ctx.dcp = &parser_dcp;
	KUNIT_ASSERT_EQ(test, dcp_modes_replace(dcp, &ctx, 1), 0);
	KUNIT_EXPECT_EQ(test, dcp->nr_modes, 0U);
	KUNIT_EXPECT_TRUE(test, dcp->modes_admitted);
	dcp_modes_release(dcp);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, NULL);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);
}

static void parser_attribute_epoch(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct parser_blob *blob = kunit_kzalloc(test, sizeof(*blob), GFP_KERNEL);
	struct dcp_parse_ctx ctx;
	u64 generation;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	dcp_modes_init(dcp);
	put_unaligned_le32(0xd3, blob->data);
	blob->size = 4;
	blob_tag(blob, 1, 3);
	blob_int(blob, "MaxHorizontalImageSize", 52);
	blob_int(blob, "MaxVerticalImageSize", 29);
	blob_bool(blob, "SupportsBacklightControl", true);
	generation = dcp_modes_transfer_begin(dcp);
	dcp_modes_begin_attachment(dcp);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_EXPECT_EQ(test, dcp_attributes_replace(dcp, &ctx, generation), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 0);
	KUNIT_EXPECT_FALSE(test, dcp->ext_backlight);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_ASSERT_EQ(test, dcp_attributes_replace(dcp, &ctx, 1), 0);
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 520);
	KUNIT_EXPECT_EQ(test, dcp->height_mm, 290);
	KUNIT_EXPECT_TRUE(test, dcp->ext_backlight);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_EXPECT_EQ(test, dcp_attributes_replace(dcp, &ctx, generation), -ESTALE);
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 520);
	dcp_modes_release(dcp);
}

static void parser_attributes_malformed(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct parser_blob *blob = kunit_kzalloc(test, sizeof(*blob), GFP_KERNEL);
	struct dcp_parse_ctx ctx;
	const s64 invalid[] = { -1, INT_MAX / 10 + 1, S64_MAX };
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	dcp_modes_init(dcp);
	dcp->width_mm = 520;
	dcp->height_mm = 290;
	dcp->ext_backlight = true;
	put_unaligned_le32(0xd3, blob->data);
	blob->size = 4;
	blob_tag(blob, 2, 0);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_EXPECT_LT(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	KUNIT_ASSERT_EQ(test, parse(blob->data, 4, &ctx), 0);
	KUNIT_EXPECT_LT(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	blob->size = 4;
	blob_tag(blob, 1, 1);
	blob_int(blob, "Unknown", 1);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size - 1, &ctx), 0);
	KUNIT_EXPECT_LT(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	for (i = 0; i < ARRAY_SIZE(invalid); ++i) {
		blob->size = 4;
		blob_tag(blob, 1, 1);
		blob_int(blob, "MaxHorizontalImageSize", invalid[i]);
		KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
		KUNIT_EXPECT_LT(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	}
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 520);
	KUNIT_EXPECT_EQ(test, dcp->height_mm, 290);
	KUNIT_EXPECT_TRUE(test, dcp->ext_backlight);
	dcp_modes_release(dcp);
}

static void parser_optional_flag_extent(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct parser_blob *blob = kunit_kzalloc(test, sizeof(*blob), GFP_KERNEL);
	struct dcp_parse_ctx ctx;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	dcp_modes_init(dcp);
	put_unaligned_le32(0xd3, blob->data);
	blob->size = 4;
	blob_tag(blob, 1, 3);
	blob_int(blob, "MaxHorizontalImageSize", 52);
	blob_int(blob, "MaxVerticalImageSize", 29);
	blob_bool(blob, "SupportsBacklightControl", true);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_ASSERT_EQ(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	KUNIT_EXPECT_TRUE(test, dcp->ext_backlight);

	blob->size = 4;
	blob_tag(blob, 1, 3);
	blob_int(blob, "MaxHorizontalImageSize", 60);
	blob_int(blob, "MaxVerticalImageSize", 40);
	blob_key(blob, "SupportsBacklightControl");
	blob_tag(blob, 9, 2);
	blob->data[blob->size++] = 'x';
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_EXPECT_LT(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 520);
	KUNIT_EXPECT_EQ(test, dcp->height_mm, 290);
	KUNIT_EXPECT_TRUE(test, dcp->ext_backlight);

	/* A complete value of another type keeps the optional false fallback. */
	blob->data[blob->size++] = 'y';
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	KUNIT_ASSERT_EQ(test, dcp_attributes_replace(dcp, &ctx, 0), 0);
	KUNIT_EXPECT_EQ(test, dcp->width_mm, 600);
	KUNIT_EXPECT_EQ(test, dcp->height_mm, 400);
	KUNIT_EXPECT_FALSE(test, dcp->ext_backlight);
	dcp_modes_release(dcp);
}

static void parser_attribute_deferred(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_connector *connector = kunit_kzalloc(test, sizeof(*connector), GFP_KERNEL);
	struct drm_device *dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	struct parser_blob *blob = kunit_kzalloc(test, sizeof(*blob), GFP_KERNEL);
	struct dcp_parse_ctx ctx;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, connector);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, blob);
	dcp_modes_init(dcp);
	mutex_init(&dev->mode_config.mutex);
	connector->base.dev = dev;
	dcp->connector = connector;
	dcp->panel.width_mm = 600;
	dcp->modes = kmalloc_obj(*dcp->modes);
	KUNIT_ASSERT_NOT_NULL(test, dcp->modes);
	dcp->nr_modes = 1;
	put_unaligned_le32(0xd3, blob->data);
	blob->size = 4;
	blob_tag(blob, 1, 2);
	blob_int(blob, "MaxHorizontalImageSize", 52);
	blob_int(blob, "MaxVerticalImageSize", 29);
	KUNIT_ASSERT_EQ(test, parse(blob->data, blob->size, &ctx), 0);
	enable_work(&dcp->dimensions_wq);
	/* RX must return while a probe holds the DRM lock awaiting a reply. */
	mutex_lock(&dev->mode_config.mutex);
	ret = dcp_attributes_replace(dcp, &ctx, 0);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, dcp->modes[0].mode.width_mm, 520);
	mutex_unlock(&dev->mode_config.mutex);
	flush_work(&dcp->dimensions_wq);
	KUNIT_EXPECT_EQ(test, connector->base.display_info.width_mm, 520U);
	KUNIT_EXPECT_EQ(test, connector->base.display_info.height_mm, 290U);
	dcp_modes_release(dcp);
}

static void parser_dimensions_lifetime(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct apple_connector *connector = kunit_kzalloc(test, sizeof(*connector), GFP_KERNEL);
	struct drm_device *dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, connector);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	dcp_modes_init(dcp);
	mutex_init(&dev->mode_config.mutex);
	connector->base.dev = dev;
	dcp->connector = connector;
	dcp->width_mm = 520;
	dcp->height_mm = 290;
	KUNIT_EXPECT_FALSE(test, schedule_work(&dcp->dimensions_wq));
	KUNIT_EXPECT_EQ(test, connector->base.display_info.width_mm, 0U);
	enable_work(&dcp->dimensions_wq);
	schedule_work(&dcp->dimensions_wq);
	flush_work(&dcp->dimensions_wq);
	KUNIT_EXPECT_EQ(test, connector->base.display_info.width_mm, 520U);
	disable_work_sync(&dcp->dimensions_wq);
	dcp->width_mm = 600;
	KUNIT_EXPECT_FALSE(test, schedule_work(&dcp->dimensions_wq));
	KUNIT_EXPECT_EQ(test, connector->base.display_info.width_mm, 520U);
	enable_work(&dcp->dimensions_wq);
	schedule_work(&dcp->dimensions_wq);
	flush_work(&dcp->dimensions_wq);
	KUNIT_EXPECT_EQ(test, connector->base.display_info.width_mm, 600U);
	dcp_modes_release(dcp);
	KUNIT_EXPECT_FALSE(test, schedule_work(&dcp->dimensions_wq));
}

static void parser_catalog_release(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	dcp_modes_init(dcp);
	dcp->modes = kmalloc_obj(*dcp->modes);
	KUNIT_ASSERT_NOT_NULL(test, dcp->modes);
	dcp->nr_modes = 1;
	dcp->modes_admitted = true;
	dcp_modes_release(dcp);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, NULL);
	KUNIT_EXPECT_EQ(test, dcp->nr_modes, 0U);
	KUNIT_EXPECT_FALSE(test, dcp->modes_admitted);
	dcp_modes_release(dcp);
	KUNIT_EXPECT_PTR_EQ(test, dcp->modes, NULL);
}

static void parser_property_bounds(struct kunit *test)
{
	char key[64];

	memset(key, 'x', sizeof(key));
	KUNIT_EXPECT_FALSE(test, dcp_property_key_valid(key, sizeof(key)));
	key[63] = '\0';
	KUNIT_EXPECT_TRUE(test, dcp_property_key_valid(key, sizeof(key)));

	KUNIT_EXPECT_FALSE(test, dcp_property_size_valid(0));
	KUNIT_EXPECT_TRUE(test, dcp_property_size_valid(SZ_16M));
	KUNIT_EXPECT_FALSE(test, dcp_property_size_valid(SZ_16M + 1));
	KUNIT_EXPECT_TRUE(test, dcp_property_chunk_valid(8192, 4096, 4096, 4096));
	KUNIT_EXPECT_FALSE(test, dcp_property_chunk_valid(8192, 0, 4097, 4096));
	KUNIT_EXPECT_FALSE(test, dcp_property_chunk_valid(8192, U32_MAX - 2, 4, 4096));
	KUNIT_EXPECT_FALSE(test, dcp_property_chunk_valid(8192, 8190, 4, 4096));
}

static void parser_route_holder_snapshot(struct kunit *test)
{
	struct apple_dcp *back = kunit_kzalloc(test, sizeof(*back), GFP_KERNEL);
	struct apple_connector *holder = kunit_kzalloc(test, sizeof(*holder), GFP_KERNEL);
	struct drm_crtc *target = kunit_kzalloc(test, sizeof(*target), GFP_KERNEL);
	struct drm_crtc *other = kunit_kzalloc(test, sizeof(*other), GFP_KERNEL);
	struct drm_connector_state live = { .crtc = target };
	struct drm_connector_state reserved = {};
	struct __drm_connnectors_state conns[1] = {};
	struct drm_atomic_state state = { .connectors = conns, .num_connector = 1 };

	KUNIT_ASSERT_NOT_NULL(test, back);
	KUNIT_ASSERT_NOT_NULL(test, holder);
	KUNIT_ASSERT_NOT_NULL(test, target);
	KUNIT_ASSERT_NOT_NULL(test, other);
	holder->base.state = &live;
	/* An absent holder snapshot is refused even if live state looks inactive. */
	KUNIT_EXPECT_FALSE(test, dcp_typec_follow_holder_off(&state, target, holder, back));
	conns[0].new_state = &reserved;
	/* The reserved off holder is retained despite a future live assignment. */
	KUNIT_EXPECT_TRUE(test, dcp_typec_follow_holder_off(&state, target, holder, back));
	reserved.crtc = target;
	live.crtc = NULL;
	KUNIT_EXPECT_FALSE(test, dcp_typec_follow_holder_off(&state, target, holder, back));
	reserved.crtc = other;
	back->crtc = to_apple_crtc(other);
	KUNIT_EXPECT_TRUE(test, dcp_typec_follow_holder_off(&state, target, holder, back));
}

static void parser_route_prepare_scope(struct kunit *test)
{
	struct apple_dcp *dcp = kunit_kzalloc(test, sizeof(*dcp), GFP_KERNEL);
	struct device *dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	struct apple_dcp_typec_route *route = kunit_kzalloc(test, sizeof(*route), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, dcp);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, route);
	mutex_init(&dcp->modes_lock);
	dcp->dev = dev;
	dcp->fw_compat = DCP_FIRMWARE_V_13_5;
	dcp->nr_typec_routes = 1;
	dcp->active_typec_route = route;
	dcp->typec_crtc_off = true;
	/* No PHY would fail strict preparation; a paired wake must bypass it. */
	KUNIT_EXPECT_EQ(test, dcp_typec_prepare_route(dcp), 0);
	dcp->typec_follow_start = true;
	dcp->typec_follow_gen = dcp->typec_generation;
	mutex_init(&dcp->hpd_mutex);
	KUNIT_EXPECT_EQ(test, dcp_typec_prepare_route(dcp), -ENODEV);
	mutex_destroy(&dcp->hpd_mutex);
	mutex_destroy(&dcp->modes_lock);
}

static struct kunit_case parser_cases[] = {
	KUNIT_CASE(parser_score_presence),
	KUNIT_CASE(parser_legacy_ties),
	KUNIT_CASE(parser_color_presence),
	KUNIT_CASE(parser_geometry),
	KUNIT_CASE(parser_bad_geometry),
	KUNIT_CASE(parser_horizontal_rate),
	KUNIT_CASE(parser_bad_score),
	KUNIT_CASE(parser_notch_vrr),
	KUNIT_CASE(parser_replacement),
	KUNIT_CASE(parser_held_keeps_edid),
	KUNIT_CASE(parser_attachment_admission),
	KUNIT_CASE(parser_stale_transfer),
	KUNIT_CASE(parser_route_catalog_handoff),
	KUNIT_CASE(parser_route_programming_guard),
	KUNIT_CASE(parser_route_prepare_scope),
	KUNIT_CASE(parser_route_holder_snapshot),
	KUNIT_CASE(parser_route_invalid_own_mode),
	KUNIT_CASE(parser_connector_ownership),
	KUNIT_CASE(parser_route_retirement),
	KUNIT_CASE(parser_rejected_replacement),
	KUNIT_CASE(parser_attribute_epoch),
	KUNIT_CASE(parser_attributes_malformed),
	KUNIT_CASE(parser_optional_flag_extent),
	KUNIT_CASE(parser_attribute_deferred),
	KUNIT_CASE(parser_dimensions_lifetime),
	KUNIT_CASE(parser_catalog_release),
	KUNIT_CASE(parser_property_bounds),
	{}
};

static struct kunit_suite parser_suite = {
	.name = "apple-display-parser",
	.test_cases = parser_cases,
};

kunit_test_suite(parser_suite);
