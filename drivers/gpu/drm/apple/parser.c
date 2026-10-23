// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#include <kunit/visibility.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/module.h>

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
#include <sound/pcm.h> // for sound format masks
#endif

#include "parser.h"
#include "trace.h"

#define DCP_PARSE_HEADER 0xd3

enum dcp_parse_type {
	DCP_TYPE_DICTIONARY = 1,
	DCP_TYPE_ARRAY = 2,
	DCP_TYPE_INT64 = 4,
	DCP_TYPE_STRING = 9,
	DCP_TYPE_BLOB = 10,
	DCP_TYPE_BOOL = 11
};

struct dcp_parse_tag {
	unsigned int size : 24;
	enum dcp_parse_type type : 5;
	unsigned int padding : 2;
	bool last : 1;
} __packed;

static const void *parse_bytes(struct dcp_parse_ctx *ctx, size_t count)
{
	const void *ptr;

	if (ctx->pos > ctx->len || count > ctx->len - ctx->pos)
		return ERR_PTR(-EINVAL);

	ptr = ctx->blob + ctx->pos;
	ctx->pos += count;
	return ptr;
}

static const u32 *parse_u32(struct dcp_parse_ctx *ctx)
{
	return parse_bytes(ctx, sizeof(u32));
}

static const struct dcp_parse_tag *parse_tag(struct dcp_parse_ctx *ctx)
{
	const struct dcp_parse_tag *tag;

	/* Align to 32-bits */
	if (ctx->pos > U32_MAX - 3)
		return ERR_PTR(-EINVAL);
	ctx->pos = round_up(ctx->pos, 4);

	tag = parse_bytes(ctx, sizeof(struct dcp_parse_tag));

	if (IS_ERR(tag))
		return tag;

	if (tag->padding)
		return ERR_PTR(-EINVAL);

	return tag;
}

static const struct dcp_parse_tag *parse_tag_of_type(struct dcp_parse_ctx *ctx,
					       enum dcp_parse_type type)
{
	const struct dcp_parse_tag *tag = parse_tag(ctx);

	if (IS_ERR(tag))
		return tag;

	if (tag->type != type)
		return ERR_PTR(-EINVAL);

	return tag;
}

/* Bound stack use when ignoring nested firmware dictionaries. */
#define DCP_PARSE_MAX_DEPTH 32

static int skip_value(struct dcp_parse_ctx *handle, unsigned int depth)
{
	const struct dcp_parse_tag *tag;
	const void *data;
	unsigned int count, i;
	int ret;

	if (depth >= DCP_PARSE_MAX_DEPTH)
		return -EINVAL;

	tag = parse_tag(handle);
	if (IS_ERR(tag))
		return PTR_ERR(tag);

	switch (tag->type) {
	case DCP_TYPE_DICTIONARY:
	case DCP_TYPE_ARRAY:
		count = tag->size;
		if (tag->type == DCP_TYPE_DICTIONARY)
			count *= 2;

		for (i = 0; i < count; ++i) {
			ret = skip_value(handle, depth + 1);
			if (ret)
				return ret;
		}
		return 0;

	case DCP_TYPE_INT64:
		data = parse_bytes(handle, sizeof(s64));
		break;

	case DCP_TYPE_STRING:
	case DCP_TYPE_BLOB:
		data = parse_bytes(handle, tag->size);
		break;

	case DCP_TYPE_BOOL:
		return 0;

	default:
		return -EINVAL;
	}

	return IS_ERR(data) ? PTR_ERR(data) : 0;
}

static int skip(struct dcp_parse_ctx *handle)
{
	return skip_value(handle, 0);
}

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
static int skip_pair(struct dcp_parse_ctx *handle)
{
	int ret;

	ret = skip(handle);
	if (ret)
		return ret;

	return skip(handle);
}

static bool consume_string(struct dcp_parse_ctx *ctx, const char *specimen)
{
	struct dcp_parse_ctx probe = *ctx;
	const struct dcp_parse_tag *tag;
	const char *key;

	tag = parse_tag_of_type(&probe, DCP_TYPE_STRING);
	if (IS_ERR(tag) || tag->size != strlen(specimen))
		return false;

	key = parse_bytes(&probe, tag->size);
	if (IS_ERR(key) || memcmp(key, specimen, tag->size))
		return false;

	*ctx = probe;
	return true;
}
#endif

/* Caller must free the result */
static char *parse_string(struct dcp_parse_ctx *handle)
{
	const struct dcp_parse_tag *tag = parse_tag_of_type(handle, DCP_TYPE_STRING);
	const char *in;
	char *out;

	if (IS_ERR(tag))
		return (void *)tag;

	in = parse_bytes(handle, tag->size);
	if (IS_ERR(in))
		return (void *)in;

	out = kmalloc(tag->size + 1, GFP_KERNEL);
	if (!out)
		return ERR_PTR(-ENOMEM);

	memcpy(out, in, tag->size);
	out[tag->size] = '\0';
	return out;
}

static int parse_int_bound(struct dcp_parse_ctx *handle, s64 *value, s64 min, s64 max)
{
	const void *tag = parse_tag_of_type(handle, DCP_TYPE_INT64);
	const void *in;

	if (IS_ERR(tag))
		return PTR_ERR(tag);

	in = parse_bytes(handle, sizeof(s64));

	if (IS_ERR(in))
		return PTR_ERR(in);

	memcpy(value, in, sizeof(*value));

	if (*value < min || *value > max)
		return -EINVAL;

	return 0;
}

static int parse_int(struct dcp_parse_ctx *handle, s64 *value)
{
	return parse_int_bound(handle, value, S64_MIN, S64_MAX);
}

static int parse_bool(struct dcp_parse_ctx *handle, bool *b)
{
	const struct dcp_parse_tag *tag = parse_tag_of_type(handle, DCP_TYPE_BOOL);

	if (IS_ERR(tag))
		return PTR_ERR(tag);

	*b = !!tag->size;
	return 0;
}

/*
 * For an optional flag: a value of another type reads as false instead of
 * failing the whole dictionary, which the key did not do before it was read.
 */
static int parse_bool_or_skip(struct dcp_parse_ctx *handle, bool *b)
{
	struct dcp_parse_ctx at = *handle;

	if (!parse_bool(handle, b))
		return 0;

	*handle = at;
	*b = false;
	return skip(handle);
}

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
static int parse_blob(struct dcp_parse_ctx *handle, size_t size, u8 const **blob)
{
	const struct dcp_parse_tag *tag = parse_tag_of_type(handle, DCP_TYPE_BLOB);
	const u8 *out;

	if (IS_ERR(tag))
		return PTR_ERR(tag);

	if (tag->size < size)
		return -EINVAL;

	out = parse_bytes(handle, tag->size);

	if (IS_ERR(out))
		return PTR_ERR(out);

	*blob = out;
	return 0;
}
#endif

struct iterator {
	struct dcp_parse_ctx *handle;
	u32 idx, len;
};

static int iterator_begin(struct dcp_parse_ctx *handle, struct iterator *it,
			  bool dict)
{
	const struct dcp_parse_tag *tag;
	enum dcp_parse_type type = dict ? DCP_TYPE_DICTIONARY : DCP_TYPE_ARRAY;

	*it = (struct iterator) {
		.handle = handle,
		.idx = 0
	};

	tag = parse_tag_of_type(it->handle, type);
	if (IS_ERR(tag))
		return PTR_ERR(tag);

	it->len = tag->size;
	return 0;
}

int parse(const void *blob, size_t size, struct dcp_parse_ctx *ctx)
{
	const u32 *header;

	if (size > U32_MAX)
		return -EINVAL;

	*ctx = (struct dcp_parse_ctx) {
		.blob = blob,
		.len = size,
		.pos = 0,
	};

	header = parse_u32(ctx);
	if (IS_ERR(header))
		return PTR_ERR(header);

	if (*header != DCP_PARSE_HEADER)
		return -EINVAL;

	return 0;
}

bool dcp_property_complete(const void *blob, size_t received, size_t advertised)
{
	struct dcp_parse_ctx ctx;

	if (!blob || !received || received > U32_MAX - 3 ||
	    received > advertised || advertised - received > 1)
		return false;
	/* H17P may count one unsent byte after the aligned serialized object. */
	if (advertised != received && (received & 3))
		return false;
	/* Bound decoding to bytes actually delivered, never allocation padding. */
	if (parse(blob, received, &ctx) || skip(&ctx))
		return false;
	return ctx.pos == received || round_up(ctx.pos, 4) == received;
}

static int parse_dimension(struct dcp_parse_ctx *handle, struct dimension *dim, bool polarity)
{
	static const char * const fields[] = { "Active", "Total", "FrontPorch", "SyncWidth", "PreciseSyncRate" };
	unsigned int seen = 0;
	struct iterator it;
	int ret = 0;

	*dim = (struct dimension) {
		.active = -1,
		.total = -1,
		.front_porch = -1,
		.sync_width = -1,
		.precise_sync_rate = -1,
		.sync_polarity = -1,
		.pixel_repetition = -1,
	};

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		char *key = parse_string(it.handle);
		unsigned int field;

		if (!IS_ERR(key) && handle->automatic_1080p180_rgb8)
			for (field = 0; field < 5; field++)
				if (!strcmp(key, fields[field])) {
					handle->ambiguous_180hz |= !!(seen & (1U << field));
					seen |= 1U << field;
				}

		if (IS_ERR(key))
			ret = PTR_ERR(key);
		else if (!strcmp(key, "Active"))
			ret = parse_int_bound(it.handle, &dim->active, 0, U16_MAX);
		else if (!strcmp(key, "Total"))
			ret = parse_int_bound(it.handle, &dim->total, 0, U16_MAX);
		else if (!strcmp(key, "FrontPorch"))
			ret = parse_int_bound(it.handle, &dim->front_porch, 0, U16_MAX);
		else if (!strcmp(key, "SyncWidth"))
			ret = parse_int_bound(it.handle, &dim->sync_width, 0, U16_MAX);
		else if (!strcmp(key, "PreciseSyncRate"))
			/* Horizontal may be 0; parse_mode() requires the vertical rate. */
			ret = parse_int_bound(it.handle, &dim->precise_sync_rate, 0, U32_MAX);
		else if (polarity && !strcmp(key, "SyncPolarity")) {
			if (dim->sync_polarity >= 0)
				ret = -EINVAL;
			else
				ret = parse_int_bound(it.handle, &dim->sync_polarity, 0, 1);
		} else if (polarity && !strcmp(key, "PixelRepetition")) {
			if (dim->pixel_repetition >= 0)
				ret = -EINVAL;
			else
				ret = parse_int_bound(it.handle, &dim->pixel_repetition,
						      0, U32_MAX);
		}
		else
			ret = skip(it.handle);

		if (!IS_ERR_OR_NULL(key))
			kfree(key);

		if (ret)
			return ret;
	}

	if (dim->active <= 0 || dim->total < dim->active ||
	    dim->front_porch < 0 || dim->sync_width < 0 ||
	    dim->front_porch + dim->sync_width > dim->total - dim->active)
		return -EINVAL;

	return 0;
}

struct color_mode {
	s64 colorimetry;
	s64 depth;
	s64 dynamic_range;
	s64 eotf;
	s64 id;
	s64 pixel_encoding;
	s64 score;
};

static int fill_color_mode(struct dcp_color_mode *color,
			   struct color_mode *cmode)
{
	if (cmode->id < 0 || cmode->id > U32_MAX)
		return -EINVAL;

	if (color->score >= cmode->score)
		return 0;

	if (cmode->colorimetry < 0 || cmode->colorimetry >= DCP_COLORIMETRY_COUNT)
		return -EINVAL;
	if (cmode->depth < 8 || cmode->depth > 12)
		return -EINVAL;
	if (cmode->dynamic_range < 0 || cmode->dynamic_range >= DCP_COLOR_YCBCR_RANGE_COUNT)
		return -EINVAL;
	if (cmode->eotf < 0 || cmode->eotf >= DCP_EOTF_COUNT)
		return -EINVAL;
	if (cmode->pixel_encoding < 0 || cmode->pixel_encoding >= DCP_COLOR_FORMAT_COUNT)
		return -EINVAL;

	color->score = cmode->score;
	color->id = cmode->id;
	color->eotf = cmode->eotf;
	color->format = cmode->pixel_encoding;
	color->colorimetry = cmode->colorimetry;
	color->range = cmode->dynamic_range;
	color->depth = cmode->depth;

	return 0;
}

/*
 * Colorimetry the SDR RGB link to an external display should prefer. By
 * firmware score alone DCP picks the widest colorimetry the display offers
 * (BT.2020 RGB on a Studio Display XDR), which puts the desktop's sRGB pixels
 * on a BT.2020 link. -1 keeps the firmware score; otherwise a dcp_colorimetry
 * value: 10 sRGB, 1 BT.709, 14 DCI-P3 D65, 16 default RGB, 9 BT.2020 RGB.
 * Read when a display's modes are parsed, so a change needs a replug.
 */
static int external_colorimetry = DCP_COLORIMETRY_SRGB;
module_param(external_colorimetry, int, 0644);
MODULE_PARM_DESC(external_colorimetry,
		 "Preferred SDR RGB link colorimetry for external displays (-1: firmware score, 10: sRGB, 1: BT.709, 14: DCI-P3 D65, 9: BT.2020 RGB)");

/* Tiers for external_colorimetry; equal tiers fall back to firmware score. */
static unsigned int external_colorimetry_rank(const struct color_mode *color)
{
	if (color->colorimetry == external_colorimetry)
		return 3;

	switch (color->colorimetry) {
	case DCP_COLORIMETRY_SRGB:
	case DCP_COLORIMETRY_BT709:
		return 2;
	case DCP_COLORIMETRY_RGB:
		return 1;
	default:
		return 0;
	}
}

/* External SDR scanout must not select a wide-gamut mode by score alone. */
static unsigned int external_sdr_rank(const struct color_mode *color)
{
	if (color->id < 0 || color->id > U32_MAX ||
	    color->eotf != DCP_EOTF_SDR_GAMMA ||
	    color->pixel_encoding != DCP_COLOR_FORMAT_RGB ||
	    color->dynamic_range != DCP_COLOR_YCBCR_RANGE_FULL ||
	    (color->depth != 8 && color->depth != 10))
		return 0;

	switch (color->colorimetry) {
	case DCP_COLORIMETRY_SRGB:
	case DCP_COLORIMETRY_BT709:
		return 2;
	case DCP_COLORIMETRY_RGB:
		return 1;
	default:
		return 0;
	}
}

/* Firmware lists color element IDs which this timing cannot safely use. */
#define DCP_TIMING_COLOR_ID_LIMIT 64

struct timing_color_ids {
	u32 ids[DCP_TIMING_COLOR_ID_LIMIT];
	unsigned int count;
};

struct timing_color_restrictions {
	struct timing_color_ids unsafe;
	struct timing_color_ids dsc_required;
};

static bool timing_color_contains(const struct timing_color_ids *ids, u32 id)
{
	unsigned int i;

	for (i = 0; i < ids->count; i++)
		if (ids->ids[i] == id)
			return true;
	return false;
}

static int parse_timing_color_ids(struct dcp_parse_ctx *handle,
				  struct timing_color_ids *ids)
{
	struct iterator it;
	int ret;

	ret = iterator_begin(handle, &it, false);
	if (ret)
		return ret;
	if (it.len > DCP_TIMING_COLOR_ID_LIMIT)
		return -E2BIG;
	for (; it.idx < it.len; it.idx++) {
		s64 id;

		ret = parse_int_bound(handle, &id, 0, U32_MAX);
		if (ret)
			return ret;
		if (timing_color_contains(ids, id))
			return -EINVAL;
		ids->ids[ids->count++] = id;
	}
	return 0;
}

static int parse_timing_color_restrictions(struct dcp_parse_ctx *handle,
					   struct timing_color_restrictions *limits)
{
	struct iterator it;
	bool unsafe_seen = false, dsc_seen = false, colors_seen = false;
	int ret;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;
	for (; it.idx < it.len; it.idx++) {
		char *key = parse_string(handle);

		if (IS_ERR(key))
			return PTR_ERR(key);
		if (!strcmp(key, "UnsafeColorElementIDs")) {
			ret = unsafe_seen ? -EINVAL :
				parse_timing_color_ids(handle, &limits->unsafe);
			unsafe_seen = true;
		} else if (!strcmp(key, "DSCRequiredColorElementIDs")) {
			ret = dsc_seen ? -EINVAL :
				parse_timing_color_ids(handle, &limits->dsc_required);
			dsc_seen = true;
		} else if (!strcmp(key, "ColorModes")) {
			ret = colors_seen ? -EINVAL : skip(handle);
			colors_seen = true;
		} else {
			ret = skip(handle);
		}
		kfree(key);
		if (ret)
			return ret;
	}
	return 0;
}

static int parse_color_modes(struct dcp_parse_ctx *handle,
			     struct dcp_display_mode *out, bool internal,
			     bool external_sdr,
			     const struct timing_color_restrictions *limits,
			     struct dcp_color_mode *unsafe_rgb8)
{
	struct iterator outer_it;
	unsigned int best_sdr_rank = 0, best_unsafe_rank = 0;
	bool prefer_colorimetry = !internal && !external_sdr &&
				  external_colorimetry >= 0;
	int ret = 0;
	out->sdr_444.score = -1;
	out->sdr_rgb.score = -1;
	out->sdr.score = -1;
	out->best.score = -1;

	ret = iterator_begin(handle, &outer_it, false);
	if (ret)
		return ret;

	for (; outer_it.idx < outer_it.len; ++outer_it.idx) {
		struct iterator it;
		static const char * const fields[] = { "Colorimetry", "Depth", "DynamicRange", "EOTF", "ID", "PixelEncoding", "Score", "IsVirtual" };
		unsigned int seen = 0;
		bool is_virtual = true;
		struct color_mode cmode = {
			.colorimetry = -1,
			.depth = -1,
			.dynamic_range = -1,
			.eotf = -1,
			.id = -1,
			.pixel_encoding = -1,
			.score = -1,
		};

		ret = iterator_begin(handle, &it, true);
		if (ret)
			return ret;

		for (; it.idx < it.len; ++it.idx) {
			char *key = parse_string(it.handle);
			unsigned int field;

			if (!IS_ERR(key) && handle->automatic_1080p180_rgb8)
				for (field = 0; field < 8; field++)
					if (!strcmp(key, fields[field])) {
						handle->ambiguous_180hz |= !!(seen & (1U << field));
						seen |= 1U << field;
					}

			if (IS_ERR(key))
				ret = PTR_ERR(key);
			else if (!strcmp(key, "Colorimetry"))
				ret = parse_int(it.handle, &cmode.colorimetry);
			else if (!strcmp(key, "Depth"))
				ret = parse_int(it.handle, &cmode.depth);
			else if (!strcmp(key, "DynamicRange"))
				ret = parse_int(it.handle, &cmode.dynamic_range);
			else if (!strcmp(key, "EOTF"))
				ret = parse_int(it.handle, &cmode.eotf);
			else if (!strcmp(key, "ID"))
				ret = parse_int(it.handle, &cmode.id);
			else if (!strcmp(key, "IsVirtual"))
				ret = parse_bool(it.handle, &is_virtual);
			else if (!strcmp(key, "PixelEncoding"))
				ret = parse_int(it.handle, &cmode.pixel_encoding);
			else if (!strcmp(key, "Score"))
				ret = parse_int(it.handle, &cmode.score);
			else
				ret = skip(it.handle);

			if (!IS_ERR_OR_NULL(key))
				kfree(key);

			if (ret)
				return ret;
		}

		/* Skip virtual or partial entries */
		if (is_virtual || cmode.score < 0 || cmode.id < 0 ||
		    cmode.id > U32_MAX)
			continue;

		trace_iomfb_color_mode(handle->dcp, cmode.id, cmode.score,
				       cmode.depth, cmode.colorimetry,
				       cmode.eotf, cmode.dynamic_range,
				       cmode.pixel_encoding);

		if (external_sdr) {
			unsigned int rank;

			if (cmode.id <= U32_MAX &&
			    timing_color_contains(&limits->dsc_required, cmode.id))
				continue;
			rank = external_sdr_rank(&cmode);
			if (cmode.id <= U32_MAX &&
			    timing_color_contains(&limits->unsafe, cmode.id)) {
				/*
				 * Collect only a timing-local bounded RGB8 candidate.
				 * Admission still requires current EDID and link proof.
				 */
				if (unsafe_rgb8 && cmode.depth == 8 && rank &&
				    cmode.colorimetry == DCP_COLORIMETRY_SRGB &&
				    rank >= best_unsafe_rank) {
					if (rank > best_unsafe_rank)
						unsafe_rgb8->score = -1;
					fill_color_mode(unsafe_rgb8, &cmode);
					best_unsafe_rank = rank;
				}
				continue;
			}
			if (!rank || rank < best_sdr_rank)
				continue;
			if (rank > best_sdr_rank)
				out->sdr_rgb.score = -1;
			fill_color_mode(&out->sdr_rgb, &cmode);
			best_sdr_rank = rank;
			continue;
		}

		if (cmode.eotf == DCP_EOTF_SDR_GAMMA) {
			if (cmode.pixel_encoding == DCP_COLOR_FORMAT_RGB &&
				cmode.depth <= 10) {
				unsigned int rank = prefer_colorimetry ?
					external_colorimetry_rank(&cmode) : 0;

				if (rank >= best_sdr_rank) {
					if (rank > best_sdr_rank)
						out->sdr_rgb.score = -1;
					fill_color_mode(&out->sdr_rgb, &cmode);
					best_sdr_rank = rank;
				}
			} else if (cmode.pixel_encoding == DCP_COLOR_FORMAT_YCBCR444 &&
				cmode.depth <= 10)
				fill_color_mode(&out->sdr_444, &cmode);
			fill_color_mode(&out->sdr, &cmode);
		}
		fill_color_mode(&out->best, &cmode);
	}

	return 0;
}

/*
 * Calculate the pixel clock for a mode given the 16:16 fixed-point refresh
 * rate. The pixel clock is the refresh rate times the pixel count. DRM
 * specifies the clock in kHz. The intermediate result may overflow a u32, so
 * use a u64 where required.
 */
static u64 calculate_clock(struct dimension *horiz, struct dimension *vert)
{
	u64 pixels = (u64)horiz->total * vert->total;
	u64 clock = pixels * vert->precise_sync_rate;

	return DIV_ROUND_CLOSEST_ULL(clock >> 16, 1000);
}

static int parse_mode(struct dcp_parse_ctx *handle,
		      struct dcp_display_mode *out, s64 *score, int width_mm,
		      int height_mm, unsigned notch_height, bool internal, bool external_sdr)
{
	static const char * const fields[] = {
		"ID", "Score", "MinimumVariableRefreshRate", "MaximumVariableRefreshRate",
	};
	unsigned int seen = 0;
	int ret = 0;
	struct iterator it;
	struct dimension horiz = { .active = -1 }, vert = { .active = -1 };
	struct timing_color_restrictions limits = {};
	struct dcp_color_mode unsafe_rgb8 = { .score = -1 };
	bool have_horiz = false, have_vert = false, have_virtual = false;
	bool duplicate_scope = false;
	s64 min_vrr = 0, max_vrr = 0;
	u64 clock;
	s64 id = -1;
	s64 best_color_mode = -1;
	bool is_virtual = false;
	bool preferred = false, interlaced = false;
	bool have_preferred = false, have_interlaced = false;
	bool is_preferred = false;
	struct drm_display_mode *mode = &out->mode;

	*out = (struct dcp_display_mode) {
		.sdr_rgb.score = -1,
		.sdr_444.score = -1,
		.sdr.score = -1,
		.best.score = -1,
	};
	*score = -1;
	handle->ambiguous_180hz = false;

	if (external_sdr) {
		struct dcp_parse_ctx scan = *handle;

		/* ColorModes can precede either list: collect restrictions first. */
		ret = parse_timing_color_restrictions(&scan, &limits);
		if (ret)
			return ret;
	}
	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		char *key = parse_string(it.handle);
		unsigned int field;

		if (!IS_ERR(key) && handle->automatic_1080p180_rgb8)
			for (field = 0; field < ARRAY_SIZE(fields); field++)
				if (!strcmp(key, fields[field])) {
					handle->ambiguous_180hz |= !!(seen & (1U << field));
					seen |= 1U << field;
				}

		if (IS_ERR(key))
			ret = PTR_ERR(key);
		else if (is_virtual)
			ret = skip(it.handle);
		else if (!strcmp(key, "HorizontalAttributes")) {
			duplicate_scope |= have_horiz;
			have_horiz = true;
			ret = parse_dimension(it.handle, &horiz, external_sdr);
		} else if (!strcmp(key, "VerticalAttributes")) {
			duplicate_scope |= have_vert;
			have_vert = true;
			ret = parse_dimension(it.handle, &vert, external_sdr);
		} else if (!strcmp(key, "MinimumVariableRefreshRate"))
			ret = parse_int_bound(it.handle, &min_vrr, 0, U32_MAX);
		else if (!strcmp(key, "MaximumVariableRefreshRate"))
			ret = parse_int_bound(it.handle, &max_vrr, 0, U32_MAX);
		else if (!strcmp(key, "ColorModes"))
			ret = parse_color_modes(it.handle, out, internal,
						external_sdr, &limits,
					handle->automatic_1080p180_rgb8 ?
					&unsafe_rgb8 : NULL);
		else if (!strcmp(key, "ID"))
			ret = parse_int_bound(it.handle, &id, 0, U32_MAX);
		else if (!strcmp(key, "IsVirtual")) {
			duplicate_scope |= have_virtual;
			have_virtual = true;
			ret = parse_bool(it.handle, &is_virtual);
		} else if (external_sdr && !strcmp(key, "IsPreferred")) {
			if (have_preferred)
				ret = -EINVAL;
			else
				ret = parse_bool(it.handle, &preferred);
			have_preferred = true;
		} else if (external_sdr && !strcmp(key, "IsInterlaced")) {
			if (have_interlaced)
				ret = -EINVAL;
			else
				ret = parse_bool(it.handle, &interlaced);
			have_interlaced = true;
		} else if (!strcmp(key, "IsPreferred")) {
			ret = parse_bool(it.handle, &is_preferred);
		} else if (!strcmp(key, "Score"))
			ret = parse_int_bound(it.handle, score, 0, S64_MAX);
		else
			ret = skip(it.handle);

		if (!IS_ERR_OR_NULL(key))
			kfree(key);

		if (ret) {
			trace_iomfb_parse_mode_fail(id, &horiz, &vert, best_color_mode, is_virtual, *score);
			return ret;
		}
	}
	/*
	 * Never replace a normal candidate or create a timing/color pairing.
	 * Only the bounded automatic policy may publish or use this candidate.
	 * Its EDID identity and successful link callbacks must remain current.
	 * The observed 2080x1106 total and 11796583/65536 Hz are deliberate:
	 * do not widen this to arbitrary nominal-180 Hz blanking or rates.
	 */
	if (external_sdr && !internal &&
	    handle->automatic_1080p180_rgb8 &&
	    !handle->ambiguous_180hz &&
	    out->sdr_rgb.score < 0 && unsafe_rgb8.score >= 0 &&
	    !duplicate_scope && have_horiz && have_vert && have_virtual && !is_virtual &&
	    have_interlaced && !interlaced && !notch_height &&
	    horiz.active == 1920 && vert.active == 1080 &&
	    horiz.total == 2080 && vert.total == 1106 &&
	    vert.precise_sync_rate == 11796583 &&
	    horiz.pixel_repetition == 0 && vert.pixel_repetition == 0 &&
	    horiz.sync_polarity >= 0 && vert.sync_polarity >= 0) {
		out->sdr_rgb = unsafe_rgb8;
		out->automatic_rgb8_quirk = handle->automatic_1080p180_rgb8;
	}

	if (out->sdr_rgb.score >= 0)
		best_color_mode = out->sdr_rgb.id;
	else if (out->sdr_444.score >= 0)
		best_color_mode = out->sdr_444.id;
	else if (out->sdr.score >= 0)
		best_color_mode = out->sdr.id;
	else if (out->best.score >= 0)
		best_color_mode = out->best.id;

	trace_iomfb_parse_mode_success(id, &horiz, &vert, best_color_mode,
				       is_virtual, *score);

	/*
	 * Reject modes without valid color mode.
	 */
	if (best_color_mode < 0 || id < 0 || *score < 0 ||
	    horiz.active <= 0 || vert.active <= notch_height ||
	    vert.precise_sync_rate <= 0)
		return -EINVAL;

	/*
	 * We need to skip virtual modes. In some cases, virtual modes are "too
	 * big" for the monitor and can cause breakage. It is unclear why the
	 * DCP reports these modes at all. Treat as a recoverable error.
	 */
	if (is_virtual)
		return -EINVAL;

	/*
	 * An internal ProMotion panel carries no EDID or DisplayID, so DCP
	 * reports no adaptive-sync range for it. Assume the ProMotion floor of
	 * 24 Hz up to whatever rate the mode itself advertises.
	 */
	if (internal && vert.precise_sync_rate >> 16 == 120) {
		out->min_vrr = 24 << 16;
		out->max_vrr = vert.precise_sync_rate;
		out->vrr = true;
	}

	/*
	 * Refresh rates are reported by DCP as 16.16 fixed-point Hz.
	 * External H17P skips adaptive-sync programming: keep the fixed timing
	 * available, but do not advertise its sink VRR range as driver support.
	 */
	if (!external_sdr && min_vrr && max_vrr > min_vrr) {
		out->min_vrr = min_vrr;
		out->max_vrr = max_vrr;
		out->vrr = true;
	}

	clock = calculate_clock(&horiz, &vert);
	if (!clock || clock > INT_MAX)
		return -EINVAL;

	vert.active -= notch_height;
	vert.sync_width += notch_height;

	/* From here we must succeed. Start filling out the mode. */
	*mode = (struct drm_display_mode) {
		.type = DRM_MODE_TYPE_DRIVER,
		.clock = clock,

		.vdisplay = vert.active,
		.vsync_start = vert.active + vert.front_porch,
		.vsync_end = vert.active + vert.front_porch + vert.sync_width,
		.vtotal = vert.total,

		.hdisplay = horiz.active,
		.hsync_start = horiz.active + horiz.front_porch,
		.hsync_end = horiz.active + horiz.front_porch +
			     horiz.sync_width,
		.htotal = horiz.total,

		.width_mm = width_mm,
		.height_mm = height_mm,
	};

	if (external_sdr && horiz.sync_polarity >= 0 && vert.sync_polarity >= 0)
		mode->flags = (horiz.sync_polarity ? DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC) |
			      (vert.sync_polarity ? DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC);
	out->metadata_timing_valid = external_sdr && !out->automatic_rgb8_quirk &&
		have_preferred && preferred &&
		have_interlaced && !interlaced && horiz.pixel_repetition == 0 &&
		vert.pixel_repetition == 0 && horiz.sync_polarity >= 0 && vert.sync_polarity >= 0;
	drm_mode_set_name(mode);

	out->timing_mode_id = id;
	out->color_mode_id = best_color_mode;
	out->preferred = is_preferred;

	trace_iomfb_timing_mode(handle->dcp, id, *score, horiz.active,
				vert.active, vert.precise_sync_rate,
				best_color_mode);

	return 0;
}

/*
 * Inspect command identities before filtering can discard conflicting siblings.
 * Only automatic exceptions use this conservative receipt-wide check.
 */
static bool dcp_180hz_color_ids_current(struct dcp_parse_ctx *handle, u32 id)
{
	static const char * const fields[] = {
		"ID", "Depth", "DynamicRange", "EOTF", "PixelEncoding", "Colorimetry",
	};
	struct iterator colors;

	if (iterator_begin(handle, &colors, false))
		return false;
	for (; colors.idx < colors.len; colors.idx++) {
		struct color_mode color = { .id = -1, .depth = -1, .dynamic_range = -1,
			.eotf = -1, .pixel_encoding = -1, .colorimetry = -1 };
		s64 *values[] = { &color.id, &color.depth, &color.dynamic_range,
			&color.eotf, &color.pixel_encoding, &color.colorimetry };
		bool is_virtual = true, virtual_seen = false;
		unsigned int seen = 0;
		struct iterator entry;

		if (iterator_begin(handle, &entry, true))
			return false;
		for (; entry.idx < entry.len; entry.idx++) {
			char *key = parse_string(handle);
			unsigned int field;
			int ret;

			if (IS_ERR(key))
				return false;
			for (field = 0; field < ARRAY_SIZE(fields); field++)
				if (!strcmp(key, fields[field]))
					break;
			if (field < ARRAY_SIZE(fields)) {
				ret = seen & (1U << field) ? -EINVAL :
					parse_int_bound(handle, values[field], 0, U32_MAX);
				seen |= 1U << field;
			} else if (!strcmp(key, "IsVirtual")) {
				ret = virtual_seen ? -EINVAL : parse_bool(handle, &is_virtual);
				virtual_seen = true;
			} else {
				ret = skip(handle);
			}
			kfree(key);
			if (ret)
				return false;
		}
		if (color.id == id &&
		    (is_virtual || !virtual_seen || color.depth != 8 ||
		     color.dynamic_range != DCP_COLOR_YCBCR_RANGE_FULL ||
		     color.eotf != DCP_EOTF_SDR_GAMMA ||
		     color.pixel_encoding != DCP_COLOR_FORMAT_RGB ||
		     color.colorimetry != DCP_COLORIMETRY_SRGB))
			return false;
	}
	return true;
}

static bool dcp_180hz_ids_current(struct dcp_parse_ctx handle,
				const struct dcp_display_mode *mode)
{
	struct iterator timings;
	unsigned int matches = 0;

	if (iterator_begin(&handle, &timings, false))
		return false;
	for (; timings.idx < timings.len; timings.idx++) {
		struct iterator entry;

		if (iterator_begin(&handle, &entry, true))
			return false;
		for (; entry.idx < entry.len; entry.idx++) {
			char *key = parse_string(&handle);
			int ret = 0;

			if (IS_ERR(key))
				return false;
			if (!strcmp(key, "ID")) {
				s64 id;

				ret = parse_int_bound(&handle, &id, 0, U32_MAX);
				if (!ret && id == mode->timing_mode_id)
					matches++;
			} else if (!strcmp(key, "ColorModes")) {
				if (!dcp_180hz_color_ids_current(&handle, mode->color_mode_id))
					ret = -EINVAL;
			} else {
				ret = skip(&handle);
			}
			kfree(key);
			if (ret || matches > 1)
				return false;
		}
	}
	return matches == 1;
}

/*
 * A timing mode that the firmware marks as preferred ranks above every mode
 * that it does not mark; the firmware score orders the modes of each group.
 */
VISIBLE_IF_KUNIT bool dcp_mode_ranks_above(bool preferred, s64 score,
					   bool best_preferred, s64 best_score)
{
	if (preferred != best_preferred)
		return preferred;

	return score > best_score;
}

struct dcp_display_mode *enumerate_modes(struct dcp_parse_ctx *handle,
					 unsigned int *count, int width_mm,
					 int height_mm, unsigned notch_height,
					 bool internal, bool external_sdr)
{
	struct dcp_parse_ctx catalog = *handle;
	struct iterator it;
	int ret;
	struct dcp_display_mode *mode, *modes;
	struct dcp_display_mode *best_mode = NULL;
	s64 score, best_score = -1;
	bool best_preferred = false;

	ret = iterator_begin(handle, &it, false);

	if (ret)
		return ERR_PTR(ret);

	/*
	 * Start with a worst case allocation. The count comes from firmware:
	 * every entry needs at least a tag in what is left of the blob, and no
	 * display has more timings than DCP_MAX_TIMING_MODES. The catalog is
	 * retained once published, so reject anything larger before
	 * allocating for it.
	 */
	if (it.len > (handle->len - handle->pos) / sizeof(struct dcp_parse_tag))
		return ERR_PTR(-EINVAL);
	if (it.len > DCP_MAX_TIMING_MODES)
		return ERR_PTR(-E2BIG);
	modes = kcalloc(it.len, sizeof(*modes), GFP_KERNEL | __GFP_NOWARN);
	*count = 0;

	if (!modes)
		return ERR_PTR(-ENOMEM);

	for (; it.idx < it.len; ++it.idx) {
		struct dcp_parse_ctx entry = *it.handle;

		mode = &modes[*count];
		ret = parse_mode(it.handle, mode, &score, width_mm, height_mm,
				 notch_height, internal, external_sdr);

		/* Errors for a single mode are recoverable -- just skip it. */
		if (ret) {
			/* A failed parse can stop in the middle of the dictionary. */
			*it.handle = entry;
			ret = skip(it.handle);
			if (ret) {
				kfree(modes);
				return ERR_PTR(ret);
			}
			continue;
		}

		/* Process a successful mode */
		(*count)++;

		if (!mode->automatic_rgb8_quirk &&
		    dcp_mode_ranks_above(mode->preferred, score, best_preferred,
					 best_score)) {
			best_score = score;
			best_preferred = mode->preferred;
			best_mode = mode;
		}
	}

	if (best_mode != NULL)
		best_mode->mode.type |= DRM_MODE_TYPE_PREFERRED;

	if (handle->automatic_1080p180_rgb8) {
		unsigned int i, kept = 0;

		for (i = 0; i < *count; i++) {
			if (modes[i].automatic_rgb8_quirk &&
			    !dcp_180hz_ids_current(catalog, &modes[i]))
				continue;
			modes[kept++] = modes[i];
		}
		*count = kept;
	}
	return modes;
}

/* Entries of the array at the cursor, which is not consumed. */
int parse_array_count(const struct dcp_parse_ctx *handle, u32 *count)
{
	struct dcp_parse_ctx extent = *handle;
	struct iterator it;
	int ret;

	ret = iterator_begin(&extent, &it, false);
	if (ret)
		return ret;
	*count = it.len;
	return 0;
}

int replace_modes(struct dcp_parse_ctx *handle,
		  struct dcp_display_mode **modes, unsigned int *count,
		  int width_mm, int height_mm, unsigned int notch_height,
		  bool internal)
{
	struct dcp_display_mode *next;
	struct dcp_parse_ctx extent = *handle;
	struct iterator it;
	unsigned int next_count;
	int ret;

	ret = iterator_begin(&extent, &it, false);
	if (ret)
		return ret;

	next = enumerate_modes(handle, &next_count, width_mm, height_mm,
			       notch_height, internal, false);
	if (IS_ERR(next))
		return PTR_ERR(next);
	if (it.len && !next_count) {
		kfree(next);
		return -EINVAL;
	}

	kfree(*modes);
	*modes = next;
	*count = next_count;
	return 0;
}

int parse_display_attributes(struct dcp_parse_ctx *handle, int *width_mm,
			     int *height_mm, bool *backlight_control)
{
	int ret = 0;
	struct iterator it;
	s64 width_cm = 0, height_cm = 0;

	*backlight_control = false;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		char *key = parse_string(it.handle);

		if (IS_ERR(key))
			ret = PTR_ERR(key);
		else if (!strcmp(key, "MaxHorizontalImageSize"))
			ret = parse_int_bound(it.handle, &width_cm, 0, INT_MAX / 10);
		else if (!strcmp(key, "MaxVerticalImageSize"))
			ret = parse_int_bound(it.handle, &height_cm, 0, INT_MAX / 10);
		else if (!strcmp(key, "SupportsBacklightControl"))
			ret = parse_bool_or_skip(it.handle, backlight_control);
		else
			ret = skip(it.handle);

		if (!IS_ERR_OR_NULL(key))
			kfree(key);

		if (ret)
			return ret;
	}

	/* 1cm = 10mm */
	*width_mm = 10 * width_cm;
	*height_mm = 10 * height_cm;

	return 0;
}

/* Metadata strings must fit an EDID text descriptor without hidden suffixes. */
static int parse_metadata_string(struct dcp_parse_ctx *handle, char *out, size_t size)
{
	const struct dcp_parse_tag *tag = parse_tag_of_type(handle, DCP_TYPE_STRING);
	const u8 *value;

	if (IS_ERR(tag))
		return PTR_ERR(tag);
	if (!tag->size || tag->size >= size)
		return -EINVAL;
	value = parse_bytes(handle, tag->size);
	if (IS_ERR(value))
		return PTR_ERR(value);
	for (unsigned int i = 0; i < tag->size; i++)
		if (value[i] < 0x20 || value[i] > 0x7e)
			return -EINVAL;
	memcpy(out, value, tag->size);
	out[tag->size] = 0;
	return 0;
}

static int parse_metadata_number(struct dcp_parse_ctx *handle, u32 *out, u32 min, u32 max)
{
	s64 value;
	int ret = parse_int_bound(handle, &value, min, max);

	if (!ret)
		*out = value;
	return ret;
}

static int parse_metadata_point(struct dcp_parse_ctx *handle, u32 point[2], u32 *gamma)
{
	struct iterator it;
	unsigned int seen = 0;
	int ret = iterator_begin(handle, &it, true);

	if (ret)
		return ret;
	for (; it.idx < it.len; it.idx++) {
		char key[64];

		ret = parse_metadata_string(handle, key, sizeof(key));
		if (ret)
			return ret;
		if (!strcmp(key, "X")) {
			if (seen & 1)
				return -EINVAL;
			seen |= 1;
			ret = parse_metadata_number(handle, &point[0], 1, 65535);
		} else if (!strcmp(key, "Y")) {
			if (seen & 2)
				return -EINVAL;
			seen |= 2;
			ret = parse_metadata_number(handle, &point[1], 1, 65535);
		} else if (gamma && !strcmp(key, "Gamma")) {
			if (seen & 4)
				return -EINVAL;
			seen |= 4;
			ret = parse_metadata_number(handle, gamma, 65536, (354U << 16) / 100);
		} else {
			ret = skip(handle);
		}
		if (ret)
			return ret;
	}
	if (seen != (gamma ? 7 : 3) || point[0] + point[1] > 65536)
		return -EINVAL;
	return 0;
}

static int parse_metadata_chromaticity(struct dcp_parse_ctx *handle,
				       struct dcp_display_metadata *out)
{
	static const char * const names[] = { "Red", "Green", "Blue" };
	struct iterator it;
	unsigned int seen = 0;
	int ret = iterator_begin(handle, &it, true);

	if (ret)
		return ret;
	for (; it.idx < it.len; it.idx++) {
		char key[64];
		unsigned int color;

		ret = parse_metadata_string(handle, key, sizeof(key));
		if (ret)
			return ret;
		for (color = 0; color < ARRAY_SIZE(names); color++)
			if (!strcmp(key, names[color]))
				break;
		if (color < ARRAY_SIZE(names)) {
			if (seen & 1U << color)
				return -EINVAL;
			seen |= 1U << color;
			ret = parse_metadata_point(handle, out->chromaticity[color], NULL);
		} else {
			ret = skip(handle);
		}
		if (ret)
			return ret;
	}
	return seen == 7 ? 0 : -EINVAL;
}

static int parse_metadata_product(struct dcp_parse_ctx *handle, struct dcp_display_metadata *out)
{
	struct iterator it;
	unsigned int seen = 0;
	u32 legacy = 0;
	int ret = iterator_begin(handle, &it, true);

	if (ret)
		return ret;
	for (; it.idx < it.len; it.idx++) {
		char key[64];

		ret = parse_metadata_string(handle, key, sizeof(key));
		if (ret)
			return ret;
		if (!strcmp(key, "ManufacturerID")) {
			if (seen & 1)
				return -EINVAL;
			seen |= 1;
			ret = parse_metadata_string(handle, out->manufacturer,
						    sizeof(out->manufacturer));
		} else if (!strcmp(key, "ProductName")) {
			if (seen & 2)
				return -EINVAL;
			seen |= 2;
			ret = parse_metadata_string(handle, out->name, sizeof(out->name));
		} else if (!strcmp(key, "ProductID")) {
			if (seen & 4)
				return -EINVAL;
			seen |= 4;
			ret = parse_metadata_number(handle, &out->product, 0, U16_MAX);
		} else if (!strcmp(key, "WeekOfManufacture")) {
			if (seen & 8)
				return -EINVAL;
			seen |= 8;
			ret = parse_metadata_number(handle, &out->week, 0, 53);
		} else if (!strcmp(key, "YearOfManufacture")) {
			if (seen & 16)
				return -EINVAL;
			seen |= 16;
			ret = parse_metadata_number(handle, &out->year, 1990, 2245);
		} else if (!strcmp(key, "SerialNumber")) {
			if (seen & 64)
				return -EINVAL;
			seen |= 64;
			ret = parse_metadata_number(handle, &out->serial, 0, U32_MAX);
		} else if (!strcmp(key, "AlphanumericSerialNumber")) {
			if (seen & 128)
				return -EINVAL;
			seen |= 128;
			ret = parse_metadata_string(handle, out->serial_text,
						    sizeof(out->serial_text));
		} else if (!strcmp(key, "LegacyManufacturerID")) {
			if (seen & 32)
				return -EINVAL;
			seen |= 32;
			ret = parse_metadata_number(handle, &legacy, 0, 0x7fff);
		} else {
			ret = skip(handle);
		}
		if (ret)
			return ret;
	}
	if ((seen & 31) != 31 || strlen(out->manufacturer) != 3)
		return -EINVAL;
	for (unsigned int i = 0; i < 3; i++)
		if (out->manufacturer[i] < 'A' || out->manufacturer[i] > 'Z')
			return -EINVAL;
	if ((seen & 32) && legacy != (u32)((out->manufacturer[0] - '@') << 10 |
				   (out->manufacturer[1] - '@') << 5 |
				   (out->manufacturer[2] - '@')))
		return -EINVAL;
	return 0;
}

int parse_display_metadata(struct dcp_parse_ctx *handle, struct dcp_display_metadata *out)
{
	struct dcp_display_metadata parsed = {};
	struct iterator it;
	unsigned int seen = 0;
	int ret = iterator_begin(handle, &it, true);

	if (ret)
		return ret;
	for (; it.idx < it.len; it.idx++) {
		char key[64];

		ret = parse_metadata_string(handle, key, sizeof(key));
		if (ret)
			return ret;
		if (!strcmp(key, "MaxHorizontalImageSize")) {
			if (seen & 1)
				return -EINVAL;
			seen |= 1;
			ret = parse_metadata_number(handle, &parsed.width_cm, 1, 255);
		} else if (!strcmp(key, "MaxVerticalImageSize")) {
			if (seen & 2)
				return -EINVAL;
			seen |= 2;
			ret = parse_metadata_number(handle, &parsed.height_cm, 1, 255);
		} else if (!strcmp(key, "Chromaticity")) {
			if (seen & 4)
				return -EINVAL;
			seen |= 4;
			ret = parse_metadata_chromaticity(handle, &parsed);
		} else if (!strcmp(key, "DefaultWhitePoint")) {
			if (seen & 8)
				return -EINVAL;
			seen |= 8;
			ret = parse_metadata_point(handle, parsed.white, &parsed.gamma);
		} else if (!strcmp(key, "ProductAttributes")) {
			if (seen & 16)
				return -EINVAL;
			seen |= 16;
			ret = parse_metadata_product(handle, &parsed);
		} else {
			ret = skip(handle);
		}
		if (ret)
			return ret;
	}
	if (seen != 31)
		return -EINVAL;
	*out = parsed;
	return 0;
}

int parse_epic_service_init(struct dcp_parse_ctx *handle, const char **name,
			    const char **class, s64 *unit, bool h17p_keys)
{
	int ret = 0;
	struct iterator it;
	bool parsed_unit = false;
	bool parsed_name = false;
	bool parsed_class = false;

	*name = ERR_PTR(-ENOENT);
	*class = ERR_PTR(-ENOENT);

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		char *key = parse_string(it.handle);

		if (IS_ERR(key)) {
			ret = PTR_ERR(key);
			break;
		}

		/*
		 * H17P advertises its system service with lower-case keys --
		 * "compartment", "name", "interface-id" -- where earlier
		 * firmware uses EPICName/EPICProviderClass/EPICUnit.  Without
		 * them the service never registers, the firmware has no system
		 * service to ask and idles the panel, so no vblank and no swap
		 * ever completes.  Only H17P callers accept them.  If a
		 * dictionary carries both spellings, the later value wins and
		 * the earlier string is freed.
		 */
		if (!strcmp(key, "EPICName") ||
		    (h17p_keys && !strcmp(key, "compartment"))) {
			if (parsed_name)
				kfree(*name);
			*name = parse_string(it.handle);
			if (IS_ERR(*name))
				ret = PTR_ERR(*name);
			else
				parsed_name = true;
		} else if (!strcmp(key, "EPICProviderClass") ||
			   (h17p_keys && !strcmp(key, "name"))) {
			if (parsed_class)
				kfree(*class);
			*class = parse_string(it.handle);
			if (IS_ERR(*class))
				ret = PTR_ERR(*class);
			else
				parsed_class = true;
		} else if (!strcmp(key, "EPICUnit") ||
			   (h17p_keys && !strcmp(key, "interface-id"))) {
			ret = parse_int(it.handle, unit);
			if (!ret)
				parsed_unit = true;
		} else {
			ret = skip(it.handle);
		}

		kfree(key);
		if (ret)
			break;
	}

	if (!parsed_unit || !parsed_name || !parsed_class)
		ret = -ENOENT;

	if (ret) {
		if (!IS_ERR(*name)) {
			kfree(*name);
			*name = ERR_PTR(ret);
		}
		if (!IS_ERR(*class)) {
			kfree(*class);
			*class = ERR_PTR(ret);
		}
	}

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
static int parse_sample_rate_bit(struct dcp_parse_ctx *handle, unsigned int *ratebit)
{
	s64 rate;
	int ret = parse_int(handle, &rate);

	if (ret)
		return ret;

	*ratebit = snd_pcm_rate_to_rate_bit(rate);
	if (*ratebit == SNDRV_PCM_RATE_KNOT) {
		/*
		 * The rate wasn't recognized, and unless we supply
		 * a supplementary constraint, the SNDRV_PCM_RATE_KNOT bit
		 * will allow any rate. So clear it.
		 */
		*ratebit = 0;
	}

	return 0;
}

static int parse_sample_fmtbit(struct dcp_parse_ctx *handle, u64 *fmtbit)
{
	s64 sample_size;
	int ret = parse_int(handle, &sample_size);

	if (ret)
		return ret;

	switch (sample_size) {
	case 16:
		*fmtbit = SNDRV_PCM_FMTBIT_S16;
		break;
	case 20:
		*fmtbit = SNDRV_PCM_FMTBIT_S20;
		break;
	case 24:
		*fmtbit = SNDRV_PCM_FMTBIT_S24;
		break;
	case 32:
		*fmtbit = SNDRV_PCM_FMTBIT_S32;
		break;
	default:
		*fmtbit = 0;
		break;
	}

	return 0;
}

static struct {
	const char *label;
	u8 type;
} chan_position_names[] = {
	{ "Front Left", SNDRV_CHMAP_FL },
	{ "Front Right", SNDRV_CHMAP_FR },
	{ "Rear Left", SNDRV_CHMAP_RL },
	{ "Rear Right", SNDRV_CHMAP_RR },
	{ "Front Center", SNDRV_CHMAP_FC },
	{ "Low Frequency Effects", SNDRV_CHMAP_LFE },
	{ "Rear Center", SNDRV_CHMAP_RC },
	{ "Front Left Center", SNDRV_CHMAP_FLC },
	{ "Front Right Center", SNDRV_CHMAP_FRC },
	{ "Rear Left Center", SNDRV_CHMAP_RLC },
	{ "Rear Right Center", SNDRV_CHMAP_RRC },
	{ "Front Left Wide", SNDRV_CHMAP_FLW },
	{ "Front Right Wide", SNDRV_CHMAP_FRW },
	{ "Front Left High", SNDRV_CHMAP_FLH },
	{ "Front Center High", SNDRV_CHMAP_FCH },
	{ "Front Right High", SNDRV_CHMAP_FRH },
	{ "Top Center", SNDRV_CHMAP_TC },
};

static void append_chmap(struct snd_pcm_chmap_elem *chmap, u8 type)
{
	if (!chmap || chmap->channels >= ARRAY_SIZE(chmap->map))
		return;

	chmap->map[chmap->channels] = type;
	chmap->channels++;
}

static int parse_chmap(struct dcp_parse_ctx *handle, struct snd_pcm_chmap_elem *chmap)
{
	struct iterator it;
	int i, ret;

	if (!chmap)
		return skip(handle);

	chmap->channels = 0;

	ret = iterator_begin(handle, &it, false);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		for (i = 0; i < ARRAY_SIZE(chan_position_names); i++)
			if (consume_string(it.handle, chan_position_names[i].label))
				break;

		if (i == ARRAY_SIZE(chan_position_names)) {
			ret = skip(it.handle);
			if (ret)
				return ret;

			append_chmap(chmap, SNDRV_CHMAP_UNKNOWN);
			continue;
		}

		append_chmap(chmap, chan_position_names[i].type);
	}

	return 0;
}

static int parse_chan_layout_element(struct dcp_parse_ctx *handle,
				     unsigned int *nchans_out,
				     struct snd_pcm_chmap_elem *chmap)
{
	struct iterator it;
	int ret;
	s64 nchans = 0;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		if (consume_string(it.handle, "ActiveChannelCount"))
			ret = parse_int(it.handle, &nchans);
		else if (consume_string(it.handle, "ChannelLayout"))
			ret = parse_chmap(it.handle, chmap);
		else
			ret = skip_pair(it.handle);

		if (ret)
			return ret;
	}

	if (nchans_out)
		*nchans_out = nchans;

	return 0;
}

static int parse_nchans_mask(struct dcp_parse_ctx *handle, unsigned int *mask)
{
	struct iterator it;
	int ret;

	*mask = 0;

	ret = iterator_begin(handle, &it, false);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		int nchans;

		ret = parse_chan_layout_element(it.handle, &nchans, NULL);
		if (ret)
			return ret;
		*mask |= 1 << nchans;
	}

	return 0;
}

static int parse_avep_element(struct dcp_parse_ctx *handle,
			      struct dcp_sound_format_mask *sieve,
			      struct dcp_sound_format_mask *hits)
{
	struct dcp_sound_format_mask mask = {0, 0, 0};
	struct iterator it;
	int ret;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		if (consume_string(handle, "StreamSampleRate"))
			ret = parse_sample_rate_bit(it.handle, &mask.rates);
		else if (consume_string(handle, "SampleSize"))
			ret = parse_sample_fmtbit(it.handle, &mask.formats);
		else if (consume_string(handle, "AudioChannelLayoutElements"))
			ret = parse_nchans_mask(it.handle, &mask.nchans);
		else
			ret = skip_pair(it.handle);

		if (ret)
			return ret;
	}

	trace_avep_sound_mode(handle->dcp, mask.rates, mask.formats, mask.nchans);

	if (!(mask.rates & sieve->rates) || !(mask.formats & sieve->formats) ||
		!(mask.nchans & sieve->nchans))
	    return 0;

	if (hits) {
		hits->rates |= mask.rates;
		hits->formats |= mask.formats;
		hits->nchans |= mask.nchans;
	}

	return 1;
}

static int parse_mode_in_avep_element(struct dcp_parse_ctx *handle,
				      unsigned int selected_nchans,
				      struct snd_pcm_chmap_elem *chmap,
				      struct dcp_sound_cookie *cookie)
{
	struct iterator it;
	struct dcp_parse_ctx save_handle;
	int ret;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		if (consume_string(it.handle, "AudioChannelLayoutElements")) {
			struct iterator inner_it;
			int nchans;

			ret = iterator_begin(it.handle, &inner_it, false);
			if (ret)
				return ret;

			for (; inner_it.idx < inner_it.len; ++inner_it.idx) {
				save_handle = *it.handle;
				ret = parse_chan_layout_element(inner_it.handle,
								&nchans, NULL);
				if (ret)
					return ret;

				if (nchans != selected_nchans)
					continue;

				/*
				 * Now that we know this layout matches the
				 * selected channel number, reread the element
				 * and fill in the channel map.
				 */
				*inner_it.handle = save_handle;
				ret = parse_chan_layout_element(inner_it.handle,
								NULL, chmap);
				if (ret)
					return ret;
			}
		} else if (consume_string(it.handle, "ElementData")) {
			size_t size = DCP_SOUND_COOKIE_SIZE;
			const u8 *blob;

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
			if (dcp_parse_v14_7_audio(handle))
				size = DCP_SOUND_COOKIE_SIZE_V14_7;
#endif
			ret = parse_blob(it.handle, size, &blob);
			if (ret)
				return ret;

			if (cookie)
				memcpy(cookie, blob, size);
		} else {
			ret = skip_pair(it.handle);
			if (ret)
				return ret;
		}
	}

	return 0;
}

int parse_sound_constraints(struct dcp_parse_ctx *handle,
			    struct dcp_sound_format_mask *sieve,
			    struct dcp_sound_format_mask *hits)
{
	int ret;
	struct iterator it;

	if (hits) {
		hits->rates = 0;
		hits->formats = 0;
		hits->nchans = 0;
	}

	ret = iterator_begin(handle, &it, false);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		ret = parse_avep_element(it.handle, sieve, hits);

		if (ret < 0)
			return ret;
	}

	return 0;
}

int parse_sound_mode(struct dcp_parse_ctx *handle,
		     struct dcp_sound_format_mask *sieve,
		     struct snd_pcm_chmap_elem *chmap,
		     struct dcp_sound_cookie *cookie)
{
	struct dcp_parse_ctx save_handle;
	struct iterator it;
	int ret;

	ret = iterator_begin(handle, &it, false);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		save_handle = *it.handle;
		ret = parse_avep_element(it.handle, sieve, NULL);

		if (!ret)
			continue;

		if (ret < 0)
			return ret;

		ret = parse_mode_in_avep_element(&save_handle, __ffs(sieve->nchans),
						 chmap, cookie);
		if (ret < 0)
			return ret;
		return 1;
	}

	return 0;
}
#endif

int parse_system_log_mnits(struct dcp_parse_ctx *handle, struct dcp_system_ev_mnits *entry)
{
	struct iterator it;
	int ret;
	s64 mnits = -1;
	s64 idac = -1;
	s64 timestamp = -1;
	bool type_match = false;

	ret = iterator_begin(handle, &it, true);
	if (ret)
		return ret;

	for (; it.idx < it.len; ++it.idx) {
		char *key = parse_string(it.handle);
		if (IS_ERR(key)) {
			ret = PTR_ERR(key);
		} else if (!strcmp(key, "mNits")) {
			ret = parse_int(it.handle, &mnits);
		} else if (!strcmp(key, "iDAC")) {
			ret = parse_int(it.handle, &idac);
		} else if (!strcmp(key, "logEvent")) {
			const char *value = parse_string(it.handle);

			ret = IS_ERR(value) ? PTR_ERR(value) : 0;
			if (!ret) {
				type_match = strcmp(value, "Display (Event Forward)") == 0;
				kfree(value);
			}
		} else if (!strcmp(key, "timestamp")) {
			ret = parse_int(it.handle, &timestamp);
		} else {
			ret = skip(it.handle);
		}

		if (!IS_ERR_OR_NULL(key))
			kfree(key);

		if (ret) {
			pr_err("dcp parser: failed to parse mNits sys event\n");
			return ret;
		}
	}

	if (!type_match ||  mnits < 0 || idac < 0 || timestamp < 0)
		return -EINVAL;
	if (mnits > U32_MAX)
		return -ERANGE;

	entry->millinits = mnits;
	entry->idac = idac;
	entry->timestamp = timestamp;

	return 0;
}
