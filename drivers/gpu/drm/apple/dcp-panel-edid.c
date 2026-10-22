// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * An EDID for a built-in panel.
 *
 * The DCP gives no EDID for a built-in panel, so a compositor sees no make,
 * model or physical size for it. This builds one from what the panel's
 * device-tree node and a table of the known panel models say: the vendor and
 * model of its most specific compatible ("apple,panel-j516": make APP, name
 * J516), the size the connector has, the model year, the panel's native timing
 * as the DCP's mode line gives it with all rows shown, and the colour of a P3
 * wide-colour panel (P3 primaries, D65 white point and a nominal gamma of
 * 2.2, which the sRGB transfer function of Display P3 approximates).
 *
 * The native timing is 3456x2234 at 120 Hz with a pixel clock of 1042 MHz.
 * The base block's detailed timing cannot hold it: its pixel clock field is
 * 16 bits in 10 kHz units (at most 655.35 MHz) and its vertical front porch 6
 * bits. The base block therefore carries the panel's geometry at 60 Hz (the
 * same totals and horizontal porches, the vertical front porch at most 63
 * lines), and a DisplayID 2.0 extension carries the real 120 Hz timing as its
 * preferred timing, with the display parameters. Neither is a mode source:
 * the driver publishes the firmware's modes and never adds modes from this
 * EDID. It makes no other claim: no product code or serial number, no
 * manufacture week, no luminance and no refresh range.
 */

#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <drm/drm_connector.h>
#include <drm/drm_edid.h>

#include "dcp-panel-edid.h"

/* Device-tree vendor prefixes and their PNP IDs. */
static const struct {
	const char *prefix;
	const char *pnp;
} dcp_panel_vendors[] = {
	{ "apple", "APP" },
};

/*
 * Panel models: the year their machines came out (the EDID model year) and the
 * native timing of the DCP's mode line, with the rows the notch hides shown.
 */
static const struct {
	const char *name;
	u16 year;
	struct dcp_panel_timing timing;
} dcp_panel_models[] = {
	/* MacBook Pro (16-inch, M3 Pro or M3 Max): 3456x2234 at 120 Hz */
	{ "J516", 2023, { 1042130, 3456, 3464, 3496, 3536, 2234, 2352, 2360, 2456 } },
};

/* EDID 1.4: a nominal gamma of 2.2 is (2.2 - 1) * 100 = 120. */
#define DCP_PANEL_EDID_GAMMA 120
/* EDID 1.4: week 0xff marks the year byte as the model year, from 1990. */
#define DCP_PANEL_EDID_MODEL_YEAR 0xff
/* The base block's detailed timing: the panel's geometry at this rate. */
#define DCP_PANEL_EDID_DTD_HZ 60
/* EDID 1.4 detailed timing: the vertical front porch field is 6 bits. */
#define DCP_PANEL_EDID_DTD_VFP_MAX 63

/* P3 primaries and the D65 white point, CIE 1931 x and y in 1/10000. */
static const u16 dcp_panel_p3[8] = { 6800, 3200, 2650, 6900, 1500, 600, 3127, 3290 };

/* DisplayID 2.0 (VESA DisplayID Standard v2.0). */
#define DISPLAYID_EXT		0x70	/* EDID extension tag */
#define DISPLAYID_V20		0x20	/* structure version 2, revision 0 */
#define DISPLAYID_GENERIC	0x02	/* primary use case: generic display */
#define DISPLAYID_PARAMS	0x21	/* Display Parameters data block */
#define DISPLAYID_PARAMS_LEN	29
#define DISPLAYID_TYPE7		0x22	/* Type VII detailed timing data block */
#define DISPLAYID_TYPE7_LEN	20
#define DISPLAYID_PREFERRED	0x80	/* timing options: preferred */
#define DISPLAYID_ASPECT_NONE	0x08	/* aspect ratio not given (from the pixels) */
#define DISPLAYID_SYNC_POS	0x8000	/* front porch word: positive sync */
#define DISPLAYID_NO_SPEAKERS	0x80	/* features: audio not integrated */
#define DISPLAYID_TECH_LCD	0x10	/* device technology: active-matrix LCD */

int dcp_panel_id_from_compatible(struct dcp_panel_id *id, const char *compatible)
{
	const char *comma = strchr(compatible, ','), *model;
	size_t vendor_len, len, i;

	memset(id, 0, sizeof(*id));
	if (!comma)
		return -EINVAL;
	vendor_len = comma - compatible;
	model = comma + 1;
	if (str_has_prefix(model, "panel-"))
		model += strlen("panel-");
	else
		return -EINVAL;
	len = strlen(model);
	if (!len || len > DCP_PANEL_NAME_LEN)
		return -EINVAL;
	for (i = 0; i < len; i++) {
		if (!isalnum(model[i]) && model[i] != '-')
			return -EINVAL;
		id->name[i] = toupper(model[i]);
	}

	for (i = 0; i < ARRAY_SIZE(dcp_panel_models); i++) {
		if (!strcmp(id->name, dcp_panel_models[i].name)) {
			id->model_year = dcp_panel_models[i].year;
			id->timing = dcp_panel_models[i].timing;
		}
	}
	for (i = 0; i < ARRAY_SIZE(dcp_panel_vendors); i++) {
		if (strlen(dcp_panel_vendors[i].prefix) == vendor_len &&
		    !strncmp(compatible, dcp_panel_vendors[i].prefix, vendor_len)) {
			strscpy(id->vendor, dcp_panel_vendors[i].pnp);
			return 0;
		}
	}
	memset(id, 0, sizeof(*id));
	return -ENOENT;
}

static void put_le16(u8 *p, u16 v)
{
	p[0] = v;
	p[1] = v >> 8;
}

static void checksum(u8 *start, size_t len)
{
	u8 sum = 0;
	size_t i;

	for (i = 0; i < len - 1; i++)
		sum += start[i];
	start[len - 1] = -sum;
}

static bool timing_valid(const struct dcp_panel_timing *t)
{
	return t->clock_khz && t->hdisplay < t->hsync_start && t->hsync_start < t->hsync_end &&
	       t->hsync_end < t->htotal && t->htotal - t->hdisplay < 4096 &&
	       t->vdisplay < t->vsync_start && t->vsync_start < t->vsync_end &&
	       t->vsync_end < t->vtotal && t->vtotal - t->vdisplay < 4096 &&
	       t->hsync_start - t->hdisplay < 1024 && t->hsync_end - t->hsync_start < 1024 &&
	       t->vsync_end - t->vsync_start < 64;
}

/* The base block's detailed timing: the panel's geometry at DCP_PANEL_EDID_DTD_HZ. */
static void base_dtd(u8 *d, const struct dcp_panel_timing *t, int width_mm, int height_mm)
{
	u32 clock = DIV_ROUND_CLOSEST((u32)t->htotal * t->vtotal * DCP_PANEL_EDID_DTD_HZ, 10000);
	u16 hblank = t->htotal - t->hdisplay, vblank = t->vtotal - t->vdisplay;
	u16 hfp = t->hsync_start - t->hdisplay, hsw = t->hsync_end - t->hsync_start;
	u16 vfp = min_t(u16, t->vsync_start - t->vdisplay, DCP_PANEL_EDID_DTD_VFP_MAX);
	u16 vsw = t->vsync_end - t->vsync_start;

	put_le16(d, clock);
	d[2] = t->hdisplay;
	d[3] = hblank;
	d[4] = (t->hdisplay >> 8) << 4 | hblank >> 8;
	d[5] = t->vdisplay;
	d[6] = vblank;
	d[7] = (t->vdisplay >> 8) << 4 | vblank >> 8;
	d[8] = hfp;
	d[9] = hsw;
	d[10] = (vfp & 0xf) << 4 | (vsw & 0xf);
	d[11] = (hfp >> 8) << 6 | (hsw >> 8) << 4 | (vfp >> 4) << 2 | (vsw >> 4);
	d[12] = width_mm;
	d[13] = height_mm;
	d[14] = (width_mm >> 8) << 4 | height_mm >> 8;
	/* Digital separate sync, positive vertical and horizontal sync. */
	d[17] = 0x1e;
}

/* The P3/D65 chromaticity of the base block, in 10-bit fractions. */
static void base_chromaticity(struct edid *block)
{
	u16 v[8];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(v); i++)
		v[i] = DIV_ROUND_CLOSEST(dcp_panel_p3[i] * 1024, 10000);
	block->red_green_lo = (v[0] & 3) << 6 | (v[1] & 3) << 4 | (v[2] & 3) << 2 | (v[3] & 3);
	block->blue_white_lo = (v[4] & 3) << 6 | (v[5] & 3) << 4 | (v[6] & 3) << 2 | (v[7] & 3);
	block->red_x = v[0] >> 2;
	block->red_y = v[1] >> 2;
	block->green_x = v[2] >> 2;
	block->green_y = v[3] >> 2;
	block->blue_x = v[4] >> 2;
	block->blue_y = v[5] >> 2;
	block->white_x = v[6] >> 2;
	block->white_y = v[7] >> 2;
}

static int base_block(const struct dcp_panel_id *id, int width_mm, int height_mm,
		      struct edid *block)
{
	struct detailed_non_pixel *name;
	u16 manufacturer = 0;
	size_t len = strnlen(id->name, sizeof(id->name));
	unsigned int i;

	for (i = 0; i < 3; i++) {
		if (id->vendor[i] < 'A' || id->vendor[i] > 'Z')
			return -EINVAL;
		manufacturer = manufacturer << 5 | (id->vendor[i] - '@');
	}

	memset(block, 0, sizeof(*block));
	memcpy(block->header, "\x00\xff\xff\xff\xff\xff\xff\x00", sizeof(block->header));
	block->product_id.manufacturer_name = cpu_to_be16(manufacturer);
	block->mfg_week = DCP_PANEL_EDID_MODEL_YEAR;
	block->mfg_year = id->model_year - 1990;
	block->version = 1;
	block->revision = 4;
	block->input = DRM_EDID_INPUT_DIGITAL;
	block->width_cm = DIV_ROUND_CLOSEST(width_mm, 10);
	block->height_cm = DIV_ROUND_CLOSEST(height_mm, 10);
	block->gamma = DCP_PANEL_EDID_GAMMA;
	base_chromaticity(block);
	/* Standard timings 0x0101 are unused slots. */
	memset(block->standard_timings, 0x01, sizeof(block->standard_timings));

	/* EDID 1.4: the first descriptor is a detailed timing; the second names the panel. */
	base_dtd((u8 *)&block->detailed_timings[0], &id->timing, width_mm, height_mm);
	name = &block->detailed_timings[1].data.other_data;
	name->type = EDID_DETAIL_MONITOR_NAME;
	memset(name->data.str.str, ' ', sizeof(name->data.str.str));
	memcpy(name->data.str.str, id->name, len);
	if (len < sizeof(name->data.str.str))
		name->data.str.str[len] = '\n';
	for (i = 2; i < ARRAY_SIZE(block->detailed_timings); i++)
		block->detailed_timings[i].data.other_data.type = 0x10;
	block->extensions = DCP_PANEL_EDID_BLOCKS - 1;
	checksum((u8 *)block, EDID_LENGTH);
	return 0;
}

/* DisplayID 2.0 CIE 1931 x/y pair in 12-bit fractions, packed in 3 bytes. */
static void displayid_xy(u8 *p, u16 x, u16 y)
{
	u16 X = DIV_ROUND_CLOSEST(x * 4095, 10000), Y = DIV_ROUND_CLOSEST(y * 4095, 10000);

	p[0] = X;
	p[1] = (X >> 8) | (Y & 0xf) << 4;
	p[2] = Y >> 4;
}

/* The DisplayID 2.0 extension: display parameters and the 120 Hz Type VII timing. */
static void displayid_block(const struct dcp_panel_id *id, int width_mm, int height_mm,
			    u8 *ext)
{
	const struct dcp_panel_timing *t = &id->timing;
	u8 *section = ext + 1, *p, *b;
	unsigned int i;

	memset(ext, 0, EDID_LENGTH);
	ext[0] = DISPLAYID_EXT;
	section[0] = DISPLAYID_V20;
	section[1] = 3 + DISPLAYID_PARAMS_LEN + 3 + DISPLAYID_TYPE7_LEN;
	section[2] = DISPLAYID_GENERIC;
	section[3] = 0;	/* no further DisplayID extensions */

	b = section + 4;
	b[0] = DISPLAYID_PARAMS;
	b[2] = DISPLAYID_PARAMS_LEN;
	p = b + 3;
	put_le16(p + 0, width_mm * 10);	/* 0.1 mm */
	put_le16(p + 2, height_mm * 10);
	put_le16(p + 4, t->hdisplay);
	put_le16(p + 6, t->vdisplay);
	/* Left to right, top to bottom; CIE 1931; the speakers are not the panel's. */
	p[8] = DISPLAYID_NO_SPEAKERS;
	for (i = 0; i < 4; i++)
		displayid_xy(p + 9 + 3 * i, dcp_panel_p3[2 * i], dcp_panel_p3[2 * i + 1]);
	/* Bytes 21-26, the luminances, are not known: left 0. Colour depth not given. */
	p[27] = DISPLAYID_TECH_LCD;
	p[28] = DCP_PANEL_EDID_GAMMA;

	b = p + DISPLAYID_PARAMS_LEN;
	b[0] = DISPLAYID_TYPE7;
	b[2] = DISPLAYID_TYPE7_LEN;
	p = b + 3;
	/* Pixel clock in kHz minus one, 24 bits. */
	p[0] = (t->clock_khz - 1);
	p[1] = (t->clock_khz - 1) >> 8;
	p[2] = (t->clock_khz - 1) >> 16;
	p[3] = DISPLAYID_PREFERRED | DISPLAYID_ASPECT_NONE;
	/* Each field minus one; the front porch words carry the sync polarity. */
	put_le16(p + 4, t->hdisplay - 1);
	put_le16(p + 6, t->htotal - t->hdisplay - 1);
	put_le16(p + 8, (t->hsync_start - t->hdisplay - 1) | DISPLAYID_SYNC_POS);
	put_le16(p + 10, t->hsync_end - t->hsync_start - 1);
	put_le16(p + 12, t->vdisplay - 1);
	put_le16(p + 14, t->vtotal - t->vdisplay - 1);
	put_le16(p + 16, (t->vsync_start - t->vdisplay - 1) | DISPLAYID_SYNC_POS);
	put_le16(p + 18, t->vsync_end - t->vsync_start - 1);

	/* The section's checksum after its payload, then the block's. */
	checksum(section, 4 + section[1] + 1);
	checksum(ext, EDID_LENGTH);
}

int dcp_panel_edid_build(const struct dcp_panel_id *id, int width_mm, int height_mm,
			 u8 edid[DCP_PANEL_EDID_BLOCKS * EDID_LENGTH])
{
	size_t len = strnlen(id->name, sizeof(id->name));
	int ret;

	if (width_mm <= 0 || height_mm <= 0 || width_mm > 4095 || height_mm > 4095 ||
	    DIV_ROUND_CLOSEST(width_mm, 10) > 255 || DIV_ROUND_CLOSEST(height_mm, 10) > 255)
		return -EINVAL;
	if (strnlen(id->vendor, sizeof(id->vendor)) != 3 || !len || len > DCP_PANEL_NAME_LEN)
		return -EINVAL;
	/* EDID 1.4 reserves the years before 2006. */
	if (id->model_year < 2006 || id->model_year > 1990 + 255)
		return -EINVAL;
	if (!timing_valid(&id->timing))
		return -EINVAL;
	ret = base_block(id, width_mm, height_mm, (struct edid *)edid);
	if (ret)
		return ret;
	displayid_block(id, width_mm, height_mm, edid + EDID_LENGTH);
	return 0;
}

int dcp_panel_edid_update(struct drm_connector *connector, const struct dcp_panel_id *id,
			  int width_mm, int height_mm)
{
	const struct drm_edid *edid;
	u8 *blocks;
	int ret;

	blocks = kmalloc(DCP_PANEL_EDID_BLOCKS * EDID_LENGTH, GFP_KERNEL);
	if (!blocks)
		return -ENOMEM;
	ret = dcp_panel_edid_build(id, width_mm, height_mm, blocks);
	if (ret)
		goto out;
	edid = drm_edid_alloc(blocks, DCP_PANEL_EDID_BLOCKS * EDID_LENGTH);
	if (!edid) {
		ret = -ENOMEM;
		goto out;
	}
	/* This sets the connector's size from the block, in whole centimetres. */
	ret = drm_edid_connector_update(connector, edid);
	drm_edid_free(edid);
out:
	kfree(blocks);
	return ret;
}
