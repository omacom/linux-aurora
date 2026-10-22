// SPDX-License-Identifier: GPL-2.0-only OR MIT

/*
 * The built-in panel's EDID, read back with the DRM EDID helpers: the make,
 * product, model year and native timing come from the panel's compatible and
 * the model table; the base block holds the panel's geometry at 60 Hz with P3
 * colour and gamma 2.2, the DisplayID extension the 120 Hz timing; an update
 * publishes both blocks on the connector and adds no mode.
 */

#include <kunit/test.h>
#include <linux/errno.h>
#include <linux/string.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_property.h>

#include "dcp-panel-edid.h"

#define J516_W_MM 346
#define J516_H_MM 216

static void panel_id_j516(struct kunit *test, struct dcp_panel_id *id)
{
	KUNIT_ASSERT_EQ(test, dcp_panel_id_from_compatible(id, "apple,panel-j516"), 0);
}

static void panel_id_from_compatible(struct kunit *test)
{
	struct dcp_panel_id id;

	panel_id_j516(test, &id);
	KUNIT_EXPECT_STREQ(test, id.vendor, "APP");
	KUNIT_EXPECT_STREQ(test, id.name, "J516");
	KUNIT_EXPECT_EQ(test, id.model_year, 2023);
	KUNIT_EXPECT_EQ(test, id.timing.clock_khz, 1042130);
	KUNIT_EXPECT_EQ(test, id.timing.vtotal, 2456);
	/* A model without a known year and timing gets none, and no EDID. */
	KUNIT_ASSERT_EQ(test, dcp_panel_id_from_compatible(&id, "apple,panel-j999"), 0);
	KUNIT_EXPECT_EQ(test, id.model_year, 0);
	KUNIT_EXPECT_EQ(test, id.timing.clock_khz, 0);
}

static void panel_id_needs_a_model_and_a_known_vendor(struct kunit *test)
{
	struct dcp_panel_id id;

	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id, "apple,panel"), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id, "apple,panel-"), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id, "panel-j516"), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id, "acme,panel-j516"), -ENOENT);
	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id, "apple,panel-j516 x"), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_panel_id_from_compatible(&id,
							   "apple,panel-j5160000000000"),
			-EINVAL);
}

static const struct drm_edid *panel_edid_j516(struct kunit *test,
					      u8 e[DCP_PANEL_EDID_BLOCKS * EDID_LENGTH])
{
	struct dcp_panel_id id;
	const struct drm_edid *edid;

	panel_id_j516(test, &id);
	KUNIT_ASSERT_EQ(test, dcp_panel_edid_build(&id, J516_W_MM, J516_H_MM, e), 0);
	edid = drm_edid_alloc(e, DCP_PANEL_EDID_BLOCKS * EDID_LENGTH);
	KUNIT_ASSERT_NOT_NULL(test, edid);
	return edid;
}

static u8 sum(const u8 *p, size_t len)
{
	u8 s = 0;

	while (len--)
		s += *p++;
	return s;
}

static void panel_edid_parses_with_make_product_and_size(struct kunit *test)
{
	struct drm_edid_product_id product;
	u8 e[DCP_PANEL_EDID_BLOCKS * EDID_LENGTH];
	const struct drm_edid *edid = panel_edid_j516(test, e);
	const struct edid *raw;
	char vendor[4], name[14];

	KUNIT_EXPECT_TRUE(test, drm_edid_valid(edid));
	drm_edid_get_product_id(edid, &product);
	drm_edid_decode_mfg_id(be16_to_cpu(product.manufacturer_name), vendor);
	KUNIT_EXPECT_STREQ(test, vendor, "APP");
	KUNIT_EXPECT_EQ(test, le16_to_cpu(product.product_code), 0);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(product.serial_number), 0);

	raw = drm_edid_raw(edid);
	KUNIT_ASSERT_NOT_NULL(test, raw);
	drm_edid_get_monitor_name(raw, name, sizeof(name));
	KUNIT_EXPECT_STREQ(test, name, "J516");
	/* The connector's size (the firmware's), in whole centimetres. */
	KUNIT_EXPECT_EQ(test, raw->width_cm, 35);
	KUNIT_EXPECT_EQ(test, raw->height_cm, 22);
	/* Gamma 2.2, not 1.00 (0) and not "in an extension" (0xff). */
	KUNIT_EXPECT_EQ(test, raw->gamma, 0x78);
	/* Model year 2023: week 0xff, year 2023 - 1990. */
	KUNIT_EXPECT_EQ(test, raw->mfg_week, 0xff);
	KUNIT_EXPECT_EQ(test, raw->mfg_year, 33);
	/* P3 red (0.680, 0.320) and the D65 white point (0.3127, 0.3290), 10-bit. */
	KUNIT_EXPECT_EQ(test, raw->red_x << 2 | raw->red_green_lo >> 6, 696);
	KUNIT_EXPECT_EQ(test, raw->red_y << 2 | (raw->red_green_lo >> 4 & 3), 328);
	KUNIT_EXPECT_EQ(test, raw->white_x << 2 | (raw->blue_white_lo >> 2 & 3), 320);
	KUNIT_EXPECT_EQ(test, raw->white_y << 2 | (raw->blue_white_lo & 3), 337);
	/* One extension; both blocks sum to 0. */
	KUNIT_EXPECT_EQ(test, raw->extensions, 1);
	KUNIT_EXPECT_EQ(test, sum(e, EDID_LENGTH), 0);
	KUNIT_EXPECT_EQ(test, sum(e + EDID_LENGTH, EDID_LENGTH), 0);
	drm_edid_free(edid);
}

/*
 * The base block's first descriptor is the panel's geometry at 60 Hz (its 120 Hz
 * clock does not fit the 16-bit field); the DisplayID extension holds the real
 * 120 Hz timing as its preferred Type VII timing.
 */
static void panel_edid_timings_are_the_panels(struct kunit *test)
{
	u8 e[DCP_PANEL_EDID_BLOCKS * EDID_LENGTH];
	const struct drm_edid *edid = panel_edid_j516(test, e);
	const struct edid *raw = drm_edid_raw(edid);
	const struct detailed_timing *d = &raw->detailed_timings[0];
	const struct detailed_pixel_timing *pt = &d->data.pixel_data;
	const u8 *ext = e + EDID_LENGTH, *section = ext + 1, *t;
	unsigned int i, names = 0;

	/* 3536 x 2456 x 60 Hz = 521.06 MHz, in 10 kHz. */
	KUNIT_EXPECT_EQ(test, le16_to_cpu(d->pixel_clock), 52106);
	KUNIT_EXPECT_EQ(test, pt->hactive_lo | (pt->hactive_hblank_hi >> 4) << 8, 3456);
	KUNIT_EXPECT_EQ(test, pt->hblank_lo | (pt->hactive_hblank_hi & 0xf) << 8, 80);
	KUNIT_EXPECT_EQ(test, pt->vactive_lo | (pt->vactive_vblank_hi >> 4) << 8, 2234);
	KUNIT_EXPECT_EQ(test, pt->vblank_lo | (pt->vactive_vblank_hi & 0xf) << 8, 222);
	KUNIT_EXPECT_EQ(test, pt->hsync_offset_lo, 8);
	KUNIT_EXPECT_EQ(test, pt->hsync_pulse_width_lo, 32);
	/* The front porch at the field's 63, the sync width the panel's 8. */
	KUNIT_EXPECT_EQ(test, (pt->vsync_offset_pulse_width_lo >> 4) |
			(pt->hsync_vsync_offset_pulse_width_hi >> 2 & 3) << 4, 63);
	KUNIT_EXPECT_EQ(test, pt->vsync_offset_pulse_width_lo & 0xf, 8);
	KUNIT_EXPECT_EQ(test, pt->width_mm_lo | (pt->width_height_mm_hi >> 4) << 8, J516_W_MM);
	KUNIT_EXPECT_EQ(test, pt->height_mm_lo | (pt->width_height_mm_hi & 0xf) << 8, J516_H_MM);
	/* It is not the preferred mode with its refresh rate. */
	KUNIT_EXPECT_EQ(test, raw->features & DRM_EDID_FEATURE_PREFERRED_TIMING, 0);
	for (i = 1; i < ARRAY_SIZE(raw->detailed_timings); i++) {
		const struct detailed_timing *o = &raw->detailed_timings[i];

		KUNIT_EXPECT_EQ(test, le16_to_cpu(o->pixel_clock), 0);
		KUNIT_EXPECT_NE(test, o->data.other_data.type, EDID_DETAIL_MONITOR_SERIAL);
		KUNIT_EXPECT_NE(test, o->data.other_data.type, EDID_DETAIL_MONITOR_RANGE);
		names += o->data.other_data.type == EDID_DETAIL_MONITOR_NAME;
	}
	KUNIT_EXPECT_EQ(test, names, 1);
	for (i = 0; i < sizeof(raw->established_timings); i++)
		KUNIT_EXPECT_EQ(test, ((const u8 *)&raw->established_timings)[i], 0);

	/* DisplayID 2.0, generic display, Display Parameters then one Type VII timing. */
	KUNIT_EXPECT_EQ(test, ext[0], 0x70);
	KUNIT_EXPECT_EQ(test, section[0], 0x20);
	KUNIT_EXPECT_EQ(test, section[2], 0x02);
	KUNIT_ASSERT_EQ(test, section[1], 3 + 29 + 3 + 20);
	KUNIT_EXPECT_EQ(test, sum(section, 4 + section[1] + 1), 0);
	KUNIT_EXPECT_EQ(test, section[4], 0x21);
	KUNIT_EXPECT_EQ(test, section[6], 29);
	/* Image size in 0.1 mm, the native pixels, gamma 2.2. */
	KUNIT_EXPECT_EQ(test, section[7] | section[8] << 8, J516_W_MM * 10);
	KUNIT_EXPECT_EQ(test, section[9] | section[10] << 8, J516_H_MM * 10);
	KUNIT_EXPECT_EQ(test, section[11] | section[12] << 8, 3456);
	KUNIT_EXPECT_EQ(test, section[13] | section[14] << 8, 2234);
	KUNIT_EXPECT_EQ(test, section[7 + 28], 0x78);
	t = section + 4 + 3 + 29;
	KUNIT_EXPECT_EQ(test, t[0], 0x22);
	KUNIT_EXPECT_EQ(test, t[2], 20);
	t += 3;
	/* 1042130 kHz, preferred, and the DCP mode line's porches (each minus one). */
	KUNIT_EXPECT_EQ(test, t[0] | t[1] << 8 | t[2] << 16, 1042130 - 1);
	KUNIT_EXPECT_EQ(test, t[3] & 0x80, 0x80);
	KUNIT_EXPECT_EQ(test, t[4] | t[5] << 8, 3456 - 1);
	KUNIT_EXPECT_EQ(test, t[6] | t[7] << 8, 3536 - 3456 - 1);
	KUNIT_EXPECT_EQ(test, (t[8] | t[9] << 8) & 0x7fff, 3464 - 3456 - 1);
	KUNIT_EXPECT_EQ(test, t[10] | t[11] << 8, 3496 - 3464 - 1);
	KUNIT_EXPECT_EQ(test, t[12] | t[13] << 8, 2234 - 1);
	KUNIT_EXPECT_EQ(test, t[14] | t[15] << 8, 2456 - 2234 - 1);
	KUNIT_EXPECT_EQ(test, (t[16] | t[17] << 8) & 0x7fff, 2352 - 2234 - 1);
	KUNIT_EXPECT_EQ(test, t[18] | t[19] << 8, 2360 - 2352 - 1);
	drm_edid_free(edid);
}

static void panel_edid_refuses_a_size_it_cannot_hold(struct kunit *test)
{
	u8 e[DCP_PANEL_EDID_BLOCKS * EDID_LENGTH];
	struct dcp_panel_id id;

	panel_id_j516(test, &id);
	KUNIT_EXPECT_EQ(test, dcp_panel_edid_build(&id, 0, J516_H_MM, e), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_panel_edid_build(&id, J516_W_MM, 2560, e), -EINVAL);
	id.model_year = 2005;
	KUNIT_EXPECT_EQ(test, dcp_panel_edid_build(&id, J516_W_MM, J516_H_MM, e), -EINVAL);
	id.model_year = 2023;
	id.timing.clock_khz = 0;
	KUNIT_EXPECT_EQ(test, dcp_panel_edid_build(&id, J516_W_MM, J516_H_MM, e), -EINVAL);
	panel_id_j516(test, &id);
	id.vendor[0] = '\0';
	KUNIT_EXPECT_EQ(test, dcp_panel_edid_build(&id, J516_W_MM, J516_H_MM, e), -EINVAL);
}

struct panel_edid_test {
	struct drm_device drm;
	struct drm_connector connector;
};

/* The DRM KUnit device is atomic: a connector needs its state helpers. */
static const struct drm_connector_funcs panel_edid_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void panel_edid_update_publishes_the_block(struct kunit *test)
{
	struct panel_edid_test *t;
	struct drm_connector *connector;
	struct dcp_panel_id id;
	struct device *dev;
	int ret;

	dev = drm_kunit_helper_alloc_device(test);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	t = drm_kunit_helper_alloc_drm_device(test, dev, struct panel_edid_test, drm,
					      DRIVER_MODESET);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t);
	connector = &t->connector;
	ret = drmm_connector_init(&t->drm, connector, &panel_edid_connector_funcs,
				  DRM_MODE_CONNECTOR_eDP, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);

	panel_id_j516(test, &id);
	mutex_lock(&t->drm.mode_config.mutex);
	ret = dcp_panel_edid_update(connector, &id, J516_W_MM, J516_H_MM);
	mutex_unlock(&t->drm.mode_config.mutex);
	KUNIT_ASSERT_EQ(test, ret, 0);

	/* The update sets 350x220 from the block; dcp_get_modes() restores the mm. */
	KUNIT_EXPECT_EQ(test, connector->display_info.width_mm, 350);
	KUNIT_EXPECT_EQ(test, connector->display_info.height_mm, 220);
	KUNIT_ASSERT_NOT_NULL(test, connector->edid_blob_ptr);
	KUNIT_EXPECT_EQ(test, connector->edid_blob_ptr->length, 2 * EDID_LENGTH);
	/* The update adds no mode: the modes stay the firmware's. */
	KUNIT_EXPECT_TRUE(test, list_empty(&connector->probed_modes));
}

static struct kunit_case panel_edid_cases[] = {
	KUNIT_CASE(panel_id_from_compatible),
	KUNIT_CASE(panel_id_needs_a_model_and_a_known_vendor),
	KUNIT_CASE(panel_edid_parses_with_make_product_and_size),
	KUNIT_CASE(panel_edid_timings_are_the_panels),
	KUNIT_CASE(panel_edid_refuses_a_size_it_cannot_hold),
	KUNIT_CASE(panel_edid_update_publishes_the_block),
	{}
};

static struct kunit_suite panel_edid_suite = {
	.name = "apple-dcp-panel-edid",
	.test_cases = panel_edid_cases,
};

kunit_test_suite(panel_edid_suite);
