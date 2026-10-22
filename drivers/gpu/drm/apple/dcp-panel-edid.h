/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* A base EDID for a built-in panel, from its device-tree node. */
#ifndef __APPLE_DCP_PANEL_EDID_H__
#define __APPLE_DCP_PANEL_EDID_H__

#include <linux/types.h>

/* The monitor-name text of an EDID descriptor holds 13 characters. */
#define DCP_PANEL_NAME_LEN 13

/* An EDID of a base block and one DisplayID extension. */
#define DCP_PANEL_EDID_BLOCKS 2

/* A panel's native timing, as the DCP's mode line gives it (all rows shown). */
struct dcp_panel_timing {
	u32 clock_khz;
	u16 hdisplay, hsync_start, hsync_end, htotal;
	u16 vdisplay, vsync_start, vsync_end, vtotal;
};

/* The make, product, model year and native timing of a panel. */
struct dcp_panel_id {
	char vendor[4];				/* PNP ID, empty if unknown */
	char name[DCP_PANEL_NAME_LEN + 1];	/* monitor name */
	u16 model_year;				/* the model's year, 0 if unknown */
	struct dcp_panel_timing timing;		/* zero if unknown */
};

struct drm_connector;

int dcp_panel_id_from_compatible(struct dcp_panel_id *id, const char *compatible);
int dcp_panel_edid_build(const struct dcp_panel_id *id, int width_mm, int height_mm,
			 u8 edid[DCP_PANEL_EDID_BLOCKS * 128]);
int dcp_panel_edid_update(struct drm_connector *connector, const struct dcp_panel_id *id,
			  int width_mm, int height_mm);

#endif
