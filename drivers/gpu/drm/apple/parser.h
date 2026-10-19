// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

#ifndef __APPLE_DCP_PARSER_H__
#define __APPLE_DCP_PARSER_H__

/* For mode parsing */
#include <drm/drm_modes.h>

#include "dcp-display-metadata.h"
#include "dcp-metadata-epoch.h"

struct apple_dcp;
struct apple_connector;
struct apple_dcp_typec_route;

struct dcp_parse_ctx {
	struct apple_dcp *dcp;
	const void *blob;
	u32 pos, len;
	/* External H17P candidates require separate publication/lookup proof. */
	bool automatic_1080p180_rgb8;
	bool ambiguous_180hz;
	/*
	 * The T6030 external processor (DCP firmware 14.7, external role): of
	 * an external SDR catalog, an 8-bit RGB colour (sRGB or default RGB)
	 * is preferred, ties on score going to the lower colour ID, and
	 * interlaced, split, multi-pipe and pixel-repeated timings are refused.
	 * A timing without such a colour keeps the common selection.
	 */
	bool v14_7_rgb8;
	/*
	 * A timing's DSC-required 8-bit RGB colour the sink takes compressed
	 * (SupportsDSC) is admissible, the firmware compressing it on the
	 * wire; an uncompressed colour of the timing is still preferred.
	 */
	bool v14_7_dsc;
	/*
	 * The routed link's width and highest rate code (RBR 0x06 .. HBR3
	 * 0x1e): a catalog is budgeted by what the link can carry. 0: no
	 * budget.
	 */
	u32 link_lanes, link_rate;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/*
	 * T6030 external processor audio elements (DCP firmware 14.7, external
	 * role): an element's ElementData, the link data the audio service
	 * takes, is 32 bytes.
	 */
	bool v14_7_audio;
#endif
};

/* The T6030 external processor's audio elements (DRM_APPLE_DCP_T6030_AUDIO only). */
static inline bool dcp_parse_v14_7_audio(const struct dcp_parse_ctx *ctx)
{
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	return ctx->v14_7_audio;
#else
	return false;
#endif
}

enum dcp_color_eotf {
	DCP_EOTF_SDR_GAMMA = 0, // "SDR gamma"
	DCP_EOTF_HDR_GAMMA = 1, // "HDR gamma"
	DCP_EOTF_ST_2084   = 2, // "ST 2084 (PQ)"
	DCP_EOTF_BT_2100   = 3, // "BT.2100 (HLG)"
	DCP_EOTF_COUNT
};

enum dcp_color_format {
	DCP_COLOR_FORMAT_RGB                 =  0, // "RGB"
	DCP_COLOR_FORMAT_YCBCR420            =  1, // "YUV 4:2:0"
	DCP_COLOR_FORMAT_YCBCR422            =  3, // "YUV 4:2:2"
	DCP_COLOR_FORMAT_YCBCR444            =  2, // "YUV 4:4:4"
	DCP_COLOR_FORMAT_DV_NATIVE           =  4, // "DolbyVision (native)"
	DCP_COLOR_FORMAT_DV_HDMI             =  5, // "DolbyVision (HDMI)"
	DCP_COLOR_FORMAT_YCBCR422_DP         =  6, // "YCbCr 4:2:2 (DP tunnel)"
	DCP_COLOR_FORMAT_YCBCR422_HDMI       =  7, // "YCbCr 4:2:2 (HDMI tunnel)"
	DCP_COLOR_FORMAT_DV_LL_YCBCR422      =  8, // "DolbyVision LL YCbCr 4:2:2"
	DCP_COLOR_FORMAT_DV_LL_YCBCR422_DP   =  9, // "DolbyVision LL YCbCr 4:2:2 (DP)"
	DCP_COLOR_FORMAT_DV_LL_YCBCR422_HDMI = 10, // "DolbyVision LL YCbCr 4:2:2 (HDMI)"
	DCP_COLOR_FORMAT_DV_LL_YCBCR444      = 11, // "DolbyVision LL YCbCr 4:4:4"
	DCP_COLOR_FORMAT_DV_LL_RGB422        = 12, // "DolbyVision LL RGB 4:2:2"
	DCP_COLOR_FORMAT_GRGB_BLUE_422       = 13, // "GRGB as YCbCr422 (Even line blue)"
	DCP_COLOR_FORMAT_GRGB_RED_422        = 14, // "GRGB as YCbCr422 (Even line red)"
	DCP_COLOR_FORMAT_COUNT
};

enum dcp_colorimetry {
	DCP_COLORIMETRY_BT601              =  0, // "SMPTE 170M/BT.601"
	DCP_COLORIMETRY_BT709              =  1, // "BT.701"
	DCP_COLORIMETRY_XVYCC_601          =  2, // "xvYCC601"
	DCP_COLORIMETRY_XVYCC_709          =  3, // "xvYCC709"
	DCP_COLORIMETRY_SYCC_601           =  4, // "sYCC601"
	DCP_COLORIMETRY_ADOBE_YCC_601      =  5, // "AdobeYCC601"
	DCP_COLORIMETRY_BT2020_CYCC        =  6, // "BT.2020 (c)"
	DCP_COLORIMETRY_BT2020_YCC         =  7, // "BT.2020 (nc)"
	DCP_COLORIMETRY_VSVDB              =  8, // "DolbyVision VSVDB"
	DCP_COLORIMETRY_BT2020_RGB         =  9, // "BT.2020 (RGB)"
	DCP_COLORIMETRY_SRGB               = 10, // "sRGB"
	DCP_COLORIMETRY_SCRGB              = 11, // "scRGB"
	DCP_COLORIMETRY_SCRGB_FIXED        = 12, // "scRGBfixed"
	DCP_COLORIMETRY_ADOBE_RGB          = 13, // "AdobeRGB"
	DCP_COLORIMETRY_DCI_P3_RGB_D65     = 14, // "DCI-P3 (D65)"
	DCP_COLORIMETRY_DCI_P3_RGB_THEATER = 15, // "DCI-P3 (Theater)"
	DCP_COLORIMETRY_RGB                = 16, // "Default RGB"
	DCP_COLORIMETRY_COUNT
};

enum dcp_color_range {
	DCP_COLOR_YCBCR_RANGE_FULL    = 0,
	DCP_COLOR_YCBCR_RANGE_LIMITED = 1,
	DCP_COLOR_YCBCR_RANGE_COUNT
};

struct dcp_color_mode {
	s64 score;
	u32 id;
	enum dcp_color_eotf eotf;
	enum dcp_color_format format;
	enum dcp_colorimetry colorimetry;
	enum dcp_color_range range;
	u8 depth;
};

/*
 * Represents a single display mode. These mode objects are populated at
 * runtime based on the TimingElements dictionary sent by the DCP.
 */
struct dcp_display_mode {
	struct drm_display_mode mode;
	u32 color_mode_id;
	u32 timing_mode_id;
	u32 min_vrr;
	u32 max_vrr;
	struct dcp_color_mode sdr_rgb;
	struct dcp_color_mode sdr_444;
	struct dcp_color_mode sdr;
	struct dcp_color_mode best;
	bool vrr;
	bool metadata_timing_valid;
	bool automatic_rgb8_quirk;
	u64 automatic_connection_sequence;
	u64 automatic_catalog_sequence;
	bool automatic_service_pending;
	/* Immutable receipt provenance; never retag a cached exception. */
	struct dcp_metadata_ticket unsafe_rgb8_ticket;
	u64 unsafe_rgb8_service_epoch;
	/* The timing's best DSC-required colour, and whether it was taken. */
	struct dcp_color_mode dsc_rgb;
	bool dsc;
};

struct dimension {
	s64 total, front_porch, sync_width, active;
	s64 precise_sync_rate;
	s64 sync_polarity;
	s64 pixel_repetition;
};

int parse(const void *blob, size_t size, struct dcp_parse_ctx *ctx);
bool dcp_property_complete(const void *blob, size_t received, size_t advertised);
struct dcp_display_mode *enumerate_modes(struct dcp_parse_ctx *handle,
					 unsigned int *count, int width_mm,
					 int height_mm, unsigned notch_height,
					 bool internal, bool external_sdr);
int parse_display_metadata(struct dcp_parse_ctx *handle, struct dcp_display_metadata *out);
unsigned int dcp_catalog_budget(struct dcp_display_mode *modes, unsigned int count,
				u32 lanes, u32 rate, unsigned int *rejected);
/* Caller holds modes_lock while checking connector ownership. */
bool dcp_modes_for_connector(struct apple_dcp *dcp,
			     struct apple_connector *connector);
void dcp_modes_begin_attachment(struct apple_dcp *dcp);
struct dcp_display_mode *dcp_modes_dup(struct apple_dcp *dcp,
				       struct apple_connector *connector,
				       unsigned int *count, u64 *generation);
void dcp_modes_adopt(struct apple_dcp *dcp, struct apple_connector *connector,
		     u64 generation, struct dcp_display_mode *modes,
		     unsigned int count);
bool dcp_modes_end_typec(struct apple_dcp *dcp, struct apple_dcp_typec_route *route);
void dcp_modes_init(struct apple_dcp *dcp);
void dcp_modes_release(void *data);
int dcp_attributes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *handle,
			   u64 generation);
u64 dcp_modes_transfer_begin(struct apple_dcp *dcp);
int dcp_modes_replace(struct apple_dcp *dcp, struct dcp_parse_ctx *handle,
		      u64 generation);

/* Caller serializes readers against replacement. */
int replace_modes(struct dcp_parse_ctx *handle,
		  struct dcp_display_mode **modes, unsigned int *count,
		  int width_mm, int height_mm, unsigned int notch_height,
		  bool internal, bool external_sdr);
int parse_display_attributes(struct dcp_parse_ctx *handle, int *width_mm,
			     int *height_mm, bool *backlight_control);
int parse_display_attributes_h17p(struct dcp_parse_ctx *handle, int *width_mm,
				  int *height_mm);
int parse_epic_service_init(struct dcp_parse_ctx *handle, const char **name,
			    const char **class, s64 *unit, bool h17p_keys);

struct dcp_sound_format_mask {
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int nchans;
};

/*
 * An audio element's ElementData, the link data of the audio service's link
 * methods: 24 bytes up to DCP firmware 13.5, 32 on the 14.7 external role.
 */
#define DCP_SOUND_COOKIE_SIZE		24
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
#define DCP_SOUND_COOKIE_SIZE_V14_7	32
#define DCP_SOUND_COOKIE_MAX		DCP_SOUND_COOKIE_SIZE_V14_7
#else
#define DCP_SOUND_COOKIE_MAX		DCP_SOUND_COOKIE_SIZE
#endif

struct dcp_sound_cookie {
	u8 data[DCP_SOUND_COOKIE_MAX];
};

struct snd_pcm_chmap_elem;
int parse_sound_constraints(struct dcp_parse_ctx *handle,
			    struct dcp_sound_format_mask *sieve,
			    struct dcp_sound_format_mask *hits);
int parse_sound_mode(struct dcp_parse_ctx *handle,
		     struct dcp_sound_format_mask *sieve,
		     struct snd_pcm_chmap_elem *chmap,
		     struct dcp_sound_cookie *cookie);

struct dcp_system_ev_mnits {
	u32 timestamp;
	u32 millinits;
	u32 idac;
};

int parse_system_log_mnits(struct dcp_parse_ctx *handle,
			   struct dcp_system_ev_mnits *entry);

#endif
