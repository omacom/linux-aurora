/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AVD_HEVC_PARSE_H
#define AVD_HEVC_PARSE_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

/*
 * Build slice parameters and entry point offsets for a whole Annex B
 * picture. On success *slices and *entry_points are owned by the caller,
 * who frees them with kvfree().
 */
int avd_hevc_parse_slices(const u8 *buf, size_t len,
			  const struct v4l2_ctrl_hevc_sps *sps,
			  const struct v4l2_ctrl_hevc_pps *pps,
			  const struct v4l2_ctrl_hevc_decode_params *dec,
			  struct v4l2_ctrl_hevc_slice_params **slices, int *num_slices,
			  u32 **entry_points, int *num_entry_points);

#endif
