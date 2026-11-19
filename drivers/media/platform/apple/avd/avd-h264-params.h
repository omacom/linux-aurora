/* SPDX-License-Identifier: GPL-2.0 */
#ifndef AVD_H264_PARAMS_H
#define AVD_H264_PARAMS_H

#include <linux/v4l2-controls.h>

/* The reference headers contain only VALID entries, in DPB array order. */
static inline unsigned int
avd_h264_ref_slot(const struct v4l2_ctrl_h264_decode_params *decode,
		  unsigned int index)
{
	unsigned int slot = 0;

	for (unsigned int i = 0; i < index; i++)
		if (decode->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)
			slot++;

	return slot;
}

static inline unsigned int
avd_h264_num_refs(const struct v4l2_ctrl_h264_decode_params *decode)
{
	return avd_h264_ref_slot(decode, V4L2_H264_NUM_DPB_ENTRIES);
}

/* SP/SI require switching-picture state which the instruction builder lacks. */
static inline bool
avd_h264_slice_type_supported(const struct v4l2_ctrl_h264_slice_params *sl)
{
	return sl->slice_type == V4L2_H264_SLICE_TYPE_I ||
	       sl->slice_type == V4L2_H264_SLICE_TYPE_P ||
	       sl->slice_type == V4L2_H264_SLICE_TYPE_B;
}

/* Pictures are held until their final slice; ASO is not implemented. */
static inline bool avd_h264_slice_order_supported(unsigned int held,
						  unsigned int last_mb,
						  unsigned int first_mb)
{
	return held ? first_mb > last_mb : first_mb == 0;
}

#endif /* AVD_H264_PARAMS_H */
