// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */

/*
 * This file is intended to be included multiple times with IOMFB_VER
 * defined to declare DCP firmware version dependent structs.
 */

#ifdef DCP_FW_VER

#include <drm/drm_crtc.h>

#include <linux/types.h>

#include "iomfb.h"
#include "iomfb_plane.h"
#include "plane.h"
#include "version_utils.h"

struct DCP_FW_NAME(dcp_swap) {
	/* IOMobileFramebuffer timestamp types 1 through 7, in wire order. */
	u64 timestamp[7];
	u64 unk_38;

	u64 flags1;
	u64 flags2;
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u8 unk_v14_7[0x48];
#endif

#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * H17P inserts 0x48 bytes between flags2 and the swap ID, which moves
	 * to +0x98.  Every following field shifts with it: surf_ids +0x9c,
	 * src_rect +0xac, surf_flags +0xec, dst_rect +0x10c,
	 * swap_enabled/completed +0x14c/+0x150, bg_color +0x154.  The gap holds
	 * two small u32 counters (at +0x50 and +0x78) that are not modelled.
	 */
	u8 h17p_pre_swap_id[0x48];
#endif

	u32 swap_id;

	u32 surf_ids[SWAP_SURFACES];
	struct dcp_rect src_rect[SWAP_SURFACES];
	u32 surf_flags[SWAP_SURFACES];
	u32 surf_unk[SWAP_SURFACES];
	struct dcp_rect dst_rect[SWAP_SURFACES];
	u32 swap_enabled;
	u32 swap_completed;

	u32 bg_color;
	u8 unk_110[0x30];
	u32 active_region_en[SWAP_SURFACES];
	struct dcp_rect active_regions[SWAP_SURFACES];
	u8 unk_190[0x138];
	u32 unk_2c8;
#if !DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u8 unk_2cc[0x14];
#else
	u8 unk_2cc[0x40];
#endif
#if !DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u32 unk_2e0;
#else
	u32 bl_update;
#endif
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u16 unk_2e2;
#else
	u8 unk_2e2[3];
#endif
#if !DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u64 bl_unk;
#else
	u32 bl_unk;
#endif
	u32 bl_value; // min value is 0x10000000
	u8  bl_power; // constant 0x40 for on
	u8 unk_2f3[0x2d];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unk_320[0x13f];
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u8 unk_14_7_2[0x30];
#endif
	u32 unk_flags;
	u32 unk_flags2;
#endif
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * The H17P A408 request is 0xe9c bytes; with a 0x22c-byte surface,
	 * SWAP_SURFACES = 4 and no surf2 block, that leaves this tail.
	 */
	u8 h17p_tail[0xda];
#endif
} __packed;

/* Information describing a surface */
struct DCP_FW_NAME(dcp_surface) {
	struct dcp_surface base;
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u8 padding[7];
#else
	u8 padding[47];
#endif
} __packed;

/* Prototypes */

struct DCP_FW_NAME(dcp_swap_submit_req) {
	struct DCP_FW_NAME(dcp_swap) swap;
	struct DCP_FW_NAME(dcp_surface) surf[SWAP_SURFACES];
	u64 surf_iova[SWAP_SURFACES];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 unk_u64_a[SWAP_SURFACES];
	/*
	 * H17P carries no surf2 block: its trailing u8 null-flag run is 8 bytes
	 * (swap_null + surf_null[4] + three tail bools).
	 */
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	struct DCP_FW_NAME(dcp_surface) surf2[5];
	u64 surf2_iova[5];
#endif
#endif
	u8 unkbool;
	u64 unkdouble;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 unkU64;
	u8 unkbool2;
#endif
	u32 clear; // or maybe switch to default fb?
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u32 unkU32Ptr;
#endif
#if DCP_FW_LAYOUT_14_7(DCP_FW_VER)
	u8 padding_14_7[0x234];
#endif
	u8 swap_null;
	u8 surf_null[SWAP_SURFACES];
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	u8 surf2_null[5];
#endif
#endif
	u8 unkoutbool_null;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unknown_pointer_null;
	u8 unknown_output_null;
#endif
#if DCP_FW_VER < DCP_FW_VERSION(26, 0, 0)
	u8 padding[1];
#endif
} __packed;

struct DCP_FW_NAME(dcp_swap_submit_resp) {
	u8 unkoutbool;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u32 unkU32out;
#endif
	u32 ret;
	u8 padding[3];
} __packed;

/*
 * A406 swap_start.  From 14.7 the 0x10-byte struct dcp_iouserclient is a bare
 * u64 handle, and the response no longer echoes it: 0x10 in, 8 out.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0) || DCP_FW_LAYOUT_14_7(DCP_FW_VER)
struct DCP_FW_NAME(dcp_swap_start_req) {
	u32 swap_id;
	u64 client;			/* IOUserClient kernel VA */
	u8 client_null;
	u8 padding[3];
} __packed;

struct DCP_FW_NAME(dcp_swap_start_resp) {
	u32 swap_id;
	u32 ret;
} __packed;

static_assert(sizeof(struct DCP_FW_NAME(dcp_swap_start_req)) == 0x10);
static_assert(sizeof(struct DCP_FW_NAME(dcp_swap_start_resp)) == 0x8);
#else
struct DCP_FW_NAME(dcp_swap_start_req) {
	u32 swap_id;
	struct dcp_iouserclient client;
	u8 swap_id_null;
	u8 client_null;
	u8 padding[2];
} __packed;

struct DCP_FW_NAME(dcp_swap_start_resp) {
	u32 swap_id;
	struct dcp_iouserclient client;
	u32 ret;
} __packed;
#endif

/*
 * A473 set_power_state.  H17P shrank the request: the iouserclient became a
 * bare u64 handle, 0x10 in.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
struct DCP_FW_NAME(dcp_set_power_state_req) {
	u64 unklong;
	u32 unkint;			/* widened from u8 on H17P */
	u8 unkint_null;
	u8 padding[3];
} __packed;

static_assert(sizeof(struct DCP_FW_NAME(dcp_set_power_state_req)) == 0x10);
#else
struct DCP_FW_NAME(dcp_set_power_state_req) {
	u64 unklong;
	u8 unkbool;
	u8 unkint_null;
	u8 padding[2];
} __packed;
#endif

/*
 * D201 map_piodma reply.  H17P declares a 16-byte output, not 20, laid out as
 * { u32 buffer index, u64 dva, u32 pad } rather than { u64 vaddr, u64 dva,
 * u32 ret }.  With the old layout the DVA lands four bytes late and the
 * display engine DMAs to a bogus address.
 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
struct DCP_FW_NAME(dcp_map_buf_resp) {
	u32 buffer;
	u64 dva;
	u32 unk;
} __packed;
static_assert(sizeof(struct DCP_FW_NAME(dcp_map_buf_resp)) == 0x10);
#else
struct DCP_FW_NAME(dcp_map_buf_resp) {
	u64 vaddr;
	u64 dva;
	u32 ret;
} __packed;
#endif

struct DCP_FW_NAME(dc_swap_complete_resp) {
	u32 swap_id;			/* +0x000 */
	u8 unkbool;			/* +0x004 */
#if DCP_FW_VER >= DCP_FW_VERSION(26, 0, 0)
	/*
	 * H17P: swap_complete_ap_gated(u32, bool, SwapCompleteData const *,
	 * SwapInfoBlob const *, u32, bool) is 0x730 bytes on the wire, 89 more
	 * than the pre-H17P record: the SwapCompleteData grew from 8 to 0x22,
	 * swap_info became eight 0xe0-byte SwapInfoBlobs rather than one
	 * 0x6c5 blob, and the tail gained a flag and three pad bytes.  Only
	 * swap_info_count of those blobs are populated; the rest are
	 * uninitialised DCP buffer memory.
	 */
	u32 hol_swap_ids[4];		/* +0x005 */
	u32 hol_arg;			/* +0x015 */
	u32 hol_count;			/* +0x019 */
	u8 hol_valid;			/* +0x01d */
	u64 displayed_data;		/* +0x01e, unaligned */
	u8 notify_displayed;		/* +0x026 */
	u8 swap_info[8][0xe0];		/* +0x027, blob[0]+0x38 == swap_id */
	u32 swap_info_count;		/* +0x727, max 8 */
	u8 completion_flag;		/* +0x72b */
	u8 swap_data_null;		/* +0x72c, 0 => data at +0x005 valid */
	u8 padding[3];			/* +0x72d, to sizeof == 0x730 */
#else
	u64 swap_data;
#if DCP_FW_VER < DCP_FW_VERSION(13, 2, 0)
	u8 swap_info[0x6c4];
#else
	u8 swap_info[0x6c5];
#endif
	u32 unkint;
	u8 swap_info_null;
#endif
} __packed;

struct DCP_FW_NAME(dcp_map_reg_req) {
	char obj[4];
	u32 index;
	u32 flags;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 unk_u64_null;
#endif
	u8 addr_null;
	u8 length_null;
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u8 padding[1];
#else
	u8 padding[2];
#endif
} __packed;

struct DCP_FW_NAME(dcp_map_reg_resp) {
#if DCP_FW_VER >= DCP_FW_VERSION(13, 2, 0)
	u64 dva;
#endif
	u64 addr;
	u64 length;
	u32 ret;
} __packed;


struct apple_dcp;

int DCP_FW_NAME(iomfb_modeset)(struct apple_dcp *dcp,
			       struct drm_crtc_state *crtc_state);
void DCP_FW_NAME(iomfb_flush)(struct apple_dcp *dcp, struct drm_crtc *crtc, struct drm_atomic_commit *commit);
void DCP_FW_NAME(iomfb_poweron)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_poweroff)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_sleep)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_start)(struct apple_dcp *dcp);
void DCP_FW_NAME(iomfb_shutdown)(struct apple_dcp *dcp);

#endif
