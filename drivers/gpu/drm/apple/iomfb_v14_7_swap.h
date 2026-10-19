/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef __APPLE_IOMFB_V14_7_SWAP_H__
#define __APPLE_IOMFB_V14_7_SWAP_H__

#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define DCP_V14_SWAP_SIZE		0x1b58
#define DCP_V14_SURFACE_SIZE		0x22c

/* Offsets in the swap record. Rectangles are x, y, w, h (u32 each). */
#define DCP_V14_SWAP_ID			0x98
#define DCP_V14_SWAP_SURF_ID(p)		(0x9c + (p) * 4)
#define DCP_V14_SWAP_SRC(p)		(0xac + (p) * 16)	/* in the surface */
#define DCP_V14_SWAP_DST(p)		(0x10c + (p) * 16)	/* on the panel */
#define DCP_V14_SWAP_ENABLED		0x14c
#define DCP_V14_SWAP_COMPLETED		0x150
#define DCP_V14_SWAP_BACKGROUND		0x154
#define DCP_V14_SWAP_SURFACE(p)		(0x508 + (p) * DCP_V14_SURFACE_SIZE)
#define DCP_V14_SWAP_IOVA(p)		(0xdb8 + (p) * 8)
#define DCP_V14_SWAP_SURF_NULL(p)	(0x1b4b + (p))
#define DCP_V14_SWAP_SURF_NULL_COUNT	10

/*
 * The panel level block of the 0x508-byte swap prefix (14.7 panel): a
 * present byte at +0x354 and the level in nits as an unaligned IEEE 754
 * binary64 at +0x35c. Zero when the level does not change.
 */
#define DCP_V14_SWAP_NITS_PRESENT	0x354
#define DCP_V14_SWAP_NITS		0x35c
/* The nits field takes integral levels up to the panel maximum. */
#define DCP_V14_NITS_MAX		500
/* Level presented by the first present after takeover. */
#define DCP_V14_NITS_DEFAULT		140

#define DCP_V14_SWAP_LAYERS		0x7
#define DCP_V14_SWAP_SET_BACKGROUND	BIT(31)

static_assert(DCP_V14_SWAP_NITS_PRESENT > DCP_V14_SWAP_BACKGROUND + 4);
static_assert(DCP_V14_SWAP_NITS + 8 <= DCP_V14_SWAP_SURFACE(0));

/* Integral nits as IEEE 754 binary64, without kernel floating point. */
static inline u64 dcp_v14_nits_binary64(u32 nits)
{
	unsigned int bit;

	if (!nits)
		return 0;
	bit = fls(nits) - 1;
	return ((u64)(1023 + bit) << 52) |
	       ((u64)(nits - (1U << bit)) << (52 - bit));
}

/* The level block of an encoded swap: @nits when @change, else none. */
static inline int dcp_v14_encode_nits(u8 *swap, bool change, u32 nits)
{
	if (change && nits > DCP_V14_NITS_MAX)
		return -EINVAL;
	swap[DCP_V14_SWAP_NITS_PRESENT] = change;
	put_unaligned_le64(change ? dcp_v14_nits_binary64(nits) : 0,
			   swap + DCP_V14_SWAP_NITS);
	return 0;
}

/* Offsets in a surface record. */
#define DCP_V14_SURF_OPAQUE		0x02
#define DCP_V14_SURF_FORMAT		0x0b
#define DCP_V14_SURF_STRIDE		0x15
#define DCP_V14_SURF_WIDTH		0x21
#define DCP_V14_SURF_HEIGHT		0x25
#define DCP_V14_SURF_SIZE		0x29
#define DCP_V14_SURF_PLANE_WIDTH	0x59
#define DCP_V14_SURF_PLANE_HEIGHT	0x5d
#define DCP_V14_SURF_PLANE_STRIDE	0x69
#define DCP_V14_SURF_PLANE_SIZE		0x6d

/* 'BGRA': 8-bit linear, ARGB8888 in DRM terms. */
#define DCP_V14_FORMAT_BGRA		0x42475241

/*
 * A single-plane linear BGRA surface covering the whole framebuffer. The
 * firmware has no XRGB format: an opaque surface blends as premultiplied
 * alpha over the black background, which shows XRGB pixels unchanged.
 */
static inline void dcp_v14_encode_surface(u8 *s, u32 stride, u32 width, u32 height,
					  bool opaque)
{
	u32 bytes = stride * height;

	memset(s, 0, DCP_V14_SURFACE_SIZE);
	s[DCP_V14_SURF_OPAQUE] = opaque;
	put_unaligned_le32(1, s + 0x03);
	put_unaligned_le32(1, s + 0x07);
	put_unaligned_le32(DCP_V14_FORMAT_BGRA, s + DCP_V14_SURF_FORMAT);
	s[0x13] = 13;
	s[0x14] = 12;
	put_unaligned_le32(stride, s + DCP_V14_SURF_STRIDE);
	put_unaligned_le16(1, s + 0x19);
	s[0x1b] = 1;
	s[0x1c] = 1;
	put_unaligned_le32(width, s + DCP_V14_SURF_WIDTH);
	put_unaligned_le32(height, s + DCP_V14_SURF_HEIGHT);
	put_unaligned_le32(bytes, s + DCP_V14_SURF_SIZE);
	put_unaligned_le32(1, s + 0x35);
	put_unaligned_le64(1, s + 0x51);
	put_unaligned_le32(width, s + DCP_V14_SURF_PLANE_WIDTH);
	put_unaligned_le32(height, s + DCP_V14_SURF_PLANE_HEIGHT);
	put_unaligned_le32(stride, s + DCP_V14_SURF_PLANE_STRIDE);
	put_unaligned_le32(bytes, s + DCP_V14_SURF_PLANE_SIZE);
	put_unaligned_le16(4, s + 0x71);
	s[0x73] = 1;
	s[0x74] = 1;
	put_unaligned_le64(1, s + 0x149);
}

static inline void dcp_v14_put_rect(u8 *r, u32 x, u32 y, u32 w, u32 h)
{
	put_unaligned_le32(x, r);
	put_unaligned_le32(y, r + 4);
	put_unaligned_le32(w, r + 8);
	put_unaligned_le32(h, r + 12);
}

/*
 * The swap record for swap @id. @background fills every panel pixel that no
 * surface covers: the whole panel when @surface is NULL. With a surface on
 * @layer, the whole @width x @height surface is read (source 0,0) and shown
 * unscaled at 0,@dst_y on the panel: 0 for the full panel, the notch height
 * to keep the rows above it black.
 */
static inline void dcp_v14_encode_swap(u8 *swap, u32 id, u32 background, const u8 *surface,
				       u64 iova, u32 width, u32 height, u32 dst_y, u32 layer)
{
	memset(swap, 0, DCP_V14_SWAP_SIZE);
	put_unaligned_le32(id, swap + DCP_V14_SWAP_ID);
	put_unaligned_le32(DCP_V14_SWAP_SET_BACKGROUND | DCP_V14_SWAP_LAYERS,
			   swap + DCP_V14_SWAP_ENABLED);
	put_unaligned_le32(DCP_V14_SWAP_SET_BACKGROUND | DCP_V14_SWAP_LAYERS,
			   swap + DCP_V14_SWAP_COMPLETED);
	put_unaligned_le32(background, swap + DCP_V14_SWAP_BACKGROUND);
	memset(swap + DCP_V14_SWAP_SURF_NULL(0), 1, DCP_V14_SWAP_SURF_NULL_COUNT);
	swap[0x1b56] = 1;
	swap[0x1b57] = 1;
	if (surface) {
		memcpy(swap + DCP_V14_SWAP_SURFACE(layer), surface, DCP_V14_SURFACE_SIZE);
		put_unaligned_le64(iova, swap + DCP_V14_SWAP_IOVA(layer));
		put_unaligned_le32(1, swap + DCP_V14_SWAP_SURF_ID(layer));
		dcp_v14_put_rect(swap + DCP_V14_SWAP_SRC(layer), 0, 0, width, height);
		dcp_v14_put_rect(swap + DCP_V14_SWAP_DST(layer), 0, dst_y, width, height);
		swap[DCP_V14_SWAP_SURF_NULL(layer)] = 0;
	}
}

#endif /* __APPLE_IOMFB_V14_7_SWAP_H__ */
