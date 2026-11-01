/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright (C) The Asahi Linux Contributors */

#ifndef __APPLE_DCP_PANEL_SCALE_H__
#define __APPLE_DCP_PANEL_SCALE_H__

#ifdef __KERNEL__
#include <linux/build_bug.h>
#include <linux/types.h>
#else
#include <assert.h>
#include <stddef.h>
typedef unsigned int u32;
typedef unsigned long long u64;
#endif

/*
 * Legacy H13P backlight codes. Each table word holds a 12-bit DBV in the
 * high half and a 12-bit iDAC in the low half. bl_value is that pair shifted
 * left by 4, so each code lands in its own 16-bit field.
 *
 * Interpolating the packed word carries the iDAC fraction into the DBV, and
 * the iDAC passed to BrightnessLCD::nitsToDBV is then not a table code. On
 * the 420-nit J313 panel that shows up at the top of the range as
 * "nitsToDBV: iDAC out of range". The halves are interpolated separately.
 *
 * The old lookup returned the last table word unshifted for 510 nits and
 * above. That word is below IOMFB_BRIGHTNESS_MIN and the panel goes to its
 * minimum, so the lookup stops at 509 and always shifts.
 *
 * Sysfs brightness on this path is CIE 1931 lightness (the pwm_bl curve)
 * across 0..panel maximum. The top step is the panel nit ceiling and a legal
 * code. The device scale is non-linear, so userspace does not apply a second
 * curve. H17P sends nits on the wire and stays linear.
 */

#define MIN_BRIGHTNESS_PART1	2U
#define MAX_BRIGHTNESS_PART1	99U
#define MIN_BRIGHTNESS_PART2	103U
#define MAX_BRIGHTNESS_PART2	510U

/*
 * Lookup for display brightness, 2 to 99 nits.
 */
static const u32 brightness_part1[] = {
	0x0000000, 0x0810038, 0x0f000bd, 0x143011c,
	0x1850165, 0x1bc01a1, 0x1eb01d4, 0x2140200,
	0x2380227, 0x2590249, 0x2770269, 0x2930285,
	0x2ac02a0, 0x2c402b8, 0x2d902cf, 0x2ee02e4,
	0x30102f8, 0x314030b, 0x325031c, 0x335032d,
	0x345033d, 0x354034d, 0x362035b, 0x3700369,
	0x37d0377, 0x38a0384, 0x3960390, 0x3a2039c,
	0x3ad03a7, 0x3b803b3, 0x3c303bd, 0x3cd03c8,
	0x3d703d2, 0x3e103dc, 0x3ea03e5, 0x3f303ef,
	0x3fc03f8, 0x4050400, 0x40d0409, 0x4150411,
	0x41d0419, 0x4250421, 0x42d0429, 0x4340431,
	0x43c0438, 0x443043f, 0x44a0446, 0x451044d,
	0x4570454, 0x45e045b, 0x4640461, 0x46b0468,
	0x471046e, 0x4770474, 0x47d047a, 0x4830480,
	0x4890486, 0x48e048b, 0x4940491, 0x4990497,
	0x49f049c, 0x4a404a1, 0x4a904a7, 0x4ae04ac,
	0x4b304b1, 0x4b804b6, 0x4bd04bb, 0x4c204c0,
	0x4c704c5, 0x4cc04c9, 0x4d004ce, 0x4d504d3,
	0x4d904d7, 0x4de04dc, 0x4e204e0, 0x4e704e4,
	0x4eb04e9, 0x4ef04ed, 0x4f304f1, 0x4f704f5,
	0x4fb04f9, 0x4ff04fd, 0x5030501, 0x5070505,
	0x50b0509, 0x50f050d, 0x5130511, 0x5160515,
	0x51a0518, 0x51e051c, 0x5210520, 0x5250523,
	0x5290527, 0x52c052a, 0x52f052e, 0x5330531,
	0x5360535, 0x53a0538, 0x53d053b, 0x540053f,
	0x5440542, 0x5470545, 0x54a0548, 0x54d054c,
	0x550054f, 0x5530552, 0x5560555, 0x5590558,
	0x55c055b, 0x55f055e, 0x5620561, 0x5650564,
	0x5680567, 0x56b056a, 0x56e056d, 0x571056f,
	0x5740572, 0x5760575, 0x5790578, 0x57c057b,
	0x57f057d, 0x5810580, 0x5840583, 0x5870585,
	0x5890588, 0x58c058b, 0x58f058d
};

static const u32 brightness_part12[] = { 0x58f058d, 0x59d058f };

/*
 * Lookup for display brightness, 103.3 to 510 nits.
 */
static const u32 brightness_part2[] = {
	0x59d058f, 0x5b805ab, 0x5d105c5, 0x5e805dd,
	0x5fe05f3, 0x6120608, 0x625061c, 0x637062e,
	0x6480640, 0x6580650, 0x6680660, 0x677066f,
	0x685067e, 0x693068c, 0x6a00699, 0x6ac06a6,
	0x6b806b2, 0x6c406be, 0x6cf06ca, 0x6da06d5,
	0x6e506df, 0x6ef06ea, 0x6f906f4, 0x70206fe,
	0x70c0707, 0x7150710, 0x71e0719, 0x7260722,
	0x72f072a, 0x7370733, 0x73f073b, 0x7470743,
	0x74e074a, 0x7560752, 0x75d0759, 0x7640760,
	0x76b0768, 0x772076e, 0x7780775, 0x77f077c,
	0x7850782, 0x78c0789, 0x792078f, 0x7980795,
	0x79e079b, 0x7a407a1, 0x7aa07a7, 0x7af07ac,
	0x7b507b2, 0x7ba07b8, 0x7c007bd, 0x7c507c2,
	0x7ca07c8, 0x7cf07cd, 0x7d407d2, 0x7d907d7,
	0x7de07dc, 0x7e307e1, 0x7e807e5, 0x7ec07ea,
	0x7f107ef, 0x7f607f3, 0x7fa07f8, 0x7fe07fc
};

#define DCP_SCALE_N(a)		(sizeof(a) / sizeof((a)[0]))

static_assert(DCP_SCALE_N(brightness_part1) == 127, "brightness_part1");
static_assert(DCP_SCALE_N(brightness_part12) == 2, "brightness_part12");
static_assert(DCP_SCALE_N(brightness_part2) == 64, "brightness_part2");

/* Q16 luminance, matching pwm_bl's PWM_LUMINANCE_SHIFT. */
#define DCP_LUMINANCE_SHIFT	16
#define DCP_LUMINANCE_SCALE	(1U << DCP_LUMINANCE_SHIFT)

/* Ten fraction bits, matching the old packed interpolator. */
#define DCP_BL_SCALE_FACTOR	1024U

/*
 * lightness is L* / 100 in Q16 (0..DCP_LUMINANCE_SCALE). The result is
 * luminance Y in the same scale. Same integer formula as pwm_bl's cie1931().
 */
static inline u32 dcp_cie1931(u32 lightness)
{
	u64 retval;

	lightness *= 100;
	if (lightness <= (8U * DCP_LUMINANCE_SCALE)) {
		retval = (lightness * 10U + 9033U / 2) / 9033U;
	} else {
		retval = (lightness + (16ULL * DCP_LUMINANCE_SCALE)) / 116;
		retval *= retval * retval;
		retval += 1ULL << (2 * DCP_LUMINANCE_SHIFT - 1);
		retval >>= 2 * DCP_LUMINANCE_SHIFT;
	}

	return retval;
}

/*
 * J313's backlight firmware floors near 4 nits. The unstretched CIE curve
 * stays under that floor for about the first 7% of a 420-nit range, so
 * levels 1, 10, 20 and 32 all read back as 32. The same floor applies when
 * the maximum is the 394-nit cap. Level 0 stays off.
 */
#define DCP_PANEL_MIN_NITS	4U

/* CIE 1931 lightness mapped into linear nits, 0..maximum. No floor. */
static inline u32 dcp_cie_nits(u32 level, u32 maximum)
{
	u32 lightness;
	u64 y;

	if (!maximum)
		return 0;
	if (level >= maximum)
		return maximum;

	lightness = (u32)((u64)level * DCP_LUMINANCE_SCALE / maximum);
	y = dcp_cie1931(lightness);
	return (u32)((y * maximum + DCP_LUMINANCE_SCALE / 2) / DCP_LUMINANCE_SCALE);
}

/* Smallest unstretched level whose nits reach DCP_PANEL_MIN_NITS. */
static inline u32 dcp_visible_floor_level(u32 maximum)
{
	u32 lo, hi, mid;

	if (maximum <= 1)
		return 1;

	lo = 1;
	hi = maximum;
	while (lo < hi) {
		mid = lo + (hi - lo) / 2;
		if (dcp_cie_nits(mid, maximum) >= DCP_PANEL_MIN_NITS)
			hi = mid;
		else
			lo = mid + 1;
	}
	return lo;
}

/*
 * CIE nits, with levels 1..maximum placed on the visible part of the curve.
 * Level 0 stays off. This still sits on the firmware floor for several steps,
 * so callers use dcp_perceptual_nits().
 */
static inline u32 dcp_curve_nits(u32 level, u32 maximum)
{
	u32 floor, span, virtual;

	if (!maximum || !level)
		return 0;
	if (level >= maximum)
		return maximum;

	floor = dcp_visible_floor_level(maximum);
	if (floor <= 1 || floor >= maximum)
		return dcp_cie_nits(level, maximum);

	span = maximum - floor;
	virtual = floor + (u32)(((u64)(level - 1) * span) / (maximum - 1));
	if (virtual > maximum)
		virtual = maximum;
	return dcp_cie_nits(virtual, maximum);
}

/*
 * Level 1 is the panel minimum. The unstretched CIE curve stays on that one
 * nit for about the first 7% of the range, so those levels used to read back
 * as one value. That span gains one nit per eight levels. Above the old
 * floor, levels follow the visible CIE curve stretched across the whole
 * range, so mid-range levels move up with the floor.
 */
static inline u32 dcp_perceptual_nits(u32 level, u32 maximum)
{
	u32 floor, rise, ramp_end, nits;

	if (!maximum || !level)
		return 0;
	if (level >= maximum)
		return maximum;

	floor = dcp_visible_floor_level(maximum);
	nits = dcp_curve_nits(level, maximum);
	if (floor <= 1 || floor >= maximum)
		return nits;

	rise = floor / 8;
	if (!rise)
		rise = 1;
	ramp_end = DCP_PANEL_MIN_NITS + rise;
	if (ramp_end > maximum)
		ramp_end = maximum;

	if (level < floor) {
		nits = DCP_PANEL_MIN_NITS +
		       (u32)(((u64)(level - 1) * rise) / (floor - 1));
		if (nits < DCP_PANEL_MIN_NITS)
			nits = DCP_PANEL_MIN_NITS;
		return nits;
	}
	if (nits < ramp_end)
		return ramp_end;
	return nits;
}

/*
 * Smallest level whose perceptual nits reach @nits. Plateaus round-trip on
 * the nit value: the forward map of this level is the forward map of any
 * level that produced @nits.
 */
static inline u32 dcp_level_from_nits(u32 nits, u32 maximum)
{
	u32 lo, hi, mid;

	if (!maximum)
		return 0;
	if (nits >= maximum)
		return maximum;

	lo = 0;
	hi = maximum;
	while (lo < hi) {
		mid = lo + (hi - lo) / 2;
		if (dcp_perceptual_nits(mid, maximum) >= nits)
			hi = mid;
		else
			lo = mid + 1;
	}
	return lo;
}

static inline u32 dcp_bl_half(u32 code, int high)
{
	return high ? code >> 16 : code & 0xffffU;
}

static inline u32 dcp_interp_half(u32 val, u32 min, u32 max, const u32 *tbl,
				  size_t tbl_size, int high)
{
	u32 span, frac, low, hi;
	u64 pos;
	size_t index;

	if (val <= min)
		return dcp_bl_half(tbl[0], high);
	if (val >= max)
		return dcp_bl_half(tbl[tbl_size - 1], high);

	span = max - min;
	pos = (tbl_size - 1) * (u64)(val - min) * DCP_BL_SCALE_FACTOR / span;
	index = pos / DCP_BL_SCALE_FACTOR;
	if (index >= tbl_size - 1)
		return dcp_bl_half(tbl[tbl_size - 1], high);

	frac = pos & (DCP_BL_SCALE_FACTOR - 1);
	low = dcp_bl_half(tbl[index], high);
	hi = dcp_bl_half(tbl[index + 1], high);
	return (frac * hi + (DCP_BL_SCALE_FACTOR - frac) * low) /
		DCP_BL_SCALE_FACTOR;
}

/* Shifted DBV/iDAC pair for @nits. Never the unshifted last table word. */
static inline u32 dcp_bl_value_for_nits(u32 nits)
{
	const u32 *tbl;
	size_t tbl_size;
	u32 min, max, dbv, idac;

	if (nits >= MAX_BRIGHTNESS_PART2)
		nits = MAX_BRIGHTNESS_PART2 - 1;

	if (nits < MAX_BRIGHTNESS_PART1) {
		tbl = brightness_part1;
		tbl_size = DCP_SCALE_N(brightness_part1);
		min = MIN_BRIGHTNESS_PART1;
		max = MAX_BRIGHTNESS_PART1;
	} else if (nits > MIN_BRIGHTNESS_PART2) {
		tbl = brightness_part2;
		tbl_size = DCP_SCALE_N(brightness_part2);
		min = MIN_BRIGHTNESS_PART2;
		max = MAX_BRIGHTNESS_PART2;
	} else {
		tbl = brightness_part12;
		tbl_size = DCP_SCALE_N(brightness_part12);
		min = MAX_BRIGHTNESS_PART1;
		max = MIN_BRIGHTNESS_PART2;
	}

	dbv = dcp_interp_half(nits, min, max, tbl, tbl_size, 1);
	idac = dcp_interp_half(nits, min, max, tbl, tbl_size, 0);
	return ((dbv << 16) | idac) << 4;
}

#undef DCP_SCALE_N

#endif /* __APPLE_DCP_PANEL_SCALE_H__ */
