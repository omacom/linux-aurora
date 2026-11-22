/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Apple ProRes (APR) encoder: register map and job descriptor.
 *
 * The APR is a register-driven engine with no firmware. Jobs are fixed-size
 * descriptors placed in a DMA ring; the driver installs the ring, advances a
 * producer head and the engine reports completion through a status register
 * and a consumer tail.
 *
 * Register names and descriptor fields follow the m1n1 proxyclient ProRes
 * experiment (MIT), proxyclient/m1n1/hw/prores.py and
 * proxyclient/experiments/prores.py. The T8140 descriptor layout (version 1,
 * frame header moved from 0x58 to 0x7c) comes from Acelogic's J700 work.
 */
#ifndef _APPLE_PRORES_H
#define _APPLE_PRORES_H

#include <linux/bits.h>
#include <linux/sizes.h>
#include <linux/types.h>

#define APR_REG_ID		0x0000
#define APR_REG_MODE		0x0008
#define APR_MODE_OFF		0x0
#define APR_MODE_RUN		0xd
#define APR_REG_IRQ_ENABLE	0x000c
#define APR_REG_IRQ_STATUS	0x0010
#define APR_IRQ_DONE		BIT(0)
#define APR_IRQ_ERROR		BIT(1)
#define APR_REG_ST0		0x0014
#define APR_REG_ST1		0x0018
#define APR_REG_RING_SIZE	0x0100
#define APR_REG_RING_LO		0x0104
#define APR_REG_RING_HI		0x0108
#define APR_REG_RING_HEAD	0x010c
#define APR_REG_RING_TAIL	0x0110
#define APR_RING_PTR_MASK	GENMASK(23, 0)
/* One statistics area, given to two consumers; their capacity reads back. */
#define APR_REG_STATS0_LO	0x038c
#define APR_REG_STATS0_HI	0x0390
#define APR_REG_STATS0_SIZE	0x0394
#define APR_REG_STATS1_LO	0x070c
#define APR_REG_STATS1_HI	0x0710
#define APR_REG_STATS1_SIZE	0x0714
#define APR_STATS_BYTES		SZ_2M

/* One 32-entry quantisation matrix per table, for luma and for chroma. */
#define APR_REG_QUANT_LUMA(t)	(0x0800 + (t) * 0x80)
#define APR_REG_QUANT_CHROMA(t)	(0x1000 + (t) * 0x80)
#define APR_REG_DC_SCALE(i)	(0x1800 + (i) * 4)
#define APR_DC_SCALE_ENTRIES	112

#define APR_RING_BYTES		SZ_128K
#define APR_DESC_BYTES		0x180

#define APRI_T6041_FIRMWARE_NAME	"apple/prores-t6041-init.bin"
#define APRI_T6041_FIRMWARE_SIZE	2728
#define APRI_T6050_FIRMWARE_NAME	"apple/prores-t6050-init.bin"
#define APRI_T6050_FIRMWARE_SIZE	2880
#define APRI_T8132_FIRMWARE_NAME	"apple/prores-t8132-init.bin"
#define APRI_T8132_FIRMWARE_SIZE	2560
#define APRI_T8142_FIRMWARE_NAME	"apple/prores-t8142-init.bin"
#define APRI_T8142_FIRMWARE_SIZE	2592
#define APRI_SYSTEM_COUNT	82
#define APRI_CORE_COUNT		80
#define APRI_WRAPPER_COUNT	62
#define APRI_CONTROL_COUNT	12
/* T8132 brings up its AXI wrapper with fewer rows than T6041 */
#define APRI_T8132_WRAPPER_COUNT	48
/* T8142's AXI-to-fabric wrapper takes 38 rows where T6041 and T6050 take 62. */
#define APRI_T8142_WRAPPER_COUNT	38
#define APRI_MAX_COUNT		APRI_SYSTEM_COUNT

enum apri_block_index {
	APRI_SYSTEM_BLOCK,
	APRI_CORE_BLOCK,
	APRI_WRAPPER_BLOCK,
	APRI_CONTROL_BLOCK,	/* T6050 and T8142 */
	APRI_MAX_BLOCKS,
};

#define APRI_T6041_BLOCKS	3
#define APRI_T6050_BLOCKS	4
#define APRI_T8142_BLOCKS	4

/* The fixed shape of one SoC's APRI container. */
struct apri_layout {
	u32 soc;
	u32 blocks;
	size_t size;
	u32 counts[APRI_MAX_BLOCKS];
	u32 limits[APRI_MAX_BLOCKS];
};

struct apr_tunable {
	u32 offset;
	u32 mask;
	u32 value;
};

struct apr_init_block {
	u32 count;
	struct apr_tunable entries[APRI_MAX_COUNT];
};

struct apr_init_data {
	struct apr_init_block blocks[APRI_MAX_BLOCKS];
};

/* Quantisation tables the engine holds, in register order. */
enum apr_qtable {
	APR_QTABLE_XQ,
	APR_QTABLE_HQ,
	APR_QTABLE_STANDARD,
	APR_QTABLE_LT,
	APR_QTABLE_PROXY,
	APR_QTABLES,
};

/*
 * A dropped coefficient: the engine always quantises it to zero. Its 12-bit
 * field is all ones; there is no ProRes weight for it.
 */
#define APR_QM_ZERO		0

enum apr_input {
	APR_INPUT_422_8,	/* NV16: 8-bit Y plane, 8-bit CbCr plane */
	APR_INPUT_444_8,	/* NV24 */
	APR_INPUT_422_16,	/* P216: 16-bit samples, data in the MSBs */
	APR_INPUT_444_16,	/* P416 */
};

/* V4L2 profile menu order: Proxy, LT, 422, HQ, 4444, 4444 XQ. */
enum apr_profile {
	APR_PROFILE_PROXY,
	APR_PROFILE_LT,
	APR_PROFILE_STANDARD,
	APR_PROFILE_HQ,
	APR_PROFILE_4444,
	APR_PROFILE_4444_XQ,
	APR_PROFILES,
};

struct apr_frame {
	u32 width;
	u32 height;
	enum apr_input input;
	enum apr_profile profile;
	u32 luma_stride;	/* bytes, multiple of 64 */
	u32 chroma_stride;	/* bytes, multiple of 64 */
	u64 luma;
	u64 chroma;
	u64 output;
	u32 output_size;
	/* ProRes frame header colour codes (ISO/IEC 23091-2). */
	u8 primaries;
	u8 transfer;
	u8 matrix;
};

/* Matrices are 8x8 in raster order, as carried in the ProRes frame header. */
extern const u8 *const apr_qmat_luma[APR_QTABLES];
extern const u8 *const apr_qmat_chroma[APR_QTABLES];

bool apr_profile_is_444(enum apr_profile profile);
bool apr_input_is_444(enum apr_input input);
enum apr_qtable apr_profile_qtable(enum apr_profile profile);
u32 apr_qmat_reg(const u8 *qmat, unsigned int index);
u32 apr_dc_scale_reg(unsigned int index);
void apr_frame_budget(enum apr_profile profile, u32 width, u32 height,
		      u32 *target, u32 *max);
u32 apr_slice_count(u32 width, u32 height, u32 log2_slice);
int apr_build_desc(const struct apr_frame *f, u8 *desc);
int apr_build_desc_v0(const struct apr_frame *f, u8 *desc);
int apr_check_output(const u8 *out, size_t capacity, u32 *frame_bytes);
extern const struct apri_layout apri_t6041;
extern const struct apri_layout apri_t6050;
extern const struct apri_layout apri_t8132;
extern const struct apri_layout apri_t8142;
int apr_parse_init(const struct apri_layout *layout, const u8 *data,
		   size_t size, struct apr_init_data *init);

static inline int apr_parse_t6041_init(const u8 *data, size_t size,
				       struct apr_init_data *init)
{
	return apr_parse_init(&apri_t6041, data, size, init);
}

#endif
