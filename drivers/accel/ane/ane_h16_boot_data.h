/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __ANE_H16_BOOT_DATA_H__
#define __ANE_H16_BOOT_DATA_H__

#include "ane_h16_boot.h"
#include "ane_h16_preload.h"

/* H16 27.0 (26A428), r5 sections 1.2-1.4. Ordered masked updates. */
static const struct ane_h16_tunable ane_h16_axi[] = {
	{ 0x0000, 0x800000ff, 0x80000022 },
	{ 0x0004, 0x80ff00ff, 0x80220003 },
	{ 0x0008, 0x80ff00ff, 0x80220009 },
	{ 0x000c, 0x80ff00ff, 0x80220003 },
	{ 0x0010, 0x80ff00ff, 0x80220003 },
	{ 0x0014, 0x80ff00ff, 0x80220003 },
	{ 0x0018, 0x800000ff, 0x8000001b },
	{ 0x001c, 0x800000ff, 0x80000022 },
	{ 0x0020, 0x80ff00ff, 0x80220003 },
	{ 0x0024, 0x80ff00ff, 0x80220003 },
	{ 0x0028, 0x80ff00ff, 0x80220003 },
	{ 0x002c, 0x80ff00ff, 0x80220003 },
	{ 0x0030, 0x800000ff, 0x8000001b },
	{ 0x0034, 0x800000ff, 0x8000001b },
	{ 0x0038, 0x800000ff, 0x8000001b },
	{ 0x0100, 0x000000ff, 0x000000ff },
	{ 0x0108, 0xffff0006, 0x00100000 },
	{ 0x0304, 0x3f3f0000, 0x02020000 },
	{ 0x0308, 0x000fffff, 0x00001681 },
	{ 0x030c, 0x000fffff, 0x00001680 },
	{ 0x0310, 0x70000fff, 0x000001e0 },
	{ 0x0314, 0xf0000fff, 0x800001e0 },
	{ 0x0318, 0x70000fff, 0x000005a0 },
	{ 0x031c, 0xf0000fff, 0x800005a0 },
	{ 0x0320, 0x70000fff, 0x000005a0 },
	{ 0x0324, 0xf0000fff, 0x800005a0 },
	{ 0x0328, 0x70000fff, 0x000005a0 },
	{ 0x032c, 0x70000fff, 0x000005a0 },
	{ 0x0330, 0x70000fff, 0x000005a0 },
	{ 0x0338, 0xffffffff, 0x00050020 },
	{ 0x033c, 0xffffffff, 0x000a0030 },
	{ 0x0408, 0xffff0020, 0x001e0020 },
	{ 0x040c, 0x000fffff, 0x00001681 },
	{ 0x0410, 0x000fffff, 0x00001680 },
	{ 0x0414, 0x70000fff, 0x000001e0 },
	{ 0x0418, 0x70000fff, 0x000001e0 },
	{ 0x041c, 0x70000fff, 0x000005a0 },
	{ 0x0420, 0x70000fff, 0x000005a0 },
	{ 0x0424, 0x70000fff, 0x000005a0 },
	{ 0x0428, 0x70000fff, 0x000005a0 },
	{ 0x042c, 0x70000fff, 0x000005a0 },
	{ 0x0430, 0x70000fff, 0x000005a0 },
	{ 0x0434, 0x70000fff, 0x000005a0 },
	{ 0x0748, 0x07ff07ff, 0x00200030 },
	{ 0x0848, 0x07ff07ff, 0x00100030 },
	{ 0x0948, 0x07ff07ff, 0x01880188 },
	{ 0x0b00, 0xffffffff, 0x7fffffff },
	{ 0x4008, 0x00000001, 0 },
	{ 0x400c, 0x00000001, 1 },
	{ 0x4014, 0x00000001, 0 },
	{ 0x4018, 0x00000001, 0 },
	{ 0x401c, 0x00000001, 1 },
};

static const struct ane_h16_tunable ane_h16_xl[] = {
	{ 0x0000, 0x800000ff, 0x80000036 },
	{ 0x0004, 0x80ff00ff, 0x80360003 },
	{ 0x0008, 0x80ff00ff, 0x80360009 },
	{ 0x000c, 0x80ff00ff, 0x80010000 },
	{ 0x0010, 0x80ff00ff, 0x80360003 },
	{ 0x0014, 0x800000ff, 0x8000002e },
	{ 0x0018, 0x800000ff, 0x80000036 },
	{ 0x001c, 0x80ff00ff, 0x80360003 },
	{ 0x0020, 0x80ff00ff, 0x80020000 },
	{ 0x0024, 0x80ff00ff, 0x80360003 },
	{ 0x0028, 0x800000ff, 0x8000002e },
	{ 0x002c, 0x800000ff, 0x8000002e },
	{ 0x0030, 0x800000ff, 0x8000002e },
	{ 0x0100, 0x000000ff, 0x000000ff },
	{ 0x0108, 0xffff0006, 0x00100000 },
	{ 0x0304, 0x3f3f0000, 0x02020000 },
	{ 0x0308, 0x000fffff, 0x00001681 },
	{ 0x030c, 0x000fffff, 0x00001680 },
	{ 0x0310, 0x70000fff, 0x000005a0 },
	{ 0x0314, 0xf0000fff, 0x800005a0 },
	{ 0x0318, 0x70000fff, 0x000005a0 },
	{ 0x031c, 0x70000fff, 0x000005a0 },
	{ 0x0320, 0x70000fff, 0x000005a0 },
	{ 0x0328, 0xffffffff, 0x0000ffff },
	{ 0x032c, 0xffffffff, 0x0000ffff },
	{ 0x0408, 0xffff0020, 0x00310020 },
	{ 0x040c, 0x000fffff, 0x00001681 },
	{ 0x0410, 0x000fffff, 0x00001680 },
	{ 0x0414, 0x70000fff, 0x000005a0 },
	{ 0x0418, 0x70000fff, 0x000005a0 },
	{ 0x041c, 0x70000fff, 0x000005a0 },
	{ 0x0420, 0x70000fff, 0x000005a0 },
	{ 0x0424, 0x70000fff, 0x000005a0 },
	{ 0x0648, 0x07ff07ff, 0x01880188 },
	{ 0x0800, 0xffffffff, 0x7fffffff },
	{ 0x4008, 0x00000001, 0 },
	{ 0x400c, 0x00000001, 1 },
	{ 0x4014, 0x00000001, 0 },
	{ 0x4018, 0x00000001, 0 },
	{ 0x401c, 0x00000001, 1 },
};

static const struct ane_h16_tunable ane_h16_c1ppt[] = {
	{ 0x20, 0x0000ffff, 0x1838 },
	{ 0x24, 0x0000ffff, 0x1034 },
	{ 0x28, 0xffffffff, 0x07d00708 },
	{ 0x2c, 0x000000ff, 4 },
	{ 0x40, 0x000003ff, 0xbf },
	{ 0x44, 0x000003ff, 0x19a },
	{ 0x48, 0x0001ffff, 0x7800 },
	{ 0x60, 0x00001f1f, 0x0103 },
	{ 0x04, 0x1, 1 },
	{ 0x3ff0, 0x007fffff, 0x0077ebf9 },
	{ 0x3ffc, 0x1, 1 },
};

static const struct ane_h16_tunables ane_h16_tunables_26a428[] = {
	{ .window = ANE_H16_ENGINE, .values = ane_h16_axi,
	  .count = ARRAY_SIZE(ane_h16_axi) },
	{ .window = ANE_H16_XL, .values = ane_h16_xl,
	  .count = ARRAY_SIZE(ane_h16_xl) },
	{ .window = ANE_H16_ENGINE, .base = 0x197c000,
	  .values = ane_h16_c1ppt, .count = ARRAY_SIZE(ane_h16_c1ppt),
	  .c1ppt = true, .lock_offset = 0x3ffc, .lock_mask = 1 },
};

/*
 * T8132 (M4): the engine AXI2AF set at +0x0, then its write-disable and lock
 * bits. The lock holds until ANE_SYS power is cycled, so a group that finds
 * it set only checks its values. M4 has no XL or C1PPT writes.
 */
static const struct ane_h16_tunable ane_t8132_axi[] = {
	{ 0x0000, 0x800000ff, 0x8000001b },
	{ 0x0004, 0x80ff00ff, 0x800d000d },
	{ 0x0008, 0x80ff00ff, 0x800d000d },
	{ 0x000c, 0x80ff00ff, 0x800d000d },
	{ 0x0010, 0x80ff00ff, 0x80010003 },
	{ 0x0014, 0x80ff00ff, 0x80010003 },
	{ 0x0018, 0x800000ff, 0x80000014 },
	{ 0x001c, 0x800000ff, 0x80000022 },
	{ 0x0020, 0x80ff00ff, 0x8012000c },
	{ 0x0024, 0x80ff00ff, 0x80180006 },
	{ 0x0028, 0x80ff00ff, 0x80060002 },
	{ 0x002c, 0x80ff00ff, 0x80060002 },
	{ 0x0030, 0x800000ff, 0x80000016 },
	{ 0x0034, 0x800000ff, 0x80000012 },
	{ 0x0038, 0x800000ff, 0x80000012 },
	{ 0x0100, 0x000000ff, 0x000000ff },
	{ 0x0104, 0x00ff0000, 0x00010000 },
	{ 0x0108, 0xffff0006, 0x00040000 },
	{ 0x0304, 0x3f3f0000, 0x02020000 },
	{ 0x0314, 0x80000000, 0x80000000 },
	{ 0x031c, 0x80000000, 0x80000000 },
	{ 0x0324, 0x80000000, 0x80000000 },
	{ 0x0338, 0xffffffff, 0x00050020 },
	{ 0x033c, 0xffffffff, 0x000a0030 },
	{ 0x0408, 0xffff0020, 0x00130020 },
	{ 0x0748, 0x07ff07ff, 0x00200030 },
	{ 0x0848, 0x07ff07ff, 0x00100030 },
	{ 0x0948, 0x07ff07ff, 0x01000100 },
	{ 0x0b00, 0xffffffff, 0x7fffffff },
	{ 0x0b04, 0x00000100, 0x00000100 },
	{ 0x4008, 0x00000001, 0x00000000 },
	{ 0x400c, 0x00000001, 0x00000000 },
	{ 0x4014, 0x00000001, 0x00000000 },
	{ 0x4018, 0x00000001, 0x00000000 },
	{ 0x401c, 0x00000001, 0x00000001 },
	{ 0x4020, 0xffff0000, 0x00000000 },
	{ 0x4024, 0xffff0000, 0x00000000 },
	{ 0x4028, 0xffff0000, 0x00000000 },
	{ 0x402c, 0xffff0000, 0x00000000 },
	{ 0x4030, 0xffff0000, 0x00000000 },
	{ 0x4034, 0xffff0000, 0x00000000 },
};

static const struct ane_h16_tunable ane_t8132_axi_wd[] = {
	{ 0x0df8, 0x00000001, 0x00000001 },
	{ 0x0ef8, 0x00000001, 0x00000001 },
	{ 0x0ff8, 0x00000001, 0x00000001 },
	{ 0x10f8, 0x00000001, 0x00000001 },
	{ 0x11f8, 0x00000001, 0x00000001 },
};

static const struct ane_h16_tunable ane_t8132_axi_lock[] = {
	{ 0x0dfc, 0x00000001, 0x00000001 },
	{ 0x0efc, 0x00000001, 0x00000001 },
	{ 0x0ffc, 0x00000001, 0x00000001 },
	{ 0x10fc, 0x00000001, 0x00000001 },
	{ 0x11fc, 0x00000001, 0x00000001 },
};

static const struct ane_h16_tunables ane_t8132_tunables[] = {
	{ .window = ANE_H16_ENGINE, .values = ane_t8132_axi,
	  .count = ARRAY_SIZE(ane_t8132_axi), .lock_offset = 0xdfc, .lock_mask = 1 },
	{ .window = ANE_H16_ENGINE, .values = ane_t8132_axi_wd,
	  .count = ARRAY_SIZE(ane_t8132_axi_wd), .lock_offset = 0xdfc, .lock_mask = 1 },
	{ .window = ANE_H16_ENGINE, .values = ane_t8132_axi_lock,
	  .count = ARRAY_SIZE(ane_t8132_axi_lock), .lock_offset = 0xdfc, .lock_mask = 1 },
};

/*
 * T6041 (J616s) ANE DARTs, revision 2.2 (PARAMS3 0x2a2a0202). The ADT
 * dart-ane0 node has one DAPF bank, the reg entry its instance property tags
 * FPAD "DAPFLLT" at DART0 + 0x10000; BRD and BWR have none, and their
 * +0x10000 is outside every ADT reg entry. iBoot leaves the bank unprogrammed
 * and it sits in the ANE_SYS domain, so the kernel writes it after power-up.
 * Every dapf-instance-0 record has r0 0x31, r4 0 and r20 1. A t8110 DAPF
 * record end does not read back its low two bits on the T8132; every J616s
 * end has them set.
 */
static const struct ane_h16_mmio ane_h16_mmio_t6041[] __maybe_unused = {
	{
		.dart_revision = 0x0202,
		.dart_fault = 0x100,
		.dart_address_lo = 0x170,
		.dart_address_hi = 0x174,
		.dart_protect = 0x200,
		.dart_lock = 0x208,
		.dapf_records = 0,
		.dapf_stride = 0x40,
		.dapf_end_mask = ~3ULL,
		.dapf_r0 = 0x31,
		.dapf_r4 = 0,
		.dapf_r20 = 1,
	},
};

static const struct ane_h16_build ane_h16_build_26a428 = {
	.cpu_control = 0x1600044,
	.rvbar = 0x1050000,
	.ce = 0x18a4000,
	.ce_delays = 5001,
	.ce_delay_us = 10,
	.ready = 0x08042006,
	.run = 0x10,
	.scratch_polls = 1000,
	.scratch_delay_us = 1000,
	.command_timeout_ns = 3000000000ULL,
	.pool_dva = 0x10000384000,
	.ipc_dva = 0x100003d0000,
	.heap_dva = 0x100003e4000,
	.shared_dva = 0x10001300000,
	.expected_rvbar = 0x10000000001,
	.pool_size = 0x40000,
	.init_offset = 0x180,
	.init_size = 0x174,
	.config_span = 0x380000,
	.config_window = 0x10000000,
	.template_count = 64,
	.template_word = 0x12c,
	.template_value = 4,
	.max_channels = 0x21,
	/* Host allocation policy, not a recovered firmware ceiling. */
	.max_heap = SZ_16M,
	.native_free = true,
	.c1ppt = true,
	.tunables = ane_h16_tunables_26a428,
	.tunable_count = ARRAY_SIZE(ane_h16_tunables_26a428),
	.startup = { { 4, 12 }, { 0, 12 }, { 3, 16 }, { 0x22, 100 } },
};

/* The J713 25G83 firmware takes the 26A428 boot values; M4 engine writes. */
static const struct ane_h16_build ane_h16_build_t8132_25g83 = {
	.cpu_control = 0x1600044,
	.rvbar = 0x1050000,
	.ce = 0x18a4000,
	.ce_delays = 5001,
	.ce_delay_us = 10,
	.ready = 0x08042006,
	.run = 0x10,
	.scratch_polls = 1000,
	.scratch_delay_us = 1000,
	.command_timeout_ns = 3000000000ULL,
	.pool_dva = 0x10000384000,
	.ipc_dva = 0x100003d0000,
	.heap_dva = 0x100003e4000,
	.shared_dva = 0x10001300000,
	.expected_rvbar = 0x10000000001,
	.pool_size = 0x40000,
	.init_offset = 0x180,
	.init_size = 0x174,
	.config_span = 0x380000,
	.config_window = 0x10000000,
	.template_count = 64,
	.template_word = 0x12c,
	.template_value = 4,
	.max_channels = 0x21,
	/* Host allocation policy, not a recovered firmware ceiling. */
	.max_heap = SZ_16M,
	.native_free = true,
	.tunables = ane_t8132_tunables,
	.tunable_count = ARRAY_SIZE(ane_t8132_tunables),
	.startup = { { 4, 12 }, { 0, 12 }, { 3, 16 }, { 0x22, 100 } },
};

/*
 * 27.0 (26A428) preload: ordered aether patchbay records in DATA (whose
 * firmware VM base is 0xc0000) and the range the bootloader rewrites.
 * Offsets are DATA-relative. Only the mutable value bytes normalize.
 */
static const struct ane_h16_preload_cell ane_h16_patchbay_26a428[] = {
	{ "BECA", 0x5a20, 8, true },
	{ "SECA", 0x5a30, 8, false },
	{ "GKTS", 0x5a40, 8, true },
	{ "otcd", 0x5a50, 8, false },
	{ "oscd", 0x5a60, 8, false },
	{ "oecd", 0x5a70, 8, false },
	{ "DILS", 0x5a80, 8, true },
	{ "SRSA", 0x5a90, 8, false },
	{ "GFCM", 0x5aa0, 8, false },
	{ "SSSC", 0x5ab0, 8, true },
	{ "qF8v", 0x5ac0, 4, true },
	{ "LRSD", 0x5acc, 1, false },
	{ "SVSD", 0x5ad5, 8, false },
	{ "LCSD", 0x5ae5, 8, false },
	{ "ZSTR", 0x5af5, 4, false },
	{ "BPTP", 0x5b01, 8, false },
	{ "TNGI", 0x5b11, 1, false },
	{ "oeNS", 0x5b1a, 1, false },
	{ "SEBC", 0x5b23, 8, false },
	{ "ECAP", 0x5b33, 8, true },
	{ "BVTP", 0x5b43, 8, true },
	{ "sP1T", 0x5b53, 8, true },
	{ "BtpG", 0x5b63, 8, true },
	{ "StpG", 0x5b73, 8, false },
	{ "BlpP", 0x5b83, 8, true },
	{ "SPTP", 0x5b93, 8, false },
	{ "SZSD", 0x5ba3, 8, false },
	{ "LZSD", 0x5bb3, 8, false },
	{ "S2xG", 0x5bc3, 8, true },
	{ "SNUT", 0x5bd3, 8, true },
	{ "ZNUT", 0x5be3, 4, false },
	{ "OTTR", 0x5bef, 8, true },
	{ "LLTR", 0x5bff, 1, false },
	{ "_COS", 0x5c08, 4, true },
	{ "RCOS", 0x5c14, 4, true },
	{ "dApC", 0x5c20, 8, true },
	{ "dArW", 0x5c30, 8, true },
	{ "fVED", 0x5c40, 1, true },
	{ "GLFp", 0x5c49, 8, true },
	{ "ABOI", 0x5c59, 8, false },
	{ "ZSOI", 0x5c69, 4, false },
	{ "ARcM", 0x5c75, 4, true },
};

static const struct ane_h16_preload_rule ane_h16_preload_26a428 = {
	.cells = ane_h16_patchbay_26a428,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_26a428),
	.zero_start = 0x17c80,
	.zero_end = 0x18320,
	.digest = {
		0x1f, 0x86, 0x5e, 0xd0, 0x4c, 0x4c, 0x38, 0xf6,
		0xfd, 0xcc, 0x45, 0x43, 0xa8, 0x81, 0x4f, 0x81,
		0x8d, 0x3c, 0xea, 0x58, 0x3d, 0xe6, 0x4d, 0x22,
		0xff, 0xe9, 0x6e, 0x3a, 0x93, 0xca, 0x46, 0x97,
	},
};

/*
 * 26.6.2 (25G83) preload, the build the J616s Linux volume boots with: the
 * same patchbay records and rewritten range as 26A428, 0x3f80 bytes later
 * in DATA. Its boot values are the 26A428 ones (25G83 facts, section 2).
 */
static const struct ane_h16_preload_cell ane_h16_patchbay_25g83[] = {
	{ "BECA", 0x99a0, 8, true },
	{ "SECA", 0x99b0, 8, false },
	{ "GKTS", 0x99c0, 8, true },
	{ "otcd", 0x99d0, 8, false },
	{ "oscd", 0x99e0, 8, false },
	{ "oecd", 0x99f0, 8, false },
	{ "DILS", 0x9a00, 8, true },
	{ "SRSA", 0x9a10, 8, false },
	{ "GFCM", 0x9a20, 8, false },
	{ "SSSC", 0x9a30, 8, true },
	{ "qF8v", 0x9a40, 4, true },
	{ "LRSD", 0x9a4c, 1, false },
	{ "SVSD", 0x9a55, 8, false },
	{ "LCSD", 0x9a65, 8, false },
	{ "ZSTR", 0x9a75, 4, false },
	{ "BPTP", 0x9a81, 8, false },
	{ "TNGI", 0x9a91, 1, false },
	{ "oeNS", 0x9a9a, 1, false },
	{ "SEBC", 0x9aa3, 8, false },
	{ "ECAP", 0x9ab3, 8, true },
	{ "BVTP", 0x9ac3, 8, true },
	{ "sP1T", 0x9ad3, 8, true },
	{ "BtpG", 0x9ae3, 8, true },
	{ "StpG", 0x9af3, 8, false },
	{ "BlpP", 0x9b03, 8, true },
	{ "SPTP", 0x9b13, 8, false },
	{ "SZSD", 0x9b23, 8, false },
	{ "LZSD", 0x9b33, 8, false },
	{ "S2xG", 0x9b43, 8, true },
	{ "SNUT", 0x9b53, 8, true },
	{ "ZNUT", 0x9b63, 4, false },
	{ "OTTR", 0x9b6f, 8, true },
	{ "LLTR", 0x9b7f, 1, false },
	{ "_COS", 0x9b88, 4, true },
	{ "RCOS", 0x9b94, 4, true },
	{ "dApC", 0x9ba0, 8, true },
	{ "dArW", 0x9bb0, 8, true },
	{ "fVED", 0x9bc0, 1, true },
	{ "GLFp", 0x9bc9, 8, true },
	{ "ABOI", 0x9bd9, 8, false },
	{ "ZSOI", 0x9be9, 4, false },
	{ "ARcM", 0x9bf5, 4, true },
};

static const struct ane_h16_preload_rule ane_h16_preload_25g83 = {
	.cells = ane_h16_patchbay_25g83,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_25g83),
	.zero_start = 0x1bc00,
	.zero_end = 0x1c2a0,
	.digest = {
		0x83, 0x4b, 0xf6, 0xa8, 0xf1, 0xa7, 0x5c, 0x02,
		0xf8, 0x95, 0x2d, 0xd7, 0xd4, 0x1b, 0xce, 0xe6,
		0x6d, 0x46, 0x80, 0x7e, 0xec, 0x7e, 0xa6, 0xa6,
		0x2e, 0x44, 0x8e, 0xfe, 0x18, 0x1d, 0x8b, 0xe8,
	},
};

/* 26.5.2 keeps the 25G83 DATA layout around a different resident image. */
static const struct ane_h16_preload_rule ane_h16_preload_25f84 = {
	.cells = ane_h16_patchbay_25g83,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_25g83),
	.zero_start = 0x1bc00,
	.zero_end = 0x1c2a0,
	.digest = {
		0x17, 0xc3, 0xef, 0x19, 0x6b, 0x89, 0xe4, 0xf5,
		0x94, 0x9a, 0xc7, 0x77, 0x76, 0x0e, 0x5b, 0x05,
		0x80, 0x1f, 0xf5, 0xaa, 0x3b, 0x9f, 0x93, 0x77,
		0x57, 0xfb, 0xd4, 0xee, 0xf5, 0x6b, 0x00, 0x45,
	},
};

/* The 25G83 layout as iBoot leaves it on a J616C with 32 GPU cores. */
static const struct ane_h16_preload_rule ane_h16_preload_25g83_j616c = {
	.cells = ane_h16_patchbay_25g83,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_25g83),
	.zero_start = 0x1bc00,
	.zero_end = 0x1c2a0,
	.digest = {
		0xf6, 0xee, 0x81, 0x1d, 0x12, 0x55, 0xba, 0x09,
		0x99, 0x4e, 0x0f, 0xb4, 0x14, 0x66, 0x82, 0xe4,
		0xc7, 0xd5, 0x55, 0xe6, 0xca, 0x4c, 0xe5, 0x04,
		0x16, 0x6b, 0x39, 0xaf, 0x51, 0xa7, 0xc4, 0xcf,
	},
};

/*
 * The same layout as iBoot leaves it on a J614C with 40 GPU cores: the
 * J616C's patch cells and zeroed range, with this machine's digest.
 */
static const struct ane_h16_preload_rule ane_h16_preload_25g83_j614c = {
	.cells = ane_h16_patchbay_25g83,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_25g83),
	.zero_start = 0x1bc00,
	.zero_end = 0x1c2a0,
	.digest = {
		0x58, 0x2c, 0x92, 0x2a, 0x74, 0xd7, 0x4a, 0xc6,
		0xd0, 0xdd, 0x04, 0xbf, 0xa5, 0x74, 0x63, 0x2e,
		0x8a, 0x51, 0x65, 0x1b, 0xe2, 0x07, 0xa7, 0x18,
		0x14, 0x77, 0xc4, 0x25, 0xa5, 0xef, 0x4b, 0x32,
	},
};

/*
 * The M4 (J713) 26.6.2 (25G83) preload: the T6041 25G83 patchbay records and
 * rewritten range 0x4f0 bytes later in DATA, plus two records whose values
 * change on every boot (GKTS and ECAP).
 */
static const struct ane_h16_preload_cell ane_h16_patchbay_25g83_t8132[] = {
	{ "GKTS", 0x9eb0, 8, true },
	{ "ECAP", 0x9fa3, 8, true },
	{ "ABOI", 0xa0c9, 8, false },
	{ "ZSOI", 0xa0d9, 4, false },
	{ "ARcM", 0xa0e5, 4, true },
};

static const struct ane_h16_preload_rule ane_h16_preload_25g83_j713 = {
	.cells = ane_h16_patchbay_25g83_t8132,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_25g83_t8132),
	.zero_start = 0x1c0f0,
	.zero_end = 0x1c790,
	.digest = {
		0x14, 0x87, 0xde, 0x7a, 0x0e, 0x39, 0xd9, 0xa3,
		0x4f, 0xcb, 0xbf, 0x9b, 0x36, 0xa5, 0xbe, 0xe7,
		0xe6, 0x6e, 0x20, 0x72, 0x2f, 0xf5, 0xc1, 0x56,
		0x55, 0xb0, 0xee, 0x7a, 0x8a, 0x85, 0x86, 0xc6,
	},
};

/* T8132 firmware builds; never offered to the T6041, nor its builds here. */
static const struct ane_h16_profile ane_t8132_profiles[] __maybe_unused = {
	{
		.identity = "H16 T8132 26.6.2 25G83 J713",
		.preload = &ane_h16_preload_25g83_j713,
		.build = &ane_h16_build_t8132_25g83,
	},
};

/* 27.2 (26B123) preload on T6041 (J616c): 26A428 DATA layout and zeroed range. */
static const struct ane_h16_preload_rule ane_h16_preload_26b123 = {
	.cells = ane_h16_patchbay_26a428,
	.cell_count = ARRAY_SIZE(ane_h16_patchbay_26a428),
	.zero_start = 0x17c80,
	.zero_end = 0x18320,
	.digest = {
		0xd1, 0xbd, 0xb9, 0xd0, 0x4d, 0x6e, 0x35, 0xa7,
		0x31, 0x71, 0xc2, 0xa8, 0xa9, 0x66, 0x38, 0xfe,
		0xc2, 0x6c, 0x4c, 0xed, 0x7a, 0x0e, 0xfd, 0xc4,
		0xb9, 0xde, 0x58, 0x5f, 0x70, 0x92, 0x33, 0x16,
	},
};

/* Every supported H16 firmware build, matched against the live preload. */
static const struct ane_h16_profile ane_h16_profiles[] __maybe_unused = {
	{
		.identity = "H16 27.0 26A428",
		.preload = &ane_h16_preload_26a428,
		.build = &ane_h16_build_26a428,
	},
	{
		.identity = "H16 27.2 26B123",
		.preload = &ane_h16_preload_26b123,
		.build = &ane_h16_build_26a428,
	},
	{
		.identity = "H16 26.6.2 25G83",
		.preload = &ane_h16_preload_25g83,
		.build = &ane_h16_build_26a428,
	},
	{
		.identity = "H16 26.6.2 25G83 J616C",
		.preload = &ane_h16_preload_25g83_j616c,
		.build = &ane_h16_build_26a428,
	},
	{
		.identity = "H16 26.6.2 25G83 J614C",
		.preload = &ane_h16_preload_25g83_j614c,
		.build = &ane_h16_build_26a428,
	},
	{
		.identity = "H16 26.5.2 25F84",
		.preload = &ane_h16_preload_25f84,
		.build = &ane_h16_build_26a428,
	},
};

#endif /* __ANE_H16_BOOT_DATA_H__ */
