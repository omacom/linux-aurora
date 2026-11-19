/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Layout of the T8152 ATC PHY: block offsets and tunable list order. No tunable values: they come from the device tree. */
#ifndef _ATC_T8152_TABLES_H
#define _ATC_T8152_TABLES_H

struct t8152_tunable_blk {
	const char *prop;	/* device tree property holding {offset, mask, value} triples relative to the block */
	u32 off;		/* block offset inside the core window */
	u32 len;		/* block length, bound for the offsets */
};

#define T8152_NUM_TUNABLES 63

static const struct t8152_tunable_blk t8152_tunables[T8152_NUM_TUNABLES] = {
	[0] = { "apple,tunable-aciophy-cmn-dflt", 0x800, 0x6c },	/* tunable_ACIOPHY_CMN_DFLT */
	[1] = { "apple,tunable-ciopll-core-dflt", 0x1600, 0x280 },	/* tunable_CIOPLL_CORE_DFLT */
	[2] = { "apple,tunable-atc-fabric", 0x6c000, 0x198 },	/* tunable_ATC_FABRIC */
	[3] = { "apple,tunable-dppll-core-dflt", 0x1000, 0x280 },	/* tunable_DPPLL_CORE_DFLT */
	[4] = { "apple,tunable-dppll-top-dflt", 0xe00, 0x200 },	/* tunable_DPPLL_TOP_DFLT */
	[5] = { "apple,tunable-ciopll-top-dflt", 0x1400, 0x200 },	/* tunable_CIOPLL_TOP_DFLT */
	[6] = { "apple,tunable-aciophy-usbc0-dflt", 0x15c00, 0x98 },	/* tunable_ACIOPHY_USBC0_DFLT */
	[7] = { "apple,tunable-aciophy-usbc0-usb", 0x15c00, 0x98 },	/* tunable_ACIOPHY_USBC0_USB */
	[8] = { "apple,tunable-ln0-arbiter-regs-dflt", 0x8280, 0xfc },	/* tunable_LN0_ARBITER_REGS_DFLT */
	[9] = { "apple,tunable-ln1-arbiter-regs-dflt", 0x11280, 0xfc },	/* tunable_LN1_ARBITER_REGS_DFLT */
	[10] = { "apple,tunable-ln0-rx-top-regs-dflt", 0x8cf0, 0x18 },	/* tunable_LN0_RX_TOP_REGS_DFLT */
	[11] = { "apple,tunable-ln1-rx-top-regs-dflt", 0x11cf0, 0x18 },	/* tunable_LN1_RX_TOP_REGS_DFLT */
	[12] = { "apple,tunable-ln0-rx-dfe-regs-dflt", 0x77c0, 0xe8 },	/* tunable_LN0_RX_DFE_REGS_DFLT */
	[13] = { "apple,tunable-ln1-rx-dfe-regs-dflt", 0x107c0, 0xe8 },	/* tunable_LN1_RX_DFE_REGS_DFLT */
	[14] = { "apple,tunable-ln0-smgr-regs-dflt", 0x7d80, 0x80 },	/* tunable_LN0_SMGR_REGS_DFLT */
	[15] = { "apple,tunable-ln1-smgr-regs-dflt", 0x10d80, 0x80 },	/* tunable_LN1_SMGR_REGS_DFLT */
	[16] = { "apple,tunable-ln0-rxaeq-top-regs-dflt", 0x79c0, 0x23c },	/* tunable_LN0_RXAEQ_TOP_REGS_DFLT */
	[17] = { "apple,tunable-ln1-rxaeq-top-regs-dflt", 0x109c0, 0x23c },	/* tunable_LN1_RXAEQ_TOP_REGS_DFLT */
	[18] = { "apple,tunable-ln0-rx-shm-regs-dflt", 0x8700, 0x194 },	/* tunable_LN0_RX_SHM_REGS_DFLT */
	[19] = { "apple,tunable-ln1-rx-shm-regs-dflt", 0x11700, 0x194 },	/* tunable_LN1_RX_SHM_REGS_DFLT */
	[20] = { "apple,tunable-ln0-txseq-regs-dflt", 0x4c80, 0x84 },	/* tunable_LN0_TXSEQ_REGS_DFLT */
	[21] = { "apple,tunable-ln1-txseq-regs-dflt", 0xdc80, 0x84 },	/* tunable_LN1_TXSEQ_REGS_DFLT */
	[22] = { "apple,tunable-ln0-tx-shm-ahb-dflt", 0x4f90, 0x94 },	/* tunable_LN0_TX_SHM_AHB_DFLT */
	[23] = { "apple,tunable-ln1-tx-shm-ahb-dflt", 0xdf90, 0x94 },	/* tunable_LN1_TX_SHM_AHB_DFLT */
	[24] = { "apple,tunable-ln0-tx-txspl-cm-dflt", 0x4e20, 0x14 },	/* tunable_LN0_TX_TXSPL_CM_DFLT */
	[25] = { "apple,tunable-ln1-tx-txspl-cm-dflt", 0xde20, 0x14 },	/* tunable_LN1_TX_TXSPL_CM_DFLT */
	[26] = { "apple,tunable-ln0-asdc-txspl-cm-dflt", 0x8980, 0x14 },	/* tunable_LN0_ASDC_TXSPL_CM_DFLT */
	[27] = { "apple,tunable-ln1-asdc-txspl-cm-dflt", 0x11980, 0x14 },	/* tunable_LN1_ASDC_TXSPL_CM_DFLT */
	[28] = { "apple,tunable-ln0-div40-regs-dflt", 0x8580, 0x80 },	/* tunable_LN0_DIV40_REGS_DFLT */
	[29] = { "apple,tunable-ln1-div40-regs-dflt", 0x11580, 0x80 },	/* tunable_LN1_DIV40_REGS_DFLT */
	[30] = { "apple,tunable-ln0-rx-pctrl-regs-dflt", 0x8600, 0xec },	/* tunable_LN0_RX_PCTRL_REGS_DFLT */
	[31] = { "apple,tunable-ln1-rx-pctrl-regs-dflt", 0x11600, 0xec },	/* tunable_LN1_RX_PCTRL_REGS_DFLT */
	[32] = { "apple,tunable-ln0-ctle-eq-regs-dflt", 0x7f80, 0x100 },	/* tunable_LN0_CTLE_EQ_REGS_DFLT */
	[33] = { "apple,tunable-ln1-ctle-eq-regs-dflt", 0x10f80, 0x100 },	/* tunable_LN1_CTLE_EQ_REGS_DFLT */
	[34] = { "apple,tunable-l0-rxcalacc-halfui-dflt", 0x7300, 0x80 },	/* tunable_L0_RXCALACC_HALFUI_DFLT */
	[35] = { "apple,tunable-l1-rxcalacc-halfui-dflt", 0x10300, 0x80 },	/* tunable_L1_RXCALACC_HALFUI_DFLT */
	[36] = { "apple,tunable-l0-rxcalacc-edskew-dflt", 0x7280, 0x80 },	/* tunable_L0_RXCALACC_EDSKEW_DFLT */
	[37] = { "apple,tunable-l1-rxcalacc-edskew-dflt", 0x10280, 0x80 },	/* tunable_L1_RXCALACC_EDSKEW_DFLT */
	[38] = { "apple,tunable-ln0-txspl-txdcc-dflt", 0x4d80, 0x40 },	/* tunable_LN0_TXSPL_TXDCC_DFLT */
	[39] = { "apple,tunable-ln1-txspl-txdcc-dflt", 0xdd80, 0x40 },	/* tunable_LN1_TXSPL_TXDCC_DFLT */
	[40] = { "apple,tunable-ln0-smgr-regs-usb", 0x7d80, 0x80 },	/* tunable_LN0_SMGR_REGS_USB */
	[41] = { "apple,tunable-ln1-smgr-regs-usb", 0x10d80, 0x80 },	/* tunable_LN1_SMGR_REGS_USB */
	[42] = { "apple,tunable-ln0-detector-regs-usb", 0x8480, 0x30 },	/* tunable_LN0_DETECTOR_REGS_USB */
	[43] = { "apple,tunable-ln1-detector-regs-usb", 0x11480, 0x30 },	/* tunable_LN1_DETECTOR_REGS_USB */
	[44] = { "apple,tunable-ln0-div40-regs-usb", 0x8580, 0x80 },	/* tunable_LN0_DIV40_REGS_USB */
	[45] = { "apple,tunable-ln1-div40-regs-usb", 0x11580, 0x80 },	/* tunable_LN1_DIV40_REGS_USB */
	[46] = { "apple,tunable-ln0-ctle-eq-regs-usb", 0x7f80, 0x100 },	/* tunable_LN0_CTLE_EQ_REGS_USB */
	[47] = { "apple,tunable-ln1-ctle-eq-regs-usb", 0x10f80, 0x100 },	/* tunable_LN1_CTLE_EQ_REGS_USB */
	[48] = { "apple,tunable-ln0-rxaeq-top-regs-usb", 0x79c0, 0x23c },	/* tunable_LN0_RXAEQ_TOP_REGS_USB */
	[49] = { "apple,tunable-ln1-rxaeq-top-regs-usb", 0x109c0, 0x23c },	/* tunable_LN1_RXAEQ_TOP_REGS_USB */
	[50] = { "apple,tunable-ln0-txseq-regs-usb", 0x4c80, 0x84 },	/* tunable_LN0_TXSEQ_REGS_USB */
	[51] = { "apple,tunable-ln1-txseq-regs-usb", 0xdc80, 0x84 },	/* tunable_LN1_TXSEQ_REGS_USB */
	[52] = { "apple,tunable-cio-shim-cio", 0x1a00, 0x68 },	/* tunable_CIO_SHIM_CIO */
	[53] = { "apple,tunable-ln0-ctle-eq-regs-cio", 0x7f80, 0x100 },	/* tunable_LN0_CTLE_EQ_REGS_CIO */
	[54] = { "apple,tunable-ln0-smgr-regs-cio", 0x7d80, 0x80 },	/* tunable_LN0_SMGR_REGS_CIO */
	[55] = { "apple,tunable-ln0-rxaeq-top-regs-cio", 0x79c0, 0x23c },	/* tunable_LN0_RXAEQ_TOP_REGS_CIO */
	[56] = { "apple,tunable-ln0-rxtx-txd-cfg-cio", 0x8af0, 0x44 },	/* tunable_LN0_RXTX_TXD_CFG_CIO */
	[57] = { "apple,tunable-ln0-tx-txd-cfg-cio", 0x4e90, 0x44 },	/* tunable_LN0_TX_TXD_CFG_CIO */
	[58] = { "apple,tunable-ln1-ctle-eq-regs-cio", 0x10f80, 0x100 },	/* tunable_LN1_CTLE_EQ_REGS_CIO */
	[59] = { "apple,tunable-ln1-smgr-regs-cio", 0x10d80, 0x80 },	/* tunable_LN1_SMGR_REGS_CIO */
	[60] = { "apple,tunable-ln1-rxaeq-top-regs-cio", 0x109c0, 0x23c },	/* tunable_LN1_RXAEQ_TOP_REGS_CIO */
	[61] = { "apple,tunable-ln1-rxtx-txd-cfg-cio", 0x11af0, 0x44 },	/* tunable_LN1_RXTX_TXD_CFG_CIO */
	[62] = { "apple,tunable-ln1-tx-txd-cfg-cio", 0xde90, 0x44 },	/* tunable_LN1_TX_TXD_CFG_CIO */
};

static const u8 t8152_seq_efuse[] = {
	0, 1,
};

static const u8 t8152_seq_usb3[] = {
	2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17,
	18, 19, 20, 21, 22, 23, 24, 25,
	26, 27, 28, 29, 30, 31, 32, 33,
	34, 35, 36, 37, 38, 39, 40, 41,
	42, 43, 44, 45, 46, 47, 48, 49,
	50, 51,
};

static const u8 t8152_seq_cio[] = {
	2, 3, 4, 5, 52, 8, 10, 12,
	14, 16, 18, 20, 22, 24, 26, 28,
	30, 32, 34, 36, 38, 53, 54, 55,
	56, 57, 9, 11, 13, 15, 17, 19,
	21, 23, 25, 27, 29, 31, 33, 35,
	37, 39, 58, 59, 60, 61, 62,
};

#endif
