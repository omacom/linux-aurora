// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Type-C PHY driver
 *
 * The Apple Type-C PHY (ATCPHY) is a combined PHY for USB 2.0, USB 3.x,
 * USB4/Thunderbolt, and DisplayPort connectivity via Type-C ports found in
 * Apple Silicon SoCs.
 *
 * The PHY handles muxing between these different protocols and also provides the
 * reset controller for the attached DWC3 USB controller.
 *
 * In order to correctly setup the high speed lanes for the various modes
 * calibration values copied from Apple's firmware by our bootloader m1n1 are
 * required. Without these only USB2 operation is possible.
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@kernel.org>
 */

#include <dt-bindings/phy/phy.h>
#include <kunit/static_stub.h>
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/iopoll.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/pm_domain.h>
#include <linux/reset-controller.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/tunable.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/pd.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>

#include "atc-dp-tunables.h"

#include "atc-t8122-dp.h"
#include "atc-tunnel.h"

#define AUSPLL_FSM_CTRL 0x1014

#define AUSPLL_APB_CMD_OVERRIDE 0x2000
#define AUSPLL_APB_CMD_OVERRIDE_REQ BIT(0)
#define AUSPLL_APB_CMD_OVERRIDE_ACK BIT(1)
#define AUSPLL_APB_CMD_OVERRIDE_UNK28 BIT(28)
#define AUSPLL_APB_CMD_OVERRIDE_CMD GENMASK(27, 3)

#define AUSPLL_FREQ_DESC_A 0x2080
#define AUSPLL_FD_FREQ_COUNT_TARGET GENMASK(9, 0)
#define AUSPLL_FD_FBDIVN_HALF BIT(10)
#define AUSPLL_FD_REV_DIVN GENMASK(13, 11)
#define AUSPLL_FD_KI_MAN GENMASK(17, 14)
#define AUSPLL_FD_KI_EXP GENMASK(21, 18)
#define AUSPLL_FD_KP_MAN GENMASK(25, 22)
#define AUSPLL_FD_KP_EXP GENMASK(29, 26)
#define AUSPLL_FD_KPKI_SCALE_HBW GENMASK(31, 30)

#define AUSPLL_FREQ_DESC_B 0x2084
#define AUSPLL_FD_FBDIVN_FRAC_DEN GENMASK(13, 0)
#define AUSPLL_FD_FBDIVN_FRAC_NUM GENMASK(27, 14)

#define AUSPLL_FREQ_DESC_C 0x2088
#define AUSPLL_FD_SDM_SSC_STEP GENMASK(7, 0)
#define AUSPLL_FD_SDM_SSC_EN BIT(8)
#define AUSPLL_FD_PCLK_DIV_SEL GENMASK(13, 9)
#define AUSPLL_FD_LFSDM_DIV GENMASK(15, 14)
#define AUSPLL_FD_LFCLK_CTRL GENMASK(19, 16)
#define AUSPLL_FD_VCLK_OP_DIVN GENMASK(21, 20)
#define AUSPLL_FD_VCLK_PRE_DIVN BIT(22)

#define AUSPLL_DCO_EFUSE_SPARE 0x222c
#define AUSPLL_RODCO_ENCAP_EFUSE GENMASK(10, 9)
#define AUSPLL_RODCO_BIAS_ADJUST_EFUSE GENMASK(14, 12)

#define AUSPLL_FRACN_CAN 0x22a4
#define AUSPLL_DLL_START_CAPCODE GENMASK(18, 17)

#define AUSPLL_CLKOUT_MASTER 0x2200
#define AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN BIT(2)
#define AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN BIT(4)
#define AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN BIT(6)
#define AUSPLL_CLKOUT_MASTER_DRVR_EN                                         \
	(AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN | AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN | \
	 AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN)

#define AUSPLL_CLKOUT_DIV 0x2208
#define AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI GENMASK(20, 16)

#define AUSPLL_BGR 0x2214
#define AUSPLL_BGR_CTRL_AVAIL BIT(0)

#define AUSPLL_CLKOUT_DTC_VREG 0x2220
#define AUSPLL_DTC_VREG_ADJUST GENMASK(16, 14)
#define AUSPLL_DTC_VREG_BYPASS BIT(7)

#define AUSPLL_FREQ_CFG 0x2224
#define AUSPLL_FREQ_REFCLK GENMASK(1, 0)

/* T8122 AUSPLL fields differ from T8103, including the APB handshake. */
#define T8122_AUSPLL_FREQ_CFG 0x2234
#define T8122_AUSPLL_BGR 0x2218
#define T8122_AUSPLL_PCLK_DRIVER BIT(3)
#define T8122_AUSPLL_REFBUF_DRIVER BIT(11)
#define T8122_DP_PCLK_STATUS 0x7034
#define T8122_APB_PRESERVE 0xe0000006U
#define T8122_PCLK_ENABLES GENMASK(15, 13)

#define AUS_COMMON_SHIM_BLK_BIAS_REG 0x0a00
#define AUS_COMMON_SHIM_BLK_BIAS_REG_BGBIAS_OV BIT(1)

#define AUS_COMMON_SHIM_BLK_VREG 0x0a04
#define AUS_VREG_TRIM GENMASK(6, 2)

#define AUS_COMMON_DIG_RCAL1 0x804
#define AUS_COMMON_DIG_RCAL1_ALL_CODES_DONE BIT(0)

#define AUS_UNK_A20 0x0a20
#define AUS_UNK_A20_TX_CAL_CODE GENMASK(23, 20)

#define ACIOPHY_CMN_SHM_STS_REG0 0x0a74
#define ACIOPHY_CMN_SHM_STS_REG0_CMD_READY BIT(0)

#define CIO3PLL_CLK_CTRL 0x2a00
#define CIO3PLL_CLK_PCLK_EN BIT(1)
#define CIO3PLL_CLK_REFCLK_EN BIT(5)

#define CIO3PLL_DCO_NCTRL 0x2a38
#define CIO3PLL_DCO_COARSEBIN_EFUSE0 GENMASK(6, 0)
#define CIO3PLL_DCO_COARSEBIN_EFUSE1 GENMASK(23, 17)

#define CIO3PLL_FRACN_CAN 0x2aa4
#define CIO3PLL_DLL_CAL_START_CAPCODE GENMASK(18, 17)

#define CIO3PLL_DTC_VREG 0x2a20
#define CIO3PLL_DTC_VREG_ADJUST GENMASK(16, 14)

#define ATCPHY_EVT_USB2_CTL 0x0
#define ATCPHY_EVT_USB2_CTL_EVT_EN 0x1
#define ATCPHY_EVT_USB2_CTL_LOAD_CNT 0x8

#define ACIOPHY_CFG0 0x08
#define ACIOPHY_CFG0_COMMON_BIG BIT(0)
#define ACIOPHY_CFG0_COMMON_BIG_OV BIT(1)
#define ACIOPHY_CFG0_COMMON_SMALL BIT(2)
#define ACIOPHY_CFG0_COMMON_SMALL_OV BIT(3)
#define ACIOPHY_CFG0_COMMON_CLAMP BIT(4)
#define ACIOPHY_CFG0_COMMON_CLAMP_OV BIT(5)
#define ACIOPHY_CFG0_RX_SMALL GENMASK(7, 6)
#define ACIOPHY_CFG0_RX_SMALL_OV GENMASK(9, 8)
#define ACIOPHY_CFG0_RX_BIG GENMASK(11, 10)
#define ACIOPHY_CFG0_RX_BIG_OV GENMASK(13, 12)
#define ACIOPHY_CFG0_RX_CLAMP GENMASK(15, 14)
#define ACIOPHY_CFG0_RX_CLAMP_OV GENMASK(17, 16)

#define ACIOPHY_CROSSBAR_T8103 0x4c
#define ACIOPHY_CROSSBAR_T8122 0x64
#define ACIOPHY_CROSSBAR_PROTOCOL GENMASK(4, 0)
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4 0x0
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED 0x1
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3 0xa
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED 0xb
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP 0x10
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED 0x11
#define ACIOPHY_CROSSBAR_PROTOCOL_DP 0x14
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA GENMASK(16, 5)
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE 0x0000
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100 0x100
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008 0x008
#define ACIOPHY_CROSSBAR_DP_BOTH_PMA BIT(17)

#define ACIOPHY_LANE_MODE_T8103 0x48
#define ACIOPHY_LANE_MODE_T8122 0x60
#define ACIOPHY_LANE_MODE_RX0 GENMASK(2, 0)
#define ACIOPHY_LANE_MODE_TX0 GENMASK(5, 3)
#define ACIOPHY_LANE_MODE_RX1 GENMASK(8, 6)
#define ACIOPHY_LANE_MODE_TX1 GENMASK(11, 9)

enum atcphy_lane_mode {
	ACIOPHY_LANE_MODE_USB4 = 0,
	ACIOPHY_LANE_MODE_USB3 = 1,
	ACIOPHY_LANE_MODE_DP = 2,
	ACIOPHY_LANE_MODE_OFF = 3,
};

#define ACIOPHY_TOP_BIST_CIOPHY_CFG1 0x84
#define ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN BIT(27)
#define ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN BIT(28)

#define ACIOPHY_TOP_BIST_OV_CFG 0x8c
#define ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV BIT(13)
#define ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV BIT(25)

#define ACIOPHY_TOP_BIST_READ_CTRL 0x90
#define ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE BIT(2)

#define ACIOPHY_TOP_PHY_STAT 0x9c
#define ACIOPHY_TOP_PHY_STAT_LN0_UNK0 BIT(0)
#define ACIOPHY_TOP_PHY_STAT_LN0_UNK23 BIT(23)

#define ACIOPHY_TOP_BIST_PHY_CFG0 0xa8
#define ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N BIT(0)

#define ACIOPHY_TOP_BIST_PHY_CFG1 0xac
#define ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN GENMASK(13, 10)

#define ACIOPHY_BIST_CFG1 0x17000
#define ACIOPHY_BIST_CFG1_CLOCK_EN BIT(0)
#define ACIOPHY_BIST_CFG1_USB_EN BIT(2)

#define ACIOPHY_BIST_CFG2 0x17004
#define ACIOPHY_BIST_CFG2_LN0_PHY_STATUS_RE BIT(2)

#define ACIOPHY_BIST_MAC_USBPHY_STATUS2 0x17080
#define ACIOPHY_BIST_MAC_USBPHY_STATUS2_LN0_PHY_STATUS2 BIT(7)

#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV1 0x17088
#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PIPE_RESET_N_OV 0x100000
#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PWR_DOWN_OV GENMASK(26, 25)

#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV2 0x1708c
#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_RESET_N_OV 0x200
#define ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_PWR_DOWN_OV 0x20

#define ACIOPHY_SLEEP_CTRL 0x1b0
#define ACIOPHY_SLEEP_CTRL_TX_BIG GENMASK(1, 0)
#define ACIOPHY_SLEEP_CTRL_TX_BIG_OV GENMASK(3, 2)
#define ACIOPHY_SLEEP_CTRL_TX_SMALL GENMASK(5, 4)
#define ACIOPHY_SLEEP_CTRL_TX_SMALL_OV GENMASK(7, 6)
#define ACIOPHY_SLEEP_CTRL_TX_CLAMP GENMASK(9, 8)
#define ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV GENMASK(11, 10)

#define ACIOPHY_PLL_PCTL_FSM_CTRL1 0x1014
#define ACIOPHY_PLL_APB_REQ_OV_SEL GENMASK(21, 13)
#define ACIOPHY_PLL_COMMON_CTRL 0x1028
#define ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT BIT(24)

#define ATCPHY_POWER_CTRL 0x20000
#define ATCPHY_POWER_STAT 0x20004
#define ATCPHY_POWER_SLEEP_SMALL BIT(0)
#define ATCPHY_POWER_SLEEP_BIG BIT(1)
#define ATCPHY_POWER_CLAMP_EN BIT(2)
#define ATCPHY_POWER_APB_RESET_N BIT(3)
#define ATCPHY_POWER_PHY_RESET_N BIT(4)

#define ATCPHY_MISC 0x20008
#define ATCPHY_MISC_RESET_N BIT(0)
#define ATCPHY_MISC_LANE_SWAP BIT(2)

#define ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0 0x7000
#define DP_PMA_BYTECLK_RESET BIT(0)
#define DP_MAC_DIV20_CLK_SEL BIT(1)
#define DPTXPHY_PMA_LANE_RESET_N BIT(2)
#define DPTXPHY_PMA_LANE_RESET_N_OV BIT(3)
#define DPTX_PCLK1_SELECT GENMASK(6, 4)
#define DPTX_PCLK2_SELECT GENMASK(9, 7)
#define DPRX_PCLK_SELECT GENMASK(12, 10)
#define DPTX_PCLK1_ENABLE BIT(13)
#define DPTX_PCLK2_ENABLE BIT(14)
#define DPRX_PCLK_ENABLE BIT(15)

#define ACIOPHY_DP_PCLK_STAT 0x7044
#define ACIOPHY_AUSPLL_LOCK BIT(3)

#define LN0_AUSPMA_RX_TOP 0x9000
#define LN0_AUSPMA_RX_EQ 0xA000
#define LN0_AUSPMA_RX_SHM 0xB000
#define LN0_AUSPMA_TX_TOP 0xC000
#define LN0_AUSPMA_TX_SHM 0xD000

#define LN1_AUSPMA_RX_TOP 0x10000
#define LN1_AUSPMA_RX_EQ 0x11000
#define LN1_AUSPMA_RX_SHM 0x12000
#define LN1_AUSPMA_TX_TOP 0x13000
#define LN1_AUSPMA_TX_SHM 0x14000

#define LN_AUSPMA_RX_TOP_PMAFSM 0x0010
#define LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV BIT(0)
#define LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ BIT(9)

#define LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE 0x00F0
#define LN_RX_TXMODE BIT(0)

#define LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0 0x00
#define LN_TX_CLK_EN BIT(20)
#define LN_TX_CLK_EN_OV BIT(21)

#define LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1 0x04
#define LN_RX_DIV20_RESET_N_OV BIT(29)
#define LN_RX_DIV20_RESET_N BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL2 0x08
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL3 0x0C
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL4 0x10
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL5 0x14
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL6 0x18
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL7 0x1C
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL8 0x20
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL9 0x24
#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10 0x28
#define LN_DTVREG_ADJUST GENMASK(31, 27)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11 0x2C
#define LN_DTVREG_BIG_EN BIT(23)
#define LN_DTVREG_BIG_EN_OV BIT(24)
#define LN_DTVREG_SML_EN BIT(25)
#define LN_DTVREG_SML_EN_OV BIT(26)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12 0x30
#define LN_TX_BYTECLK_RESET_SYNC_CLR BIT(22)
#define LN_TX_BYTECLK_RESET_SYNC_CLR_OV BIT(23)
#define LN_TX_BYTECLK_RESET_SYNC_EN BIT(24)
#define LN_TX_BYTECLK_RESET_SYNC_EN_OV BIT(25)
#define LN_TX_HRCLK_SEL BIT(28)
#define LN_TX_HRCLK_SEL_OV BIT(29)
#define LN_TX_PBIAS_EN BIT(30)
#define LN_TX_PBIAS_EN_OV BIT(31)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13 0x34
#define LN_TX_PRE_EN BIT(0)
#define LN_TX_PRE_EN_OV BIT(1)
#define LN_TX_PST1_EN BIT(2)
#define LN_TX_PST1_EN_OV BIT(3)
#define LN_DTVREG_ADJUST_OV BIT(15)

#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL14A 0x38
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL14B 0x3C
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL15A 0x40
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL15B 0x44
#define LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16 0x48
#define LN_RXTERM_EN BIT(21)
#define LN_RXTERM_EN_OV BIT(22)
#define LN_RXTERM_PULLUP_LEAK_EN BIT(23)
#define LN_RXTERM_PULLUP_LEAK_EN_OV BIT(24)
#define LN_TX_CAL_CODE GENMASK(29, 25)
#define LN_TX_CAL_CODE_OV BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17 0x4C
#define LN_TX_MARGIN GENMASK(19, 15)
#define LN_TX_MARGIN_OV BIT(20)
#define LN_TX_MARGIN_LSB BIT(21)
#define LN_TX_MARGIN_LSB_OV BIT(22)
#define LN_TX_MARGIN_P1 GENMASK(26, 23)
#define LN_TX_MARGIN_P1_OV BIT(27)
#define LN_TX_MARGIN_P1_LSB GENMASK(29, 28)
#define LN_TX_MARGIN_P1_LSB_OV BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18 0x50
#define LN_TX_P1_CODE GENMASK(3, 0)
#define LN_TX_P1_CODE_OV BIT(4)
#define LN_TX_P1_LSB_CODE GENMASK(6, 5)
#define LN_TX_P1_LSB_CODE_OV BIT(7)
#define LN_TX_MARGIN_PRE GENMASK(10, 8)
#define LN_TX_MARGIN_PRE_OV BIT(11)
#define LN_TX_MARGIN_PRE_LSB GENMASK(13, 12)
#define LN_TX_MARGIN_PRE_LSB_OV BIT(14)
#define LN_TX_PRE_LSB_CODE GENMASK(16, 15)
#define LN_TX_PRE_LSB_CODE_OV BIT(17)
#define LN_TX_PRE_CODE GENMASK(21, 18)
#define LN_TX_PRE_CODE_OV BIT(22)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19 0x54
#define LN_TX_TEST_EN BIT(21)
#define LN_TX_TEST_EN_OV BIT(22)
#define LN_TX_EN BIT(23)
#define LN_TX_EN_OV BIT(24)
#define LN_TX_CLK_DLY_CTRL_TAPGEN GENMASK(27, 25)
#define LN_TX_CLK_DIV2_EN BIT(28)
#define LN_TX_CLK_DIV2_EN_OV BIT(29)
#define LN_TX_CLK_DIV2_RST BIT(30)
#define LN_TX_CLK_DIV2_RST_OV BIT(31)

#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL20 0x58
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL21 0x5C
#define LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22 0x60
#define LN_VREF_ADJUST_GRAY GENMASK(11, 7)
#define LN_VREF_ADJUST_GRAY_OV BIT(12)
#define LN_VREF_BIAS_SEL GENMASK(14, 13)
#define LN_VREF_BIAS_SEL_OV BIT(15)
#define LN_VREF_BOOST_EN BIT(16)
#define LN_VREF_BOOST_EN_OV BIT(17)
#define LN_VREF_EN BIT(18)
#define LN_VREF_EN_OV BIT(19)
#define LN_VREF_LPBKIN_DATA GENMASK(29, 28)
#define LN_VREF_TEST_RXLPBKDT_EN BIT(30)
#define LN_VREF_TEST_RXLPBKDT_EN_OV BIT(31)

#define LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0 0x00
#define LN_BYTECLK_RESET_SYNC_EN_OV BIT(2)
#define LN_BYTECLK_RESET_SYNC_EN BIT(3)
#define LN_BYTECLK_RESET_SYNC_CLR_OV BIT(4)
#define LN_BYTECLK_RESET_SYNC_CLR BIT(5)
#define LN_BYTECLK_RESET_SYNC_SEL_OV BIT(6)

#define LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1 0x04
#define LN_TXA_DIV2_EN_OV BIT(8)
#define LN_TXA_DIV2_EN BIT(9)
#define LN_TXA_DIV2_RESET_OV BIT(10)
#define LN_TXA_DIV2_RESET BIT(11)
#define LN_TXA_CLK_EN_OV BIT(22)
#define LN_TXA_CLK_EN BIT(23)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG0 0x08
#define LN_TXA_CAL_CTRL_OV BIT(0)
#define LN_TXA_CAL_CTRL GENMASK(18, 1)
#define LN_TXA_CAL_CTRL_BASE_OV BIT(19)
#define LN_TXA_CAL_CTRL_BASE GENMASK(23, 20)
#define LN_TXA_HIZ_OV BIT(29)
#define LN_TXA_HIZ BIT(30)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG1 0x0C
#define LN_AUSPMA_TX_SHM_TXA_IMP_REG2 0x10
#define LN_TXA_MARGIN_OV BIT(0)
#define LN_TXA_MARGIN GENMASK(18, 1)
#define LN_TXA_MARGIN_2R_OV BIT(19)
#define LN_TXA_MARGIN_2R BIT(20)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG3 0x14
#define LN_TXA_MARGIN_POST_OV BIT(0)
#define LN_TXA_MARGIN_POST GENMASK(10, 1)
#define LN_TXA_MARGIN_POST_2R_OV BIT(11)
#define LN_TXA_MARGIN_POST_2R BIT(12)
#define LN_TXA_MARGIN_POST_4R_OV BIT(13)
#define LN_TXA_MARGIN_POST_4R BIT(14)
#define LN_TXA_MARGIN_PRE_OV BIT(15)
#define LN_TXA_MARGIN_PRE GENMASK(21, 16)
#define LN_TXA_MARGIN_PRE_2R_OV BIT(22)
#define LN_TXA_MARGIN_PRE_2R BIT(23)
#define LN_TXA_MARGIN_PRE_4R_OV BIT(24)
#define LN_TXA_MARGIN_PRE_4R BIT(25)

#define LN_AUSPMA_TX_SHM_TXA_UNK_REG0 0x18
#define LN_AUSPMA_TX_SHM_TXA_UNK_REG1 0x1C
#define LN_AUSPMA_TX_SHM_TXA_UNK_REG2 0x20

#define LN_AUSPMA_TX_SHM_TXA_LDOCLK 0x24
#define LN_LDOCLK_BYPASS_SML_OV BIT(8)
#define LN_LDOCLK_BYPASS_SML BIT(9)
#define LN_LDOCLK_BYPASS_BIG_OV BIT(10)
#define LN_LDOCLK_BYPASS_BIG BIT(11)
#define LN_LDOCLK_EN_SML_OV BIT(12)
#define LN_LDOCLK_EN_SML BIT(13)
#define LN_LDOCLK_EN_BIG_OV BIT(14)
#define LN_LDOCLK_EN_BIG BIT(15)

/* LPDPTX registers */
#define LPDPTX_AUX_CFG_BLK_AUX_CTRL 0x0000
#define LPDPTX_BLK_AUX_CTRL_PWRDN BIT(4)
#define LPDPTX_BLK_AUX_RXOFFSET GENMASK(25, 22)

#define LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL 0x0008

#define LPDPTX_AUX_CFG_BLK_AUX_MARGIN 0x000c
#define LPDPTX_MARGIN_RCAL_RXOFFSET_EN BIT(5)
#define LPDPTX_AUX_MARGIN_RCAL_TXSWING GENMASK(10, 6)

#define LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0 0x0204
#define LPDPTX_CFG_PMA_AUX_SEL_LF_DATA BIT(15)

#define LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1 0x0208
#define LPDPTX_CFG_PMA_PHYS_ADJ GENMASK(22, 20)
#define LPDPTX_CFG_PMA_PHYS_ADJ_OV BIT(19)

#define LPDPTX_AUX_CONTROL 0x4000
#define LPDPTX_AUX_PWN_DOWN 0x10
#define LPDPTX_AUX_CLAMP_EN 0x04
#define LPDPTX_SLEEP_B_BIG_IN 0x02
#define LPDPTX_SLEEP_B_SML_IN 0x01
#define LPDPTX_TXTERM_CODEMSB 0x400
#define LPDPTX_TXTERM_CODE GENMASK(9, 5)

/* pipehandler registers */
#define PIPEHANDLER_OVERRIDE 0x00
#define PIPEHANDLER_OVERRIDE_RXVALID BIT(0)
#define PIPEHANDLER_OVERRIDE_RXDETECT BIT(2)

#define PIPEHANDLER_OVERRIDE_VALUES 0x04
#define PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 BIT(1)
#define PIPEHANDLER_OVERRIDE_VAL_RXDETECT1 BIT(2)
#define PIPEHANDLER_OVERRIDE_VAL_PHY_STATUS BIT(4)

#define PIPEHANDLER_MUX_CTRL 0x0c
#define PIPEHANDLER_MUX_CTRL_CLK_T8103 GENMASK(5, 3)
#define PIPEHANDLER_MUX_CTRL_DATA_T8103 GENMASK(2, 0)
#define PIPEHANDLER_MUX_CTRL_CLK_T8142 GENMASK(6, 4)
#define PIPEHANDLER_MUX_CTRL_DATA_T8142 GENMASK(1, 0)
#define PIPEHANDLER_MUX_CTRL_CLK_OFF 0
#define PIPEHANDLER_MUX_CTRL_CLK_USB3 1
#define PIPEHANDLER_MUX_CTRL_CLK_USB4 2
#define PIPEHANDLER_MUX_CTRL_CLK_DUMMY 4

#define PIPEHANDLER_MUX_CTRL_DATA_USB3 0
#define PIPEHANDLER_MUX_CTRL_DATA_USB4 1
#define PIPEHANDLER_MUX_CTRL_DATA_DUMMY 2

#define PIPEHANDLER_LOCK_REQ 0x10
#define PIPEHANDLER_LOCK_ACK 0x14
#define PIPEHANDLER_LOCK_EN BIT(0)

#define PIPEHANDLER_AON_GEN 0x1C
#define PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN BIT(4)
#define PIPEHANDLER_AON_GEN_DWC3_RESET_N BIT(0)

#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NATIVE_RESET BIT(12)
#define PIPEHANDLER_DUMMY_PHY_EN BIT(15)
#define PIPEHANDLER_NATIVE_POWER_DOWN GENMASK(3, 0)

#define PIPEHANDLER_LOCK_ACK_TIMEOUT_US 1000

/* USB2 PHY regs */
#define USB2PHY_USBCTL 0x00
#define USB2PHY_USBCTL_USB_MODE 0x7
#define USB2PHY_USBCTL_RUN 2
#define USB2PHY_USBCTL_ISOLATION 4

#define USB2PHY_CTL 0x04
#define USB2PHY_CTL_RESET BIT(0)
#define USB2PHY_CTL_PORT_RESET BIT(1)
#define USB2PHY_CTL_APB_RESET_N BIT(2)
#define USB2PHY_CTL_SIDDQ BIT(3)

#define USB2PHY_SIG 0x08
#define USB2PHY_SIG_VBUSDET_FORCE_VAL BIT(0)
#define USB2PHY_SIG_VBUSDET_FORCE_EN BIT(1)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL BIT(2)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_EN BIT(3)
#define USB2PHY_SIG_HOST (7 << 12)

#define USB2PHY_MISCTUNE 0x1c
#define USB2PHY_MISCTUNE_APBCLK_GATE_OFF BIT(29)
#define USB2PHY_MISCTUNE_REFCLK_GATE_OFF BIT(30)

enum atcphy_generation {
	ATCPHY_GENERATION_T8103,
	ATCPHY_GENERATION_T8122,
};

enum atcphy_dp_link_rate {
	ATCPHY_DP_LINK_RATE_RBR,
	ATCPHY_DP_LINK_RATE_HBR,
	ATCPHY_DP_LINK_RATE_HBR2,
	ATCPHY_DP_LINK_RATE_HBR3,
};

/**
 * enum atcphy_pipehandler_state - States of the PIPE mux interface ("pipehandler")
 * @ATCPHY_PIPEHANDLER_STATE_DUMMY: "Dummy PHY" (disables USB3, USB2 only)
 * @ATCPHY_PIPEHANDLER_STATE_USB3: USB3 directly connected to the Type-C port
 * @ATCPHY_PIPEHANDLER_STATE_USB4: USB3 tunneled via USB4/Thunderbolt
 *
 * DWC3's USB3 PIPE interface is connected to a multiplexer inside this PHY
 * which can switch between a dummy state (which effectively disables any USB3
 * support and falls back to USB2 only operation via the separate ULPI interface),
 * a USB3 state (for regular USB3 or USB3+DisplayPort operation) and a USB4 state
 * (for USB3 tunneled via USB4/Thunderbolt).
 */
enum atcphy_pipehandler_state {
	ATCPHY_PIPEHANDLER_STATE_DUMMY,
	ATCPHY_PIPEHANDLER_STATE_USB3,
	ATCPHY_PIPEHANDLER_STATE_USB4,
};

/**
 * enum atcphy_mode - Operating modes of the PHY
 * @APPLE_ATCPHY_MODE_OFF: all PHYs powered off
 * @APPLE_ATCPHY_MODE_USB2: Nothing on the four SS lanes (i.e. USB2 only on D-/+)
 * @APPLE_ATCPHY_MODE_USB3: USB3 on two lanes, nothing on the other two
 * @APPLE_ATCPHY_MODE_USB3_DP: USB3 on two lanes and DisplayPort on the other two
 * @APPLE_ATCPHY_MODE_TBT: Thunderbolt on all lanes
 * @APPLE_ATCPHY_MODE_USB4: USB4 on all lanes
 * @APPLE_ATCPHY_MODE_DP: DisplayPort on all lanes
 */
enum atcphy_mode {
	APPLE_ATCPHY_MODE_OFF,
	APPLE_ATCPHY_MODE_USB2,
	APPLE_ATCPHY_MODE_USB3,
	APPLE_ATCPHY_MODE_USB3_DP,
	APPLE_ATCPHY_MODE_TBT,
	APPLE_ATCPHY_MODE_USB4,
	APPLE_ATCPHY_MODE_DP,
};

enum atcphy_lane {
	APPLE_ATCPHY_LANE_0,
	APPLE_ATCPHY_LANE_1,
};

/* Link rate configuration, field names are taken from XNU debug output or register names */
struct atcphy_dp_link_rate_configuration {
	u16 freqinit_count_target;
	u16 fbdivn_frac_den;
	u16 fbdivn_frac_num;
	u16 pclk_div_sel;
	u8 lfclk_ctrl;
	u8 vclk_op_divn;
	bool plla_clkout_vreg_bypass;
	bool txa_ldoclk_bypass;
	bool txa_div2_en;
};

/* Crossbar and lane configuration */
struct atcphy_mode_configuration {
	u32 crossbar;
	u32 crossbar_dp_single_pma;
	bool crossbar_dp_both_pma;
	enum atcphy_lane_mode lane_mode[2];
	bool dp_lane[2];
	bool set_swap;
};

struct apple_atcphy;

/**
 * struct atcphy_hw - SoC-specific PHY description
 * @configure_post_tunable: Generation-specific setup after the tunables
 * @gen: Register programming generation
 * @aciophy_lane_mode: Lane mode register offset
 * @aciophy_crossbar: Crossbar register offset
 * @has_usb4: A USB4/Thunderbolt controller sits behind the PHY
 * @pipe_mux_clk: PIPE clock selection field mask
 * @pipe_mux_data: PIPE data selection field mask
 * @has_usb2phy_reg: The PHY has the secondary eUSB2 register bank (T8140)
 * @has_t8140_dp: Uses the T8140 DisplayPort tunable bundle
 * @optional_tunables: The bootloader may leave out the common-a tunables (this
 *                     generation has none) and the SuperSpeed tunables; USB2
 *                     still works without the latter
 * @park_pipe_unlocked: A stopped controller may park without a PIPE handshake
 * @dp_t8122: DisplayPort runs the T8122 AUX, AUSPLL and lane sequences
 * @park_dummy_phy: Enable the dummy PIPE backend whenever the PIPE is parked on
 *                  it; the lock handshake that starts the next PIPE change is
 *                  only acknowledged while it runs
 * @restore_after_pd_off: The PHY's power domain may be switched off in system
 *                        sleep, which resets the block; bring the tracked
 *                        state back once the domain is on again
 */
struct atcphy_hw {
	void (*configure_post_tunable)(struct apple_atcphy *, enum atcphy_mode);
	enum atcphy_generation gen;
	int aciophy_lane_mode;
	int aciophy_crossbar;
	bool has_usb4;
	u32 pipe_mux_clk;
	u32 pipe_mux_data;
	bool has_usb2phy_reg;
	bool optional_tunables;
	bool has_t8140_dp;
	bool dp_t8122;
	bool park_pipe_unlocked;
	bool park_dummy_phy;
	bool restore_after_pd_off;
};

/**
 * struct apple_atcphy - Apple Type-C PHY device struct
 * @np: Device node pointer
 * @dev: Device pointer
 * @tunables: Firmware-provided tunable parameters
 * @tunables.axi2af: AXI to AF interface tunables
 * @tunables.common: Common tunables for all lanes
 * @tunables.lane_usb3: USB3 lane-specific tunables
 * @tunables.lane_dp: DisplayPort lane-specific tunables
 * @tunables.lane_usb4: USB4 lane-specific tunables
 * @tunables.dp_common_pre: T8140 DisplayPort common setup
 * @tunables.dp_lpdptx: T8140 DisplayPort LPDPTX setup
 * @tunables.dp_common_post: T8140 DisplayPort common completion
 * @tunables.usb2phy_reg_dflt: Defaults for the secondary eUSB2 register bank
 * @hw: SoC-specific PHY description
 * @ss_tunables: The complete SuperSpeed tunable set was supplied
 * @dp_tunables: T8140 has its complete DP bundle, or T8122 its common calibration
 * @aux_power_requested: AUX power-up began and needs matching shutdown
 * @mode_dirty: Mode transition needs cleanup before it can be retried
 * @dp_idle_pmas: J700 PMAs with an incomplete routing idle sequence
 * @dp_owned_pmas: J700 PMAs requiring main-link cleanup
 * @dp_usb_context_valid: Physical PMAs retaining their pre-DP USB field values
 * @dp_usb_context_dirty: PMAs whose static fields were programmed by DP
 * @usb_off_pending: Failed full OFF must finish before a new powered epoch
 * @usb_off_drained: Masked recovery data was discharged before OFF began
 * @pipe_pending: PIPE handoff still requires acknowledged dummy parking
 * @dp_usb_context: Saved bits from the public per-PMA DP write masks
 * @dp_spread_spectrum: J700 main-link spread-spectrum state
 * @dp_presets: Firmware-derived per-rate drive presets
 * @dp_presets_valid: Firmware supplied each drive-preset table
 * @dp_only: A DisplayPort-only instance without a USB side, such as the PHY
 *           behind the T6030 HDMI port; it maps only its core window
 * @fixed_usb2: The USB2 pairs go to a fixed hub on the USB controller, which
 *              stays in host mode; the PHY provides USB2 from probe on
 * @typec_mode: Mode the Type-C mux last asked for; a fixed-hub port returns to
 *              it when the PHY is brought back up after a power-off
 * @host_active: dwc3 has selected host mode on the USB3 PHY; a fixed-hub port
 *               then routes the PIPE itself on later Type-C mode changes
 * @usb3_port_off: The SuperSpeed root port of a fixed-hub controller was powered
 *                 off for a PIPE or lane change and is not powered on again yet
 * @usb3_port_work: Retries the power-on of @usb3_port_off after a USB failure
 * @usb3_port_retries: Retries left for @usb3_port_work
 * @mode: Current PHY operating mode
 * @swap_lanes: True if lanes must be swapped due to cable orientation
 * @dp_link_rate: DisplayPort link rate
 * @pipe_state: Backend the PIPE mux ("pipehandler") is routed to
 * @tunnel_dual_stream: This PHY connector has qualified dual-stream tunnel wiring
 * @tunnel_routes_present: Qualified dual-stream routes exist on this SoC
 * @tunnel_clock_on: True while the DisplayPort-over-Thunderbolt pixel clock runs
 * @tunnel_attempted: A T602X tunnel clock setup has been attempted
 * @tunnel_saved: T602X PHY registers have been saved for tunnel teardown
 * @tunnel_saved_regs: Original values of the T602X tunnel clock registers
 * @tunnel_rate: DP link rate code the t8103 tunnel pixel clock is set up for
 * @tunnel_users: DP IN adapters (BIT(dpin)) whose tunnel pixel clock runs; on
 *	t8103 and t600x both share the one clock
 * @tunnel_dpin_rate: DP link rate code each T602X/T6030 DP IN clock runs at
 * @tunnel_source: T6030 display source owning each DP IN clock
 * @dp_t8122: DisplayPort state of a T8122 generation PHY
 * @dp_t8122.aux: The AUX channel block is powered
 * @dp_t8122.pll: The AUSPLL runs for the DisplayPort main link
 * @dp_t8122.pairs: Lane pairs (bit mask) whose DisplayPort transmitters run
 * @dp_t8122.rate: Main link rate in Mb/s per lane, 0 while stopped
 * @pipe_state: Backend the PIPE mux ("pipehandler") is routed to
 * @t8103_dummy_pending: A live T8103-generation handoff still needs its unlock ACK
 * @t8103_usb3_retry: USB3 routing owed after a deferred role or parked lane change
 * @t8103_usb3_workaround: Base M1 admission for the USB3 handoff workaround
 * @pd_nb: Power domain notifier, registered with restore_after_pd_off
 * @pd_was_off: The power domain was switched off since the state was last
 *              brought back; set from the notifier, cleared under @lock
 * @regs: Memory-mapped registers
 * @regs.core: Core registers
 * @regs.axi2af: AXI to Apple Fabric interface registers
 * @regs.usb2phy: USB2 PHY registers
 * @regs.usb2phy_reg: Secondary eUSB2 register bank (T8140)
 * @regs.pipehandler: USB3 PIPE interface ("pipehandler") registers
 * @regs.lpdptx: DisplayPort registers
 * @res: Resources for memory-mapped registers, used to verify that tunables aren't out of bounds
 * @res.core: Core register resource
 * @res.lpdptx: DisplayPort register resource
 * @res.axi2af: AXI to Apple Fabric interface resource
 * @res.usb2phy_reg: Secondary eUSB2 register bank resource
 * @phys: PHY instances
 * @phys.usb2: USB2 PHY instance
 * @phys.usb3: USB3 PHY instance
 * @phys.dp: DisplayPort PHY instance
 * @phy_provider: PHY provider instance
 * @rcdev: Reset controller device
 * @sw: Type-C switch instance
 * @mux: Type-C mux instance
 * @lock: Mutex for synchronizing register access across PHY, Type-C switch/mux and reset controller
 */
struct apple_atcphy {
	struct device_node *np;
	struct device *dev;

	struct {
		struct apple_tunable *axi2af;
		struct apple_tunable *common[2];
		struct apple_tunable *lane_usb3[2];
		struct apple_tunable *lane_dp[2];
		struct apple_tunable *lane_usb4[2];
		struct apple_tunable *usb2phy_reg_dflt;
		struct apple_tunable *dp_common_pre;
		struct apple_tunable *dp_lpdptx;
		struct apple_tunable *dp_common_post;
	} tunables;

	const struct atcphy_hw *hw;
	bool ss_tunables;
	bool dp_tunables;
	bool aux_power_requested;
	bool mode_dirty;
	u8 dp_idle_pmas;
	u8 dp_owned_pmas;
	u8 dp_usb_context_valid, dp_usb_context_dirty;
	u32 dp_usb_context[2][27];
	bool usb_off_pending, usb_off_drained, pipe_pending;
	bool dp_spread_spectrum;
	u32 dp_presets[5][16];
	bool dp_presets_valid[5];
	bool dp_only;
	bool fixed_usb2;
	enum atcphy_mode typec_mode;
	bool host_active;
	bool usb3_port_off;
	struct delayed_work usb3_port_work;
	unsigned int usb3_port_retries;
	enum atcphy_mode mode;
	int dp_link_rate;
	bool swap_lanes;
	enum atcphy_pipehandler_state pipe_state;
	bool t8103_dummy_pending;
	bool t8103_usb3_retry;
	bool t8103_usb3_workaround;

	struct notifier_block pd_nb;
	bool pd_was_off;

	bool tunnel_dual_stream;
	bool tunnel_routes_present;
	bool tunnel_clock_on;
	bool tunnel_attempted, tunnel_saved;
	u32 tunnel_saved_regs[12];
	u8 tunnel_rate;
	u8 tunnel_users;
	u8 tunnel_dpin_rate[2];
	unsigned int tunnel_source[2]; /* T6030 source owning each DP IN clock */

	struct {
		bool aux;
		bool pll;
		u8 pairs;
		unsigned int rate;
	} dp_t8122;

	struct {
		void __iomem *core;
		void __iomem *axi2af;
		void __iomem *usb2phy;
		void __iomem *usb2phy_reg;
		void __iomem *pipehandler;
		void __iomem *lpdptx;
	} regs;

	struct {
		struct resource *core;
		struct resource *lpdptx;
		struct resource *axi2af;
		struct resource *usb2phy_reg;
	} res;

	struct {
		struct phy *usb2;
		struct phy *usb3;
		struct phy *dp;
	} phys;
	struct phy_provider *phy_provider;

	struct reset_controller_dev rcdev;

	struct mutex lock;
};

static const struct {
	const struct atcphy_mode_configuration normal;
	const struct atcphy_mode_configuration swapped;
	bool enable_dp_aux;
	enum atcphy_pipehandler_state pipehandler_state;
} atcphy_modes[] = {
	[APPLE_ATCPHY_MODE_OFF] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false, /* doesn't matter since the SS lanes are off */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_DUMMY,
	},
	[APPLE_ATCPHY_MODE_USB2] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false, /* doesn't matter since the SS lanes are off */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_DUMMY,
	},
	[APPLE_ATCPHY_MODE_USB3] = {
		/*
		 * Setting up the lanes as DP/USB3 is intentional here, USB3/USB3 does not work
		 * and isn't required since this PHY does not support 20GBps mode anyway.
		 * The only difference to APPLE_ATCPHY_MODE_USB3_DP is that DP Aux is not enabled.
		 */
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {false, true},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_USB3},
			.dp_lane = {true, false},
			.set_swap = true,
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB3,
	},
	[APPLE_ATCPHY_MODE_USB3_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {false, true},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_USB3},
			.dp_lane = {true, false},
			.set_swap = true,
		},
		.enable_dp_aux = true,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB3,
	},
	[APPLE_ATCPHY_MODE_TBT] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false, /* intentionally false */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_DUMMY,
	},
	[APPLE_ATCPHY_MODE_USB4] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false, /* intentionally false */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB4,
	},
	[APPLE_ATCPHY_MODE_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100,
			.crossbar_dp_both_pma = true,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {true, true},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false, /* intentionally false */
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {true, true},
			.set_swap = false, /* intentionally false */
		},
		.enable_dp_aux = true,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_DUMMY,
	},
};

static const struct atcphy_dp_link_rate_configuration dp_lr_config[] = {
	[ATCPHY_DP_LINK_RATE_RBR] = {
		.freqinit_count_target = 0x21c,
		.fbdivn_frac_den = 0x0,
		.fbdivn_frac_num = 0x0,
		.pclk_div_sel = 0x13,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x2,
		.plla_clkout_vreg_bypass = true,
		.txa_ldoclk_bypass = true,
		.txa_div2_en = true,
	},
	[ATCPHY_DP_LINK_RATE_HBR] = {
		.freqinit_count_target = 0x1c2,
		.fbdivn_frac_den = 0x3ffe,
		.fbdivn_frac_num = 0x1fff,
		.pclk_div_sel = 0x9,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x2,
		.plla_clkout_vreg_bypass = true,
		.txa_ldoclk_bypass = true,
		.txa_div2_en = false,
	},
	[ATCPHY_DP_LINK_RATE_HBR2] = {
		.freqinit_count_target = 0x1c2,
		.fbdivn_frac_den = 0x3ffe,
		.fbdivn_frac_num = 0x1fff,
		.pclk_div_sel = 0x4,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x0,
		.plla_clkout_vreg_bypass = true,
		.txa_ldoclk_bypass = true,
		.txa_div2_en = false,
	},
	[ATCPHY_DP_LINK_RATE_HBR3] = {
		.freqinit_count_target = 0x2a3,
		.fbdivn_frac_den = 0x3ffc,
		.fbdivn_frac_num = 0x2ffd,
		.pclk_div_sel = 0x4,
		.lfclk_ctrl = 0x6,
		.vclk_op_divn = 0x0,
		.plla_clkout_vreg_bypass = false,
		.txa_ldoclk_bypass = false,
		.txa_div2_en = false,
	},
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);

	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void core_mask32(struct apple_atcphy *atcphy, u32 reg, u32 mask, u32 set)
{
	mask32(atcphy->regs.core + reg, mask, set);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void core_set32(struct apple_atcphy *atcphy, u32 reg, u32 set)
{
	core_mask32(atcphy, reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

static inline void core_clear32(struct apple_atcphy *atcphy, u32 reg, u32 clear)
{
	core_mask32(atcphy, reg, clear, 0);
}

static const struct atcphy_mode_configuration *atcphy_get_mode_config(struct apple_atcphy *atcphy,
								      enum atcphy_mode mode)
{
	if (atcphy->swap_lanes)
		return &atcphy_modes[mode].swapped;
	else
		return &atcphy_modes[mode].normal;
}

/* Readback only; callers hold the PHY mutex and keep the fixed host live. */
static inline u32 core_read32(struct apple_atcphy *atcphy, u32 reg)
{
	return readl(atcphy->regs.core + reg);
}

/* Static configuration fields already read/modified by the public DP path. */
static void j700_dp_tunables(struct apple_atcphy *atcphy)
{
	apple_tunable_apply(atcphy->regs.axi2af, atcphy->tunables.axi2af);
	apple_tunable_apply(atcphy->regs.core, atcphy->tunables.dp_common_pre);
	apple_tunable_apply(atcphy->regs.lpdptx, atcphy->tunables.dp_lpdptx);
	apple_tunable_apply(atcphy->regs.core, atcphy->tunables.dp_common_post);
}

static void atcphy_apply_tunables(struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	const int lane0 = atcphy->swap_lanes ? 1 : 0;
	const int lane1 = atcphy->swap_lanes ? 0 : 1;

	/*
	 * A port behind a fixed hub runs USB2 with the SuperSpeed lanes off from
	 * probe on. The T8140 bring-up left the common and AXI2AF tunables
	 * unapplied in that state; they are applied together with the lane
	 * tunables on the first SuperSpeed mode change.
	 */
	if (mode == APPLE_ATCPHY_MODE_USB2 && atcphy->fixed_usb2)
		return;

	if (atcphy->hw->has_t8140_dp && mode == APPLE_ATCPHY_MODE_DP) {
		j700_dp_tunables(atcphy);
		return;
	}

	apple_tunable_apply(atcphy->regs.core, atcphy->tunables.common[0]);
	apple_tunable_apply(atcphy->regs.axi2af, atcphy->tunables.axi2af);
	apple_tunable_apply(atcphy->regs.core, atcphy->tunables.common[1]);

	switch (mode) {
	/*
	 * USB 3.2 Gen 2x2 / SuperSpeed 20Gbps is not supported by this hardware and applying USB3
	 * tunables to both lanes does not result in a working PHY configuration. Thus, both
	 * USB3-only and USB3/DP get the same tunable setup here.
	 */
	case APPLE_ATCPHY_MODE_USB3:
	case APPLE_ATCPHY_MODE_USB3_DP:
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_usb3[lane0]);
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_dp[lane1]);
		break;

	case APPLE_ATCPHY_MODE_DP:
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_dp[lane0]);
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_dp[lane1]);
		break;

	/*
	 * Even though the various Thunderbolt versions and USB4 are different protocols they need
	 * the same tunables. The actual protocol-specific setup happens inside the Thunderbolt/USB4
	 * native host interface.
	 */
	case APPLE_ATCPHY_MODE_TBT:
	case APPLE_ATCPHY_MODE_USB4:
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_usb4[lane0]);
		apple_tunable_apply(atcphy->regs.core, atcphy->tunables.lane_usb4[lane1]);
		break;

	case APPLE_ATCPHY_MODE_OFF:
	case APPLE_ATCPHY_MODE_USB2:
		break;
	}
	if (atcphy->hw->has_t8140_dp && mode == APPLE_ATCPHY_MODE_USB3_DP)
		j700_dp_tunables(atcphy);

}

static int atcphy_pipehandler_lock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;
	u32 requested;

	/* The tests stand in for the controller that acknowledges the lock */
	if (IS_ENABLED(CONFIG_PHY_APPLE_ATC_KUNIT_TEST))
		KUNIT_STATIC_STUB_REDIRECT(atcphy_pipehandler_lock, atcphy);

	requested = readl(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ);
	dev_dbg(atcphy->dev, "PIPE request: req=%#x mode=%u pipe=%u host=%u dirty=%u\n",
		requested, atcphy->mode, atcphy->pipe_state,
		atcphy->host_active, atcphy->mode_dirty);
	if (requested & PIPEHANDLER_LOCK_EN) {
		dev_dbg(atcphy->dev, "Pipehandler lock already requested\n");
	} else {
		set32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK_EN);
	}

	ret = readl_poll_timeout(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK, reg,
				 reg & PIPEHANDLER_LOCK_EN, 10, PIPEHANDLER_LOCK_ACK_TIMEOUT_US);
	if (ret) {
		dev_dbg(atcphy->dev, "PIPE lock timeout: ack=%#x mode=%u pipe=%u dirty=%u\n",
			reg, atcphy->mode, atcphy->pipe_state, atcphy->mode_dirty);
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ, 1);
		dev_warn(atcphy->dev, "Pipehandler lock not acked.\n");
	}

	return ret;
}

/* Retry only a live legacy PIPE: USB4 and stopped parking cannot clock the ACK. */
static int atcphy_pipehandler_lock_live(struct apple_atcphy *atcphy)
{
	unsigned int attempts = atcphy->t8103_usb3_workaround ? 10 : 1;
	int ret;

	for (unsigned int attempt = 0; attempt < attempts; attempt++) {
		ret = atcphy_pipehandler_lock(atcphy);
		if (!ret)
			return 0;
		if (attempt + 1 < attempts)
			usleep_range(5000, 10000);
	}
	return ret;
}

static int atcphy_pipehandler_unlock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK_EN);
	ret = readl_poll_timeout(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK, reg,
				 !(reg & PIPEHANDLER_LOCK_EN), 10, PIPEHANDLER_LOCK_ACK_TIMEOUT_US);
	if (ret) {
		dev_dbg(atcphy->dev, "PIPE unlock timeout: ack=%#x mode=%u pipe=%u dirty=%u\n",
			reg, atcphy->mode, atcphy->pipe_state, atcphy->mode_dirty);
		dev_warn(atcphy->dev, "Pipehandler lock release not acked.\n");
	}

	return ret;
}

static int atcphy_pipehandler_check(struct apple_atcphy *atcphy)
{
	int ret;

	lockdep_assert_held(&atcphy->lock);

	if (readl(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK) & PIPEHANDLER_LOCK_EN) {
		dev_warn(atcphy->dev, "Pipehandler already locked\n");

		ret = atcphy_pipehandler_unlock(atcphy);
		if (ret) {
			dev_err(atcphy->dev, "Failed to unlock pipehandler\n");
			return ret;
		}
	}

	return 0;
}

static void atcphy_clear_nonselected_phy_reset(struct apple_atcphy *atcphy)
{
	/* Clear reset for non-selected USB3 PHY (?) */
	mask32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	       PIPEHANDLER_NATIVE_POWER_DOWN, FIELD_PREP(PIPEHANDLER_NATIVE_POWER_DOWN, 3));
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
		PIPEHANDLER_NATIVE_RESET);
}

static int atcphy_configure_pipehandler_usb3_host_t8122(struct apple_atcphy *atcphy)
{
	int ret, error = 0;
	u32 reg;

	if (IS_ENABLED(CONFIG_PHY_APPLE_ATC_KUNIT_TEST))
		KUNIT_STATIC_STUB_REDIRECT(atcphy_configure_pipehandler_usb3_host_t8122, atcphy);

	core_set32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_CLOCK_EN);
	/* This should be a mask, but unsure what bits should be cleared */
	writel(ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PIPE_RESET_N_OV | FIELD_PREP(ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PWR_DOWN_OV, 2),
	       atcphy->regs.core + ACIOPHY_BIST_MAC_USBPHY_CFG_OV1);
	/* This should be a mask, but unsure what bits should be cleared */
	writel(ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_RESET_N_OV | ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_PWR_DOWN_OV,
	       atcphy->regs.core + ACIOPHY_BIST_MAC_USBPHY_CFG_OV2);
	core_set32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_BIST_CFG1, reg,
				 (reg & ACIOPHY_BIST_CFG1_USB_EN), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_dbg(atcphy->dev, "BIST USB_EN phase=1 value=%#x mode=%u pipe=%u\n",
			reg, atcphy->mode, atcphy->pipe_state);
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_BIST_CFG1_USB_EN\n");
	}
	core_clear32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_BIST_MAC_USBPHY_STATUS2, reg,
				 !(reg & ACIOPHY_BIST_MAC_USBPHY_STATUS2_LN0_PHY_STATUS2), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_dbg(atcphy->dev, "BIST STATUS2 phase=1 value=%#x swap=%u mode=%u pipe=%u\n",
			reg, atcphy->swap_lanes, atcphy->mode, atcphy->pipe_state);
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_BIST_MAC_USBPHY_STATUS2_LN0_PHY_STATUS2\n");
	}

	core_set32(atcphy, ACIOPHY_BIST_CFG2, ACIOPHY_BIST_CFG2_LN0_PHY_STATUS_RE);
	udelay(10);
	core_clear32(atcphy, ACIOPHY_BIST_CFG2, ACIOPHY_BIST_CFG2_LN0_PHY_STATUS_RE);
	core_mask32(atcphy, ACIOPHY_BIST_MAC_USBPHY_CFG_OV1, ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PWR_DOWN_OV,
		    FIELD_PREP(ACIOPHY_BIST_MAC_USBPHY_CFG_OV1_LN0_PWR_DOWN_OV, 3));

	/* there is a read and write of ACIOPHY_BIST_MAC_USBPHY_CFG_OV2 here with same values */
	udelay(10);
	core_set32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_BIST_CFG1, reg,
				 (reg & ACIOPHY_BIST_CFG1_USB_EN), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_dbg(atcphy->dev, "BIST USB_EN phase=2 value=%#x mode=%u pipe=%u\n",
			reg, atcphy->mode, atcphy->pipe_state);
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_BIST_CFG1_USB_EN 2nd toggle\n");
	}
	core_clear32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_BIST_MAC_USBPHY_STATUS2, reg,
				 !(reg & ACIOPHY_BIST_MAC_USBPHY_STATUS2_LN0_PHY_STATUS2), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_dbg(atcphy->dev, "BIST STATUS2 phase=2 value=%#x swap=%u mode=%u pipe=%u\n",
			reg, atcphy->swap_lanes, atcphy->mode, atcphy->pipe_state);
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_BIST_MAC_USBPHY_STATUS2_LN0_PHY_STATUS2 2nd time\n");
	}
	core_set32(atcphy, ACIOPHY_BIST_CFG2, ACIOPHY_BIST_CFG2_LN0_PHY_STATUS_RE);
	udelay(10);
	core_clear32(atcphy, ACIOPHY_BIST_CFG2, ACIOPHY_BIST_CFG2_LN0_PHY_STATUS_RE);

	atcphy_clear_nonselected_phy_reset(atcphy);

	core_clear32(atcphy, ACIOPHY_BIST_MAC_USBPHY_CFG_OV2,
		     ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_RESET_N_OV | ACIOPHY_BIST_MAC_USBPHY_CFG_OV2_LN0_PWR_DOWN_OV);
	core_set32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_BIST_CFG1, reg,
				 (reg & ACIOPHY_BIST_CFG1_USB_EN), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_dbg(atcphy->dev, "BIST USB_EN phase=3 value=%#x mode=%u pipe=%u\n",
			reg, atcphy->mode, atcphy->pipe_state);
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_BIST_CFG1_USB_EN 3rd toggle\n");
	}
	core_clear32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_USB_EN);
	core_clear32(atcphy, ACIOPHY_BIST_CFG1, ACIOPHY_BIST_CFG1_CLOCK_EN);
	return error;
}

static int atcphy_configure_pipehandler_usb3_host_t8103(struct apple_atcphy *atcphy)
{
	int ret, error = 0;
	u32 reg;

	core_set32(atcphy, ACIOPHY_TOP_BIST_PHY_CFG0,
		   ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N);
	core_set32(atcphy, ACIOPHY_TOP_BIST_OV_CFG, ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV);
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
				 !(reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK23), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_warn(atcphy->dev,
			 "Timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK23\n");
	}

	core_set32(atcphy, ACIOPHY_TOP_BIST_READ_CTRL,
		   ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE);
	core_clear32(atcphy, ACIOPHY_TOP_BIST_READ_CTRL,
		     ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE);

	core_mask32(atcphy, ACIOPHY_TOP_BIST_PHY_CFG1,
		    ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN,
		    FIELD_PREP(ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN, 3));

	core_set32(atcphy, ACIOPHY_TOP_BIST_OV_CFG,
		   ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV);
	core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
		   ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN);
	core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
		   ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN);
	writel(0, atcphy->regs.core + ACIOPHY_TOP_BIST_CIOPHY_CFG1);

	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
				 (reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK0), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_warn(atcphy->dev,
			 "timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK0\n");
	}

	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
				 !(reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK23), 10, 10000);
	if (ret) {
		if (!error)
			error = ret;
		dev_warn(atcphy->dev,
			 "timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK23\n");
	}

	atcphy_clear_nonselected_phy_reset(atcphy);

	/* More BIST stuff (?) */
	writel(0, atcphy->regs.core + ACIOPHY_TOP_BIST_OV_CFG);
	core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
		   ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN);
	core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
		   ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN);
	return error;
}

static void atcphy_pipehandler_set_mux(struct apple_atcphy *atcphy, u32 data, u32 clk)
{
	mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL, atcphy->hw->pipe_mux_clk,
	       field_prep(atcphy->hw->pipe_mux_clk, PIPEHANDLER_MUX_CTRL_CLK_OFF));
	udelay(10);
	mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL, atcphy->hw->pipe_mux_data,
	       field_prep(atcphy->hw->pipe_mux_data, data));
	udelay(10);
	mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL, atcphy->hw->pipe_mux_clk,
	       field_prep(atcphy->hw->pipe_mux_clk, clk));
	udelay(10);
}

static int atcphy_configure_pipehandler_usb3(struct apple_atcphy *atcphy, bool host)
{
	int ret;

	ret = atcphy_pipehandler_check(atcphy);
	if (ret)
		return ret;

	/*
	 * Only host mode requires this unknown BIST sequence to work correctly, possibly due to
	 * some hardware quirk. Guest mode breaks if we try to apply this sequence.
	 */
	if (host) {
		/* Force disable link detection */
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE_VALUES,
			PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | PIPEHANDLER_OVERRIDE_VAL_RXDETECT1);
		set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
		      PIPEHANDLER_OVERRIDE_RXVALID);
		set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
		      PIPEHANDLER_OVERRIDE_RXDETECT);

		ret = atcphy_pipehandler_lock(atcphy);
		if (ret) {
			dev_err(atcphy->dev, "Failed to lock pipehandler");
			return ret;
		}

		/* BIST dance */
		if (atcphy->hw->gen == ATCPHY_GENERATION_T8103)
			ret = atcphy_configure_pipehandler_usb3_host_t8103(atcphy);
		else
			ret = atcphy_configure_pipehandler_usb3_host_t8122(atcphy);
		if (ret) {
			int cleanup;

			clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
				PIPEHANDLER_OVERRIDE_RXVALID | PIPEHANDLER_OVERRIDE_RXDETECT);
			cleanup = atcphy_pipehandler_unlock(atcphy);
			if (cleanup)
				dev_warn(atcphy->dev, "PIPE cleanup after BIST: %d\n", cleanup);
			/* Native BIST changed state; a retry must rebuild after parking. */
			atcphy->mode_dirty = true;
			return ret;
		}
	}

	/* Configure PIPE mux to USB3 PHY */
	atcphy_pipehandler_set_mux(atcphy, PIPEHANDLER_MUX_CTRL_DATA_USB3,
				   PIPEHANDLER_MUX_CTRL_CLK_USB3);
	/* Track the selected backend even if its completion ACK fails. */
	atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_USB3;

	/* Remove link detection override */
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

	/* Pipehandler was only locked when the BIST sequence was applied for host mode */
	if (host) {
		ret = atcphy_pipehandler_unlock(atcphy);
		if (ret) {
			dev_warn(atcphy->dev, "Failed to unlock pipehandler");
			atcphy->mode_dirty = true;
			return ret;
		}
	}

	return 0;
}

static int atcphy_configure_pipehandler_usb4(struct apple_atcphy *atcphy)
{
	int ret;

	ret = atcphy_pipehandler_check(atcphy);
	if (ret)
		return ret;

	/* Force disable link detection */
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE_VALUES,
		PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | PIPEHANDLER_OVERRIDE_VAL_RXDETECT1);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

	/*
	 * The lock is best-effort here: the LOCK_PIPE_IF handshake is clocked by the
	 * DWC3 PIPE clock, which is not running once the lanes are routed to the USB4
	 * controller, so the ACK legitimately never asserts. The handshake is
	 * advisory in this path, and atcphy_configure_pipehandler_dummy() below
	 * already relies on the same warn-and-continue behaviour. Only the USB3 host
	 * BIST sequence genuinely needs the PIPE quiesced and keeps the lock fatal.
	 */
	ret = atcphy_pipehandler_lock(atcphy);
	if (ret)
		dev_warn(atcphy->dev, "Failed to lock pipehandler\n");

	/* Configure PIPE mux to the USB4/Thunderbolt controller */
	atcphy_pipehandler_set_mux(atcphy, PIPEHANDLER_MUX_CTRL_DATA_USB4,
				   PIPEHANDLER_MUX_CTRL_CLK_USB4);

	/* Remove link detection override */
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

	ret = atcphy_pipehandler_unlock(atcphy);
	if (ret)
		dev_warn(atcphy->dev, "Failed to unlock pipehandler\n");

	return 0;
}

/*
 * The T8140 DWC3 core does not complete its initialisation against a parked
 * PIPE unless the dummy PHY behind the mux is enabled as well; selecting the
 * dummy backend in PIPEHANDLER_MUX_CTRL is not enough. The earlier SoCs come
 * up and run USB2-only with the bit clear, so they are left alone.
 *
 * T8122 PHYs come out of boot with the bit set, and the PIPE lock handshake
 * that starts the routing to USB3 is only acknowledged while it is: once the
 * bit is clear (it resets to clear with the PHY's power domain), the next USB3
 * start times out with "Pipehandler lock not acked". These PHYs keep it set
 * on every park, see park_dummy_phy.
 */
static void atcphy_enable_dummy_phy(struct apple_atcphy *atcphy)
{
	set32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	      PIPEHANDLER_DUMMY_PHY_EN);
}

/*
 * Whether a controller may still be driving the PIPE, which then has to
 * acknowledge the lock before the PIPE moves. A handoff that did not complete
 * leaves the PIPE in an unknown state. A controller behind a fixed hub only
 * drives it in host mode, which it has left by the time it powers the USB3 PHY
 * off or asserts its reset.
 */
static bool atcphy_pipe_live(const struct apple_atcphy *atcphy)
{
	if (atcphy->hw->has_t8140_dp && atcphy->fixed_usb2)
		return atcphy->host_active &&
		       (atcphy->pipe_pending ||
			atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_USB3);

	return atcphy->pipe_pending;
}

static bool atcphy_needs_dummy_phy(struct apple_atcphy *atcphy)
{
	return !atcphy->hw->has_usb4 || atcphy->hw->park_dummy_phy;
}

static int atcphy_configure_pipehandler_dummy(struct apple_atcphy *atcphy, bool lock)
{
	bool live_usb3 = atcphy_pipe_live(atcphy);
	int ret;

	ret = atcphy_pipehandler_check(atcphy);
	if (ret) {
		if (live_usb3)
			if (atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY)
				atcphy->mode_dirty = true;
		return ret;
	}

	/* Force disable link detection */
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE_VALUES,
		PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | PIPEHANDLER_OVERRIDE_VAL_RXDETECT1);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

	if (lock) {
		ret = atcphy_pipehandler_lock(atcphy);
		if (ret) {
			/* A live controller must acknowledge before its backend changes. */
			if (live_usb3) {
				if (atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY)
					atcphy->mode_dirty = true;
				return ret;
			}
			dev_warn(atcphy->dev, "Failed to lock pipehandler");
		}
	}

	/* Switch to dummy PHY */
	atcphy_pipehandler_set_mux(atcphy, PIPEHANDLER_MUX_CTRL_DATA_DUMMY,
				   PIPEHANDLER_MUX_CTRL_CLK_DUMMY);

	if (lock) {
		ret = atcphy_pipehandler_unlock(atcphy);
		if (ret) {
			/* The mux changed, but the handoff has not completed. Retain ownership. */
			if (live_usb3) {
				if (atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY)
					atcphy->mode_dirty = true;
				return ret;
			}
			dev_warn(atcphy->dev, "Failed to unlock pipehandler");
		}
	}

	mask32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	       PIPEHANDLER_NATIVE_POWER_DOWN, FIELD_PREP(PIPEHANDLER_NATIVE_POWER_DOWN, 2));
	set32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	      PIPEHANDLER_NATIVE_RESET);
	if (atcphy_needs_dummy_phy(atcphy))
		atcphy_enable_dummy_phy(atcphy);

	return 0;
}

/*
 * Route the PIPE to the backend the current mode needs. The state is recorded
 * only after the sequence succeeds, so a failed attempt remains retryable.
 */
static int atcphy_configure_pipehandler(struct apple_atcphy *atcphy, bool host)
{
	enum atcphy_pipehandler_state state = atcphy_modes[atcphy->mode].pipehandler_state;
	bool host_bist = host && !atcphy->t8103_usb3_workaround;
	int ret = -EINVAL;

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->t8103_dummy_pending)
		return -EBUSY;

	switch (state) {
	case ATCPHY_PIPEHANDLER_STATE_USB3:
		atcphy->pipe_pending = true;
		/* Legacy host BIST can hide the first xHCI link-detection scan. */
		ret = atcphy_configure_pipehandler_usb3(atcphy, host_bist);
		break;
	case ATCPHY_PIPEHANDLER_STATE_USB4:
		ret = atcphy_configure_pipehandler_usb4(atcphy);
		break;
	case ATCPHY_PIPEHANDLER_STATE_DUMMY:
		ret = atcphy_configure_pipehandler_dummy(atcphy, true);
		break;
	}
	if (!ret) {
		atcphy->pipe_state = state;
		atcphy->pipe_pending = false;
		if (state == ATCPHY_PIPEHANDLER_STATE_USB3)
			atcphy->t8103_usb3_retry = false;
	}

	return ret;
}

/*
 * Park a live T8103 USB3 backend before a Type-C lane change. Unlike legacy
 * shutdown, an unacknowledged handoff must not let the lane change continue.
 */
static int atcphy_park_live_t8103_pipehandler(struct apple_atcphy *atcphy)
{
	const u32 override_mask = PIPEHANDLER_OVERRIDE_RXVALID |
				  PIPEHANDLER_OVERRIDE_RXDETECT;
	const u32 value_mask = PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 |
			      PIPEHANDLER_OVERRIDE_VAL_RXDETECT1;
	void __iomem *pipe = atcphy->regs.pipehandler;
	u32 overrides, values;
	int ret;

	lockdep_assert_held(&atcphy->lock);
	if (!atcphy->t8103_usb3_workaround)
		return -EOPNOTSUPP;

	if (!atcphy->t8103_dummy_pending) {
		if (atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_DUMMY)
			return 0;
		if (atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_USB3)
			return -EBUSY;
		ret = atcphy_pipehandler_check(atcphy);
		if (ret)
			return ret;
		overrides = readl(pipe + PIPEHANDLER_OVERRIDE);
		values = readl(pipe + PIPEHANDLER_OVERRIDE_VALUES);
		clear32(pipe + PIPEHANDLER_OVERRIDE_VALUES, value_mask);
		set32(pipe + PIPEHANDLER_OVERRIDE, override_mask);

		ret = atcphy_pipehandler_lock_live(atcphy);
		if (ret) {
			/* The mux did not move: restore only the fields we touched. */
			mask32(pipe + PIPEHANDLER_OVERRIDE_VALUES, value_mask,
			       values & value_mask);
			mask32(pipe + PIPEHANDLER_OVERRIDE, override_mask,
			       overrides & override_mask);
			return ret;
		}
		atcphy_pipehandler_set_mux(atcphy, PIPEHANDLER_MUX_CTRL_DATA_DUMMY,
					   PIPEHANDLER_MUX_CTRL_CLK_DUMMY);
		/* The physical mux moved even if completion now fails. */
		atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_DUMMY;
		atcphy->t8103_dummy_pending = true;
		atcphy->t8103_usb3_retry = true;
	}

	/* Retry completion without requesting another lock on the dummy clock. */
	ret = atcphy_pipehandler_unlock(atcphy);
	if (ret)
		return ret;
	mask32(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE,
	       PIPEHANDLER_NATIVE_POWER_DOWN, FIELD_PREP(PIPEHANDLER_NATIVE_POWER_DOWN, 2));
	set32(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE, PIPEHANDLER_NATIVE_RESET);
	atcphy->t8103_dummy_pending = false;
	atcphy->pipe_pending = false;
	return 0;
}

/* Route the PIPE back to the dummy backend unless it is there already */
static int atcphy_park_pipehandler(struct apple_atcphy *atcphy, bool stopped)
{
	bool lock = !(stopped && atcphy->hw->park_pipe_unlocked);
	int ret;

	lockdep_assert_held(&atcphy->lock);

	dev_dbg(atcphy->dev, "PIPE park: mode=%u pipe=%u host=%u dirty=%u owned=%u idle=%u\n",
		atcphy->mode, atcphy->pipe_state, atcphy->host_active, atcphy->mode_dirty,
		atcphy->dp_owned_pmas, atcphy->dp_idle_pmas);
	if (atcphy->t8103_dummy_pending ||
	    (!stopped && atcphy->t8103_usb3_workaround &&
	     atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_USB3))
		return atcphy_park_live_t8103_pipehandler(atcphy);
	if (atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_DUMMY && !atcphy->pipe_pending)
		return 0;

	ret = atcphy_configure_pipehandler_dummy(atcphy, lock);
	if (ret) {
		dev_warn(atcphy->dev, "Failed to switch PIPE to dummy: %d\n", ret);
		return ret;
	}
	atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_DUMMY;
	atcphy->pipe_pending = false;
	return 0;
}

static void atcphy_setup_pipehandler(struct apple_atcphy *atcphy)
{
	lockdep_assert_held(&atcphy->lock);

	atcphy_pipehandler_set_mux(atcphy, PIPEHANDLER_MUX_CTRL_DATA_DUMMY,
				   PIPEHANDLER_MUX_CTRL_CLK_DUMMY);
	atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_DUMMY;
	atcphy->pipe_pending = false;
	atcphy->t8103_dummy_pending = false;
	atcphy->t8103_usb3_retry = false;
	if (atcphy_needs_dummy_phy(atcphy))
		atcphy_enable_dummy_phy(atcphy);
}

#define J700_PMA_RECEIVER_CONTROL 0x000c
#define J700_PMA_POWER_CONTROL 0x0200

static int j700_field_update(struct apple_atcphy *atcphy, u32 offset, u32 mask, u32 value)
{
	u32 observed;

	core_mask32(atcphy, offset, mask, value);
	usleep_range(10, 20);
	return readl_poll_timeout_atomic(atcphy->regs.core + offset, observed,
					(observed & mask) == value, 1, 100);
}

static int j700_pma_idle(struct apple_atcphy *atcphy, unsigned int lane)
{
	u32 field;
	int ret;

	if (lane >= 2)
		return -EINVAL;
	for (unsigned int supply = 0; supply < 2; supply++) {
		field = BIT(2 + lane + 4 * supply);
		ret = j700_field_update(atcphy, J700_PMA_RECEIVER_CONTROL, field, field);
		if (ret)
			return ret;
	}
	field = BIT(4 * lane + 3);
	ret = j700_field_update(atcphy, J700_PMA_POWER_CONTROL, field, 0);
	if (ret)
		return ret;
	field = BIT(4 * lane);
	ret = j700_field_update(atcphy, J700_PMA_POWER_CONTROL, field, field);
	if (ret)
		return ret;
	for (unsigned int control = 1; control <= 2; control++) {
		field = BIT(4 * lane + control);
		ret = j700_field_update(atcphy, J700_PMA_POWER_CONTROL, field, 0);
		if (ret)
			return ret;
	}
	return 0;
}

#include "atc-j700-dp.h"

static int atcphy_configure_lanes(struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	const struct atcphy_mode_configuration *mode_cfg = atcphy_get_mode_config(atcphy, mode);

	if (atcphy->hw->has_t8140_dp &&
	    (mode == APPLE_ATCPHY_MODE_USB3 || mode == APPLE_ATCPHY_MODE_USB3_DP ||
	     mode == APPLE_ATCPHY_MODE_DP))
		return j700_dp_route(atcphy, mode == APPLE_ATCPHY_MODE_DP);

	core_mask32(atcphy, atcphy->hw->aciophy_lane_mode, ACIOPHY_LANE_MODE_RX0,
		    FIELD_PREP(ACIOPHY_LANE_MODE_RX0, mode_cfg->lane_mode[0]));
	core_mask32(atcphy, atcphy->hw->aciophy_lane_mode, ACIOPHY_LANE_MODE_TX0,
		    FIELD_PREP(ACIOPHY_LANE_MODE_TX0, mode_cfg->lane_mode[0]));
	core_mask32(atcphy, atcphy->hw->aciophy_lane_mode, ACIOPHY_LANE_MODE_RX1,
		    FIELD_PREP(ACIOPHY_LANE_MODE_RX1, mode_cfg->lane_mode[1]));
	core_mask32(atcphy, atcphy->hw->aciophy_lane_mode, ACIOPHY_LANE_MODE_TX1,
		    FIELD_PREP(ACIOPHY_LANE_MODE_TX1, mode_cfg->lane_mode[1]));
	core_mask32(atcphy, atcphy->hw->aciophy_crossbar, ACIOPHY_CROSSBAR_PROTOCOL,
		    FIELD_PREP(ACIOPHY_CROSSBAR_PROTOCOL, mode_cfg->crossbar));

	if (mode_cfg->set_swap)
		core_set32(atcphy, ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);
	else
		core_clear32(atcphy, ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);

	core_mask32(atcphy, atcphy->hw->aciophy_crossbar, ACIOPHY_CROSSBAR_DP_SINGLE_PMA,
		    FIELD_PREP(ACIOPHY_CROSSBAR_DP_SINGLE_PMA, mode_cfg->crossbar_dp_single_pma));
	if (mode_cfg->crossbar_dp_both_pma)
		core_set32(atcphy, atcphy->hw->aciophy_crossbar, ACIOPHY_CROSSBAR_DP_BOTH_PMA);
	else
		core_clear32(atcphy, atcphy->hw->aciophy_crossbar, ACIOPHY_CROSSBAR_DP_BOTH_PMA);

	/*
	 * The PMA FSM override below is for the T8103 generation. T8122 DP lanes
	 * are started by the link rate sequence, see atcphy_dp_set_rate_t8122().
	 */
	if (atcphy->hw->gen == ATCPHY_GENERATION_T8122)
		return 0;

	if (mode_cfg->dp_lane[0]) {
		core_set32(atcphy, LN0_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			   LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		udelay(10);
		core_clear32(atcphy, LN0_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ);
	} else {
		core_clear32(atcphy, LN0_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		udelay(10);
	}

	if (mode_cfg->dp_lane[1]) {
		core_set32(atcphy, LN1_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			   LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		udelay(10);
		core_clear32(atcphy, LN1_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ);
	} else {
		core_clear32(atcphy, LN1_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		udelay(10);
	}
	return 0;
}

#define J700_AUX_ANALOG_CONTROL 0x16000
#define J700_AUX_SUPPLY_CONTROL 0x16400
#define J700_AUX_ANALOG_OFF BIT(0)
#define J700_AUX_CLAMP_SELECT BIT(8)
#define J700_AUX_CLAMP_ENABLE BIT(9)

static int j700_aux_enable(struct apple_atcphy *atcphy)
{
	u32 clamp = J700_AUX_CLAMP_SELECT | J700_AUX_CLAMP_ENABLE;
	int ret;

	/* Even an incomplete power-up must be unwound before the domain is stopped. */
	atcphy->aux_power_requested = true;
	for (unsigned int stage = 0; stage < 2; stage++) {
		u32 fields = 3U << (2 * stage);

		ret = j700_field_update(atcphy, J700_AUX_SUPPLY_CONTROL, fields, fields);
		if (ret)
			return ret;
	}
	ret = j700_field_update(atcphy, J700_AUX_SUPPLY_CONTROL, clamp, J700_AUX_CLAMP_SELECT);
	if (ret)
		return ret;

	return j700_field_update(atcphy, J700_AUX_ANALOG_CONTROL, J700_AUX_ANALOG_OFF, 0);
}

static int j700_aux_disable(struct apple_atcphy *atcphy)
{
	u32 clamp = J700_AUX_CLAMP_SELECT | J700_AUX_CLAMP_ENABLE;
	int ret;

	if (!atcphy->aux_power_requested)
		return 0;
	ret = j700_field_update(atcphy, J700_AUX_ANALOG_CONTROL,
				J700_AUX_ANALOG_OFF, J700_AUX_ANALOG_OFF);
	if (ret)
		return ret;
	ret = j700_field_update(atcphy, J700_AUX_SUPPLY_CONTROL, clamp, clamp);
	if (ret)
		return ret;
	for (unsigned int remaining = 2; remaining; remaining--) {
		u32 override = BIT(2 * remaining - 1);

		ret = j700_field_update(atcphy, J700_AUX_SUPPLY_CONTROL, override, 0);
		if (ret)
			return ret;
	}
	atcphy->aux_power_requested = false;
	return 0;
}

static int atcphy_enable_dp_aux(struct apple_atcphy *atcphy)
{
	if (atcphy->hw->has_t8140_dp)
		return j700_aux_enable(atcphy);

	/* T8122 PHYs with DP support enable AUX in atcphy_dp_aux_on_t8122() */
	if (atcphy->hw->gen == ATCPHY_GENERATION_T8122)
		return 0;

	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N);
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N_OV);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPRX_PCLK_SELECT,
		    FIELD_PREP(DPRX_PCLK_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPRX_PCLK_ENABLE);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK1_SELECT,
		    FIELD_PREP(DPTX_PCLK1_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK1_ENABLE);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK2_SELECT,
		    FIELD_PREP(DPTX_PCLK2_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK2_ENABLE);

	core_set32(atcphy, ACIOPHY_PLL_COMMON_CTRL,
		   ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT);

	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_SML_IN);
	udelay(10);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_BIG_IN);
	udelay(10);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_PWN_DOWN);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_TXTERM_CODEMSB);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_TXTERM_CODE,
	       FIELD_PREP(LPDPTX_TXTERM_CODE, 0x16));

	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL, 0x1c00);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1, LPDPTX_CFG_PMA_PHYS_ADJ,
	       FIELD_PREP(LPDPTX_CFG_PMA_PHYS_ADJ, 5));
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1,
	      LPDPTX_CFG_PMA_PHYS_ADJ_OV);

	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_MARGIN,
		LPDPTX_MARGIN_RCAL_RXOFFSET_EN);

	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL, LPDPTX_BLK_AUX_CTRL_PWRDN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0,
	      LPDPTX_CFG_PMA_AUX_SEL_LF_DATA);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL, LPDPTX_BLK_AUX_RXOFFSET,
	       FIELD_PREP(LPDPTX_BLK_AUX_RXOFFSET, 3));

	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_MARGIN, LPDPTX_AUX_MARGIN_RCAL_TXSWING,
	       FIELD_PREP(LPDPTX_AUX_MARGIN_RCAL_TXSWING, 12));

	atcphy->dp_link_rate = -1;
	return 0;
}

static int atcphy_disable_dp_aux(struct apple_atcphy *atcphy)
{
	if (atcphy->hw->has_t8140_dp) {
		int ret = j700_dp_stop(atcphy);

		return ret ? ret : j700_aux_disable(atcphy);
	}

	/* T8122 PHYs with DP support stop AUX in atcphy_dp_stop_t8122() */
	if (atcphy->hw->gen != ATCPHY_GENERATION_T8122) {
		set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_PWN_DOWN);
		set32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL, LPDPTX_BLK_AUX_CTRL_PWRDN);
		set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
		clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_SML_IN);
		udelay(10);
		clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_BIG_IN);
		udelay(10);
	}
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPRX_PCLK_ENABLE);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK1_ENABLE);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK2_ENABLE);
	return 0;
}

static int atcphy_dp_configure_lane(struct apple_atcphy *atcphy, enum atcphy_lane lane,
				    const struct atcphy_dp_link_rate_configuration *cfg)
{
	void __iomem *tx_shm, *rx_shm, *rx_top;
	unsigned int tx_cal_code;

	lockdep_assert_held(&atcphy->lock);

	switch (lane) {
	case APPLE_ATCPHY_LANE_0:
		tx_shm = atcphy->regs.core + LN0_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN0_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN0_AUSPMA_RX_TOP;
		break;
	case APPLE_ATCPHY_LANE_1:
		tx_shm = atcphy->regs.core + LN1_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN1_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN1_AUSPMA_RX_TOP;
		break;
	default:
		return -EINVAL;
	}

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_SML);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_SML_OV);
	udelay(10);

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_BIG);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_BIG_OV);
	udelay(10);

	if (cfg->txa_ldoclk_bypass) {
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_SML);
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_SML_OV);
		udelay(10);

		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_BIG);
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_BIG_OV);
		udelay(10);
	} else {
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_SML);
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_SML_OV);
		udelay(10);

		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_BIG);
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_BYPASS_BIG_OV);
		udelay(10);
	}

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_SEL_OV);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_EN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_CLR);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_CLR_OV);

	if (cfg->txa_div2_en)
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_EN);
	else
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_EN_OV);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_CLK_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_CLK_EN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_RESET);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_RESET_OV);

	mask32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_BASE,
	       FIELD_PREP(LN_TXA_CAL_CTRL_BASE, 0xf));
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_BASE_OV);

	tx_cal_code = FIELD_GET(AUS_UNK_A20_TX_CAL_CODE, readl(atcphy->regs.core + AUS_UNK_A20));
	mask32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL,
	       FIELD_PREP(LN_TXA_CAL_CTRL, (1 << tx_cal_code) - 1));
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_2R_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_2R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_4R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_4R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_2R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_4R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_4R_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_HIZ);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_HIZ_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1, LN_RX_DIV20_RESET_N);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1, LN_RX_DIV20_RESET_N_OV);
	udelay(10);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1, LN_RX_DIV20_RESET_N);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_EN_OV);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_TX_CAL_CODE,
	       FIELD_PREP(LN_TX_CAL_CODE, tx_cal_code));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_TX_CAL_CODE_OV);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DLY_CTRL_TAPGEN,
	       FIELD_PREP(LN_TX_CLK_DLY_CTRL_TAPGEN, 3));

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10, LN_DTVREG_ADJUST);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_DTVREG_ADJUST_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_TEST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_TEST_EN_OV);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_TEST_RXLPBKDT_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_TEST_RXLPBKDT_EN_OV);
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_LPBKIN_DATA,
	       FIELD_PREP(LN_VREF_LPBKIN_DATA, 3));
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BIAS_SEL,
	       FIELD_PREP(LN_VREF_BIAS_SEL, 2));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BIAS_SEL_OV);
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_ADJUST_GRAY,
	       FIELD_PREP(LN_VREF_ADJUST_GRAY, 0x18));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_ADJUST_GRAY_OV);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_EN_OV);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN_OV);
	udelay(10);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN_OV);
	udelay(10);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PRE_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PRE_EN_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PST1_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PST1_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_PBIAS_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_PBIAS_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_PULLUP_LEAK_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_PULLUP_LEAK_EN_OV);

	set32(rx_top + LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE, LN_RX_TXMODE);

	if (cfg->txa_div2_en)
		set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DIV2_EN);
	else
		clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DIV2_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DIV2_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DIV2_RST);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_CLK_DIV2_RST_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_HRCLK_SEL);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_HRCLK_SEL_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_LSB_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1_LSB_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_LSB_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_LSB_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE_LSB_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_LSB_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_LSB_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_CODE_OV);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_SML_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_SML_EN_OV);
	udelay(10);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_BIG_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_BIG_EN_OV);
	udelay(10);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10, LN_DTVREG_ADJUST,
	       FIELD_PREP(LN_DTVREG_ADJUST, 0xa));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_DTVREG_ADJUST_OV);
	udelay(10);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_EN_OV);
	udelay(10);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0, LN_TX_CLK_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0, LN_TX_CLK_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_CLR);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_CLR_OV);

	return 0;
}

static int atcphy_auspll_apb_command(struct apple_atcphy *atcphy, u32 command)
{
	int ret;
	u32 reg;

	reg = readl(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE);
	reg &= ~AUSPLL_APB_CMD_OVERRIDE_CMD;
	reg |= FIELD_PREP(AUSPLL_APB_CMD_OVERRIDE_CMD, command);
	reg |= AUSPLL_APB_CMD_OVERRIDE_REQ;
	reg |= AUSPLL_APB_CMD_OVERRIDE_UNK28;
	writel(reg, atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE);

	ret = readl_poll_timeout(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE, reg,
				 (reg & AUSPLL_APB_CMD_OVERRIDE_ACK), 10, 10000);
	if (ret)
		dev_warn(atcphy->dev, "AUSPLL APB command was not acked\n");

	core_clear32(atcphy, AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_REQ);

	return ret;
}

static int atcphy_dp_configure(struct apple_atcphy *atcphy, enum atcphy_dp_link_rate lr)
{
	const struct atcphy_dp_link_rate_configuration *cfg;
	const struct atcphy_mode_configuration *mode_cfg;
	int ret;
	u32 reg;

	guard(mutex)(&atcphy->lock);
	/* A failed transition may have asserted the PHY resets already. */
	if (atcphy->mode_dirty || atcphy->pipe_pending)
		return -EBUSY;
	/* the AUSPLL is in use as the Thunderbolt DP tunnel pixel clock */
	if (atcphy->tunnel_clock_on || atcphy->tunnel_saved)
		return -EBUSY;
	mode_cfg = atcphy_get_mode_config(atcphy, atcphy->mode);
	cfg = &dp_lr_config[lr];

	if (atcphy->dp_link_rate == lr)
		return 0;

	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_CMN_SHM_STS_REG0, reg,
				 (reg & ACIOPHY_CMN_SHM_STS_REG0_CMD_READY), 10, 10000);
	if (ret) {
		dev_err(atcphy->dev, "ACIOPHY_CMN_SHM_STS_REG0_CMD_READY not set.\n");
		return ret;
	}

	/* The old rate no longer describes hardware once reprogramming starts. */
	atcphy->dp_link_rate = -1;
	core_clear32(atcphy, AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);

	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_FREQ_COUNT_TARGET,
		    FIELD_PREP(AUSPLL_FD_FREQ_COUNT_TARGET, cfg->freqinit_count_target));
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_FBDIVN_HALF);
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_REV_DIVN);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KI_MAN, FIELD_PREP(AUSPLL_FD_KI_MAN, 8));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KI_EXP, FIELD_PREP(AUSPLL_FD_KI_EXP, 3));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KP_MAN, FIELD_PREP(AUSPLL_FD_KP_MAN, 8));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KP_EXP, FIELD_PREP(AUSPLL_FD_KP_EXP, 7));
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KPKI_SCALE_HBW);

	core_mask32(atcphy, AUSPLL_FREQ_DESC_B, AUSPLL_FD_FBDIVN_FRAC_DEN,
		    FIELD_PREP(AUSPLL_FD_FBDIVN_FRAC_DEN, cfg->fbdivn_frac_den));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_B, AUSPLL_FD_FBDIVN_FRAC_NUM,
		    FIELD_PREP(AUSPLL_FD_FBDIVN_FRAC_NUM, cfg->fbdivn_frac_num));

	core_clear32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_SDM_SSC_STEP);
	core_clear32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_SDM_SSC_EN);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_PCLK_DIV_SEL,
		    FIELD_PREP(AUSPLL_FD_PCLK_DIV_SEL, cfg->pclk_div_sel));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_LFSDM_DIV,
		    FIELD_PREP(AUSPLL_FD_LFSDM_DIV, 1));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_LFCLK_CTRL,
		    FIELD_PREP(AUSPLL_FD_LFCLK_CTRL, cfg->lfclk_ctrl));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_VCLK_OP_DIVN,
		    FIELD_PREP(AUSPLL_FD_VCLK_OP_DIVN, cfg->vclk_op_divn));
	core_set32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_VCLK_PRE_DIVN);

	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
		    FIELD_PREP(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 7));

	if (cfg->plla_clkout_vreg_bypass)
		core_set32(atcphy, AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);
	else
		core_clear32(atcphy, AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);

	core_set32(atcphy, AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);

	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN);

	ret = atcphy_auspll_apb_command(atcphy, 0);
	if (ret)
		return ret;

	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT, reg,
				 (reg & ACIOPHY_AUSPLL_LOCK), 10, 10000);
	if (ret) {
		dev_err(atcphy->dev, "ACIOPHY_DP_PCLK did not lock.\n");
		return ret;
	}

	ret = atcphy_auspll_apb_command(atcphy, 0x2800);
	if (ret)
		return ret;

	if (mode_cfg->dp_lane[0]) {
		ret = atcphy_dp_configure_lane(atcphy, APPLE_ATCPHY_LANE_0, cfg);
		if (ret)
			return ret;
	}

	if (mode_cfg->dp_lane[1]) {
		ret = atcphy_dp_configure_lane(atcphy, APPLE_ATCPHY_LANE_1, cfg);
		if (ret)
			return ret;
	}

	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DP_PMA_BYTECLK_RESET);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DP_MAC_DIV20_CLK_SEL);

	atcphy->dp_link_rate = lr;
	return 0;
}

static void atcphy_usb2_power_off(struct apple_atcphy *atcphy)
{
	/* Disable the PHY, this clears USB2PHY_USBCTL_RUN */
	mask32(atcphy->regs.usb2phy + USB2PHY_USBCTL, USB2PHY_USBCTL_USB_MODE, USB2PHY_USBCTL_ISOLATION);
	udelay(10);

	/* Switch the PHY to low power mode */
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* Enable all resets */
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
	udelay(10);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_RESET);
	udelay(10);
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	udelay(10);
	set32(atcphy->regs.usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
	set32(atcphy->regs.usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);
}

static int atcphy_power_off(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	ret = atcphy_disable_dp_aux(atcphy);
	if (ret)
		return ret;

	/* Enable all reset lines */
	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	core_clear32(atcphy, ATCPHY_MISC, ATCPHY_MISC_RESET_N | ATCPHY_MISC_LANE_SWAP);
	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 !(reg & ATCPHY_POWER_SLEEP_BIG), 10, 1000);
	if (ret) {
		dev_err(atcphy->dev, "Failed to sleep atcphy \"big\"\n");
		return ret;
	}

	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 !(reg & ATCPHY_POWER_SLEEP_SMALL), 10, 1000);
	if (ret) {
		dev_err(atcphy->dev, "Failed to sleep atcphy \"small\"\n");
		return ret;
	}

	return 0;
}

static void atcphy_usb2_power_on(struct apple_atcphy *atcphy)
{
	set32(atcphy->regs.usb2phy + USB2PHY_SIG,
	      USB2PHY_SIG_VBUSDET_FORCE_VAL | USB2PHY_SIG_VBUSDET_FORCE_EN |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL | USB2PHY_SIG_VBUSVLDEXT_FORCE_EN);
	udelay(10);

	/* Take the PHY out of its low power state */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* Release reset */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_RESET);
	udelay(10);
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
	udelay(10);

	if (atcphy->hw->gen == ATCPHY_GENERATION_T8122) {
		core_set32(atcphy, ATCPHY_EVT_USB2_CTL, ATCPHY_EVT_USB2_CTL_LOAD_CNT |
			ATCPHY_EVT_USB2_CTL_EVT_EN);
		udelay(10);
	}

	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	udelay(10);
	clear32(atcphy->regs.usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
	clear32(atcphy->regs.usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);

	/*
	 * Give the eUSB2 repeater behind the T8140 PHY time to settle between the
	 * reset release and the start of the link. The 5 ms interval is the one
	 * the T8140 bring-up used; it has not been narrowed down.
	 */
	if (atcphy->hw->has_usb2phy_reg)
		fsleep(5000);

	/* Enable the PHY */
	writel(USB2PHY_USBCTL_RUN, atcphy->regs.usb2phy + USB2PHY_USBCTL);

	/*
	 * The T8140 keeps per-device defaults for its secondary eUSB2 register
	 * bank in the ADT. The power-off path asserts the PHY resets, so apply
	 * them after every power-on.
	 */
	apple_tunable_apply(atcphy->regs.usb2phy_reg, atcphy->tunables.usb2phy_reg_dflt);
}

static int atcphy_power_on(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	if (!atcphy->dp_only)
		atcphy_usb2_power_on(atcphy);

	core_set32(atcphy, ATCPHY_MISC, ATCPHY_MISC_RESET_N);

	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 reg & ATCPHY_POWER_SLEEP_SMALL, 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"small\"\n");
		return ret;
	}

	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 reg & ATCPHY_POWER_SLEEP_BIG, 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"big\"\n");
		return ret;
	}

	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

	return 0;
}

static void atcphy_configure_post_tunable_t8103(struct apple_atcphy *atcphy, enum atcphy_mode _mode)
{
	core_set32(atcphy, AUSPLL_FSM_CTRL, 0x1fe000);
	core_set32(atcphy, AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_UNK28);

	set32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
	udelay(10);
	set32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
	udelay(10);
	set32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
	udelay(10);

	mask32(atcphy->regs.core + ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL_OV,
	       FIELD_PREP(ACIOPHY_SLEEP_CTRL_TX_SMALL_OV, 3));
	udelay(10);
	mask32(atcphy->regs.core + ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG_OV,
	       FIELD_PREP(ACIOPHY_SLEEP_CTRL_TX_BIG_OV, 3));
	udelay(10);
	mask32(atcphy->regs.core + ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV,
	       FIELD_PREP(ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV, 3));
	udelay(10);

	mask32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG_OV,
	       FIELD_PREP(ACIOPHY_CFG0_RX_BIG_OV, 3));
	udelay(10);
	mask32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL_OV,
	       FIELD_PREP(ACIOPHY_CFG0_RX_SMALL_OV, 3));
	udelay(10);
	mask32(atcphy->regs.core + ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP_OV,
	       FIELD_PREP(ACIOPHY_CFG0_RX_CLAMP_OV, 3));
	udelay(10);
}

static void atcphy_wake_common_t8122(struct apple_atcphy *atcphy)
{
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL);
	udelay(10);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
	udelay(10);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG);
	udelay(10);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
	udelay(10);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP);
	udelay(10);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
	udelay(10);
	core_set32(atcphy, AUS_COMMON_SHIM_BLK_BIAS_REG, AUS_COMMON_SHIM_BLK_BIAS_REG_BGBIAS_OV);
	udelay(10);
}

static void atcphy_configure_post_tunable_t8122(struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	if (atcphy_modes[mode].pipehandler_state != ATCPHY_PIPEHANDLER_STATE_USB3 &&
	    !(atcphy->hw->dp_t8122 && atcphy_modes[mode].enable_dp_aux))
		return;
	atcphy_wake_common_t8122(atcphy);
}

/*
 * T8140 and T8142 share the T8122 register generation. Their common blocks
 * are woken after the tunables in every mode, DisplayPort included, as these
 * PHYs were brought up and qualified.
 */
static void atcphy_configure_post_tunable_all_modes(struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	atcphy_wake_common_t8122(atcphy);
}

/*
 * DisplayPort over Thunderbolt/USB4: the host router's DP IN adapter needs a
 * pixel clock from the ATC's AUSPLL. Ordinary Type-C DP sets it up together
 * with the lanes; for a tunnel the PHY blocks are woken, DPTX PCLK1 is enabled
 * with a per-rate selector and the AUSPLL gets a fixed descriptor
 * ({0,0x21c,8,3,8,7,0,0,0,0,2,1,0,5,5,1,0,1,1,1}, APB commands 0 then 0x2000),
 * without touching the lane mux. Based on Oliver Lukschander's t6020 tunnel
 * clock patch (asahi-j416s-display).
 */
/* t600x (M1 Pro/Max) has the t8103 generation ATC. */
static bool apple_atc_tunnel_is_t600x(void)
{
	return of_machine_is_compatible("apple,t6000") ||
	       of_machine_is_compatible("apple,t6001");
}

/*
 * Stop the tunnel pixel clock: switch the AUSPLL output drivers off and send
 * APB command 3. TX_DP_CTRL0, the sleep overrides and the
 * descriptor are left as they are; the next start programs them again.
 */
static int atc_t8122_tunnel_apb(struct apple_atcphy *atcphy, u32 command,
			      bool ack)
{
	u32 value;

	core_mask32(atcphy, AUSPLL_APB_CMD_OVERRIDE, ~T8122_APB_PRESERVE, command);
	return readl_poll_timeout(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE,
				 value, !!(value & BIT(1)) == ack, 1, 10000);
}

static int atc_t8122_tunnel_stop(struct apple_atcphy *atcphy)
{
	u32 value;
	int ret, err;

	lockdep_assert_held(&atcphy->lock);
	if (!atcphy->tunnel_clock_on)
		return 0;

	/* Last/only client: gates off, reset, stop PLL, then output driver off. */
	atcphy->tunnel_rate = 0;
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, T8122_PCLK_ENABLES);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPTXPHY_PMA_LANE_RESET_N | DPTXPHY_PMA_LANE_RESET_N_OV);
	ret = atc_t8122_tunnel_apb(atcphy, 0x10000001, true);
	/* Always release REQ, including a failed start/ACK, to unwind our request. */
	err = atc_t8122_tunnel_apb(atcphy, 0x10000018, false);
	if (!ret)
		ret = err;
	err = readl_poll_timeout(atcphy->regs.core + T8122_DP_PCLK_STATUS,
				 value, !(value & ACIOPHY_AUSPLL_LOCK), 1, 10000);
	if (!ret)
		ret = err;
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, T8122_AUSPLL_PCLK_DRIVER);
	/* Keep ownership on failure: do not silently hand a live PLL to a new mode. */
	if (ret) {
		dev_err(atcphy->dev, "T8122 tunnel clock shutdown incomplete: %d\n", ret);
		return ret;
	}
	atcphy->tunnel_clock_on = false;
	atcphy->tunnel_rate = 0;
	atcphy->tunnel_users = 0;
	atcphy->tunnel_dpin_rate[0] = 0;
	atcphy->tunnel_dpin_rate[1] = 0;
	return 0;
}

static void atc_t8122_tunnel_pclk(struct apple_atcphy *atcphy,
				 unsigned int pclk, u32 selector)
{
	u32 enable = pclk ? DPTX_PCLK2_ENABLE : DPTX_PCLK1_ENABLE;
	u32 select = pclk ? DPTX_PCLK2_SELECT : DPTX_PCLK1_SELECT;
	u32 value = pclk ? FIELD_PREP(DPTX_PCLK2_SELECT, selector) :
			   FIELD_PREP(DPTX_PCLK1_SELECT, selector);

	lockdep_assert_held(&atcphy->lock);
	/* PCLK1 is also DPIN1's prerequisite: retuning DPIN0 must not gate it. */
	if (!pclk && (atcphy->tunnel_users & BIT(1)))
		value |= DPTX_PCLK1_ENABLE;
	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    enable | select, value);
	/* Enable the prerequisite before the second stream's pixel clock. */
	if (pclk) {
		if (!(atcphy->tunnel_users & BIT(0)))
			core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
				    DPTX_PCLK1_SELECT,
				    FIELD_PREP(DPTX_PCLK1_SELECT, selector));
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			   DPTX_PCLK1_ENABLE);
	}
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, enable);
}

static int atc_t8122_tunnel_start(struct apple_atcphy *atcphy,
				unsigned int pclk, u8 rate)
{
	u32 selector, value, saved_tx;
	int ret, cleanup;

	lockdep_assert_held(&atcphy->lock);
	switch (rate) {
	case 0x06:
		selector = 4;
		break;
	case 0x0a:
		selector = 3;
		break;
	case 0x14:
		selector = 1;
		break;
	case 0x1e:
		selector = 0;
		break;
	default:
		return -EINVAL;
	}
	if (atcphy->tunnel_clock_on) {
		/* A failed shutdown retains ownership, but is not a running clock. */
		if (!atcphy->tunnel_rate)
			return -EBUSY;
		atc_t8122_tunnel_pclk(atcphy, pclk, selector);
		atcphy->tunnel_rate = rate;
		return 0;
	}
	saved_tx = readl(atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0);
	if ((readl(atcphy->regs.core + AUSPLL_CLKOUT_MASTER) & T8122_AUSPLL_PCLK_DRIVER) ||
	    (readl(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE) & (BIT(0) | BIT(1))) ||
	    (readl(atcphy->regs.core + T8122_DP_PCLK_STATUS) & ACIOPHY_AUSPLL_LOCK) ||
	    ((saved_tx & T8122_PCLK_ENABLES) && (saved_tx & 0xffff) != 0xe001))
		return -EBUSY;

	/* Inactive slots use selector zero; all gates stay off during PLL setup. */
	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, GENMASK(15, 4),
		    FIELD_PREP(DPTX_PCLK1_SELECT, selector) |
		    (pclk ? FIELD_PREP(DPTX_PCLK2_SELECT, selector) : 0));
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
	udelay(1);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
	udelay(1);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
	udelay(1);
	core_set32(atcphy, AUS_COMMON_SHIM_BLK_BIAS_REG, AUS_COMMON_SHIM_BLK_BIAS_REG_BGBIAS_OV);
	udelay(15);
	ret = readl_poll_timeout(atcphy->regs.core + AUS_COMMON_DIG_RCAL1, value,
				 value & AUS_COMMON_DIG_RCAL1_ALL_CODES_DONE, 1, 10000);
	if (ret) {
		writel(saved_tx, atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0);
		return ret;
	}

	atcphy->tunnel_clock_on = true;
	atcphy->tunnel_rate = 0;
	atcphy->dp_link_rate = -1;
	core_clear32(atcphy, T8122_AUSPLL_FREQ_CFG, GENMASK(1, 0));
	writel(0x1e0e021c, atcphy->regs.core + AUSPLL_FREQ_DESC_A);
	core_clear32(atcphy, AUSPLL_FREQ_DESC_B, GENMASK(27, 0));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, GENMASK(22, 0), 0x644a00);
	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, GENMASK(20, 16), BIT(16));
	core_set32(atcphy, T8122_AUSPLL_BGR, BIT(0));
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER,
		   T8122_AUSPLL_PCLK_DRIVER | T8122_AUSPLL_REFBUF_DRIVER);
	ret = atc_t8122_tunnel_apb(atcphy, 0x10000001, true);
	if (ret)
		goto fail;
	ret = readl_poll_timeout(atcphy->regs.core + T8122_DP_PCLK_STATUS, value,
				 value & ACIOPHY_AUSPLL_LOCK, 1, 10000);
	if (ret)
		goto fail;
	ret = atc_t8122_tunnel_apb(atcphy, 0x10010000, false);
	if (ret)
		goto fail;
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTXPHY_PMA_LANE_RESET_N | DPTXPHY_PMA_LANE_RESET_N_OV);
	atc_t8122_tunnel_pclk(atcphy, pclk, selector);
	atcphy->tunnel_rate = rate;
	return 0;

fail:
	dev_err(atcphy->dev, "T8122 tunnel clock start failed: %d\n", ret);
	cleanup = atc_t8122_tunnel_stop(atcphy);
	if (!cleanup)
		writel(saved_tx, atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0);
	return ret;
}

static void atc_tunnel_stop_t8103(struct apple_atcphy *atcphy)
{
	lockdep_assert_held(&atcphy->lock);
	if (!atcphy->tunnel_clock_on)
		return;
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_DRVR_EN);
	atcphy_auspll_apb_command(atcphy, 3);
	atcphy->tunnel_clock_on = false;
	atcphy->tunnel_rate = 0;
	atcphy->tunnel_users = 0;
	dev_dbg(atcphy->dev, "DP tunnel clock stopped\n");
}

/*
 * TX_DP_CTRL0: PMA lane reset release, pixel clock enable + rate selector.
 * t8103 feeds the tunnel from PCLK1 alone; t600x also runs PCLK2 and the DP IN
 * (DPRX) clock at the same rate.
 */
static void atc_tunnel_pclk_t8103(struct apple_atcphy *atcphy, u32 selector)
{
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N);
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N_OV);
	if (apple_atc_tunnel_is_t600x()) {
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPRX_PCLK_ENABLE);
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK2_ENABLE);
		core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPRX_PCLK_SELECT,
			    FIELD_PREP(DPRX_PCLK_SELECT, selector));
		core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK2_SELECT,
			    FIELD_PREP(DPTX_PCLK2_SELECT, selector));
	}
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK1_ENABLE);
	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DPTX_PCLK1_SELECT,
		    FIELD_PREP(DPTX_PCLK1_SELECT, selector));
}

static void atc_tunnel_wake_t8103(struct apple_atcphy *atcphy)
{
	/*
	 * The PHY is up in USB4/TBT mode here. These overrides only force the
	 * already awake blocks to stay awake for the DP clock path; the order
	 * and the 2 us gaps are the ones this was validated with, the USB4
	 * link stays up across them.
	 *
	 * Common block: override small/big sleep off and release the clamp.
	 */
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
	udelay(2);
	/* same for the TX lanes */
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV);
	udelay(2);
	/* and for the RX lanes */
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP_OV);
	udelay(2);
}

/* The pixel clock selector for a DP link rate code. */
static int atc_tunnel_rate_selector(u8 rate)
{
	switch (rate) {
	case 0x06:	/* RBR */
		return 4;
	case 0x0a:	/* HBR */
		return 3;
	case 0x14:	/* HBR2 */
		return 1;
	case 0x1e:	/* HBR3 */
		return 0;
	default:
		return -EINVAL;
	}
}

static int atc_tunnel_start_t8103(struct apple_atcphy *atcphy, u8 rate)
{
	int selector, ret;
	u32 value;

	lockdep_assert_held(&atcphy->lock);
	selector = atc_tunnel_rate_selector(rate);
	if (selector < 0)
		return selector;
	if (atcphy->tunnel_clock_on) {
		if (atcphy->tunnel_rate == rate)
			return 0;
		/*
		 * Rate change: DCP brackets it with WillChange/DidChange link
		 * configuration, so the crossbar is already down; reselect the clocks.
		 */
		atc_tunnel_pclk_t8103(atcphy, selector);
		atcphy->tunnel_rate = rate;
		return 0;
	}
	/*
	 * Don't take the PLL from another clock client or an in-flight command.
	 * Only live PLL outputs or a pending request mean that: the PCLK gates
	 * in TX_DP_CTRL0 read set when idle and AUSPLL_LOCK stays set after a
	 * stop, so neither says anything about another client.
	 */
	if (readl(atcphy->regs.core + AUSPLL_CLKOUT_MASTER) & AUSPLL_CLKOUT_MASTER_DRVR_EN ||
	    readl(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE) & AUSPLL_APB_CMD_OVERRIDE_REQ)
		return -EBUSY;
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_CMN_SHM_STS_REG0, value,
				 value & ACIOPHY_CMN_SHM_STS_REG0_CMD_READY, 10, 10000);
	if (ret)
		return ret;
	atcphy->tunnel_clock_on = true;
	/* the PLL no longer holds a DP alt mode configuration */
	atcphy->dp_link_rate = -1;

	atc_tunnel_wake_t8103(atcphy);
	atc_tunnel_pclk_t8103(atcphy, selector);

	/* fixed AUSPLL descriptor */
	core_clear32(atcphy, AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, U32_MAX,
		    FIELD_PREP(AUSPLL_FD_FREQ_COUNT_TARGET, 0x21c) |
		    FIELD_PREP(AUSPLL_FD_KI_MAN, 8) | FIELD_PREP(AUSPLL_FD_KI_EXP, 3) |
		    FIELD_PREP(AUSPLL_FD_KP_MAN, 8) | FIELD_PREP(AUSPLL_FD_KP_EXP, 7));
	core_clear32(atcphy, AUSPLL_FREQ_DESC_B,
		     AUSPLL_FD_FBDIVN_FRAC_DEN | AUSPLL_FD_FBDIVN_FRAC_NUM);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C,
		    AUSPLL_FD_SDM_SSC_STEP | AUSPLL_FD_SDM_SSC_EN | AUSPLL_FD_PCLK_DIV_SEL |
		    AUSPLL_FD_LFSDM_DIV | AUSPLL_FD_LFCLK_CTRL | AUSPLL_FD_VCLK_OP_DIVN |
		    AUSPLL_FD_VCLK_PRE_DIVN,
		    FIELD_PREP(AUSPLL_FD_PCLK_DIV_SEL, 5) | FIELD_PREP(AUSPLL_FD_LFSDM_DIV, 1) |
		    FIELD_PREP(AUSPLL_FD_LFCLK_CTRL, 5) | FIELD_PREP(AUSPLL_FD_VCLK_OP_DIVN, 2) |
		    AUSPLL_FD_VCLK_PRE_DIVN);
	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
		    FIELD_PREP(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 1));
	core_set32(atcphy, AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);
	core_set32(atcphy, AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN);
	ret = atcphy_auspll_apb_command(atcphy, 0);
	if (ret)
		goto err_stop;
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT, value,
				 value & ACIOPHY_AUSPLL_LOCK, 10, 10000);
	if (ret) {
		dev_err(atcphy->dev, "DP tunnel clock: AUSPLL did not lock\n");
		goto err_stop;
	}
	ret = atcphy_auspll_apb_command(atcphy, 0x2000);
	if (ret)
		goto err_stop;
	atcphy->tunnel_rate = rate;
	return 0;

err_stop:
	atc_tunnel_stop_t8103(atcphy);
	return ret;
}

struct atc_tunnel_saved_reg {
	u32 reg;
	u32 mask;
};

static const struct atc_tunnel_saved_reg atc_tunnel_regs[] = {
	{ ACIOPHY_CFG0, 0x0003ffff },
	{ ACIOPHY_SLEEP_CTRL, 0x00000fff },
	{ ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
	  0x0000207c | DPTX_PCLK2_SELECT | DPTX_PCLK2_ENABLE },
	{ AUSPLL_FREQ_CFG, 0x00000003 },
	{ AUSPLL_FREQ_DESC_A, 0xffffffff },
	{ AUSPLL_FREQ_DESC_B, 0x0fffffff },
	{ AUSPLL_FREQ_DESC_C, 0x007fffff },
	{ AUSPLL_CLKOUT_DIV, 0x001f0000 },
	{ AUSPLL_CLKOUT_DTC_VREG, 0x00000080 },
	{ AUSPLL_BGR, 0x00000001 },
	{ AUSPLL_CLKOUT_MASTER, 0x00000054 },
	{ AUSPLL_APB_CMD_OVERRIDE, 0x1ffffff9 },
};

static int atc_tunnel_command(struct apple_atcphy *atcphy, u32 command)
{
	u32 value;
	int ret;

	core_mask32(atcphy, AUSPLL_APB_CMD_OVERRIDE,
		    AUSPLL_APB_CMD_OVERRIDE_CMD,
		    FIELD_PREP(AUSPLL_APB_CMD_OVERRIDE_CMD, command) |
		    AUSPLL_APB_CMD_OVERRIDE_REQ | AUSPLL_APB_CMD_OVERRIDE_UNK28);
	ret = readl_poll_timeout(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE,
				 value, value & AUSPLL_APB_CMD_OVERRIDE_ACK, 10, 10000);
	core_clear32(atcphy, AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_REQ);
	return ret;
}

static void atc_tunnel_restore(struct apple_atcphy *atcphy)
{
	u32 value;
	int i, ret;

	lockdep_assert_held(&atcphy->lock);
	atcphy->tunnel_users = 0;
	atcphy->tunnel_dpin_rate[0] = 0;
	atcphy->tunnel_dpin_rate[1] = 0;
	if (!atcphy->tunnel_saved)
		return;
	/*
	 * Clearing the PLL output drivers alone leaves the AUSPLL running,
	 * with ACIOPHY_AUSPLL_LOCK still set. Clear the three drivers, then
	 * power the PLL down with APB command 3, after which the lock drops.
	 */
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPTX_PCLK1_ENABLE | DPTX_PCLK2_ENABLE);
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN);
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN);
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN);
	ret = atc_tunnel_command(atcphy, 3);
	if (ret)
		dev_warn(atcphy->dev, "USB4 tunnel clock: AUSPLL power-down was not acked\n");
	/* Then restore the unused PLL descriptor. */
	for (i = ARRAY_SIZE(atc_tunnel_regs) - 1; i >= 0; i--)
		core_mask32(atcphy, atc_tunnel_regs[i].reg, atc_tunnel_regs[i].mask,
			    atcphy->tunnel_saved_regs[i] & atc_tunnel_regs[i].mask);
	/*
	 * atc_tunnel_start_t602x()'s preflight refuses to proceed while
	 * ACIOPHY_AUSPLL_LOCK is set, to avoid clobbering a PLL a concurrent
	 * user actually has locked. DCP firmware retries a failed tunnel
	 * clock request quickly, so confirm the lock has dropped after the
	 * power-down before a new request can come in.
	 */
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT, value,
				 !(value & ACIOPHY_AUSPLL_LOCK), 10, 10000);
	if (ret)
		dev_warn(atcphy->dev,
			 "USB4 tunnel clock: AUSPLL_LOCK did not clear after teardown (stat=%#x)\n",
			 value);
	atcphy->tunnel_saved = false;
	atcphy->tunnel_attempted = false;
	atcphy->tunnel_rate = 0;
}

/*
 * Bring the T602X tunnel PLL up with its fixed descriptor. The rate is not
 * part of it: each DP IN adapter selects its own rate on PCLK1 or PCLK2 in
 * apple_atc_dp_tunnel_rate().
 */
static int atc_tunnel_start_t602x(struct apple_atcphy *atcphy)
{
	u32 value, gates, outputs, command, status;
	unsigned int i;
	int ret;

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->tunnel_saved)
		return 0;
	if (atcphy->tunnel_attempted)
		return -EALREADY;
	/* Read every guard before deciding, so a refusal records the whole state. */
	gates = readl(atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0);
	outputs = readl(atcphy->regs.core + AUSPLL_CLKOUT_MASTER);
	command = readl(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE);
	status = readl(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT);
	dev_dbg(atcphy->dev,
		"USB4 tunnel clock preflight: +7000=%08x +2200=%08x +2000=%08x +7044=%08x\n",
		gates, outputs, command, status);
	/*
	 * Before the first rate request, TX_DP_CTRL0 gate bits read 0xe001:
	 * the enable gates are set, the byte-clock reset is asserted, and the
	 * selector and reset-release bits are clear. Accept this initial gate
	 * state only with no enabled PLL output, no outstanding command and
	 * no lock. Any other active gate state is refused.
	 */
	if (outputs & AUSPLL_CLKOUT_MASTER_DRVR_EN || command & AUSPLL_APB_CMD_OVERRIDE_REQ ||
	    status & ACIOPHY_AUSPLL_LOCK)
		return -EBUSY;
	if ((gates & (DPTX_PCLK1_ENABLE | DPTX_PCLK2_ENABLE | DPRX_PCLK_ENABLE)) &&
	    gates != 0x0000e001)
		return -EBUSY;
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_CMN_SHM_STS_REG0,
				 value, value & ACIOPHY_CMN_SHM_STS_REG0_CMD_READY,
				 10, 10000);
	if (ret)
		return ret;
	atcphy->tunnel_attempted = true;
	for (i = 0; i < ARRAY_SIZE(atc_tunnel_regs); i++)
		atcphy->tunnel_saved_regs[i] = readl(atcphy->regs.core + atc_tunnel_regs[i].reg);
	atcphy->tunnel_saved = true;

	/* Wake the common clock blocks without changing the lane mux. */
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
	udelay(2);
	/* same for the TX lanes */
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP);
	core_set32(atcphy, ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV);
	udelay(2);
	/* and for the RX lanes */
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL_OV);
	udelay(2);
	core_clear32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP);
	core_set32(atcphy, ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP_OV);
	udelay(2);
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, BIT(2));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, BIT(3));

	/* Program the fixed tunnel clock descriptor. */
	core_clear32(atcphy, AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, 0xffffffff,
		    0x21c | (8 << 14) | (3 << 18) | (8 << 22) | (7 << 26));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_B, 0x0fffffff, 0);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, 0x007fffff,
		    (5 << 9) | (1 << 14) | (5 << 16) | (2 << 20) | BIT(22));
	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
		    FIELD_PREP(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 1));
	core_set32(atcphy, AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);
	core_set32(atcphy, AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, BIT(2));
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, BIT(4));
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, BIT(6));
	ret = atc_tunnel_command(atcphy, 0);
	if (ret)
		goto restore;
	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT,
				 value, value & ACIOPHY_AUSPLL_LOCK, 10, 10000);
	if (ret)
		goto restore;
	ret = atc_tunnel_command(atcphy, 0x2000);
	if (ret)
		goto restore;
	return 0;
restore:
	atc_tunnel_restore(atcphy);
	return ret;
}

/* Stop one T602X DP IN adapter's pixel clock; atcphy->lock held. */
static void atc_tunnel_stop_t602x(struct apple_atcphy *atcphy, unsigned int dpin)
{
	u8 user = BIT(dpin);

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->tunnel_users & user) {
		if (dpin == 0) {
			/*
			 * DPIN1 still needs the PCLK1 gate. On J414s, clearing
			 * it stopped DPIN1's video packets even though PCLK2
			 * remained enabled and locked.
			 */
			if (!(atcphy->tunnel_users & BIT(1)))
				core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
					     DPTX_PCLK1_ENABLE);
		} else {
			core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
				     DPTX_PCLK2_ENABLE);
			if (!(atcphy->tunnel_users & BIT(0)))
				core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
					     DPTX_PCLK1_ENABLE);
		}
	}
	atcphy->tunnel_users &= ~user;
	atcphy->tunnel_dpin_rate[dpin] = 0;
	/*
	 * Gate the PLL outputs but keep the descriptor across DPMS: the PLL
	 * keeps its lock with its outputs gated, and the next start re-enables
	 * them. A mode change still powers it down in atc_tunnel_restore().
	 */
	if (!atcphy->tunnel_users && atcphy->tunnel_saved)
		core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_DRVR_EN);
}

/* Run one T602X DP IN adapter's pixel clock at @rate; atcphy->lock held. */
static int atc_tunnel_set_t602x(struct apple_atcphy *atcphy, unsigned int dpin,
				u8 rate)
{
	int selector, ret = 0;
	u32 value;

	lockdep_assert_held(&atcphy->lock);
	selector = atc_tunnel_rate_selector(rate);
	if (selector < 0)
		return selector;
	if (atcphy->tunnel_dpin_rate[dpin] == rate)
		return 0;
	if (!atcphy->tunnel_users) {
		if (!atcphy->tunnel_saved) {
			ret = atc_tunnel_start_t602x(atcphy);
		} else {
			/*
			 * The PLL retains lock after its outputs are gated. Keep
			 * its descriptor until the USB4 PHY changes mode instead of
			 * reprogramming a still-locked PLL on every DPMS wake.
			 */
			core_set32(atcphy, AUSPLL_CLKOUT_MASTER, AUSPLL_CLKOUT_MASTER_DRVR_EN);
			ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT,
						 value, value & ACIOPHY_AUSPLL_LOCK,
						 10, 10000);
			if (ret)
				core_clear32(atcphy, AUSPLL_CLKOUT_MASTER,
					     AUSPLL_CLKOUT_MASTER_DRVR_EN);
		}
		if (ret)
			return ret;
	}
	if (dpin == 0) {
		core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			    DPTX_PCLK1_SELECT, FIELD_PREP(DPTX_PCLK1_SELECT, selector));
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			   DPTX_PCLK1_ENABLE);
	} else {
		/*
		 * PCLK2 alone does not sustain a DPIN1 stream on J414s. Keep
		 * PCLK1 gated on as a shared prerequisite; if DPIN0 is inactive,
		 * select DPIN1's rate for that clock as well.
		 */
		if (!(atcphy->tunnel_users & BIT(0)))
			core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
				    DPTX_PCLK1_SELECT,
				    FIELD_PREP(DPTX_PCLK1_SELECT, selector));
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			   DPTX_PCLK1_ENABLE);
		core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			    DPTX_PCLK2_SELECT, FIELD_PREP(DPTX_PCLK2_SELECT, selector));
		core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
			   DPTX_PCLK2_ENABLE);
	}
	atcphy->tunnel_dpin_rate[dpin] = rate;
	atcphy->tunnel_users |= BIT(dpin);
	return 0;
}

/*
 * T6030 (M3 Pro) and T6031 (M3 Max) use T8122-generation PHYs for both DP IN
 * clock outputs.
 */
static bool apple_atc_tunnel_is_t6030(struct apple_atcphy *atcphy)
{
	return (of_machine_is_compatible("apple,t6030") ||
		of_machine_is_compatible("apple,t6031")) &&
	       atcphy->hw->gen == ATCPHY_GENERATION_T8122;
}

/*
 * DisplayPort on the T8122 generation. The AUX block sits in the core window
 * and only comes up once the PHY reset is released and the common block has
 * calibrated, so it is enabled at the end of atcphy_configure(). The main link
 * runs from the AUSPLL: DCP sets the link rate through phy_configure(), which
 * programs the PLL and starts the transmitters of the lane pairs the current
 * mode gives to DisplayPort. The APB handshake is the one the tunnel clock
 * above uses.
 */
#define T8122_APB_CMD(cmd, req)                                     \
	(AUSPLL_APB_CMD_OVERRIDE_UNK28 |                             \
	 FIELD_PREP(AUSPLL_APB_CMD_OVERRIDE_CMD, (cmd)) |            \
	 ((req) ? AUSPLL_APB_CMD_OVERRIDE_REQ : 0))

static void atcphy_dp_aux_on_t8122(struct apple_atcphy *atcphy)
{
	void __iomem *core = atcphy->regs.core;

	lockdep_assert_held(&atcphy->lock);
	atc_t8122_dp_aux_on(core);
	atcphy->dp_t8122.aux = true;
	if (atc_t8122_dp_aux_is_on(core))
		dev_dbg(atcphy->dev, "DP AUX on (mode %d, swapped %d)\n", atcphy->mode,
			atcphy->swap_lanes);
	else
		dev_warn(atcphy->dev, "DP AUX did not power up (ctrl=%08x pwr=%08x)\n",
			 readl(core + ATC_T8122_AUX_CTRL), readl(core + ATC_T8122_AUX_PWR));
}

static int atcphy_dp_link_stop_t8122(struct apple_atcphy *atcphy)
{
	void __iomem *core = atcphy->regs.core;
	u32 value;
	int ret, err;

	lockdep_assert_held(&atcphy->lock);
	if (!atcphy->dp_t8122.pll && !atcphy->dp_t8122.pairs)
		return 0;

	atcphy->dp_t8122.rate = 0;
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DP_PMA_BYTECLK_RESET);
	for (unsigned int pair = 0; pair < 2; pair++)
		if (atcphy->dp_t8122.pairs & BIT(pair))
			atc_t8122_dp_lane_stop(core, pair);
	atcphy->dp_t8122.pairs = 0;
	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    DPTXPHY_PMA_LANE_RESET_N | DPTXPHY_PMA_LANE_RESET_N_OV,
		    DPTXPHY_PMA_LANE_RESET_N_OV);

	/* Power the PLL down: command 0, then command 3 with the request released */
	ret = atc_t8122_tunnel_apb(atcphy, T8122_APB_CMD(0, true), true);
	err = atc_t8122_tunnel_apb(atcphy, T8122_APB_CMD(3, false), false);
	if (!ret)
		ret = err;
	err = readl_poll_timeout(core + T8122_DP_PCLK_STATUS, value,
				 !(value & ACIOPHY_AUSPLL_LOCK), 1, 10000);
	if (!ret)
		ret = err;
	core_clear32(atcphy, AUSPLL_CLKOUT_MASTER, T8122_AUSPLL_PCLK_DRIVER);
	atcphy->dp_t8122.pll = false;
	if (ret)
		dev_err(atcphy->dev, "DP link shutdown incomplete: %d (PCLK_STAT=%08x)\n", ret,
			readl(core + T8122_DP_PCLK_STATUS));
	else
		dev_dbg(atcphy->dev, "DP link stopped\n");
	return ret;
}

static int atcphy_dp_link_start_t8122(struct apple_atcphy *atcphy,
				      const struct atc_t8122_dp_rate *rate, u8 pairs)
{
	void __iomem *core = atcphy->regs.core;
	u32 value;
	int ret;

	lockdep_assert_held(&atcphy->lock);

	core_clear32(atcphy, T8122_AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);
	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    DPTX_PCLK1_SELECT | DPTX_PCLK2_SELECT | DPRX_PCLK_SELECT,
		    FIELD_PREP(DPTX_PCLK1_SELECT, 1) | FIELD_PREP(DPTX_PCLK2_SELECT, 1) |
		    FIELD_PREP(DPRX_PCLK_SELECT, 1));
	writel(rate->freq_desc[0], core + AUSPLL_FREQ_DESC_A);
	writel(rate->freq_desc[1], core + AUSPLL_FREQ_DESC_B);
	writel(rate->freq_desc[2], core + AUSPLL_FREQ_DESC_C);
	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
		    FIELD_PREP(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 7));
	core_set32(atcphy, T8122_AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER, T8122_AUSPLL_PCLK_DRIVER);
	atcphy->dp_t8122.pll = true;

	ret = atc_t8122_tunnel_apb(atcphy, T8122_APB_CMD(0, true), true);
	if (ret) {
		dev_err(atcphy->dev, "DP PLL start was not acknowledged\n");
		return ret;
	}
	ret = readl_poll_timeout(core + T8122_DP_PCLK_STATUS, value,
				 value & ACIOPHY_AUSPLL_LOCK, 1, 10000);
	if (ret) {
		dev_err(atcphy->dev, "DP PLL did not lock at %u Mb/s (PCLK_STAT=%08x)\n",
			rate->link_rate, value);
		return ret;
	}
	ret = atc_t8122_tunnel_apb(atcphy, T8122_APB_CMD(0x2800, false), false);
	if (ret) {
		dev_err(atcphy->dev, "DP PLL release was not acknowledged\n");
		return ret;
	}

	for (unsigned int pair = 0; pair < 2; pair++)
		if (pairs & BIT(pair))
			atc_t8122_dp_lane_pre_reset(core, pair);
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTXPHY_PMA_LANE_RESET_N | DPTXPHY_PMA_LANE_RESET_N_OV);
	udelay(1);

	atcphy->dp_t8122.pairs = pairs;
	for (unsigned int pair = 0; pair < 2; pair++)
		if (pairs & BIT(pair))
			atc_t8122_dp_lane_start(core, pair, rate->div2);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0, DP_PMA_BYTECLK_RESET);

	return 0;
}

static int atcphy_dp_set_rate_t8122(struct apple_atcphy *atcphy, unsigned int link_rate)
{
	const struct atcphy_mode_configuration *mode_cfg;
	const struct atc_t8122_dp_rate *rate;
	u8 pairs;
	int ret;

	lockdep_assert_held(&atcphy->lock);

	if (!link_rate)
		return atcphy_dp_link_stop_t8122(atcphy);

	rate = atc_t8122_dp_rate(link_rate);
	if (!rate) {
		dev_err(atcphy->dev, "Unsupported link rate: %u\n", link_rate);
		return -EINVAL;
	}
	if (!atcphy_modes[atcphy->mode].enable_dp_aux || !atcphy->dp_t8122.aux)
		return -ENOLINK;
	/* the AUSPLL is in use as the Thunderbolt DP tunnel pixel clock */
	if (atcphy->tunnel_clock_on)
		return -EBUSY;
	if (atcphy->dp_t8122.rate == link_rate)
		return 0;

	mode_cfg = atcphy_get_mode_config(atcphy, atcphy->mode);
	pairs = (mode_cfg->dp_lane[0] ? BIT(0) : 0) | (mode_cfg->dp_lane[1] ? BIT(1) : 0);

	ret = atcphy_dp_link_stop_t8122(atcphy);
	if (ret)
		return ret;
	ret = atcphy_dp_link_start_t8122(atcphy, rate, pairs);
	if (ret) {
		atcphy_dp_link_stop_t8122(atcphy);
		return ret;
	}
	atcphy->dp_t8122.rate = link_rate;
	dev_dbg(atcphy->dev, "DP link at %u Mb/s on lane pairs %#x (PCLK_STAT=%08x)\n",
		link_rate, pairs, readl(atcphy->regs.core + T8122_DP_PCLK_STATUS));
	return 0;
}

/*
 * Program the drive level DCP chose for each lane. Lanes fill the DP lane
 * pairs in ascending order, see atc_t8122_dp_lane_map().
 */
static int atcphy_dp_set_drive_t8122(struct apple_atcphy *atcphy,
				     const struct phy_configure_opts_dp *opts)
{
	u8 pairs = atcphy->dp_t8122.pairs;
	u32 presets[4];
	unsigned int i;
	int ret;

	lockdep_assert_held(&atcphy->lock);

	if (!pairs)
		return -ENOLINK;
	if (!opts->lanes || opts->lanes > 2 * hweight8(pairs))
		return -EINVAL;
	for (i = 0; i < opts->lanes; i++) {
		ret = atc_t8122_dp_preset(opts->voltage[i], opts->pre[i], &presets[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < opts->lanes; i++) {
		unsigned int pair;
		bool tx;

		ret = atc_t8122_dp_lane_map(pairs, i, &pair, &tx);
		if (ret)
			return ret;
		atc_t8122_dp_lane_drive(atcphy->regs.core, pair, tx, presets[i]);
	}
	dev_dbg(atcphy->dev, "DP drive: %u lanes, lane 0 swing %u pre-emphasis %u\n",
		opts->lanes, opts->voltage[0], opts->pre[0]);
	return 0;
}

/* Stop the main link and the AUX channel ahead of a mode change */
static void atcphy_dp_stop_t8122(struct apple_atcphy *atcphy)
{
	lockdep_assert_held(&atcphy->lock);

	atcphy_dp_link_stop_t8122(atcphy);
	if (atcphy->dp_t8122.aux) {
		atc_t8122_dp_aux_off(atcphy->regs.core);
		atcphy->dp_t8122.aux = false;
		dev_dbg(atcphy->dev, "DP AUX off\n");
	}
}

static int atcphy_configure(struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	int ret = 0;
	u32 reg;

	if (IS_ENABLED(CONFIG_PHY_APPLE_ATC_KUNIT_TEST))
		KUNIT_STATIC_STUB_REDIRECT(atcphy_configure, atcphy, mode);

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->t8103_dummy_pending)
		return -EBUSY;
	if (atcphy->pipe_pending)
		return -EBUSY;
	if (atcphy->usb_off_pending && mode != APPLE_ATCPHY_MODE_OFF)
		return -EBUSY;
	if (atcphy->hw->has_t8140_dp && atcphy->fixed_usb2 &&
	    atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY)
		return -EBUSY;
	if (atcphy->hw->has_t8140_dp && atcphy->dp_owned_pmas &&
	    (mode != atcphy->mode || atcphy->mode_dirty)) {
		atcphy->mode_dirty = true;
		ret = j700_dp_stop(atcphy);
		if (ret)
			return ret;
	}
	if (atcphy->hw->has_t8140_dp && atcphy->mode_dirty) {
		ret = j700_dp_idle_pending(atcphy);
		if (ret)
			return ret;
	}
	if (atcphy->hw->has_t8140_dp && atcphy_modes[mode].enable_dp_aux &&
	    !atcphy->dp_tunables)
		return -EOPNOTSUPP;
	if (atcphy->hw->has_t8140_dp &&
	    (!atcphy_modes[mode].enable_dp_aux || atcphy->mode_dirty) &&
	    atcphy->aux_power_requested) {
		atcphy->mode_dirty = true;
		ret = j700_aux_disable(atcphy);
		if (ret)
			return ret;
	}
	/* Keep failed transitions retryable even when the requested mode is unchanged. */
	atcphy->mode_dirty = true;
	/* the mode change below may power the PHY down: stop the tunnel clock first */
	if (apple_atc_tunnel_is_t6030(atcphy)) {
		ret = atc_t8122_tunnel_stop(atcphy);
		if (ret)
			return ret;
	} else {
		atc_tunnel_stop_t8103(atcphy);
	}
	atc_tunnel_restore(atcphy);
	if (atcphy->hw->dp_t8122)
		atcphy_dp_stop_t8122(atcphy);

	if (mode == APPLE_ATCPHY_MODE_OFF) {
		if (atcphy->hw->has_t8140_dp && atcphy->fixed_usb2 &&
		    (atcphy->dp_usb_context_valid || atcphy->usb_off_pending)) {
			atcphy->usb_off_pending = true;
			if (!atcphy->usb_off_drained) {
				ret = j700_usb_restore(atcphy, APPLE_ATCPHY_MODE_OFF);
				if (ret)
					return ret;
				atcphy->usb_off_drained = true;
			}
		}
		ret = atcphy_power_off(atcphy);
		if (ret)
			return ret;
		/* Recovery data was discharged before OFF, not discarded while dirty. */
		atcphy->dp_usb_context_valid = 0;
		atcphy->dp_usb_context_dirty = 0;
		atcphy->usb_off_pending = false;
		atcphy->usb_off_drained = false;
		atcphy->mode = mode;
		atcphy->mode_dirty = false;
		return 0;
	}

	/*
	 * Fixed-hub SAFE keeps USB2 up rather than taking the block OFF.
	 * With PIPE parked and DP owners released, reinitialize USB3 before
	 * programming its lanes; the existing sequence releases this reset.
	 */
	if (atcphy->hw->has_t8140_dp && atcphy->fixed_usb2)
		core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);

	ret = atcphy_power_on(atcphy);
	if (ret)
		return ret;

	if (atcphy->hw->has_t8140_dp && atcphy->fixed_usb2) {
		ret = j700_usb_capture(atcphy, mode);
		if (!ret)
			ret = j700_usb_restore(atcphy, mode);
		if (ret)
			return ret;
	}

	atcphy_apply_tunables(atcphy, mode);
	atcphy->hw->configure_post_tunable(atcphy, mode);

	/* Setup AUX channel if DP altmode is requested */
	if (atcphy_modes[mode].enable_dp_aux) {
		ret = atcphy_enable_dp_aux(atcphy);
		if (ret)
			return ret;
	}

	/* Enable clocks and configure lanes */
	if (atcphy->hw->gen == ATCPHY_GENERATION_T8103) {
		core_set32(atcphy, CIO3PLL_CLK_CTRL, CIO3PLL_CLK_PCLK_EN);
		core_set32(atcphy, CIO3PLL_CLK_CTRL, CIO3PLL_CLK_REFCLK_EN);
	}
	ret = atcphy_configure_lanes(atcphy, mode);
	if (ret)
		return ret;

	/* Take the USB3 PHY out of reset */
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);
	/*
	 * NOTE: Not what macos does, it takes the phy out of reset right before
	 * writing to ACIOPHY_CFG0_COMMON_SMALL, does lane configuration with phy
	 * out of reset, and the poll is right after AUS_COMMON_SHIM_BLK_BIAS_REG
	 * write. Does not seem to matter, but if something does not work,
	 * consider re-ordering things to macos order.
	 *
	 * T8122 only calibrates for the USB3 PIPE; the T8140/T8142 PHYs, woken in
	 * every mode, wait for the calibration in every mode as on 7.1.
	 */
	if (atcphy->hw->gen == ATCPHY_GENERATION_T8122 &&
	    (atcphy_modes[mode].pipehandler_state == ATCPHY_PIPEHANDLER_STATE_USB3 ||
	     atcphy->hw->configure_post_tunable == atcphy_configure_post_tunable_all_modes ||
	     (atcphy->hw->dp_t8122 && atcphy_modes[mode].enable_dp_aux))) {
		ret = readl_poll_timeout(atcphy->regs.core + AUS_COMMON_DIG_RCAL1, reg,
					 (reg & AUS_COMMON_DIG_RCAL1_ALL_CODES_DONE), 10, 100000);
		if (ret) {
			dev_err(atcphy->dev, "Failed to wait for RCAL1 done\n");
			return ret;
		}
	}

	atcphy->mode = mode;
	atcphy->mode_dirty = false;

	/* T8122: the AUX block needs the PHY out of reset and calibrated */
	if (atcphy->hw->dp_t8122 && atcphy_modes[mode].enable_dp_aux)
		atcphy_dp_aux_on_t8122(atcphy);

	return 0;
}

static void _atcphy_dwc3_reset_assert(struct apple_atcphy *atcphy);

/*
 * A T8122 PHY loses its register state whenever its power domain is switched
 * off. In system sleep that happens to a domain that no running device holds:
 * a Type-C port whose USB controller has no role (no cable, or a cable without
 * USB), or the DisplayPort-only PHY of the HDMI port. The domain then comes back
 * with the PHY at its reset values, while the driver still tracks the mode the
 * Type-C mux selected, a parked PIPE and possibly a running DP AUX channel.
 *
 * Bring the block back to that state: first the baseline probe establishes
 * (the USB controller held in reset, USB2 and the PHY off, the PIPE parked on
 * the dummy backend), then the tracked mode, which applies the tunables again,
 * sets up the lanes and powers the DP AUX channel. The USB controller has been
 * in reset all along, since its device keeps the domain on otherwise. A DP main
 * link or a tunnel pixel clock does not survive the power-down; their users
 * set them up again, so they are recorded as stopped.
 *
 * This runs from the first PHY operation after the domain is back, or from the
 * resume callback, whichever comes first, so the order in which the Type-C
 * controller, the USB controller and the display controller resume does not
 * matter.
 */
static void atcphy_restore_after_pd_off(struct apple_atcphy *atcphy)
{
	enum atcphy_mode mode = atcphy->mode;
	int ret;

	lockdep_assert_held(&atcphy->lock);

	if (!atcphy->hw->restore_after_pd_off || !READ_ONCE(atcphy->pd_was_off))
		return;
	WRITE_ONCE(atcphy->pd_was_off, false);

	atcphy->dp_t8122.aux = false;
	atcphy->dp_t8122.pll = false;
	atcphy->dp_t8122.pairs = 0;
	atcphy->dp_t8122.rate = 0;
	atcphy->tunnel_clock_on = false;
	atcphy->tunnel_rate = 0;
	atcphy->tunnel_users = 0;

	if (!atcphy->dp_only) {
		_atcphy_dwc3_reset_assert(atcphy);
		atcphy->host_active = false;
		atcphy_usb2_power_off(atcphy);
	}
	ret = atcphy_power_off(atcphy);
	if (ret)
		dev_warn(atcphy->dev, "PHY did not power down after its domain was off: %d\n",
			 ret);
	atcphy->mode = APPLE_ATCPHY_MODE_OFF;

	if (atcphy->dp_only) {
		dev_dbg(atcphy->dev, "power domain was off, restoring mode %d\n", mode);
	} else {
		ret = atcphy_configure_pipehandler_dummy(atcphy, false);
		if (ret)
			dev_warn(atcphy->dev, "Failed to park the PIPE after its domain was off: %d\n",
				 ret);
		atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_DUMMY;
		dev_dbg(atcphy->dev,
			"power domain was off, restoring mode %d: PIPE mux=%08x lock=%08x/%08x aon=%08x nonsel=%08x\n",
			mode, readl(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL),
			readl(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ),
			readl(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK),
			readl(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN),
			readl(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE));
	}

	if (mode == APPLE_ATCPHY_MODE_OFF)
		return;
	ret = atcphy_configure(atcphy, mode);
	if (ret)
		dev_warn(atcphy->dev, "Failed to restore mode %d after its domain was off: %d\n",
			 mode, ret);
}

static int atcphy_usb2_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
		set32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		break;
	case PHY_MODE_USB_DEVICE:
		clear32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int atcphy_usb2_init(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);
	if (atcphy->usb_off_pending)
		return -EBUSY;
	if (atcphy->pipe_pending &&
	    (atcphy->mode == APPLE_ATCPHY_MODE_OFF || atcphy->mode_dirty)) {
		int ret = atcphy_park_pipehandler(atcphy, false);

		if (ret)
			return ret;
	}

	/* A delayed shutdown may have completed after the last Type-C event. */
	if (!atcphy->fixed_usb2) {
		if (atcphy->mode == APPLE_ATCPHY_MODE_OFF || atcphy->mode_dirty)
			return atcphy_configure(atcphy, atcphy->typec_mode);
		return 0;
	}

	/*
	 * dwc3 initialises its PHYs after releasing its reset, and asserting that
	 * reset powered the USB2 PHY off. A port behind a fixed hub gets no Type-C
	 * event that would power it up again before the controller starts, so do
	 * it here. The port is a host port by construction. When the USB3 PHY was
	 * powered off as well the whole block is off; bring it back in the mode
	 * the Type-C mux last asked for, USB2 until a cable event says otherwise.
	 */
	set32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
	if (atcphy->mode == APPLE_ATCPHY_MODE_OFF || atcphy->mode_dirty)
		return atcphy_configure(atcphy, atcphy->typec_mode);

	atcphy_usb2_power_on(atcphy);

	return 0;
}

static const struct phy_ops apple_atc_usb2_phy_ops = {
	.owner = THIS_MODULE,
	.init = atcphy_usb2_init,
	.set_mode = atcphy_usb2_set_mode,
};

static int atcphy_usb3_power_off(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	/*
	 * dwc3 has left host mode: nothing drives the PIPE any more, and the
	 * next host powers its SuperSpeed port on as it starts.
	 */
	atcphy->host_active = false;
	atcphy->usb3_port_off = false;

	ret = atcphy_park_pipehandler(atcphy, true);
	if (ret)
		return ret;
	atcphy->t8103_usb3_retry = false;
	/*
	 * dwc3 can tear its host down while a DisplayPort alt mode link is up,
	 * e.g. when a monitor's built-in hub re-enumerates after a hot-plug.
	 * Powering the PHY off then cuts the DP lanes under DCP, which reports a
	 * DPTX FIFO error and drops the display. The Type-C mux turns the PHY off
	 * when the alt mode actually ends, so leave a DP mode alone here.
	 */
	if (atcphy_modes[atcphy->mode].enable_dp_aux)
		return 0;

	if (atcphy->mode != APPLE_ATCPHY_MODE_OFF || atcphy->mode_dirty) {
		ret = atcphy_configure(atcphy, APPLE_ATCPHY_MODE_OFF);
		if (ret)
			return ret;
	}

	return 0;
}

/* Route the PIPE to the backend of the current mode for the role dwc3 selected. */
static int atcphy_usb3_route(struct apple_atcphy *atcphy, bool host)
{
	int ret;

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->t8103_dummy_pending) {
		ret = atcphy_park_live_t8103_pipehandler(atcphy);
		if (ret)
			return ret;
	}


	/*
	 * We may get multiple calls to set_mode (for host mode e.g. at least one from the dwc3 glue
	 * driver and then another one from the generic xhci code) but must only configure the
	 * PIPE handler once. Nothing needs to be done either when the PIPE is already routed to
	 * the backend the current mode uses, which includes the dummy backend of the USB2 modes.
	 */
	/* A failed lane/domain transition must be restored before routing PIPE. */
	if (atcphy->mode_dirty)
		return -EBUSY;
	/* dwc3 can select its role before the debounced Type-C mux update. */
	if (atcphy->t8103_usb3_workaround &&
	    atcphy->mode == APPLE_ATCPHY_MODE_OFF) {
		/* Keep the valid role's route owed across a late mux update or failure. */
		atcphy->t8103_usb3_retry = true;
		if (atcphy->typec_mode != APPLE_ATCPHY_MODE_USB3)
			return 0;
		if (atcphy->dp_only || !atcphy->ss_tunables)
			return -EOPNOTSUPP;
		ret = atcphy_configure(atcphy, APPLE_ATCPHY_MODE_USB3);
		if (ret)
			return ret;
	}
	if (atcphy->pipe_pending) {
		ret = atcphy_park_pipehandler(atcphy, false);
		if (ret)
			return ret;
	}
	if (atcphy->pipe_state == atcphy_modes[atcphy->mode].pipehandler_state &&
	    !atcphy->pipe_pending)
		return 0;

	return atcphy_configure_pipehandler(atcphy, host);
}

static int atcphy_usb3_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	if (atcphy->usb_off_pending)
		return -EBUSY;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		atcphy->host_active = true;
		/* A host that starts up powers the ports of its root hubs itself. */
		atcphy->usb3_port_off = false;
		break;
	case PHY_MODE_USB_DEVICE:
		atcphy->host_active = false;
		break;
	default:
		return -EINVAL;
	}

	ret = atcphy_usb3_route(atcphy, mode == PHY_MODE_USB_HOST);
	if (!ret || !atcphy->fixed_usb2 || mode != PHY_MODE_USB_HOST)
		return ret;

	/*
	 * The host behind a fixed hub serves that hub as well, and when this
	 * fails, xHCI registers neither of its root hubs. Let it start, with
	 * the PIPE parked if that works: the Type-C mux update that restores
	 * the lanes or retries the handoff routes the PIPE, with the
	 * SuperSpeed port off.
	 */
	if (atcphy_park_pipehandler(atcphy, false))
		dev_warn(atcphy->dev, "Host starts with its PIPE not parked\n");
	else
		dev_dbg(atcphy->dev, "Host starts without USB3: %d\n", ret);
	return 0;
}

static const struct phy_ops apple_atc_usb3_phy_ops = {
	.owner = THIS_MODULE,
	.power_off = atcphy_usb3_power_off,
	.set_mode = atcphy_usb3_set_mode,
};

static int atcphy_dpphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	/* Nothing to do here since the setup already happened in mux_set */
	if (mode == PHY_MODE_DP && submode == 0)
		return 0;
	return -EINVAL;
}

static int atcphy_dpphy_validate(struct phy *phy, enum phy_mode mode, int submode,
				 union phy_configure_opts *opts_)
{
	struct phy_configure_opts_dp *opts = &opts_->dp;
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	if (mode != PHY_MODE_DP)
		return -EINVAL;
	if (submode != 0)
		return -EINVAL;

	switch (atcphy->mode) {
	case APPLE_ATCPHY_MODE_USB3_DP:
		opts->lanes = 2;
		break;
	case APPLE_ATCPHY_MODE_DP:
		opts->lanes = 4;
		break;
	default:
		opts->lanes = 0;
	}

	return 0;
}

static int atcphy_dpphy_configure_t8122(struct apple_atcphy *atcphy,
					struct phy_configure_opts_dp *opts)
{
	int ret;

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	if (opts->set_lanes)
		return -EINVAL;
	if (opts->set_rate) {
		ret = atcphy_dp_set_rate_t8122(atcphy, opts->link_rate);
		if (ret)
			return ret;
	}
	if (opts->set_voltages)
		return atcphy_dp_set_drive_t8122(atcphy, opts);
	return 0;
}

static int atcphy_dpphy_configure(struct phy *phy, union phy_configure_opts *opts_)
{
	struct phy_configure_opts_dp *opts = &opts_->dp;
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	enum atcphy_dp_link_rate link_rate;

	if (atcphy->hw->has_t8140_dp)
		return j700_dp_configure(atcphy, opts);

	if (atcphy->hw->dp_t8122)
		return atcphy_dpphy_configure_t8122(atcphy, opts);

	if (opts->set_voltages)
		return -EINVAL;
	if (opts->set_lanes)
		return -EINVAL;

	if (opts->set_rate) {
		switch (opts->link_rate) {
		case 1620:
			link_rate = ATCPHY_DP_LINK_RATE_RBR;
			break;
		case 2700:
			link_rate = ATCPHY_DP_LINK_RATE_HBR;
			break;
		case 5400:
			link_rate = ATCPHY_DP_LINK_RATE_HBR2;
			break;
		case 8100:
			link_rate = ATCPHY_DP_LINK_RATE_HBR3;
			break;
		case 0:
			return 0;
		default:
			dev_err(atcphy->dev, "Unsupported link rate: %d\n", opts->link_rate);
			return -EINVAL;
		}

		return atcphy_dp_configure(atcphy, link_rate);
	}

	return 0;
}

static const struct phy_ops apple_atc_dp_phy_ops = {
	.owner = THIS_MODULE,
	.configure = atcphy_dpphy_configure,
	.validate = atcphy_dpphy_validate,
	.set_mode = atcphy_dpphy_set_mode,
};

/*
 * Called by appledrm when DCP sets the link rate of a DPTX that feeds
 * Thunderbolt DP IN adapter @dpin (rate is the DP link rate code, 0 = stop).
 */
/*
 * t600x (M1 Pro/Max) and t8112 (M2, whose ATC PHY is t8103-compatible) run
 * the t8103 tunnel clock sequence unchanged.
 */
static bool apple_atc_tunnel_is_t8103_style(void)
{
	return of_machine_is_compatible("apple,t8103") ||
	       of_machine_is_compatible("apple,t8112") || apple_atc_tunnel_is_t600x();
}

int apple_atc_dp_tunnel_rate_source(struct phy *phy, unsigned int source,
				    unsigned int dpin, u8 rate)
{
	struct apple_atcphy *atcphy;
	int pclk, ret;

	if (!phy || phy->ops != &apple_atc_dp_phy_ops || dpin > 1)
		return -EINVAL;
	atcphy = phy_get_drvdata(phy);
	pclk = apple_dp_tunnel_t6030_pclk(source, dpin);
	if (!apple_atc_tunnel_is_t6030(atcphy) || pclk < 0)
		return -EOPNOTSUPP;
	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);
	/* A stale source must not retune or stop another source's clock. */
	if ((atcphy->tunnel_users & BIT(dpin)) &&
	    atcphy->tunnel_source[dpin] != source)
		return -ESTALE;
	if (!rate) {
		if (!(atcphy->tunnel_users & BIT(dpin)))
			return 0;
		if (!(atcphy->tunnel_users & ~BIT(dpin)))
			return atc_t8122_tunnel_stop(atcphy);
		/* Release only this stream; the other stream keeps its PLL and gates. */
		if (pclk)
			core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
				     DPTX_PCLK2_ENABLE);
		atcphy->tunnel_users &= ~BIT(dpin);
		atcphy->tunnel_dpin_rate[dpin] = 0;
		return 0;
	}
	if (atcphy->mode != APPLE_ATCPHY_MODE_USB4 &&
	    atcphy->mode != APPLE_ATCPHY_MODE_TBT)
		return -EBUSY;
	if (atcphy->tunnel_clock_on && !atcphy->tunnel_rate)
		return -EBUSY;
	/* Keep this owner if a failed start cannot shut the PLL down. */
	if ((atcphy->tunnel_users & BIT(1 - dpin)) &&
	    atcphy->tunnel_source[1 - dpin] == source)
		return -EBUSY;
	if ((atcphy->tunnel_users & BIT(dpin)) &&
	    atcphy->tunnel_dpin_rate[dpin] == rate && atcphy->tunnel_rate)
		return 0;
	atcphy->tunnel_source[dpin] = source;
	ret = atc_t8122_tunnel_start(atcphy, pclk, rate);
	if (!ret || (atcphy->tunnel_clock_on && !atcphy->tunnel_rate)) {
		atcphy->tunnel_users |= BIT(dpin);
		atcphy->tunnel_dpin_rate[dpin] = ret ? 0 : rate;
	}
	dev_dbg(atcphy->dev, "DP tunnel source %u rate 0x%x: %d (TX_DP_CTRL0=%08x PCLK_STAT=%08x)\n",
		source, rate, ret,
		readl(atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0),
		readl(atcphy->regs.core + T8122_DP_PCLK_STATUS));
	return ret;
}
EXPORT_SYMBOL_GPL(apple_atc_dp_tunnel_rate_source);

int apple_atc_dp_tunnel_rate(struct phy *phy, unsigned int dpin, u8 rate)
{
	struct apple_atcphy *atcphy;
	int ret;

	if (!phy || phy->ops != &apple_atc_dp_phy_ops || dpin > 1)
		return -EINVAL;
	atcphy = phy_get_drvdata(phy);
	/* Keep each supported SoC on its qualified clock sequence. */
	if (apple_atc_tunnel_is_t6030(atcphy))
		return apple_atc_dp_tunnel_rate_source(phy, 0, dpin, rate);
	if (!apple_atc_tunnel_is_t8103_style()) {
		bool valid_core = apple_atc_t602x_core_valid(atcphy->np, atcphy->res.core->start,
							     resource_size(atcphy->res.core));

		ret = apple_atc_t602x_gate(atcphy->tunnel_dual_stream,
					   atcphy->tunnel_routes_present, valid_core);
		if (ret)
			return ret;
	}
	guard(mutex)(&atcphy->lock);
	if (!rate) {
		if (apple_atc_tunnel_is_t8103_style()) {
			/*
			 * One pixel clock feeds both DP IN adapters: a stream
			 * that ends must not stop the other one's clock.
			 */
			atcphy->tunnel_users &= ~BIT(dpin);
			if (!atcphy->tunnel_users)
				atc_tunnel_stop_t8103(atcphy);
		} else {
			atc_tunnel_stop_t602x(atcphy, dpin);
		}
		return 0;
	}
	if (atcphy->mode != APPLE_ATCPHY_MODE_USB4 && atcphy->mode != APPLE_ATCPHY_MODE_TBT)
		return -EBUSY;
	if (apple_atc_tunnel_is_t8103_style()) {
		ret = atc_tunnel_start_t8103(atcphy, rate);
		if (!ret)
			atcphy->tunnel_users |= BIT(dpin);
	} else {
		ret = atc_tunnel_set_t602x(atcphy, dpin, rate);
	}
	dev_dbg(atcphy->dev, "DP tunnel clock rate 0x%x: %d (TX_DP_CTRL0=%08x PCLK_STAT=%08x)\n",
		rate, ret, readl(atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0),
		readl(atcphy->regs.core + (atcphy->hw->gen == ATCPHY_GENERATION_T8122 ?
					 T8122_DP_PCLK_STATUS : ACIOPHY_DP_PCLK_STAT)));
	return ret;
}
EXPORT_SYMBOL_GPL(apple_atc_dp_tunnel_rate);

int apple_atc_dp_tunnel_open(struct phy *phy)
{
	struct apple_atcphy *atcphy;

	if (!phy || phy->ops != &apple_atc_dp_phy_ops)
		return -EINVAL;
	if (!apple_atc_tunnel_is_t600x())
		return -EOPNOTSUPP;
	atcphy = phy_get_drvdata(phy);
	guard(mutex)(&atcphy->lock);
	if (atcphy->mode != APPLE_ATCPHY_MODE_USB4 && atcphy->mode != APPLE_ATCPHY_MODE_TBT)
		return -EBUSY;
	if (!atcphy->tunnel_clock_on) {
		atc_tunnel_wake_t8103(atcphy);
		atc_tunnel_pclk_t8103(atcphy, 0);
	}
	dev_dbg(atcphy->dev, "DP tunnel open (CFG0=%08x SLEEP_CTRL=%08x TX_DP_CTRL0=%08x)\n",
		readl(atcphy->regs.core + ACIOPHY_CFG0),
		readl(atcphy->regs.core + ACIOPHY_SLEEP_CTRL),
		readl(atcphy->regs.core + ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0));
	return 0;
}
EXPORT_SYMBOL_GPL(apple_atc_dp_tunnel_open);

static struct phy *atcphy_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct apple_atcphy *atcphy = dev_get_drvdata(dev);
	struct phy *phy = NULL;

	switch (args->args[0]) {
	case PHY_TYPE_USB2:
		phy = atcphy->phys.usb2;
		break;
	case PHY_TYPE_USB3:
		phy = atcphy->phys.usb3;
		break;
	case PHY_TYPE_DP:
		phy = atcphy->phys.dp;
		break;
	}
	/* a DisplayPort-only instance has no USB PHYs */
	return phy ?: ERR_PTR(-ENODEV);
}

static int atcphy_probe_phy(struct apple_atcphy *atcphy)
{
	struct {
		struct phy **phy;
		const struct phy_ops *ops;
	} phys[] = {
		{ &atcphy->phys.usb2, &apple_atc_usb2_phy_ops },
		{ &atcphy->phys.usb3, &apple_atc_usb3_phy_ops },
		{ &atcphy->phys.dp, &apple_atc_dp_phy_ops },
	};

	for (int i = 0; i < ARRAY_SIZE(phys); i++) {
		if (atcphy->dp_only && phys[i].phy != &atcphy->phys.dp)
			continue;
		*phys[i].phy = devm_phy_create(atcphy->dev, NULL, phys[i].ops);
		if (IS_ERR(*phys[i].phy))
			return PTR_ERR(*phys[i].phy);
		phy_set_drvdata(*phys[i].phy, atcphy);
	}

	atcphy->phy_provider = devm_of_phy_provider_register(atcphy->dev, atcphy_xlate);
	if (IS_ERR(atcphy->phy_provider))
		return PTR_ERR(atcphy->phy_provider);
	return 0;
}

static void _atcphy_dwc3_reset_assert(struct apple_atcphy *atcphy)
{
	lockdep_assert_held(&atcphy->lock);

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
	      PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
}

static int atcphy_dwc3_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_atcphy *atcphy = container_of(rcdev, struct apple_atcphy, rcdev);
	int ret;

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	/* The controller is only reset once it has stopped. */
	atcphy->host_active = false;
	atcphy->usb3_port_off = false;

	/* Do not reset the controller or USB2 PHY if PIPE parking fails. */
	ret = atcphy_park_pipehandler(atcphy, true);
	if (ret)
		return ret;

	atcphy->t8103_usb3_retry = false;
	_atcphy_dwc3_reset_assert(atcphy);
	atcphy_usb2_power_off(atcphy);

	return 0;
}

static int atcphy_dwc3_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_atcphy *atcphy = container_of(rcdev, struct apple_atcphy, rcdev);

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);
	if (atcphy->t8103_dummy_pending) {
		int ret = atcphy_park_live_t8103_pipehandler(atcphy);

		if (ret)
			return ret;
	}

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
		PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);

	return 0;
}

static const struct reset_control_ops atcphy_dwc3_reset_ops = {
	.assert = atcphy_dwc3_reset_assert,
	.deassert = atcphy_dwc3_reset_deassert,
};

static int atcphy_reset_xlate(struct reset_controller_dev *rcdev,
			      const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int atcphy_probe_rcdev(struct apple_atcphy *atcphy)
{
	atcphy->rcdev.owner = THIS_MODULE;
	atcphy->rcdev.nr_resets = 1;
	atcphy->rcdev.ops = &atcphy_dwc3_reset_ops;
	atcphy->rcdev.of_node = atcphy->dev->of_node;
	atcphy->rcdev.of_reset_n_cells = 0;
	atcphy->rcdev.of_xlate = atcphy_reset_xlate;

	return devm_reset_controller_register(atcphy->dev, &atcphy->rcdev);
}

static int atcphy_set_orientation(struct apple_atcphy *atcphy, enum typec_orientation orientation)
{
	bool swap_lanes;

	guard(mutex)(&atcphy->lock);

	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		return 0;
	case TYPEC_ORIENTATION_NORMAL:
		swap_lanes = false;
		break;
	case TYPEC_ORIENTATION_REVERSE:
		swap_lanes = true;
		break;
	default:
		return -EINVAL;
	}
	/* The mode alone cannot describe a route after its orientation changes. */
	if (atcphy->swap_lanes != swap_lanes) {
		atcphy->swap_lanes = swap_lanes;
		atcphy->mode_dirty = true;
	}
	dev_dbg(atcphy->dev, "orientation=%u mode=%u dirty=%u pipe=%u host=%u\n",
		orientation, atcphy->mode, atcphy->mode_dirty,
		atcphy->pipe_state, atcphy->host_active);

	return 0;
}

static int atcphy_sw_set(struct typec_switch_dev *sw, enum typec_orientation orientation)
{
	return atcphy_set_orientation(typec_switch_get_drvdata(sw), orientation);
}

static void atcphy_typec_switch_unregister(void *data)
{
	typec_switch_unregister(data);
}

static int atcphy_probe_switch(struct apple_atcphy *atcphy)
{
	struct typec_switch_dev *sw;
	struct typec_switch_desc sw_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_sw_set,
	};

	sw = typec_switch_register(atcphy->dev, &sw_desc);
	if (IS_ERR(sw))
		return PTR_ERR(sw);

	return devm_add_action_or_reset(atcphy->dev, atcphy_typec_switch_unregister, sw);
}

/*
 * The controller of a fixed-hub port stays up across Type-C mode changes, so
 * the PHY moves the PIPE while its SuperSpeed root port is powered off, see
 * atcphy_mux_update(). Hold the controller, its xHCI device and the awake
 * SuperSpeed root hub across the change: hub_wq then cannot use the port in
 * between, and none of them can go away. They are taken before the ATC lock,
 * as USB PM can call back into the PHY on recovery.
 */
struct atcphy_usb3_pm {
	struct platform_device *controller;
	struct device *host;
	struct usb_device *root;
};

#if IS_REACHABLE(CONFIG_USB)
/* The SuperSpeed root hub of the controller; its USB2 root hub never matches */
static bool atcphy_usb3_is_root(const struct usb_device *udev, const struct device *controller)
{
	return !udev->parent && udev->speed >= USB_SPEED_SUPER && udev->bus->sysdev == controller;
}

static int atcphy_usb3_pm_find(struct usb_device *udev, void *data)
{
	struct atcphy_usb3_pm *pm = data;

	if (!atcphy_usb3_is_root(udev, &pm->controller->dev))
		return 0;

	pm->root = usb_get_dev(udev);
	return 1;
}
#endif

static int atcphy_usb3_pm_get(struct apple_atcphy *atcphy, struct atcphy_usb3_pm *pm)
{
#if IS_REACHABLE(CONFIG_USB)
	struct device_node *ep __free(device_node) = NULL;
	struct device_node *ctrl __free(device_node) = NULL;
	struct usb_hcd *hcd;
	int ret;
#endif

	KUNIT_STATIC_STUB_REDIRECT(atcphy_usb3_pm_get, atcphy, pm);

	if (!atcphy->fixed_usb2 || !atcphy->hw->has_t8140_dp)
		return 0;

#if IS_REACHABLE(CONFIG_USB)
	ep = of_graph_get_endpoint_by_regs(atcphy->np, 1, -1);
	ctrl = ep ? of_graph_get_remote_port_parent(ep) : NULL;
	if (!ctrl)
		return -ENODEV;
	pm->controller = of_find_device_by_node(ctrl);
	if (!pm->controller)
		return -EAGAIN;

	/* Driver core takes this lock before DWC3 removal touches SUSPHY. */
	device_lock(&pm->controller->dev);
	if (!pm->controller->dev.driver) {
		ret = -EAGAIN;
		goto put_controller;
	}
	if (strcmp(pm->controller->dev.driver->name, "dwc3-apple")) {
		ret = -ENODEV;
		goto put_controller;
	}

	/* Firmware identity, not a bus number, binds this PHY to its root hub. */
	usb_for_each_dev(pm, atcphy_usb3_pm_find);
	if (!pm->root) {
		ret = -EAGAIN;
		goto put_controller;
	}

	/* Also serialize direct xHCI unbind, before it marks the HCD removing. */
	pm->host = get_device(pm->root->bus->controller);
	device_lock(pm->host);
	if (!pm->host->driver) {
		ret = -EAGAIN;
		goto put_root;
	}
	if (pm->host->parent != &pm->controller->dev ||
	    strcmp(pm->host->driver->name, "xhci-hcd")) {
		ret = -ENODEV;
		goto put_root;
	}

	/* USB destruction follows the platform lifecycle locks, not vice versa. */
	usb_lock_device(pm->root);
	hcd = bus_to_hcd(pm->root->bus);
	if (READ_ONCE(pm->root->state) == USB_STATE_NOTATTACHED || HCD_DEAD(hcd)) {
		ret = -ENODEV;
		goto unlock_root;
	}
	/* The graph identifies the controller, not a port in a multiport root. */
	if (pm->root->maxchild != 1) {
		ret = pm->root->maxchild ? -ENODEV : -EAGAIN;
		goto unlock_root;
	}
	ret = pm_runtime_resume_and_get(&pm->root->dev);
	if (ret < 0)
		goto unlock_root;
	if (READ_ONCE(pm->root->state) == USB_STATE_NOTATTACHED ||
	    READ_ONCE(hcd->state) != HC_STATE_RUNNING ||
	    HCD_DEAD(hcd) || !HCD_HW_ACCESSIBLE(hcd)) {
		pm_runtime_put_noidle(&pm->root->dev);
		ret = -ENODEV;
		goto unlock_root;
	}
	return 0;

unlock_root:
	usb_unlock_device(pm->root);
put_root:
	device_unlock(pm->host);
	put_device(pm->host);
	pm->host = NULL;
	usb_put_dev(pm->root);
	pm->root = NULL;
put_controller:
	device_unlock(&pm->controller->dev);
	put_device(&pm->controller->dev);
	pm->controller = NULL;
	return ret;
#else
	/* typec_mux_set() skips EOPNOTSUPP, which would report a false success. */
	return -ENODEV;
#endif
}

static void atcphy_usb3_pm_put(struct atcphy_usb3_pm *pm)
{
#if IS_REACHABLE(CONFIG_USB)
	if (!pm->root)
		return;

	/* Restore normal runtime policy after both success and failed parking. */
	usb_mark_last_busy(pm->root);
	pm_runtime_put_autosuspend(&pm->root->dev);
	usb_unlock_device(pm->root);
	usb_put_dev(pm->root);
	device_unlock(pm->host);
	put_device(pm->host);
	device_unlock(&pm->controller->dev);
	put_device(&pm->controller->dev);
	pm->host = NULL;
	pm->root = NULL;
	pm->controller = NULL;
#endif
}

/*
 * Whether the controller of a fixed-hub port may run its SuperSpeed port: the
 * PIPE is on the USB3 backend for its host, and no lane or PIPE change is left
 * unfinished.
 */
static bool atcphy_usb3_port_ready(const struct apple_atcphy *atcphy)
{
	return atcphy->fixed_usb2 && atcphy->host_active && !atcphy->mode_dirty &&
	       !atcphy->pipe_pending && atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_USB3;
}

/*
 * Take the controller's SuperSpeed port down before the PIPE moves: the device
 * on it is disconnected and the port held powered off, so that nothing drives
 * the PIPE when it is handed over. Nothing else of the controller stops, its
 * USB2 root port and the fixed hub on it included. Returns 1 when there is no
 * such port to take down, and for a port that someone else switched off: that
 * stays theirs and is not powered on again.
 */
static int atcphy_usb3_port_off(struct apple_atcphy *atcphy, struct atcphy_usb3_pm *pm)
{
	int ret = 1;

	KUNIT_STATIC_STUB_REDIRECT(atcphy_usb3_port_off, atcphy, pm);

#if IS_REACHABLE(CONFIG_USB)
	if (!pm->root)
		return 1;

	ret = usb_hub_port_power_off(pm->root, 1);
	if (ret < 0)
		dev_warn(atcphy->dev, "Failed to power the SuperSpeed port off: %d\n", ret);
#endif
	return ret;
}

/*
 * Bring the SuperSpeed port up again once the PIPE is back on the USB3 backend.
 * Returns 1 for a port that someone else has taken over since it went down,
 * through its "disable" attribute: it stays off.
 */
static int atcphy_usb3_port_on(struct apple_atcphy *atcphy, struct atcphy_usb3_pm *pm)
{
	int ret = 1;

	KUNIT_STATIC_STUB_REDIRECT(atcphy_usb3_port_on, atcphy, pm);

#if IS_REACHABLE(CONFIG_USB)
	if (!pm->root)
		return 1;

	ret = usb_hub_port_power_on(pm->root, 1);
	if (ret < 0)
		dev_warn(atcphy->dev, "Failed to power the SuperSpeed port on: %d\n", ret);
#endif
	return ret;
}

/*
 * A mux update that keeps the mode does not fail on the USB side, so that the
 * display it may carry stays up, and no Type-C event need follow any update.
 * A SuperSpeed port power-on that an update still owes after a USB failure is
 * retried from here, a few times, with the locks of a mux update, the current
 * root hub, and only while the PIPE is on the USB3 backend of a running host.
 * The work is freezable, so it waits across system sleep, and is cancelled on
 * removal.
 */
#define ATCPHY_USB3_PORT_RETRIES	5
#define ATCPHY_USB3_PORT_RETRY_MS	100

static unsigned long atcphy_usb3_port_retry_delay(unsigned int retry)
{
	KUNIT_STATIC_STUB_REDIRECT(atcphy_usb3_port_retry_delay, retry);

	return msecs_to_jiffies(ATCPHY_USB3_PORT_RETRY_MS << retry);
}

static void atcphy_usb3_port_retry(struct apple_atcphy *atcphy, bool restart, int err)
{
	unsigned int retry = 0;

	scoped_guard(mutex, &atcphy->lock) {
		if (restart)
			atcphy->usb3_port_retries = ATCPHY_USB3_PORT_RETRIES;
		if (!atcphy->usb3_port_retries) {
			dev_warn(atcphy->dev, "SuperSpeed port left off: %d\n", err);
			return;
		}
		retry = ATCPHY_USB3_PORT_RETRIES - atcphy->usb3_port_retries--;
	}
	dev_dbg(atcphy->dev, "SuperSpeed port power-on deferred: %d\n", err);
	mod_delayed_work(system_freezable_wq, &atcphy->usb3_port_work,
			 atcphy_usb3_port_retry_delay(retry));
}

/* Power the SuperSpeed port on if it is still owed and the route still ready */
static int atcphy_usb3_port_restore(struct apple_atcphy *atcphy, struct atcphy_usb3_pm *pm)
{
	int ret;

	scoped_guard(mutex, &atcphy->lock) {
		if (!atcphy->usb3_port_off || !atcphy_usb3_port_ready(atcphy))
			return 0;
	}
	ret = atcphy_usb3_port_on(atcphy, pm);
	if (ret < 0)
		return ret;
	scoped_guard(mutex, &atcphy->lock)
		atcphy->usb3_port_off = false;

	return 0;
}

static void atcphy_usb3_port_work(struct work_struct *work)
{
	struct apple_atcphy *atcphy = container_of(to_delayed_work(work), struct apple_atcphy,
						   usb3_port_work);
	struct atcphy_usb3_pm pm = {};
	int ret;

	ret = atcphy_usb3_pm_get(atcphy, &pm);
	if (!ret) {
		ret = atcphy_usb3_port_restore(atcphy, &pm);
		atcphy_usb3_pm_put(&pm);
	}
	if (ret)
		atcphy_usb3_port_retry(atcphy, false, ret);
}

/* The PHY mode a Type-C mux state asks for; unsupported states turn the PHY off. */
static enum atcphy_mode atcphy_typec_target_mode(struct apple_atcphy *atcphy,
						 const struct typec_mux_state *state)
{
	enum atcphy_mode target_mode;

	if (state->mode == TYPEC_STATE_SAFE) {
		target_mode = APPLE_ATCPHY_MODE_OFF;
	} else if (state->mode == TYPEC_STATE_USB) {
		target_mode = APPLE_ATCPHY_MODE_USB3;
	} else if (!state->alt && state->mode == TYPEC_MODE_USB4) {
		const struct enter_usb_data *data = state->data;
		u32 eudo_usb_mode = FIELD_GET(EUDO_USB_MODE_MASK, data->eudo);

		switch (eudo_usb_mode) {
		case EUDO_USB_MODE_USB2:
			target_mode = APPLE_ATCPHY_MODE_USB2;
			break;
		case EUDO_USB_MODE_USB3:
			target_mode = APPLE_ATCPHY_MODE_USB3;
			break;
		case EUDO_USB_MODE_USB4:
			target_mode = APPLE_ATCPHY_MODE_USB4;
			break;
		default:
			dev_warn(atcphy->dev, "Unsupported EUDO USB mode: 0x%x.\n", eudo_usb_mode);
			target_mode = APPLE_ATCPHY_MODE_OFF;
		}
	} else if (state->alt && state->alt->svid == USB_TYPEC_TBT_SID) {
		target_mode = APPLE_ATCPHY_MODE_TBT;
	} else if (state->alt && state->alt->svid == USB_TYPEC_DP_SID) {
		switch (state->mode) {
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E:
			target_mode = APPLE_ATCPHY_MODE_DP;
			break;
		case TYPEC_DP_STATE_D:
			target_mode = APPLE_ATCPHY_MODE_USB3_DP;
			break;
		default:
			dev_err(atcphy->dev,
				"Unsupported DP pin assignment: 0x%lx, your connected device will not work.\n",
				state->mode);
			target_mode = APPLE_ATCPHY_MODE_OFF;
		}
	} else if (state->alt) {
		dev_err(atcphy->dev,
			"Unknown alternate mode SVID: 0x%x, your connected device will not work.\n",
			state->alt->svid);
		target_mode = APPLE_ATCPHY_MODE_OFF;
	} else {
		dev_err(atcphy->dev, "Unknown mode: 0x%lx, your connected device will not work.\n",
			state->mode);
		target_mode = APPLE_ATCPHY_MODE_OFF;
	}

	/*
	 * A port behind a fixed hub keeps its USB controller in host mode, and
	 * the controller keeps using the USB2 PHY for the hub. Keep the block in
	 * USB2 when the connector goes to its safe state instead of powering it
	 * off underneath the hub.
	 */
	if (atcphy->fixed_usb2 && target_mode == APPLE_ATCPHY_MODE_OFF)
		target_mode = APPLE_ATCPHY_MODE_USB2;

	return target_mode;
}

/*
 * Whether the lanes and the PIPE are as @target_mode needs them: the mode does
 * not change, and no earlier PIPE handoff to a running controller behind a
 * fixed hub is still outstanding.
 */
static bool atcphy_mux_in_mode(const struct apple_atcphy *atcphy, enum atcphy_mode target_mode)
{
	if (atcphy->mode != target_mode || atcphy->mode_dirty ||
	    atcphy->t8103_dummy_pending || atcphy->t8103_usb3_retry)
		return false;

	return !(atcphy->fixed_usb2 && atcphy->host_active &&
		 (atcphy->pipe_state != atcphy_modes[target_mode].pipehandler_state ||
		  atcphy->pipe_pending));
}

/*
 * Whether a mux update leaves both the lanes and the PIPE as they are, and
 * owes no SuperSpeed port its power either. A DisplayPort HPD or status
 * update with an unchanged pin assignment is one of these.
 */
static bool atcphy_mux_is_noop(const struct apple_atcphy *atcphy, enum atcphy_mode target_mode)
{
	return atcphy_mux_in_mode(atcphy, target_mode) &&
	       !(atcphy->usb3_port_off && atcphy_usb3_port_ready(atcphy));
}

/* Whether the PHY can run @mode at all, which needs no lane or PIPE work to tell */
static int atcphy_mux_check(const struct apple_atcphy *atcphy, enum atcphy_mode mode)
{
	if (atcphy->dp_only && mode != APPLE_ATCPHY_MODE_OFF &&
	    mode != APPLE_ATCPHY_MODE_DP)
		return -EOPNOTSUPP;

	switch (mode) {
	case APPLE_ATCPHY_MODE_TBT:
	case APPLE_ATCPHY_MODE_USB4:
		if (!atcphy->hw->has_usb4)
			return -EOPNOTSUPP;
		fallthrough;
	case APPLE_ATCPHY_MODE_USB3:
	case APPLE_ATCPHY_MODE_USB3_DP:
	case APPLE_ATCPHY_MODE_DP:
		if (atcphy->hw->has_t8140_dp && atcphy_modes[mode].enable_dp_aux &&
		    !atcphy->dp_tunables)
			return -EOPNOTSUPP;
		/*
		 * Without the lane tunables the SuperSpeed lanes are not calibrated.
		 * T8122 four-lane DP programs its lanes itself and needs only the
		 * common calibration.
		 */
		if (!atcphy->ss_tunables &&
		    !(mode == APPLE_ATCPHY_MODE_DP && atcphy->hw->dp_t8122 &&
		      atcphy->dp_tunables))
			return -EOPNOTSUPP;
		break;
	case APPLE_ATCPHY_MODE_OFF:
	case APPLE_ATCPHY_MODE_USB2:
		break;
	}

	return 0;
}

static int atcphy_mux_switch(struct apple_atcphy *atcphy, enum atcphy_mode target_mode)
{
	int ret;

	lockdep_assert_held(&atcphy->lock);
	if (atcphy->t8103_dummy_pending) {
		ret = atcphy_park_live_t8103_pipehandler(atcphy);
		if (ret)
			return ret;
	}

	if (atcphy->mode == target_mode && !atcphy->mode_dirty) {
		atcphy->typec_mode = target_mode;
		/* A canceled lane change owes the old USB3 backend its reconnect. */
		if (atcphy->t8103_usb3_retry) {
			if (atcphy_modes[target_mode].pipehandler_state == ATCPHY_PIPEHANDLER_STATE_USB3)
				return atcphy_configure_pipehandler(atcphy, false);
			atcphy->t8103_usb3_retry = false;
		}
		/* Retry a failed PIPE setup without disturbing the configured lanes. */
		if (atcphy->fixed_usb2 && atcphy->host_active &&
		    (atcphy->pipe_state != atcphy_modes[target_mode].pipehandler_state ||
		     atcphy->pipe_pending)) {
			if (atcphy->pipe_pending) {
				ret = atcphy_park_pipehandler(atcphy, false);
				if (ret)
					return ret;
			}
			return atcphy_configure_pipehandler(atcphy, true);
		}
		return 0;
	}

	/* A re-probe after failed role shutdown must leave the live PIPE alone. */
	if (!atcphy->fixed_usb2 && target_mode == APPLE_ATCPHY_MODE_OFF &&
	    atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY)
		return -EBUSY;

	ret = atcphy_mux_check(atcphy, target_mode);
	if (ret)
		return ret;
	atcphy->typec_mode = target_mode;

	if (atcphy->fixed_usb2) {
		enum atcphy_pipehandler_state pipe_state;

		pipe_state = atcphy_modes[target_mode].pipehandler_state;
		/*
		 * The controller of a port behind a fixed hub stays up across mode
		 * changes, so the PIPE has to follow the mode from here rather than
		 * from a later set_mode call. Even another USB3 mode can move the
		 * backend to the other PMA when orientation changes. Park before
		 * rebuilding the lanes, then reconnect while dwc3 is in host mode.
		 * The controller's SuperSpeed port is off by now, see
		 * atcphy_mux_update().
		 */
		ret = atcphy_park_pipehandler(atcphy, false);
		if (ret)
			return ret;

		ret = atcphy_configure(atcphy, target_mode);
		if (ret)
			return ret;


		if (atcphy->host_active && atcphy->pipe_state != pipe_state)
			ret = atcphy_configure_pipehandler(atcphy, true);
		if (!ret && pipe_state != ATCPHY_PIPEHANDLER_STATE_USB3)
			atcphy->t8103_usb3_retry = false;

		return ret;
	}

	/* A failed USB3 handoff must finish parking before ordinary-port lane work. */
	if (atcphy->pipe_pending) {
		ret = atcphy_park_pipehandler(atcphy, false);
		if (ret)
			return ret;
	}

	/*
	 * If the pipehandler is still/already up here there's a bug somewhere so make sure to
	 * complain loudly. We can still try to switch modes and hope for the best though,
	 * in the worst case the hardware will fall back to USB2-only.
	 */
	if (atcphy->t8103_usb3_workaround) {
		if (atcphy->pipe_state == ATCPHY_PIPEHANDLER_STATE_USB3 &&
		    atcphy_modes[target_mode].pipehandler_state == ATCPHY_PIPEHANDLER_STATE_DUMMY) {
			ret = atcphy_park_live_t8103_pipehandler(atcphy);
			if (ret)
				return ret;
		}
		WARN_ON_ONCE(atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY &&
			     atcphy_modes[target_mode].pipehandler_state ==
				ATCPHY_PIPEHANDLER_STATE_DUMMY);
	} else {
		WARN_ON_ONCE(atcphy->pipe_state != ATCPHY_PIPEHANDLER_STATE_DUMMY);
	}
	ret = atcphy_configure(atcphy, target_mode);
	if (ret)
		return ret;
	if (atcphy->t8103_usb3_retry) {
		if (atcphy_modes[target_mode].pipehandler_state == ATCPHY_PIPEHANDLER_STATE_USB3)
			return atcphy_configure_pipehandler(atcphy, false);
		atcphy->t8103_usb3_retry = false;
	}
	return 0;
}

/*
 * Change the mode under the PHY lock. @port_on tells whether the SuperSpeed
 * port of a fixed-hub controller, taken down for the change, can come up again.
 */
static int atcphy_mux_set_locked(struct apple_atcphy *atcphy, enum atcphy_mode target_mode,
				 bool *port_on)
{
	int ret;

	KUNIT_STATIC_STUB_REDIRECT(atcphy_mux_set_locked, atcphy, target_mode, port_on);

	guard(mutex)(&atcphy->lock);
	ret = atcphy_mux_switch(atcphy, target_mode);
	*port_on = atcphy->usb3_port_off && atcphy_usb3_port_ready(atcphy);
	return ret;
}

static int atcphy_mux_update(struct apple_atcphy *atcphy, const struct typec_mux_state *state)
{
	struct atcphy_usb3_pm pm = {};
	enum atcphy_mode target_mode;
	bool in_mode, port_on, usb_err = true;
	int ret;

	scoped_guard(mutex, &atcphy->lock) {
		atcphy_restore_after_pd_off(atcphy);
		target_mode = atcphy_typec_target_mode(atcphy, state);
		/*
		 * Only a lane or PIPE change, or a SuperSpeed port still owed
		 * its power, needs the USB3 root hub below. Taking it means
		 * waiting for the hub's ongoing work, which includes a whole
		 * enumeration with its timeouts, and would hold up a
		 * DisplayPort update that changes neither.
		 */
		if (atcphy_mux_is_noop(atcphy, target_mode)) {
			/* Record matching requests for the next PHY init to restore. */
			atcphy->typec_mode = target_mode;
			return 0;
		}
		/* Turn down a mode the PHY cannot run before USB3 goes down for it. */
		ret = atcphy_mux_check(atcphy, target_mode);
		if (ret)
			return ret;
		in_mode = atcphy_mux_in_mode(atcphy, target_mode);
	}

	ret = atcphy_usb3_pm_get(atcphy, &pm);
	if (ret)
		goto out;

	/*
	 * The order in which dwc3 is taken down and brought up around a PHY
	 * change (see dwc3-apple.c), for the SuperSpeed side only: the
	 * controller's SuperSpeed port goes down before the PIPE is parked and
	 * the lanes are rebuilt, and comes up only once the PIPE is on the USB3
	 * backend again. It stays down after a failed change, and in a mode
	 * without USB3.
	 */
	ret = atcphy_usb3_port_off(atcphy, &pm);
	if (ret < 0) {
		/* typec_mux_set() skips EOPNOTSUPP, which would report a false success. */
		if (ret == -EOPNOTSUPP)
			ret = -ENODEV;
		goto put;
	}
	/* A port that someone else switched off is no longer the PHY's to restore. */
	scoped_guard(mutex, &atcphy->lock)
		atcphy->usb3_port_off = !ret;

	ret = atcphy_mux_set_locked(atcphy, target_mode, &port_on);
	usb_err = false;
	if (port_on) {
		int err = atcphy_usb3_port_on(atcphy, &pm);

		if (err >= 0) {
			scoped_guard(mutex, &atcphy->lock)
				atcphy->usb3_port_off = false;
		} else if (!ret || ret == -EOPNOTSUPP) {
			/* A port left off must not hide behind a code typec_mux_set() skips */
			ret = err;
			usb_err = true;
		}
	}
put:
	atcphy_usb3_pm_put(&pm);
out:
	/*
	 * With the lanes and the PIPE already as the mode needs them, only
	 * USB3 is missing, and that must not fail what may be a DisplayPort
	 * update. Either way, retry the port from here.
	 */
	if (ret && usb_err) {
		atcphy_usb3_port_retry(atcphy, true, ret);
		if (in_mode)
			return 0;
	}
	return ret;
}

static int atcphy_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	return atcphy_mux_update(typec_mux_get_drvdata(mux), state);
}

static void atcphy_typec_mux_unregister(void *data)
{
	typec_mux_unregister(data);
}

/*
 * A controller behind a fixed hub keeps running across mode changes, so the PHY
 * parks its PIPE with the controller's SuperSpeed port taken down. The PIPE
 * lock is still only acknowledged once the controller lets go of the PIPE, and
 * on a J700 that need not happen within the lock timeout while the partner
 * renegotiates. A Mac on the port did so through a burst of power elections
 * lasting several seconds, and the safe state of its data role swap then found
 * the PIPE held:
 *
 *   phy-apple-atc 40b000000.phy: Pipehandler lock not acked.
 *   phy-apple-atc 40b000000.phy: Failed to switch PIPE to dummy: -110
 *   sn201202x 1-0c: failed to place Type-C mux in safe mode: -110
 *
 * The failed change leaves the lanes dirty and the SuperSpeed port down, and
 * the next mux update that gets through puts both right. Such a timeout says
 * nothing about the state asked for, so tell the Type-C driver to keep applying
 * its whole state, the safe state and then the USB one, until it goes through.
 * Only a timeout of this PHY counts, not one of the display route that shares
 * the connector. The PHY's other timeouts, of its power and PLL polls, are
 * faults that retrying does not clear, but the Type-C driver stops retrying
 * once a hold-up has lasted two minutes, which bounds those as well.
 */
static bool atcphy_mux_transient_timeouts(const struct apple_atcphy *atcphy)
{
	return atcphy->fixed_usb2 && atcphy->hw->has_t8140_dp;
}

static int atcphy_probe_mux(struct apple_atcphy *atcphy)
{
	struct typec_mux_dev *mux;
	struct typec_mux_desc mux_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_mux_set,
		.safe_after_role = !atcphy->fixed_usb2,
		.transient_timeouts = atcphy_mux_transient_timeouts(atcphy),
	};

	mux = typec_mux_register(atcphy->dev, &mux_desc);
	if (IS_ERR(mux))
		return PTR_ERR(mux);

	return devm_add_action_or_reset(atcphy->dev, atcphy_typec_mux_unregister, mux);
}

static int atcphy_load_t8140_dp(struct apple_atcphy *atcphy)
{
	struct {
		const char *name;
		struct resource *resource;
		struct apple_tunable **out;
	} lists[] = {
		{ "apple,tunable-dp-common-pre", atcphy->res.core,
		  &atcphy->tunables.dp_common_pre },
		{ "apple,tunable-dp-lpdptx", atcphy->res.lpdptx,
		  &atcphy->tunables.dp_lpdptx },
		{ "apple,tunable-dp-common-post", atcphy->res.core,
		  &atcphy->tunables.dp_common_post },
	};
	if (!of_machine_is_compatible("apple,j700"))
		return -EOPNOTSUPP;
	for (unsigned int i = 0; i < ARRAY_SIZE(lists); i++) {
		struct property *property = of_find_property(atcphy->np, lists[i].name, NULL);

		if (!property || !atc_dp_list_length_valid(property->length))
			return -EINVAL;
		*lists[i].out = devm_apple_tunable_parse(atcphy->dev, atcphy->np,
							 lists[i].name, lists[i].resource);
		if (IS_ERR(*lists[i].out))
			return PTR_ERR(*lists[i].out);
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(atcphy->dp_presets); i++) {
		static const char * const properties[] = {
			"apple,dp-training-table", "apple,dp-training-table-rbr",
			"apple,dp-training-table-hbr", "apple,dp-training-table-hbr2",
			"apple,dp-training-table-hbr3",
		};
		int count = of_property_count_u32_elems(atcphy->np, properties[i]);
		int ret;

		if (i && !of_find_property(atcphy->np, properties[i], NULL))
			continue;
		if (count != ARRAY_SIZE(atcphy->dp_presets[i]))
			return -EINVAL;
		ret = of_property_read_u32_array(atcphy->np, properties[i],
						 atcphy->dp_presets[i], count);
		if (ret)
			return ret;
		atcphy->dp_presets_valid[i] = true;
	}

	atcphy->dp_tunables = true;
	return 0;
}

static int atcphy_load_tunables(struct apple_atcphy *atcphy)
{
	size_t tunable_count;
	bool ss_missing = false, common_missing = false;
	struct {
		const char *dt_name;
		struct apple_tunable **tunable;
		struct resource *res;
	} tunables[] = {
		{ "apple,tunable-axi2af", &atcphy->tunables.axi2af, atcphy->res.axi2af },
		{ "apple,tunable-common-a", &atcphy->tunables.common[0], atcphy->res.core },
		{ "apple,tunable-common-b", &atcphy->tunables.common[1], atcphy->res.core },
		{ "apple,tunable-lane0-usb", &atcphy->tunables.lane_usb3[0], atcphy->res.core },
		{ "apple,tunable-lane1-usb", &atcphy->tunables.lane_usb3[1], atcphy->res.core },
		{ "apple,tunable-lane0-cio", &atcphy->tunables.lane_usb4[0], atcphy->res.core },
		{ "apple,tunable-lane1-cio", &atcphy->tunables.lane_usb4[1], atcphy->res.core },
		{ "apple,tunable-lane0-dp", &atcphy->tunables.lane_dp[0], atcphy->res.core },
		{ "apple,tunable-lane1-dp", &atcphy->tunables.lane_dp[1], atcphy->res.core },
	};
	if (atcphy->hw->gen == ATCPHY_GENERATION_T8122 && !atcphy->hw->dp_t8122) {
		tunable_count = ARRAY_SIZE(tunables) - 2;
		atcphy->tunables.lane_dp[0] = NULL;
		atcphy->tunables.lane_dp[1] = NULL;
	} else
		tunable_count = ARRAY_SIZE(tunables);

	for (size_t i = 0; i < tunable_count; i++) {
		struct apple_tunable *tunable;
		bool lane_dp = tunables[i].tunable == &atcphy->tunables.lane_dp[0] ||
			       tunables[i].tunable == &atcphy->tunables.lane_dp[1];

		if (!atcphy->hw->has_usb4 &&
		    (tunables[i].tunable == &atcphy->tunables.lane_usb4[0] ||
		     tunables[i].tunable == &atcphy->tunables.lane_usb4[1])) {
			*tunables[i].tunable = NULL;
			continue;
		}
		/* a DisplayPort-only instance has no USB lanes and maybe no axi2af window */
		if (atcphy->dp_only && (!tunables[i].res ||
		    tunables[i].tunable == &atcphy->tunables.lane_usb3[0] ||
		    tunables[i].tunable == &atcphy->tunables.lane_usb3[1] ||
		    tunables[i].tunable == &atcphy->tunables.lane_usb4[0] ||
		    tunables[i].tunable == &atcphy->tunables.lane_usb4[1])) {
			*tunables[i].tunable = NULL;
			continue;
		}

		tunable = devm_apple_tunable_parse(atcphy->dev, atcphy->np, tunables[i].dt_name,
						   tunables[i].res);
		/*
		 * The T8122 DP lane sequence programs the lanes itself; the
		 * bootloader may add DP lane tunables, which are then applied too.
		 */
		if (atcphy->hw->dp_t8122 && lane_dp && tunable == ERR_PTR(-ENOENT)) {
			*tunables[i].tunable = NULL;
			continue;
		}
		if (IS_ERR(tunable)) {
			/*
			 * The bootloader drops every tunable of a T8122 PHY when
			 * one entry is missing from its source. Probe anyway, so
			 * that the PHY's consumers do not defer forever.
			 */
			if (PTR_ERR(tunable) != -ENOENT ||
			    !(atcphy->hw->optional_tunables || atcphy->hw->dp_t8122)) {
				dev_err(atcphy->dev, "Failed to read tunable %s: %ld\n",
					tunables[i].dt_name, PTR_ERR(tunable));
				return PTR_ERR(tunable);
			}
			/* The common-a tunables do not exist on this generation */
			if (tunables[i].tunable != &atcphy->tunables.common[0])
				ss_missing = true;
			if (tunables[i].tunable == &atcphy->tunables.axi2af ||
			    tunables[i].tunable == &atcphy->tunables.common[1])
				common_missing = true;
			tunable = NULL;
		}
		*tunables[i].tunable = tunable;
	}

	if (atcphy->hw->has_usb2phy_reg) {
		struct apple_tunable *tunable;

		tunable = devm_apple_tunable_parse(atcphy->dev, atcphy->np,
						   "apple,tunable-usb2phy-reg-dflt",
						   atcphy->res.usb2phy_reg);
		if (IS_ERR(tunable))
			return dev_err_probe(atcphy->dev, PTR_ERR(tunable),
					     "Failed to read tunable apple,tunable-usb2phy-reg-dflt\n");
		atcphy->tunables.usb2phy_reg_dflt = tunable;
	}

	if (atcphy->hw->has_t8140_dp) {
		int ret = atcphy_load_t8140_dp(atcphy);

		if (ret)
			dev_warn(atcphy->dev, "DisplayPort tunable bundle unavailable: %d\n", ret);
	}

	atcphy->ss_tunables = !ss_missing;
	if (atcphy->hw->dp_t8122)
		atcphy->dp_tunables = !common_missing;
	if (atcphy->dp_only && !atcphy->dp_tunables)
		dev_warn(atcphy->dev, "Calibration tunables missing, DisplayPort disabled\n");
	else if (ss_missing && atcphy->dp_tunables && !atcphy->dp_only)
		dev_warn(atcphy->dev,
			 "SuperSpeed tunables missing, USB2 only; DisplayPort stays available\n");
	else if (ss_missing && !atcphy->dp_only)
		dev_warn(atcphy->dev, "SuperSpeed tunables missing, USB2 only\n");

	return 0;
}

static int atcphy_map_resources(struct platform_device *pdev, struct apple_atcphy *atcphy)
{
	struct {
		const char *name;
		void __iomem **addr;
		struct resource **res;
	} resources[] = {
		{ "core", &atcphy->regs.core, &atcphy->res.core },
		{ "lpdptx", &atcphy->regs.lpdptx, &atcphy->res.lpdptx },
		{ "axi2af", &atcphy->regs.axi2af, &atcphy->res.axi2af },
		{ "usb2phy", &atcphy->regs.usb2phy, NULL },
		{ "pipehandler", &atcphy->regs.pipehandler, NULL },
	};
	struct resource *res;
	void __iomem *addr;

	/*
	 * A T8122 generation PHY described without the USB2 PHY and PIPE
	 * windows is DisplayPort-only, like the one behind the T6030 HDMI port.
	 * Only its core window is required then.
	 */
	if (atcphy->hw->dp_t8122 &&
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2phy") &&
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "pipehandler"))
		atcphy->dp_only = true;

	for (int i = 0; i < ARRAY_SIZE(resources); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, resources[i].name);
		if (!res && atcphy->dp_only && i)
			continue;
		addr = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(addr))
			return dev_err_probe(atcphy->dev, PTR_ERR(addr),
					     "Unable to map %s regs", resources[i].name);

		*resources[i].addr = addr;
		if (resources[i].res)
			*resources[i].res = res;
	}

	if (atcphy->hw->has_usb2phy_reg) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2phy-reg");
		addr = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(addr))
			return dev_err_probe(atcphy->dev, PTR_ERR(addr),
					     "Unable to map usb2phy-reg regs");

		atcphy->regs.usb2phy_reg = addr;
		atcphy->res.usb2phy_reg = res;
	}

	return 0;
}

/*
 * The USB2 pairs of a port can go to a fixed hub on the USB controller instead
 * of straight to the connector. Such a board describes the hub as a child of
 * the controller our USB3 port is linked to. There is then no cable event
 * that would bring the PHY up, and the controller is a host whatever happens
 * at the connectors, so the PHY has to provide USB2 from probe on.
 */
static bool atcphy_usb2_behind_fixed_hub(struct apple_atcphy *atcphy)
{
	struct device_node *ep __free(device_node) =
		of_graph_get_endpoint_by_regs(atcphy->np, 1, -1);
	struct device_node *ctrl __free(device_node) =
		ep ? of_graph_get_remote_port_parent(ep) : NULL;
	struct device_node *hub __free(device_node) =
		ctrl ? of_get_available_child_by_name(ctrl, "hub") : NULL;

	return hub;
}

/*
 * A DisplayPort-only instance has no USB controller to reset and no PIPE: put
 * the PHY in its off state and register the mux, switch and DP PHY.
 */
static int atcphy_probe_finalize_dp_only(struct apple_atcphy *atcphy)
{
	int ret;

	lockdep_assert_held(&atcphy->lock);

	atcphy_power_off(atcphy);

	ret = atcphy_probe_mux(atcphy);
	if (ret)
		return dev_err_probe(atcphy->dev, ret, "Probing mux failed");
	ret = atcphy_probe_switch(atcphy);
	if (ret)
		return dev_err_probe(atcphy->dev, ret, "Probing switch failed");
	ret = atcphy_probe_phy(atcphy);
	if (ret)
		return dev_err_probe(atcphy->dev, ret, "Probing phy failed");

	dev_info(atcphy->dev, "DisplayPort-only PHY\n");
	return 0;
}

static int atcphy_probe_finalize(struct apple_atcphy *atcphy)
{
	int ret;

	guard(mutex)(&atcphy->lock);

	if (atcphy->dp_only)
		return atcphy_probe_finalize_dp_only(atcphy);

	/* Reset dwc3 on probe, let dwc3 (consumer) deassert it */
	_atcphy_dwc3_reset_assert(atcphy);

	/* Reset atcphy to clear any state potentially left by the bootloader */
	atcphy_usb2_power_off(atcphy);
	atcphy_power_off(atcphy);
	atcphy_setup_pipehandler(atcphy);

	/*
	 * Powering only the USB2 PHY when the controller initialises its PHYs is
	 * not enough for a port behind a fixed hub: the common block, its clamps
	 * and the crossbar are set up by the mode change a cable event would
	 * bring. Establish the complete USB2 state now, while dwc3 is still held
	 * in reset.
	 */
	if (atcphy->fixed_usb2) {
		set32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		ret = atcphy_configure(atcphy, APPLE_ATCPHY_MODE_USB2);
		if (ret)
			return dev_err_probe(atcphy->dev, ret, "Failed to bring up USB2\n");
	}

	ret = atcphy_probe_rcdev(atcphy);
	if (ret) {
		ret = dev_err_probe(atcphy->dev, ret, "Probing rcdev failed");
		goto power_off;
	}
	ret = atcphy_probe_mux(atcphy);
	if (ret) {
		ret = dev_err_probe(atcphy->dev, ret, "Probing mux failed");
		goto power_off;
	}
	ret = atcphy_probe_switch(atcphy);
	if (ret) {
		ret = dev_err_probe(atcphy->dev, ret, "Probing switch failed");
		goto power_off;
	}
	ret = atcphy_probe_phy(atcphy);
	if (ret) {
		ret = dev_err_probe(atcphy->dev, ret, "Probing phy failed");
		goto power_off;
	}

	return 0;

power_off:
	if (atcphy->fixed_usb2)
		atcphy_configure(atcphy, APPLE_ATCPHY_MODE_OFF);
	return ret;
}

static void apple_atc_tunnel_wiring(struct apple_atcphy *atcphy)
{
	struct device_node *connector __free(device_node) = NULL;
	struct device_node *candidate;

	if (!apple_atc_t602x_qualified(of_root))
		return;
	connector = of_graph_get_remote_node(atcphy->np, 0, -1);
	atcphy->tunnel_dual_stream = apple_dp_tunnel_dual_stream(connector);
	if (atcphy->tunnel_dual_stream) {
		atcphy->tunnel_routes_present = true;
		return;
	}
	/* Preserve EINVAL for invalid cores when qualified routes exist elsewhere. */
	for_each_compatible_node(candidate, NULL, "usb-c-connector") {
		if (!apple_dp_tunnel_dual_stream(candidate))
			continue;
		atcphy->tunnel_routes_present = true;
		of_node_put(candidate);
		break;
	}
}

/*
 * Called by the PM domain with its lock held, which is a spinlock for these
 * domains: only record the power-down, atcphy_restore_after_pd_off() acts on it.
 */
static int atcphy_pd_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct apple_atcphy *atcphy = container_of(nb, struct apple_atcphy, pd_nb);

	if (action == GENPD_NOTIFY_OFF)
		WRITE_ONCE(atcphy->pd_was_off, true);
	return NOTIFY_OK;
}

static void atcphy_pd_notifier_remove(void *data)
{
	dev_pm_genpd_remove_notifier(data);
}

static void atcphy_probe_pd_notifier(struct apple_atcphy *atcphy)
{
	int ret;

	if (!atcphy->hw->restore_after_pd_off)
		return;

	atcphy->pd_nb.notifier_call = atcphy_pd_notify;
	ret = dev_pm_genpd_add_notifier(atcphy->dev, &atcphy->pd_nb);
	if (!ret)
		ret = devm_add_action_or_reset(atcphy->dev, atcphy_pd_notifier_remove,
					       atcphy->dev);
	/* without a single PM domain nothing switches the PHY off underneath us */
	if (ret == -ENODEV || ret == -EOPNOTSUPP)
		dev_dbg(atcphy->dev, "no PM domain to follow: %d\n", ret);
	else if (ret)
		dev_warn(atcphy->dev,
			 "Cannot follow the PM domain (%d), PHY state is not restored after it was off\n",
			 ret);
}

/* The T8103 PHY compatible is also a fallback on unqualified M1/M2 variants. */
static bool atcphy_has_t8103_usb3_workaround(const struct atcphy_hw *hw,
					   const struct device_node *root)
{
	return hw->gen == ATCPHY_GENERATION_T8103 &&
	       of_device_is_compatible(root, "apple,t8103");
}

static int atcphy_probe(struct platform_device *pdev)
{
	struct apple_atcphy *atcphy;
	struct device *dev = &pdev->dev;
	int ret;

	atcphy = devm_kzalloc(&pdev->dev, sizeof(*atcphy), GFP_KERNEL);
	if (!atcphy)
		return -ENOMEM;

	atcphy->hw = of_device_get_match_data(&pdev->dev);
	if (!atcphy->hw)
		return -EINVAL;
	atcphy->t8103_usb3_workaround = atcphy_has_t8103_usb3_workaround(atcphy->hw, of_root);

	atcphy->dev = dev;
	atcphy->np = dev->of_node;
	apple_atc_tunnel_wiring(atcphy);
	mutex_init(&atcphy->lock);
	platform_set_drvdata(pdev, atcphy);
	ret = devm_delayed_work_autocancel(dev, &atcphy->usb3_port_work, atcphy_usb3_port_work);
	if (ret)
		return ret;

	ret = atcphy_map_resources(pdev, atcphy);
	if (ret)
		return ret;
	ret = atcphy_load_tunables(atcphy);
	if (ret)
		return ret;

	atcphy->mode = APPLE_ATCPHY_MODE_OFF;
	atcphy->pipe_state = ATCPHY_PIPEHANDLER_STATE_DUMMY;
	atcphy->fixed_usb2 = atcphy_usb2_behind_fixed_hub(atcphy);
	atcphy->typec_mode = APPLE_ATCPHY_MODE_USB2;
	atcphy_probe_pd_notifier(atcphy);

	return atcphy_probe_finalize(atcphy);
}

/*
 * Nothing to save on the way down: whatever the PHY runs stays up when its
 * domain stays on. If the domain was switched off, restore the state here
 * unless a PHY operation already did.
 */
static int atcphy_resume(struct device *dev)
{
	struct apple_atcphy *atcphy = dev_get_drvdata(dev);

	guard(mutex)(&atcphy->lock);
	atcphy_restore_after_pd_off(atcphy);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(atcphy_pm_ops, NULL, atcphy_resume);

static const struct atcphy_hw atcphy_hw_t8103 = {
	.gen = ATCPHY_GENERATION_T8103,
	.aciophy_lane_mode = ACIOPHY_LANE_MODE_T8103,
	.aciophy_crossbar = ACIOPHY_CROSSBAR_T8103,
	.configure_post_tunable = atcphy_configure_post_tunable_t8103,
	.pipe_mux_clk = PIPEHANDLER_MUX_CTRL_CLK_T8103,
	.pipe_mux_data = PIPEHANDLER_MUX_CTRL_DATA_T8103,
	.has_usb4 = true,
};

static const struct atcphy_hw atcphy_hw_t8122 = {
	.gen = ATCPHY_GENERATION_T8122,
	.aciophy_lane_mode = ACIOPHY_LANE_MODE_T8122,
	.aciophy_crossbar = ACIOPHY_CROSSBAR_T8122,
	.configure_post_tunable = atcphy_configure_post_tunable_t8122,
	.pipe_mux_clk = PIPEHANDLER_MUX_CTRL_CLK_T8103,
	.pipe_mux_data = PIPEHANDLER_MUX_CTRL_DATA_T8103,
	.has_usb4 = true,
	.dp_t8122 = true,
	.park_pipe_unlocked = true,
	.park_dummy_phy = true,
	.restore_after_pd_off = true,
};

static const struct atcphy_hw atcphy_hw_t8140 = {
	.configure_post_tunable = atcphy_configure_post_tunable_all_modes,
	.gen = ATCPHY_GENERATION_T8122,
	.aciophy_lane_mode = ACIOPHY_LANE_MODE_T8122,
	.aciophy_crossbar = ACIOPHY_CROSSBAR_T8122,
	.pipe_mux_clk = PIPEHANDLER_MUX_CTRL_CLK_T8103,
	.pipe_mux_data = PIPEHANDLER_MUX_CTRL_DATA_T8103,
	.has_usb2phy_reg = true,
	.has_t8140_dp = true,
	.optional_tunables = true,
};

static const struct atcphy_hw atcphy_hw_t8142 = {
	.configure_post_tunable = atcphy_configure_post_tunable_all_modes,
	.gen = ATCPHY_GENERATION_T8122,
	.aciophy_lane_mode = ACIOPHY_LANE_MODE_T8122,
	.aciophy_crossbar = ACIOPHY_CROSSBAR_T8122,
	.pipe_mux_clk = PIPEHANDLER_MUX_CTRL_CLK_T8142,
	.pipe_mux_data = PIPEHANDLER_MUX_CTRL_DATA_T8142,
	.has_usb4 = true,
};

static const struct of_device_id atcphy_match[] = {
	{ .compatible = "apple,t8103-atcphy", .data = &atcphy_hw_t8103 },
	{ .compatible = "apple,t8122-atcphy", .data = &atcphy_hw_t8122 },
	{ .compatible = "apple,t8142-atcphy", .data = &atcphy_hw_t8142 },
	{ .compatible = "apple,t8140-atcphy", .data = &atcphy_hw_t8140 },
	{},
};
MODULE_DEVICE_TABLE(of, atcphy_match);

static struct platform_driver atcphy_driver = {
	.driver = {
		.name = "phy-apple-atc",
		.of_match_table = atcphy_match,
		.pm = pm_sleep_ptr(&atcphy_pm_ops),
	},
	.probe = atcphy_probe,
};
module_platform_driver(atcphy_driver);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("Apple Type-C PHY driver");
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_PHY_APPLE_ATC_KUNIT_TEST)
#include "atc-test.c"
#include "atc-t8122-dp-test.c"
#include "atc-tunnel-test.c"
#endif
