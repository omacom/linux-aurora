// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple system-on-chips.
 *
 * The HW is ECAM compliant, so once the controller is initialized,
 * the driver mostly deals MSI mapping and handling of per-port
 * interrupts (INTx, management and error signals).
 *
 * Initialization requires enabling power and clocks, along with a
 * number of register pokes.
 *
 * Copyright (C) 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Copyright (C) 2021 Google LLC
 * Copyright (C) 2021 Corellium LLC
 * Copyright (C) 2021 Mark Kettenis <kettenis@openbsd.org>
 *
 * Author: Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Author: Marc Zyngier <maz@kernel.org>
 */

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/irqdomain.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mfd/macsmc.h>
#include <linux/msi.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pci-apple.h>
#include <linux/soc/apple/pci-apple-piodma.h>
#include <linux/pci-ecam.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/rcupdate.h>
#include <linux/reset.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/dart-cio.h>
#include <linux/soc/apple/dart-sleep.h>
#include <linux/soc/apple/radio-sleep.h>
#include <linux/soc/apple/sleep.h>
#include <linux/soc/apple/t8132.h>
#include <linux/soc/apple/tunable.h>
#include <linux/suspend.h>
#include <linux/blkdev.h>

#include "../pci.h"
#include "pci-host-common.h"
#include "pcie-apple-t8140-bootstrap.h"

static ATOMIC_NOTIFIER_HEAD(apple_pcie_tunnel_notifiers);

int apple_pcie_tunnel_register_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&apple_pcie_tunnel_notifiers, nb);
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_register_notifier);

void apple_pcie_tunnel_unregister_notifier(struct notifier_block *nb)
{
	atomic_notifier_chain_unregister(&apple_pcie_tunnel_notifiers, nb);
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_unregister_notifier);

static bool s2idle_keep_link = true;
module_param(s2idle_keep_link, bool, 0644);
MODULE_PARM_DESC(s2idle_keep_link, "Keep tunneled links up through suspend-to-idle");

/* Lets the NHI take a tunneled PCIe-C port through a deep sleep. */
static bool cio_s2r = true;
module_param(cio_s2r, bool, 0400);
MODULE_PARM_DESC(cio_s2r, "Enable NHI-owned J713 PCIe-C system sleep (default: on)");

static int link_up_timeout = 500;
/* Fundamental resets of a conventional port whose endpoint does not train. */
#define LINK_RESET_RETRIES	2
module_param(link_up_timeout, int, 0644);
MODULE_PARM_DESC(link_up_timeout, "PCIe link training timeout in milliseconds");

/*
 * A T6050 port Linux powers and trains (the J714s SD card reader's, port 1) after a firmware
 * sleep: the reader's power, the port's reset table and enable, the lane clocks, PERST#
 * released, the LTSSM, and the link is up again. macOS brings it back at every wake, after the
 * radio's port and after its display processors have started, alongside the rest of the wake.
 * 2, the default, brings it back once the resume is over, from a work, with its functions;
 * the bring-up's waits (the reader's power-up and the time after the training) then hold up
 * nothing else. 1 brings it back in this host's early resume, the first phase after the one
 * the radio's port comes back in, and looks for its functions once the resume is over: every
 * device of the resume's later phases, the display processors among them, waits for the port.
 * 0 leaves it parked, with no functions, until a restart.
 */
static int t6050_sd_port_wake = 2;
module_param(t6050_sd_port_wake, int, 0644);
MODULE_PARM_DESC(t6050_sd_port_wake,
		 "T6050: the SD card reader's port after a firmware sleep: back once the resume is over (2, default), back in the early resume (1), or parked until a restart (0)");

/* T8103 (original M1) and related SoCs */
#define CORE_RC_PHYIF_CTL		0x00024
#define   CORE_RC_PHYIF_CTL_RUN		BIT(0)
#define CORE_RC_PHYIF_STAT		0x00028
#define   CORE_RC_PHYIF_STAT_REFCLK	BIT(4)
#define CORE_RC_CTL			0x00050
#define   CORE_RC_CTL_RUN		BIT(0)
#define CORE_RC_STAT			0x00058
#define   CORE_RC_STAT_READY		BIT(0)
#define CORE_FABRIC_STAT		0x04000
#define   CORE_FABRIC_STAT_MASK		0x001F001F

#define CORE_PHY_DEFAULT_BASE(port)	(0x84000 + 0x4000 * (port))

#define PHY_LANE_CFG			0x00000
#define   PHY_LANE_CFG_REFCLK0REQ	BIT(0)
#define   PHY_LANE_CFG_REFCLK1REQ	BIT(1)
#define   PHY_LANE_CFG_REFCLK0ACK	BIT(2)
#define   PHY_LANE_CFG_REFCLK1ACK	BIT(3)
#define   PHY_LANE_CFG_REFBUF_OFF	BIT(4)
#define   PHY_LANE_CFG_REFCLKEN		(BIT(9) | BIT(10))
#define   PHY_LANE_CFG_REFCLKCGEN	(BIT(30) | BIT(31))
/* Bit 31 of the pair: the only one a trace of macOS shows cleared, after PERST# is asserted */
#define   PHY_LANE_CFG_REFCLKCGEN_B31	BIT(31)
#define PHY_LANE_CTL			0x00004
#define   PHY_LANE_CTL_CFGACC		BIT(15)

#define PORT_LTSSMCTL			0x00080
#define   PORT_LTSSMCTL_START		BIT(0)
/* T6050: a trace of macOS shows 0x11 written here to send PME_Turn_Off */
#define PORT_PME_TURN_OFF		0x00088
/* Reads back 0 within a few reads of the write (the trace reads 0x1, then 0x10) */
#define   PORT_PME_TURN_OFF_SEND	BIT(0)
/* Written with bit 0 and still set afterwards; meaning not known */
#define   PORT_PME_TURN_OFF_BIT4	BIT(4)
/*
 * T6050 PCIe-C, the root port of a USB4 router's PCIe tunnel: macOS sends PME_Turn_Off
 * through this register of the port, with the same two bits, waits for the send bit to
 * clear and reads the link in L2 before it stops the link. It writes nothing at
 * PORT_PME_TURN_OFF on that port. The port's tunnel control words sit elsewhere on this
 * block as well (PORT_T6050_TUNCTRL).
 */
#define PORT_T6050_PCIEC_PME_TURN_OFF	0x0008c
#define PORT_INTSTAT			0x00100
#define   PORT_INT_TUNNEL_ERR		31
#define   PORT_INT_CPL_TIMEOUT		23
#define   PORT_INT_RID2SID_MAPERR	22
#define   PORT_INT_CPL_ABORT		21
#define   PORT_INT_MSI_BAD_DATA		19
#define   PORT_INT_MSI_ERR		18
#define   PORT_INT_REQADDR_GT32		17
#define   PORT_INT_AF_TIMEOUT		15
#define   PORT_INT_LINK_DOWN		14
#define   PORT_INT_LINK_UP		12
#define   PORT_INT_LINK_BWMGMT		11
#define   PORT_INT_AER_MASK		(15 << 4)
#define   PORT_INT_PORT_ERR		4
#define   PORT_INT_INTx(i)		i
#define   PORT_INT_INTx_MASK		15
#define PORT_INTMSK			0x00104
#define PORT_INTMSKSET			0x00108
#define PORT_INTMSKCLR			0x0010c
#define PORT_MSICFG			0x00124
#define   PORT_MSICFG_EN		BIT(0)
#define   PORT_MSICFG_L2MSINUM_SHIFT	4
#define PORT_MSIBASE			0x00128
#define   PORT_MSIBASE_1_SHIFT		16
#define PORT_MSIADDR			0x00168
#define PORT_LINKSTS			0x00208
#define   PORT_LINKSTS_UP		BIT(0)
#define   PORT_LINKSTS_BUSY		BIT(2)
/* In traces of macOS: 0x9b00134d for a port that reached L2, 0x9b001001 for one that did not */
#define   PORT_LINKSTS_L2		BIT(6)
#define PORT_LINKCMDSTS			0x00210
#define PORT_OUTS_NPREQS		0x00284
#define   PORT_OUTS_NPREQS_REQ		BIT(24)
#define   PORT_OUTS_NPREQS_CPL		BIT(16)
#define PORT_RXWR_FIFO			0x00288
#define   PORT_RXWR_FIFO_HDR		GENMASK(15, 10)
#define   PORT_RXWR_FIFO_DATA		GENMASK(9, 0)
#define PORT_RXRD_FIFO			0x0028C
#define   PORT_RXRD_FIFO_REQ		GENMASK(6, 0)
#define PORT_OUTS_CPLS			0x00290
#define   PORT_OUTS_CPLS_SHRD		GENMASK(14, 8)
#define   PORT_OUTS_CPLS_WAIT		GENMASK(6, 0)
#define PORT_APPCLK			0x00800
#define   PORT_APPCLK_EN		BIT(0)
#define   PORT_APPCLK_CGDIS		BIT(8)
#define PORT_STATUS			0x00804
#define   PORT_STATUS_READY		BIT(0)
#define PORT_REFCLK			0x00810
#define   PORT_REFCLK_EN		BIT(0)
#define   PORT_REFCLK_CGDIS		BIT(8)
#define PORT_PERST			0x00814
#define   PORT_PERST_OFF		BIT(0)
#define PORT_RID2SID			0x00828
#define   PORT_RID2SID_VALID		BIT(31)
#define   PORT_RID2SID_SID_SHIFT	16
#define   PORT_RID2SID_BUS_SHIFT	8
#define   PORT_RID2SID_DEV_SHIFT	3
#define   PORT_RID2SID_FUNC_SHIFT	0
#define PORT_OUTS_PREQS_HDR		0x00980
#define   PORT_OUTS_PREQS_HDR_MASK	GENMASK(9, 0)
#define PORT_OUTS_PREQS_DATA		0x00984
#define   PORT_OUTS_PREQS_DATA_MASK	GENMASK(15, 0)
#define PORT_TUNCTRL			0x00988
#define   PORT_TUNCTRL_PERST_ON		BIT(0)
#define   PORT_TUNCTRL_PERST_ACK_REQ	BIT(1)
#define PORT_TUNSTAT			0x0098c
#define   PORT_TUNSTAT_PERST_ON		BIT(0)
#define   PORT_TUNSTAT_PERST_ACK_PEND	BIT(1)
#define PORT_PREFMEM_ENABLE		0x00994
#define PORT_COUNTER_CTRL		0x04020
#define   PORT_COUNTER_ENABLE		0x3

#define PCIEC_INTR2AXI_CTRL		0x00080
#define   PCIEC_INTR2AXI_ENABLE		BIT(0)

/* T602x (M2-pro and co) */
#define PORT_T602X_MSIADDR	0x016c
#define PORT_T602X_MSIADDR_HI	0x0170
#define PORT_T602X_PERST	0x082c
/* T8152: lane PIPE reset is released by clearing this after PERST#. */
#define   PORT_T8152_PIPE_RST	BIT(16)
#define PORT_T8152_PIPE_RST_STS	0x00a8
#define   PORT_T8152_PIPE_RST_STS_LANES	GENMASK(3, 0)
#define PORT_T602X_RID2SID	0x3000
#define PORT_T602X_MSIMAP	0x3800

#define PORT_MSIMAP_ENABLE	BIT(31)
#define PORT_MSIMAP_TARGET	GENMASK(7, 0)

/* T6050: the root port's port logic, laid out as DesignWare's */
#define RC_T6050_PORT_LINK_CTRL		0x710
#define   RC_T6050_LINK_MODE		GENMASK(21, 16)
#define RC_T6050_PORT_DEBUG0		0x728
#define   RC_T6050_LTSSM_STATE		GENMASK(5, 0)
#define   RC_T6050_LTSSM_L1_IDLE	0x14
#define RC_T6050_LINK_WIDTH_SPEED_CTRL	0x80c
#define   RC_T6050_LINK_WIDTH		GENMASK(12, 8)
#define   RC_T6050_SPEED_CHANGE		BIT(17)
#define RC_T6050_GEN3_RELATED		0x890
#define   RC_T6050_RATE_SHADOW_SEL	GENMASK(25, 24)
#define RC_T6050_MISC_CONTROL_1		0x8bc
#define   RC_T6050_DBI_RO_WR_EN		BIT(0)

/*
 * T6050 PCIe-C: the 256-entry RID2SID table runs over the T8103 tunnel block,
 * which moves up by 0x700 (posted request counters, TUNCTRL and TUNSTAT).
 */
#define PORT_T6050_TUNCTRL	0x1088

/* T6050 PCIe-C root port config space: the MPS-dependent link timers */
#define PCIEC_RP_DEVCAP		0x074
#define PCIEC_RP_DEVCTL		0x078
#define PCIEC_RP_ACK_LATENCY	0x700
#define PCIEC_RP_FC_TIMER	0x73c
#define   PCIEC_RP_FC_TIMER_KEEP	0x6000ffff
#define   PCIEC_RP_FC_TIMER_LIMIT	GENMASK(28, 16)
#define   PCIEC_RP_FC_TIMER_EN		BIT(31)

/*
 * The doorbell address must be in the bottom 4GB because the base register
 * is only 32 bits wide. Exclude it from the IOVA range and share the same
 * configured address with the DART driver.
 */
#define DOORBELL_ADDR		CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR

struct hw_info {
	u32 phy_lane_ctl;
	u32 port_msiaddr;
	u32 port_msiaddr_hi;
	u32 port_refclk;
	u32 port_perst;
	u32 port_rid2sid;
	u32 port_msimap;
	u32 port_tunctrl;
	u32 max_rid2sid;
	u32 pwren_off_ms;
	u32 pwren_on_ms;
	bool sid_indexed_rid2sid;
	bool retain_clocks;
	bool adopt_link;
	bool root_bus_only;
	bool tunneled;
	bool tunnel_intr2axi;
	bool t8132_reset;
	bool msimap_by_vector;
	/*
	 * In suspend-to-idle a tunnel host whose power domain is not always-on
	 * keeps its link while the domain stays on, see apple_pcie_hold_domains().
	 */
	bool s2idle_hold_domain;
	/* Port init clears REFBUF_OFF in the refclk buffer control. */
	bool refbuf_off;
	/*
	 * Port enable: after PERST#, wait for the
	 * lane PIPE reset status, then release PORT_T8152_PIPE_RST.
	 */
	bool pipe_rst;
	bool no_port_reset;
	bool mps_timers;
};

static const struct hw_info t8103_hw = {
	.phy_lane_ctl		= PHY_LANE_CTL,
	.port_msiaddr		= PORT_MSIADDR,
	.port_msiaddr_hi	= 0,
	.port_refclk		= PORT_REFCLK,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= 0,
	.max_rid2sid		= 64,
};

static const struct hw_info t602x_hw = {
	.phy_lane_ctl		= 0,
	.port_msiaddr		= PORT_T602X_MSIADDR,
	.port_msiaddr_hi	= PORT_T602X_MSIADDR_HI,
	.port_refclk		= 0,
	.port_perst		= PORT_T602X_PERST,
	.port_rid2sid		= PORT_T602X_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	/* 16 on t602x, guess for autodetect on future HW */
	.max_rid2sid		= 512,
};

/*
 * T8140 uses the newer port register layout and a SID-indexed RID table.
 * Keep the PHY and application clock policy established by the bootloader;
 * runtime clock gating has not been qualified on this controller.
 */
static const struct hw_info t8140_hw = {
	.port_msiaddr		= PORT_T602X_MSIADDR,
	.port_perst		= PORT_T602X_PERST,
	.port_rid2sid		= PORT_T602X_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.max_rid2sid		= 19,
	.pwren_off_ms		= 2,
	.pwren_on_ms		= 150,
	.sid_indexed_rid2sid	= true,
	.retain_clocks		= true,
	.root_bus_only		= true,
};

/* T8152 (M6) apcie: T602x port layout, refclk buffer powered down by iBoot. */
static const struct hw_info t8152_hw = {
	.phy_lane_ctl		= 0,
	.port_msiaddr		= PORT_T602X_MSIADDR,
	.port_msiaddr_hi	= PORT_T602X_MSIADDR_HI,
	.port_refclk		= 0,
	.port_perst		= PORT_T602X_PERST,
	.port_rid2sid		= PORT_T602X_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.max_rid2sid		= 512,
	.refbuf_off		= true,
	.pipe_rst		= true,
};

/*
 * Bare-metal T6050 adoption only. Firmware owns power, PHY, reset and training.
 * The 64-entry route table is never probed with test writes.
 */
static const struct hw_info t6050_hw = {
	.port_msiaddr		= PORT_T602X_MSIADDR,
	.port_msiaddr_hi		= PORT_T602X_MSIADDR_HI,
	.port_perst		= PORT_T602X_PERST,
	.port_rid2sid		= PORT_T602X_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.max_rid2sid		= 64,
	.retain_clocks		= true,
	.adopt_link		= true,
};

#define T6050_ROUTE(rid, sid)	(PORT_RID2SID_VALID | \
				 ((sid) << PORT_RID2SID_SID_SHIFT) | (rid))

static const struct hw_info t8103_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.max_rid2sid		= 64,
	.tunneled		= true,
};

/* T600x and T8112 use Intr2AXI instead of the T8103 oe-fabric. */
static const struct hw_info t8112_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.max_rid2sid		= 64,
	.tunneled		= true,
	.tunnel_intr2axi		= true,
};

/*
 * t8122 PCIe-C keeps the t8103 doorbell and RID2SID registers, but MSI
 * delivery uses the t602x map. The map is indexed by the MSI data, and
 * the dock bridge's vectors start at 32.
 */
static const struct hw_info t8122_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.max_rid2sid		= 64,
	.tunneled		= true,
};

/*
 * T8132 PCIe-C has 256 RID2SID entries, which run over the old tunnel reset
 * block: TUNCTRL/TUNSTAT move to 0x1088/0x108c. The MSI address takes its
 * high word at 0x16c.
 *
 * apple,pciec-kernel-init brings its ports up with the t8103 sequence, the
 * oe-fabric tunables included, so it is not a tunnel_intr2axi controller:
 * its Intr2AXI bridge is only pulsed, by ACIO and around the port enable.
 */
static const struct hw_info t8132_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_msiaddr_hi	= PORT_MSIADDR + 4,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.port_tunctrl		= 0x1088,
	.max_rid2sid		= 256,
	.tunneled		= true,
	.t8132_reset		= true,
	.msimap_by_vector	= true,
	.s2idle_hold_domain	= true,
};

static const struct hw_info t6040_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_msiaddr_hi	= PORT_MSIADDR + 4,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.port_tunctrl		= 0x1088,
	.max_rid2sid		= 256,
	.sid_indexed_rid2sid	= true,
	.tunneled		= true,
	.t8132_reset		= true,
	.msimap_by_vector	= true,
	.s2idle_hold_domain	= true,
};

/*
 * T6050 PCIe-C keeps the t8103 doorbell, PERST and RID2SID registers, with
 * the doorbell's high word at 0x16c and 256 RID2SID entries. MSIs go through
 * the t602x map, indexed by the vector the endpoint sends: the bridge's slice
 * of the shared window, not a local index. The port has no reset sequence of
 * its own, and the root port's ACK latency and flow control timers follow
 * the max payload size.
 *
 * Nothing hands the port over: apple,pciec-kernel-init brings it up with the
 * T600x Intr2AXI sequence.
 */
static const struct hw_info t6050_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_msiaddr_hi	= PORT_MSIADDR + 4,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	.port_tunctrl		= PORT_T6050_TUNCTRL,
	.max_rid2sid		= 256,
	.tunneled		= true,
	.tunnel_intr2axi	= true,
	.no_port_reset		= true,
	.msimap_by_vector	= true,
	.mps_timers		= true,
};

static const struct of_device_id apple_pcie_of_match[];

/**
 * apple_pcie_tunnel_needs_cold_init() - whether the kernel brings up a port
 * @np: PCIe-C host node
 *
 * DT selects kernel initialization; match data selects the hardware sequence.
 * Never reset a qualified firmware handoff. Intr2AXI controllers also need
 * DART tunables before ACIO may populate their cable-powered children.
 */
bool apple_pcie_tunnel_needs_cold_init(struct device_node *np)
{
	const struct of_device_id *match = of_match_node(apple_pcie_of_match, np);
	struct of_phandle_args iommu_spec = {};
	const struct hw_info *hw;
	bool tunables;
	u32 status;

	if (!match || !of_property_read_bool(np, "apple,pciec-kernel-init"))
		return false;
	hw = match->data;
	if (!hw->tunneled)
		return false;
	if (!of_property_read_u32(np, "apple,pciec-preinit-status", &status) &&
	    status == 1)
		return false;
	if (!hw->tunnel_intr2axi)
		return true;

	if (of_map_iommu_id(np, 0, &iommu_spec) || !iommu_spec.np)
		return false;
	tunables = of_property_present(iommu_spec.np, "apple,tunable");
	if (!tunables)
		pr_warn_once("%pOF: no PCIe-C DART tunables, not cold-initializing\n",
			     iommu_spec.np);
	of_node_put(iommu_spec.np);
	return tunables;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_needs_cold_init);

struct apple_pcie_pd {
	struct device		*dev;
	struct notifier_block	nb;
	unsigned int		vetoed;
};

struct apple_radio_sleep;

#define APPLE_PCIE_T6050_WARM_WRITES	4

struct apple_pcie {
	struct mutex		lock;
	struct device		*dev;
	void __iomem            *base;
	void __iomem		*fabric_base;
	void __iomem		*debug_base;
	void __iomem		*intr2axi_base;
	void __iomem		*oe_fabric_base;
	void __iomem		*early_cfg;
	void __iomem		*t6050_phy;	/* shared PHY, mapped by a cold train */
	/* Firmware sleep of an adopted host: its DART, three more port blocks, the port taken down */
	struct device		*t6050_dart;
	/* The port a boot loader trained and the firmware sleep shuts down and brings back */
	struct apple_pcie_port	*t6050_port;
	bool			t6050_down;
	bool			t6050_lost;
	/* Ports Linux powers and trains, parked over a firmware sleep: their functions removed */
	struct notifier_block	t6050_pm_nb;
	/* Those ports back, and their functions looked for, once the resume is over */
	struct work_struct	t6050_unpark_work;
	/* The endpoint's warm reset, given by this host while the port is down */
	struct platform_device	*t6050_smc;
	u32			t6050_warm_key;
	u32			t6050_warm_writes[2 * APPLE_PCIE_T6050_WARM_WRITES];
	unsigned int		t6050_warm_count;
	bool			t6050_warm_done;
	/* When this wake's bring-up of that port had the link up and its 100 ms over; 0 before */
	ktime_t			t6050_link_up;
	/* The endpoint requests a warm reset after link shutdown; no radio wake is armed. */
	bool			t6050_reset_before_sleep;
	unsigned long		t6050_parked;
	/* Parked ports whose registers a firmware sleep has taken: not even their root port answers */
	unsigned long		t6050_cold_lost;
	/* Ports back up whose functions are still to be looked for, and whose root port is restored */
	unsigned long		t6050_rescan;
	unsigned long		t6050_rp_restored;
	/* This wake brought the radio's port back, its endpoint reset: the port stays up from here */
	bool			t6050_woke;
	struct pci_config_window *cfg;
	struct apple_tunable	*rc_tunable;
	struct apple_tunable	*fabric_tunable;
	struct apple_tunable	*debug_tunable;
	struct apple_tunable	*oe_fabric_tunable;
	bool			power_retained;
	bool			kernel_init;
	bool			tunnel_cold_init;
	unsigned long		tunnel_ready_at;
	struct notifier_block	tunnel_pci_nb;
	bool			bus_stopped;
	bool			reset_on_resume;
	bool			link_kept;
	struct dev_pm_domain_list *pd_list;
	struct apple_pcie_pd	*pds;
	unsigned int		num_pds;
	unsigned int		pd_held;
	bool			resume_failed; /* cleared only by teardown and reprobe */
	struct reset_control	*reset;
	u16			managed_sids;
	struct apple_radio_sleep *radio;
	struct apple_dart_cio *cio_dart;
	/* orders cio_changed against config cycles already past map_bus() */
	raw_spinlock_t cio_cfg_lock;
	bool cio_prepared, cio_changed, cio_stopped, cio_port_reset, cio_lost;
	unsigned int radio_writers;
	bool radio_closed;
	bool			neo_config_ready;
	u8			neo_secondary_bus;
	struct device		*piodma_supplier;
	/* What it takes to come back from a deep sleep, if the loader gave it. */
	struct apple_pcie_neo_cold *neo_cold;
	struct notifier_block	neo_pm_nb;
	bool			neo_sleep_armed;
	struct work_struct	neo_recover_work;
	bool			neo_recover;
	/* The link and the functions back after a sleep, while the rest resumes */
	struct work_struct	neo_radio_work;
	struct completion	neo_noirq_done;
	bool			neo_radio_waking;
	u8			neo_fn_pm_cap[2];
	/* The functions' state from the last sleep that found them answering */
	struct pci_saved_state	*neo_fn_state[2];
	/* Their MSI-X, set aside while they're out of reach */
	bool			neo_fn_msix[2];
	bool			neo_rc_down;
	/* Down already when the sleep began: a failed wake left it unprogrammed */
	bool			neo_rc_was_down;
	const struct hw_info	*hw;
	unsigned long		*bitmap;
	struct list_head	ports;
	struct completion	event;
	struct irq_fwspec	fwspec;
	struct irq_domain	*msi_domain;
	u32			nvecs;
	u32			msi_vector_base;
};

struct apple_pcie_port {
	raw_spinlock_t		lock;
	raw_spinlock_t		irq_lock;
	struct apple_pcie	*pcie;
	struct device_node	*np;
	void __iomem		*base;
	void __iomem		*phy;
	/* PERST# of a conventional or adopted port with a single reset line. */
	struct gpio_desc	*perst;
	struct irq_domain	*domain;
	struct list_head	entry;
	unsigned long		*sid_map;
	u32			*saved_rid2sid;
	struct apple_tunable	*tunable;
	unsigned int		irq;
	unsigned int		link_irqs[2];
	u32			saved_intmask;
	int			sid_map_sz;
	struct gpio_desc	*perst_gpio;
	struct gpio_desc	*pwren_gpio;
	int			idx;
	bool			started;
	bool			needs_stop;
	bool			link_failed;
	bool			irq_disabled;
	void __iomem		*intr2axi;	/* adopt_link: this port's */
	u8			adopted_bus;	/* adopt_link: the secondary bus */
	unsigned long		adopted_slots;	/* adopt_link: RID2SID slots routed (slot == SID) */
	struct gpio_desc	*pwren;		/* adopt_link: endpoint power, if Linux trained */
	bool			powered;	/* cold train: pwren raised, port not parked yet */
	struct device		*cold_dart;	/* a parked port's DART: gated while the port is parked */
	struct pci_saved_state	*cold_rp_state;	/* a parked port's root port, as it was before the park */
};

static bool apple_radio_writer_closed(struct apple_pcie *pcie);
static bool apple_radio_config_allowed(struct apple_pcie *pcie);
static void __noreturn apple_radio_terminal(void);

static int apple_pcie_managed_admit(struct apple_pcie_port *port);
static struct apple_pcie *apple_pcie_lookup(struct device *dev);
static int apple_pcie_radio_check_cold(struct apple_pcie_port *port);

static void rmw_set(u32 set, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) | set, addr);
}

static void rmw_clear(u32 clr, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) & ~clr, addr);
}

/*
 * PCIe-C lives behind the same tunneled fabric as its DART.  A live Linux
 * /dev/mem replay of the root-port setup sequence showed that the aperture is
 * available, but each access needs a full completion barrier.  Without it,
 * back-to-back relaxed accesses can leave a transaction pending until a later
 * access reports an asynchronous SError.  Keep conventional root ports on the
 * existing fast path.
 */
static inline void apple_pcie_port_writel(struct apple_pcie_port *port,
					  u32 value, u32 offset)
{
	writel_relaxed(value, port->base + offset);
	if (port->pcie->hw->tunneled) {
		mb();
		isb();
	}
}

static inline u32 apple_pcie_port_readl(struct apple_pcie_port *port,
					u32 offset)
{
	u32 value = readl_relaxed(port->base + offset);

	if (port->pcie->hw->tunneled) {
		mb();
		isb();
	}

	return value;
}

static void apple_pcie_port_rmw_set(struct apple_pcie_port *port, u32 set,
				    u32 offset)
{
	apple_pcie_port_writel(port, apple_pcie_port_readl(port, offset) | set,
				 offset);
}

static void apple_pcie_port_rmw_clear(struct apple_pcie_port *port, u32 clear,
				      u32 offset)
{
	apple_pcie_port_writel(port,
				 apple_pcie_port_readl(port, offset) & ~clear,
				 offset);
}

static inline u32 apple_pcie_tunnel_readl(void __iomem *base, u32 offset)
{
	u32 value = readl_relaxed(base + offset);

	mb();
	isb();
	return value;
}

static inline void apple_pcie_tunnel_writel(void __iomem *base, u32 value,
					     u32 offset)
{
	writel_relaxed(value, base + offset);
	mb();
	isb();
}

/* Pulse only when the device tree names this bridge. */
static void apple_pcie_tunnel_pulse_intr2axi(struct apple_pcie *pcie)
{
	if (!pcie->intr2axi_base)
		return;

	apple_pcie_tunnel_writel(pcie->intr2axi_base, PCIEC_INTR2AXI_ENABLE,
				 PCIEC_INTR2AXI_CTRL);
}

static void apple_pcie_tunnel_apply_tunable(void __iomem *base,
					    struct apple_tunable *tunable)
{
	size_t i;

	for (i = 0; i < tunable->sz; i++) {
		u32 old, value;

		old = apple_pcie_tunnel_readl(base, tunable->values[i].offset);
		value = (old & ~tunable->values[i].mask) |
			tunable->values[i].value;
		if (value != old)
			apple_pcie_tunnel_writel(base, value,
						  tunable->values[i].offset);
	}
}

static void apple_msi_compose_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct apple_pcie *pcie = data->domain->host_data;

	msg->address_hi = upper_32_bits(DOORBELL_ADDR);
	msg->address_lo = lower_32_bits(DOORBELL_ADDR);
	msg->data = data->hwirq;
	/* T8132 indexes its MSI map by the shared-window vector. */
	if (pcie->hw->msimap_by_vector)
		msg->data += pcie->msi_vector_base;
}

static struct irq_chip apple_msi_bottom_chip = {
	.name			= "MSI",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_set_wake		= irq_chip_set_wake_parent,
	.irq_compose_msi_msg	= apple_msi_compose_msg,
};

static int apple_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *args)
{
	struct apple_pcie *pcie = domain->host_data;
	struct irq_fwspec fwspec = pcie->fwspec;
	unsigned int i;
	int ret, hwirq;

	mutex_lock(&pcie->lock);
	if (apple_radio_writer_closed(pcie)) {
		mutex_unlock(&pcie->lock);
		return -EBUSY;
	}
	pcie->radio_writers++;

	hwirq = bitmap_find_free_region(pcie->bitmap, pcie->nvecs,
					order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);

	if (hwirq < 0) {
		ret = -ENOSPC;
		goto done;
	}

	fwspec.param[fwspec.param_count - 2] += hwirq;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &fwspec);
	if (ret) {
		mutex_lock(&pcie->lock);
		bitmap_release_region(pcie->bitmap, hwirq,
				      order_base_2(nr_irqs));
		mutex_unlock(&pcie->lock);
		goto done;
	}

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &apple_msi_bottom_chip, pcie);
	}

	ret = 0;
done:
	mutex_lock(&pcie->lock);
	pcie->radio_writers--;
	mutex_unlock(&pcie->lock);
	return ret;
}

static void apple_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_pcie *pcie = domain->host_data;

	mutex_lock(&pcie->lock);
	if (apple_radio_writer_closed(pcie))
		apple_radio_terminal();

	bitmap_release_region(pcie->bitmap, d->hwirq, order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);
}

static const struct irq_domain_ops apple_msi_domain_ops = {
	.alloc	= apple_msi_domain_alloc,
	.free	= apple_msi_domain_free,
};

static void apple_port_irq_mask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	guard(raw_spinlock_irqsave)(&port->lock);
	apple_pcie_port_rmw_set(port, BIT(data->hwirq), PORT_INTMSK);
}

static void apple_port_irq_unmask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	guard(raw_spinlock_irqsave)(&port->lock);
	apple_pcie_port_rmw_clear(port, BIT(data->hwirq), PORT_INTMSK);
}

static bool hwirq_is_intx(unsigned int hwirq)
{
	return BIT(hwirq) & PORT_INT_INTx_MASK;
}

static void apple_port_irq_ack(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	if (!hwirq_is_intx(data->hwirq))
		apple_pcie_port_writel(port, BIT(data->hwirq), PORT_INTSTAT);
}

static int apple_port_irq_set_type(struct irq_data *data, unsigned int type)
{
	/*
	 * It doesn't seem that there is any way to configure the
	 * trigger, so assume INTx have to be level (as per the spec),
	 * and the rest is edge (which looks likely).
	 */
	if (hwirq_is_intx(data->hwirq) ^ !!(type & IRQ_TYPE_LEVEL_MASK))
		return -EINVAL;

	irqd_set_trigger_type(data, type);
	return 0;
}

static struct irq_chip apple_port_irqchip = {
	.name		= "PCIe",
	.irq_ack	= apple_port_irq_ack,
	.irq_mask	= apple_port_irq_mask,
	.irq_unmask	= apple_port_irq_unmask,
	.irq_set_type	= apple_port_irq_set_type,
};

static int apple_port_irq_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *args)
{
	struct apple_pcie_port *port = domain->host_data;
	struct irq_fwspec *fwspec = args;
	int i;

	for (i = 0; i < nr_irqs; i++) {
		irq_flow_handler_t flow = handle_edge_irq;
		unsigned int type = IRQ_TYPE_EDGE_RISING;

		if (hwirq_is_intx(fwspec->param[0] + i)) {
			flow = handle_level_irq;
			type = IRQ_TYPE_LEVEL_HIGH;
		}

		irq_domain_set_info(domain, virq + i, fwspec->param[0] + i,
				    &apple_port_irqchip, port, flow,
				    NULL, NULL);

		irq_set_irq_type(virq + i, type);
	}

	return 0;
}

static void apple_port_irq_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops apple_port_irq_domain_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= apple_port_irq_domain_alloc,
	.free		= apple_port_irq_domain_free,
};

static void apple_port_irq_handler(struct irq_desc *desc)
{
	struct apple_pcie_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long stat;
	int i;

	chained_irq_enter(chip, desc);

	raw_spin_lock(&port->irq_lock);
	if (port->irq_disabled)
		goto out;

	stat = apple_pcie_port_readl(port, PORT_INTSTAT);
	/* Masked link events can belong to the resume poller. */
	stat &= ~apple_pcie_port_readl(port, PORT_INTMSK);

	for_each_set_bit(i, &stat, 32)
		generic_handle_domain_irq(port->domain, i);

out:
	raw_spin_unlock(&port->irq_lock);
	chained_irq_exit(chip, desc);
}

static int apple_pcie_port_setup_irq(struct apple_pcie_port *port)
{
	struct fwnode_handle *fwnode = &port->np->fwnode;
	struct apple_pcie *pcie = port->pcie;
	u32 val = 0;

	if (pcie->hw->adopt_link) {
		char name[16];
		int irq;

		snprintf(name, sizeof(name), "port%d", port->idx);
		irq = of_irq_get_byname(pcie->dev->of_node, name);

		if (irq <= 0)
			return irq ?: -ENXIO;
		port->irq = irq;
	} else {
		/* FIXME: consider moving each interrupt under each port */
		port->irq = irq_of_parse_and_map(pcie->dev->of_node, port->idx);
	}
	if (!port->irq)
		return -ENXIO;

	port->domain = irq_domain_create_linear(fwnode, 32,
						&apple_port_irq_domain_ops,
						port);
	if (!port->domain) {
		irq_dispose_mapping(port->irq);
		port->irq = 0;
		return -ENOMEM;
	}

	if (pcie->hw->adopt_link)
		apple_pcie_port_writel(port, 0, PORT_MSICFG);

	/* Disable all interrupts */
	apple_pcie_port_writel(port, ~0, PORT_INTMSK);
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, ~0, PORT_LINKCMDSTS);

	/*
	 * AIC has no irq_disable callback, so a lazy disable would only mark
	 * the parent disabled, and a chained handler never masks it later.
	 * Make apple_pcie_port_disable_irq() really mask it.
	 */
	irq_set_status_flags(port->irq, IRQ_DISABLE_UNLAZY);
	irq_set_chained_handler_and_data(port->irq, apple_port_irq_handler, port);

	/* Configure MSI base address */
	BUILD_BUG_ON(upper_32_bits(DOORBELL_ADDR));
	apple_pcie_port_writel(port, lower_32_bits(DOORBELL_ADDR),
				 pcie->hw->port_msiaddr);
	if (pcie->hw->port_msiaddr_hi)
		apple_pcie_port_writel(port, 0, pcie->hw->port_msiaddr_hi);

	/* Enable MSIs, shared between all ports */
	if (pcie->hw->port_msimap) {
		for (int i = 0; i < pcie->nvecs; i++)
			apple_pcie_port_writel(port,
				FIELD_PREP(PORT_MSIMAP_TARGET, pcie->msi_vector_base + i) |
				PORT_MSIMAP_ENABLE,
				pcie->hw->port_msimap + 4 * (i +
				(pcie->hw->msimap_by_vector ? pcie->msi_vector_base : 0)));
	} else {
		apple_pcie_port_writel(port, 0, PORT_MSIBASE);
		val = ilog2(pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT;
	}

	if (pcie->hw->adopt_link) {
		if (apple_pcie_port_readl(port, pcie->hw->port_msiaddr) != DOORBELL_ADDR ||
		    apple_pcie_port_readl(port, pcie->hw->port_msiaddr_hi))
			return -EIO;
		for (int i = 0; i < pcie->nvecs; i++)
			if (apple_pcie_port_readl(port, pcie->hw->port_msimap + 4 * i) !=
			    (PORT_MSIMAP_ENABLE | (pcie->msi_vector_base + i)))
				return -EIO;
		/* Enable command is not readable; +0x84 is only a completion read. */
		writel(PCIEC_INTR2AXI_ENABLE, port->intr2axi + PCIEC_INTR2AXI_CTRL);
		readl(port->intr2axi + 0x84);
	}
	apple_pcie_port_writel(port, val | PORT_MSICFG_EN, PORT_MSICFG);
	if (pcie->hw->adopt_link &&
	    !(apple_pcie_port_readl(port, PORT_MSICFG) & PORT_MSICFG_EN))
		return -EIO;
	return 0;
}

static irqreturn_t apple_pcie_port_irq(int irq, void *data)
{
	struct apple_pcie_port *port = data;
	unsigned int hwirq = irq_domain_get_irq_data(port->domain, irq)->hwirq;

	switch (hwirq) {
	case PORT_INT_LINK_UP:
		dev_info_ratelimited(port->pcie->dev, "Link up on %pOF\n",
				     port->np);
		complete_all(&port->pcie->event);
		break;
	case PORT_INT_LINK_DOWN:
		if (port->pcie->hw->adopt_link)
			WRITE_ONCE(port->link_failed, true);
		/* A later link-up alone does not prove downstream state survived. */
		if (port->pcie->hw->tunneled && READ_ONCE(port->started)) {
			WRITE_ONCE(port->link_failed, true);
			atomic_notifier_call_chain(&apple_pcie_tunnel_notifiers,
						   APPLE_PCIE_TUNNEL_LINK_DOWN,
						   port->pcie->dev->parent);
		}
		dev_info_ratelimited(port->pcie->dev, "Link down on %pOF\n",
				     port->np);
		break;
	default:
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int apple_pcie_port_register_irqs(struct apple_pcie_port *port)
{
	static struct {
		unsigned int	hwirq;
		const char	*name;
	} port_irqs[] = {
		{ PORT_INT_LINK_UP,	"Link up",	},
		{ PORT_INT_LINK_DOWN,	"Link down",	},
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(port_irqs); i++) {
		struct irq_fwspec fwspec = {
			.fwnode		= &port->np->fwnode,
			.param_count	= 1,
			.param		= {
				[0]	= port_irqs[i].hwirq,
			},
		};
		int irq, ret;

		irq = irq_domain_alloc_irqs(port->domain, 1, NUMA_NO_NODE,
					    &fwspec);
		if (irq <= 0)
			return irq ?: -ENOMEM;

		ret = request_irq(irq, apple_pcie_port_irq, 0,
				  port_irqs[i].name, port);
		if (ret) {
			irq_domain_free_irqs(irq, 1);
			return ret;
		}

		port->link_irqs[i] = irq;
	}

	return 0;
}

static void apple_pcie_port_unregister_irqs(struct apple_pcie_port *port)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(port->link_irqs); i++) {
		if (!port->link_irqs[i])
			continue;

		free_irq(port->link_irqs[i], port);
		irq_domain_free_irqs(port->link_irqs[i], 1);
		port->link_irqs[i] = 0;
	}
}

/*
 * Mask the parent first: the gated handler no longer reads or acknowledges
 * PORT_INTSTAT, so a level interrupt left unmasked would be unmasked again at
 * every EOI and fire without end. IRQ_DISABLE_UNLAZY makes
 * disable_irq_nosync() mask at AIC right away.
 *
 * A chained handler is not tracked by IRQD_IRQ_INPROGRESS, so neither
 * disable_irq() nor synchronize_irq() would wait for one that is already
 * running. Closing the gate under irq_lock does: it waits for a handler inside
 * the gated section, and any later handler skips the port registers.
 */
static void apple_pcie_port_disable_irq(struct apple_pcie_port *port)
{
	unsigned long flags;

	if (!port->irq || !port->domain || port->irq_disabled)
		return;

	disable_irq_nosync(port->irq);
	raw_spin_lock_irqsave(&port->irq_lock, flags);
	port->irq_disabled = true;
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
}

static void apple_pcie_port_enable_irq(struct apple_pcie_port *port)
{
	unsigned long flags;

	if (!port->irq_disabled)
		return;

	raw_spin_lock_irqsave(&port->irq_lock, flags);
	port->irq_disabled = false;
	raw_spin_unlock_irqrestore(&port->irq_lock, flags);
	enable_irq(port->irq);
}

static u32 apple_pcie_rid2sid_write(struct apple_pcie_port *port,
				    int idx, u32 val);

static u32 apple_pcie_port_tunctrl(struct apple_pcie_port *port)
{
	return port->pcie->hw->port_tunctrl ?: PORT_TUNCTRL;
}

static u32 apple_pcie_port_tunstat(struct apple_pcie_port *port)
{
	return apple_pcie_port_tunctrl(port) + PORT_TUNSTAT - PORT_TUNCTRL;
}

static int apple_pcie_tunnel_release_reset(struct apple_pcie_port *port)
{
	u32 stat;
	int ret;

	/*
	 * Starting the CIO PCIe tunnel leaves the tunneled root port in reset.
	 * Releasing it means writing zero to PORT_TUNCTRL, waiting for the
	 * reset-active indication to clear, and only then enabling the LTSSM.
	 */
	apple_pcie_port_writel(port, 0, apple_pcie_port_tunctrl(port));
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       !(stat & PORT_TUNSTAT_PERST_ON),
				       1000,
				       port->pcie->kernel_init ? 250000 : 100000,
				       false, port, apple_pcie_port_tunstat(port));
	if (ret)
		dev_err(port->pcie->dev,
			"port %pOF tunnel reset release timed out\n", port->np);

	return ret;
}

struct apple_pcie_reset_reg {
	u16 offset;
	u32 value;
};

/* Port hardware reset sequence, identical on T8103 and T600x PCIe-C. */
static const struct apple_pcie_reset_reg apple_pcie_tunnel_reset_regs[] = {
	{ 0x08c, 0x00000110 },
	{ 0x100, 0xffffffff },
	{ 0x148, 0xffffffff },
	{ 0x210, 0xffffffff },
	{ 0x080, 0x00000000 },
	{ 0x084, 0x00000000 },
	{ 0x104, 0xffffffff },
	{ 0x124, 0x00000000 },
	{ 0x128, 0x00000000 },
	{ 0x168, 0x00000000 },
	{ 0x13c, 0x00000010 },
	{ 0x800, 0x00100100 },
	{ 0x808, 0x00100045 },
	{ 0x810, 0x00000100 },
	{ 0x814, 0x00000000 },
	{ 0x130, 0x00000208 },
	{ 0x140, 0x00000010 },
	{ 0x144, 0x00253770 },
	{ 0x21c, 0x00000000 },
	{ 0x81c, 0x00000000 },
	{ 0x824, 0x00000000 },
};

static void apple_pcie_tunnel_reset_hardware(struct apple_pcie_port *port)
{
	int i;

	/* T6050 has no such sequence: only its RID2SID table is cleared. */
	if (port->pcie->hw->no_port_reset)
		goto clear_rid2sid;

	for (i = 0; i < ARRAY_SIZE(apple_pcie_tunnel_reset_regs); i++) {
		u32 value = apple_pcie_tunnel_reset_regs[i].value;

		/*
		 * t8103 writes 0. 0x208 is the value used when firmware left the
		 * port up; T600x/T602x cold init without a handoff uses it too.
		 */
		if (port->pcie->kernel_init &&
		    apple_pcie_tunnel_reset_regs[i].offset == 0x130)
			value = 0;
		if (port->pcie->hw->t8132_reset) {
			/* T8132 has no 0x128 or 0x810, and new 0x124/0x130 values. */
			switch (apple_pcie_tunnel_reset_regs[i].offset) {
			case 0x128:
			case 0x810:
				continue;
			case 0x124:
				value = 0x100;
				break;
			case 0x130:
				value = 0x03020208;
				break;
			}
		}
		apple_pcie_port_writel(port, value, apple_pcie_tunnel_reset_regs[i].offset);
	}

clear_rid2sid:
	for (i = 0; i < port->pcie->hw->max_rid2sid; i++)
		apple_pcie_rid2sid_write(port, i, 0);
	if (port->pcie->hw->t8132_reset)
		for (i = 0; i < 256; i++)
			apple_pcie_port_writel(port, 0, PORT_T602X_MSIMAP + 4 * i);
}

static int apple_pcie_tunnel_reinitialize(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	u32 stat;
	int i, ret;

	/*
	 * A cable disconnect intentionally stops PCIe-C after destroying its
	 * host bridge. Its always-on power domain retains that stopped state, so
	 * a later platform reprobe cannot rely on the original m1n1 handoff.
	 * Replay the port enable sequence, which is the same on T8103 and
	 * T600x, while ACIO's tunnel and Intr2AXI aperture are live.
	 */
	apple_pcie_tunnel_apply_tunable(pcie->debug_base,
					  pcie->debug_tunable);
	apple_pcie_tunnel_apply_tunable(pcie->fabric_base,
					  pcie->fabric_tunable);
	apple_pcie_tunnel_reset_hardware(port);
	apple_pcie_tunnel_apply_tunable(port->base, port->tunable);

	/*
	 * Keep the port disabled and its downstream reset asserted while the
	 * root complex is configured.  The enable sequence does this explicitly
	 * after applying the port tunables; releasing either
	 * one early occasionally leaves a hot-reconnected USB4 endpoint unable to
	 * train even though the tunnel itself is already live.
	 */
	apple_pcie_port_rmw_clear(port, PORT_APPCLK_EN, PORT_APPCLK);
	apple_pcie_port_rmw_clear(port, PORT_PERST_OFF,
				  pcie->hw->port_perst);
	apple_pcie_tunnel_apply_tunable(pcie->cfg->win, pcie->rc_tunable);
	apple_pcie_port_rmw_clear(port, PORT_APPCLK_CGDIS, PORT_APPCLK);
	apple_pcie_port_writel(port, PORT_COUNTER_ENABLE, PORT_COUNTER_CTRL);
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, ~0, PORT_LINKCMDSTS);
	apple_pcie_tunnel_pulse_intr2axi(pcie);

	/* Release reset immediately before enabling the configured port. */
	apple_pcie_port_rmw_set(port, PORT_PERST_OFF,
				pcie->hw->port_perst);
	apple_pcie_port_rmw_set(port, PORT_APPCLK_EN, PORT_APPCLK);
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_STATUS_READY,
				       10, 250000, false, port, PORT_STATUS);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "PCIe-C port did not enter RUN after hot reconnect\n");

	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       !(stat & PORT_LINKSTS_BUSY),
				       10, 250000, false, port, PORT_LINKSTS);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "PCIe-C port did not become idle after hot reconnect\n");

	for (i = 0; i < pcie->hw->max_rid2sid; i++)
		apple_pcie_rid2sid_write(port, i, 0);

	dev_info(pcie->dev,
		 "PCIe-C hardware reinitialized after cable reconnect\n");
	return 0;
}

/*
 * Bring up a port selected by apple,pciec-kernel-init that no firmware
 * handoff left running: tunables and the reset table (T6050 has none),
 * then PERST and APPCLK, and only then the root-port config-space
 * tunables. Stop before releasing the tunnel reset or starting link
 * training.
 */
static int apple_pcie_tunnel_cold_init(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	u32 stat;
	int ret;

	stat = apple_pcie_port_readl(port, PORT_STATUS);
	dev_info(pcie->dev, "port %pOF PCIe-C cold init, status %#x\n",
		 port->np, stat);

	dev_info(pcie->dev, "port %pOF cold init: debug and fabric tunables\n",
		 port->np);
	apple_pcie_tunnel_apply_tunable(pcie->debug_base, pcie->debug_tunable);
	apple_pcie_tunnel_apply_tunable(pcie->fabric_base, pcie->fabric_tunable);
	dev_info(pcie->dev, "port %pOF cold init: port reset table\n", port->np);
	apple_pcie_tunnel_reset_hardware(port);
	apple_pcie_tunnel_apply_tunable(port->base, port->tunable);

	dev_info(pcie->dev, "port %pOF cold init: PERST off, APPCLK on\n",
		 port->np);
	apple_pcie_port_rmw_set(port, PORT_PERST_OFF, pcie->hw->port_perst);
	apple_pcie_port_rmw_set(port, PORT_APPCLK_EN, PORT_APPCLK);
	/*
	 * Without an m1n1 handoff, T600x/T602x ports must report RUN before
	 * the root complex config space is written: writing the RC tunables
	 * first hard-resets a J416c.
	 */
	if (!pcie->kernel_init) {
		ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
					       stat & PORT_STATUS_READY,
					       10, 250000, false, port,
					       PORT_STATUS);
		if (ret)
			return dev_err_probe(pcie->dev, ret,
					     "port %pOF cold init: RUN not set (status %#x)\n",
					     port->np, stat);
	} else {
		apple_pcie_port_rmw_clear(port, PORT_APPCLK_CGDIS, PORT_APPCLK);
	}

	dev_info(pcie->dev, "port %pOF cold init: %sroot tunables\n",
		 port->np, pcie->oe_fabric_base ? "oe-fabric and " : "");
	if (pcie->oe_fabric_base)
		apple_pcie_tunnel_apply_tunable(pcie->oe_fabric_base,
						pcie->oe_fabric_tunable);
	apple_pcie_tunnel_apply_tunable(pcie->cfg ? pcie->cfg->win : pcie->early_cfg,
					pcie->rc_tunable);
	if (!pcie->kernel_init)
		apple_pcie_port_rmw_clear(port, PORT_APPCLK_CGDIS, PORT_APPCLK);

	apple_pcie_port_writel(port, PORT_COUNTER_ENABLE, PORT_COUNTER_CTRL);
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, ~0, PORT_LINKCMDSTS);
	apple_pcie_tunnel_pulse_intr2axi(pcie);

	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_STATUS_READY,
				       10, 250000, false, port, PORT_STATUS);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "port %pOF cold init: RUN not set (status %#x)\n",
				     port->np, stat);

	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       !(stat & PORT_LINKSTS_BUSY),
				       10, 250000, false, port, PORT_LINKSTS);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "port %pOF cold init: link still busy (status %#x)\n",
				     port->np, stat);

	dev_info(pcie->dev,
		 "port %pOF cold init done, status %#x link %#x tunstat %#x\n",
		 port->np,
		 apple_pcie_port_readl(port, PORT_STATUS),
		 apple_pcie_port_readl(port, PORT_LINKSTS),
		 apple_pcie_port_readl(port, apple_pcie_port_tunstat(port)));
	return 0;
}

static void apple_pcie_tunnel_restore_irq_hw(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	u32 value = 0;
	int i;

	/* Keep link events pending until the resume poller has consumed them. */
	apple_pcie_port_writel(port, ~0, PORT_INTMSK);
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, ~0, PORT_LINKCMDSTS);
	apple_pcie_port_writel(port, lower_32_bits(DOORBELL_ADDR),
				 pcie->hw->port_msiaddr);
	if (pcie->hw->port_msiaddr_hi)
		apple_pcie_port_writel(port, 0, pcie->hw->port_msiaddr_hi);

	if (pcie->hw->port_msimap) {
		for (i = 0; i < pcie->nvecs; i++)
			apple_pcie_port_writel(port,
				FIELD_PREP(PORT_MSIMAP_TARGET, pcie->msi_vector_base + i) |
				PORT_MSIMAP_ENABLE,
				pcie->hw->port_msimap + 4 * (i +
				(pcie->hw->msimap_by_vector ? pcie->msi_vector_base : 0)));
	} else {
		apple_pcie_port_writel(port, 0, PORT_MSIBASE);
		value = ilog2(pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT;
	}
	apple_pcie_port_writel(port, value | PORT_MSICFG_EN, PORT_MSICFG);
}

/*
 * Functions behind a Thunderbolt dock come up after the tunnel link does. On
 * a CalDigit TS3 Plus the I210 and the Alpine Ridge xHCI answer at link-up,
 * but the dock's FL1100 and ASM1142 xHCIs train their links later, behind
 * switch ports that are not hotplug-capable, so a scan before then misses
 * them for good. PCIe allows a function 1 s after reset before it has to
 * answer configuration requests (PCIe r6.0, sec 6.6.1; PCI_RESET_WAIT in the
 * PCI core), so the bus behind a tunnel is scanned only once its link has
 * been up that long. This applies to every tunnel, on every SoC.
 */
#define APPLE_PCIE_TUNNEL_READY_MS	1000

/*
 * The T6050 PCIe-C root port's ACK latency limit (0x700, low half) and flow
 * control update timer (0x73c, bits 28:16) are sized for the max payload
 * size. Each word holds both, for MPS 128, 256 and 512.
 */
static const u32 apple_pcie_mps_timers[] = {
	0x003c003c, 0x005a005a, 0x006b006b,
};

static void apple_pcie_update_mps_timers(struct apple_pcie *pcie, u32 mps)
{
	void __iomem *rp = pcie->cfg->win;
	u32 cap, word, val;

	cap = apple_pcie_tunnel_readl(rp, PCIEC_RP_DEVCAP) & PCI_EXP_DEVCAP_PAYLOAD;
	if (mps > cap || mps >= ARRAY_SIZE(apple_pcie_mps_timers)) {
		dev_warn(pcie->dev, "no link timers for a %u-byte MPS\n", 128 << mps);
		return;
	}

	word = apple_pcie_mps_timers[mps];
	writew_relaxed(lower_16_bits(word), rp + PCIEC_RP_ACK_LATENCY);
	/* Complete the write, as apple_pcie_tunnel_writel() does. */
	mb();
	isb();
	val = apple_pcie_tunnel_readl(rp, PCIEC_RP_FC_TIMER);
	val &= PCIEC_RP_FC_TIMER_KEEP;
	val |= FIELD_PREP(PCIEC_RP_FC_TIMER_LIMIT, upper_16_bits(word)) |
	       PCIEC_RP_FC_TIMER_EN;
	apple_pcie_tunnel_writel(rp, val, PCIEC_RP_FC_TIMER);
}

/* Process context: the link of a tunnel came up. */
static void apple_pcie_tunnel_link_up(struct apple_pcie *pcie)
{
	WRITE_ONCE(pcie->tunnel_ready_at,
		   jiffies + msecs_to_jiffies(APPLE_PCIE_TUNNEL_READY_MS));
	if (pcie->hw->mps_timers)
		apple_pcie_update_mps_timers(pcie,
			FIELD_GET(PCI_EXP_DEVCTL_PAYLOAD,
				  apple_pcie_tunnel_readl(pcie->cfg->win,
							  PCIEC_RP_DEVCTL)));
}

/* Call before scanning the bus behind a tunnel; elapsed time counts. */
static void apple_pcie_tunnel_wait_ready(struct apple_pcie *pcie)
{
	long left = (long)(READ_ONCE(pcie->tunnel_ready_at) - jiffies);

	if (left > 0)
		msleep(jiffies_to_msecs(left));
}

static int apple_pcie_tunnel_start(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	u32 stat;
	int i, ret;
	u32 link_events = BIT(PORT_INT_LINK_UP) | BIT(PORT_INT_LINK_DOWN) |
			  BIT(PORT_INT_TUNNEL_ERR);

	if (pcie->resume_failed)
		return -EIO;

	/* A failed start still owns partially enabled port hardware. */
	port->needs_stop = true;

	if (pcie->kernel_init && pcie->power_retained) {
		/*
		 * M1 requires the same activation sequence after sleep as after
		 * cable insertion. In particular, release tunneled reset only
		 * after APPCLK, PERST and the root-port tunables are restored.
		 */
		apple_pcie_tunnel_pulse_intr2axi(pcie);
		apple_pcie_tunnel_apply_tunable(port->base, port->tunable);
		apple_pcie_port_rmw_set(port, PORT_APPCLK_EN, PORT_APPCLK);
		usleep_range(10, 20);
		apple_pcie_port_rmw_set(port, PORT_PERST_OFF,
					pcie->hw->port_perst);
		usleep_range(10, 20);
		ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
					       stat & PORT_STATUS_READY,
					       10, 250000, false, port, PORT_STATUS);
		if (ret)
			return ret;
		apple_pcie_tunnel_apply_tunable(pcie->cfg->win, pcie->rc_tunable);
		ret = apple_pcie_tunnel_release_reset(port);
		if (ret)
			return ret;

		/*
		 * LINKSTS can still contain the pre-suspend link-up indication.
		 * Require a new link-up event, retrying the LTSSM on a transient
		 * link-down/tunnel-error event, as during M1 tunnel activation.
		 */
		for (i = 0; i < 3; i++) {
			apple_pcie_port_writel(port, link_events, PORT_INTSTAT);
			apple_pcie_port_writel(port, PORT_LTSSMCTL_START,
					       PORT_LTSSMCTL);
			ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
						       stat & link_events,
						       1000, 500000, false, port,
						       PORT_INTSTAT);
			if (!ret && (stat & link_events) == BIT(PORT_INT_LINK_UP) &&
			    (apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP))
				break;
		}
		if (i == 3) {
			dev_err(pcie->dev,
				"port %pOF tunnel link did not restart (events %#x, link %#x)\n",
				port->np, stat,
				apple_pcie_port_readl(port, PORT_LINKSTS));
			return -ETIMEDOUT;
		}
		apple_pcie_port_writel(port, link_events, PORT_INTSTAT);
		/* Allow downstream functions to finish reset before config I/O. */
		msleep(100);
		if (!(apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP))
			return -ENOLINK;
		goto restored;
	}

	if (pcie->power_retained) {
		/*
		 * The ATC PCIe domain remains powered on these systems. Replaying
		 * the cold-init reset table against that retained state raises an
		 * asynchronous external abort on T6020.
		 * Re-activating the ACIO PCIe tunnel closes the Intr2AXI aperture,
		 * just as it does during cold cable activation, so pulse it before
		 * releasing the retained port reset.
		 * Release the completed sleep handshake before re-enabling the
		 * retained port, which is the inverse of the disable sequence.
		 */
		apple_pcie_tunnel_pulse_intr2axi(pcie);
		ret = apple_pcie_tunnel_release_reset(port);
		if (ret)
			return ret;
	} else {
		/*
		 * A genuinely power-gated PCIe-C controller lost its register
		 * state. Recreate the cold-init baseline before start.
		 */
		apple_pcie_tunnel_apply_tunable(pcie->debug_base,
						  pcie->debug_tunable);
		apple_pcie_tunnel_apply_tunable(pcie->fabric_base,
						  pcie->fabric_tunable);
		apple_pcie_tunnel_reset_hardware(port);
		apple_pcie_tunnel_apply_tunable(port->base, port->tunable);
	}

	apple_pcie_port_rmw_set(port, PORT_PERST_OFF,
				pcie->hw->port_perst);
	apple_pcie_port_rmw_set(port, PORT_APPCLK_EN, PORT_APPCLK);

	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_STATUS_READY,
				       10, 250000, false, port, PORT_STATUS);
	if (ret) {
		dev_err(pcie->dev, "port %pOF resume ready wait timed out\n",
			port->np);
		return ret;
	}

	if (!pcie->power_retained) {
		apple_pcie_tunnel_apply_tunable(pcie->cfg->win,
						  pcie->rc_tunable);
		apple_pcie_port_rmw_clear(port, PORT_APPCLK_CGDIS,
					  PORT_APPCLK);
		apple_pcie_port_writel(port, PORT_COUNTER_ENABLE,
					PORT_COUNTER_CTRL);
		apple_pcie_tunnel_pulse_intr2axi(pcie);
		apple_pcie_tunnel_restore_irq_hw(port);
		for_each_set_bit(i, port->sid_map, port->sid_map_sz)
			apple_pcie_rid2sid_write(port, i,
						port->saved_rid2sid[i]);

		ret = apple_pcie_tunnel_release_reset(port);
		if (ret)
			return ret;
	}

	apple_pcie_port_writel(port, PORT_LTSSMCTL_START, PORT_LTSSMCTL);
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_LINKSTS_UP,
				       1000, 500000, false, port, PORT_LINKSTS);
	if (ret) {
		dev_err(pcie->dev, "port %pOF resume link training timed out\n",
			port->np);
		return ret;
	}
restored:
	apple_pcie_tunnel_link_up(pcie);
	/* Publish ownership before unmasking a possible link-down event. */
	WRITE_ONCE(port->started, true);
	/* The port helper uses a relaxed MMIO write. */
	wmb();
	apple_pcie_port_writel(port, port->saved_intmask, PORT_INTMSK);

	dev_info(pcie->dev, "port %pOF tunnel link restored after suspend\n",
		 port->np);
	return 0;
}

static bool apple_pcie_tunnel_transactions_idle(struct apple_pcie_port *port)
{
	u32 value;

	value = apple_pcie_port_readl(port, PORT_OUTS_NPREQS);
	if (value & (PORT_OUTS_NPREQS_REQ | PORT_OUTS_NPREQS_CPL))
		return false;
	if (apple_pcie_port_readl(port, apple_pcie_port_tunctrl(port) -
				  PORT_TUNCTRL + PORT_OUTS_PREQS_HDR) &
	    PORT_OUTS_PREQS_HDR_MASK)
		return false;
	if (apple_pcie_port_readl(port, apple_pcie_port_tunctrl(port) -
				  PORT_TUNCTRL + PORT_OUTS_PREQS_DATA) &
	    PORT_OUTS_PREQS_DATA_MASK)
		return false;
	if (apple_pcie_port_readl(port, PORT_RXWR_FIFO) &
	    (PORT_RXWR_FIFO_HDR | PORT_RXWR_FIFO_DATA))
		return false;
	if (apple_pcie_port_readl(port, PORT_RXRD_FIFO) & PORT_RXRD_FIFO_REQ)
		return false;
	if (apple_pcie_port_readl(port, PORT_OUTS_CPLS) &
	    (PORT_OUTS_CPLS_SHRD | PORT_OUTS_CPLS_WAIT))
		return false;

	return true;
}

static int apple_pcie_tunnel_stop(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	bool idle;
	u32 stat;
	int err = 0, ret;

	if (port->started)
		port->saved_intmask = apple_pcie_port_readl(port, PORT_INTMSK);
	apple_pcie_port_writel(port, ~0, PORT_INTMSK);
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, ~0, PORT_LINKCMDSTS);
	apple_pcie_port_rmw_clear(port, PORT_LTSSMCTL_START, PORT_LTSSMCTL);
	ret = read_poll_timeout_atomic(apple_pcie_tunnel_transactions_idle, idle,
				       idle, 10, 20000, false, port);
	if (ret)
		dev_warn(pcie->dev,
			 "port %pOF transactions did not drain before suspend\n",
			 port->np);
	err = ret;

	/*
	 * PCIe-C has no external PERST# line. The USB4 router asserts the
	 * tunneled reset request, and the root port must acknowledge it before
	 * ACIO loses power. A cable disconnect requires that same ordering.
	 */
	apple_pcie_port_rmw_set(port, PORT_TUNCTRL_PERST_ON, apple_pcie_port_tunctrl(port));
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_TUNSTAT_PERST_ON,
				       1000, 100000, false, port,
				       apple_pcie_port_tunstat(port));
	if (ret)
		dev_warn(pcie->dev, "port %pOF tunnel reset assertion timed out\n",
			 port->np);
	if (ret && !err)
		err = ret;

	apple_pcie_port_rmw_clear(port, PORT_PERST_OFF,
				  pcie->hw->port_perst);
	apple_pcie_port_rmw_clear(port, PORT_APPCLK_EN, PORT_APPCLK);

	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       !(stat & PORT_STATUS_READY),
				       10, 100000, false, port, PORT_STATUS);
	if (ret)
		dev_warn(pcie->dev, "port %pOF disable timed out\n", port->np);
	if (ret && !err)
		err = ret;

	apple_pcie_port_rmw_set(port, PORT_TUNCTRL_PERST_ACK_REQ, apple_pcie_port_tunctrl(port));
	/* M1 reports an outstanding acknowledgment until ACK_PEND clears. */
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       pcie->kernel_init ?
				       !(stat & PORT_TUNSTAT_PERST_ACK_PEND) :
				       stat & PORT_TUNSTAT_PERST_ACK_PEND,
				       1000, 1000000, false, port, apple_pcie_port_tunstat(port));
	if (ret) {
		dev_warn(pcie->dev,
			 "port %pOF tunnel reset acknowledgment timed out\n",
			 port->np);
		/*
		 * The local port is already down. A removed cable cannot
		 * finish the router handshake, and reporting that as a
		 * quiesce failure makes the follow-up call skip this port
		 * and claim success.
		 */
		if (err || (apple_pcie_port_readl(port, PORT_STATUS) &
			    PORT_STATUS_READY))
			err = ret;
	}
	apple_pcie_port_rmw_clear(port, PORT_TUNCTRL_PERST_ACK_REQ,
				  apple_pcie_port_tunctrl(port));
	WRITE_ONCE(port->started, false);
	port->needs_stop = err || (apple_pcie_port_readl(port, PORT_STATUS) &
				  PORT_STATUS_READY);

	return err;
}

static void apple_pcie_port_teardown(struct apple_pcie_port *port)
{
	apple_pcie_port_disable_irq(port);

	/* Busy managed admission has not acquired or programmed this IRQ block. */
	if (port->base &&
	    (port->domain ||
	     (!port->pcie->managed_sids && !port->pcie->hw->adopt_link)))
		apple_pcie_port_writel(port, ~0, PORT_INTMSK);

	if (port->pcie->hw->adopt_link && port->domain)
		apple_pcie_port_writel(port, 0, PORT_MSICFG);
	if (port->adopted_slots) {
		int slot;

		for_each_set_bit(slot, &port->adopted_slots, BITS_PER_LONG)
			apple_pcie_rid2sid_write(port, slot, 0);
		port->adopted_slots = 0;
	}

	if (port->pcie->hw->tunneled && (port->started || port->needs_stop))
		apple_pcie_tunnel_stop(port);

	apple_pcie_port_unregister_irqs(port);

	if (port->irq) {
		/* An IRQ may have selected our handler without loading its data. */
		irq_set_chained_handler(port->irq, NULL);
		synchronize_rcu();
		irq_dispose_mapping(port->irq);
		port->irq = 0;
	}

	if (port->domain) {
		irq_domain_remove(port->domain);
		port->domain = NULL;
	}

	if (!list_empty(&port->entry))
		list_del_init(&port->entry);

	if (port->np) {
		of_node_put(port->np);
		port->np = NULL;
	}
}

static int apple_pcie_setup_refclk(struct apple_pcie *pcie,
				   struct apple_pcie_port *port)
{
	u32 stat;
	int res;

	if (pcie->hw->phy_lane_ctl)
		rmw_set(PHY_LANE_CTL_CFGACC, port->phy + pcie->hw->phy_lane_ctl);

	rmw_set(PHY_LANE_CFG_REFCLK0REQ, port->phy + PHY_LANE_CFG);

	res = readl_relaxed_poll_timeout(port->phy + PHY_LANE_CFG,
					 stat, stat & PHY_LANE_CFG_REFCLK0ACK,
					 100, 50000);
	if (res < 0)
		return res;

	rmw_set(PHY_LANE_CFG_REFCLK1REQ, port->phy + PHY_LANE_CFG);
	res = readl_relaxed_poll_timeout(port->phy + PHY_LANE_CFG,
					 stat, stat & PHY_LANE_CFG_REFCLK1ACK,
					 100, 50000);

	if (res < 0)
		return res;

	if (pcie->hw->phy_lane_ctl)
		rmw_clear(PHY_LANE_CTL_CFGACC, port->phy + pcie->hw->phy_lane_ctl);

	if (pcie->hw->refbuf_off) {
		rmw_clear(PHY_LANE_CFG_REFBUF_OFF, port->phy + PHY_LANE_CFG);
		udelay(1);
	}

	rmw_set(PHY_LANE_CFG_REFCLKEN, port->phy + PHY_LANE_CFG);

	if (pcie->hw->port_refclk)
		rmw_set(PORT_REFCLK_EN, port->base + pcie->hw->port_refclk);

	return 0;
}

static u32 port_rid2sid_offset(struct apple_pcie_port *port, int idx)
{
	return port->pcie->hw->port_rid2sid + 4 * idx;
}

static u32 apple_pcie_rid2sid_write(struct apple_pcie_port *port,
				    int idx, u32 val)
{
	u32 offset = port_rid2sid_offset(port, idx);

	apple_pcie_port_writel(port, val, offset);
	/* Read back to ensure completion of the write */
	return apple_pcie_port_readl(port, offset);
}

/*
 * The DART stream an adopted port's function `rid` maps to: one T6050 DART stream, nonzero
 * and below the RID2SID table size (slot == SID), from the host's iommu-map.
 */
static int apple_pcie_adopted_sid(struct apple_pcie_port *port, u32 rid, u32 *sid)
{
	struct apple_pcie *pcie = port->pcie;
	struct of_phandle_args iommu = {};
	int ret;

	if (rid >> 8 != port->adopted_bus) {
		/* The PCI core numbers root ports' buses in devfn order: the bootloader must too */
		dev_err(pcie->dev, "port %d: RID %04x is off bus %02x, which the bootloader numbered\n",
			port->idx, rid, port->adopted_bus);
		return -ENODEV;
	}
	ret = of_map_iommu_id(pcie->dev->of_node, rid, &iommu);
	if (ret)
		return ret;
	ret = iommu.args_count == 1 && iommu.args[0] &&
	      iommu.args[0] < pcie->hw->max_rid2sid &&
	      iommu.args[0] < BITS_PER_LONG &&
	      of_device_is_compatible(iommu.np, "apple,t6050-dart") ? 0 : -EINVAL;
	*sid = iommu.args[0];
	of_node_put(iommu.np);
	return ret;
}

/* Route an adopted port's function `rid` to its stream, or confirm the route is in place. */
static int apple_pcie_adopted_route(struct apple_pcie_port *port, u32 rid)
{
	u32 sid, value, cur;
	int ret = apple_pcie_adopted_sid(port, rid, &sid);

	if (ret)
		return ret;
	value = T6050_ROUTE(rid, sid);
	cur = apple_pcie_port_readl(port, port_rid2sid_offset(port, sid));
	if (cur == value && test_bit(sid, &port->adopted_slots))
		return 0;
	if (cur & PORT_RID2SID_VALID)
		return -EBUSY;
	set_bit(sid, &port->adopted_slots);
	if (apple_pcie_rid2sid_write(port, sid, value) != value)
		return -EIO;
	dev_info(port->pcie->dev, "port %d: RID %04x -> SID %u, slot %u\n",
		 port->idx, rid, sid, sid);
	return 0;
}

/*
 * Qualified J775D handoff of one bootloader-trained port: its root port
 * 00:<port>.0 and the endpoint on its secondary bus (function 0 routed here; a
 * multi-function endpoint's other functions when they are enabled), with DMA and
 * endpoint MSI disabled. Every resource comes from the DT by the port's index
 * (port<n>, phy<n>, intr2axi<n>, interrupt port<n>, the iommu-map entry for the
 * endpoint's RID). All checks precede the first PCIe register write.
 */
static int apple_pcie_adopt_link(struct apple_pcie_port *port)
{
	static const char *const names[] = { "port%d", "phy%d", "intr2axi%d" };
	struct apple_pcie *pcie = port->pcie;
	struct platform_device *pdev = to_platform_device(pcie->dev);
	void __iomem *rp, *ep;
	char name[16];
	u32 value, bus;
	u8 cap;
	bool msi = false;
	int ret, i;

	if (pcie->cfg->busr.start != 0 ||
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "config") ||
	    !platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc"))
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		snprintf(name, sizeof(name), names[i], port->idx);
		if (!platform_get_resource_byname(pdev, IORESOURCE_MEM, name))
			return -EINVAL;
	}

	if (!(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN) ||
	    !(readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY) ||
	    !(apple_pcie_port_readl(port, PORT_STATUS) & PORT_STATUS_READY) ||
	    (apple_pcie_port_readl(port, PORT_LINKSTS) &
	     (PORT_LINKSTS_UP | PORT_LINKSTS_BUSY)) != PORT_LINKSTS_UP ||
	    !(apple_pcie_port_readl(port, PORT_LTSSMCTL) & PORT_LTSSMCTL_START) ||
	    (apple_pcie_port_readl(port, PORT_APPCLK) & 0x101) != 0x101 ||
	    (apple_pcie_port_readl(port, pcie->hw->port_perst) & 0x10001) != 1 ||
	    (readl(port->phy + PHY_LANE_CFG) & 0x1000061f) != 0x60f)
		return -ENOLINK;

	/* The bootloader numbered the root port's buses: primary 0, one secondary. */
	rp = pcie->cfg->win + (PCI_DEVFN(port->idx, 0) << 12);
	value = readl(rp + PCI_PRIMARY_BUS);
	bus = (value >> 8) & 0xff;
	if (readl(rp) != 0x100c106b || (value & 0xff) != 0 || !bus ||
	    ((value >> 16) & 0xff) != bus || bus > pcie->cfg->busr.end ||
	    (readw(rp + PCI_BRIDGE_CONTROL) & PCI_BRIDGE_CTL_BUS_RESET))
		return -ENODEV;
	ep = pcie->cfg->win + (bus << 20);
	value = readl(ep);
	if (value == 0xffffffff || !(value & 0xffff) ||
	    (readb(ep + PCI_HEADER_TYPE) & 0x7f) != PCI_HEADER_TYPE_NORMAL ||
	    (readw(ep + PCI_COMMAND) & (PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
				       PCI_COMMAND_MASTER)))
		return -ENODEV;

	port->adopted_bus = bus;
	ret = apple_pcie_adopted_sid(port, bus << 8, &value);
	if (ret)
		return ret;

	snprintf(name, sizeof(name), "intr2axi%d", port->idx);
	if (!port->intr2axi)
		port->intr2axi = devm_platform_ioremap_resource_byname(pdev, name);
	if (IS_ERR(port->intr2axi))
		return PTR_ERR(port->intr2axi);
	/*
	 * Only a port whose endpoint needs a root-port reset names its PERST# line
	 * (reset-gpios, on the port node as on every other Apple host). The pin
	 * controller owns the pad; the line is taken as the bootloader left it,
	 * released, and driven only by the reset.
	 */
	if (!port->perst)
		port->perst = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(port->np),
						    "reset", GPIOD_ASIS, "PERST#");
	if (IS_ERR(port->perst)) {
		if (PTR_ERR(port->perst) != -ENOENT)
			return PTR_ERR(port->perst);
		port->perst = NULL;
	}

	/* The currently qualified handoff is Gen3 x1, with no retrain in progress. */
	cap = readb(rp + PCI_CAPABILITY_LIST);
	for (i = 0; cap && i < 48; i++) {
		if (cap < 0x40 || (cap & 3))
			return -EINVAL;
		if (readb(rp + cap + PCI_CAP_LIST_ID) == PCI_CAP_ID_EXP)
			break;
		cap = readb(rp + cap + PCI_CAP_LIST_NEXT);
	}
	if (!cap || i == 48)
		return -EINVAL;
	/*
	 * x1 at up to the Gen3 target the bootloader set. The endpoint picks the speed: the N1's
	 * ROM personality links at Gen1 and comes back at Gen3 after its root-port reset.
	 */
	value = readw(rp + cap + PCI_EXP_LNKSTA);
	if ((value & (PCI_EXP_LNKSTA_NLW | PCI_EXP_LNKSTA_LT)) != (1 << PCI_EXP_LNKSTA_NLW_SHIFT) ||
	    (value & PCI_EXP_LNKSTA_CLS) < PCI_EXP_LNKSTA_CLS_2_5GB ||
	    (value & PCI_EXP_LNKSTA_CLS) > PCI_EXP_LNKSTA_CLS_8_0GB)
		return -ENOLINK;

	/* Refuse an active endpoint or malformed capability chain. */
	cap = readb(ep + PCI_CAPABILITY_LIST);
	for (i = 0; cap && i < 48; i++) {
		if (cap < 0x40 || (cap & 3))
			return -EINVAL;
		value = readw(ep + cap + PCI_MSI_FLAGS);
		switch (readb(ep + cap + PCI_CAP_LIST_ID)) {
		case PCI_CAP_ID_MSI:
			if (value & PCI_MSI_FLAGS_ENABLE)
				return -EBUSY;
			msi = true;
			break;
		case PCI_CAP_ID_MSIX:
			if (value & PCI_MSIX_FLAGS_ENABLE)
				return -EBUSY;
			msi = true;
			break;
		}
		cap = readb(ep + cap + PCI_CAP_LIST_NEXT);
	}
	if (cap || !msi)
		return -EINVAL;

	for (i = 0; i < pcie->hw->max_rid2sid; i++)
		if (apple_pcie_port_readl(port, port_rid2sid_offset(port, i)) &
		    PORT_RID2SID_VALID)
			return -EBUSY;

	/* The route goes in the slot whose index is the SID, as on J775D's adopted ports. */
	ret = apple_pcie_adopted_route(port, bus << 8);
	if (!ret)
		dev_info(pcie->dev, "adopted port %d, endpoint %02x:00\n", port->idx, bus);
	return ret;
}

/*
 * T6050 port bring-up, for an adopted port whose endpoint must come back on a
 * retrained link: port disable, the port reset table, port enable, root complex
 * configuration for Gen3 x1, then the lane PHY, PERST# and the LTSSM. The port's
 * registers are its port<n>, phy<n> and intr2axi<n> blocks; the root complex's
 * are the root port's config space. The boot loader copies the port's tunables
 * onto its node.
 */
static const struct apple_pcie_reset_reg apple_pcie_t6050_reset_regs[] = {
	{ 0x088, 0x00000110 },
	{ 0x100, 0xffffffff },
	{ 0x148, 0xffffffff },
	{ 0x210, 0xffffffff },
	{ 0x080, 0x00000000 },
	{ 0x084, 0x00000000 },
	{ 0x104, 0xfffffff0 },
	{ 0x124, 0x00000100 },
	{ 0x16c, 0x00000000 },
	{ 0x13c, 0x00000000 },
	{ 0x800, 0x00100100 },
	{ 0x808, 0x001001ff },
	{ 0x82c, 0x00010000 },
};

/* Written after the RID2SID and MSI map tables are cleared */
static const struct apple_pcie_reset_reg apple_pcie_t6050_reset_regs_late[] = {
	{ 0x130, 0x03020000 },
	{ 0x140, 0x00000010 },
	{ 0x144, 0x00253770 },
	{ 0x21c, 0x00000000 },
	{ 0x834, 0x00000000 },
	{ 0x83c, 0x00000000 },
};

static void apple_pcie_t6050_rmw(void __iomem *addr, u32 clear, u32 set)
{
	writel((readl(addr) & ~clear) | set, addr);
}

/* The root port's config space through the ECAM window: there is no pci_dev before enumeration */
static void __iomem *apple_pcie_t6050_dbi(struct apple_pcie_port *port)
{
	return port->pcie->cfg->win + (PCI_DEVFN(port->idx, 0) << 12);
}

static void apple_pcie_t6050_dbi_rmw(void __iomem *dbi, int where, u32 clear, u32 set)
{
	writel((readl(dbi + where) & ~clear) | set, dbi + where);
}

static int apple_pcie_t6050_exp_cap(void __iomem *dbi)
{
	u8 cap = readb(dbi + PCI_CAPABILITY_LIST);
	int i;

	for (i = 0; cap && i < 48; i++) {
		if (cap < 0x40 || (cap & 3))
			return 0;
		if (readb(dbi + cap + PCI_CAP_LIST_ID) == PCI_CAP_ID_EXP)
			return cap;
		cap = readb(dbi + cap + PCI_CAP_LIST_NEXT);
	}
	return 0;
}

static int apple_pcie_t6050_wait(struct apple_pcie_port *port, void __iomem *base,
				 u32 offset, u32 mask, u32 value, unsigned int timeout_us)
{
	u32 stat;
	int ret;

	ret = readl_poll_timeout(base + offset, stat, (stat & mask) == value, 1, timeout_us);
	if (ret)
		dev_err(port->pcie->dev, "port %d: %#x = %#x, waiting for %#x/%#x\n",
			port->idx, offset, stat, value, mask);
	return ret;
}

/*
 * A tunable list the boot loader copies onto the root port node, groups of
 * (offset, 4, mask, value): for the port's registers, or with @rp for the root
 * port's config space. A list it did not copy is skipped.
 */
static int apple_pcie_t6050_tune(struct apple_pcie_port *port, void __iomem *dbi,
				 const char *name)
{
	u32 cells[64];
	int count, ret, i;

	if (!of_property_present(port->np, name))
		return 0;
	count = of_property_count_u32_elems(port->np, name);
	if (count <= 0 || count > ARRAY_SIZE(cells) || count % 4)
		return -EINVAL;
	ret = of_property_read_u32_array(port->np, name, cells, count);
	if (ret)
		return ret;
	for (i = 0; i < count; i += 4)
		if (cells[i + 1] != 4 || (cells[i] & 3) ||
		    cells[i] >= (dbi ? PCI_CFG_SPACE_EXP_SIZE : SZ_16K))
			return -EINVAL;

	for (i = 0; i < count; i += 4)
		apple_pcie_t6050_rmw((dbi ?: port->base) + cells[i], cells[i + 2], cells[i + 3]);
	return 0;
}

/*
 * PERST# through the pin controller: the pad as a GPIO output (its function bits
 * cleared, mode output) at the line's logical level, which the DT's
 * GPIO_ACTIVE_LOW turns into the pad level.
 */
static int apple_pcie_t6050_perst(struct apple_pcie_port *port, bool asserted)
{
	return gpiod_direction_output(port->perst, asserted);
}

/* Let the link go idle, hold the endpoint in reset and stop the port and its lane clocks */
static int apple_pcie_t6050_port_disable(struct apple_pcie_port *port)
{
	void __iomem *dbi = apple_pcie_t6050_dbi(port);
	void __iomem *perst = port->base + PORT_T602X_PERST;
	void __iomem *lane = port->phy + PHY_LANE_CFG;
	u32 stat;
	int ret;

	if (!(readl(port->base + PORT_STATUS) & BIT(2))) {
		ret = apple_pcie_t6050_wait(port, port->base, 0x820, BIT(2), BIT(2), 10000);
		if (ret)
			return ret;
	}
	ret = readl_poll_timeout_atomic(port->base + 0x820, stat,
					!(stat & GENMASK(8, 0)) || !(stat & GENMASK(23, 16)),
					1, 10000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(port->base + 0x13c, 0, BIT(8));

	/* The link out of L1 */
	ret = read_poll_timeout_atomic(readl, stat,
				       FIELD_GET(RC_T6050_LTSSM_STATE, stat) !=
					      RC_T6050_LTSSM_L1_IDLE,
				       5, 5000, false, dbi + RC_T6050_PORT_DEBUG0);
	if (ret)
		return ret;

	ret = apple_pcie_t6050_wait(port, port->base, 0xa8, BIT(0), BIT(0), 10000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(perst, 0, BIT(24));
	ret = apple_pcie_t6050_wait(port, port->base, 0xa8, BIT(16), BIT(16), 2000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(perst, 0, BIT(16));
	udelay(1);
	apple_pcie_t6050_rmw(perst, PORT_PERST_OFF, 0);
	udelay(1);
	apple_pcie_t6050_rmw(perst, BIT(24), 0);
	apple_pcie_t6050_rmw(port->base + 0x13c, BIT(8), 0);

	apple_pcie_t6050_rmw(lane, BIT(31), 0);
	apple_pcie_t6050_rmw(lane, BIT(10), 0);
	apple_pcie_t6050_rmw(lane, BIT(9), 0);
	apple_pcie_t6050_rmw(lane, 0, BIT(4));
	apple_pcie_t6050_rmw(lane, PHY_LANE_CFG_REFCLK0REQ, 0);
	apple_pcie_t6050_rmw(lane, PHY_LANE_CFG_REFCLK1REQ, 0);

	writel(~0, port->base + PORT_INTSTAT);
	apple_pcie_t6050_rmw(port->base + PORT_APPCLK, PORT_APPCLK_EN, 0);
	return apple_pcie_t6050_wait(port, port->base, PORT_STATUS, PORT_STATUS_READY, 0,
				     250000);
}

/* The port reset table, then the port clocked and out of reset, ready to train */
static int apple_pcie_t6050_port_enable(struct apple_pcie_port *port)
{
	void __iomem *perst = port->base + PORT_T602X_PERST;
	u32 stat;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(apple_pcie_t6050_reset_regs); i++)
		writel(apple_pcie_t6050_reset_regs[i].value,
		       port->base + apple_pcie_t6050_reset_regs[i].offset);
	for (i = 0; i < port->pcie->hw->max_rid2sid; i++)
		writel(0, port->base + port_rid2sid_offset(port, i));
	for (i = 0; i < 256; i++)
		writel(0, port->base + PORT_T602X_MSIMAP + 4 * i);
	for (i = 0; i < ARRAY_SIZE(apple_pcie_t6050_reset_regs_late); i++)
		writel(apple_pcie_t6050_reset_regs_late[i].value,
		       port->base + apple_pcie_t6050_reset_regs_late[i].offset);

	if (!(readl(perst) & BIT(16)))
		return -EIO;
	ret = apple_pcie_t6050_tune(port, NULL, "apple,apcie-config-tunables");
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(port->base + PORT_APPCLK, 0, PORT_APPCLK_EN);

	ret = readl_poll_timeout_atomic(port->base + 0xa8, stat,
					(stat & (BIT(16) | BIT(0))) != (BIT(16) | BIT(0)),
					1, 10000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(perst, 0, PORT_PERST_OFF);
	ret = apple_pcie_t6050_wait(port, port->base, 0xa8, BIT(0), BIT(0), 20000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(perst, BIT(16), 0);
	ret = apple_pcie_t6050_wait(port, port->base, PORT_STATUS, PORT_STATUS_READY,
				    PORT_STATUS_READY, 250000);
	if (ret)
		return ret;

	apple_pcie_t6050_rmw(port->base + PORT_APPCLK, 0, PORT_APPCLK_CGDIS);
	return apple_pcie_t6050_wait(port, port->base, PORT_LINKSTS, PORT_LINKSTS_BUSY, 0,
				     250000);
}

/* Root complex tunables and a Gen3 x1 link, with the read-only registers opened for it */
static int apple_pcie_t6050_port_configure(struct apple_pcie_port *port)
{
	static const char * const shadow[] = {
		"apple,pcie-rc-gen3-shadow-tunables",
		"apple,pcie-rc-gen4-shadow-tunables",
		"apple,pcie-rc-gen5-shadow-tunables",
	};
	static const u32 shadow_sel[] = { 0, BIT(24), 0 };
	void __iomem *dbi = apple_pcie_t6050_dbi(port);
	int cap = apple_pcie_t6050_exp_cap(dbi);
	u16 ctl2;
	int ret, i;

	if (!cap)
		return -EIO;
	apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_MISC_CONTROL_1, 0, RC_T6050_DBI_RO_WR_EN);
	ret = apple_pcie_t6050_tune(port, dbi, "apple,pcie-rc-tunables");
	for (i = 0; !ret && i < ARRAY_SIZE(shadow); i++) {
		apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_GEN3_RELATED, RC_T6050_RATE_SHADOW_SEL,
					 shadow_sel[i]);
		ret = apple_pcie_t6050_tune(port, dbi, shadow[i]);
	}
	if (!ret) {
		apple_pcie_t6050_dbi_rmw(dbi, 0x194, 0, BIT(0));
		if (!(readl(dbi + cap + PCI_EXP_LNKCAP2) & PCI_EXP_LNKCAP2_SLS_8_0GB))
			ret = -EIO;
	}
	if (!ret) {
		ctl2 = readw(dbi + cap + PCI_EXP_LNKCTL2);
		writew((ctl2 & ~PCI_EXP_LNKCTL2_TLS) | PCI_EXP_LNKCTL2_TLS_8_0GT,
		       dbi + cap + PCI_EXP_LNKCTL2);
		apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_PORT_LINK_CTRL, RC_T6050_LINK_MODE,
					 FIELD_PREP(RC_T6050_LINK_MODE, 1));
		apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_LINK_WIDTH_SPEED_CTRL, RC_T6050_LINK_WIDTH,
					 FIELD_PREP(RC_T6050_LINK_WIDTH, 1));
		apple_pcie_t6050_dbi_rmw(dbi, cap + PCI_EXP_LNKCAP, PCI_EXP_LNKCAP_MLW,
					 FIELD_PREP(PCI_EXP_LNKCAP_MLW, 1));
	}
	apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_MISC_CONTROL_1, RC_T6050_DBI_RO_WR_EN, 0);
	if (ret)
		return ret;

	apple_pcie_t6050_dbi_rmw(dbi, cap + PCI_EXP_LNKCTL, 0, PCI_EXP_LNKCTL_LBMIE);
	apple_pcie_t6050_dbi_rmw(dbi, RC_T6050_LINK_WIDTH_SPEED_CTRL, 0, RC_T6050_SPEED_CHANGE);
	writel(PCIEC_INTR2AXI_ENABLE, port->intr2axi + PCIEC_INTR2AXI_CTRL);
	return 0;
}

/* Disable, reset, enable and configure an adopted port whose endpoint is held in PERST# */
static int apple_pcie_t6050_port_restart(struct apple_pcie_port *port)
{
	return apple_pcie_t6050_port_disable(port) ?:
	       apple_pcie_t6050_port_enable(port) ?:
	       apple_pcie_t6050_port_configure(port);
}

/* Lane clocks, PERST# released, then the LTSSM to link up */
static int apple_pcie_t6050_port_link(struct apple_pcie_port *port)
{
	void __iomem *lane = port->phy + PHY_LANE_CFG;
	int ret;

	apple_pcie_t6050_rmw(lane, 0, PHY_LANE_CFG_REFCLK0REQ);
	ret = apple_pcie_t6050_wait(port, port->phy, PHY_LANE_CFG, PHY_LANE_CFG_REFCLK0ACK,
				    PHY_LANE_CFG_REFCLK0ACK, 500000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(lane, 0, PHY_LANE_CFG_REFCLK1REQ);
	ret = apple_pcie_t6050_wait(port, port->phy, PHY_LANE_CFG, PHY_LANE_CFG_REFCLK1ACK,
				    PHY_LANE_CFG_REFCLK1ACK, 500000);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(lane, BIT(4), 0);
	udelay(1);
	apple_pcie_t6050_rmw(lane, 0, BIT(9));
	apple_pcie_t6050_rmw(lane, 0, BIT(10));

	/* Tperst-clk is at least 100 us (PCIe CEM r5.0, 2.9.2) */
	usleep_range(100, 110);
	ret = apple_pcie_t6050_perst(port, false);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(port->base + PORT_LTSSMCTL, 0, PORT_LTSSMCTL_START);
	/* Up to 1 s: what m1n1 allows the J716s N1 after its power-up */
	ret = apple_pcie_t6050_wait(port, port->base, PORT_LINKSTS, PORT_LINKSTS_UP,
				    PORT_LINKSTS_UP, 1000000);
	return ret;
}

/* The same, and the wait after the link is up */
static int apple_pcie_t6050_port_train(struct apple_pcie_port *port)
{
	int ret = apple_pcie_t6050_port_link(port);

	if (ret)
		return ret;
	/* Wait for 100ms after the link trains (PCIe r5.0, 6.6.1) */
	msleep(100);
	return 0;
}

/*
 * A T6050 port the boot loader left untrained (the J714c's N1 on every boot): the shared
 * PCIe PHY clocked and out of reset, the RC running, the endpoint powered (pwren-gpios)
 * with PERST# held, then the port enable, configure and train sequences the root-port
 * reset uses, and the bus numbers a boot loader leaves. The adopt checks then run as for
 * a port trained before Linux. Order and values are m1n1's controller bring-up and the
 * kernel's own T6050 port sequence.
 */
#define T6050_PHY_CTRL			0x0
#define T6050_PHY_CTRL_CLK0REQ		BIT(0)
#define T6050_PHY_CTRL_CLK1REQ		BIT(1)
#define T6050_PHY_CTRL_CLK0ACK		BIT(2)
#define T6050_PHY_CTRL_CLK1ACK		BIT(3)
#define T6050_PHY_CTRL_RESET_T8132	BIT(4)
#define T6050_PHY_CTRL_RESET_T8103	BIT(7)
#define T6050_PHY_CTRL_LANES		GENMASK(9, 8)
#define T6050_PHY_CFG			0x4
/*
 * Set before the PHY-IP tunables. A trace of macOS bringing the port up shows no
 * access to this bit; meaning not known.
 */
#define T6050_PHY_CFG_BIT0		BIT(0)
/*
 * Set after the PHY-IP tunables. A trace of macOS shows it set too, after which
 * T6050_PHY_STAT_BIT0 rises (the trace reads 0x10 seven times, then 0x11); meaning
 * not known.
 */
#define T6050_PHY_CFG_BIT4		BIT(4)
#define T6050_PHY_STAT			0x8
/* Rises once T6050_PHY_CFG_BIT4 is set; polled by macOS and by m1n1; meaning not known */
#define T6050_PHY_STAT_BIT0		BIT(0)
/*
 * Set and held once the shared PHY is started in macOS's post-iBoot order (aura's
 * T6050 bring-up sets T6050_PHY_CFG_BIT4 and waits for it); meaning not known
 */
#define T6050_PHY_STAT_BIT4		BIT(4)
#define T6050_PHYCMN_CLK		0x0
#define T6050_PHYCMN_CLK_MODE		GENMASK(1, 0)
/*
 * Written before CORE_RC_CTL_RUN: a trace of macOS shows 0x140 written here, and m1n1
 * writes the same. Neither bit's meaning is known.
 */
#define CORE_RC_T6050_PRE_RUN		0x54
#define   CORE_RC_T6050_PRE_RUN_BIT6	BIT(6)
#define   CORE_RC_T6050_PRE_RUN_BIT8	BIT(8)

static int apple_pcie_t6050_phy_wait(struct apple_pcie *pcie, u32 mask, u32 value)
{
	u32 v;
	int ret;

	ret = readl_poll_timeout(pcie->t6050_phy + T6050_PHY_CTRL, v, (v & mask) == value,
				 10, 50000);
	if (ret)
		dev_err(pcie->dev, "shared PHY: %#x, waiting for %#x/%#x\n", v, value, mask);
	return ret;
}

/*
 * A host tunable list (apple,tunable-<name>, groups of (offset, 4, mask, value)) applied
 * to the host window @win. The window may contain windows mapped elsewhere (the PHY
 * window holds the port and shared PHYs), so it is mapped without claiming it.
 */
static int apple_pcie_t6050_tune_host(struct apple_pcie *pcie, const char *name,
				      const char *win)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct device_node *np = pcie->dev->of_node;
	struct resource *res;
	void __iomem *base;
	char prop[48];
	u32 *cells;
	int count, i, ret;

	snprintf(prop, sizeof(prop), "apple,tunable-%s", name);
	if (!of_property_present(np, prop))
		return 0;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, win);
	count = of_property_count_u32_elems(np, prop);
	if (!res)
		return dev_err_probe(pcie->dev, -EINVAL, "tunable %s names missing reg window %s\n",
				     prop, win);
	if (count <= 0 || count % 4)
		return dev_err_probe(pcie->dev, -EINVAL,
				     "tunable %s has %d cells, want a 4-cell multiple\n", prop, count);
	cells = kmalloc_array(count, sizeof(*cells), GFP_KERNEL);
	if (!cells)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, prop, cells, count);
	for (i = 0; !ret && i < count; i += 4)
		if (cells[i + 1] != 4 || (cells[i] & 3) || cells[i] > resource_size(res) - 4)
			ret = -EINVAL;
	if (!ret) {
		base = ioremap(res->start, resource_size(res));
		if (base) {
			for (i = 0; i < count; i += 4)
				apple_pcie_t6050_rmw(base + cells[i], cells[i + 2], cells[i + 3]);
			iounmap(base);
		} else {
			ret = -ENOMEM;
		}
	}
	kfree(cells);
	if (ret)
		dev_err(pcie->dev, "%s: %d\n", prop, ret);
	return ret;
}

static int apple_pcie_t6050_shared_phy_on(struct apple_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	const u32 up = T6050_PHY_CTRL_CLK0ACK | T6050_PHY_CTRL_CLK1ACK | T6050_PHY_CTRL_LANES;
	void __iomem *phy, *cmn;
	u32 v;
	int ret;

	if (!pcie->t6050_phy) {
		phy = devm_platform_ioremap_resource_byname(pdev, "phy-shared");
		if (IS_ERR(phy))
			return PTR_ERR(phy);
		pcie->t6050_phy = phy;
	}
	phy = pcie->t6050_phy;
	if ((readl(phy + T6050_PHY_CTRL) & (up | T6050_PHY_CTRL_RESET_T8132 |
					    T6050_PHY_CTRL_RESET_T8103)) == up)
		return 0;
	/*
	 * A boot loader that started the PHY in macOS's order (it trained another port)
	 * leaves status bit 4 set and the RC ready: the PHY runs, so it is not restarted
	 * under that port's link.
	 */
	if ((readl(phy + T6050_PHY_STAT) & T6050_PHY_STAT_BIT4) &&
	    (readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY))
		return 0;
	cmn = devm_platform_ioremap_resource_byname(pdev, "phy-common");
	if (IS_ERR(cmn))
		return PTR_ERR(cmn);

	/* Fabric, clock generator and PHY tunables, then the PHY clocks (m1n1's order) */
	ret = apple_pcie_t6050_tune_host(pcie, "axi2af", "axi2af") ?:
	      apple_pcie_t6050_tune_host(pcie, "cio3pllcore", "pcieclkgen") ?:
	      apple_pcie_t6050_tune_host(pcie, "pcieclkgen", "pcieclkgen") ?:
	      apple_pcie_t6050_tune_host(pcie, "phy", "phy-window");
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CTRL, 0, T6050_PHY_CTRL_CLK0REQ);
	ret = apple_pcie_t6050_phy_wait(pcie, T6050_PHY_CTRL_CLK0ACK, T6050_PHY_CTRL_CLK0ACK);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CTRL, 0, T6050_PHY_CTRL_CLK1REQ);
	ret = apple_pcie_t6050_phy_wait(pcie, T6050_PHY_CTRL_CLK1ACK, T6050_PHY_CTRL_CLK1ACK);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CTRL, T6050_PHY_CTRL_RESET_T8132, 0);
	ret = apple_pcie_t6050_phy_wait(pcie, T6050_PHY_CTRL_RESET_T8132, 0);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CTRL, T6050_PHY_CTRL_RESET_T8103, 0);
	ret = apple_pcie_t6050_phy_wait(pcie, T6050_PHY_CTRL_RESET_T8103, 0);
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CFG, 0, T6050_PHY_CFG_BIT0);
	ret = apple_pcie_t6050_tune_host(pcie, "phy-ip-pll", "phy-ip") ?:
	      apple_pcie_t6050_tune_host(pcie, "phy-ip-auspma", "phy-ip");
	if (ret)
		return ret;
	apple_pcie_t6050_rmw(phy + T6050_PHY_CFG, 0, T6050_PHY_CFG_BIT4);
	apple_pcie_t6050_rmw(cmn + T6050_PHYCMN_CLK, T6050_PHYCMN_CLK_MODE,
			     FIELD_PREP(T6050_PHYCMN_CLK_MODE, 1));
	if (readl_poll_timeout(phy + T6050_PHY_STAT, v, v & T6050_PHY_STAT_BIT0, 10, 250000))
		dev_info(pcie->dev, "shared PHY: clock status %#x\n", v);
	apple_pcie_t6050_rmw(phy + T6050_PHY_CTRL, 0, T6050_PHY_CTRL_LANES);
	return 0;
}

/* The port's DART answers once the root complex is up (apple,manual-availability) */
static void apple_pcie_dart_available(struct apple_pcie_port *port)
{
	struct of_phandle_args iommu = {};

	if (of_map_iommu_id(port->pcie->dev->of_node, port->adopted_bus << 8, &iommu) ||
	    !iommu.np)
		return;
	apple_dart_make_available(iommu.np);
	of_node_put(iommu.np);
}

static int apple_pcie_t6050_cold_train(struct apple_pcie_port *port)
{
	struct apple_pcie *pcie = port->pcie;
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct gpio_desc *pwren;
	void __iomem *dbi;
	char name[16];
	u32 v, bus;
	int ret;

	bus = pcie->cfg->busr.start + 1 + port->idx;
	if (bus > pcie->cfg->busr.end)
		return -EINVAL;

	/*
	 * Both lines before the first register write: a line that is not there yet (the SMC
	 * GPIO bank probes late) defers the probe with the hardware untouched. The endpoint
	 * is held off from here. A restart resets the SoC but not the endpoint: after one
	 * without a shutdown that powered it off, the J716s N1 still runs its firmware, and
	 * only a power cycle of 250 ms (what m1n1 measured for a warm N1) brings it back in
	 * its ROM personality.
	 */
	if (!port->perst)
		port->perst = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(port->np),
						    "reset", GPIOD_ASIS, "PERST#");
	if (IS_ERR(port->perst))
		return PTR_ERR(port->perst);
	pwren = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(port->np), "pwren",
				      GPIOD_OUT_LOW, "PWREN");
	if (IS_ERR(pwren))
		return PTR_ERR(pwren);
	port->pwren = pwren;

	ret = apple_pcie_t6050_shared_phy_on(pcie);
	if (ret)
		return ret;
	if (!(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN)) {
		writel(CORE_RC_T6050_PRE_RUN_BIT6 | CORE_RC_T6050_PRE_RUN_BIT8,
		       pcie->base + CORE_RC_T6050_PRE_RUN);
		writel(CORE_RC_CTL_RUN, pcie->base + CORE_RC_CTL);
	}
	ret = readl_poll_timeout(pcie->base + CORE_RC_STAT, v, v & CORE_RC_STAT_READY, 10, 250000);
	if (ret)
		return ret;

	snprintf(name, sizeof(name), "intr2axi%d", port->idx);
	if (!port->intr2axi)
		port->intr2axi = devm_platform_ioremap_resource_byname(pdev, name);
	if (IS_ERR(port->intr2axi))
		return PTR_ERR(port->intr2axi);
	ret = apple_pcie_t6050_perst(port, true);
	if (ret)
		return ret;

	/* The endpoint held, then powered: the boot loader leaves it off */
	msleep(250);
	/* From here the endpoint may be on, and a failure has to park the port */
	port->powered = true;
	ret = gpiod_set_value_cansleep(pwren, 1);
	if (ret)
		return ret;
	msleep(150);

	ret = apple_pcie_t6050_port_enable(port) ?:
	      apple_pcie_t6050_port_configure(port) ?:
	      apple_pcie_t6050_port_train(port);
	if (ret)
		return ret;

	/* Primary bus 0, the port's secondary and subordinate bus, as a boot loader leaves them */
	dbi = apple_pcie_t6050_dbi(port);
	v = readl(dbi + PCI_PRIMARY_BUS) & 0xff000000;
	writel(v | (bus << 8) | (bus << 16), dbi + PCI_PRIMARY_BUS);
	dev_info(pcie->dev, "port %d: trained by Linux, the boot loader left it down\n", port->idx);
	return 0;
}

/*
 * PME_Turn_Off, then wait for L2: the clock gate disabled, the message sent, and the
 * link reported in L2. Both waits are made even when the first fails (the N1 may
 * clear the send bit and never reach L2, as under macOS); the first error is returned.
 */
static int apple_pcie_t6050_port_l2(struct apple_pcie_port *port)
{
	int ret, ret2;

	apple_pcie_port_writel(port, apple_pcie_port_readl(port, PORT_APPCLK) | PORT_APPCLK_CGDIS,
			       PORT_APPCLK);
	apple_pcie_port_writel(port, PORT_PME_TURN_OFF_SEND | PORT_PME_TURN_OFF_BIT4,
			       PORT_PME_TURN_OFF);
	ret = apple_pcie_t6050_wait(port, port->base, PORT_PME_TURN_OFF, PORT_PME_TURN_OFF_SEND,
				    0, 500000);
	ret2 = apple_pcie_t6050_wait(port, port->base, PORT_LINKSTS, PORT_LINKSTS_L2,
				     PORT_LINKSTS_L2, 500000);
	return ret ?: ret2;
}

/*
 * Put a T6050 port back the way the boot stage leaves it on a cold start: link to L2
 * (only for a link that is up: PME_Turn_Off has no one to answer on a link that never
 * trained), PERST# asserted, lane clock request off, the port disabled and its clocks
 * stopped, and the endpoint's power off if Linux raised it. Errors are ignored: each
 * step is best effort and the rest still has to run. Config space is not touched.
 */
static void apple_pcie_t6050_port_park(struct apple_pcie_port *port, bool link_up)
{
	if (link_up)
		apple_pcie_t6050_port_l2(port);
	WRITE_ONCE(port->link_failed, true);
	apple_pcie_t6050_perst(port, true);
	writel(readl(port->phy) & ~PHY_LANE_CFG_REFCLKCGEN_B31, port->phy);
	apple_pcie_t6050_port_disable(port);
	/* An endpoint Linux powered goes off, so the next boot finds it cold */
	if (port->pwren)
		gpiod_set_value_cansleep(port->pwren, 0);
	port->powered = false;
}

/*
 * A port this driver cold-trained whose setup then failed: its link may be up and its
 * endpoint is powered, and nothing else will stop them (the shutdown hook walks
 * pcie->ports, which no longer holds this port). Called after apple_pcie_port_teardown(),
 * which still needs the port's registers clocked. A failure before the endpoint was
 * powered left nothing running: pwren was requested low and no port register was written.
 */
static void apple_pcie_t6050_cold_unwind(struct apple_pcie_port *port)
{
	bool link_up;

	if (!port->powered)
		return;
	link_up = apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP;
	/* The bus numbers cold_train wrote, as the shutdown hook clears them */
	if (link_up) {
		void __iomem *dbi = apple_pcie_t6050_dbi(port);

		writel(readl(dbi + PCI_PRIMARY_BUS) & 0xff0000ff, dbi + PCI_PRIMARY_BUS);
	}
	apple_pcie_t6050_port_park(port, link_up);
	dev_info(port->pcie->dev, "port %d parked after a failed setup, its endpoint powered off\n",
		 port->idx);
}

/* The reset table cleared the routes and the MSI setup: put back what Linux set */
static int apple_pcie_t6050_port_routes(struct apple_pcie_port *port, const u32 *routes)
{
	struct apple_pcie *pcie = port->pcie;
	int ret = 0, slot, i;

	for_each_set_bit(slot, &port->adopted_slots, BITS_PER_LONG)
		if (apple_pcie_rid2sid_write(port, slot, routes[slot]) != routes[slot])
			ret = -EIO;
	apple_pcie_port_writel(port, lower_32_bits(DOORBELL_ADDR), pcie->hw->port_msiaddr);
	if (pcie->hw->port_msiaddr_hi)
		apple_pcie_port_writel(port, 0, pcie->hw->port_msiaddr_hi);
	for (i = 0; i < pcie->nvecs; i++)
		apple_pcie_port_writel(port, FIELD_PREP(PORT_MSIMAP_TARGET, pcie->msi_vector_base + i) |
				       PORT_MSIMAP_ENABLE, pcie->hw->port_msimap + 4 * i);
	writel(PCIEC_INTR2AXI_ENABLE, port->intr2axi + PCIEC_INTR2AXI_CTRL);
	readl(port->intr2axi + 0x84);
	apple_pcie_port_writel(port, PORT_MSICFG_EN, PORT_MSICFG);
	return ret;
}

/*
 * Root-port reset of an adopted T6050 port, for an endpoint that must come back on a
 * retrained link (the N1 after its ROM boot): PME_Turn_Off to L2, PERST# asserted, the
 * port disabled, reset, enabled and configured, its routes and MSI map rewritten, then the
 * lane PHY, PERST# release and LTSSM to a Gen3 x1 link. The caller has quiesced the
 * endpoint and saved the root port's state; the PCI core restores it.
 */
static int apple_pcie_t6050_reset_root_port(struct pci_host_bridge *bridge,
					    struct pci_dev *rp)
{
	struct pci_config_window *cfg = rp->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct apple_pcie_port *port = NULL, *p;
	u32 routes[BITS_PER_LONG], value;
	int ret, slot;

	if (!pcie || !pcie->hw->adopt_link || !pci_is_root_bus(rp->bus))
		return -EOPNOTSUPP;
	/* Down for a firmware sleep, or since a wake that failed: nothing there is read */
	if (READ_ONCE(pcie->t6050_down))
		return -EHOSTDOWN;
	list_for_each_entry(p, &pcie->ports, entry)
		if (p->idx == PCI_SLOT(rp->devfn))
			port = p;
	if (!port || !port->perst || !port->intr2axi)
		return -EOPNOTSUPP;

	mutex_lock(&pcie->lock);
	for_each_set_bit(slot, &port->adopted_slots, BITS_PER_LONG)
		routes[slot] = apple_pcie_port_readl(port, port_rid2sid_offset(port, slot));

	/* Bridge decoding off and the secondary bus unnumbered while the link is down */
	ret = pcibios_err_to_errno(pci_write_config_word(rp, PCI_COMMAND, 0)) ?:
	      pcibios_err_to_errno(pci_read_config_dword(rp, PCI_PRIMARY_BUS, &value)) ?:
	      pcibios_err_to_errno(pci_write_config_dword(rp, PCI_PRIMARY_BUS,
							  value & 0xff0000ff));
	if (ret)
		goto out;
	/* PME_Turn_Off, then L2 */
	ret = apple_pcie_t6050_port_l2(port);
	if (ret)
		goto out;
	WRITE_ONCE(port->link_failed, true);
	ret = apple_pcie_t6050_perst(port, true);
	if (ret)
		goto out;
	writel(readl(port->phy) & ~PHY_LANE_CFG_REFCLKCGEN_B31, port->phy);
	ret = apple_pcie_t6050_port_restart(port);
	if (ret)
		goto out;

	ret = apple_pcie_t6050_port_routes(port, routes);
	if (ret)
		goto out;

	ret = apple_pcie_t6050_port_train(port);
	if (ret)
		goto out;
	/* Link interrupts stay masked after the reset table: the link is checked on access */
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	WRITE_ONCE(port->link_failed, false);
	dev_info(pcie->dev, "port %d: link retrained after a root-port reset\n", port->idx);
out:
	mutex_unlock(&pcie->lock);
	if (ret)
		dev_err(pcie->dev, "port %d: root-port reset stopped: %d\n", port->idx, ret);
	return ret;
}

static int apple_pcie_setup_link(struct apple_pcie *pcie,
				 struct apple_pcie_port *port,
				 struct device_node *np)
{
#define MAX_AUX_PERST 3
	struct gpio_desc *aux_reset[MAX_AUX_PERST] = { NULL };
	u32 num_aux_resets = 0;
	struct gpio_desc *reset, *pwren = NULL;
	u32 stat;
	int ret;

	/* Bringing the link up again, after a deep sleep. */
	if (port->perst_gpio) {
		reset = port->perst_gpio;
		pwren = port->pwren_gpio;
		goto have_gpios;
	}

	/*
	 * Assert PERST# and configure the pin as output.
	 * The Aquantia AQC113 10GB nic used desktop macs is sensitive to
	 * deasserting it without prior clock setup.
	 * Observed on M1 Max/Ultra Mac Studios under m1n1's hypervisor.
	 */
	reset = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(np), "reset",
				      GPIOD_OUT_HIGH, "PERST#");
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	// HACK: use additional "reset-gpios" until pci-pwrctrl gains PERST# support.
	for (u32 idx = 0; idx < MAX_AUX_PERST; idx++) {
		aux_reset[idx] = devm_fwnode_gpiod_get_index(pcie->dev,
							     of_fwnode_handle(np),
							     "reset", idx + 1,
							     GPIOD_OUT_HIGH,
							     "PERST#");
		if (IS_ERR(aux_reset[idx])) {
			if (PTR_ERR(aux_reset[idx]) == -ENOENT)
				break;
			else
				return PTR_ERR(aux_reset[idx]);
		}
		num_aux_resets++;
	}
	dev_info(pcie->dev, "Using %u auxiliary PERST#\n", num_aux_resets);
	if (!num_aux_resets)
		port->perst = reset;

	pwren = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(np), "pwren",
					    GPIOD_ASIS, "PWREN");
	if (IS_ERR(pwren)) {
		if (PTR_ERR(pwren) == -ENOENT)
			pwren = NULL;
		else
			return PTR_ERR(pwren);
	}
	if (!num_aux_resets) {
		port->perst_gpio = reset;
		port->pwren_gpio = pwren;
	}

have_gpios:
	rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);

	/* Assert PERST# before setting up the clock */
	gpiod_set_value_cansleep(reset, 1);
	for (u32 idx = 0; idx < num_aux_resets; idx++)
		gpiod_set_value_cansleep(aux_reset[idx], 1);

	/* Power-cycle devices which cannot inherit their firmware state. */
	if (pcie->hw->pwren_off_ms) {
		if (!pwren)
			return -EINVAL;
		rmw_clear(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
		ret = gpiod_set_value_cansleep(pwren, 0);
		if (ret)
			return ret;
		msleep(pcie->hw->pwren_off_ms);
	}
	ret = gpiod_set_value_cansleep(pwren, 1);
	if (ret)
		return ret;

	if (!pcie->hw->retain_clocks) {
		ret = apple_pcie_setup_refclk(pcie, port);
		if (ret < 0)
			return ret;
	}

	/*
	 * The minimal Tperst-clk value is 100us (PCIe CEM r5.0, 2.9.2)
	 * If powering up, the minimal Tpvperl is 100ms
	 */
	if (pwren)
		msleep(pcie->hw->pwren_on_ms ?: 100);
	else
		usleep_range(100, 200);

	/* Deassert PERST# */
	rmw_set(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
	ret = gpiod_set_value_cansleep(reset, 0);
	if (ret)
		return ret;
	for (u32 idx = 0; idx < num_aux_resets; idx++)
		gpiod_set_value_cansleep(aux_reset[idx], 0);

	if (pcie->hw->pipe_rst) {
		ret = readl_relaxed_poll_timeout(port->base + PORT_T8152_PIPE_RST_STS,
						 stat, stat & PORT_T8152_PIPE_RST_STS_LANES,
						 1, 20000);
		if (ret < 0) {
			dev_err(pcie->dev, "port %pOF lanes stuck in PIPE reset\n", np);
			return ret;
		}
		rmw_clear(PORT_T8152_PIPE_RST, port->base + pcie->hw->port_perst);
	}

	/* Wait for 100ms after PERST# deassertion (PCIe r5.0, 6.6.1) */
	msleep(100);

	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					 stat & PORT_STATUS_READY, 100, 250000);
	if (ret < 0) {
		dev_err(pcie->dev, "port %pOF ready wait timeout\n", np);
		return ret;
	}

	return 0;
}

/* DesignWare port logic, in the root port's configuration space. */
#define DWC_PORT_DEBUG0			0x728
#define   DWC_PORT_DEBUG0_LTSSM		GENMASK(5, 0)

static u32 apple_pcie_port_ltssm(struct apple_pcie *pcie,
				 struct apple_pcie_port *port)
{
	return FIELD_GET(DWC_PORT_DEBUG0_LTSSM,
			 readl_relaxed(pcie->cfg->win + (port->idx << 15) +
				       DWC_PORT_DEBUG0));
}

/*
 * Give an endpoint that did not train another fundamental reset: stop the
 * LTSSM, assert PERST#, and release it with the timing of the first bring-up
 * (Tperst of at least 100 us, PCIe CEM r5.0 2.9.2; 100 ms before
 * configuration, PCIe r5.0 6.6.1).
 */
static int apple_pcie_port_reset_endpoint(struct apple_pcie *pcie,
					  struct apple_pcie_port *port)
{
	u32 stat;
	int ret;

	apple_pcie_port_rmw_clear(port, PORT_LTSSMCTL_START, PORT_LTSSMCTL);
	rmw_clear(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
	ret = gpiod_set_value_cansleep(port->perst, 1);
	if (ret)
		return ret;
	usleep_range(100, 200);
	rmw_set(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
	ret = gpiod_set_value_cansleep(port->perst, 0);
	if (ret)
		return ret;
	msleep(100);
	return readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					  stat & PORT_STATUS_READY, 100, 250000);
}

static int apple_pcie_setup_port(struct apple_pcie *pcie,
				 struct device_node *np)
{
	struct platform_device *platform = to_platform_device(pcie->dev);
	struct apple_pcie_port *port;
	struct resource *res;
	char name[16];
	u32 link_stat, preinit_status, stat, idx;
	int ret, i;

	port = devm_kzalloc(pcie->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->sid_map = devm_bitmap_zalloc(pcie->dev, pcie->hw->max_rid2sid, GFP_KERNEL);
	if (!port->sid_map)
		return -ENOMEM;
	port->saved_rid2sid = devm_kcalloc(pcie->dev, pcie->hw->max_rid2sid,
					 sizeof(*port->saved_rid2sid), GFP_KERNEL);
	if (!port->saved_rid2sid)
		return -ENOMEM;

	ret = of_property_read_u32_index(np, "reg", 0, &idx);
	if (ret)
		return ret;

	/* Use the first reg entry to work out the port index */
	port->idx = idx >> 11;
	port->pcie = pcie;
	port->np = of_node_get(np);

	raw_spin_lock_init(&port->lock);
	raw_spin_lock_init(&port->irq_lock);
	INIT_LIST_HEAD(&port->entry);

	snprintf(name, sizeof(name), "port%d", port->idx);
	res = platform_get_resource_byname(platform, IORESOURCE_MEM, name);
	if (!res)
		res = platform_get_resource(platform, IORESOURCE_MEM, port->idx + 2);
	if (!res) {
		ret = -ENODEV;
		goto err_teardown;
	}

	/*
	 * PCIe-C register transactions are non-posted on the tunneled fabric.
	 * Mark the resource so devm_ioremap_resource() selects ioremap_np() on
	 * arm64, matching the successful /dev/mem Device-nGnRnE replay.
	 */
	if (pcie->hw->tunneled)
		res->flags |= IORESOURCE_MEM_NONPOSTED;

	port->base = devm_ioremap_resource(&platform->dev, res);
	if (IS_ERR(port->base)) {
		ret = PTR_ERR(port->base);
		port->base = NULL;
		goto err_teardown;
	}
	if (pcie->managed_sids) {
		ret = apple_pcie_managed_admit(port);
		if (ret)
			goto err_teardown;
	}
	if (pcie->hw->tunneled) {
		port->tunable = devm_apple_tunable_parse(pcie->dev, np,
							  "apple,tunable", res);
		if (IS_ERR(port->tunable)) {
			ret = dev_err_probe(pcie->dev, PTR_ERR(port->tunable),
					    "port %pOF tunables unavailable\n", np);
			goto err_teardown;
		}
	}

	if (!pcie->hw->tunneled) {
		snprintf(name, sizeof(name), "phy%d", port->idx);
		res = platform_get_resource_byname(platform, IORESOURCE_MEM, name);
		if (res)
			port->phy = devm_ioremap_resource(&platform->dev, res);
		else
			port->phy = pcie->base + CORE_PHY_DEFAULT_BASE(port->idx);
		if (IS_ERR(port->phy)) {
			ret = PTR_ERR(port->phy);
			port->phy = NULL;
			goto err_teardown;
		}
	}
	if (pcie->hw->tunneled) {
		bool preinit_ok;

		/*
		 * m1n1 owns PCIe-C cold initialization on T6020. Replaying the
		 * port reset against that live handoff raises an asynchronous
		 * SError. t8103 boots with ATC_PCIE off and no handoff, so
		 * apple,pciec-kernel-init selects the in-kernel sequence on each
		 * supported generation.
		 */
		ret = of_property_read_u32(pcie->dev->of_node,
					   "apple,pciec-preinit-status",
					   &preinit_status);
		preinit_ok = !ret && preinit_status == 1;
		if ((pcie->kernel_init || pcie->tunnel_cold_init) && !preinit_ok) {
			stat = apple_pcie_port_readl(port, PORT_STATUS);
			if (stat & PORT_STATUS_READY) {
				dev_info(pcie->dev,
					 "port %pOF already clocked, status %#x\n",
					 np, stat);
			} else {
				ret = apple_pcie_tunnel_cold_init(port);
				if (ret)
					goto err_teardown;
			}
		} else if (!preinit_ok) {
			dev_err(pcie->dev,
				"PCIe-C requires a firmware handoff or apple,pciec-kernel-init\n");
			ret = -ENODEV;
			goto err_teardown;
		} else {
			ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
						       stat & PORT_STATUS_READY,
						       100, 250000, false, port,
						       PORT_STATUS);
			if (ret < 0) {
				dev_info(pcie->dev,
					 "port %pOF stopped; replaying PCIe-C hardware initialization\n",
					 np);
				ret = apple_pcie_tunnel_reinitialize(port);
				if (ret)
					goto err_teardown;
			}
		}
	} else if (pcie->hw->adopt_link) {
		ret = apple_pcie_adopt_link(port);
		/* A port left down that names its endpoint's power line is trained here */
		if (ret == -ENOLINK && of_property_present(np, "pwren-gpios")) {
			ret = apple_pcie_t6050_cold_train(port);
			if (!ret)
				ret = apple_pcie_adopt_link(port);
		}
		if (ret)
			goto err_teardown;
		apple_pcie_dart_available(port);
	} else {
		if (pcie->hw->retain_clocks)
			rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);
		if (pcie->hw->retain_clocks &&
		    !(apple_pcie_port_readl(port, PORT_STATUS) & PORT_STATUS_READY)) {
			ret = dev_err_probe(pcie->dev, -ENODEV,
					    "port %pOF requires bootloader initialization\n",
					    np);
			goto err_teardown;
		}
		/* U-Boot may already have brought up a conventional root port. */
		link_stat = apple_pcie_port_readl(port, PORT_LINKSTS);
		if (!(link_stat & PORT_LINKSTS_UP)) {
			if (pcie->managed_sids) {
				ret = apple_pcie_radio_check_cold(port);
				if (ret)
					goto err_teardown;
			}
			ret = apple_pcie_setup_link(pcie, port, np);
			if (ret)
				goto err_teardown;
		}
	}

	if (!pcie->hw->retain_clocks) {
		if (pcie->hw->port_refclk)
			rmw_clear(PORT_REFCLK_CGDIS,
				  port->base + pcie->hw->port_refclk);
		else if (port->phy)
			rmw_set(PHY_LANE_CFG_REFCLKCGEN,
				port->phy + PHY_LANE_CFG);

		/* PCIe-C APPCLK state is part of the m1n1 handoff. */
		if (!pcie->hw->tunneled)
			rmw_clear(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);
	}

	ret = apple_pcie_port_setup_irq(port);
	if (ret)
		goto err_teardown;

	if (pcie->managed_sids || pcie->hw->adopt_link)
		goto rid2sid_ready;

	/* Reset all RID/SID mappings, and check for RAZ/WI registers */
	for (i = 0; i < pcie->hw->max_rid2sid; i++) {
		if (apple_pcie_rid2sid_write(port, i, 0xbad1d) != 0xbad1d)
			break;
		apple_pcie_rid2sid_write(port, i, 0);
	}

	dev_dbg(pcie->dev, "%pOF: %d RID/SID mapping entries\n", np, i);

	port->sid_map_sz = i;
rid2sid_ready:
	list_add_tail(&port->entry, &pcie->ports);
	init_completion(&pcie->event);

	ret = apple_pcie_port_register_irqs(port);
	if (ret)
		goto err_teardown;

	link_stat = apple_pcie_port_readl(port, PORT_LINKSTS);
	if (!(link_stat & PORT_LINKSTS_UP)) {
		unsigned long timeout, left = 0;
		int attempt, attempts = 1;

		if (pcie->hw->adopt_link) {
			ret = -ENOLINK;
			goto err_teardown;
		}
		if (pcie->hw->tunneled) {
			if (pcie->kernel_init) {
				apple_pcie_tunnel_apply_tunable(port->base,
								port->tunable);
				udelay(10);
				apple_pcie_tunnel_apply_tunable(pcie->cfg->win,
								pcie->rc_tunable);
				udelay(10);
				attempts = 3;
			}
			ret = apple_pcie_tunnel_release_reset(port);
			if (ret)
				goto err_teardown;
		}

		/*
		 * In about one boot in seven the J616s SD reader is detected
		 * but sends no training sets after the first PERST# release,
		 * and the LTSSM parks in Polling.Compliance. A Retrain Link
		 * from the root port does not recover it; a new fundamental
		 * reset does.
		 */
		if (!pcie->hw->tunneled && port->perst)
			attempts = 1 + LINK_RESET_RETRIES;

		timeout = link_up_timeout * HZ / 1000;
		for (attempt = 0; attempt < attempts; attempt++) {
			if (attempt && pcie->hw->tunneled)
				reinit_completion(&pcie->event);
			apple_pcie_port_writel(port, PORT_LTSSMCTL_START,
					       PORT_LTSSMCTL);
			left = wait_for_completion_timeout(&pcie->event, timeout);
			if (!left && (apple_pcie_port_readl(port, PORT_LINKSTS) &
				      PORT_LINKSTS_UP))
				left = 1;
			if (left)
				break;
			if (pcie->hw->tunneled || attempt + 1 == attempts) {
				dev_warn(pcie->dev,
					 "%pOF link didn't come up (LTSSM %#x)\n",
					 np, pcie->hw->tunneled ? 0 :
					 apple_pcie_port_ltssm(pcie, port));
				continue;
			}
			dev_info(pcie->dev,
				 "%pOF link down after %dms (LTSSM %#x), resetting the endpoint\n",
				 np, link_up_timeout, apple_pcie_port_ltssm(pcie, port));
			/* A link-up during the reset must not be lost. */
			reinit_completion(&pcie->event);
			ret = apple_pcie_port_reset_endpoint(pcie, port);
			if (ret) {
				dev_warn(pcie->dev, "%pOF endpoint reset failed: %d\n",
					 np, ret);
				break;
			}
		}
		if (left)
			dev_info(pcie->dev, "%pOF link up after %ldms\n", np,
				 (timeout - left) * 1000 / HZ);
		else if (pcie->managed_sids) {
			ret = -ETIMEDOUT;
			goto err_teardown;
		}
	}
	if (pcie->hw->tunneled &&
	    (apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP))
		apple_pcie_tunnel_link_up(pcie);
	if (pcie->hw->tunneled)
		WRITE_ONCE(port->started, true);
	if (pcie->hw->root_bus_only)
		dev_dbg(pcie->dev, "port %pOF RUN=%#x link=%#x; downstream config %s\n",
			np, apple_pcie_port_readl(port, PORT_STATUS),
			apple_pcie_port_readl(port, PORT_LINKSTS),
			pcie->piodma_supplier ? "waits for the bootstrap" : "blocked");

	return 0;

err_teardown:
	apple_pcie_port_teardown(port);
	apple_pcie_t6050_cold_unwind(port);
	return ret;
}

/*
 * Each MSI vector is an AIC interrupt of its own, so a vector can wake the
 * system exactly when the AIC line behind it can. Forward the wake setting
 * from the per-device PCI/MSI chip down to the AIC, which otherwise fails
 * enable_irq_wake() for every MSI and leaves PCIe functions without a way
 * to end suspend-to-idle.
 */
static bool apple_msi_init_dev_msi_info(struct device *dev,
					struct irq_domain *domain,
					struct irq_domain *real_parent,
					struct msi_domain_info *info)
{
	if (!msi_lib_init_dev_msi_info(dev, domain, real_parent, info))
		return false;

	if (!info->chip->irq_set_wake)
		info->chip->irq_set_wake = irq_chip_set_wake_parent;
	return true;
}

static const struct msi_parent_ops apple_msi_parent_ops = {
	.supported_flags	= (MSI_GENERIC_FLAGS_MASK	|
				   MSI_FLAG_PCI_MSIX		|
				   MSI_FLAG_MULTI_PCI_MSI),
	.required_flags		= (MSI_FLAG_USE_DEF_DOM_OPS	|
				   MSI_FLAG_USE_DEF_CHIP_OPS	|
				   MSI_FLAG_PCI_MSI_MASK_PARENT),
	.chip_flags		= MSI_CHIP_FLAG_SET_EOI,
	.bus_select_token	= DOMAIN_BUS_PCI_MSI,
	.init_dev_msi_info	= apple_msi_init_dev_msi_info,
};

/* T6030 has one shared AIC MSI window, partitioned by its ADT bridges. */
static bool apple_t6030_msi_slice_valid(u32 base, u32 count, u32 aic_irq,
					u32 aic_base)
{
	return base >= 0x20 && !(base & 0x1f) && base + count <= 256 &&
	       count == 32 && aic_irq == aic_base + base;
}

static int apple_msi_init(struct apple_pcie *pcie)
{
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct irq_domain_info info = {
		.fwnode		= fwnode,
		.ops		= &apple_msi_domain_ops,
		.size		= pcie->nvecs,
		.host_data	= pcie,
	};
	struct of_phandle_args args = {};
	int ret;

	ret = of_parse_phandle_with_args(to_of_node(fwnode), "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(to_of_node(fwnode), "msi-ranges",
					 args.args_count + 1, &pcie->nvecs);
	if (ret) {
		of_node_put(args.np);
		return ret;
	}

	if (pcie->hw->adopt_link) {
		ret = of_property_read_u32(to_of_node(fwnode), "apple,msi-vector-base",
					   &pcie->msi_vector_base);
		if (ret || pcie->msi_vector_base != 64 ||
		    (pcie->nvecs != 32 && pcie->nvecs != 64) ||
		    args.args_count != 4 || args.args[0] != 0 || args.args[1] != 0 ||
		    args.args[2] != 1888 || args.args[3] != IRQ_TYPE_EDGE_RISING ||
		    DOORBELL_ADDR != 0xfffff000)
			goto invalid_slice;
	}

	/*
	 * T6050's PCIe hosts share one MSI window, vector n at AIC 1824 + n.
	 * The internal host's bridges own vectors 0..127 and each PCIe-C
	 * bridge a 32-vector slice of the rest.
	 */
	if (pcie->hw == &t6050_pciec_hw) {
		ret = of_property_read_u32(to_of_node(fwnode), "apple,msi-vector-base",
					   &pcie->msi_vector_base);
		if (ret || pcie->msi_vector_base < 128 || pcie->msi_vector_base % 32 ||
		    pcie->nvecs != 32 ||
		    pcie->msi_vector_base + pcie->nvecs > FIELD_MAX(PORT_MSIMAP_TARGET) + 1 ||
		    args.args_count != 4 || args.args[0] != 0 || args.args[1] != 0 ||
		    args.args[2] != 1824 + pcie->msi_vector_base ||
		    args.args[3] != IRQ_TYPE_EDGE_RISING || DOORBELL_ADDR != 0xfffff000)
			goto invalid_slice;
	}

	/*
	 * The controller-wide ADT count (512) is not a per-host allocation.
	 * Internal PCIe owns vectors 0..31. Each PCIe-C bridge owns a distinct
	 * 32-vector slice. Keep endpoint MSI data local, and route its table
	 * entry to the corresponding shared-window vector.
	 */
	if (pcie->hw == &t8122_pciec_hw || pcie->hw == &t8132_pciec_hw ||
	    pcie->hw == &t6040_pciec_hw) {
		struct resource ecam;
		u32 expected_base, aic_base;

		if (of_address_to_resource(to_of_node(fwnode), 0, &ecam))
			goto invalid_slice;
		/* These ECAM apertures identify the ADT bridges of each SoC. */
		if (of_machine_is_compatible("apple,t6030")) {
			aic_base = 1342;
			switch (ecam.start) {
			case 0x730000000ULL:
				expected_base = 0x20;
				break;
			case 0xb30000000ULL:
				expected_base = 0x40;
				break;
			case 0xf30000000ULL:
				expected_base = 0x60;
				break;
			default:
				goto invalid_slice;
			}
		} else if (of_machine_is_compatible("apple,t6040")) {
			aic_base = 2071;
			switch (ecam.start) {
			case 0x1c50000000ULL:
				expected_base = 0x20;
				break;
			case 0x1c60000000ULL:
				expected_base = 0x40;
				break;
			case 0x1c70000000ULL:
				expected_base = 0x60;
				break;
			default:
				goto invalid_slice;
			}
		} else if (of_machine_is_compatible("apple,t8132")) {
			aic_base = 1506;
			switch (ecam.start) {
			/* The internal port's bridges own vectors 0..95. */
			case 0x1c50000000ULL:
				expected_base = 0x60;
				break;
			case 0x1c60000000ULL:
				expected_base = 0x80;
				break;
			default:
				goto invalid_slice;
			}
		} else {
			goto invalid_slice;
		}
		if (args.args_count != 3 || args.args[0] != 0 ||
		    args.args[2] != IRQ_TYPE_EDGE_RISING)
			goto invalid_slice;
		ret = of_property_read_u32(to_of_node(fwnode),
					   "apple,msi-vector-base",
					   &pcie->msi_vector_base);
		if (!of_find_property(to_of_node(fwnode),
				      "apple,msi-vector-base", NULL) &&
		    aic_base == 1342 && args.args[1] == 1342 && pcie->nvecs == 512) {
			/*
			 * Old experimental boot DTs exposed the global window to
			 * each host. Correct it here so old recovery kernels can
			 * continue using the unchanged shared bootloader DT.
			 */
			pcie->msi_vector_base = expected_base;
			pcie->nvecs = 32;
			args.args[1] += expected_base;
			dev_info(pcie->dev, "correcting legacy MSI window to IRQ %u count 32\n",
				 args.args[1]);
		} else if (ret) {
			goto invalid_slice;
		}
		if (pcie->msi_vector_base != expected_base ||
		    !apple_t6030_msi_slice_valid(pcie->msi_vector_base,
					pcie->nvecs, args.args[1], aic_base))
			goto invalid_slice;
	}

	/* Keep map targets within the supported 8-bit range. */
	if ((pcie->hw == &t8122_pciec_hw || pcie->hw == &t8132_pciec_hw ||
	     pcie->hw == &t6040_pciec_hw) &&
	    (!pcie->nvecs || pcie->nvecs > 256 ||
	     pcie->msi_vector_base >
	     FIELD_MAX(PORT_MSIMAP_TARGET) + 1 - pcie->nvecs)) {
		of_node_put(args.np);
		return dev_err_probe(pcie->dev, -EINVAL, "MSI map out of range\n");
	}
	info.size = pcie->nvecs;

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &pcie->fwspec);
	of_node_put(args.np);

	pcie->bitmap = devm_bitmap_zalloc(pcie->dev, pcie->nvecs, GFP_KERNEL);
	if (!pcie->bitmap)
		return -ENOMEM;

	info.parent = irq_find_matching_fwspec(&pcie->fwspec, DOMAIN_BUS_WIRED);
	if (!info.parent) {
		dev_err(pcie->dev, "failed to find parent domain\n");
		return -ENXIO;
	}

	pcie->msi_domain = msi_create_parent_irq_domain(&info, &apple_msi_parent_ops);
	if (!pcie->msi_domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	return 0;

invalid_slice:
	of_node_put(args.np);
	return dev_err_probe(pcie->dev, -EINVAL,
			     "invalid or missing MSI slice\n");
}

static void apple_pcie_cleanup(void *data)
{
	struct apple_pcie *pcie = data;
	struct apple_pcie_port *port, *tmp;

	list_for_each_entry_safe(port, tmp, &pcie->ports, entry)
		apple_pcie_port_teardown(port);

	if (pcie->msi_domain) {
		irq_domain_remove(pcie->msi_domain);
		pcie->msi_domain = NULL;
	}
}

static struct apple_pcie *apple_pcie_lookup(struct device *dev)
{
	return pci_host_bridge_priv(dev_get_drvdata(dev));
}

static struct apple_pcie_port *apple_pcie_get_port(struct pci_dev *pdev)
{
	struct pci_config_window *cfg = pdev->sysdata;
	struct apple_pcie *pcie;
	struct pci_dev *port_pdev;
	struct apple_pcie_port *port;

	pcie = apple_pcie_lookup(cfg->parent);
	if (WARN_ON(!pcie))
		return NULL;

	/* Find the root port this device is on */
	port_pdev = pcie_find_root_port(pdev);

	/* If finding the port itself, nothing to do */
	if (WARN_ON(!port_pdev) || pdev == port_pdev)
		return NULL;

	list_for_each_entry(port, &pcie->ports, entry) {
		if (port->idx == PCI_SLOT(port_pdev->devfn))
			return port;
	}

	return NULL;
}

/* Native T8132 normal SIDs are nonzero, four-bit, and indexed by SID. */
static int apple_pcie_managed_sid_mask(const u32 *ids, unsigned int count, u16 *mask)
{
	u16 selected = 0;
	unsigned int i;

	if (!count || count > 15)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		if (!ids[i] || ids[i] >= 16 || (selected & BIT(ids[i])))
			return -EINVAL;
		selected |= BIT(ids[i]);
	}
	*mask = selected;
	return 0;
}

static int apple_pcie_managed_word(u16 managed, u32 rid, u32 sid, u32 *word)
{
	if (!rid || rid > 0xffff || !sid || sid >= 16 || !(managed & BIT(sid)))
		return -EINVAL;
	*word = PORT_RID2SID_VALID | (sid << PORT_RID2SID_SID_SHIFT) | rid;
	return 0;
}

static int apple_pcie_managed_admit(struct apple_pcie_port *port)
{
	u32 entries[16];
	unsigned int sid, other;

	for (sid = 0; sid < ARRAY_SIZE(entries); sid++)
		entries[sid] = apple_pcie_port_readl(port, port_rid2sid_offset(port, sid));

	/* Index zero is invalid in the native ordinary-RID contract. */
	if (entries[0] & PORT_RID2SID_VALID) {
		dev_err(port->pcie->dev, "inherited RID entry0 is still valid\n");
		return -EBUSY;
	}
	for (sid = 1; sid < ARRAY_SIZE(entries); sid++) {
		u32 target = (entries[sid] >> PORT_RID2SID_SID_SHIFT) & 0xf;
		u32 rid = entries[sid] & 0xffff;

		if (!(entries[sid] & PORT_RID2SID_VALID))
			continue;
		if (target != sid || (port->pcie->managed_sids & BIT(sid)) ||
		    (port->pcie->managed_sids & BIT(target)) || rid == 0x100 || rid == 0x101) {
			dev_err(port->pcie->dev, "inherited RID[%u] %#x conflicts with ordinary ownership\n",
				sid, entries[sid]);
			return -EBUSY;
		}
		for (other = 1; other < sid; other++)
			if ((entries[other] & PORT_RID2SID_VALID) &&
			    (entries[other] & 0xffff) == rid)
				return -EBUSY;
	}
	/* No writes: every existing foreign or reserved entry stays untouched. */
	port->sid_map_sz = 16;
	return 0;
}

static int apple_pcie_radio_check_cold(struct apple_pcie_port *port)
{
	struct device_node *np = port->pcie->dev->of_node;
	struct of_phandle_args wifi = {}, bt = {};
	struct platform_device *provider;
	u32 status, link;
	unsigned int sid;
	int ret;

	status = apple_pcie_port_readl(port, PORT_STATUS);
	link = apple_pcie_port_readl(port, PORT_LINKSTS);
	if (!(status & PORT_STATUS_READY) ||
	    (link & (PORT_LINKSTS_UP | PORT_LINKSTS_BUSY)))
		return -EBUSY;
	/* A cold endpoint reset is legal only before any ordinary RID is owned. */
	for (sid = 0; sid < 16; sid++)
		if (apple_pcie_port_readl(port, port_rid2sid_offset(port, sid)) &
		    PORT_RID2SID_VALID)
			return -EBUSY;

	/* J713's two exact functions must name the same managed provider. */
	ret = of_map_iommu_id(np, 0x100, &wifi);
	if (ret)
		return ret;
	ret = of_map_iommu_id(np, 0x101, &bt);
	if (ret)
		goto out_wifi;
	if (wifi.np != bt.np || wifi.args_count != 1 || bt.args_count != 1 ||
	    wifi.args[0] != 1 || bt.args[0] != 2) {
		ret = -EINVAL;
		goto out_nodes;
	}
	provider = of_find_device_by_node(wifi.np);
	if (!provider) {
		ret = -EPROBE_DEFER;
		goto out_nodes;
	}
	ret = apple_dart_radio_check_idle(&provider->dev);
	put_device(&provider->dev);
out_nodes:
	of_node_put(bt.np);
out_wifi:
	of_node_put(wifi.np);
	return ret;
}

static int apple_pcie_enable_device(struct pci_host_bridge *bridge, struct pci_dev *pdev)
{
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);
	struct resource *res;
	u32 sid, val, readback, rid = pci_dev_id(pdev);
	struct apple_pcie_port *port;
	struct of_phandle_args iommu_spec = {};
	int idx, err;

	/* A generic rescan must not enable cached children of failed admission. */
	if (pcie->piodma_supplier && !pci_is_root_bus(pdev->bus)) {
		err = apple_piodma_radio_check(pdev);
		if (err)
			return err;
	}

	/* Also cover functions discovered by a later hotplug or rescan. */
	if (pcie->hw->tunneled)
		pdev->dev_flags |= PCI_DEV_FLAGS_NO_D3;

	/*
	 * Endpoint BARs share PCIe-C's tunneled, non-posted MMIO fabric. Mark
	 * them before the function driver maps its BAR so pci_iomap() selects
	 * Device-nGnRnE on arm64.
	 */
	/*
	 * T6020 endpoint BARs fault unless they are mapped non-posted.
	 * t8103 BARs are posted; a non-posted map there is the SError risk.
	 */
	if (pcie->hw->tunneled && !of_machine_is_compatible("apple,t8103"))
		pci_dev_for_each_resource(pdev, res)
			if (res->flags & IORESOURCE_MEM)
				res->flags |= IORESOURCE_MEM_NONPOSTED;

	port = apple_pcie_get_port(pdev);
	if (!port)
		return 0;

	dev_dbg(&pdev->dev, "added to bus %s, index %d\n",
		pci_name(pdev->bus->self), port->idx);

	err = of_map_iommu_id(port->pcie->dev->of_node, rid, &iommu_spec);
	if (err)
		return err;

	of_node_put(iommu_spec.np);
	sid = iommu_spec.args[0];
	if (pcie->hw->adopt_link) {
		if (READ_ONCE(port->link_failed))
			return -ENODEV;
		mutex_lock(&pcie->lock);
		err = apple_pcie_adopted_route(port, rid);
		mutex_unlock(&pcie->lock);
		return err;
	}
	mutex_lock(&port->pcie->lock);
	if (apple_radio_writer_closed(port->pcie)) {
		mutex_unlock(&port->pcie->lock);
		return -EBUSY;
	}

	if (port->pcie->managed_sids || port->pcie->hw->sid_indexed_rid2sid) {
		if (sid >= port->sid_map_sz ||
		    (pcie->managed_sids && (!rid || !sid || sid >= 16 ||
					  !(pcie->managed_sids & BIT(sid)))))
			idx = -EINVAL;
		else if (test_and_set_bit(sid, port->sid_map))
			idx = -EBUSY;
		else
			idx = sid;
	} else {
		idx = bitmap_find_free_region(port->sid_map, port->sid_map_sz, 0);
		if (idx < 0)
			idx = -ENOSPC;
	}
	if (idx >= 0 && pcie->managed_sids) {
		unsigned int other;

		for (other = 0; other < 16; other++) {
			u32 entry = apple_pcie_port_readl(port, port_rid2sid_offset(port, other));

			if ((entry & PORT_RID2SID_VALID) &&
			    (other == sid || (entry & 0xffff) == rid)) {
				bitmap_release_region(port->sid_map, idx, 0);
				idx = -EBUSY;
				break;
			}
		}
	}
	if (idx >= 0) {
		if (pcie->managed_sids) {
			err = apple_pcie_managed_word(pcie->managed_sids, rid, sid, &val);
			if (err) {
				bitmap_release_region(port->sid_map, idx, 0);
				mutex_unlock(&port->pcie->lock);
				return err;
			}
		} else {
			val = PORT_RID2SID_VALID | (sid << PORT_RID2SID_SID_SHIFT) | rid;
		}
		readback = apple_pcie_rid2sid_write(port, idx, val);
		if ((pcie->managed_sids || pcie->hw->sid_indexed_rid2sid) && readback != val) {
			dev_err(&pdev->dev, "RID-to-SID readback %#x expected %#x\n",
				readback, val);
			/* Reserve the failed slot; do not enable or recycle it. */
			mutex_unlock(&port->pcie->lock);
			return -EIO;
		}

		dev_dbg(&pdev->dev, "mapping RID%x to SID%x (index %d)\n",
			rid, sid, idx);
	}

	mutex_unlock(&port->pcie->lock);

	return idx >= 0 ? 0 : idx;
}

static void apple_pcie_disable_device(struct pci_host_bridge *bridge, struct pci_dev *pdev)
{
	struct apple_pcie_port *port;
	u32 rid = pci_dev_id(pdev);
	int idx;

	port = apple_pcie_get_port(pdev);
	if (!port || port->pcie->hw->adopt_link)
		return;

	mutex_lock(&port->pcie->lock);
	if (apple_radio_writer_closed(port->pcie))
		apple_radio_terminal();

	for_each_set_bit(idx, port->sid_map, port->sid_map_sz) {
		u32 val;

		val = apple_pcie_port_readl(port,
					    port_rid2sid_offset(port, idx));
		if ((val & 0xffff) == rid) {
			val = apple_pcie_rid2sid_write(port, idx, 0);
			if (port->pcie->managed_sids && val) {
				dev_err(&pdev->dev,
					"RID invalidation failed; retaining SID slot %d\n", idx);
				break;
			}
			bitmap_release_region(port->sid_map, idx, 0);
			dev_dbg(&pdev->dev, "Released %x (%d)\n", val, idx);
			break;
		}
	}

	mutex_unlock(&port->pcie->lock);
}

static int apple_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct apple_pcie *pcie;
	int ret;

	pcie = apple_pcie_lookup(dev);
	if (WARN_ON(!pcie))
		return -ENOENT;
	pcie->cfg = cfg;

	for_each_available_child_of_node_scoped(dev->of_node, of_port) {
		ret = apple_pcie_setup_port(pcie, of_port);
		if (ret) {
			dev_err(dev, "Port %pOF setup fail: %d\n", of_port, ret);
			/* Adopted ports are independent: one refused leaves the others. */
			if (!pcie->hw->adopt_link)
				return ret;
		}
	}
	if (pcie->hw->adopt_link && list_empty(&pcie->ports))
		return -ENODEV;

	/* The bus scan follows this callback. */
	apple_pcie_tunnel_wait_ready(pcie);

	return 0;
}

/*
 * The T8140 host starts with all downstream accesses blocked.
 * Only a successful owned bootstrap and typed ECAM checks open functions0/1.
 * Generic config callbacks retain their normal native widths and locking.
 */
static void __iomem *apple_pcie_map_bus(struct pci_bus *bus,
				      unsigned int devfn, int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct apple_pcie_port *port;

	if (READ_ONCE(pcie->cio_changed))
		return NULL;
	if (!apple_radio_config_allowed(pcie))
		return NULL;
	if (pcie->hw->adopt_link) {
		/* Down for a firmware sleep, or since a wake that failed: not even the root port */
		if (READ_ONCE(pcie->t6050_down))
			return NULL;
		/* Only adopted ports' root ports, and the endpoint behind each. */
		list_for_each_entry(port, &pcie->ports, entry) {
			if (bus->number == 0) {
				if (devfn != PCI_DEVFN(port->idx, 0))
					continue;
				/* Parked, and a sleep took its registers: nothing there to read or restore */
				if (READ_ONCE(pcie->t6050_cold_lost) & BIT(port->idx))
					return NULL;
				return pci_ecam_map_bus(bus, devfn, where);
			}
			if (bus->number != port->adopted_bus)
				continue;
			if (PCI_SLOT(devfn) || READ_ONCE(port->link_failed))
				return NULL;
			if ((apple_pcie_port_readl(port, PORT_LINKSTS) &
			     (PORT_LINKSTS_UP | PORT_LINKSTS_BUSY)) != PORT_LINKSTS_UP) {
				WRITE_ONCE(port->link_failed, true);
				return NULL;
			}
			return pci_ecam_map_bus(bus, devfn, where);
		}
		return NULL;
	}
	if (!pcie->hw->root_bus_only)
		return pci_ecam_map_bus(bus, devfn, where);

	/* Nothing answers on a host that has lost power and isn't set up again. */
	if (READ_ONCE(pcie->neo_rc_down))
		return NULL;

	if (bus->number != cfg->busr.start) {
		/* Pair with ready publication before consuming the bus number. */
		if (!smp_load_acquire(&pcie->neo_config_ready) ||
		    bus->number != pcie->neo_secondary_bus ||
		    PCI_SLOT(devfn) || PCI_FUNC(devfn) > 1)
			return NULL;
		return pci_ecam_map_bus(bus, devfn, where);
	}
	if (PCI_FUNC(devfn))
		return NULL;

	list_for_each_entry(port, &pcie->ports, entry)
		if (port->idx == PCI_SLOT(devfn))
			return pci_ecam_map_bus(bus, devfn, where);

	return NULL;
}

/*
 * While the link comes back after a sleep, the PCI core's resume_noirq asks the functions for
 * their power state. A function out of reset is in D0, so that is the answer until they are.
 */
static int apple_pcie_config_read(struct pci_bus *bus, unsigned int devfn, int where, int size,
				  u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	unsigned long flags;
	u8 pm_cap;
	int ret;

	if (pcie && READ_ONCE(pcie->neo_radio_waking) && bus->number == pcie->neo_secondary_bus &&
	    !PCI_SLOT(devfn) && PCI_FUNC(devfn) <= 1) {
		pm_cap = pcie->neo_fn_pm_cap[PCI_FUNC(devfn)];
		if (pm_cap && where == pm_cap + PCI_PM_CTRL && size == 2) {
			*val = PCI_D0;
			return PCIBIOS_SUCCESSFUL;
		}
	}
	if (!pcie || !pcie->cio_dart)
		return pci_generic_config_read(bus, devfn, where, size, val);
	/* A deep-sleep stop closes config space under this lock. */
	raw_spin_lock_irqsave(&pcie->cio_cfg_lock, flags);
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	raw_spin_unlock_irqrestore(&pcie->cio_cfg_lock, flags);
	return ret;
}

static int apple_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct apple_pcie_port *port;
	unsigned long flags;
	int ret;

	if (pcie && pcie->cio_dart) {
		raw_spin_lock_irqsave(&pcie->cio_cfg_lock, flags);
		ret = pci_generic_config_write(bus, devfn, where, size, val);
		raw_spin_unlock_irqrestore(&pcie->cio_cfg_lock, flags);
	} else {
		ret = pci_generic_config_write(bus, devfn, where, size, val);
	}
	if (ret || !pcie)
		return ret;
	if (bus->number != cfg->busr.start || devfn)
		return ret;
	/* The root port's link timers follow its max payload size. */
	if (pcie->hw->mps_timers && where == PCIEC_RP_DEVCTL)
		apple_pcie_update_mps_timers(pcie,
					     FIELD_GET(PCI_EXP_DEVCTL_PAYLOAD, val));
	if (!pcie->kernel_init)
		return ret;
	/* Root-port prefetchable window. The port block has to be told too. */
	if (where < 0x24 || where >= 0x30)
		return ret;

	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_writel(port, 1, PORT_PREFMEM_ENABLE);
	return 0;
}

/*
 * The tunnel wait covers only the root port's link. A switch Downstream Port
 * with a slot but no hot-plug capability (an eGPU behind an ASM2464PD) trains
 * its own link later, yet its bus is scanned once: a card that answers after
 * that scan is never enumerated with the tunnel, whose bus numbers and windows
 * are then fixed. Give such a link the same time before its bus is scanned,
 * plus 100 ms (PCIe r7.0 sec 6.6.1 requires it after training above 5 GT/s;
 * it is waited for at every speed here, which is conservative).
 * Hot-plug capable ports are left to pciehp.
 */
static int apple_pcie_add_bus(struct pci_bus *bus)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct pci_dev *port = bus->self;
	unsigned long start = jiffies;
	unsigned long timeout = start +
				msecs_to_jiffies(APPLE_PCIE_TUNNEL_READY_MS);
	u16 lnksta, sltsta;

	if (!port || pci_pcie_type(port) != PCI_EXP_TYPE_DOWNSTREAM ||
	    !(pcie_caps_reg(port) & PCI_EXP_FLAGS_SLOT) ||
	    port->is_hotplug_bridge || !port->link_active_reporting ||
	    !apple_pcie_lookup(cfg->parent)->hw->tunneled)
		return 0;

	if (pcie_capability_read_word(port, PCI_EXP_LNKSTA, &lnksta) ||
	    (lnksta & PCI_EXP_LNKSTA_DLLLA))
		return 0;

	if (pcie_capability_read_word(port, PCI_EXP_SLTSTA, &sltsta))
		return 0;
	pci_info(port, "slot link down before scan (SltSta %#06x), waiting up to %d ms\n",
		 sltsta, APPLE_PCIE_TUNNEL_READY_MS);
	do {
		msleep(10);
		if (pcie_capability_read_word(port, PCI_EXP_LNKSTA, &lnksta))
			return 0;
		if (lnksta & PCI_EXP_LNKSTA_DLLLA) {
			pci_info(port, "slot link up after %u ms\n",
				 jiffies_to_msecs(jiffies - start));
			msleep(PCIE_RESET_CONFIG_WAIT_MS);
			return 0;
		}
	} while (time_before(jiffies, timeout));

	pci_info(port, "slot link still down after %d ms\n",
		 APPLE_PCIE_TUNNEL_READY_MS);
	return 0;
}

static const struct pci_ecam_ops apple_pcie_cfg_ecam_ops = {
	.init		= apple_pcie_init,
	.enable_device	= apple_pcie_enable_device,
	.disable_device	= apple_pcie_disable_device,
	.pci_ops	= {
		.add_bus	= apple_pcie_add_bus,
		.map_bus	= apple_pcie_map_bus,
		.read		= apple_pcie_config_read,
		.write		= apple_pcie_config_write,
	}
};

static int apple_pcie_probe_port(struct device_node *np)
{
	struct gpio_desc *gd;

	/* check whether the GPPIO pin exists but leave it as is */
	gd = fwnode_gpiod_get_index(of_fwnode_handle(np), "reset", 0,
				    GPIOD_ASIS, "PERST#");
	if (IS_ERR(gd))
		return PTR_ERR(gd);

	gpiod_put(gd);

	gd = fwnode_gpiod_get_index(of_fwnode_handle(np), "pwren", 0,
				    GPIOD_ASIS, "PWREN");
	if (IS_ERR(gd)) {
		if (PTR_ERR(gd) != -ENOENT)
			return PTR_ERR(gd);
	} else {
		gpiod_put(gd);
	}

	return 0;
}

static int apple_pcie_tunnel_init_resources(struct platform_device *pdev,
					     struct apple_pcie *pcie)
{
	struct resource *config, *debug, *fabric, *intr2axi, *oe_fabric;

	config = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	debug = platform_get_resource_byname(pdev, IORESOURCE_MEM, "debug");
	fabric = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fabric");
	intr2axi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intr2axi");
	oe_fabric = platform_get_resource_byname(pdev, IORESOURCE_MEM, "oe-fabric");
	if (!config || !debug || !fabric)
		return dev_err_probe(pcie->dev, -ENODEV,
				     "PCIe-C resume resources are incomplete\n");
	if (pcie->kernel_init && !oe_fabric)
		return dev_err_probe(pcie->dev, -ENODEV,
				     "PCIe-C oe-fabric region is required\n");

	debug->flags |= IORESOURCE_MEM_NONPOSTED;
	pcie->debug_base = devm_ioremap_resource(pcie->dev, debug);
	if (IS_ERR(pcie->debug_base))
		return PTR_ERR(pcie->debug_base);

	fabric->flags |= IORESOURCE_MEM_NONPOSTED;
	pcie->fabric_base = devm_ioremap_resource(pcie->dev, fabric);
	if (IS_ERR(pcie->fabric_base))
		return PTR_ERR(pcie->fabric_base);

	if (intr2axi) {
		intr2axi->flags |= IORESOURCE_MEM_NONPOSTED;
		pcie->intr2axi_base = devm_ioremap_resource(pcie->dev, intr2axi);
		if (IS_ERR(pcie->intr2axi_base))
			return PTR_ERR(pcie->intr2axi_base);
	}

	if (oe_fabric) {
		oe_fabric->flags |= IORESOURCE_MEM_NONPOSTED;
		pcie->oe_fabric_base = devm_ioremap_resource(pcie->dev, oe_fabric);
		if (IS_ERR(pcie->oe_fabric_base))
			return PTR_ERR(pcie->oe_fabric_base);
		pcie->oe_fabric_tunable = devm_apple_tunable_parse(pcie->dev,
								   pcie->dev->of_node,
								   "apple,tunable-oe-fabric",
								   oe_fabric);
		if (IS_ERR(pcie->oe_fabric_tunable))
			return dev_err_probe(pcie->dev,
					     PTR_ERR(pcie->oe_fabric_tunable),
					     "PCIe-C oe-fabric tunables unavailable\n");
	}

	pcie->rc_tunable = devm_apple_tunable_parse(pcie->dev,
						       pcie->dev->of_node,
						       "apple,tunable-rc", config);
	if (IS_ERR(pcie->rc_tunable))
		return dev_err_probe(pcie->dev, PTR_ERR(pcie->rc_tunable),
				     "PCIe-C root-complex tunables unavailable\n");

	pcie->debug_tunable = devm_apple_tunable_parse(pcie->dev,
							  pcie->dev->of_node,
							  "apple,tunable-debug", debug);
	if (IS_ERR(pcie->debug_tunable))
		return dev_err_probe(pcie->dev, PTR_ERR(pcie->debug_tunable),
				     "PCIe-C debug tunables unavailable\n");

	pcie->fabric_tunable = devm_apple_tunable_parse(pcie->dev,
							   pcie->dev->of_node,
							   "apple,tunable-fabric", fabric);
	if (IS_ERR(pcie->fabric_tunable))
		return dev_err_probe(pcie->dev, PTR_ERR(pcie->fabric_tunable),
				     "PCIe-C fabric tunables unavailable\n");

	return 0;
}

static bool apple_pcie_tunnel_power_is_retained(struct device_node *np)
{
	struct device_node *pd_np;
	bool retained;

	pd_np = of_parse_phandle(np, "power-domains", 0);
	if (!pd_np)
		return false;

	retained = of_property_read_bool(pd_np, "apple,always-on");
	of_node_put(pd_np);
	return retained;
}

struct apple_pcie_tunnel_link {
	struct device *consumer;
	struct device *supplier;
};

static void apple_pcie_tunnel_delete_link(void *data)
{
	struct apple_pcie_tunnel_link *link = data;

	/*
	 * A hot-unplugged endpoint purges its device links during unregister, so
	 * the struct device_link returned by device_link_add() may already be
	 * gone by the time PCIe-C releases its devres. Resolve the link by its
	 * refcounted endpoints instead; device_link_remove() is a no-op after a
	 * purge and avoids dereferencing a stale link object.
	 */
	device_link_remove(link->consumer, link->supplier);
	put_device(link->consumer);
	put_device(link->supplier);
}

static int apple_pcie_tunnel_add_link(struct apple_pcie *pcie,
				      struct device *consumer,
				      struct device *supplier)
{
	struct apple_pcie_tunnel_link *ref;
	struct device_link *link;
	int ret;

	link = device_link_add(consumer, supplier, DL_FLAG_STATELESS);
	if (!link)
		return -EINVAL;

	ref = devm_kzalloc(pcie->dev, sizeof(*ref), GFP_KERNEL);
	if (!ref) {
		device_link_del(link);
		return -ENOMEM;
	}

	ref->consumer = get_device(consumer);
	ref->supplier = get_device(supplier);
	ret = devm_add_action_or_reset(pcie->dev,
				       apple_pcie_tunnel_delete_link, ref);
	return ret;
}

/*
 * A function behind a cold-initialized T600x/T602x tunnel stays in D0
 * (NO_D3) when it runtime-suspends, but its wakeup never arrives: an xHCI
 * that suspends before a power-cycled display's hub reconnects never sees
 * the hub. Hold a runtime PM reference for as long as each function exists,
 * so that neither its driver nor power/control can suspend it.
 *
 * Keep the root port's link out of ASPM L1, too. Pulling the cable while
 * the link idles in L1 wedges the root port: every later cold init leaves
 * it unable to train, until the SoC is reset. The root port is added before
 * the bus behind it is scanned, so ASPM never enables L1 on its link.
 */
static int apple_pcie_tunnel_pci_notify(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct apple_pcie *pcie = container_of(nb, struct apple_pcie,
					       tunnel_pci_nb);
	struct pci_dev *pdev = to_pci_dev(data);

	if (pci_host_bridge_priv(pci_find_host_bridge(pdev->bus)) != pcie)
		return NOTIFY_DONE;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		/*
		 * T6050 adopted ports: no ASPM at all. On the J714c the N1 in its ROM
		 * personality (no firmware yet) takes the SoC down 20-40 s after the link
		 * first idles into L1; pcie_aspm=off runs clean.
		 */
		if (pcie->hw->adopt_link) {
			if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
				pcie_aspm_remove_cap(pdev, PCI_EXP_LNKCAP_ASPM_L0S |
						     PCI_EXP_LNKCAP_ASPM_L1);
			break;
		}
		pm_runtime_get_noresume(&pdev->dev);
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
			pcie_aspm_remove_cap(pdev, PCI_EXP_LNKCAP_ASPM_L1);
		break;
	case BUS_NOTIFY_DEL_DEVICE:
		if (pcie->hw->adopt_link)
			break;
		pm_runtime_put_noidle(&pdev->dev);
		break;
	}
	return NOTIFY_OK;
}

static void apple_pcie_tunnel_pci_unregister(void *data)
{
	struct apple_pcie *pcie = data;

	bus_unregister_notifier(&pci_bus_type, &pcie->tunnel_pci_nb);
}

static int apple_pcie_tunnel_keep_d0(struct pci_dev *pdev, void *data)
{
	/*
	 * The remote USB4 PCIe hierarchy becomes unreachable while its tunnel
	 * is asleep, so a device left in D3hot cannot receive the config write
	 * that would bring it back to D0.  Quiesce drivers normally, but leave
	 * PCI power-state ownership to the tunnel across system sleep.
	 */
	pdev->dev_flags |= PCI_DEV_FLAGS_NO_D3;
	return 0;
}

static int apple_pcie_tunnel_add_links(struct apple_pcie *pcie)
{
	struct platform_device *dart_pdev = NULL;
	struct platform_device *nhi_pdev = NULL;
	int ret;

	/*
	 * NHI and PCIe-C are sibling devices under ACIO, so encode their PM
	 * dependency explicitly. PCIe-C must complete its reset handshake before
	 * NHI tears down the router state, and NHI must resume before the host
	 * touches its tunneled aperture.
	 */
	for_each_available_child_of_node_scoped(pcie->dev->parent->of_node,
						 sibling) {
		if (!of_node_name_prefix(sibling, "nhi"))
			continue;

		nhi_pdev = of_find_device_by_node(sibling);
		break;
	}
	if (!nhi_pdev)
		return dev_err_probe(pcie->dev, -EPROBE_DEFER,
				     "Apple NHI device is not ready\n");

	ret = apple_pcie_tunnel_add_link(pcie, pcie->dev, &nhi_pdev->dev);
	put_device(&nhi_pdev->dev);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "failed to order PCIe-C after NHI resume\n");

	/*
	 * The DART and PCIe-C host are synthesized as siblings. Port hardware
	 * and RID/SID forwarding must be ready before DART access resumes, so
	 * suspend DART first and resume PCIe-C first.
	 */
	for_each_available_child_of_node_scoped(pcie->dev->of_node->parent,
						 sibling) {
		if (!of_property_present(sibling, "#iommu-cells"))
			continue;

		dart_pdev = of_find_device_by_node(sibling);
		break;
	}
	if (!dart_pdev)
		return dev_err_probe(pcie->dev, -EPROBE_DEFER,
				     "PCIe-C DART device is not ready\n");

	raw_spin_lock_init(&pcie->cio_cfg_lock);
	ret = -EOPNOTSUPP;
	if (cio_s2r && pcie->hw == &t8132_pciec_hw &&
	    of_machine_is_compatible("apple,j713")) {
		ret = apple_dart_cio_bind(pcie->dev, &dart_pdev->dev, &pcie->cio_dart);
		/* An older DT gives the DART no sleep path: keep the tunnel without one. */
		if (ret == -EOPNOTSUPP)
			dev_warn(pcie->dev, "DART has no CIO sleep path; a docked deep sleep is refused\n");
	} else if (cio_s2r && pcie->hw == &t6050_pciec_hw &&
		   (of_machine_is_compatible("apple,j714s") ||
		    of_machine_is_compatible("apple,j716c")) &&
		   of_property_present(dart_pdev->dev.of_node, "apple,clock-protection")) {
		/*
		 * TEST: T6050, a router that sleeps in place. Only with the DART's clock
		 * protection slice in the device tree; without it the tunnel is as before
		 * and its port is powered off for a sleep.
		 */
		ret = apple_dart_cio_bind(pcie->dev, &dart_pdev->dev, &pcie->cio_dart);
		if (!ret)
			dev_info(pcie->dev, "T6050 sleep: the tunnel is kept across a sleep by its router's NHI\n");
	}
	if (ret == -EOPNOTSUPP)
		ret = apple_pcie_tunnel_add_link(pcie, &dart_pdev->dev, pcie->dev);
	put_device(&dart_pdev->dev);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "failed to order PCIe-C before DART resume\n");

	dev_info(pcie->dev,
		 "PCIe-C ordered after NHI and before DART resume (%s power)\n",
		 pcie->power_retained ? "retained" : "cycled");
	return 0;
}

static int apple_pcie_neo_typed_ids(struct apple_pcie *pcie, struct pci_dev *root)
{
	static const u32 ids[] = { 0x793214c3, 0x793b14c3 };
	void __iomem *cfg;
	u32 id;
	u16 vendor, device;
	u8 low, high, header;
	int function;

	for (function = 0; function < ARRAY_SIZE(ids); function++) {
		/* Bypass the closed guard only for these fixed validation reads. */
		cfg = pci_ecam_map_bus(root->subordinate, PCI_DEVFN(0, function), 0);
		if (!cfg)
			return -ENODEV;
		id = readl(cfg + PCI_VENDOR_ID);
		vendor = readw(cfg + PCI_VENDOR_ID);
		device = readw(cfg + PCI_DEVICE_ID);
		low = readb(cfg + PCI_VENDOR_ID);
		high = readb(cfg + PCI_VENDOR_ID + 1);
		header = readb(cfg + PCI_HEADER_TYPE);
		dev_dbg(pcie->dev,
			"typed ECAM function=%d id=%#x vendor=%#x device=%#x bytes=%02x:%02x header=%#x\n",
			function, id, vendor, device, low, high, header);
		if (id != ids[function] || vendor != (ids[function] & 0xffff) ||
		    device != ids[function] >> 16 || low != 0xc3 || high != 0x14 ||
		    (header & PCI_HEADER_TYPE_MASK) != PCI_HEADER_TYPE_NORMAL ||
		    (!function && !(header & PCI_HEADER_TYPE_MFD)))
			return -EIO;
	}
	return 0;
}

static int apple_pcie_neo_check_memory(struct pci_dev *root)
{
	struct pci_dev *endpoint;
	struct pci_bus_region region;
	struct resource *res;
	u32 window = U32_MAX;
	u64 base, limit;
	u16 command = U16_MAX;
	int function, bar, ret = 0;

	ret = pci_read_config_word(root, PCI_COMMAND, &command);
	if (!ret)
		ret = pci_read_config_dword(root, PCI_MEMORY_BASE, &window);
	pci_dbg(root, "NEO_ROOT_MEMORY_READBACK status=%d command=%#x window=%#x\n",
		ret, command, window);
	if (ret || command == U16_MAX || !(command & PCI_COMMAND_MEMORY) || window == U32_MAX)
		return -EIO;
	/* Each packed register field is12 bits; PCI_MEMORY_RANGE_MASK is unsigned long. */
	base = (u64)(window & GENMASK(15, 4)) << 16;
	limit = (window & GENMASK(31, 20)) | GENMASK(19, 0);
	if (base > limit)
		return -EIO;
	for (function = 0; function < 2; function++) {
		endpoint = pci_get_slot(root->subordinate, PCI_DEVFN(0, function));
		if (!endpoint)
			return -ENODEV;
		for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
			res = &endpoint->resource[bar];
			if (!(res->flags & IORESOURCE_MEM))
				continue;
			pcibios_resource_to_bus(endpoint->bus, &region, res);
			if (!res->parent || !resource_size(res) ||
			    res->flags & (IORESOURCE_UNSET | IORESOURCE_DISABLED | IORESOURCE_PREFETCH) ||
			    region.start < base || region.end > limit) {
				pci_err(endpoint, "BAR%d outside programmed root memory window\n", bar);
				ret = -ERANGE;
				break;
			}
		}
		pci_dev_put(endpoint);
		if (ret)
			return ret;
	}
	pci_dbg(root, "NEO_ROOT_MEMORY_ENABLED command=%#x window=%#x bus=%#llx-%#llx\n",
		command, window, base, limit);
	return 0;
}

/*
 * Both functions must come out of the bootstrap cold. Check that before they
 * are published, because publication lets their drivers bind and take over.
 */
static int apple_pcie_neo_check_cold(struct apple_pcie *pcie, struct pci_dev *root)
{
	struct pci_dev *endpoint;
	u16 command;
	int function, ret = 0;

	/* Check and quiesce both functions; keep the first error. */
	for (function = 0; function < 2; function++) {
		endpoint = pci_get_slot(root->subordinate, PCI_DEVFN(0, function));
		if (!endpoint) {
			ret = ret ?: -ENODEV;
			continue;
		}
		if (pci_read_config_word(endpoint, PCI_COMMAND, &command)) {
			ret = ret ?: -EIO;
		} else {
			dev_dbg(pcie->dev, "enumerated %s id=%04x:%04x command=%#x\n",
				pci_name(endpoint), endpoint->vendor, endpoint->device, command);
			if (command & PCI_COMMAND_MASTER) {
				pci_clear_master(endpoint);
				ret = ret ?: -EIO;
			}
		}
		pci_dev_put(endpoint);
	}
	return ret;
}

/*
 * Rescan lock held, downstream config still open. Remove functions that were
 * scanned but will not be published, so that a later rescan cannot publish
 * them after config access has been closed again.
 */
static void apple_pcie_neo_unscan(struct pci_dev *root)
{
	struct pci_dev *dev, *tmp;

	list_for_each_entry_safe_reverse(dev, tmp, &root->subordinate->devices, bus_list)
		pci_stop_and_remove_bus_device(dev);
}

static int apple_pcie_neo_enumerate(struct pci_host_bridge *bridge)
{
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);
	struct pci_dev *root;
	int ret;

	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return -ENODEV;
	if (!root->subordinate || root->subordinate->number != 1) {
		ret = -EINVAL;
		goto out;
	}
	/* No config/raw spinlock is held across this process-context request. */
	ret = apple_piodma_bootstrap_prime(pcie->piodma_supplier, root);
	if (ret)
		goto out;
	ret = apple_pcie_neo_typed_ids(pcie, root);
	if (ret)
		goto out;
	pcie->neo_secondary_bus = root->subordinate->number;
	pci_lock_rescan_remove();
	/* Publish validated bootstrap state and bus number before config access. */
	smp_store_release(&pcie->neo_config_ready, true);
	/* Root was already enabled by the closed scan. The bridge-specific API
	 * programs its new windows even when the generic bus rescan would skip
	 * pci_setup_bridge(). Publish children only after hardware validation.
	 */
	pci_scan_child_bus(root->subordinate);
	pci_assign_unassigned_bridge_resources(root);
	ret = apple_pcie_neo_check_memory(root);
	if (ret) {
		dev_err(pcie->dev, "root memory forwarding unavailable after assignment: %d\n", ret);
		goto close_config;
	}
	ret = apple_pcie_neo_check_cold(pcie, root);
	if (ret)
		goto close_config;
	apple_piodma_bootstrap_admitted(pcie->piodma_supplier);
	pci_bus_add_devices(root->subordinate);
close_config:
	if (ret) {
		apple_pcie_neo_unscan(root);
		WRITE_ONCE(pcie->neo_config_ready, false);
	}
	pci_unlock_rescan_remove();
	if (!ret)
		dev_info(pcie->dev, "radio PCIe bootstrap complete; Wi-Fi and Bluetooth admitted\n");
out:
	pci_dev_put(root);
	return ret;
}

/*
 * What the bootloader did to this host at boot, on the same registers, in its
 * order, with its tunables. On this SoC a deep sleep powers all of it off.
 */
struct apple_pcie_neo_cold {
	void __iomem *axi2af;
	void __iomem *cio3pll;
	void __iomem *clkgen;
	void __iomem *phy_block;
	struct apple_tunable *axi2af_tunable;
	struct apple_tunable *cio3pll_tunable;
	struct apple_tunable *clkgen_tunable;
	struct apple_tunable *phy_tunable;
	struct apple_tunable *port_tunable;
	struct apple_tunable *rc_tunable;
	struct apple_tunable *rc_gen3_tunable;
	struct apple_tunable *rc_gen4_tunable;
	/* The port's DART, clocked by the port: restored after the port is set up. */
	struct device *dart;
};

#define NEO_PHY_CTRL			0x0
#define   NEO_PHY_CTRL_CLK0REQ		BIT(0)
#define   NEO_PHY_CTRL_CLK1REQ		BIT(1)
#define   NEO_PHY_CTRL_CLK0ACK		BIT(2)
#define   NEO_PHY_CTRL_CLK1ACK		BIT(3)
#define   NEO_PHY_CTRL_RESET		BIT(4)
#define NEO_PHY_RUN			0x4
#define NEO_PHY_READY			0x8
#define NEO_PHY_CTRL_UP			BIT(27)
#define NEO_PHY_SETTLE_US		300
#define NEO_AXI_EN0			0x104
#define NEO_AXI_EN1			0x108
#define NEO_CLK_CTRL			0x0
#define NEO_CLK_BUSY			BIT(31)
#define NEO_CLK_OFF			BIT(2)
#define NEO_CLK_START			0x4
#define NEO_CLK_START_GO		BIT(31)
#define NEO_PHY_BLOCK_COMMON		0x4000
#define NEO_PHY_BLOCK_PHY		0x8000
#define NEO_PHY_POLL_US			50000
#define NEO_RC_WRITE_ZERO		0x04
#define NEO_RC_READY_CFG		0x54
#define NEO_RC_READY_START		0x50
#define NEO_RC_READY_STAT		0x58
#define NEO_RC_POLL_US			250000
#define NEO_DBI_CAP_LNKCAP		0x7c
#define NEO_DBI_CAP_LNKCAP2		0x9c
#define   NEO_DBI_LNKCAP2_SLS		GENMASK(6, 1)
#define NEO_DBI_CAP_LNKCTL2		0xa0
#define NEO_DBI_PORT_LINK_CONTROL	0x710
#define   NEO_DBI_PORT_LINK_MODE	GENMASK(21, 16)
#define NEO_DBI_WIDTH_SPEED_CONTROL	0x80c
#define   NEO_DBI_LINK_WIDTH		GENMASK(12, 8)
#define   NEO_DBI_SPEED_CHANGE		BIT(17)
#define NEO_DBI_RO_WR			0x8bc
#define   NEO_DBI_RO_WR_EN		BIT(0)

static const struct apple_pcie_reset_reg apple_pcie_neo_port_regs[] = {
	{ 0x088, 0x00000110 },
	{ 0x100, 0xffffffff },
	{ 0x148, 0xffffffff },
	{ 0x210, 0xffffffff },
	{ 0x080, 0x00000000 },
	{ 0x084, 0x00000000 },
	{ 0x104, 0xfffffff0 },
	{ 0x124, 0x00000100 },
	{ 0x16c, 0x00000000 },
	{ 0x13c, 0x00000000 },
	{ 0x800, 0x00100100 },
	{ 0x808, 0x001000ff },
	{ 0x82c, 0x00000000 },
};

static const struct apple_pcie_reset_reg apple_pcie_neo_port_regs2[] = {
	{ 0x130, 0x03020000 },
	{ 0x140, 0x00000010 },
	{ 0x144, 0x00253770 },
	{ 0x21c, 0x00000000 },
	{ 0x834, 0x00000000 },
	{ 0x83c, 0x00000000 },
};

static int apple_pcie_neo_phy_clocks(void __iomem *phy)
{
	u32 stat;
	int ret;

	rmw_set(NEO_PHY_CTRL_CLK0REQ, phy + NEO_PHY_CTRL);
	ret = readl_relaxed_poll_timeout(phy + NEO_PHY_CTRL, stat, stat & NEO_PHY_CTRL_CLK0ACK,
					 10, NEO_PHY_POLL_US);
	if (ret)
		return ret;
	rmw_set(NEO_PHY_CTRL_CLK1REQ, phy + NEO_PHY_CTRL);
	return readl_relaxed_poll_timeout(phy + NEO_PHY_CTRL, stat, stat & NEO_PHY_CTRL_CLK1ACK,
					  10, NEO_PHY_POLL_US);
}

/*
 * The part of the block that ANS shares, from power-off in macOS's order, measured on
 * one sleep. The sleep driver does it at the wake when ANS is bound, and it can't be
 * redone under a running PHY.
 */
static int apple_pcie_neo_common_init(struct apple_pcie *pcie)
{
	struct apple_pcie_neo_cold *cold = pcie->neo_cold;
	void __iomem *phy = cold->phy_block + NEO_PHY_BLOCK_PHY;
	u32 stat, val;
	int ret;

	if (readl_relaxed(pcie->base + NEO_RC_READY_STAT) & BIT(0))
		return 0;

	rmw_set(BIT(0), cold->axi2af + NEO_AXI_EN0);
	rmw_set(BIT(0), cold->axi2af + NEO_AXI_EN1);
	apple_tunable_apply(cold->axi2af, cold->axi2af_tunable);
	apple_tunable_apply(cold->cio3pll, cold->cio3pll_tunable);
	apple_tunable_apply(cold->clkgen, cold->clkgen_tunable);
	rmw_set(NEO_CLK_START_GO, cold->clkgen + NEO_CLK_START);
	rmw_clear(NEO_CLK_OFF, cold->clkgen + NEO_CLK_CTRL);
	ret = readl_relaxed_poll_timeout(cold->clkgen + NEO_CLK_CTRL, stat, !(stat & NEO_CLK_BUSY),
					 1, NEO_PHY_POLL_US);
	if (ret)
		return ret;
	apple_tunable_apply(cold->phy_block, cold->phy_tunable);

	ret = apple_pcie_neo_phy_clocks(phy);
	if (ret)
		return ret;
	rmw_clear(NEO_PHY_CTRL_RESET, phy + NEO_PHY_CTRL);
	udelay(1);
	ret = readl_relaxed_poll_timeout(phy + NEO_PHY_CTRL, stat, !(stat & NEO_PHY_CTRL_RESET),
					 10, NEO_PHY_POLL_US);
	if (ret)
		return ret;
	rmw_set(BIT(0), phy + NEO_PHY_RUN);
	udelay(NEO_PHY_SETTLE_US);
	rmw_set(BIT(4), phy + NEO_PHY_RUN);
	ret = readl_relaxed_poll_timeout(phy + NEO_PHY_READY, stat, stat & BIT(0),
					 10, NEO_PHY_POLL_US);
	if (ret)
		return ret;

	val = readl_relaxed(cold->phy_block + NEO_PHY_BLOCK_COMMON);
	writel_relaxed((val & ~GENMASK(1, 0)) | 1, cold->phy_block + NEO_PHY_BLOCK_COMMON);
	rmw_set(NEO_PHY_CTRL_UP, phy + NEO_PHY_CTRL);

	writel_relaxed(0, pcie->base + NEO_RC_WRITE_ZERO);
	writel_relaxed(0x140, pcie->base + NEO_RC_READY_CFG);
	writel_relaxed(1, pcie->base + NEO_RC_READY_START);

	return readl_relaxed_poll_timeout(pcie->base + NEO_RC_READY_STAT, stat, stat & BIT(0),
					  100, NEO_RC_POLL_US);
}

static int apple_pcie_neo_cold_init(struct apple_pcie *pcie, struct apple_pcie_port *port)
{
	struct apple_pcie_neo_cold *cold = pcie->neo_cold;
	void __iomem *dbi = pcie->cfg->win + ((resource_size_t)port->idx << 15);
	int speed = of_pci_get_max_link_speed(port->np);
	u32 stat, val;
	int i, ret;

	ret = apple_pcie_neo_common_init(pcie);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(apple_pcie_neo_port_regs); i++)
		apple_pcie_port_writel(port, apple_pcie_neo_port_regs[i].value,
				       apple_pcie_neo_port_regs[i].offset);
	for (i = 0; i < 16; i++)
		apple_pcie_port_writel(port, 0, 0x3000 + 4 * i);
	for (i = 0; i < 512; i++)
		apple_pcie_port_writel(port, 0, pcie->hw->port_msimap + 4 * i);
	for (i = 0; i < ARRAY_SIZE(apple_pcie_neo_port_regs2); i++)
		apple_pcie_port_writel(port, apple_pcie_neo_port_regs2[i].value,
				       apple_pcie_neo_port_regs2[i].offset);
	apple_tunable_apply(port->base, cold->port_tunable);
	rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);

	/* PERST# off before the PHY's clocks, as macOS has it coming from power-off. */
	rmw_set(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat, stat & PORT_STATUS_READY,
					 100, NEO_RC_POLL_US);
	if (ret)
		return ret;
	rmw_clear(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);
	ret = readl_relaxed_poll_timeout(port->base + PORT_LINKSTS, stat,
					 !(stat & PORT_LINKSTS_BUSY), 100, NEO_RC_POLL_US);
	if (ret)
		return ret;

	rmw_clear(NEO_PHY_CTRL_CLK0REQ | NEO_PHY_CTRL_CLK1REQ, port->phy + NEO_PHY_CTRL);
	ret = apple_pcie_neo_phy_clocks(port->phy);
	if (ret)
		return ret;
	rmw_clear(NEO_PHY_CTRL_RESET, port->phy + NEO_PHY_CTRL);
	rmw_set(BIT(9), port->phy + NEO_PHY_CTRL);
	rmw_set(BIT(10), port->phy + NEO_PHY_CTRL);

	rmw_set(NEO_DBI_RO_WR_EN, dbi + NEO_DBI_RO_WR);
	apple_tunable_apply(dbi, cold->rc_tunable);
	apple_tunable_apply(dbi, cold->rc_gen3_tunable);
	apple_tunable_apply(dbi, cold->rc_gen4_tunable);
	if (speed > 0) {
		val = readl_relaxed(dbi + NEO_DBI_CAP_LNKCAP);
		writel_relaxed((val & ~PCI_EXP_LNKCAP_SLS) | speed, dbi + NEO_DBI_CAP_LNKCAP);
		val = readl_relaxed(dbi + NEO_DBI_CAP_LNKCAP2);
		writel_relaxed((val & ~NEO_DBI_LNKCAP2_SLS) |
			       FIELD_PREP(NEO_DBI_LNKCAP2_SLS, (1 << speed) - 1),
			       dbi + NEO_DBI_CAP_LNKCAP2);
		writew_relaxed((readw_relaxed(dbi + NEO_DBI_CAP_LNKCTL2) & ~PCI_EXP_LNKCTL2_TLS) |
			       speed, dbi + NEO_DBI_CAP_LNKCTL2);
		rmw_set(NEO_DBI_SPEED_CHANGE, dbi + NEO_DBI_WIDTH_SPEED_CONTROL);
	}
	val = readl_relaxed(dbi + NEO_DBI_PORT_LINK_CONTROL) & ~NEO_DBI_PORT_LINK_MODE;
	writel_relaxed(val | FIELD_PREP(NEO_DBI_PORT_LINK_MODE, 1),
		       dbi + NEO_DBI_PORT_LINK_CONTROL);
	val = readl_relaxed(dbi + NEO_DBI_WIDTH_SPEED_CONTROL) & ~NEO_DBI_LINK_WIDTH;
	writel_relaxed(val | FIELD_PREP(NEO_DBI_LINK_WIDTH, 1), dbi + NEO_DBI_WIDTH_SPEED_CONTROL);
	val = readl_relaxed(dbi + NEO_DBI_CAP_LNKCAP) & ~PCI_EXP_LNKCAP_MLW;
	writel_relaxed(val | FIELD_PREP(PCI_EXP_LNKCAP_MLW, 1), dbi + NEO_DBI_CAP_LNKCAP);
	rmw_clear(NEO_DBI_RO_WR_EN, dbi + NEO_DBI_RO_WR);

	return 0;
}

static int apple_pcie_attach_domains(struct apple_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct dev_pm_domain_attach_data data = {
		.pd_flags = PD_FLAG_DEV_LINK_ON,
	};
	int count, ret, i;

	if (!IS_ENABLED(CONFIG_PM))
		return 0;
	if (!of_property_present(dev->of_node, "power-domains"))
		return 0;

	count = of_count_phandle_with_args(dev->of_node, "power-domains",
					 "#power-domain-cells");
	if (count < 0)
		return count;
	if (count > 1) {
		ret = devm_pm_domain_attach_list(dev, &data, &pcie->pd_list);
		if (ret < 0)
			return ret;
		if (ret != count)
			return -ENODEV;
	} else if (!dev->pm_domain) {
		return 0;
	}

	/* A single domain is already attached by the platform bus. */
	pcie->pds = devm_kcalloc(dev, count, sizeof(*pcie->pds), GFP_KERNEL);
	if (!pcie->pds)
		return -ENOMEM;
	pcie->num_pds = count;
	for (i = 0; i < count; i++)
		pcie->pds[i].dev = pcie->pd_list ? pcie->pd_list->pd_devs[i] : dev;

	return 0;
}

static void apple_pcie_neo_recover_work(struct work_struct *work);
static void apple_pcie_neo_radio_work(struct work_struct *work);

/* A radio left unusable by the wake, or its link. POST_SUSPEND runs this once all is thawed. */
static void apple_pcie_neo_recover(void *data)
{
	struct apple_pcie *pcie = data;

	WRITE_ONCE(pcie->neo_recover, true);
	/* Pairs with the barrier at PM_POST_SUSPEND: one side sees the other's write. */
	smp_mb();
	if (!READ_ONCE(pcie->neo_sleep_armed))
		queue_work(system_dfl_wq, &pcie->neo_recover_work);
}

/* Runs before the freezer. The radios stay bound, and their drivers do their own suspend. */
static int apple_pcie_neo_pm_notify(struct notifier_block *nb, unsigned long action, void *unused)
{
	struct apple_pcie *pcie = container_of(nb, struct apple_pcie, neo_pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		if (pm_suspend_default_s2idle())
			break;
		flush_work(&pcie->neo_radio_work);
		flush_work(&pcie->neo_recover_work);
		pcie->neo_sleep_armed = true;
		apple_piodma_bootstrap_set_sleep(pcie->piodma_supplier, true);
		break;
	case PM_POST_SUSPEND:
		if (!pcie->neo_sleep_armed)
			break;
		WRITE_ONCE(pcie->neo_sleep_armed, false);
		/* Pairs with the barrier in the recovery request. */
		smp_mb();
		apple_piodma_bootstrap_set_sleep(pcie->piodma_supplier, false);
		if (READ_ONCE(pcie->neo_recover))
			queue_work(system_dfl_wq, &pcie->neo_recover_work);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static void __iomem *apple_pcie_neo_map(struct platform_device *pdev, const char *name,
					struct resource **res)
{
	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!*res)
		return IOMEM_ERR_PTR(-ENOENT);

	/* These overlap each other and the port PHYs, so they can't be claimed. */
	return devm_ioremap(&pdev->dev, (*res)->start, resource_size(*res)) ?:
	       IOMEM_ERR_PTR(-ENOMEM);
}

/* A shared block and the tunable the loader publishes for it. */
static int apple_pcie_neo_get(struct platform_device *pdev, const char *name, const char *prop,
			      void __iomem **base, struct apple_tunable **tunable)
{
	struct resource *res;

	*base = apple_pcie_neo_map(pdev, name, &res);
	if (IS_ERR(*base))
		return PTR_ERR(*base);
	*tunable = devm_apple_tunable_parse(&pdev->dev, pdev->dev.of_node, prop, res);

	return PTR_ERR_OR_ZERO(*tunable);
}

/* The DART in the host's iommu-map, which reads nothing until the port is set up. */
static struct device *apple_pcie_neo_dart(struct device *dev, bool retried)
{
	struct device_node *np = of_parse_phandle(dev->of_node, "iommu-map", 1);
	struct platform_device *pdev = np ? of_find_device_by_node(np) : NULL;
	int ret;

	of_node_put(np);
	if (!pdev)
		return NULL;
	/* The reference is kept: this host can't be unbound. */
	ret = apple_dart_set_host_resumes(&pdev->dev);
	/* The DART resumes, and gates itself, before this host restores it. */
	if (!ret && !device_link_add(dev, &pdev->dev, DL_FLAG_STATELESS))
		ret = -EINVAL;
	if (ret) {
		put_device(&pdev->dev);
		/* Not bound yet, and the caller looks again when a sleep starts */
		if (ret == -EPROBE_DEFER && retried)
			dev_dbg(dev, "the port's DART isn't bound yet\n");
		else
			dev_warn(dev, "no deep sleep: its DART can't wait for the port: %d\n", ret);
		return NULL;
	}
	return &pdev->dev;
}

/* Without all of this the host can't come back, and a deep sleep stays refused. */
static void apple_pcie_neo_sleep_probe(struct platform_device *pdev, struct apple_pcie *pcie)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct apple_pcie_neo_cold *cold;
	struct apple_pcie_port *port;
	struct resource *res, *cfg;
	int ret;

	port = list_first_entry_or_null(&pcie->ports, struct apple_pcie_port, entry);
	cfg = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (!port || !cfg || !port->perst_gpio || !port->phy) {
		dev_warn(dev, "no deep sleep: nothing here took the link up, so no pins to restart it\n");
		return;
	}
	cold = devm_kzalloc(dev, sizeof(*cold), GFP_KERNEL);
	if (!cold)
		return;

	if (apple_pcie_neo_get(pdev, "axi2af", "apple,tunable-axi2af", &cold->axi2af,
			       &cold->axi2af_tunable) ||
	    apple_pcie_neo_get(pdev, "cio3pll", "apple,tunable-cio3pll", &cold->cio3pll,
			       &cold->cio3pll_tunable) ||
	    apple_pcie_neo_get(pdev, "clkgen", "apple,tunable-clkgen", &cold->clkgen,
			       &cold->clkgen_tunable))
		goto absent;
	cold->phy_block = apple_pcie_neo_map(pdev, "phy-block", &res);
	if (IS_ERR(cold->phy_block))
		goto absent;
	cold->phy_tunable = devm_apple_tunable_parse(dev, np, "apple,tunable-phy", res);
	if (IS_ERR(cold->phy_tunable))
		goto absent;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "port0");
	if (!res)
		goto absent;
	cold->port_tunable = devm_apple_tunable_parse(dev, port->np, "apple,tunable-port", res);
	if (IS_ERR(cold->port_tunable))
		goto absent;
	cold->rc_tunable = devm_apple_tunable_parse(dev, port->np, "apple,tunable-rc", cfg);
	if (IS_ERR(cold->rc_tunable))
		goto absent;
	cold->rc_gen3_tunable = devm_apple_tunable_parse(dev, port->np, "apple,tunable-rc-gen3",
							 cfg);
	if (IS_ERR(cold->rc_gen3_tunable))
		goto absent;
	cold->rc_gen4_tunable = devm_apple_tunable_parse(dev, port->np, "apple,tunable-rc-gen4",
							 cfg);
	if (IS_ERR(cold->rc_gen4_tunable))
		goto absent;
	cold->dart = apple_pcie_neo_dart(dev, false);
	if (!cold->dart)
		goto absent;

	pcie->neo_cold = cold;
	INIT_WORK(&pcie->neo_recover_work, apple_pcie_neo_recover_work);
	INIT_WORK(&pcie->neo_radio_work, apple_pcie_neo_radio_work);
	init_completion(&pcie->neo_noirq_done);
	apple_piodma_bootstrap_set_recover(pcie->piodma_supplier, apple_pcie_neo_recover, pcie);
	pcie->neo_pm_nb.notifier_call = apple_pcie_neo_pm_notify;
	ret = register_pm_notifier(&pcie->neo_pm_nb);
	if (ret) {
		pcie->neo_cold = NULL;
		return;
	}
	/* The radio work outlives resume_noirq; nothing that doesn't depend on it waits. */
	device_enable_async_suspend(dev);
	dev_dbg(dev, "deep sleep: radios stay powered, link stopped and retrained\n");
	return;

absent:
	dev_warn(dev, "no deep sleep: the loader didn't publish what this host needs\n");
}

#include "pcie-apple-radio.c"

/*
 * A firmware sleep of a T6050 host whose port a boot loader trained (the J714s's N1). The SoC's
 * sleep engine needs the port's link shut down before the commit, and the sleep takes the
 * port's, its DART's and the shared PHY's state while the endpoint keeps its power, held in
 * PERST#. So the noirq suspend shuts the link down (the sleep driver's sequence), with the
 * DART saved and gated first, and the noirq resume sets the port up again: the port's reset
 * table and enable, the root complex setup, the routes, the DART, the MSI setup, then the lane
 * clocks, PERST# and the LTSSM. The PCI core
 * restores the root port and the functions after that. The DART is clocked by the port: it
 * waits for the host (apple_dart_set_host_resumes()), and is not read while the port is down.
 */
/*
 * The other ports of that host, those Linux powers and trains (the J714s SD card reader): the
 * firmware sleep takes their power and their state. Before the freeze their functions are
 * removed and the port parked as a shutdown leaves it. macOS does the same to that port before
 * the radio's, and switches the reader's power off after it.
 *
 * The port's DART is clocked by the port, as the radio's is: once the port is parked it does
 * not answer, and after the sleep a write to it is an external abort. So it is saved and gated
 * here, with the port still up, and marked as one its host resumes: its own resume then leaves
 * it alone. The root port's config space goes with the port's registers as well; what the PCI
 * core has to put back is kept from before the park.
 *
 * The root port itself is suspended at run time first, with the port still up: idle, it would
 * be put into D3hot a moment later, through a port that no longer answers.
 *
 * After the wake the port comes back (apple_pcie_t6050_cold_wake()) once the resume is over or,
 * with t6050_sd_port_wake=1, in the early resume; with t6050_sd_port_wake=0 it stays parked,
 * with no functions, until a restart.
 */
static struct device *apple_pcie_t6050_cold_dart(struct apple_pcie_port *port)
{
	struct of_phandle_args iommu = {};
	struct platform_device *pdev;
	int ret;

	if (of_map_iommu_id(port->pcie->dev->of_node, port->adopted_bus << 8, &iommu) ||
	    !iommu.np)
		return NULL;
	pdev = of_find_device_by_node(iommu.np);
	of_node_put(iommu.np);
	if (!pdev)
		return NULL;
	/* The reference is kept: this host can't be unbound. Not bound: nothing resumes it. */
	ret = apple_dart_set_host_resumes(&pdev->dev);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_warn(port->pcie->dev,
				 "port %d: its DART can't wait for the port (%d): its own resume writes it at the wake\n",
				 port->idx, ret);
		put_device(&pdev->dev);
		return NULL;
	}
	return &pdev->dev;
}

static void apple_pcie_t6050_park_cold(struct apple_pcie *pcie)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	struct pci_dev *dev, *tmp;
	int slot;

	pci_lock_rescan_remove();
	list_for_each_entry(port, &pcie->ports, entry) {
		bool link_up;

		if (port == pcie->t6050_port || !port->pwren || (pcie->t6050_parked & BIT(port->idx)))
			continue;
		if (!port->cold_dart)
			port->cold_dart = apple_pcie_t6050_cold_dart(port);
		list_for_each_entry_safe_reverse(dev, tmp, &bridge->bus->devices, bus_list) {
			if (PCI_SLOT(dev->devfn) != port->idx || !dev->subordinate)
				continue;
			pm_runtime_get_sync(&dev->dev);
			while (!list_empty(&dev->subordinate->devices))
				pci_stop_and_remove_bus_device(list_last_entry(&dev->subordinate->devices,
									       struct pci_dev, bus_list));
			/* The root port as it is now, for the PCI core after the port is back */
			kfree(port->cold_rp_state);
			port->cold_rp_state = NULL;
			if (!pci_save_state(dev))
				port->cold_rp_state = pci_store_saved_state(dev);
			pm_runtime_put_sync_suspend(&dev->dev);
		}
		mutex_lock(&pcie->lock);
		link_up = apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP;
		apple_pcie_port_disable_irq(port);
		port->saved_intmask = apple_pcie_port_readl(port, PORT_INTMSK);
		for_each_set_bit(slot, &port->adopted_slots, BITS_PER_LONG)
			port->saved_rid2sid[slot] = apple_pcie_port_readl(port,
									  port_rid2sid_offset(port, slot));
		if (port->cold_dart) {
			apple_dart_state_save(port->cold_dart);
			apple_dart_quiesce_commands(port->cold_dart);
		}
		apple_pcie_t6050_port_park(port, link_up);
		mutex_unlock(&pcie->lock);
		pcie->t6050_parked |= BIT(port->idx);
		dev_info(pcie->dev, "port %d: parked for the sleep, its functions removed%s\n", port->idx,
			 port->cold_dart ? ", its DART saved and gated" : "");
	}
	pci_unlock_rescan_remove();
}

/*
 * A parked port back up, in macOS's order at its wake (after the radio's port): PERST# held,
 * the endpoint's power on and the boot-time bring-up's wait for it, the port's reset table and
 * enable, the root complex setup, the MSI setup and the routes, the DART, then the lane clocks,
 * PERST# released and the LTSSM, until the link is up; the 100 ms after it are the caller's.
 * With the host's lock held, but through the endpoint's power-up wait: the MSI vectors of the
 * radio's functions, which its driver enumerates again after the wake, take that lock.
 */
static int apple_pcie_t6050_cold_up(struct apple_pcie_port *port, u32 *status)
{
	struct apple_pcie *pcie = port->pcie;
	int ret;

	*status = 0;

	if (!(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN) ||
	    !(readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY))
		return -ENOLINK;
	ret = apple_pcie_t6050_perst(port, true);
	if (ret)
		return ret;
	/* From here the endpoint may be on, and a failure has to park the port */
	port->powered = true;
	ret = gpiod_set_value_cansleep(port->pwren, 1);
	if (ret)
		return ret;
	/*
	 * Nothing of the port is set up yet. The radio's driver may have taken the radio's port
	 * down meanwhile, the shared PHY and the root complex with it (after a suspend it
	 * abandoned): the setup is then not started, and the caller parks the port again.
	 */
	mutex_unlock(&pcie->lock);
	msleep(150);
	mutex_lock(&pcie->lock);
	if (READ_ONCE(pcie->t6050_down) || pcie->t6050_lost ||
	    !(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN) ||
	    !(readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY))
		return -ENOLINK;
	ret = apple_pcie_t6050_port_enable(port);
	if (ret)
		return ret;
	ret = apple_pcie_t6050_port_configure(port) ?:
	      apple_pcie_t6050_port_routes(port, port->saved_rid2sid);
	/* The port is clocked: its DART answers again */
	if (!ret && port->cold_dart)
		ret = apple_dart_host_restart(port->cold_dart);
	if (!ret)
		ret = apple_pcie_t6050_port_link(port);
	*status = apple_pcie_port_readl(port, PORT_LINKSTS);
	if (ret)
		return ret;
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, port->saved_intmask, PORT_INTMSK);
	WRITE_ONCE(port->link_failed, false);
	return 0;
}

/*
 * One parked port back up: the link, its DART, and, in the early resume, the root port's config
 * space. The PCI core's noirq resume of that root port came before this, with the port still
 * down: it found the root port out of reach and restored nothing. Its later phases, and the
 * port services' resume, find it as it was before the park.
 */
static int apple_pcie_t6050_cold_wake(struct apple_pcie *pcie, struct apple_pcie_port *port,
				      bool early)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	ktime_t start = ktime_get();
	struct pci_dev *rp;
	u32 status;
	int ret;

	mutex_lock(&pcie->lock);
	ret = apple_pcie_t6050_cold_up(port, &status);
	if (ret && port->powered) {
		/* Clocked but not up: held as a shutdown leaves it, its DART out of reach again */
		if (port->cold_dart)
			apple_dart_quiesce_commands(port->cold_dart);
		apple_pcie_t6050_port_park(port, false);
	}
	if (!ret) {
		/* Under the lock, as a restart's shutdown reads them: a port up is no longer parked */
		WRITE_ONCE(pcie->t6050_cold_lost, pcie->t6050_cold_lost & ~BIT(port->idx));
		pcie->t6050_parked &= ~BIT(port->idx);
		pcie->t6050_rescan |= BIT(port->idx);
	}
	mutex_unlock(&pcie->lock);
	if (ret) {
		dev_err(pcie->dev, "port %d: not back after the sleep: %d, status %#x; %s\n",
			port->idx, ret, status,
			early ? "tried once more when the resume is over" :
			"it stays parked until a restart");
		return ret;
	}
	/*
	 * Wait for 100ms after the link trains (PCIe r5.0, 6.6.1), without the host's lock: the
	 * endpoint gets no configuration request before the rescan, and the root port's config
	 * space below is the port's own.
	 */
	msleep(100);
	rp = early ? pci_get_domain_bus_and_slot(pci_domain_nr(bridge->bus), 0,
						  PCI_DEVFN(port->idx, 0)) : NULL;
	if (rp) {
		/* Runtime PM is off in this phase: the power state and the config space directly */
		if (port->cold_rp_state && !pci_set_power_state(rp, PCI_D0) &&
		    !pci_load_saved_state(rp, port->cold_rp_state)) {
			pci_restore_state(rp);
			pcie->t6050_rp_restored |= BIT(port->idx);
		}
		pci_dev_put(rp);
	}
	apple_pcie_port_enable_irq(port);
	dev_info(pcie->dev, "port %d: link up again after the sleep in %lld ms, status %#x%s\n",
		 port->idx, ktime_ms_delta(ktime_get(), start), status,
		 !early ? "" : pcie->t6050_rp_restored & BIT(port->idx) ?
		 ", in the early resume, its root port restored" :
		 ", in the early resume, its root port not restored yet");
	return 0;
}

/*
 * With every device resumed and the tasks thawed: the PCI core gets the root port's config
 * space again, if the early resume has not given it back, and finds the port's functions.
 */
static void apple_pcie_t6050_cold_rescan(struct apple_pcie *pcie, struct apple_pcie_port *port)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	bool restored = pcie->t6050_rp_restored & BIT(port->idx);
	bool found = false;
	struct pci_dev *rp;

	rp = pci_get_domain_bus_and_slot(pci_domain_nr(bridge->bus), 0, PCI_DEVFN(port->idx, 0));
	if (rp) {
		/* Should the root port be runtime-suspended, its resume restores this too */
		if (!restored && port->cold_rp_state)
			pci_load_saved_state(rp, port->cold_rp_state);
		pm_runtime_get_sync(&rp->dev);
		if (!restored) {
			pci_set_power_state(rp, PCI_D0);
			if (port->cold_rp_state) {
				pci_load_and_free_saved_state(rp, &port->cold_rp_state);
				pci_restore_state(rp);
				restored = true;
			}
		}
		if (restored) {
			pci_lock_rescan_remove();
			if (rp->subordinate) {
				pci_rescan_bus(rp->subordinate);
				found = !list_empty(&rp->subordinate->devices);
			}
			pci_unlock_rescan_remove();
		}
		pm_runtime_put(&rp->dev);
		pci_dev_put(rp);
	}
	kfree(port->cold_rp_state);
	port->cold_rp_state = NULL;
	pcie->t6050_rescan &= ~BIT(port->idx);
	pcie->t6050_rp_restored &= ~BIT(port->idx);
	dev_info(pcie->dev, "port %d: its functions %s after the sleep\n", port->idx,
		 found ? "found again" : "not found");
}

/*
 * With t6050_sd_port_wake=1, this host's early resume, the first phase with interrupts on, and
 * so the first in which the SMC can switch the endpoint's power: the parked ports come back
 * here, right after the phase the radio's port came back in. Only after a sleep whose wake
 * left the radio's port up for good: after a suspend that was abandoned, or a wake that could
 * not reset the radio, its driver takes that port down and up once more, and the shared PHY
 * with it. The parked ports then wait for the end of the resume.
 */
static void apple_pcie_t6050_unpark_cold_early(struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;

	if (t6050_sd_port_wake != 1 || !pcie->t6050_woke || READ_ONCE(pcie->t6050_down) ||
	    pcie->t6050_lost)
		return;
	list_for_each_entry(port, &pcie->ports, entry)
		if (pcie->t6050_parked & BIT(port->idx))
			apple_pcie_t6050_cold_wake(pcie, port, true);
}

/*
 * After a suspend, with every device resumed and the tasks thawed (the park ran before the
 * freeze), from apple_pcie_t6050_unpark_work(): the ports still parked come back, and the
 * functions of every port that is back are found again. Also after a suspend that was
 * abandoned, where the port was parked and nothing was lost.
 */
static void apple_pcie_t6050_unpark_cold(struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;

	list_for_each_entry(port, &pcie->ports, entry) {
		int i;

		if (pcie->t6050_rescan & BIT(port->idx)) {
			apple_pcie_t6050_cold_rescan(pcie, port);
			continue;
		}
		if (!(pcie->t6050_parked & BIT(port->idx)))
			continue;
		if (!t6050_sd_port_wake) {
			dev_info(pcie->dev, "port %d: stays parked after the sleep (t6050_sd_port_wake=0)\n",
				 port->idx);
			continue;
		}
		/*
		 * The shared PHY and the root complex come back with the radio's port, which its
		 * endpoint's driver may be taking down and up once more for the warm reset
		 */
		for (i = 0; i < 40 && READ_ONCE(pcie->t6050_down) && !pcie->t6050_lost; i++)
			msleep(50);
		if (READ_ONCE(pcie->t6050_down) || pcie->t6050_lost) {
			dev_warn(pcie->dev, "port %d: stays parked: the radio's port is not back\n",
				 port->idx);
			continue;
		}
		if (!apple_pcie_t6050_cold_wake(pcie, port, false))
			apple_pcie_t6050_cold_rescan(pcie, port);
	}
}

/*
 * From a work, not from the PM notifier chain: the rescan takes the PCI rescan lock, which the
 * radio's driver holds through its own new enumeration after the wake (more than a second), and
 * every notifier after this one, and the end of the system's resume, waited for it there. The
 * work is freezable, so a later system sleep's freezer waits for it as well as the next
 * PM_SUSPEND_PREPARE; a restart's shutdown cancels it, see apple_pcie_shutdown().
 */
static void apple_pcie_t6050_unpark_work(struct work_struct *work)
{
	struct apple_pcie *pcie = container_of(work, struct apple_pcie, t6050_unpark_work);

	if (pcie->t6050_parked || pcie->t6050_rescan)
		apple_pcie_t6050_unpark_cold(pcie);
}

static int apple_pcie_t6050_pm_notify(struct notifier_block *nb, unsigned long action, void *unused)
{
	struct apple_pcie *pcie = container_of(nb, struct apple_pcie, t6050_pm_nb);

	if (action == PM_POST_SUSPEND) {
		if (pcie->t6050_parked || pcie->t6050_rescan)
			queue_work(system_freezable_wq, &pcie->t6050_unpark_work);
		return NOTIFY_DONE;
	}
	if (action != PM_SUSPEND_PREPARE)
		return NOTIFY_DONE;
	/* The last wake's ports are back, or stay parked, before this sleep parks them again */
	flush_work(&pcie->t6050_unpark_work);
	pcie->t6050_woke = false;
	/* A suspend that never shuts the link down gives no reset: the last wake's doesn't count */
	pcie->t6050_warm_done = false;
	pcie->t6050_link_up = 0;
	pcie->t6050_reset_before_sleep = false;
	/*
	 * The DART wasn't bound when this host probed. Looked up here, before the freezer and any
	 * device callback: a device link added in the noirq phase would reorder this host in the
	 * PM lists in the middle of the suspend.
	 */
	if (!pcie->t6050_dart)
		pcie->t6050_dart = apple_pcie_neo_dart(pcie->dev, false);
	/* A "freeze" is a suspend-to-idle with deep as the default too: its check wants the link. */
	if (!pm_suspend_default_s2idle() && !pm_suspend_no_platform())
		apple_pcie_t6050_park_cold(pcie);
	return NOTIFY_DONE;
}

/*
 * The endpoint's warm reset: "apple,function-warm-reset" on the port names the SMC and the
 * key (the ADT's function-warm_reset), "apple,warm-reset-writes" the writes: the key's value,
 * its bytes from the cell's most significant down, and the wait after it in microseconds. The
 * reset has to be given while the port is down and not set up, and the port set up only after
 * it: the link then trains a chip that was just reset. Given on a trained link, it takes the
 * link down and the link does not come back. Without both properties this host gives no reset.
 */
static void apple_pcie_t6050_warm_reset_probe(struct platform_device *pdev, struct apple_pcie *pcie,
					      struct apple_pcie_port *port)
{
	struct of_phandle_args fn;
	int count, i;

	if (!IS_REACHABLE(CONFIG_MFD_MACSMC) ||
	    of_parse_phandle_with_fixed_args(port->np, "apple,function-warm-reset", 2, 0, &fn))
		return;
	pcie->t6050_smc = of_find_device_by_node(fn.np);
	of_node_put(fn.np);
	if (!pcie->t6050_smc)
		return;
	count = of_property_count_u32_elems(port->np, "apple,warm-reset-writes");
	if (count < 2 || count % 2 || count > ARRAY_SIZE(pcie->t6050_warm_writes) ||
	    of_property_read_u32_array(port->np, "apple,warm-reset-writes",
				       pcie->t6050_warm_writes, count))
		goto drop;
	for (i = 1; i < count; i += 2)
		if (pcie->t6050_warm_writes[i] > 100 * USEC_PER_MSEC)
			goto drop;
	/* The SMC's noirq resume comes before this host's: the reset is written from there */
	if (!device_link_add(&pdev->dev, &pcie->t6050_smc->dev, DL_FLAG_STATELESS)) {
		dev_warn(&pdev->dev, "no warm reset after a sleep: no link to the SMC\n");
		goto drop;
	}
	pcie->t6050_warm_key = fn.args[0];
	pcie->t6050_warm_count = count / 2;
	return;
drop:
	put_device(&pcie->t6050_smc->dev);
	pcie->t6050_smc = NULL;
}

/*
 * With the port down. In the noirq resume the SMC's reply is polled for: the one SMC write
 * outside the interrupt-driven path, because the port has to be up before the PCI core's noirq
 * restore of the root port and the functions, and the reset has to come before the port. A
 * reset requested before sleep uses the same path after the link's shutdown.
 */
static int apple_pcie_t6050_warm_reset(struct apple_pcie *pcie, bool noirq)
{
	struct apple_smc *smc;
	unsigned int i;
	int ret = 0;

	pcie->t6050_warm_done = false;
	if (!IS_REACHABLE(CONFIG_MFD_MACSMC) || !pcie->t6050_warm_count)
		return -ENODEV;
	smc = platform_get_drvdata(pcie->t6050_smc);
	if (!smc) {
		dev_warn(pcie->dev, "no warm reset: the SMC isn't bound\n");
		return -ENODEV;
	}
	for (i = 0; i < pcie->t6050_warm_count && !ret; i++) {
		__be32 value = cpu_to_be32(pcie->t6050_warm_writes[2 * i]);
		u32 wait = pcie->t6050_warm_writes[2 * i + 1];
		smc_key key = pcie->t6050_warm_key;

		if (noirq) {
			ret = apple_smc_write_noirq(smc, key, &value, sizeof(value));
			if (!ret) {
				mdelay(wait / USEC_PER_MSEC);
				udelay(wait % USEC_PER_MSEC);
			}
		} else {
			ret = apple_smc_write(smc, key, &value, sizeof(value));
			if (!ret)
				fsleep(wait);
		}
	}
	pcie->t6050_warm_done = !ret;
	if (ret)
		dev_err(pcie->dev, "endpoint warm reset before the port comes up: %d\n", ret);
	else
		dev_dbg(pcie->dev, "endpoint warm reset before the port comes up\n");
	return ret;
}

/*
 * The adopted host's firmware sleep is set up on the boards with a T6050 sleep, the J714s and
 * the J716c. On another T6050 board nothing is registered and no port is taken for it: the
 * host is as before.
 */
static void apple_pcie_t6050_sleep_probe(struct platform_device *pdev, struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;

	if (pcie->managed_sids || !(of_machine_is_compatible("apple,j714s") ||
				    of_machine_is_compatible("apple,j716c")))
		return;
	/* The one port a boot loader trained; an endpoint Linux powers is parked instead. */
	list_for_each_entry(port, &pcie->ports, entry)
		if (port->perst && port->intr2axi && port->phy && !port->pwren)
			break;
	if (list_entry_is_head(port, &pcie->ports, entry))
		return;
	pcie->t6050_port = port;
	INIT_WORK(&pcie->t6050_unpark_work, apple_pcie_t6050_unpark_work);
	pcie->t6050_pm_nb.notifier_call = apple_pcie_t6050_pm_notify;
	if (register_pm_notifier(&pcie->t6050_pm_nb))
		pcie->t6050_pm_nb.notifier_call = NULL;
	apple_pcie_t6050_warm_reset_probe(pdev, pcie, port);
	/* The host can probe before its DART: the notifier looks again when a sleep starts */
	pcie->t6050_dart = apple_pcie_neo_dart(&pdev->dev, !!pcie->t6050_pm_nb.notifier_call);
}

static int apple_pcie_t6050_sleep_down(struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;
	int slot, ret;

	if (!pcie->t6050_port)
		return -EBUSY;
	if (!pcie->t6050_dart) {
		dev_warn(pcie->dev, "deep sleep refused: the port's DART can't wait for it\n");
		return -EBUSY;
	}
	port = pcie->t6050_port;

	mutex_lock(&pcie->lock);
	apple_pcie_port_disable_irq(port);
	port->saved_intmask = apple_pcie_port_readl(port, PORT_INTMSK);
	for_each_set_bit(slot, &port->adopted_slots, BITS_PER_LONG)
		port->saved_rid2sid[slot] = apple_pcie_port_readl(port,
								  port_rid2sid_offset(port, slot));
	/* The DART's registers go with the port's clock: saved now, untouched until the wake */
	ret = apple_dart_state_save(pcie->t6050_dart);
	if (!ret) {
		apple_dart_quiesce_commands(pcie->t6050_dart);
		WRITE_ONCE(pcie->t6050_down, true);
		ret = apple_sleep_radio_port_down();
		if (ret) {
			/* Nothing was written: the port and its DART are as they were */
			WRITE_ONCE(pcie->t6050_down, false);
			apple_dart_host_resume(pcie->t6050_dart);
		}
	}
	if (ret)
		apple_pcie_port_enable_irq(port);
	mutex_unlock(&pcie->lock);
	if (ret) {
		dev_err(pcie->dev, "port %d: link not shut down for the sleep: %d\n", port->idx, ret);
		return -EBUSY;
	}
	dev_info(pcie->dev, "port %d: link shut down for the sleep\n", port->idx);
	return 0;
}

static int apple_pcie_t6050_sleep_up(struct apple_pcie *pcie, bool noirq)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	bool park = false;
	ktime_t start = ktime_get();
	int ret;

	if (!READ_ONCE(pcie->t6050_down) || pcie->t6050_lost)
		return 0;
	port = pcie->t6050_port;

	/* The sleep driver started the shared PHY and the root complex, or says not to read them */
	ret = apple_sleep_radio_rc_ready();
	if (ret)
		goto lost;
	mutex_lock(&pcie->lock);
	if (!(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN) ||
	    !(readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY)) {
		ret = -ENOLINK;
		goto unlock;
	}
	park = true;
	apple_pcie_t6050_warm_reset(pcie, noirq);
	/* The pin restore left PERST# low, as the shutdown did; it stays low until the train */
	ret = apple_pcie_t6050_perst(port, true) ?:
	      apple_pcie_t6050_port_enable(port) ?:
	      apple_pcie_t6050_port_configure(port);
	if (ret)
		goto unlock;
	apple_sleep_radio_port_wake(0);
	ret = apple_pcie_t6050_port_routes(port, port->saved_rid2sid);
	if (ret)
		goto unlock;
	/* The port is clocked: its DART answers again */
	ret = apple_dart_host_restart(pcie->t6050_dart);
	if (ret)
		goto unlock;
	apple_sleep_radio_port_wake(1);
	ret = apple_pcie_t6050_port_train(port);
	if (ret)
		goto unlock;
	apple_pcie_port_writel(port, ~0, PORT_INTSTAT);
	apple_pcie_port_writel(port, port->saved_intmask, PORT_INTMSK);
	WRITE_ONCE(port->link_failed, false);
	pcie->t6050_link_up = ktime_get();
	WRITE_ONCE(pcie->t6050_down, false);
	dev_info(pcie->dev, "port %d: link up again after the sleep in %lld ms, status %#x\n",
		 port->idx, ktime_ms_delta(ktime_get(), start),
		 apple_pcie_port_readl(port, PORT_LINKSTS));
unlock:
	if (ret && park) {
		/* Clocked but not up: held as a shutdown leaves it, its DART out of reach again */
		apple_dart_quiesce_commands(pcie->t6050_dart);
		apple_pcie_t6050_port_park(port, false);
	}
	mutex_unlock(&pcie->lock);
lost:
	if (!ret) {
		apple_pcie_port_enable_irq(port);
		return 0;
	}
	/* The port, its DART and the functions' config space stay unread until a restart */
	pcie->t6050_lost = true;
	WRITE_ONCE(port->link_failed, true);
	pci_walk_bus(bridge->bus, pci_dev_set_disconnected, NULL);
	dev_err(pcie->dev, "port %d: not back after the sleep: %d (%s); its functions are out of reach until a restart\n",
		port->idx, ret, park ? "port held down" : "port not touched");
	return 0;
}

static struct platform_driver apple_pcie_driver;

static struct apple_pcie *apple_pcie_t6050_from_bridge(struct pci_dev *bridge)
{
	struct pci_host_bridge *host = bridge ? pci_find_host_bridge(bridge->bus) : NULL;
	struct apple_pcie *pcie;

	if (!host || !host->dev.parent || host->dev.parent->driver != &apple_pcie_driver.driver)
		return NULL;
	pcie = pci_host_bridge_priv(host);
	return pcie->t6050_port ? pcie : NULL;
}

/* For the endpoint's driver: the host has reset the endpoint at this wake, before the link. */
bool apple_pcie_t6050_warm_reset_done(struct pci_dev *bridge)
{
	struct apple_pcie *pcie = apple_pcie_t6050_from_bridge(bridge);

	return pcie && pcie->t6050_warm_done;
}
EXPORT_SYMBOL_GPL(apple_pcie_t6050_warm_reset_done);

/*
 * For the endpoint's driver: when this host's last bring-up of the port had the link up, with
 * the 100 ms after the training over (see apple_pcie_t6050_port_train()); 0 when the port is
 * not up from such a bring-up.
 */
ktime_t apple_pcie_t6050_link_up_time(struct pci_dev *bridge)
{
	struct apple_pcie *pcie = apple_pcie_t6050_from_bridge(bridge);

	if (!pcie || READ_ONCE(pcie->t6050_down) || pcie->t6050_lost)
		return 0;
	return pcie->t6050_link_up;
}
EXPORT_SYMBOL_GPL(apple_pcie_t6050_link_up_time);

/*
 * For the endpoint's driver, from its suspend: nothing below the port is to wake the system
 * from this sleep. Firmware left running on the endpoint once its host is gone can ask for the
 * system some time later. Replay its warm-reset sequence after link shutdown, before
 * sleeping. This does not implement selective radio wake or hold a reset signal asserted.
 * The wake resets the endpoint again before the link comes up.
 */
void apple_pcie_t6050_sleep_reset(struct pci_dev *bridge)
{
	struct apple_pcie *pcie = apple_pcie_t6050_from_bridge(bridge);

	if (pcie && pcie->t6050_warm_count)
		pcie->t6050_reset_before_sleep = true;
}
EXPORT_SYMBOL_GPL(apple_pcie_t6050_sleep_reset);

/* In the noirq suspend, the link just shut down for the sleep. */
static int apple_pcie_t6050_reset_for_sleep(struct apple_pcie *pcie)
{
	int ret;

	pcie->t6050_reset_before_sleep = false;
	ret = apple_pcie_t6050_warm_reset(pcie, true);
	if (ret) {
		/*
		 * The PM core skips noirq resume for the device whose suspend failed.
		 * Restore the host here before its children resume. If recovery fails,
		 * the existing lost-port path blocks their config-space accesses.
		 */
		apple_pcie_t6050_sleep_up(pcie, true);
		pcie->t6050_woke = !READ_ONCE(pcie->t6050_down) && !pcie->t6050_lost &&
				   pcie->t6050_warm_done;
		dev_err(pcie->dev, "port %d: sleep refused after endpoint reset failed: %d\n",
			pcie->t6050_port->idx, ret);
		return ret;
	}
	dev_info(pcie->dev, "port %d: endpoint warm reset replayed before sleep\n",
		 pcie->t6050_port->idx);
	/* The wake's reset, not this one, is what the endpoint's driver asks about. */
	pcie->t6050_warm_done = false;
	return 0;
}

/*
 * For the endpoint's driver after a suspend that was abandoned before this host's shutdown:
 * the endpoint was taken down for the sleep and needs its warm reset, but its link never went
 * down, and a reset given on a trained link takes the link down for good. The port goes down
 * as for a sleep, the reset is given with the port down, and the port comes up as after a wake.
 * No function may exist below the port. 0: the endpoint is reset and the link is up.
 */
int apple_pcie_t6050_port_recycle(struct pci_dev *bridge)
{
	struct apple_pcie *pcie = apple_pcie_t6050_from_bridge(bridge);
	int ret;

	if (!pcie)
		return -ENODEV;
	if (!pcie->t6050_warm_count || pcie->t6050_lost || READ_ONCE(pcie->t6050_down))
		return -EBUSY;
	pci_save_state(bridge);
	ret = apple_pcie_t6050_sleep_down(pcie);
	if (ret)
		return ret;
	apple_pcie_t6050_sleep_up(pcie, false);
	if (pcie->t6050_lost)
		return -ENOLINK;
	pci_restore_state(bridge);
	dev_info(pcie->dev, "port %d: taken down and up for the endpoint's warm reset: %s\n",
		 pcie->t6050_port->idx, pcie->t6050_warm_done ? "reset" : "no reset");
	return pcie->t6050_warm_done ? 0 : -EIO;
}
EXPORT_SYMBOL_GPL(apple_pcie_t6050_port_recycle);

static int apple_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct hw_info *hw;
	struct pci_host_bridge *bridge;
	struct device_node *of_port;
	struct apple_pcie *pcie;
	struct device *piodma_supplier = NULL;
	int ret;

	hw = of_device_get_match_data(dev);
	if (!hw)
		return -ENODEV;

	/*
	 * A tunneled PCIe-C port has no host GPIO for PERST#: the tunnel
	 * firmware and m1n1 handoff own its reset state.  Requiring a
	 * reset-gpios provider here rejects the valid synthesized port before
	 * the tunneled setup path can consume that handoff.
	 */
	if (hw->adopt_link) {
		if (!of_get_available_child_count(dev->of_node) ||
		    of_property_present(dev->of_node, "power-domains"))
			return -EINVAL;
		for_each_available_child_of_node_scoped(dev->of_node, child) {
			u32 reg;

			/* A root port 00:<n>.0 on bus 0, n a GP port (0-3) */
			if (of_property_read_u32(child, "reg", &reg) || reg & ~0xf800 ||
			    (reg >> 11) > 3 ||
			    !of_property_read_bool(child, "apple,link-trained-by-bootloader"))
				return -EINVAL;
			/*
			 * A port whose endpoint needs a root-port reset names its PERST#
			 * line: probe waits for the line's controller before any port is
			 * touched (a failed adoption would otherwise swallow the deferral).
			 */
			if (of_property_present(child, "reset-gpios")) {
				ret = apple_pcie_probe_port(child);
				if (ret)
					return dev_err_probe(dev, ret, "Port %pOF probe fail\n",
							     child);
			}
		}
	} else if (!hw->tunneled) {
		/* Check for probe dependencies for all ports first */
		for_each_available_child_of_node(dev->of_node, of_port) {
			ret = apple_pcie_probe_port(of_port);
			if (ret) {
				of_node_put(of_port);
				return dev_err_probe(dev, ret,
						     "Port %pOF probe fail\n", of_port);
			}
		}
	}

	/* Only a host that the board links to a bootstrap engine opens bus 1. */
	if (hw->root_bus_only && IS_ENABLED(CONFIG_PCIE_APPLE_T8140_BOOTSTRAP) &&
	    of_property_present(dev->of_node, "apple,piodma")) {
		ret = apple_piodma_bootstrap_get(dev, &piodma_supplier);
		if (ret)
			return dev_err_probe(dev, ret, "PIODMA supplier unavailable\n");
	}

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;
	pcie->hw = hw;
	ret = apple_pcie_attach_domains(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to attach power domains\n");

	if (of_property_present(dev->of_node, "apple,managed-sids")) {
		u32 ids[15];
		int count = of_property_count_u32_elems(dev->of_node, "apple,managed-sids");

		if (!of_device_is_compatible(dev->of_node, "apple,t8132-pcie") ||
		    !apple_t8132_managed_radio() || count <= 0 || count > 15)
			return -EINVAL;
		ret = of_property_read_u32_array(dev->of_node, "apple,managed-sids", ids, count);
		if (!ret && (count != 2 || ids[0] != 1 || ids[1] != 2))
			ret = -EINVAL;
		if (!ret)
			ret = apple_pcie_managed_sid_mask(ids, count, &pcie->managed_sids);
		if (ret)
			return ret;
	}
	pcie->piodma_supplier = piodma_supplier;
	/* Firmware-owned links cannot be recovered by user reset/removal. */
	if (piodma_supplier || pcie->managed_sids || hw->adopt_link) {
		bridge->no_user_remove = true;
		bridge->reset_sensitive = true;
		bridge->native_pcie_hotplug = false;
		bridge->native_shpc_hotplug = false;
		bridge->native_aer = false;
		bridge->native_dpc = false;
		dev_dbg(dev, "firmware-owned host: automatic AER/DPC recovery disabled\n");
	}
	if (hw->root_bus_only && !piodma_supplier)
		dev_info(dev, "root-port probe only; downstream config is blocked\n");
	pcie->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);
	if (pcie->hw->tunneled) {
		pcie->kernel_init =
			!hw->tunnel_intr2axi &&
			of_property_read_bool(dev->of_node, "apple,pciec-kernel-init");
		pcie->tunnel_cold_init = hw->tunnel_intr2axi &&
			apple_pcie_tunnel_needs_cold_init(dev->of_node);
		ret = apple_pcie_tunnel_init_resources(pdev, pcie);
		if (ret)
			return ret;
		pcie->power_retained =
			apple_pcie_tunnel_power_is_retained(dev->of_node);
		if (pcie->kernel_init) {
			/* ACIO releases this line before populating the host. */
			pcie->reset = devm_reset_control_get_optional_exclusive_released(dev, NULL);
			if (IS_ERR(pcie->reset))
				return dev_err_probe(dev, PTR_ERR(pcie->reset),
						     "PCIe-C reset unavailable\n");
			ret = reset_control_acquire(pcie->reset);
			if (ret)
				return dev_err_probe(dev, ret,
						     "PCIe-C reset is still in use\n");
		}
	}

	mutex_init(&pcie->lock);
	INIT_LIST_HEAD(&pcie->ports);

	/* Resolve PM dependencies before publishing the PCI hierarchy. */
	if (pcie->hw->tunneled) {
		ret = apple_pcie_tunnel_add_links(pcie);
		if (ret)
			return ret;
	}

	/* Every function is added after this and removed before devres runs. */
	if (pcie->tunnel_cold_init || pcie->hw->adopt_link) {
		pcie->tunnel_pci_nb.notifier_call = apple_pcie_tunnel_pci_notify;
		ret = bus_register_notifier(&pci_bus_type, &pcie->tunnel_pci_nb);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, apple_pcie_tunnel_pci_unregister,
					       pcie);
		if (ret)
			return ret;
	}

	ret = apple_msi_init(pcie);
	if (ret)
		return ret;

	/* Only the retained Neo supplier needs admission before child publication. */
	if (pcie->piodma_supplier) {
		ret = apple_piodma_bootstrap_guard_begin(pcie->piodma_supplier, dev);
		if (ret) {
			apple_pcie_cleanup(pcie);
			return dev_err_probe(dev, ret, "Neo lifecycle admission failed\n");
		}
	}

	if (hw->adopt_link)
		bridge->reset_root_port = apple_pcie_t6050_reset_root_port;
	ret = pci_host_common_init(pdev, bridge, &apple_pcie_cfg_ecam_ops);
	if (ret) {
		apple_pcie_cleanup(pcie);
		if (pcie->piodma_supplier) {
			apple_piodma_bootstrap_guard_end(pcie->piodma_supplier);
			/* Dependency deferral is legal only before publishing children. */
			if (ret == -EPROBE_DEFER)
				ret = -EIO;
		}
		return ret;
	}

	/*
	 * Port mappings are allocated by the ECAM init callback. Register cleanup
	 * afterwards so devres runs it before unmapping those registers.
	 */
	ret = devm_add_action(dev, apple_pcie_cleanup, pcie);
	if (ret) {
		/* Drivers must release their IRQs before the domains disappear. */
		pci_host_common_remove(pdev);
		apple_pcie_cleanup(pcie);
		if (pcie->piodma_supplier)
			apple_piodma_bootstrap_guard_end(pcie->piodma_supplier);
		return ret;
	}

	if (pcie->piodma_supplier) {
		/* All fallible devres setup precedes publication of DMA pointers. */
		/* Retained bootstrap ownership forbids host module teardown. */
		if (!try_module_get(THIS_MODULE)) {
			pci_host_common_remove(pdev);
			apple_piodma_bootstrap_guard_end(pcie->piodma_supplier);
			return -ENODEV;
		}
		ret = apple_pcie_neo_enumerate(bridge);
		if (ret && !apple_piodma_bootstrap_retained(pcie->piodma_supplier)) {
			/* No DMA pointer escaped: remove children, then release vetoes. */
			pci_host_common_remove(pdev);
			module_put(THIS_MODULE);
			apple_piodma_bootstrap_guard_end(pcie->piodma_supplier);
			if (ret == -EPROBE_DEFER)
				ret = -EIO;
			return dev_err_probe(dev, ret, "Neo bootstrap preflight failed\n");
		}
		if (ret)
			dev_err(dev, "radio admission failed=%d; host/supplier retained, no retry\n",
				ret);
		else
			apple_pcie_neo_sleep_probe(pdev, pcie);
		return 0;
	}

	if (pcie->hw->tunneled)
		pci_walk_bus(bridge->bus, apple_pcie_tunnel_keep_d0, NULL);
	if (pcie->hw->adopt_link)
		apple_pcie_t6050_sleep_probe(pdev, pcie);

	apple_radio_register(pdev, pcie);
	return 0;
}

typedef int (*apple_pcie_dart_fn)(struct device *dev);

static int apple_pcie_walk_tunnel_darts(struct apple_pcie *pcie,
					apple_pcie_dart_fn fn, bool required)
{
	struct device_node *parent, *child;
	struct platform_device *pdev;
	bool found = false;
	int ret = 0;

	parent = of_get_parent(pcie->dev->of_node);
	if (!parent)
		return -ENODEV;

	for_each_available_child_of_node(parent, child) {
		if (!of_device_is_compatible(child, "apple,t8103-dart") &&
		    !of_device_is_compatible(child, "apple,t8103-usb4-dart") &&
		    !of_device_is_compatible(child, "apple,t8110-dart") &&
		    !of_device_is_compatible(child, "apple,t6000-dart"))
			continue;

		pdev = of_find_device_by_node(child);
		if (pdev) {
			int err = fn(&pdev->dev);

			found = true;
			if (err && !ret)
				ret = err;
			put_device(&pdev->dev);
		} else if (required && !ret) {
			ret = -ENODEV;
		}
	}

	of_node_put(parent);
	return ret ?: (required && !found ? -ENODEV : 0);
}

int apple_pcie_tunnel_quiesce(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);
	struct apple_pcie *pcie;
	struct apple_pcie_port *port;
	int ret = 0;

	if (!bridge || !bridge->bus)
		return -ENODEV;

	pcie = pci_host_bridge_priv(bridge);
	if (!pcie->hw->tunneled)
		return -EINVAL;

	/*
	 * ACIO's NHI is the control plane for tunneled PERST. Stop PCI
	 * function drivers first, then complete the port reset handshake while
	 * the NHI and PCIe-C DART are still alive. The platform device remains
	 * bound so ACIO can remove the NHI before it finally destroys this host.
	 */
	pci_lock_rescan_remove();
	if (!pcie->bus_stopped) {
		struct pci_dev *pdev, *tmp;

		pci_walk_bus(bridge->bus, pci_dev_set_disconnected, NULL);

		/*
		 * Remove rather than merely stop. Stopping unbinds the drivers
		 * but leaves the pci_dev objects behind, and a later rescan
		 * then finds stale devices instead of enumerating fresh ones:
		 * the endpoint returns with an unbalanced runtime-PM count and
		 * its driver fails to probe.
		 */
		list_for_each_entry_safe(pdev, tmp, &bridge->bus->devices,
					 bus_list)
			pci_stop_and_remove_bus_device(pdev);

		pcie->bus_stopped = true;
	}

	/* Endpoint removal may still need INTx while the port is clocked. */
	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_disable_irq(port);

	/*
	 * Removing the IOMMU later resumes it and issues a command. Gate
	 * those commands before APPCLK goes away with the port.
	 */
	ret = apple_pcie_walk_tunnel_darts(pcie, apple_dart_quiesce_commands, false);

	list_for_each_entry(port, &pcie->ports, entry) {
		int err;

		if (!port->started && !port->needs_stop)
			continue;
		err = apple_pcie_tunnel_stop(port);
		if (err && !ret)
			ret = err;
	}
	pci_unlock_rescan_remove();

	if (!ret)
		dev_info(dev, "PCIe-C hierarchy quiesced before NHI shutdown\n");

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_quiesce);

struct apple_pcie_tunnel_use {
	struct pci_host_bridge *bridge;
	unsigned int disks, open;
};

/* The disks below one device, and whether anything has them open. */
static int apple_pcie_tunnel_disk_use(struct device *dev, void *data)
{
	struct apple_pcie_tunnel_use *use = data;

	if (dev->class && dev->class->name && !strcmp(dev->class->name, "block") &&
	    !bdev_is_partition(dev_to_bdev(dev))) {
		use->disks++;
		if (disk_openers(dev_to_disk(dev)))
			use->open++;
	}
	return device_for_each_child(dev, data, apple_pcie_tunnel_disk_use);
}

/* The disks below a function of the tunnelled host. */
static int apple_pcie_tunnel_function_disks(struct pci_dev *pdev, void *data)
{
	return device_for_each_child(&pdev->dev, data, apple_pcie_tunnel_disk_use);
}

/* Only drivers known not to touch the PM notifier chain in their remove are removed there. */
static int apple_pcie_tunnel_other_driver(struct pci_dev *pdev, void *data)
{
	struct device_driver *drv = READ_ONCE(pdev->dev.driver);

	if (!drv || !strcmp(drv->name, "nvme") || !strcmp(drv->name, "pcieport"))
		return 0;
	*(const char **)data = drv->name;
	return 1;
}

/**
 * apple_pcie_tunnel_remove_orderly - remove a tunnelled hierarchy that is still reachable
 * @dev: the tunnelled host
 * @in_use_too: also when a disk below it is open
 *
 * For a sleep that takes the tunnel's router down. Unlike apple_pcie_tunnel_quiesce(), which
 * marks the functions disconnected first (their drivers then treat them as gone), the
 * functions are removed while they still answer: an NVMe controller's driver stops its
 * queues and gives the controller its normal shutdown, as at a power-off. Nothing is written
 * to a disk that has no opener. apple_pcie_tunnel_quiesce() then takes the ports down and
 * apple_pcie_tunnel_restore() enumerates the hierarchy again.
 *
 * Returns 0 when the hierarchy is removed (or was already), -EBUSY when a disk is open and
 * @in_use_too is false, -EOPNOTSUPP when a function has a driver other than nvme or the PCIe
 * port driver. Nothing is removed on an error.
 */
int apple_pcie_tunnel_remove_orderly(struct device *dev, bool in_use_too)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);
	struct apple_pcie_tunnel_use use = { .bridge = bridge };
	struct pci_dev *pdev, *tmp;
	struct apple_pcie *pcie;
	const char *other = NULL;
	unsigned int tries;

	if (!bridge || !bridge->bus)
		return -ENODEV;
	pcie = pci_host_bridge_priv(bridge);
	if (!pcie->hw->tunneled)
		return -EINVAL;

	pci_lock_rescan_remove();
	if (pcie->bus_stopped) {
		pci_unlock_rescan_remove();
		return 0;
	}
	pci_walk_bus(bridge->bus, apple_pcie_tunnel_other_driver, &other);
	if (other) {
		pci_unlock_rescan_remove();
		dev_info(dev, "PCIe-C: a function driven by %s is not removed for the sleep\n", other);
		return -EOPNOTSUPP;
	}
	/* A probe by udev right after the disk appeared closes again within moments. */
	for (tries = 0; tries < 10; tries++) {
		use.disks = use.open = 0;
		pci_walk_bus(bridge->bus, apple_pcie_tunnel_function_disks, &use);
		if (!use.open || in_use_too)
			break;
		msleep(50);
	}
	if (use.open && !in_use_too) {
		pci_unlock_rescan_remove();
		dev_warn(dev, "PCIe-C: %u of %u disk(s) behind the tunnel in use: not removed for the sleep\n",
			 use.open, use.disks);
		return -EBUSY;
	}
	list_for_each_entry_safe(pdev, tmp, &bridge->bus->devices, bus_list)
		pci_stop_and_remove_bus_device(pdev);
	pcie->bus_stopped = true;
	pci_unlock_rescan_remove();
	dev_info(dev, "PCIe-C: hierarchy removed in order for the sleep (%u disk(s), %u in use)\n",
		 use.disks, use.open);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_remove_orderly);

/*
 * Bring the tunneled host back after apple_pcie_tunnel_quiesce(). The ports
 * need the restart that resume performs, and the hierarchy below them has to be
 * enumerated again because quiescing removed it.
 */
int apple_pcie_tunnel_restore(struct device *dev)
{
	struct pci_host_bridge *bridge;
	struct apple_pcie *pcie;
	struct apple_pcie_port *port;
	int ret = 0;

	guard(device)(dev);
	if (!device_is_bound(dev))
		return -EAGAIN;
	bridge = dev_get_drvdata(dev);
	if (!bridge || !bridge->bus)
		return -ENODEV;

	pcie = pci_host_bridge_priv(bridge);
	if (!pcie->hw->tunneled)
		return -EINVAL;
	if (pcie->resume_failed)
		return -EIO;
	if (!pcie->bus_stopped)
		return 0;

	list_for_each_entry(port, &pcie->ports, entry) {
		if (port->started)
			continue;
		if (port->needs_stop) {
			ret = apple_pcie_tunnel_stop(port);
			if (ret)
				return ret;
		}
		ret = apple_pcie_tunnel_start(port);
		if (ret)
			return ret;
	}
	ret = apple_pcie_walk_tunnel_darts(pcie, apple_dart_resume_commands, false);
	if (ret)
		return ret;

	apple_pcie_tunnel_wait_ready(pcie);
	pci_lock_rescan_remove();
	/* Rescan publishes endpoints and can probe interrupt-dependent drivers. */
	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_enable_irq(port);
	pci_rescan_bus(bridge->bus);
	pci_walk_bus(bridge->bus, apple_pcie_tunnel_keep_d0, NULL);
	pcie->bus_stopped = false;
	pci_unlock_rescan_remove();

	dev_info(dev, "PCIe-C hierarchy restored after tunnel activation\n");

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_restore);

/*
 * Check recorded host state without accessing a possibly failed data path.
 * The caller serializes tunnel transitions and calls after system PM complete.
 * The device lock protects driver data against a concurrent unbind.
 */
int apple_pcie_tunnel_check_state(struct device *dev)
{
	struct pci_host_bridge *bridge;
	struct apple_pcie_port *port;
	struct apple_pcie *pcie;

	guard(device)(dev);
	if (!dev->driver)
		return -ENODEV;
	bridge = dev_get_drvdata(dev);
	if (!bridge || !bridge->bus)
		return -ENODEV;
	pcie = pci_host_bridge_priv(bridge);
	if (!pcie->hw->tunneled)
		return -EINVAL;
	if (pcie->resume_failed)
		return -EIO;
	if (pcie->bus_stopped || list_empty(&pcie->ports))
		return -ENOLINK;
	list_for_each_entry(port, &pcie->ports, entry) {
		/* Keep failure latched until a fresh host is probed. */
		if (!port->started || READ_ONCE(port->link_failed))
			return -ENOLINK;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_check_state);

/*
 * Whether the host left its tunnel running for the system sleep in progress.
 * The caller serializes against unbind through device PM ordering.
 */
bool apple_pcie_tunnel_link_kept(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);
	struct apple_pcie *pcie;

	if (!dev->driver || !bridge)
		return false;
	pcie = pci_host_bridge_priv(bridge);
	return pcie->hw->tunneled && pcie->link_kept;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_link_kept);

struct apple_pcie_map {
	void __iomem *base;
	struct resource res;
};

static void apple_pcie_unmap(struct apple_pcie_map *map)
{
	if (map->base)
		iounmap(map->base);
	map->base = NULL;
}

static int apple_pcie_map_named(struct device_node *np, const char *name,
				struct apple_pcie_map *map)
{
	int index, ret;

	index = of_property_match_string(np, "reg-names", name);
	if (index < 0)
		return index;

	ret = of_address_to_resource(np, index, &map->res);
	if (ret)
		return ret;

	map->base = ioremap_np(map->res.start, resource_size(&map->res));
	if (!map->base)
		return -ENOMEM;
	return 0;
}

static struct apple_tunable *apple_pcie_tunable_once(struct device_node *np,
						     const char *name,
						     struct resource *res)
{
	struct apple_tunable *tunable;
	struct property *prop;
	const __be32 *p;
	size_t sz;
	int i;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return ERR_PTR(-ENOENT);
	if (prop->length % (3 * sizeof(u32)))
		return ERR_PTR(-EINVAL);

	sz = prop->length / (3 * sizeof(u32));
	tunable = kzalloc(struct_size(tunable, values, sz), GFP_KERNEL);
	if (!tunable)
		return ERR_PTR(-ENOMEM);
	tunable->sz = sz;

	for (i = 0, p = NULL; i < tunable->sz; i++) {
		p = of_prop_next_u32(prop, p, &tunable->values[i].offset);
		p = of_prop_next_u32(prop, p, &tunable->values[i].mask);
		p = of_prop_next_u32(prop, p, &tunable->values[i].value);
		if (tunable->values[i].offset % 4 ||
		    tunable->values[i].offset > resource_size(res) - 4) {
			kfree(tunable);
			return ERR_PTR(-EINVAL);
		}
	}
	return tunable;
}

/*
 * Clock the tunneled port before of_platform_populate(). The DART child
 * probes immediately and its invalidate command never completes while
 * APPCLK is still gated.
 */
int apple_pcie_tunnel_prepare(struct device *dev, struct device_node *tunnel)
{
	struct device_node *np, *port_np = NULL;
	const struct of_device_id *match;
	const struct hw_info *hw;
	struct apple_pcie *pcie;
	struct apple_pcie_port *port;
	struct apple_pcie_map config = {}, debug = {}, fabric = {}, portmap = {};
	struct apple_pcie_map oe = {}, intr2axi = {};
	bool kernel_init = false;
	u32 stat;
	int ret;

	np = NULL;
	for_each_available_child_of_node(tunnel, np) {
		if (apple_pcie_tunnel_needs_cold_init(np))
			break;
	}
	if (!np)
		return 0;

	/* A qualified bootloader handoff needs no cold initialization. */
	if (!of_property_read_u32(np, "apple,pciec-preinit-status", &stat) &&
	    stat == 1) {
		ret = 0;
		goto out_np;
	}

	match = of_match_node(apple_pcie_of_match, np);
	if (!match) {
		ret = -ENODEV;
		goto out_np;
	}
	hw = match->data;
	kernel_init = !hw->tunnel_intr2axi;

	ret = apple_pcie_map_named(np, "port0", &portmap);
	if (ret)
		goto out_np;

	stat = readl(portmap.base + PORT_STATUS);
	if (stat & PORT_STATUS_READY) {
		dev_info(dev, "PCIe-C port %pOF already clocked, status %#x\n",
			 np, stat);
		ret = 0;
		goto out_np;
	}

	ret = apple_pcie_map_named(np, "config", &config);
	if (ret)
		goto out_np;
	ret = apple_pcie_map_named(np, "debug", &debug);
	if (ret)
		goto out_np;
	ret = apple_pcie_map_named(np, "fabric", &fabric);
	if (ret)
		goto out_np;
	/* t8103 has an oe-fabric region; T600x/T602x pulse Intr2AXI instead. */
	if (kernel_init)
		ret = apple_pcie_map_named(np, "oe-fabric", &oe);
	else
		ret = apple_pcie_map_named(np, "intr2axi", &intr2axi);
	if (ret)
		goto out_np;

	port_np = of_get_next_available_child(np, NULL);
	if (!port_np) {
		ret = -ENODEV;
		goto out_np;
	}

	pcie = kzalloc_obj(*pcie);
	port = kzalloc_obj(*port);
	if (!pcie || !port) {
		ret = -ENOMEM;
		goto out_free;
	}

	pcie->dev = dev;
	pcie->hw = match->data;
	/* Keep the previous fallback on unqualified M3-family machines. */
	if (pcie->hw == &t8122_pciec_hw &&
	    !of_machine_is_compatible("apple,t6030") &&
	    of_device_is_compatible(dev->of_node, "apple,t8103-pciec"))
		pcie->hw = &t8103_pciec_hw;
	pcie->kernel_init = kernel_init;
	pcie->early_cfg = config.base;
	pcie->debug_base = debug.base;
	pcie->fabric_base = fabric.base;
	pcie->oe_fabric_base = oe.base;
	pcie->intr2axi_base = intr2axi.base;
	pcie->debug_tunable = apple_pcie_tunable_once(np, "apple,tunable-debug",
						      &debug.res);
	if (IS_ERR(pcie->debug_tunable)) {
		ret = PTR_ERR(pcie->debug_tunable);
		pcie->debug_tunable = NULL;
		goto out_free;
	}
	pcie->fabric_tunable = apple_pcie_tunable_once(np, "apple,tunable-fabric",
						       &fabric.res);
	if (IS_ERR(pcie->fabric_tunable)) {
		ret = PTR_ERR(pcie->fabric_tunable);
		pcie->fabric_tunable = NULL;
		goto out_free;
	}
	pcie->rc_tunable = apple_pcie_tunable_once(np, "apple,tunable-rc",
						   &config.res);
	if (IS_ERR(pcie->rc_tunable)) {
		ret = PTR_ERR(pcie->rc_tunable);
		pcie->rc_tunable = NULL;
		goto out_free;
	}
	if (oe.base) {
		pcie->oe_fabric_tunable =
			apple_pcie_tunable_once(np, "apple,tunable-oe-fabric",
						&oe.res);
		if (IS_ERR(pcie->oe_fabric_tunable)) {
			ret = PTR_ERR(pcie->oe_fabric_tunable);
			pcie->oe_fabric_tunable = NULL;
			goto out_free;
		}
	}

	port->pcie = pcie;
	port->np = port_np;
	port->base = portmap.base;
	port->tunable = apple_pcie_tunable_once(port_np, "apple,tunable",
						&portmap.res);
	if (IS_ERR(port->tunable)) {
		ret = PTR_ERR(port->tunable);
		port->tunable = NULL;
		goto out_free;
	}

	dev_info(dev, "PCIe-C clocking %pOF before DART probe\n", np);
	ret = apple_pcie_tunnel_cold_init(port);

out_free:
	if (port && !IS_ERR_OR_NULL(port->tunable))
		kfree(port->tunable);
	if (pcie) {
		if (!IS_ERR_OR_NULL(pcie->debug_tunable))
			kfree(pcie->debug_tunable);
		if (!IS_ERR_OR_NULL(pcie->fabric_tunable))
			kfree(pcie->fabric_tunable);
		if (!IS_ERR_OR_NULL(pcie->rc_tunable))
			kfree(pcie->rc_tunable);
		if (!IS_ERR_OR_NULL(pcie->oe_fabric_tunable))
			kfree(pcie->oe_fabric_tunable);
	}
	kfree(port);
	kfree(pcie);
	of_node_put(port_np);
out_np:
	apple_pcie_unmap(&config);
	apple_pcie_unmap(&debug);
	apple_pcie_unmap(&fabric);
	apple_pcie_unmap(&portmap);
	apple_pcie_unmap(&oe);
	apple_pcie_unmap(&intr2axi);
	of_node_put(np);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_prepare);

static void apple_pcie_remove(struct platform_device *pdev)
{
	struct pci_host_bridge *bridge = platform_get_drvdata(pdev);
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);

	/*
	 * A USB4 cable pull is a surprise removal: the remote hierarchy is
	 * already unreachable by the time ACIO depopulates PCIe-C.  Mark it
	 * permanently disconnected before unbinding drivers, matching pciehp's
	 * surprise-removal path.  In particular, this keeps NVMe teardown from
	 * issuing MMIO to the dead tunneled aperture and wedging the SoC.
	 */
	pci_lock_rescan_remove();
	if (!pcie->bus_stopped) {
		if (pcie->hw->tunneled)
			pci_walk_bus(bridge->bus, pci_dev_set_disconnected, NULL);
		pci_stop_root_bus(bridge->bus);
	}
	pci_remove_root_bus(bridge->bus);
	pci_unlock_rescan_remove();
}

/*
 * A restart with an adopted port's link alive never reaches m1n1 on the J714c (hard
 * reset needed). Put each adopted port back the way the boot stage leaves it on a
 * cold start: bridge decoding off, link to L2, PERST# asserted, port and lane clocks
 * stopped (the first half of apple_pcie_t6050_reset_root_port, without the restart).
 */
static void apple_pcie_shutdown(struct platform_device *pdev)
{
	struct pci_host_bridge *bridge = platform_get_drvdata(pdev);
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);
	struct apple_pcie_port *port;
	struct pci_dev *rp;
	u32 value;

	/* A parked port's return after a wake: over, or never started, before the ports are held */
	if (pcie->t6050_port)
		cancel_work_sync(&pcie->t6050_unpark_work);
	/* A port a failed wake left down is held already, and may not answer */
	if (!pcie->hw->adopt_link || READ_ONCE(pcie->t6050_down))
		return;
	mutex_lock(&pcie->lock);
	list_for_each_entry(port, &pcie->ports, entry) {
		if (!port->perst || !port->base || !port->phy)
			continue;
		/* Parked for a sleep and not brought back: held already, and may not answer */
		if (pcie->t6050_parked & BIT(port->idx))
			continue;
		rp = pci_get_domain_bus_and_slot(pci_domain_nr(bridge->bus), 0,
						 PCI_DEVFN(port->idx, 0));
		if (!rp)
			continue;
		pci_write_config_word(rp, PCI_COMMAND, 0);
		if (!pci_read_config_dword(rp, PCI_PRIMARY_BUS, &value))
			pci_write_config_dword(rp, PCI_PRIMARY_BUS, value & 0xff0000ff);
		apple_pcie_t6050_port_park(port, true);
		dev_info(pcie->dev, "port %d held for the restart%s\n", port->idx,
			 port->pwren ? ", its endpoint powered off" : "");
		pci_dev_put(rp);
	}
	mutex_unlock(&pcie->lock);
}

static int apple_pcie_mark_power_lost(struct pci_dev *pdev, void *unused)
{
	/*
	 * Tunnel stop asserts tunneled PERST, so the functions below lose their
	 * state even when the host stays in suspend-to-idle. Make the PCI core
	 * resume them as if from D3cold: restore config space in the noirq phase,
	 * parents first, and wait for each secondary bus before touching it.
	 */
	pdev->skip_bus_pm = false;
	pdev->current_state = PCI_D3cold;
	return 0;
}

static void apple_pcie_tunnel_hierarchy_lost(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	if (bridge && bridge->bus)
		pci_walk_bus(bridge->bus, apple_pcie_mark_power_lost, NULL);
}

static void apple_pcie_tunnel_hierarchy_gone(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	/*
	 * Nothing below a port that failed to resume can be reached. Mark it
	 * disconnected so that neither the PCI core nor function drivers
	 * resuming after us issue I/O into the dead aperture; quiesce removes
	 * the hierarchy once tasks are thawed.
	 */
	if (bridge && bridge->bus)
		pci_walk_bus(bridge->bus, pci_dev_set_disconnected, NULL);
}

static bool apple_pcie_tunnel_link_healthy(struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;

	if (list_empty(&pcie->ports))
		return false;
	list_for_each_entry(port, &pcie->ports, entry) {
		if (!READ_ONCE(port->started) || READ_ONCE(port->link_failed))
			return false;
		if (apple_pcie_port_readl(port, PORT_INTSTAT) &
		    BIT(PORT_INT_LINK_DOWN))
			return false;
		if (!(apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP))
			return false;
	}
	return true;
}

/*
 * genpd powers a domain off for system sleep once every device in it is
 * through its noirq suspend. A host that keeps its link refuses that for
 * the sleep: the domain stays on, and its parents with it. IRQ-safe domains
 * call this in atomic context, so it only counts.
 */
static int apple_pcie_pd_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct apple_pcie_pd *pd = container_of(nb, struct apple_pcie_pd, nb);

	if (action != GENPD_NOTIFY_PRE_OFF)
		return NOTIFY_OK;
	pd->vetoed++;
	return notifier_from_errno(-EBUSY);
}

static void apple_pcie_release_domains(struct apple_pcie *pcie)
{
	unsigned int vetoed = 0;
	struct apple_pcie_pd *pd;

	if (!pcie->pd_held)
		return;
	while (pcie->pd_held) {
		pd = &pcie->pds[--pcie->pd_held];
		dev_pm_genpd_remove_notifier(pd->dev);
		vetoed += pd->vetoed;
	}
	dev_dbg(pcie->dev, "PCIe power domains released after suspend-to-idle (%u power-offs refused)\n",
		vetoed);
}

/*
 * Hold every attached domain through this sleep. Internal links are left
 * running and cannot be retrained after a power cycle. ACIO keeps
 * the domain of its PCIe-C host on for as long as a cable is connected (its
 * power-domain devices are syscore devices, which system sleep leaves
 * alone), so a host with a link up finds its domain on here, and the veto
 * makes sure it stays on. A domain that is already off has no link to keep.
 */
static int apple_pcie_hold_domains(struct apple_pcie *pcie)
{
	struct apple_pcie_pd *pd;
	int ret;

	if (!pcie->num_pds)
		return -ENODEV;
	while (pcie->pd_held < pcie->num_pds) {
		pd = &pcie->pds[pcie->pd_held];
		pd->vetoed = 0;
		pd->nb.notifier_call = apple_pcie_pd_notify;
		ret = dev_pm_genpd_add_notifier(pd->dev, &pd->nb);
		if (ret)
			goto err_release;
		pcie->pd_held++;
		if (!dev_pm_genpd_is_on(pd->dev)) {
			ret = -ENODEV;
			goto err_release;
		}
	}
	return 0;

err_release:
	apple_pcie_release_domains(pcie);
	return ret;
}

/*
 * In suspend-to-idle ACIO and the router links stay powered, so an up
 * tunnel can simply be left running. The functions below keep their state
 * and resume like any other PCIe device; nothing has to be re-trained.
 * The host's power domain must stay on for that: an always-on one does;
 * on a host that opts in (s2idle_hold_domain), any other is held on for the
 * sleep (apple_pcie_hold_domains()).
 */
static bool apple_pcie_keep_link(struct apple_pcie *pcie)
{
	int ret;

	if (!READ_ONCE(s2idle_keep_link) || !pm_suspend_no_platform())
		return false;
	if (!(pcie->kernel_init || pcie->tunnel_cold_init))
		return false;
	if (pcie->bus_stopped || pcie->resume_failed)
		return false;
	if (!pcie->power_retained) {
		if (!pcie->hw->s2idle_hold_domain)
			return false;
		ret = apple_pcie_hold_domains(pcie);
		if (ret) {
			dev_dbg(pcie->dev, "PCIe-C power domain not held (%d): link not kept\n",
				ret);
			return false;
		}
	}
	if (apple_pcie_tunnel_link_healthy(pcie))
		return true;
	apple_pcie_release_domains(pcie);
	return false;
}

#include "pcie-apple-cio.c"

static void apple_pcie_stop_for_sleep(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);
	struct apple_pcie_port *port;
	bool active = false, can_reset;
	int i, ret;

	can_reset = pcie->kernel_init && pcie->power_retained && pcie->reset &&
		    !pcie->bus_stopped && !pcie->resume_failed;
	if (can_reset) {
		ret = apple_pcie_walk_tunnel_darts(pcie, apple_dart_save_tunnel_state, true);
		if (ret) {
			dev_warn(dev, "PCIe-C cold resume unavailable: DART save failed: %d\n",
				 ret);
			can_reset = false;
		}
	}
	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_disable_irq(port);
	list_for_each_entry(port, &pcie->ports, entry) {
		if (port->started) {
			active = true;
			for_each_set_bit(i, port->sid_map, port->sid_map_sz)
				port->saved_rid2sid[i] =
					apple_pcie_port_readl(port,
						port_rid2sid_offset(port, i));
		}
		if (port->started || port->needs_stop) {
			ret = apple_pcie_tunnel_stop(port);
			if (ret)
				can_reset = false;
		}
	}
	pcie->reset_on_resume = can_reset && active;
	if (active && !pcie->bus_stopped)
		apple_pcie_tunnel_hierarchy_lost(dev);
}

static void apple_pcie_neo_save_functions(struct apple_pcie *pcie);

/* Whether Linux trained the port above pdev itself (cold train), rather than adopting it */
static bool apple_pcie_below_kernel_trained_port(struct pci_dev *pdev)
{
	struct pci_config_window *cfg = pdev->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct pci_dev *rp = pcie_find_root_port(pdev);
	struct apple_pcie_port *port;

	if (!rp)
		return false;
	list_for_each_entry(port, &pcie->ports, entry)
		if (port->idx == PCI_SLOT(rp->devfn))
			return !!port->pwren;
	return false;
}

/*
 * The J716c radio drivers save PCI state without entering D3. The PCI core
 * keeps the root port in D0 as well. Its host and DART have no switchable
 * power domains, and the DART saves its live translations without stopping
 * them. For s2idle, retain that running hierarchy instead of resetting it.
 *
 * Check hardware PMCSR rather than current_state: the PCI core may mark a
 * driverless root port PCI_UNKNOWN even though its hardware remains in D0.
 */
static int apple_pcie_check_d0(struct pci_dev *pdev, void *data)
{
	int *error = data;
	u16 value;

	/*
	 * Below a port the boot loader trained (the N1's), every device stays
	 * in D0. Below a port Linux trained itself (the K116s SD reader's), the
	 * PCI core restores what it suspends to D3hot, so only the devices that
	 * asked to stay awake need to remain in D0.
	 */
	if (!device_awake_path(&pdev->dev) &&
	    apple_pcie_below_kernel_trained_port(pdev))
		return 0;

	if (pci_read_config_word(pdev, PCI_VENDOR_ID, &value) ||
	    value != pdev->vendor) {
		*error = -ENODEV;
	} else if (pdev->pm_cap &&
		   (pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &value) ||
		    (value & PCI_PM_CTRL_STATE_MASK) != PCI_D0)) {
		*error = -EBUSY;
	}
	if (*error)
		pci_err(pdev, "adopted link requires accessible D0 devices for s2idle\n");
	return *error;
}

static int apple_pcie_check_adopted_s2idle(struct apple_pcie *pcie)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	int error = 0;

	/*
	 * Qualified on the J716c, the K114c, the K116s and the J714s: the same
	 * T6050 host with the N1 adopted on port 0. The K116s and the J714s
	 * (16- and 14-inch M5 Pro) also have a kernel-trained SD reader port.
	 * Other adopted platforms and deeper sleep remain unqualified.
	 */
	if (!(of_machine_is_compatible("apple,j716c") ||
	      of_machine_is_compatible("apple,k114c") ||
	      of_machine_is_compatible("apple,k116s") ||
	      of_machine_is_compatible("apple,j714s")) || !pm_suspend_no_platform())
		return -EBUSY;

	if (!(readl(pcie->base + CORE_RC_CTL) & CORE_RC_CTL_RUN) ||
	    !(readl(pcie->base + CORE_RC_STAT) & CORE_RC_STAT_READY))
		return -ENOLINK;

	list_for_each_entry(port, &pcie->ports, entry) {
		if (READ_ONCE(port->link_failed) ||
		    !(apple_pcie_port_readl(port, PORT_STATUS) & PORT_STATUS_READY) ||
		    (apple_pcie_port_readl(port, PORT_LINKSTS) &
		     (PORT_LINKSTS_UP | PORT_LINKSTS_BUSY)) != PORT_LINKSTS_UP ||
		    (apple_pcie_port_readl(port, PORT_INTSTAT) & BIT(PORT_INT_LINK_DOWN)))
			return -ENOLINK;
	}
	pci_walk_bus(bridge->bus, apple_pcie_check_d0, &error);
	return error;
}

static int apple_pcie_suspend_noirq(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);

	if (pcie->cio_prepared)
		return apple_pcie_cio_suspend(pcie);
	/* The reversed DART link is safe only with retained clocks or native PM. */
	if (pcie->cio_dart && (!pm_suspend_no_platform() || !s2idle_keep_link))
		return -EBUSY;
	if (pcie->neo_sleep_armed) {
		struct apple_pcie_port *port;

		pcie->neo_rc_was_down = READ_ONCE(pcie->neo_rc_down);
		/* A port that didn't come back from the last sleep keeps what it saved then. */
		list_for_each_entry(port, &pcie->ports, entry) {
			int i;

			apple_pcie_port_disable_irq(port);
			if (pcie->neo_rc_was_down)
				continue;
			port->saved_intmask = apple_pcie_port_readl(port, PORT_INTMSK);
			for_each_set_bit(i, port->sid_map, port->sid_map_sz)
				port->saved_rid2sid[i] =
					apple_pcie_port_readl(port, port_rid2sid_offset(port, i));
		}
		/* The functions too: the core read them through closed config space. */
		if (!pcie->neo_rc_was_down)
			apple_pcie_neo_save_functions(pcie);
		WRITE_ONCE(pcie->neo_rc_down, true);
		/*
		 * Genpd would gate the host here, and an abandoned sleep then finds
		 * it unprogrammed with nothing to bring it up. The commit lowers it.
		 */
		device_set_awake_path(dev);
		return 0;
	}

	/*
	 * The managed J713 radio has a native port/DART recovery transaction.
	 * Suspend-to-idle leaves its link in place (no power domain).
	 */
	if (pcie->managed_sids && pm_suspend_target_state != PM_SUSPEND_TO_IDLE)
		return apple_radio_suspend_noirq(pcie);
	if (!pcie->managed_sids && pcie->hw->adopt_link) {
		int ret;

		/* Down since a wake that failed: nothing to keep or shut down, nothing read */
		if (READ_ONCE(pcie->t6050_down))
			return 0;
		if (pm_suspend_via_firmware()) {
			/* The root ports below are suspended: from here a parked port's is not read */
			WRITE_ONCE(pcie->t6050_cold_lost, pcie->t6050_parked);
			ret = apple_pcie_t6050_sleep_down(pcie);
			if (!ret && pcie->t6050_reset_before_sleep)
				ret = apple_pcie_t6050_reset_for_sleep(pcie);
			return ret;
		}
		ret = apple_pcie_check_adopted_s2idle(pcie);

		pcie->link_kept = !ret;
		return ret;
	}
	if (!pcie->hw->tunneled) {
		if (pm_suspend_no_platform() && pcie->num_pds)
			return apple_pcie_hold_domains(pcie);
		return 0;
	}
	pcie->reset_on_resume = false;
	pcie->link_kept = apple_pcie_keep_link(pcie);
	if (!pcie->link_kept) {
		if (pcie->cio_dart)
			return -EBUSY;
		apple_pcie_stop_for_sleep(dev);
	}
	return 0;
}

static int apple_pcie_reset_for_resume(struct apple_pcie *pcie)
{
	struct apple_pcie_port *port;
	int i, ret;

	/*
	 * Keep the hierarchy bound and suspended. Reset only after its I/O
	 * drained, then restore the port and IOMMU before releasing tunneled
	 * reset. Endpoints must never run with missing address translations.
	 */
	ret = reset_control_reset(pcie->reset);
	if (ret)
		return ret;

	list_for_each_entry(port, &pcie->ports, entry) {
		u32 stat;

		/* A failed cold setup still owns partially enabled hardware. */
		port->needs_stop = true;
		apple_pcie_port_writel(port, PORT_TUNCTRL_PERST_ON, apple_pcie_port_tunctrl(port));
		ret = apple_pcie_tunnel_cold_init(port);
		if (ret)
			return ret;
		ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
					       stat & PORT_TUNSTAT_PERST_ON,
					       1000, 100000, false, port,
				       apple_pcie_port_tunstat(port));
		if (ret)
			return ret;
		apple_pcie_tunnel_restore_irq_hw(port);
		for_each_set_bit(i, port->sid_map, port->sid_map_sz)
			apple_pcie_rid2sid_write(port, i, port->saved_rid2sid[i]);
	}
	ret = apple_pcie_walk_tunnel_darts(pcie, apple_dart_restore_tunnel_state, true);
	if (ret)
		return ret;

	dev_info(pcie->dev, "PCIe-C reset and DART state restored before link resume\n");
	return 0;
}

/*
 * The radio kept its power and firmware, in reset since the commit. Its rail isn't touched:
 * cycling it, or the port's own reset, is what a cold start needs. PERST# released, the link
 * trains. Device interrupts are off in this phase, so this polls for it.
 */
static int apple_pcie_neo_link_up(struct apple_pcie *pcie, struct apple_pcie_port *port)
{
	ktime_t released;
	s64 left_us;
	u32 stat;
	int ret;

	usleep_range(1000, 2000);
	ret = gpiod_set_value_cansleep(port->perst_gpio, 0);
	if (ret)
		return ret;
	released = ktime_get();
	/* macOS starts the training right after PERST#. */
	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat, stat & PORT_STATUS_READY,
					 100, 250000);
	if (ret)
		return ret;
	apple_pcie_port_writel(port, PORT_LTSSMCTL_START, PORT_LTSSMCTL);
	ret = readl_relaxed_poll_timeout(port->base + PORT_LINKSTS, stat, stat & PORT_LINKSTS_UP,
					 100, link_up_timeout * 1000);
	if (ret)
		return ret;
	apple_pcie_port_writel(port, BIT(PORT_INT_LINK_UP), PORT_INTSTAT);
	/* No config request before 100 ms after PERST#, on a 5 GT/s port (PCIe r5.0, 6.6.1). */
	left_us = 100 * USEC_PER_MSEC - ktime_us_delta(ktime_get(), released);
	if (left_us > 0)
		fsleep(left_us);

	return 0;
}

/*
 * The PCI core saves the functions again at every sleep, and with the radio down it reads them
 * through closed config space. Keep the last state they saved while they answered.
 */
static void apple_pcie_neo_save_functions(struct apple_pcie *pcie)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct pci_dev *root, *fn;
	int i;

	/* Pairs with the release once the functions' configuration is loaded. */
	if (!smp_load_acquire(&pcie->neo_config_ready))
		return;
	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return;
	for (i = 0; root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_state); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		kfree(pcie->neo_fn_state[i]);
		pcie->neo_fn_state[i] = pci_store_saved_state(fn);
		pcie->neo_fn_pm_cap[i] = fn->pm_cap;
		pci_dev_put(fn);
	}
	pci_dev_put(root);
}

/* With the link back: what the core restores next is that state. */
static void apple_pcie_neo_load_functions(struct apple_pcie *pcie, struct pci_dev *root)
{
	struct pci_dev *fn;
	int i;

	for (i = 0; root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_state); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		if (pcie->neo_fn_state[i])
			pci_load_saved_state(fn, pcie->neo_fn_state[i]);
		pci_dev_put(fn);
	}
}

/*
 * Closed config space keeps the PCI core's restore out of the functions, except for one thing:
 * it writes an MSI-X table in their BAR, with no check of its own. With the functions out of
 * reach, set that aside so that it has nothing to write; their reach back puts it in place.
 */
static void apple_pcie_neo_msix_gate(struct apple_pcie *pcie, bool reachable)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct pci_dev *root, *fn;
	int i;

	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return;
	for (i = 0; root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_msix); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		if (reachable && pcie->neo_fn_msix[i]) {
			fn->msix_enabled = 1;
			pcie->neo_fn_msix[i] = false;
		} else if (!reachable && fn->msix_enabled) {
			fn->msix_enabled = 0;
			pcie->neo_fn_msix[i] = true;
		}
		pci_dev_put(fn);
	}
	pci_dev_put(root);
}

/* On the link: the one engine read that native ECAM needs, then the functions' config. */
static int apple_pcie_neo_bootstrap_up(struct apple_pcie *pcie, struct pci_dev *root)
{
	int ret;

	ret = apple_piodma_bootstrap_power_lost(pcie->piodma_supplier);
	if (!ret)
		ret = apple_piodma_bootstrap_prime(pcie->piodma_supplier, root);
	if (!ret) {
		apple_piodma_bootstrap_admitted(pcie->piodma_supplier);
		apple_pcie_neo_load_functions(pcie, root);
		/* The functions' configuration before the flag that says it's there. */
		smp_store_release(&pcie->neo_config_ready, true);
		apple_pcie_neo_msix_gate(pcie, true);
	}
	return ret;
}

/*
 * The link back, and the one engine read that native ECAM needs on it. The root port's
 * bus number is restored first, as that read crosses it. Not a firmware load.
 */
static int apple_pcie_neo_radio_up(struct apple_pcie *pcie, struct apple_pcie_port *port)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct pci_dev *root;
	int ret;

	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return -ENODEV;
	pci_restore_state(root);
	ret = apple_pcie_neo_link_up(pcie, port);
	if (!ret)
		ret = apple_pcie_neo_bootstrap_up(pcie, root);
	pci_dev_put(root);

	return ret;
}

/*
 * After a sleep: the link, then the functions' config once the PCI core's resume_noirq is
 * past them, which found nothing to restore. Their resume, and anything else below the host,
 * waits for this through the bootstrap supplier's resume.
 */
static void apple_pcie_neo_radio_work(struct work_struct *work)
{
	struct apple_pcie *pcie = container_of(work, struct apple_pcie, neo_radio_work);
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	struct pci_dev *root, *fn;
	int i, ret;

	port = list_first_entry(&pcie->ports, struct apple_pcie_port, entry);
	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	ret = root ? apple_pcie_neo_link_up(pcie, port) : -ENODEV;
	if (!ret && !wait_for_completion_timeout(&pcie->neo_noirq_done, HZ))
		ret = -ETIMEDOUT;
	if (!ret)
		ret = apple_pcie_neo_bootstrap_up(pcie, root);
	WRITE_ONCE(pcie->neo_radio_waking, false);
	for (i = 0; !ret && root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_state); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		pci_restore_state(fn);
		pci_dev_put(fn);
	}
	if (ret) {
		/* The functions' resume then finds nothing, and their drivers say so. */
		WRITE_ONCE(pcie->neo_config_ready, false);
		apple_pcie_neo_msix_gate(pcie, false);
		dev_err(pcie->dev, "radios did not come back after sleep: %d\n", ret);
		apple_pcie_neo_recover(pcie);
	} else {
		dev_dbg(pcie->dev, "radio link back after sleep, rail and firmware kept\n");
	}
	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_enable_irq(port);
	pci_dev_put(root);
	apple_piodma_bootstrap_radio_done(pcie->piodma_supplier);
}

/*
 * The rail is the one thing that brings back a chip whose link won't train: macOS's last resort
 * for a dead chip too. A cold chip then has its firmware loaded by the probe that follows.
 */
static int apple_pcie_neo_rail_cycle(struct apple_pcie *pcie, struct apple_pcie_port *port,
				     struct pci_dev *root)
{
	struct pci_dev *fn;
	int i, ret;

	ret = gpiod_set_value_cansleep(port->perst_gpio, 1);
	if (!ret)
		ret = gpiod_set_value_cansleep(port->pwren_gpio, 0);
	if (ret)
		return ret;
	msleep(pcie->hw->pwren_off_ms);
	ret = gpiod_set_value_cansleep(port->pwren_gpio, 1);
	if (ret)
		return ret;
	msleep(pcie->hw->pwren_on_ms);
	ret = apple_pcie_neo_radio_up(pcie, port);
	if (ret)
		return ret;
	/* The PCI core's restore found them gone. Wi-Fi's stop needs its BAR back. */
	for (i = 0; root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_state); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		pci_set_power_state(fn, PCI_D0);
		pci_restore_state(fn);
		pci_dev_put(fn);
	}
	return 0;
}

/* The way a hotplug would. Bluetooth first: Wi-Fi's reset needs it gone. */
static int apple_pcie_neo_stop_functions(struct apple_pcie *pcie, struct pci_dev *root)
{
	struct pci_dev *fn;
	int ret = 0;

	/* Stopping Wi-Fi writes its BAR, which needs the chip to answer. */
	fn = root->subordinate ? pci_get_slot(root->subordinate, PCI_DEVFN(0, 0)) : NULL;
	if (fn && !pci_device_is_present(fn))
		ret = -ENODEV;
	pci_dev_put(fn);
	if (ret)
		return ret;

	pci_lock_rescan_remove();
	if (root->subordinate) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, 1));
		if (fn) {
			pci_stop_and_remove_bus_device(fn);
			pci_dev_put(fn);
		}
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, 0));
		if (fn) {
			ret = apple_piodma_bootstrap_radio_stop(pcie->piodma_supplier, fn);
			if (!ret)
				pci_stop_and_remove_bus_device(fn);
			pci_dev_put(fn);
		}
	}
	pci_unlock_rescan_remove();

	return ret;
}

static void apple_pcie_neo_scan_functions(struct pci_dev *root)
{
	pci_lock_rescan_remove();
	if (root->subordinate) {
		pci_scan_child_bus(root->subordinate);
		pci_assign_unassigned_bridge_resources(root);
		pci_bus_add_devices(root->subordinate);
	}
	pci_unlock_rescan_remove();
}

/*
 * Keeping the firmware across a sleep is what's wanted, and this is for when it didn't work: the
 * radio lost power, its rings or its IPC. Removing the functions resets the chip, and their
 * probes load the firmware again from ROM, as at boot. The link only needs its rail cycled if
 * it didn't come back. Otherwise the next sleep tries the whole wake again.
 */
static void apple_pcie_neo_recover_work(struct work_struct *work)
{
	struct apple_pcie *pcie = container_of(work, struct apple_pcie, neo_recover_work);
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	struct pci_dev *root;
	int ret = 0;

	/* A wake whose radio work outlived the resume finishes it first. */
	flush_work(&pcie->neo_radio_work);
	WRITE_ONCE(pcie->neo_recover, false);
	port = list_first_entry(&pcie->ports, struct apple_pcie_port, entry);
	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return;
	if (READ_ONCE(pcie->neo_rc_down)) {
		ret = -EIO;
		goto out;
	}
	/* Pairs with the release once the functions' configuration is loaded. */
	if (!smp_load_acquire(&pcie->neo_config_ready)) {
		dev_warn(pcie->dev, "radio link down after sleep: cycling its rail\n");
		ret = apple_pcie_neo_rail_cycle(pcie, port, root);
		if (ret)
			goto out;
	}
	ret = apple_pcie_neo_stop_functions(pcie, root);
	if (ret != -ENODEV)
		apple_pcie_neo_scan_functions(root);
out:
	if (ret)
		dev_err(pcie->dev, "radios stay down until the next sleep: %d\n", ret);
	else
		dev_info(pcie->dev, "radios restarted after sleep, firmware loaded again\n");
	pci_dev_put(root);
}

/*
 * Before the PCI core restores the functions, which need the link. macOS's host bridge
 * holds them back the same way until its radio port is enabled.
 */
static int apple_pcie_neo_resume_noirq(struct apple_pcie *pcie)
{
	struct pci_host_bridge *bridge = pci_host_bridge_from_priv(pcie);
	struct apple_pcie_port *port;
	struct pci_dev *root, *fn;
	int i, ret = 0;

	/* Refused: as before the sleep. A host a failed wake left unprogrammed stays down. */
	if (!pm_resume_via_firmware()) {
		if (pcie->neo_rc_was_down)
			return 0;
		list_for_each_entry(port, &pcie->ports, entry)
			apple_pcie_port_enable_irq(port);
		WRITE_ONCE(pcie->neo_rc_down, false);
		return 0;
	}

	list_for_each_entry(port, &pcie->ports, entry) {
		/* The pin controller just put it back as it was: released, with no clock yet. */
		ret = gpiod_set_value_cansleep(port->perst_gpio, 1);
		if (!ret)
			ret = apple_pcie_neo_cold_init(pcie, port);
		if (ret) {
			dev_err(pcie->dev, "host did not come back after sleep: %d\n", ret);
			WRITE_ONCE(pcie->neo_config_ready, false);
			apple_pcie_neo_msix_gate(pcie, false);
			return ret;
		}
		apple_pcie_tunnel_restore_irq_hw(port);
		apple_pcie_port_writel(port, port->saved_intmask, PORT_INTMSK);
		for_each_set_bit(i, port->sid_map, port->sid_map_sz)
			apple_pcie_rid2sid_write(port, i, port->saved_rid2sid[i]);
	}
	ret = apple_dart_host_resume(pcie->neo_cold->dart);
	if (ret) {
		WRITE_ONCE(pcie->neo_config_ready, false);
		apple_pcie_neo_msix_gate(pcie, false);
		return ret;
	}
	WRITE_ONCE(pcie->neo_rc_down, false);
	apple_sleep_radio_port_up();

	/*
	 * The link takes 100 ms, which nothing else here waits for: the work below brings it
	 * back. The root port's bus number now, before the PCI core restores the root port. The
	 * functions' config waits for their link, so the PCI core has nothing to restore there.
	 */
	root = pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
	if (!root)
		return -ENODEV;
	pci_restore_state(root);
	for (i = 0; root->subordinate && i < ARRAY_SIZE(pcie->neo_fn_state); i++) {
		fn = pci_get_slot(root->subordinate, PCI_DEVFN(0, i));
		if (!fn)
			continue;
		fn->state_saved = false;
		pci_dev_put(fn);
	}
	pci_dev_put(root);
	WRITE_ONCE(pcie->neo_radio_waking, true);
	reinit_completion(&pcie->neo_noirq_done);
	apple_piodma_bootstrap_radio_pending(pcie->piodma_supplier);
	queue_work(system_highpri_wq, &pcie->neo_radio_work);

	return 0;
}

/* The PCI core's resume_noirq is done everywhere, the radio functions' included. */
static int apple_pcie_resume_early(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);

	if (pcie && pcie->neo_cold)
		complete_all(&pcie->neo_noirq_done);
	/* T6050: the ports parked for the sleep, now that the SMC can power their endpoints */
	if (pcie && pcie->t6050_parked)
		apple_pcie_t6050_unpark_cold_early(pcie);
	return 0;
}

static int apple_pcie_resume_noirq(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);
	struct apple_pcie_port *port;
	int ret;

	/* The domain's hold for the sleep ends first, whatever follows. */
	apple_pcie_release_domains(pcie);
	if (pcie->cio_prepared) {
		ret = apple_pcie_cio_resume(pcie);
		if (!ret)
			return 0;
		/*
		 * A dock that left or lost power during the sleep. End like a
		 * surprise removal: the port stays stopped and config space
		 * closed, and the hierarchy is removed with the tunnel after
		 * resume. The NHI still ends the DART transaction.
		 */
		pcie->resume_failed = true;
		apple_pcie_tunnel_hierarchy_gone(dev);
		/*
		 * TEST: T6050. The router's host said the tunnel's peer is gone: an unplug that
		 * happened during the sleep, not a failed resume. The port is closed as above,
		 * its DART is back for the removal that follows, and the system's resume is not
		 * failed for it (run sv6t4: "PM: failed to resume noirq: error -107").
		 */
		if (ret == -ENOTCONN && pcie->cio_lost && pcie->hw == &t6050_pciec_hw &&
		    (of_machine_is_compatible("apple,j714s") ||
		     of_machine_is_compatible("apple,j716c"))) {
			dev_warn(dev, "T6050 sleep: PCIe-C tunnel gone with its dock during the sleep: the port is closed as after an unplug, its functions return when the dock is plugged in again\n");
			return 0;
		}
		dev_err(dev, "PCIe-C not back after deep sleep: %d\n", ret);
		return ret;
	}
	if (pcie->managed_sids)
		return apple_radio_resume_noirq(pcie);
	if (pcie->neo_sleep_armed)
		return apple_pcie_neo_resume_noirq(pcie);
	if (pcie->hw->adopt_link) {
		/* Also after a sleep that was abandoned: the link is down either way */
		if (READ_ONCE(pcie->t6050_down)) {
			ret = apple_pcie_t6050_sleep_up(pcie, true);
			/* Up, and the endpoint reset if it has a reset: nothing takes it down again */
			pcie->t6050_woke = !READ_ONCE(pcie->t6050_down) && !pcie->t6050_lost &&
					   (pcie->t6050_warm_done || !pcie->t6050_warm_count);
			list_for_each_entry(port, &pcie->ports, entry)
				if (pcie->t6050_parked & BIT(port->idx))
					dev_dbg(dev, "port %d at the wake: parked since before the sleep, power off, PERST# low, no functions, root port %s, DART %s; %s\n",
						port->idx,
						pcie->t6050_cold_lost & BIT(port->idx) ?
						"not read" : "readable",
						port->cold_dart ? "gated" : "not this host's",
						!t6050_sd_port_wake ?
						"it stays parked (t6050_sd_port_wake=0)" :
						pcie->t6050_lost ? "it stays parked, the radio's port is lost" :
						t6050_sd_port_wake == 1 && pcie->t6050_woke ?
						"it comes back in the early resume" :
						"it comes back once the resume is over");
			return ret;
		}
		if (!pcie->link_kept)
			return 0;
		pcie->link_kept = false;
		ret = apple_pcie_check_adopted_s2idle(pcie);
		if (ret) {
			/* Block downstream config access if the retained link was lost. */
			list_for_each_entry(port, &pcie->ports, entry)
				WRITE_ONCE(port->link_failed, true);
			dev_err(dev, "adopted link lost during suspend-to-idle: %d\n", ret);
		}
		return ret;
	}
	if (!pcie->hw->tunneled)
		return 0;
	if (pcie->resume_failed)
		return -EIO;
	/*
	 * A host quiesced by a tunnel deactivation has no hierarchy and its
	 * ports are stopped. Leave them stopped: restarting them without a
	 * tunnel fails and marks the host dead, while
	 * apple_pcie_tunnel_restore() restarts them on reactivation.
	 */
	if (pcie->bus_stopped)
		return 0;
	if (pcie->link_kept) {
		pcie->link_kept = false;
		if (apple_pcie_tunnel_link_healthy(pcie))
			return 0;
		dev_warn(dev, "PCIe-C link lost during suspend-to-idle\n");
		/*
		 * A cold-initialized T600x/T602x port does not train again after
		 * an in-place restart. Fail like a surprise unplug instead: ACIO
		 * revalidates the connection after resume and replaces this host
		 * with a freshly cold-initialized one.
		 */
		if (pcie->tunnel_cold_init) {
			ret = -ENOLINK;
			goto failed;
		}
		/* Lost while asleep: take the same path as a stopped tunnel. */
		apple_pcie_stop_for_sleep(dev);
	}
	if (pcie->reset_on_resume) {
		pcie->reset_on_resume = false;
		ret = apple_pcie_reset_for_resume(pcie);
		if (ret) {
			dev_err(dev, "PCIe-C cold resume failed: %d\n", ret);
			goto failed;
		}
	}
	list_for_each_entry(port, &pcie->ports, entry) {
		ret = apple_pcie_tunnel_start(port);
		if (ret)
			goto failed;
	}
	list_for_each_entry(port, &pcie->ports, entry)
		apple_pcie_port_enable_irq(port);

	return 0;

failed:
	/*
	 * Ports the sleep path stopped are still gated and masked; nothing has
	 * reopened them. A retained link that failed was never gated: leave it
	 * serviceable, as after a surprise unplug, until teardown masks it.
	 */
	/* Device PM keeps resuming dependents even after this callback fails. */
	pcie->resume_failed = true;
	if (!pcie->bus_stopped)
		apple_pcie_tunnel_hierarchy_gone(dev);
	apple_pcie_walk_tunnel_darts(pcie, apple_dart_quiesce_commands, false);
	/*
	 * Leave the port clocked with its link down, the state a surprise
	 * unplug leaves behind, in which stray accesses complete with an
	 * error. Quiesce removes the hierarchy and then stops the port when
	 * ACIO is torn down after resume.
	 */
	return ret;
}

static int apple_pcie_prepare(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);

	/*
	 * A host-owned PCIe-C DART sleeps only inside the NHI's deep-sleep
	 * transaction, prepared before this, or with a kept s2idle link.
	 * Refuse anything else now, before other devices suspend.
	 */
	if (pcie && pcie->cio_dart && !pcie->cio_prepared &&
	    pm_suspend_target_state != PM_SUSPEND_ON &&
	    (!pm_suspend_no_platform() || !s2idle_keep_link)) {
		dev_warn(dev, "no CIO sleep transaction from the NHI; refusing sleep\n");
		return -EBUSY;
	}
	return apple_radio_prepare(dev);
}

static const struct dev_pm_ops apple_pcie_pm_ops = {
	.prepare = apple_pcie_prepare,
	.complete = apple_radio_complete,
	.suspend_noirq = apple_pcie_suspend_noirq,
	.resume_noirq = apple_pcie_resume_noirq,
	.resume_early = apple_pcie_resume_early,
};

static const struct of_device_id apple_pcie_of_match[] = {
	{ .compatible = "apple,t6040-pciec",	.data = &t6040_pciec_hw },
	{ .compatible = "apple,t6050-pciec",	.data = &t6050_pciec_hw },
	{ .compatible = "apple,t8132-pciec",	.data = &t8132_pciec_hw },
	{ .compatible = "apple,t8122-pciec",	.data = &t8122_pciec_hw },
	{ .compatible = "apple,t8112-pciec",	.data = &t8112_pciec_hw },
	{ .compatible = "apple,t8103-pciec",	.data = &t8103_pciec_hw },
	{ .compatible = "apple,t6000-pciec",	.data = &t8112_pciec_hw },
	/* T8132 currently inherits controller and PHY setup from m1n1. */
	{ .compatible = "apple,t8132-pcie",	.data = &t602x_hw },
	{ .compatible = "apple,t8152-pcie",	.data = &t8152_hw },
	{ .compatible = "apple,t6020-pcie",	.data = &t602x_hw },
	{ .compatible = "apple,t8140-pcie",	.data = &t8140_hw },
	{ .compatible = "apple,t6050-pcie",	.data = &t6050_hw },
	{ .compatible = "apple,pcie",		.data = &t8103_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pcie_of_match);

static struct platform_driver apple_pcie_driver = {
	.probe	= apple_pcie_probe,
	.remove	= apple_pcie_remove,
	.shutdown = apple_pcie_shutdown,
	.driver	= {
		.name			= "pcie-apple",
		.of_match_table		= apple_pcie_of_match,
		.suppress_bind_attrs	= true,
		.pm			= &apple_pcie_pm_ops,
	},
};
module_platform_driver(apple_pcie_driver);

MODULE_DESCRIPTION("Apple PCIe host bridge driver");
MODULE_LICENSE("GPL v2");
