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
#include <linux/msi.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pci-apple.h>
#include <linux/soc/apple/pci-apple-piodma.h>
#include <linux/pci-ecam.h>
#include <linux/pm_runtime.h>
#include <linux/rcupdate.h>
#include <linux/reset.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/tunable.h>
#include <linux/suspend.h>

#include "../pci.h"
#include "pci-host-common.h"
#include "pcie-apple-piodma-diag.h"

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

static int link_up_timeout = 500;
module_param(link_up_timeout, int, 0644);
MODULE_PARM_DESC(link_up_timeout, "PCIe link training timeout in milliseconds");

static bool tunnel_kernel_init = true;
module_param(tunnel_kernel_init, bool, 0444);
MODULE_PARM_DESC(tunnel_kernel_init,
		 "Cold-initialize T600x/T602x PCIe-C ports without an m1n1 handoff (default: on)");

static const char *const apple_pcie_tunnel_cold_init_machines[] = {
	"apple,t6000", "apple,t6001", "apple,t6020", "apple,t6021", NULL
};

/**
 * apple_pcie_tunnel_needs_cold_init() - whether the kernel brings up a port
 * @np: PCIe-C host node
 *
 * No m1n1 performs the PCIe-C preinit handoff on T600x/T602x, so the kernel
 * cold-initializes these ports instead; pcie_apple.tunnel_kernel_init=0 turns
 * that off. It is a parameter rather than a DT property because m1n1 hands
 * every installed kernel the same DT. The tunnel's DART cannot probe without
 * its tunables, which m1n1 does not provide, so ports whose DART lacks them
 * are refused.
 *
 * Unlike "apple,pciec-kernel-init" (t8103), this does not select the t8103
 * power and resume sequences.
 */
bool apple_pcie_tunnel_needs_cold_init(struct device_node *np)
{
	struct of_phandle_args iommu_spec = {};
	struct device_node *dart;
	bool tunables;
	u32 status;

	if (!tunnel_kernel_init ||
	    !of_device_is_compatible(np, "apple,t6000-pciec") ||
	    !of_machine_compatible_match(apple_pcie_tunnel_cold_init_machines))
		return false;

	if (!of_property_read_u32(np, "apple,pciec-preinit-status", &status) &&
	    status == 1)
		return false;

	if (of_map_iommu_id(np, 0, &iommu_spec) || !iommu_spec.np)
		return false;
	dart = iommu_spec.np;
	tunables = of_property_present(dart, "apple,tunable");
	if (!tunables)
		pr_warn_once("%pOF: no PCIe-C DART tunables, not cold-initializing\n",
			     dart);
	of_node_put(dart);
	return tunables;
}
EXPORT_SYMBOL_GPL(apple_pcie_tunnel_needs_cold_init);

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
#define   PHY_LANE_CFG_REFCLKEN		(BIT(9) | BIT(10))
#define   PHY_LANE_CFG_REFCLKCGEN	(BIT(30) | BIT(31))
#define PHY_LANE_CTL			0x00004
#define   PHY_LANE_CTL_CFGACC		BIT(15)

#define PORT_LTSSMCTL			0x00080
#define   PORT_LTSSMCTL_START		BIT(0)
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
#define PORT_T602X_RID2SID	0x3000
#define PORT_T602X_MSIMAP	0x3800

#define PORT_MSIMAP_ENABLE	BIT(31)
#define PORT_MSIMAP_TARGET	GENMASK(7, 0)

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
	u32 max_rid2sid;
	u32 pwren_off_ms;
	u32 pwren_on_ms;
	bool sid_indexed_rid2sid;
	bool retain_clocks;
	bool root_bus_only;
	bool tunneled;
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

static const struct hw_info t8103_pciec_hw = {
	.port_msiaddr		= PORT_MSIADDR,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.max_rid2sid		= 64,
	.tunneled		= true,
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

struct apple_pcie {
	struct mutex		lock;
	struct device		*dev;
	void __iomem            *base;
	void __iomem		*fabric_base;
	void __iomem		*debug_base;
	void __iomem		*intr2axi_base;
	void __iomem		*oe_fabric_base;
	void __iomem		*early_cfg;
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
	bool			resume_failed; /* cleared only by teardown and reprobe */
	struct reset_control	*reset;
	bool			neo_config_ready;
	u8			neo_secondary_bus;
	struct device		*piodma_supplier;
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
	struct irq_domain	*domain;
	struct list_head	entry;
	unsigned long		*sid_map;
	u32			*saved_rid2sid;
	struct apple_tunable	*tunable;
	unsigned int		irq;
	unsigned int		link_irqs[2];
	u32			saved_intmask;
	int			sid_map_sz;
	int			idx;
	bool			started;
	bool			needs_stop;
	bool			link_failed;
	bool			irq_disabled;
};

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
	msg->address_hi = upper_32_bits(DOORBELL_ADDR);
	msg->address_lo = lower_32_bits(DOORBELL_ADDR);
	msg->data = data->hwirq;
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

	hwirq = bitmap_find_free_region(pcie->bitmap, pcie->nvecs,
					order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);

	if (hwirq < 0)
		return -ENOSPC;

	fwspec.param[fwspec.param_count - 2] += hwirq;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &fwspec);
	if (ret) {
		mutex_lock(&pcie->lock);
		bitmap_release_region(pcie->bitmap, hwirq,
				      order_base_2(nr_irqs));
		mutex_unlock(&pcie->lock);
		return ret;
	}

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &apple_msi_bottom_chip, pcie);
	}

	return 0;
}

static void apple_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_pcie *pcie = domain->host_data;

	mutex_lock(&pcie->lock);

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

	/* FIXME: consider moving each interrupt under each port */
	port->irq = irq_of_parse_and_map(to_of_node(dev_fwnode(port->pcie->dev)),
					 port->idx);
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
				pcie->hw->port_msimap + 4 * i);
	} else {
		apple_pcie_port_writel(port, 0, PORT_MSIBASE);
		val = ilog2(pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT;
	}

	apple_pcie_port_writel(port, val | PORT_MSICFG_EN, PORT_MSICFG);
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

static int apple_pcie_tunnel_release_reset(struct apple_pcie_port *port)
{
	u32 stat;
	int ret;

	/*
	 * Starting the CIO PCIe tunnel leaves the tunneled root port in reset.
	 * Releasing it means writing zero to PORT_TUNCTRL, waiting for the
	 * reset-active indication to clear, and only then enabling the LTSSM.
	 */
	apple_pcie_port_writel(port, 0, PORT_TUNCTRL);
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       !(stat & PORT_TUNSTAT_PERST_ON),
				       1000,
				       port->pcie->kernel_init ? 250000 : 100000,
				       false, port, PORT_TUNSTAT);
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

	for (i = 0; i < ARRAY_SIZE(apple_pcie_tunnel_reset_regs); i++) {
		u32 value = apple_pcie_tunnel_reset_regs[i].value;

		/*
		 * t8103 writes 0. 0x208 is the value used when firmware left the
		 * port up; T600x/T602x cold init without a handoff uses it too.
		 */
		if (port->pcie->kernel_init &&
		    apple_pcie_tunnel_reset_regs[i].offset == 0x130)
			value = 0;
		apple_pcie_port_writel(port, value, apple_pcie_tunnel_reset_regs[i].offset);
	}

	for (i = 0; i < port->pcie->hw->max_rid2sid; i++)
		apple_pcie_rid2sid_write(port, i, 0);
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
 * Bring up a port that no firmware handoff left running (t8103, or
 * T600x/T602x with pcie_apple.tunnel_kernel_init): tunables and the reset
 * table, then PERST and APPCLK, and only then the root-port config-space
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
		 apple_pcie_port_readl(port, PORT_TUNSTAT));
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
				pcie->hw->port_msimap + 4 * i);
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

static void apple_pcie_tunnel_link_up(struct apple_pcie *pcie)
{
	WRITE_ONCE(pcie->tunnel_ready_at,
		   jiffies + msecs_to_jiffies(APPLE_PCIE_TUNNEL_READY_MS));
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
	if (apple_pcie_port_readl(port, PORT_OUTS_PREQS_HDR) &
	    PORT_OUTS_PREQS_HDR_MASK)
		return false;
	if (apple_pcie_port_readl(port, PORT_OUTS_PREQS_DATA) &
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
	apple_pcie_port_rmw_set(port, PORT_TUNCTRL_PERST_ON, PORT_TUNCTRL);
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       stat & PORT_TUNSTAT_PERST_ON,
				       1000, 100000, false, port, PORT_TUNSTAT);
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

	apple_pcie_port_rmw_set(port, PORT_TUNCTRL_PERST_ACK_REQ, PORT_TUNCTRL);
	/* M1 reports an outstanding acknowledgment until ACK_PEND clears. */
	ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
				       pcie->kernel_init ?
				       !(stat & PORT_TUNSTAT_PERST_ACK_PEND) :
				       stat & PORT_TUNSTAT_PERST_ACK_PEND,
				       1000, 1000000, false, port, PORT_TUNSTAT);
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
				  PORT_TUNCTRL);
	WRITE_ONCE(port->started, false);
	port->needs_stop = err || (apple_pcie_port_readl(port, PORT_STATUS) &
				  PORT_STATUS_READY);

	return err;
}

static void apple_pcie_port_teardown(struct apple_pcie_port *port)
{
	apple_pcie_port_disable_irq(port);

	if (port->base)
		apple_pcie_port_writel(port, ~0, PORT_INTMSK);

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

	pwren = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(np), "pwren",
					    GPIOD_ASIS, "PWREN");
	if (IS_ERR(pwren)) {
		if (PTR_ERR(pwren) == -ENOENT)
			pwren = NULL;
		else
			return PTR_ERR(pwren);
	}

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
		 * apple,pciec-kernel-init selects the in-kernel sequence, as
		 * pcie_apple.tunnel_kernel_init does on T600x/T602x.
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
				"PCIe-C requires an m1n1 preinit handoff or pcie_apple.tunnel_kernel_init=1\n");
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

	/* Reset all RID/SID mappings, and check for RAZ/WI registers */
	for (i = 0; i < pcie->hw->max_rid2sid; i++) {
		if (apple_pcie_rid2sid_write(port, i, 0xbad1d) != 0xbad1d)
			break;
		apple_pcie_rid2sid_write(port, i, 0);
	}

	dev_dbg(pcie->dev, "%pOF: %d RID/SID mapping entries\n", np, i);

	port->sid_map_sz = i;

	list_add_tail(&port->entry, &pcie->ports);
	init_completion(&pcie->event);

	ret = apple_pcie_port_register_irqs(port);
	if (ret)
		goto err_teardown;

	link_stat = apple_pcie_port_readl(port, PORT_LINKSTS);
	if (!(link_stat & PORT_LINKSTS_UP)) {
		unsigned long timeout, left = 0;
		int attempt, attempts = 1;

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

		timeout = link_up_timeout * HZ / 1000;
		for (attempt = 0; attempt < attempts; attempt++) {
			if (attempt)
				reinit_completion(&pcie->event);
			apple_pcie_port_writel(port, PORT_LTSSMCTL_START,
					       PORT_LTSSMCTL);
			left = wait_for_completion_timeout(&pcie->event, timeout);
			if (left)
				break;
			dev_warn(pcie->dev, "%pOF link didn't come up\n", np);
		}
		if (left)
			dev_info(pcie->dev, "%pOF link up after %ldms\n", np,
				 (timeout - left) * 1000 / HZ);
	}
	if (pcie->hw->tunneled &&
	    (apple_pcie_port_readl(port, PORT_LINKSTS) & PORT_LINKSTS_UP))
		apple_pcie_tunnel_link_up(pcie);
	if (pcie->hw->tunneled)
		WRITE_ONCE(port->started, true);
	if (pcie->hw->root_bus_only)
		dev_info(pcie->dev, "port %pOF RUN=%#x link=%#x; downstream config %s\n",
			 np, apple_pcie_port_readl(port, PORT_STATUS),
			 apple_pcie_port_readl(port, PORT_LINKSTS),
			 pcie->piodma_supplier ? "waits for the bootstrap" : "blocked");

	return 0;

err_teardown:
	apple_pcie_port_teardown(port);
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
static bool apple_t6030_msi_slice_valid(u32 base, u32 count, u32 aic_irq)
{
	return (base == 0x20 || base == 0x40 || base == 0x60) &&
	       count == 32 && aic_irq == 1342 + base;
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

	/*
	 * The controller-wide ADT count (512) is not a per-host allocation.
	 * Internal PCIe owns vectors 0..31. Each PCIe-C bridge owns a distinct
	 * 32-vector slice. Keep endpoint MSI data local, and route its table
	 * entry to the corresponding shared-window vector.
	 */
	if (pcie->hw == &t8122_pciec_hw) {
		struct resource ecam;
		u32 expected_base;

		/* These ECAM apertures identify the three T6030 ADT bridges. */
		if (!of_machine_is_compatible("apple,t6030") ||
		    of_address_to_resource(to_of_node(fwnode), 0, &ecam))
			goto invalid_slice;
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
		if (args.args_count != 3 || args.args[0] != 0 ||
		    args.args[2] != IRQ_TYPE_EDGE_RISING)
			goto invalid_slice;
		ret = of_property_read_u32(to_of_node(fwnode),
					   "apple,msi-vector-base",
					   &pcie->msi_vector_base);
		if (!of_find_property(to_of_node(fwnode),
				      "apple,msi-vector-base", NULL) &&
		    args.args[1] == 1342 && pcie->nvecs == 512) {
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
					pcie->nvecs, args.args[1]))
			goto invalid_slice;
	}

	/* Keep map targets within the supported 8-bit range. */
	if (pcie->hw == &t8122_pciec_hw &&
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
			     "invalid or missing PCIe-C MSI slice\n");
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
	mutex_lock(&port->pcie->lock);

	if (port->pcie->hw->sid_indexed_rid2sid) {
		if (sid >= port->sid_map_sz)
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
	if (idx >= 0) {
		val = PORT_RID2SID_VALID | (sid << PORT_RID2SID_SID_SHIFT) | rid;
		readback = apple_pcie_rid2sid_write(port, idx, val);
		if (pcie->hw->sid_indexed_rid2sid && readback != val) {
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
	if (!port)
		return;

	mutex_lock(&port->pcie->lock);

	for_each_set_bit(idx, port->sid_map, port->sid_map_sz) {
		u32 val;

		val = apple_pcie_port_readl(port,
					    port_rid2sid_offset(port, idx));
		if ((val & 0xffff) == rid) {
			apple_pcie_rid2sid_write(port, idx, 0);
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
			return ret;
		}
	}

	/* The bus scan follows this callback. */
	apple_pcie_tunnel_wait_ready(pcie);

	return 0;
}

/*
 * The T8140 experiment starts with all downstream accesses blocked.
 * Only a successful owned bootstrap and typed ECAM checks open functions0/1.
 * Generic config callbacks retain their normal native widths and locking.
 */
static void __iomem *apple_pcie_map_bus(struct pci_bus *bus,
				      unsigned int devfn, int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct apple_pcie_port *port;

	if (!pcie->hw->root_bus_only)
		return pci_ecam_map_bus(bus, devfn, where);

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

static int apple_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				   int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct apple_pcie *pcie = apple_pcie_lookup(cfg->parent);
	struct apple_pcie_port *port;
	int ret;

	ret = pci_generic_config_write(bus, devfn, where, size, val);
	if (ret || !pcie || !(pcie->kernel_init || pcie->tunnel_cold_init))
		return ret;
	/* Root-port prefetchable window. The port block has to be told too. */
	if (bus->number != cfg->busr.start || devfn)
		return ret;
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
		.read		= pci_generic_config_read,
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
		pm_runtime_get_noresume(&pdev->dev);
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
			pcie_aspm_remove_cap(pdev, PCI_EXP_LNKCAP_ASPM_L1);
		break;
	case BUS_NOTIFY_DEL_DEVICE:
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
		dev_info(pcie->dev,
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
	pci_info(root, "NEO_ROOT_MEMORY_READBACK status=%d command=%#x window=%#x\n",
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
	pci_info(root, "NEO_ROOT_MEMORY_ENABLED command=%#x window=%#x bus=%#llx-%#llx\n",
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
			dev_info(pcie->dev, "enumerated %s id=%04x:%04x command=%#x\n",
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
		dev_info(pcie->dev, "NEO_RADIO_READY; functions0/1 admitted with link power states disabled\n");
out:
	pci_dev_put(root);
	return ret;
}

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
	if (!hw->tunneled) {
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
	if (hw->root_bus_only && apple_piodma_bootstrap_enabled() &&
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
	pcie->piodma_supplier = piodma_supplier;
	/* Only this retained-DMA radio hierarchy forbids user reset/removal. */
	if (piodma_supplier) {
		bridge->no_user_remove = true;
		bridge->reset_sensitive = true;
		bridge->native_pcie_hotplug = false;
		bridge->native_shpc_hotplug = false;
		bridge->native_aer = false;
		bridge->native_dpc = false;
		dev_info_once(dev, "Neo radio host: native AER/DPC recovery disabled for retained-DMA bootstrap\n");
	}
	if (hw->root_bus_only && !piodma_supplier)
		dev_info(dev, "root-port probe only; downstream config is blocked\n");
	pcie->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);
	if (pcie->hw->tunneled) {
		pcie->kernel_init =
			of_device_is_compatible(dev->of_node, "apple,t8103-pciec") &&
			of_property_read_bool(dev->of_node, "apple,pciec-kernel-init");
		pcie->tunnel_cold_init =
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
	if (pcie->tunnel_cold_init) {
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
			dev_err(dev, "enumeration experiment failed=%d; host/supplier retained, no retry\n",
				ret);
		return 0;
	}

	if (pcie->hw->tunneled)
		pci_walk_bus(bridge->bus, apple_pcie_tunnel_keep_d0, NULL);

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
static const struct of_device_id apple_pcie_of_match[];

int apple_pcie_tunnel_prepare(struct device *dev, struct device_node *tunnel)
{
	struct device_node *np, *port_np = NULL;
	const struct of_device_id *match;
	struct apple_pcie *pcie;
	struct apple_pcie_port *port;
	struct apple_pcie_map config = {}, debug = {}, fabric = {}, portmap = {};
	struct apple_pcie_map oe = {}, intr2axi = {};
	bool kernel_init = false;
	u32 stat;
	int ret;

	np = NULL;
	for_each_available_child_of_node(tunnel, np) {
		kernel_init = of_device_is_compatible(np, "apple,t8103-pciec") &&
			      of_property_read_bool(np, "apple,pciec-kernel-init");
		if (kernel_init || apple_pcie_tunnel_needs_cold_init(np))
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
 * In suspend-to-idle ACIO and the router links stay powered, so an up
 * tunnel can simply be left running. The functions below keep their state
 * and resume like any other PCIe device; nothing has to be re-trained.
 */
static bool apple_pcie_keep_link(struct apple_pcie *pcie)
{
	if (!READ_ONCE(s2idle_keep_link) || !pm_suspend_no_platform())
		return false;
	if (!(pcie->kernel_init || pcie->tunnel_cold_init) ||
	    !pcie->power_retained)
		return false;
	if (pcie->bus_stopped || pcie->resume_failed)
		return false;
	return apple_pcie_tunnel_link_healthy(pcie);
}

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

static int apple_pcie_suspend_noirq(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);

	if (!pcie->hw->tunneled)
		return 0;
	pcie->reset_on_resume = false;
	pcie->link_kept = apple_pcie_keep_link(pcie);
	if (!pcie->link_kept)
		apple_pcie_stop_for_sleep(dev);
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
		apple_pcie_port_writel(port, PORT_TUNCTRL_PERST_ON, PORT_TUNCTRL);
		ret = apple_pcie_tunnel_cold_init(port);
		if (ret)
			return ret;
		ret = read_poll_timeout_atomic(apple_pcie_port_readl, stat,
					       stat & PORT_TUNSTAT_PERST_ON,
					       1000, 100000, false, port, PORT_TUNSTAT);
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

static int apple_pcie_resume_noirq(struct device *dev)
{
	struct apple_pcie *pcie = apple_pcie_lookup(dev);
	struct apple_pcie_port *port;
	int ret;

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

static const struct dev_pm_ops apple_pcie_pm_ops = {
	.suspend_noirq = apple_pcie_suspend_noirq,
	.resume_noirq = apple_pcie_resume_noirq,
};

static const struct of_device_id apple_pcie_of_match[] = {
	{ .compatible = "apple,t8122-pciec",	.data = &t8122_pciec_hw },
	{ .compatible = "apple,t8103-pciec",	.data = &t8103_pciec_hw },
	{ .compatible = "apple,t6000-pciec",	.data = &t8103_pciec_hw },
	{ .compatible = "apple,t6020-pcie",	.data = &t602x_hw },
	{ .compatible = "apple,t8140-pcie",	.data = &t8140_hw },
	{ .compatible = "apple,pcie",		.data = &t8103_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pcie_of_match);

static struct platform_driver apple_pcie_driver = {
	.probe	= apple_pcie_probe,
	.remove	= apple_pcie_remove,
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
