/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Thunderbolt driver - NHI driver
 *
 * Copyright (c) 2014 Andreas Noever <andreas.noever@gmail.com>
 * Copyright (C) 2018, Intel Corporation
 */

#ifndef DSL3510_H_
#define DSL3510_H_

#include <linux/thunderbolt.h>

struct tb_port;

enum nhi_fw_mode {
	NHI_FW_SAFE_MODE,
	NHI_FW_AUTH_MODE,
	NHI_FW_EP_MODE,
	NHI_FW_CM_MODE,
};

enum nhi_mailbox_cmd {
	NHI_MAILBOX_SAVE_DEVS = 0x05,
	NHI_MAILBOX_DISCONNECT_PCIE_PATHS = 0x06,
	NHI_MAILBOX_DRV_UNLOADS = 0x07,
	NHI_MAILBOX_DISCONNECT_PA = 0x10,
	NHI_MAILBOX_DISCONNECT_PB = 0x11,
	NHI_MAILBOX_ALLOW_ALL_DEVS = 0x23,
};

int nhi_mailbox_cmd(struct tb_nhi *nhi, enum nhi_mailbox_cmd cmd, u32 data);
enum nhi_fw_mode nhi_mailbox_mode(struct tb_nhi *nhi);
void nhi_enable_int_throttling(struct tb_nhi *nhi);
void nhi_disable_interrupts(struct tb_nhi *nhi);
void nhi_interrupt_work(struct work_struct *work);
irqreturn_t nhi_msi(int irq, void *data);
irqreturn_t ring_msix(int irq, void *data);
int nhi_probe(struct tb_nhi *nhi);
void nhi_shutdown(struct tb_nhi *nhi);
void nhi_reset_interface(struct tb_nhi *nhi);

extern const struct dev_pm_ops nhi_pm_ops;

/**
 * struct tb_nhi_ring_layout - Layout of the ring registers in the NHI MMIO space
 * @tx_desc_base: Offset of the descriptor registers of the first TX ring
 * @rx_desc_base: Offset of the descriptor registers of the first RX ring
 * @desc_stride: Stride between the descriptor registers of neighboring rings
 * @tx_options_base: Offset of the options registers of the first TX ring
 * @rx_options_base: Offset of the options registers of the first RX ring
 * @options_stride: Stride between the options registers of neighboring rings
 */
struct tb_nhi_ring_layout {
	u32 tx_desc_base;
	u32 rx_desc_base;
	u32 desc_stride;
	u32 tx_options_base;
	u32 rx_options_base;
	u32 options_stride;
};

/**
 * struct tb_nhi_ops - NHI specific optional operations
 * @init: NHI specific initialization
 * @suspend_noirq: NHI specific suspend_noirq hook
 * @resume_noirq: NHI specific resume_noirq hook
 * @runtime_suspend: NHI specific runtime_suspend hook
 * @runtime_resume: NHI specific runtime_resume hook
 * @shutdown: NHI specific shutdown
 * @pre_nvm_auth: hook to run before Thunderbolt 3 NVM authentication
 * @post_nvm_auth: hook to run after Thunderbolt 3 NVM authentication
 * @request_ring_irq: NHI specific interrupt retrieval hook
 * @release_ring_irq: NHI specific interrupt release hook
 * @ring_interrupt_active: NHI specific hook to activate/deactivate the
 *			   interrupt of a single ring. If not set the
 *			   standard USB4 NHI registers are used.
 * @ring_interrupt_mask: NHI specific hook to mask/unmask the interrupt of a
 *			 single ring. If not set the standard USB4 NHI
 *			 registers are used.
 * @ring_configure: NHI specific hook to program the ring options registers
 *		    and enable the ring with the given flags. If not set
 *		    the standard USB4 NHI registers are used.
 * @pci_tunnel_pre_activate: NHI specific hook run after a host PCIe tunnel has
 *			   been allocated but before its paths and adapters are enabled
 * @pci_tunnel_post_activate: NHI specific hook run after a host PCIe tunnel's
 *			    paths and adapters have been enabled, including
 *			    adoption of an enabled tunnel at domain startup
 * @pci_tunnel_deactivate: Release native host state acquired by the
 *			  PCIe post-activation hook
 * @dp_tunnel_pre_activate: NHI specific hook run after DP hops are programmed
 *			    and before VE/AE. USB4 DPTX Discovery belongs here.
 * @dp_tunnel_post_activate: NHI specific hook run after a DP tunnel's
 *			     adapters have VE/AE enabled
 * @dp_tunnel_deactivate: Releases state from an attempted post-activation hook,
 *			 before adapters are disabled or the host router is removed
 * @dp_tunnel_awaits_display: Whether the host has no display pipeline free yet
 *			      for the DP tunnel from host DP IN adapter @in and
 *			      will drive it once one frees. DPRX cannot complete
 *			      meanwhile, so the tunnel is kept instead of being
 *			      torn down at the DPRX timeout. Called with the
 *			      domain lock held.
 * @add_links: NHI specific hook that makes the native ports tunneled
 *	       through this host (PCIe) resume after the NHI, like the device
 *	       links tb_acpi_add_links() creates from firmware tables. Returns
 *	       %true if the host orders all of its tunneled native ports, so
 *	       none is left without a link.
 * @is_present: Whether the device is currently present on the parent bus
 * @link_lost: Called with the domain lock held when the router or host
 *	       connected to a lane adapter of the host router is unplugged.
 *	       Lets the NHI recover a port whose link went down while the
 *	       cable stays attached.
 * @init_interrupts: NHI specific interrupt initialization hook
 * @router_sleep: Enter host-router sleep with the control channel running
 * @router_wake: Start the host router's links again after its controller restarted
 * @reset_interface: Resets the host interface
 */
struct tb_nhi_ops {
	int (*init)(struct tb_nhi *nhi);
	int (*suspend_noirq)(struct tb_nhi *nhi, bool wakeup);
	int (*resume_noirq)(struct tb_nhi *nhi);
	int (*runtime_suspend)(struct tb_nhi *nhi);
	int (*runtime_resume)(struct tb_nhi *nhi);
	void (*shutdown)(struct tb_nhi *nhi);
	void (*pre_nvm_auth)(struct tb_nhi *nhi);
	void (*post_nvm_auth)(struct tb_nhi *nhi);
	int (*request_ring_irq)(struct tb_ring *ring, bool no_suspend);
	void (*release_ring_irq)(struct tb_ring *ring);
	void (*ring_interrupt_active)(struct tb_ring *ring, bool active);
	void (*ring_interrupt_mask)(struct tb_ring *ring, bool mask);
	void (*ring_configure)(struct tb_ring *ring, u32 flags, u32 e2e_flags);
	int (*pci_tunnel_pre_activate)(struct tb_nhi *nhi);
	int (*pci_tunnel_post_activate)(struct tb_nhi *nhi);
	int (*pci_tunnel_deactivate)(struct tb_nhi *nhi);
	int (*dp_tunnel_pre_activate)(struct tb_nhi *nhi, struct tb_port *in,
				      struct tb_port *out);
	int (*dp_tunnel_post_activate)(struct tb_nhi *nhi, struct tb_port *in,
				       struct tb_port *out, u64 generation);
	void (*dp_tunnel_deactivate)(struct tb_nhi *nhi, struct tb_port *in,
				     struct tb_port *out, u64 generation);
	bool (*add_links)(struct tb_nhi *nhi);
	bool (*is_present)(struct tb_nhi *nhi);
	void (*link_lost)(struct tb_nhi *nhi);
	void (*dp_tunnel_changed)(struct tb_nhi *nhi, u8 in_port, u64 generation, bool active);
	bool (*dp_tunnel_awaits_display)(struct tb_nhi *nhi, struct tb_port *in);
	int (*init_interrupts)(struct tb_nhi *nhi);
	void (*reset_interface)(struct tb_nhi *nhi);
	int (*router_sleep)(struct tb_nhi *nhi);
	int (*router_wake)(struct tb_nhi *nhi);
};

/*
 * PCI IDs used in this driver from Win Ridge forward. There is no
 * need for the PCI quirk anymore as we will use ICM also on Apple
 * hardware.
 */
#define PCI_DEVICE_ID_INTEL_MAPLE_RIDGE_2C_NHI		0x1134
#define PCI_DEVICE_ID_INTEL_MAPLE_RIDGE_4C_NHI		0x1137
#define PCI_DEVICE_ID_INTEL_WIN_RIDGE_2C_NHI            0x157d
#define PCI_DEVICE_ID_INTEL_WIN_RIDGE_2C_BRIDGE         0x157e
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_NHI		0x15bf
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_BRIDGE	0x15c0
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_NHI	0x15d2
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_BRIDGE	0x15d3
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_NHI	0x15d9
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_BRIDGE	0x15da
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_USBONLY_NHI	0x15dc
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_USBONLY_NHI	0x15dd
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_USBONLY_NHI	0x15de
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_BRIDGE	0x15e7
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_NHI		0x15e8
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_BRIDGE	0x15ea
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_NHI		0x15eb
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_DD_BRIDGE	0x15ef
#define PCI_DEVICE_ID_INTEL_ADL_NHI0			0x463e
#define PCI_DEVICE_ID_INTEL_ADL_NHI1			0x466d
#define PCI_DEVICE_ID_INTEL_WCL_NHI0			0x4d33
#define PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HOST_80G_NHI	0x5781
#define PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HOST_40G_NHI	0x5784
#define PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HUB_80G_BRIDGE 0x5786
#define PCI_DEVICE_ID_INTEL_BARLOW_RIDGE_HUB_40G_BRIDGE 0x57a4
#define PCI_DEVICE_ID_INTEL_MTL_M_NHI0			0x7eb2
#define PCI_DEVICE_ID_INTEL_MTL_P_NHI0			0x7ec2
#define PCI_DEVICE_ID_INTEL_MTL_P_NHI1			0x7ec3
#define PCI_DEVICE_ID_INTEL_ICL_NHI1			0x8a0d
#define PCI_DEVICE_ID_INTEL_ICL_NHI0			0x8a17
#define PCI_DEVICE_ID_INTEL_TGL_NHI0			0x9a1b
#define PCI_DEVICE_ID_INTEL_TGL_NHI1			0x9a1d
#define PCI_DEVICE_ID_INTEL_TGL_H_NHI0			0x9a1f
#define PCI_DEVICE_ID_INTEL_TGL_H_NHI1			0x9a21
#define PCI_DEVICE_ID_INTEL_RPL_NHI0			0xa73e
#define PCI_DEVICE_ID_INTEL_RPL_NHI1			0xa76d
#define PCI_DEVICE_ID_INTEL_LNL_NHI0			0xa833
#define PCI_DEVICE_ID_INTEL_LNL_NHI1			0xa834
#define PCI_DEVICE_ID_INTEL_PTL_M_NHI0			0xe333
#define PCI_DEVICE_ID_INTEL_PTL_M_NHI1			0xe334
#define PCI_DEVICE_ID_INTEL_PTL_P_NHI0			0xe433
#define PCI_DEVICE_ID_INTEL_PTL_P_NHI1			0xe434

#define PCI_DEVICE_ID_AMD_1AH_M60H_NHI0			0x1120
#define PCI_DEVICE_ID_AMD_1AH_M60H_NHI1			0x1121
#define PCI_DEVICE_ID_AMD_1AH_M68H_NHI0			0x113b
#define PCI_DEVICE_ID_AMD_1AH_M68H_NHI1			0x113c
#define PCI_DEVICE_ID_AMD_1AH_M80H_NHI0			0x1155
#define PCI_DEVICE_ID_AMD_1AH_M80H_NHI1			0x1158
#define PCI_DEVICE_ID_AMD_1AH_M80H_NHI2			0x1159
#define PCI_DEVICE_ID_AMD_1AH_M24H_NHI0			0x151c
#define PCI_DEVICE_ID_AMD_1AH_M24H_NHI1			0x151d
#define PCI_DEVICE_ID_AMD_1AH_M70H_NHI0			0x158d
#define PCI_DEVICE_ID_AMD_1AH_M70H_NHI1			0x158e

#define PCI_CLASS_SERIAL_USB_USB4			0x0c0340

/* Host interface quirks */
#define QUIRK_AUTO_CLEAR_INT				BIT(0)
#define QUIRK_E2E					BIT(1)
#define QUIRK_RESET_DMA_ON_TEARDOWN			BIT(2)
#define QUIRK_NO_DMA_PORT				BIT(3)
#define QUIRK_NO_USB3_BW_ALLOC				BIT(4)
#define QUIRK_HOST_DP_NFC_CREDITS			BIT(5)
/*
 * The host router stays powered and its links stay up across system sleep,
 * so routers must not be asked to enter sleep: a TBT3 device router that
 * was told to sleep drops its link on its own some tens of seconds later.
 */
#define QUIRK_NO_SYSTEM_SLEEP				BIT(6)
/*
 * Set for one system sleep when the routers stayed awake and the tunneled
 * PCIe host kept its link up: the tunnels are still active on resume.
 */
#define QUIRK_KEEP_TUNNELS				BIT(7)
/* The links of the host router must not enter CL states. */
#define QUIRK_NO_HOST_CLX				BIT(8)
/* The links of the domain must stay symmetric. */
#define QUIRK_NO_ASYM					BIT(9)
/* Gen4 links of the domain run with link recovery enabled on both ends. */
#define QUIRK_GEN4_LINK_RECOVERY			BIT(10)
/*
 * The host router's firmware runs its TMU, and enhanced uni-directional
 * mode takes the host's own constants, see tmu.c.
 */
#define QUIRK_HOST_TMU					BIT(11)
/* The host router has no vendor operation that saves its state for a sleep in place. */
#define QUIRK_CIO_NO_ROUTER_STATE			BIT(12)
/*
 * After a sleep in place the host router's links are waited for only as long as a cold
 * start's training takes, twice over, and a peer that is not back by then is retired as
 * unplugged; a saved state the router does not take back leaves it as from a cold start.
 */
#define QUIRK_CIO_WAKE_GIVES_UP				BIT(13)
/*
 * TEST: the sleep in place exchanges the control frames macOS sends to this router (run
 * mxt6): the PCIe and USB3 paths are left as they are for the sleep and written again
 * after it, a router below the host sleeps with every wake of its lane adapters and its
 * PCIe and USB3 wakes enabled, the host's own wake and sleep words are written by its NHI,
 * and routers are enumerated with a 10 ms notification timeout.
 */
#define QUIRK_CIO_MACOS_FRAMES				BIT(14)
/* Reinitialize device-router TMUs in bi-directional HiFi mode. */
#define QUIRK_TMU_HIFI_BI				BIT(15)

/* Host display policies are independent of which optional hooks are installed. */
#define TB_HOST_DP_NOTIFY		BIT(0)
#define TB_HOST_DP_HPD_ON_ACTIVATE	BIT(1)
#define TB_HOST_DP_ACTIVE_BEFORE_DPRX	BIT(2)
#define TB_HOST_DP_KEEP_DPRX_TIMEOUT	BIT(3)
#define TB_HOST_DP_ADAPTER_QUIRKS		BIT(4)
#define TB_HOST_DP_INITIAL_BW_GRANT	BIT(5)
/*
 * Both DP IN streams of a host PHY must run at one link rate. A DP tunnel
 * alone on the host router offers up to HBR3; when more are needed, all of
 * them offer HBR2 at most (two 4K60 streams fit), a tunnel up above that
 * being set up again at HBR2 first.
 */
#define TB_HOST_DP_SHARED_RATE		BIT(6)
/*
 * Run the host DP IN adapters' bandwidth allocation mode as the host's own
 * OS does: 1 Gb/s granularity, and the connection manager acknowledges a
 * DPTX bandwidth request by holding CM Ack until the adapter has cleared
 * the request.
 */
#define TB_HOST_DP_BW_HELD_ACK		BIT(7)

/*
 * Minimal number of vectors when we use MSI-X. Two for control channel
 * Rx/Tx and the rest four are for cross domain DMA paths.
 */
#define MSIX_MIN_VECS		6
#define MSIX_MAX_VECS		16

#endif
