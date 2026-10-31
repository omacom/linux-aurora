// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon USB4 and Thunderbolt driver
 * Copyright (c) Sven Peter <sven@kernel.org>
 *
 * This driver implements the Host Router / ACIO coprocessor as well as the
 * Native Host Interface (NHI) as shown in the diagram below.
 * The ACIO is a Cortex-M3 coprocessor which handles the Thunderbolt
 * protocol and exposes its own hardware blocks like the USB4 Native Host
 * Interface (NHI) and its IOMMU to the main SoC bus. The entire block
 * can only be brought up after the unified Type-C PHY has been initialized
 * to Thunderbolt or USB4 mode and needs an out-of-band notification from
 * the Type-C PD driver.
 *
 * +--------------------+
 * |                    |  +--------------------------------------------+
 * | Display Controller |  | Host Router / ACIO                         |
 * |       dcpext0      |  |             +----------------+             |
 * |                    |  |             |     Native     |             |
 * +--------+-----------+  |             |      Host      |             |
 *          |              |             |    Interface   |             |
 *          |              +---------+   +----------------+   +---------+     +--------+
 *          |   +--------->| DP IN   |                        | PCIE DN |<--->| APCIEC |
 *          v   |          | Adapter |   +----------------+   | Adapter |     +--------+
 *    +---------+-+        +---------+   |                |   +---------+
 *    | Display   |        |             |  IOMMU / DART  |             |
 *    | Crossbar  |        |             |                |   +---------+     +------+
 *    +---------+-+        +---------+   +----------------+   | USB3    |<--->| DWC3 |
 *          ^   |          | DP IN   |                        | Adapter |     +------+
 *          |   +--------->| Adapter |                        +---------+
 *          |              +---------+                                  |
 *          |              |                                            |
 *          |              |                 +----------+               |
 * +--------+-----------+  |                 | Type C   |               |
 * |                    |  |                 | Adapter  |               |
 * | Display Controller |  +-----------------+----------+---------------+
 * |       dcpext1      |                     ^       ^
 * |                    |                     |       |
 * +--------------------+                     |       |
 *                                            v       |
 *                                 +--------------+   | SBRX/TX
 *                                 | Apple Type-C |   |
 *                                 |    PHY       |   |
 *                                 +--------------+   |
 *                                       ^            |
 *                                       |            v
 *                                       |     +---------+
 *                                       +---->| Type C  |
 *                                     SSRX/TX | Port    |
 *                                             +---------+
 *
 * ACIO and ATCPHY have strict ordering requirements. When a USB4/Thunderbolt
 * connection is requested after a device has been connected the Type-C PD
 * driver performs the following sequence:
 *
 * 1) Configure the PHY for the negotiated mode and orientation through
 *    typec_mux_set()
 * 2) Pass the cable details to this driver through typec_thunderbolt_switch_set()
 * 3) Power up ACIO, boot its RTKit co-processor and apply the tunables
 * 4) Probe the DART and NHI children now that their MMIO is accessible from
 *    the main SoC bus
 * 5) Register the USB4 domain and write the cable details to the host router
 *    which brings up the link and starts discovery
 *
 * The reverse order is required when the cable is disconnected then:
 *
 * 1) typec_thunderbolt_switch_set() removes the USB4 domain and the ACIO
 *    children, shuts down the co-processor and powers everything off.
 * 2) typec_mux_set() puts the PHY into safe mode and powers it off.
 *
 * ACIO must not be started before the PHY is configured and the PHY must
 * remain active until ACIO has been powered down again. The child devices
 * cannot  remain populated while ACIO is powered off because they are part
 * of the ACIO block and only exposed to the main SoC bus and will SError
 * once ACIO is off.
 * Violating this ordering can panic the kernel with an async SError at best
 * and trigger some internal watchdog that will reset the entire SoC at worst.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/notifier.h>
#include <linux/pci-apple.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/tunable.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb/pd.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>
#include <linux/workqueue.h>

#include "apple-dpin-handshake.h"
#include "apple-dpin-state.h"
#include "apple-dpin-request.h"
#include "apple-dpin-platform.h"
#include "nhi.h"
#include "tb.h"

#define APPLE_CIO_M3_CTRL 0x0c
#define APPLE_CIO_M3_CTRL_START BIT(1)
#define APPLE_CIO_LSTX_CTRL_LEVEL BIT(0)
#define APPLE_CIO_LSTX_CTRL_ENABLE BIT(1)
#define APPLE_CIO_M3_STAT 0xa8
#define APPLE_CIO_M3_STAT_STATE GENMASK(30, 24)
#define APPLE_CIO_M3_STAT_STATE_READY 1
#define APPLE_CIO_M3_STAT_CODE_VALID BIT(31)
#define APPLE_CIO_M3_STAT_CODE GENMASK(23, 0)

/* MxWrap coprocessor wrapper (t6030 "cpu" region) */
#define APPLE_MXWRAP_CPU_CTRL 0x28
#define APPLE_MXWRAP_CPU_CTRL_RUN BIT(4)
#define APPLE_MXWRAP_CPU_STATUS 0x44

struct apple_cio_hw {
	bool mxwrap;
	bool lstx_explicit;
	bool strict_fw_ready;
	/* M3_STAT never becomes ready; a poll timeout is not fatal. */
	bool m3_stat_optional;
	const struct apple_dpin_policy *dp;
};

static const struct apple_cio_hw apple_cio_t8103_hw = {
	.dp = &apple_dpin_m1,
};

static const struct apple_cio_hw apple_cio_t6020_hw = {
	.dp = &apple_dpin_m2,
};

static const struct apple_cio_hw apple_cio_t6030_hw = {
	.mxwrap = true,
	.lstx_explicit = true,
	.strict_fw_ready = true,
	.dp = &apple_dpin_m3,
};

/* Type 7 uses its existing ready-state check and has no qualified DP route. */
static const struct apple_cio_hw apple_cio_t8142_hw = {
	.dp = &apple_dpin_disabled,
};

/*
 * T8152 starts the ACIO CPU through the same coprocessor wrapper block as
 * t6030 (the "cpu" region: run control +0x28, idle status +0x44), and its
 * rc M3_CTRL start bit is not decoded, so the wrapper drives the start.
 * M3_STAT never becomes ready; the RTKit boot of the co-processor is the
 * readiness check.
 */
static const struct apple_cio_hw apple_cio_t8152_hw = {
	.mxwrap = true,
	.m3_stat_optional = true,
	.dp = &apple_dpin_disabled,
};

#define APPLE_CIO_NHI_HOP_COUNT 0x0
#define APPLE_CIO_NHI_HOP_COUNT_MASK GENMASK(9, 0)

#define APPLE_CIO_NHI_TXRING_DESC_BASE 0x10000
#define APPLE_CIO_NHI_RXRING_DESC_BASE 0x80000
#define APPLE_CIO_NHI_RING_STRIDE 0x4000

#define APPLE_CIO_NHI_PDF_STRIDE 0x4000

#define APPLE_CIO_NHI_IRQ_STATUS 0xd0000
#define APPLE_CIO_NHI_IRQ_ENABLE 0xd0010
#define APPLE_CIO_NHI_IRQ_THROTTLE 0xd004c
#define APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK GENMASK(15, 0)
#define APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC 256

#define APPLE_CIO_SRAM_IOVA_BASE 0x10000000

#define APPLE_CIO_NHI_BOOT_TIMEOUT_MS 10000
#define APPLE_CIO_PCIEC_READY_DELAY_MS 300

/* Observation only: no extra control requests or downstream enumeration. */
#define APPLE_CIO_RESUME_LINK_WAIT_MS 90000
#define APPLE_CIO_RESUME_LINK_CHECK_MS 5000
#define APPLE_CIO_SLEEP_RESTART_FALLBACK_MS 10000
/* How long a forced cable cycle may take before it no longer counts. */
#define APPLE_CIO_RESUME_RECOVERY_WINDOW_MS 30000

static bool resume_recovery;
module_param(resume_recovery, bool, 0644);
MODULE_PARM_DESC(resume_recovery,
		 "Recover missing post-resume discovery once per driver load (default N)");
/* Shared by all ACIO ports; never refilled by suspend or cable changes. */
static atomic_t resume_recovery_used = ATOMIC_INIT(0);

/*
 * A connected port keeps its link trained through s2idle, which costs about
 * 1.3 W on t6000. Powering ACIO off for the sleep saves that, but drops every
 * tunnel: devices behind them are removed and rediscovered after resume.
 */
static bool s2idle_teardown;
module_param(s2idle_teardown, bool, 0644);
MODULE_PARM_DESC(s2idle_teardown,
		 "Power off connected ports for system sleep and restart them on resume (default N; drops tunnels, including USB devices tunnelled through a USB4 dock: unmount their storage first; ports with PCIe tunnels are skipped)");

#define APPLE_CIO_START_RETRIES 5
#define APPLE_CIO_START_RETRY_DELAY_MS 100

#define APPLE_CIO_PCIEC_INTR2AXI_CTRL 0x80
#define APPLE_CIO_PCIEC_INTR2AXI_ENABLE BIT(0)

/*
 * The Apple vendor-specific extended capability contains a cable-information
 * word which has to be programmed from the USB4/Thunderbolt details reported
 * out-of-band by the Type-C PD controller. It must be programmed into the
 * host router before link discovery starts.
 */
#define TB_VSE_CAP_APPLE 0x00
#define TB_VSE_CAP_APPLE_CABLE_INFO 0x01
#define TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT BIT(0)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE BIT(1)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE BIT(2)
#define TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX BIT(3)
#define TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS BIT(4)
#define TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER BIT(9)
#define TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3 BIT(10)

static unsigned int dpin_mode_value = 8;
module_param(dpin_mode_value, uint, 0644);
MODULE_PARM_DESC(dpin_mode_value,
		 "DPIN0 MODE_A/MODE_B lab value (0-" __stringify(APPLE_DPIN_MODE_VALUE_MAX) "); read at each activate");

struct apple_dpin_binding;

/**
 * struct apple_cio - Apple Converged I/O host router complex
 * @dev: ACIO device
 * @np: ACIO device tree node
 * @rtk: RTKit instance for the ACIO co-processor
 * @rc_base: ACIO root controller registers
 * @rc_res: ACIO root controller MMIO resource
 * @rc_tunable: Tunable sequence for the ACIO root controller
 * @sram_res: ACIO co-processor SRAM resource
 * @sram_base: ACIO co-processor SRAM
 * @reset: ACIO reset controller
 * @pd_list: Power domains used by the ACIO complex
 * @lock: Serializes cable transitions and ACIO power changes
 * @current_cable_info: Cable information currently programmed into ACIO
 * @target_cable_info: Cable information requested by the Type-C PD driver
 * @nhi_boot_completion: Signals completion of the NHI bringup
 * @nhi_boot_status: Status returned by the NHI bringup
 * @tbt_switch: USB4/Thunderbolt switch triggered by the Type-C PD driver
 */
struct apple_cio {
	struct device *dev;
	bool type7;
	struct device_node *np;
	const struct apple_cio_hw *hw;
	const struct apple_dpin_policy *dp;
	void __iomem *cpu_base;
	struct device_node *pcie_tunnel_np;
	bool pcie_tunnel_preinitialized;
	struct reset_control *pcie_reset;
	void __iomem *pcie_intr2axi_base;
	struct apple_rtkit *rtk;

	void __iomem *rc_base;
	struct resource *rc_res;
	struct apple_tunable *rc_tunable;
	void __iomem *pcie_adapter_base;
	struct apple_tunable *pcie_adapter_tunable;

	struct resource *sram_res;
	void __iomem *sram_base;

	struct reset_control *reset;

	struct dev_pm_domain_list *pd_list;

	struct mutex lock; /* serializes cable transitions and ACIO power up/down */
	bool removing; /* protected by lock */
	struct delayed_work resume_link_work;
	unsigned int resume_link_failures; /* protected by lock */
	u64 resume_activity; /* snapshot before the stock resume path */
	bool resume_had_peer; /* peer present before suspend */
	bool resume_recovery_active; /* forced cable cycle is in progress */
	unsigned long resume_recovery_until; /* jiffies: the cycle is over after */
	bool resume_recovery_failed; /* terminal recovery failure */
	bool resume_link_failed; /* protected by lock */
	bool resume_link_pending; /* set by NHI resume, consumed by ACIO complete */
	struct notifier_block pm_notifier;
	struct delayed_work sleep_restart_work;
	enum {
		APPLE_CIO_SLEEP_OPEN,		/* cable changes apply normally */
		APPLE_CIO_SLEEP_CLOSED,		/* torn down for sleep: changes recorded only */
		APPLE_CIO_SLEEP_RESUMED,	/* next cable state is a post-resume snapshot */
	} sleep_phase; /* protected by lock */
	bool sleep_restart_owed; /* protected by lock */
	bool sleep_aborted_early; /* protected by lock: devices never suspended */

	u32 current_cable_info;
	u32 target_cable_info;
	struct completion nhi_boot_completion;
	int nhi_boot_status;
	struct platform_device *nhi_pdev;

	struct typec_thunderbolt_switch_dev *tbt_switch;
	struct notifier_block pcie_notifier;
	/* Serializes PCIe-C population with cable teardown. */
	struct mutex pcie_tunnel_lock;
	struct delayed_work pcie_tunnel_work;
	bool pcie_tunnel_requested;
	bool pcie_tunnel_populated;
	unsigned int pcie_bind_retries; /* protected by pcie_tunnel_lock */
	bool pcie_tunnel_stopping;
	bool pcie_pm_prepared;
	bool pcie_resume_expected;
	bool pcie_quiesce_pending;

	/* Type-C connector this router is wired to, for DP tunnel routing */
	struct device_node *connector_np;
	bool dp_dual_stream;
	struct workqueue_struct *dp_wq;
	struct apple_dpin_ctx {
		struct apple_cio *acio;
		struct work_struct work;
		struct delayed_work retry;	/* re-queues work while waiting */
		struct mutex lock;	/* protects regs, alive, waiting, paused */
		void __iomem *regs;	/* mapped while a DP tunnel uses this adapter */
		unsigned int idx;
		struct apple_dpin_state state;
		struct apple_dpin_tokens tokens;
		struct apple_dpin_binding *binding; /* worker-owned, handed to DCP */
	} dpin[2];
};

/* Immutable identity for one callback handoff; freed only after provider drain. */
struct apple_dpin_binding {
	struct apple_dpin_ctx *ctx;
	u64 generation;
	typeof(apple_dcp_tb_dp_tunnel) *tunnel; /* owns the provider module reference */
};

/*
 * The ATC's DP IN adapter blocks ("atcN-dpin0/1" in the Apple device tree).
 * Registers on t8103:
 *   0x00 status, bit 2 = HPD level
 *   0x04 interrupt status (write 1 to clear)
 *   0x08 interrupt enable [1:0]
 *   0x0c bit 0 DPTX_INACTIVE
 *   0x10 bit 0 DPTX_INACTIVE_ACK
 * The block sits 0x390000 after the ACIO "rc" region; dpin1 is 0x8000 after dpin0.
 */
#define APPLE_DPIN_OFFSET		0x390000
#define APPLE_DPIN_STRIDE		0x8000
#define APPLE_DPIN_SIZE			0x4000
#define APPLE_DPIN_STATUS		0x00
#define APPLE_DPIN_STATUS_HPD		BIT(2)
#define APPLE_DPIN_IRQ_STATUS		0x04
#define APPLE_DPIN_IRQ_ENABLE		0x08
#define APPLE_DPIN_CTRL			0x0c
#define APPLE_DPIN_CTRL_INACTIVE	BIT(0)
#define APPLE_DPIN_ACK_INACTIVE		BIT(0)

static bool dp_display = true;
module_param(dp_display, bool, 0444);
MODULE_PARM_DESC(dp_display, "Drive displays behind qualified Thunderbolt DP tunnels (default: true)");

/*
 * The M1 Pro/Max ATC is the t8103 generation: same DP IN adapter registers,
 * same crossbar and the same tunnel pixel clock sequence.
 */
static const struct of_device_id apple_dpin_qualified_soc[] = {
	{ .compatible = "apple,t8103" },
	/* The M2 (t8112) ACIO, NHI and display crossbar are t8103-compatible. */
	{ .compatible = "apple,t8112" },
	{ .compatible = "apple,t6000" },
	{ .compatible = "apple,t6001" },
	{ .compatible = "apple,t6020" },
	{ .compatible = "apple,t6021" },
	{ .compatible = "apple,t6030" },
	{},
};

const struct apple_dpin_policy *
apple_dpin_policy_select(const struct apple_dpin_policy *hw,
			 const struct device_node *root, bool dual_stream)
{
	if (!of_match_node(apple_dpin_qualified_soc, root))
		return &apple_dpin_disabled;
	if (hw->flow == APPLE_DPIN_PRE_POST && !dual_stream)
		return &apple_dpin_disabled;
	return hw;
}

/* DPTX_INACTIVE handshake: request (in)active, wait for the ACK */
struct apple_dpin_poll {
	void __iomem *base;
	unsigned long deadline;
};

static unsigned int apple_dpin_read(void *ctx, unsigned int offset)
{
	struct apple_dpin_poll *poll = ctx;

	return readl(poll->base + offset);
}

static void apple_dpin_write(void *ctx, unsigned int offset, unsigned int value)
{
	struct apple_dpin_poll *poll = ctx;

	writel(value, poll->base + offset);
}

static int apple_dpin_wait(void *ctx)
{
	struct apple_dpin_poll *poll = ctx;

	if (time_after_eq(jiffies, poll->deadline))
		return -ETIMEDOUT;
	usleep_range(1000, 2000);
	return 0;
}

static int apple_dpin_set_active_t602x(struct apple_cio *acio, void __iomem *regs,
				 unsigned int idx, bool active)
{
	struct apple_dpin_poll poll = {
		.base = regs,
		.deadline = jiffies + msecs_to_jiffies(1000),
	};
	struct apple_dpin_io io = {
		.read = apple_dpin_read,
		.write = apple_dpin_write,
		.wait = apple_dpin_wait,
		.ctx = &poll,
	};
	int ret;

	ret = apple_dpin_handshake(&io, active, dpin_mode_value);
	dev_info(acio->dev, "dpin%u: %s handshake=%d\n", idx,
		 active ? "active" : "inactive", ret);
	return ret;
}

static int apple_dpin_set_active_t8103(struct apple_cio *acio, void __iomem *dpin,
				 unsigned int idx, bool active)
{
	u32 want = active ? 0 : APPLE_DPIN_ACK_INACTIVE;
	unsigned long deadline = jiffies + msecs_to_jiffies(500);
	u32 st = 0, ctrl, ack;

	/* one 500 ms budget for the whole handshake: DCP waits on this */
	if (active) {
		while (!((st = readl(dpin + APPLE_DPIN_STATUS)) & APPLE_DPIN_STATUS_HPD)) {
			if (time_after(jiffies, deadline)) {
				dev_warn(acio->dev, "dpin%u: no HPD (status=%08x)\n",
					 idx, st);
				return -ETIMEDOUT;
			}
			usleep_range(1000, 1200);
		}
	}

	ctrl = readl(dpin + APPLE_DPIN_CTRL);
	ack = readl(dpin + APPLE_DPIN_ACK);
	if (!!(ctrl & APPLE_DPIN_CTRL_INACTIVE) == !active &&
	    (ack & APPLE_DPIN_ACK_INACTIVE) == want)
		return 0;

	if (active)
		ctrl &= ~APPLE_DPIN_CTRL_INACTIVE;
	else
		ctrl |= APPLE_DPIN_CTRL_INACTIVE;
	writel(ctrl, dpin + APPLE_DPIN_CTRL);

	while (((ack = readl(dpin + APPLE_DPIN_ACK)) & APPLE_DPIN_ACK_INACTIVE) != want) {
		if (time_after(jiffies, deadline)) {
			dev_warn(acio->dev, "dpin%u: no DPTX_INACTIVE_ACK=%u (ack=%08x)\n",
				 idx, !active, ack);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1200);
	}
	dev_dbg(acio->dev, "dpin%u: %s (status=%08x)\n", idx,
		 active ? "active" : "inactive", readl(dpin + APPLE_DPIN_STATUS));
	return 0;
}

static int apple_dpin_set_active(struct apple_cio *acio, void __iomem *regs,
				 unsigned int idx, bool active)
{
	if (acio->dp->t602x_handshake)
		return apple_dpin_set_active_t602x(acio, regs, idx, active);
	return apple_dpin_set_active_t8103(acio, regs, idx, active);
}

static void apple_dpin_request_inactive(void *data)
{
	struct apple_dpin_ctx *c = data;

	apple_dpin_set_active(c->acio, c->regs, c->idx, false);
}

static void apple_dpin_request_mask_irqs(void *data)
{
	struct apple_dpin_ctx *c = data;

	writel(readl(c->regs + APPLE_DPIN_IRQ_ENABLE) & ~3,
	       c->regs + APPLE_DPIN_IRQ_ENABLE);
}

static bool apple_dpin_request(struct apple_dpin_ctx *c, u64 generation, bool active)
{
	const struct apple_dpin_request_ops ops = {
		.inactive = apple_dpin_request_inactive,
		.mask_irqs = apple_dpin_request_mask_irqs,
		.ctx = c,
	};

	lockdep_assert_held(&c->lock);
	return apple_dpin_powered_request(&c->state, &c->tokens, c->acio->dp,
					  generation, active, !!c->regs, &ops);
}

/* Called by appledrm from DCP's Activate/Deactivate calls. */
static int apple_dpin_dcp_set_active(void *data, bool active)
{
	const struct apple_dpin_binding *binding = data;
	struct apple_dpin_ctx *c = binding->ctx;

	guard(mutex)(&c->lock);
	if (!apple_dpin_token_access(&c->tokens, binding->generation))
		return -ESTALE;
	/* the tunnel is being torn down: the block may already be off */
	if (!c->state.alive || !c->regs)
		return -ENODEV;
	return apple_dpin_set_active(c->acio, c->regs, c->idx, active);
}

static bool apple_dpin_provider_get(void *data)
{
	struct apple_dpin_binding *binding = data;

	binding->tunnel = symbol_get(apple_dcp_tb_dp_tunnel);
	return !!binding->tunnel;
}

static bool apple_dpin_provider_held(void *data)
{
	struct apple_dpin_binding *binding = data;

	return !!binding->tunnel;
}

static int apple_dpin_provider_invoke(void *data, bool active)
{
	struct apple_dpin_binding *binding = data;
	struct apple_dpin_ctx *c = binding->ctx;

	return binding->tunnel(c->acio->connector_np, c->idx, binding->generation, active,
			       active ? apple_dpin_dcp_set_active : NULL,
			       active ? binding : NULL);
}

static void apple_dpin_provider_put(void *data)
{
	struct apple_dpin_binding *binding = data;

	symbol_put(apple_dcp_tb_dp_tunnel);
	binding->tunnel = NULL;
}

static int apple_dpin_connect(struct apple_dpin_ctx *c,
			      struct apple_dpin_binding *binding,
			      bool active)
{
	struct apple_cio *acio = c->acio;
	const struct apple_dpin_provider_ops ops = {
		.get = apple_dpin_provider_get,
		.held = apple_dpin_provider_held,
		.call = apple_dpin_provider_invoke,
		.put = apple_dpin_provider_put,
		.ctx = binding,
	};
	unsigned int tries;
	bool alive, paused;
	int ret;

	for (tries = 1; ; tries++) {
		/*
		 * up() admitted the first call together with mapping and IRQ setup.
		 * Prepare drains that call; every subsequent attempt needs a new gate.
		 */
		if (active) {
			guard(mutex)(&c->lock);
			if (!c->state.alive || c->tokens.requested != binding->generation)
				return -ESTALE;
			if (apple_dpin_admission_blocked(&c->state, acio->dp, tries == 1))
				return -EAGAIN;
			/* up() admitted the first call; retry admits a fresh attempt. */
			if (tries == 1) {
				if (!apple_dpin_token_access(&c->tokens, binding->generation))
					return -ESTALE;
			} else if (!apple_dpin_token_admit(&c->tokens, binding->generation)) {
				return -ESTALE;
			}
		}
		ret = apple_dpin_provider_call(active, &ops);
		if (active && ret) {
			scoped_guard(mutex, &c->lock)
				apple_dpin_token_revoke(&c->tokens, binding->generation);
		}
		/* ENODEV is driver probing; EAGAIN is a registered external
		 * route whose firmware services have not been published yet.
		 * Neither has handed a callback or changed the display route.
		 * Other failures are terminal for this tunnel event.
		 */
		if (!apple_dpin_readiness_retry(active, ret, tries))
			return ret;

		scoped_guard(mutex, &c->lock) {
			alive = c->state.alive;
			paused = c->state.paused;
		}
		if (!alive)
			return -ENODEV;
		/* not across system sleep: ask again once it is over */
		if (paused)
			return -EAGAIN;
		if (tries == 1)
			dev_info(acio->dev, "dpin%u: waiting for display driver/firmware readiness\n",
				 c->idx);
		msleep(APPLE_DP_CONNECT_WAIT_MS);
	}
}

static int apple_dpin_up(struct apple_dpin_ctx *c,
			 struct apple_dpin_binding *binding, bool *deferred)
{
	struct apple_cio *acio = c->acio;
	void __iomem *regs = c->regs;
	u32 irq;

	*deferred = false;
	scoped_guard(mutex, &c->lock) {
		if (!c->state.alive || c->tokens.requested != binding->generation)
			return -ESTALE;
		if (apple_dpin_admission_blocked(&c->state, acio->dp, false)) {
			*deferred = true;
			return -EAGAIN;
		}
		/* This admission also owns the first DCP call after dropping the lock. */
		if (!regs) {
			regs = ioremap_np(acio->rc_res->start + APPLE_DPIN_OFFSET +
					  c->idx * APPLE_DPIN_STRIDE, APPLE_DPIN_SIZE);
			if (!regs)
				return -ENOMEM;
			c->regs = regs;
		}
		/*
		 * Acknowledge what is pending and enable the DP IN interrupts:
		 * +0x4 is write-1-to-clear, and a plug event (bit 0) also
		 * latches status bits in +0x0 that are cleared by writing it
		 * back. Nothing handles these interrupts yet; they are enabled
		 * as part of bringing the adapter up.
		 */
		if (acio->dp->setup_irqs) {
			irq = readl(regs + APPLE_DPIN_IRQ_STATUS);
			writel(irq, regs + APPLE_DPIN_IRQ_STATUS);
			if (irq & BIT(0))
				writel(readl(regs + APPLE_DPIN_STATUS), regs + APPLE_DPIN_STATUS);
			writel(readl(regs + APPLE_DPIN_IRQ_ENABLE) | 3, regs + APPLE_DPIN_IRQ_ENABLE);
		}
		/* Activate may run before attach returns, so admit its immutable cookie now. */
		apple_dpin_token_admit(&c->tokens, binding->generation);
		apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_ADMITTED, true, 0);
	}

	return apple_dpin_connect(c, binding, true);
}

static void apple_dpin_release_binding(struct apple_dpin_binding *binding)
{
	struct apple_dpin_ctx *c = binding->ctx;
	int ret;

	/* Admission is revoked before the provider retires and drains this token. */
	scoped_guard(mutex, &c->lock)
		apple_dpin_token_revoke(&c->tokens, binding->generation);
	ret = apple_dpin_connect(c, binding, false);
	if (ret && ret != -ESTALE)
		dev_warn(c->acio->dev, "dpin%u: display teardown failed: %d\n",
			 c->idx, ret);
	kfree(binding);
}

static void apple_dpin_down(struct apple_dpin_ctx *c)
{
	void __iomem *regs;

	if (c->binding) {
		apple_dpin_release_binding(c->binding);
		c->binding = NULL;
	}
	/* The powered request hook idled the adapter before queued teardown. */
	scoped_guard(mutex, &c->lock) {
		regs = c->regs;
		c->regs = NULL;
		apple_dpin_step(&c->state, c->acio->dp, APPLE_DPIN_DROPPED, false, 0);
	}
	if (regs)
		iounmap(regs);
}

/*
 * Bring the display side in line with the tunnel state. One work item per
 * adapter on an ordered queue, so events are never lost and nothing has to be
 * allocated on the way down: whatever was handed to appledrm is taken back.
 */
static void apple_dpin_work_fn(struct work_struct *work);

static void apple_dpin_retry_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(to_delayed_work(work),
						struct apple_dpin_ctx, retry);
	unsigned int actions;

	scoped_guard(mutex, &c->lock)
		actions = apple_dpin_step(&c->state, c->acio->dp,
					  APPLE_DPIN_RETRY, !!c->regs, 0);
	if (actions & APPLE_DPIN_QUEUE)
		queue_work(c->acio->dp_wq, &c->work);
}

/*
 * Only where both the display allocator and the connection manager can hold
 * a tunnel for a pipeline (capacity-retry hosts, see
 * apple_nhi_dp_tunnel_awaits_display()) does a refused tunnel wait for one.
 */
static void apple_dpin_work_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(work, struct apple_dpin_ctx, work);
	struct apple_dpin_binding *binding;
	enum apple_dpin_event event;
	bool deferred, stale;
	unsigned int actions;
	u64 generation;
	int ret;

	for (;;) {
		scoped_guard(mutex, &c->lock) {
			actions = apple_dpin_step(&c->state, c->acio->dp,
						  APPLE_DPIN_WORK, !!c->regs, 0);
			generation = c->tokens.requested;
		}

		if (actions & APPLE_DPIN_CANCEL_RETRY)
			cancel_delayed_work(&c->retry);
		if (actions & APPLE_DPIN_DROP) {
			if (actions & APPLE_DPIN_AGAIN)
				dev_info(c->acio->dev,
					 "dpin%u: replacing stale display route for new tunnel\n",
					 c->idx);
			apple_dpin_down(c);
			if (actions & APPLE_DPIN_AGAIN)
				continue;
			dev_dbg(c->acio->dev, "dpin%u: DP tunnel down\n", c->idx);
		}
		if (!(actions & APPLE_DPIN_ATTACH))
			return;

		binding = kzalloc_obj(*binding);
		deferred = false;
		if (binding) {
			binding->ctx = c;
			binding->generation = generation;
			ret = apple_dpin_up(c, binding, &deferred);
		} else {
			ret = -ENOMEM;
		}
		event = deferred ? APPLE_DPIN_DEFER_FIRST : APPLE_DPIN_RESULT;
		scoped_guard(mutex, &c->lock) {
			stale = !c->state.alive || c->tokens.requested != generation;
			if (!ret) {
				stale |= !apple_dpin_token_complete(&c->tokens, generation);
				if (!stale)
					c->binding = binding;
			} else {
				apple_dpin_token_revoke(&c->tokens, generation);
			}
			if (!stale) {
				actions = apple_dpin_step(&c->state, c->acio->dp,
							  event, !!c->regs, ret);
				if (actions & APPLE_DPIN_RECOVERED)
					dev_info(c->acio->dev,
						 "dpin%u: a display pipeline came free; DP tunnel up\n",
						 c->idx);
			}
		}
		if (stale) {
			/* A late success owns only its own token, never the new request. */
			if (!ret)
				apple_dpin_release_binding(binding);
			else
				kfree(binding);
			continue;
		}
		if (ret)
			kfree(binding);
		if (actions & APPLE_DPIN_CANCEL_RETRY)
			cancel_delayed_work(&c->retry);
		if (actions & APPLE_DPIN_AGAIN) {
			if (actions & APPLE_DPIN_LOG_CONNECTED)
				dev_dbg(c->acio->dev, "dpin%u: DP tunnel up\n", c->idx);
			continue;
		}
		if (actions & APPLE_DPIN_FIRST_WAIT)
			dev_info(c->acio->dev,
				 "dpin%u: no display pipeline free; waiting for one\n",
				 c->idx);
		if (actions & APPLE_DPIN_ARM_RETRY)
			mod_delayed_work(c->acio->dp_wq, &c->retry,
					 msecs_to_jiffies(APPLE_DPIN_RETRY_MS));
		if (actions & APPLE_DPIN_WARN)
			dev_warn(c->acio->dev, "dpin%u: DP tunnel setup failed: %d\n",
				 c->idx, ret);
		return;
	}
}

static void apple_cio_destroy_wq(void *data)
{
	struct apple_cio *acio = data;

	/* a pending retry would queue onto the destroyed queue */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++)
		cancel_delayed_work_sync(&acio->dpin[i].retry);
	destroy_workqueue(acio->dp_wq);
}

static void apple_cio_of_node_put(void *data)
{
	of_node_put(data);
}

static bool
apple_cio_pcie_tunnel_is_preinitialized(struct device_node *parent)
{
	for_each_available_child_of_node_scoped(parent, child) {
		u32 status;

		if (!of_device_is_compatible(child, "apple,t8103-pciec") &&
		    !of_device_is_compatible(child, "apple,t6000-pciec"))
			continue;

		if (!of_property_read_u32(child, "apple,pciec-preinit-status",
					  &status) && status == 1)
			return true;
		/* t8103 brings the port up itself. */
		if (of_device_is_compatible(child, "apple,t8103-pciec") &&
		    of_property_read_bool(child, "apple,pciec-kernel-init"))
			return true;
		/* T600x/T602x without a handoff, when opted in. */
		return apple_pcie_tunnel_needs_cold_init(child);
	}

	return false;
}

static void apple_cio_reset_control_put(void *data)
{
	reset_control_put(data);
}

/*
 * PCIe-C's reset covers the controller and the DART beside it. It is
 * optional, since older device trees do not describe it, and only used where
 * the kernel initializes the controller: resetting it would discard a boot
 * loader's setup.
 */
static int apple_cio_get_pcie_reset(struct apple_cio *acio)
{
	struct reset_control *rst = NULL;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!of_device_is_compatible(child, "apple,t8103-pciec") &&
		    !of_device_is_compatible(child, "apple,t6000-pciec"))
			continue;
		if (!of_property_read_bool(child, "apple,pciec-kernel-init"))
			break;

		rst = of_reset_control_get_optional_exclusive(child, NULL);
		break;
	}
	if (IS_ERR(rst))
		return dev_err_probe(acio->dev, PTR_ERR(rst),
				     "Unable to get PCIe-C reset\n");
	if (!rst)
		return 0;

	/* The bound PCIe host owns the line between ACIO startup attempts. */
	reset_control_release(rst);
	acio->pcie_reset = rst;
	return devm_add_action_or_reset(acio->dev, apple_cio_reset_control_put,
					rst);
}

static void apple_cio_iounmap(void *data)
{
	iounmap((__force void __iomem *)data);
}

static int apple_cio_map_pcie_intr2axi(struct apple_cio *acio)
{
	struct resource res;
	int index, ret;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!of_device_is_compatible(child, "apple,t8103-pciec") &&
		    !of_device_is_compatible(child, "apple,t6000-pciec"))
			continue;

		index = of_property_match_string(child, "reg-names", "intr2axi");
		if (index < 0)
			return 0;

		ret = of_address_to_resource(child, index, &res);
		if (ret)
			return ret;

		/*
		 * Intr2AXI controls PCIe configuration transaction forwarding and
		 * must therefore use non-posted Device-nGnRnE accesses.  A regular
		 * ioremap() is Device-nGnRE on arm64 and can leave this pulse pending
		 * until the following DART access, which raises an asynchronous SError.
		 *
		 * Do not reserve the resource here: the PCIe-C child owns the same DT
		 * aperture and will claim it when the tunnel children are populated.
		 */
		acio->pcie_intr2axi_base =
			ioremap_np(res.start, resource_size(&res));
		if (!acio->pcie_intr2axi_base)
			return -ENOMEM;

		ret = devm_add_action_or_reset(acio->dev, apple_cio_iounmap,
					       (__force void *)acio->pcie_intr2axi_base);
		if (ret)
			return ret;

		return 0;
	}

	return -ENODEV;
}

/**
 * struct apple_nhi - Apple Native Host Interface
 * @dev: NHI device
 * @pdev: NHI platform device
 * @np: NHI device tree node
 * @acio: Parent ACIO complex
 * @tb: USB4 domain and software connection manager
 * @nhi: NHI struct
 * @nhi_base: NHI registers
 * @pdf_base: PDF (Protocol Defined Field) configuration registers
 * @tx_irqs: Transmit ring interrupts indexed by HopID
 * @rx_irqs: Receive ring interrupts indexed by HopID
 * @tx_irq_names: Names of the transmit ring interrupts
 * @rx_irq_names: Names of the receive ring interrupts
 * @n_rings: Number of rings in each direction
 */
struct apple_nhi {
	struct device *dev;
	struct platform_device *pdev;
	struct device_node *np;

	struct apple_cio *acio;

	struct tb *tb;
	struct tb_nhi nhi;
	struct tb_nhi_ops ops;

	void __iomem *nhi_base;
	void __iomem *pdf_base;

	int *tx_irqs;
	int *rx_irqs;
	const char **tx_irq_names;
	const char **rx_irq_names;
	size_t n_rings;

	/* DP IN analog/AUX: poll adapter CS after the tunnel is up. */
	struct delayed_work dp_aux_work;
	u8 dp_in_port;
	u32 analog_base;
	u32 dp_in_cs[14];
	bool dp_aux_armed;
	unsigned int dp_aux_polls;
};

#define nhi_to_anhi(nhi_) container_of((nhi_), struct apple_nhi, nhi)

/**
 * apple_cio_rtkit_shmem_setup - Map an RTKit shared memory buffer
 * @cookie: ACIO instance passed to RTKit
 * @bfr: RTKit shared memory buffer descriptor
 *
 * Translate the firmware-provided IOVA into the ACIO SRAM mapping and reject
 * buffers that fall outside the reserved SRAM resource.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_rtkit_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_cio *acio = cookie;
	struct resource res = {
		.name = "acio_rtkit_buffer",
		.flags = acio->sram_res->flags,
	};

	if (!bfr->iova)
		return -EIO;

	if (bfr->iova < APPLE_CIO_SRAM_IOVA_BASE) {
		dev_err(acio->dev, "firmware requested invalid buffer before SRAM IOVA base 0x%llx\n",
			bfr->iova);
		return -EFAULT;
	}

	res.start = bfr->iova - APPLE_CIO_SRAM_IOVA_BASE + acio->sram_res->start;
	res.end = res.start + bfr->size - 1;

	if (res.end < res.start) {
		dev_err(acio->dev, "firmware requested invalid buffer %pR\n", &res);
		return -EFAULT;
	}

	if (!resource_contains(acio->sram_res, &res)) {
		dev_err(acio->dev, "firmware requested buffer %pR outside SRAM %pR\n", &res,
			acio->sram_res);
		return -EFAULT;
	}

	bfr->iomem = acio->sram_base + (res.start - acio->sram_res->start);
	bfr->is_mapped = true;
	return 0;
}

static const struct apple_rtkit_ops apple_cio_rtkit_ops = {
	.shmem_setup = apple_cio_rtkit_shmem_setup,
};

static int apple_nhi_probe_irqs(struct apple_nhi *anhi)
{
	int n_irqs;
	char name[64];

	n_irqs = platform_irq_count(anhi->pdev);
	if (n_irqs < 0)
		return dev_err_probe(anhi->dev, n_irqs, "platform_irq_count failed\n");
	if (n_irqs == 0) {
		dev_err(anhi->dev, "No interrupts found\n");
		return -EINVAL;
	}
	if (n_irqs % 2) {
		dev_err(anhi->dev, "Invalid number of interrupts: %d must be even\n", n_irqs);
		return -EINVAL;
	}
	/*
	 * TX ring n is addressed as bit n and RX ring n as bit n + n_rings of the 32 bit
	 * IRQ status and enable registers, so both halves have to fit into a single
	 * register. hop_count is only checked against n_rings later on, and both come
	 * from firmware, so bound this before anything shifts by the derived index.
	 */
	if (n_irqs > 32) {
		dev_err(anhi->dev, "Invalid number of interrupts: %d exceeds 32 status bits\n",
			n_irqs);
		return -EINVAL;
	}
	anhi->n_rings = n_irqs / 2;

	anhi->rx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->rx_irqs), GFP_KERNEL);
	if (!anhi->rx_irqs)
		return -ENOMEM;
	anhi->tx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->tx_irqs), GFP_KERNEL);
	if (!anhi->tx_irqs)
		return -ENOMEM;
	anhi->rx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->rx_irq_names), GFP_KERNEL);
	if (!anhi->rx_irq_names)
		return -ENOMEM;
	anhi->tx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->tx_irq_names), GFP_KERNEL);
	if (!anhi->tx_irq_names)
		return -ENOMEM;

	for (int i = 0; i < anhi->n_rings; ++i) {
		snprintf(name, sizeof(name), "rxring%d", i);
		anhi->rx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->rx_irqs[i] < 0)
			return anhi->rx_irqs[i];
		anhi->rx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->rx_irq_names[i])
			return -ENOMEM;

		snprintf(name, sizeof(name), "txring%d", i);
		anhi->tx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->tx_irqs[i] < 0)
			return anhi->tx_irqs[i];
		anhi->tx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->tx_irq_names[i])
			return -ENOMEM;
	}

	return 0;
}

static unsigned int apple_cio_ring_index(struct tb_ring *ring)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);

	if (ring->is_tx)
		return ring->hop;
	else
		return ring->hop + anhi->n_rings;
}

static void apple_nhi_ring_interrupt_active(struct tb_ring *ring, bool active)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);
	u32 reg, interval;

	lockdep_assert_held(&ring->nhi->lock);

	if (active && ring->interval_nsec) {
		interval = min_t(u32, ring->interval_nsec,
				 FIELD_MAX(APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK) *
				 APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		interval = DIV_ROUND_UP(interval,
					APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		writel(interval, anhi->nhi_base + APPLE_CIO_NHI_IRQ_THROTTLE +
				 4 * idx);
	}

	reg = readl(anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);

	if (active)
		reg |= BIT(idx);
	else
		reg &= ~BIT(idx);

	writel(reg, anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);
}

static void apple_nhi_ring_interrupt_mask(struct tb_ring *ring, bool mask)
{
	apple_nhi_ring_interrupt_active(ring, !mask);
}

static irqreturn_t apple_cio_ring_irq(int irq, void *data)
{
	struct tb_ring *ring = data;
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);

	guard(spinlock)(&ring->nhi->lock);
	guard(spinlock)(&ring->lock);

	writel(BIT(idx), anhi->nhi_base + APPLE_CIO_NHI_IRQ_STATUS);
	if (!ring->running)
		return IRQ_HANDLED;

	if (ring->start_poll) {
		apple_nhi_ring_interrupt_mask(ring, true);
		ring->start_poll(ring->poll_data);
	} else {
		schedule_work(&ring->work);
	}

	return IRQ_HANDLED;
}


static int apple_nhi_request_irq(struct tb_ring *ring, bool no_suspend)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	const char *name;

	if (ring->is_tx) {
		ring->irq = anhi->tx_irqs[ring->hop];
		name = anhi->tx_irq_names[ring->hop];
	} else {
		ring->irq = anhi->rx_irqs[ring->hop];
		name = anhi->rx_irq_names[ring->hop];
	}

	return devm_request_irq(anhi->dev, ring->irq, apple_cio_ring_irq,
				no_suspend ? IRQF_NO_SUSPEND : 0, name, ring);
}

static void apple_nhi_release_irq(struct tb_ring *ring)
{
	if (ring->irq <= 0)
		return;

	devm_free_irq(ring->nhi->dev, ring->irq, ring);
	ring->irq = 0;
}

static void apple_nhi_ring_configure(struct tb_ring *ring, u32 flags, u32 e2e_flags)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	void __iomem *options;
	u32 sof_eof_mask;

	lockdep_assert_held(&ring->lock);

	options = anhi->nhi_base + ring->hop * APPLE_CIO_NHI_RING_STRIDE + 0x10;

	if (ring->is_tx) {
		options += APPLE_CIO_NHI_TXRING_DESC_BASE;

		/*
		 * Partition the 232 shared TX entries: two for control ring 0,
		 * forty each for rings 1-5, and five each for the remaining rings.
		 */
		if (ring->hop == 0)
			writel(2, options + 4);
		else if (ring->hop <= 5)
			writel(40, options + 4);
		else
			writel(5, options + 4);
	} else {
		options += APPLE_CIO_NHI_RXRING_DESC_BASE;

		sof_eof_mask = ring->sof_mask << 16 | ring->eof_mask;
		/*
		 * T8152 has no PDF range (the ACIO+0xdb0000 block of the M1/M2 DT
		 * aborts on a read) and macOS never writes RX options + 4 there:
		 * without a "pdf" range only the ring options are programmed. Type 7
		 * keeps the masks in the receive ring registers.
		 */
		if (anhi->pdf_base || (anhi->acio && anhi->acio->type7))
			writel(sof_eof_mask, options + 4);
		if (anhi->pdf_base)
			writel(sof_eof_mask,
			       anhi->pdf_base + ring->hop * APPLE_CIO_NHI_PDF_STRIDE);
	}

	/*
	 * The firmware samples E2E flow control when the valid bit is set.
	 * Program both in the same write.
	 */
	writel(flags | e2e_flags, options);
}

static struct platform_device *apple_cio_find_pcie_tunnel(struct apple_cio *acio);

static int apple_cio_populate_pcie_tunnel(struct apple_cio *acio)
{
	int ret;

	/*
	 * PCIe-C was cold-initialized by m1n1 before the ACIO M3 started, but
	 * enabling ACIO's cable-powered PCIe domain closes the Intr2AXI bridge
	 * again. The port enable sequence pulses this register immediately
	 * before forcing its DART active. Do the same after ACIO has
	 * enabled both tunnel adapters and before either child can touch DART MMIO.
	 * The bit self-clears after opening the bridge.
	 */
	if (acio->pcie_intr2axi_base) {
		writel(APPLE_CIO_PCIEC_INTR2AXI_ENABLE,
		       acio->pcie_intr2axi_base + APPLE_CIO_PCIEC_INTR2AXI_CTRL);
		/* Complete the pulse without reading the transient register. */
		mb();
		dev_info(acio->dev, "PCIe-C Intr2AXI bridge enabled\n");
	}

	/*
	 * The DART is the first child and probes as soon as it is created.
	 * Its command engine stays busy until this port clock is running, so
	 * finish cold init before either child is populated.
	 */
	ret = apple_pcie_tunnel_prepare(acio->dev, acio->pcie_tunnel_np);
	if (ret)
		return ret;

	dev_info(acio->dev, "PCIe-C populating tunnel children\n");
	return of_platform_populate(acio->pcie_tunnel_np, NULL, NULL,
				    acio->dev);
}

/* The host must have finished probing before activation can be announced. */
static int apple_cio_pcie_tunnel_ready_locked(struct apple_cio *acio)
{
	struct platform_device *pdev;
	int ret;

	lockdep_assert_held(&acio->pcie_tunnel_lock);
	pdev = apple_cio_find_pcie_tunnel(acio);
	if (!pdev)
		return -EAGAIN;

	ret = apple_pcie_tunnel_restore(&pdev->dev);
	if (!ret)
		ret = apple_pcie_tunnel_check_state(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}

static int apple_cio_activate_pcie_tunnel_locked(struct apple_cio *acio)
{
	int ret = 0;

	lockdep_assert_held(&acio->pcie_tunnel_lock);

	if (!acio->pcie_tunnel_np)
		return 0;

	/*
	 * The host survives a tunnel teardown, quiesced with its hierarchy
	 * removed. Bring that one back rather than populating a second.
	 */
	if (acio->pcie_tunnel_populated)
		return apple_cio_pcie_tunnel_ready_locked(acio);
	if (!acio->pcie_tunnel_preinitialized)
		return dev_err_probe(acio->dev, -ENODEV,
				     "PCIe-C is not initialized by m1n1 or the kernel\n");

	if (acio->pd_list->num_pds < 4 || !acio->pd_list->pd_links[3])
		return dev_err_probe(acio->dev, -ENODEV,
				     "pre-ACIO PCIe-C initialization is not active\n");

	ret = apple_cio_populate_pcie_tunnel(acio);
	if (ret)
		return dev_err_probe(acio->dev, ret,
				     "failed to populate tunneled PCIe controller\n");

	acio->pcie_tunnel_populated = true;
	dev_info(acio->dev,
		 "PCIe-C populated after USB4 PCIe tunnel adapter enable\n");
	return apple_cio_pcie_tunnel_ready_locked(acio);
}

static int apple_cio_quiesce_pcie_tunnel_locked(struct apple_cio *acio)
{
	struct platform_device *pcie_pdev;
	int ret = 0;

	lockdep_assert_held(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (pcie_pdev) {
		ret = apple_pcie_tunnel_quiesce(&pcie_pdev->dev);
		put_device(&pcie_pdev->dev);
	}
	if (!ret)
		acio->pcie_quiesce_pending = false;
	return ret;
}

#define APPLE_CIO_PCIE_BIND_RETRIES	20
#define APPLE_CIO_PCIE_BIND_WAIT_MS	250

static void apple_cio_pcie_tunnel_work(struct work_struct *work)
{
	struct apple_cio *acio =
		container_of(to_delayed_work(work), struct apple_cio,
			     pcie_tunnel_work);
	bool activated = false, failed = false;
	int ret = 0;

	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping || acio->pcie_pm_prepared)
		goto unlock;
	if (acio->pcie_quiesce_pending) {
		ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
		if (ret)
			goto unlock;
	}
	if (READ_ONCE(acio->pcie_tunnel_requested)) {
		ret = apple_cio_activate_pcie_tunnel_locked(acio);
		activated = !ret && acio->pcie_tunnel_populated;
		if (ret == -EAGAIN) {
			if (acio->pcie_bind_retries) {
				acio->pcie_bind_retries--;
				mod_delayed_work(system_freezable_wq,
						 &acio->pcie_tunnel_work,
						 msecs_to_jiffies(APPLE_CIO_PCIE_BIND_WAIT_MS));
			} else {
				ret = -ETIMEDOUT;
			}
		}
		failed = ret && ret != -EAGAIN;
	}
unlock:
	mutex_unlock(&acio->pcie_tunnel_lock);
	/* Wake a deferred Type-C check after asynchronous PCIe setup finishes. */
	if (activated)
		typec_thunderbolt_switch_notify_ready(acio->tbt_switch);
	else if (failed)
		typec_thunderbolt_switch_notify(acio->tbt_switch);
	if (ret && ret != -EAGAIN)
		dev_err(acio->dev, "deferred PCIe-C transition failed: %d\n", ret);
}

static int apple_nhi_pci_tunnel_deactivate(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	int ret = 0;

	/*
	 * The tunnel is going away while the router and NHI stay up, so nothing
	 * else tears the tunneled PCI hierarchy down. Left alone the endpoint
	 * keeps its driver bound with no data path underneath it: every access
	 * blocks until the function driver's own timeout fires, and the host has
	 * nothing to enumerate onto when the tunnel comes back.
	 */
	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping)
		goto unlock;
	WRITE_ONCE(acio->pcie_tunnel_requested, false);
	cancel_delayed_work(&acio->pcie_tunnel_work);
	acio->pcie_quiesce_pending = true;
	/* Endpoint drivers must resume before they can be unbound safely. */
	if (!acio->pcie_pm_prepared) {
		ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
		if (ret)
			dev_warn(acio->dev,
				 "failed to quiesce PCIe-C on tunnel teardown: %d\n",
				 ret);
	}

unlock:
	mutex_unlock(&acio->pcie_tunnel_lock);

	return ret;
}

static int apple_nhi_pci_tunnel_post_activate(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;

	/*
	 * This is the boundary at which the tunnel becomes usable.
	 * The USB4 configuration writes are asynchronous, so return to the
	 * Thunderbolt core before probing PCIe-C. The worker then waits on the
	 * hardware reset state without blocking completion of the tunnel paths.
	 */
	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping) {
		mutex_unlock(&acio->pcie_tunnel_lock);
		return -ESHUTDOWN;
	}
	WRITE_ONCE(acio->pcie_tunnel_requested, true);
	acio->pcie_bind_retries = APPLE_CIO_PCIE_BIND_RETRIES;
	/* No PCI removal or rescan until system resume has finished. */
	mod_delayed_work(system_freezable_wq, &acio->pcie_tunnel_work, 0);
	mutex_unlock(&acio->pcie_tunnel_lock);
	return 0;
}

static void apple_cio_stop_pcie_tunnel(struct apple_cio *acio)
{
	int ret;

	/* Block new requests before draining work, without nesting tb->lock. */
	mutex_lock(&acio->pcie_tunnel_lock);
	acio->pcie_tunnel_stopping = true;
	WRITE_ONCE(acio->pcie_tunnel_requested, false);
	mutex_unlock(&acio->pcie_tunnel_lock);
	cancel_delayed_work_sync(&acio->pcie_tunnel_work);

	/* The NHI must remain alive to complete the tunneled reset handshake. */
	mutex_lock(&acio->pcie_tunnel_lock);
	ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
	mutex_unlock(&acio->pcie_tunnel_lock);
	if (ret)
		dev_warn(acio->dev,
			 "failed to quiesce PCIe-C before NHI shutdown: %d\n", ret);
}

#define APPLE_DP_AUX_POLL_MS		500
#define APPLE_DP_AUX_POLL_MAX		24

/*
 * apple,tunable-rc programs two analog PHY blocks inside the already-mapped
 * ACIO RC window (0xc000). They match the two host DP IN adapters (0:5, 0:6).
 * Dumping 0x00-0xff never saw them. Do not scan unmapped ACIO ranges.
 */
#define APPLE_CIO_DPIN0_ANALOG		0x4000
#define APPLE_CIO_DPIN1_ANALOG		0x8000
#define APPLE_CIO_DPIN_ANALOG_SIZE	0x80
#define APPLE_CIO_DPIN_ANALOG_FSM	0x18
#define APPLE_CIO_DPIN_ANALOG_HOLE	0x20
#define APPLE_CIO_DPIN_ANALOG_EMPTY	0x80000000
#define APPLE_CIO_DPIN_ANALOG_HOLE_VAL	0x40

/*
 * Analog MMIO writes are closed: +0x00 / +0x18 / +0x20 do not stick.
 * +0x18 is first-read status 0x1017 (read-to-clear).
 *
 * 0 = do not touch analog at tunnel-up; dump it only after DPRX timeout
 *     (default). Tests whether consuming +0x18 aborted a handshake.
 * 1 = old pulse +0x00 / fill +0x20 (rollback)
 */
static bool apple_dpin_aux;
module_param_named(dpin_aux, apple_dpin_aux, bool, 0644);
MODULE_PARM_DESC(dpin_aux,
		 "Apple DP IN analog: pulse +0x00 at the next tunnel activation (default: false)");

static void apple_dp_dump_hop(struct tb_port *port, unsigned int hopid)
{
	struct tb_regs_hop hop;
	int ret;

	ret = tb_port_read(port, &hop, TB_CFG_HOPS, 2 * hopid, 2);
	if (ret) {
		tb_port_dbg(port, "DP IN hop %u read failed: %d\n", hopid, ret);
		return;
	}
	tb_port_dbg(port,
		     "DP IN hop %u enable=%u out=%u next=%u credits=%u\n",
		     hopid, hop.enable, hop.out_port, hop.next_hop,
		     hop.initial_credits);
}



static void apple_dp_dump_adapter(struct tb_port *port, const char *tag)
{
	u32 w[17], cs4 = 0xffffffff;
	int i, ret;

	ret = tb_port_read(port, &w[0], TB_CFG_PORT, port->cap_adap, 1);
	if (ret || w[0] == 0xffffffff) {
		tb_port_dbg(port, "%s config space dead ret=%d (not DPRX)\n",
			     tag, ret);
		return;
	}
	for (i = 1; i <= 16; i++) {
		ret = tb_port_read(port, &w[i], TB_CFG_PORT,
				   port->cap_adap + i, 1);
		if (ret)
			w[i] = 0xffffffff;
	}
	tb_port_read(port, &cs4, TB_CFG_PORT, ADP_CS_4, 1);
	tb_port_dbg(port,
		     "%s type=%06x cap=%d CS0=%08x CS1=%08x CS2=%08x CS3=%08x LOCAL=%08x REMOTE=%08x STAT=%08x COMMON=%08x CS8=%08x CS9=%08x CS10=%08x ADP_CS4=%08x VE=%u AE=%u HPD=%u DPRX=%u LCK=%u\n",
		     tag, port->config.type, port->cap_adap,
		     w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8],
		     w[9], w[10], cs4,
		     !!(w[0] & ADP_DP_CS_0_VE), !!(w[0] & ADP_DP_CS_0_AE),
		     !!(w[2] & ADP_DP_CS_2_HPD),
		     !!(w[7] & DP_COMMON_CAP_DPRX_DONE),
		     !!(cs4 & ADP_CS_4_LCK));
	tb_port_dbg(port,
		     "%s CS11=%08x CS12=%08x CS13=%08x CS14=%08x CS15=%08x CS16=%08x disc=%u\n",
		     tag, w[11], w[12], w[13], w[14], w[15], w[16],
		     !!(w[13] & ADP_DP_CS_13_DPTX_DISCOVERY_MODE));
}

static void apple_dp_dump_host_adapters(struct apple_nhi *anhi)
{
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (tb_port_is_dpin(port))
			apple_dp_dump_adapter(port, "host DP IN");
		else if (tb_port_is_dpout(port))
			apple_dp_dump_adapter(port, "host DP OUT");
	}
}

static void apple_dp_dump_rc_range(struct apple_cio *acio, u32 base, u32 len,
				   const char *tag, bool skip_zero)
{
	char buf[320];
	int n = 0;
	u32 off, size;

	if (!acio->rc_base || !acio->rc_res)
		return;
	size = resource_size(acio->rc_res);
	if (base >= size)
		return;
	if (base + len > size)
		len = size - base;

	for (off = 0; off < len; off += 4) {
		u32 val = readl(acio->rc_base + base + off);

		if (skip_zero && !val)
			continue;
		n += scnprintf(buf + n, sizeof(buf) - n, " %03x=%08x", off, val);
		if (n >= (int)sizeof(buf) - 24) {
			dev_dbg(acio->dev, "ACIO RC %s 0x%x:%s\n", tag, base, buf);
			n = 0;
			buf[0] = '\0';
		}
	}
	if (n)
		dev_dbg(acio->dev, "ACIO RC %s 0x%x:%s\n", tag, base, buf);
}

static void apple_dp_dump_rc(struct apple_cio *acio)
{
	apple_dp_dump_rc_range(acio, 0, 0x100, "ctrl", true);
}

static void apple_dp_dump_vse(struct tb_switch *sw)
{
	char buf[320];
	int cap, i, n = 0, ret;
	u32 w;

	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0) {
		tb_sw_dbg(sw, "Apple VSE cap missing: %d\n", cap);
		return;
	}
	for (i = 0; i < 16; i++) {
		ret = tb_sw_read(sw, &w, TB_CFG_SWITCH, cap + i, 1);
		if (ret)
			w = 0xffffffff;
		n += scnprintf(buf + n, sizeof(buf) - n, " %02x=%08x", i, w);
		if (n >= (int)sizeof(buf) - 20) {
			tb_sw_dbg(sw, "Apple VSE cap=%d:%s\n", cap, buf);
			n = 0;
			buf[0] = '\0';
		}
	}
	if (n)
		tb_sw_dbg(sw, "Apple VSE cap=%d:%s\n", cap, buf);
}



static void apple_dp_dump_analog(struct apple_cio *acio, const char *tag)
{
	apple_dp_dump_rc_range(acio, APPLE_CIO_DPIN0_ANALOG,
			      APPLE_CIO_DPIN_ANALOG_SIZE, tag, false);
	apple_dp_dump_rc_range(acio, APPLE_CIO_DPIN1_ANALOG,
			      APPLE_CIO_DPIN_ANALOG_SIZE, "dpin1 analog", false);
}

static u32 apple_dp_in_analog_base(struct apple_nhi *anhi, struct tb_port *in)
{
	unsigned int idx = 0;
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return APPLE_CIO_DPIN0_ANALOG;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port == in)
			return idx ? APPLE_CIO_DPIN1_ANALOG :
				     APPLE_CIO_DPIN0_ANALOG;
		idx++;
	}
	return APPLE_CIO_DPIN0_ANALOG;
}

static void apple_dp_start_analog(struct apple_nhi *anhi, bool pulse)
{
	struct apple_cio *acio;
	u32 block, ctrl, hole, fsm;

	if (!anhi || !anhi->acio || !anhi->acio->rc_base)
		return;
	acio = anhi->acio;
	block = anhi->analog_base ?: APPLE_CIO_DPIN0_ANALOG;
	if (block + APPLE_CIO_DPIN_ANALOG_HOLE + 4 > resource_size(acio->rc_res))
		return;

	ctrl = readl(acio->rc_base + block);
	hole = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
	fsm = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_FSM);
	dev_info(acio->dev,
		 "DP IN analog +0x00=%08x +0x18=%08x +0x20=%08x pulse=%d\n",
		 ctrl, fsm, hole, pulse);

	if (!pulse)
		return;

	/* +0x00 readback is empty status; still pulse start like PCIe-C. */
	writel(ctrl | BIT(0), acio->rc_base + block);
	mb();
	ctrl = readl(acio->rc_base + block);

	if (hole == 0) {
		writel(APPLE_CIO_DPIN_ANALOG_HOLE_VAL,
		       acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
		mb();
		hole = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
	}

	fsm = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_FSM);
	dev_info(acio->dev,
		 "DP IN analog after start +0x00=%08x +0x18=%08x +0x20=%08x\n",
		 ctrl, fsm, hole);
	apple_dp_dump_rc_range(acio, block, APPLE_CIO_DPIN_ANALOG_SIZE,
			      "dpin analog after start", false);
}

static int apple_dp_dptx_discover(struct tb_port *in)
{
	u32 cs0 = 0;

	tb_port_read(in, &cs0, TB_CFG_PORT, in->cap_adap + ADP_DP_CS_0, 1);
	tb_port_warn(in,
		     "USB4 DPTX Discovery closed (CS13 writable, CS9 static); AE=%u VE=%u\n",
		     !!(cs0 & ADP_DP_CS_0_AE), !!(cs0 & ADP_DP_CS_0_VE));
	return 0;
}

static void apple_dp_aux_work(struct work_struct *work)
{
	struct apple_nhi *anhi =
		container_of(to_delayed_work(work), struct apple_nhi,
			     dp_aux_work);
	struct tb_port *port;
	u32 cs[14];
	int i, ret;
	bool changed = false, dprx = false;

	if (!anhi->tb)
		return;

	mutex_lock(&anhi->tb->lock);
	if (!anhi->dp_aux_armed || !anhi->tb->root_switch)
		goto out_unlock;

	port = &anhi->tb->root_switch->ports[anhi->dp_in_port];
	if (!tb_port_is_dpin(port))
		goto out_unlock;

	for (i = 0; i <= 13; i++) {
		ret = tb_port_read(port, &cs[i], TB_CFG_PORT,
				   port->cap_adap + i, 1);
		if (ret)
			cs[i] = 0xffffffff;
		if (cs[i] != anhi->dp_in_cs[i])
			changed = true;
	}
	if (changed) {
		memcpy(anhi->dp_in_cs, cs, sizeof(cs));
		tb_port_dbg(port,
			     "DP IN CS changed CS0=%08x CS2=%08x CS9=%08x CS13=%08x COMMON=%08x VE=%u AE=%u HPD=%u DPRX=%u disc=%u\n",
			     cs[0], cs[2], cs[9], cs[13], cs[7],
			     !!(cs[0] & ADP_DP_CS_0_VE),
			     !!(cs[0] & ADP_DP_CS_0_AE),
			     !!(cs[2] & ADP_DP_CS_2_HPD),
			     !!(cs[7] & DP_COMMON_CAP_DPRX_DONE),
			     !!(cs[13] & ADP_DP_CS_13_DPTX_DISCOVERY_MODE));
	}
	dprx = !!(cs[7] & DP_COMMON_CAP_DPRX_DONE);
	if (dprx)
		tb_port_dbg(port, "DP IN DPRX_DONE=1 (ACIO AUX completed)\n");

	anhi->dp_aux_polls++;
	/*
	 * Deliberately not dumping or reading the analog block on every
	 * poll here. +0x18 is an ordinary one-shot, read-to-clear status
	 * register -- already documented above (APPLE_CIO_DPIN_ANALOG_EMPTY,
	 * and the comment a few lines up: "+0x18 is first-read status
	 * 0x1017 (read-to-clear)"), not a stuck error FSM, so polling it
	 * repeatedly has no diagnostic value and risks consuming a status
	 * this project's own dpin_aux default-off comment already warns
	 * could abort a handshake if read too early. A write-1-to-clear
	 * ack against it has no effect either: 0x80000000 is the
	 * register's own idle/empty sentinel, not a latched error that
	 * needs acknowledging. Stick to a single post-mortem dump below,
	 * after the fact, once polling ends.
	 */
	if (!dprx && anhi->dp_aux_polls < APPLE_DP_AUX_POLL_MAX) {
		mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work,
				 msecs_to_jiffies(APPLE_DP_AUX_POLL_MS));
	} else if (anhi->acio) {
		apple_dp_dump_analog(anhi->acio,
				     dprx ? "dpin0 analog DPRX done" :
					    "dpin0 analog at timeout");
	}

out_unlock:
	mutex_unlock(&anhi->tb->lock);
}

/*
 * Which DP IN adapter (0/1) on the host router this port is -- same
 * counting order as apple_dp_in_analog_base() above and the port-to-dpin
 * mapping in apple_nhi_dp_tunnel_changed().
 */
static int apple_dpin_index_for_port(struct apple_nhi *anhi, struct tb_port *in)
{
	unsigned int idx = 0;
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return -1;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port == in)
			return idx;
		idx++;
	}
	return -1;
}

static int apple_nhi_dp_tunnel_pre_activate(struct tb_nhi *nhi,
					    struct tb_port *in,
					    struct tb_port *out)
{
	if (tb_port_is_dpin(in))
		return apple_dp_dptx_discover(in);
	return 0;
}

static int apple_nhi_dp_tunnel_post_activate(struct tb_nhi *nhi,
					     struct tb_port *in,
					     struct tb_port *out, u64 generation)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	int i, idx;
	struct apple_dpin_ctx *c = NULL;

	/* Validate the token before changing analog/AUX or display state. */
	idx = apple_dpin_index_for_port(anhi, in);
	if (idx >= 0 && idx <= 1 && anhi->acio && anhi->acio->connector_np &&
	    anhi->acio->dp_wq) {
		c = &anhi->acio->dpin[idx];
		scoped_guard(mutex, &c->lock) {
			if (!apple_dpin_request(c, generation, true))
				return -ESTALE;
		}
	}

	dev_info(anhi->dev,
		 "DP IN tunnel routing: tunnel %u:%u <-> %u:%u\n",
		 in->sw->config.depth, in->port,
		 out ? out->sw->config.depth : 0,
		 out ? out->port : 0);

	anhi->dp_in_port = in->port;
	anhi->analog_base = apple_dp_in_analog_base(anhi, in);
	anhi->dp_aux_polls = 0;
	anhi->dp_aux_armed = true;
	/* The domain lock keeps this optional MMIO access ahead of teardown. */
	if (READ_ONCE(apple_dpin_aux))
		apple_dp_start_analog(anhi, true);

	apple_dp_dump_rc(anhi->acio);
	apple_dp_dump_vse(in->sw);

	/* Route the display pipeline through the selected host DP IN adapter. */
	if (idx < 0 || idx > 1 || !anhi->acio) {
		dev_warn(anhi->dev,
			 "DP IN tunnel: could not map port %u to a dpin index\n",
			 in->port);
	} else if (!anhi->acio->connector_np) {
		dev_warn(anhi->dev,
			 "DP IN tunnel: no Type-C connector for dpin%d\n", idx);
	} else if (!anhi->acio->dp_wq) {
		dev_warn(anhi->dev, "DP IN tunnel: no work queue for dpin%d\n", idx);
	} else {
		/* Request and REARM were published together before the diagnostics. */
		queue_work(anhi->acio->dp_wq, &c->work);
	}
	apple_dp_dump_host_adapters(anhi);
	if (tb_port_is_dpin(in)) {
		apple_dp_dump_hop(in, 8);
		apple_dp_dump_hop(in, 9);
	}
	if (out && tb_port_is_dpout(out))
		apple_dp_dump_adapter(out, "hub DP OUT");

	for (i = 0; i <= 13; i++) {
		if (tb_port_read(in, &anhi->dp_in_cs[i], TB_CFG_PORT,
				 in->cap_adap + i, 1))
			anhi->dp_in_cs[i] = 0xffffffff;
	}
	mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work,
			 msecs_to_jiffies(APPLE_DP_AUX_POLL_MS));
	return 0;
}

static void apple_nhi_dp_tunnel_deactivate(struct tb_nhi *nhi,
					   struct tb_port *in,
					   struct tb_port *out, u64 generation)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	int idx;

	/* Sleep the adapter before tunnel teardown can remove its power. */
	idx = apple_dpin_index_for_port(anhi, in);
	if (idx >= 0 && idx <= 1 && anhi->acio && anhi->acio->dp_wq) {
		struct apple_dpin_ctx *c = &anhi->acio->dpin[idx];

		scoped_guard(mutex, &c->lock) {
			if (!apple_dpin_request(c, generation, false))
				return;
		}
		queue_work(anhi->acio->dp_wq, &c->work);
	}
	anhi->dp_aux_armed = false;
	cancel_delayed_work(&anhi->dp_aux_work);
	dev_info(anhi->dev, "DP IN tunnel routing: tunnel down\n");
}

static void apple_nhi_dp_tunnel_changed(struct tb_nhi *nhi, u8 in_port,
					u64 generation, bool active)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	struct tb_port *port;
	unsigned int dpin = 0;
	bool found = false;

	/* The crossbar's dpin0/dpin1 follow the host router's DP IN adapters in order. */
	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port->port == in_port) {
			found = true;
			break;
		}
		dpin++;
	}
	if (!found || dpin > 1) {
		dev_warn(acio->dev, "DP tunnel from unexpected adapter %u\n", in_port);
		return;
	}

	/*
	 * Runs synchronously before the tunnel is torn down: from here on the
	 * DP IN block may lose power, so nothing may touch it any more.
	 */
	scoped_guard(mutex, &acio->dpin[dpin].lock) {
		struct apple_dpin_ctx *c = &acio->dpin[dpin];

		/* Publish cookie, powered retirement and lifecycle event under one lock. */
		if (!apple_dpin_request(c, generation, active))
			return;
	}
	queue_work(acio->dp_wq, &acio->dpin[dpin].work);
}

/* The display side refused this tunnel a pipeline for now and asks again. */
static bool apple_nhi_dp_tunnel_awaits_display(struct tb_nhi *nhi,
					       struct tb_port *in)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_dpin_ctx *c;
	int idx;

	idx = apple_dpin_index_for_port(anhi, in);
	if (idx < 0 || idx > 1 || !anhi->acio)
		return false;
	c = &anhi->acio->dpin[idx];
	guard(mutex)(&c->lock);
	return apple_dpin_awaits_display(&c->state, c->acio->dp);
}

static const struct tb_nhi_ops apple_nhi_ops = {
	.request_ring_irq = apple_nhi_request_irq,
	.release_ring_irq = apple_nhi_release_irq,
	.ring_interrupt_active = apple_nhi_ring_interrupt_active,
	.ring_interrupt_mask = apple_nhi_ring_interrupt_mask,
	.ring_configure = apple_nhi_ring_configure,
	.pci_tunnel_post_activate = apple_nhi_pci_tunnel_post_activate,
	.pci_tunnel_deactivate = apple_nhi_pci_tunnel_deactivate,
};

static const struct tb_nhi_ring_layout apple_nhi_ring_layout = {
	.tx_desc_base = APPLE_CIO_NHI_TXRING_DESC_BASE,
	.rx_desc_base = APPLE_CIO_NHI_RXRING_DESC_BASE,
	.desc_stride = APPLE_CIO_NHI_RING_STRIDE,
	.tx_options_base = APPLE_CIO_NHI_TXRING_DESC_BASE + 0x10,
	.rx_options_base = APPLE_CIO_NHI_RXRING_DESC_BASE + 0x10,
	.options_stride = APPLE_CIO_NHI_RING_STRIDE,
};

static int apple_nhi_probe(struct platform_device *pdev)
{
	struct apple_cio *acio = dev_get_drvdata(pdev->dev.parent);
	struct apple_nhi *anhi;
	struct apple_tunable *tunable;
	struct resource *res;
	struct tb_port *port;
	enum apple_dpin_flow dp_hooks;
	int cap_apple;
	int ret = 0;

	if (acio->type7 !=
	    of_device_is_compatible(pdev->dev.of_node, "apple,t8142-usb4-nhi")) {
		ret = dev_err_probe(&pdev->dev, -EINVAL, "ACIO/NHI generation mismatch\n");
		goto err;
	}

	anhi = devm_kzalloc(&pdev->dev, sizeof(*anhi), GFP_KERNEL);
	if (!anhi) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->pdev = pdev;
	anhi->dev = &pdev->dev;
	anhi->np = pdev->dev.of_node;
	anhi->acio = acio;
	INIT_DELAYED_WORK(&anhi->dp_aux_work, apple_dp_aux_work);
	platform_set_drvdata(pdev, anhi);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err;
	if (acio->type7 &&
	    of_property_count_u32_elems(anhi->np, "apple,tunable-nhi") <= 0) {
		ret = dev_err_probe(anhi->dev, -EINVAL,
				    "Missing bootloader NHI tuning\n");
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nhi");
	anhi->nhi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(anhi->nhi_base)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->nhi_base),
				    "Unable to map NHI regs\n");
		goto err;
	}
	tunable = devm_apple_tunable_parse(&pdev->dev, anhi->np, "apple,tunable-nhi", res);
	if (IS_ERR(tunable)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(tunable), "Unable to load NHI tunable\n");
		goto err;
	}
	apple_tunable_apply(anhi->nhi_base, tunable);

	/*
	 * The M1/M2 DTs carry a "pdf" range; type 7 keeps the PDF masks in the
	 * receive ring registers only, and T8152 has no "pdf" range at all
	 * (see apple_nhi_ring_configure).
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pdf");
	if (res && !acio->type7) {
		anhi->pdf_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(anhi->pdf_base)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->pdf_base),
					    "Unable to map PDF regs\n");
			goto err;
		}
	}

	ret = apple_nhi_probe_irqs(anhi);
	if (ret)
		goto err;

	spin_lock_init(&anhi->nhi.lock);
	anhi->nhi.iommu_dma_protection = true;
	/*
	 * DP tunnel display support (and with it the host DP IN handling in
	 * the connection manager) only where it is enabled and known to work.
	 */
	anhi->ops = apple_nhi_ops;
	anhi->nhi.host_dp_dual_stream = acio->dp_dual_stream;
	dp_hooks = apple_dpin_hooks(acio->dp, !!acio->dp_wq, dp_display);
	if (dp_hooks == APPLE_DPIN_CHANGED) {
		anhi->ops.dp_tunnel_changed = apple_nhi_dp_tunnel_changed;
		anhi->nhi.host_dp_policy = acio->dp->host_policy;
		if (acio->dp->capacity_retry)
			anhi->ops.dp_tunnel_awaits_display =
				apple_nhi_dp_tunnel_awaits_display;
	}
	if (dp_hooks == APPLE_DPIN_PRE_POST) {
		anhi->ops.dp_tunnel_pre_activate = apple_nhi_dp_tunnel_pre_activate;
		anhi->ops.dp_tunnel_post_activate = apple_nhi_dp_tunnel_post_activate;
		anhi->ops.dp_tunnel_deactivate = apple_nhi_dp_tunnel_deactivate;
		anhi->nhi.host_dp_policy = acio->dp->host_policy;
	}
	anhi->nhi.ops = &anhi->ops;
	anhi->nhi.ring_layout = &apple_nhi_ring_layout;
	anhi->nhi.iobase = anhi->nhi_base;
	anhi->nhi.quirks = QUIRK_NO_DMA_PORT | QUIRK_NO_USB3_BW_ALLOC;
	if (anhi->nhi.host_dp_policy)
		anhi->nhi.quirks |= QUIRK_HOST_DP_NFC_CREDITS;

	anhi->nhi.hop_count = readl(anhi->nhi_base + APPLE_CIO_NHI_HOP_COUNT) &
			      APPLE_CIO_NHI_HOP_COUNT_MASK;
	if (anhi->nhi.hop_count != anhi->n_rings) {
		ret = dev_err_probe(anhi->dev, -EINVAL, "Ring IRQs (%zd) != HOP_COUNT (%d)\n",
				    anhi->n_rings, anhi->nhi.hop_count);
		goto err;
	}

	anhi->nhi.tx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.tx_rings), GFP_KERNEL);
	anhi->nhi.rx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.rx_rings), GFP_KERNEL);
	if (!anhi->nhi.tx_rings || !anhi->nhi.rx_rings) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->nhi.dev = &pdev->dev;
	init_completion(&anhi->nhi.domain_released);
	anhi->tb = tb_probe(&anhi->nhi);
	if (!anhi->tb) {
		ret = dev_err_probe(anhi->dev, -ENODEV,
				    "Failed to init software connection manager\n");
		goto err;
	}

	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		acio->pcie_tunnel_stopping = false;
		acio->pcie_pm_prepared = false;
		acio->pcie_resume_expected = false;
	}
	ret = tb_domain_add(anhi->tb, false);
	if (ret) {
		dev_err_probe(anhi->dev, ret, "failed to add TB domain\n");
		apple_cio_stop_pcie_tunnel(acio);
		tb_domain_put(anhi->tb);
		wait_for_completion(&anhi->nhi.domain_released);
		goto err;
	}

	mutex_lock(&anhi->tb->lock);

	if (!anhi->tb->root_switch->drom) {
		dev_err(anhi->dev, "No valid host DROM in the device tree\n");
		ret = -EINVAL;
		goto err_unlock_tb_domain;
	}

	cap_apple = tb_switch_find_vse_cap(anhi->tb->root_switch, TB_VSE_CAP_APPLE);

	if (cap_apple < 0) {
		dev_err(anhi->dev, "Unable to find VSE Apple capability: %d\n",
			cap_apple);
		ret = cap_apple;
		goto err_unlock_tb_domain;
	}

	/*
	 * The ports are locked after reset. Unlock them before writing the cable information which
	 * will bring up the link and start the first scan such that XDomain responses are not
	 * rejected by our own router.
	 */
	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_null(port))
			continue;
		ret = tb_port_unlock(port);
		if (ret)
			dev_warn(anhi->dev, "Failed to unlock port %d: %d\n",
				 port->port, ret);
	}

	ret = tb_sw_write(anhi->tb->root_switch, &acio->target_cable_info, TB_CFG_SWITCH,
			  cap_apple + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	if (ret) {
		dev_warn(anhi->dev, "Setting VSE Apple cable info failed: %d\n", ret);
		goto err_unlock_tb_domain;
	}

	mutex_unlock(&anhi->tb->lock);

	WRITE_ONCE(acio->nhi_pdev, pdev);
	acio->nhi_boot_status = 0;
	complete(&acio->nhi_boot_completion);
	return 0;

err_unlock_tb_domain:
	mutex_unlock(&anhi->tb->lock);
	disable_delayed_work_sync(&anhi->dp_aux_work);
	apple_cio_stop_pcie_tunnel(acio);
	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
err:
	acio->nhi_boot_status = ret;
	complete(&acio->nhi_boot_completion);
	return ret;
}

static void apple_dpin_end_pm_gate(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	/* Domain removal stopped producers. No DP-IN/domain/fabric lock held. */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		cancel_delayed_work_sync(&c->retry);
		flush_work(&c->work);
		scoped_guard(mutex, &c->lock)
			apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_END_PM_GATE,
					!!c->regs, 0);
	}
}

static void apple_nhi_remove(struct platform_device *pdev)
{
	struct apple_nhi *anhi = platform_get_drvdata(pdev);

	/* Stop racing tunnel activation from rearming work during removal. */
	disable_delayed_work_sync(&anhi->dp_aux_work);
	apple_cio_stop_pcie_tunnel(anhi->acio);
	WRITE_ONCE(anhi->acio->nhi_pdev, NULL);
	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
	/* A removed prepared device may never receive complete(). */
	apple_dpin_end_pm_gate(anhi->acio);
}

/*
 * The Apple platform driver owns the NHI allocation and keeps an
 * apple_nhi pointer in driver data. The generic NHI PM callbacks cannot be
 * installed here because they expect driver data to contain struct tb.
 */
/*
 * A tunnel waiting for a display pipeline asks appledrm again every few
 * seconds.  Not while devices suspend and resume: stop asking before any of
 * them suspends, let an attempt in flight finish (a wait for appledrm to
 * load gives way, see apple_dpin_connect()), and ask again once all of them
 * have resumed. The tunnel keeps waiting meanwhile. Suspend-gated handoffs
 * are deferred until complete, including after an aborted suspend. Established
 * handed callbacks and synchronous tunnel teardown remain permitted.
 */
static void apple_dpin_pause_retries(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		scoped_guard(mutex, &c->lock)
			apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_PAUSE,
					!!c->regs, 0);
	}
	/* Close admission on both adapters before draining either one. */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		cancel_delayed_work_sync(&acio->dpin[i].retry);
		flush_work(&acio->dpin[i].work);
	}
}

static void apple_dpin_resume_retries(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];
		unsigned int actions;

		guard(mutex)(&c->lock);
		actions = apple_dpin_step(&c->state, acio->dp,
					  APPLE_DPIN_RESUME, !!c->regs, 0);
		if (actions & APPLE_DPIN_QUEUE)
			queue_work(acio->dp_wq, &c->work);
		if (actions & APPLE_DPIN_ARM_RETRY)
			mod_delayed_work(acio->dp_wq, &c->retry,
					 msecs_to_jiffies(APPLE_DPIN_RETRY_MS));
	}
}

static int apple_nhi_prepare(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);
	struct apple_cio *acio = anhi->acio;

	acio->resume_had_peer = false;
	/* Only resume recovery reads it: don't take the domain lock otherwise. */
	if (!READ_ONCE(resume_recovery))
		return 0;
	scoped_guard(mutex, &anhi->tb->lock) {
		struct tb_port *port;

		tb_switch_for_each_port(anhi->tb->root_switch, port) {
			if (tb_port_is_null(port) &&
			    ((port->xdomain && !port->xdomain->is_unplugged) ||
			     (port->remote && !port->remote->sw->is_unplugged)))
				acio->resume_had_peer = true;
		}
	}

	apple_dpin_pause_retries(acio);

	guard(mutex)(&acio->pcie_tunnel_lock);
	/* Keep an unresolved expectation across another sleep before revalidation. */
	acio->pcie_resume_expected |= READ_ONCE(acio->pcie_tunnel_requested);
	acio->pcie_pm_prepared = true;
	return 0;
}

/*
 * Router sleep saves power but ACIO retraining can stall after resume.
 * Keep it opt-in until remote-transport recovery is qualified on hardware.
 * An explicit Y also drops KEEP_TUNNELS and rebuilds tunnels on resume.
 */
static bool router_sleep;
module_param(router_sleep, bool, 0644);
MODULE_PARM_DESC(router_sleep,
		 "Ask routers to sleep on system suspend (default N; experimental, rebuilds tunnels)");

static bool apple_nhi_pcie_link_kept(struct apple_cio *acio)
{
	struct platform_device *pcie_pdev;
	bool kept = false;

	guard(mutex)(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (pcie_pdev) {
		kept = apple_pcie_tunnel_link_kept(&pcie_pdev->dev);
		put_device(&pcie_pdev->dev);
	}
	return kept;
}

static int apple_nhi_suspend_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	anhi->nhi.quirks &= ~(QUIRK_NO_SYSTEM_SLEEP | QUIRK_KEEP_TUNNELS);
	if (!READ_ONCE(router_sleep)) {
		anhi->nhi.quirks |= QUIRK_NO_SYSTEM_SLEEP;
		/* PCIe-C suspends first; its tunnel must then survive resume. */
		if (apple_nhi_pcie_link_kept(anhi->acio))
			anhi->nhi.quirks |= QUIRK_KEEP_TUNNELS;
	}

	return tb_domain_suspend_noirq(anhi->tb);
}

static int apple_nhi_resume_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	anhi->acio->resume_activity = atomic64_read(&anhi->tb->remote_activity);
	WRITE_ONCE(anhi->acio->resume_link_pending, true);
	return tb_domain_resume_noirq(anhi->tb);
}

static int apple_nhi_freeze_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_freeze_noirq(anhi->tb);
}

static int apple_nhi_thaw_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_thaw_noirq(anhi->tb);
}

static int apple_nhi_suspend(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_suspend(anhi->tb);
}

static void apple_nhi_complete(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);
	struct apple_cio *acio = anhi->acio;

	tb_domain_complete(anhi->tb);
	apple_dpin_resume_retries(acio);

	guard(mutex)(&acio->pcie_tunnel_lock);
	acio->pcie_pm_prepared = false;
	if (!acio->pcie_tunnel_stopping &&
	    (acio->pcie_quiesce_pending || READ_ONCE(acio->pcie_tunnel_requested)))
		mod_delayed_work(system_freezable_wq, &acio->pcie_tunnel_work, 0);
}

static const struct dev_pm_ops apple_nhi_pm_ops = {
	.prepare = apple_nhi_prepare,
	.suspend = apple_nhi_suspend,
	.suspend_noirq = apple_nhi_suspend_noirq,
	.resume_noirq = apple_nhi_resume_noirq,
	.freeze_noirq = apple_nhi_freeze_noirq,
	.thaw_noirq = apple_nhi_thaw_noirq,
	.restore_noirq = apple_nhi_resume_noirq,
	.poweroff = apple_nhi_suspend,
	.poweroff_noirq = apple_nhi_suspend_noirq,
	.complete = apple_nhi_complete,
};

static const struct of_device_id apple_nhi_match[] = {
	{
		.compatible = "apple,t8142-usb4-nhi",
	},
	{
		.compatible = "apple,t8103-usb4-nhi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_nhi_match);

static struct platform_driver apple_nhi_driver = {
	.driver = {
		.name = "thunderbolt-apple-nhi",
		.of_match_table = apple_nhi_match,
		.pm = &apple_nhi_pm_ops,
	},
	.probe = apple_nhi_probe,
	.remove = apple_nhi_remove,
};

static struct platform_device *apple_cio_find_pcie_tunnel(struct apple_cio *acio)
{
	struct platform_device *pdev = NULL;

	if (!acio->pcie_tunnel_np || !acio->pcie_tunnel_populated)
		return NULL;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!of_device_is_compatible(child, "apple,t8103-pciec") &&
		    !of_device_is_compatible(child, "apple,t6000-pciec"))
			continue;

		pdev = of_find_device_by_node(child);
		break;
	}

	return pdev;
}

static void apple_cio_remove_children(struct apple_cio *acio)
{
	struct platform_device *nhi_pdev;

	lockdep_assert_held(&acio->lock);

	/*
	 * Stop requests before removing the NHI. Do not hold pcie_tunnel_lock
	 * across domain removal: it takes tb->lock, which tunnel callbacks hold
	 * when they acquire pcie_tunnel_lock.
	 */
	apple_cio_stop_pcie_tunnel(acio);

	/*
	 * The NHI is the tunnel control plane. Stop it while the tunneled PCIe
	 * host and DART are still accessible, before depopulation tears down the
	 * data plane. Otherwise tb_domain_remove() walks the USB4 topology after
	 * PCIe-C has already reset its port and can wedge waiting on dead control
	 * traffic. The remaining children are still removed in reverse creation
	 * order below.
	 */
	nhi_pdev = READ_ONCE(acio->nhi_pdev);
	if (nhi_pdev)
		of_platform_device_destroy(&nhi_pdev->dev, NULL);
	/*
	 * NHI removal has released tb->lock and pcie_tunnel_lock. DP work and
	 * its DCP/crossbar/PHY callbacks never acquire acio->lock: they take
	 * only the DP-IN lock on this controller. Keep that invariant for
	 * future monitor/reconnect work; this drain holds acio->lock.
	 */
	if (acio->dp_wq)
		flush_workqueue(acio->dp_wq);

	of_platform_depopulate(acio->dev);
	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		acio->pcie_tunnel_populated = false;
		acio->pcie_quiesce_pending = false;
	}
}

/**
 * apple_cio_stop - Stop and power down the ACIO complex
 * @acio: ACIO complex to stop
 *
 * Remove the NHI and DART children before shutting down the co-processor and
 * dropping the power-domain links. The Type-C PHY must remain configured
 * until this function returns. The caller must hold @acio->lock.
 */
static void apple_cio_stop(struct apple_cio *acio)
{
	int ret, i;

	lockdep_assert_held(&acio->lock);
	if (!acio->current_cable_info)
		return;
	apple_cio_remove_children(acio);

	/* Try to shut down and power off the co-processor gracefully */
	ret = apple_rtkit_poweroff(acio->rtk);
	if (ret)
		dev_warn(acio->dev,
			 "Failed to shutdown M3 RTKit, continuing ACIO shutdown anyway\n");
	apple_rtkit_free(acio->rtk);
	acio->rtk = NULL;

	/* Finally, remove the links to the PD domains to power everything off */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], false);
	}

	acio->current_cable_info = 0;
	acio->resume_link_failed = false;
}

/* Only called while the ACIO power domains are on. */
static void apple_cio_log_state(struct apple_cio *acio, const char *when)
{
	if (acio->cpu_base)
		dev_info(acio->dev, "%s: LSTX 0x%08x, FW state 0x%08x, CPU ctrl 0x%08x, status 0x%08x\n",
			 when, readl(acio->rc_base + APPLE_CIO_M3_CTRL),
			 readl(acio->rc_base + APPLE_CIO_M3_STAT),
			 readl(acio->cpu_base + APPLE_MXWRAP_CPU_CTRL),
			 readl(acio->cpu_base + APPLE_MXWRAP_CPU_STATUS));
	else
		dev_info(acio->dev, "%s: LSTX 0x%08x, FW state 0x%08x\n", when,
			 readl(acio->rc_base + APPLE_CIO_M3_CTRL),
			 readl(acio->rc_base + APPLE_CIO_M3_STAT));
}

/*
 * Whether a forced cable cycle for resume recovery is under way. One that
 * CD321x did not take up within the window (it refused, or a suspend came
 * first) no longer counts, so a later unrelated start gets every attempt.
 * Called with acio->lock held.
 */
static bool apple_cio_recovering(struct apple_cio *acio)
{
	if (acio->resume_recovery_active &&
	    time_after(jiffies, acio->resume_recovery_until))
		acio->resume_recovery_active = false;
	return acio->resume_recovery_active;
}

/**
 * apple_cio_start - Power up and start the ACIO complex
 * @acio: ACIO complex to start
 *
 * The Type-C PHY must already be configured for USB4 or Thunderbolt. Power up
 * ACIO, boot its RTKit co-processor, populate its child devices and wait for
 * the NHI to register the USB4 domain. All completed steps are unwound on
 * failure. The caller must hold @acio->lock.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_start(struct apple_cio *acio)
{
	struct device_link *link;
	int i, ret, attempts;
	u32 state, val;

	lockdep_assert_held(&acio->lock);

	/*
	 * PCIe-C's power domain stays on across cable teardown, so apart from
	 * this reset nothing returns the controller to a known state between
	 * tunnels. After system sleep with a Thunderbolt 3 dock, state left in
	 * it has kept every later link from training until reboot. Reset it
	 * while its host and DART are unbound and before ACIO's domains come
	 * up, in the same order as after boot.
	 */
	if (acio->pcie_reset) {
		ret = reset_control_acquire(acio->pcie_reset);
		if (ret)
			return dev_err_probe(acio->dev, ret,
					     "PCIe-C reset is still in use\n");
		ret = reset_control_reset(acio->pcie_reset);
		reset_control_release(acio->pcie_reset);
		if (ret)
			return dev_err_probe(acio->dev, ret,
					     "failed to reset PCIe-C\n");
		dev_info(acio->dev, "PCIe-C reset before ACIO start\n");
	}

	/* Create device links to the power domains in order to power them on */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		link = device_link_add(acio->dev, acio->pd_list->pd_devs[i],
				       DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!link) {
			ret = -ENODEV;
			goto remove_links;
		}
		acio->pd_list->pd_links[i] = link;
		/*
		 * ACIO owns these domains explicitly for the lifetime of the cable
		 * connection.  The virtual genpd devices must not independently
		 * cycle them during system sleep: doing so destroys the live M3 and
		 * NHI state behind the still-bound child devices.  Cable teardown
		 * removes the runtime-active links and gives system PM ownership back.
		 */
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], true);
	}

	/*
	 * After the power domains are on we need to signal and wait for the ACIO block
	 * to actually start before we can bring up the co-processor.
	 */
	attempts = apple_cio_recovering(acio) ? 1 : APPLE_CIO_START_RETRIES;
	for (i = 0; i < attempts; i++) {
		ret = reset_control_deassert(acio->reset);
		if (!ret)
			break;
		if (i + 1 < attempts) {
			dev_warn(acio->dev,
				 "ACIO block failed to start (attempt %d/%d): %d, retrying\n",
				 i + 1, attempts, ret);
			msleep(APPLE_CIO_START_RETRY_DELAY_MS);
		}
	}
	if (ret) {
		dev_err(acio->dev, "ACIO block failed to start: %d\n", ret);
		goto remove_links;
	}

	/*
	 * Start and wait for the co-processor to boot.
	 *
	 * iBoot can leave other bits of this register set (0x3 has been observed
	 * on t6020 before the kernel touches it). The
	 * remaining fields are undocumented, so preserve them rather than clearing
	 * them with a bare store.
	 */
	if (acio->hw->lstx_explicit) {
		writel(APPLE_CIO_LSTX_CTRL_ENABLE | APPLE_CIO_LSTX_CTRL_LEVEL,
		       acio->rc_base + APPLE_CIO_M3_CTRL);
	} else if (!acio->hw->mxwrap) {
		val = readl(acio->rc_base + APPLE_CIO_M3_CTRL);
		writel(val | APPLE_CIO_M3_CTRL_START, acio->rc_base + APPLE_CIO_M3_CTRL);
	}

	if (acio->hw->mxwrap) {
		apple_cio_log_state(acio, "before RUN");
		val = readl(acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
		writel(val | APPLE_MXWRAP_CPU_CTRL_RUN,
		       acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
	}

	acio->rtk = apple_rtkit_init(acio->dev, acio, NULL, 0, &apple_cio_rtkit_ops);
	if (IS_ERR(acio->rtk)) {
		ret = PTR_ERR(acio->rtk);
		dev_err(acio->dev, "Failed to initialize RTKit: %d\n", ret);
		goto remove_links;
	}

	ret = apple_rtkit_boot(acio->rtk);
	if (ret) {
		dev_err(acio->dev, "M3 RTKit failed to boot: %d\n", ret);
		apple_cio_log_state(acio, "RTKit boot failed");
		goto err_free_rtkit;
	}

	state = 0;
	if (acio->hw->m3_stat_optional) {
		/*
		 * T8152: M3_STAT is not shown to exist, so do not wait for it; the
		 * RTKit boot above is the readiness check.
		 */
		ret = 0;
	} else if (acio->hw->strict_fw_ready) {
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 (state & APPLE_CIO_M3_STAT_CODE_VALID) ||
					 FIELD_GET(APPLE_CIO_M3_STAT_STATE, state) ==
					 APPLE_CIO_M3_STAT_STATE_READY,
					 100, 500000);
	} else {
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 acio->type7 ?
				 (state & APPLE_CIO_M3_STAT_STATE) == BIT(24) :
				 (state & APPLE_CIO_M3_STAT_STATE), 100, 500000);
	}
	if (!ret && acio->hw->strict_fw_ready && (state & APPLE_CIO_M3_STAT_CODE_VALID)) {
		dev_err(acio->dev, "M3 firmware assert, code 0x%06lx (state 0x%08x)\n",
			FIELD_GET(APPLE_CIO_M3_STAT_CODE, state), state);
		ret = -EIO;
	}
	if (ret < 0) {
		dev_err(acio->dev, "M3 firmware failed to get ready: %d\n", ret);
		apple_cio_log_state(acio, "firmware not ready");
		goto err_shutdown_rtkit;
	}
	if (acio->hw->mxwrap)
		apple_cio_log_state(acio, "firmware ready");

	apple_tunable_apply(acio->rc_base, acio->rc_tunable);
	if (acio->pcie_adapter_base && acio->pcie_adapter_tunable) {
		dev_info(acio->dev, "applying PCIe adapter tunable\n");
		apple_tunable_apply(acio->pcie_adapter_base,
				    acio->pcie_adapter_tunable);
	}

	/*
	 * Bring up devices which are part of ACIO and are now accessible by the main SoC
	 * and specifically wait for the NHI to be up to prevent concurrent shutdowns.
	 */
	reinit_completion(&acio->nhi_boot_completion);
	ret = of_platform_populate(acio->np, NULL, NULL, acio->dev);
	if (ret) {
		dev_err(acio->dev, "failed to populate children: %d\n", ret);
		goto err_depopulate;
	}

	if (!wait_for_completion_timeout(&acio->nhi_boot_completion,
					 msecs_to_jiffies(APPLE_CIO_NHI_BOOT_TIMEOUT_MS))) {
		dev_err(acio->dev, "Timed out waiting for the NHI to come up\n");
		ret = -ETIMEDOUT;
		goto err_depopulate;
	}
	if (acio->nhi_boot_status) {
		ret = acio->nhi_boot_status;
		goto err_depopulate;
	}

	acio->current_cable_info = acio->target_cable_info;
	return 0;

err_depopulate:
	apple_cio_remove_children(acio);
err_shutdown_rtkit:
	/* Ignore errors here since we're about to cut power to the entire block anyway */
	apple_rtkit_poweroff(acio->rtk);
err_free_rtkit:
	apple_rtkit_free(acio->rtk);
remove_links:
	acio->rtk = NULL;
	/* Cut power to reset the entire block  */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], false);
	}
	return ret;
}

static int apple_cio_check_connection(struct apple_cio *acio)
{
	struct platform_device *pdev;
	int ret;

	lockdep_assert_held(&acio->lock);
	/* Ask CD321x to cycle ACIO and the PHY using a fresh cable snapshot. */
	if (acio->resume_link_failed)
		return -ENOLINK;
	guard(mutex)(&acio->pcie_tunnel_lock);
	if (!READ_ONCE(acio->nhi_pdev) || acio->pcie_tunnel_stopping ||
	    apple_rtkit_is_crashed(acio->rtk))
		return -ENODEV;
	if (acio->pcie_pm_prepared || acio->pcie_quiesce_pending)
		return -EAGAIN;

	/* A display-only or unauthorized session need not have a PCIe tunnel. */
	if (!READ_ONCE(acio->pcie_tunnel_requested))
		return acio->pcie_resume_expected ? -ENOLINK : 0;
	if (!acio->pcie_tunnel_populated)
		return -EAGAIN;
	pdev = apple_cio_find_pcie_tunnel(acio);
	if (!pdev)
		return acio->pcie_bind_retries ? -EAGAIN : -ENODEV;
	ret = apple_pcie_tunnel_check_state(&pdev->dev);
	put_device(&pdev->dev);
	if (ret == -ENODEV && acio->pcie_bind_retries)
		return -EAGAIN;
	if (!ret)
		acio->pcie_resume_expected = false;
	return ret;
}

static int apple_cio_link_connected(struct apple_cio *acio)
{
	struct platform_device *pdev = READ_ONCE(acio->nhi_pdev);
	struct apple_nhi *anhi;
	struct tb_port *port;
	int verdict = -EAGAIN;
	u64 activity;

	lockdep_assert_held(&acio->lock);
	if (!pdev)
		return -EAGAIN;
	device_lock(&pdev->dev);
	anhi = platform_get_drvdata(pdev);
	if (!pdev->dev.driver || !anhi)
		goto unlock;

	scoped_guard(mutex, &anhi->tb->lock) {
		activity = atomic64_read(&anhi->tb->remote_activity);
		tb_switch_for_each_port(anhi->tb->root_switch, port) {
			if (!tb_port_is_null(port))
				continue;
			dev_dbg(acio->dev,
				"resume lane %u link %u: xdomain registered %d unplugged %d\n",
				port->port, port->link_nr,
				port->xdomain && device_is_registered(&port->xdomain->dev),
				port->xdomain && port->xdomain->is_unplugged);
		}
		/* ACIO exposes one physical cable per domain, including both lanes. */
		verdict = activity != acio->resume_activity ? 0 : -ENOLINK;
		dev_dbg(acio->dev, "resume discovery: activity %llu -> %llu, verdict %s\n",
			acio->resume_activity, activity,
			verdict ? "no fresh discovery (stale UP possible)" :
			"fresh remote exchange");
	}
unlock:
	device_unlock(&pdev->dev);
	return verdict;
}

static void apple_cio_resume_link_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(to_delayed_work(work),
						    struct apple_cio, resume_link_work);
	bool notify = false;
	int ret;

	scoped_guard(mutex, &acio->lock) {
		if (!READ_ONCE(resume_recovery) || acio->removing ||
		    !acio->current_cable_info || !acio->target_cable_info ||
		    acio->resume_recovery_failed)
			return;
		ret = apple_cio_link_connected(acio);
		if (!ret || ret == -EAGAIN)
			return;
		if (++acio->resume_link_failures < 3) {
			mod_delayed_work(system_freezable_wq, &acio->resume_link_work,
					 msecs_to_jiffies(APPLE_CIO_RESUME_LINK_CHECK_MS));
			return;
		}
		if (atomic_cmpxchg(&resume_recovery_used, 0, 1)) {
			dev_warn(acio->dev, "resume recovery budget already used; leaving cable alone\n");
			return;
		}
		acio->resume_link_failed = true;
		acio->resume_recovery_active = true;
		acio->resume_recovery_until = jiffies +
			msecs_to_jiffies(APPLE_CIO_RESUME_RECOVERY_WINDOW_MS);
		dev_info(acio->dev, "no fresh remote discovery after resume, revalidating cable once\n");
		notify = true;
	}

	/* CD321x may tear down the NHI: never notify under either driver lock. */
	if (notify)
		typec_thunderbolt_switch_notify(acio->tbt_switch);
}

static int apple_cio_prepare(struct device *dev)
{
	struct apple_cio *acio = dev_get_drvdata(dev);

	/* Drain before taking lock; the worker acquires it to check the link. */
	cancel_delayed_work_sync(&acio->resume_link_work);
	scoped_guard(mutex, &acio->lock) {
		WRITE_ONCE(acio->resume_link_pending, false);
		acio->resume_link_failed = false;
		/* A cycle CD321x did not take up is not carried into this sleep. */
		acio->resume_recovery_active = false;
	}
	return 0;
}

static void apple_cio_complete(struct device *dev)
{
	struct apple_cio *acio = dev_get_drvdata(dev);

	guard(mutex)(&acio->lock);
	if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED)
		acio->sleep_phase = APPLE_CIO_SLEEP_RESUMED;
	if (!READ_ONCE(acio->resume_link_pending))
		return;
	WRITE_ONCE(acio->resume_link_pending, false);
	if (!READ_ONCE(resume_recovery) || acio->removing ||
	    !acio->current_cable_info || !acio->resume_had_peer ||
	    acio->resume_recovery_failed)
		return;

	acio->resume_link_failures = 0;
	mod_delayed_work(system_freezable_wq, &acio->resume_link_work,
			 msecs_to_jiffies(APPLE_CIO_RESUME_LINK_WAIT_MS));
}

static const struct dev_pm_ops apple_cio_pm_ops = {
	.prepare = apple_cio_prepare,
	.complete = apple_cio_complete,
};

/**
 * apple_cio_tbt_switch_set - Handle a USB4/Thunderbolt cable transition
 * @sw: USB4/Thunderbolt switch
 * @data: New cable state and negotiated cable details
 *
 * Translate the cable details into the Apple host-router representation and
 * synchronously power ACIO up or down. The Type-C PD driver must configure
 * the PHY before an active transition and must request the inactive state
 * before shutting the PHY down.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_tbt_switch_set(struct typec_thunderbolt_switch_dev *sw,
				    const struct typec_thunderbolt_switch_data *data)
{
	struct apple_cio *acio = typec_thunderbolt_switch_get_drvdata(sw);

	guard(mutex)(&acio->lock);

	/* Removal turns the cable off itself; only refuse to start new work. */
	if (acio->removing)
		return data->state == TYPEC_THUNDERBOLT_SWITCH_OFF ? 0 : -ESHUTDOWN;

	dev_dbg(acio->dev, "set cable state: %d\n", data->state);

	switch (data->state) {
	case TYPEC_THUNDERBOLT_SWITCH_OFF:
		acio->target_cable_info = 0;
		break;
	case TYPEC_THUNDERBOLT_SWITCH_TBT:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3;
		if (data->tbt.cable_mode & TBT_CABLE_ACTIVE_PASSIVE) {
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
			if (!(data->tbt.cable_mode & TBT_CABLE_LINK_TRAINING))
				acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX;
		}
		/* bit 16 of the Device Discover Mode VDO is 1 for a legacy TBT2 adapter */
		if (TBT_ADAPTER(data->tbt.device_mode))
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER;
		if (TBT_CABLE_SPEED(data->tbt.cable_mode) == TBT_CABLE_10_AND_20GBPS)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev,
			"TBT cable: cable_mode 0x%x, device_mode 0x%x, enter_vdo 0x%x, orientation %d -> cable info 0x%x\n",
			data->tbt.cable_mode, data->tbt.device_mode,
			data->tbt.enter_vdo, data->orientation,
			acio->target_cable_info);
		break;
	case TYPEC_THUNDERBOLT_SWITCH_USB4:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		if (FIELD_GET(EUDO_CABLE_TYPE_MASK, data->usb4.eudo) != EUDO_CABLE_TYPE_PASSIVE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
		if (FIELD_GET(EUDO_CABLE_SPEED_MASK, data->usb4.eudo) >= EUDO_CABLE_SPEED_USB4_GEN3)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev, "USB4 cable: eudo 0x%x, orientation %d -> cable info 0x%x\n",
			data->usb4.eudo, data->orientation, acio->target_cable_info);
		break;
	}

	/* Until resume, record changes; CD321x reapplies fresh state afterwards. */
	if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED)
		return 0;
	if (acio->sleep_phase == APPLE_CIO_SLEEP_RESUMED)
		acio->sleep_phase = APPLE_CIO_SLEEP_OPEN;
	/*
	 * Only a successful start settles a restart owed from sleep: CD321x
	 * also sends OFF as an intermediate step of a reconnect. With the cable
	 * truly gone, the cost is one extra cable read per later sleep.
	 */
	if (acio->sleep_restart_owed && acio->target_cable_info)
		dev_info(acio->dev, "restarting port after system sleep\n");

	if (acio->target_cable_info == acio->current_cable_info)
		return acio->current_cable_info ? apple_cio_check_connection(acio) : 0;

	/*
	 * Changing live cable parameters requires an ACIO shutdown. Report the
	 * rejected transition so the caller does not cache the new mode as active.
	 */
	if (acio->current_cable_info && acio->target_cable_info) {
		dev_err(acio->dev,
			"Invalid cable transition from 0x%x to 0x%x, shutting down instead\n",
			acio->current_cable_info, acio->target_cable_info);
		acio->target_cable_info = 0;
		apple_cio_stop(acio);
		return -EINVAL;
	}

	/*
	 * Bring up or power down the ACIO complex
	 * current_cable_info will be updated in the start/stop functions
	 */
	if (acio->target_cable_info) {
		int ret;

		/* -EHOSTDOWN: CD321x stops retrying a port disabled for good. */
		if (acio->resume_recovery_failed)
			return -EHOSTDOWN;
		ret = apple_cio_start(acio);
		if (!ret)
			WRITE_ONCE(acio->sleep_restart_owed, false);
		if (ret && apple_cio_recovering(acio)) {
			acio->resume_recovery_failed = true;
			acio->target_cable_info = 0;
			dev_err(acio->dev,
				"resume recovery start failed: %d; port disabled\n", ret);
			ret = -EHOSTDOWN;
		}
		acio->resume_recovery_active = false;
		return ret;
	}

	apple_cio_stop(acio);
	return 0;
}

static void apple_cio_sleep_teardown(struct apple_cio *acio)
{
	cancel_delayed_work_sync(&acio->resume_link_work);

	guard(mutex)(&acio->lock);
	if (acio->removing || acio->resume_recovery_failed || !acio->current_cable_info)
		return;

	/*
	 * PCIe endpoints may unregister PM notifiers on removal, which would
	 * deadlock inside this notifier chain, and storage must not disappear
	 * under running applications. Skip ports with PCIe tunnels, and close
	 * PCIe admission in the same critical section; the next NHI probe
	 * reopens it.
	 */
	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		if (READ_ONCE(acio->pcie_tunnel_requested) || acio->pcie_tunnel_populated)
			return;
		acio->pcie_tunnel_stopping = true;
	}

	dev_info(acio->dev, "powering off port for system sleep (cable info 0x%x)\n",
		 acio->current_cable_info);
	apple_cio_stop(acio);
	acio->sleep_phase = APPLE_CIO_SLEEP_CLOSED;
	WRITE_ONCE(acio->sleep_restart_owed, true);
}

/* A restart still owed from the last sleep must not race this suspend. */
static void apple_cio_sleep_close_owed(struct apple_cio *acio)
{
	guard(mutex)(&acio->lock);
	if (acio->sleep_restart_owed && !acio->current_cable_info)
		acio->sleep_phase = APPLE_CIO_SLEEP_CLOSED;
	acio->sleep_aborted_early = false;
}

/*
 * Runs when no post-resume revalidation has restarted the port: always
 * after an aborted suspend, and as a fallback after a real one. Ask CD321x
 * for a fresh read instead of starting from the state recorded before sleep.
 */
static void apple_cio_sleep_restart_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(to_delayed_work(work), struct apple_cio,
					      sleep_restart_work);

	scoped_guard(mutex, &acio->lock) {
		acio->sleep_aborted_early = false;
		if (!acio->sleep_restart_owed)
			return;
		acio->sleep_phase = APPLE_CIO_SLEEP_OPEN;
		if (acio->removing || acio->resume_recovery_failed || acio->current_cable_info) {
			WRITE_ONCE(acio->sleep_restart_owed, false);
			return;
		}
		dev_info(acio->dev, "requesting cable restore after system sleep\n");
	}

	/* Never notify under acio->lock: CD321x calls back into .set. */
	typec_thunderbolt_switch_notify_restore(acio->tbt_switch);
}

static int apple_cio_pm_notify(struct notifier_block *nb, unsigned long event,
			       void *unused)
{
	struct apple_cio *acio = container_of(nb, struct apple_cio, pm_notifier);
	bool owed, early;

	/* Teardown off and no restart owed from an earlier sleep: nothing to do. */
	if (!READ_ONCE(s2idle_teardown) && !READ_ONCE(acio->sleep_restart_owed))
		return NOTIFY_DONE;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* Nothing from an earlier sleep may restart the port during this one. */
		cancel_delayed_work_sync(&acio->sleep_restart_work);
		apple_cio_sleep_close_owed(acio);
		/* Before tasks freeze: tearing down the NHI needs unfrozen work. */
		if (READ_ONCE(s2idle_teardown))
			apple_cio_sleep_teardown(acio);
		break;
	case PM_POST_SUSPEND:
		scoped_guard(mutex, &acio->lock) {
			/* Aborted before device suspend: complete never ran. */
			if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED) {
				acio->sleep_phase = APPLE_CIO_SLEEP_RESUMED;
				acio->sleep_aborted_early = true;
			}
			owed = acio->sleep_restart_owed;
			early = acio->sleep_aborted_early;
		}
		/* Start from a worker: probing children inside this chain can deadlock. */
		if (owed)
			queue_delayed_work(system_dfl_wq, &acio->sleep_restart_work,
					   early ? 0 :
					   msecs_to_jiffies(APPLE_CIO_SLEEP_RESTART_FALLBACK_MS));
		break;
	}
	return NOTIFY_DONE;
}

static int apple_cio_pcie_notify(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct apple_cio *acio = container_of(nb, struct apple_cio, pcie_notifier);

	if (event != APPLE_PCIE_TUNNEL_LINK_DOWN || data != acio->dev)
		return NOTIFY_DONE;

	typec_thunderbolt_switch_notify(acio->tbt_switch);
	return NOTIFY_OK;
}

static int apple_cio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_cio *acio;
	int ret;

	acio = devm_kzalloc(dev, sizeof(*acio), GFP_KERNEL);
	if (!acio)
		return -ENOMEM;
	platform_set_drvdata(pdev, acio);

	ret = devm_mutex_init(dev, &acio->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &acio->pcie_tunnel_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&acio->pcie_tunnel_work,
			  apple_cio_pcie_tunnel_work);
	INIT_DELAYED_WORK(&acio->resume_link_work, apple_cio_resume_link_work);
	INIT_DELAYED_WORK(&acio->sleep_restart_work, apple_cio_sleep_restart_work);
	init_completion(&acio->nhi_boot_completion);
	acio->dev = &pdev->dev;
	acio->np = dev->of_node;
	acio->type7 = of_device_is_compatible(acio->np, "apple,t8142-usb4-acio");
	if (acio->type7 &&
	    of_property_count_u32_elems(acio->np, "apple,tunable-rc") <= 0)
		return dev_err_probe(dev, -EINVAL, "Missing bootloader RC tuning\n");

	acio->hw = of_device_get_match_data(dev);
	if (!acio->hw)
		return -EINVAL;

	if (acio->hw->mxwrap) {
		struct resource *cpu_res;

		cpu_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cpu");
		if (!cpu_res)
			return dev_err_probe(dev, -EINVAL, "Missing MxWrap cpu region\n");
		cpu_res->flags |= IORESOURCE_MEM_NONPOSTED;
		acio->cpu_base = devm_ioremap_resource(dev, cpu_res);
		if (IS_ERR(acio->cpu_base))
			return dev_err_probe(dev, PTR_ERR(acio->cpu_base),
					     "Unable to map MxWrap cpu regs\n");
	}
	acio->pcie_tunnel_np =
		of_parse_phandle(dev->of_node, "apple,pcie-tunnel", 0);
	if (acio->pcie_tunnel_np) {
		ret = devm_add_action_or_reset(dev, apple_cio_of_node_put,
					       acio->pcie_tunnel_np);
		if (ret)
			return ret;

		acio->pcie_tunnel_preinitialized =
			apple_cio_pcie_tunnel_is_preinitialized(
				acio->pcie_tunnel_np);
		if (!acio->pcie_tunnel_preinitialized) {
			dev_warn(dev,
				 "PCIe-C tunnel disabled: not initialized by m1n1 or the kernel\n");
		} else {
			ret = apple_cio_map_pcie_intr2axi(acio);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to map PCIe-C Intr2AXI registers\n");
			ret = apple_cio_get_pcie_reset(acio);
			if (ret)
				return ret;
		}
	}

	acio->sram_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!acio->sram_res)
		return dev_err_probe(dev, -EIO, "Failed to get SRAM resource\n");
	acio->sram_base = devm_ioremap_resource(dev, acio->sram_res);
	if (IS_ERR(acio->sram_base))
		return dev_err_probe(dev, PTR_ERR(acio->sram_base), "Failed to map SRAM\n");

	acio->rc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc");
	acio->rc_base = devm_ioremap_resource(&pdev->dev, acio->rc_res);
	if (IS_ERR(acio->rc_base))
		return dev_err_probe(dev, PTR_ERR(acio->rc_base), "Unable to map rc regs\n");
	acio->rc_tunable =
		devm_apple_tunable_parse(dev, acio->np, "apple,tunable-rc", acio->rc_res);
	if (IS_ERR(acio->rc_tunable))
		return dev_err_probe(dev, PTR_ERR(acio->rc_tunable), "Unable to load rc tunable\n");

	{
		struct resource *adapter;

		adapter = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "pcie-adapter");
		if (adapter) {
			adapter->flags |= IORESOURCE_MEM_NONPOSTED;
			acio->pcie_adapter_base = devm_ioremap_resource(dev, adapter);
			if (IS_ERR(acio->pcie_adapter_base))
				return dev_err_probe(dev, PTR_ERR(acio->pcie_adapter_base),
						     "Unable to map PCIe adapter regs\n");
			acio->pcie_adapter_tunable =
				devm_apple_tunable_parse(dev, acio->np,
							 "apple,tunable-pcie-adapter",
							 adapter);
			if (IS_ERR(acio->pcie_adapter_tunable)) {
				if (PTR_ERR(acio->pcie_adapter_tunable) == -ENOENT)
					acio->pcie_adapter_tunable = NULL;
				else
					return dev_err_probe(dev,
							PTR_ERR(acio->pcie_adapter_tunable),
							"Unable to load PCIe adapter tunable\n");
			}
		}
	}

	acio->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(acio->reset))
		return dev_err_probe(dev, PTR_ERR(acio->reset), "Unable to get CIO reset\n");

	/*
	 * If there is only a single domain listed in the device tree the platform driver
	 * framework will already attach it. Thus, if we find an already attached domain
	 * here something's wrong in the device tree because we expect at least three separate
	 * domains that we have to control manually.
	 */
	if (dev->pm_domain) {
		dev_err(dev, "PM domain already attached, check if the DT lists three domains\n");
		return -EINVAL;
	}

	/*
	 * Find and attach the PM domains but don't power them on yet since we must only
	 * do that after the PHY has already been configured into USB4/Thunderbolt mode.
	 */
	struct dev_pm_domain_attach_data pd_data = {
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	ret = devm_pm_domain_attach_list(dev, &pd_data, &acio->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Unable to attach PM domains\n");
	else if (ret < 3)
		return dev_err_probe(dev, -EINVAL, "Not enough PM domains\n");

	acio->connector_np = of_graph_get_remote_node(dev->of_node, 1, -1);
	acio->dp_dual_stream = apple_dp_tunnel_dual_stream(acio->connector_np);
	/* Register layout, root qualification and connector wiring are separate. */
	acio->dp = apple_dpin_policy_select(acio->hw->dp, of_root, acio->dp_dual_stream);
	if (acio->connector_np) {
		ret = devm_add_action_or_reset(dev, apple_cio_of_node_put,
					       acio->connector_np);
		if (ret)
			return ret;
		for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
			acio->dpin[i].acio = acio;
			acio->dpin[i].idx = i;
			INIT_WORK(&acio->dpin[i].work, apple_dpin_work_fn);
			INIT_DELAYED_WORK(&acio->dpin[i].retry, apple_dpin_retry_fn);
			ret = devm_mutex_init(dev, &acio->dpin[i].lock);
			if (ret)
				return ret;
		}
		/* Prepare/remove drain this queue; it must remain non-freezable. */
		acio->dp_wq = alloc_ordered_workqueue("%s-dp", 0, dev_name(dev));
		if (!acio->dp_wq)
			return -ENOMEM;
		ret = devm_add_action_or_reset(dev, apple_cio_destroy_wq, acio);
		if (ret)
			return ret;
	}

	/* And finally register the OOB notification for Thunderbolt/USB4 cables */
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = pdev->dev.fwnode,
		.set = apple_cio_tbt_switch_set,
		.drvdata = acio,
	};
	/* A consumer cannot start a host before its failure listener is ready. */
	scoped_guard(mutex, &acio->lock) {
		acio->tbt_switch = typec_thunderbolt_switch_register(dev, &desc);
		if (IS_ERR(acio->tbt_switch))
			return dev_err_probe(dev, PTR_ERR(acio->tbt_switch),
					     "Unable to register thunderbolt switch\n");

		acio->pcie_notifier.notifier_call = apple_cio_pcie_notify;
		ret = apple_pcie_tunnel_register_notifier(&acio->pcie_notifier);
		if (ret)
			acio->removing = true;
	}
	if (ret) {
		typec_thunderbolt_switch_unregister(acio->tbt_switch);
		return ret;
	}

	acio->pm_notifier.notifier_call = apple_cio_pm_notify;
	register_pm_notifier(&acio->pm_notifier);
	return 0;
}

static void apple_cio_remove(struct platform_device *pdev)
{
	struct apple_cio *acio = platform_get_drvdata(pdev);

	/* Refuse new activation before draining callbacks and workers. */
	scoped_guard(mutex, &acio->lock)
		acio->removing = true;

	apple_pcie_tunnel_unregister_notifier(&acio->pcie_notifier);
	unregister_pm_notifier(&acio->pm_notifier);
	disable_delayed_work_sync(&acio->sleep_restart_work);

	disable_delayed_work_sync(&acio->resume_link_work);
	/* The worker can notify this switch; stop current and future enqueues. */
	disable_delayed_work_sync(&acio->pcie_tunnel_work);
	/* A set may hold the switch read lock while waiting for acio->lock. */
	typec_thunderbolt_switch_unregister(acio->tbt_switch);
	guard(mutex)(&acio->lock);
	if (acio->current_cable_info)
		apple_cio_stop(acio);
}




static const struct of_device_id apple_acio_match[] = {
	{
		.compatible = "apple,t8142-usb4-acio",
		.data = &apple_cio_t8142_hw,
	},
	{
		.compatible = "apple,t6030-usb4-acio",
		.data = &apple_cio_t6030_hw,
	},
	{
		.compatible = "apple,t6020-usb4-acio",
		.data = &apple_cio_t6020_hw,
	},
	{
		.compatible = "apple,t8152-usb4-acio",
		.data = &apple_cio_t8152_hw,
	},
	{
		.compatible = "apple,t8103-usb4-acio",
		.data = &apple_cio_t8103_hw,
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_acio_match);

static struct platform_driver apple_cio_driver = {
	.driver = {
		.name = "thunderbolt-apple-acio",
		.of_match_table = apple_acio_match,
		.pm = pm_sleep_ptr(&apple_cio_pm_ops),
	},
	.probe = apple_cio_probe,
	.remove = apple_cio_remove,
};

static struct platform_driver * const apple_cio_drivers[] = {
	&apple_nhi_driver,
	&apple_cio_driver,
};

static int __init apple_cio_init(void)
{
	return platform_register_drivers(apple_cio_drivers,
					 ARRAY_SIZE(apple_cio_drivers));
}

static void __exit apple_cio_exit(void)
{
	platform_unregister_drivers(apple_cio_drivers,
				    ARRAY_SIZE(apple_cio_drivers));
}

module_init(apple_cio_init);
module_exit(apple_cio_exit);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple Silicon USB4/Thunderbolt driver");
