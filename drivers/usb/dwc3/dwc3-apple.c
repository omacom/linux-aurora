// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon DWC3 Glue driver
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on:
 *  - dwc3-qcom.c Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *  - dwc3-of-simple.c Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/apple-atc.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/reset.h>
#include <linux/soc/apple/sleep.h>
#include <linux/soc/apple/t8132.h>
#include <linux/soc/apple/tunable.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>

#include "dwc3-apple-state.h"
#include "glue.h"
#include "io.h"

/*
 * This platform requires a very specific sequence of operations to bring up dwc3 and its USB3 PHY:
 *
 * 1) The PHY itself has to be brought up; for this we need to know the mode (USB3,
 *    USB3+DisplayPort, USB4, etc) and the lane orientation. This happens through typec_mux_set.
 * 2) DWC3 has to be brought up but we must not touch the gadget area or start xhci yet.
 * 3) The PHY bring-up has to be finalized and dwc3's PIPE interface has to be switched to the
 *    USB3 PHY, this is done inside phy_set_mode.
 * 4) We can now initialize xhci or gadget mode.
 *
 * We can switch 1 and 2 but 3 has to happen after (1 and 2) and 4 has to happen after 3.
 *
 * And then to bring this all down again:
 *
 * 1) DWC3 has to exit host or gadget mode and must no longer touch those registers
 * 2) The PHY has to switch dwc3's PIPE interface back to the dummy backend
 * 3) The PHY itself can be shut down, this happens from typec_mux_set
 *
 * We also can't transition the PHY from one mode to another while dwc3 is up and running (this is
 * slightly wrong, some transitions are possible, others aren't but because we have no documentation
 * for this I'd rather play it safe).
 *
 * After both the PHY and dwc3 are initialized we will only ever see a single "new device connected"
 * event. If we just keep them running only the first device plugged in will ever work. XHCI's port
 * status register actually does show the correct state but no interrupt ever comes in. In gadget
 * mode we don't even get a USBDisconnected event and everything looks like there's still something
 * connected on the other end.
 * This can be partially explained because the USB2 D+/D- lines are connected through a stateful
 * eUSB2 repeater which in turn is controlled by a variant of the TI TPS6598x USB PD chip which
 * resets the repeater out-of-band everytime the CC lines are (dis)connected. This then requires a
 * PHY reset to make sure the PHY and the eUSB2 repeater state are synchronized again.
 *
 * And to make this all extra fun: If we get the order of some of this wrong either the port is just
 * broken until a phy+dwc3 reset, or it's broken until a full SoC reset (likely because we can't
 * reset some parts of the PHY), or some watchdog kicks in after a few seconds and forces a full SoC
 * reset (mostly seen this with USB4/Thunderbolt but there's clearly some watchdog that hates
 * invalid states).
 *
 * Hence there's really no good way to keep dwc3 fully up and running after we disconnect a cable
 * because then we can't shut down the PHY anymore. And if we kept the PHY running in whatever mode
 * it was until the next cable is connected we'd need to tear it all down and bring it back up again
 * anyway to detect and use the next device.
 *
 * Instead, we just shut down everything when a cable is disconnected and transition to
 * DWC3_APPLE_NO_CABLE.
 * During initial probe we don't have any information about the connected cable and can't bring up
 * the PHY properly and thus also can't fully bring up dwc3. Instead, we just keep everything off
 * and defer the first dwc3 probe until we get the first cable connected event. Until then we stay
 * in DWC3_APPLE_PROBE_PENDING.
 * Once a cable is connected we then keep track of the controller mode here by transitioning to
 * DWC3_APPLE_HOST or DWC3_APPLE_DEVICE.
 *
 * Some boards do not wire the controller's USB2 pairs to a connector but to a fixed USB2 hub on
 * the board, whose downstream ports serve the Type-C connectors (Apple J700). Such a controller is
 * a host whatever happens at the connectors and never sees a role change: it is brought up in
 * DWC3_APPLE_HOST at probe and stays there. The Type-C port controllers only drive the PHY's
 * Type-C mux for the SuperSpeed lanes, and the PHY follows those mode changes on its own. It keeps
 * to the order above for the SuperSpeed side: the USB core holds the controller's SuperSpeed root
 * port powered off for the PHY, port runtime PM included, from before the PIPE goes to the dummy
 * backend until the PIPE is back on the USB3 PHY. The USB2 root port and the hub on it stay up
 * throughout.
 *
 * Whenever the core is up, for either kind of port, it is kept running across system sleep, see
 * dwc3_apple_init(). In host mode xHCI then keeps running in suspend-to-idle when a USB device
 * may wake the system, as a halted xHC could not signal the wakeup.
 *
 * A deep sleep takes the controller's power domain away. Behind a fixed hub on T8140 the host
 * sleeps in place, as macOS's does: xHCI suspends and resumes as usual, and the PHY takes the
 * core through hibernation in between, so the hub and its devices stay. The core comes back
 * without its registers and is set up again in place. With its always-on side kept, xHCI
 * restores its saved state and the hub resumes from U3, with no reset; without it xHCI starts
 * over, and the devices reset-resume. Anywhere else, or when the PHY can't, the glue stops
 * everything from preparation work, the way a cable disconnect does, and brings it back
 * once the system has resumed. See dwc3_apple_pm_notify().
 *
 * Runtime PM (T6040 and T8132, see dwc3_apple_core_probe()): the glue holds a reference from
 * the reset deassert to the end of the shutdown, so with no cable the controller can
 * runtime-suspend and its power domain drop, its DARTs (which save and restore their state) and,
 * through the device link of its phys, the PHY with it. On T8132 a port that has not seen a cable
 * yet can suspend as well, with runtime PM held by the glue until the first core probe.
 *
 * enum dwc3_apple_state is defined in dwc3-apple-state.h.
 */

#define DWC3_APPLE_NR_TUNABLES 8
#define DWC3_APPLE_NR_DRD_REGS 3

/**
 * struct dwc3_apple - Apple-specific DWC3 USB controller
 * @dwc: Core DWC3 structure
 * @dev: Pointer to the device structure
 * @mmio_resource: Resource to be passed to dwc3_core_probe
 * @apple_regs: Apple-specific DWC3 registers
 * @reset: Reset control
 * @usb2_phy: USB2 PHY, set to device mode before the core is brought up; the xHCI root hub
 *	      sets host mode
 * @usb3_phy: USB3 PHY, configured after the core is brought up
 * @role_sw: USB role switch
 * @fixed_hub: A fixed hub sits on the controller's USB2 port, the controller stays in host mode
 * @host_only: The controller must never enter DWC3_APPLE_DEVICE, see dwc3_apple_host_only()
 * @lock: Mutex for synchronizing access
 * @state: Current state of the controller, see enum dwc3_apple_state and the comment above
 * @core_active: Core still owns initialized PHYs
 * @core_exit_pending: Event buffers have been cleaned for this shutdown
 * @shutting_down: Reject new roles after the shutdown consumer has stopped
 * @reset_pending: Controller reset must complete before another start
 * @usb2_power_off_pending: Core-owned USB2 references still to release
 * @usb3_power_off_pending: Core-owned USB3 references still to release
 * @idle_rpm: Let the controller runtime-suspend while the core is down
 * @rpm_held: The glue holds a runtime PM reference for the running core
 * @pre_core_rpm: Runtime PM is the glue's own until the first core bring-up
 * @dt_dis_u3_susphy: The device tree keeps USB3 PHY suspend off itself
 * @sleep_ddr: The host votes for DRAM in a sleep while it runs (apple,sleep-ddr)
 * @drd_regs: The controller's optional blocks outside dwc3-core (drd-cfg, drd-bulk, drd-ausbc),
 *	      mapped for their tunables
 * @tunables: Controller tunables from the device tree, see dwc3_apple_tunable_list
 */
enum dwc3_apple_sleep_phase {
	DWC3_APPLE_SLEEP_IDLE,
	DWC3_APPLE_SLEEP_PREPARING,
	DWC3_APPLE_SLEEP_PREPARED,
	DWC3_APPLE_SLEEP_FINISHING,
	DWC3_APPLE_SLEEP_REMOVING,
};

struct dwc3_apple {
	struct dwc3 dwc;

	struct device *dev;
	struct resource *mmio_resource;
	void __iomem *apple_regs;

	struct reset_control *reset;
	struct phy *usb2_phy;
	struct phy *usb3_phy;
	struct usb_role_switch *role_sw;
	bool fixed_hub;
	/* The root port the fixed hub sits on, its node's reg. */
	u32 hub_port;
	bool host_only;

	struct mutex lock;

	enum dwc3_apple_state state;
	bool core_active;
	bool core_exit_pending;
	bool shutting_down;
	bool reset_pending;
	unsigned long usb2_power_off_pending;
	unsigned long usb3_power_off_pending;
	bool idle_rpm;
	bool rpm_held;
	bool pre_core_rpm;
	bool dt_dis_u3_susphy;
	bool sleep_ddr;
	struct notifier_block pm_nb;
	/* Kept through a deep sleep, with the PHY's hibernation, and the GUID it had. */
	bool sleep_in_place;
	u32 sleep_guid;
	/* The core lost its registers in that sleep and was set up again in place. */
	bool sleep_reinit;
	/* Looks for the fixed hub once a host restarted after a sleep has had time to find it. */
	struct delayed_work hub_check;
	/* The core didn't come back from a sleep or a power cycle: nothing may touch it. */
	bool sleep_dead;
	/* Stopped for a deep sleep: the state to come back to, and the wakeup setting. */
	bool sleep_stopped;
	bool sleep_wakeup;
	enum dwc3_apple_state sleep_state;
	/*
	 * The notifier only admits an epoch or queues its finish. Neither callback waits
	 * for the glue lock or for work that can disconnect a USB device. Preparation and
	 * recovery run from work; device prepare waits after the notifier chain has ended.
	 */
	/* Protects epoch transitions and desired syscore ownership, never hardware work. */
	spinlock_t sleep_pm_lock;
	enum dwc3_apple_sleep_phase sleep_phase;
	/* Desired syscore ownership, held off only while device prepare must run. */
	bool sleep_syscore;
	bool sleep_deep;
	bool sleep_pm_online;
	bool sleep_role_pending;
	enum dwc3_apple_state sleep_role;
	struct work_struct sleep_stop_work;
	struct completion sleep_stop_done;
	int sleep_stop_ret;
	ktime_t sleep_stop_start, sleep_stop_end;
	struct work_struct sleep_start_work;
	/* The start after a sleep may find its Type-C PHY not started yet: the role switch follows. */
	bool sleep_phy_wait;
	/* The start after a sleep was left to the role switch, which may also come too early. */
	bool sleep_phy_pending;
	/*
	 * J714s: a host on a USB4 router's USB3 tunnel, kept suspended in place through the
	 * router's sleep (see dwc3_apple_tunnel_keep()): the PHY's result once taken, the role the
	 * port controller asked for meanwhile, and the restart that follows a parked PIPE.
	 */
	bool sleep_tunnel;
	bool sleep_tunnel_taken;
	int sleep_tunnel_result;
	enum usb_role sleep_tunnel_role;
	struct work_struct sleep_tunnel_work;

	void __iomem *drd_regs[DWC3_APPLE_NR_DRD_REGS];
	struct apple_tunable *tunables[DWC3_APPLE_NR_TUNABLES];
};

#define to_dwc3_apple(d) container_of((d), struct dwc3_apple, dwc)

/* Callers hold the glue lock; the notifier changes only the epoch under sleep_pm_lock. */
static void dwc3_apple_syscore(struct dwc3_apple *appledwc, bool enable)
{
	unsigned long flags;

	lockdep_assert_held(&appledwc->lock);
	spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
	appledwc->sleep_syscore = enable;
	dev_pm_syscore_device(appledwc->dev,
			      enable && appledwc->sleep_phase != DWC3_APPLE_SLEEP_PREPARING);
	spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
}

/*
 * Apple Silicon dwc3 vendor-specific registers
 *
 * These registers were identified by tracing XNU's memory access patterns and correlating them with
 * debug output over serial to determine their names. We don't exactly know what these do but
 * without these USB3 devices sometimes don't work.
 */
#define APPLE_DWC3_REGS_START 0xcd00
#define APPLE_DWC3_REGS_END 0xcdff

#define APPLE_DWC3_CIO_LFPS_OFFSET 0xcd38
#define APPLE_DWC3_CIO_LFPS_OFFSET_VALUE 0xf800f80

#define APPLE_DWC3_CIO_BW_NGT_OFFSET 0xcd3c
#define APPLE_DWC3_CIO_BW_NGT_OFFSET_VALUE 0xfc00fc0

#define APPLE_DWC3_CIO_LINK_TIMER 0xcd40
#define APPLE_DWC3_CIO_PENDING_HP_TIMER GENMASK(23, 16)
#define APPLE_DWC3_CIO_PENDING_HP_TIMER_VALUE 0x14
#define APPLE_DWC3_CIO_PM_LC_TIMER GENMASK(15, 8)
#define APPLE_DWC3_CIO_PM_LC_TIMER_VALUE 0xa
#define APPLE_DWC3_CIO_PM_ENTRY_TIMER GENMASK(7, 0)
#define APPLE_DWC3_CIO_PM_ENTRY_TIMER_VALUE 0x10

static inline void dwc3_apple_writel(struct dwc3_apple *appledwc, u32 offset, u32 value)
{
	writel(value, appledwc->apple_regs + offset - APPLE_DWC3_REGS_START);
}

static inline u32 dwc3_apple_readl(struct dwc3_apple *appledwc, u32 offset)
{
	return readl(appledwc->apple_regs + offset - APPLE_DWC3_REGS_START);
}

static inline void dwc3_apple_mask(struct dwc3_apple *appledwc, u32 offset, u32 mask, u32 value)
{
	u32 reg;

	reg = dwc3_apple_readl(appledwc, offset);
	reg &= ~mask;
	reg |= value;
	dwc3_apple_writel(appledwc, offset, reg);
}

/*
 * macOS's T8140 controller reset sets GCTL's hibernation enable and a power-down scale
 * of 0xd. Without the enable the core's power management accepts a
 * D0/D3 request but saves and restores nothing, so the PHY's hibernation for a sleep
 * needs it.
 */
#define DWC3_APPLE_T8140_PWRDNSCALE 0xd

static void dwc3_apple_setup_hibernation(struct dwc3_apple *appledwc)
{
	struct dwc3 *dwc = &appledwc->dwc;
	u32 reg;

	if (!of_device_is_compatible(appledwc->dev->of_node, "apple,t8140-dwc3"))
		return;
	reg = dwc3_readl(dwc, DWC3_GCTL);
	reg &= ~DWC3_GCTL_PWRDNSCALE_MASK;
	reg |= DWC3_GCTL_PWRDNSCALE(DWC3_APPLE_T8140_PWRDNSCALE) | DWC3_GCTL_GBLHIBERNATIONEN;
	dwc3_writel(dwc, DWC3_GCTL, reg);
}

static void dwc3_apple_setup_cio(struct dwc3_apple *appledwc)
{
	dwc3_apple_writel(appledwc, APPLE_DWC3_CIO_LFPS_OFFSET, APPLE_DWC3_CIO_LFPS_OFFSET_VALUE);
	dwc3_apple_writel(appledwc, APPLE_DWC3_CIO_BW_NGT_OFFSET,
			  APPLE_DWC3_CIO_BW_NGT_OFFSET_VALUE);
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PENDING_HP_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PENDING_HP_TIMER,
				   APPLE_DWC3_CIO_PENDING_HP_TIMER_VALUE));
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PM_LC_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PM_LC_TIMER, APPLE_DWC3_CIO_PM_LC_TIMER_VALUE));
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PM_ENTRY_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PM_ENTRY_TIMER,
				   APPLE_DWC3_CIO_PM_ENTRY_TIMER_VALUE));
}

/*
 * The controller tunables of the ADT (macOS applyAllTunables after its controller reset), by
 * the register block their offsets are relative to. The core's soft reset undoes them, so they
 * are applied after every core init.
 */
enum dwc3_apple_tunable_block {
	DWC3_APPLE_TUNABLE_GLOBALS,	/* DWC3_GLOBALS_REGS_START */
	DWC3_APPLE_TUNABLE_LINK,	/* dwc3-core + 0xd000 */
	DWC3_APPLE_TUNABLE_DBG,		/* dwc3-core + 0xd800 */
	DWC3_APPLE_TUNABLE_CIO,		/* dwc3-core + 0xcd20, in dwc3-apple */
	/* The optional blocks outside dwc3-core, in drd_regs[] order */
	DWC3_APPLE_TUNABLE_DRD_CFG,	/* drd-cfg */
	DWC3_APPLE_TUNABLE_DRD_BULK,	/* drd-bulk */
	DWC3_APPLE_TUNABLE_DRD_AUSBC,	/* drd-ausbc */
	DWC3_APPLE_NR_TUNABLE_BLOCKS
};

static const char * const dwc3_apple_drd_reg_names[DWC3_APPLE_NR_DRD_REGS] = {
	"drd-cfg", "drd-bulk", "drd-ausbc",
};

static_assert(DWC3_APPLE_NR_TUNABLE_BLOCKS - DWC3_APPLE_TUNABLE_DRD_CFG == DWC3_APPLE_NR_DRD_REGS);

#define DWC3_APPLE_LINK_REGS_START	0xd000
#define DWC3_APPLE_DBG_REGS_START	0xd800
#define DWC3_APPLE_LINK_DBG_SIZE	0x800
/* The CIO registers, the fixed ones of dwc3_apple_setup_cio() included */
#define DWC3_APPLE_CIO_REGS_START	0xcd20

/*
 * In the order they are applied. A table is optional, and one for a block that the device tree
 * does not describe is refused. The CIO table follows dwc3_apple_setup_cio() and overrides its
 * fixed values (T6050).
 */
static const struct {
	const char *prop;
	enum dwc3_apple_tunable_block block;
} dwc3_apple_tunable_list[] = {
	{ "apple,tunable-bulk", DWC3_APPLE_TUNABLE_DRD_BULK },
	{ "apple,tunable-gbl-default", DWC3_APPLE_TUNABLE_GLOBALS },
	{ "apple,tunable-gbl-host", DWC3_APPLE_TUNABLE_GLOBALS },
	{ "apple,tunable-cio", DWC3_APPLE_TUNABLE_CIO },
	{ "apple,tunable-dbg", DWC3_APPLE_TUNABLE_DBG },
	{ "apple,tunable-cfg-host", DWC3_APPLE_TUNABLE_DRD_CFG },
	{ "apple,tunable-link", DWC3_APPLE_TUNABLE_LINK },
	{ "apple,tunable-ausbc", DWC3_APPLE_TUNABLE_DRD_AUSBC },
};

static_assert(ARRAY_SIZE(dwc3_apple_tunable_list) == DWC3_APPLE_NR_TUNABLES);

static void __iomem *dwc3_apple_tunable_base(struct dwc3_apple *appledwc,
					     enum dwc3_apple_tunable_block block)
{
	switch (block) {
	case DWC3_APPLE_TUNABLE_GLOBALS:
		return appledwc->dwc.regs;
	case DWC3_APPLE_TUNABLE_LINK:
		return appledwc->apple_regs + DWC3_APPLE_LINK_REGS_START - APPLE_DWC3_REGS_START;
	case DWC3_APPLE_TUNABLE_DBG:
		return appledwc->apple_regs + DWC3_APPLE_DBG_REGS_START - APPLE_DWC3_REGS_START;
	case DWC3_APPLE_TUNABLE_CIO:
		return appledwc->apple_regs + DWC3_APPLE_CIO_REGS_START - APPLE_DWC3_REGS_START;
	case DWC3_APPLE_TUNABLE_DRD_CFG:
	case DWC3_APPLE_TUNABLE_DRD_BULK:
	case DWC3_APPLE_TUNABLE_DRD_AUSBC:
		return appledwc->drd_regs[block - DWC3_APPLE_TUNABLE_DRD_CFG];
	case DWC3_APPLE_NR_TUNABLE_BLOCKS:
		break;
	}
	return NULL;
}

static void dwc3_apple_apply_tunables(struct dwc3_apple *appledwc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dwc3_apple_tunable_list); i++)
		apple_tunable_apply(dwc3_apple_tunable_base(appledwc,
							    dwc3_apple_tunable_list[i].block),
				    appledwc->tunables[i]);
}

/* The tunables are optional; a malformed one fails the probe. */
static int dwc3_apple_parse_tunables(struct dwc3_apple *appledwc, struct platform_device *pdev)
{
	struct resource *core = appledwc->mmio_resource;
	/* A block the device tree does not describe stays empty, and its table is refused. */
	struct resource blocks[DWC3_APPLE_NR_TUNABLE_BLOCKS] = {
		[DWC3_APPLE_TUNABLE_GLOBALS] = DEFINE_RES_MEM(core->start + DWC3_GLOBALS_REGS_START,
							      DWC3_GLOBALS_REGS_END -
							      DWC3_GLOBALS_REGS_START + 1),
		[DWC3_APPLE_TUNABLE_LINK] = DEFINE_RES_MEM(core->start + DWC3_APPLE_LINK_REGS_START,
							   DWC3_APPLE_LINK_DBG_SIZE),
		[DWC3_APPLE_TUNABLE_DBG] = DEFINE_RES_MEM(core->start + DWC3_APPLE_DBG_REGS_START,
							  DWC3_APPLE_LINK_DBG_SIZE),
		[DWC3_APPLE_TUNABLE_CIO] = DEFINE_RES_MEM(core->start + DWC3_APPLE_CIO_REGS_START,
							  APPLE_DWC3_REGS_END -
							  DWC3_APPLE_CIO_REGS_START + 1),
	};
	struct device *dev = appledwc->dev;
	int i;

	for (i = 0; i < DWC3_APPLE_NR_DRD_REGS; i++) {
		struct resource *res;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   dwc3_apple_drd_reg_names[i]);
		if (!res)
			continue;
		appledwc->drd_regs[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(appledwc->drd_regs[i]))
			return PTR_ERR(appledwc->drd_regs[i]);
		blocks[DWC3_APPLE_TUNABLE_DRD_CFG + i] = *res;
	}

	for (i = 0; i < ARRAY_SIZE(dwc3_apple_tunable_list); i++) {
		const char *prop = dwc3_apple_tunable_list[i].prop;
		struct apple_tunable *t;

		if (!of_property_present(dev->of_node, prop))
			continue;
		t = devm_apple_tunable_parse(dev, dev->of_node, prop,
					     &blocks[dwc3_apple_tunable_list[i].block]);
		if (IS_ERR(t))
			return dev_err_probe(dev, PTR_ERR(t), "Invalid %s\n", prop);
		appledwc->tunables[i] = t;
	}
	return 0;
}

static void dwc3_apple_set_ptrcap(struct dwc3_apple *appledwc, u32 mode)
{
	guard(spinlock_irqsave)(&appledwc->dwc.lock);
	dwc3_set_prtcap(&appledwc->dwc, mode, false);
}

/* Written to the core's GUID scratch register before a sleep in place, read back after it. */
#define DWC3_APPLE_SLEEP_GUID 0x534c5045

/* macOS waits up to 100 ms for this after the core's D0, and only logs a timeout. */
#define DWC3_APPLE_CFG_STATUS 0x2c
#define DWC3_APPLE_CFG_STATUS_PIPE_PHY_READY BIT(30)
#define DWC3_APPLE_PIPE_PHY_READY_US 100000

/*
 * The core is powered but its registers are at their reset values. Set it up again the way
 * dwc3_apple_core_start() and dwc3_apple_set_role() do, in place: the PHYs and the xHCI
 * device stay, so xHCI can restore its saved state, or start over. Runs in the resume,
 * while nothing else touches the core.
 */
static int dwc3_apple_core_reinit(struct dwc3_apple *appledwc)
{
	struct dwc3 *dwc = &appledwc->dwc;
	int i, ret;

	ret = dwc3_core_init(dwc);
	if (ret)
		return ret;
	/* dwc3_core_init() took PHY references the core holds already. */
	for (i = 0; i < dwc->num_usb3_ports; i++) {
		phy_power_off(dwc->usb3_generic_phy[i]);
		phy_exit(dwc->usb3_generic_phy[i]);
	}
	for (i = 0; i < dwc->num_usb2_ports; i++) {
		phy_power_off(dwc->usb2_generic_phy[i]);
		phy_exit(dwc->usb2_generic_phy[i]);
	}

	dwc3_apple_setup_cio(appledwc);
	dwc3_apple_apply_tunables(appledwc);
	dwc3_apple_setup_hibernation(appledwc);
	dwc3_apple_set_ptrcap(appledwc, DWC3_GCTL_PRTCAP_HOST);
	dwc->dis_u3_susphy_quirk = appledwc->dt_dis_u3_susphy;
	dwc3_enable_susphy(dwc, true);
	/* A PHY that came back in a USB3 mode routes the PIPE to it now, before xHCI starts. */
	ret = phy_set_mode(appledwc->usb3_phy, PHY_MODE_USB_HOST);
	if (ret)
		dev_warn(appledwc->dev, "USB3 PHY setup failed after the sleep, USB2 only, err=%d\n",
			 ret);

	return 0;
}

/* TEST (J714s): set up again in place by the resume, after the PHY lost it in the sleep. */
#define DWC3_APPLE_TUNNEL_REINIT	3

/* TEST (J714s): the T6050 PHY's call for a host on its router's USB3 tunnel, see apple-atc.h. */
static int dwc3_apple_tunnel_phy(struct dwc3_apple *appledwc, enum apple_t6050_cio_usb step)
{
	int (*fn)(struct device *dev, enum apple_t6050_cio_usb step);
	int ret;

	if (!appledwc->usb3_phy || !appledwc->usb3_phy->dev.parent)
		return -ENODEV;
	fn = symbol_get(apple_t6050_phy_cio_usb);
	if (!fn)
		return -ENODEV;
	ret = fn(appledwc->usb3_phy->dev.parent, step);
	symbol_put(apple_t6050_phy_cio_usb);
	return ret;
}

/*
 * TEST (J714s): before xHCI touches a core kept through its router's sleep. Kept on the tunnel,
 * or only parked after a refusal, the core has its registers and xHCI restores its state. One
 * that the PHY lost in the firmware sleep is powered again without its registers and held in
 * reset: out of reset and set up again before any access, then xHCI starts over (a positive
 * return), with the PIPE on the tunnel the router's wake gave back, or on its dummy producer
 * if the dock did not come back.
 */
static int dwc3_apple_tunnel_resume(struct dwc3_apple *appledwc)
{
	int ret;

	guard(mutex)(&appledwc->lock);
	if (!appledwc->sleep_tunnel_taken) {
		appledwc->sleep_tunnel_taken = true;
		appledwc->sleep_tunnel_result =
			dwc3_apple_tunnel_phy(appledwc, APPLE_T6050_CIO_USB_RESULT);
		/* TEST (sv6t11) */
		dev_info(appledwc->dev, "TEST sv6t11: USB host kept through the router's sleep, before xHCI resumes: PHY result %d (0 kept on the tunnel, 1 parked with its registers, 2 lost in the firmware sleep), firmware sleep %s\n",
			 appledwc->sleep_tunnel_result, pm_resume_via_firmware() ? "yes" : "no");
	}
	ret = appledwc->sleep_tunnel_result;
	if (ret == APPLE_T6050_CIO_USB_KEPT || ret == APPLE_T6050_CIO_USB_PARKED)
		return 0;
	/* Lost, or not known: nothing touches the core before it is out of reset. */
	appledwc->sleep_tunnel_result = APPLE_T6050_CIO_USB_LOST;
	ret = reset_control_deassert(appledwc->reset);
	if (!ret)
		ret = dwc3_apple_core_reinit(appledwc);
	if (ret) {
		dev_err(appledwc->dev, "not resuming xHCI after the router's sleep, the core is not set up: %d\n",
			ret);
		return ret;
	}
	appledwc->sleep_tunnel_result = DWC3_APPLE_TUNNEL_REINIT;
	dev_info(appledwc->dev, "USB host's core lost in the router's sleep: set up again, xHCI starts over\n");
	return 1;
}

/*
 * Runs before xHCI touches the core after a sleep in place. A core that may be unpowered must
 * be left alone, as an access could be an SError: dwc3_apple_sleep_kept() deals with it once
 * the system is back. A core that came back without its registers is set up again. xHCI then
 * restores its saved state, as macOS's does, unless the always-on side lost power too: then
 * it's told it lost its state (a positive return). A sleep refused before the hibernation
 * left the core as it was.
 */
static int dwc3_apple_pre_host_resume(struct dwc3 *dwc)
{
	struct dwc3_apple *appledwc = to_dwc3_apple(dwc);
	bool lost;
	u32 val;
	int ret;

	if (READ_ONCE(appledwc->sleep_tunnel))
		return dwc3_apple_tunnel_resume(appledwc);
	if (!READ_ONCE(appledwc->sleep_in_place))
		return 0;

	ret = apple_atcphy_sleep_result(appledwc->usb2_phy);
	lost = ret == APPLE_ATCPHY_SLEEP_POWER_LOST;
	if (ret == -EBUSY || lost)
		ret = 0;
	if (!ret && appledwc->drd_regs[0] &&	/* drd-cfg */
	    readl_poll_timeout(appledwc->drd_regs[0] + DWC3_APPLE_CFG_STATUS, val,
			       val & DWC3_APPLE_CFG_STATUS_PIPE_PHY_READY, 100,
			       DWC3_APPLE_PIPE_PHY_READY_US))
		dev_warn(appledwc->dev, "PIPE PHY not ready after the sleep: %#x\n", val);
	if (!ret && dwc3_readl(dwc, DWC3_GUID) != DWC3_APPLE_SLEEP_GUID) {
		ret = dwc3_apple_core_reinit(appledwc);
		if (!ret && !lost) {
			appledwc->sleep_reinit = true;
			dev_dbg(appledwc->dev,
				"core lost its registers in the sleep: set up again, xHCI restores its state\n");
			return 0;
		}
		if (!ret) {
			appledwc->sleep_reinit = true;
			dev_dbg(appledwc->dev,
				"core and its always-on side lost power in the sleep: set up again, xHCI starts over\n");
			return 1;
		}
	}
	if (ret)
		dev_err(appledwc->dev, "not resuming xHCI, the core didn't come back: %d\n", ret);

	return ret;
}

static const struct dwc3_glue_ops dwc3_apple_glue_ops = {
	.pre_host_resume = dwc3_apple_pre_host_resume,
};

static bool dwc3_apple_host_before_core_init(struct dwc3_apple *appledwc,
					     enum dwc3_apple_state state)
{
	return state == DWC3_APPLE_HOST && apple_t8132_usb_host_first() &&
		of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3");
}

static int dwc3_apple_core_probe(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	struct dwc3_probe_data probe_data = {};
	void *group;
	int ret;

	lockdep_assert_held(&appledwc->lock);
	WARN_ON_ONCE(appledwc->state != DWC3_APPLE_PROBE_PENDING);

	appledwc->dwc.dev = appledwc->dev;
	appledwc->dwc.glue_ops = &dwc3_apple_glue_ops;
	probe_data.dwc = &appledwc->dwc;
	probe_data.res = appledwc->mmio_resource;
	probe_data.ignore_clocks_and_resets = true;
	probe_data.skip_core_init_mode = true;
	if (dwc3_apple_host_before_core_init(appledwc, state))
		probe_data.initial_role = USB_ROLE_HOST;
	probe_data.properties = DWC3_DEFAULT_PROPERTIES;
	/*
	 * The controller has no wake signal of its own, and a halted xHC writes no events, so its
	 * interrupt cannot fire either. xHCI keeps running instead and decides which of its events
	 * wake the system; the core stays powered across system sleep, see dwc3_apple_init().
	 */
	probe_data.properties.xhci_run_in_s2idle = true;

	/*
	 * This runs from the role switch callback, so a failure leaves the glue
	 * device bound with everything dwc3_core_probe() acquired through devres
	 * still attached, the request of the core register region included. The
	 * next cable event retries dwc3_core_probe() on this struct and would fail
	 * with -EBUSY on that region. Group the devres of one attempt so that a
	 * failed attempt can be released.
	 */
	group = devres_open_group(appledwc->dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	ret = dwc3_core_probe(&probe_data);
	/* The glue remains bound even when a first core attempt unwinds. */
	dev_set_drvdata(appledwc->dev, &appledwc->dwc);
	if (ret) {
		devres_release_group(appledwc->dev, group);
		/*
		 * dwc3_core_probe() initialised the locks, work items and lists of
		 * the embedded struct dwc3. The retry expects a zeroed struct, as
		 * on the first attempt.
		 */
		memset(&appledwc->dwc, 0, sizeof(appledwc->dwc));
		return ret;
	}
	devres_remove_group(appledwc->dev, group);

	/*
	 * dwc3_core_probe() enabled runtime PM with the device active and forbade it, for the
	 * dwc3 platform driver's own callbacks, which this glue does not use. The core is being
	 * brought up now, so take the glue's reference (see dwc3_apple_core_start()) and, where
	 * that is tested, allow runtime PM so the reference alone decides.
	 */
	pm_runtime_get_noresume(appledwc->dev);
	appledwc->rpm_held = true;
	if (appledwc->idle_rpm)
		pm_runtime_allow(appledwc->dev);

	appledwc->state = DWC3_APPLE_NO_CABLE;
	return 0;
}

static int dwc3_apple_core_init(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_PROBE_PENDING:
		ret = dwc3_apple_core_probe(appledwc, state);
		if (ret)
			dev_err(appledwc->dev, "Failed to probe DWC3 Core, err=%d\n", ret);
		break;
	case DWC3_APPLE_NO_CABLE:
		/* Physical reset also loses the role on a later cable session. */
		if (dwc3_apple_host_before_core_init(appledwc, state)) {
			appledwc->dwc.dr_mode = USB_DR_MODE_HOST;
			dwc3_apple_set_ptrcap(appledwc, DWC3_GCTL_PRTCAP_HOST);
		}
		ret = dwc3_core_init(&appledwc->dwc);
		if (ret)
			dev_err(appledwc->dev, "Failed to initialize DWC3 Core, err=%d\n", ret);
		break;
	default:
		/* Unreachable unless there's a bug in this driver */
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* Keep failed releases owned by the glue, without repeating completed teardown. */
static int dwc3_apple_core_stop(struct dwc3_apple *appledwc)
{
	struct dwc3 *dwc = &appledwc->dwc;
	int i, ret;

	lockdep_assert_held(&appledwc->lock);
	if (appledwc->core_active) {
		if (!appledwc->core_exit_pending) {
			/* The PHY hands the PIPE back in P3, as it took it over */
			dwc->dis_u3_susphy_quirk = appledwc->dt_dis_u3_susphy;
			dwc3_enable_susphy(dwc, true);
			dwc3_event_buffers_cleanup(dwc);
			appledwc->core_exit_pending = true;
		}
		for (i = 0; i < dwc->num_usb3_ports; i++) {
			if (!(appledwc->usb3_power_off_pending & BIT(i)))
				continue;
			ret = phy_power_off(dwc->usb3_generic_phy[i]);
			if (ret)
				return ret;
			appledwc->usb3_power_off_pending &= ~BIT(i);
		}
		for (i = 0; i < dwc->num_usb2_ports; i++) {
			if (!(appledwc->usb2_power_off_pending & BIT(i)))
				continue;
			ret = phy_power_off(dwc->usb2_generic_phy[i]);
			if (ret)
				return ret;
			appledwc->usb2_power_off_pending &= ~BIT(i);
		}
		usb_phy_set_suspend(dwc->usb3_phy, 1);
		usb_phy_set_suspend(dwc->usb2_phy, 1);
		dwc3_core_exit_after_phy_power_off(dwc);
		appledwc->core_active = false;
		appledwc->core_exit_pending = false;
	}
	if (appledwc->reset_pending) {
		ret = reset_control_assert(appledwc->reset);
		if (ret)
			return ret;
		appledwc->reset_pending = false;
	}
	dwc3_apple_syscore(appledwc, false);
	if (appledwc->rpm_held) {
		appledwc->rpm_held = false;
		pm_runtime_mark_last_busy(appledwc->dev);
		pm_runtime_put_autosuspend(appledwc->dev);
	}

	return 0;
}

/*
 * Bring the core up for the given role: configure the USB2 PHY, release the reset and probe
 * (on the first call) or initialise the core.
 */
static int dwc3_apple_core_start(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	enum phy_mode mode;
	int ret, ret_reset;

	lockdep_assert_held(&appledwc->lock);

	/* A failed exit must finish before any new PHY reference is acquired. */
	ret = dwc3_apple_core_stop(appledwc);
	if (ret)
		return ret;

	/*
	 * The USB2 PHY on this platform must be configured for device mode before dwc3 tries to
	 * access it. Otherwise, the new configuration will sometimes only take affect after the
	 * *next* time dwc3 is brought up which causes the connected device to just not work.
	 * Host mode is left to the xHCI root hub, which selects it once the core is out of reset
	 * (usb_add_hcd() asks for PHY_MODE_USB_HOST_SS, which the PHY rejects, and then for
	 * PHY_MODE_USB_HOST). On a Type-C port the PHY is already running at this point, because
	 * the Type-C mux starts it before the role switch reaches this driver, and switching the
	 * running PHY to host mode while the core is still held in reset leaves the USB2 port
	 * without a connection for the whole session. A PHY that is in host mode already needs no
	 * selection here either.
	 * The USB3 PHY must be configured later after dwc3 has already been initialized.
	 * Both PHYs were looked up at probe, so device mode is also selected on the first
	 * bring-up, before dwc3_core_probe() has looked them up itself.
	 */
	ret = dwc3_apple_usb2_mode_before_reset(state, &mode);
	if (ret) {
		/* Unreachable unless there's a bug in this driver */
		return ret;
	}
	/* T8132 latches the host role while its USB2 PHY is stopped. */
	if (of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3") &&
	    state == DWC3_APPLE_HOST)
		mode = PHY_MODE_USB_HOST;
	if (mode != PHY_MODE_INVALID) {
		ret = phy_set_mode(appledwc->usb2_phy, mode);
		if (ret) {
			if (of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3"))
				return ret;
			dev_warn(appledwc->dev, "Failed to set the USB2 PHY mode, err=%d\n", ret);
		}
	}

	/*
	 * Power an idle port's controller up for its first core probe, then leave runtime PM
	 * to dwc3_core_probe(), which expects it disabled with the device active.
	 */
	if (appledwc->pre_core_rpm) {
		ret = pm_runtime_resume_and_get(appledwc->dev);
		if (ret)
			return ret;
		pm_runtime_disable(appledwc->dev);
		pm_runtime_put_noidle(appledwc->dev);
		appledwc->pre_core_rpm = false;
	}

	/*
	 * Before the first core probe the controller is powered, with runtime PM off or handed
	 * over above; dwc3_apple_core_probe() takes the reference then.
	 */
	if (appledwc->state != DWC3_APPLE_PROBE_PENDING && !appledwc->rpm_held) {
		ret = pm_runtime_resume_and_get(appledwc->dev);
		if (ret)
			return ret;
		appledwc->rpm_held = true;
	}

	ret = reset_control_deassert(appledwc->reset);
	if (ret) {
		if (ret == -ENOLINK && appledwc->sleep_phy_wait)
			dev_dbg(appledwc->dev, "PHY not started yet, reset kept\n");
		else
			dev_err(appledwc->dev, "Failed to deassert reset, err=%d\n", ret);
		/* Nothing was started: drop the reference again. */
		ret_reset = dwc3_apple_core_stop(appledwc);
		if (ret_reset)
			dev_warn(appledwc->dev, "Failed to finish core shutdown, err=%d\n", ret_reset);
		return ret;
	}

	appledwc->reset_pending = true;
	/* Keep register access powered until both PHY shutdown and reset finish. */
	dwc3_apple_syscore(appledwc, true);
	ret = dwc3_apple_core_init(appledwc, state);
	if (ret) {
		ret_reset = dwc3_apple_core_stop(appledwc);
		if (ret_reset)
			dev_warn(appledwc->dev, "Failed to finish core shutdown, err=%d\n", ret_reset);
		return ret;
	}

	/*
	 * Now that the core is initialized and already went through dwc3_core_soft_reset we can
	 * configure some unknown Apple-specific settings and then bring up xhci or gadget mode.
	 */
	/* Native J713 host reset uses the same CIO setup before xHCI starts. */
	if (!of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3") ||
	    dwc3_apple_host_before_core_init(appledwc, state))
		dwc3_apple_setup_cio(appledwc);
	dwc3_apple_apply_tunables(appledwc);
	dwc3_apple_setup_hibernation(appledwc);
	appledwc->core_active = true;
	appledwc->usb2_power_off_pending = BIT(appledwc->dwc.num_usb2_ports) - 1;
	appledwc->usb3_power_off_pending = BIT(appledwc->dwc.num_usb3_ports) - 1;

	return 0;
}

/*
 * The T6040 core came up against the dummy PHY: reset its USB3 PIPE side once the PIPE has
 * moved, as the tested bring-up does (USB2 is not affected).
 */
static void dwc3_apple_usb3_pipe_reset(struct dwc3_apple *appledwc)
{
	struct dwc3 *dwc = &appledwc->dwc;
	u32 reg;

	reg = dwc3_readl(dwc, DWC3_GUSB3PIPECTL(0));
	dwc3_writel(dwc, DWC3_GUSB3PIPECTL(0), reg | DWC3_GUSB3PIPECTL_PHYSOFTRST);
	usleep_range(1000, 1500);
	dwc3_writel(dwc, DWC3_GUSB3PIPECTL(0), reg & ~DWC3_GUSB3PIPECTL_PHYSOFTRST);
	msleep(100);
}

/* Select the role in the core and route the USB3 PHY, before xhci or the gadget is started */
static int dwc3_apple_set_role(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	bool host = state == DWC3_APPLE_HOST;
	int ret;

	lockdep_assert_held(&appledwc->lock);

	appledwc->dwc.dr_mode = host ? USB_DR_MODE_HOST : USB_DR_MODE_PERIPHERAL;
	dwc3_apple_set_ptrcap(appledwc, host ? DWC3_GCTL_PRTCAP_HOST : DWC3_GCTL_PRTCAP_DEVICE);
	/*
	 * This platform requires SUSPHY to be enabled here already in order to properly
	 * configure the PHY and switch dwc3's PIPE interface to USB3 PHY. The USB2 PHY's
	 * device mode and the T8132 host role were selected in dwc3_apple_core_start();
	 * other host roles are selected by the xHCI root hub as it starts.
	 */
	appledwc->dwc.dis_u3_susphy_quirk = appledwc->dt_dis_u3_susphy;
	dwc3_enable_susphy(&appledwc->dwc, true);
	ret = phy_set_mode(appledwc->usb3_phy, host ? PHY_MODE_USB_HOST : PHY_MODE_USB_DEVICE);
	if (ret) {
		/*
		 * A failed T8132 BIST or PIPE handoff must not start xHCI. Neither must a failed
		 * T6050 one: that PHY reports a port left on USB2 (four DisplayPort lanes) as
		 * success, and its errors leave the PIPE without a known producer.
		 */
		if (of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3") ||
		    of_device_is_compatible(appledwc->dev->of_node, "apple,t6050-dwc3")) {
			dev_err(appledwc->dev, "USB3 PHY setup failed, err=%d\n", ret);
			return ret;
		}
		dev_warn(appledwc->dev, "USB3 PHY setup failed, USB2 only, err=%d\n", ret);
	} else if (host && appledwc->usb3_phy &&
		   (of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3") ||
		    of_device_is_compatible(appledwc->dev->of_node, "apple,t8142-dwc3"))) {
		/*
		 * These PHYs do not bring a SuperSpeed link back from U3 once
		 * the core has suspended them: keep them out of P3 from here on.
		 */
		appledwc->dwc.dis_u3_susphy_quirk = true;
		dwc3_enable_susphy(&appledwc->dwc, true);
	} else if (appledwc->usb3_phy &&
		   of_device_is_compatible(appledwc->dev->of_node, "apple,t6040-dwc3")) {
		dwc3_apple_usb3_pipe_reset(appledwc);
	}
	return 0;
}

/*
 * No device mode: the T6040 USB2 PHY (phy-apple-t6040-usb2) and the T6050 Type-C PHY implement
 * host mode only, and a kernel built without gadget support turns dwc3_gadget_init() into a stub
 * that succeeds. Such a controller must not follow a USB_ROLE_DEVICE request from the Type-C port
 * controller (a charger or a host on the other end), or the core would end up in device mode on
 * top of a PHY powered in host mode.
 */
static bool dwc3_apple_host_only(struct device *dev)
{
	if (!IS_ENABLED(CONFIG_USB_DWC3_GADGET) && !IS_ENABLED(CONFIG_USB_DWC3_DUAL_ROLE))
		return true;

	return of_device_is_compatible(dev->of_node, "apple,t6040-dwc3") ||
	       of_device_is_compatible(dev->of_node, "apple,t6050-dwc3");
}

static int dwc3_apple_init(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	int ret, ret_reset;

	lockdep_assert_held(&appledwc->lock);

	/*
	 * A host-only controller starts as a host: mark the role before the core comes up so that
	 * dwc3_core_soft_reset() leaves the unused gadget side alone. A failed first core probe
	 * zeroes the struct, so this is done on every start.
	 */
	if (state == DWC3_APPLE_HOST && appledwc->host_only)
		appledwc->dwc.current_dr_role = DWC3_GCTL_PRTCAP_HOST;

	ret = dwc3_apple_core_start(appledwc, state);
	if (ret)
		return ret;

	ret = dwc3_apple_set_role(appledwc, state);
	if (ret)
		goto core_exit;

	switch (state) {
	case DWC3_APPLE_HOST:
		ret = dwc3_host_init(&appledwc->dwc);
		if (ret) {
			dev_err(appledwc->dev, "Failed to initialize host, ret=%d\n", ret);
			goto core_exit;
		}
		/*
		 * A USB2 PHY with a port reset (the T6050 Type-C PHY) takes it once the root hub is
		 * up; for the other PHYs, which have none, this is a no-op.
		 */
		ret = phy_reset(appledwc->usb2_phy);
		if (ret) {
			dev_err(appledwc->dev, "Failed to reset the USB2 port, ret=%d\n", ret);
			dwc3_host_exit(&appledwc->dwc);
			goto core_exit;
		}
		break;
	case DWC3_APPLE_DEVICE:
		ret = dwc3_gadget_init(&appledwc->dwc);
		if (ret) {
			dev_err(appledwc->dev, "Failed to initialize gadget, ret=%d\n", ret);
			goto core_exit;
		}
		break;
	default:
		/* Unreachable unless there's a bug in this driver */
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		goto core_exit;
	}

	appledwc->state = state;
	/* macOS votes USB_SLP_DDR from here until the host's Off, through every sleep. */
	if (state == DWC3_APPLE_HOST && appledwc->sleep_ddr)
		apple_sleep_usb_slp_ddr(true);
	/*
	 * A connected cable leaves the Apple DWC3 core and its PHY as one live,
	 * stateful unit.  The glue driver tears that unit down explicitly on a
	 * role change or disconnect.  Do not let generic system PM independently
	 * power-gate the DWC3 device in between: xHCI then observes a dead core on
	 * resume and cannot restore the root hubs without a full cable teardown.
	 * The same holds for a fixed-hub controller, whose core is up for as long
	 * as the driver is bound. As a syscore device the core is not counted as
	 * suspended in its power domain, so the domain stays on, and xHCI saves
	 * its state on suspend and restores it on resume.
	 */
	dwc3_apple_syscore(appledwc, true);
	return 0;

core_exit:
	ret_reset = dwc3_apple_core_stop(appledwc);
	if (ret_reset)
		dev_warn(appledwc->dev, "Failed to finish core shutdown, err=%d\n", ret_reset);

	return ret;
}

static int dwc3_apple_exit(struct dwc3_apple *appledwc);

/* A hub that hasn't enumerated by then didn't come back. */
#define DWC3_APPLE_HUB_CHECK_MS 5000

/* Any way a fixed-hub host came out of a sleep, its hub must be there a little later. */
static void dwc3_apple_check_hub_later(struct dwc3_apple *appledwc)
{
	lockdep_assert_held(&appledwc->lock);
	if (appledwc->fixed_hub && !appledwc->shutting_down && !appledwc->sleep_dead &&
	    !appledwc->sleep_stopped && appledwc->state == DWC3_APPLE_HOST &&
	    appledwc->core_active && !appledwc->core_exit_pending &&
	    READ_ONCE(appledwc->sleep_phase) == DWC3_APPLE_SLEEP_IDLE)
		mod_delayed_work(system_freezable_wq, &appledwc->hub_check,
				 msecs_to_jiffies(DWC3_APPLE_HUB_CHECK_MS));
}

/*
 * Waiting for a Type-C PHY that is not started after sleep is specific to the T6050 boards
 * whose port controllers take the sleep's power states (J714s, J716c). The notifier work
 * and its admission rules apply to every board.
 */
static bool dwc3_apple_sleep_board(void)
{
	return of_machine_is_compatible("apple,j714s") || of_machine_is_compatible("apple,j716c");
}

/*
 * Bring a controller up again after a sleep that stopped it or lost it. A fixed-hub host
 * starts as at probe, where the core has no role yet, so dwc3_core_soft_reset() runs too:
 * rebinding the driver brought the J700's hub back where a restart without it didn't.
 */
static int dwc3_apple_start_after_sleep(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);
	if (appledwc->fixed_hub)
		appledwc->dwc.current_dr_role = 0;
	/*
	 * After a real sleep a Type-C port's PHY starts with the port controller's mode request,
	 * and the role switch that follows it starts the controller, as at a plug-in.
	 * A sleep given up after the port controllers went to their sleep state leaves the PHY
	 * stopped in the same way, so this doesn't ask whether the sleep was a real one: a PHY
	 * that kept running never reports -ENOLINK. On the board that has run on; elsewhere
	 * the error is an error, as before.
	 */
	appledwc->sleep_phy_wait = !appledwc->fixed_hub && appledwc->role_sw &&
				   dwc3_apple_sleep_board();
	ret = dwc3_apple_init(appledwc, state);
	if (ret == -ENOLINK && appledwc->sleep_phy_wait) {
		appledwc->sleep_phy_wait = false;
		appledwc->sleep_phy_pending = true;
		dev_info(appledwc->dev, "waiting for the Type-C port to start the PHY after the sleep\n");
		return 0;
	}
	appledwc->sleep_phy_wait = false;
	appledwc->sleep_phy_pending = false;
	if (ret) {
		dev_err(appledwc->dev, "Failed to start after sleep, err=%d\n", ret);
		return ret;
	}
	dwc3_apple_check_hub_later(appledwc);
	return 0;
}

/* Whether the fixed hub is on its USB2 root port. Takes the xHCI device's lock. */
static bool dwc3_apple_hub_present(struct dwc3_apple *appledwc)
{
	struct platform_device *xhci = appledwc->dwc.xhci;
	struct usb_device *root;
	struct usb_hcd *hcd;
	bool present = true;

	lockdep_assert_held(&appledwc->lock);
	if (!xhci)
		return true;

	device_lock(&xhci->dev);
	hcd = xhci->dev.driver ? platform_get_drvdata(xhci) : NULL;
	root = hcd ? hcd->self.root_hub : NULL;
	if (root) {
		usb_lock_device(root);
		present = usb_hub_find_child(root, appledwc->hub_port);
		usb_unlock_device(root);
	}
	device_unlock(&xhci->dev);

	return present;
}

/* Whether @udev or a device below it can wake the system. Takes the hubs' locks. */
static bool dwc3_apple_can_wake(struct usb_device *udev)
{
	struct usb_device *child;
	bool wake = device_may_wakeup(&udev->dev);
	int port1;

	if (wake || !udev->maxchild)
		return wake;

	usb_lock_device(udev);
	usb_hub_for_each_child(udev, port1, child) {
		wake = dwc3_apple_can_wake(child);
		if (wake)
			break;
	}
	usb_unlock_device(udev);

	return wake;
}

/*
 * The fixed hub's ports with a device at or below them that can wake the system, bit n for
 * port n. The hub's own setting doesn't count. Takes the xHCI device's lock and the hubs'.
 */
static unsigned long dwc3_apple_hub_wake_ports(struct dwc3_apple *appledwc)
{
	struct platform_device *xhci = appledwc->dwc.xhci;
	struct usb_device *root, *hub, *child;
	unsigned long ports = 0;
	struct usb_hcd *hcd;
	int port1;

	lockdep_assert_held(&appledwc->lock);
	if (!xhci)
		return 0;

	device_lock(&xhci->dev);
	hcd = xhci->dev.driver ? platform_get_drvdata(xhci) : NULL;
	root = hcd ? hcd->self.root_hub : NULL;
	if (root) {
		usb_lock_device(root);
		hub = usb_hub_find_child(root, appledwc->hub_port);
		if (hub) {
			usb_lock_device(hub);
			usb_hub_for_each_child(hub, port1, child) {
				if (port1 < BITS_PER_LONG && dwc3_apple_can_wake(child))
					ports |= BIT(port1);
			}
			usb_unlock_device(hub);
		}
		usb_unlock_device(root);
	}
	device_unlock(&xhci->dev);

	return ports;
}

/*
 * Once after a sleep: if the hub isn't there, stop the host and start it again, with the
 * hub's link down for a moment first, as unbinding and binding the driver does. This also
 * catches an xHCI that didn't come back after the core was set up again.
 */
static void dwc3_apple_hub_check(struct work_struct *work)
{
	struct dwc3_apple *appledwc = container_of(to_delayed_work(work), struct dwc3_apple,
						   hub_check);
	int ret;

	guard(mutex)(&appledwc->lock);
	if (appledwc->shutting_down || appledwc->sleep_dead || appledwc->sleep_stopped ||
	    !appledwc->core_active || appledwc->core_exit_pending ||
	    READ_ONCE(appledwc->sleep_phase) != DWC3_APPLE_SLEEP_IDLE ||
	    appledwc->state != DWC3_APPLE_HOST || dwc3_apple_hub_present(appledwc))
		return;

	dev_warn(appledwc->dev, "hub not back after the sleep: starting the host again\n");
	ret = dwc3_apple_exit(appledwc);
	if (ret) {
		dev_err(appledwc->dev, "Failed to stop the host again, err=%d\n", ret);
		return;
	}
	msleep(500);
	appledwc->dwc.current_dr_role = 0;
	ret = dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	if (ret)
		dev_err(appledwc->dev, "Failed to start the host again, err=%d\n", ret);
}

static int dwc3_apple_exit(struct dwc3_apple *appledwc)
{
	lockdep_assert_held(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_PROBE_PENDING:
	case DWC3_APPLE_NO_CABLE:
		break;
	case DWC3_APPLE_DEVICE:
		dwc3_gadget_exit(&appledwc->dwc);
		appledwc->state = DWC3_APPLE_NO_CABLE;
		break;
	case DWC3_APPLE_HOST:
		/* dwc3_host_exit() lets the USB3 PHY into P3 for the PIPE handback */
		appledwc->dwc.dis_u3_susphy_quirk = appledwc->dt_dis_u3_susphy;
		dwc3_host_exit(&appledwc->dwc);
		appledwc->state = DWC3_APPLE_NO_CABLE;
		if (appledwc->sleep_ddr)
			apple_sleep_usb_slp_ddr(false);
		break;
	}

	return dwc3_apple_core_stop(appledwc);
}

static int dwc3_usb_role_switch_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);
	int ret;

	guard(mutex)(&appledwc->lock);

	if (appledwc->shutting_down)
		return -ESHUTDOWN;
	if (role != USB_ROLE_NONE && role != USB_ROLE_HOST && role != USB_ROLE_DEVICE)
		return -EINVAL;
	if (role == USB_ROLE_DEVICE && appledwc->host_only)
		role = USB_ROLE_NONE;

	/* A controller behind a fixed hub is a host regardless of the connectors */
	if (appledwc->fixed_hub)
		return 0;

	/* Stopped for a sleep: this is what comes back after it. */
	if (appledwc->sleep_stopped) {
		appledwc->sleep_state = dwc3_apple_state_for_role(role);
		return 0;
	}
	/* J714s: kept suspended for a router's sleep: applied once the sleep is over. */
	if (appledwc->sleep_tunnel) {
		appledwc->sleep_tunnel_role = role;
		return 0;
	}

	/* Freeze ownership through PM preparation and its matching completion. */
	if (READ_ONCE(appledwc->sleep_phase) != DWC3_APPLE_SLEEP_IDLE) {
		appledwc->sleep_role = dwc3_apple_state_for_role(role);
		appledwc->sleep_role_pending = true;
		return 0;
	}

	/*
	 * Skip role switches if appledwc is already in the desired state. The
	 * USB-C port controller on M2 and M1/M2 Pro/Max/Ultra devices issues
	 * additional interrupts which results in usb_role_switch_set_role()
	 * calls with the current role.
	 * Ignore those calls here to ensure the USB-C port controller and
	 * appledwc are in a consistent state.
	 * This matches the behaviour in __dwc3_set_mode().
	 * Do no handle USB_ROLE_NONE for DWC3_APPLE_NO_CABLE and
	 * DWC3_APPLE_PROBE_PENDING since that is no-op anyway.
	 */
	if (appledwc->state == DWC3_APPLE_HOST && role == USB_ROLE_HOST)
		return 0;
	if (appledwc->state == DWC3_APPLE_DEVICE && role == USB_ROLE_DEVICE)
		return 0;

	/*
	 * We need to tear all of dwc3 down and re-initialize it every time a cable is
	 * connected or disconnected or when the mode changes. See the documentation for enum
	 * dwc3_apple_state for details.
	 */
	ret = dwc3_apple_exit(appledwc);
	if (ret)
		return ret;

	switch (role) {
	case USB_ROLE_NONE:
		/* Nothing to do if no cable is connected */
		appledwc->sleep_phy_pending = false;
		return 0;
	case USB_ROLE_HOST:
	case USB_ROLE_DEVICE:
		/*
		 * While the start after a sleep waits for the PHY, a role switch can still come
		 * before the port controller's mode request. The error goes back as it is, so the
		 * port controller tries again, without a line in the log.
		 */
		appledwc->sleep_phy_wait = appledwc->sleep_phy_pending;
		ret = dwc3_apple_init(appledwc, dwc3_apple_state_for_role(role));
		appledwc->sleep_phy_wait = false;
		if (ret != -ENOLINK)
			appledwc->sleep_phy_pending = false;
		return ret;
	default:
		dev_err(appledwc->dev, "Invalid target role: %d\n", role);
		return -EINVAL;
	}
}

static enum usb_role dwc3_usb_role_switch_get(struct usb_role_switch *sw)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);

	guard(mutex)(&appledwc->lock);

	switch (appledwc->sleep_role_pending ? appledwc->sleep_role :
		appledwc->sleep_stopped ? appledwc->sleep_state : appledwc->state) {
	case DWC3_APPLE_HOST:
		return USB_ROLE_HOST;
	case DWC3_APPLE_DEVICE:
		return USB_ROLE_DEVICE;
	case DWC3_APPLE_NO_CABLE:
	case DWC3_APPLE_PROBE_PENDING:
		return USB_ROLE_NONE;
	default:
		/* Unreachable unless there's a bug in this driver */
		dev_err(appledwc->dev, "Invalid internal state: %d\n", appledwc->state);
		return USB_ROLE_NONE;
	}
}

static int dwc3_apple_setup_role_switch(struct dwc3_apple *appledwc)
{
	struct usb_role_switch_desc dwc3_role_switch = { NULL };

	dwc3_role_switch.fwnode = dev_fwnode(appledwc->dev);
	dwc3_role_switch.set = dwc3_usb_role_switch_set;
	dwc3_role_switch.get = dwc3_usb_role_switch_get;
	dwc3_role_switch.driver_data = appledwc;
	appledwc->role_sw = usb_role_switch_register(appledwc->dev, &dwc3_role_switch);
	if (IS_ERR(appledwc->role_sw))
		return PTR_ERR(appledwc->role_sw);

	return 0;
}

/* Keep the restart obligation even when exit fails after removing the host. */
static int dwc3_apple_sleep_stop(struct dwc3_apple *appledwc)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);
	if (appledwc->shutting_down)
		return -ESHUTDOWN;
	if (!appledwc->sleep_stopped) {
		appledwc->sleep_state = appledwc->state;
		appledwc->sleep_wakeup = device_may_wakeup(appledwc->dev);
		appledwc->sleep_stopped = true;
	}
	ret = dwc3_apple_exit(appledwc);
	if (!ret)
		device_set_wakeup_enable(appledwc->dev, false);
	return ret;
}

/* A role accepted during admission must be applied by this epoch's finish. */
static void dwc3_apple_sleep_take_role(struct dwc3_apple *appledwc)
{
	lockdep_assert_held(&appledwc->lock);
	if (appledwc->sleep_stopped && appledwc->sleep_role_pending) {
		appledwc->sleep_state = appledwc->sleep_role;
		appledwc->sleep_role_pending = false;
	}
}

static int dwc3_apple_sleep_start(struct dwc3_apple *appledwc)
{
	int ret = 0;

	lockdep_assert_held(&appledwc->lock);
	if (!appledwc->sleep_stopped || appledwc->shutting_down)
		return 0;
	dwc3_apple_sleep_take_role(appledwc);
	device_set_wakeup_enable(appledwc->dev, appledwc->sleep_wakeup);
	if (appledwc->sleep_state == DWC3_APPLE_HOST ||
	    appledwc->sleep_state == DWC3_APPLE_DEVICE)
		ret = dwc3_apple_start_after_sleep(appledwc, appledwc->sleep_state);
	else
		ret = dwc3_apple_core_stop(appledwc);
	/* NONE also owes PHY/reset cleanup; consume the obligation only on success. */
	if (!ret)
		appledwc->sleep_stopped = false;
	return ret;
}

/* No core access until the PHY says that the controller is powered again. */
static int dwc3_apple_recover(struct dwc3_apple *appledwc)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);
	ret = apple_atcphy_sleep_recover(appledwc->usb2_phy);
	if (ret < 0) {
		appledwc->sleep_dead = true;
		dev_err(appledwc->dev, "USB host still unpowered: sleep refused (%d)\n", ret);
		return ret;
	}
	appledwc->sleep_dead = false;
	ret = dwc3_apple_sleep_stop(appledwc);
	/* A fixed-hub host comes back even if an earlier exit already removed its role. */
	appledwc->sleep_state = DWC3_APPLE_HOST;
	return ret ?: dwc3_apple_sleep_start(appledwc);
}

/*
 * TEST (J714s, T6050): macOS keeps the port's USB host powered through the USB4 router's sleep:
 * ATC2_USB goes off 28 to 37 ms after the router firmware's state word reads asleep (runs mxt4,
 * mxt5, mxt6, mxt6b, seven sleeps), so the router's USB3 tunnel keeps its host. Linux stopped the
 * host here, 2.66 s before the dock's sleep bit (run sv6t9), and parked its PIPE. With its PIPE
 * on the router's USB3 tunnel the host now stays: xHCI and the devices behind it suspend as for
 * any sleep, their links in U3; the glue stays a syscore device, so the core keeps its power and
 * registers; the router's host parks the PIPE behind the router firmware's sleep, or with the
 * router awake again after a refusal, see apple_t6050_phy_cio_usb().
 */
static bool dwc3_apple_tunnel_keep(struct dwc3_apple *appledwc)
{
	lockdep_assert_held(&appledwc->lock);
	if (appledwc->fixed_hub || !(of_machine_is_compatible("apple,j714s") ||
				     of_machine_is_compatible("apple,j716c")) ||
	    !of_device_is_compatible(appledwc->dev->of_node, "apple,t6050-dwc3") ||
	    dwc3_apple_tunnel_phy(appledwc, APPLE_T6050_CIO_USB_KEEP))
		return false;
	appledwc->sleep_tunnel = true;
	appledwc->sleep_tunnel_taken = false;
	appledwc->sleep_tunnel_result = APPLE_T6050_CIO_USB_KEPT;
	appledwc->sleep_tunnel_role = USB_ROLE_HOST;
	dev_info(appledwc->dev, "USB host kept for the USB4 router's sleep: xHCI suspends in place\n");
	return true;
}

/*
 * TEST (J714s): after the sleep, or one given up, a host kept on its router's tunnel whose PIPE
 * was parked, whose core was lost and not set up again, or whose port controller asked for
 * another role meanwhile is stopped and started again from a work, outside the notifier chain,
 * as after a cable's return. One kept on the tunnel, or set up again in place, goes on.
 */
static void dwc3_apple_tunnel_done(struct dwc3_apple *appledwc)
{
	lockdep_assert_held(&appledwc->lock);
	appledwc->sleep_tunnel = false;
	if (!appledwc->sleep_tunnel_taken) {
		appledwc->sleep_tunnel_taken = true;
		appledwc->sleep_tunnel_result =
			dwc3_apple_tunnel_phy(appledwc, APPLE_T6050_CIO_USB_RESULT);
	}
	if (appledwc->sleep_tunnel_result < 0)
		appledwc->sleep_tunnel_result = APPLE_T6050_CIO_USB_LOST;
	/* TEST (sv6t11) */
	dev_info(appledwc->dev, "TEST sv6t11: USB host kept through the router's sleep, after the sleep: result %d (0 kept, 1 parked, 2 lost and not set up again, 3 set up again in place), the port controller's role meanwhile %d: %s\n",
		 appledwc->sleep_tunnel_result, appledwc->sleep_tunnel_role,
		 (appledwc->sleep_tunnel_result == APPLE_T6050_CIO_USB_KEPT ||
		  appledwc->sleep_tunnel_result == DWC3_APPLE_TUNNEL_REINIT) &&
		 appledwc->sleep_tunnel_role == USB_ROLE_HOST ? "goes on" : "stopped and started again");
	if ((appledwc->sleep_tunnel_result == APPLE_T6050_CIO_USB_KEPT ||
	     appledwc->sleep_tunnel_result == DWC3_APPLE_TUNNEL_REINIT) &&
	    appledwc->sleep_tunnel_role == USB_ROLE_HOST)
		return;
	queue_work(system_dfl_wq, &appledwc->sleep_tunnel_work);
}

static void dwc3_apple_tunnel_work(struct work_struct *work)
{
	struct dwc3_apple *appledwc = container_of(work, struct dwc3_apple, sleep_tunnel_work);
	int ret;

	guard(mutex)(&appledwc->lock);
	if (appledwc->shutting_down || appledwc->sleep_tunnel ||
	    appledwc->state != DWC3_APPLE_HOST)
		return;
	/* xHCI's stop touches the core: a lost one comes out of reset first. */
	if (appledwc->sleep_tunnel_result == APPLE_T6050_CIO_USB_LOST) {
		ret = reset_control_deassert(appledwc->reset);
		if (!ret)
			ret = dwc3_apple_core_reinit(appledwc);
		if (ret) {
			dev_err(appledwc->dev, "USB host's core not back after the router's sleep (%d): left as it is, sleeps refused\n",
				ret);
			appledwc->sleep_dead = true;
			return;
		}
	}
	ret = dwc3_apple_exit(appledwc);
	if (ret) {
		dev_err(appledwc->dev, "USB host not stopped after the router's sleep: %d\n", ret);
		return;
	}
	if (appledwc->sleep_tunnel_role == USB_ROLE_HOST)
		dwc3_apple_start_after_sleep(appledwc, DWC3_APPLE_HOST);
	dev_info(appledwc->dev, "USB host started again after the router's sleep (%d, role %d)\n",
		 appledwc->sleep_tunnel_result, appledwc->sleep_tunnel_role);
}

/* Called only by finish work, after the normal device/supplier resume ordering. */
static int dwc3_apple_sleep_kept(struct dwc3_apple *appledwc, bool *kept)
{
	struct dwc3 *dwc = &appledwc->dwc;
	u32 guid = 0;
	int ret;

	lockdep_assert_held(&appledwc->lock);
	appledwc->sleep_in_place = false;
	ret = apple_atcphy_sleep_result(appledwc->usb2_phy);
	if (ret == -ENODEV)
		return dwc3_apple_recover(appledwc);
	if (appledwc->sleep_reinit) {
		appledwc->sleep_reinit = false;
		*kept = true;
		dwc3_apple_check_hub_later(appledwc);
		return 0;
	}
	if (!ret || ret == -EBUSY || ret == APPLE_ATCPHY_SLEEP_POWER_LOST) {
		guid = dwc3_readl(dwc, DWC3_GUID);
		if (guid == DWC3_APPLE_SLEEP_GUID) {
			dwc3_writel(dwc, DWC3_GUID, appledwc->sleep_guid);
			*kept = true;
			dwc3_apple_check_hub_later(appledwc);
			return 0;
		}
		ret = -EIO;
	}
	dev_warn(appledwc->dev, "host lost its state in the sleep (%d, GUID %#x): restarting it\n",
		 ret, guid);
	ret = dwc3_apple_sleep_stop(appledwc);
	return ret ?: dwc3_apple_sleep_start(appledwc);
}

/* The notifier never waits. This existing bound belongs after the chain and freezer. */
#define DWC3_APPLE_SLEEP_STOP_PREPARE_MS	5000

static int dwc3_apple_sleep_prepare_work(struct dwc3_apple *appledwc)
{
	bool in_place;
	int ret;

	/* A restart left to the work after the last router sleep ends first. */
	flush_work(&appledwc->sleep_tunnel_work);
	/* No controller device lock is taken by this worker or its PHY prepare operation. */
	scoped_guard(mutex, &appledwc->lock) {
		if (appledwc->shutting_down)
			return -ESHUTDOWN;
		if (appledwc->sleep_dead) {
			ret = dwc3_apple_recover(appledwc);
			if (ret)
				return ret;
		}
		ret = dwc3_apple_sleep_start(appledwc);
		if (ret)
			return ret;
		if (!appledwc->sleep_deep)
			return 0;
		in_place = appledwc->fixed_hub && appledwc->state == DWC3_APPLE_HOST;
		/* A J714s host on a USB4 router's USB3 tunnel stays up for its router's sleep. */
		if (!in_place && appledwc->state == DWC3_APPLE_HOST && dwc3_apple_tunnel_keep(appledwc))
			return 0;
	}
	/* This only records a PHY sleep token; its hardware work runs at noirq. */
	in_place = in_place && !apple_atcphy_sleep_prepare(appledwc->usb2_phy);
	guard(mutex)(&appledwc->lock);
	if (in_place) {
		/* Own finish even if a role change made retention impossible meanwhile. */
		appledwc->sleep_in_place = true;
		if (appledwc->shutting_down || appledwc->state != DWC3_APPLE_HOST)
			return -EBUSY;
		appledwc->sleep_guid = dwc3_readl(&appledwc->dwc, DWC3_GUID);
		dwc3_writel(&appledwc->dwc, DWC3_GUID, DWC3_APPLE_SLEEP_GUID);
		apple_sleep_usb_set_wake_ports(dwc3_apple_hub_wake_ports(appledwc));
		return 0;
	}
	ret = dwc3_apple_sleep_stop(appledwc);
	dwc3_apple_sleep_take_role(appledwc);
	return ret;
}

static void dwc3_apple_sleep_stop_work(struct work_struct *work)
{
	struct dwc3_apple *appledwc = container_of(work, struct dwc3_apple, sleep_stop_work);

	appledwc->sleep_stop_ret = dwc3_apple_sleep_prepare_work(appledwc);
	appledwc->sleep_stop_end = ktime_get();
	complete(&appledwc->sleep_stop_done);
}

static void dwc3_apple_sleep_start_work(struct work_struct *work)
{
	struct dwc3_apple *appledwc = container_of(work, struct dwc3_apple, sleep_start_work);
	bool finish = false, kept = false;
	unsigned long flags;
	int ret = 0;

	/* May still need notifier unregister: never joined by a notifier callback. */
	flush_work(&appledwc->sleep_stop_work);
	/* FINISHING stays visible until every old hub callback has returned. */
	cancel_delayed_work_sync(&appledwc->hub_check);
	scoped_guard(mutex, &appledwc->lock) {
		if (appledwc->sleep_tunnel) {
			dwc3_apple_tunnel_done(appledwc);
		} else if (appledwc->sleep_in_place) {
			finish = true;
			apple_sleep_usb_set_wake_ports(0);
			if (!appledwc->shutting_down)
				ret = dwc3_apple_sleep_kept(appledwc, &kept);
			else
				appledwc->sleep_in_place = false;
		} else if (!appledwc->shutting_down) {
			if (appledwc->sleep_dead)
				ret = dwc3_apple_recover(appledwc);
			else
				ret = dwc3_apple_sleep_start(appledwc);
		}
		if (!ret && !appledwc->shutting_down && appledwc->sleep_role_pending) {
			if (appledwc->sleep_role == appledwc->state && appledwc->core_active &&
			    !appledwc->core_exit_pending && !appledwc->sleep_stopped &&
			    !appledwc->sleep_dead) {
				/* The latest request still matches the live role. */
				appledwc->sleep_role_pending = false;
			} else {
				ret = dwc3_apple_sleep_stop(appledwc);
				dwc3_apple_sleep_take_role(appledwc);
				if (!ret)
					ret = dwc3_apple_sleep_start(appledwc);
			}
		}
		/* A failed exit must retain power for all remaining PHY/reset ownership. */
		dwc3_apple_syscore(appledwc, appledwc->core_active || appledwc->reset_pending);
		/* Publish IDLE with the role lock held, after queuing the PHY's finish. */
		if (finish)
			apple_atcphy_sleep_finish_async(appledwc->usb2_phy, kept);
		spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
		if (appledwc->sleep_phase != DWC3_APPLE_SLEEP_REMOVING)
			appledwc->sleep_phase = DWC3_APPLE_SLEEP_IDLE;
		spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
		dwc3_apple_check_hub_later(appledwc);
	}
	if (ret)
		dev_err(appledwc->dev, "USB host recovery failed: %d\n", ret);
}

/* The blocking notifier chain must never join teardown, even indirectly through a lock. */
static int dwc3_apple_pm_notify(struct notifier_block *nb, unsigned long action, void *unused)
{
	struct dwc3_apple *appledwc = container_of(nb, struct dwc3_apple, pm_nb);
	unsigned long flags;
	bool start = false;

	if (action == PM_POST_SUSPEND) {
		spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
		if (appledwc->sleep_phase == DWC3_APPLE_SLEEP_PREPARING ||
		    appledwc->sleep_phase == DWC3_APPLE_SLEEP_PREPARED) {
			appledwc->sleep_phase = DWC3_APPLE_SLEEP_FINISHING;
			start = true;
		}
		spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
		if (start)
			queue_work(system_dfl_wq, &appledwc->sleep_start_work);
		return NOTIFY_DONE;
	}
	if (action != PM_SUSPEND_PREPARE || !READ_ONCE(appledwc->sleep_pm_online))
		return NOTIFY_DONE;
	if (!mutex_trylock(&appledwc->lock))
		return notifier_from_errno(-EBUSY);
	/* Finish queues PHY work under this mutex; check readiness before claiming the epoch. */
	if (!apple_atcphy_sleep_finish_done(appledwc->usb2_phy)) {
		mutex_unlock(&appledwc->lock);
		return notifier_from_errno(-EBUSY);
	}
	spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
	if (appledwc->sleep_phase != DWC3_APPLE_SLEEP_IDLE || appledwc->shutting_down) {
		spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
		mutex_unlock(&appledwc->lock);
		return notifier_from_errno(-EBUSY);
	}
	appledwc->sleep_phase = DWC3_APPLE_SLEEP_PREPARING;
	/* A "freeze" is a suspend-to-idle with deep as the default too: the host stays. */
	appledwc->sleep_deep = !pm_suspend_default_s2idle() && !pm_suspend_no_platform();
	/* Every admitted epoch must reach device prepare, including a retained host. */
	dev_pm_syscore_device(appledwc->dev, false);
	reinit_completion(&appledwc->sleep_stop_done);
	appledwc->sleep_stop_ret = 0;
	appledwc->sleep_stop_start = ktime_get();
	spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
	/* A running check either owned the mutex already, or sees the active epoch and exits. */
	cancel_delayed_work(&appledwc->hub_check);
	mutex_unlock(&appledwc->lock);
	queue_work(system_dfl_wq, &appledwc->sleep_stop_work);
	/* Always successful once admitted, so robust rollback includes this notifier. */
	return NOTIFY_DONE;
}

static void dwc3_apple_pm_notifier_remove(void *data)
{
	struct dwc3_apple *appledwc = data;
	unsigned long flags;

	unregister_pm_notifier(&appledwc->pm_nb);
	spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
	appledwc->sleep_phase = DWC3_APPLE_SLEEP_REMOVING;
	spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
}

static int dwc3_apple_pm_prepare(struct device *dev)
{
	struct dwc3_apple *appledwc = to_dwc3_apple(dev_get_drvdata(dev));
	unsigned long flags;
	int ret;

	if (READ_ONCE(appledwc->sleep_phase) != DWC3_APPLE_SLEEP_PREPARING)
		return 0;
	if (!wait_for_completion_timeout(&appledwc->sleep_stop_done,
					 msecs_to_jiffies(DWC3_APPLE_SLEEP_STOP_PREPARE_MS))) {
		dev_err(dev, "USB preparation not over after %d ms: sleep refused\n",
			DWC3_APPLE_SLEEP_STOP_PREPARE_MS);
		return -EBUSY;
	}
	ret = appledwc->sleep_stop_ret;
	dev_dbg(dev, "USB preparation ended %lld ms after admission: %d\n",
		ktime_ms_delta(appledwc->sleep_stop_end, appledwc->sleep_stop_start), ret);
	/* Reserve -EAGAIN for the syscore retry, not a completed worker error. */
	if (ret)
		return ret == -EAGAIN ? -EBUSY : ret;
	spin_lock_irqsave(&appledwc->sleep_pm_lock, flags);
	appledwc->sleep_phase = DWC3_APPLE_SLEEP_PREPARED;
	if (appledwc->sleep_syscore) {
		dev_pm_syscore_device(dev, true);
		/*
		 * Unwind genpd's prepared_count before the retry skips this syscore device.
		 * Returning success here would skip genpd complete and leak that count.
		 */
		ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&appledwc->sleep_pm_lock, flags);
	return ret;
}

static const struct dev_pm_ops dwc3_apple_pm_ops = {
	.prepare = pm_sleep_ptr(dwc3_apple_pm_prepare),
};

/* Called under the device lock, but never with the glue lock held across a work drain. */
static void dwc3_apple_pm_quiesce(struct dwc3_apple *appledwc)
{
	devm_release_action(appledwc->dev, dwc3_apple_pm_notifier_remove, appledwc);
	scoped_guard(mutex, &appledwc->lock)
		appledwc->shutting_down = true;
	cancel_work_sync(&appledwc->sleep_start_work);
	cancel_work_sync(&appledwc->sleep_stop_work);
	cancel_delayed_work_sync(&appledwc->hub_check);
	guard(mutex)(&appledwc->lock);
	/* Cancelling a queued finish must still release the PHY's retained-sleep token. */
	if (appledwc->sleep_in_place) {
		appledwc->sleep_in_place = false;
		apple_sleep_usb_set_wake_ports(0);
		apple_atcphy_sleep_finish_async(appledwc->usb2_phy, false);
	}
}

/* A fixed hub's node gives the root port it sits on in its reg, 1 if it doesn't. */
static bool dwc3_apple_has_fixed_hub(struct device *dev, u32 *port)
{
	struct device_node *hub __free(device_node) =
		of_get_available_child_by_name(dev->of_node, "hub");

	*port = 1;
	if (hub)
		of_property_read_u32(hub, "reg", port);

	return hub;
}

static int dwc3_apple_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_apple *appledwc;
	int ret;

	appledwc = devm_kzalloc(&pdev->dev, sizeof(*appledwc), GFP_KERNEL);
	if (!appledwc)
		return -ENOMEM;

	appledwc->dev = &pdev->dev;
	appledwc->fixed_hub = dwc3_apple_has_fixed_hub(dev, &appledwc->hub_port);
	appledwc->host_only = dwc3_apple_host_only(dev);
	appledwc->dt_dis_u3_susphy = device_property_read_bool(dev, "snps,dis_u3_susphy_quirk");
	appledwc->sleep_ddr = device_property_read_bool(dev, "apple,sleep-ddr");
	/* The PHY driver and the DARTs follow runtime PM on these SoCs. */
	appledwc->idle_rpm = of_device_is_compatible(dev->of_node, "apple,t6040-dwc3") ||
			     of_device_is_compatible(dev->of_node, "apple,t8132-dwc3");
	mutex_init(&appledwc->lock);
	spin_lock_init(&appledwc->sleep_pm_lock);
	/* A never-connected port has no generic core probe to publish this. */
	appledwc->dwc.dev = dev;
	platform_set_drvdata(pdev, &appledwc->dwc);

	appledwc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(appledwc->reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(appledwc->reset),
				     "Failed to get reset control\n");

	appledwc->usb2_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(appledwc->usb2_phy))
		return dev_err_probe(dev, PTR_ERR(appledwc->usb2_phy),
				     "Failed to get the USB2 PHY\n");

	/*
	 * Older T8132 device trees and the T6040 ports without a USB3 PHY
	 * expose only USB2; a NULL PHY is a no-op.
	 */
	if (of_device_is_compatible(dev->of_node, "apple,t8132-dwc3") ||
	    of_device_is_compatible(dev->of_node, "apple,t6040-dwc3"))
		appledwc->usb3_phy = devm_phy_optional_get(dev, "usb3-phy");
	else
		appledwc->usb3_phy = devm_phy_get(dev, "usb3-phy");
	if (IS_ERR(appledwc->usb3_phy))
		return dev_err_probe(dev, PTR_ERR(appledwc->usb3_phy),
				     "Failed to get the USB3 PHY\n");

	ret = reset_control_assert(appledwc->reset);
	if (ret) {
		dev_err(&pdev->dev, "Failed to assert reset, err=%d\n", ret);
		return ret;
	}

	appledwc->mmio_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dwc3-core");
	if (!appledwc->mmio_resource) {
		dev_err(dev, "Failed to get DWC3 MMIO\n");
		return -EINVAL;
	}

	appledwc->apple_regs = devm_platform_ioremap_resource_byname(pdev, "dwc3-apple");
	if (IS_ERR(appledwc->apple_regs))
		return dev_err_probe(dev, PTR_ERR(appledwc->apple_regs),
				     "Failed to map Apple-specific MMIO\n");

	ret = dwc3_apple_parse_tunables(appledwc, pdev);
	if (ret)
		return ret;

	/*
	 * Like a PCI xHCI controller, allow USB devices to wake the system by default. This has
	 * to be set before dwc3_core_probe(), which hands it on to the xHCI device.
	 */
	ret = devm_device_init_wakeup(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set up wakeup\n");

	/*
	 * On this platform, DWC3 can only be brought up after parts of the PHY have been
	 * initialized with knowledge of the target mode and cable orientation from typec_set_mux.
	 * Since this has not happened here we cannot setup DWC3 yet and instead defer this until
	 * the first cable is connected. See the documentation for enum dwc3_apple_state for
	 * details.
	 */
	appledwc->state = DWC3_APPLE_PROBE_PENDING;
	ret = devm_delayed_work_autocancel(dev, &appledwc->hub_check, dwc3_apple_hub_check);
	if (ret)
		return ret;
	init_completion(&appledwc->sleep_stop_done);
	ret = devm_work_autocancel(dev, &appledwc->sleep_stop_work, dwc3_apple_sleep_stop_work);
	if (ret)
		return ret;
	ret = devm_work_autocancel(dev, &appledwc->sleep_start_work, dwc3_apple_sleep_start_work);
	if (ret)
		return ret;
	ret = devm_work_autocancel(dev, &appledwc->sleep_tunnel_work, dwc3_apple_tunnel_work);
	if (ret)
		return ret;
	appledwc->pm_nb.notifier_call = dwc3_apple_pm_notify;
	ret = register_pm_notifier(&appledwc->pm_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, dwc3_apple_pm_notifier_remove, appledwc);
	if (ret)
		return ret;
	/*
	 * Until the first cable there is no core to keep powered: let the controller
	 * runtime-suspend, and its DARTs with it, so that its power domain can drop.
	 * Tested on T8132; on the T6041 (M4 Max) the never-plugged ports are the usual case.
	 */
	if (appledwc->idle_rpm && !appledwc->fixed_hub &&
	    (of_device_is_compatible(dev->of_node, "apple,t8132-dwc3") ||
	     of_machine_is_compatible("apple,t6041"))) {
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
		appledwc->pre_core_rpm = true;
	}
	ret = dwc3_apple_setup_role_switch(appledwc);
	if (ret) {
		if (appledwc->pre_core_rpm)
			pm_runtime_disable(dev);
		return dev_err_probe(&pdev->dev, ret, "Failed to setup role switch\n");
	}

	if (!appledwc->fixed_hub) {
		WRITE_ONCE(appledwc->sleep_pm_online, true);
		return 0;
	}

	/*
	 * The hub is the controller's permanent USB2 peer, and the PHY has kept
	 * its USB2 path up for it since its own probe, so there is nothing to
	 * wait for: bring the host up now.
	 */
	scoped_guard(mutex, &appledwc->lock)
		ret = dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	if (ret) {
		/* A failed cleanup remains visible even when probe cannot be retried. */
		if (appledwc->core_active || appledwc->reset_pending)
			dev_err(dev, "Host startup left pending PHY/reset cleanup\n");
		if (appledwc->state != DWC3_APPLE_PROBE_PENDING)
			dwc3_core_remove_no_shutdown(&appledwc->dwc);
		usb_role_switch_unregister(appledwc->role_sw);
		return dev_err_probe(dev, ret, "Failed to start host mode\n");
	}

	WRITE_ONCE(appledwc->sleep_pm_online, true);
	return 0;
}

static void dwc3_apple_remove(struct platform_device *pdev)
{
	struct dwc3 *dwc = platform_get_drvdata(pdev);
	struct dwc3_apple *appledwc = to_dwc3_apple(dwc);

	dwc3_apple_pm_quiesce(appledwc);
	guard(mutex)(&appledwc->lock);

	usb_role_switch_unregister(appledwc->role_sw);

	/*
	 * If we're still in DWC3_APPLE_PROBE_PENDING we never got any cable connected event and
	 * dwc3_core_probe was never called and there's hence no need to call dwc3_core_remove.
	 * dwc3_apple_exit can be called unconditionally because it checks the state itself.
	 */
	if (dwc3_apple_exit(appledwc))
		dev_err(appledwc->dev, "Removal left pending PHY/reset cleanup\n");
	if (appledwc->state != DWC3_APPLE_PROBE_PENDING)
		dwc3_core_remove_no_shutdown(&appledwc->dwc);
	if (appledwc->pre_core_rpm)
		pm_runtime_disable(appledwc->dev);
}

/* Stop the consumer and release its PHY epoch before the supplier shutdown. */
static void dwc3_apple_shutdown(struct platform_device *pdev)
{
	struct dwc3_apple *appledwc = to_dwc3_apple(platform_get_drvdata(pdev));
	int ret;

	dwc3_apple_pm_quiesce(appledwc);
	if (!of_device_is_compatible(appledwc->dev->of_node, "apple,t8132-dwc3"))
		return;
	guard(mutex)(&appledwc->lock);
	ret = dwc3_apple_exit(appledwc);
	if (ret)
		dev_err(appledwc->dev,
			"Shutdown retains incomplete PHY/reset ownership: %d\n", ret);
}

static const struct of_device_id dwc3_apple_of_match[] = {
	{ .compatible = "apple,t8132-dwc3" },
	{ .compatible = "apple,t8103-dwc3" },
	{}
};
MODULE_DEVICE_TABLE(of, dwc3_apple_of_match);

static struct platform_driver dwc3_apple_driver = {
	.probe		= dwc3_apple_probe,
	.remove		= dwc3_apple_remove,
	.shutdown	= dwc3_apple_shutdown,
	.driver		= {
		.name	= "dwc3-apple",
		.of_match_table	= dwc3_apple_of_match,
		.pm	= pm_sleep_ptr(&dwc3_apple_pm_ops),
	},
};

module_platform_driver(dwc3_apple_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("DesignWare DWC3 Apple Silicon Glue Driver");
