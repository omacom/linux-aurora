// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon DWC3 Glue driver
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on:
 *  - dwc3-qcom.c Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *  - dwc3-of-simple.c Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/reset.h>
#include <linux/usb/phy.h>

#include "dwc3-apple-state.h"
#include "glue.h"

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
 * enum dwc3_apple_state is defined in dwc3-apple-state.h.
 */

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
 * @lock: Mutex for synchronizing access
 * @state: Current state of the controller, see enum dwc3_apple_state and the comment above
 * @core_active: Core still owns initialized PHYs
 * @core_exit_pending: Event buffers have been cleaned for this shutdown
 * @reset_pending: Controller reset must complete before another start
 * @usb2_power_off_pending: Core-owned USB2 references still to release
 * @usb3_power_off_pending: Core-owned USB3 references still to release
 */
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

	struct mutex lock;

	enum dwc3_apple_state state;
	bool core_active;
	bool core_exit_pending;
	bool reset_pending;
	unsigned long usb2_power_off_pending;
	unsigned long usb3_power_off_pending;
};

#define to_dwc3_apple(d) container_of((d), struct dwc3_apple, dwc)

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

static void dwc3_apple_set_ptrcap(struct dwc3_apple *appledwc, u32 mode)
{
	guard(spinlock_irqsave)(&appledwc->dwc.lock);
	dwc3_set_prtcap(&appledwc->dwc, mode, false);
}

static int dwc3_apple_core_probe(struct dwc3_apple *appledwc)
{
	struct dwc3_probe_data probe_data = {};
	void *group;
	int ret;

	lockdep_assert_held(&appledwc->lock);
	WARN_ON_ONCE(appledwc->state != DWC3_APPLE_PROBE_PENDING);

	appledwc->dwc.dev = appledwc->dev;
	probe_data.dwc = &appledwc->dwc;
	probe_data.res = appledwc->mmio_resource;
	probe_data.ignore_clocks_and_resets = true;
	probe_data.skip_core_init_mode = true;
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

	appledwc->state = DWC3_APPLE_NO_CABLE;
	return 0;
}

static int dwc3_apple_core_init(struct dwc3_apple *appledwc)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_PROBE_PENDING:
		ret = dwc3_apple_core_probe(appledwc);
		if (ret)
			dev_err(appledwc->dev, "Failed to probe DWC3 Core, err=%d\n", ret);
		break;
	case DWC3_APPLE_NO_CABLE:
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
	dev_pm_syscore_device(appledwc->dev, false);

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
	if (mode != PHY_MODE_INVALID) {
		ret = phy_set_mode(appledwc->usb2_phy, mode);
		if (ret)
			dev_warn(appledwc->dev, "Failed to set the USB2 PHY mode, err=%d\n", ret);
	}

	ret = reset_control_deassert(appledwc->reset);
	if (ret) {
		dev_err(appledwc->dev, "Failed to deassert reset, err=%d\n", ret);
		return ret;
	}

	appledwc->reset_pending = true;
	/* Keep register access powered until both PHY shutdown and reset finish. */
	dev_pm_syscore_device(appledwc->dev, true);
	ret = dwc3_apple_core_init(appledwc);
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
	dwc3_apple_setup_cio(appledwc);
	appledwc->core_active = true;
	appledwc->usb2_power_off_pending = BIT(appledwc->dwc.num_usb2_ports) - 1;
	appledwc->usb3_power_off_pending = BIT(appledwc->dwc.num_usb3_ports) - 1;

	return 0;
}

/* Select the role in the core and route the USB3 PHY, before xhci or the gadget is started */
static void dwc3_apple_set_role(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	bool host = state == DWC3_APPLE_HOST;
	int ret;

	lockdep_assert_held(&appledwc->lock);

	appledwc->dwc.dr_mode = host ? USB_DR_MODE_HOST : USB_DR_MODE_PERIPHERAL;
	dwc3_apple_set_ptrcap(appledwc, host ? DWC3_GCTL_PRTCAP_HOST : DWC3_GCTL_PRTCAP_DEVICE);
	/*
	 * This platform requires SUSPHY to be enabled here already in order to properly
	 * configure the PHY and switch dwc3's PIPE interface to USB3 PHY. The USB2 PHY's
	 * device mode has already been selected in dwc3_apple_core_start(); its host mode
	 * is selected by the xHCI root hub as it starts.
	 */
	dwc3_enable_susphy(&appledwc->dwc, true);
	ret = phy_set_mode(appledwc->usb3_phy, host ? PHY_MODE_USB_HOST : PHY_MODE_USB_DEVICE);
	if (ret)
		dev_warn(appledwc->dev, "USB3 PHY setup failed, USB2 only, err=%d\n", ret);
}

static int dwc3_apple_init(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	int ret, ret_reset;

	lockdep_assert_held(&appledwc->lock);

	ret = dwc3_apple_core_start(appledwc, state);
	if (ret)
		return ret;

	dwc3_apple_set_role(appledwc, state);

	switch (state) {
	case DWC3_APPLE_HOST:
		ret = dwc3_host_init(&appledwc->dwc);
		if (ret) {
			dev_err(appledwc->dev, "Failed to initialize host, ret=%d\n", ret);
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
	dev_pm_syscore_device(appledwc->dev, true);
	return 0;

core_exit:
	ret_reset = dwc3_apple_core_stop(appledwc);
	if (ret_reset)
		dev_warn(appledwc->dev, "Failed to finish core shutdown, err=%d\n", ret_reset);

	return ret;
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
		dwc3_host_exit(&appledwc->dwc);
		appledwc->state = DWC3_APPLE_NO_CABLE;
		break;
	}

	return dwc3_apple_core_stop(appledwc);
}

static int dwc3_usb_role_switch_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);
	int ret;

	guard(mutex)(&appledwc->lock);

	/* A controller behind a fixed hub is a host regardless of the connectors */
	if (appledwc->fixed_hub)
		return 0;

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
		return 0;
	case USB_ROLE_HOST:
		return dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	case USB_ROLE_DEVICE:
		return dwc3_apple_init(appledwc, DWC3_APPLE_DEVICE);
	default:
		dev_err(appledwc->dev, "Invalid target role: %d\n", role);
		return -EINVAL;
	}
}

static enum usb_role dwc3_usb_role_switch_get(struct usb_role_switch *sw)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);

	guard(mutex)(&appledwc->lock);

	switch (appledwc->state) {
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

static bool dwc3_apple_has_fixed_hub(struct device *dev)
{
	struct device_node *hub __free(device_node) =
		of_get_available_child_by_name(dev->of_node, "hub");

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
	appledwc->fixed_hub = dwc3_apple_has_fixed_hub(dev);
	mutex_init(&appledwc->lock);

	appledwc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(appledwc->reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(appledwc->reset),
				     "Failed to get reset control\n");

	appledwc->usb2_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(appledwc->usb2_phy))
		return dev_err_probe(dev, PTR_ERR(appledwc->usb2_phy),
				     "Failed to get the USB2 PHY\n");

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

	/*
	 * dwc3_core_probe() may not run until the first cable connects, but
	 * remove() needs the driver data on every bound device.
	 * dwc3_core_probe() stores this same pointer.
	 */
	platform_set_drvdata(pdev, &appledwc->dwc);

	ret = dwc3_apple_setup_role_switch(appledwc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to setup role switch\n");

	if (!appledwc->fixed_hub)
		return 0;

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

	return 0;
}

static void dwc3_apple_remove(struct platform_device *pdev)
{
	struct dwc3 *dwc = platform_get_drvdata(pdev);
	struct dwc3_apple *appledwc = to_dwc3_apple(dwc);

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
}

static const struct of_device_id dwc3_apple_of_match[] = {
	{ .compatible = "apple,t8103-dwc3" },
	{}
};
MODULE_DEVICE_TABLE(of, dwc3_apple_of_match);

static struct platform_driver dwc3_apple_driver = {
	.probe		= dwc3_apple_probe,
	.remove		= dwc3_apple_remove,
	.driver		= {
		.name	= "dwc3-apple",
		.of_match_table	= dwc3_apple_of_match,
	},
};

module_platform_driver(dwc3_apple_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("DesignWare DWC3 Apple Silicon Glue Driver");
