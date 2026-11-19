// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8152 (M6) Type-C PHY: the SuperSpeed / USB4 side of an ATC port.
 *
 * The USB2 side of the same port is apple,t8152-usb2-phy (usb2-t8152.c), the DisplayPort side dptx-t8152.c. This driver brings the PHY block
 * (ADT atc-phy reg[2]) up for one transport at a time and routes the dwc3 PIPE (usb-drd reg[3], the "pipehandler") to it:
 *
 *   PHY index 0, "usb3": USB3 transport. set_mode(HOST) powers the PHY if needed and routes the PIPE to it, which is the only call the dwc3-apple glue makes
 *                       (like atc.c there is no power_on: dwc3 would call it before its core reset; power_off shuts the PHY down).
 *   PHY index 1, "cio":  CIO / USB4 transport for the thunderbolt_apple driver (phy_power_on / phy_power_off).
 *
 * A transport change is shutdown + full bring-up, as in the vendor driver (it has no incremental path). The cable orientation comes from the Type-C
 * switch; it takes effect at the next bring-up, or immediately (re-run) while the PHY is up.
 *
 * The register sequences are in atc-t8152-seq.h; the tunable lists are not part of the driver, the boot loader passes them as
 * apple,tunable-<name> properties (see Documentation/devicetree/bindings/phy/apple,t8152-atcphy.yaml).
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/usb/typec_mux.h>

#include "atc-t8152-seq.h"

#define ATC_CORE_MIN_SIZE	(T8152_POWER + 0x40)
#define ATC_PIPEH_MIN_SIZE	0x30

enum atc_mode {
	ATC_OFF,
	ATC_USB3,
	ATC_CIO,
};

struct apple_t8152_atc {
	struct device *dev;
	struct t8152_regs r;
	struct apple_tunable *tun[T8152_NUM_TUNABLES];
	struct phy *usb3;
	struct phy *cio;
	struct typec_switch_dev *sw;
	struct mutex lock;	/* serialises mode changes, bring-up, shutdown and the PIPE route */
	enum atc_mode mode;
	enum atc_mode want_mode;	/* transport the consumers last asked for; ATC_OFF after a deliberate power_off */
	bool swap;		/* cable orientation is reversed */
	bool restart;		/* an orientation restart did not complete; retried on the next orientation event */
	bool pipe_usb3;		/* the dwc3 PIPE is routed to the PHY */
	bool route_locked;	/* the route sequence finished with the handler lock released */
	bool route_ready;	/* the routed PIPE reported PIPE_PHY_READY */
	unsigned int cio_lanes;	/* lane mask of the CIO bring-up */
};

static int atc_up(struct apple_t8152_atc *atc, enum atc_mode mode)
{
	int ret;

	/* The efuse calibration lists are the minimum without which the PHY would run uncalibrated. */
	if (!atc->tun[t8152_seq_efuse[0]] || !atc->tun[t8152_seq_efuse[1]]) {
		dev_err(atc->dev, "no PHY calibration tunables passed by the boot loader\n");
		return -ENODEV;
	}

	if (mode == ATC_USB3) {
		ret = t8152_up_usb3(&atc->r, atc->tun, atc->swap);
		if (!ret)
			ret = t8152_pipe_override_enable(&atc->r);
	} else {
		ret = t8152_up_cio(&atc->r, atc->tun, atc->swap, atc->cio_lanes);
	}
	if (ret) {
		dev_err(atc->dev, "%s bring-up failed: %d\n", mode == ATC_USB3 ? "USB3" : "CIO", ret);
		t8152_shutdown(&atc->r);
		return ret;
	}
	atc->mode = mode;
	atc->restart = false;
	return 0;
}

static int atc_down(struct apple_t8152_atc *atc)
{
	int ret;

	if (atc->mode == ATC_OFF)
		return 0;
	if (atc->pipe_usb3) {
		/* a failed park leaves the controller routed to the PHY: keep the route
		 * recorded and the mode set, so a later down retries the park
		 */
		ret = t8152_pipehandler_park(&atc->r);
		if (ret) {
			dev_err(atc->dev, "failed to park the PIPE: %d\n", ret);
			atc->route_ready = false;
			return ret;
		}
		atc->pipe_usb3 = false;
	}
	atc->route_locked = false;
	atc->route_ready = false;
	ret = t8152_shutdown(&atc->r);
	if (ret)
		dev_warn(atc->dev, "PHY power-down did not settle: %d\n", ret);
	atc->mode = ATC_OFF;
	return ret;
}

static int atc_route_usb3(struct apple_t8152_atc *atc)
{
	int ret = 0;
	bool routed = false;

	/* the route stands, the handler lock is released and the PIPE is confirmed ready */
	if (atc->pipe_usb3 && atc->route_locked && atc->route_ready)
		return 0;
	if (!atc->pipe_usb3 || !atc->route_locked) {
		ret = t8152_pipehandler_usb3_host(&atc->r, &routed);
		/* a failed retry never un-records a standing route */
		if (routed)
			atc->pipe_usb3 = true;
		if (!atc->pipe_usb3) {
			atc->route_locked = false;
			return ret;	/* the mux never moved and no route stands */
		}
		atc->route_locked = !ret;
		/* a lock-release timeout leaves the route standing; route_locked stays false, so a
		 * retry re-runs the host sequence, whose entry re-checks the lock ack
		 */
		if (ret)
			return ret;
	}
	/* the PIPE points at the PHY: confirm it; a retry repeats only this wait */
	ret = t8152_pipehandler_usb3_ready(&atc->r);
	atc->route_ready = !ret;
	return ret;
}

static int atc_usb3_power_off(struct phy *phy)
{
	struct apple_t8152_atc *atc = phy_get_drvdata(phy);

	guard(mutex)(&atc->lock);

	/* a deliberate power_off withdraws this transport's request only */
	if (atc->want_mode == ATC_USB3)
		atc->want_mode = ATC_OFF;
	if (atc->mode != ATC_USB3)
		return 0;
	return atc_down(atc);
}

/* dwc3-apple only calls set_mode() on its USB3 PHY (after the core reset), so this brings the PHY up as well when nothing else has. */
static int atc_usb3_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct apple_t8152_atc *atc = phy_get_drvdata(phy);
	int ret;

	if (mode != PHY_MODE_USB_HOST)
		return -EOPNOTSUPP;

	guard(mutex)(&atc->lock);

	if (atc->mode == ATC_CIO)
		return -EBUSY;
	atc->want_mode = ATC_USB3;
	if (atc->mode == ATC_OFF) {
		ret = atc_up(atc, ATC_USB3);
		if (ret)
			return ret;
	}
	return atc_route_usb3(atc);
}

static const struct phy_ops atc_usb3_ops = {
	.power_off = atc_usb3_power_off,
	.set_mode = atc_usb3_set_mode,
	.owner = THIS_MODULE,
};

static int atc_cio_power_on(struct phy *phy)
{
	struct apple_t8152_atc *atc = phy_get_drvdata(phy);

	guard(mutex)(&atc->lock);

	if (atc->mode == ATC_CIO)
		return 0;
	if (atc->mode == ATC_USB3)
		return -EBUSY;
	atc->want_mode = ATC_CIO;
	return atc_up(atc, ATC_CIO);
}

static int atc_cio_power_off(struct phy *phy)
{
	struct apple_t8152_atc *atc = phy_get_drvdata(phy);

	guard(mutex)(&atc->lock);

	/* a deliberate power_off withdraws this transport's request only */
	if (atc->want_mode == ATC_CIO)
		atc->want_mode = ATC_OFF;
	if (atc->mode != ATC_CIO)
		return 0;
	return atc_down(atc);
}

static const struct phy_ops atc_cio_ops = {
	.power_on = atc_cio_power_on,
	.power_off = atc_cio_power_off,
	.owner = THIS_MODULE,
};

static struct phy *atc_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct apple_t8152_atc *atc = dev_get_drvdata(dev);

	if (args->args_count != 1 || args->args[0] > 1)
		return ERR_PTR(-EINVAL);
	return args->args[0] ? atc->cio : atc->usb3;
}

static int atc_sw_set(struct typec_switch_dev *sw, enum typec_orientation orientation)
{
	struct apple_t8152_atc *atc = typec_switch_get_drvdata(sw);
	enum atc_mode mode;
	bool routed, swap;
	int ret = 0;

	if (orientation == TYPEC_ORIENTATION_NONE)
		return 0;
	swap = orientation == TYPEC_ORIENTATION_REVERSE;

	guard(mutex)(&atc->lock);

	/* a pending restart retries here even when the orientation did not change */
	if (swap == atc->swap && !atc->restart)
		return 0;
	atc->swap = swap;
	if (atc->mode == ATC_OFF) {
		/* a failed orientation restart: bring the PHY back for the remembered transport
		 * (the fixed-role dwc3 calls phy_set_mode only once, at core start)
		 */
		if (!atc->restart || atc->want_mode == ATC_OFF)
			return 0;
		ret = atc_up(atc, atc->want_mode);
		if (ret)
			return ret;	/* the restart stays pending */
		if (atc->want_mode == ATC_USB3)
			ret = atc_route_usb3(atc);
		if (ret)
			atc->restart = true;
		return ret;
	}

	/* The vendor driver re-runs shutdown + bring-up when the orientation changes
	 * with the PHY up.
	 */
	mode = atc->mode;
	routed = atc->pipe_usb3;
	ret = atc_down(atc);
	if (ret) {
		/* the PHY is still up and routed; the next orientation event retries */
		atc->restart = true;
		return ret;
	}
	ret = atc_up(atc, mode);
	/* route for the USB3 transport even when the previous route attempt never moved the
	 * mux: otherwise the PHY comes back up with the PIPE still on the dummy backend
	 */
	if (!ret && (routed || atc->mode == ATC_USB3))
		ret = atc_route_usb3(atc);
	/* the PHY is down again or half-routed; the next orientation event retries */
	if (ret)
		atc->restart = true;
	return ret;
}

static void atc_pm_put(void *data)
{
	pm_runtime_put_sync(data);
}

static void atc_sw_unregister(void *data)
{
	typec_switch_unregister(data);
}

static int atc_dwc3_window(struct device_node *dwc3, const char *name, struct resource *res)
{
	int idx = of_property_match_string(dwc3, "reg-names", name);

	return idx < 0 ? idx : of_address_to_resource(dwc3, idx, res);
}

static int atc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *core, pipeh, dcore, dapple, blk;
	resource_size_t cfg_phys;
	struct device_node *dwc3;
	struct apple_t8152_atc *atc;
	struct typec_switch_desc sw_desc = { };
	struct phy_provider *provider;
	unsigned int i;
	int ret;

	atc = devm_kzalloc(dev, sizeof(*atc), GFP_KERNEL);
	if (!atc)
		return -ENOMEM;
	atc->dev = dev;
	atc->cio_lanes = T8152_LANES_BOTH;
	mutex_init(&atc->lock);
	platform_set_drvdata(pdev, atc);

	/*
	 * The windows are mapped without reserving them. The PHY core window of the ADT also contains the USB DART banks (+0x20000, +0x40000), which the
	 * platform bus nests inside it. The pipe handler and the PIPE_PHY_READY word are already resources of the dwc3 node (a second node with the same or
	 * a partly overlapping range cannot be added), so both are found through the dwc3 node.
	 */
	core = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core");
	if (!core)
		return dev_err_probe(dev, -EINVAL, "core window is required\n");
	if (resource_size(core) < ATC_CORE_MIN_SIZE)
		return dev_err_probe(dev, -EINVAL, "core window too small\n");
	atc->r.core = devm_ioremap(dev, core->start, resource_size(core));
	if (!atc->r.core)
		return -ENOMEM;

	dwc3 = of_parse_phandle(dev->of_node, "apple,dwc3", 0);
	if (!dwc3)
		return dev_err_probe(dev, -EINVAL, "apple,dwc3 is required\n");
	ret = atc_dwc3_window(dwc3, "pipe-handler", &pipeh);
	if (!ret)
		ret = atc_dwc3_window(dwc3, "dwc3-core", &dcore);
	if (!ret)
		ret = atc_dwc3_window(dwc3, "dwc3-apple", &dapple);
	of_node_put(dwc3);
	if (ret)
		return dev_err_probe(dev, ret, "the dwc3 node lacks pipe-handler, dwc3-core or dwc3-apple\n");
	if (resource_size(&pipeh) < ATC_PIPEH_MIN_SIZE)
		return dev_err_probe(dev, -EINVAL, "pipehandler window too small\n");
	atc->r.pipeh = devm_ioremap(dev, pipeh.start, resource_size(&pipeh));
	if (!atc->r.pipeh)
		return -ENOMEM;

	/* usb-drd reg[1] of the ADT, 0x10000 above the dwc3 core, holds PIPE_PHY_READY; the dwc3 node maps it as the tail of its "dwc3-apple" window */
	cfg_phys = dcore.start + T8152_CFG_OFFSET;
	if (cfg_phys < dapple.start || cfg_phys + T8152_CFG_PHY_READY_REG + sizeof(u32) - 1 > dapple.end)
		return dev_err_probe(dev, -EINVAL, "the PIPE_PHY_READY word is outside the dwc3-apple window\n");
	atc->r.cfg = devm_ioremap(dev, cfg_phys, T8152_CFG_PHY_READY_REG + sizeof(u32));
	if (!atc->r.cfg)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "apple,cio-lanes", &atc->cio_lanes);
	if (!atc->cio_lanes || atc->cio_lanes > T8152_LANES_BOTH)
		return dev_err_probe(dev, -EINVAL, "apple,cio-lanes must be 1, 2 or 3\n");

	for (i = 0; i < T8152_NUM_TUNABLES; i++) {
		blk = DEFINE_RES_MEM(core->start + t8152_tunables[i].off, t8152_tunables[i].len);
		if (t8152_tunables[i].off + t8152_tunables[i].len > resource_size(core))
			return dev_err_probe(dev, -EINVAL, "%s lies outside the core window\n", t8152_tunables[i].prop);
		atc->tun[i] = devm_apple_tunable_parse(dev, dev->of_node, t8152_tunables[i].prop, &blk);
		if (IS_ERR(atc->tun[i])) {
			if (PTR_ERR(atc->tun[i]) != -ENOENT)
				return dev_err_probe(dev, PTR_ERR(atc->tun[i]), "invalid %s\n", t8152_tunables[i].prop);
			atc->tun[i] = NULL;	/* like the vendor driver, a list the firmware does not carry is skipped */
		}
	}

	if (of_property_present(dev->of_node, "power-domains")) {
		struct dev_pm_domain_list *domains;

		/* one domain is attached by the platform bus, several by index (the PHY, AON and ATC domains of the port) */
		if (of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells") > 1) {
			ret = devm_pm_domain_attach_list(dev, NULL, &domains);
			if (ret < 0)
				return dev_err_probe(dev, ret, "failed to attach the PHY power domains\n");
		}
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
		/*
		 * The domains stay on for the whole driver lifetime: the core, pipe
		 * handler and PIPE_PHY_READY windows are touched from the phy ops and
		 * the Type-C switch, which hold no runtime-PM reference of their own,
		 * and the ATC USB domain (psd_atc*_usb) is shared with the dwc3 node,
		 * whose runtime PM may gate it on its own. A get/put in
		 * atc_up/atc_down would gate the banks between transports and under
		 * an idle dwc3 (e.g. while the CIO PHY serves the ACIO controller).
		 */
		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to power the PHY domains\n");
		ret = devm_add_action_or_reset(dev, atc_pm_put, dev);
		if (ret)
			return ret;
	}

	atc->usb3 = devm_phy_create(dev, NULL, &atc_usb3_ops);
	if (IS_ERR(atc->usb3))
		return PTR_ERR(atc->usb3);
	phy_set_drvdata(atc->usb3, atc);
	atc->cio = devm_phy_create(dev, NULL, &atc_cio_ops);
	if (IS_ERR(atc->cio))
		return PTR_ERR(atc->cio);
	phy_set_drvdata(atc->cio, atc);

	if (of_property_read_bool(dev->of_node, "orientation-switch")) {
		sw_desc.drvdata = atc;
		sw_desc.fwnode = dev->fwnode;
		sw_desc.set = atc_sw_set;
		atc->sw = typec_switch_register(dev, &sw_desc);
		if (IS_ERR(atc->sw))
			return dev_err_probe(dev, PTR_ERR(atc->sw), "failed to register the Type-C switch\n");
		ret = devm_add_action_or_reset(dev, atc_sw_unregister, atc->sw);
		if (ret)
			return ret;
	}

	provider = devm_of_phy_provider_register(dev, atc_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static void atc_remove(struct platform_device *pdev)
{
	struct apple_t8152_atc *atc = platform_get_drvdata(pdev);

	guard(mutex)(&atc->lock);
	/* a failed park leaves the route standing and the PHY powered: shutting the PHY
	 * down under a controller that still points at it is worse, so the failure is
	 * only logged (the driver is going away, nothing retries the park)
	 */
	if (atc_down(atc))
		dev_err(atc->dev, "the PIPE did not park; the PHY is left up and routed\n");
}

static const struct of_device_id atc_match[] = {
	{ .compatible = "apple,t8152-atcphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, atc_match);

static struct platform_driver atc_driver = {
	.probe = atc_probe,
	.remove = atc_remove,
	.driver = {
		.name = "apple-t8152-atcphy",
		.of_match_table = atc_match,
	},
};
module_platform_driver(atc_driver);

MODULE_DESCRIPTION("Apple T8152 Type-C PHY (SuperSpeed and USB4 lanes)");
MODULE_LICENSE("Dual MIT/GPL");
