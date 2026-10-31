// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC CIO (USB4/Thunderbolt) block reset driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * The CIO blocks have a reset inside the power manager (PMGR) that
 * has to be deasserted before their co-processor can be booted. On
 * t8103 each port comes with a dedicated register page while t600x
 * uses a single register shared by all ports of a die inside the PMGR
 * MMIO region.
 *
 * Only the deassert direction is implemented: a request to assert returns
 * immediately on every SoC, hence the missing .assert op here. The register
 * layout is not shared between generations:
 *
 *	SoC	regmap	offset		request		busy
 *	t8103	0	0x84000 + 0x4000 * port	BIT(0), rmw	BIT(1)
 *	t8112	0x63	0x8000 / 0x4000	BIT(0), rmw	BIT(1)
 *	t8122	0x62	0x8		BIT(slot)	BIT(16 + slot)
 *	t8142	0x62	0x30		BIT(2 * port)	BIT(2 * port + 1)
 *	t6000	0	0x1a034		BIT(port)	BIT(16 + port)
 *	t6020	0	0xa02c		BIT(port)	BIT(16 + port)
 *	t6030	0x4c	0x8		BIT(slot)	BIT(16 + slot)
 *	t6031	?	0x2028		BIT(port)	BIT(16 + port)
 *	t6041	?	0x2024		BIT(slot)	BIT(16 + slot)
 *	t6050	?	0x20060		BIT(2 * port)	BIT(2 * port + 1)
 *
 * t6000, t6020, t6031 and t6050 split slot into a die index and a port with
 * "port = slot % 4", which the device tree expresses as one controller node
 * per die instead. The others index by slot directly.
 *
 * The register-map selector is an index into the PMGR's register windows
 * rather than an address. 0 is the PMGR block itself: t8103 resolves to
 * 0x23b784000 and t6020 to the pmgr node plus 0xa02c, both of which match this
 * driver. On t6030 the selector is 0x4c, which resolves to pmgr reg[34], a
 * separate 16 KiB page at 0x350780000, so the register is 0x350780008. The
 * device tree describes that page as its own syscon. t8122 is unresolved.
 *
 * On t8142, RegMap 0x62 is a separate MMIO region at 0x380780000.
 * The t8132 layout has not been decoded.
 *
 * On t8152 (M6) the request and done registers are separate words in the PMGR
 * block at 0x300780000: BIT(cluster) is written to the request register at
 * +0x68, BIT(cluster) is then polled in the done register at +0x6c (the vendor
 * driver allows 10 s), and the done bit is cleared by writing it back. There
 * is one cluster per port (the pmgr-child cio-cluster property: devices
 * 320..323, CIOn_RECONFIG-V, clusters 0..3), and the node's reg is the offset
 * of the request register in the PMGR syscon (0x80068).
 *
 * Reference firmware polls the busy bits, stores the request, then polls
 * again, with a 192 ms budget. It discards the first poll's result. The
 * t8142 implementation below instead refuses to issue a request while busy.
 */

#include <linux/bits.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#define APPLE_CIO_RESET_POLL_US 100
#define APPLE_CIO_RESET_TIMEOUT_US 100000

#define T8103_CIO_CTRL_STRIDE 0x4000
#define T8103_CIO_CTRL_INIT_REQ BIT(0)
#define T8103_CIO_CTRL_INIT_BUSY BIT(1)
#define T8103_CIO_CTRL_INIT_DONE BIT(2)

/* The node's reg is the request register; the done register follows it. */
#define T8152_CIO_REQ 0x0
#define T8152_CIO_DONE 0x4
/* The vendor driver waits up to 10 s for a cluster to come online. */
#define T8152_CIO_TIMEOUT_US 10000000

#define T6000_CIO_CTRL_INIT_REQ(port) BIT(port)
#define T6000_CIO_CTRL_INIT_BUSY(port) BIT(16 + (port))

#define T8142_CIO_CTRL 0x30
#define T8142_CIO_CTRL_INIT_REQ(port) BIT(2 * (port))
#define T8142_CIO_CTRL_INIT_BUSY(port) BIT(2 * (port) + 1)
#define T8142_CIO_CTRL_BUSY_MASK 0xaa

struct apple_cio_reset;

struct apple_cio_reset_variant {
	unsigned int nr_resets;
	bool pmgr_child;
	int (*deassert)(struct apple_cio_reset *priv, unsigned long id);
};

struct apple_cio_reset {
	struct reset_controller_dev rcdev;
	const struct apple_cio_reset_variant *variant;
	struct device *dev;
	struct regmap *regmap;
	u32 offset;
	struct mutex lock; /* serializes access to the shared t600x register */
};

static int t8103_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 offset = priv->offset + id * T8103_CIO_CTRL_STRIDE;
	u32 val;
	int ret;

	/*
	 * After writing BIT(0) the reset is deasserted and the CIO block comes up.
	 * Once the sequence is done BIT(0) is cleared automatically and then BIT(2)
	 * is set. There's no way to assert the reset again other than cutting the power
	 * to the entire CIO block. Writing the bit again has no effect.
	 *
	 * Wait for a reconfiguration already in flight, as Apple does, and like Apple
	 * carry on regardless of the result. See the t6000 variant for why this is only
	 * a warning.
	 *
	 * Apple also sets the request bit with a read-modify-write rather than a plain
	 * store. That is deliberately not copied here: the poll below waits for the
	 * whole register to read exactly INIT_DONE, which works on every machine this
	 * has run on, and preserving unknown bits could stop that value ever appearing.
	 */
	ret = regmap_read_poll_timeout(priv->regmap, offset, val,
				       !(val & T8103_CIO_CTRL_INIT_BUSY),
				       APPLE_CIO_RESET_POLL_US,
				       APPLE_CIO_RESET_TIMEOUT_US);
	if (ret)
		dev_warn(priv->dev, "CIO %lu still busy before reconfigure request\n", id);

	ret = regmap_write(priv->regmap, offset, T8103_CIO_CTRL_INIT_REQ);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, offset, val,
					val == T8103_CIO_CTRL_INIT_DONE,
					APPLE_CIO_RESET_POLL_US,
					APPLE_CIO_RESET_TIMEOUT_US);
}

static int t6000_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 val;
	int ret;

	/*
	 * Note that even the mutex is likely paranoid and not actually required
	 * here: The way this controller works is that setting bit N immediately
	 * results in bit 16+n being set. Once the reset is fully released both
	 * bits are cleared again but macOS specifically waits for the upper one
	 * to clear before continuing.
	 * Manually clearing bits is ignored and has no effect, the only way to
	 * put the block into reset again is shut down its power domains.
	 * Thus, there's no need to either RMW here or wait before polling.
	 */
	guard(mutex)(&priv->lock);

	/*
	 * Every port on the die shares this register, so a reconfiguration requested for
	 * another port can still be running. The busy bits must clear before the request
	 * is stored, and the result of that wait is advisory, so only
	 * warn here: waiting can only help, while failing would turn a slow reconfigure
	 * into a probe failure.
	 */
	ret = regmap_read_poll_timeout(priv->regmap, priv->offset, val,
				       !(val & T6000_CIO_CTRL_INIT_BUSY(id)),
				       APPLE_CIO_RESET_POLL_US,
				       APPLE_CIO_RESET_TIMEOUT_US);
	if (ret)
		dev_warn(priv->dev, "CIO %lu still busy before reconfigure request\n", id);

	ret = regmap_write(priv->regmap, priv->offset, T6000_CIO_CTRL_INIT_REQ(id));
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, priv->offset, val,
					!(val & T6000_CIO_CTRL_INIT_BUSY(id)),
					APPLE_CIO_RESET_POLL_US,
					APPLE_CIO_RESET_TIMEOUT_US);
}

static int t8142_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 val;
	int ret;

	guard(mutex)(&priv->lock);

	/* Serialize all four ports sharing the request register. */
	ret = regmap_read_poll_timeout(priv->regmap, T8142_CIO_CTRL, val,
				       !(val & T8142_CIO_CTRL_BUSY_MASK),
				       APPLE_CIO_RESET_POLL_US,
				       APPLE_CIO_RESET_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, T8142_CIO_CTRL,
			   T8142_CIO_CTRL_INIT_REQ(id));
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, T8142_CIO_CTRL, val,
				       !(val & T8142_CIO_CTRL_INIT_BUSY(id)),
				       APPLE_CIO_RESET_POLL_US,
				       APPLE_CIO_RESET_TIMEOUT_US);
}

static const struct apple_cio_reset_variant apple_t8142_cio_reset = {
	.nr_resets = 4,
	.deassert = t8142_cio_deassert,
};

static int t8152_cio_deassert(struct apple_cio_reset *priv, unsigned long id)
{
	u32 val;
	int ret;

	guard(mutex)(&priv->lock);

	/* A request for this cluster may still be pending; advisory, as on the other SoCs. */
	ret = regmap_read_poll_timeout(priv->regmap, priv->offset + T8152_CIO_REQ, val,
				       !(val & BIT(id)), APPLE_CIO_RESET_POLL_US,
				       APPLE_CIO_RESET_TIMEOUT_US);
	if (ret)
		dev_warn(priv->dev, "CIO %lu still busy before reconfigure request\n", id);

	ret = regmap_write(priv->regmap, priv->offset + T8152_CIO_REQ, BIT(id));
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(priv->regmap, priv->offset + T8152_CIO_DONE, val,
				       val & BIT(id), APPLE_CIO_RESET_POLL_US,
				       T8152_CIO_TIMEOUT_US);
	if (ret)
		return ret;

	/* The done bit is write-one-to-clear. */
	return regmap_write(priv->regmap, priv->offset + T8152_CIO_DONE, BIT(id));
}

static const struct apple_cio_reset_variant apple_t8103_cio_reset = {
	.nr_resets = 2,
	.deassert = t8103_cio_deassert,
};

static const struct apple_cio_reset_variant apple_t6000_cio_reset = {
	.nr_resets = 4,
	.pmgr_child = true,
	.deassert = t6000_cio_deassert,
};

static const struct apple_cio_reset_variant apple_t8152_cio_reset = {
	.nr_resets = 4,
	.pmgr_child = true,
	.deassert = t8152_cio_deassert,
};

static int apple_cio_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_cio_reset *priv =
		container_of(rcdev, struct apple_cio_reset, rcdev);

	return priv->variant->deassert(priv, id);
}

static const struct reset_control_ops apple_cio_reset_ops = {
	.deassert = apple_cio_reset_deassert,
};

static const struct regmap_config apple_cio_reset_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
};

static int apple_cio_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_cio_reset *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->variant = of_device_get_match_data(dev);
	if (!priv->variant)
		return -ENODEV;
	priv->dev = dev;

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	if (priv->variant->pmgr_child) {
		priv->regmap = syscon_node_to_regmap(dev->of_node->parent);
		if (IS_ERR(priv->regmap))
			return dev_err_probe(dev, PTR_ERR(priv->regmap),
					     "Failed to get parent regmap\n");

		ret = of_property_read_u32(dev->of_node, "reg", &priv->offset);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to read reg offset\n");
	} else {
		void __iomem *base;

		base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(base))
			return PTR_ERR(base);

		priv->regmap = devm_regmap_init_mmio(dev, base,
						     &apple_cio_reset_regmap_config);
		if (IS_ERR(priv->regmap))
			return dev_err_probe(dev, PTR_ERR(priv->regmap),
					     "Failed to init MMIO regmap\n");
	}

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.ops = &apple_cio_reset_ops;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.nr_resets = priv->variant->nr_resets;

	return devm_reset_controller_register(dev, &priv->rcdev);
}

static const struct of_device_id apple_cio_reset_match[] = {
	{
		.compatible = "apple,t8142-cio-reset",
		.data = &apple_t8142_cio_reset,
	},
	{
		.compatible = "apple,t8103-cio-reset",
		.data = &apple_t8103_cio_reset,
	},
	{
		.compatible = "apple,t6000-cio-reset",
		.data = &apple_t6000_cio_reset,
	},
	{
		.compatible = "apple,t6030-cio-reset",
		.data = &apple_t6000_cio_reset,
	},
	{
		.compatible = "apple,t8152-cio-reset",
		.data = &apple_t8152_cio_reset,
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_cio_reset_match);

static struct platform_driver apple_cio_reset_driver = {
	.driver = {
		.name = "apple-cio-reset",
		.of_match_table = apple_cio_reset_match,
	},
	.probe = apple_cio_reset_probe,
};
module_platform_driver(apple_cio_reset_driver);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("Apple SoC CIO block reset driver");
MODULE_LICENSE("Dual MIT/GPL");
