// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Silicon Display Crossbar multiplexer driver
 *
 * Copyright (C) Asahi Linux Contributors
 *
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/dp-tunnel.h>

/*
 * T602x register interface is cleary different so most of the names below are
 * probably wrong.
 */

#define T602X_FIFO_WR_DPTX_CLK_EN 0x000
#define T602X_FIFO_WR_N_CLK_EN 0x004
#define T602X_FIFO_WR_UNK_EN 0x008
#define T602X_REG_00C 0x00c
#define T602X_REG_010 0x010
#define T602X_REG_014 0x014
#define T602X_REG_018 0x018
#define T602X_REG_01C 0x01c
#define T602X_FIFO_RD_PCLK2_EN 0x024
#define T602X_FIFO_RD_N_CLK_EN 0x028
#define T602X_FIFO_RD_UNK_EN 0x02c
#define T602X_REG_030 0x030
#define T602X_REG_034 0x034

#define T602X_REG_804_STAT 0x804 // status of 0x004
#define T602X_REG_810_STAT 0x810 // status of 0x014
#define T602X_REG_81C_STAT 0x81c // status of 0x024

/*
 * T8013, T600x, T8112 dp crossbar registers.
 */

#define FIFO_WR_DPTX_CLK_EN 0x000
#define FIFO_WR_N_CLK_EN 0x004
#define FIFO_WR_UNK_EN 0x008
#define FIFO_RD_PCLK1_EN 0x020
#define FIFO_RD_PCLK2_EN 0x024
#define FIFO_RD_N_CLK_EN 0x028
#define FIFO_RD_UNK_EN 0x02c

#define OUT_PCLK1_EN 0x040
#define OUT_PCLK2_EN 0x044
#define OUT_N_CLK_EN 0x048
#define OUT_UNK_EN 0x04c

#define CROSSBAR_DISPEXT_EN 0x050
#define CROSSBAR_MUX_CTRL 0x060
#define CROSSBAR_MUX_CTRL_DPPHY_SELECT0 GENMASK(23, 20)
#define CROSSBAR_MUX_CTRL_DPIN1_SELECT0 GENMASK(19, 16)
#define CROSSBAR_MUX_CTRL_DPIN0_SELECT0 GENMASK(15, 12)
#define CROSSBAR_MUX_CTRL_DPPHY_SELECT1 GENMASK(11, 8)
#define CROSSBAR_MUX_CTRL_DPIN1_SELECT1 GENMASK(7, 4)
#define CROSSBAR_MUX_CTRL_DPIN0_SELECT1 GENMASK(3, 0)
#define CROSSBAR_ATC_EN 0x070

#define FIFO_WR_DPTX_CLK_EN_STAT 0x800
#define FIFO_WR_N_CLK_EN_STAT 0x804
#define FIFO_RD_PCLK1_EN_STAT 0x820
#define FIFO_RD_PCLK2_EN_STAT 0x824
#define FIFO_RD_N_CLK_EN_STAT 0x828

#define OUT_PCLK1_EN_STAT 0x840
#define OUT_PCLK2_EN_STAT 0x844
#define OUT_N_CLK_EN_STAT 0x848

#define UNK_TUNABLE 0xc00

#define ATC_DPIN0 BIT(0)
#define ATC_DPIN1 BIT(4)
#define ATC_DPPHY BIT(8)

enum { MUX_DPPHY = 0, MUX_DPIN0 = 1, MUX_DPIN1 = 2, MUX_MAX = 3 };
static const char *apple_dpxbar_names[MUX_MAX] = { "dpphy", "dpin0", "dpin1" };

struct apple_dpxbar_hw {
	unsigned int n_ufp;
	u32 tunable;
	const struct mux_control_ops *ops;
	/* DP IN runs without the FIFO_RD_PCLK1 cycle-slip toggle (t600x) */
	bool dpin_no_cycle_slip;
};

struct apple_dpxbar {
	struct device *dev;
	const struct apple_dpxbar_hw *hw;
	void __iomem *regs;
	int selected_dispext[MUX_MAX];
	/* T6030 programs all its outputs itself, see apple_dpxbar_t6030_link() */
	bool t6030_dpin;
	bool tunnel_link_up[MUX_MAX];
	/* T6030: the crossbar's power is held from the first select until idle. */
	bool pm_held;
	spinlock_t lock;
};

static inline void dpxbar_mask32(struct apple_dpxbar *xbar, u32 reg, u32 mask,
				 u32 set)
{
	u32 value = readl(xbar->regs + reg);
	value &= ~mask;
	value |= set;
	writel(value, xbar->regs + reg);
}

static inline void dpxbar_set32(struct apple_dpxbar *xbar, u32 reg, u32 set)
{
	dpxbar_mask32(xbar, reg, 0, set);
}

static inline void dpxbar_clear32(struct apple_dpxbar *xbar, u32 reg, u32 clear)
{
	dpxbar_mask32(xbar, reg, clear, 0);
}

static u32 t602x_atc_bit(unsigned int index)
{
	switch (index) {
	case MUX_DPIN0:
		return ATC_DPIN0;
	case MUX_DPIN1:
		return ATC_DPIN1;
	case MUX_DPPHY:
	default:
		return ATC_DPPHY;
	}
}

static u32 t602x_mux_mask(unsigned int index)
{
	switch (index) {
	case MUX_DPIN0:
		return CROSSBAR_MUX_CTRL_DPIN0_SELECT0 |
		       CROSSBAR_MUX_CTRL_DPIN0_SELECT1;
	case MUX_DPIN1:
		return CROSSBAR_MUX_CTRL_DPIN1_SELECT0 |
		       CROSSBAR_MUX_CTRL_DPIN1_SELECT1;
	case MUX_DPPHY:
	default:
		return CROSSBAR_MUX_CTRL_DPPHY_SELECT0 |
		       CROSSBAR_MUX_CTRL_DPPHY_SELECT1;
	}
}

static u32 t602x_mux_set(unsigned int index, unsigned int mux_state)
{
	switch (index) {
	case MUX_DPIN0:
		return FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT0, mux_state) |
		       FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT1, mux_state);
	case MUX_DPIN1:
		return FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT0, mux_state) |
		       FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT1, mux_state);
	case MUX_DPPHY:
	default:
		return FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT0, mux_state) |
		       FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT1, mux_state);
	}
}

static void t602x_dump(struct apple_dpxbar *xbar, const char *tag)
{
	static const u32 offs[] = {
		0x000, 0x004, 0x008, 0x00c, 0x014, 0x018, 0x01c, 0x024,
		0x028, 0x02c, 0x030, 0x034, 0x040, 0x044, 0x048, 0x04c,
		0x050, 0x060, 0x070,
	};
	char buf[320];
	int n = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(offs); i++) {
		u32 val = readl(xbar->regs + offs[i]);

		n += scnprintf(buf + n, sizeof(buf) - n, " %03x=%08x",
			       offs[i], val);
		if (n >= (int)sizeof(buf) - 20)
			break;
	}
	dev_dbg(xbar->dev, "t602x %s:%s\n", tag, buf);
	dev_dbg(xbar->dev,
		"t602x %s clk: 000=%08x 800=%08x 020=%08x 820=%08x 024=%08x 81c=%08x\n",
		tag,
		readl(xbar->regs + T602X_FIFO_WR_DPTX_CLK_EN),
		readl(xbar->regs + FIFO_WR_DPTX_CLK_EN_STAT),
		readl(xbar->regs + FIFO_RD_PCLK1_EN),
		readl(xbar->regs + FIFO_RD_PCLK1_EN_STAT),
		readl(xbar->regs + T602X_FIFO_RD_PCLK2_EN),
		readl(xbar->regs + T602X_REG_81C_STAT));
}

/*
 * T6030 DP IN outputs. T6030 has the T6020 crossbar, but it brings a DP IN
 * output up and down with its own sequence: the mux selection only routes, and
 * DCP's DidChangeLinkConfiguration enables the clocks. The M2 Pro and M2 Max
 * keep apple_dpxbar_t602x_link_up() and apple_dpxbar_t602x_link_down().
 */
#define T602X_RD_PCLK1 0x00c
#define T602X_RD_PCLK2 0x010
#define T602X_RD_GATE 0x014
#define T602X_RD_SELECT 0x018
#define T602X_OUT_PCLK1 0x01c
#define T602X_OUT_PCLK2 0x020
#define T602X_OUT_GATE 0x024
#define T602X_OUT_SELECT 0x028
#define T602X_DISPEXT_ENABLE 0x02c
#define T602X_MUX_SELECT 0x030
#define T602X_ATC_ENABLE 0x034

/*
 * Field of a T6030 output in the gate, select and enable registers. DP IN 0,
 * DP IN 1 and the DP PHY output use bits 0, 4 and 8.
 */
static unsigned int apple_dpxbar_t6030_shift(unsigned int index)
{
	switch (index) {
	case MUX_DPIN0:
		return 0;
	case MUX_DPIN1:
		return 4;
	case MUX_DPPHY:
	default:
		return 8;
	}
}

/*
 * The register image of a T6030 crossbar with no connection up: the gates
 * at +0x004/+0x014/+0x024 closed and their status bits set, everything else
 * clear. The selectors at +0x030 are not part of it (a route stays selected
 * while its connection is down), and +0x034 may keep an output's bit from an
 * earlier connection, as the hardware leaves it.
 */
static const struct {
	u16 offset;
	u16 value;
} apple_dpxbar_t6030_idle[] = {
	{ 0x000, 0x000 }, { 0x004, 0x1ff }, { 0x008, 0x000 }, { 0x00c, 0x000 },
	{ 0x010, 0x000 }, { 0x014, 0x1ff }, { 0x018, 0x000 }, { 0x01c, 0x000 },
	{ 0x020, 0x000 }, { 0x024, 0x111 }, { 0x028, 0x000 }, { 0x02c, 0x000 },
	{ 0x030, 0x000 }, { 0x034, 0x000 }, { 0x800, 0x000 }, { 0x804, 0x1ff },
	{ 0x808, 0x000 }, { 0x80c, 0x000 }, { 0x810, 0x1ff }, { 0x814, 0x000 },
	{ 0x818, 0x000 }, { 0x81c, 0x111 },
};

static void apple_dpxbar_t6030_dump(struct apple_dpxbar *xbar, unsigned int index,
				    const char *what)
{
	char buf[256];
	unsigned int i;
	int n = 0;

	for (i = 0; i < ARRAY_SIZE(apple_dpxbar_t6030_idle); i++)
		n += scnprintf(buf + n, sizeof(buf) - n, " %03x=%x",
			       apple_dpxbar_t6030_idle[i].offset,
			       readl(xbar->regs + apple_dpxbar_t6030_idle[i].offset));
	dev_warn(xbar->dev, "%s: %s:%s\n", apple_dpxbar_names[index], what, buf);
}

/* No output's connection is up, so the whole crossbar must show the idle image. */
static bool apple_dpxbar_t6030_is_idle(struct apple_dpxbar *xbar)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(apple_dpxbar_t6030_idle); i++) {
		u16 offset = apple_dpxbar_t6030_idle[i].offset;
		u32 v = readl(xbar->regs + offset);

		if (offset == T602X_MUX_SELECT)
			continue;
		if (offset == T602X_ATC_ENABLE)
			v &= ~0x111;
		if (v != apple_dpxbar_t6030_idle[i].value)
			return false;
	}
	return true;
}

/*
 * Checks the image of one output after its connection went up or down:
 * its source bit @u in the write-side registers, its bit @d in the output
 * registers and its two-bit fields in the select registers. The other
 * outputs' bits are left alone, they may be connected.
 */
static bool apple_dpxbar_t6030_output_is(struct apple_dpxbar *xbar, unsigned int index,
					 bool up)
{
	u32 state = xbar->selected_dispext[index];
	u32 u = BIT(state), shift = apple_dpxbar_t6030_shift(index);
	u32 d = BIT(shift);
	bool pclk2 = (index == MUX_DPIN1) ^ (state & 1);
	u32 ps = pclk2 ? 3 : 1;
	u32 rd = pclk2 ? T602X_RD_PCLK2 : T602X_RD_PCLK1;
	u32 out = pclk2 ? T602X_OUT_PCLK2 : T602X_OUT_PCLK1;
	u32 sel = (state << shift) | (state << (shift + 12));
	u32 sel_mask = (0xfU << shift) | (0xfU << (shift + 12));
	bool ok;

#define OUT_BIT(reg, bit, set) ((readl(xbar->regs + (reg)) & (bit)) == ((set) ? (bit) : 0))
	/* The clocks and enables of this output, and the gates' status. */
	ok = OUT_BIT(T602X_FIFO_WR_DPTX_CLK_EN, u, up) && OUT_BIT(T602X_FIFO_WR_N_CLK_EN, u, !up) &&
	     OUT_BIT(T602X_FIFO_WR_UNK_EN, u, up) && OUT_BIT(rd, u, up) &&
	     OUT_BIT(T602X_RD_GATE, u, !up) && OUT_BIT(out, d, up) &&
	     OUT_BIT(T602X_OUT_GATE, d, !up) && OUT_BIT(T602X_DISPEXT_ENABLE, u, up) &&
	     OUT_BIT(0x804, u, !up) && OUT_BIT(0x810, u, !up) && OUT_BIT(0x81c, d, !up) &&
	     ((readl(xbar->regs + T602X_RD_SELECT) >> (2 * state)) & 3) == (up ? ps : 0) &&
	     ((readl(xbar->regs + T602X_OUT_SELECT) >> shift) & 3) == (up ? ps : 0) &&
	     (readl(xbar->regs + T602X_MUX_SELECT) & sel_mask) == sel;
	/*
	 * The clock status bits at +0x800/+0x808/+0x80c/+0x814/+0x818 are
	 * part of the idle image only: the connection-up image leaves them
	 * out. The ATC enable is set with the connection and left set after it.
	 */
	if (up)
		ok = ok && OUT_BIT(T602X_ATC_ENABLE, d, true);
	else
		ok = ok && OUT_BIT(0x800, u, false) && OUT_BIT(pclk2 ? 0x80c : 0x808, u, false) &&
		     OUT_BIT(pclk2 ? 0x818 : 0x814, d, false);
#undef OUT_BIT
	return ok;
}

/*
 * Verifies the register image after a connection transition, with the lock
 * held: this output's own image, and, when no connection is left up, the
 * crossbar's idle image. A mismatch is reported with the complete image;
 * the transition is not undone, as the hardware may have taken part of it.
 */
static bool apple_dpxbar_t6030_verify(struct apple_dpxbar *xbar, unsigned int index, bool up)
{
	unsigned int i;
	bool ok;

	lockdep_assert_held(&xbar->lock);
	ok = apple_dpxbar_t6030_output_is(xbar, index, up);
	if (!ok)
		apple_dpxbar_t6030_dump(xbar, index, up ? "connection not up" :
						       "connection not down");
	if (up)
		return ok;
	for (i = 0; i < MUX_MAX; i++)
		if (xbar->tunnel_link_up[i])
			return ok;
	if (!apple_dpxbar_t6030_is_idle(xbar)) {
		apple_dpxbar_t6030_dump(xbar, index, "crossbar not idle after connection down");
		return false;
	}
	return ok;
}

/*
 * Called with the crossbar lock held and a selected output. Returns whether
 * the register image is the expected one afterwards.
 */
static bool apple_dpxbar_t6030_link(struct apple_dpxbar *xbar,
				    unsigned int index, bool up)
{
	u32 state = xbar->selected_dispext[index];
	u32 u = BIT(state), shift = apple_dpxbar_t6030_shift(index);
	u32 d = BIT(shift);
	bool pclk2 = (index == MUX_DPIN1) ^ (state & 1);
	u32 ps = pclk2 ? 3 : 1;
	u32 rd = pclk2 ? T602X_RD_PCLK2 : T602X_RD_PCLK1;
	u32 out = pclk2 ? T602X_OUT_PCLK2 : T602X_OUT_PCLK1;
	unsigned int i;

	lockdep_assert_held(&xbar->lock);
	if (xbar->tunnel_link_up[index] == up)
		return true;
	if (up) {
		/* A connection starts from the idle image when it is the first. */
		for (i = 0; i < MUX_MAX; i++)
			if (xbar->tunnel_link_up[i])
				break;
		if (i == MUX_MAX && !apple_dpxbar_t6030_is_idle(xbar))
			apple_dpxbar_t6030_dump(xbar, index, "crossbar not idle before connection up");
		dpxbar_clear32(xbar, T602X_FIFO_WR_N_CLK_EN, u);
		dpxbar_clear32(xbar, T602X_RD_GATE, u);
		dpxbar_clear32(xbar, T602X_OUT_GATE, d);
		udelay(1);
		if ((readl(xbar->regs + 0x804) & u) ||
		    (readl(xbar->regs + 0x810) & u) ||
		    (readl(xbar->regs + 0x81c) & d))
			dev_warn(xbar->dev, "%s: T602X clock gate status remains set\n",
				 apple_dpxbar_names[index]);
		dpxbar_set32(xbar, T602X_FIFO_WR_UNK_EN, u);
		dpxbar_mask32(xbar, T602X_RD_SELECT, 3U << (2 * state), ps << (2 * state));
		dpxbar_mask32(xbar, T602X_OUT_SELECT, 3U << shift, ps << shift);
		dpxbar_set32(xbar, T602X_FIFO_WR_DPTX_CLK_EN, u);
		dpxbar_set32(xbar, rd, u);
		dpxbar_set32(xbar, out, d);
		dpxbar_set32(xbar, T602X_ATC_ENABLE, d);
		dpxbar_set32(xbar, T602X_DISPEXT_ENABLE, u);
	} else {
		dpxbar_clear32(xbar, T602X_DISPEXT_ENABLE, u);
		dpxbar_clear32(xbar, T602X_FIFO_WR_DPTX_CLK_EN, u);
		dpxbar_clear32(xbar, rd, u);
		dpxbar_clear32(xbar, out, d);
		udelay(1);
		if ((readl(xbar->regs + 0x800) & u) ||
		    (readl(xbar->regs + (pclk2 ? 0x80c : 0x808)) & u) ||
		    (readl(xbar->regs + (pclk2 ? 0x818 : 0x814)) & d))
			dev_warn(xbar->dev, "%s: T602X clocks remain active\n",
				 apple_dpxbar_names[index]);
		dpxbar_clear32(xbar, T602X_FIFO_WR_UNK_EN, u);
		dpxbar_mask32(xbar, T602X_RD_SELECT, 3U << (2 * state), 0);
		dpxbar_mask32(xbar, T602X_OUT_SELECT, 3U << shift, 0);
		dpxbar_set32(xbar, T602X_FIFO_WR_N_CLK_EN, u);
		dpxbar_set32(xbar, T602X_RD_GATE, u);
		dpxbar_set32(xbar, T602X_OUT_GATE, d);
	}
	/* No T8103 cycle-slip workaround: T602X does not require one. */
	xbar->tunnel_link_up[index] = up;
	return apple_dpxbar_t6030_verify(xbar, index, up);
}

/*
 * The crossbar's power (its ATC's USB domain, power-domains) is held from
 * the first select until the last output has been deselected and the
 * crossbar verified idle. Called without the lock: resuming may sleep.
 */
static int apple_dpxbar_t6030_pm_get(struct apple_dpxbar *xbar)
{
	int ret;

	if (xbar->pm_held)
		return 0;
	ret = pm_runtime_resume_and_get(xbar->dev);
	if (ret) {
		dev_err(xbar->dev, "cannot power the crossbar: %d\n", ret);
		return ret;
	}
	xbar->pm_held = true;
	return 0;
}

/* @release: every output deselected and the idle image verified. */
static void apple_dpxbar_t6030_pm_put(struct apple_dpxbar *xbar, bool release)
{
	/* A crossbar that did not reach the idle image keeps its power. */
	if (!xbar->pm_held || !release)
		return;
	xbar->pm_held = false;
	pm_runtime_put(xbar->dev);
}

/* Standard mux ownership, but Activate only routes; DidChange enables clocks. */
static int apple_dpxbar_t6030_dpin_set(struct mux_control *mux, int state)
{
	struct apple_dpxbar *xbar = mux_chip_priv(mux->chip);
	unsigned int index = mux_control_get_index(mux);
	unsigned long flags;
	u32 shift = apple_dpxbar_t6030_shift(index);
	u32 mask = (0xfU << shift) | (0xfU << (shift + 12));
	unsigned int i;
	bool idle = true;
	int ret;

	if (state != MUX_IDLE_DISCONNECT && (state < 0 || state >= 9))
		return -EINVAL;
	if (state >= 0) {
		ret = apple_dpxbar_t6030_pm_get(xbar);
		if (ret)
			return ret;
	} else if (!xbar->pm_held) {
		return 0;	/* never selected: nothing was written */
	}
	spin_lock_irqsave(&xbar->lock, flags);
	if (state >= 0) {
		for (i = 0; i < MUX_MAX; i++) {
			if (i != index && xbar->selected_dispext[i] == state) {
				spin_unlock_irqrestore(&xbar->lock, flags);
				return -EBUSY;
			}
		}
	}
	/* A disconnect must leave the idle image behind to release the power. */
	if (xbar->selected_dispext[index] >= 0)
		idle = apple_dpxbar_t6030_link(xbar, index, false);
	if (state >= 0)
		dpxbar_mask32(xbar, T602X_MUX_SELECT, mask,
			      (state << shift) | (state << (shift + 12)));
	/* Leave MUX and ATC enable intact on disconnect, as the hardware does. */
	xbar->selected_dispext[index] = state;
	for (i = 0; i < MUX_MAX && idle; i++)
		if (xbar->selected_dispext[i] >= 0 || xbar->tunnel_link_up[i])
			idle = false;
	if (idle && !apple_dpxbar_t6030_is_idle(xbar)) {
		apple_dpxbar_t6030_dump(xbar, index, "crossbar not idle after disconnect");
		idle = false;
	}
	spin_unlock_irqrestore(&xbar->lock, flags);
	if (state < 0)
		apple_dpxbar_t6030_pm_put(xbar, idle);
	dev_info(xbar->dev, "%s: T602X route %d, clocks disabled\n",
		 apple_dpxbar_names[index], state);
	return 0;
}

static int apple_dpxbar_set_t602x(struct mux_control *mux, int state)
{
	struct apple_dpxbar *dpxbar = mux_chip_priv(mux->chip);
	unsigned int index = mux_control_get_index(mux);
	unsigned long flags;
	unsigned int mux_state;
	unsigned int dispext_bit;
	unsigned int dispext_bit_en;
	u32 atc_bit, mux_mask, mux_val;
	bool enable;
	/* J700 physical DP retains the original T602x clock-gate sequence. */
	bool j700_physical = index == MUX_DPPHY && of_machine_is_compatible("apple,j700");
	int ret = 0;

	if (index >= MUX_MAX)
		return -EINVAL;
	/* T6030 routes every output on select and clocks it on link up. */
	if (dpxbar->t6030_dpin)
		return apple_dpxbar_t6030_dpin_set(mux, state);

	if (state == MUX_IDLE_DISCONNECT) {
		mux_state = 0;
		enable = false;
	} else if (state >= 0 && state < 9) {
		dispext_bit = 1 << state;
		dispext_bit_en = 1 << (2 * state);
		mux_state = state;
		enable = true;
	} else {
		return -EINVAL;
	}

	atc_bit = t602x_atc_bit(index);
	mux_mask = t602x_mux_mask(index);
	mux_val = t602x_mux_set(index, mux_state);

	spin_lock_irqsave(&dpxbar->lock, flags);

	/* ensure the selected dispext isn't already used in this crossbar */
	if (enable) {
		for (int i = 0; i < MUX_MAX; ++i) {
			if (i == index)
				continue;
			if (dpxbar->selected_dispext[i] == state) {
				spin_unlock_irqrestore(&dpxbar->lock, flags);
				return -EBUSY;
			}
		}
	}

	if (dpxbar->selected_dispext[index] >= 0) {
		u32 prev_dispext_bit = 1 << dpxbar->selected_dispext[index];
		u32 prev_dispext_bit_en = 1 << (2 * dpxbar->selected_dispext[index]);

		dpxbar_clear32(dpxbar, T602X_FIFO_RD_UNK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, T602X_FIFO_WR_DPTX_CLK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar,
			      index == MUX_DPIN1 ? T602X_REG_010 : T602X_REG_00C,
			      prev_dispext_bit);

		dpxbar_clear32(dpxbar,
			      index == MUX_DPIN1 ? FIFO_RD_PCLK1_EN : T602X_REG_01C,
			      atc_bit);

		dpxbar_clear32(dpxbar, T602X_FIFO_WR_UNK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, T602X_REG_018,
			      prev_dispext_bit_en *
			      (index == MUX_DPIN1 ? 3 : 1));

		dpxbar_clear32(dpxbar, T602X_FIFO_RD_N_CLK_EN,
			      atc_bit * (index == MUX_DPIN1 ? 3 : 1));

		dpxbar_set32(dpxbar, T602X_FIFO_WR_N_CLK_EN, prev_dispext_bit);
		dpxbar_set32(dpxbar, T602X_REG_014, 0x4);

		/* Native T602x DPIN0 teardown restores the read reset at +0x24. */
		dpxbar_set32(dpxbar, j700_physical ? FIFO_RD_PCLK1_EN :
			     T602X_FIFO_RD_PCLK2_EN, atc_bit);

		dpxbar_clear32(dpxbar, T602X_REG_034, atc_bit);
		if (!j700_physical) {
			dpxbar_clear32(dpxbar, CROSSBAR_ATC_EN, atc_bit);
			dpxbar_clear32(dpxbar, CROSSBAR_DISPEXT_EN, prev_dispext_bit);
		}
		dpxbar_mask32(dpxbar, T602X_REG_030, mux_mask, 0);

		dpxbar->selected_dispext[index] = -1;
	}

	if (enable) {
		dpxbar_mask32(dpxbar, T602X_REG_030, mux_mask, mux_val);
		udelay(10);

		dpxbar_clear32(dpxbar, T602X_FIFO_WR_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, T602X_REG_014, 0x4);

		/* Match the DPIN1 sequence used by link_up(): clear +0x24
		 * before setting the separate +0x20 field below.
		 */
		dpxbar_clear32(dpxbar, T602X_FIFO_RD_PCLK2_EN, atc_bit);
		if (index == MUX_DPIN1)
			dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, atc_bit);

		dpxbar_set32(dpxbar, T602X_FIFO_WR_UNK_EN, dispext_bit);
		/* Native T602x encodes DPIN1's role as 3 in both two-bit
		 * source and sink fields, versus 1 for DPIN0.
		 */
		if (index == MUX_DPIN1) {
			dpxbar_mask32(dpxbar, T602X_REG_018,
				       dispext_bit_en * 3,
				       dispext_bit_en * 3);
			dpxbar_mask32(dpxbar, T602X_FIFO_RD_N_CLK_EN,
				       atc_bit * 3, atc_bit * 3);
		} else {
			dpxbar_set32(dpxbar, T602X_REG_018,
				     dispext_bit_en);
			dpxbar_set32(dpxbar, T602X_FIFO_RD_N_CLK_EN,
				     atc_bit);
		}
		dpxbar_set32(dpxbar, T602X_FIFO_WR_DPTX_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar,
			     index == MUX_DPIN1 ? T602X_REG_010 : T602X_REG_00C,
			     dispext_bit);

		dpxbar_set32(dpxbar,
			     index == MUX_DPIN1 ? FIFO_RD_PCLK1_EN : T602X_REG_01C,
			     atc_bit);
		dpxbar_set32(dpxbar, T602X_REG_034, atc_bit);
		if (!j700_physical)
			dpxbar_set32(dpxbar, CROSSBAR_ATC_EN, atc_bit);
		/*
		 * t8103 enables the dispext source at 0x050. T602x left
		 * that register at 0, so dpin0 was selected with no
		 * source clock into the ACIO analog PHY.
		 */
		if (!j700_physical)
			dpxbar_set32(dpxbar, CROSSBAR_DISPEXT_EN, dispext_bit);

		dpxbar_set32(dpxbar, T602X_FIFO_RD_UNK_EN, dispext_bit);

		dpxbar->selected_dispext[index] = state;
	}

	spin_unlock_irqrestore(&dpxbar->lock, flags);

	if (enable) {
		dev_info(dpxbar->dev, "Switched %s to dispext%u,%u\n",
			 apple_dpxbar_names[index], mux_state >> 1,
			 mux_state & 1);
		dev_dbg(dpxbar->dev, "t602x atc=0x%x mux=0x%x\n", atc_bit,
			mux_val);
	} else {
		dev_info(dpxbar->dev, "Switched %s to disconnected state\n",
			 apple_dpxbar_names[index]);
	}

	t602x_dump(dpxbar, enable ? apple_dpxbar_names[index] : "idle");

	return ret;
}

/* Warn if the clock gates of an output did not release. Diagnostic only. */
static void apple_dpxbar_check_gates(struct apple_dpxbar *dpxbar, unsigned int index,
				     u32 dispext_bit, u32 atc_bit)
{
	u32 wr = readl(dpxbar->regs + FIFO_WR_N_CLK_EN_STAT);
	u32 rd = readl(dpxbar->regs + FIFO_RD_N_CLK_EN_STAT);
	u32 out = readl(dpxbar->regs + OUT_N_CLK_EN_STAT);

	if ((wr & dispext_bit) || (rd & dispext_bit) || (out & atc_bit))
		dev_warn(dpxbar->dev, "%s: clock gates still set (%08x %08x %08x)\n",
			 apple_dpxbar_names[index], wr, rd, out);
}

/* The RD_PCLK toggle below is needed for the PHY output, and for DP IN on some SoCs. */
static bool apple_dpxbar_cycle_slip(struct apple_dpxbar *dpxbar, unsigned int index)
{
	return index == MUX_DPPHY || !dpxbar->hw->dpin_no_cycle_slip;
}

static int apple_dpxbar_set(struct mux_control *mux, int state)
{
	struct apple_dpxbar *dpxbar = mux_chip_priv(mux->chip);
	unsigned int index = mux_control_get_index(mux);
	unsigned long flags;
	unsigned int mux_state;
	unsigned int dispext_bit;
	unsigned int dispext_bit_en;
	unsigned int atc_bit;
	bool enable;
	int ret = 0;
	u32 mux_mask, mux_set;

	if (state == MUX_IDLE_DISCONNECT) {
		/*
		 * Technically this will select dispext0,0 in the mux control
		 * register. Practically that doesn't matter since everything
		 * else is disabled.
		 */
		mux_state = 0;
		enable = false;
	} else if (state >= 0 && state < 9) {
		dispext_bit = 1 << state;
		dispext_bit_en = 1 << (2 * state);
		mux_state = state;
		enable = true;
	} else {
		return -EINVAL;
	}

	switch (index) {
	case MUX_DPPHY:
		mux_mask = CROSSBAR_MUX_CTRL_DPPHY_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPPHY_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPPHY_SELECT1, mux_state);
		atc_bit = ATC_DPPHY;
		break;
	case MUX_DPIN0:
		mux_mask = CROSSBAR_MUX_CTRL_DPIN0_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPIN0_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT1, mux_state);
		atc_bit = ATC_DPIN0;
		break;
	case MUX_DPIN1:
		mux_mask = CROSSBAR_MUX_CTRL_DPIN1_SELECT0 |
			   CROSSBAR_MUX_CTRL_DPIN1_SELECT1;
		mux_set =
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT0, mux_state) |
			FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT1, mux_state);
		atc_bit = ATC_DPIN1;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&dpxbar->lock, flags);

	/* ensure the selected dispext isn't already used in this crossbar */
	if (enable) {
		for (int i = 0; i < MUX_MAX; ++i) {
			if (i == index)
				continue;
			if (dpxbar->selected_dispext[i] == state) {
				spin_unlock_irqrestore(&dpxbar->lock, flags);
				return -EBUSY;
			}
		}
	}

	dpxbar_set32(dpxbar, OUT_N_CLK_EN, atc_bit);
	dpxbar_clear32(dpxbar, OUT_UNK_EN, atc_bit);
	dpxbar_clear32(dpxbar, OUT_PCLK1_EN, atc_bit);
	dpxbar_clear32(dpxbar, CROSSBAR_ATC_EN, atc_bit);

	if (dpxbar->selected_dispext[index] >= 0) {
		u32 prev_dispext_bit = 1 << dpxbar->selected_dispext[index];
		u32 prev_dispext_bit_en = 1 << (2 * dpxbar->selected_dispext[index]);

		dpxbar_set32(dpxbar, FIFO_WR_N_CLK_EN, prev_dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_N_CLK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_WR_UNK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_UNK_EN, prev_dispext_bit_en);
		dpxbar_clear32(dpxbar, FIFO_WR_DPTX_CLK_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, prev_dispext_bit);
		dpxbar_clear32(dpxbar, CROSSBAR_DISPEXT_EN, prev_dispext_bit);

		dpxbar->selected_dispext[index] = -1;
	}

	dpxbar_mask32(dpxbar, CROSSBAR_MUX_CTRL, mux_mask, mux_set);

	if (enable) {
		dpxbar_clear32(dpxbar, FIFO_WR_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, OUT_N_CLK_EN, atc_bit);
		/* let the gates release before enabling the clocks behind them */
		udelay(1);
		apple_dpxbar_check_gates(dpxbar, index, dispext_bit, atc_bit);
		dpxbar_set32(dpxbar, FIFO_WR_UNK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_UNK_EN, dispext_bit_en);
		dpxbar_set32(dpxbar, OUT_UNK_EN, atc_bit);
		dpxbar_set32(dpxbar, FIFO_WR_DPTX_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		dpxbar_set32(dpxbar, OUT_PCLK1_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_ATC_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_DISPEXT_EN, dispext_bit);

		/*
		 * Work around some HW quirk:
		 * Without toggling the RD_PCLK enable here the connection
		 * doesn't come up. Testing has shown that a delay of about
		 * 5 usec is required which is doubled here to be on the
		 * safe side.
		 */
		if (apple_dpxbar_cycle_slip(dpxbar, index)) {
			dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
			udelay(10);
			dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		}

		dpxbar->selected_dispext[index] = state;
	}

	spin_unlock_irqrestore(&dpxbar->lock, flags);

	if (enable)
		dev_info(dpxbar->dev, "Switched %s to dispext%u,%u\n",
			 apple_dpxbar_names[index], mux_state >> 1,
			 mux_state & 1);
	else
		dev_info(dpxbar->dev, "Switched %s to disconnected state\n",
			 apple_dpxbar_names[index]);

	return ret;
}

static const struct mux_control_ops apple_dpxbar_ops;

static const u32 apple_dpxbar_atc_bits[MUX_MAX] = { ATC_DPPHY, ATC_DPIN0, ATC_DPIN1 };

/*
 * The output's current selection; dpxbar->lock held. The caller keeps the
 * output selected (holds the mux) around the link helpers.
 */
static int apple_dpxbar_link_bits(struct apple_dpxbar *dpxbar, unsigned int index,
				  u32 *dispext_bit, u32 *atc_bit)
{
	int state;

	lockdep_assert_held(&dpxbar->lock);
	state = dpxbar->selected_dispext[index];
	if (state < 0)
		return -ENODEV;
	*dispext_bit = 1 << state;
	*atc_bit = apple_dpxbar_atc_bits[index];
	return 0;
}

static struct apple_dpxbar *apple_dpxbar_from_mux(struct mux_control *mux,
						  unsigned int *index)
{
	if (mux->chip->ops != &apple_dpxbar_ops)
		return NULL;
	*index = mux_control_get_index(mux);
	if (*index >= MUX_MAX)
		return NULL;
	return mux_chip_priv(mux->chip);
}

static int apple_dpxbar_t8103_link_down(struct mux_control *mux)
{
	struct apple_dpxbar *dpxbar;
	u32 dispext_bit, atc_bit;
	unsigned long flags;
	unsigned int index;
	int ret;

	dpxbar = apple_dpxbar_from_mux(mux, &index);
	if (!dpxbar)
		return -EINVAL;

	spin_lock_irqsave(&dpxbar->lock, flags);
	ret = apple_dpxbar_link_bits(dpxbar, index, &dispext_bit, &atc_bit);
	if (!ret) {
		dpxbar_clear32(dpxbar, CROSSBAR_DISPEXT_EN, dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_WR_DPTX_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		dpxbar_clear32(dpxbar, OUT_PCLK1_EN, atc_bit);
		udelay(1);
		dpxbar_set32(dpxbar, FIFO_WR_N_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_N_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar, OUT_N_CLK_EN, atc_bit);
	}
	spin_unlock_irqrestore(&dpxbar->lock, flags);
	return ret;
}

static int apple_dpxbar_t8103_link_up(struct mux_control *mux)
{
	struct apple_dpxbar *dpxbar;
	u32 dispext_bit, atc_bit;
	unsigned long flags;
	unsigned int index;
	int ret;

	dpxbar = apple_dpxbar_from_mux(mux, &index);
	if (!dpxbar)
		return -EINVAL;

	spin_lock_irqsave(&dpxbar->lock, flags);
	ret = apple_dpxbar_link_bits(dpxbar, index, &dispext_bit, &atc_bit);
	if (!ret) {
		dpxbar_clear32(dpxbar, FIFO_WR_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, FIFO_RD_N_CLK_EN, dispext_bit);
		dpxbar_clear32(dpxbar, OUT_N_CLK_EN, atc_bit);
		udelay(1);
		apple_dpxbar_check_gates(dpxbar, index, dispext_bit, atc_bit);
		dpxbar_set32(dpxbar, FIFO_WR_DPTX_CLK_EN, dispext_bit);
		dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		dpxbar_set32(dpxbar, OUT_PCLK1_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_ATC_EN, atc_bit);
		dpxbar_set32(dpxbar, CROSSBAR_DISPEXT_EN, dispext_bit);
		if (apple_dpxbar_cycle_slip(dpxbar, index)) {
			dpxbar_clear32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
			udelay(10);
			dpxbar_set32(dpxbar, FIFO_RD_PCLK1_EN, dispext_bit);
		}
	}
	spin_unlock_irqrestore(&dpxbar->lock, flags);
	if (ret)
		return ret;

	dev_dbg(dpxbar->dev, "link up: ATC_EN=%08x STAT WR=%08x RD=%08x OUT=%08x\n",
		readl(dpxbar->regs + CROSSBAR_ATC_EN),
		readl(dpxbar->regs + FIFO_WR_DPTX_CLK_EN_STAT),
		readl(dpxbar->regs + FIFO_RD_PCLK1_EN_STAT),
		readl(dpxbar->regs + OUT_PCLK1_EN_STAT));
	return 0;
}

int apple_dpxbar_preselect(struct mux_control *mux, int state)
{
	struct apple_dpxbar *dpxbar;
	unsigned int index, mux_state;
	unsigned long flags;
	u32 mask, set;
	int ret = 0;

	if (!mux)
		return -EINVAL;
	dpxbar = apple_dpxbar_from_mux(mux, &index);
	if (!dpxbar)
		return -EOPNOTSUPP;
	if (state == MUX_IDLE_DISCONNECT)
		mux_state = 0;
	else if (state >= 0 && state < 9)
		mux_state = state;
	else
		return -EINVAL;

	switch (index) {
	case MUX_DPIN0:
		mask = CROSSBAR_MUX_CTRL_DPIN0_SELECT0 | CROSSBAR_MUX_CTRL_DPIN0_SELECT1;
		set = FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT0, mux_state) |
		      FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN0_SELECT1, mux_state);
		break;
	case MUX_DPIN1:
		mask = CROSSBAR_MUX_CTRL_DPIN1_SELECT0 | CROSSBAR_MUX_CTRL_DPIN1_SELECT1;
		set = FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT0, mux_state) |
		      FIELD_PREP(CROSSBAR_MUX_CTRL_DPIN1_SELECT1, mux_state);
		break;
	default:
		return -EOPNOTSUPP;
	}

	spin_lock_irqsave(&dpxbar->lock, flags);
	/* a selected output already points at its source */
	if (dpxbar->selected_dispext[index] >= 0)
		ret = -EBUSY;
	else
		dpxbar_mask32(dpxbar, CROSSBAR_MUX_CTRL, mask, set);
	spin_unlock_irqrestore(&dpxbar->lock, flags);
	if (ret)
		return ret;

	dev_dbg(dpxbar->dev, "%s: source preselected, MUX_CTRL=%08x\n",
		apple_dpxbar_names[index], readl(dpxbar->regs + CROSSBAR_MUX_CTRL));
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dpxbar_preselect);

static const struct mux_control_ops apple_dpxbar_ops = {
	.set = apple_dpxbar_set,
};

static const struct mux_control_ops apple_dpxbar_t602x_ops = {
	.set = apple_dpxbar_set_t602x,
};

/*
 * Select the DCP source for a USB4 DP IN before DCP probes AUX. Do not
 * enable any FIFO clock or ATC output here: full activation still waits for
 * DCP's SetLinkRate and the tunnel pixel clock. dispext0 works with the
 * reset selector value (0); dispext1 needs this explicit preselection.
 */
int apple_dpxbar_tunnel_select_source(struct mux_control *mux, int state)
{
	struct apple_dpxbar *xbar;
	unsigned long flags;
	unsigned int index;
	int ret = 0;

	if (!mux || mux->chip->ops != &apple_dpxbar_t602x_ops)
		return -EOPNOTSUPP;
	xbar = mux_chip_priv(mux->chip);
	if (xbar->t6030_dpin)
		return -EOPNOTSUPP;
	index = mux_control_get_index(mux);
	if (index != MUX_DPIN0 && index != MUX_DPIN1)
		return -EINVAL;
	if (state < -1 || state >= 9)
		return -EINVAL;

	spin_lock_irqsave(&xbar->lock, flags);
	if (xbar->selected_dispext[index] >= 0 &&
	    xbar->selected_dispext[index] != state) {
		ret = -EBUSY;
	} else {
		dpxbar_mask32(xbar, T602X_REG_030, t602x_mux_mask(index),
			      state < 0 ? 0 : t602x_mux_set(index, state));
	}
	spin_unlock_irqrestore(&xbar->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_dpxbar_tunnel_select_source);

static int apple_dpxbar_t602x_link_up(struct mux_control *mux)
{
	struct apple_dpxbar *xbar;
	unsigned int index;
	unsigned long flags;
	u32 dispext_bit, dispext_bit_en, atc_bit;
	int state, ret = 0;

	if (!mux || mux->chip->ops != &apple_dpxbar_t602x_ops)
		return -EINVAL;
	index = mux_control_get_index(mux);
	if (index >= MUX_MAX)
		return -EINVAL;
	atc_bit = t602x_atc_bit(index);
	xbar = mux_chip_priv(mux->chip);

	spin_lock_irqsave(&xbar->lock, flags);
	state = xbar->selected_dispext[index];
	if (state < 0) {
		spin_unlock_irqrestore(&xbar->lock, flags);
		return -ENODEV;
	}
	dispext_bit = 1 << state;
	dispext_bit_en = 1 << (2 * state);

	dpxbar_clear32(xbar, T602X_FIFO_WR_N_CLK_EN, dispext_bit);
	dpxbar_clear32(xbar, T602X_REG_014, dispext_bit);
	dpxbar_clear32(xbar, T602X_FIFO_RD_PCLK2_EN, atc_bit);
	udelay(1);

	dpxbar_set32(xbar, T602X_FIFO_WR_UNK_EN, dispext_bit);
	if (index == MUX_DPIN1) {
		dpxbar_mask32(xbar, T602X_REG_018,
			       dispext_bit_en * 3, dispext_bit_en * 3);
		dpxbar_mask32(xbar, T602X_FIFO_RD_N_CLK_EN,
			       atc_bit * 3, atc_bit * 3);
	} else {
		dpxbar_set32(xbar, T602X_REG_018, dispext_bit_en);
	}
	/* set_t602x() already enabled this output's N clock. In particular,
	 * do not rewrite the low bits here: DPIN1 would clear DPIN0's clock.
	 */
	dpxbar_set32(xbar, T602X_FIFO_WR_DPTX_CLK_EN, dispext_bit);
	dpxbar_set32(xbar,
		     index == MUX_DPIN1 ? T602X_REG_010 : T602X_REG_00C,
		     dispext_bit);
	dpxbar_set32(xbar,
		     index == MUX_DPIN1 ? FIFO_RD_PCLK1_EN : T602X_REG_01C,
		     atc_bit);
	dpxbar_set32(xbar, T602X_REG_034, atc_bit);
	dpxbar_set32(xbar, CROSSBAR_ATC_EN, atc_bit);
	dpxbar_set32(xbar, CROSSBAR_DISPEXT_EN, dispext_bit);
	dpxbar_set32(xbar, T602X_FIFO_RD_UNK_EN, dispext_bit);
	spin_unlock_irqrestore(&xbar->lock, flags);

	dev_info(xbar->dev, "%s: crossbar link up (dispext=%d atc=0x%x)\n",
		 apple_dpxbar_names[index], state, atc_bit);
	return ret;
}

/* T6030: bring a selected output's connection up or down. */
static int apple_dpxbar_t6030_link_set(struct mux_control *mux, bool up)
{
	struct apple_dpxbar *xbar = mux_chip_priv(mux->chip);
	unsigned int index = mux_control_get_index(mux);
	unsigned long flags;
	int ret = 0;

	if (index >= MUX_MAX)
		return -EINVAL;
	spin_lock_irqsave(&xbar->lock, flags);
	if (xbar->selected_dispext[index] < 0)
		ret = -ENODEV;
	else
		apple_dpxbar_t6030_link(xbar, index, up);
	spin_unlock_irqrestore(&xbar->lock, flags);
	return ret;
}

int apple_dpxbar_link_up(struct mux_control *mux)
{
	if (!mux)
		return -EINVAL;
	if (mux->chip->ops == &apple_dpxbar_t602x_ops) {
		struct apple_dpxbar *xbar = mux_chip_priv(mux->chip);

		if (xbar->t6030_dpin)
			return apple_dpxbar_t6030_link_set(mux, true);
		return apple_dpxbar_t602x_link_up(mux);
	}
	return apple_dpxbar_t8103_link_up(mux);
}
EXPORT_SYMBOL_GPL(apple_dpxbar_link_up);

static int apple_dpxbar_t602x_link_down(struct mux_control *mux)
{
	struct apple_dpxbar *xbar;
	unsigned int index;
	unsigned long flags;
	u32 dispext_bit, atc_bit;
	int state;

	if (!mux || mux->chip->ops != &apple_dpxbar_t602x_ops)
		return -EINVAL;
	index = mux_control_get_index(mux);
	if (index >= MUX_MAX)
		return -EINVAL;
	atc_bit = t602x_atc_bit(index);
	xbar = mux_chip_priv(mux->chip);

	spin_lock_irqsave(&xbar->lock, flags);
	state = xbar->selected_dispext[index];
	if (state < 0) {
		spin_unlock_irqrestore(&xbar->lock, flags);
		return -ENODEV;
	}
	dispext_bit = 1 << state;
	dpxbar_set32(xbar, T602X_FIFO_WR_N_CLK_EN, dispext_bit);
	dpxbar_set32(xbar, T602X_REG_014, dispext_bit);
	dpxbar_set32(xbar, T602X_FIFO_RD_PCLK2_EN, atc_bit);
	spin_unlock_irqrestore(&xbar->lock, flags);

	dev_info(xbar->dev, "%s: crossbar link down (dispext=%d atc=0x%x)\n",
		 apple_dpxbar_names[index], state, atc_bit);
	return 0;
}

int apple_dpxbar_link_down(struct mux_control *mux)
{
	if (!mux)
		return -EINVAL;
	if (mux->chip->ops == &apple_dpxbar_t602x_ops) {
		struct apple_dpxbar *xbar = mux_chip_priv(mux->chip);

		if (xbar->t6030_dpin)
			return apple_dpxbar_t6030_link_set(mux, false);
		return apple_dpxbar_t602x_link_down(mux);
	}
	return apple_dpxbar_t8103_link_down(mux);
}
EXPORT_SYMBOL_GPL(apple_dpxbar_link_down);

static int apple_dpxbar_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_chip *mux_chip;
	struct apple_dpxbar *dpxbar;
	const struct apple_dpxbar_hw *hw;
	int ret;

	hw = of_device_get_match_data(dev);
	mux_chip = devm_mux_chip_alloc(dev, MUX_MAX, sizeof(*dpxbar));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	dpxbar = mux_chip_priv(mux_chip);
	dpxbar->hw = hw;
	mux_chip->ops = hw->ops;
	spin_lock_init(&dpxbar->lock);

	dpxbar->dev = dev;
	dpxbar->t6030_dpin = of_device_is_compatible(dev->of_node, "apple,t6020-display-crossbar") &&
			     of_machine_is_compatible("apple,t6030");
	dpxbar->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dpxbar->regs))
		return PTR_ERR(dpxbar->regs);

	if (dpxbar->t6030_dpin) {
		/*
		 * T6030: probe writes nothing. The crossbar is powered (its
		 * ATC's USB domain) from the first select until it is idle.
		 */
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
	} else if (!of_device_is_compatible(dev->of_node, "apple,t6020-display-crossbar")) {
		readl(dpxbar->regs + UNK_TUNABLE);
		writel(hw->tunable, dpxbar->regs + UNK_TUNABLE);
		readl(dpxbar->regs + UNK_TUNABLE);
	}

	for (unsigned int i = 0; i < MUX_MAX; ++i) {
		mux_chip->mux[i].states = hw->n_ufp;
		mux_chip->mux[i].idle_state = MUX_IDLE_DISCONNECT;
		dpxbar->selected_dispext[i] = -1;
	}

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, dpxbar);
	return 0;
}

/*
 * A T6030 crossbar loses its registers when its power domain goes down in
 * system sleep, but its routes stay selected: a DCP selects a fixed route
 * once at probe, and the mux core skips a select of the state it already
 * holds. Rewrite every selected route on resume.
 *
 * The link clocks are not turned back on here. The PHY's link stays stopped
 * until the DCP configures it again, and the clocks have to follow the PHY.
 * If the enables did not survive, record the link as down, so the DCP's next
 * apple_dpxbar_link_up() turns them on again.
 */
static int apple_dpxbar_resume(struct device *dev)
{
	struct apple_dpxbar *xbar = dev_get_drvdata(dev);
	unsigned long restored = 0, flags;
	unsigned int i;

	if (!xbar || !xbar->t6030_dpin)
		return 0;

	spin_lock_irqsave(&xbar->lock, flags);
	for (i = 0; i < MUX_MAX; i++) {
		int state = xbar->selected_dispext[i];
		u32 shift = apple_dpxbar_t6030_shift(i);
		u32 mask = (0xfU << shift) | (0xfU << (shift + 12));
		u32 route = (state << shift) | (state << (shift + 12));

		if (state < 0)
			continue;
		if ((readl(xbar->regs + T602X_MUX_SELECT) & mask) != route) {
			dpxbar_mask32(xbar, T602X_MUX_SELECT, mask, route);
			restored |= BIT(i);
		}
		if (xbar->tunnel_link_up[i] &&
		    (!(readl(xbar->regs + T602X_ATC_ENABLE) & BIT(shift)) ||
		     !(readl(xbar->regs + T602X_DISPEXT_ENABLE) & BIT(state)))) {
			xbar->tunnel_link_up[i] = false;
			restored |= BIT(i);
		}
	}
	spin_unlock_irqrestore(&xbar->lock, flags);

	for_each_set_bit(i, &restored, MUX_MAX)
		dev_info(dev, "%s: route %d restored after power loss, link waits for the DCP\n",
			 apple_dpxbar_names[i], xbar->selected_dispext[i]);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_dpxbar_pm_ops, NULL, apple_dpxbar_resume);

static const struct apple_dpxbar_hw apple_dpxbar_hw_t8103 = {
	.n_ufp = 2,
	.tunable = 0,
	.ops = &apple_dpxbar_ops,
};

static const struct apple_dpxbar_hw apple_dpxbar_hw_t8112 = {
	.n_ufp = 4,
	.tunable = 4278196325,
	.ops = &apple_dpxbar_ops,
};

static const struct apple_dpxbar_hw apple_dpxbar_hw_t6000 = {
	.n_ufp = 9,
	.tunable = 5,
	.ops = &apple_dpxbar_ops,
	.dpin_no_cycle_slip = true,
};

static const struct apple_dpxbar_hw apple_dpxbar_hw_t6020 = {
	.n_ufp = 9,
	.ops = &apple_dpxbar_t602x_ops,
};

static const struct of_device_id apple_dpxbar_ids[] = {
	{
		.compatible = "apple,t8103-display-crossbar",
		.data = &apple_dpxbar_hw_t8103,
	},
	{
		.compatible = "apple,t8112-display-crossbar",
		.data = &apple_dpxbar_hw_t8112,
	},
	{
		.compatible = "apple,t6000-display-crossbar",
		.data = &apple_dpxbar_hw_t6000,
	},
	{
		.compatible = "apple,t6020-display-crossbar",
		.data = &apple_dpxbar_hw_t6020,
	},
	{}
};
MODULE_DEVICE_TABLE(of, apple_dpxbar_ids);

static struct platform_driver apple_dpxbar_driver = {
	.driver = {
		.name = "apple-display-crossbar",
		.of_match_table	= apple_dpxbar_ids,
		.pm = pm_sleep_ptr(&apple_dpxbar_pm_ops),
	},
	.probe = apple_dpxbar_probe,
};
module_platform_driver(apple_dpxbar_driver);

MODULE_DESCRIPTION("Apple Silicon display crossbar multiplexer driver");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("GPL v2");
