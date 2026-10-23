/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple T8152 (M6) ATC PHY: register sequences, in the order the vendor driver runs them.
 * Kept free of driver glue so the same code can be run against a fake memory.
 *
 * All offsets are relative to the 'core' window (ADT atc-phy reg[2], 0x84000 bytes) unless a
 * function takes the pipe handler or config window.
 * Block layout: XBAR 0x0, ACIOPHY_CMN 0x800, CIOPLL core 0x1600, lane AON 0x1e00,
 * PIPE/BIST 0x2400, USB PCS 0x15c00, POWER 0x60000, ATC_FABRIC 0x6c000.
 */
#ifndef _ATC_T8152_SEQ_H
#define _ATC_T8152_SEQ_H

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/soc/apple/tunable.h>

#include "atc-t8152-tables.h"

#define T8152_XBAR		0x0
#define T8152_XBAR_PROTO	0x4c
#define T8152_LANE		0x1e00
#define T8152_LANE0_CTRL	0x18
#define T8152_LANE1_CTRL	0x1c
#define T8152_LANE_FINAL	0x24
#define T8152_PIPE		0x2400
#define T8152_POWER		0x60000
#define T8152_PWR0		0x00
#define T8152_PWR1		0x04
#define T8152_PWR_SPARE		0x08
#define T8152_PWR_S2R_SYNC	0x30

#define T8152_PWR0_SML_IN	BIT(0)
#define T8152_PWR0_BIG_IN	BIT(1)
#define T8152_PWR0_BIT2		BIT(2)
#define T8152_PWR0_CLAMP_EN	BIT(3)
#define T8152_PWR0_RESET_N	BIT(4)
#define T8152_PWR1_SML_OUT	BIT(0)
#define T8152_PWR1_BIG_OUT	BIT(1)

#define T8152_PIPE_CTRL		0x00
#define T8152_PIPE_STROBE	0x0c
#define T8152_PIPE_STATUS	0xa0
#define T8152_PIPE_OV1		0xb0
#define T8152_PIPE_OV2		0xb8

#define T8152_PH_OVERRIDE	0x00
#define T8152_PH_OVERRIDE_VALUES 0x04
#define T8152_PH_MUX		0x0c
#define T8152_PH_LOCK_REQ	0x10
#define T8152_PH_LOCK_ACK	0x14
#define T8152_PH_NONSELECTED	0x20
/* the T8152 live boot value is 0x42: clock field bits 6:4, data bits 2:0 */
#define T8152_PH_MUX_CLK_SHIFT	4

#define T8152_CFG_OFFSET	0x10000	/* usb-drd reg[1] = dwc3 core + 0x10000 */
#define T8152_CFG_PHY_READY_REG	0x2c
#define T8152_CFG_PHY_READY	BIT(30)

/* Lanes of a CIO bring-up (partB x3): bit 0 lane 0, bit 1 lane 1. */
#define T8152_LANES_BOTH	3

struct t8152_regs {
	void __iomem *core;
	void __iomem *pipeh;
	void __iomem *cfg;	/* optional: dwc3 config window (PIPE_PHY_READY) */
};

static inline void t8152_wr(void __iomem *base, u32 off, u32 clear, u32 set)
{
	writel((readl(base + off) & ~clear) | set, base + off);
}

/* The vendor driver rewrites every record even when the value is unchanged, so this does not use
 * apple_tunable_apply(), which skips equal values.
 */
static inline void t8152_apply(const struct t8152_regs *r,
			       struct apple_tunable *const *tun, unsigned int i)
{
	void __iomem *blk = r->core + t8152_tunables[i].off;
	size_t n;

	if (!tun[i])
		return;
	for (n = 0; n < tun[i]->sz; n++)
		t8152_wr(blk, tun[i]->values[n].offset,
			 tun[i]->values[n].mask, tun[i]->values[n].value);
}

static inline void t8152_apply_seq(const struct t8152_regs *r,
				   struct apple_tunable *const *tun,
				   const u8 *seq, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		t8152_apply(r, tun, seq[i]);
}

/* Sleeps between reads: every caller runs under atc->lock, so only for process context. */
static inline int t8152_poll_us(void __iomem *base, u32 off, u32 bit,
				bool want_set, unsigned int timeout_us, unsigned int step_us)
{
	u32 val;

	return read_poll_timeout(readl, val, !!(val & bit) == want_set,
				 step_us, timeout_us, false, base + off);
}

/* Stage A: wake the PHY power block. */
static inline int t8152_powerup_rst(const struct t8152_regs *r)
{
	void __iomem *p = r->core + T8152_POWER;
	int ret;

	t8152_wr(p, T8152_PWR_S2R_SYNC, 0, BIT(0));
	usleep_range(1000, 2000);
	t8152_wr(p, T8152_PWR0, 0, T8152_PWR0_SML_IN);
	ret = t8152_poll_us(p, T8152_PWR1, T8152_PWR1_SML_OUT, true, 1000000, 1);
	if (ret)
		return ret;
	t8152_wr(p, T8152_PWR0, 0, T8152_PWR0_BIG_IN);
	ret = t8152_poll_us(p, T8152_PWR1, T8152_PWR1_BIG_OUT, true, 1000000, 1);
	if (ret)
		return ret;
	t8152_wr(p, T8152_PWR0, T8152_PWR0_BIT2, 0);
	usleep_range(1000, 2000);
	t8152_wr(p, T8152_PWR0, 0, T8152_PWR0_CLAMP_EN);
	return 0;
}

/* Stage C: power up one lane (bit 5 of the lane control register is the AHB reset). */
static inline void t8152_lane_up(const struct t8152_regs *r, unsigned int lane)
{
	void __iomem *a = r->core + T8152_LANE;
	u32 ctrl = lane ? T8152_LANE1_CTRL : T8152_LANE0_CTRL;

	t8152_wr(a, T8152_LANE_FINAL, 0, lane ? 0x4000 : 0x1000);
	t8152_wr(a, T8152_LANE_FINAL, 0, lane ? 0x40000 : 0x10000);
	t8152_wr(a, ctrl, 0, 0x2);
	t8152_wr(a, ctrl, 0, 0x4);
	t8152_wr(a, ctrl, 0x1, 0);
	t8152_wr(a, ctrl, 0, BIT(5));
}

/* SPARE_REGISTERS_0 bit 1 follows the orientation flag; then ACIOPHY_RESET_N is deasserted. */
static inline void t8152_release_reset(const struct t8152_regs *r, bool flag)
{
	void __iomem *p = r->core + T8152_POWER;

	t8152_wr(p, T8152_PWR_SPARE, 0x2, flag ? 0x2 : 0);
	t8152_wr(p, T8152_PWR0, 0, T8152_PWR0_RESET_N);
	usleep_range(1000, 2000);
}

/* USB3 transport (4): one lane, crossbar 0x110 (normal) or 0x111 (swapped), USB tunable lists. */
static inline int t8152_up_usb3(const struct t8152_regs *r,
				struct apple_tunable *const *tun, bool swap)
{
	int ret;

	ret = t8152_powerup_rst(r);
	if (ret)
		return ret;
	t8152_apply_seq(r, tun, t8152_seq_efuse, ARRAY_SIZE(t8152_seq_efuse));
	t8152_lane_up(r, swap);
	usleep_range(1000, 2000);
	t8152_wr(r->core + T8152_XBAR, T8152_XBAR_PROTO, 0x1ffff, swap ? 0x111 : 0x110);
	t8152_apply_seq(r, tun, t8152_seq_usb3, ARRAY_SIZE(t8152_seq_usb3));
	t8152_release_reset(r, swap);
	return 0;
}

/* CIO / USB4 transport (2): lanes from the bit mask, crossbar low 5 bits = orientation, CIO
 * tunable lists.
 */
static inline int t8152_up_cio(const struct t8152_regs *r,
			       struct apple_tunable *const *tun, bool flag, unsigned int lanes)
{
	int ret;

	ret = t8152_powerup_rst(r);
	if (ret)
		return ret;
	t8152_apply_seq(r, tun, t8152_seq_efuse, ARRAY_SIZE(t8152_seq_efuse));
	if (lanes & BIT(0))
		t8152_lane_up(r, 0);
	if (lanes & BIT(1))
		t8152_lane_up(r, 1);
	usleep_range(1000, 2000);
	t8152_wr(r->core + T8152_XBAR, T8152_XBAR_PROTO, 0x1f, flag ? 1 : 0);
	t8152_apply_seq(r, tun, t8152_seq_cio, ARRAY_SIZE(t8152_seq_cio));
	t8152_release_reset(r, flag);
	return 0;
}

/*
 * Stage H, PIPE/BIST override enable: the PIPE overrides the USB3 link needs, held for the whole
 * attach (the vendor driver only releases them on power-down). Returns -ETIMEDOUT if the PHY
 * never reports ready (STATUS bit 6). The two waits on STATUS bit 7 are best effort: a timeout
 * there does not stop the bring-up.
 */
static inline int t8152_pipe_override_enable(const struct t8152_regs *r)
{
	void __iomem *q = r->core + T8152_PIPE;
	int ret;

	t8152_wr(q, T8152_PIPE_CTRL, 0, 0x1);
	t8152_wr(q, T8152_PIPE_OV1, 0x1a0, 0x41);
	t8152_wr(q, T8152_PIPE_OV2, 0x440, 0x220);
	t8152_wr(q, T8152_PIPE_CTRL, 0, 0x40);
	t8152_wr(q, T8152_PIPE_CTRL, 0x40, 0);
	t8152_poll_us(q, T8152_PIPE_STATUS, BIT(7), false, 20000, 50);
	t8152_wr(q, T8152_PIPE_STROBE, 0, 0x1);
	t8152_wr(q, T8152_PIPE_STROBE, 0x1, 0);
	t8152_wr(q, T8152_PIPE_OV1, 0, 0x60);
	t8152_wr(q, T8152_PIPE_OV2, 0, 0x200);
	t8152_wr(q, T8152_PIPE_CTRL, 0, 0x40);
	t8152_wr(q, T8152_PIPE_CTRL, 0x40, 0);
	ret = t8152_poll_us(q, T8152_PIPE_STATUS, BIT(6), true, 130000, 50);
	t8152_poll_us(q, T8152_PIPE_STATUS, BIT(7), false, 1000, 50);
	t8152_wr(q, T8152_PIPE_STROBE, 0, 0x1);
	t8152_wr(q, T8152_PIPE_STROBE, 0x1, 0);
	return ret;
}

/* Stage 5: route the dwc3 PIPE from the dummy backend to the USB3 PHY, like
 * atcphy_configure_pipehandler_usb3(host) in atc.c. *routed is set as soon as the mux points at
 * the PHY, before the waits: their failure leaves the route in place, so the caller must park it
 * at shutdown.
 */
static inline int t8152_pipehandler_usb3_host(const struct t8152_regs *r, bool *routed)
{
	void __iomem *h = r->pipeh;
	int ret;

	*routed = false;
	if (readl(h + T8152_PH_LOCK_ACK) & 1) {
		t8152_wr(h, T8152_PH_LOCK_REQ, 0x1, 0);
		ret = t8152_poll_us(h, T8152_PH_LOCK_ACK, BIT(0), false, 1000, 50);
		if (ret)
			return ret;
	}
	t8152_wr(h, T8152_PH_OVERRIDE_VALUES, BIT(1) | BIT(2), 0);
	t8152_wr(h, T8152_PH_OVERRIDE, 0, BIT(0));
	t8152_wr(h, T8152_PH_OVERRIDE, 0, BIT(2));
	t8152_wr(h, T8152_PH_LOCK_REQ, 0, 0x1);
	ret = t8152_poll_us(h, T8152_PH_LOCK_ACK, BIT(0), true, 1000, 50);
	if (ret) {
		t8152_wr(h, T8152_PH_LOCK_REQ, 0x1, 0);
		t8152_wr(h, T8152_PH_OVERRIDE, BIT(0) | BIT(2), 0);
		return ret;
	}
	t8152_wr(h, T8152_PH_NONSELECTED, 0xf, 0x3);
	t8152_wr(h, T8152_PH_NONSELECTED, BIT(12), 0);
	t8152_wr(h, T8152_PH_MUX, 0x7 << T8152_PH_MUX_CLK_SHIFT, 0);
	udelay(10);
	t8152_wr(h, T8152_PH_MUX, GENMASK(2, 0), 0);
	udelay(10);
	t8152_wr(h, T8152_PH_MUX, 0x7 << T8152_PH_MUX_CLK_SHIFT, 1 << T8152_PH_MUX_CLK_SHIFT);
	*routed = true;
	udelay(10);
	t8152_wr(h, T8152_PH_OVERRIDE, BIT(0), 0);
	t8152_wr(h, T8152_PH_OVERRIDE, BIT(2), 0);
	t8152_wr(h, T8152_PH_LOCK_REQ, 0x1, 0);
	return t8152_poll_us(h, T8152_PH_LOCK_ACK, BIT(0), false, 1000, 50);
}

/* The vendor driver waits for PIPE_PHY_READY whenever the mux is not on the dummy backend. Safe
 * to repeat while the route stands: a second set_mode(HOST) after a failed wait re-runs it.
 */
static inline int t8152_pipehandler_usb3_ready(const struct t8152_regs *r)
{
	if (!r->cfg)
		return 0;
	return t8152_poll_us(r->cfg, T8152_CFG_PHY_READY_REG, T8152_CFG_PHY_READY,
			     true, 100000, 50);
}

/*
 * Power-down, in the order of the vendor driver (it shuts the PHY down before every transport
 * change and then re-runs the whole bring-up, so a mode switch is shutdown + up):
 *   aux shutdown: lane AON +0x24 |= 0x400, &= ~0x40, &= ~0x100            (1 us after each)
 *   lane AON +0x24 &= ~0x1000, ~0x10000, ~0x4000, ~0x40000   (the LFPS FINAL bits of lane_pwrup)
 *   lane power-down for lane 0, then lane 1: AHB_RESET_N bit 5 clear, ctrl |= 1, &= ~4, &= ~2
 *     (1 us after each)
 *   POWER +0x0 &= ~0x10 (RESET_N), 1 us, &= ~0x8, |= 0x4, &= ~0x2, wait +0x4 bit 1 clear,
 *     &= ~0x1, wait +0x4 bit 0 clear, POWER +0x30 &= ~0x1
 * The xbar value is left as it is, like the vendor driver. The PIPE overrides are not released
 * here, the vendor driver does not release them either.
 */
static inline int t8152_shutdown(const struct t8152_regs *r)
{
	void __iomem *a = r->core + T8152_LANE;
	void __iomem *p = r->core + T8152_POWER;
	int ret, ret2;

	t8152_wr(a, T8152_LANE_FINAL, 0, 0x400);
	udelay(1);
	t8152_wr(a, T8152_LANE_FINAL, 0x40, 0);
	udelay(1);
	t8152_wr(a, T8152_LANE_FINAL, 0x100, 0);
	udelay(1);
	t8152_wr(a, T8152_LANE_FINAL, 0x1000, 0);
	t8152_wr(a, T8152_LANE_FINAL, 0x10000, 0);
	t8152_wr(a, T8152_LANE_FINAL, 0x4000, 0);
	t8152_wr(a, T8152_LANE_FINAL, 0x40000, 0);
	udelay(1);
	for (unsigned int lane = 0; lane < 2; lane++) {
		u32 ctrl = lane ? T8152_LANE1_CTRL : T8152_LANE0_CTRL;

		t8152_wr(a, ctrl, BIT(5), 0);
		udelay(1);
		t8152_wr(a, ctrl, 0, 0x1);
		udelay(1);
		t8152_wr(a, ctrl, 0x4, 0);
		udelay(1);
		t8152_wr(a, ctrl, 0x2, 0);
		udelay(1);
	}
	t8152_wr(p, T8152_PWR0, T8152_PWR0_RESET_N, 0);
	udelay(1);
	t8152_wr(p, T8152_PWR0, T8152_PWR0_CLAMP_EN, 0);
	t8152_wr(p, T8152_PWR0, 0, T8152_PWR0_BIT2);
	t8152_wr(p, T8152_PWR0, T8152_PWR0_BIG_IN, 0);
	ret = t8152_poll_us(p, T8152_PWR1, T8152_PWR1_BIG_OUT, false, 1000000, 1);
	t8152_wr(p, T8152_PWR0, T8152_PWR0_SML_IN, 0);
	ret2 = t8152_poll_us(p, T8152_PWR1, T8152_PWR1_SML_OUT, false, 1000000, 1);
	t8152_wr(p, T8152_PWR_S2R_SYNC, 0x1, 0);
	return ret ? ret : ret2;
}

/*
 * Route the dwc3 PIPE back to the dummy backend (USB2 only): the mirror of
 * t8152_pipehandler_usb3_host() and the order of atcphy_configure_pipehandler_dummy() in atc.c
 * with the T8152 mux layout.
 */
static inline int t8152_pipehandler_park(const struct t8152_regs *r)
{
	void __iomem *h = r->pipeh;
	int ret;

	t8152_wr(h, T8152_PH_OVERRIDE_VALUES, BIT(1) | BIT(2), 0);
	t8152_wr(h, T8152_PH_OVERRIDE, 0, BIT(0));
	t8152_wr(h, T8152_PH_OVERRIDE, 0, BIT(2));
	t8152_wr(h, T8152_PH_LOCK_REQ, 0, 0x1);
	ret = t8152_poll_us(h, T8152_PH_LOCK_ACK, BIT(0), true, 1000, 50);
	if (ret) {
		t8152_wr(h, T8152_PH_LOCK_REQ, 0x1, 0);
		t8152_wr(h, T8152_PH_OVERRIDE, BIT(0) | BIT(2), 0);
		return ret;
	}
	t8152_wr(h, T8152_PH_MUX, 0x7 << T8152_PH_MUX_CLK_SHIFT, 0);
	udelay(10);
	t8152_wr(h, T8152_PH_MUX, GENMASK(2, 0), 2);
	udelay(10);
	t8152_wr(h, T8152_PH_MUX, 0x7 << T8152_PH_MUX_CLK_SHIFT, 4 << T8152_PH_MUX_CLK_SHIFT);
	udelay(10);
	t8152_wr(h, T8152_PH_OVERRIDE, BIT(0), 0);
	t8152_wr(h, T8152_PH_OVERRIDE, BIT(2), 0);
	t8152_wr(h, T8152_PH_LOCK_REQ, 0x1, 0);
	ret = t8152_poll_us(h, T8152_PH_LOCK_ACK, BIT(0), false, 1000, 50);
	t8152_wr(h, T8152_PH_NONSELECTED, 0xf, 0x2);
	t8152_wr(h, T8152_PH_NONSELECTED, 0, BIT(12));
	return ret;
}

#endif
