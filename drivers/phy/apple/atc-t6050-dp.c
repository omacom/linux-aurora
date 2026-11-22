// SPDX-License-Identifier: GPL-2.0-only
/* T6050 DisplayPort PHY for fixed bridges and Type-C DP alternate mode. */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <dt-bindings/phy/phy.h>
#include "usb-t6050/apple_t6050_usb_phy.h"
#include "usb-t6050/apple_t6050_usb3_pipe.h"
#include "usb-t6050/apple_t6050_usb2_reset.h"
#include "usb-t6050/apple_t6050_cio_phy.h"
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/soc/apple/tunable.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/phy/apple-atc.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/sleep.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_tbt.h>

enum atc_operation { ATC_MASK, ATC_TUNABLE, ATC_DELAY, ATC_POLL, ATC_POWER, ATC_FIRMWARE };
struct atc_op {
	u8 op, bank;
	u16 offset;
	u32 mask, value, timeout;
};
struct atc_rate_op {
	struct atc_op op;
	u8 rates;
	u32 values[4];
};
#include "atc-t6050-dp-data.h"
#include "atc-t6050-typec-data.h"

static const struct { u8 bank; u32 offset; } core_banks[] = {
	{2, 0}, {3, 0x400}, {4, 0x800}, {5, 0xa00}, {6, 0x1000},
	{7, 0x3000}, {8, 0x3200}, {9, 0x3800}, {10, 0x3a00},
	{14, 0x7000}, {16, 0x9000}, {18, 0xb000}, {19, 0xc000},
	{20, 0xd000}, {21, 0xe000}, {22, 0xf000}, {25, 0x19000},
	{27, 0x1b000}, {28, 0x1c000}, {29, 0x1d000}, {30, 0x1e000},
	{31, 0x1f000}, {33, 0x28000}, {34, 0x28400}, {35, 0x29000},
};
static const struct { u8 bank; const char *name; } extra_banks[] = {
	{39, "reset"}, {40, "fabric"}, {41, "axi2af"},
	{42, "lioa"}, {43, "common-cfg"},
	{38, "usb-control"},
};
static const struct { u8 bank; const char *name; } tunable_names[] = {
	{41, "apple,tunable-axi2af"}, {40, "apple,tunable-fabric"},
	{43, "apple,tunable-common-cfg"}, {42, "apple,tunable-lioa"},
	{20, "apple,tunable-lane0-rx-clock"}, {29, "apple,tunable-lane1-rx-clock"},
	{18, "apple,tunable-lane0-rx-eq"}, {27, "apple,tunable-lane1-rx-eq"},
	{8, "apple,tunable-auspll"},
	{5, "apple,tunable-aus40cmn-shm"}, {10, "apple,tunable-cio4pll"},
};
static const unsigned int link_rates[] = {1620, 2700, 5400, 8100};
static const char * const training_names[] = {
	"apple,dp-training-table-rbr", "apple,dp-training-table-hbr",
	"apple,dp-training-table-hbr2", "apple,dp-training-table-hbr3",
};
struct atc_domain {
	struct device *dev;
	bool active;
};
enum combo_mode { COMBO_SAFE, COMBO_USB, COMBO_DP4, COMBO_DP2, COMBO_CIO };

/*
 * USB4 DisplayPort tunnel pixel clocks, k = 1..4. Each has a clock gate, a
 * source select word in the power manager's clock block (the DP link rate's
 * source, then a busy bit to wait out), and a switch in the PHY's CFG block
 * (bank 39): +0x14 holds a 3-bit field per clock (k = on, 7 = parked) and a
 * UHBR bit 11 + k, +0x18 an enable bit per clock, +0x1c its status. Switch k
 * feeds the display crossbar's pixel clock k.
 */
#define ATC_DP_TUNNELS		4
#define ATC_CFG_DP_SWITCH	0x14
#define ATC_CFG_DP_ENABLE	0x18
#define ATC_CFG_DP_STATUS	0x1c
#define ATC_DP_SWITCH_PARKED	7
#define ATC_DP_SRC_SELECT	GENMASK(27, 24)
#define ATC_DP_SRC_BUSY		BIT(30)

/* The PHY CPU's control word, in bank 38, and its run bit */
#define ATC_CPU_CONTROL		0x28
#define ATC_CPU_RUN		BIT(4)

struct t6050_dp {
	bool combo, usb_prepared, usb2_active, pipe_active, controller_active, cio_prepared;
	bool pipe_tunnel;	/* the active PIPE is a USB4 router's USB3 tunnel */
	enum combo_mode combo_mode;
	enum typec_orientation orientation;
	struct phy *combo_phy[3];
	struct reset_controller_dev reset;
	struct apple_t6050_usb_io usb_io;
	struct apple_t6050_usb_platform usb_platform;
	struct apple_tunable *usb2_default, *usb2_host, *pipe_default;
	struct apple_tunable *cio_tunables[5]; /* optional; empty on J716c except RX_TOP */
	struct device *dev;
	struct mutex lock;
	void __iomem *banks[45];
	struct resource resources[45];
	struct apple_tunable *tunables[ARRAY_SIZE(tunable_names)];
	/* USB AON, common, wrapper: native order; a combo may add USB. */
	struct atc_domain domain[4];
	unsigned int ndomains;
	u32 training[4][16];
	u32 voltage[4], pre[4];
	int rate;
	bool enabled, faulted;
	bool typec, reverse, multifunction;
	struct apple_rtkit *rtkit;
	void __iomem *fw_data;
	bool fw_running;
	bool fw_cold;		/* idled for a system sleep and halted: its next start is a cold one */
	void *fw_data_saved;	/* the firmware data SRAM as the first start found it */
	/* USB4 DP tunnel pixel clocks: source words, gates, each one's link rate (0: off) */
	void __iomem *dp_clk_src;
	struct atc_domain dp_gate[ATC_DP_TUNNELS];
	bool dp_gates;
	u8 tunnel_rate[ATC_DP_TUNNELS];
	/* A combo PHY's system sleep. */
	u32 port;
	struct notifier_block pm_nb;
	enum combo_mode sleep_mode;	/* the mode the sleep's stop took down */
	bool sleep_stopped;		/* stopped for a sleep that has not been accounted for yet */
	bool aon_syscore;		/* the always-on domain is held out of this system transition */
	bool aon_kept;			/* the stop left the always-on domain's reference in place */
	/* A USB4 router's sleep in place: its host stops and restarts the port, see the end. */
	bool cio_sleep;			/* asked for: the PHY's own stop for the sleep is skipped */
	bool cio_sleep_off;		/* stopped by the router's host */
	bool cio_sleep_hold;		/* the domains' references are left alone while it runs */
	bool cio_tunnel_lost;		/* the router woke without its dock: no USB3 tunnel PIPE */
	/* TEST (J714s): the USB host kept on the tunnel's PIPE through the router's sleep. */
	bool cio_usb_host;		/* a router host that sleeps in place is bound to this PHY */
	bool cio_usb_kept;		/* kept for this sleep, until its USB controller's RESULT */
	bool cio_usb_parked;		/* its PIPE parked behind the router's sleep or refusal */
};

static int atc_domain_set(struct atc_domain *pd, bool active)
{
	int ret;

	if (pd->active == active)
		return 0;
	if (active) {
		ret = pm_runtime_resume_and_get(pd->dev);
		if (ret < 0)
			return ret;
	} else {
		ret = pm_runtime_put_sync_suspend(pd->dev);
		if (ret < 0) {
			/* put decremented usage even when suspend failed. */
			pm_runtime_get_noresume(pd->dev);
			return ret;
		}
	}
	pd->active = active;
	return 0;
}

static void atc_domain_release(void *data)
{
	struct atc_domain *pd = data;

	if (pd->active)
		pm_runtime_put_sync_suspend(pd->dev);
	dev_pm_domain_detach(pd->dev, true);
}

/* The bootloader preloads CIO80 firmware into local SRAM. The firmware's
 * data segment is 0x10000000..0x10010000, not guest system RAM. */
static int atc_fw_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct t6050_dp *dp = cookie;

	if (bfr->iova < 0x10000000 ||
	    bfr->size > 0x10000 || bfr->iova - 0x10000000 > 0x10000 - bfr->size) {
		dev_err(dp->dev, "unsupported PHY firmware buffer %llx size %zx\n",
			(unsigned long long)bfr->iova, bfr->size);
		return -EINVAL;
	}
	bfr->iomem = dp->fw_data + (bfr->iova - 0x10000000);
	bfr->is_mapped = true;
	return 0;
}

static void atc_fw_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* SRAM is owned by the PHY and mapped for the device lifetime. */
}

static void atc_fw_message(void *cookie, u8 endpoint, u64 message)
{
	struct t6050_dp *dp = cookie;

	dev_dbg(dp->dev, "PHY firmware ep%x: %016llx\n", endpoint, message);
}

static void atc_fw_crashed(void *cookie, const void *log, size_t size)
{
	struct t6050_dp *dp = cookie;

	WRITE_ONCE(dp->faulted, true);
	dev_err(dp->dev, "PHY firmware crashed\n");
}

static const struct apple_rtkit_ops atc_fw_ops = {
	.shmem_setup = atc_fw_shmem_setup,
	.shmem_destroy = atc_fw_shmem_destroy,
	.recv_message = atc_fw_message,
	.crashed = atc_fw_crashed,
};

static int atc_firmware(struct t6050_dp *dp, bool start)
{
	int ret;

	if (!dp->rtkit)
		return -ENODEV;
	if (dp->fw_running == start)
		return 0;
	if (start) {
		/* After a sleep's idle the CPU was halted: RTKit starts over, receiving on again. */
		if (dp->fw_cold) {
			ret = apple_rtkit_reinit(dp->rtkit);
			if (ret)
				goto out;
			apple_rtkit_set_recv_enabled(dp->rtkit, true);
			dp->fw_cold = false;
		}
		/*
		 * The firmware is the boot loader's, preloaded with its data segment, and keeps
		 * its state there: restarted on the state its last run left, it never finishes
		 * calibrating the lanes (UC_TNBL_DONE). Give every start the data segment the
		 * first one found. The PHY's domains are on here, so the SRAM is accessible.
		 */
		if (!dp->fw_data_saved) {
			dp->fw_data_saved = devm_kmalloc(dp->dev, 0x10000, GFP_KERNEL);
			if (!dp->fw_data_saved)
				return -ENOMEM;
			memcpy_fromio(dp->fw_data_saved, dp->fw_data, 0x10000);
		} else {
			memcpy_toio(dp->fw_data, dp->fw_data_saved, 0x10000);
		}
		/* Native coprocessor run, after its +0x40 firmware enable. */
		writel(ATC_CPU_RUN, dp->banks[38] + ATC_CPU_CONTROL);
		ret = apple_rtkit_wake(dp->rtkit);
		if (!ret)
			ret = apple_rtkit_start_ep(dp->rtkit, 0x20);
	} else if (pm_suspend_target_state != PM_SUSPEND_ON && pm_suspend_via_firmware()) {
		/*
		 * A system sleep: the PHY CPU idled, with no power-down message: AP IDLE and its
		 * echo, IOP IDLE and its ack, receiving off, then the CPU's run bit off as at a
		 * shutdown. The next start is cold (above).
		 */
		ret = apple_rtkit_idle(dp->rtkit);
		if (!ret) {
			apple_rtkit_set_recv_enabled(dp->rtkit, false);
			writel((u32)~ATC_CPU_RUN, dp->banks[38] + ATC_CPU_CONTROL);
			dp->fw_cold = true;
			dev_dbg(dp->dev, "PHY firmware idled for a system sleep\n");
		}
	} else {
		/* The PHY mailbox Power Down message, then the RTKit power transition. */
		ret = apple_rtkit_send_message(dp->rtkit, 0x20, BIT_ULL(56), NULL, false);
		if (!ret)
			ret = apple_rtkit_shutdown(dp->rtkit);
		if (!ret)
			writel((u32)~ATC_CPU_RUN, dp->banks[38] + ATC_CPU_CONTROL);
	}
out:
	if (ret)
		dev_err(dp->dev, "PHY firmware %s failed: %d\n", start ? "start" : "stop", ret);
	else
		dp->fw_running = start;
	return ret;
}

static int atc_validate_step(struct t6050_dp *dp, const struct atc_op *s)
{
	if (s->op == ATC_DELAY)
		return s->value <= 10000 ? 0 : -EINVAL;
	if (s->op == ATC_POWER)
		return s->value <= 1 ? 0 : -EINVAL;
	if (s->op == ATC_FIRMWARE)
		return dp->typec && s->value <= 1 ? 0 : -EINVAL;
	if (s->bank >= ARRAY_SIZE(dp->banks) || !dp->banks[s->bank] ||
	    s->offset % 4 || s->offset + 4 > resource_size(&dp->resources[s->bank]))
		return -EINVAL;
	if (s->op == ATC_TUNABLE)
		return s->value < ARRAY_SIZE(tunable_names) &&
		       tunable_names[s->value].bank == s->bank ? 0 : -EINVAL;
	if (s->op == ATC_MASK || s->op == ATC_POLL)
		return s->value & ~s->mask ? -EINVAL : 0;
	return -EINVAL;
}

static int atc_step(struct t6050_dp *dp, const struct atc_op *s)
{
	void __iomem *reg;
	u32 value;
	int ret = atc_validate_step(dp, s);

	if (ret)
		return ret;
	if (s->op == ATC_DELAY) {
		udelay(s->value);
		return 0;
	}
	if (s->op == ATC_FIRMWARE)
		return atc_firmware(dp, s->value);
	if (s->op == ATC_POWER)
		return atc_domain_set(&dp->domain[2], s->value);
	if (s->bank >= ARRAY_SIZE(dp->banks) || !dp->banks[s->bank] ||
	    s->offset + 4 > resource_size(&dp->resources[s->bank]))
		return -EINVAL;
	reg = dp->banks[s->bank] + s->offset;
	switch (s->op) {
	case ATC_MASK:
		value = readl(reg);
		writel((value & ~s->mask) | s->value, reg);
		return 0;
	case ATC_TUNABLE:
		if (s->value >= ARRAY_SIZE(dp->tunables))
			return -EINVAL;
		apple_tunable_apply(dp->banks[s->bank], dp->tunables[s->value]);
		return 0;
	case ATC_POLL:
		ret = readl_poll_timeout(reg, value, (value & s->mask) == s->value,
					s->timeout > 1000 ? 1000 : 1, s->timeout);
		if (ret)
			dev_err(dp->dev, "bank%u+%x timeout: %x & %x != %x\n",
				s->bank, s->offset, value, s->mask, s->value);
		return ret;
	default:
		return -EINVAL;
	}
}

static int atc_sequence(struct t6050_dp *dp, const struct atc_op *s, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		int ret = atc_step(dp, &s[i]);

		if (ret) {
			dp->faulted = true;
			return ret;
		}
	}
	return 0;
}
#define ATC_SEQUENCE(dp, seq) atc_sequence(dp, seq, ARRAY_SIZE(seq))

static int atc_link_rate(struct t6050_dp *dp, int rate)
{
	const struct atc_rate_op *on = dp_link_on;
	size_t nr_on = ARRAY_SIZE(dp_link_on);
	int ret;

	if (dp->multifunction) {
		on = dp->reverse ? dp_group0_on : dp_group1_on;
		nr_on = dp->reverse ? ARRAY_SIZE(dp_group0_on) : ARRAY_SIZE(dp_group1_on);
	}
	if (dp->rate == rate)
		return 0;
	if (dp->rate >= 0) {
		if (dp->multifunction)
			ret = dp->reverse ? ATC_SEQUENCE(dp, dp_group0_off) :
					    ATC_SEQUENCE(dp, dp_group1_off);
		else
			ret = ATC_SEQUENCE(dp, dp_link_off);
		if (ret)
			return ret;
		dp->rate = -1;
	}
	if (rate < 0)
		return 0;
	for (size_t i = 0; i < nr_on; i++) {
		struct atc_op op = on[i].op;

		if (!(on[i].rates & BIT(rate)))
			continue;
		op.value = on[i].values[rate];
		ret = atc_step(dp, &op);
		if (ret) {
			dp->faulted = true;
			return ret;
		}
	}
	dp->rate = rate;
	memset(dp->voltage, 0, sizeof(dp->voltage));
	memset(dp->pre, 0, sizeof(dp->pre));
	return 0;
}

static int atc_start(struct t6050_dp *dp)
{
	int ret;

	if (dp->faulted)
		return -EIO;
	if (dp->enabled)
		return 0;
	ret = atc_domain_set(&dp->domain[0], true);
	if (ret)
		return ret;
	ret = atc_domain_set(&dp->domain[1], true);
	if (ret) {
		atc_domain_set(&dp->domain[0], false);
		return ret;
	}
	if (dp->multifunction)
		ret = dp->reverse ? ATC_SEQUENCE(dp, mf_reverse_init) :
				    ATC_SEQUENCE(dp, mf_normal_init);
	else
		ret = dp->reverse ? ATC_SEQUENCE(dp, dp_reverse_init) :
				    ATC_SEQUENCE(dp, dp_init);
	if (!ret)
		ret = ATC_SEQUENCE(dp, aux_on);
	if (ret)
		return ret;
	dp->enabled = true;
	return 0;
}

static int atc_stop(struct t6050_dp *dp)
{
	int ret;

	if (dp->faulted)
		return -EIO;
	if (!dp->enabled)
		return 0;
	ret = atc_link_rate(dp, -1);
	if (!ret)
		ret = ATC_SEQUENCE(dp, aux_off);
	if (!ret)
		ret = dp->multifunction ? ATC_SEQUENCE(dp, mf_poweroff) :
					 ATC_SEQUENCE(dp, dp_poweroff);
	if (ret)
		return ret;
	dp->enabled = false;
	/* Combo owns domain shutdown after USB2 and PIPE have also quiesced. */
	if (dp->combo)
		return 0;
	ret = atc_domain_set(&dp->domain[1], false);
	if (!ret)
		ret = atc_domain_set(&dp->domain[0], false);
	return ret;
}

static void atc_mask(void __iomem *reg, u32 mask, u32 value)
{
	writel((readl(reg) & ~mask) | value, reg);
}

static void atc_drive(struct t6050_dp *dp, unsigned int lane, u32 preset)
{
	/* Native DP lane order is RX/TX in each orientation-selected group. */
	unsigned int group = (lane < 2 ? 1 : 0) ^ dp->reverse;
	bool tx = lane & 1;
	void __iomem *base = dp->banks[(tx ? 21 : 20) + 9 * group];
	u32 ctrl = ((preset >> 2) & 0xf) | ((preset & 3) << 14);
	u32 eq = ((preset >> 5) & 0x7fffe) | 1;

	if (tx)
		atc_mask(base + 0x100, BIT(20), BIT(20));
	atc_mask(base + (tx ? 0x58 : 0x68), 0xc00f, ctrl);
	atc_mask(base + (tx ? 0x104 : 0xac), BIT(29), BIT(29));
	atc_mask(base + (tx ? 0x100 : 0xa8), 0x7ffff, eq);
}

static u32 atc_dp_switch_mask(unsigned int k)
{
	return GENMASK(3 * k - 1, 3 * k - 3) | BIT(11 + k);
}

/* Enable (and status) bits 1, 8, 15 and 22 */
static u32 atc_dp_enable_bit(unsigned int k)
{
	return BIT(7 * k - 6);
}

/* The power manager's source code for a DP link rate */
static int atc_dp_source(u8 rate)
{
	switch (rate) {
	case 0x06: return 10;	/* RBR */
	case 0x0a: return 9;	/* HBR */
	case 0x14: return 8;	/* HBR2 */
	case 0x1e: return 7;	/* HBR3 */
	case 0x81: return 6;	/* UHBR10 */
	case 0x82: return 5;	/* UHBR20 */
	default: return -EINVAL;
	}
}

/*
 * Set switch @k as tunnel clock @k's rate has it: field k (and the UHBR bit
 * for a 128b/132b rate) with its enable bit for a clock with a rate, field 7
 * (parked) with its enable bit clear for one without. The switch is written
 * before the enable, then the status read back for the enable bit.
 */
static int atc_dp_tunnel_switch(struct t6050_dp *dp, unsigned int k, u32 *status)
{
	void __iomem *cfg = dp->banks[39];
	u8 rate = dp->tunnel_rate[k - 1];
	u32 en = atc_dp_enable_bit(k);
	u32 sw = readl(cfg + ATC_CFG_DP_SWITCH) & ~atc_dp_switch_mask(k);
	u32 on = readl(cfg + ATC_CFG_DP_ENABLE) & ~en;
	u32 v;
	int ret;

	if (rate) {
		sw |= k << (3 * k - 3);
		if (rate == 0x81 || rate == 0x82)
			sw |= BIT(11 + k);
		on |= en;
	} else {
		sw |= ATC_DP_SWITCH_PARKED << (3 * k - 3);
	}
	writel(sw, cfg + ATC_CFG_DP_SWITCH);
	writel(on, cfg + ATC_CFG_DP_ENABLE);
	ret = readl_poll_timeout(cfg + ATC_CFG_DP_STATUS, v, !(v & en) == !rate, 10, 10000);
	if (status)
		*status = v;
	return ret;
}

/*
 * Bring tunnel clock @k to its rate: for a clock with one, its gate on, then
 * the power manager's source for the rate (its busy bit waited out), then its
 * switch; for a clock without one, its switch parked, then its gate off.
 */
static int atc_dp_tunnel_apply(struct t6050_dp *dp, unsigned int k, u32 *status)
{
	unsigned int i = k - 1;
	u8 rate = dp->tunnel_rate[i];
	void __iomem *word;
	int ret, src;
	u32 v;

	if (!rate) {
		if (atc_dp_tunnel_switch(dp, k, &v))
			dev_warn(dp->dev, "DP tunnel clock %u: switch status %08x still on\n",
				 k, v);
		/* A failed start or gate suspend can leave a held domain at rate zero. */
		return dp->dp_gates ? atc_domain_set(&dp->dp_gate[i], false) : 0;
	}
	if (dp->dp_gates) {
		ret = atc_domain_set(&dp->dp_gate[i], true);
		if (ret)
			return ret;
	}
	src = atc_dp_source(rate);
	word = dp->dp_clk_src + 4 * i;
	writel((readl(word) & ~ATC_DP_SRC_SELECT) | FIELD_PREP(ATC_DP_SRC_SELECT, src), word);
	ret = readl_poll_timeout(word, v, !(v & ATC_DP_SRC_BUSY), 10, 200000);
	if (ret) {
		dev_err(dp->dev, "DP tunnel clock %u: source %d not taken (%08x)\n", k, src, v);
		return ret;
	}
	ret = atc_dp_tunnel_switch(dp, k, &v);
	if (ret)
		dev_err(dp->dev, "DP tunnel clock %u: switch status %08x not on\n", k, v);
	if (status)
		*status = v;
	return ret;
}

/*
 * Apply every tunnel clock in clock order after one has changed: each clock
 * with a rate keeps it, its source and switch written again as they are,
 * and each clock without one stays parked and gated. A clock is therefore
 * only ever started from off at its own user's rate, and only stopped to off;
 * a clock no display uses never runs. Returns @clock's result.
 */
static int atc_dp_tunnels_apply(struct t6050_dp *dp, unsigned int clock, u32 *status)
{
	int ret = 0;

	for (unsigned int k = 1; k <= ATC_DP_TUNNELS; k++) {
		int err = atc_dp_tunnel_apply(dp, k, k == clock ? status : NULL);

		if (k == clock)
			ret = err;
	}
	return ret;
}

/*
 * Park every tunnel clock (field 7, enable off), once the PHY is in USB4
 * mode: an unparked switch selects the native DP clock for its crossbar pixel
 * clock. No clock has a rate yet.
 */
static void atc_dp_tunnels_park(struct t6050_dp *dp)
{
	atc_dp_tunnels_apply(dp, 0, NULL);
}

/*
 * Stop every tunnel clock, before the PHY leaves USB4 mode. The switches are
 * written only while a clock runs, which it does only in USB4 mode.
 */
static void atc_dp_tunnels_stop(struct t6050_dp *dp)
{
	bool running = memchr_inv(dp->tunnel_rate, 0, sizeof(dp->tunnel_rate));

	memset(dp->tunnel_rate, 0, sizeof(dp->tunnel_rate));
	if (running) {
		atc_dp_tunnels_apply(dp, 0, NULL);
		return;
	}
	/* A failed start or gate suspend can leave a held domain at rate zero. */
	for (unsigned int k = 1; dp->dp_gates && k <= ATC_DP_TUNNELS; k++)
		atc_domain_set(&dp->dp_gate[k - 1], false);
}

#include "atc-t6050-combo.h"

static int t6050_dp_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);

	guard(mutex)(&dp->lock);
	if (mode == PHY_MODE_DP)
		return dp->typec ? (dp->enabled ? 0 : -ENOLINK) : atc_start(dp);
	if (mode == PHY_MODE_INVALID)
		return dp->combo ? atc_link_rate(dp, -1) : atc_stop(dp);
	return -EINVAL;
}

static int t6050_dp_validate(struct phy *phy, enum phy_mode mode, int submode,
			    union phy_configure_opts *opts)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);

	guard(mutex)(&dp->lock);
	if (mode != PHY_MODE_DP)
		return -EINVAL;
	memset(opts, 0, sizeof(*opts));
	opts->dp.lanes = dp->multifunction ? 2 : 4;
	opts->dp.link_rate = 8100;
	opts->dp.ssc = 0;
	return 0;
}

static int t6050_dp_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);
	struct phy_configure_opts_dp *cfg = &opts->dp;
	int rate = -1, ret;

	/* Validate the whole request before altering a running transmitter. */
	if (cfg->set_rate && cfg->link_rate) {
		for (unsigned int i = 0; i < ARRAY_SIZE(link_rates); i++)
			if (cfg->link_rate == link_rates[i])
				rate = i;
		if (rate < 0 || cfg->ssc)
			return -EINVAL;
	}
	if ((cfg->set_lanes || cfg->set_voltages) &&
	    cfg->lanes != 0 && cfg->lanes != 1 && cfg->lanes != 2 && cfg->lanes != 4)
		return -EINVAL;
	if (cfg->set_voltages)
		for (unsigned int i = 0; i < cfg->lanes; i++)
			if (cfg->voltage[i] > 3 || cfg->pre[i] > 3 ||
			    cfg->voltage[i] + cfg->pre[i] > 3)
				return -EINVAL;
	if (cfg->set_voltages && cfg->set_rate && rate < 0 && cfg->lanes)
		return -EINVAL;
	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (!dp->enabled)
		return -EPIPE;
	if ((cfg->set_lanes || cfg->set_voltages) && dp->multifunction && cfg->lanes > 2)
		return -EINVAL;
	if (cfg->set_rate) {
		ret = atc_link_rate(dp, rate);
		if (ret)
			return ret;
	}
	/* Native setActiveLaneCount is a no-op; DCP controls the active count. */
	if (cfg->set_voltages) {
		if (dp->rate < 0)
			return -EPIPE;
		for (unsigned int i = 0; i < cfg->lanes; i++) {
			u32 preset = dp->training[dp->rate][4 * cfg->voltage[i] + cfg->pre[i]];

			atc_drive(dp, i, preset);
			dp->voltage[i] = cfg->voltage[i];
			dp->pre[i] = cfg->pre[i];
		}
	}
	return 0;
}

static const struct phy_ops t6050_dp_ops = {
	.set_mode = t6050_dp_set_mode,
	.validate = t6050_dp_validate,
	.configure = t6050_dp_configure,
	.owner = THIS_MODULE,
};

int apple_dp_phy_link_config(struct phy *phy, unsigned int *pclk, bool *uhbr)
{
	struct t6050_dp *dp;

	if (!phy || phy->ops != &t6050_dp_ops)
		return -EOPNOTSUPP;
	dp = phy_get_drvdata(phy);
	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (!dp->enabled || dp->rate < 0)
		return -ENOLINK;
	/* Native direct DPPHY uses logical clock0, mapped to xbar PCLK1.
	 * The other four logical clocks are separately managed tunnel clocks. */
	*pclk = 1;
	*uhbr = false;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dp_phy_link_config);

/*
 * A tunnel clock has one rate while its display uses it, the one its link
 * was set up at, and none otherwise. It starts from off at that rate when the
 * link is set up and stops to off when the link goes; a clock that runs is
 * never given another rate in place. Every change applies all four clocks in
 * order (atc_dp_tunnels_apply()).
 */
int apple_t6050_dp_tunnel_clock(struct phy *phy, unsigned int clock, u8 rate)
{
	struct t6050_dp *dp;
	unsigned int i = clock - 1;
	int src = 0, ret;
	u32 v = 0;

	if (!phy || phy->ops != &t6050_dp_ops)
		return -EOPNOTSUPP;
	if (clock < 1 || clock > ATC_DP_TUNNELS)
		return -EINVAL;
	if (rate) {
		src = atc_dp_source(rate);
		if (src < 0)
			return src;
	}
	dp = phy_get_drvdata(phy);
	guard(mutex)(&dp->lock);
	if (!dp->combo || !dp->dp_clk_src)
		return -EOPNOTSUPP;
	if (dp->tunnel_rate[i] == rate) {
		/* A failed start or gate suspend can leave a held domain at rate zero. */
		if (!rate && dp->dp_gates)
			return atc_domain_set(&dp->dp_gate[i], false);
		return 0;
	}
	/*
	 * Stop a running clock to off: its switch parked and its gate off. One
	 * whose user now asks for another rate is stopped the same way and then
	 * started again from off below.
	 */
	if (dp->tunnel_rate[i]) {
		u8 old = dp->tunnel_rate[i];

		dp->tunnel_rate[i] = 0;
		ret = atc_dp_tunnels_apply(dp, clock, NULL);
		if (!rate) {
			if (!ret)
				dev_info(dp->dev, "DP tunnel clock %u stopped\n", clock);
			return ret;
		}
		if (ret)
			dev_warn(dp->dev, "DP tunnel clock %u: gate not released: %d\n",
				 clock, ret);
		dev_info(dp->dev, "DP tunnel clock %u: link rate 0x%02x to 0x%02x, restarting from off\n",
			 clock, old, rate);
	}
	if (dp->faulted)
		return -EIO;
	if (dp->combo_mode != COMBO_CIO)
		return -ENOLINK;
	dp->tunnel_rate[i] = rate;
	ret = atc_dp_tunnels_apply(dp, clock, &v);
	if (ret) {
		/* Undo the partial start: the clock is left without a rate, for a retry. */
		dp->tunnel_rate[i] = 0;
		atc_dp_tunnel_apply(dp, clock, NULL);
		return ret;
	}
	dev_info(dp->dev, "DP tunnel clock %u: link rate 0x%02x, source %d, switch status %08x\n",
		 clock, rate, src, v);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_t6050_dp_tunnel_clock);

/* The connector sets orientation before changing the alternate-mode mux. */
static int t6050_set_orientation(struct t6050_dp *dp,
				enum typec_orientation orientation)
{
	bool reverse = orientation == TYPEC_ORIENTATION_REVERSE;
	int ret;

	if (dp->combo)
		return combo_orientation(dp, orientation);
	guard(mutex)(&dp->lock);
	if (orientation == TYPEC_ORIENTATION_NONE)
		return atc_stop(dp);
	if (dp->reverse == reverse)
		return 0;
	ret = atc_stop(dp);
	if (!ret)
		dp->reverse = reverse;
	return ret;
}

static int t6050_set_typec_mode(struct t6050_dp *dp, struct typec_mux_state *state)
{
	bool multifunction;
	int ret;

	if (dp->combo)
		return combo_mux(dp, state);
	guard(mutex)(&dp->lock);
	if (state->mode == TYPEC_STATE_SAFE)
		return atc_stop(dp);
	/* USB-only and USB4 require their controller's separate PHY interface. */
	if (!state->alt || state->alt->svid != USB_TYPEC_DP_SID)
		return -EOPNOTSUPP;
	switch (state->mode) {
	case TYPEC_DP_STATE_C:
	case TYPEC_DP_STATE_E:
		multifunction = false;
		break;
	case TYPEC_DP_STATE_D:
		multifunction = true;
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (dp->enabled && dp->multifunction == multifunction)
		return 0;
	ret = atc_stop(dp);
	if (ret)
		return ret;
	dp->multifunction = multifunction;
	ret = atc_start(dp);
	if (!ret)
		dev_info(dp->dev, "Type-C DP: %u lanes, %s orientation\n",
			 multifunction ? 2 : 4, dp->reverse ? "reverse" : "normal");
	return ret;
}

static int t6050_typec_orientation(struct typec_switch_dev *sw,
				 enum typec_orientation orientation)
{
	return t6050_set_orientation(typec_switch_get_drvdata(sw), orientation);
}

static int t6050_typec_mode(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	return t6050_set_typec_mode(typec_mux_get_drvdata(mux), state);
}

static void t6050_typec_switch_release(void *data)
{
	typec_switch_unregister(data);
}

static void t6050_typec_mux_release(void *data)
{
	typec_mux_unregister(data);
}

static int t6050_typec_register(struct t6050_dp *dp)
{
	struct typec_switch_desc sw_desc = {
		.fwnode = dev_fwnode(dp->dev), .drvdata = dp,
		.set = t6050_typec_orientation,
	};
	struct typec_mux_desc mux_desc = {
		.fwnode = dev_fwnode(dp->dev), .drvdata = dp,
		.set = t6050_typec_mode,
		/* USB 3 starts with the lanes as pin D has them: combo_dp2_join_usb() */
		.dp_joins_usb = dp->combo,
	};
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
	int ret;

	sw = typec_switch_register(dp->dev, &sw_desc);
	if (IS_ERR(sw))
		return PTR_ERR(sw);
	ret = devm_add_action_or_reset(dp->dev, t6050_typec_switch_release, sw);
	if (ret)
		return ret;
	mux = typec_mux_register(dp->dev, &mux_desc);
	if (IS_ERR(mux))
		return PTR_ERR(mux);
	return devm_add_action_or_reset(dp->dev, t6050_typec_mux_release, mux);
}

static int t6050_dp_probe(struct platform_device *pdev)
{
	static const char * const domains[] = {"usb-aon", "common", "wrapper", "usb"};
	struct device *dev = &pdev->dev;
	struct t6050_dp *dp;
	struct phy_provider *provider;
	struct phy *phy;
	struct resource *core;
	void __iomem *base;
	int ndomains, gates, ret;

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;
	dp->dev = dev;
	dp->combo = of_device_is_compatible(dev->of_node, "apple,t6050-combo-phy");
	if (dp->combo) {
		u32 port;
		/* Only ports explicitly handed over by the boot producer. */
		if (of_property_read_u32(dev->of_node, "apple,port-index", &port) ||
		    port > 5 ||
		    !of_property_present(dev->of_node, "apple,guest-port-reserved") ||
		    !of_property_present(dev->of_node, "mode-switch"))
			return -EINVAL;
		dp->port = port;
	}
	dp->typec = of_property_present(dev->of_node, "mode-switch");
	dp->rate = -1;
	mutex_init(&dp->lock);
	platform_set_drvdata(pdev, dp);
	core = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core");
	if (!core || resource_size(core) < (dp->combo ? 0x2c000 : 0x2c400))
		return -EINVAL;
	base = devm_ioremap_resource(dev, core);
	if (IS_ERR(base))
		return PTR_ERR(base);
	for (unsigned int i = 0; i < ARRAY_SIZE(core_banks); i++) {
		u8 bank = core_banks[i].bank;

		dp->banks[bank] = base + core_banks[i].offset;
		dp->resources[bank] = *core;
		dp->resources[bank].start += core_banks[i].offset;
		dp->resources[bank].end = core->end;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(extra_banks); i++) {
		u8 bank = extra_banks[i].bank;
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
								   extra_banks[i].name);

		if (bank == 38 && !dp->typec)
			continue;
		if (!res || resource_size(res) < 4)
			return -EINVAL;
		dp->banks[bank] = devm_ioremap_resource(dev, res);
		if (IS_ERR(dp->banks[bank]))
			return PTR_ERR(dp->banks[bank]);
		dp->resources[bank] = *res;
	}
	/* Check every register step before any domain is enabled. */
	{
		static const struct { const struct atc_op *ops; size_t count; } sequences[] = {
			{ dp_init, ARRAY_SIZE(dp_init) },
			{ aux_on, ARRAY_SIZE(aux_on) },
			{ aux_off, ARRAY_SIZE(aux_off) },
			{ dp_link_off, ARRAY_SIZE(dp_link_off) },
			{ dp_poweroff, ARRAY_SIZE(dp_poweroff) },
		};

		for (unsigned int i = 0; i < ARRAY_SIZE(sequences); i++)
			for (size_t j = 0; j < sequences[i].count; j++) {
				ret = atc_validate_step(dp, &sequences[i].ops[j]);
				if (ret)
					return ret;
			}
		for (unsigned int i = 0; i < ARRAY_SIZE(dp_link_on); i++)
			for (unsigned int rate = 0; rate < ARRAY_SIZE(link_rates); rate++) {
				struct atc_op op = dp_link_on[i].op;

				if (!(dp_link_on[i].rates & BIT(rate)))
					continue;
				op.value = dp_link_on[i].values[rate];
				ret = atc_validate_step(dp, &op);
				if (ret)
					return ret;
			}
	}
	if (dp->typec) {
		static const struct { const struct atc_op *ops; size_t count; } sequences[] = {
			{ dp_reverse_init, ARRAY_SIZE(dp_reverse_init) },
			{ mf_normal_init, ARRAY_SIZE(mf_normal_init) },
			{ mf_reverse_init, ARRAY_SIZE(mf_reverse_init) },
			{ mf_poweroff, ARRAY_SIZE(mf_poweroff) },
			{ dp_group0_off, ARRAY_SIZE(dp_group0_off) },
			{ dp_group1_off, ARRAY_SIZE(dp_group1_off) },
		};
		const struct atc_rate_op *rates[] = { dp_group0_on, dp_group1_on };
		const size_t counts[] = { ARRAY_SIZE(dp_group0_on), ARRAY_SIZE(dp_group1_on) };

		for (unsigned int i = 0; i < ARRAY_SIZE(sequences); i++)
			for (size_t j = 0; j < sequences[i].count; j++) {
				ret = atc_validate_step(dp, &sequences[i].ops[j]);
				if (ret)
					return ret;
			}
		for (unsigned int group = 0; group < ARRAY_SIZE(rates); group++)
			for (size_t i = 0; i < counts[group]; i++)
				for (unsigned int rate = 0; rate < ARRAY_SIZE(link_rates); rate++) {
					struct atc_op op = rates[group][i].op;
					if (!(rates[group][i].rates & BIT(rate)))
						continue;
					op.value = rates[group][i].values[rate];
					ret = atc_validate_step(dp, &op);
					if (ret)
						return ret;
				}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(tunable_names); i++) {
		if (i >= 9 && !dp->typec)
			continue;
		dp->tunables[i] = devm_apple_tunable_parse(dev, dev->of_node,
				 tunable_names[i].name, &dp->resources[tunable_names[i].bank]);
		if (IS_ERR(dp->tunables[i]))
			return dev_err_probe(dev, PTR_ERR(dp->tunables[i]),
					     "missing or invalid %s\n", tunable_names[i].name);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(training_names); i++) {
		const char *name = training_names[i];

		if (!of_property_present(dev->of_node, name))
			name = "apple,dp-training-table";
		if (!of_property_present(dev->of_node, name)) {
			memcpy(dp->training[i], dp_training_default, sizeof(dp_training_default));
			continue;
		}
		if (of_property_count_u32_elems(dev->of_node, name) != 16)
			return -EINVAL;
		ret = of_property_read_u32_array(dev->of_node, name, dp->training[i], 16);
		if (ret)
			return ret;
	}
	/*
	 * Named domains prevent automatic wrapper enable at probe time. A combo
	 * PHY may also name the USB controller's domain, which its USB2 banks
	 * are in (combo_start()).
	 */
	ndomains = of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells");
	/* A combo's tunnel clock gates, "dp0".."dp3", follow its own domains. */
	gates = dp->combo ?
		of_property_match_string(dev->of_node, "power-domain-names", "dp0") : -EINVAL;
	if (gates > 0) {
		if (ndomains != gates + ATC_DP_TUNNELS)
			return -EINVAL;
		ndomains = gates;
	}
	if (ndomains != 3 && !(dp->combo && ndomains == 4))
		return -EINVAL;
	dp->ndomains = ndomains;
	for (unsigned int i = 0; i < dp->ndomains; i++) {
		dp->domain[i].dev = dev_pm_domain_attach_by_name(dev, domains[i]);
		if (IS_ERR_OR_NULL(dp->domain[i].dev))
			return dev_err_probe(dev, dp->domain[i].dev ? PTR_ERR(dp->domain[i].dev) : -ENODEV,
					     "cannot attach %s domain\n", domains[i]);
		ret = devm_add_action_or_reset(dev, atc_domain_release, &dp->domain[i]);
		if (ret)
			return ret;
	}
	for (unsigned int i = 0; gates > 0 && i < ATC_DP_TUNNELS; i++) {
		char name[4];

		snprintf(name, sizeof(name), "dp%u", i);
		dp->dp_gate[i].dev = dev_pm_domain_attach_by_name(dev, name);
		if (IS_ERR_OR_NULL(dp->dp_gate[i].dev)) {
			ret = dp->dp_gate[i].dev ? PTR_ERR(dp->dp_gate[i].dev) : -ENODEV;
			return dev_err_probe(dev, ret, "cannot attach %s domain\n", name);
		}
		ret = devm_add_action_or_reset(dev, atc_domain_release, &dp->dp_gate[i]);
		if (ret)
			return ret;
		dp->dp_gates = true;
	}
	if (dp->combo) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
								   "dp-clock-source");

		if (res) {
			if (resource_size(res) < 4 * ATC_DP_TUNNELS)
				return -EINVAL;
			dp->dp_clk_src = devm_ioremap_resource(dev, res);
			if (IS_ERR(dp->dp_clk_src))
				return PTR_ERR(dp->dp_clk_src);
		}
	}
	if (dp->typec) {
		dp->fw_data = devm_platform_ioremap_resource_byname(pdev, "firmware-data");
		if (IS_ERR(dp->fw_data))
			return PTR_ERR(dp->fw_data);
		dp->rtkit = devm_apple_rtkit_init(dev, dp, NULL, 0, &atc_fw_ops);
		if (IS_ERR(dp->rtkit))
			return dev_err_probe(dev, PTR_ERR(dp->rtkit), "cannot initialize PHY firmware IPC\n");
	}
	if (dp->combo) {
		ret = combo_setup(pdev, dp);
		if (ret)
			return ret;
	}
	phy = devm_phy_create(dev, NULL, &t6050_dp_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, dp);
	dp->combo_phy[2] = phy;
	provider = devm_of_phy_provider_register(dev, dp->combo ? combo_xlate : of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);
	return dp->typec ? t6050_typec_register(dp) : 0;
}

static void t6050_dp_shutdown(struct platform_device *pdev)
{
	struct t6050_dp *dp = platform_get_drvdata(pdev);

	guard(mutex)(&dp->lock);
	if (dp->combo ? combo_stop(dp, false) : atc_stop(dp))
		dev_warn(dp->dev, "PHY shutdown incomplete after an earlier error\n");
}

static int t6050_dp_suspend_late(struct device *dev)
{
	struct t6050_dp *dp = dev_get_drvdata(dev);

	if (!dp->combo || !pm_suspend_via_firmware())
		return 0;
	/* A router sleeping in place still uses the link: its host stops the PHY after it. */
	scoped_guard(mutex, &dp->lock) {
		if (dp->cio_sleep) {
			dp->sleep_stopped = false;
			dp->sleep_mode = dp->combo_mode;
			return 0;
		}
	}
	return combo_suspend_late(dp);
}

static int t6050_dp_resume_noirq(struct device *dev)
{
	struct t6050_dp *dp = dev_get_drvdata(dev);

	if (dp->combo && pm_resume_via_firmware())
		combo_resume_noirq(dp);
	return 0;
}

static int t6050_dp_resume(struct device *dev)
{
	struct t6050_dp *dp = dev_get_drvdata(dev);

	if (dp->combo)
		combo_resume(dp);
	return 0;
}

/*
 * TEST: a USB4 router that sleeps in place (T6050, from a macOS sleep with a dock). macOS
 * stops the port's PHY after the router: the router processor's power-off and the CIO
 * domains first, then the PHY's CIO shutdown with its processor idled, then the wrapper, USB
 * and common domains; after the wake the PHY is started from cold in the same mode before
 * the router's domains and processor. The router's host calls these in its last suspend
 * step and its first resume step, where runtime PM no longer runs: the domains'
 * references stay as they are and the system's own power-down takes the domains.
 */
static struct t6050_dp *t6050_dp_from_dev(struct device *dev)
{
	if (!dev || !dev->driver || strcmp(dev->driver->name, "apple-t6050-dp-phy"))
		return NULL;
	return dev_get_drvdata(dev);
}

int apple_t6050_phy_cio_prepare(struct device *dev)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);

	if (!dp || !dp->combo)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	if (dp->cio_sleep || dp->faulted || dp->combo_mode != COMBO_CIO || !dp->cio_prepared)
		return -EBUSY;
	dp->cio_sleep = true;
	dp->cio_sleep_off = false;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_prepare);

int apple_t6050_phy_cio_off(struct device *dev)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);
	ktime_t start = ktime_get();
	int ret;

	if (!dp)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	if (!dp->cio_sleep || dp->cio_sleep_off || dp->faulted || dp->combo_mode != COMBO_CIO)
		return -EBUSY;
	atc_dp_tunnels_stop(dp);
	/* A host kept for the sleep is out of reset here, its PIPE parked on the dummy producer. */
	if (dp->pipe_active || (dp->controller_active && !dp->cio_usb_parked))
		return -EBUSY;
	dp->cio_sleep_hold = true;
	ret = combo_quiesce(dp);
	if (!ret && dp->usb2_active) {
		ret = apple_t6050_usb2_stop(&dp->usb_platform);
		if (!ret)
			dp->usb2_active = false;
	}
	dp->cio_sleep_hold = false;
	if (ret) {
		dev_err(dp->dev, "CIO mode not stopped for the router's sleep (%d)\n", ret);
		dp->faulted = true;
		return ret < 0 ? ret : -EIO;
	}
	dp->cio_sleep_off = true;
	dp->combo_mode = COMBO_SAFE;
	dev_info(dp->dev, "CIO mode stopped for the router's sleep in %lld ms, domains left to the system\n",
		 ktime_ms_delta(ktime_get(), start));
	return 0;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_off);

int apple_t6050_phy_cio_on(struct device *dev)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);
	int ret;

	if (!dp)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	if (!dp->cio_sleep)
		return -EINVAL;
	if (!dp->cio_sleep_off)
		return 0;
	/* After a real sleep the resume has already forgotten the old state; a stop alone left it. */
	dp->cio_sleep_hold = true;
	ret = combo_start(dp, COMBO_CIO);
	dp->cio_sleep_hold = false;
	if (!ret)
		dp->cio_sleep_off = false;
	return ret;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_on);

int apple_t6050_phy_cio_finish(struct device *dev)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);
	bool off;

	if (!dp)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	if (!dp->cio_sleep)
		return -EINVAL;
	off = dp->cio_sleep_off;
	dp->cio_sleep = dp->cio_sleep_off = false;
	return off ? -EUCLEAN : 0;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_finish);

/*
 * TEST: the router's host found no dock after a sleep in place and retired its tunnels. Until
 * the port's mode is started again, DWC3 is not given the router's USB3 tunnel as its PIPE
 * producer. Called in the host's resume, before the USB host is started again.
 */
int apple_t6050_phy_cio_lost(struct device *dev)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);

	if (!dp || !dp->combo)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	dp->cio_tunnel_lost = true;
	dev_info(dp->dev, "the router's dock is gone after the sleep: USB3 PIPE %s\n",
		 dp->pipe_active && dp->pipe_tunnel ? "ALREADY on the USB4 tunnel" :
		 "not given the USB4 tunnel until the port starts again");
	return dp->pipe_active && dp->pipe_tunnel ? -EBUSY : 0;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_lost);

/*
 * TEST (J714s): macOS keeps the port's USB host powered (ATC2_USB on) through the router's sleep
 * and gates it 28 to 37 ms after the router firmware's state word reads asleep, after the CIO
 * domains, the PHY's CIO shutdown and its wrapper (runs mxt4, mxt5, mxt6, mxt6b, seven sleeps).
 * Linux removed that host before the freezer and parked its PIPE, leaving the USB3 tunnel with
 * no host. Here the host is kept instead: xHCI suspends in place (its ports in U3), its PIPE
 * stays on the tunnel and the controller out of reset until the router's host parks the PIPE
 * on its dummy producer, behind the router firmware's sleep, or, for a refused sleep, with the
 * router awake again, before anything can stop the router under it (run sv6t4: a host on a
 * router's tunnel PIPE when that router's domains went off, then an SError). The controller's
 * domain stays as it is and goes off with the system's sleep. KEEP refuses (-EBUSY) unless the
 * PIPE is on the tunnel now and a router host that sleeps in place is bound; the controller
 * then stops for the sleep as before.
 */
int apple_t6050_phy_cio_usb(struct device *dev, enum apple_t6050_cio_usb step)
{
	struct t6050_dp *dp = t6050_dp_from_dev(dev);
	u32 mux = 0, lock = 0;	/* TEST */
	bool parked;
	int ret;

	if (!dp || !dp->combo)
		return -EOPNOTSUPP;
	guard(mutex)(&dp->lock);
	/* TEST (sv6t11): the PIPE handler, the controller's bank, read while it holds its domain. */
	if (step == APPLE_T6050_CIO_USB_PARK && dp->cio_usb_kept && !dp->cio_usb_parked) {
		mux = readl(dp->banks[44] + 0xc);
		lock = readl(dp->banks[44] + 0x14);
	}
	switch (step) {
	case APPLE_T6050_CIO_USB_BIND:
	case APPLE_T6050_CIO_USB_UNBIND:
		dp->cio_usb_host = step == APPLE_T6050_CIO_USB_BIND;
		return 0;
	case APPLE_T6050_CIO_USB_KEEP:
		dp->cio_usb_kept = dp->cio_usb_parked = false;
		if (!(of_machine_is_compatible("apple,j714s") ||
		      of_machine_is_compatible("apple,j716c")) ||
		    !dp->cio_usb_host || dp->faulted ||
		    dp->combo_mode != COMBO_CIO || !dp->cio_prepared || !dp->controller_active ||
		    !dp->pipe_active || !dp->pipe_tunnel)
			return -EBUSY;
		dp->cio_usb_kept = true;
		dev_info(dp->dev, "T6050 sleep: USB host kept on the USB4 tunnel's PIPE for the router's sleep\n");
		return 0;
	case APPLE_T6050_CIO_USB_PARK:
		if (!dp->cio_usb_kept || dp->cio_usb_parked)
			return 0;
		if (dp->faulted)
			return -EIO;
		/* xHCI is suspended: no DMA, its SuperSpeed port in U3, the PIPE in P3. */
		if (dp->pipe_active && dp->pipe_tunnel) {
			ret = apple_t6050_usb3_pipe_stop(&dp->usb_io, combo_pipe_keep, dp);
			if (ret) {
				combo_pipe_fault(dp, "USB3 tunnel PIPE park for the router's sleep", ret);
				return -EIO;
			}
			dp->pipe_active = dp->pipe_tunnel = false;
		}
		dp->cio_usb_parked = true;
		dev_info(dp->dev, "T6050 sleep: USB3 PIPE parked on its dummy producer, the USB host suspended in place, its controller out of reset\n");
		dev_info(dp->dev, "TEST sv6t11: PIPE handler at the park: mux %#x (%#x) and lock %#x before, mux %#x (%#x) and lock %#x after (0x42 under 0x73: its dummy producer)\n",
			 mux, mux & 0x73, lock, readl(dp->banks[44] + 0xc),
			 readl(dp->banks[44] + 0xc) & 0x73, readl(dp->banks[44] + 0x14));
		return 0;
	case APPLE_T6050_CIO_USB_RESULT:
		if (!dp->cio_usb_kept)
			return -ENOENT;
		parked = dp->cio_usb_parked;
		dp->cio_usb_kept = dp->cio_usb_parked = false;
		/* A PIPE that did not park: its state is not known, the port needs its cable again. */
		if (dp->faulted)
			return -EIO;
		/* A firmware sleep's resume forgot the controller: it lost its power and registers. */
		if (!dp->controller_active)
			return APPLE_T6050_CIO_USB_LOST;
		return parked ? APPLE_T6050_CIO_USB_PARKED : APPLE_T6050_CIO_USB_KEPT;
	}
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(apple_t6050_phy_cio_usb);

static const struct dev_pm_ops t6050_dp_pm_ops = {
	.suspend_late = pm_sleep_ptr(t6050_dp_suspend_late),
	.resume_noirq = pm_sleep_ptr(t6050_dp_resume_noirq),
	.resume = pm_sleep_ptr(t6050_dp_resume),
};

static const struct of_device_id t6050_dp_match[] = {
	{ .compatible = "apple,t6050-dp-phy" },
	{ .compatible = "apple,t6050-combo-phy" },
	{},
};
MODULE_DEVICE_TABLE(of, t6050_dp_match);
static struct platform_driver t6050_dp_driver = {
	.probe = t6050_dp_probe,
	.remove = t6050_dp_shutdown,
	.shutdown = t6050_dp_shutdown,
	.driver = {
		.name = "apple-t6050-dp-phy",
		.of_match_table = t6050_dp_match,
		.pm = &t6050_dp_pm_ops,
	},
};
module_platform_driver(t6050_dp_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple T6050 DisplayPort and Type-C USB combo PHY");
