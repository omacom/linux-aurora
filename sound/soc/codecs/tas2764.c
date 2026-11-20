// SPDX-License-Identifier: GPL-2.0
//
// Driver for the Texas Instruments TAS2764 CODEC
// Copyright (C) 2020 Texas Instruments Inc.

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2764.h"

enum tas2764_devid {
	DEVID_TAS2764  = 0,
	DEVID_SN012776 = 1
};

/*
 * Amps that share one SDZ supply and carry "apple,sdz-group-fail-closed" form
 * a group. An over-temperature or over-current fault on any of them forces the
 * supply off, which puts every amp of the group in hardware shutdown, and no
 * amp of the group enables the supply again until the driver is reloaded. The
 * supply's own enable count cannot do this: one amp's disable leaves a shared
 * line up for the others.
 */
struct tas2764_sdz_group {
	struct list_head node;
	struct device_node *supply;	/* the SDZ-supply node: the group's key */
	refcount_t users;
	bool failed;
};

static LIST_HEAD(tas2764_sdz_groups);
static DEFINE_MUTEX(tas2764_sdz_groups_lock);

struct tas2764_priv {
	struct snd_soc_component *component;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *sdz_gpio;
	struct regulator *sdz_reg;	/* NULL when the SDZ line is a GPIO or not described */
	struct regmap *regmap;
	struct device *dev;
	int irq;
	/* Set while the component holds the fault IRQ */
	bool irq_requested;
	enum tas2764_devid devid;

	int v_sense_slot;
	int i_sense_slot;
	/* TAS2764_CLK_CFG value for a fixed BCLK/FS ratio, or -1 to auto-detect */
	int bclk_ratio_code;

	bool dac_powered;
	bool unmuted;
	struct tas2764_sdz_group *sdz_group;

	struct {
		int tx_mode;
		unsigned int tx_mask;
	} idle_slot_config;

	/* apple,bop-config: the SN012776 brownout prevention block */
	u8 bop_config[TAS2764_BOP_CFG_LEN];
	bool have_bop_config;
	/* apple,amp-dcblocker-config: the SN012776 DC_BLK0 and DC_BLK1 */
	u8 dc_blk[2];
	bool have_dc_blk;
	/* apple,amp-tx-zd-config: the SN012776's SDOUT idle slots, driven to zero */
	u32 tx_zd_mask;
	bool have_tx_zd;

	/* apple,deferred-shutdown: software shutdown waits for the bus clock to stop */
	bool deferred_shutdown;
	/* Muted at a stream's end, until the next one: the bus clock may stop */
	bool stream_stopped;
	struct delayed_work shutdown_work;
	/*
	 * Orders the fault handler's read and judgement of the latches against
	 * an unmute clearing a stale clock fault and stream_stopped
	 */
	struct mutex fault_lock;

	/*
	 * What the fault interrupt reported, for "Amp Fault Status": the
	 * INT_LTCH0 bits it judged a fault or a report since probe, and how
	 * many such interrupts there were.
	 */
	u8 fault_bits;
	u32 fault_events;
};

#include "tas2764-quirks.h"

/* LTCH0 faults that shut the whole SDZ group down: over temperature, over current */
#define TAS2764_INT_LTCH0_GROUP_FAULTS	(BIT(0) | BIT(1))
#define TAS2764_INT_LTCH0_TDM_CLOCK	BIT(2)

/*
 * With apple,deferred-shutdown, how long after a stream's end the muted amp
 * goes to software shutdown.  On the J713 the AOP's speaker bus ran on for
 * up to 3520 frames (73 ms at 48 kHz) after a stream's end; this leaves
 * margin over that.
 */
#define TAS2764_DEFERRED_SHUTDOWN_MS	150

static bool tas2764_group_failed(struct tas2764_priv *tas2764)
{
	return tas2764->sdz_group && READ_ONCE(tas2764->sdz_group->failed);
}

/* Force the shared SDZ supply off for the whole group; idempotent. */
static void tas2764_group_shutdown(struct tas2764_priv *tas2764, unsigned int cause)
{
	struct tas2764_sdz_group *group = tas2764->sdz_group;
	int ret;

	/* Under tas2764_sdz_enable()'s lock: no amp can enable the supply in between */
	mutex_lock(&tas2764_sdz_groups_lock);
	WRITE_ONCE(group->failed, true);
	ret = regulator_force_disable(tas2764->sdz_reg);
	mutex_unlock(&tas2764_sdz_groups_lock);
	dev_crit(tas2764->dev, "SDZ group shut down (cause %#x): %s\n", cause,
		 ret ? "forcing the supply off FAILED" :
		       "every amp of the group is in hardware shutdown");
}

/* Enable the SDZ supply unless the amp's group failed; the check and the enable are one step. */
static int tas2764_sdz_enable(struct tas2764_priv *tas2764)
{
	int ret;

	if (!tas2764->sdz_group)
		return tas2764->sdz_reg ? regulator_enable(tas2764->sdz_reg) : 0;

	mutex_lock(&tas2764_sdz_groups_lock);
	ret = READ_ONCE(tas2764->sdz_group->failed) ? -EIO : regulator_enable(tas2764->sdz_reg);
	mutex_unlock(&tas2764_sdz_groups_lock);
	return ret;
}

static void tas2764_group_leave(void *data)
{
	struct tas2764_sdz_group *group = data;

	mutex_lock(&tas2764_sdz_groups_lock);
	if (refcount_dec_and_test(&group->users)) {
		list_del(&group->node);
		of_node_put(group->supply);
		kfree(group);
	}
	mutex_unlock(&tas2764_sdz_groups_lock);
}

static int tas2764_group_join(struct device *dev, struct tas2764_priv *tas2764)
{
	struct tas2764_sdz_group *group;
	struct device_node *supply;

	if (!of_property_read_bool(dev->of_node, "apple,sdz-group-fail-closed"))
		return 0;
	/* A group acts on the shared supply: an amp's own shutdown GPIO would bypass it */
	if (tas2764->sdz_gpio)
		return dev_err_probe(dev, -EINVAL, "SDZ group with a shutdown GPIO of its own\n");
	/* Without its fault interrupt the amp could never take the group down */
	if (!tas2764->irq)
		return dev_err_probe(dev, -EINVAL, "SDZ group without a fault interrupt\n");
	supply = of_parse_phandle(dev->of_node, "SDZ-supply", 0);
	if (!supply || !tas2764->sdz_reg) {
		of_node_put(supply);
		return dev_err_probe(dev, -EINVAL, "SDZ group without an SDZ supply\n");
	}

	mutex_lock(&tas2764_sdz_groups_lock);
	list_for_each_entry(group, &tas2764_sdz_groups, node) {
		if (group->supply == supply) {
			refcount_inc(&group->users);
			of_node_put(supply);
			goto found;
		}
	}
	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group) {
		mutex_unlock(&tas2764_sdz_groups_lock);
		of_node_put(supply);
		return -ENOMEM;
	}
	group->supply = supply;
	refcount_set(&group->users, 1);
	list_add(&group->node, &tas2764_sdz_groups);
found:
	mutex_unlock(&tas2764_sdz_groups_lock);
	tas2764->sdz_group = group;
	return devm_add_action_or_reset(dev, tas2764_group_leave, group);
}

static const char *tas2764_int_ltch0_msgs[8] = {
	"fault: over temperature", /* INT_LTCH0 & BIT(0) */
	"fault: over current",
	"fault: bad TDM clock",
	"limiter active",
	"fault: PVDD below limiter inflection point",
	"fault: limiter max attenuation",
	"fault: BOP infinite hold",
	"fault: BOP mute", /* INT_LTCH0 & BIT(7) */
};

static const unsigned int tas2764_int_readout_regs[6] = {
	TAS2764_INT_LTCH0,
	TAS2764_INT_LTCH1,
	TAS2764_INT_LTCH1_0,
	TAS2764_INT_LTCH2,
	TAS2764_INT_LTCH3,
	TAS2764_INT_LTCH4,
};

static irqreturn_t tas2764_irq_locked(struct tas2764_priv *tas2764)
{
	u8 latched[6] = {0, 0, 0, 0, 0, 0};
	unsigned int val;
	int ret = IRQ_NONE;
	int i;

	/* A failed read is no fault report: an errno would read as every fault bit. */
	for (i = 0; i < ARRAY_SIZE(latched); i++) {
		if (regmap_read(tas2764->regmap, tas2764_int_readout_regs[i], &val)) {
			dev_err_ratelimited(tas2764->dev, "interrupt latch %d unreadable\n", i);
			return IRQ_NONE;
		}
		latched[i] = val;
	}

	/*
	 * An amp muted at a stream's end with its shutdown deferred, or shut
	 * down since, sees the bus clock stop: that is expected, not a fault.
	 */
	if (latched[0] == TAS2764_INT_LTCH0_TDM_CLOCK && READ_ONCE(tas2764->stream_stopped)) {
		snd_soc_component_update_bits(tas2764->component, TAS2764_INT_CLK_CFG,
					      TAS2764_INT_CLK_CFG_IRQZ_CLR,
					      TAS2764_INT_CLK_CFG_IRQZ_CLR);
		return IRQ_HANDLED;
	}

	for (i = 0; i < 8; i++) {
		if (latched[0] & BIT(i)) {
			dev_crit_ratelimited(tas2764->dev, "%s\n",
					     tas2764_int_ltch0_msgs[i]);
			ret = IRQ_HANDLED;
		}
	}

	if ((latched[0] & TAS2764_INT_LTCH0_GROUP_FAULTS) && tas2764->sdz_group)
		tas2764_group_shutdown(tas2764, latched[0]);

	if (latched[0]) {
		WRITE_ONCE(tas2764->fault_bits, tas2764->fault_bits | latched[0]);
		if (tas2764->fault_events != U32_MAX)
			WRITE_ONCE(tas2764->fault_events, tas2764->fault_events + 1);
		snd_soc_component_notify_control(tas2764->component, "Amp Fault Status");
	}

	if (latched[0]) {
		dev_err_ratelimited(tas2764->dev, "other context to the fault: %02x,%02x,%02x,%02x,%02x",
				    latched[1], latched[2], latched[3], latched[4], latched[5]);
		snd_soc_component_update_bits(tas2764->component,
					      TAS2764_INT_CLK_CFG,
					      TAS2764_INT_CLK_CFG_IRQZ_CLR,
					      TAS2764_INT_CLK_CFG_IRQZ_CLR);
	}

	return ret;
}

static irqreturn_t tas2764_irq(int irq, void *data)
{
	struct tas2764_priv *tas2764 = data;
	irqreturn_t ret;

	/* A stream's unmute does not come between reading the latches and judging them */
	mutex_lock(&tas2764->fault_lock);
	ret = tas2764_irq_locked(tas2764);
	mutex_unlock(&tas2764->fault_lock);

	return ret;
}

static void tas2764_reset(struct tas2764_priv *tas2764)
{
	if (tas2764->reset_gpio) {
		gpiod_set_value_cansleep(tas2764->reset_gpio, 0);
		msleep(20);
		gpiod_set_value_cansleep(tas2764->reset_gpio, 1);
		usleep_range(1000, 2000);
	}

	snd_soc_component_write(tas2764->component, TAS2764_SW_RST,
				TAS2764_RST);
	usleep_range(1000, 2000);
}

static int tas2764_update_pwr_ctrl(struct tas2764_priv *tas2764)
{
	struct snd_soc_component *component = tas2764->component;
	unsigned int val;
	int ret;

	if (tas2764_group_failed(tas2764))
		return -EIO;

	if (tas2764->dac_powered)
		val = tas2764->unmuted ?
			TAS2764_PWR_CTRL_ACTIVE : TAS2764_PWR_CTRL_MUTE;
	else
		val = TAS2764_PWR_CTRL_SHUTDOWN;

	if (ENABLED_APPLE_QUIRKS & TAS2764_SHUTDOWN_DANCE)
		return tas2764_do_quirky_pwr_ctrl_change(tas2764, val);

	ret = snd_soc_component_update_bits(component, TAS2764_PWR_CTRL,
					    TAS2764_PWR_CTRL_MASK, val);
	if (ret < 0)
		return ret;

	return 0;
}

static void tas2764_flush_shutdown(struct tas2764_priv *tas2764);
static int tas2764_apply_init_quirks(struct tas2764_priv *tas2764);

#ifdef CONFIG_PM
/*
 * Members of an SDZ group keep the shared SDZ line high across system sleep: the amp waits in
 * software shutdown, as it does between streams, and keeps its registers. Pulling SDZ low here
 * meant a power-on reset at resume, and soc_resume_deferred() unmutes the amps of a running stream
 * a few milliseconds later: their I/V sense then read a negative average power for several
 * seconds, and the uncached page-0xfd quirk writes were lost. After cold boot no amp is unmuted
 * that fresh out of reset.
 *
 * Amps outside a group keep the earlier behaviour (SDZ low across sleep) until the change has run
 * on the machines that use them.
 */
static int tas2764_codec_suspend(struct snd_soc_component *component)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	int ret;

	/*
	 * Until resume the handler can neither read nor clear the latches: the
	 * regmap goes cache-only below, and the I2C controller suspends after
	 * the card. A fault latched meanwhile, such as the clock fault when the
	 * bus clock stops, would hold the shared level-low line asserted and
	 * run every amp's handler in a loop. Keep the line masked until the
	 * cache is live again; disable_irq() nests across the amps sharing it.
	 * ASoC resumes every suspended component, even after a failed suspend.
	 */
	if (tas2764->irq_requested)
		disable_irq(tas2764->irq);

	tas2764_flush_shutdown(tas2764);

	ret = snd_soc_component_update_bits(component, TAS2764_PWR_CTRL,
					    TAS2764_PWR_CTRL_MASK,
					    TAS2764_PWR_CTRL_SHUTDOWN);

	if (ret < 0)
		return ret;

	if (!tas2764->sdz_group) {
		if (tas2764->sdz_gpio)
			gpiod_set_value_cansleep(tas2764->sdz_gpio, 0);

		if (tas2764->sdz_reg)
			regulator_disable(tas2764->sdz_reg);
	}

	regcache_cache_only(tas2764->regmap, true);
	regcache_mark_dirty(tas2764->regmap);

	usleep_range(6000, 7000);

	return 0;
}

/*
 * Whether the amp came back from a power-on reset, read from the amp itself:
 * PWR_CTRL resets to 0x1a with BOP_SRC clear, and the probe sets BOP_SRC on
 * every SN012776. A failed read counts as lost too. Bypasses the cache, which
 * still holds the probe's value.
 */
static bool tas2764_lost_state(struct tas2764_priv *tas2764)
{
	unsigned int val;
	int ret;

	if (tas2764->devid != DEVID_SN012776)
		return false;
	regcache_cache_bypass(tas2764->regmap, true);
	ret = regmap_read(tas2764->regmap, TAS2764_PWR_CTRL, &val);
	regcache_cache_bypass(tas2764->regmap, false);
	return ret || !(val & TAS2764_PWR_CTRL_BOP_SRC);
}

/*
 * An amp back from a power-on reset that could not be restored keeps its
 * reset defaults (gain, brownout, interrupt masks, no quirk page), while the
 * cache holds the restored values. ASoC ignores the resume's error, so a
 * later unmute would set it active as it is. Fail the group closed, as for a
 * latched fault: its supply goes off and every stream is refused.
 */
static void tas2764_restore_failed(struct tas2764_priv *tas2764)
{
	if (!tas2764_group_failed(tas2764))
		tas2764_group_shutdown(tas2764, 0);
}

static int tas2764_codec_resume(struct snd_soc_component *component)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	/*
	 * A firmware-driven (deep) sleep powers the SoC's pin controller off,
	 * so the group's SDZ line drops while asleep and every amp comes back
	 * from a power-on reset when the pin controller restores the line
	 * (its resume_noirq, before this). s2idle keeps the line and the
	 * amps' registers and stays on the path below.
	 */
	bool power_on = tas2764->sdz_group && pm_resume_via_firmware();
	int ret;

	if (!tas2764->sdz_group) {
		ret = tas2764->sdz_reg ? regulator_enable(tas2764->sdz_reg) : 0;
		if (ret) {
			dev_err(tas2764->dev, "Failed to enable regulator\n");
			goto out_irq;
		}

		if (tas2764->sdz_gpio)
			gpiod_set_value_cansleep(tas2764->sdz_gpio, 1);

		usleep_range(1000, 2000);
	}

	regcache_cache_only(tas2764->regmap, false);

	/*
	 * Any other sleep that cut the line or the supply (an s2idle wake
	 * after a deep sleep that failed half way) leaves the same reset
	 * state without the firmware saying so; a stream started on it
	 * latches an over-current on a woofer amp. Check the amp itself.
	 */
	if (tas2764->sdz_group && !power_on && tas2764_lost_state(tas2764)) {
		dev_warn(tas2764->dev, "registers lost in a sleep: restoring as from power-on\n");
		power_on = true;
	}

	if (power_on) {
		/* SDZ high to the first I2C access, as at probe */
		usleep_range(1000, 2000);
		/*
		 * Start from the reset state the cache sync assumes. The page
		 * register is back at 0 while the cache may hold another page:
		 * drop it, so the next page switch is read from the amp.
		 */
		ret = regmap_write(tas2764->regmap, TAS2764_SW_RST, TAS2764_RST);
		if (ret) {
			dev_err(tas2764->dev,
				"no answer on I2C after a deep sleep (%d): SDZ or supply not back\n",
				ret);
			tas2764_restore_failed(tas2764);
			goto out_irq;
		}
		usleep_range(1000, 2000);
		regcache_drop_region(tas2764->regmap, TAS2764_PAGE, TAS2764_PAGE);
	}

	ret = regcache_sync(tas2764->regmap);
	/*
	 * An amp whose SDZ went low came back from a power-on reset, and the
	 * cache sync restores only the cached registers: the init quirks
	 * also write TI's volatile quirk page, which the reset cleared and
	 * nothing wrote again. Without them a J316 woofer amp latched an
	 * over-current at the first stream after a resume. Apply them again,
	 * as the probe does.
	 */
	if (ret >= 0 && (!tas2764->sdz_group || power_on) &&
	    tas2764->devid == DEVID_SN012776)
		ret = tas2764_apply_init_quirks(tas2764);
	if (ret < 0 && power_on) {
		dev_err(tas2764->dev, "restore after a deep sleep failed: %d\n", ret);
		tas2764_restore_failed(tas2764);
	}
	if (ret >= 0)
		ret = tas2764_update_pwr_ctrl(tas2764);

out_irq:
	/* Balance the suspend; a fault latched in sleep is handled now. */
	if (tas2764->irq_requested)
		enable_irq(tas2764->irq);
	return ret;
}
#else
#define tas2764_codec_suspend NULL
#define tas2764_codec_resume NULL
#endif

static const char * const tas2764_ASI1_src[] = {
	"I2C offset", "Left", "Right", "LeftRightDiv2",
};

static SOC_ENUM_SINGLE_DECL(
	tas2764_ASI1_src_enum, TAS2764_TDM_CFG2, TAS2764_TDM_CFG2_SCFG_SHIFT,
	tas2764_ASI1_src);

static const struct snd_kcontrol_new tas2764_asi1_mux =
	SOC_DAPM_ENUM("ASI1 Source", tas2764_ASI1_src_enum);

static const struct snd_kcontrol_new isense_switch =
	SOC_DAPM_SINGLE("Switch", TAS2764_PWR_CTRL, TAS2764_ISENSE_POWER_EN, 1, 1);
static const struct snd_kcontrol_new vsense_switch =
	SOC_DAPM_SINGLE("Switch", TAS2764_PWR_CTRL, TAS2764_VSENSE_POWER_EN, 1, 1);

static const struct snd_soc_dapm_widget tas2764_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("ASI1 Sel", SND_SOC_NOPM, 0, 0, &tas2764_asi1_mux),
	SND_SOC_DAPM_SWITCH("ISENSE", TAS2764_PWR_CTRL, TAS2764_ISENSE_POWER_EN,
			    1, &isense_switch),
	SND_SOC_DAPM_SWITCH("VSENSE", TAS2764_PWR_CTRL, TAS2764_VSENSE_POWER_EN,
			    1, &vsense_switch),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON"),
};

static const struct snd_soc_dapm_route tas2764_audio_map[] = {
	{"ASI1 Sel", "I2C offset", "ASI1"},
	{"ASI1 Sel", "Left", "ASI1"},
	{"ASI1 Sel", "Right", "ASI1"},
	{"ASI1 Sel", "LeftRightDiv2", "ASI1"},
	{"DAC", NULL, "ASI1 Sel"},
	{"OUT", NULL, "DAC"},
	{"ISENSE", "Switch", "IMON"},
	{"VSENSE", "Switch", "VMON"},
};

/* A failed SDZ group refuses every stream until the driver is reloaded */
static int tas2764_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(dai->component);

	if (tas2764_group_failed(tas2764))
		return -EIO;
	return 0;
}

static void tas2764_shutdown_work(struct work_struct *work)
{
	struct tas2764_priv *tas2764 = container_of(to_delayed_work(work),
						    struct tas2764_priv, shutdown_work);
	int ret;

	tas2764->dac_powered = false;
	ret = tas2764_update_pwr_ctrl(tas2764);
	/* a failed group has its SDZ supply forced off already */
	if (ret && !tas2764_group_failed(tas2764))
		dev_err(tas2764->dev, "deferred shutdown failed: %d\n", ret);
}

/* A deferred shutdown happens now (suspend, removal) */
static void tas2764_flush_shutdown(struct tas2764_priv *tas2764)
{
	if (cancel_delayed_work_sync(&tas2764->shutdown_work))
		tas2764_shutdown_work(&tas2764->shutdown_work.work);
}

static int tas2764_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct tas2764_priv *tas2764 =
			snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (!mute) {
		if (tas2764->deferred_shutdown) {
			unsigned int latched;

			/* still muted from the last stream: no shutdown in between */
			cancel_delayed_work_sync(&tas2764->shutdown_work);

			/*
			 * A clock fault the amp latched while the bus was stopped is
			 * not this stream's: the handler, which can run several
			 * milliseconds behind on the shared line, must not see it
			 * after stream_stopped clears.
			 */
			mutex_lock(&tas2764->fault_lock);
			if (tas2764->irq &&
			    !regmap_read(tas2764->regmap, TAS2764_INT_LTCH0, &latched) &&
			    latched == TAS2764_INT_LTCH0_TDM_CLOCK) {
				snd_soc_component_update_bits(tas2764->component,
							      TAS2764_INT_CLK_CFG,
							      TAS2764_INT_CLK_CFG_IRQZ_CLR,
							      TAS2764_INT_CLK_CFG_IRQZ_CLR);
			}
			WRITE_ONCE(tas2764->stream_stopped, false);
			mutex_unlock(&tas2764->fault_lock);
		}
		tas2764->dac_powered = true;
		ret = tas2764_update_pwr_ctrl(tas2764);
		if (ret)
			return ret;
	}

	tas2764->unmuted = !mute;
	ret = tas2764_update_pwr_ctrl(tas2764);
	if (ret)
		return ret;

	if (mute) {
		/* Wait for ramp-down */
		usleep_range(6000, 7000);

		if (tas2764->deferred_shutdown) {
			/* the bus still runs: shut down once its clock has stopped */
			WRITE_ONCE(tas2764->stream_stopped, true);
			schedule_delayed_work(&tas2764->shutdown_work,
					      msecs_to_jiffies(TAS2764_DEFERRED_SHUTDOWN_MS));
			return 0;
		}

		tas2764->dac_powered = false;
		ret = tas2764_update_pwr_ctrl(tas2764);
		if (ret)
			return ret;

		/* Wait a bit after shutdown */
		usleep_range(2000, 3000);
	}

	return 0;
}

static int tas2764_set_bitwidth(struct tas2764_priv *tas2764, int bitwidth)
{
	struct snd_soc_component *component = tas2764->component;
	int val;
	int ret;

	switch (bitwidth) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ret = snd_soc_component_update_bits(component,
						    TAS2764_TDM_CFG2,
						    TAS2764_TDM_CFG2_RXW_MASK,
						    TAS2764_TDM_CFG2_RXW_16BITS);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ret = snd_soc_component_update_bits(component,
						    TAS2764_TDM_CFG2,
						    TAS2764_TDM_CFG2_RXW_MASK,
						    TAS2764_TDM_CFG2_RXW_24BITS);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ret = snd_soc_component_update_bits(component,
						    TAS2764_TDM_CFG2,
						    TAS2764_TDM_CFG2_RXW_MASK,
						    TAS2764_TDM_CFG2_RXW_32BITS);
		break;

	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	val = snd_soc_component_read(tas2764->component, TAS2764_PWR_CTRL);
	if (val < 0)
		return val;

	return 0;
}

static int tas2764_set_samplerate(struct tas2764_priv *tas2764, int samplerate)
{
	struct snd_soc_component *component = tas2764->component;
	int ramp_rate_val;
	int ret;

	switch (samplerate) {
	case 48000:
		ramp_rate_val = TAS2764_TDM_CFG0_SMP_48KHZ |
				TAS2764_TDM_CFG0_44_1_48KHZ;
		break;
	case 44100:
		ramp_rate_val = TAS2764_TDM_CFG0_SMP_44_1KHZ |
				TAS2764_TDM_CFG0_44_1_48KHZ;
		break;
	case 96000:
		ramp_rate_val = TAS2764_TDM_CFG0_SMP_48KHZ |
				TAS2764_TDM_CFG0_88_2_96KHZ;
		break;
	case 88200:
		ramp_rate_val = TAS2764_TDM_CFG0_SMP_44_1KHZ |
				TAS2764_TDM_CFG0_88_2_96KHZ;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG0,
					    TAS2764_TDM_CFG0_SMP_MASK |
					    TAS2764_TDM_CFG0_MASK,
					    ramp_rate_val);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas2764_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	int ret;

	ret = tas2764_set_bitwidth(tas2764, params_format(params));
	if (ret < 0)
		return ret;

	return tas2764_set_samplerate(tas2764, params_rate(params));
}

static int tas2764_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	u8 tdm_rx_start_slot = 0, asi_cfg_0 = 0, asi_cfg_1 = 0, asi_cfg_4 = 0;
	int ret;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_IF:
		asi_cfg_0 ^= TAS2764_TDM_CFG0_FRAME_START;
		fallthrough;
	case SND_SOC_DAIFMT_NB_NF:
		asi_cfg_1 = TAS2764_TDM_CFG1_RX_RISING;
		asi_cfg_4 = TAS2764_TDM_CFG4_TX_FALLING;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		asi_cfg_0 ^= TAS2764_TDM_CFG0_FRAME_START;
		fallthrough;
	case SND_SOC_DAIFMT_IB_NF:
		asi_cfg_1 = TAS2764_TDM_CFG1_RX_FALLING;
		asi_cfg_4 = TAS2764_TDM_CFG4_TX_RISING;
		break;
	}

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG1,
					    TAS2764_TDM_CFG1_RX_MASK,
					    asi_cfg_1);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG4,
					    TAS2764_TDM_CFG4_TX_MASK,
					    asi_cfg_4);
	if (ret < 0)
		return ret;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		asi_cfg_0 ^= TAS2764_TDM_CFG0_FRAME_START;
		fallthrough;
	case SND_SOC_DAIFMT_DSP_A:
		tdm_rx_start_slot = 1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_LEFT_J:
		tdm_rx_start_slot = 0;
		break;
	default:
		dev_err(tas2764->dev,
			"DAI Format is not found, fmt=0x%x\n", fmt);
		return -EINVAL;
	}

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG0,
					    TAS2764_TDM_CFG0_FRAME_START,
					    asi_cfg_0);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG1,
					    TAS2764_TDM_CFG1_MASK,
					    (tdm_rx_start_slot << TAS2764_TDM_CFG1_51_SHIFT));
	if (ret < 0)
		return ret;

	return 0;
}

static int tas2764_set_dai_tdm_slot(struct snd_soc_dai *dai,
				unsigned int tx_mask,
				unsigned int rx_mask,
				int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	int left_slot, right_slot;
	int slots_cfg;
	int slot_size;
	int ret;

	if (tx_mask == 0 || rx_mask != 0)
		return -EINVAL;

	left_slot = __ffs(tx_mask);
	tx_mask &= ~(1 << left_slot);
	if (tx_mask == 0) {
		right_slot = left_slot;
	} else {
		right_slot = __ffs(tx_mask);
		tx_mask &= ~(1 << right_slot);
	}

	if (tx_mask != 0 || left_slot >= slots || right_slot >= slots)
		return -EINVAL;

	slots_cfg = (right_slot << TAS2764_TDM_CFG3_RXS_SHIFT) | left_slot;

	ret = snd_soc_component_write(component, TAS2764_TDM_CFG3, slots_cfg);
	if (ret)
		return ret;

	switch (slot_width) {
	case 16:
		slot_size = TAS2764_TDM_CFG2_RXS_16BITS;
		break;
	case 24:
		slot_size = TAS2764_TDM_CFG2_RXS_24BITS;
		break;
	case 32:
		slot_size = TAS2764_TDM_CFG2_RXS_32BITS;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_component_update_bits(component, TAS2764_TDM_CFG2,
					    TAS2764_TDM_CFG2_RXS_MASK,
					    slot_size);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas2764_set_ivsense_transmit(struct tas2764_priv *tas2764, int i_slot, int v_slot)
{
	int ret;

	ret = snd_soc_component_update_bits(tas2764->component, TAS2764_TDM_CFG5,
					    TAS2764_TDM_CFG5_VSNS_ENABLE |
					    TAS2764_TDM_CFG5_50_MASK,
					    TAS2764_TDM_CFG5_VSNS_ENABLE |
					    v_slot);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(tas2764->component, TAS2764_TDM_CFG6,
					    TAS2764_TDM_CFG6_ISNS_ENABLE |
					    TAS2764_TDM_CFG6_50_MASK,
					    TAS2764_TDM_CFG6_ISNS_ENABLE |
					    i_slot);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas2764_write_sdout_idle_mask(struct tas2764_priv *tas2764, u32 mask)
{
	struct snd_soc_component *component = tas2764->component;
	int i, ret;

	/* Hardware supports up to 64 slots, but we don't */
	for (i = 0; i < 4; i++) {
		ret = snd_soc_component_write(component,
					      TAS2764_SDOUT_HIZ_1 + i,
					      (mask >> (i * 8)) & 0xff);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int tas2764_set_dai_tdm_idle(struct snd_soc_dai *dai,
				    unsigned int tx_mask, unsigned int rx_mask,
				    int tx_mode, int rx_mode)
{
	struct snd_soc_component *component = dai->component;
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	int ret;

	/* We don't support setting anything on SDIN */
	if (rx_mode)
		return -EOPNOTSUPP;

	/* The idle slots the Mac's ADT gives were set at probe and stay */
	if (tas2764->have_tx_zd)
		return 0;

	if (tas2764->idle_slot_config.tx_mask == tx_mask &&
	    tas2764->idle_slot_config.tx_mode == tx_mode)
		return 0;

	switch (tx_mode) {
	case SND_SOC_DAI_TDM_IDLE_ZERO:
		if (!tx_mask)
			return -EINVAL;

		ret = tas2764_write_sdout_idle_mask(tas2764, tx_mask);
		if (ret < 0)
			return ret;

		ret = snd_soc_component_update_bits(component,
						    TAS2764_SDOUT_HIZ_9,
						    TAS2764_SDOUT_HIZ_9_FORCE_0_EN,
						    TAS2764_SDOUT_HIZ_9_FORCE_0_EN);
		if (ret < 0)
			return ret;

		tas2764->idle_slot_config.tx_mask = tx_mask;
		tas2764->idle_slot_config.tx_mode = tx_mode;
		break;
	case SND_SOC_DAI_TDM_IDLE_HIZ:
	case SND_SOC_DAI_TDM_IDLE_OFF:
		/* HiZ mode does not support a slot mask */
		ret = tas2764_write_sdout_idle_mask(tas2764, 0);
		if (ret < 0)
			return ret;

		ret = snd_soc_component_update_bits(component,
						    TAS2764_SDOUT_HIZ_9,
						    TAS2764_SDOUT_HIZ_9_FORCE_0_EN, 0);
		if (ret < 0)
			return ret;

		tas2764->idle_slot_config.tx_mask = 0;
		tas2764->idle_slot_config.tx_mode = tx_mode;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* The SDOUT idle slot mask must be cropped based on the BCLK ratio */
static int tas2764_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(dai->component);

	if (!tas2764->idle_slot_config.tx_mask)
		return 0;

	tas2764->idle_slot_config.tx_mask &= GENMASK((ratio / 8) - 1, 0);

	return tas2764_write_sdout_idle_mask(tas2764, tas2764->idle_slot_config.tx_mask);
}

static const struct snd_soc_dai_ops tas2764_dai_ops = {
	.startup = tas2764_startup,
	.mute_stream = tas2764_mute,
	.hw_params  = tas2764_hw_params,
	.set_fmt    = tas2764_set_fmt,
	.set_bclk_ratio = tas2764_set_bclk_ratio,
	.set_tdm_slot = tas2764_set_dai_tdm_slot,
	.set_tdm_idle = tas2764_set_dai_tdm_idle,
	.no_capture_mute = 1,
};

#define TAS2764_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2764_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
		       SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_88200)

static struct snd_soc_dai_driver tas2764_dai_driver[] = {
	{
		.name = "tas2764 ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 1,
			.channels_max   = 2,
			.rates      = TAS2764_RATES,
			.formats    = TAS2764_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 0,
			.channels_max   = 2,
			.rates = TAS2764_RATES,
			.formats = TAS2764_FORMATS,
		},
		.ops = &tas2764_dai_ops,
		.symmetric_rate = 1,
	},
};

static const struct regmap_config tas2764_i2c_regmap;

static int tas2764_apply_init_quirks(struct tas2764_priv *tas2764)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(tas2764_quirk_init_sequences); i++) {
		const struct tas2764_quirk_init_sequence *init_seq =
						&tas2764_quirk_init_sequences[i];

		if (!init_seq->seq)
			continue;

		if (!(BIT(i) & ENABLED_APPLE_QUIRKS))
			continue;

		ret = regmap_multi_reg_write(tas2764->regmap, init_seq->seq,
					     init_seq->len);

		if (ret < 0)
			return ret;
	}

	return 0;
}

static int tas2764_read_die_temp(struct tas2764_priv *tas2764, long *result)
{
	int ret, reg;

	ret = regmap_read(tas2764->regmap, TAS2764_TEMP, &reg);
	if (ret)
		return ret;
	/*
	 * As per datasheet, subtract 93 from raw value to get degrees
	 * Celsius. hwmon wants millidegrees.
	 *
	 * NOTE: The TAS2764 datasheet mentions initialising TAS2764_TEMP
	 * such that the temperature is 2.6 *C, however the register
	 * is actually initialised to 0. The ADC is also powered down during
	 * software shutdown. The last sampled temperature will persist
	 * in the register while the amp is in this power state.
	 */
	if (reg == 0)
		return -ENODATA;

	*result = (reg - 93) * 1000;
	return 0;
}

static int tas2764_hwmon_is_fault(struct tas2764_priv *tas2764, long *result)
{
	int ret;
	long temp;

	ret = tas2764_read_die_temp(tas2764, &temp);
	if (ret == -ENODATA) {
		*result = true;
		return 0;
	}

	return ret;
}

static umode_t tas2764_hwmon_is_visible(const void *data,
					enum hwmon_sensor_types type, u32 attr,
					int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_fault:
		return 0444;
	default:
		break;
	}

	return 0;
}

static int tas2764_hwmon_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct tas2764_priv *tas2764 = dev_get_drvdata(dev);
	int ret;

	switch (attr) {
	case hwmon_temp_input:
		ret = tas2764_read_die_temp(tas2764, val);
		break;
	case hwmon_temp_fault:
		ret = tas2764_hwmon_is_fault(tas2764, val);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static const struct hwmon_channel_info *const tas2764_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_FAULT),
	NULL
};

static const struct hwmon_ops tas2764_hwmon_ops = {
	.is_visible	= tas2764_hwmon_is_visible,
	.read		= tas2764_hwmon_read,
};

static const struct hwmon_chip_info tas2764_hwmon_chip_info = {
	.ops	= &tas2764_hwmon_ops,
	.info	= tas2764_hwmon_info,
};

/*
 * Program the fixed BCLK/FS ratio from ti,bclk-ratio (tas2764_parse_bclk_ratio()).
 * Without one nothing is written: the amplifier auto-detects the ratio.
 */
static int tas2764_apply_bclk_ratio(struct tas2764_priv *tas2764)
{
	int ret;

	if (tas2764->bclk_ratio_code < 0)
		return 0;

	ret = regmap_write(tas2764->regmap, TAS2764_CLK_CFG, tas2764->bclk_ratio_code);
	if (ret < 0)
		return ret;

	return regmap_update_bits(tas2764->regmap, TAS2764_TDM_CFG0,
				  TAS2764_TDM_CFG0_FIXED_RATIO,
				  TAS2764_TDM_CFG0_FIXED_RATIO);
}

static int tas2764_codec_probe(struct snd_soc_component *component)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);
	int ret, i;

	tas2764->component = component;

	ret = tas2764_sdz_enable(tas2764);
	if (ret != 0) {
		dev_err(tas2764->dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	if (tas2764->sdz_gpio) {
		gpiod_set_value_cansleep(tas2764->sdz_gpio, 1);
	}

	usleep_range(1000, 2000);

	tas2764_reset(tas2764);
	regmap_reinit_cache(tas2764->regmap, &tas2764_i2c_regmap);

	/* The reset cleared the idle slot setup: make the next set_tdm_idle() write it */
	tas2764->idle_slot_config.tx_mask = 0;
	tas2764->idle_slot_config.tx_mode = SND_SOC_DAI_TDM_IDLE_NONE;

	if (tas2764->irq) {
		ret = snd_soc_component_write(tas2764->component, TAS2764_INT_MASK0, 0x00);
		if (ret < 0)
			goto err_sdz;

		ret = snd_soc_component_write(tas2764->component, TAS2764_INT_MASK1, 0xff);
		if (ret < 0)
			goto err_sdz;

		ret = snd_soc_component_write(tas2764->component, TAS2764_INT_MASK2, 0xff);
		if (ret < 0)
			goto err_sdz;

		ret = snd_soc_component_write(tas2764->component, TAS2764_INT_MASK3, 0xff);
		if (ret < 0)
			goto err_sdz;

		ret = snd_soc_component_write(tas2764->component, TAS2764_INT_MASK4, 0xff);
		if (ret < 0)
			goto err_sdz;
	}

	if (tas2764->i_sense_slot != -1 && tas2764->v_sense_slot != -1) {
		ret = tas2764_set_ivsense_transmit(tas2764, tas2764->i_sense_slot,
						   tas2764->v_sense_slot);

		if (ret < 0)
			goto err_sdz;
	}

	switch (tas2764->devid) {
	case DEVID_SN012776:
		/*
		 * PVDD is the brownout source on every SN012776, and the
		 * brownout prevention block is the one the Mac's boot ADT
		 * gives, when it gives one.
		 */
		ret = snd_soc_component_update_bits(component, TAS2764_PWR_CTRL,
					TAS2764_PWR_CTRL_BOP_SRC,
					TAS2764_PWR_CTRL_BOP_SRC);
		if (ret < 0)
			goto err_sdz;

		for (i = 0; tas2764->have_bop_config && i < TAS2764_BOP_CFG_LEN; i++) {
			ret = snd_soc_component_write(component,
						TAS2764_BOP_CFG0 + i,
						tas2764->bop_config[i]);
			if (ret < 0)
				goto err_sdz;
		}

		/* The DC blockers (the playback and sense high-pass corners) */
		if (tas2764->have_dc_blk) {
			ret = snd_soc_component_write(component, TAS2764_DC_BLK0,
						      tas2764->dc_blk[0]);
			if (ret < 0)
				goto err_sdz;
			ret = snd_soc_component_write(component, TAS2764_DC_BLK1,
						      tas2764->dc_blk[1]);
			if (ret < 0)
				goto err_sdz;
		}

		/* The SDOUT idle slots that drive zeros, from the ADT */
		if (tas2764->have_tx_zd && tas2764->tx_zd_mask) {
			ret = tas2764_write_sdout_idle_mask(tas2764, tas2764->tx_zd_mask);
			if (ret < 0)
				goto err_sdz;
			ret = snd_soc_component_update_bits(component, TAS2764_SDOUT_HIZ_9,
						TAS2764_SDOUT_HIZ_9_FORCE_0_EN,
						TAS2764_SDOUT_HIZ_9_FORCE_0_EN);
			if (ret < 0)
				goto err_sdz;
			tas2764->idle_slot_config.tx_mask = tas2764->tx_zd_mask;
			tas2764->idle_slot_config.tx_mode = SND_SOC_DAI_TDM_IDLE_ZERO;
		}

		/* Apply all enabled Apple quirks */
		ret = tas2764_apply_init_quirks(tas2764);

		if (ret < 0)
			goto err_sdz;

		break;
	default:
		break;
	}

	ret = tas2764_apply_bclk_ratio(tas2764);
	if (ret < 0)
		goto err_sdz;

	/*
	 * Last, so that no failure leaves the handler behind. The component owns the IRQ and
	 * the remove frees it; not a devres of the I2C device, which outlives the component
	 * and, on an unbind, would release the IRQ before the component is removed.
	 */
	if (tas2764->irq) {
		ret = request_threaded_irq(tas2764->irq, NULL, tas2764_irq,
					   IRQF_ONESHOT | IRQF_SHARED | IRQF_TRIGGER_LOW,
					   "tas2764", tas2764);
		if (ret && tas2764->sdz_group) {
			dev_err_probe(tas2764->dev, ret, "SDZ group needs its fault IRQ\n");
			goto err_sdz;
		}
		if (ret)
			dev_warn(tas2764->dev, "failed to request IRQ: %d\n", ret);
		else
			tas2764->irq_requested = true;
	}

	return 0;

err_sdz:
	/* Undo the power-up: ASoC doesn't call the remove after a failed probe */
	if (tas2764->sdz_gpio)
		gpiod_set_value_cansleep(tas2764->sdz_gpio, 0);
	/* As in the remove: no supply to drop with a GPIO SDZ, none left in a failed group */
	if (tas2764->sdz_reg && !tas2764_group_failed(tas2764))
		regulator_disable(tas2764->sdz_reg);
	return ret;
}

static void tas2764_codec_remove(struct snd_soc_component *component)
{
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);

	if (tas2764->irq_requested) {
		free_irq(tas2764->irq, tas2764);
		tas2764->irq_requested = false;
	}

	tas2764_flush_shutdown(tas2764);

	/* A failed group's supply was forced off: there is no enable left to balance. */
	if (tas2764_group_failed(tas2764))
		return;

	if (tas2764->sdz_reg)
		regulator_disable(tas2764->sdz_reg);
}

static DECLARE_TLV_DB_SCALE(tas2764_digital_tlv, 1100, 50, 0);
static DECLARE_TLV_DB_SCALE(tas2764_playback_volume, -10050, 50, 1);

static const char * const tas2764_hpf_texts[] = {
	"Disabled", "2 Hz", "50 Hz", "100 Hz", "200 Hz",
	"400 Hz", "800 Hz"
};

static SOC_ENUM_SINGLE_DECL(
	tas2764_hpf_enum, TAS2764_DC_BLK0,
	TAS2764_DC_BLK0_HPF_FREQ_PB_SHIFT, tas2764_hpf_texts);

static const char * const tas2764_oce_texts[] = {
	"Disable", "Retry",
};

static SOC_ENUM_SINGLE_DECL(
	tas2764_oce_enum, TAS2764_MISC_CFG1,
	TAS2764_MISC_CFG1_OCE_RETRY_SHIFT, tas2764_oce_texts);

/*
 * "Amp Fault Status", read-only: [0] the INT_LTCH0 bits the fault interrupt
 * reported since probe (bit 0 over temperature, 1 over current, 2 bad TDM
 * clock, 3 limiter active, 4 PVDD below the limiter inflection point, 5
 * limiter at maximum attenuation, 6 BOP infinite hold, 7 BOP mute), [1] the
 * number of those interrupts, saturating, [2] 1 once this amp's SDZ group
 * was forced off. A change is notified.
 */
static int tas2764_fault_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = U32_MAX;
	return 0;
}

static int tas2764_fault_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct tas2764_priv *tas2764 = snd_soc_component_get_drvdata(component);

	uvalue->value.integer.value[0] = READ_ONCE(tas2764->fault_bits);
	uvalue->value.integer.value[1] = READ_ONCE(tas2764->fault_events);
	uvalue->value.integer.value[2] = tas2764_group_failed(tas2764);
	return 0;
}

static const struct snd_kcontrol_new tas2764_snd_controls[] = {
	SOC_SINGLE_TLV("Speaker Volume", TAS2764_DVC, 0,
		       TAS2764_DVC_MAX, 1, tas2764_playback_volume),
	SOC_SINGLE_TLV("Amp Gain Volume", TAS2764_CHNL_0, 1, 0x14, 0,
		       tas2764_digital_tlv),
	SOC_ENUM("HPF Corner Frequency", tas2764_hpf_enum),
	SOC_ENUM("OCE Handling", tas2764_oce_enum),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Amp Fault Status",
		.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = tas2764_fault_info,
		.get = tas2764_fault_get,
	},
};

static const struct snd_soc_component_driver soc_component_driver_tas2764 = {
	.probe			= tas2764_codec_probe,
	.remove			= tas2764_codec_remove,
	.suspend		= tas2764_codec_suspend,
	.resume			= tas2764_codec_resume,
	.controls		= tas2764_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2764_snd_controls),
	.dapm_widgets		= tas2764_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas2764_dapm_widgets),
	.dapm_routes		= tas2764_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas2764_audio_map),
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static const struct reg_default tas2764_reg_defaults[] = {
	{ TAS2764_PAGE, 0x00 },
	{ TAS2764_SW_RST, 0x00 },
	{ TAS2764_PWR_CTRL, 0x1a },
	{ TAS2764_CHNL_0, 0x28 },
	{ TAS2764_TDM_CFG0, 0x09 },
	{ TAS2764_TDM_CFG1, 0x02 },
	{ TAS2764_TDM_CFG2, 0x0a },
	{ TAS2764_TDM_CFG3, 0x10 },
	{ TAS2764_TDM_CFG5, 0x42 },
	{ TAS2764_DVC, 0x00 },
	{ TAS2764_INT_CLK_CFG, 0x19 },
};

static const struct regmap_range_cfg tas2764_regmap_ranges[] = {
	{
		.range_min = 0,
		.range_max = 0xffff,
		.selector_reg = TAS2764_PAGE,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static bool tas2764_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2764_SW_RST:
	case TAS2764_TEMP:
	case TAS2764_INT_LTCH0 ... TAS2764_INT_LTCH4:
	case TAS2764_INT_CLK_CFG:
		return true;
	case TAS2764_REG(0xf0, 0x0) ... TAS2764_REG(0xff, 0x0):
		/* TI's undocumented registers for the application of quirks */
		return true;
	default:
		return false;
	}
}

/*
 * The registers the driver writes: page 0, the page 1 and page 6 registers of
 * the quirk sequences and the SDOUT high-impedance masks, and TI's quirk pages
 * (volatile). A cache sync after a resume writes back only these. Anything
 * else in the cache got there through a read alone, such as a regmap debugfs
 * dump of every page, and some of those registers are reserved: a raw block
 * written over them at resume is never completed by the amp and leaves the
 * I2C controller stuck in its transaction, taking every amp on the bus with
 * it.
 */
static bool tas2764_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2764_REG(0x0, 0x00) ... TAS2764_REG(0x0, 0x7f):
	case TAS2764_REG(0x1, 0x33):
	case TAS2764_REG(0x1, 0x37):
	case TAS2764_SDOUT_HIZ_1 ... TAS2764_SDOUT_HIZ_9:
	case TAS2764_REG(0x1, 0x47):
	case TAS2764_REG(0x6, 0x14) ... TAS2764_REG(0x6, 0x1f):
	case TAS2764_REG(0xf0, 0x0) ... TAS2764_REG(0xff, 0x7f):
		return true;
	default:
		return false;
	}
}

static const struct regmap_config tas2764_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2764_writeable_register,
	.volatile_reg = tas2764_volatile_register,
	.reg_defaults = tas2764_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tas2764_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.ranges = tas2764_regmap_ranges,
	.num_ranges = ARRAY_SIZE(tas2764_regmap_ranges),
	.max_register = 0xffff,
};

/*
 * The SN012776's brownout prevention block (BOP_CFG0 on, TAS2764_BOP_CFG_LEN
 * registers), from the device tree's apple,bop-config: the boot loader copies
 * it from the amp group's configuration in the Mac's ADT, which macOS programs
 * into every amp of the group. Without it the block stays at its reset values.
 * A property of the wrong length is a broken device tree, not a reason to
 * guess.
 */
static int tas2764_parse_bop_config(struct device *dev, struct tas2764_priv *tas2764)
{
	int len;

	tas2764->have_bop_config = false;
	if (tas2764->devid != DEVID_SN012776)
		return 0;

	len = of_property_count_u8_elems(dev->of_node, "apple,bop-config");
	if (len == -EINVAL) {
		dev_dbg(dev, "no brownout configuration in the device tree\n");
		return 0;
	}
	if (len != TAS2764_BOP_CFG_LEN)
		return dev_err_probe(dev, -EINVAL,
				     "apple,bop-config: %d bytes, the block has %d\n",
				     len, TAS2764_BOP_CFG_LEN);

	of_property_read_u8_array(dev->of_node, "apple,bop-config",
				  tas2764->bop_config, TAS2764_BOP_CFG_LEN);
	tas2764->have_bop_config = true;
	return 0;
}

/*
 * The SN012776's DC blockers, DC_BLK0 and DC_BLK1, from the device tree's
 * apple,amp-dcblocker-config: the boot loader copies it from the amp group's
 * configuration in the Mac's ADT, whose first two bytes are the two registers
 * (the rest is padding), the same for every amp of the group. Without it the
 * SN012776 uses DC_BLK0 = 0x01 and DC_BLK1 = 0x00 on every amp.
 */
static int tas2764_parse_dcblocker_config(struct device *dev, struct tas2764_priv *tas2764)
{
	int len, ret;

	tas2764->have_dc_blk = false;
	if (tas2764->devid != DEVID_SN012776)
		return 0;

	len = of_property_count_u8_elems(dev->of_node, "apple,amp-dcblocker-config");
	if (len == -EINVAL) {
		tas2764->dc_blk[0] = 0x01;
		tas2764->dc_blk[1] = 0x00;
		tas2764->have_dc_blk = true;
		return 0;
	}
	if (len < 2)
		return dev_err_probe(dev, -EINVAL,
				     "apple,amp-dcblocker-config: %d bytes, need 2\n", len);

	ret = of_property_read_u8_array(dev->of_node, "apple,amp-dcblocker-config",
					tas2764->dc_blk, 2);
	if (ret)
		return ret;
	tas2764->have_dc_blk = true;
	return 0;
}

/*
 * The SN012776's SDOUT idle slots that drive zeros (SDOUT_HIZ_1..), from the
 * device tree's apple,amp-tx-zd-config: the boot loader copies this amp's row
 * of its group's amp-tx-zd-config in the Mac's ADT, eight bytes, one mask
 * byte per eight slots from slot 0. Like the device tree's TDM idle masks,
 * this driver drives slots 0 to 31 only. With the property, it replaces the
 * idle masks of the sound card's device tree; an all-zero row leaves every
 * idle slot in high impedance.
 */
#define TAS2764_TX_ZD_ROW	8

static int tas2764_parse_tx_zd_config(struct device *dev, struct tas2764_priv *tas2764)
{
	u8 row[TAS2764_TX_ZD_ROW];
	int len, ret, i;

	tas2764->have_tx_zd = false;
	tas2764->tx_zd_mask = 0;
	if (tas2764->devid != DEVID_SN012776)
		return 0;

	len = of_property_count_u8_elems(dev->of_node, "apple,amp-tx-zd-config");
	if (len == -EINVAL)
		return 0;
	if (len != TAS2764_TX_ZD_ROW)
		return dev_err_probe(dev, -EINVAL,
				     "apple,amp-tx-zd-config: %d bytes, the row has %d\n",
				     len, TAS2764_TX_ZD_ROW);

	ret = of_property_read_u8_array(dev->of_node, "apple,amp-tx-zd-config",
					row, TAS2764_TX_ZD_ROW);
	if (ret)
		return ret;
	for (i = 4; i < TAS2764_TX_ZD_ROW; i++) {
		if (row[i])
			dev_warn(dev, "idle slots from %d on are not supported\n", i * 8);
	}
	tas2764->tx_zd_mask = row[0] | row[1] << 8 | row[2] << 16 | (u32)row[3] << 24;
	tas2764->have_tx_zd = true;
	return 0;
}

/*
 * BCLK/FS ratios and the TAS2764_CLK_CFG value that makes the amplifier
 * accept them. Only the J873 speaker frame (250 BCLKs, which the ratio
 * auto-detection rejects with "bad TDM clock") is verified on hardware;
 * other ratios are left to the auto-detection.
 */
static const struct {
	u16 ratio;
	u8 code;
} tas2764_bclk_ratio_codes[] = {
	{ 250, 0x30 },
};

/*
 * ti,bclk-ratio: a fixed BCLK/FS ratio, as its TAS2764_CLK_CFG code in
 * bclk_ratio_code. Without the property bclk_ratio_code stays -1, set before
 * the device tree is read, and the ratio is auto-detected.
 */
static int tas2764_parse_bclk_ratio(struct device *dev, struct tas2764_priv *tas2764)
{
	u32 ratio;
	int i;

	if (device_property_read_u32(dev, "ti,bclk-ratio", &ratio))
		return 0;

	for (i = 0; i < ARRAY_SIZE(tas2764_bclk_ratio_codes); i++)
		if (tas2764_bclk_ratio_codes[i].ratio == ratio)
			tas2764->bclk_ratio_code = tas2764_bclk_ratio_codes[i].code;
	if (tas2764->bclk_ratio_code < 0)
		return dev_err_probe(dev, -EINVAL, "unsupported ti,bclk-ratio %u\n", ratio);

	return 0;
}

static int tas2764_parse_dt(struct device *dev, struct tas2764_priv *tas2764)
{
	int ret = 0;

	/*
	 * The SDZ supply is optional: most boards drive SDZ from a GPIO
	 * (shutdown-gpios, shared between amps through the GPIO core), and only
	 * boards that model the line as a regulator have one. A dummy regulator
	 * in its place did nothing but warn on every boot.
	 */
	tas2764->sdz_reg = devm_regulator_get_optional(dev, "SDZ");
	if (IS_ERR(tas2764->sdz_reg)) {
		if (PTR_ERR(tas2764->sdz_reg) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(tas2764->sdz_reg),
					     "Failed to get SDZ supply\n");
		tas2764->sdz_reg = NULL;
	}

	tas2764->reset_gpio = devm_gpiod_get_optional(tas2764->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(tas2764->reset_gpio)) {
		if (PTR_ERR(tas2764->reset_gpio) == -EPROBE_DEFER) {
			tas2764->reset_gpio = NULL;
			return -EPROBE_DEFER;
		}
	}

	tas2764->sdz_gpio = devm_gpiod_get_optional(dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(tas2764->sdz_gpio)) {
		if (PTR_ERR(tas2764->sdz_gpio) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		tas2764->sdz_gpio = NULL;
	}

	ret = fwnode_property_read_u32(dev->fwnode, "ti,imon-slot-no",
				       &tas2764->i_sense_slot);
	if (ret)
		tas2764->i_sense_slot = -1;

	ret = fwnode_property_read_u32(dev->fwnode, "ti,vmon-slot-no",
				       &tas2764->v_sense_slot);
	if (ret)
		tas2764->v_sense_slot = -1;

	tas2764->deferred_shutdown = of_property_read_bool(dev->of_node,
							   "apple,deferred-shutdown");

	ret = tas2764_parse_bop_config(dev, tas2764);
	if (ret)
		return ret;

	ret = tas2764_parse_dcblocker_config(dev, tas2764);
	if (ret)
		return ret;

	ret = tas2764_parse_tx_zd_config(dev, tas2764);
	if (ret)
		return ret;

	ret = tas2764_parse_bclk_ratio(dev, tas2764);
	if (ret)
		return ret;

	return tas2764_group_join(dev, tas2764);
}

static int tas2764_i2c_probe(struct i2c_client *client)
{
	struct tas2764_priv *tas2764;
	int result;

	tas2764 = devm_kzalloc(&client->dev, sizeof(struct tas2764_priv),
			       GFP_KERNEL);
	if (!tas2764)
		return -ENOMEM;

	tas2764->devid = (kernel_ulong_t)of_device_get_match_data(&client->dev);

	tas2764->dev = &client->dev;
	tas2764->irq = client->irq;
	INIT_DELAYED_WORK(&tas2764->shutdown_work, tas2764_shutdown_work);
	result = devm_mutex_init(&client->dev, &tas2764->fault_lock);
	if (result)
		return result;
	/* Auto-detect the ratio unless the firmware node sets one. */
	tas2764->bclk_ratio_code = -1;
	i2c_set_clientdata(client, tas2764);
	dev_set_drvdata(&client->dev, tas2764);

	tas2764->regmap = devm_regmap_init_i2c(client, &tas2764_i2c_regmap);
	if (IS_ERR(tas2764->regmap)) {
		result = PTR_ERR(tas2764->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
					result);
		return result;
	}

	if (client->dev.of_node) {
		result = tas2764_parse_dt(&client->dev, tas2764);
		if (result)
			return dev_err_probe(tas2764->dev, result,
					     "Failed to parse devicetree\n");
	}

	if (IS_REACHABLE(CONFIG_HWMON)) {
		struct device *hwmon;

		hwmon = devm_hwmon_device_register_with_info(&client->dev, "tas2764",
							tas2764,
							&tas2764_hwmon_chip_info,
							NULL);
		if (IS_ERR(hwmon)) {
			return dev_err_probe(&client->dev, PTR_ERR(hwmon),
					     "Failed to register temp sensor\n");
		}
	}


	return devm_snd_soc_register_component(tas2764->dev,
					       &soc_component_driver_tas2764,
					       tas2764_dai_driver,
					       ARRAY_SIZE(tas2764_dai_driver));
}

static const struct i2c_device_id tas2764_i2c_id[] = {
	{ .name = "tas2764" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2764_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2764_of_match[] = {
	{ .compatible = "ti,tas2764",  .data = (void *)DEVID_TAS2764 },
	{ .compatible = "ti,sn012776", .data = (void *)DEVID_SN012776 },
	{},
};
MODULE_DEVICE_TABLE(of, tas2764_of_match);
#endif

static struct i2c_driver tas2764_i2c_driver = {
	.driver = {
		.name   = "tas2764",
		.of_match_table = of_match_ptr(tas2764_of_match),
	},
	.probe      = tas2764_i2c_probe,
	.id_table   = tas2764_i2c_id,
};
module_i2c_driver(tas2764_i2c_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TAS2764 I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");

#if IS_ENABLED(CONFIG_SND_SOC_TAS2764_KUNIT_TEST)
#include "tas2764-bop-test.h"
#include "tas2764-bclk-ratio-test.h"
#endif
