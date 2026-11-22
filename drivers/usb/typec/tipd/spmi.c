// SPDX-License-Identifier: GPL-2.0

#include <kunit/visibility.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/soc/apple/sleep.h>
#include <linux/spmi.h>
#include <linux/suspend.h>
#include <linux/usb/role.h>

#include "tps6598x.h"

#define tps_to_sn(tps) container_of_const((tps), struct sn201202x, cd.tps)

/* A sleeping controller does not answer a select; give up on it sooner. */
#define SN201202X_ATOMIC_SELECT_MS	10

/* An idle controller sleeps after this long without register access. */
#define SN201202X_AUTOSUSPEND_MS	1000
/* Failed tries to sleep before it waits for the next access */
#define SN201202X_SLEEP_TRIES		3

#define SN201202X_REG_LDCM_STATUS	0x74
#define SN201202X_LDCM_MITIGATE		BIT(6)
#define SN201202X_LDCM_DETECTED		BIT(2)
#define SN201202X_LDCM_STATE		GENMASK(3, 1)
#define SN201202X_LDCM_STATE_IDLE	0
#define SN201202X_LDCM_RESULT		GENMASK(5, 4)
#define SN201202X_LDCM_RETRY_MS		500

static int regmap_sn201202x_select_reg(struct spmi_device *sdev, u8 reg)
{
	int err;
	u8 val;
	bool warned = false;
	int attempts = 5;
	struct tps6598x *tps = spmi_device_get_drvdata(sdev);
	struct sn201202x *sn = tps_to_sn(tps);

	if (reg >= 0x80) {
		WARN_ON(1);
		return -EINVAL;
	}
	reinit_completion(&sn->select_completion);
	err = spmi_register_zero_write(sdev, reg);
	if (err)
		return err;

	if (sn->polled || sn->select_polled) {
		int poll_err;

		if (sn->atomic)
			poll_err = read_poll_timeout_atomic(spmi_register_read, err,
							    err || val == reg, 100,
							    SN201202X_ATOMIC_SELECT_MS * 1000,
							    false, sdev, 0, &val);
		else
			poll_err = read_poll_timeout(spmi_register_read, err,
						     err || val == reg, 100, 100000,
						     false, sdev, 0, &val);
		return err ?: poll_err;
	}

	if (!wait_for_completion_timeout(&sn->select_completion, msecs_to_jiffies(100)))
		return -ETIMEDOUT;

	while (--attempts) {
		err = spmi_register_read(sdev, 0, &val);
		if (err)
			return err;
		if (val == (reg | 0x80)) {
			if (!warned) {
				dev_warn(tps->dev,
					 "Got interrupt but selection not complete?\n");
				warned = true;
			}
			msleep(20);
			continue;
		}
		if (val == reg)
			return 0;
		return -EIO;
	}

	return -EIO;
}

static int __regmap_sn201202x_read(void *context,
				   const void *reg, size_t reg_size,
				   void *val, size_t val_size)
{
	struct spmi_device *sdev = context;
	int err;
	unsigned int offset = 0x20;
	size_t len;
	u8 addr;

	if (reg_size != 1) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (val_size > 0x40) {
		WARN_ON(1);
		return -EINVAL;
	}

	addr = *(u8 *)reg;

	err = regmap_sn201202x_select_reg(sdev, addr);
	if (err)
		return err;

	if (of_device_is_compatible(sdev->dev.of_node, "apple,t8132-sn201202x")) {
		u8 size;

		/* Selection completion does not establish the payload's length. */
		err = spmi_register_read(sdev, 0x1f, &size);
		if (err)
			return err;
		if (size < val_size || size > TPS_MAX_LEN) {
			dev_err_ratelimited(&sdev->dev,
					    "Register %#x reports %u bytes for a %zu-byte read\n",
					    addr, size, val_size);
			return -EIO;
		}
	}

	while (val_size) {
		len = min_t(size_t, val_size, 16);
		err = spmi_ext_register_read(context, offset, val, len);
		if (err)
			return err;
		offset += len;
		val += len;
		val_size -= len;
	}

	return 0;
}

static int __regmap_sn201202x_write(void *context, const void *data,
				    size_t count)
{
	int err = 0;
	unsigned int offset = 0xa0;
	size_t len;
	u8 addr;

	if (count < 1) {
		WARN_ON(1);
		return -EINVAL;
	}

	addr = *(u8 *)data;
	data += 1;
	count -= 1;

	if (count > 0x40) {
		WARN_ON(1);
		return -EINVAL;
	}

	err = regmap_sn201202x_select_reg(context, addr);
	if (err)
		return err;

	while (count) {
		len = min_t(size_t, count, 16);
		err = spmi_ext_register_write(context, offset, data, len);
		if (err)
			return err;
		offset += len;
		data += len;
		count -= len;
	}

	return err;
}

static int sn201202x_wake(struct sn201202x *sn);

/*
 * J714s: from the SLEEP at the prepare to the wake nothing reaches the controller, as on
 * macOS (sn201202x_prepare()). A register access that does all the same is logged, with its
 * caller the first time in a sleep, and gets the controller woken for it: a sleeping
 * controller ACKs a register selection but never completes it. Before the suspend the
 * controller then stays awake, and the suspend sends SLEEP as without the macOS entry; after
 * it, nothing would, so it is put back to sleep after the access (*resleep).
 */
static int sn201202x_entry_stray(struct sn201202x *sn, bool *resleep)
{
	int err;

	*resleep = false;
	if (likely(!READ_ONCE(sn->entry_asleep)))
		return 0;
	if (!sn->entry_strays++) {
		dev_warn(&sn->sdev->dev,
			 "sleep entry: register access while asleep since the prepare: woken for it\n");
		dump_stack();
	}
	err = sn201202x_wake(sn);
	if (err)
		return err;
	if (READ_ONCE(sn->entry_suspended))
		*resleep = true;
	else
		WRITE_ONCE(sn->entry_asleep, false);
	return 0;
}

/*
 * A register access wakes a controller that sleeps idle, and lets it sleep
 * again SN201202X_AUTOSUSPEND_MS after the last one, with its tries to sleep
 * counted afresh. In system sleep the event path wakes it and puts it back
 * to sleep itself.
 */
static int sn201202x_access_get(struct tps6598x *tps, bool *rpm, bool *resleep)
{
	int err;

	*resleep = false;
	*rpm = !READ_ONCE(tps_to_sn(tps)->atomic);
	if (!*rpm)
		return 0;
	err = pm_runtime_resume_and_get(tps->dev);
	if (err)
		return err;
	/* Held awake, so no runtime suspend runs meanwhile */
	tps_to_sn(tps)->sleep_tries = 0;
	err = sn201202x_entry_stray(tps_to_sn(tps), resleep);
	if (err)
		pm_runtime_put_autosuspend(tps->dev);
	return err;
}

static void sn201202x_access_put(struct tps6598x *tps, bool rpm, bool resleep)
{
	if (!rpm)
		return;
	/* Back to sleep after an access sn201202x_entry_stray() woke it for after the suspend */
	if (resleep)
		spmi_command_sleep(tps_to_sn(tps)->sdev);
	pm_runtime_put_autosuspend(tps->dev);
}

static int regmap_sn201202x_read(void *context,
				 const void *reg, size_t reg_size,
				 void *val, size_t val_size)
{
	struct tps6598x *tps = spmi_device_get_drvdata(context);
	bool rpm, resleep;
	int err;

	err = sn201202x_access_get(tps, &rpm, &resleep);
	if (err)
		return err;
	err = __regmap_sn201202x_read(context, reg, reg_size, val, val_size);
	sn201202x_access_put(tps, rpm, resleep);
	return err;
}

static int regmap_sn201202x_write(void *context, const void *data,
				  size_t count)
{
	struct tps6598x *tps = spmi_device_get_drvdata(context);
	bool rpm, resleep;
	int err;

	err = sn201202x_access_get(tps, &rpm, &resleep);
	if (err)
		return err;
	err = __regmap_sn201202x_write(context, data, count);
	sn201202x_access_put(tps, rpm, resleep);
	return err;
}

static irqreturn_t sn201202x_irq(int irq, void *data)
{
	struct completion *c = data;

	complete(c);
	return IRQ_HANDLED;
}

VISIBLE_IF_KUNIT const struct regmap_bus regmap_sn201202x = {
	.read				= regmap_sn201202x_read,
	.write				= regmap_sn201202x_write,
	.reg_format_endian_default	= REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default	= REGMAP_ENDIAN_NATIVE,
};
EXPORT_SYMBOL_IF_KUNIT(regmap_sn201202x);

VISIBLE_IF_KUNIT enum typec_contaminant
sn201202x_ldcm_decode(const u8 status[SN201202X_LDCM_STATUS_LEN])
{
	if (FIELD_GET(SN201202X_LDCM_STATE, status[1]) !=
	    SN201202X_LDCM_STATE_IDLE)
		return TYPEC_CONTAMINANT_UNKNOWN;

	if ((status[0] & SN201202X_LDCM_DETECTED) &&
	    (status[1] & SN201202X_LDCM_RESULT))
		return TYPEC_CONTAMINANT_LIQUID;

	return TYPEC_CONTAMINANT_NONE;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_ldcm_decode);

static int sn201202x_ldcm_read(void *ctx,
			      enum typec_contaminant *contaminant)
{
	struct tps6598x *tps = ctx;
	u8 status[SN201202X_LDCM_STATUS_LEN];
	int ret;

	ret = tps6598x_block_read(tps, SN201202X_REG_LDCM_STATUS, status,
				  sizeof(status));
	*contaminant = ret ? TYPEC_CONTAMINANT_UNKNOWN :
			     sn201202x_ldcm_decode(status);
	return ret;
}

static int sn201202x_ldcm_set_mitigation(void *ctx, bool enable)
{
	struct tps6598x *tps = ctx;
	struct sn201202x *sn = tps_to_sn(tps);
	u8 command = enable ? SN201202X_LDCM_MITIGATE : 0;
	int ret;

	/*
	 * The controller keeps its mitigation until told otherwise or reset.
	 * Repeating the current state would only disturb the port, so send a
	 * command only for a change or after a reset or failure.
	 */
	if (sn->ldcm_mitigation_known && sn->ldcm_mitigated == enable)
		return 0;

	/*
	 * As macOS, which reads a dry port's status but sends it no LDCM
	 * command: only a mitigation this driver requested since the
	 * controller's last reset needs removing.
	 */
	if (!enable && !sn->ldcm_requested) {
		sn->ldcm_mitigation_known = true;
		sn->ldcm_mitigated = false;
		return 0;
	}
	if (enable)
		sn->ldcm_requested = true;

	/*
	 * Successful task completion is the controller's mitigation ACK.
	 * Startup commands can take more than a second on SN201202x. Wait for
	 * completion before a retry can request the opposite mitigation state.
	 */
	ret = tps6598x_exec_cmd_tmo(tps, "LDCM", sizeof(command), &command,
				   0, NULL, 5000, 0);
	sn->ldcm_mitigation_known = !ret;
	if (!ret)
		sn->ldcm_mitigated = enable;
	if (!ret && !enable)
		sn->ldcm_requested = false;
	return ret;
}

static void sn201202x_ldcm_publish(void *ctx,
				   enum typec_contaminant contaminant)
{
	struct tps6598x *tps = ctx;

	typec_set_contaminant(tps->port, contaminant);
	tps->contaminant = contaminant;
}

VISIBLE_IF_KUNIT int
sn201202x_ldcm_update_state(void *ctx, const struct sn201202x_ldcm_ops *ops,
			    enum typec_contaminant last)
{
	enum typec_contaminant contaminant;
	int read_ret, ret;

	read_ret = ops->read(ctx, &contaminant);
	/*
	 * A measurement in progress is not a result. Keep the last result and
	 * its mitigation; the controller reports the new one when it is done.
	 */
	if (!read_ret && contaminant == TYPEC_CONTAMINANT_UNKNOWN &&
	    last != TYPEC_CONTAMINANT_UNKNOWN)
		return -EINPROGRESS;
	if (read_ret || contaminant != TYPEC_CONTAMINANT_NONE) {
		ret = ops->set_mitigation(ctx, true);
		if (ret) {
			ops->publish(ctx, TYPEC_CONTAMINANT_UNKNOWN);
			return ret;
		}

		ops->publish(ctx, contaminant);
		if (read_ret)
			return read_ret;
		return contaminant == TYPEC_CONTAMINANT_UNKNOWN ? -EAGAIN : 0;
	}

	ret = ops->set_mitigation(ctx, false);
	if (ret) {
		ops->publish(ctx, TYPEC_CONTAMINANT_UNKNOWN);
		return ret;
	}

	/* Do not report dry if liquid appeared while mitigation was removed. */
	read_ret = ops->read(ctx, &contaminant);
	if (!read_ret && contaminant == TYPEC_CONTAMINANT_UNKNOWN) {
		/*
		 * Removing mitigation can start another measurement. The dry
		 * snapshot above is still the latest result; preserve it just
		 * as we preserve a previously published result at entry. Forcing
		 * mitigation back on here can prevent the measurement finishing.
		 */
		ops->publish(ctx, TYPEC_CONTAMINANT_NONE);
		return -EINPROGRESS;
	}
	if (read_ret || contaminant != TYPEC_CONTAMINANT_NONE) {
		ret = ops->set_mitigation(ctx, true);
		if (ret) {
			ops->publish(ctx, TYPEC_CONTAMINANT_UNKNOWN);
			return ret;
		}
	}

	ops->publish(ctx, contaminant);
	if (read_ret)
		return read_ret;
	return contaminant == TYPEC_CONTAMINANT_UNKNOWN ? -EAGAIN : 0;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_ldcm_update_state);

static const struct sn201202x_ldcm_ops sn201202x_ldcm_ops = {
	.read = sn201202x_ldcm_read,
	.set_mitigation = sn201202x_ldcm_set_mitigation,
	.publish = sn201202x_ldcm_publish,
};

static int sn201202x_ldcm_update(struct tps6598x *tps)
{
	if (!tps->contaminant_ops)
		return 0;

	return sn201202x_ldcm_update_state(tps, &sn201202x_ldcm_ops,
					   tps->contaminant);
}

/* Measuring, or mitigated without a result yet: wait for the controller. */
static bool sn201202x_ldcm_pending(int ret)
{
	return ret == -EINPROGRESS || ret == -EAGAIN;
}

static void sn201202x_ldcm_event(struct tps6598x *tps)
{
	struct sn201202x *sn = tps_to_sn(tps);

	mod_delayed_work(system_freezable_wq, &sn->ldcm_work, 0);
}

static void sn201202x_ldcm_retry(struct tps6598x *tps, int ret)
{
	struct sn201202x *sn = tps_to_sn(tps);

	if (!sn201202x_ldcm_pending(ret))
		dev_err_ratelimited(tps->dev,
				    "liquid-detection status unavailable: %d\n", ret);
	mod_delayed_work(system_freezable_wq, &sn->ldcm_work,
			 msecs_to_jiffies(SN201202X_LDCM_RETRY_MS));
}

static void sn201202x_ldcm_work(struct work_struct *work)
{
	struct sn201202x *sn = container_of(to_delayed_work(work),
					  struct sn201202x, ldcm_work);
	struct tps6598x *tps = &sn->cd.tps;
	enum typec_contaminant last;
	int ret;

	mutex_lock(&tps->lock);
	last = tps->contaminant;
	ret = sn201202x_ldcm_update(tps);
	/* Apply the cable state again only if the published state changed. */
	if (tps->contaminant != last)
		mod_delayed_work(system_freezable_wq, &sn->cd.resume_work, 0);
	mutex_unlock(&tps->lock);

	if (ret)
		sn201202x_ldcm_retry(tps, ret);
}

/*
 * Refresh the snapshot in the caller's context, before it applies the cable
 * state, so that a port is never dropped for a stale "unknown". Context:
 * tps->lock held.
 */
static void sn201202x_ldcm_refresh(struct tps6598x *tps)
{
	int ret;

	lockdep_assert_held(&tps->lock);
	ret = sn201202x_ldcm_update(tps);
	if (ret)
		sn201202x_ldcm_retry(tps, ret);
}

/* The controller was reset, which also removed its mitigation. */
static void sn201202x_ldcm_invalidate(struct tps6598x *tps)
{
	struct sn201202x *sn = tps_to_sn(tps);

	tps->contaminant = TYPEC_CONTAMINANT_UNKNOWN;
	sn->ldcm_mitigated = false;
	sn->ldcm_mitigation_known = false;
	sn->ldcm_requested = false;
	sn201202x_ldcm_event(tps);
}

static void sn201202x_ldcm_stop(struct tps6598x *tps)
{
	/* Reset recovery is drained later and may still try to queue an update. */
	disable_delayed_work_sync(&tps_to_sn(tps)->ldcm_work);
}

VISIBLE_IF_KUNIT const struct tipd_contaminant_ops sn201202x_contaminant_ops = {
	.update = sn201202x_ldcm_update,
	.refresh = sn201202x_ldcm_refresh,
	.event = sn201202x_ldcm_event,
	.invalidate = sn201202x_ldcm_invalidate,
	.stop = sn201202x_ldcm_stop,
};
EXPORT_SYMBOL_IF_KUNIT(sn201202x_contaminant_ops);

static struct regmap *__devm_regmap_init_sn201202x(struct spmi_device *sdev,
						   const struct regmap_config *config,
						   struct lock_class_key *lock_key,
						   const char *lock_name)
{
	return __devm_regmap_init(&sdev->dev, &regmap_sn201202x, sdev, config,
				  lock_key, lock_name);
}

#define devm_regmap_init_sn201202x(dev, config)				\
	__regmap_lockdep_wrapper(__devm_regmap_init_sn201202x, #config,	\
				dev, config)

static const struct of_device_id sn201202x_of_match[] = {
	{ .compatible = "apple,sn201202x", (void *)TIPD_VARIANT_SN201202X},
	{}
};

/*
 * In system sleep the wake interrupt is suspended, so the controller is
 * polled as without interrupts.
 */
static int sn201202x_wake_poll(struct sn201202x *sn, bool poll)
{
	/* Busy-waiting has no clock to go by: each step is charged instead */
	int budget_ms = 500;
	unsigned int tries = 0;
	unsigned long deadline;
	u8 size;
	int ret;

	reinit_completion(&sn->wake_completion);
	ret = spmi_command_wakeup(sn->sdev);
	if (ret)
		return ret;
	/* A WAKEUP sent while it was still going to sleep may go unanswered */
	if (!poll && wait_for_completion_timeout(&sn->wake_completion,
						 msecs_to_jiffies(100)))
		return 0;

	/* A sleeping ACE3 ACKs commands but returns zeros until it is awake. */
	deadline = jiffies + msecs_to_jiffies(500);
	do {
		if (++tries % 20 == 0) {
			ret = spmi_command_wakeup(sn->sdev);
			if (ret)
				return ret;
		}
		ret = regmap_sn201202x_select_reg(sn->sdev, TPS_REG_MODE);
		if (ret == -ETIMEDOUT) {
			budget_ms -= SN201202X_ATOMIC_SELECT_MS;
			continue;
		}
		if (ret)
			return ret;
		ret = spmi_register_read(sn->sdev, 0x1f, &size);
		if (ret)
			return ret;
		if (size)
			return 0;
		if (sn->atomic)
			mdelay(1);
		else
			usleep_range(1000, 2000);
		budget_ms--;
	} while (sn->atomic ? budget_ms > 0 : time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int sn201202x_wake(struct sn201202x *sn)
{
	return sn201202x_wake_poll(sn, sn->polled);
}

/* For the commands the core gives only to a controller just woken */
static int sn201202x_cmd_wake(struct tps6598x *tps)
{
	return sn201202x_wake(tps_to_sn(tps));
}

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_runtime_suspend(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);

	/*
	 * A display, Thunderbolt or USB4 link keeps it awake: its events are
	 * untested with the controller asleep. The next event, such as the
	 * unplug, lets it sleep again. The data status is not read again
	 * without a plug, so the plug status says when the link is gone.
	 */
	if ((READ_ONCE(tps->status) & TPS_STATUS_PLUG_PRESENT) &&
	    (READ_ONCE(tps->data_status) & (TPS_DATA_STATUS_DP_CONNECTION |
					    TPS_DATA_STATUS_TBT_CONNECTION |
					    CD321X_DATA_STATUS_USB4_CONNECTION)))
		return -EBUSY;
	/* So does Apple's "DBMa" debug mode, for the routes it carries */
	if (tps->dbma_at_probe || tipd_debugfs_dbma_held(tps))
		return -EBUSY;

	reinit_completion(&sn->sleep_completion);
	if (spmi_command_sleep(sn->sdev))
		goto retry;
	if (!sn->polled &&
	    !wait_for_completion_timeout(&sn->sleep_completion, msecs_to_jiffies(100))) {
		/*
		 * It accepted SLEEP, so it may be asleep without having
		 * confirmed it. Woken, it tries again later; if it does not
		 * wake, it is taken as asleep, and the next access wakes it.
		 */
		if (sn201202x_wake(sn))
			goto asleep;
		goto retry;
	}
asleep:
	sn->sleep_tries = 0;
	return 0;

retry:
	/*
	 * With a new expiry, runtime PM tries again after the delay. Without
	 * a delay it does not, and the controller waits for the next access.
	 */
	if (++sn->sleep_tries < SN201202X_SLEEP_TRIES) {
		pm_runtime_mark_last_busy(dev);
		return -EAGAIN;
	}
	sn->sleep_tries = 0;
	dev_warn_once(dev, "cannot sleep while idle, awake until the next access\n");
	return -EBUSY;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_runtime_suspend);

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_runtime_resume(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);

	return sn201202x_wake(tps_to_sn(tps));
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_runtime_resume);

/*
 * The SPMI bus holds a runtime PM reference while the driver is bound.
 * Probe gives it to idle sleep only where the event interrupt stays live
 * while the controller sleeps (sn->idle_sleep); elsewhere the controller
 * stays awake.
 */
VISIBLE_IF_KUNIT void sn201202x_idle_sleep_start(struct tps6598x *tps)
{
	if (!tps_to_sn(tps)->idle_sleep)
		return;
	pm_runtime_set_autosuspend_delay(tps->dev, SN201202X_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(tps->dev);
	pm_runtime_put_autosuspend(tps->dev);
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_idle_sleep_start);

/* On remove, once the bus has woken it: take back the reference */
VISIBLE_IF_KUNIT void sn201202x_idle_sleep_stop(struct tps6598x *tps)
{
	if (!tps_to_sn(tps)->idle_sleep)
		return;
	pm_runtime_get_noresume(tps->dev);
	pm_runtime_dont_use_autosuspend(tps->dev);
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_idle_sleep_stop);

/*
 * The links that wake the system when they appear in system sleep. A port
 * attaches as a USB device before any data role swap, as it does to a
 * charger, and a charger may still show a USB link for a moment after one:
 * a USB link counts only with this port as the host, and from a power source
 * only if the source is capable of USB communication.
 */
VISIBLE_IF_KUNIT u32 sn201202x_sleep_links(const struct cd321x_sleep_state *st)
{
	u32 links = st->data_status & (TPS_DATA_STATUS_DP_CONNECTION |
				       TPS_DATA_STATUS_TBT_CONNECTION |
				       CD321X_DATA_STATUS_USB4_CONNECTION);

	if (st->data_status & TPS_DATA_STATUS_USB_DATA_ROLE)
		return links;
	if (!(st->status & TPS_STATUS_PORTROLE) && !st->usb_comm)
		return links;
	return links | (st->data_status & (TPS_DATA_STATUS_USB2_CONNECTION |
					   TPS_DATA_STATUS_USB3_CONNECTION));
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_sleep_links);

/*
 * The event interrupt stays live in system sleep (IRQF_NO_SUSPEND). Wake the
 * controller to acknowledge the event, and wake the system only for a new
 * data connection, or if the controller cannot be read. It goes back to sleep
 * either way, as the resume expects.
 */
static bool sn201202x_sleep_event(struct tps6598x *tps)
{
	struct sn201202x *sn = tps_to_sn(tps);
	struct cd321x_sleep_state st = {};
	u32 links = 0;
	bool wake, ldcm_changed = false;
	int ldcm_ret = 0;
	int ret;

	guard(mutex)(&tps->lock);
	if (!READ_ONCE(sn->sleep_watch))
		return false;

	WRITE_ONCE(sn->atomic, true);
	ret = sn201202x_wake_poll(sn, true);
	if (!ret)
		ret = cd321x_sleep_events(tps, &st);
	if (!ret)
		links = sn201202x_sleep_links(&st);
	if (!ret && st.contaminant_event) {
		enum typec_contaminant last = tps->contaminant;

		ldcm_ret = sn201202x_ldcm_update(tps);
		/* Wake for a new result or a failure, not for a measurement. */
		if (ldcm_ret == -EINPROGRESS)
			ldcm_ret = 0;
		ldcm_changed = tps->contaminant != last;
	}
	wake = ret || ldcm_ret || ldcm_changed || (links & ~sn->sleep_links);
	if (!ret)
		sn->sleep_links = links;
	spmi_command_sleep(sn->sdev);
	WRITE_ONCE(sn->atomic, false);
	if (wake)
		pm_wakeup_dev_event(tps->dev, 0, true);
	return true;
}

/*
 * For a controller that follows the sleep's rules (apple,sleep-ssps-rules), the probe and the
 * sleep's SLEEP command print a line; for others they don't. A controller without a connector (the
 * MagSafe one, hpm5) is probed as any other: it registers its Type-C port and power supply, and
 * with apple,sleep-ssps it follows the system's sleep as the USB-C ports' controllers do (woken and
 * switched to S3 at the prepare, SLEEP at the suspend, woken at the resume and switched to S0 at
 * the complete; on the J714s as macOS does it, see sn201202x_prepare_macos()).
 */
static bool sn201202x_sleep_board(struct sn201202x *sn)
{
	return device_property_read_bool(sn->cd.tps.dev, "apple,sleep-ssps-rules");
}

/*
 * A controller that the previous boot left in the middle of a command can fail its first
 * probe on the reads that follow. On the J714s, after a soft reboot that ended a deep-sleep
 * test run, the controllers of both USB-C ports ran their LDCM command for more than three
 * seconds. Their SPMI buses then reported frames that failed the parity check and commands
 * that were not acknowledged, the power status read failed with -EIO, and the probe failed with
 * -EINVAL. Both ports, and every USB, Thunderbolt and DisplayPort device on them, stayed gone
 * until the next boot.
 *
 * On that board a failed probe is tried again after a growing wait. After the third failed try
 * the controller's firmware is cold-reset ("Gaid", as the debug link's recovery does), unless
 * it is in the debug mode of the boot loader's debug link, and it is tried twice more. Each try
 * runs in a devres group of its own, so what a failed try set up is released before the next.
 * A controller that is still not up after that defers its probe: the driver core tries it
 * again later, and the port is never left failed while its controller is there.
 */
#define SN201202X_INIT_TRIES		5
#define SN201202X_INIT_RESET_AFTER	3
#define SN201202X_RESET_MS		1000
#define SN201202X_RESET_UP_MS		2000

static const unsigned int sn201202x_init_wait_ms[SN201202X_INIT_TRIES - 1] = {
	200, 500, 0, 1000,
};

/* 0 with the controller's mode in @mode, or the error of the read */
static int sn201202x_mode(struct tps6598x *tps, char mode[4])
{
	int ret = tps6598x_block_read(tps, TPS_REG_MODE, mode, 4);

	if (ret)
		return ret;
	return memchr_inv(mode, 0, 4) ? 0 : -ENODEV;
}

static int sn201202x_cold_reset(struct sn201202x *sn)
{
	struct tps6598x *tps = &sn->cd.tps;
	char mode[4];
	int ms, ret;

	sn201202x_wake(sn);
	ret = sn201202x_mode(tps, mode);
	/* The boot loader's debug link runs in this mode: it is never reset from here */
	if (!ret && !memcmp(mode, "DBMa", 4))
		return -EBUSY;
	scoped_guard(mutex, &tps->lock)
		ret = tps6598x_exec_cmd_tmo(tps, "Gaid", 0, NULL, 0, NULL, 2000, 0);
	if (ret)
		dev_dbg(tps->dev, "reset command: %d\n", ret);
	msleep(SN201202X_RESET_MS);
	for (ms = 0; ms < SN201202X_RESET_UP_MS; ms += 50) {
		if (!sn201202x_wake(sn) && !sn201202x_mode(tps, mode) && !memcmp(mode, "APP ", 4))
			return 0;
		msleep(50);
	}
	return -ETIMEDOUT;
}

static int sn201202x_init(struct sn201202x *sn)
{
	struct tps6598x *tps = &sn->cd.tps;
	unsigned int try;
	void *group;
	int ret;

	if (!device_property_read_bool(tps->dev, "apple,reset-on-failed-probe")) {
		ret = tipd_init(tps);
		if (ret)
			cancel_delayed_work_sync(&sn->ldcm_work);
		/*
		 * A command still running when the probe left the boot power state (tipd_init():
		 * tipd_boot_power_on()) is no reason to give the port up: try again later. The
		 * loop below tries again itself, with a cold reset for a controller stuck in one.
		 */
		return ret == -EBUSY ? -EPROBE_DEFER : ret;
	}

	for (try = 0; try < SN201202X_INIT_TRIES; try++) {
		group = devres_open_group(tps->dev, NULL, GFP_KERNEL);
		if (!group)
			return -ENOMEM;
		ret = tipd_init(tps);
		if (!ret) {
			devres_remove_group(tps->dev, group);
			if (try)
				dev_info(tps->dev, "port controller up at try %u\n", try + 1);
			return 0;
		}
		devres_release_group(tps->dev, group);
		cancel_delayed_work_sync(&sn->ldcm_work);
		if (ret == -EPROBE_DEFER || ret == -ENOMEM)
			return ret;
		if (try + 1 == SN201202X_INIT_TRIES)
			break;
		dev_warn(tps->dev, "probe try %u failed: %d; trying again\n", try + 1, ret);
		if (try + 1 == SN201202X_INIT_RESET_AFTER)
			dev_info(tps->dev, "controller reset before try %u: %d\n", try + 2,
				 sn201202x_cold_reset(sn));
		msleep(sn201202x_init_wait_ms[try]);
		sn201202x_wake(sn);
	}
	dev_err(tps->dev, "not up after %u tries and a reset (%d): probe deferred\n",
		SN201202X_INIT_TRIES, ret);
	return -EPROBE_DEFER;
}

static int sn201202x_usbw_notify(struct notifier_block *nb, unsigned long usb_asleep, void *unused);

static int sn201202x_probe(struct spmi_device *device)
{
	const struct of_device_id *match;
	const struct tipd_data *data;
	struct usb_role_switch *sw;
	struct fwnode_handle *fwnode;
	struct sn201202x *sn;
	struct tps6598x *tps;
	int irq_select, irq_sleep, irq_wake;
	int ret;

	match = of_match_device(sn201202x_of_match, &device->dev);
	if (!match)
		return -EINVAL;
	data = tipd_hw_variants[(enum tipd_hw_variant)(uintptr_t)match->data];

	/*
	 * tipd_init() defers until the connector's USB role switch (the dwc3)
	 * is registered, but only after it has woken the chip, switched it to
	 * S0 and set its interrupt mask, and the deferral then puts the chip
	 * back to sleep. With both drivers loaded by udev at boot that cycle
	 * could repeat several times. Defer here instead, before the chip is
	 * touched. A connector without a role switch, or one whose remote node
	 * is disabled, gives NULL, and the probe continues as before.
	 */
	fwnode = device_get_named_child_node(&device->dev, "connector");
	sw = fwnode_usb_role_switch_get(fwnode);
	fwnode_handle_put(fwnode);
	if (IS_ERR(sw))
		return dev_err_probe(&device->dev, PTR_ERR(sw), "waiting for the USB role switch\n");
	usb_role_switch_put(sw);

	sn = devm_kzalloc(&device->dev, data->tps_struct_size, GFP_KERNEL);
	if (!sn)
		return -ENOMEM;
	sn->sdev = device;
	tps = &sn->cd.tps;

	mutex_init(&tps->lock);
	tps->dev = &device->dev;
	tps->data = data;
	/* A port without liquid detection (MagSafe 3) gets no LDCM commands */
	if (!of_property_read_bool(device->dev.of_node, "apple,no-liquid-detection"))
		tps->contaminant_ops = &sn201202x_contaminant_ops;
	INIT_DELAYED_WORK(&sn->ldcm_work, sn201202x_ldcm_work);

	init_completion(&sn->select_completion);
	init_completion(&sn->sleep_completion);
	init_completion(&sn->wake_completion);

	sn->polled = !of_property_present(device->dev.of_node, "interrupts") &&
		     !of_property_present(device->dev.of_node, "interrupts-extended");
	if (!sn->polled) {
		tps->irq = of_irq_get_byname(device->dev.of_node, "irq");
		if (tps->irq < 0)
			return tps->irq;
		irq_select = of_irq_get_byname(device->dev.of_node, "select");
		/* T8132 has no select notification, but retains event/sleep/wake IRQs. */
		if (irq_select == -EINVAL || irq_select == -ENODATA) {
			if (!of_device_is_compatible(device->dev.of_node,
						     "apple,t8132-sn201202x"))
				return irq_select;
			sn->select_polled = true;
		} else if (irq_select < 0) {
			return irq_select;
		}
		irq_sleep = of_irq_get_byname(device->dev.of_node, "sleep");
		if (irq_sleep < 0)
			return irq_sleep;
		irq_wake = of_irq_get_byname(device->dev.of_node, "wake");
		if (irq_wake < 0)
			return irq_wake;

		if (!sn->select_polled) {
			ret = devm_request_irq(&device->dev, irq_select, sn201202x_irq,
					       0, NULL, &sn->select_completion);
			if (ret)
				return ret;
		}
		ret = devm_request_irq(&device->dev, irq_sleep, sn201202x_irq,
				       0, NULL, &sn->sleep_completion);
		if (ret)
			return ret;
		ret = devm_request_irq(&device->dev, irq_wake, sn201202x_irq,
				       0, NULL, &sn->wake_completion);
		if (ret)
			return ret;
	}

	/*
	 * In system sleep only the event interrupt stays live. Register access
	 * then has to poll, which a select notification would not.
	 */
	if (!sn->polled && sn->select_polled)
		tps->sleep_event = sn201202x_sleep_event;
	/*
	 * Idle sleep needs only the event interrupt to stay live while the
	 * controller sleeps. T8132's select-polled controllers have that, as
	 * the system sleep filter above relies on; elsewhere the DT says so
	 * (apple,event-irq-in-sleep). A register access then wakes the
	 * controller through runtime PM, select notification or not. Those
	 * controllers keep their own system sleep path (a wake IRQ).
	 */
	sn->idle_sleep = !sn->polled &&
			 (sn->select_polled ||
			  device_property_read_bool(&device->dev, "apple,event-irq-in-sleep"));

	tps->wake = sn201202x_cmd_wake;
	spmi_device_set_drvdata(device, tps);
	tps->regmap = devm_regmap_init_sn201202x(device, &tps6598x_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	ret = sn201202x_wake(sn);
	if (ret)
		return ret;

	ret = sn201202x_init(sn);
	if (ret) {
		spmi_command_sleep(device);
		return ret;
	}
	sn201202x_ldcm_event(tps);

	/* Events can latch before the IRQ is registered. Consume that state now. */
	tps->data->irq_handler(tps->irq, tps);

	sn201202x_idle_sleep_start(tps);
	/* A controller that follows the system's power state can take macOS's USBw at its entry */
	if (device_property_read_bool(tps->dev, "apple,sleep-ssps")) {
		struct fwnode_handle *con = device_get_named_child_node(tps->dev, "connector");

		/* MagSafe's controller has no connector and never gets macOS's USBw */
		sn->usbw_port = !!con;
		fwnode_handle_put(con);
		sn->usbw_nb.notifier_call = sn201202x_usbw_notify;
		ret = apple_sleep_usb_asleep_notifier_register(&sn->usbw_nb);
		if (ret)
			dev_warn(tps->dev, "USB wake arm: not registered: %d\n", ret);
	}
	if (sn201202x_sleep_board(sn))
		dev_info(tps->dev, "port controller ready%s\n", sn->polled ? ", polled" : "");
	return 0;
}

static void sn201202x_remove(struct spmi_device *device)
{
	struct tps6598x *tps = spmi_device_get_drvdata(device);
	struct sn201202x *sn = tps_to_sn(tps);

	if (sn->usbw_nb.notifier_call)
		apple_sleep_usb_asleep_notifier_unregister(&sn->usbw_nb);
	sn201202x_idle_sleep_stop(tps);
	tipd_remove(tps);
	spmi_command_sleep(sn->sdev);
}

/*
 * macOS's hand-back of an arm sent at the entry (sn201202x_usbw_notify()): USBw 0x80, then
 * 0x00. At the wake, in its SSPS 0x00 window; at the complete if the wake's WAKEUP failed, as
 * the arm's watch keeps the USB2 lines until then.
 */
static void sn201202x_usbw_hand_back(struct sn201202x *sn, const char *when)
{
	struct tps6598x *tps = &sn->cd.tps;
	int back, off;

	lockdep_assert_held(&tps->lock);
	if (!sn->usbw_macos)
		return;
	back = tipd_usbw_locked(tps, TIPD_USBW_HAND_BACK);
	off = tipd_usbw_locked(tps, TIPD_USBW_OFF);
	sn->usbw_macos = false;
	dev_info(tps->dev, "sleep wake: USBw %#x: %d, USBw %#x: %d%s\n", TIPD_USBW_HAND_BACK, back,
		 TIPD_USBW_OFF, off, when);
}

/*
 * J714s: macOS's wake of a controller it sent SLEEP at the entry: WAKEUP,
 * INT_EVENT1 read and the same bits cleared, the status read, then SSPS back to S0, all before
 * the rest of the resume, which goes on as before. Linux's S0 used to wait for the complete,
 * whose own S0 now finds it done. For an empty USB-C port macOS uses 0x02, for an
 * attached one and MagSafe 0x00; Linux has only S0, which this keeps.
 */
static void sn201202x_wake_macos(struct sn201202x *sn, int wake_err)
{
	struct tps6598x *tps = &sn->cd.tps;
	u8 event[CD321X_WAKE_EVENT_LEN] = {};
	u32 status = 0, data_status = 0;
	ktime_t start = ktime_get();
	int ret, s0;

	if (wake_err) {
		dev_info(tps->dev, "sleep wake: WAKEUP failed: %d, nothing read; %u register accesses while asleep\n",
			 wake_err, sn->entry_strays);
		return;
	}
	scoped_guard(mutex, &tps->lock) {
		ret = cd321x_wake_events(tps, event, &status, &data_status);
		s0 = tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
		/* macOS's hand-back in the same window */
		sn201202x_usbw_hand_back(sn, "");
	}
	dev_info(tps->dev, "sleep wake: events %*phN cleared: %d; status %#010x, data status %#010x; S0: %d, %lld ms after the WAKEUP; %u register accesses while asleep\n",
		 CD321X_WAKE_EVENT_LEN, event, ret, status, data_status, s0,
		 ktime_ms_delta(ktime_get(), start), sn->entry_strays);
}

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_resume(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	int err;

	if (READ_ONCE(sn->sleep_watch)) {
		/* Waits for a running sleep_event() */
		disable_irq(tps->irq);
		WRITE_ONCE(sn->sleep_watch, false);
	}
	/* J714s: asleep since the prepare, until the WAKEUP below */
	WRITE_ONCE(sn->entry_asleep, false);
	WRITE_ONCE(sn->entry_suspended, false);
	/* One that slept on from idle sleep wakes through runtime PM */
	if (pm_runtime_status_suspended(dev)) {
		err = pm_runtime_resume(dev);
		if (err > 0)
			err = 0;
	} else {
		err = sn201202x_wake(sn);
	}
	if (sn->macos_sleep)
		sn201202x_wake_macos(sn, err);
	if (!err) {
		/* Registered before xHCI, so this is before the root port resumes. */
		tipd_usb_wake(tps, false);
		err = tipd_resume(tps);
	}
	/* The resume revalidation acknowledges events latched while asleep. */
	if (tps->irq)
		enable_irq(tps->irq);
	pm_runtime_put_autosuspend(dev);
	return err;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_resume);

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_suspend(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	bool watch;
	int err, ret;

	/*
	 * A sleeping controller cannot be read, so its events must not be
	 * handled until it is awake again. An interrupt arriving meanwhile is
	 * replayed when it is enabled.
	 */
	if (tps->irq)
		disable_irq(tps->irq);
	/* Awake, also from idle sleep, until resume */
	err = pm_runtime_resume_and_get(dev);
	if (err) {
		/*
		 * It sleeps idle and does not wake: let it sleep on without
		 * the wakeup watch, and let resume wake it.
		 */
		dev_warn(dev, "asleep and not waking: %d, no wakeup for an attach\n", err);
		pm_runtime_get_noresume(dev);
		return 0;
	}
	err = tipd_suspend(tps);
	if (err)
		goto out_put;
	watch = tps->sleep_event && device_may_wakeup(dev) &&
		pm_suspend_target_state == PM_SUSPEND_TO_IDLE;
	if (watch) {
		struct cd321x_sleep_state st;

		guard(mutex)(&tps->lock);
		/* Without a baseline, a plug-in in sleep cannot be told from a charger */
		watch = !cd321x_sleep_events(tps, &st);
		if (watch)
			sn->sleep_links = sn201202x_sleep_links(&st);
		else
			dev_warn(dev, "port state unreadable, no wakeup for an attach\n");
	}
	/* Registered before xHCI, so its root port has suspended by now. */
	tipd_usb_wake(tps, true);
	/* J714s: asleep since the prepare, as on macOS */
	if (READ_ONCE(sn->entry_asleep)) {
		WRITE_ONCE(sn->entry_suspended, true);
		return 0;
	}
	reinit_completion(&sn->sleep_completion);
	err = spmi_command_sleep(sn->sdev);
	if (pm_suspend_via_firmware() && sn201202x_sleep_board(sn))
		dev_info(dev, "SLEEP: %d\n", err);
	if (err)
		goto out_resume;
	if (!sn->polled &&
	    !wait_for_completion_timeout(&sn->sleep_completion, msecs_to_jiffies(100))) {
		err = -ETIMEDOUT;
		goto out_wake;
	}
	if (watch) {
		WRITE_ONCE(sn->sleep_watch, true);
		enable_irq(tps->irq);
	}
	return 0;

out_wake:
	/* It accepted SLEEP, so it may be asleep without having confirmed it. */
	ret = sn201202x_wake(sn);
	if (ret)
		dev_err(dev, "failed to wake up after failed suspend: %d\n", ret);
out_resume:
	tipd_resume(tps);
out_put:
	pm_runtime_put_autosuspend(dev);
	if (tps->irq)
		enable_irq(tps->irq);
	return err;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_suspend);

/*
 * J714s (macOS's sleeps traced with and without a partner): macOS arms USB wake on every USB-C
 * port controller with a partner, never on MagSafe's, in a window of its own 0.5 ms after
 * the port's PHY has stopped and 54-73 ms after its SSPS 0x03: (WAKEUP) CMD1 idle read, DATA1 =
 * 0x09, USBw, its end read, DATA1 read, SLEEP; no SYS_PWR_STATE read and nothing else. At the
 * wake, in the same window as its SSPS 0x00: USBw 0x80, then USBw 0x00. Linux armed none (no
 * apple,usb-remote-wake). With the macOS entry, apple-sleep's .prepare_late (after every noirq
 * suspend) calls sn201202x_usbw_notify().
 */
static int sn201202x_usbw_notify(struct notifier_block *nb, unsigned long usb_asleep, void *unused)
{
	struct sn201202x *sn = container_of(nb, struct sn201202x, usbw_nb);
	struct tps6598x *tps = &sn->cd.tps;
	ktime_t start;
	int ret;

	if (!sn->macos_sleep || !sn->usbw_port)
		return NOTIFY_DONE;
	guard(mutex)(&tps->lock);
	sn->usbw_macos = false;
	if (!tps->partner)
		return NOTIFY_DONE;
	start = ktime_get();
	WRITE_ONCE(sn->atomic, true);
	ret = sn201202x_wake_poll(sn, true);
	if (!ret) {
		ret = tipd_usbw_locked(tps, TIPD_USBW_HIGH_SPEED);
		/*
		 * Even a USBw whose end wasn't seen can have taken the USB2 lines (seen with plain
		 * CMD1 reads: every USB device on both USB-C ports gone after the wake, the dock's
		 * until a reboot): the wake hands them back whenever it went out.
		 */
		sn->usbw_macos = true;
	}
	spmi_command_sleep(sn->sdev);
	WRITE_ONCE(sn->atomic, false);
	dev_info(tps->dev, "sleep entry: USBw %#x after the PHYs' stop (a USB host asleep in place: %lu): %d, %lld us\n",
		 TIPD_USBW_HIGH_SPEED, usb_asleep, ret, ktime_us_delta(ktime_get(), start));
	return NOTIFY_DONE;
}

/*
 * J714s: enter the sleep as macOS does (its sleeps traced on the J714s). There, each
 * port controller gets, before any USB or PHY stop: WAKEUP, a read of its system power state,
 * SSPS 0x03 read to its end, and SPMI SLEEP straight after its result is read, and then nothing
 * until the wake (sn201202x_wake_macos()). No events are read or cleared, no LDCM and no LOCK.
 * Linux sent S3 at the prepare but SLEEP only at the suspend. In between its 500 ms poll, on a
 * workqueue the freezer leaves running, read and cleared INT_EVENT1 and read the status of a
 * controller already in S3. And the prepare first read the liquid detector and could send LDCM,
 * a command of up to 5 s, which these controllers have been seen to run for over 3 s with SPMI
 * parity faults and NACKs after it (sn201202x_init()); right after a replug a measurement can
 * still be running. Here the poll stops first (one already running finishes, and its own
 * liquid-detection handling with it), the prepare neither reads the detector nor sends LDCM
 * (sn201202x_macos_dry() takes only a port last known dry or mitigated), S3 goes as before,
 * and SLEEP right after it. The port's other work is freezable and stays frozen until after the
 * complete. Outside tipd only apple-sleep's debug link cut talks to a port controller during a
 * sleep, at the noirq suspend, and it sends its own WAKEUP first.
 */
static bool sn201202x_macos_entry(struct sn201202x *sn)
{
	struct device *dev = sn->cd.tps.dev;

	/*
	 * Keyed on what the controller presents: polled (no interrupt), following the system's
	 * power state, for a firmware sleep. A port that arms USB wake itself (none on the J714s)
	 * needs the controller at the suspend.
	 */
	return sn->polled &&
	       pm_suspend_via_firmware() &&
	       device_property_read_bool(dev, "apple,sleep-ssps") &&
	       !device_property_read_bool(dev, "apple,usb-remote-wake");
}

/*
 * The detector isn't read at the macOS entry, so it is taken only from a port last known dry or
 * mitigated, read once the poll (whose events can change that) has stopped; any other enters as
 * before, whose reading or mitigation can refuse the sleep.
 */
static bool sn201202x_macos_dry(struct sn201202x *sn)
{
	struct tps6598x *tps = &sn->cd.tps;

	guard(mutex)(&tps->lock);
	return !tps->contaminant_ops || tps->contaminant == TYPEC_CONTAMINANT_NONE ||
	       sn->ldcm_mitigated;
}

static int sn201202x_prepare_macos(struct sn201202x *sn)
{
	struct tps6598x *tps = &sn->cd.tps;
	ktime_t start, s3_end;
	int ret, err;

	/* The poll stopped already, then the port's work and S3: woken, read, commanded, its end read */
	start = ktime_get();
	ret = tipd_prepare(tps->dev);
	s3_end = ktime_get();
	if (ret) {
		/* Completed again there, and no complete follows: poll back, detector read */
		sn->macos_sleep = false;
		tipd_poll_start(tps);
		scoped_guard(mutex, &tps->lock)
			sn201202x_ldcm_refresh(tps);
		dev_info(tps->dev, "sleep entry: poll and port work stopped, LDCM skipped; S3 refused the sleep: %d after %lld ms; no SLEEP, poll started again\n",
			 ret, ktime_ms_delta(s3_end, start));
		return ret;
	}
	scoped_guard(mutex, &tps->lock) {
		err = spmi_command_sleep(sn->sdev);
		WRITE_ONCE(sn->entry_asleep, !err);
	}
	/* The S3 command's own line comes just before this one */
	dev_info(tps->dev, "sleep entry: poll and port work stopped, LDCM skipped (last result %d); S3 at the prepare, %lld ms; SLEEP: %d, %lld us after it%s; no sleep confirmation on the polled bus\n",
		 tps->contaminant_ops ? (int)tps->contaminant : -1,
		 ktime_ms_delta(s3_end, start), err, ktime_us_delta(ktime_get(), s3_end),
		 err ? ", sent again at the suspend" : "");
	return 0;
}

static int sn201202x_prepare(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	int ret;

	cancel_delayed_work_sync(&sn->ldcm_work);
	sn->entry_strays = 0;
	WRITE_ONCE(sn->entry_suspended, false);
	sn->macos_sleep = sn201202x_macos_entry(sn);
	if (sn->macos_sleep) {
		tipd_poll_stop(tps);
		if (sn201202x_macos_dry(sn))
			return sn201202x_prepare_macos(sn);
		sn->macos_sleep = false;
		tipd_poll_start(tps);
	}
	mutex_lock(&tps->lock);
	ret = sn201202x_ldcm_update(tps);
	mutex_unlock(&tps->lock);
	/* A measurement in progress keeps the last result; its event wakes us. */
	if (ret == -EINPROGRESS)
		ret = 0;
	if (ret && !sn->ldcm_mitigated) {
		/* Do not enter sleep without a dry snapshot or mitigation ACK. */
		sn201202x_ldcm_event(tps);
		return ret;
	}
	if (ret)
		dev_warn(tps->dev,
			 "connector state unavailable; entering sleep mitigated: %d\n",
			 ret);
	ret = tipd_prepare(dev);
	/* Refused there and completed again: the detector's snapshot as after a wake. */
	if (ret) {
		scoped_guard(mutex, &tps->lock)
			sn201202x_ldcm_refresh(tps);
	}
	return ret;
}

static void sn201202x_complete(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	int err;

	/* J714s: a sleep given up after the prepare's SLEEP, with no resume: woken here */
	WRITE_ONCE(sn->entry_suspended, false);
	if (READ_ONCE(sn->entry_asleep)) {
		WRITE_ONCE(sn->entry_asleep, false);
		err = sn201202x_wake(sn);
		dev_info(dev, "sleep wake: sleep given up after the entry: WAKEUP %d at the complete; %u register accesses while asleep\n",
			 err, sn->entry_strays);
	}
	/* J714s: an entry's USBw arm the wake couldn't hand back, its WAKEUP having failed */
	if (sn->usbw_macos) {
		err = sn201202x_wake(sn);
		scoped_guard(mutex, &tps->lock)
			sn201202x_usbw_hand_back(sn, err ? ", at the complete, WAKEUP failed" :
						     ", at the complete");
	}
	/* Refresh the detector snapshot before cable revalidation is thawed. */
	scoped_guard(mutex, &tps->lock)
		sn201202x_ldcm_refresh(tps);
	tipd_complete(dev);
	/* J714s: the poll stopped at the prepare, if no resume started it again */
	if (sn->macos_sleep) {
		sn->macos_sleep = false;
		tipd_poll_start(tps);
	}
}

MODULE_DEVICE_TABLE(of, sn201202x_of_match);

static const struct dev_pm_ops sn201202x_pm_ops = {
	.prepare = pm_sleep_ptr(sn201202x_prepare),
	.complete = pm_sleep_ptr(sn201202x_complete),
	SET_SYSTEM_SLEEP_PM_OPS(sn201202x_suspend, sn201202x_resume)
	SET_RUNTIME_PM_OPS(sn201202x_runtime_suspend, sn201202x_runtime_resume, NULL)
};

static struct spmi_driver sn201202x_driver = {
	.driver = {
		.name = "sn201202x",
		.pm = &sn201202x_pm_ops,
		.of_match_table = sn201202x_of_match,
		/*
		 * Renegotiating an inherited connection waits several seconds per
		 * port; let the ports of one machine probe at the same time.
		 */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = sn201202x_probe,
	.remove = sn201202x_remove,
};
module_spmi_driver(sn201202x_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI SN201202x USB Power Delivery Controller Driver");
