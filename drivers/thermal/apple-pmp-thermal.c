// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * T8132 CPU/GPU thermal policy using read-only samples from the PMP owner.
 * The policy and thresholds are carried from the existing M4 implementation;
 * these are Linux operating limits, not inferred Apple hardware safety limits.
 * Guards are installed before cpufreq relinquishes its unmanaged ceiling.
 */

#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/hwmon.h>
#include <linux/hrtimer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/macsmc.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/soc/apple/cpufreq-thermal.h>
#include <linux/soc/apple/pmp-temps.h>
#include <linux/soc/apple/pmp-telemetry.h>
#include <linux/soc/apple/pmp-cooling.h>
#include <linux/soc/apple/pmp-power-policy.h>
#include <linux/soc/apple/t8132.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#include "apple-pmp-confirm.h"

/* Fast raw protection and ordinary governor regulation have separate clocks.
 * No sysfs read advances either controller. Thresholds remain a development
 * policy, not an inference of Apple's limits from macOS temperature peaks.
 */
#define APPLE_PMP_HYST_MC		2000
#define APPLE_PMP_POLL_MS		5
/* Inside the cooling slope window (50-250 ms); used while cool and idle. */
#define APPLE_PMP_SLOW_POLL_MS		100
#define APPLE_PMP_GOVERNOR_NS	100000000ULL

struct apple_pmp_cluster {
	struct cpufreq_policy *policy;
	struct thermal_cooling_device *cdev;
	struct freq_qos_request guard;
};

static struct {
	const struct apple_pmp_sensor_data *sensors;
	struct thermal_zone_device *zone;
	struct apple_pmp_cluster clusters[2];
	struct mutex lock; /* Serializes sensor reads, mode changes and guard updates. */
	struct apple_pmp_cooling cooling;
	struct apple_pmp_power_policy power;
	struct delayed_work setup_work;
	struct delayed_work power_work;
	struct device *power_hwmon;
	unsigned int rail_mw[3];
	int power_error;
	u64 gpu_epoch;
	bool power_polling;
	unsigned int guard_level;
	bool enabled;
	long cached_mc;
	int cached_error;
	/* The last reading from a sensor that was valid, and when. */
	u64 valid_ns;
	long valid_mc;
	u64 governor_ns;
	unsigned long long sample_count, governor_count;
	struct apple_pmp_confirmation critical;
	bool shutdown_requested;
	struct hrtimer poll_timer;
	struct work_struct poll_work;
	bool polling;
	bool fast_poll;
	bool owner; /* apple-soc-cpufreq's unmanaged cap is released */
	bool critical_dumped;
	bool ready;
	bool sleep_paused;
} pmp_thermal = {
	.lock = __MUTEX_INITIALIZER(pmp_thermal.lock),
};

/* Read-only cadence diagnostics: reading temperatures must not advance these. */
module_param_named(sample_count, pmp_thermal.sample_count, ullong, 0444);
module_param_named(governor_count, pmp_thermal.governor_count, ullong, 0444);
module_param_named(ready, pmp_thermal.ready, bool, 0444);

static int apple_pmp_power_status(char *buffer, const struct kernel_param *kp)
{
	struct apple_pmp_power_policy *p = &pmp_thermal.power;

	guard(mutex)(&pmp_thermal.lock);
	return scnprintf(buffer, PAGE_SIZE,
		"ns=%llu p_mw=%u e_mw=%u gpu_mw=%u total_mw=%d sustainable_mw=%d allowed_mw=%d credit_mj=%lld filtered_mc=%d p_level=%u e_level=%u gpu_level=%u fault=%d error=%d\n",
		p->last_ns, pmp_thermal.rail_mw[0], pmp_thermal.rail_mw[1],
		pmp_thermal.rail_mw[2], p->total_mw, p->sustainable_mw,
		p->allowed_mw, p->credit_mj, p->filtered_mc,
		p->level[0], p->level[1], p->level[2], p->fault, pmp_thermal.power_error);
}
static const struct kernel_param_ops apple_pmp_power_status_ops = {
	.get = apple_pmp_power_status,
};
module_param_cb(power_status, &apple_pmp_power_status_ops, NULL, 0400);

static const smc_key apple_pmp_power_keys[] = {
	SMC_KEY(PP0b), SMC_KEY(PPbb), SMC_KEY(PP1b),
};

static int apple_pmp_power_read_rails(unsigned int mw[3])
{
	struct device_node *np;
	struct platform_device *pdev;
	struct device *dev;
	struct apple_smc *smc;
	unsigned int i;
	int ret = -EPROBE_DEFER;

	np = of_find_compatible_node(NULL, NULL, "apple,t8132-smc");
	if (!np)
		return -ENODEV;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	dev = &pdev->dev;
	/* A device reference alone does not pin its driver's private data.
	 * Serialize every transaction with supplier unbind and fetch drvdata
	 * only while the bound supplier is locked. Do not cache the SMC pointer.
	 */
	device_lock(dev);
	smc = dev_get_drvdata(dev);
	if (!dev->driver || !smc)
		goto out;
	for (i = 0; i < ARRAY_SIZE(apple_pmp_power_keys); i++) {
		struct apple_smc_key_info info;
		__le32 bits;
		int bytes;

		ret = apple_smc_get_key_info(smc, apple_pmp_power_keys[i], &info);
		if (ret)
			goto out;
		if (info.size != 4 || info.type_code != _SMC_KEY("flt ") ||
		    !(info.flags & APPLE_SMC_READABLE)) {
			ret = -EINVAL;
			goto out;
		}
		bytes = apple_smc_read(smc, apple_pmp_power_keys[i], &bits,
				       sizeof(bits));

		if (bytes != sizeof(bits)) {
			ret = bytes < 0 ? bytes : -EIO;
			goto out;
		}
		if (!apple_pmp_power_decode(le32_to_cpu(bits), &mw[i])) {
			ret = -ERANGE;
			goto out;
		}
	}
	ret = 0;
out:
	device_unlock(dev);
	put_device(dev);
	return ret;
}

/* SMC transactions never run in the 5 ms raw-protection worker or while its
 * mutex is held. A stalled SMC read expires both CPU/GPU budgets independently.
 */
static void apple_pmp_power_work(struct work_struct *work)
{
	struct apple_pmp_power_policy *p = &pmp_thermal.power;
	unsigned int mw[3] = {}, active = BIT(0) | BIT(1);
	/* Tag the read before sleeping in SMC. A replacement consumer must wait
	 * for a read started in its own epoch, even if detach/attach was brief.
	 */
	u64 epoch = apple_pmp_gpu_epoch(pmp_thermal.sensors);
	bool dark = false;
	u64 now;
	int ret = apple_pmp_power_read_rails(mw);
	/* The running reader cannot read zero on both CPU rails. A firmware
	 * reset/all-zero image must not refill the energy bucket; an idle
	 * system may still draw well under 0.1 W on all three rails.
	 */
	if (!ret && !mw[0] && !mw[1])
		ret = -ENODATA;
	/* Clock under the lock: a raw sample published while the SMC reads ran
	 * must not make the unsigned age below wrap into a false -EAGAIN.
	 */
	mutex_lock(&pmp_thermal.lock);
	now = ktime_get_ns();
	/*
	 * Every sensor dark (-ENODATA) is not a telemetry fault: the clusters are
	 * powered down, the rails were read, and the thermal guard already holds
	 * the floor for that sample. Skip this budget sample instead of resetting
	 * every budget to full restriction, which then releases one step per
	 * second and keeps a CPU that just woke up near its minimum for tens of
	 * seconds. The GPU gets full protection for the dark sample instead.
	 * A dark span longer than APPLE_PMP_POWER_STALE_NS still fails closed:
	 * the guard and the GPU treat the budget as stale, and the first
	 * sample after the span takes the policy's gap path.
	 */
	if (!ret && pmp_thermal.enabled && pmp_thermal.cached_error == -ENODATA &&
	    now - pmp_thermal.cooling.last_ns <= 500000000ULL)
		dark = true;
	else if (!ret && (!pmp_thermal.enabled || pmp_thermal.cached_error ||
			  now - pmp_thermal.cooling.last_ns > 500000000ULL))
		ret = -EAGAIN;
	pmp_thermal.power_error = ret;
	if (!ret)
		memcpy(pmp_thermal.rail_mw, mw, sizeof(mw));
	if (pmp_thermal.gpu_epoch != epoch) {
		pmp_thermal.gpu_epoch = epoch;
		p->level[2] = 8;
	}
	if (epoch)
		active |= BIT(2);
	apple_pmp_power_set_active(p, active);
	if (dark) {
		/*
		 * Nothing else restricts the GPU's budget while the sensors are
		 * dark: a zero publication selects full protection, and the next
		 * real budget sample republishes its level at once.
		 */
		apple_pmp_gpu_publish(pmp_thermal.sensors, epoch, 8, 0);
	} else {
		apple_pmp_power_sample(p, now, mw, pmp_thermal.cached_mc, ret);
		apple_pmp_gpu_publish(pmp_thermal.sensors, epoch, p->level[2],
				      active & BIT(2) ? p->last_ns : 0);
	}
	pr_debug("apple-pmp-thermal: power ns=%llu mc=%ld p_mw=%u e_mw=%u gpu_mw=%u total_mw=%d sustainable_mw=%d allowed_mw=%d credit_mj=%lld p_level=%u e_level=%u gpu_level=%u error=%d\n",
		now, pmp_thermal.cached_mc, mw[0], mw[1], mw[2], p->total_mw,
		p->sustainable_mw, p->allowed_mw, p->credit_mj,
		p->level[0], p->level[1], p->level[2], ret);
	mutex_unlock(&pmp_thermal.lock);
	if (ret)
		pr_warn_ratelimited("apple-pmp-thermal: power telemetry error %d, retaining minimum CPU/GPU ceilings\n",
				    ret);
	schedule_delayed_work(&pmp_thermal.power_work, msecs_to_jiffies(500));
}

static umode_t apple_pmp_power_visible(const void *data,
		enum hwmon_sensor_types type, u32 attr, int channel)
{
	return type == hwmon_power && channel >= 0 && channel < 4 ? 0444 : 0;
}

static int apple_pmp_power_read(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long *value)
{
	guard(mutex)(&pmp_thermal.lock);
	if (type != hwmon_power || attr != hwmon_power_input || channel < 0 || channel > 3)
		return -EINVAL;
	if (pmp_thermal.power.fault || pmp_thermal.power_error ||
	    ktime_get_ns() - pmp_thermal.power.last_ns > APPLE_PMP_POWER_STALE_NS)
		return -EAGAIN;
	*value = 1000L * (channel == 3 ? pmp_thermal.power.total_mw : pmp_thermal.rail_mw[channel]);
	return 0;
}

static int apple_pmp_power_label(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, const char **label)
{
	static const char * const labels[] = {
		"P CPU rail (PP0b)", "E CPU rail (PPbb)", "GPU rail (PP1b)",
		"CPU/GPU rail sum (budgeted)",
	};
	if (type != hwmon_power || channel < 0 || channel > 3)
		return -EINVAL;
	*label = labels[channel];
	return 0;
}
static const struct hwmon_ops apple_pmp_power_ops = {
	.is_visible = apple_pmp_power_visible, .read = apple_pmp_power_read,
	.read_string = apple_pmp_power_label,
};
static const struct hwmon_channel_info * const apple_pmp_power_info[] = {
	HWMON_CHANNEL_INFO(power, HWMON_P_INPUT | HWMON_P_LABEL,
		HWMON_P_INPUT | HWMON_P_LABEL, HWMON_P_INPUT | HWMON_P_LABEL,
		HWMON_P_INPUT | HWMON_P_LABEL), NULL,
};
static const struct hwmon_chip_info apple_pmp_power_chip = {
	.ops = &apple_pmp_power_ops, .info = apple_pmp_power_info,
};


/* Retain the actual samples used for the decision, including invalid or
 * unavailable channels, for the critical log. A later read could miss the spike.
 */
struct apple_pmp_snapshot {
	u64 started_ns;
	u64 finished_ns;
	long values[PMP_TEMP_CHANNELS];
	int errors[PMP_TEMP_CHANNELS];
};

static int apple_pmp_sample_all(struct apple_pmp_snapshot *snapshot,
				  long *value)
{
	bool found = false;
	int first_error = 0;
	unsigned int i;

	memset(snapshot, 0, sizeof(*snapshot));
	snapshot->started_ns = ktime_get_ns();
	for (i = 0; i < PMP_TEMP_CHANNELS; i++) {
		int ret = pmp_thermal.sensors->read(pmp_thermal.sensors->cookie, i,
						&snapshot->values[i]);

		snapshot->errors[i] = ret;
		if (ret) {
			if (ret != -ENODATA && !first_error)
				first_error = ret;
			continue;
		}
		if (!found || snapshot->values[i] > *value)
			*value = snapshot->values[i];
		found = true;
	}
	snapshot->finished_ns = ktime_get_ns();
	return first_error ?: (found ? 0 : -ENODATA);
}

/* Logged at most twice, from the poll worker, before the shutdown request. */
static void apple_pmp_critical_dump(const struct apple_pmp_snapshot *snapshot,
				    const char *reason, long value, int error)
{
	unsigned int i;

	pr_emerg("apple-pmp-thermal: %s hotspot_mc=%ld error=%d sample_start_ns=%llu sample_end_ns=%llu\n",
		 reason, value, error, snapshot->started_ns, snapshot->finished_ns);
	for (i = 0; i < PMP_TEMP_CHANNELS; i++)
		pr_emerg("apple-pmp-thermal: %s mc=%ld error=%d\n",
			 apple_pmp_temp_records[i].name, snapshot->values[i],
			 snapshot->errors[i]);
	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		struct cpufreq_policy *policy = pmp_thermal.clusters[i].policy;

		if (policy)
			pr_emerg("apple-pmp-thermal: cpu%u min_khz=%u max_khz=%u\n",
				 policy->cpu, policy->min, policy->max);
	}
}

/* Fail closed independently of the governor, including while the zone is off. */
static int apple_pmp_guard(unsigned int level)
{
	unsigned int i;
	int ret, err = 0;

	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		struct apple_pmp_cluster *c = &pmp_thermal.clusters[i];

		unsigned int low = c->policy->cpuinfo.min_freq;
		unsigned int high = c->policy->cpuinfo.max_freq;
		unsigned int power_level = pmp_thermal.power.level[i];
		unsigned int effective;

		if (!pmp_thermal.power.last_ns || ktime_get_ns() -
		    pmp_thermal.power.last_ns > APPLE_PMP_POWER_STALE_NS)
			power_level = 8;
		effective = max(level, power_level);

		/* QoS composes with user caps and governor cooling. Intermediate
		 * requests are rounded down to supported OPPs by cpufreq.
		 */
		ret = freq_qos_update_request(&c->guard, effective ?
			low + (high - low) * (8 - effective) / 8 :
			FREQ_QOS_MAX_DEFAULT_VALUE);
		if (ret < 0)
			err = ret;
	}
	if (!err)
		pmp_thermal.guard_level = level;
	return err;
}

static int apple_pmp_sample(void)
{
	struct apple_pmp_snapshot snapshot;
	struct apple_pmp_cooling *cooling = &pmp_thermal.cooling;
	long value = 0;
	bool was_confirmed;
	unsigned int old_level, level, i;
	int old_reason, channel = -1, ret, guard_ret;
	const char *reason;

	guard(mutex)(&pmp_thermal.lock);
	old_level = pmp_thermal.guard_level;
	old_reason = cooling->reason;
	ret = apple_pmp_sample_all(&snapshot, &value);
	if (!ret) {
		pmp_thermal.valid_ns = snapshot.finished_ns;
		pmp_thermal.valid_mc = value;
	} else if (ret == -ENODATA && pmp_thermal.valid_ns &&
		   snapshot.finished_ns - pmp_thermal.valid_ns < APPLE_PMP_GATED_HOLD_NS &&
		   pmp_thermal.valid_mc < APPLE_PMP_GATED_HOLD_MC) {
		value = pmp_thermal.valid_mc;
		ret = 0;
	}
	was_confirmed = pmp_thermal.critical.confirmed;
	apple_pmp_confirmation_sample(&pmp_thermal.critical, value, ret,
				      snapshot.finished_ns);
	apple_pmp_cooling_sample(cooling, snapshot.finished_ns, value, ret);
	/* Capture governor demand before it releases a trip; the independent
	 * QoS envelope then restores CPU capacity gradually. Convert the cooling
	 * device's own state, not policy->max (which includes this guard).
	 */
	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		struct thermal_cooling_device *cdev = pmp_thermal.clusters[i].cdev;
		unsigned long state;

		if (cdev && cdev->max_state && !cdev->ops->get_cur_state(cdev, &state))
			apple_pmp_cooling_observe(cooling, snapshot.finished_ns,
				DIV_ROUND_UP(state * 8, cdev->max_state));
	}
	level = apple_pmp_cooling_level(cooling);
	WRITE_ONCE(pmp_thermal.fast_poll, apple_pmp_cooling_fast(cooling, value, ret));
	/* Never release startup/fault protection with disabled or unbound cooling. */
	if (!pmp_thermal.enabled)
		level = 8;
	guard_ret = apple_pmp_guard(level);
	if (guard_ret)
		apple_pmp_guard(8);
	for (i = 0; i < PMP_TEMP_CHANNELS; i++)
		if (!snapshot.errors[i] && snapshot.values[i] == value) {
			channel = i;
			break;
		}
	switch (cooling->reason) {
	case 0:
		reason = "startup";
		break;
	case 1:
		reason = "sensor";
		break;
	case 2:
		reason = "measured-heat";
		break;
	case 3:
		reason = "prediction";
		break;
	case 5:
		reason = "recovery";
		break;
	default:
		reason = "governor";
		break;
	}
	if (old_level != pmp_thermal.guard_level || old_reason != cooling->reason)
		pr_info_ratelimited("apple-pmp-thermal: decision ns=%llu mc=%ld channel=%d error=%d previous_mc=%ld dt_ns=%llu projected_mc=%ld reason=%s hard=%d prediction=%u level=%u qos_error=%d\n",
			snapshot.finished_ns, value, channel, ret, cooling->previous_mc,
			cooling->dt_ns, cooling->projected_mc, reason, cooling->hard,
			cooling->prediction, pmp_thermal.guard_level, guard_ret);
	pmp_thermal.cached_mc = value;
	pmp_thermal.cached_error = ret ?: guard_ret;
	if (ret) {
		pr_warn_ratelimited("apple-pmp-thermal: sensor error %d; CPU minimum caps retained\n",
				    ret);
		return ret;
	}
	ret = guard_ret;
	/* First real critical sample, before core poweroff. */
	if (value >= APPLE_PMP_CRITICAL_MC && !pmp_thermal.critical_dumped) {
		pmp_thermal.critical_dumped = true;
		apple_pmp_critical_dump(&snapshot, "critical-sample", value, 0);
	}
	if (!was_confirmed && pmp_thermal.critical.confirmed)
		apple_pmp_critical_dump(&snapshot, "critical-confirmed-third-sample", value, 0);
	return ret;
}

/* Thermal-core/sysfs reads observe the last raw sample without changing the
 * sample count, QoS guard, or governor cadence. The fast worker owns sampling.
 */
static int apple_pmp_get_temp(struct thermal_zone_device *zone, int *temp)
{
	guard(mutex)(&pmp_thermal.lock);
	if (!pmp_thermal.cooling.last_ns ||
	    ktime_get_ns() - pmp_thermal.cooling.last_ns > 500000000ULL)
		return -EAGAIN;
	if (pmp_thermal.cached_error)
		return pmp_thermal.cached_error;
	*temp = pmp_thermal.cached_mc;
	return 0;
}

/* The thermal core calls .critical only on upward crossings. The poll
 * worker also checks after every raw sample so sustained heat reaches sample 3.
 * Preserve the raw temperature and the normal thermal-core poweroff path.
 */
static void apple_pmp_critical(struct thermal_zone_device *zone)
{
	bool shutdown = false;

	mutex_lock(&pmp_thermal.lock);
	if (pmp_thermal.critical.confirmed && !pmp_thermal.shutdown_requested) {
		pmp_thermal.shutdown_requested = true;
		shutdown = true;
	}
	mutex_unlock(&pmp_thermal.lock);
	if (shutdown) {
		pr_emerg("apple-pmp-thermal: three >=120 C samples at least 5 ms apart; requesting shutdown\n");
		thermal_zone_device_critical(zone);
	}
}

static void apple_pmp_poll_work(struct work_struct *work)
{
	u64 now;

	apple_pmp_sample();
	pmp_thermal.sample_count++;
	apple_pmp_critical(pmp_thermal.zone);
	now = ktime_get_ns();
	if (now - pmp_thermal.governor_ns >= APPLE_PMP_GOVERNOR_NS) {
		pmp_thermal.governor_ns = now;
		pmp_thermal.governor_count++;
		thermal_zone_device_update(pmp_thermal.zone, THERMAL_EVENT_UNSPECIFIED);
	}
}

static enum hrtimer_restart apple_pmp_poll_timer(struct hrtimer *timer)
{
	/* Sampling/QoS may sleep, so never perform either in timer context.
	 * One work item coalesces missed ticks; no burst of stale confirmations.
	 */
	queue_work(system_highpri_wq, &pmp_thermal.poll_work);
	hrtimer_forward_now(timer, ms_to_ktime(READ_ONCE(pmp_thermal.fast_poll) ?
					     APPLE_PMP_POLL_MS :
					     APPLE_PMP_SLOW_POLL_MS));
	return HRTIMER_RESTART;
}

static int apple_pmp_change_mode(struct thermal_zone_device *zone,
				 enum thermal_device_mode mode)
{
	guard(mutex)(&pmp_thermal.lock);
	pmp_thermal.enabled = mode == THERMAL_DEVICE_ENABLED;
	apple_pmp_gpu_publish(pmp_thermal.sensors, 0, 8, 0);
	apple_pmp_cooling_init(&pmp_thermal.cooling);
	WRITE_ONCE(pmp_thermal.fast_poll, true);
	return apple_pmp_guard(8);
}

static bool apple_pmp_should_bind(struct thermal_zone_device *zone,
				  const struct thermal_trip *trip,
				  struct thermal_cooling_device *cdev,
				  struct cooling_spec *spec)
{
	unsigned int i;

	if (trip->type != THERMAL_TRIP_PASSIVE)
		return false;
	/* Match the actual PMP supplier, never an unrelated cooling-device name. */
	if (of_device_is_compatible(cdev->np, "apple,agx-t8132")) {
		struct device_node *np = of_parse_phandle(cdev->np, "apple,pmp", 0);
		bool match = np && np == pmp_thermal.sensors->dev->of_node;

		of_node_put(np);
		if (match) {
			spec->lower = 0;
			spec->upper = cdev->max_state;
			return true;
		}
	}

	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		struct apple_pmp_cluster *c = &pmp_thermal.clusters[i];

		if (cdev != c->cdev)
			continue;
		spec->lower = 0;
		spec->upper = cdev->max_state;
		return true;
	}
	return false;
}

static const struct thermal_zone_device_ops apple_pmp_thermal_ops = {
	.get_temp = apple_pmp_get_temp,
	.critical = apple_pmp_critical,
	.change_mode = apple_pmp_change_mode,
	.should_bind = apple_pmp_should_bind,
};

static const struct thermal_trip apple_pmp_trips[] = {
	{
		.temperature = APPLE_PMP_PASSIVE_MC,
		.hysteresis = APPLE_PMP_HYST_MC,
		.type = THERMAL_TRIP_PASSIVE,
	},
	{
		.temperature = APPLE_PMP_TARGET_MC,
		.hysteresis = 4000,
		.type = THERMAL_TRIP_PASSIVE,
	},
	{
		.temperature = APPLE_PMP_CRITICAL_MC,
		.type = THERMAL_TRIP_CRITICAL,
	},
};

static const struct thermal_zone_params apple_pmp_params = {
	.governor_name = "step_wise",
	.no_hwmon = true, /* Keep the existing six-channel hwmon ABI. */
};

static int apple_pmp_check_bindings(struct thermal_trip *trip, void *data)
{
	struct thermal_zone_device *zone = data;
	unsigned int i;

	if (trip->type != THERMAL_TRIP_PASSIVE)
		return 0;

	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++)
		if (!thermal_trip_is_bound_to_cdev(zone, trip, pmp_thermal.clusters[i].cdev))
			return -ENODEV;
	return 0;
}

/* Leaves every field reset, so a deferred setup can be attempted again. */
static void apple_pmp_thermal_cleanup(void)
{
	unsigned int i;

	apple_pmp_gpu_publish(pmp_thermal.sensors, 0, 8, 0);
	/* Restore the cpufreq cap before this policy's own guard goes away. */
	if (pmp_thermal.owner) {
		for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++)
			apple_soc_cpufreq_set_thermal_owner(pmp_thermal.clusters[i].policy,
							    false);
		pmp_thermal.owner = false;
	}
	if (pmp_thermal.power_polling) {
		cancel_delayed_work_sync(&pmp_thermal.power_work);
		pmp_thermal.power_polling = false;
	}
	if (pmp_thermal.power_hwmon) {
		hwmon_device_unregister(pmp_thermal.power_hwmon);
		pmp_thermal.power_hwmon = NULL;
	}
	if (pmp_thermal.polling) {
		hrtimer_cancel(&pmp_thermal.poll_timer);
		cancel_work_sync(&pmp_thermal.poll_work);
		pmp_thermal.polling = false;
	}
	/* Drain the publisher before the final invalidation. */
	apple_pmp_gpu_publish(pmp_thermal.sensors, 0, 8, 0);
	if (pmp_thermal.zone) {
		thermal_zone_device_unregister(pmp_thermal.zone);
		pmp_thermal.zone = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		struct apple_pmp_cluster *c = &pmp_thermal.clusters[i];

		cpufreq_cooling_unregister(c->cdev);
		c->cdev = NULL;
		if (freq_qos_request_active(&c->guard))
			freq_qos_remove_request(&c->guard);
		if (c->policy)
			cpufreq_cpu_put(c->policy);
		c->policy = NULL;
	}

	pmp_thermal.enabled = false;
	pmp_thermal.ready = false;
}

static int apple_pmp_add_cluster(unsigned int index, unsigned int cpu,
				 unsigned long members)
{
	struct apple_pmp_cluster *c = &pmp_thermal.clusters[index];
	unsigned int n;
	int ret;

	c->policy = cpufreq_cpu_get(cpu);
	if (!c->policy)
		return -EPROBE_DEFER;
	guard(cpufreq_policy_read)(c->policy);
	for_each_possible_cpu(n)
		if (cpumask_test_cpu(n, c->policy->related_cpus) !=
		    (n < BITS_PER_LONG && !!(members & BIT(n))))
			return -EINVAL;
	if (!c->policy->freq_table)
		return -EPROBE_DEFER;

	ret = freq_qos_add_request(&c->policy->constraints, &c->guard,
				   FREQ_QOS_MAX, c->policy->cpuinfo.min_freq);
	if (ret < 0)
		return ret;
	c->cdev = cpufreq_cooling_register(c->policy);
	if (IS_ERR(c->cdev)) {
		ret = PTR_ERR(c->cdev);
		c->cdev = NULL;
		return ret;
	}
	return 0;
}

static int apple_pmp_thermal_setup(void)
{
	struct thermal_zone_device *zone;
	unsigned long p_members = BIT(0) | GENMASK(9, 7);
	unsigned long e_members = GENMASK(6, 1);
	unsigned int i;
	int ret;

	/* J773g numbers the four P cores 0,3-5 and the six E cores 1-2,6-9. */
	if (of_machine_is_compatible("apple,j773g")) {
		p_members = BIT(0) | GENMASK(5, 3);
		e_members = GENMASK(2, 1) | GENMASK(9, 6);
	}
	apple_pmp_power_init(&pmp_thermal.power);
	apple_pmp_cooling_init(&pmp_thermal.cooling);
	pmp_thermal.guard_level = 8;
	ret = apple_pmp_add_cluster(0, 0, p_members);
	if (ret)
		goto fail;
	ret = apple_pmp_add_cluster(1, 1, e_members);
	if (ret)
		goto fail;
	zone = thermal_zone_device_register_with_trips("apple-pmp-die", apple_pmp_trips,
						       ARRAY_SIZE(apple_pmp_trips), NULL,
						       &apple_pmp_thermal_ops, &apple_pmp_params,
						       0, 0);
	pmp_thermal.zone = zone;
	if (IS_ERR(pmp_thermal.zone)) {
		ret = PTR_ERR(pmp_thermal.zone);
		pmp_thermal.zone = NULL;
		goto fail;
	}
	/* Trip array is immutable; the binding query takes the zone lock itself. */
	ret = for_each_thermal_trip(pmp_thermal.zone, apple_pmp_check_bindings,
				    pmp_thermal.zone);
	if (ret)
		goto fail;
	ret = thermal_zone_device_enable(pmp_thermal.zone);
	if (ret)
		goto fail;
	pmp_thermal.power_hwmon = hwmon_device_register_with_info(
		thermal_zone_device(pmp_thermal.zone),
			"apple_pmp_power", NULL, &apple_pmp_power_chip, NULL);
	if (IS_ERR(pmp_thermal.power_hwmon)) {
		ret = PTR_ERR(pmp_thermal.power_hwmon);
		pmp_thermal.power_hwmon = NULL;
		goto fail;
	}
	pmp_thermal.power_polling = true;
	schedule_delayed_work(&pmp_thermal.power_work, msecs_to_jiffies(500));
	pmp_thermal.polling = true;
	pmp_thermal.fast_poll = true;
	hrtimer_start(&pmp_thermal.poll_timer, ms_to_ktime(APPLE_PMP_POLL_MS),
		      HRTIMER_MODE_REL);
	/* Both guards start at the minimum frequency: hand over the cpufreq cap. */
	pmp_thermal.owner = true;
	for (i = 0; i < ARRAY_SIZE(pmp_thermal.clusters); i++) {
		ret = apple_soc_cpufreq_set_thermal_owner(pmp_thermal.clusters[i].policy,
							  true);
		if (ret)
			goto fail;
	}
	pr_info("apple-pmp-thermal: T8132 provisional policy: CPU/GPU cooling from 106 C, target 110 C, full protection 115 C, three >=120 C samples for shutdown, 5 ms sampling from 95 C or while cooling (100 ms below), 100 ms governor, shared measured-rail power budget\n");
	pmp_thermal.ready = true;
	return 0;
fail:
	apple_pmp_thermal_cleanup();
	return ret;
}

static void apple_pmp_thermal_setup_work(struct work_struct *work)
{
	int ret = apple_pmp_thermal_setup();

	/* cpufreq policy creation is not a driver bind, so it need not retrigger
	 * deferred platform probes. Retry only this startup dependency while
	 * cpufreq keeps its unmanaged QoS ceiling and no ownership is claimed.
	 */
	if (ret == -EPROBE_DEFER)
		schedule_delayed_work(&pmp_thermal.setup_work, HZ);
	else if (ret)
		pr_err("apple-pmp-thermal: setup failed: %d; cpufreq ownership retained\n", ret);
}

static int apple_pmp_thermal_probe(struct platform_device *pdev)
{
	const struct apple_pmp_sensor_data *sensors = dev_get_platdata(&pdev->dev);

	/* The M4 PMP sensor records and the Airs' fanless policy are qualified. */
	if (!apple_t8132_firmware_path())
		return -ENODEV;
	if (!sensors || !sensors->read || !sensors->dev)
		return -EINVAL;
	pmp_thermal.sensors = sensors;
	pmp_thermal.cached_error = -EAGAIN;
	pmp_thermal.power_error = -EAGAIN;
	pmp_thermal.valid_ns = 0;
	pmp_thermal.critical = (struct apple_pmp_confirmation) {};
	pmp_thermal.shutdown_requested = false;
	pmp_thermal.critical_dumped = false;
	pmp_thermal.governor_ns = 0;
	INIT_DELAYED_WORK(&pmp_thermal.setup_work, apple_pmp_thermal_setup_work);
	INIT_DELAYED_WORK(&pmp_thermal.power_work, apple_pmp_power_work);
	INIT_WORK(&pmp_thermal.poll_work, apple_pmp_poll_work);
	hrtimer_setup(&pmp_thermal.poll_timer, apple_pmp_poll_timer,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	schedule_delayed_work(&pmp_thermal.setup_work, 0);
	return 0;
}

/*
 * A deep sleep stops the PMP: sampling on through it would read its sensors stopped and
 * restrict the CPUs and the GPU for seconds after the wake. Nothing runs to heat the die
 * from the first device callback to the last, so neither sampler runs in between.
 */
static int apple_pmp_thermal_prepare(struct device *dev)
{
	if (!pm_suspend_via_firmware())
		return 0;
	if (pmp_thermal.polling) {
		hrtimer_cancel(&pmp_thermal.poll_timer);
		cancel_work_sync(&pmp_thermal.poll_work);
	}
	if (pmp_thermal.power_polling)
		cancel_delayed_work_sync(&pmp_thermal.power_work);
	pmp_thermal.sleep_paused = true;
	return 0;
}

static void apple_pmp_thermal_complete(struct device *dev)
{
	struct apple_pmp_power_policy *p = &pmp_thermal.power;

	if (!pmp_thermal.sleep_paused)
		return;
	pmp_thermal.sleep_paused = false;
	scoped_guard(mutex, &pmp_thermal.lock) {
		/* The sleep is no telemetry gap, nor cool time to refill the bucket. */
		if (p->last_ns)
			p->last_ns = ktime_get_ns();
		if (p->active & BIT(2))
			apple_pmp_gpu_publish(pmp_thermal.sensors, pmp_thermal.gpu_epoch,
					      p->level[2], p->last_ns);
	}
	if (pmp_thermal.power_polling)
		schedule_delayed_work(&pmp_thermal.power_work, msecs_to_jiffies(500));
	if (pmp_thermal.polling)
		hrtimer_start(&pmp_thermal.poll_timer, ms_to_ktime(APPLE_PMP_POLL_MS),
			      HRTIMER_MODE_REL);
}

static const struct dev_pm_ops apple_pmp_thermal_pm_ops = {
	.prepare = apple_pmp_thermal_prepare,
	.complete = apple_pmp_thermal_complete,
};

static void apple_pmp_thermal_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&pmp_thermal.setup_work);
	apple_pmp_thermal_cleanup();
}

static struct platform_driver apple_pmp_thermal_driver = {
	.probe = apple_pmp_thermal_probe,
	.remove = apple_pmp_thermal_remove,
	.driver = {
		.name = "apple-pmp-thermal",
		.pm = pm_sleep_ptr(&apple_pmp_thermal_pm_ops),
	},
};
module_platform_driver(apple_pmp_thermal_driver);
MODULE_ALIAS("platform:apple-pmp-thermal");

MODULE_DESCRIPTION("Apple J713 PMP package CPU/GPU thermal policy");
MODULE_LICENSE("Dual MIT/GPL");
