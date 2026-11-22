// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * KUnit tests for the Apple SMC hwmon driver: channel numbering of the
 * temperature sensors, which thermal zones depend on, and manual fan control
 * by userspace.
 *
 * Included by macsmc-hwmon.c.
 */

#include <kunit/static_stub.h>
#include <kunit/test.h>

/* What macsmc_hwmon_create_sensor() leaves behind for one node. */
static void macsmc_hwmon_test_add(struct macsmc_hwmon_sensors *temp,
				  const char *key, int create_ret)
{
	struct macsmc_hwmon_sensor *sensor = &temp->sensors[temp->count];

	sensor->macsmc_key = _SMC_KEY(key);
	strscpy_pad(sensor->label, key, sizeof(sensor->label));
	macsmc_hwmon_add_temp_channel(temp, create_ret);
}

/* A node that fails to set up keeps its channel; later channels do not move. */
static void macsmc_hwmon_test_failed_temp_keeps_slot(struct kunit *test)
{
	struct macsmc_hwmon_sensor sensors[3] = {};
	struct macsmc_hwmon hwmon = {
		.temp = { .sensors = sensors },
	};
	u32 configs[ARRAY_SIZE(sensors) + 1] = {};

	macsmc_hwmon_test_add(&hwmon.temp, "Te05", 0);
	macsmc_hwmon_test_add(&hwmon.temp, "Te0S", -EINVAL);
	macsmc_hwmon_test_add(&hwmon.temp, "Tp01", 0);

	KUNIT_ASSERT_EQ(test, hwmon.temp.count, 3);
	KUNIT_EXPECT_EQ(test, sensors[0].macsmc_key, _SMC_KEY("Te05"));
	KUNIT_EXPECT_EQ(test, sensors[2].macsmc_key, _SMC_KEY("Tp01"));

	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_input, 0), 0444);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_input, 1), 0);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_label, 1), 0);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_input, 2), 0444);

	/* A zero entry would end the channel list at the failed sensor. */
	macsmc_hwmon_populate_configs(configs, &hwmon.temp);
	KUNIT_EXPECT_NE(test, configs[0], 0);
	KUNIT_EXPECT_NE(test, configs[1], 0);
	KUNIT_EXPECT_NE(test, configs[2], 0);
	KUNIT_EXPECT_EQ(test, configs[3], 0);
}

static void macsmc_hwmon_test_temp_label(struct kunit *test)
{
	struct macsmc_hwmon_sensor sensors[2] = {};
	struct macsmc_hwmon hwmon = {
		.temp = { .sensors = sensors },
	};

	macsmc_hwmon_test_add(&hwmon.temp, "Tp05", 0);
	sensors[1].macsmc_key = _SMC_KEY("Tp09");
	macsmc_hwmon_add_temp_channel(&hwmon.temp, 0);

	KUNIT_ASSERT_EQ(test, hwmon.temp.count, 2);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_label, 0), 0444);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_label, 1), 0);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_is_visible(&hwmon, hwmon_temp,
						      hwmon_temp_input, 1), 0444);
}

/* The channel a gated die key joins the zone through (see populate_sensors). */
static void macsmc_hwmon_test_find_temp(struct kunit *test)
{
	struct macsmc_hwmon_sensor sensors[3] = {};
	struct macsmc_hwmon hwmon = {
		.temp = { .sensors = sensors },
	};

	macsmc_hwmon_test_add(&hwmon.temp, "Te05", 0);
	macsmc_hwmon_test_add(&hwmon.temp, "Tg0e", -EINVAL);
	macsmc_hwmon_test_add(&hwmon.temp, "Tg0f", 0);

	KUNIT_EXPECT_PTR_EQ(test, macsmc_hwmon_find_temp(&hwmon, SMC_KEY(Tg0f)),
			    &sensors[2]);
	KUNIT_EXPECT_PTR_EQ(test, macsmc_hwmon_find_temp(&hwmon, SMC_KEY(Te05)),
			    &sensors[0]);
	/* A node that failed to set up has no key and takes no die key. */
	KUNIT_EXPECT_NULL(test, macsmc_hwmon_find_temp(&hwmon, SMC_KEY(Tg0e)));
	KUNIT_EXPECT_NULL(test, macsmc_hwmon_find_temp(&hwmon, SMC_KEY(Tp01)));
}

#if IS_ENABLED(CONFIG_THERMAL) && IS_ENABLED(CONFIG_MFD_MACSMC_KUNIT_TEST)
#define DIE_TEST_F4_607	0x40936c8b	/* 4.607f, a GPU key with the GPU off */
#define DIE_TEST_F40	0x42200000	/* 40.0f */
#define DIE_TEST_F45_5	0x42360000	/* 45.5f */
#define DIE_TEST_F50	0x42480000	/* 50.0f */

static u32 die_test_gpu;

static int die_test_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	u32 value;

	switch (key) {
	case __SMC_KEY('T', 'g', '0', 'f'):
		value = die_test_gpu;
		break;
	case __SMC_KEY('T', 'e', '0', '5'):
		value = DIE_TEST_F40;
		break;
	case __SMC_KEY('T', 'p', '0', '1'):
		value = DIE_TEST_F45_5;
		break;
	default:
		return -EINVAL;
	}
	memcpy(buf, &value, min(size, sizeof(value)));
	return size;
}

static void die_test_sensor(struct macsmc_hwmon_sensor *sensor, smc_key key)
{
	sensor->macsmc_key = key;
	sensor->info.type_code = __SMC_KEY('f', 'l', 't', ' ');
	sensor->info.size = 4;
	sensor->die = true;
}

/* A GPU key below its no-data threshold is left out of the SoC die zone. */
static void macsmc_hwmon_test_die_skips_no_data(struct kunit *test)
{
	struct apple_smc smc = {};
	struct macsmc_hwmon_sensor sensors[3] = {};
	struct macsmc_hwmon hwmon = {
		.smc = &smc,
		.temp = { .sensors = sensors, .count = 3 },
	};
	int temp = 0;

	die_test_sensor(&sensors[0], SMC_KEY(Tg0f));
	sensors[0].has_no_data_below = true;
	sensors[0].no_data_below = 10000;
	die_test_sensor(&sensors[1], SMC_KEY(Te05));
	die_test_sensor(&sensors[2], SMC_KEY(Tp01));
	kunit_activate_static_stub(test, apple_smc_read, die_test_read);

	/* GPU off: the placeholder neither fails the zone nor counts. */
	die_test_gpu = DIE_TEST_F4_607;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_die_hottest(&hwmon, &temp), 0);
	KUNIT_EXPECT_EQ(test, temp, 45500);

	/* GPU on and hottest: it counts. */
	die_test_gpu = DIE_TEST_F50;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_die_hottest(&hwmon, &temp), 0);
	KUNIT_EXPECT_EQ(test, temp, 50000);

	/* Only powered-down sensors: no data. */
	hwmon.temp.count = 1;
	die_test_gpu = DIE_TEST_F4_607;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_die_hottest(&hwmon, &temp), -ENODATA);
}
#endif

static struct kunit_case macsmc_hwmon_test_cases[] = {
	KUNIT_CASE(macsmc_hwmon_test_failed_temp_keeps_slot),
	KUNIT_CASE(macsmc_hwmon_test_temp_label),
	KUNIT_CASE(macsmc_hwmon_test_find_temp),
#if IS_ENABLED(CONFIG_THERMAL) && IS_ENABLED(CONFIG_MFD_MACSMC_KUNIT_TEST)
	KUNIT_CASE(macsmc_hwmon_test_die_skips_no_data),
#endif
	{}
};

static struct kunit_suite macsmc_hwmon_test_suite = {
	.name = "macsmc-hwmon",
	.test_cases = macsmc_hwmon_test_cases,
};

/*
 * Manual fan control against a model of the SMC fan keys. In "locked" firmware
 * the fans start in system mode (3) and refuse manual mode until Ftst is 1;
 * Ftst back to 0 returns them to system mode. "Legacy" firmware takes the mode
 * write directly. The model needs the SMC read/write stubs of
 * CONFIG_MFD_MACSMC_KUNIT_TEST.
 */
#define FAN_TEST_F1350	0x44a8c000	/* 1350.0f */
#define FAN_TEST_F2500	0x451c4000	/* 2500.0f */
#define FAN_TEST_F5777	0x45b48800	/* 5777.0f */

struct macsmc_fan_test {
	struct apple_smc smc;
	struct kunit *test;
	struct macsmc_hwmon hwmon;
	struct macsmc_hwmon_fan fans[2];
	bool legacy;
	u8 mode[2];
	u32 target[2];
	u8 ftst;
	int writes;
	int ftst_writes;
	int refused;
	int mode_reads;
	int manual_busy[2];
	int target_writes;
	smc_key fail_key;
};

static int fan_test_index(smc_key key)
{
	return ((key >> 16) & 0xff) - '0';
}

static int fan_test_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	struct macsmc_fan_test *t = container_of(smc, struct macsmc_fan_test, smc);
	u32 value;
	int i = fan_test_index(key);

	switch (key & 0xff00ffff) {
	case __SMC_KEY('F', 0, 'M', 'd'):
		t->mode_reads++;
		value = t->mode[i];
		break;
	case __SMC_KEY('F', 0, 'M', 'n'):
		value = FAN_TEST_F1350;
		break;
	case __SMC_KEY('F', 0, 'M', 'x'):
		value = FAN_TEST_F5777;
		break;
	case __SMC_KEY('F', 0, 'T', 'g'):
		value = t->target[i];
		break;
	default:
		KUNIT_FAIL(t->test, "unexpected read of %08x", key);
		return -EINVAL;
	}
	memcpy(buf, &value, size);
	return size;
}

static int fan_test_write(struct apple_smc *smc, smc_key key, const void *buf, size_t size)
{
	struct macsmc_fan_test *t = container_of(smc, struct macsmc_fan_test, smc);
	u32 value = 0;
	int i = fan_test_index(key);

	t->writes++;
	memcpy(&value, buf, min(size, sizeof(value)));
	if (key == t->fail_key)
		return -EIO;

	if (key == SMC_KEY(Ftst)) {
		t->ftst_writes++;
		t->ftst = value;
		for (i = 0; i < ARRAY_SIZE(t->mode); i++) {
			if (value && t->mode[i] == MACSMC_FAN_MODE_SYSTEM)
				t->mode[i] = MACSMC_FAN_MODE_AUTO;
			else if (!value)
				t->mode[i] = MACSMC_FAN_MODE_SYSTEM;
		}
		return size;
	}

	switch (key & 0xff00ffff) {
	case __SMC_KEY('F', 0, 'M', 'd'):
		if (!t->legacy && t->mode[i] == MACSMC_FAN_MODE_SYSTEM) {
			t->refused++;
			return -EIO;
		}
		if (value == MACSMC_FAN_MODE_MANUAL && t->manual_busy[i]) {
			t->manual_busy[i]--;
			return -EIO;
		}
		t->mode[i] = value;
		return size;
	case __SMC_KEY('F', 0, 'T', 'g'):
		KUNIT_EXPECT_EQ(t->test, t->mode[i], MACSMC_FAN_MODE_MANUAL);
		t->target_writes++;
		t->target[i] = value;
		return size;
	}
	KUNIT_FAIL(t->test, "unexpected write of %08x", key);
	return -EINVAL;
}

static void fan_test_key(struct macsmc_hwmon_sensor *sensor, const char *key, u32 type)
{
	sensor->macsmc_key = _SMC_KEY(key);
	sensor->info.type_code = type;
	sensor->info.flags = APPLE_SMC_READABLE | APPLE_SMC_WRITABLE;
}

static int macsmc_fan_test_init(struct kunit *test)
{
	static const char *const keys[2][5] = {
		{ "F0Ac", "F0Mn", "F0Mx", "F0Tg", "F0Md" },
		{ "F1Ac", "F1Mn", "F1Mx", "F1Tg", "F1Md" },
	};
	const u32 flt = __SMC_KEY('f', 'l', 't', ' '), ui8 = __SMC_KEY('u', 'i', '8', ' ');
	struct macsmc_fan_test *t;
	int i;

	if (!IS_ENABLED(CONFIG_MFD_MACSMC_KUNIT_TEST))
		kunit_skip(test, "needs CONFIG_MFD_MACSMC_KUNIT_TEST for the SMC stubs");

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->test = test;
	for (i = 0; i < 2; i++) {
		fan_test_key(&t->fans[i].now, keys[i][0], flt);
		fan_test_key(&t->fans[i].min, keys[i][1], flt);
		fan_test_key(&t->fans[i].max, keys[i][2], flt);
		fan_test_key(&t->fans[i].set, keys[i][3], flt);
		fan_test_key(&t->fans[i].mode, keys[i][4], ui8);
		t->fans[i].manual_control = true;
		t->mode[i] = MACSMC_FAN_MODE_SYSTEM;
	}
	t->hwmon.smc = &t->smc;
	t->hwmon.fan.fans = t->fans;
	t->hwmon.fan.count = 2;
	t->hwmon.fan.unlock_key = SMC_KEY(Ftst);
	mutex_init(&t->hwmon.fan.lock);
	test->priv = t;
	kunit_activate_static_stub(test, apple_smc_read, fan_test_read);
	kunit_activate_static_stub(test, apple_smc_write, fan_test_write);
	return 0;
}

static void fan_test_released(struct kunit *test, struct macsmc_fan_test *t)
{
	KUNIT_EXPECT_EQ(test, t->ftst, 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_SYSTEM);
	KUNIT_EXPECT_EQ(test, t->mode[1], MACSMC_FAN_MODE_SYSTEM);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 0);
	KUNIT_EXPECT_FALSE(test, t->hwmon.fan.unlocked);
	KUNIT_EXPECT_FALSE(test, t->fans[0].manual);
}

/* Locked firmware: unlock on the first manual request, release on auto. */
static void macsmc_fan_test_unlock_and_release(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->ftst, 1);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_MANUAL);
	KUNIT_EXPECT_EQ(test, t->target[0], FAN_TEST_F2500);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 1);

	/* A second target needs no new unlock. */
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 1);

	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	fan_test_released(test, t);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 2);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
}

/* Targets are clamped to [min, max]; negative ones are refused. */
static void macsmc_fan_test_clamp(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 100), 0);
	KUNIT_EXPECT_EQ(test, t->target[0], FAN_TEST_F1350);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 100000), 0);
	KUNIT_EXPECT_EQ(test, t->target[0], FAN_TEST_F5777);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, -1), -EINVAL);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	fan_test_released(test, t);
}

/* A failed target write hands the fan straight back. */
static void macsmc_fan_test_target_write_fails(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->fail_key = SMC_KEY(F0Tg);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EIO);
	fan_test_released(test, t);
}

/* Two manual fans: the unlock is released only with the last one. */
static void macsmc_fan_test_two_fans(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 1, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 2);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, t->ftst, 1);
	KUNIT_EXPECT_EQ(test, t->mode[1], MACSMC_FAN_MODE_MANUAL);
	/* Removal, shutdown and suspend all restore through this. */
	macsmc_hwmon_fans_restore(&t->hwmon);
	fan_test_released(test, t);
}

/* System mode without an unlock key: fail without attempting a mode write. */
static void macsmc_fan_test_no_unlock_key(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->hwmon.fan.unlock_key = 0;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, t->writes, 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_SYSTEM);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 0);
}

/* A fan without apple,fan-manual-control refuses a target, with no SMC write. */
static void macsmc_fan_test_not_manual(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->fans[0].manual_control = false;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, t->writes, 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_SYSTEM);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 0);
}

/* Firmware that takes the mode directly keeps the mode 0 restore. */
static void macsmc_fan_test_legacy(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->legacy = true;
	t->mode[0] = MACSMC_FAN_MODE_AUTO;
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_MANUAL);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_AUTO);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 0);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
}

/* A legacy write-only mode key keeps working without a new read requirement. */
static void macsmc_fan_test_legacy_write_only(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->legacy = true;
	t->mode[0] = MACSMC_FAN_MODE_AUTO;
	t->fans[0].mode.info.flags &= ~APPLE_SMC_READABLE;
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_MANUAL);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_AUTO);
	KUNIT_EXPECT_EQ(test, t->mode_reads, 0);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 0);
}

/* Suspend, resume and shutdown do not write untouched fan keys. */
static void macsmc_fan_test_untouched(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;
	struct platform_device pdev = {};

	platform_set_drvdata(&pdev, &t->hwmon);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_suspend(&pdev.dev), 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_resume(&pdev.dev), 0);
	macsmc_hwmon_shutdown(&pdev);
	KUNIT_EXPECT_EQ(test, t->writes, 0);
	fan_test_released(test, t);
}

/* Suspend releases a manual target; resume never reapplies it. */
static void macsmc_fan_test_suspend_manual(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;
	struct device dev = {};
	int writes;

	dev_set_drvdata(&dev, &t->hwmon);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_suspend(&dev), 0);
	fan_test_released(test, t);
	writes = t->writes;
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EBUSY);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_resume(&dev), 0);
	KUNIT_EXPECT_EQ(test, t->writes, writes);
	fan_test_released(test, t);
	/* A fresh userspace request is still accepted after resume. */
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
}

/* A failed unlock leaves the fan with the firmware. */
static void macsmc_fan_test_unlock_fails(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->fail_key = SMC_KEY(Ftst);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EIO);
	fan_test_released(test, t);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
}

/* A stopped fan may leave mode 3 before the manual-mode write is ready. */
static void macsmc_fan_test_stopped_manual_busy(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	KUNIT_ASSERT_EQ(test, t->target[0], 0);
	t->manual_busy[0] = 2;
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->manual_busy[0], 0);
	KUNIT_EXPECT_EQ(test, t->target[0], FAN_TEST_F2500);
	KUNIT_EXPECT_EQ(test, t->target_writes, 1);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 1);
	/* Unlock, three mode attempts, then the target. No locked-mode write. */
	KUNIT_EXPECT_EQ(test, t->writes, 5);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	fan_test_released(test, t);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 2);
}

/* Exhausting the post-unlock retries must release Ftst, without a target. */
static void macsmc_fan_test_manual_busy_timeout(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->fail_key = SMC_KEY(F0Md);
	KUNIT_EXPECT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), -EIO);
	fan_test_released(test, t);
	KUNIT_EXPECT_GT(test, t->writes, 3);
	KUNIT_EXPECT_EQ(test, t->target_writes, 0);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 2);
	KUNIT_EXPECT_EQ(test, t->refused, 0);
}

/* Taking over an already spinning firmware fan needs no unlock or retry. */
static void macsmc_fan_test_spinning(struct kunit *test)
{
	struct macsmc_fan_test *t = test->priv;

	t->mode[0] = MACSMC_FAN_MODE_AUTO;
	t->target[0] = FAN_TEST_F1350;
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 2500), 0);
	KUNIT_EXPECT_EQ(test, t->target[0], FAN_TEST_F2500);
	KUNIT_EXPECT_EQ(test, t->writes, 2);
	KUNIT_EXPECT_EQ(test, t->ftst_writes, 0);
	KUNIT_ASSERT_EQ(test, macsmc_hwmon_set_fan(&t->hwmon, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, t->mode[0], MACSMC_FAN_MODE_AUTO);
	KUNIT_EXPECT_EQ(test, t->hwmon.fan.manual, 0);
}

static struct kunit_case macsmc_fan_test_cases[] = {
	KUNIT_CASE(macsmc_fan_test_stopped_manual_busy),
	KUNIT_CASE(macsmc_fan_test_manual_busy_timeout),
	KUNIT_CASE(macsmc_fan_test_spinning),
	KUNIT_CASE(macsmc_fan_test_unlock_and_release),
	KUNIT_CASE(macsmc_fan_test_clamp),
	KUNIT_CASE(macsmc_fan_test_target_write_fails),
	KUNIT_CASE(macsmc_fan_test_two_fans),
	KUNIT_CASE(macsmc_fan_test_no_unlock_key),
	KUNIT_CASE(macsmc_fan_test_not_manual),
	KUNIT_CASE(macsmc_fan_test_legacy),
	KUNIT_CASE(macsmc_fan_test_legacy_write_only),
	KUNIT_CASE(macsmc_fan_test_untouched),
	KUNIT_CASE(macsmc_fan_test_suspend_manual),
	KUNIT_CASE(macsmc_fan_test_unlock_fails),
	{}
};

static struct kunit_suite macsmc_fan_test_suite = {
	.name = "macsmc-hwmon-fan",
	.init = macsmc_fan_test_init,
	.test_cases = macsmc_fan_test_cases,
};
kunit_test_suites(&macsmc_hwmon_test_suite, &macsmc_fan_test_suite);
