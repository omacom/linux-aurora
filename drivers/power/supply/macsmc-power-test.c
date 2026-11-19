// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Included by macsmc-power.c. The first suite stubs the CHTE reader; the
 * charge suite replaces apple_smc_read() and apple_smc_write() with a model
 * of the charge keys that can fail any one access.
 */
#include <kunit/test.h>
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/kthread.h>

struct macsmc_power_test_context {
	struct macsmc_power power;
	int result;
};

static int macsmc_power_test_read_chte(struct macsmc_power *power, u8 *buf)
{
	struct macsmc_power_test_context *ctx =
		container_of(power, struct macsmc_power_test_context, power);

	memset(buf, 0xa5, 4);
	buf[0] = 1;
	return ctx->result;
}

static int macsmc_power_test_init(struct kunit *test)
{
	struct macsmc_power_test_context *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_init(&ctx->power.charge_lock);
	ctx->power.has_chte = true;
	test->priv = ctx;
	kunit_activate_static_stub(test, macsmc_power_read_chte,
				   macsmc_power_test_read_chte);
	return 0;
}

static void macsmc_power_short_chte_test(struct kunit *test)
{
	struct macsmc_power_test_context *ctx = test->priv;

	ctx->result = 0;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&ctx->power), -EIO);
	ctx->result = 2;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&ctx->power), -EIO);
	ctx->result = 4;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&ctx->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);
}

static struct kunit_case macsmc_power_test_cases[] = {
	KUNIT_CASE(macsmc_power_short_chte_test),
	{}
};

static struct kunit_suite macsmc_power_test_suite = {
	.name = "macsmc-power",
	.init = macsmc_power_test_init,
	.test_cases = macsmc_power_test_cases,
};

/* Charge settings against a model of the SMC's charge keys. */

#define CHARGE_TEST_MAX_OPS 32

struct macsmc_charge_op {
	smc_key key;
	bool write;
	u32 value;
};

struct charge_test_writer;

struct macsmc_charge_test {
	struct kunit *test;
	struct apple_smc smc;
	struct macsmc_power power;
	struct power_supply psy;

	/* The keys' values */
	u16 chls;
	u8 ch0i;
	u8 ch0c;
	u32 chte;

	/* Fail the access to fail_key (a write if fail_write) with fail_err */
	smc_key fail_key;
	bool fail_write;
	int fail_err;

	/* Every access, in order */
	struct macsmc_charge_op ops[CHARGE_TEST_MAX_OPS];
	unsigned int nops;

	/* Charge-key accesses must hold charge_lock (set by the tests) */
	bool expect_locked;

	/* Runs once from within the next CHLS read, after the value is taken */
	void (*on_chls_read)(struct macsmc_charge_test *t);

	/* The concurrent writer of the serialization test */
	struct charge_test_writer *writer;
	bool writer_early;
	/* The test's own task; the writer records its findings for it to report */
	struct task_struct *task;
	atomic_t writer_unlocked;
};

static bool charge_test_key_is_charge_setting(smc_key key)
{
	return key == SMC_KEY(CHLS) || key == SMC_KEY(CH0I) ||
	       key == SMC_KEY(CH0C) || key == SMC_KEY(CHTE);
}

static void charge_test_log(struct macsmc_charge_test *t, smc_key key, bool write,
			    u32 value)
{
	bool unlocked = t->expect_locked && charge_test_key_is_charge_setting(key) &&
			!mutex_is_locked(&t->power.charge_lock);

	/* KUnit reports only from the test's task; the writer leaves it a count. */
	if (current != t->task) {
		if (unlocked)
			atomic_inc(&t->writer_unlocked);
		return;
	}
	KUNIT_EXPECT_FALSE_MSG(t->test, unlocked, "charge key %08x accessed without charge_lock",
			       key);
	if (t->nops >= CHARGE_TEST_MAX_OPS) {
		KUNIT_FAIL(t->test, "more than %d charge key accesses", CHARGE_TEST_MAX_OPS);
		return;
	}
	t->ops[t->nops++] = (struct macsmc_charge_op){ key, write, value };
}

static int charge_test_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	struct macsmc_charge_test *t = container_of(smc, struct macsmc_charge_test, smc);
	void (*hook)(struct macsmc_charge_test *t) = NULL;
	u32 value;

	if (key == SMC_KEY(CHLS) && size == sizeof(u16)) {
		value = t->chls;
		hook = t->on_chls_read;
		t->on_chls_read = NULL;
	} else if (key == SMC_KEY(CH0I) && size == sizeof(u8)) {
		value = t->ch0i;
	} else if (key == SMC_KEY(CH0C) && size == sizeof(u8)) {
		value = t->ch0c;
	} else if (key == SMC_KEY(CHTE) && size == sizeof(u32)) {
		value = t->chte;
	} else {
		KUNIT_FAIL(t->test, "unexpected read of %08x, %zu bytes", key, size);
		return -EINVAL;
	}

	charge_test_log(t, key, false, value);
	if (t->fail_key == key && !t->fail_write)
		return t->fail_err;
	/* Little-endian, like the SMC's own replies */
	memcpy(buf, &value, size);
	/* Another writer gets its turn after this read has taken the value */
	if (hook)
		hook(t);
	return size;
}

static int charge_test_write(struct apple_smc *smc, smc_key key, const void *buf,
			     size_t size)
{
	struct macsmc_charge_test *t = container_of(smc, struct macsmc_charge_test, smc);
	u32 value = 0;

	memcpy(&value, buf, min(size, sizeof(value)));
	charge_test_log(t, key, true, value);
	if (t->fail_key == key && t->fail_write)
		return t->fail_err;

	if (key == SMC_KEY(CHLS) && size == sizeof(u16))
		t->chls = value;
	else if (key == SMC_KEY(CH0I) && size == sizeof(u8))
		t->ch0i = value;
	else if (key == SMC_KEY(CH0C) && size == sizeof(u8))
		t->ch0c = value;
	else if (key == SMC_KEY(CHTE) && size == sizeof(u32))
		t->chte = value;
	else
		KUNIT_FAIL(t->test, "unexpected write of %08x, %zu bytes", key, size);
	/* An SMC write reply may echo the request length; it is not an error. */
	return size;
}

static int macsmc_charge_test_init(struct kunit *test)
{
	struct macsmc_charge_test *t;

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->test = test;
	t->task = current;
	atomic_set(&t->writer_unlocked, 0);
	t->power.smc = &t->smc;
	mutex_init(&t->power.charge_lock);
	t->psy.drv_data = &t->power;
	t->expect_locked = true;
	test->priv = t;
	kunit_activate_static_stub(test, apple_smc_read, charge_test_read);
	kunit_activate_static_stub(test, apple_smc_write, charge_test_write);
	return 0;
}

static int charge_test_set(struct macsmc_charge_test *t, enum power_supply_property psp,
			   int value)
{
	union power_supply_propval val = { .intval = value };

	return macsmc_battery_set_property(&t->psy, psp, &val);
}

static int charge_test_get(struct macsmc_charge_test *t, enum power_supply_property psp,
			   int *value)
{
	union power_supply_propval val = {};
	int ret = macsmc_battery_get_property(&t->psy, psp, &val);

	*value = val.intval;
	return ret;
}

static unsigned int charge_test_writes(struct macsmc_charge_test *t)
{
	unsigned int i, n = 0;

	for (i = 0; i < t->nops; i++)
		n += t->ops[i].write;
	return n;
}

static void charge_test_expect_op(struct kunit *test, struct macsmc_charge_test *t,
				  unsigned int i, smc_key key, bool write, u32 value)
{
	if (i >= t->nops) {
		KUNIT_FAIL(test, "access %u missing, only %u", i, t->nops);
		return;
	}
	KUNIT_EXPECT_EQ_MSG(test, t->ops[i].key, key, "access %u", i);
	KUNIT_EXPECT_EQ_MSG(test, t->ops[i].write, write, "access %u", i);
	if (write)
		KUNIT_EXPECT_EQ_MSG(test, t->ops[i].value, value, "access %u", i);
}

/* The behaviours a battery offers follow from its keys alone. */
static void macsmc_charge_behaviours_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;
	struct macsmc_power *p = &t->power;
	unsigned int combo;

	for (combo = 0; combo < 32; combo++) {
		unsigned int got;

		p->has_ch0i = combo & BIT(0);
		p->has_ch0c = combo & BIT(1);
		p->has_chte = combo & BIT(2);
		p->has_chls = combo & BIT(3);
		p->has_chwa = combo & BIT(4);
		got = macsmc_battery_charge_behaviours(p);

		KUNIT_EXPECT_EQ_MSG(test, !!(got & BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO)),
				    p->has_ch0i || p->has_ch0c || p->has_chte || p->has_chls,
				    "combo %u", combo);
		KUNIT_EXPECT_EQ_MSG(test,
				    !!(got & BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE)),
				    p->has_ch0i, "combo %u", combo);
		KUNIT_EXPECT_EQ_MSG(test,
				    !!(got & BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)),
				    p->has_chte || p->has_ch0c, "combo %u", combo);
		KUNIT_EXPECT_EQ_MSG(test,
				    !!(got & BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE)),
				    p->has_chls, "combo %u", combo);
		KUNIT_EXPECT_EQ_MSG(test,
				    got & BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE_AWAKE),
				    0U, "combo %u", combo);
	}

	/* CHWA alone offers no behaviour, as before; CHLS alone offers both autos */
	p->has_ch0i = false;
	p->has_ch0c = false;
	p->has_chte = false;
	p->has_chls = false;
	p->has_chwa = true;
	KUNIT_EXPECT_EQ(test, macsmc_battery_charge_behaviours(p), 0U);
	p->has_chwa = false;
	p->has_chls = true;
	KUNIT_EXPECT_EQ(test, macsmc_battery_charge_behaviours(p),
			(unsigned int)(BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
				       BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE)));
	KUNIT_EXPECT_EQ(test, t->nops, 0U);
}

/* The end threshold keeps the force-discharge bit only while there is a limit. */
static void macsmc_charge_end_threshold_test(struct kunit *test)
{
	static const struct {
		u16 before;
		int threshold;
		u16 after;
	} cases[] = {
		{ 0x0000, 80, 0x0050 },
		{ 0x0150, 70, 0x0146 },	/* auto-discharge stays */
		{ 0x0150, 99, 0x0163 },
		{ 0x0000, 5, 0x000a },	/* below the minimum: the minimum */
		{ 0x0150, 0, 0x010a },
		{ 0x0000, -1, 0x000a },
		{ 0x0350, 60, 0x013c },	/* other high bits are not kept */
		{ 0x0150, 100, 0x0000 },	/* no limit ends auto-discharge */
		{ 0x0050, 100, 0x0000 },
		{ 0x0150, 150, 0x0000 },
	};
	struct macsmc_charge_test *t = test->priv;
	unsigned int i;

	t->power.has_chls = true;
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		int ret;

		t->chls = cases[i].before;
		t->nops = 0;
		ret = charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
				      cases[i].threshold);
		KUNIT_EXPECT_EQ_MSG(test, ret, 0, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, t->chls, cases[i].after, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, t->nops, 2U, "case %u", i);
		charge_test_expect_op(test, t, 0, SMC_KEY(CHLS), false, 0);
		charge_test_expect_op(test, t, 1, SMC_KEY(CHLS), true, cases[i].after);
	}
}

/* A failed CHLS read changes nothing; a failed write is reported. */
static void macsmc_charge_end_threshold_fault_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;

	t->power.has_chls = true;
	t->chls = 0x0150;
	t->fail_key = SMC_KEY(CHLS);
	t->fail_err = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
					      60), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, charge_test_writes(t), 0U);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0150);

	t->nops = 0;
	t->fail_write = true;
	t->fail_err = -EIO;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
					      60), -EIO);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0150);
}

/* auto-discharge clears the inhibitors, then sets the bit next to the limit. */
static void macsmc_charge_auto_discharge_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;
	int value;

	t->power.has_ch0i = true;
	t->power.has_chte = true;
	t->power.has_chls = true;
	t->chls = 0x0050;
	t->ch0i = 1;
	t->chte = 1;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE), 0);
	KUNIT_EXPECT_EQ(test, t->nops, 4U);
	charge_test_expect_op(test, t, 0, SMC_KEY(CHLS), false, 0);
	charge_test_expect_op(test, t, 1, SMC_KEY(CH0I), true, 0);
	charge_test_expect_op(test, t, 2, SMC_KEY(CHTE), true, 0);
	charge_test_expect_op(test, t, 3, SMC_KEY(CHLS), true, 0x0150);
	KUNIT_EXPECT_EQ(test, charge_test_get(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &value), 0);
	KUNIT_EXPECT_EQ(test, value, POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE);

	/* The threshold reads past the bit */
	t->expect_locked = false;
	KUNIT_EXPECT_EQ(test, charge_test_get(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
					      &value), 0);
	KUNIT_EXPECT_EQ(test, value, 80);
	KUNIT_EXPECT_EQ(test, charge_test_get(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
					      &value), 0);
	KUNIT_EXPECT_EQ(test, value, 75);
	t->expect_locked = true;

	/* auto clears it again and keeps the limit */
	t->nops = 0;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO), 0);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0050);
	KUNIT_EXPECT_EQ(test, charge_test_get(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &value), 0);
	KUNIT_EXPECT_EQ(test, value, POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO);
}

/* Without a limit there is nothing to discharge to: refused, nothing written. */
static void macsmc_charge_auto_discharge_no_limit_test(struct kunit *test)
{
	static const u16 no_limit[] = { 0x0000, 0x0009, 0x0064, 0x00ff, 0x0100 };
	struct macsmc_charge_test *t = test->priv;
	unsigned int i;

	t->power.has_ch0i = true;
	t->power.has_chte = true;
	t->power.has_chls = true;
	for (i = 0; i < ARRAY_SIZE(no_limit); i++) {
		t->chls = no_limit[i];
		t->ch0i = 1;
		t->nops = 0;
		KUNIT_EXPECT_EQ_MSG(test,
				    charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
						    POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE),
				    -EINVAL, "chls %04x", no_limit[i]);
		KUNIT_EXPECT_EQ_MSG(test, charge_test_writes(t), 0U, "chls %04x", no_limit[i]);
		KUNIT_EXPECT_EQ(test, t->ch0i, 1);
		KUNIT_EXPECT_EQ(test, t->chls, no_limit[i]);
	}

	/* The lowest and highest limits are limits */
	t->chls = 0x000a;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE), 0);
	KUNIT_EXPECT_EQ(test, t->chls, 0x010a);
	t->chls = 0x0063;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE), 0);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0163);

	/* auto is always allowed, and clears a bit left without a limit */
	t->chls = 0x0100;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO), 0);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0000);
}

/* Without CHLS, auto-discharge is not offered and touches no key. */
static void macsmc_charge_auto_discharge_unsupported_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;

	t->power.has_ch0i = true;
	t->power.has_chte = true;
	t->power.has_chwa = true;
	t->ch0i = 1;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE),
			-EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, t->nops, 0U);
	KUNIT_EXPECT_EQ(test, t->ch0i, 1);
}

/* An unreadable CHLS leaves every inhibitor as it was. */
static void macsmc_charge_auto_chls_read_fault_test(struct kunit *test)
{
	static const int modes[] = {
		POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO,
		POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE,
	};
	struct macsmc_charge_test *t = test->priv;
	unsigned int i;

	t->power.has_ch0i = true;
	t->power.has_chte = true;
	t->power.has_chls = true;
	t->fail_key = SMC_KEY(CHLS);
	t->fail_err = -ETIMEDOUT;
	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		t->chls = 0x0150;
		t->ch0i = 1;
		t->chte = 1;
		t->nops = 0;
		KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
						      modes[i]), -ETIMEDOUT);
		KUNIT_EXPECT_EQ(test, charge_test_writes(t), 0U);
		KUNIT_EXPECT_EQ(test, t->ch0i, 1);
		KUNIT_EXPECT_EQ(test, t->chte, 1U);
		KUNIT_EXPECT_EQ(test, t->chls, 0x0150);
	}
}

/*
 * A failure partway through is reported and stops there. The keys are
 * separate SMC writes, so the ones already written stay written.
 */
static void macsmc_charge_auto_partial_failure_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;

	t->power.has_ch0i = true;
	t->power.has_chte = true;
	t->power.has_chls = true;
	t->chls = 0x0050;
	t->ch0i = 1;
	t->chte = 1;
	t->fail_key = SMC_KEY(CHTE);
	t->fail_write = true;
	t->fail_err = -EIO;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE), -EIO);
	KUNIT_EXPECT_EQ(test, t->ch0i, 0);
	KUNIT_EXPECT_EQ(test, t->chte, 1U);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0050);
	KUNIT_EXPECT_EQ(test, t->nops, 3U);

	t->nops = 0;
	t->fail_key = SMC_KEY(CHLS);
	t->chte = 1;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
					      POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE), -EIO);
	KUNIT_EXPECT_EQ(test, t->chte, 0U);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0050);
}

/* Reported behaviour: CH0I, then the inhibitors, then CHLS; failures propagate. */
static void macsmc_charge_get_precedence_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;

	t->power.has_ch0i = true;
	t->power.has_ch0c = true;
	t->power.has_chls = true;
	t->chls = 0x0150;
	t->ch0i = 1;
	t->ch0c = 1;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE);
	t->ch0i = 0;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE);
	t->ch0c = 0;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE);
	t->chls = 0x0050;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO);
	/* The bit without a limit, as an earlier kernel could leave it, reads as auto */
	t->nops = 0;
	t->chls = 0x0100;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO);
	t->chls = 0x0164;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power),
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO);
	t->fail_key = SMC_KEY(CHLS);
	t->fail_err = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, macsmc_battery_get_charge_behaviour(&t->power), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, charge_test_writes(t), 0U);
}

/*
 * Two writers to CHLS at once: the end threshold's read-modify-write and the
 * behaviour's. The behaviour's writer starts while the threshold's has read
 * CHLS but not yet written it. Serialized, it waits, and both settings
 * survive; without the lock its bit would be lost.
 */
struct charge_test_writer {
	struct macsmc_charge_test *t;
	struct completion done;
	int ret;
};

static int charge_test_writer_fn(void *data)
{
	struct charge_test_writer *w = data;

	current->kunit_test = w->t->test;
	w->ret = charge_test_set(w->t, POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
				 POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO_DISCHARGE);
	current->kunit_test = NULL;
	complete(&w->done);
	return 0;
}

static void charge_test_start_writer(struct macsmc_charge_test *t)
{
	struct charge_test_writer *w = t->writer;
	struct task_struct *task;

	task = kthread_run(charge_test_writer_fn, w, "macsmc-charge-test");
	if (IS_ERR(task)) {
		KUNIT_FAIL(t->test, "no writer thread: %ld", PTR_ERR(task));
		w->ret = PTR_ERR(task);
		complete(&w->done);
		return;
	}
	/* Serialized, the writer cannot finish while this setter runs */
	t->writer_early = wait_for_completion_timeout(&w->done, msecs_to_jiffies(200));
}

static void macsmc_charge_serialized_test(struct kunit *test)
{
	struct macsmc_charge_test *t = test->priv;
	struct charge_test_writer *w;

	w = kunit_kzalloc(test, sizeof(*w), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, w);
	w->t = t;
	init_completion(&w->done);
	t->writer = w;

	t->power.has_chls = true;
	t->chls = 0x0050;
	t->on_chls_read = charge_test_start_writer;
	KUNIT_EXPECT_EQ(test, charge_test_set(t, POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
					      60), 0);
	/*
	 * Always wait for the writer, which uses this test's memory: a writer that never
	 * finishes is left to the KUnit timeout.
	 */
	wait_for_completion(&w->done);
	KUNIT_EXPECT_FALSE(test, t->writer_early);
	KUNIT_EXPECT_EQ(test, atomic_read(&t->writer_unlocked), 0);
	KUNIT_EXPECT_EQ(test, w->ret, 0);
	/* Both the new limit and auto-discharge */
	KUNIT_EXPECT_EQ(test, t->chls, 0x013c);
}

/* Probe clears force-discharge left without a limit, and only that. */
static void macsmc_charge_clear_stray_discharge_test(struct kunit *test)
{
	static const struct {
		u16 before, after;
		unsigned int writes;
	} cases[] = {
		{ 0x0100, 0x0000, 1 },
		{ 0x0164, 0x0064, 1 },
		{ 0x0105, 0x0005, 1 },
		{ 0x0150, 0x0150, 0 },	/* auto-discharge with a limit stays */
		{ 0x0050, 0x0050, 0 },
		{ 0x0000, 0x0000, 0 },
	};
	struct macsmc_charge_test *t = test->priv;
	unsigned int i;

	t->power.has_chls = true;
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		t->chls = cases[i].before;
		t->nops = 0;
		KUNIT_EXPECT_EQ_MSG(test, macsmc_battery_clear_stray_discharge(&t->power), 0,
				    "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, t->chls, cases[i].after, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, charge_test_writes(t), cases[i].writes, "case %u", i);
	}

	t->chls = 0x0100;
	t->fail_key = SMC_KEY(CHLS);
	t->fail_err = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, macsmc_battery_clear_stray_discharge(&t->power), -ETIMEDOUT);
	t->fail_write = true;
	t->fail_err = -EIO;
	KUNIT_EXPECT_EQ(test, macsmc_battery_clear_stray_discharge(&t->power), -EIO);
	KUNIT_EXPECT_EQ(test, t->chls, 0x0100);
}

static struct kunit_case macsmc_charge_test_cases[] = {
	KUNIT_CASE(macsmc_charge_behaviours_test),
	KUNIT_CASE(macsmc_charge_end_threshold_test),
	KUNIT_CASE(macsmc_charge_end_threshold_fault_test),
	KUNIT_CASE(macsmc_charge_auto_discharge_test),
	KUNIT_CASE(macsmc_charge_auto_discharge_no_limit_test),
	KUNIT_CASE(macsmc_charge_auto_discharge_unsupported_test),
	KUNIT_CASE(macsmc_charge_auto_chls_read_fault_test),
	KUNIT_CASE(macsmc_charge_auto_partial_failure_test),
	KUNIT_CASE(macsmc_charge_get_precedence_test),
	KUNIT_CASE(macsmc_charge_serialized_test),
	KUNIT_CASE(macsmc_charge_clear_stray_discharge_test),
	{}
};

static struct kunit_suite macsmc_charge_test_suite = {
	.name = "macsmc-power-charge",
	.init = macsmc_charge_test_init,
	.test_cases = macsmc_charge_test_cases,
};

static void macsmc_wake_insert_test(struct kunit *test)
{
	struct macsmc_power power = { .wakeup_mode = true };

	/* Insertion on each J316C port: far-left, near-left, right */
	KUNIT_EXPECT_TRUE(test, macsmc_power_event_wakes(&power, 0x71130005));
	KUNIT_EXPECT_TRUE(test, macsmc_power_event_wakes(&power, 0x71130105));
	KUNIT_EXPECT_TRUE(test, macsmc_power_event_wakes(&power, 0x71130205));
}

static void macsmc_wake_awake_test(struct kunit *test)
{
	struct macsmc_power power = { .wakeup_mode = false };

	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71130105));
}

static void macsmc_wake_other_events_test(struct kunit *test)
{
	struct macsmc_power power = { .wakeup_mode = true };

	/* Removal and contract changes, seen alongside every plug */
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71130102));
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71130103));
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71130104));
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x7113ff04));
	/* Charger and port charge state, and the lid */
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71010101));
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x71060005));
	KUNIT_EXPECT_FALSE(test, macsmc_power_event_wakes(&power, 0x72030001));
}

static struct kunit_case macsmc_wake_test_cases[] = {
	KUNIT_CASE(macsmc_wake_insert_test),
	KUNIT_CASE(macsmc_wake_awake_test),
	KUNIT_CASE(macsmc_wake_other_events_test),
	{}
};

static struct kunit_suite macsmc_wake_test_suite = {
	.name = "macsmc-power-wake",
	.test_cases = macsmc_wake_test_cases,
};

kunit_test_suites(&macsmc_power_test_suite, &macsmc_charge_test_suite,
		  &macsmc_wake_test_suite);
