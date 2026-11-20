// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright The Asahi Linux Contributors
 *
 * CPU idle support for Apple SoCs
 */

#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <asm/cpuidle.h>

enum idle_state {
	STATE_WFI,
	STATE_PWRDOWN,
	STATE_COUNT
};

asm(
	".pushsection .cpuidle.text, \"ax\"\n"
	".type apple_cpu_deep_wfi, @function\n"
	"apple_cpu_deep_wfi:\n"
		"str x30, [sp, #-16]!\n"
		"stp x28, x29, [sp, #-16]!\n"
		"stp x26, x27, [sp, #-16]!\n"
		"stp x24, x25, [sp, #-16]!\n"
		"stp x22, x23, [sp, #-16]!\n"
		"stp x20, x21, [sp, #-16]!\n"
		"stp x18, x19, [sp, #-16]!\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"orr x0, x0, #(3L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

	"1:\n"
		"dsb sy\n"
		"wfi\n"

		"mrs x0, ISR_EL1\n"
		"cbz x0, 1b\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"bic x0, x0, #(1L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

		"ldp x18, x19, [sp], #16\n"
		"ldp x20, x21, [sp], #16\n"
		"ldp x22, x23, [sp], #16\n"
		"ldp x24, x25, [sp], #16\n"
		"ldp x26, x27, [sp], #16\n"
		"ldp x28, x29, [sp], #16\n"
		"ldr x30, [sp], #16\n"

		"ret\n"
	".popsection\n"
);

void apple_cpu_deep_wfi(void);

static __cpuidle int apple_enter_wfi(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static __cpuidle int apple_enter_idle(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	/*
	 * Deep WFI will clobber FP state, among other things.
	 * The CPU PM notifier will take care of saving that and anything else
	 * that needs to be notified of the CPU powering down.
	 */
	if (cpu_pm_enter())
		return -1;

	ct_cpuidle_enter();

	switch(index) {
	case STATE_PWRDOWN:
		apple_cpu_deep_wfi();
		break;
	default:
		WARN_ON(1);
		break;
	}

	ct_cpuidle_exit();

	cpu_pm_exit();

	return index;
}

static struct cpuidle_driver apple_idle_driver = {
	.name = "apple_idle",
	.owner = THIS_MODULE,
	.states = {
		[STATE_WFI] = {
			.enter			= apple_enter_wfi,
			.enter_s2idle		= apple_enter_wfi,
			.exit_latency		= 1,
			.target_residency	= 1,
			.power_usage            = UINT_MAX,
			.name			= "WFI",
			.desc			= "CPU clock-gated",
			.flags			= 0,
		},
		/*
		 * The cluster only grants its top P-states while few of its
		 * cores are out of this state: a core idling in WFI counts as
		 * active. With a 10 ms residency the governor almost never
		 * picked CPU PD on busy systems, so a single busy M2 P core saw
		 * 2-3 "active" WFI siblings and was held at 2208-2724 MHz
		 * instead of 3204 MHz. 100 us is where the grant and the idle
		 * cluster power stop improving on T8112; shorter only adds
		 * wake latency.
		 *
		 * The exit latency is the measured cost of waking from this
		 * state over WFI: on T8112 the cyclictest p99 delta was 13-17 us
		 * on a P core and 18-26 us on an E core. PM QoS and the
		 * governor's latency limit compare against it.
		 */
		[STATE_PWRDOWN] = {
			.enter			= apple_enter_idle,
			.enter_s2idle		= apple_enter_idle,
			.exit_latency		= 30,
			.target_residency	= 100,
			.power_usage            = 0,
			.name			= "CPU PD",
			.desc			= "CPU/cluster powered down",
			.flags			= CPUIDLE_FLAG_RCU_IDLE,
		},
	},
	.safe_state_index = STATE_WFI,
	.state_count = STATE_COUNT,
};

/* SoCs whose CYC_OVRD the OS can write: M1 to M3. */
static const char *const apple_cyc_ovrd_machines[] = {
	"apple,t8103", "apple,t6000", "apple,t6001", "apple,t6002",
	"apple,t8112", "apple,t6020", "apple,t6021", "apple,t6022",
	"apple,t8122", "apple,t6030", "apple,t6031", "apple,t6032",
	"apple,t6034",
	NULL
};

static int apple_cpuidle_probe(struct platform_device *pdev)
{
	/*
	 * CPU PD writes CYC_OVRD, which only M1 to M3 let the OS write. From
	 * M4 on it is locked and the write is undefined, and WFI already
	 * power-gates an idle core, so both entries use WFI. Keep index 1:
	 * the generic s2idle selector skips fallback index 0, otherwise
	 * tick_freeze() and timekeeping suspension are never reached.
	 */
	if (!of_machine_compatible_match(apple_cyc_ovrd_machines))
		apple_idle_driver.states[STATE_PWRDOWN] = apple_idle_driver.states[STATE_WFI];

	return cpuidle_register(&apple_idle_driver, NULL);
}

static const struct of_device_id apple_cpuidle_of_match[] = {
	{ .compatible = "apple,t6041-cpuidle" },
	{ .compatible = "apple,t8132-cpuidle" },
	{ .compatible = "apple,t8140-cpuidle" },
	{ }
};

static struct platform_driver apple_cpuidle_driver = {
	.driver = {
		.name = "cpuidle-apple",
		.of_match_table = apple_cpuidle_of_match,
	},
	.probe = apple_cpuidle_probe,
};

static int __init apple_cpuidle_init(void)
{
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	ret = platform_driver_register(&apple_cpuidle_driver);
	if (ret)
		return ret;

	/* DT-described systems are populated by the OF platform code. */
	np = of_find_matching_node(NULL, apple_cpuidle_of_match);
	if (np) {
		of_node_put(np);
		return 0;
	}

	if (!of_machine_is_compatible("apple,arm-platform"))
		return 0;

	/*
	 * The M1 to M3 machines (CPU PD) plus the M4-and-later ones that load
	 * the driver for its WFI state. The device-tree parts above are matched
	 * by their cpuidle node instead.
	 */
	if (!of_machine_compatible_match(apple_cyc_ovrd_machines) &&
	    !of_machine_is_compatible("apple,t6050") &&
	    !of_machine_is_compatible("apple,t8140") &&
	    !of_machine_is_compatible("apple,t8142"))
		return 0;

	/*
	 * T8142 under aura's PSCI monitor idles through cpuidle-psci
	 * (t8142-j813-psci-idle.dtsi). Raw aura has no /psci node and would
	 * otherwise have no cpuidle driver, so s2idle never freezes the tick.
	 */
	if (of_machine_is_compatible("apple,t8142")) {
		np = of_find_node_by_path("/psci");
		if (np) {
			of_node_put(np);
			return 0;
		}
	}

	pdev = platform_device_register_simple("cpuidle-apple", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&apple_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(apple_cpuidle_init);
