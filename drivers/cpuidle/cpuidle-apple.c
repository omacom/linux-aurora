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
		[STATE_PWRDOWN] = {
			.enter			= apple_enter_idle,
			.enter_s2idle		= apple_enter_idle,
			.exit_latency		= 10,
			.target_residency	= 10000,
			.power_usage            = 0,
			.name			= "CPU PD",
			.desc			= "CPU/cluster powered down",
			.flags			= CPUIDLE_FLAG_RCU_IDLE,
		},
	},
	.safe_state_index = STATE_WFI,
	.state_count = STATE_COUNT,
};

static int apple_cpuidle_probe(struct platform_device *pdev)
{
	/*
	 * CPU PD writes CYC_OVRD, which is an undefined instruction on
	 * T8140. Its WFI already lets the hardware power-gate idle
	 * secondary cores, so both entries must use WFI. Keep index 1:
	 * the generic s2idle selector skips fallback index 0, otherwise
	 * tick_freeze() and timekeeping suspension are never reached.
	 */
	if (of_machine_is_compatible("apple,t8140")) {
		apple_idle_driver.states[STATE_PWRDOWN] = apple_idle_driver.states[STATE_WFI];
	} else if (of_machine_is_compatible("apple,j514s")) {
		/*
		 * Measured on this machine (T6030): the defaults keep cores out
		 * of CPU PD for most idle periods of a running desktop, although
		 * it saves power from far shorter ones. 500 us takes most of
		 * that saving (about 1.1 W in 1080p60 video playback) at half
		 * the cost to QD1 storage reads that 100 us has, and none to
		 * single-client database throughput. A wakeup from CPU PD takes
		 * 10-20 us longer than one from WFI.
		 */
		apple_idle_driver.states[STATE_PWRDOWN].target_residency = 500;
		apple_idle_driver.states[STATE_PWRDOWN].exit_latency = 20;
	}

	return cpuidle_register(&apple_idle_driver, NULL);
}

static const struct of_device_id apple_cpuidle_of_match[] = {
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

	if (!(of_machine_is_compatible("apple,t8103") ||
	      of_machine_is_compatible("apple,t8112") ||
	      of_machine_is_compatible("apple,t8122") ||
	      of_machine_is_compatible("apple,t6000") ||
	      of_machine_is_compatible("apple,t6001") ||
	      of_machine_is_compatible("apple,t6002") ||
	      of_machine_is_compatible("apple,t6020") ||
	      of_machine_is_compatible("apple,t6021") ||
	      of_machine_is_compatible("apple,t6022") ||
	      of_machine_is_compatible("apple,t6030") ||
	      of_machine_is_compatible("apple,t6031") ||
	      of_machine_is_compatible("apple,t6032") ||
	      of_machine_is_compatible("apple,t6034") ||
	      of_machine_is_compatible("apple,t8140")))
		return 0;

	pdev = platform_device_register_simple("cpuidle-apple", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&apple_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(apple_cpuidle_init);
