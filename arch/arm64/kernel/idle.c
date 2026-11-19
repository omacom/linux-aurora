// SPDX-License-Identifier: GPL-2.0-only
/*
 * Low-level idle sequences
 */

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/jump_label.h>

#include <asm/apple-idle.h>
#include <asm/barrier.h>
#include <asm/cpuidle.h>
#include <asm/cpufeature.h>
#include <asm/cputype.h>
#include <asm/fpsimd.h>
#include <asm/sysreg.h>

enum {
    ARM64_IDLE_WFI,
    ARM64_IDLE_YIELD,
    ARM64_IDLE_NOP,
} idle = ARM64_IDLE_WFI;

static int __init setup_idle(char *arg)
{
	if (!arg)
		return -1;
	else if (!strcmp(arg, "wfi"))
		idle = ARM64_IDLE_WFI;
	else if (!strcmp(arg, "yield"))
		idle = ARM64_IDLE_YIELD;
	else if (!strcmp(arg, "nop"))
		idle = ARM64_IDLE_NOP;
	else
		return -1;

	return 0;
}
early_param("idle", setup_idle);

#ifdef CONFIG_ARCH_APPLE
/*
 * Preserve general-purpose and FP/SIMD state across WFI on M4 and later
 * Apple cores. The key is enabled once at boot; elsewhere the check is a
 * patched-out branch.
 */
static DEFINE_STATIC_KEY_FALSE(apple_wfi_loses_regs);
static bool apple_keep_idle_nop __initdata;

/*
 * The boot loader appends idle=nop after the user's arguments, so it cannot
 * be overridden on the command line. This flag keeps it anyway.
 */
static int __init setup_apple_keep_idle_nop(char *arg)
{
	apple_keep_idle_nop = true;
	return 0;
}
early_param("apple_keep_idle_nop", setup_apple_keep_idle_nop);

static int __init apple_wfi_quirk_init(void)
{
	u32 model = read_cpuid_id() & MIDR_CPU_MODEL_MASK;

	if (midr_is_apple_m4_or_later(read_cpuid_id())) {
		static_branch_enable(&apple_wfi_loses_regs);
		pr_info("Apple: preserving registers across WFI\n");
	}
	/*
	 * m1n1 passes idle=nop on M4 and later for kernels that do not
	 * preserve the registers WFI loses there. This one does, from here
	 * on, so on T6050 spinning only keeps every idle core busy: the
	 * system draws about 7 W more at idle. The secondary CPUs are not up
	 * yet, and the boot CPU has spun until now, so WFI is safe from here.
	 */
	if (idle == ARM64_IDLE_NOP && !apple_keep_idle_nop &&
	    (model == MIDR_APPLE_T6050_M || model == MIDR_APPLE_T6050_P)) {
		idle = ARM64_IDLE_WFI;
		pr_info("Apple: idling in WFI, ignoring idle=nop\n");
	}
	return 0;
}
early_initcall(apple_wfi_quirk_init);

static __always_inline bool apple_wfi_needs_save(void)
{
	return static_branch_unlikely(&apple_wfi_loses_regs);
}
#else
static __always_inline bool apple_wfi_needs_save(void)
{
	return false;
}
#endif

/*
 *	cpu_do_idle()
 *
 *	Idle the processor (wait for interrupt).
 *
 *	If the CPU supports priority masking we must do additional work to
 *	ensure that interrupts are not masked at the PMR (because the core will
 *	not wake up if we block the wake up signal in the interrupt controller).
 */
void __cpuidle cpu_do_idle(void)
{
	struct arm_cpuidle_irq_context context;

	arm_cpuidle_save_irq_context(&context);

	if (likely(idle == ARM64_IDLE_WFI)) {
		if (apple_wfi_needs_save()) {
			/*
			 * The FP/SIMD registers do not survive either: drop the
			 * lazy ownership so the next return to userspace reloads
			 * them, even when the same task resumes on this CPU.
			 */
			fpsimd_save_and_flush_cpu_state();
			apple_cpu_wfi();
		} else {
			dsb(sy);
			wfi();
		}
	} else if (idle == ARM64_IDLE_YIELD) {
		dsb(sy);
		asm volatile("yield" ::: "memory");
	}

	arm_cpuidle_restore_irq_context(&context);
}

/*
 * This is our default idle handler.
 */
void __cpuidle arch_cpu_idle(void)
{
	/*
	 * This should do all the clock switching and wait for interrupt
	 * tricks
	 */
	cpu_do_idle();
}
