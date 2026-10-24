// SPDX-License-Identifier: GPL-2.0-only
/* One-shot J314s secondary CPU context-loss experiment; never an idle driver. */
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/stop_machine.h>
#include <linux/uaccess.h>
#include <asm/arch_timer.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/irqflags.h>
#include <asm/memory.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <asm/sysreg.h>

#include "cpu-context-lab-policy.h"
#include "cpu-context-lab.h"

#define LAB_TARGET 1
#define LAB_OBSERVER 0
#define LAB_CPU_COUNT 10
#define LAB_STATUS 0x100
#define LAB_OFF_US 50000
#define LAB_RETURN_US 500000
#define LAB_READY_US 50000
#define LAB_STATUS_POLLS 500000
#define LAB_CYC sys_reg(3, 5, 15, 5, 0)
#define LAB_EHID4 sys_reg(3, 0, 15, 4, 1)
#define LAB_IPI_SR sys_reg(3, 5, 15, 1, 1)
#define LAB_IPI_PENDING BIT_ULL(0) /* AIC IPI_SR_PENDING: write one to acknowledge. */
#define LAB_CYC_WAKE_MASK GENMASK_ULL(25, 20)
#define LAB_CYC_STOP ((2ULL << 20) | (2ULL << 22) | (3ULL << 24) | BIT_ULL(0))
#define LAB_TIMER_CTL_MASK GENMASK_ULL(1, 0) /* ENABLE and IMASK; ISTATUS is read-only. */

enum lab_target_step {
	LAB_STEP_NONE,
	LAB_STEP_LOCAL_READY,
	LAB_STEP_CPU_PM_ENTER,
	LAB_STEP_CPU_PM_READY,
	LAB_STEP_FINISHER,
	LAB_STEP_STOP_COMMITTED,
	LAB_STEP_RESUMED,
	LAB_STEP_COMPLETE,
};

/* Built-in/static: an unrecovered CPU must never outlive this storage. */
static struct {
	void __iomem *impl;
	void __iomem *start;
	void __iomem *uart;
	__le64 __iomem *release;
	u64 release_pa;
	u64 old_target;
	u64 resume_pa;
	u64 cyc_before, cyc_after, ehid4, csselr;
	u64 ipi_before, ipi_resume_raw, ipi_after;
	u64 clidr, ccsidr;
	u64 cntp_ctl, cntp_cval, cntv_ctl, cntv_cval;
	u64 mpidr_before, mpidr_after;
	u64 counter_hz;
	unsigned long parked;
	int phase;
	int result;
	int observer_result;
	int cpu_pm_result;
	enum lab_target_step target_step;
	u64 last_status;
	u64 status_before;
	bool off_observed;
	bool start_written;
	bool release_restored;
	bool same_task;
	bool timers_restored;
	atomic_t consumed;
} lab;

static bool lab_enabled;
static DEFINE_MUTEX(lab_lock);

static int __init apple_context_lab_enable(char *arg)
{
	return !kstrtobool(arg, &lab_enabled);
}
__setup("apple_cpu_context=", apple_context_lab_enable);

static u64 lab_now(void)
{
	return arch_timer_read_counter();
}

static bool lab_expired(u64 start, unsigned int us)
{
	return lab_now() - start >= div_u64(lab.counter_hz * us, 1000000);
}

static void notrace lab_uart_char(u8 value)
{
	unsigned int left = LAB_UART_POLLS;

	while (readl_relaxed(lab.uart + LAB_UART_UFSTAT) & LAB_UART_TX_FULL) {
		if (!--left)
			return;
		cpu_relax();
	}
	writel_relaxed(value, lab.uart + LAB_UART_UTXH);
}

static void notrace lab_uart_mark(u8 value)
{
	/* All CPUs are parked during transition markers; no console lock is used. */
	lab_uart_char('^');
	lab_uart_char(value);
	lab_uart_char('\n');
	dsb(sy);
}

static void notrace lab_uart_value(u8 tag, u64 value)
{
	static const char digits[] = "0123456789abcdef";
	int shift;

	lab_uart_char('^');
	lab_uart_char(tag);
	lab_uart_char('=');
	for (shift = 60; shift >= 0; shift -= 4)
		lab_uart_char(digits[(value >> shift) & 0xf]);
	lab_uart_char('\n');
	dsb(sy);
}

static void lab_uart_unmap(void *mapping)
{
	iounmap(mapping);
}

static int __init lab_uart_probe(struct platform_device *pdev)
{
	struct device_node *np = of_find_node_by_path("serial0");
	struct resource resource;
	int ret;

	if (!np)
		return -ENODEV;
	ret = of_device_is_available(np) && of_device_is_compatible(np, "apple,s5l-uart") ?
		of_address_to_resource(np, 0, &resource) : -ENODEV;
	of_node_put(np);
	if (ret)
		return ret;
	if (resource.start != LAB_UART_PA || resource_size(&resource) < LAB_UART_SIZE)
		return -ESTALE;
	/* Match the Apple early console's MMIO32, non-posted device mapping. */
	lab.uart = ioremap_np(resource.start, LAB_UART_SIZE);
	if (!lab.uart)
		return -ENOMEM;
	return devm_add_action_or_reset(&pdev->dev, lab_uart_unmap, lab.uart);
}

static int lab_get_phase(void)
{
	/* Observe the state payload published by the other CPU before its phase. */
	return smp_load_acquire(&lab.phase);
}

static void lab_set_phase(int phase)
{
	/* Publish saved context/result before allowing the other CPUs to proceed. */
	smp_store_release(&lab.phase, phase);
}

static bool lab_wait_phase(int phase, unsigned int us)
{
	u64 start = lab_now();

	while (lab_get_phase() != phase) {
		if (lab_get_phase() == LAB_ABORT ||
		    lab_expired(start, us))
			return false;
		cpu_relax();
	}
	return true;
}

static void lab_publish_target(u64 target)
{
	writeq_relaxed(target, lab.release);
	dcache_clean_poc((__force unsigned long)lab.release,
			 (__force unsigned long)lab.release + sizeof(*lab.release));
	dsb(sy);
}

static void lab_restore_target(void)
{
	lab_publish_target(lab.old_target);
	WRITE_ONCE(lab.release_restored,
		   readq_relaxed(lab.release) == lab.old_target);
}

static unsigned int lab_stash_index(void)
{
	u64 mpidr = cpu_logical_map(LAB_TARGET) & mpidr_hash.mask;

	return ((mpidr & 0xff) >> mpidr_hash.shift_aff[0]) |
	       ((mpidr & 0xff00) >> mpidr_hash.shift_aff[1]) |
	       ((mpidr & 0xff0000) >> mpidr_hash.shift_aff[2]) |
	       ((mpidr & 0xff00000000ULL) >> mpidr_hash.shift_aff[3]);
}

static int notrace lab_finisher(unsigned long unused)
{
	unsigned long slot = (unsigned long)&sleep_save_stash[lab_stash_index()];
	unsigned long state = READ_ONCE(*(unsigned long *)slot);
	u64 cyc;

	WRITE_ONCE(lab.target_step, LAB_STEP_FINISHER);
	lab_uart_mark('d');
	/* The current Linux resume implementation consumes a virtual state pointer. */
	if (!state)
		return -EINVAL;
	dcache_clean_poc(state, state + sizeof(struct sleep_stack_data));
	dcache_clean_poc(slot, slot + sizeof(unsigned long));
	lab_publish_target(lab.resume_pa);
	if (readq_relaxed(lab.release) != lab.resume_pa) {
		lab_restore_target();
		return -EIO;
	}
	lab_uart_mark('e');

	/* Preserve the stopped-machine test's two architectural timer snapshots. */
	write_sysreg(0, cntp_ctl_el0);
	write_sysreg(0, cntv_ctl_el0);
	cyc = (lab.cyc_before & ~LAB_CYC_WAKE_MASK) | LAB_CYC_STOP;
	write_sysreg_s(cyc, LAB_CYC);
	isb();
	/* Serialized before STOPPING: CPU0 cannot interleave its status trace. */
	lab_uart_value('K', lab.cyc_before);
	lab_uart_value('Y', read_sysreg_s(LAB_CYC));
	lab_uart_value('I', read_sysreg(isr_el1));
	lab_uart_value('J', read_sysreg_s(LAB_IPI_SR));
	/* Cancellation of GO must win before any irreversible CPU stop request. */
	if (cmpxchg_release(&lab.phase, LAB_GO, LAB_STOPPING) != LAB_GO)
		return -ECANCELED;
	WRITE_ONCE(lab.target_step, LAB_STEP_STOP_COMMITTED);
	lab_uart_mark('f');
	/* No C, stack writes or shared-cache maintenance after the final L1 sweep. */
	apple_context_lab_poweroff(lab.start, BIT(LAB_TARGET), lab.uart);
}

static int lab_target(void)
{
	struct task_struct *task = current;
	u64 ways, sets, bytes;
	int ret;

	if (!lab_wait_phase(LAB_GO, LAB_READY_US)) {
		ret = -ETIMEDOUT;
		goto abort;
	}
	lab.mpidr_before = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	lab.cyc_before = read_sysreg_s(LAB_CYC);
	lab.ehid4 = read_sysreg_s(LAB_EHID4);
	lab.ipi_before = read_sysreg_s(LAB_IPI_SR);
	lab.csselr = read_sysreg(csselr_el1);
	lab.clidr = read_sysreg(clidr_el1);
	write_sysreg(0, csselr_el1);
	isb();
	lab.ccsidr = read_sysreg(ccsidr_el1);
	write_sysreg(lab.csselr, csselr_el1);
	isb();
	if ((read_sysreg(id_aa64mmfr2_el1) >> 20) & 0xf) {
		ways = ((lab.ccsidr >> 3) & GENMASK_ULL(20, 0)) + 1;
		sets = ((lab.ccsidr >> 32) & GENMASK_ULL(23, 0)) + 1;
	} else {
		ways = ((lab.ccsidr >> 3) & GENMASK_ULL(9, 0)) + 1;
		sets = ((lab.ccsidr >> 13) & GENMASK_ULL(14, 0)) + 1;
	}
	bytes = ways * sets * BIT_ULL((lab.ccsidr & 7) + 4);
	if ((lab.clidr & 7) < 2 || (lab.clidr & 7) > 4 || bytes != 0x10000) {
		ret = -EOPNOTSUPP;
		goto abort;
	}
	lab.cntp_ctl = read_sysreg(cntp_ctl_el0);
	lab.cntp_cval = read_sysreg(cntp_cval_el0);
	lab.cntv_ctl = read_sysreg(cntv_ctl_el0);
	lab.cntv_cval = read_sysreg(cntv_cval_el0);
	WRITE_ONCE(lab.target_step, LAB_STEP_LOCAL_READY);
	lab_uart_mark('a');
	if (lab.ipi_before & LAB_IPI_PENDING) {
		ret = -EBUSY;
		goto abort;
	}

	/*
	 * This is a stopped-machine context test, not schedulable idle. All CPUs
	 * already mask IRQ/FIQ; CPU0 stays active and explicitly restarts CPU1.
	 * A CPU-local broadcast hrtimer cannot deliver deadlines in this window
	 * and may veto its own owner. Preserve/restore the local timer registers
	 * instead; expired events are handled after stop_machine releases CPUs.
	 */
	WRITE_ONCE(lab.target_step, LAB_STEP_CPU_PM_ENTER);
	lab_uart_mark('b');
	ret = cpu_pm_enter();
	WRITE_ONCE(lab.cpu_pm_result, ret);
	if (ret)
		goto abort; /* Robust enter already sent ENTER_FAILED. */
	WRITE_ONCE(lab.target_step, LAB_STEP_CPU_PM_READY);
	lab_uart_mark('c');

	ret = cpu_suspend(0, lab_finisher);
	WRITE_ONCE(lab.target_step, LAB_STEP_RESUMED);
	lab_uart_mark('u');
	lab_restore_target();
	write_sysreg_s(lab.cyc_before, LAB_CYC);
	write_sysreg(lab.csselr, csselr_el1);
	write_sysreg(lab.cntp_cval, cntp_cval_el0);
	write_sysreg(lab.cntv_cval, cntv_cval_el0);
	write_sysreg(lab.cntp_ctl, cntp_ctl_el0);
	write_sysreg(lab.cntv_ctl, cntv_ctl_el0);
	isb();
	cpu_pm_exit();
	lab.timers_restored = (read_sysreg(cntp_ctl_el0) & LAB_TIMER_CTL_MASK) ==
		(lab.cntp_ctl & LAB_TIMER_CTL_MASK) &&
		read_sysreg(cntp_cval_el0) == lab.cntp_cval &&
		(read_sysreg(cntv_ctl_el0) & LAB_TIMER_CTL_MASK) ==
		(lab.cntv_ctl & LAB_TIMER_CTL_MASK) &&
		read_sysreg(cntv_cval_el0) == lab.cntv_cval;
	lab.cyc_after = read_sysreg_s(LAB_CYC);
	lab.ipi_resume_raw = read_sysreg_s(LAB_IPI_SR);
	if (!ret) {
		/* AURA skips this ack when the resume target was already published. */
		write_sysreg_s(LAB_IPI_PENDING, LAB_IPI_SR);
		isb();
	}
	lab.ipi_after = read_sysreg_s(LAB_IPI_SR);
	lab.mpidr_after = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	lab.same_task = current == task && smp_processor_id() == LAB_TARGET;
	if (!ret && (!lab.release_restored || !lab.same_task || !lab.timers_restored ||
		     lab.mpidr_before != lab.mpidr_after ||
		     lab.cyc_before != lab.cyc_after || lab.ipi_after & LAB_IPI_PENDING))
		ret = -EUCLEAN;
	WRITE_ONCE(lab.target_step, LAB_STEP_COMPLETE);
	lab_uart_mark('C');
abort:
	WRITE_ONCE(lab.result, ret);
	lab_set_phase(ret ? LAB_ABORT : LAB_RETURNED);
	return ret;
}

static int lab_observer(void)
{
	u64 start = lab_now();
	unsigned int polls = 0;
	int phase, observed;

	/* All callbacks are running with IRQs off before the target may proceed. */
	while (READ_ONCE(lab.parked) != GENMASK(LAB_CPU_COUNT - 1, 0)) {
		if (lab_expired(start, LAB_READY_US))
			goto abort;
		cpu_relax();
	}
	/* A zero before the target even enters its finisher is not an OFF transition. */
	lab_uart_mark('B');
	lab.status_before = readq_relaxed(lab.impl + LAB_STATUS);
	lab_uart_value('V', lab.status_before);
	if (!(lab.status_before & 0xff))
		goto abort;
	lab_uart_mark('O');
	lab_set_phase(LAB_GO);
	if (!lab_wait_phase(LAB_STOPPING, LAB_READY_US))
		goto abort;

	start = lab_now();
	for (;;) {
		/* CPU_IMPL registers use the firmware's 64-bit access width. */
		lab.last_status = readq_relaxed(lab.impl + LAB_STATUS);
		/* Leave UART to CPU1 until OFF or timeout, preserving its snapshots. */
		if (!(lab.last_status & 0xff))
			break;
		if (++polls >= LAB_STATUS_POLLS || lab_expired(start, LAB_OFF_US)) {
			lab_uart_value('X', lab.last_status);
			lab_uart_value('E', lab_now() - start);
			goto abort;
		}
		cpu_relax();
	}
	if (!apple_context_lab_may_start(lab.status_before, lab.last_status,
					 lab_get_phase() == LAB_STOPPING, lab.start_written,
					 div_u64((lab_now() - start) * 1000000, lab.counter_hz)))
		goto abort;
	lab.off_observed = true;
	lab_uart_mark('F');
	dsb(sy);
	/* AURA cpu_start_bit(): BIT(4 * cluster + core); target is cluster0/core1. */
	lab_uart_mark('R');
	writel(BIT(1), lab.start + 4);
	writel(BIT(1), lab.start + 8);
	lab.start_written = true;
	lab_uart_mark('S');
	if (!lab_wait_phase(LAB_RETURNED, LAB_RETURN_US))
		goto abort;
	lab_uart_mark('D');
	return 0;
abort:
	lab_uart_mark('A');
	/* Observer errors must not overwrite the target CPU's return value. */
	WRITE_ONCE(lab.observer_result, -ETIMEDOUT);
	phase = lab_get_phase();
	while (phase != LAB_RETURNED && phase != LAB_ABORT) {
		/* Compete with the finisher's GO -> STOPPING commit, not its result. */
		observed = cmpxchg_release(&lab.phase, phase, LAB_ABORT);
		if (observed == phase)
			break;
		phase = observed;
	}
	if (apple_context_lab_may_restore_after_cancel(phase, lab.start_written))
		lab_restore_target();
	return -ETIMEDOUT;
}

static int lab_stop_cpu(void *unused)
{
	unsigned int cpu = smp_processor_id();

	/* AIC timers and IPIs are FIQs: all ten CPUs must have both masks set. */
	if ((read_sysreg(daif) & (PSR_I_BIT | PSR_F_BIT)) != (PSR_I_BIT | PSR_F_BIT)) {
		lab_set_phase(LAB_ABORT);
		return -EINVAL;
	}
	set_bit(cpu, &lab.parked);
	if (cpu == LAB_OBSERVER)
		return lab_observer();
	if (cpu == LAB_TARGET)
		return lab_target();
	while (lab_get_phase() != LAB_RETURNED &&
	       lab_get_phase() != LAB_ABORT)
		cpu_relax();
	return 0;
}

static int lab_preflight(void)
{
	if (!(readl_relaxed(lab.uart + LAB_UART_UFCON) & LAB_UART_FIFO_ENABLED) ||
	    !(readl_relaxed(lab.uart + LAB_UART_UCON) & 0xc))
		return -ENODEV;
	if (num_online_cpus() != LAB_CPU_COUNT ||
	    cpumask_last(cpu_online_mask) != LAB_CPU_COUNT - 1 ||
	    cpu_logical_map(LAB_OBSERVER) != 0 || cpu_logical_map(LAB_TARGET) != 1 ||
	    readq_relaxed(lab.release) != __pa_symbol(secondary_holding_pen))
		return -ESTALE;
	return 0;
}

static ssize_t lab_trigger(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	char command[sizeof("cpu1-context-loss\n")];
	int ret;

	if (count != sizeof(command) - 1 || copy_from_user(command, buf, count))
		return -EINVAL;
	command[count] = '\0';
	if (strcmp(command, "cpu1-context-loss\n"))
		return -EINVAL;
	if (!mutex_trylock(&lab_lock))
		return -EBUSY;
	if (atomic_cmpxchg(&lab.consumed, 0, 1)) {
		ret = -EALREADY;
		goto unlock;
	}
	cpus_read_lock();
	ret = lab_preflight();
	if (!ret) {
		pr_notice("apple-context-lab: CPU1 context-loss attempt consumed\n");
		lab_uart_mark('G');
		ret = stop_machine_cpuslocked(lab_stop_cpu, NULL, cpu_online_mask);
		pr_notice("apple-context-lab: return=%d phase=%d off=%d start=%d restored=%d\n",
			  ret, lab.phase, lab.off_observed, lab.start_written,
			  lab.release_restored);
	}
	cpus_read_unlock();
unlock:
	mutex_unlock(&lab_lock);
	return ret ? ret : count;
}

static const struct file_operations lab_trigger_fops = {
	.owner = THIS_MODULE,
	.write = lab_trigger,
	.open = nonseekable_open,
};

static int lab_result_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "consumed=%d phase=%d result=%d observer_result=%d\n",
		   atomic_read(&lab.consumed), READ_ONCE(lab.phase),
		   READ_ONCE(lab.result), READ_ONCE(lab.observer_result));
	seq_printf(s, "target=1 observer=0 off=%d start=%d status=%#llx restored=%d\n",
		   lab.off_observed, lab.start_written, lab.last_status, lab.release_restored);
	seq_printf(s, "same_task=%d timers_restored=%d mpidr_before=%#llx mpidr_after=%#llx\n",
		   lab.same_task, lab.timers_restored, lab.mpidr_before, lab.mpidr_after);
	seq_printf(s, "cyc_before=%#llx cyc_after=%#llx ehid4=%#llx\n",
		   lab.cyc_before, lab.cyc_after, lab.ehid4);
	seq_printf(s, "ipi_before=%#llx ipi_resume_raw=%#llx ipi_after=%#llx\n",
		   lab.ipi_before, lab.ipi_resume_raw, lab.ipi_after);
	seq_printf(s, "clidr=%#llx ccsidr=%#llx status_before=%#llx\n",
		   lab.clidr, lab.ccsidr, lab.status_before);
	seq_printf(s, "target_step=%d cpu_pm_result=%d\n",
		   READ_ONCE(lab.target_step), READ_ONCE(lab.cpu_pm_result));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lab_result);

static int __init lab_probe(struct platform_device *pdev)
{
	struct device_node *np, *chosen;
	struct dentry *dir;
	const char *tag;
	int ret;

	if (!lab_enabled || !of_machine_is_compatible("apple,j314s") ||
	    !of_machine_is_compatible("apple,t6000"))
		return -ENODEV;
	chosen = of_find_node_by_path("/chosen");
	ret = of_property_read_string(chosen, "asahi,m1n1-stage2-version", &tag);
	if (!ret && strcmp(tag, "v0.1.0-smp-79d1c20c"))
		ret = -ESTALE;
	of_node_put(chosen);
	if (ret)
		return ret;
	np = of_get_cpu_node(LAB_TARGET, NULL);
	ret = of_property_read_u64(np, "cpu-release-addr", &lab.release_pa);
	of_node_put(np);
	if (ret || !lab.release_pa || !IS_ALIGNED(lab.release_pa, sizeof(u64)))
		return -EINVAL;
	lab.impl = devm_platform_ioremap_resource_byname(pdev, "cpu-impl");
	if (IS_ERR(lab.impl))
		return PTR_ERR(lab.impl);
	lab.start = devm_platform_ioremap_resource_byname(pdev, "cpu-start");
	if (IS_ERR(lab.start))
		return PTR_ERR(lab.start);
	ret = lab_uart_probe(pdev);
	if (ret)
		return ret;
	lab.release = ioremap_cache(lab.release_pa, sizeof(*lab.release));
	if (!lab.release)
		return -ENOMEM;
	lab.old_target = __pa_symbol(secondary_holding_pen);
	lab.resume_pa = __pa_symbol(apple_context_lab_resume);
	lab.counter_hz = read_sysreg(cntfrq_el0);
	if (!lab.counter_hz || lab.counter_hz > 1000000000ULL || lab_preflight()) {
		iounmap(lab.release);
		return -ESTALE;
	}
	dir = debugfs_create_dir("apple_cpu_context_lab", NULL);
	debugfs_create_file("trigger", 0200, dir, NULL, &lab_trigger_fops);
	debugfs_create_file("result", 0400, dir, NULL, &lab_result_fops);
	dev_info(&pdev->dev, "one-shot CPU1 experiment available; no test run\n");
	return 0;
}

static const struct of_device_id lab_match[] = {
	{ .compatible = "apple,j314s-cpu-context-lab" },
	{}
};

static struct platform_driver lab_driver = {
	.driver = {
		.name = "apple-cpu-context-lab",
		.of_match_table = lab_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver_probe(lab_driver, lab_probe);
