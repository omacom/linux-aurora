// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Staged H16 backend, using the accepted H16 cold-boot facts, round 5. */
#include <drm/drm_mm.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-map-ops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/clpc.h>
#include <linux/soc/apple/pmgr-pwrstate.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "ane_fw_sleep.h"
#include "ane_fw_platform.h"
#include "ane_h16_accel.h"
#include "ane_h16_boot.h"
#include "ane_h16_preload.h"
#include "ane_t6021_lifetime.h"

#define ANE_H16_TX_WORDS 272
/* Private program window above the SHAREDMALLOC range; host policy. */
#define ANE_H16_PROGRAM_WINDOW SZ_1G
/* ane_h16_alloc_prot(): a DVA range from the program window. */
#define ANE_H16_PROGRAM_ANY U64_MAX

struct ane_h16_mapping {
	struct list_head node;
	u64 dva;
	size_t size;
	size_t mapped;
	struct page **pages;
	u32 blocks;
	void *cpu;
	/* The DVA range, for a mapping in the program window. */
	struct drm_mm_node range;
};

struct ane_h16_device {
	struct ane_h16_boot boot;
	struct device *dev;
	void __iomem *window[ANE_H16_WINDOWS];
	struct device *power[5];
	/* Optional "perf" floor, voted only while an EXECUTE is in flight. */
	struct device *perf;
	bool perf_held;
	/* Optional CLPC input: DRAM and fabric floors while an EXECUTE is in flight. */
	struct apple_clpc_ane *clpc;
	struct device *dart[3];
	u32 powered;
	u32 darts_powered;
	u64 preload_phys[2];
	/*
	 * Guards mappings and program_window. Program mappings come and go
	 * under the session lock while the engine is up; SHAREDMALLOC may be
	 * mapped at the same time.
	 */
	struct mutex window_lock;
	struct list_head mappings;
	struct iommu_domain *domain;
	struct iommu_group *group;
	bool claimed;
	bool attached;
	bool pinned;
	/*
	 * Registered at probe, outside ane_h16_lock: the PM chain calls it with
	 * its rwsem held and it takes ane_h16_lock. Until pinned it does nothing.
	 * While pinned: refuse system sleep (ane_fw_sleep.h).
	 */
	struct notifier_block pm_notifier;
	int irq;
	/*
	 * The channel IRQ (ANE_FW_IRQ_CHANNELS) as a wakeup for the polled
	 * waits once the firmware is up: pause() arms it before sleeping and
	 * the handler disables it again. The transport still reads,
	 * acknowledges and drains every ring itself, as before. irq_seq
	 * counts IRQs, so concurrent waiters each see their own wakeup.
	 */
	spinlock_t irq_lock;
	wait_queue_head_t irq_wq;
	bool irq_wait;
	bool irq_armed;
	unsigned long irq_seq;
	u64 irq_wakeups;
	u64 irq_timeouts;
	struct ane_fw_rtk rtk;
	struct {
		u64 word0;
		u64 word1;
	} tx[ANE_H16_TX_WORDS];
	u32 tx_read;
	u32 tx_write;
	bool rtk_failed;
	u64 shared_next;
	/* Program sections and surfaces: [program_start, program_end). */
	struct drm_mm program_window;
	u64 program_start;
	u64 program_end;
	struct dentry *debugfs;
	bool acquired;
	bool refused;
	bool program_ready;
	/* Firmware suspended and the engine power gated for system sleep. */
	bool slept;
	/* genpd's transitions of each of the five domains across a sleep. */
	struct ane_h16_genpd {
		struct notifier_block nb;
		struct ane_h16_device *h;
		bool added;
		unsigned int offs, ons;
		u64 off_at, off_ns;
	} genpd[5];
	u64 resume_ready_ns;
	/* EXCLAVE_MODE_STOP completed in this firmware boot (variant build). */
	bool mode_stopped;
	/*
	 * For a cold boot after sleep: the firmware runs from the preload in
	 * place, so its DATA is live. The verified DATA from bring-up is kept
	 * (warm-sleep builds only) and copied back; the heap tail is cleared.
	 */
	void *pristine_data;
	struct ane_fw_surface heap_tail;
	/* Our runtime references on the five domains and the DARTs are held. */
	bool engine_on;
	u64 resume_first_ns;
	unsigned int resume_replayed;
	unsigned int resume_restarts;
	bool resume_cold;
	/*
	 * A quarantined session at system sleep: its engine goes off without
	 * the firmware and boots cold on resume, if every domain went off.
	 */
	bool sleep_quarantined;
	/* Recovery of a session quarantined at runtime (ane_h16_hung()). */
	struct work_struct recover;
	bool recovering;
	/* System sleep is under way: it recovers a quarantined session itself. */
	bool recover_blocked;
	u64 recoveries;
	u64 recover_failures;
	struct ane_h16_session session;
	struct work_struct start;
};

static DEFINE_MUTEX(ane_h16_lock);
static bool ane_h16_released;
/* <debugfs>/ane-h16, shared by bound devices; protected by ane_h16_lock. */
static struct dentry *ane_h16_debugfs;
static unsigned int ane_h16_debugfs_users;

static u32 ane_h16_mmio_read32(void *ctx, enum ane_h16_window w, u32 off)
{
	struct ane_h16_device *h = ctx;

	return readl(h->window[w] + off);
}

static u64 ane_h16_mmio_read64(void *ctx, enum ane_h16_window w, u32 off)
{
	struct ane_h16_device *h = ctx;

	return readq(h->window[w] + off);
}

static void ane_h16_mmio_write32(void *ctx, enum ane_h16_window w, u32 off, u32 value)
{
	struct ane_h16_device *h = ctx;

	writel(value, h->window[w] + off);
}

static void ane_h16_mmio_write64(void *ctx, enum ane_h16_window w, u32 off, u64 value)
{
	struct ane_h16_device *h = ctx;

	writeq(value, h->window[w] + off);
}

static void ane_h16_delay(void *ctx, unsigned int us)
{
	if (us < 100)
		udelay(us);
	else
		usleep_range(us, us + 50);
}

static void ane_h16_log(void *ctx, const char *item, u64 a, u64 b)
{
	struct ane_h16_device *h = ctx;

	dev_info(h->dev, "H16 %s %llx %llx\n", item, a, b);
}

static void ane_h16_startup_fwlog(struct ane_h16_device *h);

static void ane_h16_quarantine(void *ctx, int error)
{
	struct ane_h16_device *h = ctx;

	if ((error == -ETIMEDOUT || error == -ESHUTDOWN) && h->boot.stage == ANE_H16_BOOT)
		ane_h16_startup_fwlog(h);

	dev_err(h->dev, "H16 quarantine error %d; mappings, power and module held until reboot\n",
		error);
}

/*
 * System sleep the macOS way (SPEC-ANE-SLEEP.md sections 4.1, 4.2 and 5.1):
 * suspend the firmware with SUSPEND (0x24), power gate the engine through
 * the PM domains it holds (DARTs first, then base, td, mpm, cpu, sys), and
 * after resume power it back up in order, let the DARTs restore their
 * translation state, rewrite the DAPF records, restore CE and the tunables
 * and release the CPU with SCRATCH7 = 1, the warm-boot argument. The same
 * firmware instance resumes with its programs; no LOAD or CREATE is replayed.
 */
#define ANE_H16_CMD_SUSPEND 0x24
#define ANE_H16_WARM_BOOT 1

static void ane_h16_busy(void *ctx, bool busy);
static void ane_h16_sleep_trace(struct ane_h16_device *h, const char *when);

/*
 * Let the channel IRQ wake the waits, or go back to pure polling. Off
 * across power down and the firmware startup that follows it, so the
 * startup handshake keeps its proven polled timing.
 */
static void ane_h16_irq_wait_set(struct ane_h16_device *h, bool on)
{
	if (h->irq < 0 || h->boot.soc->irq_role != ANE_FW_IRQ_CHANNELS)
		return;
	/*
	 * Arming and disarming both happen under irq_lock, so a pause racing
	 * with power down either armed before this (and is disarmed here) or
	 * sees irq_wait clear and polls.
	 */
	scoped_guard(spinlock_irqsave, &h->irq_lock) {
		WRITE_ONCE(h->irq_wait, on);
		if (!on && h->irq_armed) {
			h->irq_armed = false;
			disable_irq_nosync(h->irq);
		}
	}
	if (!on)
		synchronize_irq(h->irq);
}

static int ane_h16_suspend_firmware(struct ane_h16_device *h)
{
	struct ane_fw_transport *t = &h->boot.transport;
	const struct ane_h16_build *f = h->boot.build;
	u8 command[8] = {};
	u32 status;
	int ret;

	if (t->error)
		return t->error;
	for (unsigned int ch = 0; ch < ANE_FW_CHANNELS; ch++)
		if (t->pending[ch].active)
			return -EBUSY;
	ret = ane_fw_transport_settle(t, 2 * NSEC_PER_MSEC, 20 * NSEC_PER_MSEC);
	if (ret < 0)
		return ret;
	dev_info(h->dev, "H16 ANE: SET_SNE_PMU_BASE2 %s at startup (%#llx)\n",
		 h->boot.pmu_base_set ? "completed" : "not sent", h->boot.pmu_base);
	/* Variant: leave Exclave mode first, once per firmware boot. */
	if (h->boot.soc->exclave_mode_stop && !h->mode_stopped) {
		ret = ane_fw_transport_publish_mode_stop(t, ktime_get_ns() + 3 * NSEC_PER_SEC);
		if (!ret)
			ret = ane_fw_transport_wait(t, t->control_ch);
		dev_info(h->dev, "H16 ANE: EXCLAVE_MODE_STOP (0x31) on ring %s: %d\n",
			 ane_fw_channel_name(t->control_ch), ret);
		if (ret) {
			ane_h16_sleep_trace(h, "0x31 failed");
			ane_h16_session_dump_fwlog(&h->session);
			return ret;
		}
		h->mode_stopped = true;
	}
	ane_h16_sleep_trace(h, "before SUSPEND");
	put_unaligned_le16(ANE_H16_CMD_SUSPEND, command + 4);
	ret = ane_fw_transport_publish(t, ANE_FW_IO, command, sizeof(command),
				       ktime_get_ns() + 3 * NSEC_PER_SEC);
	if (!ret) {
		u8 *slot = ane_fw_transport_slot(t, ANE_FW_IO);

		ret = ane_fw_transport_wait(t, ANE_FW_IO);
		/*
		 * The H16 firmware is not expected to run in Exclave mode (the
		 * J616s macOS power-off finds no Exclave clients), but an H17
		 * style hang would look like this: the slot never taken.
		 */
		if (ret && t->ops->load64(t->ctx, slot) == t->pending[ANE_FW_IO].dva)
			dev_err(h->dev, "H16 ANE: 0x24 not consumed; Exclave mode suspected\n");
	}
	if (ret) {
		ane_h16_sleep_trace(h, "SUSPEND failed");
		ane_h16_session_dump_fwlog(&h->session);
		return ret;
	}
	status = h->boot.ops->read32(h->boot.ctx, ANE_H16_ENGINE, f->cpu_control + 4);
	dev_info(h->dev, "H16 ANE: SUSPEND acknowledged, wrapper status %#x\n", status);
	/*
	 * The ANE CPU idles in WFI (ASC wrapper status). The held H16 shutdown
	 * read 0x6e against the 0x6a reset value: bit 2. TODO(hardware): confirm.
	 */
	for (u64 deadline = ktime_get_ns() + NSEC_PER_SEC;;) {
		status = h->boot.ops->read32(h->boot.ctx, ANE_H16_ENGINE, f->cpu_control + 4);
		if ((status & BIT(2)) || ktime_get_ns() >= deadline)
			break;
		usleep_range(20, 50);
	}
	dev_info(h->dev, "H16 ANE: firmware suspended, wrapper status %#x\n", status);
	return status & BIT(2) ? 0 : -ETIMEDOUT;
}

/* Reverse of acquisition: perf floor, DARTs, then base, td, mpm, cpu, sys. */
static void ane_h16_power_down(struct ane_h16_device *h)
{
	unsigned int i;

	ane_h16_busy(h, false);
	ane_h16_irq_wait_set(h, false);
	for (i = ARRAY_SIZE(h->dart); i-- > 0; )
		pm_runtime_put_sync(h->dart[i]);
	for (i = h->powered; i-- > 0; )
		pm_runtime_put_sync(h->power[i]);
	h->engine_on = false;
	dev_info(h->dev, "H16 ANE: powered down for system sleep\n");
}

/* The five domains in order, then the DARTs (their resume restores SID0). */
static int ane_h16_power_up(struct ane_h16_device *h)
{
	unsigned int i, j;
	int ret;

	for (i = 0; i < h->powered; i++) {
		ret = pm_runtime_resume_and_get(h->power[i]);
		if (ret < 0)
			goto power;
	}
	for (j = 0; j < ARRAY_SIZE(h->dart); j++) {
		ret = pm_runtime_resume_and_get(h->dart[j]);
		if (ret < 0)
			goto dart;
	}
	h->engine_on = true;
	return 0;
dart:
	while (j--)
		pm_runtime_put_sync(h->dart[j]);
power:
	while (i--)
		pm_runtime_put_sync(h->power[i]);
	return ret;
}

/*
 * RVBAR must name TEXT. It keeps the latched entry while the domain stays
 * on; a real power-off may clear it, and the entry is written again only
 * while bit 0 (latched) is clear.
 */
static int ane_h16_rvbar_ready(struct ane_h16_device *h)
{
	struct ane_h16_boot *b = &h->boot;
	const struct ane_h16_build *f = b->build;
	u64 rvbar = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->rvbar);

	if (rvbar == f->expected_rvbar)
		return 0;
	if (rvbar & 1) {
		dev_err(h->dev, "H16 ANE: RVBAR %#llx latched at another entry\n", rvbar);
		return -EIO;
	}
	b->ops->write64(b->ctx, ANE_H16_ENGINE, f->rvbar, f->expected_rvbar & ~1ULL);
	rvbar = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->rvbar);
	dev_info(h->dev, "H16 ANE: RVBAR rewritten, reads %#llx\n", rvbar);
	return rvbar == f->expected_rvbar || rvbar == (f->expected_rvbar & ~1ULL) ? 0 : -EIO;
}

static int ane_h16_warm_boot(struct ane_h16_device *h)
{
	struct ane_h16_boot *b = &h->boot;
	const struct ane_h16_build *f = b->build;
	unsigned int i;
	int ret;

	/* The DART runtime resume restores TCR/TTBR and syncs the TLBs. */
	ret = ane_h16_power_up(h);
	if (ret)
		return ret;
	ret = ane_h16_dapf(b, true);
	if (!ret)
		ret = ane_h16_prepare(b);
	if (!ret)
		ret = ane_h16_rvbar_ready(h);
	if (ret)
		return ret;
	for (i = 0; i < 8; i++)
		ane_h16_write(b, i, 0);
	ane_h16_write(b, 6, 1);
	ane_h16_write(b, 7, ANE_H16_WARM_BOOT);
	b->ops->write32(b->ctx, ANE_H16_ENGINE, f->cpu_control, 0);
	b->transport_ops->write_barrier(b->ctx);
	b->ops->write32(b->ctx, ANE_H16_ENGINE, f->cpu_control, f->run);
	ret = ane_h16_wait(b, 7, f->ready, false);
	dev_info(h->dev, "H16 ANE: warm boot RVBAR %#llx SCRATCH7 %#x wrapper status %#x (%d)\n",
		 b->ops->read64(b->ctx, ANE_H16_ENGINE, f->rvbar), ane_h16_read(b, 7),
		 b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control + 4), ret);
	if (ret)
		return ret;
	ret = ane_fw_transport_check(&b->transport);
	h->resume_ready_ns = ktime_get_ns();
	if (!ret) {
		ane_h16_irq_wait_set(h, true);
		dev_info(h->dev, "H16 ANE: firmware resumed (warm boot); programs kept\n");
	}
	return ret;
}

/* Called by genpd under its lock, in the noirq phase too: record only. */
static int ane_h16_genpd_event(struct notifier_block *nb, unsigned long action, void *data)
{
	struct ane_h16_genpd *g = container_of(nb, struct ane_h16_genpd, nb);
	u64 now = ktime_get_boottime_ns();

	if (action == GENPD_NOTIFY_OFF) {
		g->offs++;
		g->off_at = now;
	} else if (action == GENPD_NOTIFY_ON) {
		g->ons++;
		if (g->off_at)
			g->off_ns += now - g->off_at;
		g->off_at = 0;
	}
	return NOTIFY_OK;
}

static void ane_h16_genpd_record(struct ane_h16_device *h)
{
	char line[160];
	int n = 0;

	for (unsigned int i = 0; i < h->powered; i++)
		n += scnprintf(line + n, sizeof(line) - n, "%s%u: off %u, on %u, %llu ms",
			       i ? "; " : "", i, h->genpd[i].offs, h->genpd[i].ons,
			       div_u64(h->genpd[i].off_ns, NSEC_PER_MSEC));
	dev_info(h->dev, "H16 ANE: domains while asleep (genpd): %s\n", line);
}

/*
 * Power the engine off (if on) and on again, then restore what a release of
 * the CPU from the preload needs: DAPF records, RVBAR, the verified DATA, a
 * cleared heap tail and fresh RTKit state. With @verify, the cpu, mpm, td and
 * base domains must each have reported an OFF transition, so the CPU and the
 * engine were reset. The first domain, sys, is their parent and stays on at
 * runtime under the DARTs, so with @verify it is then power cycled on its own
 * (ane_h16_sys_cycle()); in system sleep it goes off with the others.
 */
/*
 * Power the sys domain (the first, the parent of the other four) off and on
 * again. At runtime the ANE DARTs, which sit in it, hold it on, so it stays
 * on across a runtime power cycle of the other four. A firmware startup stall
 * on the J616s survives that, and is cleared once sys is off too, as it is in
 * system sleep. The DARTs keep their SID0 state across it, as they do across
 * system sleep (the ANE DART's suspend and resume neither save nor restore).
 */
static int ane_h16_sys_cycle(struct ane_h16_device *h)
{
	struct device_node *np = of_parse_phandle(h->dev->of_node, "power-domains", 0);
	int ret;

	if (!np)
		return -ENODEV;
	ret = apple_pmgr_ps_cycle(np);
	of_node_put(np);
	if (ret)
		dev_err(h->dev, "H16 ANE: sys domain power cycle failed, error %d\n", ret);
	return ret;
}

static int ane_h16_power_cycle(struct ane_h16_device *h, bool verify)
{
	struct ane_h16_boot *b = &h->boot;
	size_t size = b->soc->preload_size[1];
	unsigned int offs[ARRAY_SIZE(h->genpd)], i;
	void *live;
	int ret;

	if (!h->pristine_data)
		return -ENODATA;
	for (i = 0; i < h->powered; i++)
		offs[i] = h->genpd[i].offs;
	if (h->engine_on)
		ane_h16_power_down(h);
	for (i = 1; verify && i < h->powered; i++) {
		if (!h->genpd[i].added || h->genpd[i].offs == offs[i]) {
			dev_err(h->dev, "H16 ANE: domain %u did not power off; no restart\n", i);
			/* Powered again, so the quarantine can read its evidence. */
			ret = ane_h16_power_up(h);
			return ret ?: -EIO;
		}
	}
	if (verify) {
		ret = ane_h16_sys_cycle(h);
		if (ret) {
			ret = ane_h16_power_up(h) ?: ret;
			return ret;
		}
	}
	ret = ane_h16_power_up(h);
	if (!ret)
		ret = ane_h16_dapf(b, true);
	if (!ret)
		ret = ane_h16_rvbar_ready(h);
	if (ret)
		return ret;
	live = memremap(h->preload_phys[1], size, MEMREMAP_WB);
	if (!live)
		return -ENOMEM;
	memcpy(live, h->pristine_data, size);
	memunmap(live);
	if (h->heap_tail.cpu)
		memset(h->heap_tail.cpu, 0, h->heap_tail.size);
	b->transport_ops->write_barrier(b->ctx);
	memset(&h->rtk, 0, sizeof(h->rtk));
	h->rtk_failed = false;
	h->tx_read = 0;
	h->tx_write = 0;
	h->mode_stopped = false;
	return 0;
}

/*
 * The cold fallback after a failed warm boot: the engine is powered off
 * again, then booted from the preload as at bring-up (pristine DATA, heap
 * tail, pool, IPC and heap cleared and reused, RTKit restarted, 0x29 at
 * startup), the shared allocations carried over, and every open program
 * replayed behind its handle. The firmware can stop part way through this
 * startup as it does at bring-up; it gets the same bounded, verified restart,
 * with a budget of its own for each wake.
 */
static int ane_h16_boot_again(struct ane_h16_device *h, unsigned int *restarts,
			      const char *what, const char *why, bool recover)
{
	struct ane_h16_boot *b = &h->boot;
	struct ane_fw_transport *old;
	int ret;

	old = kmemdup(&b->transport, sizeof(*old), GFP_KERNEL);
	if (!old)
		return -ENOMEM;
	ret = ane_h16_scratch_boot(b);
	if (!ret)
		ane_fw_transport_carry_shared(&b->transport, old);
	kfree(old);
	if (!ret)
		ret = ane_h16_startup_restarting(b, restarts, what);
	if (ret)
		return ret;
	h->resume_ready_ns = ktime_get_ns();
	ane_h16_irq_wait_set(h, true);
	dev_info(h->dev, "H16 ANE: booted again from the preload %s\n", why);
	ane_h16_session_restart(&h->session);
	/* Its domains were verified off: a quarantine of the old firmware ends. */
	if (recover) {
		ane_h16_session_recovered(&h->session);
		b->quarantined = false;
		b->error = 0;
	}
	ret = ane_h16_session_replay(&h->session, &h->resume_replayed, &h->resume_first_ns);
	if (ret) {
		dev_err(h->dev, "H16 ANE: PROGRAM REPLAY FAILED %d after %u programs; session quarantined\n",
			ret, h->resume_replayed);
		WARN_ONCE(1, "H16 ANE: program replay failed %s (%d)\n", why, ret);
	}
	return ret;
}

static int ane_h16_cold_boot(struct ane_h16_device *h)
{
	int ret;

	ret = ane_h16_power_cycle(h, false);
	if (ret)
		return ret;
	return ane_h16_boot_again(h, &h->resume_restarts, "resume restart/budget",
				  "after system sleep", h->sleep_quarantined);
}

/*
 * Hold the ANE DARTs (and with them the sys domain) powered while a hung
 * engine goes off, so translation stays up for anything still in flight.
 * Returns the mask of DARTs held, for ane_h16_release_darts().
 */
static unsigned int ane_h16_hold_darts(struct ane_h16_device *h)
{
	unsigned int i, held = 0;

	for (i = 0; i < ARRAY_SIZE(h->dart); i++) {
		if (!h->dart[i])
			continue;
		if (pm_runtime_resume_and_get(h->dart[i]) < 0) {
			dev_warn(h->dev, "H16 ANE: %s not held powered\n", dev_name(h->dart[i]));
			continue;
		}
		held |= BIT(i);
	}
	return held;
}

static void ane_h16_release_darts(struct ane_h16_device *h, unsigned int held)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(h->dart); i++)
		if (held & BIT(i))
			pm_runtime_put(h->dart[i]);
}

/* Every domain reported an OFF transition since ane_h16_sleep_prepare(). */
static bool ane_h16_slept_off(struct ane_h16_device *h)
{
	for (unsigned int i = 0; i < h->powered; i++)
		if (!h->genpd[i].added || !h->genpd[i].offs)
			return false;
	return h->powered != 0;
}

static int ane_h16_sleep_prepare(struct ane_h16_device *h)
{
	int ret;

	guard(mutex)(&ane_h16_lock);
	for (unsigned int i = 0; i < ARRAY_SIZE(h->genpd); i++) {
		h->genpd[i].offs = 0;
		h->genpd[i].ons = 0;
		h->genpd[i].off_at = 0;
		h->genpd[i].off_ns = 0;
	}
	h->sleep_quarantined = false;
	if (!h->program_ready || (h->boot.quarantined && !ane_t6021_hung_stop))
		return -EBUSY;
	/* Without hung_stop a quarantined session stays so across the sleep. */
	if (ane_t6021_hung_stop && (h->boot.quarantined || h->session.quarantined)) {
		unsigned int held;

		/*
		 * The firmware cannot be asked for anything: power the engine
		 * off under it, DARTs held until its domains are off, and boot
		 * it cold on resume if every domain went off.
		 */
		ane_h16_session_sleep(&h->session, true);
		if (h->engine_on) {
			held = ane_h16_hold_darts(h);
			ane_h16_power_down(h);
			ane_h16_release_darts(h, held);
		}
		h->sleep_quarantined = true;
		dev_info(h->dev, "H16 ANE: quarantined engine powered off for system sleep\n");
		WRITE_ONCE(h->slept, true);
		return 0;
	}
	ane_h16_session_sleep(&h->session, true);
	if (!h->boot.soc->sleep_warm) {
		/* Cold cycle: nothing in flight, then power off under the firmware. */
		for (unsigned int ch = 0; ch < ANE_FW_CHANNELS; ch++) {
			if (h->boot.transport.pending[ch].active) {
				ane_h16_session_sleep(&h->session, false);
				return -EBUSY;
			}
		}
		ane_h16_power_down(h);
		WRITE_ONCE(h->slept, true);
		return 0;
	}
	ret = ane_h16_suspend_firmware(h);
	if (ret) {
		/* Powered, maybe suspended: keep everything, refuse, close. */
		ane_h16_quarantine(h, ret);
		h->boot.quarantined = true;
		h->session.quarantined = true;
		ane_h16_session_sleep(&h->session, false);
		return ret;
	}
	ane_h16_power_down(h);
	WRITE_ONCE(h->slept, true);
	return 0;
}

static void ane_h16_sleep_resume(struct ane_h16_device *h)
{
	s64 ready, first;
	u64 start;
	int ret;

	guard(mutex)(&ane_h16_lock);
	if (!h->slept)
		return;
	start = ktime_get_ns();
	h->resume_ready_ns = 0;
	h->resume_first_ns = 0;
	h->resume_replayed = 0;
	h->resume_cold = !h->boot.soc->sleep_warm || h->sleep_quarantined;
	h->resume_restarts = 0;
	if (h->sleep_quarantined && !ane_h16_slept_off(h)) {
		/* Not proven off: the old firmware may still run. Keep it. */
		dev_err(h->dev, "H16 ANE: a domain of the quarantined engine never went off; kept quarantined\n");
		ret = ane_h16_power_up(h);
		if (!ret)
			ret = -EIO;
	} else if (h->resume_cold) {
		ret = ane_h16_cold_boot(h);
	} else {
		ret = ane_h16_warm_boot(h);
		if (ret) {
			dev_warn(h->dev, "H16 ANE: warm boot failed (%d); cold boot and program replay\n",
				 ret);
			h->resume_cold = true;
			ret = ane_h16_cold_boot(h);
		}
	}
	WRITE_ONCE(h->slept, false);
	if (ret) {
		/* Every mapping and backing stays for diagnosis. */
		ane_h16_quarantine(h, ret);
		h->boot.quarantined = true;
		h->session.quarantined = true;
	} else if (h->sleep_quarantined) {
		h->recoveries++;
		dev_info(h->dev, "H16 ANE: quarantined engine recovered by system sleep\n");
	}
	h->sleep_quarantined = false;
	ane_h16_session_sleep(&h->session, false);
	ane_h16_genpd_record(h);
	/* One line per wake; times from PM_POST_SUSPEND, which follows the thaw. */
	ready = h->resume_ready_ns ? div_u64(h->resume_ready_ns - start, NSEC_PER_MSEC) : -1;
	first = h->resume_first_ns ? div_u64(h->resume_first_ns - start, NSEC_PER_MSEC) : -1;
	dev_info(h->dev,
		 "H16 ANE: resume %s: %s, firmware ready %lld ms, first program ready %lld ms, node reopened %llu ms, %u programs replayed, %u restarts\n",
		 ret ? "FAILED" : "ok", h->resume_cold ? "cold" : "warm", ready, first,
		 div_u64(ktime_get_ns() - start, NSEC_PER_MSEC), h->resume_replayed,
		 h->resume_restarts);
}

static int ane_h16_pm_event(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct ane_h16_device *h = container_of(nb, struct ane_h16_device, pm_notifier);
	bool pinned = READ_ONCE(h->pinned);

	/*
	 * Registered at probe, before the device is pinned: until then there
	 * is nothing to cycle or refuse. Pinning happens under the system
	 * sleep lock (ane_h16_start_work()), never inside a transition.
	 */
	if (!pinned)
		return NOTIFY_DONE;
	/* Without a sleep path on this platform, every sleep is refused as before. */
	if (!h->boot.soc->sleep_warm && !h->boot.soc->sleep_cold) {
		if (ane_fw_sleep_refused(event, pinned))
			dev_warn_once(h->dev, ANE_FW_SLEEP_REFUSED);
		return ane_fw_sleep_event(event, pinned);
	}
	if (event == PM_POST_SUSPEND) {
		ane_h16_sleep_resume(h);
		WRITE_ONCE(h->recover_blocked, false);
		return NOTIFY_DONE;
	}
	if (event == PM_SUSPEND_PREPARE && pinned) {
		/* A recovery under way finishes first; sleep handles later ones. */
		WRITE_ONCE(h->recover_blocked, true);
		flush_work(&h->recover);
		if (!ane_h16_sleep_prepare(h))
			return NOTIFY_DONE;
		WRITE_ONCE(h->recover_blocked, false);
		dev_warn_once(h->dev, ANE_FW_SLEEP_REFUSED);
		return NOTIFY_BAD;
	}
	/* Hibernation and restore stay refused. */
	if (ane_fw_sleep_refused_cycle(event, pinned, true)) {
		dev_warn_once(h->dev, ANE_FW_SLEEP_REFUSED);
		return NOTIFY_BAD;
	}
	return NOTIFY_DONE;
}

static int ane_h16_pin(void *ctx)
{
	struct ane_h16_device *h = ctx;

	/* A cold boot after sleep runs the boot again; pinned once for good. */
	if (h->pinned)
		return 0;
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	/*
	 * Pinned for good. The PM notifier was registered at probe: registering
	 * it here, under ane_h16_lock, would order the PM chain rwsem after
	 * ane_h16_lock, while ane_h16_pm_event() takes them the other way round.
	 */
	WRITE_ONCE(h->pinned, true);
	ane_h16_released = true;
	return 0;
}

/*
 * System sleep callbacks of a bound H16 device. While the firmware runs the
 * notifier normally refuses the transition first; refuse here too, as a
 * backstop for firmware started after it ran (the start work item is not
 * freezable) or a path that skipped the notifiers. A device whose engine
 * was never started has nothing to save.
 */
int ane_h16_backend_suspend(struct device *dev)
{
	struct ane_h16_device *h = dev_get_drvdata(dev);
	/* Running firmware that the notifier did not suspend: refuse. */
	bool running = h && READ_ONCE(h->pinned) && !READ_ONCE(h->slept);

	if (running)
		dev_warn_once(dev, ANE_FW_SLEEP_REFUSED);
	return ane_fw_sleep_backstop(running);
}

int ane_h16_backend_resume(struct device *dev)
{
	return 0;
}

/* Nonblocking transmit queue. RTKit progress never gates scratch progress. */
static int ane_h16_mailbox_send(void *ctx, u64 word0, u64 word1)
{
	struct ane_h16_device *h = ctx;

	if (h->tx_write - h->tx_read == ARRAY_SIZE(h->tx))
		return -ENOSPC;
	h->tx[h->tx_write % ARRAY_SIZE(h->tx)].word0 = word0;
	h->tx[h->tx_write % ARRAY_SIZE(h->tx)].word1 = word1;
	h->tx_write++;
	return 0;
}

static void ane_h16_mailbox(void *ctx)
{
	struct ane_h16_device *h = ctx;
	void __iomem *mb = h->window[ANE_H16_ENGINE] + h->boot.soc->mailbox;
	unsigned int i;

	for (i = 0; i < 16; i++) {
		u64 word0, word1;
		int ret;

		if (readl(mb + 0x114) & BIT(17))
			break;
		/* Two 64-bit accesses. Reading the second word pops the entry. */
		word0 = readq(mb + 0x830);
		word1 = readq(mb + 0x838);
		dev_info(h->dev, "H16 ASC RX %016llx %016llx\n", word0, word1);
		if (h->rtk_failed)
			continue;
		ret = ane_fw_rtk_receive(&h->rtk, word0, word1, h, ane_h16_mailbox_send);
		if (ret) {
			dev_info(h->dev,
				 "H16 RTKit message declined %d; scratch remains independent\n",
				 ret);
			if (ret != -EOPNOTSUPP)
				h->rtk_failed = true;
		}
	}
	for (i = 0; i < 16 && h->tx_read != h->tx_write; i++) {
		u32 index = h->tx_read % ARRAY_SIZE(h->tx);

		if (readl(mb + 0x110) & BIT(16))
			break;
		writeq(h->tx[index].word0, mb + 0x800);
		/* Publish word 0 before the word-1 FIFO commit. */
		wmb();
		writeq(h->tx[index].word1, mb + 0x808);
		dev_info(h->dev, "H16 ASC TX %016llx %016llx\n",
			 h->tx[index].word0, h->tx[index].word1);
		h->tx_read++;
	}
}

static int ane_h16_map_range(struct ane_h16_device *h, u64 dva, u64 pa,
			     size_t size, int prot)
{
	struct ane_h16_mapping *m;
	int ret;

	m = kzalloc_obj(*m);
	if (!m)
		return -ENOMEM;
	m->dva = dva;
	m->size = size;
	mutex_lock(&h->window_lock);
	list_add_tail(&m->node, &h->mappings);
	mutex_unlock(&h->window_lock);
	ret = iommu_map(h->domain, dva, pa, size, prot, GFP_KERNEL);
	if (!ret)
		m->mapped = size;
	return ret;
}

static int ane_h16_count_group(struct device *dev, void *data)
{
	unsigned int *count = data;

	(*count)++;
	return 0;
}

static int ane_h16_alloc_prot(struct ane_h16_device *h, u64 dva, size_t size, int prot,
			      struct ane_fw_surface *out);

/*
 * The firmware span is config_span bytes from the TEXT DVA. The preload covers
 * TEXT and DATA only; the firmware keeps runtime heap state in the rest, which
 * the host supplies zeroed and device-writable before boot.
 */
static int ane_h16_map_heap_tail(struct ane_h16_device *h)
{
	struct ane_h16_boot *b = &h->boot;
	u64 tail = b->soc->preload_dva[1] + b->soc->preload_size[1];
	u64 end = b->soc->preload_dva[0] + b->build->config_span;
	struct ane_fw_surface s;
	int ret;

	if (tail == end)
		return 0;
	if (tail > end || !IS_ALIGNED(tail, SZ_16K) || !IS_ALIGNED(end, SZ_16K) ||
	    end > b->build->pool_dva)
		return -ERANGE;
	ret = ane_h16_alloc_prot(h, tail, end - tail,
				 IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE, &s);
	if (ret)
		return ret;
	h->heap_tail = s;
	dev_info(h->dev, "H16 firmware heap tail mapped at %llx bytes %llx writable\n",
		 tail, end - tail);
	return 0;
}

static int ane_h16_map(void *ctx)
{
	struct ane_h16_device *h = ctx;
	struct ane_h16_boot *b = &h->boot;
	unsigned int count = 0, i;
	int ret;

	/* Unmanaged domains can otherwise attach retained, firmware-owned roots. */
	ret = ane_h16_dart_protection(b, true);
	if (ret)
		return ret;
	if (of_count_phandle_with_args(h->dev->of_node, "iommus", "#iommu-cells") != 3) {
		dev_err(h->dev, "H16 map requires only SID 0 on three DARTs; refusing extra streams\n");
		return -EOPNOTSUPP;
	}
	h->group = iommu_group_get(h->dev);
	if (!h->group)
		return -ENODEV;
	ret = iommu_group_for_each_dev(h->group, &count, ane_h16_count_group);
	if (ret || count != 1)
		return ret ?: -EBUSY;
	ret = iommu_device_claim_dma_owner(h->dev, h);
	if (ret)
		return ret;
	h->claimed = true;
	h->domain = iommu_paging_domain_alloc(h->dev);
	if (IS_ERR_OR_NULL(h->domain)) {
		ret = h->domain ? PTR_ERR(h->domain) : -ENOMEM;
		h->domain = NULL;
		return ret;
	}
	if (h->domain->type != IOMMU_DOMAIN_UNMANAGED ||
	    !(h->domain->pgsize_bitmap & SZ_16K))
		return -EOPNOTSUPP;
	/* Never widen a finalized DART aperture by assigning geometry fields. */
	for (i = 0; i < b->soc->dapf_count; i++) {
		u64 page = round_down(b->ranges[i][0], SZ_16K);

		if (!ane_fw_contains(h->domain->geometry.aperture_start,
				     h->domain->geometry.aperture_end -
				     h->domain->geometry.aperture_start + 1,
				     page, SZ_16K))
			return -ERANGE;
	}
	if (h->domain->geometry.aperture_end < b->build->shared_dva + ANE_FW_SHARED_TOTAL)
		return -ERANGE;
	if (h->domain->geometry.aperture_end < h->program_end - 1)
		return -ERANGE;
	ret = iommu_attach_device(h->domain, h->dev);
	if (ret)
		return ret;
	h->attached = true;
	for (i = 0; i < 2; i++) {
		ret = ane_h16_map_range(h, b->soc->preload_dva[i], h->preload_phys[i],
					b->soc->preload_size[i],
					IOMMU_READ | IOMMU_CACHE | (i ? IOMMU_WRITE : 0));
		if (ret)
			return ret;
		dev_info(h->dev, "H16 preload %s mapped at %llx bytes %llx\n",
			 i ? "DATA" : "TEXT", b->soc->preload_dva[i], b->soc->preload_size[i]);
	}
	ret = ane_h16_map_heap_tail(h);
	if (ret)
		return ret;
	for (i = 0; i < b->soc->dapf_count; i++) {
		u64 page = round_down(b->ranges[i][0], SZ_16K);

		ret = ane_h16_map_range(h, page, page, SZ_16K,
					IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO);
		if (ret)
			return ret;
		dev_info(h->dev, "H16 identity MMIO %llx mapped\n", page);
	}
	return 0;
}

/* Memory only: off h->mappings, its range (if any) already given back. */
static void ane_h16_mapping_free(struct ane_h16_mapping *m)
{
	unsigned int i, pages_per_block = SZ_16K / PAGE_SIZE;

	if (m->cpu)
		vunmap(m->cpu);
	for (i = 0; i < m->blocks; i++)
		__free_pages(m->pages[i * pages_per_block], get_order(SZ_16K));
	kvfree(m->pages);
	kfree(m);
}

/*
 * Map size bytes of fresh zeroed memory at dva, or with ANE_H16_PROGRAM_ANY
 * at a range of the program window. Memory that never reached the IOMMU is
 * freed on failure; a failed or partial iommu_map_sgtable() keeps the
 * mapping, its memory and its range until reset.
 */
static int ane_h16_alloc_prot(struct ane_h16_device *h, u64 dva, size_t size, int prot,
			      struct ane_fw_surface *out)
{
	struct ane_h16_mapping *m;
	u32 i, j, pages_per_block = SZ_16K / PAGE_SIZE;
	unsigned int order = get_order(SZ_16K);
	struct sg_table sgt;
	ssize_t mapped;
	int ret;

	if (!dev_is_dma_coherent(h->dev)) {
		dev_err(h->dev, "H16 private surfaces require a coherent client\n");
		return -EOPNOTSUPP;
	}
	if (!size || !IS_ALIGNED(size, SZ_16K) || PAGE_SIZE > SZ_16K ||
	    (dva != ANE_H16_PROGRAM_ANY && !IS_ALIGNED(dva, SZ_16K)))
		return -EINVAL;
	m = kzalloc_obj(*m);
	if (!m)
		return -ENOMEM;
	m->size = size;
	ret = -ENOMEM;
	m->pages = kvcalloc(size / PAGE_SIZE, sizeof(*m->pages), GFP_KERNEL);
	if (!m->pages)
		goto free;
	for (i = 0; i < size / SZ_16K; i++) {
		struct page *page = alloc_pages(GFP_KERNEL | __GFP_ZERO, order);

		if (!page)
			goto free;
		for (j = 0; j < pages_per_block; j++)
			m->pages[i * pages_per_block + j] = page + j;
		m->blocks++;
	}
	/* The coherent-client gate permits matching cached CPU/device aliases. */
	m->cpu = vmap(m->pages, size / PAGE_SIZE, VM_MAP, PAGE_KERNEL);
	if (!m->cpu)
		goto free;
	ret = sg_alloc_table_from_pages(&sgt, m->pages, size / PAGE_SIZE, 0, size, GFP_KERNEL);
	if (ret)
		goto free;
	mutex_lock(&h->window_lock);
	if (dva == ANE_H16_PROGRAM_ANY) {
		ret = drm_mm_insert_node_in_range(&h->program_window, &m->range, size,
						  SZ_16K, 0, 0, ANE_H16_PROGRAM_WINDOW,
						  DRM_MM_INSERT_LOW);
		if (ret) {
			mutex_unlock(&h->window_lock);
			sg_free_table(&sgt);
			goto free;
		}
		dva = h->program_start + m->range.start;
	}
	m->dva = dva;
	/* Recorded before publication, including a failed/partial mapping. */
	list_add_tail(&m->node, &h->mappings);
	mutex_unlock(&h->window_lock);
	/* A failed call has already unmapped what it mapped. */
	mapped = iommu_map_sgtable(h->domain, dva, &sgt, prot);
	sg_free_table(&sgt);
	if (mapped < 0)
		return mapped;
	m->mapped = mapped;
	*out = (struct ane_fw_surface) { .cpu = m->cpu, .dva = dva, .size = size };
	return 0;
free:
	ane_h16_mapping_free(m);
	return ret;
}

static int ane_h16_alloc_at(void *ctx, u64 dva, size_t size, struct ane_fw_surface *out)
{
	return ane_h16_alloc_prot(ctx, dva, size, IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE, out);
}

/* Program sections and surfaces. Only outputs and scratch are device-writable. */
static int ane_h16_program_map(void *ctx, size_t size, bool write, struct ane_fw_surface *out)
{
	struct ane_h16_device *h = ctx;
	int ret;

	if (!size || size > ANE_H16_PROGRAM_WINDOW)
		return -ENOSPC;
	ret = ane_h16_alloc_prot(h, ANE_H16_PROGRAM_ANY, size,
				 IOMMU_READ | IOMMU_CACHE | (write ? IOMMU_WRITE : 0), out);
	if (!ret)
		dev_info(h->dev, "H16 program mapping %llx bytes %zx%s\n", out->dva, size,
			 write ? " writable" : "");
	return ret;
}

/*
 * A program surface the firmware no longer uses (ops->unmap). The session
 * calls it with the engine up and its lock held. iommu_unmap() ends with
 * the DART TLB invalidation, so the engine cannot reach the memory once it
 * returns; only then do the pages and the DVA range go back. A short unmap
 * keeps everything, as a failed mapping does. Only program-window mappings
 * qualify: SHAREDMALLOC, preload, heap tail and MMIO stay until reset.
 */
static void ane_h16_program_unmap(void *ctx, const struct ane_fw_surface *s)
{
	struct ane_h16_device *h = ctx;
	struct ane_h16_mapping *m, *found = NULL;
	size_t unmapped;

	mutex_lock(&h->window_lock);
	list_for_each_entry(m, &h->mappings, node) {
		if (m->dva == s->dva && m->size == s->size && m->mapped == m->size &&
		    drm_mm_node_allocated(&m->range)) {
			found = m;
			list_del(&m->node);
			break;
		}
	}
	mutex_unlock(&h->window_lock);
	if (WARN_ON_ONCE(!found))
		return;
	m = found;
	unmapped = iommu_unmap(h->domain, m->dva, m->size);
	if (unmapped != m->size) {
		dev_err(h->dev, "H16 program unmap short at %llx: %zx of %zx; kept\n",
			m->dva, unmapped, m->size);
		mutex_lock(&h->window_lock);
		list_add_tail(&m->node, &h->mappings);
		mutex_unlock(&h->window_lock);
		return;
	}
	dev_dbg(h->dev, "H16 program unmapped %llx bytes %zx\n", m->dva, m->size);
	mutex_lock(&h->window_lock);
	drm_mm_remove_node(&m->range);
	mutex_unlock(&h->window_lock);
	ane_h16_mapping_free(m);
}

static u64 ane_h16_load64(void *ctx, const void *p)
{
	return le64_to_cpu(READ_ONCE(*(const __le64 *)p));
}

static void ane_h16_store64(void *ctx, void *p, u64 value)
{
	WRITE_ONCE(*(__le64 *)p, cpu_to_le64(value));
}

static void ane_h16_dma_read_barrier(void *ctx)
{
	dma_rmb();
}

static void ane_h16_dma_write_barrier(void *ctx)
{
	dma_wmb();
}

static u32 ane_h16_engine_read(void *ctx, u32 offset)
{
	return ane_h16_mmio_read32(ctx, ANE_H16_ENGINE, offset);
}

static void ane_h16_engine_write(void *ctx, u32 offset, u32 value)
{
	struct ane_h16_device *h = ctx;

	ane_h16_mmio_write32(ctx, ANE_H16_ENGINE, offset, value);
	if (offset == h->boot.soc->irq_ack) {
		/* Complete the snapshot acknowledgment before ring reads (r5). */
		mb();
	}
}

static u64 ane_h16_now(void *ctx)
{
	return ktime_get_ns();
}

/*
 * Every wait polls, with the shared pacing (see ane_fw_poll_sleep_us()):
 * busy steps for the first 30 us, then short sleeps. A fixed 1 ms sleep
 * made each of the four waits of an EXECUTE cost at least a millisecond.
 *
 * After the busy window an EXECUTE reply (typically 40-50 us) landed in a
 * 20-50 us sleep and was seen 40-50 us late. Once the firmware is up, the
 * sleep therefore also ends on the channel IRQ, the way macOS drains the
 * rings from IRQ 1054; the timeout keeps the polled cadence as a bound.
 */
static void ane_h16_irq_pause(struct ane_h16_device *h, u64 waited_ns)
{
	unsigned int min_us, max_us;
	unsigned long seq = 0;
	bool armed = false;
	long ret;

	min_us = ane_fw_poll_sleep_us(waited_ns, &max_us);
	scoped_guard(spinlock_irqsave, &h->irq_lock) {
		if (h->irq_wait) {
			seq = h->irq_seq;
			if (!h->irq_armed) {
				h->irq_armed = true;
				enable_irq(h->irq);
			}
			armed = true;
		}
	}
	if (!armed) {
		ane_fw_poll_pause(waited_ns);
		return;
	}
	ret = wait_event_hrtimeout(h->irq_wq, READ_ONCE(h->irq_seq) != seq,
				   us_to_ktime(min_us));
	scoped_guard(spinlock_irqsave, &h->irq_lock) {
		if (ret)
			h->irq_timeouts++;
		else
			h->irq_wakeups++;
	}
}

/* The waiting half of relax(); callers may hold no lock. */
static void ane_h16_pause(void *ctx, u64 waited_ns)
{
	struct ane_h16_device *h = ctx;

	if (READ_ONCE(h->irq_wait) && waited_ns >= ANE_FW_POLL_SPIN_NS &&
	    !READ_ONCE(h->boot.transport.error))
		ane_h16_irq_pause(h, waited_ns);
	else
		ane_fw_poll_pause(waited_ns);
}

static void ane_h16_service(void *ctx)
{
	struct ane_h16_device *h = ctx;
	int ret;

	ane_h16_mailbox(h);
	ret = ane_h16_faults(&h->boot, false);
	if (ret)
		ane_fw_transport_fail(&h->boot.transport, ret);
}

static void ane_h16_relax(void *ctx, u64 waited_ns)
{
	ane_h16_service(ctx);
	ane_h16_pause(ctx, waited_ns);
}

static int ane_h16_alloc_shared(void *ctx, size_t size, u32 tag, struct ane_fw_surface *out)
{
	struct ane_h16_device *h = ctx;
	int ret = ane_h16_alloc_at(h, h->shared_next, size, out);

	if (!ret) {
		h->shared_next += size;
		dev_info(h->dev, "H16 SHAREDMALLOC bytes %zx tag %08x DVA %llx\n",
			 size, tag, out->dva);
	}
	return ret;
}

static void ane_h16_refused(void *ctx, u64 w0, u64 w1, u64 w2, int error)
{
	struct ane_h16_device *h = ctx;

	dev_err(h->dev, "H16 SHAREDMALLOC refused %llx %llx %llx error %d\n",
		w0, w1, w2, error);
}

static void ane_h16_shared_note(void *ctx, const char *event, u64 dva, u64 size, u32 tag)
{
	struct ane_h16_device *h = ctx;

	dev_info(h->dev, "H16 SHAREDMALLOC %s bytes %llx tag %08x DVA %llx\n",
		 event, size, tag, dva);
}

/*
 * The firmware's own text log (its first SHAREDMALLOC) after a startup that
 * stopped: where the firmware got to. Read only.
 */
static void ane_h16_startup_fwlog(struct ane_h16_device *h)
{
	struct ane_fw_transport *t = &h->boot.transport;
	const struct ane_fw_surface *log = ane_fw_transport_shared_tagged(t, ANE_FW_SHARED_LOG);
	char line[ANE_H16_FWLOG_RECORD];
	unsigned int i, lines = 0;

	if (!log || !log->cpu)
		return;
	t->ops->read_barrier(t->ctx);
	for (i = 0; i < log->size / ANE_H16_FWLOG_RECORD; i++) {
		const u8 *r = (const u8 *)log->cpu + i * ANE_H16_FWLOG_RECORD;

		if (!ane_h16_fwlog_line(r, line))
			continue;
		dev_info(h->dev, "H16 startup fwlog[%03x] %s\n", i, line);
		lines++;
	}
	dev_info(h->dev, "H16 startup fwlog records %u\n", lines);
}

/*
 * Power cycles allowed after a startup timeout or stop at bring-up. On the
 * J616s the firmware's CPU is stopped part way through startup on about one
 * boot in six: it posts no further request and never answers the pending
 * command.
 */
#define ANE_H16_STARTUP_RESTARTS 2

/*
 * A startup that timed out at bring-up: keep the log, then a verified power
 * cycle. Every mapping stays; nothing is released while the engine may hold it.
 */
static bool ane_h16_powered(void *ctx)
{
	return ((struct ane_h16_device *)ctx)->engine_on;
}

static int ane_h16_restart_engine(void *ctx)
{
	struct ane_h16_device *h = ctx;
	int ret;

	ane_h16_startup_fwlog(h);
	ret = ane_h16_power_cycle(h, true);
	if (ret)
		dev_err(h->dev, "H16 ANE: startup timed out; power cycle failed, error %d\n", ret);
	else
		dev_warn(h->dev, "H16 ANE: startup timed out; engine power cycled, booting again\n");
	return ret;
}

static const struct ane_h16_boot_ops ane_h16_ops = {
	.read32 = ane_h16_mmio_read32,
	.read64 = ane_h16_mmio_read64,
	.write32 = ane_h16_mmio_write32,
	.write64 = ane_h16_mmio_write64,
	.delay_us = ane_h16_delay,
	.map = ane_h16_map,
	.alloc_at = ane_h16_alloc_at,
	.pin = ane_h16_pin,
	.mailbox = ane_h16_mailbox,
	.log = ane_h16_log,
	.quarantine = ane_h16_quarantine,
	.restart = ane_h16_restart_engine,
	.powered = ane_h16_powered,
};

static void ane_h16_busy(void *ctx, bool busy);

/*
 * On the J616s the firmware's CPU is sometimes stopped part way through its
 * startup, 8 to 12 ms after RUN: its wrapper control word reads 0 (RUN
 * cleared) and the status 0x30006a. It then answers nothing until it is
 * power cycled. Seen during startup only, so only watched there.
 */
static bool ane_h16_cpu_stopped(void *ctx)
{
	struct ane_h16_device *h = ctx;
	const struct ane_h16_build *f = h->boot.build;

	if (!h->boot.starting || !h->engine_on)
		return false;
	return !(ane_h16_mmio_read32(h, ANE_H16_ENGINE, f->cpu_control) & f->run);
}

/*
 * Recovery of a session quarantined at runtime by a hung request, with the
 * restart sequence of the T8112 path (ane_t6021_restart_run(), unplanned):
 * RUN cleared (halt), hung_settle_ms for the engine to drain (contain), the
 * CPU and engine domains powered off through genpd with the DARTs held on,
 * cpu, mpm, td and base each verified off before sys is cycled on its own
 * (power_cycle), then the firmware booted again from the preload with every
 * open program replayed behind its handle (boot). The H16 wrapper status
 * cannot prove a stop, so the verified power-off is the proof: halt always
 * reports a busy core, and without hung_stop the session stays quarantined.
 * Nothing asserts a reset. Requests wait in ane_h16_session_enter() while
 * it runs.
 */
static int ane_h16_rs_planned_only(void *ctx)
{
	return -EOPNOTSUPP;
}

static int ane_h16_rs_halt(void *ctx)
{
	struct ane_h16_device *h = ctx;
	struct ane_h16_boot *b = &h->boot;
	const struct ane_h16_build *f = b->build;

	if (!h->engine_on)
		return -ENODEV;
	dev_info(h->dev, "H16 ANE: hung engine: wrapper control %#x status %#x\n",
		 b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control),
		 b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control + 4));
	b->ops->write32(b->ctx, ANE_H16_ENGINE, f->cpu_control, 0);
	b->transport_ops->write_barrier(b->ctx);
	return ANE_T6021_ASC_BUSY;
}

static int ane_h16_rs_contain(void *ctx)
{
	struct ane_h16_device *h = ctx;
	struct ane_h16_boot *b = &h->boot;
	unsigned int settle = min(ane_t6021_hung_settle_ms, 60000U);

	if (!ane_t6021_hung_stop)
		return -EOPNOTSUPP;
	msleep(settle);
	dev_info(h->dev,
		 "H16 ANE: RUN cleared, wrapper status %#x after %u ms; powering the ANE off through its domains\n",
		 b->ops->read32(b->ctx, ANE_H16_ENGINE, b->build->cpu_control + 4), settle);
	return 0;
}

static int ane_h16_rs_power_cycle(void *ctx)
{
	struct ane_h16_device *h = ctx;
	unsigned int held = ane_h16_hold_darts(h);
	int ret = ane_h16_power_cycle(h, true);

	ane_h16_release_darts(h, held);
	return ret;
}

static int ane_h16_rs_boot(void *ctx)
{
	struct ane_h16_device *h = ctx;
	unsigned int restarts = 0;

	return ane_h16_boot_again(h, &restarts, "recovery restart/budget",
				  "after a hung request", true);
}

static const struct ane_t6021_restart_ops ane_h16_restart_ops = {
	.power_down = ane_h16_rs_planned_only,
	.wait_idle = ane_h16_rs_planned_only,
	.halt = ane_h16_rs_halt,
	.contain = ane_h16_rs_contain,
	.power_cycle = ane_h16_rs_power_cycle,
	.boot = ane_h16_rs_boot,
};

static void ane_h16_recover_work(struct work_struct *work)
{
	struct ane_h16_device *h = container_of(work, struct ane_h16_device, recover);
	static const char * const stages[] = {
		[ANE_T6021_RESTART_STOP] = "stop",
		[ANE_T6021_RESTART_CYCLE] = "power cycle",
		[ANE_T6021_RESTART_BOOT] = "boot",
		[ANE_T6021_RESTART_DONE] = "done",
	};
	enum ane_t6021_restart_stage stage;
	u64 start = ktime_get_ns();
	int ret;

	guard(mutex)(&ane_h16_lock);
	if (READ_ONCE(h->recover_blocked) || h->slept || !h->program_ready ||
	    h->boot.quarantined || !h->session.quarantined || !ane_t6021_hung_stop) {
		WRITE_ONCE(h->recovering, false);
		return;
	}
	ane_h16_session_sleep(&h->session, true);
	h->resume_replayed = 0;
	h->resume_first_ns = 0;
	ret = ane_t6021_restart_run(&ane_h16_restart_ops, h, false, &stage);
	if (ret) {
		h->recover_failures++;
		dev_err(h->dev,
			"H16 ANE: recovery after a hung request failed at the %s, error %d; quarantined until the next system sleep\n",
			stages[stage], ret);
	} else {
		h->recoveries++;
		dev_info(h->dev,
			 "H16 ANE: firmware recovered after a hung request in %llu ms, %u programs replayed\n",
			 div_u64(ktime_get_ns() - start, NSEC_PER_MSEC), h->resume_replayed);
	}
	ane_h16_session_sleep(&h->session, false);
	WRITE_ONCE(h->recovering, false);
}

/* Session ops->hung: may run under the session lock, so only queue. */
static void ane_h16_hung(void *ctx, int error)
{
	struct ane_h16_device *h = ctx;

	if (!ane_t6021_hung_stop || READ_ONCE(h->recover_blocked) || READ_ONCE(h->recovering))
		return;
	WRITE_ONCE(h->recovering, true);
	dev_warn(h->dev, "H16 ANE: session quarantined (%d); recovering the engine\n", error);
	queue_work(system_dfl_wq, &h->recover);
}

static const struct ane_h16_session_ops ane_h16_session_ops = {
	.map = ane_h16_program_map,
	.unmap = ane_h16_program_unmap,
	.log = ane_h16_log,
	.busy = ane_h16_busy,
	.hung = ane_h16_hung,
};

static const struct ane_fw_transport_ops ane_h16_transport_ops = {
	.load64 = ane_h16_load64,
	.store64 = ane_h16_store64,
	.read_barrier = ane_h16_dma_read_barrier,
	.write_barrier = ane_h16_dma_write_barrier,
	.read32 = ane_h16_engine_read,
	.write32 = ane_h16_engine_write,
	.now_ns = ane_h16_now,
	.relax = ane_h16_relax,
	.pause = ane_h16_pause,
	.service = ane_h16_service,
	.alloc = ane_h16_alloc_shared,
	.refused = ane_h16_refused,
	.shared_note = ane_h16_shared_note,
	.stopped = ane_h16_cpu_stopped,
};

/* Pre-RUN rollback of everything a stage acquired. h itself stays. */
static void ane_h16_release(struct ane_h16_device *h)
{
	struct ane_h16_mapping *m, *next;
	unsigned int i;

	if (h->boot.ran || h->pinned || !h->acquired)
		return;
	list_for_each_entry_safe_reverse(m, next, &h->mappings, node) {
		if (m->mapped)
			iommu_unmap(h->domain, m->dva, m->mapped);
		list_del(&m->node);
		if (drm_mm_node_allocated(&m->range))
			drm_mm_remove_node(&m->range);
		ane_h16_mapping_free(m);
	}
	if (h->attached)
		iommu_detach_device(h->domain, h->dev);
	h->attached = false;
	if (h->domain)
		iommu_domain_free(h->domain);
	h->domain = NULL;
	if (h->claimed)
		iommu_device_release_dma_owner(h->dev);
	h->claimed = false;
	if (h->group)
		iommu_group_put(h->group);
	h->group = NULL;
	if (h->irq >= 0)
		free_irq(h->irq, h);
	h->irq = -1;
	for (i = ANE_H16_WINDOWS; i-- > 0; ) {
		if (h->window[i])
			iounmap(h->window[i]);
		h->window[i] = NULL;
	}
	while (h->darts_powered)
		pm_runtime_put_sync(h->dart[--h->darts_powered]);
	for (i = 0; i < 3; i++) {
		if (h->dart[i])
			put_device(h->dart[i]);
		h->dart[i] = NULL;
	}
	if (h->perf) {
		if (h->perf_held)
			pm_runtime_put_sync(h->perf);
		h->perf_held = false;
		pm_runtime_dont_use_autosuspend(h->perf);
		dev_pm_domain_detach(h->perf, false);
		h->perf = NULL;
	}
	apple_clpc_ane_put(h->clpc);
	h->clpc = NULL;
	while (h->powered) {
		struct device *pd = h->power[--h->powered];

		if (h->genpd[h->powered].added)
			dev_pm_genpd_remove_notifier(pd);
		h->genpd[h->powered].added = false;
		pm_runtime_put_sync(pd);
		dev_pm_domain_detach(pd, true);
	}
	h->acquired = false;
	vfree(h->pristine_data);
	h->pristine_data = NULL;
	dev_info(h->dev, "H16 pre-RUN rollback complete; no CPU release\n");
}

/* Wake the sleeping wait and stay disabled until relax() arms it again. */
static irqreturn_t ane_h16_irq(int irq, void *data)
{
	struct ane_h16_device *h = data;

	scoped_guard(spinlock_irqsave, &h->irq_lock) {
		if (h->irq_armed) {
			disable_irq_nosync(irq);
			h->irq_armed = false;
		}
		WRITE_ONCE(h->irq_seq, h->irq_seq + 1);
	}
	wake_up(&h->irq_wq);
	return IRQ_HANDLED;
}

/*
 * The optional "perf" domain is a performance floor (the PMC SoC-max floor
 * on the M4 Pro): engine time roughly halves with the SoC and fabric at
 * their highest state. Attached powered off, it is voted for each EXECUTE
 * and released 100 ms after the last one ends (autosuspend), so a burst of
 * requests keeps it and an idle engine does not hold the SoC up.
 */
#define ANE_H16_PERF_IDLE_MS 100

static int ane_h16_attach_perf(struct ane_h16_device *h)
{
	struct device *pd = dev_pm_domain_attach_by_id(h->dev, 5);

	if (IS_ERR_OR_NULL(pd))
		return pd ? PTR_ERR(pd) : -ENODEV;
	pm_runtime_set_autosuspend_delay(pd, ANE_H16_PERF_IDLE_MS);
	pm_runtime_use_autosuspend(pd);
	h->perf = pd;
	dev_info(h->dev, "H16 perf floor attached, held per request (%u ms idle)\n",
		 ANE_H16_PERF_IDLE_MS);
	return 0;
}

static void ane_h16_busy(void *ctx, bool busy)
{
	struct ane_h16_device *h = ctx;

	apple_clpc_ane_set_active(h->clpc, busy);
	if (!h->perf)
		return;
	if (busy) {
		/* Best effort: without the floor the request still runs, slower. */
		if (!h->perf_held && pm_runtime_resume_and_get(h->perf) >= 0)
			h->perf_held = true;
	} else if (h->perf_held) {
		pm_runtime_mark_last_busy(h->perf);
		pm_runtime_put_autosuspend(h->perf);
		h->perf_held = false;
	}
}

static int ane_h16_acquire(struct platform_device *pdev, struct ane_h16_device *h);

/* Scratch, wrapper status and the rings around the sleep handshake. */
static void ane_h16_sleep_trace(struct ane_h16_device *h, const char *when)
{
	struct ane_fw_transport *t = &h->boot.transport;
	const struct ane_h16_build *f = h->boot.build;
	u8 *io = ane_fw_transport_slot(t, ANE_FW_IO);
	u8 *t2h = ane_fw_transport_slot(t, ANE_FW_IO_T2H);
	u8 *shm = ane_fw_transport_slot(t, ANE_FW_SHAREDMALLOC);

	dev_info(h->dev, "H16 ANE: %s: scratch %#x %#x %#x %#x %#x %#x %#x %#x wrapper %#x\n", when,
		 ane_h16_read(&h->boot, 0), ane_h16_read(&h->boot, 1), ane_h16_read(&h->boot, 2),
		 ane_h16_read(&h->boot, 3), ane_h16_read(&h->boot, 4), ane_h16_read(&h->boot, 5),
		 ane_h16_read(&h->boot, 6), ane_h16_read(&h->boot, 7),
		 h->boot.ops->read32(h->boot.ctx, ANE_H16_ENGINE, f->cpu_control + 4));
	dev_info(h->dev, "H16 ANE: %s: IO head %u %#llx %#llx %#llx; T2H head %u %#llx %#llx; SHAREDMALLOC head %u %#llx %#llx; irq %#x; error %d at ane_fw_transport.h:%u\n",
		 when, t->head[ANE_FW_IO], get_unaligned_le64(io), get_unaligned_le64(io + 8),
		 get_unaligned_le64(io + 16), t->head[ANE_FW_IO_T2H], get_unaligned_le64(t2h),
		 get_unaligned_le64(t2h + 8), t->head[ANE_FW_SHAREDMALLOC],
		 get_unaligned_le64(shm), get_unaligned_le64(shm + 8),
		 t->ops->read32(t->ctx, t->soc->irq_status), t->error, t->error_line);
}

static int ane_h16_acquire(struct platform_device *pdev, struct ane_h16_device *h)
{
	struct device_node *np = pdev->dev.of_node;
	static const char * const names[] = { "engine", "pmgr", "aux-3" };
	unsigned int i, j;
	bool perf;
	int ret, irq, n;

	/* SET_SNE_PMU_BASE2 at startup, for the warm sleep path only. */
	if (h->boot.soc->sleep_warm) {
		struct resource *pmgr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmgr");

		h->boot.pmu_base = pmgr ? pmgr->start : 0;
	}
	n = of_count_phandle_with_args(np, "power-domains", "#power-domain-cells");
	perf = n == 6 && of_property_match_string(np, "power-domain-names", "perf") == 5;
	if (n != 5 && !perf)
		return -EINVAL;
	for (i = 0; i < 5; i++) {
		struct device *pd = dev_pm_domain_attach_by_id(h->dev, i);

		if (IS_ERR_OR_NULL(pd))
			return pd ? PTR_ERR(pd) : -ENODEV;
		ret = pm_runtime_resume_and_get(pd);
		if (ret < 0) {
			dev_pm_domain_detach(pd, true);
			return ret;
		}
		h->genpd[h->powered].h = h;
		h->genpd[h->powered].nb.notifier_call = ane_h16_genpd_event;
		h->genpd[h->powered].added =
			!dev_pm_genpd_add_notifier(pd, &h->genpd[h->powered].nb);
		h->power[h->powered++] = pd;
	}
	if (perf) {
		ret = ane_h16_attach_perf(h);
		if (ret)
			return ret;
	}
	/* Optional: success with no input when the SoC's CLPC takes none. */
	ret = apple_clpc_ane_get(h->dev, &h->clpc);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct resource *r = platform_get_resource_byname(pdev, IORESOURCE_MEM, names[i]);

		if (i == ANE_H16_XL && !h->boot.soc->has_xl)
			continue;
		if (!r)
			return -ENODEV;
		h->window[i] = ioremap_np(r->start, resource_size(r));
		if (!h->window[i])
			return -ENOMEM;
	}
	for (i = 0; i < 3; i++) {
		struct of_phandle_args args;
		struct platform_device *dart;
		struct resource r;

		ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", i, &args);
		if (ret)
			return ret;
		if (args.args_count != 1 || args.args[0] != 0 ||
		    !of_device_is_compatible(args.np, "apple,t8110-dart") ||
		    !of_device_is_available(args.np)) {
			of_node_put(args.np);
			return -EINVAL;
		}
		dart = of_find_device_by_node(args.np);
		ret = of_address_to_resource(args.np, 0, &r);
		of_node_put(args.np);
		if (!dart)
			return -EPROBE_DEFER;
		h->dart[i] = &dart->dev;
		for (j = 0; j < i; j++)
			if (h->dart[j] == h->dart[i])
				return -EINVAL;
		if (ret)
			return ret;
		ret = pm_runtime_resume_and_get(h->dart[i]);
		if (ret < 0)
			return ret;
		h->darts_powered++;
		h->engine_on = h->darts_powered == ARRAY_SIZE(h->dart);
		h->window[ANE_H16_DART0 + i] = ioremap_np(r.start, resource_size(&r));
		if (!h->window[ANE_H16_DART0 + i])
			return -ENOMEM;
		/* Only the instance the platform names has a DAPF bank. */
		if (i != h->boot.soc->dapf_dart)
			continue;
		h->window[ANE_H16_DAPF] = ioremap_np(r.start + h->boot.soc->dapf_offset,
						     h->boot.soc->dapf_size);
		if (!h->window[ANE_H16_DAPF])
			return -ENOMEM;
	}
	if (!h->window[ANE_H16_DAPF])
		return -ENODEV;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = request_irq(irq, ane_h16_irq, IRQF_NO_AUTOEN, "ane-h16", h);
	if (ret)
		return ret;
	h->irq = irq;
	return 0;
}

static int ane_h16_preload_identity(struct ane_h16_device *h)
{
	struct ane_h16_boot *b = &h->boot;
	void *snap[2] = {}, *second = NULL;
	const char *identity = NULL;
	unsigned int i;
	int ret = -ENOMEM;

	for (i = 0; i < 2; i++) {
		u64 base, size;
		void *live;

		ret = ane_fw_preload_range(h->dev->of_node, i, &base, &size);
		if (!ret && size != b->soc->preload_size[i])
			ret = -EINVAL;
		if (ret)
			goto out;
		ret = -ENOMEM;
		h->preload_phys[i] = base;
		snap[i] = vzalloc(size);
		second = vzalloc(size);
		if (!snap[i] || !second)
			goto out;
		live = memremap(base, size, MEMREMAP_WB);
		if (!live)
			goto out;
		memcpy(snap[i], live, size);
		/* Separate the two observations used to reject an unstable preload. */
		rmb();
		memcpy(second, live, size);
		memunmap(live);
		if (memcmp(snap[i], second, size)) {
			ret = -EBUSY;
			goto out;
		}
		vfree(second);
		second = NULL;
	}
	ret = -ENOEXEC;
	for (i = 0; i < b->soc->h16_profile_count && ret == -ENOEXEC; i++) {
		const struct ane_h16_profile *p = &b->soc->h16_profiles[i];
		u8 digest[SHA256_DIGEST_SIZE];

		ret = ane_h16_preload_digest(p->preload, snap[0], b->soc->preload_size[0],
					     snap[1], b->soc->preload_size[1], digest);
		if (ret)
			continue;
		if (memcmp(digest, p->preload->digest, sizeof(digest))) {
			/* The layout matched; name the image so a new build can be pinned. */
			dev_info(h->dev, "H16 preload has the %s layout but digest %*phN\n",
				 p->identity, (int)sizeof(digest), digest);
			ret = -ENOEXEC;
			continue;
		}
		b->build = p->build;
		identity = p->identity;
	}
	if (ret) {
		dev_err(h->dev, "H16 preload matches none of %u firmware profiles, error %d\n",
			b->soc->h16_profile_count, ret);
		goto out;
	}
	/* Firmware DVA layout belongs to the build just selected. */
	/* Warm-sleep builds keep the verified DATA for a cold boot after sleep. */
	if (b->soc->sleep_warm || b->soc->sleep_cold) {
		h->pristine_data = snap[1];
		snap[1] = NULL;
		b->max_restarts = ANE_H16_STARTUP_RESTARTS;
	}
	h->shared_next = b->build->shared_dva;
	h->program_start = b->build->shared_dva + ANE_FW_SHARED_TOTAL;
	h->program_end = h->program_start + ANE_H16_PROGRAM_WINDOW;
	dev_info(h->dev, "H16 preload identity %s accepted\n", identity);
out:
	vfree(second);
	vfree(snap[0]);
	vfree(snap[1]);
	return ret;
}

/*
 * How long dynamic power gating stays off after the last EXECUTE. macOS 27.0
 * on the J616s keeps it off across a run of back-to-back requests and turns
 * it back on about 1 ms after the last one (two 0x002d per run, not per
 * request); toggling it per request costs about 50 us of each EXECUTE ack.
 */
#define ANE_H16_GATE_HOLD_NS (2 * NSEC_PER_MSEC)

/* After startup: one program session and the ABI 3 accel node. */
static int ane_h16_program_stage(struct ane_h16_device *h)
{
	int ret;

	/* Before the node exists: the CLPC may have bound after acquire. */
	if (!h->clpc && apple_clpc_ane_get(h->dev, &h->clpc))
		h->clpc = NULL;
	ret = ane_h16_session_init(&h->session, &h->boot.transport, &ane_h16_session_ops, h,
				   h->boot.build->command_timeout_ns);
	if (!ret) {
		h->session.gate_hold_ns = ANE_H16_GATE_HOLD_NS;
		ret = ane_h16_accel_register(h->dev, &h->session);
	}
	if (ret) {
		dev_err(h->dev, "H16 program stage unavailable, error %d; held until reboot\n",
			ret);
		return ret;
	}
	h->program_ready = true;
	ane_h16_irq_wait_set(h, true);
	dev_info(h->dev, "H16 stage 5 program window %llx..%llx%s\n", h->program_start,
		 h->program_end - 1, h->clpc ? ", CLPC input attached" : "");
	return 0;
}

/*
 * Advance the staged bring-up to @stage. Caller holds ane_h16_lock. Before
 * RUN any failure rolls back power and mappings and refuses further stages
 * until the module is reloaded. From RUN on, everything stays held.
 */
static int ane_h16_advance(struct ane_h16_device *h, enum ane_h16_stage stage)
{
	enum ane_h16_stage target = stage > ANE_H16_STARTUP ? ANE_H16_STARTUP : stage;
	int ret;

	if (h->refused || h->boot.failed || h->boot.quarantined)
		return -EBUSY;
	if (h->program_ready || (stage != ANE_H16_PROGRAM && stage <= h->boot.stage))
		return -EALREADY;
	if (!h->acquired) {
		h->acquired = true;
		ret = ane_h16_preload_identity(h);
		if (!ret)
			ret = ane_h16_acquire(to_platform_device(h->dev), h);
		if (ret)
			goto refuse;
	}
	if (target > h->boot.stage) {
		ret = ane_h16_run(&h->boot, target);
		if (ret && !h->boot.ran)
			goto refuse;
		if (ret) {
			dev_err(h->dev, "H16 stage %s failed after RUN, error %d; held until reboot\n",
				ane_h16_stage_name(stage), ret);
			return ret;
		}
		if (h->boot.stage == ANE_H16_STARTUP)
			print_hex_dump(KERN_INFO, "H16 RESOURCE_INFO ", DUMP_PREFIX_OFFSET, 16, 1,
				       h->boot.resource_info, sizeof(h->boot.resource_info),
				       false);
	}
	if (stage == ANE_H16_PROGRAM) {
		ret = ane_h16_program_stage(h);
		if (ret) {
			h->refused = true;
			return ret;
		}
	}
	dev_info(h->dev, "H16 stage %s complete%s\n", ane_h16_stage_name(stage),
		 h->boot.ran ? "; held until reboot" : "");
	return 0;
refuse:
	dev_err(h->dev, "H16 stage %s refused before RUN, error %d\n",
		ane_h16_stage_name(stage), ret);
	h->refused = true;
	ane_h16_release(h);
	return ret;
}

/*
 * The bootloader handed off a preload that passed metadata preflight, so the
 * engine is brought up to the program stage once per boot, off the probe
 * path. A failure before RUN leaves the device refused; after RUN it is held
 * until reboot, as for a stage written to debugfs.
 */
/* How long bring-up waits for the PMP before it starts the firmware anyway. */
#define ANE_H16_PMP_WAIT_MS 10000

/*
 * On the J616s, Linux starts the PMP itself, about a second before the ANE
 * probes, and its firmware takes about another second to come up. A H16 CPU
 * released while it does is stopped part way through its startup on about
 * one boot in six (the wrapper control word reads 0, RUN cleared): 0 of 22
 * when the release waited until the PMP ran. Wait for it, outside every lock.
 * The stop detection and restart stay as the fallback.
 */
static void ane_h16_wait_pmp(struct ane_h16_device *h)
{
	const char *compatible = h->boot.soc->pmp_report;
	struct device_node *np;
	int ret;

	if (!compatible)
		return;
	np = of_find_compatible_node(NULL, NULL, compatible);
	if (!np)
		return;
	ret = apple_pmp_report_wait_running(np, ANE_H16_PMP_WAIT_MS);
	of_node_put(np);
	if (ret && ret != -ENODEV)
		dev_warn(h->dev, "H16 ANE: PMP not running (%d); starting the firmware anyway\n",
			 ret);
}

static void ane_h16_start_work(struct work_struct *work)
{
	struct ane_h16_device *h = container_of(work, struct ane_h16_device, start);
	unsigned int flags;

	ane_h16_wait_pmp(h);

	/* Never start the firmware in the middle of a system sleep transition. */
	flags = lock_system_sleep();
	mutex_lock(&ane_h16_lock);
	if (!h->acquired)
		ane_h16_advance(h, ANE_H16_PROGRAM);
	mutex_unlock(&ane_h16_lock);
	unlock_system_sleep(flags);
}

/*
 * Diagnostic control in debugfs: read the stage state, or, before the boot
 * bring-up has run, write a stage name to <debugfs>/ane-h16/<dev>/stage to
 * run every stage up to it.
 */
static ssize_t ane_h16_stage_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct ane_h16_device *h = file->private_data;
	unsigned int flags;
	char buf[16];
	int stage, ret;

	if (!count || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = 0;
	strim(buf);
	stage = ane_h16_stage_parse(buf);
	if (stage <= ANE_H16_REFUSE)
		return -EINVAL;
	/* As the start work: a stage that pins never lands inside a sleep transition. */
	flags = lock_system_sleep();
	mutex_lock(&ane_h16_lock);
	ret = ane_h16_advance(h, stage);
	mutex_unlock(&ane_h16_lock);
	unlock_system_sleep(flags);
	return ret ? ret : count;
}

static ssize_t ane_h16_stage_read(struct file *file, char __user *ubuf, size_t count,
				  loff_t *ppos)
{
	struct ane_h16_device *h = file->private_data;
	char buf[96];
	int len;

	mutex_lock(&ane_h16_lock);
	len = scnprintf(buf, sizeof(buf), "%s ran=%d refused=%d quarantined=%d error=%d\n",
			h->program_ready ? "program" : ane_h16_stage_name(h->boot.stage),
			h->boot.ran, h->refused || h->boot.failed, h->boot.quarantined,
			h->boot.error);
	mutex_unlock(&ane_h16_lock);
	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations ane_h16_stage_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ane_h16_stage_read,
	.write = ane_h16_stage_write,
	.llseek = default_llseek,
};

/*
 * <debugfs>/ane-h16/<dev>/execute: per-phase EXECUTE times, see
 * ane_h16_session_stats_show(). Writing anything resets them.
 */
static int ane_h16_execute_show(struct seq_file *m, void *v)
{
	struct ane_h16_device *h = m->private;
	bool ready;

	mutex_lock(&ane_h16_lock);
	ready = h->program_ready;
	mutex_unlock(&ane_h16_lock);
	if (!ready) {
		seq_puts(m, "executes 0 (program stage not reached)\n");
		return 0;
	}
	/* program_ready never goes back, and the session outlives this file. */
	return ane_h16_session_stats_show(m, &h->session);
}

static int ane_h16_execute_open(struct inode *inode, struct file *file)
{
	return single_open(file, ane_h16_execute_show, inode->i_private);
}

static ssize_t ane_h16_execute_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct ane_h16_device *h = ((struct seq_file *)file->private_data)->private;
	bool ready;

	mutex_lock(&ane_h16_lock);
	ready = h->program_ready;
	mutex_unlock(&ane_h16_lock);
	if (ready)
		ane_h16_session_stats(&h->session, NULL, true);
	return count;
}

static const struct file_operations ane_h16_execute_fops = {
	.owner = THIS_MODULE,
	.open = ane_h16_execute_open,
	.read = seq_read,
	.write = ane_h16_execute_write,
	.llseek = seq_lseek,
	.release = single_release,
};

int ane_h16_backend_probe(struct platform_device *pdev, const struct ane_fw_platform *soc)
{
	struct ane_h16_device *h;
	unsigned int i, j;
	int ret;

	mutex_lock(&ane_h16_lock);
	if (ane_h16_released) {
		ret = -EBUSY;
		goto unlock;
	}
	h = kzalloc_obj(*h);
	if (!h) {
		ret = -ENOMEM;
		goto unlock;
	}
	h->dev = &pdev->dev;
	h->irq = -1;
	spin_lock_init(&h->irq_lock);
	init_waitqueue_head(&h->irq_wq);
	mutex_init(&h->window_lock);
	INIT_LIST_HEAD(&h->mappings);
	/* Offsets from program_start, which the preload's build decides. */
	drm_mm_init(&h->program_window, 0, ANE_H16_PROGRAM_WINDOW);
	h->boot.soc = soc;
	/* The build, and with it the DVA layout, is chosen from the preload. */
	h->boot.build = NULL;
	h->boot.ctx = h;
	h->boot.ops = &ane_h16_ops;
	h->boot.transport_ops = &ane_h16_transport_ops;
	ret = -EINVAL;
	if (!soc->dapf_count || soc->dapf_count > ANE_FW_DAPF_MAX)
		goto free;
	ret = of_property_read_u64_array(h->dev->of_node, "apple,dapf-ranges",
					 &h->boot.ranges[0][0], soc->dapf_count * 2);
	if (ret)
		goto free;
	for (i = 0; i < soc->dapf_count; i++) {
		u64 start = h->boot.ranges[i][0], end = h->boot.ranges[i][1];

		ret = -EINVAL;
		if (!start || end < start || round_down(start, SZ_16K) != round_down(end, SZ_16K))
			goto free;
		for (j = 0; j < i; j++)
			if (round_down(start, SZ_16K) == round_down(h->boot.ranges[j][0], SZ_16K))
				goto free;
	}
	if (!ane_h16_debugfs_users++)
		ane_h16_debugfs = debugfs_create_dir("ane-h16", NULL);
	h->debugfs = debugfs_create_dir(dev_name(h->dev), ane_h16_debugfs);
	debugfs_create_file("stage", 0600, h->debugfs, h, &ane_h16_stage_fops);
	debugfs_create_file("execute", 0600, h->debugfs, h, &ane_h16_execute_fops);
	debugfs_create_u64("irq_wakeups", 0400, h->debugfs, &h->irq_wakeups);
	debugfs_create_u64("irq_timeouts", 0400, h->debugfs, &h->irq_timeouts);
	debugfs_create_u64("recoveries", 0400, h->debugfs, &h->recoveries);
	debugfs_create_u64("recover_failures", 0400, h->debugfs, &h->recover_failures);
	platform_set_drvdata(pdev, h);
	INIT_WORK(&h->recover, ane_h16_recover_work);
	INIT_WORK(&h->start, ane_h16_start_work);
	mutex_unlock(&ane_h16_lock);

	/*
	 * Outside ane_h16_lock (see pm_notifier), and before the start work
	 * that can pin the device, so a pinned device always has it.
	 */
	h->pm_notifier.notifier_call = ane_h16_pm_event;
	ret = register_pm_notifier(&h->pm_notifier);
	if (ret) {
		dev_err(h->dev, "H16 PM notifier refused, error %d\n", ret);
		debugfs_remove_recursive(h->debugfs);
		mutex_lock(&ane_h16_lock);
		if (!--ane_h16_debugfs_users) {
			debugfs_remove_recursive(ane_h16_debugfs);
			ane_h16_debugfs = NULL;
		}
		mutex_unlock(&ane_h16_lock);
		platform_set_drvdata(pdev, NULL);
		kfree(h);
		return ret;
	}
	schedule_work(&h->start);
	return 0;
free:
	dev_err(h->dev, "H16 DT description refused, error %d\n", ret);
	drm_mm_takedown(&h->program_window);
	kfree(h);
unlock:
	mutex_unlock(&ane_h16_lock);
	return ret;
}

void ane_h16_backend_remove(struct platform_device *pdev)
{
	struct ane_h16_device *h = platform_get_drvdata(pdev);
	bool keep;

	if (!h)
		return;
	/* Waits for the boot bring-up and any stage write in progress. */
	cancel_work_sync(&h->start);
	cancel_work_sync(&h->recover);
	if (h->program_ready) {
		cancel_delayed_work_sync(&h->session.gate_work);
		/* Return released exports; mapped ones stay with their mappings. */
		flush_work(&h->session.export_work);
	}
	debugfs_remove_recursive(h->debugfs);
	/*
	 * Nothing can pin the device any more: the start work, the recovery
	 * and the stage file are gone. An unpinned device drops its PM
	 * notifier here, outside ane_h16_lock; a pinned one keeps it.
	 */
	scoped_guard(mutex, &ane_h16_lock)
		keep = h->boot.ran || h->pinned;
	if (!keep)
		unregister_pm_notifier(&h->pm_notifier);
	mutex_lock(&ane_h16_lock);
	if (!--ane_h16_debugfs_users) {
		debugfs_remove_recursive(ane_h16_debugfs);
		ane_h16_debugfs = NULL;
	}
	if (keep) {
		/* The module reference taken before RUN makes this unreachable by rmmod. */
		dev_warn(h->dev, "H16 remove held; reboot reclaims firmware DMA\n");
	} else {
		ane_h16_release(h);
		drm_mm_takedown(&h->program_window);
		kfree(h);
	}
	mutex_unlock(&ane_h16_lock);
}
