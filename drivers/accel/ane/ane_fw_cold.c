// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Shared firmware backend. H17 is the first measured cold power provider. */
#include <crypto/sha2.h>
#include <drm/drm_mm.h>
#include <linux/crash_dump.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kexec.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/soc/apple/clpc.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>

#include "ane_fw_cold.h"
#include "ane_fw_sleep.h"
#include "ane_h16_accel.h"

#define ANE_FW_POOL_DVA 0x10000384000ULL
#define ANE_FW_IPC_DVA 0x100003d0000ULL
#define ANE_FW_HEAP_DVA 0x100003e4000ULL
#define ANE_FW_ALLOC_DVA 0x10001300000ULL
/* SHAREDMALLOC and program mappings: DVAs from ANE_FW_ALLOC_DVA on. */
#define ANE_FW_WINDOW SZ_512M
/* ane_fw_alloc_mem(): a DVA range from the window. */
#define ANE_FW_WINDOW_ANY U64_MAX
#define ANE_FW_TIMEOUT_NS (3ULL * NSEC_PER_SEC)

enum ane_fw_stage {
	ANE_STAGE_UNVERIFIED,
	ANE_STAGE_PROBED,
	ANE_STAGE_MAPPED,
	ANE_STAGE_BOOTED,
	ANE_STAGE_STARTUP,
	ANE_STAGE_PROGRAM,
	ANE_STAGE_QUARANTINED,
};

struct ane_fw_mapping {
	struct list_head list;
	struct ane_fw_surface surface;
	dma_addr_t physical;
	/* The DVA range, for a mapping in the window. */
	struct drm_mm_node node;
};

struct ane_fw_backend {
	struct platform_device *pdev;
	const struct ane_fw_platform *soc;
	void __iomem *engine, *pmgr, *gate, *dart[ANE_FW_DARTS];
	/* Mapped register windows, for the MMIO trace hook. */
	struct {
		void __iomem *base;
		phys_addr_t pa;
		resource_size_t size;
	} window[3 + ANE_FW_DARTS];
	struct device *power, *memory;
	struct iommu_domain *domain;
	struct ane_fw_transport transport;
	/* Programs and their ABI 3 node, after the program stage. */
	struct ane_h16_session session;
	struct apple_clpc_ane *clpc;
	struct ane_fw_surface pool, ipc, heap;
	/*
	 * For the cold boot after system sleep: the private TEXT/DATA copies,
	 * the reserved preload they came from (no-map and untouched, since the
	 * firmware runs from the copies), the digest DATA had, the heap tail.
	 */
	struct ane_fw_surface preload[2], heap_tail;
	u64 preload_phys[2];
	u8 text_digest[SHA256_DIGEST_SIZE], data_digest[SHA256_DIGEST_SIZE];
	/* Powered down for system sleep; a (cold or warm) boot is due. */
	bool slept;
	/*
	 * Parked: powered off with no firmware, as macOS keeps it without a
	 * client, until a request brings it up (ane_fw_session_wake()).
	 */
	bool parked;
	/* Between PM_SUSPEND_PREPARE and PM_POST_SUSPEND. */
	bool in_sleep;
	/* The last cold boot after sleep: firmware ready, first replay, count. */
	u64 resume_ready_ns, resume_first_ns;
	unsigned int resume_replayed;
	/*
	 * EXCLAVE_MODE_STOP completed in this firmware boot: the warm boots
	 * keep the firmware out of Exclave mode. Cleared by every cold boot.
	 */
	bool exclave_mode_stopped;
	/* The bound device holds ANE_SYS for a hand-off until the bring-up does. */
	bool bind_held;
	/* SET_SNE_PMU_BASE2 completed in this firmware boot. */
	bool pmu_base_set;
	/* The engine's own ANE_SYS reference (b->power) is held. */
	bool power_ref;
	struct list_head mappings;
	/* Serialize stage changes, command publication and retained lifetimes. */
	struct mutex lock;
	struct notifier_block pm_notifier;
	/*
	 * genpd's own ANE_SYS power transitions, which during system sleep
	 * happen only in the noirq phase: counted, with the PMGR state read
	 * back at each, so a wake can say whether the domain really went off.
	 */
	struct notifier_block genpd_nb;
	bool genpd_nb_added;
	unsigned int sys_offs, sys_ons;
	u32 sys_off_pmgr, sys_on_pmgr;
	u64 sys_off_at, sys_off_ns;
	struct ane_fw_cold_state cold;
	u64 ranges[ANE_FW_DAPF_MAX * 2];
	/*
	 * The DVA window and the mappings list. Program mappings come and go
	 * under the session lock while the engine is up; the others are made
	 * with the session closed.
	 */
	struct drm_mm dva_window;
	struct mutex window_lock;
	enum ane_fw_stage stage;
	bool retained;
	/* Brings the engine up to the program stage after a loader hand-off. */
	struct work_struct start;
	/* ANE_SYS and the child power states are on: the engine is readable. */
	bool powered;
};

/* One ANE per SoC. The observer exists before platform driver registration:
 * the platform bus would otherwise resume genpd and attach DMA before probe.
 */
static struct ane_fw_backend *ane_backend;
/* The H17 platform device, bound by ane_fw_staged_bind(). */
static struct device *ane_fw_bound_dev;

/* The bring-up holds ANE_SYS itself now: drop the bind's hand-off reference. */
static void ane_fw_bind_release(struct ane_fw_backend *b)
{
	if (!b->bind_held)
		return;
	b->bind_held = false;
	pm_runtime_put(&b->pdev->dev);
}

/*
 * Every register access of this backend goes through these helpers, so a
 * debug build can log the physical address of each access before it is
 * made. The hook is empty here.
 */
static inline void ane_fw_trace(struct ane_fw_backend *b, char op, const void __iomem *a,
				u64 value)
{
}

static u32 ane_rd32(struct ane_fw_backend *b, const void __iomem *a)
{
	ane_fw_trace(b, 'r', a, 0);
	return readl(a);
}

static u64 ane_rd64(struct ane_fw_backend *b, const void __iomem *a)
{
	ane_fw_trace(b, 'R', a, 0);
	return readq(a);
}

static void ane_wr32(struct ane_fw_backend *b, u32 value, void __iomem *a)
{
	ane_fw_trace(b, 'w', a, value);
	writel(value, a);
}

static void ane_wr64(struct ane_fw_backend *b, u64 value, void __iomem *a)
{
	ane_fw_trace(b, 'W', a, value);
	writeq(value, a);
}

static int ane_poll32(struct ane_fw_backend *b, void __iomem *a, u32 mask, u32 want,
		      unsigned long sleep_us, u64 timeout_us)
{
	u32 value;

	ane_fw_trace(b, 'p', a, want);
	return readl_poll_timeout(a, value, (value & mask) == want, sleep_us, timeout_us);
}
static struct kexec_blocker ane_fw_kexec = {
	.users = ATOMIC_INIT(0),
	.reason = "ANE firmware retains private DMA mappings until hardware reset",
};

static int ane_fw_quarantine(struct ane_fw_backend *b, int error)
{
	unsigned int i;

	b->stage = ANE_STAGE_QUARANTINED;
	dev_err(&b->pdev->dev, "ANE quarantined (%d); mappings and power retained, reboot required\n",
		error);
	/* The engine and DARTs are only readable once powered. */
	if (!b->powered)
		return error;
	for (i = 0; i < 8; i++)
		dev_info(&b->pdev->dev, "scratch%u=%#x\n", i,
			 ane_rd32(b, b->engine + b->soc->scratch + i * 4));
	for (i = 0; i < ANE_FW_DARTS; i++)
		dev_info(&b->pdev->dev, "DART%u error=%#x address=%#x:%#x\n", i,
			 ane_rd32(b, b->dart[i] + 0x100), ane_rd32(b, b->dart[i] + 0x174),
			 ane_rd32(b, b->dart[i] + 0x170));
	return error;
}

static bool ane_fw_raw_boot(void)
{
	struct device_node *chosen __free(device_node) = of_find_node_by_path("/chosen");
	const char *evidence;

	return chosen && !of_property_read_string(chosen, "apple,ane-cold-boot", &evidence) &&
	       !strcmp(evidence, ANE_FW_COLD_BOOT_EVIDENCE);
}

/*
 * Step 1: PMGR words only. These are the only reads made with ANE_SYS in an
 * unknown power state; the engine block and its DARTs sit behind ANE_SYS and
 * an access to them while it is off stalls the bus.
 */
static void ane_fw_snapshot_pmgr(struct ane_fw_backend *b)
{
	struct ane_fw_cold_state *s = &b->cold;

	s->sys = ane_rd32(b, b->pmgr + b->soc->pmgr_sys);
	s->cpu = ane_rd32(b, b->pmgr + b->soc->pmgr_cpu);
	s->subblock = ane_rd32(b, b->pmgr + b->soc->pmgr_subblock);
	dev_dbg(&b->pdev->dev, "U2 ANE_SYS=%#x CPU_PS=%#x subblock=%#x\n",
		s->sys, s->cpu, s->subblock);
}

/* Step 2: PWGATE, with ANE_SYS on and before the child power step. */
static void ane_fw_snapshot_gate(struct ane_fw_backend *b)
{
	struct ane_fw_cold_state *s = &b->cold;

	s->gate_request = ane_rd32(b, b->gate + 0xc);
	s->gate_ack = ane_rd32(b, b->gate + 0x30);
	dev_dbg(&b->pdev->dev, "U2 PWGATE=%#x/%#x\n", s->gate_request, s->gate_ack);
}

/* Step 3: the engine and its DARTs, after the child power step. */
static void ane_fw_snapshot_engine(struct ane_fw_backend *b)
{
	struct ane_fw_cold_state *s = &b->cold;
	struct device *dev = &b->pdev->dev;
	unsigned int i, j;

	s->ce_state = ane_rd64(b, b->engine + 0x18a4010);
	s->ce_error = ane_rd64(b, b->engine + 0x18a4018);
	s->rvbar = ane_rd64(b, b->engine + 0x1050000);
	s->control = ane_rd32(b, b->engine + b->soc->wrapper + 0x44);
	s->status = ane_rd32(b, b->engine + b->soc->wrapper + 0x48);
	s->pending = ane_rd32(b, b->engine + b->soc->irq_status);
	for (i = 0; i < 8; i++)
		s->scratch[i] = ane_rd32(b, b->engine + b->soc->scratch + i * 4);
	dev_dbg(dev, "U2 control=%#x status=%#x RVBAR=%#llx CE=%#llx/%#llx pending=%#x\n",
		s->control, s->status, s->rvbar, s->ce_state, s->ce_error, s->pending);
	for (i = 0; i < ANE_FW_DARTS; i++) {
		s->dart[i].version = ane_rd32(b, b->dart[i] + 8);
		s->dart[i].protect = ane_rd32(b, b->dart[i] + 0x200);
		s->dart[i].unprotect = ane_rd32(b, b->dart[i] + 0x204);
		s->dart[i].lock = ane_rd32(b, b->dart[i] + 0x208);
		s->dart[i].error = ane_rd32(b, b->dart[i] + 0x100);
		for (j = 0; j < 16; j++)
			s->dart[i].enabled[j] = ane_rd32(b, b->dart[i] + 0xc00 + 4 * j);
		dev_dbg(dev, "U2 DART%u version=%#x protect=%#x unprotect=%#x lock=%#x error=%#x\n",
			i, s->dart[i].version, s->dart[i].protect,
			s->dart[i].unprotect, s->dart[i].lock, s->dart[i].error);
		for (j = 0; j < 16; j++)
			if (s->dart[i].enabled[j])
				dev_dbg(dev, "U2 DART%u stream enable word %u=%#x\n", i, j,
					s->dart[i].enabled[j]);
	}
	for (i = 0; i < b->soc->dapf_count; i++) {
		void __iomem *r = b->engine + 0x1810000 + i * 0x40;

		dev_dbg(dev, "U2 DAPF%u flags=%#x start=%#llx end=%#llx enable=%#x\n",
			i, ane_rd32(b, r), ane_rd64(b, r + 8), ane_rd64(b, r + 0x10),
			ane_rd32(b, r + 0x20));
	}
}

static int ane_fw_power_up(struct ane_fw_backend *b, bool raw_boot, bool cold_engine);
static int ane_fw_dapf(struct ane_fw_backend *b);
static int ane_fw_scratch_wait(struct ane_fw_backend *b, unsigned int cell, u32 wanted);

/*
 * System sleep the macOS way (SPEC-ANE-SLEEP.md, t-4515): the firmware is
 * suspended, not torn down. At PM_SUSPEND_PREPARE the session is closed to
 * new requests, the firmware gets SUSPEND (0x24), the engine is power gated
 * (sub-block, PWGATE, CPU, then ANE_SYS through genpd); every mapping, the
 * live TEXT/DATA copies, pool, IPC, heap, shared allocations and programs
 * stay as they are. After resume the engine is powered again, the DART and
 * DAPF state the power-off lost is restored, CE is brought up, and the CPU is
 * released with SCRATCH7 = 1, the warm-boot argument, so the same firmware
 * instance resumes. Programs, processes and transport cursors survive; no
 * LOAD or CREATE is replayed. A failure before the power-off refuses the
 * sleep and keeps everything; a failure after it quarantines.
 */
#define ANE_FW_CMD_SUSPEND 0x24
#define ANE_FW_WARM_BOOT 1

/*
 * Two system sleep paths, chosen by the platform's sleep_warm flag:
 * - cold (the H17 default): the engine is powered off with no firmware
 *   cooperation; after resume the firmware is booted again from the
 *   kernel-held preload and every open program is replayed (LOAD, CREATE)
 *   behind its existing handle, so applications see no change;
 * - warm: EXCLAVE_MODE_STOP (0x31, once per firmware boot), SUSPEND (0x24),
 *   WFI, power-off, then a warm boot with SCRATCH7 = 1 that keeps the
 *   firmware and its programs (SPEC-ANE-SLEEP-H17.md). On 25G83 0x24 is
 *   still not taken after 0x31 (C7). A failed warm boot falls back to the
 *   cold one.
 */
#define ane_fw_sleep_warm(b) ((b)->soc->sleep_warm)
#define ANE_FW_SUSPEND_REPORT_MAX 0x100

static int ane_fw_command(struct ane_fw_backend *b, unsigned int channel,
			  void *command, size_t size, u64 deadline);
static u64 ane_fw_load64(void *ctx, const void *p);

/* Scratch, wrapper status and the IO slot, around the sleep handshake. */
static void ane_fw_sleep_trace(struct ane_fw_backend *b, const char *when, u8 *slot)
{
	void __iomem *scratch = b->engine + b->soc->scratch;
	u8 *t2h, *shm;

	dev_info(&b->pdev->dev,
		 "H17 ANE: %s: scratch %#x %#x %#x %#x %#x %#x %#x %#x wrapper %#x\n", when,
		 ane_rd32(b, scratch), ane_rd32(b, scratch + 4), ane_rd32(b, scratch + 8),
		 ane_rd32(b, scratch + 12), ane_rd32(b, scratch + 16), ane_rd32(b, scratch + 20),
		 ane_rd32(b, scratch + 24), ane_rd32(b, scratch + 28),
		 ane_rd32(b, b->engine + b->soc->wrapper + 0x48));
	if (slot)
		dev_info(&b->pdev->dev, "H17 ANE: %s: IO slot %#llx %#llx %#llx, IO head %u\n",
			 when, ane_fw_load64(b, slot), ane_fw_load64(b, slot + 8),
			 ane_fw_load64(b, slot + 16), b->transport.head[ANE_FW_IO]);
	t2h = ane_fw_transport_slot(&b->transport, ANE_FW_IO_T2H);
	shm = ane_fw_transport_slot(&b->transport, ANE_FW_SHAREDMALLOC);
	dev_info(&b->pdev->dev,
		 "H17 ANE: %s: T2H head %u %#llx %#llx %#llx, SHAREDMALLOC head %u %#llx %#llx %#llx, irq %#x\n",
		 when, b->transport.head[ANE_FW_IO_T2H], ane_fw_load64(b, t2h),
		 ane_fw_load64(b, t2h + 8), ane_fw_load64(b, t2h + 16),
		 b->transport.head[ANE_FW_SHAREDMALLOC], ane_fw_load64(b, shm),
		 ane_fw_load64(b, shm + 8), ane_fw_load64(b, shm + 16),
		 ane_rd32(b, b->engine + b->soc->irq_status));
}

/*
 * Wait for the SUSPEND reply. In C2 (boot 0d5a5af9) the firmware took the
 * command: its log gained a task hand-off record at the SUSPEND time, but the
 * IO slot never completed. ane_fw_transport_wait() serves SHAREDMALLOC only;
 * here every T2H report the firmware posts meanwhile is also logged and
 * acknowledged, the way the session pump does during EXECUTE.
 */
static int ane_fw_suspend_wait(struct ane_fw_backend *b)
{
	struct ane_fw_transport *t = &b->transport;
	unsigned int reports = 0, shared = 0;
	u64 start = ktime_get_ns(), now;
	u32 pending, seen = 0;
	size_t size;
	u8 *report;
	int ret;

	report = kmalloc(ANE_FW_SUSPEND_REPORT_MAX, GFP_KERNEL);
	if (!report)
		return -ENOMEM;
	for (;;) {
		ret = ane_fw_transport_poll(t, ANE_FW_IO);
		if (ret != -EAGAIN)
			break;
		ret = ane_fw_transport_consume(t, report, ANE_FW_SUSPEND_REPORT_MAX, &size);
		if (!ret) {
			reports++;
			dev_info(&b->pdev->dev, "H17 ANE: report %u during SUSPEND (%zu bytes): %*ph\n",
				 reports, size, (int)min_t(size_t, size, 64), report);
			continue;
		}
		if (ret != -EAGAIN)
			break;
		ret = ane_fw_transport_irq(t, &pending);
		if (ret)
			break;
		seen |= pending;
		ret = ane_fw_transport_shared(t);
		if (!ret)
			shared++;
		else if (ret != -EAGAIN)
			break;
		now = ktime_get_ns();
		if (now >= t->pending[ANE_FW_IO].deadline_ns) {
			ret = ane_fw_transport_fail(t, -ETIMEDOUT);
			break;
		}
		t->polls++;
		t->ops->relax(t->ctx, now - start);
	}
	dev_info(&b->pdev->dev,
		 "H17 ANE: SUSPEND wait %d after %llu us: %u reports, %u allocations, irq seen %#x\n",
		 ret, div_u64(ktime_get_ns() - start, NSEC_PER_USEC), reports, shared, seen);
	kfree(report);
	return ret;
}

/*
 * Leave the firmware's Exclave mode before SUSPEND, once per firmware boot:
 * the 24-byte zero-body EXCLAVE_MODE_STOP on the control ring, with its
 * normal completion. Linux has no Exclave state, so the warm wake never
 * starts the mode again (no 0x30) and later cycles skip this step.
 */
static int ane_fw_exclave_mode_stop(struct ane_fw_backend *b)
{
	struct ane_fw_transport *t = &b->transport;
	struct device *dev = &b->pdev->dev;
	u32 ch = t->control_ch;
	u64 before[3] = {};
	int published, ret;
	u8 *slot = NULL;

	if (!b->soc->exclave_mode_stop || b->exclave_mode_stopped)
		return 0;
	if (ch < ANE_FW_CHANNELS) {
		slot = ane_fw_transport_slot(t, ch);
		for (unsigned int i = 0; i < 3; i++)
			before[i] = ane_fw_load64(b, slot + 8 * i);
	}
	published = ane_fw_transport_publish_mode_stop(t, ktime_get_ns() + ANE_FW_TIMEOUT_NS);
	ret = published ?: ane_fw_transport_wait(t, ch);
	if (ret) {
		const struct ane_fw_transaction *tx = &t->pending[ch < ANE_FW_CHANNELS ? ch : 0];

		dev_err(dev, "H17 ANE: EXCLAVE_MODE_STOP (0x31) failed %d in %s on %s (table position %d); no SUSPEND, no power-off\n",
			ret, published ? "publish" : "wait", ane_fw_channel_name(ch),
			ch < ANE_FW_CHANNELS ? (int)t->channels[ch].index : -1);
		dev_err(dev, "H17 ANE: 0x31 transport error %d raised at ane_fw_transport.h:%u\n",
			t->error, t->error_line);
		if (slot) {
			dev_err(dev, "H17 ANE: 0x31 slot head %u before %#llx %#llx %#llx after %#llx %#llx %#llx\n",
				t->head[ch], before[0], before[1], before[2],
				ane_fw_load64(b, slot), ane_fw_load64(b, slot + 8),
				ane_fw_load64(b, slot + 16));
		}
		/* Only a published command has its copy in the pool. */
		if (slot && !published)
			dev_err(dev, "H17 ANE: 0x31 command at %#llx size %zu: %*ph\n", tx->dva,
				tx->size, ANE_FW_MODE_STOP_SIZE,
				(u8 *)t->pool.cpu + (tx->dva - t->pool.dva));
		return ret;
	}
	b->exclave_mode_stopped = true;
	dev_info(&b->pdev->dev, "H17 ANE: EXCLAVE_MODE_STOP (0x31) completed on channel %u\n",
		 t->channels[t->control_ch].index);
	return 0;
}

static int ane_fw_suspend_firmware(struct ane_fw_backend *b)
{
	u8 command[8] = {};
	u8 *slot;
	u32 status;
	int ret;

	if (b->transport.error)
		return b->transport.error;
	for (unsigned int ch = 0; ch < ANE_FW_CHANNELS; ch++)
		if (b->transport.pending[ch].active)
			return -EBUSY;
	/* Answer any SHAREDMALLOC request already posted (a late FREE). */
	ret = ane_fw_transport_settle(&b->transport, 2 * NSEC_PER_MSEC, 20 * NSEC_PER_MSEC);
	if (ret < 0)
		return ret;
	ret = ane_fw_exclave_mode_stop(b);
	if (ret) {
		ane_h16_session_dump_fwlog(&b->session);
		return ret;
	}
	put_unaligned_le16(ANE_FW_CMD_SUSPEND, command + 4);
	ane_fw_sleep_trace(b, "before SUSPEND", NULL);
	slot = ane_fw_transport_slot(&b->transport, ANE_FW_IO);
	dev_info(&b->pdev->dev, "H17 ANE: SUSPEND %*ph on IO head %u slot %#llx %#llx %#llx\n",
		 (int)sizeof(command), command, b->transport.head[ANE_FW_IO],
		 ane_fw_load64(b, slot), ane_fw_load64(b, slot + 8), ane_fw_load64(b, slot + 16));
	ret = ane_fw_transport_publish(&b->transport, ANE_FW_IO, command, sizeof(command),
				       ktime_get_ns() + ANE_FW_TIMEOUT_NS);
	if (!ret)
		ret = ane_fw_suspend_wait(b);
	if (!ret)
		memcpy(command, b->pool.cpu + b->transport.pending[ANE_FW_IO].dva - b->pool.dva,
		       sizeof(command));
	ane_fw_sleep_trace(b, ret ? "SUSPEND failed" : "SUSPEND done", slot);
	if (ret && ane_fw_load64(b, slot) == b->transport.pending[ANE_FW_IO].dva)
		dev_err(&b->pdev->dev, "H17 ANE: 0x24 not consumed; Exclave mode suspected\n");
	if (ret) {
		/* What the firmware made of it, if anything. */
		ane_h16_session_dump_fwlog(&b->session);
		return ret;
	}
	dev_info(&b->pdev->dev, "H17 ANE: SUSPEND reply %*ph\n", (int)sizeof(command), command);
	if (get_unaligned_le16(command + 6))
		return -EREMOTEIO;
	dev_info(&b->pdev->dev, "H17 ANE: SUSPEND acknowledged, wrapper status %#x\n",
		 ane_rd32(b, b->engine + b->soc->wrapper + 0x48));
	/*
	 * Wait for the ANE CPU to idle in WFI (ASC wrapper status). The held
	 * H16 shutdown reads 0x6e against the 0x6a reset value; this takes
	 * bit 2 as the idle indication. TODO(t-4515): confirm the bit.
	 */
	ret = ane_poll32(b, b->engine + b->soc->wrapper + 0x48, BIT(2), BIT(2), 10, 1000000);
	status = ane_rd32(b, b->engine + b->soc->wrapper + 0x48);
	dev_info(&b->pdev->dev, "H17 ANE: firmware suspended, wrapper status %#x\n", status);
	return ret;
}

static int ane_fw_boot(struct ane_fw_backend *b);
static int ane_fw_startup(struct ane_fw_backend *b);
static int ane_fw_first_bringup(struct ane_fw_backend *b);
static ssize_t ane_fw_stage_action(struct device *dev, const char *buf, size_t count);
static int ane_fw_rvbar_ready(struct ane_fw_backend *b);

/* A power word goes off whole, and off means target, actual and BUSY clear. */
#define ANE_FW_PS_OFF 0x300
#define ANE_FW_PS_OFF_MASK 0x8ff
#define ANE_FW_PS_BUSY BIT(11)
#define ANE_FW_PS_UNLATCH BIT(12)

/* A word left BUSY stalls a wake that keeps DRAM up: clear it, or send this sleep deep. */
static int ane_fw_ps_off(struct ane_fw_backend *b, u32 offset)
{
	void __iomem *reg = b->pmgr + offset;
	int ret;

	ane_wr32(b, ANE_FW_PS_OFF, reg);
	ret = ane_poll32(b, reg, ANE_FW_PS_OFF_MASK, 0, 1, 100000);
	if (!ret)
		return 0;
	dev_warn(&b->pdev->dev, "H17 ANE: power word %#x stuck at %#x\n", offset, ane_rd32(b, reg));
	if (!(ane_rd32(b, reg) & 0xff)) {
		ane_wr32(b, ANE_FW_PS_OFF | ANE_FW_PS_UNLATCH, reg);
		ane_poll32(b, reg, ANE_FW_PS_BUSY, 0, 1, 1000);
		ane_wr32(b, ANE_FW_PS_OFF, reg);
		if (!ane_poll32(b, reg, ANE_FW_PS_OFF_MASK, 0, 1, 1000))
			return 0;
	}
	if (!apple_sleep_request_deep_once())
		return ret;
	dev_warn(&b->pdev->dev, "H17 ANE: power word %#x still stuck, taking the deep path\n", offset);
	return 0;
}

/* Reverse of the proven power-up: sub-block, PWGATE, CPU, then ANE_SYS. */
static int ane_fw_power_down_staged(struct ane_fw_backend *b)
{
	void __iomem *cpu = b->pmgr + b->soc->pmgr_cpu, *sub = b->pmgr + b->soc->pmgr_subblock;
	u32 sys;
	int ret;

	ane_wr32(b, ane_rd32(b, sub) & ~0xf, sub);
	ret = ane_poll32(b, sub, 0xf0, 0, 1, 100000);
	if (ret)
		return ret;
	ane_wr32(b, 3, b->gate + 0xc);
	ret = ane_poll32(b, b->gate + 0xc, 3, 3, 1, 100000);
	if (ret)
		return ret;
	ane_wr32(b, ane_rd32(b, cpu) & ~0xf, cpu);
	ret = ane_poll32(b, cpu, 0xf0, 0, 1, 100000);
	if (ret)
		return ret;
	/* No engine or DART access from here until ANE_SYS is back on. */
	b->powered = false;
	b->power_ref = false;
	pm_runtime_put_sync(b->power);
	/*
	 * PMGR reads are safe in any power state. ANE_SYS's actual state
	 * (bits 7:4) reads 0 once genpd has really turned the domain off.
	 */
	sys = ane_rd32(b, b->pmgr + b->soc->pmgr_sys);
	/*
	 * Here only requested: while other devices of the domain (the ANE
	 * DARTs) are active, genpd turns it off in the noirq phase of the
	 * system suspend; the genpd notifier records when it really does.
	 */
	dev_info(&b->pdev->dev,
		 "H17 ANE: powered down for system sleep; ANE_SYS requested off, %s at the put (%#x)\n",
		 sys & 0xf0 ? "still on" : "off", sys);
	return 0;
}

/* The order macOS uses for this engine: sub-block, PWGATE, its ack, CPU, then ANE_SYS. */
static int ane_fw_power_down(struct ane_fw_backend *b)
{
	int ret;

	if (!b->soc->macos_sleep)
		return ane_fw_power_down_staged(b);
	ret = ane_fw_ps_off(b, b->soc->pmgr_subblock);
	if (ret)
		return ret;
	ane_wr32(b, 3, b->gate + 0xc);
	ret = ane_poll32(b, b->gate + 0xc, 3, 3, 1, 100000);
	if (ret)
		return ret;
	ane_wr32(b, 0, b->gate + 0x30);
	ret = ane_poll32(b, b->gate + 0x30, 1, 0, 1, 100000);
	if (ret)
		return ret;
	ret = ane_fw_ps_off(b, b->soc->pmgr_cpu);
	if (ret)
		return ret;
	/* No engine or DART access from here until ANE_SYS is back on. */
	b->powered = false;
	b->power_ref = false;
	/*
	 * Our reference was the last one on ANE_SYS: genpd turns it off once the
	 * ANE DARTs are idle, or at the latest in the noirq phase of the system
	 * suspend. ane_fw_genpd_event() records that, and the resume reports a
	 * domain that stayed on.
	 */
	pm_runtime_put_sync(b->power);
	dev_info(&b->pdev->dev, "H17 ANE: powered down for system sleep\n");
	return 0;
}

/*
 * After resume: power up from the cold child state, give SID0 its
 * translation back and the DAPF records, bring CE up, then release the CPU
 * into the retained firmware with the warm-boot argument.
 */
static int ane_fw_warm_boot(struct ane_fw_backend *b)
{
	struct device *dev = &b->pdev->dev;
	void __iomem *scratch = b->engine + b->soc->scratch;
	unsigned int i;
	int ret;

	ret = ane_fw_power_up(b, true, false);
	if (ret)
		return ret;
	/*
	 * The DART lost SID0's translation registers with ANE_SYS; attaching
	 * the retained domain again rewrites them from its page tables and
	 * synchronizes the TLB. TODO(hardware): check the cold ANE DART
	 * profile accepts the detach.
	 */
	iommu_detach_device(b->domain, dev);
	ret = iommu_attach_device(b->domain, dev);
	if (ret)
		return ret;
	ret = ane_fw_dapf(b);
	if (ret)
		return ret;
	/* RVBAR must name TEXT: retained, or written again after a real power-off. */
	ret = ane_fw_rvbar_ready(b);
	if (ret)
		return ret;
	for (i = 0; i < 8; i++)
		ane_wr32(b, 0, scratch + i * 4);
	ane_wr32(b, 1, scratch + 6 * 4);
	ane_wr32(b, ANE_FW_WARM_BOOT, scratch + 7 * 4);
	ane_wr32(b, 0, b->engine + b->soc->wrapper + 0x44);
	dma_wmb();
	ane_wr32(b, 0x10, b->engine + b->soc->wrapper + 0x44);
	ret = ane_fw_scratch_wait(b, 7, 0x08042006);
	dev_info(dev, "H17 ANE: warm boot RVBAR %#llx SCRATCH7 %#x wrapper status %#x (%d)\n",
		 b->cold.rvbar, ane_rd32(b, scratch + 7 * 4),
		 ane_rd32(b, b->engine + b->soc->wrapper + 0x48), ret);
	if (ret)
		return ret;
	/* The retained channel table must be what the firmware still uses. */
	ret = ane_fw_transport_check(&b->transport);
	if (ret)
		return ret;
	dev_info(dev, "H17 ANE: firmware resumed (warm boot); programs kept\n");
	return 0;
}

/*
 * RVBAR must name TEXT before the CPU is released. It keeps the latched
 * entry while ANE_SYS stays on; if a real power-off cleared it, the entry
 * is written again, which the RVBAR lifecycle allows only while bit 0
 * (latched) is clear.
 */
static int ane_fw_rvbar_ready(struct ane_fw_backend *b)
{
	u64 text = b->soc->preload_dva[0];
	u64 rvbar = ane_rd64(b, b->engine + 0x1050000);

	switch (ane_fw_rvbar_check(rvbar, text)) {
	case ANE_FW_RVBAR_LATCHED:
		return 0;
	case ANE_FW_RVBAR_WRITABLE:
		ane_wr64(b, text, b->engine + 0x1050000);
		dev_info(&b->pdev->dev, "H17 ANE: RVBAR was %#llx; wrote %#llx, reads %#llx\n",
			 rvbar, text, ane_rd64(b, b->engine + 0x1050000));
		rvbar = ane_rd64(b, b->engine + 0x1050000);
		return rvbar == text || rvbar == (text | 1) ? 0 : -EIO;
	default:
		dev_err(&b->pdev->dev, "H17 ANE: RVBAR %#llx latched at another entry\n", rvbar);
		return -EIO;
	}
}

/* Copy one preload segment back from its reservation and check its digest. */
static int ane_fw_reload_segment(struct ane_fw_backend *b, unsigned int i)
{
	struct ane_fw_surface *s = &b->preload[i];
	u8 digest[SHA256_DIGEST_SIZE];
	void *source;

	if (!s->cpu || !b->preload_phys[i])
		return -ENODATA;
	source = memremap(b->preload_phys[i], s->size, MEMREMAP_WB);
	if (!source)
		return -ENOMEM;
	memcpy(s->cpu, source, s->size);
	memunmap(source);
	dma_wmb();
	sha256(s->cpu, s->size, digest);
	return memcmp(digest, i ? b->data_digest : b->text_digest, sizeof(digest)) ?
	       -EKEYREJECTED : 0;
}

/*
 * After resume, the cold way: power up from the cold state with the same
 * checks as at boot (the engine and DARTs must read their reset values),
 * give SID0 its translation back and the DAPF records, put back pristine
 * TEXT and DATA and clean heap, pool and IPC, then run the boot handshake
 * and startup as start-at-boot does. The shared allocations are carried
 * into the new transport and served again; programs loaded before are lost.
 * Every wait is bounded (power polls, READY, ANE_FW_TIMEOUT_NS commands).
 */
static int ane_fw_cold_cycle_boot(struct ane_fw_backend *b)
{
	struct device *dev = &b->pdev->dev;
	struct ane_fw_transport *old;
	u64 start = ktime_get_ns();
	int ret;

	/*
	 * We powered it down ourselves: the cold PMGR state is our evidence.
	 * The engine either lost power (reset values) or kept it with the CPU
	 * stopped by its power state (C4: status 0x62, CE 0x5, DAPF kept);
	 * either boots from the preload once the DARTs are clean and RVBAR
	 * names TEXT.
	 */
	ret = ane_fw_power_up(b, true, false);
	if (ret)
		return ret;
	/*
	 * Whether the engine registers kept their values (DAPF, status 0x62)
	 * although genpd really turned ANE_SYS off says whether they live in
	 * an always-on block.
	 */
	dev_dbg(dev, "H17 ANE: cold cycle finds the engine %s after ANE_SYS %s (status %#x, DAPF0 %#x)\n",
		ane_fw_cold_engine_valid(&b->cold, b->soc) ? "at its reset values" :
		"with registers kept", b->sys_offs ? "really went off" : "stayed on",
		b->cold.status, ane_rd32(b, b->engine + 0x1810000));
	if (!ane_fw_cold_engine_restartable(&b->cold, b->soc))
		return -EPERM;
	ret = ane_fw_rvbar_ready(b);
	if (ret)
		return ret;
	iommu_detach_device(b->domain, dev);
	ret = iommu_attach_device(b->domain, dev);
	if (ret)
		return ret;
	ret = ane_fw_dapf(b);
	if (ret)
		return ret;
	ret = ane_fw_reload_segment(b, 0);
	if (!ret)
		ret = ane_fw_reload_segment(b, 1);
	if (ret)
		return ret;
	memset(b->heap.cpu, 0, b->heap.size);
	if (b->heap_tail.cpu)
		memset(b->heap_tail.cpu, 0, b->heap_tail.size);
	memset(b->pool.cpu, 0, b->pool.size);
	memset(b->ipc.cpu, 0, b->ipc.size);
	dma_wmb();
	old = kmemdup(&b->transport, sizeof(*old), GFP_KERNEL);
	if (!old)
		return -ENOMEM;
	ret = ane_fw_boot(b);
	if (!ret)
		ane_fw_transport_carry_shared(&b->transport, old);
	kfree(old);
	if (!ret)
		ret = ane_fw_startup(b);
	if (ret)
		return ret;
	b->transport.allow_free = true;
	b->resume_ready_ns = ktime_get_ns();
	dev_dbg(dev, "H17 ANE: booted again from the preload after system sleep in %llu ms\n",
		div_u64(b->resume_ready_ns - start, NSEC_PER_MSEC));
	/* Every open program back behind its handle, as the application left it. */
	ane_h16_session_restart(&b->session);
	ret = ane_h16_session_replay(&b->session, &b->resume_replayed, &b->resume_first_ns);
	if (ret) {
		dev_err(dev, "H17 ANE: PROGRAM REPLAY FAILED %d after %u programs; session quarantined\n",
			ret, b->resume_replayed);
		WARN_ONCE(1, "H17 ANE: program replay failed after system sleep (%d)\n", ret);
	}
	return ret;
}

/*
 * macOS stops this firmware before its power goes (ANE_deInit, 25G83): command
 * 1, then SCRATCH0 armed and POWER_DOWN, which is answered in SCRATCH0 and not
 * on the ring. SUSPEND (0x24) is for chips with a warm sleep, not this one.
 */
#define ANE_FW_CMD_STOP 0x1
#define ANE_FW_STOP_SIZE 12
#define ANE_FW_CMD_POWER_DOWN 0xb
#define ANE_FW_POWER_DOWN_SIZE 8
#define ANE_FW_STOP_ARM 0xf7fbdff9
#define ANE_FW_STOP_DONE 0x08042006

static bool clean_stop = true;
module_param(clean_stop, bool, 0644);
MODULE_PARM_DESC(clean_stop, "Stop the H17 firmware before powering it off for system sleep");

#define ane_fw_clean_stop(b) (clean_stop && (b)->soc->exclave_mode_stop)

/* Until SCRATCH0 answers, keep serving what the firmware posts on its way down. */
static int ane_fw_stop_wait(struct ane_fw_backend *b)
{
	struct ane_fw_transport *t = &b->transport;
	void __iomem *scratch0 = b->engine + b->soc->scratch;
	u64 deadline = ktime_get_ns() + NSEC_PER_SEC;
	u8 report[ANE_FW_SUSPEND_REPORT_MAX];
	u32 pending;
	size_t size;
	int ret;

	while (ane_rd32(b, scratch0) != ANE_FW_STOP_DONE) {
		if (ktime_get_ns() >= deadline)
			return -ETIMEDOUT;
		ret = ane_fw_transport_consume(t, report, sizeof(report), &size);
		if (ret && ret != -EAGAIN)
			return ret;
		ret = ane_fw_transport_irq(t, &pending);
		if (ret)
			return ret;
		ret = ane_fw_transport_shared(t);
		if (ret && ret != -EAGAIN)
			return ret;
		usleep_range(100, 200);
	}
	return 0;
}

/* Every port of every ANE DART idle: what macOS checks before it powers one down. */
static int ane_fw_dart_idle(struct ane_fw_backend *b)
{
	for (unsigned int i = 0; i < ANE_FW_DARTS; i++) {
		u32 params = ane_rd32(b, b->dart[i] + 4);
		unsigned int ports = max((params >> 16) & 7, (params >> 20) & 7);

		for (unsigned int p = 0; p < min(ports, 4U); p++) {
			int ret = ane_poll32(b, b->dart[i] + 0x230 + 4 * p, U32_MAX, 0, 1, 10000);

			if (ret) {
				dev_err(&b->pdev->dev, "H17 ANE: DART %u port %u not idle: %#x\n",
					i, p, ane_rd32(b, b->dart[i] + 0x230 + 4 * p));
				return ret;
			}
		}
	}
	return 0;
}

/*
 * Each DART as macOS leaves it before the power words (recorded, 25G83): its counters
 * cleared, error status acknowledged, every enabled stream disabled one at a time, then
 * the TLB flushed.
 */
static void ane_fw_dart_deactivate(struct ane_fw_backend *b)
{
	for (unsigned int i = 0; i < ANE_FW_DARTS; i++) {
		void __iomem *dart = b->dart[i];
		u32 enabled;

		ane_wr32(b, U32_MAX, dart + 0x4060);
		ane_rd32(b, dart + 0x4060);
		ane_rd32(b, dart + 0x4000);
		ane_wr32(b, 0, dart + 0x4000);
		ane_wr32(b, U32_MAX, dart + 0x704);
		ane_rd32(b, dart + 0x704);
		enabled = ane_rd32(b, dart + 0xc00);
		for (unsigned int bit = 0; bit < 16; bit++) {
			if (!(enabled & BIT(bit)) && bit != 11 && bit != 15)
				continue;
			ane_wr32(b, BIT(bit), dart + 0xc20);
			ane_rd32(b, dart + 0xc20);
		}
		/* Flush all: V2 | STT, as macOS writes it. */
		for (unsigned int j = 0; j < ANE_FW_DARTS; j++)
			ane_wr32(b, 0xa000, b->dart[j] + 0x80);
		for (unsigned int j = 0; j < ANE_FW_DARTS; j++)
			ane_rd32(b, b->dart[j] + 0x80);
	}
}

/* h17 push telemetry control; macOS sets bit 0 at start-up. */
#define ANE_FW_TELEMETRY 0x196c000

/*
 * macOS's step right before the stop on this chip (ANE_deInit, its h17 case):
 * push telemetry off once bit 4 reads clear, then SCRATCH14 = 0 and wait for
 * SCRATCH15 bit 0, then SCRATCH16 = 1 and wait for SCRATCH17 bit 0. Each wait
 * is 50 ms and macOS goes on after a miss.
 */
static void ane_fw_stop_handshake(struct ane_fw_backend *b)
{
	void __iomem *scratch = b->engine + b->soc->scratch;
	void __iomem *telemetry = b->engine + ANE_FW_TELEMETRY;
	struct device *dev = &b->pdev->dev;

	if (ane_poll32(b, telemetry, BIT(4), 0, 10, 50000))
		dev_warn(dev, "H17 ANE: telemetry busy before the stop: %#x\n", ane_rd32(b, telemetry));
	ane_wr32(b, ane_rd32(b, telemetry) & ~BIT(0), telemetry);
	ane_wr32(b, 0, scratch + 0x38);
	if (ane_poll32(b, scratch + 0x3c, 1, 1, 10, 50000))
		dev_warn(dev, "H17 ANE: SCRATCH15 not set: %#x\n", ane_rd32(b, scratch + 0x3c));
	ane_wr32(b, 1, scratch + 0x40);
	if (ane_poll32(b, scratch + 0x44, 1, 1, 10, 50000))
		dev_warn(dev, "H17 ANE: SCRATCH17 not set: %#x\n", ane_rd32(b, scratch + 0x44));
}

static int ane_fw_firmware_stop(struct ane_fw_backend *b)
{
	struct ane_fw_transport *t = &b->transport;
	struct device *dev = &b->pdev->dev;
	void __iomem *status = b->engine + b->soc->wrapper + 0x48;
	u8 stop[ANE_FW_STOP_SIZE] = {};
	u8 down[ANE_FW_POWER_DOWN_SIZE] = {};
	u32 val;
	int ret;

	/* A core in WFI can't finish an allocation request, so answer those first. */
	ret = ane_fw_transport_settle(t, 2 * NSEC_PER_MSEC, 20 * NSEC_PER_MSEC);
	if (ret < 0)
		return ret;
	/*
	 * No EXCLAVE_MODE_STOP here: macOS's AP never rings that channel around its sleep
	 * (recorded), and after it this firmware doesn't answer the stop.
	 */
	/* Without the PMU base the stop can't finish: its DM-bus idle query needs it. */
	if (!b->pmu_base_set)
		return -ENODEV;
	ane_fw_stop_handshake(b);
	put_unaligned_le16(ANE_FW_CMD_STOP, stop + 4);
	ret = ane_fw_command(b, ANE_FW_IO, stop, sizeof(stop), ktime_get_ns() + ANE_FW_TIMEOUT_NS);
	if (ret) {
		dev_err(dev, "H17 ANE: STOP failed: %d\n", ret);
		return ret;
	}
	put_unaligned_le16(ANE_FW_CMD_POWER_DOWN, down + 4);
	ane_wr32(b, ANE_FW_STOP_ARM, b->engine + b->soc->scratch);
	ret = ane_fw_transport_publish(t, ANE_FW_IO, down, sizeof(down),
				       ktime_get_ns() + ANE_FW_TIMEOUT_NS);
	if (!ret)
		ret = ane_fw_stop_wait(b);
	if (ret) {
		ane_fw_sleep_trace(b, "POWER_DOWN failed", ane_fw_transport_slot(t, ANE_FW_IO));
		ane_h16_session_dump_fwlog(&b->session);
		return ret;
	}
	/* It never completes on the ring; the boot after the wake starts a new one. */
	t->pending[ANE_FW_IO].active = false;
	/* macOS waits up to 200 ms for WFI and goes on without it. */
	if (readl_poll_timeout(status, val, val & 3, 10000, 200000))
		dev_warn(dev, "H17 ANE: no WFI after POWER_DOWN, status %#x\n", ane_rd32(b, status));
	dev_info(dev, "H17 ANE: firmware stopped, status %#x\n", ane_rd32(b, status));
	ane_wr32(b, 0xfeedbabe, b->engine + b->soc->scratch + 4);
	ret = ane_fw_dart_idle(b);
	if (!ret)
		ane_fw_dart_deactivate(b);
	return ret;
}

/* The cold way down: nothing in flight, stop the firmware, then power off. */
static int ane_fw_cold_cycle_down(struct ane_fw_backend *b)
{
	int ret;

	for (unsigned int ch = 0; ch < ANE_FW_CHANNELS; ch++)
		if (b->transport.pending[ch].active)
			return -EBUSY;
	if (ane_fw_clean_stop(b)) {
		ret = ane_fw_firmware_stop(b);
		/* A refused sleep is a failed suspend: the deep path cuts its power cleanly. */
		if (ret && apple_sleep_request_deep_once())
			dev_warn(&b->pdev->dev, "H17 ANE: no clean stop (%d), taking the deep path\n", ret);
		else if (ret)
			return ret == -EBUSY ? -EIO : ret;
	}
	return ane_fw_power_down(b);
}

/* A request for a parked engine: boot it from the preload, as after a sleep. */
static int ane_fw_session_wake(void *ctx)
{
	struct ane_fw_backend *b = ctx;
	int ret;

	if (READ_ONCE(b->stage) == ANE_STAGE_PROBED)
		return READ_ONCE(b->in_sleep) ? -EAGAIN : ane_fw_first_bringup(b);
	guard(mutex)(&b->lock);
	if (!b->parked)
		return 0;
	/* The system is going to sleep or coming back: PM_POST_SUSPEND wakes waiters. */
	if (b->in_sleep)
		return -EAGAIN;
	ret = ane_fw_cold_cycle_boot(b);
	WRITE_ONCE(b->parked, false);
	WRITE_ONCE(b->session.parked, false);
	if (ret) {
		ane_fw_quarantine(b, ret);
		b->session.quarantined = true;
	}
	ane_h16_session_sleep(&b->session, false);
	dev_info(&b->pdev->dev, "H17 ANE: up for a request%s\n", ret ? " FAILED" : "");
	return ret;
}

static int ane_fw_sleep_prepare(struct ane_fw_backend *b)
{
	int ret;

	guard(mutex)(&b->lock);
	/* Parked: already off, with nothing to stop. */
	if (b->parked)
		return 0;
	/*
	 * A quarantined engine that never came up has nothing to lose; give
	 * back an ANE_SYS reference a failed power-up kept.
	 */
	if (b->stage == ANE_STAGE_QUARANTINED && !b->powered) {
		if (b->power_ref) {
			b->power_ref = false;
			pm_runtime_put_sync(b->power);
		}
		return 0;
	}
	/*
	 * Quarantined and still powered: its state was dumped when it was
	 * quarantined. Power it off without the firmware so that the system
	 * can sleep; it stays quarantined and is not booted again.
	 */
	if (b->stage == ANE_STAGE_QUARANTINED) {
		ret = ane_fw_power_down(b);
		dev_info(&b->pdev->dev, "H17 ANE: quarantined engine powered off for system sleep (%d)\n",
			 ret);
		return ret;
	}
	if (b->stage != ANE_STAGE_PROGRAM)
		return -EBUSY;
	ane_h16_session_sleep(&b->session, true);
	b->sys_offs = 0;
	b->sys_ons = 0;
	b->sys_off_ns = 0;
	b->sys_off_at = 0;
	b->sys_off_pmgr = 0;
	b->sys_on_pmgr = 0;
	if (!ane_fw_sleep_warm(b)) {
		ret = ane_fw_cold_cycle_down(b);
		if (ret == -EBUSY) {
			/* Nothing was written: refuse this sleep and keep running. */
			ane_h16_session_sleep(&b->session, false);
			return ret;
		}
	} else {
		ret = ane_fw_suspend_firmware(b);
		if (!ret)
			ret = ane_fw_power_down(b);
	}
	if (ret) {
		/*
		 * Powered, maybe suspended: the firmware state is uncertain.
		 * Keep power and every mapping, refuse the sleep and close the
		 * session (SPEC-ANE-SLEEP.md section 6).
		 */
		ane_fw_quarantine(b, ret);
		b->session.quarantined = true;
		ane_h16_session_sleep(&b->session, false);
		return ret;
	}
	b->slept = true;
	return 0;
}

static bool dart_off = true;
module_param(dart_off, bool, 0644);
MODULE_PARM_DESC(dart_off, "Deactivate the H17 ANE DARTs before ANE_SYS powers off, as macOS does");

/*
 * Called by genpd under its lock, in the noirq phase too: nothing that sleeps.
 *
 * macOS deactivates the three ANE DARTs before every ANE_SYS power-off, with
 * or without firmware: it also powers ANE_SYS up for DART work alone and down
 * again this way (recorded, 25G83). Here ANE_SYS goes off in genpd, at
 * runtime once the ANE and its DARTs are idle or in the noirq phase of a
 * system suspend, with the DARTs' streams still enabled; without this step
 * it reached off with BUSY latched on most sleeps. Do macOS's step right
 * before the off.
 */
static int ane_fw_genpd_event(struct notifier_block *nb, unsigned long action, void *data)
{
	struct ane_fw_backend *b = container_of(nb, struct ane_fw_backend, genpd_nb);
	u64 now = ktime_get_boottime_ns();

	if (action == GENPD_NOTIFY_PRE_OFF && dart_off && b->soc->exclave_mode_stop &&
	    b->dart[0] && b->dart[1] && b->dart[2]) {
		u32 streams[ANE_FW_DARTS];

		for (unsigned int i = 0; i < ANE_FW_DARTS; i++)
			streams[i] = ane_rd32(b, b->dart[i] + 0xc00);
		ane_fw_dart_deactivate(b);
		dev_info(&b->pdev->dev,
			 "H17 ANE: DARTs off before ANE_SYS: streams were %#x %#x %#x, PMGR %#x\n",
			 streams[0], streams[1], streams[2], ane_rd32(b, b->pmgr + b->soc->pmgr_sys));
	} else if (action == GENPD_NOTIFY_OFF) {
		b->sys_offs++;
		b->sys_off_pmgr = ane_rd32(b, b->pmgr + b->soc->pmgr_sys);
		b->sys_off_at = now;
	} else if (action == GENPD_NOTIFY_ON) {
		b->sys_ons++;
		b->sys_on_pmgr = ane_rd32(b, b->pmgr + b->soc->pmgr_sys);
		if (b->sys_off_at)
			b->sys_off_ns += now - b->sys_off_at;
		b->sys_off_at = 0;
	}
	return NOTIFY_OK;
}

static void ane_fw_sleep_resume(struct ane_fw_backend *b)
{
	s64 ready, first;
	u64 start;
	int ret;

	guard(mutex)(&b->lock);
	if (!b->slept)
		return;
	b->slept = false;
	/* Stay off until a request needs the engine, as macOS does. */
	if (b->soc->macos_sleep && !b->session.quarantined) {
		WRITE_ONCE(b->parked, true);
		WRITE_ONCE(b->session.parked, true);
		dev_info(&b->pdev->dev, "H17 ANE: resume ok: parked until the next request\n");
		return;
	}
	start = ktime_get_ns();
	b->resume_ready_ns = 0;
	b->resume_first_ns = 0;
	b->resume_replayed = 0;
	ret = ane_fw_sleep_warm(b) ? ane_fw_warm_boot(b) : ane_fw_cold_cycle_boot(b);
	if (ret && ane_fw_sleep_warm(b)) {
		/* Fall back to the cold cycle: off again, then boot from the preload. */
		dev_warn(&b->pdev->dev, "H17 ANE: warm boot failed (%d); cold boot and program replay\n",
			 ret);
		ret = b->powered ? ane_fw_power_down(b) : 0;
		if (!ret)
			ret = ane_fw_cold_cycle_boot(b);
	}
	if (ret) {
		/* Every mapping and backing stays for diagnosis. */
		ane_fw_quarantine(b, ret);
		b->session.quarantined = true;
	}
	ane_h16_session_sleep(&b->session, false);
	/* genpd's record of the domain across this sleep (boot time, so it counts s2idle). */
	if (!b->sys_offs || (b->sys_off_pmgr & 0xf0))
		dev_warn(&b->pdev->dev,
			 "H17 ANE: ANE_SYS stayed on through the sleep: genpd off %u, on %u; PMGR %#x at off, %#x at on\n",
			 b->sys_offs, b->sys_ons, b->sys_off_pmgr, b->sys_on_pmgr);
	else
		dev_dbg(&b->pdev->dev,
			"H17 ANE: ANE_SYS while asleep: genpd off %u, on %u, off for %llu ms; PMGR %#x at off, %#x at on\n",
			b->sys_offs, b->sys_ons, div_u64(b->sys_off_ns, NSEC_PER_MSEC),
			b->sys_off_pmgr, b->sys_on_pmgr);
	/* One line per wake; times from PM_POST_SUSPEND, which follows the thaw. */
	ready = b->resume_ready_ns ? div_u64(b->resume_ready_ns - start, NSEC_PER_MSEC) : -1;
	first = b->resume_first_ns ? div_u64(b->resume_first_ns - start, NSEC_PER_MSEC) : -1;
	dev_info(&b->pdev->dev,
		 "H17 ANE: resume %s: firmware ready %lld ms, first program ready %lld ms, node reopened %llu ms, %u programs replayed\n",
		 ret ? "FAILED" : "ok", ready, first,
		 div_u64(ktime_get_ns() - start, NSEC_PER_MSEC), b->resume_replayed);
}

/*
 * The sleep this notifier refuses does not happen, and the PM core's rollback gives PM_POST_SUSPEND
 * only to the notifiers before the refusing one: undo PM_SUSPEND_PREPARE's block here, or every
 * request would wait (-EAGAIN) until a later sleep completes.
 */
static void ane_fw_sleep_unblock(struct ane_fw_backend *b)
{
	WRITE_ONCE(b->in_sleep, false);
	WRITE_ONCE(b->session.wake_blocked, false);
	wake_up_all(&b->session.wake);
}

static int ane_fw_pm_event(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct ane_fw_backend *b = container_of(nb, struct ane_fw_backend, pm_notifier);
	bool retained = READ_ONCE(b->retained);

	/*
	 * Registered at init, before anything is retained: until then there
	 * is nothing to cycle or refuse. Retaining happens under the system
	 * sleep lock, so it never changes in the middle of a transition.
	 */
	if (!retained)
		return NOTIFY_DONE;
	if (event == PM_POST_SUSPEND) {
		ane_fw_sleep_resume(b);
		WRITE_ONCE(b->in_sleep, false);
		WRITE_ONCE(b->session.wake_blocked, false);
		/* Requests that waited out the sleep now wake the parked engine. */
		wake_up_all(&b->session.wake);
		return NOTIFY_DONE;
	}
	if (event == PM_SUSPEND_PREPARE) {
		WRITE_ONCE(b->in_sleep, true);
		WRITE_ONCE(b->session.wake_blocked, true);
	}
	if (event == PM_SUSPEND_PREPARE && retained) {
		if (!ane_fw_sleep_prepare(b))
			return NOTIFY_DONE;
		ane_fw_sleep_unblock(b);
		dev_warn_once(&b->pdev->dev, ANE_FW_SLEEP_REFUSED);
		return NOTIFY_BAD;
	}
	/* Hibernation and restore stay refused. */
	if (ane_fw_sleep_refused_cycle(event, retained, true)) {
		if (event == PM_SUSPEND_PREPARE)
			ane_fw_sleep_unblock(b);
		dev_warn_once(&b->pdev->dev, ANE_FW_SLEEP_REFUSED);
		return NOTIFY_BAD;
	}
	return NOTIFY_DONE;
}

static int ane_fw_retain(struct ane_fw_backend *b)
{
	int ret;

	if (b->retained)
		return 0;

	/*
	 * The kexec blocker takes built-in users only. A module, like the H16
	 * backend, pins itself and refuses system sleep, but cannot veto kexec.
	 */
	if (IS_BUILTIN(CONFIG_DRM_ACCEL_ANE)) {
		ret = kexec_block(&ane_fw_kexec);
		if (ret)
			return ret;
	}
	/*
	 * The PM notifier was registered at init: registering it here, under
	 * b->lock, would order the PM chain rwsem after b->lock, while
	 * ane_fw_pm_event() takes them the other way round.
	 */
	__module_get(THIS_MODULE);
	WRITE_ONCE(b->retained, true);
	return 0;
}

static int ane_fw_map(struct ane_fw_backend *b, u64 dva, phys_addr_t pa,
		      size_t size, int prot)
{
	int ret = iommu_map(b->domain, dva, pa, size, prot, GFP_KERNEL);

	if (ret)
		return ret;
	/* iommu_map includes the DART's normal range synchronization and
	 * propagates its timeout. Extra invalidates do not fix stale output
	 * BAR bindings between programs; the transport's queued first EXECUTE
	 * after a program change does.
	 */
	for (unsigned int i = 0; i < ANE_FW_DARTS; i++)
		if (ane_rd32(b, b->dart[i] + 0x100))
			return -EIO;
	return 0;
}

/*
 * Coherent (uncached) memory for firmware-owned state, or, with cached,
 * CPU-cacheable memory for program surfaces that the CPU copies to and from
 * on every request. The engine does not snoop either; cached surfaces are
 * kept coherent by ane_fw_program_sync() around each use.
 */
static int ane_fw_alloc_mem(struct ane_fw_backend *b, u64 dva, size_t size,
			    int prot, bool cached, struct ane_fw_surface *s)
{
	struct ane_fw_mapping *m;
	int ret;

	/* No memory for a range the window cannot hold. */
	if (dva == ANE_FW_WINDOW_ANY && size > ANE_FW_WINDOW)
		return -ENOSPC;

	m = kzalloc_obj(*m);
	if (!m)
		return -ENOMEM;
	if (cached)
		m->surface.cpu = dma_alloc_noncoherent(b->memory, size, &m->physical,
						       DMA_BIDIRECTIONAL, GFP_KERNEL);
	else
		m->surface.cpu = dma_alloc_coherent(b->memory, size, &m->physical,
						    GFP_KERNEL);
	if (!m->surface.cpu) {
		kfree(m);
		return -ENOMEM;
	}
	if (cached) {
		/* Zero it and write the zeroes back before the engine sees it. */
		memset(m->surface.cpu, 0, size);
		dma_sync_single_for_device(b->memory, m->physical, size, DMA_BIDIRECTIONAL);
		m->surface.handle = m->physical;
	}
	mutex_lock(&b->window_lock);
	if (dva == ANE_FW_WINDOW_ANY) {
		ret = drm_mm_insert_node_in_range(&b->dva_window, &m->node, size, SZ_16K, 0,
						  ANE_FW_ALLOC_DVA,
						  ANE_FW_ALLOC_DVA + ANE_FW_WINDOW,
						  DRM_MM_INSERT_LOW);
		if (ret) {
			mutex_unlock(&b->window_lock);
			if (cached)
				dma_free_noncoherent(b->memory, size, m->surface.cpu,
						     m->physical, DMA_BIDIRECTIONAL);
			else
				dma_free_coherent(b->memory, size, m->surface.cpu, m->physical);
			kfree(m);
			return ret;
		}
		dva = m->node.start;
	}
	m->surface.dva = dva;
	m->surface.size = size;
	/*
	 * Record before publication, including a failed/partial mapping: it
	 * keeps its memory and range until reset.
	 */
	list_add_tail(&m->list, &b->mappings);
	mutex_unlock(&b->window_lock);
	ret = ane_fw_map(b, dva, m->physical, size, prot);
	if (!ret)
		*s = m->surface;
	return ret;
}

static int ane_fw_alloc_at(struct ane_fw_backend *b, u64 dva, size_t size,
			   int prot, struct ane_fw_surface *s)
{
	return ane_fw_alloc_mem(b, dva, size, prot, false, s);
}

static int ane_fw_alloc(void *ctx, size_t size, u32 tag, struct ane_fw_surface *s)
{
	struct ane_fw_backend *b = ctx;
	int ret;

	if (!size || size > ANE_FW_SHARED_TOTAL)
		return -ENOSPC;
	ret = ane_fw_alloc_at(b, ANE_FW_WINDOW_ANY, size, IOMMU_READ | IOMMU_WRITE, s);
	if (!ret)
		dev_dbg(&b->pdev->dev, "SHAREDMALLOC tag=%#x bytes=%zu\n", tag, size);
	return ret;
}

static u64 ane_fw_load64(void *ctx, const void *p)
{
	return le64_to_cpu(READ_ONCE(*(__le64 *)p));
}

static void ane_fw_store64(void *ctx, void *p, u64 value)
{
	WRITE_ONCE(*(__le64 *)p, cpu_to_le64(value));
}

static void ane_fw_rmb(void *ctx)
{
	dma_rmb();
}

static void ane_fw_wmb(void *ctx)
{
	dma_wmb();
}

static u32 ane_fw_read32(void *ctx, u32 reg)
{
	struct ane_fw_backend *b = ctx;

	return ane_rd32(b, b->engine + reg);
}

static void ane_fw_write32(void *ctx, u32 reg, u32 value)
{
	struct ane_fw_backend *b = ctx;

	ane_wr32(b, value, b->engine + reg);
	if (reg == b->soc->irq_ack) {
		/* Complete the snapshot acknowledgment before coherent ring reads. */
		mb();
	}
}

static u64 ane_fw_now(void *ctx)
{
	return ktime_get_ns();
}

/*
 * Every wait polls, with the shared pacing (see ane_fw_poll_sleep_us()):
 * busy steps for the first 30 us, then short sleeps, backing off for long
 * waits such as a stalled request.
 */
static void ane_fw_relax(void *ctx, u64 waited_ns)
{
	ane_fw_poll_pause(waited_ns);
}

static void ane_fw_refused(void *ctx, u64 word0, u64 word1, u64 word2, int error)
{
	struct ane_fw_backend *b = ctx;

	dev_info(&b->pdev->dev, "SHAREDMALLOC refused tuple=%#llx/%#llx/%#llx error=%d\n",
		 word0, word1, word2, error);
}

static void ane_fw_cold_shared_note(void *ctx, const char *event, u64 dva, u64 size, u32 tag)
{
	struct ane_fw_backend *b = ctx;

	dev_dbg(&b->pdev->dev, "SHAREDMALLOC %s tag=%#x bytes=%llu dva=%#llx\n",
		event, tag, size, dva);
}

static const struct ane_fw_transport_ops ane_fw_ops = {
	.load64 = ane_fw_load64, .store64 = ane_fw_store64,
	.read_barrier = ane_fw_rmb, .write_barrier = ane_fw_wmb,
	.read32 = ane_fw_read32, .write32 = ane_fw_write32,
	.now_ns = ane_fw_now, .relax = ane_fw_relax,
	.alloc = ane_fw_alloc, .refused = ane_fw_refused,
	.shared_note = ane_fw_cold_shared_note,
};

/*
 * Power in the order the H17 U2 runs measured: genpd ANE_SYS on, PWGATE read,
 * CPU power state on, PWGATE request cleared, sub-block power state on. Only
 * then are the engine and DARTs read and checked for their reset state, before
 * Linux writes anything to the engine. A failed check after power-on retains
 * power and mappings until reboot, like any other quarantine.
 */
static int ane_fw_power(struct ane_fw_backend *b, bool raw_boot)
{
	struct device *dev = &b->pdev->dev;
	int ret;

	/*
	 * The bound device carries ANE_SYS itself (platform_probe attached it
	 * to its single domain), so genpd sees the engine's own reference.
	 */
	if (!dev->pm_domain || !ane_fw_bound_dev)
		return -ENODEV;
	b->power = dev;
	ret = ane_fw_power_up(b, raw_boot, true);
	/* The bring-up holds ANE_SYS now (or keeps it, quarantined). */
	ane_fw_bind_release(b);
	return ret;
}

/*
 * ANE_SYS on, then the child power steps from the cold state, then (for a
 * cold boot) the engine's reset values, and CE. At boot and again after a
 * power-down for sleep.
 */
static int ane_fw_power_up(struct ane_fw_backend *b, bool raw_boot, bool cold_engine)
{
	void __iomem *cpu = b->pmgr + b->soc->pmgr_cpu, *sub = b->pmgr + b->soc->pmgr_subblock;
	struct device *dev = &b->pdev->dev;
	u64 ce;
	int ret;

	/* Taken once; a failed attempt keeps it for the retry or the quarantine. */
	if (!b->power_ref) {
		ret = pm_runtime_resume_and_get(b->power);
		if (ret < 0)
			return ret;
		b->power_ref = true;
	}
	if ((ane_rd32(b, b->pmgr + b->soc->pmgr_sys) & 0xff) != 0xff)
		return -EIO;
	/* The child state may not have moved since the probe stage. */
	ane_fw_snapshot_pmgr(b);
	ane_fw_snapshot_gate(b);
	if (!ane_fw_cold_pmgr_valid(&b->cold, raw_boot) || !ane_fw_cold_gate_valid(&b->cold)) {
		dev_info(dev, "H17 ANE: child power or gate not in the cold state; refusing\n");
		return -EPERM;
	}
	ane_wr32(b, ane_rd32(b, cpu) | 0xf, cpu);
	ret = ane_poll32(b, cpu, 0xff, 0xff, 1, 100000);
	if (ret)
		return ret;
	ane_wr32(b, 0, b->gate + 0xc);
	ret = ane_poll32(b, b->gate + 0xc, 3, 0, 1, 100000);
	if (ret)
		return ret;
	ane_wr32(b, ane_rd32(b, sub) | 0xf, sub);
	ret = ane_poll32(b, sub, 0xff, 0xff, 1, 100000);
	if (ret)
		return ret;
	ret = ane_poll32(b, b->gate + 0x30, U32_MAX, 0, 1, 100000);
	if (ret)
		return ret;
	b->powered = true;
	ane_fw_snapshot_engine(b);
	/* A warm resume finds a suspended firmware's state, not reset values. */
	if (cold_engine && !ane_fw_cold_engine_valid(&b->cold, b->soc)) {
		dev_info(dev, "H17 ANE: engine or DART not in its reset state; refusing\n");
		return -EPERM;
	}
	ane_wr64(b, 3, b->engine + 0x18a4000);
	ane_fw_trace(b, 'P', b->engine + 0x18a4008, 3);
	ret = readq_poll_timeout(b->engine + 0x18a4008, ce, (ce & 3) == 3, 1, 100000);
	if (ret)
		return ret;
	ane_wr64(b, 3, b->engine + 0x18a4008);
	if ((ane_rd64(b, b->engine + 0x18a4010) & 3) != 3 ||
	    ane_rd64(b, b->engine + 0x18a4018))
		return -EIO;
	dev_dbg(dev, "ANE power: ANE_SYS on, child/PWGATE/CE ready\n");
	return 0;
}

static int ane_fw_dapf(struct ane_fw_backend *b)
{
	unsigned int i;

	for (i = 0; i < b->soc->dapf_count; i++) {
		void __iomem *r = b->engine + 0x1810000 + i * 0x40;
		u64 start = b->ranges[i * 2], end = b->ranges[i * 2 + 1];

		if (ane_rd32(b, b->dart[0] + 0x208) & BIT(2))
			return -EPERM;
		ane_wr32(b, 0x31, r);
		ane_wr32(b, lower_32_bits(start), r + 8);
		ane_wr32(b, upper_32_bits(start), r + 0xc);
		ane_wr32(b, lower_32_bits(end), r + 0x10);
		ane_wr32(b, upper_32_bits(end), r + 0x14);
		ane_wr32(b, 1, r + 0x20);
		if ((ane_rd32(b, r) & 0x333) != 0x31 || ane_rd64(b, r + 8) != start ||
		    ane_rd64(b, r + 0x10) != end || (ane_rd32(b, r + 0x20) & 0xffff) != 1)
			return -EIO;
	}
	return 0;
}

/*
 * Whether TEXT is the build's firmware: as loaded (text_pin, measured) or as
 * code, with the loader-filled environment sections zeroed (text_code_pin,
 * from the image). Both digests are returned for the log; code is zero
 * without a code pin or with an environment table that does not fit TEXT.
 */
static bool ane_fw_text_pinned(const struct ane_fw_platform *soc, const u8 *text,
			       size_t size, u8 raw[SHA256_DIGEST_SIZE],
			       u8 code[SHA256_DIGEST_SIZE])
{
	static const u8 zero[256];
	struct sha256_ctx hash;
	size_t pos = 0;
	unsigned int i;
	bool ok;

	sha256(text, size, raw);
	ok = soc->text_pin && !memcmp(raw, soc->text_pin, SHA256_DIGEST_SIZE);
	memset(code, 0, SHA256_DIGEST_SIZE);
	if (!soc->text_code_pin || !soc->text_env[0].size)
		return ok;
	for (i = 0; i < ARRAY_SIZE(soc->text_env) && soc->text_env[i].size; i++) {
		const struct ane_fw_text_env *env = &soc->text_env[i];

		if (env->offset < pos || env->offset > size || env->size > size - env->offset)
			return ok;
		pos = env->offset + env->size;
	}
	sha256_init(&hash);
	pos = 0;
	for (i = 0; i < ARRAY_SIZE(soc->text_env) && soc->text_env[i].size; i++) {
		const struct ane_fw_text_env *env = &soc->text_env[i];
		size_t left = env->size;

		sha256_update(&hash, text + pos, env->offset - pos);
		while (left) {
			size_t n = min(left, sizeof(zero));

			sha256_update(&hash, zero, n);
			left -= n;
		}
		pos = env->offset + env->size;
	}
	sha256_update(&hash, text + pos, size - pos);
	sha256_final(&hash, code);
	return ok || !memcmp(code, soc->text_code_pin, SHA256_DIGEST_SIZE);
}

/*
 * The loader's preload hand-off. apple,preload-data-sha256 is optional: with
 * it the private DATA copy must match the loader's digest of what it handed
 * off; without it, the live DATA at copy time (ane_fw_preload()).
 */
static int ane_fw_preload_metadata(struct ane_fw_backend *b, u64 physical[2],
				   u64 size[2], u64 dva[2], u8 data_pin[SHA256_DIGEST_SIZE],
				   bool *have_data_pin)
{
	struct device_node *np = b->pdev->dev.of_node;
	unsigned int i;
	int ret;

	if (!of_property_read_bool(np, "apple,firmware-preloaded") ||
	    of_property_count_u32_elems(np, "memory-region") != 2 ||
	    of_property_read_u64_array(np, "apple,firmware-iovas", dva, 2))
		return -ENODATA;
	*have_data_pin = !of_property_read_u8_array(np, "apple,preload-data-sha256",
						     data_pin, SHA256_DIGEST_SIZE);
	/* The same no-map, outside-System-RAM rule as the H16 preload. */
	for (i = 0; i < 2; i++) {
		const char *name;

		if (of_property_read_string_index(np, "memory-region-names", i, &name) ||
		    strcmp(name, i ? "firmware-data" : "firmware-text"))
			return -EINVAL;
		ret = ane_fw_preload_range(np, i, &physical[i], &size[i]);
		if (ret)
			return ret;
	}
	if (!ane_fw_preload_ranges_valid(b->soc, physical, size, dva))
		return -EINVAL;
	return 0;
}

static int ane_fw_preload(struct ane_fw_backend *b)
{
	u64 physical[2], size[2], dva[2], tail, tail_size;
	struct ane_fw_surface snapshots[2], heap_tail;
	u8 digest[SHA256_DIGEST_SIZE], code[SHA256_DIGEST_SIZE];
	u8 data_pin[SHA256_DIGEST_SIZE];
	bool have_data_pin;
	unsigned int i;
	int ret;

	ret = ane_fw_preload_metadata(b, physical, size, dva, data_pin, &have_data_pin);
	if (ret)
		return ret;
	for (i = 0; i < 2; i++) {
		void *source = memremap(physical[i], size[i], MEMREMAP_WB);
		struct sha256_ctx hash;

		if (!source)
			return -ENOMEM;
		/* Without a loader digest, the copy must match the live DATA. */
		if (i && !have_data_pin)
			sha256(source, size[i], data_pin);
		ret = ane_fw_alloc_at(b, dva[i], size[i],
				      IOMMU_READ | (i ? IOMMU_WRITE : 0), &snapshots[i]);
		if (!ret) {
			memcpy(snapshots[i].cpu, source, size[i]);
			b->preload[i] = snapshots[i];
			b->preload_phys[i] = physical[i];
		}
		memunmap(source);
		if (ret)
			return ret;
		if (!i) {
			/* The copy is what a reload after sleep must give back. */
			if (!ane_fw_text_pinned(b->soc, snapshots[0].cpu, size[0],
						b->text_digest, code))
				return -EKEYREJECTED;
			continue;
		}
		sha256_init(&hash);
		sha256_update(&hash, snapshots[i].cpu, size[i]);
		sha256_final(&hash, digest);
		if (memcmp(digest, data_pin, sizeof(digest)))
			return -EKEYREJECTED;
	}
	/* Preserve every personalized DATA byte, including the iBoot TLV table. */
	dma_wmb();
	memcpy(b->data_digest, data_pin, sizeof(b->data_digest));
	ret = ane_fw_heap_tail(dva, size, ANE_FW_POOL_DVA, &tail, &tail_size);
	if (ret || !tail_size)
		return ret;
	/* Zeroed by the coherent allocation; kept until reset like the preload. */
	ret = ane_fw_alloc_at(b, tail, tail_size, IOMMU_READ | IOMMU_WRITE, &heap_tail);
	if (!ret)
		b->heap_tail = heap_tail;
	if (!ret)
		dev_info(&b->pdev->dev, "ANE map: firmware heap tail at %#llx bytes %#llx\n",
			 tail, tail_size);
	return ret;
}

/* An earlier DAPF record already covers record i's 16 KiB page. */
static bool ane_fw_dapf_page_seen(const struct ane_fw_backend *b, unsigned int i)
{
	for (unsigned int j = 0; j < i; j++)
		if (round_down(b->ranges[j * 2], SZ_16K) == round_down(b->ranges[i * 2], SZ_16K))
			return true;
	return false;
}

static int ane_fw_prepare(struct ane_fw_backend *b)
{
	struct device *dev = &b->pdev->dev;
	unsigned int i;
	int ret;

	ret = ane_fw_power(b, ane_fw_raw_boot());
	if (ret)
		return ret;
	ret = of_dma_configure(dev, dev->of_node, true);
	if (ret)
		return ret;
	ret = iommu_device_claim_dma_owner(dev, b);
	if (ret)
		return ret;
	b->domain = iommu_paging_domain_alloc(dev);
	if (IS_ERR(b->domain))
		return PTR_ERR(b->domain);
	ret = iommu_attach_device(b->domain, dev);
	if (ret)
		return ret;
	if (!(b->domain->pgsize_bitmap & SZ_16K) || !b->domain->ops->iotlb_sync_map ||
	    b->domain->geometry.aperture_start ||
	    b->domain->geometry.aperture_end < ANE_FW_ALLOC_DVA + ANE_FW_WINDOW)
		return -ERANGE;
	b->memory = root_device_register("ane-fw-memory");
	if (IS_ERR(b->memory))
		return PTR_ERR(b->memory);
	ret = dma_coerce_mask_and_coherent(b->memory, DMA_BIT_MASK(64));
	if (ret)
		return ret;
	ret = ane_fw_preload(b);
	if (ret)
		return ret;
	for (i = 0; i < b->soc->dapf_count; i++) {
		u64 page = round_down(b->ranges[i * 2], SZ_16K);

		/* T6050's first two records share a PMGR page: map it once. */
		if (ane_fw_dapf_page_seen(b, i))
			continue;
		ret = ane_fw_map(b, page, page, SZ_16K, IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO);
		if (ret)
			return ret;
	}
	ret = ane_fw_dapf(b);
	if (ret)
		return ret;
	ret = ane_fw_alloc_at(b, ANE_FW_POOL_DVA, SZ_256K,
			      IOMMU_READ | IOMMU_WRITE, &b->pool);
	if (ret)
		return ret;
	ret = ane_fw_alloc_at(b, ANE_FW_IPC_DVA, SZ_64K,
			      IOMMU_READ | IOMMU_WRITE, &b->ipc);
	if (ret)
		return ret;
	ret = ane_fw_alloc_at(b, ANE_FW_HEAP_DVA, 15 * SZ_1M,
			      IOMMU_READ | IOMMU_WRITE, &b->heap);
	if (!ret)
		dev_info(dev, "ANE map: reserved preload, SID0 on three DARTs, DAPF verified\n");
	return ret;
}

static int ane_fw_scratch_wait(struct ane_fw_backend *b, unsigned int cell, u32 wanted)
{
	return ane_poll32(b, b->engine + b->soc->scratch + cell * 4, U32_MAX, wanted,
			  100, 3000000);
}

static int ane_fw_boot(struct ane_fw_backend *b)
{
	void __iomem *scratch = b->engine + b->soc->scratch;
	u8 *init = b->pool.cpu + 0x180;
	u64 table;
	unsigned int i;
	int ret;

	/* A cold boot starts the firmware in Exclave mode again. */
	b->exclave_mode_stopped = false;
	for (i = 0; i < 8; i++)
		ane_wr32(b, 0, scratch + i * 4);
	ane_wr32(b, 1, scratch + 6 * 4);
	ane_wr32(b, 0, b->engine + b->soc->wrapper + 0x44);
	dma_wmb();
	ane_wr32(b, 0x10, b->engine + b->soc->wrapper + 0x44);
	ret = ane_fw_scratch_wait(b, 7, 0x08042006);
	if (ret)
		return ret;
	dev_dbg(&b->pdev->dev, "ANE boot: READY\n");
	if (ane_rd32(b, scratch) != 8 || ane_rd32(b, scratch + 4) != 0xc440 ||
	    ane_rd32(b, scratch + 12) != 0xf00000 || ane_rd32(b, scratch + 16))
		return -EPROTO;
	put_unaligned_le64(b->ipc.dva, init + 8);
	put_unaligned_le64(ANE_FW_SPAN, init + 0x10);
	put_unaligned_le64(0xfc80000, init + 0x18);
	put_unaligned_le64(b->heap.dva, init + 0x20);
	put_unaligned_le64(b->heap.size, init + 0x28);
	put_unaligned_le64(0xc441, init + 0x50);
	put_unaligned_le64(b->pool.dva, init + 0x58);
	put_unaligned_le64(b->pool.size, init + 0x60);
	put_unaligned_le32(64, init + 0x68);
	put_unaligned_le32(4, init + 0x12c);
	dma_wmb();
	ane_wr32(b, lower_32_bits(b->pool.dva + 0x180), scratch);
	ane_wr32(b, upper_32_bits(b->pool.dva + 0x180), scratch + 4);
	ane_wr32(b, 0xf7fbdff9, scratch + 7 * 4);
	ret = ane_fw_scratch_wait(b, 7, 0x08042006);
	if (ret)
		return ret;
	table = (u64)ane_rd32(b, scratch + 4) << 32 | ane_rd32(b, scratch);
	if (table != b->ipc.dva)
		return -EPROTO;
	ret = ane_fw_transport_init(&b->transport, b->soc, &ane_fw_ops, b,
				    b->ipc, b->pool, 0x4000, table, ANE_FW_TABLE_SIZE);
	if (ret)
		return ret;
	ret = ane_fw_transport_arm(&b->transport);
	if (ret)
		return ret;
	ane_wr32(b, 0x08042006, scratch + 3 * 4);
	ret = ane_fw_scratch_wait(b, 3, 0);
	if (!ret)
		dev_dbg(&b->pdev->dev, "ANE boot: eight channels armed, scratch3 cleared (polled)\n");
	/* The firmware's table order, which the macOS channel selectors index. */
	for (i = 0; !ret && i < ANE_FW_CHANNELS; i++) {
		const struct ane_fw_channel *c = &b->transport.channels[i];

		dev_dbg(&b->pdev->dev,
			"ANE channel %s: table position %u, type %u, doorbell bit %u, %u slots at IPC +%#llx\n",
			ane_fw_channel_name(i), c->index, c->type, c->bit, c->slots, c->offset);
	}
	return ret;
}

static int ane_fw_command(struct ane_fw_backend *b, unsigned int channel,
			  void *command, size_t size, u64 deadline)
{
	struct ane_fw_transaction *tx = &b->transport.pending[channel];
	int ret;

	ret = ane_fw_transport_publish(&b->transport, channel, command, size, deadline);
	if (ret)
		return ret;
	ret = ane_fw_transport_wait(&b->transport, channel);
	if (!ret)
		memcpy(command, b->pool.cpu + tx->dva - b->pool.dva, size);
	return ret;
}

/*
 * SET_SNE_PMU_BASE2 (0x29) with the DT "pmgr" range base, once per firmware
 * boot, right after START, on the IO ring. macOS's command helper takes a
 * queue, not a table position: 0 is IO, 1 DEBUG, 2 DATA_CHAIN_H2T
 * (aneFirmwareCommandSend, 25G83), and 0x29 goes on queue 0, never DEBUG.
 */
static int ane_fw_pmu_base(struct ane_fw_backend *b)
{
	u8 command[ANE_FW_PMU_BASE_SIZE] = {};
	u64 base = b->window[1].pa;
	int ret;

	b->pmu_base_set = false;
	put_unaligned_le16(ANE_FW_CMD_SET_PMU_BASE, command + 4);
	put_unaligned_le64(base, command + 8);
	ret = ane_fw_command(b, ANE_FW_IO, command, sizeof(command),
			     ktime_get_ns() + ANE_FW_TIMEOUT_NS);
	dev_info(&b->pdev->dev, "H17 ANE: SET_SNE_PMU_BASE2 (0x29) %#llx on ring IO: %d\n",
		 base, ret);
	if (!ret)
		b->pmu_base_set = true;
	return ret;
}

static int ane_fw_startup(struct ane_fw_backend *b)
{
	static const struct { u16 id, size; } commands[] = {
		{ 4, 12 }, { 0, 12 }, { 3, 16 }, { 0x22, 100 },
	};
	u8 command[100];
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		memset(command, 0, sizeof(command));
		put_unaligned_le16(commands[i].id, command + 4);
		ret = ane_fw_command(b, ANE_FW_IO, command, commands[i].size,
				     ktime_get_ns() + ANE_FW_TIMEOUT_NS);
		if (ret)
			return ret;
		if (commands[i].id == 4 && get_unaligned_le32(command + 8))
			return -EPROTO;
		if (commands[i].id == 3 && ane_fw_config_status(command, 16))
			return -EPROTO;
		dev_dbg(&b->pdev->dev, "ANE startup: command %#x ack=0\n", commands[i].id);
		if (commands[i].id == 3)
			dev_dbg(&b->pdev->dev, "ANE CONFIG_GET tick=%u\n",
				get_unaligned_le32(command + 8));
		if (commands[i].id == 0x22)
			dev_dbg(&b->pdev->dev, "ANE RESOURCE_INFO: %*ph\n", 100, command);
		/* macOS order: START, then SET_SNE_PMU_BASE2, then CONFIG_GET. */
		if (commands[i].id == 0 && (b->soc->sleep_warm || ane_fw_clean_stop(b))) {
			ret = ane_fw_pmu_base(b);
			if (ret)
				return ret;
		}
	}
	/* The host flags macOS sets once the firmware is up (recorded, 25G83). */
	if (b->soc->macos_sleep) {
		ane_wr32(b, 1, b->engine + b->soc->scratch + 0x38);
		ane_wr32(b, 0, b->engine + b->soc->scratch + 0x40);
	}
	return 0;
}

/* Private program mappings come from the same window as SHAREDMALLOC. */
static int ane_fw_program_map(void *ctx, size_t size, bool write, struct ane_fw_surface *out)
{
	struct ane_fw_backend *b = ctx;

	if (!size || !IS_ALIGNED(size, SZ_16K))
		return -ENOSPC;
	return ane_fw_alloc_mem(b, ANE_FW_WINDOW_ANY, size,
				IOMMU_READ | (write ? IOMMU_WRITE : 0), true, out);
}

/*
 * A program surface the firmware no longer uses (ops->unmap). The session
 * calls it with the engine up, so the DART invalidation that iommu_unmap()
 * ends with reaches the engine before its pages and its DVA range go back.
 * A short unmap keeps everything, as a failed mapping does.
 */
static void ane_fw_program_unmap(void *ctx, const struct ane_fw_surface *s)
{
	struct ane_fw_backend *b = ctx;
	struct ane_fw_mapping *m, *found = NULL;
	size_t unmapped;

	mutex_lock(&b->window_lock);
	list_for_each_entry(m, &b->mappings, list) {
		if (m->surface.dva == s->dva && m->surface.size == s->size &&
		    m->surface.handle && drm_mm_node_allocated(&m->node)) {
			found = m;
			list_del(&m->list);
			break;
		}
	}
	mutex_unlock(&b->window_lock);
	if (WARN_ON_ONCE(!found))
		return;
	m = found;
	unmapped = iommu_unmap(b->domain, s->dva, s->size);
	if (unmapped != s->size) {
		dev_err(&b->pdev->dev, "H17 ANE: program unmap short at %#llx: %zu of %zu; kept\n",
			s->dva, unmapped, s->size);
		mutex_lock(&b->window_lock);
		list_add_tail(&m->list, &b->mappings);
		mutex_unlock(&b->window_lock);
		return;
	}
	dma_free_noncoherent(b->memory, s->size, m->surface.cpu, m->physical,
			     DMA_BIDIRECTIONAL);
	mutex_lock(&b->window_lock);
	drm_mm_remove_node(&m->node);
	mutex_unlock(&b->window_lock);
	kfree(m);
}

static void ane_fw_program_log(void *ctx, const char *item, u64 a, u64 c)
{
	struct ane_fw_backend *b = ctx;

	dev_info(&b->pdev->dev, "H17 %s %llx %llx\n", item, a, c);
}

/* Program surfaces are cacheable: clean before the engine, invalidate after. */
static void ane_fw_program_sync(void *ctx, const struct ane_fw_surface *s, bool for_device)
{
	struct ane_fw_backend *b = ctx;

	if (for_device)
		dma_sync_single_for_device(b->memory, s->handle, s->size, DMA_BIDIRECTIONAL);
	else
		dma_sync_single_for_cpu(b->memory, s->handle, s->size, DMA_BIDIRECTIONAL);
}

static void ane_fw_program_busy(void *ctx, bool busy)
{
	struct ane_fw_backend *b = ctx;

	apple_clpc_ane_set_active(b->clpc, busy);
}

static const struct ane_h16_session_ops ane_fw_session_ops = {
	.map = ane_fw_program_map,
	.unmap = ane_fw_program_unmap,
	.log = ane_fw_program_log,
	.sync = ane_fw_program_sync,
	.busy = ane_fw_program_busy,
	.wake = ane_fw_session_wake,
};

/*
 * After startup: the shared program session and its ABI 3 node. From here on
 * the node serializes all firmware traffic. The firmware frees each process
 * pool after UNLOAD, often only during a later program, so FREE stays
 * admitted; backing is retained until reboot.
 */
/*
 * <debugfs>/ane-h17/<dev>/execute: per-phase EXECUTE times of the shared
 * session, see ane_h16_session_stats_show(). Writing anything resets them.
 * Created at the program stage; the retained backend pins the module, so
 * the file and its session live until reboot.
 */
static int ane_fw_execute_show(struct seq_file *m, void *v)
{
	return ane_h16_session_stats_show(m, m->private);
}

static int ane_fw_execute_open(struct inode *inode, struct file *file)
{
	return single_open(file, ane_fw_execute_show, inode->i_private);
}

static ssize_t ane_fw_execute_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	ane_h16_session_stats(((struct seq_file *)file->private_data)->private, NULL, true);
	return count;
}

static const struct file_operations ane_fw_execute_fops = {
	.owner = THIS_MODULE,
	.open = ane_fw_execute_open,
	.read = seq_read,
	.write = ane_fw_execute_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void ane_fw_debugfs(struct ane_fw_backend *b)
{
	struct dentry *root = debugfs_lookup("ane-h17", NULL);
	bool found = root;

	if (!found)
		root = debugfs_create_dir("ane-h17", NULL);
	debugfs_create_file("execute", 0600,
			    debugfs_create_dir(dev_name(&b->pdev->dev), root),
			    &b->session, &ane_fw_execute_fops);
	if (found)
		dput(root);
}

/*
 * How long dynamic power gating stays off after the last EXECUTE, as on
 * H16: toggling it around every request costs the H17 engine about 70 us
 * (gate open plus gate close) of each back-to-back EXECUTE.
 */
#define ANE_FW_GATE_HOLD_NS (2 * NSEC_PER_MSEC)

static int ane_fw_program_stage(struct ane_fw_backend *b)
{
	int ret;

	b->transport.allow_free = true;
	ret = ane_h16_session_init(&b->session, &b->transport, &ane_fw_session_ops, b,
				   ANE_FW_TIMEOUT_NS);
	if (ret)
		return ret;
	ret = apple_clpc_ane_get(&b->pdev->dev, &b->clpc);
	if (ret)
		return ret;
	b->session.gate_hold_ns = ANE_FW_GATE_HOLD_NS;
	ret = ane_h16_accel_register(&b->pdev->dev, &b->session);
	if (ret) {
		apple_clpc_ane_put(b->clpc);
		b->clpc = NULL;
		return ret;
	}
	ane_fw_debugfs(b);
	dev_info(&b->pdev->dev, "ANE program: session and ABI 3 node ready\n");
	return 0;
}

/*
 * The node without the firmware: macOS has no ANE running until its first
 * client, so the domains stay off and a sleep has nothing to stop. The first
 * request maps, boots and starts the engine (ane_fw_first_bringup()).
 */
static int ane_fw_register_parked(struct ane_fw_backend *b)
{
	int ret;

	ret = ane_fw_retain(b);
	if (ret)
		return ret;
	b->transport.soc = b->soc;
	ret = ane_h16_session_init_parked(&b->session, &b->transport, &ane_fw_session_ops, b,
					  ANE_FW_TIMEOUT_NS);
	if (ret)
		return ret;
	ret = apple_clpc_ane_get(&b->pdev->dev, &b->clpc);
	if (ret)
		return ret;
	b->session.gate_hold_ns = ANE_FW_GATE_HOLD_NS;
	ret = ane_h16_accel_register(&b->pdev->dev, &b->session);
	if (ret) {
		apple_clpc_ane_put(b->clpc);
		b->clpc = NULL;
		return ret;
	}
	ane_fw_debugfs(b);
	WRITE_ONCE(b->parked, true);
	return 0;
}

/* The first request: what the boot used to do, map, boot and start-up, then admit it. */
static int ane_fw_first_bringup(struct ane_fw_backend *b)
{
	static const char *const steps[] = { "map", "boot", "startup" };
	unsigned int flags, i;
	ssize_t ret = 0;

	flags = lock_system_sleep();
	if (READ_ONCE(b->stage) != ANE_STAGE_PROBED)
		goto out;
	for (i = 0; ret >= 0 && i < ARRAY_SIZE(steps); i++)
		ret = ane_fw_stage_action(&b->pdev->dev, steps[i], strlen(steps[i]));
	mutex_lock(&b->lock);
	if (ret >= 0) {
		b->transport.allow_free = true;
		b->stage = ANE_STAGE_PROGRAM;
		ret = 0;
	} else {
		b->session.quarantined = true;
	}
	WRITE_ONCE(b->parked, false);
	WRITE_ONCE(b->session.parked, false);
	ane_h16_session_sleep(&b->session, false);
	mutex_unlock(&b->lock);
	dev_info(&b->pdev->dev, "H17 ANE: first bring-up for a request: %zd\n", ret);
out:
	unlock_system_sleep(flags);
	return ret;
}

static ssize_t firmware_stage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	static const char * const names[] = {
		"unverified", "probe", "map", "boot", "startup", "program", "quarantined",
	};
	struct ane_fw_backend *b = ane_backend;

	return sysfs_emit(buf, "%s\n", names[READ_ONCE(b->stage)]);
}

static ssize_t ane_fw_stage_action(struct device *dev, const char *buf, size_t count)
{
	struct ane_fw_backend *b = ane_backend;
	enum ane_fw_stage next;
	int ret = 0;

	guard(mutex)(&b->lock);
	if (b->stage == ANE_STAGE_QUARANTINED)
		return -EIO;
	if (sysfs_streq(buf, "probe")) {
		if (b->retained)
			return -EBUSY;
		/* PMGR only: ANE_SYS may be off, which makes the engine unreadable. */
		ane_fw_snapshot_pmgr(b);
		if (!ane_fw_cold_pmgr_valid(&b->cold, ane_fw_raw_boot())) {
			dev_info(dev, "H17 ANE: cold acquisition unproven (loader or child power state); refusing\n");
			return -EPERM;
		}
		b->stage = ANE_STAGE_PROBED;
		dev_info(dev, "ANE probe: raw-boot evidence and cold child power verified; no writes\n");
		return count;
	}
	if (sysfs_streq(buf, "map"))
		next = ANE_STAGE_MAPPED;
	else if (sysfs_streq(buf, "boot"))
		next = ANE_STAGE_BOOTED;
	else if (sysfs_streq(buf, "startup"))
		next = ANE_STAGE_STARTUP;
	else if (sysfs_streq(buf, "program"))
		next = ANE_STAGE_PROGRAM;
	else
		return -EINVAL;
	if (next != b->stage + 1)
		return -EINVAL;
	if (next == ANE_STAGE_MAPPED) {
		u64 physical[2], size[2], dva[2];
		u8 data_pin[SHA256_DIGEST_SIZE];
		bool have_data_pin;

		ane_fw_snapshot_pmgr(b);
		if (!ane_fw_cold_pmgr_valid(&b->cold, ane_fw_raw_boot())) {
			dev_info(dev, "H17 ANE: cold acquisition unproven (loader or child power state); refusing\n");
			return -EPERM;
		}
		ret = ane_fw_preload_metadata(b, physical, size, dva, data_pin, &have_data_pin);
		if (ret) {
			dev_info(dev, "H17 ANE: loader preload reservation missing; refusing\n");
			return ret;
		}
		ret = ane_fw_retain(b);
		if (ret)
			return ret;
		ret = ane_fw_prepare(b);
	} else if (next == ANE_STAGE_BOOTED) {
		ret = ane_fw_boot(b);
	} else if (next == ANE_STAGE_STARTUP) {
		ret = ane_fw_startup(b);
	} else {
		ret = ane_fw_program_stage(b);
	}
	if (ret)
		return ane_fw_quarantine(b, ret);
	b->stage = next;
	return count;
}

static ssize_t firmware_stage_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int sleep_flags = lock_system_sleep();
	ssize_t ret = ane_fw_stage_action(dev, buf, count);

	unlock_system_sleep(sleep_flags);
	return ret;
}
static DEVICE_ATTR_RW(firmware_stage);

/*
 * Read the loader hand-off without touching the engine: the raw-boot marker,
 * the preload reservations (disabled placeholders until a loader fills them)
 * and the TEXT segment against the driver's pin.
 */
static bool ane_fw_handoff_present(struct platform_device *pdev,
				   const struct ane_fw_platform *soc, struct ane_fw_handoff *hp)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *chosen __free(device_node) = of_find_node_by_path("/chosen");
	struct ane_fw_handoff h = {};
	u8 digest[SHA256_DIGEST_SIZE] = {}, code[SHA256_DIGEST_SIZE] = {};
	bool mapped = false;
	unsigned int i;

	if (chosen)
		of_property_read_string(chosen, "apple,ane-cold-boot", &h.marker);
	h.preloaded = of_property_read_bool(np, "apple,firmware-preloaded");
	h.regions_ok = of_property_count_u32_elems(np, "memory-region") == 2 &&
		       of_property_count_u64_elems(np, "apple,firmware-iovas") == 2 &&
		       !of_property_read_u64_array(np, "apple,firmware-iovas", h.dva, 2);
	for (i = 0; h.regions_ok && i < 2; i++) {
		const char *name;

		h.regions_ok = !of_property_read_string_index(np, "memory-region-names", i,
							      &name) &&
			       !strcmp(name, i ? "firmware-data" : "firmware-text") &&
			       !ane_fw_preload_range(np, i, &h.phys[i], &h.size[i]);
	}
	/*
	 * Hash what the loader handed over even when its layout is not this
	 * SoC's, so that one boot of a new firmware or SoC reports both.
	 */
	if (h.regions_ok) {
		void *text = memremap(h.phys[0], h.size[0], MEMREMAP_WB);

		if (text) {
			bool pinned = ane_fw_text_pinned(soc, text, h.size[0], digest, code);
			size_t pre = soc->text_env[0].offset;

			/*
			 * On a code pin miss, the code before the first loader-filled
			 * section tells a loader write elsewhere from a different build.
			 */
			if (!pinned && soc->text_code_pin && soc->text_env[0].size &&
			    pre <= h.size[0]) {
				u8 head[SHA256_DIGEST_SIZE];

				sha256(text, pre, head);
				dev_info(&pdev->dev,
					 "H17 ANE: TEXT before 0x%zx sha256 %*phN\n",
					 pre, (int)sizeof(head), head);
			}
			memunmap(text);
			mapped = true;
			h.text_ok = ane_fw_preload_ranges_valid(soc, h.phys, h.size, h.dva) &&
				    pinned;
		}
	}
	/* Each boot shows whether the code pin holds, whatever matched. */
	if (mapped && soc->text_code_pin)
		dev_info(&pdev->dev, "H17 ANE: firmware %s, TEXT code sha256 %*phN (%s)\n",
			 soc->build_name ?: "?", (int)sizeof(code), code,
			 memcmp(code, soc->text_code_pin, sizeof(code)) ?
			 "not the code pin" : "code pin");
	*hp = h;
	if (ane_fw_cold_handoff_loader_ok(&h, soc))
		return true;
	dev_info(&pdev->dev, "H17 ANE: no loader hand-off (%s); leaving the engine alone\n",
		 !h.marker || strcmp(h.marker, ANE_FW_COLD_BOOT_EVIDENCE) ? "raw-boot marker" :
		 !h.preloaded || !h.regions_ok ? "preload reservations" :
		 !h.text_ok ? "firmware TEXT pin" : "preload layout");
	if (h.regions_ok && !h.text_ok) {
		dev_info(&pdev->dev, "H17 ANE: preloaded TEXT sha256 %*phN\n",
			 (int)sizeof(digest), digest);
		dev_info(&pdev->dev,
			 "H17 ANE: preload TEXT %#llx+%#llx @%#llx, DATA %#llx+%#llx @%#llx (%s)\n",
			 h.phys[0], h.size[0], h.dva[0], h.phys[1], h.size[1], h.dva[1],
			 ane_fw_preload_ranges_valid(soc, h.phys, h.size, h.dva) ?
			 "layout as expected" : "layout differs");
	}
	return false;
}

/*
 * Run probe..program once, as the stage file would, so the accel node exists
 * on every boot with a loader hand-off. A failure stops at that stage; after
 * acquisition it quarantines, as from the stage file.
 */
/* The staged bring-up at boot, for SoCs without macos_sleep. */
static void ane_fw_start_staged(struct ane_fw_backend *b)
{
	static const char *const steps[] = { "probe", "map", "boot", "startup", "program" };
	unsigned int i, flags;
	ssize_t ret;

	for (i = 0; i < ARRAY_SIZE(steps); i++) {
		flags = lock_system_sleep();
		ret = ane_fw_stage_action(&b->pdev->dev, steps[i], strlen(steps[i]));
		unlock_system_sleep(flags);
		if (ret < 0) {
			dev_err(&b->pdev->dev, "H17 ANE: boot bring-up stopped at %s, error %zd\n",
				steps[i], ret);
			ane_fw_bind_release(b);
			return;
		}
	}
	dev_info(&b->pdev->dev, "H17 ANE: up at boot, accel node ready\n");
}

static void ane_fw_start_work(struct work_struct *work)
{
	struct ane_fw_backend *b = container_of(work, struct ane_fw_backend, start);
	unsigned int flags;
	ssize_t ret;

	if (!b->soc->macos_sleep) {
		ane_fw_start_staged(b);
		return;
	}
	flags = lock_system_sleep();
	ret = ane_fw_stage_action(&b->pdev->dev, "probe", strlen("probe"));
	if (ret >= 0) {
		mutex_lock(&b->lock);
		ret = ane_fw_register_parked(b);
		mutex_unlock(&b->lock);
	}
	unlock_system_sleep(flags);
	/* Nothing holds ANE_SYS now: with no firmware, genpd turns it off, as macOS keeps it. */
	ane_fw_bind_release(b);
	if (ret < 0)
		dev_err(&b->pdev->dev, "H17 ANE: node registration stopped: %zd\n", ret);
	else
		dev_info(&b->pdev->dev, "H17 ANE: accel node ready; firmware starts on the first request\n");
}

static void ane_fw_observer_free(struct ane_fw_backend *b)
{
	unsigned int i;

	for (i = 0; i < ANE_FW_DARTS; i++)
		if (b->dart[i])
			iounmap(b->dart[i]);
	if (b->engine)
		iounmap(b->engine);
	if (b->pmgr)
		iounmap(b->pmgr);
	if (b->gate)
		iounmap(b->gate);
	apple_clpc_ane_put(b->clpc);
	put_device(&b->pdev->dev);
	kfree(b);
}

/* Record a mapped register window; the trace hook names accesses by PA. */
static int ane_fw_window(struct ane_fw_backend *b, unsigned int index, void __iomem *base,
			 struct device_node *np, unsigned int reg)
{
	struct resource res;

	if (!base)
		return -ENOMEM;
	if (of_address_to_resource(np, reg, &res))
		return -EINVAL;
	b->window[index].base = base;
	b->window[index].pa = res.start;
	b->window[index].size = resource_size(&res);
	return 0;
}

int ane_fw_staged_init(struct platform_device *pdev, const struct ane_fw_platform *soc)
{
	struct device_node *np = pdev->dev.of_node;
	struct ane_fw_handoff handoff;
	struct ane_fw_backend *b;
	unsigned int i;
	u32 dart_phandles[ANE_FW_DARTS];
	int ret = -EINVAL;

	if (ane_backend || !ane_fw_platform_staged(soc) || soc->dapf_count > ANE_FW_DAPF_MAX)
		return -EOPNOTSUPP;
	/* The firmware build the loader preloaded, if the SoC has several. */
	soc = ane_fw_platform_build(np, soc);
	/* No hand-off (an older loader, a warm boot): leave the engine alone. */
	if (!ane_fw_handoff_present(pdev, soc, &handoff))
		return -ENODEV;
	b = kzalloc_obj(*b);
	if (!b)
		return -ENOMEM;
	b->pdev = pdev;
	get_device(&pdev->dev);
	b->soc = soc;
	drm_mm_init(&b->dva_window, ANE_FW_ALLOC_DVA, ANE_FW_WINDOW);
	mutex_init(&b->window_lock);
	mutex_init(&b->lock);
	INIT_LIST_HEAD(&b->mappings);
	if (of_property_count_strings(np, "reg-names") != soc->reg_count ||
	    of_irq_count(np) != soc->interrupt_count ||
	    of_count_phandle_with_args(np, "power-domains", "#power-domain-cells") != 1)
		goto fail;
	for (i = 0; i < soc->reg_count; i++) {
		struct resource res;
		const char *name;

		if (of_address_to_resource(np, i, &res) ||
		    resource_size(&res) != soc->reg_sizes[i] ||
		    of_property_read_string_index(np, "reg-names", i, &name) ||
		    strcmp(name, soc->reg_names[i]))
			goto fail;
	}
	if (of_property_count_u64_elems(np, "apple,dapf-ranges") != soc->dapf_count * 2 ||
	    of_property_read_u64_array(np, "apple,dapf-ranges", b->ranges, soc->dapf_count * 2))
		goto fail;
	for (i = 0; i < soc->dapf_count; i++) {
		u64 start = b->ranges[i * 2], end = b->ranges[i * 2 + 1];

		if (start > end || round_down(start, SZ_16K) != round_down(end, SZ_16K))
			goto fail;
	}
	if (of_count_phandle_with_args(np, "iommus", "#iommu-cells") != ANE_FW_DARTS)
		goto fail;
	for (i = 0; i < ANE_FW_DARTS; i++) {
		struct of_phandle_args args;

		ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", i, &args);
		if (ret)
			goto fail;
		if (args.args_count != 1 || args.args[0] ||
		    !of_device_is_compatible(args.np, soc->dart_compatible)) {
			of_node_put(args.np);
			ret = -EINVAL;
			goto fail;
		}
		for (unsigned int j = 0; j < i; j++) {
			if (dart_phandles[j] == args.np->phandle) {
				of_node_put(args.np);
				ret = -EINVAL;
				goto fail;
			}
		}
		dart_phandles[i] = args.np->phandle;
		b->dart[i] = of_iomap(args.np, 0);
		ret = ane_fw_window(b, 3 + i, b->dart[i], args.np, 0);
		of_node_put(args.np);
		if (ret)
			goto fail;
	}
	b->engine = of_iomap(np, 0);
	b->pmgr = of_iomap(np, 1);
	b->gate = of_iomap(np, 2);
	ret = ane_fw_window(b, 0, b->engine, np, 0);
	if (!ret)
		ret = ane_fw_window(b, 1, b->pmgr, np, 1);
	if (!ret)
		ret = ane_fw_window(b, 2, b->gate, np, 2);
	if (ret)
		goto fail;
	/*
	 * The marker survives a kexec. Read PMGR (safe in any power state) and
	 * start only if the ANE CPU and sub-block are still off, never in a
	 * kdump kernel.
	 */
	ane_fw_snapshot_pmgr(b);
	handoff.pmgr = &b->cold;
	handoff.crash_kernel = is_kdump_kernel();
	if (!ane_fw_cold_handoff_valid(&handoff, soc)) {
		dev_info(&pdev->dev, "H17 ANE: engine not cold (%s); leaving it alone\n",
			 handoff.crash_kernel ? "kdump kernel" : "CPU or sub-block powered");
		ret = -ENODEV;
		goto fail;
	}
	ane_backend = b;
	ret = device_create_file(&pdev->dev, &dev_attr_firmware_stage);
	if (ret) {
		ane_backend = NULL;
		goto fail;
	}
	/* Once, outside b->lock (see ane_fw_retain()); it acts once retained. */
	b->pm_notifier.notifier_call = ane_fw_pm_event;
	ret = register_pm_notifier(&b->pm_notifier);
	if (ret) {
		device_remove_file(&pdev->dev, &dev_attr_firmware_stage);
		ane_backend = NULL;
		goto fail;
	}
	dev_info(&pdev->dev, "H17 ANE: loader hand-off verified; starting the engine\n");
	INIT_WORK(&b->start, ane_fw_start_work);
	return 0;
fail:
	ane_fw_observer_free(b);
	return ret;
}

/*
 * Bind the H17 platform device. Left unbound, it is a fw_devlink consumer
 * that never binds, so the PMGR power controller's sync_state never runs and
 * genpd keeps every boot-on domain under it on, ANE_SYS included, whatever
 * the backend puts (C4: ANE_SYS stayed on across system sleep). Bound, the
 * device carries ANE_SYS through its own runtime PM: platform_probe attached
 * it to its single domain. With a loader hand-off, the engine is powered as
 * the loader left it, so the bind holds ANE_SYS until the bring-up takes its
 * own reference (ane_fw_power()) or stops; without one the device is
 * runtime suspended and the domain is free to go off.
 */
int ane_fw_staged_bind(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ane_fw_backend *b = ane_backend;
	bool handoff = b && b->pdev == pdev;
	int ret;

	if (ane_fw_bound_dev)
		return -EBUSY;
	if (handoff) {
		ret = pm_runtime_set_active(dev);
		if (ret)
			return ret;
		pm_runtime_get_noresume(dev);
		b->bind_held = true;
	}
	ret = devm_pm_runtime_enable(dev);
	if (ret) {
		if (handoff) {
			b->bind_held = false;
			pm_runtime_put_noidle(dev);
			pm_runtime_set_suspended(dev);
		}
		return ret;
	}
	ane_fw_bound_dev = dev;
	if (handoff) {
		b->genpd_nb.notifier_call = ane_fw_genpd_event;
		b->genpd_nb_added = !dev_pm_genpd_add_notifier(dev, &b->genpd_nb);
		if (!b->genpd_nb_added)
			dev_info(dev, "H17 ANE: no genpd notifier; ANE_SYS transitions not recorded\n");
	}
	dev_info(dev, "H17 ANE: bound; %s\n", handoff ?
		 "ANE_SYS held until the bring-up owns it" : "no hand-off, engine left off");
	return 0;
}

bool ane_fw_staged_bound(struct device *dev)
{
	return dev && dev == ane_fw_bound_dev;
}

void ane_fw_staged_unbind(struct platform_device *pdev)
{
	struct ane_fw_backend *b = ane_backend;

	if (b && b->pdev == pdev) {
		ane_fw_bind_release(b);
		if (b->genpd_nb_added)
			dev_pm_genpd_remove_notifier(&pdev->dev);
		b->genpd_nb_added = false;
	}
	ane_fw_bound_dev = NULL;
}

/*
 * Device suspend: the notifier already cycled the engine. A backstop for
 * running firmware it did not reach (a sleep path that skipped notifiers).
 */
int ane_fw_staged_suspend(struct device *dev)
{
	struct ane_fw_backend *b = ane_backend;
	bool running = b && &b->pdev->dev == dev && READ_ONCE(b->retained) &&
		       READ_ONCE(b->powered) && !READ_ONCE(b->slept) &&
		       READ_ONCE(b->stage) == ANE_STAGE_PROGRAM;

	if (running)
		dev_warn_once(dev, ANE_FW_SLEEP_REFUSED);
	return ane_fw_sleep_backstop(running);
}

/*
 * Start the bring-up once the platform driver is registered: its probe of
 * this device has bound it (ane_fw_staged_bind()) by then and cannot race
 * the power sequence.
 */
void ane_fw_staged_start(void)
{
	if (ane_backend)
		schedule_work(&ane_backend->start);
}

bool ane_fw_staged_exit(void)
{
	struct ane_fw_backend *b = ane_backend;

	if (!b)
		return false;
	cancel_work_sync(&b->start);
	/* A retained backend pins the module. No unload or in-place retry. */
	if (WARN_ON(b->retained))
		return true;
	unregister_pm_notifier(&b->pm_notifier);
	device_remove_file(&b->pdev->dev, &dev_attr_firmware_stage);
	ane_backend = NULL;
	ane_fw_observer_free(b);
	return true;
}
