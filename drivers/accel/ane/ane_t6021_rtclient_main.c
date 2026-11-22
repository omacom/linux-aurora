// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * ane_t6021_rtclient.c — T6021 (H14 / M2) installed ANE driver.
 *
 * One module, two surfaces:
 *   - boot/transport: fenced Linux-context firmware start (fw_start=1
 *     default), shared boot/fwload helpers, then the legacy 13.5 ChMan
 *     transport for the three load-time commands (LOAD_PROGRAM 0x200,
 *     CREATE_PROCESS 0x202, PROCEDURE_CALL 0x204) driven through the
 *     ioctls in this file;
 *   - DRM accel: BO_INIT/BO_FREE plus PROG_LOAD/PROC_CREATE/EXEC per
 *     include/uapi/drm/ane_accel.h (ABI 2). Sections ride BOs the
 *     user supplies; one global mutex serializes every firmware
 *     command; completion = the legacy exchange's reply PLUS the
 *     firmware's finish event on the IO_T2H ring. EXEC also returns the
 *     fw's target-to-host slots (drained after every exchange).
 *
 * The compiled defaults are the configuration that boots the firmware:
 * a bare `insmod ane_t6021.ko` runs the whole sequence (fw_load=1
 * fw_start=1 legacy_only=1 legacy_query=1 scratch3_ack=1 poll_rx=1 ...).
 * Remaining parameters are overridable from sysfs for bisection only.
 *
 * Division of labor:
 *  - CPU start: from kernel context — the m1n1 RTBuddy path stays
 *    dormant (no MGMT session comes up) and the 13.5 (22G74) selene
 *    image brings up the legacy ChMan transport after DONE without
 *    RTKit. Boot is fenced in ane_t6021_boot.c (entry fold, RVBAR
 *    skip-or-fold, scratch, RUN, poll A/B, publish/wake).
 *  - genpd/pmgr: Runtime PM + the DT power-domains binding owns the
 *    raise; probe verifies ACTUAL on ane_cpu (pmgr window) before
 *    the first engine read (G1 gate). The pre-CPU engine table block
 *    is NOT written by default (preboot_table_mode = 2 = SKIP: the
 *    block has wedged the machine). The macOS-form mpm_off step is
 *    skipped: the boot-raised ane_sys_mpm stays on; the boot writes do
 *    not depend on it.
 *  - mailbox: ane_rtclient_validate_chman sees the 'IPC ' surface the
 *    fw published during boot; the three load-time commands travel
 *    the IO ring (channel 1) over ane_rtclient_legacy_exchange. Each
 *    command advances the cursor on the next 64-byte slot — resending
 *    to slot 0 wedges the ring.
 *  - non-posted MMIO (DT "nonposted-mmio" -> IORESOURCE_MEM_NONPOSTED)
 *    is mandatory throughout the ANE aperture; posted writel hangs the
 *    machine.
 *  - Firmware alias: the staged copy (own memory, default) or, with
 *    fw_alias_reserved=1, the iBoot-reserved SEG0/SEGi phys
 *    is aliased at the latched RVBAR entry; the subsequent kernel writes
 *    into the entry region translate through dart-ane0 instead of
 *    faulting.
 */

#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/util_macros.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include <drm/drm_accel.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_mm.h>
#include <crypto/sha2.h>

#include "ane_fw_platform.h"
#include "ane_fw_cold.h"
#include "ane_h16_protocol.h"
#include "ane_h16_pending.h"
#include "ane_h16_resources.h"
#include "ane_fw_protocol.h"
#include "ane_stats.h"
#include "ane_pm.h"
#include "ane_fw_ids.h"
#include "ane_t6021.h"
#include "ane_t6021_boot.h"
#include "ane_t6021_lifetime.h"
#include "ane_t6021_pm.h"

#include <drm/ane_accel.h>

/* Doorbell (IPI) block, engine + 0x1844000: set +0, pending +0x8000,
 * ack +0xc000 (an interrupt is handled by reading +0x184c000 and writing
 * +0x1850000, with no SoC difference).
 */
#define ANE_IPI_OFF			0x1844000

/* CPU_STATUS bits (m1n1 ASCRegs shape). On T8112 RUNNING is clear while
 * the firmware is live and IDLE (WFI) is set whenever it waits, so STOPPED
 * is the only stop predicate.
 */
#define ANE_ASC_CPU_STATUS_RUNNING	BIT(0)
#define ANE_ASC_CPU_STATUS_STOPPED	BIT(1)
#define ANE_ASC_CPU_STATUS_IDLE		BIT(5)

/* Legacy 13.5 POWER_DOWN: exactly the 8-byte header (any other length
 * asserts). The firmware stops its timers, requests its power state and
 * waits in WFI for good. It never posts a completion: its slot stays
 * firmware-owned. The dispatcher zeroes the completion field at +6 on
 * entry, which is the only sign the command was taken.
 */
#define ANE_T6021_CMD_POWER_DOWN	0x000b
#define ANE_T6021_POWER_DOWN_LEN	8
#define ANE_T6021_DISPATCH_MARK		0xa55a

/* Boot-time module parameters — defaults are the configuration that
 * boots the firmware, with fw_alias_reserved defaulting to own memory.
 * fw_load, fw_extra_ram and fw_alias_reserved live in
 * ane_t6021_fwload.c and boot_prevent_nap in ane_t6021_boot.c (single
 * registration each). Remaining knobs are overridable from sysfs for
 * bisection only.
 */
static bool fw_start = true;
module_param(fw_start, bool, 0444);
MODULE_PARM_DESC(fw_start,
		 "Fenced Linux-context firmware start: stage + alias, then the boot sequence; default on.");

static int fw_start_table_mode = 2;
module_param(fw_start_table_mode, int, 0444);
MODULE_PARM_DESC(fw_start_table_mode,
		 "Pre-CPU engine table block: 0 abort, 1 write, 2 skip (default).");

static bool fw_start_rtb_mode;
module_param(fw_start_rtb_mode, bool, 0444);
MODULE_PARM_DESC(fw_start_rtb_mode,
		 "RTBuddy/RTKit-app-endpoint select; default off (legacy ChMan transport).");

/* Drive RX by apple_rtkit_poll from a workqueue even though a recv
 * IRQ exists (default on: the raw recv line is not relied on, so the
 * worker is what services HELLO/EPMAP).
 */
static bool poll_rx = true;
module_param(poll_rx, bool, 0444);
MODULE_PARM_DESC(poll_rx,
		 "Drive RX by apple_rtkit_poll from a workqueue (default on).");

/* STARTEP every fw-announced app endpoint (>= 0x20) after a successful
 * RTKit handshake (default on).
 */
static bool start_app_eps = true;
module_param(start_app_eps, bool, 0444);
MODULE_PARM_DESC(start_app_eps,
		 "STARTEP every fw-announced app endpoint after the handshake (default on).");

static bool scratch3_ack = true;
module_param(scratch3_ack, bool, 0444);
MODULE_PARM_DESC(scratch3_ack,
		 "After DONE, write SCRATCH3 = 0x08042006 (host ack). Default on; 0 withholds.");

static bool legacy_only = true;
module_param(legacy_only, bool, 0444);
MODULE_PARM_DESC(legacy_only,
		 "Use the pinned 13.5 legacy ChMan transport (default on).");

static bool legacy_query = true;
module_param(legacy_query, bool, 0444);
MODULE_PARM_DESC(legacy_query,
		 "Service bounded startup allocations and CONFIG_GET; default on.");

/*
 * A job that hangs the engine leaves the ANE CPU busy: clearing RUN never
 * brings STOPPED. The vendor driver then powers the CPU off through its
 * power controller and cold-starts the firmware later; it never resets the
 * live CPU domain (doing so resets the SoC). 1: power the CPU and engine
 * domains off through genpd (children first, each PMGR transition run by
 * its state machine, the DARTs kept powered), read them all back off and
 * cold-boot. 0: keep the instance quarantined until reboot.
 */
unsigned int ane_t6021_hung_stop = 1;
module_param_named(hung_stop, ane_t6021_hung_stop, uint, 0644);
MODULE_PARM_DESC(hung_stop,
		 "After a hung job (T8112: ANE CPU left busy; H16: session quarantined): 1 power the ANE off through its power domains and cold-boot (default), 0 keep it quarantined until reboot.");

/* How long a busy CPU may take to stop by itself before it is powered off. */
unsigned int ane_t6021_hung_settle_ms = 100;
module_param_named(hung_settle_ms, ane_t6021_hung_settle_ms, uint, 0644);
MODULE_PARM_DESC(hung_settle_ms,
		 "Extra wait after a hung job before the power-off (T8112: for STOPPED; H16: after RUN clear) (ms, default 100, max 60000).");

/* 0 (default) skips RTKit in legacy mode, so the ANE mailbox never
 * starts: the 13.5 firmware sends no HELLO (-ETIME after
 * 1000 ms), and starting the mailbox enables AIC2 884, a level line
 * that then fires ~700,000 times/s (one CPU of hardirq time). A
 * firmware that speaks RTKit needs hello_wait_ms=1000.
 */
static unsigned int hello_wait_ms;
module_param(hello_wait_ms, uint, 0444);
MODULE_PARM_DESC(hello_wait_ms,
		 "RTKit HELLO wait in legacy mode; 0 (default) skips RTKit and leaves the mailbox stopped. A firmware that speaks RTKit needs 1000.");

/*
 * Producer-side stats (ane_stats sysfs, ane_timeline debugfs), read
 * once at probe. stats=0 keeps the hot path at its single predictable
 * branch (the NULL stats_slots check) and no files are created. Same
 * name and meaning as ane.ko's parameter.
 */
static bool stats = true;
module_param(stats, bool, 0444);
MODULE_PARM_DESC(stats,
		 "Expose ane_stats sysfs and ane_timeline debugfs (default 1; 0 = no files and the hot path skips the counters)");

/* Caps on the live firmware allocations (SHAREDMALLOC) and the command
 * buffer. A FREE from the firmware releases its slot and bytes.
 */
#define ANE_LEGACY_ALLOCS 8192
#define ANE_LEGACY_BYTES SZ_512M

struct ane_t6021_fd;

/* Populated only by an original-container loader and decoded process reply.
 * Those interfaces remain refused until their complete wire contract exists.
 * The firmware mutex protects this list and every pending UUID operation.
 */
struct ane_h16_process {
	struct list_head node;
	struct list_head owner_node;
	struct ane_h16_hwx_snapshot original;
	struct ane_h16_request_key key;
	struct ane_h16_resource resources[ANE_H16_SURFACES];
	struct ane_legacy_buffer mappings[ANE_H16_SURFACES];
	u32 resource_count;
};

/* Retain every old arena until a fresh firmware boot is validated. */
struct ane_rtclient_retired {
	struct list_head node;
	struct ane_legacy_buffer *buffers;
	u32 allocated;
	size_t bytes;
	struct ane_t6021_boot_allocs boot;
};

struct ane_rtclient_restart;

struct ane_rtclient {
	struct device *dev;
	void __iomem *engine;
	void __iomem *pmgr;
	void __iomem *set;
	bool recovering;
	bool engine_ps_valid;
	u32 engine_ps_off;
	bool registered;
	bool desired_perf;
	struct list_head retired;
	u32 retired_count;
	u32 retired_allocated;
	size_t retired_bytes;
	const struct ane_t602x_soc *soc;
	struct apple_rtkit *rtk;
	struct reset_control *cpu_rst;
	struct delayed_work poll_work;
	struct ane_t6021_pm sleep_pm;
	struct ane_rtclient_restart *sleep_restart;
	/* A system resume left the firmware restart to sleep_work. */
	bool sleep_deferred;
	/* The deferred restart must power-cycle a CPU that stayed on. */
	bool sleep_cycle;
	struct work_struct sleep_work;

	/* fw_start=1 view over the shared boot/fwload helpers. */
	struct ane_t6021 *fw;

	/* A CPU we released is (or may be) running: state HELD — surfaces,
	 * rings, IRQ, power links preserved; no unwind; reboot reclaims
	 * (wedged-pin rule).
	 */
	bool held;

	bool boot_done;

	/* ChMan descriptor table validated against the 'IPC ' surface. */
	bool chman_ok;
	struct ane_fw_channel channels[ANE_FW_CHANNELS];
	u8 channel_snapshot[ANE_FW_TABLE_SIZE];
	u64 channel_table_offset;
	struct ane_h16_pending_table pending;
	struct ane_h16_pending *waiting;
	struct list_head h16_processes;
	struct ane_legacy_buffer *legacy_buffers;
	u32 legacy_allocated;	/* high-water mark of the table */
	u32 legacy_live;	/* slots in use now */
	size_t legacy_bytes;
	u32 legacy_malloc_cursor;
	u32 legacy_cmd_cursor[ANE_T6021_CHMAN_COUNT];
	/* One reusable 16 KiB command buffer for every host command
	 * (CONFIG_GET + the three ioctls). Protocol-legal to reuse: an
	 * exchange returns 0 only after the IO slot flipped back to
	 * host-owned with a zero status (the completion
	 * predicate), so the firmware has fully consumed the previous
	 * command. The legacy_buffers table stays for fw MALLOC replies,
	 * which the firmware may reference forever (held until reboot).
	 */
	struct ane_legacy_buffer *cmd_buf;

	/*
	 * Producer-side stats (see ane_stats.h). stats=0 or a failed
	 * ring allocation leaves stats_slots NULL, which is the one
	 * hot-path gate: no files, no counter updates.
	 */
	struct ane_stats_counters stats_ctrs;
	struct ane_stats_ring stats_ring;
	struct ane_stats_ring_entry *stats_slots;
};

/* Per-open BO ownership (drm_file->driver_priv). Handles live in the
 * fd's list; closing the fd drops its handles (postclose). The
 * coherent buffers themselves live as long as something references
 * them: a user mapping, or the firmware (see ane_t6021_bo_release).
 */
struct ane_t6021_fd {
	struct list_head bos;
	struct list_head h16_processes;
	/* H14 programs this file loaded; under ane_t6021_fw_lock. */
	struct ane_t6021_fdprogs progs;
};

/* Per-file handle counter and every per-fd list against same-fd concurrent
 * ioctls. A BO whose IOVA reached the firmware (fw_ref) never goes back to
 * the kernel: a program section is held until a verified reset, an io BO is parked
 * for reuse (see ane_t6021_bo_release). Every other BO frees at its last
 * reference.
 *
 * Per-BO cap: matvec weights emitted by the H14 compiler are bounded by
 * `reduction * columns * 2`, which reaches 20 MiB at (K,N)=(2048,5120).
 * The same cap serves an attention constant at (K,N)=(4096,4096), 32 MiB,
 * and any H14 softmax/reduction with an 8 MiB table. Total BO bytes
 * across all fds are capped separately (bo_total_max_mb) by an atomic
 * counter, enforced at alloc and released when the memory is really
 * freed. Program sections stay held until a verified reset, and one large model's
 * programs alone can hold gigabytes.
 * A cap of 0 refuses every BO_INIT with -ENOSPC; there is no unlimited
 * value. Every BO also needs IOVA below 4 GiB (32-bit DMA mask), so
 * allocations fail with -ENOMEM near that bound whatever the cap.
 * The 16 KiB alignment check is unchanged: every DMA site assumes it.
 */
#define ANE_T6021_BO_MAX		SZ_1G
#define ANE_T6021_BO_HASH_CHUNK		SZ_1M

struct ane_t6021_bo {
	struct list_head node;
	struct ane_t6021_fd *owner;
	u32 handle;
	struct kref refcount;		/* handle + user mappings */
	void *cpu;			/* coherent mapping */
	dma_addr_t dma;
	size_t size;
	u64 fw_epoch;
	u32 prog_refs;			/* program sections using this BO */
	bool fw_ref;			/* the firmware received this IOVA */
	bool fw_program;		/* a loaded program keeps this IOVA */
	bool fw_io;			/* published as a CALL buffer */
	struct device *dev;
};

static DEFINE_MUTEX(ane_t6021_bo_lock);
static u32 ane_t6021_next_handle = 1;
static atomic64_t ane_t6021_bo_total_bytes = ATOMIC64_INIT(0);

static unsigned int bo_total_max_mb = 12288;
module_param(bo_total_max_mb, uint, 0444);
MODULE_PARM_DESC(bo_total_max_mb,
		 "Cap on the BO bytes held at one time, in MiB (default 12288)");

static int ane_t6021_bo_total_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%lld\n",
			  (long long)atomic64_read(&ane_t6021_bo_total_bytes));
}

static int ane_t6021_bo_total_set(const char *val,
				  const struct kernel_param *kp)
{
	return -EPERM;
}

static const struct kernel_param_ops ane_t6021_bo_total_ops = {
	.set = ane_t6021_bo_total_set,
	.get = ane_t6021_bo_total_get,
};
module_param_cb(bo_total_bytes, &ane_t6021_bo_total_ops, NULL, 0444);
MODULE_PARM_DESC(bo_total_bytes,
		 "Read only: the BO bytes counted against bo_total_max_mb now");

/* Mark the device quarantined: a timed-out command left the firmware
 * queue state unknown. The only safe next step is to refuse further
 * ioctls until a verified firmware restart reclaims the old surfaces.
 */
static atomic_t ane_t6021_quarantined = ATOMIC_INIT(0);

/* Parked io BOs: their last user is gone, the IOVA stays mapped and the
 * bytes stay counted. BO_INIT of the same page-aligned size takes one,
 * so held memory stays at the peak of concurrent io BOs instead of
 * growing with every process until the BO cap refuses BO_INIT.
 */
static LIST_HEAD(ane_t6021_bo_pool);
static LIST_HEAD(ane_t6021_bo_held);
static u64 ane_t6021_bo_epoch = 1;
static DEFINE_SPINLOCK(ane_t6021_bo_pool_lock);

/* Final put: the last handle or mapping is gone. The firmware must never
 * see a freed IOVA, so a current-generation fw_ref BO is retained: a program
 * section stays held, because a cached firmware program keeps reading
 * it; an io BO goes to the pool, unless a quarantined firmware may still
 * write it. Every other BO frees here, so the BO cap bounds only the
 * memory the firmware may touch.
 */
static void ane_t6021_bo_destroy(struct ane_t6021_bo *bo)
{
	atomic64_sub(PAGE_ALIGN(bo->size), &ane_t6021_bo_total_bytes);
	dma_free_coherent(bo->dev, bo->size, bo->cpu, bo->dma);
	kfree(bo);
}

static void ane_t6021_bo_release(struct kref *ref)
{
	struct ane_t6021_bo *bo = container_of(ref, struct ane_t6021_bo,
					       refcount);
	bool release;

	spin_lock(&ane_t6021_bo_pool_lock);
	release = !bo->fw_ref || bo->fw_epoch != ane_t6021_bo_epoch;
	if (!release) {
		if (bo->fw_program || atomic_read(&ane_t6021_quarantined))
			list_add_tail(&bo->node, &ane_t6021_bo_held);
		else
			list_add_tail(&bo->node, &ane_t6021_bo_pool);
	}
	spin_unlock(&ane_t6021_bo_pool_lock);
	if (release)
		ane_t6021_bo_destroy(bo);
}

/* After verified restart, closed BOs are reclaimable and live old BOs
 * become ordinary client-owned memory until published to firmware again.
 */
static void ane_t6021_bo_new_epoch(void)
{
	struct ane_t6021_bo *bo, *next;
	LIST_HEAD(reap);

	spin_lock(&ane_t6021_bo_pool_lock);
	ane_t6021_bo_epoch++;
	list_splice_init(&ane_t6021_bo_pool, &reap);
	list_splice_init(&ane_t6021_bo_held, &reap);
	spin_unlock(&ane_t6021_bo_pool_lock);
	list_for_each_entry_safe(bo, next, &reap, node) {
		list_del(&bo->node);
		ane_t6021_bo_destroy(bo);
	}
}

/* bo_lock and a live handle protect the BO while its IOVA is published. */
static void ane_t6021_bo_publish(struct ane_t6021_bo *bo, bool program)
{
	if (bo->fw_epoch != ane_t6021_bo_epoch) {
		bo->fw_program = false;
		bo->fw_io = false;
	}
	bo->fw_epoch = ane_t6021_bo_epoch;
	bo->fw_ref = true;
	if (program)
		bo->fw_program = true;
	else
		bo->fw_io = true;
}

/* A parked io BO of SIZE's page-aligned size, or NULL. */
static struct ane_t6021_bo *ane_t6021_bo_pool_take(size_t size)
{
	struct ane_t6021_bo *bo;

	if (atomic_read(&ane_t6021_quarantined))
		return NULL;
	spin_lock(&ane_t6021_bo_pool_lock);
	list_for_each_entry(bo, &ane_t6021_bo_pool, node) {
		if (PAGE_ALIGN(bo->size) == PAGE_ALIGN(size)) {
			list_del(&bo->node);
			spin_unlock(&ane_t6021_bo_pool_lock);
			return bo;
		}
	}
	spin_unlock(&ane_t6021_bo_pool_lock);
	return NULL;
}

/* A user mapping holds its BO's memory until it is torn down: open
 * (fork, mremap split) takes a reference, close drops it. A BO the
 * user freed while mapped stays allocated until the last mapping
 * goes away.
 */
static void ane_t6021_vm_open(struct vm_area_struct *vma)
{
	kref_get(&((struct ane_t6021_bo *)vma->vm_private_data)->refcount);
}

static void ane_t6021_vm_close(struct vm_area_struct *vma)
{
	kref_put(&((struct ane_t6021_bo *)vma->vm_private_data)->refcount,
		 ane_t6021_bo_release);
}

static const struct vm_operations_struct ane_t6021_vm_ops = {
	.open = ane_t6021_vm_open,
	.close = ane_t6021_vm_close,
};

/* Serializes every firmware command (LOAD/CREATE/CALL) so the
 * cursor-on-next-64-byte-slot rule and the PMGR/TM gate cannot race.
 */
static DEFINE_MUTEX(ane_t6021_fw_lock);

/* Forward declaration (defined below). */
static int ane_rtclient_recover(struct ane_rtclient *ane);
static int ane_rtclient_admit(struct ane_rtclient *ane);

static int ane_rtclient_legacy_exchange(struct ane_rtclient *ane,
					struct ane_legacy_buffer *command,
				       size_t length, u16 opcode,
				       unsigned int channel,
				       unsigned long deadline);

static bool ane_rtclient_drain_t2h(struct ane_rtclient *ane,
				   unsigned int channel);

/* ---- boot helpers (shared with boot.c / fwload.c) ---- */

/* Allocate a firmware-visible buffer in the first free table slot. Returns
 * the slot index or a negative errno. legacy_allocated is the high-water
 * mark; legacy_live and legacy_bytes count what is allocated now.
 */
static int ane_rtclient_legacy_alloc(struct ane_rtclient *ane, u64 size)
{
	struct ane_legacy_buffer *buffer;
	u32 cap = ANE_LEGACY_ALLOCS - min_t(u32, ane->retired_allocated,
					    ANE_LEGACY_ALLOCS);
	int slot;

	if (!size || !IS_ALIGNED(size, SZ_16K) || size > SZ_2M ||
	    size > ANE_LEGACY_BYTES - ane->legacy_bytes - ane->retired_bytes)
		return -E2BIG;
	slot = ane_t6021_legtab_pick(ane->legacy_buffers, ane->legacy_allocated,
				     cap);
	if (slot < 0)
		return -E2BIG;
	buffer = &ane->legacy_buffers[slot];
	buffer->cpu = dma_alloc_coherent(ane->dev, size, &buffer->dma,
					 GFP_KERNEL);
	if (!buffer->cpu)
		return -ENOMEM;
	if (!IS_ALIGNED(buffer->dma, SZ_16K) ||
	    (ane->fw && !ane_t6021_fw_alias_iova_ok(ane->fw, buffer->dma, size))) {
		dma_free_coherent(ane->dev, size, buffer->cpu, buffer->dma);
		memset(buffer, 0, sizeof(*buffer));
		return -ERANGE;
	}
	memset(buffer->cpu, 0, size);
	buffer->size = size;
	ane->legacy_bytes += size;
	ane->legacy_live++;
	if ((u32)slot == ane->legacy_allocated)
		ane->legacy_allocated++;
	return slot;
}

/* Validate the chman table the fw published in the 'IPC ' surface. */
static void ane_rtclient_validate_chman(struct ane_rtclient *ane)
{
	struct ane_t6021 *a = ane->fw;
	const struct ane_t6021_chman_desc *t;
	unsigned int i, bad;

	if (!a || !a->boot_ipc) {
		dev_info(ane->dev,
			 "chman: no host IPC surface — table not validated\n");
		return;
	}
	if (ane->soc->platform) {
		int ret;

		if (!ane_fw_contains(a->boot_ipc_iova, a->boot_ipc_size,
				     a->boot_scratch_result, ANE_FW_TABLE_SIZE)) {
			dev_err(ane->dev, "27.x channel table outside IPC\n");
			return;
		}
		ane->channel_table_offset = a->boot_scratch_result - a->boot_ipc_iova;
		dma_rmb();
		memcpy(ane->channel_snapshot,
		       a->boot_ipc + ane->channel_table_offset, ANE_FW_TABLE_SIZE);
		ret = ane_fw_channels_parse(ane->channel_snapshot, a->boot_ipc_iova,
					    a->boot_ipc_size, a->boot_scratch_result,
					    ane->channels);
		dma_rmb();
		if (ret || memcmp(ane->channel_snapshot,
				  a->boot_ipc + ane->channel_table_offset,
				  ANE_FW_TABLE_SIZE)) {
			dev_err(ane->dev, "27.x channel table rejected or changed\n");
			return;
		}
		ane->chman_ok = true;
		return;
	}
	if (a->boot_ipc_size < ANE_T6021_CHMAN_TOTAL) {
		dev_warn(ane->dev,
			 "chman: IPC surface %#llx bytes < fw layout %#x — table not validated\n",
			 a->boot_ipc_size, ANE_T6021_CHMAN_TOTAL);
		return;
	}

	dma_rmb();
	t = a->boot_ipc;
	bad = ane_t6021_chman_check(t, a->boot_ipc_iova);
	for (i = 0; i < ANE_T6021_CHMAN_COUNT; i++) {
		const struct ane_t6021_chman_desc *d = &t[i];
		const struct ane_t6021_chman_static *s = &ane_t6021_chman_layout[i];

		dev_dbg(ane->dev,
			"chman[%u]: name=\"%.*s\" type=%u bit=%u size=%#llx %s (static: %s/%u/%u/%#llx/ipc+%#x)\n",
			i, ANE_T6021_CHMAN_NAME_LEN, d->name, d->type, d->bit,
			d->size,
			(bad & BIT(i)) ? "MISMATCH" : "OK",
			s->name, s->type, s->bit, s->size, s->off);
	}

	ane->chman_ok = !bad;
	if (!bad)
		for (i = 0; i < ANE_FW_CHANNELS; i++) {
			const struct ane_t6021_chman_static *c = &ane_t6021_chman_layout[i];

			ane->channels[i] = (struct ane_fw_channel) {
				.offset = c->off, .slots = c->size,
				.type = c->type, .bit = c->bit,
			};
		}
	dev_info(ane->dev, "chman: table %s (mismatch mask %#x)\n",
		 bad ? "NOT VALIDATED" : "VALIDATED", bad);
}

static bool ane_rtclient_channels_unchanged(struct ane_rtclient *ane)
{
	struct ane_t6021 *a = ane->fw;

	if (!a || !ane->chman_ok)
		return false;
	dma_rmb();
	if (ane->soc->platform)
		return !memcmp(ane->channel_snapshot,
			       a->boot_ipc + ane->channel_table_offset,
			       ANE_FW_TABLE_SIZE);
	return !ane_t6021_chman_check(a->boot_ipc, a->boot_ipc_iova);
}

static void __iomem *ane_rtclient_doorbell(struct ane_rtclient *ane)
{
	return ane->engine + (ane->soc->platform ?
		ane->soc->platform->doorbell : ANE_IPI_OFF);
}

/*
 * Answer the SHAREDMALLOC slot at *cursor if the firmware posted a request
 * there (see ane_t6021_lifetime.h). Returns 1 when a request was answered
 * and the cursor moved, 0 when the slot holds none, or a negative errno for
 * a slot no request has the shape of or an allocation that failed.
 */
static int ane_rtclient_malloc_service(struct ane_rtclient *ane,
				       unsigned int *cursor)
{
	const struct ane_fw_channel *c = &ane->channels[5];
	u64 *slot = ane->fw->boot_ipc + c->offset + (size_t)*cursor * 64;
	void __iomem *ipi = ane_rtclient_doorbell(ane);
	u64 hdr = READ_ONCE(slot[0]), size, mode;
	struct ane_legacy_buffer *buf;
	int idx;

	if (hdr & 1)
		return 0;
	/* H16 needs its own validated allocation/reply provider. Never answer
	 * it with the legacy H14 allocation index.
	 */
	if (ane->soc->platform)
		return -EOPNOTSUPP;
	dma_rmb();
	size = READ_ONCE(slot[1]);
	mode = READ_ONCE(slot[2]);
	switch (ane_t6021_mring_classify(hdr, size, mode)) {
	case ANE_T6021_MRING_IDLE:
		return 0;
	case ANE_T6021_MRING_MALLOC:
		idx = ane_rtclient_legacy_alloc(ane, size);
		if (idx < 0) {
			dev_err(ane->dev,
				"fw malloc of %#llx bytes (mode %#llx) refused %d (live %u, %zu bytes)\n",
				size, mode, idx, ane->legacy_live, ane->legacy_bytes);
			return -ENOMEM;
		}
		buf = &ane->legacy_buffers[idx];
		WRITE_ONCE(slot[1], 0);
		WRITE_ONCE(slot[2], idx + 1);
		dma_wmb();
		WRITE_ONCE(slot[0], buf->dma | 1);
		dev_dbg(ane->dev,
			"fw malloc slot %u: %#llx bytes mode %#llx -> %pad index %d (live %u, %zu bytes)\n",
			*cursor, size, mode, &buf->dma, idx + 1, ane->legacy_live,
			ane->legacy_bytes);
		break;
	case ANE_T6021_MRING_FREE:
		/* The firmware dropped its mapping before it asked and waits
		 * for this answer holding its shared-memory lock. Answer first,
		 * then give the buffer back. The command buffer never matches.
		 */
		WRITE_ONCE(slot[1], 0);
		WRITE_ONCE(slot[2], 0);
		dma_wmb();
		WRITE_ONCE(slot[0], 1);
		idx = ane_t6021_legtab_find(ane->legacy_buffers,
					    ane->legacy_allocated, hdr,
					    ane->cmd_buf);
		if (idx >= 0) {
			buf = &ane->legacy_buffers[idx];
			dma_free_coherent(ane->dev, buf->size, buf->cpu, buf->dma);
			ane->legacy_bytes -= buf->size;
			ane->legacy_live--;
			memset(buf, 0, sizeof(*buf));
		}
		if (idx >= 0)
			dev_dbg(ane->dev,
				"fw free slot %u: iova %#llx released (live %u, %zu bytes)\n",
				*cursor, hdr, ane->legacy_live, ane->legacy_bytes);
		else
			dev_info_ratelimited(ane->dev,
					     "fw free slot %u: iova %#llx is not a firmware allocation; answered, nothing released\n",
					     *cursor, hdr);
		break;
	default:
		dev_err(ane->dev,
			"fw malloc ring slot %u is not a request: %#llx %#llx %#llx\n",
			*cursor, hdr, size, mode);
		return -EPROTO;
	}
	dma_wmb();
	writel(BIT(c->bit), ipi);
	*cursor = (*cursor + 1) % c->slots;
	return 1;
}

/* Answer every request already posted, at most one ring's worth. */
static int ane_rtclient_malloc_drain(struct ane_rtclient *ane)
{
	unsigned int n;
	int ret = 0;

	for (n = 0; n < ane->channels[5].slots; n++) {
		ret = ane_rtclient_malloc_service(ane, &ane->legacy_malloc_cursor);
		if (ret <= 0)
			break;
	}
	return ret < 0 ? ret : 0;
}

/*
 * A FREE caused by a command can land after its acknowledgment. Keep
 * answering until the ring was quiet for quiet_ms, at most 64 answers.
 */
static int ane_rtclient_malloc_settle(struct ane_rtclient *ane,
				      unsigned int quiet_ms)
{
	unsigned long until = jiffies + msecs_to_jiffies(quiet_ms);
	unsigned int answered = 0;
	int ret;

	while (time_before(jiffies, until)) {
		ret = ane_rtclient_malloc_service(ane, &ane->legacy_malloc_cursor);
		if (ret < 0)
			return ret;
		if (ret) {
			if (++answered > 64)
				return -EBUSY;
			until = jiffies + msecs_to_jiffies(quiet_ms);
		} else {
			usleep_range(200, 400);
		}
	}
	return 0;
}

/* What the rings held when a command timed out, for the log. */
static void ane_rtclient_dump_rings(struct ane_rtclient *ane)
{
	static const unsigned int chans[] = { 4, 5, 6 };
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(chans); i++) {
		const struct ane_fw_channel *c = &ane->channels[chans[i]];
		unsigned int cur = chans[i] == 5 ? ane->legacy_malloc_cursor :
				   ane->legacy_cmd_cursor[chans[i]];
		unsigned int n = min_t(unsigned int, c->slots, 8);

		if (!c->slots)
			continue;
		/* The whole allocation ring; T2H rings from the host cursor. */
		for (j = 0; j < n; j++) {
			unsigned int k = chans[i] == 5 ? j : (cur + j) % c->slots;
			u64 *slot = ane->fw->boot_ipc + c->offset + (size_t)k * 64;

			dev_info(ane->dev,
				 "ring ch%u slot %u: %016llx %016llx %016llx%s\n",
				 chans[i], k, READ_ONCE(slot[0]),
				 READ_ONCE(slot[1]), READ_ONCE(slot[2]),
				 k == cur ? " <- host cursor" : "");
		}
	}
	dev_info(ane->dev, "fw allocations: live %u, high water %u, %zu bytes\n",
		 ane->legacy_live, ane->legacy_allocated, ane->legacy_bytes);
}

/* Legacy ChMan exchange (post-DONE 13.5 transport). Resends to the same
 * 64-byte IO slot would wedge the ring; cursor advances after each
 * completed command (the fw treats bit0 as host-owned).
 */
static int ane_rtclient_legacy_exchange(struct ane_rtclient *ane,
					struct ane_legacy_buffer *command,
				       size_t length, u16 opcode,
				       unsigned int channel,
				       unsigned long deadline)
{
	struct ane_t6021 *a = ane->fw;
	u64 *io, header;
	void __iomem *ipi = ane_rtclient_doorbell(ane);
	int result = -ETIMEDOUT;

	if (!ane->held || !ane_rtclient_channels_unchanged(ane))
		return -EPROTO;
	if (length < 8 || length > command->size ||
	    channel >= ANE_T6021_CHMAN_COUNT) {
		result = -EINVAL;
		goto out;
	}
	io = a->boot_ipc + ane->channels[channel].offset +
	     (size_t)ane->legacy_cmd_cursor[channel] * 64;

	if (!(READ_ONCE(io[0]) & 1)) {
		result = -EBUSY;
		goto out;
	}
	if (time_after_eq(jiffies, deadline))
		goto out;
	/* A FREE posted since the last command (an idle pool shrink) is
	 * answered before this command goes out.
	 */
	if (!ane->soc->platform) {
		result = ane_rtclient_malloc_drain(ane);
		if (result)
			goto out;
		result = -ETIMEDOUT;
	}
	if (ane->soc->platform) {
		result = ane_h16_command_prepare(command->cpu, length, opcode,
						 ane->channels[channel].index);
		if (result)
			goto out;
	} else {
		ane_fw_command_header(command->cpu, opcode, false);
	}
	if (ane->soc->platform)
		memset((u8 *)command->cpu + length, 0xa5, command->size - length);
	WRITE_ONCE(io[1], length);
	WRITE_ONCE(io[2], length);
	dma_wmb();
	WRITE_ONCE(io[0], command->dma);
	dma_wmb();
	writel(BIT(ane->channels[channel].bit), ipi);
	while (time_before(jiffies, deadline)) {
		u32 pending;

		if (atomic_read(&ane_t6021_quarantined)) {
			result = -EIO;
			goto out;
		}
		if (!ane_rtclient_channels_unchanged(ane)) {
			result = -EPROTO;
			goto out;
		}
		if (ane->soc->platform && opcode == ANE_H16_EXECUTE) {
			ane_rtclient_drain_t2h(ane, 6);
			if (ane->pending.quarantined) {
				result = ane->pending.error ?: -EIO;
				goto out;
			}
		}
		pending = readl(ipi + 0x8000);

		if (pending) {
			writel(pending, ipi + 0xc000);
			/* Complete the IPI acknowledgment before the
			 * next reads of the command and malloc rings.
			 */
			mb();
		}
		header = READ_ONCE(io[0]);
		if (!(header & 1)) {
			/* malloc-ring service: the fw may ask for an
			 * allocation, or give one back, while we wait.
			 */
			int served = ane_rtclient_malloc_service(ane,
								 &ane->legacy_malloc_cursor);

			if (served < 0) {
				result = served;
				goto out;
			}
		}
		/* Allocation service cannot extend the command deadline. */
		if (time_after_eq(jiffies, deadline))
			goto out;
		header = READ_ONCE(io[0]);
		if (header & 1) {
			dma_rmb();
			result = ane_fw_reply_status(header, READ_ONCE(io[1]),
						     READ_ONCE(io[2]), command->dma,
						     length, opcode, command->cpu,
						     !!ane->soc->platform);
			/* Corruption does not retire the unknown slot. A refusal
			 * retires ownership but remains a nonzero result.
			 */
			if (!ane_rtclient_channels_unchanged(ane) ||
			    (ane->soc->platform &&
			     memchr_inv((u8 *)command->cpu + length, 0xa5,
					command->size - length)))
				result = -EPROTO;
			if (result == -EREMOTEIO)
				dev_info(ane->dev, "firmware refused op=%#x ack=%#x; no retry\n",
					 opcode, get_unaligned_le16((u8 *)command->cpu + 6));
			if (!result || result == -EREMOTEIO)
				ane->legacy_cmd_cursor[channel] =
					(ane->legacy_cmd_cursor[channel] + 1) %
					ane->channels[channel].slots;
			goto out;
		}
		/* Fast poll: check the rings every 50 us instead of
		 * sleeping 1-2 ms.
		 */
		udelay(50);
	}
	dev_info(ane->dev, "LEGACY timeout ch=%u op=%#x io=%016llx\n",
		 channel, opcode, READ_ONCE(io[0]));
	ane_rtclient_dump_rings(ane);
out:
	return result;
}

/*
 * Extra sleep after a CALL's finish event. 0 by default: the finish event
 * already marks the output in DRAM (ane_rtclient_call_wait). The 1 ms
 * default it replaces dated from a wait on dispatch signals; set it only
 * to test a suspected late output write.
 */
static unsigned int call_settle_us;
module_param(call_settle_us, uint, 0644);
MODULE_PARM_DESC(call_settle_us,
		 "Microseconds to sleep after a CALL's finish event (default 0: the event marks the output in DRAM)");

/*
 * The finish-event wait polls the coherent IO_T2H ring, so the poll cadence is
 * the whole detection lag: a uniform 50..100 us sleep adds 0..100 us of pure
 * wait to every CALL. call_poll_us is how long the head of the wait sleeps
 * only 1..2 us between looks (the real period is the hrtimer wake latency)
 * before it falls back to the 50..100 us sleep, which keeps long programs (the
 * Parakeet encoder, about 254 ms) from spending the wait on hrtimer wakeups.
 * 0 restores the uniform 50..100 us cadence. A very large value makes the
 * whole wait 1..2 us sleeps. Writable at run time (0644), so both cadences can
 * be compared in one boot.
 */
static unsigned int call_poll_us = 1000;
module_param(call_poll_us, uint, 0644);
MODULE_PARM_DESC(call_poll_us,
		 "Microseconds at the head of the CALL finish-event wait that sleep only 1..2 us between looks, then 50..100 us (default 1000; 0 = uniform 50..100 us; a very large value makes the whole wait 1..2 us sleeps)");

/* trace_td: a read-only timeline of each CALL for performance work. Off
 * by default; switch it at runtime with
 * /sys/module/ane_t6021/parameters/trace_td (no device needed). Off, the
 * CALL path is unchanged. On, the completion wait polls every 20-40 us
 * and also reads the last-committed-TD word. It reads that word only
 * while the seven ANE pmgr PS words read 0x3ff. No register is written.
 *
 * - The TD word: engine + ane_t602x_soc.trace_td_off; on T602x TM
 *   0x285c00000 + TD window 0x20400 + 0x58. Its layout: the call's nid
 *   in bits 23:16 (+1 per call), the index of the last task taken in
 *   bits 15:0. Not known on T8112, so trace_td records nothing there.
 * - The seven ANE power-state words at ane_t602x_soc.pmu_pa + ps_off
 *   (T602x: DT power-domains ane_sys_mpm 0x4000 .. ane_set4 0x4030 of
 *   the pmgr at 0x28e080000). The guard: PS words 0x3ff before any TM
 *   read, since a TM read while the compute domains are off hangs the
 *   SoC.
 *
 * Each record has a ktime_get_ns() stamp: CALL before the exchange, ACK
 * when the firmware acked it, TD for each new TD word value (word = the
 * value), EVENT for each IO_T2H event of the CALL (word = its state),
 * GATE when a PS word reads other than 0x3ff (word = its offset), DONE
 * when the wait ends (word = the TD samples taken). The buffer is
 * allocated at the first switch-on and kept until unload; each switch-on
 * empties it, and records past its end are counted in `dropped`. Read
 * it, while no CALL runs, from debugfs ane_t6021/trace_td (0400). All
 * trace state is protected by ane_t6021_fw_lock.
 */
#define ANE_TRACE_MAGIC		0x31445441	/* "ATD1" */
#define ANE_TRACE_RECS		BIT(18)
#define ANE_PMGR_PS_LAST_OFF	0x30

enum {
	ANE_TR_CALL = 1,
	ANE_TR_ACK,
	ANE_TR_TD,
	ANE_TR_EVENT,
	ANE_TR_GATE,
	ANE_TR_DONE,
};

struct ane_t6021_trace_rec {
	u64 t_ns;
	u32 word;
	u16 kind;
	u16 call;
};

struct ane_t6021_trace {
	u32 magic;
	u32 rec_size;
	u32 capacity;
	u32 n;
	u32 dropped;
	u32 calls;
	u64 reserved;
	struct ane_t6021_trace_rec r[];
};

static bool trace_td;
static bool ane_t6021_tracing;	/* the running CALL is traced */
static struct ane_t6021_trace *ane_t6021_trace;
static struct debugfs_blob_wrapper ane_t6021_trace_blob;
static struct dentry *ane_t6021_trace_dir;

static void ane_t6021_trace_add(u16 kind, u32 word)
{
	struct ane_t6021_trace *t = ane_t6021_trace;

	if (t->n == t->capacity) {
		t->dropped++;
		return;
	}
	t->r[t->n++] = (struct ane_t6021_trace_rec){
		.t_ns = ktime_get_ns(), .word = word, .kind = kind,
		.call = t->calls,
	};
}

static int ane_t6021_trace_set(const char *val, const struct kernel_param *kp)
{
	size_t size = struct_size_t(struct ane_t6021_trace, r, ANE_TRACE_RECS);
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	mutex_lock(&ane_t6021_fw_lock);
	if (on && !ane_t6021_trace) {
		ane_t6021_trace = vzalloc(size);
		if (!ane_t6021_trace) {
			ret = -ENOMEM;
			goto out;
		}
		ane_t6021_trace->magic = ANE_TRACE_MAGIC;
		ane_t6021_trace->rec_size = sizeof(struct ane_t6021_trace_rec);
		ane_t6021_trace->capacity = ANE_TRACE_RECS;
		ane_t6021_trace_blob.data = ane_t6021_trace;
		ane_t6021_trace_blob.size = size;
		ane_t6021_trace_dir = debugfs_create_dir("ane_t6021", NULL);
		debugfs_create_blob("trace_td", 0400, ane_t6021_trace_dir,
				    &ane_t6021_trace_blob);
	}
	if (on && !trace_td) {
		ane_t6021_trace->n = 0;
		ane_t6021_trace->dropped = 0;
		ane_t6021_trace->calls = 0;
	}
	trace_td = on;
out:
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

static const struct kernel_param_ops ane_t6021_trace_ops = {
	.set = ane_t6021_trace_set,
	.get = param_get_bool,
};
module_param_cb(trace_td, &ane_t6021_trace_ops, &trace_td, 0644);
MODULE_PARM_DESC(trace_td,
		 "Record a read-only per-CALL TD-word timeline in debugfs ane_t6021/trace_td (default 0; T602x only)");

static void ane_t6021_trace_free(void)
{
	debugfs_remove_recursive(ane_t6021_trace_dir);
	vfree(ane_t6021_trace);
}

/* The CALL cookie (CALL +0x20); the firmware returns it in the call's
 * IO_T2H events.
 */
#define ANE_CALL_COOKIE		0xADD0

/* IO_T2H event of a PROCEDURE_CALL, 0x28 bytes: u32 sequence, u32 0x300,
 * u64 cookie, u32 program id, u32 process id, u32 0, u32 state. The
 * firmware posts two per call: state 0 about 0.2-0.5 ms after the ack
 * whatever the program length (a long program can take its last task
 * hundreds of ms later), and state 1 when the procedure has finished.
 */
#define ANE_T2H_CALL_COOKIE_OFF		0x08
#define ANE_T2H_CALL_STATE_OFF		0x1c
#define ANE_T2H_CALL_FINISHED		1

/* The CPU address of LEN bytes at the firmware IOVA, or NULL. The firmware
 * places T2H payloads in memory the host gave it: a SHAREDMALLOC buffer or
 * the 'IPC ' surface.
 */
static const void *ane_rtclient_fw_cpu(struct ane_rtclient *ane, u64 iova,
				       size_t len)
{
	const void *found = NULL;
	u32 i;

	for (i = 0; i < ane->legacy_allocated; i++) {
		const struct ane_legacy_buffer *b = &ane->legacy_buffers[i];

		if (ane_fw_contains(b->dma, b->size, iova, len)) {
			if (!ane->soc->platform)
				return b->cpu + (iova - b->dma);
			if (found)
				return NULL;
			found = b->cpu + (iova - b->dma);
		}
	}
	if (ane_fw_contains(ane->fw->boot_ipc_iova, ane->fw->boot_ipc_size,
			    iova, len))
		return found ? NULL : ane->fw->boot_ipc + (iova - ane->fw->boot_ipc_iova);
	return found;
}

/* Return every firmware-owned slot on a target-to-host ring to the fw:
 * the fw publishes by writing the DMA address with bit0 clear; the host
 * returns the slot by setting bit0 and ringing the channel's doorbell
 * bit, as the allocation ring does. The ioctls drain channels 4/6 after
 * every completed exchange. Returns true when
 * an IO_T2H slot held the finish event of a CALL.
 */
static bool ane_rtclient_drain_t2h(struct ane_rtclient *ane,
				   unsigned int channel)
{
	const struct ane_fw_channel *c = &ane->channels[channel];
	unsigned int n = 0, slot_i = ane->legacy_cmd_cursor[channel];
	void __iomem *ipi = ane_rtclient_doorbell(ane);
	bool finished = false;

	if (!ane_rtclient_channels_unchanged(ane)) {
		atomic_set(&ane_t6021_quarantined, 1);
		if (ane->soc->platform) {
			ane->pending.error = -EPROTO;
			ane->pending.quarantined = true;
		}
		return false;
	}
	if (c->type != 1)
		return false;
	while (n < c->slots) {
		u64 *slot = ane->fw->boot_ipc + c->offset + (size_t)slot_i * 64;
		u64 hdr = READ_ONCE(slot[0]), len;

		if (hdr & 1)
			break;
		dma_rmb();
		len = READ_ONCE(slot[1]);
		dev_dbg(ane->dev, "T2H ch=%u slot=%u len=%#llx\n",
			channel, slot_i, len);
		if (ane->soc->platform) {
			u8 snapshot[0x60];
			const void *ev;
			int status;

			/* Other channels have different message types; never apply
			 * the event predicate to logs or allocation traffic.
			 */
			if (c->index != 6 || (len != sizeof(snapshot) && len != 0xc)) {
				atomic_set(&ane_t6021_quarantined, 1);
				ane->pending.error = -EPROTO;
				ane->pending.quarantined = true;
				return false;
			}
			ev = ane_rtclient_fw_cpu(ane, hdr, len);
			if (!ev) {
				atomic_set(&ane_t6021_quarantined, 1);
				ane->pending.error = -EPROTO;
				ane->pending.quarantined = true;
				return false;
			}
			memcpy(snapshot, ev, len);
			status = ane_h16_pending_dispatch(&ane->pending, snapshot,
							  len, c->index, jiffies);
			if (ane->pending.quarantined) {
				atomic_set(&ane_t6021_quarantined, 1);
				return false;
			}
			if (status == ANE_H16_EVENT_COMPLETE)
				finished = true;
		} else if (channel == 6 && len >= ANE_T2H_CALL_STATE_OFF + 4) {
			const u8 *ev = ane_rtclient_fw_cpu(ane, hdr,
							   ANE_T2H_CALL_STATE_OFF + 4);

			if (ev &&
			    get_unaligned_le64(ev + ANE_T2H_CALL_COOKIE_OFF) ==
			    ANE_CALL_COOKIE) {
				u32 state = get_unaligned_le32(ev +
							       ANE_T2H_CALL_STATE_OFF);

				if (ane_t6021_tracing)
					ane_t6021_trace_add(ANE_TR_EVENT, state);
				if (state == ANE_T2H_CALL_FINISHED)
					finished = true;
			}
		}
		n++;
		WRITE_ONCE(slot[0], hdr | 1);
		dma_wmb();
		writel(BIT(c->bit), ipi);
		slot_i = (slot_i + 1) % c->slots;
		ane->legacy_cmd_cursor[channel] = slot_i;
	}
	return finished;
}

/* The completion wait with trace_td on: the same finish-event test with a
 * 20-40 us poll, and one TD-word sample per poll under the PS-word guard
 * (see trace_td).
 */
static int ane_rtclient_call_wait_traced(struct ane_rtclient *ane,
					 unsigned long deadline)
{
	void __iomem *td = ane->engine + ane->soc->trace_td_off;
	void __iomem *ps = ioremap_np(ane->soc->pmu_pa + ane->soc->ps_off,
				      ANE_PMGR_PS_LAST_OFF + 4);
	unsigned int i, gate = ANE_PMGR_PS_LAST_OFF + 8;
	u32 last = U32_MAX, samples = 0;
	int ret = -ETIMEDOUT;

	do {
		if (ane_rtclient_drain_t2h(ane, 6)) {
			ret = 0;
			break;
		}
		if (ps) {
			for (i = 0; i <= ANE_PMGR_PS_LAST_OFF; i += 8)
				if ((readl(ps + i) & 0x3ff) != 0x3ff)
					break;
			if (i > ANE_PMGR_PS_LAST_OFF) {
				u32 w = readl(td);

				samples++;
				if (w != last)
					ane_t6021_trace_add(ANE_TR_TD, w);
				last = w;
			} else if (i != gate) {
				ane_t6021_trace_add(ANE_TR_GATE, i);
			}
			gate = i;
		}
		usleep_range(20, 40);
	} while (time_before(jiffies, deadline));
	if (ret && ane_rtclient_drain_t2h(ane, 6))
		ret = 0;
	ane_t6021_trace_add(ANE_TR_DONE, samples);
	if (ps)
		iounmap(ps);
	return ret;
}

/*
 * Wait for the PROCEDURE_CALL finish event on IO_T2H channel 6.
 * Acknowledgment, TQ status and the last-committed TD word indicate
 * dispatch rather than execution completion. The finish event marks
 * completion and output visibility, so no extra settle delay is needed.
 * Return zero on the event, or -ETIMEDOUT.
 */
static int ane_rtclient_call_wait(struct ane_rtclient *ane,
				  unsigned long deadline)
{
	unsigned int poll_us = ane->soc->platform ? 0 : READ_ONCE(call_poll_us);
	ktime_t tight_end = ktime_add_ns(ktime_get(),
					 (u64)poll_us * NSEC_PER_USEC);

	if (ane_t6021_tracing)
		return ane_rtclient_call_wait_traced(ane, deadline);
	if (time_after_eq(jiffies, deadline))
		return -ETIMEDOUT;

	do {
		if (atomic_read(&ane_t6021_quarantined))
			return -EIO;
		if (ane->soc->platform) {
			ane_rtclient_drain_t2h(ane, 6);
			if (atomic_read(&ane_t6021_quarantined))
				return ane->pending.error ?: -EIO;
			if (ane->waiting && ane->waiting->terminal)
				return ane->waiting->status;
		} else if (ane_rtclient_drain_t2h(ane, 6)) {
			return 0;
		} else if (ane_rtclient_malloc_drain(ane)) {
			/* A program waiting on an unanswered request never
			 * finishes; a malformed ring ends the call now.
			 */
			return -EPROTO;
		}
		if (poll_us && ktime_before(ktime_get(), tight_end))
			usleep_range(1, 2);
		else
			usleep_range(50, 100);
	} while (time_before(jiffies, deadline));
	if (atomic_read(&ane_t6021_quarantined))
		return -EIO;
	if (ane->soc->platform)
		return -ETIMEDOUT;
	return ane_rtclient_drain_t2h(ane, 6) ? 0 : -ETIMEDOUT;
}

static int ane_rtclient_command(struct ane_rtclient *ane,
				struct ane_legacy_buffer *command,
						     size_t length, u16 opcode,
						     unsigned int channel,
						     unsigned int timeout_ms)
{
	unsigned long deadline;
	int ret;
	u64 stats_ticket = 0;
	u64 stats_submit_ns = 0;
	bool execute = ane->soc->platform ? opcode == ANE_H16_EXECUTE :
					   opcode == CSNE_CMD_PROCEDURE_CALL;

	ane_t6021_tracing = opcode == CSNE_CMD_PROCEDURE_CALL && trace_td &&
			    ane->soc->trace_td_off;
	if (ane_t6021_tracing) {
		ane_t6021_trace->calls++;
		ane_t6021_trace_add(ANE_TR_CALL, 0);
	}
	/* Producer-side stats hot path (ane_stats.h): record submit at
	 * command enqueue, completion after the call returns, one
	 * begin/complete pair per engine submission. Only
	 * CSNE_CMD_PROCEDURE_CALL is engine work; the control-plane
	 * exchanges that ride this function (LOAD_PROGRAM,
	 * CREATE_PROCESS, CH_PROPERTY_WRITE, CONFIG_GET) are not
	 * counted, so jobs matches the engine calls the workload made.
	 * The T6021 path can run concurrently (MBI per-channel rings),
	 * so overlapping calls share a busy period and busy_ns is the
	 * union of the submit-to-completion windows. tmst is 0 (no
	 * host TM on T6021; documented in the file header line).
	 * tasks = 1 (one call per submission); rc = ret. The gate is the
	 * ring, as on ane.ko: stats=0 or a failed probe allocation leaves
	 * stats_slots NULL, and the hooks must not run on a NULL ring.
	 */
	bool stats_call = ane->stats_slots && execute;

	if (ane->soc->platform && execute &&
	    (!ane->waiting || !ane->waiting->active || ane->pending.quarantined))
		return -EIO;

	/* One bounded budget covers both ACK and finish, including H14. */
	timeout_ms = min(timeout_ms, 5000U);
	deadline = ane->soc->platform && ane->waiting ?
		ane->waiting->deadline : jiffies + msecs_to_jiffies(timeout_ms);

	if (stats_call) {
		stats_submit_ns = ktime_get_ns();
		stats_ticket = ane_stats_begin(&ane->stats_ctrs,
					       &ane->stats_ring,
					       stats_submit_ns, 1);
	}
	ret = ane_rtclient_legacy_exchange(ane, command, length, opcode,
					   channel, deadline);
	if (ret) {
		ane_t6021_tracing = false;
		if (stats_call)
			ane_stats_complete(&ane->stats_ctrs,
					   &ane->stats_ring, stats_ticket,
					   ktime_get_ns(), (u32)ret, 0);
		dev_info(ane->dev, "EXCH op=%#x failed %d (fw allocs %u, %zu bytes)\n",
			 opcode, ret, ane->legacy_allocated, ane->legacy_bytes);
		atomic_set(&ane_t6021_quarantined, 1);
		if (!ane->recovering)
			ane_rtclient_recover(ane);
		return ret;
	}
	if (execute) {
		if (ane_t6021_tracing)
			ane_t6021_trace_add(ANE_TR_ACK, 0);
		ret = ane_rtclient_call_wait(ane, deadline);
		ane_t6021_tracing = false;
		if (!ret)
			dma_rmb();
		if (!ret && !ane->soc->platform && call_settle_us)
			usleep_range(call_settle_us, call_settle_us + 100);
		if (ret) {
			if (stats_call)
				ane_stats_complete(&ane->stats_ctrs,
						   &ane->stats_ring,
						   stats_ticket,
						   ktime_get_ns(),
						   (u32)ret, 0);
			dev_err(ane->dev,
				"call completion failed %d within %u ms\n",
				ret, timeout_ms);
			atomic_set(&ane_t6021_quarantined, 1);
			if (!ane->recovering)
				ane_rtclient_recover(ane);
			return ret;
		}
	}
	if (stats_call)
		ane_stats_complete(&ane->stats_ctrs, &ane->stats_ring,
				   stats_ticket, ktime_get_ns(), 0, 0);
	/* The fw talks back on the target-to-host rings (fwlog, perf);
	 * hand those slots back so the rings never fill (channels 4
	 * and 6).
	 */
	if (!ane->soc->platform)
		ane_rtclient_drain_t2h(ane, 4);
	ane_rtclient_drain_t2h(ane, 6);
	if (atomic_read(&ane_t6021_quarantined))
		return -EIO;
	/* BOs are dma_alloc_coherent memory mapped write-combined for the
	 * CPU, so no cache maintenance is needed on either side.
	 */
	return 0;
}

/* ---- LOAD / CREATE / CALL wrappers ---- */

/* Two loads of byte-identical sections share one program and one process.
 * The key is SHA-256 over every section's id, size and bytes, so a client
 * can only reach a program whose bytes it also supplied, and every file
 * holds a reference on each program it loaded. The last close unloads a
 * program, at most ANE_T6021_FW_RESIDENT_MAX programs are resident, and a
 * program a file still holds loads again on its next use after it was
 * unloaded for room or a restart (ane_t6021_lifetime.h). Protected by
 * ane_t6021_fw_lock.
 */
static struct ane_t6021_progtab ane_t6021_progtab;
static struct ane_fw_ids ane_t6021_ids;

static_assert(sizeof(ane_t6021_progtab.progs[0].digest) == SHA256_DIGEST_SIZE);

static struct ane_t6021_prog *ane_t6021_prog_find(const u8 *digest)
{
	return ane_t6021_progtab_find(&ane_t6021_progtab, digest);
}

static struct ane_t6021_prog *ane_t6021_prog_by_id(u32 prog_id)
{
	return ane_t6021_progtab_by_id(&ane_t6021_progtab, prog_id);
}

/* The LIVE program PROG_ID if this file holds a reference on it. */
static struct ane_t6021_prog *ane_t6021_prog_owned(struct ane_t6021_fd *fd,
						   u32 prog_id, int *err)
{
	struct ane_t6021_prog *p;

	if (!fd || !ane_t6021_fdprogs_has(&fd->progs, prog_id)) {
		*err = -ENOENT;
		return NULL;
	}
	p = ane_t6021_prog_by_id(prog_id);
	/* A program a restart or eviction took out of the firmware stays known
	 * while a file holds it. One that is gone failed in a way that left
	 * the firmware uncertain: load it again.
	 */
	*err = p ? 0 : -ESTALE;
	return p;
}

static int ane_t6021_unload_cmd(void *ctx, u16 opcode, u32 fw_prog, u32 fw_proc)
{
	struct ane_rtclient *ane = ctx;
	struct ane_legacy_buffer *command = ane->cmd_buf;
	size_t length = opcode == ANE_T6021_CMD_TERMINATE_PROCESS ? 0x10 : 0x0c;
	int ret;

	if (ane_t6021_pm_submit_error(ane->sleep_pm.state))
		return -EHOSTDOWN;
	if (!command || atomic_read(&ane_t6021_quarantined))
		return -ENODEV;
	memset(command->cpu, 0, SZ_16K);
	*(u32 *)((u8 *)command->cpu + 0x08) = cpu_to_le32(fw_prog);
	if (opcode == ANE_T6021_CMD_TERMINATE_PROCESS)
		*(u32 *)((u8 *)command->cpu + 0x0c) = cpu_to_le32(fw_proc);
	/* A failed command has already quarantined and tried a restart. */
	ret = ane_rtclient_command(ane, command, length, opcode, 1, 3000);
	if (ret)
		return ret;
	/* Terminating a process frees its spill buffers; their FREE requests
	 * can land after the acknowledgment and must be answered before the
	 * next command or the firmware stays locked.
	 */
	ret = ane_rtclient_malloc_settle(ane, 10);
	if (ret) {
		atomic_set(&ane_t6021_quarantined, 1);
		if (!ane->recovering)
			ane_rtclient_recover(ane);
	}
	return ret;
}

/* Firmware side of an unload. Zero only when both commands were acked. */
static int ane_t6021_prog_unload_op(void *ctx, struct ane_t6021_prog *p)
{
	struct ane_rtclient *ane = ctx;
	int ret;

	if (!ane)
		return -ENODEV;
	ret = ane_t6021_prog_unload_cmds(p, ane_t6021_unload_cmd, ane);
	dev_dbg(ane->dev, "unload program %u (fw %u/%d): %d (fw allocations live %u, %zu bytes)\n",
		p->prog_id, p->fw_prog_id, (int)p->fw_proc_id, ret,
		ane->legacy_live, ane->legacy_bytes);
	return ret;
}

/* The firmware no longer reads the sections: drop the program's references.
 * A BO no other program uses stops being a program BO, and is freed or
 * pooled by the usual rule at its last put.
 */
static void ane_t6021_prog_release_op(void *ctx, struct ane_t6021_prog *p)
{
	unsigned int i;

	mutex_lock(&ane_t6021_bo_lock);
	for (i = 0; i < ANE_T6021_PROG_SECS; i++) {
		struct ane_t6021_bo *bo = p->sec[i];

		if (!bo)
			continue;
		p->sec[i] = NULL;
		if (!WARN_ON(!bo->prog_refs) && !--bo->prog_refs) {
			bo->fw_program = false;
			bo->fw_ref = bo->fw_io;
		}
		kref_put(&bo->refcount, ane_t6021_bo_release);
	}
	mutex_unlock(&ane_t6021_bo_lock);
}

/*
 * LOAD_PROGRAM (0x200) from the program's own section records: one 0x30-byte
 * record per section at its identity slot (slot = id - 1; tdprop id 7 lands
 * at slot 6 and slot 5 stays empty when id 6 is absent). flags bit0 = 1, id
 * at +0x04, iova at +0x18, size at +0x20. The firmware writes the program ID
 * back at +0x1b8. Used for the first load and for every reload after an
 * eviction or a restart; the section IOVAs are republished to the current
 * firmware generation before the doorbell.
 */
static int ane_rtclient_send_load(struct ane_rtclient *ane,
				  struct ane_t6021_prog *p)
{
	struct ane_legacy_buffer *command = ane->cmd_buf;
	u8 *cmd;
	u32 fw_prog;
	unsigned int i;
	int ret;

	if (!command || atomic_read(&ane_t6021_quarantined))
		return -ENODEV;
	cmd = command->cpu;
	memset(cmd, 0, SZ_16K);
	mutex_lock(&ane_t6021_bo_lock);
	for (i = 0; i < ANE_T6021_PROG_SECS; i++) {
		u64 slot_base = 0x8 + (u64)i * 0x30;

		if (!p->sec[i])
			continue;
		/* The firmware may read the section any time after the
		 * doorbell, including during a timeout.
		 */
		ane_t6021_bo_publish(p->sec[i], true);
		*(u32 *)(cmd + slot_base + 0x00) = cpu_to_le32(1);
		*(u32 *)(cmd + slot_base + 0x04) = cpu_to_le32(i + 1);
		*(u64 *)(cmd + slot_base + 0x18) = cpu_to_le64(p->sec_iova[i]);
		*(u64 *)(cmd + slot_base + 0x20) = cpu_to_le64(p->sec_size[i]);
	}
	mutex_unlock(&ane_t6021_bo_lock);
	*(u32 *)(cmd + 0x1b8) = cpu_to_le32(U32_MAX);
	/* The LOAD command is 0x1C0 bytes: the nine 0x30 records end at
	 * 0x1b8, ProgramId at +0x1b8, 8 bytes of zero tail.
	 */
	BUILD_BUG_ON(sizeof(struct ane_csne_cmd_load_program) != 0x1b8);
	ret = ane_rtclient_command(ane, command, 0x1c0, CSNE_CMD_LOAD_PROGRAM,
				   1, 5000);
	if (ret)
		return ret;
	fw_prog = le32_to_cpu(*(u32 *)(cmd + 0x1b8));
	/* A LOAD acked without an id leaves the program HELD. */
	if (fw_prog == U32_MAX)
		return -EPROTO;
	p->fw_prog_id = fw_prog;
	return 0;
}

static int ane_t6021_prog_load_op(void *ctx, struct ane_t6021_prog *p)
{
	struct ane_rtclient *ane = ctx;
	int ret = ane_rtclient_send_load(ane, p);

	dev_dbg(ane->dev, "reload program %u: %d (fw %u)\n", p->prog_id, ret,
		p->fw_prog_id);
	return ret;
}

/* CREATE_PROCESS (0x202): programId at +8, the firmware writes the
 * process ID back at +0xc.
 */
static int ane_t6021_prog_create_op(void *ctx, struct ane_t6021_prog *p)
{
	struct ane_rtclient *ane = ctx;
	struct ane_legacy_buffer *command = ane->cmd_buf;
	u8 *cmd;
	u32 fw_proc;
	int ret;

	if (!command || atomic_read(&ane_t6021_quarantined))
		return -ENODEV;
	cmd = command->cpu;
	memset(cmd, 0, SZ_16K);
	*(u32 *)(cmd + 0x08) = cpu_to_le32(p->fw_prog_id);
	*(u32 *)(cmd + 0x0c) = cpu_to_le32(U32_MAX);
	ret = ane_rtclient_command(ane, command, 0x10, CSNE_CMD_CREATE_PROCESS,
				   1, 3000);
	if (ret)
		return ret;
	fw_proc = le32_to_cpu(*(u32 *)(cmd + 0x0c));
	if (fw_proc == U32_MAX)
		return -EPROTO;
	p->fw_proc_id = fw_proc;
	return 0;
}

static int ane_rtclient_restart(struct ane_rtclient *ane);

static int ane_t6021_prog_restart_op(void *ctx)
{
	return ane_rtclient_restart(ctx);
}

static const struct ane_t6021_progtab_ops ane_t6021_progtab_ops = {
	.unload = ane_t6021_prog_unload_op,
	.release = ane_t6021_prog_release_op,
	.load = ane_t6021_prog_load_op,
	.create = ane_t6021_prog_create_op,
	.restart = ane_t6021_prog_restart_op,
};

/* Build a LOAD_PROGRAM (0x200) message in a kernel-owned buffer. The
 * section bytes live in the BOs the user supplied (section_ptr), the
 * generic binds (bufferId, bo_handle, size) patch the generic section
 * at iova +0x20 / +0x28 once the BO is copied. Returns 0 with
 * *prog_id set on success.
 */
static int ane_rtclient_load_program(struct ane_rtclient *ane,
				     struct drm_file *file,
				     const struct drm_ane_prog_load *user,
				     __u32 *prog_id)
{
	struct drm_ane_section *sections;
	struct drm_ane_generic_bind *binds;
	struct ane_t6021_fd *fd = file->driver_priv;
	struct ane_t6021_prog *cached, *prog = NULL;
	bool sent = false;
	int td_flags = 0;
	u32 key, gen;
	u8 digest[SHA256_DIGEST_SIZE];
	size_t binds_size;
	int ret, i, j;

	BUILD_BUG_ON(DRM_ANE_MAX_SECTIONS != ANE_T6021_LOAD_SEC_COUNT);
	/* H16 requires the original HWX, not caller-asserted section identity. */
	if (ane->soc->platform)
		return -EOPNOTSUPP;
	if (user->pad || user->section_count < 1 ||
	    user->section_count > ANE_T6021_LOAD_SEC_COUNT ||
	    user->generic_count > DRM_ANE_MAX_BINDS || !fd)
		return -EINVAL;

	sections = kmalloc_array(user->section_count, sizeof(*sections),
				 GFP_KERNEL);
	binds = kmalloc_array(user->generic_count, sizeof(*binds),
			      GFP_KERNEL);
	if (!sections || !binds) {
		ret = -ENOMEM;
		goto out;
	}
	if (copy_from_user(sections,
			   u64_to_user_ptr(user->sections_ptr),
			   user->section_count * sizeof(*sections))) {
		ret = -EFAULT;
		goto out;
	}
	binds_size = user->generic_count * sizeof(*binds);
	if (binds_size && copy_from_user(binds,
					 u64_to_user_ptr(user->generic_ptr),
					binds_size)) {
		ret = -EFAULT;
		goto out;
	}
	for (i = 0; i < user->generic_count; i++) {
		if (binds[i].pad) {
			ret = -EINVAL;
			goto out;
		}
	}

	/* The record slot IS the section identity (slot =
	 * id - 1; tdprop id 7 lands at slot 6 and slot 5 stays empty
	 * when id 6 is not supplied). Reject out-of-range and
	 * duplicate ids before building the wire image.
	 */
	for (i = 0; i < user->section_count; i++) {
		if (sections[i].id < 1 ||
		    sections[i].id > ANE_T6021_LOAD_SEC_COUNT) {
			ret = -EINVAL;
			goto out;
		}
		for (j = i + 1; j < user->section_count; j++) {
			if (sections[i].id == sections[j].id) {
				ret = -EINVAL;
				goto out;
			}
		}
	}

	/* Identical sections share one firmware program (see the table).
	 * A constant section can reach 20 MiB (the H14 (K,N)=(2048,5120)
	 * matvec), so feed the hash in ANE_T6021_BO_HASH_CHUNK-sized slices
	 * under bo_lock: a single sha256_update of a 20 MiB buffer would
	 * rely on a 20 MiB stack argument list and has no upper bound on
	 * the chunk that the BO could supply.
	 */
	{
		struct sha256_ctx sha;
		void *scratch;

		sha256_init(&sha);
		scratch = kvmalloc(ANE_T6021_BO_HASH_CHUNK, GFP_KERNEL);
		if (!scratch) {
			ret = -ENOMEM;
			goto out;
		}
		for (i = 0; i < user->section_count; i++) {
			struct ane_t6021_bo *bo = NULL, *b;
			u64 hdr[2] = { sections[i].id, sections[i].size };
			size_t left;

			mutex_lock(&ane_t6021_bo_lock);
			list_for_each_entry(b, &fd->bos, node) {
				if (b->handle == sections[i].bo_handle) {
					bo = b;
					break;
				}
			}
			if (!bo || sections[i].size > bo->size ||
			    sections[i].offset > bo->size - sections[i].size) {
				mutex_unlock(&ane_t6021_bo_lock);
				kvfree(scratch);
				ret = -EINVAL;
				goto out;
			}
			sha256_update(&sha, (const u8 *)hdr, sizeof(hdr));
			/* Checked in the same snapshot as the key. */
			if (sections[i].id == ANE_T6021_SEC_TEXT)
				td_flags = ane_t6021_td_flags_check((const u8 *)bo->cpu +
								    sections[i].offset,
								    sections[i].size);
			left = sections[i].size;
			while (left) {
				size_t n = min_t(size_t, left,
						 ANE_T6021_BO_HASH_CHUNK);

				memcpy(scratch, (const u8 *)bo->cpu +
				       sections[i].offset +
				       (sections[i].size - left), n);
				sha256_update(&sha, scratch, n);
				left -= n;
			}
			mutex_unlock(&ane_t6021_bo_lock);
		}
		kvfree(scratch);
		sha256_final(&sha, digest);
	}
	/* A program that opens or closes on the wrong task wedges the firmware
	 * for every client; refuse it before the firmware sees it.
	 */
	if (td_flags) {
		dev_info_ratelimited(ane->dev,
				     "LOAD refused: TD first/last task flags malformed\n");
		ret = td_flags;
		goto out;
	}
	cached = ane_t6021_prog_find(digest);
	if (cached) {
		/* A resident cache entry or a non-resident program is reused as
		 * is; it goes back into the firmware on its next use.
		 */
		ret = ane_t6021_fdprogs_ref(&ane_t6021_progtab, &fd->progs,
					    cached);
		if (!ret)
			*prog_id = cached->prog_id;
		goto out;
	}
	/* Check the file can hold the reference before the firmware loads a
	 * program nobody could then unload.
	 */
	if (!ane_t6021_fdprogs_room(&ane_t6021_progtab, &fd->progs)) {
		ret = -ENOSPC;
		goto out;
	}
	if (!ane->cmd_buf) {
		ret = -ENODEV;
		goto out;
	}
	/* Room first: a restart frees every unreferenced slot. */
	ret = ane_t6021_progtab_make_room(&ane_t6021_progtab, NULL,
					  &ane_t6021_progtab_ops, ane);
	if (ret)
		goto out;
	ret = ane_fw_id_alloc(&ane_t6021_ids, &key);
	if (ret)
		goto out;
	prog = ane_t6021_progtab_reserve(&ane_t6021_progtab, digest);
	if (!prog) {
		ret = -ENOSPC;
		goto out;
	}
	gen = ane_t6021_progtab.gen;
	/* The record slot IS the section identity (slot = id - 1). The program
	 * takes its own reference on each section buffer, kept until the
	 * firmware dropped the program for good (or forever if uncertain).
	 */
	for (i = 0; i < user->section_count; i++) {
		struct ane_t6021_bo *bo = NULL, *b;
		unsigned int slot = sections[i].id - 1;

		mutex_lock(&ane_t6021_bo_lock);
		list_for_each_entry(b, &fd->bos, node) {
			if (b->handle == sections[i].bo_handle) {
				bo = b;
				break;
			}
		}
		if (bo && sections[i].size <= bo->size &&
		    sections[i].offset <= bo->size - sections[i].size) {
			kref_get(&bo->refcount);
			bo->prog_refs++;
			prog->sec[slot] = bo;
			prog->sec_iova[slot] = bo->dma + sections[i].offset;
			prog->sec_size[slot] = sections[i].size;
		} else {
			bo = NULL;
		}
		mutex_unlock(&ane_t6021_bo_lock);
		if (!bo) {
			ret = -EINVAL;
			goto out;
		}
	}
	/* Generic binds are accepted for ABI compatibility and unused: the
	 * LOAD record already carries iova + size, and the generic section
	 * image is prepared by userspace (generic.bin).
	 */
	sent = true;
	ret = ane_rtclient_send_load(ane, prog);
	if (!ret) {
		ane_t6021_progtab_commit(&ane_t6021_progtab, prog, key,
					 prog->fw_prog_id);
		/* Room was checked under the same lock. */
		ret = ane_t6021_fdprogs_ref(&ane_t6021_progtab, &fd->progs, prog);
		if (WARN_ON(ret))
			goto out;
		*prog_id = key;
		dev_dbg(ane->dev, "loaded program %u (fw %u) (fw allocations live %u, %zu bytes)\n",
			key, prog->fw_prog_id, ane->legacy_live, ane->legacy_bytes);
	} else if (ane_t6021_progtab.gen != gen) {
		/* A firmware restart already released the reservation. */
		prog = NULL;
	}

out:
	/* The firmware never saw these IOVAs: give the reservation back. */
	if (prog && !sent)
		ane_t6021_progtab_cancel(prog, &ane_t6021_progtab_ops, ane);
	kfree(sections);
	kfree(binds);
	return ret;
}

static int ane_rtclient_create_process(struct ane_rtclient *ane,
				       struct drm_file *file,
				       __u32 prog_id, __u32 *proc_id)
{
	struct ane_t6021_prog *prog;
	int ret;

	/* H16 uses 0x258 bytes; its body is not the H14 0x10-byte body. */
	if (ane->soc->platform)
		return -EOPNOTSUPP;
	prog = ane_t6021_prog_owned(file->driver_priv, prog_id, &ret);
	if (!prog)
		return ret;
	/* One firmware process per program, shared by its files. The public
	 * process ID stays the same when a restart or an eviction replaces
	 * the firmware process.
	 */
	ret = ane_t6021_progtab_ensure(&ane_t6021_progtab, prog, true,
				       &ane_t6021_progtab_ops, ane);
	if (ret)
		return ret;
	prog->proc_id = prog->prog_id;
	*proc_id = prog->proc_id;
	return 0;
}

/* H16 consumes only backend-owned process resources. The ABI-2 section
 * loader and short process packet cannot create an entry in h16_processes.
 */
static int ane_rtclient_h16_call(struct ane_rtclient *ane, struct drm_file *file,
				 const struct drm_ane_exec *user)
{
	struct ane_t6021_fd *fd = file->driver_priv;
	struct ane_h16_process *process = NULL, *p;
	struct ane_h16_execute_buffer buffers[ANE_H16_SURFACES];
	struct ane_t6021_bo *bos[ANE_H16_SURFACES] = {};
	struct drm_ane_exec_io *ios;
	struct ane_h16_pending request = {};
	struct ane_h16_execute call = {};
	unsigned int i, j, pinned = 0;
	int ret = -EOPNOTSUPP;

	if (!fd || user->pad || user->priority != 5 || user->count < 2 ||
	    user->count > ANE_H16_SURFACES || user->timeout_ms > 30000)
		return -EINVAL;
	ios = memdup_user(u64_to_user_ptr(user->io_ptr),
			  array_size(user->count, sizeof(*ios)));
	if (IS_ERR(ios))
		return PTR_ERR(ios);
	mutex_lock(&ane_t6021_fw_lock);
	if (ane_rtclient_admit(ane)) {
		ret = -EHOSTDOWN;
		goto out;
	}
	if (atomic_read(&ane_t6021_quarantined) || ane->pending.quarantined) {
		ret = -EIO;
		goto out;
	}
	list_for_each_entry(p, &fd->h16_processes, owner_node) {
		if (p->key.program == user->prog_id &&
		    p->key.process == user->proc_id) {
			process = p;
			break;
		}
	}
	if (!process)
		goto out;
	ret = ane_h16_hwx_snapshot_check(&process->original);
	if (!ret)
		ret = ane_h16_resources_validate(&process->original.info,
						 process->resources, process->resource_count);
	if (ret)
		goto out;
	if (user->count != process->resource_count || !ane->cmd_buf ||
	    process->key.program == U32_MAX || process->key.process == U32_MAX ||
	    process->key.procedure == U32_MAX) {
		ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < user->count; i++) {
		struct ane_t6021_bo *b;
		const struct ane_h16_resource *r = &process->resources[i];
		const struct ane_legacy_buffer *m = &process->mappings[i];

		if (ios[i].flags || ios[i].reserved ||
		    ios[i].buffer_id != r->buffer.index || ios[i].type != r->buffer.class ||
		    ios[i].size != r->buffer.size || !m->cpu ||
		    m->dma != r->buffer.mapping_dva || m->size != r->buffer.mapping_size) {
			ret = -EINVAL;
			goto out;
		}
		for (j = 0; j < i; j++) {
			if (ios[j].bo_handle == ios[i].bo_handle) {
				ret = -EINVAL;
				goto out;
			}
		}
		mutex_lock(&ane_t6021_bo_lock);
		list_for_each_entry(b, &fd->bos, node) {
			if (b->handle == ios[i].bo_handle && b->owner == fd &&
			    b->size >= ios[i].size) {
				kref_get(&b->refcount);
				bos[pinned++] = b;
				break;
			}
		}
		mutex_unlock(&ane_t6021_bo_lock);
		if (pinned != i + 1) {
			ret = -ENOENT;
			goto out;
		}
		buffers[i] = r->buffer;
	}
	/* All user-facing BOs are staging only. Concurrent mmap writes cannot
	 * change code, input snapshots or firmware-visible output allocations.
	 */
	for (i = 0; i < user->count; i++) {
		if (process->resources[i].permissions == 1)
			memcpy(process->mappings[i].cpu, bos[i]->cpu, buffers[i].size);
		else
			memset(process->mappings[i].cpu, 0xff, buffers[i].size);
	}
	request.key = process->key;
	request.deadline = jiffies + msecs_to_jiffies(user->timeout_ms ?: 5000);
	ret = ane_h16_pending_insert(&ane->pending, &request, jiffies);
	if (ret)
		goto out;
	call.key = request.key;
	call.buffers = buffers;
	call.buffer_count = user->count;
	/* Custom BAR selectors/values have no validated process map yet. */
	ret = ane_h16_execute_encode(ane->cmd_buf->cpu, ANE_H16_EXECUTE_BASE_SIZE, &call);
	if (ret) {
		ane_h16_pending_remove(&ane->pending, &request);
		goto out;
	}
	ane->waiting = &request;
	/* Coherent allocations require ordering, not streaming-map sync calls. */
	wmb();
	dma_wmb();
	ret = ane_rtclient_command(ane, ane->cmd_buf, ANE_H16_EXECUTE_BASE_SIZE,
				   ANE_H16_EXECUTE, 7, user->timeout_ms ?: 5000);
	ane->waiting = NULL;
	if (ret || !request.terminal || request.status) {
		ret = ret ?: -EIO;
		ane_h16_pending_abort(&ane->pending, &request, ret);
		atomic_set(&ane_t6021_quarantined, 1);
		goto out;
	}
	dma_rmb();
	for (i = 0; i < user->count; i++)
		if (process->resources[i].permissions == 2)
			memcpy(bos[i]->cpu, process->mappings[i].cpu, buffers[i].size);
	/* Private DMA mappings remain resident for the entire process lifetime,
	 * including after terminal failure or an uncertain transport outcome.
	 */
out:
	while (pinned)
		kref_put(&bos[--pinned]->refcount, ane_t6021_bo_release);
	mutex_unlock(&ane_t6021_fw_lock);
	kfree(ios);
	return ret;
}

static int ane_rtclient_procedure_call(struct ane_rtclient *ane,
				       struct drm_file *file,
				       const struct drm_ane_exec *user)
{
	struct drm_ane_exec_io *ios;
	struct ane_t6021_bo **exec_bos;
	struct ane_legacy_buffer *command;
	struct ane_t6021_prog *program;
	struct ane_t6021_fd *fd = file->driver_priv;
	size_t ios_size;
	size_t cmd_size;
	int ret, i, pinned = 0;

	if (ane->soc->platform)
		return ane_rtclient_h16_call(ane, file, user);
	if (user->pad || user->count < 1 || user->count > DRM_ANE_MAX_BINDS ||
	    user->priority < 2 || user->priority > 7 || !fd)
		return -EINVAL;

	ios_size = (size_t)user->count * sizeof(*ios);
	ios = kmalloc(ios_size, GFP_KERNEL);
	if (!ios)
		return -ENOMEM;
	if (copy_from_user(ios, u64_to_user_ptr(user->io_ptr), ios_size)) {
		kfree(ios);
		return -EFAULT;
	}
	for (i = 0; i < user->count; i++) {
		if (ios[i].flags || ios[i].reserved) {
			kfree(ios);
			return -EINVAL;
		}
	}

	cmd_size = sizeof(struct ane_csne_cmd_procedure_call) +
		   (size_t)user->count * sizeof(struct ane_csne_io_elem);
	if (cmd_size > SZ_16K) {
		kfree(ios);
		return -E2BIG;
	}

	exec_bos = kcalloc(user->count, sizeof(*exec_bos), GFP_KERNEL);
	if (!exec_bos) {
		kfree(ios);
		return -ENOMEM;
	}

	mutex_lock(&ane_t6021_fw_lock);
	if (ane_rtclient_admit(ane)) {
		ret = -EHOSTDOWN;
		goto unlock;
	}
	if (atomic_read(&ane_t6021_quarantined)) {
		ret = -ETIMEDOUT;
		goto unlock;
	}
	program = ane_t6021_prog_owned(fd, user->prog_id, &ret);
	if (!program)
		goto unlock;
	if (program->proc_id == U32_MAX || program->proc_id != user->proc_id) {
		ret = -ESTALE;
		goto unlock;
	}
	/* Load it and its process again if an eviction or a restart took
	 * them out of the firmware. This may restart the firmware.
	 */
	ret = ane_t6021_progtab_ensure(&ane_t6021_progtab, program, true,
				       &ane_t6021_progtab_ops, ane);
	if (ret)
		goto unlock;
	command = ane->cmd_buf;
	if (!command) {
		ret = -ENODEV;
		goto unlock;
	}
	memset(command->cpu, 0, SZ_16K);
	{
		u8 *cmd = command->cpu;

		*(u32 *)(cmd + 0x08) = cpu_to_le32(program->fw_prog_id);
		*(u32 *)(cmd + 0x0c) = cpu_to_le32(program->fw_proc_id);
		*(u64 *)(cmd + 0x10) = cpu_to_le64(0);
		*(u32 *)(cmd + 0x18) = cpu_to_le32(user->priority);
		*(u64 *)(cmd + 0x20) = cpu_to_le64(ANE_CALL_COOKIE);
		*(u32 *)(cmd + 0x28) = cpu_to_le32(user->count);
		for (i = 0; i < user->count; i++) {
			struct ane_t6021_bo *bo = NULL, *b;
			u64 slot_base = 0x60 + (u64)i * 0x30;
			u64 iova;

			mutex_lock(&ane_t6021_bo_lock);
			list_for_each_entry(b, &fd->bos, node) {
				if (b->handle == ios[i].bo_handle) {
					bo = b;
					break;
				}
			}
			if (!bo || ios[i].size > bo->size) {
				mutex_unlock(&ane_t6021_bo_lock);
				ret = -EINVAL;
				goto unlock;
			}
			/* Keep this IO element out of the reuse pool until completion. */
			kref_get(&bo->refcount);
			exec_bos[pinned++] = bo;
			ane_t6021_bo_publish(bo, false);
			iova = bo->dma;
			mutex_unlock(&ane_t6021_bo_lock);
			*(u32 *)(cmd + slot_base + 0x00) = cpu_to_le32(1);
			*(u32 *)(cmd + slot_base + 0x04) =
				cpu_to_le32(ios[i].buffer_id);
			*(u32 *)(cmd + slot_base + 0x08) =
				cpu_to_le32(ios[i].type);
			*(u64 *)(cmd + slot_base + 0x18) =
				cpu_to_le64(iova);
			*(u64 *)(cmd + slot_base + 0x20) =
				cpu_to_le64(ios[i].size);
		}
		/* Drain the CPU write buffers so the input BOs the user
		 * filled through its uncached mapping are in DRAM before
		 * the fw starts reading them.
		 */
		wmb();
		ret = ane_rtclient_command(ane, command,
					   cmd_size,
							       CSNE_CMD_PROCEDURE_CALL,
							       1,
							       user->timeout_ms ?
							       user->timeout_ms : 5000);
	}
unlock:
	/* Command failures latch quarantine before these final puts. */
	while (pinned)
		kref_put(&exec_bos[--pinned]->refcount, ane_t6021_bo_release);
	kfree(exec_bos);
	mutex_unlock(&ane_t6021_fw_lock);
	kfree(ios);
	return ret;
}

/* ---- DRM accel glue (minimal BO + ioctl table) ---- */

struct ane_t6021_drm {
	struct drm_device drm;
	struct device *dev;
	struct ane_rtclient *ane;
};

static struct ane_t6021_drm *to_ane_t6021_drm(struct drm_device *drm)
{
	return container_of(drm, struct ane_t6021_drm, drm);
}

static int ane_t6021_bo_init_locked(struct drm_device *drm, void *data,
				    struct drm_file *file)
{
	struct drm_ane_bo_init *args = data;
	struct ane_t6021_fd *fd = file->driver_priv;
	struct ane_t6021_bo *bo;
	struct ane_rtclient *ane = to_ane_t6021_drm(drm)->ane;

	if (ane_rtclient_admit(ane))
		return -EHOSTDOWN;
	if (atomic_read(&ane_t6021_quarantined) || READ_ONCE(ane->recovering))
		return -EIO;
	if (args->pad || args->size == 0 || args->size > ANE_T6021_BO_MAX ||
	    !fd)
		return -EINVAL;
	/* A parked io BO is already mapped and counted; its old contents
	 * belong to another process, so it is zeroed like a new one.
	 */
	bo = ane_t6021_bo_pool_take(args->size);
	if (bo) {
		memset(bo->cpu, 0, PAGE_ALIGN(args->size));
		goto publish;
	}
	/* Global coherent-memory accounting. Each BO is 16 KiB-aligned;
	 * a BO whose IOVA reaches the firmware is never freed (held or
	 * pooled), so this bound caps the memory that outlives its
	 * users.
	 */
	if (atomic64_add_return(PAGE_ALIGN(args->size), &ane_t6021_bo_total_bytes) >
	    (s64)bo_total_max_mb << 20) {
		atomic64_sub(PAGE_ALIGN(args->size), &ane_t6021_bo_total_bytes);
		return -ENOSPC;
	}
	bo = kzalloc_obj(*bo);
	if (!bo) {
		atomic64_sub(PAGE_ALIGN(args->size), &ane_t6021_bo_total_bytes);
		return -ENOMEM;
	}
	bo->cpu = dma_alloc_coherent(drm->dev, args->size, &bo->dma,
				     GFP_KERNEL);
	if (!bo->cpu) {
		atomic64_sub(PAGE_ALIGN(args->size), &ane_t6021_bo_total_bytes);
		kfree(bo);
		return -ENOMEM;
	}
	/* Every fw-visible DMA surface must be 16 KiB aligned and clear
	 * of the firmware entry alias (the same invariant
	 * ane_rtclient_legacy_alloc enforces).
	 */
	if (!IS_ALIGNED(bo->dma, SZ_16K) ||
	    (ane->fw && !ane_t6021_fw_alias_iova_ok(ane->fw, bo->dma,
						    args->size))) {
		dma_free_coherent(drm->dev, args->size, bo->cpu, bo->dma);
		atomic64_sub(PAGE_ALIGN(args->size), &ane_t6021_bo_total_bytes);
		kfree(bo);
		return -ERANGE;
	}
publish:
	bo->size = args->size;
	bo->owner = fd;
	bo->dev = drm->dev;
	kref_init(&bo->refcount); /* the handle holds this reference */
	mutex_lock(&ane_t6021_bo_lock);
	bo->handle = ane_t6021_next_handle++;
	if (bo->handle == 0)
		bo->handle = ane_t6021_next_handle++;
	list_add_tail(&bo->node, &fd->bos);
	mutex_unlock(&ane_t6021_bo_lock);
	args->handle = bo->handle;
	/* mmap offset = the handle; libane mmaps the fd at exactly this
	 * offset and ane_t6021_mmap resolves the BO from vm_pgoff.
	 */
	args->offset = (u64)bo->handle << PAGE_SHIFT;
	return 0;
}

static int ane_t6021_bo_init_ioctl(struct drm_device *drm, void *data,
				   struct drm_file *file)
{
	int ret;

	mutex_lock(&ane_t6021_fw_lock);
	ret = ane_t6021_bo_init_locked(drm, data, file);
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

/* Drop one handle owned by this fd. The kref may keep the memory
 * alive past this call — a user mapping still holds a reference — so
 * the final ane_t6021_bo_release makes the free-or-hold decision.
 */
static void ane_t6021_bo_drop(struct ane_t6021_bo *bo)
{
	list_del(&bo->node);
	kref_put(&bo->refcount, ane_t6021_bo_release);
}

static int ane_t6021_bo_free_locked(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	struct drm_ane_bo_free *args = data;
	struct ane_t6021_fd *fd = file->driver_priv;
	struct ane_t6021_bo *bo = NULL, *b;

	if (!fd)
		return -ENODEV;
	if (args->pad)
		return -EINVAL;
	mutex_lock(&ane_t6021_bo_lock);
	list_for_each_entry(b, &fd->bos, node) {
		if (b->handle == args->handle) {
			bo = b;
			break;
		}
	}
	if (bo)
		ane_t6021_bo_drop(bo);
	mutex_unlock(&ane_t6021_bo_lock);
	return bo ? 0 : -ENOENT;
}

static int ane_t6021_bo_free_ioctl(struct drm_device *drm, void *data,
				 struct drm_file *file)
{
	struct ane_t6021_drm *adrm = to_ane_t6021_drm(drm);
	int ret;

	mutex_lock(&ane_t6021_fw_lock);
	ret = ane_rtclient_admit(adrm->ane);
	if (!ret)
		ret = ane_t6021_bo_free_locked(drm, data, file);
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

/* Resolve the BO a user mmap names (BO_INIT returned offset = handle
 * << PAGE_SHIFT) and map the coherent buffer cacheably — the device
 * half coheres through the DART (IOMMU_CACHE), so reads after EXEC
 * observe the firmware's writes without extra sync. The mapping takes
 * one BO reference: it survives BO_FREE until the vma is gone.
 */
static int ane_t6021_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file = filp->private_data;
	struct ane_t6021_fd *fd = file->driver_priv;
	struct drm_device *drm = file->minor->dev;
	struct ane_t6021_bo *bo = NULL, *b;
	size_t size = vma->vm_end - vma->vm_start;
	u32 handle;
	int ret;

	if (!fd)
		return -ENODEV;
	/* vm_pgoff is the BO_INIT offset in pages: handle << PAGE_SHIFT
	 * >> PAGE_SHIFT == handle.
	 */
	handle = (u32)vma->vm_pgoff;
	if (!handle)
		return -EINVAL;
	mutex_lock(&ane_t6021_bo_lock);
	list_for_each_entry(b, &fd->bos, node) {
		if (b->handle == handle && b->owner == fd) {
			kref_get(&b->refcount);
			bo = b;
			break;
		}
	}
	if (!bo || size > PAGE_ALIGN(bo->size)) {
		mutex_unlock(&ane_t6021_bo_lock);
		if (bo)
			kref_put(&bo->refcount, ane_t6021_bo_release);
		return -ENOENT;
	}
	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(drm->dev, vma, bo->cpu, bo->dma, size);
	mutex_unlock(&ane_t6021_bo_lock);
	if (ret) {
		kref_put(&bo->refcount, ane_t6021_bo_release);
		return ret;
	}
	/* The vma owns the lookup reference: close (and open on fork)
	 * go through ane_t6021_vm_ops.
	 */
	vma->vm_private_data = bo;
	vma->vm_ops = &ane_t6021_vm_ops;
	return 0;
}

static int ane_t6021_open(struct drm_device *drm, struct drm_file *file)
{
	struct ane_t6021_fd *fd;

	fd = kzalloc_obj(*fd);
	if (!fd)
		return -ENOMEM;
	INIT_LIST_HEAD(&fd->bos);
	INIT_LIST_HEAD(&fd->h16_processes);
	file->driver_priv = fd;
	return 0;
}

static void ane_t6021_postclose(struct drm_device *drm, struct drm_file *file)
{
	struct ane_t6021_fd *fd = file->driver_priv;
	struct ane_t6021_bo *bo, *tmp;
	struct ane_h16_process *process, *next;

	if (!fd)
		return;
	mutex_lock(&ane_t6021_fw_lock);
	/* Revoke per-file authority, retaining device-owned DMA allocations. */
	list_for_each_entry_safe(process, next, &fd->h16_processes, owner_node)
		list_del_init(&process->owner_node);
	/* Unload every program this file was the last user of. A failed
	 * unload holds the program and its sections until a verified restart.
	 */
	if (ane_t6021_progtab_fd_close(&ane_t6021_progtab, &fd->progs,
				       &ane_t6021_progtab_ops,
				       to_ane_t6021_drm(drm)->ane))
		dev_warn(drm->dev, "program unload failed; sections held until the firmware restarts\n");
	mutex_lock(&ane_t6021_bo_lock);
	list_for_each_entry_safe(bo, tmp, &fd->bos, node)
		ane_t6021_bo_drop(bo);
	mutex_unlock(&ane_t6021_bo_lock);
	mutex_unlock(&ane_t6021_fw_lock);
	kfree(fd);
	file->driver_priv = NULL;
}

static int ane_t6021_get_caps_ioctl(struct drm_device *drm, void *data,
				    struct drm_file *file)
{
	struct drm_ane_get_caps *args = data;

	if (args->flags || args->pad)
		return -EINVAL;

	*args = (struct drm_ane_get_caps) {
		.size = sizeof(*args),
		.abi_version = DRM_ANE_ABI_V2,
		.chip_family = DRM_ANE_CHIP_H14,
		.section_size = sizeof(struct drm_ane_section),
		.bind_size = sizeof(struct drm_ane_generic_bind),
		.exec_io_size = sizeof(struct drm_ane_exec_io),
	};
	return 0;
}

static int ane_t6021_submit_ioctl(struct drm_device *drm, void *data,
				  struct drm_file *file)
{
	return -ENOTTY; /* ABI 1 SUBMIT is rejected on T6021/M2 */
}

static int ane_t6021_prog_load_ioctl(struct drm_device *drm, void *data,
				     struct drm_file *file)
{
	struct ane_t6021_drm *adrm = to_ane_t6021_drm(drm);
	struct drm_ane_prog_load *args = data;
	int ret;

	mutex_lock(&ane_t6021_fw_lock);
	if (ane_rtclient_admit(adrm->ane)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -EHOSTDOWN;
	}
	if (atomic_read(&ane_t6021_quarantined)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -ETIMEDOUT;
	}
	ret = ane_rtclient_load_program(adrm->ane, file, args,
					&args->prog_id_out);
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

/* Firmware perf mode (ane_t6021_perf_mode_cmd()): sent at every firmware
 * power-on on T8112, as macOS does, and again after each restart. Writing 1
 * to /sys/module/ane_t6021/parameters/fw_perf_mode sends it at run time on
 * a SoC that does not send it at power-on; it reads back whether it was
 * sent. Only 1 is accepted: no other value is known to be safe.
 */
static struct ane_rtclient *ane_t6021_perf_ane;
static bool fw_perf_mode;

static bool perf_mode_at_boot = true;
module_param(perf_mode_at_boot, bool, 0444);
MODULE_PARM_DESC(perf_mode_at_boot,
		 "T8112: set the firmware perf mode at power-on, as macOS does (default 1)");

/* Send the perf-mode property on the command channel and remember it, so a
 * firmware restart sends it again. Caller holds the command buffer.
 */
static int ane_rtclient_send_perf_mode(struct ane_rtclient *ane,
				       unsigned long deadline)
{
	struct ane_legacy_buffer *command = ane->cmd_buf;
	size_t len;
	int ret;

	memset(command->cpu, 0, SZ_16K);
	len = ane_t6021_perf_mode_cmd(command->cpu);
	ret = ane_rtclient_legacy_exchange(ane, command, len,
					   CSNE_CMD_CH_PROPERTY_WRITE, 1,
					   deadline);
	if (!ret) {
		fw_perf_mode = true;
		ane->desired_perf = true;
	}
	return ret;
}

/* debugfs fw_memory: what the firmware holds now. */
static int ane_t6021_fw_memory_show(struct seq_file *m, void *v)
{
	struct drm_debugfs_entry *entry = m->private;
	struct ane_rtclient *ane = entry->file.data;
	struct ane_t6021_progtab_stats st;
	int ret;

	ret = mutex_lock_interruptible(&ane_t6021_fw_lock);
	if (ret)
		return ret;
	ane_t6021_progtab_count(&ane_t6021_progtab, &st);
	seq_printf(m, "allocations: %u\nbytes: %zu\nhigh_water: %u\n",
		   ane->legacy_live, ane->legacy_bytes, ane->legacy_allocated);
	seq_printf(m, "programs: %u\nheld: %u\nresident: %u\ncached: %u\n",
		   st.live, st.held, st.resident, st.cached);
	seq_printf(m, "resident_max: %u\nrestarts: %u\nfw_generation: %u\nquarantined: %d\n",
		   ANE_T6021_FW_RESIDENT_MAX, ane_t6021_progtab.restarts,
		   ane_t6021_progtab.gen, atomic_read(&ane_t6021_quarantined));
	mutex_unlock(&ane_t6021_fw_lock);
	return 0;
}

static int ane_t6021_perf_mode_set(const char *val,
				   const struct kernel_param *kp)
{
	struct ane_rtclient *ane = READ_ONCE(ane_t6021_perf_ane);
	struct ane_legacy_buffer *command;
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;
	if (!on || fw_perf_mode)
		return on ? 0 : -EINVAL;
	if (!ane)
		return -ENODEV;
	mutex_lock(&ane_t6021_fw_lock);
	if (ane_rtclient_admit(ane)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -EHOSTDOWN;
	}
	if (atomic_read(&ane_t6021_quarantined)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -ETIMEDOUT;
	}
	command = ane->cmd_buf;
	if (!command) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -ENODEV;
	}
	memset(command->cpu, 0, SZ_16K);
	ret = ane_rtclient_command(ane, command,
				   ane_t6021_perf_mode_cmd(command->cpu),
				   CSNE_CMD_CH_PROPERTY_WRITE, 1, 3000);
	if (!ret) {
		fw_perf_mode = true;
		ane->desired_perf = true;
		dev_info(ane->dev, "fw perf mode set\n");
	}
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

static const struct kernel_param_ops ane_t6021_perf_mode_ops = {
	.set = ane_t6021_perf_mode_set,
	.get = param_get_bool,
};
module_param_cb(fw_perf_mode, &ane_t6021_perf_mode_ops, &fw_perf_mode, 0644);
MODULE_PARM_DESC(fw_perf_mode,
		 "Write 1 once to set the firmware perf mode; reads back whether it was sent");

static int ane_t6021_proc_create_ioctl(struct drm_device *drm, void *data,
				       struct drm_file *file)
{
	struct ane_t6021_drm *adrm = to_ane_t6021_drm(drm);
	struct drm_ane_proc_create *args = data;
	int ret;

	mutex_lock(&ane_t6021_fw_lock);
	if (ane_rtclient_admit(adrm->ane)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -EHOSTDOWN;
	}
	if (atomic_read(&ane_t6021_quarantined)) {
		mutex_unlock(&ane_t6021_fw_lock);
		return -ETIMEDOUT;
	}
	args->proc_id_out = 0;
	ret = ane_rtclient_create_process(adrm->ane, file, args->prog_id,
					  &args->proc_id_out);
	mutex_unlock(&ane_t6021_fw_lock);
	return ret;
}

static int ane_t6021_exec_ioctl(struct drm_device *drm, void *data,
				struct drm_file *file)
{
	struct ane_t6021_drm *adrm = to_ane_t6021_drm(drm);
	struct drm_ane_exec *args = data;

	return ane_rtclient_procedure_call(adrm->ane, file, args);
}

static const struct drm_ioctl_desc ane_t6021_ioctls[] = {
	DRM_IOCTL_DEF_DRV(ANE_GET_CAPS, ane_t6021_get_caps_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_BO_INIT, ane_t6021_bo_init_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_BO_FREE, ane_t6021_bo_free_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_SUBMIT, ane_t6021_submit_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_PROG_LOAD, ane_t6021_prog_load_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_PROC_CREATE, ane_t6021_proc_create_ioctl, 0),
	DRM_IOCTL_DEF_DRV(ANE_EXEC, ane_t6021_exec_ioctl, 0),
};

/* Driver fops: the accel-core entry points plus our BO mmap (the core
 * default maps only GEM objects; this driver keeps its own BO table,
 * so .mmap resolves the handle from BO_INIT's returned offset).
 */
static const struct file_operations ane_t6021_fops = {
	.owner = THIS_MODULE,
	.fop_flags = FOP_UNSIGNED_OFFSET,
	.open = accel_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
	.mmap = ane_t6021_mmap,
};

/* Version reported through DRM_IOCTL_VERSION: ABI 2 (T6021).
 * DRIVER_COMPUTE_ACCEL puts the node at /dev/accel/accelN.
 */
static const struct drm_driver ane_t6021_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_COMPUTE_ACCEL,
	.open = ane_t6021_open,
	.postclose = ane_t6021_postclose,
	.ioctls = ane_t6021_ioctls,
	.num_ioctls = ARRAY_SIZE(ane_t6021_ioctls),
	.fops = &ane_t6021_fops,
	.major = DRM_ANE_ABI_V2,
	.minor = 0,
	.name = "ane",
	.desc = "Apple Neural Engine (T6021/M2)",
};

/* ---- RTKit ops (kept on the boot path; the legacy ChMan transport
 * is used by the ioctls once probe confirms legacy_only / chman_ok).
 */

static void ane_rtclient_recv(void *cookie, u8 ep, u64 message)
{
	struct ane_rtclient *ane = cookie;

	dev_dbg(ane->dev,
		"rtkit app msg: ep=%#x msg=%016llx\n", ep, message);
}

static void ane_rtclient_crashed(void *cookie, const void *crashlog,
				 size_t size)
{
	struct ane_rtclient *ane = cookie;

	dev_err(ane->dev, "rtkit: coprocessor crashed (crashlog %zu bytes)\n",
		size);
	print_hex_dump(KERN_ERR, "ANE crashlog: ", DUMP_PREFIX_OFFSET, 16, 1,
		       crashlog, min_t(size_t, size, 256), false);
}

static int ane_rtclient_shmem_setup(void *cookie,
				    struct apple_rtkit_shmem *bfr)
{
	struct ane_rtclient *ane = cookie;

	if (bfr->iova) {
		dev_warn(ane->dev,
			 "rtkit: fw-provided shmem iova=%pad size=%#zx — refused\n",
			 &bfr->iova, bfr->size);
		return -EINVAL;
	}
	bfr->buffer = dma_alloc_coherent(ane->dev, bfr->size, &bfr->iova,
					 GFP_KERNEL);
	if (!bfr->buffer)
		return -ENOMEM;
	if (ane->fw && !ane_t6021_fw_alias_iova_ok(ane->fw, bfr->iova,
						   bfr->size)) {
		dev_err(ane->dev,
			"rtkit: shmem grant %pad+%#zx overlaps the fw alias — refusing\n",
			&bfr->iova, bfr->size);
		dma_free_coherent(ane->dev, bfr->size, bfr->buffer, bfr->iova);
		bfr->buffer = NULL;
		return -EBUSY;
	}
	return 0;
}

static void ane_rtclient_shmem_destroy(void *cookie,
				       struct apple_rtkit_shmem *bfr)
{
	struct ane_rtclient *ane = cookie;

	if (!bfr->buffer)
		return;
	if (ane->held) {
		dev_warn(ane->dev,
			 "rtkit: shmem %pad HELD (CPU started) — not freed\n",
			 &bfr->iova);
		return;
	}
	dma_free_coherent(ane->dev, bfr->size, bfr->buffer, bfr->iova);
}

static const struct apple_rtkit_ops ane_rtclient_rtkit_ops = {
	.crashed = ane_rtclient_crashed,
	.recv_message = ane_rtclient_recv,
	.shmem_setup = ane_rtclient_shmem_setup,
	.shmem_destroy = ane_rtclient_shmem_destroy,
};

/* ---- poll worker: RX fallback, the recv line is not relied on ----
 * Runs every 10 ms until the handshake completes,
 * then every second while poll_rx stays on. Armed BEFORE the host ack
 * so the fw's HELLO is never missed.
 */
static void ane_rtclient_post_boot(struct work_struct *w)
{
	struct ane_rtclient *ane =
		container_of(to_delayed_work(w), struct ane_rtclient,
			     poll_work);

	apple_rtkit_poll(ane->rtk);

	if (!ane->boot_done) {
		schedule_delayed_work(&ane->poll_work, msecs_to_jiffies(10));
		return;
	}

	if (poll_rx)
		schedule_delayed_work(&ane->poll_work, HZ);
}

/* STARTEP every fw-announced app endpoint (>= 0x20; the fw mgmt
 * dispatcher starts an endpoint on flag bit 1). Same call every Asahi
 * RTKit client makes.
 */
static void ane_rtclient_start_app_eps(struct ane_rtclient *ane)
{
	int ep;

	for (ep = 0x20; ep < 0x100; ep++) {
		int ret;

		if (!apple_rtkit_has_endpoint(ane->rtk, ep))
			continue;
		ret = apple_rtkit_start_ep(ane->rtk, ep);
		dev_dbg(ane->dev, "rtkit: STARTEP app ep %#x -> %pe\n",
			ep, ERR_PTR(ret));
	}
}

/* Multi-domain genpd attach, ownership-correct and idempotent. */
struct ane_rtclient_pd {
	struct list_head list;
	struct device *dev;
	struct device **pd_dev;
	struct device_link **pd_link;
	int count;

	/* Notifier on power-domains[0] (ane_sys_cpu): counts its power-offs
	 * and keeps the T8112 power gate in step with it.
	 */
	struct notifier_block pwgate_nb;
	void __iomem *pwgate_base;
	void __iomem *pwgate;
	bool pwgate_nb_added;
	bool pwgate_opened;
	unsigned int cpu_off_count;
};

static LIST_HEAD(ane_rtclient_pd_list);
static DEFINE_MUTEX(ane_rtclient_pd_lock);
static bool ane_rtclient_pinned;
static bool ane_rtclient_probe_failed_dirty;
static DEFINE_MUTEX(ane_rtclient_probe_guard_lock);
static struct ane_rtclient *ane_rtclient_active;

static void ane_rtclient_pd_free(struct ane_rtclient_pd *pd)
{
	list_del(&pd->list);
	if (pd->pwgate_base)
		iounmap(pd->pwgate_base);
	kfree(pd->pd_dev);
	kfree(pd->pd_link);
	kfree(pd);
}

/*
 * T8112 power gate. A word in the "set" window gates the ANE: bits 29:28
 * set (the word reads 0x30000000) is closed, both clear is open. It must be
 * open before the ANE CPU power state (ane_sys_cpu) turns on and is closed
 * again after that state turns off, with ane_sys, the parent, on both
 * times. power-domains[0] is ane_sys_cpu, and genpd calls this notifier
 * after ane_sys is on and before ane_sys_cpu is written (PRE_ON), and
 * after ane_sys_cpu is off and before ane_sys goes off (OFF), which is that
 * order. Only the two known values are written. Any other word fails the
 * power-up without a write.
 */
#define ANE_PWGATE_CLOSED	GENMASK(29, 28)
#define ANE_PWGATE_POLL_US	10
#define ANE_PWGATE_TIMEOUT_US	100000

static int ane_rtclient_pwgate_notify(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	struct ane_rtclient_pd *pd =
		container_of(nb, struct ane_rtclient_pd, pwgate_nb);
	u32 gate;
	int ret;

	if (action == GENPD_NOTIFY_OFF)
		WRITE_ONCE(pd->cpu_off_count, pd->cpu_off_count + 1);
	if (!pd->pwgate)
		return action == GENPD_NOTIFY_OFF ||
		       action == GENPD_NOTIFY_PRE_ON ? NOTIFY_OK : NOTIFY_DONE;

	switch (action) {
	case GENPD_NOTIFY_PRE_ON:
		gate = readl(pd->pwgate);
		if (!(gate & ANE_PWGATE_CLOSED))
			return NOTIFY_OK;
		if (gate != ANE_PWGATE_CLOSED) {
			dev_err(pd->dev, "power gate word %#x is not a known state\n",
				gate);
			return notifier_from_errno(-EIO);
		}
		writel(0, pd->pwgate);
		ret = readl_poll_timeout_atomic(pd->pwgate, gate,
						!(gate & ANE_PWGATE_CLOSED),
						ANE_PWGATE_POLL_US,
						ANE_PWGATE_TIMEOUT_US);
		if (ret) {
			writel(ANE_PWGATE_CLOSED, pd->pwgate);
			dev_err(pd->dev, "power gate did not open (word %#x)\n",
				gate);
			return notifier_from_errno(ret);
		}
		pd->pwgate_opened = true;
		return NOTIFY_OK;
	case GENPD_NOTIFY_OFF:
		if (!pd->pwgate_opened)
			return NOTIFY_OK;
		pd->pwgate_opened = false;
		writel(ANE_PWGATE_CLOSED, pd->pwgate);
		ret = readl_poll_timeout_atomic(pd->pwgate, gate,
						(gate & ANE_PWGATE_CLOSED) ==
						ANE_PWGATE_CLOSED,
						ANE_PWGATE_POLL_US,
						ANE_PWGATE_TIMEOUT_US);
		if (ret)
			dev_warn(pd->dev, "power gate did not close (word %#x)\n",
				 gate);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

/*
 * Register the power-domains[0] notifier before its device link is created:
 * the link resumes the domain at once. Every SoC on the legacy restart path
 * gets it (system resume needs the power-off count); only T8112 also has
 * the power gate.
 */
static int ane_rtclient_pwgate_add(struct ane_rtclient *ane,
				   struct ane_rtclient_pd *pd)
{
	int ret;

	if ((!ane->soc->pwgate_off && ane->soc->platform) || pd->pwgate_nb_added)
		return 0;
	if (ane->soc->pwgate_off && !pd->pwgate_base) {
		pd->pwgate_base = of_iomap(ane->dev->of_node, 2);
		if (!pd->pwgate_base)
			return -ENOMEM;
		pd->pwgate = pd->pwgate_base + ane->soc->pwgate_off;
	}
	pd->pwgate_nb.notifier_call = ane_rtclient_pwgate_notify;
	ret = dev_pm_genpd_add_notifier(pd->pd_dev[0], &pd->pwgate_nb);
	/* Without the count, system resume trusts the PM core (the gate is
	 * required, the count only makes resume stricter).
	 */
	if (ret)
		return ane->soc->pwgate_off ? ret : 0;
	pd->pwgate_nb_added = true;
	return 0;
}

static int ane_rtclient_attach_genpd(struct ane_rtclient *ane)
{
	struct device *dev = ane->dev;
	struct ane_rtclient_pd *pd = NULL;
	int count, i, err = 0;

	count = of_count_phandle_with_args(dev->of_node, "power-domains",
					   "#power-domain-cells");
	if (count == -ENOENT)
		return 0;
	if (count < 0)
		return count;
	if (count <= 1)
		return 0;

	mutex_lock(&ane_rtclient_pd_lock);
	list_for_each_entry(pd, &ane_rtclient_pd_list, list)
		if (pd->dev == dev)
			goto found;

	pd = kzalloc_obj(*pd);
	if (!pd) {
		err = -ENOMEM;
		goto out;
	}
	INIT_LIST_HEAD(&pd->list);
	pd->dev = dev;
	pd->pd_dev = kcalloc(count, sizeof(*pd->pd_dev), GFP_KERNEL);
	pd->pd_link = kcalloc(count, sizeof(*pd->pd_link), GFP_KERNEL);
	if (!pd->pd_dev || !pd->pd_link) {
		ane_rtclient_pd_free(pd);
		pd = NULL;
		err = -ENOMEM;
		goto out;
	}
	list_add_tail(&pd->list, &ane_rtclient_pd_list);

found:
	for (i = pd->count; i < count; i++) {
		if (!pd->pd_dev[i]) {
			pd->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
			if (IS_ERR_OR_NULL(pd->pd_dev[i])) {
				err = IS_ERR(pd->pd_dev[i]) ?
				      PTR_ERR(pd->pd_dev[i]) : -ENODEV;
				pd->pd_dev[i] = NULL;
				goto out;
			}
		}
		if (i == 0) {
			err = ane_rtclient_pwgate_add(ane, pd);
			if (err)
				goto out;
		}
		if (!pd->pd_link[i]) {
			if (!ane_rtclient_pinned) {
				if (!try_module_get(THIS_MODULE)) {
					err = -ENODEV;
					goto out;
				}
				ane_rtclient_pinned = true;
			}
			pd->pd_link[i] =
				device_link_add(dev, pd->pd_dev[i],
						DL_FLAG_STATELESS |
						DL_FLAG_PM_RUNTIME |
						DL_FLAG_RPM_ACTIVE);
			if (!pd->pd_link[i]) {
				err = -EINVAL;
				goto out;
			}
		}
		pd->count = i + 1;
	}

	dev_dbg(dev, "BOOT-PHASE genpd domains attached: %d\n", count);
out:
	mutex_unlock(&ane_rtclient_pd_lock);
	return err;
}

static void ane_rtclient_detach_genpd(struct device *dev)
{
	struct ane_rtclient_pd *pd, *tmp;
	int count, i;

	count = of_count_phandle_with_args(dev->of_node, "power-domains",
					   "#power-domain-cells");
	if (count <= 1)
		return;

	mutex_lock(&ane_rtclient_pd_lock);
	list_for_each_entry_safe(pd, tmp, &ane_rtclient_pd_list, list) {
		if (pd->dev != dev)
			continue;
		for (i = count - 1; i >= 0; i--) {
			if (pd->pd_link[i])
				device_link_del(pd->pd_link[i]);
			/*
			 * genpd keeps a notifier past detach, so drop it first.
			 * The domain may then go off with the gate open, the
			 * state between the two power-up steps.
			 */
			if (i == 0 && pd->pwgate_nb_added) {
				dev_pm_genpd_remove_notifier(pd->pd_dev[0]);
				pd->pwgate_nb_added = false;
			}
			if (pd->pd_dev[i])
				dev_pm_domain_detach(pd->pd_dev[i], true);
		}
		ane_rtclient_pd_free(pd);
		if (list_empty(&ane_rtclient_pd_list) && ane_rtclient_pinned) {
			module_put(THIS_MODULE);
			ane_rtclient_pinned = false;
		}
		break;
	}
	mutex_unlock(&ane_rtclient_pd_lock);
}

struct ane_rtclient_cycle {
	struct ane_rtclient *ane;
	struct ane_rtclient_pd *pd;
};

static int ane_rtclient_engine_accessible(struct ane_rtclient *ane)
{
	if (FIELD_GET(ANE_PS_ACTUAL,
		      readl(ane->pmgr + ane->soc->ps_cpu_off)) != ANE_PS_ON)
		return -EIO;
	if (ane->soc->pwgate_off &&
	    (IS_ERR_OR_NULL(ane->set) ||
	     readl(ane->set + ane->soc->pwgate_off) & GENMASK(29, 28)))
		return -EACCES;
	return 0;
}

static bool ane_rtclient_domains_off(struct ane_rtclient_cycle *cycle)
{
	struct ane_rtclient *ane = cycle->ane;

	if (readl(ane->pmgr + ane->soc->ps_cpu_off) & ANE_PS_ACTUAL)
		return false;
	/* Firmware controls these seven PMGR words itself. Logical genpd
	 * state alone cannot prove they stayed off during CPU shutdown.
	 */
	for (int i = 0; i < 7; i++)
		if (readl(ane->pmgr + ane->engine_ps_off + i * 8) & ANE_PS_ACTUAL)
			return false;
	for (int i = 0; i < cycle->pd->count; i++)
		if (dev_pm_genpd_is_on(cycle->pd->pd_dev[i]))
			return false;
	return true;
}

static int ane_rtclient_verify_off(void *ctx)
{
	struct ane_rtclient_cycle *cycle = ctx;
	bool off;

	return read_poll_timeout(ane_rtclient_domains_off, off, off,
				100, 100000, false, cycle);
}

static void ane_rtclient_reap_retired(struct ane_rtclient *ane)
{
	struct ane_rtclient_retired *r, *next;

	list_for_each_entry_safe(r, next, &ane->retired, node) {
		for (u32 i = 0; i < r->allocated; i++)
			if (r->buffers[i].cpu)
				dma_free_coherent(ane->dev, r->buffers[i].size,
						  r->buffers[i].cpu, r->buffers[i].dma);
		devm_kfree(ane->dev, r->buffers);
		if (r->boot.pool)
			dma_free_coherent(ane->dev, ANE_T6021_BOOT_POOL_SIZE,
					  r->boot.pool, r->boot.pool_dva);
		if (r->boot.ipc)
			dma_free_coherent(ane->dev, r->boot.ipc_size,
					  r->boot.ipc, r->boot.ipc_dva);
		if (r->boot.heap)
			dma_free_coherent(ane->dev, r->boot.heap_size,
					  r->boot.heap, r->boot.heap_dva);
		list_del(&r->node);
		kfree(r);
	}
	ane->retired_bytes = 0;
	ane->retired_allocated = 0;
	ane->retired_count = 0;
}

#define ANE_RTCLIENT_MAX_DARTS	4

struct ane_rtclient_restart {
	struct ane_rtclient *ane;
	struct ane_rtclient_cycle cycle;
	struct ane_rtclient_retired *retired;
	struct ane_legacy_buffer *fresh;
	bool live;		/* POWER_DOWN never reached the firmware */
	bool unplanned;		/* timeout recovery: the firmware cannot answer */
	bool busy;		/* core never stopped; domains contain it */
	/* DARTs held powered while a busy core is powered off. */
	struct device *dart[ANE_RTCLIENT_MAX_DARTS];
	unsigned int darts;
	/* System sleep: ane_sys_cpu power-off count when the ASC stopped. */
	bool cpu_off_known;
	unsigned int cpu_off_mark;
};

/*
 * Post a command that never completes (POWER_DOWN). Same slot rules as
 * ane_rtclient_legacy_exchange(): only a host-owned slot, and every request
 * already on the SHAREDMALLOC ring is answered first, because a core in
 * WFI cannot finish that handshake. The completion field carries a mark the
 * dispatcher clears when it takes the command.
 */
static int ane_rtclient_legacy_post(struct ane_rtclient *ane,
				    struct ane_legacy_buffer *command,
				    size_t length, u16 opcode,
				    unsigned int channel)
{
	u64 *io;
	int ret;

	if (!ane->held || !ane_rtclient_channels_unchanged(ane) ||
	    ane->soc->platform)
		return -EPROTO;
	if (length < 8 || length > command->size)
		return -EINVAL;
	io = ane->fw->boot_ipc + ane->channels[channel].offset +
	     (size_t)ane->legacy_cmd_cursor[channel] * 64;
	if (!(READ_ONCE(io[0]) & 1))
		return -EBUSY;
	ret = ane_rtclient_malloc_drain(ane);
	if (ret)
		return ret;
	ane_fw_command_header(command->cpu, opcode, false);
	put_unaligned_le16(ANE_T6021_DISPATCH_MARK, (u8 *)command->cpu + 6);
	WRITE_ONCE(io[1], length);
	WRITE_ONCE(io[2], length);
	dma_wmb();
	WRITE_ONCE(io[0], command->dma);
	dma_wmb();
	writel(BIT(ane->channels[channel].bit), ane_rtclient_doorbell(ane));
	return 0;
}

static u16 ane_rtclient_cmd_mark(struct ane_legacy_buffer *command)
{
	return le16_to_cpu(READ_ONCE(*(__le16 *)((u8 *)command->cpu + 6)));
}

/* Ask the firmware to shut down and wait until its dispatcher took the
 * command. Only a live, idle instance can take it: after a timeout the
 * command worker is blocked and nothing can overtake the stuck command, so
 * this is never tried then.
 */
static int ane_rtclient_rs_power_down(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	u16 mark;
	int ret;

	if (!ane->cmd_buf)
		return -ENODEV;
	memset(ane->cmd_buf->cpu, 0, SZ_16K);
	ret = ane_rtclient_legacy_post(ane, ane->cmd_buf,
				       ANE_T6021_POWER_DOWN_LEN,
				       ANE_T6021_CMD_POWER_DOWN, 1);
	/* Nothing was posted: the instance keeps serving. */
	rs->live = ret == -EINVAL;
	if (!ret)
		ret = read_poll_timeout(ane_rtclient_cmd_mark, mark, !mark, 50,
					100000, false, ane->cmd_buf);
	dev_info(ane->dev, "restart: POWER_DOWN %s %d (CPU_STATUS %#x)\n",
		 ret ? "not taken" : "taken", ret,
		 readl(ane->engine + ANE_ASC_CPU_STATUS));
	return ret;
}

/* The shutdown path ends in WFI. IDLE is also set at ordinary idle points,
 * so require it on twenty reads 100 us apart after the command was taken.
 */
static int ane_rtclient_rs_wait_idle(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	void __iomem *st = rs->ane->engine + ANE_ASC_CPU_STATUS;
	unsigned long until = jiffies + msecs_to_jiffies(100);
	unsigned int run = 0;

	while (time_before(jiffies, until)) {
		run = readl(st) & ANE_ASC_CPU_STATUS_IDLE ? run + 1 : 0;
		if (run >= 20)
			return 0;
		usleep_range(100, 150);
	}
	return -ETIMEDOUT;
}

/* Clear RUN (read-modify-write, read back), then require STOPPED with RUN
 * clear. Nothing is powered off unless this returns zero.
 */
static int ane_rtclient_rs_halt(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	void __iomem *ctl = ane->engine + ANE_ASC_CPU_CONTROL;
	u32 status;
	int ret;

	ret = ane_rtclient_engine_accessible(ane);
	if (ret)
		return ret;
	writel(readl(ctl) & ~ANE_ASC_CPU_CONTROL_RUN, ctl);
	readl(ctl);
	ret = readl_poll_timeout(ane->engine + ANE_ASC_CPU_STATUS, status,
				 (status & ANE_ASC_CPU_STATUS_STOPPED) &&
				 !(status & ANE_ASC_CPU_STATUS_RUNNING),
				 100, 100000);
	if (!ret && (readl(ctl) & ANE_ASC_CPU_CONTROL_RUN))
		ret = -EBUSY;
	dev_info(ane->dev, "restart: ASC stop %s: CPU_STATUS %#x CPU_CONTROL %#x\n",
		 ret ? "NOT proven" : "proven", status, readl(ctl));
	/* RUN is clear but the core never stopped: a busy core, not a gone one. */
	if (ret == -ETIMEDOUT && !(readl(ctl) & ANE_ASC_CPU_CONTROL_RUN))
		return ANE_T6021_ASC_BUSY;
	return ret;
}

static void ane_rtclient_log_ps(struct ane_rtclient *ane, const char *what)
{
	u32 ps[7] = {};

	if (ane->engine_ps_valid)
		for (int i = 0; i < 7; i++)
			ps[i] = readl(ane->pmgr + ane->engine_ps_off + i * 8);
	dev_info(ane->dev,
		 "%s: PMGR cpu %#x engine %#x %#x %#x %#x %#x %#x %#x\n", what,
		 readl(ane->pmgr + ane->soc->ps_cpu_off), ps[0], ps[1], ps[2],
		 ps[3], ps[4], ps[5], ps[6]);
}

/*
 * A busy core after a timeout (unplanned only): give it hung_settle_ms to
 * stop by itself, then let the power cycle contain it. Only the PMGR power
 * state machines act on it, children first; nothing touches the engine
 * once its domains start to go off.
 */
static int ane_rtclient_rs_contain(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	u32 status;
	int ret;

	if (!ane_t6021_hung_stop || !rs->unplanned)
		return -EOPNOTSUPP;
	ret = ane_rtclient_engine_accessible(ane);
	if (ret)
		return ret;
	ret = readl_poll_timeout(ane->engine + ANE_ASC_CPU_STATUS, status,
				 (status & ANE_ASC_CPU_STATUS_STOPPED) &&
				 !(status & ANE_ASC_CPU_STATUS_RUNNING),
				 1000, min(ane_t6021_hung_settle_ms, 60000U) * 1000ULL);
	if (!ret) {
		dev_info(ane->dev, "restart: ASC stopped late (CPU_STATUS %#x)\n",
			 status);
		return 0;
	}
	ane_rtclient_log_ps(ane, "restart: busy ASC");
	dev_info(ane->dev,
		 "restart: ASC busy after a hung job (CPU_STATUS %#x); powering the ANE off through its domains\n",
		 status);
	rs->busy = true;
	return 0;
}

/*
 * Keep the ANE DARTs (and with them ane_sys, the outer supply) powered while
 * a busy core goes off, so translation stays up for anything still in flight.
 */
static void ane_rtclient_hold_darts(struct ane_rtclient_restart *rs)
{
	struct device_node *np = rs->ane->dev->of_node;
	struct of_phandle_args args;

	for (int i = 0; rs->darts < ANE_RTCLIENT_MAX_DARTS &&
	     !of_parse_phandle_with_args(np, "iommus", "#iommu-cells", i, &args);
	     i++) {
		struct platform_device *pdev = of_find_device_by_node(args.np);

		of_node_put(args.np);
		if (!pdev)
			continue;
		if (pm_runtime_resume_and_get(&pdev->dev) < 0) {
			dev_warn(rs->ane->dev, "restart: %s not held powered\n",
				 dev_name(&pdev->dev));
			put_device(&pdev->dev);
			continue;
		}
		rs->dart[rs->darts++] = &pdev->dev;
	}
}

static void ane_rtclient_release_darts(struct ane_rtclient_restart *rs)
{
	while (rs->darts) {
		struct device *dart = rs->dart[--rs->darts];

		pm_runtime_put(dart);
		put_device(dart);
	}
}

/* Shared post-power-transition admission, including system resume. */
static int ane_rtclient_rs_ready(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	u32 status;
	int ret;

	ret = ane_rtclient_engine_accessible(ane);
	if (ret)
		return ret;
	status = readl(ane->engine + ANE_ASC_CPU_STATUS);
	if ((status & ANE_ASC_CPU_STATUS_RUNNING) ||
	    !(status & ANE_ASC_CPU_STATUS_STOPPED) ||
	    (readl(ane->engine + ANE_ASC_CPU_CONTROL) & ANE_ASC_CPU_CONTROL_RUN))
		return -EBUSY;
	return 0;
}

static int ane_rtclient_rs_power_cycle(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	int ret;

	if (rs->busy)
		ane_rtclient_hold_darts(rs);
	/* Probe retains one consumer reference for this firmware lifetime. */
	ret = ane_pm_cycle(ane->dev, rs->cycle.pd->pd_dev, rs->cycle.pd->count,
			   ane_rtclient_verify_off, &rs->cycle);
	ane_rtclient_release_darts(rs);
	if (rs->busy)
		ane_rtclient_log_ps(ane, ret ? "restart: busy ASC cycle failed" :
				    "restart: busy ASC cycled");
	if (ret)
		return ret;
	/* The cold CPU must read STOPPED with RUN clear before it boots. */
	return ane_rtclient_rs_ready(ctx);
}

static int ane_rtclient_rs_boot(void *ctx)
{
	struct ane_rtclient_restart *rs = ctx;
	struct ane_rtclient *ane = rs->ane;
	struct ane_rtclient_retired *retired = rs->retired;
	struct ane_t6021 *a = ane->fw;
	int ret;

	/* No old allocation is discarded until the new boot is validated. */
	retired->buffers = ane->legacy_buffers;
	retired->allocated = ane->legacy_allocated;
	retired->bytes = ane->legacy_bytes;
	retired->boot = (struct ane_t6021_boot_allocs) {
		.pool = a->boot_pool, .pool_dva = a->boot_pool_iova,
		.ipc = a->boot_ipc, .ipc_dva = a->boot_ipc_iova,
		.ipc_size = a->boot_ipc_size,
		.heap = a->boot_heap, .heap_dva = a->boot_heap_iova,
		.heap_size = a->boot_heap_size,
	};
	list_add_tail(&retired->node, &ane->retired);
	rs->retired = NULL;
	ane->retired_count++;
	ane->retired_bytes += retired->bytes;
	ane->retired_allocated += retired->allocated;
	ane_t6021_boot_release_pin(a);
	a->boot_pool = NULL;
	a->boot_ipc = NULL;
	a->boot_heap = NULL;
	a->boot_pool_iova = 0;
	a->boot_ipc_iova = 0;
	a->boot_heap_iova = 0;
	a->boot_ipc_size = 0;
	a->boot_heap_size = 0;
	ane->legacy_buffers = rs->fresh;
	rs->fresh = NULL;
	ane->legacy_allocated = 0;
	ane->legacy_live = 0;
	ane->legacy_bytes = 0;
	ane->legacy_malloc_cursor = 0;
	memset(ane->legacy_cmd_cursor, 0, sizeof(ane->legacy_cmd_cursor));
	memset(ane->channels, 0, sizeof(ane->channels));
	memset(ane->channel_snapshot, 0, sizeof(ane->channel_snapshot));
	ane->cmd_buf = NULL;
	ane->chman_ok = false;
	ane->held = false;
	fw_perf_mode = false;

	ret = ane_t6021_fwload_reload(a);
	if (ret)
		return ret;
	ret = ane_t6021_boot_start(a, 0, fw_start_table_mode, 0);
	ane->held = a->cpu_started;
	if (ret || !a->fw_alive || !a->booted)
		return ret ?: -ETIMEDOUT;
	ane_rtclient_validate_chman(ane);
	if (!ane->chman_ok ||
	    !ane_t6021_chman_host_init(a->boot_ipc, a->boot_ipc_size,
				       a->boot_ipc_iova))
		return -EPROTO;
	dma_wmb();
	writel(ANE_T6021_BOOT_ACK, ane->engine + ANE_MBI_SCRATCH0 + 4 * 3);
	ret = ane_rtclient_legacy_alloc(ane, SZ_16K);
	if (ret < 0)
		return ret;
	ane->cmd_buf = &ane->legacy_buffers[ret];
	memset(ane->cmd_buf->cpu, 0, SZ_16K);
	/* The firmware lock still fences commands; BO_INIT also checks recovering.
	 * Clearing the old latch lets transport validation detect a new error.
	 */
	atomic_set(&ane_t6021_quarantined, 0);
	ret = ane_rtclient_legacy_exchange(ane, ane->cmd_buf, 16, 0x03, 1,
					   jiffies + msecs_to_jiffies(3000));
	if (!ret && !READ_ONCE(((u32 *)ane->cmd_buf->cpu)[2]))
		ret = -EPROTO;
	if (ret)
		return ret;
	if (ane->desired_perf) {
		ret = ane_rtclient_send_perf_mode(ane, jiffies + msecs_to_jiffies(3000));
		if (ret)
			return ret;
	}
	return atomic_read(&ane_t6021_quarantined) ? -EIO : 0;
}

static const struct ane_t6021_restart_ops ane_rtclient_restart_ops = {
	.power_down = ane_rtclient_rs_power_down,
	.wait_idle = ane_rtclient_rs_wait_idle,
	.halt = ane_rtclient_rs_halt,
	.contain = ane_rtclient_rs_contain,
	.power_cycle = ane_rtclient_rs_power_cycle,
	.boot = ane_rtclient_rs_boot,
};

/* Allocate the lifetime owner's transaction before either PM path stops it.
 * The same transaction survives the system power boundary in sleep_restart.
 */
static int ane_rtclient_rs_prepare(struct ane_rtclient_restart *rs)
{
	struct ane_rtclient *ane = rs->ane;
	struct ane_t6021 *a = ane->fw;
	struct ane_rtclient_pd *pd;
	unsigned int domains = ane->soc->soc == 0x8112 ? 7 : 8;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!ane->registered)
		return -EOPNOTSUPP;
	/* Only the qualified, Linux-owned legacy boot profile can restart.
	 * Borrowed firmware, RTKit and later-generation ownership stay fenced.
	 */
	if (ane->soc->platform || !a || ane->rtk ||
	    !legacy_only || !scratch3_ack || !legacy_query || fw_start_rtb_mode ||
	    fw_start_table_mode != 2 || !ane_t6021_fwload_recoverable(a))
		return -EOPNOTSUPP;
	/* Keep failed-boot retention bounded and charged against existing caps. */
	if (ane->retired_count >= 2 || ane_t6021_bo_epoch == U64_MAX)
		return -ENOSPC;
	mutex_lock(&ane_rtclient_pd_lock);
	list_for_each_entry(pd, &ane_rtclient_pd_list, list)
		if (pd->dev == ane->dev && pd->count == domains)
			rs->cycle.pd = pd;
	mutex_unlock(&ane_rtclient_pd_lock);
	if (!rs->cycle.pd || !ane->engine_ps_valid)
		return -EOPNOTSUPP;
	rs->retired = kzalloc_obj(*rs->retired);
	if (!rs->retired)
		return -ENOMEM;
	rs->fresh = devm_kcalloc(ane->dev, ANE_LEGACY_ALLOCS, sizeof(*rs->fresh),
				 GFP_KERNEL);
	if (!rs->fresh) {
		kfree(rs->retired);
		return -ENOMEM;
	}

	WRITE_ONCE(ane->recovering, true);
	cancel_delayed_work_sync(&ane->poll_work);
	return 0;
}

/* Commit or retain the lifetime owner's transaction in exactly one place. */
static int ane_rtclient_rs_finish(struct ane_rtclient_restart *rs, bool planned,
				  enum ane_t6021_restart_stage stage, int ret)
{
	struct ane_rtclient *ane = rs->ane;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (ret)
		goto failed;
	/* Opaque IDs never alias a new firmware instance; programs files still
	 * hold load again on their next use. Advance the BO epoch only after
	 * old DMA and the new boot are verified.
	 */
	ane_t6021_bo_new_epoch();
	ane_t6021_progtab_restarted(&ane_t6021_progtab, &ane_t6021_progtab_ops,
				    ane);
	ane_rtclient_reap_retired(ane);
	WRITE_ONCE(ane->recovering, false);
	ane->sleep_pm.state = ANE_T6021_PM_ACTIVE;
	dev_info(ane->dev, "firmware %s; programs reload on next use\n",
		 planned ? "restarted" : "recovered");
	return 0;

failed:
	if (rs->fresh)
		devm_kfree(ane->dev, rs->fresh);
	kfree(rs->retired);
	/* A POWER_DOWN that was never posted left the instance in service.
	 * Once posted, the firmware is shut down or about to be.
	 */
	if (!rs->live)
		atomic_set(&ane_t6021_quarantined, 1);
	WRITE_ONCE(ane->recovering, false);
	dev_err(ane->dev,
		"firmware %s failed at %s: %d; DMA backing and any live-CPU pin retained\n",
		planned ? "restart" : "recovery",
		stage == ANE_T6021_RESTART_STOP ? "the ASC stop" :
		stage == ANE_T6021_RESTART_CYCLE ? "the power cycle" : "the boot",
		ret);
	return ret;
}

/*
 * Cold-restart the Linux-owned firmware. @planned: the instance is live and
 * idle (the caller holds the firmware lock, so no command or call is in
 * flight) and is asked to shut down first. Otherwise a command timed out
 * and only the RUN clear is tried. The ASC must be proven stopped before
 * its domains go off; if it is not, nothing changes and the device stays
 * quarantined with every mapping, its power claim and its pin.
 */
static int __ane_rtclient_recover(struct ane_rtclient *ane, bool planned)
{
	struct ane_rtclient_restart rs = {
		.ane = ane, .cycle.ane = ane, .unplanned = !planned,
	};
	enum ane_t6021_restart_stage stage = ANE_T6021_RESTART_STOP;
	int ret;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!ane->registered)
		return -EOPNOTSUPP;
	if (!planned && !atomic_read(&ane_t6021_quarantined))
		return 0;
	ret = ane_rtclient_rs_prepare(&rs);
	if (ret)
		return ret;
	ret = ane_t6021_restart_run(&ane_rtclient_restart_ops, &rs, planned, &stage);
	return ane_rtclient_rs_finish(&rs, planned, stage, ret);
}

static int ane_rtclient_recover(struct ane_rtclient *ane)
{
	return __ane_rtclient_recover(ane,
		!atomic_read(&ane_t6021_quarantined) &&
		ane->sleep_pm.state == ANE_T6021_PM_FAILED);
}

/*
 * The restart for room: every resident program is HELD (its load or unload
 * outcome is unknown), so none can be given back, and a new one must load.
 */
static int ane_rtclient_restart(struct ane_rtclient *ane)
{
	int ret;

	if (atomic_read(&ane_t6021_quarantined))
		return -EIO;
	dev_info(ane->dev, "restart: %u programs resident, none can be unloaded\n",
		 ane_t6021_progtab_resident(&ane_t6021_progtab));
	ret = __ane_rtclient_recover(ane, true);
	return ret == -EOPNOTSUPP || ret == -ENOSPC ? -ENOSPC : ret;
}

/* System PM owns the supplier transition and holds a consumer reference.
 * Reuse the lifetime stop/boot/commit boundaries without invoking the
 * runtime power-cycle transaction. Runtime PM is disabled in the late/noirq
 * phases, after this stop and before the normal resume callback.
 */
static int ane_rtclient_sleep_stop(void *ctx)
{
	struct ane_rtclient *ane = ctx;
	struct ane_rtclient_restart *rs;
	bool planned = !atomic_read(&ane_t6021_quarantined);
	int ret;

	lockdep_assert_held(&ane_t6021_fw_lock);
	/* A quarantined instance (its recovery failed) cannot shut down;
	 * with hung_stop its busy core goes off with the sleep instead.
	 */
	if ((!planned && !ane_t6021_hung_stop) || ane->sleep_restart)
		return -EBUSY;
	rs = kzalloc_obj(*rs);
	if (!rs)
		return -ENOMEM;
	rs->ane = ane;
	rs->cycle.ane = ane;
	rs->unplanned = !planned;
	ret = ane_rtclient_rs_prepare(rs);
	if (ret)
		goto free;
	ret = ane_t6021_restart_stop(&ane_rtclient_restart_ops, rs, planned);
	if (ret) {
		ane_rtclient_rs_finish(rs, true, ANE_T6021_RESTART_STOP, ret);
		goto free;
	}
	rs->cpu_off_known = rs->cycle.pd->pwgate_nb_added;
	rs->cpu_off_mark = READ_ONCE(rs->cycle.pd->cpu_off_count);
	ane->sleep_restart = rs;
	ane->sleep_deferred = false;
	ane->sleep_cycle = false;
	return 0;
free:
	kfree(rs);
	return ret;
}

/*
 * System resume. The firmware boot takes about 1.2 s on a T8112, so it never
 * runs here: resume only queues the work below, and the first client that
 * beats the work finishes the restart itself (ane_rtclient_admit()). Until
 * then submissions are held off as for any SUSPENDED device. Probe holds a
 * runtime PM reference for the firmware lifetime, so the domains stay up
 * between resume and the work.
 *
 * The stopped ASC may only cold-start after its CPU domain went off. A
 * suspend that aborted before genpd powered the domains off (a wake event
 * during the device phase) resumes a CPU that was only halted; booting it
 * then fails and leaves the firmware half-started. The work power-cycles
 * that CPU first.
 */
static int ane_rtclient_sleep_restart(void *ctx)
{
	struct ane_rtclient *ane = ctx;
	struct ane_rtclient_restart *rs = ane->sleep_restart;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!rs)
		return -EIO;
	if (rs->cpu_off_known &&
	    READ_ONCE(rs->cycle.pd->cpu_off_count) == rs->cpu_off_mark) {
		if (!ane->sleep_cycle)
			dev_info(ane->dev,
				 "resume: CPU domain stayed on through the sleep; restart after a power cycle\n");
		ane->sleep_cycle = true;
	}
	WRITE_ONCE(ane->sleep_deferred, true);
	queue_work(system_freezable_wq, &ane->sleep_work);
	return ANE_T6021_PM_RESTART_DEFERRED;
}

/* The deferred half: runtime power cycle (verified off) if needed, then boot. */
static int ane_rtclient_sleep_restart_deferred(void *ctx)
{
	struct ane_rtclient *ane = ctx;
	struct ane_rtclient_restart *rs = ane->sleep_restart;
	enum ane_t6021_restart_stage stage = ANE_T6021_RESTART_CYCLE;
	bool cycle = ane->sleep_cycle;
	int ret;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!rs)
		return -EIO;
	ane->sleep_restart = NULL;
	ane->sleep_cycle = false;
	ret = cycle ? ane_rtclient_rs_power_cycle(rs) : ane_rtclient_rs_ready(rs);
	if (!ret) {
		stage = ANE_T6021_RESTART_BOOT;
		ret = ane_rtclient_rs_boot(rs);
	}
	ret = ane_rtclient_rs_finish(rs, true, stage, ret);
	kfree(rs);
	/* Cleared only now: wedged_show() reads it without the lock. */
	WRITE_ONCE(ane->sleep_deferred, false);
	return ret;
}

/*
 * A FAILED device is suspended: the engine stays out until reboot, but its
 * CPU may only lose power once it is proven stopped (or already unpowered).
 * Clear RUN and require STOPPED, as the restart's halt does. If that cannot
 * be proven only this suspend is refused.
 */
static int ane_rtclient_sleep_park(void *ctx)
{
	struct ane_rtclient *ane = ctx;
	void __iomem *ctl = ane->engine + ANE_ASC_CPU_CONTROL;
	u32 status;
	int ret;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!ane->registered || ane_rtclient_engine_accessible(ane))
		return 0;
	/* A halted instance cannot take POWER_DOWN: recovery must cycle. */
	atomic_set(&ane_t6021_quarantined, 1);
	status = readl(ane->engine + ANE_ASC_CPU_STATUS);
	if ((status & ANE_ASC_CPU_STATUS_STOPPED) &&
	    !(status & ANE_ASC_CPU_STATUS_RUNNING) &&
	    !(readl(ctl) & ANE_ASC_CPU_CONTROL_RUN))
		return 0;
	writel(readl(ctl) & ~ANE_ASC_CPU_CONTROL_RUN, ctl);
	readl(ctl);
	ret = readl_poll_timeout(ane->engine + ANE_ASC_CPU_STATUS, status,
				 (status & ANE_ASC_CPU_STATUS_STOPPED) &&
				 !(status & ANE_ASC_CPU_STATUS_RUNNING),
				 100, 100000);
	if (!ret && (readl(ctl) & ANE_ASC_CPU_CONTROL_RUN))
		ret = -EBUSY;
	/* A busy core (RUN clear, never STOPPED) is contained by the same
	 * domain power-off the sleep performs anyway.
	 */
	if (ret == -ETIMEDOUT && ane_t6021_hung_stop && !(readl(ctl) & ANE_ASC_CPU_CONTROL_RUN)) {
		ane_rtclient_log_ps(ane, "suspend while unavailable: busy ASC");
		dev_info(ane->dev,
			 "suspend while unavailable: ASC busy (CPU_STATUS %#x); its domains go off with the sleep\n",
			 status);
		return 0;
	}
	dev_info(ane->dev, "suspend while unavailable: ASC stop %s (CPU_STATUS %#x)\n",
		 ret ? "NOT proven, sleep refused" : "proven", status);
	return ret;
}

static const struct ane_t6021_pm_ops ane_rtclient_sleep_ops = {
	.stop = ane_rtclient_sleep_stop,
	.restart = ane_rtclient_sleep_restart,
	.restart_deferred = ane_rtclient_sleep_restart_deferred,
	.park = ane_rtclient_sleep_park,
};

/* Finish a deferred sleep restart. Caller holds the firmware mutex. */
static void ane_rtclient_sleep_finish(struct ane_rtclient *ane)
{
	int ret;

	lockdep_assert_held(&ane_t6021_fw_lock);
	if (!ane->sleep_deferred)
		return;
	ret = ane_t6021_pm_restart_deferred(&ane->sleep_pm);
	if (ret)
		dev_err(ane->dev, "deferred restart after resume failed: %d\n", ret);
}

/* Freezable: never runs between a suspend and its resume. */
static void ane_rtclient_sleep_work(struct work_struct *work)
{
	struct ane_rtclient *ane =
		container_of(work, struct ane_rtclient, sleep_work);

	mutex_lock(&ane_t6021_fw_lock);
	ane_rtclient_sleep_finish(ane);
	mutex_unlock(&ane_t6021_fw_lock);
}

/* Submission admission: a client that beats the work finishes the restart. */
static int ane_rtclient_admit(struct ane_rtclient *ane)
{
	ane_rtclient_sleep_finish(ane);
	return ane_t6021_pm_submit_error(ane->sleep_pm.state);
}

static struct ane_t6021_pm *ane_rtclient_get_sleep_pm(struct device *dev)
{
	struct ane_rtclient *ane = dev_get_drvdata(dev);

	return &ane->sleep_pm;
}

/*
 * One platform driver binds the H13/H14 rtclient and, through the shared
 * firmware platform, the H16 backend, each with its own drvdata. Dispatch by
 * generation: the rtclient sleep path must never see an H16 device.
 */
static bool ane_rtclient_is_platform(struct device *dev)
{
	const struct ane_t602x_soc *soc = of_device_get_match_data(dev);

	return soc && ane_fw_sleep_path(soc->platform) == ANE_SLEEP_FW_BACKEND;
}

static int __maybe_unused ane_rtclient_sleep_pm_suspend(struct device *dev)
{
	if (ane_rtclient_is_platform(dev))
		return ane_fw_platform_suspend(dev);
	return ane_t6021_pm_suspend(ane_rtclient_get_sleep_pm(dev));
}

static int __maybe_unused ane_rtclient_sleep_pm_resume(struct device *dev)
{
	if (ane_rtclient_is_platform(dev))
		return ane_fw_platform_resume(dev);
	return ane_t6021_pm_resume(ane_rtclient_get_sleep_pm(dev));
}

static const struct dev_pm_ops __maybe_unused ane_rtclient_sleep_pm = {
	SYSTEM_SLEEP_PM_OPS(ane_rtclient_sleep_pm_suspend, ane_rtclient_sleep_pm_resume)
};

static int ane_rtclient_probe_inner(struct platform_device *pdev,
				    bool *hardware_touched)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct ane_rtclient *ane;
	struct ane_t6021 *a;
	u32 cpu_status, ps_cpu;
	u64 rvbar;
	int ret;

	if (!ane_t6021_fwload_options_ok()) {
		dev_err(dev,
			"invalid firmware RAM-grant options; refusing before power access\n");
		return -EINVAL;
	}

	if (!ane_t6021_fwload_placement_ok(dev)) {
		dev_err(dev,
			"fw_alias_reserved=1, but no no-map /reserved-memory node covers the iBoot firmware windows (this aura does not reserve them); refusing before power access, ANE off\n");
		return -ENODEV;
	}

	/* Early, BEFORE any allocation/power: legacy_only must never be
	 * rejected after the CPU release, where an unwind could drop
	 * domains under a running ASC.
	 */
	if (legacy_only && (!fw_start || fw_start_rtb_mode)) {
		dev_err(dev,
			"legacy_only=1 requires fw_start=1 and fw_start_rtb_mode=0\n");
		return -EINVAL;
	}
	if (legacy_query && !legacy_only) {
		dev_err(dev, "legacy_query requires legacy_only=1\n");
		return -EINVAL;
	}
	if (fw_start && ane_t6021_fwload_requested()) {
		ret = ane_t6021_fwload_check(dev);
		if (ret)
			return dev_err_probe(dev, ret, "firmware preflight failed\n");
	}

	ane = devm_kzalloc(dev, sizeof(*ane), GFP_KERNEL);
	if (!ane)
		return -ENOMEM;
	ane->legacy_buffers = devm_kcalloc(dev, ANE_LEGACY_ALLOCS,
					   sizeof(*ane->legacy_buffers),
					   GFP_KERNEL);
	if (!ane->legacy_buffers)
		return -ENOMEM;
	ane->dev = dev;
	INIT_LIST_HEAD(&ane->h16_processes);
	ane->soc = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, ane);
	INIT_DELAYED_WORK(&ane->poll_work, ane_rtclient_post_boot);
	INIT_WORK(&ane->sleep_work, ane_rtclient_sleep_work);
	ane->sleep_pm = (struct ane_t6021_pm) {
		.fw_lock = &ane_t6021_fw_lock,
		.ops = &ane_rtclient_sleep_ops,
		.ctx = ane,
		.state = ANE_T6021_PM_ACTIVE,
	};
	INIT_LIST_HEAD(&ane->retired);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	if (!(res->flags & IORESOURCE_MEM_NONPOSTED))
		dev_warn(dev, "engine window is not flagged non-posted\n");
	ane->engine = ioremap_np(res->start, resource_size(res));
	if (!ane->engine)
		return -ENOMEM;
	ane->cpu_rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(ane->cpu_rst))
		return dev_err_probe(dev, PTR_ERR(ane->cpu_rst),
				     "ane_cpu reset control\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	*hardware_touched = true;
	ret = ane_rtclient_attach_genpd(ane);
	if (ret)
		return dev_err_probe(dev, ret, "extra genpd attach\n");
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret, "genpd raise failed\n");
	}

	ane->pmgr = devm_of_iomap(dev, dev->of_node, 1, NULL);
	if (IS_ERR(ane->pmgr)) {
		ret = PTR_ERR(ane->pmgr);
		pm_runtime_put_sync_suspend(dev);
		pm_runtime_disable(dev);
		return dev_err_probe(dev, ret,
				     "pmgr window map failed; G1 gate cannot run\n");
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		u64 first = ane->soc->pmu_pa + ane->soc->ps_off;

		if (first >= res->start && first <= res->end &&
		    res->end - first >= 6 * 8 + sizeof(u32) - 1) {
			ane->engine_ps_off = first - res->start;
			ane->engine_ps_valid = true;
		}
	}
	ps_cpu = readl(ane->pmgr + ane->soc->ps_cpu_off);
	dev_dbg(dev, "ane_cpu ACTUAL = 0x%x\n", ps_cpu);
	if (FIELD_GET(ANE_PS_ACTUAL, ps_cpu) != ANE_PS_ON) {
		pm_runtime_put_sync_suspend(dev);
		pm_runtime_disable(dev);
		return -EPROBE_DEFER;
	}
	/* T8112: the power gate notifier opened the gate before ane_sys_cpu
	 * turned on. A gate that still reads closed means the state was
	 * already on without it (no power-up ran), so refuse before any
	 * engine read.
	 */
	if (ane->soc->pwgate_off) {
		void __iomem *set = devm_of_iomap(dev, dev->of_node, 2, NULL);

		u32 gate = IS_ERR(set) ? U32_MAX :
			   readl(set + ane->soc->pwgate_off);

		ane->set = set;

		dev_dbg(dev, "PWGATE = 0x%x\n", gate);
		if (gate & GENMASK(29, 28)) {
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return dev_err_probe(dev, -ENODEV,
					     "PWGATE closed; refusing before any engine read\n");
		}
	}

	cpu_status = readl(ane->engine + ANE_ASC_CPU_STATUS);
	rvbar = readq(ane->engine + ANE_ASC_RVBAR);
	dev_dbg(dev,
		"BOOT-PHASE engine reads ok: CPU_STATUS = 0x%x, RVBAR = %016llx (bit0=%u)\n",
		cpu_status, rvbar, (u32)(rvbar & 1));

	if (!(cpu_status & ANE_ASC_CPU_STATUS_RUNNING)) {
		if (!fw_start) {
			dev_err(dev,
				"ANE firmware not alive (CPU_STATUS 0x%x) — start it from a quiesce context, or retry with fw_start=1\n",
				cpu_status);
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return -EPROBE_DEFER;
		}

		/* fw_start order: staging requires fw_load and an
		 * IOMMU-mapped device BEFORE any allocation/staging.
		 */
		if (!ane_t6021_fwload_requested()) {
			dev_err(dev,
				"fw_start: requires fw_load=1 (no staged firmware)\n");
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return -EINVAL;
		}
		if (!device_iommu_mapped(dev)) {
			dev_err(dev,
				"fw_start: device not IOMMU-mapped — a staged DVA/entry alias would be untranslated; refusing\n");
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return -EINVAL;
		}

		a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
		if (!a) {
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return -ENOMEM;
		}
		a->dev = dev;
		a->base[ANE_T6021_REG_ENGINE] = ane->engine;
		a->irq = -1;
		a->power_gated = true;
		ane->fw = a;

		dev_dbg(dev, "BOOT-PHASE fwload stage+alias begin\n");
		ret = ane_t6021_fwload_probe(a);
		if (ret) {
			dev_err_probe(dev, ret, "fw_start: staging failed\n");
			ane_t6021_fwload_remove(a);
			ane->fw = NULL;
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return ret;
		}
		if (!ane_t6021_rvbar_entry_ok(a->fw_iova)) {
			dev_err(dev, "fw_start: staged iova %pad invalid\n",
				&a->fw_iova);
			ane_t6021_fwload_remove(a);
			ane->fw = NULL;
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return -EINVAL;
		}

		dev_dbg(dev, "BOOT-PHASE dispatch (table_mode=%d)\n",
			fw_start_table_mode);
		ret = ane_t6021_boot_start(a, 0, fw_start_table_mode,
					   fw_start_rtb_mode);
		if (ret == -ENODATA || ret == -EAGAIN ||
		    ret == -EBUSY || ret == -ECANCELED) {
			ane_t6021_fwload_remove(a);
			ane->fw = NULL;
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
			return ret;
		}

		ane->held = true;
		cpu_status = readl(ane->engine + ANE_ASC_CPU_STATUS);
		dev_info(dev,
			 "BOOT-PHASE sequence returned %pe (cpu_started=%u fw_alive=%u booted=%u) CPU_STATUS=0x%x\n",
			 ERR_PTR(ret), a->cpu_started, a->fw_alive, a->booted,
			 cpu_status);
		if (!a->fw_alive && !fw_start_rtb_mode) {
			dev_err(dev,
				"fw_start: poll A timeout, no READY — HELD until reboot, RTKit handshake skipped\n");
			return 0;
		}
	}

	if (legacy_only) {
		int cfg_err = 0;

		ane_rtclient_validate_chman(ane);
		if (ane->fw && ane->fw->booted && scratch3_ack &&
		    ane->chman_ok &&
		    ane_t6021_chman_host_init(ane->fw->boot_ipc,
					      ane->fw->boot_ipc_size,
					     ane->fw->boot_ipc_iova)) {
			int hello_ret = 0;

			dma_wmb();
			dev_dbg(dev,
				"LEGACY P8 host ack: SCRATCH3 <- %08x\n",
				ANE_T6021_BOOT_ACK);
			/* hello_wait_ms > 0 only: init rtkit and arm the RX
			 * poll worker BEFORE writing the ack, so a HELLO
			 * after the ack is not missed. The 13.5 fw sends none.
			 */
			if (hello_wait_ms && !ane->rtk) {
				ane->rtk = devm_apple_rtkit_init(dev, ane,
								 NULL, 0,
								&ane_rtclient_rtkit_ops);
				if (IS_ERR(ane->rtk)) {
					dev_err(dev,
						"LEGACY hello: rtkit init %pe\n",
						ane->rtk);
					ane->rtk = NULL;
				} else {
					schedule_delayed_work(&ane->poll_work,
							      msecs_to_jiffies(10));
				}
			}
			writel(ANE_T6021_BOOT_ACK,
			       ane->engine + ANE_MBI_SCRATCH0 + 4 * 3);
			if (ane->rtk) {
				unsigned long hello_deadline =
					jiffies + msecs_to_jiffies(hello_wait_ms);

				dev_dbg(dev, "LEGACY hello: boot begin (%u ms)\n",
					hello_wait_ms);
				do {
					hello_ret = apple_rtkit_boot(ane->rtk);
				} while (hello_ret == -ETIME &&
					 time_before(jiffies, hello_deadline));
				dev_info(dev,
					 "LEGACY hello: boot %pe running=%d crashed=%d\n",
					 ERR_PTR(hello_ret),
					 apple_rtkit_is_running(ane->rtk),
					 apple_rtkit_is_crashed(ane->rtk));
				if (!hello_ret) {
					ane->boot_done = true;
					if (start_app_eps)
						ane_rtclient_start_app_eps(ane);
				} else {
					cancel_delayed_work_sync(&ane->poll_work);
				}
			}
		} else {
			dev_warn(dev,
				 "LEGACY ack withheld (booted=%u scratch3_ack=%u chman_ok=%u)\n",
				 ane->fw ? ane->fw->booted : 0,
				 scratch3_ack, ane->chman_ok);
		}
		if (ane->chman_ok) {
			/* One reusable command buffer for the transport
			 * (CONFIG_GET + every ioctl); reuse is legal
			 * because each exchange completes with the slot
			 * host-owned again. The rest of the table is
			 * for fw MALLOC replies only.
			 */
			int slot = ane_rtclient_legacy_alloc(ane, SZ_16K);

			if (slot >= 0)
				ane->cmd_buf = &ane->legacy_buffers[slot];
			else
				cfg_err = -ENOMEM;
		}
		if (!cfg_err && legacy_query && ane->chman_ok) {
			/* CONFIG_GET (opcode 0x03) keeps the legacy ChMan
			 * transport armed for the ioctls; the reply's
			 * word +0x08 must be nonzero. The
			 * boot heap + 'IPC ' allocations stay HELD until
			 * reboot.
			 */
			struct ane_legacy_buffer *command = ane->cmd_buf;
			unsigned long query_deadline = jiffies + msecs_to_jiffies(3000);
			int qret;

			if (command) {
				memset(command->cpu, 0, SZ_16K);
				qret = ane_rtclient_legacy_exchange(ane, command, 16, 0x03, 1,
								    query_deadline);
				dev_dbg(dev,
					"LEGACY CONFIG_GET words %08x %08x result=%d (DMA remains held)\n",
					READ_ONCE(((u32 *)command->cpu)[1]),
					READ_ONCE(((u32 *)command->cpu)[2]),
					qret);
				if (qret) {
					cfg_err = qret;
				} else if (!READ_ONCE(((u32 *)command->cpu)[2])) {
					dev_err(dev,
						"LEGACY CONFIG_GET reply word +0x08 zero\n");
					cfg_err = -EPROTO;
				} else if (perf_mode_at_boot &&
					   ane->soc == &ane_t8112_soc) {
					/* As macOS at power-on; restarts send it
					 * again (desired_perf). A refusal leaves
					 * the ring usable and the default mode.
					 */
					qret = ane_rtclient_send_perf_mode(ane, query_deadline);
					if (!qret)
						dev_info(dev, "fw perf mode set at power-on\n");
					else if (qret == -EREMOTEIO)
						dev_warn(dev, "fw perf mode refused; default mode kept\n");
					else
						cfg_err = qret;
				}
			} else {
				cfg_err = -ENOMEM;
			}
		}
		ane->boot_done = true;
		if (cfg_err) {
			dev_err(dev,
				"install: CONFIG_GET failed (%d) — refusing to register DRM device (ioctls would run on an unproven ring)\n",
				cfg_err);
			if (!ane->held) {
				pm_runtime_put_sync_suspend(dev);
				pm_runtime_disable(dev);
			}
			return cfg_err;
		}
	} else {
		unsigned long deadline;
		int boot_ret;

		ane->rtk = devm_apple_rtkit_init(dev, ane, NULL, 0,
						 &ane_rtclient_rtkit_ops);
		if (IS_ERR(ane->rtk)) {
			ret = PTR_ERR(ane->rtk);
			ane->rtk = NULL;
			dev_err_probe(dev, ret, "apple_rtkit_init failed\n");
			goto err_pm_or_hold;
		}
		/* RX fallback worker first: the fw's HELLO must not be
		 * missed, and the recv line is not relied on.
		 */
		schedule_delayed_work(&ane->poll_work, msecs_to_jiffies(10));
		deadline = jiffies + msecs_to_jiffies(hello_wait_ms);
		do {
			boot_ret = apple_rtkit_boot(ane->rtk);
		} while (boot_ret == -ETIME && time_before(jiffies, deadline));
		if (boot_ret) {
			dev_err(dev, "rtkit boot handshake failed: %pe\n",
				ERR_PTR(boot_ret));
			cancel_delayed_work_sync(&ane->poll_work);
			ret = boot_ret;
			goto err_pm_or_hold;
		}
		ane->boot_done = true;
		if (start_app_eps)
			ane_rtclient_start_app_eps(ane);
	}

	if (!ane->chman_ok) {
		dev_err(dev,
			"install: ChMan table not validated — refusing to register DRM device (ioctls would stall the ring)\n");
		cancel_delayed_work_sync(&ane->poll_work);
		if (!ane->held) {
			pm_runtime_put_sync_suspend(dev);
			pm_runtime_disable(dev);
		}
		return -EPROTO;
	}

	/* Producer-side stats: preallocate the ring at probe; the files
	 * and the hot-path branch key off stats_slots.
	 */
	if (stats) {
		ane->stats_slots = devm_kcalloc(dev,
						1u << ANE_STATS_RING_ORDER_DEFAULT,
						sizeof(*ane->stats_slots),
						GFP_KERNEL);
		if (ane->stats_slots) {
			ane_stats_counters_init(&ane->stats_ctrs,
						&ane->stats_ring,
						ANE_STATS_RING_ORDER_DEFAULT);
			ane->stats_ring.slots = ane->stats_slots;
		} else {
			dev_warn(dev,
				 "ane_stats ring allocation failed; stats disabled\n");
		}
	}

	{
		struct ane_t6021_drm *adrm;
		int drmret;

		/* This kernel dropped drm_dev_init; the resource-managed
		 * alloc registers the same ABI-2 device.
		 */
		adrm = devm_drm_dev_alloc(dev, &ane_t6021_drm_driver,
					  struct ane_t6021_drm, drm);
		if (IS_ERR(adrm)) {
			ret = dev_err_probe(dev, PTR_ERR(adrm),
					    "drm device alloc\n");
			goto err_pm_or_hold;
		}
		adrm->dev = dev;
		adrm->ane = ane;
		if (ane->stats_slots)
			drm_debugfs_add_file(&adrm->drm, "ane_timeline",
					     ane_timeline_show,
					     &ane->stats_ring);
		if (!ane->soc->platform)
			drm_debugfs_add_file(&adrm->drm, "fw_memory",
					     ane_t6021_fw_memory_show, ane);
		drmret = drm_dev_register(&adrm->drm, 0);
		if (!drmret)
			ane->registered = true;
		if (drmret) {
			dev_err_probe(dev, drmret, "drm_dev_register\n");
			ret = drmret;
			goto err_pm_or_hold;
		}
		WRITE_ONCE(ane_t6021_perf_ane, ane);
		dev_info(dev,
			 "loaded ane_t6021 (DRM major %d minor %d; ABI 2; legacy_only=%u chman_ok=%u booted=%u; state %s; BO cap %u MiB)\n",
			 DRM_ANE_ABI_V2, 0,
			 legacy_only, ane->chman_ok,
			 ane->fw ? ane->fw->booted : 0,
			 ane->held ? "HELD" : "ready", bo_total_max_mb);
	}

	return 0;

err_pm_or_hold:
	if (ane->held) {
		dev_warn(dev,
			 "probe failed after CPU start (%pe) — binding fenced; power/rings/IRQ HELD until reboot\n",
			 ERR_PTR(ret));
	} else {
		pm_runtime_put_sync_suspend(dev);
		pm_runtime_disable(dev);
	}
	return ret;
}

static int ane_rtclient_probe(struct platform_device *pdev)
{
	const struct ane_t602x_soc *soc = of_device_get_match_data(&pdev->dev);
	bool hardware_touched = false;
	int ret;

	if (!soc)
		return -ENODEV;
	/* Later generations cannot enter H14 boot or parameter paths. */
	if (soc->platform)
		return ane_fw_platform_probe(pdev, soc->platform);

	mutex_lock(&ane_rtclient_probe_guard_lock);
	if (ane_rtclient_probe_failed_dirty || ane_rtclient_active) {
		mutex_unlock(&ane_rtclient_probe_guard_lock);
		dev_err(&pdev->dev,
			"ANE already owned or previous probe left hardware dirty; refusing another binding\n");
		return -EBUSY;
	}

	ret = ane_rtclient_probe_inner(pdev, &hardware_touched);
	if (!ret)
		ane_rtclient_active = platform_get_drvdata(pdev);
	if (ret) {
		struct ane_rtclient *ane = platform_get_drvdata(pdev);

		if (ane)
			cancel_delayed_work_sync(&ane->poll_work);
		if (hardware_touched) {
			ane_rtclient_probe_failed_dirty = true;
			dev_err(&pdev->dev,
				"probe failed after hardware access; reboot required before retry\n");
		}
		if (!ane || !ane->held)
			ane_rtclient_detach_genpd(&pdev->dev);
	}
	mutex_unlock(&ane_rtclient_probe_guard_lock);
	return ret;
}

static void ane_rtclient_remove(struct platform_device *pdev)
{
	const struct ane_t602x_soc *soc = of_device_get_match_data(&pdev->dev);
	struct ane_rtclient *ane = platform_get_drvdata(pdev);

	if (soc->platform) {
		ane_fw_platform_remove(pdev);
		return;
	}

	if (READ_ONCE(ane_t6021_perf_ane) == ane)
		WRITE_ONCE(ane_t6021_perf_ane, NULL);

	cancel_delayed_work_sync(&ane->poll_work);
	cancel_work_sync(&ane->sleep_work);

	if (ane->held)
		dev_warn(&pdev->dev,
			 "remove HELD: no teardown — reboot reclaims\n");
}

static const struct ane_t602x_soc ane_h16_soc = {
	.platform = &ane_h16_platform,
};

static const struct ane_t602x_soc ane_t8132_soc = {
	.platform = &ane_t8132_platform,
};

static const struct ane_t602x_soc ane_h17_soc = {
	.platform = &ane_h17_platform,
};

static const struct ane_t602x_soc ane_t6050_soc = {
	.platform = &ane_t6050_platform,
};

static const struct ane_t602x_soc ane_t8142_soc = {
	.platform = &ane_t8142_platform,
};

static const struct of_device_id ane_rtclient_of_match[] = {
	{ .compatible = "apple,t6020-ane", .data = &ane_t6020_soc },
	{ .compatible = "apple,t6021-ane", .data = &ane_t6021_soc },
	{ .compatible = "apple,t6022-ane", .data = &ane_t6022_soc },
	{ .compatible = "apple,t8112-ane", .data = &ane_t8112_soc },
	{ .compatible = "apple,t6041-ane", .data = &ane_h16_soc },
	{ .compatible = "apple,t8132-ane", .data = &ane_t8132_soc },
	{ .compatible = "apple,t8140-ane", .data = &ane_h17_soc },
	{ .compatible = "apple,t6050-ane", .data = &ane_t6050_soc },
	{ .compatible = "apple,t8142-ane", .data = &ane_t8142_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, ane_rtclient_of_match);

/*
 * ane_stats: cumulative busy_ns/jobs for this device (mode 0444, no
 * root needed). Formatting lives in ane_stats_emit() (ane_stats.h),
 * shared with ane.ko.
 */
static ssize_t ane_stats_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct ane_rtclient *ane = dev_get_drvdata(dev);

	return ane_stats_emit(buf, &ane->stats_ctrs);
}
static DEVICE_ATTR_RO(ane_stats);

static ssize_t wedged_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ane_rtclient *ane = dev_get_drvdata(dev);

	/* A restart left pending by system resume is not a wedge: the first
	 * client finishes it (ane_rtclient_admit()) and its outcome sets FAILED.
	 */
	return sysfs_emit(buf, "%d\n", atomic_read(&ane_t6021_quarantined) ||
			  (READ_ONCE(ane->recovering) &&
			   !READ_ONCE(ane->sleep_deferred)) ||
			  !READ_ONCE(ane->registered) ||
			  READ_ONCE(ane->sleep_pm.state) == ANE_T6021_PM_FAILED);
}
static DEVICE_ATTR_RO(wedged);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct ane_rtclient *ane = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&ane_t6021_fw_lock);
	ret = ane_rtclient_recover(ane);
	mutex_unlock(&ane_t6021_fw_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(reset);

static struct attribute *ane_t6021_stats_attrs[] = {
	&dev_attr_ane_stats.attr,
	&dev_attr_wedged.attr,
	&dev_attr_reset.attr,
	NULL,
};

/* ane_stats appears only when stats=1 gave the device a ring. */
static umode_t ane_t6021_stats_is_visible(struct kobject *kobj,
					  struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	const struct ane_t602x_soc *soc = of_device_get_match_data(dev);
	struct ane_rtclient *ane = dev_get_drvdata(dev);

	if (soc->platform)
		return 0;
	if (attr == &dev_attr_ane_stats.attr && !ane->stats_slots)
		return 0;
	return attr->mode;
}

static const struct attribute_group ane_t6021_stats_group = {
	.attrs = ane_t6021_stats_attrs,
	.is_visible = ane_t6021_stats_is_visible,
};

static const struct attribute_group *ane_t6021_stats_groups[] = {
	&ane_t6021_stats_group,
	NULL,
};

static struct platform_driver ane_rtclient_driver = {
	.driver = {
		.name = "ane_t6021",
		.of_match_table = ane_rtclient_of_match,
		.dev_groups = ane_t6021_stats_groups,
		.pm = pm_sleep_ptr(&ane_rtclient_sleep_pm),
		.suppress_bind_attrs = true,
	},
	.probe = ane_rtclient_probe,
	.remove = ane_rtclient_remove,
};

static int __init ane_rtclient_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	int ret;

	/* platform_probe() powers a single genpd, and DMA configuration
	 * happens even earlier. Refuse later generations before registering
	 * the driver, including disabled nodes, so these core paths cannot
	 * precede the ownership and firmware preflight. These SoCs have one
	 * ANE generation; refusing registration cannot hide an H14 engine.
	 */
	for_each_matching_node_and_match(np, ane_rtclient_of_match, &match) {
		const struct ane_t602x_soc *soc = match->data;
		struct platform_device *pdev;

		if (!soc->platform)
			continue;
		if (!of_device_is_available(np))
			continue;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (!pdev) {
			pr_info("ane: H16 ANE: firmware transport unvalidated; keeping disabled\n");
			return -EOPNOTSUPP;
		}
		if (ane_fw_platform_staged(soc->platform)) {
			ret = ane_fw_staged_init(pdev, soc->platform);
			/* No loader hand-off: the probe below refuses quietly. */
			if (ret == -ENODEV)
				ret = 0;
		} else {
			ret = ane_fw_platform_preflight(pdev, soc->platform);
		}
		put_device(&pdev->dev);
		if (ret)
			return ret;
		ane_rtclient_driver.driver_managed_dma = true;
		break;
	}

	WRITE_ONCE(ane_rtclient_probe_failed_dirty, false);
	ret = platform_driver_register(&ane_rtclient_driver);

	if (ret)
		ane_t6021_trace_free();
	else
		ane_fw_staged_start();
	return ret;
}
module_init(ane_rtclient_init);

static void __exit ane_rtclient_exit(void)
{
	if (ane_fw_staged_exit())
		return;
	platform_driver_unregister(&ane_rtclient_driver);
	ane_t6021_trace_free();
}
module_exit(ane_rtclient_exit);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple Neural Engine firmware driver with H16/H17 preflight");
