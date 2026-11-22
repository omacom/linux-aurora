// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC system sleep
 *
 * On some Apple SoCs a kernel can run directly at EL2 behind the CTXR execute
 * window (Documentation/arch/arm64/ctxr-execmem.rst), with no monitor below it.
 * So no PSCI answers a request to power a CPU off or to suspend the system, and
 * the kernel does both itself.
 *
 * iBoot locks every core's reset vector to the boot object. A core that comes
 * back from a power-off therefore restarts inside the boot object (aura), and
 * that has to send it back here. The boot object opts in by publishing a mailbox
 * in /chosen/aurora,s2r-mailbox, which also gives its spin-table CPUs the parking
 * spin table (arch/arm64/kernel/cpu_ops.c). Without it this driver does nothing:
 *
 *  - A secondary core goes back through its spin-table release address, which
 *    the boot object keeps, into secondary_holding_pen.
 *  - The boot core takes cpu_resume from the mailbox: EL2, MMU off.
 *
 * The register addresses are in the device tree. The ADT has every one of them,
 * as /arm-io/pmgr's or /arm-io/wdt's reg entries. Register offsets and order not
 * otherwise explained here were measured from a macOS sleep under aura.
 *
 * What the kernel does in suspend-to-RAM is what macOS does in its last 14 ms,
 * with the order kept. What differs between SoCs, their power words and the
 * lists macOS was measured with, is each SoC's match data. Only SoCs measured
 * and tested that way can suspend. Inventory-only matches report why they are
 * unavailable, before any hardware access; s2idle stays the only sleep.
 *
 * T8132 takes the same path with its own tables and operations, t8132_sleep_ops,
 * measured the same way from a macOS sleep under aura's hypervisor. Its blocks are
 * T8140's at the same offsets from a SoC base 0x80000000 higher unless noted.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/cacheflush.h>
#include <linux/percpu.h>
#include <asm/kvm_arm.h>
#include <linux/math64.h>
#include <linux/unaligned.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqchip/irq-apple-aic.h>
#include <linux/ktime.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/percpu.h>
#include <linux/sched/task_stack.h>
#include <asm/sections.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <linux/platform_device.h>
#include <linux/panic.h>
#include <linux/pm_domain.h>
#include <linux/smp.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/sleep.h>
#include <linux/soc/apple/dcp-v27-sleep.h>
#include <linux/soc/apple/sleep-ans.h>
#include <linux/soc/apple/spmi-fifo.h>
#include <linux/soc/apple/tunable.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <clocksource/arm_arch_timer.h>
#include <asm/apple-idle.h>
#include <dt-bindings/gpio/gpio.h>
#include <asm/arch_timer.h>
#include <asm/barrier.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/memory.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <asm/sysreg.h>
#include <asm/virt.h>

/* Must match aura's struct aura_s2r_mailbox. */
#define AURA_S2R_MAGIC		0x3152325341525541ULL	/* "AURAS2R1" */

struct aura_s2r_mailbox {
	u64 magic;		/* aura, before the kernel starts */
	u64 boot_mpidr;		/* aura: the only core that may take resume_pa */
	u64 resume_pa;		/* kernel, armed before a commit; aura clears it */
	u64 resumes;		/* aura: resume_pa taken this many times */
};

/* PS word */
#define PS_AUTO_ENABLE		BIT(28)
#define PS_UNLATCH		BIT(12)
#define PS_DEV_DISABLE		BIT(10)
#define PS_FLAGS		GENMASK(9, 8)
#define PS_ACTUAL		GENMASK(7, 4)
#define PS_TARGET		GENMASK(3, 0)

/* Clock protection, per slice: its control word, and the bit that guards the slice's range. */
#define PS_PROT_SLICE(n)	(0x100 + 0x40 * (n))
#define PS_PROT_SLICE_ON	BIT(4)
#define PS_OFF_TIMEOUT_US	5000
#define PS_BUSY_SETTLE_US	1000

/* CPU start block, as aura's smp_start_cpu() uses it. A core's bit is the SoC's. */
#define CPU_START_STOP		0x0
#define CPU_START_SYS		0x4
#define CPU_START_CLUSTER(n)	(0x8 + (n) * 4)

/* Power-gate page: reads back this value already. Purpose unknown. */
#define PWRGATE_ON		0x80000000

/* Cluster control: one word per CPU cluster, then the AVE complex's where the SoC has one. */
#define CLUSTER_AVE		0x8
#define CLUSTER_ON		0xf
/* AVE: a transition still in progress, as when a lower waits for a domain it powers */
#define CLUSTER_PENDING		BIT(4)
/* Set when a complex's up sequence ends, and cleared by writing 1. */
#define CLUSTER_UP_DONE		BIT(5)
#define CLUSTER_E_READY		BIT(5)
#define CLUSTER_TIMEOUT_US	5000


/* Watchdog block, ADT /arm-io/wdt reg[0]. WD2 is the sleep safety net. */
#define WDT_WD0_COUNT		0x00
#define WDT_WD0_CTL		0x0c
#define WDT_WD1_COUNT		0x10
#define WDT_WD2_COUNT		0x20
#define WDT_WD2_ALARM		0x28
#define WDT_WD2_CTL		0x2c
#define WDT_TICKS_PER_S		24000000
/* The sleep's own safety net: ADT /arm-io/wdt simple-reconfig-wdog-icc-time. */
#define WDT_SLEEP_SECS		5
/* Device suspend and resume can take a while, and nothing else recovers a hang in them. */
#define WDT_SUSPEND_SECS	120

#define AON_AP_DEADLINE		0x08
/*
 * T8142: a word of the watchdog's second page (/arm-io/wdt + 0x802c). macOS's entry
 * read-modify-writes it from 0 to 0x20 right before WD0's control and WD2 (26B131 trace,
 * 11.1 ms before the S2R timer); XNU writes 0x10 there during boot. Meaning unknown.
 */
#define AON_WDT_SLEEP		0x2c

/* Wake sources: a mask, so 0 enables. Status words are sticky. Where and how many: the SoC. */
#define WAKE_WORDS		9
#define WAKE_SOURCES_MAX	32

/* The AP queue of the PMU's SPMI controller, and its response vector's enable and ack */
#define SPMI_STATUS		0x00
#define SPMI_STATUS_RX_EMPTY	BIT(24)
#define SPMI_CMD		0x04
#define SPMI_REPLY		0x08
#define SPMI_NOTIFY_ENABLE	0x40
#define SPMI_NOTIFY_REPLY	BIT(0)
#define SPMI_NOTIFY_ACK		0x80
#define SPMI_CMD_ACTIVE		BIT(15)
#define SPMI_OP_EXT_WRITE(n)	((n) - 1)
#define SPMI_OP_EXT_READ(n)	(0x20 | ((n) - 1))
#define SPMI_OP_ZERO_WRITE	0x80
#define SPMI_OP_SLEEP		0x11
#define SPMI_OP_WAKEUP		0x13
#define SPMI_POLL_US		20000

/* A port controller on SPMI: registers go through a page select, then a data window. */
#define HPM_REG_SELECT		0x00
#define HPM_REG_DATA		0x20
#define HPM_REG_DATA_CMD	0xa0
#define HPM_PAGE_CMD1		0x08
#define HPM_PAGE_DATA1		0x09
#define HPM_CMD_SSPS		0x53505353
#define HPM_CMD_COLD_RESET	0x64696147
#define HPM_SSPS_S3		3
/* The first look after the reset, and how long a contract that never seemed to go is trusted to have gone. */
#define HPM_RESET_FIRST_MS	50
#define HPM_RESET_FLOOR_MS	1000
#define HPM_PAGE_POWER_STATUS	0x3f
#define HPM_POWER_CONNECTED	BIT(0)
#define HPM_POWER_TYPEC_MODE	GENMASK(3, 2)
#define HPM_POWER_TYPEC_PD	3
/* A contract that comes back does by 0.7 s; one that doesn't is back after another reset. */
#define HPM_CONTRACT_MS		1500
#define HPM_RESET_TRIES		3

/*
 * T8142's controllers are SPMI gen 4 (ADT "gen" = 4 on nub-spmi0, nub-spmi-a0 and the others).
 * macOS 27.2.1's AppleSPMI Gen4 class, from its init: status +0x200 (RX empty bit 30), request
 * FIFO +0x210, response FIFO +0x220, IRQ enable bank +0x400 and ack bank +0x600, word 8 (the
 * notification word) at +0x420/+0x620 as gen 1's at 0x40/0x80. Gen 1's offsets land on other
 * registers there: +0x4 is the FIFO flush, and +0x0 never reads RX empty.
 */
#define SPMI4_STATUS		0x200
#define SPMI4_STATUS_RX_EMPTY	BIT(30)
#define SPMI4_CMD		0x210
#define SPMI4_REPLY		0x220
#define SPMI4_NOTIFY_ENABLE	0x420
#define SPMI4_NOTIFY_ACK	0x620
/*
 * Gen 4: the peripheral sources' status words, below the controller's own (write 1 to
 * clear). A latched source that is enabled holds the controller's interrupt output, an AP
 * wake source, up (T6050, aurora-silicon/linux#1963 commit 24).
 */
#define SPMI4_SRC_ENABLE	0x400
#define SPMI4_SRC_STATUS	0x600
#define SPMI4_SRC_WORDS		8
/* A FIFO holds a few replies at most; never spin on a status that doesn't change. */
#define SPMI_DRAIN_MAX		16

/* A one-byte register write on the sleep's SPMI bus, to the PMU's USID */
#define SPMI_CMD_WRITE		(BIT(15) | 0x30)
#define SPMI_CMD_USID		GENMASK(11, 8)

/* A PMU write: the reply's vector masked, then acknowledged, so none is left raised. */
static inline void apple_sleep_pmu_write(void __iomem *spmi, u8 usid, u16 reg, u8 val)
{
	u32 status, enable;

	enable = readl(spmi + SPMI_NOTIFY_ENABLE);
	writel(enable & ~SPMI_NOTIFY_REPLY, spmi + SPMI_NOTIFY_ENABLE);
	writel(((u32)reg << 16) | SPMI_CMD_WRITE | FIELD_PREP(SPMI_CMD_USID, usid),
	       spmi + SPMI_CMD);
	writel(val, spmi + SPMI_CMD);
	if (!readl_poll_timeout_atomic(spmi + SPMI_STATUS, status,
				       !(status & SPMI_STATUS_RX_EMPTY), 1, 1000))
		readl(spmi + SPMI_REPLY);
	writel(1, spmi + SPMI_NOTIFY_ACK);
	writel(enable, spmi + SPMI_NOTIFY_ENABLE);
}

/* T8132's power words where they aren't T8140's */
#define T8132_PS_ATC0_COMMON		0x378
#define T8132_PS_AON_ATC0_USB_AON	0xb0
#define T8132_PS_AON_ATC0_USB		0xc0

/* The memory controller's channels the SoC steps down, and what macOS steps down on each. */
#define MCC_CHANNELS		16
#define MCC_ENABLES		0x4
#define MCC_LANES		0x8
#define MCC_LANES_MASK		0xc
#define MCC_LANE_STROBE(n)	BIT(16 + 4 * (n))
#define MCC_LANE(n)		BIT(4 * (n))

/* Two words macOS zeroes just before the power gates. */
#define PTD_A			0x0
#define PTD_B			0x8

/* Perf-state words, each like DCS's. The SoC's is first. */
#define CLK_PERF_SOC		0x0
#define SOC_SLEEP_STATE		3

/* The PMC's voter agents, each with a word holding its floor for every rail. */
#define PMC_VOTER_AGENTS	4
#define PMC_VOTER_STRIDE	0x4000

/* DCS: the low nibble asks for a state, and [11:8] is the state it has reached. */
#define DCS_STATE		GENMASK(11, 8)
#define DCS_BUSY		BIT(31)
#define DCS_SLEEP_STATE		4
#define DCS_STEP_STATE		6
#define DCS_TIMEOUT_US		5000

/*
 * Word 0 of the wake block. Bit 0 lets the sleep keep DRAM for the USB host's always-on side.
 * macOS sets it from the host's start to its stop, not at the commit.
 */
#define WAKE_SLP_DDR		0x00
#define WAKE_SLP_DDR_USB	BIT(0)

/*
 * AMCC error enables. Any error enabled and pending at the commit turns the
 * sleep around within 20 ms with no error handler to service it. Linux has none.
 * Measured: with macOS's handler hidden, as Linux's position is, 6 of 6 sleeps
 * held with both of these off, and bit 25 alone still bounced. Their status
 * stays set; only the enables matter. A deep sleep sets them again, so this has
 * to be done before every commit.
 */
#define AMCC_WORD0_ENABLE	0x0
#define AMCC_WORD1_ENABLE	0x8
#define AMCC_WORD0_DSID_AGE_OUT	BIT(25)
#define AMCC_WORD1_UNEXP_RT_HIT	BIT(8)

/* A coprocessor's ASC */
#define ASC_CPU_CONTROL		0x44
#define ASC_CPU_CONTROL_RUN	BIT(4)
#define ASC_CPU_STATUS		0x48

/* The part of the PCIe block that ANS shares, whether or not its host is bound. */
#define APCIE_AXI_EN0		0x104
#define APCIE_AXI_EN1		0x108
#define APCIE_CLK_CTRL		0x0
#define APCIE_CLK_BUSY		BIT(31)
#define APCIE_CLK_RUN		BIT(5)
#define APCIE_CLK_OFF		BIT(1)
#define APCIE_CLK_OFF2		BIT(2)
#define APCIE_CLK_START		0x4
#define APCIE_CLK_START_GO	BIT(31)
#define APCIE_PHY_COMMON	0x4000
#define APCIE_PHY_CTRL		0x8000
#define APCIE_PHY_CTRL_CLK0REQ	BIT(0)
#define APCIE_PHY_CTRL_CLK1REQ	BIT(1)
#define APCIE_PHY_CTRL_CLK0ACK	BIT(2)
#define APCIE_PHY_CTRL_CLK1ACK	BIT(3)
#define APCIE_PHY_CTRL_RESET	BIT(4)
#define APCIE_PHY_CTRL_UP	BIT(27)
#define APCIE_PHY_RUN		0x8004
#define APCIE_PHY_RUN_EN	BIT(0)
#define APCIE_PHY_RUN_EN2	BIT(4)
#define APCIE_PHY_READY		0x8008
#define APCIE_PHY_IP_PLL_CTL	0x90
#define APCIE_PHY_IP_PLL_HOLD	BIT(16)
#define APCIE_PORT_INTSTAT	0x100
#define APCIE_PORT_INTMSK	0x104
#define APCIE_PORT_INT_LINK	GENMASK(7, 4)
#define APCIE_PORT_L2		0x88
#define APCIE_PORT_L2_ENTER	0x11
#define APCIE_PORT_L2_BUSY	BIT(0)
#define APCIE_PORT_PERST_GUARD	0x13c
#define APCIE_PORT_LINKSTS	0x208
#define APCIE_PORT_LINKSTS_UP	BIT(0)
#define APCIE_PORT_APPCLK	0x800
#define APCIE_PORT_APPCLK_EN	BIT(0)
#define APCIE_PORT_APPCLK_CGDIS	BIT(8)
#define APCIE_PORT_PERST	0x82c
#define APCIE_PORT_PHY_LINK	BIT(31)
#define APCIE_PORT_PHY_EN0	BIT(9)
#define APCIE_PORT_PHY_EN1	BIT(10)
#define APCIE_ENGINE_CTRL	0x0
#define APCIE_ENGINE_IRQ_MASK	0x8
#define APCIE_ENGINE_STOP	BIT(0)
#define APCIE_FABRIC_STOP	0x4
#define APCIE_PORT_STOP_US	5000
#define APCIE_RC_ZERO		0x04
#define APCIE_RC_READY_START	0x50
#define APCIE_RC_READY_CFG	0x54
#define APCIE_RC_READY_STAT	0x58
#define APCIE_POLL_US		50000

/* The coprocessors the last mile stops. Each driver idles its own first. */
#define SLEEP_COPROCS		16
#define SLEEP_GATES_MAX		16

/*
 * T8142 (APCIECoreRCGen4): the two PHY-IP words its _shutdownGen4Phy writes, and the PLL
 * control word in pcieclkgen (/arm-io/apcie reg[6]), whose run, off and busy bits are the
 * clock generator's above.
 */
#define APCIE_GEN4_PHY_IP_38	0x38
#define APCIE_GEN4_PHY_IP_38_EN	GENMASK(27, 26)
#define APCIE_GEN4_PHY_IP_8C	0x8c
#define APCIE_GEN4_PHY_IP_8C_OV	GENMASK(27, 24)
#define APCIE_GEN4_CLK_HOLD	GENMASK(4, 3)
#define APCIE_GEN4_CLK_EN	BIT(0)

static struct apple_sleep_coproc {
	struct device *dev;
	void __iomem *asc;
	bool pcie_ans;
	int (*last_check)(void *data);
	int (*last_stop)(void *data);
	void *data;
	bool ready;
	bool stopped;
	u32 status;
	int stop_ret;
	/* No ASC: its last stop takes it whole, power word @ps included, before the others. */
	bool first;
	/* Between ANS and the others, whatever order they registered in. */
	bool early;
	u32 ps;
	struct device_node *domain;
} apple_sleep_coprocs[SLEEP_COPROCS];
static DEFINE_MUTEX(apple_sleep_coprocs_lock);

/* Irreversible enclave stop; nonzero is legal only if SLEEP was not sent. */
static struct {
	struct device *dev;
	int (*stop)(void *data);
	void *data;
} apple_sleep_enclave;

/*
 * Written with bits 0 and 63 by every core right before its last WFI, measured
 * from macOS. This is not CYC_OVRD, which is undefined on this SoC.
 */
#define SYS_APL_DEEP_SLEEP	sys_reg(3, 5, 15, 6, 2)
#define DEEP_SLEEP_ENABLE	(BIT_ULL(0) | BIT_ULL(63))

#define SYS_APL_IPI_SR_EL1	sys_reg(3, 5, 15, 1, 1)
#define IPI_SR_PENDING		BIT(0)
#define SYS_APL_SIQ_CFG_EL1	sys_reg(3, 4, 15, 10, 4)

#define SLEEP_DISPLAYS_MAX	5

/* CPU clusters: 0 is E, 1 is P, for the cpm-dvfm windows */
#define SLEEP_CLUSTERS		2
/* Cluster-control words, CPU clusters only */
#define SLEEP_CPU_CLUSTERS_MAX	4
#define SLEEP_CLK_MUX_MAX	4
#define SLEEP_CLLT_MAX		4
#define SLEEP_AMCC_MAX		16
#define SLEEP_SOCNI_MAX		8

/* cpm-dvfm */
#define DVFM_CTL		0x178
#define DVFM_WORD_0		0x140
#define DVFM_WORD_1		0x160
#define DVFM_WORD_2		0x190
#define DVFM_WORD_3		0x198
#define DVFM_ENABLE		BIT_ULL(63)

/* In macOS's order, on the way down. */
static const u32 sleep_dvfm_words[] = {
	DVFM_WORD_0, DVFM_WORD_1, DVFM_WORD_2, DVFM_WORD_3,
};

/* A power word by its offset, named for the log */
struct apple_sleep_word {
	const char *name;
	u32 offset;
};

#define T8142_PS_ATC0_COMMON		0x2c8
#define T8142_PS_PMP			0x598
#define T8142_PS_AON_ATC0_USB_AON	0xc8
#define T8142_PS_AON_ATC0_USB		0xd0

/* A power word the match leaves alone when its group's bit is set in apple_sleep.ps_keep */
struct apple_sleep_ps_keep {
	const char *name;
	u32 offset;
	bool aon;
	u8 group;
};

/* Power words by their offsets */
struct apple_sleep_words {
	const u32 *offset;
	unsigned int count;
};

#define SLEEP_WORDS(a)		{ .offset = (a), .count = ARRAY_SIZE(a) }

/*
 * A coprocessor that must be bound for a sleep, by its path. @must_stop: it must also have
 * stopped by the last step. @ps: its CPU's own domain, where a driver may power it off.
 */
struct apple_sleep_gate_coproc {
	const char *path;
	bool must_stop;
	u32 ps;
	/* Only with a proven-readable ASC RUN control, never an arbitrary status. */
	bool allow_unstarted;
	bool strict_unbound;
	const char *compatible;
	u64 match_start;
	int run_index;
	bool explicit_run_index;
};

/* A source in the ADT's pmgr/ap-wake-sources: for the log, and to tell the SMC's apart */
struct apple_sleep_wake_name {
	u32 id;
	const char *name;
};

/* A word read, these bits cleared, and written back */
struct apple_sleep_clear {
	u32 offset;
	u32 clear;
};

/*
 * T8142: a DART's PS write-disable bit (SPTM's /chosen/dart-ps-wr-dis-entries), as an offset
 * in "ps" and a bit. While it is set the PMGR drops the AP's writes to that DART's power word,
 * so a word lowered with it set stays on with BUSY latched. macOS's SPTM clears each right
 * before the domain goes down; without SPTM the sleep clears them itself.
 */
struct apple_sleep_ps_wr_dis {
	const char *name;
	u32 offset;
	u8 bit;
};

#define SLEEP_PS_WR_DIS_MAX	8

/* The power words in "ps" that the sleep drives itself */
struct apple_sleep_domains {
	/* CPU, front end and system, of the DCP and then of the external DCP */
	u32 disp[SLEEP_DISPLAYS_MAX][3];
	u32 sio_cpu;
	u32 sep;
	/* The USB host's complex: always-on, common and the controller */
	u32 usb_aon;
	u32 usb_common;
	u32 usb;
	/* Only the deep path's wake turns it on. */
	u32 ap_tmm;
	u32 gpio;
	u32 ans;
	/* ANS's PCIe block */
	u32 apcie_gp;
	u32 apcie_sys_gp;
	u32 apcie_st;
	u32 apcie_sys_st;
	u32 apcie_phy_sw;
	/* T8142: the shared block's AUS5 pair, the last its gate loop powers */
	u32 apcie_aus5_aon;
	u32 apcie_aus5;
};

struct apple_sleep;

struct apple_sleep_region_state;
struct apple_sleep_region_desc {
	const char *name;
	u32 size;
	bool power;
};

struct apple_sleep_region_table {
	u32 region, stride, count;
};

struct apple_sleep_region_profile {
	const struct apple_sleep_region_desc *region;
	unsigned int count;
	struct apple_sleep_words power, replay, entry_clk, entry_perf;
	const struct apple_sleep_region_table *table;
	unsigned int ntables;
};

static int sleep_v27_scope_begin(void);
static void sleep_v27_scope_end(void);
static int sleep_v27_final_check(unsigned int i);
static bool sleep_v27_gate_prepared(const struct device_node *np);
static int sleep_v27_final_stop(struct apple_sleep *s);

/*
 * The steps whose operations differ between SoCs, not only their values: SoCs park their
 * clocks, stop their memory, lower their fabric and arm the cores' deep WFI each their own
 * way. The refusals, the coprocessor stops, macOS's power words, the mailbox and the commit
 * word are shared, and run around these.
 */
struct apple_sleep_extra_off {
	const char *name;
	u32 offset;
	u32 parent;
};

#define SLEEP_EXTRA_OFF_MAX	16

struct apple_sleep_ops {
	/* What makes a core's last WFI end in a reset, after its timers are masked */
	void (*deep_wfi_arm)(void);
	/* Once the other cores stop: the I/O domains the commit lowers itself, ANS's last */
	void (*lower_io)(struct apple_sleep *s);
	/*
	 * The commit, from the still-on polls to the mailbox: the SoC, memory and clusters
	 * down. It calls sleep_ps_save() once the power words are as the wake will find them.
	 * It can't fail, can't print, and has the engine's 33 ms to the last WFI.
	 */
	void (*last_mile)(struct apple_sleep *s);
	/*
	 * In place of lower_io, the watchdog, the still-on polls and last_mile, for a SoC that
	 * shares none of them: every irreversible register step from the stopped cores to the
	 * mailbox. No protocol/lifecycle duplication here.
	 */
	void (*commit_regs)(struct apple_sleep *s);
	/* Where the wake enters the kernel with the MMU off: cpu_resume, or a stub ending there */
	void (*resume)(void);
	/* Optional, once the mailbox is armed: the last step before the commit word */
	void (*pre_commit)(struct apple_sleep *s);
	/* Back on the boot core with the AIC restored: clusters, gates and words back up */
	void (*wake)(struct apple_sleep *s);
	/* Every interrupt line masked before the last WFI, one that ended it acked, restored */
	void (*aic_mask_all)(void);
	u32 (*aic_ack)(void);
	void (*aic_restore)(void);

	/* Optional: the SoC's own register map, in place of the common one */
	int (*map_regs)(struct apple_sleep *s);
	int (*begin)(struct apple_sleep *s);
	int (*stage)(struct apple_sleep *s);
	int (*final_check)(struct apple_sleep *s);
	/* After final peer stops: errors are fatal, never a veto. */
	int (*commit_notice)(struct apple_sleep *s);
	void (*finish)(struct apple_sleep *s);
	/* Optional: the PMU's wake steps, after the SMC's first wake commands */
	void (*pmu_wake)(struct apple_sleep *s);
	int (*cpu_prepare)(u64 mpidr);
	void (*cpu_restore)(u64 mpidr);
	/* Optional: a stopped core's own last steps and WFI at the commit (never returns) */
	void (*cpu_down)(struct apple_sleep *s, u64 mpidr);
	/* Optional: past the point of no return, between ANS's stop and SIO's */
	void (*after_ans)(struct apple_sleep *s);

	/* Pure bounded planning only; no register read/write or entry authorization. */
	int (*plan_complex_request)(u32 observed, u32 *request);
	/* Reversible register preparation, after all refusal checks. */
	void (*prepare_regs)(struct apple_sleep *s);
	void (*undo_regs)(struct apple_sleep *s);
	int (*display_off)(struct apple_sleep *s);
	int (*display_stop)(struct apple_sleep *s);
	/* After the always-on GPU stop, right before SEP's sleep request. */
	void (*before_enclave)(struct apple_sleep *s);
	/* After cpu_suspend saved context, before any stopped core loses power. */
	void (*commit_begin)(struct apple_sleep *s);
	int (*check)(struct apple_sleep *s);
	/* Optional traffic strictly after timer, before AIC/release/DEEP. */
	void (*post_timer)(struct apple_sleep *s);
	/* The dry run's list: does macOS have this ps word off at its commit? */
	bool (*macos_off_at_commit)(u32 offset);
};

/*
 * What differs between SoCs. Power words are offsets in "ps", or in "ps-aon" where named
 * so, and the lists keep the order macOS was measured with.
 */
struct apple_sleep_soc {
	/* Inventory-only profiles cannot enter any hardware preparation callback. */
	const char *unavailable_reason;
	const struct apple_sleep_ops *ops;
	/* The boot core's WFI comes last, this long after the commit, in ms */
	unsigned int boot_wfi_delay_ms;
	bool has_ave;
	bool ack_cluster_dcs;
	bool wake_optional;
	unsigned int ndisp;
	bool lower_unbound_displays;
	/*
	 * apple_sleep.dispext: the external displays (index 1 and up) are lowered only with it on,
	 * and a missing phandle for one is no error. Each one's PS write-disable bit (offset, bit;
	 * offset 0 = none) is cleared before its lowering, as macOS's SPTM leaves it.
	 */
	bool dispext_optional;
	u32 disp_wr_dis[SLEEP_DISPLAYS_MAX][2];
	/*
	 * apple_sleep.extra_off: each display's ADMA word (a child of its SYS word; 0 = none),
	 * lowered before the display's own, and the words lower_io lowers first, children before
	 * parents, each restored at the wake only once its parent (0 = root) is on.
	 */
	u32 disp_adma[SLEEP_DISPLAYS_MAX];
	const struct apple_sleep_extra_off *extra_off;
	unsigned int nextra_off;
	/* The clock-protection slice guarding each display's CPU word, 0 if none needs clearing. */
	u8 disp_prot_slice[SLEEP_DISPLAYS_MAX];
	/* Which display to lower next, in macOS's order: index into ps.disp. */
	u8 display_order[SLEEP_DISPLAYS_MAX];
	const struct apple_sleep_region_profile *regions;
	const struct apple_sleep_smc_policy *smc_policy;
	bool early_context;
	/* The SoC's operations arm and read its wake sources: none of the common wake words */
	bool own_wake;
	bool strict_gate_inventory;
	/*
	 * No PMP runs and its report is disabled: the PTD entries the commit drops have no
	 * report driver to take them from. A board whose report is enabled still takes them.
	 */
	bool pmp_unstarted;
	/*
	 * T6050: the clusters other than the boot cluster are taken down at the start of the
	 * commit, ahead of the boot cluster (t6050_clusters_down_first()).
	 */
	bool clusters_down_first;
	/* T6050: the first video encoder's system word lowered at the entry */
	bool lower_venc0;

	/*
	 * Only for a SoC that states its CPU topology, which the probe then checks: MPIDR's
	 * cluster affinity level (0: level 2) and the CPU-start block's per-cluster bits.
	 */
	unsigned int cpu_cluster_affinity;
	unsigned int cpu_clusters;
	unsigned int cpu_cores_per_cluster;
	bool (*cpu_valid)(u64 mpidr);
	const char *machine;
	u64 boot_mpidr;
	unsigned int cpu_count;
	struct apple_sleep_domains ps;
	/*
	 * In "cluster": each CPU cluster's word, E first. The commit writes them off in this
	 * order, and the wake turns the first on and waits for it before the others.
	 */
	u32 cluster_cpu[SLEEP_CPU_CLUSTERS_MAX];
	unsigned int ncluster_cpu;
	/* "cpu-start": a core's bit is BIT(cpu_start_stride * cluster + core). */
	unsigned int cpu_start_stride;
	/* In "clk-mux": parked in this order before the commit, with this value */
	u32 clk_mux_parked[SLEEP_CLK_MUX_MAX];
	unsigned int nclk_mux_parked;
	u32 clk_mux_parked_val;
	/* In "cllt": bits cleared before the commit */
	struct apple_sleep_clear cllt[SLEEP_CLLT_MAX];
	unsigned int ncllt;
	/* In "ps": PS write-disable bits cleared after the enclave's stop, before the others' */
	const struct apple_sleep_ps_wr_dis *ps_wr_dis;
	unsigned int nps_wr_dis;
	/* In "aon-misc": AON_WDT_SLEEP's bits set before WD2 is armed (0: none) */
	u32 aon_wdt_sleep;
	/* The PMU on the sleep's SPMI bus, and the byte it is told the AP sleeps with */
	u8 pmu_usid;
	u16 pmu_sleep_reg;
	u8 pmu_sleep_val;
	/* The SPMI response-vector enables the sleep runs with */
	u32 spmi_notify_sleep;
	/* "mcc0".. and "amcc0"..: how many the SoC has */
	unsigned int nmcc;
	unsigned int namcc;
	/* "socni0"..: the SoC NI tables macOS zeroes before the memory goes down, in words */
	const u8 *socni_words;
	unsigned int nsocni;
	/* In "wake": the mask and status words, and how many */
	u32 wake_mask;
	u32 wake_status;
	unsigned int wake_words;
	/* In "ps-aon": held up by a live debug link, and off on macOS */
	u32 ps_aon_debug_switch;
	/* In "pwrgate": each pair written as macOS does */
	u32 pwrgate_wake[2];
	u32 pwrgate_cpm[2];
	u32 pwrgate_disp[SLEEP_DISPLAYS_MAX][2];
	/* Every power word, saved at the commit and checked at the wake */
	struct apple_sleep_words ps_all;
	struct apple_sleep_words ps_aon_all;
	/* On at macOS's commit and left off by Linux */
	struct apple_sleep_words ps_macos_on;
	/* Off at macOS's commit, in the order it lowers them */
	struct apple_sleep_words ps_macos_off;
	struct apple_sleep_words ps_aon_macos_off;
	/* In "ps-aon": on at macOS's commit, with AUTO_ENABLE off */
	struct apple_sleep_words ps_aon_macos_no_auto;
	/* In "ps-aon": on at macOS's commit and off under Linux */
	struct apple_sleep_words ps_aon_macos_on;
	/* Written back on by macOS's wake before any driver runs, in its order */
	struct apple_sleep_words ps_wake_order;
	struct apple_sleep_words ps_aon_wake_order;
	/* Polled off just before the complex words */
	struct apple_sleep_words ps_still_on;
	/* The wake's timed power-gate polls: off with BUSY clear at the commit, on macOS */
	struct apple_sleep_words ps_engine_polled;
	/* Put under AUTO_ENABLE by XNU's PMGR start at boot, iBoot leaving them on without it */
	struct apple_sleep_words ps_boot_auto;
	struct apple_sleep_words ps_aon_boot_auto;
	/* T8142: replay XNU's memory-controller start at the entry (apple_sleep.mem_boot) */
	bool mem_boot;
	/* Lowered at the last step if still on, with no user left */
	const struct apple_sleep_word *stray;
	unsigned int nstray;
	/* Words the match leaves as they are, by group (apple_sleep.ps_keep) */
	const struct apple_sleep_ps_keep *ps_keep;
	unsigned int nps_keep;
	/* Off on macOS by the last step: one still on only warns. */
	const struct apple_sleep_word *gate_off;
	unsigned int ngate_off;
	const struct apple_sleep_gate_coproc *gate_coprocs;
	unsigned int ngate_coprocs;
	const struct apple_sleep_wake_name *wake_names;
	unsigned int nwake_names;
	/* The sources in the wake block's last word */
	u32 wake_last_word;
	/* The encoders' system power words: their domains need the AVE complex up */
	const u32 *ave_domains;
	unsigned int nave_domains;
	/* The PMC whose cluster vote for the encoders needs the complex up, and its bit */
	const char *ave_pmc;
	u32 ave_pmc_cluster;
	/* What macOS reads once after its complex words; reported if still on. */
	struct apple_sleep_words quiesce;
	/* Blocks the SoC doesn't have, by reg-name, NULL-terminated: left unmapped. */
	const char *const *regs_absent;
	/* The sleep's SPMI bus has gen 4's queue layout (T8142). */
	bool spmi_gen4;
	/*
	 * Derived from the binaries and another SoC's measurements, never through a full sleep and
	 * wake: deep sleep is offered only with apple_sleep.deep=1 and is never the default, a
	 * sleep stops before the commit unless apple_sleep.untested_commit=1, and each step is
	 * noted in PMU scratch for after a reset.
	 */
	bool untested;
	/*
	 * ANS's PCIe block is T8142's (APCIECoreRCGen4): no clock generator or CIO3 PLL tables,
	 * and its PHY's control words this far into "phy-block" (0: APCIE_PHY_CTRL).
	 */
	bool apcie_gen4;
	u32 apcie_phy_ctrl;
	/*
	 * The radio's link is the loader's, under a generic ECAM host ("apple,radio-host"): it is
	 * stopped at the commit and trained again after the wake by this driver.
	 */
	bool radio_retrain;
	/* A core restarted after the commit takes the deep WFI in the holding pen. */
	bool pen_deep;
	/*
	 * The stopped cores take their deep WFI before the boot core's last mile, not after
	 * the commit (apple_sleep.early_wfi): XNU's order (T8142, J813 ten-core trace).
	 */
	bool early_cpu_wfi;
};

struct apple_sleep {
	struct device *dev;
	const struct apple_sleep_soc *soc;
	struct apple_sleep_region_state *region;
	void __iomem *ps;
	void __iomem *pwrgate;
	void __iomem *cpu_start;
	void __iomem *cllt;
	void __iomem *cluster;
	void __iomem *commit;
	void __iomem *wdt;
	void __iomem *aon_misc;
	void __iomem *wake;
	void __iomem *amcc[SLEEP_AMCC_MAX];
	void __iomem *pmc_dcp;
	void __iomem *ps_prot;
	void __iomem *spmi;
	void __iomem *pmc_voters;
	void __iomem *dcs;
	void __iomem *ps_aon;
	void __iomem *spmi_a0;
	void __iomem *hpm_spmi;
	void __iomem *wdt_aux, *disp_gate, *pmc_rails;
	/* The port controller whose cold reset ends the debug link. */
	struct device_node *hpm_np;
	u64 siq_cfg;
	u32 hpm_sid;
	int usb_down_ret;
	/* Cut, or found already down, for this sleep. */
	bool usb_down_ran;
	/* The PMGR block "ps" maps, for the power controllers' nodes. */
	struct resource ps_res;
	/* Its words a driver took down with apple_sleep_domain_set() and hasn't raised. */
	unsigned long *driver_down;
	void __iomem *clk_mux;
	void __iomem *mcc[MCC_CHANNELS];
	void __iomem *socni[SLEEP_SOCNI_MAX];
	/* T8142: the clock word macOS writes after the memory controller ("mem-clk") */
	void __iomem *mem_clk;
	void __iomem *ptd;
	void __iomem *clk_perf;
	void __iomem *cluster_dcs;
	/* As the match found them: one for each word of the SoC's lists */
	u32 *ps_on_saved;
	u32 *ps_off_saved;
	u32 *ps_aon_saved;
	u32 *ps_aon_on_saved;
	u32 *ps_aon_auto_saved;
	/* apple_sleep.ps_keep as the match took it, for its undo and lower_io */
	unsigned int ps_keep_mask;
	/* apple_sleep.boot_auto: the words it put on AUTO_ENABLE (ps first, then ps-aon) */
	u64 boot_auto_set;
	u32 boot_auto_saved[64];
	/* Of those, how many read AUTO_ENABLE back right after the write */
	unsigned int boot_auto_nset, boot_auto_stuck;
	/* apple_sleep.mem_boot: the pages, the fingerprint's mismatches, whether it ran */
	void __iomem *mem_boot_page[40];
	unsigned int mem_boot_mismatch;
	bool mem_boot_done, mem_floor_done;
	/* apple_sleep.pmp_off: PMP's word as lower_io found it, and whether it went off */
	u32 pmp_saved;
	bool pmp_lowered;
	int pmp_off_ret;
	/* apple_sleep.dispext: each external display's CPU, FE, SYS words before the lowering */
	u32 dispext_saved[SLEEP_DISPLAYS_MAX][3];
	u8 dispext_lowered;
	u8 dispext_note;
	/* apple_sleep.extra_off */
	u32 disp_adma_saved[SLEEP_DISPLAYS_MAX];
	u8 disp_adma_lowered;
	u32 extra_saved[SLEEP_EXTRA_OFF_MAX];
	u16 extra_lowered;
	/* bits 0-15 extra_off's words, bit 31 a display's ADMA */
	u32 extra_stuck;
	/* For each word of ps_all and ps_aon_all: at the commit, as the wake found it, late */
	u32 *ps_commit;
	u32 *ps_aon_commit;
	u32 *ps_woke;
	u32 *ps_aon_woke;
	bool *ps_late;
	bool *ps_aon_late;
	/* The rewake's own books */
	bool *ps_done;
	bool *ps_aon_done;
	u32 spmi_saved[2];
	bool matched;
	void __iomem *dvfm[SLEEP_CLUSTERS];
	struct aura_s2r_mailbox *mbox;

	u32 wake_enable[WAKE_SOURCES_MAX];
	int wake_enable_count;
	u32 wake_saved[WAKE_WORDS];
	/* The sources this sleep left enabled. */
	u32 wake_armed[WAKE_WORDS];

	struct {
		struct device_node *np;
		void __iomem *asc;
		bool bound;
		bool quiesced;
	} display[SLEEP_DISPLAYS_MAX];

	/* Filled in for the report after a return. */
	u32 wake_status[WAKE_WORDS];
	u64 dvfm_ctl[SLEEP_CLUSTERS];
	u64 dvfm_saved[SLEEP_CLUSTERS][ARRAY_SIZE(sleep_dvfm_words)];
	bool dvfm_parked;
	u64 resumes_before;
	u64 resumes_after;
	/* T8132: ATC0's words as they were before the commit path lowered them. */
	u32 atc0_saved[3];
	/*
	 * T8142: what the deep sleep loses and XNU puts back at the wake (J813 ten-core trace,
	 * 2026-11-16): the memory side as it was before the entry's stop (amcc, socni, mcc,
	 * "amcc-cfg<n>"), the clock word and the PMGR clkgen slots ("clk-slots").
	 */
	void __iomem *amcc_cfg[SLEEP_AMCC_MAX];
	void __iomem *clk_slots;
	u64 amcc_saved[SLEEP_AMCC_MAX];
	u64 amcc_cfg_saved[SLEEP_AMCC_MAX][3];
	u32 amcc_ctl_saved[SLEEP_AMCC_MAX];
	u32 socni_saved[SLEEP_SOCNI_MAX][32];
	u32 mcc_saved[MCC_CHANNELS][65];
	u32 clk_slot_saved[126][2];
	u32 mem_clk_saved;
	bool mem_saved, clk_saved;
	int mem_restore_ret;
	int cluster_e_ret;
	int cluster_ave_ret;
	/* The AVE complex, as the parent domain of the encoders' enabled domains */
	struct generic_pm_domain ave_pd;
	bool ave_pd_owned;
	/* Which of the SoC's encoder domains are its subdomains, one bit each */
	unsigned long ave_linked;
	/* Its subdomains: the encoders' system domains, then the PMC's cluster vote */
	struct device_node *ave_children[4];
	unsigned int ave_nchildren;
	int cluster_ave_commit_ret;
	unsigned int ps_rewoken;
	unsigned int ps_rewake_timeouts;
	u64 cntkctl;
	int cpus_stop_ret;
	int ans_off_ret;
	/* Per-attempt M5 ANS receipt; survives platform .wake reporting. */
	int ans_m5_wake_error;
	int pcie_ans_on_ret;
	int apcie_port_clk_ret[2];
	void __iomem *apcie_axi;
	void __iomem *apcie_cio3pll;
	void __iomem *apcie_clkgen;
	void __iomem *apcie_phy;
	void __iomem *apcie_rc;
	bool apcie_port_stopped;
	/* A real sleep left the radio's port down, until its host sets it up again */
	bool apcie_port_down;
	void __iomem *apcie_port;
	void __iomem *apcie_port_phy;
	/* Ports no host sets up again after a real sleep: no managed radio contract */
	bool apcie_ports_unowned;
	void __iomem *apcie_port2;
	void __iomem *apcie_port2_phy;
	void __iomem *apcie_engine;
	void __iomem *apcie_fabric;
	struct apple_tunable *apcie_axi_tunable;
	struct apple_tunable *apcie_cio3pll_tunable;
	struct apple_tunable *apcie_clkgen_tunable;
	struct apple_tunable *apcie_phy_tunable;
	void __iomem *apcie_phy_ip;
	struct apple_tunable *apcie_phy_ip_pll;
	struct apple_tunable *apcie_phy_ip_lanes;
	bool pcie_ans_stopped;
	/* The words apple_sleep_lower_stray() lowered, one bit each */
	unsigned long stray_lowered;
	/* What must be bound for a sleep, and what must have stopped by the last step */
	struct apple_sleep_gate {
		const char *name;
		/* Compared only, never dereferenced: nodes outlive the gate. */
		const struct device_node *np;
		void __iomem *asc;
		bool bound;
		bool must_stop;
		bool sep;
		u32 ps;
		bool allow_unstarted;
		bool strict_unbound;
	} gate[SLEEP_GATES_MAX];
	unsigned int gate_count;
	/* The AP pins, which a deep sleep resets */
	void __iomem *pins;
	u32 *pins_saved;
	unsigned long *pins_skip;
	unsigned int npins;
	unsigned int pin_irq_groups;
	bool pins_saved_valid;
	bool pins_lost;
	/* The radio port's PERST# among them, or -1 */
	int radio_perst;
	bool radio_perst_low;
	bool returned;
	/* This wake came back down the deep path, not the DRAM-kept one. */
	bool deep_ran;
	bool regs_staged, wake_masks_saved, entry_prepared, entry_committed;
	u64 smc_idle_sent, smc_idle_ack, timer_at;
	struct {
		struct device *dev;
		const struct apple_sleep_smc_saved_ops *saved_ops;
		bool held;
		int (*late_notice)(void *data, u64 *sent, u64 *ack);
		void *data;
	} smc;
	/* This sleep's wake came from the SMC's sources alone. */
	bool smc_woke_alone;
	/* A power word the DRAM-kept wake polls stayed latched: this sleep went deep. */
	bool deep_once;
	/* Each latched word the commit cleared, as it found it */
	struct {
		u32 offset;
		u32 val;
	} ps_unlatched[48];
	unsigned int ps_unlatched_count;
	/* T8132: what macOS polls off after its complex words, still on at that point. */
	u32 quiesce_still_on;
	/* The sleep bus's queue, by its SPMI generation */
	u32 spmi_status;
	u32 spmi_rx_empty;
	u32 spmi_cmd;
	u32 spmi_reply;
	u32 spmi_notify_enable;
	u32 spmi_notify_ack;
	/*
	 * Gen 4: nub-spmi0's controller device, whose exchange lock the sleep's own PMU writes
	 * take (apple_spmi_fifo_trylock()); the PMU's latched sources in its status words, from
	 * apple,spmi-pmu-sources; the words as read at the entry and at the wake; and how many
	 * PMU writes found the lock taken.
	 */
	struct device *pmu_bus;
	u32 spmi_pmu_status, spmi_pmu_bits;
	u32 spmi_pmu_enable_saved;
	bool spmi_pmu_wake_armed;
	u32 spmi_src[2][SPMI4_SRC_WORDS];
	bool spmi_src_read[2];
	unsigned int pmu_busy, pmu_busy_forced;
	/*
	 * T8142: each PS write-disable word as read at the entry and after the wake, and which
	 * of the SoC's bits the entry cleared; AON_WDT_SLEEP as read before WD2 was armed.
	 */
	u32 ps_wr_dis_entry[SLEEP_PS_WR_DIS_MAX];
	u32 ps_wr_dis_wake[SLEEP_PS_WR_DIS_MAX];
	u32 ps_wr_dis_cleared;
	bool ps_wr_dis_read;
	u32 aon_wdt_saved;
	bool aon_wdt_marked;
	/* T8142: each complex's ACC word 0xe440f8 (optional), and what its steps returned */
	void __iomem *acc_e;
	void __iomem *acc_p;
	int cluster_p_ret;
	int perf_soc_ret;
	int perf_dcs_ret;
	unsigned int perf_dcs_from;
	/* With radio_retrain: the radio's PCIe host, and its port's blocks and tables */
	struct device_node *radio_host_np;
	void __iomem *radio_port;
	void __iomem *radio_lane;
	void __iomem *radio_cfg;
	struct apple_tunable *radio_port_tunable;
	struct apple_tunable *radio_rc_tunable[3];
	bool radio_trainable;
	int radio_train_ret;
	/* The radio's port stop at the commit (sleep_radio_port_stop()), and its first timeout */
	int radio_stop_ret;
	const char *radio_stop_step;
};

static struct apple_sleep *apple_sleep;
/* Publication/removal only; never acquired in the saved-context callback. */
static DEFINE_MUTEX(apple_sleep_smc_lock);
static int apple_sleep_profile_check(struct apple_sleep *s);
static void __iomem *sleep_region_reg(struct apple_sleep *s, u32 word);
static int sleep_ps_offset(struct apple_sleep *s, struct device_node *pd);
static int sleep_region_domain_offset(struct apple_sleep *s, struct device_node *pd);
static unsigned long *sleep_region_domain_bitmap(struct apple_sleep *s, u32 word, unsigned int *bit);
static void __iomem *sleep_power_reg(struct apple_sleep *s, u32 word)
{
	return s->soc->regions ? sleep_region_reg(s, word) : s->ps + word;
}

static unsigned long *sleep_domain_bitmap(struct apple_sleep *s, u32 word, unsigned int *bit)
{
	if (s->soc->regions)
		return sleep_region_domain_bitmap(s, word, bit);
	*bit = word / 4;
	return s->driver_down;
}


static struct apple_sleep_coproc *apple_sleep_coproc_find(struct device *dev)
{
	int i;

	for (i = 0; i < SLEEP_COPROCS; i++)
		if (apple_sleep_coprocs[i].dev == dev)
			return &apple_sleep_coprocs[i];

	return NULL;
}

/* A registered coprocessor its driver has readied, whose run bit the last step clears. */
static bool apple_sleep_coproc_ready_node(const struct device_node *np)
{
	int i;

	for (i = 0; np && i < SLEEP_COPROCS; i++)
		if (apple_sleep_coprocs[i].dev && apple_sleep_coprocs[i].ready &&
		    apple_sleep_coprocs[i].dev->of_node == np)
			return true;
	return false;
}

/*
 * Untested SoCs (apple_sleep_soc.untested): deep sleep offered at all, and the commit allowed.
 * The second is for one sleep at a time: aurora-sleep-test sets it and clears it after.
 */
static bool sleep_deep;
module_param_named(deep, sleep_deep, bool, 0444);
MODULE_PARM_DESC(deep, "Untested SoC: offer deep sleep in mem_sleep (never the default)");
static bool sleep_untested_commit;
module_param_named(untested_commit, sleep_untested_commit, bool, 0644);
MODULE_PARM_DESC(untested_commit,
		 "Untested SoC: let a deep sleep through to the commit (else a dry run that refuses)");

/*
 * Untested SoCs: each step of a sleep in PMU scratch, readable after a reset. J815's ADT
 * pmu-main info-scrpad is 0x8000 + 0x2800; aurora-silicon/linux#1664 (T6041) notes its steps
 * at 0x8308 too. +0 magic, +1 the last step, +2/+4 the step's own detail, +3 the attempt's
 * count this boot. The boot after reads and clears them.
 */
#define PMU_NOTE_BASE		0x8308
#define PMU_NOTE_MAGIC		0xa5
static bool sleep_pmu_notes = true;
module_param_named(pmu_notes, sleep_pmu_notes, bool, 0644);
MODULE_PARM_DESC(pmu_notes, "Untested SoC: note each sleep step in PMU scratch, for after a reset");

static void sleep_note(struct apple_sleep *s, u8 step);
static void sleep_note_at(struct apple_sleep *s, u8 off, u8 val);
static int sleep_ps_restore(void __iomem *reg, u32 saved);

static bool sleep_usb_asleep;

void apple_sleep_usb_set_asleep(bool asleep)
{
	WRITE_ONCE(sleep_usb_asleep, asleep);
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_set_asleep);

/* J714s: see apple_sleep_late_scope_begin() */
static BLOCKING_NOTIFIER_HEAD(sleep_usb_asleep_chain);

int apple_sleep_usb_asleep_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&sleep_usb_asleep_chain, nb);
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_asleep_notifier_register);

void apple_sleep_usb_asleep_notifier_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&sleep_usb_asleep_chain, nb);
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_asleep_notifier_unregister);

static unsigned long sleep_usb_wake_ports;

void apple_sleep_usb_set_wake_ports(unsigned long ports)
{
	WRITE_ONCE(sleep_usb_wake_ports, ports);
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_set_wake_ports);

bool apple_sleep_usb_port_can_wake(unsigned int port)
{
	return port < BITS_PER_LONG && (READ_ONCE(sleep_usb_wake_ports) & BIT(port));
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_port_can_wake);

/* macOS has every port's always-on USB domain on at its commit with both ports in use. */

static unsigned long sleep_usb_aon_kept;

bool apple_sleep_usb_aon_keep(unsigned int port, bool keep)
{
	if (port >= BITS_PER_LONG)
		return false;
	if (!keep) {
		clear_bit(port, &sleep_usb_aon_kept);
		return false;
	}
	set_bit(port, &sleep_usb_aon_kept);
	return true;
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_aon_keep);

/* The USB host's vote, kept until the word is mapped. Only bit 0 is touched, as on macOS. */
static DEFINE_SPINLOCK(sleep_slp_ddr_lock);
static bool sleep_slp_ddr_usb;
static void __iomem *sleep_slp_ddr_word;

/*
 * A coprocessor that runs through a sleep crashed. The AOP is the always-on master SLP_DDR keeps
 * DRAM up for: after its firmware crashed, the next SLP_DDR wake stalled in the sleep engine,
 * while the deep path doesn't need it. Nothing restarts it short of a reboot.
 */
static bool sleep_slp_ddr_crashed;

/* This sleep only: the DRAM-kept wake would stall on something this commit leaves wrong. */
static bool sleep_slp_ddr_deep_once;

static void sleep_slp_ddr_write(void)
{
	u32 val;

	lockdep_assert_held(&sleep_slp_ddr_lock);
	if (!sleep_slp_ddr_word)
		return;
	val = readl(sleep_slp_ddr_word) & ~WAKE_SLP_DDR_USB;
	if (sleep_slp_ddr_usb && !sleep_slp_ddr_crashed && !sleep_slp_ddr_deep_once)
		val |= WAKE_SLP_DDR_USB;
	writel(val, sleep_slp_ddr_word);
}

/* A driver found something the DRAM-kept wake would stall on: this sleep goes deep. */
static bool sleep_deep_requested;

bool apple_sleep_request_deep_once(void)
{
	if (!READ_ONCE(apple_sleep))
		return false;
	WRITE_ONCE(sleep_deep_requested, true);

	return true;
}
EXPORT_SYMBOL_GPL(apple_sleep_request_deep_once);

static void sleep_slp_ddr_set_deep_once(bool deep)
{
	guard(spinlock_irqsave)(&sleep_slp_ddr_lock);
	sleep_slp_ddr_deep_once = deep;
	sleep_slp_ddr_write();
}

void apple_sleep_usb_slp_ddr(bool up)
{
	guard(spinlock_irqsave)(&sleep_slp_ddr_lock);
	sleep_slp_ddr_usb = up;
	sleep_slp_ddr_write();
}
EXPORT_SYMBOL_GPL(apple_sleep_usb_slp_ddr);

/*
 * From RTKit, for every firmware crash. One this driver stops and restarts around each sleep
 * doesn't run through it; for any other, every later sleep takes the deep path. It may come
 * from a poll with interrupts off, so the table is read without its lock: entries only change
 * at registration.
 */
void apple_sleep_coproc_crashed(struct device *dev)
{
	if (!apple_sleep || apple_sleep_coproc_find(dev))
		return;
	scoped_guard(spinlock_irqsave, &sleep_slp_ddr_lock) {
		if (sleep_slp_ddr_crashed)
			return;
		sleep_slp_ddr_crashed = true;
		sleep_slp_ddr_write();
	}
	dev_warn(apple_sleep->dev, "%s crashed: later sleeps take the deep path\n", dev_name(dev));
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_crashed);

static bool sleep_debug_link_cut;

/* Only the debug port's controller took the cold reset. */
static bool sleep_debug_port_is(struct device *dev)
{
	return apple_sleep && apple_sleep->hpm_np && dev->of_node == apple_sleep->hpm_np;
}

bool apple_sleep_debug_link_was_cut(struct device *dev)
{
	return sleep_debug_port_is(dev) && xchg(&sleep_debug_link_cut, false);
}
EXPORT_SYMBOL_GPL(apple_sleep_debug_link_was_cut);

bool apple_sleep_debug_link_cut_peek(struct device *dev)
{
	return sleep_debug_port_is(dev) && READ_ONCE(sleep_debug_link_cut);
}
EXPORT_SYMBOL_GPL(apple_sleep_debug_link_cut_peek);

/* The radio's host set its port up again after a real sleep: the next commit may stop it. */
void apple_sleep_radio_port_up(void)
{
	if (apple_sleep)
		WRITE_ONCE(apple_sleep->apcie_port_down, false);
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_port_up);

#define PIN_REG(n)		(4 * (n))
#define PIN_DATA		BIT(0)
#define PIN_LOCK		BIT(21)
#define PIN_CTL			0xc48
#define PIN_IRQ(g, w)		(0x800 + 0x40 * (g) + 4 * (w))

/* Before any device suspends, with the pins' domain on. Skipped pins aren't read. */
static void sleep_pins_save(struct apple_sleep *s)
{
	unsigned int pin;

	s->pins_lost = false;
	s->pins_saved_valid = false;
	if (!s->pins)
		return;
	for (pin = 0; pin < s->npins; pin++)
		if (!test_bit(pin, s->pins_skip))
			s->pins_saved[pin] = readl(s->pins + PIN_REG(pin));
	s->pins_saved_valid = true;
}

/*
 * A deep sleep resets every pin, and the pin controller's driver puts back only the
 * ones it configured. macOS puts all of them back right before ANS's run bit, and
 * ANS's link doesn't come up without that: 1 written at 0xc48, every interrupt
 * group's pending bits cleared, then each pin written back as it reads, then as it
 * was, its lock bit last.
 */
void apple_sleep_restore_pins(void)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int pin, grp, w, changed = 0;

	if (!s || !s->pins_lost)
		return;
	s->pins_lost = false;

	writel(1, s->pins + PIN_CTL);
	for (grp = 0; grp < s->pin_irq_groups; grp++)
		for (w = 0; w < DIV_ROUND_UP(s->npins, 32); w++)
			writel(U32_MAX, s->pins + PIN_IRQ(grp, w));
	for (pin = 0; pin < s->npins; pin++) {
		void __iomem *reg = s->pins + PIN_REG(pin);
		u32 want = s->pins_saved[pin], cur;

		if (test_bit(pin, s->pins_skip))
			continue;
		cur = readl(reg);
		changed += cur != want;
		writel(cur, reg);
		if (want & PIN_LOCK)
			writel(want & ~PIN_LOCK, reg);
		writel(want, reg);
	}
	dev_info(s->dev, "%u of %u AP pins changed by the sleep, all put back\n", changed,
		 s->npins - bitmap_weight(s->pins_skip, s->npins));
}
EXPORT_SYMBOL_GPL(apple_sleep_restore_pins);

/*
 * T6050's ANS owner: the sleep stops ANS itself in its last steps. One
 * attempt runs from .begin to .end: NVMe's own .suspend registers before
 * .prepare closes registration, .prepare_late (after device noirq suspend)
 * reserves the one ready owner, and .finish or .end releases it on the suspend
 * task, after early resume has taken its receipt. The manager never takes the
 * owner's device lock: no late or noirq PM phase does either, so an unregister
 * can wait for the release.
 */
static struct {
	struct task_struct *task;
	struct device *dev;
	struct module *module;
	bool open, reserved, leaving;
} sleep_ans_late;
static DECLARE_COMPLETION(sleep_ans_late_released);

static bool sleep_ans_late_selected(void)
{
	return apple_sleep_v27_late();
}

bool apple_sleep_ans_late(void)
{
	return sleep_ans_late_selected();
}
EXPORT_SYMBOL_GPL(apple_sleep_ans_late);

static void sleep_ans_late_open(void)
{
	if (!sleep_ans_late_selected())
		return;
	guard(mutex)(&apple_sleep_coprocs_lock);
	sleep_ans_late.task = current;
	sleep_ans_late.open = true;
}

static void sleep_ans_late_close(void)
{
	if (!sleep_ans_late_selected())
		return;
	guard(mutex)(&apple_sleep_coprocs_lock);
	sleep_ans_late.open = false;
}

/* Exactly one bound, registered, ready ANS owner. Pins it; takes no device lock. */
static int sleep_ans_late_reserve(void)
{
	struct apple_sleep_coproc *c = NULL;
	struct device *dev;
	int i, n = 0;

	if (!sleep_ans_late_selected())
		return 0;
	guard(mutex)(&apple_sleep_coprocs_lock);
	dev = sleep_ans_late.dev;
	if (sleep_ans_late.task != current || sleep_ans_late.reserved || sleep_ans_late.leaving ||
	    !dev)
		return -EBUSY;
	for (i = 0; i < SLEEP_COPROCS; i++)
		if (apple_sleep_coprocs[i].dev && apple_sleep_coprocs[i].pcie_ans) {
			c = &apple_sleep_coprocs[i];
			n++;
		}
	if (n != 1 || c->dev != dev || !c->ready || c->stopped || !c->last_check ||
	    !c->last_stop || !device_is_bound(dev) || !dev->driver)
		return -EBUSY;
	if (!try_module_get(dev->driver->owner))
		return -ENODEV;
	sleep_ans_late.module = dev->driver->owner;
	get_device(dev);
	reinit_completion(&sleep_ans_late_released);
	sleep_ans_late.reserved = true;
	return 0;
}

/* .finish, then .end: both on the suspend task, both after early resume. */
static void sleep_ans_late_release(bool end)
{
	struct module *module = NULL;
	struct device *dev = NULL;

	if (!sleep_ans_late_selected())
		return;
	scoped_guard(mutex, &apple_sleep_coprocs_lock) {
		WARN_ON_ONCE(sleep_ans_late.task && sleep_ans_late.task != current);
		if (sleep_ans_late.reserved) {
			dev = sleep_ans_late.dev;
			module = sleep_ans_late.module;
			sleep_ans_late.module = NULL;
			sleep_ans_late.reserved = false;
			complete_all(&sleep_ans_late_released);
		}
		if (end) {
			sleep_ans_late.open = false;
			sleep_ans_late.task = NULL;
		}
	}
	if (dev) {
		put_device(dev);
		module_put(module);
	}
}

/* Unregister waits out a reservation; no new one starts once it is leaving. */
static void sleep_ans_late_leave(struct device *dev)
{
	for (;;) {
		scoped_guard(mutex, &apple_sleep_coprocs_lock) {
			if (sleep_ans_late.dev != dev)
				return;
			sleep_ans_late.leaving = true;
			if (!sleep_ans_late.reserved)
				return;
		}
		wait_for_completion(&sleep_ans_late_released);
	}
}

/* Below EL2 a monitor owns the last mile, and this code doesn't run it. */
int apple_sleep_coproc_register(struct device *dev, void __iomem *asc, bool pcie_ans,
				int (*last_stop)(void *data), void *data)
{
	struct apple_sleep_coproc *c;

	if (!is_kernel_in_hyp_mode())
		return -ENODEV;

	guard(mutex)(&apple_sleep_coprocs_lock);
	if (apple_sleep_coproc_find(dev))
		return -EEXIST;
	c = apple_sleep_coproc_find(NULL);
	if (!c)
		return -ENOSPC;
	c->asc = asc;
	c->pcie_ans = pcie_ans;
	c->last_check = NULL;
	c->last_stop = last_stop;
	c->data = data;
	c->ready = false;
	c->stopped = false;
	c->first = false;
	c->early = false;
	c->ps = 0;
	c->dev = dev;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_register);

void apple_sleep_coproc_set_early(struct device *dev)
{
	struct apple_sleep_coproc *c;

	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	if (c)
		c->early = true;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_set_early);

int apple_sleep_coproc_register_first(struct device *dev, int (*last_stop)(void *data),
				      void *data)
{
	struct apple_sleep_coproc *c;
	struct device_node *np;
	u32 ps = 0;
	int ret;

	if (!last_stop)
		return -EINVAL;
	np = of_parse_phandle(dev->of_node, "power-domains", 0);
	if (np) {
		of_property_read_u32(np, "reg", &ps);
		of_node_put(np);
	}
	ret = apple_sleep_coproc_register(dev, NULL, false, last_stop, data);
	if (ret)
		return ret;

	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	if (c) {
		c->first = true;
		c->ps = ps;
		c->domain = of_parse_phandle(dev->of_node, "power-domains", 0);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_register_first);

int apple_sleep_coproc_set_last_check(struct device *dev, int (*last_check)(void *data))
{
	struct apple_sleep_coproc *c;

	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	if (!c)
		return -ENOENT;
	c->last_check = last_check;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_set_last_check);

int apple_sleep_enclave_register(struct device *dev, int (*stop)(void *data), void *data)
{
	if (!is_kernel_in_hyp_mode())
		return -ENODEV;

	if (!stop)
		return -EINVAL;
	if (!apple_sleep_commit_stops_coprocs())
		return -EOPNOTSUPP;

	guard(mutex)(&apple_sleep_coprocs_lock);
	if (apple_sleep_enclave.dev)
		return -EBUSY;
	apple_sleep_enclave.stop = stop;
	apple_sleep_enclave.data = data;
	apple_sleep_enclave.dev = dev;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_enclave_register);

void apple_sleep_enclave_unregister(struct device *dev)
{
	guard(mutex)(&apple_sleep_coprocs_lock);
	if (apple_sleep_enclave.dev == dev)
		memset(&apple_sleep_enclave, 0, sizeof(apple_sleep_enclave));
}
EXPORT_SYMBOL_GPL(apple_sleep_enclave_unregister);

void apple_sleep_coproc_unregister(struct device *dev)
{
	struct apple_sleep_coproc *c;

	if (!dev)
		return;
	sleep_ans_late_leave(dev);
	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	if (c) {
		of_node_put(c->domain);
		memset(c, 0, sizeof(*c));
	}
	if (sleep_ans_late.dev == dev) {
		sleep_ans_late.dev = NULL;
		sleep_ans_late.leaving = false;
	}
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_unregister);

void apple_sleep_coproc_set_ready(struct device *dev, bool ready)
{
	struct apple_sleep_coproc *c;

	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	if (!c)
		return;
	c->ready = ready;
	c->stopped = false;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_set_ready);

bool apple_sleep_coproc_stopped(struct device *dev)
{
	struct apple_sleep_coproc *c;

	guard(mutex)(&apple_sleep_coprocs_lock);
	c = apple_sleep_coproc_find(dev);
	return c && c->stopped;
}
EXPORT_SYMBOL_GPL(apple_sleep_coproc_stopped);

bool apple_sleep_commit_stops_coprocs(void)
{
	return READ_ONCE(apple_sleep) && pm_suspend_via_firmware();
}
EXPORT_SYMBOL_GPL(apple_sleep_commit_stops_coprocs);

/*
 * The last thing that can fail on the way to the commit: no CPU but this one is up and
 * interrupts are off, so no lock can be held. A coprocessor stopped here comes back only
 * through a real sleep, so every refusal has to be behind this, and nothing after it may
 * go back. In the order measured on macOS, on ANS itself: whatever its driver does last,
 * status and control read, then 0 written twice.
 */
static void apple_sleep_coproc_run_off(struct apple_sleep_coproc *c)
{
	/* Its own last stop cleared its run bits. */
	if (!c->asc) {
		c->stopped = true;
		return;
	}
	if (!apple_sleep->soc->regions)
		c->status = readl(c->asc + ASC_CPU_STATUS);
	writel(readl(c->asc + ASC_CPU_CONTROL) & ~ASC_CPU_CONTROL_RUN, c->asc + ASC_CPU_CONTROL);
	writel(readl(c->asc + ASC_CPU_CONTROL) & ~ASC_CPU_CONTROL_RUN, c->asc + ASC_CPU_CONTROL);
	c->stopped = true;
}

/* A power word that a ready coprocessor's last stop lowers: on until the commit. */
static bool apple_sleep_coproc_lowers(u32 ps)
{
	int i;

	for (i = 0; i < SLEEP_COPROCS; i++)
		if (apple_sleep_coprocs[i].dev && apple_sleep_coprocs[i].ready &&
		    apple_sleep_coprocs[i].first &&
		    ((apple_sleep->soc->regions && apple_sleep_coprocs[i].domain) ?
		     sleep_ps_offset(apple_sleep, apple_sleep_coprocs[i].domain) == ps :
		     apple_sleep_coprocs[i].ps == ps))
			return true;
	return false;
}

/* The last refusals, each coprocessor's own: nothing has stopped yet. */
static int apple_sleep_coprocs_check(struct apple_sleep *s)
{
	int i, ret;

	for (i = 0; i < SLEEP_COPROCS; i++) {
		struct apple_sleep_coproc *c = &apple_sleep_coprocs[i];

		if (!c->dev || !c->ready || c->stopped || !c->last_check)
			continue;
		ret = c->last_check(c->data);
		if (ret) {
			dev_err(s->dev, "refused: %s: %d\n", dev_name(c->dev), ret);
			return ret;
		}
	}

	return 0;
}

/*
 * The other cores stop at the commit, and one left awake there keeps the engine waiting
 * until WD2 resets the SoC. Each answers here, before the display goes off, and waits for the
 * stop or a refusal.
 */
static int apple_sleep_cpus_hold(struct apple_sleep *s)
{
	int ret = spin_table_hold_parked();

	if (ret)
		dev_err(s->dev, "refused: a CPU didn't answer the hold: %d\n", ret);

	return ret;
}

/* A nonzero result must mean no SLEEP was sent; post-send errors are fatal. */
static int apple_sleep_enclave_stop(struct apple_sleep *s)
{
	int ret;

	if (!apple_sleep_enclave.stop)
		return 0;
	ret = apple_sleep_enclave.stop(apple_sleep_enclave.data);
	if (ret)
		dev_err(s->dev, "refused before SEP SLEEP was sent: %d\n", ret);

	return ret;
}

/*
 * In two passes, after the last refusal: from the first stop only a real sleep restarts the
 * GPU, so a failure here goes on to the commit. Those registered first stop whole first,
 * before macOS's off words drop under them: the GPU withdraws its PMP request there, and the
 * PMP has to see that before its own stop. The rest after the match.
 */
static void apple_sleep_coproc_stop(struct apple_sleep *s, struct apple_sleep_coproc *c)
{
	s->pcie_ans_stopped |= c->pcie_ans;
	if (c->stopped)
		return;
	/* +2: the slot being stopped (1-based), +4: its own step (apple_sleep_note_sub). */
	sleep_note_at(s, 2, c - apple_sleep_coprocs + 1);
	sleep_note_at(s, 4, 0);
	c->stop_ret = c->last_stop ? c->last_stop(c->data) : 0;
	if (s->soc->regions && c->pcie_ans && c->stop_ret)
		s->ans_m5_wake_error = -EIO;
	apple_sleep_coproc_run_off(c);
}

static void apple_sleep_coprocs_stop(struct apple_sleep *s, bool first)
{
	/*
	 * Slot order, except that T6050's second pass stops ANS before the rest, as macOS
	 * does on the M5.
	 * The ANS owner registers at its .suspend, after the PMP's probe-time slot.
	 */
	bool ans_first = s->soc->regions && !first;
	int pass, i;

	if (first)
		s->pcie_ans_stopped = false;
	/* macOS's order: ANS, SIO (early), then the rest, PMP among them. */
	for (pass = ans_first ? 0 : 2; pass < 3; pass++) {
		if (ans_first && pass == 1 && s->soc->ops->after_ans)
			s->soc->ops->after_ans(s);
		for (i = 0; i < SLEEP_COPROCS; i++) {
			struct apple_sleep_coproc *c = &apple_sleep_coprocs[i];

			if (!c->dev || !c->ready || c->first != first ||
			    (ans_first && pass != (c->pcie_ans ? 0 : c->early ? 1 : 2)))
				continue;
			apple_sleep_coproc_stop(s, c);
		}
	}
	dsb(sy);
}

const struct apple_sleep_smc_policy *apple_sleep_smc_policy(void)
{
	return apple_sleep && pm_suspend_via_firmware() ? apple_sleep->soc->smc_policy : NULL;
}
EXPORT_SYMBOL_GPL(apple_sleep_smc_policy);

static bool sleep_j813_smc_late = true;
module_param_named(j813_smc_late, sleep_j813_smc_late, bool, 0444);
MODULE_PARM_DESC(j813_smc_late,
		 "TEST ONLY: J813 final SMC sleep notice after peer stops");

bool apple_sleep_smc_defer_notice(void)
{
	return apple_sleep && READ_ONCE(sleep_j813_smc_late) &&
		of_machine_is_compatible("apple,j813") && pm_suspend_via_firmware();
}
EXPORT_SYMBOL_GPL(apple_sleep_smc_defer_notice);

int apple_sleep_smc_register(struct device *dev,
		int (*late_notice)(void *data, u64 *sent, u64 *ack), void *data)
{
	struct apple_sleep *s = apple_sleep;
	guard(mutex)(&apple_sleep_smc_lock);

	if (!s || !s->soc->smc_policy || !pm_suspend_via_firmware())
		return -EOPNOTSUPP;
	if (!late_notice || s->smc.saved_ops || (s->smc.dev && s->smc.dev != dev))
		return -EBUSY;
	if (!s->smc.dev)
		s->smc.dev = get_device(dev);
	s->smc.late_notice = late_notice;
	s->smc.data = data;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_smc_register);

int apple_sleep_smc_register_saved(struct device *dev,
		const struct apple_sleep_smc_saved_ops *ops, void *data)
{
	struct apple_sleep *s = apple_sleep;
	guard(mutex)(&apple_sleep_smc_lock);

	if (!s || ((!s->soc->early_context || !s->soc->smc_policy) &&
		   !apple_sleep_smc_defer_notice()) || !pm_suspend_via_firmware())
		return -EOPNOTSUPP;
	if (!ops || !ops->begin || !ops->late_notice || !ops->end || s->smc.dev)
		return -EBUSY;
	if (s->soc->ops->commit_notice && (!ops->check || !ops->commit_notice))
		return -EINVAL;
	s->smc.dev = get_device(dev);
	s->smc.saved_ops = ops;
	s->smc.late_notice = ops->late_notice;
	s->smc.data = data;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_smc_register_saved);

void apple_sleep_smc_unregister(struct device *dev)
{
	guard(mutex)(&apple_sleep_smc_lock);

	/* Callers hold dev's lock (.complete or .remove), so a held scope
	 * cannot be torn down underneath the IRQ-disabled callback.
	 */
	if (apple_sleep && apple_sleep->smc.dev == dev) {
		put_device(apple_sleep->smc.dev);
		memset(&apple_sleep->smc, 0, sizeof(apple_sleep->smc));
	}
}
EXPORT_SYMBOL_GPL(apple_sleep_smc_unregister);

void apple_sleep_display_quiesced(struct device *dev, bool quiesced)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int i;

	if (!s || !s->soc->regions)
		return;
	for (i = 0; i < s->soc->ndisp; i++)
		if (s->display[i].np == dev->of_node)
			WRITE_ONCE(s->display[i].quiesced, quiesced);
}
EXPORT_SYMBOL_GPL(apple_sleep_display_quiesced);

/*
 * MPIDR's cluster is affinity 2, its core affinity 0, as 0.0.n and 1.1.n on T8140, unless the
 * SoC names another level for its clusters.
 */
static unsigned int sleep_cpu_cluster(const struct apple_sleep *s, u64 mpidr)
{
	/* A variable first: MPIDR_LEVEL_SHIFT() doesn't parenthesize its argument. */
	unsigned int level = s->soc->cpu_cluster_affinity ?: 2;

	return MPIDR_AFFINITY_LEVEL(mpidr, level);
}

static unsigned int sleep_cpu_core(u64 mpidr)
{
	return MPIDR_AFFINITY_LEVEL(mpidr, 0);
}

static u32 sleep_cpu_start_bit(const struct apple_sleep *s, u64 mpidr)
{
	return BIT(s->soc->cpu_start_stride * sleep_cpu_cluster(s, mpidr) +
		   sleep_cpu_core(mpidr));
}

/*
 * For a SoC that states its topology (cpu_clusters): validate every possible hotplug target
 * before any power callback is exposed.
 */
static int apple_sleep_validate_cpu_topology(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	struct device_node *np = s->dev->of_node;
	struct resource res;
	u32 used = 0;
	unsigned int cpu;
	int index, ret;

	if (soc->cpu_cluster_affinity > 3 || !soc->cpu_clusters || !soc->cpu_cores_per_cluster ||
	    soc->cpu_cores_per_cluster > 32 || soc->cpu_start_stride > 32 ||
	    soc->cpu_start_stride < soc->cpu_cores_per_cluster ||
	    soc->cpu_start_stride * (soc->cpu_clusters - 1) +
		    soc->cpu_cores_per_cluster > 32)
		return dev_err_probe(s->dev, -EINVAL, "invalid CPU-start geometry\n");

	if (soc->machine && (!of_machine_is_compatible(soc->machine) ||
	    cpu_logical_map(0) != soc->boot_mpidr ||
	    num_possible_cpus() != soc->cpu_count))
		return dev_err_probe(s->dev, -EINVAL, "unsupported board/CPU inventory\n");

	index = of_property_match_string(np, "reg-names", "cpu-start");
	if (index < 0)
		return index;
	ret = of_address_to_resource(np, index, &res);
	if (ret)
		return ret;
	if (resource_size(&res) < max_t(unsigned int, CPU_START_STOP,
		CPU_START_CLUSTER(soc->cpu_clusters - 1)) + sizeof(u32))
		return dev_err_probe(s->dev, -EINVAL, "CPU-start resource too small\n");

	for_each_possible_cpu(cpu) {
		u64 mpidr = cpu_logical_map(cpu);
		unsigned int cluster = sleep_cpu_cluster(s, mpidr);
		unsigned int core = sleep_cpu_core(mpidr);
		u32 bit;

		if (cluster >= soc->cpu_clusters || core >= soc->cpu_cores_per_cluster ||
		    (soc->cpu_valid && !soc->cpu_valid(mpidr)))
			return dev_err_probe(s->dev, -EINVAL,
					     "unsupported CPU%u MPIDR %#llx\n", cpu, mpidr);
		bit = sleep_cpu_start_bit(s, mpidr);
		if (used & bit)
			return dev_err_probe(s->dev, -EINVAL,
					     "duplicate CPU-start bit for CPU%u\n", cpu);
		used |= bit;
	}

	return 0;
}

static void t8140_deep_wfi_arm(void)
{
	sysreg_clear_set_s(SYS_APL_DEEP_SLEEP, 0, DEEP_SLEEP_ENABLE);
}

/*
 * T8142 (M5): the core's power-down control, S3_5_C15_C1_6 (#1963's M5_C1_6 on T6050). XNU's
 * per-core init sets its bits 28:26 (26B131 kernelcache +0x4ed4098: orr #0x1c000000) and a
 * kext programs the rest; iBoot leaves them clear. #1963 measured on the M5 Pro that a core
 * running at EL2 without them stays powered in its last WFI, so its cluster never goes down and
 * WD2 resets the SoC ~5 s after the commit: the J815's signature on every committing sleep (note
 * 0x65, P complex BUSY). Set right before each core's last deep WFI, never for idle (a core that
 * powered off in Linux's idle would not come back); a reset follows, so nothing restores it.
 * apple_sleep.m5_pwrdn=0 leaves it alone (A/B).
 */
#define SLEEP_M5_C1_6		sys_reg(3, 5, 15, 1, 6)
#define SLEEP_M5_C1_6_PWRDN	GENMASK_ULL(28, 26)
static bool sleep_m5_pwrdn = true;
module_param_named(m5_pwrdn, sleep_m5_pwrdn, bool, 0644);
MODULE_PARM_DESC(m5_pwrdn, "T8142: set S3_5_C15_C1_6[28:26] before each core's last deep WFI");

/* Probe: each core's value as the boot left it (E cores powered off in b45, the boot core's P complex did not). */
static void t8142_c1_6_read(void *info)
{
	((u64 *)info)[smp_processor_id()] = read_sysreg_s(SLEEP_M5_C1_6);
}

static void t8142_c1_6_report(struct device *dev)
{
	u64 *v = kcalloc(nr_cpu_ids, sizeof(*v), GFP_KERNEL);
	char line[256];
	int cpu, n = 0;

	if (!v)
		return;
	on_each_cpu(t8142_c1_6_read, v, 1);
	for_each_online_cpu(cpu) {
		n += scnprintf(line + n, sizeof(line) - n, " %d:%llx", cpu, v[cpu]);
	}
	dev_info(dev, "probe: S3_5_C15_C1_6 per cpu (boot cpu%d; bits 28:26 = 0x1c000000 set before each last WFI: %s):%s\n",
		 raw_smp_processor_id(), READ_ONCE(sleep_m5_pwrdn) ? "yes" : "no, m5_pwrdn=0", line);
	kfree(v);
}

/*
 * T8142: each core's CoreSight external debug block (J815 ADT cpuN coresight-reg 0x210010000 +
 * cluster * 0x1000000 + core * 0x100000, the T6040 layout). XNU's ml_arm_sleep (26B131 kc
 * +0x4ed7470) unlocks it (EDLAR := 0xc5acce55) and writes EDPRCR := 0 on every core, boot and
 * secondary, right before arm64_prepare_for_sleep(1); with EDPRCR.CORENPDRQ left set a core only
 * emulates its power-down. apple_sleep.ed_release=0 leaves it alone (A/B).
 */
#define T8142_ED_BASE_PA	0x210010000ULL
#define T8142_ED_CLUSTER_STRIDE	0x1000000ULL
#define T8142_ED_CORE_STRIDE	0x100000ULL
#define T8142_ED_EDPRCR		0x310
#define T8142_ED_EDPRSR		0x314
#define T8142_ED_EDLAR		0xfb0
#define T8142_ED_KEY		0xc5acce55
#define T8142_ED_CLUSTERS	2
#define T8142_ED_CORES		6
static void __iomem *t8142_ed[T8142_ED_CLUSTERS][T8142_ED_CORES];
static bool sleep_ed_release = true;
module_param_named(ed_release, sleep_ed_release, bool, 0644);
MODULE_PARM_DESC(ed_release, "T8142: EDLAR unlock + EDPRCR := 0 before each core's last deep WFI");

static void t8142_ed_unmap(void *p)
{
	iounmap((void __iomem *)p);
}

static void t8142_ed_map(struct device *dev)
{
	unsigned int cpu, n = 0;

	for_each_possible_cpu(cpu) {
		u64 mpidr = cpu_logical_map(cpu);
		unsigned int c = MPIDR_AFFINITY_LEVEL(mpidr, 2), k = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		void __iomem *ed;

		if (c >= T8142_ED_CLUSTERS || k >= T8142_ED_CORES || t8142_ed[c][k])
			continue;
		ed = ioremap_np(T8142_ED_BASE_PA + c * T8142_ED_CLUSTER_STRIDE +
				k * T8142_ED_CORE_STRIDE, 0x1000);
		if (!ed || devm_add_action_or_reset(dev, t8142_ed_unmap, (void __force *)ed))
			continue;
		t8142_ed[c][k] = ed;
		n++;
	}
	dev_info(dev, "probe: %u external debug windows mapped (EDPRCR := 0 before each last WFI: %s)\n",
		 n, READ_ONCE(sleep_ed_release) ? "yes" : "no, ed_release=0");
}

/* The dry run: every core's EDPRCR/EDPRSR, read from the boot core while all are up and held */
static void t8142_ed_report(struct device *dev)
{
	char line[256];
	unsigned int cpu, n = 0;

	for_each_possible_cpu(cpu) {
		u64 mpidr = cpu_logical_map(cpu);
		unsigned int c = MPIDR_AFFINITY_LEVEL(mpidr, 2), k = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		void __iomem *ed = (c < T8142_ED_CLUSTERS && k < T8142_ED_CORES) ? t8142_ed[c][k] : NULL;

		if (ed)
			n += scnprintf(line + n, sizeof(line) - n, " %d:%x/%x", cpu,
				       readl_relaxed(ed + T8142_ED_EDPRCR),
				       readl_relaxed(ed + T8142_ED_EDPRSR));
	}
	dev_info(dev, "dry run: EDPRCR/EDPRSR per cpu (CORENPDRQ = EDPRCR bit 0):%s\n", line);
}

static void t8142_ed_release(void)
{
	u64 mpidr = read_cpuid_mpidr();
	unsigned int c = MPIDR_AFFINITY_LEVEL(mpidr, 2), k = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	void __iomem *ed;

	if (!READ_ONCE(sleep_ed_release) || c >= T8142_ED_CLUSTERS || k >= T8142_ED_CORES)
		return;
	ed = t8142_ed[c][k];
	if (!ed)
		return;
	writel_relaxed(T8142_ED_KEY, ed + T8142_ED_EDLAR);
	writel_relaxed(0, ed + T8142_ED_EDPRCR);
}

/*
 * The core's own FIQ sources, quiet before its last WFI (a pending one ends the WFI, masked or
 * not). With VHE, sleep_deep_wfi_arm's cntp/cntv_ctl_el0 reach the EL2 timers only; the EL1
 * pair (_EL02) is what macOS masks (per-core init: cntp_ctl = 2). XNU's core-offline path
 * (26B131 kc +0x4ed69d4) also stops the core PMU (PMCR0 := 0) before its deep WFI.
 * apple_sleep.m5_quiet=0 leaves them alone (A/B).
 */
#define T8142_PMCR0		sys_reg(3, 1, 15, 0, 0)
#define T8142_UPMCR0		sys_reg(3, 7, 15, 0, 4)
#define T8142_UPMSR		sys_reg(3, 7, 15, 6, 4)
#define T8142_VM_TMR_FIQ_ENA	sys_reg(3, 5, 15, 1, 3)
#define T8142_CLPC_C5_0		sys_reg(3, 7, 15, 5, 0)
static bool sleep_m5_quiet = true;
module_param_named(m5_quiet, sleep_m5_quiet, bool, 0644);
MODULE_PARM_DESC(m5_quiet, "T8142: mask the EL1 timers and stop the core PMU before each core's last deep WFI");

/*
 * The last WFI at EL1, not EL2. Under aura on J815 (kit v10, 2026-11-16), macOS's own sleep
 * completed when its cores took their last WFI natively at EL1 (sleep-go-keep) and reset on WD2
 * when aura took the same WFI at EL2 for them (sleep-go-el2): Linux's signature (note 0x65,
 * E complex off, P complex asked off but BUSY, the boot core's WFI held: b45 observer 0x98).
 * Linux runs at EL2 (VHE), so each core drops to EL1h, MMU off, every exception masked, stage 2
 * off, no traps, and loops on WFI there. Nothing returns: a reset or the wake (RVBAR) follows.
 * apple_sleep.el1_wfi: 0 = at EL2 (default), 1 = the boot core only, 2 = every core.
 */
/*
 * The dry run's register readouts (IMPDEF sysregs, every core's ED block, the C1_6 write test):
 * never booted before 2026-11-16, and a dry-run report has reset the Mac before (b32), so they
 * only run when asked: echo 1 > /sys/module/apple_sleep/parameters/dry_regs.
 */
static bool sleep_dry_regs;
module_param_named(dry_regs, sleep_dry_regs, bool, 0644);
MODULE_PARM_DESC(dry_regs, "T8142: print the core register readouts at each dry run (off by default)");

/*
 * Default 0 again since 2026-11-16 14:4x: with 2, the E complex stayed BUSY 2 s after the commit
 * (note 0x67, boot 475520fe); with 0 it powered off, in either release order (fe1573cb, 2335a7e7).
 */
static unsigned int sleep_el1_wfi;
module_param_named(el1_wfi, sleep_el1_wfi, uint, 0644);
MODULE_PARM_DESC(el1_wfi, "T8142: take the last WFI at EL1: 0 no, 1 boot core, 2 every core");

/*
 * Where the stopped cores take their deep WFI. XNU (ml_arm_sleep): each other core writes its
 * stop bit, waits for the boot core's flag, then takes its deep WFI; the boot core clears the
 * flag and only then runs the platform quiesce actions, the PMGR's last mile and the S2R
 * timer among them. Thomas's J813 trace (all ten cores, 27.2, 2026-11-16) has it so: the nine
 * stop bits at -6.7..-6.06 ms, the boot core's first PCIe word at -5.79 ms, the timer at 0.
 * Linux held them in WFE through the memory stop and the clusters' off words and released
 * them after the timer (#1963's T6050 order). 1 (default): XNU's order, the cores in their
 * deep WFI before the last mile; 0: after the commit, as before.
 */
static bool sleep_early_wfi = true;
module_param_named(early_wfi, sleep_early_wfi, bool, 0644);
MODULE_PARM_DESC(early_wfi, "T8142: stopped cores take their deep WFI before the last mile (XNU's order), not after the commit");
#define SLEEP_EARLY_WFI_SETTLE_US	300

/*
 * T8142 DIAGNOSTIC, b45's observer back on the current tree (2026-11-16 14:4x): E core 0 does not
 * take its last WFI; it waits for the boot core's, samples at +20 ms, +300 ms and +observer_ms,
 * writes note 0x6a (one flag byte per sample: bit 0 commit word nonzero, 1/2 complex E/P asked
 * on, 3/4 E/P busy, 5 the boot core's WFI came back, 6 an armed wake source pending, 7 the boot
 * core reached its WFI), then takes its own last WFI, so the sleep can still complete. It answers
 * what the post-commit probe can't: does complex P power down once the boot core is in its WFI.
 * 0: off (default).
 */
static unsigned int sleep_observer_ms;
module_param_named(observer_ms, sleep_observer_ms, uint, 0644);
MODULE_PARM_DESC(observer_ms, "T8142 DIAGNOSTIC: E core 0 watches complex P for this many ms after the boot core's last WFI (note 0x6a), then sleeps (0 = off)");
static u32 t8142_obs_boot_wfi, t8142_obs_wfi_back;

/*
 * T8142 DIAGNOSTIC (b43's post_commit_note, back on the current tree, 2026-11-16): after the
 * commit and the other cores' release, the boot core spins this many ms instead of taking its
 * WFI, writes note 0x66 (what the engine left: the commit word, both complexes, a pending armed
 * wake, the perf states, the PMC voters), then takes its WFI, so the sleep can still go down.
 * The one J815 run that slept and ran the engine's wake program (b43 run 32535966) was such a
 * probe. 0: off (default). The SPMI writes come after the S2R timer, as macOS's own do.
 */
static unsigned int sleep_post_commit_ms;
module_param_named(post_commit_ms, sleep_post_commit_ms, uint, 0644);
MODULE_PARM_DESC(post_commit_ms, "T8142 DIAGNOSTIC: the boot core notes the engine's state this many ms after the commit, then takes its WFI (0 = off)");
/*
 * With post_commit_ms: after note 0x66, every core's EDPRSR through its external debug window
 * (bit 0 PU = the core is powered, bit 1 SPD = it powered down since the last read), note 0x67.
 * A read that hangs leaves 0x66 in the PMU. 0 skips it.
 */
/* Off by default: in boot 475520fe it read PU 0 for every core, the running boot core too. */
static bool sleep_post_commit_ed;
module_param_named(post_commit_ed, sleep_post_commit_ed, bool, 0644);
MODULE_PARM_DESC(post_commit_ed, "T8142 DIAGNOSTIC: with post_commit_ms, also note each core's EDPRSR power status (note 0x67)");

/*
 * T8142: words the T8132-derived lists lower but macOS 27.2 leaves on at the commit (J813 trace
 * g01, 2026-11-16). Bit 0: PMS_C1PPT, SOC_RC. Bit 1: I2C0/2/4, SPI0/1/3/5. macOS raises both
 * groups with AUTO_ENABLE at boot and at each wake and writes neither before any of the run's
 * three committed sleeps; none has a node in the J815 DT, so only this match touches them.
 * Bit 2: ATC1_COMMON, ATC1_USB(_AON). Bit 3: ATC0_USB(_AON), ATC0_COMMON at
 * lower_io. Both ATC ports were hidden from that guest, so bits 2 and 3 have no evidence
 * behind them yet and are off by default.
 */
/*
 * T8142: XNU's PMGR start puts 54 power words under AUTO_ENABLE at boot (J813 trace g01: read,
 * the value with its flags cleared, then the same | AUTO_ENABLE), the same 22 + 4 as at each
 * wake and 28 more. iBoot leaves them on with AUTO_ENABLE clear, and on J815 Linux leaves them
 * so (dry run: PMS_C1PPT 0x2ff, I2C0 0xff): T6050's seq-boot does the same for its own words
 * ("so none can gate"). This does it at the deep entry, right after the dry-run gate, for the
 * words that are on without it; a refusal before the commit takes it back.
 */
static bool sleep_boot_auto_on = true;
module_param_named(boot_auto, sleep_boot_auto_on, bool, 0644);
MODULE_PARM_DESC(boot_auto, "T8142: put XNU's boot list of power words under AUTO_ENABLE at the deep entry (default Y)");
/*
 * T8142: XNU's AppleT8142MCC start (J813 trace g01: ~1120 accesses in the first ms of XNU, 677
 * writes): DCS channels' +0x580/+0x584 enables bit by bit, their lanes (+0x588 strobe/enable with
 * +0x680/+0x58c), the group words, every channel's +0x24 bit 12, the AMCC planes' sleep bits 1..8
 * and config, the SoC NI tables with their +0x100 strobe. iBoot leaves all of it at reset values
 * and no Linux driver does it, so J815's memory side reads 0 where macOS has 0x3ff1, 0x11, ...
 * Replayed at the deep entry after the last refusal point, only if the 34 words XNU reads before
 * its first write hold exactly what they hold on the J813 (iBoot's values). J813 constants on
 * J815's memory controller: default off.
 */
static bool sleep_mem_boot;
module_param_named(mem_boot, sleep_mem_boot, bool, 0644);
MODULE_PARM_DESC(mem_boot, "T8142 EXPERIMENT: replay XNU's memory-controller start at the deep entry if iBoot's fingerprint matches (default N)");
/*
 * T8142: T6050's memory performance floor (its voter 1: 2, 1 at 9 ms, then 0 before the memory
 * stop; "the value read back stays 0"). No T8142 evidence: J813's trace didn't hook the voters.
 */
/*
 * T8142: PMP's power domain off after its firmware's final stop (Thomas Glanzmann,
 * m0d16l14n1/linux#4 e754b17c, 2026-11-16): the PMP is idled and RUN cleared, but its domain
 * stayed at target f; macOS lowers it (J813 g01: 0x598 0x0f0f00ff -> 0x0f0f03f0 at -12.2 ms),
 * and PMS_SRAM then gates by itself. On J813 the first real sleep and return came with it.
 */
static bool sleep_pmp_off = true;
module_param_named(pmp_off, sleep_pmp_off, bool, 0644);
MODULE_PARM_DESC(pmp_off, "T8142: power the PMP's domain off after its final stop, as macOS (default Y)");

/*
 * J815 macOS 26B131 (notes/sleep-trace/t8142-j815-26B131-entry.md): DISPEXT0, then DISPEXT1,
 * then DISP, each SYS/FE up, the CPU word down in four steps, SYS/FE down, 159-137 ms before
 * the S2R timer; all six DISPEXT words are off at macOS's commit. Linux's T8142 profile only
 * lowered the internal DCP, while both external DCPs are bound here and their FE/SYS words read
 * 0x20ff in the dry run.
 */
static bool sleep_dispext = true;
module_param_named(dispext, sleep_dispext, bool, 0644);
MODULE_PARM_DESC(dispext, "T8142: lower the external DCPs' domains with the display, as macOS (default Y)");

/*
 * The OS-managed words (not no_ps in the J815 ADT) that macOS has off at its commit (J813 g01
 * end state) and no other step of Linux's sleep lowers: SIO_CPU (on from iBoot, no driver on
 * J815; macOS lowers it 53 ms before the timer, Thomas's "inactive SIO CPU domain"), FPWM0 (the
 * keyboard backlight), GLOBAL_AUX_CTRL0/1, the PCIe OCLAs (children of APCIE_SYS_GP/ST, which
 * lower_io lowers) and the displays' ADMA words (children of their SYS words). Each only if on.
 */
static bool sleep_extra_off = true;
module_param_named(extra_off, sleep_extra_off, bool, 0644);
MODULE_PARM_DESC(extra_off, "T8142: lower the words macOS has off at its commit that nothing else lowers (default Y)");

static bool sleep_mem_floor;
module_param_named(mem_floor, sleep_mem_floor, bool, 0644);
MODULE_PARM_DESC(mem_floor, "T8142 EXPERIMENT: step PMC voter 1 down (2, 1, 0) as T6050 does (default N)");

static bool sleep_boot_auto_dry = true;
module_param_named(boot_auto_dry, sleep_boot_auto_dry, bool, 0644);
MODULE_PARM_DESC(boot_auto_dry, "T8142: a dry run also does boot_auto's writes, reads them back and undoes them (default Y)");

static unsigned int sleep_ps_keep = 0x3;
module_param_named(ps_keep, sleep_ps_keep, uint, 0644);
MODULE_PARM_DESC(ps_keep, "T8142: power words left as they are at the match, by group (bit 0 PMS_C1PPT/SOC_RC, 1 I2C/SPI, 2 ATC1, 3 ATC0 at lower_io; default 0x3)");

/* The T8142 wake's own restores (J813 trace), each switchable for an A/B. */
static bool sleep_mem_restore = true;
module_param_named(mem_restore, sleep_mem_restore, bool, 0644);
MODULE_PARM_DESC(mem_restore, "T8142: put the memory side (AMCC, SoC NI, DCS) back at the wake as XNU does");
static bool sleep_clk_restore = true;
module_param_named(clk_restore, sleep_clk_restore, bool, 0644);
MODULE_PARM_DESC(clk_restore, "T8142: put the 126 PMGR clkgen slots and the memory clock word back at the wake as XNU does");
static bool sleep_perf_down = true;
module_param_named(perf_down, sleep_perf_down, bool, 0644);
MODULE_PARM_DESC(perf_down, "T8142: SoC/DCS perf states down before the memory stop (macOS on J813 never writes them; A/B)");
static u64 t8142_boot_mpidr = ~0ULL;

extern char t8142_el1_wfi_stub[], t8142_el1_wfi_stub_end[];
asm(".pushsection .text, \"ax\"\n"
	".balign 64\n"
	".global t8142_el1_wfi_stub\n"
	"t8142_el1_wfi_stub:\n"
	"1:	dsb sy\n"
	"	isb\n"
	"	wfi\n"
	"	b 1b\n"
	".global t8142_el1_wfi_stub_end\n"
	"t8142_el1_wfi_stub_end:\n"
	".popsection\n");

static void __noreturn t8142_el1_wfi(void)
{
	write_sysreg_s(INIT_SCTLR_EL1_MMU_OFF, SYS_SCTLR_EL12);
	isb();
	write_sysreg(HCR_RW | HCR_E2H, hcr_el2);
	write_sysreg(PSR_MODE_EL1h | PSR_D_BIT | PSR_A_BIT | PSR_I_BIT | PSR_F_BIT, spsr_el2);
	write_sysreg(__pa_symbol(t8142_el1_wfi_stub), elr_el2);
	/* The EL1 loop acks nothing: no fast IPI may be left pending to end its WFI. */
	write_sysreg_s(IPI_SR_PENDING, SYS_APL_IPI_SR_EL1);
	isb();
	asm volatile("eret" ::: "memory");
	unreachable();
}

static bool t8142_el1_wfi_here(void)
{
	unsigned int mode = READ_ONCE(sleep_el1_wfi);

	if (!is_kernel_in_hyp_mode() || !mode)
		return false;
	return mode >= 2 || (read_cpuid_mpidr() & MPIDR_HWID_BITMASK) == READ_ONCE(t8142_boot_mpidr);
}

static void t8142_deep_wfi_arm(void)
{
	if (READ_ONCE(sleep_m5_pwrdn) && is_kernel_in_hyp_mode())
		sysreg_clear_set_s(SLEEP_M5_C1_6, 0, SLEEP_M5_C1_6_PWRDN);
	if (READ_ONCE(sleep_m5_quiet)) {
		if (is_kernel_in_hyp_mode()) {
			write_sysreg_s(ARCH_TIMER_CTRL_IT_MASK, SYS_CNTP_CTL_EL02);
			write_sysreg_s(ARCH_TIMER_CTRL_IT_MASK, SYS_CNTV_CTL_EL02);
		}
		write_sysreg_s(0, T8142_PMCR0);
		isb();
	}
	t8142_ed_release();
	t8140_deep_wfi_arm();
	if (t8142_el1_wfi_here()) {
		dsb(sy);
		t8142_el1_wfi();
	}
}

static u8 t8142_fiq_sources(void)
{
	u8 m = 0;
#define FIRING(v)	(((v) & (ARCH_TIMER_CTRL_ENABLE | ARCH_TIMER_CTRL_IT_MASK | ARCH_TIMER_CTRL_IT_STAT)) == \
			 (ARCH_TIMER_CTRL_ENABLE | ARCH_TIMER_CTRL_IT_STAT))
	if (FIRING(read_sysreg(cntp_ctl_el0)))
		m |= 0x01;
	if (FIRING(read_sysreg(cntv_ctl_el0)))
		m |= 0x02;
	if (is_kernel_in_hyp_mode()) {
		if (FIRING(read_sysreg_s(SYS_CNTP_CTL_EL02)))
			m |= 0x04;
		if (FIRING(read_sysreg_s(SYS_CNTV_CTL_EL02)))
			m |= 0x08;
	}
#undef FIRING
	if (read_sysreg_s(T8142_PMCR0) & BIT(11))
		m |= 0x10;
	if (read_sysreg_s(SYS_APL_IPI_SR_EL1) & IPI_SR_PENDING)
		m |= 0x20;
	if (read_sysreg_s(T8142_UPMSR) & BIT(0))
		m |= 0x40;
	return m;
}

/*
 * The boot core's last WFI (at EL2) returned while it should be powered off: note 0x6c once, 100 ms
 * into the loop (past the engine's 33 ms window, so no SPMI traffic while the sleep can still
 * go through): +2 0x80 | ISR_EL1's A/I/F (bits 2/1/0), +4 the core's own FIQ sources still
 * firing (0x01 CNTP/EL2, 0x02 CNTV/EL2, 0x04 CNTP_EL02, 0x08 CNTV_EL02, 0x10 PMC, 0x20 IPI,
 * 0x40 UPMC). The next boot reads 0x6c (it woke over and over) or 0x65 (it sat in WFI).
 */
static void t8142_wfi_returned(struct apple_sleep *s, u64 t0, bool *noted)
{
	u64 isr;

	if (*noted || read_sysreg(cntpct_el0) - t0 < arch_timer_get_cntfrq() / 10)
		return;
	*noted = true;
	isr = read_sysreg(isr_el1);
	sleep_note_at(s, 2, 0x80 | ((isr >> 6) & 7));
	sleep_note_at(s, 4, t8142_fiq_sources());
	sleep_note(s, 0x6c);
}

static void sleep_deep_wfi_arm(void)
{
	/* A timer left running ends the WFI, and costs power until reset. */
	write_sysreg(ARCH_TIMER_CTRL_IT_MASK, cntp_ctl_el0);
	write_sysreg(ARCH_TIMER_CTRL_IT_MASK, cntv_ctl_el0);
	apple_sleep->soc->ops->deep_wfi_arm();
}

/* A core's last instructions: nothing brings it back but a reset. */
static void __noreturn sleep_deep_wfi(void)
{
	sleep_deep_wfi_arm();

	while (true) {
		dsb(sy);
		isb();
		wfi();
		write_sysreg_s(IPI_SR_PENDING, SYS_APL_IPI_SR_EL1);
	}
}

static void sleep_cpu_stop(u64 mpidr)
{
	struct apple_sleep *s = apple_sleep;

	writel(sleep_cpu_start_bit(s, mpidr), s->cpu_start + CPU_START_STOP);
	dsb(sy);
}

static u8 t8142_obs_sample(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	u32 e = readl(s->cluster + soc->cluster_cpu[0]);
	u32 p = readl(s->cluster + soc->cluster_cpu[1]);
	u8 f = 0;
	int i;

	if (readl(s->commit))
		f |= BIT(0);
	if (e & CLUSTER_ON)
		f |= BIT(1);
	if (p & CLUSTER_ON)
		f |= BIT(2);
	if (e & BIT(4))
		f |= BIT(3);
	if (p & BIT(4))
		f |= BIT(4);
	if (READ_ONCE(t8142_obs_wfi_back))
		f |= BIT(5);
	for (i = 0; i < soc->wake_words; i++)
		if (readl(s->wake + soc->wake_status + 4 * i) & s->wake_armed[i])
			f |= BIT(6);
	if (READ_ONCE(t8142_obs_boot_wfi))
		f |= BIT(7);
	return f;
}

/* E core 0 with apple_sleep.observer_ms: see sleep_observer_ms. */
static void t8142_observe(struct apple_sleep *s, unsigned int ms)
{
	u64 freq = arch_timer_get_cntfrq(), t0 = read_sysreg(cntpct_el0);
	static const unsigned int at_ms[2] = { 20, 300 };
	u8 b[3];
	int i;

	/* At most 30 s for the boot core to get there (a hang before it ends at WD2 anyway). */
	while (!READ_ONCE(t8142_obs_boot_wfi) && read_sysreg(cntpct_el0) - t0 < 30 * freq)
		cpu_relax();
	t0 = read_sysreg(cntpct_el0);
	for (i = 0; i < 3; i++) {
		u64 wait = (i < 2 ? at_ms[i] : max(ms, 301U)) * freq / 1000;

		while (read_sysreg(cntpct_el0) - t0 < wait)
			cpu_relax();
		b[i] = t8142_obs_sample(s);
	}
	sleep_note_at(s, 2, b[0]);
	sleep_note_at(s, 3, b[1]);
	sleep_note_at(s, 4, b[2]);
	sleep_note(s, 0x6a);
}

static void sleep_cpu_off(u64 mpidr)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int ms = READ_ONCE(sleep_observer_ms);

	if (ms && s && s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm &&
	    (mpidr & MPIDR_HWID_BITMASK) == 0)
		t8142_observe(s, ms);
	sleep_deep_wfi();
}

/* A stopped core with last steps of its own before the WFI */
static void sleep_cpu_down_off(u64 mpidr)
{
	if (apple_sleep && apple_sleep->soc->ops->cpu_down)
		apple_sleep->soc->ops->cpu_down(apple_sleep, mpidr);
	sleep_deep_wfi();
}

/*
 * After a sleep, one line per core start, when its dynamic debug site is on (no runtime switch;
 * without CONFIG_DYNAMIC_DEBUG it is compiled out, and so is its wait). aura's spin-table slot is
 * { mpidr, flag, target, ... } with cpu-release-addr = &target, and aura bumps flag as the core
 * enters its reset path: "flag a -> b" tells a core that never left reset from one that ran
 * and stopped later (in the kernel's pen or its bring-up). Also the cpu-start words before and
 * after, the core's power and power-gate words and its EDPRSR (bit 0 PU). On with
 * dyndbg="func sleep_cpu_on +p" (only +p matters: the line is a pr_info).
 */
static void __iomem *sleep_cpu_slot[NR_CPUS];

static void __iomem *sleep_cpu_slot_map(int cpu)
{
	struct device_node *np;
	u64 rel;

	if (cpu < 0 || cpu >= NR_CPUS)
		return NULL;
	if (sleep_cpu_slot[cpu])
		return sleep_cpu_slot[cpu];
	np = of_get_cpu_node(cpu, NULL);
	if (!np)
		return NULL;
	if (!of_property_read_u64(np, "cpu-release-addr", &rel) && rel >= 16)
		sleep_cpu_slot[cpu] = ioremap_cache(rel - 16, 24);
	of_node_put(np);
	return sleep_cpu_slot[cpu];
}

static u64 sleep_cpu_slot_read(void __iomem *slot, unsigned int off)
{
	dcache_inval_poc((unsigned long)slot + off, (unsigned long)slot + off + 8);
	return readq(slot + off);
}

static void sleep_cpu_start_words(struct apple_sleep *s, u32 *w)
{
	w[0] = readl(s->cpu_start + CPU_START_STOP);
	w[1] = readl(s->cpu_start + CPU_START_SYS);
	w[2] = readl(s->cpu_start + CPU_START_CLUSTER(0));
	w[3] = readl(s->cpu_start + CPU_START_CLUSTER(1));
}

static int sleep_cpu_on(u64 mpidr)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int cluster = sleep_cpu_cluster(s, mpidr), core = sleep_cpu_core(mpidr);
	int cpu = get_logical_index(mpidr & MPIDR_HWID_BITMASK);
	void __iomem *slot = NULL;
	u32 before[4], after[4], ps = 0, gate = 0, edprsr = ~0U;
	u64 flag0 = 0, flag1 = 0;
	unsigned int us = 0;
	bool trace;
	DEFINE_DYNAMIC_DEBUG_METADATA(cpu_on_trace, "apple-sleep: cpu start after a sleep\n");

	trace = DYNAMIC_DEBUG_BRANCH(cpu_on_trace) && s->soc->cpu_clusters >= 2;
	if (trace) {
		slot = sleep_cpu_slot_map(cpu);
		sleep_cpu_start_words(s, before);
		if (slot)
			flag0 = sleep_cpu_slot_read(slot, 8);
	}

	writel(sleep_cpu_start_bit(s, mpidr), s->cpu_start + CPU_START_SYS);
	writel(BIT(core), s->cpu_start + CPU_START_CLUSTER(cluster));

	if (!trace)
		return 0;
	/* Up to 20 ms for aura's entry to count it; the kernel's own wait is 5 s. */
	for (us = 0; slot && us < 20000; us += 20) {
		flag1 = sleep_cpu_slot_read(slot, 8);
		if (flag1 != flag0)
			break;
		udelay(20);
	}
	sleep_cpu_start_words(s, after);
	if (s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm) {
		/* T8142 core words: E k at 8k, P k at 0x30 + 8k; power gates at a 0x10 stride */
		ps = readl(s->ps + (cluster ? 0x30 : 0) + 8 * core);
		gate = readl(s->pwrgate + (cluster ? 0x60 : 0) + 0x10 * core);
		if (cluster < T8142_ED_CLUSTERS && core < T8142_ED_CORES && t8142_ed[cluster][core])
			edprsr = readl_relaxed(t8142_ed[cluster][core] + T8142_ED_EDPRSR);
	}
	pr_info("apple-sleep: cpu%d start %#llx: stop/sys/e/p %x/%x/%x/%x -> %x/%x/%x/%x; aura slot %s flag %llu -> %llu after %u us; ps %#x pwrgate %#x edprsr %#x\n",
		cpu, mpidr, before[0], before[1], before[2], before[3], after[0], after[1],
		after[2], after[3], slot ? "mapped" : "UNMAPPED", flag0, flag1, us, ps, gate, edprsr);

	return 0;
}

static int sleep_cpu_hold(u64 mpidr)
{
	return apple_sleep->soc->ops->cpu_prepare ?
		apple_sleep->soc->ops->cpu_prepare(mpidr) : 0;
}

static void sleep_cpu_restore(u64 mpidr)
{
	if (apple_sleep->soc->ops->cpu_restore)
		apple_sleep->soc->ops->cpu_restore(mpidr);
}

static const struct spin_table_power_ops sleep_cpu_power_ops = {
	.cpu_stop = sleep_cpu_stop,
	.cpu_off = sleep_cpu_off,
	.cpu_on = sleep_cpu_on,
};

/* For a SoC that prepares each held core (cpu_prepare): the same, with its hooks */
static const struct spin_table_power_ops sleep_cpu_prepared_power_ops = {
	.cpu_hold = sleep_cpu_hold,
	.cpu_restore = sleep_cpu_restore,
	.cpu_stop = sleep_cpu_stop,
	.cpu_off = sleep_cpu_down_off,
	.cpu_on = sleep_cpu_on,
};

/*
 * As macOS lowers a domain: AUTO_ENABLE off first if it's set, then target 0.
 * Writing the flags clears them. Values measured, for instance ANS 0x1f0020ff,
 * 0x0f0020ff, 0x0f0023f0.
 */
static u32 sleep_ps_lower_start(struct apple_sleep *s, u32 offset)
{
	void __iomem *reg = sleep_power_reg(s, offset);
	u32 val = readl(reg);

	if (val & PS_AUTO_ENABLE) {
		val &= ~(PS_AUTO_ENABLE | PS_FLAGS);
		writel(val, reg);
	}

	return val;
}

static void sleep_ps_lower_finish(struct apple_sleep *s, u32 offset, u32 val)
{
	writel((val & ~PS_TARGET) | PS_FLAGS, sleep_power_reg(s, offset));
}

static void sleep_ps_lower(struct apple_sleep *s, u32 offset)
{
	sleep_ps_lower_finish(s, offset, sleep_ps_lower_start(s, offset));
}

/*
 * As macOS raises a domain: the target, then AUTO_ENABLE once it reads on. Measured
 * byte for byte on ANS and PCIe, for instance 0x0f002300, 0x0f00200f, 0x1f00200f.
 */
static int sleep_ps_raise_start(struct apple_sleep *s, u32 offset)
{
	void __iomem *reg = sleep_power_reg(s, offset);
	u32 now;

	writel((readl(reg) & ~(PS_AUTO_ENABLE | GENMASK(11, 0))) | PS_TARGET, reg);

	return readl_poll_timeout_atomic(reg, now, (now & PS_ACTUAL) == PS_ACTUAL, 1,
					 PS_OFF_TIMEOUT_US);
}

static void sleep_ps_raise_finish(struct apple_sleep *s, u32 offset)
{
	void __iomem *reg = sleep_power_reg(s, offset);

	writel((readl(reg) & ~(PS_AUTO_ENABLE | GENMASK(11, 0))) | PS_TARGET | PS_AUTO_ENABLE,
	       reg);
}

static int sleep_ps_raise(struct apple_sleep *s, u32 offset)
{
	int ret = sleep_ps_raise_start(s, offset);

	sleep_ps_raise_finish(s, offset);

	return ret;
}

/* A domain the hardware idled under AUTO_ENABLE comes back once that's cleared. */
static bool sleep_ps_wait_on(struct apple_sleep *s, u32 offset, u32 val)
{
	if (!FIELD_GET(PS_TARGET, val))
		return false;

	return !readl_poll_timeout_atomic(sleep_power_reg(s, offset), val, FIELD_GET(PS_ACTUAL, val), 1,
					  PS_OFF_TIMEOUT_US);
}

static int sleep_ps_wait_off(struct apple_sleep *s, u32 offset)
{
	u32 val;

	return readl_poll_timeout_atomic(sleep_power_reg(s, offset), val, !FIELD_GET(PS_ACTUAL, val),
					 10, PS_OFF_TIMEOUT_US);
}

static bool sleep_ps_on(struct apple_sleep *s, u32 offset)
{
	return FIELD_GET(PS_ACTUAL, readl(sleep_power_reg(s, offset)));
}

/* Below full power the hardware has idled it, and its registers may not answer. */
static bool sleep_ps_full(struct apple_sleep *s, u32 offset)
{
	return FIELD_GET(PS_ACTUAL, readl(sleep_power_reg(s, offset))) == FIELD_MAX(PS_ACTUAL);
}

/*
 * A coprocessor CPU's run bit, read only where it can answer: its domain at full power. Below
 * that the hardware has idled the block (these words are hardware-managed, PS_AUTO_ENABLE) and
 * a read of its registers can stall the bus; at .enter, with interrupts off on the last CPU,
 * nothing then runs until WD2 resets the SoC. On J815 every sleep that reached .enter did
 * exactly that (b28, boots 5343a2e5 to 03fd8d38; again after the port onto the per-SoC ops,
 * boot 14343f67: last PMU note 0x10), while every pm_test step up to core came back. An idled
 * CPU isn't running, so below full power this says no. Without a power word nothing says the
 * block is powered: no read either.
 */
static bool sleep_asc_running(struct apple_sleep *s, u32 ps, void __iomem *asc)
{
	return ps && asc && sleep_ps_full(s, ps) &&
	       (readl(asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN);
}

/* @pd's word in "ps": a power controller node, child of the PMGR that "ps" maps. */
static int sleep_ps_offset(struct apple_sleep *s, struct device_node *pd)
{
	struct device_node *pmgr __free(device_node) = NULL;
	struct resource res;
	u32 off;

	if (s->soc->regions)
		return sleep_region_domain_offset(s, pd);
	pmgr = of_get_parent(pd);
	if (!pmgr || of_address_to_resource(pmgr, 0, &res) || res.start != s->ps_res.start ||
	    of_property_read_u32(pd, "reg", &off) || off % 4 ||
	    off + 4 > resource_size(&s->ps_res))
		return -EINVAL;

	return off;
}

int apple_sleep_domain_check(struct device_node *pd)
{
	int off;

	if (!apple_sleep)
		return -ENODEV;
	off = sleep_ps_offset(apple_sleep, pd);
	return off < 0 ? off : 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_domain_check);

/*
 * A power domain lowered or raised by its own driver for a sleep, before the commit or after
 * a refused one, as macOS's drivers do through its PMGR. A lower waits for off. Until the
 * driver raises it, the PMGR's driver leaves it alone at the wake.
 */
int apple_sleep_domain_set(struct device_node *pd, bool on)
{
	struct apple_sleep *s = apple_sleep;
	int off, ret = 0;
	u32 val;
	unsigned int bit;
	unsigned long *down;

	if (!s)
		return -ENODEV;
	off = sleep_ps_offset(s, pd);
	if (off < 0)
		return off;

	down = sleep_domain_bitmap(s, off, &bit);
	if (!down)
		return -EINVAL;

	val = readl(sleep_power_reg(s, off));
	if (on) {
		if (FIELD_GET(PS_TARGET, val) != 0xf || FIELD_GET(PS_ACTUAL, val) != 0xf)
			ret = sleep_ps_raise(s, off);
		else if (!(val & PS_AUTO_ENABLE))
			sleep_ps_raise_finish(s, off);
		if (!ret)
			clear_bit(bit, down);
		return ret;
	}
	set_bit(bit, down);
	if (FIELD_GET(PS_TARGET, val))
		sleep_ps_lower(s, off);
	return sleep_ps_wait_off(s, off);
}
EXPORT_SYMBOL_GPL(apple_sleep_domain_set);

bool apple_sleep_domain_is_down(struct device_node *pd)
{
	struct apple_sleep *s = apple_sleep;
	unsigned long *down;
	unsigned int bit;
	int off;

	if (!s)
		return false;
	off = sleep_ps_offset(s, pd);
	if (off < 0)
		return false;
	down = sleep_domain_bitmap(s, off, &bit);
	return down && test_bit(bit, down);
}
EXPORT_SYMBOL_GPL(apple_sleep_domain_is_down);

/*
 * After a refused sleep, the sleep engine's own restart of a firmware's CPU whose domain shares
 * a power switch and can't gate alone, as SIO's: each of the device's own domains reset through
 * target 4 and 0, released with the domain off, then raised again, its DARTs' state kept. The
 * caller has cleared the run bit, and starts the firmware after.
 */
#define PS_BUSY			BIT(11)
#define PS_RESET		BIT(31)
#define PS_ENGINE_RESET_4	0x8f000244
#define PS_ENGINE_RESET_0	0x80000000
#define ASC_CPU_STATUS_READY	BIT(1)
#define ENGINE_RESET_PS		4
#define ENGINE_RESET_DARTS	2

static int sleep_engine_reset_wait(struct apple_sleep *s, u32 offset, u32 actual)
{
	u32 val;

	return readl_poll_timeout_atomic(sleep_power_reg(s, offset), val,
					 FIELD_GET(PS_ACTUAL, val) == actual && !(val & PS_BUSY),
					 10, PS_OFF_TIMEOUT_US);
}

/* A domain of the PMGR block mapped here as "ps", and its offset in it. */
static bool sleep_ps_domain(struct apple_sleep *s, struct device_node *pd, u32 *offset)
{
	int idx = of_property_match_string(s->dev->of_node, "reg-names", "ps");
	struct device_node *parent = of_get_parent(pd);
	struct resource ps, res;
	bool ours;

	ours = idx >= 0 && parent && !of_address_to_resource(s->dev->of_node, idx, &ps) &&
	       !of_address_to_resource(parent, 0, &res) && res.start == ps.start &&
	       !of_property_read_u32(pd, "reg", offset);
	of_node_put(parent);

	return ours;
}

/* The @index-th DART in @np's iommus, a reference held, or NULL past the last. */
static struct device *sleep_iommus_dart(struct device_node *np, int index)
{
	struct of_phandle_args args;
	struct platform_device *pdev;

	if (of_parse_phandle_with_args(np, "iommus", "#iommu-cells", index, &args))
		return NULL;
	pdev = of_find_device_by_node(args.np);
	of_node_put(args.np);

	return pdev ? &pdev->dev : NULL;
}

int apple_sleep_engine_reset(struct device *dev)
{
	struct apple_sleep *s = apple_sleep;
	struct device *darts[ENGINE_RESET_DARTS] = {};
	u32 ps[ENGINE_RESET_PS], saved[ENGINE_RESET_PS], val;
	struct of_phandle_iterator it;
	void __iomem *asc;
	int i, n = 0, idx, err, ret = 0;

	if (!s)
		return -ENODEV;
	if (s->soc->regions)
		return -EOPNOTSUPP; /* T6050 has no standalone engine reset here. */
	of_for_each_phandle(&it, err, dev->of_node, "power-domains", NULL, 0) {
		if (!sleep_ps_domain(s, it.node, &val))
			continue;
		if (n == ENGINE_RESET_PS) {
			of_node_put(it.node);
			return -E2BIG;
		}
		ps[n++] = val;
	}
	if (!n)
		return -ENODEV;

	idx = of_property_match_string(dev->of_node, "reg-names", "coproc");
	if (idx < 0)
		idx = of_property_match_string(dev->of_node, "reg-names", "asc");
	asc = idx >= 0 ? of_iomap(dev->of_node, idx) : NULL;
	if (!asc)
		return -ENODEV;

	for (i = 0; i < ENGINE_RESET_DARTS; i++) {
		darts[i] = sleep_iommus_dart(dev->of_node, i);
		if (!darts[i])
			break;
		apple_dart_state_save(darts[i]);
	}

	for (i = 0; i < n && !ret; i++) {
		saved[i] = readl(sleep_power_reg(s, ps[i]));
		writel(PS_ENGINE_RESET_4, sleep_power_reg(s, ps[i]));
		ret = sleep_engine_reset_wait(s, ps[i], 4);
		if (!ret) {
			writel(PS_ENGINE_RESET_0, sleep_power_reg(s, ps[i]));
			ret = sleep_engine_reset_wait(s, ps[i], 0);
		}
	}
	if (!ret) {
		usleep_range(1000, 2000);
		for (i = 0; i < n; i++)
			writel(0, sleep_power_reg(s, ps[i]));
		for (i = n - 1; i >= 0 && !ret; i--) {
			writel(PS_TARGET, sleep_power_reg(s, ps[i]));
			ret = sleep_engine_reset_wait(s, ps[i], FIELD_MAX(PS_ACTUAL));
			if (!ret)
				writel(saved[i] & ~(PS_FLAGS | PS_RESET | PS_ACTUAL),
				       sleep_power_reg(s, ps[i]));
		}
	}
	if (!ret)
		ret = readl_poll_timeout(asc + ASC_CPU_STATUS, val, val & ASC_CPU_STATUS_READY, 10,
					 100 * USEC_PER_MSEC);

	for (i = 0; i < ENGINE_RESET_DARTS && darts[i]; i++) {
		apple_dart_state_restore(darts[i]);
		put_device(darts[i]);
	}
	iounmap(asc);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_engine_reset);

/* The order macOS does it in, across clusters. */
static void sleep_dvfm_park(struct apple_sleep *s)
{
	int cluster, word;

	for (cluster = 0; cluster < SLEEP_CLUSTERS; cluster++) {
		s->dvfm_ctl[cluster] = readq(s->dvfm[cluster] + DVFM_CTL);
		for (word = 0; word < ARRAY_SIZE(sleep_dvfm_words); word++)
			s->dvfm_saved[cluster][word] =
				readq(s->dvfm[cluster] + sleep_dvfm_words[word]);
	}

	writeq(0, s->dvfm[0] + DVFM_CTL);
	writeq(0, s->dvfm[1] + DVFM_CTL);
	for (word = 0; word < 2; word++)
		for (cluster = 0; cluster < SLEEP_CLUSTERS; cluster++)
			writeq(s->dvfm_saved[cluster][word] & ~DVFM_ENABLE,
			       s->dvfm[cluster] + sleep_dvfm_words[word]);
	for (cluster = 0; cluster < SLEEP_CLUSTERS; cluster++)
		for (word = 2; word < 4; word++)
			writeq(s->dvfm_saved[cluster][word] & ~DVFM_ENABLE,
			       s->dvfm[cluster] + sleep_dvfm_words[word]);
	s->dvfm_parked = true;
}

static void sleep_dvfm_unpark(struct apple_sleep *s)
{
	int cluster, word;

	if (!s->dvfm_parked)
		return;
	s->dvfm_parked = false;

	writeq(s->dvfm_ctl[0], s->dvfm[0] + DVFM_CTL);
	writeq(s->dvfm_ctl[1], s->dvfm[1] + DVFM_CTL);
	for (cluster = 0; cluster < SLEEP_CLUSTERS; cluster++)
		for (word = 2; word < 4; word++)
			writeq(s->dvfm_saved[cluster][word],
			       s->dvfm[cluster] + sleep_dvfm_words[word]);
	for (word = 1; word >= 0; word--)
		for (cluster = 0; cluster < SLEEP_CLUSTERS; cluster++)
			writeq(s->dvfm_saved[cluster][word],
			       s->dvfm[cluster] + sleep_dvfm_words[word]);
}

/*
 * Every device has suspended by the last step, so a stray word still on has no user: it's
 * lowered there, and a sleep with one that won't go off is refused. They come back on after
 * the sleep or a refusal, for genpd's books.
 */
static void apple_sleep_stray_restore(struct apple_sleep *s)
{
	const struct apple_sleep_word *stray = s->soc->stray;
	int i;

	for (i = s->soc->nstray - 1; i >= 0; i--)
		if (s->stray_lowered & BIT(i))
			sleep_ps_raise(s, stray[i].offset);
	s->stray_lowered = 0;
}

static int apple_sleep_lower_stray(struct apple_sleep *s, bool dry)
{
	const struct apple_sleep_word *stray = s->soc->stray;
	int i, ret = 0;

	s->stray_lowered = 0;
	for (i = 0; i < s->soc->nstray; i++) {
		u32 off = stray[i].offset;

		if (!sleep_ps_on(s, off))
			continue;
		/* Its driver's own last stop lowers it, at the commit. */
		if (apple_sleep_coproc_lowers(off)) {
			dev_dbg(s->dev, "%s: left on for its driver's last stop\n",
				stray[i].name);
			continue;
		}
		/*
		 * A dry run writes no power word. J815's first one with every check passing reset
		 * the Mac by WD2 (boot 1e652ea3) right after the core step had passed, and this
		 * lowering, of a domain whose block may still run, was the only write in it.
		 */
		if (dry) {
			dev_info(s->dev, "%s is on at the last step: a real sleep lowers it here\n",
				 stray[i].name);
			continue;
		}
		sleep_ps_lower(s, off);
		s->stray_lowered |= BIT(i);
		if (sleep_ps_wait_off(s, off)) {
			dev_err(s->dev, "refused: %s won't go off: %#x\n",
				stray[i].name, readl(sleep_power_reg(s, off)));
			ret = -EBUSY;
		} else {
			dev_info(s->dev, "%s was on at the last step, lowered\n",
				 stray[i].name);
		}
	}
	if (ret)
		apple_sleep_stray_restore(s);

	return ret;
}

/*
 * The SoC's coprocessors that must be bound for a sleep, the ones that must also have stopped
 * by the last step, and the domains macOS has off by then. Taken in process context, since
 * the checks run with interrupts off. A driver whose stop failed has already failed the
 * suspend, naming itself, so what's left is whether it's bound at all, and what the hardware
 * shows.
 */
static struct device_node *sleep_gate_node(const struct apple_sleep_gate_coproc *g)
{
	struct device_node *np;
	struct resource res;
	unsigned int i;

	if (g->path)
		return of_find_node_by_path(g->path);
	for_each_compatible_node(np, NULL, g->compatible) {
		if (!g->match_start)
			return np;
		for (i = 0; !of_address_to_resource(np, i, &res); i++)
			if (res.start == g->match_start)
				return np;
	}
	return NULL;
}

static void apple_sleep_gate_prepare(struct apple_sleep *s)
{
	const struct apple_sleep_gate_coproc *coprocs = s->soc->gate_coprocs;
	int i;

	s->gate_count = 0;
	for (i = 0; i < s->soc->ngate_coprocs && s->gate_count < ARRAY_SIZE(s->gate); i++) {
		struct device_node *c = sleep_gate_node(&coprocs[i]);
		struct platform_device *pdev = c ? of_find_device_by_node(c) : NULL;
		struct apple_sleep_gate *g;

		if (!c) {
			dev_warn(s->dev, "%s isn't in this tree\n",
				 coprocs[i].path ?: coprocs[i].compatible);
			if (s->soc->strict_gate_inventory) {
				g = &s->gate[s->gate_count++];
				g->name = coprocs[i].compatible;
				g->bound = false;
				g->strict_unbound = true;
			}
			continue;
		}
		g = &s->gate[s->gate_count++];
		g->name = c->name;
		g->np = c;
		g->bound = pdev && device_is_bound(&pdev->dev);
		g->sep = of_node_name_eq(c, "sep");
		g->must_stop = coprocs[i].must_stop;
		g->ps = coprocs[i].ps;
		g->allow_unstarted = coprocs[i].allow_unstarted;
		g->strict_unbound = coprocs[i].strict_unbound;
		if ((g->must_stop || g->allow_unstarted) && !g->asc) {
			int idx = coprocs[i].explicit_run_index ? coprocs[i].run_index :
				of_property_match_string(c, "reg-names", "coproc");

			if (idx < 0)
				idx = of_property_match_string(c, "reg-names", "asc");
			if (idx >= 0 && (g->allow_unstarted || coprocs[i].explicit_run_index)) {
				struct resource res;

				if (of_address_to_resource(c, idx, &res) ||
				    resource_size(&res) < ASC_CPU_CONTROL + sizeof(u32))
					idx = -EINVAL;
			}
			if (idx >= 0)
				g->asc = of_iomap(c, idx);
		}
		if (pdev)
			put_device(&pdev->dev);
		of_node_put(c);
	}
}

/*
 * Names everything wrong, so one boot shows the whole list. A coprocessor that must stop and
 * still runs refuses the sleep; the rest only warns, as a real sleep is the only full test.
 */
static bool apple_sleep_gate_failed(struct apple_sleep *s)
{
	int i, bad = 0, running = 0;

	for (i = 0; i < s->gate_count; i++) {
		struct apple_sleep_gate *g = &s->gate[i];

		/* Even an always-on domain may read actual0: require a prepared protocol. */
		if (g->sep && (!g->bound || !apple_sleep_enclave.stop)) {
			dev_err(s->dev, "refused: SEP has no prepared irreversible sleep request\n");
			bad++;
			running++;
			continue;
		}
		/* A RUN-clear image loaded by iBoot is not running firmware. Unknown
		 * mappings refuse; never probe the SMC's unsafe ASC status aperture.
		 */
		if (!g->bound && g->allow_unstarted) {
			if (g->ps && !sleep_ps_on(s, g->ps)) {
				dev_err(s->dev, "%s RUN state inaccessible; refusing unknown state\n", g->name);
				bad++;
				running++;
				continue;
			}
			if (g->asc && !(readl(g->asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN))
				continue;
			dev_err(s->dev, "%s running or RUN control unavailable without driver\n", g->name);
			bad++;
			running++;
			continue;
		}
		if (!g->bound) {
			dev_err(s->dev, "%s isn't bound\n", g->name);
			bad++;
			/* Nothing will stop it, and a commit over a running CPU resets the SoC. */
			if (g->must_stop && sleep_asc_running(s, g->ps, g->asc)) {
				dev_err(s->dev, "%s runs unbound: control %#x status %#x\n", g->name,
					readl(g->asc + ASC_CPU_CONTROL),
					readl(g->asc + ASC_CPU_STATUS));
				running++;
			}
			if (g->strict_unbound)
				running++;
			continue;
		}
		if (!g->must_stop)
			continue;
		/* A CPU whose domain is off isn't running, and its registers aren't there. */
		if (g->ps && !sleep_ps_on(s, g->ps))
			continue;
		/* Its driver quiesced it: as before, that answers for a run bit, now unread. */
		if (g->asc && apple_sleep_coproc_ready_node(g->np))
			continue;
		/* A firmware 27 display whose driver prepared its gate for the late stop. */
		if (g->asc && sleep_v27_gate_prepared(g->np))
			continue;
		/* Idled below full power, or stopped: its registers read only at full power. */
		if (g->asc && g->ps && !sleep_asc_running(s, g->ps, g->asc))
			continue;
		if (g->asc && g->ps && (g->allow_unstarted || s->soc->regions))
			dev_err(s->dev, "%s didn't stop: control %#x\n", g->name,
				readl(g->asc + ASC_CPU_CONTROL));
		else if (g->asc && g->ps)
			/* At full power (sleep_asc_running): its registers answer. */
			dev_err(s->dev, "%s didn't stop: control %#x status %#x\n", g->name,
				readl(g->asc + ASC_CPU_CONTROL), readl(g->asc + ASC_CPU_STATUS));
		else
			dev_err(s->dev, "%s didn't stop: %s\n", g->name,
				g->asc ? "no power word says its registers can be read" :
					 "no CPU registers mapped");
		bad++;
		running++;
	}

	sleep_note(s, 0x26);

	for (i = 0; i < s->soc->ngate_off; i++) {
		const struct apple_sleep_word *w = &s->soc->gate_off[i];

		if (sleep_ps_on(s, w->offset)) {
			dev_err(s->dev, "%s is on: %#x\n", w->name, readl(sleep_power_reg(s, w->offset)));
			bad++;
		}
	}

	if (running) {
		dev_err(s->dev, "refused: %d coprocessors that must stop or gate are still running\n",
			running);
		return true;
	}
	if (bad && s->soc->strict_gate_inventory)
		return true;
	if (bad)
		dev_warn(s->dev, "going ahead with %d problems\n", bad);
	else
		dev_info(s->dev, "%u coprocessors up and stopped\n", s->gate_count);

	return false;
}

static void sleep_wake_unmask(const struct apple_sleep *s, u32 *val, u32 id)
{
	if (id / 32 < s->soc->wake_words)
		val[id / 32] &= ~BIT(id % 32);
}

/*
 * Mask every source but the ones asked for. These words read all zero until
 * something writes them, which leaves every source enabled, listed in the ADT or
 * not. macOS reads all ones outside the ADT's ap-wake-sources (measured). The AOP's
 * source 0 is masked with the rest: that held under macOS with its handler hidden,
 * as did leaving it enabled with the AMCC enables off. The two together are unmeasured.
 */
static void sleep_wake_masks(struct apple_sleep *s, u32 *val)
{
	int i;

	for (i = 0; i < s->soc->wake_words; i++)
		val[i] = i == s->soc->wake_words - 1 ? s->soc->wake_last_word : U32_MAX;
	for (i = 0; i < s->wake_enable_count; i++)
		sleep_wake_unmask(s, val, s->wake_enable[i]);
}

/* The last checks, then the stray words lowered: what has stopped may not return until a wake. */
static int apple_sleep_refuse(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;
	bool refused = false, idled = false;
	int i;

	sleep_note(s, 0x20);
	for (i = 0; i < SLEEP_COPROCS; i++) {
		struct apple_sleep_coproc *c = &apple_sleep_coprocs[i];

		if (!c->dev)
			continue;
		if (c->ready) {
			idled |= c->pcie_ans;
		} else {
			/* Nothing may lower a coprocessor's domain under its driver. */
			dev_err(s->dev, "refused: %s is running and its driver has not quiesced it\n",
				dev_name(c->dev));
			refused = true;
		}
	}

	if (num_online_cpus() != 1 ||
	    (read_cpuid_mpidr() & MPIDR_HWID_BITMASK) != s->mbox->boot_mpidr) {
		dev_err(s->dev, "refused: wrong CPU, or other CPUs online\n");
		refused = true;
	}

	/* Only a driver takes the display to where its domain can be lowered. */
	if (!s->display[0].bound) {
		dev_err(s->dev, "refused: no driver bound to the display coprocessor\n");
		refused = true;
	}

	/*
	 * With nothing to wake it the sleep ends only at the watchdog. Where it's optional in
	 * the tree, this is the only way a sleep gets past probe without any.
	 */
	if (!s->soc->own_wake && !s->wake_enable_count) {
		dev_err(s->dev, "refused: no wake source enabled in the device tree\n");
		refused = true;
	}

	sleep_note(s, 0x21);
	/* Its first config access after the power loss would hang on the untrained link. */
	if (s->radio_host_np && !s->radio_trainable) {
		struct platform_device *host = of_find_device_by_node(s->radio_host_np);

		if (host && READ_ONCE(host->dev.driver)) {
			dev_err(s->dev, "refused: %s holds the radio's PCIe link, and its port can't be trained again after a real sleep\n",
				dev_name(&host->dev));
			refused = true;
		}
		if (host)
			put_device(&host->dev);
	}

	sleep_note(s, 0x22);
	/* ANS's link needs these after a real power loss. */
	if (idled && !s->soc->regions && !s->apcie_phy_tunable) {
		dev_err(s->dev, "refused: no tunables to bring ANS's PCIe block back\n");
		refused = true;
	}

	/* Only ANS's driver can stop it, and it doesn't come back without a real sleep. */
	if (!idled && (sleep_ps_on(s, d->ans) || sleep_ps_on(s, d->apcie_sys_gp))) {
		dev_err(s->dev, "refused: ANS or PCIe is on with no driver to stop it\n");
		refused = true;
	}

	sleep_note(s, 0x23);
	/* The engine waits for DISP_CPU with no timeout, so this must never slip. */
	for (i = 0; i < s->soc->ndisp; i++) {
		u32 ps = d->disp[i][0];

		/*
		 * T8142's external DCPs have no sleep stop in their driver (no s2r_ext, not
		 * T6050's late stop): with apple_sleep.dispext the display's lowering stops each
		 * one's CPU cold, as for an unbound DCP, so a running one is no refusal.
		 */
		if (i != 0 && s->soc->dispext_optional) {
			/*
			 * Undescribed (no phandle), it can't be stopped here, and iBoot leaves its
			 * firmware running with the domain up: the engine would wait on it.
			 */
			if (READ_ONCE(sleep_dispext) && !s->display[i].np && sleep_ps_on(s, ps)) {
				dev_err(s->dev,
					"refused: external display coprocessor %d is on (%#x: %#x) and not in apple,display-coprocessors\n",
					i - 1, ps, readl(sleep_power_reg(s, ps)));
				refused = true;
			}
			continue;
		}
		if (s->display[i].bound && sleep_asc_running(s, ps, s->display[i].asc) &&
		    (!apple_sleep_v27_late() || sleep_v27_final_check(i))) {
			dev_err(s->dev, "refused: a bound display coprocessor is still running\n");
			refused = true;
		}
	}

	sleep_note(s, 0x24);
	if (apple_sleep_gate_failed(s))
		refused = true;

	if (refused)
		return -EBUSY;

	sleep_note(s, 0x28);
	/* A dry run, which stops right after this, writes no power word (as in b27). */
	return apple_sleep_lower_stray(s, s->soc->untested && !READ_ONCE(sleep_untested_commit));
}

static void sleep_wake_arm(struct apple_sleep *s)
{
	u32 val[WAKE_WORDS];
	int i;

	for (i = 0; i < s->soc->wake_words; i++)
		s->wake_saved[i] = readl(s->wake + s->soc->wake_mask + i * 4);
	sleep_wake_masks(s, val);
	for (i = 0; i < s->soc->wake_words; i++) {
		writel(val[i], s->wake + s->soc->wake_mask + i * 4);
		s->wake_armed[i] = ~val[i];
	}
}

static void sleep_wake_restore(struct apple_sleep *s)
{
	int i;

	for (i = 0; i < s->soc->wake_words; i++)
		writel(s->wake_saved[i], s->wake + s->soc->wake_mask + i * 4);
}

/*
 * The sleep engine waits on DISP_CPU with no timeout, so the display domains
 * must be off. A coprocessor's driver stops its firmware first, and nothing here
 * talks to the firmware. Otherwise no driver is bound, and the bootloader left the
 * firmware running: on the DCP, stop its CPU cold. That's unproven. The coprocessor
 * restarts from its reset vector after any sleep, and the engine itself clears the
 * DCP's run bit on the way down.
 *
 * Domain order and values as macOS's: DCPEXT then DCP, CPU, front end, system.
 * The rest of the display's domains go off with these, by nothing Linux or macOS
 * writes. The coprocessors' secure mailboxes are not used here. Whether that
 * holds is open.
 */
static bool sleep_dispext_skip(struct apple_sleep *s, int i)
{
	return i != 0 && s->soc->dispext_optional &&
	       (!READ_ONCE(sleep_dispext) || !s->display[i].np);
}

/*
 * apple_sleep.dispext: an external display's words back as the lowering found them, parents
 * first (SYS, FE, CPU), after a refusal or at the wake (the commit saw them off, so the rewake
 * leaves them). Its firmware was stopped cold; its driver's resume starts it again.
 */
static void sleep_dispext_restore(struct apple_sleep *s)
{
	const u32 (*domains)[3] = s->soc->ps.disp;
	int i, k;

	for (i = 1; i < s->soc->ndisp; i++) {
		if (!(s->dispext_lowered & BIT(i)))
			continue;
		for (k = 2; k >= 0; k--)
			if (FIELD_GET(PS_TARGET, s->dispext_saved[i][k]))
				sleep_ps_restore(s->ps + domains[i][k], s->dispext_saved[i][k]);
	}
	s->dispext_lowered = 0;
	/*
	 * An external display's ADMA once its SYS word is up again. The internal display's stays
	 * off: no Linux driver uses it, and its SYS word comes back with the DCP's own resume.
	 */
	for (i = 1; i < s->soc->ndisp; i++) {
		if (!(s->disp_adma_lowered & BIT(i)))
			continue;
		if (FIELD_GET(PS_ACTUAL, readl(s->ps + domains[i][2])))
			sleep_ps_restore(sleep_power_reg(s, s->soc->disp_adma[i]),
					 s->disp_adma_saved[i]);
		s->disp_adma_lowered &= ~BIT(i);
	}
}

static int sleep_display_off(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	const u32 (*domains)[3] = soc->ps.disp;
	int n = soc->ndisp, j, i, ret = 0;

	s->dispext_lowered = 0;
	s->disp_adma_lowered = 0;
	s->extra_stuck = 0;
	s->dispext_note = soc->dispext_optional && READ_ONCE(sleep_dispext) ? 1 : 0;
	for (j = 0; j < n; j++) {
		void __iomem *asc;

		i = soc->display_order[j];
		asc = s->display[i].asc;

		if (sleep_dispext_skip(s, i))
			continue;
		/*
		 * T8140's engine doesn't wait for its external display. On T8132 macOS has them
		 * all off.
		 */
		if (i != 0 && !s->display[i].bound) {
			if (!s->soc->lower_unbound_displays)
				continue;
			/* Otherwise writing its CPU word faults. */
			if (s->ps_prot && soc->disp_prot_slice[i]) {
				void __iomem *slice;

				slice = s->ps_prot + PS_PROT_SLICE(soc->disp_prot_slice[i]);
				writel(readl(slice) & ~PS_PROT_SLICE_ON, slice);
			}
		}
		if (i != 0 && soc->dispext_optional) {
			int k;

			for (k = 0; k < 3; k++)
				s->dispext_saved[i][k] = readl(s->ps + domains[i][k]);
			s->dispext_lowered |= BIT(i);
			/* +22 bits 1/2 (DISPEXT0) and 3/4 (DISPEXT1): CPU on, FE or SYS on */
			if (i <= 2 && FIELD_GET(PS_ACTUAL, s->dispext_saved[i][0]))
				s->dispext_note |= BIT(2 * i - 1);
			if (i <= 2 && (FIELD_GET(PS_ACTUAL, s->dispext_saved[i][1]) ||
				       FIELD_GET(PS_ACTUAL, s->dispext_saved[i][2])))
				s->dispext_note |= BIT(2 * i);
			if (sleep_asc_running(s, domains[i][0], asc))
				s->dispext_note |= 0x20;
			if (soc->disp_wr_dis[i][0]) {
				void __iomem *wr = s->ps + soc->disp_wr_dis[i][0];
				u32 bit = BIT(soc->disp_wr_dis[i][1]);

				if (readl(wr) & bit) {
					writel(readl(wr) & ~bit, wr);
					s->dispext_note |= 0x40;
				}
			}
		}

		/* apple_sleep.extra_off: the display's ADMA, a child of its SYS word, first */
		if (soc->disp_adma[i] && READ_ONCE(sleep_extra_off)) {
			u32 a = soc->disp_adma[i], v = readl(sleep_power_reg(s, a));

			s->disp_adma_saved[i] = v;
			if (FIELD_GET(PS_ACTUAL, v)) {
				sleep_ps_lower(s, a);
				s->disp_adma_lowered |= BIT(i);
				if (sleep_ps_wait_off(s, a))
					s->extra_stuck |= BIT(31);
			}
		}

		if (sleep_asc_running(s, domains[i][0], asc)) {
			writel(0, asc + ASC_CPU_CONTROL);
			writel(0, asc + ASC_CPU_CONTROL);
		}

		/* Reads as a count of the DCPs powered, and macOS ends at 0. */
		if (s->pmc_dcp)
			writel(n - 1 - j, s->pmc_dcp);
		if (soc->pwrgate_disp[i][0]) {
			if (readl(s->ps + domains[i][0]) & PS_AUTO_ENABLE)
				writel(readl(s->ps + domains[i][0]) &
				       ~(PS_AUTO_ENABLE | PS_FLAGS), s->ps + domains[i][0]);
			writel(3, s->pwrgate + soc->pwrgate_disp[i][0]);
			writel(0, s->pwrgate + soc->pwrgate_disp[i][1]);
		}
		sleep_ps_lower(s, domains[i][0]);
		sleep_ps_lower(s, domains[i][1]);
		sleep_ps_lower(s, domains[i][2]);
	}

	for (i = 0; i < n; i++) {
		int err;

		if (sleep_dispext_skip(s, i))
			continue;
		if (i != 0 && !s->display[i].bound && !s->soc->lower_unbound_displays)
			continue;

		err = sleep_ps_wait_off(s, domains[i][0]);

		if (err) {
			dev_err(s->dev, "refused: display domain %#x stayed on: %#x\n",
				domains[i][0], readl(s->ps + domains[i][0]));
			if (i != 0 && soc->dispext_optional)
				s->dispext_note |= 0x80;
			ret = err;
		}
	}
	if (soc->dispext_optional)
		sleep_note_at(s, 22, s->dispext_note);

	return ret;
}

/* One command on the port controller's bus, polled: no other CPU, and interrupts are off. */
static int sleep_hpm_cmd(struct apple_sleep *s, u32 op, u32 extra, const u32 *in, u32 *out)
{
	void __iomem *b = s->hpm_spmi;
	u32 status, reply;
	int ret;

	while (!(readl(b + SPMI_STATUS) & SPMI_STATUS_RX_EMPTY))
		readl(b + SPMI_REPLY);

	writel(extra << 16 | SPMI_CMD_ACTIVE | s->hpm_sid << 8 | op, b + SPMI_CMD);
	if (in)
		writel(*in, b + SPMI_CMD);
	ret = readl_poll_timeout_atomic(b + SPMI_STATUS, status,
					!(status & SPMI_STATUS_RX_EMPTY), 10, SPMI_POLL_US);
	if (ret)
		return ret;
	reply = readl(b + SPMI_REPLY);
	writel(1, b + SPMI_NOTIFY_ACK);
	if (out) {
		ret = readl_poll_timeout_atomic(b + SPMI_STATUS, status,
						!(status & SPMI_STATUS_RX_EMPTY), 10,
						SPMI_POLL_US);
		if (ret)
			return ret;
		*out = readl(b + SPMI_REPLY);
	}

	/* A zero write's reply carries the value written. */
	return op == SPMI_OP_ZERO_WRITE || (reply & 0xff) == op ? 0 : -EIO;
}

static int sleep_hpm_select(struct apple_sleep *s, u32 page)
{
	u32 val = ~0U;
	int i, ret;

	ret = sleep_hpm_cmd(s, SPMI_OP_ZERO_WRITE, page << 8, NULL, NULL);
	for (i = 0; !ret && i < 2000; i++) {
		ret = sleep_hpm_cmd(s, SPMI_OP_EXT_READ(1), HPM_REG_SELECT, NULL, &val);
		if (!ret && (val & 0xff) == page)
			return 0;
		/* A select it dropped, waking, is sent again. page | 0x80 is one in progress. */
		if (!ret && (val & 0xff) != (page | 0x80) && i % 20 == 19) {
			ret = sleep_hpm_cmd(s, SPMI_OP_WAKEUP, 0, NULL, NULL);
			if (!ret)
				ret = sleep_hpm_cmd(s, SPMI_OP_ZERO_WRITE, page << 8, NULL, NULL);
		}
		udelay(10);
	}

	return ret ?: -ETIMEDOUT;
}

/*
 * What macOS sends the other port controller about a second before a sleep, measured
 * in three runs: system power state 3, then the bus's sleep command.
 */
static int sleep_hpm_sleep(struct apple_sleep *s)
{
	u32 data = HPM_SSPS_S3, cmd = HPM_CMD_SSPS, val = ~0U;
	int i, ret;

	ret = sleep_hpm_cmd(s, SPMI_OP_WAKEUP, 0, NULL, NULL);
	if (!ret)
		ret = sleep_hpm_select(s, HPM_PAGE_DATA1);
	if (!ret)
		ret = sleep_hpm_cmd(s, SPMI_OP_EXT_WRITE(1), HPM_REG_DATA_CMD, &data, NULL);
	if (!ret)
		ret = sleep_hpm_select(s, HPM_PAGE_CMD1);
	if (!ret)
		ret = sleep_hpm_cmd(s, SPMI_OP_EXT_WRITE(4), HPM_REG_DATA_CMD, &cmd, NULL);
	/* Selected afresh for each read, as macOS does. */
	for (i = 0; !ret && val && i < 100; i++) {
		udelay(1000);
		ret = sleep_hpm_select(s, HPM_PAGE_CMD1);
		if (!ret)
			ret = sleep_hpm_cmd(s, SPMI_OP_EXT_READ(4), HPM_REG_DATA, NULL, &val);
	}
	if (!ret && val)
		ret = -ETIMEDOUT;
	/* Asleep either way: a failure here only means it wasn't told first. */
	sleep_hpm_cmd(s, SPMI_OP_SLEEP, 0, NULL, NULL);

	return ret;
}

/*
 * The partner takes the reset as an unplug, attaches again and renegotiates its contract.
 * Left to finish across the commit, that sometimes failed: neither side had a contract or
 * saw the other, and no VDM got through until a replug. Returns the ms it took.
 */
static int sleep_hpm_wait_contract(struct apple_sleep *s, int *gone_ms)
{
	ktime_t start = ktime_get();
	bool wake = true, up;
	u32 val = 0;
	int ms, ret;

	/*
	 * It reads back as before for a moment after the reset. Count it as back once it was
	 * seen gone (no contract, or no answer while the controller restarts), or after the
	 * floor if it never seemed to go.
	 */
	*gone_ms = -1;
	mdelay(HPM_RESET_FIRST_MS);
	for (;;) {
		ms = ktime_ms_delta(ktime_get(), start);
		if (ms >= HPM_CONTRACT_MS)
			return -ETIMEDOUT;
		ret = wake ? sleep_hpm_cmd(s, SPMI_OP_WAKEUP, 0, NULL, NULL) : 0;
		if (!ret)
			ret = sleep_hpm_select(s, HPM_PAGE_POWER_STATUS);
		if (!ret)
			ret = sleep_hpm_cmd(s, SPMI_OP_EXT_READ(4), HPM_REG_DATA, NULL, &val);
		up = !ret && (val & HPM_POWER_CONNECTED) &&
		     FIELD_GET(HPM_POWER_TYPEC_MODE, val) == HPM_POWER_TYPEC_PD;
		if (!up && *gone_ms < 0)
			*gone_ms = ms;
		if (up && (*gone_ms >= 0 || ms >= HPM_RESET_FLOOR_MS))
			return ms;
		wake = ret;
		mdelay(10);
	}
}

/* A live debug link holds the debug switch domain on. */
static bool sleep_debug_link_up(struct apple_sleep *s)
{
	return s->hpm_spmi &&
	       FIELD_GET(PS_ACTUAL, readl(s->ps_aon + s->soc->ps_aon_debug_switch));
}

/*
 * The debug link stays up with the ATC0 domains off, and it holds the debug domains
 * active, which macOS has idle. The only known way to end it is the port controller's
 * cold reset. To the SMC that's a cable pulled: it records the port's events as wake
 * reasons once it has been warned of the sleep, and wakes the AP right after the commit.
 * So the SMC's driver warns it only after this. tipd rearms the link after a refusal.
 */
static bool apple_sleep_usb_down(struct apple_sleep *s)
{
	u32 cmd = HPM_CMD_COLD_RESET;
	u32 enable;
	int i, ret, ms, gone;

	if (s->usb_down_ran)
		return false;
	s->usb_down_ran = true;

	if (!sleep_debug_link_up(s))
		return false;

	dev_info(s->dev, "debug link: cutting it for the sleep\n");
	enable = readl(s->hpm_spmi + SPMI_NOTIFY_ENABLE);
	writel(enable & ~SPMI_NOTIFY_REPLY, s->hpm_spmi + SPMI_NOTIFY_ENABLE);
	/* A reset that the partner doesn't come back from is done again. */
	for (i = 1; i <= HPM_RESET_TRIES; i++) {
		ret = sleep_hpm_cmd(s, SPMI_OP_WAKEUP, 0, NULL, NULL);
		if (!ret)
			ret = sleep_hpm_select(s, HPM_PAGE_CMD1);
		if (!ret)
			ret = sleep_hpm_cmd(s, SPMI_OP_EXT_WRITE(4), HPM_REG_DATA, &cmd, NULL);
		if (ret)
			break;
		WRITE_ONCE(sleep_debug_link_cut, true);
		ms = sleep_hpm_wait_contract(s, &gone);
		if (ms >= 0) {
			dev_info(s->dev, "debug link: contract back %d ms after reset %d (gone at %d ms)\n",
				 ms, i, gone);
			break;
		}
		dev_warn(s->dev, "debug link: no contract with the partner after reset %d\n", i);
	}
	if (!ret)
		ret = sleep_hpm_sleep(s);
	writel(1, s->hpm_spmi + SPMI_NOTIFY_ACK);
	writel(enable, s->hpm_spmi + SPMI_NOTIFY_ENABLE);
	s->usb_down_ret = ret;

	return true;
}

bool apple_sleep_debug_link_cut(void)
{
	return apple_sleep ? apple_sleep_usb_down(apple_sleep) : false;
}
EXPORT_SYMBOL_GPL(apple_sleep_debug_link_cut);

bool apple_sleep_debug_link_up(void)
{
	struct apple_sleep *s = apple_sleep;

	return s && !s->usb_down_ran && sleep_debug_link_up(s);
}
EXPORT_SYMBOL_GPL(apple_sleep_debug_link_up);

/*
 * macOS takes the wake for a lid closing during the sleep as a dark one: every driver wakes,
 * the display stays off, and the root domain sleeps again as soon as it sees the lid closed.
 */
void apple_sleep_lid_closed_wake(void)
{
	struct apple_sleep *s = apple_sleep;

	if (!s || !s->smc_woke_alone)
		return;
	dev_info(s->dev, "the lid closed during the sleep: dark wake\n");
	pm_set_dark_wake();
}
EXPORT_SYMBOL_GPL(apple_sleep_lid_closed_wake);

/*
 * macOS leaves a wake for a charger or a cable alone a dark one. With the lid closed
 * nothing there is for the user: sleep again.
 */
void apple_sleep_port_wake(bool lid_closed)
{
	struct apple_sleep *s = apple_sleep;

	if (!s || !s->smc_woke_alone || !lid_closed)
		return;
	dev_info(s->dev, "woken for a port with the lid closed: dark wake\n");
	pm_set_dark_wake();
}
EXPORT_SYMBOL_GPL(apple_sleep_port_wake);

/* Read in the SMC's noirq resume. Forgotten as each sleep begins. */
static int sleep_lid_closed = -ENODATA;

void apple_sleep_lid_at_wake(bool closed)
{
	WRITE_ONCE(sleep_lid_closed, closed);
}
EXPORT_SYMBOL_GPL(apple_sleep_lid_at_wake);

int apple_sleep_lid_closed_at_wake(void)
{
	return READ_ONCE(sleep_lid_closed);
}
EXPORT_SYMBOL_GPL(apple_sleep_lid_closed_at_wake);

/* Set in the SMC's noirq resume. Forgotten as each sleep begins. */
static int sleep_radio_woke = -ENODATA;

void apple_sleep_radio_at_wake(bool woke)
{
	WRITE_ONCE(sleep_radio_woke, woke);
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_at_wake);

int apple_sleep_radio_woke(void)
{
	return READ_ONCE(sleep_radio_woke);
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_woke);

/*
 * The SMC woke the AP for the port alone, and this sleep cut the debug link: the cut's own
 * events. Sleep again. The link stays down until tipd rearms it at a full wake, so the next
 * entry doesn't cut it.
 */
void apple_sleep_debug_link_cut_wake(void)
{
	struct apple_sleep *s = apple_sleep;

	if (!s || !s->smc_woke_alone || !READ_ONCE(sleep_debug_link_cut))
		return;
	dev_info(s->dev, "debug link: its cut woke the sleep: dark wake\n");
	pm_set_dark_wake();
}
EXPORT_SYMBOL_GPL(apple_sleep_debug_link_cut_wake);

/*
 * The radio's link and the engine that starts it, in macOS's order, measured on one
 * sleep: the link into L2, the engine stopped, PERST# asserted, the PHY's clocks off.
 * macOS does it 40 ms before ANS's idle. Here it follows, so that a refused sleep finds
 * the link untouched. A link that's down or a radio that's gone only makes the poll time
 * out, and the power goes next either way.
 */
static void sleep_apcie_port_stop(struct apple_sleep *s)
{
	void __iomem *port = s->apcie_port;
	void __iomem *phy = s->apcie_port_phy;
	u32 val;

	writel(readl(port + APCIE_PORT_INTMSK) | APCIE_PORT_INT_LINK, port + APCIE_PORT_INTMSK);
	writel(APCIE_PORT_INT_LINK, port + APCIE_PORT_INTSTAT);

	writel(readl(port + APCIE_PORT_APPCLK) | APCIE_PORT_APPCLK_CGDIS, port + APCIE_PORT_APPCLK);
	/* A loader can leave the port ready with the link never trained. */
	if (readl(port + APCIE_PORT_LINKSTS) & APCIE_PORT_LINKSTS_UP) {
		writel(APCIE_PORT_L2_ENTER, port + APCIE_PORT_L2);
		readl_poll_timeout_atomic(port + APCIE_PORT_L2, val, !(val & APCIE_PORT_L2_BUSY),
					  10, APCIE_PORT_STOP_US);
	}
	/* The radio keeps its power and firmware, in reset until its host releases it. */
	if (s->radio_perst >= 0 && sleep_ps_on(s, s->soc->ps.gpio)) {
		void __iomem *reg = s->pins + PIN_REG(s->radio_perst);

		val = readl(reg);
		writel(s->radio_perst_low ? val & ~PIN_DATA : val | PIN_DATA, reg);
	}
	writel(readl(phy) & ~APCIE_PORT_PHY_LINK, phy);

	if (s->apcie_engine && s->apcie_fabric) {
		writel(0, s->apcie_engine + APCIE_ENGINE_IRQ_MASK);
		writel(3, s->apcie_fabric + APCIE_FABRIC_STOP);
		writel(readl(s->apcie_engine + APCIE_ENGINE_CTRL) | APCIE_ENGINE_STOP,
		       s->apcie_engine + APCIE_ENGINE_CTRL);
	}
	udelay(1000);

	/* What this bit does isn't known. macOS sets it around PERST#. */
	writel(readl(port + APCIE_PORT_PERST_GUARD) | BIT(8), port + APCIE_PORT_PERST_GUARD);
	writel(0, port + APCIE_PORT_PERST);
	writel(readl(port + APCIE_PORT_PERST_GUARD) & ~BIT(8), port + APCIE_PORT_PERST_GUARD);

	writel(readl(phy) & ~APCIE_PORT_PHY_EN1, phy);
	writel(readl(phy) & ~APCIE_PORT_PHY_EN0, phy);
	writel(readl(phy) | APCIE_PHY_CTRL_RESET, phy);
	writel(readl(phy) & ~APCIE_PHY_CTRL_CLK0REQ, phy);
	readl_poll_timeout_atomic(phy, val, !(val & APCIE_PHY_CTRL_CLK0ACK), 1, APCIE_POLL_US);
	writel(readl(phy) & ~APCIE_PHY_CTRL_CLK1REQ, phy);

	writel(readl(port + APCIE_PORT_APPCLK) & ~APCIE_PORT_APPCLK_EN, port + APCIE_PORT_APPCLK);
	writel(readl(port + APCIE_PORT_INTSTAT), port + APCIE_PORT_INTSTAT);
}

static void sleep_apcie_port_stop_once(struct apple_sleep *s)
{
	/* Only with the port set up, by the loader or by its host after a wake, or it aborts. */
	if (s->apcie_port_stopped || s->apcie_port_down || !s->apcie_port || !s->apcie_port_phy ||
	    !sleep_ps_on(s, s->soc->ps.apcie_gp) || !sleep_ps_on(s, s->soc->ps.apcie_phy_sw))
		return;
	s->apcie_port_stopped = true;
	sleep_apcie_port_stop(s);
}

/*
 * A port that no host sets up again after a real sleep (one without the managed radio
 * contract) gets its clocks back as the loader set them at boot: the DART in its unit is
 * clocked by the port and its resume reads it. PERST# and the link stay as the wake left them.
 * It stops before the reset release when a clock doesn't ack: the wake's report names the port.
 */
static int sleep_apcie_port_clock(void __iomem *port, void __iomem *phy)
{
	u32 val;
	int ret;

	if (!port || !phy)
		return 0;
	writel(readl(port + APCIE_PORT_APPCLK) | APCIE_PORT_APPCLK_EN, port + APCIE_PORT_APPCLK);
	writel(readl(phy) & ~(APCIE_PHY_CTRL_CLK0REQ | APCIE_PHY_CTRL_CLK1REQ), phy);
	writel(readl(phy) | APCIE_PHY_CTRL_CLK0REQ, phy);
	ret = readl_poll_timeout_atomic(phy, val, val & APCIE_PHY_CTRL_CLK0ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(phy) | APCIE_PHY_CTRL_CLK1REQ, phy);
	ret = readl_poll_timeout_atomic(phy, val, val & APCIE_PHY_CTRL_CLK1ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(phy) & ~APCIE_PHY_CTRL_RESET, phy);
	writel(readl(phy) | APCIE_PORT_PHY_EN0, phy);
	writel(readl(phy) | APCIE_PORT_PHY_EN1, phy);
	return 0;
}

/* A poll here only paces the steps: the power goes next whatever it reads. */
static void sleep_apcie_phy_off(struct apple_sleep *s)
{
	void __iomem *ctrl = s->apcie_phy + APCIE_PHY_CTRL;
	void __iomem *run = s->apcie_phy + APCIE_PHY_RUN;
	u32 val;

	writel(readl(ctrl) | APCIE_PHY_CTRL_RESET, ctrl);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_CLK1REQ, ctrl);
	readl_poll_timeout_atomic(ctrl, val, !(val & APCIE_PHY_CTRL_CLK1ACK), 1, APCIE_POLL_US);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_CLK0REQ, ctrl);
	readl_poll_timeout_atomic(ctrl, val, !(val & APCIE_PHY_CTRL_CLK0ACK), 1, APCIE_POLL_US);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_UP, ctrl);
	writel(readl(run) & ~APCIE_PHY_RUN_EN2, run);
	writel(readl(run) & ~APCIE_PHY_RUN_EN, run);
}

static void sleep_apcie_clk_off(struct apple_sleep *s)
{
	void __iomem *clk = s->apcie_clkgen + APCIE_CLK_CTRL;
	u32 val;

	writel(readl(clk) & ~APCIE_CLK_RUN, clk);
	writel(readl(clk) | APCIE_CLK_OFF, clk);
	readl_poll_timeout_atomic(clk, val, !(val & APCIE_CLK_BUSY), 1, APCIE_POLL_US);
	writel(readl(clk) | APCIE_CLK_OFF2, clk);
}

/* The PHY's control words: at the start of "phy-block", or this far in (T8142). */
static void __iomem *sleep_apcie_ctrl(struct apple_sleep *s)
{
	return s->apcie_phy + (s->soc->apcie_phy_ctrl ?: APCIE_PHY_CTRL);
}

/*
 * APCIECoreRCGen4::_shutdownGen4Phy (AppleT8142PCIe, macOS 27.2.1): T8132's PHY stop with
 * the UP bit set first, two PHY-IP words and its delays around it.
 */
static void sleep_apcie_gen4_phy_off(struct apple_sleep *s)
{
	void __iomem *ctrl = sleep_apcie_ctrl(s);
	void __iomem *run = ctrl + (APCIE_PHY_RUN - APCIE_PHY_CTRL);
	void __iomem *ip = s->apcie_phy_ip;
	u32 val;

	writel(readl(ctrl) | APCIE_PHY_CTRL_UP, ctrl);
	udelay(10);
	writel(readl(ip + APCIE_GEN4_PHY_IP_38) & ~APCIE_GEN4_PHY_IP_38_EN,
	       ip + APCIE_GEN4_PHY_IP_38);
	writel(readl(ip + APCIE_GEN4_PHY_IP_8C) | APCIE_GEN4_PHY_IP_8C_OV,
	       ip + APCIE_GEN4_PHY_IP_8C);
	udelay(5);
	writel(readl(ctrl) | APCIE_PHY_CTRL_RESET, ctrl);
	udelay(1);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_CLK1REQ, ctrl);
	readl_poll_timeout_atomic(ctrl, val, !(val & APCIE_PHY_CTRL_CLK1ACK), 1, APCIE_POLL_US);
	udelay(1);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_CLK0REQ, ctrl);
	readl_poll_timeout_atomic(ctrl, val, !(val & APCIE_PHY_CTRL_CLK0ACK), 1, APCIE_POLL_US);
	udelay(1);
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_UP, ctrl);
	udelay(10);
	writel(readl(run) & ~APCIE_PHY_RUN_EN2, run);
	writel(readl(run) & ~APCIE_PHY_RUN_EN, run);
}

/*
 * APCIECoreRCGen4::_shutdownPciePLLs: the only pcieclkgen writes in the kext. Nothing starts
 * the PLLs again: they come back with the block's power.
 */
static void sleep_apcie_gen4_pll_off(struct apple_sleep *s)
{
	void __iomem *clk = s->apcie_clkgen + APCIE_CLK_CTRL;
	u32 val;

	writel((readl(clk) | APCIE_GEN4_CLK_HOLD) & ~APCIE_CLK_RUN, clk);
	writel(readl(clk) | APCIE_CLK_OFF, clk);
	writel(readl(clk) & ~APCIE_GEN4_CLK_EN, clk);
	readl_poll_timeout_atomic(clk, val, !(val & APCIE_CLK_BUSY), 1, APCIE_POLL_US);
	writel(readl(clk) | APCIE_CLK_OFF2, clk);
	writel((readl(clk) & ~APCIE_CLK_OFF) | APCIE_GEN4_CLK_EN, clk);
}

/*
 * T8142's, from AppleT8142PCIe (APCIECoreRCGen4's power-down, then its gate loop), with all
 * of the block's domains still on: the PHY stopped, the AUS5 domains off inside an UP-bit
 * window (only once the PHY's first clock request is down), the PLLs off, the AXI enables
 * cleared. Then the gate loop: gates 4 down to 0, SYS_ST, ST, SYS_GP, GP, ANS. PHY_SW is
 * left on, as macOS leaves it. The radio's port is stopped first (sleep_radio_port_stop()),
 * and its link is trained again after the wake (apple_sleep_radio_train()).
 */
static int sleep_radio_port_stop(struct apple_sleep *s);

static void apple_sleep_lower_apcie_gen4(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;
	void __iomem *ctrl = sleep_apcie_ctrl(s);
	bool on = sleep_ps_on(s, d->apcie_gp) && sleep_ps_on(s, d->apcie_sys_gp) &&
		  sleep_ps_on(s, d->apcie_st) && sleep_ps_on(s, d->apcie_sys_st) &&
		  sleep_ps_on(s, d->apcie_phy_sw);
	bool clk0;

	/* The radio's port first: its link runs on this block's PHY. */
	if (s->radio_trainable && on) {
		s->radio_stop_ret = sleep_radio_port_stop(s);
	} else if (s->radio_trainable) {
		s->radio_stop_step = "shared block already off";
		s->radio_stop_ret = -ENOLINK;
	}

	if (on && s->apcie_phy && s->apcie_phy_ip && sleep_ps_on(s, d->apcie_aus5_aon) &&
	    sleep_ps_on(s, d->apcie_aus5)) {
		udelay(10);
		sleep_apcie_gen4_phy_off(s);
		clk0 = readl(ctrl) & APCIE_PHY_CTRL_CLK0REQ;
		writel(readl(ctrl) | APCIE_PHY_CTRL_UP, ctrl);
		udelay(10);
		if (!clk0) {
			sleep_ps_lower(s, d->apcie_aus5);
			sleep_ps_lower(s, d->apcie_aus5_aon);
		}
		writel(readl(ctrl) & ~APCIE_PHY_CTRL_UP, ctrl);
	} else {
		sleep_ps_lower(s, d->apcie_aus5);
		sleep_ps_lower(s, d->apcie_aus5_aon);
	}
	if (on && s->apcie_clkgen)
		sleep_apcie_gen4_pll_off(s);
	if (on && s->apcie_axi) {
		writel(readl(s->apcie_axi + APCIE_AXI_EN0) & ~BIT(0), s->apcie_axi + APCIE_AXI_EN0);
		writel(readl(s->apcie_axi + APCIE_AXI_EN1) & ~BIT(0), s->apcie_axi + APCIE_AXI_EN1);
	}

	/*
	 * macOS's order on T8142 (26B131 trace, 13.0..12.8 ms before the timer): AUS5, AUS5_AON,
	 * PHY_SW, SYS_ST, ST, SYS_GP, GP, ANS. apple_sleep_raise_apcie_gen4() brings PHY_SW back
	 * after SYS_ST.
	 */
	sleep_ps_lower(s, d->apcie_phy_sw);
	sleep_ps_lower(s, d->apcie_sys_st);
	sleep_ps_lower(s, d->apcie_st);
	sleep_ps_lower(s, d->apcie_sys_gp);
	sleep_ps_lower(s, d->apcie_gp);
	sleep_ps_lower(s, d->ans);
}

/*
 * ANS's own link runs on this block, and macOS takes it down with the domains. Its
 * order, measured on one sleep, in the same microseconds as the power words. A block's
 * registers fault with its domain off, and a loader may have left any of these off.
 */
static void apple_sleep_lower_apcie(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;
	u32 val;

	if (s->soc->apcie_gen4) {
		apple_sleep_lower_apcie_gen4(s);
		return;
	}
	sleep_apcie_port_stop_once(s);
	/* The PHY's registers need both domains on. */
	if (s->apcie_phy && sleep_ps_on(s, d->apcie_phy_sw) && sleep_ps_on(s, d->apcie_sys_st))
		sleep_apcie_phy_off(s);
	sleep_ps_lower(s, d->apcie_phy_sw);

	if (s->apcie_clkgen && sleep_ps_on(s, d->apcie_sys_st))
		sleep_apcie_clk_off(s);
	sleep_ps_lower(s, d->apcie_sys_st);
	sleep_ps_lower(s, d->apcie_st);

	/*
	 * The AXI enables are the block's link into the fabric. macOS reads the domain on
	 * before clearing them: powered off with them set, SoC NI 1 latches a PCIe receive
	 * link error once the block's last domain goes off.
	 */
	val = sleep_ps_lower_start(s, d->apcie_sys_gp);
	if (s->apcie_axi && sleep_ps_wait_on(s, d->apcie_sys_gp, val)) {
		writel(readl(s->apcie_axi + APCIE_AXI_EN0) & ~BIT(0), s->apcie_axi + APCIE_AXI_EN0);
		writel(readl(s->apcie_axi + APCIE_AXI_EN1) & ~BIT(0), s->apcie_axi + APCIE_AXI_EN1);
	}
	sleep_ps_lower_finish(s, d->apcie_sys_gp, val);

	sleep_ps_lower(s, d->apcie_gp);
	sleep_ps_lower(s, d->ans);
}

/* Read-modify-write in table order, with no write where nothing changes. */
static void sleep_tunable_apply_changed(void __iomem *regs, const struct apple_tunable *t)
{
	size_t i;

	for (i = 0; i < t->sz; i++) {
		void __iomem *reg = regs + t->values[i].offset;
		u32 old = readl(reg);
		u32 val = (old & ~t->values[i].mask) | (t->values[i].value & t->values[i].mask);

		if (val != old)
			writel(val, reg);
	}
}

/*
 * The power loss takes the PHY's PLL and lane settings with it, and ANS's link can't
 * train without them. Applied as macOS does: the PLL table, then the lanes, then the
 * PLL's hold released. That takes about the settling time the PHY needs here.
 */
static void sleep_apcie_phy_ip_apply(struct apple_sleep *s)
{
	void __iomem *ctl = s->apcie_phy_ip + APCIE_PHY_IP_PLL_CTL;

	sleep_tunable_apply_changed(s->apcie_phy_ip, s->apcie_phy_ip_pll);
	sleep_tunable_apply_changed(s->apcie_phy_ip, s->apcie_phy_ip_lanes);
	writel(readl(ctl) & ~APCIE_PHY_IP_PLL_HOLD, ctl);
}

/*
 * T8142's, as AppleT8142PCIe brings the block up and as aura does at boot, where it trains
 * the radio's link after it (j813_n1.c): the domains with the AXI enables the lowering
 * cleared, the PHY table and its reference PLL, the PHY's clocks out of reset, its run
 * enables around the PLL and lane tables, the AXI tables, then the root complex. No clock
 * generator or CIO3 PLL tables. Never run after a sleep.
 */
static int apple_sleep_raise_apcie_gen4(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;
	void __iomem *ctrl = sleep_apcie_ctrl(s);
	void __iomem *run = ctrl + (APCIE_PHY_RUN - APCIE_PHY_CTRL);
	void __iomem *ready = ctrl + (APCIE_PHY_READY - APCIE_PHY_CTRL);
	void __iomem *common = s->apcie_phy + APCIE_PHY_COMMON;
	u32 val;
	int ret;

	ret = sleep_ps_raise(s, d->ans);
	ret |= sleep_ps_raise(s, d->apcie_gp);
	ret |= sleep_ps_raise_start(s, d->apcie_sys_gp);
	if (!ret) {
		writel(readl(s->apcie_axi + APCIE_AXI_EN0) | BIT(0), s->apcie_axi + APCIE_AXI_EN0);
		writel(readl(s->apcie_axi + APCIE_AXI_EN1) | BIT(0), s->apcie_axi + APCIE_AXI_EN1);
	}
	sleep_ps_raise_finish(s, d->apcie_sys_gp);
	ret |= sleep_ps_raise(s, d->apcie_st);
	ret |= sleep_ps_raise(s, d->apcie_sys_st);
	ret |= sleep_ps_raise(s, d->apcie_phy_sw);
	/* The gate loop powers all eight in the ADT's order: the AUS5 pair last. */
	ret |= sleep_ps_raise(s, d->apcie_aus5_aon);
	ret |= sleep_ps_raise(s, d->apcie_aus5);
	if (ret)
		return ret;

	/* _apcieGPInit: the PHY table, then its reference PLL's clock-good. */
	apple_tunable_apply(s->apcie_phy, s->apcie_phy_tunable);
	ret = readl_poll_timeout_atomic(common, val, val & BIT(31), 1, APCIE_POLL_US);
	if (ret)
		return ret;

	/* _initializeGen4Phy */
	writel(readl(ctrl) | APCIE_PHY_CTRL_CLK0REQ, ctrl);
	ret = readl_poll_timeout_atomic(ctrl, val, val & APCIE_PHY_CTRL_CLK0ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	udelay(1000);
	writel(readl(ctrl) | APCIE_PHY_CTRL_CLK1REQ, ctrl);
	ret = readl_poll_timeout_atomic(ctrl, val, val & APCIE_PHY_CTRL_CLK1ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_RESET, ctrl);
	udelay(1000);
	writel(readl(run) | APCIE_PHY_RUN_EN, run);
	sleep_apcie_phy_ip_apply(s);
	writel(readl(run) | APCIE_PHY_RUN_EN2, run);
	ret = readl_poll_timeout_atomic(ready, val, val & BIT(0), 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(common) | BIT(0), common);
	writel(readl(ctrl) | APCIE_PHY_CTRL_UP, ctrl);
	apple_tunable_apply(s->apcie_axi, s->apcie_axi_tunable);

	/* _enableRootComplex */
	writel(0x140, s->apcie_rc + APCIE_RC_READY_CFG);
	writel(readl(s->apcie_rc + APCIE_RC_READY_START) | BIT(0),
	       s->apcie_rc + APCIE_RC_READY_START);

	return readl_poll_timeout_atomic(s->apcie_rc + APCIE_RC_READY_STAT, val, val & BIT(0), 1,
					 APCIE_POLL_US);
}

/* T8142's radio port, as aura trains it at boot (j813_n1.c: APCIECoreRCGen4Port) */
#define RADIO_PORT_LTSSMCTL	0x80
#define RADIO_PORT_PIPE_STS	0xa8
#define RADIO_PORT_LINKSTS	0x208
#define RADIO_PORT_LINKSTS_UP	BIT(0)
#define RADIO_PORT_LINKSTS_BUSY	BIT(2)
#define RADIO_PORT_APPCLK	0x800
#define RADIO_PORT_STATUS	0x804
#define RADIO_PORT_PERST	0x82c
#define RADIO_LANE_REQ0		BIT(0)
#define RADIO_LANE_REQ1		BIT(1)
#define RADIO_LANE_ACK0		BIT(2)
#define RADIO_LANE_ACK1		BIT(3)
#define RADIO_LANE_HOLD		BIT(4)
#define RADIO_LANE_EN0		BIT(9)
#define RADIO_LANE_EN1		BIT(10)
#define RADIO_CFG_BUSES		0x18
#define RADIO_CFG_RO_WR		0x8bc
#define RADIO_LINK_US		1000000
#define PIN_MODE		GENMASK(3, 1)
#define PIN_MODE_OUT		1

/* APCIECoreRCGen4Port::_resetPortHardware */
static const u32 radio_port_reset[][2] = {
	{ 0x088, 0x110 },	{ 0x148, 0xffffffff },	{ 0x210, 0xffffffff },	{ 0x084, 0 },
	{ 0x104, 0xfffffff0 },	{ 0x124, 0x100 },	{ 0x16c, 0 },		{ 0x13c, 0 },
	{ 0x800, 0x100100 },	{ 0x808, 0x1001ff },	{ 0x82c, 0x10000 },	{ 0x130, 0x3020000 },
	{ 0x140, 0x10 },	{ 0x144, 0x253770 },	{ 0x834, 0 },		{ 0x83c, 0 },
};

static void sleep_radio_perst(struct apple_sleep *s, bool asserted)
{
	void __iomem *reg = s->pins + PIN_REG(s->radio_perst);
	bool high = asserted == !s->radio_perst_low;

	writel((readl(reg) & ~(PIN_MODE | PIN_DATA)) | FIELD_PREP(PIN_MODE, PIN_MODE_OUT) |
	       (high ? PIN_DATA : 0), reg);
}

/*
 * After the power loss the radio's link is down, and the generic host's first config
 * access to it would hang. Trained again as aura does it at boot, after the shared block
 * (apple_sleep_raise_apcie_gen4()): the radio held in reset, the port's reset values and
 * table, its application clock, the lane's reference clock buffer, PERST# released, the
 * root port's tables with its read-only registers opened, then the LTSSM. The SMC's N1
 * reset at boot isn't repeated: the SMC keeps its state across the sleep, as T8140's
 * radio keeps its rail. macOS 27.2.1's AppleCentauriManager doesn't repeat it at a wake
 * either: its function-reset (SMC gP13) runs at start, at shutdown, and to recover a
 * crashed driver or a chip found in ROM, after two tries of function-warm_reset (SMC
 * 'bpli'). Never run after a sleep.
 */
static int apple_sleep_radio_train(struct apple_sleep *s)
{
	void __iomem *port = s->radio_port, *lane = s->radio_lane, *cfg = s->radio_cfg;
	unsigned int i;
	u32 val;
	int ret;

	if (!sleep_ps_on(s, s->soc->ps.gpio))
		return -ENODEV;
	sleep_radio_perst(s, true);

	for (i = 0; i < ARRAY_SIZE(radio_port_reset); i++)
		writel(radio_port_reset[i][1], port + radio_port_reset[i][0]);
	apple_tunable_apply(port, s->radio_port_tunable);
	writel(readl(port + RADIO_PORT_APPCLK) | BIT(0), port + RADIO_PORT_APPCLK);

	/* APCIECoreRCGen4Port::_initializeRefclkBuffer */
	writel(readl(lane) | RADIO_LANE_REQ0, lane);
	ret = readl_poll_timeout_atomic(lane, val, val & RADIO_LANE_ACK0, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(lane) | RADIO_LANE_REQ1, lane);
	ret = readl_poll_timeout_atomic(lane, val, val & RADIO_LANE_ACK1, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(lane) & ~RADIO_LANE_HOLD, lane);
	udelay(1000);
	writel(readl(lane) | RADIO_LANE_EN0, lane);
	writel(readl(lane) | RADIO_LANE_EN1, lane);
	/* T(refclk-to-PERST#) */
	udelay(1000);

	writel(readl(port + RADIO_PORT_PERST) | BIT(0), port + RADIO_PORT_PERST);
	sleep_radio_perst(s, false);
	ret = readl_poll_timeout_atomic(port + RADIO_PORT_PIPE_STS, val, val & BIT(0), 1,
					APCIE_POLL_US * 2);
	if (!ret)
		ret = readl_poll_timeout_atomic(port + RADIO_PORT_STATUS, val, val & BIT(0), 1,
						APCIE_POLL_US * 5);
	if (!ret)
		ret = readl_poll_timeout_atomic(port + RADIO_PORT_LINKSTS, val,
						!(val & RADIO_PORT_LINKSTS_BUSY), 1,
						APCIE_POLL_US * 10);
	if (ret)
		return ret;

	/* _initializeRootComplex: its tables need the read-only registers open. */
	writel(readl(cfg + RADIO_CFG_RO_WR) | BIT(0), cfg + RADIO_CFG_RO_WR);
	for (i = 0; i < ARRAY_SIZE(s->radio_rc_tunable); i++)
		apple_tunable_apply(cfg, s->radio_rc_tunable[i]);
	writel(readl(cfg + RADIO_CFG_RO_WR) & ~BIT(0), cfg + RADIO_CFG_RO_WR);
	writel(1, port + RADIO_PORT_LTSSMCTL);
	ret = readl_poll_timeout_atomic(port + RADIO_PORT_LINKSTS, val,
					val & RADIO_PORT_LINKSTS_UP, 1, RADIO_LINK_US);
	if (ret)
		return ret;

	/* The root port's buses as the loader set them; the PCI core restores the rest. */
	writel(0x00010100, cfg + RADIO_CFG_BUSES);
	/* PCIe's 100 ms from PERST# release to the first configuration request */
	mdelay(100);

	return 0;
}

/*
 * The radio port's stop: APCIECoreRCGen4Port::disablePortHardware in macOS 27.2.1's
 * AppleT8142PCIe, the same class (APCIECoreRCGen4Port.cpp) the M5 Pro's AppleT6050PCIe
 * builds and pcie-apple.c's apple_pcie_t6050_port_disable() follows.
 */
#define RADIO_PORT_INTSTAT	0x100
#define RADIO_PORT_PERST_GUARD	0x13c
#define RADIO_PORT_LINKSTS_PHY_ALL0	BIT(12)
#define RADIO_PORT_OUTS_TRANS	0x820
#define RADIO_PORT_STATUS_RESET	BIT(2)
#define RADIO_LANE_CGEN		BIT(31)
#define RADIO_CFG_DEBUG0	0x728
#define RADIO_CFG_LTSSM		GENMASK(5, 0)
#define RADIO_CFG_LTSSM_L1_IDLE	0x14

/* One of disablePortHardware's waits: macOS logs a timeout and goes on, and so does this. */
static int sleep_radio_poll(struct apple_sleep *s, const char *step, void __iomem *reg,
			    u32 mask, u32 want, unsigned int us)
{
	u32 val;
	int ret = readl_poll_timeout_atomic(reg, val, (val & mask) == want, 1, us);

	if (ret && !s->radio_stop_step)
		s->radio_stop_step = step;
	return ret;
}

/*
 * The reverse of apple_sleep_radio_train(), just before the shared block's PHY stops and its
 * power goes, in disablePortHardware's order (27.2.1 AppleT8142PCIe; its timeouts in
 * brackets): PORT_STS's link reset flag (10 ms), the outstanding transactions drained
 * (10 ms), the PERST# guard bit, the link out of L1 (errata #55), PHY_STATUS_ALL0 (5 ms),
 * the lane's PIPE reset status (10 ms), the port's PERST# handshake with LANE_UCTRL_IDLE_ACK
 * (2 ms), PERST# asserted in the port, the lane's reference clocks off, the interrupts
 * cleared, the application clock off and the port's ready flag gone. Like macOS, a wait
 * that times out is only noted, and the rest still runs. Then PERST# on the pin, as macOS
 * holds the Neo's radio across a sleep (T8140, measured): in reset, its power and firmware
 * kept, until the wake's train releases it. Before this the N1 saw its link vanish with
 * PERST# released. Derived from the binary, never run.
 */
static int sleep_radio_port_stop(struct apple_sleep *s)
{
	void __iomem *port = s->radio_port, *lane = s->radio_lane, *cfg = s->radio_cfg;
	void __iomem *perst = port + RADIO_PORT_PERST;
	u32 val;
	int ret = 0, err;

	/* A port the loader never brought up has no link to stop. */
	if (!(readl(port + RADIO_PORT_STATUS) & BIT(0))) {
		s->radio_stop_step = "port not ready";
		ret = -ENOLINK;
		goto pin;
	}

	ret |= sleep_radio_poll(s, "PORT_STS link reset", port + RADIO_PORT_STATUS,
				RADIO_PORT_STATUS_RESET, RADIO_PORT_STATUS_RESET, 10000);
	err = readl_poll_timeout_atomic(port + RADIO_PORT_OUTS_TRANS, val,
					!(val & GENMASK(8, 0)) || !(val & GENMASK(23, 16)),
					1, 10000);
	if (err && !s->radio_stop_step)
		s->radio_stop_step = "OUTS_TRANS_STS";
	ret |= err;
	writel(readl(port + RADIO_PORT_PERST_GUARD) | BIT(8), port + RADIO_PORT_PERST_GUARD);

	err = read_poll_timeout_atomic(readl, val,
				       FIELD_GET(RADIO_CFG_LTSSM, val) != RADIO_CFG_LTSSM_L1_IDLE,
				       5, 5000, false, cfg + RADIO_CFG_DEBUG0);
	if (err && !s->radio_stop_step)
		s->radio_stop_step = "LTSSM stuck in L1 idle";
	ret |= err;
	ret |= sleep_radio_poll(s, "PHY_STATUS_ALL0", port + RADIO_PORT_LINKSTS,
				RADIO_PORT_LINKSTS_PHY_ALL0, RADIO_PORT_LINKSTS_PHY_ALL0, 5000);
	ret |= sleep_radio_poll(s, "LANE_PIPE_RESET_STATUS", port + RADIO_PORT_PIPE_STS, BIT(0),
				BIT(0), 10000);
	writel(readl(perst) | BIT(24), perst);
	ret |= sleep_radio_poll(s, "LANE_UCTRL_IDLE_ACK", port + RADIO_PORT_PIPE_STS, BIT(16),
				BIT(16), 2000);
	writel(readl(perst) | BIT(16), perst);
	udelay(1);
	writel(readl(perst) & ~BIT(0), perst);
	udelay(1);
	writel(readl(perst) & ~BIT(24), perst);
	writel(readl(port + RADIO_PORT_PERST_GUARD) & ~BIT(8), port + RADIO_PORT_PERST_GUARD);

	writel(readl(lane) & ~RADIO_LANE_CGEN, lane);
	writel(readl(lane) & ~RADIO_LANE_EN1, lane);
	writel(readl(lane) & ~RADIO_LANE_EN0, lane);
	writel(readl(lane) | RADIO_LANE_HOLD, lane);
	writel(readl(lane) & ~RADIO_LANE_REQ0, lane);
	writel(readl(lane) & ~RADIO_LANE_REQ1, lane);

	writel(~0, port + RADIO_PORT_INTSTAT);
	writel(readl(port + RADIO_PORT_APPCLK) & ~BIT(0), port + RADIO_PORT_APPCLK);
	ret |= sleep_radio_poll(s, "port stop", port + RADIO_PORT_STATUS, BIT(0), 0, 250000);
	ret = ret ? -ETIMEDOUT : 0;
pin:
	if (sleep_ps_on(s, s->soc->ps.gpio))
		sleep_radio_perst(s, true);
	return ret;
}

/*
 * Brought up from power-off in macOS's order, measured on one sleep: all of it follows
 * the domains on the waking core, well before ANS runs. Whether ANS needs each step is
 * inferred. Nothing else does this if no host driver is bound.
 */
static int apple_sleep_raise_apcie(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;
	void __iomem *ctrl = s->apcie_phy + APCIE_PHY_CTRL;
	void __iomem *run = s->apcie_phy + APCIE_PHY_RUN;
	u32 val;
	int ret;

	if (s->soc->apcie_gen4)
		return apple_sleep_raise_apcie_gen4(s);

	ret = sleep_ps_raise(s, d->ans);
	ret |= sleep_ps_raise(s, d->apcie_gp);
	ret |= sleep_ps_raise_start(s, d->apcie_sys_gp);
	if (!ret) {
		writel(readl(s->apcie_axi + APCIE_AXI_EN0) | BIT(0), s->apcie_axi + APCIE_AXI_EN0);
		writel(readl(s->apcie_axi + APCIE_AXI_EN1) | BIT(0), s->apcie_axi + APCIE_AXI_EN1);
	}
	sleep_ps_raise_finish(s, d->apcie_sys_gp);
	ret |= sleep_ps_raise(s, d->apcie_st);
	ret |= sleep_ps_raise(s, d->apcie_sys_st);
	if (ret)
		return ret;

	apple_tunable_apply(s->apcie_axi, s->apcie_axi_tunable);
	apple_tunable_apply(s->apcie_cio3pll, s->apcie_cio3pll_tunable);
	apple_tunable_apply(s->apcie_clkgen, s->apcie_clkgen_tunable);
	writel(readl(s->apcie_clkgen + APCIE_CLK_START) | APCIE_CLK_START_GO,
	       s->apcie_clkgen + APCIE_CLK_START);
	/* Both off bits: the lowering sets them, and only a power-off clears OFF. */
	writel(readl(s->apcie_clkgen + APCIE_CLK_CTRL) & ~(APCIE_CLK_OFF2 | APCIE_CLK_OFF),
	       s->apcie_clkgen + APCIE_CLK_CTRL);
	ret = readl_poll_timeout_atomic(s->apcie_clkgen + APCIE_CLK_CTRL, val,
					!(val & APCIE_CLK_BUSY), 1, APCIE_POLL_US);
	if (ret)
		return ret;

	ret = sleep_ps_raise(s, d->apcie_phy_sw);
	if (ret)
		return ret;

	apple_tunable_apply(s->apcie_phy, s->apcie_phy_tunable);
	writel(readl(ctrl) | APCIE_PHY_CTRL_CLK0REQ, ctrl);
	ret = readl_poll_timeout_atomic(ctrl, val, val & APCIE_PHY_CTRL_CLK0ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(ctrl) | APCIE_PHY_CTRL_CLK1REQ, ctrl);
	ret = readl_poll_timeout_atomic(ctrl, val, val & APCIE_PHY_CTRL_CLK1ACK, 1, APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(ctrl) & ~APCIE_PHY_CTRL_RESET, ctrl);

	writel(readl(run) | APCIE_PHY_RUN_EN, run);
	sleep_apcie_phy_ip_apply(s);
	writel(readl(run) | APCIE_PHY_RUN_EN2, run);
	ret = readl_poll_timeout_atomic(s->apcie_phy + APCIE_PHY_READY, val, val & BIT(0), 1,
					APCIE_POLL_US);
	if (ret)
		return ret;
	writel(readl(s->apcie_phy + APCIE_PHY_COMMON) | BIT(0), s->apcie_phy + APCIE_PHY_COMMON);
	writel(readl(ctrl) | APCIE_PHY_CTRL_UP, ctrl);

	writel(0, s->apcie_rc + APCIE_RC_ZERO);
	writel(0x140, s->apcie_rc + APCIE_RC_READY_CFG);
	writel(1, s->apcie_rc + APCIE_RC_READY_START);

	return readl_poll_timeout_atomic(s->apcie_rc + APCIE_RC_READY_STAT, val, val & BIT(0), 1,
					 APCIE_POLL_US);
}

/* About 4 ms after the other cores stop on macOS: deepest first, ANS last. */
static void apple_sleep_lower_io(struct apple_sleep *s)
{
	const struct apple_sleep_domains *d = &s->soc->ps;

	/*
	 * USB's complex, deepest first. macOS's host lowers the controller's and the common
	 * domain in its own suspend, so here they're only lowered for a host that didn't. A host
	 * that sleeps in place keeps the always-on one, as macOS's does.
	 */
	if (FIELD_GET(PS_TARGET, readl(sleep_power_reg(s, d->usb))))
		sleep_ps_lower(s, d->usb);
	if (FIELD_GET(PS_TARGET, readl(sleep_power_reg(s, d->usb_common))))
		sleep_ps_lower(s, d->usb_common);
	if (!READ_ONCE(sleep_usb_asleep))
		sleep_ps_lower(s, d->usb_aon);
	if (s->pcie_ans_stopped) {
		apple_sleep_lower_apcie(s);
		s->ans_off_ret = sleep_ps_wait_off(s, d->ans);
	}
}

static void apple_sleep_set_wd2(struct apple_sleep *s, unsigned int secs)
{
	writel(0, s->wdt + WDT_WD2_CTL);
	writel(0, s->wdt + WDT_WD2_COUNT);
	writel(secs * WDT_TICKS_PER_S, s->wdt + WDT_WD2_ALARM);
	writel(1, s->wdt + WDT_WD2_CTL);
}

#define PS_PARENT_MISSING	BIT(11)
#define PS_AUTO_STATE		GENMASK(27, 24)
#define PS_REWAKE_PASSES	2

static void sleep_ps_save(struct apple_sleep *s)
{
	const struct apple_sleep_words *all = &s->soc->ps_all, *aon = &s->soc->ps_aon_all;
	int i;

	for (i = 0; i < all->count; i++)
		s->ps_commit[i] = readl(s->ps + all->offset[i]);
	for (i = 0; i < aon->count; i++)
		s->ps_aon_commit[i] = readl(s->ps_aon + aon->offset[i]);
}

static int sleep_ps_find(const u32 *all, int n, u32 offset)
{
	int i;

	for (i = 0; i < n; i++)
		if (all[i] == offset)
			return i;
	return -1;
}

/*
 * As macOS raises a domain: the target, then AUTO_ENABLE once it reads on. Never
 * DEV_DISABLE, which keeps it off: the SEP's is set at the commit, and macOS's wake writes
 * that word without it. The hardware owns the auto state field, so it stays as read.
 */
static int sleep_ps_restore(void __iomem *reg, u32 saved)
{
	u32 keep = (saved & ~(PS_AUTO_ENABLE | PS_AUTO_STATE | PS_PARENT_MISSING | PS_DEV_DISABLE |
			      PS_FLAGS | PS_ACTUAL)) | (readl(reg) & PS_AUTO_STATE);
	u32 now;
	int ret;

	/*
	 * Off under AUTO_ENABLE at the commit, as the SEP's is: the hardware powers it when
	 * it's wanted, and forcing it on doesn't. macOS writes that word once, AUTO set.
	 */
	if ((saved & PS_AUTO_ENABLE) && !FIELD_GET(PS_ACTUAL, saved)) {
		writel(keep | PS_AUTO_ENABLE, reg);
		return 0;
	}

	writel(keep, reg);
	ret = readl_poll_timeout_atomic(reg, now,
					FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, saved), 1,
					PS_OFF_TIMEOUT_US);
	if (saved & PS_AUTO_ENABLE)
		writel(keep | PS_AUTO_ENABLE, reg);

	return ret;
}

/* Whether the domain at @offset was on as the wake found it. */
static bool sleep_ps_woke_on(struct apple_sleep *s, u32 offset)
{
	const struct apple_sleep_words *all = &s->soc->ps_all;
	int i = sleep_ps_find(all->offset, all->count, offset);

	return i >= 0 && FIELD_GET(PS_ACTUAL, s->ps_woke[i]);
}

/* Linux had it on and the wake left it otherwise. With AUTO_ENABLE, off is allowed. */
static bool sleep_ps_dropped(u32 saved, u32 woke)
{
	u32 control = PS_AUTO_ENABLE | PS_DEV_DISABLE | PS_TARGET;

	if (!FIELD_GET(PS_TARGET, saved))
		return false;
	if ((saved ^ woke) & control)
		return true;
	return !(saved & PS_AUTO_ENABLE) &&
	       FIELD_GET(PS_ACTUAL, woke) != FIELD_GET(PS_TARGET, saved);
}

/*
 * Every domain back as it was at the commit, before any driver's resume reads its block:
 * macOS's wake words first, in its order, whatever they read, if Linux had them on. Then
 * any other domain the wake left otherwise, a second time for one whose parent came later.
 */
static void sleep_ps_rewake(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	const u32 *all = soc->ps_all.offset, *aon = soc->ps_aon_all.offset;
	unsigned int n = soc->ps_all.count, naon = soc->ps_aon_all.count;
	bool *redo = s->ps_late, *redo_aon = s->ps_aon_late;
	bool *done = s->ps_done, *done_aon = s->ps_aon_done;
	int i, j, pass;

	memset(done, 0, n * sizeof(*done));
	memset(done_aon, 0, naon * sizeof(*done_aon));
	memset(s->ps_late, 0, n * sizeof(*s->ps_late));
	memset(s->ps_aon_late, 0, naon * sizeof(*s->ps_aon_late));
	s->ps_rewoken = 0;
	s->ps_rewake_timeouts = 0;
	for (i = 0; i < n; i++)
		s->ps_woke[i] = readl(sleep_power_reg(s, all[i]));
	for (i = 0; i < naon; i++)
		s->ps_aon_woke[i] = readl(s->ps_aon + aon[i]);

	for (j = 0; j < soc->ps_wake_order.count; j++) {
		i = sleep_ps_find(all, n, soc->ps_wake_order.offset[j]);
		if (i < 0 || !FIELD_GET(PS_TARGET, s->ps_commit[i]))
			continue;
		done[i] = true;
		s->ps_rewoken++;
		if (sleep_ps_restore(sleep_power_reg(s, all[i]), s->ps_commit[i]))
			redo[i] = true;
	}
	for (j = 0; j < soc->ps_aon_wake_order.count; j++) {
		i = sleep_ps_find(aon, naon, soc->ps_aon_wake_order.offset[j]);
		if (i < 0 || !FIELD_GET(PS_TARGET, s->ps_aon_commit[i]))
			continue;
		done_aon[i] = true;
		s->ps_rewoken++;
		if (sleep_ps_restore(s->ps_aon + aon[i], s->ps_aon_commit[i]))
			redo_aon[i] = true;
	}

	for (pass = 0; pass < PS_REWAKE_PASSES; pass++) {
		for (i = 0; i < n; i++) {
			if (!redo[i] &&
			    (done[i] || !sleep_ps_dropped(s->ps_commit[i], s->ps_woke[i])))
				continue;
			if (!done[i]) {
				done[i] = true;
				s->ps_rewoken++;
			}
			redo[i] = sleep_ps_restore(sleep_power_reg(s, all[i]), s->ps_commit[i]);
		}
		for (i = 0; i < naon; i++) {
			if (!redo_aon[i] &&
			    (done_aon[i] ||
			     !sleep_ps_dropped(s->ps_aon_commit[i], s->ps_aon_woke[i])))
				continue;
			if (!done_aon[i]) {
				done_aon[i] = true;
				s->ps_rewoken++;
			}
			redo_aon[i] = sleep_ps_restore(s->ps_aon + aon[i], s->ps_aon_commit[i]);
		}
	}

	for (i = 0; i < n; i++)
		s->ps_rewake_timeouts += redo[i];
	for (i = 0; i < naon; i++)
		s->ps_rewake_timeouts += redo_aon[i];
}

/*
 * Which blocks lost their power across this sleep: a coprocessor whose word is listed lost its
 * firmware as well, whatever the wake wrote back, and a driver that talks to it after the wake
 * talks to a reset block. A line at info for the PMGR or the always-on PMGR only when one of
 * its words was lost; the usual case, every word back on, is at debug.
 */

static void sleep_ps_report_off(struct apple_sleep *s, const char *name,
				const struct apple_sleep_words *w, const u32 *commit,
				const u32 *woke)
{
	char buf[384];
	int i, len = 0, off = 0;

	for (i = 0; i < w->count; i++) {
		if (!FIELD_GET(PS_TARGET, commit[i]) || FIELD_GET(PS_ACTUAL, woke[i]))
			continue;
		off++;
		if (len < sizeof(buf) - 8)
			len += scnprintf(buf + len, sizeof(buf) - len, " %#x", w->offset[i]);
	}
	if (off)
		dev_info(s->dev, "%s: %d words on at the commit were off at the wake:%s\n", name,
			 off, buf);
	else
		dev_dbg(s->dev, "%s: all words on at the commit were on at the wake\n", name);
}

static void sleep_ps_rewake_report(struct apple_sleep *s)
{
	const struct apple_sleep_words *all = &s->soc->ps_all, *aon = &s->soc->ps_aon_all;
	int i;

	sleep_ps_report_off(s, "PMGR", all, s->ps_commit, s->ps_woke);
	sleep_ps_report_off(s, "AON PMGR", aon, s->ps_aon_commit, s->ps_aon_woke);

	dev_dbg(s->dev, "power words: %u written back, %u not on in time\n", s->ps_rewoken,
		s->ps_rewake_timeouts);
	for (i = 0; i < all->count; i++)
		if (s->ps_late[i])
			dev_err(s->dev, "power word %#x not on in time: %#x at the commit, %#x now\n",
				all->offset[i], s->ps_commit[i],
				readl(s->ps + all->offset[i]));
	for (i = 0; i < aon->count; i++)
		if (s->ps_aon_late[i])
			dev_err(s->dev, "power word aon %#x not on in time: %#x at the commit, %#x now\n",
				aon->offset[i], s->ps_aon_commit[i],
				readl(s->ps_aon + aon->offset[i]));
}

#define T8132_PS_AON_ON_STATE	4

/*
 * Gen 4: the SPMI driver's exchange lock, tried. A debug note gives way to an exchange under
 * way; a write the sleep needs (@must) goes ahead as before and is counted. Returns whether
 * the lock is held (and must be released).
 */
static bool sleep_pmu_lock(struct apple_sleep *s, bool must, bool *skip)
{
	int ret;

	*skip = false;
	if (!s->pmu_bus)
		return false;
	ret = apple_spmi_fifo_trylock(s->pmu_bus);
	if (!ret)
		return true;
	/* -ENODEV: its driver is not bound, and nothing else is on the bus. */
	if (ret == -EBUSY) {
		if (must)
			s->pmu_busy_forced++;
		else
			*skip = true;
		s->pmu_busy++;
	}
	return false;
}

static void sleep_pmu_write_one(struct apple_sleep *s, u16 reg, u8 val, bool must)
{
	u32 status, enable;
	bool locked, skip;
	int i;

	if (!s->soc->spmi_gen4) {
		apple_sleep_pmu_write(s->spmi, s->soc->pmu_usid, reg, val);
		return;
	}
	locked = sleep_pmu_lock(s, must, &skip);
	if (skip)
		return;

	enable = readl(s->spmi + s->spmi_notify_enable);
	writel(enable & ~SPMI_NOTIFY_REPLY, s->spmi + s->spmi_notify_enable);
	/*
	 * Gen 4 (T8142): a reply someone else left (aura writes the PMU on this queue at a wake)
	 * would be taken for ours, and ours left for the SPMI driver's next command. Drain before
	 * and after, each bounded, as aurora-silicon/linux#1664 does on T6041.
	 */
	for (i = 0; s->soc->spmi_gen4 && i < SPMI_DRAIN_MAX &&
	     !(readl(s->spmi + s->spmi_status) & s->spmi_rx_empty); i++)
		readl(s->spmi + s->spmi_reply);
	writel(((u32)reg << 16) | SPMI_CMD_WRITE |
	       FIELD_PREP(SPMI_CMD_USID, s->soc->pmu_usid), s->spmi + s->spmi_cmd);
	writel(val, s->spmi + s->spmi_cmd);
	if (!readl_poll_timeout_atomic(s->spmi + s->spmi_status, status,
				       !(status & s->spmi_rx_empty), 1, 1000))
		readl(s->spmi + s->spmi_reply);
	for (i = 0; s->soc->spmi_gen4 && i < SPMI_DRAIN_MAX &&
	     !(readl(s->spmi + s->spmi_status) & s->spmi_rx_empty); i++)
		readl(s->spmi + s->spmi_reply);
	writel(1, s->spmi + s->spmi_notify_ack);
	writel(enable, s->spmi + s->spmi_notify_enable);
	if (locked)
		apple_spmi_fifo_unlock(s->pmu_bus);
}

static void sleep_pmu_write(struct apple_sleep *s, u16 reg, u8 val)
{
	sleep_pmu_write_one(s, reg, val, false);
}

/*
 * The PMU alarm also needs its peripheral interrupt routed by nub-spmi0.
 * Arm only the DT-owned sources, after the old RTC event has been cleared and
 * the next compare programmed. A latched event from the previous alarm must
 * not turn this sleep into an immediate wake.
 */
static void sleep_spmi_alarm_wake_arm(struct apple_sleep *s)
{
	void __iomem *enable;
	u32 latched, val;

	if (!s->spmi_pmu_bits)
		return;
	enable = s->spmi + SPMI4_SRC_ENABLE + s->spmi_pmu_status - SPMI4_SRC_STATUS;
	latched = readl(s->spmi + s->spmi_pmu_status) & s->spmi_pmu_bits;
	if (latched)
		writel(latched, s->spmi + s->spmi_pmu_status);
	val = readl(enable);
	s->spmi_pmu_enable_saved = val & s->spmi_pmu_bits;
	s->spmi_pmu_wake_armed = true;
	writel(val | s->spmi_pmu_bits, enable);
}

/*
 * Gen 4: nub-spmi0's peripheral source words, read at the entry (@when 0) and at the wake
 * (1), and the PMU's latched sources among them acknowledged where the DT names them. The
 * T6050 sleep found that after the first sleep's alarm every later sleep was back within a
 * second, the latched alarm holding the controller's output (an AP wake source) up; macOS
 * acknowledges it at each wake. T8142's nub-spmi0 is the same generation and its output is
 * the same wake source (NUB.SPMI0.SW3). Without the property the words are only read, so
 * that a sleep's report shows which bits to name.
 */
static void sleep_spmi_sources(struct apple_sleep *s, unsigned int when)
{
	u32 latched;
	int i;

	if (!s->soc->spmi_gen4 || s->soc->regions)
		return;
	for (i = 0; i < SPMI4_SRC_WORDS; i++)
		s->spmi_src[when][i] = readl(s->spmi + SPMI4_SRC_STATUS + 4 * i);
	s->spmi_src_read[when] = true;
	if (!s->spmi_pmu_bits)
		return;
	latched = s->spmi_src[when][(s->spmi_pmu_status - SPMI4_SRC_STATUS) / 4] &
		  s->spmi_pmu_bits;
	if (latched)
		writel(latched, s->spmi + s->spmi_pmu_status);
	if (when && s->spmi_pmu_wake_armed) {
		void __iomem *enable = s->spmi + SPMI4_SRC_ENABLE +
				      s->spmi_pmu_status - SPMI4_SRC_STATUS;

		writel((readl(enable) & ~s->spmi_pmu_bits) | s->spmi_pmu_enable_saved,
		       enable);
		s->spmi_pmu_wake_armed = false;
	}
}

static void sleep_spmi_sources_report(struct apple_sleep *s)
{
	static const char *const when[] = { "entry", "wake" };
	u32 *w;
	int i;

	for (i = 0; i < 2; i++) {
		if (!s->spmi_src_read[i])
			continue;
		s->spmi_src_read[i] = false;
		w = s->spmi_src[i];
		dev_info(s->dev, "nub-spmi0 sources at the %s: %08x %08x %08x %08x %08x %08x %08x %08x%s\n",
			 when[i], w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
			 s->spmi_pmu_bits ? ", PMU's acknowledged" : "");
	}
	if (s->pmu_busy)
		dev_info(s->dev, "PMU writes found the SPMI exchange lock taken %u times (%u went ahead)\n",
			 s->pmu_busy, s->pmu_busy_forced);
	s->pmu_busy = s->pmu_busy_forced = 0;
}

static bool sleep_notes_on(struct apple_sleep *s)
{
	return s && s->spmi && s->soc->untested && READ_ONCE(sleep_pmu_notes);
}

/*
 * Only where nothing else talks SPMI: from .prepare (every device suspended) to .wake (none
 * resumed yet), on the one CPU left or before the others go.
 */
static void sleep_note_at(struct apple_sleep *s, u8 off, u8 val)
{
	if (sleep_notes_on(s))
		sleep_pmu_write(s, PMU_NOTE_BASE + off, val);
}

static void sleep_note(struct apple_sleep *s, u8 step)
{
	sleep_note_at(s, 1, step);
}

void apple_sleep_note_sub(u8 step)
{
	sleep_note_at(apple_sleep, 4, step);
}
EXPORT_SYMBOL_GPL(apple_sleep_note_sub);

/*
 * The DRAM-kept wake writes each of these words and gives its switch 171 us to report closed.
 * A domain that went off with BUSY latched holds that poll until WD2 resets the SoC. The PMGR
 * driver doesn't clear the latch in a system sleep, so do it here. Returns whether one stayed
 * latched; the deep path then cuts their power instead.
 */
static bool apple_sleep_ps_unlatch(struct apple_sleep *s)
{
	const struct apple_sleep_words *words = &s->soc->ps_engine_polled;
	bool stuck = false;
	unsigned int i;

	s->ps_unlatched_count = 0;
	for (i = 0; i < words->count && i < ARRAY_SIZE(s->ps_unlatched); i++) {
		void __iomem *reg = s->ps + words->offset[i];
		u32 val = readl(reg), off;

		if (!(val & PS_BUSY) || (val & (PS_ACTUAL | PS_TARGET)))
			continue;
		/* A switch still finishing reads BUSY for a moment: only one that stays is a latch. */
		if (!readl_poll_timeout_atomic(reg, val, !(val & PS_BUSY), 1, PS_BUSY_SETTLE_US))
			continue;
		s->ps_unlatched[s->ps_unlatched_count].offset = words->offset[i];
		s->ps_unlatched[s->ps_unlatched_count++].val = val;
		off = val & ~(PS_AUTO_ENABLE | PS_FLAGS | PS_BUSY | PS_UNLATCH | PS_DEV_DISABLE);
		writel(off | PS_UNLATCH, reg);
		readl_poll_timeout_atomic(reg, val, !(val & PS_BUSY), 1, PS_OFF_TIMEOUT_US);
		writel(off, reg);
		if (readl(reg) & PS_BUSY)
			stuck = true;
	}

	return stuck;
}

struct t8142_mem_op {
	u64 pa;
	u64 val;
	u8 op;
};
#define T8142_MEM_WRITE	BIT(0)
#define T8142_MEM_64	BIT(1)
#define T8142_MEM_FP	BIT(4)
#include "sleep-t8142-mem.h"

static void t8142_mem_map(struct apple_sleep *s)
{
	unsigned int i;

	BUILD_BUG_ON(ARRAY_SIZE(t8142_mem_pages) > ARRAY_SIZE(s->mem_boot_page));
	for (i = 0; i < ARRAY_SIZE(t8142_mem_pages); i++)
		s->mem_boot_page[i] = devm_ioremap(s->dev, t8142_mem_pages[i], SZ_16K);
}

static void __iomem *t8142_mem_va(struct apple_sleep *s, u64 pa)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(t8142_mem_pages); i++)
		if ((pa & ~(u64)(SZ_16K - 1)) == t8142_mem_pages[i])
			return s->mem_boot_page[i] ? s->mem_boot_page[i] + (pa & (SZ_16K - 1)) : NULL;
	return NULL;
}

/* The words XNU reads before its first write, against J813's (iBoot's) values: mismatches. */
static unsigned int t8142_mem_fingerprint(struct apple_sleep *s, u64 *pa, u64 *got, u64 *want)
{
	unsigned int i, bad = 0;

	*pa = 0;
	for (i = 0; i < ARRAY_SIZE(t8142_mem_boot); i++) {
		const struct t8142_mem_op *o = &t8142_mem_boot[i];
		void __iomem *va;
		u64 v;

		if (!(o->op & T8142_MEM_FP))
			continue;
		va = t8142_mem_va(s, o->pa);
		if (!va) {
			bad++;
			continue;
		}
		v = (o->op & T8142_MEM_64) ? readq(va) : readl(va);
		if (v == o->val)
			continue;
		if (!bad++) {
			*pa = o->pa;
			*got = v;
			*want = o->val;
		}
	}
	return bad;
}

/* XNU's accesses in its order: the writes as it wrote them, the reads done and dropped. */
static void t8142_mem_replay(struct apple_sleep *s)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(t8142_mem_boot); i++) {
		const struct t8142_mem_op *o = &t8142_mem_boot[i];
		void __iomem *va = t8142_mem_va(s, o->pa);

		if (!va)
			continue;
		if (o->op & T8142_MEM_WRITE) {
			if (o->op & T8142_MEM_64)
				writeq(o->val, va);
			else
				writel(o->val, va);
		} else if (o->op & T8142_MEM_64) {
			readq(va);
		} else {
			readl(va);
		}
	}
}

static void t8142_mem_boot_entry(struct apple_sleep *s)
{
	u64 pa, got, want;

	s->mem_boot_done = false;
	s->mem_boot_mismatch = 0;
	if (!s->soc->mem_boot || !READ_ONCE(sleep_mem_boot))
		return;
	s->mem_boot_mismatch = t8142_mem_fingerprint(s, &pa, &got, &want);
	if (s->mem_boot_mismatch)
		return;
	t8142_mem_replay(s);
	s->mem_boot_done = true;
}

static void __iomem *sleep_boot_auto_reg(struct apple_sleep *s, unsigned int i)
{
	unsigned int n = s->soc->ps_boot_auto.count;

	return i < n ? s->ps + s->soc->ps_boot_auto.offset[i] :
		       s->ps_aon + s->soc->ps_aon_boot_auto.offset[i - n];
}

static unsigned int sleep_boot_auto_count(struct apple_sleep *s)
{
	return min_t(unsigned int, s->soc->ps_boot_auto.count + s->soc->ps_aon_boot_auto.count,
		     ARRAY_SIZE(s->boot_auto_saved));
}

/* As XNU's PMGR start: the value with its flags cleared, then AUTO_ENABLE. On words that are on. */
static int sleep_boot_auto(struct apple_sleep *s)
{
	unsigned int i, n = sleep_boot_auto_count(s);
	int set = 0;

	s->boot_auto_set = 0;
	s->boot_auto_nset = s->boot_auto_stuck = 0;
	if (!READ_ONCE(sleep_boot_auto_on))
		return 0;
	for (i = 0; i < n; i++) {
		void __iomem *reg = sleep_boot_auto_reg(s, i);
		u32 v = readl(reg);

		if ((v & PS_AUTO_ENABLE) || !FIELD_GET(PS_TARGET, v) || !FIELD_GET(PS_ACTUAL, v))
			continue;
		s->boot_auto_saved[i] = v;
		s->boot_auto_set |= BIT_ULL(i);
		writel(v & ~PS_FLAGS, reg);
		readl(reg);
		writel((v & ~PS_FLAGS) | PS_AUTO_ENABLE, reg);
		set++;
		if (readl(reg) & PS_AUTO_ENABLE)
			s->boot_auto_stuck++;
	}
	s->boot_auto_nset = set;
	return set;
}

/* A refusal before the commit: AUTO_ENABLE off again where this set it, the target as it was. */
static void sleep_boot_auto_undo(struct apple_sleep *s)
{
	unsigned int i, n = sleep_boot_auto_count(s);

	for (i = 0; i < n; i++)
		if (s->boot_auto_set & BIT_ULL(i))
			writel(s->boot_auto_saved[i] & ~PS_FLAGS, sleep_boot_auto_reg(s, i));
	s->boot_auto_set = 0;
}

/* Whether the match leaves this word as it is (apple_sleep.ps_keep, as the match took it) */
static bool sleep_ps_kept(struct apple_sleep *s, u32 offset, bool aon)
{
	unsigned int i;

	for (i = 0; i < s->soc->nps_keep; i++) {
		const struct apple_sleep_ps_keep *k = &s->soc->ps_keep[i];

		if (k->offset == offset && k->aon == aon)
			return s->ps_keep_mask & BIT(k->group);
	}

	return false;
}

static void apple_sleep_match(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i, raised = 0, lowered = 0, failed = 0, kept = 0;

	s->ps_keep_mask = READ_ONCE(sleep_ps_keep);

	writel(0, s->wdt + WDT_WD0_COUNT);
	writel(0, s->wdt + WDT_WD1_COUNT);
	sleep_pmu_write_one(s, soc->pmu_sleep_reg, soc->pmu_sleep_val, true);

	sleep_spmi_sources(s, 0);
	s->spmi_saved[0] = readl(s->spmi + s->spmi_notify_enable);
	s->spmi_saved[1] = readl(s->spmi_a0);
	writel(soc->spmi_notify_sleep, s->spmi + s->spmi_notify_enable);
	writel(soc->spmi_notify_sleep, s->spmi_a0);

	for (i = 0; i < soc->ps_macos_on.count; i++) {
		u32 offset = soc->ps_macos_on.offset[i];

		s->ps_on_saved[i] = readl(sleep_power_reg(s, offset));
		/* State 4 reads as on, and macOS has these at f. */
		if (FIELD_GET(PS_ACTUAL, s->ps_on_saved[i]) == 0xf)
			continue;
		/* Measured at the commit with AUTO_ENABLE clear, unlike after a wake. */
		if (sleep_ps_raise_start(s, offset))
			failed++;
		raised++;
	}
	for (i = 0; i < soc->ps_macos_off.count; i++) {
		u32 offset = soc->ps_macos_off.offset[i];

		s->ps_off_saved[i] = readl(sleep_power_reg(s, offset));
		if (!sleep_ps_on(s, offset))
			continue;
		if (sleep_ps_kept(s, offset, false)) {
			kept++;
			continue;
		}
		sleep_ps_lower(s, offset);
		lowered++;
	}
	for (i = 0; i < soc->ps_aon_macos_off.count; i++) {
		void __iomem *reg = s->ps_aon + soc->ps_aon_macos_off.offset[i];
		u32 val = readl(reg);

		s->ps_aon_saved[i] = val;
		if (!FIELD_GET(PS_ACTUAL, val))
			continue;
		if (sleep_ps_kept(s, soc->ps_aon_macos_off.offset[i], true)) {
			kept++;
			continue;
		}
		if (val & PS_AUTO_ENABLE) {
			val &= ~(PS_AUTO_ENABLE | PS_FLAGS);
			writel(val, reg);
		}
		writel((val & ~PS_TARGET) | PS_FLAGS, reg);
		lowered++;
	}
	for (i = 0; i < soc->ps_aon_macos_no_auto.count; i++) {
		void __iomem *reg = s->ps_aon + soc->ps_aon_macos_no_auto.offset[i];
		u32 val = readl(reg);

		s->ps_aon_auto_saved[i] = val;
		writel(val & ~(PS_AUTO_ENABLE | PS_FLAGS), reg);
	}
	/* T8132's audio words stand at state 4 under macOS, not at f. */
	for (i = 0; i < soc->ps_aon_macos_on.count; i++) {
		void __iomem *reg = s->ps_aon + soc->ps_aon_macos_on.offset[i];
		u32 val = readl(reg), now;

		s->ps_aon_on_saved[i] = val;
		if (FIELD_GET(PS_ACTUAL, val) == T8132_PS_AON_ON_STATE)
			continue;
		writel((val & ~(PS_AUTO_ENABLE | GENMASK(11, 0))) | T8132_PS_AON_ON_STATE, reg);
		if (readl_poll_timeout_atomic(reg, now,
					      FIELD_GET(PS_ACTUAL, now) == T8132_PS_AON_ON_STATE, 1,
					      PS_OFF_TIMEOUT_US))
			failed++;
		raised++;
	}
	s->matched = true;

	for (i = 0; i < soc->ps_macos_off.count; i++) {
		u32 offset = soc->ps_macos_off.offset[i];

		if (sleep_ps_kept(s, offset, false))
			continue;
		if (sleep_ps_wait_off(s, offset))
			dev_dbg(s->dev, "still on, macOS has it off: %x = %x\n", offset,
				readl(sleep_power_reg(s, offset)));
	}
	dev_dbg(s->dev, "macOS's power words: raised %d, %d of them failed, lowered %d, kept on %d\n",
		raised, failed, lowered, kept);
}

/* Back as they were, after a refusal past the match or a real wake. */
static void apple_sleep_match_undo(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i;

	if (!s->matched)
		return;
	s->matched = false;

	for (i = soc->ps_aon_macos_on.count - 1; i >= 0; i--)
		writel(s->ps_aon_on_saved[i] & ~PS_FLAGS,
		       s->ps_aon + soc->ps_aon_macos_on.offset[i]);
	for (i = soc->ps_aon_macos_no_auto.count - 1; i >= 0; i--)
		writel(s->ps_aon_auto_saved[i] & ~PS_FLAGS,
		       s->ps_aon + soc->ps_aon_macos_no_auto.offset[i]);
	for (i = soc->ps_aon_macos_off.count - 1; i >= 0; i--)
		if (FIELD_GET(PS_ACTUAL, s->ps_aon_saved[i]) &&
		    !sleep_ps_kept(s, soc->ps_aon_macos_off.offset[i], true))
			writel(s->ps_aon_saved[i] & ~PS_FLAGS,
			       s->ps_aon + soc->ps_aon_macos_off.offset[i]);
	for (i = soc->ps_macos_off.count - 1; i >= 0; i--)
		if (FIELD_GET(PS_ACTUAL, s->ps_off_saved[i]) &&
		    !sleep_ps_kept(s, soc->ps_macos_off.offset[i], false))
			sleep_ps_raise(s, soc->ps_macos_off.offset[i]);
	for (i = soc->ps_macos_on.count - 1; i >= 0; i--) {
		u32 val = s->ps_on_saved[i];

		if (!FIELD_GET(PS_ACTUAL, val))
			sleep_ps_lower(s, soc->ps_macos_on.offset[i]);
		else if (FIELD_GET(PS_ACTUAL, val) != 0xf)
			writel(val & ~PS_FLAGS, s->ps + soc->ps_macos_on.offset[i]);
	}
	/* What the PMU sent while the SoC slept (its wake), before the run-time mask */
	sleep_spmi_sources(s, 1);
	writel(s->spmi_saved[0], s->spmi + s->spmi_notify_enable);
	writel(s->spmi_saved[1], s->spmi_a0);
}

/*
 * After the enclave's stop, with nothing left that refuses: each PS write-disable bit the SoC
 * names, cleared where set, so that the coprocessor stops and the lowering below reach their
 * words. They are not set again after the wake. Linux never needs them, the drivers' power-ups
 * would be dropped if they were, and a deep sleep loses the PMGR's state anyway; the words
 * are read again after the wake so the report shows what the PMGR came back with.
 */
static void sleep_ps_wr_dis_clear(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i;

	s->ps_wr_dis_cleared = 0;
	for (i = 0; i < soc->nps_wr_dis; i++) {
		void __iomem *reg = s->ps + soc->ps_wr_dis[i].offset;
		u32 val = readl(reg);

		s->ps_wr_dis_entry[i] = val;
		if (!(val & BIT(soc->ps_wr_dis[i].bit)))
			continue;
		writel(val & ~BIT(soc->ps_wr_dis[i].bit), reg);
		s->ps_wr_dis_cleared |= BIT(i);
	}
}

static void sleep_ps_wr_dis_woke(struct apple_sleep *s)
{
	int i;

	for (i = 0; i < s->soc->nps_wr_dis; i++)
		s->ps_wr_dis_wake[i] = readl(s->ps + s->soc->ps_wr_dis[i].offset);
	s->ps_wr_dis_read = s->soc->nps_wr_dis > 0;
}

static void sleep_ps_wr_dis_report(struct apple_sleep *s)
{
	const struct apple_sleep_ps_wr_dis *e = s->soc->ps_wr_dis;
	int i;

	if (!s->ps_wr_dis_read)
		return;
	s->ps_wr_dis_read = false;
	for (i = 0; i < s->soc->nps_wr_dis; i++)
		dev_info(s->dev, "PS write-disable %s (ps+%#x bit %u): %#x at the entry%s, %#x after the wake\n",
			 e[i].name, e[i].offset, e[i].bit, s->ps_wr_dis_entry[i],
			 s->ps_wr_dis_cleared & BIT(i) ? " (cleared)" : "",
			 s->ps_wr_dis_wake[i]);
}

/* The safety net, as macOS arms it. */
static void apple_sleep_arm_wd2(struct apple_sleep *s)
{
	if (s->soc->aon_wdt_sleep) {
		s->aon_wdt_saved = readl(s->aon_misc + AON_WDT_SLEEP);
		writel(s->aon_wdt_saved | s->soc->aon_wdt_sleep, s->aon_misc + AON_WDT_SLEEP);
		s->aon_wdt_marked = true;
	}
	writel(0, s->wdt + WDT_WD0_CTL);
	apple_sleep_set_wd2(s, WDT_SLEEP_SECS);
	writel(0, s->aon_misc + AON_AP_DEADLINE);
}

static int sleep_perf_wait(void __iomem *reg, unsigned int state)
{
	u32 val;

	return readl_poll_timeout_atomic(reg, val,
					 (val & (DCS_BUSY | DCS_STATE)) ==
					 FIELD_PREP(DCS_STATE, state),
					 1, DCS_TIMEOUT_US);
}

static int sleep_dcs_wait(struct apple_sleep *s, unsigned int state)
{
	return sleep_perf_wait(s->dcs, state);
}

/*
 * As macOS does at about -18 ms, on each channel in turn: the enables down a bit at a time,
 * then each of four lanes strobed and cleared, two lanes on every channel before the
 * next two. Each write is built from a fresh read, as the trace has it.
 */
static void apple_sleep_mcc_stop(struct apple_sleep *s)
{
	int ch, pass, n;
	u32 val;

	for (ch = 0; ch < s->soc->nmcc; ch++) {
		val = readl(s->mcc[ch] + MCC_ENABLES);
		for (n = 0; val && n < 32; n++) {
			val &= val - 1;
			writel(val, s->mcc[ch] + MCC_ENABLES);
			val = readl(s->mcc[ch] + MCC_ENABLES);
		}
	}

	for (pass = 0; pass < 2; pass++) {
		for (ch = 0; ch < s->soc->nmcc; ch++) {
			void __iomem *mcc = s->mcc[ch];

			for (n = 2 * pass; n < 2 * pass + 2; n++) {
				writel(readl(mcc + MCC_LANES_MASK), mcc + MCC_LANES_MASK);
				writel(readl(mcc + MCC_LANES) | MCC_LANE_STROBE(n),
				       mcc + MCC_LANES);
				writel(readl(mcc + MCC_LANES) & ~MCC_LANE(n), mcc + MCC_LANES);
			}
		}
	}
}

/*
 * The engine's first step polls for DCS at this state and has no way to get it there.
 * macOS has every voter at 0 by now, which is how it lands there, and steps it down itself
 * on the path without the PMC. Both, here, in that order. Nothing can fail by now.
 */
static void apple_sleep_dcs_down(struct apple_sleep *s)
{
	unsigned int state;
	int agent;

	for (agent = 0; agent < PMC_VOTER_AGENTS; agent++)
		if (readl(s->pmc_voters + agent * PMC_VOTER_STRIDE))
			writel(0, s->pmc_voters + agent * PMC_VOTER_STRIDE);
	if (!sleep_dcs_wait(s, DCS_SLEEP_STATE))
		return;

	state = FIELD_GET(DCS_STATE, readl(s->dcs));
	if (state > DCS_STEP_STATE) {
		writel(DCS_STEP_STATE, s->dcs);
		sleep_dcs_wait(s, DCS_STEP_STATE);
	}
	if (state > DCS_SLEEP_STATE) {
		writel(DCS_SLEEP_STATE, s->dcs);
		sleep_dcs_wait(s, DCS_SLEEP_STATE);
	}
}

/* The boot core's last WFI. An event that ends it is taken, and the WFI taken again. */
static void __noreturn sleep_boot_wfi(struct apple_sleep *s)
{
	bool t8142 = s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm, noted = false;
	bool observed = t8142 && READ_ONCE(sleep_observer_ms);
	u64 t0;

	if (observed) {
		WRITE_ONCE(t8142_obs_boot_wfi, 1);
		dsb(sy);
	}
	sleep_deep_wfi_arm();
	t0 = read_sysreg(cntpct_el0);

	while (true) {
		dsb(sy);
		isb();
		wfi();
		if (observed)
			WRITE_ONCE(t8142_obs_wfi_back, READ_ONCE(t8142_obs_wfi_back) + 1);
		else if (t8142)
			t8142_wfi_returned(s, t0, &noted);
		s->soc->ops->aic_ack();
		write_sysreg_s(IPI_SR_PENDING, SYS_APL_IPI_SR_EL1);
	}
}

/* XNU polls these off just before its complex words. The sleep goes on whatever they read. */
static void apple_sleep_wait_still_on(struct apple_sleep *s)
{
	const struct apple_sleep_words *words = &s->soc->ps_still_on;
	int i;

	for (i = 0; i < words->count; i++)
		sleep_ps_wait_off(s, words->offset[i]);
}

/*
 * The last mile, in macOS's order. It starts after every refusal: this can't be
 * undone, and returning now would leave the clusters armed off. No printing, and
 * under 33 ms from the complex words to the WFI (the engine's bound for the
 * cores to go idle). A hang from here ends at the watchdog.
 *
 * What's left out on purpose:
 *  - The two writes of 0 to 0x302800000: reads 0, changes nothing.
 *  - The RTC alarm: macOS arms it after the commit. rtcwake has armed it long
 *    before this, with no SPMI traffic near the engine.
 */
static bool sleep_v27_selected_for(struct apple_sleep *s);
static void t8142_last_mile(struct apple_sleep *s);
static void t8142_post_commit_probe(struct apple_sleep *s, unsigned int ms);

static int apple_sleep_commit(unsigned long unused)
{
	struct apple_sleep *s = apple_sleep;
	bool early;

	if (s->soc->ops->commit_begin)
		s->soc->ops->commit_begin(s);

	/*
	 * The other cores stop first, as on macOS. Each answered the hold in .enter, and they
	 * stay coherent until the commit.
	 */
	s->siq_cfg = read_sysreg_s(SYS_APL_SIQ_CFG_EL1);
	WRITE_ONCE(t8142_boot_mpidr, read_cpuid_mpidr() & MPIDR_HWID_BITMASK);
	sleep_note(s, 0x60);
	WRITE_ONCE(t8142_obs_boot_wfi, 0);
	WRITE_ONCE(t8142_obs_wfi_back, 0);
	early = s->soc->early_cpu_wfi && READ_ONCE(sleep_early_wfi);
	if (early) {
		/* Released before the stop: each core goes from its stop bit to its deep WFI. */
		spin_table_pen_deep_el1(s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm &&
					is_kernel_in_hyp_mode() && READ_ONCE(sleep_el1_wfi) >= 2);
		spin_table_release_stopped();
	}
	s->cpus_stop_ret = spin_table_stop_parked();
	if (early)
		udelay(SLEEP_EARLY_WFI_SETTLE_US);

	if (s->soc->ops->commit_regs) {
		/* A SoC whose irreversible steps share nothing with the ones below. */
		s->soc->ops->commit_regs(s);
	} else {
		s->soc->ops->lower_io(s);
		apple_sleep_arm_wd2(s);
		apple_sleep_wait_still_on(s);

		/* Everything the wake polls is down by now, the words just lowered included. */
		if (apple_sleep_ps_unlatch(s)) {
			s->deep_once = true;
			sleep_slp_ddr_set_deep_once(true);
		}

		s->soc->ops->last_mile(s);
	}

	/* aura reads this with the MMU off. */
	s->resumes_before = READ_ONCE(s->mbox->resumes);
	WRITE_ONCE(s->mbox->resume_pa, __pa_symbol(s->soc->ops->resume));
	dcache_clean_poc((unsigned long)s->mbox, (unsigned long)(s->mbox + 1));
	if (s->soc->ops->pre_commit)
		s->soc->ops->pre_commit(s);
	dsb(sy);

	writel(1, s->commit);

	/* As on macOS: a raised line would end the last WFI, and the engine waits for it. */
	s->soc->ops->aic_mask_all();
	/* Optional: a SoC's own traffic after the timer, every line masked, finished before the release */
	if (s->soc->ops->post_timer)
		s->soc->ops->post_timer(s);

	/* Each core's deep WFI comes after the commit, this one last. No SPMI from here on. */
	if (!early) {
		spin_table_pen_deep_el1(s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm &&
					is_kernel_in_hyp_mode() && READ_ONCE(sleep_el1_wfi) >= 2);
		spin_table_release_stopped();
	}
	if (s->soc->boot_wfi_delay_ms)
		mdelay(s->soc->boot_wfi_delay_ms);
	if (s->soc->ops->last_mile == t8142_last_mile && READ_ONCE(sleep_post_commit_ms))
		t8142_post_commit_probe(s, READ_ONCE(sleep_post_commit_ms));
	sleep_boot_wfi(s);
}

/*
 * T8140, in macOS's order: SoC and DCS states down, MCC lanes stopped, the CPM gates, PTD
 * and voters, the clusters off, the clock muxes parked, CLLT and DVFM, AMCC errors masked.
 */
static void t8140_last_mile(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int plane, agent, i;
	u32 val;

	/* The SoC comes down before memory does, as on macOS. */
	if (sleep_perf_wait(s->clk_perf + CLK_PERF_SOC, SOC_SLEEP_STATE)) {
		writel(SOC_SLEEP_STATE, s->clk_perf + CLK_PERF_SOC);
		sleep_perf_wait(s->clk_perf + CLK_PERF_SOC, SOC_SLEEP_STATE);
	}
	apple_sleep_dcs_down(s);
	apple_sleep_mcc_stop(s);

	/* Gates, PTD, the first voter twice, gates: macOS's order, measured once. */
	for (agent = 0; agent < 2; agent++) {
		writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[0]);
		writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[1]);
	}
	writeq(0, s->ptd + PTD_A);
	writeq(0, s->ptd + PTD_B);
	for (agent = 0; agent < 2; agent++) {
		readl(s->pmc_voters);
		writel(0, s->pmc_voters);
		readl(s->pmc_voters + 4);
	}
	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[0]);
	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[1]);

	/* Already off since .prepare, or since genpd turned the encoders off, as on macOS. */
	writel(0, s->cluster + CLUSTER_AVE);
	s->cluster_ave_commit_ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, !val,
							      1, 5000);
	for (i = 0; i < soc->ncluster_cpu; i++)
		writel(0, s->cluster + soc->cluster_cpu[i]);

	writel(readl(s->ps + soc->ps.sio_cpu) & ~PS_AUTO_ENABLE, s->ps + soc->ps.sio_cpu);

	/* The SoC refuses the first, on macOS too. */
	for (i = 0; i < soc->nclk_mux_parked; i++)
		writel(soc->clk_mux_parked_val, s->clk_mux + soc->clk_mux_parked[i]);

	for (i = 0; i < soc->ncllt; i++)
		writel(readl(s->cllt + soc->cllt[i].offset) & ~soc->cllt[i].clear,
		       s->cllt + soc->cllt[i].offset);

	/*
	 * Two runs put these writes after the CLLT's: 5 to 6 ms before the commit
	 * in one, and the CLLT at 5.75 ms in the other. No run has hooked both.
	 */
	sleep_dvfm_park(s);

	sleep_ps_save(s);
	for (plane = 0; plane < soc->namcc; plane++) {
		void __iomem *amcc = s->amcc[plane];

		writel(readl(amcc + AMCC_WORD0_ENABLE) & ~AMCC_WORD0_DSID_AGE_OUT,
		       amcc + AMCC_WORD0_ENABLE);
		writel(readl(amcc + AMCC_WORD1_ENABLE) & ~AMCC_WORD1_UNEXP_RT_HIT,
		       amcc + AMCC_WORD1_ENABLE);
	}
}

#define T8132_AMCC_ENABLES	0x0
#define T8132_AMCC_STAGES	0x800
#define T8132_PMC_VOTER3	0xc000
#define T8132_PMC_VOTER3_B	0xc004

/*
 * What XNU reads just after its complex words on T8132, in its order, once each: the ATC
 * commons, the displays', ANS, AVD, VENC, GFX, JPG and APCIE's SYS_GP. It doesn't wait, and
 * neither does this, unlike T8140's polls (its order there is before the complex words).
 */
static const u32 t8132_quiesce_words[] = {
	T8132_PS_ATC0_COMMON, 0x428, 0x450, 0x480, 0x240, 0x1d8, 0x4e0, 0x538, 0x580, 0x308, 0x250,
	0x578, 0x4d8,
};

/*
 * The AON domains of ATC0, deepest first. Its order on the one complex macOS lowers around
 * sleep that's been traced, ATC1: USB, USB_AON, then the common word.
 */
static void t8132_aon_lower(struct apple_sleep *s, u32 offset)
{
	void __iomem *reg = s->ps_aon + offset;
	u32 val = readl(reg);

	if (val & PS_AUTO_ENABLE) {
		val &= ~(PS_AUTO_ENABLE | PS_FLAGS);
		writel(val, reg);
	}
	writel((val & ~PS_TARGET) | PS_FLAGS, reg);
}

/* The kernel's writable image, the per-CPU areas and where cpu_resume finds its context. */
static void t8132_clean_ram(void)
{
	unsigned long base = (unsigned long)sleep_save_stash;
	unsigned int cpu;

	dcache_clean_poc((unsigned long)_sdata, (unsigned long)_end);
	for_each_possible_cpu(cpu) {
		unsigned long area = (unsigned long)__per_cpu_start + per_cpu_offset(cpu);

		dcache_clean_poc(area, area + (__per_cpu_end - __per_cpu_start));
	}
	dcache_clean_poc(base, base + mpidr_hash_size() * sizeof(*sleep_save_stash));
	dsb(sy);
}

/*
 * What the wake reads cleaned to the point of persistence first, then ATC0's complex instead
 * of T8140's USB words, and ANS's PCIe as there.
 */
static void t8132_lower_io(struct apple_sleep *s)
{
	t8132_clean_ram();
	s->atc0_saved[0] = readl(s->ps_aon + T8132_PS_AON_ATC0_USB);
	s->atc0_saved[1] = readl(s->ps_aon + T8132_PS_AON_ATC0_USB_AON);
	s->atc0_saved[2] = readl(s->ps + T8132_PS_ATC0_COMMON);
	t8132_aon_lower(s, T8132_PS_AON_ATC0_USB);
	if (!READ_ONCE(sleep_usb_asleep))
		t8132_aon_lower(s, T8132_PS_AON_ATC0_USB_AON);
	sleep_ps_lower(s, T8132_PS_ATC0_COMMON);
	if (s->pcie_ans_stopped) {
		apple_sleep_lower_apcie(s);
		s->ans_off_ret = sleep_ps_wait_off(s, s->soc->ps.ans);
	}
}

/*
 * Each 64-bit word is cleared one bit at a time, from a fresh read each time, as the trace
 * has it. No read of the per-bit words at +0x1000 that macOS takes before and after each
 * write: their purpose is unknown. @bits in macOS's order.
 */
static void t8132_amcc_clear_bits(void __iomem *reg, const u8 *bits, unsigned int n)
{
	unsigned int i;
	u64 val;

	for (i = 0; i < n; i++) {
		val = readq(reg);
		if (val & BIT_ULL(bits[i]))
			writeq(val & ~BIT_ULL(bits[i]), reg);
	}
}

static void t8132_mcc_step_down(void __iomem *reg)
{
	u32 val = readl(reg);
	int n;

	for (n = 0; val && n < 32; n++) {
		writel(val & (val - 1), reg);
		val = readl(reg);
	}
}

/*
 * What macOS does on the memory controller before the gates, measured on T8132: each AMCC
 * plane's enables cleared in step, bit 8 first (bit 9 exists only on the second plane), then
 * two memory controllers' lane enables down to 0, the first controller's second word, and
 * each plane's stage word stepped down from 7.
 */
static void t8132_sleep_mcc_stop(struct apple_sleep *s)
{
	static const u8 order[] = { 8, 0, 1, 2, 3, 4, 9, 5, 6, 7 };
	static const u8 stages[] = { 0, 1, 2 };
	int i, plane, ch;

	for (i = 0; i < ARRAY_SIZE(order); i++)
		for (plane = 0; plane < s->soc->namcc; plane++)
			t8132_amcc_clear_bits(s->amcc[plane] + T8132_AMCC_ENABLES, &order[i], 1);

	for (ch = 0; ch < s->soc->nmcc; ch++)
		t8132_mcc_step_down(s->mcc[ch] + MCC_ENABLES);

	t8132_mcc_step_down(s->mcc[0] + MCC_LANES);

	for (plane = 0; plane < s->soc->namcc; plane++)
		t8132_amcc_clear_bits(s->amcc[plane] + T8132_AMCC_STAGES, stages,
				      ARRAY_SIZE(stages));
}

/*
 * T8132, in macOS's order after the cores stopped, ATC0 and ANS lowered and WD2: the memory
 * controller, the gates, the clusters, the clock muxes. Measured from a macOS 25G83 sleep
 * under aura.
 *
 * What's left out on purpose, because the same trace shows macOS doesn't write it in
 * this window: DVFM, CLLT, PTD and an AVE complex, and the AMCC error enables (no error
 * turned a T8132 sleep around under the hypervisor).
 *
 * A hang from here ends at the watchdog, as on T8140.
 */
static void t8132_last_mile(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i;

	/*
	 * macOS has the SoC at its sleep perf state and DCS at state 4 by now. Its firmware
	 * does that, with no write from the CPU. T8140's way gets the same states here.
	 */
	if (sleep_perf_wait(s->clk_perf + CLK_PERF_SOC, SOC_SLEEP_STATE)) {
		writel(SOC_SLEEP_STATE, s->clk_perf + CLK_PERF_SOC);
		sleep_perf_wait(s->clk_perf + CLK_PERF_SOC, SOC_SLEEP_STATE);
	}
	apple_sleep_dcs_down(s);
	t8132_sleep_mcc_stop(s);

	/* Gates twice, the fourth voter, gates. On macOS the voter reads 0 already. */
	for (i = 0; i < 2; i++) {
		writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[0]);
		writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[1]);
	}
	readl(s->pmc_voters + T8132_PMC_VOTER3);
	writel(0, s->pmc_voters + T8132_PMC_VOTER3);
	readl(s->pmc_voters + T8132_PMC_VOTER3_B);
	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[0]);
	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[1]);

	for (i = 0; i < soc->ncluster_cpu; i++)
		writel(0, s->cluster + soc->cluster_cpu[i]);

	s->quiesce_still_on = 0;
	for (i = 0; i < soc->quiesce.count; i++)
		if (sleep_ps_on(s, soc->quiesce.offset[i]))
			s->quiesce_still_on |= BIT(i);

	/* All four read back unchanged on macOS. */
	for (i = 0; i < soc->nclk_mux_parked; i++)
		writel(soc->clk_mux_parked_val, s->clk_mux + soc->clk_mux_parked[i]);

	sleep_ps_save(s);
	/* macOS leaves this one to AUTO_ENABLE at the commit. */
	writel((readl(s->ps_aon + 0xa0) & ~PS_FLAGS) | PS_AUTO_ENABLE, s->ps_aon + 0xa0);
}

/* What the commit has dirtied since the sweep: this stack and the driver's state. */
static void t8132_pre_commit(struct apple_sleep *s)
{
	dcache_clean_poc((unsigned long)task_stack_page(current),
			 (unsigned long)task_stack_page(current) + THREAD_SIZE);
	dcache_clean_poc((unsigned long)s, (unsigned long)(s + 1));
}

/* Back on the boot core through the reset vector, MMU on, in macOS's order. */
static void apple_sleep_return(struct apple_sleep *s)
{
	int i;

	/*
	 * The reset leaves the timer control at its EL2 default, without the event stream
	 * or EL0's counter. The timer driver's CPU PM exit puts it back too, later.
	 */
	write_sysreg(s->cntkctl, cntkctl_el1);
	isb();

	/* This core takes the AIC's IRQs, and a reset may have changed that. */
	write_sysreg_s(s->siq_cfg, SYS_APL_SIQ_CFG_EL1);

	spin_table_stop_finished();
	s->soc->ops->aic_restore();

	s->soc->ops->wake(s);

	dcache_inval_poc((unsigned long)s->mbox, (unsigned long)(s->mbox + 1));
	s->resumes_after = READ_ONCE(s->mbox->resumes);

	if (!s->soc->own_wake)
		for (i = 0; i < s->soc->wake_words; i++)
			s->wake_status[i] = readl(s->wake + s->soc->wake_status + i * 4);
	s->returned = true;
}

/* T8140, in macOS's order: E then the other clusters, the gates, the words, the complex. */
static void t8140_wake(struct apple_sleep *s)
{
	u32 val;
	int i;

	/* The E cluster first, as macOS does, then the others. */
	writel(CLUSTER_ON, s->cluster + s->soc->cluster_cpu[0]);
	s->cluster_e_ret = readl_poll_timeout_atomic(s->cluster + s->soc->cluster_cpu[0], val,
						     val & CLUSTER_E_READY, 1,
						     CLUSTER_TIMEOUT_US);
	for (i = 1; i < s->soc->ncluster_cpu; i++)
		writel(CLUSTER_ON, s->cluster + s->soc->cluster_cpu[i]);

	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_wake[0]);
	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_wake[1]);

	sleep_ps_rewake(s);

	/* Only C2S turns AP_TMM on. */
	s->deep_ran = sleep_ps_woke_on(s, s->soc->ps.ap_tmm);

	val = readl(s->ps + s->soc->ps.sep);
	if (val & PS_DEV_DISABLE)
		writel(val & ~PS_DEV_DISABLE, s->ps + s->soc->ps.sep);

	/* The complex comes back on, and macOS turns it off. */
	writel(0, s->cluster + CLUSTER_AVE);
	s->cluster_ave_ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, !val,
						       1, CLUSTER_TIMEOUT_US);

	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[0]);
	writel(PWRGATE_ON, s->pwrgate + s->soc->pwrgate_cpm[1]);

	sleep_dvfm_unpark(s);

	/* macOS stops it here. Keep a longer one over device resume, until .end. */
	apple_sleep_set_wd2(s, WDT_SUSPEND_SECS);

	/*
	 * What the commit lowered for a driver comes back before anything uses it, in
	 * macOS's order, measured about 14 ms after this point. With no driver, genpd's
	 * books decide, as they do for every other domain.
	 */
	if (s->pcie_ans_stopped)
		s->pcie_ans_on_ret = apple_sleep_raise_apcie(s);
	s->apcie_port_down = true;
	s->pins_lost = s->pins_saved_valid;
}

/*
 * The first thing Linux runs after a wake, MMU off and no stack. The last wake program
 * leaves WD2 running for 5 s: restart it for 60 s, in the order apple_sleep_set_wd2() uses. No
 * SPMI here: macOS leaves that bus alone for a long time after a wake.
 */
asm(
"	.pushsection .text, \"ax\"\n"
"	.type	t8132_resume_stub, %function\n"
"t8132_resume_stub:\n"
"	movz	x9, #0x882b, lsl #16\n"
"	movk	x9, #0x3, lsl #32\n"
"	str	wzr, [x9, #0x2c]\n"
"	str	wzr, [x9, #0x20]\n"
"	movz	w10, #0xa800\n"
"	movk	w10, #0x55d4, lsl #16\n"
"	str	w10, [x9, #0x28]\n"
"	mov	w10, #1\n"
"	str	w10, [x9, #0x2c]\n"
"	b	cpu_resume\n"
"	.size	t8132_resume_stub, . - t8132_resume_stub\n"
"	.popsection\n"
);
void t8132_resume_stub(void);

/*
 * T8132: T8140's steps with its own words, no AVE complex to bring back, and ATC0's words
 * after the others.
 */
static void t8132_wake(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	u32 val;
	int i;

	writel(CLUSTER_ON, s->cluster + soc->cluster_cpu[0]);
	s->cluster_e_ret = readl_poll_timeout_atomic(s->cluster + soc->cluster_cpu[0], val,
						     val & CLUSTER_E_READY, 1,
						     CLUSTER_TIMEOUT_US);
	for (i = 1; i < soc->ncluster_cpu; i++)
		writel(CLUSTER_ON, s->cluster + soc->cluster_cpu[i]);

	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_wake[0]);
	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_wake[1]);

	sleep_ps_rewake(s);
	/*
	 * The commit saved ATC0's words already lowered, so the common restore skips them, and
	 * their domains still count as on: nothing else would raise them. Reverse of the lowering.
	 */
	sleep_ps_restore(s->ps + T8132_PS_ATC0_COMMON, s->atc0_saved[2]);
	sleep_ps_restore(s->ps_aon + T8132_PS_AON_ATC0_USB_AON, s->atc0_saved[1]);
	sleep_ps_restore(s->ps_aon + T8132_PS_AON_ATC0_USB, s->atc0_saved[0]);

	s->deep_ran = sleep_ps_woke_on(s, soc->ps.ap_tmm);

	val = readl(s->ps + soc->ps.sep);
	if (val & PS_DEV_DISABLE)
		writel(val & ~PS_DEV_DISABLE, s->ps + soc->ps.sep);

	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[0]);
	writel(PWRGATE_ON, s->pwrgate + soc->pwrgate_cpm[1]);

	apple_sleep_set_wd2(s, WDT_SUSPEND_SECS);

	if (s->pcie_ans_stopped)
		s->pcie_ans_on_ret = apple_sleep_raise_apcie(s);
	s->apcie_port_clk_ret[0] = 0;
	s->apcie_port_clk_ret[1] = 0;
	if (s->pcie_ans_stopped && !s->pcie_ans_on_ret && s->apcie_ports_unowned) {
		s->apcie_port_clk_ret[0] = sleep_apcie_port_clock(s->apcie_port,
								  s->apcie_port_phy);
		s->apcie_port_clk_ret[1] = sleep_apcie_port_clock(s->apcie_port2,
								  s->apcie_port2_phy);
	}
	s->apcie_port_down = true;
	s->pins_lost = s->pins_saved_valid;
}

static int apple_sleep_smc_late_notice(struct apple_sleep *s)
{
	if (!s->soc->smc_policy && !apple_sleep_smc_defer_notice())
		return 0;
	if (!s->smc.dev || !s->smc.late_notice)
		return -ENODEV;
	if ((s->soc->early_context || apple_sleep_smc_defer_notice()) &&
	    (!s->smc.saved_ops || !s->smc.held))
		return -EBUSY;
	return s->smc.late_notice(s->smc.data, &s->smc_idle_sent, &s->smc_idle_ack);
}

/* Past the point of no return: one pmVC write alone, as macOS's after ANS. */
static int apple_sleep_smc_pmvc(struct apple_sleep *s)
{
	if (!s->smc.dev || !s->smc.held || !s->smc.saved_ops || !s->smc.saved_ops->pmvc)
		return -ENODEV;
	return s->smc.saved_ops->pmvc(s->smc.data);
}

static int apple_sleep_smc_commit_notice(struct apple_sleep *s)
{
	if (!s->smc.dev || !s->smc.held || !s->smc.saved_ops ||
	    !s->smc.saved_ops->commit_notice)
		return -ENODEV;
	return s->smc.saved_ops->commit_notice(s->smc.data,
		&s->smc_idle_sent, &s->smc_idle_ack);
}

static void apple_sleep_enter_cleanup(struct apple_sleep *s)
{
	if (s->regs_staged)
		s->soc->ops->undo_regs(s);
	s->regs_staged = false;
	apple_sleep_stray_restore(s);
	if (s->wake_masks_saved)
		sleep_wake_restore(s);
	s->wake_masks_saved = false;
	s->entry_prepared = false;
}

/*
 * T6050, in macOS's order on the M5 Pro: the display sets, then GFX and GFX1, the first always-on stop and so the
 * point of no return, then SEP; ANS, the PMP and the SMC follow as before. Always-on last, as
 * on the Neo, where the display also goes off before SEP.
 */
static void apple_sleep_display_then_gpu(struct apple_sleep *s)
{
	int ret;

	s->entry_committed = true;
	ret = s->soc->ops->display_stop(s);
	if (ret)
		panic("Apple committed display stop failed: %d", ret);
	apple_sleep_coprocs_stop(s, true);
}

/*
 * The entry of a SoC with early_context, called with the boot core's context saved. All
 * returning checks precede enclave/final-stop.
 */
static int apple_sleep_enter_prepare(struct apple_sleep *s)
{
	const char *step = "SoC check";
	int ret;

	if (s->soc->ops->check) {
		ret = s->soc->ops->check(s);
		if (ret) {
			if (s->soc->regions)
				dev_info(s->dev, "refused at .enter: %s: %d\n", step, ret);
			return ret;
		}
	}
	ret = apple_sleep_refuse(s);
	if (ret)
		return ret;
	ret = apple_sleep_cpus_hold(s);
	if (!ret) {
		step = "display prepare";
		if (s->soc->regions)
			s->regs_staged = true;
		ret = s->soc->ops->display_off(s);
	}
	if (!ret && s->soc->ops->stage) {
		step = "stage";
		s->regs_staged = true; /* Partial stage must also be undone on failure. */
		ret = s->soc->ops->stage(s);
	}
	if (!ret) {
		step = "coprocessor checks";
		ret = apple_sleep_coprocs_check(s);
	}
	if (!ret && !s->soc->ops->commit_notice) {
		step = "SMC notice";
		ret = apple_sleep_smc_late_notice(s);
	}
	if (!ret && s->soc->ops->final_check) {
		step = "final check";
		ret = s->soc->ops->final_check(s);
	}
	if (!ret && pm_wakeup_pending()) {
		step = "wakeup pending";
		ret = -EBUSY;
	}
	if (ret && s->soc->regions)
		dev_info(s->dev, "refused at .enter: %s: %d\n", step, ret);
	if (!ret && s->soc->ops->display_stop) {
		apple_sleep_display_then_gpu(s);
		if (s->soc->ops->before_enclave)
			s->soc->ops->before_enclave(s);
		ret = apple_sleep_enclave_stop(s);
		if (ret)
			panic("Apple SEP refused after the always-on GPU stop: %d", ret);
	} else if (!ret) {
		ret = apple_sleep_enclave_stop(s);
	}
	if (ret) {
		dev_dbg(s->dev, "refused before any commit coprocessor stop: %d\n", ret);
		spin_table_unhold_parked();
		apple_sleep_enter_cleanup(s);
		return ret;
	}
	/* SEP success belongs to irreversible commitment, never cancellation. */
	s->entry_committed = true;
	/* No returning error path below this point. */
	if (!s->soc->ops->display_stop)
		apple_sleep_coprocs_stop(s, true);
	if (!s->soc->own_wake) {
		sleep_wake_arm(s);
		s->wake_masks_saved = true;
	}
	s->soc->ops->prepare_regs(s);
	s->regs_staged = true;
	apple_sleep_coprocs_stop(s, false);
	if (s->soc->ops->commit_notice) {
		ret = s->soc->ops->commit_notice(s);
		if (ret)
			panic("Apple committed SMC notice failed after peer stop: %d", ret);
	}
	if (!s->soc->regions)
		apple_sleep_usb_down(s);
	s->deep_once = READ_ONCE(sleep_deep_requested);
	if (s->deep_once)
		sleep_slp_ddr_set_deep_once(true);
	s->entry_prepared = true;
	return 0;
}

static int apple_sleep_enter_saved(unsigned long unused)
{
	struct apple_sleep *s = apple_sleep;
	int ret = s->soc->ops->cpu_prepare(s->mbox->boot_mpidr);

	if (ret)
		return ret;
	ret = apple_sleep_enter_prepare(s);
	if (ret)
		return ret;
	pm_set_resume_via_firmware();
	/* cpu_suspend has already saved boot context. Commit never returns. */
	return apple_sleep_commit(0);
}

/* A SoC with early_context: the whole entry runs inside cpu_suspend(), on the saved context. */
static int apple_sleep_enter_early(struct apple_sleep *s)
{
	int ret;

	s->entry_committed = false;
	s->cntkctl = read_sysreg(cntkctl_el1);
	ret = cpu_suspend(0, apple_sleep_enter_saved);
	if (ret && s->entry_committed)
		panic("Apple committed sleep returned without physical wake: %d", ret);
	if (!ret) {
		apple_sleep_return(s);
		if (s->deep_once)
			sleep_slp_ddr_set_deep_once(false);
	}
	if (s->entry_prepared)
		apple_sleep_enter_cleanup(s);
	s->soc->ops->cpu_restore(s->mbox->boot_mpidr);
	return ret;
}

static void t8142_mem_save(struct apple_sleep *s);
static void t8142_clk_save(struct apple_sleep *s);

static int apple_sleep_enter(suspend_state_t state)
{
	struct apple_sleep *s = apple_sleep;
	int ret;

	if (s->soc->early_context)
		return apple_sleep_enter_early(s);

	/* Refuse while all peer stops are still reversible. */
	if (apple_sleep_smc_defer_notice()) {
		if (!s->smc.dev || !s->smc.late_notice)
			return -ENODEV;
		if (!s->smc.saved_ops || !s->smc.held)
			return -EBUSY;
	}

	sleep_note(s, 0x10);
	ret = apple_sleep_refuse(s);
	if (ret) {
		sleep_note(s, 0x2f);
		return ret;
	}
	sleep_note(s, 0x29);
	/*
	 * An untested SoC stops here unless asked: every check has run, the stray words are
	 * lowered (and come back below), and nothing that can't be undone has.
	 */
	if (s->soc->untested && !READ_ONCE(sleep_untested_commit)) {
		sleep_note(s, 0x31);
		apple_sleep_stray_restore(s);
		/* nub-spmi0's latched sources as a sleep would find them, for apple,spmi-pmu-sources:
		 * read only while the DT names no bits (then this acknowledges them, as the entry does).
		 */
		sleep_spmi_sources(s, 0);
		sleep_spmi_sources_report(s);
		/* The boot core's power controls here, to diff against macOS's at its commit (aura's
		 * R.* ring markers). Reads only: interrupts are off and the secondaries are held.
		 */
		if (READ_ONCE(sleep_dry_regs) && s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm && is_kernel_in_hyp_mode())
			dev_info(s->dev, "dry run: boot cpu%d C1_6 %#llx C6_2 %#llx C5_0 %#llx C10_4 %#llx C7_4 %#llx C1_1 %#llx\n",
				 raw_smp_processor_id(), read_sysreg_s(SLEEP_M5_C1_6),
				 read_sysreg_s(sys_reg(3, 5, 15, 6, 2)), read_sysreg_s(sys_reg(3, 5, 15, 5, 0)),
				 read_sysreg_s(sys_reg(3, 4, 15, 10, 4)), read_sysreg_s(sys_reg(3, 1, 15, 7, 4)),
				 read_sysreg_s(sys_reg(3, 5, 15, 1, 1)));
		/* What XNU's per-core init (26B131 kc +0x4ed3d3c) writes on every core: c1_5 = 0,
		 * c14_6 = 3, c10_4 = 0, c9_6 |= ..|0xf, c0_5 = 1, c13_4 = 2; cntkctl |= 3.
		 */
		if (READ_ONCE(sleep_dry_regs) && s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm && is_kernel_in_hyp_mode())
			dev_info(s->dev, "dry run: boot cpu XNU-init regs: s3_1_c15_c1_5 %#llx s3_4_c15_c14_6 %#llx s3_4_c15_c9_6 %#llx s3_1_c15_c0_5 %#llx s3_1_c15_c13_4 %#llx cntkctl %#llx\n",
				 read_sysreg_s(sys_reg(3, 1, 15, 1, 5)), read_sysreg_s(sys_reg(3, 4, 15, 14, 6)),
				 read_sysreg_s(sys_reg(3, 4, 15, 9, 6)), read_sysreg_s(sys_reg(3, 1, 15, 0, 5)),
				 read_sysreg_s(sys_reg(3, 1, 15, 13, 4)), read_sysreg(cntkctl_el1));
		if (READ_ONCE(sleep_dry_regs) && s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm) {
			dev_info(s->dev, "dry run: boot cpu FIQ sources: PMCR0 %#llx UPMCR0 %#llx UPMSR %#llx IPI_SR %#llx cntp/cntv_el0 %#llx/%#llx cntp/cntv_el02 %#llx/%#llx VM_TMR_FIQ_ENA %#llx ISR %#llx CLPC C5_0 %#llx; still firing %#x\n",
				 read_sysreg_s(T8142_PMCR0), read_sysreg_s(T8142_UPMCR0), read_sysreg_s(T8142_UPMSR),
				 read_sysreg_s(SYS_APL_IPI_SR_EL1), read_sysreg(cntp_ctl_el0), read_sysreg(cntv_ctl_el0),
				 is_kernel_in_hyp_mode() ? read_sysreg_s(SYS_CNTP_CTL_EL02) : 0,
				 is_kernel_in_hyp_mode() ? read_sysreg_s(SYS_CNTV_CTL_EL02) : 0,
				 is_kernel_in_hyp_mode() ? read_sysreg_s(T8142_VM_TMR_FIQ_ENA) : 0,
				 read_sysreg(isr_el1), read_sysreg_s(T8142_CLPC_C5_0), t8142_fiq_sources());
			t8142_ed_report(s->dev);
			if (is_kernel_in_hyp_mode()) {
				u64 o = read_sysreg_s(SLEEP_M5_C1_6), w;

				write_sysreg_s(o | SLEEP_M5_C1_6_PWRDN, SLEEP_M5_C1_6);
				isb();
				w = read_sysreg_s(SLEEP_M5_C1_6);
				write_sysreg_s(o, SLEEP_M5_C1_6);
				isb();
				dev_info(s->dev, "dry run: C1_6 write test: %#llx | 0x1c000000 reads back %#llx (%s)\n",
					 o, w, (w & SLEEP_M5_C1_6_PWRDN) == SLEEP_M5_C1_6_PWRDN ? "sticks" : "IGNORED");
			}
		}
		/* The wake restores' reads, here too, so a read that hangs shows in the ladder. */
		if (s->soc->ops->last_mile == t8142_last_mile) {
			t8142_mem_save(s);
			t8142_clk_save(s);
			dev_info(s->dev, "dry run: wake saves: mem %d (amcc0 %#llx cfg0 %#llx ch0 %#x/%#x/%#x) clk %d (slot0 %#x:%#x) early_wfi %d\n",
				 s->mem_saved, s->amcc_saved[0], s->amcc_cfg_saved[0][0],
				 s->mcc_saved[0][0], s->mcc_saved[0][MCC_ENABLES / 4],
				 s->mcc_saved[0][MCC_LANES / 4], s->clk_saved,
				 s->clk_slot_saved[0][1], s->clk_slot_saved[0][0],
				 s->soc->early_cpu_wfi && READ_ONCE(sleep_early_wfi));
			s->mem_saved = s->clk_saved = false;
		}
		/* Read-only: what the match would find in the words macOS leaves on (ps_keep). */
		if (s->soc->nps_keep) {
			char line[400];
			int i, n = 0;

			for (i = 0; i < s->soc->nps_keep; i++) {
				const struct apple_sleep_ps_keep *k = &s->soc->ps_keep[i];

				n += scnprintf(line + n, sizeof(line) - n, " %s %#x", k->name,
					       readl((k->aon ? s->ps_aon : s->ps) + k->offset));
			}
			n += scnprintf(line + n, sizeof(line) - n, " ATC0_COMMON %#x ATC0_USB_AON %#x ATC0_USB %#x",
				       readl(s->ps + T8142_PS_ATC0_COMMON),
				       readl(s->ps_aon + T8142_PS_AON_ATC0_USB_AON),
				       readl(s->ps_aon + T8142_PS_AON_ATC0_USB));
			dev_info(s->dev, "dry run: ps_keep %#x, the words:%s\n",
				 READ_ONCE(sleep_ps_keep), line);
		}
		/* Read-only: XNU's boot AUTO_ENABLE list as Linux has it (apple_sleep.boot_auto) */
		if (sleep_boot_auto_count(s)) {
			char line[700];
			unsigned int i, n = 0, missing = 0, autoed = 0, off_words = 0;
			u32 v;

			for (i = 0; i < sleep_boot_auto_count(s); i++) {
				bool aon = i >= s->soc->ps_boot_auto.count;
				u32 off = aon ? s->soc->ps_aon_boot_auto.offset[i - s->soc->ps_boot_auto.count] :
						s->soc->ps_boot_auto.offset[i];

				v = readl(sleep_boot_auto_reg(s, i));
				if (v & PS_AUTO_ENABLE) {
					autoed++;
				} else if (FIELD_GET(PS_TARGET, v) && FIELD_GET(PS_ACTUAL, v)) {
					missing++;
					n += scnprintf(line + n, sizeof(line) - n, " %s%x=%x",
						       aon ? "a" : "", off, v);
				} else {
					off_words++;
				}
			}
			dev_info(s->dev, "dry run: boot_auto %d: of XNU's %u AUTO_ENABLE words %u already have it, %u are off, %u on without it:%s\n",
				 READ_ONCE(sleep_boot_auto_on), sleep_boot_auto_count(s), autoed,
				 off_words, missing, line);
			/*
			 * The one power-word write a dry run makes (apple_sleep.boot_auto_dry): XNU's
			 * boot write on those words, each read back at once and 1 ms later, then undone.
			 */
			if (READ_ONCE(sleep_boot_auto_on) && READ_ONCE(sleep_boot_auto_dry)) {
				u32 now[64];
				unsigned int set;

				sleep_note(s, 0x3d);
				set = sleep_boot_auto(s);
				mdelay(1);
				n = 0;
				for (i = 0; i < sleep_boot_auto_count(s); i++) {
					bool aon = i >= s->soc->ps_boot_auto.count;
					u32 off = aon ? s->soc->ps_aon_boot_auto.offset[i - s->soc->ps_boot_auto.count] :
							s->soc->ps_boot_auto.offset[i];

					if (!(s->boot_auto_set & BIT_ULL(i)))
						continue;
					now[i] = readl(sleep_boot_auto_reg(s, i));
					n += scnprintf(line + n, sizeof(line) - n, " %s%x=%x", aon ? "a" : "",
						       off, now[i]);
				}
				sleep_boot_auto_undo(s);
				dev_info(s->dev, "dry run: boot_auto write test: %u set, %u read AUTO_ENABLE back at once; 1 ms later:%s (undone)\n",
					 set, s->boot_auto_stuck, line);
			}
		}
		/* Read-only: the memory-controller start's fingerprint and the PMC's four voters */
		if (s->soc->mem_boot) {
			u64 pa = 0, got = 0, want = 0;
			unsigned int bad = t8142_mem_fingerprint(s, &pa, &got, &want);

			dev_info(s->dev, "dry run: mem_boot %d mem_floor %d: fingerprint %u of 34 differ (first %#llx: %#llx, J813 %#llx); PMC voters +0/+4: %#x/%#x %#x/%#x %#x/%#x %#x/%#x\n",
				 READ_ONCE(sleep_mem_boot), READ_ONCE(sleep_mem_floor), bad, pa, got, want,
				 readl(s->pmc_voters), readl(s->pmc_voters + 4),
				 readl(s->pmc_voters + PMC_VOTER_STRIDE),
				 readl(s->pmc_voters + PMC_VOTER_STRIDE + 4),
				 readl(s->pmc_voters + 2 * PMC_VOTER_STRIDE),
				 readl(s->pmc_voters + 2 * PMC_VOTER_STRIDE + 4),
				 readl(s->pmc_voters + 3 * PMC_VOTER_STRIDE),
				 readl(s->pmc_voters + 3 * PMC_VOTER_STRIDE + 4));
		}
		/* Read-only: every word of the match's off lists that reads on, i.e. that it lowers. */
		if (s->soc->ps_macos_off.count || s->soc->ps_aon_macos_off.count) {
			const struct apple_sleep_words *off = &s->soc->ps_macos_off;
			const struct apple_sleep_words *aoff = &s->soc->ps_aon_macos_off;
			char line[800];
			int i, n = 0;
			u32 v;

			for (i = 0; i < off->count; i++) {
				v = readl(sleep_power_reg(s, off->offset[i]));
				if (FIELD_GET(PS_ACTUAL, v))
					n += scnprintf(line + n, sizeof(line) - n, " %x=%x",
						       off->offset[i], v);
			}
			n += scnprintf(line + n, sizeof(line) - n, " | aon");
			for (i = 0; i < aoff->count; i++) {
				v = readl(s->ps_aon + aoff->offset[i]);
				if (FIELD_GET(PS_ACTUAL, v))
					n += scnprintf(line + n, sizeof(line) - n, " %x=%x",
						       aoff->offset[i], v);
			}
			dev_info(s->dev, "dry run: macos_off words on now (the match lowers them unless kept):%s\n",
				 line);
		}
		/* Read-only: what apple_sleep.dispext lowers (T8142's external DCPs). */
		if (s->soc->dispext_optional) {
			char line[400];
			int i, k, n = 0;

			for (i = 1; i < s->soc->ndisp; i++) {
				n += scnprintf(line + n, sizeof(line) - n, " | ext%d %s:", i - 1,
					       !s->display[i].np ? "no node" :
					       s->display[i].bound ? "bound" : "unbound");
				for (k = 0; k < 3; k++)
					n += scnprintf(line + n, sizeof(line) - n, " %x=%x",
						       s->soc->ps.disp[i][k],
						       readl(s->ps + s->soc->ps.disp[i][k]));
				n += scnprintf(line + n, sizeof(line) - n, " run %d",
					       sleep_asc_running(s, s->soc->ps.disp[i][0],
								 s->display[i].asc));
				if (s->soc->disp_wr_dis[i][0])
					n += scnprintf(line + n, sizeof(line) - n, " wr-dis %d",
						       !!(readl(s->ps + s->soc->disp_wr_dis[i][0]) &
							  BIT(s->soc->disp_wr_dis[i][1])));
			}
			dev_info(s->dev, "dry run: dispext %d:%s\n", READ_ONCE(sleep_dispext), line);
		}
		/* Read-only: what apple_sleep.extra_off would lower (only those on). */
		if (s->soc->nextra_off) {
			static char line[900];
			int i, n = 0;

			for (i = 0; i < s->soc->nextra_off; i++)
				n += scnprintf(line + n, sizeof(line) - n, " %s %x=%x",
					       s->soc->extra_off[i].name, s->soc->extra_off[i].offset,
					       readl(sleep_power_reg(s, s->soc->extra_off[i].offset)));
			for (i = 0; i < s->soc->ndisp; i++)
				if (s->soc->disp_adma[i])
					n += scnprintf(line + n, sizeof(line) - n, " adma%d %x=%x", i,
						       s->soc->disp_adma[i],
						       readl(sleep_power_reg(s, s->soc->disp_adma[i])));
			dev_info(s->dev, "dry run: extra_off %d:%s\n", READ_ONCE(sleep_extra_off), line);
		}
		/*
		 * Read-only, before the display, coprocessor and match steps: every ps word on
		 * (index:offset=value; * = macOS has it off at its commit), the list the commit's PMU
		 * digest can only count; aon: every word on without AUTO_ENABLE.
		 */
		if (s->soc->ops->macos_off_at_commit) {
			const struct apple_sleep_words *all = &s->soc->ps_all;
			const struct apple_sleep_words *aon = &s->soc->ps_aon_all;
			/* Interrupts are off here: no allocation. */
			static char line[1024];
			int i, n = 0;
			u32 v;

			line[0] = 0;
			for (i = 0; i < all->count; i++) {
				v = readl(s->ps + all->offset[i]);
				if (!FIELD_GET(PS_ACTUAL, v))
					continue;
				n += scnprintf(line + n, 1024 - n, " %d:%x=%x%s", i, all->offset[i], v,
					       s->soc->ops->macos_off_at_commit(all->offset[i]) ? "*" : "");
				/* printk would cut a long line: a new one every 24 words */
				if (n > 400) {
					dev_info(s->dev, "dry run: ps on (* = off at macOS's commit):%s\n",
						 line);
					n = 0;
					line[0] = 0;
				}
			}
			dev_info(s->dev, "dry run: ps on (* = off at macOS's commit):%s\n", line);
			line[0] = 0;
			for (i = 0, n = 0; i < aon->count; i++) {
				v = readl(s->ps_aon + aon->offset[i]);
				if (!FIELD_GET(PS_ACTUAL, v) || (v & PS_AUTO_ENABLE))
					continue;
				n += scnprintf(line + n, 1024 - n, " %d:%x=%x", i, aon->offset[i], v);
			}
			dev_info(s->dev, "dry run: aon on without AUTO:%s\n", line);
		}
		dev_info(s->dev,
			 "dry run: every check passed; refusing before the commit (untested SoC, apple_sleep.untested_commit=1 lets it through)\n");
		return -EAGAIN;
	}
	if (sleep_boot_auto_count(s)) {
		sleep_note(s, 0x3e);
		sleep_note_at(s, 2, min(sleep_boot_auto(s), 255));
	}
	sleep_note(s, 0x40);
	ret = apple_sleep_cpus_hold(s);
	/* As on macOS, the display goes off before the GPU. */
	if (!ret) {
		sleep_note(s, 0x41);
		ret = sleep_display_off(s);
	}
	if (!ret) {
		sleep_note(s, 0x42);
		ret = apple_sleep_coprocs_check(s);
	}
	/* Cover the platform's own preparation, after the PM core's early check. */
	if (!ret && pm_wakeup_pending())
		ret = -EBUSY;
	if (!ret) {
		sleep_note(s, 0x43);
		ret = apple_sleep_enclave_stop(s);
	}
	if (ret) {
		dev_dbg(s->dev, "refused before any commit coprocessor stop: %d\n", ret);
		spin_table_unhold_parked();
		sleep_dispext_restore(s);
		sleep_boot_auto_undo(s);
		apple_sleep_stray_restore(s);
		return ret;
	}
	sleep_note(s, 0x44);
	sleep_ps_wr_dis_clear(s);
	apple_sleep_coprocs_stop(s, true);
	if (s->soc->mem_boot && READ_ONCE(sleep_mem_boot)) {
		sleep_note(s, 0x4c);
		t8142_mem_boot_entry(s);
	}
	s->mem_floor_done = false;
	if (s->soc->mem_boot && READ_ONCE(sleep_mem_floor)) {
		/* T6050's stage: the floor's voter stepped down 2, then 1, 9 ms apart */
		sleep_note(s, 0x4d);
		writel(2, s->pmc_voters + PMC_VOTER_STRIDE * 1);
		mdelay(9);
		writel(1, s->pmc_voters + PMC_VOTER_STRIDE * 1);
		mdelay(9);
		s->mem_floor_done = true;
	}

	sleep_note(s, 0x45);
	sleep_wake_arm(s);
	sleep_note(s, 0x46);
	apple_sleep_match(s);
	sleep_note(s, 0x48);

	/* Acknowledge a leftover done flag, as the DCS word has none on macOS. */
	if (s->soc->ack_cluster_dcs && (readl(s->cluster_dcs) & CLUSTER_UP_DONE))
		writel(readl(s->cluster_dcs), s->cluster_dcs);
	apple_sleep_coprocs_stop(s, false);
	/*
	 * J813 native trace: ANS idle completes before SMC slpa/AP IDLE.
	 * In the battery failure SMC faulted at ANS + 0xc110 after ANS's
	 * idle ACK was missed. Keep its final notice behind the peer stops
	 * and ahead of the domain power-down in the last mile.
	 */
	if (apple_sleep_smc_defer_notice()) {
		ret = apple_sleep_smc_late_notice(s);
		dev_info(s->dev, "TEST: SMC notice after peer stops: %d\n", ret);
		if (ret)
			panic("J813 SMC notice failed after peer stops: %d", ret);
	}
	/* Done by now unless no SMC driver is bound. */
	apple_sleep_usb_down(s);
	s->deep_once = READ_ONCE(sleep_deep_requested);
	if (s->deep_once)
		sleep_slp_ddr_set_deep_once(true);

	s->cntkctl = read_sysreg(cntkctl_el1);
	pm_set_resume_via_firmware();
	sleep_note(s, 0x4a);
	ret = cpu_suspend(0, apple_sleep_commit);
	if (ret)
		panic("Apple committed sleep returned without physical wake: %d", ret);
	apple_sleep_return(s);
	sleep_note(s, 0x50);
	if (s->deep_once)
		sleep_slp_ddr_set_deep_once(false);

	sleep_ps_wr_dis_woke(s);
	apple_sleep_match_undo(s);
	apple_sleep_stray_restore(s);
	sleep_wake_restore(s);

	return ret;
}

/* From here to .end a hang resets the machine. Nobody has to be there to recover it. */
static int apple_sleep_begin(suspend_state_t state)
{
	int ret;
	/* The last sleep's cut is its port controller's to take, by its .complete. */
	WRITE_ONCE(sleep_debug_link_cut, false);
	WRITE_ONCE(sleep_lid_closed, -ENODATA);
	WRITE_ONCE(sleep_radio_woke, -ENODATA);
	/* The PM core also calls .begin for s2idle when no s2idle ops are installed. */
	if (state != PM_SUSPEND_MEM)
		return 0;

	ret = apple_sleep_profile_check(apple_sleep);
	if (ret)
		return ret;

	if (apple_sleep->soc->ops->begin) {
		ret = apple_sleep->soc->ops->begin(apple_sleep);
		if (ret)
			return ret;
	}

	if (apple_sleep) {
		apple_sleep->apcie_port_stopped = false;
		apple_sleep->usb_down_ran = false;
		apple_sleep->smc_woke_alone = false;
		sleep_pins_save(apple_sleep);
	}
	apple_sleep_set_wd2(apple_sleep, WDT_SUSPEND_SECS);
	pm_set_suspend_via_firmware();
	sleep_ans_late_open();

	return 0;
}

static void apple_sleep_end(void)
{
	WRITE_ONCE(sleep_deep_requested, false);
	sleep_ans_late_release(true);
	sleep_v27_scope_end();
	if (pm_suspend_via_firmware() && apple_sleep->soc->ops->finish)
		apple_sleep->soc->ops->finish(apple_sleep);
	/* Nothing left by now after a wake; the platform wake takes it */
	if (pm_suspend_via_firmware() && apple_sleep->soc->ops->pmu_wake)
		apple_sleep->soc->ops->pmu_wake(apple_sleep);
	if (pm_suspend_via_firmware())
		writel(0, apple_sleep->wdt + WDT_WD2_CTL);
}

/*
 * macOS turns this complex off at boot and again at the end of each wake, so at the commit
 * its word is already 0 and the engine has no sequence to run for it. Turning it off at the
 * commit instead puts that sequence, with its untimed wait, right before the CPU complexes'.
 */
static void apple_sleep_ave_off(struct apple_sleep *s)
{
	u32 val;
	int ret;

	/* Target 0, and the leftover done flag from iBoot's up, as XNU's first write does. */
	writel(CLUSTER_UP_DONE, s->cluster + CLUSTER_AVE);
	ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, !val, 1, CLUSTER_TIMEOUT_US);
	if (ret)
		dev_warn(s->dev, "AVE complex didn't go off: %#x\n", val);
}

/*
 * The encoders' system power domains (VENC0_SYS, VENC1_SYS) only power on with the complex
 * up: with it off, VENC0_SYS refuses its target and the complex is left pending, and with
 * one of them on the complex doesn't go off. genpd ignores a failed power-on in the noirq
 * resume, so without the parent link the first wake raised VENC0_SYS under a complex that
 * .prepare and the wake had turned off, and apple-dart's restore of the encoder's DARTs in
 * that unpowered block reset the SoC. As their parent, the complex comes on before either
 * of them and goes off after both: off at boot and after each wake, on while an encoder is
 * in use, as macOS has it. The PMC's cluster vote for the encoders (VENC-CLSTR-V) is the
 * same: raised with the complex off, the complex's next power-up never completes, and with
 * the complex off first the vote doesn't drop. So it's a subdomain too.
 */
static int apple_sleep_ave_pd_on(struct generic_pm_domain *pd)
{
	struct apple_sleep *s = container_of(pd, struct apple_sleep, ave_pd);
	u32 val;
	int ret;

	/* A raise over a lower still in progress never completes. */
	ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, !(val & CLUSTER_PENDING),
					1, CLUSTER_TIMEOUT_US);
	if (ret) {
		dev_err(s->dev, "AVE complex still busy: %#x\n", val);
		return ret;
	}

	writel(CLUSTER_ON, s->cluster + CLUSTER_AVE);
	ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, val & CLUSTER_UP_DONE, 1,
					CLUSTER_TIMEOUT_US);
	if (ret)
		dev_err(s->dev, "AVE complex didn't come on: %#x\n", val);

	return ret;
}

static int apple_sleep_ave_pd_off(struct generic_pm_domain *pd)
{
	struct apple_sleep *s = container_of(pd, struct apple_sleep, ave_pd);
	unsigned int i;
	u32 val;
	int ret;

	/*
	 * Each encoder domain reads actual 0 a moment before its switch is done. Lowered
	 * before that, the complex stays pending and its next raise never completes, so wait
	 * for BUSY to clear too, as macOS waits for a lowered word to read 0 in its low bits.
	 */
	for (i = 0; i < s->soc->nave_domains; i++) {
		if (!(s->ave_linked & BIT(i)))
			continue;
		ret = readl_poll_timeout_atomic(s->ps + s->soc->ave_domains[i], val,
						!(val & (PS_BUSY | PS_ACTUAL)), 1,
						PS_OFF_TIMEOUT_US);
		if (ret) {
			dev_err(s->dev, "encoder domain %#x not off: %#x\n",
				s->soc->ave_domains[i], val);
			return -EBUSY;
		}
	}

	/* Target 0, and the done flag of its last power-up. */
	writel(CLUSTER_UP_DONE, s->cluster + CLUSTER_AVE);
	ret = readl_poll_timeout_atomic(s->cluster + CLUSTER_AVE, val, !val, 1,
					CLUSTER_TIMEOUT_US);
	if (ret)
		dev_err(s->dev, "AVE complex didn't go off: %#x\n", val);

	return ret;
}

/* The PMGR node "ps" maps */
static struct device_node *apple_sleep_ps_pmgr(struct apple_sleep *s)
{
	struct device_node *np;
	struct resource res;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr")
		if (!of_address_to_resource(np, 0, &res) && res.start == s->ps_res.start)
			return np;

	return NULL;
}

static void apple_sleep_ave_pd_remove(void *data)
{
	struct apple_sleep *s = data;
	struct of_phandle_args parent = { .np = s->dev->of_node }, child = {};
	unsigned int i;

	for (i = 0; i < s->ave_nchildren; i++) {
		child.np = s->ave_children[i];
		of_genpd_remove_subdomain(&parent, &child);
		of_node_put(s->ave_children[i]);
	}
	s->ave_nchildren = 0;
	of_genpd_del_provider(s->dev->of_node);
	pm_genpd_remove(&s->ave_pd);
	s->ave_pd_owned = false;
}

/* The enabled encoder domains, and with any of them the PMC's encoder cluster vote. */
static void apple_sleep_ave_find(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	struct device_node *pmgr, *pmc, *np;
	unsigned int i;
	u32 val;

	pmgr = apple_sleep_ps_pmgr(s);
	if (!pmgr)
		return;
	for_each_available_child_of_node(pmgr, np) {
		for (i = 0; i < soc->nave_domains; i++) {
			if (of_property_read_u32(np, "reg", &val) || val != soc->ave_domains[i] ||
			    s->ave_nchildren == ARRAY_SIZE(s->ave_children))
				continue;
			s->ave_linked |= BIT(i);
			s->ave_children[s->ave_nchildren++] = of_node_get(np);
		}
	}
	of_node_put(pmgr);
	if (!s->ave_nchildren || !soc->ave_pmc)
		return;

	for_each_compatible_node(pmc, NULL, soc->ave_pmc) {
		if (!of_device_is_available(pmc))
			continue;
		for_each_available_child_of_node(pmc, np) {
			if (of_property_read_u32(np, "apple,cluster-bit", &val) ||
			    val != soc->ave_pmc_cluster ||
			    s->ave_nchildren == ARRAY_SIZE(s->ave_children))
				continue;
			s->ave_children[s->ave_nchildren++] = of_node_get(np);
		}
	}
}

/* The complex as the parent of the encoders' domains, or left to .prepare if none is enabled. */
static int apple_sleep_ave_pd_init(struct apple_sleep *s)
{
	struct of_phandle_args parent = { .np = s->dev->of_node }, child = {};
	unsigned int i;
	int ret;

	apple_sleep_ave_find(s);
	if (!s->ave_nchildren)
		return 0;

	s->ave_pd.name = "ave_complex";
	s->ave_pd.power_on = apple_sleep_ave_pd_on;
	s->ave_pd.power_off = apple_sleep_ave_pd_off;
	s->ave_pd.flags = GENPD_FLAG_IRQ_SAFE | GENPD_FLAG_NO_STAY_ON | GENPD_FLAG_NO_SYNC_STATE;
	ret = pm_genpd_init(&s->ave_pd, NULL,
			    !(readl(s->cluster + CLUSTER_AVE) & CLUSTER_ON));
	if (!ret) {
		ret = of_genpd_add_provider_simple(s->dev->of_node, &s->ave_pd);
		if (ret)
			pm_genpd_remove(&s->ave_pd);
	}
	if (ret) {
		for (i = 0; i < s->ave_nchildren; i++)
			of_node_put(s->ave_children[i]);
		s->ave_nchildren = 0;
		s->ave_linked = 0;
		return ret;
	}
	s->ave_pd_owned = true;
	ret = devm_add_action_or_reset(s->dev, apple_sleep_ave_pd_remove, s);
	if (ret)
		return ret;

	/* -EPROBE_DEFER until the PMGR's and the PMC's drivers have their domains */
	for (i = 0; i < s->ave_nchildren; i++) {
		child.np = s->ave_children[i];
		ret = of_genpd_add_subdomain(&parent, &child);
		if (ret)
			return ret;
	}

	return 0;
}

static int apple_sleep_smc_scope_begin(void)
{
	struct apple_sleep *s = apple_sleep;
	int ret = 0;

	if ((!s->soc->early_context || !s->soc->smc_policy) &&
	    !apple_sleep_smc_defer_notice())
		return 0;
	mutex_lock(&apple_sleep_smc_lock);
	if (!s->smc.saved_ops || !s->smc.dev) {
		ret = -ENODEV;
		goto out;
	}
	/* Unregister runs with device_lock held. Never block on the inverse
	 * order: refusal is safe here, before CPUs/IRQs/parked cores go down.
	 */
	if (!device_trylock(s->smc.dev)) {
		ret = -EBUSY;
		goto out;
	}
	s->smc.held = true;
	mutex_unlock(&apple_sleep_smc_lock);
	ret = s->smc.saved_ops->begin(s->smc.data);
	if (ret) {
		s->smc.held = false;
		device_unlock(s->smc.dev);
	}
	return ret;
out:
	mutex_unlock(&apple_sleep_smc_lock);
	return ret;
}

static void apple_sleep_smc_scope_end(void)
{
	struct apple_sleep *s = apple_sleep;
	int ret;

	if (!s->smc.held)
		return;
	ret = s->smc.saved_ops->end(s->smc.data);
	if (ret)
		dev_err(s->smc.dev, "SMC state not restored after the sleep: %d\n", ret);
	else if (s->smc.saved_ops->woken)
		s->smc.saved_ops->woken(s->smc.data);
	s->smc.held = false;
	device_unlock(s->smc.dev);
}

static int apple_sleep_late_scope_begin(void)
{
	const char *step = "V27 display scope";
	int ret;

	/*
	 * J714s: every device's noirq suspend is done, the USB PHYs' stops with them. macOS opens
	 * its USB-C port controllers' USBw 0x09 window 0.5 ms after the port's ATC PHY CPU stops,
	 * before the SMC's slpa (traced on the J714s).
	 */
	blocking_notifier_call_chain(&sleep_usb_asleep_chain, READ_ONCE(sleep_usb_asleep), NULL);
	ret = sleep_v27_scope_begin();

	if (!ret) {
		step = "SMC scope";
		ret = apple_sleep_smc_scope_begin();
	}
	if (!ret) {
		step = "ANS reservation";
		ret = sleep_ans_late_reserve();
	}
	if (ret && apple_sleep->soc->regions)
		dev_info(apple_sleep->dev, "refused at .prepare_late: %s: %d\n", step, ret);
	if (ret) {
		apple_sleep_smc_scope_end();
		sleep_v27_scope_end();
	}
	return ret;
}

static void apple_sleep_late_scope_finish(void)
{
	/* .finish also runs when noirq suspend fails before platform .wake. */
	apple_sleep_smc_scope_end();
	sleep_v27_scope_end();
	/* After device early resume, which consumed the ANS receipt. */
	sleep_ans_late_release(false);
}

static int apple_sleep_prepare(void)
{
	struct apple_sleep *s = apple_sleep;
	static u8 attempts;
	int i;

	/* Every device .suspend has run: no owner registers after this. */
	sleep_ans_late_close();
	sleep_note_at(s, 0, PMU_NOTE_MAGIC);
	sleep_note_at(s, 2, 0);
	sleep_note_at(s, 3, ++attempts);
	sleep_note(s, 0x02);

	/* With an encoder's domain linked, genpd has it off once the encoders are. */
	if (s->soc->has_ave && !s->ave_pd_owned)
		apple_sleep_ave_off(s);

	for (i = 0; i < s->soc->ndisp; i++) {
		struct platform_device *pdev;

		s->display[i].bound = false;
		if (!s->display[i].np)
			continue;
		pdev = of_find_device_by_node(s->display[i].np);
		if (pdev) {
			s->display[i].bound = device_is_bound(&pdev->dev);
			put_device(&pdev->dev);
		}
	}
	apple_sleep_gate_prepare(s);

	return 0;
}

/* Only the SMC's sources woke the SoC. */
static bool t6050_smc_woke_alone(struct apple_sleep *s);

static bool apple_sleep_smc_woke_alone(struct apple_sleep *s)
{
	const struct apple_sleep_wake_name *names = s->soc->wake_names;
	u32 smc[WAKE_WORDS] = {};
	bool any = false;
	int i;

	if (s->soc->regions)
		return t6050_smc_woke_alone(s);

	/* SMC.CPUWakeupAP and SMC.OutboxNotEmpty, by their names. */
	for (i = 0; i < s->soc->nwake_names; i++) {
		u32 id = names[i].id;

		if (strstarts(names[i].name, "SMC.") && id / 32 < s->soc->wake_words)
			smc[id / 32] |= BIT(id % 32);
	}
	for (i = 0; i < s->soc->wake_words; i++) {
		u32 woke = s->wake_status[i] & s->wake_armed[i];

		if (woke & ~smc[i])
			return false;
		any |= !!woke;
	}

	return any;
}

static void apple_sleep_report(void)
{
	struct apple_sleep *s = apple_sleep;
	const struct apple_sleep_wake_name *names = s->soc->wake_names;
	int i;

	/* Back in Linux, before any device resumes: a reset after this is in the resume. */
	sleep_note(s, 0x7e);
	if (s->soc->ops->finish)
		s->soc->ops->finish(s);
	/* AP ON, then the SMC's wake record and waka, then the PMU's wake steps: macOS's order */
	apple_sleep_smc_scope_end();
	if (s->soc->ops->pmu_wake)
		s->soc->ops->pmu_wake(s);
	sleep_v27_scope_end();
	s->smc_woke_alone = false;
	if (!s->returned)
		return;
	s->returned = false;
	s->smc_woke_alone = apple_sleep_smc_woke_alone(s);

	dev_info(s->dev, "back from sleep, boot object resumes %llu -> %llu\n",
		 s->resumes_before, s->resumes_after);
	for (i = 0; i < s->ps_unlatched_count; i++)
		dev_warn(s->dev, "power word %#x was off with BUSY latched at the commit: %#x\n",
			 s->ps_unlatched[i].offset, s->ps_unlatched[i].val);
	if (s->deep_once)
		dev_info(s->dev, "took the deep path: a power word stayed latched or stuck\n");
	if (s->deep_ran)
		dev_info(s->dev, "the deep path ran\n");
	for (i = 0; i < s->soc->nwake_names; i++) {
		u32 id = names[i].id;

		if (s->wake_status[id / 32] & BIT(id % 32))
			dev_info(s->dev, "wake source: %s\n", names[i].name);
	}
	if (!s->soc->regions) {
		sleep_ps_rewake_report(s);
		sleep_spmi_sources_report(s);
		sleep_ps_wr_dis_report(s);
	}

	for (i = 0; i < SLEEP_COPROCS; i++) {
		struct apple_sleep_coproc *c = &apple_sleep_coprocs[i];

		if (c->dev && c->stopped && c->stop_ret)
			dev_err(s->dev, "%s stop reported %d; physical sleep completed\n",
				dev_name(c->dev), c->stop_ret);
	}
	for (i = 0; i < s->soc->quiesce.count; i++)
		if (s->quiesce_still_on & BIT(i))
			dev_err(s->dev, "power word %#x was on at the last step\n",
				s->soc->quiesce.offset[i]);
	if (s->cpus_stop_ret)
		dev_err(s->dev, "other CPUs did not stop: %d\n", s->cpus_stop_ret);
	if (s->perf_soc_ret || s->perf_dcs_ret)
		dev_warn(s->dev, "sleep perf states: SoC %d, DCS %d (DCS was %u)\n", s->perf_soc_ret,
			 s->perf_dcs_ret, s->perf_dcs_from);
	if (s->cluster_p_ret)
		dev_err(s->dev, "P complex did not come back on: %d\n", s->cluster_p_ret);
	if (s->radio_trainable && s->radio_stop_ret)
		dev_warn(s->dev, "the radio's port stop at the commit: first timeout at %s: %d\n",
			 s->radio_stop_step ?: "?", s->radio_stop_ret);
	if (s->radio_trainable && s->radio_train_ret)
		dev_err(s->dev, "the radio's PCIe link did not train after the wake: %d\n",
			s->radio_train_ret);
	else if (s->radio_trainable)
		dev_info(s->dev, "the radio's PCIe link trained again after the wake\n");
	if (s->cluster_e_ret || s->cluster_ave_ret || s->cluster_ave_commit_ret)
		dev_err(s->dev, "cluster power timed out: E %d AVE %d, AVE at the commit %d\n",
			s->cluster_e_ret, s->cluster_ave_ret, s->cluster_ave_commit_ret);
	if (s->ans_off_ret)
		dev_err(s->dev, "ANS did not read off at the commit: %d\n", s->ans_off_ret);
	if (s->usb_down_ret)
		dev_err(s->dev, "debug link: the stop failed: %d\n", s->usb_down_ret);
	if (s->pcie_ans_stopped && s->pcie_ans_on_ret)
		dev_err(s->dev, "ANS's PCIe block did not come up after the wake: %d\n",
			s->pcie_ans_on_ret);
	if (s->apcie_port_clk_ret[0])
		dev_err(s->dev, "PCIe port 0's PHY clocks did not ack after the wake: %d\n",
			s->apcie_port_clk_ret[0]);
	if (s->apcie_port_clk_ret[1])
		dev_err(s->dev, "PCIe port 2's PHY clocks did not ack after the wake: %d\n",
			s->apcie_port_clk_ret[1]);
}

static int apple_sleep_valid(suspend_state_t state)
{
	if (state != PM_SUSPEND_MEM)
		return 0;
	return !apple_sleep || !apple_sleep->soc->untested || READ_ONCE(sleep_deep);
}

static const struct platform_suspend_ops apple_sleep_ops = {
	.valid = apple_sleep_valid,
	.begin = apple_sleep_begin,
	.prepare = apple_sleep_prepare,
	.prepare_late = apple_sleep_smc_scope_begin,
	.finish = apple_sleep_smc_scope_end,
	.enter = apple_sleep_enter,
	.wake = apple_sleep_report,
	.end = apple_sleep_end,
};

/* For a SoC with early_context: the same, with the scopes its entry needs held */
static const struct platform_suspend_ops apple_sleep_early_ops = {
	.valid = suspend_valid_only_mem,
	.begin = apple_sleep_begin,
	.prepare = apple_sleep_prepare,
	.prepare_late = apple_sleep_late_scope_begin,
	.finish = apple_sleep_late_scope_finish,
	.enter = apple_sleep_enter,
	.wake = apple_sleep_report,
	.end = apple_sleep_end,
};

static int apple_sleep_read_wake_sources(struct apple_sleep *s)
{
	struct device_node *np = s->dev->of_node;
	int count;

	count = of_property_read_variable_u32_array(np, "apple,wake-enable", s->wake_enable,
						    1, WAKE_SOURCES_MAX);
	if (count < 0) {
		if (!s->soc->wake_optional)
			return dev_err_probe(s->dev, count, "no wake source enabled\n");
		dev_warn(s->dev, "no wake source in the device tree: a sleep is refused\n");
		return 0;
	}
	s->wake_enable_count = count;

	return 0;
}

static int apple_sleep_map_displays(struct apple_sleep *s)
{
	int i;

	for (i = 0; i < s->soc->ndisp; i++) {
		struct device_node *np;
		int index;

		np = of_parse_phandle(s->dev->of_node, "apple,display-coprocessors", i);
		if (!np && i != 0 && s->soc->dispext_optional) {
			dev_info(s->dev, "no external display coprocessor %d: not lowered\n", i);
			continue;
		}
		if (!np)
			return dev_err_probe(s->dev, -ENODEV, "no display coprocessor %d\n", i);
		s->display[i].np = np;

		index = of_property_match_string(np, "reg-names", "coproc");
		if (index < 0)
			return dev_err_probe(s->dev, index, "%pOF has no coproc registers\n", np);
		s->display[i].asc = of_iomap(np, index);
		if (!s->display[i].asc)
			return dev_err_probe(s->dev, -ENOMEM, "cannot map %pOF\n", np);
	}

	return 0;
}

static void __iomem *apple_sleep_map(struct device *dev, const char *name)
{
	struct device_node *np = dev->of_node;
	struct resource res;
	void __iomem *base;
	int index;

	index = of_property_match_string(np, "reg-names", name);
	if (index < 0)
		return IOMEM_ERR_PTR(index);
	if (of_address_to_resource(np, index, &res))
		return IOMEM_ERR_PTR(-EINVAL);

	/* Shared with the drivers of these blocks, so not claimed. */
	base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!base)
		return IOMEM_ERR_PTR(-ENOMEM);

	return base;
}

/* The register blocks every sleep writes, by their reg-names. */
static const struct {
	const char *name;
	size_t offset;
} apple_sleep_regs[] = {
	{ "ps", offsetof(struct apple_sleep, ps) },
	{ "pwrgate", offsetof(struct apple_sleep, pwrgate) },
	{ "cpu-start", offsetof(struct apple_sleep, cpu_start) },
	{ "cllt", offsetof(struct apple_sleep, cllt) },
	{ "cluster", offsetof(struct apple_sleep, cluster) },
	{ "commit", offsetof(struct apple_sleep, commit) },
	{ "wdt", offsetof(struct apple_sleep, wdt) },
	{ "aon-misc", offsetof(struct apple_sleep, aon_misc) },
	{ "wake", offsetof(struct apple_sleep, wake) },
	{ "pmc-dcp", offsetof(struct apple_sleep, pmc_dcp) },
	{ "spmi", offsetof(struct apple_sleep, spmi) },
	{ "pmc-voters", offsetof(struct apple_sleep, pmc_voters) },
	{ "dcs", offsetof(struct apple_sleep, dcs) },
	{ "ps-aon", offsetof(struct apple_sleep, ps_aon) },
	{ "spmi-a0", offsetof(struct apple_sleep, spmi_a0) },
	{ "clk-mux", offsetof(struct apple_sleep, clk_mux) },
	{ "ptd", offsetof(struct apple_sleep, ptd) },
	{ "clk-perf", offsetof(struct apple_sleep, clk_perf) },
	{ "cluster-dcs", offsetof(struct apple_sleep, cluster_dcs) },
	{ "dvfm-e", offsetof(struct apple_sleep, dvfm[0]) },
	{ "dvfm-p", offsetof(struct apple_sleep, dvfm[1]) },
};

/* "<prefix>0", "<prefix>1", ...: @count of them */
static int apple_sleep_map_numbered(struct apple_sleep *s, const char *prefix,
				    void __iomem **regs, unsigned int count)
{
	char name[16];
	int i;

	for (i = 0; i < count; i++) {
		snprintf(name, sizeof(name), "%s%d", prefix, i);
		regs[i] = apple_sleep_map(s->dev, name);
		if (IS_ERR(regs[i]))
			return dev_err_probe(s->dev, PTR_ERR(regs[i]), "no %s registers\n", name);
	}

	return 0;
}

static void apple_sleep_put_device(void *dev)
{
	put_device(dev);
}

/*
 * Gen 4: the controller at this node's "spmi" block (nub-spmi0), as the T6050 profile finds
 * it, and the PMU's sources in its status words: apple,spmi-pmu-sources = <word bits>, the
 * word's offset in 0x600..0x61c. Neither is required.
 */
static void sleep_map_pmu_bus(struct apple_sleep *s)
{
	struct resource *own = platform_get_resource_byname(to_platform_device(s->dev),
							    IORESOURCE_MEM, "spmi");
	struct device_node *np;
	struct resource res;
	u32 v[2];

	for_each_compatible_node(np, NULL, "apple,t8142-spmi") {
		struct platform_device *bus;

		if (!own || of_address_to_resource(np, 0, &res) || res.start != own->start)
			continue;
		bus = of_find_device_by_node(np);
		if (bus && !devm_add_action_or_reset(s->dev, apple_sleep_put_device, &bus->dev))
			s->pmu_bus = &bus->dev;
		of_node_put(np);
		break;
	}
	if (!of_property_read_u32_array(s->dev->of_node, "apple,spmi-pmu-sources", v, 2) &&
	    v[1] && v[0] >= SPMI4_SRC_STATUS && v[0] < SPMI4_NOTIFY_ACK && !(v[0] % 4)) {
		s->spmi_pmu_status = v[0];
		s->spmi_pmu_bits = v[1];
	}
	dev_info(s->dev, "PMU's SPMI controller %s, latched sources %s\n",
		 s->pmu_bus ? dev_name(s->pmu_bus) : "not found (writes unlocked)",
		 s->spmi_pmu_bits ? "acknowledged at entry and wake" : "only reported");
}

static int apple_sleep_map_regs(struct apple_sleep *s)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(apple_sleep_regs); i++) {
		void __iomem **field = (void __iomem **)((u8 *)s + apple_sleep_regs[i].offset);

		if (s->soc->regs_absent &&
		    match_string(s->soc->regs_absent, -1, apple_sleep_regs[i].name) >= 0)
			continue;
		*field = apple_sleep_map(s->dev, apple_sleep_regs[i].name);
		if (IS_ERR(*field))
			return dev_err_probe(s->dev, PTR_ERR(*field), "no %s registers\n",
					     apple_sleep_regs[i].name);
	}

	if (s->soc->spmi_gen4 && !s->soc->regions)
		sleep_map_pmu_bus(s);

	/* T8142: each complex's ACC word, used only with apple_sleep.acc_wake */
	s->acc_e = apple_sleep_map(s->dev, "acc-e");
	s->acc_p = apple_sleep_map(s->dev, "acc-p");
	if (IS_ERR(s->acc_e) || IS_ERR(s->acc_p))
		s->acc_e = s->acc_p = NULL;

	ret = apple_sleep_map_numbered(s, "mcc", s->mcc, s->soc->nmcc);
	if (ret)
		return ret;

	ret = apple_sleep_map_numbered(s, "amcc", s->amcc, s->soc->namcc);
	if (ret)
		return ret;

	ret = apple_sleep_map_numbered(s, "socni", s->socni, s->soc->nsocni);
	if (ret)
		return ret;

	s->mem_clk = apple_sleep_map(s->dev, "mem-clk");
	if (IS_ERR(s->mem_clk))
		s->mem_clk = NULL;

	/* T8142's memory-controller start, mapped page by page (shared with nothing that binds) */
	if (s->soc->mem_boot)
		t8142_mem_map(s);

	/* T8142's wake restores (optional: a tree without them skips that restore). */
	s->clk_slots = apple_sleep_map(s->dev, "clk-slots");
	if (IS_ERR(s->clk_slots))
		s->clk_slots = NULL;
	for (i = 0; i < s->soc->namcc; i++) {
		char name[16];

		snprintf(name, sizeof(name), "amcc-cfg%d", i);
		s->amcc_cfg[i] = apple_sleep_map(s->dev, name);
		if (IS_ERR(s->amcc_cfg[i]))
			s->amcc_cfg[i] = NULL;
	}

	/* Lowering an unbound display clears its clock protection first. */
	if (s->soc->lower_unbound_displays) {
		s->ps_prot = apple_sleep_map(s->dev, "ps-prot");
		if (IS_ERR(s->ps_prot))
			return dev_err_probe(s->dev, PTR_ERR(s->ps_prot), "no ps-prot registers\n");
	}

	return 0;
}

static int apple_sleep_map_mailbox(struct apple_sleep *s)
{
	struct device_node *chosen;
	u64 pa;
	int ret;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return -ENODEV;
	ret = of_property_read_u64(chosen, "aurora,s2r-mailbox", &pa);
	of_node_put(chosen);
	if (ret) {
		dev_info(s->dev, "no aurora,s2r-mailbox from the boot object: no deep sleep\n");
		return -ENODEV;
	}

	s->mbox = devm_memremap(s->dev, pa, sizeof(*s->mbox), MEMREMAP_WB);
	if (IS_ERR(s->mbox))
		return PTR_ERR(s->mbox);

	/* aura wrote it with the MMU off, so nothing of it can be in a cache. */
	if (s->mbox->magic != AURA_S2R_MAGIC)
		return dev_err_probe(s->dev, -EINVAL, "bad mailbox magic %llx\n",
				     s->mbox->magic);
	/* Probe can run on any CPU. Logical CPU 0 is always the boot CPU. */
	if (s->mbox->boot_mpidr != cpu_logical_map(0))
		return dev_err_probe(s->dev, -EINVAL,
				     "mailbox is for CPU %llx, not this boot CPU\n",
				     s->mbox->boot_mpidr);

	return 0;
}

static struct apple_tunable *apple_sleep_apcie_tunable(struct apple_sleep *s,
						       struct device_node *np,
						       const char *name, const char *prop)
{
	int idx = of_property_match_string(np, "reg-names", name);
	struct apple_tunable *t;
	struct resource res;

	if (idx < 0 || of_address_to_resource(np, idx, &res))
		return NULL;
	t = devm_apple_tunable_parse(s->dev, np, prop, &res);

	return IS_ERR(t) ? NULL : t;
}

static void __iomem *apple_sleep_apcie_block(struct apple_sleep *s, struct device_node *np,
					     const char *name, const char *prop,
					     struct apple_tunable **tunable)
{
	int idx = of_property_match_string(np, "reg-names", name);

	if (idx < 0)
		return NULL;
	if (prop) {
		*tunable = apple_sleep_apcie_tunable(s, np, name, prop);
		if (!*tunable)
			return NULL;
	}

	return of_iomap(np, idx);
}

/*
 * Its registers go with PCIe's node, which a test tree may have disabled, and a loader
 * publishes the tunables there. Missing ones only refuse a real sleep with ANS stopped.
 */
static void apple_sleep_map_apcie(struct apple_sleep *s)
{
	struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,apcie", 0);
	struct device_node *engine;

	if (!np)
		return;
	s->apcie_axi = apple_sleep_apcie_block(s, np, "axi2af", "apple,tunable-axi2af",
					       &s->apcie_axi_tunable);
	s->apcie_cio3pll = apple_sleep_apcie_block(s, np, "cio3pll", "apple,tunable-cio3pll",
						   &s->apcie_cio3pll_tunable);
	s->apcie_clkgen = apple_sleep_apcie_block(s, np, "clkgen", "apple,tunable-clkgen",
						  &s->apcie_clkgen_tunable);
	s->apcie_phy = apple_sleep_apcie_block(s, np, "phy-block", "apple,tunable-phy",
					       &s->apcie_phy_tunable);
	s->apcie_rc = apple_sleep_apcie_block(s, np, "rc", NULL, NULL);
	s->apcie_phy_ip = apple_sleep_apcie_block(s, np, "phy-ip", "apple,tunable-phy-ip-pll",
						  &s->apcie_phy_ip_pll);
	s->apcie_phy_ip_lanes = apple_sleep_apcie_tunable(s, np, "phy-ip",
							  "apple,tunable-phy-ip-auspma");
	/*
	 * T8142's block has neither table: those steps are skipped. Its clock generator
	 * (pcieclkgen) is only written at the power-down, with no table.
	 */
	if (s->soc->apcie_gen4) {
		s->apcie_cio3pll = NULL;
		s->apcie_cio3pll_tunable = s->apcie_clkgen_tunable = NULL;
		s->apcie_clkgen = apple_sleep_apcie_block(s, np, "clkgen", NULL, NULL);
	}
	if (!s->apcie_axi || !s->apcie_phy || !s->apcie_rc || !s->apcie_phy_ip ||
	    !s->apcie_phy_ip_lanes ||
	    (!s->soc->apcie_gen4 && (!s->apcie_cio3pll || !s->apcie_clkgen))) {
		dev_info(s->dev,
			 "no PCIe tunables for ANS: suspend-to-RAM with it stopped will be refused\n");
		s->apcie_phy_tunable = NULL;
		s->apcie_phy = NULL;
		s->apcie_clkgen = NULL;
		s->apcie_axi = NULL;
	}

	s->apcie_port = apple_sleep_apcie_block(s, np, "port0", NULL, NULL);
	s->apcie_port_phy = apple_sleep_apcie_block(s, np, "phy0", NULL, NULL);
	s->apcie_port2 = apple_sleep_apcie_block(s, np, "port2", NULL, NULL);
	s->apcie_port2_phy = apple_sleep_apcie_block(s, np, "phy2", NULL, NULL);
	s->apcie_ports_unowned = !of_property_present(np, "apple,managed-sids");
	engine = of_parse_phandle(np, "apple,piodma", 0);
	if (engine) {
		s->apcie_engine = apple_sleep_apcie_block(s, engine, "engine", NULL, NULL);
		s->apcie_fabric = apple_sleep_apcie_block(s, engine, "fabric", NULL, NULL);
		of_node_put(engine);
	}
	of_node_put(np);
}

/* The radio port's PERST# on the AP pins @np, from the ports of the SoC's enabled PCIe host. */
static void apple_sleep_find_radio_perst(struct apple_sleep *s, struct device_node *np)
{
	struct device_node *host, *port;
	struct of_phandle_args args;

	host = of_parse_phandle(s->dev->of_node, "apple,apcie", 0);
	if (!host)
		return;
	if (of_device_is_available(host)) {
		for_each_available_child_of_node(host, port) {
			if (of_parse_phandle_with_args(port, "reset-gpios", "#gpio-cells", 0,
						       &args))
				continue;
			if (args.np == np && args.args_count >= 2 && args.args[0] < s->npins &&
			    !test_bit(args.args[0], s->pins_skip)) {
				s->radio_perst = args.args[0];
				s->radio_perst_low = args.args[1] & GPIO_ACTIVE_LOW;
			}
			of_node_put(args.np);
		}
	}
	of_node_put(host);
}

/* macOS leaves the skipped pins alone, and reading one may fault. */
static void apple_sleep_map_pins(struct apple_sleep *s)
{
	struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,pinctrl", 0);
	u32 npins, pin;
	int i, n;

	s->radio_perst = -1;
	if (!np)
		return;
	if (!of_device_is_available(np) || of_property_read_u32(np, "apple,npins", &npins) ||
	    !npins || npins > 512)
		goto out;
	/* Without the pins to leave alone none is saved: reading some of them faults. */
	if (s->soc->regions && !of_property_present(s->dev->of_node, "apple,pinctrl-skip-pins"))
		goto out;
	s->pins_saved = devm_kcalloc(s->dev, npins, sizeof(*s->pins_saved), GFP_KERNEL);
	s->pins_skip = devm_bitmap_zalloc(s->dev, npins, GFP_KERNEL);
	if (!s->pins_saved || !s->pins_skip)
		goto out;
	n = of_property_count_u32_elems(s->dev->of_node, "apple,pinctrl-skip-pins");
	for (i = 0; i < n; i++)
		if (!of_property_read_u32_index(s->dev->of_node, "apple,pinctrl-skip-pins", i,
						&pin) && pin < npins)
			set_bit(pin, s->pins_skip);
	s->pin_irq_groups = of_irq_count(np);
	s->npins = npins;
	s->pins = of_iomap(np, 0);
	if (s->pins)
		apple_sleep_find_radio_perst(s, np);
out:
	of_node_put(np);
}

/*
 * T8142's radio port: its blocks with the shared block's ("apple,apcie": radio-port,
 * radio-lane and radio-cfg, the root port's configuration space), the loader's tables for
 * them, and its PERST# ("apple,radio-perst", on the AP pins). All of it, or the port isn't
 * trained again and a real sleep is refused while its host is held.
 */
static void apple_sleep_map_radio(struct apple_sleep *s)
{
	static const char *const rc[] = {
		"apple,tunable-radio-rc", "apple,tunable-radio-rc-gen3", "apple,tunable-radio-rc-gen4",
	};
	struct device_node *np, *pins;
	struct of_phandle_args args;
	unsigned int i;

	if (!s->soc->radio_retrain || !s->radio_host_np || !s->pins)
		return;
	np = of_parse_phandle(s->dev->of_node, "apple,apcie", 0);
	if (!np)
		return;
	s->radio_port = apple_sleep_apcie_block(s, np, "radio-port", "apple,tunable-radio-port",
						&s->radio_port_tunable);
	s->radio_lane = apple_sleep_apcie_block(s, np, "radio-lane", NULL, NULL);
	s->radio_cfg = apple_sleep_apcie_block(s, np, "radio-cfg", NULL, NULL);
	for (i = 0; i < ARRAY_SIZE(rc); i++)
		s->radio_rc_tunable[i] = apple_sleep_apcie_tunable(s, np, "radio-cfg", rc[i]);
	of_node_put(np);

	pins = of_parse_phandle(s->dev->of_node, "apple,pinctrl", 0);
	if (!of_parse_phandle_with_args(s->dev->of_node, "apple,radio-perst", "#gpio-cells", 0,
					&args)) {
		if (args.np == pins && args.args_count >= 2 && args.args[0] < s->npins &&
		    !test_bit(args.args[0], s->pins_skip)) {
			s->radio_perst = args.args[0];
			s->radio_perst_low = args.args[1] & GPIO_ACTIVE_LOW;
		}
		of_node_put(args.np);
	}
	of_node_put(pins);

	s->radio_trainable = s->radio_port && s->radio_lane && s->radio_cfg &&
			     s->radio_port_tunable && s->radio_rc_tunable[0] &&
			     s->radio_rc_tunable[1] && s->radio_rc_tunable[2] && s->radio_perst >= 0;
	if (!s->radio_trainable)
		dev_info(s->dev, "the radio's port can't be trained after a sleep: a real sleep is refused while its host is bound\n");
}

static void apple_sleep_map_debug_port(struct apple_sleep *s)
{
	struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,debug-port", 0);
	struct device_node *bus;

	if (!np)
		return;
	/* Its commands are written for the gen 1 layout only. */
	if (s->soc->spmi_gen4) {
		dev_warn(s->dev, "apple,debug-port on SPMI gen 4: not driven\n");
		of_node_put(np);
		return;
	}
	bus = of_get_parent(np);
	if (bus && !of_property_read_u32_index(np, "reg", 0, &s->hpm_sid))
		s->hpm_spmi = of_iomap(bus, 0);
	of_node_put(bus);
	/* Kept, for as long as the driver: no unbind. */
	s->hpm_np = np;
}

/* The books kept for each word of the SoC's lists */
static int apple_sleep_alloc_books(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	unsigned int n = soc->ps_all.count, naon = soc->ps_aon_all.count;
	struct device *dev = s->dev;

	/* One bit, and one gate, for each */
	if (soc->nstray > BITS_PER_LONG || soc->ngate_coprocs > SLEEP_GATES_MAX ||
	    soc->nps_wr_dis > SLEEP_PS_WR_DIS_MAX)
		return dev_err_probe(dev, -EINVAL, "the SoC's lists are too long\n");

	s->ps_on_saved = devm_kcalloc(dev, soc->ps_macos_on.count, sizeof(u32), GFP_KERNEL);
	s->ps_off_saved = devm_kcalloc(dev, soc->ps_macos_off.count, sizeof(u32), GFP_KERNEL);
	s->ps_aon_on_saved = devm_kcalloc(dev, soc->ps_aon_macos_on.count, sizeof(u32), GFP_KERNEL);
	s->ps_aon_saved = devm_kcalloc(dev, soc->ps_aon_macos_off.count, sizeof(u32),
				       GFP_KERNEL);
	s->ps_aon_auto_saved = devm_kcalloc(dev, soc->ps_aon_macos_no_auto.count, sizeof(u32),
					    GFP_KERNEL);
	s->ps_commit = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	s->ps_woke = devm_kcalloc(dev, n, sizeof(u32), GFP_KERNEL);
	s->ps_late = devm_kcalloc(dev, n, sizeof(bool), GFP_KERNEL);
	s->ps_done = devm_kcalloc(dev, n, sizeof(bool), GFP_KERNEL);
	s->ps_aon_commit = devm_kcalloc(dev, naon, sizeof(u32), GFP_KERNEL);
	s->ps_aon_woke = devm_kcalloc(dev, naon, sizeof(u32), GFP_KERNEL);
	s->ps_aon_late = devm_kcalloc(dev, naon, sizeof(bool), GFP_KERNEL);
	s->ps_aon_done = devm_kcalloc(dev, naon, sizeof(bool), GFP_KERNEL);
	if (!s->ps_on_saved || !s->ps_off_saved || !s->ps_aon_saved || !s->ps_aon_on_saved ||
	    !s->ps_aon_auto_saved || !s->ps_commit || !s->ps_woke || !s->ps_late || !s->ps_done ||
	    !s->ps_aon_commit ||
	    !s->ps_aon_woke || !s->ps_aon_late || !s->ps_aon_done)
		return -ENOMEM;

	return 0;
}

static void apple_sleep_put_node(void *np)
{
	of_node_put(np);
}

#include "sleep-t6050.h"

/* ANS's sleep owner on T6050: registered at its .suspend, its final stop in the last steps. */
int apple_sleep_ans_m5_register(struct device *dev, void __iomem *asc,
	int (*check)(void *), int (*stop)(void *), void *data)
{
	struct apple_sleep *s = apple_sleep;
	bool late = sleep_ans_late_selected();
	int ret;

	if (!late || !s || !apple_sleep_t6050_full(s))
		return -EOPNOTSUPP;
	if (!check || !stop || !asc ||
	    !of_device_is_compatible(dev->of_node, "apple,t6050-nvme-ans2"))
		return -EINVAL;
	if (late) {
		guard(mutex)(&apple_sleep_coprocs_lock);
		/* Only from an open attempt's own .suspend, never cold probe. */
		if (!sleep_ans_late.open || sleep_ans_late.leaving || sleep_ans_late.dev)
			return -EBUSY;
		sleep_ans_late.dev = dev;
	}
	ret = apple_sleep_coproc_register(dev, asc, true, stop, data);
	if (!ret) {
		ret = apple_sleep_coproc_set_last_check(dev, check);
		if (ret)
			apple_sleep_coproc_unregister(dev);
	}
	if (ret && late) {
		guard(mutex)(&apple_sleep_coprocs_lock);
		if (sleep_ans_late.dev == dev)
			sleep_ans_late.dev = NULL;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_ans_m5_register);

int apple_sleep_ans_m5_preflight(struct device *dev)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_coproc *c;
	unsigned int i;

	/* Called noirq: PM owns registration/lifetimes; acquire no sleeping lock. */
	if (!sleep_ans_late_selected() || !s || !apple_sleep_t6050_full(s) || !s->region ||
	    !s->region->snapshot_valid)
		return -EOPNOTSUPP;
	/* The entry boundary: the attempt's own task, its reserved owner, late scope armed. */
	if (sleep_ans_late_selected() &&
	    (READ_ONCE(sleep_ans_late.task) != current || !READ_ONCE(sleep_ans_late.reserved) ||
	     READ_ONCE(sleep_ans_late.dev) != dev || !s->smc.held))
		return -EBUSY;
	c = apple_sleep_coproc_find(dev);
	if (!c || !c->pcie_ans || !c->ready || c->stopped)
		return -EBUSY;
	for (i = 0; i < ARRAY_SIZE(t6050_ans_storage_ps); i++) {
		u32 word = SLEEP_REGION_WORD(T6050_PS1, t6050_ans_storage_ps[i]);
		int n = t6050_ans_storage_index(s, word);
		u32 value;

		if (n < 0 || test_bit(n, s->region->driver_down) ||
		    test_bit(n, s->region->manager_down))
			return -EBUSY;
		value = readl(sleep_region_reg(s, word));
		/*
		 * On, or under auto power at any actual: the NVMe node has no power domains on
		 * T6050, so the idle storage PHY gates itself, as on macOS until its late lowering.
		 */
		if ((value & (PS_TARGET | PS_ACTUAL)) == (PS_TARGET | PS_ACTUAL))
			continue;
		if (FIELD_GET(PS_TARGET, value) == 0xf && !(value & PS_DEV_DISABLE) &&
		    (value & PS_AUTO_ENABLE))
			continue;
		dev_info(s->dev, "ANS storage word %#x not on: %#x\n", t6050_ans_storage_ps[i], value);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_ans_m5_preflight);

int apple_sleep_ans_m5_domains_ready(struct device *dev)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_coproc *c;

	if (!sleep_ans_late_selected() || !s || !apple_sleep_t6050_full(s) ||
	    !pm_resume_via_firmware() ||
	    s->resumes_after <= s->resumes_before)
		return -EOPNOTSUPP;
	/* The reserved owner's receipt, valid until .finish releases it. */
	if (sleep_ans_late_selected() &&
	    (!READ_ONCE(sleep_ans_late.reserved) || READ_ONCE(sleep_ans_late.dev) != dev))
		return -EIO;
	c = apple_sleep_coproc_find(dev);
	if (!c || !c->pcie_ans || !c->stopped || !s->pcie_ans_stopped)
		return -EIO;
	return s->ans_m5_wake_error;
}
EXPORT_SYMBOL_GPL(apple_sleep_ans_m5_domains_ready);


/* The display sleep's read-only power-provider handoff. The region
 * manager alone restores saved SYS/FE/CPU and DISP gate values. No consumer
 * borrows a raw PMGR base or calls T8140's standalone reset on T6050.
 */
int apple_sleep_display_v27_power_ready(struct device *dev)
{
	struct apple_sleep *s = apple_sleep;
	unsigned int d, j;

	if (!s || !s->soc->regions || !s->region ||
	    !apple_sleep_t6050_full(s) || s->soc->ndisp != 5)
		return -EOPNOTSUPP;
	if (s->region->late_errors || s->region->display_lowered)
		return -EIO;
	for (d = 0; d < s->soc->ndisp; d++) {
		if (s->display[d].np != dev->of_node)
			continue;
		if (!s->display[d].asc || !device_is_bound(dev))
			return -ENODEV;
		for (j = 0; j < 3; j++) {
			u32 value = readl(sleep_region_reg(s, s->soc->ps.disp[d][j]));

			if (!t6050_display_word_ready(d, value)) {
				dev_info(s->dev, "display %u: power word %u reads %#x\n", d, j, value);
				return -EAGAIN;
			}
		}
		return 0;
	}
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_sleep_display_v27_power_ready);

/*
 * T6050 (firmware 27): the internal display's three power words (this profile's ps.disp[0]: CPU,
 * FE, SYS) and its coprocessor core's gate pair, lowered or raised around one stop and start of
 * the display coprocessor after a wake, outside a system sleep, in this order and with these
 * values. No PMC word is written here: the display driver does the PMC requests around this. The
 * display pipe DART's Linux-owned streams are remembered before the power-off and written back
 * after the power-on. The sleep's own entry and exit don't call this.
 */
static void __iomem *t6050_disp_dcp_gate;

static int t6050_disp_ps_settle(void __iomem *word, u32 mask, u32 want)
{
	u32 v;

	return readl_poll_timeout_atomic(word, v, (v & mask) == want, 1, 2000);
}

int apple_sleep_m5_display_power(bool on)
{
	struct apple_sleep *s = apple_sleep;
	void __iomem *cpu, *fe, *sys;
	int ret = 0;

	if (!s || !s->soc->regions || !s->region || !apple_sleep_t6050_full(s))
		return -ENODEV;
	/* The coprocessor core's gate pair: 3, 0 off and 0, 1 on. */
	if (!t6050_disp_dcp_gate)
		t6050_disp_dcp_gate = ioremap(0x280920500ULL, 0x100);
	if (!t6050_disp_dcp_gate)
		return -ENOMEM;
	cpu = sleep_region_reg(s, s->soc->ps.disp[0][0]);
	fe = sleep_region_reg(s, s->soc->ps.disp[0][1]);
	sys = sleep_region_reg(s, s->soc->ps.disp[0][2]);
	if (on) {
		writel(0xf, sys);
		ret |= t6050_disp_ps_settle(sys, 0xf0, 0xf0);
		writel(0xf, fe);
		ret |= t6050_disp_ps_settle(fe, 0xf0, 0xf0);
		writel(0x0f00000f, cpu);
		ret |= t6050_disp_ps_settle(cpu, 0xf0, 0xf0);
		writel(0, t6050_disp_dcp_gate + 0x1c);
		writel(1, t6050_disp_dcp_gate + 0x40);
		writel(0x1f00000f, cpu);
		apple_dart_m5_display_rewrite(NULL);
		dev_dbg(s->dev, "display power on: %d, CPU %#x FE %#x SYS %#x gate %#x %#x\n", ret,
			readl(cpu), readl(fe), readl(sys), readl(t6050_disp_dcp_gate + 0x1c),
			readl(t6050_disp_dcp_gate + 0x40));
		return ret;
	}
	apple_dart_m5_display_snapshot(NULL);
	writel(0x0f0000ff, cpu);
	ret |= t6050_disp_ps_settle(cpu, 0xf0, 0xf0);
	writel(3, t6050_disp_dcp_gate + 0x1c);
	writel(0, t6050_disp_dcp_gate + 0x40);
	writel(0x0f0003f0, cpu);
	ret |= t6050_disp_ps_settle(cpu, 0xf0, 0);
	writel(0x3f0, fe);
	ret |= t6050_disp_ps_settle(fe, 0xf0, 0);
	writel(0x3f0, sys);
	ret |= t6050_disp_ps_settle(sys, 0xf0, 0);
	dev_dbg(s->dev, "display power off: %d, CPU %#x FE %#x SYS %#x gate %#x %#x\n", ret,
		readl(cpu), readl(fe), readl(sys), readl(t6050_disp_dcp_gate + 0x1c),
		readl(t6050_disp_dcp_gate + 0x40));
	return ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_m5_display_power);

/*
 * T6050 (firmware 27): one external display coprocessor's power words (this profile's
 * ps.disp[1..4]: CPU, FE, SYS) and its bits in the PMC's boost and bandwidth enable words,
 * lowered or raised around one stop and start of that coprocessor after a wake, with the values
 * and in the order of macOS's own stop and start of it (the same for all four and for each of
 * its stops): off, the two PMC bits cleared, then CPU, FE, SYS; on, SYS, FE, CPU, then the two
 * bits set. The PMC's request word is not written: macOS writes it only when the first of the
 * four comes on or the last goes off, and here the others stay on. @dev is the coprocessor. The
 * sleep's own entry and exit don't call this.
 */
static const struct {
	u8 boost, bwr;
} t6050_disp_ext_pmc[] = {
	{ 2, 3 },	/* DISPEXT0 */
	{ 4, 4 },	/* DISPEXT1 */
	{ 6, 5 },	/* DISPEXT2 */
	{ 8, 6 },	/* DISPEXT3 */
};
#define T6050_DISP_PMC_BWR	0x20
#define T6050_DISP_PMC_BOOST	0x28
static void __iomem *t6050_disp_pmc_en;

int apple_sleep_m5_dispext_power(struct device *dev, bool on)
{
	struct apple_sleep *s = apple_sleep;
	void __iomem *cpu, *fe, *sys, *pmc;
	unsigned int d;
	u32 boost, bwr;
	int ret = 0;

	if (!dev || !s || !s->soc->regions || !s->region || !apple_sleep_t6050_full(s) ||
	    s->soc->ndisp != ARRAY_SIZE(t6050_disp_ext_pmc) + 1)
		return -ENODEV;
	for (d = 1; d < s->soc->ndisp; d++)
		if (s->display[d].np == dev->of_node)
			break;
	if (d == s->soc->ndisp)
		return -ENODEV;
	if (!t6050_disp_pmc_en)
		t6050_disp_pmc_en = ioremap(0x286808000ULL, 0x30);
	pmc = t6050_disp_pmc_en;
	if (!pmc)
		return -ENOMEM;
	boost = BIT(t6050_disp_ext_pmc[d - 1].boost);
	bwr = BIT(t6050_disp_ext_pmc[d - 1].bwr);
	cpu = sleep_region_reg(s, s->soc->ps.disp[d][0]);
	fe = sleep_region_reg(s, s->soc->ps.disp[d][1]);
	sys = sleep_region_reg(s, s->soc->ps.disp[d][2]);
	if (on) {
		writel(0xf, sys);
		writel(0x1000000f, sys);
		ret |= t6050_disp_ps_settle(sys, 0xf0, 0xf0);
		writel(0xf, fe);
		writel(0x1000000f, fe);
		ret |= t6050_disp_ps_settle(fe, 0xf0, 0xf0);
		writel(0x0f040000, cpu);
		writel(0x0f04000f, cpu);
		ret |= t6050_disp_ps_settle(cpu, 0xf0, 0xf0);
		writel(0x1f04000f, cpu);
		writel(readl(pmc + T6050_DISP_PMC_BOOST) | boost, pmc + T6050_DISP_PMC_BOOST);
		writel(readl(pmc + T6050_DISP_PMC_BWR) | bwr, pmc + T6050_DISP_PMC_BWR);
		dev_dbg(s->dev, "external display %u power on: %d, CPU %#x FE %#x SYS %#x, PMC boost %#x bandwidth %#x\n",
			d - 1, ret, readl(cpu), readl(fe), readl(sys),
			readl(pmc + T6050_DISP_PMC_BOOST), readl(pmc + T6050_DISP_PMC_BWR));
		return ret;
	}
	writel(readl(pmc + T6050_DISP_PMC_BOOST) & ~boost, pmc + T6050_DISP_PMC_BOOST);
	writel(readl(pmc + T6050_DISP_PMC_BWR) & ~bwr, pmc + T6050_DISP_PMC_BWR);
	writel(0x0f0400ff, cpu);
	writel(0x0f0403f0, cpu);
	ret |= t6050_disp_ps_settle(cpu, 0xf0, 0);
	writel(0x3f0, fe);
	ret |= t6050_disp_ps_settle(fe, 0xf0, 0);
	writel(0x3f0, sys);
	ret |= t6050_disp_ps_settle(sys, 0xf0, 0);
	dev_dbg(s->dev, "external display %u power off: %d, CPU %#x FE %#x SYS %#x, PMC boost %#x bandwidth %#x\n",
		d - 1, ret, readl(cpu), readl(fe), readl(sys),
		readl(pmc + T6050_DISP_PMC_BOOST), readl(pmc + T6050_DISP_PMC_BWR));
	return ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_m5_dispext_power);




#include "sleep-v27-late.inc"

static int apple_sleep_profile_validate(const struct apple_sleep_soc *soc)
{
	if (!soc)
		return -ENODEV;
	if (soc->unavailable_reason)
		return -EOPNOTSUPP;

	/* Only a SoC with its own entry: what that entry calls without a test */
	if (!soc->early_context)
		return 0;
	if (soc->ngate_coprocs > SLEEP_GATES_MAX || soc->nstray > BITS_PER_LONG)
		return -EINVAL;
	if (!soc->ops || !soc->ops->cpu_prepare || !soc->ops->cpu_restore ||
	    !soc->ops->prepare_regs || !soc->ops->undo_regs ||
	    !soc->ops->display_off || !soc->ops->wake || !soc->ops->resume ||
	    !soc->ops->deep_wfi_arm || !soc->ops->aic_mask_all || !soc->ops->aic_ack ||
	    !soc->ops->aic_restore ||
	    (!soc->ops->commit_regs && (!soc->ops->lower_io || !soc->ops->last_mile)) ||
	    !soc->ndisp || soc->ndisp > SLEEP_DISPLAYS_MAX || !soc->cpu_clusters)
		return -EINVAL;
	return 0;
}

static int apple_sleep_profile_check(struct apple_sleep *s)
{
	int ret;

	if (!s)
		return -ENODEV;
	ret = apple_sleep_profile_validate(s->soc);

	if (ret)
		return dev_err_probe(s->dev, ret, "%s\n",
				     s->soc->unavailable_reason ?: "incomplete sleep profile");
	return 0;
}

static int apple_sleep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_sleep *s;
	int ret;

	/* Below EL2 a monitor owns all of this. */
	if (!is_kernel_in_hyp_mode())
		return -ENODEV;

	BUILD_BUG_ON(ARRAY_SIZE(t8132_quiesce_words) > 32);

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	s->soc = of_device_get_match_data(dev);
	if (!s->soc)
		return -ENODEV;
	if (s->soc->spmi_gen4) {
		s->spmi_status = SPMI4_STATUS;
		s->spmi_rx_empty = SPMI4_STATUS_RX_EMPTY;
		s->spmi_cmd = SPMI4_CMD;
		s->spmi_reply = SPMI4_REPLY;
		s->spmi_notify_enable = SPMI4_NOTIFY_ENABLE;
		s->spmi_notify_ack = SPMI4_NOTIFY_ACK;
	} else {
		s->spmi_status = SPMI_STATUS;
		s->spmi_rx_empty = SPMI_STATUS_RX_EMPTY;
		s->spmi_cmd = SPMI_CMD;
		s->spmi_reply = SPMI_REPLY;
		s->spmi_notify_enable = SPMI_NOTIFY_ENABLE;
		s->spmi_notify_ack = SPMI_NOTIFY_ACK;
	}

	ret = apple_sleep_profile_check(s);
	if (ret)
		return ret;

	/* Reject another board or CPU layout before map_regs can program hardware. */
	if (s->soc->cpu_clusters) {
		ret = apple_sleep_validate_cpu_topology(s);
		if (ret)
			return ret;
	}

	/* Without the boot object's half, a wake would rerun its boot flow. */
	ret = apple_sleep_map_mailbox(s);
	if (ret)
		return ret;

	if (s->soc->ops->map_regs) {
		ret = s->soc->ops->map_regs(s);
		if (ret)
			return ret;
	} else {
		ret = apple_sleep_map_regs(s);
		if (ret)
			return ret;
		ret = of_address_to_resource(dev->of_node,
					     of_property_match_string(dev->of_node, "reg-names", "ps"),
					     &s->ps_res);
		if (ret)
			return ret;
		ret = apple_sleep_ave_pd_init(s);
		if (ret)
			return dev_err_probe(dev, ret, "cannot link the AVE complex\n");
		s->driver_down = devm_bitmap_zalloc(dev, resource_size(&s->ps_res) / 4, GFP_KERNEL);
		if (!s->driver_down)
			return -ENOMEM;
		ret = apple_sleep_alloc_books(s);
		if (ret)
			return ret;
		scoped_guard(spinlock_irqsave, &sleep_slp_ddr_lock) {
			sleep_slp_ddr_word = s->wake + WAKE_SLP_DDR;
			sleep_slp_ddr_write();
		}

		ret = apple_sleep_read_wake_sources(s);
		if (ret)
			return ret;
	}

	ret = apple_sleep_map_displays(s);
	if (ret)
		return ret;
	if (!s->soc->regions)
		apple_sleep_map_apcie(s);
	if (s->soc->radio_retrain) {
		s->radio_host_np = of_parse_phandle(dev->of_node, "apple,radio-host", 0);
		if (s->radio_host_np &&
		    devm_add_action_or_reset(dev, apple_sleep_put_node, s->radio_host_np))
			return -ENOMEM;
	}
	apple_sleep_map_pins(s);
	apple_sleep_map_radio(s);
	apple_sleep_map_debug_port(s);

	platform_set_drvdata(pdev, s);

	ret = spin_table_set_power_ops(s->soc->ops->cpu_prepare ? &sleep_cpu_prepared_power_ops :
				       &sleep_cpu_power_ops);
	if (ret)
		return dev_err_probe(dev, ret, "cannot control CPU power\n");
	if (s->soc->pen_deep)
		spin_table_pen_deep_arm(true);

	/* Last: the exported helpers take it as a probed driver, and a failure frees s. */
	apple_sleep = s;
	/* Deep becomes the default sleep state, unless mem_sleep_default= chose another. */
	suspend_set_ops(s->soc->early_context ? &apple_sleep_early_ops : &apple_sleep_ops);
	/* Not on a SoC whose data is derived: there it is asked for by name. */
	if (s->soc->untested) {
		if (mem_sleep_current == PM_SUSPEND_MEM)
			mem_sleep_current = PM_SUSPEND_TO_IDLE;
		if (s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm && is_kernel_in_hyp_mode())
			t8142_c1_6_report(dev);
		if (s->soc->ops->deep_wfi_arm == t8142_deep_wfi_arm) {
			t8142_ed_map(dev);
			dcache_clean_poc((unsigned long)t8142_el1_wfi_stub,
					 (unsigned long)t8142_el1_wfi_stub_end);
			dev_info(dev, "probe: last WFI at %s (stub at PA %pa)\n",
				 !is_kernel_in_hyp_mode() || !READ_ONCE(sleep_el1_wfi) ? "the kernel's EL" :
				 READ_ONCE(sleep_el1_wfi) >= 2 ? "EL1 on every core" : "EL1 on the boot core",
				 &(phys_addr_t){ __pa_symbol(t8142_el1_wfi_stub) });
		}
		dev_info(dev, "probe: pwrgate wake %#x %#x cpm %#x %#x, commit %#x, clusters %#x %#x\n",
			 readl(s->pwrgate + s->soc->pwrgate_wake[0]),
			 readl(s->pwrgate + s->soc->pwrgate_wake[1]),
			 readl(s->pwrgate + s->soc->pwrgate_cpm[0]),
			 readl(s->pwrgate + s->soc->pwrgate_cpm[1]), readl(s->commit),
			 readl(s->cluster + s->soc->cluster_cpu[0]),
			 readl(s->cluster + s->soc->cluster_cpu[1]));
		dev_warn(dev, "sleep data derived for this SoC, untested: deep sleep is %s, and %s\n",
			 READ_ONCE(sleep_deep) ? "offered (apple_sleep.deep=1)" : "off (apple_sleep.deep=1 offers it)",
			 READ_ONCE(sleep_untested_commit) ? "the commit is allowed" :
			 "a sleep stops before the commit (dry run)");
	}

	dev_info(dev, "deep sleep %s: boot object resumes %llu\n",
		 mem_sleep_current == PM_SUSPEND_MEM ? "is the default" :
		 "available, not the default", READ_ONCE(s->mbox->resumes));

	return 0;
}

/*
 * T8140 (MacBook Neo). Power words from the ADT's /arm-io/pmgr, measured from macOS's sleep
 * under aura. CPU clusters: 0 is E, 1 is P, ADT /arm-io/pmgr reg[5], [7], [9] and [14], [16],
 * [18].
 */
#define T8140_PS_DISP_CPU	0x10000
#define T8140_PS_DISP_FE	0x318
#define T8140_PS_DISP_SYS	0x280
#define T8140_PS_DISPEXT0_CPU	0x328
#define T8140_PS_DISPEXT0_FE	0x320
#define T8140_PS_DISPEXT0_SYS	0x288
#define T8140_PS_SIO_CPU	0x2a8
#define T8140_PS_SEP		0xc00
#define T8140_PS_ATC0_USB_AON	0x3e0
#define T8140_PS_ATC0_COMMON	0x3e8
#define T8140_PS_ATC0_USB	0x3f0
#define T8140_PS_AP_TMM		0x1a0
#define T8140_PS_GPIO		0x120
#define T8140_PS_ANS		0x278
#define T8140_PS_APCIE_GP	0x258
#define T8140_PS_APCIE_SYS_GP	0x2a0
#define T8140_PS_APCIE_ST	0x310
#define T8140_PS_APCIE_SYS_ST	0x3a0
#define T8140_PS_APCIE_PHY_SW	0x3c0

/*
 * Every power word in the ADT, read just before ANS's stop so a run shows the whole chip.
 */
static const u32 t8140_ps_all[] = {
	0x0, 0x8, 0x10, 0x18, 0x30, 0x38, 0x50, 0x58, 0x60, 0x100, 0x108, 0x110, 0x118,
	0x120, 0x128, 0x130, 0x138, 0x140, 0x148, 0x150, 0x158, 0x160, 0x168, 0x170, 0x178,
	0x180, 0x188, 0x190, 0x198, 0x1a0, 0x1a8, 0x1b0, 0x1b8, 0x1c0, 0x1c8, 0x1d0, 0x1d8,
	0x1e0, 0x1e8, 0x1f0, 0x1f8, 0x200, 0x208, 0x210, 0x218, 0x220, 0x228, 0x230, 0x238,
	0x240, 0x248, 0x250, 0x258, 0x260, 0x268, 0x270, 0x278, 0x280, 0x288, 0x290, 0x298,
	0x2a0, 0x2a8, 0x2b0, 0x2b8, 0x2c0, 0x2c8, 0x2d0, 0x2d8, 0x2e0, 0x2e8, 0x2f0, 0x2f8,
	0x300, 0x308, 0x310, 0x318, 0x320, 0x328, 0x330, 0x338, 0x340, 0x348, 0x350, 0x358,
	0x360, 0x368, 0x370, 0x378, 0x380, 0x388, 0x390, 0x398, 0x3a0, 0x3a8, 0x3b0, 0x3b8,
	0x3c0, 0x3c8, 0x3d0, 0x3d8, 0x3e0, 0x3e8, 0x3f0, 0xc00, 0x4000, 0x4008, 0x4010,
	0x4018, 0x4020, 0x4028, 0x4030, 0x4038, 0x4040, 0x4048, 0x4050, 0x4058, 0x4060,
	0x8000, 0x8008, 0x8010, 0x8018, 0x8020, 0x8028, 0x8030, 0x8038, 0x8040, 0x8048,
	0xc000, 0xc008, 0xc010, 0x10000, 0x10008, 0x10010, 0x10018, 0x10020, 0x10028,
	0x10030, 0x10038, 0x10040,
};

static const u32 t8140_ps_aon_all[] = {
	0x0, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80, 0x88, 0x90, 0x98, 0xa0, 0xa8, 0xb0,
	0x4000, 0x4008, 0x4010, 0x4018, 0x4020, 0x4028, 0x4030, 0x4038, 0x4040, 0x4048,
	0x4050, 0x4058, 0x4060, 0x4068, 0x4070, 0x4078, 0x4080, 0x4088, 0x4090, 0x4098,
	0x40a0, 0x40a8, 0x40b0, 0x40b8, 0x40c0, 0x40c8, 0x40d0, 0x40d8, 0x40e0, 0x40e8,
	0x40f0, 0x40f8, 0x4100, 0x4108, 0x4110, 0x4118, 0x4120, 0x4128, 0x4130, 0x4138,
	0x4140, 0x4148, 0x4150, 0x4158, 0x4160, 0x4168, 0x4170, 0x4178, 0x4180, 0x8000,
	0x8008, 0x8010, 0x8018, 0x8020, 0x8028, 0x8030, 0x8038, 0x8040, 0x8048, 0x8050,
	0x8058, 0x8060, 0x8068, 0x8070, 0x8078,
};

/* What macOS has on at the commit and Linux leaves off, in its order: nothing uses them here. */
static const u32 t8140_ps_macos_on[] = {
	0x2f0, 0x2f8, 0x358, 0x360, 0x368, 0x370, 0x378, 0x380, 0x388, 0x118,
};

/*
 * The power words macOS's wake writes back on before any driver runs, in its order
 * (logs/wakerec-20261105-154137Z). The engine leaves domains like GPIO dark, and genpd still
 * has them on, so nothing else would.
 */
static const u32 t8140_ps_wake_order[] = {
	0xc00, 0x110, 0x118, 0x120, 0x168, 0x178, 0x2a8, 0x2f0, 0x2f8, 0x300, 0x358, 0x360,
	0x368, 0x370, 0x378, 0x380, 0x388, 0x3d0, 0x3d8,
};

static const u32 t8140_ps_aon_wake_order[] = { 0x78, 0x88, 0x8030, 0x8038 };

/* What macOS has off, highest first, as it lowers them. Domains with their own stop aren't here. */
static const u32 t8140_ps_macos_off[] = {
	0x4060, 0x4058, 0x4050, 0x4048, 0x4040, 0x4038, 0x4030, 0x4028, 0x4020, 0x4018,
	0x4010, 0x4008, 0x4000, 0x398, 0x390, 0x350, 0x348, 0x340, 0x338, 0x330,
	0x308, 0x2e8, 0x2e0, 0x2d8, 0x2d0, 0x2c8, 0x2c0, 0x2b8, 0x2b0, 0x298,
	0x290, 0x270, 0x268, 0x250, 0x248, 0x240, 0x238, 0x230, 0x1a0, 0x190,
	0x188, 0x180, 0x170, 0x160, 0x158, 0x150, 0x148, 0x140, 0x108,
};

static const u32 t8140_ps_aon_macos_off[] = {
	0x8028, 0x4140, 0x40f8, 0x80,
};

/* The AOP's SPMI words: on, with AUTO_ENABLE off on macOS. */
static const u32 t8140_ps_aon_macos_no_auto[] = {
	0x4078, 0x4080,
};

/* M2A_1B's power-gate polls, less the SEP, PMP and SIO switch, which macOS doesn't have off. */
static const u32 t8140_ps_engine_polled[] = {
	0xc008, 0xc010, 0xc000, 0x290, 0x270, 0x8038, 0x8030, 0x8048, 0x8040, 0x8028, 0x248,
	0x8010, 0x8008, 0x8020, 0x8018, 0x8000, 0x240, 0x4038, 0x4030, 0x4028, 0x4040, 0x4060,
	0x4058, 0x4050, 0x4048, 0x4018, 0x4010, 0x4008, 0x4000, 0x268, 0x1f0, 0x298, 0x238,
	0x3b8, 0x3b0, 0x328, 0x288, 0x308, 0x3a0, 0x2a0, 0x278,
};

static const u32 t8140_ps_still_on[] = {
	T8140_PS_ATC0_COMMON, T8140_PS_DISP_SYS, T8140_PS_DISPEXT0_SYS, T8140_PS_ANS,
	0x270 /* AVD_SYS */, 0x240 /* VENC0_SYS */, 0x248 /* VENC1_SYS */, 0x1f0 /* GFX */,
	0x230 /* JPG */, T8140_PS_APCIE_SYS_GP,
};

/*
 * What XNU polls just before its complex words, and panics on if one is on, besides the
 * display's, ANS's and ATC0's, which the commit lowers itself.
 */
static const struct apple_sleep_word t8140_stray_words[] = {
	{ "AVD_SYS", 0x270 }, { "VENC0_SYS", 0x240 }, { "VENC1_SYS", 0x248 },
	{ "GFX", 0x1f0 }, { "JPG", 0x230 },
};

/*
 * A commit with the PMP and SIO still running reset the SoC 5.5 s later. The DCPs' drivers
 * power their CPUs off at DPMS off.
 */
static const struct apple_sleep_gate_coproc t8140_gate_coprocs[] = {
	{ "/soc/dcp@412e00000", true, T8140_PS_DISP_CPU },
	{ "/soc/dcp@416e00000", true, T8140_PS_DISPEXT0_CPU },
	{ "/soc/pmp@300500000", true },	{ "/soc/dma-controller@385e00000", true, T8140_PS_SIO_CPU },
	{ "/soc/nvme@3cdcc0000" },	{ "/soc/mtp@314600000" },
	{ "/soc/aop@310c00000" },	{ "/soc/smc@30c600000" },
	{ "/soc/sep@282600000" },	{ "/soc/gpu@482600000" },
};

static const struct apple_sleep_word t8140_gate_off[] = {
	{ "ane_sys", 0x290 }, { "isp_sys", 0x268 }, { "prores", 0x250 },
	{ "avd_sys", 0x270 }, { "jpg", 0x230 }, { "venc0_sys", 0x240 },
};

/* VENC0_SYS, VENC1_SYS */
static const u32 t8140_ave_domains[] = { 0x240, 0x248 };

static const struct apple_sleep_wake_name t8140_wake_names[] = {
	{ 0, "AOP.AICWakeupAP" }, { 5, "AOP.CPUWakeupAP" },
	{ 42, "AOP.OutboxNotEmpty" }, { 62, "AOP.SPMI0.SW3" },
	{ 66, "AOP.SPMI1.SW3" }, { 70, "ATC0_PMGR_USB_WAKEUP" },
	{ 80, "MTP.DOCK.CHANNELS.AP0.IRQ" }, { 99, "MTP.OUTBOX.N.EMPTY_IRQ" },
	{ 109, "NUB.APWakeupTime" }, { 110, "NUB.APWatchdog" },
	{ 125, "NUB.GPIO.IRQ6" }, { 131, "NUB.SPMI0.SW3" },
	{ 136, "NUB.SPMI1.SW3" }, { 140, "NUB.WakeupAP" },
	{ 145, "SMC.CPUWakeupAP" }, { 165, "SMC.OutboxNotEmpty" },
	{ 211, "AOP.TrustedOutboxNotEmpty" }, { 227, "AOP2.OutboxNotEmpty" },
	{ 231, "AOP2.TrustedOutboxNotEmpty" }, { 236, "AOP2.CPUWakeupAP" },
};

static const struct apple_sleep_ops t8140_sleep_ops = {
	.deep_wfi_arm = t8140_deep_wfi_arm,
	.lower_io = apple_sleep_lower_io,
	.last_mile = t8140_last_mile,
	.resume = cpu_resume,
	.wake = t8140_wake,
	.aic_mask_all = aic_sleep_mask_all,
	.aic_ack = aic_sleep_ack,
	.aic_restore = aic_sleep_restore,
};

static const struct apple_sleep_soc t8140_sleep_soc = {
	.ops = &t8140_sleep_ops,
	.has_ave = true,
	.ack_cluster_dcs = true,
	.ndisp = 2,
	.display_order = { 1, 0 },
	/* E, P */
	.cluster_cpu = { 0x0, 0x4 },
	.ncluster_cpu = 2,
	.cpu_start_stride = 4,
	.clk_mux_parked = { 0x4, 0x8, 0x0 },
	.nclk_mux_parked = 3,
	.clk_mux_parked_val = 0x80100000,
	/* CLLT: names from nothing. Bits measured cleared by macOS before the commit. */
	.cllt = { { 0x30, GENMASK(7, 0) }, { 0x34, GENMASK(7, 0) }, { 0x3c, GENMASK(17, 16) } },
	.ncllt = 3,
	/* The PMU's boot-stage register: the AP sleeps */
	.pmu_usid = 0xe,
	.pmu_sleep_reg = 0xf701,
	.pmu_sleep_val = 0x50,
	.spmi_notify_sleep = 0x2000001,
	.nmcc = 3,
	.namcc = 4,
	.wake_mask = 0x08,
	.wake_status = 0x28,
	.wake_words = 8,
	.ps = {
		.disp = {
			{ T8140_PS_DISP_CPU, T8140_PS_DISP_FE, T8140_PS_DISP_SYS },
			{ T8140_PS_DISPEXT0_CPU, T8140_PS_DISPEXT0_FE, T8140_PS_DISPEXT0_SYS },
		},
		.sio_cpu = T8140_PS_SIO_CPU,
		.sep = T8140_PS_SEP,
		.usb_aon = T8140_PS_ATC0_USB_AON,
		.usb_common = T8140_PS_ATC0_COMMON,
		.usb = T8140_PS_ATC0_USB,
		.ap_tmm = T8140_PS_AP_TMM,
		.gpio = T8140_PS_GPIO,
		.ans = T8140_PS_ANS,
		.apcie_gp = T8140_PS_APCIE_GP,
		.apcie_sys_gp = T8140_PS_APCIE_SYS_GP,
		.apcie_st = T8140_PS_APCIE_ST,
		.apcie_sys_st = T8140_PS_APCIE_SYS_ST,
		.apcie_phy_sw = T8140_PS_APCIE_PHY_SW,
	},
	.ps_aon_debug_switch = 0xb0,
	.pwrgate_wake = { 0x000, 0x040 },
	.pwrgate_cpm = { 0x060, 0x068 },
	.pwrgate_disp = { { 0xc8c, 0xcd0 } },
	.ps_all = SLEEP_WORDS(t8140_ps_all),
	.ps_aon_all = SLEEP_WORDS(t8140_ps_aon_all),
	.ps_macos_on = SLEEP_WORDS(t8140_ps_macos_on),
	.ps_macos_off = SLEEP_WORDS(t8140_ps_macos_off),
	.ps_aon_macos_off = SLEEP_WORDS(t8140_ps_aon_macos_off),
	.ps_aon_macos_no_auto = SLEEP_WORDS(t8140_ps_aon_macos_no_auto),
	.ps_wake_order = SLEEP_WORDS(t8140_ps_wake_order),
	.ps_aon_wake_order = SLEEP_WORDS(t8140_ps_aon_wake_order),
	.ps_still_on = SLEEP_WORDS(t8140_ps_still_on),
	.ps_engine_polled = SLEEP_WORDS(t8140_ps_engine_polled),
	.stray = t8140_stray_words,
	.nstray = ARRAY_SIZE(t8140_stray_words),
	.gate_off = t8140_gate_off,
	.ngate_off = ARRAY_SIZE(t8140_gate_off),
	.gate_coprocs = t8140_gate_coprocs,
	.ngate_coprocs = ARRAY_SIZE(t8140_gate_coprocs),
	.wake_names = t8140_wake_names,
	.nwake_names = ARRAY_SIZE(t8140_wake_names),
	/* 21 sources. Measured: macOS reads 0x001fef77 there. */
	.wake_last_word = GENMASK(20, 0),
	.ave_domains = t8140_ave_domains,
	.nave_domains = ARRAY_SIZE(t8140_ave_domains),
	.ave_pmc = "apple,t8140-pmc",
	.ave_pmc_cluster = 3,
};

static const struct apple_sleep_wake_name t8132_wake_names[] = {
	{ 5, "AOP.CPUWakeupAP" }, { 12, "AOP.GPIO.IRQ4" }, { 14, "AOP.GPIO.IRQ6" },
	{ 42, "AOP.OutboxNotEmpty" }, { 60, "AOP.SPMI0.SW3" }, { 64, "AOP.SPMI1.SW3" },
	{ 68, "ATC0.CIOWakeup" }, { 69, "ATC0.USBWakeup" }, { 70, "ATC1.CIOWakeup" },
	{ 71, "ATC1.USBWakeup" }, { 72, "ATC2.CIOWakeup" }, { 73, "ATC2.USBWakeup" },
	{ 74, "ATC3.CIOWakeup" }, { 75, "ATC3.USBWakeup" },
	{ 85, "MTP.DOCK.CHANNELS.AP0.IRQ" }, { 104, "MTP.OutboxNotEmpty" },
	{ 122, "NUB.APWakeupTime" }, { 123, "PMGR_AON.APWatchdog" }, { 138, "NUB.GPIO.IRQ6" },
	{ 145, "NUB.SPMI0.SW3" }, { 150, "NUB.SPMI1.SW3" }, { 156, "NUB.WakeupAP" },
	{ 161, "SMC.CPUWakeupAP" }, { 183, "SMC.OutboxNotEmpty" },
	{ 229, "AOP.TrustedOutboxNotEmpty" },
};

static const struct apple_sleep_word t8132_stray_words[] = {
	{ "AVD_SYS", 0x580 }, { "VENC_SYS", 0x308 }, { "GFX", 0x250 }, { "JPG", 0x578 },
};

/* Every power word in the J713's ADT, as t8140_ps_all[] is T8140's. */
static const u32 t8132_ps_all[] = {
	0x0, 0x8, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48,
	0x50, 0x58, 0x100, 0x108, 0x110, 0x118, 0x120, 0x128, 0x130, 0x138,
	0x140, 0x148, 0x150, 0x158, 0x160, 0x168, 0x170, 0x178, 0x180, 0x188,
	0x190, 0x198, 0x1a0, 0x1a8, 0x1b0, 0x1b8, 0x1c0, 0x1c8, 0x1d0, 0x1d8,
	0x1e0, 0x1e8, 0x228, 0x230, 0x238, 0x240, 0x248, 0x250, 0x258, 0x260,
	0x268, 0x270, 0x278, 0x280, 0x288, 0x290, 0x298, 0x2a0, 0x2a8, 0x2b0,
	0x2b8, 0x2c0, 0x2c8, 0x2d0, 0x2d8, 0x2e0, 0x2e8, 0x2f0, 0x2f8, 0x300,
	0x308, 0x310, 0x318, 0x320, 0x328, 0x330, 0x338, 0x340, 0x348, 0x350,
	0x358, 0x360, 0x368, 0x370, 0x378, 0x380, 0x388, 0x390, 0x398, 0x3a0,
	0x3a8, 0x3b0, 0x3b8, 0x3c0, 0x3c8, 0x3d0, 0x3d8, 0x3e0, 0x3e8, 0x3f0,
	0x3f8, 0x400, 0x408, 0x410, 0x418, 0x420, 0x428, 0x430, 0x438, 0x440,
	0x448, 0x450, 0x458, 0x460, 0x468, 0x470, 0x478, 0x480, 0x488, 0x490,
	0x498, 0x4a0, 0x4a8, 0x4b0, 0x4b8, 0x4c0, 0x4c8, 0x4d0, 0x4d8, 0x4e0,
	0x4e8, 0x4f0, 0x530, 0x538, 0x540, 0x548, 0x550, 0x558, 0x560, 0x568,
	0x570, 0x578, 0x580, 0xc00, 0x4000, 0x4008, 0x4010, 0x4018, 0x8000, 0x8008,
	0x8010, 0x8018, 0x8020, 0xc000, 0xc008, 0xc010, 0xc018, 0x10000,
};

static const u32 t8132_ps_aon_all[] = {
	0x0, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80, 0x88, 0x90, 0x98,
	0xa0, 0xa8, 0xb0, 0xb8, 0xc0, 0xc8, 0xd0, 0xd8, 0xe0, 0xe8,
	0x4020, 0x4090, 0x40b8, 0x40c0, 0x40c8, 0x40d0, 0x40d8, 0x40e0, 0x40e8, 0x40f0,
	0x40f8, 0x4100, 0x4108, 0x4110, 0x4118, 0x4120, 0x4128, 0x4130, 0x4138, 0x4140,
	0x4148, 0x4150, 0x4158, 0x4160, 0x4168, 0x8028, 0x8030, 0x8038, 0x8040, 0x8048,
	0x8080, 0x8088,
};

static const u32 t8132_ps_wake_order[] = {
	0xc00, 0x110, 0x118, 0x120, 0x178, 0x2d0, 0x2d8, 0x2e0,
	0x3c8, 0x3d0, 0x3d8, 0x3e0, 0x3e8, 0x3f0, 0x488, 0x490,
	0x538, 0x4d0, 0x4d8, 0x540, 0x548, 0x550, 0x268, 0x560,
	0x568, 0x250, 0x4e0, 0x4e8, 0x4f0, 0x240, 0x248, 0x10000,
	0x1d8, 0x1e0, 0x1e8, 0x380, 0x2a0,
};

static const u32 t8132_ps_aon_wake_order[] = { 0x90 };

/*
 * T8132's: J713, macOS 25G83 under aura, against Linux at the commit. The domains left out
 * have their own stop (display, ANS, APCIE, SEP, GFX, SIO_CPU), or macOS leaves them to
 * AUTO_ENABLE with the target at f: the AFISOCNI words and NUB_SRAM.
 */
/* What macOS has at f at the commit that idles lower. T8140's list has DWI too. */
static const u32 t8132_ps_macos_on[] = {
	0x2f8, 0x310, 0x318, 0x320, 0x348, 0x358, 0x368, 0x118,
};

static const u32 t8132_ps_macos_off[] = {
	0xc018, 0xc010, 0xc008, 0xc000, 0x8020, 0x8018, 0x8010, 0x8008, 0x8000, 0x4018,
	0x4010, 0x4008, 0x4000, 0x580, 0x578, 0x570, 0x568, 0x560, 0x530, 0x4b8,
	0x4b0, 0x4a0, 0x498, 0x480, 0x478, 0x468, 0x460, 0x458, 0x450, 0x448,
	0x440, 0x438, 0x430, 0x428, 0x420, 0x3c8, 0x3c0, 0x3b8, 0x3b0, 0x3a8,
	0x3a0, 0x398, 0x390, 0x388, 0x380, 0x370, 0x308, 0x300, 0x2f0, 0x2e8,
	0x2a8, 0x2a0, 0x298, 0x290, 0x288, 0x278, 0x230, 0x228, 0x1b8, 0x190,
	0x188, 0x170, 0x168,
};

static const u32 t8132_ps_aon_macos_off[] = {
	0x4160, 0x40e8, 0x40d8, 0x40b8, 0x4090, 0x4020, 0xe8, 0xe0, 0xd8, 0xd0, 0xc8, 0xb8,
};

/* On at state 4 at the commit on macOS, and off under Linux. */
static const u32 t8132_ps_aon_macos_on[] = {
	0x40c0, 0x40c8, 0x40f8, 0x4118, 0x4128, 0x4130, 0x4138,
};

/* T8132's, from the J713 device tree. @ps is the DCP CPU's domain, as on T8140. */
static const struct apple_sleep_gate_coproc t8132_gate_coprocs[] = {
	{ "/soc/dcp@30ae00000", true, 0x10000 },
	{ "/soc/dcp@286e00000", true, 0x1e8 },
	{ "/soc/dcp@48ae00000", true, 0x4f0 },
	{ "/soc/pmp@380500000", true },	{ "/soc/dma-controller@3ade00000", true },
	{ "/soc/nvme@40000" },		{ "/soc/mtp@394600000" },
	{ "/soc/aop@390600000" },	{ "/soc/smc@38c600000" },
	{ "/soc/sep@3aa600000" },
	/* Its driver's last stop lowers the domain. */
	{ "/soc/gpu@302600000", true, 0x250 },
};

static const struct apple_sleep_word t8132_gate_off[] = {
	{ "ane_sys", 0x570 }, { "isp_sys", 0x530 }, { "avd_sys", 0x580 },
	{ "jpg", 0x578 }, { "venc_sys", 0x308 },
};

static const struct apple_sleep_ops t8132_sleep_ops = {
	/* As T8140's */
	.deep_wfi_arm = t8140_deep_wfi_arm,
	.lower_io = t8132_lower_io,
	.last_mile = t8132_last_mile,
	/* It restarts WD2 for the resume. */
	.resume = t8132_resume_stub,
	.pre_commit = t8132_pre_commit,
	.wake = t8132_wake,
	.aic_mask_all = aic_sleep_mask_all,
	.aic_ack = aic_sleep_ack,
	.aic_restore = aic_sleep_restore,
};

static const struct apple_sleep_soc t8132_sleep_soc = {
	.ops = &t8132_sleep_ops,
	.boot_wfi_delay_ms = 3,
	.wake_optional = true,
	/* DCP, DISPEXT0, DISPEXT1, each with its own pwrgate pair, measured in macOS's trace. */
	.ndisp = 3,
	.lower_unbound_displays = true,
	.disp_prot_slice = { 0, 5, 30 },
	.display_order = { 1, 2, 0 },
	/* E, P */
	.cluster_cpu = { 0x0, 0x4 },
	.ncluster_cpu = 2,
	/* Six E cores. Measured: macOS writes bits 0-5 and 7-9, and its boot core is bit 6. */
	.cpu_start_stride = 6,
	.clk_mux_parked = { 0x10, 0x14, 0x0, 0x4 },
	.nclk_mux_parked = 4,
	.clk_mux_parked_val = 0x80100000,
	/* The PMU's boot-stage register: the AP sleeps */
	.pmu_usid = 0xe,
	.pmu_sleep_reg = 0xf801,
	.pmu_sleep_val = 0x50,
	.spmi_notify_sleep = 0x2000001,
	/* The two of each that macOS steps down */
	.nmcc = 2,
	.namcc = 2,
	.wake_mask = 0x08,
	.wake_status = 0x28,
	.wake_words = 8,
	.ps = {
		.disp = {
			{ 0x10000, 0x248, 0x240 },
			{ 0x1e8, 0x1e0, 0x1d8 },
			{ 0x4f0, 0x4e8, 0x4e0 },
		},
		.sio_cpu = 0x268,
		.sep = 0xc00,
		.ap_tmm = 0x1b8,
		.gpio = 0x120,
		.ans = 0x538,
		.apcie_gp = 0x4d0,
		.apcie_sys_gp = 0x4d8,
		.apcie_st = 0x540,
		.apcie_sys_st = 0x548,
		.apcie_phy_sw = 0x550,
	},
	.ps_aon_debug_switch = 0xa8,
	.pwrgate_wake = { 0x000, 0x060 },
	.pwrgate_cpm = { 0x0a0, 0x0a8 },
	.pwrgate_disp = { { 0x104c, 0x1070 }, { 0x16cc, 0x16f0 }, { 0x1b4c, 0x1b70 } },
	.ps_all = SLEEP_WORDS(t8132_ps_all),
	.ps_aon_all = SLEEP_WORDS(t8132_ps_aon_all),
	.ps_macos_on = SLEEP_WORDS(t8132_ps_macos_on),
	.ps_macos_off = SLEEP_WORDS(t8132_ps_macos_off),
	.ps_aon_macos_off = SLEEP_WORDS(t8132_ps_aon_macos_off),
	.ps_aon_macos_on = SLEEP_WORDS(t8132_ps_aon_macos_on),
	.ps_wake_order = SLEEP_WORDS(t8132_ps_wake_order),
	.ps_aon_wake_order = SLEEP_WORDS(t8132_ps_aon_wake_order),
	.stray = t8132_stray_words,
	.nstray = ARRAY_SIZE(t8132_stray_words),
	.gate_off = t8132_gate_off,
	.ngate_off = ARRAY_SIZE(t8132_gate_off),
	.gate_coprocs = t8132_gate_coprocs,
	.ngate_coprocs = ARRAY_SIZE(t8132_gate_coprocs),
	.wake_names = t8132_wake_names,
	.nwake_names = ARRAY_SIZE(t8132_wake_names),
	.wake_last_word = GENMASK(20, 0),
	.quiesce = SLEEP_WORDS(t8132_quiesce_words),
};

/*
 * T8142 (M5: MacBook Air J813/J815, MacBook Pro J704). No M5 sleep has been traced: this is
 * T8132's sleep with T8142's words, translated by name from the J815 ADT and checked against
 * the macOS 27.2.1 kernelcache (26B131) where the code says more:
 *  - XNU's last-WFI path is the same code for both SoCs.
 *  - AppleT8142PMGR's sleep entry and restoreHW are T8132's in order. A complex comes on as
 *    (word | 0xf), then bit 4 clears; restoreHW then sets pwrgate +0x0 and +0x60, the boot
 *    cores of E and P, read-modify-write.
 *  - Its quiescePerfState takes the SoC to 3 and DCS to 3, through 5 from 6 and up, before
 *    the complexes go off, and adds a "sleep hack" right before the S2R timer:
 *    PMGR_SOC_CLLT.EARLY_WAKEUP_CFG (+0x50) bits 17:16 cleared. Its EVENT_CFG step calls a
 *    method that is empty on T8142. The S2R timer is T8132's commit word.
 *  - The wake block has nine mask words (sources 0..267), then nine status words.
 *  - No clock mux and no memory-controller lane step: AppleT8142MCC turns the cache's ways
 *    off as its platform sleep action, which mcc.c does at suspend_noirq.
 *  - Its SPMI controllers are gen 4.
 * iBoot leaves the boot core's power-gate enable (+0x60) clear: with it clear the boot core
 * never powers off in its last WFI, the P complex stays BUSY, and WD2 resets the SoC after
 * the commit. The DT's cluster power gate sets it at boot (apple-cluster-pwrgate.c), as on
 * T8132 and T6050.
 */
#define T8142_PMC_VOTER_AP		(3 * PMC_VOTER_STRIDE)
#define T8142_CLUSTER_BUSY		BIT(4)
/*
 * AppleT8142PMGR::setPerfState's words in PMGRCLKGEN_SOC_TOP (/arm-io/pmgr reg[2]): the SoC's
 * at 0x24000, DCS's at 0x25000, each like T8132's: [3:0] asks, [11:8] has reached, bit 31 busy.
 */
#define T8142_PERF_REQ			GENMASK(3, 0)
#define T8142_DCS_SLEEP_STATE		3
#define T8142_DCS_STEP_STATE		5
#define T8142_DCS_STEP_FROM		6
#define T8142_DCS_MAX_STATE		10

/*
 * Every Linux access to the complexes' ACC word 0xe440f8 has hung J815 so far: a read at
 * probe, an on-demand read, and the one wake that wrote it (the only T8142 sleep that went
 * down, then hung on its way back). macOS writes 1 after each complex powers on and at the
 * end of restoreHW; here that is only done when asked for.
 */
static bool sleep_acc_wake;
module_param_named(acc_wake, sleep_acc_wake, bool, 0644);
MODULE_PARM_DESC(acc_wake,
		 "T8142: write ACC 0xe440f8 = 1 per complex at a wake, as restoreHW does (default 0)");

static int t8142_perf_request(void __iomem *reg, unsigned int state)
{
	writel((readl(reg) & ~T8142_PERF_REQ) | FIELD_PREP(T8142_PERF_REQ, state), reg);
	return sleep_perf_wait(reg, state);
}

/*
 * AppleT8142PMGR::quiescePerfState, the PMGR's first sleep step: the SoC to Vmin (3), then DCS
 * to 3, through 5 from 6..10. macOS waits on BUSY with no limit and panics on a DCS state
 * above 10; here each wait is bounded, and such a state is left alone.
 */
static void t8142_perf_down(struct apple_sleep *s)
{
	unsigned int state = FIELD_GET(DCS_STATE, readl(s->dcs));

	s->perf_dcs_from = state;
	s->perf_soc_ret = t8142_perf_request(s->clk_perf + CLK_PERF_SOC, SOC_SLEEP_STATE);
	if (state > T8142_DCS_MAX_STATE) {
		s->perf_dcs_ret = -ERANGE;
		return;
	}
	if (state >= T8142_DCS_STEP_FROM)
		t8142_perf_request(s->dcs, T8142_DCS_STEP_STATE);
	s->perf_dcs_ret = t8142_perf_request(s->dcs, T8142_DCS_SLEEP_STATE);
}

/* AppleT8142PMGR vtable 0xfa0: each complex's CPM gate word, read-modify-write. */
static void t8142_cpm_on(struct apple_sleep *s)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s->soc->pwrgate_cpm); i++) {
		void __iomem *reg = s->pwrgate + s->soc->pwrgate_cpm[i];

		writel(readl(reg) | PWRGATE_ON, reg);
	}
}

/*
 * apple_sleep.extra_off, after every coprocessor's last stop and before ATC0/PCIe/PMP: each listed
 * word that is on, in the list's order (children first), waited off; one that stays on is noted,
 * not refused (past the last refusal point).
 */
static void t8142_extra_lower(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	unsigned int i;

	s->extra_lowered = 0;
	if (!READ_ONCE(sleep_extra_off))
		return;
	for (i = 0; i < min_t(unsigned int, soc->nextra_off, SLEEP_EXTRA_OFF_MAX); i++) {
		u32 off = soc->extra_off[i].offset, v = readl(sleep_power_reg(s, off));

		s->extra_saved[i] = v;
		if (!FIELD_GET(PS_ACTUAL, v))
			continue;
		sleep_ps_lower(s, off);
		s->extra_lowered |= BIT(i);
		if (sleep_ps_wait_off(s, off))
			s->extra_stuck |= BIT(i);
	}
}

/* At the wake, parents first: each lowered word back once its parent reads on. */
static void t8142_extra_restore(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i;

	for (i = min_t(unsigned int, soc->nextra_off, SLEEP_EXTRA_OFF_MAX) - 1; i >= 0; i--) {
		u32 parent = soc->extra_off[i].parent;

		if (!(s->extra_lowered & BIT(i)))
			continue;
		if (!parent || FIELD_GET(PS_ACTUAL, readl(sleep_power_reg(s, parent))))
			sleep_ps_restore(sleep_power_reg(s, soc->extra_off[i].offset),
					 s->extra_saved[i]);
	}
	s->extra_lowered = 0;
}

/* T8132's, with T8142's ATC0 words; ANS's PCIe block in AppleT8142PCIe's order. */
static void t8142_lower_io(struct apple_sleep *s)
{
	sleep_note(s, 0x61);
	t8132_clean_ram();
	sleep_note(s, 0x62);
	t8142_extra_lower(s);
	s->atc0_saved[0] = readl(s->ps_aon + T8142_PS_AON_ATC0_USB);
	s->atc0_saved[1] = readl(s->ps_aon + T8142_PS_AON_ATC0_USB_AON);
	s->atc0_saved[2] = readl(s->ps + T8142_PS_ATC0_COMMON);
	/* apple_sleep.ps_keep bit 3: as macOS would, if its ports weren't hidden in the trace */
	if (!(s->ps_keep_mask & BIT(3))) {
		t8132_aon_lower(s, T8142_PS_AON_ATC0_USB);
		if (!READ_ONCE(sleep_usb_asleep))
			t8132_aon_lower(s, T8142_PS_AON_ATC0_USB_AON);
		sleep_ps_lower(s, T8142_PS_ATC0_COMMON);
	}
	if (s->pcie_ans_stopped) {
		apple_sleep_lower_apcie(s);
		s->ans_off_ret = sleep_ps_wait_off(s, s->soc->ps.ans);
	}
	/* PMP's final stop has idled its firmware and saved its SRAM; RUN is clear (#4). */
	s->pmp_lowered = false;
	s->pmp_off_ret = 0;
	if (READ_ONCE(sleep_pmp_off) && sleep_ps_on(s, T8142_PS_PMP)) {
		s->pmp_saved = readl(s->ps + T8142_PS_PMP);
		sleep_ps_lower(s, T8142_PS_PMP);
		s->pmp_lowered = true;
		s->pmp_off_ret = sleep_ps_wait_off(s, T8142_PS_PMP);
	}
	sleep_note(s, 0x63);
}

/*
 * The last note before the cores' window (0x65): +2 flags, +4 the first armed wake source
 * already pending (low 8 bits; 0xff none). Flags: 0x01 a parked CPU didn't stop, 0x02 the SoC
 * perf state, 0x04 the DCS state didn't settle, 0x08 an armed wake source is pending, 0x10 it
 * is >= 256, 0x20 the commit word read non-zero (it reads 1 from boot on J815). No PMU traffic
 * after it: the engine gives the cores 33 ms from the complex words to their WFI.
 */
/*
 * The PMU's RTC alarm after the S2R timer, as macOS 27.2 sends it on the J813 (Thomas
 * Glanzmann, m0d16l14n1/linux#3, from trace g01: 0.11 to 0.25 ms after the timer, boot core):
 * read RTC_CTL, 0x43 if it had fired, the 4-byte compare at 0x2008, 0x200e <- 0, RTC_CTL <-
 * 0x41. T6050's post-timer step at the same addresses (apple,pmu-alarm-rearm 0x43 0x41). The
 * opcode carries the byte count (his follow-up fix). The compare and the counter are read here,
 * before the cores' window, with the notes; the compare is now + apple_sleep.t8142_wake_s
 * (0 keeps the PMU's own), which is also a wake source: J815 has no RTC device under Linux.
 * J813 still resets with it (PMU took all four commands), so this is parity, not the fix.
 */
static bool sleep_pmu_alarm = true;
module_param_named(pmu_alarm, sleep_pmu_alarm, bool, 0644);
MODULE_PARM_DESC(pmu_alarm, "T8142: send the PMU's RTC alarm after the S2R timer, as macOS does (default Y)");
static unsigned int t8142_wake_s = 30;
module_param(t8142_wake_s, uint, 0644);
MODULE_PARM_DESC(t8142_wake_s, "T8142: the PMU RTC alarm this many s after the commit (0: keep the PMU's)");
static u8 t8142_alarm[4];
static bool t8142_alarm_ok;
static int t8142_post_ret = -ENODATA;
static u64 t8142_alarm_counter;

static int t8142_pmu(struct apple_sleep *s, u32 mask, u8 opcode, u16 address, const u8 *wr,
		     u8 *rd, unsigned int len)
{
	return t6050_sleep_pmu_xfer_post(s, mask, opcode | (len - 1), address, wr, rd, len);
}

static void t8142_alarm_prepare(struct apple_sleep *s)
{
	u32 mask = s->soc->spmi_notify_sleep;
	u8 ticks[8] = {};

	t8142_alarm_ok = false;
	t8142_post_ret = -ENODATA;
	if (!s->spmi || !READ_ONCE(sleep_pmu_alarm))
		return;
	t6050_sleep_pmu_drain(s);
	if (!t8142_pmu(s, mask, T6050_SPMI_EXT_READL, T6050_RTC_COMPARE, NULL, t8142_alarm, 4) &&
	    !t8142_pmu(s, mask, T6050_SPMI_EXT_READL, T6050_RTC_COUNTER, NULL, ticks, 6)) {
		t8142_alarm_counter = get_unaligned_le64(ticks) >> 16;
		if (READ_ONCE(t8142_wake_s))
			put_unaligned_le32(t8142_alarm_counter + READ_ONCE(t8142_wake_s), t8142_alarm);
		t8142_alarm_ok = true;
	}
	t6050_sleep_pmu_drain(s);
}

static void t8142_post_timer(struct apple_sleep *s)
{
	u32 mask = s->soc->spmi_notify_sleep & ~M5_SPMI_IRQ_DONE;
	u8 ctl = 0, v;
	int ret;

	if (!t8142_alarm_ok)
		return;
	ret = t8142_pmu(s, mask, T6050_SPMI_EXT_READL, T6050_RTC_CTL, NULL, &ctl, 1);
	if (!ret && (ctl & T6050_RTC_CTL_FIRED)) {
		v = 0x43;
		ret = t8142_pmu(s, mask, T6050_SPMI_EXT_WRITEL, T6050_RTC_CTL, &v, NULL, 1);
	}
	if (!ret)
		ret = t8142_pmu(s, mask, T6050_SPMI_EXT_WRITEL, T6050_RTC_COMPARE, t8142_alarm, NULL, 4);
	if (!ret) {
		v = 0;
		ret = t8142_pmu(s, mask, T6050_SPMI_EXT_WRITEL, T6050_RTC_MASK, &v, NULL, 1);
	}
	if (!ret) {
		v = 0x41;
		ret = t8142_pmu(s, mask, T6050_SPMI_EXT_WRITEL, T6050_RTC_CTL, &v, NULL, 1);
	}
	if (!ret)
		sleep_spmi_alarm_wake_arm(s);
	t8142_post_ret = ret;
}

/*
 * What survives the reset, at 0x8310.. (J813's #4 used the same bytes): the power words as the
 * commit has them. +8 ps_all words on without AUTO_ENABLE, +9 ps_all words on that macOS has off,
 * +10/+11 ps_all words on that macOS never touched (past the CPUs) and the first one's index
 * (0xff none), +12 ps_aon_all words on
 * without it, +13 the first such index, +14/+15 the SEP word's bits 7:0 and 31:24 (macOS: 0x0f,
 * 0x10), +5..+7 and +16..+19 the first seven of +9's. PMU scratch from 0x831f up holds data
 * that isn't Linux's (the same on every boot): nothing is written there.
 */
#define T8142_PS_SEP	0xc00
/*
 * The ps_all words macOS has OFF (actual state 0) at its commit: the last value each had in the
 * J813 g01 end state (notes/sleep-trace/macos-ps-endstate-g01-sleep1.txt). Of t8142_ps_all's
 * 156: 66 off, 38 on, 52 never touched in the trace (unknown). By actual state only: a written
 * 0x100000ff reads back without AUTO_ENABLE on macOS too, so that bit can't be compared.
 */
static bool t8142_macos_off_at_commit(u32 offset)
{
	static const u32 off[] = {
		0x190, 0x198, 0x1a0, 0x1a8, 0x1b0, 0x290, 0x298, 0x2a0, 0x2a8, 0x2b8,
		0x2c0, 0x2e8, 0x2f0, 0x2f8, 0x300, 0x308, 0x310, 0x318, 0x320, 0x328,
		0x330, 0x350, 0x360, 0x3a0, 0x3a8, 0x3b0, 0x3b8, 0x3c0, 0x3c8, 0x3d0,
		0x3d8, 0x3e0, 0x408, 0x410, 0x418, 0x420, 0x428, 0x440, 0x450, 0x460,
		0x4d8, 0x4e0, 0x4e8, 0x4f0, 0x518, 0x520, 0x560, 0x568, 0x578, 0x580,
		0x598, 0xc00, 0x4020, 0x4030, 0x4048, 0x4050, 0x8000, 0x8008, 0x8010, 0x8018,
		0x8020, 0xc000, 0xc008, 0xc010, 0x10000, 0x10068,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(off); i++)
		if (off[i] == offset)
			return true;
	return false;
}

/* The 38 ps_all words macOS has ON at its commit (same end state, actual state != 0) */
static bool t8142_macos_on_at_commit(u32 offset)
{
	static const u32 on[] = {
		0x110, 0x118, 0x120, 0x138, 0x140, 0x148, 0x150, 0x158, 0x160, 0x168,
		0x170, 0x178, 0x180, 0x2c8, 0x2d0, 0x340, 0x348, 0x358, 0x368, 0x370,
		0x378, 0x380, 0x388, 0x390, 0x398, 0x430, 0x438, 0x448, 0x458, 0x468,
		0x470, 0x478, 0x480, 0x488, 0x490, 0x498, 0x4a0, 0x5a0,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(on); i++)
		if (on[i] == offset)
			return true;
	return false;
}

/*
 * Neither: never touched in macOS's trace. Past the CPU words (< 0x60), all but ATC2/3 are no_ps
 * children of the displays and ISP, which their firmware lowers, not the AP: one left on by a
 * cold-stopped DCP keeps its FE word up.
 */
static bool t8142_macos_untraced(u32 offset)
{
	return offset >= 0x60 && !t8142_macos_on_at_commit(offset) &&
	       !t8142_macos_off_at_commit(offset);
}

static void t8142_commit_digest(struct apple_sleep *s)
{
	const struct apple_sleep_words *all = &s->soc->ps_all, *aon = &s->soc->ps_aon_all;
	u8 noauto = 0, untraced = 0, first[1] = { 0xff }, aon_noauto = 0, aon_first = 0xff;
	unsigned int i;
	u32 v;

	for (i = 0; i < all->count; i++) {
		v = readl(s->ps + all->offset[i]);
		if (!FIELD_GET(PS_ACTUAL, v))
			continue;
		/* +10/+11: untraced words on (count, first index): see t8142_macos_untraced */
		if (t8142_macos_untraced(all->offset[i])) {
			if (!untraced)
				first[0] = min(i, 254U);
			untraced += untraced < 255;
		}
		if (v & PS_AUTO_ENABLE)
			continue;
		noauto += noauto < 255;
	}
	for (i = 0; i < aon->count; i++) {
		v = readl(s->ps_aon + aon->offset[i]);
		if (!FIELD_GET(PS_ACTUAL, v) || (v & PS_AUTO_ENABLE))
			continue;
		if (!aon_noauto)
			aon_first = min(i, 254U);
		aon_noauto += aon_noauto < 255;
	}
	v = readl(s->ps + T8142_PS_SEP);
	sleep_note_at(s, 8, noauto);
	sleep_note_at(s, 10, untraced);
	sleep_note_at(s, 11, first[0]);
	sleep_note_at(s, 12, aon_noauto);
	sleep_note_at(s, 13, aon_first);
	sleep_note_at(s, 14, v & 0xff);
	sleep_note_at(s, 15, v >> 24);

	/*
	 * +5..+7, +16..+19: the first seven ps_all indices on here (actual state, AUTO_ENABLE or
	 * not) that macOS has off at its commit (t8142_macos_off_at_commit); 0xff = none. +9 their
	 * count (replaces the AUTO count). The full list is the dry run's.
	 */
	{
		static const u8 slot[] = { 5, 6, 7, 16, 17, 18, 19 };
		unsigned int nodd = 0;

		for (i = 0; i < all->count; i++) {
			v = readl(s->ps + all->offset[i]);
			if (!FIELD_GET(PS_ACTUAL, v) || !t8142_macos_off_at_commit(all->offset[i]))
				continue;
			if (nodd < ARRAY_SIZE(slot))
				sleep_note_at(s, slot[nodd], min(i, 254U));
			nodd++;
		}
		for (i = nodd; i < ARRAY_SIZE(slot); i++)
			sleep_note_at(s, slot[i], 0xff);
		sleep_note_at(s, 9, min(nodd, 255U));
	}
}

static void t8142_commit_note(struct apple_sleep *s)
{
	u8 flags = 0, src = 0xff;
	int i;

	if (!sleep_notes_on(s))
		return;
	if (s->cpus_stop_ret)
		flags |= 0x01;
	if (s->perf_soc_ret)
		flags |= 0x02;
	if (s->perf_dcs_ret)
		flags |= 0x04;
	for (i = 0; i < s->soc->wake_words; i++) {
		u32 pend = readl(s->wake + s->soc->wake_status + i * 4) & s->wake_armed[i];

		if (!pend)
			continue;
		if (!(flags & 0x08)) {
			u32 id = i * 32 + __ffs(pend);

			src = id & 0xff;
			if (id > 0xff)
				flags |= 0x10;
		}
		flags |= 0x08;
	}
	if (readl(s->commit))
		flags |= 0x20;
	t8142_commit_digest(s);
	t8142_alarm_prepare(s);
	/* +20: the alarm read (1 = ready, 0 = off or failed) */
	sleep_note_at(s, 20, t8142_alarm_ok);
	/* +21: mem_boot (bit 0 asked, bit 1 replayed, bit 2 floor stepped), +22 fingerprint mismatches */
	sleep_note_at(s, 21, (READ_ONCE(sleep_mem_boot) ? 1 : 0) | (s->mem_boot_done ? 2 : 0) |
			     (s->mem_floor_done ? 4 : 0) | (s->pmp_lowered ? 8 : 0) |
			     (s->pmp_off_ret ? 0x10 : 0) |
			     (READ_ONCE(sleep_extra_off) ? 0x20 : 0) |
			     (s->extra_lowered || s->disp_adma_lowered ? 0x40 : 0) |
			     (s->extra_stuck ? 0x80 : 0));
	/* +22: dispext (sleep_display_off) */
	sleep_note_at(s, 2, flags);
	sleep_note_at(s, 4, src);
	sleep_note(s, 0x65);
}

/*
 * T8142's memory side, as macOS steps it down 4.6 to 2.7 ms before the S2R timer (J813, 27.2.1,
 * all ten cores: gig3m's trace, gist 89aedeca, 2026-11-15; the J815 ADT has the same mcc
 * layout). The PMGR words around it match our J815 trace write for write.
 *
 * - AMCC: each plane's +0x28000 word (mapped as "amcc<n>") holds 0x1fe; macOS clears it two
 *   bits at a time, in the order below (plane, bit pair), 64-bit read-modify-writes. Whatever
 *   is left on a plane macOS didn't list (the fourth, on J813) goes the same way, low bit first.
 * - SoC NI (mcc reg[2..7], "socni<n>"): each table zeroed from its first word, as many as
 *   macOS writes.
 * - DCS channels (reg[10..17] + 0x580, "mcc<n>"): the enables down a bit at a time, low bit
 *   first; then two lanes per channel per pass, as T8140's, with +0x680 rewritten before each
 *   lane and only the lanes that are enabled (J813's fifth channel has lanes 0 and 2).
 */
#define T8142_AMCC_SLEEP_BITS	GENMASK_ULL(8, 1)
#define T8142_MCC_LANE_EXTRA	0x100
#define T8142_MEM_CLK_SLEEP	0x30000

/*
 * What T8142's deep sleep loses and XNU writes back at the wake, before the AMCC/MCC and
 * clkgen drivers' state is used again (Thomas's J813 trace, 27.2, all ten cores, 2026-11-16;
 * the J815 ADT has the same mcc and clkgen layout). Every one of the 411 memory-side words
 * XNU writes at the wake ends at the value it held before the entry's stop, and the clkgen
 * slots at the values it read 3.2 ms before the timer: so this saves J815's own values and
 * replays them in XNU's order. Nothing here is a J813 constant but the clkgen slot commands.
 */
/*
 * The AMCC sleep bits by (plane, pair), the pair's bits being 2 * pair + 1 and 2 * pair + 2:
 * the order XNU clears them in at the entry and sets them again at the wake.
 */
static const u8 t8142_amcc_order[][2] = {
	{ 1, 0 }, { 2, 2 }, { 1, 2 }, { 0, 1 }, { 1, 3 }, { 0, 0 },
	{ 1, 1 }, { 2, 3 }, { 0, 3 }, { 2, 1 }, { 0, 2 }, { 2, 0 },
};

#define T8142_CLK_SLOTS		126
#define T8142_CLK_SLOT(n)	((n) * 0x20)
#define T8142_CLK_SLOT_LO	0x10
#define T8142_CLK_SLOT_HI	0x14
/* The command XNU writes to each slot's +0x0 after its value (4, or 5 for these 48 slots). */
static const u64 t8142_clk_slot_cmd5[2] = { 0x54aaaaaaaa000000ULL, 0x02aaaaaaa9555555ULL };
/* Per DCS channel ("mcc<n>" = +0x580): +0x0 enables, +0x4 groups, +0x10.. six words a group */
#define T8142_MCC_SAVE_WORDS	65
#define T8142_MCC_GROUPS	10
#define T8142_MCC_GROUP(k)	(0x10 + (k) * 0x18)
/* Per AMCC plane ("amcc-cfg<n>" = plane + 0x2a000): 64-bit +0x0, +0x8, +0x10; 32-bit +0x4000 */
#define T8142_AMCC_CFG_CTL	0x4000
#define T8142_SOCNI_STROBE	0x100

static void t8142_mem_save(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i, n;

	s->mem_saved = false;
	if (!READ_ONCE(sleep_mem_restore))
		return;
	for (i = 0; i < soc->namcc; i++) {
		s->amcc_saved[i] = readq(s->amcc[i]);
		if (!s->amcc_cfg[i])
			continue;
		for (n = 0; n < 3; n++)
			s->amcc_cfg_saved[i][n] = readq(s->amcc_cfg[i] + 8 * n);
		s->amcc_ctl_saved[i] = readl(s->amcc_cfg[i] + T8142_AMCC_CFG_CTL);
	}
	for (i = 0; i < soc->nsocni; i++)
		for (n = 0; n < soc->socni_words[i] && n < ARRAY_SIZE(s->socni_saved[0]); n++)
			s->socni_saved[i][n] = readl(s->socni[i] + 4 * n);
	for (i = 0; i < soc->nmcc; i++)
		for (n = 0; n < T8142_MCC_SAVE_WORDS; n++)
			s->mcc_saved[i][n] = readl(s->mcc[i] + 4 * n);
	s->mem_saved = true;
}

static void t8142_clk_save(struct apple_sleep *s)
{
	int i;

	s->clk_saved = false;
	if (!s->clk_slots || !READ_ONCE(sleep_clk_restore))
		return;
	for (i = 0; i < T8142_CLK_SLOTS; i++) {
		void __iomem *slot = s->clk_slots + T8142_CLK_SLOT(i);
		u32 hi, lo;
		int tries = 4;

		/* As XNU reads them: high, low, high again (bounded: interrupts are off). */
		do {
			hi = readl(slot + T8142_CLK_SLOT_HI);
			lo = readl(slot + T8142_CLK_SLOT_LO);
		} while (readl(slot + T8142_CLK_SLOT_HI) != hi && --tries);
		s->clk_slot_saved[i][0] = lo;
		s->clk_slot_saved[i][1] = hi;
	}
	s->clk_saved = true;
}

/* XNU's wake at +4.6..8.8 ms: each slot's value, then its command; then the memory clock word. */
static void t8142_clk_restore(struct apple_sleep *s)
{
	int i;

	if (s->clk_saved) {
		for (i = 0; i < T8142_CLK_SLOTS; i++) {
			void __iomem *slot = s->clk_slots + T8142_CLK_SLOT(i);
			bool five = t8142_clk_slot_cmd5[i / 64] & BIT_ULL(i % 64);

			writel(s->clk_slot_saved[i][0], slot + T8142_CLK_SLOT_LO);
			writel(s->clk_slot_saved[i][1], slot + T8142_CLK_SLOT_HI);
			writel(five ? 5 : 4, slot);
		}
		s->clk_saved = false;
	}
	if (s->mem_clk && s->mem_clk_saved && READ_ONCE(sleep_clk_restore))
		writel(s->mem_clk_saved, s->mem_clk);
}

/* One bit at a time, low first, as XNU raises each AMCC pair back. */
static void t8142_amcc_set(void __iomem *reg, u64 bits)
{
	u64 val = readq(reg);
	unsigned int b;

	for_each_set_bit(b, (unsigned long *)&bits, 64) {
		if (val & BIT_ULL(b))
			continue;
		val |= BIT_ULL(b);
		writeq(val, reg);
		val = readq(reg);
	}
}

/* Byte @n of @reg back to its saved value, as XNU does before each lane comes up. */
static void t8142_byte_restore(void __iomem *reg, u32 saved, int n)
{
	u32 mask = GENMASK(8 * n + 7, 8 * n);

	writel((readl(reg) & ~mask) | (saved & mask), reg);
}

/*
 * XNU's wake from +9.06 ms, in its order: the AMCC planes' sleep bits back in the entry's pair
 * order; each SoC NI table word with its strobe; each DCS channel's region groups, each
 * followed by its enable bits; the lanes, two per channel per pass; the AMCC planes' control
 * words. Skipped: XNU's read-and-rewrite of each channel's +0x24 (outside the DT's map).
 */
static int t8142_mem_restore(struct apple_sleep *s, const u8 (*order)[2], int norder)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i, n, ch, pass, k;

	if (!s->mem_saved)
		return -ENODATA;

	for (i = 0; i < norder; i++) {
		int plane = order[i][0];

		if (plane >= soc->namcc)
			continue;
		t8142_amcc_set(s->amcc[plane], s->amcc_saved[plane] &
			       GENMASK_ULL(2 * order[i][1] + 2, 2 * order[i][1] + 1));
	}
	for (i = 0; i < soc->namcc; i++)
		t8142_amcc_set(s->amcc[i], s->amcc_saved[i] & T8142_AMCC_SLEEP_BITS);

	for (i = 0; i < soc->nsocni; i++)
		for (n = 0; n < soc->socni_words[i] && n < ARRAY_SIZE(s->socni_saved[0]); n++) {
			writel(s->socni_saved[i][n], s->socni[i] + 4 * n);
			writel(1, s->socni[i] + T8142_SOCNI_STROBE);
		}

	for (ch = 0; ch < soc->nmcc; ch++) {
		void __iomem *mcc = s->mcc[ch];
		const u32 *sv = s->mcc_saved[ch];
		u32 en = 0, groups = 0;

		for (k = 0; k < T8142_MCC_GROUPS; k++) {
			if (!(sv[MCC_ENABLES / 4] & BIT(k)))
				continue;
			for (n = 0; n < 6; n++)
				writel(sv[(T8142_MCC_GROUP(k) + 4 * n) / 4],
				       mcc + T8142_MCC_GROUP(k) + 4 * n);
			en |= GENMASK(3, 0) | BIT(4 + k);
			groups |= BIT(k);
			writel(readl(mcc) | (sv[0] & en), mcc);
			writel(readl(mcc + MCC_ENABLES) | (sv[MCC_ENABLES / 4] & groups),
			       mcc + MCC_ENABLES);
		}
		/* Whatever else was on, as XNU's last group leaves the words at their old values. */
		if (readl(mcc) != sv[0])
			writel(sv[0], mcc);
		if (readl(mcc + MCC_ENABLES) != sv[MCC_ENABLES / 4])
			writel(sv[MCC_ENABLES / 4], mcc + MCC_ENABLES);
	}

	for (pass = 0; pass < 2; pass++) {
		for (ch = 0; ch < soc->nmcc; ch++) {
			void __iomem *mcc = s->mcc[ch];
			const u32 *sv = s->mcc_saved[ch];

			for (n = 2 * pass; n < 2 * pass + 2; n++) {
				if (!(sv[MCC_LANES / 4] & MCC_LANE(n)))
					continue;
				t8142_byte_restore(mcc + T8142_MCC_LANE_EXTRA,
						   sv[T8142_MCC_LANE_EXTRA / 4], n);
				t8142_byte_restore(mcc + MCC_LANES_MASK, sv[MCC_LANES_MASK / 4], n);
				writel(readl(mcc + MCC_LANES) | MCC_LANE_STROBE(n), mcc + MCC_LANES);
				writel(readl(mcc + MCC_LANES) | MCC_LANE(n), mcc + MCC_LANES);
			}
		}
	}
	for (ch = 0; ch < soc->nmcc; ch++) {
		void __iomem *mcc = s->mcc[ch];
		const u32 *sv = s->mcc_saved[ch];

		if (readl(mcc + T8142_MCC_LANE_EXTRA) != sv[T8142_MCC_LANE_EXTRA / 4])
			writel(sv[T8142_MCC_LANE_EXTRA / 4], mcc + T8142_MCC_LANE_EXTRA);
		if (readl(mcc + MCC_LANES_MASK) != sv[MCC_LANES_MASK / 4])
			writel(sv[MCC_LANES_MASK / 4], mcc + MCC_LANES_MASK);
	}

	/* +14.56 ms: plane 0's +0x10, then each plane's +0x4000, +0x8, +0x0. */
	for (i = 0; i < soc->namcc; i++) {
		void __iomem *cfg = s->amcc_cfg[i];

		if (!cfg)
			continue;
		if (i == 0)
			writeq(s->amcc_cfg_saved[i][2], cfg + 0x10);
		writel(s->amcc_ctl_saved[i], cfg + T8142_AMCC_CFG_CTL);
		writeq(s->amcc_cfg_saved[i][1], cfg + 0x8);
		writeq(s->amcc_cfg_saved[i][0], cfg);
	}
	s->mem_saved = false;
	return 0;
}

static void t8142_amcc_clear(void __iomem *reg, u64 bits)
{
	u64 val = readq(reg);
	unsigned int b;

	for_each_set_bit(b, (unsigned long *)&bits, 64) {
		if (!(val & BIT_ULL(b)))
			continue;
		val &= ~BIT_ULL(b);
		writeq(val, reg);
		val = readq(reg);
	}
}

static void t8142_sleep_mcc_stop(struct apple_sleep *s)
{
	const u8 (*order)[2] = t8142_amcc_order;
	const struct apple_sleep_soc *soc = s->soc;
	int i, plane, ch, pass, n;

	/* What the wake puts back (t8142_mem_restore), before anything here changes it. */
	t8142_mem_save(s);

	for (i = 0; i < ARRAY_SIZE(t8142_amcc_order); i++) {
		if (order[i][0] >= soc->namcc)
			continue;
		t8142_amcc_clear(s->amcc[order[i][0]], GENMASK_ULL(2 * order[i][1] + 2,
								 2 * order[i][1] + 1));
	}
	for (plane = 0; plane < soc->namcc; plane++)
		t8142_amcc_clear(s->amcc[plane], T8142_AMCC_SLEEP_BITS);

	for (i = 0; i < soc->nsocni; i++)
		for (n = 0; n < soc->socni_words[i]; n++)
			writel(0, s->socni[i] + 4 * n);

	for (ch = 0; ch < soc->nmcc; ch++)
		t8132_mcc_step_down(s->mcc[ch] + MCC_ENABLES);

	for (pass = 0; pass < 2; pass++) {
		for (ch = 0; ch < soc->nmcc; ch++) {
			void __iomem *mcc = s->mcc[ch];

			for (n = 2 * pass; n < 2 * pass + 2; n++) {
				if (!(readl(mcc + MCC_LANES) & MCC_LANE(n)))
					continue;
				writel(readl(mcc + T8142_MCC_LANE_EXTRA), mcc + T8142_MCC_LANE_EXTRA);
				writel(readl(mcc + MCC_LANES_MASK), mcc + MCC_LANES_MASK);
				writel(readl(mcc + MCC_LANES) | MCC_LANE_STROBE(n), mcc + MCC_LANES);
				writel(readl(mcc + MCC_LANES) & ~MCC_LANE(n), mcc + MCC_LANES);
			}
		}
	}

	if (s->mem_clk) {
		s->mem_clk_saved = readl(s->mem_clk);
		writel(T8142_MEM_CLK_SLEEP, s->mem_clk);
	}
	/* macOS reads the clkgen slots right after the clock word (-3.2 ms). */
	t8142_clk_save(s);
}

/*
 * T8132's order after the cores stopped, ATC0 and ANS lowered and WD2: quiescePerfState, the
 * CPM gates around the AP's PMC vote (agent 3, as T8132's), the complexes off, the quiesce
 * read, the early wake-up bits off, the words saved.
 */
static void t8142_last_mile(struct apple_sleep *s)
{
	const struct apple_sleep_soc *soc = s->soc;
	int i;

	/* Not in macOS's J813 entry (its perf-state page is untouched in the whole trace): A/B. */
	if (READ_ONCE(sleep_perf_down))
		t8142_perf_down(s);
	sleep_note(s, 0x64);

	/* T6050's commit window: the floor's voter to 0, not read, right before the memory stop */
	if (s->mem_floor_done)
		writel(0, s->pmc_voters + PMC_VOTER_STRIDE * 1);
	/* The memory controller, after the SoC and DCS states, as on T8132 (and macOS on J813). */
	t8142_sleep_mcc_stop(s);
	sleep_note(s, 0x66);

	t8142_cpm_on(s);
	t8142_cpm_on(s);
	readl(s->pmc_voters + T8142_PMC_VOTER_AP);
	writel(0, s->pmc_voters + T8142_PMC_VOTER_AP);
	readl(s->pmc_voters + T8142_PMC_VOTER_AP + 4);
	t8142_cpm_on(s);

	t8142_commit_note(s);

	for (i = 0; i < soc->ncluster_cpu; i++)
		writel(0, s->cluster + soc->cluster_cpu[i]);

	s->quiesce_still_on = 0;
	for (i = 0; i < soc->quiesce.count; i++)
		if (sleep_ps_on(s, soc->quiesce.offset[i]))
			s->quiesce_still_on |= BIT(i);

	/* "Applying Sleep hacks": the early wake-up bits off, right before the S2R timer. */
	for (i = 0; i < soc->ncllt; i++)
		writel(readl(s->cllt + soc->cllt[i].offset) & ~soc->cllt[i].clear,
		       s->cllt + soc->cllt[i].offset);

	sleep_ps_save(s);
}

/*
 * Note 0x67 (replaces 0x66 once every read came back): +2 E cores' PU bits [5:0], bit 6 complex
 * E busy, bit 7 complex P busy; +3 P cores' PU bits [3:0], bit 4 commit word nonzero, bit 5 an
 * armed wake pending, bit 6 E asked on, bit 7 P asked on; +4 E cores' SPD bits [5:0], bit 6 any
 * P core's SPD, bit 7 a window read all-ones (no answer). Unmapped windows read as PU 1.
 */
static void t8142_post_commit_ed(u8 b2_0x66)
{
	u8 b2 = 0, b3 = 0, b4 = 0;
	int k;

	for (k = 0; k < T8142_ED_CORES; k++) {
		u32 v;

		if (k < 6) {
			v = t8142_ed[0][k] ? readl_relaxed(t8142_ed[0][k] + T8142_ED_EDPRSR) : 1;
			if (v == ~0U)
				b4 |= BIT(7);
			if (v & BIT(0))
				b2 |= BIT(k);
			if (v & BIT(1))
				b4 |= BIT(k);
		}
		if (k < 4) {
			v = t8142_ed[1][k] ? readl_relaxed(t8142_ed[1][k] + T8142_ED_EDPRSR) : 1;
			if (v == ~0U)
				b4 |= BIT(7);
			if (v & BIT(0))
				b3 |= BIT(k);
			if (v & BIT(1))
				b4 |= BIT(6);
		}
	}
	b2 |= (b2_0x66 & BIT(3) ? BIT(6) : 0) | (b2_0x66 & BIT(4) ? BIT(7) : 0);
	b3 |= (b2_0x66 & BIT(0) ? BIT(4) : 0) | (b2_0x66 & BIT(5) ? BIT(5) : 0) |
	      (b2_0x66 & BIT(1) ? BIT(6) : 0) | (b2_0x66 & BIT(2) ? BIT(7) : 0);
	sleep_note_at(apple_sleep, 2, b2);
	sleep_note_at(apple_sleep, 3, b3);
	sleep_note_at(apple_sleep, 4, b4);
	sleep_note(apple_sleep, 0x67);
}

/*
 * apple_sleep.post_commit_ms: note 0x66, decoded by aurora-bootlog as b43's was.
 * +2: bit 0 commit word nonzero, 1/2 complex E/P asked on, 3/4 E/P busy, 5 an armed wake source
 *     pending. +3: SoC perf state [3:0], bit 4 DCS busy, bit 5 SoC busy. +4: DCS state [3:0],
 *     bits 4..7 PMC voters PMP/CLPC/GFX/AP nonzero.
 */
static void t8142_post_commit_probe(struct apple_sleep *s, unsigned int ms)
{
	const struct apple_sleep_soc *soc = s->soc;
	u32 e, p, soc_perf = 0, dcs = 0;
	u8 b2 = 0, b3, b4;
	int i;

	mdelay(ms);
	e = readl(s->cluster + soc->cluster_cpu[0]);
	p = readl(s->cluster + soc->cluster_cpu[1]);
	if (readl(s->commit))
		b2 |= BIT(0);
	if (e & CLUSTER_ON)
		b2 |= BIT(1);
	if (p & CLUSTER_ON)
		b2 |= BIT(2);
	if (e & T8142_CLUSTER_BUSY)
		b2 |= BIT(3);
	if (p & T8142_CLUSTER_BUSY)
		b2 |= BIT(4);
	for (i = 0; i < soc->wake_words; i++)
		if (readl(s->wake + soc->wake_status + 4 * i) & s->wake_armed[i])
			b2 |= BIT(5);
	if (s->clk_perf)
		soc_perf = readl(s->clk_perf + CLK_PERF_SOC);
	if (s->dcs)
		dcs = readl(s->dcs);
	b3 = FIELD_GET(DCS_STATE, soc_perf) | (dcs & DCS_BUSY ? BIT(4) : 0) |
	     (soc_perf & DCS_BUSY ? BIT(5) : 0);
	b4 = FIELD_GET(DCS_STATE, dcs);
	for (i = 0; s->pmc_voters && i < 4; i++)
		if (readl(s->pmc_voters + i * PMC_VOTER_STRIDE))
			b4 |= BIT(4 + i);
	sleep_note_at(s, 2, b2);
	sleep_note_at(s, 3, b3);
	sleep_note_at(s, 4, b4);
	sleep_note(s, 0x66);
	if (READ_ONCE(sleep_post_commit_ed))
		t8142_post_commit_ed(b2);
}

/*
 * AppleT8142PMGR's complex power-on: ApplePMGR's cluster method writes (word & ~0xf) | 0xf and
 * waits for bit 4 to clear; T8142's override then writes 1 to the complex's ACC word.
 */
static int t8142_cluster_on(struct apple_sleep *s, u32 word, void __iomem *acc)
{
	void __iomem *reg = s->cluster + word;
	u32 val;
	int ret;

	writel(readl(reg) | CLUSTER_ON, reg);
	ret = readl_poll_timeout_atomic(reg, val, !(val & T8142_CLUSTER_BUSY), 1,
					CLUSTER_TIMEOUT_US);
	if (!ret && acc && READ_ONCE(sleep_acc_wake))
		writel(1, acc);
	return ret;
}

/*
 * AppleT8142PMGR::restoreHW: each complex on, pwrgate +0x0 and +0x60 read-modify-written, the
 * words back, the CPM gates, the ACC words once more. Its two "early wake hacks" (SEP's ps
 * word, map 0x4e's ten words) are gated by flags its constructor clears and nothing sets.
 * Then T8132's: ATC0's words, SEP, WD2, ANS's PCIe block and the radio's link.
 */
static void t8142_wake(struct apple_sleep *s)
{
	if (t8142_alarm_ok)
		pr_info("apple-sleep: T8142 PMU alarm after the timer: %d (counter %llu, compare %u)\n",
			t8142_post_ret, t8142_alarm_counter, get_unaligned_le32(t8142_alarm));

	const struct apple_sleep_soc *soc = s->soc;
	u32 val;
	int i;

	s->cluster_e_ret = t8142_cluster_on(s, soc->cluster_cpu[0], s->acc_e);
	s->cluster_p_ret = t8142_cluster_on(s, soc->cluster_cpu[1], s->acc_p);

	for (i = 0; i < ARRAY_SIZE(soc->pwrgate_wake); i++) {
		void __iomem *reg = s->pwrgate + soc->pwrgate_wake[i];

		writel(readl(reg) | PWRGATE_ON, reg);
	}

	sleep_ps_rewake(s);
	/* Off at the commit by pmp_off, so the rewake left it: back as lower_io found it. */
	if (s->pmp_lowered) {
		s->pmp_lowered = false;
		sleep_ps_restore(s->ps + T8142_PS_PMP, s->pmp_saved);
	}
	/* Off at the commit by dispext, likewise: back before the display drivers resume. */
	sleep_dispext_restore(s);
	t8142_extra_restore(s);
	/* The commit saved ATC0's words already lowered: reverse of the lowering. */
	sleep_ps_restore(s->ps + T8142_PS_ATC0_COMMON, s->atc0_saved[2]);
	sleep_ps_restore(s->ps_aon + T8142_PS_AON_ATC0_USB_AON, s->atc0_saved[1]);
	sleep_ps_restore(s->ps_aon + T8142_PS_AON_ATC0_USB, s->atc0_saved[0]);

	s->deep_ran = sleep_ps_woke_on(s, soc->ps.ap_tmm);

	val = readl(s->ps + soc->ps.sep);
	if (val & PS_DEV_DISABLE)
		writel(val & ~PS_DEV_DISABLE, s->ps + soc->ps.sep);

	/* XNU's order: the clkgen slots and the memory clock, the CPM gates, the memory side. */
	t8142_clk_restore(s);
	t8142_cpm_on(s);
	s->mem_restore_ret = t8142_mem_restore(s, t8142_amcc_order, ARRAY_SIZE(t8142_amcc_order));
	if (READ_ONCE(sleep_acc_wake) && s->acc_e && s->acc_p) {
		writel(1, s->acc_e);
		writel(1, s->acc_p);
	}

	/* macOS's wake is not traced: back to what the entry read, before WD2's re-arm. */
	if (s->aon_wdt_marked) {
		writel(s->aon_wdt_saved, s->aon_misc + AON_WDT_SLEEP);
		s->aon_wdt_marked = false;
	}
	apple_sleep_set_wd2(s, WDT_SUSPEND_SECS);

	if (s->pcie_ans_stopped)
		s->pcie_ans_on_ret = apple_sleep_raise_apcie(s);
	/* Before the PCI core restores the radio's functions, which needs the link. */
	if (s->radio_trainable)
		s->radio_train_ret = s->pcie_ans_stopped && !s->pcie_ans_on_ret ?
				     apple_sleep_radio_train(s) : -ENOLINK;
	s->apcie_port_down = true;
	s->pins_lost = s->pins_saved_valid;
}

/* The J815 ADT's /arm-io/pmgr ap-wake-sources, all of them. */
static const struct apple_sleep_wake_name t8142_wake_names[] = {
	{ 5, "AOP.CPUWakeupAP" }, { 12, "AOP.GPIO.IRQ4" }, { 14, "AOP.GPIO.IRQ6" },
	{ 44, "AOP.OutboxNotEmpty" }, { 63, "AOP.SPMI0.SW3" }, { 67, "AOP.SPMI1.SW3" },
	{ 71, "ATC0.CIOWakeup" }, { 72, "ATC0.USBWakeup" }, { 73, "ATC1.CIOWakeup" },
	{ 74, "ATC1.USBWakeup" }, { 75, "ATC2.CIOWakeup" }, { 76, "ATC2.USBWakeup" },
	{ 77, "ATC3.CIOWakeup" }, { 78, "ATC3.USBWakeup" },
	{ 89, "MTP.DOCK.CHANNELS.AP0.IRQ" }, { 108, "MTP.OutboxNotEmpty" },
	{ 128, "NUB.APWakeupTime" }, { 129, "PMGR_AON.APWatchdog" }, { 146, "NUB.GPIO.IRQ6" },
	{ 152, "NUB.SPMI0.SW3" }, { 157, "NUB.SPMI1.SW3" }, { 163, "NUB.WakeupAP" },
	{ 168, "SMC.CPUWakeupAP" }, { 190, "SMC.OutboxNotEmpty" },
	{ 237, "AOP.TrustedOutboxNotEmpty" }, { 256, "AOP2.TrustedOutboxNotEmpty" },
	{ 261, "AOP2.CPUWakeAP" },
};

/*
 * Every word in the J815 ADT's /arm-io/pmgr power states, in "ps" and in "ps-aon". The lists
 * after them are T8132's measured ones, each word translated to T8142's by its ADT name; a
 * T8132 word with no T8142 namesake is dropped.
 */
/*
 * Less DCS0..7 (0x1d8, 0x1e0, 0x1e8, 0x1f0, 0x218, 0x220, 0x228, 0x230): DRAM slices, no_ps in
 * the J815 ADT and left to the memory controller. Reading or writing one at .enter stalled J815
 * (boot 45257eaf, DCS5 in the match), so neither the dry run's report, the commit's save nor the
 * rewake after a wake touches them.
 */
/*
 * Also less the 34 words that are OS-managed on T8132 (or absent there) but no_ps in the J815
 * ADT (PCPM_SRAM, SBR, PMS(_BUSIF), PMGR_SOC_OCLA, PIOGW, the AF* fabrics and AMCC ports, SIO,
 * FPWM1, TRACE_FAB, DPTX_PHY, ISP_CPU*, ISP_FE, DISP_SMB*): the rewake after a wake writes back
 * any word of this list whose control bits moved, and on T8142 these are firmware's to move
 * (rule: t8142_ps_macos_off).
 */
static const u32 t8142_ps_all[] = {
	0x8, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50,
	0x58, 0x110, 0x118, 0x120, 0x138, 0x140, 0x148, 0x150, 0x158, 0x160,
	0x168, 0x170, 0x178, 0x180, 0x190, 0x198, 0x1a0, 0x1a8, 0x1b0, 0x290,
	0x298, 0x2a0, 0x2a8, 0x2b8, 0x2c0, 0x2c8, 0x2d0, 0x2d8, 0x2e0, 0x2e8,
	0x2f0, 0x2f8, 0x300, 0x308, 0x310, 0x318, 0x320, 0x328, 0x330, 0x340,
	0x348, 0x350, 0x358, 0x360, 0x368, 0x370, 0x378, 0x380, 0x388, 0x390,
	0x398, 0x3a0, 0x3a8, 0x3b0, 0x3b8, 0x3c0, 0x3c8, 0x3d0, 0x3d8, 0x3e0,
	0x3e8, 0x3f0, 0x3f8, 0x400, 0x408, 0x410, 0x418, 0x420, 0x428, 0x430,
	0x438, 0x440, 0x448, 0x450, 0x458, 0x460, 0x468, 0x470, 0x478, 0x480,
	0x488, 0x490, 0x498, 0x4a0, 0x4a8, 0x4b0, 0x4b8, 0x4c0, 0x4c8, 0x4d0,
	0x4d8, 0x4e0, 0x4e8, 0x4f0, 0x4f8, 0x500, 0x508, 0x510, 0x518, 0x520,
	0x528, 0x530, 0x538, 0x540, 0x548, 0x550, 0x558, 0x560, 0x568, 0x570,
	0x578, 0x580, 0x598, 0x5a0, 0xc00, 0x4020, 0x4028, 0x4030, 0x4038, 0x4040,
	0x4048, 0x4050, 0x4058, 0x4060, 0x8000, 0x8008, 0x8010, 0x8018, 0x8020, 0xc000,
	0xc008, 0xc010, 0x10000, 0x10008, 0x10010, 0x10018, 0x10020, 0x10028, 0x10030, 0x10038,
	0x10040, 0x10048, 0x10050, 0x10068, 0x10070, 0x10078,
};

/*
 * Less the 9 AON words T8142 took from the OS (no_ps here, OS-managed or absent on T8132):
 * NUB_AON, NUB_OCLA, NUB_FABRIC, NUB_SRAM, DEBUG_SWITCH, AOP_DART, AOP_I2CM3, AOP2_PIODMA,
 * AOP2_CPU. Same reason as t8142_ps_all.
 */
static const u32 t8142_ps_aon_all[] = {
	0x58, 0x60, 0x68, 0x70, 0x78, 0x88, 0x98, 0xa0, 0xa8, 0xc8,
	0xd0, 0xd8, 0xe0, 0xe8, 0xf0, 0xf8, 0x100, 0x4000, 0x4008, 0x4010,
	0x4018, 0x4028, 0x4030, 0x4038, 0x4048, 0x4050, 0x4058, 0x4060, 0x4068, 0x4070,
	0x4078, 0x4080, 0x4088, 0x4090, 0x40a0, 0x40a8, 0x40b8, 0x40c0, 0x40c8, 0x40d0,
	0x40d8, 0x40e0, 0x40e8, 0x40f0, 0x40f8, 0x4100, 0x4108, 0x4110, 0x4118, 0x4120,
	0x4128, 0x4130, 0x4138, 0x4140, 0x4148, 0x4150, 0x4158, 0x4160, 0x4168, 0x4170,
	0x4178, 0x4180, 0x4188, 0x8000, 0x8008, 0x8010, 0x8018, 0x8020, 0x8028, 0x8030,
	0x8038, 0x8040, 0x8048, 0x8050, 0x8058, 0x8060, 0x8068, 0x8070, 0x8078, 0x8080,
	0x8088, 0xc000, 0xc008, 0xc010, 0xc018, 0xc020, 0xc028, 0xc030, 0xc038, 0xc040,
	0xc048, 0xc050, 0xc058, 0xc060,
};

/*
 * XNU's own wake order on T8142 (J813 27.2 trace, 2026-11-16: the 22 words it raises with
 * AUTO_ENABLE right after the clusters and SEP), then the rest of the T8132-derived list. A
 * word is only raised if Linux had it on at the commit (sleep_ps_rewake).
 */
static const u32 t8142_ps_wake_order[] = {
	0xc00, 0x118, 0x120, 0x138, 0x170, 0x178, 0x180, 0x348, 0x358, 0x368,
	0x390, 0x398, 0x430, 0x438, 0x448, 0x458, 0x468, 0x470, 0x478, 0x480,
	0x488, 0x5a0,
	0x598, 0x300, 0x2f0, 0x408, 0x410, 0x520, 0x560, 0x328, 0x2a8, 0x320,
	0x2b8, 0x3a8, 0x3b8, 0x290, 0x318, 0x10000, 0x308, 0x418, 0x428, 0x440,
	0x360,
};

/* XNU's PMGR start at boot (J813 trace g01, -218649 ms): put on AUTO_ENABLE, in its order */
static const u32 t8142_ps_boot_auto[] = {
	0x118, 0x120, 0x138, 0x140, 0x170, 0x178, 0x180, 0x290, 0x2b8, 0x2c8,
	0x2d0, 0x2f0, 0x300, 0x308, 0x328, 0x348, 0x350, 0x358, 0x360, 0x368,
	0x390, 0x398, 0x3a0, 0x3a8, 0x410, 0x418, 0x430, 0x438, 0x440, 0x448,
	0x450, 0x458, 0x460, 0x468, 0x470, 0x478, 0x480, 0x488, 0x578, 0x598,
	0x5a0,
};

static const u32 t8142_ps_aon_boot_auto[] = {
	0x58, 0x60, 0x68, 0x70, 0x78, 0x88, 0x98, 0xa0, 0xa8, 0xc8,
	0xd0, 0xd8, 0xe0,
};

static const u32 t8142_ps_aon_wake_order[] = {
	0x88, 0x98, 0xa0, 0xa8,
};

/*
 * T8132's match raises its AFIMCCNI0/1, FAB_AFC0, AMCC0/1, DCS5 and DCS7 (and DWI). On T8142 every
 * one of those, by name or by its split (AFCMCCNI0, AMCC0_P0..P3), is no_ps in the J815 ADT: the
 * memory path's words belong to firmware there, and J815 stalled the first time a sleep wrote
 * one (DCS5, boot 45257eaf). T8142 raises none; see t8142_ps_macos_off for the rule.
 */

/*
 * T8132's lowered words by name, less those T8142 took out of the OS's hands: ISP_FE,
 * ISP_CPU_CORE1/0, ISP_CPU, DPTX_PHY, TRACE_FAB, FPWM1 and AFIAFT are OS-managed on T8132 but
 * no_ps in the J815 ADT (FPWM1 next to T8142's own no_ps SMC_FPWM0..3). J815 stalled lowering
 * FPWM1 (boot b7f0f04d: PMU note 0x47, phase 5, index 44). The rule for T8142's sleep lists:
 * no word the J815 ADT marks no_ps, unless T8132 writes the same no_ps word (its AON audio ones).
 */
static const u32 t8142_ps_macos_off[] = {
	0xc010, 0xc008, 0xc000, 0x8020, 0x8018, 0x8010, 0x8008, 0x8000, 0x510, 0x508,
	0x500, 0x4f8, 0x4f0, 0x4e8, 0x4e0, 0x4d8, 0x460, 0x458, 0x450, 0x448,
	0x440, 0x438, 0x430, 0x400, 0x3f8, 0x3f0, 0x3e8, 0x3e0, 0x3d8, 0x3d0,
	0x3c8, 0x3c0, 0x3a0, 0x368, 0x360, 0x358, 0x350, 0x348, 0x320, 0x310,
	0x2f8, 0x2e8, 0x2e0, 0x2d8, 0x2d0, 0x2c0, 0x2a8, 0x2a0, 0x298, 0x1b0,
	0x198, 0x190, 0x178, 0x170,
};

static const u32 t8142_ps_aon_macos_off[] = {
	0x4180, 0x4108, 0x40f8, 0x40d8, 0x40b8, 0x4080, 0x100, 0xf8, 0xf0, 0xe8,
	0xe0, 0xd8,
};

/*
 * T8142 has no ps_aon_macos_on list. These audio domains are no_ps in its
 * ADT and remain at actual/target zero in the J813 macOS sleep snapshot;
 * the traced AP never accesses them. The T8132 state-4 rule does not apply.
 */

/*
 * T8132's quiesce words by name: the ATC commons, the display's and DISPEXT's SYS, ANS, AVD,
 * VENC, JPG and APCIE's SYS_GP. GFX's is in its own block here.
 */
static const u32 t8142_quiesce_words[] = {
	0x2c8, 0x2d0, 0x2d8, 0x2e0, 0x290, 0x308, 0x2b8, 0x300, 0x310, 0x298,
	0x2a0, 0x408,
};
static_assert(ARRAY_SIZE(t8142_quiesce_words) <= 32);

/* In t8142_ps_macos_off / t8142_ps_aon_macos_off, but on at macOS 27.2's commit (apple_sleep.ps_keep) */
static const struct apple_sleep_ps_keep t8142_ps_keep[] = {
	{ "PMS_C1PPT", 0x170, false, 0 },	{ "SOC_RC", 0x178, false, 0 },
	{ "I2C0", 0x348, false, 1 },		{ "I2C2", 0x358, false, 1 },
	{ "I2C4", 0x368, false, 1 },		{ "SPI0", 0x430, false, 1 },
	{ "SPI1", 0x438, false, 1 },		{ "SPI3", 0x448, false, 1 },
	{ "SPI5", 0x458, false, 1 },
	{ "ATC1_COMMON", 0x2d0, false, 2 },	{ "ATC1_USB_AON", 0xd8, true, 2 },
	{ "ATC1_USB", 0xe0, true, 2 },
};

static const struct apple_sleep_word t8142_stray_words[] = {
	{ "AVD_SYS", 0x310 }, { "VENC_SYS", 0x298 }, { "JPG", 0x2a0 },
};

static const struct apple_sleep_word t8142_gate_off[] = {
	{ "ane_sys", 0x2e8 }, { "isp_sys", 0x2f8 }, { "avd_sys", 0x310 },
	{ "jpg", 0x2a0 }, { "venc_sys", 0x298 },
};

/* T8142's, from the J815 tree. GFX's word is in its own block, not "ps". */
static const struct apple_sleep_gate_coproc t8142_gate_coprocs[] = {
	{ "/soc/dcp@286e00000", true, 0x10000 },
	{ "/soc/dcp@506e00000", true, 0x428 },
	{ "/soc/dcp@3aae00000", true, 0x3b8 },
	{ "/soc/pmp@380500000", true },
	{ "/soc/nvme@4c5cc0000" },	{ "/soc/mtp@394600000" },
	{ "/soc/aop@390c00000" },	{ "/soc/smc@38c600000" },
	{ "/soc/sep@30a600000" },
	{ "/soc/gpu@582600000", true },
};


static const struct apple_sleep_ops t8142_sleep_ops = {
	/* XNU's last-WFI helper is the same code as T8132's and T8140's; plus M5's power-down bits. */
	.deep_wfi_arm = t8142_deep_wfi_arm,
	.lower_io = t8142_lower_io,
	.last_mile = t8142_last_mile,
	/* WD2 is at T8132's address (/arm-io/wdt), so the same stub restarts it. */
	.resume = t8132_resume_stub,
	.pre_commit = t8132_pre_commit,
	.post_timer = t8142_post_timer,
	.macos_off_at_commit = t8142_macos_off_at_commit,
	.wake = t8142_wake,
	.aic_mask_all = aic_sleep_mask_all,
	.aic_ack = aic_sleep_ack,
	.aic_restore = aic_sleep_restore,
};

/*
 * The J815 /chosen/dart-ps-wr-dis-entries that macOS's SPTM clears during its entry (26B131
 * trace), but the DCP's (ps+0x17fdc bit 6, which the DART driver owns) and DCPEXT0/1's (no
 * external DCP in this tree), in the trace's order. The rest of each word stays set, as on macOS.
 */
static const struct apple_sleep_ps_wr_dis t8142_ps_wr_dis[] = {
	{ "SEP (InvalidateHmac)", 0x17fe0, 12 },
	{ "centauri-beta", 0x17fe8, 10 },
	{ "PMP", 0x17ff8, 17 },
	{ "SIO DMA", 0x17ff0, 28 },
};

/* mcc reg[2..7]: the words macOS zeroes in each SoC NI table (J813 trace) */
static const u8 t8142_socni_words[] = { 15, 16, 28, 30, 21, 14 };

static const char *const t8142_regs_absent[] = {
	"pmc-dcp", "clk-mux", "ptd", "cluster-dcs", "dvfm-e", "dvfm-p", NULL,
};

/* Children before parents (J815 ADT pmgr devices: parent ids) */
static const struct apple_sleep_extra_off t8142_extra_off[] = {
	{ "SYS_GP_OCLA", 0x518, 0x408 },	/* parent APCIE_SYS_GP */
	{ "SYS_ST_OCLA", 0x568, 0x520 },	/* parent APCIE_SYS_ST */
	{ "GLOBAL_AUX_CTRL0", 0x1a0, 0 },
	{ "GLOBAL_AUX_CTRL1", 0x1a8, 0 },
	{ "FPWM0", 0x330, 0x2b0 },		/* parent SIO */
	{ "SIO_CPU", 0x328, 0x2b0 },		/* parent SIO; macOS -53 ms */
	/* ATC2/3: no port on the Airs, never touched by macOS; lowered only if on */
	{ "ATC2_CIO_PCIE", 0x4f8, 0x3f0 },
	{ "ATC2_CIO_USB", 0x500, 0x3f0 },
	{ "ATC2_PCIE", 0x3e8, 0x2d8 },
	{ "ATC2_CIO", 0x3f0, 0x2d8 },
	{ "ATC2_COMMON", 0x2d8, 0 },
	{ "ATC3_CIO_PCIE", 0x508, 0x400 },
	{ "ATC3_CIO_USB", 0x510, 0x400 },
	{ "ATC3_PCIE", 0x3f8, 0x2e0 },
	{ "ATC3_CIO", 0x400, 0x2e0 },
	{ "ATC3_COMMON", 0x2e0, 0 },
};

static const struct apple_sleep_soc t8142_sleep_soc = {
	.ops = &t8142_sleep_ops,
	.untested = true,
	.regs_absent = t8142_regs_absent,
	.spmi_gen4 = true,
	.boot_wfi_delay_ms = 3,
	.wake_optional = true,
	/*
	 * DCP, DISPEXT0 (0x428/0x418/0x308), DISPEXT1 (0x3b8/0x3a8/0x2b8), lowered in macOS's order
	 * (J815 26B131: DISPEXT0, DISPEXT1, DISP). Both external DCPs are bound on J813/J815 now;
	 * apple_sleep.dispext=0 leaves them as before. macOS writes no pwrgate word for them. Their
	 * DARTs' PS write-disable bits (trace pmgr-soc-ps+0x3fe8 bit 15, +0x3fec bit 31) are clear
	 * at macOS's commit.
	 */
	.ndisp = 3,
	.display_order = { 1, 2, 0 },
	.dispext_optional = true,
	.disp_wr_dis = { { 0 }, { 0x17fe8, 15 }, { 0x17fec, 31 } },
	/* DISP_ADMA (pmgr base 1 +0x68), DISPEXT0_ADMA, DISPEXT1_ADMA */
	.disp_adma = { 0x10068, 0x420, 0x3b0 },
	.extra_off = t8142_extra_off,
	.nextra_off = ARRAY_SIZE(t8142_extra_off),
	/* E, P (ADT soc-clusters) */
	.cluster_cpu = { 0x0, 0x4 },
	.ncluster_cpu = 2,
	/* Six E cores: E bits 0-5, P bits 6-9, the boot core P0. */
	.cpu_start_stride = 6,
	.cllt = { { 0x50, GENMASK(17, 16) } },
	.ncllt = 1,
	.ps_wr_dis = t8142_ps_wr_dis,
	.nps_wr_dis = ARRAY_SIZE(t8142_ps_wr_dis),
	/* ADT mcc: amcc-plane-enable-mask 0xf, dcs-count-per-amcc 8, socni-count 6 */
	.namcc = 4,
	.nmcc = 8,
	.socni_words = t8142_socni_words,
	.nsocni = ARRAY_SIZE(t8142_socni_words),
	.aon_wdt_sleep = 0x20,
	/* The same PMU, and the same boot-stage byte, as T8132 (J815 ADT info-leg_scrpad). */
	.pmu_usid = 0xe,
	.pmu_sleep_reg = 0xf801,
	.pmu_sleep_val = 0x50,
	.spmi_notify_sleep = 0x2000001,
	.wake_mask = 0x08,
	/* AppleT8142PMGR, 27.2.1: masks at map 0x42 + 0x08..0x28, status 0x2c..0x4c */
	.wake_status = 0x2c,
	.wake_words = 9,
	.ps = {
		.disp = {
			{ 0x10000, 0x318, 0x290 },
			{ 0x428, 0x418, 0x308 },
			{ 0x3b8, 0x3a8, 0x2b8 },
		},
		.sio_cpu = 0x328,
		.sep = 0xc00,
		.ap_tmm = 0x1b0,
		.gpio = 0x120,
		.ans = 0x300,
		.apcie_gp = 0x2f0,
		.apcie_sys_gp = 0x408,
		.apcie_st = 0x410,
		.apcie_sys_st = 0x520,
		.apcie_phy_sw = 0x560,
		.apcie_aus5_aon = 0x578,
		.apcie_aus5 = 0x580,
	},
	.ps_aon_debug_switch = 0xc0,
	/* ADT /arm-io/apcie: the PHY-PHY block (reg[3]) is 0x20000 into the PHY's (reg[2]). */
	.apcie_phy_ctrl = 0x20000,
	.apcie_gen4 = true,
	.radio_retrain = true,
	.pen_deep = true,
	.early_cpu_wfi = true,
	.pwrgate_wake = { 0x000, 0x060 },
	/* AppleT8142PMGR vtable 0xfa0: 0xa0 + 8 * complex */
	.pwrgate_cpm = { 0x0a0, 0x0a8 },
	/* DISP_CPU's retention words: the J815 pmgr's retention_config_ff/_mem (device 0xb7) */
	.pwrgate_disp = { { 0x11cc, 0x11f0 } },
	.ps_all = SLEEP_WORDS(t8142_ps_all),
	.ps_aon_all = SLEEP_WORDS(t8142_ps_aon_all),
	.ps_macos_off = SLEEP_WORDS(t8142_ps_macos_off),
	.ps_aon_macos_off = SLEEP_WORDS(t8142_ps_aon_macos_off),
	.ps_wake_order = SLEEP_WORDS(t8142_ps_wake_order),
	.ps_aon_wake_order = SLEEP_WORDS(t8142_ps_aon_wake_order),
	.ps_boot_auto = SLEEP_WORDS(t8142_ps_boot_auto),
	.ps_aon_boot_auto = SLEEP_WORDS(t8142_ps_aon_boot_auto),
	.mem_boot = true,
	.quiesce = SLEEP_WORDS(t8142_quiesce_words),
	.stray = t8142_stray_words,
	.nstray = ARRAY_SIZE(t8142_stray_words),
	.ps_keep = t8142_ps_keep,
	.nps_keep = ARRAY_SIZE(t8142_ps_keep),
	.gate_off = t8142_gate_off,
	.ngate_off = ARRAY_SIZE(t8142_gate_off),
	.gate_coprocs = t8142_gate_coprocs,
	.ngate_coprocs = ARRAY_SIZE(t8142_gate_coprocs),
	.wake_names = t8142_wake_names,
	.nwake_names = ARRAY_SIZE(t8142_wake_names),
	/* AppleT8142PMGR's valid-source table: word 8 holds sources 256..267. */
	.wake_last_word = GENMASK(11, 0),
};

/*
 * Qualified profiles expose system sleep. Inventory-only profiles match for a
 * precise refusal and cannot reach hardware preparation or suspend_set_ops().
 * T8142 remains opt-in and requires its separate physical-commit gate.
 */
static const struct of_device_id apple_sleep_of_match[] = {
	{ .compatible = "apple,t8140-sleep", .data = &t8140_sleep_soc },
	{ .compatible = "apple,t8132-sleep", .data = &t8132_sleep_soc },
	{ .compatible = "apple,t8142-sleep", .data = &t8142_sleep_soc },
	{ .compatible = "apple,t6050-sleep", .data = &t6050_sleep_soc },
	{ .compatible = "apple,t6050-j716c-sleep", .data = &t6050_j716c_sleep_soc },
	{ }
};

static struct platform_driver apple_sleep_driver = {
	.driver = {
		.name = "apple-sleep",
		.of_match_table = apple_sleep_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = apple_sleep_probe,
};
builtin_platform_driver(apple_sleep_driver);

#if IS_ENABLED(CONFIG_APPLE_SOC_SLEEP_KUNIT_TEST)
#include "sleep-test.c"
#endif
