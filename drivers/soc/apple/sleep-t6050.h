/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * T6050 (M5 Pro) suspend to RAM: the sleep entry and the wake on the PMGR banks, the clusters,
 * the PMU and the coprocessors, with the SoC data the boot loader puts in the DT. Included by
 * sleep.c after its mapping and common power helpers.
 */
#define SLEEP_REGION_WORD(r, off) (((r) << 20) | (off))
#define SLEEP_REGION(w) (((w) >> 20) & 0x1f)
#define SLEEP_REGION_OFFSET(w) ((w) & GENMASK(19, 0))
#define SLEEP_REPLAY_PS BIT(31)
#define SLEEP_REPLAY_INDEX(w) ((w) & ~SLEEP_REPLAY_PS)

enum {
	T6050_PS0, T6050_PS1, T6050_PS_AON, T6050_PS_GPX,
	T6050_PS_MEDIA, T6050_PS_GFX_AON, T6050_CLK, T6050_CLK_SOC,
	T6050_CLK1, T6050_AFR_CLK, T6050_MEDIA_CLK, T6050_PERF,
	T6050_MEDIA_PERF, T6050_TABLE0, T6050_TABLE1, T6050_MEDIA_TABLE,
	T6050_REGIONS,
};

#define T6050_W(r, off) SLEEP_REGION_WORD(r, off)
#include "sleep-t6050-tables.h"
#undef T6050_W

/* apple,sleep-darts' order, then dart-apcie0 by name */
enum { T6050_DART_SEP, T6050_DART_PMP0, T6050_DART_SIO, T6050_DART_DCP, T6050_DART_APCIE0 };

/*
 * Register sequences the boot loader supplies for the radio's link and the shared PCIe PHY,
 * as <kind block offset value> each, run in order. Bits 7:0 of kind: write the value, read,
 * set the value's bits, clear them, write back what is read, wait for the value's bits to be
 * set, wait for them to be clear, a 16-bit write. Bits 31:16: a wait's timeout in ms (0: 10
 * ms); bit 8: its timeout is not an error.
 */
enum {
	T6050_OP_WRITE, T6050_OP_READ, T6050_OP_SET, T6050_OP_CLEAR, T6050_OP_REWRITE,
	T6050_OP_WAIT_SET, T6050_OP_WAIT_CLEAR, T6050_OP_WRITE16, T6050_OP_KINDS,
};
#define T6050_OP_KIND		GENMASK(7, 0)
#define T6050_OP_OPTIONAL	BIT(8)
#define T6050_OP_TIMEOUT	GENMASK(31, 16)

/* The blocks: the radio host's config, port0 (0x24000 from it), phy0, rc, phy0's four port words
 * (0x10000 from it) and the PHY's common page below them, then the sleep's storage-phy and
 * storage-phy-ip.
 */
enum {
	T6050_RBLK_CONFIG, T6050_RBLK_PORT, T6050_RBLK_PHY, T6050_RBLK_RC, T6050_RBLK_LANES,
	T6050_RBLK_PHY_CMN, T6050_RBLK_SHARED, T6050_RBLK_IP, T6050_RBLK_SPMI,
	T6050_RBLK_NODE = 16, T6050_RBLK_COUNT = 64,
};

/* The DT's commit schedule: each step's time before the sleep timer, in us. */
enum {
	T6050_STEP_MCC0, T6050_STEP_MCC1, T6050_STEP_PHY_PARK, T6050_STEP_STORAGE, T6050_STEP_WDT,
	T6050_STEP_FLOOR, T6050_STEP_MCC_STOP, T6050_STEP_CLKGEN, T6050_STEP_CLK,
	T6050_STEP_PTD0, T6050_STEP_PTD1, T6050_STEP_PTD2,
	T6050_STEP_PWRGATE0, T6050_STEP_PWRGATE1, T6050_STEP_PWRGATE2,
	T6050_STEP_PERF, T6050_STEP_AFR, T6050_STEP_CLUSTER, T6050_STEP_SIO, T6050_STEP_TABLES,
	T6050_STEPS,
};

/* The DT's named register sequences */
enum {
	T6050_SEQ_BOOT, T6050_SEQ_STAGE, T6050_SEQ_WDT, T6050_SEQ_AFR, T6050_SEQ_SIO,
	T6050_SEQ_DISP_GATE, T6050_SEQS,
};

struct t6050_ops {
	const u32 *op;
	unsigned int n;
};

/* An SPMI controller's peripheral sources: eight words of 32 */
#define T6050_SPMI_SRC_WORDS	8
/* apple,spmi-buses (spmi_bus[]), and with nub-spmi0 */
#define T6050_SPMI_BUSES	7
#define T6050_SPMI_CTRLS	(1 + T6050_SPMI_BUSES)

struct apple_sleep_region_state {
	void __iomem *base[T6050_REGIONS];
	struct resource resource[T6050_REGIONS];
	u32 cluster_saved[5], complex_observed, complex_request;
	u32 rails[6], floor[5], floor_changed;
	u64 ptd_saved;
	u32 park_saved, gate_saved[2];
	u32 display_saved[SLEEP_DISPLAYS_MAX][3];
	unsigned long display_lowered;
	bool staged, committed, pmc_saved, ptd_owned, attempt_active, boot_stage_written;
	unsigned int late_errors;
	struct device_node *ptd_entry[4];
	u32 *saved, *woke, *replay;
	bool *done, *late;
	/* Connected to shared domain APIs; owners clear exclusions after a successful raise. */
	unsigned long *driver_down;
	bool snapshot_valid;
	unsigned long *manager_down;
	u8 alarm_compare[4];
	bool alarm_valid;
	unsigned int pmu_errors;
	/* The DART GAPF bank, read only, if the node names it. */
	void __iomem *gapf;
	/* Words lowered or capped after the last coprocessor stop, and those that timed out. */
	u32 late_lowered, state4_set, late_timeouts;
	/* Each as it read before the entry lowered or capped it, and the PMC channels it took down */
	u32 late_saved[17], state4_saved[16], pmc154_lowered;
	/* AUSS and the routers' CIO_AUSS words the entry lowered, and each as it read before */
	u32 auss_lowered, auss_timeouts, auss_saved[5];
	/* VENC0_SYS as it read before the entry lowered it, if it did */
	u32 venc0_saved;
	bool venc0_lowered;
	/* The idle words the last refusal switched off and on again: their blocks' registers are at reset */
	u32 refusal_cycled;
	/* Type-C ports, by port index, whose always-on USB word the last entry left on. */
	unsigned long usb_aon_kept;
	/* apple,sleep-darts, then the radio port's DART, mapped unclaimed */
	void __iomem *dart[5];
	u32 dart_done, dart_timeouts;
	/* The radio's port, from apple,radio-pcie, for its link shutdown and the wake's restart */
	void __iomem *n1_ecam, *n1_port, *n1_phy, *n1_rc;
	struct platform_device *radio_host, *radio_dart;
	u32 n1_perst, n1_held, n1_gapf;
	int n1_state, n1_up_ret;
	bool n1_host;
	/* The GFX die's interrupt controller word over the sleep, and GFX at the wake */
	u32 gfx_aic[3];
	u32 gfx_wake[2];
	u32 pmc154[2];
	u8 pmu_rd[16];
	int after_ans_ret[4];
	u16 disp_children_lowered[5];
	/* Each display child's word as it read before the stop lowered it */
	u32 disp_child_saved[5][16];
	u32 mcc_cycle[2][10];
	void __iomem *spmi_bus[T6050_SPMI_BUSES];
	/* nub-spmi0's controller: its driver's lock is held through a returning PMU exchange */
	struct device *pmu_bus;
	void __iomem *aon_clk[2];
	void __iomem *ap_gpio;
	u32 pins_low[4];
	unsigned int npins_low;
	void __iomem *storage_phy, *storage_phy_ip;
	bool storage_phy_parked;
	/* The radio's and the shared PHY's register sequences from the DT, and their blocks */
	void __iomem *rblk[T6050_RBLK_COUNT];
	u32 rblk_size[T6050_RBLK_COUNT];
	struct t6050_ops link_down, link_down_port, port_wake, dart_wake, phy_park, phy_down,
			 phy_wake, seq[T6050_SEQS];
	/* From the DT: the commit schedule, the word lists the entry and the wake take */
	u32 sched[T6050_STEPS];
	struct apple_sleep_words wake_order, entry_clk, entry_perf;
	/* PTD: the request entry, and the clusters' and storage's bits in it */
	u32 ptd_req, ptd_bit[4];
	bool ptd_valid;
	/* Each stopped DART's GAPF slice, the value written there, and what it held before */
	u32 dart_slice[5], dart_slice_off, dart_slice_saved[5], dart_slice_written;
	/* The PMU's reads after the boot stage, the SPMI buses' masking order and gap */
	u32 pmu_reads[8][2], npmu_reads, spmi_order[7], nspmi_order, spmi_gap_us;
	/* nub-spmi0's interrupt mask at run time */
	u32 spmi_runtime_mask;
	/* nub-spmi0's status word holding the PMU's interrupt sources, and their bits (the DT's) */
	u32 spmi_pmu_status, spmi_pmu_bits;
	/* From the SMC's idle notice to the timer: every step of the schedule is nearer the timer */
	u32 runway_us;
	/* The PMU RTC's control after the timer, the complex requests, the floor steps */
	u32 alarm_rearm[2], complex_map[4][2], ncomplex, floor_steps[4], nfloor_steps;
	u32 floor_step_ms, pmc_channels[4], npmc_channels, pmu_wake_stage;
	bool alarm_rearm_valid, pmu_wake_stage_valid;
	/* ANS's power word and its parents', as indices into the power words */
	unsigned int ans_chain[4], nans_chain;
	/* Clock generator page 0's +0x78000 on both dies, MCC reg[22..26] */
	void __iomem *clkgen78, *media_clkgen78, *mcc[5];
	void __iomem *cpm[3];
	/* From the DT: the boot set-up written to them at probe */
	u64 *cpm_boot;
	unsigned int ncpm_boot;
	/* The AP wake-enable words, on this die and the GFX die */
	void __iomem *wake, *wake_gfx, *aic_gfx;
	u32 wake_enabled[11], wake_saved[11];
	/* The raw sticky wake status words as the wake found them */
	u32 wake_status[11];
	bool wake_status_valid;
	bool wake_armed, wake_sources;
	u32 spmi_mask[T6050_SPMI_CTRLS];
	bool spmi_masked;
	bool spmi_post_timer;
	/*
	 * nub-spmi0's and each of apple,spmi-buses's peripheral sources as the last acknowledgement
	 * found them (t6050_spmi_src_ack): latched, enabled, and those it acknowledged
	 */
	u32 spmi_src_latched[T6050_SPMI_CTRLS][T6050_SPMI_SRC_WORDS];
	u32 spmi_src_enabled[T6050_SPMI_CTRLS][T6050_SPMI_SRC_WORDS];
	u32 spmi_src_acked[T6050_SPMI_CTRLS][T6050_SPMI_SRC_WORDS];
	bool spmi_src_valid;
	/* This wake's status words are read and not yet in its events (t6050_wake_events_report) */
	bool wake_status_fresh;
	/* After a wake, the PMU's steps wait for the SMC's (t6050_sleep_pmu_wake) */
	bool pmu_wake_pending;
	u8 pmu_wake_rtc_ctl;
	int pmu_wake_rtc_ret;
	/* Each cluster's cores that reached their last WFI (t6050_cpu_down()) */
	atomic_t cpus_down[3];
	/*
	 * The secondary clusters taken down first (t6050_clusters_down_first()): the result of the
	 * cores' wait and of each cluster's sequencer poll, the sequencers' last reads and the
	 * time each poll took, reported after the wake.
	 */
	bool early_done, early_dropped;
	int early_ret[3];
	u32 early_seq[2], early_us[2];
};

static const struct apple_sleep_region_desc t6050_regions[] = {
	{ "ps0", 0x4000, true }, { "ps1", 0x4000, true },
	{ "ps-aon", 0xc000, true }, { "ps-gpx", 0x4000, true },
	{ "ps-media", 0xc000, true }, { "ps-gfx-aon", 0x4000, true },
	{ "clk", 0x10000 }, { "clk-soc", 0x4000 },
	{ "clk1", 0x4000 }, { "afr-clk", 0x4000 },
	{ "media-clk", 0x10000 }, { "perf", 0x20000 },
	{ "media-perf", 0x14000 }, { "table0", 0x4000 },
	{ "table1", 0x4000 }, { "media-table", 0x4000 },
};

static const struct apple_sleep_region_table t6050_tables[] = {
	{ T6050_TABLE0, 0x44, 27 }, { T6050_TABLE1, 0x24, 25 },
	{ T6050_MEDIA_TABLE, 0x44, 14 },
};

static const struct apple_sleep_region_profile t6050_region_profile = {
	.region = t6050_regions,
	.count = ARRAY_SIZE(t6050_regions),
	.power = SLEEP_WORDS(t6050_ps_all),
	.table = t6050_tables,
	.ntables = ARRAY_SIZE(t6050_tables),
};

/*
 * The Max inventory, for the J716c's full sleep. Its keepalive tables have the Pro's bases,
 * counts and strides.
 */
static const struct apple_sleep_region_profile t6050_j716c_region_profile = {
	.region = t6050_regions,
	.count = ARRAY_SIZE(t6050_regions),
	.power = SLEEP_WORDS(t6050_j716c_ps_all),
	.table = t6050_tables,
	.ntables = ARRAY_SIZE(t6050_tables),
};

static bool sleep_region_word_valid(const struct apple_sleep_region_profile *p,
				    u32 w, bool power)
{
	unsigned int r = SLEEP_REGION(w);
	u32 off = SLEEP_REGION_OFFSET(w);

	return !(w & ~GENMASK(24, 0)) && r < p->count && !(off & 3) &&
	       p->region[r].size >= sizeof(u32) &&
	       off <= p->region[r].size - sizeof(u32) &&
	       p->region[r].power == power;
}

/* All indices/apertures checked before mapping or installing any power ops. */
static int sleep_region_validate(const struct apple_sleep_region_profile *p)
{
	unsigned int i, j;

	if (!p || !p->count || p->count > T6050_REGIONS || !p->power.count)
		return -EINVAL;
	for (i = 0; i < p->count; i++)
		if (!p->region[i].name || p->region[i].size < sizeof(u32))
			return -EINVAL;
	for (i = 0; i < p->power.count; i++) {
		if (!sleep_region_word_valid(p, p->power.offset[i], true))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (p->power.offset[j] == p->power.offset[i])
				return -EINVAL;
	}
	for (i = 0; i < p->ntables; i++) {
		const struct apple_sleep_region_table *t = &p->table[i];

		if (t->region >= p->count || p->region[t->region].power ||
		    !t->count || t->stride < 4 || t->stride % 4 ||
		    (u64)(t->count - 1) * t->stride + 4 > p->region[t->region].size)
			return -EINVAL;
	}
	return 0;
}

static void __iomem *sleep_region_reg(struct apple_sleep *s, u32 w)
{
	return s->region->base[SLEEP_REGION(w)] + SLEEP_REGION_OFFSET(w);
}

static int sleep_region_map(struct apple_sleep *s)
{
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct apple_sleep_region_state *r;
	unsigned int i;
	int ret = sleep_region_validate(p);

	if (ret)
		return dev_err_probe(s->dev, ret, "invalid region/replay data\n");
	r = devm_kzalloc(s->dev, sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	for (i = 0; i < p->count; i++) {
		struct resource res;
		int idx = of_property_match_string(s->dev->of_node, "reg-names", p->region[i].name);

		if (idx < 0 || of_address_to_resource(s->dev->of_node, idx, &res) ||
		    resource_size(&res) != p->region[i].size)
			return dev_err_probe(s->dev, -EINVAL, "wrong %s aperture\n", p->region[i].name);
		r->resource[i] = res;
		r->base[i] = apple_sleep_map(s->dev, p->region[i].name);
		if (IS_ERR(r->base[i]))
			return PTR_ERR(r->base[i]);
	}
	r->saved = devm_kcalloc(s->dev, p->power.count, sizeof(u32), GFP_KERNEL);
	r->woke = devm_kcalloc(s->dev, p->power.count, sizeof(u32), GFP_KERNEL);
	r->done = devm_kcalloc(s->dev, p->power.count, sizeof(bool), GFP_KERNEL);
	r->late = devm_kcalloc(s->dev, p->power.count, sizeof(bool), GFP_KERNEL);
	r->driver_down = devm_bitmap_zalloc(s->dev, p->power.count, GFP_KERNEL);
	r->manager_down = devm_bitmap_zalloc(s->dev, p->power.count, GFP_KERNEL);
	if (!r->saved || !r->woke || !r->done || !r->late ||
	    !r->driver_down || !r->manager_down)
		return -ENOMEM;
	s->region = r;
	return 0;
}

static void sleep_region_snapshot(struct apple_sleep *s)
{
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	for (i = 0; i < p->power.count; i++)
		r->saved[i] = readl(sleep_region_reg(s, p->power.offset[i]));
	for (i = 0; i < r->wake_order.count; i++)
		if (!(r->wake_order.offset[i] & SLEEP_REPLAY_PS))
			r->replay[i] = readl(sleep_region_reg(s, r->wake_order.offset[i]));
	r->snapshot_valid = true;
}

/* The storage link's power words the sleep lowers after ANS's stop, as macOS. */
static const u32 t6050_ans_storage_ps[] = {
	0x268, /* APHY_ST0_AUS5 */
	0x240, /* APHY_ST0_AUS5_A */
	0x270, /* APHY_ST1_AUS5 */
	0x248, /* APHY_ST1_AUS5_A */
	/* macOS lowers these three as well, and leaves ANS and APCIE_SYS_ST0/1 on auto. */
	0x280, /* APHY_GP_AUS5 */
	0x258, /* APHY_GP_AUS5_A */
	0x1b0, /* APCIE_PHY_SW */
};
static const u8 t6050_ans_storage_on[] = { 6, 5, 4, 1, 3, 0, 2 };

static bool t6050_ans_storage_word(u32 word)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(t6050_ans_storage_ps); i++)
		if (word == SLEEP_REGION_WORD(T6050_PS1, t6050_ans_storage_ps[i]))
			return true;
	return false;
}

static int t6050_ans_storage_index(struct apple_sleep *s, u32 word)
{
	unsigned int i;

	for (i = 0; i < s->soc->regions->power.count; i++)
		if (s->soc->regions->power.offset[i] == word)
			return i;
	return -ENOENT;
}

/* The wake's order interleaves power and clock, perf and table words: never split by bank. */
static bool t6050_v27_retained_display_word(struct apple_sleep *s, u32 word)
{
	unsigned int d, j;

	if (!apple_sleep_v27_late())
		return false;
	for (d = 1; d < s->soc->ndisp; d++)
		for (j = 1; j < 3; j++)
			if (word == s->soc->ps.disp[d][j])
				return true;
	return false;
}


/*
 * The SEP's domain is under automatic power: the hardware raises it when the enclave is used,
 * and a forced raise at the wake does not take. Whatever the snapshot caught it doing, its word
 * goes back under automatic power without a wait (sleep_ps_restore()'s path for a word off
 * under AUTO_ENABLE), rather than waiting for a state the hardware gives only on demand.
 */
static u32 t6050_replay_saved(struct apple_sleep *s, u32 word, u32 saved)
{
	if (s->soc->ps.sep && word == s->soc->ps.sep && (saved & PS_AUTO_ENABLE))
		return saved & ~PS_ACTUAL;
	return saved;
}

static void sleep_region_replay(struct apple_sleep *s)
{
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct apple_sleep_region_state *r = s->region;
	unsigned int i, pass;

	if (!r->snapshot_valid)
		return;
	memset(r->done, 0, p->power.count * sizeof(*r->done));
	memset(r->late, 0, p->power.count * sizeof(*r->late));
	s->ps_rewoken = 0;
	s->ps_rewake_timeouts = 0;
	for (i = 0; i < p->power.count; i++)
		r->woke[i] = readl(sleep_region_reg(s, p->power.offset[i]));
	/*
	 * GFX comes back on from the sleep, and macOS writes it off at once:
	 * it stays off until the GPU's start raises it, so the firmware starts from a power cycle.
	 */
	r->gfx_wake[0] = r->gfx_wake[1] = 0;
	if (r->committed) {
		void __iomem *gfx = r->base[T6050_PS_GPX] + 0x110;
		u32 v;

		r->gfx_wake[0] = readl(gfx);
		/* Target 0, its auto state and flags acknowledged */
		writel(PS_AUTO_STATE | PS_FLAGS | PS_ACTUAL, gfx);
		readl_poll_timeout_atomic(gfx, v, !FIELD_GET(PS_ACTUAL, v), 1, PS_OFF_TIMEOUT_US);
		r->gfx_wake[1] = v;
	}
	for (i = 0; i < r->wake_order.count; i++) {
		u32 w = r->wake_order.offset[i];
		unsigned int n;

		if (!(w & SLEEP_REPLAY_PS)) {
			writel(r->replay[i], sleep_region_reg(s, w));
			continue;
		}
		n = SLEEP_REPLAY_INDEX(w);
		r->done[n] = true;
		if (r->committed &&
		    p->power.offset[n] == SLEEP_REGION_WORD(T6050_PS_GPX, 0x110))
			continue;
		if (t6050_v27_retained_display_word(s, p->power.offset[n])) {
			u32 value;

			/* Lowered by the display stop: its restore raises it, after this. */
			if (test_bit(n, r->manager_down))
				continue;
			value = readl(sleep_region_reg(s, p->power.offset[n]));

			if ((value & (PS_TARGET | PS_ACTUAL)) != (PS_TARGET | PS_ACTUAL) ||
			    (value & PS_DEV_DISABLE))
				r->late_errors++;
			continue; /* no write through an unknown display protection slice */
		}
		if (s->pcie_ans_stopped && t6050_ans_storage_word(p->power.offset[n]))
			continue;
		if ((test_bit(n, r->driver_down) || test_bit(n, r->manager_down)) || !FIELD_GET(PS_TARGET, r->saved[n]))
			continue;
		s->ps_rewoken++;
		r->late[n] = !!sleep_ps_restore(sleep_region_reg(s, p->power.offset[n]),
				t6050_replay_saved(s, p->power.offset[n], r->saved[n]));
	}
	for (pass = 0; pass < PS_REWAKE_PASSES; pass++)
		for (i = 0; i < p->power.count; i++) {
			if (t6050_v27_retained_display_word(s, p->power.offset[i]))
				continue;
			if (s->pcie_ans_stopped && t6050_ans_storage_word(p->power.offset[i]))
				continue;
			if (r->committed &&
			    p->power.offset[i] == SLEEP_REGION_WORD(T6050_PS_GPX, 0x110))
				continue;
			if ((test_bit(i, r->driver_down) || test_bit(i, r->manager_down)) || (!r->late[i] &&
			    (r->done[i] || !sleep_ps_dropped(r->saved[i], r->woke[i]))))
				continue;
			if (!r->done[i])
				s->ps_rewoken++;
			r->done[i] = true;
			r->late[i] = !!sleep_ps_restore(sleep_region_reg(s, p->power.offset[i]),
				t6050_replay_saved(s, p->power.offset[i], r->saved[i]));
		}
	for (i = 0; i < p->power.count; i++)
		s->ps_rewake_timeouts += r->late[i];
	r->snapshot_valid = false;
}

/* SPMI extended register commands, sent to the PMU by hand in the sleep's last steps */
#define T6050_SPMI_EXT_WRITEL	0x30
#define T6050_SPMI_EXT_READL	0x38
#define M5_SPMI_STATUS 0x200
#define M5_SPMI_CMD 0x210
#define M5_SPMI_REPLY 0x220
#define M5_SPMI_RX_EMPTY BIT(30)
#define M5_SPMI_IRQ_EN 0x420
#define M5_SPMI_IRQ_STATUS 0x620
#define M5_SPMI_IRQ_DONE BIT(0)
/* The peripheral sources' status words, below the controller's own: write 1 to clear */
#define M5_SPMI_SRC_STATUS 0x600
#define M5_PMU_SID 0x0e
/* The PMU's RTC: control (bit 1 alarm fired, bit 6 alarm armed), counter, compare, mask */
#define T6050_RTC_CTL		0x2000
#define   T6050_RTC_CTL_FIRED	BIT(1)
#define   T6050_RTC_CTL_ARMED	BIT(6)
#define T6050_RTC_COUNTER	0x2002
#define T6050_RTC_COMPARE	0x2008
#define T6050_RTC_MASK		0x200e
#define T6050_PMU_BOOT_STAGE	0xf801
#define T6050_PMU_BOOT_STAGE_SLEEP	0x50


/*
 * A PMU command on nub-spmi0 by hand: status, the interrupt mask (the sleep's without its done
 * bit after the timer, the sleep's or the runtime one before it), the command and its data, the done bit
 * polled and acked, then status and one reply word at a time, and status. A reply word is read
 * only once the status says the FIFO holds one: the done bit alone does not prove it.
 */
static int t6050_sleep_pmu_xfer_post(struct apple_sleep *s, u32 mask, u8 command, u16 address,
				     const u8 *wr, u8 *rd, unsigned int len)
{
	u32 value, data;
	unsigned int i, n;
	int ret = 0;

	readl(s->spmi + M5_SPMI_STATUS);
	writel(mask, s->spmi + M5_SPMI_IRQ_EN);
	writel(command | M5_PMU_SID << 8 | (u32)address << 16 | BIT(15), s->spmi + M5_SPMI_CMD);
	for (i = 0; wr && i < len; i += 4) {
		data = 0;
		for (n = 0; n < min(4U, len - i); n++)
			data |= (u32)wr[i + n] << (8 * n);
		writel(data, s->spmi + M5_SPMI_CMD);
	}
	/* The done bit latches under the sleep's mask; if it doesn't, the reply still counts. */
	if (!readl_poll_timeout_atomic(s->spmi + M5_SPMI_IRQ_STATUS, value,
				       value & M5_SPMI_IRQ_DONE, 1, 1000))
		writel(value, s->spmi + M5_SPMI_IRQ_STATUS);
	else if (readl_poll_timeout_atomic(s->spmi + M5_SPMI_STATUS, value,
					   !(value & M5_SPMI_RX_EMPTY), 1, 1000))
		return -ETIMEDOUT;
	/* The same single status read when the reply is there, as it is after the done bit. */
	if (readl_poll_timeout_atomic(s->spmi + M5_SPMI_STATUS, value,
				      !(value & M5_SPMI_RX_EMPTY), 1, 1000))
		return -ETIMEDOUT;
	value = readl(s->spmi + M5_SPMI_REPLY);
	if ((value & 0xff) != command || ((value >> 8) & 0x7f) != M5_PMU_SID ||
	    (wr && !(value & BIT(15))))
		ret = -EIO;
	for (i = 0; !ret && rd && i < len; i += 4) {
		if (readl(s->spmi + M5_SPMI_STATUS) & M5_SPMI_RX_EMPTY) {
			ret = -EIO; /* A short reply: no word is invented for it. */
			break;
		}
		data = readl(s->spmi + M5_SPMI_REPLY);
		for (n = 0; n < min(4U, len - i); n++)
			rd[i + n] = data >> (8 * n);
	}
	readl(s->spmi + M5_SPMI_STATUS);
	return ret;
}

/* The reply FIFO read empty: at most its eight entries. */
static void t6050_sleep_pmu_drain(struct apple_sleep *s)
{
	unsigned int i;

	for (i = 0; i < 8 && !(readl(s->spmi + M5_SPMI_STATUS) & M5_SPMI_RX_EMPTY); i++)
		readl(s->spmi + M5_SPMI_REPLY);
}

/* Longer than any reply takes: after it, a failed command's late reply is in the FIFO. */
#define T6050_PMU_LATE_REPLY_US	2000

/*
 * A PMU command while the entry can still return to the running system, where the bus has an
 * owner: the SPMI controller's driver. Its API can't be used here: these commands run inside
 * cpu_suspend(), on the boot core alone with interrupts off, and the driver's exchange takes a
 * mutex and sleeps while it polls. So the exchange stays by hand, under the driver's own lock,
 * taken without waiting: a lock that is held means one of the driver's exchanges is under way,
 * and the sleep is refused rather than mixed into it. Nothing is left behind for the driver
 * either: after a failure the late reply is waited for and the FIFO read empty, and a reply
 * with more words than the command asked for is a failure.
 */
static int t6050_sleep_pmu_xfer_returning(struct apple_sleep *s, u8 command, u16 address,
					  const u8 *wr, u8 *rd, unsigned int len)
{
	struct apple_sleep_region_state *r = s->region;
	int ret = r->pmu_bus ? apple_spmi_fifo_trylock(r->pmu_bus) : -ENODEV;
	bool locked = !ret;

	/* No driver on the bus (-ENODEV): no other user either. */
	if (ret == -EBUSY)
		return ret;
	/* Every reply is read, so the FIFO should be empty here: a stale entry is read first. */
	t6050_sleep_pmu_drain(s);
	ret = t6050_sleep_pmu_xfer_post(s, r->spmi_runtime_mask, command, address, wr, rd, len);
	if (!ret && !(readl(s->spmi + M5_SPMI_STATUS) & M5_SPMI_RX_EMPTY))
		ret = -EIO;
	if (ret) {
		udelay(T6050_PMU_LATE_REPLY_US);
		t6050_sleep_pmu_drain(s);
	}
	if (locked)
		apple_spmi_fifo_unlock(r->pmu_bus);
	return ret;
}

/*
 * Polled, length and reply checked. Behind the SPMI masks the entry is committed and the
 * driver sends nothing until the wake: no lock there.
 */
static int t6050_sleep_pmu_xfer(struct apple_sleep *s, u8 opcode, u16 address,
				const u8 *wr, u8 *rd, unsigned int len)
{
	u8 command;

	if (!len || len > 8 || (!!wr == !!rd))
		return -EINVAL;
	command = opcode | (len - 1);
	if (s->region->spmi_post_timer)
		return t6050_sleep_pmu_xfer_post(s, s->soc->spmi_notify_sleep & ~M5_SPMI_IRQ_DONE,
						 command, address, wr, rd, len);
	if (!s->region->spmi_masked)
		return t6050_sleep_pmu_xfer_returning(s, command, address, wr, rd, len);
	/* Every reply is read, so the FIFO should be empty here: a stale entry is read first. */
	t6050_sleep_pmu_drain(s);
	return t6050_sleep_pmu_xfer_post(s, s->soc->spmi_notify_sleep, command, address, wr, rd,
					 len);
}

/* Before the shared last-stop/enclave boundary: failure must leave ANS running. */
static int t6050_sleep_check_alarm(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u8 control, mask;
	int ret;

	r->alarm_valid = false;
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_CTL, NULL, &control, 1);
	if (!ret)
		ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_MASK, NULL, &mask, 1);
	/* No armed alarm is no refusal: the SMC's wake sources (lid, keys) do not need it */
	if (!ret && (!(control & T6050_RTC_CTL_ARMED) || (mask & BIT(0))))
		return 0;
	if (!ret)
		ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_COMPARE, NULL,
					   r->alarm_compare, 4);
	r->alarm_valid = !ret;
	return ret;
}

/* PTD: an entry is read at 16 bytes per entry, written at the update page, 8 bytes per entry */
#define M5_PTD_READ(e) (16 * (e))
#define M5_PTD_WRITE(e) (0x10000 + 8 * (e))

/*
 * The storage link's PHY is parked before its power words go down, in the commit window. Its
 * registers fault with its domains off: nothing is touched unless they are on.
 */
#define T6050_PS1_APHY_ST0_AUS5	0x268
#define T6050_PS1_APCIE_PHY_SW	0x1b0

static bool t6050_storage_phy_up(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;

	return r->storage_phy && r->storage_phy_ip &&
	       sleep_ps_on(s, SLEEP_REGION_WORD(T6050_PS1, T6050_PS1_APCIE_PHY_SW)) &&
	       sleep_ps_on(s, SLEEP_REGION_WORD(T6050_PS1, T6050_PS1_APHY_ST0_AUS5));
}

static unsigned int t6050_ops_run(struct apple_sleep *s, const struct t6050_ops *ops);

/* The DT's apple,phy-park */
static void t6050_storage_phy_park(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;

	r->storage_phy_parked = r->phy_park.n && t6050_storage_phy_up(s);
	if (r->storage_phy_parked)
		t6050_ops_run(s, &r->phy_park);
}

/* After APHY_GP_AUS5_A, before APCIE_PHY_SW: the DT's apple,phy-down */
static void t6050_storage_phy_down(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;

	if (r->storage_phy_parked)
		t6050_ops_run(s, &r->phy_down);
}




static void t6050_sleep_post_timer(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u8 control, value;
	int ret;

	/*
	 * The boot core's PMU commands after the timer: the alarm rearmed, with the RTC control
	 * values the DT gives (apple,pmu-alarm-rearm: after a fired alarm, then the final one).
	 */
	r->spmi_post_timer = r->spmi_masked;
	r->pmu_errors = 0;
	if (!r->alarm_valid || !r->alarm_rearm_valid) {
		r->pmu_errors++;
		return;
	}
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_CTL, NULL, &control, 1);
	if (ret)
		goto failed; /* No fabricated status or conditional ack after a failed read. */
	if (control & T6050_RTC_CTL_FIRED) {
		value = r->alarm_rearm[0];
		ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_RTC_CTL, &value, NULL, 1);
		if (ret)
			goto failed;
	}
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_RTC_COMPARE, r->alarm_compare,
				   NULL, 4);
	if (ret)
		goto failed;
	value = 0;
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_RTC_MASK, &value, NULL, 1);
	if (ret)
		goto failed;
	value = r->alarm_rearm[1];
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_RTC_CTL, &value, NULL, 1);
failed:
	r->pmu_errors += !!ret;
}

static bool t6050_sleep_cpu_valid(u64 mpidr)
{
	unsigned int c = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	unsigned int type = MPIDR_AFFINITY_LEVEL(mpidr, 2);

	if ((mpidr & ~MPIDR_HWID_BITMASK) || MPIDR_AFFINITY_LEVEL(mpidr, 3) ||
	    MPIDR_AFFINITY_LEVEL(mpidr, 0) >= 6)
		return false;
	return (c < 2 && type == 4) || (c == 2 && type == 1);
}

/*
 * The complex request the commit writes for the state it finds, from the DT's
 * apple,complex-requests (observed, request) pairs. Any other state refuses the sleep before
 * anything irreversible.
 */
static int t6050_sleep_plan_complex_request(u32 observed, u32 *request)
{
	struct apple_sleep_region_state *r = apple_sleep ? apple_sleep->region : NULL;
	unsigned int i;

	if (!request || !r)
		return -EINVAL;
	for (i = 0; i < r->ncomplex; i++)
		if (r->complex_map[i][0] == observed) {
			*request = r->complex_map[i][1];
			return 0;
		}
	return -EOPNOTSUPP;
}

/* The coprocessors a sleep needs bound and, where it stops them, stopped. */
#define M5_GATE(c, base, idx, stop, word, loaded) \
	{ .compatible = (c), .match_start = (base), .run_index = (idx), \
	  .explicit_run_index = true, .must_stop = (stop), .ps = (word), \
	  .allow_unstarted = (loaded), .strict_unbound = true }
static const struct apple_sleep_gate_coproc t6050_gate_coprocs[] = {
	{ .compatible = "apple,t6050-smc", .strict_unbound = true },
	M5_GATE("apple,t6050-mtp", 0, 0, false, 0, true),
	M5_GATE("apple,t6050-aop", 0, 1, false, 0, true),
	M5_GATE("apple,t6050-aop2", 0, 0, false, 0, true),
	M5_GATE("apple,t6050-pmp-v2", 0, 1, true, SLEEP_REGION_WORD(T6050_PS0, 0x128), true),
	M5_GATE("apple,t6050-sio", 0, 0, true, SLEEP_REGION_WORD(T6050_PS1, 0x2a0), true),
	{ .compatible = "apple,t6050-sep", .ps = SLEEP_REGION_WORD(T6050_PS1, 0x298),
	  .strict_unbound = true },
	{ .compatible = "apple,t6050-nvme-ans2", .strict_unbound = true },
	M5_GATE("apple,t6050-dcp", 0, 0, true, SLEEP_REGION_WORD(T6050_PS1, 0x1a8), false),
	M5_GATE("apple,t6050-dcpext", 0x2b2e00000ULL, 0, true, SLEEP_REGION_WORD(T6050_PS0, 0x278), false),
	M5_GATE("apple,t6050-dcpext", 0x392e00000ULL, 0, true, SLEEP_REGION_WORD(T6050_PS0, 0x2e0), false),
	M5_GATE("apple,t6050-dcpext", 0x302e00000ULL, 0, true, SLEEP_REGION_WORD(T6050_PS0, 0x2b0), false),
	M5_GATE("apple,t6050-dcpext", 0x306e00000ULL, 0, true, SLEEP_REGION_WORD(T6050_PS0, 0x2c8), false),
	/* The GPU driver owns its readiness for the sleep. */
	{ .compatible = "apple,agx-g17s", .strict_unbound = true },
};
#undef M5_GATE
static const struct apple_sleep_word t6050_gate_off[] = {
	{ "ACIO0", SLEEP_REGION_WORD(T6050_PS0, 0x280) },
	{ "ACIO1", SLEEP_REGION_WORD(T6050_PS0, 0x290) },
	{ "ACIO2", SLEEP_REGION_WORD(T6050_PS0, 0x2e8) },
};

/* The boot core's power-down control, set for the sleep's own cpu_suspend() and put back after */
#define M5_C1_6 sys_reg(3, 5, 15, 1, 6)
#define M5_C1_6_BITS GENMASK_ULL(28, 26)

struct t6050_cpu_control {
	u64 saved;
	bool changed;
};
static DEFINE_PER_CPU(struct t6050_cpu_control, t6050_cpu_control);

static int t6050_sleep_cpu_prepare(u64 mpidr)
{
	struct t6050_cpu_control *c = this_cpu_ptr(&t6050_cpu_control);
	u64 value;

	/* Only for a kernel running at EL2 with VHE */
	if (!is_kernel_in_hyp_mode() ||
	    (read_sysreg(hcr_el2) & (HCR_E2H | HCR_TGE)) != (HCR_E2H | HCR_TGE))
		return -EOPNOTSUPP;
	if (!c->changed) {
		c->saved = read_sysreg_s(M5_C1_6);
		c->changed = true;
	}
	write_sysreg_s(c->saved | M5_C1_6_BITS, M5_C1_6);
	isb();
	value = read_sysreg_s(M5_C1_6);
	return (value & M5_C1_6_BITS) == M5_C1_6_BITS ? 0 : -EIO;
}

static void t6050_sleep_cpu_restore(u64 mpidr)
{
	struct t6050_cpu_control *c = this_cpu_ptr(&t6050_cpu_control);

	if (!c->changed)
		return;
	write_sysreg_s(c->saved, M5_C1_6);
	isb();
	c->changed = false;
}

/* Resolve only the two actual M5 shapes: a PMGR word or one PMP report
 * entry whose sole dependency is that PMGR word. A report's reg is an ID,
 * never a byte offset. This does not acquire/withdraw/restore the PMP vote.
 */
static struct device_node *sleep_region_pmgr_node(struct device_node *pd)
{
	struct device_node *parent __free(device_node) = NULL;
	struct of_phandle_args args;
	u32 cells;
	int ret;

	if (of_property_read_u32(pd, "#power-domain-cells", &cells) || cells)
		return ERR_PTR(-EINVAL);
	if (of_device_is_compatible(pd, "apple,t6050-pmgr-pwrstate"))
		return of_node_get(pd);
	if (!of_device_is_compatible(pd, "apple,t6050-pmp-v2-report-entry"))
		return ERR_PTR(-EINVAL);
	parent = of_get_parent(pd);
	if (!parent || !of_device_is_compatible(parent, "apple,t6050-pmp-v2-report") ||
	    of_count_phandle_with_args(pd, "power-domains", "#power-domain-cells") != 1)
		return ERR_PTR(-EINVAL);
	ret = of_parse_phandle_with_args(pd, "power-domains", "#power-domain-cells", 0, &args);
	if (ret)
		return ERR_PTR(ret);
	if (args.args_count || !of_device_is_compatible(args.np, "apple,t6050-pmgr-pwrstate") ||
	    of_property_read_u32(args.np, "#power-domain-cells", &cells) || cells) {
		of_node_put(args.np);
		return ERR_PTR(-EINVAL);
	}
	return args.np;
}

/* The selected profile owns register encoding; exported driver APIs stay shared. */
static int sleep_region_domain_offset(struct apple_sleep *s, struct device_node *pd)
{
	struct device_node *word __free(device_node) = NULL;
	struct device_node *resolved = sleep_region_pmgr_node(pd);
	struct device_node *parent __free(device_node) = NULL;
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct resource res;
	u32 reg[2], off, encoded;
	unsigned int i;

	if (IS_ERR(resolved))
		return PTR_ERR(resolved);
	word = resolved;
	parent = of_get_parent(word);
	if (!parent || of_address_to_resource(parent, 0, &res) ||
	    of_property_count_u32_elems(word, "reg") != ARRAY_SIZE(reg) ||
	    of_property_read_u32_array(word, "reg", reg, ARRAY_SIZE(reg)))
		return -EINVAL;
	off = reg[0];
	if (reg[1] != sizeof(u32) || (off & 3) || resource_size(&res) < sizeof(u32) ||
	    off > resource_size(&res) - sizeof(u32))
		return -EINVAL;
	for (i = 0; i < p->count; i++) {
		if (!p->region[i].power || res.start != s->region->resource[i].start)
			continue;
		if (p->region[i].size < sizeof(u32) || off > p->region[i].size - sizeof(u32))
			return -EINVAL;
		encoded = SLEEP_REGION_WORD(i, off);
		if (!sleep_region_word_valid(p, encoded, true) ||
		    sleep_ps_find(p->power.offset, p->power.count, encoded) < 0)
			return -EINVAL;
		return encoded;
	}
	return -EINVAL;
}

static unsigned long *sleep_region_domain_bitmap(struct apple_sleep *s, u32 word,
						unsigned int *bit)
{
	const struct apple_sleep_words *p = &s->soc->regions->power;
	int index = sleep_ps_find(p->offset, p->count, word);

	if (index < 0)
		return NULL;
	*bit = index;
	return s->region->driver_down;
}

/*
 * The one-time clock-gating set-up of the PMGR's clock pages, where the boot loader puts it
 * (apple,clk-gating-boot: reg index, offset, value). Without it the pages keep iBoot's values.
 */
static void t6050_boot_clk_gating(struct apple_sleep *s)
{
	struct device_node *np = s->dev->of_node;
	int n = of_property_count_u32_elems(np, "apple,clk-gating-boot");
	void __iomem *base[48] = {};
	u32 size[48] = {};
	unsigned int i, bad = 0;
	u32 *w;

	if (n <= 0)
		return;
	if (n % 3) {
		dev_warn(s->dev, "apple,clk-gating-boot is not in threes\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(base); i++) {
		const char *name;
		struct resource res;

		if (of_property_read_string_index(np, "reg-names", i, &name) ||
		    of_address_to_resource(np, i, &res))
			break;
		/* Only the clock pages and tables: the sleep's mapped regions and the AON pages */
		if (strstr(name, "clk") || strstr(name, "table")) {
			base[i] = of_iomap(np, i);
			size[i] = resource_size(&res);
		}
	}
	w = kcalloc(n, sizeof(*w), GFP_KERNEL);
	if (w && !of_property_read_u32_array(np, "apple,clk-gating-boot", w, n)) {
		for (i = 0; i < n; i += 3) {
			if (w[i] >= ARRAY_SIZE(base) || !base[w[i]] || w[i + 1] % 4 ||
			    w[i + 1] + 4 > size[w[i]]) {
				bad++;
				continue;
			}
			writel(w[i + 2], base[w[i]] + w[i + 1]);
		}
	}
	kfree(w);
	for (i = 0; i < ARRAY_SIZE(base); i++)
		if (base[i])
			iounmap(base[i]);
	if (bad)
		dev_warn(s->dev, "%u apple,clk-gating-boot writes outside the clock pages\n", bad);
}

static int t6050_cpm_prop(struct apple_sleep *s, const char *name, u64 **ops, unsigned int *n);
static void t6050_seq(struct apple_sleep *s, unsigned int seq);

static void t6050_sleep_boot_auto(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	/* The clock-gating set-up comes first, before the fabric words go on auto. */
	t6050_boot_clk_gating(s);
	for (i = 0; i < r->ncpm_boot; i++) {
		const u64 *w = &r->cpm_boot[3 * i];

		if (w[0] < ARRAY_SIZE(r->cpm) && r->cpm[w[0]])
			writeq(w[2], r->cpm[w[0]] + w[1]);
	}
	/*
	 * The DT's boot sequence: the fabric and memory power words put under automatic power
	 * (iBoot leaves them on with AUTO_ENABLE clear, so none can gate), and the SPMI buses'
	 * interrupt masks and nub-spmi0's event sources.
	 */
	t6050_seq(s, T6050_SEQ_BOOT);
	/* nub-spmi0's mask through each runtime PMU command: the DT's, or what boot left */
	if (!r->spmi_runtime_mask)
		r->spmi_runtime_mask = readl(s->spmi + M5_SPMI_IRQ_EN);
}

/* A block an op names: the radio's and the PHY's, an SPMI bus, or one of this node's reg entries */
static bool t6050_ops_block(struct apple_sleep *s, u32 b)
{
	struct apple_sleep_region_state *r = s->region;
	struct device_node *np = s->dev->of_node;
	struct resource res;

	if (b >= T6050_RBLK_COUNT)
		return false;
	if (r->rblk[b])
		return true;
	if (b >= T6050_RBLK_SPMI && b < T6050_RBLK_SPMI + ARRAY_SIZE(r->spmi_bus)) {
		struct device_node *bus = of_parse_phandle(np, "apple,spmi-buses", b - T6050_RBLK_SPMI);

		if (bus && r->spmi_bus[b - T6050_RBLK_SPMI] && !of_address_to_resource(bus, 0, &res)) {
			r->rblk[b] = r->spmi_bus[b - T6050_RBLK_SPMI];
			r->rblk_size[b] = resource_size(&res);
		}
		of_node_put(bus);
	} else if (b >= T6050_RBLK_NODE && !of_address_to_resource(np, b - T6050_RBLK_NODE, &res)) {
		/* Shared with the drivers of those blocks: not claimed */
		r->rblk[b] = of_iomap(np, b - T6050_RBLK_NODE);
		r->rblk_size[b] = r->rblk[b] ? resource_size(&res) : 0;
	}
	return r->rblk[b];
}

static int t6050_ops_get(struct apple_sleep *s, const char *name, struct t6050_ops *ops)
{
	struct apple_sleep_region_state *r = s->region;
	int len = of_property_count_u32_elems(s->dev->of_node, name);
	unsigned int i;
	u32 *op;

	ops->n = 0;
	if (len <= 0)
		return 0;
	if (len % 4)
		return dev_err_probe(s->dev, -EINVAL, "%s: not in fours\n", name);
	op = devm_kcalloc(s->dev, len, sizeof(*op), GFP_KERNEL);
	if (!op)
		return -ENOMEM;
	if (of_property_read_u32_array(s->dev->of_node, name, op, len))
		return -EINVAL;
	/* A sequence naming a block this DT doesn't map is left out, not run in part. */
	for (i = 0; i < len; i += 4)
		if (FIELD_GET(T6050_OP_KIND, op[i]) >= T6050_OP_KINDS ||
		    !t6050_ops_block(s, op[i + 1]) || op[i + 2] % 2 ||
		    op[i + 2] + 4 > r->rblk_size[op[i + 1]]) {
			dev_warn(s->dev, "%s: entry %u outside its block, not used\n", name, i / 4);
			devm_kfree(s->dev, op);
			return 0;
		}
	ops->op = op;
	ops->n = len / 4;
	return 0;
}

/* The number of waits that timed out, other than the optional ones */
static unsigned int t6050_ops_run(struct apple_sleep *s, const struct t6050_ops *ops)
{
	unsigned int i, timeouts = 0;
	u32 v;

	for (i = 0; i < ops->n; i++) {
		const u32 *op = &ops->op[4 * i];
		void __iomem *reg = s->region->rblk[op[1]] + op[2];
		u32 ms = FIELD_GET(T6050_OP_TIMEOUT, op[0]) ?: 10;
		int ret = 0;

		switch (FIELD_GET(T6050_OP_KIND, op[0])) {
		case T6050_OP_WRITE:
			writel(op[3], reg);
			break;
		case T6050_OP_READ:
			readl(reg);
			break;
		case T6050_OP_SET:
			writel(readl(reg) | op[3], reg);
			break;
		case T6050_OP_CLEAR:
			writel(readl(reg) & ~op[3], reg);
			break;
		case T6050_OP_REWRITE:
			writel(readl(reg), reg);
			break;
		case T6050_OP_WAIT_SET:
			ret = readl_poll_timeout_atomic(reg, v, (v & op[3]) == op[3], 1, ms * 1000);
			break;
		case T6050_OP_WAIT_CLEAR:
			ret = readl_poll_timeout_atomic(reg, v, !(v & op[3]), 1, ms * 1000);
			break;
		case T6050_OP_WRITE16:
			writew(op[3], reg);
			break;
		}
		if (ret && !(op[0] & T6050_OP_OPTIONAL))
			timeouts++;
	}
	return timeouts;
}

/* The shared PHY's blocks, then every sequence the DT gives */
static int t6050_sleep_map_ops(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	static const char *const names[] = { "storage-phy", "storage-phy-ip" };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct resource res;
		int idx = of_property_match_string(s->dev->of_node, "reg-names", names[i]);

		if (idx >= 0 && !of_address_to_resource(s->dev->of_node, idx, &res)) {
			r->rblk[T6050_RBLK_SHARED + i] = i ? r->storage_phy_ip : r->storage_phy;
			r->rblk_size[T6050_RBLK_SHARED + i] = resource_size(&res);
		}
	}
	return t6050_ops_get(s, "apple,radio-link-down", &r->link_down) ?:
	       t6050_ops_get(s, "apple,radio-link-down-port", &r->link_down_port) ?:
	       t6050_ops_get(s, "apple,radio-port-wake", &r->port_wake) ?:
	       t6050_ops_get(s, "apple,radio-dart-wake", &r->dart_wake) ?:
	       t6050_ops_get(s, "apple,radio-phy-wake", &r->phy_wake) ?:
	       t6050_ops_get(s, "apple,phy-park", &r->phy_park) ?:
	       t6050_ops_get(s, "apple,phy-down", &r->phy_down);
}

static void t6050_seq(struct apple_sleep *s, unsigned int seq)
{
	t6050_ops_run(s, &s->region->seq[seq]);
}

/*
 * A word list from the DT: each (reg index << 20) | offset in a clock, perf or table bank, or,
 * where power words are allowed, 0x80000000 | the same for a power word. A list with an unknown
 * word is left out.
 */
static int t6050_words_get(struct apple_sleep *s, const char *name, struct apple_sleep_words *w,
			   bool power)
{
	const struct apple_sleep_region_profile *p = s->soc->regions;
	int n = of_property_count_u32_elems(s->dev->of_node, name), i, k;
	u32 *v;

	w->count = 0;
	if (n <= 0)
		return 0;
	v = devm_kcalloc(s->dev, n, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	if (of_property_read_u32_array(s->dev->of_node, name, v, n))
		return -EINVAL;
	for (i = 0; i < n; i++) {
		if (power && (v[i] & SLEEP_REPLAY_PS)) {
			k = sleep_ps_find(p->power.offset, p->power.count, v[i] & ~SLEEP_REPLAY_PS);
			if (k < 0)
				goto bad;
			v[i] = SLEEP_REPLAY_PS | k;
		} else if (!sleep_region_word_valid(p, v[i], false)) {
			goto bad;
		}
	}
	w->offset = v;
	w->count = n;
	return 0;
bad:
	dev_warn(s->dev, "%s: entry %d is not a known word, list not used\n", name, i);
	devm_kfree(s->dev, v);
	return 0;
}

static unsigned int t6050_u32s(struct apple_sleep *s, const char *name, u32 *v, unsigned int max)
{
	int n = of_property_read_variable_u32_array(s->dev->of_node, name, v, 1, max);

	return n > 0 ? n : 0;
}

/*
 * The margin from the SMC's idle notice to the timer, which no step of the schedule may reach,
 * is the device tree's (apple,commit-runway-us, where the boot loader is to put it). Without
 * the property it is the schedule's earliest step and this much.
 */
#define T6050_RUNWAY_OVER_US	1000
#define T6050_RUNWAY_MAX_US	USEC_PER_SEC

static struct apple_sleep_smc_policy t6050_smc_policy;

/*
 * The SoC data the boot loader gives: without the commit schedule and the SMC's spacings there
 * is no deep sleep; any other item missing leaves its step out.
 */
static int t6050_sleep_map_data(struct apple_sleep *s)
{
	static const char *const seqs[T6050_SEQS] = {
		[T6050_SEQ_BOOT] = "apple,seq-boot",
		[T6050_SEQ_STAGE] = "apple,seq-stage",
		[T6050_SEQ_WDT] = "apple,seq-commit-wdt",
		[T6050_SEQ_AFR] = "apple,seq-commit-afr",
		[T6050_SEQ_SIO] = "apple,seq-commit-sio",
		[T6050_SEQ_DISP_GATE] = "apple,seq-display-gate",
	};
	struct apple_sleep_region_state *r = s->region;
	struct device_node *np = s->dev->of_node;
	u32 v[16];
	unsigned int i, n;
	int ret;

	if (of_property_read_u32_array(np, "apple,commit-schedule-us", r->sched, T6050_STEPS))
		return dev_err_probe(s->dev, -ENODATA, "no commit schedule in the DT\n");
	r->runway_us = 0;
	of_property_read_u32(np, "apple,commit-runway-us", &r->runway_us);
	for (i = 0, n = 0; i < T6050_STEPS; i++) {
		if (r->runway_us && r->sched[i] >= r->runway_us)
			return dev_err_probe(s->dev, -EINVAL, "commit step %u too early\n", i);
		n = max(n, r->sched[i]);
	}
	if (!r->runway_us)
		r->runway_us = n + T6050_RUNWAY_OVER_US;
	if (r->runway_us > T6050_RUNWAY_MAX_US)
		return dev_err_probe(s->dev, -EINVAL, "commit runway too long\n");
	if (of_property_read_u32_array(np, "apple,smc-sleep-policy", v, 6))
		return dev_err_probe(s->dev, -ENODATA, "no SMC sleep spacings in the DT\n");
	t6050_smc_policy = (struct apple_sleep_smc_policy) {
		.pmvc = v[0], .warn_to_pmvc_us = v[1], .pmvc_to_slpa_us = v[2],
		.slpa_to_idle_us = v[3], .idle_to_timer_us = v[4], .waka_to_pmvc_us = v[5],
	};
	for (i = 0; i < T6050_SEQS; i++) {
		ret = t6050_ops_get(s, seqs[i], &r->seq[i]);
		if (ret)
			return ret;
	}
	ret = t6050_words_get(s, "apple,wake-order", &r->wake_order, true) ?:
	      t6050_words_get(s, "apple,entry-clk-words", &r->entry_clk, false) ?:
	      t6050_words_get(s, "apple,entry-perf-words", &r->entry_perf, false);
	if (ret)
		return ret;
	r->replay = devm_kcalloc(s->dev, max(r->wake_order.count, 1U), sizeof(u32), GFP_KERNEL);
	if (!r->replay)
		return -ENOMEM;
	/* PTD: the request entry, then the three clusters' and storage's bits in it */
	if (t6050_u32s(s, "apple,ptd-sleep", v, 5) == 5 && v[0] < 0x1000 &&
	    v[1] < 64 && v[2] < 64 && v[3] < 64 && v[4] < 64) {
		r->ptd_req = v[0];
		memcpy(r->ptd_bit, v + 1, sizeof(r->ptd_bit));
		r->ptd_valid = true;
	}
	n = t6050_u32s(s, "apple,pmu-sleep-reads", v, 16) / 2;
	for (i = 0, r->npmu_reads = 0; i < n; i++)
		if (v[2 * i] <= 0xffff && v[2 * i + 1] && v[2 * i + 1] <= 8) {
			r->pmu_reads[r->npmu_reads][0] = v[2 * i];
			r->pmu_reads[r->npmu_reads++][1] = v[2 * i + 1];
		}
	n = t6050_u32s(s, "apple,spmi-sleep-order", v, ARRAY_SIZE(r->spmi_order));
	for (i = 0, r->nspmi_order = 0; i < n; i++)
		if (v[i] < ARRAY_SIZE(r->spmi_bus))
			r->spmi_order[r->nspmi_order++] = v[i];
	if (!r->nspmi_order)
		for (; r->nspmi_order < ARRAY_SIZE(r->spmi_bus); r->nspmi_order++)
			r->spmi_order[r->nspmi_order] = r->nspmi_order;
	r->spmi_gap_us = 0;
	of_property_read_u32(np, "apple,spmi-sleep-gap-us", &r->spmi_gap_us);
	r->spmi_gap_us = min(r->spmi_gap_us, 2000U);
	r->spmi_runtime_mask = 0;
	of_property_read_u32(np, "apple,spmi-runtime-mask", &r->spmi_runtime_mask);
	/* One of nub-spmi0's peripheral status words, with at least one bit: else none acked */
	r->spmi_pmu_bits = 0;
	if (t6050_u32s(s, "apple,spmi-pmu-sources", v, 2) == 2 && v[1] &&
	    v[0] >= M5_SPMI_SRC_STATUS && v[0] < M5_SPMI_IRQ_STATUS && !(v[0] % 4)) {
		r->spmi_pmu_status = v[0];
		r->spmi_pmu_bits = v[1];
	}
	r->pmu_wake_stage_valid = !of_property_read_u32(np, "apple,pmu-wake-stage",
							&r->pmu_wake_stage) && r->pmu_wake_stage <= 0xff;
	r->alarm_rearm_valid = t6050_u32s(s, "apple,pmu-alarm-rearm", r->alarm_rearm, 2) == 2 &&
			       r->alarm_rearm[0] <= 0xff && r->alarm_rearm[1] <= 0xff;
	n = t6050_u32s(s, "apple,complex-requests", v, 8) / 2;
	for (i = 0; i < n; i++) {
		r->complex_map[i][0] = v[2 * i];
		r->complex_map[i][1] = v[2 * i + 1];
	}
	r->ncomplex = n;
	r->nfloor_steps = t6050_u32s(s, "apple,perf-floor-steps", r->floor_steps,
				     ARRAY_SIZE(r->floor_steps));
	r->floor_step_ms = 0;
	of_property_read_u32(np, "apple,perf-floor-step-ms", &r->floor_step_ms);
	r->floor_step_ms = min(r->floor_step_ms, 20U);
	n = t6050_u32s(s, "apple,pmc-sleep-channels", v, ARRAY_SIZE(r->pmc_channels));
	for (i = 0, r->npmc_channels = 0; i < n; i++)
		if (v[i] < 8)
			r->pmc_channels[r->npmc_channels++] = v[i];
	/* Each stopped DART's GAPF slice, from its node */
	for (i = 0; i < ARRAY_SIZE(r->dart_slice); i++) {
		struct device_node *dn = i < T6050_DART_APCIE0 ?
			of_parse_phandle(np, "apple,sleep-darts", i) :
			(r->radio_dart ? of_node_get(r->radio_dart->dev.of_node) : NULL);
		u32 off[2];

		r->dart_slice[i] = 0;
		if (dn && !of_property_read_u32(dn, "apple,gapf-slice", &v[0]) && v[0] < 0x4000 &&
		    !(v[0] % 4))
			r->dart_slice[i] = v[0];
		if (dn && !of_property_read_u32_array(dn, "apple,gapf-slice-values", off, 2))
			r->dart_slice_off = off[1];
		of_node_put(dn);
	}
	return 0;
}

static void __iomem *t6050_radio_block(struct apple_sleep *s, struct device_node *np,
				       const char *name, unsigned int blk, u32 size)
{
	struct apple_sleep_region_state *r = s->region;
	struct resource res;
	int idx = of_property_match_string(np, "reg-names", name);

	if (idx < 0 || of_address_to_resource(np, idx, &res))
		return NULL;
	r->rblk_size[blk] = size ?: resource_size(&res);
	r->rblk[blk] = ioremap(res.start, r->rblk_size[blk]);
	return r->rblk[blk];
}

/*
 * The radio's PCIe port, for its link shutdown: the host's root port config page, port and PHY
 * registers by name, its PERST# pin, and the DART its functions are behind.
 */
static int t6050_sleep_map_radio(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	struct device_node *np, *port;
	struct of_phandle_args args, dart = {};
	struct platform_device *pdev;
	struct resource res;
	int ret = 0;

	np = of_parse_phandle(s->dev->of_node, "apple,radio-pcie", 0);
	if (!np)
		return 0;
	if (!of_device_is_available(np))
		goto out;
	r->n1_ecam = t6050_radio_block(s, np, "config", T6050_RBLK_CONFIG, SZ_4K);
	r->n1_port = t6050_radio_block(s, np, "port0", T6050_RBLK_PORT, 0x24000);
	r->n1_phy = t6050_radio_block(s, np, "phy0", T6050_RBLK_PHY, 0);
	r->n1_rc = t6050_radio_block(s, np, "rc", T6050_RBLK_RC, 0);
	/* The four ports' PHY words from phy0, and the PHY's common page below them */
	if (!of_address_to_resource(np, of_property_match_string(np, "reg-names", "phy0"), &res) &&
	    (res.start & 0xfffff) == 0x10000) {
		r->rblk[T6050_RBLK_LANES] = ioremap(res.start, 0x10000);
		r->rblk_size[T6050_RBLK_LANES] = 0x10000;
		r->rblk[T6050_RBLK_PHY_CMN] = ioremap(res.start - 0xc000, 0x4000);
		r->rblk_size[T6050_RBLK_PHY_CMN] = 0x4000;
	}
	r->radio_host = of_find_device_by_node(np);
	port = of_get_next_child(np, NULL);
	if (port && !of_parse_phandle_with_args(port, "reset-gpios", "#gpio-cells", 0, &args)) {
		r->n1_perst = args.args[0];
		of_node_put(args.np);
	}
	of_node_put(port);
	if (!of_map_iommu_id(np, 0x100, &dart)) {
		pdev = of_find_device_by_node(dart.np);
		r->dart[T6050_DART_APCIE0] = of_iomap(dart.np, 0);
		of_node_put(dart.np);
		if (!pdev) {
			ret = -EPROBE_DEFER;
			goto out;
		}
		r->radio_dart = pdev;
	}
	if (!r->n1_ecam || !r->n1_port || !r->n1_phy || !r->n1_perst)
		ret = -EINVAL;
out:
	of_node_put(np);
	return ret;
}

static void t6050_wake_masks(struct apple_sleep *s);

/* ANS's power word and the parents the DT gives it, which its wake depends on */
static void t6050_ans_chain(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct device_node *np, *found = NULL;
	int word, n;

	r->nans_chain = 0;
	for_each_compatible_node(np, NULL, "apple,t6050-pmgr-pwrstate")
		if (sleep_region_domain_offset(s, np) == s->soc->ps.ans) {
			found = of_node_get(np);
			of_node_put(np);
			break;
		}
	np = found;
	while (np && r->nans_chain < ARRAY_SIZE(r->ans_chain)) {
		struct device_node *parent;

		word = sleep_region_domain_offset(s, np);
		n = word < 0 ? -1 : sleep_ps_find(p->power.offset, p->power.count, word);
		if (n >= 0)
			r->ans_chain[r->nans_chain++] = n;
		parent = of_parse_phandle(np, "power-domains", 0);
		of_node_put(np);
		np = parent;
	}
	of_node_put(np);
}

static int t6050_sleep_map_regs(struct apple_sleep *s)
{
	static const struct { const char *name; size_t offset; u32 minimum; } regs[] = {
		{ "cluster", offsetof(struct apple_sleep, cluster), 0x20 },
		{ "pwrgate", offsetof(struct apple_sleep, pwrgate), 0x134 },
		{ "cpu-start", offsetof(struct apple_sleep, cpu_start), 0x14 },
		{ "commit", offsetof(struct apple_sleep, commit), 4 },
		{ "wdt", offsetof(struct apple_sleep, wdt), 0x100 },
		{ "wdt-aux", offsetof(struct apple_sleep, wdt_aux), 0x30 },
		{ "disp-gate", offsetof(struct apple_sleep, disp_gate), 0x544 },
		{ "spmi", offsetof(struct apple_sleep, spmi), 0x224 },
		{ "pmc-voters", offsetof(struct apple_sleep, pmc_voters), 0x10008 },
		{ "pmc-rails", offsetof(struct apple_sleep, pmc_rails), 0x202c },
		{ "ptd", offsetof(struct apple_sleep, ptd), 0x10f08 },
	};
	unsigned int i;
	int ret = sleep_region_map(s);

	BUILD_BUG_ON(T6050_RBLK_SPMI + ARRAY_SIZE(s->region->spmi_bus) > T6050_RBLK_NODE);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		struct resource resource;
		void __iomem **field = (void __iomem **)((u8 *)s + regs[i].offset);
		int index = of_property_match_string(s->dev->of_node, "reg-names", regs[i].name);

		if (index < 0 || of_address_to_resource(s->dev->of_node, index, &resource) ||
		    resource_size(&resource) < regs[i].minimum)
			return dev_err_probe(s->dev, -EINVAL, "missing/short %s aperture\n", regs[i].name);
		*field = apple_sleep_map(s->dev, regs[i].name);
		if (IS_ERR(*field))
			return PTR_ERR(*field);
	}
	if (of_property_match_string(s->dev->of_node, "reg-names", "gapf") >= 0) {
		s->region->gapf = apple_sleep_map(s->dev, "gapf");
		if (IS_ERR(s->region->gapf))
			s->region->gapf = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(s->region->dart); i++) {
		struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,sleep-darts", i);

		if (!np)
			break;
		/* Shared with the DART driver, so not claimed. */
		s->region->dart[i] = of_iomap(np, 0);
		of_node_put(np);
	}
	for (i = 0; i < ARRAY_SIZE(s->region->spmi_bus); i++) {
		struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,spmi-buses", i);

		if (!np)
			break;
		/* Each has its own driver, or none: not claimed. */
		s->region->spmi_bus[i] = of_iomap(np, 0);
		of_node_put(np);
	}
	{
		/* nub-spmi0's controller: the node at this node's "spmi" block. Kept. */
		struct resource *own = platform_get_resource_byname(to_platform_device(s->dev),
								    IORESOURCE_MEM, "spmi");
		struct platform_device *bus;
		struct device_node *np;
		struct resource res;

		for_each_compatible_node(np, NULL, "apple,t8142-spmi") {
			if (!own || of_address_to_resource(np, 0, &res) || res.start != own->start)
				continue;
			bus = of_find_device_by_node(np);
			if (bus)
				s->region->pmu_bus = &bus->dev;
			of_node_put(np);
			break;
		}
	}
	{
		/* Blocks other drivers also map, or none does: not claimed. */
		static const struct { const char *name; size_t off; } opt[] = {
			{ "aon-clk", offsetof(struct apple_sleep_region_state, aon_clk[0]) },
			{ "media-aon-clk", offsetof(struct apple_sleep_region_state, aon_clk[1]) },
			{ "cpm0", offsetof(struct apple_sleep_region_state, cpm[0]) },
			{ "cpm1", offsetof(struct apple_sleep_region_state, cpm[1]) },
			{ "cpm2", offsetof(struct apple_sleep_region_state, cpm[2]) },
			{ "storage-phy", offsetof(struct apple_sleep_region_state, storage_phy) },
			{ "storage-phy-ip", offsetof(struct apple_sleep_region_state, storage_phy_ip) },
			{ "clkgen-78", offsetof(struct apple_sleep_region_state, clkgen78) },
			{ "media-clkgen-78", offsetof(struct apple_sleep_region_state, media_clkgen78) },
			{ "mcc22", offsetof(struct apple_sleep_region_state, mcc[0]) },
			{ "mcc23", offsetof(struct apple_sleep_region_state, mcc[1]) },
			{ "mcc24", offsetof(struct apple_sleep_region_state, mcc[2]) },
			{ "mcc25", offsetof(struct apple_sleep_region_state, mcc[3]) },
			{ "mcc26", offsetof(struct apple_sleep_region_state, mcc[4]) },
			{ "wake", offsetof(struct apple_sleep_region_state, wake) },
			{ "wake-gfx", offsetof(struct apple_sleep_region_state, wake_gfx) },
			{ "aic-gfx", offsetof(struct apple_sleep_region_state, aic_gfx) },
		};
		int idx;

		for (i = 0; i < ARRAY_SIZE(opt); i++) {
			idx = of_property_match_string(s->dev->of_node, "reg-names", opt[i].name);
			if (idx >= 0)
				*(void __iomem **)((u8 *)s->region + opt[i].off) =
					of_iomap(s->dev->of_node, idx);
		}
	}
	{
		struct device_node *np = of_parse_phandle(s->dev->of_node, "apple,pinctrl", 0);

		if (np) {
			s->region->ap_gpio = of_iomap(np, 0);
			of_node_put(np);
		}
	}
	ret = of_property_read_variable_u32_array(s->dev->of_node, "apple,sleep-low-pins",
						  s->region->pins_low, 0,
						  ARRAY_SIZE(s->region->pins_low));
	s->region->npins_low = max(ret, 0);
	t6050_wake_masks(s);
	t6050_ans_chain(s);
	/* The clusters' CPM set-up, where the boot loader puts it: no entry, no write */
	ret = t6050_cpm_prop(s, "apple,cpm-boot", &s->region->cpm_boot, &s->region->ncpm_boot);
	if (ret)
		return ret;
	ret = t6050_sleep_map_radio(s) ?: t6050_sleep_map_ops(s) ?: t6050_sleep_map_data(s);
	if (ret)
		return ret;
	t6050_sleep_boot_auto(s);
	return 0;
}


#define M5_PMC_STRIDE 0x4000

static int t6050_sleep_ptd_ownership(struct apple_sleep *s, bool own)
{
	struct apple_sleep_region_state *r = s->region;
	const u32 *ids = r->ptd_bit;
	struct device_node *report, *entry;
	unsigned int i;
	int ret = 0;

	if (!own) {
		for (i = 0; i < ARRAY_SIZE(r->ptd_entry); i++)
			if (r->ptd_entry[i]) {
				apple_pmp_report_entry_commit_owned(r->ptd_entry[i], false);
				of_node_put(r->ptd_entry[i]);
				r->ptd_entry[i] = NULL;
			}
		r->ptd_owned = false;
		return 0;
	}
	if (!r->ptd_valid)
		return -ENODEV;
	report = of_find_compatible_node(NULL, NULL, "apple,t6050-pmp-v2-report");
	if (!report)
		return -ENODEV;
	/* Nothing else writes the entries: the gate refuses a PMP that runs unbound. */
	if (s->soc->pmp_unstarted && !of_device_is_available(report)) {
		of_node_put(report);
		r->ptd_owned = true;
		return 0;
	}
	for_each_child_of_node(report, entry) {
		u32 id;

		if (of_property_read_u32(entry, "reg", &id))
			continue;
		for (i = 0; i < ARRAY_SIZE(r->ptd_entry); i++)
			if (id == ids[i]) {
				ret = apple_pmp_report_entry_commit_owned(entry, true);
				if (ret) {
					of_node_put(entry);
					goto out;
				}
				r->ptd_entry[i] = of_node_get(entry);
			}
	}
	for (i = 0; i < ARRAY_SIZE(r->ptd_entry); i++)
		if (!r->ptd_entry[i])
			ret = -ENODEV;
out:
	of_node_put(report);
	if (ret)
		t6050_sleep_ptd_ownership(s, false);
	else
		r->ptd_owned = true;
	return ret;
}

#define SYS_APL_AGT_CTL_EL02		sys_reg(3, 4, 15, 2, 7)
#define SYS_APL_AGTCNTP_CTL_EL02	sys_reg(3, 4, 15, 4, 3)

/*
 * EL1's Apple timers off on the boot core before the sleep; under Linux they keep iBoot's
 * state. Done on the core itself, in a bound worker.
 */
static struct {
	struct work_struct work;
	struct completion done;
} t6050_agt_off;

static void t6050_agt_off_work(struct work_struct *w)
{
	write_sysreg_s(0, SYS_APL_AGT_CTL_EL02);
	write_sysreg_s(0, SYS_APL_AGTCNTP_CTL_EL02);
	isb();
	complete(&t6050_agt_off.done);
}

static void t6050_sleep_boot_timer_off(struct apple_sleep *s)
{
	int cpu;

	if (!is_kernel_in_hyp_mode())
		return;
	for_each_online_cpu(cpu) {
		if (cpu_logical_map(cpu) != s->mbox->boot_mpidr)
			continue;
		INIT_WORK(&t6050_agt_off.work, t6050_agt_off_work);
		init_completion(&t6050_agt_off.done);
		queue_work_on(cpu, system_highpri_wq, &t6050_agt_off.work);
		if (!wait_for_completion_timeout(&t6050_agt_off.done, 2 * HZ))
			dev_warn(s->dev, "boot core's timers not turned off\n");
		return;
	}
}

/* WD2 from the commit: a net until the sleep engine arms it, long enough for the resume. */
#define T6050_WDT_COMMIT_SECS	13

static void t6050_sleep_n1_down(struct apple_sleep *s, bool perst_alone);
static bool t6050_n1_host_bound(struct apple_sleep *s);

#define T6050_GPIO_DATA		BIT(0)
#define T6050_GPIO_MODE		GENMASK(3, 1)
#define T6050_GPIO_MODE_OUT	1

/* A pin driven high goes low; one in any other state is left alone. */
static void t6050_pin_low(void __iomem *pin)
{
	u32 v = readl(pin);

	if (FIELD_GET(T6050_GPIO_MODE, v) == T6050_GPIO_MODE_OUT && (v & T6050_GPIO_DATA))
		writel(v & ~T6050_GPIO_DATA, pin);
}

/* The AP pins macOS drives low in its sleep entry. */
static void t6050_sleep_pins_low(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	for (i = 0; r->ap_gpio && i < r->npins_low; i++)
		t6050_pin_low(r->ap_gpio + 4 * r->pins_low[i]);
}

static int t6050_sleep_begin(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	int ret;

	t6050_sleep_pins_low(s);
	r->n1_up_ret = 0;
	r->n1_host = t6050_n1_host_bound(s);
	if (r->n1_host) {
		/* Its functions are in use until their own suspend: the host's noirq suspend runs it. */
		r->n1_held = 0;
		r->n1_state = 5;
	} else {
		t6050_sleep_n1_down(s, true);
		/* The port's DART is clocked by the port: its restore waits for a host. */
		if (r->radio_dart && (!r->n1_state || r->n1_state == 4))
			apple_dart_set_host_resumes(&r->radio_dart->dev);
	}

	/* The GFX die's interrupt controller configuration, written back at the wake. */
	if (r->aic_gfx)
		r->gfx_aic[0] = readl(r->aic_gfx + 0x14);
	/* Wake writes these back: iBoot's requests. */
	for (i = 0; i < 5; i++)
		r->cluster_saved[i] = readl(s->cluster + 4 * i);
	s->ans_m5_wake_error = -EAGAIN;
	s->ans_off_ret = 0;
	r->staged = false;
	r->committed = false;
	r->early_done = false;
	r->early_dropped = false;
	r->venc0_lowered = false;
	r->wake_status_valid = false;
	r->wake_status_fresh = false;
	r->spmi_src_valid = false;
	r->pmu_wake_pending = false;
	r->pmc_saved = false;
	r->floor_changed = 0;
	r->display_lowered = 0;
	bitmap_zero(r->manager_down, s->soc->regions->power.count);
	r->late_errors = 0;
	r->auss_lowered = 0;
	r->refusal_cycled = 0;
	r->pmu_errors = 0;
	r->alarm_valid = false;
	r->boot_stage_written = false;
	s->regs_staged = s->entry_prepared = s->wake_masks_saved = false;
	s->smc_idle_sent = s->smc_idle_ack = 0;
	for (i = 0; i < s->soc->ndisp; i++)
		s->display[i].quiesced = false;
	t6050_sleep_boot_timer_off(s);
	/* Preserve actual driver ownership bitmap, including a prior incomplete recovery. */
	ret = t6050_sleep_ptd_ownership(s, true);
	r->attempt_active = !ret;
	return ret;
}

static void t6050_pmc_floor(struct apple_sleep *s, unsigned int agent)
{
	s->region->floor_changed |= BIT(agent);
	writel(0, s->pmc_voters + M5_PMC_STRIDE * agent);
	readl(s->pmc_voters + M5_PMC_STRIDE * agent + 4);
}

/* The ADT's no_ps PMGR devices with real PS-bank offsets: 154 words */
static const struct { u32 word; const char *name; } t6050_no_ps[] = {
	{ SLEEP_REGION_WORD(T6050_PS0, 0x100), "PMS" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x188), "FAB0_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x190), "FAB0_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x198), "FAB0_AFI_2" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1a0), "FAB0_AFI_3" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1a8), "FAB_B2BC_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1b0), "FAB_B2BC_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1b8), "B2BC_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1c0), "B2BC_AFI_0_DFX" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1c8), "B2BC_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1d0), "B2BC_AFI_1_DFX" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1d8), "FAB0_PIOGW" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1e0), "FAB3_SOC" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x1f8), "SBR" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x308), "DISPEXT0_DBE" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x310), "DPEXT0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x318), "DISPEXT0_GP0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x320), "DISPEXT0_GP1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x328), "DISPEXT0_GPL" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x330), "DISPEXT0_PPP" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x368), "DISPEXT2_DBE" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x370), "DPEXT2" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x378), "DISPEXT2_GP0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x380), "DISPEXT2_GP1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x388), "DISPEXT2_GPL" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x390), "DISPEXT2_PPP" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x398), "DISPEXT3_DBE" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3a0), "DPEXT3" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3a8), "DISPEXT3_GP0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3b0), "DISPEXT3_GP1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3b8), "DISPEXT3_GPL" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3c0), "DISPEXT3_PPP" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3c8), "DISPEXT1_DBE" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3d0), "DPEXT1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3d8), "DISPEXT1_GP0" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3e0), "DISPEXT1_GP1" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3e8), "DISPEXT1_GPL" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x3f0), "DISPEXT1_PPP" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x428), "DISPEXT0_DSC" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x430), "DISPEXT2_DSC" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x438), "DISPEXT3_DSC" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x440), "DISPEXT1_DSC" },
	{ SLEEP_REGION_WORD(T6050_PS0, 0x448), "FAB0_AFIGW" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x120), "APCIE_GE" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x188), "APCIE_SYS_GE" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1b8), "APCIE_SYS_GE_OL" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1c8), "DISP_SPI" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1d0), "DISP_SPMI" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1d8), "DISP_SMB0" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1e0), "DISP_SMB1" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1e8), "DISP_DBE" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1f0), "DP" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x1f8), "DISP_GP0" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x200), "DISP_GP1" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x208), "DISP_GPL" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x210), "DISP_CMB" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x218), "DISP_PPP" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x220), "DISP_BRC" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x228), "DISP_BLC" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x230), "DISP_HILO" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x238), "DISP_RCA" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x250), "APHY_GE_AUS5_A" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x260), "DISP_DSC" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x278), "APHY_GE_AUS5" },
	{ SLEEP_REGION_WORD(T6050_PS1, 0x2b0), "FPWM1" },
	{ SLEEP_REGION_WORD(T6050_PS_AON, 0x78), "NUB_AON" },
	{ SLEEP_REGION_WORD(T6050_PS_AON, 0x90), "NUB_OCLA" },
	{ SLEEP_REGION_WORD(T6050_PS_AON, 0x1a8), "NUB_FABRIC" },
	{ SLEEP_REGION_WORD(T6050_PS_AON, 0x1b0), "NUB_SRAM" },
	{ SLEEP_REGION_WORD(T6050_PS_AON, 0x1b8), "DEBUG_SWITCH" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x100), "PMS_GFX" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x150), "FAB1_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x158), "FAB1_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x160), "FAB_B2BG_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x168), "FAB_B2BG_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x170), "B2BG_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x178), "B2BG_AFI_0_DFX" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x180), "B2BG_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x190), "FAB1_AFIGW" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x1a0), "FAB1_PIOGW" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x1b8), "FAB2_SOC" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x1e8), "SBR_GFX" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x1f0), "VENC1_SYS" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x210), "FAB_AFI_AMCC_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x248), "DCS_00" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x258), "DCS_01" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x268), "DCS_02" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x278), "DCS_03" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x290), "FAB_AFI_AMCC_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x2c8), "DCS_04" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x2d8), "DCS_05" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x2e8), "DCS_06" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x2f8), "DCS_07" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x310), "FAB_AFI_AMCC_2" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x348), "DCS_08" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x358), "DCS_09" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x368), "DCS_10" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x378), "DCS_11" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x390), "FAB_AFI_AMCC_3" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x3c8), "DCS_12" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x3d8), "DCS_13" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x3e8), "DCS_14" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x3f8), "DCS_15" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x410), "FAB_AFI_AMCC_4" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x418), "AMCC16" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x420), "AMCC17" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x428), "AMCC18" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x430), "AMCC19" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x448), "DCS_16" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x458), "DCS_17" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x468), "DCS_18" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x478), "DCS_19" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x490), "FAB_AFI_AMCC_5" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x498), "AMCC20" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4a0), "AMCC21" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4a8), "AMCC22" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4b0), "AMCC23" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4c8), "DCS_20" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4d8), "DCS_21" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4e8), "DCS_22" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x4f8), "DCS_23" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x510), "FAB_AFI_AMCC_6" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x518), "AMCC24" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x520), "AMCC25" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x528), "AMCC26" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x530), "AMCC27" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x548), "DCS_24" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x558), "DCS_25" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x568), "DCS_26" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x578), "DCS_27" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x590), "FAB_AFI_AMCC_7" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x598), "AMCC28" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5a0), "AMCC29" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5a8), "AMCC30" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5b0), "AMCC31" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5d0), "DCS_28" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5e0), "DCS_29" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x5f0), "DCS_30" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x600), "DCS_31" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x618), "FAB_B2BD_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x620), "FAB_B2BD_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x628), "B2BD_AFI_0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x630), "B2BD_AFI_0_DFX" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x638), "B2BD_AFI_1" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x640), "B2BD_AFI_1_DFX" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x8028), "VENC1_DMA" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x8030), "VENC1_PIPE4" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x8038), "VENC1_PIPE5" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x8040), "VENC1_ME0" },
	{ SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x8048), "VENC1_ME1" },
	{ SLEEP_REGION_WORD(T6050_PS_GFX_AON, 0x60), "NUB_GFX_AON" },
	{ SLEEP_REGION_WORD(T6050_PS_GFX_AON, 0x70), "NUB_GFX_OCLA" },
	{ SLEEP_REGION_WORD(T6050_PS_GFX_AON, 0x78), "NUB_GFX_FABRIC" },
	{ SLEEP_REGION_WORD(T6050_PS_GFX_AON, 0x80), "DEBUG_GFX_SWITCH" },
};
/* /arm-io/pmgr device-check-ps-b4-sleep: each display set's children, as indices into t6050_no_ps; ps.disp[] order */
static const u8 t6050_disp_children[5][16] = {
	{ 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 62 },
	{ 14, 15, 16, 17, 18, 19, 38 },
	{ 32, 33, 34, 35, 36, 37, 41 },
	{ 20, 21, 22, 23, 24, 25, 39 },
	{ 26, 27, 28, 29, 30, 31, 40 },
};
static const u8 t6050_disp_nchildren[5] = { 16, 7, 7, 7, 7 };

static int t6050_sleep_entry_lower(struct apple_sleep *s);

/*
 * Driver-provided readiness is required; never clear a running DCP's RUN bit. One set's words,
 * right after its own firmware stop, as macOS lowers each set's after its idle.
 */
static int t6050_sleep_display_off_one(struct apple_sleep *s, unsigned int d)
{
	struct apple_sleep_region_state *r = s->region;
	u32 cpu = s->soc->ps.disp[d][0];
	u32 val = readl(sleep_region_reg(s, cpu));
	unsigned int j;
	int ret;

	if (!FIELD_GET(PS_TARGET, val) && !FIELD_GET(PS_ACTUAL, val))
		return 0;
	if (!s->display[d].bound || !READ_ONCE(s->display[d].quiesced) || !s->display[d].asc ||
	    (readl(s->display[d].asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN))
		return -EBUSY;
	if (!s->entry_committed)
		return -EPERM;
	/*
	 * The ADT's device-check-ps-b4-sleep children of this set: any that reads on goes down first.
	 */
	for (j = 0; j < t6050_disp_nchildren[d]; j++) {
		u32 child = t6050_no_ps[t6050_disp_children[d][j]].word;
		u32 cv = readl(sleep_region_reg(s, child));

		if (!FIELD_GET(PS_TARGET, cv) && !FIELD_GET(PS_ACTUAL, cv))
			continue;
		r->disp_child_saved[d][j] = cv;
		sleep_ps_lower(s, child);
		r->disp_children_lowered[d] |= BIT(j);
		r->late_timeouts += !!sleep_ps_wait_off(s, child);
	}
	/* Snapshot belongs to the returning pre-SEP phase. */
	r->display_lowered |= BIT(d);
	for (j = 0; j < 3; j++) {
		int index = sleep_ps_find(s->soc->regions->power.offset,
				  s->soc->regions->power.count, s->soc->ps.disp[d][j]);

		if (index >= 0)
			set_bit(index, r->manager_down);
	}
	val &= ~(PS_AUTO_ENABLE | PS_FLAGS);
	writel(val, sleep_region_reg(s, cpu));
	if (!d) {
		/* Internal gate snapshot was taken before SEP: the DT's display gate sequence */
		t6050_seq(s, T6050_SEQ_DISP_GATE);
	}
	writel((val & ~PS_TARGET) | PS_FLAGS, sleep_region_reg(s, cpu));
	ret = sleep_ps_wait_off(s, cpu);
	/* FE, then SYS, on every set, right after its CPU word, as the internal DISP's */
	for (j = 1; !ret && j < 3; j++) {
		sleep_ps_lower(s, s->soc->ps.disp[d][j]);
		ret = sleep_ps_wait_off(s, s->soc->ps.disp[d][j]);
	}
	return ret;
}

/*
 * A display word ready for the V27 stop: target 0xf, not disabled, and actually on. An external
 * set (d > 0) with no display attached idles under automatic power: target 0xf, AUTO_ENABLE and a
 * lower actual state. macOS's own stop of those words starts by
 * clearing AUTO_ENABLE, so that state is ready too.
 */
static bool t6050_display_word_ready(unsigned int d, u32 value)
{
	if (FIELD_GET(PS_TARGET, value) != 0xf || (value & PS_DEV_DISABLE))
		return false;
	if (FIELD_GET(PS_ACTUAL, value) == 0xf)
		return true;
	return d && (value & PS_AUTO_ENABLE) && FIELD_GET(PS_ACTUAL, value);
}

static int t6050_sleep_display_prepare(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int d, j;
	int ret;

	if (!apple_sleep_v27_late() || s->entry_committed)
		return -EOPNOTSUPP;
	for (d = 0; d < s->soc->ndisp; d++) {
		ret = sleep_v27_final_check(d);
		if (ret) {
			dev_info(s->dev, "display %u: final check %d\n", d, ret);
			return ret;
		}
		for (j = 0; j < 3; j++) {
			u32 value = readl(sleep_region_reg(s, s->soc->ps.disp[d][j]));

			if (!t6050_display_word_ready(d, value)) {
				dev_info(s->dev, "display %u: word %u reads %#x\n", d, j, value);
				return -EAGAIN;
			}
			r->display_saved[d][j] = value;
		}
	}
	r->gate_saved[0] = readl(s->disp_gate + 0x51c);
	r->gate_saved[1] = readl(s->disp_gate + 0x540);
	memset(r->disp_children_lowered, 0, sizeof(r->disp_children_lowered));
	sleep_region_snapshot(s);
	return t6050_sleep_entry_lower(s);
}

/*
 * The PMC's channel requests (+0x154: bit n on, bit 8 + n its off request; +0x158 the
 * channels' idle bits). The DT's apple,pmc-sleep-channels go down, each from its own write,
 * and only those that read on. Linux never writes it otherwise.
 */
#define T6050_PMC_CHANNELS		0x154
#define T6050_PMC_CHANNEL_IDLE		0x158
#define   T6050_PMC_CHANNEL_ON(n)	BIT(n)
#define   T6050_PMC_CHANNEL_OFF(n)	BIT(8 + (n))
#define   T6050_PMC_CHANNELS_IDLE	GENMASK(3, 0)

static void t6050_pmc_154(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	void __iomem *req = s->pmc_rails + T6050_PMC_CHANNELS;
	u32 v = readl(req), status;
	unsigned int i, n;

	r->pmc154[0] = v;
	r->pmc154[1] = readl(s->pmc_rails + T6050_PMC_CHANNEL_IDLE);
	r->pmc154_lowered = 0;
	for (i = 0; i < r->npmc_channels; i++) {
		n = r->pmc_channels[i];
		if (!(v & T6050_PMC_CHANNEL_ON(n)))
			continue;
		r->pmc154_lowered |= BIT(n);
		writel((v & ~T6050_PMC_CHANNEL_ON(n)) | T6050_PMC_CHANNEL_OFF(n), req);
		readl_poll_timeout_atomic(s->pmc_rails + T6050_PMC_CHANNEL_IDLE, status,
					  status == T6050_PMC_CHANNELS_IDLE, 1, 2000);
		v = readl(req);
	}
}

/*
 * The memory performance floor stepped down through the DT's apple,perf-floor-steps, then 0.
 * Stepped from any floor, 0 included: the memory controller follows the writes, not the value
 * read back (which stays 0).
 */
static void t6050_floor_steps(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	void __iomem *reg = s->pmc_voters + M5_PMC_STRIDE * 1;
	unsigned int i;

	r->floor_changed |= BIT(1);
	for (i = 0; i < r->nfloor_steps; i++) {
		writel(r->floor_steps[i], reg);
		mdelay(r->floor_step_ms);
	}
	t6050_pmc_floor(s, 1);
}

/*
 * The AP wake-enable words (1 = masked), one bit per wake source, ten on this die and one on
 * the GFX die. iBoot leaves them 0, every source enabled, and a pending source then turns the
 * entry round at once. At the stage every implemented source (the bits that read back set after
 * writing all ones) but the DT's apple,wake-enable (and apple,wake-enable-gfx) is masked; the
 * saved values go back at the wake. The sources come from the boot loader: without them the
 * words are left alone.
 */
#define T6050_WAKE_ENABLE	0x0c
#define T6050_WAKE_WORDS	10

static void t6050_wake_masks(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u32 ids[64];
	int n, i;

	/* The sources left enabled, as bits; the implemented ones are found when arming. */
	for (i = 0; i <= T6050_WAKE_WORDS; i++)
		r->wake_enabled[i] = 0;
	n = of_property_read_variable_u32_array(s->dev->of_node, "apple,wake-enable", ids, 0,
						ARRAY_SIZE(ids));
	r->wake_sources = n > 0;
	if (!r->wake_sources)
		dev_warn(s->dev, "no wake sources in the DT: the AP wake masks stay as iBoot left them\n");
	for (i = 0; i < n; i++)
		if (ids[i] < 32 * T6050_WAKE_WORDS)
			r->wake_enabled[ids[i] / 32] |= BIT(ids[i] % 32);
	n = of_property_read_variable_u32_array(s->dev->of_node, "apple,wake-enable-gfx", ids, 0,
						ARRAY_SIZE(ids));
	for (i = 0; i < n; i++)
		if (ids[i] < 32)
			r->wake_enabled[T6050_WAKE_WORDS] |= BIT(ids[i]);
}

static void t6050_wake_arm(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	u32 implemented;

	if (!r->wake || !r->wake_gfx || !r->wake_sources)
		return;
	for (i = 0; i <= T6050_WAKE_WORDS; i++) {
		void __iomem *reg = i < T6050_WAKE_WORDS ? r->wake + T6050_WAKE_ENABLE + 4 * i :
				    r->wake_gfx + T6050_WAKE_ENABLE;

		r->wake_saved[i] = readl(reg);
		/* Every implemented source masked, then the enabled ones cleared */
		writel(U32_MAX, reg);
		implemented = readl(reg);
		writel(implemented & ~r->wake_enabled[i], reg);
		dev_dbg(s->dev, "wake word %u: implemented %#x, enabled %#x\n", i, implemented,
			r->wake_enabled[i]);
	}
	r->wake_armed = true;
}

static void t6050_wake_disarm(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	if (!r->wake_armed)
		return;
	for (i = 0; i < T6050_WAKE_WORDS; i++)
		writel(r->wake_saved[i], r->wake + T6050_WAKE_ENABLE + 4 * i);
	writel(r->wake_saved[T6050_WAKE_WORDS], r->wake_gfx + T6050_WAKE_ENABLE);
	r->wake_armed = false;
}

static int t6050_sleep_stage(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	u32 park;

	r->staged = true;
	for (i = 0; i < 5; i++)
		r->floor[i] = readl(s->pmc_voters + M5_PMC_STRIDE * i);
	for (i = 0; i < 4; i++)
		r->rails[i] = readl(s->pmc_rails + 0x2000 + 4 * i);
	r->rails[4] = readl(s->pmc_rails + 0x2028);
	r->rails[5] = readl(s->pmc_rails + 0x2020);
	r->pmc_saved = true;
	/* The PMC's sleep channels, then the memory performance floor's steps */
	t6050_pmc_154(s);
	t6050_floor_steps(s);
	r->park_saved = readl(r->base[T6050_PERF] + 0x18000);
	writel(0, r->base[T6050_PERF] + 0x18000);
	readl_poll_timeout_atomic(r->base[T6050_PERF] + 0x182a0, park,
		!(park & BIT(1)) && FIELD_GET(GENMASK(15, 8), park) ==
		FIELD_GET(GENMASK(23, 16), park), 1, 1000);
	/* The DT's stage sequence: the PMC's rail words for the sleep */
	t6050_seq(s, T6050_SEQ_STAGE);
	r->ptd_saved = r->ptd_valid ? readq(s->ptd + M5_PTD_READ(r->ptd_req)) : 0;
	t6050_wake_arm(s);
	return 0;
}

#define T6050_LATE(r, off, name) { SLEEP_REGION_WORD(r, off), name }

/*
 * Power words macOS has off by its commit that Linux can still have on then, each at macOS's own
 * last write to it. The first nine at the start of .enter, in the order of macOS's runtime
 * power-downs. The rest after ANS, in macOS's order, with the pmVC writes, the PMU's write and the
 * SPMI masks between them (t6050_sleep_after_ans). Lowered only if on, and polled off.
 */
enum {
	T6050_LATE_ENTRY_END = 9,
	T6050_LATE_ATC2_USB = 9, T6050_LATE_ATC1_USB, T6050_LATE_I2C3, T6050_LATE_I2C1,
	T6050_LATE_I2C2, T6050_LATE_I2C6, T6050_LATE_SPI4, T6050_LATE_SPI2,
};
static const struct { u32 word; const char *name; } t6050_late_off[] = {
	T6050_LATE(T6050_PS1, 0x110, "global_aux_hpd_1"),
	T6050_LATE(T6050_PS1, 0x108, "global_aux_hpd_0"),
	T6050_LATE(T6050_PS1, 0x2c0, "i2c0"),
	T6050_LATE(T6050_PS1, 0x2e8, "i2c5"),
	T6050_LATE(T6050_PS1, 0x3a0, "isp_sys"),
	T6050_LATE(T6050_PS1, 0x318, "aes"),
	T6050_LATE(T6050_PS1, 0x100, "ispsens0"),
	T6050_LATE(T6050_PS1, 0x2a8, "fpwm0"),
	T6050_LATE(T6050_PS_AON, 0x0d8, "atc2_common_dp_0"),
	T6050_LATE(T6050_PS_AON, 0x178, "atc2_usb_aon"),
	T6050_LATE(T6050_PS_AON, 0x160, "atc1_usb_aon"),
	T6050_LATE(T6050_PS1, 0x2d8, "i2c3"),
	T6050_LATE(T6050_PS1, 0x2c8, "i2c1"),
	T6050_LATE(T6050_PS1, 0x2d0, "i2c2"),
	T6050_LATE(T6050_PS1, 0x2f0, "i2c6"),
	T6050_LATE(T6050_PS1, 0x340, "spi4"),
	T6050_LATE(T6050_PS1, 0x330, "spi2"),
};

/*
 * No Linux driver touches these. macOS has them at state 4 up to its commit, and writes them
 * only in its wake restore. Capped at 4 if higher at the start of .enter. ATC2_COMMON_DP_0 isn't one:
 * macOS turns it off (above).
 */
#define T6050_STATE4	4
static const struct { u32 word; const char *name; } t6050_state4[] = {
	T6050_LATE(T6050_PS0, 0x138, "pmgr_soc_ocla"),
	T6050_LATE(T6050_PS_MEDIA, 0x120, "pmgr_soc_ocla_g"),
	T6050_LATE(T6050_PS1, 0x2b8, "fpwm2"),
	T6050_LATE(T6050_PS1, 0x2f8, "i2c7"),
	T6050_LATE(T6050_PS_AON, 0x8058, "smc_i2cm6"),
	T6050_LATE(T6050_PS_AON, 0x0b8, "atc1_common_dp_0"),
	T6050_LATE(T6050_PS_AON, 0x0c0, "atc1_common_dp_1"),
	T6050_LATE(T6050_PS_AON, 0x0c8, "atc1_common_dp_2"),
	T6050_LATE(T6050_PS_AON, 0x0d0, "atc1_common_dp_3"),
	T6050_LATE(T6050_PS_AON, 0x0e0, "atc2_common_dp_1"),
	T6050_LATE(T6050_PS_AON, 0x0e8, "atc2_common_dp_2"),
	T6050_LATE(T6050_PS_AON, 0x0f0, "atc2_common_dp_3"),
	T6050_LATE(T6050_PS_AON, 0x0f8, "atc3_common_dp_0"),
	T6050_LATE(T6050_PS_AON, 0x100, "atc3_common_dp_1"),
	T6050_LATE(T6050_PS_AON, 0x108, "atc3_common_dp_2"),
	T6050_LATE(T6050_PS_AON, 0x110, "atc3_common_dp_3"),
};

static bool t6050_late_word_on(struct apple_sleep *s, u32 word)
{
	u32 val = readl(sleep_region_reg(s, word));

	return FIELD_GET(PS_TARGET, val) || FIELD_GET(PS_ACTUAL, val);
}

static void t6050_late_lower(struct apple_sleep *s, unsigned int i)
{
	struct apple_sleep_region_state *r = s->region;
	u32 word = t6050_late_off[i].word;

	if (!t6050_late_word_on(s, word))
		return;
	r->late_saved[i] = readl(sleep_region_reg(s, word));
	sleep_ps_lower(s, word);
	r->late_lowered |= BIT(i);
	r->late_timeouts += !!sleep_ps_wait_off(s, word);
}

/*
 * AUSS, and the Type-C routers' CIO_AUSS words, which have AUSS as their second parent.
 *
 * macOS has AUSS on only while a router uses it. Before a sleep it takes that router's
 * CIO_AUSS word down and then AUSS, and it never sleeps with AUSS on. A sleep entered with
 * AUSS on does not complete: the machine resets five seconds after the commit.
 *
 * Linux has AUSS always on, since not all of its users are described, so genpd leaves it up
 * through the suspend. Here it goes down as on macOS: the children first, AUSS last, each only
 * if on, and polled off. After a sleep the snapshot's replay raises them again, parent first;
 * after a refusal t6050_auss_restore() does.
 */
static const u32 t6050_auss_words[] = {
	SLEEP_REGION_WORD(T6050_PS0, 0x348),	/* ATC0_CIO_AUSS */
	SLEEP_REGION_WORD(T6050_PS0, 0x360),	/* ATC1_CIO_AUSS */
	SLEEP_REGION_WORD(T6050_PS0, 0x408),	/* ATC2_CIO_AUSS */
	SLEEP_REGION_WORD(T6050_PS0, 0x420),	/* ATC3_CIO_AUSS */
	SLEEP_REGION_WORD(T6050_PS0, 0x248),	/* AUSS: the last one */
};

static void t6050_auss_lower(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	BUILD_BUG_ON(ARRAY_SIZE(t6050_auss_words) != ARRAY_SIZE(r->auss_saved));
	BUILD_BUG_ON(ARRAY_SIZE(t6050_late_off) != ARRAY_SIZE(r->late_saved));
	BUILD_BUG_ON(ARRAY_SIZE(t6050_state4) != ARRAY_SIZE(r->state4_saved));
	r->auss_lowered = r->auss_timeouts = 0;
	for (i = 0; i < ARRAY_SIZE(t6050_auss_words); i++) {
		u32 word = t6050_auss_words[i];

		r->auss_saved[i] = readl(sleep_region_reg(s, word));
		if (!t6050_late_word_on(s, word))
			continue;
		sleep_ps_lower(s, word);
		r->auss_lowered |= BIT(i);
		r->auss_timeouts += !!sleep_ps_wait_off(s, word);
	}
}

/* After a refusal: AUSS first, then its children, each as it read before the entry lowered it. */
static void t6050_auss_restore(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	for (i = ARRAY_SIZE(t6050_auss_words); i-- > 0;) {
		if (!(r->auss_lowered & BIT(i)))
			continue;
		r->late_errors += !!sleep_ps_restore(sleep_region_reg(s, t6050_auss_words[i]),
						     r->auss_saved[i]);
	}
	r->auss_lowered = 0;
}

/*
 * VENC0_SYS, the first video encoder's system word. On the J716c it reads 0x0f0003ff (on, no
 * automatic state) at the commit, with no Linux user, while every other media word of the GPU
 * die is off or under automatic power; and the entry's last phase waits for its power gate.
 * It is lowered at the entry, after AUSS, and polled off: one that does not go off refuses the
 * sleep. A refusal puts it back as it read before; after a sleep the snapshot's replay raises
 * it again.
 */
#define T6050_VENC0_SYS	SLEEP_REGION_WORD(T6050_PS_MEDIA, 0x1c0)

static int t6050_venc0_lower(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	int ret;

	r->venc0_lowered = false;
	if (!s->soc->lower_venc0 || !t6050_late_word_on(s, T6050_VENC0_SYS))
		return 0;
	r->venc0_saved = readl(sleep_region_reg(s, T6050_VENC0_SYS));
	sleep_ps_lower(s, T6050_VENC0_SYS);
	r->venc0_lowered = true;
	ret = sleep_ps_wait_off(s, T6050_VENC0_SYS);
	if (ret) {
		dev_warn(s->dev, "entry: VENC0_SYS %#x -> %#x, not off: refusing the sleep\n",
			 r->venc0_saved, readl(sleep_region_reg(s, T6050_VENC0_SYS)));
		return ret;
	}
	dev_info(s->dev, "entry: VENC0_SYS %#x -> %#x\n", r->venc0_saved,
		 readl(sleep_region_reg(s, T6050_VENC0_SYS)));
	return 0;
}

/* After a refusal: VENC0_SYS as it read before the entry lowered it */
static void t6050_venc0_restore(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;

	if (!r->venc0_lowered)
		return;
	if (sleep_ps_restore(sleep_region_reg(s, T6050_VENC0_SYS), r->venc0_saved))
		dev_warn(s->dev, "refusal: VENC0_SYS not back: reads %#x, was %#x\n",
			 readl(sleep_region_reg(s, T6050_VENC0_SYS)), r->venc0_saved);
	r->venc0_lowered = false;
}

/*
 * At the start of .enter, after the snapshot a refusal restores: the first lowerings and the
 * state-4 caps. An error refuses the sleep; the refusal raises what was lowered.
 */
static int t6050_sleep_entry_lower(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	int ret;

	r->late_lowered = r->state4_set = r->late_timeouts = 0;
	for (i = 0; i < T6050_LATE_ENTRY_END; i++)
		t6050_late_lower(s, i);
	t6050_auss_lower(s);
	ret = t6050_venc0_lower(s);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(t6050_state4); i++) {
		void __iomem *reg = sleep_region_reg(s, t6050_state4[i].word);
		u32 val = readl(reg);

		if (FIELD_GET(PS_TARGET, val) <= T6050_STATE4 && !(val & PS_AUTO_ENABLE))
			continue;
		r->state4_saved[i] = val;
		writel((val & ~(PS_AUTO_ENABLE | PS_TARGET)) | T6050_STATE4, reg);
		r->state4_set |= BIT(i);
	}
	dev_info(s->dev, "entry: idle words lowered %#x (%u timed out), capped at 4 %#x\n",
		 r->late_lowered, r->late_timeouts, r->state4_set);
	/* AUSS as the sleep's last steps find it: the engine wants it off. */
	i = ARRAY_SIZE(t6050_auss_words) - 1;
	dev_info(s->dev, "entry: AUSS %#x -> %#x, lowered %#x (%u timed out)\n",
		 r->auss_saved[i], readl(sleep_region_reg(s, t6050_auss_words[i])),
		 r->auss_lowered, r->auss_timeouts);
	return 0;
}

/*
 * Each SPMI controller latches the interrupts its peripherals send, one bit a source, in eight
 * peripheral status words at +0x600 (write 1 to clear); its enables at +0x400 pick the sources
 * that raise its interrupt output. A latched source that is enabled holds the output up, and
 * nub-spmi0's outputs are AP wake sources, which the sleep keeps armed: before nub-spmi0's PMU
 * sources (the DT's) were acknowledged, each sleep after the first alarm woke right after its
 * commit.
 *
 * macOS's handler takes a controller's latched and enabled sources as they come, so none is
 * pending at the sleep's entry. At the wake it runs first, 57 ms in, before the SMC hears that
 * the AP is on: it reads the eight words, acknowledges each enabled source that is set, masks it
 * until its driver has seen it, and names it in the wake's events. In the macOS-HV traces
 * mxt4, dk1 and dk2, those are nub-spmi0's 0x02 (the PMU's), nub-spmi-a1's 0x0b, and in dk1
 * also nub-spmi-a0's 0x3f (dk1's pmset log: "NUB.SPMI0Sw3IRQ nub-spmi-a0.0x3F nub-spmi0.0x02
 * nub-spmi-a1.0x0B rtc"). A latched source that isn't enabled stays set: nub-spmi-a0's +0x600
 * reads 0x12051c00 through every macOS sleep.
 *
 * Linux takes no SPMI interrupt, and acknowledged only nub-spmi0's PMU sources. Do to the
 * controllers what macOS's handler does: at the entry, before the sleep's masks, and at the
 * wake, before the run-time masks are back, acknowledge each latched source that is enabled
 * (and on nub-spmi0 the DT's PMU sources) on nub-spmi0 and each of apple,spmi-buses. The
 * enables are left as they are. What the wake found is kept for its events. An event after the
 * entry's acknowledgement wakes the sleep, as it should.
 */
#define T6050_SPMI_SRC_EN	0x400

static void t6050_spmi_src_ack(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int b, i, pmu_word = (r->spmi_pmu_status - M5_SPMI_SRC_STATUS) / 4;
	void __iomem *bus;
	u32 ack;

	BUILD_BUG_ON(ARRAY_SIZE(r->spmi_src_acked) != 1 + ARRAY_SIZE(r->spmi_bus));
	for (b = 0; b < ARRAY_SIZE(r->spmi_src_acked); b++) {
		bus = b ? r->spmi_bus[b - 1] : s->spmi;
		for (i = 0; i < T6050_SPMI_SRC_WORDS; i++) {
			r->spmi_src_latched[b][i] = 0;
			r->spmi_src_enabled[b][i] = 0;
			if (bus) {
				r->spmi_src_latched[b][i] = readl(bus + M5_SPMI_SRC_STATUS + 4 * i);
				r->spmi_src_enabled[b][i] = readl(bus + T6050_SPMI_SRC_EN + 4 * i);
			}
			ack = r->spmi_src_latched[b][i] & r->spmi_src_enabled[b][i];
			if (!b && r->spmi_pmu_bits && i == pmu_word)
				ack |= r->spmi_src_latched[b][i] & r->spmi_pmu_bits;
			r->spmi_src_acked[b][i] = ack;
			if (ack)
				writel(ack, bus + M5_SPMI_SRC_STATUS + 4 * i);
		}
	}
	r->spmi_src_valid = true;
}

/*
 * Past the point of no return, between ANS's stop and SIO's: ATC2_USB_AON, pmVC, ATC1_USB_AON,
 * pmVC, the watchdog service, the PMU's boot stage and the DT's PMU reads, I2C3/I2C1/I2C2/I2C6,
 * the PMU's latched interrupts acknowledged, the SPMI buses' sleep masks (nub-spmi0 first,
 * then the DT's order and spacing), SPI4, SPI2.
 */
static void t6050_sleep_after_ans(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i, n, failed = 0;
	u8 stage = T6050_PMU_BOOT_STAGE_SLEEP;

	/* A port in use keeps its word, as macOS with both ports in use. */
	r->usb_aon_kept = READ_ONCE(sleep_usb_aon_kept) & (BIT(1) | BIT(2));
	if (!(r->usb_aon_kept & BIT(2)))
		t6050_late_lower(s, T6050_LATE_ATC2_USB);
	r->after_ans_ret[0] = apple_sleep_smc_pmvc(s);
	if (!(r->usb_aon_kept & BIT(1)))
		t6050_late_lower(s, T6050_LATE_ATC1_USB);
	r->after_ans_ret[1] = apple_sleep_smc_pmvc(s);
	writel(0, s->wdt);
	writel(0, s->wdt + 0x10);
	r->boot_stage_written = true; /* Restored at the wake even if the reply is lost. */
	r->after_ans_ret[2] = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_PMU_BOOT_STAGE,
						   &stage, NULL, 1);
	memset(r->pmu_rd, 0, sizeof(r->pmu_rd));
	for (i = 0, n = 0; i < r->npmu_reads; n += r->pmu_reads[i][1], i++) {
		if (n + r->pmu_reads[i][1] > sizeof(r->pmu_rd))
			break;
		if (t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, r->pmu_reads[i][0], NULL,
					 r->pmu_rd + n, r->pmu_reads[i][1]))
			failed |= BIT(i);
	}
	r->after_ans_ret[3] = failed;
	for (i = T6050_LATE_I2C3; i <= T6050_LATE_I2C6; i++)
		t6050_late_lower(s, i);
	r->spmi_mask[0] = readl(s->spmi + M5_SPMI_IRQ_EN);
	/* Every controller's pending sources, as macOS's handler leaves them for the sleep */
	t6050_spmi_src_ack(s);
	r->spmi_src_valid = false;
	/* nub-spmi0's mask first: its interrupt outputs are AP wake sources. */
	writel(readl(s->spmi + M5_SPMI_IRQ_STATUS), s->spmi + M5_SPMI_IRQ_STATUS);
	writel(s->soc->spmi_notify_sleep, s->spmi + M5_SPMI_IRQ_EN);
	for (i = 0; i < r->nspmi_order; i++) {
		void __iomem *bus = r->spmi_bus[r->spmi_order[i]];

		if (!bus)
			continue;
		udelay(r->spmi_gap_us);
		/* What polled transactions latched since boot, acknowledged */
		writel(readl(bus + M5_SPMI_IRQ_STATUS), bus + M5_SPMI_IRQ_STATUS);
		r->spmi_mask[1 + r->spmi_order[i]] = readl(bus + M5_SPMI_IRQ_EN);
		writel(s->soc->spmi_notify_sleep, bus + M5_SPMI_IRQ_EN);
	}
	r->spmi_masked = true;
	for (i = T6050_LATE_SPI4; i <= T6050_LATE_SPI2; i++)
		t6050_late_lower(s, i);
}

/*
 * Non-display DART stops: mask and clear the DART's error interrupts, disable each enabled
 * stream with its own write, flush the TLB, then write the DART's GAPF slice (from its node)
 * with the off value. SEP's DART gets the interrupt words and the slice only, right before the
 * SEP request.
 */
#define T6050_DART_ERROR_MASK	0x4060
#define T6050_DART_EXCEPTION	0x4000
#define T6050_DART_ERROR_STATUS	0x4204
#define T6050_DART_PROTECT	0x200
#define T6050_DART_ENABLE	0xc00
#define T6050_DART_DISABLE	0xc20
#define T6050_DART_TLB_CMD	0x80
#define T6050_DART_TLB_FLUSH	0xa000
#define T6050_DART_TLB_BUSY	BIT(31)

static const struct {
	bool streams;
	u32 ps;	/* Its domain's word; touched only while that reads on. 0: always on. */
	bool all16;	/* Streams 1-15 each written whether enabled or not, then any enabled above */
} t6050_darts[] = {
	[T6050_DART_SEP] = { false, 0 },
	[T6050_DART_PMP0] = { true, 0 },
	[T6050_DART_SIO] = { true, SLEEP_REGION_WORD(T6050_PS1, 0x290) },
	[T6050_DART_APCIE0] = { true, SLEEP_REGION_WORD(T6050_PS1, 0x118), true },
};

static void t6050_sleep_dart_stop(struct apple_sleep *s, unsigned int i)
{
	struct apple_sleep_region_state *r = s->region;
	void __iomem *dart = r->dart[i];
	u32 enabled, val;
	unsigned int sid;

	if (!dart || (t6050_darts[i].ps && !sleep_ps_on(s, t6050_darts[i].ps)))
		return;
	writel(U32_MAX, dart + T6050_DART_ERROR_MASK);
	writel(0, dart + T6050_DART_EXCEPTION);
	writel(U32_MAX, dart + T6050_DART_ERROR_STATUS);
	if (t6050_darts[i].streams) {
		enabled = readl(dart + T6050_DART_ENABLE);
		for (sid = 0; sid < 32; sid++)
			if ((t6050_darts[i].all16 && sid && sid < 16) || (enabled & BIT(sid))) {
				writel(BIT(sid), dart + T6050_DART_DISABLE);
				if (t6050_darts[i].all16)
					readl(dart + T6050_DART_DISABLE);
			}
		writel(T6050_DART_TLB_FLUSH, dart + T6050_DART_TLB_CMD);
		r->dart_timeouts += !!readl_poll_timeout_atomic(dart + T6050_DART_TLB_CMD, val,
								!(val & T6050_DART_TLB_BUSY), 1, 1000);
	}
	if (r->gapf && r->dart_slice[i]) {
		r->dart_slice_saved[i] = readl(r->gapf + r->dart_slice[i]);
		writel(r->dart_slice_off, r->gapf + r->dart_slice[i]);
		r->dart_slice_written |= BIT(i);
	}
	r->dart_done |= BIT(i);
}

/*
 * The radio's link shutdown before the sleep: the DT's apple,radio-link-down (the link's own
 * shutdown, through PME_Turn_Off and its ack), PERST# low, the PHY's reference clock gate off,
 * the port's DART stopped and its GAPF slice, then apple,radio-link-down-port. The port's SYS
 * words idle off under automatic power: they are held on for the sequence and given back
 * after it. Only with the link up. With a host driver bound it runs from the host's noirq
 * suspend instead.
 * n1_state: 0 done, 1 no registers or no sequence, 2 a domain off, 3 link down, 4 no PME_TO
 * ack, 5 the host's.
 */
#define T6050_PORT_LINKSTS		0x208
#define   T6050_PORT_LINKSTS_UP		BIT(0)
#define T6050_PHY_LANE_CFG_REFCLKCG	BIT(31)

static void t6050_sleep_n1_down(struct apple_sleep *s, bool perst_alone)
{
	static const u32 ps[] = { 0x118, 0x128, 0x148, 0x150 };	/* APCIE_GP, _ST0, _SYS_GP, _SYS_ST0 */
	struct apple_sleep_region_state *r = s->region;
	void __iomem *port = r->n1_port, *phy = r->n1_phy;
	void __iomem *perst = r->ap_gpio && r->n1_perst ? r->ap_gpio + 4 * r->n1_perst : NULL;
	u32 v, held[ARRAY_SIZE(ps)] = {};
	unsigned int i;

	r->n1_held = 0;
	r->n1_state = 1;
	if (!perst)
		return;
	if (!r->n1_ecam || !port || !phy || !r->dart[T6050_DART_APCIE0] || !r->gapf ||
	    !r->link_down.n)
		goto perst_only;
	r->n1_state = 2;
	for (i = 0; i < ARRAY_SIZE(ps); i++) {
		void __iomem *reg = sleep_power_reg(s, SLEEP_REGION_WORD(T6050_PS1, ps[i]));

		v = readl(reg);
		if (FIELD_GET(PS_ACTUAL, v))
			continue;
		if (!(v & PS_AUTO_ENABLE) || !FIELD_GET(PS_TARGET, v))
			goto perst_only;
		held[i] = v;
		writel((v & ~PS_AUTO_ENABLE) | PS_TARGET, reg);
		if (readl_poll_timeout_atomic(reg, v, FIELD_GET(PS_ACTUAL, v) == 0xf, 10, 5000))
			goto perst_only;
	}
	r->n1_state = 3;
	if (!(readl(port + T6050_PORT_LINKSTS) & T6050_PORT_LINKSTS_UP))
		goto perst_only;
	r->n1_state = t6050_ops_run(s, &r->link_down) ? 4 : 0;
	t6050_pin_low(perst);
	writel(readl(phy) & ~T6050_PHY_LANE_CFG_REFCLKCG, phy);
	r->n1_gapf = r->dart_slice[T6050_DART_APCIE0] ?
		     readl(r->gapf + r->dart_slice[T6050_DART_APCIE0]) : 0;
	t6050_sleep_dart_stop(s, T6050_DART_APCIE0);
	readl(r->dart[T6050_DART_APCIE0] + T6050_DART_EXCEPTION);
	t6050_ops_run(s, &r->link_down_port);
	goto unhold;

perst_only:
	if (perst_alone)
		t6050_pin_low(perst);
unhold:
	for (i = 0; i < ARRAY_SIZE(ps); i++)
		if (held[i]) {
			writel(held[i], sleep_power_reg(s, SLEEP_REGION_WORD(T6050_PS1, ps[i])));
			r->n1_held |= BIT(i);
		}
}

static bool t6050_n1_host_bound(struct apple_sleep *s)
{
	struct platform_device *pdev = s->region->radio_host;

	return pdev && device_is_bound(&pdev->dev);
}

/*
 * From the radio host driver's noirq suspend, once its functions are quiet and its DART is saved
 * and gated: the same link shutdown, before every step of the commit. An error means nothing
 * was written.
 */
int apple_sleep_radio_port_down(void)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_region_state *r = s ? s->region : NULL;

	if (!r || !r->n1_host || !r->ap_gpio)
		return -ENODEV;
	t6050_sleep_n1_down(s, false);
	if (r->n1_state && r->n1_state != 4)
		return -EIO;
	/* The wake puts PERST# back as the shutdown left it, low, as macOS restores it */
	if (s->pins_saved_valid && s->npins > r->n1_perst && !test_bit(r->n1_perst, s->pins_skip))
		s->pins_saved[r->n1_perst] = readl(r->ap_gpio + 4 * r->n1_perst);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_port_down);

/*
 * For the radio host's noirq resume: 0 once the root complex runs again (or never stopped),
 * else nothing behind it may be read. A sleep abandoned after the shutdown lost no power: the
 * DART's slice goes back, and the host resets the DART itself.
 */
int apple_sleep_radio_rc_ready(void)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_region_state *r = s ? s->region : NULL;

	if (!r || !r->n1_host)
		return 0;
	if (!r->committed && r->gapf && r->dart_slice[T6050_DART_APCIE0] &&
	    (!r->n1_state || r->n1_state == 4))
		writel(r->n1_gapf, r->gapf + r->dart_slice[T6050_DART_APCIE0]);
	return r->n1_up_ret;
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_rc_ready);

/*
 * The shared PCIe PHY and the root complex after a real sleep, as the boot loader brings them
 * up: each block's power word on, held if automatic, before any read; the root complex's +0x4
 * cleared; the DT's apple,radio-phy-wake (the PHY's tunables and its start); then the root
 * complex's pre-run word, its run bit and ready.
 */
#define T6050_RC_CTL			0x50
#define   T6050_RC_CTL_RUN		BIT(0)
#define T6050_RC_PRE_RUN		0x54
#define T6050_RC_STAT			0x58
#define   T6050_RC_STAT_READY		BIT(0)

static int t6050_n1_unpark(struct apple_sleep *s)
{
	/* APCIE_GP, APCIE_SYS_GP, APCIE_PHY_SW, APHY_GP_AUS5_A, APHY_GP_AUS5 */
	static const u32 ps[] = { 0x118, 0x148, 0x1b0, 0x258, 0x280 };
	struct apple_sleep_region_state *r = s->region;
	void __iomem *rc = r->n1_rc;
	u32 v;
	unsigned int i;
	bool running;

	if (!rc || !r->phy_wake.n)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(ps); i++) {
		void __iomem *reg = sleep_power_reg(s, SLEEP_REGION_WORD(T6050_PS1, ps[i]));

		v = readl(reg);
		if (FIELD_GET(PS_ACTUAL, v) == 0xf)
			continue;
		writel((v & ~PS_AUTO_ENABLE) | PS_TARGET, reg);
		if (readl_poll_timeout_atomic(reg, v, FIELD_GET(PS_ACTUAL, v) == 0xf, 10, 5000))
			return -EIO;
	}
	/* Still running after a commit that lost no power: only the park is undone then */
	running = readl(rc + T6050_RC_CTL) & T6050_RC_CTL_RUN;
	if (!running)
		writel(0, rc + 0x4);
	if (t6050_ops_run(s, &r->phy_wake) && !running)
		return -ETIMEDOUT;
	if (!running) {
		writel(0x140, rc + T6050_RC_PRE_RUN);
		writel(readl(rc + T6050_RC_CTL) | T6050_RC_CTL_RUN, rc + T6050_RC_CTL);
	}
	return readl_poll_timeout_atomic(rc + T6050_RC_STAT, v, v & T6050_RC_STAT_READY, 10,
					 250000);
}

/* The radio host's port wake steps from the DT: 0 after the port's set-up, 1 after its DART's. */
void apple_sleep_radio_port_wake(unsigned int step)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_region_state *r = s ? s->region : NULL;

	if (r && r->n1_host)
		t6050_ops_run(s, step ? &r->dart_wake : &r->port_wake);
}
EXPORT_SYMBOL_GPL(apple_sleep_radio_port_wake);

/* Right before SEP's sleep request: SEP's DART, interrupts and slice only. */
/*
 * After GFX/GFX1's commit stop, before SEP: the cluster block's +0x10 request released and
 * polled down. With the GPU's firmware still up the release doesn't complete. Past the point of
 * no return: reported, never refused.
 */
static void t6050_sleep_mcc_release_10(struct apple_sleep *s)
{
	void __iomem *reg = s->cluster + 0x10;
	u64 start = __arch_counter_get_cntpct();
	u32 before = readl(reg), value;
	int ret;

	/* Released at the stage: only polled here, and written only if it isn't pending. */
	if (before && before != 0x10)
		writel(0, reg);
	ret = readl_poll_timeout_atomic(reg, value, !value, 1, 5000);
	dev_dbg(s->dev, "MCC +0x10 after the GPU stop: %x -> %x%s in %llu us\n", before, value,
		 ret ? " (NOT down)" : "",
		 (__arch_counter_get_cntpct() - start) * USEC_PER_SEC / arch_timer_get_cntfrq());
}

static void t6050_sleep_before_enclave(struct apple_sleep *s)
{
	t6050_sleep_mcc_release_10(s);
	s->region->dart_done = s->region->dart_timeouts = 0;
	t6050_sleep_dart_stop(s, T6050_DART_SEP);
}

/*
 * The early secondary-cluster step counts each present core entering its
 * last WFI. A core left off by a failed restart cannot acknowledge it.
 * Parked cores, including intentionally offline ones, still take part.
 */
static int t6050_check_early_cpus(struct apple_sleep *s)
{
	unsigned int cpu, cluster;

	if (!s->soc->clusters_down_first)
		return 0;
	for_each_present_cpu(cpu) {
		cluster = sleep_cpu_cluster(s, cpu_logical_map(cpu));
		if ((cluster == 1 || cluster == 2) && spin_table_cpu_tracked_off(cpu)) {
			dev_warn(s->dev, "refused: early cluster CPU%u still tracked off\n", cpu);
			return -EBUSY;
		}
	}
	return 0;
}

/* Final returning checks: no last_stop/enclave action has happened yet. */
static int t6050_sleep_final_check(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u8 ticks[8] = {};
	u64 now, delay, counter;
	u32 compare;
	int ret;

	ret = t6050_check_early_cpus(s);
	if (ret)
		return ret;

	if (!r->ptd_owned || !s->smc.held || !s->smc.saved_ops ||
	    !s->smc.saved_ops->check || !s->smc.saved_ops->commit_notice) {
		dev_info(s->dev, "final check: PTD owned %d, SMC held %d\n", r->ptd_owned, s->smc.held);
		return -EBUSY;
	}
	ret = s->smc.saved_ops->check(s->smc.data);
	if (ret) {
		dev_info(s->dev, "final check: SMC check %d\n", ret);
		return ret;
	}
	r->complex_observed = readl(s->cluster + 0xc);
	ret = t6050_sleep_plan_complex_request(r->complex_observed, &r->complex_request);
	if (ret)
		return ret;
	/* Returning RTC admission precedes SEP. The real AP notice follows peer
	 * stops; schedule is anchored again to its actual request/ACK afterward.
	 */
	now = __arch_counter_get_cntpct();
	s->timer_at = now + (u64)arch_timer_get_cntfrq() *
		max(s->region->runway_us, s->soc->smc_policy->idle_to_timer_us) / USEC_PER_SEC;
	ret = t6050_sleep_check_alarm(s);
	if (ret)
		return ret;
	/* No alarm armed: nothing to hold against the sleep timer */
	if (!r->alarm_valid)
		return 0;
	ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_COUNTER, NULL, ticks, 6);
	if (ret)
		return ret;
	counter = get_unaligned_le64(ticks) >> 16; /* In the alarm compare's units */
	compare = get_unaligned_le32(r->alarm_compare);
	delay = div_u64((s->timer_at - now) * USEC_PER_SEC, arch_timer_get_cntfrq());
	if (compare <= counter + DIV_ROUND_UP_ULL(delay, USEC_PER_SEC) + 2)
		return -ETIME;
	/* The PMU's boot stage is written after ANS: t6050_sleep_after_ans. */
	return 0;
}

static void t6050_sleep_prepare_regs(struct apple_sleep *s)
{
	/* The watchdog counts cleared before the irreversible complex sequence. */
	writel(0, s->wdt + WDT_WD0_COUNT);
	writel(0, s->wdt + WDT_WD1_COUNT);
}

static void t6050_sleep_schedule(struct apple_sleep *s)
{
	u64 now = __arch_counter_get_cntpct();

	/* Last-stop/enclave latency cannot consume the runway: every commit step fits in it. */
	s->timer_at = max(now + arch_timer_get_cntfrq() * (u64)s->region->runway_us / USEC_PER_SEC,
		s->smc_idle_sent + (u64)arch_timer_get_cntfrq() *
		s->soc->smc_policy->idle_to_timer_us / USEC_PER_SEC);
}

/* This callback may return an error only to the common fatal path.
 * No newly pending wake/alarm can become a returning post-SEP veto here.
 */
static int t6050_sleep_commit_notice(struct apple_sleep *s)
{
	int ret;

	/* PMP0's DART right after the PMP's run bit, the last stop of the second pass. */
	t6050_sleep_dart_stop(s, T6050_DART_PMP0);
	ret = apple_sleep_smc_commit_notice(s);

	if (ret)
		return ret;
	if (!s->smc_idle_sent || s->smc_idle_ack < s->smc_idle_sent)
		return -EPROTO;
	/* SIO's DART after the SMC's AP IDLE ack, the last write before the window. */
	t6050_sleep_dart_stop(s, T6050_DART_SIO);
	t6050_sleep_schedule(s);
	return 0;
}

/* The PTD request's bit for the DT's entry @which (0-2 the clusters, 3 storage) dropped */
static void t6050_sleep_ptd_drop(struct apple_sleep *s, unsigned int which)
{
	struct apple_sleep_region_state *r = s->region;
	u64 request;

	if (!r->ptd_valid)
		return;
	request = readq(s->ptd + M5_PTD_READ(r->ptd_req)) & ~BIT_ULL(r->ptd_bit[which]);
	writeq(request, s->ptd + M5_PTD_WRITE(r->ptd_req));
}


static void t6050_sleep_wait_before(struct apple_sleep *s, u32 before_us)
{
	u64 target = s->timer_at - (u64)arch_timer_get_cntfrq() * before_us / USEC_PER_SEC;

	while (__arch_counter_get_cntpct() < target)
		cpu_relax();
}

/* Waits until the DT schedule's time for commit step @step */
static void t6050_step(struct apple_sleep *s, unsigned int step)
{
	t6050_sleep_wait_before(s, s->region->sched[step]);
}

static int t6050_cpm_prop(struct apple_sleep *s, const char *name, u64 **ops, unsigned int *n)
{
	int len = of_property_count_u64_elems(s->dev->of_node, name);

	*n = 0;
	if (len <= 0)
		return 0;
	if (len % 3)
		return -EINVAL;
	*ops = devm_kcalloc(s->dev, len, sizeof(u64), GFP_KERNEL);
	if (!*ops)
		return -ENOMEM;
	if (of_property_read_u64_array(s->dev->of_node, name, *ops, len))
		return -EINVAL;
	*n = len / 3;
	return 0;
}


#define SYS_APL_S3_7_C15_C5_0	sys_reg(3, 7, 15, 5, 0)
#define SYS_APL_S3_4_C15_C10_4	sys_reg(3, 4, 15, 10, 4)

/*
 * A stopped core after the commit, as macOS's parked cores: its interrupts opted out, the
 * deep-sleep bits set, then WFI, powered until the sleep engine takes the cluster down.
 */
static void __noreturn t6050_cpu_down(struct apple_sleep *s, u64 mpidr)
{
	unsigned int c = sleep_cpu_cluster(s, mpidr);

	if (c < ARRAY_SIZE(s->region->cpus_down))
		atomic_inc(&s->region->cpus_down[c]);
	write_sysreg(ARCH_TIMER_CTRL_IT_MASK, cntp_ctl_el0);
	write_sysreg(ARCH_TIMER_CTRL_IT_MASK, cntv_ctl_el0);
	isb();
	write_sysreg_s(read_sysreg_s(SYS_APL_S3_4_C15_C10_4) & ~3ULL, SYS_APL_S3_4_C15_C10_4);
	isb();
	write_sysreg_s(1, SYS_APL_S3_7_C15_C5_0);
	isb();
	write_sysreg_s(read_sysreg_s(SYS_APL_DEEP_SLEEP) | DEEP_SLEEP_ENABLE, SYS_APL_DEEP_SLEEP);
	isb();
	for (;;) {
		write_sysreg_s(IPI_SR_PENDING, SYS_APL_IPI_SR_EL1);
		dsb(sy);
		isb();
		wfi();
	}
}

static void t6050_mcc_release(struct apple_sleep *s, unsigned int i)
{
	struct apple_sleep_region_state *r = s->region;
	void __iomem *reg = s->cluster + (i ? 8 : 4);
	void __iomem *ack = s->cluster + (i ? 0x1c : 0x18);
	u32 *c = r->mcc_cycle[i], v;

	t6050_sleep_ptd_drop(s, i ? 2 : 1);
	writel(0, reg);
	readl_poll_timeout_atomic(reg, v, !v || v == 0x10, 1, 200);
	c[8] = v;
	c[9] = readl(ack);
}

/*
 * The MCC channel stop, after the cluster words, each access at its width:
 * - mcc reg[22..25]: for each set bit of +0x584, lowest first, the 64-bit counter at +0x4000 + 8n
 *   (n = the step), then +0x584 read and written with that bit cleared, down to 0. reg[26]'s +0x584
 *   isn't touched.
 * - Then lanes 0-1 on every channel, then lanes 2-3: the 64-bit counter at +0x4050 + 8n, +0x58c
 *   (lanes 0-1) or +0x590 (2-3) rewritten, +0x588 strobe bit 16 + 4n set, then bit 4n cleared. reg[26]
 *   has lanes 0 and 2 only.
 * The counters fault on 32-bit reads: only 64-bit ones.
 */
static const u8 t6050_mcc_lanes[5] = { 0xf, 0xf, 0xf, 0xf, 0x5 };

static void t6050_mcc_stop(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int ch, pass, n, k;
	u32 v;

	for (ch = 0; ch < 4; ch++) {
		void __iomem *m = r->mcc[ch];

		if (!m)
			continue;
		for (k = 0; k < 32; k++) {
			readq(m + 0x4000 + 8 * k);
			v = readl(m + 0x584);
			if (!v)
				break;
			v &= v - 1;
			writel(v, m + 0x584);
			if (!v)
				break;
		}
	}
	for (pass = 0; pass < 2; pass++)
		for (ch = 0; ch < ARRAY_SIZE(t6050_mcc_lanes); ch++) {
			void __iomem *m = r->mcc[ch];
			u32 mask = pass ? 0x590 : 0x58c;

			if (!m)
				continue;
			for (n = 2 * pass; n < 2 * pass + 2; n++) {
				if (!(t6050_mcc_lanes[ch] & BIT(n)))
					continue;
				readq(m + 0x4050 + 8 * n);
				writel(readl(m + mask), m + mask);
				writel(readl(m + 0x588) | BIT(16 + 4 * n), m + 0x588);
				writel(readl(m + 0x588) & ~BIT(4 * n), m + 0x588);
			}
		}
}

/* Timekeeping is stopped at the commit: waits there count the CPU's own counter. */
static u64 t6050_cnt_ticks(u32 us)
{
	return div_u64((u64)arch_timer_get_cntfrq() * us, USEC_PER_SEC);
}

static u32 t6050_cnt_us(u64 ticks)
{
	return div_u64(ticks * USEC_PER_SEC, arch_timer_get_cntfrq());
}

/* Polls @reg until it reads @want, for up to @limit_us: 0, or -ETIMEDOUT */
static int t6050_cnt_poll(void __iomem *reg, u32 want, u32 limit_us, u32 *last, u32 *us)
{
	u64 start = __arch_counter_get_cntpct(), end = start + t6050_cnt_ticks(limit_us), now;
	u32 v;

	do {
		v = readl(reg);
		now = __arch_counter_get_cntpct();
		if (v == want)
			break;
		cpu_relax();
	} while (now < end);
	*last = v;
	*us = t6050_cnt_us(now - start);
	return v == want ? 0 : -ETIMEDOUT;
}

#define T6050_EARLY_CPUS_US	2000
#define T6050_EARLY_SETTLE_US	50
#define T6050_EARLY_SEQ_US	5000
#define T6050_EARLY_SEQ1_DONE	0x1a
#define T6050_EARLY_SEQ2_DONE	0x1c01a

/*
 * The clusters other than the boot cluster (MPIDR affinity 1 of 1 and 2) taken down at the
 * start of the commit, before anything else in it.
 *
 * The clusters' power-down programs run one at a time, on one engine. Dropped together in the
 * commit window, as the cluster words step below does, the P cluster's program queues behind
 * the boot cluster's, and that one cannot finish while the boot core still runs the commit:
 * the P cluster never goes down and the sleep does not complete. Taken down in this order, the
 * secondary clusters first and the boot cluster last, all three programs complete.
 *
 * Their cores are released to their last WFI (spin_table_release_cpus()) and counted there,
 * then each cluster's PTD bit and request drop (t6050_mcc_release()), then each cluster's
 * sequencer is polled until its program is done. Nothing in the commit may touch those
 * clusters' own blocks (0x211xxxxxx, 0x212xxxxxx) after this. The results are kept for the
 * report after the wake: nothing is printed this late.
 *
 * This runs past the point of no return, so nothing here refuses the sleep. Every wait is
 * bounded on the CPU's counter (2 ms for the cores, 5 ms for each sequencer), well inside the
 * commit's runway. If the cores are not all counted, the cluster requests are left up and the
 * commit drops them later, as it did before this step. Any failure here ends in the engine's
 * watchdog reset 5 s after the commit, the same as without this step, so there is nothing to
 * undo.
 */
static void t6050_clusters_down_first(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	static struct cpumask cpus;
	unsigned int cpu, c, want[3] = {};
	u64 end;

	memset(r->early_seq, 0, sizeof(r->early_seq));
	memset(r->early_us, 0, sizeof(r->early_us));
	r->early_done = true;
	cpumask_clear(&cpus);
	for_each_present_cpu(cpu) {
		c = sleep_cpu_cluster(s, cpu_logical_map(cpu));
		if (c == 1 || c == 2) {
			cpumask_set_cpu(cpu, &cpus);
			want[c]++;
		}
	}
	atomic_set(&r->cpus_down[1], 0);
	atomic_set(&r->cpus_down[2], 0);
	spin_table_release_cpus(&cpus);
	end = __arch_counter_get_cntpct() + t6050_cnt_ticks(T6050_EARLY_CPUS_US);
	r->early_ret[0] = -ETIMEDOUT;
	do {
		if (atomic_read(&r->cpus_down[1]) == want[1] &&
		    atomic_read(&r->cpus_down[2]) == want[2]) {
			r->early_ret[0] = 0;
			break;
		}
		cpu_relax();
	} while (__arch_counter_get_cntpct() < end);
	if (r->early_ret[0]) {
		/* The requests stay up: the commit's own step drops them, as before */
		r->early_ret[1] = -ECANCELED;
		r->early_ret[2] = -ECANCELED;
		return;
	}
	/* Counted on the way in: let the last ones reach the WFI itself */
	end = __arch_counter_get_cntpct() + t6050_cnt_ticks(T6050_EARLY_SETTLE_US);
	while (__arch_counter_get_cntpct() < end)
		cpu_relax();
	t6050_mcc_release(s, 0);
	t6050_mcc_release(s, 1);
	r->early_dropped = true;
	r->early_ret[1] = t6050_cnt_poll(s->cluster + 0x18, T6050_EARLY_SEQ1_DONE,
					 T6050_EARLY_SEQ_US, &r->early_seq[0], &r->early_us[0]);
	r->early_ret[2] = t6050_cnt_poll(s->cluster + 0x1c, T6050_EARLY_SEQ2_DONE,
					 T6050_EARLY_SEQ_US, &r->early_seq[1], &r->early_us[1]);
}

/* After the wake: how the early cluster step went, in one line */
#define T6050_EARLY_REPORT "secondary clusters down first: cores %d+%d (%d), cluster 1 %#x in %u us (%d), cluster 2 %#x in %u us (%d)\n"

static void t6050_clusters_down_first_report(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	int c1 = atomic_read(&r->cpus_down[1]), c2 = atomic_read(&r->cpus_down[2]);

	if (!r->early_done)
		return;
	r->early_done = false;
	if (r->early_ret[0] || r->early_ret[1] || r->early_ret[2])
		dev_warn(s->dev, T6050_EARLY_REPORT, c1, c2, r->early_ret[0],
			 r->early_seq[0], r->early_us[0], r->early_ret[1],
			 r->early_seq[1], r->early_us[1], r->early_ret[2]);
	else
		dev_info(s->dev, T6050_EARLY_REPORT, c1, c2, r->early_ret[0],
			 r->early_seq[0], r->early_us[0], r->early_ret[1],
			 r->early_seq[1], r->early_us[1], r->early_ret[2]);
}

/*
 * The commit window, each step at the DT schedule's time before the sleep timer: the MCC
 * requests released, the storage link's PHY parked and its words down, the watchdogs, the
 * memory floor, the MCC channels stopped, the clock words, PTD's drops and the power gates, the
 * perf words, AFR, the cluster words and the complex request, SIO, then the tables.
 */
static void t6050_sleep_commit_regs(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	const struct apple_sleep_region_profile *p = s->soc->regions;
	unsigned int i, j;
	bool media78 = false;
	u32 value;

	r->committed = true;
	/* Past every refusal: the secondary clusters go first, where the SoC needs that */
	r->early_dropped = false;
	if (s->soc->clusters_down_first)
		t6050_clusters_down_first(s);
	/*
	 * Only +0x4 and +0x8 are released, each after pwrgate's three words: no raise or re-arm.
	 * Already released by t6050_clusters_down_first() when it dropped them.
	 */
	for (i = 0; i < 2; i++) {
		t6050_step(s, T6050_STEP_MCC0 + i);
		readl(s->pwrgate + 0x120);
		writel(PWRGATE_ON, s->pwrgate + 0x120);
		readl(s->pwrgate + 0x128);
		writel(PWRGATE_ON, s->pwrgate + 0x128);
		readl(s->pwrgate + 0x130);
		writel(PWRGATE_ON, s->pwrgate + 0x130);
		if (!r->early_dropped)
			t6050_mcc_release(s, i);
	}
	/* Only after ANS's registered driver stop. Never stop ANS here. */
	t6050_step(s, T6050_STEP_PHY_PARK);
	if (s->pcie_ans_stopped)
		t6050_storage_phy_park(s);
	t6050_step(s, T6050_STEP_STORAGE);
	if (s->pcie_ans_stopped) {
		for (i = 0; i < ARRAY_SIZE(t6050_ans_storage_ps); i++) {
			u32 word = SLEEP_REGION_WORD(T6050_PS1, t6050_ans_storage_ps[i]);

			if (t6050_ans_storage_ps[i] == T6050_PS1_APCIE_PHY_SW)
				t6050_storage_phy_down(s);
			sleep_ps_lower(s, word);
			/* Diagnostic only: a final-stop error cannot return to callers. */
			if (sleep_ps_wait_off(s, word))
				s->ans_off_ret = -ETIMEDOUT;
		}
	}
	t6050_step(s, T6050_STEP_WDT);
	t6050_seq(s, T6050_SEQ_WDT);
	writel(0, s->wdt + WDT_WD0_CTL);
	apple_sleep_set_wd2(s, T6050_WDT_COMMIT_SECS);
	writel(0, s->wdt_aux + 8);
	/* The memory floor's voter to 0, not read */
	t6050_step(s, T6050_STEP_FLOOR);
	s->region->floor_changed |= BIT(1);
	writel(0, s->pmc_voters + M5_PMC_STRIDE * 1);
	t6050_step(s, T6050_STEP_MCC_STOP);
	t6050_mcc_stop(s);
	/* clkgen +0x78000 to 0, right before the clock words */
	t6050_step(s, T6050_STEP_CLKGEN);
	if (r->clkgen78) {
		readl(r->clkgen78);
		writel(0, r->clkgen78);
	}
	t6050_step(s, T6050_STEP_CLK);
	for (i = 0; i < r->entry_clk.count; i++) {
		void __iomem *reg = sleep_region_reg(s, r->entry_clk.offset[i]);

		/* and the media die's, just before its own clock words */
		if (!media78 && SLEEP_REGION(r->entry_clk.offset[i]) == T6050_MEDIA_CLK) {
			media78 = true;
			if (r->media_clkgen78) {
				readl(r->media_clkgen78);
				writel(0, r->media_clkgen78);
			}
		}
		writel(readl(reg) & ~BIT(0), reg);
	}
	/* PTD's three drops of the boot cluster's bit, then the three pwrgate rounds */
	for (i = 0; i < 3; i++) {
		t6050_step(s, T6050_STEP_PTD0 + i);
		t6050_sleep_ptd_drop(s, 0);
	}
	for (i = 0; i < 3; i++) {
		t6050_step(s, T6050_STEP_PWRGATE0 + i);
		/*
		 * CPMn's counters read, then pwrgate's three R/W. Clusters 1 and 2's CPM blocks
		 * are gone once t6050_clusters_down_first() took them down.
		 */
		if (r->cpm[i] && !(i && r->early_dropped)) {
			readq(r->cpm[i] + 0x8000);
			readq(r->cpm[i] + 0x8008);
		}
		readl(s->pwrgate + 0x120);
		writel(PWRGATE_ON, s->pwrgate + 0x120);
		readl(s->pwrgate + 0x128);
		writel(PWRGATE_ON, s->pwrgate + 0x128);
		readl(s->pwrgate + 0x130);
		writel(PWRGATE_ON, s->pwrgate + 0x130);
	}
	t6050_step(s, T6050_STEP_PERF);
	for (i = 0; i < r->entry_perf.count; i++)
		writel(0, sleep_region_reg(s, r->entry_perf.offset[i]));
	/* The AP's voter to 0, between the perf words and AFR */
	s->region->floor_changed |= BIT(3);
	readl(s->pmc_voters + M5_PMC_STRIDE * 3);
	writel(0, s->pmc_voters + M5_PMC_STRIDE * 3);
	readl(s->pmc_voters + M5_PMC_STRIDE * 3 + 4);
	/* AFR, if on: the DT's sequence */
	t6050_step(s, T6050_STEP_AFR);
	value = readl(r->base[T6050_PS_GPX] + 0x100);
	if (FIELD_GET(PS_TARGET, value))
		t6050_seq(s, T6050_SEQ_AFR);
	/* The cluster words: the boot cluster's released, the others requested down */
	t6050_step(s, T6050_STEP_CLUSTER);
	for (i = 0; i < 3; i++) {
		readl(s->cluster + 4 * i);
		writel(i ? 0x10 : 0, s->cluster + 4 * i);
	}
	/* The complex request the plan checked before anything irreversible */
	writel(r->complex_request, s->cluster + 0xc);
	r->late_errors += !!readl_poll_timeout_atomic(s->cluster + 0xc, value, !value, 1, 1000);
	writel(0, r->base[T6050_PERF] + 0x18000);
	readl(s->cluster + 0x10);
	writel(0, s->cluster + 0x10);
	/* SIO and SIO_CPU, then the clock muxes parked: the DT's sequence */
	t6050_step(s, T6050_STEP_SIO);
	t6050_seq(s, T6050_SEQ_SIO);
	t6050_step(s, T6050_STEP_TABLES);
	for (i = 0; i < p->ntables; i++)
		for (j = 0; j < p->table[i].count; j++) {
			void __iomem *reg = r->base[p->table[i].region] + j * p->table[i].stride;
			readl(reg);
			writel(0, reg);
		}
	t6050_sleep_wait_before(s, 0);
}

static void t6050_sleep_pre_commit(struct apple_sleep *s)
{
	t6050_sleep_schedule(s);
}

static void t6050_sleep_restore_display(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	int d, j;
	u32 saved, keep, now;
	void __iomem *cpu;

	for (d = s->soc->ndisp - 1; d >= 0; d--) {
		if (!(r->display_lowered & BIT(d)))
			continue;
		for (j = 2; j >= 1; j--)
			r->late_errors += !!sleep_ps_restore(sleep_region_reg(s,
				s->soc->ps.disp[d][j]), r->display_saved[d][j]);
		cpu = sleep_region_reg(s, s->soc->ps.disp[d][0]);
		saved = r->display_saved[d][0];
		keep = (saved & ~(PS_AUTO_ENABLE | PS_AUTO_STATE | PS_PARENT_MISSING |
			PS_DEV_DISABLE | PS_FLAGS | PS_ACTUAL)) | (readl(cpu) & PS_AUTO_STATE);
		/* SYS, FE, CPU target, saved gate A/B, actual-state poll, AUTO_ENABLE. */
		writel(keep, cpu);
		if (!d) {
			writel(r->gate_saved[0], s->disp_gate + 0x51c);
			writel(r->gate_saved[1], s->disp_gate + 0x540);
			/* Restore result must reach the DCP's warm-start gate. */
			if (readl(s->disp_gate + 0x51c) != r->gate_saved[0] ||
			    readl(s->disp_gate + 0x540) != r->gate_saved[1])
				r->late_errors++;
		}
		r->late_errors += !!readl_poll_timeout_atomic(cpu, now,
			FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, saved), 1, 5000);
		if (saved & PS_AUTO_ENABLE)
			writel(keep | PS_AUTO_ENABLE, cpu);
		/*
		 * The set's children the stop lowered (the panel's DP, SPMI and SPI words among them)
		 * go back to what they read before: nothing else raises them, and the display firmware
		 * stalls at its first link access without them.
		 */
		for (j = 0; j < t6050_disp_nchildren[d]; j++) {
			if (!(r->disp_children_lowered[d] & BIT(j)))
				continue;
			r->late_errors += !!sleep_ps_restore(sleep_region_reg(s,
				t6050_no_ps[t6050_disp_children[d][j]].word), r->disp_child_saved[d][j]);
		}
		r->disp_children_lowered[d] = 0;
	}
	r->display_lowered = 0;
	bitmap_zero(r->manager_down, s->soc->regions->power.count);
}

static void t6050_sleep_restore_pmc(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	if (!r->pmc_saved)
		return;
	for (i = 0; i < 5; i++)
		if (r->floor_changed & BIT(i))
			writel(r->floor[i], s->pmc_voters + M5_PMC_STRIDE * i);
	r->floor_changed = 0;
	for (i = 0; i < 4; i++)
		writel(r->rails[i], s->pmc_rails + 0x2000 + 4 * i);
	writel(r->rails[4], s->pmc_rails + 0x2028);
	writel(r->rails[5], s->pmc_rails + 0x2020);
	r->pmc_saved = false;
}

/*
 * The stopped DARTs' GAPF slices back to what they held before their stop, before any of those
 * DARTs is used again: SIO, PMP0, SEP, then the radio's.
 */
static void t6050_sleep_restore_slices(struct apple_sleep *s)
{
	static const u8 order[] = { T6050_DART_SIO, T6050_DART_PMP0, T6050_DART_SEP,
				    T6050_DART_APCIE0 };
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	for (i = 0; r->gapf && i < ARRAY_SIZE(order); i++)
		if (r->dart_slice_written & BIT(order[i]))
			writel(r->dart_slice_saved[order[i]], r->gapf + r->dart_slice[order[i]]);
	r->dart_slice_written = 0;
}

/*
 * After a refusal at .enter: what the entry and the stage took down goes back, last first, each
 * word to what it read before. Nothing else raises them: the wake's replay only runs after a
 * sleep, and the power-domain driver still counts these words as on. Left down, the first
 * refusal took the camera's domains with it (isp_sys), and the next access to one was an SError.
 * The PMC's channels get their request bit back as the display's own restart gives it back.
 */
static void t6050_sleep_refusal_raise(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	void __iomem *req = s->pmc_rails + T6050_PMC_CHANNELS;
	u32 late = r->late_lowered, state4 = r->state4_set, pmc = r->pmc154_lowered;
	unsigned int i, bad = 0;
	u32 status;

	for (i = ARRAY_SIZE(t6050_state4); i-- > 0;) {
		if (!(r->state4_set & BIT(i)))
			continue;
		if (sleep_ps_restore(sleep_region_reg(s, t6050_state4[i].word), r->state4_saved[i])) {
			bad++;
			dev_warn(s->dev, "refusal: %s not back: reads %#x, was %#x\n",
				 t6050_state4[i].name,
				 readl(sleep_region_reg(s, t6050_state4[i].word)), r->state4_saved[i]);
		}
	}
	r->state4_set = 0;
	t6050_venc0_restore(s);
	/* AUSS and its children: between the caps and the idle words, as the entry had them */
	t6050_auss_restore(s);
	for (i = ARRAY_SIZE(t6050_late_off); i-- > 0;) {
		if (!(r->late_lowered & BIT(i)))
			continue;
		if (sleep_ps_restore(sleep_region_reg(s, t6050_late_off[i].word), r->late_saved[i])) {
			bad++;
			dev_warn(s->dev, "refusal: %s not back: reads %#x, was %#x\n",
				 t6050_late_off[i].name,
				 readl(sleep_region_reg(s, t6050_late_off[i].word)), r->late_saved[i]);
		}
	}
	r->late_lowered = 0;
	for (i = r->npmc_channels; i-- > 0;) {
		unsigned int n = r->pmc_channels[i];

		if (!(r->pmc154_lowered & BIT(n)))
			continue;
		writel(readl(req) | T6050_PMC_CHANNEL_ON(n), req);
		if (readl_poll_timeout_atomic(s->pmc_rails + T6050_PMC_CHANNEL_IDLE, status,
					      status == T6050_PMC_CHANNELS_IDLE, 1, 2000)) {
			bad++;
			dev_warn(s->dev, "refusal: PMC channel %u not back: request %#x, status %#x\n",
				 n, readl(req), status);
		}
	}
	r->pmc154_lowered = 0;
	if (late || state4 || pmc)
		dev_info(s->dev, "refusal: raised again what the entry lowered: idle words %#x, state-4 caps %#x, PMC channels %#x (request %#x, was %#x); %u not back\n",
			 late, state4, pmc, readl(req), r->pmc154[0], bad);
	r->late_errors += bad;
	/*
	 * A word that went off and on again is back, but its block's registers are not: they
	 * read as after a reset, as they do after a real sleep, and no driver's resume writes
	 * them for a sleep that didn't happen. Kept for apple_sleep_refusal_cycled(), and named
	 * once here.
	 */
	if (late) {
		char names[160];
		int n = 0;

		r->refusal_cycled = late;
		names[0] = 0;
		for (i = 0; i < ARRAY_SIZE(t6050_late_off); i++)
			if (late & BIT(i))
				n += scnprintf(names + n, sizeof(names) - n, " %s",
					       t6050_late_off[i].name);
		dev_info(s->dev, "refusal: switched off and on again, their blocks' registers at reset:%s\n",
			 names);
	}
}

/*
 * For a driver whose block sits in @pd, a power controller's node, and keeps its registers
 * while that word stays on: whether the last entry, refused, switched the word off and on
 * again. True until the next sleep begins.
 */
bool apple_sleep_refusal_cycled(struct device_node *pd)
{
	struct apple_sleep *s = apple_sleep;
	struct apple_sleep_region_state *r = s ? s->region : NULL;
	unsigned int i;
	int word;

	if (!r || !r->refusal_cycled || !pd)
		return false;
	word = sleep_region_domain_offset(s, pd);
	if (word < 0)
		return false;
	for (i = 0; i < ARRAY_SIZE(t6050_late_off); i++)
		if ((r->refusal_cycled & BIT(i)) && t6050_late_off[i].word == word)
			return true;
	return false;
}
EXPORT_SYMBOL_GPL(apple_sleep_refusal_cycled);

static void t6050_sleep_undo_regs(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;

	/* Never write +c on abort: the requests keep iBoot's values. */
	t6050_wake_disarm(s);
	if (r->staged && !r->committed)
		writel(r->park_saved, r->base[T6050_PERF] + 0x18000);
	t6050_sleep_restore_pmc(s);
	t6050_sleep_restore_display(s);
	t6050_sleep_restore_slices(s);
	/* After a sleep the wake's replay has raised them from the snapshot already. */
	if (r->committed)
		r->auss_lowered = r->late_lowered = r->state4_set = r->pmc154_lowered = 0;
	else
		t6050_sleep_refusal_raise(s);
	r->staged = false;
	r->snapshot_valid = false;
}

static int t6050_ans_storage_restore(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(t6050_ans_storage_on); i++) {
		u32 word = SLEEP_REGION_WORD(T6050_PS1,
			t6050_ans_storage_ps[t6050_ans_storage_on[i]]);
		int n = t6050_ans_storage_index(s, word);
		void __iomem *reg;
		u32 value, keep;

		if (n < 0 || test_bit(n, r->driver_down) || test_bit(n, r->manager_down) ||
		    (r->saved[n] & PS_TARGET) != PS_TARGET)
			return -EIO;
		/* As macOS at the wake: forced on, polled, then automatic again. */
		reg = sleep_region_reg(s, word);
		keep = (r->saved[n] & ~(PS_AUTO_ENABLE | PS_AUTO_STATE | PS_PARENT_MISSING |
					PS_DEV_DISABLE | PS_FLAGS | PS_ACTUAL)) |
		       (readl(reg) & PS_AUTO_STATE);
		writel(keep, reg);
		if (readl_poll_timeout_atomic(reg, value, FIELD_GET(PS_ACTUAL, value) == 0xf, 1,
					      PS_OFF_TIMEOUT_US))
			ret = -ETIMEDOUT;
		if (r->saved[n] & PS_AUTO_ENABLE)
			writel(keep | PS_AUTO_ENABLE, reg);
	}
	return ret;
}

/*
 * After the replay: each power word it could not raise is named, by bank and offset, with the
 * word as the snapshot, the wake and the replay left it; and AUSS, if a word the entry lowered
 * is not back. Reads and console lines only.
 *
 * A word under automatic power that was not fully on at the snapshot is not a loss: the
 * replay's wait is for the snapshot's target, which the hardware gives only when the block is
 * wanted, and its owner starts it later. The SEP's is one. Such a word is left out of the
 * count, when what the OS controls in it (the target, AUTO_ENABLE, DEV_DISABLE) reads as
 * before the sleep.
 */
#define T6050_WAKE_REPORT_WORDS	8

static bool t6050_wake_word_expected(u32 saved, u32 now)
{
	u32 control = PS_AUTO_ENABLE | PS_DEV_DISABLE | PS_TARGET;

	return (saved & PS_AUTO_ENABLE) &&
	       FIELD_GET(PS_ACTUAL, saved) != FIELD_GET(PS_TARGET, saved) &&
	       !((saved ^ now) & control);
}

static void t6050_wake_report(struct apple_sleep *s)
{
	const struct apple_sleep_region_profile *p = s->soc->regions;
	struct apple_sleep_region_state *r = s->region;
	unsigned int i, shown = 0, restored = 0, timeouts = 0, expected = 0;

	for (i = 0; i < p->power.count; i++)
		if (r->late[i] && t6050_wake_word_expected(r->saved[i],
							   readl(sleep_region_reg(s, p->power.offset[i]))))
			expected++;
	if (s->ps_rewake_timeouts > expected)
		dev_warn(s->dev, "%u power words not back at the wake\n",
			 s->ps_rewake_timeouts - expected);
	for (i = 0; i < p->power.count && shown < T6050_WAKE_REPORT_WORDS; i++) {
		u32 word = p->power.offset[i], now;

		if (!r->late[i])
			continue;
		now = readl(sleep_region_reg(s, word));
		if (t6050_wake_word_expected(r->saved[i], now)) {
			dev_dbg(s->dev, "wake: %s+%#x under automatic power and not fully on before the sleep, as expected: %#x before the sleep, %#x at the wake, %#x now\n",
				p->region[SLEEP_REGION(word)].name, (u32)SLEEP_REGION_OFFSET(word),
				r->saved[i], r->woke[i], now);
			continue;
		}
		shown++;
		dev_warn(s->dev, "wake: %s+%#x not back: %#x before the sleep, %#x at the wake, %#x now\n",
			 p->region[SLEEP_REGION(word)].name, (u32)SLEEP_REGION_OFFSET(word),
			 r->saved[i], r->woke[i], now);
	}
	if (!r->auss_lowered)
		return;
	for (i = 0; i < ARRAY_SIZE(t6050_auss_words); i++) {
		int n = sleep_ps_find(p->power.offset, p->power.count, t6050_auss_words[i]);
		u32 now;

		if (n < 0 || !(r->auss_lowered & BIT(i)))
			continue;
		now = readl(sleep_region_reg(s, t6050_auss_words[i]));
		if (FIELD_GET(PS_TARGET, r->saved[n]) &&
		    FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, r->saved[n]))
			restored |= BIT(i);
		timeouts += r->late[n];
	}
	i = ARRAY_SIZE(t6050_auss_words) - 1;
	if (restored != r->auss_lowered || timeouts)
		dev_warn(s->dev, "wake: AUSS %#x (restored %#x of %#x, %u timed out)\n",
			 readl(sleep_region_reg(s, t6050_auss_words[i])), restored,
			 r->auss_lowered, timeouts);
	else
		dev_dbg(s->dev, "wake: AUSS %#x (restored %#x of %#x, %u timed out)\n",
			readl(sleep_region_reg(s, t6050_auss_words[i])), restored,
			r->auss_lowered, timeouts);
}

/*
 * The AP wake sources by number, as the ADT's /arm-io/pmgr ap-wake-sources names them (the
 * J714s's and the J716c's are the same). Their status words are sticky: each source that
 * raised a wake keeps its bit set, at +0x34 for the ten words on this die and at +0x10 for the
 * GFX die's one.
 */
#define T6050_WAKE_STATUS	0x34
#define T6050_WAKE_STATUS_GFX	0x10

static const struct {
	u16 id;
	const char *name;
} t6050_wake_names[] = {
	{ 16, "AOP.GPIO.IRQ4" }, { 18, "AOP.GPIO.IRQ6" }, { 28, "AOP.Outbox0_NotEmpty" },
	{ 30, "AOP.Outbox2_NotEmpty" }, { 38, "AOP2.Outbox2_NotEmpty" }, { 39, "AOP.CPUWakeupAP" },
	{ 62, "NUB.WakeupAP" }, { 65, "NUB.GPIO.IRQ2" }, { 69, "NUB.GPIO.IRQ6" },
	{ 71, "NUB.APWatchdog" }, { 72, "NUB.APWakeupTime" }, { 73, "NUB.SPMI0Sw0IRQ" },
	{ 74, "NUB.SPMI0Sw1IRQ" }, { 76, "NUB.SPMI0Sw3IRQ" }, { 122, "SMC.CPUWakeupAP" },
	{ 127, "SMC.OutboxNotEmpty" }, { 147, "ATC0.PMGRCIOWakeup" }, { 148, "ATC1.PMGRCIOWakeup" },
	{ 149, "ATC2.PMGRCIOWakeup" }, { 150, "ATC3.PMGRCIOWakeup" }, { 151, "ATC0.PMGRUSBWakeup" },
	{ 152, "ATC1.PMGRUSBWakeup" }, { 153, "ATC2.PMGRUSBWakeup" }, { 154, "ATC3.PMGRUSBWakeup" },
	{ 171, "MTP.DOCK.CHANNELS.AP0.IRQ" }, { 183, "MTP.OutboxNotEmpty" },
	{ 290, "AOP2.CPUWakeupAP" },
};

/* At the start of the wake, before anything is touched: the sticky wake status */
static void t6050_wake_source_read(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;

	r->wake_status_valid = false;
	if (!r->wake || !r->wake_gfx)
		return;
	for (i = 0; i < T6050_WAKE_WORDS; i++)
		r->wake_status[i] = readl(r->wake + T6050_WAKE_STATUS + 4 * i);
	r->wake_status[T6050_WAKE_WORDS] = readl(r->wake_gfx + T6050_WAKE_STATUS_GFX);
	r->wake_status_valid = true;
	r->wake_status_fresh = true;
}

/* Raw latched status, not exclusive attribution or proof that a source was enabled. */
static void t6050_wake_source_report(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int w, b, i, id;

	BUILD_BUG_ON(ARRAY_SIZE(r->wake_status) != T6050_WAKE_WORDS + 1);
	if (!r->wake_status_valid)
		return;
	r->wake_status_valid = false;
	for (w = 0; w < T6050_WAKE_WORDS; w++)
		for (b = 0; b < 32; b++) {
			if (!(r->wake_status[w] & BIT(b)))
				continue;
			id = 32 * w + b;
			for (i = 0; i < ARRAY_SIZE(t6050_wake_names); i++)
				if (t6050_wake_names[i].id == id)
					break;
			if (i < ARRAY_SIZE(t6050_wake_names))
				dev_info(s->dev, "wake status set: AP %u %s\n", id,
					 t6050_wake_names[i].name);
			else
				dev_info(s->dev, "wake status set: AP %u\n", id);
		}
	for (b = 0; b < 32; b++)
		if (r->wake_status[T6050_WAKE_WORDS] & BIT(b))
			dev_info(s->dev, "wake status set: GFX %u\n", b);
}

/* Only the SMC's lines are latched: its own wake, whatever else was armed. */
static bool t6050_smc_woke_alone(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u32 smc[T6050_WAKE_WORDS] = {};
	unsigned int i;
	bool any = false;

	if (!r || !r->wake || !r->wake_gfx || r->wake_status[T6050_WAKE_WORDS])
		return false;
	for (i = 0; i < ARRAY_SIZE(t6050_wake_names); i++) {
		u32 id = t6050_wake_names[i].id;

		if (strstarts(t6050_wake_names[i].name, "SMC.") &&
		    id / 32 < T6050_WAKE_WORDS)
			smc[id / 32] |= BIT(id % 32);
	}
	for (i = 0; i < T6050_WAKE_WORDS; i++) {
		if (r->wake_status[i] & ~smc[i])
			return false;
		any |= !!r->wake_status[i];
	}

	return any;
}

/* @n words appended to @buf at @len as " %08x"; the new length */
static int t6050_wake_words(char *buf, int len, int size, const u32 *w, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		len += scnprintf(buf + len, size - len, " %08x", w[i]);
	return len;
}

/*
 * After each wake, one line in the form of macOS's console's ("system wake events: NUB.SPMI0Sw3IRQ
 * nub-spmi0.0x02 nub-spmi-a1.0x0B rtc", wm2): the AP wake sources whose status is set, by their ADT
 * names; the SPMI sources the wake acknowledged (latched and enabled), as <controller>.0x<source>
 * with nub-spmi0 by its ADT name and the others by their nodes; and rtc if the PMU's alarm had
 * fired. The SMC's reasons are its wake record's. Like macOS's, these are latched events, not
 * proof that one of them alone raised the wake. The words as read go to the debug log.
 */
static void t6050_wake_events_report(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	char ev[320], raw[320];
	const int lim = sizeof(ev) - 4;	/* room for " ..." */
	struct device_node *np;
	unsigned int w, b, i, id;
	int n = 0, m;

	if (r->wake_status_fresh) {
		for (w = 0; w < T6050_WAKE_WORDS; w++)
			for (b = 0; b < 32; b++) {
				if (!(r->wake_status[w] & BIT(b)))
					continue;
				id = 32 * w + b;
				for (i = 0; i < ARRAY_SIZE(t6050_wake_names); i++)
					if (t6050_wake_names[i].id == id)
						break;
				if (i < ARRAY_SIZE(t6050_wake_names))
					n += scnprintf(ev + n, lim - n, " %s",
						       t6050_wake_names[i].name);
				else
					n += scnprintf(ev + n, lim - n, " AP%u", id);
			}
		for (b = 0; b < 32; b++)
			if (r->wake_status[T6050_WAKE_WORDS] & BIT(b))
				n += scnprintf(ev + n, lim - n, " GFX%u", b);
	}
	for (b = 0; r->spmi_src_valid && b < ARRAY_SIZE(r->spmi_src_acked); b++) {
		np = b ? of_parse_phandle(s->dev->of_node, "apple,spmi-buses", b - 1) : NULL;
		for (i = 0; i < 32 * T6050_SPMI_SRC_WORDS; i++) {
			if (!(r->spmi_src_acked[b][i / 32] & BIT(i % 32)))
				continue;
			if (np)
				n += scnprintf(ev + n, lim - n, " %pOFP.0x%02X", np, i);
			else
				n += scnprintf(ev + n, lim - n, " %s.0x%02X",
					       b ? "?" : "nub-spmi0", i);
		}
		of_node_put(np);
	}
	if (!r->pmu_wake_rtc_ret && (r->pmu_wake_rtc_ctl & T6050_RTC_CTL_FIRED))
		n += scnprintf(ev + n, lim - n, " rtc");
	if (n >= lim - 1)
		n += scnprintf(ev + n, sizeof(ev) - n, " ...");
	dev_info(s->dev, "wake events:%s\n", n ? ev : " none");

	if (r->wake_status_fresh) {
		m = scnprintf(raw, sizeof(raw), "AP status");
		m = t6050_wake_words(raw, m, sizeof(raw), r->wake_status, T6050_WAKE_WORDS + 1);
		dev_dbg(s->dev, "wake events: %s\n", raw);
	}
	for (b = 0; r->spmi_src_valid && b < ARRAY_SIZE(r->spmi_src_acked); b++) {
		for (i = 0; i < T6050_SPMI_SRC_WORDS; i++)
			if (r->spmi_src_latched[b][i] | r->spmi_src_enabled[b][i])
				break;
		if (i == T6050_SPMI_SRC_WORDS)
			continue;
		np = b ? of_parse_phandle(s->dev->of_node, "apple,spmi-buses", b - 1) : NULL;
		if (np)
			m = scnprintf(raw, sizeof(raw), "%pOFP latched", np);
		else
			m = scnprintf(raw, sizeof(raw), "%s latched", b ? "?" : "nub-spmi0");
		of_node_put(np);
		m = t6050_wake_words(raw, m, sizeof(raw), r->spmi_src_latched[b],
				     T6050_SPMI_SRC_WORDS);
		m += scnprintf(raw + m, sizeof(raw) - m, ", enabled");
		m = t6050_wake_words(raw, m, sizeof(raw), r->spmi_src_enabled[b],
				     T6050_SPMI_SRC_WORDS);
		dev_dbg(s->dev, "wake events: %s\n", raw);
	}
}

static void t6050_sleep_wake_regs(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	unsigned int i;
	u32 value;

	t6050_wake_source_read(s);

	if (r->spmi_masked) {
		/* What the controllers latched while the SoC slept, before the run-time masks */
		t6050_spmi_src_ack(s);
		writel(r->spmi_mask[0], s->spmi + M5_SPMI_IRQ_EN);
		for (i = 0; i < ARRAY_SIZE(r->spmi_bus); i++)
			if (r->spmi_bus[i])
				writel(r->spmi_mask[1 + i], r->spmi_bus[i] + M5_SPMI_IRQ_EN);
		r->spmi_masked = false;
		r->spmi_post_timer = false;
	}

	if (r->aic_gfx) {
		r->gfx_aic[1] = readl(r->aic_gfx + 0x14);
		writel(r->gfx_aic[0], r->aic_gfx + 0x14);
		r->gfx_aic[2] = readl(r->aic_gfx + 0x14);
	}
	/* The complexes' requests back, in macOS's early wake order */
	writel(0, s->wdt + WDT_WD2_CTL);
	apple_sleep_set_wd2(s, WDT_SUSPEND_SECS);
	for (i = 0; i < 3; i++)
		writel(r->cluster_saved[i], s->cluster + 4 * i);
	writel(PWRGATE_ON, s->pwrgate);
	writel(PWRGATE_ON, s->pwrgate + 0xc0);
	value = readl(r->base[T6050_PS1] + 0x298);
	if (value & PS_DEV_DISABLE)
		writel(value & ~PS_DEV_DISABLE, r->base[T6050_PS1] + 0x298);
	writel(r->cluster_saved[4], s->cluster + 0x10);
	/* Replay only Linux snapshots; driver_down excludes owners that resume later. */
	sleep_region_replay(s);
	if (s->pcie_ans_stopped) {
		int ret = t6050_ans_storage_restore(s);

		/* A final-stop failure stays sticky even if domains returned. */
		if (s->ans_m5_wake_error != -EIO)
			s->ans_m5_wake_error = ret ?: s->ans_off_ret;
		/* Only ANS's own word and its parents fence it; another word is only reported. */
		for (i = 0; i < r->nans_chain; i++)
			if (r->late[r->ans_chain[i]])
				s->ans_m5_wake_error = -EIO;
	}
	t6050_wake_report(s);
	t6050_sleep_restore_pmc(s);
	/* Request word is Linux's saved state; report ownership remains until finish. */
	if (r->ptd_valid) {
		u64 owned = BIT_ULL(r->ptd_bit[0]) | BIT_ULL(r->ptd_bit[1]) |
			    BIT_ULL(r->ptd_bit[2]) | BIT_ULL(r->ptd_bit[3]);
		u64 request = readq(s->ptd + M5_PTD_READ(r->ptd_req));

		writeq((request & ~owned) | (r->ptd_saved & owned),
		       s->ptd + M5_PTD_WRITE(r->ptd_req));
	}
	t6050_sleep_restore_display(s);
	/* Every AP pin but the skipped ones reset in the sleep: all put back, before the PCIe PHY start */
	s->pins_lost = s->pins_saved_valid;
	apple_sleep_restore_pins();
}

static void t6050_sleep_finish(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u8 value;
	int ret;

	if (!r->attempt_active)
		return;
	r->attempt_active = false;
	t6050_wake_source_report(s);
	t6050_clusters_down_first_report(s);
	t6050_sleep_undo_regs(s);
	/* Before any device resumes: the radio host's noirq resume asks for the result. */
	if (r->committed && r->n1_host) {
		r->n1_up_ret = t6050_n1_unpark(s);
		if (r->n1_up_ret)
			dev_err(s->dev, "radio's PCIe PHY and root complex not back: %d\n", r->n1_up_ret);
	}
	/*
	 * After a wake the PMU's steps wait for the SMC's first wake commands
	 * (t6050_sleep_pmu_wake). After a reversible refusal the DT's stage goes back now. No PMU
	 * sleep/wake command.
	 */
	ret = 0;
	if (r->committed) {
		r->pmu_wake_pending = true;
	} else {
		value = r->pmu_wake_stage;
		if (r->boot_stage_written && r->pmu_wake_stage_valid)
			ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_PMU_BOOT_STAGE,
						   &value, NULL, 1);
		r->boot_stage_written = false;
	}
	t6050_sleep_ptd_ownership(s, false);
	r->usb_aon_kept = 0;
	dev_dbg(s->dev, "entry %s: replay %u, timeouts %u, late %u, PMU %u/%d, DART stops %#x\n",
		r->committed ? "returned" : "refused", s->ps_rewoken, s->ps_rewake_timeouts,
		r->late_errors, r->pmu_errors, ret, r->dart_done);
}

/*
 * After a wake, once the SMC has had AP ON, its wake record read and waka (apple_sleep_report),
 * the PMU's steps in macOS's order: its RTC control read, the DT's boot stage, the DT's PMU
 * reads as at the entry. In the macOS-HV traces mxt4 and dk1 they start 95 ms into the wake,
 * 27 ms after waka (no wait here: the SMC's steps are done when the PMU's begin). Linux wrote the
 * boot stage first, and the SMC's MBSE then read 0x106fe where macOS's reads 0x10650. Then the
 * wake's events.
 */
static void t6050_sleep_pmu_wake(struct apple_sleep *s)
{
	struct apple_sleep_region_state *r = s->region;
	u8 value, rd[sizeof_field(struct apple_sleep_region_state, pmu_rd)];
	unsigned int i, n;
	int ret = 0;

	if (!r->pmu_wake_pending)
		return;
	r->pmu_wake_pending = false;
	r->pmu_wake_rtc_ctl = 0;
	r->pmu_wake_rtc_ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, T6050_RTC_CTL, NULL,
						   &r->pmu_wake_rtc_ctl, 1);
	if (r->boot_stage_written && r->pmu_wake_stage_valid) {
		value = r->pmu_wake_stage;
		ret = t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_WRITEL, T6050_PMU_BOOT_STAGE, &value,
					   NULL, 1);
		for (i = 0, n = 0; i < r->npmu_reads; n += r->pmu_reads[i][1], i++) {
			if (n + r->pmu_reads[i][1] > sizeof(rd))
				break;
			t6050_sleep_pmu_xfer(s, T6050_SPMI_EXT_READL, r->pmu_reads[i][0], NULL,
					     rd + n, r->pmu_reads[i][1]);
		}
	}
	r->boot_stage_written = false;
	if (ret)
		dev_warn(s->dev, "PMU boot stage not put back after the wake: %d\n", ret);
	t6050_wake_events_report(s);
	r->spmi_src_valid = false;
	r->wake_status_fresh = false;
}

static const struct apple_sleep_ops t6050_sleep_ops = {
	/* As T8140's */
	.deep_wfi_arm = t8140_deep_wfi_arm,
	.resume = cpu_resume,
	.aic_mask_all = aic_sleep_mask_all,
	.aic_ack = aic_sleep_ack,
	.aic_restore = aic_sleep_restore,
	.plan_complex_request = t6050_sleep_plan_complex_request,
	.map_regs = t6050_sleep_map_regs,
	.begin = t6050_sleep_begin,
	.check = t6050_sleep_check_alarm,
	.display_off = t6050_sleep_display_prepare,
	.display_stop = sleep_v27_final_stop,
	.before_enclave = t6050_sleep_before_enclave,
	.stage = t6050_sleep_stage,
	.final_check = t6050_sleep_final_check,
	.commit_notice = t6050_sleep_commit_notice,
	.after_ans = t6050_sleep_after_ans,
	.prepare_regs = t6050_sleep_prepare_regs,
	.undo_regs = t6050_sleep_undo_regs,
	.commit_begin = t6050_sleep_pre_commit,
	.commit_regs = t6050_sleep_commit_regs,
	.post_timer = t6050_sleep_post_timer,
	.cpu_down = t6050_cpu_down,
	.wake = t6050_sleep_wake_regs,
	.finish = t6050_sleep_finish,
	.pmu_wake = t6050_sleep_pmu_wake,
	.cpu_prepare = t6050_sleep_cpu_prepare,
	.cpu_restore = t6050_sleep_cpu_restore,
};

static const struct apple_sleep_soc t6050_sleep_soc = {
	.strict_gate_inventory = true,
	.gate_coprocs = t6050_gate_coprocs,
	.ngate_coprocs = ARRAY_SIZE(t6050_gate_coprocs),
	.gate_off = t6050_gate_off,
	.ngate_off = ARRAY_SIZE(t6050_gate_off),
	.ops = &t6050_sleep_ops,
	.smc_policy = &t6050_smc_policy,
	.early_context = true,
	.own_wake = true,
	.machine = "apple,j714s",
	/* The interrupt mask every SPMI bus has through the sleep, as on T8140 and T8132 */
	.spmi_notify_sleep = 0x2000001,
	.boot_mpidr = 0x040000,
	.cpu_count = 18,
	.cpu_cluster_affinity = 1,
	.cpu_clusters = 3,
	.cpu_cores_per_cluster = 6,
	.cpu_start_stride = 6,
	.cpu_valid = t6050_sleep_cpu_valid,
	.ndisp = 5,
	.regions = &t6050_region_profile,
	.ps = {
		.ans = SLEEP_REGION_WORD(T6050_PS1, 0x140),
		.apcie_sys_gp = SLEEP_REGION_WORD(T6050_PS1, 0x280),
		.sep = SLEEP_REGION_WORD(T6050_PS1, 0x298),
		.sio_cpu = SLEEP_REGION_WORD(T6050_PS1, 0x2a0),
		.disp = {
			{ SLEEP_REGION_WORD(T6050_PS1, 0x1a8), SLEEP_REGION_WORD(T6050_PS1, 0x198),
			  SLEEP_REGION_WORD(T6050_PS1, 0x180) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x278), SLEEP_REGION_WORD(T6050_PS0, 0x268),
			  SLEEP_REGION_WORD(T6050_PS0, 0x200) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2e0), SLEEP_REGION_WORD(T6050_PS0, 0x2d0),
			  SLEEP_REGION_WORD(T6050_PS0, 0x230) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2b0), SLEEP_REGION_WORD(T6050_PS0, 0x2a0),
			  SLEEP_REGION_WORD(T6050_PS0, 0x220) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2c8), SLEEP_REGION_WORD(T6050_PS0, 0x2b8),
			  SLEEP_REGION_WORD(T6050_PS0, 0x228) },
		},
	},
};

/*
 * The J716c's banks, checked against the addresses its own hardware description gives before
 * anything is mapped, and its display slots against the coprocessors they name.
 */
static int t6050_j716c_apertures(struct apple_sleep *s)
{
	static const resource_size_t base[T6050_REGIONS] = {
		0x280600000ULL, 0x280900000ULL, 0x288300000ULL, 0x2300e80000ULL,
		0x2380600000ULL, 0x2388300000ULL, 0x280760000ULL, 0x2807d4000ULL,
		0x280930000ULL, 0x2300e90000ULL, 0x2380760000ULL, 0x280700000ULL,
		0x2380700000ULL, 0x2807ec000ULL, 0x280950000ULL, 0x23807dc000ULL,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(base); i++) {
		struct resource res;
		int index = of_property_match_string(s->dev->of_node, "reg-names",
						     t6050_regions[i].name);

		if (index < 0 || of_address_to_resource(s->dev->of_node, index, &res) ||
		    res.start != base[i] || resource_size(&res) != t6050_regions[i].size)
			return dev_err_probe(s->dev, -EINVAL, "wrong J716c aperture %s\n",
					     t6050_regions[i].name);
	}
	/* Power words must describe the actual display instance in this slot. */
	for (i = 0; i < s->soc->ndisp; i++) {
		static const resource_size_t asc[] = {
			0x48ae00000ULL, 0x2b2e00000ULL, 0x392e00000ULL,
			0x302e00000ULL, 0x306e00000ULL,
		};
		struct device_node *np __free(device_node) =
			of_parse_phandle(s->dev->of_node, "apple,display-coprocessors", i);
		struct resource res;
		int index;

		if (!np || !of_device_is_compatible(np, i ? "apple,t6050-dcpext" :
							  "apple,t6050-dcp"))
			return -EINVAL;
		index = of_property_match_string(np, "reg-names", "coproc");
		if (index < 0 || of_address_to_resource(np, index, &res) ||
		    res.start != asc[i] || resource_size(&res) != 0x4000)
			return dev_err_probe(s->dev, -EINVAL, "wrong display %u aperture\n", i);
	}
	return 0;
}

/* The full sleep's map, from the same checked banks */
static int t6050_j716c_sleep_map(struct apple_sleep *s)
{
	return t6050_j716c_apertures(s) ?: t6050_sleep_map_regs(s);
}

/* The J714s's operations, with the J716c's checked map. */
static const struct apple_sleep_ops t6050_j716c_sleep_ops = {
	.deep_wfi_arm = t8140_deep_wfi_arm,
	.resume = cpu_resume,
	.aic_mask_all = aic_sleep_mask_all,
	.aic_ack = aic_sleep_ack,
	.aic_restore = aic_sleep_restore,
	.plan_complex_request = t6050_sleep_plan_complex_request,
	.map_regs = t6050_j716c_sleep_map,
	.begin = t6050_sleep_begin,
	.check = t6050_sleep_check_alarm,
	.display_off = t6050_sleep_display_prepare,
	.display_stop = sleep_v27_final_stop,
	.before_enclave = t6050_sleep_before_enclave,
	.stage = t6050_sleep_stage,
	.final_check = t6050_sleep_final_check,
	.commit_notice = t6050_sleep_commit_notice,
	.after_ans = t6050_sleep_after_ans,
	.prepare_regs = t6050_sleep_prepare_regs,
	.undo_regs = t6050_sleep_undo_regs,
	.commit_begin = t6050_sleep_pre_commit,
	.commit_regs = t6050_sleep_commit_regs,
	.post_timer = t6050_sleep_post_timer,
	.cpu_down = t6050_cpu_down,
	.wake = t6050_sleep_wake_regs,
	.finish = t6050_sleep_finish,
	.pmu_wake = t6050_sleep_pmu_wake,
	.cpu_prepare = t6050_sleep_cpu_prepare,
	.cpu_restore = t6050_sleep_cpu_restore,
};

/*
 * The J716c (M5 Max): the J714s's entry and wake over the Max inventory. The Max's own power
 * words (the second encoder, AMCC16-31) are in no list the entry lowers: saved at the entry
 * and put back at the wake if they dropped. No PMP runs on this board.
 */
static const struct apple_sleep_soc t6050_j716c_sleep_soc = {
	.strict_gate_inventory = true,
	.gate_coprocs = t6050_gate_coprocs,
	.ngate_coprocs = ARRAY_SIZE(t6050_gate_coprocs),
	.gate_off = t6050_gate_off,
	.ngate_off = ARRAY_SIZE(t6050_gate_off),
	.ops = &t6050_j716c_sleep_ops,
	.smc_policy = &t6050_smc_policy,
	.early_context = true,
	.own_wake = true,
	.pmp_unstarted = true,
	.clusters_down_first = true,
	.lower_venc0 = true,
	.machine = "apple,j716c",
	.spmi_notify_sleep = 0x2000001,
	.boot_mpidr = 0x040000,
	.cpu_count = 18,
	.cpu_cluster_affinity = 1,
	.cpu_clusters = 3,
	.cpu_cores_per_cluster = 6,
	.cpu_start_stride = 6,
	.cpu_valid = t6050_sleep_cpu_valid,
	.ndisp = 5,
	.regions = &t6050_j716c_region_profile,
	.ps = {
		.ans = SLEEP_REGION_WORD(T6050_PS1, 0x140),
		.apcie_sys_gp = SLEEP_REGION_WORD(T6050_PS1, 0x280),
		.sep = SLEEP_REGION_WORD(T6050_PS1, 0x298),
		.sio_cpu = SLEEP_REGION_WORD(T6050_PS1, 0x2a0),
		.disp = {
			{ SLEEP_REGION_WORD(T6050_PS1, 0x1a8), SLEEP_REGION_WORD(T6050_PS1, 0x198),
			  SLEEP_REGION_WORD(T6050_PS1, 0x180) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x278), SLEEP_REGION_WORD(T6050_PS0, 0x268),
			  SLEEP_REGION_WORD(T6050_PS0, 0x200) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2e0), SLEEP_REGION_WORD(T6050_PS0, 0x2d0),
			  SLEEP_REGION_WORD(T6050_PS0, 0x230) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2b0), SLEEP_REGION_WORD(T6050_PS0, 0x2a0),
			  SLEEP_REGION_WORD(T6050_PS0, 0x220) },
			{ SLEEP_REGION_WORD(T6050_PS0, 0x2c8), SLEEP_REGION_WORD(T6050_PS0, 0x2b8),
			  SLEEP_REGION_WORD(T6050_PS0, 0x228) },
		},
	},
};

/* The full T6050 entries: the J714s's and the J716c's */
static bool apple_sleep_t6050_full(const struct apple_sleep *s)
{
	return s && (s->soc == &t6050_sleep_soc || s->soc == &t6050_j716c_sleep_soc);
}
