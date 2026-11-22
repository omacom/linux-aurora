/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * PMP CPU memory-floor governor: pure decision logic.
 *
 * No MMIO, locking or time source here, so the same code runs in the kernel,
 * in KUnit and in the host harness (tools/asahi/pmp-floor-tests.py).
 * The caller samples CPU and GPU busy time, calls pmp_floor_gov_step() once
 * per sample and performs the returned transition.
 */
#ifndef _APPLE_PMP_FLOOR_GOV_H
#define _APPLE_PMP_FLOOR_GOV_H

#ifdef __KERNEL__
#include <linux/bits.h>
#include <linux/string.h>
#include <linux/types.h>
#endif

/*
 * A CPU-client SOC-DEV-DVFS request: the valid bit (61) and four-bit levels
 * in the request-class lanes, classes 0x21 to 0x25 at bits 35:32 to 51:48
 * (the lanes of apple,dvfs-request-levels); the default level, bits 4:0,
 * stays 0. The DT names the classes and levels (apple,cpu-memory-floor), so
 * no SoC's lane assignment is coded here.
 */
#define PMP_FLOOR_VALID		BIT_ULL(61)
#define PMP_FLOOR_CLASS_FIRST	0x21
#define PMP_FLOOR_CLASS_LAST	0x25
#define PMP_FLOOR_CLASS_SHIFT	32
#define PMP_FLOOR_CLASS_BITS	4
#define PMP_FLOOR_LEVEL_MAX	0xf
/* <class level> pairs, each class at most once. */
#define PMP_FLOOR_CELLS_MAX	(2 * (PMP_FLOOR_CLASS_LAST - PMP_FLOOR_CLASS_FIRST + 1))

static inline unsigned int pmp_floor_class_shift(unsigned int cls)
{
	return PMP_FLOOR_CLASS_SHIFT + PMP_FLOOR_CLASS_BITS * (cls - PMP_FLOOR_CLASS_FIRST);
}

/*
 * The raised request from the DT's <class level> pairs. Refused: an odd or
 * empty list, a class outside 0x21..0x25 or given twice, a level wider than
 * its lane, and a list whose levels are all 0 (a request for nothing).
 */
static inline bool pmp_floor_word_of(const unsigned int *cells, unsigned int n,
				     unsigned long long *word)
{
	unsigned long long w = PMP_FLOOR_VALID;
	unsigned int i, seen = 0;

	if (!cells || !word || !n || n % 2 || n > PMP_FLOOR_CELLS_MAX)
		return false;
	for (i = 0; i < n; i += 2) {
		unsigned int cls = cells[i], level = cells[i + 1];

		if (cls < PMP_FLOOR_CLASS_FIRST || cls > PMP_FLOOR_CLASS_LAST ||
		    level > PMP_FLOOR_LEVEL_MAX || seen & (1U << (cls - PMP_FLOOR_CLASS_FIRST)))
			return false;
		seen |= 1U << (cls - PMP_FLOOR_CLASS_FIRST);
		w |= (unsigned long long)level << pmp_floor_class_shift(cls);
	}
	if (w == PMP_FLOOR_VALID)
		return false;
	*word = w;
	return true;
}

/* The level a request word carries for @cls. */
static inline unsigned int pmp_floor_level(unsigned long long word, unsigned int cls)
{
	return (word >> pmp_floor_class_shift(cls)) & PMP_FLOOR_LEVEL_MAX;
}

#define PMP_FLOOR_NS_PER_MS	1000000ULL

/* A response is aggregate, so only time out a release after all known
 * independent demand has been clear for a complete confirmation interval.
 * A newer concurrent edge can be later than the caller's sampled time.
 */
static inline bool pmp_floor_drop_timeout(unsigned long long now,
		unsigned long long published, unsigned long long other_changed,
		bool other_on, unsigned int confirm_ms)
{
	unsigned long long quiet = published > other_changed ? published : other_changed;

	return !other_on && now >= quiet &&
		now - quiet >= (unsigned long long)confirm_ms * PMP_FLOOR_NS_PER_MS;
}

struct pmp_floor_gov_cfg {
	unsigned int cpu_busy_pct;	/* any one CPU at or above this counts */
	/*
	 * So does the busy time of one CPU cluster summed over its CPUs, in
	 * percent of one CPU (0: off). A single thread that moves between the
	 * CPUs of a cluster within a sample keeps no one CPU busy enough.
	 */
	unsigned int cpu_sum_pct;
	unsigned int gpu_busy_pct;	/* GPU work outstanding for this share */
	unsigned int raise_samples;	/* consecutive demand samples to raise */
	unsigned int hold_ms;		/* no-demand time before the drop */
	unsigned int min_interval_ms;	/* between any two governor transitions */
};

struct pmp_floor_gov {
	bool raised;
	bool transitioned;		/* @last_transition_ns is valid */
	unsigned int streak;
	unsigned long long last_demand_ns;
	unsigned long long last_transition_ns;
	unsigned long long rate_limited;	/* transitions deferred by min_interval */
};

enum pmp_floor_action {
	PMP_FLOOR_KEEP,
	PMP_FLOOR_RAISE,
	PMP_FLOOR_DROP,
};

/* A foreign producer is anything but an empty word or a bare valid marker. */
static inline bool pmp_floor_initial_ok(unsigned long long word)
{
	return word == 0 || word == PMP_FLOOR_VALID;
}

static inline unsigned int pmp_floor_clamp(unsigned int v, unsigned int lo, unsigned int hi)
{
	return v < lo ? lo : v > hi ? hi : v;
}

/* Share of @wall spent busy, in percent, 0..100. */
static inline unsigned int pmp_floor_busy_pct(unsigned long long busy, unsigned long long wall)
{
	if (!wall)
		return 0;
	if (busy >= wall)
		return 100;
	/* busy < wall; ns * 100 overflows only after ~5 years. */
	return (unsigned int)(busy * 100 / wall);
}

/* CPU busy time from cumulative idle and wall time (cpufreq's get_cpu_idle_time). */
static inline unsigned int pmp_floor_cpu_pct(unsigned long long idle0, unsigned long long wall0,
					     unsigned long long idle1, unsigned long long wall1)
{
	unsigned long long dw, di;

	/* A counter that went backwards (CPU hotplug) is no evidence of load. */
	if (wall1 <= wall0 || idle1 < idle0)
		return 0;
	dw = wall1 - wall0;
	di = idle1 - idle0;
	return di >= dw ? 0 : pmp_floor_busy_pct(dw - di, dw);
}

/* CPU demand of one sample: the busiest CPU, or the busiest cluster's sum. */
static inline bool pmp_floor_cpu_demand(const struct pmp_floor_gov_cfg *c,
					unsigned int busiest_pct, unsigned int cluster_sum_pct)
{
	return busiest_pct >= c->cpu_busy_pct ||
	       (c->cpu_sum_pct && cluster_sum_pct >= c->cpu_sum_pct);
}

static inline bool pmp_floor_rate_ok(const struct pmp_floor_gov *g,
				     const struct pmp_floor_gov_cfg *c,
				     unsigned long long now)
{
	return !g->transitioned ||
	       now - g->last_transition_ns >= c->min_interval_ms * PMP_FLOOR_NS_PER_MS;
}

/*
 * One sample. @demand is true when a CPU or the GPU crossed its threshold in
 * the sample that ended at @now (monotonic ns). Raise after raise_samples
 * consecutive demand samples; drop hold_ms after the last demand. No two
 * transitions are closer than min_interval_ms; a deferred one is retried on
 * the next sample.
 */
static inline enum pmp_floor_action pmp_floor_gov_step(struct pmp_floor_gov *g,
						      const struct pmp_floor_gov_cfg *c,
						      unsigned long long now, bool demand)
{
	enum pmp_floor_action want = PMP_FLOOR_KEEP;

	if (demand) {
		if (g->streak < ~0U)
			g->streak++;
		g->last_demand_ns = now;
	} else {
		g->streak = 0;
	}

	if (!g->raised && demand && g->streak >= (c->raise_samples ? c->raise_samples : 1))
		want = PMP_FLOOR_RAISE;
	else if (g->raised && !demand &&
		 now - g->last_demand_ns >= c->hold_ms * PMP_FLOOR_NS_PER_MS)
		want = PMP_FLOOR_DROP;

	if (want == PMP_FLOOR_KEEP)
		return want;
	if (!pmp_floor_rate_ok(g, c, now)) {
		g->rate_limited++;
		return PMP_FLOOR_KEEP;
	}
	g->raised = want == PMP_FLOOR_RAISE;
	g->transitioned = true;
	g->last_transition_ns = now;
	return want;
}

/* Release outside the governor (disable, suspend, shutdown, error): never rate limited. */
static inline void pmp_floor_gov_reset(struct pmp_floor_gov *g, unsigned long long now)
{
	g->raised = false;
	g->streak = 0;
	g->transitioned = true;
	g->last_transition_ns = now;
}

/*
 * DVFS-STATE response check, on the first entry of that range, for a report
 * whose responses follow the floor (J613). There the low nibble follows the
 * class 0x25 lane of the request and the next nibble the class 0x21 lane
 * (0x3200 with no floor, 0x3204/0x3214/0x3224 with class 0x25 at 4 and
 * class 0x21 at 0/1/2). A raise is confirmed once both reach at least the
 * requested level (a lower firmware limit is reported as unconfirmed, not
 * overridden). A release is confirmed once that byte differs from the one
 * seen while the floor was held.
 */
#define PMP_FLOOR_STATE_LEVEL_MASK 0xf
#define PMP_FLOOR_STATE_HI_SHIFT 4
#define PMP_FLOOR_STATE_MASK 0xff
#define PMP_FLOOR_STATE_LO_CLASS 0x25
#define PMP_FLOOR_STATE_HI_CLASS 0x21

static inline bool pmp_floor_raise_confirmed(unsigned long long resp, unsigned long long word)
{
	return (resp & PMP_FLOOR_STATE_LEVEL_MASK) >=
		       pmp_floor_level(word, PMP_FLOOR_STATE_LO_CLASS) &&
	       ((resp >> PMP_FLOOR_STATE_HI_SHIFT) & PMP_FLOOR_STATE_LEVEL_MASK) >=
		       pmp_floor_level(word, PMP_FLOOR_STATE_HI_CLASS);
}

static inline bool pmp_floor_drop_confirmed(unsigned long long resp, unsigned long long held)
{
	return (resp & PMP_FLOOR_STATE_MASK) != (held & PMP_FLOOR_STATE_MASK);
}

/*
 * Table admission, from this boot's PMP tables. The SOC-DEV-DVFS (id 12) and
 * DVFS-STATE (id 3) ranges of apple,tunable-ptd-range are looked up by id and
 * name, each exactly once, with room for the two CPU clients (two request
 * entries, four response entries). apple,tunable-soc-device must name both
 * CPU clients, EACC0 and PACC0, exactly once. The CPU clients are the first
 * two SOC-DEV-DVFS entries; the floor is requested on the second (PACC0).
 * Where a second P-cluster client (PACC1) is named once too, it is the third
 * entry, and a report that floors every CPU client also uses it.
 * Returns NULL when admitted, else the refusal reason.
 */
#define PMP_FLOOR_REC_SIZE	0x7c	/* soc-device record */
#define PMP_FLOOR_REC_NAME	0x74	/* its 8-byte, NUL-padded name */
#define PMP_FLOOR_RANGE_SIZE	32	/* ptd-range record: id, base, count, name at 16 */
#define PMP_FLOOR_RANGE_NAME	16
#define PMP_FLOOR_RANGE_BASE	4
#define PMP_FLOOR_RANGE_COUNT	8
#define PMP_FLOOR_CPU_RANGE	12
#define PMP_FLOOR_STATE_RANGE	3
#define PMP_FLOOR_CLIENTS	2
#define PMP_FLOOR_PACC0		1	/* CPU client index of the floor */

#define PMP_FLOOR_CLIENTS_MAX	3	/* EACC0, PACC0 and, where named, PACC1 */

struct pmp_floor_layout {
	unsigned int cpu_first;		/* PTD index of SOC-DEV-DVFS entry 0 */
	unsigned int state_first;	/* PTD index of DVFS-STATE entry 0 */
	unsigned int clients;		/* CPU clients, the first SOC-DEV-DVFS entries */
};

/* PTD entry @i: read alias (16 bytes apart) and update alias (8 bytes apart). */
#define PMP_FLOOR_READ_STRIDE	16
#define PMP_FLOOR_UPDATE_STRIDE	8
#define PMP_FLOOR_UPDATE_BASE	0x10000ULL
#define PMP_FLOOR_WORD_SIZE	8
#define PMP_FLOOR_READ(i)	((unsigned long long)(i) * PMP_FLOOR_READ_STRIDE)
#define PMP_FLOOR_UPDATE(i)	(PMP_FLOOR_UPDATE_BASE + \
				 (unsigned long long)(i) * PMP_FLOOR_UPDATE_STRIDE)

static inline unsigned long long pmp_floor_req_read(const struct pmp_floor_layout *l)
{
	return PMP_FLOOR_READ(l->cpu_first + PMP_FLOOR_PACC0);
}

static inline unsigned long long pmp_floor_req_update(const struct pmp_floor_layout *l)
{
	return PMP_FLOOR_UPDATE(l->cpu_first + PMP_FLOOR_PACC0);
}

static inline unsigned long long pmp_floor_state_read(const struct pmp_floor_layout *l,
						      unsigned int i)
{
	return PMP_FLOOR_READ(l->state_first + i);
}

/* The highest offset the owner touches, plus one: the resource must cover it. */
static inline unsigned long long pmp_floor_layout_end(const struct pmp_floor_layout *l)
{
	unsigned int last = l->clients > PMP_FLOOR_PACC0 + 1 ? l->clients - 1 : PMP_FLOOR_PACC0;
	unsigned long long a = PMP_FLOOR_UPDATE(l->cpu_first + last) + PMP_FLOOR_WORD_SIZE;
	unsigned long long b = pmp_floor_req_read(l) + PMP_FLOOR_WORD_SIZE;
	unsigned long long c = pmp_floor_state_read(l, 2 * PMP_FLOOR_CLIENTS - 1) +
			       PMP_FLOOR_WORD_SIZE;

	return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

static inline unsigned int pmp_floor_le32(const unsigned char *p)
{
	return (unsigned int)p[0] | (unsigned int)p[1] << 8 |
	       (unsigned int)p[2] << 16 | (unsigned int)p[3] << 24;
}

/* The one soc-device record named @name, or NULL if none or several are. */
static inline const unsigned char *pmp_floor_named_once(const unsigned char *t, size_t len,
							const char *name)
{
	const unsigned char *rec = NULL;
	char want[8] = { 0 };
	unsigned int i, found = 0;
	size_t pos;

	for (i = 0; i < sizeof(want) && name[i]; i++)
		want[i] = name[i];
	for (pos = 0; pos + PMP_FLOOR_REC_SIZE <= len; pos += PMP_FLOOR_REC_SIZE)
		if (!memcmp(t + pos + PMP_FLOOR_REC_NAME, want, sizeof(want))) {
			found++;
			rec = t + pos;
		}
	return found == 1 ? rec : NULL;
}

/* The single range with @id, named @name, of at least @minimum entries. */
static inline bool pmp_floor_range_find(const unsigned char *t, size_t len, unsigned int id,
					const char *name, unsigned int minimum,
					unsigned int *base)
{
	char want[16] = { 0 };
	unsigned int i, found = 0;
	size_t pos;

	for (i = 0; i < sizeof(want) && name[i]; i++)
		want[i] = name[i];
	for (pos = 0; pos + PMP_FLOOR_RANGE_SIZE <= len; pos += PMP_FLOOR_RANGE_SIZE) {
		if (pmp_floor_le32(t + pos) != id)
			continue;
		if (++found != 1 || memcmp(t + pos + PMP_FLOOR_RANGE_NAME, want, sizeof(want)) ||
		    pmp_floor_le32(t + pos + PMP_FLOOR_RANGE_COUNT) < minimum)
			return false;
		*base = pmp_floor_le32(t + pos + PMP_FLOOR_RANGE_BASE);
	}
	return found == 1;
}

static inline const char *pmp_floor_tables_check(const unsigned char *dev, size_t dlen,
						 const unsigned char *rng, size_t rlen,
						 struct pmp_floor_layout *l)
{
	if (!dev || !dlen || dlen % PMP_FLOOR_REC_SIZE)
		return "soc-device table size";
	if (!pmp_floor_named_once(dev, dlen, "EACC0") || !pmp_floor_named_once(dev, dlen, "PACC0"))
		return "CPU clients EACC0/PACC0 not named once";
	l->clients = pmp_floor_named_once(dev, dlen, "PACC1") ? 3 : PMP_FLOOR_CLIENTS;
	if (!rng || !rlen || rlen % PMP_FLOOR_RANGE_SIZE)
		return "ptd-range table size";
	if (!pmp_floor_range_find(rng, rlen, PMP_FLOOR_CPU_RANGE, "SOC-DEV-DVFS",
				  l->clients, &l->cpu_first))
		return "SOC-DEV-DVFS range";
	if (!pmp_floor_range_find(rng, rlen, PMP_FLOOR_STATE_RANGE, "DVFS-STATE",
				  2 * PMP_FLOOR_CLIENTS, &l->state_first))
		return "DVFS-STATE range";
	/* A base near 2^32 would wrap the alias arithmetic; no real table has one. */
	if (l->cpu_first > 0x100000 || l->state_first > 0x100000)
		return "ptd-range base out of range";
	return NULL;
}

#endif /* _APPLE_PMP_FLOOR_GOV_H */
