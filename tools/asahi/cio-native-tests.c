// SPDX-License-Identifier: GPL-2.0-only
/* Ordered PM model. It deliberately does not emulate silicon or firmware. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int64_t s64;
#define __iomem
#define BIT(n) (1U << (n))
#define GENMASK(h, l) ((~0U << (l)) & (~0U >> (31 - (h))))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define FIELD_GET(m, v) (((v) & (m)) / ((m) & -(m)))
#define READ_ONCE(v) (v)
#define WRITE_ONCE(v, x) ((v) = (x))
#define EXPORT_SYMBOL_GPL(n)
#define PM_SUSPEND_MEM 3
/* The MMIO mock records write order synchronously. */
#define wmb() ((void)0)
/* Arguments are not evaluated, but they are used: a diagnostic's reads change no mock. */
static inline int dev_log(const void *dev, const char *fmt, ...)
{
	(void)dev;
	(void)fmt;
	return 0;
}
#define dev_warn(...) ((void)sizeof(dev_log(__VA_ARGS__)))
#define dev_info(...) ((void)sizeof(dev_log(__VA_ARGS__)))
#define dev_err(...) ((void)sizeof(dev_log(__VA_ARGS__)))
#define dev_emerg(...) ((void)sizeof(dev_log(__VA_ARGS__)))
typedef s64 ktime_t;
static ktime_t ktime_get(void)
{
	return 0;
}
#define ktime_us_delta(a, b) ((s64)((a) - (b)))
#define ktime_ms_delta(a, b) ((s64)((a) - (b)))
typedef struct {
	int counter;
} atomic_t;
#define atomic_read(v) ((v)->counter)
/* Only J714s takes the T6050 paths, and only the cases that ask for them run as one. */
static bool j714s;
static bool of_machine_is_compatible(const char *compat)
{
	assert(!strcmp(compat, "apple,j714s"));
	return j714s;
}
#define list_is_singular(l) ((l)->count == 1)
#define list_first_entry(l, t, m) ((void)sizeof(#m), (t *)(l)->first)
static unsigned int next_set_bit(unsigned long long m, unsigned int n, unsigned int i)
{
	while (i < n && !(m & BIT(i)))
		i++;
	return i;
}

#define for_each_set_bit(i, m, n) \
	for ((i) = next_set_bit(*(m), (n), 0); (i) < (n); (i) = next_set_bit(*(m), (n), (i) + 1))
#define read_poll_timeout_atomic(fn, v, cond, delay, time, sleep, ...) \
	({                                                             \
		(void)(delay);                                         \
		(void)(time);                                          \
		(void)(sleep);                                         \
		int e = -ETIMEDOUT;                                    \
		for (unsigned int j = 0; j < 2; j++) {                 \
			(v) = fn(__VA_ARGS__);                         \
			if (cond) {                                    \
				e = 0;                                 \
				break;                                 \
			}                                              \
		}                                                      \
		e;                                                     \
	})
#define read_poll_timeout read_poll_timeout_atomic
#define readl_poll_timeout(addr, v, cond, delay, time) \
	read_poll_timeout_atomic(readl, v, cond, delay, time, 0, addr)
/* ACTUAL_PORT_DEFINES */
struct list_head {
	void *first;
	unsigned int count;
};
struct device {
	void *data;
	unsigned int index, refs;
	bool on;
};
struct platform_device {
	struct device dev;
};
struct apple_dart_cio {
	unsigned int id;
	bool active, off;
};
struct phy {
	bool prepared, changed, restored;
};
struct hw_info {
	u32 port_perst;
};
struct pci_config_window {
	void *win;
};
struct apple_pcie {
	struct device *dev;
	struct apple_dart_cio *cio_dart;
	int cio_cfg_lock;
	bool cio_prepared, cio_changed, cio_stopped, cio_port_reset, cio_lost,
		bus_stopped, resume_failed;
	struct list_head ports;
	const struct hw_info *hw;
	struct pci_config_window *cfg;
	void *fabric_base, *intr2axi_base;
};
struct apple_pcie_port {
	struct apple_pcie *pcie;
	unsigned long *sid_map;
	int sid_map_sz;
	u32 saved_rid2sid[4], saved_intmask;
	bool started, needs_stop, irq_disabled;
};
struct pd_list {
	struct device *pd_devs[4];
};
struct rtkit {
	bool off;
};
struct tb {
	bool prepared, stopped, lost, restarted;
	int prepares, refusals;
};
struct apple_cio {
	atomic_t fw_at_answers, fw_at_timeouts, fw_wake_seen, fw_lane_action;
	int lock, pcie_tunnel_lock;
	bool removing, native_sleep, lstx_high;
	u32 current_cable_info, target_cable_info;
	struct rtkit *rtk;
	struct pd_list *pd_list;
	void *reset, *rc_base, *pcie_adapter_base, *router_ap_fw_base;
	void *rc_tunable, *pcie_adapter_tunable, *router_ap_fw_tunable;
	struct device *dev;
	void *dp_wq;
};
struct ring_nhi {
	void *tx_rings[2], *rx_rings[2];
};
struct apple_nhi {
	struct apple_cio *acio;
	struct apple_dart_cio *cio_dart;
	struct phy *cio_phy;
	/* T6050's combo PHY calls; this model has T8132's lane PHY, so they stay unset. */
	struct device *cio_phy_dev;
	int (*cio_phy_fn[5])(struct device *dev);
	bool cio_t6050;
	struct device *cio_pcie;
	struct device *dev;
	struct tb *tb;
	struct ring_nhi nhi;
	unsigned int n_rings, cio_pd_suspended;
	bool cio_prepared, cio_rtk_off, cio_cycled, dp_aux_armed;
	int dp_aux_work;
	void *nhi_base, *nhi_tunable;
};
static struct platform_device pdev;
static struct apple_pcie pc;
static struct apple_pcie_port port;
static struct hw_info hw = { .port_perst = PORT_PERST };
static const struct hw_info t6050_pciec_hw = { .port_perst = PORT_PERST };
static struct apple_dart_cio darts[2];
static struct phy phy;
static struct apple_cio cio;
static struct apple_nhi nhi;
static struct pd_list pds;
static struct device pd[4];
static struct rtkit rtk;
static struct tb tb;
static unsigned long sids = 3;
static u32 reg[2048], rid[4], rc[0x100 / 4];
static int pm_suspend_target_state = PM_SUSPEND_MEM;
static int fault, polls, maps, checks, balances[4], resets, finished, callbacks;
static bool pending_fault, expect_panic, cpu_reset;
static bool port_setup, aperture;
static jmp_buf fatal;
static void panic(const char *fmt, ...)
{
	assert(expect_panic);
	longjmp(fatal, 1);
}
static bool fail(int point)
{
	if (pending_fault && fault == point) {
		pending_fault = false;
		return true;
	}
	return false;
}
static struct apple_pcie *apple_pcie_lookup(struct device *d)
{
	return d->data;
}
static bool apple_pcie_tunnel_link_healthy(struct apple_pcie *p)
{
	return !fail(1);
}
static int apple_dart_cio_prepare(struct apple_dart_cio *d)
{
	if (fail(d->id ? 3 : 2))
		return -EBUSY;
	assert(!d->active);
	d->active = true;
	return 0;
}
static int apple_dart_cio_off(struct apple_dart_cio *d)
{
	assert(d->active);
	if (d->id)
		assert(!(reg[PORT_PERST / 4] & PORT_PERST_OFF) &&
		       (reg[PORT_APPCLK / 4] & PORT_APPCLK_EN));
	else
		assert(tb.stopped && (pc.cio_stopped || !nhi.cio_pcie));
	if (d->id && fail(16))
		return -EOPNOTSUPP;
	d->off = true;
	return fail(d->id ? 13 : 20) ? -EIO : 0;
}
static int apple_dart_cio_on(struct apple_dart_cio *d)
{
	assert(d->active);
	if (d->id)
		assert(pd[1].on && (reg[PORT_APPCLK / 4] & PORT_APPCLK_EN));
	else
		assert(!rtk.off && pd[0].on && pd[1].on && pd[2].on &&
		       pd[3].on);
	if (fail(d->id ? 32 : 31))
		return -EIO;
	d->off = false;
	return 0;
}
/* As the real one: always ends; a DART left off stays closed. */
static int apple_dart_cio_finish(struct apple_dart_cio *d)
{
	assert(d->active);
	d->active = false;
	finished++;
	return d->off ? -EIO : 0;
}
static u32 apple_pcie_port_readl(struct apple_pcie_port *p, u32 off)
{
	/* A link that never trains stays down for the whole poll. */
	if (off == PORT_LINKSTS && pending_fault && fault == 33)
		return 0;
	if (off == PORT_STATUS && pending_fault && fault == 14 &&
	    !(reg[PORT_APPCLK / 4] & PORT_APPCLK_EN)) {
		if (++polls == 2)
			pending_fault = false;
		return PORT_STATUS_READY;
	}
	if (off == 0x108c && pending_fault &&
	    (fault == 12 ||
	     (fault == 15 && (reg[0x1088 / 4] & PORT_TUNCTRL_PERST_ACK_REQ)))) {
		u32 val = fault == 12 ? 0 : PORT_TUNSTAT_PERST_ON;

		if (++polls == 2)
			pending_fault = false;
		return val;
	}
	return reg[off / 4];
}
static void apple_pcie_port_writel(struct apple_pcie_port *p, u32 v, u32 off)
{
	if (off == PORT_PERST && !(v & PORT_PERST_OFF) &&
	    (reg[off / 4] & PORT_PERST_OFF)) {
		port_setup = aperture = false;
		memset(rid, 0, sizeof(rid));
	}
	reg[off / 4] = v;
	if (off == 0x1088)
		reg[0x108c / 4] = v & (PORT_TUNCTRL_PERST_ON |
				       PORT_TUNCTRL_PERST_ACK_REQ);
	if (off == PORT_APPCLK)
		reg[PORT_STATUS / 4] =
			(v & PORT_APPCLK_EN) ? PORT_STATUS_READY : 0;
	if (off == PORT_LTSSMCTL) {
		bool train = (v & PORT_LTSSMCTL_START) && port_setup &&
			     aperture && !reg[0x108c / 4] && !darts[1].off;
		reg[PORT_LINKSTS / 4] = train ? PORT_LINKSTS_UP : 0;
	}
}
static void apple_pcie_port_rmw_set(struct apple_pcie_port *p, u32 m, u32 off)
{
	apple_pcie_port_writel(p, reg[off / 4] | m, off);
}
static void apple_pcie_port_rmw_clear(struct apple_pcie_port *p, u32 m, u32 off)
{
	apple_pcie_port_writel(p, reg[off / 4] & ~m, off);
}
static u32 apple_pcie_port_tunctrl(struct apple_pcie_port *p)
{
	return 0x1088;
}
static u32 apple_pcie_port_tunstat(struct apple_pcie_port *p)
{
	return 0x108c;
}
static u32 port_rid2sid_offset(struct apple_pcie_port *p, int i)
{
	return 0x828 + 4 * i;
}
static void apple_pcie_rid2sid_write(struct apple_pcie_port *p, int i, u32 v)
{
	rid[i] = v;
	maps++;
}
static bool apple_pcie_tunnel_transactions_idle(struct apple_pcie_port *p)
{
	if (pending_fault && fault == 11) {
		if (++polls == 2)
			pending_fault = false;
		return false;
	}
	return true;
}
static void apple_pcie_port_disable_irq(struct apple_pcie_port *p)
{
	p->irq_disabled = true;
}
static void apple_pcie_port_enable_irq(struct apple_pcie_port *p)
{
	assert(!darts[1].off);
	p->irq_disabled = false;
}
static void apple_pcie_tunnel_hierarchy_lost(struct device *d)
{
	callbacks++;
}
static void apple_pcie_tunnel_pulse_intr2axi(struct apple_pcie *p)
{
	assert(port_setup && (reg[PORT_PERST / 4] & PORT_PERST_OFF));
	aperture = true;
}
static int apple_pcie_tunnel_cold_init(struct apple_pcie_port *p)
{
	assert(reg[0x1088 / 4] & PORT_TUNCTRL_PERST_ON);
	memset(rid, 0, sizeof(rid));
	resets++;
	port_setup = true;
	apple_pcie_port_rmw_set(p, PORT_PERST_OFF, PORT_PERST);
	apple_pcie_port_rmw_set(p, PORT_APPCLK_EN, PORT_APPCLK);
	apple_pcie_tunnel_pulse_intr2axi(p->pcie);
	return 0;
}
static void apple_pcie_tunnel_restore_irq_hw(struct apple_pcie_port *p)
{
	checks++;
}
static int apple_pcie_tunnel_release_reset(struct apple_pcie_port *p)
{
	assert(!darts[1].off && maps >= 2);
	apple_pcie_port_writel(p, 0, 0x1088);
	return 0;
}
static void apple_pcie_tunnel_link_up(struct apple_pcie *p)
{
}
static void apple_pcie_tunnel_wait_ready(struct apple_pcie *p)
{
}
static int *take(int *l)
{
	assert(!*l);
	*l = 1;
	return l;
}
static void drop(int **l)
{
	if (*l) {
		assert(**l);
		**l = 0;
	}
}
#define scoped_guard(type, l)                                               \
	for (int *g __attribute__((cleanup(drop))) = take(l), *once = NULL; \
	     !once; once = g)
#define CAT_(a, b) a##b
#define CAT(a, b) CAT_(a, b)
#define guard(type) int *CAT(guard_, __COUNTER__) __attribute__((cleanup(drop))) = take
/*
 * T6050 PCIe-C (J714s): compiled, but only its refusal before anything is touched runs here;
 * its PME_Turn_Off, L2 and fabric registers are not modelled.
 */
#define PCI_COMMAND 0x04
#define PCI_PRIMARY_BUS 0x18
#define PCI_EXP_SLTCTL_ABPE 0x0001
#define PCI_EXP_SLTCTL_PDCE 0x0008
#define PCI_EXP_SLTCTL_HPIE 0x0020
#define PCI_EXP_SLTCTL_DLLSCE 0x1000
static u16 readw(void *p)
{
	assert(false);
	return *(u16 *)p;
}
static void writew(u16 v, void *p)
{
	assert(false);
	*(u16 *)p = v;
}
static u32 readl(void *p)
{
	return *(u32 *)p;
}
static void writel(u32 v, void *p)
{
	assert(false);
	*(u32 *)p = v;
}
struct pci_bus;
struct pci_host_bridge {
	struct pci_bus *bus;
};
struct pci_dev {
	int current_state;
};
static void *dev_get_drvdata(struct device *d)
{
	assert(false);
	return NULL;
}
static void pci_walk_bus(struct pci_bus *b, int (*fn)(struct pci_dev *, void *), void *data)
{
	assert(false);
}
static const char *pci_name(struct pci_dev *p)
{
	return "";
}
static const char *pci_power_name(int state)
{
	return "";
}
static int scnprintf(char *buf, size_t size, const char *fmt, ...)
{
	return 0;
}
static unsigned int dart_checks;
static int dart_check_error;
static int apple_dart_cio_check(struct apple_dart_cio *d, bool quiesced)
{
	assert(d->active && !d->off);
	dart_checks++;
	return dart_check_error;
}
static void apple_pcie_tunnel_hierarchy_gone(struct device *d)
{
	assert(false);
}
/* ACTUAL_PCIE */
static void put_device(struct device *d)
{
	if (!d)
		return;
	assert(d->refs);
	d->refs--;
}
static struct platform_device *apple_cio_find_pcie_tunnel(struct apple_cio *a)
{
	if (fail(4))
		return NULL;
	pdev.dev.refs++;
	return &pdev;
}
static void cancel_delayed_work_sync(int *w)
{
}
static void flush_workqueue(void *q)
{
}
static void *system_dfl_wq;
static void mod_delayed_work(void *wq, int *w, int d)
{
}
static int apple_atcphy_cio_prepare(struct phy *p)
{
	if (fail(5))
		return -EBUSY;
	assert(!p->prepared);
	p->prepared = true;
	return 0;
}
static int apple_atcphy_cio_off(struct phy *p)
{
	assert(rtk.off);
	for (unsigned int i = 0; i < 4; i++)
		assert(!pd[i].on);
	p->changed = true;
	return fail(25) ? -EIO : 0;
}
static int apple_atcphy_cio_on(struct phy *p)
{
	assert(p->prepared);
	if (fail(30))
		return -EIO;
	p->restored = true;
	return 0;
}
static int apple_atcphy_cio_finish(struct phy *p)
{
	assert(p->prepared && (!p->changed || p->restored));
	*p = (struct phy){};
	return 0;
}
static int tb_cio_sleep_prepare(struct tb *t)
{
	t->prepares++;
	if (fail(6))
		return -EBUSY;
	t->prepared = true;
	return 0;
}
static int tb_cio_sleep(struct tb *t)
{
	/* The PCIe port behind the router is down first, if there is one. */
	assert(pc.cio_stopped || !nhi.cio_pcie);
	if (fail(19))
		return -EIO;
	t->stopped = true;
	t->restarted = false;
	return 0;
}
/* After the vendor sleep only a controller restart wakes the router. */
static int tb_cio_wake(struct tb *t, bool *pci_lost)
{
	assert(!darts[0].off && !rtk.off && (!t->stopped || t->restarted));
	if (fail(34))
		return -EIO;
	t->stopped = false;
	if (pci_lost)
		*pci_lost = t->lost;
	return 0;
}
static void tb_cio_sleep_complete(struct tb *t)
{
	assert(t->prepared && !t->stopped);
	t->prepared = false;
}
/* A refused prepare: complete if the transaction is open, then the CM's rescan. */
static void tb_cio_sleep_refused(struct tb *t)
{
	assert(!t->stopped);
	t->prepared = false;
	t->refusals++;
}
static int apple_rtkit_poweroff(struct rtkit *r)
{
	assert(tb.stopped && darts[0].off);
	if (fail(21))
		return -EIO;
	r->off = true;
	return 0;
}
static void apple_rtkit_mailbox_off(struct rtkit *r)
{
	assert(r->off);
}
static int apple_cio_lstx_hold(struct apple_cio *a, bool high)
{
	return fail(22) ? -EIO : 0;
}
static void dev_pm_genpd_suspend(struct device *d)
{
	assert(rtk.off && darts[0].off && (darts[1].off || !nhi.cio_pcie));
	balances[d->index]++;
	d->on = !fail(23) ? false : true;
	if (!pd[0].on && !pd[1].on && !pd[2].on && !pd[3].on)
		cpu_reset = true;
}
static bool dev_pm_genpd_is_on(struct device *d)
{
	return d->on;
}
static void dev_pm_genpd_resume(struct device *d)
{
	assert(balances[d->index] == 1);
	balances[d->index]--;
	d->on = true;
}
static int apple_pmgr_ps_verify_state(struct device *d, bool on)
{
	assert(d->on == on);
	return fail(on ? 35 : 24) ? -EIO : 0;
}
/* The ACIO reset only deasserts: its domains going off are what reset the CPU. */
static void usleep_range(unsigned long min, unsigned long max)
{
	(void)min;
	(void)max;
}
static int reset_control_deassert(void *r)
{
	assert(phy.restored);
	rtk.off = false;
	return 0;
}
/* A cold restart needs a CPU that went through reset. */
static int apple_rtkit_cold_restart(struct rtkit *r, int (*start)(void *),
				    void *p)
{
	for (unsigned int i = 0; i < 4; i++)
		assert(pd[i].on);
	assert(cpu_reset);
	cpu_reset = false;
	tb.restarted = true;
	return start(p);
}
static int apple_cio_lstx_release(struct apple_cio *a)
{
	return 0;
}
static void apple_tunable_apply(void *p, void *t)
{
}
/* T6050's sleep in place (J714s) is not modelled: this NHI is T8132's (cio_t6050 unset). */
static bool apple_nhi_cio_peer(struct apple_nhi *a)
{
	assert(false);
	return true;
}
static int apple_nhi_cio_suspend_t6050(struct apple_nhi *a)
{
	assert(false);
	return -EINVAL;
}
static int apple_nhi_cio_power_on_t6050(struct apple_nhi *a)
{
	assert(false);
	return -EINVAL;
}
/* ACTUAL_NHI */
static void setup(void)
{
	memset(&pc, 0, sizeof(pc));
	memset(&nhi, 0, sizeof(nhi));
	memset(&cio, 0, sizeof(cio));
	memset(&port, 0, sizeof(port));
	memset(darts, 0, sizeof(darts));
	memset(&phy, 0, sizeof(phy));
	memset(reg, 0, sizeof(reg));
	memset(&tb, 0, sizeof(tb));
	memset(&rtk, 0, sizeof(rtk));
	memset(balances, 0, sizeof(balances));
	memset(rid, 0, sizeof(rid));
	fault = polls = maps = checks = resets = finished = callbacks = 0;
	pending_fault = expect_panic = cpu_reset = false;
	j714s = false;
	dart_checks = dart_check_error = 0;
	port_setup = aperture = true;
	pdev.dev = (struct device){ .data = &pc, .refs = 1 };
	pc.dev = &pdev.dev;
	pc.hw = &hw;
	pc.cio_dart = &darts[1];
	darts[1].id = 1;
	pc.ports = (struct list_head){ .first = &port, .count = 1 };
	port.pcie = &pc;
	port.sid_map = &sids;
	port.sid_map_sz = 2;
	port.started = true;
	for (unsigned int i = 0; i < 4; i++) {
		pd[i] = (struct device){ .index = i, .on = true };
		pds.pd_devs[i] = &pd[i];
	}
	cio.rtk = &rtk;
	cio.pd_list = &pds;
	cio.rc_base = rc;
	cio.current_cable_info = cio.target_cable_info = 1;
	rc[APPLE_CIO_M3_STAT / 4] = BIT(24);
	nhi.acio = &cio;
	nhi.cio_phy = &phy;
	nhi.cio_dart = &darts[0];
	nhi.tb = &tb;
	nhi.n_rings = 2;
	reg[PORT_APPCLK / 4] = PORT_APPCLK_EN;
	reg[PORT_PERST / 4] = PORT_PERST_OFF;
	reg[PORT_LINKSTS / 4] = PORT_LINKSTS_UP;
	reg[PORT_STATUS / 4] = PORT_STATUS_READY;
	reg[0x828 / 4] = 0x1234;
	reg[0x82c / 4] = 0x5678;
}
static void inject(int point)
{
	fault = point;
	pending_fault = true;
	polls = 0;
}
static void clean(void)
{
	assert(!phy.prepared && !pc.cio_prepared && !nhi.cio_prepared &&
	       !cio.native_sleep);
	assert(!darts[0].active && !darts[1].active && !tb.prepared &&
	       !tb.stopped);
	assert(!port.irq_disabled && !pc.cio_changed && pdev.dev.refs == 1);
	for (unsigned int i = 0; i < 4; i++)
		assert(pd[i].on && !balances[i]);
}
int main(void)
{
	unsigned int cases = 0;

	setup();
	for (int cycle = 0; cycle < 3; cycle++) {
		assert(!apple_nhi_cio_prepare(&nhi));
		assert(!apple_pcie_cio_suspend(&pc));
		assert(!apple_nhi_cio_suspend(&nhi));
		assert(nhi.cio_pd_suspended == 15 && phy.changed);
		assert(!apple_nhi_cio_resume(&nhi));
		assert(!apple_pcie_cio_resume(&pc));
		assert(rid[0] == 0x1234 && rid[1] == 0x5678 &&
		       resets == cycle + 1);
		apple_nhi_cio_complete(&nhi);
		clean();
		cases++;
	}
	for (int point = 1; point <= 6; point++) {
		/* 4 (no PCIe tunnel behind the router) is not a refusal: see below. */
		if (point == 4)
			continue;
		setup();
		inject(point);
		assert(apple_nhi_cio_prepare(&nhi) < 0);
		/* Every refusal after the CM's prepare rescans; none before it. */
		assert(tb.refusals == tb.prepares);
		clean();
		cases++;
	}
	/*
	 * T6050 without a PCIe tunnel refuses before the connection manager releases anything:
	 * a host peer's link and services stay as they are.
	 */
	setup();
	nhi.cio_t6050 = true;
	inject(4);
	assert(apple_nhi_cio_prepare(&nhi) == -ENODEV);
	assert(!tb.prepares && !tb.refusals && !phy.prepared && !darts[0].active);
	assert(pdev.dev.refs == 1 && !cio.native_sleep);
	nhi.cio_t6050 = false;
	clean();
	cases++;
	/*
	 * A partner with no PCIe tunnel (a host-to-host cable): the router sleeps and wakes
	 * with nothing behind it, and no PCIe callback runs.
	 */
	for (int cycle = 0; cycle < 2; cycle++) {
		setup();
		inject(4);
		assert(!apple_nhi_cio_prepare(&nhi) && !nhi.cio_pcie && pdev.dev.refs == 1);
		assert(!apple_nhi_cio_suspend(&nhi));
		assert(nhi.cio_pd_suspended == 15 && phy.changed);
		assert(!apple_nhi_cio_resume(&nhi));
		assert(!pc.cio_prepared && !resets && !darts[1].active);
		apple_nhi_cio_complete(&nhi);
		clean();
		cases++;
	}
	/* The same, refused after the prepare: complete gives back what prepare took. */
	setup();
	inject(4);
	assert(!apple_nhi_cio_prepare(&nhi) && !nhi.cio_pcie);
	apple_nhi_cio_complete(&nhi);
	clean();
	cases++;
	setup();
	assert(!apple_nhi_cio_prepare(&nhi));
	apple_nhi_cio_complete(&nhi);
	clean();
	cases++;
	for (int point = 11; point <= 16; point++) {
		setup();
		assert(!apple_nhi_cio_prepare(&nhi));
		inject(point);
		assert(apple_pcie_cio_suspend(&pc) < 0);
		assert(resets == (point >= 13));
		apple_nhi_cio_complete(&nhi);
		clean();
		cases++;
	}
	/*
	 * A DART that cannot be saved with the router asleep, or a domain that
	 * stays on, leaves no way to restart the controller.
	 */
	for (int point = 20; point <= 23; point += 3) {
		setup();
		assert(!apple_nhi_cio_prepare(&nhi));
		assert(!apple_pcie_cio_suspend(&pc));
		inject(point);
		expect_panic = true;
		if (!setjmp(fatal)) {
			apple_nhi_cio_suspend(&nhi);
			assert(false);
		}
		cases++;
	}
	for (int point = 19; point <= 25; point++) {
		if (point == 20 || point == 21 || point == 23)
			continue;
		setup();
		assert(!apple_nhi_cio_prepare(&nhi));
		assert(!apple_pcie_cio_suspend(&pc));
		inject(point);
		assert(apple_nhi_cio_suspend(&nhi) < 0);
		assert(!apple_pcie_cio_resume(&pc));
		apple_nhi_cio_complete(&nhi);
		clean();
		cases++;
	}
	/* A dock gone during the sleep: the port stays down, the DART comes back. */
	setup();
	assert(!apple_nhi_cio_prepare(&nhi));
	assert(!apple_pcie_cio_suspend(&pc));
	assert(!apple_nhi_cio_suspend(&nhi));
	tb.lost = true;
	assert(!apple_nhi_cio_resume(&nhi) && pc.cio_lost);
	assert(apple_pcie_cio_resume(&pc) == -ENOTCONN);
	assert(!darts[1].off && pc.cio_changed && !(reg[PORT_LINKSTS / 4] & PORT_LINKSTS_UP));
	/* A later teardown still has to stop the half-restarted port. */
	assert(port.needs_stop && !port.started);
	pc.resume_failed = true;
	apple_nhi_cio_complete(&nhi);
	assert(!pc.cio_prepared && !darts[0].active && !darts[1].active && !tb.prepared);
	cases++;
	/* A link that does not train, or a port DART that does not come back. */
	for (int point = 32; point <= 33; point++) {
		setup();
		assert(!apple_nhi_cio_prepare(&nhi));
		assert(!apple_pcie_cio_suspend(&pc));
		assert(!apple_nhi_cio_suspend(&nhi));
		assert(!apple_nhi_cio_resume(&nhi));
		inject(point);
		assert(apple_pcie_cio_resume(&pc) < 0);
		assert(darts[1].off == (point == 32) && port.needs_stop);
		pc.resume_failed = true;
		apple_nhi_cio_complete(&nhi);
		assert(!pc.cio_prepared && !darts[0].active && !darts[1].active);
		cases++;
	}
	for (int point = 30; point <= 35; point++) {
		if (point == 32 || point == 33)
			continue;
		setup();
		assert(!apple_nhi_cio_prepare(&nhi));
		assert(!apple_pcie_cio_suspend(&pc));
		assert(!apple_nhi_cio_suspend(&nhi));
		inject(point);
		expect_panic = true;
		if (!setjmp(fatal)) {
			apple_nhi_cio_resume(&nhi);
			assert(false);
		}
		cases++;
	}
	/*
	 * J714s's PCIe-C: a port DART that could not be saved refuses the sleep before anything
	 * of the port is touched (its link, its interrupts, its functions' state).
	 */
	setup();
	j714s = true;
	pc.hw = &t6050_pciec_hw;
	assert(!apple_nhi_cio_prepare(&nhi));
	u32 before[ARRAY_SIZE(reg)];

	memcpy(before, reg, sizeof(reg));
	dart_check_error = -EOPNOTSUPP;
	assert(apple_pcie_cio_suspend(&pc) == -EOPNOTSUPP && dart_checks == 1);
	assert(!memcmp(before, reg, sizeof(reg)) && !port.irq_disabled && !callbacks);
	assert(!pc.cio_changed && !pc.cio_stopped && !pc.cio_port_reset && !darts[1].off);
	apple_nhi_cio_complete(&nhi);
	clean();
	cases++;
	printf("PASS %u native CIO ownership cases\n", cases);
}
