/* SPDX-License-Identifier: GPL-2.0-only */
/* This fixture selects the original controller layout, not T8132. */
#define of_device_is_compatible(...) false
struct apple_atcphy {
    /* T8132-only paths are covered by drivers/apple/atc-t8132-usb3.py. */
    bool t8132_ss, t8132_common_owned, t8132_failed, cio_tunables, t8132_dp_alt;
	/* A CIO sleep is T8132's too, covered by tools/asahi/atc-cio-sleep-tests.py. */
	struct {
		bool prepared;
	} cio_sleep;
	struct {
		void *usb3_dp, *dp_lioa, *dp_common_config;
	} tunables;

	const struct atcphy_hw *hw;
	struct {
		unsigned char *core, *lpdptx, *lioa, *pipehandler, *usb2phy;
	} regs;
	struct reset_controller_dev rcdev;
	void *dev;
	int lock, dp_link_rate;
	bool swap_lanes, tunnel_clock_on, tunnel_saved, dp_only, pd_was_off;
	unsigned int tunnel_rate, tunnel_users;
	struct {
		bool aux, pll;
		unsigned int pairs, rate;
	} dp_t8122;
	bool fixed_usb2, host_active, mode_dirty, ss_tunables, dp_tunables,
		aux_power_requested;
	unsigned int dp_owned_pmas, dp_idle_pmas;
	unsigned int dp_usb_context_valid, dp_usb_context_dirty;
	u32 dp_usb_context[2][27];
	bool pipe_pending, usb_off_pending, usb_off_drained;
	bool usb3_port_off, park_owed;
	bool t8103_dummy_pending, t8103_usb3_retry, t8103_usb3_workaround;
	enum atcphy_mode mode, typec_mode;
	enum atcphy_pipehandler_state pipe_state;
};
struct phy;
struct phy_ops {
	int (*init)(struct phy *);
	int (*exit)(struct phy *);
	int (*power_on)(struct phy *);
	int (*power_off)(struct phy *);
	int (*reset)(struct phy *);
};
struct phy {
	struct apple_atcphy *data;
	const struct phy_ops *ops;
	int mutex, power_count, init_count, pm_refs, pwr, dev;
	bool pm_enabled;
};
#define phy_get_drvdata(p) ((p)->data)
static int phy_pm_runtime_get_sync(struct phy *p)
{
	if (!p->pm_enabled)
		return -ENOTSUPP;
	p->pm_refs++;
	return 0;
}
static void phy_pm_runtime_put(struct phy *p)
{
	if (p->pm_enabled)
		p->pm_refs--;
}
static void phy_pm_runtime_put_sync(struct phy *p)
{
	phy_pm_runtime_put(p);
}
static int regulator_enable(int p)
{
	assert(!p);
	return 0;
}
static void regulator_disable(int p)
{
	assert(!p);
}
struct dwc3 {
	struct phy *usb2_generic_phy[3], *usb3_generic_phy[3];
	int num_usb2_ports, num_usb3_ports, usb2_phy, usb3_phy;
	struct reset_controller_dev *reset;
	int dev, usb_psy, vbus_draw_work, dr_mode, current_dr_role;
	bool dis_u3_susphy_quirk;
	struct { void *notifier_call; } psy_nb;
};
struct dwc3_apple {
	struct dwc3 dwc;
	enum dwc3_apple_state state;
	struct reset_controller_dev *reset;
	int lock;
	void *dev;
	struct phy *usb2_phy, *usb3_phy;
	bool fixed_hub, core_active, core_exit_pending, reset_pending, shutting_down;
	/* Stopped for a deep sleep, and the state to bring back. */
	bool sleep_stopped;
	enum dwc3_apple_state sleep_state;
	/* Votes sleep DDR while the host is up; the fixtures leave it off. */
	bool sleep_ddr;
	bool rpm_held, host_only;
	/* T8132 only: runtime PM before the first core probe; never set here. */
	bool pre_core_rpm;
	/* J714s only: the start after a sleep waits for its Type-C PHY; never set here. */
	bool sleep_phy_wait, sleep_phy_pending;
	/* J714s only: a SuperSpeed device looked for after a sleep; never set here. */
	bool ss_retry;
	bool dt_dis_u3_susphy;
	unsigned long usb2_power_off_pending, usb3_power_off_pending;
	/* The PM sleep epoch (dwc3_apple_pm_notify()); the fixtures stay IDLE. */
	int sleep_pm_lock;
	enum dwc3_apple_sleep_phase sleep_phase;
	bool sleep_syscore, sleep_role_pending;
	enum dwc3_apple_state sleep_role;
	/*
	 * J714s only: a host kept suspended on its USB4 router's USB3 tunnel through the
	 * router's sleep, and the role its port controller asked for meanwhile.
	 */
	bool sleep_tunnel;
	enum usb_role sleep_tunnel_role;
};
/* The epoch spinlock: taken only when free, released only when held. */
#define spin_lock_irqsave(l, f) do { assert(!*(l)); *(l) = 1; (f) = 0; } while (0)
#define spin_unlock_irqrestore(l, f) do { assert(*(l)); *(l) = 0; (void)(f); } while (0)
static void __attribute__((unused)) apple_sleep_usb_slp_ddr(bool on)
{
	(void)on;
}
static unsigned int polls, fail_poll, reset_calls, reset_writes, core_writes;
static unsigned int host_exits, clocks_disabled, callback_calls;
static unsigned int gadget_exits, event_cleanups, legacy_suspends,
	core_mode_exits, free_buffers;
static unsigned int starts, deasserts, setup_cio, root_refs;
static int host_init_error, gadget_init_error, core_init_error, reset_error,
	other_off_error;
static bool init_cleanup_fault;
static bool syscore;
static bool stuck_ack;
static u32 core_regs[0x30000 / 4], pipe_regs[0x1000 / 4], usb2_regs[0x1000 / 4];
static u32 readl(void *p)
{
	return *(u32 *)p;
}
static void mask32(void *p, u32 m, u32 v)
{
	*(u32 *)p = (*(u32 *)p & ~m) | v;
	if (p == &pipe_regs[PIPEHANDLER_LOCK_REQ / 4] && !stuck_ack)
		pipe_regs[PIPEHANDLER_LOCK_ACK / 4] = *(u32 *)p &
						      PIPEHANDLER_LOCK_EN;
	if (p == &pipe_regs[PIPEHANDLER_AON_GEN / 4])
		reset_writes++;
	if ((uintptr_t)p >= (uintptr_t)core_regs &&
	    (uintptr_t)p < (uintptr_t)core_regs + sizeof(core_regs))
		core_writes++;
}
static void writel(u32 v, void *p)
{
	mask32(p, ~0U, v);
}
static void set32(void *p, u32 v)
{
	mask32(p, 0, v);
}
static void clear32(void *p, u32 v)
{
	mask32(p, v, 0);
}
static void core_mask32(struct apple_atcphy *p, u32 o, u32 m, u32 v)
{
	assert(o < sizeof(core_regs));
	mask32(p->regs.core + o, m, v);
	if (o == ATCPHY_POWER_CTRL)
		core_regs[ATCPHY_POWER_STAT / 4] = core_regs[o / 4];
}
static void core_set32(struct apple_atcphy *p, u32 o, u32 v)
{
	core_mask32(p, o, 0, v);
}
static void core_clear32(struct apple_atcphy *p, u32 o, u32 v)
{
	core_mask32(p, o, v, 0);
}
#define readl_poll_timeout(a, v, c, i, t)                    \
	({                                                   \
		(v) = readl(a);                              \
		++polls;                                     \
		polls == fail_poll || !(c) ? -ETIMEDOUT : 0; \
	})
#define readl_poll_timeout_atomic readl_poll_timeout
/* These dependencies are outside the cross-layer test; no DP ownership is seeded. */
static int j700_dp_stop(struct apple_atcphy *p)
{
	assert(!p->dp_owned_pmas);
	return 0;
}
static int j700_dp_idle_pending(struct apple_atcphy *p)
{
	(void)p;
	return 0;
}
static void atc_tunnel_stop_t8103(struct apple_atcphy *p)
{
	(void)p;
}
static void atc_tunnel_restore(struct apple_atcphy *p)
{
	(void)p;
}
/* The teardown fixture leaves electrical tunnel/main-link resources stopped. */
static int atc_t8122_tunnel_stop(struct apple_atcphy *p)
{
	assert(!p->tunnel_clock_on && !p->tunnel_users);
	return 0;
}

static void atcphy_dp_stop_t8122(struct apple_atcphy *p)
{
	assert(!p->dp_t8122.aux && !p->dp_t8122.pll);
}

static void atcphy_dp_aux_on_t8122(struct apple_atcphy *p)
{
	assert(!p->hw->dp_t8122);
}

static bool of_machine_is_compatible(const char *name)
{
	return !strcmp(name, "apple,t8140");
}
/* As in <linux/soc/apple/t8132.h>. */
static inline bool apple_t8132_air(void)
{
	return of_machine_is_compatible("apple,j713") ||
	       of_machine_is_compatible("apple,j715");
}
static inline bool apple_t8132_pro_mini_imac(void)
{
	return of_machine_is_compatible("apple,j604") ||
	       of_machine_is_compatible("apple,j623") ||
	       of_machine_is_compatible("apple,j624") ||
	       of_machine_is_compatible("apple,j773g");
}
static inline bool apple_t8132_usb_host_first(void)
{
	return apple_t8132_air() || apple_t8132_pro_mini_imac();
}

#define READ_ONCE(x) (x)
#define WRITE_ONCE(p, v) ((p) = (v))
static void _atcphy_dwc3_reset_assert(struct apple_atcphy *p);
static void atcphy_usb2_power_off(struct apple_atcphy *p);
static void atcphy_restore_after_pd_off(struct apple_atcphy *p);
static int atcphy_power_off(struct apple_atcphy *p);
static int atcphy_configure(struct apple_atcphy *p, enum atcphy_mode mode);

/* These units exercise older PHY generations. Never accept a T8132 dispatch. */
static int __attribute__((unused)) atcphy_configure_t8132(struct apple_atcphy *p, enum atcphy_mode mode)
{ (void)p; (void)mode; assert(false); return -EOPNOTSUPP; }
static int __attribute__((unused)) atcphy_configure_pipehandler_usb3_host_t8132(struct apple_atcphy *p)
{ (void)p; assert(false); return -EOPNOTSUPP; }


/* Model the USB2 power-on boundary's running state. */
static unsigned int usb2_starts;
static void atcphy_usb2_power_on(struct apple_atcphy *p)
{
	usb2_starts++;
	writel(USB2PHY_USBCTL_RUN, p->regs.usb2phy + USB2PHY_USBCTL);
}

/* This teardown fixture never selects T8142 DP calibration. */
static void apple_tunable_apply(void *base, void *tunable)
{
	(void)base;
	(void)tunable;
	assert(!"unexpected banked DP calibration");
}

static void atcphy_apply_tunables(struct apple_atcphy *p, enum atcphy_mode m)
{
	(void)p;
	(void)m;
}
static int atcphy_configure_lanes(struct apple_atcphy *p, enum atcphy_mode m)
{
	(void)p;
	(void)m;
	return 0;
}
static int atcphy_enable_dp_aux(struct apple_atcphy *p)
{
	(void)p;
	return 0;
}
static int atcphy_auspll_apb_command(struct apple_atcphy *p, int command)
{
	(void)p;
	(void)command;
	return 0;
}
static int
atcphy_dp_configure_lane(struct apple_atcphy *p, int lane,
			 const struct atcphy_dp_link_rate_configuration *config)
{
	(void)p;
	(void)lane;
	(void)config;
	return 0;
}
static void usb_phy_set_suspend(int p, int s)
{
	(void)p;
	(void)s;
	legacy_suspends++;
}
static void usb_phy_shutdown(int p)
{
	(void)p;
}
static void dwc3_event_buffers_cleanup(struct dwc3 *d)
{
	(void)d;
	event_cleanups++;
}
static void dwc3_clk_disable(struct dwc3 *d)
{
	(void)d;
	clocks_disabled++;
}
static void dwc3_host_exit(struct dwc3 *d);
static void dwc3_gadget_exit(struct dwc3 *d)
{
	(void)d;
	gadget_exits++;
}
static void dev_pm_syscore_device(void *d, bool s)
{
	(void)d;
	syscore = s;
}
static void dwc3_enable_susphy(struct dwc3 *d, bool s)
{
	(void)d;
	(void)s;
}
static int atcphy_dwc3_reset_assert(struct reset_controller_dev *,
				    unsigned long);
static int reset_control_assert(struct reset_controller_dev *r)
{
	if (!r)
		return 0;
	reset_calls++;
	if (reset_error)
		return reset_error;
	return atcphy_dwc3_reset_assert(r, 0);
}
static int atcphy_dwc3_reset_deassert(struct reset_controller_dev *,
				      unsigned long);
static int reset_control_deassert(struct reset_controller_dev *r)
{
	deasserts++;
	return atcphy_dwc3_reset_deassert(r, 0);
}
static int phy_set_mode(struct phy *p, int m)
{
	(void)p;
	(void)m;
	return 0;
}
static int dwc3_apple_core_probe(struct dwc3_apple *a, enum dwc3_apple_state s);
static int dwc3_core_init(struct dwc3 *d);
static void dwc3_apple_setup_cio(struct dwc3_apple *a)
{
	(void)a;
	setup_cio++;
}
/* The check after a sleep is only for a fixed hub or a J714s SuperSpeed device; neither here. */
static void dwc3_apple_check_hub_later(struct dwc3_apple *a)
{
	assert(!a->fixed_hub && !a->ss_retry);
}
/* The hibernation block's register setup; teardown order doesn't depend on it. */
static void __attribute__((unused)) dwc3_apple_setup_hibernation(struct dwc3_apple *a)
{
	(void)a;
}
static int dwc3_apple_usb2_mode_before_reset(enum dwc3_apple_state s,
					     enum phy_mode *m)
{
	if (s != DWC3_APPLE_HOST && s != DWC3_APPLE_DEVICE)
		return -EINVAL;
	*m = s == DWC3_APPLE_HOST ? PHY_MODE_INVALID : PHY_MODE_USB_DEVICE;
	return 0;
}
static int dwc3_apple_set_role(struct dwc3_apple *a, enum dwc3_apple_state s);
static int dwc3_host_init(struct dwc3 *d)
{
	(void)d;
	if (init_cleanup_fault) {
		stuck_ack = true;
		pipe_regs[PIPEHANDLER_LOCK_ACK / 4] = PIPEHANDLER_LOCK_EN;
	}
	return host_init_error;
}
static int dwc3_gadget_init(struct dwc3 *d)
{
	(void)d;
	return gadget_init_error;
}
#define pm_runtime_get_sync(...) ((void)0)
#define pm_runtime_allow(...) ((void)0)
#define pm_runtime_disable(...) ((void)0)
#define pm_runtime_dont_use_autosuspend(...) ((void)0)
#define pm_runtime_put_noidle(...) ((void)0)
#define pm_runtime_set_suspended(...) ((void)0)
#define dev_set_drvdata(...) ((void)0)
#define cancel_work_sync(...) ((void)0)
#define power_supply_put(...) ((void)0)
#define power_supply_unreg_notifier(...) ((void)0)
static void dwc3_core_exit_mode(struct dwc3 *d)
{
	(void)d;
	core_mode_exits++;
}
static void dwc3_debugfs_exit(struct dwc3 *d)
{
	(void)d;
}
static void dwc3_ulpi_exit(struct dwc3 *d)
{
	(void)d;
}
static void dwc3_free_event_buffers(struct dwc3 *d)
{
	(void)d;
	free_buffers++;
}

/* J713 host-before-init is exercised by the dedicated initial-role test. */
enum { USB_DR_MODE_HOST = 1, DWC3_GCTL_PRTCAP_HOST = 1 };
static void dwc3_apple_set_ptrcap(struct dwc3_apple *a, u32 mode)
{ (void)a; (void)mode; assert(!"J713 pre-init path entered a legacy fixture"); }

/* Runtime-PM ownership is observable even though this fixture has no device PM core. */
static int pm_runtime_resume_and_get(void *dev)
{
	(void)dev;
	return 0;
}
static void pm_runtime_mark_last_busy(void *dev)
{
	(void)dev;
}
static void pm_runtime_put_autosuspend(void *dev)
{
	(void)dev;
}
/* The legacy profile never has T8132 tunables. */
static void dwc3_apple_apply_tunables(struct dwc3_apple *appledwc)
{
	assert(!appledwc->host_only);
}
#define dev_info_ratelimited(...) ((void)0)
