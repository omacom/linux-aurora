/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* One Type-C owner reuses the established DP and USB backend sequences.
 * All calls below hold dp->lock; only PHY power_off authorizes PIPE teardown
 * while the controller reset is deasserted. Lane changes require reset/off.
 */
static u32 combo_read(void *ctx, unsigned int bank, unsigned int offset)
{
	struct t6050_dp *dp = ctx;
	return readl(dp->banks[bank] + offset);
}
static void combo_write(void *ctx, unsigned int bank, unsigned int offset, u32 value)
{
	struct t6050_dp *dp = ctx;
	writel(value, dp->banks[bank] + offset);
}
static int combo_poll(void *ctx, unsigned int bank, unsigned int offset,
		      unsigned int mask, unsigned int expected, unsigned int usec)
{
	struct t6050_dp *dp = ctx;
	u32 value;
	int ret = readl_poll_timeout(dp->banks[bank] + offset, value,
		(value & mask) == expected,
		(bank == 44 || bank == 35 || usec <= 1000) ? 1 : 1000, usec);

	/* The PIPE switch carries on without the lock ACK; say so. */
	if (ret && bank == 44 && offset == 0x14 && usec == 6000)
		dev_warn(dp->dev,
			 "PIPE %slock not acknowledged; continuing (override=%x values=%x mux=%x req=%x ack=%x aon=%x nonsel=%x)\n",
			 expected ? "" : "un", readl(dp->banks[44]),
			 readl(dp->banks[44] + 4), readl(dp->banks[44] + 0xc),
			 readl(dp->banks[44] + 0x10), value,
			 readl(dp->banks[44] + 0x1c), readl(dp->banks[44] + 0x20));
	return ret;
}
static void combo_delay(void *ctx, unsigned int usec) { udelay(usec); }
static void combo_pause(void *ctx, unsigned int nsec) { ndelay(nsec); }
static void combo_sleep(void *ctx, unsigned int msec) { msleep(msec); }
static int combo_wrapper(void *ctx, unsigned int active)
{
	struct t6050_dp *dp = ctx;

	/* A router's sleep in place: no runtime PM there, the wrapper's reference stays. */
	if (dp->cio_sleep_hold)
		return 0;
	return atc_domain_set(&dp->domain[2], active);
}
static int combo_firmware(void *ctx, unsigned int active)
{
	return atc_firmware(ctx, active);
}
static int combo_tunable(void *ctx, unsigned int bank, const char *name)
{
	static const struct { u8 bank, index; const char *name; } common[] = {
		{41, 0, "tunable_ATC0AXI2AF"}, {40, 1, "tunable_ATC_FABRIC"},
		{43, 2, "tunable_ATC_COMMON_CFG"}, {42, 3, "tunable_ATC0AXI2AF_LIOA"},
		{20, 4, "tunable_LN0_RX_CFG_TX_OF_RXCLK"}, {29, 5, "tunable_LN1_RX_CFG_TX_OF_RXCLK"},
		{18, 6, "tunable_LN0_RX_EQ_DFLT"}, {27, 7, "tunable_LN1_RX_EQ_DFLT"},
		{8, 8, "tunable_AUSPLL_CORE"}, {5, 9, "tunable_AUS40CMN_SHM"},
		{10, 10, "tunable_CIO4PLL_CORE"},
	};
	static const struct { u8 bank, index; const char *name; } cio[] = {
		{3, 0, "tunable_UC_REGS_CIO_DFLT"}, {16, 1, "tunable_LN0_RX_TOP_CIO_DFLT"},
		{25, 2, "tunable_LN1_RX_TOP_CIO_DFLT"}, {18, 3, "tunable_LN0_RX_EQ_CIO_DFLT"},
		{27, 4, "tunable_LN1_RX_EQ_CIO_DFLT"},
	};
	struct t6050_dp *dp = ctx;

	if (bank == 0 && !strcmp(name, "tunable_USB2PHY_DFLT")) {
		apple_tunable_apply(dp->banks[0], dp->usb2_default);
		return 0;
	}
	if (bank == 0 && !strcmp(name, "tunable_USB2PHY_HOST")) {
		apple_tunable_apply(dp->banks[0], dp->usb2_host);
		return 0;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(common); i++)
		if (bank == common[i].bank && !strcmp(name, common[i].name)) {
			apple_tunable_apply(dp->banks[bank], dp->tunables[common[i].index]);
			return 0;
		}
	/* A missing CIO80 tunable is skipped. */
	for (unsigned int i = 0; i < ARRAY_SIZE(cio); i++)
		if (bank == cio[i].bank && !strcmp(name, cio[i].name)) {
			apple_tunable_apply(dp->banks[bank], dp->cio_tunables[cio[i].index]);
			return 0;
		}
	return -EINVAL;
}

static int combo_quiesce(void *ctx)
{
	struct t6050_dp *dp = ctx;
	int ret = 0;

	if (dp->usb_prepared) {
		ret = APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_usb3_shutdown);
		if (!ret)
			dp->usb_prepared = false;
	} else if (dp->cio_prepared) {
		ret = APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_cio_shutdown);
		if (!ret)
			dp->cio_prepared = false;
	} else if (dp->enabled) {
		/* Multifunction has a distinct native shutdown; never run the
		 * USB-only analog sequence over live DisplayPort lane state. */
		ret = atc_stop(dp);
	}
	return ret;
}

/* The pipehandler bank is the controller's: read it only while the controller holds its domain. */
static void combo_pipe_fault(struct t6050_dp *dp, const char *what, int ret)
{
	dev_err(dp->dev, "%s failed (%d): pipe 00=%08x 04=%08x 0c=%08x 10=%08x 14=%08x 1c=%08x 20=%08x\n",
		what, ret, readl(dp->banks[44]), readl(dp->banks[44] + 0x4),
		readl(dp->banks[44] + 0xc), readl(dp->banks[44] + 0x10),
		readl(dp->banks[44] + 0x14), readl(dp->banks[44] + 0x1c),
		readl(dp->banks[44] + 0x20));
	dp->faulted = true;
}

/*
 * What the combo PHY gained for a deep sleep outside its sleep callbacks (a wrapper domain
 * still held at a stop is released, a safe state with a domain still held is stopped again,
 * the always-on domain's hold over the sleep) is in force on the boards with a T6050 sleep,
 * the J714s and the J716c. On another T6050 board the stop, the safe state and the probe are
 * as before.
 */
static bool combo_sleep_board(void)
{
	return of_machine_is_compatible("apple,j714s") || of_machine_is_compatible("apple,j716c");
}

static bool combo_domains_held(struct t6050_dp *dp)
{
	for (unsigned int i = 0; i < dp->ndomains; i++)
		if (dp->domain[i].active)
			return true;
	return false;
}

/* @keep_aon leaves the always-on domain's reference, for a sleep or for the start that follows. */
static int combo_stop(struct t6050_dp *dp, bool keep_aon)
{
	int ret;

	lockdep_assert_held(&dp->lock);
	if (dp->faulted)
		return -EIO;
	/* No tunnel clock outlives USB4 mode. */
	atc_dp_tunnels_stop(dp);
	if (dp->controller_active || dp->pipe_active)
		return -EBUSY;
	ret = combo_quiesce(dp);
	if (!ret && dp->usb2_active) {
		ret = apple_t6050_usb2_stop(&dp->usb_platform);
		if (!ret)
			dp->usb2_active = false;
	}
	/* The mode's own shutdown drops the wrapper; it is still held only if that did not run. */
	if (!ret && combo_sleep_board())
		ret = atc_domain_set(&dp->domain[2], false);
	if (!ret && dp->ndomains > 3)
		ret = atc_domain_set(&dp->domain[3], false);
	if (!ret)
		ret = atc_domain_set(&dp->domain[1], false);
	if (!ret && !keep_aon)
		ret = atc_domain_set(&dp->domain[0], false);
	if (ret) {
		dev_err(dp->dev, "stop failed (%d)\n", ret);
		dp->faulted = true;
		return ret < 0 ? ret : -EIO;
	}
	dp->aon_kept = keep_aon && dp->domain[0].active;
	dp->combo_mode = COMBO_SAFE;
	return 0;
}

static int combo_start(struct t6050_dp *dp, enum combo_mode mode)
{
	int ret;

	lockdep_assert_held(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (dp->orientation == TYPEC_ORIENTATION_NONE)
		return -ENOLINK;
	/* A new start of the port's mode: what a router lost over a sleep is not carried on. */
	dp->cio_tunnel_lost = false;
	ret = atc_domain_set(&dp->domain[0], true);
	if (!ret)
		ret = atc_domain_set(&dp->domain[1], true);
	/*
	 * The USB2 PHY and digital banks are unclocked without the USB domain,
	 * which a USB controller holds only while one is bound to this port.
	 */
	if (!ret && dp->ndomains > 3)
		ret = atc_domain_set(&dp->domain[3], true);
	dp->aon_kept = false;
	if (!ret)
		ret = APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_usb2_host);
	if (!ret)
		dp->usb2_active = true;
	if (!ret && mode == COMBO_USB) {
		ret = dp->reverse ?
			APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_usb3_init_reverse) :
			APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_usb3_init_normal);
		if (!ret)
			dp->usb_prepared = true;
	} else if (!ret && mode == COMBO_CIO) {
		ktime_t start = ktime_get();

		/* USB2 stays native beside a USB4/TBT link; USB3 comes tunnelled. */
		ret = dp->reverse ?
			APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_cio_init_reverse) :
			APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_cio_init_normal);
		if (!ret) {
			dp->cio_prepared = true;
			atc_dp_tunnels_park(dp);
		}
		dev_info(dp->dev, "CIO mode %s (%d), %s orientation, %lld ms\n",
			 ret ? "failed" : "up: UC_TNBL_DONE", ret,
			 dp->reverse ? "reverse" : "normal",
			 ktime_ms_delta(ktime_get(), start));
	} else if (!ret) {
		dp->multifunction = mode == COMBO_DP2;
		ret = atc_start(dp);
	}
	if (ret) {
		/* A partial shared transition has no cold-start ownership proof. */
		dev_err(dp->dev, "start of mode %d failed (%d)\n", mode, ret);
		dp->faulted = true;
		return ret < 0 ? ret : -EIO;
	}
	dp->combo_mode = mode;
	return 0;
}

static int combo_orientation(struct t6050_dp *dp, enum typec_orientation orientation)
{
	int ret;

	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (orientation != TYPEC_ORIENTATION_NONE && orientation != TYPEC_ORIENTATION_NORMAL &&
	    orientation != TYPEC_ORIENTATION_REVERSE)
		return -EINVAL;
	if (dp->orientation == orientation)
		return 0;
	ret = combo_stop(dp, false);
	if (!ret) {
		dp->orientation = orientation;
		dp->reverse = orientation == TYPEC_ORIENTATION_REVERSE;
	}
	return ret;
}

/*
 * DisplayPort pin D comes to a port that runs USB 3: the USB 3 start (apple_t6050_usb3_init_*)
 * is the pin D start (mf_*_init) step for step, so the lanes are already as pin D has them,
 * USB 3 on one pair and the other pair free. Only the AUX channel is turned on, as macOS does
 * there on the running PHY once the port controller reports pin D; the PHY's firmware, the
 * PIPE and the USB controller run on. The port is then as a pin D start leaves it, so its
 * stop is pin D's (atc_stop()).
 */
static int combo_dp2_join_usb(struct t6050_dp *dp)
{
	int ret;

	lockdep_assert_held(&dp->lock);
	ret = ATC_SEQUENCE(dp, aux_on);
	if (ret) {
		dev_err(dp->dev, "AUX on beside the running USB 3 failed (%d)\n", ret);
		return ret;
	}
	dp->usb_prepared = false;
	dp->multifunction = true;
	dp->enabled = true;
	dp->combo_mode = COMBO_DP2;
	dev_info(dp->dev, "Type-C DP: 2 lanes beside the running USB 3, %s orientation\n",
		 dp->reverse ? "reverse" : "normal");
	return 0;
}

static int combo_mux(struct t6050_dp *dp, struct typec_mux_state *state)
{
	enum combo_mode mode;
	int ret;

	if (state->mode == TYPEC_STATE_SAFE)
		mode = COMBO_SAFE;
	else if (state->mode == TYPEC_STATE_USB && !state->alt)
		mode = COMBO_USB;
	else if (!state->alt && state->mode == TYPEC_MODE_USB4)
		mode = COMBO_CIO;
	else if (state->alt && state->alt->svid == USB_TYPEC_TBT_SID &&
		 state->mode == TYPEC_TBT_MODE)
		mode = COMBO_CIO;
	else if (state->alt && state->alt->svid == USB_TYPEC_DP_SID) {
		switch (state->mode) {
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E: mode = COMBO_DP4; break;
		case TYPEC_DP_STATE_D: mode = COMBO_DP2; break;
		default: return -EOPNOTSUPP;
		}
	} else {
		return -EOPNOTSUPP;
	}
	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	/* SAFE with a domain still held: a sleep kept it, and nothing came back on the port. */
	if (mode == dp->combo_mode &&
	    (mode == COMBO_SAFE ? !(combo_sleep_board() && combo_domains_held(dp)) :
	     dp->usb_prepared || dp->cio_prepared || dp->enabled))
		return 0;
	if (mode != COMBO_SAFE && dp->orientation == TYPEC_ORIENTATION_NONE)
		return -ENOLINK;
	if (mode == COMBO_DP2 && dp->combo_mode == COMBO_USB && dp->usb_prepared)
		return combo_dp2_join_usb(dp);
	/* A start right after a sleep finds the always-on side as the sleep kept it. */
	ret = combo_stop(dp, mode != COMBO_SAFE && dp->aon_kept);
	if (ret || mode == COMBO_SAFE)
		return ret;
	return combo_start(dp, mode);
}

/*
 * Before a firmware sleep, once every user of the port has suspended: the mode's shutdown with
 * the idle form of the PHY CPU's stop, then the wrapper, USB and common domains. The always-on
 * domain stays up for a port in use. A stop that cannot run
 * leaves the PHY as it is, and the wake forgets it.
 */
static int combo_suspend_late(struct t6050_dp *dp)
{
	bool keep;
	int ret;

	guard(mutex)(&dp->lock);
	dp->sleep_stopped = false;
	dp->sleep_mode = dp->combo_mode;
	if (dp->faulted) {
		dev_warn(dp->dev, "faulted before the sleep: left as it is\n");
		return 0;
	}
	if (dp->controller_active || dp->pipe_active) {
		dev_warn(dp->dev, "USB controller still up at the sleep: PHY left as it is\n");
		return 0;
	}
	keep = dp->aon_syscore && dp->combo_mode != COMBO_SAFE && dp->domain[0].active;
	ret = combo_stop(dp, keep);
	if (ret) {
		dev_warn(dp->dev, "not stopped for the sleep (%d)\n", ret);
		return 0;
	}
	dp->sleep_stopped = true;
	if (dp->sleep_mode != COMBO_SAFE)
		dev_info(dp->dev, "mode %d stopped for the sleep, always-on domain %s\n",
			 dp->sleep_mode, dp->aon_kept ? "kept" : "released");
	return 0;
}

/*
 * After a firmware sleep the PHY core and its CPU were powered off: nothing cached describes
 * the hardware any more, and no register is touched here. The domain references stay as they
 * are; the next mode request releases them and starts from cold.
 */
static void combo_resume_noirq(struct t6050_dp *dp)
{
	guard(mutex)(&dp->lock);
	dp->usb_prepared = dp->cio_prepared = dp->enabled = false;
	dp->usb2_active = dp->pipe_active = dp->pipe_tunnel = dp->controller_active = false;
	/* A CPU that was still running lost its power with the rest: its next start is a cold one. */
	if (dp->fw_running) {
		dp->fw_running = false;
		dp->fw_cold = true;
	}
	dp->rate = -1;
	memset(dp->voltage, 0, sizeof(dp->voltage));
	memset(dp->pre, 0, sizeof(dp->pre));
	dp->combo_mode = COMBO_SAFE;
	dp->faulted = false;
}

/* A sleep that was stopped for and did not happen: back to the mode the stop took down. */
static void combo_resume(struct t6050_dp *dp)
{
	int ret;

	guard(mutex)(&dp->lock);
	if (!dp->sleep_stopped)
		return;
	dp->sleep_stopped = false;
	if (pm_resume_via_firmware() || dp->faulted ||
	    dp->sleep_mode == COMBO_SAFE || dp->combo_mode != COMBO_SAFE)
		return;
	ret = combo_start(dp, dp->sleep_mode);
	if (ret)
		dev_warn(dp->dev, "mode %d not back after the sleep that did not happen (%d)\n",
			 dp->sleep_mode, ret);
	else
		dev_info(dp->dev, "mode %d back: the sleep did not happen\n", dp->sleep_mode);
}

/*
 * A port in use keeps its always-on domain through a deep sleep: held out of the system
 * transition from before its prepare to after its complete, so the power domain is not
 * counted as suspended, and asked of the sleep driver, which lowers the word otherwise.
 */
static int combo_pm_notify(struct notifier_block *nb, unsigned long action, void *unused)
{
	struct t6050_dp *dp = container_of(nb, struct t6050_dp, pm_nb);

	guard(mutex)(&dp->lock);
	switch (action) {
	case PM_SUSPEND_PREPARE:
		if (pm_suspend_default_s2idle() || dp->faulted || dp->aon_syscore ||
		    dp->combo_mode == COMBO_SAFE || !dp->domain[0].active)
			break;
		if (!apple_sleep_usb_aon_keep(dp->port, true))
			break;
		dev_pm_syscore_device(dp->domain[0].dev, true);
		dp->aon_syscore = true;
		break;
	case PM_POST_SUSPEND:
		apple_sleep_usb_aon_keep(dp->port, false);
		if (dp->aon_syscore) {
			dev_pm_syscore_device(dp->domain[0].dev, false);
			dp->aon_syscore = false;
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static void combo_pm_notifier_remove(void *data)
{
	struct t6050_dp *dp = data;

	unregister_pm_notifier(&dp->pm_nb);
}

/*
 * USB2 as a mode's start brings it up (combo_start()), for a port whose mode is up. A USB
 * controller's power-off stops USB2 with its PIPE (combo_usb3_poweroff()); while the mode
 * runs on, no mode request comes that would start it again. Without a mode, -ENOLINK: the
 * port controller's next mode request starts USB2 with it.
 */
static int combo_usb2_start(struct t6050_dp *dp)
{
	int ret;

	lockdep_assert_held(&dp->lock);
	if (!dp->usb_prepared && !dp->cio_prepared && !dp->enabled)
		return -ENOLINK;
	if (dp->usb2_active)
		return 0;
	ret = APPLE_T6050_USB_RUN(&dp->usb_platform, apple_t6050_usb2_host);
	if (ret) {
		dev_err(dp->dev, "USB2 host start failed (%d)\n", ret);
		dp->faulted = true;
	} else {
		dp->usb2_active = true;
	}
	return ret;
}

static int combo_usb2_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);

	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (mode != PHY_MODE_USB_HOST)
		return -EOPNOTSUPP;
	return combo_usb2_start(dp);
}

static int combo_usb2_reset(struct phy *phy)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&dp->lock);
	if (dp->faulted || !dp->usb2_active)
		return -ENOLINK;
	ret = apple_t6050_usb2_port_reset(&dp->usb_io, dp, combo_sleep);
	if (ret) {
		dev_err(dp->dev, "USB2 port reset failed (%d)\n", ret);
		dp->faulted = true;
	}
	return ret ? -EIO : 0;
}

/* A tunnelled PIPE leaves the CIO PHY to the router: nothing to quiesce on the PHY side. */
static int combo_pipe_keep(void *ctx)
{
	return 0;
}

/*
 * In CIO mode the lanes carry USB4/TBT and USB3 reaches DWC3 through the router's USB3 adapter:
 * move DWC3 from its dummy PIPE producer to the tunnel producer, as is done on the older PHYs.
 */
static int combo_pipe_tunnel(struct t6050_dp *dp)
{
	int ret;

	lockdep_assert_held(&dp->lock);
	if (dp->pipe_active)
		return dp->pipe_tunnel ? 0 : -EBUSY;
	ret = apple_t6050_usb3_pipe_tunnel(&dp->usb_io);
	if (ret) {
		combo_pipe_fault(dp, "USB3 tunnel PIPE switch", ret);
		return -EIO;
	}
	dp->pipe_active = true;
	dp->pipe_tunnel = true;
	dev_info(dp->dev, "USB3 PIPE on the USB4 tunnel\n");
	return 0;
}

static int combo_usb3_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	if (mode != PHY_MODE_USB_HOST)
		return -EOPNOTSUPP;
	if (!dp->controller_active || !dp->usb2_active)
		return -ENOLINK;
	/* Four DP lanes leave DWC3 on its established dummy PIPE; USB2 still
	 * uses its independent D+/D- PHY. Do not switch USB3 onto DP lanes. */
	if (dp->combo_mode == COMBO_DP4 && dp->enabled)
		return 0;
	/*
	 * TEST: the router woke from a sleep in place without its dock, and its USB3 tunnel was
	 * retired. A DWC3 moved to the tunnel's PIPE producer then takes its clock from a router
	 * that the port controller stops next with the host still running (run sv6t4: this line's
	 * "USB3 PIPE on the USB4 tunnel" at the resume, the router's domains off 3.1 s later with
	 * the host up, an SError in that power-off). DWC3 stays on its dummy producer, as with
	 * four DP lanes; USB2 is not touched. The port's next start gives the tunnel back.
	 */
	if (dp->combo_mode == COMBO_CIO && dp->cio_prepared && dp->cio_tunnel_lost) {
		dev_info(dp->dev, "USB3 PIPE left on its dummy producer: the router's dock did not come back from the sleep\n");
		return 0;
	}
	if (dp->combo_mode == COMBO_CIO && dp->cio_prepared)
		return combo_pipe_tunnel(dp);
	if (!dp->usb_prepared && !(dp->combo_mode == COMBO_DP2 && dp->enabled))
		return -ENOLINK;
	if (dp->pipe_active)
		return (readl(dp->banks[44] + 0xc) & 0x73) == 0x10 ? 0 : -EIO;
	apple_tunable_apply(dp->banks[44], dp->pipe_default);
	ret = apple_t6050_usb3_pipe_switch(&dp->usb_io);
	if (ret >= -6 && ret < 0 && (readl(dp->banks[44] + 0xc) & 0x73) == 0x42) {
		/* The PIPE never left its dummy producer: the port stays USB2, as with four DP lanes. */
		dev_warn(dp->dev, "USB3 PIPE switch failed (%d), the port runs USB2 only\n", ret);
		return 0;
	}
	if (ret)
		combo_pipe_fault(dp, "USB3 PIPE switch", ret);
	else
		dp->pipe_active = true;
	return ret ? -EIO : 0;
}

static int combo_usb3_poweroff(struct phy *phy)
{
	struct t6050_dp *dp = phy_get_drvdata(phy);
	int ret = 0;

	guard(mutex)(&dp->lock);
	if (dp->faulted)
		return -EIO;
	/* DWC3 calls power_off only after host/gadget shutdown and SUSPHY.
	 * PIPE must select its dummy producer before the controller is reset. */
	if (dp->pipe_active) {
		ret = apple_t6050_usb3_pipe_stop(&dp->usb_io,
						 dp->pipe_tunnel ? combo_pipe_keep : combo_quiesce, dp);
		if (!ret)
			dp->pipe_tunnel = false;
		if (!ret)
			dp->pipe_active = false;
	}
	if (!ret && dp->usb2_active) {
		ret = apple_t6050_usb2_stop(&dp->usb_platform);
		if (!ret)
			dp->usb2_active = false;
	}
	if (ret) {
		dev_err(dp->dev, "USB3 power-off failed (%d)\n", ret);
		dp->faulted = true;
	}
	return ret ? -EIO : 0;
}
static const struct phy_ops combo_usb2_ops = {
	.set_mode = combo_usb2_mode, .reset = combo_usb2_reset, .owner = THIS_MODULE,
};
static const struct phy_ops combo_usb3_ops = {
	.set_mode = combo_usb3_mode, .power_off = combo_usb3_poweroff, .owner = THIS_MODULE,
};

static int combo_reset_assert(struct reset_controller_dev *reset, unsigned long id)
{
	struct t6050_dp *dp = container_of(reset, struct t6050_dp, reset);
	int ret;

	guard(mutex)(&dp->lock);
	if (id || dp->pipe_active)
		return -EBUSY;
	if (dp->faulted)
		return -EIO;
	apple_t6050_usb_mask(&dp->usb_io, 44, 0x1c, 1, 0);
	apple_t6050_usb_mask(&dp->usb_io, 44, 0x1c, 0x10, 0x10);
	ret = apple_t6050_usb3_pipe_cold_dummy(&dp->usb_io);
	if (ret)
		combo_pipe_fault(dp, "dummy PIPE park", ret);
	else
		dp->controller_active = false;
	return ret ? -EIO : 0;
}
static int combo_reset_deassert(struct reset_controller_dev *reset, unsigned long id)
{
	struct t6050_dp *dp = container_of(reset, struct t6050_dp, reset);
	int ret;

	guard(mutex)(&dp->lock);
	if (id)
		return -EINVAL;
	if (dp->faulted)
		return -ENOLINK;
	/*
	 * A controller that starts again while the port's mode still runs finds USB2 stopped by
	 * its own power-off, and no mode request follows to start it: a system sleep given up
	 * after the controllers stopped at its entry but before the PHY's own stop at its last
	 * step (combo_suspend_late()), which therefore has nothing to bring back, or a role
	 * change on the same plug. Start it here, before the reset goes, as the mode's start
	 * does. After a real sleep no mode is up yet (combo_resume_noirq()): still -ENOLINK,
	 * and the mode request starts USB2 as before.
	 */
	ret = combo_usb2_start(dp);
	if (ret)
		return ret == -ENOLINK ? ret : -EIO;
	/*
	 * The PIPE lock that starts the USB3 handoff is acknowledged only while the dummy PHY
	 * behind the mux is enabled (NONSELECTED_OVERRIDE bit 15), as on the earlier ATC PHYs.
	 * The bit resets to clear with the controller's domain, so set it for every session.
	 */
	apple_t6050_usb_mask(&dp->usb_io, 44, 0x20, BIT(15), BIT(15));
	apple_t6050_usb_mask(&dp->usb_io, 44, 0x1c, 0x10, 0);
	apple_t6050_usb_mask(&dp->usb_io, 44, 0x1c, 1, 1);
	dp->controller_active = true;
	return 0;
}
static const struct reset_control_ops combo_reset_ops = {
	.assert = combo_reset_assert, .deassert = combo_reset_deassert,
};

static struct phy *combo_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct t6050_dp *dp = dev_get_drvdata(dev);
	if (args->args_count != 1)
		return ERR_PTR(-EINVAL);
	switch (args->args[0]) {
	case PHY_TYPE_USB2: return dp->combo_phy[0];
	case PHY_TYPE_USB3: return dp->combo_phy[1];
	case PHY_TYPE_DP: return dp->combo_phy[2];
	default: return ERR_PTR(-EINVAL);
	}
}

static int combo_setup(struct platform_device *pdev, struct t6050_dp *dp)
{
	/* Same order as the cio[] table in combo_tunable(). */
	static const struct { u8 bank; const char *name; } cio_tunables[] = {
		{3, "apple,tunable-uc-regs-cio"}, {16, "apple,tunable-lane0-rx-top-cio"},
		{25, "apple,tunable-lane1-rx-top-cio"}, {18, "apple,tunable-lane0-rx-eq-cio"},
		{27, "apple,tunable-lane1-rx-eq-cio"},
	};
	static const struct { u8 bank; u32 minimum; const char *name; } extra[] = {
		{0, 0x20, "usb2phy"}, {1, 0x24, "usb2-digital"}, {44, 0x48, "pipehandler"},
	};
	struct device *dev = dp->dev;
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(extra); i++) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, extra[i].name);
		if (!res || resource_size(res) < extra[i].minimum)
			return -EINVAL;
		dp->banks[extra[i].bank] = devm_ioremap_resource(dev, res);
		if (IS_ERR(dp->banks[extra[i].bank]))
			return PTR_ERR(dp->banks[extra[i].bank]);
		dp->resources[extra[i].bank] = *res;
	}
	dp->usb2_default = devm_apple_tunable_parse(dev, dev->of_node,
		"apple,tunable-usb2-default", &dp->resources[0]);
	if (IS_ERR(dp->usb2_default))
		return PTR_ERR(dp->usb2_default);
	dp->usb2_host = devm_apple_tunable_parse(dev, dev->of_node,
		"apple,tunable-usb2-host", &dp->resources[0]);
	if (IS_ERR(dp->usb2_host))
		return PTR_ERR(dp->usb2_host);
	dp->pipe_default = devm_apple_tunable_parse(dev, dev->of_node,
		"apple,tunable-pipe-default", &dp->resources[44]);
	if (IS_ERR(dp->pipe_default))
		return PTR_ERR(dp->pipe_default);
	for (unsigned int i = 0; i < ARRAY_SIZE(cio_tunables); i++) {
		struct apple_tunable *t = devm_apple_tunable_parse(dev, dev->of_node,
			cio_tunables[i].name, &dp->resources[cio_tunables[i].bank]);

		if (IS_ERR(t) && PTR_ERR(t) != -ENOENT)
			return PTR_ERR(t);
		dp->cio_tunables[i] = IS_ERR(t) ? NULL : t;
	}
	dp->usb_io = (struct apple_t6050_usb_io) {
		.context = dp, .read = combo_read, .write = combo_write,
		.poll = combo_poll, .delay = combo_delay,
	};
	dp->usb_platform = (struct apple_t6050_usb_platform) {
		.context = dp, .io = &dp->usb_io, .wrapper_power = combo_wrapper,
		.firmware = combo_firmware, .tunable = combo_tunable, .pause_ns = combo_pause,
	};
	dp->reset = (struct reset_controller_dev) {
		.owner = THIS_MODULE, .ops = &combo_reset_ops, .dev = dev,
		.of_node = dev->of_node, .nr_resets = 1,
	};
	ret = devm_reset_controller_register(dev, &dp->reset);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < 2; i++) {
		dp->combo_phy[i] = devm_phy_create(dev, NULL, i ? &combo_usb3_ops : &combo_usb2_ops);
		if (IS_ERR(dp->combo_phy[i]))
			return PTR_ERR(dp->combo_phy[i]);
		phy_set_drvdata(dp->combo_phy[i], dp);
	}
	/* A deep sleep's hold of the always-on domain: see combo_sleep_board(). */
	if (!combo_sleep_board())
		return 0;
	dp->pm_nb.notifier_call = combo_pm_notify;
	ret = register_pm_notifier(&dp->pm_nb);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, combo_pm_notifier_remove, dp);
}

