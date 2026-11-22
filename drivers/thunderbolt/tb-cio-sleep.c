// SPDX-License-Identifier: GPL-2.0-only
/* Included by tb.c. Native CIO PM keeps tunnel allocations and host ownership. */
static const struct tb_cm_ops tb_cm_ops;
static void tb_restore_dp_resources(struct tb_switch *sw);

static int tb_cio_check_topology(struct tb_switch *sw, bool wake)
{
	struct tb_port *port;
	int ret;

	if (sw->is_unplugged || !sw->uid)
		return -ENODEV;
	tb_switch_for_each_port(sw, port) {
		/* Released for the sleep, see tb_cio_release_xdomains(). */
		if (port->xdomain)
			continue;
		if (!tb_port_has_remote(port))
			continue;
		/* After a wake, a router lost in the sleep is retired at complete. */
		if (wake && port->remote->sw->is_unplugged)
			continue;
		ret = tb_cio_check_topology(port->remote->sw, wake);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * A host at the other end of a link (an XDomain, such as a host-to-host cable to another
 * Mac) owns its half of it, which the router's power cycle cannot keep: release it as an
 * unplug does, its service drivers and paths with it. Nothing finds it again after the
 * wake: the link does not train back without a plug event (both routers stay in lane
 * init), so the peer returns only with a replug or a reboot.
 */
static void tb_cio_release_xdomains(struct tb *tb, struct tb_switch *sw)
{
	struct tb_port *port;

	tb_switch_for_each_port(sw, port) {
		struct tb_xdomain *xd;

		if (!tb_port_is_null(port) || tb_is_upstream_port(port))
			continue;
		if (tb_port_has_remote(port)) {
			tb_cio_release_xdomains(tb, port->remote->sw);
			continue;
		}
		/*
		 * By its route, through the router tree: on the J713 the adapter iterated here
		 * did not hold the peer itself when the release first ran (0:1's xdomain was
		 * unset), while the route lookup finds it.
		 */
		xd = tb_xdomain_find_by_route(tb, tb_downstream_route(port));
		tb_port_dbg(port, "sleep: xdomain %s, port's %s\n", xd ? "found" : "none",
			    port->xdomain ? "set" : "unset");
		if (!xd)
			continue;
		tb_port_info(port, "xdomain released for the sleep\n");
		xd->is_unplugged = true;
		tb_xdomain_remove(xd);
		if (port->xdomain == xd)
			port->xdomain = NULL;
		__tb_disconnect_xdomain_paths(tb, xd, -1, -1, -1, -1);
		tb_xdomain_put(xd);
		tb_port_unconfigure_xdomain(port);
	}
}

int tb_cio_sleep_prepare(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	struct tb_port *port;
	int ret;

	if (tb->cm_ops != &tb_cm_ops || !tb->nhi->ops->router_sleep ||
	    !tb_switch_is_usb4(tb->root_switch))
		return -EOPNOTSUPP;
	mutex_lock(&tb->lock);
	if (tcm->cio_prepared || tcm->stopping || !tcm->hotplug_active) {
		ret = -EBUSY;
		goto out;
	}
	ret = tb_cio_check_topology(tb->root_switch, false);
	if (ret)
		goto out;
	/*
	 * TEST (J714s): macOS's sleep entry starts with its links' PM roles and its CL states'
	 * disable, before it tears the displays down (see tb_switch_cio_pm_roles()). A link
	 * that does not take them, or fails its CL-state cycle, refuses the sleep here, before
	 * anything is released.
	 */
	tb_switch_for_each_port(tb->root_switch, port) {
		if (!tb_port_has_remote(port))
			continue;
		ret = tb_switch_cio_pm_roles(port->remote->sw);
		if (ret)
			goto out;
	}
	/*
	 * Past the last refusal here. A tunnel's prepare can still fail below: the peers stay
	 * released then too, as after an unplug.
	 */
	tb_cio_release_xdomains(tb, tb->root_switch);
	tcm->hotplug_active = false;
	tcm->cio_prepared = true;
	/*
	 * Displays do not survive the host router's power cycle: release them
	 * now, while their owners still run, and pair them again afterwards as
	 * after a plug. Only the PCIe and USB tunnels are kept and replayed.
	 */
	tb_disconnect_and_release_dp(tb);
	tcm->cio_dp_released = true;
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		ret = tb_tunnel_sleep_prepare(tunnel);
		if (ret)
			break;
	}
	mutex_unlock(&tb->lock);
	/* A DPRX callback already outside the lock must finish before device PM. */
	flush_workqueue(tb->wq);
	/*
	 * Outside the lock, as after an unplug: the released peers' services unbind now and
	 * give back their rings, which the host must not hold across its power cycle.
	 */
	tb_domain_unregister_unplugged_xdomains(tb);
	/* Bandwidth release runs on a system queue, outside the CM workqueue. */
	for (unsigned int i = 0; i < ARRAY_SIZE(tcm->groups); i++) {
		if (cancel_delayed_work_sync(&tcm->groups[i].release_work)) {
			guard(mutex)(&tb->lock);
			tcm->cio_bw_pending |= BIT(i);
		}
	}
	if (ret)
		tb_cio_sleep_complete(tb);
	return ret;
out:
	mutex_unlock(&tb->lock);
	return ret;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep_prepare, "thunderbolt_apple");

/*
 * The restarted host router trains its links again once it has the cable word.
 * Return -ENOTCONN if a peer did not come back: it left or lost power.
 */
static int tb_cio_wait_links(struct tb *tb)
{
	/*
	 * This runs inside the system's resume. A host that asks for it waits two rounds, a
	 * little over 2 s: a cold start's training takes 0.45 s from the cable word, and a
	 * router woken from its own sleep has its retimer's answer 23 ms after it (macOS).
	 */
	unsigned int max = tb->nhi->quirks & QUIRK_CIO_WAKE_GIVES_UP ? 2 : 30;
	struct tb_port *port;
	int ret, lost = 0;

	tb_switch_for_each_port(tb->root_switch, port) {
		ktime_t start = ktime_get();
		unsigned int tries;

		if (!tb_port_has_remote(port))
			continue;
		for (tries = 0; tries < max; tries++) {
			ret = tb_wait_for_port(port, true);
			if (ret)
				break;
			if (tries + 1 < max)
				msleep(100);
		}
		tb_info(tb, "CIO port %u link after wake: %d (%u retries of %u at most, %lld ms)\n",
			port->port, ret, tries, max, ktime_ms_delta(ktime_get(), start));
		if (ret < 0)
			return ret;
		if (!ret)
			lost = -ENOTCONN;
	}
	return lost;
}

/*
 * Only the host side has to come back. Routers that left or lost power during
 * the sleep are marked unplugged by the router resume, and their tunnels are
 * skipped here and retired at complete, as after a hot unplug.
 */
static void tb_cio_test_usb3_back(struct tb *tb, struct tb_tunnel *tunnel, bool restarted);

static int tb_cio_wake_locked(struct tb *tb, bool *host_pci_lost)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	const char *step;
	bool restarted;
	u64 uid = 0;
	int ret;

	lockdep_assert_held(&tb->lock);
	if (!tcm->cio_changed)
		return 0;
	if (tcm->cio_ctl_stopped) {
		tb_ctl_start(tb->ctl);
		tcm->cio_ctl_stopped = false;
	}
	/* After the vendor sleep, the controller has been restarted by its NHI. */
	restarted = tcm->cio_host_slept;
	if (restarted) {
		/* A restarted router answers for its adapters only once enumerated. */
		step = "router enumerate";
		ret = tb_switch_configure(tb->root_switch);
		if (ret)
			goto fail;
	}
	if (tcm->cio_router.valid) {
		step = "router restore";
		ret = apple_router_restore_sleep(tb->root_switch, &tcm->cio_router);
		if (ret && !(tb->nhi->quirks & QUIRK_CIO_WAKE_GIVES_UP))
			goto fail;
		if (ret) {
			/* The wake goes on: the router then starts its link as from cold. */
			tb_warn(tb, "CIO router state not taken back after the sleep (%d): the router starts as from cold\n",
				ret);
			tcm->cio_router.valid = false;
		} else {
			tb_info(tb, "CIO router state given back after the sleep\n");
		}
	} else if (restarted && !(tb->nhi->quirks & QUIRK_CIO_NO_ROUTER_STATE)) {
		tb_warn(tb, "CIO router restarted with no saved state to give back: it starts as from cold\n");
	}
	step = "host router UID";
	ret = usb4_switch_read_uid(tb->root_switch, &uid);
	if (ret || uid != tcm->cio_host_uid) {
		ret = ret ?: -ENODEV;
		goto fail;
	}
	if (restarted && tb->nhi->ops->router_wake) {
		step = "router wake";
		ret = tb->nhi->ops->router_wake(tb->nhi);
		if (ret)
			goto fail;
		step = "link";
		ret = tb_cio_wait_links(tb);
		if (ret && ret != -ENOTCONN)
			goto fail;
	}
	step = "router resume";
	ret = tb_switch_resume(tb->root_switch, false);
	if (ret)
		goto fail;
	step = "topology";
	ret = tb_cio_check_topology(tb->root_switch, true);
	if (ret)
		goto fail;
	tb_restore_children(tb->root_switch);
	step = "tunnel replay";
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		if (tb_tunnel_is_invalid(tunnel)) {
			tb_tunnel_info(tunnel, "lost during CIO sleep\n");
			if (host_pci_lost && tb_tunnel_is_pci(tunnel) &&
			    !tb_route(tunnel->src_port->sw))
				*host_pci_lost = true;
			continue;
		}
		/* After a refused sleep the hops left enabled are found, not written. */
		tunnel->sleep_lost = restarted;
		ret = tb_tunnel_wake(tunnel);
		if (ret)
			goto fail;
		tb_cio_test_usb3_back(tb, tunnel, restarted);	/* TEST */
	}
	tb_switch_enter_redrive(tb->root_switch);
	tcm->cio_host_slept = false;
	tcm->cio_changed = false;
	/* Do not run DP callbacks or reopen hotplug until all drivers resume. */
	return 0;
fail:
	tb_err(tb, "CIO wake failed at %s: %d (UID %#llx)\n", step, ret, uid);
	return ret;
}

/*
 * TEST (sv6t9, J714s): what the next run has to tell about the host router's own sleep.
 * Before the routers' sleep bits: the LTSSM state of both ends of every PCIe tunnel, as
 * the routers see it (their tunnel's root port has sent PME_Turn_Off by then; macOS's link
 * reported L2 at once). Behind the host's request, once its outcome is known: how long the
 * dock took to report sleep ready, when the host's frames began, and the frames against
 * macOS's (the frame record, cio_frames, must be running).
 */
static const char *tb_cio_test_ltssm(int state)
{
	static const char * const names[] = {
		"Detect", "Polling", "Configuration", "Configuration.Idle", "Recovery",
		"Recovery.Idle", "L0", "L1", "L2", "Disabled", "Hot Reset",
	};

	if (state < 0)
		return "not read";
	return state < (int)ARRAY_SIZE(names) ? names[state] : "unknown";
}

static bool tb_cio_test_on(struct tb *tb)
{
	return (tb->nhi->quirks & QUIRK_CIO_MACOS_FRAMES) &&
	       of_machine_is_compatible("apple,j714s");
}

static void tb_cio_test_pcie_ltssm(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	int down, up;

	if (!tb_cio_test_on(tb))
		return;
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		if (!tb_tunnel_is_pci(tunnel))
			continue;
		down = usb4_pci_port_ltssm_state(tunnel->src_port);
		up = usb4_pci_port_ltssm_state(tunnel->dst_port);
		tb_info(tb, "TEST sv6t11: before the routers' sleep bits, the tunnelled PCIe's domain off on J714s, PCIe tunnel %llx:%u <-> %llx:%u: LTSSM %s (%d) and %s (%d) (8 is L2; sv6t9, that domain on: Detect (0) and Polling (1))\n",
			tb_route(tunnel->src_port->sw), tunnel->src_port->port,
			tb_route(tunnel->dst_port->sw), tunnel->dst_port->port,
			tb_cio_test_ltssm(down), down, tb_cio_test_ltssm(up), up);
	}
}

/*
 * TEST (sv6t10, J714s): the USB3 tunnel's adapters as the routers see them, read before its
 * hops and path enables go down for the sleep and once more after: ADP_USB3_CS_0..4 of each
 * end (CS_0: 31 PE, 30 V; CS_4 bit 7: the USB3 link valid in the USB4 specification, 6:0 its
 * rate). macOS never reads them; its USB host is powered there until the firmware sleeps.
 */
struct tb_cio_test_usb3 {
	struct tb_tunnel *tunnel;
	u32 down[5], up[5];
	int ret;
};

static void tb_cio_test_usb3_read(struct tb *tb, struct tb_cio_test_usb3 *t)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;

	memset(t, 0, sizeof(*t));
	t->ret = -ENOENT;
	if (!tb_cio_test_on(tb))
		return;
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		if (!tb_tunnel_is_usb3(tunnel) || !tunnel->src_port->cap_adap ||
		    !tunnel->dst_port->cap_adap)
			continue;
		t->tunnel = tunnel;
		t->ret = tb_port_read(tunnel->src_port, t->down, TB_CFG_PORT,
				      tunnel->src_port->cap_adap, ARRAY_SIZE(t->down)) ?:
			 tb_port_read(tunnel->dst_port, t->up, TB_CFG_PORT,
				      tunnel->dst_port->cap_adap, ARRAY_SIZE(t->up));
		return;
	}
}

static void tb_cio_test_usb3_report(struct tb *tb, const struct tb_cio_test_usb3 *before)
{
	struct tb_cio_test_usb3 after;

	if (!tb_cio_test_on(tb) || !before->tunnel)
		return;
	tb_cio_test_usb3_read(tb, &after);
	/*
	 * TEST (sv6t11): with the port's USB host suspended in place (xHCI and the dock's hubs,
	 * their links in U3) instead of removed, the tunnel is as macOS has it: up, with a host.
	 */
	tb_info(tb, "TEST sv6t11: USB3 tunnel %llx:%u <-> %llx:%u before the routers' sleep bits, kept up, its USB host suspended in place on J714s (dwc3-apple says so): ADP_USB3_CS_0 %#x and %#x (%d), path enable %s; ADP_USB3_CS_4 %#x and %#x before the tunnels' save, %#x and %#x after it (%d): USB3 link valid %s and %s, then %s and %s (sv6t10: the same tunnel with its host removed 2.6 s before the dock's sleep bit; macOS never reads these, its host is powered until 28 to 37 ms after the firmware's sleep)\n",
		tb_route(before->tunnel->src_port->sw), before->tunnel->src_port->port,
		tb_route(before->tunnel->dst_port->sw), before->tunnel->dst_port->port,
		after.down[ADP_USB3_CS_0], after.up[ADP_USB3_CS_0], before->ret,
		after.down[ADP_USB3_CS_0] & after.up[ADP_USB3_CS_0] & ADP_USB3_CS_0_PE ?
		"on at both ends" : "NOT on at both ends",
		before->down[ADP_USB3_CS_4], before->up[ADP_USB3_CS_4],
		after.down[ADP_USB3_CS_4], after.up[ADP_USB3_CS_4], after.ret,
		before->down[ADP_USB3_CS_4] & BIT(7) ? "yes" : "no",
		before->up[ADP_USB3_CS_4] & BIT(7) ? "yes" : "no",
		after.down[ADP_USB3_CS_4] & BIT(7) ? "yes" : "no",
		after.up[ADP_USB3_CS_4] & BIT(7) ? "yes" : "no");
}

/* TEST (sv6t10, J714s): the USB3 tunnel given back by tb_tunnel_wake(). */
static void tb_cio_test_usb3_back(struct tb *tb, struct tb_tunnel *tunnel, bool restarted)
{
	u32 down = 0, up = 0;
	int r1, r2;

	if (!tb_cio_test_on(tb) || !tb_tunnel_is_usb3(tunnel) || !tunnel->src_port->cap_adap ||
	    !tunnel->dst_port->cap_adap)
		return;
	r1 = tb_port_read(tunnel->src_port, &down, TB_CFG_PORT,
			  tunnel->src_port->cap_adap + ADP_USB3_CS_0, 1);
	r2 = tb_port_read(tunnel->dst_port, &up, TB_CFG_PORT,
			  tunnel->dst_port->cap_adap + ADP_USB3_CS_0, 1);
	tb_info(tb, "TEST sv6t11: USB3 tunnel %llx:%u <-> %llx:%u given back %s: ADP_USB3_CS_0 %#x and %#x (%d, %d), path enable %s (macOS mxt6 W1: 0xc0000400 on the host's adapter, 0xc0000410 on the dock's)\n",
		tb_route(tunnel->src_port->sw), tunnel->src_port->port,
		tb_route(tunnel->dst_port->sw), tunnel->dst_port->port,
		restarted ? "after the routers' sleep" : "with the routers woken where they were",
		down, up, r1, r2,
		(down & up & ADP_USB3_CS_0_PE) ? "on at both ends" : "NOT on at both ends");
}

static void tb_cio_test_report(struct tb *tb, ktime_t host_at, bool slept)
{
	ktime_t bit, ready;
	char text[768];
	u64 route;
	int ret;

	if (!tb_cio_test_on(tb))
		return;
	tb_switch_cio_test_dock(&route, &bit, &ready, &ret);
	if (route)
		tb_info(tb, "TEST sv6t11: dock %llx reported sleep ready %lld us after its sleep bit (%d), the port's USB host suspended in place on its USB3 tunnel; the host router's frames began %lld us after that (macOS mxt6b: the dock's notification 0.27 and 0.29 ms after its bit, ready at its first read again, the host 12.1 ms behind it in all four sleeps; sv6t8, sv6t9, the USB3 tunnel up without its host: ready 25.3 ms after the bit, 25317 and 25293 us to its notification)\n",
			route, ktime_us_delta(ready, bit), ret,
			host_at ? ktime_us_delta(host_at, ready) : -1LL);
	tb_switch_cio_test_host(tb, slept);	/* TEST (sv6u2) */
	tb_ctl_frames_sleep_check(tb->ctl, text, sizeof(text));
	tb_info(tb, "TEST sv6t9: router sleep %s; its frames against macOS's (mxt6): %s\n",
		slept ? "taken" : "refused", text);
}

int tb_cio_sleep(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;
	ktime_t host_at = 0;	/* TEST */
	struct tb_cio_test_usb3 usb3;	/* TEST */
	int ret, recover;

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared || tcm->cio_changed)
		return -EINVAL;
	/* A host router without a UID of its own reads zero here; wake compares like with like. */
	ret = usb4_switch_read_uid(tb->root_switch, &tcm->cio_host_uid);
	if (ret)
		return ret;
	tcm->cio_changed = true;
	tb_cio_test_usb3_read(tb, &usb3);	/* TEST */
	list_for_each_entry(tunnel, &tcm->tunnel_list, list) {
		ret = tb_tunnel_sleep(tunnel);
		if (ret)
			goto unwind;
	}
	tb_switch_exit_redrive(tb->root_switch);
	tb_cio_test_pcie_ltssm(tb);
	tb_cio_test_usb3_report(tb, &usb3);	/* TEST */
	ret = tb_switch_suspend_cio(tb->root_switch);
	host_at = ktime_get();
	if (!ret)
		ret = tb->nhi->ops->router_sleep(tb->nhi);
	if (ret)
		goto unwind;
	/*
	 * The controller took the vendor sleep: from here on only the NHI's
	 * restart wakes it. Without a saved state it comes back as after a
	 * cold start, and the wake enumerates and replays it the same way.
	 */
	tcm->cio_host_slept = true;
	if (!(tb->nhi->quirks & QUIRK_CIO_NO_ROUTER_STATE)) {
		ret = apple_router_save_sleep(tb->root_switch, &tcm->cio_router);
		if (ret)
			tb_warn(tb, "CIO router state not saved: %d\n", ret);
		else
			tb_info(tb, "CIO router state saved for the sleep\n");
	}
	tb_cio_test_report(tb, host_at, true);
	tb_ctl_stop(tb->ctl);
	tcm->cio_ctl_stopped = true;
	return 0;
unwind:
	tb_cio_test_report(tb, host_at, false);
	recover = tb_cio_wake_locked(tb, NULL);
	if (recover)
		panic("Apple CIO: router suspend %d, recovery %d", ret, recover);
	return ret;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep, "thunderbolt_apple");

/* Sets @host_pci_lost if the host's PCIe tunnel went with a lost router. */
int tb_cio_wake(struct tb *tb, bool *host_pci_lost)
{
	struct tb_cm *tcm = tb_priv(tb);

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared)
		return -EINVAL;
	return tb_cio_wake_locked(tb, host_pci_lost);
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_wake, "thunderbolt_apple");

void tb_cio_sleep_complete(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);
	struct tb_tunnel *tunnel;

	guard(mutex)(&tb->lock);
	if (!tcm->cio_prepared)
		return;
	if (tcm->cio_changed || tcm->cio_ctl_stopped || tcm->cio_router.valid ||
	    tcm->cio_host_slept)
		panic("Apple CIO: completing an unrestored router transaction");
	list_for_each_entry(tunnel, &tcm->tunnel_list, list)
		tb_tunnel_sleep_complete(tunnel);
	tcm->cio_prepared = false;
	tcm->hotplug_active = true;
	if (tcm->cio_dp_released) {
		tcm->cio_dp_released = false;
		tb_restore_dp_resources(tb->root_switch);
		tcm->dp_tunnel_pending = false;
		tb_tunnel_dp(tb);
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(tcm->groups); i++) {
		if (tcm->cio_bw_pending & BIT(i))
			mod_delayed_work(system_percpu_wq, &tcm->groups[i].release_work, 0);
	}
	tcm->cio_bw_pending = 0;
	/* The ordinary complete hook rescans events suppressed during sleep. */
	tcm->resume_cleanup_pending = true;
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep_complete, "thunderbolt_apple");

/*
 * A prepare refused at or after tb_cio_sleep_prepare(): nothing slept, but the PM core
 * calls no complete for the refusing device, so do here what tb_complete() would. Peers
 * released for the sleep still have their links up: unregister them and rescan, which
 * finds them again. Without a transaction open it is an ordinary rescan.
 */
void tb_cio_sleep_refused(struct tb *tb)
{
	struct tb_cm *tcm = tb_priv(tb);

	tb_cio_sleep_complete(tb);
	scoped_guard(mutex, &tb->lock) {
		if (tcm->resume_cleanup_pending) {
			tb_resume_cleanup(tb);
			tcm->resume_cleanup_pending = false;
		}
	}
	tb_domain_unregister_unplugged_xdomains(tb);
	guard(mutex)(&tb->lock);
	/* Not while the connection manager stops, or before it scans at all. */
	if (tcm->hotplug_active && !tcm->stopping)
		tb_scan_switch(tb->root_switch);
}
EXPORT_SYMBOL_FOR_MODULES(tb_cio_sleep_refused, "thunderbolt_apple");
