// SPDX-License-Identifier: GPL-2.0-only
/* Included by apple.c: one owner for the bound host's complete power cycle. */
static int apple_nhi_cio_router_sleep(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct tb_switch *sw = anhi->tb->root_switch;
	u32 command = 1, state, val;
	int cap, ret;

	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0)
		return cap;
	/*
	 * The controller only accepts the vendor sleep request once the
	 * host router itself was told to sleep. Router-level wakes stay off.
	 */
	ret = tb_sw_read(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		return ret;
	val &= ~(ROUTER_CS_5_WOP | ROUTER_CS_5_WOU);
	val |= ROUTER_CS_5_SLP;
	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		return ret;
	/* Sleep ready is advisory here; the controller state below decides. */
	for (unsigned int i = 0; i < 20; i++) {
		ret = tb_sw_read(sw, &val, TB_CFG_SWITCH, ROUTER_CS_6, 1);
		if (ret)
			return ret;
		if (val & ROUTER_CS_6_SLPR)
			break;
		usleep_range(10000, 11000);
	}
	dev_info(anhi->dev, "CIO host router sleep set, ready %s\n",
		 val & ROUTER_CS_6_SLPR ? "reported" : "not reported");
	ret = tb_sw_write(sw, &command, TB_CFG_SWITCH, cap + 0xb, 1);
	if (ret)
		return ret;
	ret = readl_poll_timeout(anhi->acio->rc_base + APPLE_CIO_M3_STAT, state,
				 state == 0x02000000, 10000, 1000000);
	dev_info(anhi->dev, "CIO controller sleep state %#x: %d\n", state, ret);
	return ret;
}

/* As at the first start: unlock the lane adapters, then the cable word starts the link. */
static int apple_nhi_cio_router_wake(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct tb_switch *sw = anhi->tb->root_switch;
	struct tb_port *port;
	int cap, ret;

	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0)
		return cap;
	tb_switch_for_each_port(sw, port) {
		if (!tb_port_is_null(port))
			continue;
		ret = tb_port_unlock(port);
		if (ret)
			dev_warn(anhi->dev, "CIO port %u unlock failed: %d\n", port->port, ret);
	}
	ret = tb_sw_write(sw, &anhi->acio->current_cable_info, TB_CFG_SWITCH,
			  cap + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	dev_info(anhi->dev, "CIO cable word %#x written: %d\n", anhi->acio->current_cable_info,
		 ret);
	return ret;
}

static int apple_nhi_cio_bind_owner(struct apple_nhi *anhi)
{
	struct of_phandle_args args;
	struct platform_device *dart;
	struct phy *phy;
	int ret;

	if (anhi->acio->pd_list->num_pds != 4)
		return -EINVAL;
	phy = devm_of_phy_get(anhi->dev, anhi->acio->np, "cio");
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	ret = of_parse_phandle_with_args(anhi->np, "iommus", "#iommu-cells", 0, &args);
	if (ret)
		return ret;
	dart = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!dart)
		return -EPROBE_DEFER;
	ret = apple_dart_cio_bind(anhi->dev, &dart->dev, &anhi->cio_dart);
	put_device(&dart->dev);
	if (ret)
		return ret;
	anhi->cio_phy = phy;
	anhi->ops.router_sleep = apple_nhi_cio_router_sleep;
	anhi->ops.router_wake = apple_nhi_cio_router_wake;
	return 0;
}

/*
 * TEST: T6050, from macOS sleeps and wakes with a dock on the right port (runs mxt4, mxt5).
 * The router sleeps in place: the display paths go, the PCIe and USB tunnels stay allocated,
 * the router is told to sleep and its firmware runs its own sleep; then the NHI's DART, the
 * CIO_PCIE, CIO_USB and CIO_AUSS domains, the processor's power-off, the CIO domain, and the
 * PHY. After the wake: the PHY, the CIO domains, the processor from cold, and the tunnelled
 * PCIe's domain last.
 *
 * The firmware's own sleep is the J713's, not the sleep bit alone. In all three traced
 * sleeps the controller's state word (rc block +0xa8) is read as 0x01000000 right behind the
 * last configuration exchange that follows the sleep bit, and as 0x02000000 10 to 12 ms
 * later; the firmware logs "Entering Phase 1. action=4" and "ACIO SLEEP process completed"
 * in between. Five or six exchanges follow (the state transfer the connection manager makes
 * next), the rings stop 27 ms later and the processor is powered off. The exchanges'
 * contents are not in the traces: the request is the one the J713 path already sends. With
 * the sleep bit alone (run sv6t4) the firmware logged the bit and nothing else, was powered
 * off 6 ms later, and came back from cold (action 0) with the dock's retimer silent.
 */
#define APPLE_CIO_VSE_SLEEP		0x0b	/* vendor capability word: 1 = enter sleep */
#define APPLE_CIO_M3_STAT_ASLEEP	0x02000000
#define APPLE_CIO_T6050_SLEEP_US	500000	/* macOS: asleep within 12 ms of the request */
#define APPLE_CIO_PORT_CS_19_WAKES	GENMASK(19, 16)	/* WoC, WoD, WoU4 and bit 19 (name not known) */

static int apple_nhi_cio_router_sleep_t6050(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	struct tb_switch *sw = anhi->tb->root_switch;
	u32 command = 1, before, state, val, cs5[3] = {}, cs6 = 0, p19[3] = {};
	struct tb_port *port, *lane = NULL;
	bool cs5_written = false;
	int cap, ret, sent, done;
	ktime_t start;

	done = atomic_read(&acio->fw_sleep_done);
	/*
	 * TEST: the frames macOS sends to the host router, in its order (run mxt6, both sleeps;
	 * C1 -0.2033 to -0.2017, behind the dock's own sleep bit and sleep ready):
	 *   PORT_CS_19 of the connected lane adapter with bits 19:16 set   (0x800f000e)
	 *   ROUTER_CS_5 <- 0x00000006 (WoP, WoU)
	 *   PORT_CS_19 with bit 18, wake on USB4, alone of the four          (0x8004000e)
	 *   ROUTER_CS_5 <- 0x00000000
	 *   ROUTER_CS_5 <- 0x00000001: the sleep bit alone
	 *   one read of ROUTER_CS_6, the walk to the vendor capability, its word +0xb <- 1
	 * The three ROUTER_CS_5 words are written as the trace has them, not as read and
	 * changed: what the router held before each is printed, and given back on a refusal.
	 * macOS's replies are not in the trace, so whether it read the same words is not known.
	 */
	tb_switch_for_each_port(sw, port) {
		if (!tb_port_is_null(port) || !port->cap_usb4 || !tb_port_has_remote(port))
			continue;
		lane = port;
		break;
	}
	if (lane) {
		ret = tb_port_read(lane, &p19[0], TB_CFG_PORT, lane->cap_usb4 + PORT_CS_19, 1);
		if (ret)
			return ret;
		p19[1] = p19[0] | APPLE_CIO_PORT_CS_19_WAKES;
		ret = tb_port_write(lane, &p19[1], TB_CFG_PORT, lane->cap_usb4 + PORT_CS_19, 1);
		if (ret)
			return ret;
	}
	ret = tb_sw_read(sw, &cs5[0], TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	val = ROUTER_CS_5_WOP | ROUTER_CS_5_WOU;
	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	cs5_written = true;
	if (lane) {
		ret = tb_port_read(lane, &val, TB_CFG_PORT, lane->cap_usb4 + PORT_CS_19, 1);
		if (ret)
			goto undo;
		p19[2] = (val & ~APPLE_CIO_PORT_CS_19_WAKES) | PORT_CS_19_WOU4;
		ret = tb_port_write(lane, &p19[2], TB_CFG_PORT, lane->cap_usb4 + PORT_CS_19, 1);
		if (ret)
			goto undo;
	}
	ret = tb_sw_read(sw, &cs5[1], TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	val = 0;
	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	ret = tb_sw_read(sw, &cs5[2], TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	val = ROUTER_CS_5_SLP;
	ret = tb_sw_write(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
	if (ret)
		goto undo;
	/* Sleep ready is read once, as macOS does: the controller's state below decides. */
	ret = tb_sw_read(sw, &cs6, TB_CFG_SWITCH, ROUTER_CS_6, 1);
	if (ret)
		goto undo;
	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0) {
		ret = cap;
		goto undo;
	}
	before = readl(acio->rc_base + APPLE_CIO_M3_STAT);
	start = ktime_get();
	/* A firmware already on its way down may not answer the request: its state decides. */
	sent = tb_sw_write(sw, &command, TB_CFG_SWITCH, cap + APPLE_CIO_VSE_SLEEP, 1);
	ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
				 state == APPLE_CIO_M3_STAT_ASLEEP, 1000,
				 APPLE_CIO_T6050_SLEEP_US);
	anhi->cio_test_asleep = ret ? 0 : ktime_get();	/* TEST (sv6t11), within the 1 ms poll */
	/* Its log comes over the mailbox a little behind the state word. */
	for (unsigned int i = 0; !ret && i < 20 && atomic_read(&acio->fw_sleep_done) == done; i++)
		usleep_range(1000, 2000);
	/* Printed behind the request: macOS sends it 0.5 ms after the sleep bit. */
	dev_info(anhi->dev, "CIO host router sleep set, ready %s\n",
		 cs6 & ROUTER_CS_6_SLPR ? "reported" : "not reported");
	dev_info(anhi->dev, "T6050 sleep: host frames as macOS: lane adapter %d PORT_CS_19 %#x -> %#x -> %#x (macOS: 0x800f000e, 0x8004000e); ROUTER_CS_5 held %#x, %#x, %#x before 0x6, 0x0, 0x1 were written; ROUTER_CS_6 %#x; vendor capability at %#x (macOS: 0x35), request word %#x\n",
		 lane ? lane->port : -1, p19[0], p19[1], p19[2], cs5[0], cs5[1], cs5[2], cs6, cap,
		 cap + APPLE_CIO_VSE_SLEEP);
	dev_info(anhi->dev, "T6050 sleep: router firmware sleep requested (%d): state %#x -> %#x in %lld us (macOS: 0x1000000 -> 0x2000000 within 12 ms; waited for %u ms at most): %d; its log: lane init action %d, sleep process completed %s\n",
		 sent, before, state, ktime_us_delta(ktime_get(), start),
		 APPLE_CIO_T6050_SLEEP_US / 1000, ret, atomic_read(&acio->fw_lane_action),
		 atomic_read(&acio->fw_sleep_done) != done ? "yes" : "NO");
	if (!ret)
		return 0;
	dev_err(anhi->dev, "T6050 sleep: the router firmware did not take its sleep (state %#x, request %d): the sleep is refused, the router is woken as it is\n",
		state, sent);
undo:
	/* The router is awake: its wake and sleep words go back to what it held. */
	if (cs5_written) {
		val = cs5[0] & ~ROUTER_CS_5_SLP;
		sent = tb_sw_write(sw, &val, TB_CFG_SWITCH, ROUTER_CS_5, 1);
		dev_info(anhi->dev, "T6050 sleep: ROUTER_CS_5 given back as %#x: %d\n", val, sent);
	}
	if (lane) {
		sent = tb_port_write(lane, &p19[0], TB_CFG_PORT, lane->cap_usb4 + PORT_CS_19, 1);
		dev_info(anhi->dev, "T6050 sleep: lane adapter %u PORT_CS_19 given back as %#x: %d\n",
			 lane->port, p19[0], sent);
	}
	return ret;
}

/*
 * TEST: the wake as macOS sends it (run mxt6, both wakes). The connection manager has
 * enumerated the router (ROUTER_CS_1..4) and given its state back (operation 0x8001: the
 * firmware answers "Waking up from sleep!" and lane init action 5) before this is called.
 * Then, before the cable word: router operation 0x31 with 17 words of 0, and router
 * operation 0x32 with 16 words of 0 and metadata 0x101. Their names and what they return
 * are not known here (macOS's replies are not in the trace): the two operation numbers and
 * the metadata word are placeholders copied from the trace, to be justified; an operation
 * the router refuses is printed and the wake goes on. No lane adapter is unlocked before
 * the cable word: macOS writes ADP_CS_4 of the lane adapter only behind its plug event,
 * when the link has trained, which the router's resume does here.
 */
#define APPLE_CIO_T6050_WAKE_OP_A	0x31
#define APPLE_CIO_T6050_WAKE_OP_B	0x32
#define APPLE_CIO_T6050_WAKE_OP_B_META	0x101

static void apple_nhi_cio_wake_op_t6050(struct apple_nhi *anhi, u16 opcode, u32 metadata)
{
	struct tb_switch *sw = anhi->tb->root_switch;
	u32 words[17] = {}, command = opcode | ROUTER_CS_26_OV;
	ktime_t deadline = ktime_add_ms(ktime_get(), 50);
	int ret;

	words[16] = metadata;
	ret = tb_sw_write(sw, words, TB_CFG_SWITCH, ROUTER_CS_9, ARRAY_SIZE(words));
	if (!ret)
		ret = tb_sw_write(sw, &command, TB_CFG_SWITCH, ROUTER_CS_26, 1);
	while (!ret) {
		ret = tb_sw_read(sw, &command, TB_CFG_SWITCH, ROUTER_CS_26, 1);
		if (ret || !(command & ROUTER_CS_26_OV))
			break;
		if (ktime_after(ktime_get(), deadline))
			ret = -ETIMEDOUT;
		else
			usleep_range(100, 200);
	}
	if (!ret)
		ret = tb_sw_read(sw, words, TB_CFG_SWITCH, ROUTER_CS_9, ARRAY_SIZE(words));
	dev_info(anhi->dev, "T6050 sleep: router operation %#x (metadata %#x) before the cable word: %d, ROUTER_CS_26 %#x, data %#x %#x %#x %#x, metadata %#x\n",
		 opcode, metadata, ret, command, words[0], words[1], words[2], words[3], words[16]);
}

static int apple_nhi_cio_router_wake_t6050(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct tb_switch *sw = anhi->tb->root_switch;
	int cap, ret;

	apple_nhi_cio_wake_op_t6050(anhi, APPLE_CIO_T6050_WAKE_OP_A, 0);
	apple_nhi_cio_wake_op_t6050(anhi, APPLE_CIO_T6050_WAKE_OP_B, APPLE_CIO_T6050_WAKE_OP_B_META);
	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0)
		return cap;
	ret = tb_sw_write(sw, &anhi->acio->current_cable_info, TB_CFG_SWITCH,
			  cap + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	dev_info(anhi->dev, "CIO cable word %#x written: %d\n", anhi->acio->current_cable_info,
		 ret);
	return ret;
}

enum { CIO_PHY_PREPARE, CIO_PHY_OFF, CIO_PHY_ON, CIO_PHY_FINISH, CIO_PHY_LOST };

static void apple_nhi_cio_phy_put(struct apple_nhi *anhi)
{
	if (anhi->cio_phy_fn[CIO_PHY_PREPARE])
		symbol_put(apple_t6050_phy_cio_prepare);
	if (anhi->cio_phy_fn[CIO_PHY_OFF])
		symbol_put(apple_t6050_phy_cio_off);
	if (anhi->cio_phy_fn[CIO_PHY_ON])
		symbol_put(apple_t6050_phy_cio_on);
	if (anhi->cio_phy_fn[CIO_PHY_FINISH])
		symbol_put(apple_t6050_phy_cio_finish);
	if (anhi->cio_phy_fn[CIO_PHY_LOST])
		symbol_put(apple_t6050_phy_cio_lost);
	memset(anhi->cio_phy_fn, 0, sizeof(anhi->cio_phy_fn));
}

/*
 * TEST (J714s): the port's USB host is kept on the router's USB3 tunnel through the sleep, see
 * apple_t6050_phy_cio_usb(). Without the PHY's call the host stops before the sleep, as before.
 */
static void apple_nhi_cio_usb_bind(struct apple_nhi *anhi, bool bind)
{
	if (bind) {
		if (!(of_machine_is_compatible("apple,j714s") ||
		      of_machine_is_compatible("apple,j716c")))
			return;
		anhi->cio_phy_usb = symbol_get(apple_t6050_phy_cio_usb);
		if (anhi->cio_phy_usb)
			anhi->cio_phy_usb(anhi->cio_phy_dev, APPLE_T6050_CIO_USB_BIND);
		return;
	}
	if (!anhi->cio_phy_usb)
		return;
	anhi->cio_phy_usb(anhi->cio_phy_dev, APPLE_T6050_CIO_USB_UNBIND);
	symbol_put(apple_t6050_phy_cio_usb);
	anhi->cio_phy_usb = NULL;
}

/*
 * TEST (J714s): the kept host's PIPE goes to its dummy producer once the router firmware has
 * taken its sleep, before the NHI's DART, the CIO domains, the processor and the PHY's CIO
 * shutdown. macOS gates ATC2_CIO_USB 26 to 28 ms and ATC2_USB 28 to 37 ms after the firmware's
 * state word reads asleep, with no traced AP access in the 25 to 27 ms before its rings stop
 * (runs mxt4, mxt5, mxt6, mxt6b). After a refused sleep the same is done with the router woken
 * again, before anything can stop that router under the host (run sv6t4).
 */
static int apple_nhi_cio_usb_park(struct apple_nhi *anhi, const char *when)
{
	int ret;

	if (!anhi->cio_phy_usb)
		return 0;
	ret = anhi->cio_phy_usb(anhi->cio_phy_dev, APPLE_T6050_CIO_USB_PARK);
	if (ret)
		dev_err(anhi->dev, "T6050 sleep: the port's USB host not parked off the router's USB3 tunnel %s: %d\n",
			when, ret);
	return ret;
}

static void apple_nhi_cio_unbind_t6050(void *data)
{
	struct apple_nhi *anhi = data;

	WRITE_ONCE(anhi->acio->sleep_in_place, false);
	apple_nhi_cio_usb_bind(anhi, false);
	apple_nhi_cio_phy_put(anhi);
	put_device(anhi->cio_phy_dev);
}

/*
 * What the device tree has to give for it: five domains in the order CIO, CIO_PCIE,
 * CIO_USB, CIO_AUSS, PCIE; the port's combo PHY ("apple,cio-phy"); and the clock
 * protection slice of the NHI's DART. Without one of them the port is powered off for the
 * sleep instead.
 */
static int apple_nhi_cio_bind_owner_t6050(struct apple_nhi *anhi)
{
	struct of_phandle_args args;
	struct platform_device *dart, *phy;
	struct device_node *np;
	int ret;

	if (anhi->acio->pd_list->num_pds != 5)
		return -EINVAL;
	np = of_parse_phandle(anhi->acio->np, "apple,cio-phy", 0);
	if (!np)
		return -ENODEV;
	phy = of_find_device_by_node(np);
	of_node_put(np);
	if (!phy)
		return -EPROBE_DEFER;
	anhi->cio_phy_fn[CIO_PHY_PREPARE] = symbol_get(apple_t6050_phy_cio_prepare);
	anhi->cio_phy_fn[CIO_PHY_OFF] = symbol_get(apple_t6050_phy_cio_off);
	anhi->cio_phy_fn[CIO_PHY_ON] = symbol_get(apple_t6050_phy_cio_on);
	anhi->cio_phy_fn[CIO_PHY_FINISH] = symbol_get(apple_t6050_phy_cio_finish);
	anhi->cio_phy_fn[CIO_PHY_LOST] = symbol_get(apple_t6050_phy_cio_lost);
	if (!anhi->cio_phy_fn[CIO_PHY_PREPARE] || !anhi->cio_phy_fn[CIO_PHY_OFF] ||
	    !anhi->cio_phy_fn[CIO_PHY_ON] || !anhi->cio_phy_fn[CIO_PHY_FINISH] ||
	    !anhi->cio_phy_fn[CIO_PHY_LOST]) {
		ret = -ENODEV;
		goto put;
	}
	ret = of_parse_phandle_with_args(anhi->np, "iommus", "#iommu-cells", 0, &args);
	if (ret)
		goto put;
	dart = of_find_device_by_node(args.np);
	of_node_put(args.np);
	if (!dart) {
		ret = -EPROBE_DEFER;
		goto put;
	}
	ret = apple_dart_cio_bind(anhi->dev, &dart->dev, &anhi->cio_dart);
	put_device(&dart->dev);
	if (ret)
		goto put;
	anhi->cio_phy_dev = &phy->dev;
	anhi->cio_t6050 = true;
	anhi->ops.router_sleep = apple_nhi_cio_router_sleep_t6050;
	anhi->ops.router_wake = apple_nhi_cio_router_wake_t6050;
	WRITE_ONCE(anhi->acio->sleep_in_place, true);
	ret = devm_add_action(anhi->dev, apple_nhi_cio_unbind_t6050, anhi);
	if (ret) {
		/* Nothing is prepared yet: the port takes the power-off path. */
		WRITE_ONCE(anhi->acio->sleep_in_place, false);
		anhi->cio_t6050 = false;
		anhi->cio_dart = NULL;
		anhi->cio_phy_dev = NULL;
		anhi->ops.router_sleep = NULL;
		anhi->ops.router_wake = NULL;
		goto put;
	}
	apple_nhi_cio_usb_bind(anhi, true);
	dev_info(anhi->dev, "T6050 sleep: the router sleeps in place (PHY %s)\n",
		 dev_name(&phy->dev));
	return 0;
put:
	apple_nhi_cio_phy_put(anhi);
	put_device(&phy->dev);
	return ret;
}

/* Without the owner, a deep sleep with a dock attached takes the stock path. */
static int apple_nhi_cio_bind(struct apple_nhi *anhi)
{
	int ret;

	if (!cio_s2r)
		return 0;
	if (anhi->acio->hw == &apple_cio_t6050_hw &&
	    (of_machine_is_compatible("apple,j714s") || of_machine_is_compatible("apple,j716c"))) {
		ret = apple_nhi_cio_bind_owner_t6050(anhi);
		if (ret == -EPROBE_DEFER)
			return ret;
		if (ret)
			dev_info(anhi->dev, "T6050 sleep: no sleep in place for this router (%d): the port is powered off for a sleep\n",
				 ret);
		return 0;
	}
	if (anhi->acio->hw != &apple_cio_t8132_hw ||
	    !of_machine_is_compatible("apple,j713"))
		return 0;
	ret = apple_nhi_cio_bind_owner(anhi);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_warn(anhi->dev, "no CIO sleep owner: %d\n", ret);
	return 0;
}

/* The port's PHY: T8132's lane PHY handle, or T6050's combo PHY through its module's calls. */
static int apple_nhi_cio_phy(struct apple_nhi *anhi, unsigned int step)
{
	if (anhi->cio_phy_dev)
		return anhi->cio_phy_fn[step](anhi->cio_phy_dev);
	switch (step) {
	case CIO_PHY_PREPARE:
		return apple_atcphy_cio_prepare(anhi->cio_phy);
	case CIO_PHY_OFF:
		return apple_atcphy_cio_off(anhi->cio_phy);
	case CIO_PHY_ON:
		return apple_atcphy_cio_on(anhi->cio_phy);
	default:
		return apple_atcphy_cio_finish(anhi->cio_phy);
	}
}

static int apple_nhi_cio_prepare(struct apple_nhi *anhi)
{
	struct apple_cio *acio = anhi->acio;
	struct platform_device *pcie;
	int ret, release;

	if (!anhi->cio_dart || pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;
	scoped_guard(mutex, &acio->lock) {
		if (!acio->rtk || !acio->current_cable_info || acio->removing ||
		    acio->current_cable_info != acio->target_cable_info || acio->native_sleep)
			return -EBUSY;
		WRITE_ONCE(acio->native_sleep, true);
	}
	/*
	 * A partner with no PCIe tunnel (a host-to-host cable, a USB4 device that tunnels
	 * only USB or DisplayPort) leaves the router nothing to stop behind it. T6050's
	 * sleep has only run with a dock's tunnel: it still refuses without one, before
	 * the connection manager releases anything (a host peer's link stays up).
	 * pcie_pm_prepared keeps the tunnel from changing until complete.
	 */
	scoped_guard(mutex, &acio->pcie_tunnel_lock)
		pcie = apple_cio_find_pcie_tunnel(acio);
	if (!pcie && anhi->cio_t6050) {
		ret = -ENODEV;
		goto release_cable;
	}
	/* No new rings or topology may appear after the connection manager closes. */
	ret = tb_cio_sleep_prepare(anhi->tb);
	if (ret)
		goto release_cm;
	/* The released displays' routes go down on this queue: wait for them. */
	if (acio->dp_wq)
		flush_workqueue(acio->dp_wq);
	for (unsigned int i = 1; i < anhi->n_rings; i++) {
		if (anhi->nhi.tx_rings[i] || anhi->nhi.rx_rings[i]) {
			ret = -EBUSY;
			goto release_cm;
		}
	}
	cancel_delayed_work_sync(&anhi->dp_aux_work);
	ret = apple_nhi_cio_phy(anhi, CIO_PHY_PREPARE);
	if (ret)
		goto release_cm;
	ret = apple_dart_cio_prepare(anhi->cio_dart);
	if (ret)
		goto release_phy;
	if (pcie) {
		ret = apple_pcie_tunnel_cio_prepare(&pcie->dev);
		if (ret)
			goto release_dart;
	} else {
		dev_info(anhi->dev, "CIO sleep with no PCIe tunnel behind the router\n");
	}
	anhi->cio_pcie = pcie ? &pcie->dev : NULL;
	anhi->cio_prepared = true;
	anhi->cio_pd_suspended = 0;
	anhi->cio_rtk_off = false;
	anhi->cio_cycled = false;
	return 0;
release_dart:
	/*
	 * Nothing has been stopped yet: on T6050 a transaction that cannot be let go is said and
	 * the sleep refused all the same (the port may need its cable plugged in again).
	 */
	release = apple_dart_cio_finish(anhi->cio_dart);
	if (release) {
		if (!anhi->cio_t6050)
			panic("Apple CIO: failed to release unused DART transaction");
		dev_err(anhi->dev, "CIO prepare refused (%d) and the unused DART transaction was not released: %d\n",
			ret, release);
	}
release_phy:
	release = apple_nhi_cio_phy(anhi, CIO_PHY_FINISH);
	if (release) {
		if (!anhi->cio_t6050)
			panic("Apple CIO: failed to release unused PHY transaction");
		dev_err(anhi->dev, "CIO prepare refused (%d) and the unused PHY transaction was not released: %d\n",
			ret, release);
	}
release_cm:
	/* No complete follows a refused prepare: this one finds released peers again. */
	tb_cio_sleep_refused(anhi->tb);
	if (pcie)
		put_device(&pcie->dev);
release_cable:
	scoped_guard(mutex, &acio->lock)
		WRITE_ONCE(acio->native_sleep, false);
	if (anhi->dp_aux_armed)
		mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work, 0);
	return ret;
}

static int apple_nhi_cio_start_cpu(void *data)
{
	struct apple_nhi *anhi = data;

	return reset_control_deassert(anhi->acio->reset);
}

static int apple_nhi_cio_power_on(struct apple_nhi *anhi)
{
	static const unsigned int order[] = { 3, 0, 2, 1 };
	struct apple_cio *acio = anhi->acio;
	u32 state;
	int ret;

	dev_info(anhi->dev, "CIO restore PHY\n");
	ret = apple_nhi_cio_phy(anhi, CIO_PHY_ON);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < ARRAY_SIZE(order); i++) {
		unsigned int n = order[i];
		struct device *pd = acio->pd_list->pd_devs[n];

		if (!(anhi->cio_pd_suspended & BIT(n)))
			continue;
		/* Balance every attempted suspend, including one which stayed on. */
		dev_info(anhi->dev, "CIO restore domain %u\n", n);
		dev_pm_genpd_resume(pd);
		anhi->cio_pd_suspended &= ~BIT(n);
		ret = apple_pmgr_ps_verify_state(pd, true);
		if (ret)
			return ret;
	}
	if (anhi->cio_rtk_off) {
		/* The ACIO reset only deasserts: only its domains going off reset the CPU. */
		if (!anhi->cio_cycled)
			return -EUCLEAN;
		dev_info(anhi->dev, "CIO restart controller\n");
		ret = apple_rtkit_cold_restart(acio->rtk, apple_nhi_cio_start_cpu, anhi);
		if (ret)
			return ret;
		ret = apple_cio_lstx_release(acio);
		if (ret)
			return ret;
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 (state & APPLE_CIO_M3_STAT_CODE_VALID) ||
					 FIELD_GET(APPLE_CIO_M3_STAT_STATE, state) ==
					 APPLE_CIO_M3_STAT_STATE_READY, 100, 500000);
		if (ret || (state & APPLE_CIO_M3_STAT_CODE_VALID))
			return ret ?: -EIO;
		apple_tunable_apply(acio->rc_base, acio->rc_tunable);
		if (acio->pcie_adapter_tunable)
			apple_tunable_apply(acio->pcie_adapter_base, acio->pcie_adapter_tunable);
		apple_tunable_apply(acio->router_ap_fw_base, acio->router_ap_fw_tunable);
		apple_tunable_apply(anhi->nhi_base, anhi->nhi_tunable);
		anhi->cio_rtk_off = false;
		anhi->cio_cycled = false;
	}
	/* The CPU and domains are back before the DART opens the NHI streams. */
	dev_info(anhi->dev, "CIO restore NHI DART\n");
	return apple_dart_cio_on(anhi->cio_dart);
}

/* Does the host router still have a router behind one of its lane adapters? */
static bool apple_nhi_cio_peer(struct apple_nhi *anhi)
{
	struct tb_port *port;

	guard(mutex)(&anhi->tb->lock);
	tb_switch_for_each_port(anhi->tb->root_switch, port)
		if (tb_port_is_null(port) && port->remote && !port->remote->sw->is_unplugged)
			return true;
	return false;
}

/*
 * A PCIe-C port whose tunnel went with a lost router does not wait for its link. On T6050
 * a router that is back without its dock is given up here, inside the resume and once: the
 * tunnels are retired by the connection manager's complete as after an unplug, the PCIe-C
 * port is closed without failing the resume, and the port's USB host is kept off the
 * router's USB3 tunnel, because the port controller may stop this router next with that
 * host still running.
 */
static int apple_nhi_cio_wake(struct apple_nhi *anhi)
{
	struct apple_cio *acio = anhi->acio;
	int answers = atomic_read(&acio->fw_at_answers);
	int timeouts = atomic_read(&acio->fw_at_timeouts);
	int woke = atomic_read(&acio->fw_wake_seen);
	ktime_t start = ktime_get();
	bool pci_lost = false, peer;
	int ret, phy = 0;

	ret = tb_cio_wake(anhi->tb, &pci_lost);
	if (!ret && pci_lost) {
		dev_warn(anhi->dev, "CIO dock lost during sleep; retiring its tunnels\n");
		if (anhi->cio_pcie)
			apple_pcie_tunnel_cio_lost(anhi->cio_pcie);
	}
	if (ret || !anhi->cio_t6050)
		return ret;
	peer = apple_nhi_cio_peer(anhi);
	if (!peer) {
		/* Also with no PCIe tunnel among the lost ones: the port waits for no link. */
		if (anhi->cio_pcie)
			apple_pcie_tunnel_cio_lost(anhi->cio_pcie);
		phy = apple_nhi_cio_phy(anhi, CIO_PHY_LOST);
	}
	dev_info(anhi->dev, "T6050 sleep: router woken in %lld ms: firmware lane init action %d (5 = its wake from the sleep, 0 = a cold start), \"Waking up from sleep\" %s, retimer register 12 answered %d, timed out %d; dock %s\n",
		 ktime_ms_delta(ktime_get(), start), atomic_read(&acio->fw_lane_action),
		 atomic_read(&acio->fw_wake_seen) != woke ? "yes" : "NO",
		 atomic_read(&acio->fw_at_answers) - answers,
		 atomic_read(&acio->fw_at_timeouts) - timeouts, peer ? "back" : "NOT back");
	if (!peer)
		dev_warn(anhi->dev, "T6050 sleep: the dock did not come back from the sleep: given up inside the resume, its tunnels retired as after an unplug, PCIe-C closed, USB3 kept off the router's tunnel (%d); the machine goes on without it\n",
			 phy);
	return 0;
}

/* Take the four domains down in order; one already attempted is only checked. */
static int apple_nhi_cio_domains_off(struct apple_nhi *anhi)
{
	static const unsigned int order[] = { 1, 2, 0, 3 };
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(order); i++) {
		unsigned int n = order[i];
		struct device *pd = anhi->acio->pd_list->pd_devs[n];

		if (!(anhi->cio_pd_suspended & BIT(n))) {
			anhi->cio_pd_suspended |= BIT(n);
			dev_info(anhi->dev, "CIO stop domain %u\n", n);
			dev_pm_genpd_suspend(pd);
		}
		ret = dev_pm_genpd_is_on(pd) ? -EBUSY : apple_pmgr_ps_verify_state(pd, false);
		if (ret)
			return ret;
	}
	anhi->cio_cycled = true;
	return 0;
}

/* A map or unmap in flight closes the DART only briefly: wait for it. */
static int apple_nhi_cio_dart_off(struct apple_nhi *anhi)
{
	int ret;

	for (unsigned int i = 0; i < 20; i++) {
		ret = apple_dart_cio_off(anhi->cio_dart);
		if (ret != -EBUSY)
			break;
		usleep_range(500, 1000);
	}
	return ret;
}

#include <linux/soc/apple/sleep.h>

/*
 * The router's own domain (ATC2_CIO, the acio node's first) across its sleep in place. macOS
 * takes it down before the PHY's CIO shutdown, and raises it at the wake only once the PHY's
 * CIO mode is up (right after UC_TNBL_DONE and the PHY's reset release), before CIO_PCIE,
 * CIO_USB and AUSS. Linux's router DART still holds the domain at the entry, so the PMGR's
 * driver would raise it at the very start of the wake, and the router would be powered while
 * the PHY is reset and started again: its link then reaches the dock's broadcast but never
 * trains, and the dock is lost. Taken down through the sleep driver, the domain is left to
 * this driver at the wake.
 */
static int apple_nhi_cio_router_pd_set(struct apple_nhi *anhi, bool on)
{
	struct device_node *np = of_parse_phandle(anhi->acio->np, "power-domains", 0);
	int ret;

	if (!np)
		return -ENODEV;
	ret = apple_sleep_domain_set(np, on);
	of_node_put(np);
	return ret;
}

/* T6050's domains: 0 CIO, 1 CIO_PCIE, 2 CIO_USB, 3 CIO_AUSS, 4 the tunnelled PCIe's. */
static int apple_nhi_cio_domains_t6050(struct apple_nhi *anhi, const u8 *order, unsigned int n,
				       bool on)
{
	bool held = false;
	int ret;

	for (unsigned int i = 0; i < n; i++) {
		struct device *pd = anhi->acio->pd_list->pd_devs[order[i]];

		if (on) {
			if (!(anhi->cio_pd_suspended & BIT(order[i])))
				continue;
			dev_info(anhi->dev, "CIO restore domain %u\n", order[i]);
			dev_pm_genpd_resume(pd);
			anhi->cio_pd_suspended &= ~BIT(order[i]);
			ret = apple_pmgr_ps_verify_state(pd, true);
		} else {
			if (!(anhi->cio_pd_suspended & BIT(order[i]))) {
				anhi->cio_pd_suspended |= BIT(order[i]);
				dev_info(anhi->dev, "CIO stop domain %u\n", order[i]);
				dev_pm_genpd_suspend(pd);
			}
			/*
			 * A device that suspends after the NHI (its DART, the tunnelled
			 * PCIe's) may still hold the domain: it then goes off with that
			 * device, and the sleep's last checks name a word left on.
			 */
			ret = dev_pm_genpd_is_on(pd) ? 1 : apple_pmgr_ps_verify_state(pd, false);
			if (ret) {
				dev_info(anhi->dev, "CIO domain %u still on (%d): left to its other users\n",
					 order[i], ret);
				held = true;
				ret = 0;
			}
		}
		if (ret) {
			dev_warn(anhi->dev, "CIO domain %u not on: %d\n", order[i], ret);
			return ret;
		}
	}
	return held;
}

static int apple_nhi_cio_start_cpu_t6050(void *data)
{
	struct apple_nhi *anhi = data;
	struct apple_cio *acio = anhi->acio;
	int ret;

	/* As at a start: the CIO reset clocks the wrapper, then the run value is stored. */
	ret = reset_control_deassert(acio->reset);
	if (ret)
		return ret;
	writel(APPLE_MXWRAP_CPU_CTRL_RUN, acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
	acio->cpu_running = true;
	return 0;
}

static int apple_nhi_cio_power_on_t6050(struct apple_nhi *anhi)
{
	static const u8 first[] = { 0, 1, 2, 3 }, last[] = { 4 };
	struct apple_cio *acio = anhi->acio;
	ktime_t start = ktime_get();
	u32 state;
	int ret;

	dev_info(anhi->dev, "CIO restore PHY\n");
	ret = apple_nhi_cio_phy(anhi, CIO_PHY_ON);
	/* The router's domain only now, with the PHY's CIO mode up (or failed: raised all the same). */
	if (anhi->cio_router_pd_down) {
		int up = apple_nhi_cio_router_pd_set(anhi, true);

		anhi->cio_router_pd_down = false;
		if (up)
			dev_err(anhi->dev, "CIO router domain not raised after the PHY: %d\n", up);
		if (!ret)
			ret = up;
	}
	if (ret)
		return ret;
	ret = apple_nhi_cio_domains_t6050(anhi, first, ARRAY_SIZE(first), true);
	if (ret < 0)
		return ret;
	if (anhi->cio_rtk_off) {
		/*
		 * Only its power going off resets the processor for a cold start. After a
		 * sleep that was refused late its domain may have gone off only with a
		 * device that suspended after the NHI: the start is tried all the same.
		 */
		if (!anhi->cio_cycled && !pm_resume_via_firmware())
			dev_warn(anhi->dev, "CIO domain not seen off before this restart\n");
		dev_info(anhi->dev, "CIO restart controller\n");
		ret = apple_rtkit_cold_restart(acio->rtk, apple_nhi_cio_start_cpu_t6050, anhi);
		if (ret)
			return ret;
		ret = apple_cio_lstx_release(acio);
		if (ret)
			return ret;
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 (state & APPLE_CIO_M3_STAT_CODE_VALID) ||
					 FIELD_GET(APPLE_CIO_M3_STAT_STATE, state) ==
					 APPLE_CIO_M3_STAT_STATE_READY, 100, 500000);
		if (ret || (state & APPLE_CIO_M3_STAT_CODE_VALID)) {
			dev_err(anhi->dev, "CIO controller not ready after the sleep: %d, state %#x\n",
				ret, state);
			return ret ?: -EIO;
		}
		apple_tunable_apply(acio->rc_base, acio->rc_tunable);
		if (acio->pcie_adapter_base && acio->pcie_adapter_tunable)
			apple_tunable_apply(acio->pcie_adapter_base, acio->pcie_adapter_tunable);
		apple_tunable_apply(acio->router_ap_fw_base, acio->router_ap_fw_tunable);
		apple_tunable_apply(anhi->nhi_base, anhi->nhi_tunable);
		anhi->cio_rtk_off = false;
		anhi->cio_cycled = false;
	}
	ret = apple_nhi_cio_domains_t6050(anhi, last, ARRAY_SIZE(last), true);
	if (ret < 0)
		return ret;
	dev_info(anhi->dev, "CIO restore NHI DART\n");
	ret = apple_dart_cio_on(anhi->cio_dart);
	dev_info(anhi->dev, "T6050 sleep: router back in place in %lld ms: %d\n",
		 ktime_ms_delta(ktime_get(), start), ret);
	return ret;
}

/* The rest of the way down once the router has taken its sleep. */
static int apple_nhi_cio_power_off_t6050(struct apple_nhi *anhi)
{
	static const u8 before[] = { 4, 1, 2, 3 }, after[] = { 0 };
	struct apple_cio *acio = anhi->acio;
	int ret;

	ret = apple_nhi_cio_domains_t6050(anhi, before, ARRAY_SIZE(before), false);
	if (ret < 0)
		return ret;
	if (!anhi->cio_rtk_off) {
		dev_info(anhi->dev, "CIO stop controller\n");
		ret = apple_rtkit_poweroff(acio->rtk);
		/*
		 * A processor that does not acknowledge its power-off is halted all the same:
		 * the caller then resets it by its domains and restarts it, which reopens the
		 * DART, and the sleep is refused. Only that restart failing is beyond recovery.
		 */
		if (ret)
			dev_err(anhi->dev, "CIO controller did not acknowledge its power-off (%d): halted, restarted by its domains, sleep refused\n",
				ret);
		anhi->cio_rtk_off = true;
		apple_rtkit_mailbox_off(acio->rtk);
		apple_cio_cpu_halt(acio);
		if (ret)
			return ret;
		/* J714s and J716c hold LSTX high through sleep; do not read the halted rc block. */
		ret = apple_cio_lstx_hold(acio, of_machine_is_compatible("apple,j714s") ||
					  of_machine_is_compatible("apple,j716c"));
		if (ret)
			return ret;
	}
	ret = apple_nhi_cio_domains_t6050(anhi, after, ARRAY_SIZE(after), false);
	if (ret < 0)
		return ret;
	/* Held on by its DART: taken down here, before the PHY's shutdown, as macOS does. */
	if (ret && (of_machine_is_compatible("apple,j714s") ||
		    of_machine_is_compatible("apple,j716c"))) {
		int down = apple_nhi_cio_router_pd_set(anhi, false);

		if (!down) {
			anhi->cio_router_pd_down = true;
			ret = 0;
		} else {
			dev_warn(anhi->dev, "CIO router domain not taken down for the sleep: %d\n", down);
		}
	}
	/* Held on by another user: the firmware sleep itself takes its power. */
	anhi->cio_cycled = !ret;
	return 0;
}

/* T6050's domain 4: the tunnelled PCIe's (atc2-pcie). */
static const u8 apple_nhi_cio_tunnel_pd[] = { 4 };

static int apple_nhi_cio_suspend_t6050(struct apple_nhi *anhi)
{
	ktime_t start = ktime_get();
	ktime_t park_at;	/* TEST */
	int ret, recover, usb;

	if (!apple_pcie_tunnel_cio_stopped(anhi->cio_pcie))
		return -EBUSY;
	/*
	 * The last point where the router is awake: a DART that could not be saved behind
	 * its sleep bit refuses the sleep here (the PCIe-C port is started again by its own
	 * resume), instead of ending in the panic below.
	 */
	ret = apple_dart_cio_check(anhi->cio_dart, true);
	if (ret) {
		dev_err(anhi->dev, "T6050 sleep: the NHI's DART cannot be saved (%d): sleep refused before the router is told to sleep\n",
			ret);
		return ret;
	}
	/*
	 * TEST: J714s. macOS gates the tunnelled PCIe's domain (ATC2_PCIE, domain 4 here) once
	 * the PCIe-C port is down, before any router is told to sleep (runs mxt4, mxt5, mxt6,
	 * mxt6b, seven sleeps: 5 to 7 us after APPCLK goes off, 0.46 to 0.58 s before the router
	 * firmware's request); it is on again only after the CIO domains and the processor at
	 * the wake. Linux gated it only behind the router's sleep. The port's stop has cleared
	 * its DART's clock protection by now, as macOS does before this gate. A sleep refused
	 * below powers it again here, before the PCIe-C port's resume starts the port from cold.
	 */
	if ((of_machine_is_compatible("apple,j714s") || of_machine_is_compatible("apple,j716c"))) {
		struct device *pd = anhi->acio->pd_list->pd_devs[apple_nhi_cio_tunnel_pd[0]];
		int held, on, ps;	/* TEST */

		held = apple_nhi_cio_domains_t6050(anhi, apple_nhi_cio_tunnel_pd,
						   ARRAY_SIZE(apple_nhi_cio_tunnel_pd), false);
		/* TEST (sv6t10): the gate as the routers are told to sleep. */
		on = dev_pm_genpd_is_on(pd);
		ps = on ? 1 : apple_pmgr_ps_verify_state(pd, false);
		dev_info(anhi->dev, "TEST sv6t10: the tunnelled PCIe's domain (4, atc2-pcie) before the routers' sleep bits: %s (held by another user %d, genpd %s, power state check %d) (macOS mxt4, mxt5, mxt6, mxt6b, seven sleeps: off 5 to 7 us after the PCIe-C port's clock, 0.46 to 0.58 s before the firmware's request; sv6t9: on until the ACIO stop behind the refusal)\n",
			 !on && !ps ? "off" : "STILL ON", held, on ? "on" : "off", ps);
	}
	dev_info(anhi->dev, "CIO save paths and routers\n");
	ret = tb_cio_sleep(anhi->tb);
	if (ret) {
		/* TEST (J714s): the router is awake again: its tunnel's USB host comes off it first. */
		apple_nhi_cio_usb_park(anhi, "after the refused sleep");
		if ((of_machine_is_compatible("apple,j714s") ||
		     of_machine_is_compatible("apple,j716c")) &&
		    apple_nhi_cio_domains_t6050(anhi, apple_nhi_cio_tunnel_pd,
						ARRAY_SIZE(apple_nhi_cio_tunnel_pd), true) < 0)
			dev_err(anhi->dev, "T6050 sleep: the tunnelled PCIe's domain not back after the refused sleep: the PCIe-C port's resume follows all the same\n");
		return ret;
	}
	/* TEST (J714s): the router firmware has taken its sleep: the USB host comes off its tunnel. */
	park_at = ktime_get();	/* TEST */
	usb = apple_nhi_cio_usb_park(anhi, "behind the router's sleep");
	/* TEST (sv6t11): the park against the firmware's state word, with macOS's order beside it. */
	if (anhi->cio_phy_usb)
		dev_info(anhi->dev, "TEST sv6t11: the port's USB host parked behind the router's sleep (%d): begun %lld us after the firmware's state word read asleep, %lld us long (macOS mxt4, mxt5, mxt6, mxt6b, seven sleeps, after that word: no traced AP access from its last NHI write, +0.40 to +0.45 ms, to the rings' stop at +25.6 to +27.5 ms; ATC2_CIO_USB off +26.1 to +27.9 ms, ATC2_CIO +27.0 to +29.6, ATC2_PHYMXWRAP +28.0 to +31.5, dart-usb2's PS_PROT slices 8 and 9 cleared +28.2 to +37.2, ATC2_USB off +28.3 to +37.3)\n",
			 usb, anhi->cio_test_asleep ? ktime_us_delta(park_at, anhi->cio_test_asleep) : -1LL,
			 ktime_us_delta(ktime_get(), park_at));
	dev_info(anhi->dev, "CIO stop NHI DART\n");
	ret = apple_nhi_cio_dart_off(anhi);
	/* Asleep, the router wakes only through a restart, which loses this DART. */
	if (ret) {
		dev_emerg(anhi->dev, "CIO step \"stop NHI DART\" failed (%d) after the router took its sleep: its translations cannot be restored\n",
			  ret);
		panic("Apple CIO: NHI DART not saved (%d) with the router asleep", ret);
	}
	/* A PIPE that did not park refuses the sleep: the router is restarted in place below. */
	ret = usb ?: apple_nhi_cio_power_off_t6050(anhi);
	if (!ret) {
		dev_info(anhi->dev, "CIO stop PHY\n");
		ret = apple_nhi_cio_phy(anhi, CIO_PHY_OFF);
	}
	if (!ret) {
		/* TEST (sv6t11): where the CIO side ends against macOS's ATC2_CIO and ATC2_USB. */
		if (anhi->cio_phy_usb && anhi->cio_test_asleep)
			dev_info(anhi->dev, "TEST sv6t11: CIO domains, processor and PHY's CIO mode down %lld us after the firmware's state word read asleep (macOS: ATC2_CIO off +27.0 to +29.6 ms); the USB controller's domain (atc2_usb) left on for the system's sleep, as its wrapper and common domains (macOS gates ATC2_USB itself, 0.10 to 0.30 ms after clearing dart-usb2's PS_PROT slices 8 and 9)\n",
				 ktime_us_delta(ktime_get(), anhi->cio_test_asleep));
		dev_info(anhi->dev, "T6050 sleep: router asleep in place, tunnels kept, in %lld ms\n",
			 ktime_ms_delta(ktime_get(), start));
		return 0;
	}
	dev_warn(anhi->dev, "CIO suspend failed %d; restarting the controller\n", ret);
	/* The router took its sleep: only a processor reset, by its domains, wakes it. */
	if (!anhi->cio_cycled) {
		recover = apple_nhi_cio_power_off_t6050(anhi);
		if (recover || !anhi->cio_cycled) {
			dev_emerg(anhi->dev, "CIO suspend failed (%d) with the router asleep and its processor could not be reset by its domains (%d, domain seen off %d): nothing wakes the router\n",
				  ret, recover, anhi->cio_cycled);
			panic("Apple CIO: suspend %d; controller not reset: %d", ret, recover);
		}
	}
	recover = apple_nhi_cio_power_on_t6050(anhi);
	if (recover)
		dev_emerg(anhi->dev, "CIO suspend failed (%d) and the router's restart failed: %d\n", ret, recover);
	else
		recover = apple_nhi_cio_wake(anhi);
	if (recover) {
		dev_emerg(anhi->dev, "CIO suspend failed (%d) and the router was not brought back (%d): its tunnels' devices cannot be resumed\n",
			  ret, recover);
		panic("Apple CIO: suspend %d, recovery %d", ret, recover);
	}
	dev_warn(anhi->dev, "T6050 sleep: router restarted in place after the failed step: sleep refused (%d)\n", ret);
	return ret;
}

static int apple_nhi_cio_suspend(struct apple_nhi *anhi)
{
	struct apple_cio *acio = anhi->acio;
	int ret, recover;

	if (anhi->cio_t6050)
		return apple_nhi_cio_suspend_t6050(anhi);
	if (anhi->cio_pcie && !apple_pcie_tunnel_cio_stopped(anhi->cio_pcie))
		return -EBUSY;
	dev_info(anhi->dev, "CIO save paths and routers\n");
	ret = tb_cio_sleep(anhi->tb);
	if (ret)
		return ret;
	dev_info(anhi->dev, "CIO stop NHI DART\n");
	ret = apple_nhi_cio_dart_off(anhi);
	/* Asleep, the controller wakes only through a restart, which loses this DART. */
	if (ret)
		panic("Apple CIO: NHI DART not saved (%d) with the router asleep", ret);
	dev_info(anhi->dev, "CIO stop controller\n");
	ret = apple_rtkit_poweroff(acio->rtk);
	if (ret)
		panic("Apple CIO: CPU shutdown failed %d; DMA stays closed", ret);
	anhi->cio_rtk_off = true;
	apple_rtkit_mailbox_off(acio->rtk);
	ret = apple_cio_lstx_hold(acio, !!(readl(acio->rc_base + APPLE_CIO_M3_CTRL) &
						 APPLE_CIO_LSTX_CTRL_LEVEL));
	if (ret)
		goto unwind;
	ret = apple_nhi_cio_domains_off(anhi);
	if (ret)
		goto unwind;
	dev_info(anhi->dev, "CIO stop PHY\n");
	ret = apple_nhi_cio_phy(anhi, CIO_PHY_OFF);
	if (!ret)
		return 0;
unwind:
	dev_warn(anhi->dev, "CIO suspend failed %d; restarting the controller\n", ret);
	/*
	 * The host router took the vendor sleep: only a controller restart wakes
	 * it, and only its domains going off reset the CPU for that restart.
	 */
	if (!anhi->cio_cycled) {
		recover = apple_nhi_cio_domains_off(anhi);
		if (recover)
			panic("Apple CIO: suspend %d; controller not reset: %d", ret, recover);
	}
	recover = apple_nhi_cio_power_on(anhi) ?: apple_nhi_cio_wake(anhi);
	if (recover)
		panic("Apple CIO: suspend %d, recovery %d", ret, recover);
	return ret;
}

static int apple_nhi_cio_resume(struct apple_nhi *anhi)
{
	ktime_t start = ktime_get();
	/* A lost dock is an unplug; only the host side must come back. */
	int ret = (anhi->cio_t6050 ? apple_nhi_cio_power_on_t6050(anhi) :
		   apple_nhi_cio_power_on(anhi)) ?: apple_nhi_cio_wake(anhi);

	if (anhi->cio_t6050)
		dev_info(anhi->dev, "T6050 sleep: the router's part of the resume took %lld ms: %d\n",
			 ktime_ms_delta(ktime_get(), start), ret);

	/* Child resume would otherwise access a dead tunnel or missing translations. */
	if (ret) {
		dev_emerg(anhi->dev, "CIO resume failed (%d): the router or its tunnels are not back, and the devices behind them resume next\n",
			  ret);
		panic("Apple CIO: resume failed %d; children remain suspended", ret);
	}
	return 0;
}

static void apple_nhi_cio_complete(struct apple_nhi *anhi)
{
	struct apple_cio *acio = anhi->acio;
	int ret;

	if (!anhi->cio_prepared)
		return;
	if (anhi->cio_t6050) {
		int pcie = anhi->cio_pcie ? apple_pcie_tunnel_cio_finish(anhi->cio_pcie) : 0;
		int dart = apple_dart_cio_finish(anhi->cio_dart);
		int phy = apple_nhi_cio_phy(anhi, CIO_PHY_FINISH);

		/* Everything has resumed: a transaction that does not end is said, not fatal. */
		if (pcie || dart || phy)
			dev_err(anhi->dev, "CIO complete: transactions not ended (PCIe-C %d, DART %d, PHY %d): the port may need its cable plugged in again\n",
				pcie, dart, phy);
		ret = 0;
	} else {
		ret = (anhi->cio_pcie ? apple_pcie_tunnel_cio_finish(anhi->cio_pcie) : 0) ?:
		      apple_dart_cio_finish(anhi->cio_dart) ?:
		      apple_nhi_cio_phy(anhi, CIO_PHY_FINISH);
	}
	if (ret)
		panic("Apple CIO: incomplete recovery at complete: %d", ret);
	tb_cio_sleep_complete(anhi->tb);
	put_device(anhi->cio_pcie);
	anhi->cio_pcie = NULL;
	anhi->cio_prepared = false;
	scoped_guard(mutex, &acio->lock) {
		WRITE_ONCE(acio->native_sleep, false);
		if (acio->target_cable_info != acio->current_cable_info)
			dev_warn(acio->dev, "cable changed during native sleep; revalidation required\n");
	}
	if (anhi->dp_aux_armed)
		mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work, 0);
}
