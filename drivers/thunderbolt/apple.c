// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon USB4 and Thunderbolt driver
 * Copyright (c) Sven Peter <sven@kernel.org>
 *
 * This driver implements the Host Router / ACIO coprocessor as well as the
 * Native Host Interface (NHI) as shown in the diagram below.
 * The ACIO is a Cortex-M3 coprocessor which handles the Thunderbolt
 * protocol and exposes its own hardware blocks like the USB4 Native Host
 * Interface (NHI) and its IOMMU to the main SoC bus. The entire block
 * can only be brought up after the unified Type-C PHY has been initialized
 * to Thunderbolt or USB4 mode and needs an out-of-band notification from
 * the Type-C PD driver.
 *
 * +--------------------+
 * |                    |  +--------------------------------------------+
 * | Display Controller |  | Host Router / ACIO                         |
 * |       dcpext0      |  |             +----------------+             |
 * |                    |  |             |     Native     |             |
 * +--------+-----------+  |             |      Host      |             |
 *          |              |             |    Interface   |             |
 *          |              +---------+   +----------------+   +---------+     +--------+
 *          |   +--------->| DP IN   |                        | PCIE DN |<--->| APCIEC |
 *          v   |          | Adapter |   +----------------+   | Adapter |     +--------+
 *    +---------+-+        +---------+   |                |   +---------+
 *    | Display   |        |             |  IOMMU / DART  |             |
 *    | Crossbar  |        |             |                |   +---------+     +------+
 *    +---------+-+        +---------+   +----------------+   | USB3    |<--->| DWC3 |
 *          ^   |          | DP IN   |                        | Adapter |     +------+
 *          |   +--------->| Adapter |                        +---------+
 *          |              +---------+                                  |
 *          |              |                                            |
 *          |              |                 +----------+               |
 * +--------+-----------+  |                 | Type C   |               |
 * |                    |  |                 | Adapter  |               |
 * | Display Controller |  +-----------------+----------+---------------+
 * |       dcpext1      |                     ^       ^
 * |                    |                     |       |
 * +--------------------+                     |       |
 *                                            v       |
 *                                 +--------------+   | SBRX/TX
 *                                 | Apple Type-C |   |
 *                                 |    PHY       |   |
 *                                 +--------------+   |
 *                                       ^            |
 *                                       |            v
 *                                       |     +---------+
 *                                       +---->| Type C  |
 *                                     SSRX/TX | Port    |
 *                                             +---------+
 *
 * ACIO and ATCPHY have strict ordering requirements. When a USB4/Thunderbolt
 * connection is requested after a device has been connected the Type-C PD
 * driver performs the following sequence:
 *
 * 1) Configure the PHY for the negotiated mode and orientation through
 *    typec_mux_set()
 * 2) Pass the cable details to this driver through typec_thunderbolt_switch_set()
 * 3) Power up ACIO, boot its RTKit co-processor and apply the tunables
 * 4) Probe the DART and NHI children now that their MMIO is accessible from
 *    the main SoC bus
 * 5) Register the USB4 domain and write the cable details to the host router
 *    which brings up the link and starts discovery
 *
 * The reverse order is required when the cable is disconnected then:
 *
 * 1) typec_thunderbolt_switch_set() removes the USB4 domain and the ACIO
 *    children, shuts down the co-processor and powers everything off.
 * 2) typec_mux_set() puts the PHY into safe mode and powers it off.
 *
 * ACIO must not be started before the PHY is configured and the PHY must
 * remain active until ACIO has been powered down again. The child devices
 * cannot  remain populated while ACIO is powered off because they are part
 * of the ACIO block and only exposed to the main SoC bus and will SError
 * once ACIO is off.
 * Violating this ordering can panic the kernel with an async SError at best
 * and trigger some internal watchdog that will reset the entire SoC at worst.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/notifier.h>
#include <linux/pci-apple.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>
#include <linux/phy/apple-atc.h>
#include <linux/soc/apple/dart-cio.h>
#include <linux/soc/apple/dcp-v27-sleep.h>
#include <linux/soc/apple/pmgr-pwrstate.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/tunable.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb/pd.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>
#include <linux/workqueue.h>

#include "apple-dpin-handshake.h"
#include "apple-dpin-state.h"
#include "apple-dpin-request.h"
#include "apple-dpin-platform.h"
#include "nhi.h"
#include "tb.h"

#define APPLE_CIO_M3_CTRL 0x0c
#define APPLE_CIO_M3_CTRL_START BIT(1)
#define APPLE_CIO_LSTX_CTRL_LEVEL BIT(0)
#define APPLE_CIO_LSTX_CTRL_ENABLE BIT(1)
#define APPLE_CIO_M3_STAT 0xa8
#define APPLE_CIO_M3_STAT_STATE GENMASK(30, 24)
#define APPLE_CIO_M3_STAT_STATE_READY 1
#define APPLE_CIO_M3_STAT_CODE_VALID BIT(31)
#define APPLE_CIO_M3_STAT_CODE GENMASK(23, 0)

/* MxWrap coprocessor wrapper (t6030 "cpu" region) */
#define APPLE_MXWRAP_CPU_CTRL 0x28
#define APPLE_MXWRAP_CPU_CTRL_RUN BIT(4)
#define APPLE_MXWRAP_CPU_STATUS 0x44

/* DP IN adapter blocks a host router may have (T6050 has four) */
#define APPLE_DPIN_MAX 4

struct apple_cio_hw {
	/* DP IN adapter blocks of the router; 0 means the two of t8103 */
	unsigned int dpin_count;
	bool mxwrap;
	bool lstx_explicit;
	/* Leave MxWrap RUN alone: the CIO reconfig restarts the CPU. */
	bool skip_run;
	/* Hand LSTX to the router after RTKit boot, not before it. */
	bool lstx_after_boot;
	bool strict_fw_ready;
	/* Keep the links of the host router out of CL states. */
	bool no_host_clx;
	/* Keep the domain's links symmetric. */
	bool no_asym;
	/* Type 7 router: Enter_USB cable words, bootloader RC tuning, Type 7 NHI. */
	bool type7;
	/*
	 * Release the CPU with a plain store of the RUN value rather than a
	 * read-modify-write, and halt it again when the router stops.
	 */
	bool run_store;
	/* Hold LSTX low as a GPIO from probe until the router first takes it. */
	bool lstx_hold_from_probe;
	/* The router trains Gen4 only on a cable word that says Gen4. */
	bool gen4_cable;
	/* Run Gen4 links with link recovery enabled at both ends. */
	bool gen4_link_recovery;
	/* Leave the host router's TMU to its firmware; host TMU constants. */
	bool host_tmu;
	const struct apple_dpin_policy *dp;
};

static const struct apple_cio_hw apple_cio_t8103_hw = {
	.dp = &apple_dpin_m1,
};

static const struct apple_cio_hw apple_cio_t6020_hw = {
	.dp = &apple_dpin_m2,
};

static const struct apple_cio_hw apple_cio_t6030_hw = {
	.mxwrap = true,
	.lstx_explicit = true,
	.strict_fw_ready = true,
	.dp = &apple_dpin_m3,
};

/*
 * T8132 has the T6030 ACIO layout and MxWrap CPU block, but the CIO reconfig
 * already starts the CPU from its preloaded SRAM: RUN is never written. LSTX
 * is handed over once RTKit is up, followed by the LSTX pin. Its links can
 * retrain shortly after CL states are enabled, so they stay in CL0. Its
 * DP IN adapters take the M4 policy, see apple_dpin_m4.
 */
static const struct apple_cio_hw apple_cio_t8132_hw = {
	.mxwrap = true,
	.lstx_explicit = true,
	.skip_run = true,
	.lstx_after_boot = true,
	.strict_fw_ready = true,
	.no_host_clx = true,
	.dp = &apple_dpin_m4,
};

/* Type 7 uses its existing ready-state check and has no qualified DP route. */
static const struct apple_cio_hw apple_cio_t8142_hw = {
	.type7 = true,
	.dp = &apple_dpin_disabled,
};

/*
 * T6050 is Type 7 with an MxWrap CPU wrapper that is only clocked once the
 * CIO reset (the reconfig request) has run. Its CPU is released with a plain
 * store of RUN; as on T8132 it takes LSTX over only once its RTKit is up, and
 * the AP holds the pin while the router is off. Its four DP IN adapters take
 * the M5 policy, see apple_dpin_m5. A Gen4 link times out CL training once CL
 * states are enabled, then drops; it also dropped in the middle of a
 * transition to an asymmetric width. Its links stay in CL0 and symmetric.
 */
static const struct apple_cio_hw apple_cio_t6050_hw = {
	.dpin_count = 4,
	.mxwrap = true,
	.type7 = true,
	.run_store = true,
	.lstx_after_boot = true,
	.lstx_hold_from_probe = true,
	.strict_fw_ready = true,
	.gen4_cable = true,
	.no_host_clx = true,
	.no_asym = true,
	.gen4_link_recovery = true,
	.host_tmu = true,
	.dp = &apple_dpin_m5,
};

static const struct apple_cio_hw apple_cio_t6040_hw = {
	.mxwrap = true,
	.type7 = true,
	.lstx_explicit = true,
	.skip_run = true,
	.lstx_after_boot = true,
	.strict_fw_ready = true,
	.no_host_clx = true,
	.dp = &apple_dpin_disabled,
};

#define APPLE_CIO_NHI_HOP_COUNT 0x0
#define APPLE_CIO_NHI_HOP_COUNT_MASK GENMASK(9, 0)

#define APPLE_CIO_NHI_TXRING_DESC_BASE 0x10000
#define APPLE_CIO_NHI_RXRING_DESC_BASE 0x80000
#define APPLE_CIO_NHI_RING_STRIDE 0x4000

#define APPLE_CIO_NHI_PDF_STRIDE 0x4000

#define APPLE_CIO_NHI_IRQ_STATUS 0xd0000
#define APPLE_CIO_NHI_IRQ_ENABLE 0xd0010
#define APPLE_CIO_NHI_IRQ_THROTTLE 0xd004c
#define APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK GENMASK(15, 0)
#define APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC 256

#define APPLE_CIO_SRAM_IOVA_BASE 0x10000000

#define APPLE_CIO_NHI_BOOT_TIMEOUT_MS 10000
#define APPLE_CIO_PCIEC_READY_DELAY_MS 300

/* Observation only: no extra control requests or downstream enumeration. */
#define APPLE_CIO_RESUME_LINK_WAIT_MS 90000
#define APPLE_CIO_RESUME_LINK_CHECK_MS 5000
#define APPLE_CIO_SLEEP_RESTART_FALLBACK_MS 10000
/* How long a forced cable cycle may take before it no longer counts. */
#define APPLE_CIO_RESUME_RECOVERY_WINDOW_MS 30000
/*
 * After a peer drops the link without the cable going away, the ACIO
 * firmware parks the port and waits for the host to power it off. A peer
 * that reboots or goes away electrically is reported as a cable change
 * within about 0.5 s and restarts the port anyway.
 */
#define APPLE_CIO_LINK_LOST_MS 3000
#define APPLE_CIO_LINK_LOST_CYCLES 3

static bool resume_recovery;
module_param(resume_recovery, bool, 0644);
MODULE_PARM_DESC(resume_recovery,
		 "Recover missing post-resume discovery once per driver load (default N)");
/* Shared by all ACIO ports; never refilled by suspend or cable changes. */
static atomic_t resume_recovery_used = ATOMIC_INIT(0);

/*
 * A connected port keeps its link trained through s2idle, which costs about
 * 1.3 W on t6000. Powering ACIO off for the sleep saves that, but drops every
 * tunnel: devices behind them are removed and rediscovered after resume.
 */
static bool s2idle_teardown;
module_param(s2idle_teardown, bool, 0644);
MODULE_PARM_DESC(s2idle_teardown,
		 "Power off connected ports for system sleep and restart them on resume (default N; drops tunnels, including USB devices tunnelled through a USB4 dock: unmount their storage first; ports with PCIe tunnels are skipped)");

/*
 * T6050: a firmware sleep takes the routers' power, and its sleep driver refuses one while a
 * router's power word is on. macOS shuts a connected port's router down for its sleep: the
 * router processor's power-off handshake, then the port's CIO power domains, and the reverse
 * after the wake. The port's power-off above does the same through the driver's own stop
 * (the tunnels released while the display processors still run, the processor's power-off
 * handshake, the domains released), and the restart after the sleep brings the dock and its
 * displays back as newly attached. Default for a T6050 firmware sleep; a port with a PCIe
 * tunnel is skipped as above, and the sleep is then refused with its router still on.
 */
static bool t6050_sleep_teardown = true;
module_param(t6050_sleep_teardown, bool, 0644);
MODULE_PARM_DESC(t6050_sleep_teardown,
		 "T6050: power off connected ports for a firmware sleep and restart them after the wake, as macOS does (default Y; N: the routers stay up and the sleep is refused)");

/*
 * With t6050_sleep_teardown, a port whose dock carries a PCIe tunnel (storage behind the
 * dock). 1: the tunnelled functions are removed in order before the port goes down, the
 * way a power-off stops them (an NVMe controller gets its normal shutdown), if they are NVMe
 * controllers and their PCIe ports only and no disk of theirs is open; otherwise the port
 * stays up and the sleep is refused. They are found again after the wake. 2: also with a
 * disk open: the block layer syncs and shuts down what is on it, and a filesystem mounted
 * from it has to be mounted again after the wake. 0: such a port is always skipped.
 */
static int t6050_sleep_pcie = 1;
module_param(t6050_sleep_pcie, int, 0644);
MODULE_PARM_DESC(t6050_sleep_pcie,
		 "T6050: PCIe devices tunnelled through a dock at a firmware sleep: 0 keep the port up (sleep refused), 1 remove unused NVMe storage in order (default), 2 also storage in use");

/*
 * TEST, from a macOS sleep and wake with a dock on the right port (two tunnelled displays, a
 * PCIe and a USB tunnel). Two things macOS has that the port's power-off above did not:
 * the router's LSTX line is low across the sleep (it logs the level it leaves, 0), where the
 * stop keeps the level the router last drove while the cable is in; and the router is
 * started again at once after the wake (its processor is up 0.21 s after it), where the
 * restart waited for the port controller's own revalidation and otherwise ten seconds.
 */
/*
 * sv6r3, the first sleep and wake with this (Y): the router started from cold after the wake
 * and its lane initialisation got no answer from the retimer ("Timeout for at_rt command
 * response", register 12, every 100 ms; at the boot the same read is answered at once), and
 * the dock never came back. macOS's low LSTX follows its router's own sleep (lane
 * initialisation action 4, "ACIO SLEEP process completed"), and its wake is that firmware's
 * (action 5, "Waking up from sleep!"), not a cold start; the port's power-off here has
 * neither, so a low line for the whole sleep is a pulled cable to the retimer and the dock.
 * Default N again: the level the router last drove, as every other stop of a port with its
 * cable in. The router's sleep in place is where the low line belongs.
 */
static bool t6050_sleep_lstx_low;
module_param(t6050_sleep_lstx_low, bool, 0644);
MODULE_PARM_DESC(t6050_sleep_lstx_low,
		 "T6050: hold LSTX low while a port is powered off for a firmware sleep (default N: the level the router last drove; Y lost the dock after the wake, sv6r3)");

static unsigned int t6050_sleep_restart_ms;
module_param(t6050_sleep_restart_ms, uint, 0644);
MODULE_PARM_DESC(t6050_sleep_restart_ms,
		 "T6050: delay from the end of a firmware sleep's resume to the restart of a port powered off for it (default 0, as macOS; 10000 was the fallback)");

/*
 * TEST, macOS's order in the same trace: the processors of the dock's displays are idle 0.23 s
 * before the router's paths for those displays are destroyed and the router is put to sleep.
 * The port's power-off took the router's domain down first, and the displays were released
 * from its teardown, while their paths went. With this the displays behind the port are
 * released first, by the request a tunnel's own end makes, and the router goes once the
 * display side has answered. A release that does not end in time leaves the port up and
 * refuses the sleep.
 */
static bool t6050_sleep_dp_first = true;
module_param(t6050_sleep_dp_first, bool, 0644);
MODULE_PARM_DESC(t6050_sleep_dp_first,
		 "T6050: release the displays behind a port before its router is powered off for a firmware sleep, as macOS does (default Y; N: from the router's teardown)");

/*
 * TEST, the same order across ports: macOS has every display processor idle before the first
 * port loses anything (the dock's displays and the one on the other port alike). The release
 * above ran from each port's own power-off, so a display behind a second dock went after the
 * first port was off. With this every port releases its displays from a notifier that runs
 * before all the power-offs (and before the USB hosts' stops and the PCIe ports' parks), and
 * no tunnel offered from then on is handed to a display processor until the sleep is over or
 * given up. A sleep given up with displays released and the port still up restarts the port,
 * which brings them back as after a sleep.
 */
static bool t6050_sleep_dp_all_first = true;
module_param(t6050_sleep_dp_all_first, bool, 0644);
MODULE_PARM_DESC(t6050_sleep_dp_all_first,
		 "T6050: release the displays behind every port before any port is powered off for a firmware sleep, and hand no new tunnel to a display until it is over (default Y; N: each port's displays at its own power-off)");

#define APPLE_CIO_START_RETRIES 5
#define APPLE_CIO_START_RETRY_DELAY_MS 100

#define APPLE_CIO_PCIEC_INTR2AXI_CTRL 0x80
#define APPLE_CIO_PCIEC_INTR2AXI_ENABLE BIT(0)

/*
 * The Apple vendor-specific extended capability contains a cable-information
 * word which has to be programmed from the USB4/Thunderbolt details reported
 * out-of-band by the Type-C PD controller. It must be programmed into the
 * host router before link discovery starts.
 */
#define TB_VSE_CAP_APPLE 0x00
#define TB_VSE_CAP_APPLE_CABLE_INFO 0x01
#define TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT BIT(0)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE BIT(1)
#define TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE BIT(2)
#define TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX BIT(3)
#define TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS BIT(4)
#define TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER BIT(9)
#define TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3 BIT(10)
#define TB_VSE_CAP_APPLE_CABLE_INFO_GEN4 BIT(11)

/* Enter_USB cable speed: USB PD 3.2 adds USB4 Gen4 after Gen3. */
#define APPLE_EUDO_CABLE_SPEED_USB4_GEN4 4

static unsigned int dpin_mode_value = 8;
module_param(dpin_mode_value, uint, 0644);
MODULE_PARM_DESC(dpin_mode_value,
		 "DPIN0 MODE_A/MODE_B lab value (0-" __stringify(APPLE_DPIN_MODE_VALUE_MAX) "); read at each activate");

/* DP IN adapters without an interrupt are polled this often (ms). */
#define APPLE_DPIN_POLL_MS 20

static bool cio_s2r = true;
module_param(cio_s2r, bool, 0400);
MODULE_PARM_DESC(cio_s2r, "Enable J713 attached-dock system sleep (default: on)");

struct apple_dpin_binding;

/**
 * struct apple_cio - Apple Converged I/O host router complex
 * @dev: ACIO device
 * @np: ACIO device tree node
 * @rtk: RTKit instance for the ACIO co-processor
 * @rc_base: ACIO root controller registers
 * @rc_res: ACIO root controller MMIO resource
 * @rc_tunable: Tunable sequence for the ACIO root controller
 * @sram_res: ACIO co-processor SRAM resource
 * @sram_base: ACIO co-processor SRAM
 * @reset: ACIO reset controller
 * @pd_list: Power domains used by the ACIO complex
 * @lock: Serializes cable transitions and ACIO power changes
 * @current_cable_info: Cable information currently programmed into ACIO
 * @target_cable_info: Cable information requested by the Type-C PD driver
 * @nhi_boot_completion: Signals completion of the NHI bringup
 * @nhi_boot_status: Status returned by the NHI bringup
 * @tbt_switch: USB4/Thunderbolt switch triggered by the Type-C PD driver
 */
struct apple_cio {
	struct device *dev;
	bool type7;
	struct device_node *np;
	const struct apple_cio_hw *hw;
	const struct apple_dpin_policy *dp;
	struct pinctrl *pinctrl;
	struct pinctrl_state *lstx_pins;
	struct gpio_desc *lstx_gpio;
	bool lstx_hold;
	bool lstx_high;
	void __iomem *cpu_base;
	struct device_node *pcie_tunnel_np;
	bool pcie_tunnel_preinitialized;
	struct reset_control *pcie_reset;
	void __iomem *pcie_intr2axi_base;
	struct apple_rtkit *rtk;

	void __iomem *rc_base;
	struct resource *rc_res;
	struct apple_tunable *rc_tunable;
	void __iomem *pcie_adapter_base;
	struct apple_tunable *pcie_adapter_tunable;
	void __iomem *router_ap_fw_base;
	struct apple_tunable *router_ap_fw_tunable;
	/* A run_store router CPU that apple_cio_start() released */
	bool cpu_running;

	struct resource *sram_res;
	void __iomem *sram_base;

	struct reset_control *reset;

	struct dev_pm_domain_list *pd_list;

	struct mutex lock; /* serializes cable transitions and ACIO power up/down */
	bool removing; /* protected by lock */
	bool native_sleep; /* freeze cable transitions for the bound NHI */
	struct delayed_work resume_link_work;
	unsigned int resume_link_failures; /* protected by lock */
	u64 resume_activity; /* snapshot before the stock resume path */
	bool resume_had_peer; /* peer present before suspend */
	bool resume_recovery_active; /* forced cable cycle is in progress */
	unsigned long resume_recovery_until; /* jiffies: the cycle is over after */
	bool resume_recovery_failed; /* terminal recovery failure */
	bool resume_link_failed; /* protected by lock */
	bool resume_link_pending; /* set by NHI resume, consumed by ACIO complete */
	struct delayed_work link_lost_work;
	unsigned int start_gen; /* protected by lock: bumped by every start */
	unsigned int link_lost_gen; /* start_gen when the peer went away */
	unsigned int link_lost_cycles; /* protected by lock */
	u64 link_lost_cycle_activity; /* remote activity at the last cycle */
	bool link_lost_restart; /* protected by lock: cycle requested */
	struct notifier_block pm_notifier;
	/*
	 * T6050: the displays behind this port go at the start of the sleep's entry, before any
	 * port's power-off. @sleep_dp_paused: this entry closed the DP IN admission;
	 * @sleep_dp_released: displays it released, with the port still up; @sleep_dp_undo: the
	 * sleep was given up with the port as that, and its restart is owed. Protected by lock.
	 */
	struct notifier_block pm_notifier_dp;
	struct work_struct sleep_dp_undo_work;
	/* A dock or host was on this port when it was powered off for a sleep: expected back. */
	bool sleep_peer_wanted;
	bool sleep_dp_paused;
	unsigned int sleep_dp_released;
	bool sleep_dp_undo;
	struct delayed_work sleep_restart_work;
	enum {
		APPLE_CIO_SLEEP_OPEN,		/* cable changes apply normally */
		APPLE_CIO_SLEEP_CLOSED,		/* torn down for sleep: changes recorded only */
		APPLE_CIO_SLEEP_RESUMED,	/* next cable state is a post-resume snapshot */
	} sleep_phase; /* protected by lock */
	bool sleep_restart_owed; /* protected by lock */
	bool sleep_t6050; /* powered off for a T6050 firmware sleep; protected by lock */
	bool sleep_in_place; /* T6050: the bound NHI sleeps the router in place */
	/* TEST: what the router firmware's log said last, for the sleep in place's lines */
	atomic_t fw_lane_action;	/* "Entering Phase 1. action=N", -1 before the first */
	atomic_t fw_sleep_done;		/* "ACIO SLEEP process completed" */
	atomic_t fw_wake_seen;		/* "Waking up from sleep!" */
	atomic_t fw_at_answers;		/* "Got AT register 12" */
	atomic_t fw_at_timeouts;	/* "Timeout for at_rt command response" */
	bool sleep_aborted_early; /* protected by lock: devices never suspended */

	u32 current_cable_info;
	u32 target_cable_info;
	struct completion nhi_boot_completion;
	int nhi_boot_status;
	struct platform_device *nhi_pdev;

	struct typec_thunderbolt_switch_dev *tbt_switch;
	struct notifier_block pcie_notifier;
	/* Serializes PCIe-C population with cable teardown. */
	struct mutex pcie_tunnel_lock;
	struct delayed_work pcie_tunnel_work;
	bool pcie_tunnel_requested;
	bool pcie_tunnel_populated;
	unsigned int pcie_bind_retries; /* protected by pcie_tunnel_lock */
	bool pcie_tunnel_stopping;
	bool pcie_pm_prepared;
	bool pcie_resume_expected;
	bool pcie_quiesce_pending;

	/* Type-C connector this router is wired to, for DP tunnel routing */
	struct device_node *connector_np;
	bool dp_dual_stream;
	unsigned int dpin_count;
	struct workqueue_struct *dp_wq;
	struct apple_dpin_ctx {
		struct apple_cio *acio;
		struct work_struct work;
		struct delayed_work retry;	/* re-queues work while waiting */
		struct mutex lock;	/* protects regs, alive, waiting, paused */
		void __iomem *regs;	/* mapped while a DP tunnel uses this adapter */
		unsigned int idx;
		struct apple_dpin_state state;
		struct apple_dpin_tokens tokens;
		unsigned long settle_until;	/* new tunnel's admission deadline */
		struct apple_dpin_binding *binding; /* worker-owned, handed to DCP */
		bool dcp_touched;	/* DCP ran the handshake on this mapping */
		bool dcp_active;	/* DCP activated the link (DPTX_INACTIVE clear) */
		/* DP IN interrupts: the line, or a poll where there is none */
		int irq;
		bool irq_enabled;
		struct delayed_work poll;
		struct work_struct event_work;	/* passes events on, on dp_wq */
		unsigned int events;	/* APPLE_DP_TUNNEL_*, under lock */
		bool hpd;		/* HPD level last seen, under lock */
		unsigned int irq_count;
	} dpin[APPLE_DPIN_MAX];
};

/* Immutable identity for one callback handoff; freed only after provider drain. */
struct apple_dpin_binding {
	struct apple_dpin_ctx *ctx;
	u64 generation;
	typeof(apple_dcp_tb_dp_tunnel) *tunnel; /* owns the provider module reference */
};

/*
 * The ATC's DP IN adapter blocks ("atcN-dpin0..3" in the Apple device tree).
 * Registers on t8103:
 *   0x00 status, bit 2 = HPD level
 *   0x04 interrupt status (write 1 to clear)
 *   0x08 interrupt enable [1:0]
 *   0x0c bit 0 DPTX_INACTIVE
 *   0x10 bit 0 DPTX_INACTIVE_ACK
 * The blocks sit 0x390000 after the ACIO "rc" region: dpin1 0x8000 after
 * dpin0, and on routers with four, dpin2 and dpin3 0x18000 and 0x20000 after.
 */
#define APPLE_DPIN_OFFSET		0x390000
#define APPLE_DPIN_SIZE			0x4000

static const u32 apple_dpin_offsets[APPLE_DPIN_MAX] = {
	0x0, 0x8000, 0x18000, 0x20000,
};
#define APPLE_DPIN_STATUS		0x00
#define APPLE_DPIN_STATUS_HPD		BIT(2)
#define APPLE_DPIN_IRQ_STATUS		0x04
#define APPLE_DPIN_IRQ_HPD_CHANGE	BIT(0)
#define APPLE_DPIN_IRQ_SINK		BIT(1)
#define APPLE_DPIN_IRQ_ENABLE		0x08
#define APPLE_DPIN_CTRL			0x0c
#define APPLE_DPIN_CTRL_INACTIVE	BIT(0)
#define APPLE_DPIN_ACK_INACTIVE		BIT(0)

static bool dp_display = true;
module_param(dp_display, bool, 0444);
MODULE_PARM_DESC(dp_display, "Drive displays behind qualified Thunderbolt DP tunnels (default: true)");

/*
 * The M1 Pro/Max ATC is the t8103 generation: same DP IN adapter registers,
 * same crossbar and the same tunnel pixel clock sequence.
 */
static const struct of_device_id apple_dpin_qualified_soc[] = {
	{ .compatible = "apple,t8103" },
	/* The M2 (t8112) ACIO, NHI and display crossbar are t8103-compatible. */
	{ .compatible = "apple,t8112" },
	{ .compatible = "apple,t6000" },
	{ .compatible = "apple,t6001" },
	{ .compatible = "apple,t6020" },
	{ .compatible = "apple,t6021" },
	{ .compatible = "apple,t6030" },
	{ .compatible = "apple,t8132" },
	/* T6050: t8103 DP IN handshake, M3 flow, four DP IN adapters */
	{ .compatible = "apple,t6050" },
	{},
};

const struct apple_dpin_policy *
apple_dpin_policy_select(const struct apple_dpin_policy *hw,
			 const struct device_node *root, bool dual_stream)
{
	if (!of_match_node(apple_dpin_qualified_soc, root))
		return &apple_dpin_disabled;
	if (hw->flow == APPLE_DPIN_PRE_POST && !dual_stream)
		return &apple_dpin_disabled;
	return hw;
}

/* DPTX_INACTIVE handshake: request (in)active, wait for the ACK */
struct apple_dpin_poll {
	void __iomem *base;
	unsigned long deadline;
};

static unsigned int apple_dpin_read(void *ctx, unsigned int offset)
{
	struct apple_dpin_poll *poll = ctx;

	return readl(poll->base + offset);
}

static void apple_dpin_write(void *ctx, unsigned int offset, unsigned int value)
{
	struct apple_dpin_poll *poll = ctx;

	writel(value, poll->base + offset);
}

static int apple_dpin_wait(void *ctx)
{
	struct apple_dpin_poll *poll = ctx;

	if (time_after_eq(jiffies, poll->deadline))
		return -ETIMEDOUT;
	usleep_range(1000, 2000);
	return 0;
}

static int apple_dpin_set_active_t602x(struct apple_cio *acio, void __iomem *regs,
				 unsigned int idx, bool active)
{
	struct apple_dpin_poll poll = {
		.base = regs,
		.deadline = jiffies + msecs_to_jiffies(1000),
	};
	struct apple_dpin_io io = {
		.read = apple_dpin_read,
		.write = apple_dpin_write,
		.wait = apple_dpin_wait,
		.ctx = &poll,
	};
	int ret;

	ret = apple_dpin_handshake(&io, active, dpin_mode_value);
	dev_info(acio->dev, "dpin%u: %s handshake=%d\n", idx,
		 active ? "active" : "inactive", ret);
	return ret;
}

static int apple_dpin_set_active_t8103(struct apple_cio *acio, void __iomem *dpin,
				 unsigned int idx, bool active)
{
	u32 want = active ? 0 : APPLE_DPIN_ACK_INACTIVE;
	unsigned long deadline = jiffies + msecs_to_jiffies(500);
	u32 st = 0, ctrl, ack;

	/* one 500 ms budget for the whole handshake: DCP waits on this */
	if (active) {
		while (!((st = readl(dpin + APPLE_DPIN_STATUS)) & APPLE_DPIN_STATUS_HPD)) {
			if (time_after(jiffies, deadline)) {
				dev_warn(acio->dev, "dpin%u: no HPD (status=%08x)\n",
					 idx, st);
				return -ETIMEDOUT;
			}
			usleep_range(1000, 1200);
		}
	}

	ctrl = readl(dpin + APPLE_DPIN_CTRL);
	ack = readl(dpin + APPLE_DPIN_ACK);
	if (!!(ctrl & APPLE_DPIN_CTRL_INACTIVE) == !active &&
	    (ack & APPLE_DPIN_ACK_INACTIVE) == want)
		return 0;

	if (active)
		ctrl &= ~APPLE_DPIN_CTRL_INACTIVE;
	else
		ctrl |= APPLE_DPIN_CTRL_INACTIVE;
	writel(ctrl, dpin + APPLE_DPIN_CTRL);

	while (((ack = readl(dpin + APPLE_DPIN_ACK)) & APPLE_DPIN_ACK_INACTIVE) != want) {
		if (time_after(jiffies, deadline)) {
			dev_warn(acio->dev, "dpin%u: no DPTX_INACTIVE_ACK=%u (ack=%08x)\n",
				 idx, !active, ack);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1200);
	}
	dev_dbg(acio->dev, "dpin%u: %s (status=%08x)\n", idx,
		 active ? "active" : "inactive", readl(dpin + APPLE_DPIN_STATUS));
	return 0;
}

static int apple_dpin_set_active(struct apple_cio *acio, void __iomem *regs,
				 unsigned int idx, bool active)
{
	if (acio->dp->t602x_handshake)
		return apple_dpin_set_active_t602x(acio, regs, idx, active);
	return apple_dpin_set_active_t8103(acio, regs, idx, active);
}

static void apple_dpin_request_inactive(void *data)
{
	struct apple_dpin_ctx *c = data;

	/* Never woken by DCP: the block is still as the tunnel found it. */
	if (c->acio->dp->lazy_idle && !c->dcp_touched)
		return;
	apple_dpin_set_active(c->acio, c->regs, c->idx, false);
}

/*
 * DP IN interrupts. +0x4 holds the events, write-1-to-clear: bit 0 the HPD
 * level in +0x0 changed (+0x0 latches status bits that are cleared by
 * writing it back), bit 1 the sink raised IRQ_HPD. Read +0x4 until it is
 * stable, clear it, and hand what happened to appledrm: the display's DPTX
 * port has no HPD input of its own on a tunnel. c->lock held.
 */
static unsigned int apple_dpin_irq_collect(struct apple_dpin_ctx *c, bool *seen)
{
	unsigned int events = 0, tries;
	u32 irq, again, st;
	bool hpd;

	lockdep_assert_held(&c->lock);
	*seen = false;
	/* A handler or poll may have waited on the lock while teardown stopped it. */
	if (!c->irq_enabled || !c->state.alive || !c->regs)
		return 0;
	irq = readl(c->regs + APPLE_DPIN_IRQ_STATUS);
	for (tries = 0; tries < 8; tries++) {
		again = readl(c->regs + APPLE_DPIN_IRQ_STATUS);
		if (again == irq)
			break;
		irq = again;
	}
	if (!irq)
		return 0;
	*seen = true;
	writel(irq, c->regs + APPLE_DPIN_IRQ_STATUS);
	st = readl(c->regs + APPLE_DPIN_STATUS);
	if (irq & APPLE_DPIN_IRQ_HPD_CHANGE)
		writel(st, c->regs + APPLE_DPIN_STATUS);
	hpd = st & APPLE_DPIN_STATUS_HPD;
	if (hpd != c->hpd)
		events |= APPLE_DP_TUNNEL_HPD_CHANGED;
	c->hpd = hpd;
	if (irq & APPLE_DPIN_IRQ_SINK)
		events |= APPLE_DP_TUNNEL_IRQ_HPD;
	c->irq_count++;
	dev_dbg(c->acio->dev, "dpin%u: interrupt %#x (#%u), status %#x: HPD %d%s%s\n",
		c->idx, irq, c->irq_count, st, hpd,
		events & APPLE_DP_TUNNEL_IRQ_HPD ? ", IRQ_HPD" : "",
		events & APPLE_DP_TUNNEL_HPD_CHANGED ? ", HPD changed" : "");
	events &= APPLE_DP_TUNNEL_IRQ_HPD | APPLE_DP_TUNNEL_HPD_CHANGED;
	c->events |= events;
	return events;
}

/* Returns false if the adapter had nothing pending. */
static bool apple_dpin_irq_dispatch(struct apple_dpin_ctx *c)
{
	unsigned int events;
	bool seen;

	scoped_guard(mutex, &c->lock)
		events = apple_dpin_irq_collect(c, &seen);
	if (events)
		queue_work(c->acio->dp_wq, &c->event_work);
	return seen;
}

static irqreturn_t apple_dpin_irq_thread(int irq, void *data)
{
	return apple_dpin_irq_dispatch(data) ? IRQ_HANDLED : IRQ_NONE;
}

static void apple_dpin_poll_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(to_delayed_work(work),
						struct apple_dpin_ctx, poll);
	bool mapped;

	apple_dpin_irq_dispatch(c);
	scoped_guard(mutex, &c->lock)
		mapped = c->irq_enabled && c->regs;
	if (mapped)
		queue_delayed_work(system_dfl_wq, &c->poll,
				   msecs_to_jiffies(APPLE_DPIN_POLL_MS));
}

/* Start taking the interrupts of a mapped adapter. c->lock held. */
static void apple_dpin_irq_start(struct apple_dpin_ctx *c)
{
	lockdep_assert_held(&c->lock);
	if (c->irq_enabled || !c->regs)
		return;
	c->hpd = readl(c->regs + APPLE_DPIN_STATUS) & APPLE_DPIN_STATUS_HPD;
	c->events = 0;
	c->irq_enabled = true;
	if (c->irq > 0)
		enable_irq(c->irq);
	else
		queue_delayed_work(system_dfl_wq, &c->poll,
				   msecs_to_jiffies(APPLE_DPIN_POLL_MS));
}

/* The handler takes c->lock: never wait for it here. c->lock held. */
static void apple_dpin_irq_stop(struct apple_dpin_ctx *c)
{
	lockdep_assert_held(&c->lock);
	if (!c->irq_enabled)
		return;
	c->irq_enabled = false;
	c->events = 0;
	if (c->irq > 0)
		disable_irq_nosync(c->irq);
	else
		cancel_delayed_work(&c->poll);
}

/* On dp_wq, ordered with bring-up and teardown: the binding is stable. */
static void apple_dpin_event_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(work, struct apple_dpin_ctx, event_work);
	typeof(apple_dcp_tb_dp_event) *event;
	unsigned int events;
	bool hpd, active;
	int ret;

	scoped_guard(mutex, &c->lock) {
		events = c->events;
		c->events = 0;
		hpd = c->hpd;
		active = c->dcp_active && c->state.alive;
	}
	/* The sink's requests matter once the display has activated the link. */
	if (!active)
		events &= ~APPLE_DP_TUNNEL_IRQ_HPD;
	if (!events || !c->binding)
		return;
	event = symbol_get(apple_dcp_tb_dp_event);
	if (!event)
		return;
	ret = event(c->acio->connector_np, c->idx, c->binding->generation, events, hpd);
	symbol_put(apple_dcp_tb_dp_event);
	if (ret)
		dev_info(c->acio->dev, "dpin%u: display did not take events %#x: %d\n",
			 c->idx, events, ret);
}

static void apple_dpin_request_mask_irqs(void *data)
{
	struct apple_dpin_ctx *c = data;

	/* Nothing to mask where the interrupts were never enabled. */
	if (c->acio->dp->lazy_idle && !c->acio->dp->setup_irqs)
		return;

	writel(readl(c->regs + APPLE_DPIN_IRQ_ENABLE) & ~3,
	       c->regs + APPLE_DPIN_IRQ_ENABLE);
	apple_dpin_irq_stop(c);
}

static bool apple_dpin_request(struct apple_dpin_ctx *c, u64 generation, bool active)
{
	const struct apple_dpin_request_ops ops = {
		.inactive = apple_dpin_request_inactive,
		.mask_irqs = apple_dpin_request_mask_irqs,
		.ctx = c,
	};
	bool replacement = active && c->tokens.requested != generation;

	lockdep_assert_held(&c->lock);
	if (!apple_dpin_powered_request(&c->state, &c->tokens, c->acio->dp,
					generation, active, !!c->regs, &ops))
		return false;
	if (replacement)
		c->settle_until = jiffies + msecs_to_jiffies(c->acio->dp->settle_ms);
	return true;
}

/* Called by appledrm from DCP's Activate/Deactivate calls. */
static int apple_dpin_dcp_set_active(void *data, bool active)
{
	const struct apple_dpin_binding *binding = data;
	struct apple_dpin_ctx *c = binding->ctx;
	int ret;

	guard(mutex)(&c->lock);
	if (!apple_dpin_token_access(&c->tokens, binding->generation))
		return -ESTALE;
	/* the tunnel is being torn down: the block may already be off */
	if (!c->state.alive || !c->regs)
		return -ENODEV;
	c->dcp_touched = true;
	ret = apple_dpin_set_active(c->acio, c->regs, c->idx, active);
	if (!ret)
		c->dcp_active = active;
	return ret;
}

bool apple_dp_tunnel_binding_live(void *data)
{
	const struct apple_dpin_binding *binding = data;
	struct apple_dpin_ctx *c = binding->ctx;

	guard(mutex)(&c->lock);
	return c->state.alive &&
	       apple_dpin_token_access(&c->tokens, binding->generation);
}
EXPORT_SYMBOL_GPL(apple_dp_tunnel_binding_live);

static bool apple_dpin_provider_get(void *data)
{
	struct apple_dpin_binding *binding = data;

	binding->tunnel = symbol_get(apple_dcp_tb_dp_tunnel);
	return !!binding->tunnel;
}

static bool apple_dpin_provider_held(void *data)
{
	struct apple_dpin_binding *binding = data;

	return !!binding->tunnel;
}

static int apple_dpin_provider_invoke(void *data, bool active)
{
	struct apple_dpin_binding *binding = data;
	struct apple_dpin_ctx *c = binding->ctx;

	return binding->tunnel(c->acio->connector_np, c->idx, binding->generation, active,
			       active ? apple_dpin_dcp_set_active : NULL,
			       active ? binding : NULL);
}

static void apple_dpin_provider_put(void *data)
{
	struct apple_dpin_binding *binding = data;

	symbol_put(apple_dcp_tb_dp_tunnel);
	binding->tunnel = NULL;
}

static int apple_dpin_connect(struct apple_dpin_ctx *c,
			      struct apple_dpin_binding *binding,
			      bool active)
{
	struct apple_cio *acio = c->acio;
	const struct apple_dpin_provider_ops ops = {
		.get = apple_dpin_provider_get,
		.held = apple_dpin_provider_held,
		.call = apple_dpin_provider_invoke,
		.put = apple_dpin_provider_put,
		.ctx = binding,
	};
	unsigned int tries;
	bool alive, paused;
	int ret;

	for (tries = 1; ; tries++) {
		/*
		 * up() admitted the first call together with mapping and IRQ setup.
		 * Prepare drains that call; every subsequent attempt needs a new gate.
		 */
		if (active) {
			guard(mutex)(&c->lock);
			if (!c->state.alive || c->tokens.requested != binding->generation)
				return -ESTALE;
			if (apple_dpin_admission_blocked(&c->state, acio->dp, tries == 1))
				return -EAGAIN;
			/* up() admitted the first call; retry admits a fresh attempt. */
			if (tries == 1) {
				if (!apple_dpin_token_access(&c->tokens, binding->generation))
					return -ESTALE;
			} else if (!apple_dpin_token_admit(&c->tokens, binding->generation)) {
				return -ESTALE;
			}
		}
		ret = apple_dpin_provider_call(active, &ops);
		if (active && ret) {
			scoped_guard(mutex, &c->lock)
				apple_dpin_token_revoke(&c->tokens, binding->generation);
		}
		/* ENODEV is driver probing; EAGAIN is a registered external
		 * route whose firmware services have not been published yet.
		 * Neither has handed a callback or changed the display route.
		 * Other failures are terminal for this tunnel event.
		 */
		if (!apple_dpin_readiness_retry(active, ret, tries))
			return ret;

		scoped_guard(mutex, &c->lock) {
			alive = c->state.alive;
			paused = c->state.paused;
		}
		if (!alive)
			return -ENODEV;
		/* not across system sleep: ask again once it is over */
		if (paused)
			return -EAGAIN;
		if (tries == 1)
			dev_info(acio->dev, "dpin%u: waiting for display driver/firmware readiness\n",
				 c->idx);
		msleep(APPLE_DP_CONNECT_WAIT_MS);
	}
}

/*
 * A USB4 dock can withdraw and re-advertise its DP OUT adapters while the
 * initial capability exchange completes. On T6050, starting DCP on the
 * temporary tunnel can leave its AUX request stuck after that tunnel goes
 * away. Let the request settle before admitting any DCP callback or touching
 * the adapter. Teardown still idles immediately in apple_dpin_request().
 * A replacement owns a new deadline; this worker never admits the old token.
 */
static int apple_dpin_wait_settled(struct apple_dpin_ctx *c, u64 generation)
{
	unsigned long delay, now;

	if (!c->acio->dp->settle_ms)
		return 0;
	for (;;) {
		scoped_guard(mutex, &c->lock) {
			if (!c->state.alive || c->tokens.requested != generation)
				return -ESTALE;
			now = jiffies;
			if (time_after_eq(now, c->settle_until))
				return 0;
			delay = c->settle_until - now;
		}
		msleep(jiffies_to_msecs(delay) + 1);
	}
}

static int apple_dpin_up(struct apple_dpin_ctx *c,
			 struct apple_dpin_binding *binding, bool *deferred)
{
	struct apple_cio *acio = c->acio;
	void __iomem *regs = c->regs;
	int ret;
	u32 irq;

	*deferred = false;
	ret = apple_dpin_wait_settled(c, binding->generation);
	if (ret)
		return ret;
	scoped_guard(mutex, &c->lock) {
		if (!c->state.alive || c->tokens.requested != binding->generation)
			return -ESTALE;
		if (apple_dpin_admission_blocked(&c->state, acio->dp, false)) {
			*deferred = true;
			return -EAGAIN;
		}
		/* This admission also owns the first DCP call after dropping the lock. */
		if (!regs) {
			regs = ioremap_np(acio->rc_res->start + APPLE_DPIN_OFFSET +
					  apple_dpin_offsets[c->idx], APPLE_DPIN_SIZE);
			if (!regs)
				return -ENOMEM;
			c->regs = regs;
		}
		/*
		 * Acknowledge what is pending and enable the DP IN interrupts:
		 * +0x4 is write-1-to-clear, and a plug event (bit 0) also
		 * latches status bits in +0x0 that are cleared by writing it
		 * back. See apple_dpin_irq_collect() for what they report.
		 */
		if (acio->dp->setup_irqs) {
			irq = readl(regs + APPLE_DPIN_IRQ_STATUS);
			writel(irq, regs + APPLE_DPIN_IRQ_STATUS);
			if (irq & BIT(0))
				writel(readl(regs + APPLE_DPIN_STATUS), regs + APPLE_DPIN_STATUS);
			writel(readl(regs + APPLE_DPIN_IRQ_ENABLE) | 3, regs + APPLE_DPIN_IRQ_ENABLE);
			apple_dpin_irq_start(c);
		}
		/* Activate may run before attach returns, so admit its immutable cookie now. */
		apple_dpin_token_admit(&c->tokens, binding->generation);
		apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_ADMITTED, true, 0);
	}

	return apple_dpin_connect(c, binding, true);
}

static void apple_dpin_release_binding(struct apple_dpin_binding *binding)
{
	struct apple_dpin_ctx *c = binding->ctx;
	int ret;

	/* Admission is revoked before the provider retires and drains this token. */
	scoped_guard(mutex, &c->lock)
		apple_dpin_token_revoke(&c->tokens, binding->generation);
	ret = apple_dpin_connect(c, binding, false);
	if (ret && ret != -ESTALE)
		dev_warn(c->acio->dev, "dpin%u: display teardown failed: %d\n",
			 c->idx, ret);
	kfree(binding);
}

static void apple_dpin_down(struct apple_dpin_ctx *c)
{
	void __iomem *regs;

	if (c->binding) {
		apple_dpin_release_binding(c->binding);
		c->binding = NULL;
	}
	/* The powered request hook idled the adapter before queued teardown. */
	scoped_guard(mutex, &c->lock) {
		apple_dpin_irq_stop(c);
		regs = c->regs;
		c->regs = NULL;
		c->dcp_touched = false;
		c->dcp_active = false;
		apple_dpin_step(&c->state, c->acio->dp, APPLE_DPIN_DROPPED, false, 0);
	}
	if (regs)
		iounmap(regs);
}

/*
 * Bring the display side in line with the tunnel state. One work item per
 * adapter on an ordered queue, so events are never lost and nothing has to be
 * allocated on the way down: whatever was handed to appledrm is taken back.
 */
static void apple_dpin_work_fn(struct work_struct *work);

static void apple_dpin_retry_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(to_delayed_work(work),
						struct apple_dpin_ctx, retry);
	unsigned int actions;

	scoped_guard(mutex, &c->lock)
		actions = apple_dpin_step(&c->state, c->acio->dp,
					  APPLE_DPIN_RETRY, !!c->regs, 0);
	if (actions & APPLE_DPIN_QUEUE)
		queue_work(c->acio->dp_wq, &c->work);
}

/*
 * Only where both the display allocator and the connection manager can hold
 * a tunnel for a pipeline (capacity-retry hosts, see
 * apple_nhi_dp_tunnel_awaits_display()) does a refused tunnel wait for one.
 */
static void apple_dpin_work_fn(struct work_struct *work)
{
	struct apple_dpin_ctx *c = container_of(work, struct apple_dpin_ctx, work);
	struct apple_dpin_binding *binding;
	enum apple_dpin_event event;
	bool deferred, stale;
	unsigned int actions;
	u64 generation;
	int ret;

	for (;;) {
		scoped_guard(mutex, &c->lock) {
			actions = apple_dpin_step(&c->state, c->acio->dp,
						  APPLE_DPIN_WORK, !!c->regs, 0);
			generation = c->tokens.requested;
		}

		if (actions & APPLE_DPIN_CANCEL_RETRY)
			cancel_delayed_work(&c->retry);
		if (actions & APPLE_DPIN_DROP) {
			if (actions & APPLE_DPIN_AGAIN)
				dev_info(c->acio->dev,
					 "dpin%u: replacing stale display route for new tunnel\n",
					 c->idx);
			apple_dpin_down(c);
			if (actions & APPLE_DPIN_AGAIN)
				continue;
			dev_dbg(c->acio->dev, "dpin%u: DP tunnel down\n", c->idx);
		}
		if (!(actions & APPLE_DPIN_ATTACH))
			return;

		binding = kzalloc_obj(*binding);
		deferred = false;
		if (binding) {
			binding->ctx = c;
			binding->generation = generation;
			ret = apple_dpin_up(c, binding, &deferred);
		} else {
			ret = -ENOMEM;
		}
		event = deferred ? APPLE_DPIN_DEFER_FIRST : APPLE_DPIN_RESULT;
		scoped_guard(mutex, &c->lock) {
			stale = !c->state.alive || c->tokens.requested != generation;
			if (!ret) {
				stale |= !apple_dpin_token_complete(&c->tokens, generation);
				if (!stale)
					c->binding = binding;
			} else {
				apple_dpin_token_revoke(&c->tokens, generation);
			}
			if (!stale) {
				actions = apple_dpin_step(&c->state, c->acio->dp,
							  event, !!c->regs, ret);
				if (actions & APPLE_DPIN_RECOVERED)
					dev_info(c->acio->dev,
						 "dpin%u: a display pipeline came free; DP tunnel up\n",
						 c->idx);
			}
		}
		if (stale) {
			/* A late success owns only its own token, never the new request. */
			if (!ret)
				apple_dpin_release_binding(binding);
			else
				kfree(binding);
			continue;
		}
		if (ret)
			kfree(binding);
		if (actions & APPLE_DPIN_CANCEL_RETRY)
			cancel_delayed_work(&c->retry);
		if (actions & APPLE_DPIN_AGAIN) {
			if (actions & APPLE_DPIN_LOG_CONNECTED)
				dev_dbg(c->acio->dev, "dpin%u: DP tunnel up\n", c->idx);
			continue;
		}
		if (actions & APPLE_DPIN_FIRST_WAIT)
			dev_info(c->acio->dev,
				 "dpin%u: no display pipeline free; waiting for one\n",
				 c->idx);
		if (actions & APPLE_DPIN_ARM_RETRY)
			mod_delayed_work(c->acio->dp_wq, &c->retry,
					 msecs_to_jiffies(APPLE_DPIN_RETRY_MS));
		if (actions & APPLE_DPIN_WARN)
			dev_warn(c->acio->dev, "dpin%u: DP tunnel setup failed: %d\n",
				 c->idx, ret);
		return;
	}
}

static void apple_cio_destroy_wq(void *data)
{
	struct apple_cio *acio = data;

	/* a pending retry would queue onto the destroyed queue */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		cancel_delayed_work_sync(&acio->dpin[i].retry);
		cancel_delayed_work_sync(&acio->dpin[i].poll);
	}
	destroy_workqueue(acio->dp_wq);
}

static void apple_cio_of_node_put(void *data)
{
	of_node_put(data);
}

static bool apple_cio_pciec_eligible(const struct device_node *np)
{
	return of_device_is_compatible(np, "apple,t6040-pciec") ||
	       of_device_is_compatible(np, "apple,t8103-pciec") ||
	       of_device_is_compatible(np, "apple,t6000-pciec") ||
	       of_device_is_compatible(np, "apple,t6050-pciec");
}

static bool
apple_cio_pcie_tunnel_is_preinitialized(struct device_node *parent)
{
	for_each_available_child_of_node_scoped(parent, child) {
		u32 status;

		if (!apple_cio_pciec_eligible(child))
			continue;

		if (!of_property_read_u32(child, "apple,pciec-preinit-status",
					  &status) && status == 1)
			return true;
		/* Use the PCI driver's generation and DT admission checks. */
		return apple_pcie_tunnel_needs_cold_init(child);
	}

	return false;
}

static void apple_cio_reset_control_put(void *data)
{
	reset_control_put(data);
}

/*
 * PCIe-C's reset covers the controller and the DART beside it. It is
 * optional, since older device trees do not describe it, and only used where
 * the kernel initializes the controller: resetting it would discard a boot
 * loader's setup.
 */
static int apple_cio_get_pcie_reset(struct apple_cio *acio)
{
	struct reset_control *rst = NULL;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!apple_cio_pciec_eligible(child))
			continue;
		if (!of_property_read_bool(child, "apple,pciec-kernel-init"))
			break;

		rst = of_reset_control_get_optional_exclusive(child, NULL);
		break;
	}
	if (IS_ERR(rst))
		return dev_err_probe(acio->dev, PTR_ERR(rst),
				     "Unable to get PCIe-C reset\n");
	if (!rst)
		return 0;

	/* The bound PCIe host owns the line between ACIO startup attempts. */
	reset_control_release(rst);
	acio->pcie_reset = rst;
	return devm_add_action_or_reset(acio->dev, apple_cio_reset_control_put,
					rst);
}

static void apple_cio_iounmap(void *data)
{
	iounmap((__force void __iomem *)data);
}

static int apple_cio_map_pcie_intr2axi(struct apple_cio *acio)
{
	struct resource res;
	int index, ret;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!apple_cio_pciec_eligible(child))
			continue;

		index = of_property_match_string(child, "reg-names", "intr2axi");
		if (index < 0)
			return 0;

		ret = of_address_to_resource(child, index, &res);
		if (ret)
			return ret;

		/*
		 * Intr2AXI controls PCIe configuration transaction forwarding and
		 * must therefore use non-posted Device-nGnRnE accesses.  A regular
		 * ioremap() is Device-nGnRE on arm64 and can leave this pulse pending
		 * until the following DART access, which raises an asynchronous SError.
		 *
		 * Do not reserve the resource here: the PCIe-C child owns the same DT
		 * aperture and will claim it when the tunnel children are populated.
		 */
		acio->pcie_intr2axi_base =
			ioremap_np(res.start, resource_size(&res));
		if (!acio->pcie_intr2axi_base)
			return -ENOMEM;

		ret = devm_add_action_or_reset(acio->dev, apple_cio_iounmap,
					       (__force void *)acio->pcie_intr2axi_base);
		if (ret)
			return ret;

		return 0;
	}

	return -ENODEV;
}

/**
 * struct apple_nhi - Apple Native Host Interface
 * @dev: NHI device
 * @pdev: NHI platform device
 * @np: NHI device tree node
 * @acio: Parent ACIO complex
 * @tb: USB4 domain and software connection manager
 * @nhi: NHI struct
 * @nhi_base: NHI registers
 * @pdf_base: PDF (Protocol Defined Field) configuration registers
 * @tx_irqs: Transmit ring interrupts indexed by HopID
 * @rx_irqs: Receive ring interrupts indexed by HopID
 * @tx_irq_names: Names of the transmit ring interrupts
 * @rx_irq_names: Names of the receive ring interrupts
 * @n_rings: Number of rings in each direction
 */
struct apple_nhi {
	struct device *dev;
	struct platform_device *pdev;
	struct device_node *np;

	struct apple_cio *acio;

	struct tb *tb;
	struct tb_nhi nhi;
	struct tb_nhi_ops ops;

	void __iomem *nhi_base;
	struct apple_tunable *nhi_tunable;
	struct apple_dart_cio *cio_dart;
	struct phy *cio_phy;
	/* T6050: the port's combo PHY and its four calls, taken from its module */
	struct device *cio_phy_dev;
	int (*cio_phy_fn[5])(struct device *dev);
	/* TEST (J714s): the PHY's call for the port's USB host kept through the router's sleep */
	int (*cio_phy_usb)(struct device *dev, enum apple_t6050_cio_usb step);
	ktime_t cio_test_asleep;	/* TEST (sv6t11): the firmware's state word read asleep */
	bool cio_t6050;
	struct device *cio_pcie;
	u8 cio_pd_suspended;
	bool cio_prepared, cio_rtk_off, cio_cycled;
	/* ATC2_CIO is down for a router's sleep in place, until the PHY's CIO mode is up again */
	bool cio_router_pd_down;
	void __iomem *pdf_base;

	int *tx_irqs;
	int *rx_irqs;
	const char **tx_irq_names;
	const char **rx_irq_names;
	size_t n_rings;

	/* DP IN analog/AUX: poll adapter CS after the tunnel is up. */
	struct delayed_work dp_aux_work;
	u8 dp_in_port;
	u32 analog_base;
	u32 dp_in_cs[14];
	bool dp_aux_armed;
	unsigned int dp_aux_polls;
};

static int apple_nhi_cio_bind(struct apple_nhi *anhi);
static int apple_nhi_cio_prepare(struct apple_nhi *anhi);
static int apple_nhi_cio_suspend(struct apple_nhi *anhi);
static int apple_nhi_cio_resume(struct apple_nhi *anhi);
static void apple_nhi_cio_complete(struct apple_nhi *anhi);

#define nhi_to_anhi(nhi_) container_of((nhi_), struct apple_nhi, nhi)

/**
 * apple_cio_rtkit_shmem_setup - Map an RTKit shared memory buffer
 * @cookie: ACIO instance passed to RTKit
 * @bfr: RTKit shared memory buffer descriptor
 *
 * Translate the firmware-provided IOVA into the ACIO SRAM mapping and reject
 * buffers that fall outside the reserved SRAM resource.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_rtkit_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_cio *acio = cookie;
	struct resource res = {
		.name = "acio_rtkit_buffer",
		.flags = acio->sram_res->flags,
	};

	if (!bfr->iova)
		return -EIO;

	if (bfr->iova < APPLE_CIO_SRAM_IOVA_BASE) {
		dev_err(acio->dev, "firmware requested invalid buffer before SRAM IOVA base 0x%llx\n",
			bfr->iova);
		return -EFAULT;
	}

	res.start = bfr->iova - APPLE_CIO_SRAM_IOVA_BASE + acio->sram_res->start;
	res.end = res.start + bfr->size - 1;

	if (res.end < res.start) {
		dev_err(acio->dev, "firmware requested invalid buffer %pR\n", &res);
		return -EFAULT;
	}

	if (!resource_contains(acio->sram_res, &res)) {
		dev_err(acio->dev, "firmware requested buffer %pR outside SRAM %pR\n", &res,
			acio->sram_res);
		return -EFAULT;
	}

	bfr->iomem = acio->sram_base + (res.start - acio->sram_res->start);
	bfr->is_mapped = true;
	return 0;
}

/*
 * TEST: T6050. The router firmware names the lane initialisation it runs: action 0 at a cold
 * start, 4 for its own sleep, 5 for its wake from that sleep (macOS, runs mxt4 and mxt5:
 * "Entering Phase 1. action=4", "ACIO SLEEP process completed" before the processor's
 * power-off; "Waking up from sleep!", "Entering Phase 1. action=5", "Got AT register 12"
 * after the restart). A router restarted without its sleep runs action 0 and the dock's
 * retimer never answers register 12. The records are only read here: every one is still
 * logged and acknowledged as before.
 */
static bool apple_cio_rtkit_syslog(void *cookie, const char *context, const char *message)
{
	struct apple_cio *acio = cookie;
	unsigned int action, from, to;
	const char *p;

	if (acio->hw != &apple_cio_t6050_hw)
		return false;
	p = strstr(message, "Entering Phase 1. action=");
	if (p && !kstrtouint(p + strlen("Entering Phase 1. action="), 10, &action)) {
		atomic_set(&acio->fw_lane_action, action);
		dev_info(acio->dev, "T6050 sleep: router firmware reports lane init action %u (%s)\n",
			 action, action == 4 ? "its own sleep" :
			 action == 5 ? "its wake from that sleep" :
			 action == 0 ? "a cold start" : "not one the sleep knows");
	} else if (strstr(message, "ACIO SLEEP process completed")) {
		atomic_inc(&acio->fw_sleep_done);
	} else if (strstr(message, "Waking up from sleep")) {
		atomic_inc(&acio->fw_wake_seen);
	} else if (strstr(message, "Got AT register 12")) {
		atomic_inc(&acio->fw_at_answers);
	} else if (strstr(message, "Timeout for at_rt command response")) {
		atomic_inc(&acio->fw_at_timeouts);
	} else if ((p = strstr(message, "clx_en changed ")) &&
		   sscanf(p, "clx_en changed %x --> %x", &from, &to) == 2) {
		/*
		 * TEST (sv6u4): its CL configuration change (macOS: "CLX CFG: clx_en changed"
		 * at each of the host's CL-state writes), against the link's CL writes.
		 */
		tb_clx_test_fw_log(from, to);
	}
	return false;
}

static const struct apple_rtkit_ops apple_cio_rtkit_ops = {
	.shmem_setup = apple_cio_rtkit_shmem_setup,
	.syslog_is_debug = apple_cio_rtkit_syslog,
};

static int apple_nhi_probe_irqs(struct apple_nhi *anhi)
{
	int n_irqs;
	char name[64];

	n_irqs = platform_irq_count(anhi->pdev);
	if (n_irqs < 0)
		return dev_err_probe(anhi->dev, n_irqs, "platform_irq_count failed\n");
	/* T6050 adds a named SRAM parity interrupt after the ring interrupts. */
	if (n_irqs > 0 &&
	    platform_get_irq_byname_optional(anhi->pdev, "parity") > 0)
		n_irqs--;
	if (n_irqs == 0) {
		dev_err(anhi->dev, "No interrupts found\n");
		return -EINVAL;
	}
	if (n_irqs % 2) {
		dev_err(anhi->dev, "Invalid number of interrupts: %d must be even\n", n_irqs);
		return -EINVAL;
	}
	/*
	 * TX ring n is addressed as bit n and RX ring n as bit n + n_rings of the 32 bit
	 * IRQ status and enable registers, so both halves have to fit into a single
	 * register. hop_count is only checked against n_rings later on, and both come
	 * from firmware, so bound this before anything shifts by the derived index.
	 */
	if (n_irqs > 32) {
		dev_err(anhi->dev, "Invalid number of interrupts: %d exceeds 32 status bits\n",
			n_irqs);
		return -EINVAL;
	}
	anhi->n_rings = n_irqs / 2;

	anhi->rx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->rx_irqs), GFP_KERNEL);
	if (!anhi->rx_irqs)
		return -ENOMEM;
	anhi->tx_irqs = devm_kcalloc(anhi->dev, anhi->n_rings, sizeof(*anhi->tx_irqs), GFP_KERNEL);
	if (!anhi->tx_irqs)
		return -ENOMEM;
	anhi->rx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->rx_irq_names), GFP_KERNEL);
	if (!anhi->rx_irq_names)
		return -ENOMEM;
	anhi->tx_irq_names = devm_kcalloc(anhi->dev, anhi->n_rings,
					  sizeof(*anhi->tx_irq_names), GFP_KERNEL);
	if (!anhi->tx_irq_names)
		return -ENOMEM;

	for (int i = 0; i < anhi->n_rings; ++i) {
		snprintf(name, sizeof(name), "rxring%d", i);
		anhi->rx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->rx_irqs[i] < 0)
			return anhi->rx_irqs[i];
		anhi->rx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->rx_irq_names[i])
			return -ENOMEM;

		snprintf(name, sizeof(name), "txring%d", i);
		anhi->tx_irqs[i] = platform_get_irq_byname(anhi->pdev, name);
		if (anhi->tx_irqs[i] < 0)
			return anhi->tx_irqs[i];
		anhi->tx_irq_names[i] = devm_kasprintf(anhi->dev, GFP_KERNEL, "%s-%s",
						       dev_name(anhi->dev), name);
		if (!anhi->tx_irq_names[i])
			return -ENOMEM;
	}

	return 0;
}

static unsigned int apple_cio_ring_index(struct tb_ring *ring)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);

	if (ring->is_tx)
		return ring->hop;
	else
		return ring->hop + anhi->n_rings;
}

static void apple_nhi_ring_interrupt_active(struct tb_ring *ring, bool active)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);
	u32 reg, interval;

	lockdep_assert_held(&ring->nhi->lock);

	if (active && ring->interval_nsec) {
		interval = min_t(u32, ring->interval_nsec,
				 FIELD_MAX(APPLE_CIO_NHI_IRQ_THROTTLE_INTERVAL_MASK) *
				 APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		interval = DIV_ROUND_UP(interval,
					APPLE_CIO_NHI_IRQ_THROTTLE_GRANULARITY_NSEC);
		writel(interval, anhi->nhi_base + APPLE_CIO_NHI_IRQ_THROTTLE +
				 4 * idx);
	}

	reg = readl(anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);

	if (active)
		reg |= BIT(idx);
	else
		reg &= ~BIT(idx);

	writel(reg, anhi->nhi_base + APPLE_CIO_NHI_IRQ_ENABLE);
}

static void apple_nhi_ring_interrupt_mask(struct tb_ring *ring, bool mask)
{
	apple_nhi_ring_interrupt_active(ring, !mask);
}

static irqreturn_t apple_cio_ring_irq(int irq, void *data)
{
	struct tb_ring *ring = data;
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	unsigned int idx = apple_cio_ring_index(ring);

	guard(spinlock)(&ring->nhi->lock);
	guard(spinlock)(&ring->lock);

	writel(BIT(idx), anhi->nhi_base + APPLE_CIO_NHI_IRQ_STATUS);
	if (!ring->running)
		return IRQ_HANDLED;

	if (ring->start_poll) {
		apple_nhi_ring_interrupt_mask(ring, true);
		ring->start_poll(ring->poll_data);
	} else {
		schedule_work(&ring->work);
	}

	return IRQ_HANDLED;
}


static int apple_nhi_request_irq(struct tb_ring *ring, bool no_suspend)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	const char *name;

	if (ring->is_tx) {
		ring->irq = anhi->tx_irqs[ring->hop];
		name = anhi->tx_irq_names[ring->hop];
	} else {
		ring->irq = anhi->rx_irqs[ring->hop];
		name = anhi->rx_irq_names[ring->hop];
	}

	return devm_request_irq(anhi->dev, ring->irq, apple_cio_ring_irq,
				no_suspend ? IRQF_NO_SUSPEND : 0, name, ring);
}

static void apple_nhi_release_irq(struct tb_ring *ring)
{
	if (ring->irq <= 0)
		return;

	devm_free_irq(ring->nhi->dev, ring->irq, ring);
	ring->irq = 0;
}

static void apple_nhi_ring_configure(struct tb_ring *ring, u32 flags, u32 e2e_flags)
{
	struct apple_nhi *anhi = nhi_to_anhi(ring->nhi);
	void __iomem *options;
	u32 sof_eof_mask;

	lockdep_assert_held(&ring->lock);

	options = anhi->nhi_base + ring->hop * APPLE_CIO_NHI_RING_STRIDE + 0x10;

	if (ring->is_tx) {
		options += APPLE_CIO_NHI_TXRING_DESC_BASE;

		/*
		 * Partition the 232 shared TX entries: two for control ring 0,
		 * forty each for rings 1-5, and five each for the remaining rings.
		 */
		if (ring->hop == 0)
			writel(2, options + 4);
		else if (ring->hop <= 5)
			writel(40, options + 4);
		else
			writel(5, options + 4);
	} else {
		options += APPLE_CIO_NHI_RXRING_DESC_BASE;

		sof_eof_mask = ring->sof_mask << 16 | ring->eof_mask;
		writel(sof_eof_mask, options + 4);
		if (anhi->pdf_base)
			writel(sof_eof_mask,
			       anhi->pdf_base + ring->hop * APPLE_CIO_NHI_PDF_STRIDE);
	}

	/*
	 * The firmware samples E2E flow control when the valid bit is set.
	 * Program both in the same write.
	 */
	writel(flags | e2e_flags, options);
}

static struct platform_device *apple_cio_find_pcie_tunnel(struct apple_cio *acio);

static int apple_cio_populate_pcie_tunnel(struct apple_cio *acio)
{
	int ret;

	/*
	 * PCIe-C was cold-initialized by m1n1 before the ACIO M3 started, but
	 * enabling ACIO's cable-powered PCIe domain closes the Intr2AXI bridge
	 * again. The port enable sequence pulses this register immediately
	 * before forcing its DART active. Do the same after ACIO has
	 * enabled both tunnel adapters and before either child can touch DART MMIO.
	 * The bit self-clears after opening the bridge.
	 */
	if (acio->pcie_intr2axi_base) {
		writel(APPLE_CIO_PCIEC_INTR2AXI_ENABLE,
		       acio->pcie_intr2axi_base + APPLE_CIO_PCIEC_INTR2AXI_CTRL);
		/* Complete the pulse without reading the transient register. */
		mb();
		dev_info(acio->dev, "PCIe-C Intr2AXI bridge enabled\n");
	}

	/*
	 * The DART is the first child and probes as soon as it is created.
	 * Its command engine stays busy until this port clock is running, so
	 * finish cold init before either child is populated.
	 */
	ret = apple_pcie_tunnel_prepare(acio->dev, acio->pcie_tunnel_np);
	if (ret)
		return ret;

	dev_info(acio->dev, "PCIe-C populating tunnel children\n");
	return of_platform_populate(acio->pcie_tunnel_np, NULL, NULL,
				    acio->dev);
}

/* The host must have finished probing before activation can be announced. */
static int apple_cio_pcie_tunnel_ready_locked(struct apple_cio *acio)
{
	struct platform_device *pdev;
	int ret;

	lockdep_assert_held(&acio->pcie_tunnel_lock);
	pdev = apple_cio_find_pcie_tunnel(acio);
	if (!pdev)
		return -EAGAIN;

	ret = apple_pcie_tunnel_restore(&pdev->dev);
	if (!ret)
		ret = apple_pcie_tunnel_check_state(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}

/*
 * PCIe-C is the only native port this host tunnels. Order it after the NHI,
 * which rebuilds the tunnel underneath it at resume (the job of the ACPI
 * device links on PC hosts). The link is stateless: it orders system sleep
 * only and leaves probe and unbind alone, because apple_cio_remove_children()
 * must stop the NHI while PCIe-C is still bound. Removing either device drops
 * the link; both are removed together and PCIe-C is populated only once per
 * NHI, so this runs once per pair.
 */
static void apple_cio_link_pcie_tunnel(struct apple_cio *acio)
{
	struct platform_device *nhi_pdev = READ_ONCE(acio->nhi_pdev);
	struct platform_device *pcie_pdev;

	lockdep_assert_held(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (!pcie_pdev)
		return;
	if (!nhi_pdev ||
	    !device_link_add(&pcie_pdev->dev, &nhi_pdev->dev, DL_FLAG_STATELESS))
		dev_warn(acio->dev, "PCIe-C: device link to the NHI failed\n");
	else
		dev_dbg(acio->dev, "PCIe-C: linked %s to %s\n",
			dev_name(&pcie_pdev->dev), dev_name(&nhi_pdev->dev));
	put_device(&pcie_pdev->dev);
}

static int apple_cio_activate_pcie_tunnel_locked(struct apple_cio *acio)
{
	int ret = 0;

	lockdep_assert_held(&acio->pcie_tunnel_lock);

	if (!acio->pcie_tunnel_np)
		return 0;

	/*
	 * The host survives a tunnel teardown, quiesced with its hierarchy
	 * removed. Bring that one back rather than populating a second.
	 */
	if (acio->pcie_tunnel_populated)
		return apple_cio_pcie_tunnel_ready_locked(acio);
	if (!acio->pcie_tunnel_preinitialized)
		return dev_err_probe(acio->dev, -ENODEV,
				     "PCIe-C is not initialized by aura or the kernel\n");

	if (acio->pd_list->num_pds < 4 || !acio->pd_list->pd_links[3])
		return dev_err_probe(acio->dev, -ENODEV,
				     "pre-ACIO PCIe-C initialization is not active\n");

	ret = apple_cio_populate_pcie_tunnel(acio);
	if (ret)
		return dev_err_probe(acio->dev, ret,
				     "failed to populate tunneled PCIe controller\n");

	acio->pcie_tunnel_populated = true;
	apple_cio_link_pcie_tunnel(acio);
	dev_info(acio->dev,
		 "PCIe-C populated after USB4 PCIe tunnel adapter enable\n");
	return apple_cio_pcie_tunnel_ready_locked(acio);
}

static int apple_cio_quiesce_pcie_tunnel_locked(struct apple_cio *acio)
{
	struct platform_device *pcie_pdev;
	int ret = 0;

	lockdep_assert_held(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (pcie_pdev) {
		ret = apple_pcie_tunnel_quiesce(&pcie_pdev->dev);
		put_device(&pcie_pdev->dev);
	}
	if (!ret)
		acio->pcie_quiesce_pending = false;
	return ret;
}

#define APPLE_CIO_PCIE_BIND_RETRIES	20
#define APPLE_CIO_PCIE_BIND_WAIT_MS	250

static void apple_cio_pcie_tunnel_work(struct work_struct *work)
{
	struct apple_cio *acio =
		container_of(to_delayed_work(work), struct apple_cio,
			     pcie_tunnel_work);
	bool activated = false, failed = false;
	int ret = 0;

	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping || acio->pcie_pm_prepared)
		goto unlock;
	if (acio->pcie_quiesce_pending) {
		ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
		if (ret)
			goto unlock;
	}
	if (READ_ONCE(acio->pcie_tunnel_requested)) {
		ret = apple_cio_activate_pcie_tunnel_locked(acio);
		activated = !ret && acio->pcie_tunnel_populated;
		if (ret == -EAGAIN) {
			if (acio->pcie_bind_retries) {
				acio->pcie_bind_retries--;
				mod_delayed_work(system_freezable_wq,
						 &acio->pcie_tunnel_work,
						 msecs_to_jiffies(APPLE_CIO_PCIE_BIND_WAIT_MS));
			} else {
				ret = -ETIMEDOUT;
			}
		}
		failed = ret && ret != -EAGAIN;
	}
unlock:
	mutex_unlock(&acio->pcie_tunnel_lock);
	/* Wake a deferred Type-C check after asynchronous PCIe setup finishes. */
	if (activated)
		typec_thunderbolt_switch_notify_ready(acio->tbt_switch);
	else if (failed)
		typec_thunderbolt_switch_notify(acio->tbt_switch);
	if (ret && ret != -EAGAIN)
		dev_err(acio->dev, "deferred PCIe-C transition failed: %d\n", ret);
}

static int apple_nhi_pci_tunnel_deactivate(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	int ret = 0;

	/*
	 * The tunnel is going away while the router and NHI stay up, so nothing
	 * else tears the tunneled PCI hierarchy down. Left alone the endpoint
	 * keeps its driver bound with no data path underneath it: every access
	 * blocks until the function driver's own timeout fires, and the host has
	 * nothing to enumerate onto when the tunnel comes back.
	 */
	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping)
		goto unlock;
	WRITE_ONCE(acio->pcie_tunnel_requested, false);
	cancel_delayed_work(&acio->pcie_tunnel_work);
	acio->pcie_quiesce_pending = true;
	/* Endpoint drivers must resume before they can be unbound safely. */
	if (!acio->pcie_pm_prepared) {
		ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
		if (ret)
			dev_warn(acio->dev,
				 "failed to quiesce PCIe-C on tunnel teardown: %d\n",
				 ret);
	}

unlock:
	mutex_unlock(&acio->pcie_tunnel_lock);

	return ret;
}

static int apple_nhi_pci_tunnel_post_activate(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;

	/*
	 * This is the boundary at which the tunnel becomes usable.
	 * The USB4 configuration writes are asynchronous, so return to the
	 * Thunderbolt core before probing PCIe-C. The worker then waits on the
	 * hardware reset state without blocking completion of the tunnel paths.
	 */
	mutex_lock(&acio->pcie_tunnel_lock);
	if (acio->pcie_tunnel_stopping) {
		mutex_unlock(&acio->pcie_tunnel_lock);
		return -ESHUTDOWN;
	}
	WRITE_ONCE(acio->pcie_tunnel_requested, true);
	acio->pcie_bind_retries = APPLE_CIO_PCIE_BIND_RETRIES;
	/* No PCI removal or rescan until system resume has finished. */
	mod_delayed_work(system_freezable_wq, &acio->pcie_tunnel_work, 0);
	mutex_unlock(&acio->pcie_tunnel_lock);
	return 0;
}

static void apple_cio_stop_pcie_tunnel(struct apple_cio *acio)
{
	int ret;

	/* Block new requests before draining work, without nesting tb->lock. */
	mutex_lock(&acio->pcie_tunnel_lock);
	acio->pcie_tunnel_stopping = true;
	WRITE_ONCE(acio->pcie_tunnel_requested, false);
	mutex_unlock(&acio->pcie_tunnel_lock);
	cancel_delayed_work_sync(&acio->pcie_tunnel_work);

	/* The NHI must remain alive to complete the tunneled reset handshake. */
	mutex_lock(&acio->pcie_tunnel_lock);
	ret = apple_cio_quiesce_pcie_tunnel_locked(acio);
	mutex_unlock(&acio->pcie_tunnel_lock);
	if (ret)
		dev_warn(acio->dev,
			 "failed to quiesce PCIe-C before NHI shutdown: %d\n", ret);
}

#define APPLE_DP_AUX_POLL_MS		500
#define APPLE_DP_AUX_POLL_MAX		24

/*
 * apple,tunable-rc programs two analog PHY blocks inside the already-mapped
 * ACIO RC window (0xc000). They match the two host DP IN adapters (0:5, 0:6).
 * Dumping 0x00-0xff never saw them. Do not scan unmapped ACIO ranges.
 */
#define APPLE_CIO_DPIN0_ANALOG		0x4000
#define APPLE_CIO_DPIN1_ANALOG		0x8000
#define APPLE_CIO_DPIN_ANALOG_SIZE	0x80
#define APPLE_CIO_DPIN_ANALOG_FSM	0x18
#define APPLE_CIO_DPIN_ANALOG_HOLE	0x20
#define APPLE_CIO_DPIN_ANALOG_EMPTY	0x80000000
#define APPLE_CIO_DPIN_ANALOG_HOLE_VAL	0x40

/*
 * Analog MMIO writes are closed: +0x00 / +0x18 / +0x20 do not stick.
 * +0x18 is first-read status 0x1017 (read-to-clear).
 *
 * 0 = do not touch analog at tunnel-up; dump it only after DPRX timeout
 *     (default). Tests whether consuming +0x18 aborted a handshake.
 * 1 = old pulse +0x00 / fill +0x20 (rollback)
 */
static bool apple_dpin_aux;
module_param_named(dpin_aux, apple_dpin_aux, bool, 0644);
MODULE_PARM_DESC(dpin_aux,
		 "Apple DP IN analog: pulse +0x00 at the next tunnel activation (default: false)");

static void apple_dp_dump_hop(struct tb_port *port, unsigned int hopid)
{
	struct tb_regs_hop hop;
	int ret;

	ret = tb_port_read(port, &hop, TB_CFG_HOPS, 2 * hopid, 2);
	if (ret) {
		tb_port_dbg(port, "DP IN hop %u read failed: %d\n", hopid, ret);
		return;
	}
	tb_port_dbg(port,
		     "DP IN hop %u enable=%u out=%u next=%u credits=%u\n",
		     hopid, hop.enable, hop.out_port, hop.next_hop,
		     hop.initial_credits);
}



static void apple_dp_dump_adapter(struct tb_port *port, const char *tag)
{
	u32 w[17], cs4 = 0xffffffff;
	int i, ret;

	ret = tb_port_read(port, &w[0], TB_CFG_PORT, port->cap_adap, 1);
	if (ret || w[0] == 0xffffffff) {
		tb_port_dbg(port, "%s config space dead ret=%d (not DPRX)\n",
			     tag, ret);
		return;
	}
	for (i = 1; i <= 16; i++) {
		ret = tb_port_read(port, &w[i], TB_CFG_PORT,
				   port->cap_adap + i, 1);
		if (ret)
			w[i] = 0xffffffff;
	}
	tb_port_read(port, &cs4, TB_CFG_PORT, ADP_CS_4, 1);
	tb_port_dbg(port,
		     "%s type=%06x cap=%d CS0=%08x CS1=%08x CS2=%08x CS3=%08x LOCAL=%08x REMOTE=%08x STAT=%08x COMMON=%08x CS8=%08x CS9=%08x CS10=%08x ADP_CS4=%08x VE=%u AE=%u HPD=%u DPRX=%u LCK=%u\n",
		     tag, port->config.type, port->cap_adap,
		     w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8],
		     w[9], w[10], cs4,
		     !!(w[0] & ADP_DP_CS_0_VE), !!(w[0] & ADP_DP_CS_0_AE),
		     !!(w[2] & ADP_DP_CS_2_HPD),
		     !!(w[7] & DP_COMMON_CAP_DPRX_DONE),
		     !!(cs4 & ADP_CS_4_LCK));
	tb_port_dbg(port,
		     "%s CS11=%08x CS12=%08x CS13=%08x CS14=%08x CS15=%08x CS16=%08x disc=%u\n",
		     tag, w[11], w[12], w[13], w[14], w[15], w[16],
		     !!(w[13] & ADP_DP_CS_13_DPTX_DISCOVERY_MODE));
}

static void apple_dp_dump_host_adapters(struct apple_nhi *anhi)
{
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (tb_port_is_dpin(port))
			apple_dp_dump_adapter(port, "host DP IN");
		else if (tb_port_is_dpout(port))
			apple_dp_dump_adapter(port, "host DP OUT");
	}
}

static void apple_dp_dump_rc_range(struct apple_cio *acio, u32 base, u32 len,
				   const char *tag, bool skip_zero)
{
	char buf[320];
	int n = 0;
	u32 off, size;

	if (!acio->rc_base || !acio->rc_res)
		return;
	size = resource_size(acio->rc_res);
	if (base >= size)
		return;
	if (base + len > size)
		len = size - base;

	for (off = 0; off < len; off += 4) {
		u32 val = readl(acio->rc_base + base + off);

		if (skip_zero && !val)
			continue;
		n += scnprintf(buf + n, sizeof(buf) - n, " %03x=%08x", off, val);
		if (n >= (int)sizeof(buf) - 24) {
			dev_dbg(acio->dev, "ACIO RC %s 0x%x:%s\n", tag, base, buf);
			n = 0;
			buf[0] = '\0';
		}
	}
	if (n)
		dev_dbg(acio->dev, "ACIO RC %s 0x%x:%s\n", tag, base, buf);
}

static void apple_dp_dump_rc(struct apple_cio *acio)
{
	apple_dp_dump_rc_range(acio, 0, 0x100, "ctrl", true);
}

static void apple_dp_dump_vse(struct tb_switch *sw)
{
	char buf[320];
	int cap, i, n = 0, ret;
	u32 w;

	cap = tb_switch_find_vse_cap(sw, TB_VSE_CAP_APPLE);
	if (cap < 0) {
		tb_sw_dbg(sw, "Apple VSE cap missing: %d\n", cap);
		return;
	}
	for (i = 0; i < 16; i++) {
		ret = tb_sw_read(sw, &w, TB_CFG_SWITCH, cap + i, 1);
		if (ret)
			w = 0xffffffff;
		n += scnprintf(buf + n, sizeof(buf) - n, " %02x=%08x", i, w);
		if (n >= (int)sizeof(buf) - 20) {
			tb_sw_dbg(sw, "Apple VSE cap=%d:%s\n", cap, buf);
			n = 0;
			buf[0] = '\0';
		}
	}
	if (n)
		tb_sw_dbg(sw, "Apple VSE cap=%d:%s\n", cap, buf);
}



static void apple_dp_dump_analog(struct apple_cio *acio, const char *tag)
{
	apple_dp_dump_rc_range(acio, APPLE_CIO_DPIN0_ANALOG,
			      APPLE_CIO_DPIN_ANALOG_SIZE, tag, false);
	apple_dp_dump_rc_range(acio, APPLE_CIO_DPIN1_ANALOG,
			      APPLE_CIO_DPIN_ANALOG_SIZE, "dpin1 analog", false);
}

static u32 apple_dp_in_analog_base(struct apple_nhi *anhi, struct tb_port *in)
{
	unsigned int idx = 0;
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return APPLE_CIO_DPIN0_ANALOG;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port == in)
			return idx ? APPLE_CIO_DPIN1_ANALOG :
				     APPLE_CIO_DPIN0_ANALOG;
		idx++;
	}
	return APPLE_CIO_DPIN0_ANALOG;
}

static void apple_dp_start_analog(struct apple_nhi *anhi, bool pulse)
{
	struct apple_cio *acio;
	u32 block, ctrl, hole, fsm;

	if (!anhi || !anhi->acio || !anhi->acio->rc_base)
		return;
	acio = anhi->acio;
	block = anhi->analog_base ?: APPLE_CIO_DPIN0_ANALOG;
	if (block + APPLE_CIO_DPIN_ANALOG_HOLE + 4 > resource_size(acio->rc_res))
		return;

	ctrl = readl(acio->rc_base + block);
	hole = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
	fsm = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_FSM);
	dev_info(acio->dev,
		 "DP IN analog +0x00=%08x +0x18=%08x +0x20=%08x pulse=%d\n",
		 ctrl, fsm, hole, pulse);

	if (!pulse)
		return;

	/* +0x00 readback is empty status; still pulse start like PCIe-C. */
	writel(ctrl | BIT(0), acio->rc_base + block);
	mb();
	ctrl = readl(acio->rc_base + block);

	if (hole == 0) {
		writel(APPLE_CIO_DPIN_ANALOG_HOLE_VAL,
		       acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
		mb();
		hole = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_HOLE);
	}

	fsm = readl(acio->rc_base + block + APPLE_CIO_DPIN_ANALOG_FSM);
	dev_info(acio->dev,
		 "DP IN analog after start +0x00=%08x +0x18=%08x +0x20=%08x\n",
		 ctrl, fsm, hole);
	apple_dp_dump_rc_range(acio, block, APPLE_CIO_DPIN_ANALOG_SIZE,
			      "dpin analog after start", false);
}

static int apple_dp_dptx_discover(struct tb_port *in)
{
	u32 cs0 = 0;

	tb_port_read(in, &cs0, TB_CFG_PORT, in->cap_adap + ADP_DP_CS_0, 1);
	tb_port_warn(in,
		     "USB4 DPTX Discovery closed (CS13 writable, CS9 static); AE=%u VE=%u\n",
		     !!(cs0 & ADP_DP_CS_0_AE), !!(cs0 & ADP_DP_CS_0_VE));
	return 0;
}

static void apple_dp_aux_work(struct work_struct *work)
{
	struct apple_nhi *anhi =
		container_of(to_delayed_work(work), struct apple_nhi,
			     dp_aux_work);
	struct tb_port *port;
	u32 cs[14];
	int i, ret;
	bool changed = false, dprx = false;

	if (!anhi->tb)
		return;

	mutex_lock(&anhi->tb->lock);
	if (!anhi->dp_aux_armed || !anhi->tb->root_switch || READ_ONCE(anhi->acio->native_sleep))
		goto out_unlock;

	port = &anhi->tb->root_switch->ports[anhi->dp_in_port];
	if (!tb_port_is_dpin(port))
		goto out_unlock;

	for (i = 0; i <= 13; i++) {
		ret = tb_port_read(port, &cs[i], TB_CFG_PORT,
				   port->cap_adap + i, 1);
		if (ret)
			cs[i] = 0xffffffff;
		if (cs[i] != anhi->dp_in_cs[i])
			changed = true;
	}
	if (changed) {
		memcpy(anhi->dp_in_cs, cs, sizeof(cs));
		tb_port_dbg(port,
			     "DP IN CS changed CS0=%08x CS2=%08x CS9=%08x CS13=%08x COMMON=%08x VE=%u AE=%u HPD=%u DPRX=%u disc=%u\n",
			     cs[0], cs[2], cs[9], cs[13], cs[7],
			     !!(cs[0] & ADP_DP_CS_0_VE),
			     !!(cs[0] & ADP_DP_CS_0_AE),
			     !!(cs[2] & ADP_DP_CS_2_HPD),
			     !!(cs[7] & DP_COMMON_CAP_DPRX_DONE),
			     !!(cs[13] & ADP_DP_CS_13_DPTX_DISCOVERY_MODE));
	}
	dprx = !!(cs[7] & DP_COMMON_CAP_DPRX_DONE);
	if (dprx)
		tb_port_dbg(port, "DP IN DPRX_DONE=1 (ACIO AUX completed)\n");

	anhi->dp_aux_polls++;
	/*
	 * Deliberately not dumping or reading the analog block on every
	 * poll here. +0x18 is an ordinary one-shot, read-to-clear status
	 * register -- already documented above (APPLE_CIO_DPIN_ANALOG_EMPTY,
	 * and the comment a few lines up: "+0x18 is first-read status
	 * 0x1017 (read-to-clear)"), not a stuck error FSM, so polling it
	 * repeatedly has no diagnostic value and risks consuming a status
	 * this project's own dpin_aux default-off comment already warns
	 * could abort a handshake if read too early. A write-1-to-clear
	 * ack against it has no effect either: 0x80000000 is the
	 * register's own idle/empty sentinel, not a latched error that
	 * needs acknowledging. Stick to a single post-mortem dump below,
	 * after the fact, once polling ends.
	 */
	if (!dprx && anhi->dp_aux_polls < APPLE_DP_AUX_POLL_MAX) {
		mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work,
				 msecs_to_jiffies(APPLE_DP_AUX_POLL_MS));
	} else if (anhi->acio) {
		apple_dp_dump_analog(anhi->acio,
				     dprx ? "dpin0 analog DPRX done" :
					    "dpin0 analog at timeout");
	}

out_unlock:
	mutex_unlock(&anhi->tb->lock);
}

/*
 * Which DP IN adapter (0..3) on the host router this port is -- same
 * counting order as apple_dp_in_analog_base() above and the port-to-dpin
 * mapping in apple_nhi_dp_tunnel_changed().
 */
static int apple_dpin_index_for_port(struct apple_nhi *anhi, struct tb_port *in)
{
	unsigned int idx = 0;
	struct tb_port *port;

	if (!anhi->tb || !anhi->tb->root_switch)
		return -1;

	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port == in)
			return idx;
		idx++;
	}
	return -1;
}

/* A DP IN adapter index with a block of its own on this router */
static bool apple_dpin_valid(const struct apple_cio *acio, int idx)
{
	return acio && idx >= 0 && idx < acio->dpin_count;
}

static int apple_nhi_dp_tunnel_pre_activate(struct tb_nhi *nhi,
					    struct tb_port *in,
					    struct tb_port *out)
{
	if (tb_port_is_dpin(in))
		return apple_dp_dptx_discover(in);
	return 0;
}

static int apple_nhi_dp_tunnel_post_activate(struct tb_nhi *nhi,
					     struct tb_port *in,
					     struct tb_port *out, u64 generation)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	int i, idx;
	struct apple_dpin_ctx *c = NULL;

	/* Validate the token before changing analog/AUX or display state. */
	idx = apple_dpin_index_for_port(anhi, in);
	if (apple_dpin_valid(anhi->acio, idx) && anhi->acio->connector_np &&
	    anhi->acio->dp_wq) {
		c = &anhi->acio->dpin[idx];
		scoped_guard(mutex, &c->lock) {
			if (!apple_dpin_request(c, generation, true))
				return -ESTALE;
		}
	}

	dev_info(anhi->dev,
		 "DP IN tunnel routing: tunnel %u:%u <-> %u:%u\n",
		 in->sw->config.depth, in->port,
		 out ? out->sw->config.depth : 0,
		 out ? out->port : 0);

	anhi->dp_in_port = in->port;
	anhi->analog_base = apple_dp_in_analog_base(anhi, in);
	anhi->dp_aux_polls = 0;
	anhi->dp_aux_armed = true;
	/* The domain lock keeps this optional MMIO access ahead of teardown. */
	if (READ_ONCE(apple_dpin_aux))
		apple_dp_start_analog(anhi, true);

	apple_dp_dump_rc(anhi->acio);
	apple_dp_dump_vse(in->sw);

	/* Route the display pipeline through the selected host DP IN adapter. */
	if (!apple_dpin_valid(anhi->acio, idx)) {
		dev_warn(anhi->dev,
			 "DP IN tunnel: could not map port %u to a dpin index\n",
			 in->port);
	} else if (!anhi->acio->connector_np) {
		dev_warn(anhi->dev,
			 "DP IN tunnel: no Type-C connector for dpin%d\n", idx);
	} else if (!anhi->acio->dp_wq) {
		dev_warn(anhi->dev, "DP IN tunnel: no work queue for dpin%d\n", idx);
	} else {
		/* Request and REARM were published together before the diagnostics. */
		queue_work(anhi->acio->dp_wq, &c->work);
	}
	apple_dp_dump_host_adapters(anhi);
	if (tb_port_is_dpin(in)) {
		apple_dp_dump_hop(in, 8);
		apple_dp_dump_hop(in, 9);
	}
	if (out && tb_port_is_dpout(out))
		apple_dp_dump_adapter(out, "hub DP OUT");

	for (i = 0; i <= 13; i++) {
		if (tb_port_read(in, &anhi->dp_in_cs[i], TB_CFG_PORT,
				 in->cap_adap + i, 1))
			anhi->dp_in_cs[i] = 0xffffffff;
	}
	mod_delayed_work(system_dfl_wq, &anhi->dp_aux_work,
			 msecs_to_jiffies(APPLE_DP_AUX_POLL_MS));
	return 0;
}

static void apple_nhi_dp_tunnel_deactivate(struct tb_nhi *nhi,
					   struct tb_port *in,
					   struct tb_port *out, u64 generation)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	int idx;

	/* Sleep the adapter before tunnel teardown can remove its power. */
	idx = apple_dpin_index_for_port(anhi, in);
	if (apple_dpin_valid(anhi->acio, idx) && anhi->acio->dp_wq) {
		struct apple_dpin_ctx *c = &anhi->acio->dpin[idx];

		scoped_guard(mutex, &c->lock) {
			if (!apple_dpin_request(c, generation, false))
				return;
		}
		queue_work(anhi->acio->dp_wq, &c->work);
	}
	anhi->dp_aux_armed = false;
	cancel_delayed_work(&anhi->dp_aux_work);
	dev_info(anhi->dev, "DP IN tunnel routing: tunnel down\n");
}

static void apple_nhi_dp_tunnel_changed(struct tb_nhi *nhi, u8 in_port,
					u64 generation, bool active)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	struct tb_port *port;
	unsigned int dpin = 0;
	bool found = false;

	/* The crossbar's dpin0..3 follow the host router's DP IN adapters in order. */
	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_dpin(port))
			continue;
		if (port->port == in_port) {
			found = true;
			break;
		}
		dpin++;
	}
	if (!found || dpin >= acio->dpin_count) {
		dev_warn(acio->dev, "DP tunnel from unexpected adapter %u\n", in_port);
		return;
	}

	/*
	 * Runs synchronously before the tunnel is torn down: from here on the
	 * DP IN block may lose power, so nothing may touch it any more.
	 */
	scoped_guard(mutex, &acio->dpin[dpin].lock) {
		struct apple_dpin_ctx *c = &acio->dpin[dpin];

		/* Publish cookie, powered retirement and lifecycle event under one lock. */
		if (!apple_dpin_request(c, generation, active))
			return;
	}
	queue_work(acio->dp_wq, &acio->dpin[dpin].work);
}

/* The display side refused this tunnel a pipeline for now and asks again. */
static bool apple_nhi_dp_tunnel_awaits_display(struct tb_nhi *nhi,
					       struct tb_port *in)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_dpin_ctx *c;
	int idx;

	idx = apple_dpin_index_for_port(anhi, in);
	if (!apple_dpin_valid(anhi->acio, idx))
		return false;
	c = &anhi->acio->dpin[idx];
	guard(mutex)(&c->lock);
	return apple_dpin_awaits_display(&c->state, c->acio->dp);
}

/*
 * There are no firmware tables naming the tunneled ports: PCIe-C, when the
 * port has one, is linked to the NHI as it is populated (see
 * apple_cio_link_pcie_tunnel()). A port without PCIe-C tunnels no native port.
 */
static bool apple_nhi_add_links(struct tb_nhi *nhi)
{
	return true;
}

/*
 * Called with the domain lock held when the peer on a host router lane
 * adapter went away: only (re)arm the check.
 */
static void apple_nhi_link_lost(struct tb_nhi *nhi)
{
	struct apple_nhi *anhi = nhi_to_anhi(nhi);
	struct apple_cio *acio = anhi->acio;
	u64 activity = atomic64_read(&anhi->tb->remote_activity);

	/* A peer was found since the last cycle: that cycle worked */
	if (activity != READ_ONCE(acio->link_lost_cycle_activity))
		WRITE_ONCE(acio->link_lost_cycles, 0);
	WRITE_ONCE(acio->link_lost_gen, READ_ONCE(acio->start_gen));
	mod_delayed_work(system_freezable_wq, &acio->link_lost_work,
			 msecs_to_jiffies(APPLE_CIO_LINK_LOST_MS));
}

static const struct tb_nhi_ops apple_nhi_ops = {
	.add_links = apple_nhi_add_links,
	.link_lost = apple_nhi_link_lost,
	.request_ring_irq = apple_nhi_request_irq,
	.release_ring_irq = apple_nhi_release_irq,
	.ring_interrupt_active = apple_nhi_ring_interrupt_active,
	.ring_interrupt_mask = apple_nhi_ring_interrupt_mask,
	.ring_configure = apple_nhi_ring_configure,
	.pci_tunnel_post_activate = apple_nhi_pci_tunnel_post_activate,
	.pci_tunnel_deactivate = apple_nhi_pci_tunnel_deactivate,
};

static const struct tb_nhi_ring_layout apple_nhi_ring_layout = {
	.tx_desc_base = APPLE_CIO_NHI_TXRING_DESC_BASE,
	.rx_desc_base = APPLE_CIO_NHI_RXRING_DESC_BASE,
	.desc_stride = APPLE_CIO_NHI_RING_STRIDE,
	.tx_options_base = APPLE_CIO_NHI_TXRING_DESC_BASE + 0x10,
	.rx_options_base = APPLE_CIO_NHI_RXRING_DESC_BASE + 0x10,
	.options_stride = APPLE_CIO_NHI_RING_STRIDE,
};

static int apple_nhi_probe(struct platform_device *pdev)
{
	struct apple_cio *acio = dev_get_drvdata(pdev->dev.parent);
	struct apple_nhi *anhi;
	struct apple_tunable *tunable;
	struct resource *res;
	struct tb_port *port;
	enum apple_dpin_flow dp_hooks;
	int cap_apple;
	int ret = 0;

	/* The T6050 router is the Type 7 one released by a RUN store. */
	if (acio->type7 !=
	    (of_device_is_compatible(pdev->dev.of_node, "apple,t8142-usb4-nhi") ||
	     of_device_is_compatible(pdev->dev.of_node, "apple,t6040-usb4-nhi") ||
	     of_device_is_compatible(pdev->dev.of_node, "apple,t6050-usb4-nhi")) ||
	    acio->hw->run_store !=
	    !!of_device_is_compatible(pdev->dev.of_node, "apple,t6050-usb4-nhi")) {
		ret = dev_err_probe(&pdev->dev, -EINVAL, "ACIO/NHI generation mismatch\n");
		goto err;
	}

	/*
	 * The rings must never run untranslated. The DART is populated with
	 * the NHI, so it is normally bound first; but once the driver core
	 * has stopped waiting for missing suppliers (DART driver not loaded
	 * yet), the NHI would probe without its IOMMU. Defer, and fail the
	 * ACIO start now instead of letting it wait for the NHI to time out.
	 */
	if (of_property_present(pdev->dev.of_node, "iommus") &&
	    !device_iommu_mapped(&pdev->dev)) {
		dev_warn(&pdev->dev, "DART not bound yet, not starting the NHI\n");
		ret = -EPROBE_DEFER;
		goto err;
	}

	anhi = devm_kzalloc(&pdev->dev, sizeof(*anhi), GFP_KERNEL);
	if (!anhi) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->pdev = pdev;
	anhi->dev = &pdev->dev;
	anhi->np = pdev->dev.of_node;
	anhi->acio = acio;
	INIT_DELAYED_WORK(&anhi->dp_aux_work, apple_dp_aux_work);
	platform_set_drvdata(pdev, anhi);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(42));
	if (ret)
		goto err;
	if (acio->type7 &&
	    of_property_count_u32_elems(anhi->np, "apple,tunable-nhi") <= 0) {
		ret = dev_err_probe(anhi->dev, -EINVAL,
				    "Missing bootloader NHI tuning\n");
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nhi");
	anhi->nhi_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(anhi->nhi_base)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->nhi_base),
				    "Unable to map NHI regs\n");
		goto err;
	}
	tunable = devm_apple_tunable_parse(&pdev->dev, anhi->np, "apple,tunable-nhi", res);
	if (IS_ERR(tunable)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(tunable), "Unable to load NHI tunable\n");
		goto err;
	}
	anhi->nhi_tunable = tunable;
	apple_tunable_apply(anhi->nhi_base, tunable);

	/* Type 7 keeps the PDF masks in the receive ring registers only. */
	if (!acio->type7) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pdf");
		anhi->pdf_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(anhi->pdf_base)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(anhi->pdf_base),
					    "Unable to map PDF regs\n");
			goto err;
		}
	}

	ret = apple_nhi_probe_irqs(anhi);
	if (ret)
		goto err;

	spin_lock_init(&anhi->nhi.lock);
	anhi->nhi.iommu_dma_protection = true;
	/*
	 * DP tunnel display support (and with it the host DP IN handling in
	 * the connection manager) only where it is enabled and known to work.
	 */
	anhi->ops = apple_nhi_ops;
	anhi->nhi.host_dp_dual_stream = acio->dp_dual_stream;
	dp_hooks = apple_dpin_hooks(acio->dp, !!acio->dp_wq, dp_display);
	if (dp_hooks == APPLE_DPIN_CHANGED) {
		anhi->ops.dp_tunnel_changed = apple_nhi_dp_tunnel_changed;
		anhi->nhi.host_dp_policy = acio->dp->host_policy;
		if (acio->dp->capacity_retry)
			anhi->ops.dp_tunnel_awaits_display =
				apple_nhi_dp_tunnel_awaits_display;
	}
	if (dp_hooks == APPLE_DPIN_PRE_POST) {
		anhi->ops.dp_tunnel_pre_activate = apple_nhi_dp_tunnel_pre_activate;
		anhi->ops.dp_tunnel_post_activate = apple_nhi_dp_tunnel_post_activate;
		anhi->ops.dp_tunnel_deactivate = apple_nhi_dp_tunnel_deactivate;
		anhi->nhi.host_dp_policy = acio->dp->host_policy;
	}
	ret = apple_nhi_cio_bind(anhi);
	if (ret)
		goto err;
	anhi->nhi.ops = &anhi->ops;
	anhi->nhi.ring_layout = &apple_nhi_ring_layout;
	anhi->nhi.iobase = anhi->nhi_base;
	anhi->nhi.quirks = QUIRK_NO_DMA_PORT | QUIRK_NO_USB3_BW_ALLOC |
			   QUIRK_TMU_HIFI_BI;
	if (anhi->nhi.host_dp_policy)
		anhi->nhi.quirks |= QUIRK_HOST_DP_NFC_CREDITS;
	if (acio->hw->no_host_clx)
		anhi->nhi.quirks |= QUIRK_NO_HOST_CLX;
	if (acio->hw->no_asym)
		anhi->nhi.quirks |= QUIRK_NO_ASYM;
	if (acio->hw->gen4_link_recovery)
		anhi->nhi.quirks |= QUIRK_GEN4_LINK_RECOVERY;
	if (acio->hw->host_tmu)
		anhi->nhi.quirks |= QUIRK_HOST_TMU;

	/*
	 * TEST: T6050 takes the vendor sleep and the state transfer as the J713 does (macOS,
	 * runs mxt4 and mxt5: the exchanges behind the firmware's sleep state and before its
	 * "Waking up from sleep"), and its wake waits for the link no longer than a cold
	 * start's training takes.
	 */
	/*
	 * The control frames of that sleep and wake are in run mxt6: the request and the state
	 * transfer are what was inferred, and what macOS sends around them is sent here too.
	 */
	if (anhi->cio_t6050)
		anhi->nhi.quirks |= QUIRK_CIO_WAKE_GIVES_UP | QUIRK_CIO_MACOS_FRAMES;

	anhi->nhi.hop_count = readl(anhi->nhi_base + APPLE_CIO_NHI_HOP_COUNT) &
			      APPLE_CIO_NHI_HOP_COUNT_MASK;
	if (anhi->nhi.hop_count != anhi->n_rings) {
		ret = dev_err_probe(anhi->dev, -EINVAL, "Ring IRQs (%zd) != HOP_COUNT (%d)\n",
				    anhi->n_rings, anhi->nhi.hop_count);
		goto err;
	}

	anhi->nhi.tx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.tx_rings), GFP_KERNEL);
	anhi->nhi.rx_rings = devm_kcalloc(&pdev->dev, anhi->nhi.hop_count,
					  sizeof(*anhi->nhi.rx_rings), GFP_KERNEL);
	if (!anhi->nhi.tx_rings || !anhi->nhi.rx_rings) {
		ret = -ENOMEM;
		goto err;
	}

	anhi->nhi.dev = &pdev->dev;
	init_completion(&anhi->nhi.domain_released);
	anhi->tb = tb_probe(&anhi->nhi);
	if (!anhi->tb) {
		ret = dev_err_probe(anhi->dev, -ENODEV,
				    "Failed to init software connection manager\n");
		goto err;
	}

	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		acio->pcie_tunnel_stopping = false;
		acio->pcie_pm_prepared = false;
		acio->pcie_resume_expected = false;
	}
	ret = tb_domain_add(anhi->tb, false);
	if (ret) {
		dev_err_probe(anhi->dev, ret, "failed to add TB domain\n");
		apple_cio_stop_pcie_tunnel(acio);
		tb_domain_put(anhi->tb);
		wait_for_completion(&anhi->nhi.domain_released);
		goto err;
	}

	mutex_lock(&anhi->tb->lock);

	if (!anhi->tb->root_switch->drom) {
		dev_err(anhi->dev, "No valid host DROM in the device tree\n");
		ret = -EINVAL;
		goto err_unlock_tb_domain;
	}

	cap_apple = tb_switch_find_vse_cap(anhi->tb->root_switch, TB_VSE_CAP_APPLE);

	if (cap_apple < 0) {
		dev_err(anhi->dev, "Unable to find VSE Apple capability: %d\n",
			cap_apple);
		ret = cap_apple;
		goto err_unlock_tb_domain;
	}

	/*
	 * The ports are locked after reset. Unlock them before writing the cable information which
	 * will bring up the link and start the first scan such that XDomain responses are not
	 * rejected by our own router.
	 */
	tb_switch_for_each_port(anhi->tb->root_switch, port) {
		if (!tb_port_is_null(port))
			continue;
		ret = tb_port_unlock(port);
		if (ret)
			dev_warn(anhi->dev, "Failed to unlock port %d: %d\n",
				 port->port, ret);
	}

	ret = tb_sw_write(anhi->tb->root_switch, &acio->target_cable_info, TB_CFG_SWITCH,
			  cap_apple + TB_VSE_CAP_APPLE_CABLE_INFO, 1);
	if (ret) {
		dev_warn(anhi->dev, "Setting VSE Apple cable info failed: %d\n", ret);
		goto err_unlock_tb_domain;
	}

	mutex_unlock(&anhi->tb->lock);

	WRITE_ONCE(acio->nhi_pdev, pdev);
	acio->nhi_boot_status = 0;
	complete(&acio->nhi_boot_completion);
	return 0;

err_unlock_tb_domain:
	mutex_unlock(&anhi->tb->lock);
	disable_delayed_work_sync(&anhi->dp_aux_work);
	apple_cio_stop_pcie_tunnel(acio);
	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
err:
	acio->nhi_boot_status = ret;
	complete(&acio->nhi_boot_completion);
	return ret;
}

static void apple_dpin_end_pm_gate(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	/* Domain removal stopped producers. No DP-IN/domain/fabric lock held. */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		cancel_delayed_work_sync(&c->retry);
		flush_work(&c->work);
		scoped_guard(mutex, &c->lock)
			apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_END_PM_GATE,
					!!c->regs, 0);
	}
}

static void apple_nhi_remove(struct platform_device *pdev)
{
	struct apple_nhi *anhi = platform_get_drvdata(pdev);

	/* Stop racing tunnel activation from rearming work during removal. */
	disable_delayed_work_sync(&anhi->dp_aux_work);
	apple_cio_stop_pcie_tunnel(anhi->acio);
	WRITE_ONCE(anhi->acio->nhi_pdev, NULL);
	tb_domain_remove(anhi->tb);
	wait_for_completion(&anhi->nhi.domain_released);
	/* A removed prepared device may never receive complete(). */
	apple_dpin_end_pm_gate(anhi->acio);
}

/*
 * The Apple platform driver owns the NHI allocation and keeps an
 * apple_nhi pointer in driver data. The generic NHI PM callbacks cannot be
 * installed here because they expect driver data to contain struct tb.
 */
/*
 * A tunnel waiting for a display pipeline asks appledrm again every few
 * seconds.  Not while devices suspend and resume: stop asking before any of
 * them suspends, let an attempt in flight finish (a wait for appledrm to
 * load gives way, see apple_dpin_connect()), and ask again once all of them
 * have resumed. The tunnel keeps waiting meanwhile. Suspend-gated handoffs
 * are deferred until complete, including after an aborted suspend. Established
 * handed callbacks and synchronous tunnel teardown remain permitted.
 */
static void apple_dpin_pause_retries(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		scoped_guard(mutex, &c->lock)
			apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_PAUSE,
					!!c->regs, 0);
	}
	/* Close admission on both adapters before draining either one. */
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		cancel_delayed_work_sync(&acio->dpin[i].retry);
		flush_work(&acio->dpin[i].work);
	}
}

static void apple_dpin_resume_retries(struct apple_cio *acio)
{
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];
		unsigned int actions;

		guard(mutex)(&c->lock);
		actions = apple_dpin_step(&c->state, acio->dp,
					  APPLE_DPIN_RESUME, !!c->regs, 0);
		if (actions & APPLE_DPIN_QUEUE)
			queue_work(acio->dp_wq, &c->work);
		if (actions & APPLE_DPIN_ARM_RETRY)
			mod_delayed_work(acio->dp_wq, &c->retry,
					 msecs_to_jiffies(APPLE_DPIN_RETRY_MS));
	}
}

static int apple_nhi_prepare(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);
	struct apple_cio *acio = anhi->acio;
	bool recovery = READ_ONCE(resume_recovery);
	/*
	 * Only a sleep that may cost the router its tunnels expects them back:
	 * one with resume recovery, as before, and the J713's native deep sleep,
	 * which power-cycles the router.
	 */
	bool cycles = recovery ||
		      (anhi->cio_dart && pm_suspend_target_state == PM_SUSPEND_MEM);
	bool expected;
	int ret;

	acio->resume_had_peer = false;
	/* Only resume recovery reads it: don't take the domain lock otherwise. */
	if (recovery) {
		scoped_guard(mutex, &anhi->tb->lock) {
			struct tb_port *port;

			tb_switch_for_each_port(anhi->tb->root_switch, port) {
				if (tb_port_is_null(port) &&
				    ((port->xdomain && !port->xdomain->is_unplugged) ||
				     (port->remote && !port->remote->sw->is_unplugged)))
					acio->resume_had_peer = true;
			}
		}
	}

	/* Worker admission and native PCIe PM ordering are required on every sleep. */
	apple_dpin_pause_retries(acio);

	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		expected = acio->pcie_resume_expected;
		/* Keep an unresolved expectation until cable revalidation. */
		if (cycles)
			acio->pcie_resume_expected |= READ_ONCE(acio->pcie_tunnel_requested);
		acio->pcie_pm_prepared = true;
	}
	ret = apple_nhi_cio_prepare(anhi);
	if (ret) {
		apple_dpin_resume_retries(acio);
		scoped_guard(mutex, &acio->pcie_tunnel_lock) {
			acio->pcie_pm_prepared = false;
			acio->pcie_resume_expected = expected;
		}
	}
	return ret;
}

/*
 * Router sleep saves power but ACIO retraining can stall after resume.
 * Keep it opt-in until remote-transport recovery is qualified on hardware.
 * An explicit Y also drops KEEP_TUNNELS and rebuilds tunnels on resume.
 */
static bool router_sleep;
module_param(router_sleep, bool, 0644);
MODULE_PARM_DESC(router_sleep,
		 "Ask routers to sleep on system suspend (default N; experimental, rebuilds tunnels)");

static bool apple_nhi_pcie_link_kept(struct apple_cio *acio)
{
	struct platform_device *pcie_pdev;
	bool kept = false;

	guard(mutex)(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (pcie_pdev) {
		kept = apple_pcie_tunnel_link_kept(&pcie_pdev->dev);
		put_device(&pcie_pdev->dev);
	}
	return kept;
}

/* Whether this port has a PCIe-C tunnel host at all. */
static bool apple_nhi_has_pcie_tunnel(struct apple_cio *acio)
{
	struct platform_device *pcie_pdev;

	guard(mutex)(&acio->pcie_tunnel_lock);
	pcie_pdev = apple_cio_find_pcie_tunnel(acio);
	if (pcie_pdev)
		put_device(&pcie_pdev->dev);
	return pcie_pdev;
}

static int apple_nhi_suspend_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	if (anhi->cio_prepared)
		return apple_nhi_cio_suspend(anhi);
	anhi->nhi.quirks &= ~(QUIRK_NO_SYSTEM_SLEEP | QUIRK_KEEP_TUNNELS);
	if (!READ_ONCE(router_sleep)) {
		anhi->nhi.quirks |= QUIRK_NO_SYSTEM_SLEEP;
		/* PCIe-C suspends first; its tunnel must then survive resume. */
		if (apple_nhi_pcie_link_kept(anhi->acio))
			anhi->nhi.quirks |= QUIRK_KEEP_TUNNELS;
		/*
		 * A DP tunnel to an external display behind a dock is never
		 * released here: that would have its DCP release the route (its
		 * locks, AFK calls and waits) with the DCP already suspended. With
		 * the routers awake every tunnel stays as it is: keep them all,
		 * unless a PCIe-C tunnel's link is not kept (it is then rebuilt at
		 * resume, which tears down the others too): refuse the sleep.
		 */
		if (!(anhi->nhi.quirks & QUIRK_KEEP_TUNNELS) &&
		    tb_has_apple_host_dp_tunnel(anhi->tb)) {
			if (apple_nhi_has_pcie_tunnel(anhi->acio)) {
				dev_warn(dev, "refusing system sleep: a DP tunnel to an external display cannot be kept (the PCIe-C link is not kept)\n");
				return -EBUSY;
			}
			anhi->nhi.quirks |= QUIRK_KEEP_TUNNELS;
			dev_dbg(dev, "tunnels kept across sleep for an external display\n");
		}
	} else if (tb_has_apple_host_dp_tunnel(anhi->tb)) {
		/* The routers sleep: the DP tunnels would go here, see above. */
		dev_warn(dev, "refusing system sleep: a DP tunnel to an external display is up and router_sleep is set\n");
		return -EBUSY;
	}

	return tb_domain_suspend_noirq(anhi->tb);
}

static int apple_nhi_resume_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	if (anhi->cio_prepared)
		return apple_nhi_cio_resume(anhi);
	anhi->acio->resume_activity = atomic64_read(&anhi->tb->remote_activity);
	WRITE_ONCE(anhi->acio->resume_link_pending, true);
	return tb_domain_resume_noirq(anhi->tb);
}

static int apple_nhi_freeze_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_freeze_noirq(anhi->tb);
}

static int apple_nhi_thaw_noirq(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_thaw_noirq(anhi->tb);
}

static int apple_nhi_suspend(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);

	return tb_domain_suspend(anhi->tb);
}

static void apple_nhi_complete(struct device *dev)
{
	struct apple_nhi *anhi = dev_get_drvdata(dev);
	struct apple_cio *acio = anhi->acio;

	apple_nhi_cio_complete(anhi);
	tb_domain_complete(anhi->tb);
	/*
	 * A dark wake sleeps again at once and lights no display: DP IN admission, paused again by
	 * the next .prepare, stays paused through the gap and reopens at the final wake.
	 */
	if (!pm_dark_wake())
		apple_dpin_resume_retries(acio);

	guard(mutex)(&acio->pcie_tunnel_lock);
	acio->pcie_pm_prepared = false;
	if (!acio->pcie_tunnel_stopping &&
	    (acio->pcie_quiesce_pending || READ_ONCE(acio->pcie_tunnel_requested)))
		mod_delayed_work(system_freezable_wq, &acio->pcie_tunnel_work, 0);
}

static const struct dev_pm_ops apple_nhi_pm_ops = {
	.prepare = apple_nhi_prepare,
	.suspend = apple_nhi_suspend,
	.suspend_noirq = apple_nhi_suspend_noirq,
	.resume_noirq = apple_nhi_resume_noirq,
	.freeze_noirq = apple_nhi_freeze_noirq,
	.thaw_noirq = apple_nhi_thaw_noirq,
	.restore_noirq = apple_nhi_resume_noirq,
	.poweroff = apple_nhi_suspend,
	.poweroff_noirq = apple_nhi_suspend_noirq,
	.complete = apple_nhi_complete,
};

static const struct of_device_id apple_nhi_match[] = {
	{
		.compatible = "apple,t6040-usb4-nhi",
	},
	{
		.compatible = "apple,t6050-usb4-nhi",
	},
	{
		.compatible = "apple,t8142-usb4-nhi",
	},
	{
		.compatible = "apple,t8103-usb4-nhi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_nhi_match);

static struct platform_driver apple_nhi_driver = {
	.driver = {
		.name = "thunderbolt-apple-nhi",
		.of_match_table = apple_nhi_match,
		.pm = &apple_nhi_pm_ops,
	},
	.probe = apple_nhi_probe,
	.remove = apple_nhi_remove,
};

static struct platform_device *apple_cio_find_pcie_tunnel(struct apple_cio *acio)
{
	struct platform_device *pdev = NULL;

	if (!acio->pcie_tunnel_np || !acio->pcie_tunnel_populated)
		return NULL;

	for_each_available_child_of_node_scoped(acio->pcie_tunnel_np, child) {
		if (!apple_cio_pciec_eligible(child))
			continue;

		pdev = of_find_device_by_node(child);
		break;
	}

	return pdev;
}

static void apple_cio_remove_children(struct apple_cio *acio)
{
	struct platform_device *nhi_pdev;

	lockdep_assert_held(&acio->lock);

	/*
	 * Stop requests before removing the NHI. Do not hold pcie_tunnel_lock
	 * across domain removal: it takes tb->lock, which tunnel callbacks hold
	 * when they acquire pcie_tunnel_lock.
	 */
	apple_cio_stop_pcie_tunnel(acio);

	/*
	 * The NHI is the tunnel control plane. Stop it while the tunneled PCIe
	 * host and DART are still accessible, before depopulation tears down the
	 * data plane. Otherwise tb_domain_remove() walks the USB4 topology after
	 * PCIe-C has already reset its port and can wedge waiting on dead control
	 * traffic. The remaining children are still removed in reverse creation
	 * order below.
	 */
	nhi_pdev = READ_ONCE(acio->nhi_pdev);
	if (nhi_pdev)
		of_platform_device_destroy(&nhi_pdev->dev, NULL);
	/*
	 * NHI removal has released tb->lock and pcie_tunnel_lock. DP work and
	 * its DCP/crossbar/PHY callbacks never acquire acio->lock: they take
	 * only the DP-IN lock on this controller. Keep that invariant for
	 * future monitor/reconnect work; this drain holds acio->lock.
	 */
	if (acio->dp_wq)
		flush_workqueue(acio->dp_wq);

	of_platform_depopulate(acio->dev);
	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		acio->pcie_tunnel_populated = false;
		acio->pcie_quiesce_pending = false;
	}
}

/*
 * While the router is off, the AP holds LSTX at the level the router last
 * drove. The pin goes back to the router once it is up again.
 */
static int apple_cio_lstx_hold(struct apple_cio *acio, bool high)
{
	struct gpio_desc *gpio;

	if (!acio->lstx_hold)
		return 0;
	acio->lstx_high = high;

	/* The pin controller is strict: release the router's mux first. */
	if (acio->pinctrl) {
		devm_pinctrl_put(acio->pinctrl);
		acio->pinctrl = NULL;
		acio->lstx_pins = NULL;
	}
	gpio = devm_gpiod_get(acio->dev, "lstx",
			      high ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		acio->lstx_high = false;
		dev_warn(acio->dev, "failed to take the LSTX pin back: %ld\n",
			 PTR_ERR(gpio));
		return PTR_ERR(gpio);
	}
	acio->lstx_gpio = gpio;
	return 0;
}

static int apple_cio_lstx_release(struct apple_cio *acio)
{
	writel(APPLE_CIO_LSTX_CTRL_ENABLE |
	       (acio->lstx_high ? APPLE_CIO_LSTX_CTRL_LEVEL : 0),
	       acio->rc_base + APPLE_CIO_M3_CTRL);
	if (acio->lstx_gpio) {
		devm_gpiod_put(acio->dev, acio->lstx_gpio);
		acio->lstx_gpio = NULL;
	}
	if (!acio->pinctrl) {
		struct pinctrl *pinctrl = devm_pinctrl_get(acio->dev);
		struct pinctrl_state *state;

		if (IS_ERR(pinctrl))
			return PTR_ERR(pinctrl);
		acio->pinctrl = pinctrl;
		state = pinctrl_lookup_state(pinctrl, "lstx");
		if (IS_ERR(state))
			return PTR_ERR(state);
		acio->lstx_pins = state;
	}
	if (!acio->lstx_pins)
		return 0;
	return pinctrl_select_state(acio->pinctrl, acio->lstx_pins);
}

/*
 * Halt a run_store router CPU that apple_cio_start() released. Its wrapper is
 * only clocked after the CIO reset, so never touch it otherwise.
 */
static void apple_cio_cpu_halt(struct apple_cio *acio)
{
	u32 val;

	if (!acio->hw->run_store || !acio->cpu_running)
		return;
	acio->cpu_running = false;
	val = readl(acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
	writel(val & ~APPLE_MXWRAP_CPU_CTRL_RUN,
	       acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
}

/* TEST (sv6t7): the PS_PROT word of the slice a DART node names as its clock protection. */
static int apple_cio_test_slice(struct device_node *np, unsigned int *slice, u32 *word)
{
	struct of_phandle_args args;
	struct regmap *map;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, "apple,clock-protection", 1, 0, &args);
	if (ret)
		return ret;
	map = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(map))
		return PTR_ERR(map);
	*slice = args.args[0];
	return regmap_read(map, 0x100 + 0x40 * args.args[0], word);
}

/*
 * TEST (sv6t7): as the ACIO drops its domain links, the guard word of each of the port's
 * DARTs (bit 4 set: the domain's power-off faults) and genpd's view of each domain.
 */
static void apple_cio_test_log_guards(struct apple_cio *acio)
{
	struct device_node *parents[] = { acio->np, acio->pcie_tunnel_np };
	char buf[384];
	int len = 0, i;

	if (!of_machine_is_compatible("apple,j714s"))
		return;
	buf[0] = '\0';
	for (i = 0; i < ARRAY_SIZE(parents); i++) {
		if (!parents[i])
			continue;
		for_each_available_child_of_node_scoped(parents[i], child) {
			unsigned int slice = 0;
			u32 word = 0;
			int ret;

			if (!of_property_present(child, "apple,clock-protection"))
				continue;
			ret = apple_cio_test_slice(child, &slice, &word);
			len += scnprintf(buf + len, sizeof(buf) - len, " %pOFn slice %u %#x (%d);",
					 child, slice, word, ret);
		}
	}
	len += scnprintf(buf + len, sizeof(buf) - len, " domains:");
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		struct device *pd = acio->pd_list->pd_devs[i];
		struct generic_pm_domain *genpd =
			pd && pd->pm_domain ? pd_to_genpd(pd->pm_domain) : NULL;

		len += scnprintf(buf + len, sizeof(buf) - len, " %s %s%s",
				 genpd ? genpd->name : "?",
				 genpd && genpd->status == GENPD_STATE_ON ? "on" : "off",
				 acio->pd_list->pd_links[i] ? "" : " (no link)");
	}
	dev_info(acio->dev, "TEST sv6t7: ACIO stop, its domain links go next: PS_PROT%s\n", buf);
}

/**
 * apple_cio_stop - Stop and power down the ACIO complex
 * @acio: ACIO complex to stop
 *
 * Remove the NHI and DART children before shutting down the co-processor and
 * dropping the power-domain links. The Type-C PHY must remain configured
 * until this function returns. The caller must hold @acio->lock.
 */
static void apple_cio_stop(struct apple_cio *acio)
{
	int ret, i;

	lockdep_assert_held(&acio->lock);
	if (!acio->current_cable_info)
		return;
	apple_cio_remove_children(acio);

	/* Try to shut down and power off the co-processor gracefully */
	ret = apple_rtkit_poweroff(acio->rtk);
	if (ret)
		dev_warn(acio->dev,
			 "Failed to shutdown M3 RTKit, continuing ACIO shutdown anyway\n");
	apple_rtkit_free(acio->rtk);
	acio->rtk = NULL;
	apple_cio_cpu_halt(acio);

	/* High only if the router left it driven high with the cable still in */
	if (acio->hw->lstx_after_boot)
		apple_cio_lstx_hold(acio, acio->target_cable_info &&
				    (readl(acio->rc_base + APPLE_CIO_M3_CTRL) &
				     APPLE_CIO_LSTX_CTRL_LEVEL));

	apple_cio_test_log_guards(acio);

	/* Finally, remove the links to the PD domains to power everything off */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], false);
	}

	acio->current_cable_info = 0;
	acio->resume_link_failed = false;
	acio->link_lost_restart = false;
}

/*
 * Only called while the ACIO power domains are on. Neither a T6050 router's rc
 * block nor its CPU wrapper is known to be readable while it starts (see
 * apple_cio_start()), so log nothing for it.
 */
static void apple_cio_log_state(struct apple_cio *acio, const char *when)
{
	if (acio->hw->run_store)
		return;
	if (acio->cpu_base)
		dev_info(acio->dev, "%s: LSTX 0x%08x, FW state 0x%08x, CPU ctrl 0x%08x, status 0x%08x\n",
			 when, readl(acio->rc_base + APPLE_CIO_M3_CTRL),
			 readl(acio->rc_base + APPLE_CIO_M3_STAT),
			 readl(acio->cpu_base + APPLE_MXWRAP_CPU_CTRL),
			 readl(acio->cpu_base + APPLE_MXWRAP_CPU_STATUS));
	else
		dev_info(acio->dev, "%s: LSTX 0x%08x, FW state 0x%08x\n", when,
			 readl(acio->rc_base + APPLE_CIO_M3_CTRL),
			 readl(acio->rc_base + APPLE_CIO_M3_STAT));
}

/*
 * Whether a forced cable cycle for resume recovery is under way. One that
 * CD321x did not take up within the window (it refused, or a suspend came
 * first) no longer counts, so a later unrelated start gets every attempt.
 * Called with acio->lock held.
 */
static bool apple_cio_recovering(struct apple_cio *acio)
{
	if (acio->resume_recovery_active &&
	    time_after(jiffies, acio->resume_recovery_until))
		acio->resume_recovery_active = false;
	return acio->resume_recovery_active;
}

/**
 * apple_cio_start - Power up and start the ACIO complex
 * @acio: ACIO complex to start
 *
 * The Type-C PHY must already be configured for USB4 or Thunderbolt. Power up
 * ACIO, boot its RTKit co-processor, populate its child devices and wait for
 * the NHI to register the USB4 domain. All completed steps are unwound on
 * failure. The caller must hold @acio->lock.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_start(struct apple_cio *acio)
{
	struct device_link *link;
	int i, ret, attempts;
	u32 state, val;

	lockdep_assert_held(&acio->lock);

	/*
	 * PCIe-C's power domain stays on across cable teardown, so apart from
	 * this reset nothing returns the controller to a known state between
	 * tunnels. After system sleep with a Thunderbolt 3 dock, state left in
	 * it has kept every later link from training until reboot. Reset it
	 * while its host and DART are unbound and before ACIO's domains come
	 * up, in the same order as after boot.
	 */
	if (acio->pcie_reset) {
		ret = reset_control_acquire(acio->pcie_reset);
		if (ret)
			return dev_err_probe(acio->dev, ret,
					     "PCIe-C reset is still in use\n");
		ret = reset_control_reset(acio->pcie_reset);
		reset_control_release(acio->pcie_reset);
		if (ret)
			return dev_err_probe(acio->dev, ret,
					     "failed to reset PCIe-C\n");
		dev_info(acio->dev, "PCIe-C reset before ACIO start\n");
	}

	/* Create device links to the power domains in order to power them on */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		link = device_link_add(acio->dev, acio->pd_list->pd_devs[i],
				       DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!link) {
			ret = -ENODEV;
			goto remove_links;
		}
		acio->pd_list->pd_links[i] = link;
		/*
		 * ACIO owns these domains explicitly for the lifetime of the cable
		 * connection.  The virtual genpd devices must not independently
		 * cycle them during system sleep: doing so destroys the live M3 and
		 * NHI state behind the still-bound child devices.  Cable teardown
		 * removes the runtime-active links and gives system PM ownership back.
		 */
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], true);
	}

	/*
	 * After the power domains are on we need to signal and wait for the ACIO block
	 * to actually start before we can bring up the co-processor.
	 */
	attempts = apple_cio_recovering(acio) ? 1 : APPLE_CIO_START_RETRIES;
	for (i = 0; i < attempts; i++) {
		ret = reset_control_deassert(acio->reset);
		if (!ret)
			break;
		if (i + 1 < attempts) {
			dev_warn(acio->dev,
				 "ACIO block failed to start (attempt %d/%d): %d, retrying\n",
				 i + 1, attempts, ret);
			msleep(APPLE_CIO_START_RETRY_DELAY_MS);
		}
	}
	if (ret) {
		dev_err(acio->dev, "ACIO block failed to start: %d\n", ret);
		goto remove_links;
	}

	/*
	 * Start and wait for the co-processor to boot.
	 *
	 * iBoot can leave other bits of this register set (0x3 has been observed
	 * on t6020 before the kernel touches it). The
	 * remaining fields are undocumented, so preserve them rather than clearing
	 * them with a bare store.
	 */
	if (acio->hw->lstx_after_boot) {
		/*
		 * Written below, once RTKit is up. A T6050 CPU is started through
		 * its wrapper below, and its rc block takes an external abort until
		 * the CPU runs, so M3_CTRL is left alone here.
		 */
	} else if (acio->hw->lstx_explicit) {
		writel(APPLE_CIO_LSTX_CTRL_ENABLE | APPLE_CIO_LSTX_CTRL_LEVEL,
		       acio->rc_base + APPLE_CIO_M3_CTRL);
	} else {
		val = readl(acio->rc_base + APPLE_CIO_M3_CTRL);
		writel(val | APPLE_CIO_M3_CTRL_START, acio->rc_base + APPLE_CIO_M3_CTRL);
	}

	if (acio->hw->mxwrap && !acio->hw->skip_run) {
		apple_cio_log_state(acio, "before RUN");
		if (acio->hw->run_store) {
			/*
			 * The CIO reset above clocks the wrapper; the CPU runs the
			 * image the boot loader left in its SRAM once RUN is set.
			 * The run value is stored outright, not merged.
			 */
			writel(APPLE_MXWRAP_CPU_CTRL_RUN,
			       acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
			acio->cpu_running = true;
			apple_cio_log_state(acio, "after RUN");
		} else {
			val = readl(acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
			writel(val | APPLE_MXWRAP_CPU_CTRL_RUN,
			       acio->cpu_base + APPLE_MXWRAP_CPU_CTRL);
		}
	}

	acio->rtk = apple_rtkit_init(acio->dev, acio, NULL, 0, &apple_cio_rtkit_ops);
	if (IS_ERR(acio->rtk)) {
		ret = PTR_ERR(acio->rtk);
		dev_err(acio->dev, "Failed to initialize RTKit: %d\n", ret);
		goto remove_links;
	}

	ret = apple_rtkit_boot(acio->rtk);
	if (ret) {
		dev_err(acio->dev, "M3 RTKit failed to boot: %d\n", ret);
		apple_cio_log_state(acio, "RTKit boot failed");
		goto err_free_rtkit;
	}

	/*
	 * The router takes LSTX over at the level it last drove, low after a
	 * cold start, and then gets its pin back from the AOP GPIO controller.
	 */
	if (acio->hw->lstx_after_boot) {
		ret = apple_cio_lstx_release(acio);
		if (ret) {
			dev_err(acio->dev, "failed to hand over the LSTX pin: %d\n", ret);
			goto err_shutdown_rtkit;
		}
	}

	if (acio->hw->strict_fw_ready)
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 (state & APPLE_CIO_M3_STAT_CODE_VALID) ||
					 FIELD_GET(APPLE_CIO_M3_STAT_STATE, state) ==
					 APPLE_CIO_M3_STAT_STATE_READY,
					 100, 500000);
	else
		ret = readl_poll_timeout(acio->rc_base + APPLE_CIO_M3_STAT, state,
					 acio->type7 ?
				 (state & APPLE_CIO_M3_STAT_STATE) == BIT(24) :
				 (state & APPLE_CIO_M3_STAT_STATE), 100, 500000);
	if (!ret && acio->hw->strict_fw_ready && (state & APPLE_CIO_M3_STAT_CODE_VALID)) {
		dev_err(acio->dev, "M3 firmware assert, code 0x%06lx (state 0x%08x)\n",
			FIELD_GET(APPLE_CIO_M3_STAT_CODE, state), state);
		ret = -EIO;
	}
	if (ret < 0) {
		dev_err(acio->dev, "M3 firmware failed to get ready: %d\n", ret);
		apple_cio_log_state(acio, "firmware not ready");
		goto err_shutdown_rtkit;
	}
	if (acio->hw->mxwrap)
		apple_cio_log_state(acio, "firmware ready");

	apple_tunable_apply(acio->rc_base, acio->rc_tunable);
	if (acio->pcie_adapter_base && acio->pcie_adapter_tunable) {
		dev_info(acio->dev, "applying PCIe adapter tunable\n");
		apple_tunable_apply(acio->pcie_adapter_base,
				    acio->pcie_adapter_tunable);
	}
	/* The router AP firmware tunable goes with the others once the firmware is ready. */
	apple_tunable_apply(acio->router_ap_fw_base, acio->router_ap_fw_tunable);

	/*
	 * Bring up devices which are part of ACIO and are now accessible by the main SoC
	 * and specifically wait for the NHI to be up to prevent concurrent shutdowns.
	 */
	reinit_completion(&acio->nhi_boot_completion);
	ret = of_platform_populate(acio->np, NULL, NULL, acio->dev);
	if (ret) {
		dev_err(acio->dev, "failed to populate children: %d\n", ret);
		goto err_depopulate;
	}

	if (!wait_for_completion_timeout(&acio->nhi_boot_completion,
					 msecs_to_jiffies(APPLE_CIO_NHI_BOOT_TIMEOUT_MS))) {
		dev_err(acio->dev, "Timed out waiting for the NHI to come up\n");
		ret = -ETIMEDOUT;
		goto err_depopulate;
	}
	if (acio->nhi_boot_status) {
		ret = acio->nhi_boot_status;
		goto err_depopulate;
	}

	acio->current_cable_info = acio->target_cable_info;
	acio->start_gen++;
	return 0;

err_depopulate:
	apple_cio_remove_children(acio);
err_shutdown_rtkit:
	/* Ignore errors here since we're about to cut power to the entire block anyway */
	apple_rtkit_poweroff(acio->rtk);
	if (acio->hw->lstx_after_boot)
		apple_cio_lstx_hold(acio, false);
err_free_rtkit:
	apple_rtkit_free(acio->rtk);
remove_links:
	acio->rtk = NULL;
	apple_cio_cpu_halt(acio);
	/* Cut power to reset the entire block  */
	for (i = 0; i < acio->pd_list->num_pds; i++) {
		if (acio->pd_list->pd_links[i])
			device_link_del(acio->pd_list->pd_links[i]);
		acio->pd_list->pd_links[i] = NULL;
		dev_pm_syscore_device(acio->pd_list->pd_devs[i], false);
	}
	return ret;
}

static int apple_cio_check_connection(struct apple_cio *acio)
{
	struct platform_device *pdev;
	int ret;

	lockdep_assert_held(&acio->lock);
	/* Ask CD321x to cycle ACIO and the PHY using a fresh cable snapshot. */
	if (acio->resume_link_failed || acio->link_lost_restart)
		return -ENOLINK;
	guard(mutex)(&acio->pcie_tunnel_lock);
	if (!READ_ONCE(acio->nhi_pdev) || acio->pcie_tunnel_stopping ||
	    apple_rtkit_is_crashed(acio->rtk))
		return -ENODEV;
	if (acio->pcie_pm_prepared || acio->pcie_quiesce_pending)
		return -EAGAIN;

	/* A display-only or unauthorized session need not have a PCIe tunnel. */
	if (!READ_ONCE(acio->pcie_tunnel_requested))
		return acio->pcie_resume_expected ? -ENOLINK : 0;
	if (!acio->pcie_tunnel_populated)
		return -EAGAIN;
	pdev = apple_cio_find_pcie_tunnel(acio);
	if (!pdev)
		return acio->pcie_bind_retries ? -EAGAIN : -ENODEV;
	ret = apple_pcie_tunnel_check_state(&pdev->dev);
	put_device(&pdev->dev);
	if (ret == -ENODEV && acio->pcie_bind_retries)
		return -EAGAIN;
	if (!ret)
		acio->pcie_resume_expected = false;
	return ret;
}

static int apple_cio_link_connected(struct apple_cio *acio)
{
	struct platform_device *pdev = READ_ONCE(acio->nhi_pdev);
	struct apple_nhi *anhi;
	struct tb_port *port;
	int verdict = -EAGAIN;
	u64 activity;

	lockdep_assert_held(&acio->lock);
	if (!pdev)
		return -EAGAIN;
	device_lock(&pdev->dev);
	anhi = platform_get_drvdata(pdev);
	if (!pdev->dev.driver || !anhi)
		goto unlock;

	scoped_guard(mutex, &anhi->tb->lock) {
		activity = atomic64_read(&anhi->tb->remote_activity);
		tb_switch_for_each_port(anhi->tb->root_switch, port) {
			if (!tb_port_is_null(port))
				continue;
			dev_dbg(acio->dev,
				"resume lane %u link %u: xdomain registered %d unplugged %d\n",
				port->port, port->link_nr,
				port->xdomain && device_is_registered(&port->xdomain->dev),
				port->xdomain && port->xdomain->is_unplugged);
		}
		/* ACIO exposes one physical cable per domain, including both lanes. */
		verdict = activity != acio->resume_activity ? 0 : -ENOLINK;
		dev_dbg(acio->dev, "resume discovery: activity %llu -> %llu, verdict %s\n",
			acio->resume_activity, activity,
			verdict ? "no fresh discovery (stale UP possible)" :
			"fresh remote exchange");
	}
unlock:
	device_unlock(&pdev->dev);
	return verdict;
}

/* Whether a router or host is connected to a lane adapter of the host router */
static bool apple_cio_has_peer(struct apple_cio *acio, u64 *activity)
{
	struct platform_device *pdev = READ_ONCE(acio->nhi_pdev);
	struct apple_nhi *anhi;
	struct tb_port *port;
	bool peer = true;

	lockdep_assert_held(&acio->lock);
	if (!pdev)
		return true;
	device_lock(&pdev->dev);
	anhi = platform_get_drvdata(pdev);
	if (!pdev->dev.driver || !anhi || !anhi->tb)
		goto unlock;

	scoped_guard(mutex, &anhi->tb->lock) {
		*activity = atomic64_read(&anhi->tb->remote_activity);
		peer = false;
		tb_switch_for_each_port(anhi->tb->root_switch, port) {
			if (!tb_port_is_null(port))
				continue;
			if ((port->xdomain && !port->xdomain->is_unplugged) ||
			    (port->remote && !port->remote->sw->is_unplugged))
				peer = true;
		}
	}
unlock:
	device_unlock(&pdev->dev);
	return peer;
}

/*
 * The peer dropped the link but the cable is still attached and no cable
 * change followed. The firmware leaves the port down until it is powered
 * off, so have CD321x cycle it with a fresh cable snapshot, a few times.
 */
static void apple_cio_link_lost_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(to_delayed_work(work),
					      struct apple_cio, link_lost_work);
	u64 activity = 0;
	bool notify = false;

	scoped_guard(mutex, &acio->lock) {
		if (acio->removing || !acio->current_cable_info ||
		    !acio->target_cable_info ||
		    acio->sleep_phase != APPLE_CIO_SLEEP_OPEN ||
		    acio->resume_recovery_failed ||
		    acio->start_gen != READ_ONCE(acio->link_lost_gen))
			return;
		if (apple_cio_has_peer(acio, &activity)) {
			if (acio->sleep_peer_wanted)
				dev_info(acio->dev, "T6050 sleep: the dock is back after the port's restart (%u more restart(s))\n",
					 acio->link_lost_cycles);
			acio->sleep_peer_wanted = false;
			return;
		}
		if (acio->sleep_peer_wanted)
			dev_warn(acio->dev, "T6050 sleep: no dock %d ms after the port's restart\n",
				 APPLE_CIO_LINK_LOST_MS);
		if (acio->link_lost_cycles >= APPLE_CIO_LINK_LOST_CYCLES) {
			dev_warn(acio->dev,
				 "peer link still down after %u port restarts, leaving the port alone\n",
				 acio->link_lost_cycles);
			acio->sleep_peer_wanted = false;
			return;
		}
		acio->link_lost_cycles++;
		WRITE_ONCE(acio->link_lost_cycle_activity, activity);
		acio->link_lost_restart = true;
		dev_info(acio->dev,
			 "peer link lost with the cable attached, restarting the port (%u/%u)\n",
			 acio->link_lost_cycles, APPLE_CIO_LINK_LOST_CYCLES);
		notify = true;
	}

	/* CD321x may tear down the NHI: never notify under either driver lock. */
	if (notify)
		typec_thunderbolt_switch_notify(acio->tbt_switch);
}

static void apple_cio_resume_link_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(to_delayed_work(work),
						    struct apple_cio, resume_link_work);
	bool notify = false;
	int ret;

	scoped_guard(mutex, &acio->lock) {
		if (!READ_ONCE(resume_recovery) || acio->removing ||
		    !acio->current_cable_info || !acio->target_cable_info ||
		    acio->resume_recovery_failed)
			return;
		ret = apple_cio_link_connected(acio);
		if (!ret || ret == -EAGAIN)
			return;
		if (++acio->resume_link_failures < 3) {
			mod_delayed_work(system_freezable_wq, &acio->resume_link_work,
					 msecs_to_jiffies(APPLE_CIO_RESUME_LINK_CHECK_MS));
			return;
		}
		if (atomic_cmpxchg(&resume_recovery_used, 0, 1)) {
			dev_warn(acio->dev, "resume recovery budget already used; leaving cable alone\n");
			return;
		}
		acio->resume_link_failed = true;
		acio->resume_recovery_active = true;
		acio->resume_recovery_until = jiffies +
			msecs_to_jiffies(APPLE_CIO_RESUME_RECOVERY_WINDOW_MS);
		dev_info(acio->dev, "no fresh remote discovery after resume, revalidating cable once\n");
		notify = true;
	}

	/* CD321x may tear down the NHI: never notify under either driver lock. */
	if (notify)
		typec_thunderbolt_switch_notify(acio->tbt_switch);
}

#include "apple-cio-sleep.c"

static int apple_cio_prepare(struct device *dev)
{
	struct apple_cio *acio = dev_get_drvdata(dev);

	/* Drain before taking lock; the worker acquires it to check the link. */
	cancel_delayed_work_sync(&acio->resume_link_work);
	cancel_delayed_work_sync(&acio->link_lost_work);
	scoped_guard(mutex, &acio->lock) {
		WRITE_ONCE(acio->resume_link_pending, false);
		acio->resume_link_failed = false;
		/* A cycle CD321x did not take up is not carried into this sleep. */
		acio->resume_recovery_active = false;
	}
	return 0;
}

static void apple_cio_complete(struct device *dev)
{
	struct apple_cio *acio = dev_get_drvdata(dev);

	guard(mutex)(&acio->lock);
	if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED)
		acio->sleep_phase = APPLE_CIO_SLEEP_RESUMED;
	if (!READ_ONCE(acio->resume_link_pending))
		return;
	WRITE_ONCE(acio->resume_link_pending, false);
	if (!READ_ONCE(resume_recovery) || acio->removing ||
	    !acio->current_cable_info || !acio->resume_had_peer ||
	    acio->resume_recovery_failed)
		return;

	acio->resume_link_failures = 0;
	mod_delayed_work(system_freezable_wq, &acio->resume_link_work,
			 msecs_to_jiffies(APPLE_CIO_RESUME_LINK_WAIT_MS));
}

static const struct dev_pm_ops apple_cio_pm_ops = {
	.prepare = apple_cio_prepare,
	.complete = apple_cio_complete,
};

/**
 * apple_cio_tbt_switch_set - Handle a USB4/Thunderbolt cable transition
 * @sw: USB4/Thunderbolt switch
 * @data: New cable state and negotiated cable details
 *
 * Translate the cable details into the Apple host-router representation and
 * synchronously power ACIO up or down. The Type-C PD driver must configure
 * the PHY before an active transition and must request the inactive state
 * before shutting the PHY down.
 *
 * Return: 0 on success or a negative error code on failure.
 */
static int apple_cio_tbt_switch_set(struct typec_thunderbolt_switch_dev *sw,
				    const struct typec_thunderbolt_switch_data *data)
{
	struct apple_cio *acio = typec_thunderbolt_switch_get_drvdata(sw);

	guard(mutex)(&acio->lock);

	/* Removal turns the cable off itself; only refuse to start new work. */
	if (acio->removing)
		return data->state == TYPEC_THUNDERBOLT_SWITCH_OFF ? 0 : -ESHUTDOWN;

	dev_dbg(acio->dev, "set cable state: %d\n", data->state);

	switch (data->state) {
	case TYPEC_THUNDERBOLT_SWITCH_OFF:
		acio->target_cable_info = 0;
		break;
	case TYPEC_THUNDERBOLT_SWITCH_TBT:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_TBT2_3;
		if (data->tbt.cable_mode & TBT_CABLE_ACTIVE_PASSIVE) {
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
			if (!(data->tbt.cable_mode & TBT_CABLE_LINK_TRAINING))
				acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_BIDIR_LSRX;
		}
		/* bit 16 of the Device Discover Mode VDO is 1 for a legacy TBT2 adapter */
		if (TBT_ADAPTER(data->tbt.device_mode))
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_LEGACY_ADAPTER;
		if (TBT_CABLE_SPEED(data->tbt.cable_mode) == TBT_CABLE_10_AND_20GBPS)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev,
			"TBT cable: cable_mode 0x%x, device_mode 0x%x, enter_vdo 0x%x, orientation %d -> cable info 0x%x\n",
			data->tbt.cable_mode, data->tbt.device_mode,
			data->tbt.enter_vdo, data->orientation,
			acio->target_cable_info);
		break;
	case TYPEC_THUNDERBOLT_SWITCH_USB4:
		acio->target_cable_info = TB_VSE_CAP_APPLE_CABLE_INFO_PRESENT;
		if (FIELD_GET(EUDO_CABLE_TYPE_MASK, data->usb4.eudo) != EUDO_CABLE_TYPE_PASSIVE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ACTIVE_CABLE;
		/*
		 * The T6050 router firmware trains Gen4 only on a Gen4 cable, and
		 * takes Gen4 alone: no 20 Gb/s bit beside it.
		 */
		if (acio->hw->gen4_cable &&
		    FIELD_GET(EUDO_CABLE_SPEED_MASK, data->usb4.eudo) >=
		    APPLE_EUDO_CABLE_SPEED_USB4_GEN4)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_GEN4;
		else if (FIELD_GET(EUDO_CABLE_SPEED_MASK, data->usb4.eudo) >=
			 EUDO_CABLE_SPEED_USB4_GEN3)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_20_GBPS;
		if (data->orientation == TYPEC_ORIENTATION_REVERSE)
			acio->target_cable_info |= TB_VSE_CAP_APPLE_CABLE_INFO_ORIENTATION_REVERSE;
		dev_dbg(acio->dev, "USB4 cable: eudo 0x%x, orientation %d -> cable info 0x%x\n",
			data->usb4.eudo, data->orientation, acio->target_cable_info);
		break;
	}

	if (acio->native_sleep)
		return acio->target_cable_info == acio->current_cable_info ? 0 : -EBUSY;
	/* Until resume, record changes; CD321x reapplies fresh state afterwards. */
	if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED)
		return 0;
	if (acio->sleep_phase == APPLE_CIO_SLEEP_RESUMED)
		acio->sleep_phase = APPLE_CIO_SLEEP_OPEN;
	/*
	 * Only a successful start settles a restart owed from sleep: CD321x
	 * also sends OFF as an intermediate step of a reconnect. With the cable
	 * truly gone, the cost is one extra cable read per later sleep.
	 */
	if (acio->sleep_restart_owed && acio->target_cable_info)
		dev_info(acio->dev, "restarting port after system sleep\n");

	if (acio->target_cable_info == acio->current_cable_info)
		return acio->current_cable_info ? apple_cio_check_connection(acio) : 0;

	/*
	 * Changing live cable parameters requires an ACIO shutdown. Report the
	 * rejected transition so the caller does not cache the new mode as active.
	 */
	if (acio->current_cable_info && acio->target_cable_info) {
		dev_err(acio->dev,
			"Invalid cable transition from 0x%x to 0x%x, shutting down instead\n",
			acio->current_cable_info, acio->target_cable_info);
		acio->target_cable_info = 0;
		apple_cio_stop(acio);
		return -EINVAL;
	}

	/*
	 * Bring up or power down the ACIO complex
	 * current_cable_info will be updated in the start/stop functions
	 */
	if (acio->target_cable_info) {
		int ret;

		/* -EHOSTDOWN: CD321x stops retrying a port disabled for good. */
		if (acio->resume_recovery_failed)
			return -EHOSTDOWN;
		ret = apple_cio_start(acio);
		if (!ret)
			WRITE_ONCE(acio->sleep_restart_owed, false);
		/*
		 * The port had a dock before its power-off for the sleep: one that is not
		 * found again gets the port restarted, as a peer lost with the cable in.
		 */
		if (!ret && acio->sleep_peer_wanted) {
			WRITE_ONCE(acio->link_lost_gen, READ_ONCE(acio->start_gen));
			mod_delayed_work(system_freezable_wq, &acio->link_lost_work,
					 msecs_to_jiffies(APPLE_CIO_LINK_LOST_MS));
		}
		if (ret && apple_cio_recovering(acio)) {
			acio->resume_recovery_failed = true;
			acio->target_cable_info = 0;
			dev_err(acio->dev,
				"resume recovery start failed: %d; port disabled\n", ret);
			ret = -EHOSTDOWN;
		}
		acio->resume_recovery_active = false;
		return ret;
	}

	apple_cio_stop(acio);
	return 0;
}

/* The T6050 firmware sleep, selected by the sleep driver's profile and mem_sleep. */
static bool apple_cio_t6050_sleep(struct apple_cio *acio)
{
	return (of_machine_is_compatible("apple,j714s") ||
		of_machine_is_compatible("apple,j716c")) &&
	       READ_ONCE(t6050_sleep_teardown) && acio->hw == &apple_cio_t6050_hw &&
	       apple_sleep_v27_late() && !pm_suspend_default_s2idle();
}

/* The longest the displays behind a port may take to go before its router does. */
#define APPLE_CIO_SLEEP_DP_RELEASE_MS	3000

/*
 * The displays on this router's DP IN adapters, released as at their tunnels' end and before
 * anything of the router is touched. The tunnels' own end, at the domain's removal, then finds
 * its request already made. With @pause the adapters' admission is closed first, as the NHI's
 * prepare closes it: a tunnel offered from here on waits, not handed to a display processor,
 * until the admission is opened again (the NHI's removal or complete, or the end of a sleep
 * that was given up). Returns how many requests were ended, or -EBUSY if the display side is
 * still at it after APPLE_CIO_SLEEP_DP_RELEASE_MS: nothing of the port has changed then but
 * those displays.
 */
static int apple_cio_sleep_release_displays(struct apple_cio *acio, bool pause)
{
	ktime_t start = ktime_get();
	ktime_t deadline = ktime_add_ms(start, APPLE_CIO_SLEEP_DP_RELEASE_MS);
	unsigned int i, asked = 0, busy;

	lockdep_assert_held(&acio->lock);
	if (!acio->dp_wq)
		return 0;
	if (pause && acio->dp->defer_new_bringup) {
		for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++) {
			struct apple_dpin_ctx *c = &acio->dpin[i];

			scoped_guard(mutex, &c->lock)
				apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_PAUSE, !!c->regs, 0);
			/* Not waited for: a retry that still fires finds the admission closed. */
			cancel_delayed_work(&c->retry);
		}
		acio->sleep_dp_paused = true;
	}
	for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];
		bool queue = false;

		scoped_guard(mutex, &c->lock) {
			u64 generation = c->tokens.requested;

			if (generation)
				queue = apple_dpin_request(c, generation, false);
		}
		if (queue) {
			queue_work(acio->dp_wq, &c->work);
			asked |= BIT(i);
		}
	}
	/* With the admission closed, an attempt that was on its way ends too. */
	if (!asked && !pause)
		return 0;
	for (;;) {
		busy = 0;
		for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++)
			if (((asked & BIT(i)) || pause) && work_busy(&acio->dpin[i].work))
				busy |= BIT(i);
		if (!busy)
			break;
		if (ktime_after(ktime_get(), deadline)) {
			dev_err(acio->dev, "T6050 sleep: the displays behind this port are not released after %lld ms (DP IN %#x still busy, %#x asked): the port stays up and the sleep is refused\n",
				ktime_ms_delta(ktime_get(), start), busy, asked);
			return -EBUSY;
		}
		usleep_range(1000, 2000);
	}
	if (asked || pause)
		dev_info(acio->dev, "T6050 sleep: %u display request(s) behind this port ended in %lld ms, before %s (DP IN %#x)%s\n",
			 hweight32(asked), ktime_ms_delta(ktime_get(), start),
			 pause ? "any port's power-off" : "the router", asked,
			 pause && acio->sleep_dp_paused ? "; new tunnels wait for the end of the sleep" : "");
	return hweight32(asked);
}

static int apple_cio_sleep_teardown(struct apple_cio *acio)
{
	ktime_t start = ktime_get(), stop;
	const char *lstx = "not held";
	bool t6050;
	int released;

	cancel_delayed_work_sync(&acio->resume_link_work);

	guard(mutex)(&acio->lock);
	if (acio->removing || acio->resume_recovery_failed || !acio->current_cable_info)
		return 0;

	if (apple_cio_t6050_sleep(acio) && READ_ONCE(t6050_sleep_dp_first)) {
		released = apple_cio_sleep_release_displays(acio, false);
		if (released < 0)
			return released;
	}

	/*
	 * PCIe endpoints may unregister PM notifiers on removal, which would
	 * deadlock inside this notifier chain, and storage must not disappear
	 * under running applications. Skip ports with PCIe tunnels, and close
	 * PCIe admission in the same critical section; the next NHI probe
	 * reopens it.
	 */
	scoped_guard(mutex, &acio->pcie_tunnel_lock) {
		if (READ_ONCE(acio->pcie_tunnel_requested) || acio->pcie_tunnel_populated) {
			struct platform_device *pcie;
			int mode = READ_ONCE(t6050_sleep_pcie);
			int ret = 0;

			/*
			 * The T6050 firmware sleep takes the router's power whatever is behind
			 * it. The tunnelled functions go first and in order, while the tunnel
			 * still carries them: only NVMe controllers, whose driver touches no
			 * notifier chain on removal, and by default only with no disk in use.
			 */
			if (!mode || !apple_cio_t6050_sleep(acio))
				return 0;
			pcie = apple_cio_find_pcie_tunnel(acio);
			if (pcie) {
				ret = apple_pcie_tunnel_remove_orderly(&pcie->dev, mode == 2);
				put_device(&pcie->dev);
			}
			if (ret) {
				dev_warn(acio->dev,
					 "PCIe devices behind this port stay for the sleep (%d): the port is not powered off\n",
					 ret);
				return 0;
			}
		}
		acio->pcie_tunnel_stopping = true;
	}

	dev_info(acio->dev, "powering off port for system sleep (cable info 0x%x)\n",
		 acio->current_cable_info);
	t6050 = apple_cio_t6050_sleep(acio);
	if (t6050) {
		u64 activity;

		/* With no NHI the answer is "yes": then nothing is expected back. */
		acio->sleep_peer_wanted = READ_ONCE(acio->nhi_pdev) &&
					  apple_cio_has_peer(acio, &activity);
		acio->link_lost_cycles = 0;
	}
	stop = ktime_get();
	apple_cio_stop(acio);
	acio->sleep_phase = APPLE_CIO_SLEEP_CLOSED;
	WRITE_ONCE(acio->sleep_restart_owed, true);
	/* Its displays come back with the port's restart. */
	acio->sleep_dp_released = 0;
	if (!t6050)
		return 0;
	/* The stop left the pin at the router's last level, held by the AP. */
	if (READ_ONCE(t6050_sleep_lstx_low) && acio->lstx_gpio && acio->lstx_high) {
		gpiod_set_value_cansleep(acio->lstx_gpio, 0);
		acio->lstx_high = false;
		lstx = "lowered";
	} else if (acio->lstx_gpio) {
		lstx = acio->lstx_high ? "left high" : "low";
	}
	acio->sleep_t6050 = true;
	dev_info(acio->dev, "T6050 sleep: port off: PCIe functions %lld ms, tunnels, router and domains %lld ms, LSTX %s\n",
		 ktime_ms_delta(stop, start), ktime_ms_delta(ktime_get(), stop), lstx);
	return 0;
}

/* A restart still owed from the last sleep must not race this suspend. */
static void apple_cio_sleep_close_owed(struct apple_cio *acio)
{
	guard(mutex)(&acio->lock);
	if (acio->sleep_restart_owed && !acio->current_cable_info)
		acio->sleep_phase = APPLE_CIO_SLEEP_CLOSED;
	acio->sleep_aborted_early = false;
}

/*
 * Runs when no post-resume revalidation has restarted the port: always
 * after an aborted suspend, and as a fallback after a real one. Ask CD321x
 * for a fresh read instead of starting from the state recorded before sleep.
 */
static void apple_cio_sleep_restart_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(to_delayed_work(work), struct apple_cio,
					      sleep_restart_work);

	scoped_guard(mutex, &acio->lock) {
		acio->sleep_aborted_early = false;
		if (!acio->sleep_restart_owed)
			return;
		acio->sleep_phase = APPLE_CIO_SLEEP_OPEN;
		if (acio->removing || acio->resume_recovery_failed || acio->current_cable_info) {
			WRITE_ONCE(acio->sleep_restart_owed, false);
			return;
		}
		dev_info(acio->dev, "requesting cable restore after system sleep\n");
	}

	/* Never notify under acio->lock: CD321x calls back into .set. */
	typec_thunderbolt_switch_notify_restore(acio->tbt_switch);
}

/* Is this port powered off for the sleep about to start? Asked at PM_SUSPEND_PREPARE. */
static bool apple_cio_sleep_tears_down(struct apple_cio *acio)
{
	if (READ_ONCE(s2idle_teardown))
		return true;
	/* A "freeze" is a suspend-to-idle with deep as the default too: the port stays up. */
	if (pm_suspend_no_platform())
		return false;
	/* A port whose router sleeps in place keeps its tunnels: its NHI owns the sleep. */
	return apple_cio_t6050_sleep(acio) && !READ_ONCE(acio->sleep_in_place);
}

static int apple_cio_pm_notify(struct notifier_block *nb, unsigned long event,
			       void *unused)
{
	struct apple_cio *acio = container_of(nb, struct apple_cio, pm_notifier);
	bool tears_down = event == PM_SUSPEND_PREPARE && apple_cio_sleep_tears_down(acio);
	unsigned int delay = APPLE_CIO_SLEEP_RESTART_FALLBACK_MS;
	bool owed, early;
	int ret;

	/* Teardown off and no restart owed from an earlier sleep: nothing to do. */
	if (!tears_down && !READ_ONCE(s2idle_teardown) &&
	    !READ_ONCE(acio->sleep_restart_owed))
		return NOTIFY_DONE;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* Nothing from an earlier sleep may restart the port during this one. */
		cancel_delayed_work_sync(&acio->sleep_restart_work);
		apple_cio_sleep_close_owed(acio);
		/* Before tasks freeze: tearing down the NHI needs unfrozen work. */
		if (tears_down) {
			/* Only the displays' release refuses, with the port as it was. */
			ret = apple_cio_sleep_teardown(acio);
			if (ret)
				return notifier_from_errno(ret);
		}
		break;
	case PM_POST_SUSPEND:
		scoped_guard(mutex, &acio->lock) {
			/* Aborted before device suspend: complete never ran. */
			if (acio->sleep_phase == APPLE_CIO_SLEEP_CLOSED) {
				acio->sleep_phase = APPLE_CIO_SLEEP_RESUMED;
				acio->sleep_aborted_early = true;
			}
			owed = acio->sleep_restart_owed;
			early = acio->sleep_aborted_early;
			/* TEST: macOS starts the router again straight after the wake. */
			if (acio->sleep_t6050) {
				acio->sleep_t6050 = false;
				delay = READ_ONCE(t6050_sleep_restart_ms);
				if (owed)
					dev_info(acio->dev, "T6050 sleep: port restart asked %u ms after the resume%s\n",
						 early ? 0 : delay, early ? " (the suspend was given up early)" : "");
			}
		}
		/* Start from a worker: probing children inside this chain can deadlock. */
		if (owed)
			queue_delayed_work(system_dfl_wq, &acio->sleep_restart_work,
					   early ? 0 : msecs_to_jiffies(delay));
		break;
	}
	return NOTIFY_DONE;
}

/*
 * A sleep was given up with this port's displays released and the port still up (a later
 * step of the entry refused, before the port's own power-off). A released tunnel cannot be
 * requested again, so the port is powered off and restarted, as it would have been for the
 * sleep: the dock and its displays come back as newly attached. From a work: the stop
 * removes devices, and must not run inside the PM notifier chain.
 */
static void apple_cio_sleep_dp_undo_work(struct work_struct *work)
{
	struct apple_cio *acio = container_of(work, struct apple_cio, sleep_dp_undo_work);
	bool down;
	int ret;

	scoped_guard(mutex, &acio->lock) {
		/* Not owed any more: a new sleep's entry has begun, and its restart does it. */
		if (!acio->sleep_dp_undo)
			return;
		acio->sleep_dp_undo = false;
		if (acio->removing || !acio->current_cable_info)
			return;
	}
	dev_info(acio->dev, "T6050 sleep given up with this port's displays released: the port is restarted to bring them back\n");
	ret = apple_cio_sleep_teardown(acio);
	scoped_guard(mutex, &acio->lock) {
		down = !ret && !acio->current_cable_info && acio->sleep_restart_owed;
		acio->sleep_t6050 = false;
	}
	if (!down) {
		/* PCIe functions that stay keep the port up: only a replug brings the displays. */
		dev_warn(acio->dev, "T6050 sleep given up: the port stays up (%d), its released displays stay off until the dock is plugged in again\n",
			 ret);
		apple_dpin_resume_retries(acio);
		return;
	}
	queue_delayed_work(system_dfl_wq, &acio->sleep_restart_work, 0);
}

/*
 * A port whose router sleeps in place keeps its tunnels until its NHI's prepare, after the
 * freezer. Until then no tunnel of it may be handed a display: a display that goes at the
 * start of the entry frees a pipeline, and a tunnel that was waiting for one would light a
 * display in the middle of the entry (sv6t2: "dpin1: a display pipeline came free; DP tunnel
 * up" 0.6 s after the freezer). So the admission closes here, ahead of the display processors'
 * own step, as for a port that is powered off; nothing is released. An attempt already on
 * its way is waited for, and the sleep refused if it does not end.
 */
static int apple_cio_sleep_close_admission(struct apple_cio *acio)
{
	ktime_t start = ktime_get();
	ktime_t deadline = ktime_add_ms(start, APPLE_CIO_SLEEP_DP_RELEASE_MS);
	unsigned int i, busy, waiting = 0;

	lockdep_assert_held(&acio->lock);
	if (!acio->dp_wq || !acio->dp->defer_new_bringup)
		return 0;
	for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		scoped_guard(mutex, &c->lock) {
			apple_dpin_step(&c->state, acio->dp, APPLE_DPIN_PAUSE, !!c->regs, 0);
			if (c->state.alive && (c->state.waiting || c->state.deferred_first))
				waiting |= BIT(i);
		}
		/* Not waited for: a retry that still fires finds the admission closed. */
		cancel_delayed_work(&c->retry);
	}
	acio->sleep_dp_paused = true;
	for (;;) {
		busy = 0;
		for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++)
			if (work_busy(&acio->dpin[i].work))
				busy |= BIT(i);
		if (!busy)
			break;
		if (ktime_after(ktime_get(), deadline)) {
			dev_err(acio->dev, "T6050 sleep: a display request behind this port is still on its way after %lld ms (DP IN %#x): the sleep is refused\n",
				ktime_ms_delta(ktime_get(), start), busy);
			return -EBUSY;
		}
		usleep_range(1000, 2000);
	}
	dev_info(acio->dev, "T6050 sleep: DP IN admission closed for the sleep's entry in %lld ms, the router sleeps in place: no tunnel is handed a display until the sleep is over (waiting for a pipeline: DP IN %#x)\n",
		 ktime_ms_delta(ktime_get(), start), waiting);
	return 0;
}

/* The entry of a sleep is over, slept or given up: tunnels that waited are handed on. */
static void apple_cio_sleep_dp_reopen(struct apple_cio *acio)
{
	unsigned int i, waiting = 0;

	if (!acio->dp_wq)
		return;
	for (i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++) {
		struct apple_dpin_ctx *c = &acio->dpin[i];

		guard(mutex)(&c->lock);
		if (c->state.alive && c->state.paused && c->state.deferred_first)
			waiting |= BIT(i);
	}
	if (waiting)
		dev_info(acio->dev, "T6050 sleep: %u tunnelled display(s) offered during the sleep's entry waited (DP IN %#x): handed to the display side now\n",
			 hweight32(waiting), waiting);
	apple_dpin_resume_retries(acio);
}

/*
 * The start of a T6050 firmware sleep's entry on this port. Returns 0, or the error that
 * refuses the sleep with *@paused telling whether the admission was closed.
 */
static int apple_cio_sleep_dp_entry(struct apple_cio *acio, bool *paused)
{
	int ret;

	*paused = false;
	guard(mutex)(&acio->lock);
	/*
	 * A restart owed from a sleep given up is this sleep's own from here, if it powers
	 * the port off. A suspend-to-idle doesn't: it is refused until the undo's work has run.
	 */
	if (acio->sleep_dp_undo && !apple_cio_sleep_tears_down(acio))
		return -EBUSY;
	acio->sleep_dp_undo = false;
	acio->sleep_dp_released = 0;
	if (!READ_ONCE(t6050_sleep_dp_all_first) || !READ_ONCE(t6050_sleep_dp_first) ||
	    !apple_cio_t6050_sleep(acio) || pm_suspend_no_platform() ||
	    acio->removing || acio->resume_recovery_failed || !acio->current_cable_info)
		return 0;
	if (!apple_cio_sleep_tears_down(acio)) {
		if (!READ_ONCE(acio->sleep_in_place))
			return 0;
		/* Nothing released, so no restart is owed if this refuses: only the reopening. */
		ret = apple_cio_sleep_close_admission(acio);
		if (ret) {
			*paused = acio->sleep_dp_paused;
			acio->sleep_dp_paused = false;
		}
		return ret;
	}
	ret = apple_cio_sleep_release_displays(acio, true);
	if (ret >= 0) {
		acio->sleep_dp_released = ret;
		return 0;
	}
	/* No PM_POST_SUSPEND comes to a notifier that refused: undone from here. */
	*paused = acio->sleep_dp_paused;
	acio->sleep_dp_paused = false;
	acio->sleep_dp_undo = true;
	return ret;
}

/* The end of a sleep, or of one given up after this port's entry step. */
static bool apple_cio_sleep_dp_exit(struct apple_cio *acio, bool *paused)
{
	bool undo;

	guard(mutex)(&acio->lock);
	*paused = acio->sleep_dp_paused;
	acio->sleep_dp_paused = false;
	/* Powered off since: the restart the port's own notifier asks for brings them back. */
	undo = acio->sleep_dp_released && acio->current_cable_info && !acio->removing;
	acio->sleep_dp_released = 0;
	acio->sleep_dp_undo = undo;
	return undo;
}

/*
 * Ahead of every other step of a T6050 firmware sleep's entry (the USB hosts' stops, the
 * PCIe ports' parks, the ports' power-offs, all at the default priority): the displays
 * behind this port are released and the DP IN admission closed. Refuses the sleep if the
 * display side does not answer in time.
 */
static int apple_cio_dp_pm_notify(struct notifier_block *nb, unsigned long event,
				  void *unused)
{
	struct apple_cio *acio = container_of(nb, struct apple_cio, pm_notifier_dp);
	bool paused;
	int ret;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		ret = apple_cio_sleep_dp_entry(acio, &paused);
		if (!ret)
			break;
		if (paused)
			apple_cio_sleep_dp_reopen(acio);
		queue_work(system_dfl_wq, &acio->sleep_dp_undo_work);
		return notifier_from_errno(ret);
	case PM_POST_SUSPEND:
		if (apple_cio_sleep_dp_exit(acio, &paused))
			queue_work(system_dfl_wq, &acio->sleep_dp_undo_work);
		else if (paused)
			apple_cio_sleep_dp_reopen(acio);
		break;
	}
	return NOTIFY_DONE;
}

static int apple_cio_pcie_notify(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct apple_cio *acio = container_of(nb, struct apple_cio, pcie_notifier);

	if (event != APPLE_PCIE_TUNNEL_LINK_DOWN || data != acio->dev)
		return NOTIFY_DONE;

	typec_thunderbolt_switch_notify(acio->tbt_switch);
	return NOTIFY_OK;
}

static int apple_cio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_cio *acio;
	int ret;

	acio = devm_kzalloc(dev, sizeof(*acio), GFP_KERNEL);
	if (!acio)
		return -ENOMEM;
	atomic_set(&acio->fw_lane_action, -1);
	platform_set_drvdata(pdev, acio);

	ret = devm_mutex_init(dev, &acio->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &acio->pcie_tunnel_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&acio->pcie_tunnel_work,
			  apple_cio_pcie_tunnel_work);
	INIT_DELAYED_WORK(&acio->resume_link_work, apple_cio_resume_link_work);
	INIT_DELAYED_WORK(&acio->link_lost_work, apple_cio_link_lost_work);
	INIT_DELAYED_WORK(&acio->sleep_restart_work, apple_cio_sleep_restart_work);
	INIT_WORK(&acio->sleep_dp_undo_work, apple_cio_sleep_dp_undo_work);
	init_completion(&acio->nhi_boot_completion);
	acio->dev = &pdev->dev;
	acio->np = dev->of_node;
	acio->hw = of_device_get_match_data(dev);
	if (!acio->hw)
		return -EINVAL;

	acio->type7 = acio->hw->type7;
	if (acio->type7 &&
	    of_property_count_u32_elems(acio->np, "apple,tunable-rc") <= 0)
		return dev_err_probe(dev, -EINVAL, "Missing bootloader RC tuning\n");

	if (acio->hw->mxwrap) {
		struct resource *cpu_res;

		cpu_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cpu");
		if (!cpu_res)
			return dev_err_probe(dev, -EINVAL, "Missing MxWrap cpu region\n");
		cpu_res->flags |= IORESOURCE_MEM_NONPOSTED;
		acio->cpu_base = devm_ioremap_resource(dev, cpu_res);
		if (IS_ERR(acio->cpu_base))
			return dev_err_probe(dev, PTR_ERR(acio->cpu_base),
					     "Unable to map MxWrap cpu regs\n");
	}
	if (acio->hw->lstx_after_boot) {
		acio->pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(acio->pinctrl))
			return dev_err_probe(dev, PTR_ERR(acio->pinctrl),
					     "Unable to get the LSTX pin\n");
		acio->lstx_pins = pinctrl_lookup_state(acio->pinctrl, "lstx");
		if (IS_ERR(acio->lstx_pins))
			acio->lstx_pins = NULL;
		acio->lstx_hold = acio->lstx_pins &&
				  of_property_present(dev->of_node, "lstx-gpios");
		/* Hold the line low until the router first takes it. */
		if (acio->hw->lstx_hold_from_probe)
			apple_cio_lstx_hold(acio, false);
	}
	acio->pcie_tunnel_np =
		of_parse_phandle(dev->of_node, "apple,pcie-tunnel", 0);
	if (acio->pcie_tunnel_np) {
		ret = devm_add_action_or_reset(dev, apple_cio_of_node_put,
					       acio->pcie_tunnel_np);
		if (ret)
			return ret;

		acio->pcie_tunnel_preinitialized =
			apple_cio_pcie_tunnel_is_preinitialized(
				acio->pcie_tunnel_np);
		if (!acio->pcie_tunnel_preinitialized) {
			dev_info(dev,
				 "PCIe-C tunnel disabled: not initialized by aura or the kernel\n");
		} else {
			ret = apple_cio_map_pcie_intr2axi(acio);
			if (ret)
				return dev_err_probe(dev, ret,
						     "failed to map PCIe-C Intr2AXI registers\n");
			ret = apple_cio_get_pcie_reset(acio);
			if (ret)
				return ret;
		}
	}

	acio->sram_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!acio->sram_res)
		return dev_err_probe(dev, -EIO, "Failed to get SRAM resource\n");
	acio->sram_base = devm_ioremap_resource(dev, acio->sram_res);
	if (IS_ERR(acio->sram_base))
		return dev_err_probe(dev, PTR_ERR(acio->sram_base), "Failed to map SRAM\n");

	acio->rc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc");
	acio->rc_base = devm_ioremap_resource(&pdev->dev, acio->rc_res);
	if (IS_ERR(acio->rc_base))
		return dev_err_probe(dev, PTR_ERR(acio->rc_base), "Unable to map rc regs\n");
	acio->rc_tunable =
		devm_apple_tunable_parse(dev, acio->np, "apple,tunable-rc", acio->rc_res);
	if (IS_ERR(acio->rc_tunable))
		return dev_err_probe(dev, PTR_ERR(acio->rc_tunable), "Unable to load rc tunable\n");

	{
		struct resource *adapter;

		adapter = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "pcie-adapter");
		if (adapter) {
			adapter->flags |= IORESOURCE_MEM_NONPOSTED;
			acio->pcie_adapter_base = devm_ioremap_resource(dev, adapter);
			if (IS_ERR(acio->pcie_adapter_base))
				return dev_err_probe(dev, PTR_ERR(acio->pcie_adapter_base),
						     "Unable to map PCIe adapter regs\n");
			acio->pcie_adapter_tunable =
				devm_apple_tunable_parse(dev, acio->np,
							 "apple,tunable-pcie-adapter",
							 adapter);
			if (IS_ERR(acio->pcie_adapter_tunable)) {
				if (PTR_ERR(acio->pcie_adapter_tunable) == -ENOENT)
					acio->pcie_adapter_tunable = NULL;
				else
					return dev_err_probe(dev,
							PTR_ERR(acio->pcie_adapter_tunable),
							"Unable to load PCIe adapter tunable\n");
			}
		}
	}

	{
		struct resource *ap_fw;

		ap_fw = platform_get_resource_byname(pdev, IORESOURCE_MEM, "router-ap-fw");
		if (ap_fw) {
			acio->router_ap_fw_base = devm_ioremap_resource(dev, ap_fw);
			if (IS_ERR(acio->router_ap_fw_base))
				return dev_err_probe(dev, PTR_ERR(acio->router_ap_fw_base),
						     "Unable to map router AP firmware regs\n");
			acio->router_ap_fw_tunable =
				devm_apple_tunable_parse(dev, acio->np,
							 "apple,tunable-router-ap-fw", ap_fw);
			if (IS_ERR(acio->router_ap_fw_tunable)) {
				if (PTR_ERR(acio->router_ap_fw_tunable) != -ENOENT)
					return dev_err_probe(dev,
							PTR_ERR(acio->router_ap_fw_tunable),
							"Unable to load router AP firmware tunable\n");
				acio->router_ap_fw_tunable = NULL;
			}
		}
	}

	/*
	 * On T6050 this is the CIO reconfig request, which also clocks the
	 * router CPU's wrapper: nothing may touch the wrapper before it.
	 */
	acio->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(acio->reset))
		return dev_err_probe(dev, PTR_ERR(acio->reset), "Unable to get CIO reset\n");

	/*
	 * If there is only a single domain listed in the device tree the platform driver
	 * framework will already attach it. Thus, if we find an already attached domain
	 * here something's wrong in the device tree because we expect at least three separate
	 * domains that we have to control manually.
	 */
	if (dev->pm_domain) {
		dev_err(dev, "PM domain already attached, check if the DT lists three domains\n");
		return -EINVAL;
	}

	/*
	 * Find and attach the PM domains but don't power them on yet since we must only
	 * do that after the PHY has already been configured into USB4/Thunderbolt mode.
	 */
	struct dev_pm_domain_attach_data pd_data = {
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	ret = devm_pm_domain_attach_list(dev, &pd_data, &acio->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Unable to attach PM domains\n");
	else if (ret < 3)
		return dev_err_probe(dev, -EINVAL, "Not enough PM domains\n");

	acio->dpin_count = acio->hw->dpin_count ?: 2;
	acio->connector_np = of_graph_get_remote_node(dev->of_node, 1, -1);
	acio->dp_dual_stream = apple_dp_tunnel_dual_stream(acio->connector_np);
	/* Register layout, root qualification and connector wiring are separate. */
	acio->dp = apple_dpin_policy_select(acio->hw->dp, of_root, acio->dp_dual_stream);
	if (acio->connector_np) {
		ret = devm_add_action_or_reset(dev, apple_cio_of_node_put,
					       acio->connector_np);
		if (ret)
			return ret;
		for (unsigned int i = 0; i < ARRAY_SIZE(acio->dpin); i++) {
			acio->dpin[i].acio = acio;
			acio->dpin[i].idx = i;
			INIT_WORK(&acio->dpin[i].work, apple_dpin_work_fn);
			INIT_DELAYED_WORK(&acio->dpin[i].retry, apple_dpin_retry_fn);
			INIT_DELAYED_WORK(&acio->dpin[i].poll, apple_dpin_poll_fn);
			INIT_WORK(&acio->dpin[i].event_work, apple_dpin_event_fn);
			ret = devm_mutex_init(dev, &acio->dpin[i].lock);
			if (ret)
				return ret;
		}
		/* Prepare/remove drain this queue; it must remain non-freezable. */
		acio->dp_wq = alloc_ordered_workqueue("%s-dp", 0, dev_name(dev));
		if (!acio->dp_wq)
			return -ENOMEM;
		ret = devm_add_action_or_reset(dev, apple_cio_destroy_wq, acio);
		if (ret)
			return ret;
		/* Freed before the queue: started and stopped with the mapping. */
		for (unsigned int i = 0; i < acio->dpin_count && i < APPLE_DPIN_MAX; i++) {
			struct apple_dpin_ctx *c = &acio->dpin[i];
			char name[8];

			snprintf(name, sizeof(name), "dpin%u", i);
			c->irq = platform_get_irq_byname_optional(to_platform_device(dev), name);
			if (c->irq <= 0)
				continue;
			ret = devm_request_threaded_irq(dev, c->irq, NULL,
							apple_dpin_irq_thread,
							IRQF_ONESHOT | IRQF_NO_AUTOEN,
							devm_kasprintf(dev, GFP_KERNEL, "%s-%s",
								       dev_name(dev), name),
							c);
			if (ret)
				return dev_err_probe(dev, ret,
						     "cannot request %s interrupt\n", name);
		}
	}

	/* And finally register the OOB notification for Thunderbolt/USB4 cables */
	struct typec_thunderbolt_switch_desc desc = {
		.fwnode = pdev->dev.fwnode,
		.set = apple_cio_tbt_switch_set,
		.drvdata = acio,
	};
	/* A consumer cannot start a host before its failure listener is ready. */
	scoped_guard(mutex, &acio->lock) {
		acio->tbt_switch = typec_thunderbolt_switch_register(dev, &desc);
		if (IS_ERR(acio->tbt_switch))
			return dev_err_probe(dev, PTR_ERR(acio->tbt_switch),
					     "Unable to register thunderbolt switch\n");

		acio->pcie_notifier.notifier_call = apple_cio_pcie_notify;
		ret = apple_pcie_tunnel_register_notifier(&acio->pcie_notifier);
		if (ret)
			acio->removing = true;
	}
	if (ret) {
		typec_thunderbolt_switch_unregister(acio->tbt_switch);
		return ret;
	}

	acio->pm_notifier.notifier_call = apple_cio_pm_notify;
	register_pm_notifier(&acio->pm_notifier);
	/* Before the notifiers of the default priority, after the port controllers' (30). */
	acio->pm_notifier_dp.notifier_call = apple_cio_dp_pm_notify;
	acio->pm_notifier_dp.priority = 20;
	register_pm_notifier(&acio->pm_notifier_dp);
	return 0;
}

static void apple_cio_remove(struct platform_device *pdev)
{
	struct apple_cio *acio = platform_get_drvdata(pdev);

	/* Refuse new activation before draining callbacks and workers. */
	scoped_guard(mutex, &acio->lock)
		acio->removing = true;

	apple_pcie_tunnel_unregister_notifier(&acio->pcie_notifier);
	unregister_pm_notifier(&acio->pm_notifier_dp);
	unregister_pm_notifier(&acio->pm_notifier);
	disable_work_sync(&acio->sleep_dp_undo_work);
	disable_delayed_work_sync(&acio->sleep_restart_work);

	disable_delayed_work_sync(&acio->resume_link_work);
	disable_delayed_work_sync(&acio->link_lost_work);
	/* The worker can notify this switch; stop current and future enqueues. */
	disable_delayed_work_sync(&acio->pcie_tunnel_work);
	/* A set may hold the switch read lock while waiting for acio->lock. */
	typec_thunderbolt_switch_unregister(acio->tbt_switch);
	guard(mutex)(&acio->lock);
	if (acio->current_cable_info)
		apple_cio_stop(acio);
}




static const struct of_device_id apple_acio_match[] = {
	{
		.compatible = "apple,t6040-usb4-acio",
		.data = &apple_cio_t6040_hw,
	},
	{
		.compatible = "apple,t6050-usb4-acio",
		.data = &apple_cio_t6050_hw,
	},
	{
		.compatible = "apple,t8142-usb4-acio",
		.data = &apple_cio_t8142_hw,
	},
	{
		.compatible = "apple,t8132-usb4-acio",
		.data = &apple_cio_t8132_hw,
	},
	{
		.compatible = "apple,t6030-usb4-acio",
		.data = &apple_cio_t6030_hw,
	},
	{
		.compatible = "apple,t6020-usb4-acio",
		.data = &apple_cio_t6020_hw,
	},
	{
		.compatible = "apple,t8103-usb4-acio",
		.data = &apple_cio_t8103_hw,
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_acio_match);

static struct platform_driver apple_cio_driver = {
	.driver = {
		.name = "thunderbolt-apple-acio",
		.of_match_table = apple_acio_match,
		.pm = pm_sleep_ptr(&apple_cio_pm_ops),
	},
	.probe = apple_cio_probe,
	.remove = apple_cio_remove,
};

static struct platform_driver * const apple_cio_drivers[] = {
	&apple_nhi_driver,
	&apple_cio_driver,
};

static int __init apple_cio_init(void)
{
	return platform_register_drivers(apple_cio_drivers,
					 ARRAY_SIZE(apple_cio_drivers));
}

static void __exit apple_cio_exit(void)
{
	platform_unregister_drivers(apple_cio_drivers,
				    ARRAY_SIZE(apple_cio_drivers));
}

module_init(apple_cio_init);
module_exit(apple_cio_exit);

MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple Silicon USB4/Thunderbolt driver");
