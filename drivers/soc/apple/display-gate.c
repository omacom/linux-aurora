// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple internal display enable gate (T6030, T6031, T8122 and T6041)
 *
 * The T6030 device tree describes the internal display (the DCP, its
 * mailbox, the two display DARTs and the display subsystem) with every node
 * disabled. Kernels that cannot take the display over from iBoot therefore
 * keep scanning out through the simple framebuffer, and all of them can
 * share one device tree.
 *
 * This gate enables those five nodes before platform devices are created.
 * It acts only if the boot loader has set apple,t6030-handoff = <1> on the
 * DCP and on the display subsystem, which it does after it has checked and
 * locked the inherited DART mappings. apple_t6030_display.enable=0 on the
 * kernel command line keeps the display on the boot framebuffer instead;
 * apple_t6030_display.dcpext=0 and apple_t6030_display.scanout=0 do the same
 * for the external display processors and their display DARTs (see
 * gate_dcpext() below). The targets are found through the DCP's and the
 * display subsystem's phandles, and each must still be disabled. Otherwise
 * nothing is changed.
 *
 * The DCP firmware needs the power management processor (PMP) running,
 * which iBoot leaves loaded but halted. Once the display nodes are enabled,
 * the gate also adds the PMP from a built-in overlay: its node, DART and
 * mailbox (disabled), and a PMP report node whose driver sets the display
 * and storage requests and starts the PMP when the display driver asks for
 * it. The PMP node needs this Mac's board and DRAM vendor ids and its PMP
 * tunables. A boot loader that copies them from the ADT into
 * /chosen/asahi,t6030-pmp gets the T6030 overlay with those values, on any
 * T6030 board; without them, only a J516S gets the PMP, from an overlay with
 * one J516S's values. Before any driver probes, the gate then
 *  - raises the minimum power state of the display domains (display
 *    subsystem, front end and DCP CPU) to "active", so that the PMP can
 *    never power the running DCP down;
 *  - keeps the PMP and PMS_SRAM domains on, as iBoot left them: the PTD the
 *    DCP firmware and the PMP share lives in that SRAM;
 *  - gives the PMP DART and mailbox the interrupt parent of the DCP mailbox.
 * Everything the PMP needs is checked before the display nodes are enabled.
 * Without PMP values for this Mac, or if a check fails or the PMP cannot be
 * added, the display nodes stay (or are put back) disabled and the display
 * stays on the boot framebuffer.
 *
 * What differs between SoCs is in struct gate_soc. On T8122 (M3) the gate
 * serves the same handoff, marked apple,t8122-handoff by the boot loader on
 * apple,t8122-dcp and apple,t8122-display-subsystem nodes. Its device tree
 * already describes the PMP, its DART, mailbox and report (disabled where it
 * must be), so the gate adds no overlay there: it checks that the PMP
 * topology is complete and unused, and then completes it (see
 * gate_reuse_resolve()). There is no T8122 fallback: the boot loader must pass
 * this Mac's PMP values in /chosen/asahi,t8122-pmp, or nothing is enabled. A
 * T8122 device tree without display nodes is normal and the gate says
 * nothing. apple_t8122_display.enable=0 has the same effect as the T6030
 * option.
 *
 * On T6031 (M3 Max) the gate serves the same handoff, marked
 * apple,t6031-handoff. Its default is scoped to J514C and J516C: there the native
 * display is on unless apple_t6031_display.enable=0; other T6031 boards
 * need apple_t6031_display.enable=1. The T6031 GPU starts its firmware only
 * after this native display is up, so enable=0 (or a display that never
 * comes up) keeps the GPU off for that boot too: it is not a display-only
 * switch. The default changes no check of the
 * handoff or the PMP below. The PMP values must come from the boot loader, in
 * /chosen/asahi,t6031-pmp; there is no fallback. Its PMP topology is
 * reused from the device tree, as on T8122. The T6031 display power
 * states nest the other way round (the DCP's DISP_FE under DISP_CPU under
 * DISP_SYS), and its PMP power states are in another PMGR block. On every
 * T6031 boot the gate also logs what the boot loader handed over, so that
 * a boot without the option already records what the next one needs.
 *
 * On T6041 (the display coprocessor of the M4 Pro, apple,t6040) m1n1 hands
 * over the DCP that iBoot leaves running. It marks the DCP, the display
 * subsystem and the DCP's piodma child with apple,t6041-handoff and names
 * the display firmware in apple,firmware-compat. There is no PMP to add or
 * complete: the device tree describes it and its own driver starts it. The
 * gate checks the handoff and the running DCP, enables the internal display
 * nodes, and powers up the external display chain in stages once the PMP is
 * bound, even long after the boot (see gate_t6041_run()). This part was a
 * driver of its own, t6041-display-gate.c, by Ryan Murray and František
 * Balázsy.
 *
 * On the M4 Max (J614c and J616c, apple,t6041) the internal display is
 * handed over and enabled as on the M4 Pro. The PMP never runs there: iBoot
 * leaves the external chains powered and the boot loader fills each external
 * display processor's firmware memory. The gate enables a chain once its
 * processor has that memory and the display subsystem is enabled, before any
 * device exists (see gate_t6041_max_run()).
 */

#define pr_fmt(fmt) fmt

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fwnode.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/memblock.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/soc/apple/t8132.h>
#include <linux/string.h>
#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <video/nomodeset.h>

#ifdef CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST
#include <kunit/static_stub.h>
#endif

enum {
	GATE_DCP_DART,
	GATE_DISP0_DART,
	GATE_DCP_MBOX,
	GATE_DCP,
	GATE_DISPLAY,
	GATE_NR_NODES,
};

enum {
	PMP_PS_DISP_SYS,
	PMP_PS_DISP_FE,
	PMP_PS_DISP_CPU,
	PMP_PS_PMP,
	PMP_PS_PMS_SRAM,
	PMP_PS_NR,
};

static const char *const pmp_ps_labels[PMP_PS_NR] __initconst = {
	"disp_sys", "disp_fe", "disp_cpu", "pmp", "pms_sram",
};

#define PMGR_PS_ACTIVE	15

/*
 * Built-in overlays with the PMP, its DART and mailbox, and its report. The
 * T6030 one has no per-Mac values: the gate copies them from the node the
 * boot loader fills from this Mac's ADT. The J516S one carries the values
 * of one J516S, for a boot loader that passes none.
 */
extern const u8 __dtbo_t6030_pmp_begin[];
extern const u8 __dtbo_t6030_pmp_end[];
extern const u8 __dtbo_t6030_j516s_pmp_begin[];
extern const u8 __dtbo_t6030_j516s_pmp_end[];

/* This Mac's PMP values, from the boot loader, or NULL. */
static struct device_node *gate_pmp_values __initdata;

/*
 * The PMP firmware image the T6030 PMP support was brought up with, as the
 * J516S overlay names it. The report driver starts only the image the PMP
 * node names; values from the boot loader must name this one too.
 */
static const char gate_pmp_uuid[] __initconst = "2F4EB4C4-001B-3ACF-A9A0-68D8E42FC3A7";

/* The T8122 PMP image of the 14.x system firmware. */
static const char gate_t8122_pmp_uuid[] __initconst = "3B18C886-4C70-349B-B1C4-70F35DE2C5CD";

/*
 * The T6031 PMP image, as the J516C ADT names it (iop-pmp-nub, passed on by
 * the boot loader in /chosen/asahi,t6031-pmp). It is not the T6030 image.
 */
static const char gate_t6031_pmp_uuid[] __initconst = "07FF4C5B-CB2E-3DE6-95FF-3B107D19B7AB";

/*
 * apple_t6030_display.enable and apple_t8122_display.enable: the native
 * display is on unless one of them is 0.
 */
static bool gate_requested __initdata = true;

/*
 * apple_t6031_display.enable: -1 while it is not on the command line, else 0
 * or 1. Unset, the native display is on for the exact J514C/J516C + T6031 pairs.
 */
static int gate_t6031_switch __initdata = -1;

/* Only the current KUnit case may substitute its synthetic root identity. */
static bool __init gate_t6031_is_compatible(const char *compat)
{
#ifdef CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST
	KUNIT_STATIC_STUB_REDIRECT(gate_t6031_is_compatible, compat);
#endif
	return of_machine_is_compatible(compat);
}

static const char *__init gate_t6031_default_board(void)
{
	if (!gate_t6031_is_compatible("apple,t6031"))
		return NULL;
	if (gate_t6031_is_compatible("apple,j516c"))
		return "apple,j516c";
	if (gate_t6031_is_compatible("apple,j514c"))
		return "apple,j514c";
	return NULL;
}
/* The resolved request, set by gate_run() on a T6031. */
static bool gate_t6031_requested __initdata;

/* Whether the T6031 native display is requested: an explicit 0 or 1 wins. */
static bool __init gate_t6031_wanted(int sw, bool default_board)
{
	return sw < 0 ? default_board : sw;
}

/* T6034 has an independent owner disable and never admits an unmeasured board. */
static int gate_t6034_switch __initdata = -1;
static bool gate_t6034_requested __initdata;

static bool __init gate_t6034_default_board(void)
{
	return of_machine_is_compatible("apple,t6034") &&
	       of_machine_is_compatible("apple,j514m");
}

static bool __init gate_t6034_wanted(int sw)
{
	return gate_t6034_default_board() && gate_t6031_wanted(sw, true);
}

/*
 * J516M: the T6031 display layout on a T6034, with J514M's owner switch
 * (apple_t6034_display.enable) and the same default: on unless it is 0.
 */
static bool __init gate_j516m_default_board(void)
{
	return of_machine_is_compatible("apple,t6034") &&
	       of_machine_is_compatible("apple,j516m");
}

static bool __init gate_j516m_wanted(int sw)
{
	return gate_j516m_default_board() && gate_t6031_wanted(sw, true);
}

/* One SoC's internal display handoff, as the gate checks and completes it */
struct gate_soc {
	const char *machine;		/* root compatible */
	const char *prefix;		/* log prefix */
	const char *name;
	const char *dcp_compat;
	const char *display_compat;
	const char *marker;		/* set to <1> by the boot loader */
	const char *pwrstate_compat;	/* the PMGR power states */
	const char *dcp_full_name;	/* the DCP node the PMP overlay expects, under /soc */
	const char *pmp_values;		/* the boot loader's PMP values for this Mac */
	const char *pmp_uuid;		/* the PMP image the PMP support is written for */
	const char *report_compat;	/* the PMP report the overlay adds, or the reused one */
	const char *disp_entry;		/* its display request entry, which the DCP waits for */
	const u8 *pmp_dtbo, *pmp_dtbo_end;	/* NULL: no PMP description built in */
	/*
	 * Set: the device tree describes the PMP (pmp_compat), its DART, its
	 * mailbox and its report, and the gate completes those nodes instead of
	 * adding an overlay. ans_entry is the storage request the PMP must see
	 * when it starts; dispext_entry the external display request.
	 */
	bool reuse_pmp;
	const char *pmp_compat;
	const char *ans_entry;
	const char *dispext_entry;
	bool records;			/* PTD bits are soc-device records (T8122) */
	const char *fallback_board;	/* gets the fallback overlay without boot loader values */
	const u8 *fallback_dtbo, *fallback_dtbo_end;
	bool *requested;		/* the command line switch */
	const char *opt_in;		/* set: off unless this is on the command line */
	/* The DCP's power state and its parents, nearest first, as PMP_PS_*. */
	u8 ps_chain[3];
	bool pmp_ps_any_pmgr;		/* PMP and PMS_SRAM are not beside the DCP's power state */
	bool describe;			/* log the boot loader's handoff on every boot */
	const char *fw_uuids;		/* /chosen strings naming each firmware image */
	bool quiet_without_nodes;	/* no display nodes in the device tree is normal */
	bool dcpext;			/* the external display processors and their DARTs */
	/*
	 * Set: this handoff has its own sequence instead of gate_run() (T6041:
	 * the DCP is adopted running, and the PMP is the device tree's own).
	 */
	void (*run)(void);
};

static const struct gate_soc gate_t6030 __initconst = {
	.machine = "apple,t6030",
	.prefix = "apple-t6030-display: ",
	.name = "T6030",
	.dcp_compat = "apple,t6030-dcp",
	.display_compat = "apple,t6030-display-subsystem",
	.marker = "apple,t6030-handoff",
	.pwrstate_compat = "apple,t6030-pmgr-pwrstate",
	.dcp_full_name = "dcp@28ec00000",
	.pmp_values = "/chosen/asahi,t6030-pmp",
	.pmp_uuid = gate_pmp_uuid,
	.report_compat = "apple,t6030-pmp-v2-report",
	.disp_entry = "report@7",
	.pmp_dtbo = __dtbo_t6030_pmp_begin,
	.pmp_dtbo_end = __dtbo_t6030_pmp_end,
	.fallback_board = "apple,j516s",
	.fallback_dtbo = __dtbo_t6030_j516s_pmp_begin,
	.fallback_dtbo_end = __dtbo_t6030_j516s_pmp_end,
	.requested = &gate_requested,
	.ps_chain = { PMP_PS_DISP_CPU, PMP_PS_DISP_FE, PMP_PS_DISP_SYS },
	.dcpext = true,
};

/*
 * T8122 (M3 MacBook Air): the same handoff and power states. The PMP and its
 * report are the device tree's own. The PMP values must come from the boot
 * loader: there is no built-in fallback.
 */
static const struct gate_soc gate_t8122 __initconst = {
	.machine = "apple,t8122",
	.prefix = "apple-t8122-display: ",
	.name = "T8122",
	.dcp_compat = "apple,t8122-dcp",
	.display_compat = "apple,t8122-display-subsystem",
	.marker = "apple,t8122-handoff",
	.pwrstate_compat = "apple,t8122-pmgr-pwrstate",
	.dcp_full_name = "dcp@28ec00000",
	.pmp_values = "/chosen/asahi,t8122-pmp",
	.pmp_uuid = gate_t8122_pmp_uuid,
	.report_compat = "apple,t8122-pmp-v2-report",
	.disp_entry = "report@7",
	.reuse_pmp = true,
	.pmp_compat = "apple,t8122-pmp-v2",
	.ans_entry = "report@11",
	.dispext_entry = "report@8",
	.records = true,
	.requested = &gate_requested,
	.ps_chain = { PMP_PS_DISP_CPU, PMP_PS_DISP_FE, PMP_PS_DISP_SYS },
	.quiet_without_nodes = true,
};

/*
 * T6031 (M3 Max): the T6030 handoff at the T6031 addresses, on by default on
 * J514C/J516C and off unless apple_t6031_display.enable=1 on other boards;
 * apple_t6031_display.enable=0 keeps it, and with it the GPU, off everywhere.
 * The PMP values must come from the boot
 * loader: there is no built-in fallback. The PMP image is T6031's own;
 * any other, the T6030 image included, is refused. The report driver checks
 * the image's SRAM layout before it starts the PMP. The internal display's
 * PMP request is bit 0x10.
 */
static const struct gate_soc gate_t6031 __initconst = {
	.machine = "apple,t6031",
	.prefix = "apple-t6031-display: ",
	.name = "T6031",
	.dcp_compat = "apple,t6031-dcp",
	.display_compat = "apple,t6031-display-subsystem",
	.marker = "apple,t6031-handoff",
	.pwrstate_compat = "apple,t6031-pmgr-pwrstate",
	.dcp_full_name = "dcp@386c00000",
	.pmp_values = "/chosen/asahi,t6031-pmp",
	.pmp_uuid = gate_t6031_pmp_uuid,
	.report_compat = "apple,t6031-pmp-v2-report",
	.disp_entry = "report@10",
	.reuse_pmp = true,
	.pmp_compat = "apple,t6031-pmp-v2",
	.ans_entry = "report@21",
	.requested = &gate_t6031_requested,
	.opt_in = "apple_t6031_display.enable=1",
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6031-fw-uuids",
};

static void __init gate_t6041_run(void);

/*
 * T6041 (M4 Pro): the DCP is adopted running; no PMP is added or completed,
 * and the command line options of the other SoCs do not apply.
 */
static const struct gate_soc gate_t6041 __initconst = {
	.machine = "apple,t6040",
	.prefix = "apple-t6041-display: ",
	.name = "T6041",
	.dcp_compat = "apple,t6041-dcp",
	.display_compat = "apple,display-subsystem",
	.marker = "apple,t6041-handoff",
	.run = gate_t6041_run,
};

static void __init gate_t6041_max_run(void);

/*
 * T6041 (M4 Max): the M4 Pro's internal display handoff; no PMP, so the
 * external chains follow the boot loader's firmware memory.
 */
static const struct gate_soc gate_t6041_max __initconst = {
	.machine = "apple,t6041",
	.prefix = "apple-t6041-display: ",
	.name = "T6041",
	.dcp_compat = "apple,t6041-dcp",
	.display_compat = "apple,display-subsystem",
	.marker = "apple,t6041-handoff",
	.run = gate_t6041_max_run,
};

/* Fresh J514M ADT: shared PMP image/layout, separate root and handoff. */
static const struct gate_soc gate_t6034 __initconst = {
	.machine = "apple,t6034",
	.prefix = "apple-t6034-display: ",
	.name = "T6034",
	.dcp_compat = "apple,t6034-dcp",
	.display_compat = "apple,t6034-display-subsystem",
	.marker = "apple,t6034-handoff",
	.pwrstate_compat = "apple,t6031-pmgr-pwrstate",
	.dcp_full_name = "dcp@386c00000",
	.pmp_values = "/chosen/asahi,t6034-pmp",
	.pmp_uuid = gate_t6031_pmp_uuid,
	.report_compat = "apple,t6034-pmp-v2-report",
	.disp_entry = "report@10",
	.reuse_pmp = true,
	.pmp_compat = "apple,t6034-pmp-v2",
	.ans_entry = "report@21",
	.requested = &gate_t6034_requested,
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6034-fw-uuids",
};

static bool gate_j516m_requested __initdata;

/* ADT-confirmed T6031 display layout, admitted only on the J516M. */
static const struct gate_soc gate_j516m __initconst = {
	.machine = "apple,j516m",
	.prefix = "apple-t6034-display: ",
	.name = "T6034",
	.dcp_compat = "apple,t6031-dcp",
	.display_compat = "apple,t6031-display-subsystem",
	.marker = "apple,t6031-handoff",
	.pwrstate_compat = "apple,t6031-pmgr-pwrstate",
	.dcp_full_name = "dcp@386c00000",
	.pmp_values = "/chosen/asahi,t6031-pmp",
	.pmp_uuid = gate_t6031_pmp_uuid,
	.report_compat = "apple,t6031-pmp-v2-report",
	.disp_entry = "report@10",
	.reuse_pmp = true,
	.pmp_compat = "apple,t6031-pmp-v2",
	.ans_entry = "report@21",
	.requested = &gate_j516m_requested,
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6031-fw-uuids",
	/* External displays (dcpext) are not qualified on the J516M: left off. */
};

static const struct gate_soc *const gate_socs[] __initconst = {
	&gate_t6030,
	&gate_t8122,
	&gate_t6031,
	&gate_t6041,
	&gate_t6041_max,
	&gate_j516m,
	&gate_t6034,
};

/* The SoC this boot runs on, if the gate serves it. */
static const struct gate_soc *gate_soc __initdata;

#define gate_info(fmt, ...) pr_info("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)
#define gate_warn(fmt, ...) pr_warn("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)
#define gate_err(fmt, ...) pr_err("%s" fmt, gate_soc->prefix, ##__VA_ARGS__)

/* The boot loader's values the gate copies onto the PMP node. */
static bool __init gate_pmp_value_wanted(const struct property *prop)
{
	if (!strcmp(prop->name, "apple,board-id") ||
	    !strcmp(prop->name, "apple,dram-vendor-id") ||
	    !strcmp(prop->name, "apple,dram-capacity"))
		return prop->length == sizeof(u32);
	return strstarts(prop->name, "apple,tunable-");
}

/* Kept after a successful apply: the live tree now holds its properties. */
static struct of_changeset gate_cs;
static struct of_changeset gate_pmp_cs;
/* The PMP overlay, once applied; only the KUnit test removes it again. */
static int gate_pmp_ovcs __initdata;
/* gate_pmp_cs completed the device tree's own PMP; only the KUnit test reverts it. */
static bool gate_pmp_reused __initdata;

/* The device tree's own PMP topology, found by gate_reuse_resolve(). */
struct gate_reuse {
	struct device_node *report, *pmp, *dart, *mbox, *disp, *ans;
};

static struct gate_reuse gate_reuse __initdata;

static void __init gate_reuse_put(void)
{
	of_node_put(gate_reuse.ans);
	of_node_put(gate_reuse.disp);
	of_node_put(gate_reuse.mbox);
	of_node_put(gate_reuse.dart);
	of_node_put(gate_reuse.pmp);
	of_node_put(gate_reuse.report);
	memset(&gate_reuse, 0, sizeof(gate_reuse));
}

static int __init gate_setup(char *arg)
{
	return kstrtobool(arg, &gate_requested);
}
early_param("apple_t6030_display.enable", gate_setup);

static int __init gate_setup_t8122(char *arg)
{
	return gate_setup(arg);
}
early_param("apple_t8122_display.enable", gate_setup_t8122);

/*
 * Its own switch: the T6030 and T8122 options do not change it, nor it them.
 * A value kstrtobool() rejects sets nothing, so the board default stays.
 */
static int __init gate_setup_t6031(char *arg)
{
	bool on;
	int ret = kstrtobool(arg, &on);

	if (!ret)
		gate_t6031_switch = on;
	return ret;
}
early_param("apple_t6031_display.enable", gate_setup_t6031);

static int __init gate_setup_t6034(char *arg)
{
	bool on;
	int ret = kstrtobool(arg, &on);

	if (!ret)
		gate_t6034_switch = on;
	return ret;
}
early_param("apple_t6034_display.enable", gate_setup_t6034);

/* Returns the only node compatible with @compat, or NULL if there are none or several. */
static struct device_node *__init gate_find_one(const char *compat)
{
	struct device_node *np, *found = NULL;

	for_each_compatible_node(np, NULL, compat) {
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}

	return found;
}

static bool __init gate_marked(const struct device_node *np)
{
	u32 val;

	return !of_property_read_u32(np, gate_soc->marker, &val) && val == 1;
}

static bool __init gate_disabled(const struct device_node *np)
{
	const char *status;

	return !of_property_read_string(np, "status", &status) &&
	       !strcmp(status, "disabled");
}

/*
 * Returns the node named by the single entry of @list in @np, if that entry
 * has @nargs cells (the first equal to @arg0) and the node is compatible
 * with @compat. Returns NULL otherwise.
 */
static struct device_node *__init gate_target(const struct device_node *np,
					      const char *list,
					      const char *cells_name,
					      int nargs, u32 arg0,
					      const char *compat)
{
	struct of_phandle_args args;

	if (of_count_phandle_with_args(np, list, cells_name) != 1)
		return NULL;
	if (of_parse_phandle_with_args(np, list, cells_name, 0, &args))
		return NULL;

	if (args.args_count != nargs || (nargs && args.args[0] != arg0) ||
	    !of_device_is_compatible(args.np, compat)) {
		of_node_put(args.np);
		return NULL;
	}

	return args.np;
}

static int __init gate_resolve(struct device_node **np)
{
	int i;

	struct device_node *any;

	if (gate_soc->quiet_without_nodes) {
		any = of_find_compatible_node(NULL, NULL, gate_soc->dcp_compat);
		if (!any)
			return -ENODEV;
		of_node_put(any);
	}

	np[GATE_DCP] = gate_find_one(gate_soc->dcp_compat);
	if (!np[GATE_DCP]) {
		gate_warn("not enabling: need exactly one %s node\n", gate_soc->dcp_compat);
		return -ENODEV;
	}

	np[GATE_DISPLAY] = gate_find_one(gate_soc->display_compat);
	if (!np[GATE_DISPLAY]) {
		gate_warn("not enabling: need exactly one %s node\n", gate_soc->display_compat);
		return -ENODEV;
	}

	if (!gate_marked(np[GATE_DCP]) || !gate_marked(np[GATE_DISPLAY])) {
		gate_warn("not enabling: the boot loader did not set %s\n", gate_soc->marker);
		return -EPERM;
	}

	np[GATE_DCP_DART] = gate_target(np[GATE_DCP], "iommus", "#iommu-cells",
					1, 5, "apple,t8110-dart");
	if (!np[GATE_DCP_DART]) {
		gate_warn("not enabling: %pOF iommus is not one apple,t8110-dart stream 5\n",
			np[GATE_DCP]);
		return -EINVAL;
	}

	np[GATE_DISP0_DART] = gate_target(np[GATE_DISPLAY], "iommus",
					  "#iommu-cells", 1, 0,
					  "apple,t8110-dart");
	if (!np[GATE_DISP0_DART]) {
		gate_warn("not enabling: %pOF iommus is not one apple,t8110-dart stream 0\n",
			np[GATE_DISPLAY]);
		return -EINVAL;
	}

	np[GATE_DCP_MBOX] = gate_target(np[GATE_DCP], "mboxes", "#mbox-cells",
					0, 0, "apple,asc-mailbox-v4");
	if (!np[GATE_DCP_MBOX]) {
		gate_warn("not enabling: %pOF mboxes is not one apple,asc-mailbox-v4\n",
			np[GATE_DCP]);
		return -EINVAL;
	}

	if (np[GATE_DCP_DART] == np[GATE_DISP0_DART]) {
		gate_warn("not enabling: the DCP and the display subsystem share %pOF\n",
			np[GATE_DCP_DART]);
		return -EINVAL;
	}

	for (i = 0; i < GATE_NR_NODES; i++) {
		if (!gate_disabled(np[i])) {
			gate_warn("not enabling: %pOF is not disabled\n", np[i]);
			return -EBUSY;
		}
	}

	return 0;
}

static int __init gate_apply(struct device_node **np)
{
	int i, ret = 0;

	of_changeset_init(&gate_cs);

	for (i = 0; i < GATE_NR_NODES; i++) {
		ret = of_changeset_update_prop_string(&gate_cs, np[i], "status",
						      "okay");
		if (ret)
			break;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_cs);

	if (ret) {
		of_changeset_destroy(&gate_cs);
		gate_err("not enabling: changeset failed: %d\n", ret);
		return ret;
	}

	gate_info("enabled %pOF, %pOF, %pOF, %pOF and %pOF\n",
		np[GATE_DCP_DART], np[GATE_DISP0_DART], np[GATE_DCP_MBOX],
		np[GATE_DCP], np[GATE_DISPLAY]);

	return 0;
}

/* Disables the display nodes again when the PMP could not be added. */
static void __init gate_revert(void)
{
	int ret = of_changeset_revert(&gate_cs);

	if (ret) {
		gate_err("could not disable the display nodes again: %d\n", ret);
		return;
	}
	of_changeset_destroy(&gate_cs);
	gate_info("display nodes disabled again, display stays on the boot framebuffer\n");
}

/* The single power domain of @np, if it is one of the SoC's power states. */
static struct device_node *__init gate_ps_parent(struct device_node *np)
{
	return gate_target(np, "power-domains", "#power-domain-cells", 0, 0,
			   gate_soc->pwrstate_compat);
}

static bool __init gate_ps_is(const struct device_node *np, const char *label)
{
	const char *name;

	return np && of_device_is_compatible(np, gate_soc->pwrstate_compat) &&
	       !of_property_read_string(np, "label", &name) && !strcmp(name, label);
}

/* The only power state below @pmgr labelled @label. */
static struct device_node *__init gate_ps_child(struct device_node *pmgr, const char *label)
{
	struct device_node *child, *found = NULL;

	for_each_child_of_node(pmgr, child) {
		if (!gate_ps_is(child, label))
			continue;
		if (found) {
			of_node_put(child);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(child);
	}

	return found;
}

/* The only power state of this SoC labelled @label, in any PMGR block. */
static struct device_node *__init gate_ps_unique(const char *label)
{
	struct device_node *np, *found = NULL;

	for_each_compatible_node(np, NULL, gate_soc->pwrstate_compat) {
		if (!gate_ps_is(np, label))
			continue;
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}

	return found;
}

/* The available report entry of @report named @full_name. */
static struct device_node *__init gate_report_entry(struct device_node *report,
						    const char *full_name)
{
	struct device_node *child;

	for_each_available_child_of_node(report, child) {
		if (!strcmp(of_node_full_name(child), full_name) &&
		    of_device_is_compatible(child, "apple,t6000-pmp-v2-report-entry"))
			return child;
	}
	return NULL;
}

/*
 * The PMP topology a device tree describes itself (T8122, T6031): the report,
 * linked to its PMP by apple,pmp; the PMP, disabled and with no values yet,
 * with one DART stream and one ASC mailbox; the display request entry, always
 * on and the DCP's apple,pmp-report; and the storage request entry. Nothing
 * here is changed. The report driver starts the PMP, and enables it, once the
 * display driver asks; the DART and the mailbox are left as described.
 */
static int __init gate_reuse_resolve(struct device_node **np)
{
	struct gate_reuse *r = &gate_reuse;
	struct device_node *other, *link;
	bool single;

	r->report = gate_find_one(gate_soc->report_compat);
	if (!r->report || !of_device_is_available(r->report)) {
		gate_warn("PMP not added: need exactly one available %s node\n",
			  gate_soc->report_compat);
		return -ENODEV;
	}
	r->pmp = of_parse_phandle(r->report, "apple,pmp", 0);
	other = gate_find_one(gate_soc->pmp_compat);
	single = r->pmp && r->pmp == other;
	of_node_put(other);
	if (!single || !of_device_is_compatible(r->pmp, "apple,t6000-pmp-v2") ||
	    !gate_disabled(r->pmp) || !r->pmp->phandle) {
		gate_warn("PMP not added: %pOF apple,pmp is not the only %s node, disabled\n",
			  r->report, gate_soc->pmp_compat);
		return -ENODEV;
	}
	if (of_property_present(r->pmp, "apple,board-id") ||
	    of_property_present(r->pmp, "apple,dram-vendor-id") ||
	    of_property_present(r->pmp, "apple,tunable-uuid")) {
		gate_warn("PMP not added: %pOF already has PMP values\n", r->pmp);
		return -EEXIST;
	}
	r->dart = gate_target(r->pmp, "iommus", "#iommu-cells", 1, 0, "apple,t8110-dart");
	r->mbox = gate_target(r->pmp, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	if (!r->dart || !r->mbox) {
		gate_warn("PMP not added: %pOF is not on one apple,t8110-dart stream 0 and one apple,asc-mailbox-v4\n",
			  r->pmp);
		return -EINVAL;
	}

	r->disp = gate_report_entry(r->report, gate_soc->disp_entry);
	if (!r->disp || !r->disp->phandle ||
	    !of_property_read_bool(r->disp, "apple,always-on") ||
	    of_property_read_bool(r->disp, "apple,no-ack")) {
		gate_warn("PMP not added: %pOF has no acknowledged always-on display request %s\n",
			  r->report, gate_soc->disp_entry);
		return -ENODEV;
	}
	link = of_parse_phandle(np[GATE_DCP], "apple,pmp-report", 0);
	of_node_put(link);
	if (link != r->disp) {
		gate_warn("PMP not added: %pOF apple,pmp-report is not %pOF\n",
			  np[GATE_DCP], r->disp);
		return -EINVAL;
	}
	r->ans = gate_report_entry(r->report, gate_soc->ans_entry);
	if (!r->ans) {
		gate_warn("PMP not added: %pOF has no storage request %s\n",
			  r->report, gate_soc->ans_entry);
		return -ENODEV;
	}
	return 0;
}

/*
 * The display domains are the DCP's power domain and its parents, in the
 * SoC's order; the PMP and PMS_SRAM domains are found by label beside them,
 * or anywhere on a SoC that has them in another PMGR block.
 */
static int __init gate_pmp_resolve(struct device_node **np, struct device_node **ps,
				   struct device_node **aic)
{
	const u8 *chain = gate_soc->ps_chain;
	struct device_node *pmgr, *other;
	int i;

	if (!gate_soc->reuse_pmp && !gate_soc->pmp_dtbo) {
		gate_warn("PMP not added: no %s PMP description is built in\n", gate_soc->name);
		return -ENODEV;
	}

	gate_pmp_values = of_find_node_by_path(gate_soc->pmp_values);
	if (gate_pmp_values) {
		const char *uuid = NULL;
		u32 v;

		if (of_property_read_u32(gate_pmp_values, "apple,board-id", &v) ||
		    of_property_read_u32(gate_pmp_values, "apple,dram-vendor-id", &v) ||
		    of_property_read_string(gate_pmp_values, "apple,tunable-uuid", &uuid)) {
			gate_warn("PMP not added: %pOF is incomplete\n", gate_pmp_values);
			of_node_put(gate_pmp_values);
			gate_pmp_values = NULL;
			return -EINVAL;
		}
		if (strcmp(uuid, gate_soc->pmp_uuid)) {
			gate_warn("PMP not added: this Mac's PMP firmware image is %s, not %s\n",
				uuid, gate_soc->pmp_uuid);
			of_node_put(gate_pmp_values);
			gate_pmp_values = NULL;
			return -EINVAL;
		}
	}
	if (!gate_pmp_values && (!gate_soc->fallback_board ||
				 !of_machine_is_compatible(gate_soc->fallback_board))) {
		gate_warn("PMP not added: the boot loader passed no PMP values for this Mac\n");
		return -ENODEV;
	}

	if (gate_soc->reuse_pmp) {
		int ret = gate_reuse_resolve(np);

		if (ret)
			return ret;
	} else {
		other = of_find_compatible_node(NULL, NULL, "apple,t6000-pmp-v2");
		if (!other)
			other = of_find_compatible_node(NULL, NULL, gate_soc->report_compat);
		if (other) {
			gate_warn("PMP not added: %pOF already exists\n", other);
			of_node_put(other);
			return -EEXIST;
		}

		/* The overlay's DCP fragment names this path. */
		if (strcmp(of_node_full_name(np[GATE_DCP]), gate_soc->dcp_full_name) ||
		    !of_node_name_eq(np[GATE_DCP]->parent, "soc") ||
		    !of_node_is_root(np[GATE_DCP]->parent->parent)) {
			gate_warn("PMP not added: the DCP is %pOF, not /soc/%s\n",
				  np[GATE_DCP], gate_soc->dcp_full_name);
			return -EINVAL;
		}
	}

	ps[chain[0]] = gate_ps_parent(np[GATE_DCP]);
	if (ps[chain[0]])
		ps[chain[1]] = gate_ps_parent(ps[chain[0]]);
	if (ps[chain[1]])
		ps[chain[2]] = gate_ps_parent(ps[chain[1]]);
	if (gate_soc->pmp_ps_any_pmgr) {
		ps[PMP_PS_PMP] = gate_ps_unique("pmp");
		ps[PMP_PS_PMS_SRAM] = gate_ps_unique("pms_sram");
	} else {
		pmgr = of_get_parent(ps[PMP_PS_DISP_CPU]);
		if (pmgr) {
			ps[PMP_PS_PMP] = gate_ps_child(pmgr, "pmp");
			ps[PMP_PS_PMS_SRAM] = gate_ps_child(pmgr, "pms_sram");
			of_node_put(pmgr);
		}
	}
	for (i = 0; i < PMP_PS_NR; i++) {
		if (!gate_ps_is(ps[i], pmp_ps_labels[i])) {
			gate_warn("PMP not added: no %s power state where expected\n",
				pmp_ps_labels[i]);
			return -ENODEV;
		}
	}
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU; i++) {
		if (!of_property_read_bool(ps[i], "apple,inherited-on")) {
			gate_warn("PMP not added: %pOF is not apple,inherited-on\n", ps[i]);
			return -EINVAL;
		}
	}

	/* Only the overlay's nodes need it: the reused ones have their own. */
	if (gate_soc->reuse_pmp)
		return 0;
	*aic = of_parse_phandle(np[GATE_DCP_MBOX], "interrupt-parent", 0);
	if (!*aic || !of_property_read_bool(*aic, "interrupt-controller")) {
		gate_warn("PMP not added: %pOF has no interrupt parent\n", np[GATE_DCP_MBOX]);
		return -EINVAL;
	}

	return 0;
}

/* Sets the u32 property @name of @np, adding it if it is missing. */
static int __init gate_set_u32(struct of_changeset *cs, struct device_node *np,
			       const char *name, u32 val)
{
	struct property *prop;
	__be32 *value;

	/* Kept for good: the live tree refers to it once applied. */
	prop = kzalloc_obj(*prop);
	value = kmalloc_obj(*value);
	if (!prop || !value) {
		kfree(prop);
		kfree(value);
		return -ENOMEM;
	}
	*value = cpu_to_be32(val);
	prop->name = (char *)name;
	prop->length = sizeof(*value);
	prop->value = value;

	return of_changeset_update_property(cs, np, prop);
}

/*
 * Adds the boot loader's PMP values (gate_pmp_value_wanted()) to @pmp, which
 * the T6030 overlay leaves without them. Returns the number added.
 */
static int __init gate_pmp_copy_values(struct of_changeset *cs, struct device_node *pmp)
{
	struct property *src, *prop;
	int n = 0, ret;

	for_each_property_of_node(gate_pmp_values, src) {
		if (!gate_pmp_value_wanted(src))
			continue;
		/* Kept for good: the live tree refers to it once applied. */
		prop = kzalloc_obj(*prop);
		if (!prop)
			return -ENOMEM;
		prop->name = kstrdup(src->name, GFP_KERNEL);
		prop->value = kmemdup(src->value, src->length, GFP_KERNEL);
		prop->length = src->length;
		if (!prop->name || (src->length && !prop->value)) {
			kfree(prop->name);
			kfree(prop->value);
			kfree(prop);
			return -ENOMEM;
		}
		ret = of_changeset_add_property(cs, pmp, prop);
		if (ret) {
			kfree(prop->name);
			kfree(prop->value);
			kfree(prop);
			return ret;
		}
		n++;
	}
	return n;
}

static int __init gate_gpu_pmp_link(struct of_changeset *cs, struct device_node *pmp)
{
	struct device_node *gpu, *linked;
	bool t6030 = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) && gate_soc == &gate_t6030;
	int ret = 0;

	if (gate_soc != &gate_t8122 && !t6030)
		return 0;
	gpu = t6030 ? gate_find_one("apple,agx-t6030") :
		of_find_compatible_node(NULL, NULL, "apple,agx-t8122");
	if (!gpu || !of_device_is_available(gpu))
		goto out;
	/* Do not replace a malformed or unresolved pre-existing GPU supplier. */
	if (t6030 && of_property_present(gpu, "apple,pmp") &&
	    of_property_count_u32_elems(gpu, "apple,pmp") != 1) {
		ret = -EINVAL;
		goto out;
	}
	linked = of_parse_phandle(gpu, "apple,pmp", 0);
	if (!pmp || !pmp->phandle || (linked && linked != pmp) ||
	    (t6030 && of_property_present(gpu, "apple,pmp") && !linked))
		ret = -EINVAL;
	else if (!linked)
		ret = of_changeset_add_prop_u32(cs, gpu, "apple,pmp", pmp->phandle);
	of_node_put(linked);
out:
	of_node_put(gpu);
	return ret;
}

/* Adds the boolean @name to @np unless it is there already. */
static int __init gate_add_bool(struct of_changeset *cs, struct device_node *np,
				const char *name)
{
	if (of_property_read_bool(np, name))
		return 0;
	return of_changeset_add_prop_bool(cs, np, name);
}

/*
 * Completes the device tree's own PMP topology (gate_reuse_resolve()) in one
 * changeset, as the overlay path completes the overlay's:
 *  - the display request is requested by soc-device record where the PTD
 *    is indexed that way (T8122);
 *  - the storage request is always on, and not acknowledged, as the PMP
 *    keeps a block up only if its request was set when it started;
 *  - the T8122 GPU, if available, is linked to the PMP;
 *  - the display domains' floor, and the PMP and PMS_SRAM domains kept on;
 *  - this Mac's PMP values on the PMP node.
 * The PMP stays disabled: the report driver enables it once it starts it.
 */
static int __init gate_reuse_apply(struct device_node **ps)
{
	struct gate_reuse *r = &gate_reuse;
	int ret = 0, i, values = 0;

	of_changeset_init(&gate_pmp_cs);
	if (gate_soc->records)
		ret = gate_add_bool(&gate_pmp_cs, r->disp, "apple,soc-device-record");
	if (!ret)
		ret = gate_add_bool(&gate_pmp_cs, r->ans, "apple,always-on");
	/* The PMP does not acknowledge storage requests. */
	if (!ret)
		ret = gate_add_bool(&gate_pmp_cs, r->ans, "apple,no-ack");
	/* The current14 T8122 GPU needs an inner AGX vote from this Mac's PMP. */
	if (!ret)
		ret = gate_gpu_pmp_link(&gate_pmp_cs, r->pmp);
	/* The display domains' floor, applied by the PMGR driver when it probes. */
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU && !ret; i++)
		ret = gate_set_u32(&gate_pmp_cs, ps[i], "apple,min-state", PMGR_PS_ACTIVE);
	/* Linux would otherwise power these off as unused. */
	for (i = PMP_PS_PMP; i <= PMP_PS_PMS_SRAM && !ret; i++)
		ret = gate_add_bool(&gate_pmp_cs, ps[i], "apple,always-on");
	if (!ret) {
		values = gate_pmp_copy_values(&gate_pmp_cs, r->pmp);
		ret = values < 0 ? values : 0;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_pmp_cs);
	if (ret) {
		of_changeset_destroy(&gate_pmp_cs);
		gate_err("PMP not added: changeset failed: %d\n", ret);
		return ret;
	}
	gate_pmp_reused = true;

	gate_info("PMP: this Mac's %d values from %pOF on %pOF\n", values, gate_pmp_values,
		  r->pmp);
	gate_info("completed %pOF with its display (%pOF) and storage (%pOF) requests; minimum power state of %s, %s and %s raised to active; %s and %s kept on\n",
		  r->report, r->disp, r->ans,
		  pmp_ps_labels[PMP_PS_DISP_SYS], pmp_ps_labels[PMP_PS_DISP_FE],
		  pmp_ps_labels[PMP_PS_DISP_CPU], pmp_ps_labels[PMP_PS_PMP],
		  pmp_ps_labels[PMP_PS_PMS_SRAM]);
	return 0;
}

static int __init gate_pmp_apply(struct device_node *dcp, struct device_node **ps,
				 struct device_node *aic)
{
	const u8 *dtbo = gate_pmp_values ? gate_soc->pmp_dtbo : gate_soc->fallback_dtbo;
	const size_t size = gate_pmp_values ?
		gate_soc->pmp_dtbo_end - gate_soc->pmp_dtbo :
		gate_soc->fallback_dtbo_end - gate_soc->fallback_dtbo;
	struct device_node *report, *disp = NULL, *pmp = NULL, *dart = NULL, *mbox = NULL;
	int ovcs_id = 0, ret, i, values = 0;

	if (gate_soc->reuse_pmp)
		return gate_reuse_apply(ps);

	ret = of_overlay_fdt_apply(dtbo, size, &ovcs_id, NULL);
	if (ret) {
		gate_err("PMP not added: overlay failed: %d\n", ret);
		return ret;
	}

	report = of_find_compatible_node(NULL, NULL, gate_soc->report_compat);
	if (report) {
		struct device_node *child;

		pmp = of_parse_phandle(report, "apple,pmp", 0);
		/* The display request entry: DISP is PMP device 7 (0x10 on T6031). */
		for_each_child_of_node(report, child) {
			if (!strcmp(of_node_full_name(child), gate_soc->disp_entry)) {
				disp = child;
				break;
			}
		}
	}
	if (pmp) {
		dart = of_parse_phandle(pmp, "iommus", 0);
		mbox = of_parse_phandle(pmp, "mboxes", 0);
	}

	of_changeset_init(&gate_pmp_cs);
	ret = dart && mbox && disp && disp->phandle ? 0 : -ENODEV;
	/* The DCP driver starts once the PMP acknowledges this request. */
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, dcp, "apple,pmp-report",
						disp->phandle);
	/* T8122 and an opted-in T6030 GPU need this Mac's PMP AGX vote. */
	if (!ret)
		ret = gate_gpu_pmp_link(&gate_pmp_cs, pmp);
	/* The DART and mailbox interrupt parent lives in the base tree. */
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, dart, "interrupt-parent",
						aic->phandle);
	if (!ret)
		ret = of_changeset_add_prop_u32(&gate_pmp_cs, mbox, "interrupt-parent",
						aic->phandle);
	/* The display domains' floor, applied by the PMGR driver when it probes. */
	for (i = PMP_PS_DISP_SYS; i <= PMP_PS_DISP_CPU && !ret; i++)
		ret = gate_set_u32(&gate_pmp_cs, ps[i], "apple,min-state", PMGR_PS_ACTIVE);
	/* Linux would otherwise power these off as unused. */
	for (i = PMP_PS_PMP; i <= PMP_PS_PMS_SRAM && !ret; i++) {
		if (!of_property_read_bool(ps[i], "apple,always-on"))
			ret = of_changeset_add_prop_bool(&gate_pmp_cs, ps[i], "apple,always-on");
	}
	if (!ret && gate_pmp_values) {
		values = gate_pmp_copy_values(&gate_pmp_cs, pmp);
		ret = values < 0 ? values : 0;
	}
	if (!ret)
		ret = of_changeset_apply(&gate_pmp_cs);

	of_node_put(mbox);
	of_node_put(dart);
	of_node_put(pmp);
	of_node_put(disp);
	of_node_put(report);

	if (ret) {
		of_changeset_destroy(&gate_pmp_cs);
		of_overlay_remove(&ovcs_id);
		gate_err("PMP not added: changeset failed: %d\n", ret);
		return ret;
	}
	gate_pmp_ovcs = ovcs_id;

	if (gate_pmp_values)
		gate_info("PMP: this Mac's %d values from %pOF\n", values, gate_pmp_values);
	else
		gate_info("PMP: the built-in J516S values; the boot loader passed none\n");
	gate_info("added the PMP and its display and storage report; minimum power state of %s, %s and %s raised to active; %s and %s kept on\n",
		pmp_ps_labels[PMP_PS_DISP_SYS], pmp_ps_labels[PMP_PS_DISP_FE],
		pmp_ps_labels[PMP_PS_DISP_CPU], pmp_ps_labels[PMP_PS_PMP],
		pmp_ps_labels[PMP_PS_PMS_SRAM]);
	return 0;
}

/*
 * External display processors (dcpext0, dcpext1, ...). The boot loader hands
 * each one over separately: its memory (apple,t6030-dcpext-memory-ready), and
 * its display DART with a scanout child that names it
 * (apple,t6030-dispext-handoff). Nothing here touches a processor's CPU: the
 * display driver starts the firmware when a display is first attached.
 *
 * Each processor ends in one of three states:
 *  - native (the default): the processor, its DART, mailbox, PMP request and
 *    Type-C routes are enabled; its display DART is added to the display
 *    subsystem's iommus, so every framebuffer is mapped for this pipe as on
 *    M1/M2, and a piodma child is added for the DART's PIODMA stream 4. The
 *    processor then joins the main DRM device.
 *  - manual (apple_t6030_display.dcpext_manual=1): the earlier diagnostic
 *    path, with explicit start and a separately mapped scanout buffer.
 *  - refused: everything stays disabled, and so do its Type-C routes, so the
 *    USB-C ports never wait for a display mode switch nobody registers.
 * apple_t6030_display.dcpext=0 refuses every processor;
 * apple_t6030_display.scanout=0 refuses the display DART, which the native
 * path needs.
 */
static bool gate_dcpext_requested __initdata = true;

static int __init gate_dcpext_setup(char *arg)
{
	return kstrtobool(arg, &gate_dcpext_requested);
}
early_param("apple_t6030_display.dcpext", gate_dcpext_setup);

static bool gate_scanout_requested __initdata = true;

static int __init gate_scanout_setup(char *arg)
{
	return kstrtobool(arg, &gate_scanout_requested);
}
early_param("apple_t6030_display.scanout", gate_scanout_setup);

static bool gate_dcpext_manual __initdata;

static int __init gate_dcpext_manual_setup(char *arg)
{
	return kstrtobool(arg, &gate_dcpext_manual);
}
early_param("apple_t6030_display.dcpext_manual", gate_dcpext_manual_setup);

#define GATE_DCPEXT_MAX		4

/* One external display processor and what the gate found for it. */
struct gate_ext {
	struct device_node *dcp, *dart, *mbox;
	struct device_node *cpu, *fe, *sys;
	struct device_node *entry;	/* its PMP request, pmp-dispextN */
	struct device_node *scanout;	/* the boot loader's scanout child */
	struct device_node *disp_dart;	/* the display DART that child names */
	unsigned int index;		/* N of dispextN */
	bool memory;			/* the memory handoff is present */
	bool scanout_ok;		/* the display DART handoff checked out */
};

static void __init gate_ext_put(struct gate_ext *ext)
{
	of_node_put(ext->disp_dart);
	of_node_put(ext->scanout);
	of_node_put(ext->entry);
	of_node_put(ext->sys);
	of_node_put(ext->fe);
	of_node_put(ext->cpu);
	of_node_put(ext->mbox);
	of_node_put(ext->dart);
	of_node_put(ext->dcp);
}

/*
 * True if @ps is the power state labelled "dispext<N>_<suffix>". The first
 * call sets *@index (UINT_MAX on entry); later calls must name the same N.
 */
static bool __init gate_ps_dispext(const struct device_node *ps, const char *suffix,
				   unsigned int *index)
{
	const char *label, *p;
	unsigned int n = 0;

	if (!ps || !of_device_is_compatible(ps, gate_soc->pwrstate_compat) ||
	    of_property_read_string(ps, "label", &label) || !strstarts(label, "dispext"))
		return false;
	p = label + strlen("dispext");
	if (*p < '0' || *p > '9')
		return false;
	while (*p >= '0' && *p <= '9' && n < GATE_DCPEXT_MAX)
		n = n * 10 + *p++ - '0';
	if (n >= GATE_DCPEXT_MAX || *p++ != '_' || strcmp(p, suffix))
		return false;
	if (*index != UINT_MAX && *index != n)
		return false;
	*index = n;
	return true;
}

/* The PMP request entry labelled pmp-dispext<index>. */
static struct device_node *__init gate_pmp_dispext_entry(unsigned int index)
{
	struct device_node *report, *child, *found = NULL;
	char want[16];
	const char *label;

	report = gate_find_one(gate_soc->report_compat);
	if (!report || !of_device_is_available(report)) {
		of_node_put(report);
		return NULL;
	}
	snprintf(want, sizeof(want), "pmp-dispext%u", index);
	for_each_child_of_node(report, child) {
		if (!of_property_read_string(child, "label", &label) && !strcmp(label, want)) {
			found = child;
			break;
		}
	}
	of_node_put(report);
	return found;
}

/*
 * A no-map DT reservation is recorded by the reserved-memory parser, but
 * need not appear in memblock.reserved: no-map marks memblock.memory, and
 * the bootloader's carveouts can be outside the usable-memory list entirely.
 * memblock_is_region_reserved() only checks intersection, not full coverage.
 */
static bool __init gate_scanout_table_reserved(struct device_node *region,
					     const struct resource *table)
{
	struct reserved_mem *rmem;
	struct memblock_region *m;
	u64 size;

	if (!region || !region->parent ||
	    !of_node_name_eq(region->parent, "reserved-memory") ||
	    !of_node_is_root(region->parent->parent) ||
	    !of_device_is_available(region) ||
	    !of_property_read_bool(region, "no-map") ||
	    of_property_read_bool(region, "reusable") || table->end < table->start)
		return false;
	size = resource_size(table);
	if (size < SZ_16K || size > SZ_1M || !IS_ALIGNED(table->start, SZ_16K) ||
	    !IS_ALIGNED(size, SZ_16K))
		return false;
	rmem = of_reserved_mem_lookup(region);
	if (!rmem || rmem->base != table->start || rmem->size != size)
		return false;
	/* Reject even partially allocatable or linearly mapped RAM. */
	for_each_mem_region(m) {
		if (m->size && m->base <= table->end &&
		    (table->start < m->base || table->start - m->base < m->size) &&
		    !memblock_is_nomap(m))
			return false;
	}
	return true;
}

/*
 * Read-only check of the display DART the boot loader handed over with
 * @ext's scanout child, before Linux may attach to it: locked, translating,
 * with empty stream 0 and stream 4 roots in reserved tables, and powered by
 * the processor's own CPU domain.
 */
static int __init gate_dispext_check(struct gate_ext *ext)
{
	struct device_node *dart = NULL, *region = NULL;
	struct resource regs, table, first_table = {};
	struct of_phandle_args spec;
	void __iomem *mmio = NULL;
	void *root;
	u32 state[6], marker;
	u64 phys;
	int i, ret = -EINVAL;

	ext->scanout = of_get_child_by_name(ext->dcp, "scanout");
	if (!ext->scanout || !gate_disabled(ext->scanout) ||
	    !of_device_is_compatible(ext->scanout, "apple,t6030-dispext-scanout") ||
	    of_property_read_u32(ext->scanout, "apple,t6030-dispext-handoff", &marker) ||
	    marker != 1)
		return -ENODEV;
	if (of_count_phandle_with_args(ext->scanout, "iommus", "#iommu-cells") != 1 ||
	    of_parse_phandle_with_args(ext->scanout, "iommus", "#iommu-cells", 0, &spec))
		return -EINVAL;
	dart = spec.np;
	if (spec.args_count != 1 || spec.args[0] != 0 || !gate_disabled(dart) ||
	    !dart->phandle || dart == ext->dart ||
	    !of_device_is_compatible(dart, "apple,t8110-dart") ||
	    of_property_read_u32(dart, "apple,t6030-dispext-handoff", &marker) || marker != 1 ||
	    of_property_count_u32_elems(dart, "apple,inherited-dart-state") != 6 ||
	    of_property_read_u32_array(dart, "apple,inherited-dart-state", state, 6) ||
	    of_count_phandle_with_args(dart, "memory-region", NULL) != 2 ||
	    of_property_match_string(dart, "memory-region-names", "sid0-page-tables") != 0 ||
	    of_property_match_string(dart, "memory-region-names", "sid4-page-tables") != 1 ||
	    state[0] != 0 || state[3] != 4 ||
	    of_address_to_resource(dart, 0, &regs) || resource_size(&regs) != SZ_16K)
		goto out;
	/* The DART belongs to this processor's pipe: it shares its CPU domain. */
	region = gate_ps_parent(dart);
	if (region != ext->cpu)
		goto out;
	of_node_put(region);
	region = NULL;
	mmio = ioremap(regs.start, resource_size(&regs));
	if (!mmio || !(readl(mmio + 0x200) & BIT(0)))
		goto out;
	for (i = 0; i < 2; i++) {
		u32 sid = state[3 * i], tcr = state[3 * i + 1], ttbr = state[3 * i + 2];

		/* A valid SID0 root must not turn a later SID4 refusal into success. */
		ret = -EINVAL;
		if ((tcr & (BIT(0) | BIT(1) | BIT(3))) != BIT(0) || !(ttbr & BIT(0)) ||
		    (ttbr & ~(GENMASK(29, 2) | BIT(0))) ||
		    readl(mmio + 0x1000 + 4 * sid) != tcr ||
		    readl(mmio + 0x1400 + 4 * sid) != ttbr)
			goto out;
		/* T8110: address bits29:2, 16KiB page shift (42-bit output PA). */
		phys = (u64)(ttbr & GENMASK(29, 2)) << 12;
		region = of_parse_phandle(dart, "memory-region", i);
		if (!region ||
		    of_address_to_resource(region, 0, &table) ||
		    !gate_scanout_table_reserved(region, &table) ||
		    phys < table.start || phys > table.end - SZ_16K + 1 ||
		    (i && table.start <= first_table.end && first_table.start <= table.end))
			goto out;
		if (!i)
			first_table = table;
		/* arch_initcall runs after vmalloc initialization. On arm64,
		 * MEMREMAP_WB uses ioremap_cache for no-map/outside-usable RAM;
		 * the validation above excludes an allocatable/direct-map alias.
		 */
		root = memremap(phys, SZ_16K, MEMREMAP_WB);
		if (!root)
			goto out;
		ret = memchr_inv(root, 0, SZ_16K) ? -EBUSY : 0;
		memunmap(root);
		if (ret)
			goto out;
		of_node_put(region);
		region = NULL;
	}
	ext->disp_dart = of_node_get(dart);
	ext->scanout_ok = true;
out:
	if (mmio)
		iounmap(mmio);
	of_node_put(region);
	of_node_put(dart);
	return ret;
}

/* Everything @ext's processor needs, checked without changing anything. */
static int __init gate_ext_resolve(struct gate_ext *ext)
{
	struct device_node *domain;
	u32 ready, id;
	int i;

	ext->index = UINT_MAX;
	ext->memory = !of_property_read_u32(ext->dcp, "apple,t6030-dcpext-memory-ready", &ready) &&
		      ready == 1 && of_property_present(ext->dcp, "memory-region");
	if (!ext->memory)
		return -ENODEV;
	ext->dart = gate_target(ext->dcp, "iommus", "#iommu-cells", 1, 5, "apple,t8110-dart");
	ext->mbox = gate_target(ext->dcp, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	if (!ext->dart || !ext->mbox || !gate_disabled(ext->dcp) ||
	    !gate_disabled(ext->dart) || !gate_disabled(ext->mbox))
		return -EINVAL;

	/* Do not enable a CPU whose power ownership is not fully described. */
	ext->cpu = gate_ps_parent(ext->dcp);
	if (!gate_ps_dispext(ext->cpu, "cpu", &ext->index))
		return -EINVAL;
	ext->fe = gate_ps_parent(ext->cpu);
	if (!gate_ps_dispext(ext->fe, "fe", &ext->index))
		return -EINVAL;
	ext->sys = gate_ps_parent(ext->fe);
	if (!gate_ps_dispext(ext->sys, "sys", &ext->index))
		return -EINVAL;
	for (i = 0; i < 2; i++) {
		domain = gate_ps_parent(i ? ext->mbox : ext->dart);
		of_node_put(domain);
		if (domain != ext->cpu)
			return -EINVAL;
	}

	/* The driver waits for this request's acknowledgement before RUN. */
	ext->entry = gate_pmp_dispext_entry(ext->index);
	if (!ext->entry || !gate_disabled(ext->entry) || !ext->entry->phandle ||
	    !of_device_is_compatible(ext->entry, "apple,t6000-pmp-v2-report-entry") ||
	    of_property_read_u32(ext->entry, "reg", &id) ||
	    !of_property_read_bool(ext->entry, "apple,always-on") ||
	    of_property_read_bool(ext->entry, "apple,no-ack"))
		return -ENOENT;
	return 0;
}

/* Kept for good once applied: the live tree refers to it. */
static struct of_changeset *__init gate_cs_alloc(void)
{
	struct of_changeset *cs = kzalloc_obj(*cs);

	if (cs)
		of_changeset_init(cs);
	return cs;
}

static void __init gate_cs_free(struct of_changeset *cs)
{
	of_changeset_destroy(cs);
	kfree(cs);
}

/* Sets "status" of every Type-C route of @dcp that does not already have it. */
static int __init gate_ext_routes(struct of_changeset *cs, struct device_node *dcp,
				  const char *status)
{
	struct device_node *routes, *route;
	const char *now;
	int ret = 0;

	routes = of_get_child_by_name(dcp, "typec-routes");
	for_each_child_of_node(routes, route) {
		if (!of_property_read_string(route, "status", &now) && !strcmp(now, status))
			continue;
		ret = of_changeset_update_prop_string(cs, route, "status", status);
		if (ret) {
			of_node_put(route);
			break;
		}
	}
	of_node_put(routes);
	return ret;
}

/*
 * @display's iommus with <@dart 0> appended, in @cs. Every entry must have one
 * cell and none may name @dart already. Not __init: the T6041 external chain
 * is powered up from a work item.
 */
static int gate_iommus_append(struct of_changeset *cs, struct device_node *display,
			      struct device_node *dart)
{
	struct of_phandle_args args;
	struct property *old, *prop;
	__be32 *value;
	int i, n;

	old = of_find_property(display, "iommus", NULL);
	n = of_count_phandle_with_args(display, "iommus", "#iommu-cells");
	if (!old || !dart->phandle || n < 1 || old->length != n * 2 * sizeof(__be32))
		return -EINVAL;
	for (i = 0; i < n; i++) {
		if (of_parse_phandle_with_args(display, "iommus", "#iommu-cells", i, &args))
			return -EINVAL;
		of_node_put(args.np);
		if (args.np == dart || args.args_count != 1)
			return -EINVAL;
	}
	prop = kzalloc_obj(*prop);
	value = kmalloc(old->length + 2 * sizeof(*value), GFP_KERNEL);
	if (!prop || !value) {
		kfree(prop);
		kfree(value);
		return -ENOMEM;
	}
	memcpy(value, old->value, old->length);
	value[old->length / sizeof(*value)] = cpu_to_be32(dart->phandle);
	value[old->length / sizeof(*value) + 1] = cpu_to_be32(0);
	prop->name = "iommus";
	prop->length = old->length + 2 * sizeof(*value);
	prop->value = value;
	return of_changeset_update_property(cs, display, prop);
}

/* The display subsystem's iommus with <@dart 0> appended. */
static int __init gate_display_add_dart(struct of_changeset *cs, struct device_node *dart)
{
	struct device_node *display = gate_find_one(gate_soc->display_compat);
	int ret;

	if (!display)
		return -ENODEV;
	ret = gate_iommus_append(cs, display, dart);
	of_node_put(display);
	return ret;
}

/*
 * The processor's PIODMA stream on its display DART, as the internal DCP's
 * piodma child: the driver maps firmware buffers there on request.
 */
static int __init gate_ext_piodma(struct of_changeset *cs, struct gate_ext *ext)
{
	u32 iommus[2] = { ext->disp_dart->phandle, 4 };
	struct device_node *piodma;
	int ret;

	piodma = of_get_child_by_name(ext->dcp, "piodma");
	if (piodma) {
		ret = of_changeset_update_prop_string(cs, piodma, "status", "okay");
		if (!ret) {
			struct property *prop = kzalloc_obj(*prop);
			__be32 *value = kmalloc(sizeof(iommus), GFP_KERNEL);

			if (!prop || !value) {
				kfree(prop);
				kfree(value);
				ret = -ENOMEM;
			} else {
				value[0] = cpu_to_be32(iommus[0]);
				value[1] = cpu_to_be32(iommus[1]);
				prop->name = "iommus";
				prop->length = sizeof(iommus);
				prop->value = value;
				ret = of_changeset_update_property(cs, piodma, prop);
			}
		}
		if (!ret)
			ret = gate_set_u32(cs, piodma, "apple,t6030-dispext-handoff", 1);
		of_node_put(piodma);
		return ret;
	}
	piodma = of_changeset_create_node(cs, ext->dcp, "piodma");
	if (!piodma)
		return -ENOMEM;
	/* The new node is kept for good, like the changeset that attaches it. */
	ret = of_changeset_add_prop_u32_array(cs, piodma, "iommus", iommus, ARRAY_SIZE(iommus));
	if (!ret)
		ret = of_changeset_add_prop_u32(cs, piodma, "apple,t6030-dispext-handoff", 1);
	return ret;
}

/* Enables @ext's processor, in the native or the manual configuration. */
static int __init gate_ext_apply(struct gate_ext *ext, bool native)
{
	struct of_changeset *cs = gate_cs_alloc();
	int ret;

	if (!cs)
		return -ENOMEM;
	/* PMGR applies the floor at probe, before the PMP can manage this CPU. */
	ret = gate_set_u32(cs, ext->cpu, "apple,min-state", PMGR_PS_ACTIVE);
	/* The driver must wait for this request's acknowledgement before RUN. */
	if (!ret)
		ret = gate_set_u32(cs, ext->dcp, "apple,pmp-report", ext->entry->phandle);
	/* No external display is attached in system sleep: the PMP may drop it. */
	if (!ret)
		ret = gate_add_bool(cs, ext->entry, "apple,release-for-sleep");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->entry, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->mbox, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, ext->dcp, "status", "okay");
	/* A processor's routes are usable exactly when the processor is. */
	if (!ret)
		ret = gate_ext_routes(cs, ext->dcp, "okay");
	if (!ret && ext->scanout_ok)
		ret = of_changeset_update_prop_string(cs, ext->disp_dart, "status", "okay");
	if (!ret && native) {
		ret = gate_ext_piodma(cs, ext);
		if (!ret)
			ret = gate_display_add_dart(cs, ext->disp_dart);
	} else if (!ret && ext->scanout_ok) {
		ret = gate_set_u32(cs, ext->scanout, "apple,t6030-scanout-verified", 1);
		if (!ret)
			ret = of_changeset_update_prop_string(cs, ext->scanout, "status", "okay");
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret)
		gate_cs_free(cs);
	return ret;
}

/* Leaves @dcp disabled, and its routes with it. */
static void __init gate_ext_refuse(struct device_node *dcp, const char *why)
{
	struct of_changeset *cs;
	int ret = -ENOMEM;

	cs = gate_cs_alloc();
	if (cs) {
		ret = gate_ext_routes(cs, dcp, "disabled");
		if (!ret)
			ret = of_changeset_apply(cs);
		if (ret)
			gate_cs_free(cs);
	}
	gate_info("%pOF left disabled: %s%s\n", dcp, why,
		  ret ? "; its Type-C routes could not be disabled" : "");
}

/* The firmware the display driver runs external processors with. */
static bool __init gate_ext_firmware_ok(struct device_node *np)
{
	u32 ver[3];

	return of_property_count_u32_elems(np, "apple,firmware-compat") == 3 &&
	       !of_property_read_u32_array(np, "apple,firmware-compat", ver, 3) &&
	       ver[0] == 14 && ver[1] == 7 && ver[2] == 0;
}

static void __init gate_dcpext_one(struct device_node *np, const char *refuse)
{
	struct gate_ext ext = { .dcp = of_node_get(np) };
	bool native = !gate_dcpext_manual;
	int ret;

	if (!gate_disabled(np)) {
		gate_warn("%pOF is not disabled; external display gate skipped\n", np);
		goto put;
	}
	if (refuse) {
		gate_ext_refuse(np, refuse);
		goto put;
	}
	ret = gate_ext_resolve(&ext);
	if (ret == -ENODEV) {
		gate_ext_refuse(np, "the boot loader did not hand it over");
		goto put;
	}
	if (ret) {
		gate_ext_refuse(np, "power, memory or PMP prerequisites are not described");
		goto put;
	}
	/*
	 * The display driver refuses any other firmware at probe. Its Type-C
	 * routes would then never register, and every port that lists them
	 * would wait for them.
	 */
	if (!gate_ext_firmware_ok(np)) {
		gate_ext_refuse(np, "its firmware-compat is not 14.7.0");
		goto put;
	}
	if (gate_scanout_requested) {
		ret = gate_dispext_check(&ext);
		if (ret && ret != -ENODEV)
			gate_warn("%pOF: display DART handoff failed validation (%d)\n", np, ret);
	}
	if (native && !ext.scanout_ok) {
		gate_ext_refuse(np, "no usable display DART handoff");
		goto put;
	}
	ret = gate_ext_apply(&ext, native);
	if (ret) {
		gate_warn("%pOF: changeset failed: %d\n", np, ret);
		gate_ext_refuse(np, "the gate could not enable it");
		goto put;
	}
	if (native)
		gate_info("enabled dcpext%u (%pOF) with PMP DISPEXT%u request, CPU power floor, display DART %pOF and its Type-C routes; started by the display driver on first use\n",
			  ext.index, np, ext.index, ext.disp_dart);
	else
		gate_info("enabled dcpext%u (%pOF) for manual start%s\n", ext.index, np,
			  ext.scanout_ok ? " with the verified scanout DART" : "");
put:
	gate_ext_put(&ext);
}

/* Every external processor; @refuse, if set, says why none is enabled. */
static void __init gate_dcpext(const char *refuse)
{
	struct device_node *np;

	if (!refuse && !gate_dcpext_requested)
		refuse = "disabled on the command line";
	for_each_compatible_node(np, NULL, "apple,t6030-dcpext")
		gate_dcpext_one(np, refuse);
}

/* A one-cell property of @np as text, or "-" if there is none. */
static const char *__init gate_u32_text(const struct device_node *np, const char *name,
					char *buf, size_t len)
{
	u32 val;

	if (!np || of_property_count_u32_elems(np, name) != 1 ||
	    of_property_read_u32(np, name, &val))
		return "-";
	snprintf(buf, len, "%#x", val);
	return buf;
}

/* A firmware version property (3 to 5 cells) of @np as text, or "-". */
static const char *__init gate_version_text(const struct device_node *np, const char *name,
					    char *buf, size_t len)
{
	u32 ver[5];
	int i, n, pos;

	n = np ? of_property_read_variable_u32_array(np, name, ver, 3, ARRAY_SIZE(ver)) : -EINVAL;
	if (n < 3)
		return "-";
	pos = scnprintf(buf, len, "%u", ver[0]);
	for (i = 1; i < n; i++)
		pos += scnprintf(buf + pos, len - pos, ".%u", ver[i]);
	return buf;
}

/*
 * What the boot loader handed over, as found before the gate changes
 * anything: the markers, the DCP firmware identity and memory, this Mac's
 * PMP values and the DCP OS log reservations. Logged on every boot, also
 * when the native display is not requested.
 */
static void __init gate_describe(void)
{
	struct device_node *dcp, *display, *piodma = NULL, *values, *resv, *node;
	char m_dcp[12], m_disp[12], m_pio[12], compat[48], version[48], bdid[12], dvid[12];
	const char *uuid = "-", *pmp_uuid = "-";
	int i, regions = 0, oslog = 0, tunables = 0, reserved = 0;
	struct property *prop;
	struct resource res;

	dcp = gate_find_one(gate_soc->dcp_compat);
	display = gate_find_one(gate_soc->display_compat);
	if (dcp) {
		piodma = of_get_child_by_name(dcp, "piodma");
		of_property_read_string(dcp, "apple,firmware-uuid", &uuid);
		for (i = 0; (node = of_parse_phandle(dcp, "memory-region", i)); i++) {
			regions++;
			if (of_property_read_bool(node, "apple,dcp-os-log"))
				oslog++;
			of_node_put(node);
		}
	}
	gate_info("handoff: %s dcp %s display %s piodma %s; dcp firmware-compat %s version %s uuid %s; %d memory regions, %d OS log\n",
		  gate_soc->marker,
		  gate_u32_text(dcp, gate_soc->marker, m_dcp, sizeof(m_dcp)),
		  gate_u32_text(display, gate_soc->marker, m_disp, sizeof(m_disp)),
		  gate_u32_text(piodma, gate_soc->marker, m_pio, sizeof(m_pio)),
		  gate_version_text(dcp, "apple,firmware-compat", compat, sizeof(compat)),
		  gate_version_text(dcp, "apple,firmware-version", version, sizeof(version)),
		  uuid, regions, oslog);

	values = of_find_node_by_path(gate_soc->pmp_values);
	if (values) {
		of_property_read_string(values, "apple,tunable-uuid", &pmp_uuid);
		for_each_property_of_node(values, prop)
			if (strstarts(prop->name, "apple,tunable-"))
				tunables++;
		gate_info("handoff: PMP values %pOF: board-id %s dram-vendor-id %s tunable-uuid %s, %d tunables (image pinned: %s)\n",
			  values, gate_u32_text(values, "apple,board-id", bdid, sizeof(bdid)),
			  gate_u32_text(values, "apple,dram-vendor-id", dvid, sizeof(dvid)),
			  pmp_uuid, tunables, gate_soc->pmp_uuid);
	} else {
		gate_info("handoff: no PMP values (%s)\n", gate_soc->pmp_values);
	}

	resv = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(resv, node) {
		if (!of_node_name_prefix(node, "dcp-oslog"))
			continue;
		reserved++;
		if (!of_address_to_resource(node, 0, &res))
			gate_info("handoff: %pOFn %pR%s\n", node, &res,
				  of_property_read_bool(node, "no-map") ? " no-map" : "");
	}
	gate_info("handoff: %d dcp-oslog reservations\n", reserved);

	if (gate_soc->fw_uuids) {
		const char *image;

		i = 0;
		of_property_for_each_string(of_chosen, gate_soc->fw_uuids, prop, image) {
			gate_info("handoff: firmware image %s\n", image);
			i++;
		}
		if (!i)
			gate_info("handoff: no /chosen/%s\n", gate_soc->fw_uuids);
	}

	of_node_put(resv);
	of_node_put(values);
	of_node_put(piodma);
	of_node_put(display);
	of_node_put(dcp);
}

/*
 * The T8122 (M3) external display processor, dcpext. Unlike T6030's, the
 * boot loader does not hand it over: the display driver starts its firmware
 * itself, as on M1/M2 (DCP_FIRMWARE_V_14_7_TEMPLATE). The boot loader only
 * reserves that firmware (memory-region and iommu-addresses), sets its
 * firmware version and marks the node apple,t8122-dcpext-firmware-ready =
 * <1>. The node stays disabled in the device tree, so a kernel that cannot
 * drive it never sees it, and neither does a USB-C port controller waiting
 * for its Type-C display route. So do its DARTs, its mailbox and the display
 * crossbars of its routes, and the ATC power states are left as they are:
 * without the marker, nothing of the dcpext's probes or stays powered.
 *
 * Once the internal display was handed over, this enables, in one
 * changeset:
 *  - the dcpext node and its Type-C routes (a processor's routes are usable
 *    exactly when the processor is);
 *  - its DART, its display DART and its mailbox;
 *  - the display crossbars its routes switch (mux-controls), and
 *    apple,always-on on the common power state of each crossbar's ATC, as
 *    the M1/M2 laptops have for DP alt mode across suspend;
 *  - <&dispextN_dart 0> in the display subsystem's iommus, so that every
 *    framebuffer is mapped for the external pipe too, as on M1/M2 (the
 *    display DART is the one the dcpext's piodma child names);
 *  - an active floor on the dcpext's CPU power state, as on T6030, so that
 *    the running PMP cannot power the processor down under its firmware;
 *  - its DISPEXT request, the report entry the dcpext names in
 *    apple,pmp-report, made always on, as on T6030: once the PMP runs it
 *    owns the SoC power and performance states and keeps a block up only if
 *    that block's request bit was set when it started. The display driver
 *    waits for its acknowledgment before it runs the dcpext CPU.
 * The DARTs, the mailbox and the crossbars must be described and still
 * disabled. Without the marker, with other firmware, or if anything does not
 * check out, all of them stay disabled and the ATC power states are left
 * alone. apple_t8122_display.dcpext=0 keeps them disabled too.
 */
static bool gate_t8122_dcpext_requested __initdata = true;

static int __init gate_t8122_dcpext_setup(char *arg)
{
	return kstrtobool(arg, &gate_t8122_dcpext_requested);
}
early_param("apple_t8122_display.dcpext", gate_t8122_dcpext_setup);

/* The most display crossbars a T8122 dcpext's routes switch: ATC0's and ATC1's. */
#define GATE_T8122_XBARS	2

/*
 * The ATC common power state above display crossbar @xbar: the crossbar sits
 * in atcN_usb, one of whose parents is atcN_common.
 */
static struct device_node *__init gate_t8122_atc_common(struct device_node *xbar)
{
	struct device_node *usb, *found = NULL;
	struct of_phandle_args args;
	const char *label;
	char want[32];
	size_t len;
	int i, n;

	usb = gate_ps_parent(xbar);
	if (!usb || of_property_read_string(usb, "label", &label))
		goto out;
	len = strlen(label);
	if (!strstarts(label, "atc") || len < 8 || strcmp(label + len - 4, "_usb"))
		goto out;
	snprintf(want, sizeof(want), "%.*s_common", (int)(len - 4), label);
	n = of_count_phandle_with_args(usb, "power-domains", "#power-domain-cells");
	for (i = 0; i < n && !found; i++) {
		if (of_parse_phandle_with_args(usb, "power-domains", "#power-domain-cells",
					       i, &args))
			break;
		if (!args.args_count && gate_ps_is(args.np, want))
			found = args.np;
		else
			of_node_put(args.np);
	}
out:
	of_node_put(usb);
	return found;
}

/* Is @np among the first @n entries of @nodes? */
static bool __init gate_node_in(struct device_node *const *nodes, int n,
				const struct device_node *np)
{
	while (n--)
		if (nodes[n] == np)
			return true;
	return false;
}

static void __init gate_t8122_dcpext_one(struct device_node *np)
{
	struct device_node *dart = NULL, *mbox = NULL, *piodma = NULL;
	struct device_node *disp_dart = NULL, *ps = NULL, *entry = NULL, *link;
	struct device_node *xbar[GATE_T8122_XBARS] = {};
	struct device_node *common[GATE_T8122_XBARS] = {};
	struct of_phandle_args args;
	struct of_changeset *cs;
	const char *label;
	int ret, i, nxbar = 0;
	u32 ready;

	if (!gate_disabled(np)) {
		gate_warn("%pOF is not disabled; external display gate skipped\n", np);
		return;
	}
	if (of_property_read_u32(np, "apple,t8122-dcpext-firmware-ready", &ready) ||
	    ready != 1) {
		gate_ext_refuse(np, "the boot loader did not reserve its firmware");
		return;
	}
	if (!gate_ext_firmware_ok(np)) {
		gate_ext_refuse(np, "its firmware-compat is not 14.7.0");
		return;
	}
	if (!of_property_present(np, "memory-region")) {
		gate_ext_refuse(np, "it has no firmware memory");
		goto put;
	}
	dart = gate_target(np, "iommus", "#iommu-cells", 1, 5, "apple,t8110-dart");
	mbox = gate_target(np, "mboxes", "#mbox-cells", 0, 0, "apple,asc-mailbox-v4");
	piodma = of_get_child_by_name(np, "piodma");
	if (piodma)
		disp_dart = gate_target(piodma, "iommus", "#iommu-cells", 1, 4,
					"apple,t8110-dart");
	if (!dart || !mbox || !disp_dart || dart == disp_dart || !gate_disabled(dart) ||
	    !gate_disabled(mbox) || !gate_disabled(disp_dart)) {
		gate_ext_refuse(np, "its DARTs or mailbox are not described or not disabled");
		goto put;
	}
	ret = of_count_phandle_with_args(np, "mux-controls", "#mux-control-cells");
	if (ret != -ENOENT && (ret < 0 || ret > GATE_T8122_XBARS)) {
		gate_ext_refuse(np, "its display crossbars are not described");
		goto put;
	}
	for (i = 0; i < ret; i++, nxbar++) {
		if (of_parse_phandle_with_args(np, "mux-controls", "#mux-control-cells",
					       i, &args))
			break;
		xbar[i] = args.np;
		if (args.args_count != 1 || args.args[0] ||
		    !of_device_is_compatible(xbar[i], "apple,t6020-display-crossbar") ||
		    !gate_disabled(xbar[i]))
			break;
		common[i] = gate_t8122_atc_common(xbar[i]);
		if (!common[i])
			break;
	}
	if (i < ret) {
		gate_ext_refuse(np,
				"a display crossbar or its ATC power state is not described, or the crossbar is not disabled");
		goto put;
	}
	ps = of_parse_phandle(np, "power-domains", 0);
	if (!ps || !of_device_is_compatible(ps, gate_soc->pwrstate_compat) ||
	    of_property_read_string(ps, "label", &label) || strcmp(label, "dispext_cpu")) {
		gate_ext_refuse(np, "its power state is not dispext_cpu");
		goto put;
	}

	/*
	 * The PMP DISPEXT request: the first external mode set does not
	 * complete without it, so there is no dcpext without it, as on T6030.
	 * It is the device tree's own entry, which the dcpext names in
	 * apple,pmp-report, and nothing requests it yet.
	 */
	entry = gate_reuse.report ?
		gate_report_entry(gate_reuse.report, gate_soc->dispext_entry) : NULL;
	link = of_parse_phandle(np, "apple,pmp-report", 0);
	of_node_put(link);
	if (!entry || !entry->phandle || link != entry ||
	    of_property_read_bool(entry, "apple,always-on") ||
	    of_property_read_bool(entry, "apple,no-ack")) {
		gate_ext_refuse(np, "its PMP DISPEXT request is not described or not usable");
		goto put;
	}

	cs = gate_cs_alloc();
	if (!cs) {
		gate_ext_refuse(np, "out of memory");
		goto put;
	}
	ret = gate_set_u32(cs, ps, "apple,min-state", PMGR_PS_ACTIVE);
	/*
	 * The driver waits for this request's acknowledgment before RUN. It
	 * is seeded with the display's, by soc-device record, and a missing
	 * acknowledgment must not hold up the panel.
	 */
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,always-on");
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,soc-device-record");
	if (!ret)
		ret = gate_add_bool(cs, entry, "apple,defer-startup-ack");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, disp_dart, "status", "okay");
	if (!ret)
		ret = of_changeset_update_prop_string(cs, mbox, "status", "okay");
	/* DP alt mode keeps the ATC's common power state on, as on M1/M2. */
	for (i = 0; i < nxbar && !ret; i++) {
		if (!gate_node_in(xbar, i, xbar[i]))
			ret = of_changeset_update_prop_string(cs, xbar[i], "status", "okay");
		if (!ret && !gate_node_in(common, i, common[i]) &&
		    !of_property_read_bool(common[i], "apple,always-on"))
			ret = of_changeset_add_prop_bool(cs, common[i], "apple,always-on");
	}
	if (!ret)
		ret = of_changeset_update_prop_string(cs, np, "status", "okay");
	if (!ret)
		ret = gate_ext_routes(cs, np, "okay");
	if (!ret)
		ret = gate_display_add_dart(cs, disp_dart);
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret) {
		gate_cs_free(cs);
		gate_warn("%pOF: changeset failed: %d\n", np, ret);
		gate_ext_refuse(np, "the gate could not enable it");
		goto put;
	}
	gate_info("enabled %pOF with its Type-C routes, DARTs, mailbox, %d display crossbar(s) with ATC common power kept on, display DART %pOF in the display subsystem, CPU power floor and PMP DISPEXT request; started by the display driver\n",
		  np, nxbar, disp_dart);
put:
	for (i = 0; i < GATE_T8122_XBARS; i++) {
		of_node_put(common[i]);
		of_node_put(xbar[i]);
	}
	of_node_put(entry);
	of_node_put(ps);
	of_node_put(disp_dart);
	of_node_put(piodma);
	of_node_put(mbox);
	of_node_put(dart);
}

static void __init gate_t8122_dcpext(bool panel)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "apple,t8122-dcpext") {
		if (!panel)
			gate_ext_refuse(np, "the internal display was not handed over");
		else if (!gate_t8122_dcpext_requested)
			gate_ext_refuse(np, "disabled on the command line");
		/*
		 * Enabled routes make the Type-C ports wait for the display
		 * driver's mode switches: without a display driver that would
		 * take USB from both ports of a J613.
		 */
		else if (!IS_ENABLED(CONFIG_DRM_APPLE))
			gate_ext_refuse(np, "the display driver is not built");
		else if (IS_ENABLED(CONFIG_VIDEO) && video_firmware_drivers_only())
			gate_ext_refuse(np, "nomodeset is on the command line");
		else
			gate_t8122_dcpext_one(np);
	}
}

static void __init gate_run(void)
{
	struct device_node *np[GATE_NR_NODES] = {};
	struct device_node *ps[PMP_PS_NR] = {};
	struct device_node *aic = NULL;
	bool panel = false;
	int i;

	if (gate_soc->describe)
		gate_describe();

	if (gate_soc == &gate_t6031) {
		const char *board = gate_t6031_default_board();

		gate_t6031_requested = gate_t6031_wanted(gate_t6031_switch, !!board);
		if (gate_t6031_switch < 0 && board)
			gate_info("native display on by default on %s; apple_t6031_display.enable=0 keeps the boot framebuffer\n",
				  board);
	}

	if (gate_soc == &gate_j516m) {
		if (!gate_j516m_default_board()) {
			gate_info("unsupported board; native display stays disabled\n");
			return;
		}
		gate_j516m_requested = gate_j516m_wanted(gate_t6034_switch);
		if (gate_t6034_switch < 0)
			gate_info("native display on by default on J516M; apple_t6034_display.enable=0 keeps the boot framebuffer\n");
	}

	if (gate_soc == &gate_t6034) {
		if (!gate_t6034_default_board()) {
			gate_info("unsupported board; native display stays disabled\n");
			return;
		}
		gate_t6034_requested = gate_t6034_wanted(gate_t6034_switch);
		if (gate_t6034_switch < 0)
			gate_info("native display on by default on J514M; apple_t6034_display.enable=0 keeps the boot framebuffer\n");
	}

	if (!*gate_soc->requested) {
		/* Each opt-in SoC's own switch: T6031's is not T6034's */
		int sw = gate_soc == &gate_t6034 || gate_soc == &gate_j516m ?
			 gate_t6034_switch : gate_t6031_switch;

		if (gate_soc->opt_in && sw < 0)
			gate_info("native display not requested (%s is not on the command line), display stays on the boot framebuffer\n",
				  gate_soc->opt_in);
		else
			gate_info("disabled on the command line, display stays on the boot framebuffer\n");
		if (gate_soc->dcpext)
			gate_dcpext("the display gate is disabled");
		return;
	}

	if (!gate_resolve(np) && !gate_pmp_resolve(np, ps, &aic) && !gate_apply(np)) {
		if (gate_pmp_apply(np[GATE_DCP], ps, aic))
			gate_revert();
		else
			panel = true;
	}
	/* External processors need the panel's PMP; without it they stay off. */
	if (gate_soc->dcpext)
		gate_dcpext(panel ? NULL : "the internal display was not handed over");
	if (gate_soc == &gate_t8122)
		gate_t8122_dcpext(panel);
	gate_reuse_put();
	of_node_put(aic);
	for (i = 0; i < PMP_PS_NR; i++)
		of_node_put(ps[i]);
	for (i = 0; i < GATE_NR_NODES; i++)
		of_node_put(np[i]);
}

/*
 * T6041 internal display. The handoff describes the DCP that iBoot left
 * running: the DCP, the display subsystem and the DCP's piodma child carry
 * the marker, the DCP names the 26.6.2 firmware the desktop IOMFB instance is
 * written for, and the DCP, display and piodma streams sit on the expected
 * DARTs. The display nodes are enabled only when all of that checks out and
 * the DCP is running.
 */
#define T6041_DCP_SID		23
#define T6041_DISP_SID		0
#define T6041_PIODMA_SID	16
#define ASC_CPU_CONTROL		0x44
#define ASC_CPU_CONTROL_RUN	BIT(4)

enum {
	T6041_DCP, T6041_DISPLAY, T6041_PIODMA, T6041_DCP_DART,
	T6041_DISP_DART, T6041_DCP_MBOX, T6041_PANEL, T6041_NR_NODES
};

/* Nodes enabled, in this order: the DARTs and the mailbox before their users. */
static const int t6041_enable[] __initconst = { T6041_DCP_DART, T6041_DISP_DART, T6041_DCP_MBOX,
						T6041_DISPLAY, T6041_DCP, T6041_PANEL };

/* Applied, it stays: the live tree refers to it. */
static struct of_changeset t6041_gate;

static void __init gate_t6041_internal(void)
{
	struct device_node *nodes[T6041_NR_NODES] = {};
	struct device_node *piodma_dart = NULL;
	void __iomem *cpu;
	u32 version[3];
	unsigned int i;
	int ret = 0;

	nodes[T6041_DCP] = of_find_node_by_path("dcp");
	if (!nodes[T6041_DCP] || !of_device_is_compatible(nodes[T6041_DCP], gate_soc->dcp_compat) ||
	    !of_property_present(nodes[T6041_DCP], gate_soc->marker))
		goto out;	/* no handoff: the boot framebuffer stays */

	ret = -EINVAL;
	nodes[T6041_DISPLAY] = of_find_node_by_path("disp0");
	nodes[T6041_PIODMA] = of_get_child_by_name(nodes[T6041_DCP], "piodma");
	nodes[T6041_PANEL] = of_get_child_by_name(nodes[T6041_DCP], "panel");
	if (!nodes[T6041_DISPLAY] || !nodes[T6041_PIODMA] || !nodes[T6041_PANEL] ||
	    !of_device_is_compatible(nodes[T6041_DISPLAY], gate_soc->display_compat) ||
	    !gate_marked(nodes[T6041_DCP]) || !gate_marked(nodes[T6041_DISPLAY]) ||
	    !gate_marked(nodes[T6041_PIODMA]))
		goto out;

	/* The desktop IOMFB instance is written for the 26.6.2 firmware only. */
	if (of_property_count_u32_elems(nodes[T6041_DCP], "apple,firmware-compat") != 3 ||
	    of_property_read_u32_array(nodes[T6041_DCP], "apple,firmware-compat", version, 3) ||
	    version[0] != 26 || version[1] != 6 || version[2] != 2)
		goto out;

	nodes[T6041_DCP_DART] = gate_target(nodes[T6041_DCP], "iommus", "#iommu-cells",
					    1, T6041_DCP_SID, "apple,t8110-dart");
	nodes[T6041_DISP_DART] = gate_target(nodes[T6041_DISPLAY], "iommus", "#iommu-cells",
					     1, T6041_DISP_SID, "apple,t8110-dart");
	piodma_dart = gate_target(nodes[T6041_PIODMA], "iommus", "#iommu-cells",
				  1, T6041_PIODMA_SID, "apple,t8110-dart");
	nodes[T6041_DCP_MBOX] = gate_target(nodes[T6041_DCP], "mboxes", "#mbox-cells",
					    0, 0, "apple,asc-mailbox-v4");
	if (!nodes[T6041_DCP_DART] || !nodes[T6041_DISP_DART] || !nodes[T6041_DCP_MBOX] ||
	    piodma_dart != nodes[T6041_DISP_DART] ||
	    nodes[T6041_DCP_DART] == nodes[T6041_DISP_DART])
		goto out;

	for (i = 0; i < ARRAY_SIZE(t6041_enable); i++) {
		if (!gate_disabled(nodes[t6041_enable[i]])) {
			ret = -EBUSY;
			goto out;
		}
	}

	/* The handoff describes a DCP that iBoot left running. */
	cpu = of_iomap(nodes[T6041_DCP], 0);
	if (!cpu) {
		ret = -ENOMEM;
		goto out;
	}
	ret = readl(cpu + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN ? 0 : -EBUSY;
	iounmap(cpu);
	if (ret)
		goto out;

	of_changeset_init(&t6041_gate);
	for (i = 0; i < ARRAY_SIZE(t6041_enable) && !ret; i++)
		ret = of_changeset_update_prop_string(&t6041_gate, nodes[t6041_enable[i]],
						      "status", "okay");
	if (!ret)
		ret = of_changeset_apply(&t6041_gate);
	if (ret)
		of_changeset_destroy(&t6041_gate);
	else
		gate_info("display handoff accepted\n");
out:
	of_node_put(piodma_dart);
	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		of_node_put(nodes[i]);
	if (ret)
		gate_info("display handoff refused: %d; keeping the boot framebuffer\n", ret);
}

/*
 * The external chain is powered up from work items, after the init sections
 * are gone, so its messages keep their own prefix.
 */
static const char *staged_prefix = "";
#define staged_info(fmt, ...) pr_info("%s" fmt, staged_prefix, ##__VA_ARGS__)
#define staged_warn(fmt, ...) pr_warn("%s" fmt, staged_prefix, ##__VA_ARGS__)
#define staged_err(fmt, ...) pr_err("%s" fmt, staged_prefix, ##__VA_ARGS__)

/*
 * External displays (HDMI behind dcpext0, Type-C behind dcpext1). The
 * device tree describes the whole chain disabled; it is enabled here once the
 * boot loader has given each external display processor its firmware memory
 * (the memory-region nodes of the dcpext are available, or they stay disabled
 * placeholders). Before any device exists only the Type-C routes are enabled
 * (the port controllers then wait for dcpext1's mode switches) and the
 * display subsystem is held back. The power-up waits for the PMP firmware,
 * whose acknowledgement the DISPEXT domains need: HDMI chain, Type-C chain,
 * then both processors and the display subsystem together, so the display
 * master is created once with every DART in its iommus.
 *
 * The PMP can bind long after the boot: on an encrypted root its driver
 * (apple-pmp.ko) loads only after the passphrase. Nothing that drives a
 * display can come up before it, as appledrm needs apple-pmp's symbols, so
 * the display subsystem stays held back until the PMP binds, however late,
 * and then the whole chain comes up. Only the Type-C ports cannot wait for
 * it: after STAGED_PMP_WAIT_MS their controllers are let go with the USB
 * muxes only, and they take the display routes after a late power-up (see
 * below). The internal panel alone, for the rest of the boot, is left for a
 * PMP whose probe failed and that is still unbound at the deadline, or
 * STAGED_PMP_WAIT_MS after a failure past it (a deferral looks the same, so
 * it gets that long to bind), and for a power-up that fails.
 */

/*
 * Two stages, each with its own display DART (a DART still disabled in the
 * display subsystem's iommus would make of_iommu drop its whole fwspec): the
 * HDMI chain, then the Type-C chain. Run at the same time the two processors
 * crashed (dcpext0: axi_rd_err), as they never did one after the other.
 * Parents first; the genpd links in the tree do the real ordering.
 */
struct gate_staged {
	const char *dart;		/* display DART of this stage */
	const char *const *nodes;
	unsigned int nr;
};

static const char *const t6041_hdmi_nodes[] = {
	/* ATC3 power, PMP DISPEXT0 request, DISPEXT0, AOP pins, PHY, crossbar, DARTs, mailbox, dcpext0 */
	"/power-management@502800000/power-controller@408",
	"/power-management@508300000/power-controller@e0",
	"/power-management@508300000/power-controller@168",
	"/power-management@508300000/power-controller@b8",
	"/power-management@508300000/power-controller@148",
	"/pmp-report@5003c0000/report@11",
	"/power-management@502800000/power-controller@340",
	"/power-management@502800000/power-controller@348",
	"/pinctrl@510824000",
	"/mailbox@39b148000",
	"/phy@39aa90000",
	"/mux@39b24c000",
	"/iommu@3a2340000",
	"/iommu@3a2300000",
	"/mailbox@3a2e08000",
	"/dcp@3a2e00000/typec-routes/route@0",
	"/dcp@3a2e00000/typec-routes/route@1",
	"/dcp@3a2e00000/typec-routes/route@2",
	"/dcp@3a2e00000",
};

static const char *const t6041_usbc_nodes[] = {
	/* PMP DISPEXT1 request, FAB7_SOC, DISPEXT1, crossbars, DARTs, mailbox, routes, dcpext1 */
	"/pmp-report@5003c0000/report@12",
	"/power-management@502800000/power-controller@488",
	"/power-management@502800000/power-controller@490",
	"/power-management@502800000/power-controller@498",
	"/mux@38324c000",
	"/mux@38b24c000",
	"/mux@39324c000",
	"/iommu@602340000",
	"/iommu@602300000",
	"/mailbox@602e08000",
	"/dcp@602e00000/typec-routes/route@0",
	"/dcp@602e00000/typec-routes/route@1",
	"/dcp@602e00000/typec-routes/route@2",
	"/dcp@602e00000",
};

static bool staged_is_route_path(const char *path)
{
	return strstr(path, "/typec-routes/");
}

/*
 * J614s HDMI: its ADT has no dcpext0 and its DISPEXT0 power states are
 * virtual, so the J616s chain above must never be powered there. HDMI is
 * dcpext2 instead (t6040-j614s-extdisp-hdmi.dtsi): the same ATC3 PHY,
 * crossbar and converter, behind the DISPEXT2 block. As the J616s dcpext0,
 * it also has a route to every Type-C port, so two USB-C displays use
 * dcpext1 and dcpext2.
 */
static const char *const t6041_j614s_hdmi_nodes[] = {
	/* ATC3 power, PMP DISPEXT2 request, FAB7_SOC, DISPEXT2, AOP pins, PHY, crossbar, DARTs, mailbox, routes, dcpext2 */
	"/power-management@502800000/power-controller@408",
	"/power-management@508300000/power-controller@e0",
	"/power-management@508300000/power-controller@168",
	"/power-management@508300000/power-controller@b8",
	"/power-management@508300000/power-controller@148",
	"/pmp-report@5003c0000/report@13",
	"/power-management@502800000/power-controller@488",
	"/power-management@502800000/power-controller@4e0",
	"/power-management@502800000/power-controller@4e8",
	"/pinctrl@510824000",
	"/mailbox@39b148000",
	"/phy@39aa90000",
	"/mux@39b24c000",
	"/iommu@606340000",
	"/iommu@606300000",
	"/mailbox@606e08000",
	"/dcp@606e00000/typec-routes/route@0",
	"/dcp@606e00000/typec-routes/route@1",
	"/dcp@606e00000/typec-routes/route@2",
	"/dcp@606e00000",
};

enum { STAGED_HDMI, STAGED_USBC };

/* The J614s HDMI stage replaces the J616s one in gate_t6041_staged(). */
static struct gate_staged t6041_stages[] = {
	[STAGED_HDMI] = { "/iommu@3a2300000", t6041_hdmi_nodes, ARRAY_SIZE(t6041_hdmi_nodes) },
	[STAGED_USBC] = { "/iommu@602300000", t6041_usbc_nodes, ARRAY_SIZE(t6041_usbc_nodes) },
};

/*
 * The chains this boot enables: each one whose processor the boot loader gave
 * firmware memory. The loader hands the processors over separately (the
 * J614s ADT has no dcpext0: its HDMI processor is dcpext2, so only dcpext1
 * is handed over), so one missing processor must not keep the other off.
 * Set once by gate_t6041_staged().
 */
static unsigned long staged_on;

static bool staged_stage_on(unsigned int i)
{
	return test_bit(i, &staged_on);
}

/*
 * The HDMI processor's Type-C routes (dcpext0's, or J614s's dcpext2's hybrid
 * DP outputs) switch the Type-C crossbars, which are the USB-C chain's.
 * Without that chain they stay disabled and the processor drives HDMI only;
 * the crossbars are enabled for it, as its mux-controls name them.
 */
static const char *const t6041_hdmi_only_muxes[] = {
	"/mux@38324c000",
	"/mux@38b24c000",
	"/mux@39324c000",
};

static bool staged_path_on(unsigned int i, const char *path)
{
	if (!staged_stage_on(i))
		return false;
	return !(i == STAGED_HDMI && staged_is_route_path(path) && !staged_stage_on(STAGED_USBC));
}

/* A display processor (a direct child of the root) and a Type-C display route. */
static bool staged_is_dcp(const char *path)
{
	return !strncmp(path, "/dcp@", 5) && !strchr(path + 5, '/');
}

static bool staged_is_route(const char *path)
{
	return strstr(path, "/typec-routes/");
}

/*
 * The processor's firmware memory must be described: every memory-region of
 * the dcpext node (the stage's last node) must be an available reserved-memory
 * node. A loader that fills the regions from the ADT (m4-loader
 * m4_dcpext_fixup) leaves disabled placeholders when it refuses, and an older
 * loader leaves them all disabled; enabling the processor then would only
 * make its driver fail. /chosen m4,dcpext is that loader's record (0 filled,
 * 2 static memory), printed when the check fails.
 */
static bool staged_firmware_memory(const char *dcp)
{
	struct device_node *np, *region;
	int i, n;
	u32 record[2];

	np = of_find_node_by_path(dcp);
	if (!np)
		return false;
	n = of_count_phandle_with_args(np, "memory-region", NULL);
	for (i = 0; i < n; i++) {
		region = of_parse_phandle(np, "memory-region", i);
		if (region && of_device_is_available(region) &&
		    of_device_is_compatible(region, "apple,asc-mem")) {
			of_node_put(region);
			continue;
		}
		of_node_put(region);
		n = -ENODEV;
		break;
	}
	of_node_put(np);
	if (n > 0)
		return true;
	np = of_find_node_by_path("/chosen");
	if (np && !of_property_read_u32_array(np, "m4,dcpext", record, 2))
		staged_err("%s: no firmware memory (loader record <%u %u>), nothing enabled\n", dcp,
		       record[0], record[1]);
	else
		staged_err("%s: no firmware memory, nothing enabled\n", dcp);
	of_node_put(np);
	return false;
}

/* Every node of @st is there and still disabled, and its processor has firmware memory. */
static bool staged_ready(const struct gate_staged *st)
{
	struct device_node *np;
	unsigned int i;

	if (!staged_firmware_memory(st->nodes[st->nr - 1]))
		return false;
	for (i = 0; i < st->nr; i++) {
		np = of_find_node_by_path(st->nodes[i]);
		if (!np) {
			staged_err("%s: no such node, nothing enabled\n", st->nodes[i]);
			return false;
		}
		if (of_device_is_available(np)) {
			staged_err("%s: already enabled, nothing changed\n", st->nodes[i]);
			of_node_put(np);
			return false;
		}
		of_node_put(np);
	}
	return true;
}

/* Enable @st's nodes and put its display DART into the display subsystem's iommus. */
static int staged_apply(const struct gate_staged *st, struct device_node *display)
{
	struct device_node *np;
	struct of_changeset *cs;
	unsigned int i;
	int ret;

	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	of_changeset_init(cs);

	np = of_find_node_by_path(st->dart);
	ret = np ? gate_iommus_append(cs, display, np) : -ENODEV;
	of_node_put(np);
	for (i = 0; !ret && i < st->nr; i++) {
		if (staged_is_dcp(st->nodes[i]) || staged_is_route(st->nodes[i]))
			continue;
		np = of_find_node_by_path(st->nodes[i]);
		ret = np ? of_changeset_update_prop_string(cs, np, "status", "okay") : -ENODEV;
		of_node_put(np);
	}
	/* The Type-C crossbars the HDMI processor's mux-controls name. */
	for (i = 0; !ret && st == &t6041_stages[STAGED_HDMI] && !staged_stage_on(STAGED_USBC) &&
		    i < ARRAY_SIZE(t6041_hdmi_only_muxes); i++) {
		np = of_find_node_by_path(t6041_hdmi_only_muxes[i]);
		ret = np ? of_changeset_update_prop_string(cs, np, "status", "okay") : -ENODEV;
		of_node_put(np);
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret) {
		of_changeset_destroy(cs);
		kfree(cs);
	}
	/* Applied, it stays: the live tree refers to it. */
	return ret;
}

#define STAGED_PMP_WAIT_MS 20000

/* A tether boot reserves ATC0 for the debug link. Its DP PHY is disabled,
 * so exposing a route@0 would make its DCP (dcpext1, or the dcpext0 hybrid)
 * defer on that missing supplier and take the usable ATC1/2 display routes
 * (and dcpext0's HDMI) down with it.
 */
static bool staged_route_phy_available(struct device_node *route)
{
	struct device_node *routes, *dcp;
	struct of_phandle_args phy;
	char name[16];
	u32 index;
	int match;
	bool available = false;

	if (of_property_read_u32(route, "reg", &index))
		return false;
	routes = of_get_parent(route);
	dcp = routes ? of_get_parent(routes) : NULL;
	of_node_put(routes);
	if (!dcp)
		return false;
	/* The fabric takes a route's PHY by name ("typecN"), not by position. */
	snprintf(name, sizeof(name), "typec%u", index);
	match = of_property_match_string(dcp, "phy-names", name);
	if (match >= 0)
		index = match;
	if (!of_parse_phandle_with_args(dcp, "phys", "#phy-cells", index, &phy)) {
		available = of_device_is_available(phy.np);
		of_node_put(phy.np);
	}
	of_node_put(dcp);
	return available;
}

/* Set the status of the processors (externals) or the routes, and the display subsystem, in one changeset. */
static int staged_set(struct device_node *display, bool dcps, const char *dcp_status, bool routes,
		    const char *route_status, const char *display_status)
{
	struct device_node *np;
	struct of_changeset *cs;
	unsigned int i, j;
	int ret = 0;

	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	of_changeset_init(cs);
	for (i = 0; !ret && i < ARRAY_SIZE(t6041_stages); i++)
		for (j = 0; !ret && j < t6041_stages[i].nr; j++) {
			const char *path = t6041_stages[i].nodes[j];

			if (!(dcps && staged_is_dcp(path)) && !(routes && staged_is_route(path)))
				continue;
			if (!staged_path_on(i, path))
				continue;
			np = of_find_node_by_path(path);
			if (np && staged_is_route(path) && route_status &&
			    !strcmp(route_status, "okay") && !staged_route_phy_available(np)) {
				staged_info("%s remains disabled: DP PHY unavailable\n", path);
				of_node_put(np);
				continue;
			}
			ret = np ? of_changeset_update_prop_string(cs, np, "status",
								   staged_is_dcp(path) ? dcp_status :
								   route_status) : -ENODEV;
			of_node_put(np);
		}
	if (!ret && display_status)
		ret = of_changeset_update_prop_string(cs, display, "status", display_status);
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret) {
		of_changeset_destroy(cs);
		kfree(cs);
	}
	return ret;
}

/*
 * The Type-C core passes over a mode switch inside a disabled node, so a port
 * controller that probes before dcpext1 is enabled only takes the USB muxes.
 * Taking the display ones afterwards means probing it again, and that tears
 * its USB host down while the devices behind it are being enumerated. Instead,
 * each port controller wired to a route waits for the processor that registers
 * the route: a supplier link from the controller to that processor makes it
 * probe once, when it can take every mux it is wired to.
 *
 * The device link that fw_devlink makes of it is a managed one, and has to
 * be. A port controller holds one mux handle for all of its muxes, and once
 * one of them is unregistered every later transaction on that handle fails
 * with -ENODEV, the USB and safe states included. So a processor that is
 * unbound takes its port controllers with it, and they probe again when it
 * is bound again, instead of staying bound to ports they cannot switch.
 *
 * The deferred probe timeout cannot reorder this. Until the processor has a
 * device the wait is a fwnode link, which the timeout leaves alone. The
 * device is created with its node enabled, so a controller that the timeout
 * lets go before the processor binds finds the routes not yet registered and
 * defers in the Type-C core, which never gives up on a mux. A processor that
 * gives up on its routes disables them (dcp_typec_routes_disable()), and its
 * controllers then probe without them.
 *
 * The wait ends at STAGED_PMP_WAIT_MS, PMP or not: USB must work while the
 * PMP driver is still on an encrypted root, behind its passphrase prompt.
 * The routes are disabled again and the controllers probe with the USB
 * muxes only. A PMP that binds later enables the routes again, together with
 * the processors. Once every processor that still has an enabled route is
 * bound, so that every route mode switch is registered, staged_take_routes()
 * gives each released controller that is still wired to an enabled route
 * the managed link it would have had, and probes it again, and only those.
 * That takes their USB hosts down once, after the PMP has bound, and the
 * devices behind them enumerate again. A controller whose routes its
 * processor gave up on keeps its USB-only probe.
 */

/* A port controller and the processor whose routes it is wired to. */
struct staged_port_ctrl {
	struct fwnode_handle *ctrl, *dcp;
};

/* Guards the port controller lists, staged_routes_off and staged_late's writers. */
static DEFINE_MUTEX(staged_lock);
/* Held by a fwnode link until the processors are bound or the deadline. */
static struct staged_port_ctrl staged_held[ARRAY_SIZE(t6041_usbc_nodes)];
static unsigned int staged_nr_held;
/* Let go at the deadline with the USB muxes only, for a late power-up. */
static struct staged_port_ctrl staged_released[ARRAY_SIZE(t6041_usbc_nodes)];
static unsigned int staged_nr_released;
static bool staged_routes_off;	/* the deadline disabled the routes */
static bool staged_late;	/* a late power-up waits for the processors */

/* The port controller wired to a route (the parent of the connector at its far end). */
static struct device_node *staged_route_port_ctrl(struct device_node *route)
{
	struct device_node *ep, *connector, *ctrl;

	ep = of_graph_get_next_endpoint(route, NULL);
	connector = ep ? of_graph_get_remote_port_parent(ep) : NULL;
	ctrl = connector ? of_get_parent(connector) : NULL;
	of_node_put(connector);
	of_node_put(ep);
	return ctrl;
}

static void __init staged_hold_port_controllers(void)
{
	struct device_node *route, *routes, *dcp, *ctrl;
	unsigned int i, k;

	/* Only dcpext1's routes are held for; without its chain nothing waits. */
	if (!staged_stage_on(STAGED_USBC))
		return;

	for (i = 0; i < ARRAY_SIZE(t6041_usbc_nodes); i++) {
		if (!staged_is_route(t6041_usbc_nodes[i]))
			continue;
		route = of_find_node_by_path(t6041_usbc_nodes[i]);
		if (!route)
			continue;
		routes = of_get_parent(route);
		dcp = routes ? of_get_parent(routes) : NULL;
		ctrl = staged_route_port_ctrl(route);
		of_node_put(routes);
		of_node_put(route);
		for (k = 0; ctrl && k < staged_nr_held; k++)
			if (staged_held[k].ctrl == of_fwnode_handle(ctrl))
				break;
		/*
		 * A disabled port controller never gets a device: a link from it
		 * would land on its bound bus controller as a sync-state-only
		 * link, which the device core refuses.
		 */
		if (ctrl && dcp && k == staged_nr_held && of_device_is_available(ctrl) &&
		    !fwnode_link_add(of_fwnode_handle(ctrl), of_fwnode_handle(dcp), 0)) {
			staged_held[k].ctrl = fwnode_handle_get(of_fwnode_handle(ctrl));
			staged_held[k].dcp = fwnode_handle_get(of_fwnode_handle(dcp));
			staged_nr_held++;
		}
		of_node_put(dcp);
		of_node_put(ctrl);
	}
}

static void staged_forget(struct staged_port_ctrl *list, unsigned int *nr)
{
	while (*nr) {
		struct staged_port_ctrl *pc = &list[--*nr];

		fwnode_handle_put(pc->dcp);
		fwnode_handle_put(pc->ctrl);
	}
}

/*
 * The processors are not on (yet): drop the wait, so that the port
 * controllers probe now, with the USB muxes only, and keep them for a late
 * power-up. By this time the controllers' other suppliers are device links;
 * what is left are links to nodes without a device.
 */
static void staged_release_port_controllers(void)
{
	struct device *dev;

	lockdep_assert_held(&staged_lock);
	while (staged_nr_held) {
		struct staged_port_ctrl *pc = &staged_held[--staged_nr_held];

		fwnode_links_purge(pc->ctrl);
		dev = get_device(READ_ONCE(pc->ctrl->dev));
		if (dev && !READ_ONCE(dev->driver) && device_attach(dev) < 0)
			dev_err(dev, "probe without the display routes failed\n");
		put_device(dev);
		staged_released[staged_nr_released++] = *pc;
	}
}

/* @ctrl is wired to a route of dcpext1 that is still enabled. */
static bool staged_ctrl_routed(struct fwnode_handle *ctrl)
{
	struct device_node *route, *np;
	bool routed = false;
	unsigned int i;

	for (i = 0; !routed && i < ARRAY_SIZE(t6041_usbc_nodes); i++) {
		if (!staged_is_route(t6041_usbc_nodes[i]))
			continue;
		route = of_find_node_by_path(t6041_usbc_nodes[i]);
		np = route && of_device_is_available(route) ? staged_route_port_ctrl(route) : NULL;
		routed = np && of_fwnode_handle(np) == ctrl;
		of_node_put(np);
		of_node_put(route);
	}
	return routed;
}

static struct device_node *staged_dcps[ARRAY_SIZE(t6041_stages)];

/* Every processor with an enabled route is bound: all route mode switches are registered. */
static bool staged_processors_bound(void)
{
	struct device_node *routes, *route;
	struct device *dev;
	bool bound = true;
	unsigned int i;

	for (i = 0; bound && i < ARRAY_SIZE(staged_dcps); i++) {
		routes = of_get_child_by_name(staged_dcps[i], "typec-routes");
		route = routes ? of_get_next_available_child(routes, NULL) : NULL;
		dev = route ? READ_ONCE(staged_dcps[i]->fwnode.dev) : NULL;
		bound = !route || (dev && device_is_bound(dev));
		of_node_put(route);
		of_node_put(routes);
	}
	return bound;
}

/* After a late power-up: the released port controllers take the display routes. */
static void staged_take_routes(struct work_struct *work)
{
	struct device *ctrl, *dcp;
	unsigned int n = 0;

	mutex_lock(&staged_lock);
	if (!staged_late || !staged_processors_bound()) {
		mutex_unlock(&staged_lock);
		return;
	}
	WRITE_ONCE(staged_late, false);
	while (staged_nr_released) {
		struct staged_port_ctrl *pc = &staged_released[--staged_nr_released];

		ctrl = get_device(READ_ONCE(pc->ctrl->dev));
		dcp = get_device(READ_ONCE(pc->dcp->dev));
		if (ctrl && dcp && staged_ctrl_routed(pc->ctrl)) {
			/* What fw_devlink makes of a held controller's fwnode link. */
			if (!device_link_add(ctrl, dcp, DL_FLAG_AUTOPROBE_CONSUMER))
				dev_warn(ctrl, "no link to %s, keeping the USB muxes only\n",
					 dev_name(dcp));
			else if (device_reprobe(ctrl))
				dev_err(ctrl, "probe with the display routes failed\n");
			else
				n++;
		}
		put_device(dcp);
		put_device(ctrl);
		fwnode_handle_put(pc->dcp);
		fwnode_handle_put(pc->ctrl);
	}
	mutex_unlock(&staged_lock);
	if (n)
		staged_info("%u Type-C port controllers probed again with the display routes\n", n);
}
static DECLARE_WORK(staged_take_routes_work, staged_take_routes);

/*
 * Where the power-up is. Only staged_move() changes it, so each step is
 * taken once: the first of the PMP's bind, its failure and, before either,
 * the deadline wins.
 */
enum staged_state {
	STAGED_WAITING,		/* routes on, port controllers held, display subsystem off */
	STAGED_RELEASED,	/* past the deadline: routes off, port controllers on USB only */
	STAGED_STARTED,		/* the PMP is bound: the power-up runs or has run */
	STAGED_FALLBACK,	/* internal panel only, for the rest of the boot */
};

enum staged_event {
	STAGED_PMP_BOUND,
	STAGED_DEADLINE,
	STAGED_PMP_FAILED,
};

static enum staged_state staged_next(enum staged_state state, enum staged_event event)
{
	if (state != STAGED_WAITING && state != STAGED_RELEASED)
		return state;
	switch (event) {
	case STAGED_PMP_BOUND:
		return STAGED_STARTED;
	case STAGED_DEADLINE:
		return STAGED_RELEASED;
	case STAGED_PMP_FAILED:
		return STAGED_FALLBACK;
	}
	return state;
}

static struct device_node *staged_display;
static struct device_node *staged_pmp;
static atomic_t staged_state = ATOMIC_INIT(STAGED_WAITING);
static atomic_t staged_pmp_tried = ATOMIC_INIT(0);

/* Take the step @event leads to; false if it leads nowhere from here. */
static bool staged_move(enum staged_event event)
{
	int from = atomic_read(&staged_state);
	int to;

	do {
		to = staged_next(from, event);
		if (to == from)
			return false;
	} while (!atomic_try_cmpxchg(&staged_state, &from, to));
	return true;
}

/* Internal panel only: the routes go again, so the port controllers probe without mode switches. */
static void staged_fallback(const char *why)
{
	int ret = staged_set(staged_display, false, NULL, true, "disabled", "okay");

	atomic_set(&staged_state, STAGED_FALLBACK);
	staged_err("%s: external displays stay off, internal panel only (%d)\n", why, ret);
	mutex_lock(&staged_lock);
	WRITE_ONCE(staged_late, false);
	staged_release_port_controllers();
	staged_forget(staged_released, &staged_nr_released);
	mutex_unlock(&staged_lock);
}

/*
 * A processor that finds its crossbar or DP PHY without a driver is given up
 * for good (the deferred probe timeout is 0 here), and those drivers are
 * modules that load only once the stage has created their devices. Wait until
 * every such device of the enabled infrastructure has its driver. Returns the
 * first path still without one, or NULL.
 */
static const char *staged_unbound_infra(void)
{
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(t6041_stages); i++)
		for (j = 0; staged_stage_on(i) && j < t6041_stages[i].nr; j++) {
			const char *path = t6041_stages[i].nodes[j];
			struct device_node *np;
			struct device *dev;

			/* Power domains are the genpd core's to wait for; the modules are the point here. */
			if (staged_is_dcp(path) || staged_is_route(path) ||
			    !strncmp(path, "/power-management@", 18))
				continue;
			np = of_find_node_by_path(path);
			dev = np ? READ_ONCE(np->fwnode.dev) : NULL;
			of_node_put(np);
			if (dev && !READ_ONCE(dev->driver))
				return path;
		}
	return NULL;
}

#define STAGED_DRIVER_WAIT_MS 10000

static void staged_wait_for_drivers(void)
{
	const char *path = NULL;
	unsigned int tries;

	for (tries = 0; tries < STAGED_DRIVER_WAIT_MS / 100; tries++) {
		path = staged_unbound_infra();
		if (!path)
			return;
		msleep(100);
	}
	staged_warn("%s has no driver after " __stringify(STAGED_DRIVER_WAIT_MS) " ms, enabling the processors anyway\n",
		path);
}

static void staged_timeout(struct work_struct *work);
static DECLARE_DELAYED_WORK(staged_timeout_work, staged_timeout);

static void staged_power_up(struct work_struct *work)
{
	unsigned int i;
	bool late;
	int ret = 0;

	/* Let a deadline that is letting the port controllers go finish first. */
	cancel_delayed_work_sync(&staged_timeout_work);
	mutex_lock(&staged_lock);
	late = staged_routes_off || staged_nr_released;
	WRITE_ONCE(staged_late, late);
	mutex_unlock(&staged_lock);

	for (i = 0; !ret && i < ARRAY_SIZE(t6041_stages); i++) {
		if (!staged_stage_on(i))
			continue;
		ret = staged_apply(&t6041_stages[i], staged_display);
		if (!ret)
			staged_info("stage %u enabled\n", i);
	}
	if (!ret)
		staged_wait_for_drivers();
	/*
	 * Both processors and the display subsystem together: the master binds
	 * once, with all DCPs. Past the deadline the routes come back in the
	 * same changeset, so each processor finds them when it probes.
	 */
	if (!ret)
		ret = staged_set(staged_display, true, "okay", late, "okay", "okay");
	if (ret) {
		staged_fallback("could not enable the external displays");
		return;
	}
	staged_info("processors and display subsystem enabled\n");
	mutex_lock(&staged_lock);
	staged_routes_off = false;
	/* Held ones probe through their supplier links. */
	staged_forget(staged_held, &staged_nr_held);
	mutex_unlock(&staged_lock);
	if (late)
		queue_work(system_long_wq, &staged_take_routes_work);
}
static DECLARE_WORK(staged_power_up_work, staged_power_up);

/* The PMP's probe failed, as a deferral's does too, and it is neither bound nor probing now. */
static bool staged_pmp_failed(void)
{
	struct platform_device *pdev;
	bool failed;

	if (!atomic_read(&staged_pmp_tried))
		return false;
	pdev = of_find_device_by_node(staged_pmp);
	failed = pdev && !READ_ONCE(pdev->dev.driver);
	platform_device_put(pdev);
	return failed;
}

static void staged_timeout(struct work_struct *work)
{
	int ret;

	if (staged_pmp_failed() && staged_move(STAGED_PMP_FAILED)) {
		staged_fallback("the PMP did not bind");
		return;
	}
	if (!staged_move(STAGED_DEADLINE))
		return;
	mutex_lock(&staged_lock);
	ret = staged_set(staged_display, false, NULL, true, "disabled", NULL);
	staged_routes_off = !ret;
	staged_release_port_controllers();
	mutex_unlock(&staged_lock);
	staged_info("no PMP within " __stringify(STAGED_PMP_WAIT_MS) " ms: Type-C ports on USB only, external displays wait for it (%d)\n",
		    ret);
}

static bool staged_is_processor(const struct device_node *np)
{
	unsigned int i;

	for (i = 0; np && i < ARRAY_SIZE(staged_dcps); i++)
		if (np == staged_dcps[i])
			return true;
	return false;
}

/*
 * The PMP's driver is bound: DISPEXT can be acknowledged now, before or after
 * the deadline. A failed probe of the PMP (a deferral too) past the deadline
 * gives it STAGED_PMP_WAIT_MS more before the internal panel fallback; before
 * the deadline, the deadline decides. After a late power-up, each processor's
 * probe may be the last the port controllers wait for.
 */
static int staged_bus_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct device *dev = data;

	if (action != BUS_NOTIFY_BOUND_DRIVER && action != BUS_NOTIFY_DRIVER_NOT_BOUND)
		return NOTIFY_DONE;
	if (dev->of_node == staged_pmp) {
		if (action == BUS_NOTIFY_BOUND_DRIVER) {
			if (!staged_move(STAGED_PMP_BOUND))
				return NOTIFY_DONE;
			cancel_delayed_work(&staged_timeout_work);
			queue_work(system_long_wq, &staged_power_up_work);
			return NOTIFY_OK;
		}
		atomic_set(&staged_pmp_tried, 1);
		if (atomic_read(&staged_state) == STAGED_RELEASED)
			schedule_delayed_work(&staged_timeout_work,
					      msecs_to_jiffies(STAGED_PMP_WAIT_MS));
		return NOTIFY_DONE;
	}
	if (READ_ONCE(staged_late) && staged_is_processor(dev->of_node))
		queue_work(system_long_wq, &staged_take_routes_work);
	return NOTIFY_DONE;
}
static struct notifier_block staged_nb = { .notifier_call = staged_bus_notify };

static void __init gate_t6041_staged(void)
{
	unsigned int i;
	int ret;

	staged_prefix = gate_soc->prefix;

	if (of_machine_is_compatible("apple,j614s"))
		t6041_stages[STAGED_HDMI] = (struct gate_staged){
			"/iommu@606300000", t6041_j614s_hdmi_nodes,
			ARRAY_SIZE(t6041_j614s_hdmi_nodes) };

	/*
	 * Each chain must be all there and disabled, with firmware memory,
	 * before anything of it is changed; a chain that is not stays as it is
	 * and does not keep the other one off.
	 */
	for (i = 0; i < ARRAY_SIZE(t6041_stages); i++)
		if (staged_ready(&t6041_stages[i]))
			__set_bit(i, &staged_on);
	if (!staged_on)
		return;
	if (!staged_stage_on(STAGED_USBC))
		staged_info("Type-C chain not handed over: HDMI only, dcpext0 Type-C routes stay disabled\n");
	else if (!staged_stage_on(STAGED_HDMI))
		staged_info("HDMI chain not handed over: Type-C displays only\n");
	staged_display = of_find_node_by_path("/display-subsystem");
	staged_pmp = of_find_node_by_path("/pmp@500700000");
	if (!staged_display || !staged_pmp || !of_device_is_available(staged_display) ||
	    !of_device_is_available(staged_pmp)) {
		staged_err("no display subsystem or PMP, nothing enabled\n");
		return;
	}
	/* staged_ready() found them. */
	for (i = 0; i < ARRAY_SIZE(t6041_stages); i++) {
		const struct gate_staged *st = &t6041_stages[i];

		if (staged_stage_on(i))
			staged_dcps[i] = of_find_node_by_path(st->nodes[st->nr - 1]);
	}

	/* No device exists yet: routes on, display subsystem held back until the power-up. */
	ret = staged_set(staged_display, false, NULL, true, "okay", "disabled");
	if (ret) {
		staged_err("could not prepare the tree: %d, nothing enabled\n", ret);
		return;
	}
	staged_hold_port_controllers();
	ret = bus_register_notifier(&platform_bus_type, &staged_nb);
	if (ret) {
		staged_fallback("no bus notifier");
		return;
	}
	schedule_delayed_work(&staged_timeout_work, msecs_to_jiffies(STAGED_PMP_WAIT_MS));
	staged_info("routes enabled, waiting for the PMP\n");
}

static void __init gate_t6041_run(void)
{
	if (IS_ENABLED(CONFIG_VIDEO) && video_firmware_drivers_only())
		return;
	gate_t6041_internal();
	gate_t6041_staged();
}

/*
 * J616c's external chains. No PMP, so no DISPEXT power domain is described and
 * nothing is requested or waited for: the domains are on as iBoot left them.
 */
static const char *const t6041_max_hdmi_nodes[] = {
	/* ATC3 power, AOP pins, PHY, crossbar, DARTs, mailbox, dcpext0 */
	"/power-management@502800000/power-controller@408",
	"/power-management@508300000/power-controller@e0",
	"/power-management@508300000/power-controller@168",
	"/power-management@508300000/power-controller@b8",
	"/power-management@508300000/power-controller@148",
	"/pinctrl@510824000",
	"/mailbox@39b148000",
	"/phy@39aa90000",
	"/mux@39b24c000",
	"/iommu@3a2340000",
	"/iommu@3a2300000",
	"/mailbox@3a2e08000",
	"/dcp@3a2e00000",
};

static const char *const t6041_max_usbc_nodes[] = {
	/*
	 * ATC0-2 crossbars, DARTs, mailbox, a route per port, dcpext1, as the
	 * M4 Pro's. The ATC mailboxes are the PHYs', on for USB3 whatever the
	 * display does.
	 */
	"/mux@38324c000",
	"/mux@38b24c000",
	"/mux@39324c000",
	"/iommu@602340000",
	"/iommu@602300000",
	"/mailbox@602e08000",
	"/dcp@602e00000/typec-routes/route@0",
	"/dcp@602e00000/typec-routes/route@1",
	"/dcp@602e00000/typec-routes/route@2",
	"/dcp@602e00000",
};

/*
 * The second USB-C processor of the J614c/J616c: DARTs, mailbox, a route per
 * port, dcpext2. It shares the ATC0-2 crossbars with dcpext1, whose chain
 * enables them, so they are not part of this one.
 */
static const char *const t6041_max_usbc2_nodes[] = {
	"/iommu@606340000",
	"/iommu@606300000",
	"/mailbox@606e08000",
	"/dcp@606e00000/typec-routes/route@0",
	"/dcp@606e00000/typec-routes/route@1",
	"/dcp@606e00000/typec-routes/route@2",
	"/dcp@606e00000",
};

static const struct gate_staged t6041_max_stages[] = {
	{ "/iommu@3a2300000", t6041_max_hdmi_nodes, ARRAY_SIZE(t6041_max_hdmi_nodes) },
	{ "/iommu@602300000", t6041_max_usbc_nodes, ARRAY_SIZE(t6041_max_usbc_nodes) },
	{ "/iommu@606300000", t6041_max_usbc2_nodes, ARRAY_SIZE(t6041_max_usbc2_nodes) },
};

/*
 * The J575c's USB-C chain: the crossbars of its four back Type-C ports
 * (ATC0-3), DARTs, mailbox, a route per port, dcpext1. The ports are also
 * USB3 ports, so the device tree enables the ATC PHYs' coprocessor
 * mailboxes, as the M4 Pro boards do: they are not part of the chain here.
 */
static const char *const t6041_j575c_usbc_nodes[] = {
	"/mux@38324c000",
	"/mux@38b24c000",
	"/mux@39324c000",
	"/mux@39b24c000",
	"/iommu@602340000",
	"/iommu@602300000",
	"/mailbox@602e08000",
	"/dcp@602e00000/typec-routes/route@0",
	"/dcp@602e00000/typec-routes/route@1",
	"/dcp@602e00000/typec-routes/route@2",
	"/dcp@602e00000/typec-routes/route@3",
	"/dcp@602e00000",
};

static const struct gate_staged t6041_j575c_usbc = {
	"/iommu@602300000", t6041_j575c_usbc_nodes, ARRAY_SIZE(t6041_j575c_usbc_nodes),
};

/*
 * The J575c's second USB-C processor, as the J614c/J616c's, with a route to
 * the fourth port too. It shares the crossbars with dcpext1's chain.
 */
static const char *const t6041_j575c_usbc2_nodes[] = {
	"/iommu@606340000",
	"/iommu@606300000",
	"/mailbox@606e08000",
	"/dcp@606e00000/typec-routes/route@0",
	"/dcp@606e00000/typec-routes/route@1",
	"/dcp@606e00000/typec-routes/route@2",
	"/dcp@606e00000/typec-routes/route@3",
	"/dcp@606e00000",
};

static const struct gate_staged t6041_j575c_usbc2 = {
	"/iommu@606300000", t6041_j575c_usbc2_nodes, ARRAY_SIZE(t6041_j575c_usbc2_nodes),
};

/*
 * The J575c's HDMI chain. Its ATC3 is a USB-C port, and its HDMI port runs
 * through the LPDPTX PHY instead (t6041-j575c-hdmi.dtsi): that PHY's power,
 * the AOP pins (the converter's hotplug), the PHY, then dcpext0's chain.
 */
static const char *const t6041_j575c_hdmi_nodes[] = {
	"/power-management@502280000/power-controller@360",
	"/pinctrl@510824000",
	"/phy@51c200000",
	"/iommu@3a2340000",
	"/iommu@3a2300000",
	"/mailbox@3a2e08000",
	"/dcp@3a2e00000",
};

static const struct gate_staged t6041_j575c_hdmi = {
	"/iommu@3a2300000", t6041_j575c_hdmi_nodes, ARRAY_SIZE(t6041_j575c_hdmi_nodes),
};

static const struct gate_staged *t6041_max_stage(unsigned int i)
{
	if (i == 0 && of_machine_is_compatible("apple,j575c"))
		return &t6041_j575c_hdmi;
	if (i == 1 && of_machine_is_compatible("apple,j575c"))
		return &t6041_j575c_usbc;
	if (i == 2 && of_machine_is_compatible("apple,j575c"))
		return &t6041_j575c_usbc2;
	return &t6041_max_stages[i];
}

/* Applied, they stay: the live tree refers to them. Only the KUnit test reverts them. */
static struct of_changeset t6041_max_cs[ARRAY_SIZE(t6041_max_stages)];

/* Enable every node of stage @i and put its display DART into the display subsystem's iommus. */
static int __init t6041_max_apply(unsigned int i, struct device_node *display)
{
	const struct gate_staged *st = t6041_max_stage(i);
	struct of_changeset *cs = &t6041_max_cs[i];
	struct device_node *np;
	unsigned int n;
	int ret;

	of_changeset_init(cs);
	np = of_find_node_by_path(st->dart);
	ret = np ? gate_iommus_append(cs, display, np) : -ENODEV;
	of_node_put(np);
	for (n = 0; !ret && n < st->nr; n++) {
		np = of_find_node_by_path(st->nodes[n]);
		ret = np ? of_changeset_update_prop_string(cs, np, "status", "okay") : -ENODEV;
		of_node_put(np);
	}
	if (!ret)
		ret = of_changeset_apply(cs);
	if (ret)
		of_changeset_destroy(cs);
	return ret;
}

/*
 * The internal display first, as on the M4 Pro: an accepted handoff enables
 * the display subsystem with the DCP. Then each external chain is enabled in
 * one changeset, once its processor has firmware memory. A chain that has
 * none, or is not all there and disabled (a boot loader that enabled it
 * itself), stays as it is, and the other chain does not depend on it. The
 * chains need the display subsystem enabled, by the handoff or by the boot
 * loader, and only add their DARTs to it: its status is not written here.
 */
static void __init gate_t6041_max_run(void)
{
	struct device_node *display;
	unsigned int i;
	int ret;

	if (IS_ENABLED(CONFIG_VIDEO) && video_firmware_drivers_only())
		return;

	gate_t6041_internal();
	staged_prefix = gate_soc->prefix;
	display = of_find_node_by_path("/display-subsystem");
	if (!display) {
		staged_err("no display subsystem, nothing enabled\n");
		return;
	}
	if (!of_device_is_available(display)) {
		staged_info("display subsystem not enabled, external displays left off\n");
		of_node_put(display);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(t6041_max_stages); i++) {
		const struct gate_staged *st = t6041_max_stage(i);
		const char *dcp = st->nodes[st->nr - 1];

		if (!staged_ready(st))
			continue;
		ret = t6041_max_apply(i, display);
		if (ret)
			staged_err("%s: could not enable the chain: %d\n", dcp, ret);
		else
			staged_info("%s enabled\n", dcp);
	}
	of_node_put(display);
}

/*
 * M4: the dcpext Type-C routes are disabled in the device tree,
 * as on J613, because a USB-C port controller waits for the mode switch of
 * every available route. Enable them only for a dcpext the boot loader handed
 * over (apple,t8132-dcpext-firmware-ready = 1). A handed-over dcpext that is
 * not adopted then disables them again, see dcp_platform_probe_t8132_ext().
 * Only the device tree changes here; no register is touched.
 */
static void __init gate_j713_dcpext_routes(void)
{
	struct of_changeset *cs = gate_cs_alloc();
	struct device_node *np;
	unsigned int n = 0;
	u32 ready;
	int ret = 0;

	if (!cs)
		return;
	for_each_compatible_node(np, NULL, "apple,t8132-dcpext") {
		if (!of_device_is_available(np) ||
		    of_property_read_u32(np, "apple,t8132-dcpext-firmware-ready", &ready) ||
		    ready != 1)
			continue;
		ret = gate_ext_routes(cs, np, "okay");
		if (ret) {
			of_node_put(np);
			break;
		}
		n++;
	}
	if (!ret && n)
		ret = of_changeset_apply(cs);
	if (ret)
		pr_err("apple-j713-display: dcpext Type-C routes left disabled: %d\n", ret);
	else if (n)
		pr_info("apple-j713-display: Type-C routes enabled for %u handed-over dcpext(s)\n",
			n);
	/* Applied, it stays: the live tree refers to it. */
	if (ret || !n)
		gate_cs_free(cs);
}

/* Runs before of_platform_default_populate_init() at arch_initcall_sync. */
static int __init apple_display_gate(void)
{
	int i;

	if (apple_t8132_firmware_path()) {
		gate_j713_dcpext_routes();
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(gate_socs) && !gate_soc; i++)
		if (of_machine_is_compatible(gate_socs[i]->machine))
			gate_soc = gate_socs[i];
	if (!gate_soc)
		return 0;

	/* The exact 25G83 tree (J613; J615 experimental) has its own firmware and PMP admission gate. */
	if (of_machine_is_compatible("apple,j613") || of_machine_is_compatible("apple,j615")) {
		struct device_node *dcp = of_find_node_by_path("/soc/dcp@28ec00000");
		bool native = dcp && of_property_present(dcp, "apple,j613-25g83-profile");

		of_node_put(dcp);
		if (native)
			return 0;
	}

	if (gate_soc->run)
		gate_soc->run();
	else
		gate_run();
	return 0;
}
arch_initcall(apple_display_gate);

#if IS_ENABLED(CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST)
#include "display-gate-test.c"
#endif
