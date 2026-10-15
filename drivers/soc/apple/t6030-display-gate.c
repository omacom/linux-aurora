// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T6030 internal display enable gate, and its T8122 counterpart
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
 * apple,t8122-dcp and apple,t8122-display-subsystem nodes, and adds the T8122
 * PMP the same way. There is no T8122 fallback: the boot loader must pass
 * this Mac's PMP values in /chosen/asahi,t8122-pmp, or nothing is enabled. A
 * T8122 device tree without display nodes is normal and the gate says
 * nothing. apple_t8122_display.enable=0 has the same effect as the T6030
 * option.
 *
 * On T6031 (M3 Max) the gate serves the same handoff, marked
 * apple,t6031-handoff, but only when apple_t6031_display.enable=1 is on the
 * kernel command line: the native display is off by default there until it
 * has been seen to work. The PMP values must come from the boot loader, in
 * /chosen/asahi,t6031-pmp; there is no fallback. The T6031 display power
 * states nest the other way round (the DCP's DISP_FE under DISP_CPU under
 * DISP_SYS), and its PMP power states are in another PMGR block. On every
 * T6031 boot the gate also logs what the boot loader handed over, so that
 * a boot without the option already records what the next one needs.
 */

#define pr_fmt(fmt) fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/soc/apple/j613-display.h>
#include <linux/string.h>
#include <linux/types.h>
#include <video/nomodeset.h>

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
extern const u8 __dtbo_t8122_pmp_begin[];
extern const u8 __dtbo_t8122_pmp_end[];
extern const u8 __dtbo_t6031_pmp_begin[];
extern const u8 __dtbo_t6031_pmp_end[];

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

/* apple_t6031_display.enable: the T6031 native display is off unless it is 1. */
static bool gate_t6031_requested __initdata;

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
	const char *report_compat;	/* the PMP report the overlay adds */
	const char *disp_entry;		/* its display request entry, which the DCP waits for */
	const u8 *pmp_dtbo, *pmp_dtbo_end;	/* NULL: no PMP description built in */
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
 * T8122 (M3 MacBook Air): the same handoff and power states, and a PMP
 * overlay at the T8122 addresses. The PMP values must come from the boot
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
	.pmp_dtbo = __dtbo_t8122_pmp_begin,
	.pmp_dtbo_end = __dtbo_t8122_pmp_end,
	.requested = &gate_requested,
	.ps_chain = { PMP_PS_DISP_CPU, PMP_PS_DISP_FE, PMP_PS_DISP_SYS },
	.quiet_without_nodes = true,
};

/*
 * T6031 (M3 Max): the T6030 handoff at the T6031 addresses, off unless
 * apple_t6031_display.enable=1. The PMP values must come from the boot
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
	.pmp_dtbo = __dtbo_t6031_pmp_begin,
	.pmp_dtbo_end = __dtbo_t6031_pmp_end,
	.requested = &gate_t6031_requested,
	.opt_in = "apple_t6031_display.enable=1",
	.ps_chain = { PMP_PS_DISP_FE, PMP_PS_DISP_CPU, PMP_PS_DISP_SYS },
	.pmp_ps_any_pmgr = true,
	.describe = true,
	.fw_uuids = "asahi,t6031-fw-uuids",
};

static const struct gate_soc *const gate_socs[] __initconst = {
	&gate_t6030,
	&gate_t8122,
	&gate_t6031,
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

bool __init apple_t6030_display_gate_enabled(void)
{
	return gate_requested;
}

/* Kept after a successful apply: the live tree now holds its properties. */
static struct of_changeset gate_cs;
static struct of_changeset gate_pmp_cs;
/* The PMP overlay, once applied; only the KUnit test removes it again. */
static int gate_pmp_ovcs __initdata;

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

/* Its own switch: the T6030 and T8122 options do not change it, nor it them. */
static int __init gate_setup_t6031(char *arg)
{
	return kstrtobool(arg, &gate_t6031_requested);
}
early_param("apple_t6031_display.enable", gate_setup_t6031);

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

	if (!gate_soc->pmp_dtbo) {
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

static int __init gate_t8122_gpu_pmp_link(struct of_changeset *cs, struct device_node *pmp)
{
	struct device_node *gpu, *linked;
	int ret = 0;

	if (gate_soc != &gate_t8122)
		return 0;
	gpu = of_find_compatible_node(NULL, NULL, "apple,agx-t8122");
	if (!gpu || !of_device_is_available(gpu))
		goto out;
	linked = of_parse_phandle(gpu, "apple,pmp", 0);
	if (!pmp || !pmp->phandle || (linked && linked != pmp))
		ret = -EINVAL;
	else if (!linked)
		ret = of_changeset_add_prop_u32(cs, gpu, "apple,pmp", pmp->phandle);
	of_node_put(linked);
out:
	of_node_put(gpu);
	return ret;
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
	/* The current14 T8122 GPU needs an inner AGX vote from this Mac's PMP. */
	if (!ret)
		ret = gate_t8122_gpu_pmp_link(&gate_pmp_cs, pmp);
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

/* The display subsystem's iommus with <@dart 0> appended. */
static int __init gate_display_add_dart(struct of_changeset *cs, struct device_node *dart)
{
	struct device_node *display = gate_find_one(gate_soc->display_compat);
	struct of_phandle_args args;
	struct property *old, *prop;
	__be32 *value;
	int i, n, ret = -EINVAL;

	if (!display)
		return -ENODEV;
	old = of_find_property(display, "iommus", NULL);
	n = of_count_phandle_with_args(display, "iommus", "#iommu-cells");
	if (!old || n < 1 || old->length != n * 2 * sizeof(__be32))
		goto out;
	for (i = 0; i < n; i++) {
		if (of_parse_phandle_with_args(display, "iommus", "#iommu-cells", i, &args))
			goto out;
		of_node_put(args.np);
		if (args.np == dart || args.args_count != 1)
			goto out;
	}
	ret = -ENOMEM;
	prop = kzalloc_obj(*prop);
	value = kmalloc(old->length + 2 * sizeof(*value), GFP_KERNEL);
	if (!prop || !value) {
		kfree(prop);
		kfree(value);
		goto out;
	}
	memcpy(value, old->value, old->length);
	value[old->length / sizeof(*value)] = cpu_to_be32(dart->phandle);
	value[old->length / sizeof(*value) + 1] = cpu_to_be32(0);
	prop->name = "iommus";
	prop->length = old->length + 2 * sizeof(*value);
	prop->value = value;
	ret = of_changeset_update_property(cs, display, prop);
out:
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
 *  - if the PMP was added, its DISPEXT request (pmp-dispext0), as on
 *    T6030: once the PMP runs it owns the SoC power and performance states
 *    and keeps a block up only if that block's request bit was set when it
 *    started. The dcpext points at the entry (apple,pmp-report) and the
 *    display driver waits for its acknowledgement before it runs the dcpext
 *    CPU.
 * The DARTs, the mailbox and the crossbars must be described and still
 * disabled. Without the marker, with other firmware, or if anything does not
 * check out, all of them stay disabled and the ATC power states are left
 * alone. apple_t8122_display.dcpext=0 keeps them disabled too.
 *
 * The same changeset enables the Type-C routes the device tree gives the
 * internal DCP (J613: to the left-front port, for a second external display
 * with the lid closed). They lead to ports that dcpext drives too, and only
 * the display driver's lid handling makes them usable, so they are enabled
 * exactly when dcpext is, and must be disabled until then.
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

/*
 * The internal DCP's Type-C routes, which the gate enables together with
 * dcpext: they must be disabled, and every crossbar the internal DCP switches
 * must be one of dcpext's (@xbar), which the same changeset enables. Returns
 * why not, or NULL.
 */
static const char *__init gate_t8122_dcp_routes_bad(struct device_node *dcp,
						    struct device_node *routes,
						    struct device_node *const *xbar, int nxbar)
{
	struct device_node *route;
	struct of_phandle_args args;
	bool listed;
	int i, n;

	for_each_child_of_node(routes, route) {
		if (!gate_disabled(route)) {
			of_node_put(route);
			return "the internal display's Type-C route is not disabled";
		}
	}
	n = of_count_phandle_with_args(dcp, "mux-controls", "#mux-control-cells");
	if (n < 1)
		return "the internal display's Type-C route has no display crossbar";
	for (i = 0; i < n; i++) {
		if (of_parse_phandle_with_args(dcp, "mux-controls", "#mux-control-cells",
					       i, &args))
			return "the internal display's display crossbar is not described";
		listed = gate_node_in(xbar, nxbar, args.np);
		of_node_put(args.np);
		if (!listed)
			return "the internal display's display crossbar is not one of the dcpext's";
	}
	return NULL;
}

static void __init gate_t8122_dcpext_one(struct device_node *np, struct device_node *dcp)
{
	struct device_node *dart = NULL, *mbox = NULL, *piodma = NULL;
	struct device_node *disp_dart = NULL, *ps = NULL, *entry = NULL;
	struct device_node *xbar[GATE_T8122_XBARS] = {};
	struct device_node *common[GATE_T8122_XBARS] = {};
	struct device_node *dcp_routes = NULL;
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
	 */
	entry = gate_pmp_dispext_entry(0);
	if (!entry || !entry->phandle ||
	    !of_device_is_compatible(entry, "apple,t6000-pmp-v2-report-entry") ||
	    !gate_disabled(entry)) {
		gate_ext_refuse(np, "its PMP DISPEXT request is not described or not usable");
		goto put;
	}

	dcp_routes = of_get_child_by_name(dcp, "typec-routes");
	if (dcp_routes) {
		const char *why = gate_t8122_dcp_routes_bad(dcp, dcp_routes, xbar, nxbar);

		if (why) {
			gate_ext_refuse(np, why);
			goto put;
		}
	}
	cs = gate_cs_alloc();
	if (!cs) {
		gate_ext_refuse(np, "out of memory");
		goto put;
	}
	ret = gate_set_u32(cs, ps, "apple,min-state", PMGR_PS_ACTIVE);
	/* The driver waits for this request's acknowledgement before RUN. */
	if (!ret) {
		ret = gate_set_u32(cs, np, "apple,pmp-report", entry->phandle);
		if (!ret)
			ret = of_changeset_update_prop_string(cs, entry, "status", "okay");
	}
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
	if (!ret && dcp_routes)
		ret = gate_ext_routes(cs, dcp, "okay");
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
	if (dcp_routes)
		gate_info("enabled the Type-C routes of %pOF with it, for a second external display with the lid closed\n",
			  dcp);
put:
	of_node_put(dcp_routes);
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

/* @dcp: the internal DCP, if the internal display was handed over. */
static void __init gate_t8122_dcpext(struct device_node *dcp)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "apple,t8122-dcpext") {
		if (!dcp)
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
			gate_t8122_dcpext_one(np, dcp);
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

	if (!*gate_soc->requested) {
		if (gate_soc->opt_in)
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
		gate_t8122_dcpext(panel ? np[GATE_DCP] : NULL);
	of_node_put(aic);
	for (i = 0; i < PMP_PS_NR; i++)
		of_node_put(ps[i]);
	for (i = 0; i < GATE_NR_NODES; i++)
		of_node_put(np[i]);
}

/* Runs before of_platform_default_populate_init() at arch_initcall_sync. */
static int __init apple_t6030_display_gate(void)
{
	int i;

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

	gate_run();
	return 0;
}
arch_initcall(apple_t6030_display_gate);

#if IS_ENABLED(CONFIG_APPLE_DISPLAY_GATE_KUNIT_TEST)
#include "t6030-display-gate-test.c"
#endif
