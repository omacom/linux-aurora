// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Coldplug of the exact 25G83 display and its same-boot PMP supplier: a J613,
 * or (experimental) a J615 whose boot loader carries the J615 opt-in.
 */
#include <linux/bitfield.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/uuid.h>

#include "dcp.h"
#include "j613-25g83.h"

static unsigned int display_clock_hz;
static struct of_changeset gate;

unsigned int apple_j613_25g83_clock_hz(void)
{
	return display_clock_hz;
}

/* Require one Air board identity and the explicit J615 experimental switch. */
bool apple_t8122_25g83_board(void)
{
	struct device_node *chosen = of_find_node_by_path("/chosen");
	const char *optin = NULL;
	int len = 0;
	bool ok;

	if (chosen)
		optin = of_get_property(chosen, J615_25G83_OPT_IN, &len);
	ok = t8122_25g83_board(of_machine_is_compatible("apple,j613"),
			       of_machine_is_compatible("apple,j615"), optin, len);
	of_node_put(chosen);
	return ok;
}

/* Match the PMGR provider: automatic domains can be clock-gated while active. */
static bool j613_power_state_active(u32 state)
{
	return FIELD_GET(GENMASK(7, 4), state) == 15 ||
	       ((state & BIT(28)) && FIELD_GET(GENMASK(3, 0), state) == 15);
}

/* These are DT domain nodes, carrying the PMGR offsets, not copied registers. */
static int j613_power_ready(void)
{
	static const char *const labels[] = {
		"disp_sys", "disp_fe", "disp_cpu", "pmp", "pms_sram",
	};
	static const u32 offsets[] = { 0x1d8, 0x208, 0x10000, 0x450, 0x458 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(labels); i++) {
		struct device_node *np, *found = NULL;
		const char *label;
		void __iomem *pmgr;
		struct resource res;
		u32 offset, state;

		for_each_compatible_node(np, NULL, "apple,t8122-pmgr-pwrstate") {
			if (of_property_read_string(np, "label", &label) ||
			    strcmp(label, labels[i]))
				continue;
			if (found) {
				of_node_put(found);
				of_node_put(np);
				return -EINVAL;
			}
			found = of_node_get(np);
		}
		if (!found)
			return -ENODEV;
		if (of_property_read_u32(found, "reg", &offset) || offset != offsets[i] ||
		    of_address_to_resource(found->parent, 0, &res) ||
		    resource_size(&res) < 4 || offset > resource_size(&res) - 4) {
			of_node_put(found);
			return -EINVAL;
		}
		/* The register range is described by the parent syscon. */
		pmgr = of_iomap(found->parent, 0);
		of_node_put(found);
		if (!pmgr)
			return -ENOMEM;
		state = readl(pmgr + offset);
		iounmap(pmgr);
		if (!j613_power_state_active(state))
			return -EBUSY;
	}
	return 0;
}

/* The SRAM image and integer bootargs must match this boot's nub. */
static int j613_pmp_image(struct device_node *pmp)
{
	void __iomem *sram, *asc, *ptd;
	struct resource res;
	uuid_t image, expected;
	u32 offset, size;
	u8 *args;
	unsigned int pos;
	int ret = -EINVAL;

	if (uuid_parse(J613_25G83_PMP_UUID, &expected) ||
	    of_address_to_resource(pmp, 0, &res))
		return -EINVAL;
	sram = of_iomap(pmp, 0);
	if (!sram)
		return -ENOMEM;
	if (resource_size(&res) < 0x234)
		goto out;
	for (pos = 0; pos < sizeof(image.b); pos += 4)
		put_unaligned_le32(readl(sram + 0x214 + pos), image.b + pos);
	offset = readl(sram + 0x22c);
	size = readl(sram + 0x230);
	if (!uuid_equal(&image, &expected) || !size || size > SZ_4K ||
	    size > resource_size(&res) || offset > resource_size(&res) - size || offset & 3)
		goto out;
	args = kmalloc(size, GFP_KERNEL);
	if (!args) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy_fromio(args, sram + offset, size);
	ret = apple_pmp_bootargs_valid(args, size) ? 0 : -EINVAL;
	kfree(args);
out:
	iounmap(sram);
	if (ret)
		return ret;
	asc = of_iomap(pmp, 1);
	if (!asc)
		return -ENOMEM;
	/* A new RTKit session starts only the halted, matching PMP image. */
	ret = readl(asc + 0x44) & BIT(4) ? -EBUSY : 0;
	iounmap(asc);
	if (ret)
		return ret;
	ptd = ioremap_np(0x2d03c0000ULL, 0x14000);
	if (!ptd)
		return -ENOMEM;
	ret = readq(ptd + 0x10) || readq(ptd + 0x1000) ? -EBUSY : 0;
	iounmap(ptd);
	return ret;
}

static int j613_pmp_values(struct device_node *pmp)
{
	struct device_node *values = of_find_node_by_path("/chosen/asahi,t8122-pmp");
	struct property *prop, *copy;
	const char *uuid;
	const u8 *table;
	u32 value;
	int length, ret = -EINVAL;

	if (!values)
		return -ENODEV;
	if (of_property_read_u32(values, "apple,board-id", &value) ||
	    of_property_read_u32(values, "apple,dram-vendor-id", &value) ||
	    of_property_read_string(values, "apple,tunable-uuid", &uuid) ||
	    strcmp(uuid, J613_25G83_PMP_UUID))
		goto out;
	table = of_get_property(values, "apple,tunable-ptd-range", &length);
	if (!apple_pmp_ranges_valid(table, length, 0x1000, 0x10800, 0x1080))
		goto out;
	table = of_get_property(values, "apple,tunable-soc-device", &length);
	if (!table || length != 20 * 0x7c ||
	    get_unaligned_le32(table + 11 * 0x7c) != 8 ||
	    memcmp(table + 11 * 0x7c + 0x74, "DISP\0\0\0\0", 8))
		goto out;
	ret = j613_pmp_image(pmp);
	if (ret)
		goto out;
	for_each_property_of_node(values, prop) {
		if (strcmp(prop->name, "apple,board-id") &&
		    strcmp(prop->name, "apple,dram-vendor-id") &&
		    strcmp(prop->name, "apple,dram-capacity") &&
		    !strstarts(prop->name, "apple,tunable-"))
			continue;
		if (!prop->length || prop->length > SZ_64K ||
		    of_property_present(pmp, prop->name)) {
			ret = -EINVAL;
			break;
		}
		copy = kzalloc_obj(*copy);
		if (!copy) {
			ret = -ENOMEM;
			break;
		}
		copy->name = kstrdup(prop->name, GFP_KERNEL);
		copy->value = kmemdup(prop->value, prop->length, GFP_KERNEL);
		copy->length = prop->length;
		if (!copy->name || !copy->value) {
			kfree(copy->name);
			kfree(copy->value);
			kfree(copy);
			ret = -ENOMEM;
			break;
		}
		ret = of_changeset_add_property(&gate, pmp, copy);
		if (ret) {
			kfree(copy->name);
			kfree(copy->value);
			kfree(copy);
		}
		if (ret)
			break;
	}
out:
	of_node_put(values);
	return ret;
}

/* The nodes the coldplug enables, in the order its changeset enables them. */
enum {
	J613_PMP_REPORT,
	J613_PMP_DART,
	J613_PMP_MBOX,
	J613_PMP,
	J613_DCP_DART,
	J613_DISP0_DART,
	J613_DCP_MBOX,
	J613_DISPLAY,		/* these three carry the mapping handoff markers */
	J613_PIODMA,
	J613_DCP,
	J613_NR_NODES,
};

/*
 * The node named by the single entry of @list in @np, if it is compatible
 * with @compat and, for a DART (@stream >= 0), is that DART's stream @stream.
 * Returns NULL otherwise.
 */
static struct device_node *j613_target(const struct device_node *np,
				       const char *list, const char *cells,
				       int stream, const char *compat)
{
	struct of_phandle_args args;

	if (of_count_phandle_with_args(np, list, cells) != 1 ||
	    of_parse_phandle_with_args(np, list, cells, 0, &args))
		return NULL;
	if (args.args_count != (stream < 0 ? 0 : 1) ||
	    (stream >= 0 && args.args[0] != (u32)stream) ||
	    !of_device_is_compatible(args.np, compat)) {
		of_node_put(args.np);
		return NULL;
	}
	return args.np;
}

/*
 * Finds the display and PMP nodes from nodes[J613_DCP] through the phandles
 * that wire them, and the display subsystem through the "disp0" alias, so
 * that the lookup follows the device tree rather than its node names and
 * places: the profile reuses Asahi's PMP report and mailbox nodes, and the
 * display subsystem sits at the root, outside the simple-bus. A node that is
 * not found stays NULL.
 */
static void j613_resolve(struct device_node **nodes)
{
	struct device_node *dcp = nodes[J613_DCP], *display, *pmp;

	nodes[J613_DCP_DART] = j613_target(dcp, "iommus", "#iommu-cells", 5,
					   "apple,t8122-dart");
	nodes[J613_DCP_MBOX] = j613_target(dcp, "mboxes", "#mbox-cells", -1,
					   "apple,t8122-asc-mailbox");
	nodes[J613_PIODMA] = of_get_child_by_name(dcp, "piodma");

	display = of_find_node_by_path("disp0");
	if (display && !of_device_is_compatible(display, "apple,t8122-display-subsystem")) {
		of_node_put(display);
		display = NULL;
	}
	nodes[J613_DISPLAY] = display;
	if (display)
		nodes[J613_DISP0_DART] = j613_target(display, "iommus", "#iommu-cells",
						     0, "apple,t8122-dart");

	pmp = j613_target(dcp, "apple,pmp", NULL, -1, "apple,t8122-pmp-v2");
	nodes[J613_PMP] = pmp;
	if (!pmp)
		return;
	nodes[J613_PMP_REPORT] = j613_target(pmp, "apple,pmp-report", NULL, -1,
					     "apple,j613-25g83-pmp-report");
	nodes[J613_PMP_DART] = j613_target(pmp, "iommus", "#iommu-cells", 0,
					   "apple,t8122-dart");
	nodes[J613_PMP_MBOX] = j613_target(pmp, "mboxes", "#mbox-cells", -1,
					   "apple,t8122-asc-mailbox");
}

static struct of_changeset external_gate;
static bool external_gate_applied;

/*
 * UPDATE_PROPERTY also inserts an absent property and records its old value
 * for changeset rollback. Firmware DTs may already constrain these suppliers.
 */
static int __init j613_external_update(struct device_node *np, const char *name,
				     const void *value, unsigned int length)
{
	struct property *prop = kzalloc_obj(*prop);
	int ret;

	if (!prop)
		return -ENOMEM;
	prop->name = kstrdup(name, GFP_KERNEL);
	prop->length = length;
	if (length)
		prop->value = kmemdup(value, length, GFP_KERNEL);
	if (!prop->name || (length && !prop->value)) {
		ret = -ENOMEM;
		goto free;
	}
	ret = of_changeset_update_property(&external_gate, np, prop);
	if (!ret)
		return 0;
free:
	kfree(prop->value);
	kfree(prop->name);
	kfree(prop);
	return ret;
}


/* Optional external transaction; its failure must not unwind the panel. */
static int __init j613_external_gate(struct device_node *display)
{
	struct device_node *chosen, *ext = NULL, *pio = NULL, *report = NULL;
	struct device_node *nodes[9] = {}, *cpu = NULL, *routes = NULL, *route;
	struct of_phandle_args args;
	const char *opt, *uuid, *status, *loader_result;
	const char *reason = "external node or compatible";
	struct property *prop = NULL;
	const void *old;
	__be32 *cells;
	u32 state[4], version[3], ready;
	int len, ret = -EINVAL, i, count = 0;

	chosen = of_find_node_by_path("/chosen");
	opt = chosen ? of_get_property(chosen, "asahi,j613-25g83-dcpext", &len) : NULL;
	if (!opt || len != 2 || opt[0] != '1' || opt[1]) {
		of_node_put(chosen);
		return 0;
	}
	loader_result = of_get_property(chosen, "asahi,j613-25g83-dcpext-result", &len);
	if (loader_result && len > 0 && loader_result[len - 1] == 0)
		pr_info("J613 native26 external loader result: %s\n", loader_result);
	else
		pr_info("J613 native26 external loader result: absent or malformed\n");
	of_node_put(chosen);
	if (!of_machine_is_compatible("apple,j613"))
		return -ENODEV;
	ext = of_find_node_by_path("dcpext");
	if (!ext)
		ext = of_find_node_by_path("dcpext0");
#define J613_EXT_REQUIRE(test, why) do { \
	if (!(test)) { \
		reason = why; \
		goto out; \
	} \
} while (0)
	J613_EXT_REQUIRE(ext && of_device_is_compatible(ext, "apple,t8122-dcpext"),
			"external node or compatible");
	J613_EXT_REQUIRE(of_property_count_u32_elems(ext, "apple,firmware-compat") == 3 &&
			!of_property_read_u32_array(ext, "apple,firmware-compat", version, 3),
			"missing or malformed apple,firmware-compat");
	J613_EXT_REQUIRE(!of_property_read_string(ext, "apple,firmware-uuid", &uuid),
			"missing apple,firmware-uuid");
	J613_EXT_REQUIRE(j613_25g83_identity(true, 1, version, 3, uuid), "firmware identity");
	J613_EXT_REQUIRE(!of_property_read_u32(ext, "apple,j613-25g83-dcpext-handoff", &ready) && ready == 1,
			"missing native handoff marker");
	J613_EXT_REQUIRE(!of_property_read_u32(ext, "apple,t8122-dcpext-firmware-ready", &ready) && ready == 1,
			"missing firmware-ready marker");
	J613_EXT_REQUIRE(of_property_count_u32_elems(ext, "apple,j613-25g83-dcpext-boot-state") == 4 &&
			!of_property_read_u32_array(ext, "apple,j613-25g83-dcpext-boot-state", state, 4),
			"missing or malformed boot-state");
	J613_EXT_REQUIRE(j613_25g83_external_state(state, 4), "boot-state not powered RUN and locked");
	J613_EXT_REQUIRE(of_property_present(ext, "memory-region"), "missing memory-region");
#undef J613_EXT_REQUIRE
	pr_info("J613 native26 external handoff: identity, markers, boot-state and memory-region accepted\n");
	reason = "PMP/DART/mailbox suppliers";
	nodes[count++] = of_node_get(ext);
	nodes[count++] = j613_target(ext, "iommus", "#iommu-cells", 5, "apple,t8122-dart");
	nodes[count++] = j613_target(ext, "mboxes", "#mbox-cells", -1, "apple,t8122-asc-mailbox");
	pio = of_get_child_by_name(ext, "piodma");
	nodes[count++] = pio ? j613_target(pio, "iommus", "#iommu-cells", 4, "apple,t8122-dart") : NULL;
	nodes[count++] = j613_target(ext, "apple,pmp-report", NULL, -1,
					"apple,t6000-pmp-v2-report-entry");
	if (!nodes[4] || of_property_read_u32(nodes[4], "reg", &ready) || ready != 8 ||
	    of_property_read_bool(nodes[4], "apple,no-ack") || !nodes[1] ||
	    !nodes[3] || nodes[1] == nodes[3])
		goto out;
	reason = "PMP report compatible";
	report = of_get_parent(nodes[4]);
	if (!report || !of_device_is_compatible(report, "apple,j613-25g83-pmp-report"))
		goto out;
	reason = "DISPEXT CPU power domain";
	cpu = of_parse_phandle(ext, "power-domains", 0);
	if (!cpu || of_property_read_string(cpu, "label", &status) || strcmp(status, "dispext_cpu") ||
	    !of_property_read_bool(cpu, "apple,firmware-owned"))
		goto out;
	reason = "two crossbar routes";
	if (of_count_phandle_with_args(ext, "mux-controls", "#mux-control-cells") != 2)
		goto out;
	for (i = 0; i < 2; i++) {
		if (of_parse_phandle_with_args(ext, "mux-controls", "#mux-control-cells", i, &args))
			goto out;
		nodes[count++] = args.np;
		if (args.args_count != 1 || args.args[0] ||
		    !of_device_is_compatible(args.np, "apple,t6020-display-crossbar"))
			goto out;
	}
	reason = "supplier must be disabled";
	for (i = 0; i < count; i++) {
		if (!nodes[i] || of_property_read_string(nodes[i], "status", &status) ||
		    strcmp(status, "disabled"))
			goto out;
	}
	reason = "two Type-C route children";
	routes = of_get_child_by_name(ext, "typec-routes");
	if (of_get_child_count(routes) != 2)
		goto out;
	reason = "internal display IOMMU shape";
	old = of_get_property(display, "iommus", &len);
	if (!old || len != 8 || !nodes[3]->phandle)
		goto out;
	reason = "changeset allocation";
	prop = kzalloc_obj(*prop);
	cells = kmalloc(16, GFP_KERNEL);
	if (!prop || !cells) {
		kfree(prop);
		kfree(cells);
		prop = NULL;
		ret = -ENOMEM;
		goto out;
	}
	memcpy(cells, old, 8);
	cells[2] = cpu_to_be32(nodes[3]->phandle);
	cells[3] = 0;
	prop->name = kstrdup("iommus", GFP_KERNEL);
	prop->length = 16;
	prop->value = cells;
	if (!prop->name) {
		kfree(cells);
		kfree(prop);
		ret = -ENOMEM;
		goto out;
	}
	reason = "resource changeset";
	of_changeset_init(&external_gate);
	ret = of_changeset_update_property(&external_gate, display, prop);
	if (!ret)
		ret = j613_external_update(nodes[4], "apple,always-on", NULL, 0);
	for (i = 0; i < count && !ret; i++)
		ret = of_changeset_update_prop_string(&external_gate, nodes[i], "status", "okay");
	for_each_child_of_node(routes, route) {
		if (!ret)
			ret = of_changeset_update_prop_string(&external_gate, route, "status", "okay");
	}
	if (!ret)
		ret = of_changeset_apply(&external_gate);
	if (ret)
		of_changeset_destroy(&external_gate);
	else {
		external_gate_applied = true;
		pr_info("J613 native26 external gate accepted: reserved firmware, DISPEXT12 and both Type-C routes\n");
	}
out:
	if (ret)
		pr_info("J613 native26 external gate refused: %d (%s); internal display retained\n", ret, reason);
	for (i = 0; i < count; i++)
		of_node_put(nodes[i]);
	of_node_put(cpu);
	of_node_put(routes);
	of_node_put(report);
	of_node_put(pio);
	of_node_put(ext);
	return ret;
}

int __init apple_j613_25g83_coldplug(void)
{
	struct device_node *nodes[J613_NR_NODES] = {}, *chosen;
	const char *uuid, *status;
	void __iomem *cpu;
	u32 profile, version[3], witness[5], markers[3];
	unsigned int i;
	int ret = -EINVAL;

	if (!of_machine_is_compatible("apple,j613") && !of_machine_is_compatible("apple,j615"))
		return 0;
	nodes[J613_DCP] = of_find_node_by_path("dcp");
	if (nodes[J613_DCP] && !of_device_is_compatible(nodes[J613_DCP], "apple,t8122-dcp")) {
		of_node_put(nodes[J613_DCP]);
		nodes[J613_DCP] = NULL;
	}
	/*
	 * Only the 25G83 profile is coldplugged here. This tree does not
	 * carry the T8122 path of the display gate yet, so an ordinary 14.x
	 * J613/J615 boot leaves the display disabled, on the boot framebuffer.
	 */
	if (!nodes[J613_DCP] || !of_property_present(nodes[J613_DCP], "apple,j613-25g83-profile")) {
		ret = 0;
		goto out;
	}
	/* A J615 25G83 device tree alone is not consent: the owner's opt-in is. */
	if (!apple_t8122_25g83_board()) {
		pr_info("J615/25G83 display handoff not admitted: no /chosen/%s = \"1\"\n",
			J615_25G83_OPT_IN);
		ret = -EPERM;
		goto out;
	}
	if (of_property_count_u32_elems(nodes[J613_DCP], "apple,j613-25g83-profile") != 1 ||
	    of_property_read_u32(nodes[J613_DCP], "apple,j613-25g83-profile", &profile) ||
	    of_property_count_u32_elems(nodes[J613_DCP], "apple,firmware-compat") != 3 ||
	    of_property_read_u32_array(nodes[J613_DCP], "apple,firmware-compat", version, 3) ||
	    of_property_read_string(nodes[J613_DCP], "apple,firmware-uuid", &uuid) ||
	    !j613_25g83_identity(true, profile, version, 3, uuid))
		goto out;
	chosen = of_find_node_by_path("/chosen");
	ret = chosen && of_property_count_u32_elems(chosen,
		"apple,j613-25g83-display-clock-adt") == 5 ?
		of_property_read_u32_array(chosen, "apple,j613-25g83-display-clock-adt", witness, 5) : -EINVAL;
	of_node_put(chosen);
	if (ret || !j613_25g83_clock(witness, 5)) {
		ret = -EINVAL;
		goto out;
	}
	j613_resolve(nodes);
	for (i = 0; i < ARRAY_SIZE(nodes); i++) {
		if (!nodes[i]) {
			ret = -ENODEV;
			goto out;
		}
		if (of_property_read_string(nodes[i], "status", &status) ||
		    strcmp(status, "disabled")) {
			ret = -EBUSY;
			goto out;
		}
	}
	if (nodes[J613_DCP_DART] == nodes[J613_DISP0_DART]) {
		ret = -EINVAL;
		goto out;
	}
	/* The display subsystem, the PIODMA node and the DCP, in that order. */
	for (i = 0; i < 3; i++) {
		struct device_node *np = nodes[J613_DISPLAY + i];

		if (of_property_count_u32_elems(np, J613_25G83_HANDOFF) != 1 ||
		    of_property_read_u32(np, J613_25G83_HANDOFF, &markers[i])) {
			ret = -EINVAL;
			goto out;
		}
	}
	if (!j613_25g83_mappings(markers, 3)) {
		ret = -EINVAL;
		goto out;
	}
	ret = j613_power_ready();
	if (ret)
		goto out;
	cpu = of_iomap(nodes[J613_DCP], 0);
	if (!cpu) {
		ret = -ENOMEM;
		goto out;
	}
	ret = readl(cpu + 0x44) & BIT(4) ? 0 : -EBUSY;
	iounmap(cpu);
	if (ret)
		goto out;
	of_changeset_init(&gate);
	ret = j613_pmp_values(nodes[J613_PMP]);
	for (i = 0; i < ARRAY_SIZE(nodes) && !ret; i++)
		ret = of_changeset_update_prop_string(&gate, nodes[i], "status", "okay");
	if (!ret) {
		j613_external_gate(nodes[J613_DISPLAY]);
		display_clock_hz = witness[3];
		ret = of_changeset_apply(&gate);
	}
	if (ret) {
		display_clock_hz = 0;
		if (external_gate_applied) {
			of_changeset_revert(&external_gate);
			of_changeset_destroy(&external_gate);
			external_gate_applied = false;
		}
		of_changeset_destroy(&gate);
	} else {
		display_clock_hz = witness[3];
		pr_info("%s/25G83 display handoff accepted; ADT clock %u Hz\n",
			of_machine_is_compatible("apple,j615") ? "J615 (experimental)" : "J613",
			display_clock_hz);
	}
out:
	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		of_node_put(nodes[i]);
	return ret;
}
