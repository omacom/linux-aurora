// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Supply missing platform descriptions for the original J713 loader ABI,
 * which the J715 shares, and the M4 PMGR ownership policy on the qualified
 * firmware for every M4 Mac.
 * Keep the shared m1n1 payload usable by the stable kernel. Hardware control
 * remains in the ordinary MCC, PMC and CLPC drivers, with one DTS data source.
 * J713 PMGR ownership/status also comes from its board DTS, before any domain
 * registers can be accessed. Native descriptions take precedence.
 */
#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/soc/apple/t8132.h>
#include <linux/string.h>

extern const u8 __dtb_t8132_platform_data_begin[];

static int __init t8132_copy_property(struct of_changeset *cs,
				      struct device_node *node,
				      const char *name, const void *value,
				      int len)
{
	struct property *prop;
	int ret;

	prop = kzalloc_obj(*prop);
	if (!prop)
		return -ENOMEM;
	prop->name = kstrdup(name, GFP_KERNEL);
	prop->value = kmemdup(value, len, GFP_KERNEL);
	prop->length = len;
	if (!prop->name || !prop->value) {
		ret = -ENOMEM;
		goto fail;
	}
	of_property_set_flag(prop, OF_DYNAMIC);
	ret = cs ? of_changeset_add_property(cs, node, prop) :
		   of_add_property(node, prop);
	if (!ret) {
		if (cs) {
			prop->next = node->deadprops;
			node->deadprops = prop;
		}
		return 0;
	}
fail:
	kfree(prop->name);
	kfree(prop->value);
	kfree(prop);
	return ret;
}

static int __init t8132_copy_node(struct of_changeset *cs,
				  struct device_node *parent, int offset,
				  phandle pmp, phandle gfx, unsigned int *added)
{
	const void *data = __dtb_t8132_platform_data_begin;
	const char *name = fdt_get_name(data, offset, NULL);
	char *base;
	struct device_node *node;
	int prop_offset, child, len, ret = 0;

	/* Compare the unit address too; of_get_child_by_name() omits it. */
	for_each_child_of_node_scoped(parent, child_node)
		if (!strcmp(kbasename(child_node->full_name), name))
			return 0;
	node = of_changeset_create_node(cs, parent, name);
	if (!node)
		return -ENOMEM;
	/* The attach action reads name before queued properties are applied. */
	base = kstrndup(name, strcspn(name, "@"), GFP_KERNEL);
	if (!base) {
		ret = -ENOMEM;
		goto out;
	}
	ret = t8132_copy_property(NULL, node, "name", base, strlen(base) + 1);
	kfree(base);
	if (ret)
		goto out;
	fdt_for_each_property_offset(prop_offset, data, offset) {
		const struct fdt_property *prop =
			fdt_get_property_by_offset(data, prop_offset, &len);

		if (!prop) {
			ret = -EINVAL;
			goto out;
		}
		name = fdt_string(data, fdt32_to_cpu(prop->nameoff));
		if (!strcmp(name, "status") || !strcmp(name, "phandle") ||
		    !strcmp(name, "name"))
			continue;
		if (!strcmp(name, "apple,pmp") ||
		    !strcmp(name, "apple,boot-dma-devices"))
			ret = of_changeset_add_prop_u32(cs, node, name, pmp);
		else if (!strcmp(name, "apple,gpu-power-state"))
			ret = of_changeset_add_prop_u32(cs, node, name, gfx);
		else
			ret = t8132_copy_property(cs, node, name, prop->data,
						  len);
		if (ret)
			goto out;
	}
	ret = of_changeset_add_prop_string(cs, node, "status", "okay");
	if (ret)
		goto out;
	fdt_for_each_subnode(child, data, offset) {
		ret = t8132_copy_node(cs, node, child, pmp, gfx, added);
		if (ret)
			break;
	}
	if (!ret)
		(*added)++;
out:
	of_node_put(node);
	return ret;
}

/* Resolve the OF ancestry: full_name is a component, not an absolute path. */
static int __init t8132_fdt_node_offset(const void *data,
				      const struct device_node *node)
{
	int parent;

	if (!node || !node->full_name)
		return -FDT_ERR_BADPATH;
	if (of_node_is_root(node))
		return *kbasename(node->full_name) ? -FDT_ERR_BADPATH : 0;

	parent = t8132_fdt_node_offset(data, node->parent);
	if (parent < 0)
		return parent;
	return fdt_subnode_offset(data, parent, kbasename(node->full_name));
}

static bool __init t8132_policy_applies(const void *data, int offset)
{
	const char *machine = fdt_getprop(data, offset, "apple,policy-machine", NULL);

	return !machine || of_machine_is_compatible(machine);
}

/*
 * Apply only availability/ownership, never imported register values. Only the
 * original J713 loader tree (the Airs') can mark a domain okay that the policy
 * refuses. Other trees state what their board wires, so keep their status,
 * except on domains the policy gives the firmware.
 */
static int __init t8132_pmgr_policy(struct of_changeset *cs, unsigned int *changed,
				    bool legacy)
{
	const void *data = __dtb_t8132_platform_data_begin;
	const char * const ownership[] = { "apple,firmware-owned", "apple,inherited-on" };
	struct device_node *pmgr;
	int ret;

	for_each_compatible_node(pmgr, NULL, "apple,t8132-pmgr") {
		int parent = t8132_fdt_node_offset(data, pmgr);

		if (parent < 0) {
			pr_err("t8132-platform-handoff: cannot resolve PMGR policy for %pOF: %d\n",
			       pmgr, parent);
			ret = -EINVAL;
			goto put_pmgr;
		}
		for_each_child_of_node_scoped(pmgr, node) {
			const char *status, *live;
			int offset, len;
			bool owned;

			if (!of_device_is_compatible(node, "apple,t8132-pmgr-pwrstate"))
				continue;
			offset = parent < 0 ? parent :
				fdt_subnode_offset(data, parent, kbasename(node->full_name));
			/* An entry that names its machine is unknown to every other one. */
			if (offset >= 0 && !t8132_policy_applies(data, offset))
				offset = -FDT_ERR_NOTFOUND;
			status = offset < 0 ? NULL : fdt_getprop(data, offset, "status", &len);
			/* Unknown legacy domains stay inaccessible, rather than being probed. */
			if (!status)
				status = "disabled";
			owned = offset >= 0 &&
				(fdt_getprop(data, offset, ownership[0], NULL) ||
				 fdt_getprop(data, offset, ownership[1], NULL));
			if (of_property_read_string(node, "status", &live)) {
				ret = of_changeset_add_prop_string(cs, node, "status", status);
				if (ret)
					goto put_pmgr;
				(*changed)++;
			} else if (strcmp(live, status) &&
				   (legacy ? !strcmp(status, "disabled") : owned)) {
				ret = of_changeset_update_prop_string(cs, node, "status", status);
				if (ret)
					goto put_pmgr;
				(*changed)++;
			}
			for (unsigned int i = 0; i < ARRAY_SIZE(ownership); i++) {
				if (offset < 0 || !fdt_getprop(data, offset, ownership[i], NULL) ||
				    of_property_present(node, ownership[i]))
					continue;
				ret = of_changeset_add_prop_bool(cs, node, ownership[i]);
				if (ret)
					goto put_pmgr;
				(*changed)++;
			}
		}
	}
	return 0;
put_pmgr:
	of_node_put(pmgr);
	return ret;
}

/*
 * A boot framebuffer must not wait on a domain the policy keeps from Linux:
 * that provider never registers, so the consumer only defers until the
 * timeout. Linux never switches such a domain; it stays as the loader left it.
 */
static int __init t8132_framebuffer_domains(void)
{
	struct device_node *fb;
	int ret = 0;

	for_each_compatible_node(fb, NULL, "simple-framebuffer") {
		struct of_phandle_args args;
		struct property *prop;
		__be32 *cells;
		int count, len, kept = 0, domains = 0;

		if (!of_find_property(fb, "power-domains", &len) || len <= 0)
			continue;
		count = of_count_phandle_with_args(fb, "power-domains", "#power-domain-cells");
		if (count <= 0)
			continue;
		cells = kmalloc(len, GFP_KERNEL);
		if (!cells) {
			ret = -ENOMEM;
			goto put_fb;
		}
		for (int i = 0; i < count; i++) {
			bool available;

			ret = of_parse_phandle_with_args(fb, "power-domains", "#power-domain-cells",
							 i, &args);
			if (ret)
				break;
			available = of_device_is_available(args.np);
			if (available && kept + 1 + args.args_count <= len / 4) {
				domains++;
				cells[kept++] = cpu_to_be32(args.np->phandle);
				for (int j = 0; j < args.args_count; j++)
					cells[kept++] = cpu_to_be32(args.args[j]);
			}
			of_node_put(args.np);
		}
		/* Leave a list it cannot read, or one with nothing to drop, as it is. */
		if (ret || kept * 4 == len) {
			kfree(cells);
			ret = 0;
			continue;
		}
		prop = kzalloc_obj(*prop);
		if (!prop) {
			kfree(cells);
			ret = -ENOMEM;
			goto put_fb;
		}
		prop->name = kstrdup("power-domains", GFP_KERNEL);
		prop->value = cells;
		prop->length = kept * 4;
		if (!prop->name) {
			kfree(cells);
			kfree(prop);
			ret = -ENOMEM;
			goto put_fb;
		}
		of_property_set_flag(prop, OF_DYNAMIC);
		ret = of_update_property(fb, prop);
		if (ret) {
			kfree(prop->name);
			kfree(cells);
			kfree(prop);
			goto put_fb;
		}
		pr_info("t8132-platform-handoff: %pOF keeps %d of %d power domains\n",
			fb, domains, count);
	}
	return 0;
put_fb:
	of_node_put(fb);
	return ret;
}

/* Follow the shared DTS reference instead of interpreting a magic PMGR offset. */
static struct device_node *__init t8132_find_gfx(void)
{
	const void *data = __dtb_t8132_platform_data_begin;
	const fdt32_t *prop;
	char path[256];
	int offset, len;

	offset = fdt_node_offset_by_compatible(data, -1, "apple,t8132-clpc");
	if (offset < 0)
		return NULL;
	prop = fdt_getprop(data, offset, "apple,gpu-power-state", &len);
	if (!prop || len != sizeof(*prop))
		return NULL;
	offset = fdt_node_offset_by_phandle(data, fdt32_to_cpu(*prop));
	if (offset < 0 || fdt_get_path(data, offset, path, sizeof(path)))
		return NULL;
	return of_find_node_by_path(path);
}

static int __init t8132_platform_handoff(void)
{
	const void *data = __dtb_t8132_platform_data_begin;
	struct device_node *soc, *pmp = NULL, *gpu = NULL, *gfx = NULL;
	struct of_changeset *cs;
	unsigned int added = 0, changed = 0;
	int root, node, ret;

	if (!apple_t8132_firmware_path())
		return 0;
	if (fdt_check_header(data))
		return -EINVAL;
	soc = of_find_node_by_path("/soc");
	if (!soc)
		return -ENODEV;
	cs = kzalloc_obj(*cs);
	if (!cs) {
		ret = -ENOMEM;
		goto out;
	}
	of_changeset_init(cs);
	ret = t8132_pmgr_policy(cs, &changed, apple_t8132_air());
	if (ret)
		goto destroy;

	pmp = of_find_compatible_node(NULL, NULL, "apple,t8132-pmp-v2");
	gpu = of_find_compatible_node(NULL, NULL, "apple,agx-t8132");
	gfx = gpu ? of_parse_phandle(gpu, "power-domains", 0) : NULL;
	if (!gfx)
		gfx = t8132_find_gfx();
	if (!pmp || !gfx || !pmp->phandle || !gfx->phandle) {
		pr_warn("t8132-platform-handoff: memory compatibility requires PMP/GFX providers\n");
		goto apply;
	}
	root = fdt_path_offset(data, "/soc");
	if (root < 0) {
		ret = -EINVAL;
		goto destroy;
	}
	fdt_for_each_subnode(node, data, root) {
		/* The embedded PMGR nodes contain policy for existing providers only. */
		if (!fdt_getprop(data, node, "compatible", NULL))
			continue;
		ret = t8132_copy_node(cs, soc, node, pmp->phandle,
				      gfx->phandle, &added);
		if (ret)
			goto destroy;
	}
apply:
	ret = of_changeset_apply(cs);
destroy:
	if (ret) {
		of_changeset_destroy(cs);
		kfree(cs);
	} else {
		if (added || changed)
			/* The applied node/property storage lives until reboot. */
			pr_info("t8132-platform-handoff: supplied %u platform nodes, %u PMGR policy properties\n",
				added, changed);
		ret = t8132_framebuffer_domains();
	}
out:
	of_node_put(gfx);
	of_node_put(gpu);
	of_node_put(pmp);
	of_node_put(soc);
	return ret;
}

/* Like the T6030 handoff, before of_platform_default_populate(). */
arch_initcall(t8132_platform_handoff);
