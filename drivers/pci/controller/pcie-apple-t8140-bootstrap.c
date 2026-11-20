// SPDX-License-Identifier: GPL-2.0-only
/*
 * T8140 radio PCIe bootstrap: one validated config read before native ECAM.
 *
 * The Linux DMA/IOMMU APIs own the SID17 mappings. See
 * Documentation/PCI/controller/apple-t8140.rst for the public history and
 * the retained-ownership contract. Completion does not establish DMA teardown.
 * The arena, IRQ and power reference remain owned until full hardware reset.
 */
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/kexec.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/soc/apple/pci-apple-piodma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#include "pcie-apple-t8140-bootstrap.h"
#include "pcie-apple-t8140-bootstrap-state.h"

#define PIODMA_SID		17
#define PIODMA_REQUEST_COUNT	1
#define PIODMA_SLOT_SIZE	(3 * PAGE_SIZE)
#define PIODMA_ARENA_SIZE	(PIODMA_REQUEST_COUNT * PIODMA_SLOT_SIZE)
#define PIODMA_DATA_OFFSET	(2 * PAGE_SIZE)
#define PIODMA_COMPLETION_OFFSET	PAGE_SIZE
#define PIODMA_CONFIG_BASE	0x1cb0000000ULL
#define PIODMA_CONFIG_SIZE	SZ_256M
#define PIODMA_IRQ_MASK		GENMASK(9, 0)
#define PIODMA_FIFO_MASK		GENMASK(5, 0)

/*
 * The board DT opts in by pointing the host's "apple,piodma" at this engine;
 * a host without that property stays root-port-only.
 */

struct apple_piodma_request {
	u16 offset;
	u8 function;
	u32 mask;
	u32 expected;
};

static const struct apple_piodma_request requests[PIODMA_REQUEST_COUNT] = {
	{ PCI_VENDOR_ID, 0, U32_MAX, 0x793214c3 },
};

struct apple_piodma_slot {
	struct completion event;
	__le32 packet[4];
	__le32 completion[4];
	u32 result;
	u32 matched_status;
	bool submitted;
	bool observed;
};

struct apple_t8140_bootstrap {
	struct device *dev;
	struct iommu_domain *domain;
	void __iomem *engine;
	void __iomem *fabric;
	struct apple_piodma_slot slots[PIODMA_REQUEST_COUNT];
	atomic_t irq_count;
	atomic_t state;
	u32 active_tag;
	u64 config_source;
	int irq;
	bool irq_fault;
	void *arena;
	dma_addr_t dma;
	u64 pointer_prefix;
	u8 secondary_bus;
	struct notifier_block pm_notifier;
	struct kexec_blocker kexec_blocker;
	struct delayed_work admission_retry;
	struct device *retry_host;
	/* The host stops the radios for a sleep that takes this engine's power. */
	bool sleep_armed;
	bool power_lost;
	/* The engine has answered once since boot; later bootstraps follow a lost power. */
	bool answered;
	/* After a real sleep: read the engine once the host's port is up again. */
	bool power_check;
	bool irq_held;
	int (*radio_stop)(struct pci_dev *pdev);
	struct module *radio_stop_owner;
	void (*radio_recover)(void *data);
	void *radio_recover_data;
	/* Done once the host has the radios back after a sleep, or gave up. */
	struct completion radio_ready;
};

/* A successful supplier probe owns its unpublished arena until full reset. */
static struct apple_t8140_bootstrap *active_bootstrap;

/* Host device_lock serializes transitions; IRQ faults are recorded separately. */
static enum apple_piodma_state apple_piodma_state(struct apple_t8140_bootstrap *bootstrap)
{
	return atomic_read_acquire(&bootstrap->state);
}

static int apple_piodma_transition(struct apple_t8140_bootstrap *bootstrap,
				   enum apple_piodma_event event)
{
	enum apple_piodma_state state = apple_piodma_state(bootstrap);
	int next = apple_piodma_next(state, event);

	if (next < 0)
		return next;
	return atomic_cmpxchg(&bootstrap->state, state, next) == state ? 0 : -EALREADY;
}

static bool apple_piodma_trylock_sleep(unsigned int *flags)
{
	*flags = 0;
	if (!IS_ENABLED(CONFIG_PM_SLEEP))
		return true;

	/*
	 * Host probe holds device_lock(), while system sleep holds the transition
	 * mutex before waiting for probes and taking device locks. Never wait
	 * for that mutex here: if sleep won, defer without publishing any DMA.
	 * If admission won, publish its veto before releasing the mutex.
	 */
	*flags = current->flags;
	current->flags |= PF_NOFREEZE;
	if (mutex_trylock(&system_transition_mutex))
		return true;
	if (!(*flags & PF_NOFREEZE))
		current->flags &= ~PF_NOFREEZE;
	return false;
}

static int apple_piodma_lock_transition(struct apple_t8140_bootstrap *bootstrap,
					unsigned int *sleep_flags)
{
	unsigned int attempt;
	int ret;

	/* Probe cannot wait on either lock while holding its device lock. */
	for (attempt = 0; attempt < 50; attempt++) {
		if (apple_piodma_trylock_sleep(sleep_flags)) {
			ret = kexec_block(&bootstrap->kexec_blocker);
			if (!ret)
				return 0;
			unlock_system_sleep(*sleep_flags);
			if (ret != -EBUSY)
				return ret;
		}
		if (attempt != 49)
			msleep(20);
	}
	/* A short-lived loader may not trigger deferred-probe retries. */
	return -EPROBE_DEFER;
}

static int apple_piodma_pm_notify(struct notifier_block *nb,
				  unsigned long action, void *unused)
{
	struct apple_t8140_bootstrap *bootstrap =
		container_of(nb, struct apple_t8140_bootstrap, pm_notifier);

	switch (action) {
	case PM_SUSPEND_PREPARE:
		/*
		 * Nothing is vetoed here: apple_piodma_pm_prepare() makes
		 * the check in the device prepare phase, which a suspend
		 * through /dev/snapshot reaches too.
		 */
		return NOTIFY_DONE;
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
		dev_warn_ratelimited(bootstrap->dev,
				     "sleep refused: Neo guard active during radio admission or retained DMA lifetime\n");
		return notifier_from_errno(-EOPNOTSUPP);
	default:
		return NOTIFY_DONE;
	}
}

static int apple_piodma_pm_prepare(struct device *dev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(dev);

	/* Acquire admission before either PCI children or DMA pointers escape. */
	if (!bootstrap || !apple_piodma_guarded(apple_piodma_state(bootstrap)))
		return 0;
	if (apple_piodma_sleep_allowed(apple_piodma_state(bootstrap), pm_suspend_no_platform()))
		return 0;
	if (READ_ONCE(bootstrap->sleep_armed))
		return 0;
	/*
	 * A /dev/snapshot file may have sent its PREPARE notifier before this
	 * supplier probed. Its later image/restore/suspend ioctls go straight
	 * to device PM, so retain a veto in the device prepare phase as well.
	 */
	dev_warn_ratelimited(dev,
			     "sleep refused: Neo guard active during radio admission or retained DMA lifetime\n");
	return -EOPNOTSUPP;
}

static int apple_t8140_bootstrap_root(struct apple_t8140_bootstrap *bootstrap, struct pci_dev *root)
{
	int ret;

	if (!pci_is_root_bus(root->bus) || pci_domain_nr(root->bus) ||
	    root->bus->number || root->devfn ||
	    root->vendor != PCI_VENDOR_ID_APPLE || root->device != 0x100c ||
	    root->class != PCI_CLASS_BRIDGE_PCI << 8)
		return -ENODEV;
	ret = pci_read_config_byte(root, PCI_SECONDARY_BUS, &bootstrap->secondary_bus);
	if (ret)
		return pcibios_err_to_errno(ret);
	if (bootstrap->secondary_bus != 1)
		return -EINVAL;
	bootstrap->config_source = PIODMA_CONFIG_BASE + ((u64)bootstrap->secondary_bus << 20);
	return 0;
}

static int apple_t8140_bootstrap_iommu(struct apple_t8140_bootstrap *bootstrap)
{
	struct of_phandle_args args;
	u64 range[2];
	int ret;

	ret = of_parse_phandle_with_args(bootstrap->dev->of_node, "iommus",
					 "#iommu-cells", 0, &args);
	if (ret)
		return ret;
	if (args.args_count != 1 || args.args[0] != PIODMA_SID ||
	    !of_device_is_compatible(args.np, "apple,t8140-dart")) {
		ret = -EINVAL;
		goto out;
	}
	/* Keep the prior pointer convention with low IAS36 page tables. */
	if (of_property_present(args.np, "apple,dma-offset")) {
		ret = -EINVAL;
		goto out;
	}
	bootstrap->pointer_prefix = PIODMA_POINTER_PREFIX;
	ret = of_property_read_u64_array(args.np, "apple,dma-range", range, 2);
	if (ret)
		goto out;
	if (bootstrap->pointer_prefix != PIODMA_POINTER_PREFIX || range[0] ||
	    range[1] != BIT_ULL(36)) {
		ret = -EINVAL;
		goto out;
	}
	bootstrap->domain = iommu_get_domain_for_dev(bootstrap->dev);
	if (!bootstrap->domain ||
	    (bootstrap->domain->type != IOMMU_DOMAIN_DMA &&
	     bootstrap->domain->type != IOMMU_DOMAIN_DMA_FQ)) {
		ret = -ENODEV;
		goto out;
	}
	ret = dma_set_mask_and_coherent(bootstrap->dev, PIODMA_PACKET_MASK);
out:
	of_node_put(args.np);
	return ret;
}

static int apple_t8140_bootstrap_arena(struct apple_t8140_bootstrap *bootstrap)
{
	int i;

	bootstrap->arena = dma_alloc_coherent(bootstrap->dev, PIODMA_ARENA_SIZE,
					      &bootstrap->dma, GFP_KERNEL);
	if (!bootstrap->arena)
		return -ENOMEM;
	if (!apple_piodma_arena_valid(bootstrap->dma, PIODMA_ARENA_SIZE,
				      bootstrap->pointer_prefix,
				      bootstrap->domain->geometry.force_aperture,
				      bootstrap->domain->geometry.aperture_start,
				      bootstrap->domain->geometry.aperture_end))
		return -ERANGE;
	memset(bootstrap->arena, 0, PIODMA_ARENA_SIZE);
	for (i = 0; i < PIODMA_REQUEST_COUNT; i++) {
		void *slot = bootstrap->arena + i * PIODMA_SLOT_SIZE;
		u64 dma = bootstrap->dma + i * PIODMA_SLOT_SIZE;

		memset(slot + PIODMA_DATA_OFFSET, 0xa5, PAGE_SIZE);
		*(__le32 *)(slot + PIODMA_DATA_OFFSET) = cpu_to_le32(U32_MAX);
		dev_dbg(bootstrap->dev,
			"owned SID17 slot=%d dma=%#llx descriptor=%#llx completion=%#llx data=%#llx\n",
			i, dma, dma + bootstrap->pointer_prefix,
			dma + bootstrap->pointer_prefix + PIODMA_COMPLETION_OFFSET,
			dma + PIODMA_DATA_OFFSET);
	}
	dma_wmb();
	return 0;
}


static int apple_t8140_bootstrap_resources(struct platform_device *pdev,
					   struct apple_t8140_bootstrap *bootstrap)
{
	static const struct {
		const char *name;
		resource_size_t base;
	} regions[] = {
		{ "engine", 0x390030000ULL },
		{ "irq-aggregator", 0x390034000ULL },
		{ "fabric", 0x390038000ULL },
	};
	struct device_node *host;
	struct resource ecam, *res;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(regions); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, regions[i].name);
		if (!res || res->start != regions[i].base || resource_size(res) != SZ_16K)
			return -EINVAL;
	}
	host = of_parse_phandle(bootstrap->dev->of_node, "apple,pcie-host", 0);
	if (!host)
		return -EINVAL;
	ret = of_address_to_resource(host, 0, &ecam);
	if (!ret && (!of_device_is_compatible(host, "apple,t8140-pcie") ||
		     ecam.start != PIODMA_CONFIG_BASE ||
		     resource_size(&ecam) != PIODMA_CONFIG_SIZE))
		ret = -EINVAL;
	of_node_put(host);
	if (ret)
		return ret;
	bootstrap->config_source = ecam.start;
	if (bootstrap->config_source + (1 << 12) + sizeof(u32) - 1 > ecam.end)
		return -ERANGE;
	bootstrap->engine = devm_platform_ioremap_resource_byname(pdev, "engine");
	if (IS_ERR(bootstrap->engine))
		return PTR_ERR(bootstrap->engine);
	bootstrap->fabric = devm_platform_ioremap_resource_byname(pdev, "fabric");
	if (IS_ERR(bootstrap->fabric))
		return PTR_ERR(bootstrap->fabric);
	bootstrap->irq = platform_get_irq(pdev, 0);
	return bootstrap->irq < 0 ? bootstrap->irq : 0;
}

static int apple_t8140_bootstrap_idle(struct apple_t8140_bootstrap *bootstrap)
{
	static const struct {
		u32 offset;
		u32 expected;
	} engine[] = {
		{ 0x00, 2 }, { 0x04, 0 }, { 0x08, 0 }, { 0x0c, 16 },
		{ 0x44, 0 }, { 0x48, 0 }, { 0x4c, 0 }, { 0x50, 0 },
		{ 0x54, 0 }, { 0xd8, 0 }, { 0xdc, 0 },
	};
	int i;
	u32 value;

	for (i = 0; i < ARRAY_SIZE(engine); i++) {
		value = readl(bootstrap->engine + engine[i].offset);
		if (value != engine[i].expected) {
			dev_err(bootstrap->dev, "idle mismatch engine[%#x]=%#x expected=%#x\n",
				engine[i].offset, value, engine[i].expected);
			return -EBUSY;
		}
	}
	if (readl(bootstrap->fabric) || readl(bootstrap->fabric + 4))
		return -EBUSY;
	return 0;
}

static irqreturn_t apple_t8140_bootstrap_irq(int irq, void *data)
{
	struct apple_t8140_bootstrap *bootstrap = data;
	struct apple_piodma_slot *slot;
	u32 fabric, status, tag, active;
	int count;

	/* After a real sleep its block can't answer until the host's port is back. */
	if (READ_ONCE(bootstrap->irq_held))
		return IRQ_NONE;

	/* Acknowledge order follows the attributed prior controller contract. */
	fabric = readl(bootstrap->fabric);
	writel(fabric, bootstrap->fabric);
	status = readl(bootstrap->engine + 4);
	writel(status, bootstrap->engine + 4);
	tag = readl(bootstrap->engine + 0xdc) & 0xff;
	active = READ_ONCE(bootstrap->active_tag);
	count = atomic_inc_return(&bootstrap->irq_count);
	dev_dbg(bootstrap->dev, "irq=%d fabric=%#x status=%#x tag=%#x active=%#x\n",
		count, fabric, status, tag, active);
	if (apple_piodma_irq_invalid(count, status, tag, active)) {
		WRITE_ONCE(bootstrap->irq_fault, true);
		disable_irq_nosync(irq);
		if (active)
			complete(&bootstrap->slots[active - 1].event);
	} else if (apple_piodma_irq_matches(status, tag, active)) {
		slot = &bootstrap->slots[active - 1];
		WRITE_ONCE(slot->matched_status, READ_ONCE(slot->matched_status) | status);
		complete(&slot->event);
	}
	/* A late old-tag IRQ never wakes or qualifies the current request. */
	return IRQ_HANDLED;
}

static int apple_t8140_bootstrap_initialize(struct apple_t8140_bootstrap *bootstrap)
{
	static const struct {
		u32 offset;
		u32 mask;
		u32 value;
	} tunables[] = {
		{ 0x00, 0x2, 0 },
		{ 0x10, 0x1fffffff, 0x1fbf2 },
		{ 0x14, 0x1fffffff, 0x1fbf2 },
		{ 0x18, 0x1fffffff, 0x1fbf3 },
		{ 0x1c, 0x7ffff, 0x1fbf0 },
		{ 0x20, 0x7ffff, 0x1fbf0 },
	};
	int i;
	u32 value;

	writel(PIODMA_IRQ_MASK, bootstrap->engine + 8);
	writel(0, bootstrap->fabric + 4);
	for (i = 0; i < ARRAY_SIZE(tunables); i++) {
		value = readl(bootstrap->engine + tunables[i].offset);
		value = (value & ~tunables[i].mask) | tunables[i].value;
		writel(value, bootstrap->engine + tunables[i].offset);
	}
	writel(readl(bootstrap->engine) & ~BIT(0), bootstrap->engine);
	/* Exact prior bank values; their architectural meaning is unqualified. */
	for (i = 0; i < 8; i++) {
		writel(0, bootstrap->engine + 0x24 + 4 * i);
		writel(i == 1 ? 1 : 0, bootstrap->engine + 0xf0 + 4 * i);
	}
	/* Order the bank writes before their configuration readback. */
	wmb();
	if (readl(bootstrap->engine) || readl(bootstrap->engine + 8) != PIODMA_IRQ_MASK ||
	    readl(bootstrap->engine + 0xf4) != 1 || READ_ONCE(bootstrap->irq_fault))
		return -EIO;
	return 0;
}

static bool apple_t8140_bootstrap_canary(const u8 *memory, size_t size, u8 value)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (READ_ONCE(memory[i]) != value)
			return false;
	return true;
}

static int apple_t8140_bootstrap_check_slots(struct apple_t8140_bootstrap *bootstrap)
{
	int i, j;

	dma_rmb();
	for (i = 0; i < PIODMA_REQUEST_COUNT; i++) {
		struct apple_piodma_slot *slot = &bootstrap->slots[i];
		void *memory = bootstrap->arena + i * PIODMA_SLOT_SIZE;
		__le32 *packet = memory;
		__le32 *completion = memory + PIODMA_COMPLETION_OFFSET;
		__le32 *result = memory + PIODMA_DATA_OFFSET;

		for (j = 0; j < ARRAY_SIZE(slot->packet); j++) {
			if (READ_ONCE(packet[j]) != slot->packet[j] ||
			    (slot->observed && READ_ONCE(completion[j]) != slot->completion[j]) ||
			    (!slot->submitted && READ_ONCE(completion[j])))
				goto changed;
		}
		if ((slot->observed && le32_to_cpu(READ_ONCE(*result)) != slot->result) ||
		    (!slot->submitted && le32_to_cpu(READ_ONCE(*result)) != U32_MAX) ||
		    !apple_t8140_bootstrap_canary(memory + sizeof(slot->packet),
					     PAGE_SIZE - sizeof(slot->packet), 0) ||
		    !apple_t8140_bootstrap_canary(memory + PIODMA_COMPLETION_OFFSET +
					     sizeof(slot->completion),
					     PAGE_SIZE - sizeof(slot->completion), 0) ||
		    !apple_t8140_bootstrap_canary(memory + PIODMA_DATA_OFFSET + sizeof(u32),
					     PAGE_SIZE - sizeof(u32), 0xa5))
			goto changed;
	}
	return 0;

changed:
	dev_err(bootstrap->dev, "slot=%d changed outside its permitted response; stop\n", i);
	return -EIO;
}

static int apple_t8140_bootstrap_submit(struct apple_t8140_bootstrap *bootstrap, unsigned int index)
{
	const struct apple_piodma_request *request = &requests[index];
	struct apple_piodma_slot *slot = &bootstrap->slots[index];
	void *memory = bootstrap->arena + index * PIODMA_SLOT_SIZE;
	__le32 *packet = memory;
	__le32 *result = memory + PIODMA_DATA_OFFSET;
	__le32 *completion = memory + PIODMA_COMPLETION_OFFSET;
	u64 dma = bootstrap->dma + index * PIODMA_SLOT_SIZE;
	u64 source = bootstrap->config_source + ((u64)request->function << 12) + request->offset;
	u64 dst = dma + PIODMA_DATA_OFFSET;
	u64 src = source & PIODMA_PACKET_MASK;
	u64 descriptor = dma + bootstrap->pointer_prefix;
	u64 comp = descriptor + PIODMA_COMPLETION_OFFSET;
	unsigned long waited;
	u32 fifo, command, tag = index + 1;
	int i, checks, ret;

	ret = apple_t8140_bootstrap_check_slots(bootstrap);
	if (ret || READ_ONCE(bootstrap->irq_fault))
		return ret ?: -EIO;
	/* Only these fixed, validated ECAM addresses use aperture1/low36. */
	slot->packet[0] = cpu_to_le32(0xc007 | BIT(18) | ((src & 0xff) << 24));
	slot->packet[1] = cpu_to_le32((src >> 8) | ((dst & 0xf) << 28));
	slot->packet[2] = cpu_to_le32(dst >> 4);
	slot->packet[3] = cpu_to_le32(sizeof(u32));
	memcpy(packet, slot->packet, sizeof(slot->packet));
	dma_wmb();
	ret = readl_poll_timeout(bootstrap->engine + 0x0c, fifo,
				 (fifo & PIODMA_FIFO_MASK) == 16, 10, 50000);
	if (ret || READ_ONCE(bootstrap->irq_fault))
		return ret ?: -EIO;
	slot->submitted = true;
	WRITE_ONCE(bootstrap->active_tag, tag);
	dev_dbg(bootstrap->dev,
		"request=%u function=%u offset=%#x source=%#llx packet=%08x:%08x:%08x:%08x tag=%u\n",
		index, request->function, request->offset, source,
		le32_to_cpu(packet[0]), le32_to_cpu(packet[1]),
		le32_to_cpu(packet[2]), le32_to_cpu(packet[3]), tag);
	writel(tag, bootstrap->engine + 0xd8);
	writel(upper_32_bits(descriptor) & 0x3ff, bootstrap->engine + 0x50);
	writel(lower_32_bits(descriptor), bootstrap->engine + 0x4c);
	writel(upper_32_bits(comp) & 0x3ff, bootstrap->engine + 0x48);
	writel(lower_32_bits(comp), bootstrap->engine + 0x44);
	writel(0x411, bootstrap->engine + 0x54);
	waited = wait_for_completion_timeout(&slot->event, msecs_to_jiffies(50));
	dma_rmb();
	slot->result = le32_to_cpu(READ_ONCE(*result));
	for (i = 0; i < ARRAY_SIZE(slot->completion); i++)
		slot->completion[i] = READ_ONCE(completion[i]);
	slot->observed = true;
	dev_dbg(bootstrap->dev,
		"response=%u waited=%lu matched_status=%#x word=%#x completion=%08x:%08x:%08x:%08x\n",
		index, waited, READ_ONCE(slot->matched_status), slot->result,
		le32_to_cpu(slot->completion[0]), le32_to_cpu(slot->completion[1]),
		le32_to_cpu(slot->completion[2]), le32_to_cpu(slot->completion[3]));
	if (!waited)
		return -ETIMEDOUT;
	if (READ_ONCE(bootstrap->irq_fault) || !READ_ONCE(slot->matched_status) ||
	    slot->result == U32_MAX ||
	    (slot->result & request->mask) != request->expected)
		return -EIO;
	/* Admission observations only: all published memory remains retained. */
	ret = readl_poll_timeout(bootstrap->engine + 0x54, command, !(command & BIT(0)),
				 10, 50000);
	if (ret)
		return ret;
	ret = readl_poll_timeout(bootstrap->engine + 0x0c, fifo,
				 (fifo & PIODMA_FIFO_MASK) == 16, 10, 50000);
	if (ret)
		return ret;
	/*
	 * The slots must hold still after the response. After a sleep, the same engine has
	 * answered the same request before, and the slots are read again before every sleep.
	 */
	checks = bootstrap->answered ? 2 : 10;
	for (i = 0; i < checks; i++) {
		usleep_range(1000, 2000);
		ret = apple_t8140_bootstrap_check_slots(bootstrap);
		if (ret || READ_ONCE(bootstrap->irq_fault))
			return ret ?: -EIO;
	}
	bootstrap->answered = true;
	dev_dbg(bootstrap->dev,
		"READ_VALIDATED request=%u word=%#x FIFO=%#x command=%#x; retained slots stable\n",
		index, slot->result, fifo, command);
	return 0;
}

static void apple_piodma_put_supplier(void *data)
{
	put_device(data);
}

int apple_piodma_bootstrap_get(struct device *host, struct device **supplier)
{
	struct device_node *node, *host_node;
	struct apple_t8140_bootstrap *bootstrap;
	struct platform_device *pdev;
	struct device_link *link;
	bool correct_host;
	int ret;

	if (!of_device_is_compatible(host->of_node, "apple,t8140-pcie"))
		return -ENODEV;
	node = of_parse_phandle(host->of_node, "apple,piodma", 0);
	if (!node)
		return -EINVAL;
	if (!of_device_is_compatible(node, "apple,t8140-pcie-bootstrap") &&
	    !of_device_is_compatible(node, "apple,t8140-piodma-diagnostic")) {
		of_node_put(node);
		return -EINVAL;
	}
	host_node = of_parse_phandle(node, "apple,pcie-host", 0);
	correct_host = host_node == host->of_node;
	of_node_put(host_node);
	pdev = correct_host ? of_find_device_by_node(node) : NULL;
	of_node_put(node);
	if (!correct_host)
		return -EINVAL;
	if (!pdev)
		return -EPROBE_DEFER;
	bootstrap = platform_get_drvdata(pdev);
	if (!device_is_bound(&pdev->dev) || !bootstrap) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	/* Reject late software attachment before any Neo host hardware setup. */
	if (system_state > SYSTEM_RUNNING) {
		put_device(&pdev->dev);
		return -ESHUTDOWN;
	}
	/* Host probe holds its device lock; never rebuild a consumed hierarchy. */
	if (apple_piodma_attempted(apple_piodma_state(bootstrap))) {
		put_device(&pdev->dev);
		return -EALREADY;
	}
	link = device_link_add(host, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link) {
		put_device(&pdev->dev);
		return -EINVAL;
	}
	ret = devm_add_action_or_reset(host, apple_piodma_put_supplier, &pdev->dev);
	if (ret)
		return ret;
	*supplier = &pdev->dev;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_get);

static void apple_piodma_retry_admission(struct work_struct *work)
{
	struct apple_t8140_bootstrap *bootstrap =
		container_of(to_delayed_work(work), struct apple_t8140_bootstrap,
			     admission_retry);
	/* Pairs with the scheduler's cmpxchg of the boot-lifetime host reference. */
	struct device *host = smp_load_acquire(&bootstrap->retry_host);
	int ret;

	if (system_state > SYSTEM_RUNNING || !host ||
	    !device_is_registered(host) || apple_piodma_attempted(apple_piodma_state(bootstrap)) ||
	    apple_piodma_guarded(apple_piodma_state(bootstrap)))
		return;
	/* The core rechecks registration and binding under the host device lock. */
	ret = device_attach(host);
	if (ret < 0)
		dev_dbg(bootstrap->dev, "deferred host attachment returned %d\n", ret);
	/* Only fresh pre-child guard contention can schedule another attempt. */
}

static void apple_piodma_schedule_admission(struct apple_t8140_bootstrap *bootstrap,
					    struct device *host)
{
	struct device *previous;

	if (system_state > SYSTEM_RUNNING || !device_is_registered(host))
		return;
	/* Publish a boot-lifetime reference; this supplier has one matched host. */
	previous = cmpxchg(&bootstrap->retry_host, NULL, get_device(host));
	if (previous) {
		put_device(host);
		if (WARN_ON_ONCE(previous != host))
			return;
	}
	mod_delayed_work(system_dfl_wq, &bootstrap->admission_retry, HZ);
}

int apple_piodma_bootstrap_guard_begin(struct device *supplier, struct device *host)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);
	unsigned int sleep_flags;
	int ret;

	/* Reboot blocks core probing too; reject an attachment already in flight. */
	if (system_state > SYSTEM_RUNNING)
		return -ESHUTDOWN;
	/* A worker may have waited behind a host probe that consumed its attempt. */
	if (apple_piodma_attempted(apple_piodma_state(bootstrap)) ||
	    apple_piodma_guarded(apple_piodma_state(bootstrap)))
		return -EALREADY;
	/* This is the last dependency admission before PCI children exist. */
	ret = apple_piodma_lock_transition(bootstrap, &sleep_flags);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			apple_piodma_schedule_admission(bootstrap, host);
		return ret;
	}
	/* Do not wait for a worker that may be waiting on this host device lock. */
	cancel_delayed_work(&bootstrap->admission_retry);
	bootstrap->pm_notifier.notifier_call = apple_piodma_pm_notify;
	ret = register_pm_notifier(&bootstrap->pm_notifier);
	if (ret) {
		kexec_unblock(&bootstrap->kexec_blocker);
	} else {
		/* Publish both vetoes before permitting system sleep to proceed. */
		WARN_ON_ONCE(apple_piodma_transition(bootstrap, PIODMA_GUARD));
	}
	unlock_system_sleep(sleep_flags);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_guard_begin);

void apple_piodma_bootstrap_guard_end(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	/* Post-publication ownership is irreversible until hardware reset. */
	if (!apple_piodma_guarded(apple_piodma_state(bootstrap)) ||
	    apple_piodma_retained(apple_piodma_state(bootstrap)))
		return;
	/* The host has removed every child before permitting sleep again. */
	if (WARN_ON_ONCE(apple_piodma_transition(bootstrap, PIODMA_RELEASE)))
		return;
	unregister_pm_notifier(&bootstrap->pm_notifier);
	kexec_unblock(&bootstrap->kexec_blocker);
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_guard_end);

int apple_piodma_bootstrap_prime(struct device *supplier, struct pci_dev *root)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);
	int ret;

	/* Admission was completed before pci_host_common_init published children. */
	if (!apple_piodma_guarded(apple_piodma_state(bootstrap)))
		return -EINVAL;
	/* One hardware attempt, including permanent preflight failures. */
	if (apple_piodma_transition(bootstrap, PIODMA_PRIME))
		return -EALREADY;
	ret = apple_t8140_bootstrap_root(bootstrap, root);
	if (!ret)
		ret = apple_t8140_bootstrap_idle(bootstrap);
	if (!ret) {
		/* Ownership becomes irreversible before the first engine write. */
		ret = apple_piodma_transition(bootstrap, PIODMA_PUBLISH);
		if (!ret)
			ret = apple_t8140_bootstrap_initialize(bootstrap);
	}
	if (!ret)
		ret = apple_t8140_bootstrap_submit(bootstrap, 0);
	if (ret) {
		WARN_ON_ONCE(apple_piodma_transition(bootstrap, PIODMA_FAIL));
		dev_err(supplier, "bootstrap failed: %d; full hardware reset required\n", ret);
	} else {
		WARN_ON_ONCE(apple_piodma_transition(bootstrap, PIODMA_COMPLETE));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_prime);

void apple_piodma_bootstrap_set_sleep(struct device *supplier, bool armed)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	WRITE_ONCE(bootstrap->sleep_armed, armed);
	/* A check still pending means the block never came back, and stays unread. */
	if (armed)
		bootstrap->power_lost = false;
}

/* The next bootstrap starts as the first did, on the same retained memory. */
int apple_piodma_bootstrap_power_lost(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);
	void *arena = bootstrap->arena;
	int i;

	/*
	 * Everything reading as it did before the first command is what shows the sleep took
	 * the engine's pointers with it. Read only now: the block answers once the host's
	 * port is set up, after resume_noirq.
	 */
	if (bootstrap->power_check) {
		bootstrap->power_check = false;
		bootstrap->power_lost = !apple_t8140_bootstrap_idle(bootstrap);
		if (!bootstrap->power_lost)
			dev_err(supplier, "engine kept state across sleep; radios stay down\n");
		WRITE_ONCE(bootstrap->irq_held, false);
	}
	if (!bootstrap->power_lost || READ_ONCE(bootstrap->irq_fault))
		return -EIO;
	bootstrap->power_lost = false;

	memset(arena, 0, PIODMA_ARENA_SIZE);
	for (i = 0; i < PIODMA_REQUEST_COUNT; i++) {
		void *slot = arena + i * PIODMA_SLOT_SIZE;

		memset(slot + PIODMA_DATA_OFFSET, 0xa5, PAGE_SIZE);
		*(__le32 *)(slot + PIODMA_DATA_OFFSET) = cpu_to_le32(U32_MAX);
		memset(bootstrap->slots[i].packet, 0, sizeof(bootstrap->slots[i].packet));
		memset(bootstrap->slots[i].completion, 0, sizeof(bootstrap->slots[i].completion));
		bootstrap->slots[i].result = 0;
		WRITE_ONCE(bootstrap->slots[i].matched_status, 0);
		bootstrap->slots[i].submitted = false;
		bootstrap->slots[i].observed = false;
		reinit_completion(&bootstrap->slots[i].event);
	}
	dma_wmb();
	WRITE_ONCE(bootstrap->active_tag, 0);
	atomic_set(&bootstrap->irq_count, 0);

	return apple_piodma_transition(bootstrap, PIODMA_POWER_LOST);
}

/* The host brings the radios back after its resume_noirq; until then, see the resume below. */
void apple_piodma_bootstrap_radio_pending(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	reinit_completion(&bootstrap->radio_ready);
}

void apple_piodma_bootstrap_radio_done(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	complete_all(&bootstrap->radio_ready);
}

bool apple_piodma_bootstrap_retained(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	/* Acquire the prime's irreversible publication and guard ownership. */
	return apple_piodma_retained(apple_piodma_state(bootstrap));
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_retained);

/* The stop and the module it's in, set and read together. */
static DEFINE_SPINLOCK(radio_stop_lock);

int apple_piodma_bootstrap_radio_stop(struct device *supplier, struct pci_dev *pdev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);
	int (*stop)(struct pci_dev *pdev);
	struct module *owner;
	bool held;
	int ret;

	/* Its module stays while it runs. One that is unloading: its remove stops the radio. */
	spin_lock(&radio_stop_lock);
	stop = bootstrap->radio_stop;
	owner = bootstrap->radio_stop_owner;
	held = !stop || try_module_get(owner);
	spin_unlock(&radio_stop_lock);
	if (!held)
		return -EBUSY;
	if (!stop)
		return 0;

	ret = stop(pdev);
	module_put(owner);

	return ret;
}

void apple_piodma_bootstrap_set_recover(struct device *supplier, void (*recover)(void *data),
					void *data)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	bootstrap->radio_recover_data = data;
	WRITE_ONCE(bootstrap->radio_recover, recover);
}

void apple_piodma_bootstrap_admitted(struct device *supplier)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(supplier);

	if (WARN_ON_ONCE(!bootstrap || !apple_piodma_retained(apple_piodma_state(bootstrap))))
		return;
	/* Publish the host's completed cold, resource and ASPM admission checks. */
	WARN_ON_ONCE(apple_piodma_transition(bootstrap, PIODMA_ADMIT));
}
EXPORT_SYMBOL_GPL(apple_piodma_bootstrap_admitted);

int apple_piodma_radio_check(struct pci_dev *pdev)
{
	/* Pairs with probe's smp_store_release of the boot-lifetime supplier. */
	struct apple_t8140_bootstrap *bootstrap = smp_load_acquire(&active_bootstrap);
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	struct device_node *host;
	bool matches;

	if (!bootstrap)
		return -ENODEV;
	/* A retained supplier alone does not prove that host admission succeeded. */
	if (apple_piodma_state(bootstrap) != PIODMA_ADMITTED || !bridge->dev.parent ||
	    pdev->vendor != PCI_VENDOR_ID_MEDIATEK ||
	    pci_domain_nr(pdev->bus) || pdev->bus->number != bootstrap->secondary_bus ||
	    PCI_SLOT(pdev->devfn) ||
	    (pdev->devfn != PCI_DEVFN(0, 0) && pdev->devfn != PCI_DEVFN(0, 1)) ||
	    pdev->device != (PCI_FUNC(pdev->devfn) ? 0x793b : 0x7932))
		return -ENODEV;

	host = of_parse_phandle(bootstrap->dev->of_node, "apple,pcie-host", 0);
	matches = host && host == bridge->dev.parent->of_node;
	of_node_put(host);
	return matches ? 0 : -ENODEV;
}
EXPORT_SYMBOL_GPL(apple_piodma_radio_check);

void apple_piodma_radio_recover(void)
{
	/* Pairs with the release at registration. */
	struct apple_t8140_bootstrap *bootstrap = smp_load_acquire(&active_bootstrap);
	void (*recover)(void *data) = bootstrap ? READ_ONCE(bootstrap->radio_recover) : NULL;

	if (recover)
		recover(bootstrap->radio_recover_data);
}
EXPORT_SYMBOL_GPL(apple_piodma_radio_recover);

void __apple_piodma_radio_set_stop(int (*stop)(struct pci_dev *pdev), struct module *owner)
{
	/* Pairs with the release at registration. */
	struct apple_t8140_bootstrap *bootstrap = smp_load_acquire(&active_bootstrap);

	if (!bootstrap)
		return;
	spin_lock(&radio_stop_lock);
	bootstrap->radio_stop = stop;
	bootstrap->radio_stop_owner = owner;
	spin_unlock(&radio_stop_lock);
}
EXPORT_SYMBOL_GPL(__apple_piodma_radio_set_stop);

static int apple_t8140_bootstrap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_t8140_bootstrap *bootstrap;
	int i, ret;

	if (PAGE_SIZE != SZ_16K)
		return -ENODEV;
	bootstrap = kzalloc_obj(*bootstrap);
	if (!bootstrap)
		return -ENOMEM;
	bootstrap->dev = dev;
	INIT_DELAYED_WORK(&bootstrap->admission_retry, apple_piodma_retry_admission);
	bootstrap->kexec_blocker.reason =
		"Neo radio admission or retained DMA ownership";
	for (i = 0; i < PIODMA_REQUEST_COUNT; i++)
		init_completion(&bootstrap->slots[i].event);
	init_completion(&bootstrap->radio_ready);
	complete_all(&bootstrap->radio_ready);
	atomic_set(&bootstrap->irq_count, 0);
	atomic_set(&bootstrap->state, PIODMA_IDLE);
	ret = apple_t8140_bootstrap_iommu(bootstrap);
	if (ret)
		goto err_free;
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto err_pm;
	ret = apple_t8140_bootstrap_arena(bootstrap);
	if (ret)
		goto err_arena;
	ret = apple_t8140_bootstrap_resources(pdev, bootstrap);
	if (ret)
		goto err_arena;
	/* The host's resume_noirq waits for one completion, with device interrupts still off. */
	ret = devm_request_irq(dev, bootstrap->irq, apple_t8140_bootstrap_irq, IRQF_EARLY_RESUME,
			       dev_name(dev), bootstrap);
	if (ret)
		goto err_arena;
	/* The supplier and unpublished arena keep their device alive. */
	get_device(dev);
	platform_set_drvdata(pdev, bootstrap);
	/* Pairs with radio_check's smp_load_acquire of this boot-lifetime owner. */
	smp_store_release(&active_bootstrap, bootstrap);
	/* Its resume waits for the radios, with nothing that doesn't depend on them. */
	device_enable_async_suspend(dev);
	dev_dbg(dev, "bootstrap supplier ready; no command submitted during probe\n");
	return 0;

err_arena:
	if (bootstrap->arena)
		dma_free_coherent(dev, PIODMA_ARENA_SIZE, bootstrap->arena, bootstrap->dma);
	pm_runtime_put_sync(dev);
err_pm:
	pm_runtime_disable(dev);
err_free:
	kfree(bootstrap);
	return dev_err_probe(dev, ret, "PCIe bootstrap preflight failed\n");
}

static void apple_t8140_bootstrap_shutdown(struct platform_device *pdev)
{
	struct apple_t8140_bootstrap *bootstrap = platform_get_drvdata(pdev);

	/*
	 * Shutdown holds supplier/parent locks: never join a host attachment
	 * here. Admission rejects shutdown, and the supplier lifetime is retained.
	 */
	cancel_delayed_work(&bootstrap->admission_retry);
	if (apple_piodma_retained(apple_piodma_state(bootstrap)))
		dev_warn(&pdev->dev, "bootstrap arena retained; require full hardware reset\n");
}

/* No new request is permitted once the host has admitted its endpoints. */
static int apple_t8140_bootstrap_sleep_check(struct apple_t8140_bootstrap *bootstrap)
{
	if (!apple_piodma_ready(apple_piodma_state(bootstrap)) || READ_ONCE(bootstrap->irq_fault))
		return -EIO;
	if (readl(bootstrap->engine + 0x54) & BIT(0))
		return -EBUSY;
	if ((readl(bootstrap->engine + 0x0c) & PIODMA_FIFO_MASK) != 16)
		return -EBUSY;
	return apple_t8140_bootstrap_check_slots(bootstrap);
}

static int apple_t8140_bootstrap_suspend_noirq(struct device *dev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(dev);
	int ret;

	if (!apple_piodma_retained(apple_piodma_state(bootstrap)))
		return 0;
	if (pm_suspend_target_state != PM_SUSPEND_TO_IDLE && !READ_ONCE(bootstrap->sleep_armed))
		return -EOPNOTSUPP;
	/* Radios that failed to come back don't cost the machine a deep sleep. */
	if (READ_ONCE(bootstrap->sleep_armed) &&
	    (!apple_piodma_ready(apple_piodma_state(bootstrap)) || bootstrap->power_check))
		return 0;
	ret = apple_t8140_bootstrap_sleep_check(bootstrap);
	if (!ret && READ_ONCE(bootstrap->sleep_armed))
		WRITE_ONCE(bootstrap->irq_held, true);
	return ret;
}

static int apple_t8140_bootstrap_resume_noirq(struct device *dev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(dev);
	int ret;

	if (!apple_piodma_retained(apple_piodma_state(bootstrap)))
		return 0;
	if (!apple_piodma_ready(apple_piodma_state(bootstrap)) || READ_ONCE(bootstrap->irq_fault))
		return -EIO;
	/*
	 * Its block reads nothing until the host sets its port up again, so neither read it
	 * nor take its interrupt until then: apple_piodma_bootstrap_power_lost() checks it.
	 */
	if (READ_ONCE(bootstrap->sleep_armed) && pm_resume_via_firmware()) {
		bootstrap->power_check = true;
		return 0;
	}
	WRITE_ONCE(bootstrap->irq_held, false);
	/* Genpd may gate this island; restore configuration, never a command. */
	ret = apple_t8140_bootstrap_initialize(bootstrap);
	if (ret)
		return ret;
	return apple_t8140_bootstrap_sleep_check(bootstrap);
}

/*
 * The host is this device's consumer and the radios are below the host, so none of them
 * resumes before the host has the link and the functions back.
 */
static int apple_t8140_bootstrap_resume(struct device *dev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(dev);

	if (!wait_for_completion_timeout(&bootstrap->radio_ready, HZ))
		dev_warn(dev, "radios not back 1 s into the resume\n");
	return 0;
}

static int apple_t8140_bootstrap_freeze(struct device *dev)
{
	struct apple_t8140_bootstrap *bootstrap = dev_get_drvdata(dev);

	return apple_piodma_retained(apple_piodma_state(bootstrap)) ? -EOPNOTSUPP : 0;
}

static const struct dev_pm_ops apple_t8140_bootstrap_pm_ops = {
	.prepare = apple_piodma_pm_prepare,
	.suspend_noirq = apple_t8140_bootstrap_suspend_noirq,
	.resume_noirq = apple_t8140_bootstrap_resume_noirq,
	.resume = apple_t8140_bootstrap_resume,
	.freeze = apple_t8140_bootstrap_freeze,
	.poweroff = apple_t8140_bootstrap_freeze,
};

static const struct of_device_id apple_t8140_bootstrap_match[] = {
	{ .compatible = "apple,t8140-pcie-bootstrap" },
	{ .compatible = "apple,t8140-piodma-diagnostic" },
	{ }
};

static struct platform_driver apple_t8140_bootstrap_driver = {
	.probe = apple_t8140_bootstrap_probe,
	.shutdown = apple_t8140_bootstrap_shutdown,
	.driver = {
		.name = "apple-t8140-bootstrap",
		.of_match_table = apple_t8140_bootstrap_match,
		.pm = pm_sleep_ptr(&apple_t8140_bootstrap_pm_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_t8140_bootstrap_driver);
MODULE_DESCRIPTION("Apple T8140 radio PCIe bootstrap");
MODULE_LICENSE("GPL");
