// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/apple-centauri.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/mfd/macsmc.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/pci.h>
#include <linux/pci-apple.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>

#include "centauri_t6050.h"

/* Registry owns one reference, registrations and queued work own additional ones. */
struct centauri_t6050_rewake {
	struct list_head node;
	struct kref ref;
	struct work_struct work;
	struct pci_dev *bridge;
	unsigned int slot;
	bool active, rerun;
	unsigned int generation, work_generation;
	int error;
	struct pci_dev *control;
	bool control_lease;
	struct centauri_t6050_ipc *ipc;
	void *firmware, *descriptor;
	dma_addr_t firmware_dma, descriptor_dma;
	size_t firmware_size;
	struct pci_dev *beta;
	const struct apple_centauri_sleep_ops *ops;
	void *data, *arena;
	dma_addr_t arena_dma;
	size_t arena_size;
	bool beta_lease;
};

static LIST_HEAD(cen_recovery_owners);
static DEFINE_MUTEX(cen_recovery_lock);
static struct workqueue_struct *cen_rewake_wq;
static bool cen_exiting;
static void centauri_t6050_rewake_worker(struct work_struct *work);
static int centauri_t6050_quiesce(struct centauri_t6050 *c);

static void centauri_t6050_owner_free(struct kref *ref)
{
	struct centauri_t6050_rewake *rw = container_of(ref, struct centauri_t6050_rewake, ref);

	pci_dev_put(rw->bridge);
	kfree(rw);
}

/* Called before address publication; never allocated by a recovery request. */
static struct centauri_t6050_rewake *centauri_t6050_owner_get(struct pci_dev *fn)
{
	struct centauri_t6050_rewake *rw;

	if (!fn->bus->self)
		return ERR_PTR(-ENODEV);
	guard(mutex)(&cen_recovery_lock);
	if (cen_exiting)
		return ERR_PTR(-ESHUTDOWN);
	list_for_each_entry(rw, &cen_recovery_owners, node) {
		if (rw->bridge == fn->bus->self && rw->slot == PCI_SLOT(fn->devfn)) {
			kref_get(&rw->ref);
			return rw;
		}
	}
	rw = kzalloc(sizeof(*rw), GFP_KERNEL);
	if (!rw)
		return ERR_PTR(-ENOMEM);
	kref_init(&rw->ref);
	INIT_WORK(&rw->work, centauri_t6050_rewake_worker);
	rw->bridge = pci_dev_get(fn->bus->self);
	rw->slot = PCI_SLOT(fn->devfn);
	list_add_tail(&rw->node, &cen_recovery_owners);
	kref_get(&rw->ref);
	return rw;
}

int apple_centauri_sleep_register(struct pci_dev *fn,
				  const struct apple_centauri_sleep_ops *ops, void *data,
				  void *arena, dma_addr_t dma, size_t size)
{
	struct centauri_t6050_rewake *rw;
	int ret;

	if (IS_ENABLED(CONFIG_ARCH_HAS_TEARDOWN_DMA_OPS))
		return -EOPNOTSUPP;
	if (PCI_FUNC(fn->devfn) != 2 || !ops || !ops->quiesce ||
	    !ops->release || !ops->retained || !arena || !size)
		return -EINVAL;
	rw = centauri_t6050_owner_get(fn);
	if (IS_ERR(rw))
		return PTR_ERR(rw);
	/* fn's device lock also serializes the worker and unregister. */
	if (rw->beta) {
		ret = -EBUSY;
		goto put;
	}
	ret = iommu_device_use_default_domain(&fn->dev);
	if (ret)
		goto put;
	rw->beta = pci_dev_get(fn);
	rw->beta_lease = true;
	rw->ops = ops;
	rw->data = data;
	rw->arena = arena;
	rw->arena_dma = dma;
	rw->arena_size = size;
	return 0;
put:
	kref_put(&rw->ref, centauri_t6050_owner_free);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_centauri_sleep_register);

void apple_centauri_sleep_unregister(struct pci_dev *fn)
{
	struct centauri_t6050_rewake *rw, *found = NULL;

	mutex_lock(&cen_recovery_lock);
	list_for_each_entry(rw, &cen_recovery_owners, node) {
		if (rw->bridge == fn->bus->self && rw->slot == PCI_SLOT(fn->devfn)) {
			found = rw;
			break;
		}
	}
	mutex_unlock(&cen_recovery_lock);
	if (!found || found->beta != fn || !found->ops)
		return;
	rw = found;
	if (!rw->ops->retained(rw->data)) {
		rw->arena = NULL;
		if (rw->beta_lease)
			iommu_device_unuse_default_domain(&fn->dev);
		rw->beta_lease = false;
		pci_dev_put(rw->beta);
		rw->beta = NULL;
	}
	/* No callback or devm data survives driver unbind. A retained tuple does. */
	rw->ops = NULL;
	rw->data = NULL;
	kref_put(&rw->ref, centauri_t6050_owner_free);
}
EXPORT_SYMBOL_GPL(apple_centauri_sleep_unregister);

void apple_centauri_sleep_recovery_needed(struct pci_dev *fn, int error)
{
	struct centauri_t6050_rewake *rw;

	guard(mutex)(&cen_recovery_lock);
	if (cen_exiting || !fn->bus->self)
		return;
	list_for_each_entry(rw, &cen_recovery_owners, node) {
		if (rw->bridge != fn->bus->self || rw->slot != PCI_SLOT(fn->devfn))
			continue;
		if (error && !rw->error)
			rw->error = error;
		/* One owner and one work reference even while the work is running. */
		if (rw->active && rw->generation != rw->work_generation)
			rw->rerun = true;
		if (!rw->active) {
			rw->active = true;
			rw->work_generation = rw->generation;
			kref_get(&rw->ref);
			queue_work(cen_rewake_wq, &rw->work);
		}
		return;
	}
}
EXPORT_SYMBOL_GPL(apple_centauri_sleep_recovery_needed);

static struct pci_driver centauri_t6050_driver;

bool apple_centauri_control_ready(struct pci_dev *pdev)
{
	struct centauri_t6050 *c;

	if (pdev->driver != &centauri_t6050_driver)
		return false;
	c = pci_get_drvdata(pdev);
	return c && !READ_ONCE(c->removing) && smp_load_acquire(&c->ready);
}
EXPORT_SYMBOL_GPL(apple_centauri_control_ready);

/* No DMA backing is released without verified bus-master shutdown. */
bool centauri_t6050_stop_dma(struct pci_dev *pdev)
{
	u16 command;
	int ret;

	pci_clear_master(pdev);
	ret = pci_read_config_word(pdev, PCI_COMMAND, &command);
	if (ret || (command & PCI_COMMAND_MASTER))
		return false;
	return pci_wait_for_pending_transaction(pdev);
}

void centauri_t6050_free_irqs(struct centauri_t6050_irqs *irqs)
{
	if (!irqs->pdev)
		return;
	while (irqs->count)
		free_irq(pci_irq_vector(irqs->pdev, --irqs->count), irqs->data);
	pci_free_irq_vectors(irqs->pdev);
	irqs->pdev = NULL;
}

int centauri_t6050_request_irqs(struct centauri_t6050_irqs *irqs, struct pci_dev *pdev,
			 unsigned int count, irq_handler_t handler, void *data)
{
	int ret;

	ret = pci_alloc_irq_vectors(pdev, count, count, PCI_IRQ_MSI);
	if (ret < 0)
		return ret;
	irqs->pdev = pdev;
	irqs->data = data;
	for (irqs->count = 0; irqs->count < count; irqs->count++) {
		ret = request_irq(pci_irq_vector(pdev, irqs->count), handler, 0,
				  "centauri_t6050", data);
		if (ret) {
			centauri_t6050_free_irqs(irqs);
			return ret;
		}
	}
	return 0;
}

static irqreturn_t centauri_t6050_control_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static void centauri_t6050_free_boot(struct centauri_t6050 *c)
{
	struct device *dev = &c->pdev->dev;

	if (!c->link_ready && c->boot_attempted) {
		while (c->alpha_irqs.count)
			free_irq(pci_irq_vector(c->alpha_irqs.pdev, --c->alpha_irqs.count), c);
		while (c->irqs.count)
			free_irq(pci_irq_vector(c->pdev, --c->irqs.count), c);
		dev_warn(dev, "retaining DMA buffers after failed link transition; reboot required\n");
		return;
	}

	/* Already released for a deep sleep: the function may be off the bus by now */
	if (c->firmware_sleep && !c->ipc && !c->irqs.pdev && !c->firmware && !c->descriptor)
		return;
	centauri_t6050_free_ipc(c);
	centauri_t6050_free_irqs(&c->irqs);
	if (c->ipc || !centauri_t6050_stop_dma(c->pdev)) {
		dev_warn(dev, "retaining boot DMA until outstanding transactions stop\n");
		return;
	}
	if (c->firmware)
		dma_free_coherent(dev, c->firmware_size, c->firmware, c->firmware_dma);
	if (c->descriptor)
		dma_free_coherent(dev, SZ_16K, c->descriptor, c->descriptor_dma);
	c->firmware = NULL;
	c->descriptor = NULL;
}

static int centauri_t6050_rom_boot(struct centauri_t6050 *c)
{
	const struct firmware *fw;
	struct device *dev = &c->pdev->dev;
	u32 stage;
	int ret;

	if (c->pdev->device != 0x1900 && c->pdev->device != 0x1901)
		return -ENODEV;
	if (!c->link_ready || c->boot_attempted)
		return -EBUSY;
	stage = readl(c->bar + 0x8000);
	if (stage != 0)
		return -EBUSY;

	ret = request_firmware_direct(&fw, c->firmware_name, dev);
	if (ret) {
		dev_err(dev, "required firmware %s unavailable: %d\n", c->firmware_name, ret);
		return ret;
	}
	if (READ_ONCE(c->removing)) {
		ret = -ECANCELED;
		goto release;
	}
	if (fw->size < 0x30 || fw->size > SZ_64M ||
	    memcmp(fw->data + 0x20, "rkosftab", 8)) {
		ret = -EINVAL;
		goto release;
	}
	c->firmware_size = PAGE_ALIGN(fw->size);
	c->firmware = dma_alloc_coherent(dev, c->firmware_size, &c->firmware_dma, GFP_KERNEL);
	c->descriptor = dma_alloc_coherent(dev, SZ_16K, &c->descriptor_dma, GFP_KERNEL);
	if (!c->firmware || !c->descriptor) {
		ret = -ENOMEM;
		goto free_buffers;
	}
	memset(c->firmware, 0, c->firmware_size);
	memcpy(c->firmware, fw->data, fw->size);
	memset(c->descriptor, 0, SZ_16K);

	put_unaligned_le64(c->firmware_dma, c->descriptor);
	put_unaligned_le32(ALIGN(fw->size, SZ_4K), c->descriptor + 8);
	ret = centauri_t6050_request_irqs(&c->irqs, c->pdev, 4,
				    centauri_t6050_control_irq, c);
	if (ret)
		goto free_buffers;
	pci_set_master(c->pdev);
	writel(c->platform_id, c->bar + 0x807c);
	writel(0, c->bar + 0x803c);
	writel(lower_32_bits(c->descriptor_dma), c->bar + 0x8044);
	writel(upper_32_bits(c->descriptor_dma), c->bar + 0x8048);
	writel(0x20, c->bar + 0x804c);
	dma_wmb();
	c->boot_attempted = true;
	writel(1, c->bar + 0x9000);
	ret = readl_poll_timeout(c->bar + 0x8040, c->image_response,
				READ_ONCE(c->removing) || c->image_response != 0, 20000, 20000000);
	if (READ_ONCE(c->removing)) {
		ret = -ECANCELED;
		goto release;
	}
	if (!ret && c->image_response != 1)
		ret = -EIO;
	if (!ret)
		ret = readl_poll_timeout(c->bar + 0x8000, stage,
				READ_ONCE(c->removing) || stage != 0, 20000, 20000000);
	else
		stage = readl(c->bar + 0x8000);
	if (READ_ONCE(c->removing))
		ret = -ECANCELED;
	dma_rmb();

	if (!ret && stage != 4)
		ret = -EIO;
	goto release;
free_buffers:
	centauri_t6050_free_boot(c);
release:
	release_firmware(fw);
	return ret;
}

/*
 * After the ROM boot the N1 runs its firmware from host memory and must come back on a
 * retrained link: the isolated function 0 is quiesced, put in D3hot with PME enabled
 * (the host's PME_Turn_Off then takes the link to L2), the host resets and retrains the
 * root port (PERST# asserted and released around it; the firmware keeps running), and the
 * root port's and the endpoint's state are restored.
 */
static int centauri_t6050_port_cycle(struct pci_dev *endpoint)
{
	struct pci_host_bridge *host = pci_find_host_bridge(endpoint->bus);
	struct pci_dev *bridge = endpoint->bus->self, *sibling;
	u16 command, pmctrl;
	u32 identity;
	int ret, pm;

	if (!host->reset_root_port || !bridge || !pci_is_root_bus(bridge->bus))
		return -EOPNOTSUPP;
	pm = endpoint->pm_cap;
	if (!pm)
		return -ENODEV;
	if (!pci_trylock_rescan_remove())
		return -EAGAIN;
	list_for_each_entry(sibling, &endpoint->bus->devices, bus_list) {
		if (sibling != endpoint) {
			ret = -EBUSY;
			goto out;
		}
	}
	ret = pci_save_state(bridge) ?: pci_save_state(endpoint);
	if (ret)
		goto out;
	ret = pcibios_err_to_errno(pci_read_config_word(endpoint, PCI_COMMAND, &command));
	if (ret)
		goto out;
	if (!centauri_t6050_stop_dma(endpoint)) {
		ret = -EBUSY;
		goto stopped;
	}
	ret = pcibios_err_to_errno(pci_read_config_word(endpoint, pm + PCI_PM_CTRL, &pmctrl));
	if (ret)
		goto stopped;
	ret = pcibios_err_to_errno(pci_write_config_word(endpoint, pm + PCI_PM_CTRL,
		(pmctrl & ~PCI_PM_CTRL_STATE_MASK) | PCI_D3hot | PCI_PM_CTRL_PME_ENABLE));
	if (ret)
		goto stopped;
	usleep_range(10000, 11000);

	ret = host->reset_root_port(host, bridge);
	if (ret)
		goto stopped;
	pci_restore_state(bridge);
	ret = pcibios_err_to_errno(pci_read_config_dword(endpoint, PCI_VENDOR_ID, &identity));
	if (!ret && identity != 0x1900106b && identity != 0x1901106b)
		ret = -ENODEV;
	if (ret)
		goto stopped;
	pci_restore_state(endpoint);
	ret = pcibios_err_to_errno(pci_write_config_word(endpoint, pm + PCI_PM_CTRL,
		pmctrl & ~PCI_PM_CTRL_STATE_MASK));
	if (ret)
		goto stopped;
	usleep_range(10000, 11000);
	ret = pcibios_err_to_errno(pci_write_config_word(endpoint, PCI_COMMAND, command));
	if (!ret)
		goto out;
stopped:
	/* A partial transition never restores endpoint DMA or bridge decoding. */
	pci_clear_master(endpoint);
	pci_write_config_word(bridge, PCI_COMMAND, 0);
	pci_err(endpoint, "N1 port cycle stopped: %d\n", ret);
out:
	pci_unlock_rescan_remove();
	return ret;
}

static int centauri_t6050_cycle(struct centauri_t6050 *c)
{
	int ret;

	if (!c->firmware || !c->link_ready || readl(c->bar + 0x8000) != 4)
		return -EBUSY;
	c->link_ready = false;
	ret = centauri_t6050_port_cycle(c->pdev);
	if (ret == -EAGAIN) {
		c->link_ready = true;
	} else if (!ret) {
		pci_restore_msi_state(c->pdev);
		c->link_ready = true;
	}
	return ret;
}

static void centauri_t6050_boot_worker(struct work_struct *work)
{
	struct centauri_t6050 *c = container_of(to_delayed_work(work), struct centauri_t6050, boot_work);
	bool retry = false;
	int ret;

	mutex_lock(&c->boot_lock);
	while (!READ_ONCE(c->removing)) {
		switch (c->boot_phase) {
		case CEN_BOOT_ROM:
			ret = centauri_t6050_rom_boot(c);
			if (!ret)
				c->boot_phase = readl(c->bar + 0x8000) == 1 ?
					CEN_BOOT_IPC : CEN_BOOT_CYCLE;
			break;
		case CEN_BOOT_CYCLE:
			ret = centauri_t6050_cycle(c);
			if (!ret)
				c->boot_phase = CEN_BOOT_IPC;
			break;
		case CEN_BOOT_IPC:
			ret = centauri_t6050_start_ipc(c);
			if (!ret)
				c->boot_phase = CEN_BOOT_READY;
			break;
		default:
			goto out;
		}
		if (ret == -EAGAIN && (c->boot_phase == CEN_BOOT_CYCLE ||
		    (c->boot_phase == CEN_BOOT_IPC && !c->ipc))) {
			retry = true;
			break;
		}
		if (ret) {
			dev_err(&c->pdev->dev, "N1 automatic startup failed in phase%u: %d\n",
				c->boot_phase, ret);
			c->boot_phase = CEN_BOOT_FAILED;
			break;
		}
	}
out:
	if (READ_ONCE(c->removing))
		c->boot_phase = CEN_BOOT_STOPPED;
	mutex_unlock(&c->boot_lock);
	if (retry && !READ_ONCE(c->removing))
		queue_delayed_work(system_long_wq, &c->boot_work, msecs_to_jiffies(20));
}

static int centauri_t6050_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct centauri_t6050 *c;
	int ret;
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);

	if (!bridge || !bridge->dev.parent ||
	    !of_device_is_compatible(bridge->dev.parent->of_node, "apple,t6050-pcie"))
		return -ENODEV;
	if (PCI_FUNC(pdev->devfn) != 0)
		return -ENODEV;
	/* Retention across driver unbind requires the DMA ops to survive it. */
	if (IS_ENABLED(CONFIG_ARCH_HAS_TEARDOWN_DMA_OPS))
		return -EOPNOTSUPP;
	c = devm_kzalloc(&pdev->dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->pdev = pdev;
	c->link_ready = true;
	mutex_init(&c->boot_lock);
	INIT_DELAYED_WORK(&c->boot_work, centauri_t6050_boot_worker);
	c->boot_phase = CEN_BOOT_ROM;
	ret = device_property_read_string(&pdev->dev, "firmware-name", &c->firmware_name);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "missing personalized firmware name\n");
	ret = device_property_read_u32(&pdev->dev, "apple,centauri-platform-id", &c->platform_id);
	if (ret)
		return ret;
	ret = device_property_read_u32(&pdev->dev, "apple,centauri-protocol", &c->protocol);
	if (ret || c->protocol != 27)
		return -EINVAL;
	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;
	c->size = pci_resource_len(pdev, 0);
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM) ||
	    c->size < 0x9080)
		return dev_err_probe(&pdev->dev, -ENODEV, "invalid ACIPC BAR0\n");
	ret = pcim_iomap_regions(pdev, BIT(0), "apple_centauri_t6050");
	if (ret)
		return ret;
	c->bar = pcim_iomap_table(pdev)[0];
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "40-bit DMA unavailable\n");

	c->recovery = centauri_t6050_owner_get(pdev);
	if (IS_ERR(c->recovery))
		return PTR_ERR(c->recovery);
	if (c->recovery->control) {
		kref_put(&c->recovery->ref, centauri_t6050_owner_free);
		return -EBUSY;
	}
	ret = iommu_device_use_default_domain(&pdev->dev);
	if (ret) {
		kref_put(&c->recovery->ref, centauri_t6050_owner_free);
		return ret;
	}
	c->recovery->control = pci_dev_get(pdev);
	c->recovery->control_lease = true;
	mutex_lock(&cen_recovery_lock);
	c->recovery->generation++;
	mutex_unlock(&cen_recovery_lock);
	pci_clear_master(pdev);
	pci_set_drvdata(pdev, c);
	queue_delayed_work(system_long_wq, &c->boot_work, 0);
	return 0;
}

static void centauri_t6050_remove(struct pci_dev *pdev)
{
	struct centauri_t6050 *c = pci_get_drvdata(pdev);
	struct centauri_t6050_rewake *rw = c->recovery;
	int ret;

	ret = centauri_t6050_quiesce(c);
	/* Independent ownership before devm c is destroyed, including failed fences. */
	centauri_t6050_detach_ipc(c->ipc);
	rw->ipc = c->ipc;
	rw->firmware = c->firmware;
	rw->firmware_dma = c->firmware_dma;
	rw->firmware_size = c->firmware_size;
	rw->descriptor = c->descriptor;
	rw->descriptor_dma = c->descriptor_dma;
	c->ipc = NULL;
	c->firmware = NULL;
	c->descriptor = NULL;
	if (!rw->ipc && !rw->firmware && !rw->descriptor) {
		iommu_device_unuse_default_domain(&pdev->dev);
		rw->control_lease = false;
		pci_dev_put(rw->control);
		rw->control = NULL;
	}
	if (ret)
		apple_centauri_sleep_recovery_needed(pdev, ret);
	kref_put(&rw->ref, centauri_t6050_owner_free);
}

/*
 * A firmware sleep takes the N1's power state with it, and its firmware does not survive one:
 * the N1 is reset after the wake and boots from its ROM again, as at power-on.
 * - Before the sleep, Bluetooth tears itself down in its own suspend, then this function
 *   tears Wi-Fi and the IPC/boot resources down. DMA buffers are retained if DMA shutdown
 *   cannot be verified. No firmware sleep handover is implemented. The root port's
 *   "apple,endpoint-sleep-smc-writes", if any, are written after that.
 * - Neither function arms a wake for the sleep: Wi-Fi has no wake-on-WLAN, and Bluetooth's HCI
 *   device is gone. Request a replay of the N1 warm-reset sequence after link shutdown to
 *   avoid leaving its firmware running without its host. Selective WoWLAN and paired-device
 *   Bluetooth wake require a firmware handover that is not implemented here.
 * - The host gives the warm reset in its noirq resume, while the port is still down (see
 *   pcie-apple). This resume writes the port's "apple,endpoint-wake-smc-writes", if any; once
 *   tasks run again, a work item removes the three functions and enumerates them again.
 * - After a suspend that was abandoned before the host's shutdown the link never went down. The
 *   work item then has the host take the port down and up around the reset.
 * Only when the root port names the warm reset ("apple,function-warm-reset") and the SMC is
 * bound. Each SMC write is four cells: the key, the eight value bytes as two cells, each from
 * its most significant byte down, and the wait after the write in microseconds.
 */
#define CEN_SMC_WRITES_MAX 4

/*
 * The root port's "apple,function-warm-reset" names the SMC: the SMC, if bound (a reference on
 * its device in *smc_pdev).
 */
static struct apple_smc *centauri_t6050_smc(struct pci_dev *bridge,
					    struct platform_device **smc_pdev)
{
	struct of_phandle_args fn;
	struct apple_smc *smc;

	*smc_pdev = NULL;
	if (!IS_REACHABLE(CONFIG_MFD_MACSMC))
		return NULL;
	if (!bridge || of_parse_phandle_with_fixed_args(pci_device_to_OF_node(bridge),
							"apple,function-warm-reset", 2, 0, &fn))
		return NULL;
	*smc_pdev = of_find_device_by_node(fn.np);
	of_node_put(fn.np);
	if (!*smc_pdev)
		return NULL;
	smc = platform_get_drvdata(*smc_pdev);
	if (!smc) {
		put_device(&(*smc_pdev)->dev);
		*smc_pdev = NULL;
		return NULL;
	}
	return smc;
}

/* The root port's list @name of SMC writes, in order; no list is no error. */
static int centauri_t6050_smc_writes(struct pci_dev *bridge, const char *name)
{
	struct device_node *np = bridge ? pci_device_to_OF_node(bridge) : NULL;
	u32 cells[4 * CEN_SMC_WRITES_MAX];
	struct platform_device *smc_pdev;
	struct apple_smc *smc;
	int count, i, ret = 0;

	count = np ? of_property_count_u32_elems(np, name) : -EINVAL;
	if (count == -EINVAL)
		return 0;
	if (count <= 0 || count % 4 || count > ARRAY_SIZE(cells) ||
	    of_property_read_u32_array(np, name, cells, count))
		return -EINVAL;
	smc = centauri_t6050_smc(bridge, &smc_pdev);
	if (!smc)
		return -ENODEV;
	for (i = 0; i < count && !ret; i += 4) {
		__be32 value[2] = { cpu_to_be32(cells[i + 1]), cpu_to_be32(cells[i + 2]) };

		ret = apple_smc_write(smc, cells[i], value, sizeof(value));
		if (!ret && cells[i + 3])
			fsleep(cells[i + 3]);
	}
	put_device(&smc_pdev->dev);
	return ret;
}

bool apple_centauri_sleep_reboot(struct pci_dev *fn)
{
	struct platform_device *smc_pdev;

	if (!pm_suspend_via_firmware() || !fn->bus->self)
		return false;
	if (!centauri_t6050_smc(fn->bus->self, &smc_pdev))
		return false;
	put_device(&smc_pdev->dev);
	return true;
}
EXPORT_SYMBOL_GPL(apple_centauri_sleep_reboot);

static bool centauri_t6050_link_active(struct pci_dev *bridge)
{
	u16 lnksta;

	return !pcie_capability_read_word(bridge, PCI_EXP_LNKSTA, &lnksta) &&
	       (lnksta & PCI_EXP_LNKSTA_DLLLA);
}

/*
 * A suspend given up after Bluetooth's teardown and before this function's own suspend leaves
 * function 0 as it ran: Wi-Fi registered, and its IPC holding interrupt vectors and DMA buffers
 * of functions 1 and 2. Removed in that state, function 1 goes with those vectors still
 * allocated, and function 0's removal then frees them on a device that no longer has any: a
 * NULL dereference under the rescan lock, which the PCI host's resume notifier then waits for
 * without end. So function 0 is taken down here first, as its suspend takes it down, while all
 * three functions exist; its removal then finds nothing left, as after a completed suspend.
 */
/* Stop producers while Beta is still bound, then fence the complete DMA domain. */
static int centauri_t6050_beta_call(struct centauri_t6050_rewake *rw, bool release)
{
	struct pci_dev *fn = pci_get_slot(rw->bridge->subordinate, PCI_DEVFN(rw->slot, 2));
	const struct apple_centauri_sleep_ops *ops;
	int ret = 0;

	if (!fn)
		return rw->arena ? -ENODEV : 0;
	device_lock(&fn->dev);
	if (rw->beta && rw->beta != fn) {
		ret = -ENODEV;
		goto unlock;
	}
	ops = rw->ops;
	if (ops) {
		if (!try_module_get(ops->owner)) {
			ret = -ENODEV;
		} else {
			ret = release ? ops->release(rw->data) : ops->quiesce(rw->data);
			if (release && !ops->retained(rw->data))
				rw->arena = NULL;
			module_put(ops->owner);
		}
	} else if (release && rw->arena && rw->beta == fn) {
		/* Original DMA setup is held by the explicit default-domain lease. */
		memzero_explicit(rw->arena, rw->arena_size);
		dma_free_coherent(&fn->dev, rw->arena_size, rw->arena, rw->arena_dma);
		rw->arena = NULL;
		iommu_device_unuse_default_domain(&fn->dev);
		rw->beta_lease = false;
		pci_dev_put(rw->beta);
		rw->beta = NULL;
	}
unlock:
	device_unlock(&fn->dev);
	pci_dev_put(fn);
	return ret;
}

static bool centauri_t6050_domain_fence(struct centauri_t6050_rewake *rw)
{
	struct pci_dev *fn[3];
	bool stopped = true;
	int i;

	for (i = 0; i < 3; i++) {
		fn[i] = pci_get_slot(rw->bridge->subordinate, PCI_DEVFN(rw->slot, i));
		if ((i == 0 && rw->control && fn[i] != rw->control) ||
		    (i == 2 && rw->beta && fn[i] != rw->beta))
			stopped = false;
		if (fn[i])
			pci_clear_master(fn[i]);
	}
	for (i = 0; i < 3; i++) {
		if (fn[i] && !centauri_t6050_stop_dma(fn[i]))
			stopped = false;
		pci_dev_put(fn[i]);
	}
	return stopped;
}

static int centauri_t6050_quiesce(struct centauri_t6050 *c)
{
	int ret;

	WRITE_ONCE(c->removing, true);
	/* Pairs with control_ready: no new consumer can attach. */
	smp_store_release(&c->ready, false);
	cancel_delayed_work_sync(&c->boot_work);
	/* Never hold boot_lock over an HCI close or Bluetooth work cancellation. */
	ret = centauri_t6050_beta_call(c->recovery, false);
	mutex_lock(&c->boot_lock);
	centauri_t6050_wifi_unregister(c);
	centauri_t6050_free_irqs(&c->alpha_irqs);
	centauri_t6050_free_irqs(&c->irqs);
	c->firmware_sleep = true;
	c->boot_phase = CEN_BOOT_STOPPED;
	/*
	 * A completed teardown already released every published DMA tuple. After
	 * the host resets N1, the old Alpha/Beta PCI devices can still exist in
	 * software while only Control is visible in ROM. Do not fence those
	 * retired functions again unless there is backing left to release.
	 */
	if (!ret && (c->ipc || c->firmware || c->descriptor || c->recovery->arena) &&
	    (!centauri_t6050_domain_fence(c->recovery) ||
	     !centauri_t6050_ipc_fenced(c->ipc)))
		ret = -EBUSY;
	mutex_unlock(&c->boot_lock);
	if (!ret)
		ret = centauri_t6050_beta_call(c->recovery, true);
	if (!ret) {
		mutex_lock(&c->boot_lock);
		centauri_t6050_free_boot(c);
		if (c->ipc || c->firmware || c->descriptor)
			ret = -EBUSY;
		mutex_unlock(&c->boot_lock);
	}
	return ret;
}

static int centauri_t6050_rewake_teardown(struct centauri_t6050_rewake *rw)
{
	struct pci_dev *fn = pci_get_slot(rw->bridge->subordinate, PCI_DEVFN(rw->slot, 0));
	struct centauri_t6050 *c;
	int ret = 0;

	if (!fn)
		return -ENODEV;
	/*
	 * sysfs unbind does not take the rescan lock. Keep the original F0
	 * locked through tuple fencing and release: .remove transfers these
	 * same tuples before its devm data and DMA setup can disappear.
	 */
	device_lock(&fn->dev);
	if (rw->control && rw->control != fn) {
		ret = -ENODEV;
		goto out;
	}
	c = fn->dev.driver == &centauri_t6050_driver.driver ? pci_get_drvdata(fn) : NULL;
	if (c)
		ret = centauri_t6050_quiesce(c);
	/* F0 may have unbound already; the independent owner holds its resources. */
	if (!ret)
		ret = centauri_t6050_beta_call(rw, false);
	/* The independent owner needs a fence only for tuples it still holds. */
	if (!ret && (rw->ipc || rw->firmware || rw->descriptor || rw->arena) &&
	    (!centauri_t6050_domain_fence(rw) ||
	     !centauri_t6050_ipc_fenced(rw->ipc)))
		ret = -EBUSY;
	if (!ret)
		ret = centauri_t6050_beta_call(rw, true);
	if (ret)
		goto out;
	centauri_t6050_release_ipc(rw->ipc);
	rw->ipc = NULL;
	if (rw->control) {
		if (rw->firmware)
			dma_free_coherent(&fn->dev, rw->firmware_size,
					  rw->firmware, rw->firmware_dma);
		if (rw->descriptor)
			dma_free_coherent(&fn->dev, SZ_16K,
					  rw->descriptor, rw->descriptor_dma);
		rw->firmware = NULL;
		rw->descriptor = NULL;
		/* A still-bound control registration keeps its lease until remove. */
		if (fn->dev.driver != &centauri_t6050_driver.driver) {
			iommu_device_unuse_default_domain(&fn->dev);
			rw->control_lease = false;
			pci_dev_put(rw->control);
			rw->control = NULL;
		}
	}
	ret = rw->arena ? -EBUSY : 0;
out:
	device_unlock(&fn->dev);
	pci_dev_put(fn);
	return ret;
}

/*
 * The functions are removed while their link is still up, then the warm reset, then the link
 * (if it went down, until it is back), then a new enumeration, all under the rescan lock: no
 * device exists below the bridge meanwhile, so nothing reads one while a link is down (a read
 * then latches pcie-apple's link_failed).
 */
static void centauri_t6050_rewake_worker(struct work_struct *work)
{
	struct centauri_t6050_rewake *rw = container_of(work, struct centauri_t6050_rewake, work);
	struct pci_dev *bridge = rw->bridge, *fn;
	struct pci_bus *bus = bridge->subordinate;
	s64 down_ms = -1, up_ms = -1, wait_ms;
	ktime_t begin, start, up_at;
	u16 lnksta = 0, first = 0;
	int i, ret;

	if (!bus)
		goto out;
	/*
	 * The bridge stays active to the new enumeration: with its functions removed it is
	 * idle, and a runtime suspend that starts while the host has the port down for the
	 * reset finds the bridge unreachable and leaves the link down after it.
	 */
	pm_runtime_get_sync(&bridge->dev);
	pci_lock_rescan_remove();
	ret = centauri_t6050_rewake_teardown(rw);
	if (ret) {
		pci_err(bridge, "N1 recovery refused: DMA resources retained (%d, original %d)\n",
			ret, READ_ONCE(rw->error));
		goto unlock;
	}
	for (i = 2; i >= 0; i--) {
		fn = pci_get_slot(bus, PCI_DEVFN(rw->slot, i));
		if (fn)
			pci_stop_and_remove_bus_device(fn);
		pci_dev_put(fn);
	}
	/*
	 * The host resets the N1 in its noirq resume, while the port is still down. A reset given
	 * here, on a trained link, would take the link down for good: after a suspend that kept the
	 * link the host takes the port down and up around the reset instead.
	 */
	if (apple_pcie_t6050_warm_reset_done(bridge)) {
		pci_dbg(bridge, "N1 after the sleep: reset by the host before its port came up\n");
		ret = 0;
	} else {
		ret = apple_pcie_t6050_port_recycle(bridge);
		if (!ret)
			pci_dbg(bridge, "N1 after a suspend that kept its link: port cycled by the host for the reset\n");
	}
	if (ret) {
		pci_err(bridge, "N1 after the sleep: no warm reset (%d); Wi-Fi and Bluetooth stay down\n",
			ret);
		goto unlock;
	}
	/*
	 * The watch's times are from the host's bring-up of the port: in the wake's noirq resume
	 * the link trained long before this work runs, once tasks run again. A host that can't
	 * say: from now.
	 */
	begin = ktime_get();
	start = apple_pcie_t6050_link_up_time(bridge) ?: begin;
	up_at = start;
	pcie_capability_read_word(bridge, PCI_EXP_LNKSTA, &first);
	/*
	 * The link is normally up and stays up: the reset came before it trained. A link that
	 * still goes down in the first second after it trained must train again by itself. It is
	 * looked at once at least, however late this work runs.
	 */
	for (;;) {
		if (!centauri_t6050_link_active(bridge)) {
			down_ms = ktime_ms_delta(ktime_get(), start);
			break;
		}
		if (ktime_ms_delta(ktime_get(), start) >= 1000)
			break;
		usleep_range(1000, 2000);
	}
	if (down_ms >= 0) {
		while (!centauri_t6050_link_active(bridge)) {
			/* 10 s in all from this work's start: a later suspend's freezer waits for it */
			if (ktime_ms_delta(ktime_get(), begin) > 10000) {
				pcie_capability_read_word(bridge, PCI_EXP_LNKSTA, &lnksta);
				pci_err(bridge, "N1 after the warm reset: LNKSTA %#x then down at %lld ms, not back in 10 s (LNKSTA %#x)\n",
					first, down_ms, lnksta);
				goto unlock;
			}
			usleep_range(5000, 6000);
		}
		up_ms = ktime_ms_delta(ktime_get(), start);
		up_at = ktime_get();
	}
	/* The PCIe wait from link up to configuration requests (the ADT's perst-to-config): its rest */
	wait_ms = 100 - ktime_ms_delta(ktime_get(), up_at);
	if (wait_ms > 0)
		msleep(wait_ms);
	pcie_capability_read_word(bridge, PCI_EXP_LNKSTA, &lnksta);
	pci_rescan_bus(bus);
	fn = pci_get_slot(bus, PCI_DEVFN(rw->slot, 0));
	if (fn)
		pci_dbg(bridge, "N1 reset after the sleep: LNKSTA %#x, link down at %lld ms, up at %lld ms (-1: not seen), LNKSTA %#x; enumerated again: %s\n",
			first, down_ms, up_ms, lnksta, pci_name(fn));
	else
		pci_err(bridge, "N1 reset after the sleep: nothing enumerated (LNKSTA %#x); Wi-Fi and Bluetooth stay down\n",
			lnksta);
	pci_dev_put(fn);
unlock:
	pci_unlock_rescan_remove();
	pm_runtime_put(&bridge->dev);
out:
	mutex_lock(&cen_recovery_lock);
	if (rw->rerun && !cen_exiting) {
		rw->rerun = false;
		rw->work_generation = rw->generation;
		/* Requeue transfers this work reference to the next generation. */
		queue_work(cen_rewake_wq, &rw->work);
		mutex_unlock(&cen_recovery_lock);
		return;
	}
	rw->active = false;
	rw->error = 0;
	mutex_unlock(&cen_recovery_lock);
	kref_put(&rw->ref, centauri_t6050_owner_free);
}

static void centauri_t6050_queue_rewake(struct pci_dev *fn)
{
	apple_centauri_sleep_recovery_needed(fn, 0);
}

/* Bluetooth's resume: a suspend stopped after Bluetooth's teardown, before this function's */
void apple_centauri_sleep_rewake(struct pci_dev *fn)
{
	centauri_t6050_queue_rewake(fn);
}
EXPORT_SYMBOL_GPL(apple_centauri_sleep_rewake);

static int centauri_t6050_prepare(struct device *dev)
{
	struct centauri_t6050 *c = pci_get_drvdata(to_pci_dev(dev));

	if (!apple_centauri_sleep_reboot(c->pdev))
		return 0;
	/* No destructive progress unless the independent owner already exists. */
	if (!c->recovery)
		return -ENODEV;
	mutex_lock(&c->boot_lock);
	centauri_t6050_wifi_pm_prepare(c);
	c->pm_prepared = true;
	mutex_unlock(&c->boot_lock);
	return 0;
}

static void centauri_t6050_complete(struct device *dev)
{
	struct centauri_t6050 *c = pci_get_drvdata(to_pci_dev(dev));
	int error;

	mutex_lock(&c->boot_lock);
	if (!c->pm_prepared) {
		mutex_unlock(&c->boot_lock);
		return;
	}
	error = centauri_t6050_wifi_pm_error(c);
	centauri_t6050_wifi_pm_complete(c);
	c->pm_prepared = false;
	mutex_unlock(&c->boot_lock);
	/* Complete also runs if cfg80211 refused suspend before F0 suspend ran. */
	if (error || c->firmware_sleep)
		apple_centauri_sleep_recovery_needed(c->pdev, error);
}

static int centauri_t6050_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct centauri_t6050 *c = pci_get_drvdata(pdev);
	int ret;

	if (!apple_centauri_sleep_reboot(pdev))
		return c->boot_phase == CEN_BOOT_READY ? pci_save_state(pdev) : -EBUSY;
	ret = centauri_t6050_quiesce(c);
	if (ret)
		goto recover;
	ret = centauri_t6050_smc_writes(pdev->bus->self, "apple,endpoint-sleep-smc-writes");
	if (ret)
		goto recover;
	apple_pcie_t6050_sleep_reset(pdev->bus->self);
	ret = pci_save_state(pdev);
	if (!ret)
		return 0;
recover:
	apple_centauri_sleep_recovery_needed(pdev, ret);
	return ret;
}

static int centauri_t6050_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct centauri_t6050 *c = pci_get_drvdata(pdev);
	int ret;

	if (!c->firmware_sleep)
		return 0;
	ret = centauri_t6050_smc_writes(pdev->bus->self, "apple,endpoint-wake-smc-writes");
	if (ret)
		dev_warn(dev, "N1 after the sleep: SMC writes: %d\n", ret);
	/* The removal, the warm reset and the new enumeration run once tasks run again */
	centauri_t6050_queue_rewake(pdev);
	return 0;
}

static const struct dev_pm_ops centauri_t6050_pm_ops = {
	.prepare = centauri_t6050_prepare,
	.complete = centauri_t6050_complete,
	SYSTEM_SLEEP_PM_OPS(centauri_t6050_suspend, centauri_t6050_resume)
};

static const struct pci_device_id centauri_t6050_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x1900) },
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x1901) },
	{ }
};
MODULE_DEVICE_TABLE(pci, centauri_t6050_ids);

static struct pci_driver centauri_t6050_driver = {
	.name = "apple_centauri_t6050",
	.id_table = centauri_t6050_ids,
	.probe = centauri_t6050_probe,
	.remove = centauri_t6050_remove,
	.shutdown = centauri_t6050_remove,
	.driver.pm = pm_sleep_ptr(&centauri_t6050_pm_ops),
};

static int __init centauri_t6050_init(void)
{
	int ret;

	cen_rewake_wq = alloc_ordered_workqueue("centauri_t6050_rewake", WQ_FREEZABLE);
	if (!cen_rewake_wq)
		return -ENOMEM;
	ret = pci_register_driver(&centauri_t6050_driver);
	if (ret)
		destroy_workqueue(cen_rewake_wq);
	return ret;
}
module_init(centauri_t6050_init);

static void __exit centauri_t6050_exit(void)
{
	struct centauri_t6050_rewake *rw, *next;

	mutex_lock(&cen_recovery_lock);
	cen_exiting = true;
	mutex_unlock(&cen_recovery_lock);
	/* Drain before unregister: no worker can race callback data destruction. */
	flush_workqueue(cen_rewake_wq);
	pci_unregister_driver(&centauri_t6050_driver);
	/* Drains a pending rewake before the module's code goes */
	destroy_workqueue(cen_rewake_wq);
	list_for_each_entry_safe(rw, next, &cen_recovery_owners, node) {
		list_del(&rw->node);
		/* Retained DMA must keep its device/domain ownership even at unload. */
		if (rw->arena || rw->ipc || rw->firmware || rw->descriptor) {
			pci_warn(rw->bridge, "N1 unload retaining unfenced DMA; reboot required\n");
			continue;
		}
		kref_put(&rw->ref, centauri_t6050_owner_free);
	}
}
module_exit(centauri_t6050_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Apple N1 Centauri PCI firmware and ACIPC transport");
